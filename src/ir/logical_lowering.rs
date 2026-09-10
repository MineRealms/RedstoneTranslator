use std::collections::HashMap;

use eyre::{ContextCompat, WrapErr};

use super::logical::{
    ClockEdge, LogicalCell, LogicalCellKind, LogicalDesign, LogicalModule, LogicalPortDirection,
    LogicalValue,
};
use super::mapping::{
    bit_signal_name, classify_logical_net, combinational_output_module, composite_module,
    d_latch_routable_module, finish_design, instance_port, logical_ports, lower_flat_module,
    module_is_simple_state_design, not_clock_module, requires_general_flat_lowering,
    routable_leaf_from_graph, self_port, single_endpoint, NetConnections,
};
use super::target::{MappingPolicy, TargetSpec};
use super::{
    Endpoint, NetClass, RoutableDesign, RoutableModule, RoutableNet, RoutablePortDirection,
    ROUTABLE_IR_TARGET,
};
use crate::graph::logic::LogicGraph;
use crate::graph::{Graph, GraphNode, GraphNodeKind};
use crate::logic::{Logic, LogicType};
use crate::sequential::{SequentialPrimitive, SequentialType};

/// Lowers the typed logical netlist without reconstructing Verilog syntax.
///
pub(crate) fn lower_logical_to_routable(design: &LogicalDesign) -> eyre::Result<RoutableDesign> {
    lower_logical_to_routable_with_target(
        design,
        &TargetSpec::redstone_v1(),
        &MappingPolicy::default(),
    )
}

/// Lowers a logical design for one explicit target and mapping policy.
pub(crate) fn lower_logical_to_routable_with_target(
    design: &LogicalDesign,
    target: &TargetSpec,
    policy: &MappingPolicy,
) -> eyre::Result<RoutableDesign> {
    design.validate()?;
    policy.validate(target)?;
    let mut routable = lower_logical_design(design, target, policy).wrap_err_with(|| {
        format!(
            "logical IR uses a construct not yet supported by `{}` mapping",
            target.name()
        )
    })?;
    routable.debug = design.debug.clone();
    attach_routable_debug_locations(design, &mut routable);
    Ok(routable)
}

fn attach_routable_debug_locations(design: &LogicalDesign, routable: &mut RoutableDesign) {
    use super::debug::{logical_entity, routable_entity, DebugRelationKind};
    use super::routable::RoutableModuleBody;

    let Some(logical_top) = design.module(&design.top) else {
        return;
    };
    let increment = logical_top
        .cells
        .iter()
        .find(|cell| matches!(cell.kind, LogicalCellKind::Inc | LogicalCellKind::Add))
        .and_then(|cell| {
            design
                .debug
                .get(&logical_entity(&logical_top.name, "cell", &cell.name))
        });
    let state = logical_top
        .cells
        .iter()
        .find(|cell| cell.kind.is_sequential())
        .and_then(|cell| {
            design
                .debug
                .get(&logical_entity(&logical_top.name, "cell", &cell.name))
        });

    for logical_module in &design.modules {
        for instance in &logical_module.instances {
            routable.debug.relate(
                DebugRelationKind::Instantiates,
                logical_entity(&logical_module.name, "instance", &instance.name),
                logical_entity(&instance.module, "module", &instance.module),
            );
        }
    }

    let modules = routable.modules.clone();
    for module in modules {
        let logical_definition =
            logical_definition_for_routable_module(design, logical_top, &module.name);
        let module_parent = logical_definition
            .and_then(|definition| {
                design.debug.get(&logical_entity(
                    &definition.name,
                    "module",
                    &definition.name,
                ))
            })
            .or_else(|| location_for_generated_name(&module.name, increment, state));
        if let Some(parent) = module_parent {
            let entity = routable_entity(&module.name, "module", &module.name);
            let location = routable.debug.derived(entity.clone(), parent);
            routable.debug.bind(entity, location);
        }
        for port in &module.ports {
            let parent = logical_definition
                .and_then(|definition| {
                    definition
                        .ports
                        .iter()
                        .find(|candidate| candidate.name == port.name)
                        .and_then(|candidate| {
                            design.debug.get(&logical_entity(
                                &definition.name,
                                "net",
                                &candidate.net,
                            ))
                        })
                })
                .or(module_parent);
            let Some(parent) = parent else { continue };
            let entity = routable_entity(&module.name, "port", &port.name);
            let location = routable.debug.derived(entity.clone(), parent);
            routable.debug.bind(entity, location);
        }

        match module.body {
            RoutableModuleBody::Composite { instances, nets } => {
                for instance in instances {
                    let parent = location_for_generated_name(&instance.name, increment, state)
                        .or_else(|| {
                            design.debug.get(&logical_entity(
                                &logical_top.name,
                                "instance",
                                &instance.name,
                            ))
                        });
                    let Some(parent) = parent else { continue };
                    let entity = routable_entity(&module.name, "instance", &instance.name);
                    let location = routable.debug.derived(entity.clone(), parent);
                    routable.debug.bind(entity.clone(), location);
                    routable.debug.relate(
                        DebugRelationKind::Instantiates,
                        entity,
                        routable_entity(&instance.module, "module", &instance.module),
                    );
                }
                for net in nets {
                    let parent = logical_net_location_from_endpoints(design, logical_top, &net)
                        .or_else(|| logical_net_location(design, logical_top, &net.name))
                        .or_else(|| location_for_generated_name(&net.name, increment, state));
                    let Some(parent) = parent else { continue };
                    let entity = routable_entity(&module.name, "net", &net.name);
                    let location = routable.debug.derived(entity.clone(), parent);
                    routable.debug.bind(entity, location);
                }
            }
            RoutableModuleBody::Leaf { nodes } => {
                for node in nodes {
                    let parent = logical_definition
                        .and_then(|definition| logical_node_location(design, definition, &node))
                        .or_else(|| location_for_generated_name(&module.name, increment, state));
                    let Some(parent) = parent else { continue };
                    let entity = routable_entity(&module.name, "node", &node.id.to_string());
                    let location = routable.debug.derived(entity.clone(), parent);
                    routable.debug.bind(entity, location);
                }
            }
        }
    }
}

fn logical_definition_for_routable_module<'a>(
    design: &'a LogicalDesign,
    logical_top: &'a LogicalModule,
    routable_module: &str,
) -> Option<&'a LogicalModule> {
    if logical_top.name == routable_module {
        return Some(logical_top);
    }
    logical_top
        .instances
        .iter()
        .find(|instance| instance.name == routable_module)
        .and_then(|instance| design.module(&instance.module))
}

fn logical_node_location(
    design: &LogicalDesign,
    definition: &LogicalModule,
    node: &super::routable::RoutableNode,
) -> Option<usize> {
    use super::debug::logical_entity;
    use super::routable::RoutableNodeKind;

    let net_location = |name: &str| {
        let net = definition
            .ports
            .iter()
            .find(|port| port.name == name)
            .map_or(name, |port| port.net.as_str());
        design
            .debug
            .get(&logical_entity(&definition.name, "net", net))
    };

    match &node.kind {
        RoutableNodeKind::Input { name } | RoutableNodeKind::Output { name } => net_location(name),
        RoutableNodeKind::Constant { .. } => None,
        RoutableNodeKind::Not
        | RoutableNodeKind::And
        | RoutableNodeKind::Or
        | RoutableNodeKind::Xor => definition
            .cells
            .iter()
            .find(|cell| cell.name == node.tag)
            .and_then(|cell| {
                design
                    .debug
                    .get(&logical_entity(&definition.name, "cell", &cell.name))
            }),
        RoutableNodeKind::Sequential { .. } => definition
            .cells
            .iter()
            .find(|cell| cell.kind.is_sequential())
            .and_then(|cell| {
                design
                    .debug
                    .get(&logical_entity(&definition.name, "cell", &cell.name))
            }),
    }
}

fn logical_net_location_from_endpoints(
    design: &LogicalDesign,
    module: &LogicalModule,
    net: &super::routable::RoutableNet,
) -> Option<usize> {
    use super::routable::Endpoint;

    std::iter::once(&net.driver)
        .chain(net.sinks.iter())
        .find_map(|endpoint| {
            let logical_net = match endpoint {
                Endpoint::SelfPort { port } => module
                    .ports
                    .iter()
                    .find(|candidate| candidate.name == *port)
                    .map(|candidate| candidate.net.as_str()),
                Endpoint::InstancePort { instance, port } => module
                    .instances
                    .iter()
                    .find(|candidate| candidate.name == *instance)
                    .and_then(|candidate| {
                        candidate
                            .bindings
                            .iter()
                            .find(|binding| binding.port == *port)
                    })
                    .map(|binding| binding.net.as_str()),
            }?;
            logical_net_location(design, module, logical_net)
        })
}

fn location_for_generated_name(
    name: &str,
    increment: Option<usize>,
    state: Option<usize>,
) -> Option<usize> {
    if name.contains("_next") || name.contains("_carry") {
        increment
    } else if name.contains("_clk_inv") || name.contains("_master") || name.contains("_slave") {
        state
    } else {
        None
    }
}

fn logical_net_location(
    design: &LogicalDesign,
    module: &LogicalModule,
    routable_name: &str,
) -> Option<usize> {
    use super::debug::logical_entity;
    if let Some(location) = design
        .debug
        .get(&logical_entity(&module.name, "net", routable_name))
    {
        return Some(location);
    }
    let base = routable_name
        .rsplit_once('_')
        .filter(|(_, suffix)| suffix.chars().all(|ch| ch.is_ascii_digit()))
        .map_or(routable_name, |(base, _)| base);
    design.debug.get(&logical_entity(&module.name, "net", base))
}

fn lower_logical_design(
    design: &LogicalDesign,
    target: &TargetSpec,
    policy: &MappingPolicy,
) -> eyre::Result<RoutableDesign> {
    let definitions = design
        .modules
        .iter()
        .map(|module| (module.name.as_str(), module))
        .collect::<HashMap<_, _>>();
    let top = definitions
        .get(design.top.as_str())
        .copied()
        .with_context(|| format!("unknown logical top module `{}`", design.top))?;

    if top.instances.is_empty() {
        if let Some(state_design) = lower_state_design(top)? {
            return Ok(state_design);
        }
        if requires_general_flat_lowering(top) {
            return lower_flat_module(top, target, policy);
        }
        return match graph_backed_module(top, &top.name) {
            Ok(module) => finish_design(&top.name, target.name(), vec![module]),
            Err(scalar_error) => lower_flat_module(top, target, policy).wrap_err_with(|| {
                format!("legacy scalar leaf lowering failed first: {scalar_error}")
            }),
        };
    }

    if !top.cells.is_empty() {
        eyre::bail!(
            "mixed logical cells and instances are not supported in module `{}` yet",
            top.name
        );
    }

    let mut modules = Vec::new();
    for instance in &top.instances {
        let definition = definitions
            .get(instance.module.as_str())
            .copied()
            .with_context(|| {
                format!(
                    "instance `{}` references unknown logical module `{}`",
                    instance.name, instance.module
                )
            })?;
        if !definition.instances.is_empty() {
            eyre::bail!(
                "nested logical hierarchy is not supported for child `{}` yet",
                instance.name
            );
        }
        modules.push(graph_backed_module(definition, &instance.name)?);
    }
    modules.push(hierarchical_module(top, &definitions)?);
    finish_design(&top.name, target.name(), modules)
}

fn lower_state_design(module: &LogicalModule) -> eyre::Result<Option<RoutableDesign>> {
    let sequential = module
        .cells
        .iter()
        .filter(|cell| cell.kind.is_sequential())
        .collect::<Vec<_>>();
    let [state] = sequential.as_slice() else {
        return Ok(None);
    };
    // The legacy special case only covers modules whose entire combinational
    // logic is the state's next-state cone. Everything else goes to the
    // general mapper.
    if !module_is_simple_state_design(module) {
        return Ok(None);
    }

    let (width, edge) = match state.kind {
        LogicalCellKind::Dff { edge } => (1, edge),
        LogicalCellKind::Register { width, edge } => (width, edge),
        LogicalCellKind::DLatch { .. } => return Ok(None),
        _ => unreachable!(),
    };
    if edge != ClockEdge::Posedge {
        return Ok(None);
    }

    let output = state.output("q")?;
    let clock = net_value(state.input_value("clock")?, "register clock")?;
    let data = net_value(state.input_value("d")?, "register data")?;
    let driver = module.cells.iter().find(|cell| {
        !cell.kind.is_sequential() && cell.output("result").ok().map(String::as_str) == Some(data)
    });

    if width > 1 {
        let Some(driver) = driver else {
            return Ok(None);
        };
        if !matches!(driver.kind, LogicalCellKind::Inc)
            || net_value(driver.input_value("value")?, "increment input")? != output
        {
            return Ok(None);
        }
        return register_increment_design(module, output, clock, width).map(Some);
    }

    let next_expr = match driver {
        Some(driver) if matches!(driver.kind, LogicalCellKind::Inc) => {
            let input = net_value(driver.input_value("value")?, "increment input")?;
            if input != output {
                return Ok(None);
            }
            format!("~{output}")
        }
        Some(driver) if matches!(driver.kind, LogicalCellKind::Not) => {
            format!("~{}", net_value(driver.input_value("value")?, "not input")?)
        }
        Some(driver) if matches!(driver.kind, LogicalCellKind::Buffer) => {
            net_value(driver.input_value("value")?, "buffer input")?.to_owned()
        }
        Some(_) => return Ok(None),
        None => data.to_owned(),
    };
    scalar_dff_design(module, output, clock, &next_expr).map(Some)
}

fn scalar_dff_design(
    logical: &LogicalModule,
    output: &str,
    clock: &str,
    next_expr: &str,
) -> eyre::Result<RoutableDesign> {
    let clock_inverter = format!("{output}_clk_inv");
    let next = format!("{output}_next");
    let master = format!("{output}_master");
    let slave = format!("{output}_slave");

    let clock_module = not_clock_module(&clock_inverter)?;
    let next_module = combinational_output_module(&next, next_expr, "d")?;
    let master_module = d_latch_routable_module(&master);
    let slave_module = d_latch_routable_module(&slave);
    let mut connections = NetConnections::default();
    connections.connect(
        clock,
        NetClass::Clock,
        self_port(clock),
        instance_port(&clock_inverter, "clk"),
    );
    connections.connect(
        clock,
        NetClass::Clock,
        self_port(clock),
        instance_port(&slave, "en"),
    );
    connections.connect(
        "next_d",
        NetClass::Data,
        instance_port(&next, "d"),
        instance_port(&master, "d"),
    );
    connections.connect(
        "master_q",
        NetClass::Data,
        instance_port(&master, "q"),
        instance_port(&slave, "d"),
    );
    connections.connect(
        "clk_n",
        NetClass::Clock,
        instance_port(&clock_inverter, "clk_n"),
        instance_port(&master, "en"),
    );
    connections.connect(
        output,
        NetClass::Io,
        instance_port(&slave, "q"),
        self_port(output),
    );
    for port in &next_module.ports {
        if port.direction != RoutablePortDirection::Input {
            continue;
        }
        if port.name == output {
            connections.connect(
                output,
                NetClass::Data,
                instance_port(&slave, "q"),
                instance_port(&next, &port.name),
            );
        } else {
            let top_port = logical
                .ports
                .iter()
                .find(|candidate| candidate.net == port.name || candidate.name == port.name)
                .with_context(|| {
                    format!("next-state input `{}` is not a top-level port", port.name)
                })?;
            connections.connect(
                &top_port.name,
                NetClass::Io,
                self_port(&top_port.name),
                instance_port(&next, &port.name),
            );
        }
    }
    let top = composite_module(
        &logical.name,
        logical_ports(logical)?,
        [&clock_inverter, &next, &master, &slave],
        connections.finish(),
    );
    finish_design(
        &logical.name,
        ROUTABLE_IR_TARGET,
        vec![clock_module, next_module, master_module, slave_module, top],
    )
}

fn register_increment_design(
    logical: &LogicalModule,
    output: &str,
    clock: &str,
    width: usize,
) -> eyre::Result<RoutableDesign> {
    let mut modules = Vec::new();
    let mut instances = Vec::new();
    let mut connections = NetConnections::default();

    for carry_bit in 2..width {
        let carry = carry_module_name(output, carry_bit);
        modules.push(combinational_output_module(
            &carry,
            &carry_expr(output, carry_bit),
            &carry_signal_name(carry_bit),
        )?);
        instances.push(carry.clone());
        for input in carry_inputs(carry_bit) {
            connect_increment_input(&mut connections, output, input, &carry);
        }
    }

    for bit in 0..width {
        let bit_name = bit_signal_name(output, bit);
        let clock_inverter = format!("{bit_name}_clk_inv");
        let next = format!("{bit_name}_next");
        let master = format!("{bit_name}_master");
        let slave = format!("{bit_name}_slave");

        modules.push(not_clock_module(&clock_inverter)?);
        modules.push(next_bit_module(&next, output, bit)?);
        modules.push(d_latch_routable_module(&master));
        modules.push(d_latch_routable_module(&slave));
        instances.extend([
            clock_inverter.clone(),
            next.clone(),
            master.clone(),
            slave.clone(),
        ]);

        connections.connect(
            &format!("{next}_d"),
            NetClass::Data,
            instance_port(&next, "d"),
            instance_port(&master, "d"),
        );
        connections.connect(
            &format!("{master}_q"),
            NetClass::Data,
            instance_port(&master, "q"),
            instance_port(&slave, "d"),
        );
        connections.connect(
            &format!("{clock_inverter}_clk_n"),
            NetClass::Clock,
            instance_port(&clock_inverter, "clk_n"),
            instance_port(&master, "en"),
        );
        for input in next_bit_inputs(bit) {
            connect_increment_input(&mut connections, output, input, &next);
        }
        connections.connect(
            &bit_name,
            NetClass::Io,
            instance_port(&slave, "q"),
            self_port(&bit_name),
        );
        connections.connect(
            clock,
            NetClass::Clock,
            self_port(clock),
            instance_port(&clock_inverter, "clk"),
        );
        connections.connect(
            clock,
            NetClass::Clock,
            self_port(clock),
            instance_port(&slave, "en"),
        );
    }
    let top = composite_module(
        &logical.name,
        logical_ports(logical)?,
        instances.iter(),
        connections.finish(),
    );
    modules.push(top);
    finish_design(&logical.name, ROUTABLE_IR_TARGET, modules)
}

fn graph_backed_module(module: &LogicalModule, name: &str) -> eyre::Result<RoutableModule> {
    if !module.instances.is_empty() {
        eyre::bail!("logical child module `{}` is not a leaf", module.name);
    }
    if module.nets.iter().any(|net| net.width != 1) {
        eyre::bail!(
            "generic logical leaf lowering currently requires scalar nets in module `{}`",
            module.name
        );
    }

    let mut nodes = Vec::<GraphNode>::new();
    let mut producers = HashMap::<String, usize>::new();
    for port in module
        .ports
        .iter()
        .filter(|port| port.direction == LogicalPortDirection::Input)
    {
        let id = nodes.len();
        nodes.push(GraphNode {
            kind: GraphNodeKind::Input(port.name.clone()),
            ..Default::default()
        });
        producers.insert(port.net.clone(), id);
    }

    let mut pending = module.cells.iter().collect::<Vec<_>>();
    while !pending.is_empty() {
        let before = pending.len();
        let mut unresolved = Vec::new();
        for cell in pending {
            if !emit_graph_cell(cell, &mut nodes, &mut producers)? {
                unresolved.push(cell);
            }
        }
        pending = unresolved;
        if pending.len() == before {
            let names = pending
                .iter()
                .map(|cell| cell.name.as_str())
                .collect::<Vec<_>>()
                .join(", ");
            eyre::bail!("unresolved or cyclic logical leaf cells: {names}");
        }
    }

    for port in module
        .ports
        .iter()
        .filter(|port| port.direction == LogicalPortDirection::Output)
    {
        let input = producers
            .get(&port.net)
            .copied()
            .with_context(|| format!("output port `{}` has no logical producer", port.name))?;
        nodes.push(GraphNode {
            kind: GraphNodeKind::Output(port.name.clone()),
            inputs: vec![input],
            ..Default::default()
        });
    }

    let mut graph = Graph::from_nodes(nodes);
    graph.build_outputs();
    graph.build_producers();
    graph.build_consumers();
    graph.verify()?;
    routable_leaf_from_graph(name, graph)
}

fn emit_graph_cell(
    cell: &LogicalCell,
    nodes: &mut Vec<GraphNode>,
    producers: &mut HashMap<String, usize>,
) -> eyre::Result<bool> {
    let input_node = |pin: &str| -> eyre::Result<Option<usize>> {
        match cell.input_value(pin)? {
            LogicalValue::Net { net } => Ok(producers.get(net).copied()),
            LogicalValue::Slice { net, bit } => {
                if *bit != 0 {
                    eyre::bail!(
                        "logical slice lowering requires target mapping for cell `{}`",
                        cell.name
                    )
                }
                Ok(producers.get(net).copied())
            }
            LogicalValue::Constant { .. } => {
                eyre::bail!("logical constants are not supported by scalar leaf lowering yet")
            }
        }
    };
    let output = cell
        .outputs
        .first()
        .with_context(|| format!("cell `{}` has no output", cell.name))?
        .net
        .clone();

    if matches!(cell.kind, LogicalCellKind::Buffer) {
        let Some(source) = input_node("value")? else {
            return Ok(false);
        };
        producers.insert(output, source);
        return Ok(true);
    }

    let (kind, pins): (GraphNodeKind, &[&str]) = match &cell.kind {
        LogicalCellKind::Not => (
            GraphNodeKind::Logic(Logic {
                logic_type: LogicType::Not,
            }),
            &["value"],
        ),
        LogicalCellKind::And => (
            GraphNodeKind::Logic(Logic {
                logic_type: LogicType::And,
            }),
            &["lhs", "rhs"],
        ),
        LogicalCellKind::Or => (
            GraphNodeKind::Logic(Logic {
                logic_type: LogicType::Or,
            }),
            &["lhs", "rhs"],
        ),
        LogicalCellKind::Xor => (
            GraphNodeKind::Logic(Logic {
                logic_type: LogicType::Xor,
            }),
            &["lhs", "rhs"],
        ),
        LogicalCellKind::DLatch { .. } => (
            GraphNodeKind::Sequential(SequentialPrimitive::new(
                SequentialType::DLatch,
                vec!["d".to_owned(), "en".to_owned()],
                vec!["q".to_owned()],
            )),
            &["d", "enable"],
        ),
        LogicalCellKind::Add
        | LogicalCellKind::Inc
        | LogicalCellKind::Eq { .. }
        | LogicalCellKind::Mux
        | LogicalCellKind::Dff { .. }
        | LogicalCellKind::Register { .. } => {
            eyre::bail!(
                "logical cell `{}` requires target mapping before scalar leaf lowering",
                cell.name
            )
        }
        LogicalCellKind::Buffer => unreachable!(),
    };
    let mut inputs = Vec::with_capacity(pins.len());
    for pin in pins {
        let Some(node) = input_node(pin)? else {
            return Ok(false);
        };
        inputs.push(node);
    }
    let id = nodes.len();
    nodes.push(GraphNode {
        kind,
        inputs,
        tag: cell.name.clone(),
        ..Default::default()
    });
    producers.insert(output, id);
    Ok(true)
}

fn hierarchical_module(
    module: &LogicalModule,
    definitions: &HashMap<&str, &LogicalModule>,
) -> eyre::Result<RoutableModule> {
    if module.nets.iter().any(|net| net.width != 1) {
        eyre::bail!("hierarchical routable lowering currently requires scalar nets");
    }
    let mut sources = HashMap::<String, Vec<Endpoint>>::new();
    let mut sinks = HashMap::<String, Vec<Endpoint>>::new();
    for instance in &module.instances {
        let definition = definitions
            .get(instance.module.as_str())
            .copied()
            .with_context(|| format!("unknown logical module `{}`", instance.module))?;
        for binding in &instance.bindings {
            let port = definition
                .ports
                .iter()
                .find(|port| port.name == binding.port)
                .with_context(|| {
                    format!(
                        "instance `{}.{}` has no matching port",
                        instance.name, binding.port
                    )
                })?;
            let endpoint = instance_port(&instance.name, &binding.port);
            match port.direction {
                LogicalPortDirection::Input => {
                    sinks.entry(binding.net.clone()).or_default().push(endpoint)
                }
                LogicalPortDirection::Output => sources
                    .entry(binding.net.clone())
                    .or_default()
                    .push(endpoint),
            }
        }
    }

    let mut nets = Vec::new();
    for net in &module.nets {
        if net.width != 1 {
            eyre::bail!("hierarchical routable lowering currently requires scalar nets");
        }
        let top_input = module
            .ports
            .iter()
            .find(|port| port.net == net.name && port.direction == LogicalPortDirection::Input);
        let top_output = module
            .ports
            .iter()
            .find(|port| port.net == net.name && port.direction == LogicalPortDirection::Output);
        let driver = if let Some(port) = top_input {
            self_port(&port.name)
        } else {
            single_endpoint(sources.get(&net.name))
                .with_context(|| format!("logical net `{}` has no unique driver", net.name))?
        };
        let mut net_sinks = sinks.remove(&net.name).unwrap_or_default();
        if let Some(port) = top_output {
            net_sinks.push(self_port(&port.name));
        }
        if net_sinks.is_empty() {
            continue;
        }
        nets.push(RoutableNet {
            name: net.name.clone(),
            class: classify_logical_net(&net.name, top_input.is_some() || top_output.is_some()),
            driver,
            sinks: net_sinks,
            origin: Some(net.name.clone()),
        });
    }
    Ok(composite_module(
        &module.name,
        logical_ports(module)?,
        module.instances.iter().map(|instance| &instance.name),
        nets,
    ))
}

fn next_bit_module(name: &str, signal: &str, bit: usize) -> eyre::Result<RoutableModule> {
    let bit_name = bit_signal_name(signal, bit);
    if bit == 0 {
        return combinational_output_module(name, &format!("~{bit_name}"), "d");
    }
    let rhs = if bit == 1 {
        bit_signal_name(signal, 0)
    } else {
        carry_signal_name(bit)
    };
    buffered_xor_output_module(name, &bit_name, &rhs, "d")
}

fn buffered_xor_output_module(
    name: &str,
    left: &str,
    right: &str,
    output: &str,
) -> eyre::Result<RoutableModule> {
    let product = format!("{output}_and");
    let mut graph = LogicGraph::from_stmt(&format!("{left}&{right}"), &product)?;
    graph.graph.merge(
        LogicGraph::from_stmt(
            &format!("(~({product}|~{left}))|(~({product}|~{right}))"),
            output,
        )?
        .graph,
    );
    graph.graph.remove_output(&product);
    routable_leaf_from_graph(name, graph.graph)
}

#[derive(Clone, Copy)]
enum IncrementInput {
    Bit(usize),
    Carry(usize),
}

fn next_bit_inputs(bit: usize) -> Vec<IncrementInput> {
    match bit {
        0 => vec![IncrementInput::Bit(0)],
        1 => vec![IncrementInput::Bit(1), IncrementInput::Bit(0)],
        _ => vec![IncrementInput::Bit(bit), IncrementInput::Carry(bit)],
    }
}

fn carry_inputs(bit: usize) -> Vec<IncrementInput> {
    if bit == 2 {
        vec![IncrementInput::Bit(1), IncrementInput::Bit(0)]
    } else {
        vec![IncrementInput::Bit(bit - 1), IncrementInput::Carry(bit - 1)]
    }
}

fn connect_increment_input(
    connections: &mut NetConnections,
    signal: &str,
    input: IncrementInput,
    target: &str,
) {
    let (source_instance, source_port) = match input {
        IncrementInput::Bit(bit) => (format!("{}_{}_slave", signal, bit), "q".to_owned()),
        IncrementInput::Carry(bit) => (carry_module_name(signal, bit), carry_signal_name(bit)),
    };
    let target_port = match input {
        IncrementInput::Bit(bit) => bit_signal_name(signal, bit),
        IncrementInput::Carry(bit) => carry_signal_name(bit),
    };
    connections.connect(
        &format!("{source_instance}_{source_port}"),
        NetClass::Data,
        instance_port(&source_instance, &source_port),
        instance_port(target, &target_port),
    );
}

fn carry_expr(signal: &str, bit: usize) -> String {
    if bit == 2 {
        format!(
            "{}&{}",
            bit_signal_name(signal, 1),
            bit_signal_name(signal, 0)
        )
    } else {
        format!(
            "{}&{}",
            bit_signal_name(signal, bit - 1),
            carry_signal_name(bit - 1)
        )
    }
}

fn carry_signal_name(bit: usize) -> String {
    format!("carry_{bit}")
}

fn carry_module_name(signal: &str, bit: usize) -> String {
    format!("{signal}_carry_{bit}")
}

fn net_value<'a>(value: &'a LogicalValue, role: &str) -> eyre::Result<&'a str> {
    match value {
        LogicalValue::Net { net } => Ok(net),
        LogicalValue::Slice { .. } => eyre::bail!("{role} must be a plain net"),
        LogicalValue::Constant { .. } => eyre::bail!("{role} must be a net"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn counter_lowering_does_not_require_verilog_reconstruction() -> eyre::Result<()> {
        let logical = LogicalDesign::from_verilog_source(
            r#"
            module counter(clk, q);
              input clk;
              output reg [1:0] q;
              always @(posedge clk) begin
                q <= q + 1;
              end
            endmodule
            "#,
        )?;

        let routable = lower_logical_to_routable(&logical)?;
        let top = routable.module("counter").context("missing counter top")?;
        let super::super::routable::RoutableModuleBody::Composite { instances, .. } = &top.body
        else {
            panic!("counter must lower to a composite routable module");
        };
        assert!(instances
            .iter()
            .any(|instance| instance.name == "q_0_master"));
        assert!(instances
            .iter()
            .any(|instance| instance.name == "q_1_slave"));
        Ok(())
    }
}
