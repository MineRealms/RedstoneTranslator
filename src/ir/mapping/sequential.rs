//! General decomposition of logical state cells into target-supported leaves.
//!
//! A flat logical module with state is partitioned into:
//!
//! - one next-state combinational leaf per live state cell (when its data is
//!   not a direct boundary net),
//! - a clock inverter and a master/slave D-latch pair per `Dff`/`Register`
//!   bit (the [`RegisterMapping::MasterSlaveLatches`] decomposition),
//! - one latch leaf per `DLatch` bit,
//! - one combinational output leaf for module outputs that are not driven
//!   directly by a state cell.
//!
//! State cells whose output cannot reach any module output are eliminated
//! before leaves are built, so dead registers never produce unconnected
//! instance ports. The top composite wires the leaves through typed nets.

use std::collections::{BTreeSet, HashMap, HashSet};

use eyre::ContextCompat;

use super::scalar::{LeafBuilder, ScalarNets};
use super::{
    classify_logical_net, combinational, composite_module, d_latch_routable_module, finish_design,
    instance_port, logical_ports_filtered, not_clock_module, self_port, NetConnections,
    LEAF_NODE_BUDGET,
};
use crate::ir::target::{MappingPolicy, TargetOp, TargetSpec};
use crate::ir::{
    ClockEdge, Endpoint, LogicalCell, LogicalCellKind, LogicalModule, LogicalPortDirection,
    LogicalValue, NetClass, RoutableDesign, RoutableModule,
};

/// One scalar bit of a logical net.
struct BitRef {
    net: String,
    bit: usize,
}

enum NextSource {
    /// The data bit is already a boundary net (module input or state output).
    Direct(String),
    /// The data bit is produced by a generated next-state leaf.
    Leaf { instance: String, port: String },
}

/// A next-state bit before cone partitioning resolves its producing leaf.
enum PendingSource {
    Direct(String),
    Port(String),
}

/// The next-state data of a state cell: a net, a net bit, or a constant.
enum StateData {
    Net(String),
    Slice { net: String, bit: usize },
    Constant { value: u128, width: usize },
}

impl StateData {
    fn from_value(value: &LogicalValue) -> Self {
        match value {
            LogicalValue::Net { net } => StateData::Net(net.clone()),
            LogicalValue::Slice { net, bit } => StateData::Slice {
                net: net.clone(),
                bit: *bit,
            },
            LogicalValue::Constant { value, width } => StateData::Constant {
                value: *value,
                width: *width,
            },
        }
    }
}

enum StateKind {
    Dff {
        edge: ClockEdge,
        clock: String,
        data: StateData,
    },
    DLatch {
        enable: String,
        data: StateData,
    },
}

impl StateKind {
    fn data(&self) -> &StateData {
        match self {
            StateKind::Dff { data, .. } | StateKind::DLatch { data, .. } => data,
        }
    }
}

struct StateBitPlan {
    bit: usize,
    /// Scalar name of the state output bit, also the instance-name tag.
    output: String,
    /// Final element that exposes the state output (`_slave` or `_latch`).
    element: String,
}

struct StatePlan {
    width: usize,
    kind: StateKind,
    next_instance: String,
    bits: Vec<StateBitPlan>,
}

/// Lowers a flat module that contains at least one state cell.
pub(super) fn lower_sequential_module(
    module: &LogicalModule,
    nets: &ScalarNets,
    state_cells: &[&LogicalCell],
    target: &TargetSpec,
    policy: &MappingPolicy,
) -> eyre::Result<RoutableDesign> {
    target.require(TargetOp::DLatch)?;

    let mut driver_of = HashMap::new();
    for cell in &module.cells {
        if let Some(output) = cell.outputs.first() {
            driver_of.insert(output.net.as_str(), cell);
        }
    }
    let input_nets = module
        .ports
        .iter()
        .filter(|port| port.direction == LogicalPortDirection::Input)
        .map(|port| port.net.as_str())
        .collect::<HashSet<_>>();
    let mut input_scalars = HashSet::new();
    for port in module
        .ports
        .iter()
        .filter(|port| port.direction == LogicalPortDirection::Input)
    {
        for scalar in nets.bits(&port.net)? {
            input_scalars.insert(scalar);
        }
    }
    let state_outputs = state_cells
        .iter()
        .flat_map(|cell| cell.outputs.iter().map(|output| output.net.as_str()))
        .collect::<HashSet<_>>();

    let live = live_state_cells(module, &driver_of);
    let live_cells = state_cells
        .iter()
        .copied()
        .filter(|cell| live.contains(cell.name.as_str()))
        .collect::<Vec<_>>();

    let emitter = ConeEmitter {
        nets,
        driver_of: driver_of.clone(),
        state_outputs,
        input_nets,
        target,
        policy,
    };

    // Phase 1: name every element before any wiring so forward references
    // between state cells resolve to the final element instance.
    let mut plans = Vec::<StatePlan>::new();
    for cell in &live_cells {
        let output = cell.output("q")?.clone();
        let width = nets.width(&output)?;
        let data = StateData::from_value(cell.input_value("d")?);
        let kind = match &cell.kind {
            LogicalCellKind::Dff { edge } | LogicalCellKind::Register { edge, .. } => {
                StateKind::Dff {
                    edge: *edge,
                    clock: net_name(cell.input_value("clock")?, "state clock")?,
                    data,
                }
            }
            LogicalCellKind::DLatch { .. } => StateKind::DLatch {
                enable: net_name(cell.input_value("enable")?, "latch enable")?,
                data,
            },
            _ => unreachable!("live_cells only contains sequential cells"),
        };
        let is_dff = matches!(
            cell.kind,
            LogicalCellKind::Dff { .. } | LogicalCellKind::Register { .. }
        );
        let mut bits = Vec::with_capacity(width);
        for bit in 0..width {
            let scalar = nets.scalar(&output, bit)?;
            let element = if is_dff {
                format!("{scalar}_slave")
            } else {
                format!("{scalar}_latch")
            };
            bits.push(StateBitPlan {
                bit,
                output: scalar,
                element,
            });
        }
        plans.push(StatePlan {
            next_instance: format!("{output}_next"),
            width,
            kind,
            bits,
        });
    }

    let mut state_endpoints = HashMap::<String, Endpoint>::new();
    for plan in &plans {
        for bit_plan in &plan.bits {
            state_endpoints.insert(
                bit_plan.output.clone(),
                instance_port(&bit_plan.element, "q"),
            );
        }
    }

    let mut modules = Vec::<RoutableModule>::new();
    let mut instance_names = Vec::<String>::new();
    let mut used_inputs = BTreeSet::<String>::new();
    let mut next_sources = Vec::<Vec<NextSource>>::new();
    let mut next_leaf_instances = Vec::new();
    let mut next_leaf_intermediates = Vec::<HashMap<String, String>>::new();

    // Phase 2: build the next-state combinational cone of every state cell and
    // split it into leaves that fit the local placer node budget.
    for plan in &plans {
        let data = plan.kind.data();
        let mut builder = LeafBuilder::new();
        let mut pending = Vec::with_capacity(plan.width);
        for bit in 0..plan.width {
            let producer = match data {
                StateData::Net(net) => {
                    emitter.ensure(
                        &BitRef {
                            net: net.clone(),
                            bit,
                        },
                        &mut builder,
                        &mut used_inputs,
                    )?;
                    let scalar = nets.scalar(net, bit)?;
                    builder
                        .producer(&scalar)
                        .with_context(|| format!("next-state lowering lost `{scalar}`"))?
                }
                StateData::Slice { net, bit: data_bit } => {
                    emitter.ensure(
                        &BitRef {
                            net: net.clone(),
                            bit: *data_bit,
                        },
                        &mut builder,
                        &mut used_inputs,
                    )?;
                    let scalar = nets.scalar(net, *data_bit)?;
                    builder
                        .producer(&scalar)
                        .with_context(|| format!("next-state lowering lost `{scalar}`"))?
                }
                StateData::Constant { value, width } => {
                    builder.constant(bit < *width && (*value >> bit) & 1 == 1)
                }
            };
            match builder.input_name(producer) {
                Some(name) => pending.push(PendingSource::Direct(name.to_owned())),
                None => {
                    // The generated port must not collide with any boundary
                    // net that feeds the cone, so it uses a reserved prefix.
                    let port = if plan.width == 1 {
                        "__next".to_owned()
                    } else {
                        format!("__next_{bit}")
                    };
                    builder.add_output(&port, producer);
                    pending.push(PendingSource::Port(port));
                }
            }
        }
        let partitioned = builder.partition(&plan.next_instance, LEAF_NODE_BUDGET)?;
        let mut sources = Vec::with_capacity(pending.len());
        for pending in pending {
            match pending {
                PendingSource::Direct(name) => sources.push(NextSource::Direct(name)),
                PendingSource::Port(port) => {
                    let (instance, leaf_port) = partitioned
                        .output_sources
                        .get(&port)
                        .with_context(|| format!("partitioned next cone lost `{port}`"))?
                        .clone();
                    sources.push(NextSource::Leaf {
                        instance,
                        port: leaf_port,
                    });
                }
            }
        }
        for leaf in &partitioned.instances {
            instance_names.push(leaf.instance.clone());
        }
        modules.extend(partitioned.modules);
        next_leaf_intermediates.push(partitioned.intermediates);
        next_leaf_instances.push(partitioned.instances);
        next_sources.push(sources);
    }

    // Phase 2b: build the state elements themselves.
    for plan in &plans {
        for bit_plan in &plan.bits {
            match plan.kind {
                StateKind::Dff { .. } => {
                    let clock_inverter = format!("{}_clk_inv", bit_plan.output);
                    let master = format!("{}_master", bit_plan.output);
                    modules.push(not_clock_module(&clock_inverter)?);
                    modules.push(d_latch_routable_module(&master));
                    modules.push(d_latch_routable_module(&bit_plan.element));
                    instance_names.push(clock_inverter);
                    instance_names.push(master);
                    instance_names.push(bit_plan.element.clone());
                }
                StateKind::DLatch { .. } => {
                    modules.push(d_latch_routable_module(&bit_plan.element));
                    instance_names.push(bit_plan.element.clone());
                }
            }
        }
    }

    // Phase 3: wire clocks, next-state inputs and state data.
    let mut connections = NetConnections::default();
    for (index, plan) in plans.iter().enumerate() {
        let data = plan.kind.data();
        for leaf in &next_leaf_instances[index] {
            for input in &leaf.inputs {
                let endpoint = if let Some(producer) = next_leaf_intermediates[index].get(input) {
                    instance_port(producer, input)
                } else {
                    endpoint_for_scalar(input, &state_endpoints, &input_scalars, &mut used_inputs)?
                };
                connections.connect(
                    input,
                    classify_logical_net(input, false),
                    endpoint,
                    instance_port(&leaf.instance, input),
                );
            }
        }
        match &plan.kind {
            StateKind::Dff { edge, clock, .. } => {
                let clock_scalar = nets.scalar(clock, 0)?;
                for (bit_index, bit_plan) in plan.bits.iter().enumerate() {
                    let clock_inverter = format!("{}_clk_inv", bit_plan.output);
                    let master = format!("{}_master", bit_plan.output);
                    let slave = &bit_plan.element;
                    let clock_endpoint = input_endpoint(&mut used_inputs, &clock_scalar);
                    connections.connect(
                        clock,
                        NetClass::Clock,
                        clock_endpoint.clone(),
                        instance_port(&clock_inverter, "clk"),
                    );
                    let inverted_clock = format!("{clock_inverter}_clk_n");
                    match edge {
                        ClockEdge::Posedge => {
                            connections.connect(
                                &inverted_clock,
                                NetClass::Clock,
                                instance_port(&clock_inverter, "clk_n"),
                                instance_port(&master, "en"),
                            );
                            connections.connect(
                                clock,
                                NetClass::Clock,
                                clock_endpoint,
                                instance_port(slave, "en"),
                            );
                        }
                        ClockEdge::Negedge => {
                            connections.connect(
                                &inverted_clock,
                                NetClass::Clock,
                                instance_port(&clock_inverter, "clk_n"),
                                instance_port(slave, "en"),
                            );
                            connections.connect(
                                clock,
                                NetClass::Clock,
                                clock_endpoint,
                                instance_port(&master, "en"),
                            );
                        }
                    }
                    let data_scalar = match data {
                        StateData::Net(net) => nets.scalar(net, bit_plan.bit)?,
                        StateData::Slice { net, bit } => nets.scalar(net, *bit)?,
                        StateData::Constant { .. } => format!("__const_{}", bit_plan.bit),
                    };
                    let driver = next_source_endpoint(
                        &next_sources[index][bit_index],
                        &state_endpoints,
                        &input_scalars,
                        &mut used_inputs,
                    )?;
                    connections.connect(
                        &data_scalar,
                        NetClass::Data,
                        driver,
                        instance_port(&master, "d"),
                    );
                    connections.connect(
                        &format!("{master}_q"),
                        NetClass::Data,
                        instance_port(&master, "q"),
                        instance_port(slave, "d"),
                    );
                }
            }
            StateKind::DLatch { enable, .. } => {
                let enable_scalar = nets.scalar(enable, 0)?;
                for (bit_index, bit_plan) in plan.bits.iter().enumerate() {
                    let enable_endpoint = input_endpoint(&mut used_inputs, &enable_scalar);
                    connections.connect(
                        enable,
                        NetClass::Data,
                        enable_endpoint,
                        instance_port(&bit_plan.element, "en"),
                    );
                    let data_scalar = match data {
                        StateData::Net(net) => nets.scalar(net, bit_plan.bit)?,
                        StateData::Slice { net, bit } => nets.scalar(net, *bit)?,
                        StateData::Constant { .. } => format!("__const_{}", bit_plan.bit),
                    };
                    let driver = next_source_endpoint(
                        &next_sources[index][bit_index],
                        &state_endpoints,
                        &input_scalars,
                        &mut used_inputs,
                    )?;
                    connections.connect(
                        &data_scalar,
                        NetClass::Data,
                        driver,
                        instance_port(&bit_plan.element, "d"),
                    );
                }
            }
        }
    }

    // Phase 4: combinational module outputs get one shared output leaf.
    let output_instance = format!("{}_out", module.name);
    let mut output_builder = LeafBuilder::new();
    let mut output_wires = Vec::<(String, Endpoint)>::new();
    let mut output_logic = Vec::<String>::new();
    for port in module
        .ports
        .iter()
        .filter(|port| port.direction == LogicalPortDirection::Output)
    {
        for bit in 0..nets.width(&port.net)? {
            let scalar = nets.scalar(&port.net, bit)?;
            if let Some(endpoint) = state_endpoints.get(&scalar) {
                output_wires.push((scalar, endpoint.clone()));
                continue;
            }
            let Some(cell) = driver_of.get(port.net.as_str()).copied() else {
                eyre::bail!("output port `{}` has no logical driver", port.name);
            };
            if cell.kind.is_sequential() {
                eyre::bail!(
                    "output port `{}` references a state cell that was not lowered",
                    port.name
                );
            }
            emitter.ensure(
                &BitRef {
                    net: port.net.clone(),
                    bit,
                },
                &mut output_builder,
                &mut used_inputs,
            )?;
            let producer = output_builder
                .producer(&scalar)
                .with_context(|| format!("output lowering lost `{scalar}`"))?;
            match output_builder.input_name(producer) {
                Some(name) => {
                    let name = name.to_owned();
                    let endpoint = endpoint_for_scalar(
                        &name,
                        &state_endpoints,
                        &input_scalars,
                        &mut used_inputs,
                    )?;
                    output_wires.push((scalar, endpoint));
                }
                None => {
                    output_builder.add_output(&scalar, producer);
                    output_logic.push(scalar);
                }
            }
        }
    }
    if output_builder.has_outputs() {
        let partitioned = output_builder.partition(&output_instance, LEAF_NODE_BUDGET)?;
        for leaf in &partitioned.instances {
            instance_names.push(leaf.instance.clone());
            for input in &leaf.inputs {
                let endpoint = if let Some(producer) = partitioned.intermediates.get(input) {
                    instance_port(producer, input)
                } else {
                    endpoint_for_scalar(input, &state_endpoints, &input_scalars, &mut used_inputs)?
                };
                connections.connect(
                    input,
                    classify_logical_net(input, false),
                    endpoint,
                    instance_port(&leaf.instance, input),
                );
            }
        }
        for scalar in output_logic {
            let (instance, port) = partitioned
                .output_sources
                .get(&scalar)
                .with_context(|| format!("partitioned output cone lost `{scalar}`"))?
                .clone();
            output_wires.push((scalar, instance_port(&instance, &port)));
        }
        modules.extend(partitioned.modules);
    }
    for (scalar, driver) in output_wires {
        connections.connect(&scalar, NetClass::Io, driver, self_port(&scalar));
    }

    let ports = logical_ports_filtered(module, nets, &used_inputs)?;
    let top = composite_module(
        &module.name,
        ports,
        instance_names.iter(),
        connections.finish(),
    );
    modules.push(top);
    finish_design(&module.name, target.name(), modules)
}

/// Marks state cells whose output can reach a module output. The worklist also
/// follows the data net of every live state cell, so chains of state cells are
/// discovered to a fixed point. Dead state cells are eliminated entirely.
fn live_state_cells<'a>(
    module: &'a LogicalModule,
    driver_of: &HashMap<&'a str, &'a LogicalCell>,
) -> HashSet<&'a str> {
    let mut live = HashSet::new();
    let mut needed = HashSet::new();
    let mut stack = module
        .ports
        .iter()
        .filter(|port| port.direction == LogicalPortDirection::Output)
        .map(|port| port.net.as_str())
        .collect::<Vec<_>>();
    while let Some(net) = stack.pop() {
        if !needed.insert(net) {
            continue;
        }
        let Some(cell) = driver_of.get(net).copied() else {
            continue;
        };
        if cell.kind.is_sequential() {
            if live.insert(cell.name.as_str()) {
                if let Ok(LogicalValue::Net { net }) = cell.input_value("d") {
                    stack.push(net.as_str());
                }
            }
            continue;
        }
        for input in &cell.inputs {
            if let LogicalValue::Net { net } = &input.value {
                stack.push(net.as_str());
            }
        }
    }
    live
}

/// Emits the combinational fan-in cone of one scalar bit on demand.
struct ConeEmitter<'a> {
    nets: &'a ScalarNets,
    driver_of: HashMap<&'a str, &'a LogicalCell>,
    state_outputs: HashSet<&'a str>,
    input_nets: HashSet<&'a str>,
    target: &'a TargetSpec,
    policy: &'a MappingPolicy,
}

impl ConeEmitter<'_> {
    fn ensure(
        &self,
        bit: &BitRef,
        builder: &mut LeafBuilder,
        used_inputs: &mut BTreeSet<String>,
    ) -> eyre::Result<()> {
        let scalar = self.nets.scalar(&bit.net, bit.bit)?;
        if builder.has_producer(&scalar) {
            return Ok(());
        }
        if self.input_nets.contains(bit.net.as_str()) {
            used_inputs.insert(scalar.clone());
            builder.add_input(&scalar);
            return Ok(());
        }
        if self.state_outputs.contains(bit.net.as_str()) {
            builder.add_input(&scalar);
            return Ok(());
        }
        let Some(cell) = self.driver_of.get(bit.net.as_str()).copied() else {
            eyre::bail!("logical net `{}` has no driver", bit.net);
        };
        if cell.kind.is_sequential() {
            builder.add_input(&scalar);
            return Ok(());
        }
        for input in &cell.inputs {
            match &input.value {
                LogicalValue::Net { net } => {
                    for input_bit in 0..self.nets.width(net)? {
                        self.ensure(
                            &BitRef {
                                net: net.clone(),
                                bit: input_bit,
                            },
                            builder,
                            used_inputs,
                        )?;
                    }
                }
                // Constants are materialized by `emit_cell` itself.
                LogicalValue::Constant { .. } => {}
                LogicalValue::Slice { net, bit } => {
                    self.ensure(
                        &BitRef {
                            net: net.clone(),
                            bit: *bit,
                        },
                        builder,
                        used_inputs,
                    )?;
                }
            }
        }
        if !combinational::emit_cell(cell, self.nets, builder, self.target, self.policy)? {
            eyre::bail!("logical cell `{}` could not be emitted", cell.name);
        }
        Ok(())
    }
}

fn next_source_endpoint(
    source: &NextSource,
    state_endpoints: &HashMap<String, Endpoint>,
    input_scalars: &HashSet<String>,
    used_inputs: &mut BTreeSet<String>,
) -> eyre::Result<Endpoint> {
    match source {
        NextSource::Direct(name) => {
            endpoint_for_scalar(name, state_endpoints, input_scalars, used_inputs)
        }
        NextSource::Leaf { instance, port } => Ok(instance_port(instance, port)),
    }
}

fn endpoint_for_scalar(
    scalar: &str,
    state_endpoints: &HashMap<String, Endpoint>,
    input_scalars: &HashSet<String>,
    used_inputs: &mut BTreeSet<String>,
) -> eyre::Result<Endpoint> {
    if let Some(endpoint) = state_endpoints.get(scalar) {
        return Ok(endpoint.clone());
    }
    if input_scalars.contains(scalar) {
        used_inputs.insert(scalar.to_owned());
        return Ok(self_port(scalar));
    }
    eyre::bail!("cannot resolve the source of scalar net `{scalar}`")
}

fn input_endpoint(used_inputs: &mut BTreeSet<String>, scalar: &str) -> Endpoint {
    used_inputs.insert(scalar.to_owned());
    self_port(scalar)
}

fn net_name(value: &LogicalValue, role: &str) -> eyre::Result<String> {
    match value {
        LogicalValue::Net { net } => Ok(net.clone()),
        LogicalValue::Slice { .. } => eyre::bail!("{role} must be a plain net"),
        LogicalValue::Constant { .. } => eyre::bail!("{role} must be a net"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ir::{LogicalDesign, RoutableModuleBody, RoutableNodeKind};

    fn lower(source: &str) -> RoutableDesign {
        LogicalDesign::from_verilog_source(source)
            .expect("test source must parse")
            .lower_to_routable()
            .expect("test design must lower")
    }

    fn top_composite(
        design: &RoutableDesign,
    ) -> (&[crate::ir::RoutableInstance], &[crate::ir::RoutableNet]) {
        let top = design.module(&design.top).expect("top module");
        let RoutableModuleBody::Composite { instances, nets } = &top.body else {
            panic!("top module must be a composite");
        };
        (instances, nets)
    }

    #[test]
    fn enabled_dff_lowers_with_mux_next_state() {
        let design = lower(
            r#"
            module dff_en(clk, en, d, q);
              input clk, en, d;
              output reg q;
              always @(posedge clk) begin
                if (en) begin
                  q <= d;
                end
              end
            endmodule
            "#,
        );
        let (instances, _) = top_composite(&design);
        for expected in ["q_clk_inv", "q_next", "q_master", "q_slave"] {
            assert!(
                instances.iter().any(|instance| instance.name == expected),
                "missing instance `{expected}`"
            );
        }

        let next = design.module("q_next").expect("next leaf");
        let RoutableModuleBody::Leaf { nodes } = &next.body else {
            panic!("next-state module must be a leaf");
        };
        assert!(nodes
            .iter()
            .any(|node| matches!(node.kind, RoutableNodeKind::And)));
        assert!(nodes
            .iter()
            .any(|node| matches!(node.kind, RoutableNodeKind::Or)));
        assert!(nodes
            .iter()
            .any(|node| matches!(node.kind, RoutableNodeKind::Not)));
    }

    #[test]
    fn negedge_register_swaps_master_and_slave_enables() {
        let logical: LogicalDesign = r#"
rcir 1;
stage logical;
top m;
module m {
  port input  clk : bit;
  port input  d : bit;
  port output q : bit;
  cell state : logical.dff<1> {
    in d = d;
    in clock = clk;
    out q = q;
    edge = negedge;
  }
}
"#
        .parse()
        .expect("rcir must parse");
        let design = logical.lower_to_routable().expect("negedge dff must lower");
        let (_, nets) = top_composite(&design);

        let inverted = nets
            .iter()
            .find(|net| {
                matches!(
                    &net.driver,
                    Endpoint::InstancePort { instance, port }
                        if instance == "q_clk_inv" && port == "clk_n"
                )
            })
            .expect("inverted clock net");
        assert!(inverted.sinks.contains(&Endpoint::InstancePort {
            instance: "q_slave".to_owned(),
            port: "en".to_owned(),
        }));
        assert!(!inverted.sinks.contains(&Endpoint::InstancePort {
            instance: "q_master".to_owned(),
            port: "en".to_owned(),
        }));

        let clock = nets
            .iter()
            .find(|net| {
                matches!(
                    &net.driver,
                    Endpoint::SelfPort { port } if port == "clk"
                )
            })
            .expect("clock net");
        assert!(clock.sinks.contains(&Endpoint::InstancePort {
            instance: "q_master".to_owned(),
            port: "en".to_owned(),
        }));
    }

    #[test]
    fn chained_registers_wire_directly_without_next_leaves() {
        let logical: LogicalDesign = r#"
rcir 1;
stage logical;
top m;
module m {
  port input  clk : bit;
  port input  d : bit;
  port output q0 : bit;
  port output q1 : bit;
  cell s0 : logical.dff<1> {
    in d = q1;
    in clock = clk;
    out q = q0;
    edge = posedge;
  }
  cell s1 : logical.dff<1> {
    in d = q0;
    in clock = clk;
    out q = q1;
    edge = posedge;
  }
}
"#
        .parse()
        .expect("rcir must parse");
        let design = logical
            .lower_to_routable()
            .expect("chained registers must lower");
        let (instances, nets) = top_composite(&design);

        for expected in ["q0_master", "q0_slave", "q1_master", "q1_slave"] {
            assert!(instances.iter().any(|instance| instance.name == expected));
        }
        assert!(design.module("q0_next").is_none());
        assert!(design.module("q1_next").is_none());

        let q1_to_q0 = nets
            .iter()
            .find(|net| {
                matches!(
                    &net.driver,
                    Endpoint::InstancePort { instance, port }
                        if instance == "q1_slave" && port == "q"
                )
            })
            .expect("q1 must feed q0");
        assert!(q1_to_q0.sinks.contains(&Endpoint::InstancePort {
            instance: "q0_master".to_owned(),
            port: "d".to_owned(),
        }));
    }

    #[test]
    fn dead_register_is_eliminated() {
        let logical: LogicalDesign = r#"
rcir 1;
stage logical;
top m;
module m {
  port input  clk : bit;
  port input  d : bit;
  port output q : bit;
  net unused : bit;
  cell live : logical.dff<1> {
    in d = d;
    in clock = clk;
    out q = q;
    edge = posedge;
  }
  cell dead : logical.dff<1> {
    in d = d;
    in clock = clk;
    out q = unused;
    edge = posedge;
  }
}
"#
        .parse()
        .expect("rcir must parse");
        let design = logical
            .lower_to_routable()
            .expect("dead register design must lower");
        let (instances, _) = top_composite(&design);

        assert!(instances.iter().any(|instance| instance.name == "q_master"));
        assert!(!instances
            .iter()
            .any(|instance| instance.name == "unused_master"));
    }

    #[test]
    fn enabled_dff_lowers_to_a_pnr_ready_topology() {
        let design = lower(
            r#"
            module dff_en(clk, en, d, q);
              input clk, en, d;
              output reg q;
              always @(posedge clk) begin
                if (en) begin
                  q <= d;
                end
              end
            endmodule
            "#,
        );
        let topology =
            crate::transform::place_and_route::global_pnr::topology::ResolvedPnrTopology::from_routable(
                &design,
            )
            .expect("general lowering output must resolve to a PnR topology");
        for expected in ["q_next", "q_clk_inv", "q_master", "q_slave"] {
            assert!(
                topology.instance_by_name(expected).is_some(),
                "missing resolved instance `{expected}`"
            );
        }
    }

    #[test]
    fn wide_register_next_state_is_partitioned_into_placeable_leaves() {
        let logical: LogicalDesign = r#"
rcir 1;
stage logical;
top m;
module m {
  port input  clk : bit;
  port input  x : bits[32];
  port output q : bits[32];
  net next : bits[32];
  cell add : logical.add<32> {
    in lhs = q;
    in rhs = x;
    out result = next;
  }
  cell state : logical.register<32> {
    in d = next;
    in clock = clk;
    out q = q;
    edge = posedge;
  }
}
"#
        .parse()
        .expect("rcir must parse");
        let design = logical
            .lower_to_routable()
            .expect("wide register must lower");

        let mut next_leaves = 0;
        for module in &design.modules {
            let RoutableModuleBody::Leaf { nodes } = &module.body else {
                continue;
            };
            assert!(
                nodes.len() <= 40,
                "leaf `{}` has {} nodes, above the local placer limit",
                module.name,
                nodes.len()
            );
            if module.name.starts_with("q_next") {
                next_leaves += 1;
            }
        }
        assert!(
            next_leaves >= 2,
            "a 32-bit next-state cone must partition into several leaves, found {next_leaves}"
        );

        let topology =
            crate::transform::place_and_route::global_pnr::topology::ResolvedPnrTopology::from_routable(
                &design,
            )
            .expect("partitioned design must resolve to a PnR topology");
        assert!(topology.instance_by_name("q_next_p0").is_some());
        assert!(topology.instance_by_name("q_0_master").is_some());
    }
}
