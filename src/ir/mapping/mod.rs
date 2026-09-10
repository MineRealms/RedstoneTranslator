//! General Logical-to-Routable lowering.
//!
//! This module is the target- and policy-aware lowering path. It handles flat
//! logical modules: buses are bit-blasted, combinational operations are
//! expanded per bit (including ripple-carry add and increment), and state
//! cells are decomposed into target-supported primitives. It shares the
//! leaf/composite writers with the legacy special-case lowering in
//! `logical_lowering.rs`, which keeps the historical counter and DFF
//! structures stable while this path grows.
//!
//! Known limits of this first slice:
//!
//! - constants are not supported in scalar cones yet,
//! - hierarchy is still handled by the legacy path (children must be scalar),
//! - one combinational leaf per state cell; a future partitioning step should
//!   split leaves that exceed the local placer node limit.

mod combinational;
mod scalar;
mod sequential;

use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet};

use eyre::ContextCompat;
use scalar::{LeafBuilder, ScalarNets};

/// Node budget for one partitioned leaf. The local placer hard limit is 40
/// graph nodes, so a chunk is kept below it with room for generated ports.
pub(super) const LEAF_NODE_BUDGET: usize = 28;

use super::target::{MappingPolicy, TargetSpec};
use super::{
    Endpoint, LogicalCellKind, LogicalModule, LogicalPortDirection, LogicalValue, NetClass,
    RoutableDesign, RoutableInstance, RoutableModule, RoutableModuleBody, RoutableNet,
    RoutableNodeKind, RoutablePort, RoutablePortDirection, ROUTABLE_IR_VERSION,
};
use crate::graph::logic::LogicGraph;
use crate::graph::Graph;
use crate::logic::LogicType;
use crate::sequential::{SequentialPrimitive, SequentialType};

/// Lowers one flat logical module (no instances) with the general mapper.
pub(super) fn lower_flat_module(
    module: &LogicalModule,
    target: &TargetSpec,
    policy: &MappingPolicy,
) -> eyre::Result<RoutableDesign> {
    let nets = ScalarNets::from_module(module);
    let state_cells = module
        .cells
        .iter()
        .filter(|cell| cell.kind.is_sequential())
        .collect::<Vec<_>>();
    if state_cells.is_empty() {
        lower_combinational_module(module, &nets, target, policy)
    } else {
        sequential::lower_sequential_module(module, &nets, &state_cells, target, policy)
    }
}

fn lower_combinational_module(
    module: &LogicalModule,
    nets: &ScalarNets,
    target: &TargetSpec,
    policy: &MappingPolicy,
) -> eyre::Result<RoutableDesign> {
    let mut builder = LeafBuilder::new();
    for port in module
        .ports
        .iter()
        .filter(|port| port.direction == LogicalPortDirection::Input)
    {
        for scalar in nets.bits(&port.net)? {
            builder.add_input(&scalar);
        }
    }

    let mut pending = module.cells.iter().collect::<Vec<_>>();
    while !pending.is_empty() {
        let before = pending.len();
        let mut unresolved = Vec::new();
        for cell in pending {
            if !combinational::emit_cell(cell, nets, &mut builder, target, policy)? {
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
        for scalar in nets.bits(&port.net)? {
            let producer = builder
                .producer(&scalar)
                .with_context(|| format!("output port `{}` has no logical producer", port.name))?;
            builder.add_output(&scalar, producer);
        }
    }

    let partitioned = builder.partition(&module.name, LEAF_NODE_BUDGET)?;
    if partitioned.instances.len() == 1 && partitioned.direct_outputs.is_empty() {
        return finish_design(&module.name, target.name(), partitioned.modules);
    }

    let mut connections = NetConnections::default();
    let mut used_inputs = BTreeSet::new();
    let mut instance_names = Vec::new();
    for leaf in &partitioned.instances {
        instance_names.push(leaf.instance.clone());
        for input in &leaf.inputs {
            let endpoint = if let Some(producer) = partitioned.intermediates.get(input) {
                instance_port(producer, input)
            } else {
                used_inputs.insert(input.clone());
                self_port(input)
            };
            connections.connect(
                input,
                classify_logical_net(input, false),
                endpoint,
                instance_port(&leaf.instance, input),
            );
        }
    }
    for port in module
        .ports
        .iter()
        .filter(|port| port.direction == LogicalPortDirection::Output)
    {
        for bit in 0..nets.width(&port.net)? {
            let scalar = nets.scalar(&port.net, bit)?;
            let driver =
                if let Some((instance, port_name)) = partitioned.output_sources.get(&scalar) {
                    instance_port(instance, port_name)
                } else if let Some(source) = partitioned.direct_outputs.get(&scalar) {
                    used_inputs.insert(source.clone());
                    self_port(source)
                } else {
                    eyre::bail!("output `{scalar}` was not produced by the partitioned cone");
                };
            connections.connect(&scalar, NetClass::Io, driver, self_port(&scalar));
        }
    }
    let ports = logical_ports_filtered(module, nets, &used_inputs)?;
    let top = composite_module(
        &module.name,
        ports,
        instance_names.iter(),
        connections.finish(),
    );
    let mut modules = partitioned.modules;
    modules.push(top);
    finish_design(&module.name, target.name(), modules)
}

/// Routable ports for a logical module, dropping unused input bits.
fn logical_ports_filtered(
    module: &LogicalModule,
    nets: &ScalarNets,
    used_inputs: &BTreeSet<String>,
) -> eyre::Result<Vec<RoutablePort>> {
    let mut ports = Vec::new();
    for port in &module.ports {
        for bit in 0..nets.width(&port.net)? {
            let scalar = nets.scalar(&port.net, bit)?;
            match port.direction {
                LogicalPortDirection::Input => {
                    if used_inputs.contains(&scalar) {
                        ports.push(RoutablePort {
                            name: scalar,
                            direction: RoutablePortDirection::Input,
                        });
                    }
                }
                LogicalPortDirection::Output => ports.push(RoutablePort {
                    name: scalar,
                    direction: RoutablePortDirection::Output,
                }),
            }
        }
    }
    Ok(ports)
}

/// True when the legacy single-state special case covers the whole module.
///
/// The legacy path only wires the state's next-state cone and its direct
/// output, so it must not run for modules with extra outputs or extra
/// combinational logic. When this returns false the general mapper takes over.
pub(super) fn module_is_simple_state_design(module: &LogicalModule) -> bool {
    let state_cells = module
        .cells
        .iter()
        .filter(|cell| cell.kind.is_sequential())
        .collect::<Vec<_>>();
    let [state] = state_cells.as_slice() else {
        return false;
    };
    let state_outputs = state
        .outputs
        .iter()
        .map(|output| output.net.as_str())
        .collect::<HashSet<_>>();
    if !module
        .ports
        .iter()
        .filter(|port| port.direction == LogicalPortDirection::Output)
        .all(|port| state_outputs.contains(port.net.as_str()))
    {
        return false;
    }

    let mut driver_of = HashMap::new();
    for cell in &module.cells {
        if let Some(output) = cell.outputs.first() {
            driver_of.insert(output.net.as_str(), cell);
        }
    }
    let Some(data) = state.input_value("d").ok().and_then(|value| match value {
        LogicalValue::Net { net } => Some(net.as_str()),
        LogicalValue::Slice { .. } | LogicalValue::Constant { .. } => None,
    }) else {
        return false;
    };

    let mut seen_nets = HashSet::new();
    let mut cone_cells = HashSet::new();
    let mut stack = vec![data];
    while let Some(net) = stack.pop() {
        if !seen_nets.insert(net) {
            continue;
        }
        let Some(cell) = driver_of.get(net).copied() else {
            continue;
        };
        if cell.kind.is_sequential() {
            continue;
        }
        cone_cells.insert(cell.name.as_str());
        for input in &cell.inputs {
            if let LogicalValue::Net { net } = &input.value {
                stack.push(net.as_str());
            }
        }
    }
    module
        .cells
        .iter()
        .filter(|cell| !cell.kind.is_sequential())
        .all(|cell| cone_cells.contains(cell.name.as_str()))
}

/// True when the module needs capabilities the legacy scalar leaf writer lacks.
pub(super) fn requires_general_flat_lowering(module: &LogicalModule) -> bool {
    module.nets.iter().any(|net| net.width != 1)
        || module
            .cells
            .iter()
            .filter(|cell| cell.kind.is_sequential())
            .count()
            >= 2
        || module.cells.iter().any(|cell| {
            matches!(
                cell.kind,
                LogicalCellKind::Add
                    | LogicalCellKind::Inc
                    | LogicalCellKind::Eq { .. }
                    | LogicalCellKind::Mux
                    | LogicalCellKind::Dff { .. }
                    | LogicalCellKind::Register { .. }
            )
        })
}

/// Deterministic Routable design assembly shared by every lowering path.
pub(super) fn finish_design(
    top: &str,
    target: &str,
    modules: Vec<RoutableModule>,
) -> eyre::Result<RoutableDesign> {
    let mut canonical = HashMap::<String, String>::new();
    let mut deduplicated = Vec::<RoutableModule>::new();
    for module in modules {
        let existing = if module.name != top
            && matches!(module.body, RoutableModuleBody::Leaf { .. })
        {
            deduplicated
                .iter()
                .find(|candidate| candidate.ports == module.ports && candidate.body == module.body)
                .map(|candidate| candidate.name.clone())
        } else {
            None
        };
        if let Some(existing) = existing {
            canonical.insert(module.name, existing);
        } else {
            canonical.insert(module.name.clone(), module.name.clone());
            deduplicated.push(module);
        }
    }
    for module in &mut deduplicated {
        if let RoutableModuleBody::Composite { instances, .. } = &mut module.body {
            for instance in instances {
                if let Some(name) = canonical.get(&instance.module) {
                    instance.module = name.clone();
                }
            }
        }
    }
    deduplicated.sort_by(|left, right| left.name.cmp(&right.name));
    let design = RoutableDesign {
        version: ROUTABLE_IR_VERSION,
        target: target.to_owned(),
        top: top.to_owned(),
        modules: deduplicated,
        debug: Default::default(),
    };
    design.validate()?;
    Ok(design)
}

/// Builds a leaf module from a graph, one-to-one with `routable_leaf_from_graph`.
pub(super) fn routable_leaf_from_graph(name: &str, graph: Graph) -> eyre::Result<RoutableModule> {
    let ports = graph
        .nodes
        .iter()
        .filter_map(|node| match &node.kind {
            crate::graph::GraphNodeKind::Input(name) => Some(RoutablePort {
                name: name.clone(),
                direction: RoutablePortDirection::Input,
            }),
            crate::graph::GraphNodeKind::Output(name) => Some(RoutablePort {
                name: name.clone(),
                direction: RoutablePortDirection::Output,
            }),
            _ => None,
        })
        .collect();
    let nodes = graph
        .nodes
        .into_iter()
        .map(|node| {
            let kind = match &node.kind {
                crate::graph::GraphNodeKind::Input(name) => {
                    RoutableNodeKind::Input { name: name.clone() }
                }
                crate::graph::GraphNodeKind::Constant(value) => {
                    RoutableNodeKind::Constant { value: *value }
                }
                crate::graph::GraphNodeKind::Output(name) => {
                    RoutableNodeKind::Output { name: name.clone() }
                }
                crate::graph::GraphNodeKind::Logic(logic) => match logic.logic_type {
                    LogicType::Not => RoutableNodeKind::Not,
                    LogicType::And => RoutableNodeKind::And,
                    LogicType::Or => RoutableNodeKind::Or,
                    LogicType::Xor => RoutableNodeKind::Xor,
                },
                crate::graph::GraphNodeKind::Sequential(sequential) => {
                    RoutableNodeKind::Sequential {
                        primitive: match sequential.sequential_type {
                            SequentialType::RsLatch => super::RoutableSequentialPrimitive::RsLatch,
                            SequentialType::DLatch => super::RoutableSequentialPrimitive::DLatch,
                        },
                        input_ports: sequential.input_ports.clone(),
                        output_ports: sequential.output_ports.clone(),
                    }
                }
                crate::graph::GraphNodeKind::None => {
                    eyre::bail!("routable leaf contains unresolved node")
                }
                crate::graph::GraphNodeKind::Block(_) => {
                    eyre::bail!("routable leaf contains physical block node")
                }
                crate::graph::GraphNodeKind::Clustered(_) => {
                    eyre::bail!("routable leaf contains clustered node")
                }
            };
            Ok(super::RoutableNode {
                id: node.id,
                kind,
                inputs: node.inputs.clone(),
                tag: node.tag.clone(),
            })
        })
        .collect::<eyre::Result<Vec<_>>>()?;
    Ok(RoutableModule {
        name: name.to_owned(),
        ports,
        body: RoutableModuleBody::Leaf { nodes },
    })
}

/// Routable ports for a logical module; vector ports are bit-blasted.
pub(super) fn logical_ports(module: &LogicalModule) -> eyre::Result<Vec<RoutablePort>> {
    let mut ports = Vec::new();
    for port in &module.ports {
        let width = module
            .nets
            .iter()
            .find(|net| net.name == port.net)
            .with_context(|| format!("port `{}` references missing net `{}`", port.name, port.net))?
            .width;
        let direction = match port.direction {
            LogicalPortDirection::Input => RoutablePortDirection::Input,
            LogicalPortDirection::Output => RoutablePortDirection::Output,
        };
        if width == 1 {
            ports.push(RoutablePort {
                name: port.name.clone(),
                direction,
            });
        } else {
            ports.extend((0..width).map(|bit| RoutablePort {
                name: bit_signal_name(&port.name, bit),
                direction,
            }));
        }
    }
    Ok(ports)
}

pub(super) fn bit_signal_name(signal: &str, bit: usize) -> String {
    format!("{signal}_{bit}")
}

pub(super) fn composite_module<I, S>(
    name: &str,
    ports: Vec<RoutablePort>,
    instance_names: I,
    nets: Vec<RoutableNet>,
) -> RoutableModule
where
    I: IntoIterator<Item = S>,
    S: AsRef<str>,
{
    RoutableModule {
        name: name.to_owned(),
        ports,
        body: RoutableModuleBody::Composite {
            instances: instance_names
                .into_iter()
                .map(|name| RoutableInstance {
                    name: name.as_ref().to_owned(),
                    module: name.as_ref().to_owned(),
                    origin: None,
                })
                .collect(),
            nets,
        },
    }
}

/// A leaf module with `~clk` as its only logic.
pub(super) fn not_clock_module(name: &str) -> eyre::Result<RoutableModule> {
    combinational_output_module(name, "~clk", "clk_n")
}

/// A leaf module whose output is one combinational expression.
pub(super) fn combinational_output_module(
    name: &str,
    expr: &str,
    output: &str,
) -> eyre::Result<RoutableModule> {
    routable_leaf_from_graph(name, LogicGraph::from_stmt(expr, output)?.graph)
}

/// A leaf module with one D-latch node exposing `d`, `en`, and `q`.
pub(super) fn d_latch_routable_module(name: &str) -> RoutableModule {
    let mut graph = Graph::from_nodes(vec![
        crate::graph::GraphNode {
            kind: crate::graph::GraphNodeKind::Input("d".to_owned()),
            ..Default::default()
        },
        crate::graph::GraphNode {
            kind: crate::graph::GraphNodeKind::Input("en".to_owned()),
            ..Default::default()
        },
        crate::graph::GraphNode {
            kind: crate::graph::GraphNodeKind::Sequential(SequentialPrimitive::new(
                SequentialType::DLatch,
                vec!["d".to_owned(), "en".to_owned()],
                vec!["q".to_owned()],
            )),
            inputs: vec![0, 1],
            ..Default::default()
        },
        crate::graph::GraphNode {
            kind: crate::graph::GraphNodeKind::Output("q".to_owned()),
            inputs: vec![2],
            ..Default::default()
        },
    ]);
    graph.build_outputs();
    graph.build_producers();
    graph.build_consumers();
    graph
        .verify()
        .expect("built-in D latch graph must be valid");
    routable_leaf_from_graph(name, graph).expect("built-in D latch must lower to Routable IR")
}

pub(super) fn single_endpoint(endpoints: Option<&Vec<Endpoint>>) -> Option<Endpoint> {
    let endpoints = endpoints?;
    (endpoints.len() == 1).then(|| endpoints[0].clone())
}

pub(super) fn self_port(port: &str) -> Endpoint {
    Endpoint::SelfPort {
        port: port.to_owned(),
    }
}

pub(super) fn instance_port(instance: &str, port: &str) -> Endpoint {
    Endpoint::InstancePort {
        instance: instance.to_owned(),
        port: port.to_owned(),
    }
}

pub(super) fn classify_logical_net(name: &str, is_io: bool) -> NetClass {
    if is_io {
        NetClass::Io
    } else if name.contains("clk") || name.contains("clock") {
        NetClass::Clock
    } else if name.contains("reset") || name.starts_with("rst") {
        NetClass::Reset
    } else {
        NetClass::Data
    }
}

/// Accumulates nets by driver so one endpoint never drives two nets.
#[derive(Default)]
pub(super) struct NetConnections {
    by_driver: BTreeMap<Endpoint, (String, NetClass, Vec<Endpoint>)>,
}

impl NetConnections {
    pub(super) fn connect(
        &mut self,
        preferred_name: &str,
        class: NetClass,
        driver: Endpoint,
        sink: Endpoint,
    ) {
        let entry = self
            .by_driver
            .entry(driver)
            .or_insert_with(|| (preferred_name.to_owned(), class, Vec::new()));
        if !entry.2.contains(&sink) {
            entry.2.push(sink);
        }
        if class == NetClass::Io {
            entry.1 = NetClass::Io;
        }
    }

    pub(super) fn finish(self) -> Vec<RoutableNet> {
        let mut used = HashSet::new();
        self.by_driver
            .into_iter()
            .map(|(driver, (preferred, class, sinks))| {
                let mut name = preferred.clone();
                let mut suffix = 1;
                while !used.insert(name.clone()) {
                    name = format!("{preferred}_{suffix}");
                    suffix += 1;
                }
                RoutableNet {
                    name,
                    class,
                    driver,
                    sinks,
                    origin: None,
                }
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::graph::logic::LogicTruthTable;

    fn lower(source: &str) -> RoutableDesign {
        crate::ir::LogicalDesign::from_verilog_source(source)
            .expect("test source must parse")
            .lower_to_routable()
            .expect("test design must lower")
    }

    fn lower_rcir(source: &str) -> RoutableDesign {
        source
            .parse::<crate::ir::LogicalDesign>()
            .expect("test rcir must parse")
            .lower_to_routable()
            .expect("test design must lower")
    }

    fn leaf_truth_table(design: &RoutableDesign, module: &str) -> LogicTruthTable {
        let leaf = design.module(module).expect("leaf module");
        let graph = crate::ir::graph_from_routable_leaf(leaf).expect("leaf graph");
        LogicGraph { graph }
            .truth_table()
            .expect("truth table must build")
    }

    #[test]
    fn vector_and_is_bit_blasted_and_matches_truth_table() {
        let design = lower(
            r#"
            module m(a, b, y);
              input [3:0] a, b;
              output [3:0] y;
              assign y = a & b;
            endmodule
            "#,
        );
        let table = leaf_truth_table(&design, "m");

        assert_eq!(
            table.input_names,
            ["a_0", "a_1", "a_2", "a_3", "b_0", "b_1", "b_2", "b_3"]
        );
        for mask in 0..(1usize << table.input_names.len()) {
            for bit in 0..4 {
                let a = (mask >> bit) & 1 == 1;
                let b = (mask >> (4 + bit)) & 1 == 1;
                assert_eq!(
                    table.output_tables[&format!("y_{bit}")][mask],
                    a && b,
                    "mask {mask} bit {bit}"
                );
            }
        }
    }

    #[test]
    fn vector_add_lowers_to_ripple_carry_and_matches_truth_table() {
        let design = lower(
            r#"
            module m(a, b, y);
              input [3:0] a, b;
              output [3:0] y;
              assign y = a + b;
            endmodule
            "#,
        );
        let table = leaf_truth_table(&design, "m");

        for mask in 0..(1usize << table.input_names.len()) {
            let mut a = 0usize;
            let mut b = 0usize;
            for bit in 0..4 {
                if (mask >> bit) & 1 == 1 {
                    a |= 1 << bit;
                }
                if (mask >> (4 + bit)) & 1 == 1 {
                    b |= 1 << bit;
                }
            }
            let expected = (a + b) & 0xF;
            for bit in 0..4 {
                assert_eq!(
                    table.output_tables[&format!("y_{bit}")][mask],
                    (expected >> bit) & 1 == 1,
                    "mask {mask} bit {bit}"
                );
            }
        }
    }

    #[test]
    fn combinational_mux_lowers_to_and_or_not_and_matches_truth_table() {
        let design = lower_rcir(
            r#"
rcir 1;
stage logical;
top m;
module m {
  port input  s : bit;
  port input  a : bits[2];
  port input  b : bits[2];
  port output y : bits[2];
  cell choose : logical.mux<2> {
    in select = s;
    in when_true = a;
    in when_false = b;
    out result = y;
  }
}
"#,
        );
        let leaf = design.module("m").expect("leaf module");
        let RoutableModuleBody::Leaf { nodes } = &leaf.body else {
            panic!("mux design must lower to a leaf");
        };
        assert!(!nodes
            .iter()
            .any(|node| matches!(node.kind, RoutableNodeKind::Xor)));
        assert!(nodes
            .iter()
            .any(|node| matches!(node.kind, RoutableNodeKind::And)));
        assert!(nodes
            .iter()
            .any(|node| matches!(node.kind, RoutableNodeKind::Or)));
        assert!(nodes
            .iter()
            .any(|node| matches!(node.kind, RoutableNodeKind::Not)));

        let table = leaf_truth_table(&design, "m");
        assert_eq!(table.input_names, ["a_0", "a_1", "b_0", "b_1", "s"]);
        for mask in 0..(1usize << table.input_names.len()) {
            let select = (mask >> 4) & 1 == 1;
            for bit in 0..2 {
                let a = (mask >> bit) & 1 == 1;
                let b = (mask >> (2 + bit)) & 1 == 1;
                assert_eq!(
                    table.output_tables[&format!("y_{bit}")][mask],
                    if select { a } else { b },
                    "mask {mask} bit {bit}"
                );
            }
        }
    }

    #[test]
    fn and_or_not_xor_policy_removes_native_xor_nodes() {
        let logical = crate::ir::LogicalDesign::from_verilog_source(
            r#"
            module m(a, b, y);
              input [1:0] a, b;
              output [1:0] y;
              assign y = a ^ b;
            endmodule
            "#,
        )
        .expect("test source must parse");
        let policy = MappingPolicy {
            xor: crate::ir::XorMapping::AndOrNot,
            ..MappingPolicy::default()
        };
        let design = logical
            .lower_to_routable_with_target(&TargetSpec::redstone_v1(), &policy)
            .expect("policy variant must lower");
        let leaf = design.module("m").expect("leaf module");
        let RoutableModuleBody::Leaf { nodes } = &leaf.body else {
            panic!("xor design must lower to a leaf");
        };
        assert!(!nodes
            .iter()
            .any(|node| matches!(node.kind, RoutableNodeKind::Xor)));
        assert!(nodes
            .iter()
            .any(|node| matches!(node.kind, RoutableNodeKind::And)));
    }

    #[test]
    fn constant_operands_lower_and_match_truth_tables() {
        let design = lower_rcir(
            r#"
rcir 1;
stage logical;
top m;
module m {
  port input  a : bit;
  port output y : bit;
  cell g : logical.and<1> {
    in lhs = a;
    in rhs = const<1>(1);
    out result = y;
  }
}
"#,
        );
        let leaf = design.module("m").expect("leaf module");
        let RoutableModuleBody::Leaf { nodes } = &leaf.body else {
            panic!("constant design must lower to a leaf");
        };
        assert!(nodes
            .iter()
            .any(|node| matches!(node.kind, RoutableNodeKind::Constant { value: true })));

        let table = leaf_truth_table(&design, "m");
        assert_eq!(table.input_names, ["a"]);
        for mask in 0..2 {
            assert_eq!(table.output_tables["y"][mask], mask == 1, "mask {mask}");
        }
    }

    #[test]
    fn constant_zero_is_expanded_through_an_inverter() {
        let design = lower_rcir(
            r#"
rcir 1;
stage logical;
top m;
module m {
  port input  a : bit;
  port output y : bit;
  cell g : logical.and<1> {
    in lhs = a;
    in rhs = const<1>(0);
    out result = y;
  }
}
"#,
        );
        let leaf = design.module("m").expect("leaf module");
        let RoutableModuleBody::Leaf { nodes } = &leaf.body else {
            panic!("constant design must lower to a leaf");
        };
        assert!(nodes
            .iter()
            .any(|node| matches!(node.kind, RoutableNodeKind::Constant { value: true })));
        assert!(nodes
            .iter()
            .any(|node| matches!(node.kind, RoutableNodeKind::Not)));

        let table = leaf_truth_table(&design, "m");
        assert!(table.output_tables["y"].iter().all(|value| !value));
    }

    #[test]
    fn register_reset_constant_lowers_to_a_next_leaf_with_a_constant() {
        let logical: crate::ir::LogicalDesign = r#"
rcir 1;
stage logical;
top m;
module m {
  port input  clk : bit;
  port output q : bit;
  cell state : logical.dff<1> {
    in d = const<1>(0);
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
            .expect("constant register data must lower");

        let next = design.module("q_next").expect("next leaf");
        let RoutableModuleBody::Leaf { nodes } = &next.body else {
            panic!("next-state module must be a leaf");
        };
        assert!(nodes
            .iter()
            .any(|node| matches!(node.kind, RoutableNodeKind::Constant { .. })));

        let topology =
            crate::transform::place_and_route::global_pnr::topology::ResolvedPnrTopology::from_routable(
                &design,
            )
            .expect("constant register design must resolve to a PnR topology");
        assert!(topology.instance_by_name("q_master").is_some());
    }

    #[test]
    fn local_placer_places_constant_one_as_a_redstone_block() {
        use crate::graph::{Graph, GraphNode, GraphNodeKind};
        use crate::transform::place_and_route::local_placer::{LocalPlacer, LocalPlacerConfig};
        use crate::world::position::DimSize;

        let mut graph = Graph::from_nodes(vec![
            GraphNode {
                kind: GraphNodeKind::Constant(true),
                ..Default::default()
            },
            GraphNode {
                kind: GraphNodeKind::Output("y".to_owned()),
                inputs: vec![0],
                ..Default::default()
            },
        ]);
        graph.build_outputs();
        graph.build_producers();
        graph.build_consumers();

        let config = LocalPlacerConfig {
            materialize_outputs: true,
            ..Default::default()
        };
        let placer =
            LocalPlacer::new(LogicGraph { graph }, config).expect("constant graph must verify");
        let worlds = placer.generate_with_outputs(DimSize(4, 4, 3), None);

        assert!(
            !worlds.is_empty(),
            "constant placement must produce a world"
        );
        assert!(worlds
            .iter()
            .any(|placed| placed
                .world
                .iter_block()
                .into_iter()
                .any(|(_, block)| matches!(
                    block.kind,
                    crate::world::block::BlockKind::RedstoneBlock
                ))));
    }

    #[test]
    fn bit_select_of_a_declared_vector_lowers_and_matches_truth_table() {
        let design = lower(
            r#"
            module m(a, y);
              input [3:0] a;
              output y;
              assign y = ~a[2];
            endmodule
            "#,
        );
        let table = leaf_truth_table(&design, "m");
        // The partition keeps only the boundary bits the cone actually uses.
        assert_eq!(table.input_names, ["a_2"]);
        for mask in 0..2 {
            assert_eq!(table.output_tables["y"][mask], mask == 0, "mask {mask}");
        }
    }

    #[test]
    fn clocked_register_can_capture_a_vector_bit() {
        let design = lower(
            r#"
            module m(clk, a, q);
              input clk;
              input [3:0] a;
              output reg q;
              always @(posedge clk) begin
                q <= a[1];
              end
            endmodule
            "#,
        );
        let topology =
            crate::transform::place_and_route::global_pnr::topology::ResolvedPnrTopology::from_routable(
                &design,
            )
            .expect("slice register must resolve to a PnR topology");
        assert!(topology.instance_by_name("q_master").is_some());
    }

    #[test]
    fn equality_operator_lowers_through_eq_cells() {
        let design = lower(
            r#"
            module m(a, b, y);
              input [1:0] a, b;
              output y;
              assign y = a == b;
            endmodule
            "#,
        );
        let table = leaf_truth_table(&design, "m");
        assert_eq!(table.input_names, ["a_0", "a_1", "b_0", "b_1"]);
        for mask in 0..16 {
            let a = mask & 0b11;
            let b = (mask >> 2) & 0b11;
            assert_eq!(table.output_tables["y"][mask], a == b, "mask {mask}");
        }
    }

    #[test]
    fn clocked_if_with_equality_and_constants_lowers_to_a_pnr_topology() {
        let design = lower(
            r#"
            module m(clk, a, b, q);
              input clk;
              input [1:0] a, b;
              output reg q;
              always @(posedge clk) begin
                if (a == b) begin
                  q <= 1;
                end else begin
                  q <= 0;
                end
              end
            endmodule
            "#,
        );
        let topology =
            crate::transform::place_and_route::global_pnr::topology::ResolvedPnrTopology::from_routable(
                &design,
            )
            .expect("equality dff must resolve to a PnR topology");
        assert!(topology.instance_by_name("q_master").is_some());
    }

    #[test]
    fn nested_hierarchy_flattens_and_lowers() {
        let design = lower(
            r#"
            module inv(a, y);
              input a;
              output y;
              assign y = ~a;
            endmodule

            module pair(a, y);
              input a;
              output y;
              wire mid;
              inv u0(.a(a), .y(mid));
              inv u1(.a(mid), .y(y));
            endmodule

            module top(a, y);
              input a;
              output y;
              pair p(.a(a), .y(y));
            endmodule
            "#,
        );

        let table = leaf_truth_table(&design, "top");
        assert_eq!(table.input_names, ["a"]);
        assert_eq!(table.output_tables["y"], vec![false, true]);
    }

    #[test]
    fn hierarchy_with_vector_ports_flattens_and_lowers() {
        let design = lower(
            r#"
            module xor2(a, b, y);
              input [1:0] a, b;
              output [1:0] y;
              assign y = a ^ b;
            endmodule

            module top(a, b, y);
              input [1:0] a, b;
              output [1:0] y;
              xor2 u(.a(a), .b(b), .y(y));
            endmodule
            "#,
        );

        let table = leaf_truth_table(&design, "top");
        assert_eq!(table.input_names, ["a_0", "a_1", "b_0", "b_1"]);
        for mask in 0..16 {
            for bit in 0..2 {
                let a = (mask >> bit) & 1 == 1;
                let b = (mask >> (2 + bit)) & 1 == 1;
                assert_eq!(
                    table.output_tables[&format!("y_{bit}")][mask],
                    a ^ b,
                    "mask {mask} bit {bit}"
                );
            }
        }
    }

    #[test]
    fn fsm_with_case_and_combinational_output_lowers_to_a_pnr_topology() {
        let design = lower(
            r#"
            module fsm(clk, go, state, done);
              input clk, go;
              output reg [1:0] state;
              output reg done;
              always @(posedge clk) begin
                case (state)
                  0: begin if (go) begin state <= 1; end end
                  1, 2: state <= 3;
                  default: state <= 0;
                endcase
              end
              always @(*) begin
                case (state)
                  3: done <= 1;
                  default: done <= 0;
                endcase
              end
            endmodule
            "#,
        );

        let top = design.module("fsm").expect("top module");
        assert!(matches!(top.body, RoutableModuleBody::Composite { .. }));
        for module in &design.modules {
            if let RoutableModuleBody::Leaf { nodes } = &module.body {
                assert!(
                    nodes.len() <= 40,
                    "leaf `{}` has {} nodes, above the local placer limit",
                    module.name,
                    nodes.len()
                );
            }
        }

        let topology =
            crate::transform::place_and_route::global_pnr::topology::ResolvedPnrTopology::from_routable(
                &design,
            )
            .expect("fsm must resolve to a PnR topology");
        for expected in ["state_0_master", "state_0_slave", "fsm_out"] {
            assert!(
                topology.instance_by_name(expected).is_some(),
                "missing resolved instance `{expected}`"
            );
        }
    }

    #[test]
    fn wide_combinational_add_is_partitioned_into_a_composite() {
        let design = lower(
            r#"
            module m(a, b, y);
              input [31:0] a, b;
              output [31:0] y;
              assign y = a + b;
            endmodule
            "#,
        );
        let top = design.module("m").expect("top module");
        assert!(matches!(top.body, RoutableModuleBody::Composite { .. }));
        for module in &design.modules {
            if let RoutableModuleBody::Leaf { nodes } = &module.body {
                assert!(
                    nodes.len() <= 40,
                    "leaf `{}` has {} nodes, above the local placer limit",
                    module.name,
                    nodes.len()
                );
            }
        }
        let topology =
            crate::transform::place_and_route::global_pnr::topology::ResolvedPnrTopology::from_routable(
                &design,
            )
            .expect("partitioned design must resolve to a PnR topology");
        assert!(topology.instance_by_name("m_p0").is_some());
        assert!(topology.instance_by_name("m_p1").is_some());
    }
}
