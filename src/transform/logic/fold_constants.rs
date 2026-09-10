//! Constant folding for prepared logic graphs.
//!
//! The mapper materializes constants as `Constant(true)` and
//! `Not(Constant(true))` (false). Arithmetic, muxes, and case decoding then
//! produce identity and absorbing operations such as `and(x, 1)` or
//! `or(x, 0)`. Those patterns both waste blocks and create nodes the local
//! placer cannot route (for example an Or fed directly by a redstone block),
//! so this pass folds them to a canonical form before decomposition.
//!
//! Canonical constants stay in the physical representation the placer
//! understands: true is `Constant(true)`, false is `Not(Constant(true))`.
//! `Constant(false)` is never produced.

use crate::graph::{GraphNode, GraphNodeId, GraphNodeKind};
use crate::logic::{Logic, LogicType};

use super::LogicGraphTransformer;

impl LogicGraphTransformer {
    pub fn fold_constants(&mut self) -> eyre::Result<()> {
        // Each pass performs one rewrite and restarts; the number of nodes is
        // small and this keeps id bookkeeping simple.
        for _ in 0..1024 {
            if !self.fold_one_rewrite()? {
                break;
            }
        }
        self.rebuild_after_fold();
        self.graph.graph.verify()?;
        Ok(())
    }

    fn fold_one_rewrite(&mut self) -> eyre::Result<bool> {
        let node_ids = self.graph.graph.nodes.iter().map(|node| node.id).collect::<Vec<_>>();
        for id in node_ids {
            if self.fold_node(id)? {
                return Ok(true);
            }
        }
        Ok(false)
    }

    fn fold_node(&mut self, id: GraphNodeId) -> eyre::Result<bool> {
        let Some(node) = self.graph.graph.find_node_by_id(id) else {
            return Ok(false);
        };
        let GraphNodeKind::Logic(logic) = &node.kind else {
            return Ok(false);
        };
        let logic_type = logic.logic_type;
        let inputs = node.inputs.clone();

        match logic_type {
            LogicType::Not => {
                if inputs.len() == 1
                    && let Some(value) = self.constant_value(inputs[0])
                {
                    let target = self.materialize_constant(!value);
                    if target != id {
                        self.replace_node(id, target);
                        return Ok(true);
                    }
                }
            }
            LogicType::And => {
                if inputs
                    .iter()
                    .any(|input| self.constant_value(*input) == Some(false))
                {
                    let target = self.materialize_constant(false);
                    self.replace_node(id, target);
                    return Ok(true);
                }
                if let Some(true_input) = inputs
                    .iter()
                    .find(|input| self.constant_value(**input) == Some(true))
                    .copied()
                {
                    let remaining = inputs
                        .iter()
                        .copied()
                        .filter(|input| *input != true_input)
                        .collect::<Vec<_>>();
                    match remaining.len() {
                        0 => {
                            let target = self.materialize_constant(true);
                            if target != id {
                                self.replace_node(id, target);
                            } else {
                                return Ok(false);
                            }
                        }
                        1 => self.replace_node(id, remaining[0]),
                        _ => {
                            self.graph
                                .graph
                                .replace_target_input_node_ids(id, true_input, Vec::new());
                            self.remove_unused_node(true_input);
                            self.rebuild_after_fold();
                        }
                    }
                    return Ok(true);
                }
            }
            LogicType::Or => {
                if inputs
                    .iter()
                    .any(|input| self.constant_value(*input) == Some(true))
                {
                    let target = self.materialize_constant(true);
                    self.replace_node(id, target);
                    return Ok(true);
                }
                if let Some(false_input) = inputs
                    .iter()
                    .find(|input| self.constant_value(**input) == Some(false))
                    .copied()
                {
                    let remaining = inputs
                        .iter()
                        .copied()
                        .filter(|input| *input != false_input)
                        .collect::<Vec<_>>();
                    match remaining.len() {
                        0 => {
                            let target = self.materialize_constant(false);
                            if target != id {
                                self.replace_node(id, target);
                            } else {
                                return Ok(false);
                            }
                        }
                        1 => self.replace_node(id, remaining[0]),
                        _ => {
                            self.graph
                                .graph
                                .replace_target_input_node_ids(id, false_input, Vec::new());
                            self.remove_unused_node(false_input);
                            self.rebuild_after_fold();
                        }
                    }
                    return Ok(true);
                }
            }
            LogicType::Xor => {
                if let Some(false_input) = inputs
                    .iter()
                    .find(|input| self.constant_value(**input) == Some(false))
                    .copied()
                {
                    let remaining = inputs
                        .iter()
                        .copied()
                        .filter(|input| *input != false_input)
                        .collect::<Vec<_>>();
                    match remaining.len() {
                        0 => {
                            let target = self.materialize_constant(false);
                            if target != id {
                                self.replace_node(id, target);
                            } else {
                                return Ok(false);
                            }
                        }
                        1 => self.replace_node(id, remaining[0]),
                        _ => {
                            self.graph
                                .graph
                                .replace_target_input_node_ids(id, false_input, Vec::new());
                            self.remove_unused_node(false_input);
                            self.rebuild_after_fold();
                        }
                    }
                    return Ok(true);
                }
                let true_count = inputs
                    .iter()
                    .filter(|input| self.constant_value(**input) == Some(true))
                    .count();
                let non_constant = inputs
                    .iter()
                    .copied()
                    .filter(|input| self.constant_value(*input).is_none())
                    .collect::<Vec<_>>();
                if true_count > 0 && non_constant.len() == 1 {
                    let value = non_constant[0];
                    if true_count % 2 == 1 {
                        let inverted = self.graph.graph.add_node(GraphNode {
                            kind: GraphNodeKind::Logic(Logic {
                                logic_type: LogicType::Not,
                            }),
                            inputs: vec![value],
                            tag: "folded-xor".to_owned(),
                            ..Default::default()
                        });
                        self.replace_node(id, inverted);
                    } else {
                        self.replace_node(id, value);
                    }
                    return Ok(true);
                }
            }
        }

        Ok(false)
    }

    /// Value of a canonical constant node, if it is one.
    fn constant_value(&self, id: GraphNodeId) -> Option<bool> {
        let node = self.graph.graph.find_node_by_id(id)?;
        match &node.kind {
            GraphNodeKind::Constant(value) => Some(*value),
            GraphNodeKind::Logic(logic)
                if logic.logic_type == LogicType::Not && node.inputs.len() == 1 =>
            {
                match self.graph.graph.find_node_by_id(node.inputs[0])?.kind {
                    GraphNodeKind::Constant(true) => Some(false),
                    _ => None,
                }
            }
            _ => None,
        }
    }

    /// Returns an existing or fresh canonical constant node for `value`.
    fn materialize_constant(&mut self, value: bool) -> GraphNodeId {
        let existing = self.graph.graph.nodes.iter().find_map(|node| {
            match &node.kind {
                GraphNodeKind::Constant(stored) if *stored == value => Some(node.id),
                GraphNodeKind::Logic(logic)
                    if !value
                        && logic.logic_type == LogicType::Not
                        && node.inputs.len() == 1 =>
                {
                    let input = self.graph.graph.find_node_by_id(node.inputs[0])?;
                    matches!(input.kind, GraphNodeKind::Constant(true)).then_some(node.id)
                }
                _ => None,
            }
        });
        if let Some(id) = existing {
            return id;
        }

        let one = self.materialize_true();
        if value {
            one
        } else {
            self.graph.graph.add_node(GraphNode {
                kind: GraphNodeKind::Logic(Logic {
                    logic_type: LogicType::Not,
                }),
                inputs: vec![one],
                tag: "folded-constant".to_owned(),
                ..Default::default()
            })
        }
    }

    fn materialize_true(&mut self) -> GraphNodeId {
        if let Some(id) = self.graph.graph.nodes.iter().find_map(|node| {
            matches!(node.kind, GraphNodeKind::Constant(true)).then_some(node.id)
        }) {
            return id;
        }
        self.graph.graph.add_node(GraphNode {
            kind: GraphNodeKind::Constant(true),
            tag: "folded-constant".to_owned(),
            ..Default::default()
        })
    }

    fn replace_node(&mut self, old: GraphNodeId, new: GraphNodeId) {
        self.graph.graph.replace_input_node_id_lazy(old, new);
        self.graph.graph.remove_by_node_id_lazy(old);
        self.rebuild_after_fold();
    }

    fn remove_unused_node(&mut self, id: GraphNodeId) {
        let unused = self
            .graph
            .graph
            .find_node_by_id(id)
            .is_some_and(|node| node.outputs.is_empty());
        if unused {
            self.graph.graph.remove_by_node_id_lazy(id);
        }
    }

    fn rebuild_after_fold(&mut self) {
        // `build_outputs` must run before `build_inputs`: inputs are rebuilt
        // from the outputs lists, so stale outputs would erase fresh rewires.
        self.graph.graph.build_outputs();
        self.graph.graph.build_inputs();
        self.graph.graph.build_outputs();
        self.graph.graph.build_producers();
        self.graph.graph.build_consumers();
    }
}

#[cfg(test)]
mod tests {
    use crate::graph::logic::LogicGraph;
    use crate::graph::{Graph, GraphNode, GraphNodeId, GraphNodeKind};
    use crate::logic::{Logic, LogicType};

    fn input(name: &str) -> GraphNode {
        GraphNode {
            kind: GraphNodeKind::Input(name.to_owned()),
            ..Default::default()
        }
    }

    fn logic(logic_type: LogicType, inputs: Vec<GraphNodeId>) -> GraphNode {
        GraphNode {
            kind: GraphNodeKind::Logic(Logic { logic_type }),
            inputs,
            ..Default::default()
        }
    }

    fn output(input: GraphNodeId) -> GraphNode {
        GraphNode {
            kind: GraphNodeKind::Output("y".to_owned()),
            inputs: vec![input],
            ..Default::default()
        }
    }

    fn finish(nodes: Vec<GraphNode>) -> LogicGraph {
        let mut graph = Graph::from_nodes(nodes);
        graph.build_outputs();
        graph.build_inputs();
        graph.build_outputs();
        graph.build_producers();
        graph.build_consumers();
        graph.verify().unwrap();
        LogicGraph { graph }
    }

    fn output_producer(graph: &LogicGraph) -> GraphNodeId {
        let output = graph
            .nodes
            .iter()
            .find(|node| matches!(node.kind, GraphNodeKind::Output(_)))
            .expect("output node");
        output.inputs[0]
    }

    #[test]
    fn folds_identity_and_absorbing_operations() {
        // and(x, 1) -> x
        let mut transformer = super::LogicGraphTransformer::new(finish(vec![
            input("x"),
            GraphNode {
                kind: GraphNodeKind::Constant(true),
                ..Default::default()
            },
            logic(LogicType::And, vec![0, 1]),
            output(2),
        ]));
        transformer.fold_constants().unwrap();
        let graph = transformer.finish();
        let producer = graph.find_node_by_id(output_producer(&graph)).unwrap();
        assert!(matches!(producer.kind, GraphNodeKind::Input(_)));

        // or(x, 0) -> x, with zero represented as Not(Constant(true))
        let mut transformer = super::LogicGraphTransformer::new(finish(vec![
            input("x"),
            GraphNode {
                kind: GraphNodeKind::Constant(true),
                ..Default::default()
            },
            logic(LogicType::Not, vec![1]),
            logic(LogicType::Or, vec![0, 2]),
            output(3),
        ]));
        transformer.fold_constants().unwrap();
        let graph = transformer.finish();
        let producer = graph.find_node_by_id(output_producer(&graph)).unwrap();
        assert!(matches!(producer.kind, GraphNodeKind::Input(_)));

        // and(x, 0) -> 0
        let mut transformer = super::LogicGraphTransformer::new(finish(vec![
            input("x"),
            GraphNode {
                kind: GraphNodeKind::Constant(true),
                ..Default::default()
            },
            logic(LogicType::Not, vec![1]),
            logic(LogicType::And, vec![0, 2]),
            output(3),
        ]));
        transformer.fold_constants().unwrap();
        let graph = transformer.finish();
        let producer = graph.find_node_by_id(output_producer(&graph)).unwrap();
        assert!(matches!(
            producer.kind,
            GraphNodeKind::Logic(Logic {
                logic_type: LogicType::Not
            })
        ));
    }

    #[test]
    fn folds_xor_with_constants_and_not_of_constant() {
        // xor(x, 1) -> not(x)
        let mut transformer = super::LogicGraphTransformer::new(finish(vec![
            input("x"),
            GraphNode {
                kind: GraphNodeKind::Constant(true),
                ..Default::default()
            },
            logic(LogicType::Xor, vec![0, 1]),
            output(2),
        ]));
        transformer.fold_constants().unwrap();
        let graph = transformer.finish();
        let producer = graph.find_node_by_id(output_producer(&graph)).unwrap();
        assert!(matches!(
            producer.kind,
            GraphNodeKind::Logic(Logic {
                logic_type: LogicType::Not
            })
        ));

        // not(0) -> 1
        let mut transformer = super::LogicGraphTransformer::new(finish(vec![
            GraphNode {
                kind: GraphNodeKind::Constant(true),
                ..Default::default()
            },
            logic(LogicType::Not, vec![0]),
            logic(LogicType::Not, vec![1]),
            output(2),
        ]));
        transformer.fold_constants().unwrap();
        let graph = transformer.finish();
        let producer = graph.find_node_by_id(output_producer(&graph)).unwrap();
        assert!(matches!(producer.kind, GraphNodeKind::Constant(true)));
    }

    #[test]
    fn folding_preserves_fanout() {
        // and(x, 1) feeding two outputs: both must read x.
        let mut graph = Graph::from_nodes(vec![
            input("x"),
            GraphNode {
                kind: GraphNodeKind::Constant(true),
                ..Default::default()
            },
            logic(LogicType::And, vec![0, 1]),
            GraphNode {
                kind: GraphNodeKind::Output("y0".to_owned()),
                inputs: vec![2],
                ..Default::default()
            },
            GraphNode {
                kind: GraphNodeKind::Output("y1".to_owned()),
                inputs: vec![2],
                ..Default::default()
            },
        ]);
        graph.build_outputs();
        graph.build_inputs();
        graph.build_outputs();
        graph.build_producers();
        graph.build_consumers();
        graph.verify().unwrap();

        let mut transformer = super::LogicGraphTransformer::new(LogicGraph { graph });
        transformer.fold_constants().unwrap();
        let graph = transformer.finish();
        for node in graph.nodes.iter() {
            if let GraphNodeKind::Output(_) = node.kind {
                let producer = graph.find_node_by_id(node.inputs[0]).unwrap();
                assert!(matches!(producer.kind, GraphNodeKind::Input(_)));
            }
        }
    }
}
