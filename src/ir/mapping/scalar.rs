//! Bit-blasting names and leaf construction for general lowering.
//!
//! Logical IR is bus-aware; Routable IR is scalar. [`ScalarNets`] maps a
//! logical net plus a bit index to the deterministic scalar net name used by
//! the physical flow. [`LeafBuilder`] accumulates the scalar graph nodes of one
//! Routable leaf and converts them through the shared leaf writer.

use std::collections::{HashMap, HashSet, VecDeque};

use eyre::{ContextCompat, WrapErr};

use super::routable_leaf_from_graph;
use crate::graph::logic::LogicGraph;
use crate::graph::{Graph, GraphNode, GraphNodeKind};
use crate::ir::RoutableModule;
use crate::logic::LogicType;

/// Hard ceiling on nodes per partitioned leaf. The local placer rejects leaves
/// above `K_MAX_LOCAL_PLACE_NODE_COUNT = 40`.
const PARTITION_NODE_LIMIT: usize = 40;

/// Placement-quality target for prepared leaves. The local placer's search is
/// fragile near its hard limit, so chunks are kept smaller than the ceiling.
const PARTITION_PREPARED_NODE_BUDGET: usize = 40;

/// Scalar net names for one bit-blasted logical module.
#[derive(Clone, Debug)]
pub(super) struct ScalarNets {
    widths: HashMap<String, usize>,
}

impl ScalarNets {
    pub(super) fn from_module(module: &crate::ir::LogicalModule) -> Self {
        Self {
            widths: module
                .nets
                .iter()
                .map(|net| (net.name.clone(), net.width))
                .collect(),
        }
    }

    pub(super) fn width(&self, net: &str) -> eyre::Result<usize> {
        self.widths
            .get(net)
            .copied()
            .with_context(|| format!("unknown logical net `{net}`"))
    }

    /// Scalar name of one bit of a logical net.
    ///
    /// Scalar nets keep their logical name; wider nets use the same `name_bit`
    /// spelling that the Verilog frontend uses when it flattens `name[bit]`.
    pub(super) fn scalar(&self, net: &str, bit: usize) -> eyre::Result<String> {
        let width = self.width(net)?;
        if bit >= width {
            eyre::bail!("bit {bit} is out of range for logical net `{net}` of width {width}");
        }
        Ok(scalar_name(net, width, bit))
    }

    pub(super) fn bits(&self, net: &str) -> eyre::Result<Vec<String>> {
        let width = self.width(net)?;
        Ok((0..width).map(|bit| scalar_name(net, width, bit)).collect())
    }
}

pub(super) fn scalar_name(net: &str, width: usize, bit: usize) -> String {
    if width == 1 {
        net.to_owned()
    } else {
        format!("{net}_{bit}")
    }
}

/// Accumulates the scalar nodes of one Routable leaf.
#[derive(Default)]
pub(super) struct LeafBuilder {
    nodes: Vec<GraphNode>,
    producers: HashMap<String, usize>,
    input_order: Vec<String>,
    input_nodes: HashMap<usize, String>,
    outputs: Vec<(String, usize)>,
    constants: HashMap<bool, usize>,
}

impl LeafBuilder {
    pub(super) fn new() -> Self {
        Self::default()
    }

    /// Declares a boundary input; repeated declarations reuse the same node.
    pub(super) fn add_input(&mut self, name: &str) -> usize {
        if let Some(node) = self.producers.get(name) {
            return *node;
        }
        let node = self.nodes.len();
        self.nodes.push(GraphNode {
            kind: GraphNodeKind::Input(name.to_owned()),
            ..Default::default()
        });
        self.producers.insert(name.to_owned(), node);
        self.input_order.push(name.to_owned());
        self.input_nodes.insert(node, name.to_owned());
        node
    }

    /// Materializes a logical constant bit. Constant one becomes a physical
    /// redstone block; constant zero is expanded to `not(constant one)` so the
    /// physical flow only ever places powered sources.
    pub(super) fn constant(&mut self, value: bool) -> usize {
        if let Some(node) = self.constants.get(&value) {
            return *node;
        }
        let one = match self.constants.get(&true) {
            Some(node) => *node,
            None => {
                let node = self.nodes.len();
                self.nodes.push(GraphNode {
                    kind: GraphNodeKind::Constant(true),
                    ..Default::default()
                });
                self.constants.insert(true, node);
                node
            }
        };
        let node = if value {
            one
        } else {
            self.add_logic(LogicType::Not, vec![one], "__const0")
        };
        self.constants.insert(value, node);
        node
    }

    pub(super) fn add_logic(
        &mut self,
        logic_type: LogicType,
        inputs: Vec<usize>,
        tag: &str,
    ) -> usize {
        let node = self.nodes.len();
        self.nodes.push(GraphNode {
            kind: GraphNodeKind::Logic(crate::logic::Logic { logic_type }),
            inputs,
            tag: tag.to_owned(),
            ..Default::default()
        });
        node
    }

    pub(super) fn add_output(&mut self, name: &str, input: usize) {
        self.outputs.push((name.to_owned(), input));
    }

    pub(super) fn set_producer(&mut self, net: &str, node: usize) {
        self.producers.insert(net.to_owned(), node);
    }

    pub(super) fn producer(&self, net: &str) -> Option<usize> {
        self.producers.get(net).copied()
    }

    pub(super) fn has_producer(&self, net: &str) -> bool {
        self.producers.contains_key(net)
    }

    pub(super) fn input_name(&self, node: usize) -> Option<&str> {
        self.input_nodes.get(&node).map(String::as_str)
    }

    pub(super) fn input_names(&self) -> &[String] {
        &self.input_order
    }

    pub(super) fn has_outputs(&self) -> bool {
        !self.outputs.is_empty()
    }

    /// Appends the requested outputs and returns the finished scalar graph.
    pub(super) fn into_graph(mut self) -> eyre::Result<Graph> {
        for (output, input) in self.outputs {
            self.nodes.push(GraphNode {
                kind: GraphNodeKind::Output(output),
                inputs: vec![input],
                ..Default::default()
            });
        }
        let mut graph = Graph::from_nodes(self.nodes);
        graph.build_outputs();
        graph.build_producers();
        graph.build_consumers();
        graph.verify()?;
        Ok(graph)
    }

    pub(super) fn finish(self, name: &str) -> eyre::Result<RoutableModule> {
        let graph = self
            .into_graph()
            .wrap_err_with(|| format!("lowered leaf `{name}` is not a valid graph"))?;
        routable_leaf_from_graph(name, graph)
    }

    /// Splits the accumulated combinational cone into one or more leaves that
    /// each stay below the local placer node budget.
    ///
    /// The partition is a deterministic greedy pass over a topological order.
    /// Cross-chunk producers become `__t{id}` intermediate ports; requested
    /// outputs keep their names. A cone that fits in one chunk reproduces the
    /// single-leaf result, and requested outputs that are boundary inputs are
    /// reported as pass-through connections instead of producing a leaf.
    pub(super) fn partition(self, base: &str, node_budget: usize) -> eyre::Result<PartitionedLeaf> {
        let LeafBuilder {
            nodes,
            producers: _,
            input_order: _,
            input_nodes: _,
            outputs,
            constants: _,
        } = self;

        let mut consumers = HashMap::<usize, Vec<usize>>::new();
        let mut indegree = HashMap::<usize, usize>::new();
        for (id, node) in nodes.iter().enumerate() {
            if matches!(node.kind, GraphNodeKind::Input(_)) {
                continue;
            }
            let mut degree = 0;
            for input in &node.inputs {
                consumers.entry(*input).or_default().push(id);
                if !matches!(nodes[*input].kind, GraphNodeKind::Input(_)) {
                    degree += 1;
                }
            }
            indegree.insert(id, degree);
        }
        let mut queue = indegree
            .iter()
            .filter(|(_, degree)| **degree == 0)
            .map(|(id, _)| *id)
            .collect::<VecDeque<_>>();
        queue.make_contiguous().sort();
        let mut order = Vec::with_capacity(indegree.len());
        while let Some(id) = queue.pop_front() {
            order.push(id);
            let mut newly = Vec::new();
            for consumer in consumers.get(&id).into_iter().flatten() {
                let degree = indegree
                    .get_mut(consumer)
                    .expect("consumer must have an indegree");
                *degree -= 1;
                if *degree == 0 {
                    newly.push(*consumer);
                }
            }
            newly.sort();
            for id in newly {
                queue.push_back(id);
            }
        }
        if order.len() != indegree.len() {
            eyre::bail!("combinational cone is cyclic");
        }

        let final_names = outputs
            .iter()
            .map(|(name, _)| name.clone())
            .collect::<HashSet<_>>();
        let mut output_requests = HashMap::<usize, Vec<String>>::new();
        for (name, node) in &outputs {
            output_requests.entry(*node).or_default().push(name.clone());
        }

        let mut chunk_of = vec![usize::MAX; nodes.len()];
        let mut chunks = Vec::<Vec<usize>>::new();
        let mut chunk_inputs = Vec::<HashSet<String>>::new();
        for id in order {
            let node = &nodes[id];
            let current_index = chunks.len().saturating_sub(1);
            // Inputs that are not produced inside the current chunk: boundary
            // nets and cross-chunk intermediates.
            let external_names = node
                .inputs
                .iter()
                .filter_map(|input| {
                    let is_input_node = matches!(nodes[*input].kind, GraphNodeKind::Input(_));
                    if !is_input_node && chunk_of[*input] == current_index {
                        return None;
                    }
                    Some(chunk_input_name(*input, &nodes))
                })
                .collect::<Vec<_>>();
            let needs_new_chunk = match chunks.last() {
                None => true,
                Some(chunk) => {
                    let current_inputs = chunk_inputs.last().expect("chunk inputs align");
                    let new_inputs = external_names
                        .iter()
                        .filter(|name| !current_inputs.contains(*name))
                        .count();
                    chunk.len() + current_inputs.len() + new_inputs + 1 > node_budget
                }
            };
            if needs_new_chunk {
                chunks.push(Vec::new());
                chunk_inputs.push(HashSet::new());
            }
            let index = chunks.len() - 1;
            chunk_of[id] = index;
            chunks[index].push(id);
            for name in external_names {
                chunk_inputs[index].insert(name);
            }
        }

        // The greedy pass sizes chunks by logic and external inputs, but every
        // cross-chunk or requested output also becomes a node. Split any chunk
        // that still exceeds the local placer limit until every chunk fits.
        loop {
            let counts =
                chunk_node_counts(&chunks, &chunk_of, &nodes, &consumers, &output_requests);
            let Some(index) = counts
                .iter()
                .position(|count| *count > PARTITION_NODE_LIMIT)
            else {
                break;
            };
            if chunks[index].len() < 2 {
                break;
            }
            split_chunk(&mut chunks, &mut chunk_of, index);
        }

        // `prepare_place` expands And/Xor and inserts buffers, so the graph the
        // local placer sees can be much larger than the raw chunk. Split until
        // every chunk stays below the limit after preparation.
        loop {
            let multiple_chunks = chunks.len() > 1;
            let producer_instance = chunk_producer_instances(&chunks, base, multiple_chunks);
            let mut oversized = None;
            for (index, chunk) in chunks.iter().enumerate() {
                let count = chunk_prepared_node_count(
                    &nodes,
                    chunk,
                    &chunk_of,
                    &consumers,
                    &output_requests,
                    &producer_instance,
                )?;
                if count > PARTITION_PREPARED_NODE_BUDGET {
                    oversized = Some(index);
                    break;
                }
            }
            let Some(index) = oversized else {
                break;
            };
            if chunks[index].len() < 2 {
                break;
            }
            split_chunk(&mut chunks, &mut chunk_of, index);
        }

        let multiple_chunks = chunks.len() > 1;
        let producer_instance = chunk_producer_instances(&chunks, base, multiple_chunks);
        let mut modules = Vec::with_capacity(chunks.len());
        let mut instances = Vec::with_capacity(chunks.len());
        let mut intermediates = HashMap::new();
        let mut output_sources = HashMap::new();
        let mut direct_outputs = HashMap::new();

        for (chunk_index, chunk) in chunks.iter().enumerate() {
            let instance = chunk_instance_name(base, chunk_index, multiple_chunks);
            let mut builder = LeafBuilder::new();
            let local = build_chunk_nodes(
                &nodes,
                chunk,
                &producer_instance,
                &mut intermediates,
                &mut builder,
            )?;
            for (id, name) in
                chunk_outputs(chunk, chunk_index, &chunk_of, &consumers, &output_requests)
            {
                let local_id = local[&id];
                builder.add_output(&name, local_id);
                if final_names.contains(&name) {
                    output_sources.insert(name.clone(), (instance.clone(), name));
                }
            }
            let inputs = builder.input_names().to_vec();
            modules.push(builder.finish(&instance)?);
            instances.push(LeafInstance { instance, inputs });
        }

        for (name, node) in &outputs {
            if let GraphNodeKind::Input(source) = &nodes[*node].kind {
                direct_outputs.insert(name.clone(), source.clone());
            }
        }

        Ok(PartitionedLeaf {
            modules,
            instances,
            intermediates,
            output_sources,
            direct_outputs,
        })
    }
}

/// One generated leaf of a partitioned combinational cone.
pub(super) struct LeafInstance {
    pub instance: String,
    pub inputs: Vec<String>,
}

/// The leaves and wiring produced by [`LeafBuilder::partition`].
pub(super) struct PartitionedLeaf {
    pub modules: Vec<RoutableModule>,
    pub instances: Vec<LeafInstance>,
    /// Intermediate port name -> producing instance.
    pub intermediates: HashMap<String, String>,
    /// Requested output port -> (instance, port).
    pub output_sources: HashMap<String, (String, String)>,
    /// Requested output port -> boundary input net (pass-through).
    pub direct_outputs: HashMap<String, String>,
}

fn chunk_input_name(input: usize, nodes: &[GraphNode]) -> String {
    match &nodes[input].kind {
        GraphNodeKind::Input(name) => name.clone(),
        _ => format!("__t{input}"),
    }
}

/// Instance name of one chunk. A single chunk keeps the requested base name so
/// a cone that fits one leaf stays a plain leaf; multiple chunks are suffixed.
fn chunk_instance_name(base: &str, index: usize, multiple_chunks: bool) -> String {
    if multiple_chunks {
        format!("{base}_p{index}")
    } else {
        base.to_owned()
    }
}

fn chunk_producer_instances(
    chunks: &[Vec<usize>],
    base: &str,
    multiple_chunks: bool,
) -> HashMap<usize, String> {
    let mut producers = HashMap::new();
    for (index, chunk) in chunks.iter().enumerate() {
        let instance = chunk_instance_name(base, index, multiple_chunks);
        for id in chunk {
            producers.insert(*id, instance.clone());
        }
    }
    producers
}

fn split_chunk(chunks: &mut Vec<Vec<usize>>, chunk_of: &mut [usize], index: usize) {
    let mid = chunks[index].len() / 2;
    let tail = chunks[index].split_off(mid);
    chunks.insert(index + 1, tail);
    for (chunk_index, chunk) in chunks.iter().enumerate() {
        for id in chunk {
            chunk_of[*id] = chunk_index;
        }
    }
}

/// Requested outputs and cross-chunk intermediates a chunk must expose.
fn chunk_outputs(
    chunk: &[usize],
    chunk_index: usize,
    chunk_of: &[usize],
    consumers: &HashMap<usize, Vec<usize>>,
    output_requests: &HashMap<usize, Vec<String>>,
) -> Vec<(usize, String)> {
    let mut outputs = Vec::new();
    for id in chunk {
        for name in output_requests.get(id).cloned().unwrap_or_default() {
            outputs.push((*id, name));
        }
        let consumed_later = consumers
            .get(id)
            .into_iter()
            .flatten()
            .any(|consumer| chunk_of[*consumer] != chunk_index);
        if consumed_later {
            outputs.push((*id, format!("__t{id}")));
        }
    }
    outputs
}

/// Copies one chunk's nodes into `builder`, resolving cross-chunk producers to
/// leaf inputs. Shared by the sizing pass and the final build.
fn build_chunk_nodes(
    nodes: &[GraphNode],
    chunk: &[usize],
    producer_instance: &HashMap<usize, String>,
    intermediates: &mut HashMap<String, String>,
    builder: &mut LeafBuilder,
) -> eyre::Result<HashMap<usize, usize>> {
    let mut local = HashMap::<usize, usize>::new();
    for id in chunk {
        let node = &nodes[*id];
        let mut inputs = Vec::with_capacity(node.inputs.len());
        for input in &node.inputs {
            if let Some(local_id) = local.get(input) {
                inputs.push(*local_id);
            } else {
                let name = chunk_input_name(*input, nodes);
                let local_id = builder.add_input(&name);
                if !matches!(nodes[*input].kind, GraphNodeKind::Input(_)) {
                    let producer = producer_instance
                        .get(input)
                        .with_context(|| format!("missing producer for `{name}`"))?
                        .clone();
                    intermediates.insert(name, producer);
                }
                inputs.push(local_id);
            }
        }
        let local_id = match &node.kind {
            GraphNodeKind::Logic(logic) => builder.add_logic(logic.logic_type, inputs, &node.tag),
            GraphNodeKind::Constant(value) => {
                if !inputs.is_empty() {
                    eyre::bail!("partitioned constant node {id} has inputs");
                }
                let local_id = builder.nodes.len();
                builder.nodes.push(GraphNode {
                    kind: GraphNodeKind::Constant(*value),
                    ..Default::default()
                });
                local_id
            }
            other => eyre::bail!("partitioned cone contains non-logic node {other:?}"),
        };
        local.insert(*id, local_id);
    }
    Ok(local)
}

/// Node count of one chunk after `prepare_place`, which is what the local
/// placer's limit applies to.
fn chunk_prepared_node_count(
    nodes: &[GraphNode],
    chunk: &[usize],
    chunk_of: &[usize],
    consumers: &HashMap<usize, Vec<usize>>,
    output_requests: &HashMap<usize, Vec<String>>,
    producer_instance: &HashMap<usize, String>,
) -> eyre::Result<usize> {
    let mut builder = LeafBuilder::new();
    let mut scratch = HashMap::new();
    let local = build_chunk_nodes(nodes, chunk, producer_instance, &mut scratch, &mut builder)?;
    let chunk_index = chunk_of[chunk[0]];
    for (id, name) in chunk_outputs(chunk, chunk_index, chunk_of, consumers, output_requests) {
        builder.add_output(&name, local[&id]);
    }
    let graph = builder.into_graph()?;
    let prepared = LogicGraph { graph }.prepare_place()?;
    Ok(prepared.nodes.len())
}

/// Actual graph node count of every chunk: logic nodes plus the leaf inputs
/// and outputs the chunk will expose.
fn chunk_node_counts(
    chunks: &[Vec<usize>],
    chunk_of: &[usize],
    nodes: &[GraphNode],
    consumers: &HashMap<usize, Vec<usize>>,
    output_requests: &HashMap<usize, Vec<String>>,
) -> Vec<usize> {
    let mut counts = chunks.iter().map(|chunk| chunk.len()).collect::<Vec<_>>();
    let mut inputs = vec![HashSet::new(); chunks.len()];
    for (chunk_index, chunk) in chunks.iter().enumerate() {
        for id in chunk {
            for input in &nodes[*id].inputs {
                if matches!(nodes[*input].kind, GraphNodeKind::Input(_))
                    || chunk_of[*input] != chunk_index
                {
                    inputs[chunk_index].insert(chunk_input_name(*input, nodes));
                }
            }
            counts[chunk_index] += output_requests.get(id).map_or(0, Vec::len);
            if consumers
                .get(id)
                .into_iter()
                .flatten()
                .any(|consumer| chunk_of[*consumer] != chunk_index)
            {
                counts[chunk_index] += 1;
            }
        }
    }
    for (count, inputs) in counts.iter_mut().zip(inputs) {
        *count += inputs.len();
    }
    counts
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn scalar_names_flatten_vectors_and_keep_scalars() {
        assert_eq!(scalar_name("a", 1, 0), "a");
        assert_eq!(scalar_name("a", 4, 0), "a_0");
        assert_eq!(scalar_name("a", 4, 3), "a_3");
    }

    #[test]
    fn leaf_builder_deduplicates_inputs_and_keeps_order() {
        let mut builder = LeafBuilder::new();
        let first = builder.add_input("a");
        let again = builder.add_input("a");
        let second = builder.add_input("b");

        assert_eq!(first, again);
        assert_ne!(first, second);
        assert_eq!(builder.input_names(), ["a", "b"]);
    }

    #[test]
    fn partition_splits_a_long_chain_and_keeps_connectivity() {
        let mut builder = LeafBuilder::new();
        let mut value = builder.add_input("a");
        for _ in 0..5 {
            value = builder.add_logic(LogicType::Not, vec![value], "chain");
        }
        builder.add_output("y", value);

        let partitioned = builder.partition("chain", 2).expect("partition");
        assert!(
            partitioned.modules.len() >= 2,
            "a five-node chain with budget two must split"
        );
        for leaf in &partitioned.instances {
            for input in &leaf.inputs {
                assert!(
                    input == "a" || partitioned.intermediates.contains_key(input),
                    "unresolved leaf input `{input}`"
                );
            }
        }
        let (instance, port) = partitioned
            .output_sources
            .get("y")
            .expect("requested output must be produced");
        assert_eq!(port, "y");
        assert!(partitioned
            .instances
            .iter()
            .any(|leaf| &leaf.instance == instance));
        for module in &partitioned.modules {
            let crate::ir::RoutableModuleBody::Leaf { nodes } = &module.body else {
                panic!("partitioned chunks must be leaves");
            };
            assert!(
                nodes.len() <= 5,
                "chunk `{}` has {} nodes",
                module.name,
                nodes.len()
            );
        }
    }

    #[test]
    fn partition_keeps_a_small_cone_in_one_named_leaf() {
        let mut builder = LeafBuilder::new();
        let a = builder.add_input("a");
        let b = builder.add_input("b");
        let value = builder.add_logic(LogicType::And, vec![a, b], "and");
        builder.add_output("y", value);

        let partitioned = builder.partition("small", 28).expect("partition");
        assert_eq!(partitioned.instances.len(), 1);
        assert_eq!(partitioned.instances[0].instance, "small");
        assert!(partitioned.intermediates.is_empty());
        assert_eq!(
            partitioned.output_sources.get("y"),
            Some(&("small".to_owned(), "y".to_owned()))
        );
    }
}
