use std::cmp::Reverse;
use std::collections::{BTreeMap, HashSet, VecDeque};
use std::ops::{Deref, DerefMut};

use eyre::ContextCompat;

use crate::graph::logic::LogicGraph;
use crate::graph::{Graph, GraphNodeKind};
use crate::ir::{graph_from_routable_leaf, RoutableModule, RoutablePortDirection};
use crate::output::{OutputEndpoint, PlacedWorld};
use crate::transform::place_and_route::detailed_router;
use crate::transform::place_and_route::global_pnr::cell_library::CellPhysicalContract;
use crate::transform::place_and_route::global_pnr::ir::{
    pareto_frontier, LayoutCandidate, PhysicalPort, PhysicalPortDirection, PortConnection,
};
use crate::transform::place_and_route::local_placer::{
    LocalPlacer, LocalPlacerConfig, LocalPlacerInputConstraints,
};
use crate::transform::place_and_route::placed_node::PlacedNode;
use crate::world::block::Block;
use crate::world::position::{DimSize, Position};
use crate::world::simulator::Simulator;
use crate::world::{World, World3D};

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct UnitCandidateConfig {
    pub dim: DimSize,
    pub local_config: LocalPlacerConfig,
    pub input_constraints: LocalPlacerInputConstraints,
    pub max_candidates: usize,
    pub combinational_sampling_limit: Option<usize>,
}

/// Local-candidate preparation policy before it is resolved against a typed
/// Routable definition. The default preserves the legacy single-policy API;
/// definition and port entries provide the scoped model used by RCIR.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CandidatePolicySet {
    pub default: UnitCandidateConfig,
    pub definition_overrides: BTreeMap<String, UnitCandidateConfig>,
    pub pin_search: BTreeMap<(String, String), Vec<Position>>,
    /// Reusable named cell implementations. A matching implementation wins
    /// over the default policy but not over an explicit definition override.
    pub cell_library: crate::transform::place_and_route::global_pnr::cell_library::CellLibrary,
}

impl CandidatePolicySet {
    pub fn new(default: UnitCandidateConfig) -> Self {
        Self {
            default,
            definition_overrides: BTreeMap::new(),
            pin_search: BTreeMap::new(),
            cell_library:
                crate::transform::place_and_route::global_pnr::cell_library::CellLibrary::redstone_v1(
                ),
        }
    }

    pub fn with_cell_library(
        mut self,
        library: crate::transform::place_and_route::global_pnr::cell_library::CellLibrary,
    ) -> Self {
        self.cell_library = library;
        self
    }

    pub fn with_definition_override(
        mut self,
        definition: impl Into<String>,
        policy: UnitCandidateConfig,
    ) -> Self {
        self.definition_overrides.insert(definition.into(), policy);
        self
    }

    pub fn with_pin_search(
        mut self,
        definition: impl Into<String>,
        port: impl Into<String>,
        positions: impl IntoIterator<Item = Position>,
    ) -> Self {
        self.pin_search.insert(
            (definition.into(), port.into()),
            positions.into_iter().collect(),
        );
        self
    }

    pub fn effective_for_definition(&self, definition: &str) -> UnitCandidateConfig {
        let mut policy = self
            .definition_overrides
            .get(definition)
            .cloned()
            .or_else(|| {
                self.cell_library
                    .implementation_for_definition(definition)
                    .map(|implementation| implementation.candidate.clone())
            })
            .unwrap_or_else(|| self.default.clone());
        for ((owner, port), positions) in &self.pin_search {
            if owner == definition {
                policy.input_constraints = policy
                    .input_constraints
                    .with_input_positions(port.clone(), positions.iter().copied());
            }
        }
        policy
    }
    /// Physical contract of the cell library implementation matching a
    /// definition, if any. An explicit definition override changes the search
    /// policy but the library contract still describes the variant.
    pub fn effective_contract_for_definition(
        &self,
        definition: &str,
    ) -> Option<crate::transform::place_and_route::global_pnr::cell_library::CellPhysicalContract>
    {
        self.cell_library
            .implementation_for_definition(definition)
            .map(|implementation| implementation.contract.clone())
    }
}

impl Default for CandidatePolicySet {
    fn default() -> Self {
        Self::new(UnitCandidateConfig::default())
    }
}

impl From<UnitCandidateConfig> for CandidatePolicySet {
    fn from(default: UnitCandidateConfig) -> Self {
        Self::new(default)
    }
}

impl Deref for CandidatePolicySet {
    type Target = UnitCandidateConfig;

    fn deref(&self) -> &Self::Target {
        &self.default
    }
}

impl DerefMut for CandidatePolicySet {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.default
    }
}

impl Default for UnitCandidateConfig {
    fn default() -> Self {
        Self {
            dim: DimSize(16, 16, 6),
            local_config: LocalPlacerConfig::default(),
            input_constraints: LocalPlacerInputConstraints::default(),
            max_candidates: 16,
            combinational_sampling_limit: Some(32),
        }
    }
}

pub fn generate_routable_module_candidates_with_progress_label(
    module: &RoutableModule,
    config: &UnitCandidateConfig,
    contract: Option<&CellPhysicalContract>,
    progress_label: Option<&str>,
) -> eyre::Result<Vec<LayoutCandidate>> {
    let graph = graph_from_routable_leaf(module)?;
    let ports = module
        .ports
        .iter()
        .map(|port| {
            CandidatePort::new(
                &port.name,
                &port.name,
                match port.direction {
                    RoutablePortDirection::Input => PhysicalPortDirection::Input,
                    RoutablePortDirection::Output => PhysicalPortDirection::Output,
                },
            )
        })
        .collect();
    generate_unit_candidates(&module.name, graph, ports, config, contract, progress_label)
}

#[derive(Clone, Debug)]
struct CandidatePort {
    name: String,
    target: String,
    direction: PhysicalPortDirection,
}

impl CandidatePort {
    fn new(name: &str, target: &str, direction: PhysicalPortDirection) -> Self {
        Self {
            name: name.to_owned(),
            target: target.to_owned(),
            direction,
        }
    }
}

fn generate_unit_candidates(
    module_name: &str,
    graph: Graph,
    ports: Vec<CandidatePort>,
    config: &UnitCandidateConfig,
    contract: Option<&CellPhysicalContract>,
    progress_label: Option<&str>,
) -> eyre::Result<Vec<LayoutCandidate>> {
    crate::perf::check_budget("local candidate generation")?;
    let graph = LogicGraph { graph }.prepare_place()?;
    let placer = LocalPlacer::new(graph.clone(), config.local_config)?;

    let placed = placer.generate_with_outputs_and_input_constraints_progress(
        config.dim,
        None,
        &config.input_constraints,
        progress_label,
    );
    if crate::perf::budget_exceeded() {
        eyre::bail!("memory budget exceeded during local candidate generation for `{module_name}`");
    }
    if crate::perf::work_exceeded() {
        eyre::bail!(
            "local candidate generation for `{module_name}` exceeded its work limit (clone budget {} / placement budget {}); the design is too dense for the current local placer",
            crate::perf::local_clone_limit(),
            crate::perf::LOCAL_WORK_LIMIT
        );
    }

    let contains_sequential = graph
        .graph
        .nodes
        .iter()
        .any(|node| matches!(node.kind, GraphNodeKind::Sequential(_)));
    let validate_truth_table = !contains_sequential;
    // Collect a bounded pool so the Pareto frontier can drop dominated
    // candidates instead of truncating generation order. The multiplier keeps
    // the extra truth-table validation work small.
    let pool_limit = config.max_candidates.saturating_mul(4).clamp(1, 64);
    let mut candidates = Vec::new();
    for placed in placed {
        if candidates.len() >= pool_limit {
            break;
        }
        if validate_truth_table && !candidate_matches_truth_table(&graph, &placed)? {
            crate::perf::note_candidate_truth_reject();
            continue;
        }
        let (world, physical_ports) = switchless_candidate_layout(
            &ports,
            contains_sequential,
            contract,
            &config.input_constraints,
            placed.world,
            &placed.inputs,
            &placed.outputs,
        );
        if !candidate_ports_cover_module_ports(&ports, &physical_ports) {
            crate::perf::note_candidate_port_reject();
            continue;
        }
        let mut candidate =
            LayoutCandidate::from_world(module_name.to_owned(), world, physical_ports)?;
        if let Some(contract) = contract {
            candidate.halo = contract.halo;
        }
        candidates.push(candidate);
    }
    Ok(pareto_frontier(candidates, config.max_candidates))
}

fn candidate_ports_cover_module_ports(expected: &[CandidatePort], actual: &[PhysicalPort]) -> bool {
    expected
        .iter()
        .all(|expected| actual.iter().any(|port| port.name == expected.name))
}

fn truth_debug_enabled() -> bool {
    use std::sync::atomic::{AtomicU8, Ordering};
    static FLAG: AtomicU8 = AtomicU8::new(0);
    match FLAG.load(Ordering::Relaxed) {
        1 => true,
        2 => false,
        _ => {
            let enabled = std::env::var_os("MCHDL_DEBUG_TRUTH_TABLE").is_some();
            FLAG.store(if enabled { 1 } else { 2 }, Ordering::Relaxed);
            enabled
        }
    }
}

fn candidate_matches_truth_table(
    expected: &LogicGraph,
    placed: &PlacedWorld,
) -> eyre::Result<bool> {
    let graph = expected;
    let expected = expected.truth_table()?;
    let inputs = expected
        .input_names
        .iter()
        .map(|name| {
            placed
                .inputs
                .iter()
                .find(|input| input.name == *name)
                .map(|input| input.position())
                .with_context(|| format!("missing input endpoint `{name}`"))
        })
        .collect::<eyre::Result<Vec<_>>>()?;
    let outputs = expected
        .output_tables
        .keys()
        .map(|name| {
            placed
                .outputs
                .iter()
                .find(|output| output.name == *name)
                .map(|output| (name.as_str(), output.position()))
                .with_context(|| format!("missing output endpoint `{name}`"))
        })
        .collect::<eyre::Result<Vec<_>>>()?;
    let world = World::from(&placed.world);
    let debug_truth = truth_debug_enabled();
    let mask_count = 1usize << inputs.len();
    static TRUTH_DEBUG_PRINTED: std::sync::atomic::AtomicBool =
        std::sync::atomic::AtomicBool::new(false);
    let verbose_table =
        debug_truth && !TRUTH_DEBUG_PRINTED.swap(true, std::sync::atomic::Ordering::Relaxed);
    if verbose_table {
        eprintln!("[truth] input_names={:?}", expected.input_names);
        for (name, position) in &outputs {
            eprintln!("[truth] output `{name}` pos={position:?}");
        }
        for (name, table) in &expected.output_tables {
            eprintln!("[truth] expected `{name}` = {table:?}");
        }
        for node in &graph.graph.nodes {
            eprintln!(
                "[truth] node {:?} kind={:?} inputs={:?}",
                node.id, node.kind, node.inputs
            );
        }
    }
    let print_mismatch = |stage: &str, mask: usize, previous: Option<usize>, world: &World3D| {
        if !debug_truth {
            return;
        }
        eprintln!(
            "[truth] stage={stage} mask={mask:0width$b}/{mask_count} previous={previous:?}",
            width = inputs.len()
        );
        for (index, position) in inputs.iter().enumerate() {
            eprintln!(
                "[truth]   input[{index}] pos={position:?} level={}",
                (mask & (1 << index)) != 0
            );
        }
        for (name, position) in &outputs {
            let expected_value = expected
                .output_tables
                .get(*name)
                .and_then(|table| table.get(mask))
                .copied();
            eprintln!(
                "[truth]   output `{name}` pos={position:?} expected={expected_value:?} actual={}",
                world[*position].kind.is_powered()
            );
        }
    };

    for mask in 0..mask_count {
        let mut sim = Simulator::from_with_limits_and_trace(&world, 256, 50_000, 0)
            .map_err(|error| eyre::eyre!(error.message().to_owned()))?;
        sim.change_state_with_limits(
            inputs
                .iter()
                .enumerate()
                .map(|(index, position)| (*position, (mask & (1 << index)) != 0))
                .collect(),
            256,
            50_000,
        )?;

        for (output_name, output_position) in &outputs {
            let Some(expected_output) = expected.output_tables.get(*output_name) else {
                return Ok(false);
            };
            let actual = sim.world()[*output_position].kind.is_powered();
            if verbose_table {
                eprintln!(
                    "[truth] fresh mask={mask:0width$b} `{output_name}` expected={} actual={actual}",
                    expected_output[mask],
                    width = inputs.len()
                );
            }
            if actual != expected_output[mask] {
                print_mismatch("fresh", mask, None, sim.world());
                return Ok(false);
            }
        }
    }

    // A candidate is used as a persistent child inside a routed design, so
    // matching each truth-table row from a freshly initialized world is not
    // sufficient. Exercise both directions in one simulator as well; this
    // rejects layouts whose redstone network powers correctly from reset but
    // fails to release after an input transition.
    let mut sim = Simulator::from_with_limits_and_trace(&world, 256, 50_000, 0)
        .map_err(|error| eyre::eyre!(error.message().to_owned()))?;
    let mut previous_mask: Option<usize> = None;
    for mask in (0..mask_count).chain((0..mask_count).rev()) {
        sim.change_state_with_limits(
            inputs
                .iter()
                .enumerate()
                .map(|(index, position)| (*position, (mask & (1 << index)) != 0))
                .collect(),
            256,
            50_000,
        )?;
        for (output_name, output_position) in &outputs {
            let Some(expected_output) = expected.output_tables.get(*output_name) else {
                return Ok(false);
            };
            if sim.world()[*output_position].kind.is_powered() != expected_output[mask] {
                print_mismatch("transition", mask, previous_mask, sim.world());
                return Ok(false);
            }
        }
        sim.advance_idle_cycles(crate::world::simulator::MANUAL_INPUT_IDLE_CYCLES)?;
        previous_mask = Some(mask);
    }

    Ok(true)
}

// LocalPlacer는 아직 standalone 회로를 기준으로 switch/output layout을 만든다.
// Global PnR child layout에서는 switch를 제거하고 외부 route가 물릴 수 있는
// module port metadata로 다시 노출한다.
// TODO(high-level): make LocalPlacer produce either standalone layouts with switches
// or child-module layouts with PhysicalPort metadata, instead of rewriting switches here.
fn switchless_candidate_layout(
    module_ports: &[CandidatePort],
    contains_sequential: bool,
    contract: Option<&CellPhysicalContract>,
    input_constraints: &LocalPlacerInputConstraints,
    mut world: World3D,
    inputs: &[OutputEndpoint],
    outputs: &[OutputEndpoint],
) -> (World3D, Vec<PhysicalPort>) {
    let mut ports = Vec::new();
    // Sequential child layout은 내부 feedback/state signal이 외부 route와 직접
    // 합쳐지면 back-power 때문에 latch 상태가 깨질 수 있어서 diode 연결을 요구한다.
    // A cell library contract can require the same isolation explicitly.
    let needs_output_isolation =
        contains_sequential || contract.is_some_and(|contract| contract.requires_output_isolation);
    let needs_input_isolation =
        contains_sequential || contract.is_some_and(|contract| contract.requires_input_isolation);
    let use_direct_input_ports = !contains_sequential
        && module_ports
            .iter()
            .filter(|port| port.direction == PhysicalPortDirection::Input)
            .count()
            > 1;
    let preserve_switch_position_inputs = contains_sequential || use_direct_input_ports;
    for port in module_ports {
        match port.direction {
            PhysicalPortDirection::Input => {
                let input_name = &port.target;
                let position = inputs
                    .iter()
                    .find(|input| input.name == *input_name)
                    .map(|input| input.position())
                    .or_else(|| {
                        input_constraints
                            .positions_for_input_name(input_name)
                            .and_then(|positions| positions.into_iter().next())
                    });
                if let Some(input_position) = position {
                    let Some(position) = expose_switchless_input_port(
                        &mut world,
                        input_position,
                        preserve_switch_position_inputs,
                        use_direct_input_ports,
                    ) else {
                        continue;
                    };
                    ports.push(PhysicalPort {
                        name: port.name.clone(),
                        direction: PhysicalPortDirection::Input,
                        position,
                        route_position: None,
                        access_points: vec![position],
                        connection: if needs_input_isolation || world[position].kind.is_redstone() {
                            PortConnection::InputDiode
                        } else {
                            PortConnection::Direct
                        },
                    });
                }
            }
            PhysicalPortDirection::Output => {
                let output_name = &port.target;
                if let Some(output) = outputs.iter().find(|output| output.name == *output_name) {
                    let position = output.position();
                    let access_points = expose_routeable_output_ports(&world, position);
                    let route_position = access_points[0];
                    ports.push(PhysicalPort {
                        name: port.name.clone(),
                        direction: PhysicalPortDirection::Output,
                        position,
                        route_position: Some(route_position),
                        access_points,
                        connection: if needs_output_isolation {
                            PortConnection::OutputDiode
                        } else {
                            PortConnection::Direct
                        },
                    });
                }
            }
        }
    }
    for input in inputs {
        let _ = expose_switchless_input_port(
            &mut world,
            input.position(),
            preserve_switch_position_inputs,
            use_direct_input_ports,
        );
    }
    remove_local_input_switches(&mut world);
    ports.sort_by(|a, b| a.name.cmp(&b.name));
    world.initialize_redstone_states();
    (world, ports)
}

fn remove_local_input_switches(world: &mut World3D) {
    for (position, block) in world.iter_block() {
        if block.kind.is_switch() {
            world[position] = Block::default();
        }
    }
}

// Torch/switch/repeater 같은 출력 블록은 바로 route하기 어려울 수 있으므로,
// 해당 출력이 실제로 power하는 redstone tap들을 route access point로 노출한다.
fn expose_routeable_output_ports(world: &World3D, output_position: Position) -> Vec<Position> {
    if !world.size.bound_on(output_position)
        || (!world[output_position].kind.is_torch()
            && !world[output_position].kind.is_switch()
            && !world[output_position].kind.is_repeater())
    {
        return vec![output_position];
    }

    let mut access_points = world
        .iter_block()
        .into_iter()
        .filter(|(position, block)| {
            block.kind.is_redstone()
                && detailed_router::target_powers_position(world, output_position, *position)
        })
        .map(|(position, _)| position)
        .collect::<Vec<_>>();
    access_points.sort_by_key(|position| {
        (
            output_position.manhattan_distance(position),
            position.0,
            position.1,
            position.2,
        )
    });
    access_points.dedup();
    if access_points.is_empty() {
        access_points.push(output_position);
    }
    access_points
}

fn expose_routeable_output_port(world: &World3D, output_position: Position) -> Position {
    expose_routeable_output_ports(world, output_position)[0]
}

// LocalPlacer 입력은 보통 switch로 시작하므로 global PnR child layout에서는
// switch를 제거하고, switch가 물리던 cobble 또는 redstone fanout을 input port로 노출한다.
// TODO(low-level): replace this inference with explicit input-port placement metadata
// from LocalPlacer, so this code does not need to guess from switch wiring.
fn expose_switchless_input_port(
    world: &mut World3D,
    input_position: Position,
    preserve_switch_position_input: bool,
    use_direct_input_port: bool,
) -> Option<Position> {
    if !world.size.bound_on(input_position) {
        return None;
    }
    if !world[input_position].kind.is_switch() {
        return Some(input_position);
    }

    let switch_target = input_position.walk(world[input_position].direction);
    if let Some(target) = switch_target
        .filter(|position| world.size.bound_on(*position) && world[*position].kind.is_cobble())
    {
        if use_direct_input_port {
            if let Some(port_position) = switch_powered_redstone_port(world, input_position, true) {
                world[input_position] = Block::default();
                return Some(port_position);
            }
        }
        world[input_position] = Block::default();
        return Some(target);
    }

    if preserve_switch_position_input && switch_powers_redstone(world, input_position) {
        ensure_redstone_support(world, input_position)?;
        world[input_position] = PlacedNode::new_redstone(input_position).block;
        return Some(input_position);
    }

    if let Some(port_position) =
        switch_powered_redstone_port(world, input_position, use_direct_input_port)
    {
        world[input_position] = Block::default();
        return Some(port_position);
    }

    let port_position = expose_routeable_output_port(world, input_position);
    (port_position != input_position).then(|| {
        world[input_position] = Block::default();
        port_position
    })
}

fn switch_powers_redstone(world: &World3D, input_position: Position) -> bool {
    world.iter_block().into_iter().any(|(position, block)| {
        block.kind.is_redstone()
            && detailed_router::target_powers_position(world, input_position, position)
    })
}

fn ensure_redstone_support(world: &mut World3D, position: Position) -> Option<()> {
    let support_position = position.down()?;
    if !world.size.bound_on(support_position) {
        return None;
    }
    if world[support_position].kind.is_cobble() {
        return Some(());
    }
    if !world[support_position].kind.is_air() {
        return None;
    }
    world[support_position] = PlacedNode::new_cobble(support_position).block;
    Some(())
}

fn switch_powered_redstone_port(
    world: &World3D,
    input_position: Position,
    direct_only: bool,
) -> Option<Position> {
    let direct = world
        .iter_block()
        .into_iter()
        .filter_map(|(position, block)| {
            (block.kind.is_redstone()
                && detailed_router::target_powers_position(world, input_position, position))
            .then_some(position)
        })
        .collect::<Vec<_>>();
    if direct.is_empty() {
        return None;
    }

    let candidates = if direct_only {
        direct
    } else {
        redstone_network_positions(world, &direct)
    };

    candidates.into_iter().max_by_key(|position| {
        (
            downstream_consumer_count(world, *position),
            Reverse(input_position.manhattan_distance(position)),
            Reverse(position.0),
            Reverse(position.1),
            Reverse(position.2),
        )
    })
}

fn redstone_network_positions(world: &World3D, seeds: &[Position]) -> Vec<Position> {
    let redstones = world
        .iter_block()
        .into_iter()
        .filter_map(|(position, block)| block.kind.is_redstone().then_some(position))
        .collect::<Vec<_>>();
    let mut visited = HashSet::new();
    let mut queue = VecDeque::new();
    for &seed in seeds {
        if visited.insert(seed) {
            queue.push_back(seed);
        }
    }

    while let Some(position) = queue.pop_front() {
        for &next in &redstones {
            if visited.contains(&next) {
                continue;
            }
            if detailed_router::target_powers_position(world, position, next)
                || detailed_router::target_powers_position(world, next, position)
            {
                visited.insert(next);
                queue.push_back(next);
            }
        }
    }

    visited.into_iter().collect()
}

fn downstream_consumer_count(world: &World3D, source: Position) -> usize {
    world
        .iter_block()
        .into_iter()
        .filter(|(position, block)| {
            *position != source
                && !block.kind.is_redstone()
                && detailed_router::target_powers_position(world, source, *position)
        })
        .count()
}

pub fn d_latch_child_candidate_config(local_config: LocalPlacerConfig) -> UnitCandidateConfig {
    UnitCandidateConfig {
        dim: DimSize(14, 10, 6),
        local_config,
        input_constraints: LocalPlacerInputConstraints::new()
            .with_input_positions("d", [Position(0, 2, 1)])
            .with_input_positions("en", [Position(0, 6, 1)]),
        max_candidates: 1,
        combinational_sampling_limit: None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::world::block::{BlockKind, Direction};

    #[test]
    fn candidate_pin_search_is_scoped_by_definition_and_port() {
        let policies = CandidatePolicySet::default()
            .with_pin_search("first", "d", [Position(1, 2, 3)])
            .with_pin_search("second", "d", [Position(4, 5, 1)]);

        assert_eq!(
            policies
                .effective_for_definition("first")
                .input_constraints
                .positions_for_input_name("d"),
            Some(vec![Position(1, 2, 3)])
        );
        assert_eq!(
            policies
                .effective_for_definition("second")
                .input_constraints
                .positions_for_input_name("d"),
            Some(vec![Position(4, 5, 1)])
        );
        assert_eq!(
            policies
                .effective_for_definition("third")
                .input_constraints
                .positions_for_input_name("d"),
            None
        );
    }

    #[test]
    fn switchless_direct_input_exposes_powered_redstone_instead_of_support_cobble() {
        let switch = Position(1, 1, 1);
        let support = Position(1, 1, 0);
        let input_cobble = Position(2, 1, 1);
        let input_redstone = Position(2, 1, 2);
        let mut world = World3D::new(DimSize(4, 3, 4));
        world[support] = PlacedNode::new_cobble(support).block;
        world[switch] = Block {
            kind: BlockKind::Switch { is_on: false },
            direction: Direction::East,
        };
        world[input_cobble] = PlacedNode::new_cobble(input_cobble).block;
        world[input_redstone] = PlacedNode::new_redstone(input_redstone).block;
        world.initialize_redstone_states();

        let port =
            expose_switchless_input_port(&mut world, switch, false, true).expect("input port");

        assert_eq!(port, input_redstone);
        assert!(world[switch].kind.is_air());
    }

    #[test]
    fn cell_library_implementation_overrides_default_but_not_explicit_override() {
        use super::super::cell_library::{CellImplementation, CellLibrary, CellPhysicalContract};

        let library = CellLibrary {
            implementations: vec![CellImplementation {
                name: "d_latch.compact".to_owned(),
                definitions: vec!["d_latch".to_owned()],
                candidate: UnitCandidateConfig {
                    max_candidates: 7,
                    ..Default::default()
                },
                contract: CellPhysicalContract::default(),
                priority: 1,
            }],
            ..CellLibrary::redstone_v1()
        };
        let policies = CandidatePolicySet::default().with_cell_library(library.clone());

        assert_eq!(
            policies.effective_for_definition("d_latch").max_candidates,
            7
        );
        assert_eq!(
            policies.effective_for_definition("other").max_candidates,
            UnitCandidateConfig::default().max_candidates
        );

        let explicit = policies.with_definition_override(
            "d_latch",
            UnitCandidateConfig {
                max_candidates: 9,
                ..Default::default()
            },
        );
        assert_eq!(
            explicit.effective_for_definition("d_latch").max_candidates,
            9
        );
    }

    fn inverter_module() -> crate::ir::RoutableModule {
        use crate::ir::{
            RoutableModuleBody, RoutableNode, RoutableNodeKind, RoutablePort, RoutablePortDirection,
        };

        crate::ir::RoutableModule {
            name: "inv".to_owned(),
            ports: vec![
                RoutablePort {
                    name: "a".to_owned(),
                    direction: RoutablePortDirection::Input,
                },
                RoutablePort {
                    name: "y".to_owned(),
                    direction: RoutablePortDirection::Output,
                },
            ],
            body: RoutableModuleBody::Leaf {
                nodes: vec![
                    RoutableNode {
                        id: 0,
                        kind: RoutableNodeKind::Input {
                            name: "a".to_owned(),
                        },
                        inputs: Vec::new(),
                        tag: String::new(),
                    },
                    RoutableNode {
                        id: 1,
                        kind: RoutableNodeKind::Not,
                        inputs: vec![0],
                        tag: String::new(),
                    },
                    RoutableNode {
                        id: 2,
                        kind: RoutableNodeKind::Output {
                            name: "y".to_owned(),
                        },
                        inputs: vec![1],
                        tag: String::new(),
                    },
                ],
            },
        }
    }

    #[test]
    fn generated_candidates_form_a_pareto_frontier() -> eyre::Result<()> {
        let module = inverter_module();
        let config = UnitCandidateConfig {
            dim: DimSize(6, 6, 3),
            max_candidates: 4,
            local_config: LocalPlacerConfig {
                materialize_outputs: true,
                ..Default::default()
            },
            ..Default::default()
        };

        let candidates =
            generate_routable_module_candidates_with_progress_label(&module, &config, None, None)?;

        assert!(
            !candidates.is_empty(),
            "inverter leaf must produce at least one candidate"
        );
        for (left_index, left) in candidates.iter().enumerate() {
            for (right_index, right) in candidates.iter().enumerate() {
                if left_index == right_index {
                    continue;
                }
                assert!(
                    !super::super::ir::dominates(&left.cost, &right.cost),
                    "candidate {left_index} dominates candidate {right_index}"
                );
            }
        }
        Ok(())
    }

    #[test]
    fn contract_halo_and_isolation_reach_generated_candidates() -> eyre::Result<()> {
        use super::super::cell_library::CellPhysicalContract;

        let module = inverter_module();
        let contract = CellPhysicalContract {
            halo: 2,
            requires_output_isolation: true,
            ..Default::default()
        };
        let config = UnitCandidateConfig {
            dim: DimSize(6, 6, 3),
            max_candidates: 2,
            local_config: LocalPlacerConfig {
                materialize_outputs: true,
                ..Default::default()
            },
            ..Default::default()
        };

        let candidates = generate_routable_module_candidates_with_progress_label(
            &module,
            &config,
            Some(&contract),
            None,
        )?;

        assert!(!candidates.is_empty());
        for candidate in &candidates {
            assert_eq!(candidate.halo, 2);
            assert!(candidate.placement_bbox().width() > candidate.bbox.width());
            let output = candidate
                .ports
                .iter()
                .find(|port| port.direction == PhysicalPortDirection::Output)
                .expect("output port");
            assert_eq!(output.connection, PortConnection::OutputDiode);
        }
        Ok(())
    }

    #[test]
    fn effective_contract_resolves_from_the_cell_library() {
        use super::super::cell_library::{CellImplementation, CellLibrary, CellPhysicalContract};

        let library = CellLibrary {
            implementations: vec![CellImplementation {
                name: "inv.isolated".to_owned(),
                definitions: vec!["inv".to_owned()],
                candidate: UnitCandidateConfig::default(),
                contract: CellPhysicalContract {
                    halo: 1,
                    requires_input_isolation: true,
                    ..Default::default()
                },
                priority: 0,
            }],
            ..CellLibrary::redstone_v1()
        };
        let policies = CandidatePolicySet::default().with_cell_library(library);

        let contract = policies
            .effective_contract_for_definition("inv")
            .expect("matching contract");
        assert_eq!(contract.halo, 1);
        assert!(contract.requires_input_isolation);
        assert!(policies
            .effective_contract_for_definition("other")
            .is_none());
    }

    fn repro_node(
        id: usize,
        kind: crate::ir::RoutableNodeKind,
        inputs: Vec<usize>,
    ) -> crate::ir::RoutableNode {
        crate::ir::RoutableNode {
            id,
            kind,
            inputs,
            tag: String::new(),
        }
    }

    fn repro_leaf(name: &str, nodes: Vec<crate::ir::RoutableNode>) -> crate::ir::RoutableModule {
        use crate::ir::{RoutableModule, RoutableModuleBody, RoutablePort, RoutablePortDirection};
        let ports = nodes
            .iter()
            .filter_map(|node| match &node.kind {
                crate::ir::RoutableNodeKind::Input { name } => Some(RoutablePort {
                    name: name.clone(),
                    direction: RoutablePortDirection::Input,
                }),
                crate::ir::RoutableNodeKind::Output { name } => Some(RoutablePort {
                    name: name.clone(),
                    direction: RoutablePortDirection::Output,
                }),
                _ => None,
            })
            .collect();
        RoutableModule {
            name: name.to_owned(),
            ports,
            body: RoutableModuleBody::Leaf { nodes },
        }
    }

    fn repro_state_next_nodes(output_input: usize) -> Vec<crate::ir::RoutableNode> {
        use crate::ir::RoutableNodeKind::{Constant, Input, Not, Or, Output};
        vec![
            repro_node(0, Constant { value: true }, vec![]),
            repro_node(
                1,
                Input {
                    name: "go".to_owned(),
                },
                vec![],
            ),
            repro_node(3, Not, vec![0]),
            repro_node(
                5,
                Input {
                    name: "state".to_owned(),
                },
                vec![],
            ),
            repro_node(8, Or, vec![1, 18]),
            repro_node(
                14,
                Output {
                    name: "__next".to_owned(),
                },
                vec![output_input],
            ),
            repro_node(16, Not, vec![5]),
            repro_node(17, Or, vec![1, 16]),
            repro_node(18, Not, vec![17]),
            repro_node(19, Not, vec![8]),
            repro_node(20, Not, vec![16]),
            repro_node(21, Or, vec![19, 20]),
            repro_node(22, Not, vec![21]),
        ]
    }

    fn repro_base_config() -> UnitCandidateConfig {
        use crate::transform::place_and_route::local_placer::{
            InputPlacementStrategy, LocalPlacerConfig, NotRouteStrategy, PlacementSamplingPolicy,
            TorchPlacementStrategy,
        };
        use crate::transform::place_and_route::sampling::SamplingPolicy;
        let local_config = LocalPlacerConfig {
            random_seed: 42,
            greedy_input_generation: true,
            input_placement_strategy: InputPlacementStrategy::Boundary,
            input_candidate_limit: None,
            step_sampling_policy: SamplingPolicy::Random(32),
            placement_sampling_policy: PlacementSamplingPolicy::StepPolicy,
            leak_sampling: false,
            route_torch_directly: true,
            materialize_outputs: false,
            torch_placement_strategy: TorchPlacementStrategy::DirectOnly,
            not_route_strategy: NotRouteStrategy::DirectAndRedstone,
            max_not_route_step: 6,
            not_route_step_sampling_policy: SamplingPolicy::Random(32),
            max_route_step: 8,
            route_step_sampling_policy: SamplingPolicy::Random(32),
        };
        UnitCandidateConfig {
            dim: crate::world::position::DimSize(16, 16, 6),
            local_config,
            max_candidates: 2,
            combinational_sampling_limit: Some(32),
            ..Default::default()
        }
    }

    fn run_reproducer(
        label: &str,
        nodes: Vec<crate::ir::RoutableNode>,
        base: &UnitCandidateConfig,
    ) -> eyre::Result<()> {
        crate::perf::reset_for_tests();
        let module = repro_leaf(label, nodes);
        let config =
            crate::transform::place_and_route::global_pnr::candidate_config_for_routable_child(
                &module, base,
            );
        let candidates =
            generate_routable_module_candidates_with_progress_label(&module, &config, None, None)?;
        eprintln!(
            "[repro] {label}: candidates={} truth_rejects={} port_rejects={}",
            candidates.len(),
            crate::perf::candidate_truth_rejects(),
            crate::perf::candidate_port_rejects()
        );
        Ok(())
    }

    #[test]
    #[ignore = "diagnostic: state_next truth-table reproducer"]
    fn state_next_graph_candidate_truth_reproducer() -> eyre::Result<()> {
        let base = repro_base_config();
        run_reproducer("full", repro_state_next_nodes(22), &base)?;

        let no_dead = repro_state_next_nodes(22)
            .into_iter()
            .filter(|node| node.id != 0 && node.id != 3)
            .collect();
        run_reproducer("no_dead", no_dead, &base)?;

        run_reproducer("tail_n8", repro_state_next_nodes(8), &base)?;
        run_reproducer("tail_n19", repro_state_next_nodes(19), &base)?;
        run_reproducer("tail_n20", repro_state_next_nodes(20), &base)?;
        run_reproducer("tail_n21", repro_state_next_nodes(21), &base)?;

        let mut state_direct = repro_state_next_nodes(22);
        for node in &mut state_direct {
            if node.id == 20 {
                node.inputs = vec![5];
            }
        }
        run_reproducer("state_direct", state_direct, &base)?;

        let mut n8_direct = repro_state_next_nodes(22);
        for node in &mut n8_direct {
            if node.id == 8 {
                node.inputs = vec![1, 5];
            }
        }
        run_reproducer("n8_direct", n8_direct, &base)?;
        Ok(())
    }
}
