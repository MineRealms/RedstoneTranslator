//! Physical electrical connectivity analysis (PECA): compile-time electrical
//! facts plus the DRC rules that consume them.
//!
//! The local placer emits, alongside each candidate, a `PlacedAnalysis` that
//! records every node's output anchor and every pin's electrical contract.
//! This module re-derives the physical electrical structure from the world
//! (redstone components and their driving terminals) using the shared rules in
//! `world::electrical`, then checks each pin contract. It never modifies the
//! world, the simulator, or any output; violations are reported only.

use std::collections::{BTreeSet, HashMap, HashSet};

use crate::graph::GraphNodeId;
use crate::world::block::BlockKind;
use crate::world::electrical;
use crate::world::position::Position;
use crate::world::World3D;

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum PinContract {
    Single {
        expected_net: GraphNodeId,
    },
    Merge {
        input_nets: Vec<GraphNodeId>,
    },
    Passive,
}

/// A port on a node. Primitives use positional ports (`Index`); sequential and
/// macro pins use names, reusing the `MacroPin.name` convention.
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub enum PinPort {
    Named(String),
    Index(usize),
}

/// The physical identity of a pin. The logical net identity stays
/// `GraphNodeId` (the truth-table identity); the port qualifies the pin.
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub struct PinId {
    pub node: GraphNodeId,
    pub port: PinPort,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum PinIsolationPolicy {
    /// NOT/repeater/latch input: exactly one logical source.
    SingleSource,
    /// OR tap: the union of the declared input nets.
    Merge,
    /// Sequential-macro internals are not checked in the first release.
    None,
}

#[derive(Clone, Debug)]
pub struct PinRecord {
    pub id: PinId,
    pub position: Position,
    pub contract: PinContract,
    pub policy: PinIsolationPolicy,
}

impl PinRecord {
    pub fn new(
        node: GraphNodeId,
        port: PinPort,
        position: Position,
        contract: PinContract,
    ) -> Self {
        let policy = match &contract {
            PinContract::Single { .. } => PinIsolationPolicy::SingleSource,
            PinContract::Merge { .. } => PinIsolationPolicy::Merge,
            PinContract::Passive => PinIsolationPolicy::None,
        };
        Self {
            id: PinId { node, port },
            position,
            contract,
            policy,
        }
    }
}

#[derive(Clone, Debug, Default)]
pub struct PlacedAnalysis {
    pub anchors: Vec<(GraphNodeId, Position)>,
    pub pins: Vec<PinRecord>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum ConnectivityConfidence {
    /// The violation follows directly from the shared electrical rules; safe to
    /// enforce.
    Certain,
    /// Static analysis cannot decide; the candidate must be simulated before
    /// the violation is enforced.
    SimulationRequired,
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum ViolationReason {
    ExtraDriver,
    MissingBranch(GraphNodeId),
    ForeignNet(GraphNodeId),
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Violation {
    pub pin: PinId,
    pub position: Position,
    pub drivers: Vec<(GraphNodeId, Position)>,
    pub reason: ViolationReason,
    pub confidence: ConnectivityConfidence,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum DrcResult {
    Pass,
    Reject(Vec<Violation>),
}

pub fn debug_enabled() -> bool {
    use std::sync::atomic::{AtomicU8, Ordering};
    static FLAG: AtomicU8 = AtomicU8::new(0);
    match FLAG.load(Ordering::Relaxed) {
        1 => true,
        2 => false,
        _ => {
            let enabled = std::env::var_os("MCHDL_DEBUG_PECA").is_some();
            FLAG.store(if enabled { 1 } else { 2 }, Ordering::Relaxed);
            enabled
        }
    }
}

/// Electrical components over redstone dust. Dust connects to dust; a cobble
/// only relays terminal power one hop (see `terminal_nets_per_component`),
/// because the simulator ignores redstone events on cobbles.
fn electrical_components(
    world: &World3D,
) -> (
    HashMap<Position, usize>,
    HashMap<usize, HashSet<Position>>,
) {
    let mut component_of = HashMap::new();
    let mut component_positions = HashMap::new();
    let mut next_id = 0usize;

    for (pos, block) in world.iter_block() {
        if !block.kind.is_redstone() || component_of.contains_key(&pos) {
            continue;
        }
        let mut stack = vec![pos];
        let mut positions = HashSet::new();
        component_of.insert(pos, next_id);
        positions.insert(pos);
        while let Some(p) = stack.pop() {
            let BlockKind::Redstone { state, .. } = world[p].kind else {
                continue;
            };
            for target in electrical::redstone_propagate_targets(world, p, state) {
                if world.size.bound_on(target)
                    && world[target].kind.is_redstone()
                    && !component_of.contains_key(&target)
                {
                    component_of.insert(target, next_id);
                    positions.insert(target);
                    stack.push(target);
                }
            }
        }
        component_positions.insert(next_id, positions);
        next_id += 1;
    }

    (component_of, component_positions)
}

/// For every dust component, the set of logical nets whose driver terminal can
/// power a redstone inside it. A terminal that powers a cobble relays its net
/// one hop to the cobble's adjacent redstone (the simulator's cobble event
/// handling); dust never relays through a cobble.
fn terminal_nets_per_component(
    world: &World3D,
    component_of: &HashMap<Position, usize>,
    anchor: &HashMap<GraphNodeId, Position>,
) -> HashMap<usize, BTreeSet<GraphNodeId>> {
    let mut result: HashMap<usize, BTreeSet<GraphNodeId>> = HashMap::new();
    for (net, pos) in anchor {
        let kind = world[*pos].kind;
        if kind.is_redstone() || kind.is_cobble() || kind.is_air() {
            continue;
        }
        for (target, _) in electrical::power_targets(world, *pos) {
            if !world.size.bound_on(target) {
                continue;
            }
            let target_kind = world[target].kind;
            if target_kind.is_redstone() {
                if let Some(&cid) = component_of.get(&target) {
                    result.entry(cid).or_default().insert(*net);
                }
            } else if target_kind.is_cobble() {
                for neighbor in target.forwards() {
                    if world.size.bound_on(neighbor) && world[neighbor].kind.is_redstone() {
                        if let Some(&cid) = component_of.get(&neighbor) {
                            result.entry(cid).or_default().insert(*net);
                        }
                    }
                }
            }
        }
    }
    result
}

/// Precomputed electrical facts about a placed world plus its anchors: dust
/// components, the nets that drive each component, and the position -> net map.
pub struct NetIndex {
    anchor: HashMap<GraphNodeId, Position>,
    position_to_net: HashMap<Position, GraphNodeId>,
    component_of: HashMap<Position, usize>,
    component_positions: HashMap<usize, HashSet<Position>>,
    component_nets: HashMap<usize, BTreeSet<GraphNodeId>>,
}

impl NetIndex {
    pub fn build(world: &World3D, anchors: &[(GraphNodeId, Position)]) -> Self {
        let mut anchor = HashMap::new();
        let mut position_to_net = HashMap::new();
        for (net, pos) in anchors {
            anchor.insert(*net, *pos);
            position_to_net.entry(*pos).or_insert(*net);
        }
        let (component_of, component_positions) = electrical_components(world);
        let component_nets = terminal_nets_per_component(world, &component_of, &anchor);
        Self {
            anchor,
            position_to_net,
            component_of,
            component_positions,
            component_nets,
        }
    }

    /// The positions a logical net is allowed to drive from, for a `Single`
    /// pin. A terminal net drives from its own block plus any dust component
    /// that carries exactly that net; an OR net drives from the dust component
    /// containing its tap.
    fn allowed_driver_positions(&self, net: GraphNodeId, world: &World3D) -> HashSet<Position> {
        let Some(anchor_pos) = self.anchor.get(&net).copied() else {
            return HashSet::new();
        };

        let mut allowed = HashSet::new();
        if world[anchor_pos].kind.is_redstone() {
            if let Some(&cid) = self.component_of.get(&anchor_pos) {
                if let Some(positions) = self.component_positions.get(&cid) {
                    allowed.extend(positions.iter().copied());
                }
            }
            return allowed;
        }

        allowed.insert(anchor_pos);
        for (cid, nets) in &self.component_nets {
            // A terminal net may only drive a component that carries exactly
            // that net; a component shared with a foreign net is a short.
            if nets.len() == 1 && nets.contains(&net) {
                if let Some(positions) = self.component_positions.get(cid) {
                    allowed.extend(positions.iter().copied());
                }
            }
        }
        allowed
    }

    fn driver_nets(&self, source: Position, world: &World3D) -> Vec<GraphNodeId> {
        if world[source].kind.is_redstone() {
            let Some(&cid) = self.component_of.get(&source) else {
                return Vec::new();
            };
            self.component_nets
                .get(&cid)
                .map(|nets| nets.iter().copied().collect())
                .unwrap_or_default()
        } else {
            self.position_to_net
                .get(&source)
                .copied()
                .into_iter()
                .collect()
        }
    }
}

/// Check a single pin against the current world. Used by `analyze` (after
/// placement) and by the pre-route hook, where the connecting route does not
/// exist yet and a `Single` pin only requires the absence of foreign drivers.
pub fn check_pin(world: &World3D, index: &NetIndex, pin: &PinRecord) -> DrcResult {
    let mut violations = Vec::new();
    match &pin.contract {
        PinContract::Single { expected_net } => {
            let allowed = index.allowed_driver_positions(*expected_net, world);
            for (source, _) in electrical::cobble_power_sources(world, pin.position) {
                if allowed.contains(&source) {
                    continue;
                }
                let drivers = index
                    .driver_nets(source, world)
                    .into_iter()
                    .map(|net| (net, source))
                    .collect::<Vec<_>>();
                if drivers.is_empty() {
                    // A driver with no possible driving net is never powered;
                    // it is not a violation.
                    continue;
                }
                violations.push(Violation {
                    pin: pin.id.clone(),
                    position: pin.position,
                    drivers,
                    reason: ViolationReason::ExtraDriver,
                    confidence: ConnectivityConfidence::Certain,
                });
            }
        }
        PinContract::Merge { input_nets } => {
            let expected: BTreeSet<GraphNodeId> = input_nets.iter().copied().collect();
            let actual = index
                .component_of
                .get(&pin.position)
                .and_then(|cid| index.component_nets.get(cid))
                .cloned()
                .unwrap_or_default();
            for missing in expected.difference(&actual) {
                violations.push(Violation {
                    pin: pin.id.clone(),
                    position: pin.position,
                    drivers: Vec::new(),
                    reason: ViolationReason::MissingBranch(*missing),
                    confidence: ConnectivityConfidence::SimulationRequired,
                });
            }
            for foreign in actual.difference(&expected) {
                violations.push(Violation {
                    pin: pin.id.clone(),
                    position: pin.position,
                    drivers: vec![(*foreign, pin.position)],
                    reason: ViolationReason::ForeignNet(*foreign),
                    confidence: ConnectivityConfidence::SimulationRequired,
                });
            }
        }
        PinContract::Passive => {}
    }

    if violations.is_empty() {
        DrcResult::Pass
    } else {
        DrcResult::Reject(violations)
    }
}

/// Check whether a newly placed terminal (torch, switch, redstone block,
/// repeater) would power an existing foreign pin. This catches coupling that
/// is introduced *after* the pin was placed (the reverse of `check_pin`).
pub fn check_new_driver_against_pins(
    world: &World3D,
    new_net: GraphNodeId,
    driver: Position,
    pins: &[PinRecord],
) -> DrcResult {
    let mut violations = Vec::new();
    for (target, _) in electrical::power_targets(world, driver) {
        for pin in pins {
            if pin.position != target {
                continue;
            }
            let allowed = match &pin.contract {
                PinContract::Single { expected_net } => *expected_net == new_net,
                PinContract::Merge { input_nets } => input_nets.contains(&new_net),
                PinContract::Passive => true,
            };
            if !allowed {
                violations.push(Violation {
                    pin: pin.id.clone(),
                    position: pin.position,
                    drivers: vec![(new_net, driver)],
                    reason: ViolationReason::ExtraDriver,
                    confidence: ConnectivityConfidence::Certain,
                });
            }
        }
    }

    if violations.is_empty() {
        DrcResult::Pass
    } else {
        DrcResult::Reject(violations)
    }
}

pub fn analyze(world: &World3D, analysis: &PlacedAnalysis) -> Vec<Violation> {
    let index = NetIndex::build(world, &analysis.anchors);
    let mut violations = Vec::new();
    for pin in &analysis.pins {
        if let DrcResult::Reject(pin_violations) = check_pin(world, &index, pin) {
            violations.extend(pin_violations);
        }
    }

    violations.sort_by_key(|v| (v.pin.node, v.position, v.reason.clone(), v.confidence));
    violations
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::world::block::{Block, BlockKind, Direction};
    use crate::world::position::DimSize;
    use crate::world::World;

    fn cobble() -> Block {
        Block {
            kind: BlockKind::Cobble {
                on_count: 0,
                on_base_count: 0,
            },
            direction: Direction::None,
        }
    }

    fn torch(direction: Direction) -> Block {
        Block {
            kind: BlockKind::Torch { is_on: false },
            direction,
        }
    }

    fn switch(direction: Direction) -> Block {
        Block {
            kind: BlockKind::Switch { is_on: false },
            direction,
        }
    }

    fn world(blocks: Vec<(Position, Block)>) -> World3D {
        World3D::from(&World {
            size: DimSize(8, 8, 4),
            blocks,
        })
    }

    fn analysis(anchors: Vec<(GraphNodeId, Position)>, pins: Vec<PinRecord>) -> PlacedAnalysis {
        PlacedAnalysis { anchors, pins }
    }

    /// The confirmed `state_next` shape: the second inverter's support cobble is
    /// adjacent to both the `state` input switch and the first inverter's torch,
    /// so the switch is a foreign driver.
    #[test]
    fn foreign_switch_adjacent_to_not_input_is_a_single_violation() {
        let state = Position(0, 5, 1);
        let n16 = Position(1, 4, 1);
        let n20 = Position(0, 3, 1);
        let n20_support = Position(0, 4, 1);
        let n16_support = Position(2, 4, 1);

        let w = world(vec![
            (state, switch(Direction::East)),
            (n16, torch(Direction::East)),
            (n16_support, cobble()),
            (n20, torch(Direction::North)),
            (n20_support, cobble()),
        ]);

        let a = analysis(
            vec![(5, state), (16, n16), (20, n20)],
            vec![PinRecord::new(
                20,
                PinPort::Index(0),
                n20_support,
                PinContract::Single { expected_net: 16 },
            )],
        );

        let violations = analyze(&w, &a);
        assert_eq!(
            violations,
            vec![Violation {
                pin: PinId {
                    node: 20,
                    port: PinPort::Index(0),
                },
                position: n20_support,
                drivers: vec![(5, state)],
                reason: ViolationReason::ExtraDriver,
                confidence: ConnectivityConfidence::Certain,
            }]
        );
    }

    /// A clean double inversion: the second support is driven only by the first
    /// torch, so there is no violation.
    #[test]
    fn double_not_with_single_driver_is_clean() {
        let state = Position(0, 5, 1);
        let n16 = Position(2, 4, 1);
        let n16_support = Position(3, 4, 1);
        let n20 = Position(1, 3, 1);
        let n20_support = Position(1, 4, 1);

        let w = world(vec![
            (state, switch(Direction::East)),
            (n16, torch(Direction::East)),
            (n16_support, cobble()),
            (n20, torch(Direction::North)),
            (n20_support, cobble()),
        ]);

        let a = analysis(
            vec![(5, state), (16, n16), (20, n20)],
            vec![
                PinRecord::new(
                    16,
                    PinPort::Index(0),
                    n16_support,
                    PinContract::Single { expected_net: 5 },
                ),
                PinRecord::new(
                    20,
                    PinPort::Index(0),
                    n20_support,
                    PinContract::Single { expected_net: 16 },
                ),
            ],
        );

        assert!(analyze(&w, &a).is_empty());
    }

    /// Fanout: one source driving two pins is not a violation.
    #[test]
    fn fanout_of_a_single_source_is_clean() {
        let source = Position(2, 3, 1);
        let n16 = Position(1, 2, 1);
        let n17 = Position(3, 2, 1);

        let w = world(vec![
            (
                source,
                Block {
                    kind: BlockKind::RedstoneBlock,
                    direction: Direction::None,
                },
            ),
            (n16, torch(Direction::North)),
            (Position(1, 3, 1), cobble()),
            (n17, torch(Direction::North)),
            (Position(3, 3, 1), cobble()),
        ]);

        let a = analysis(
            vec![(7, source), (16, n16), (17, n17)],
            vec![
                PinRecord::new(
                    16,
                    PinPort::Index(0),
                    Position(1, 3, 1),
                    PinContract::Single { expected_net: 7 },
                ),
                PinRecord::new(
                    17,
                    PinPort::Index(0),
                    Position(3, 3, 1),
                    PinContract::Single { expected_net: 7 },
                ),
            ],
        );

        assert!(analyze(&w, &a).is_empty());
    }

    fn dust() -> Block {
        Block {
            kind: BlockKind::Redstone {
                on_count: 0,
                state: 0,
                strength: 0,
            },
            direction: Direction::None,
        }
    }

    /// The `tail_n8` shape: the tap dust is reached by the second branch
    /// through a powered cobble (torch -> cobble -> dust). The cobble is a
    /// conducting relay, so both branches are observed and the merge is clean.
    #[test]
    fn or_merge_reached_through_cobble_is_clean() {
        let a_switch = Position(2, 4, 1);
        let d1 = Position(2, 3, 1);
        let d2 = Position(1, 3, 1);
        let c = Position(1, 3, 0);
        let b_torch = Position(0, 3, 0);

        let w = world(vec![
            (a_switch, switch(Direction::East)),
            (d1, dust()),
            (d2, dust()),
            (c, cobble()),
            (b_torch, torch(Direction::North)),
        ]);

        let a = analysis(
            vec![(1, a_switch), (16, b_torch)],
            vec![PinRecord::new(
                17,
                PinPort::Named("tap".to_owned()),
                d2,
                PinContract::Merge {
                    input_nets: vec![1, 16],
                },
            )],
        );

        assert!(analyze(&w, &a).is_empty());
    }

    /// The same shape without the second branch must report the missing input
    /// with `SimulationRequired` confidence (report-only, not enforced).
    #[test]
    fn or_merge_missing_branch_is_reported() {
        let a_switch = Position(2, 4, 1);
        let d1 = Position(2, 3, 1);
        let d2 = Position(1, 3, 1);
        let c = Position(1, 3, 0);

        let w = world(vec![
            (a_switch, switch(Direction::East)),
            (d1, dust()),
            (d2, dust()),
            (c, cobble()),
        ]);

        let a = analysis(
            vec![(1, a_switch)],
            vec![PinRecord::new(
                17,
                PinPort::Named("tap".to_owned()),
                d2,
                PinContract::Merge {
                    input_nets: vec![1, 16],
                },
            )],
        );

        assert_eq!(
            analyze(&w, &a),
            vec![Violation {
                pin: PinId {
                    node: 17,
                    port: PinPort::Named("tap".to_owned()),
                },
                position: d2,
                drivers: Vec::new(),
                reason: ViolationReason::MissingBranch(16),
                confidence: ConnectivityConfidence::SimulationRequired,
            }]
        );
    }

    /// A foreign source joining the merge zone must be reported.
    #[test]
    fn or_merge_foreign_driver_is_reported() {
        let a_switch = Position(2, 4, 1);
        let d1 = Position(2, 3, 1);
        let d2 = Position(1, 3, 1);
        let c = Position(1, 3, 0);
        let b_torch = Position(0, 3, 0);
        let foreign = Position(1, 2, 1);

        let w = world(vec![
            (a_switch, switch(Direction::East)),
            (d1, dust()),
            (d2, dust()),
            (c, cobble()),
            (b_torch, torch(Direction::North)),
            (foreign, switch(Direction::East)),
        ]);

        let a = analysis(
            vec![(1, a_switch), (16, b_torch), (99, foreign)],
            vec![PinRecord::new(
                17,
                PinPort::Named("tap".to_owned()),
                d2,
                PinContract::Merge {
                    input_nets: vec![1, 16],
                },
            )],
        );

        assert_eq!(
            analyze(&w, &a),
            vec![Violation {
                pin: PinId {
                    node: 17,
                    port: PinPort::Named("tap".to_owned()),
                },
                position: d2,
                drivers: vec![(99, d2)],
                reason: ViolationReason::ForeignNet(99),
                confidence: ConnectivityConfidence::SimulationRequired,
            }]
        );
    }

    /// Fanout into an OR input is not a merge violation.
    #[test]
    fn fanout_into_or_is_clean() {
        let a_switch = Position(2, 4, 1);
        let d1 = Position(2, 3, 1);
        let d2 = Position(1, 3, 1);
        let c = Position(1, 3, 0);
        let b_torch = Position(0, 3, 0);
        let d3 = Position(2, 5, 1);

        let w = world(vec![
            (a_switch, switch(Direction::East)),
            (d1, dust()),
            (d2, dust()),
            (c, cobble()),
            (b_torch, torch(Direction::North)),
            (d3, dust()),
        ]);

        let a = analysis(
            vec![(1, a_switch), (16, b_torch)],
            vec![PinRecord::new(
                17,
                PinPort::Named("tap".to_owned()),
                d2,
                PinContract::Merge {
                    input_nets: vec![1, 16],
                },
            )],
        );

        assert!(analyze(&w, &a).is_empty());
    }

    /// A terminal placed after a pin must not drive it unless it is the
    /// expected source (driver-side check).
    #[test]
    fn new_foreign_driver_hitting_an_existing_pin_is_reported() {
        let support = Position(1, 3, 1);
        let expected = Position(0, 3, 1);
        let foreign = Position(2, 3, 1);

        let w = world(vec![
            (support, cobble()),
            (expected, torch(Direction::North)),
            (foreign, torch(Direction::North)),
        ]);

        let pin = PinRecord::new(
            20,
            PinPort::Index(0),
            support,
            PinContract::Single { expected_net: 16 },
        );

        assert_eq!(
            check_new_driver_against_pins(&w, 16, expected, std::slice::from_ref(&pin)),
            DrcResult::Pass
        );

        assert_eq!(
            check_new_driver_against_pins(&w, 5, foreign, std::slice::from_ref(&pin)),
            DrcResult::Reject(vec![Violation {
                pin: PinId {
                    node: 20,
                    port: PinPort::Index(0),
                },
                position: support,
                drivers: vec![(5, foreign)],
                reason: ViolationReason::ExtraDriver,
                confidence: ConnectivityConfidence::Certain,
            }])
        );
    }
}
