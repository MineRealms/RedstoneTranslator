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

#[derive(Clone, Debug)]
pub struct PinRecord {
    pub node: GraphNodeId,
    pub position: Position,
    pub contract: PinContract,
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
    pub node: GraphNodeId,
    pub position: Position,
    pub drivers: Vec<(GraphNodeId, Position)>,
    pub reason: ViolationReason,
    pub confidence: ConnectivityConfidence,
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

/// The positions a logical net is allowed to drive from, for a `Single` pin.
/// A terminal net drives from its own block plus any redstone component it
/// powers; an OR net drives from the redstone component containing its tap.
fn allowed_driver_positions(
    net: GraphNodeId,
    world: &World3D,
    anchor: &HashMap<GraphNodeId, Position>,
    component_of: &HashMap<Position, usize>,
    component_positions: &HashMap<usize, HashSet<Position>>,
    component_nets: &HashMap<usize, BTreeSet<GraphNodeId>>,
) -> HashSet<Position> {
    let Some(anchor_pos) = anchor.get(&net).copied() else {
        return HashSet::new();
    };

    let mut allowed = HashSet::new();
    if world[anchor_pos].kind.is_redstone() {
        if let Some(&cid) = component_of.get(&anchor_pos) {
            if let Some(positions) = component_positions.get(&cid) {
                allowed.extend(positions.iter().copied());
            }
        }
        return allowed;
    }

    allowed.insert(anchor_pos);
    for (cid, nets) in component_nets {
        // A terminal net may only drive a component that carries exactly that
        // net; a component shared with a foreign net is a short.
        if nets.len() == 1 && nets.contains(&net) {
            if let Some(positions) = component_positions.get(cid) {
                allowed.extend(positions.iter().copied());
            }
        }
    }
    allowed
}

fn driver_nets(
    source: Position,
    world: &World3D,
    position_to_net: &HashMap<Position, GraphNodeId>,
    component_of: &HashMap<Position, usize>,
    component_nets: &HashMap<usize, BTreeSet<GraphNodeId>>,
) -> Vec<GraphNodeId> {
    if world[source].kind.is_redstone() {
        let Some(&cid) = component_of.get(&source) else {
            return Vec::new();
        };
        component_nets.get(&cid).map(|nets| nets.iter().copied().collect()).unwrap_or_default()
    } else {
        position_to_net
            .get(&source)
            .copied()
            .into_iter()
            .collect()
    }
}

pub fn analyze(world: &World3D, analysis: &PlacedAnalysis) -> Vec<Violation> {
    let mut anchor = HashMap::new();
    let mut position_to_net = HashMap::new();
    for (net, pos) in &analysis.anchors {
        anchor.insert(*net, *pos);
        position_to_net.entry(*pos).or_insert(*net);
    }

    let (component_of, component_positions) = electrical_components(world);
    let component_nets = terminal_nets_per_component(world, &component_of, &anchor);

    let mut violations = Vec::new();
    for pin in &analysis.pins {
        match &pin.contract {
            PinContract::Single { expected_net } => {
                let allowed = allowed_driver_positions(
                    *expected_net,
                    world,
                    &anchor,
                    &component_of,
                    &component_positions,
                    &component_nets,
                );
                for (source, _) in electrical::cobble_power_sources(world, pin.position) {
                    if allowed.contains(&source) {
                        continue;
                    }
                    let drivers = driver_nets(source, world, &position_to_net, &component_of, &component_nets)
                        .into_iter()
                        .map(|net| (net, source))
                        .collect::<Vec<_>>();
                    if drivers.is_empty() {
                        // A driver with no possible driving net is never powered;
                        // it is not a violation.
                        continue;
                    }
                    violations.push(Violation {
                        node: pin.node,
                        position: pin.position,
                        drivers,
                        reason: ViolationReason::ExtraDriver,
                        confidence: ConnectivityConfidence::Certain,
                    });
                }
            }
            PinContract::Merge { input_nets } => {
                let expected: BTreeSet<GraphNodeId> = input_nets.iter().copied().collect();
                let Some(&cid) = component_of.get(&pin.position) else {
                    for missing in &expected {
                        violations.push(Violation {
                            node: pin.node,
                            position: pin.position,
                            drivers: Vec::new(),
                            reason: ViolationReason::MissingBranch(*missing),
                            confidence: ConnectivityConfidence::SimulationRequired,
                        });
                    }
                    continue;
                };
                let actual = component_nets.get(&cid).cloned().unwrap_or_default();
                for missing in expected.difference(&actual) {
                    violations.push(Violation {
                        node: pin.node,
                        position: pin.position,
                        drivers: Vec::new(),
                        reason: ViolationReason::MissingBranch(*missing),
                        confidence: ConnectivityConfidence::SimulationRequired,
                    });
                }
                for foreign in actual.difference(&expected) {
                    violations.push(Violation {
                        node: pin.node,
                        position: pin.position,
                        drivers: vec![(*foreign, pin.position)],
                        reason: ViolationReason::ForeignNet(*foreign),
                        confidence: ConnectivityConfidence::SimulationRequired,
                    });
                }
            }
            PinContract::Passive => {}
        }
    }

    violations.sort_by_key(|v| (v.node, v.position, v.reason.clone(), v.confidence));
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
            vec![PinRecord {
                node: 20,
                position: n20_support,
                contract: PinContract::Single { expected_net: 16 },
            }],
        );

        let violations = analyze(&w, &a);
        assert_eq!(
            violations,
            vec![Violation {
                node: 20,
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
                PinRecord {
                    node: 16,
                    position: n16_support,
                    contract: PinContract::Single { expected_net: 5 },
                },
                PinRecord {
                    node: 20,
                    position: n20_support,
                    contract: PinContract::Single { expected_net: 16 },
                },
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
                PinRecord {
                    node: 16,
                    position: Position(1, 3, 1),
                    contract: PinContract::Single { expected_net: 7 },
                },
                PinRecord {
                    node: 17,
                    position: Position(3, 3, 1),
                    contract: PinContract::Single { expected_net: 7 },
                },
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
            vec![PinRecord {
                node: 17,
                position: d2,
                contract: PinContract::Merge {
                    input_nets: vec![1, 16],
                },
            }],
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
            vec![PinRecord {
                node: 17,
                position: d2,
                contract: PinContract::Merge {
                    input_nets: vec![1, 16],
                },
            }],
        );

        assert_eq!(
            analyze(&w, &a),
            vec![Violation {
                node: 17,
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
            vec![PinRecord {
                node: 17,
                position: d2,
                contract: PinContract::Merge {
                    input_nets: vec![1, 16],
                },
            }],
        );

        assert_eq!(
            analyze(&w, &a),
            vec![Violation {
                node: 17,
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
            vec![PinRecord {
                node: 17,
                position: d2,
                contract: PinContract::Merge {
                    input_nets: vec![1, 16],
                },
            }],
        );

        assert!(analyze(&w, &a).is_empty());
    }
}
