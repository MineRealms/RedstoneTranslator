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
}

fn redstone_components(
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
            let crate::world::block::BlockKind::Redstone { state, .. } = world[p].kind else {
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

/// For every redstone component, the set of logical nets whose driver terminal
/// can power a redstone inside it.
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
            if world.size.bound_on(target) && world[target].kind.is_redstone() {
                if let Some(&cid) = component_of.get(&target) {
                    result.entry(cid).or_default().insert(*net);
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
        if nets.contains(&net) {
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

    let (component_of, component_positions) = redstone_components(world);
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
                    });
                }
                for foreign in actual.difference(&expected) {
                    violations.push(Violation {
                        node: pin.node,
                        position: pin.position,
                        drivers: vec![(*foreign, pin.position)],
                        reason: ViolationReason::ForeignNet(*foreign),
                    });
                }
            }
            PinContract::Passive => {}
        }
    }

    violations.sort_by_key(|v| (v.node, v.position, v.reason.clone()));
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
}
