//! Static electrical connectivity rules shared by the simulator and the PECA
//! (physical electrical connectivity analysis) layer.
//!
//! These are the single source of truth for "which blocks can power which
//! positions". The simulator consumes them to build its dynamic event model;
//! the DRC consumes them to enumerate possible drivers without simulating.
//! Keeping them shared prevents rule drift between validation and simulation.

use crate::world::block::{BlockKind, Direction};
use crate::world::position::Position;
use crate::world::World3D;

pub fn is_power_source(kind: BlockKind) -> bool {
    matches!(
        kind,
        BlockKind::Torch { .. }
            | BlockKind::Switch { .. }
            | BlockKind::Redstone { .. }
            | BlockKind::RedstoneBlock
            | BlockKind::Repeater { .. }
    )
}

/// The set of positions a redstone dust can propagate to. This must stay
/// byte-for-byte identical to the simulator's connectivity rules.
pub fn redstone_propagate_targets(world: &World3D, pos: Position, state: usize) -> Vec<Position> {
    let mut propagate_targets = Vec::new();

    propagate_targets.extend(pos.cardinal_redstone(state));

    let up_pos = pos.up();
    if world.size.bound_on(up_pos) && !world[up_pos].kind.is_cobble() {
        propagate_targets.extend(
            up_pos
                .cardinal_redstone(state)
                .into_iter()
                .filter(|&pos| world.size.bound_on(pos) && world[pos].kind.is_redstone()),
        );
    }

    if let Some(down_pos) = pos.down() {
        if world[down_pos].kind.is_cobble() {
            propagate_targets.push(down_pos);

            propagate_targets.extend(
                pos.cardinal_redstone(state)
                    .into_iter()
                    .filter(|&pos| world.size.bound_on(pos))
                    .filter(|&pos| !world[pos].kind.is_cobble())
                    .filter_map(|pos| pos.walk(Direction::Bottom))
                    .filter(|&pos| world.size.bound_on(pos) && world[pos].kind.is_redstone()),
            );
        }
    }

    propagate_targets
}

/// Every `(target, hard)` pair a power source can drive, regardless of what
/// block is at the target. This generalizes the simulator's cobble-power-input
/// construction to all target kinds.
pub fn power_targets(world: &World3D, source: Position) -> Vec<(Position, bool)> {
    let source_block = world[source];
    let mut targets = Vec::new();

    match source_block.kind {
        BlockKind::Torch { .. } => {
            let soft_targets = match source_block.direction {
                Direction::Bottom => source.cardinal(),
                Direction::East | Direction::West | Direction::South | Direction::North => {
                    let mut positions = source.cardinal_except(source_block.direction);
                    positions.extend(source.down());
                    positions
                }
                _ => Vec::new(),
            };
            targets.extend(soft_targets.into_iter().map(|target| (target, false)));
            targets.push((source.up(), true));
        }
        BlockKind::Switch { .. } => {
            targets.extend(
                source
                    .forwards_except(source_block.direction)
                    .into_iter()
                    .map(|target| (target, false)),
            );
            if let Some(target) = source.walk(source_block.direction) {
                targets.push((target, true));
            }
        }
        BlockKind::Redstone { state, .. } => {
            targets.extend(
                redstone_propagate_targets(world, source, state)
                    .into_iter()
                    .map(|target| (target, false)),
            );
        }
        BlockKind::RedstoneBlock => {
            targets.extend(source.forwards().into_iter().map(|target| (target, false)));
        }
        BlockKind::Repeater { .. } => {
            if let Some(target) = source.walk(source_block.direction.inverse()) {
                targets.push((target, true));
            }
        }
        _ => {}
    }

    targets
}

/// Possible power sources that drive a cobble, deduplicated by source position
/// with `hard` OR-ed together, exactly as the simulator builds its
/// `cobble_power_inputs` table.
pub fn cobble_power_sources(world: &World3D, target: Position) -> Vec<(Position, bool)> {
    if !world.size.bound_on(target) || !world[target].kind.is_cobble() {
        return Vec::new();
    }

    let mut result = Vec::new();
    for (source, block) in world.iter_block() {
        if !is_power_source(block.kind) {
            continue;
        }
        for (candidate, hard) in power_targets(world, source) {
            if candidate != target {
                continue;
            }
            if let Some(existing) = result.iter_mut().find(|(source_pos, _)| *source_pos == source)
            {
                existing.1 |= hard;
            } else {
                result.push((source, hard));
            }
        }
    }
    result
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::world::block::{Block, BlockKind};
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

    fn world(blocks: Vec<(Position, Block)>) -> World3D {
        World3D::from(&World {
            size: DimSize(4, 4, 3),
            blocks,
        })
    }

    #[test]
    fn switch_drives_adjacent_cobble_soft() {
        let switch = Position(1, 2, 0);
        let target = Position(1, 1, 0);
        let w = world(vec![
            (target, cobble()),
            (
                switch,
                Block {
                    kind: BlockKind::Switch { is_on: true },
                    direction: Direction::Top,
                },
            ),
        ]);

        let sources = cobble_power_sources(&w, target);
        assert_eq!(
            sources,
            vec![(switch, false)],
            "a switch drives an adjacent cobble as a soft source"
        );
    }

    #[test]
    fn torch_drives_block_above_hard() {
        let torch = Position(1, 1, 0);
        let target = Position(1, 1, 1);
        let w = world(vec![
            (target, cobble()),
            (
                torch,
                Block {
                    kind: BlockKind::Torch { is_on: true },
                    direction: Direction::Bottom,
                },
            ),
        ]);

        let sources = cobble_power_sources(&w, target);
        assert_eq!(sources, vec![(torch, true)]);
    }

    #[test]
    fn non_cobble_target_has_no_sources() {
        let w = world(vec![(
            Position(1, 1, 0),
            Block {
                kind: BlockKind::Switch { is_on: true },
                direction: Direction::Top,
            },
        )]);

        assert!(cobble_power_sources(&w, Position(1, 0, 0)).is_empty());
    }
}
