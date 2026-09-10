use itertools::Itertools;
use strum::IntoEnumIterator;

use crate::world::block::{BlockKind, Direction};
use crate::world::position::Position;
use crate::world::World3D;

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub enum PropagateType {
    Soft,
    Hard,
    Torch,
    Repeater,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash)]
pub struct PlaceBound(pub PropagateType, pub Position, pub Direction);

impl PlaceBound {
    pub fn propagation_type(&self) -> PropagateType {
        self.0
    }

    pub fn position(&self) -> Position {
        self.1
    }

    pub fn direction(&self) -> Direction {
        self.2
    }

    pub fn is_bound_on(&self, world: &World3D) -> bool {
        world.size.bound_on(self.position())
    }

    // Signal을 보낼 수 있는 블록들을 탐색
    // 탐색 지점에 어떤 블록이 있는지는 검사하지 않음 (이 검사에는 propagate_to를 사용)
    #[allow(clippy::let_and_return)]
    pub fn propagation_bound(&self, kind: &BlockKind, world: Option<&World3D>) -> Vec<PlaceBound> {
        let dir = self.direction();
        let pos = self.position();

        match kind {
            BlockKind::Switch { .. } => {
                let result = pos
                    .forwards_except(dir)
                    .into_iter()
                    .map(|pos_src| PlaceBound(PropagateType::Torch, pos_src, pos.diff(pos_src)))
                    .chain(|| -> Option<PlaceBound> {
                        let pos = pos.walk(dir)?;

                        Some(PlaceBound(PropagateType::Hard, pos, Direction::None))
                    }())
                    .collect_vec();

                result
            }
            BlockKind::Redstone { state, .. } => {
                let world = world.unwrap();

                let mut propagate_targets = Vec::new();
                propagate_targets.extend(pos.cardinal_redstone(*state));

                if world.size.bound_on(pos.up()) && !world[pos.up()].kind.is_cobble() {
                    propagate_targets.extend(
                        pos.up()
                            .cardinal_redstone(*state)
                            .into_iter()
                            .filter(|&up_cardinal| {
                                world.size.bound_on(up_cardinal)
                                    && (world[up_cardinal].kind.is_air()
                                        || world[up_cardinal].kind.is_redstone())
                            }),
                    );
                }

                if let Some(down_pos) = pos.down() {
                    if !world[down_pos].kind.is_cobble() {
                        // Ensure redstone floors must have a block
                        unreachable!();
                    }

                    propagate_targets.push(down_pos);
                    propagate_targets.extend(
                        pos.cardinal_redstone(*state)
                            .into_iter()
                            .filter(|&pos| world.size.bound_on(pos))
                            .filter(|&pos| !world[pos].kind.is_cobble())
                            .filter_map(|pos| pos.walk(Direction::Bottom))
                            .filter(|&pos| !world[pos].kind.is_cobble()),
                    );
                }

                propagate_targets
                    .iter()
                    .map(|pos_src| PlaceBound(PropagateType::Soft, *pos_src, pos_src.diff(pos)))
                    .collect_vec()
            }
            BlockKind::Torch { .. } => {
                let result = match dir {
                    Direction::Bottom => pos.cardinal(),
                    Direction::East | Direction::West | Direction::South | Direction::North => {
                        let mut positions = pos.cardinal_except(dir);
                        positions.extend(pos.down());
                        positions
                    }
                    _ => unreachable!(),
                }
                .into_iter()
                // CHECK: 현재 propagation_bound는 블록 검사의 책임이 없기 때문에 다음과 같은 조건 체크는 외부의 책임임
                // .filter(|&pos| !world.unwrap()[pos].kind.is_cobble())
                .map(|pos_src| PlaceBound(PropagateType::Torch, pos_src, pos_src.diff(pos)))
                .chain(Some(PlaceBound(
                    PropagateType::Hard,
                    pos.up(),
                    Direction::None,
                )))
                .collect_vec();

                result
            }
            BlockKind::Repeater { .. } => {
                let walk = pos.walk(dir.inverse());
                let mut result: Vec<PlaceBound> = Vec::new();

                if let Some(pos) = walk {
                    result.push(PlaceBound(PropagateType::Repeater, pos, dir));
                }

                result
            }
            BlockKind::RedstoneBlock => pos
                .forwards()
                .into_iter()
                .map(|pos_src| PlaceBound(PropagateType::Soft, pos_src, pos_src.diff(pos)))
                .collect_vec(),
            BlockKind::Piston { .. } => unimplemented!(),
            BlockKind::Air | BlockKind::Cobble { .. } => unreachable!(),
        }
    }

    // Signal을 받은 경우 어디에다 전파할 것인지 탐색
    pub fn propagate_to(&self, world: &World3D) -> Vec<(Direction, Position)> {
        let propagate_type = self.propagation_type();
        let pos = self.position();
        let dir = self.direction();
        let block = &world[pos];

        match block.kind {
            BlockKind::Air
            | BlockKind::Switch { .. }
            | BlockKind::RedstoneBlock
            | BlockKind::Torch { .. } => Vec::new(),
            // 코블에 붙어있는 레드스톤 토치, 리피터만 반응함
            BlockKind::Cobble { .. } => {
                if matches!(propagate_type, PropagateType::Torch) {
                    return Vec::new();
                }

                let mut cardinal_propagation = pos
                    .cardinal_except(dir)
                    .iter()
                    .filter(|&&pos| world.size.bound_on(pos))
                    .filter_map(|&pos_src| match world[pos_src].kind {
                        BlockKind::Torch { .. } => match propagate_type {
                            PropagateType::Torch => None,
                            PropagateType::Soft | PropagateType::Hard | PropagateType::Repeater => {
                                if pos_src.walk(world[pos_src].direction) == Some(pos) {
                                    Some((Direction::None, pos_src))
                                } else {
                                    None
                                }
                            }
                        },
                        BlockKind::Repeater { .. } => match propagate_type {
                            PropagateType::Torch => None,
                            PropagateType::Soft | PropagateType::Hard | PropagateType::Repeater => {
                                if pos_src.walk(world[pos_src].direction) == Some(pos) {
                                    Some((Direction::None, pos_src))
                                } else {
                                    None
                                }
                            }
                        },
                        BlockKind::Redstone { .. } => match propagate_type {
                            PropagateType::Soft | PropagateType::Torch => None,
                            PropagateType::Hard | PropagateType::Repeater => {
                                Some((Direction::None, pos_src))
                            }
                        },
                        _ => None,
                    })
                    .collect_vec();

                let up_pos = pos.up();
                if world.size.bound_on(up_pos) {
                    let up_block = &world[up_pos];
                    if (up_block.direction == Direction::Bottom
                        && matches!(up_block.kind, BlockKind::Torch { .. }))
                        || (!matches!(propagate_type, PropagateType::Soft)
                            && matches!(up_block.kind, BlockKind::Redstone { .. }))
                    {
                        cardinal_propagation.push((Direction::None, up_pos));
                    }
                }
                if let Some(down_pos) = pos.down() {
                    if !matches!(propagate_type, PropagateType::Soft)
                        && matches!(world[down_pos].kind, BlockKind::Redstone { .. })
                    {
                        cardinal_propagation.push((Direction::None, down_pos));
                    }
                }

                cardinal_propagation
            }
            BlockKind::Redstone { .. } => vec![(Direction::None, pos)],
            BlockKind::Repeater { .. } => {
                if block.direction == dir {
                    vec![(Direction::None, pos)]
                } else if block.direction.is_othogonal_plane(dir) {
                    // lock
                    match propagate_type {
                        PropagateType::Repeater => vec![(block.direction, pos)],
                        _ => vec![],
                    }
                } else {
                    vec![]
                }
            }
            BlockKind::Piston { .. } => todo!(),
        }
    }

    // Signal을 받을 수 있는 블록들을 탐색
    pub fn propagated_from(pos_src: Position, kind: &BlockKind, world: &World3D) -> Vec<Self> {
        let mut result = Vec::new();
        for dir in Direction::iter().filter(|dir| !dir.is_none()) {
            let Some(pos) = pos_src.walk(dir) else {
                continue;
            };

            if !world.size.bound_on(pos) {
                continue;
            }

            match kind {
                BlockKind::Redstone { .. } => {
                    // Stick to Redstone
                    if (dir.is_cardinal() || dir.is_top()) && world[pos].kind.is_stick_to_redstone()
                    {
                        result.push(Self(PropagateType::Soft, pos, dir.inverse()));
                    }

                    // Cobble
                    if world[pos].kind.is_cobble() {
                        let hard_propagation = Self::propagated_from(pos, &world[pos].kind, world)
                            .into_iter()
                            .filter(|bound| {
                                bound.position() != pos
                                    && matches!(bound.propagation_type(), PropagateType::Hard)
                            });

                        result.extend(hard_propagation.map(|bound| {
                            Self(PropagateType::Soft, bound.position(), bound.direction())
                        }));
                    }
                }
                BlockKind::Cobble { .. } => match world[pos].kind {
                    BlockKind::Repeater { .. } => {
                        if dir.is_cardinal() && world[pos].direction == dir.inverse() {
                            result.push(Self(PropagateType::Hard, pos, dir.inverse()));
                        }
                    }
                    BlockKind::Switch { .. } => {
                        if dir.is_cardinal() && world[pos].direction == dir.inverse() {
                            result.push(Self(PropagateType::Hard, pos, dir.inverse()));
                        }
                    }
                    BlockKind::Redstone { state, .. } => {
                        if (dir.is_cardinal() || dir.is_top())
                            && pos
                                .cardinal_redstone(state)
                                .into_iter()
                                .any(|pos_redstone| pos_redstone == pos)
                        {
                            result.push(Self(PropagateType::Soft, pos, dir.inverse()));
                        }
                    }
                    BlockKind::Torch { .. } => {
                        if dir.is_bottom() {
                            result.push(Self(PropagateType::Hard, pos, dir.inverse()));
                        }
                    }
                    _ => (),
                },
                BlockKind::Torch { .. } => {
                    let support = pos_src.walk(world[pos_src].direction);
                    if support != Some(pos) || !world[pos].kind.is_cobble() {
                        continue;
                    }

                    result.extend(
                        Self::propagated_from(pos, &world[pos].kind, world)
                            .into_iter()
                            .filter(|bound| {
                                matches!(bound.propagation_type(), PropagateType::Hard)
                            }),
                    );
                }
                BlockKind::Repeater { .. } => {
                    if pos_src.walk(world[pos_src].direction) != Some(pos) {
                        continue;
                    }

                    let source_type = match world[pos].kind {
                        BlockKind::Redstone { .. } | BlockKind::RedstoneBlock => {
                            PropagateType::Soft
                        }
                        BlockKind::Torch { .. } => PropagateType::Torch,
                        BlockKind::Repeater { .. } => PropagateType::Repeater,
                        BlockKind::Switch { .. } => PropagateType::Hard,
                        _ => continue,
                    };

                    result.push(Self(source_type, pos, dir.inverse()));
                }
                BlockKind::Piston { .. } => todo!(),
                BlockKind::RedstoneBlock | BlockKind::Switch { .. } => (),
                BlockKind::Air => (),
            }
        }

        result
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::world::block::Block;
    use crate::world::position::DimSize;

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
            kind: BlockKind::Torch { is_on: true },
            direction,
        }
    }

    fn switch(direction: Direction) -> Block {
        Block {
            kind: BlockKind::Switch { is_on: true },
            direction,
        }
    }

    fn repeater(direction: Direction) -> Block {
        Block {
            kind: BlockKind::Repeater {
                is_on: false,
                is_locked: false,
                delay: 1,
                lock_input1: None,
                lock_input2: None,
            },
            direction,
        }
    }

    fn redstone() -> Block {
        Block {
            kind: BlockKind::Redstone {
                on_count: 0,
                state: 0,
                strength: 0,
            },
            direction: Direction::None,
        }
    }

    #[test]
    fn propagated_from_repeater_follows_the_input_side() {
        let mut world = World3D::new(DimSize(5, 3, 3));
        let repeater_position = Position(2, 1, 1);
        let input_position = Position(3, 1, 1);
        let output_position = Position(1, 1, 1);
        world[repeater_position] = repeater(Direction::East);
        world[input_position] = torch(Direction::Bottom);
        world[output_position] = torch(Direction::Bottom);

        let bounds =
            PlaceBound::propagated_from(repeater_position, &world[repeater_position].kind, &world);

        assert!(bounds.contains(&PlaceBound(
            PropagateType::Torch,
            input_position,
            Direction::West
        )));
        assert!(!bounds.contains(&PlaceBound(
            PropagateType::Torch,
            output_position,
            Direction::East
        )));
    }

    #[test]
    fn propagated_from_repeater_classifies_redstone_input() {
        let mut world = World3D::new(DimSize(5, 3, 3));
        let repeater_position = Position(2, 1, 1);
        let input_position = Position(3, 1, 1);
        world[repeater_position] = repeater(Direction::East);
        world[input_position] = redstone();

        let bounds =
            PlaceBound::propagated_from(repeater_position, &world[repeater_position].kind, &world);

        assert_eq!(
            bounds,
            vec![PlaceBound(
                PropagateType::Soft,
                input_position,
                Direction::West
            )]
        );
    }

    #[test]
    fn propagated_from_torch_reaches_support_hard_power() {
        let mut world = World3D::new(DimSize(5, 3, 3));
        let torch_position = Position(2, 1, 1);
        let support_position = Position(2, 1, 0);
        let switch_position = Position(1, 1, 0);
        world[torch_position] = torch(Direction::Bottom);
        world[support_position] = cobble();
        world[switch_position] = switch(Direction::East);

        let bounds =
            PlaceBound::propagated_from(torch_position, &world[torch_position].kind, &world);

        assert!(bounds.contains(&PlaceBound(
            PropagateType::Hard,
            switch_position,
            Direction::East
        )));
    }

    #[test]
    fn propagated_from_primary_sources_has_no_inputs() {
        let mut world = World3D::new(DimSize(4, 3, 3));
        let redstone_block_position = Position(1, 1, 1);
        let switch_position = Position(2, 1, 1);
        world[redstone_block_position] = Block {
            kind: BlockKind::RedstoneBlock,
            direction: Direction::None,
        };
        world[switch_position] = switch(Direction::Bottom);

        assert!(PlaceBound::propagated_from(
            redstone_block_position,
            &world[redstone_block_position].kind,
            &world
        )
        .is_empty());
        assert!(
            PlaceBound::propagated_from(switch_position, &world[switch_position].kind, &world)
                .is_empty()
        );
    }
}
