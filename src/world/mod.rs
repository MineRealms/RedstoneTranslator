use std::cmp;
use std::collections::BTreeMap;
use std::fmt::Debug;
use std::ops::{Index, IndexMut};
use std::sync::Arc;

use block::{Block, BlockKind, Direction, RedstoneState};
use itertools::Itertools;
use position::{DimSize, Position, PositionIndex};

pub mod block;
pub mod electrical;
pub mod gate;
pub mod position;
pub mod simulator;

#[derive(Debug, Clone)]
pub struct World {
    pub size: DimSize,
    pub blocks: Vec<(Position, Block)>,
}

impl World {
    pub fn new(size: DimSize) -> Self {
        Self {
            size,
            blocks: Default::default(),
        }
    }
}

#[derive(Default)]
pub struct World3D {
    pub size: DimSize,
    /// z -> flat layer (`y * size.0 + x`). Layers are shared copy-on-write, so
    /// a clone is cheap and only written layers are duplicated.
    pub map: Vec<Arc<Vec<Block>>>,
}

impl Clone for World3D {
    fn clone(&self) -> Self {
        crate::perf::record_world_clone(self.size);
        Self {
            size: self.size,
            map: self.map.clone(),
        }
    }
}

impl World3D {
    pub fn new(size: DimSize) -> Self {
        crate::perf::record_world_alloc(size);
        let layer_len = size.0.saturating_mul(size.1);
        Self {
            size,
            map: (0..size.2)
                .map(|_| Arc::new(vec![Block::default(); layer_len]))
                .collect(),
        }
    }

    pub fn iter_pos(&self) -> Vec<Position> {
        let mut result = Vec::new();

        for z in 0..self.size.2 {
            for y in 0..self.size.1 {
                for x in 0..self.size.0 {
                    result.push(Position(x, y, z));
                }
            }
        }

        result
    }

    pub fn iter_block(&self) -> Vec<(Position, Block)> {
        self.iter_pos()
            .into_iter()
            .filter(|&pos| !self[pos].kind.is_air())
            .map(|pos| (pos, self[pos]))
            .collect_vec()
    }

    pub fn initialize_redstone_states(&mut self) {
        self.iter_pos()
            .iter()
            .for_each(|pos| self.update_redstone_states(*pos));
    }

    pub fn update_redstone_states(&mut self, pos: Position) {
        let BlockKind::Redstone {
            on_count, strength, ..
        } = self[pos].kind
        else {
            return;
        };

        let mut state = 0;

        let has_up_block = self.size.bound_on(pos.up()) && self[pos.up()].kind.is_cobble();

        pos.cardinal().iter().for_each(|&pos_src| {
            if !self.size.bound_on(pos_src) {
                return;
            }

            let flat_check = self[pos_src].kind.is_stick_to_redstone();
            let up_check = !has_up_block
                && self.size.bound_on(pos.up())
                && self[pos_src.up()].kind.is_redstone();
            let down_check = !self[pos_src].kind.is_cobble()
                && pos_src
                    .down()
                    .is_some_and(|pos| self[pos].kind.is_redstone());
            let flat_repeater_check = self[pos_src].kind.is_repeater()
                && (pos_src.walk(self[pos_src].direction) == Some(pos));

            if !(flat_check || flat_repeater_check || up_check || down_check) {
                return;
            }

            state |= match pos.diff(pos_src) {
                Direction::East => RedstoneState::East,
                Direction::West => RedstoneState::West,
                Direction::South => RedstoneState::South,
                Direction::North => RedstoneState::North,
                _ => unreachable!(),
            } as usize;
        });

        if state.count_ones() == 1 {
            if state & RedstoneState::Horizontal as usize > 0 {
                state |= RedstoneState::Horizontal as usize;
            } else {
                state |= RedstoneState::Vertical as usize;
            }
        } else if state == 0 {
            state |= RedstoneState::Cardinal as usize;
        }

        self[pos].kind = BlockKind::Redstone {
            on_count,
            state,
            strength,
        };
    }

    pub fn concat(&self, other: &World3D, direction: Direction) -> Self {
        match direction {
            Direction::None => unreachable!(),
            Direction::Bottom | Direction::West | Direction::South => {
                other.concat(self, direction.inverse())
            }
            Direction::Top | Direction::North | Direction::East => {
                let mut world = Self::new(DimSize(
                    if matches!(direction, Direction::East) {
                        self.size.0 + other.size.0
                    } else {
                        cmp::max(self.size.0, other.size.0)
                    },
                    if matches!(direction, Direction::North) {
                        self.size.1 + other.size.1
                    } else {
                        cmp::max(self.size.1, other.size.1)
                    },
                    if matches!(direction, Direction::Top) {
                        self.size.2 + other.size.2
                    } else {
                        cmp::max(self.size.2, other.size.2)
                    },
                ));

                for (pos, block) in self.iter_block() {
                    world[pos] = block;
                }

                for (mut pos, block) in other.iter_block() {
                    match direction {
                        Direction::East => pos.0 += self.size.0,
                        Direction::North => pos.1 += self.size.1,
                        Direction::Top => pos.2 += self.size.2,
                        _ => (),
                    }
                    world[pos] = block;
                }

                world
            }
        }
    }

    pub fn concat_tiled(worlds: Vec<World3D>) -> Self {
        let east_chunk_len = (worlds.len() as f32).sqrt() as usize + 1;

        let mut east_worlds = worlds
            .into_iter()
            .chunks(east_chunk_len)
            .into_iter()
            .map(|chunk| chunk.collect_vec())
            .map(|mut worlds| {
                let mut world = worlds.remove(0);
                for other in worlds {
                    world = world.concat(&other, Direction::East);
                }
                world
            })
            .collect_vec();

        let mut world = east_worlds.remove(0);
        for other in east_worlds {
            world = world.concat(&other, Direction::North);
        }
        world
    }
}

impl<'a> From<&'a World> for World3D {
    fn from(value: &'a World) -> Self {
        crate::perf::record_world_alloc(value.size);
        let mut block_map: BTreeMap<PositionIndex, &Block> = BTreeMap::default();

        for block in &value.blocks {
            block_map.insert(block.0.index(&value.size), &block.1);
        }

        let width = value.size.0;
        let height = value.size.1;
        let mut map = Vec::with_capacity(value.size.2);
        for z in 0..value.size.2 {
            let mut layer = vec![Block::default(); width.saturating_mul(height)];
            for y in 0..height {
                for x in 0..width {
                    let pos = PositionIndex(x + y * width + z * width * height);
                    if let Some(&&block) = block_map.get(&pos) {
                        layer[y * width + x] = block;
                    }
                }
            }
            map.push(Arc::new(layer));
        }

        Self {
            size: value.size,
            map,
        }
    }
}

impl<'a> From<&'a World3D> for World {
    fn from(value: &'a World3D) -> Self {
        Self {
            size: value.size,
            blocks: value.iter_block(),
        }
    }
}

impl Index<Position> for World3D {
    type Output = Block;

    fn index(&self, index: Position) -> &Self::Output {
        &self.map[index.2][index.1 * self.size.0 + index.0]
    }
}

impl IndexMut<Position> for World3D {
    fn index_mut(&mut self, index: Position) -> &mut Self::Output {
        let layer = &mut self.map[index.2];
        if Arc::get_mut(layer).is_none() {
            crate::perf::record_layer_copy(self.size.0.saturating_mul(self.size.1));
        }
        let layer = Arc::make_mut(layer);
        &mut layer[index.1 * self.size.0 + index.0]
    }
}

impl Debug for World3D {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let width = self.size.0;
        for (height, layer) in self.map.iter().enumerate().rev() {
            writeln!(f, "h={height:?}")?;

            for y in (0..self.size.1).rev() {
                let row = &layer[y * width..(y + 1) * width];
                writeln!(
                    f,
                    "  {}",
                    row.iter()
                        .map(|block| match block.kind {
                            BlockKind::Air => ".",
                            BlockKind::Cobble { .. } => "c",
                            BlockKind::Switch { .. } => "s",
                            BlockKind::Redstone { .. } => "r",
                            BlockKind::Torch { .. } => "t",
                            BlockKind::Repeater { .. } => "t",
                            BlockKind::RedstoneBlock => "b",
                            BlockKind::Piston { .. } => "p",
                        })
                        .collect::<Vec<_>>()
                        .join("")
                )?;
            }
        }

        Ok(())
    }
}
