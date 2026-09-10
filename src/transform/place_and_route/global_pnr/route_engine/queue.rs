//! Search queue strategies for the point-to-point routing engine.

use std::cmp::Ordering;
use std::collections::{BinaryHeap, VecDeque};

use super::cost::RouteCostModel;
use super::state::RouteSearchState;
use crate::transform::place_and_route::global_pnr::router::GlobalRoutingStrategy;
use crate::world::position::Position;

const GLOBAL_ROUTE_ASTAR_MAX_EXPANSIONS: usize = 2_000;

pub(crate) fn route_expansion_limit(strategy: GlobalRoutingStrategy) -> Option<usize> {
    match strategy {
        GlobalRoutingStrategy::BreadthFirst => None,
        GlobalRoutingStrategy::AStar => Some(GLOBAL_ROUTE_ASTAR_MAX_EXPANSIONS),
        GlobalRoutingStrategy::DirectGreedy { max_steps } => Some(max_steps),
        GlobalRoutingStrategy::GreedyBeam { max_expansions, .. } => Some(max_expansions),
    }
}

pub(crate) enum RouteSearchQueue {
    BreadthFirst(VecDeque<RouteSearchState>),
    AStar {
        heap: BinaryHeap<AStarQueueEntry>,
        next_sequence: usize,
        sink: Position,
        cost_model: RouteCostModel,
    },
    DirectGreedy {
        entry: Option<AStarQueueEntry>,
        next_sequence: usize,
        sink: Position,
        cost_model: RouteCostModel,
    },
    GreedyBeam {
        entries: Vec<AStarQueueEntry>,
        beam_width: usize,
        next_sequence: usize,
        sink: Position,
        variant_seed: u64,
        cost_model: RouteCostModel,
    },
}

impl RouteSearchQueue {
    pub(crate) fn new(
        strategy: GlobalRoutingStrategy,
        sink: Position,
        initial_states: Vec<RouteSearchState>,
        cost_model: RouteCostModel,
    ) -> Self {
        match strategy {
            GlobalRoutingStrategy::BreadthFirst => Self::BreadthFirst(initial_states.into()),
            GlobalRoutingStrategy::AStar => {
                let mut queue = Self::AStar {
                    heap: BinaryHeap::new(),
                    next_sequence: 0,
                    sink,
                    cost_model,
                };
                for state in initial_states {
                    queue.push(state);
                }
                queue
            }
            GlobalRoutingStrategy::DirectGreedy { .. } => {
                let mut queue = Self::DirectGreedy {
                    entry: None,
                    next_sequence: 0,
                    sink,
                    cost_model,
                };
                for state in initial_states {
                    queue.push(state);
                }
                queue
            }
            GlobalRoutingStrategy::GreedyBeam {
                beam_width,
                variant_seed,
                ..
            } => {
                let mut queue = Self::GreedyBeam {
                    entries: Vec::new(),
                    beam_width: beam_width.max(1),
                    next_sequence: 0,
                    sink,
                    variant_seed,
                    cost_model,
                };
                for state in initial_states {
                    queue.push(state);
                }
                queue
            }
        }
    }

    pub(crate) fn pop(&mut self) -> Option<RouteSearchState> {
        match self {
            Self::BreadthFirst(queue) => queue.pop_front(),
            Self::AStar { heap, .. } => heap.pop().map(|entry| entry.state),
            Self::DirectGreedy { entry, .. } => entry.take().map(|entry| entry.state),
            Self::GreedyBeam { entries, .. } => {
                let best = entries
                    .iter()
                    .enumerate()
                    .min_by_key(|(_, entry)| entry.priority)
                    .map(|(index, _)| index)?;
                Some(entries.swap_remove(best).state)
            }
        }
    }

    pub(crate) fn push(&mut self, state: RouteSearchState) {
        match self {
            Self::BreadthFirst(queue) => queue.push_back(state),
            Self::AStar {
                heap,
                next_sequence,
                sink,
                cost_model,
            } => {
                heap.push(AStarQueueEntry::new(
                    state,
                    *sink,
                    *next_sequence,
                    *cost_model,
                ));
                *next_sequence += 1;
            }
            Self::DirectGreedy {
                entry,
                next_sequence,
                sink,
                cost_model,
            } => {
                let candidate = AStarQueueEntry::new(state, *sink, *next_sequence, *cost_model);
                *next_sequence += 1;
                if entry
                    .as_ref()
                    .is_none_or(|current| candidate.priority < current.priority)
                {
                    *entry = Some(candidate);
                }
            }
            Self::GreedyBeam {
                entries,
                beam_width,
                next_sequence,
                sink,
                variant_seed,
                cost_model,
            } => {
                entries.push(AStarQueueEntry::new_with_variant(
                    state,
                    *sink,
                    *next_sequence,
                    *variant_seed,
                    *cost_model,
                ));
                *next_sequence += 1;
                if entries.len() > *beam_width {
                    let worst = entries
                        .iter()
                        .enumerate()
                        .max_by_key(|(_, entry)| entry.priority)
                        .map(|(index, _)| index)
                        .expect("greedy beam contains the pushed entry");
                    entries.swap_remove(worst);
                }
            }
        }
    }
}
pub(crate) struct AStarQueueEntry {
    priority: AStarPriority,
    state: RouteSearchState,
}

impl AStarQueueEntry {
    fn new(
        state: RouteSearchState,
        sink: Position,
        sequence: usize,
        cost_model: RouteCostModel,
    ) -> Self {
        Self::new_with_variant(state, sink, sequence, 0, cost_model)
    }

    fn new_with_variant(
        state: RouteSearchState,
        sink: Position,
        sequence: usize,
        variant_seed: u64,
        cost_model: RouteCostModel,
    ) -> Self {
        Self {
            priority: AStarPriority::new(&state, sink, sequence, variant_seed, cost_model),
            state,
        }
    }
}

impl Ord for AStarQueueEntry {
    fn cmp(&self, other: &Self) -> Ordering {
        other.priority.cmp(&self.priority)
    }
}

impl PartialOrd for AStarQueueEntry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl PartialEq for AStarQueueEntry {
    fn eq(&self, other: &Self) -> bool {
        self.priority == other.priority
    }
}

impl Eq for AStarQueueEntry {}
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
struct AStarPriority {
    estimated_total_cost: usize,
    route_len: usize,
    manhattan_to_sink: usize,
    low_strength_penalty: usize,
    variant_tie_break: u64,
    sequence: usize,
}

impl AStarPriority {
    fn new(
        state: &RouteSearchState,
        sink: Position,
        sequence: usize,
        variant_seed: u64,
        cost_model: RouteCostModel,
    ) -> Self {
        let route_len = state.route.len().saturating_sub(1);
        let manhattan_to_sink = state.terminal.manhattan_distance(&sink);
        let low_strength_penalty = cost_model.low_strength_penalty(state);
        let variant_tie_break = if variant_seed == 0 {
            0
        } else {
            let Position(x, y, z) = state.terminal;
            variant_seed
                .wrapping_mul(0x9E37_79B9_7F4A_7C15)
                .wrapping_add((x as u64).wrapping_mul(0xBF58_476D_1CE4_E5B9))
                .wrapping_add((y as u64).wrapping_mul(0x94D0_49BB_1331_11EB))
                .wrapping_add(z as u64)
                .rotate_left((state.route.len() % 64) as u32)
        };
        Self {
            estimated_total_cost: cost_model.weighted_route_cost(state)
                + state.extra_cost
                + manhattan_to_sink
                + low_strength_penalty,
            route_len,
            manhattan_to_sink,
            low_strength_penalty,
            variant_tie_break,
            sequence,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::transform::place_and_route::global_pnr::route_engine::state::PoweredRouteSource;
    use crate::world::block::{Block, BlockKind, Direction};
    use crate::world::position::DimSize;
    use crate::world::World3D;

    fn state(route: Vec<Position>) -> RouteSearchState {
        let mut world = World3D::new(DimSize(4, 4, 4));
        for position in &route {
            world[*position] = Block {
                kind: BlockKind::Redstone {
                    on_count: 0,
                    state: 0,
                    strength: 0,
                },
                direction: Direction::None,
            };
        }
        RouteSearchState {
            world,
            terminal: *route.last().unwrap(),
            route,
            signal_strength: 15,
            powered_taps: vec![PoweredRouteSource {
                position: Position(0, 0, 0),
                strength: 15,
            }],
            pending_bounds: None,
            extra_cost: 0,
        }
    }

    #[test]
    fn default_cost_model_preserves_the_legacy_priority() {
        let state = state(vec![Position(0, 0, 0), Position(1, 0, 0)]);
        let priority =
            AStarPriority::new(&state, Position(2, 0, 0), 3, 0, RouteCostModel::default());

        assert_eq!(priority.estimated_total_cost, 2);
        assert_eq!(priority.route_len, 1);
        assert_eq!(priority.low_strength_penalty, 0);
    }

    #[test]
    fn turn_cost_reorders_zigzag_behind_straight_route() {
        let straight = state(vec![
            Position(0, 0, 0),
            Position(1, 0, 0),
            Position(2, 0, 0),
            Position(3, 0, 0),
        ]);
        let zigzag = state(vec![
            Position(0, 0, 0),
            Position(1, 0, 0),
            Position(1, 1, 0),
            Position(2, 1, 0),
        ]);
        let sink = Position(3, 0, 0);
        let model = RouteCostModel {
            turn_cost: 3,
            ..Default::default()
        };

        let straight_priority = AStarPriority::new(&straight, sink, 0, 0, model);
        let zigzag_priority = AStarPriority::new(&zigzag, sink, 0, 0, model);

        assert!(straight_priority.estimated_total_cost < zigzag_priority.estimated_total_cost);
    }
}
