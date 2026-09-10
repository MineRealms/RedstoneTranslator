//! Search queue strategies for the point-to-point routing engine.

use std::cmp::Ordering;
use std::collections::{BinaryHeap, VecDeque};

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
    },
    DirectGreedy {
        entry: Option<AStarQueueEntry>,
        next_sequence: usize,
        sink: Position,
    },
    GreedyBeam {
        entries: Vec<AStarQueueEntry>,
        beam_width: usize,
        next_sequence: usize,
        sink: Position,
        variant_seed: u64,
    },
}

impl RouteSearchQueue {
    pub(crate) fn new(
        strategy: GlobalRoutingStrategy,
        sink: Position,
        initial_states: Vec<RouteSearchState>,
    ) -> Self {
        match strategy {
            GlobalRoutingStrategy::BreadthFirst => Self::BreadthFirst(initial_states.into()),
            GlobalRoutingStrategy::AStar => {
                let mut queue = Self::AStar {
                    heap: BinaryHeap::new(),
                    next_sequence: 0,
                    sink,
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
            } => {
                heap.push(AStarQueueEntry::new(state, *sink, *next_sequence));
                *next_sequence += 1;
            }
            Self::DirectGreedy {
                entry,
                next_sequence,
                sink,
            } => {
                let candidate = AStarQueueEntry::new(state, *sink, *next_sequence);
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
            } => {
                entries.push(AStarQueueEntry::new_with_variant(
                    state,
                    *sink,
                    *next_sequence,
                    *variant_seed,
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
    fn new(state: RouteSearchState, sink: Position, sequence: usize) -> Self {
        Self::new_with_variant(state, sink, sequence, 0)
    }

    fn new_with_variant(
        state: RouteSearchState,
        sink: Position,
        sequence: usize,
        variant_seed: u64,
    ) -> Self {
        Self {
            priority: AStarPriority::new(&state, sink, sequence, variant_seed),
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
    fn new(state: &RouteSearchState, sink: Position, sequence: usize, variant_seed: u64) -> Self {
        let route_len = state.route.len().saturating_sub(1);
        let manhattan_to_sink = state.terminal.manhattan_distance(&sink);
        let low_strength_penalty = usize::from(state.signal_strength <= 2) * 4;
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
            estimated_total_cost: route_len + manhattan_to_sink + low_strength_penalty,
            route_len,
            manhattan_to_sink,
            low_strength_penalty,
            variant_tie_break,
            sequence,
        }
    }
}
