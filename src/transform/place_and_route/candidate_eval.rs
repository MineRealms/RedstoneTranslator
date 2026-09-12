//! Candidate IR and the evaluator boundary between enumeration and the exact
//! physical engine (placement, routing, PECA).
//!
//! Enumeration produces `PlacementCandidate` records without mutating the
//! world; the evaluator returns a conservative verdict plus integer scores.
//! The CPU implementation is the reference; the optional `gpu` feature adds a
//! wgpu backend that implements the same trait (`docs/gpu_acceleration_plan.md`).

use crate::world::block::{BlockKind, Direction};
use crate::world::position::Position;
use crate::world::World3D;

/// Ranking penalty for each foreign power source next to the support. This is
/// a heuristic used by the evaluator; it never changes `valid`.
pub const FOREIGN_DRIVER_PENALTY: u32 = 100;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CandidateKind {
    Torch,
}

/// A physical placement intent. This is compile-time only: it never enters
/// `Block`, NBT, or the simulator.
#[derive(Clone, Copy, Debug)]
pub struct PlacementCandidate {
    /// Index of the frontier entry that produced the candidate.
    pub entry: u32,
    pub kind: CandidateKind,
    pub torch: Position,
    pub direction: Direction,
    pub support: Position,
    pub source: Position,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CandidateScore {
    pub valid: bool,
    pub drc_penalty: u32,
    pub estimated_route_cost: u32,
}

pub trait CandidateEvaluator {
    fn evaluate(&self, world: &World3D, candidates: &[PlacementCandidate])
        -> Vec<CandidateScore>;
}

pub struct CpuCandidateEvaluator;

impl CandidateEvaluator for CpuCandidateEvaluator {
    fn evaluate(
        &self,
        world: &World3D,
        candidates: &[PlacementCandidate],
    ) -> Vec<CandidateScore> {
        candidates
            .iter()
            .map(|candidate| evaluate_one(world, candidate))
            .collect()
    }
}

/// Conservative cell-level checks only. Every rejection here is a necessary
/// condition of `place_torch_with_cobble`, so dropping the candidate does not
/// change which placements survive.
fn evaluate_one(world: &World3D, candidate: &PlacementCandidate) -> CandidateScore {
    let torch_ok = world.size.bound_on(candidate.torch) && world[candidate.torch].kind.is_air();
    let support_ok = world.size.bound_on(candidate.support)
        && (world[candidate.support].kind.is_air() || world[candidate.support].kind.is_cobble());

    if !torch_ok || !support_ok {
        return CandidateScore {
            valid: false,
            drc_penalty: 10_000,
            estimated_route_cost: 0,
        };
    }

    CandidateScore {
        valid: true,
        drc_penalty: foreign_driver_penalty(world, candidate),
        estimated_route_cost: candidate.source.manhattan_distance(&candidate.support) as u32,
    }
}

/// Counts power sources next to the support (except the expected source).
/// Mirrors the GPU kernel exactly for the differential test.
fn foreign_driver_penalty(world: &World3D, candidate: &PlacementCandidate) -> u32 {
    let mut penalty = 0;
    for neighbor in candidate.support.forwards() {
        if !world.size.bound_on(neighbor) || neighbor == candidate.source {
            continue;
        }
        let kind = world[neighbor].kind;
        if kind.is_torch()
            || kind.is_switch()
            || kind.is_repeater()
            || matches!(kind, BlockKind::RedstoneBlock)
        {
            penalty += FOREIGN_DRIVER_PENALTY;
        }
    }
    penalty
}

/// Evaluate a batch with the configured backend (GPU when the `gpu` feature is
/// enabled and `MCHDL_GPU=1`, CPU otherwise).
pub fn evaluate(world: &World3D, candidates: &[PlacementCandidate]) -> Vec<CandidateScore> {
    #[cfg(feature = "gpu")]
    if crate::gpu::enabled() {
        return crate::gpu::evaluate(world, candidates);
    }

    CpuCandidateEvaluator.evaluate(world, candidates)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::world::block::{Block, BlockKind};
    use crate::world::position::DimSize;
    use crate::world::World;

    fn world(blocks: Vec<(Position, Block)>) -> World3D {
        World3D::from(&World {
            size: DimSize(8, 8, 4),
            blocks,
        })
    }

    fn cobble() -> Block {
        Block {
            kind: BlockKind::Cobble {
                on_count: 0,
                on_base_count: 0,
            },
            direction: Direction::None,
        }
    }

    fn torch_block(direction: Direction) -> Block {
        Block {
            kind: BlockKind::Torch { is_on: false },
            direction,
        }
    }

    fn candidate(torch: Position, direction: Direction, support: Position) -> PlacementCandidate {
        PlacementCandidate {
            entry: 0,
            kind: CandidateKind::Torch,
            torch,
            direction,
            support,
            source: Position(0, 0, 0),
        }
    }

    #[test]
    fn free_cells_are_valid_with_route_cost() {
        let w = world(vec![]);
        let scores = evaluate(&w, &[candidate(Position(1, 1, 1), Direction::East, Position(2, 1, 1))]);
        assert_eq!(scores.len(), 1);
        assert!(scores[0].valid);
        assert_eq!(scores[0].drc_penalty, 0);
        assert_eq!(scores[0].estimated_route_cost, 4);
    }

    #[test]
    fn occupied_torch_cell_is_invalid() {
        let w = world(vec![(Position(1, 1, 1), cobble())]);
        let scores = evaluate(&w, &[candidate(Position(1, 1, 1), Direction::East, Position(2, 1, 1))]);
        assert!(!scores[0].valid);
    }

    #[test]
    fn existing_cobble_support_is_allowed() {
        let w = world(vec![(Position(2, 1, 1), cobble())]);
        let scores = evaluate(&w, &[candidate(Position(1, 1, 1), Direction::East, Position(2, 1, 1))]);
        assert!(scores[0].valid);
    }

    #[test]
    fn out_of_bounds_is_invalid() {
        let w = world(vec![]);
        let scores = evaluate(&w, &[candidate(Position(1, 1, 1), Direction::East, Position(9, 1, 1))]);
        assert!(!scores[0].valid);
    }

    #[test]
    fn foreign_driver_neighbour_adds_penalty() {
        let w = world(vec![(Position(3, 1, 1), torch_block(Direction::West))]);
        let scores = evaluate(&w, &[candidate(Position(1, 1, 1), Direction::East, Position(2, 1, 1))]);
        assert!(scores[0].valid);
        assert_eq!(scores[0].drc_penalty, FOREIGN_DRIVER_PENALTY);
    }
}
