//! Batch move-evaluation boundary for simulated annealing (G2).
//!
//! SA proposes single moves and evaluates their cost delta one at a time. This
//! module defines the batch boundary (`MoveEvaluator`) so many proposed moves
//! can be scored together, first by the CPU reference and later by the wgpu
//! backend behind the `gpu` feature (`docs/gpu_acceleration_plan.md`).

use crate::transform::place_and_route::placement_ir::{MacroInstance, PlacementProblem};
use crate::transform::place_and_route::sa_placer::{
    placement_cost, PlacementCost, PlacementCostModel,
};
use crate::world::position::{DimSize, Position};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PlacementMove {
    pub instance: usize,
    pub position: Position,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MoveDelta {
    pub wire_length: i64,
    pub bounding_box_volume: i64,
    pub blocked_pins: i64,
    pub overlapping_pairs: i64,
    pub spacing_violations: i64,
    pub pin_access_violations: i64,
    pub total: f64,
}

impl MoveDelta {
    fn between(before: &PlacementCost, after: &PlacementCost) -> Self {
        Self {
            wire_length: after.wire_length as i64 - before.wire_length as i64,
            bounding_box_volume: after.bounding_box_volume as i64
                - before.bounding_box_volume as i64,
            blocked_pins: after.blocked_pins as i64 - before.blocked_pins as i64,
            overlapping_pairs: after.overlapping_pairs as i64 - before.overlapping_pairs as i64,
            spacing_violations: after.spacing_violations as i64 - before.spacing_violations as i64,
            pin_access_violations: after.pin_access_violations as i64
                - before.pin_access_violations as i64,
            total: after.total - before.total,
        }
    }
}

pub trait MoveEvaluator {
    fn evaluate(
        &self,
        problem: &PlacementProblem,
        instances: &[MacroInstance],
        model: &PlacementCostModel,
        world: DimSize,
        moves: &[PlacementMove],
    ) -> eyre::Result<Vec<MoveDelta>>;
}

/// Reference implementation: recomputes the full placement cost per move.
/// The GPU backend must reproduce these deltas exactly (differential test).
pub struct CpuMoveEvaluator;

impl MoveEvaluator for CpuMoveEvaluator {
    fn evaluate(
        &self,
        problem: &PlacementProblem,
        instances: &[MacroInstance],
        model: &PlacementCostModel,
        world: DimSize,
        moves: &[PlacementMove],
    ) -> eyre::Result<Vec<MoveDelta>> {
        let before = placement_cost(problem, instances, model, world)?;
        let mut moved = instances.to_vec();
        let mut deltas = Vec::with_capacity(moves.len());
        for mv in moves {
            moved[mv.instance].position = mv.position;
            let after = placement_cost(problem, &moved, model, world)?;
            deltas.push(MoveDelta::between(&before, &after));
            moved[mv.instance].position = instances[mv.instance].position;
        }
        Ok(deltas)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cost(total: f64) -> PlacementCost {
        PlacementCost {
            wire_length: 10,
            bounding_box_volume: 20,
            blocked_pins: 1,
            overlapping_pairs: 2,
            spacing_violations: 3,
            pin_access_violations: 4,
            total,
        }
    }

    #[test]
    fn delta_subtracts_every_component() {
        let before = cost(100.0);
        let after = PlacementCost {
            wire_length: 12,
            bounding_box_volume: 18,
            blocked_pins: 0,
            overlapping_pairs: 3,
            spacing_violations: 1,
            pin_access_violations: 6,
            total: 90.0,
        };
        let delta = MoveDelta::between(&before, &after);
        assert_eq!(delta.wire_length, 2);
        assert_eq!(delta.bounding_box_volume, -2);
        assert_eq!(delta.blocked_pins, -1);
        assert_eq!(delta.overlapping_pairs, 1);
        assert_eq!(delta.spacing_violations, -2);
        assert_eq!(delta.pin_access_violations, 2);
        assert_eq!(delta.total, -10.0);
    }
}
