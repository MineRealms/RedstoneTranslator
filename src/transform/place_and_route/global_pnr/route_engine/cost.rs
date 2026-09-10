//! Search cost model for the point-to-point routing engine.
//!
//! The default model reproduces the historical priority formula exactly:
//! one unit per step and a fixed penalty for routes at low signal strength.
//! Turn and repeater terms are available for future cost-driven routing and
//! are zero by default so extraction stays behavior-equivalent.

use super::congestion::{CongestionConfig, CongestionMap};
use super::state::RouteSearchState;
use crate::world::position::Position;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RouteCostModel {
    pub(crate) step_cost: usize,
    pub(crate) turn_cost: usize,
    pub(crate) repeater_cost: usize,
    pub(crate) low_strength_penalty: usize,
    pub(crate) congestion: CongestionConfig,
}

impl Default for RouteCostModel {
    fn default() -> Self {
        Self {
            step_cost: 1,
            turn_cost: 0,
            repeater_cost: 0,
            low_strength_penalty: 4,
            congestion: CongestionConfig::default(),
        }
    }
}

impl RouteCostModel {
    pub(crate) fn weighted_route_cost(&self, state: &RouteSearchState) -> usize {
        let steps = state.route.len().saturating_sub(1);
        let mut cost = self.step_cost * steps;

        if self.turn_cost > 0 {
            cost += self.turn_cost * route_turns(&state.route);
        }

        if self.repeater_cost > 0 {
            cost += self.repeater_cost * route_repeaters(state);
        }

        cost
    }

    pub(crate) fn congestion_cost(&self, position: Position, map: &CongestionMap) -> usize {
        map.cell_cost(position, &self.congestion)
    }

    pub(crate) fn low_strength_penalty(&self, state: &RouteSearchState) -> usize {
        usize::from(state.signal_strength <= 2) * self.low_strength_penalty
    }
}

fn route_turns(route: &[Position]) -> usize {
    route
        .windows(3)
        .filter(|window| window[0].diff(window[1]) != window[1].diff(window[2]))
        .count()
}

fn route_repeaters(state: &RouteSearchState) -> usize {
    state
        .route
        .iter()
        .filter(|position| state.world[**position].kind.is_repeater())
        .count()
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
    fn default_model_matches_the_legacy_formula() {
        let state = state(vec![
            Position(0, 0, 0),
            Position(1, 0, 0),
            Position(2, 0, 0),
        ]);
        let model = RouteCostModel::default();

        assert_eq!(model.weighted_route_cost(&state), 2);
        assert_eq!(model.low_strength_penalty(&state), 0);

        let mut low = state;
        low.signal_strength = 2;
        assert_eq!(model.low_strength_penalty(&low), 4);
    }

    #[test]
    fn turn_cost_counts_direction_changes() {
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
        let model = RouteCostModel {
            turn_cost: 3,
            ..Default::default()
        };

        assert_eq!(model.weighted_route_cost(&straight), 3);
        assert_eq!(model.weighted_route_cost(&zigzag), 9);
    }

    #[test]
    fn repeater_cost_counts_repeaters_on_the_route() {
        let mut state = state(vec![
            Position(0, 0, 0),
            Position(1, 0, 0),
            Position(2, 0, 0),
        ]);
        state.world[Position(1, 0, 0)] = Block {
            kind: BlockKind::Repeater {
                is_on: false,
                is_locked: false,
                delay: 1,
                lock_input1: None,
                lock_input2: None,
            },
            direction: Direction::East,
        };
        let model = RouteCostModel {
            repeater_cost: 8,
            ..Default::default()
        };

        assert_eq!(model.weighted_route_cost(&state), 10);
    }
}
