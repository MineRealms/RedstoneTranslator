//! Search state for the point-to-point routing engine.

use crate::transform::place_and_route::global_pnr::router::GlobalRoutingStrategy;
use crate::transform::place_and_route::place_bound::PlaceBound;
use crate::world::block::BlockKind;
use crate::world::position::Position;
use crate::world::World3D;

pub(crate) const MAX_REDSTONE_STRENGTH: usize = 15;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct PoweredRouteSource {
    pub(crate) position: Position,
    pub(crate) strength: usize,
}

#[derive(Clone)]
pub(crate) struct RouteSearchState {
    pub(crate) world: World3D,
    pub(crate) terminal: Position,
    pub(crate) route: Vec<Position>,
    pub(crate) signal_strength: usize,
    pub(crate) powered_taps: Vec<PoweredRouteSource>,
    pub(crate) pending_bounds: Option<Vec<PlaceBound>>,
}

pub(crate) fn route_visited_key(
    strategy: GlobalRoutingStrategy,
    state: &RouteSearchState,
) -> (Position, usize, usize) {
    (
        state.terminal,
        route_visited_depth(strategy, state.route.len().saturating_sub(1)),
        state.signal_strength,
    )
}

pub(crate) fn route_visited_depth(strategy: GlobalRoutingStrategy, route_depth: usize) -> usize {
    match strategy {
        GlobalRoutingStrategy::BreadthFirst => route_depth,
        GlobalRoutingStrategy::AStar
        | GlobalRoutingStrategy::DirectGreedy { .. }
        | GlobalRoutingStrategy::GreedyBeam { .. } => 0,
    }
}

pub(crate) fn initial_signal_strength(world: &World3D, source: Position) -> usize {
    match world[source].kind {
        BlockKind::Redstone { strength, .. } if strength > 0 => strength,
        BlockKind::Redstone { .. } => 2,
        BlockKind::Switch { .. }
        | BlockKind::Torch { .. }
        | BlockKind::Repeater { .. }
        | BlockKind::RedstoneBlock => MAX_REDSTONE_STRENGTH,
        _ => 0,
    }
}

pub(crate) fn powered_route_source(
    world: &World3D,
    position: Position,
) -> Option<PoweredRouteSource> {
    (world.size.bound_on(position) && is_route_terminal(world, position)).then_some(
        PoweredRouteSource {
            position,
            strength: initial_signal_strength(world, position),
        },
    )
}

pub(crate) fn is_route_terminal(world: &World3D, position: Position) -> bool {
    world[position].kind.is_redstone()
        || world[position].kind.is_switch()
        || world[position].kind.is_torch()
        || world[position].kind.is_repeater()
        || matches!(world[position].kind, BlockKind::RedstoneBlock)
}
