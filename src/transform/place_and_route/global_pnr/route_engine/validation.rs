//! Simulator-backed validation for completed routes.
//!
//! Geometric checks stay in the search expansion (level 1). This module owns
//! the level 2 contract: settle the routed world with the discrete-event
//! simulator and verify that every required position is powered while the
//! source is active, and released when a switch source is off.

use std::collections::{HashSet, VecDeque};

use super::engine::is_signal_terminal_block;
use crate::transform::place_and_route::detailed_router;
use crate::transform::place_and_route::global_pnr::assembly::reset_dynamic_power_states;
use crate::transform::place_and_route::global_pnr::router::{
    GlobalRoutingStrategy, RouteValidationMode, RoutedNet,
};
use crate::world::block::BlockKind;
use crate::world::position::Position;
use crate::world::simulator::Simulator;
use crate::world::{World, World3D};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct RouteValidationResult {
    pub(crate) accepted: bool,
}

pub(crate) trait RouteValidator {
    fn validate(
        &self,
        before: &World3D,
        after: &World3D,
        route: &RoutedNet,
    ) -> RouteValidationResult;
}

pub(crate) struct SimulatorRouteValidator;

impl RouteValidator for SimulatorRouteValidator {
    fn validate(
        &self,
        before: &World3D,
        after: &World3D,
        route: &RoutedNet,
    ) -> RouteValidationResult {
        RouteValidationResult {
            accepted: active_route_powers_sink(before, after, route),
        }
    }
}

pub(crate) fn active_route_powers_sink(
    before: &World3D,
    after: &World3D,
    route: &RoutedNet,
) -> bool {
    if !can_validate_active_route_source(before, route.source)
        && !can_validate_active_route_source(after, route.source)
    {
        return true;
    }

    let Some(before_source_active) = route_source_power_after_settle(before, route.source, false)
    else {
        return false;
    };
    let Some((after_source_active, required_positions_powered)) =
        active_route_power_after_settle(after, route)
    else {
        return false;
    };

    if before_source_active && !after_source_active {
        return false;
    }
    if !after_source_active {
        return true;
    }

    required_positions_powered
}

pub(crate) fn route_candidate_powers_sink(
    before: &World3D,
    after: &World3D,
    route: &RoutedNet,
    strategy: GlobalRoutingStrategy,
) -> bool {
    // DirectGreedy is the cheap global probe. Its complete routed world is
    // dynamically validated by global PnR, so simulating every tentative tap,
    // adapter, and source alternative here only multiplies rejection cost.
    matches!(
        strategy,
        GlobalRoutingStrategy::DirectGreedy { .. } | GlobalRoutingStrategy::GreedyBeam { .. }
    ) || SimulatorRouteValidator
        .validate(before, after, route)
        .accepted
}

fn active_route_power_after_settle(world: &World3D, route: &RoutedNet) -> Option<(bool, bool)> {
    let mut world = world.clone();
    reset_dynamic_power_states(&mut world);
    world.initialize_redstone_states();
    if matches!(world[route.source].kind, BlockKind::Switch { .. }) {
        world[route.source].kind = BlockKind::Switch { is_on: true };
    }

    let world = World::from(&world);
    let sim = Simulator::from_preserving_torch_states_with_limits_and_trace(&world, 256, 50_000, 0)
        .ok()?;
    Some((
        block_is_powered(sim.world(), route.source),
        route
            .required_powered_positions
            .iter()
            .all(|position| block_is_powered(sim.world(), *position)),
    ))
}

fn route_source_power_after_settle(
    world: &World3D,
    source: Position,
    force_switch_on: bool,
) -> Option<bool> {
    let mut world = world.clone();
    reset_dynamic_power_states(&mut world);
    world.initialize_redstone_states();
    if force_switch_on && matches!(world[source].kind, BlockKind::Switch { .. }) {
        world[source].kind = BlockKind::Switch { is_on: true };
    }

    let world = World::from(&world);
    let sim = Simulator::from_preserving_torch_states_with_limits_and_trace(&world, 256, 50_000, 0)
        .ok()?;
    Some(block_is_powered(sim.world(), source))
}

pub(crate) fn route_power_contract_holds(
    before: &World3D,
    after: &World3D,
    route: &RoutedNet,
) -> bool {
    route_power_contract_failure_reason(before, after, route).is_none()
}

pub(crate) fn eager_route_failure_reason(
    validation: RouteValidationMode,
    before: &World3D,
    after: &World3D,
    route: &RoutedNet,
) -> Option<&'static str> {
    if route_has_signal_feedback_cycle(after, route) {
        return Some("route contains a self-sustaining signal feedback cycle");
    }
    if validation == RouteValidationMode::Deferred {
        return None;
    }
    route_power_contract_failure_reason_without_cycle(before, after, route)
}

fn route_power_contract_failure_reason(
    before: &World3D,
    after: &World3D,
    route: &RoutedNet,
) -> Option<&'static str> {
    if route_has_signal_feedback_cycle(after, route) {
        return Some("route contains a self-sustaining signal feedback cycle");
    }
    route_power_contract_failure_reason_without_cycle(before, after, route)
}

fn route_power_contract_failure_reason_without_cycle(
    before: &World3D,
    after: &World3D,
    route: &RoutedNet,
) -> Option<&'static str> {
    if !active_route_powers_sink(before, after, route) {
        return Some("active source does not power all required route positions");
    }
    if !switch_route_releases_required_positions_when_off(after, route) {
        return Some("switch-off source still powers at least one required route position");
    }

    None
}

pub(crate) fn switch_route_releases_required_positions_when_off(
    world: &World3D,
    route: &RoutedNet,
) -> bool {
    if !matches!(world[route.source].kind, BlockKind::Switch { .. }) {
        return true;
    }

    let mut inactive_world = world.clone();
    reset_dynamic_power_states(&mut inactive_world);
    inactive_world[route.source].kind = BlockKind::Switch { is_on: false };
    inactive_world.initialize_redstone_states();

    let world = World::from(&inactive_world);
    let Ok(sim) =
        Simulator::from_preserving_torch_states_with_limits_and_trace(&world, 256, 50_000, 0)
    else {
        return false;
    };

    !route
        .required_released_positions
        .iter()
        .any(|position| block_is_powered(sim.world(), *position))
}

pub(crate) fn first_invalid_active_route<'a>(
    world: &World3D,
    routes: &'a [RoutedNet],
) -> Option<&'a RoutedNet> {
    routes
        .iter()
        .find(|route| !route_power_contract_holds(world, world, route))
}

pub(crate) fn can_validate_active_route_source(world: &World3D, position: Position) -> bool {
    matches!(
        world[position].kind,
        BlockKind::Redstone { .. }
            | BlockKind::Torch { .. }
            | BlockKind::Repeater { .. }
            | BlockKind::Switch { .. }
    )
}

fn block_is_powered(world: &World3D, position: Position) -> bool {
    world[position].kind.is_powered()
}
fn route_has_signal_feedback_cycle(world: &World3D, route: &RoutedNet) -> bool {
    let signal_positions = route
        .path
        .iter()
        .copied()
        .chain(route.blocks.iter().map(|(position, _)| *position))
        .filter(|position| {
            world.size.bound_on(*position) && is_signal_terminal_block(world[*position])
        })
        .collect::<HashSet<_>>();

    for start in signal_positions
        .iter()
        .copied()
        .filter(|position| matches!(world[*position].kind, BlockKind::Repeater { .. }))
    {
        let mut visited = HashSet::from([start]);
        let mut frontier = VecDeque::from([start]);
        while let Some(source) = frontier.pop_front() {
            for target in signal_positions.iter().copied() {
                if target == source
                    || !detailed_router::target_powers_position(world, source, target)
                {
                    continue;
                }
                if target == start {
                    return true;
                }
                if visited.insert(target) {
                    frontier.push_back(target);
                }
            }
        }
    }
    false
}
