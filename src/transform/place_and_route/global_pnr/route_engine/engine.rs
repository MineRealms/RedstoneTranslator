//! Point-to-point routing engine extracted from `router.rs`.
//!
//! M2.0 moves the existing search machinery here unchanged so behavior stays
//! identical while `router.rs` keeps net ordering, fanout handling, topology
//! orchestration, and simulator validation.

use std::collections::{HashMap, HashSet, VecDeque};

use super::cost::RouteCostModel;
use super::goal::RouteGoal;
use super::queue::{route_expansion_limit, RouteSearchQueue};
use super::state::{
    initial_signal_strength, is_route_terminal, route_visited_depth, route_visited_key,
    PoweredRouteSource, RouteSearchState, MAX_REDSTONE_STRENGTH,
};
use crate::transform::place_and_route::detailed_router::{
    self, PlaceRedstoneResult, PlaceRepeaterResult,
};
use crate::transform::place_and_route::global_pnr::router::{
    GlobalRoutingStrategy, RouteFailure, RoutedNet,
};
use crate::transform::place_and_route::place_bound::{PlaceBound, PropagateType};
use crate::transform::place_and_route::placed_node::PlacedNode;
use crate::world::block::{Block, BlockKind, Direction};
use crate::world::position::Position;
use crate::world::World3D;

const GLOBAL_ROUTE_MAX_STEPS: usize = 128;
const SIGNAL_CONTACT_SEARCH_RADIUS: usize = 3;
const OUTPUT_ISOLATION_ESCAPE_MAX_STEPS: usize = 4;

pub fn route_point_to_point(
    world: &World3D,
    source: Position,
    sink: Position,
) -> Result<(RoutedNet, World3D), RouteFailure> {
    route_point_to_point_with_strategy(world, source, sink, GlobalRoutingStrategy::BreadthFirst)
}

pub fn route_point_to_point_with_strategy(
    world: &World3D,
    source: Position,
    sink: Position,
    strategy: GlobalRoutingStrategy,
) -> Result<(RoutedNet, World3D), RouteFailure> {
    route_point_to_point_with_cost_model(world, source, sink, strategy, RouteCostModel::default())
}

pub(crate) fn route_point_to_point_with_strategy_and_allowed_contacts(
    world: &World3D,
    source: Position,
    sink: Position,
    strategy: GlobalRoutingStrategy,
    additional_allowed_contacts: Vec<Position>,
) -> Result<(RoutedNet, World3D), RouteFailure> {
    route_point_to_point_with_strategy_and_allowed_contacts_and_initial_strength(
        world,
        source,
        sink,
        strategy,
        additional_allowed_contacts,
        initial_signal_strength(world, source),
    )
}

pub(crate) fn route_point_to_point_with_strategy_and_allowed_contacts_and_initial_strength(
    world: &World3D,
    source: Position,
    sink: Position,
    strategy: GlobalRoutingStrategy,
    additional_allowed_contacts: Vec<Position>,
    initial_strength: usize,
) -> Result<(RoutedNet, World3D), RouteFailure> {
    let goal = RouteGoal::for_sink(world, sink);
    route_point_to_point_with_bounds_and_initial_strength(
        world,
        source,
        sink,
        goal,
        BoundSearchMode::Propagation,
        None,
        strategy,
        &additional_allowed_contacts,
        initial_strength,
        RouteCostModel::default(),
    )
    .or_else(|_| {
        route_point_to_point_with_bounds_and_initial_strength(
            world,
            source,
            sink,
            goal,
            BoundSearchMode::Nearby,
            None,
            strategy,
            &additional_allowed_contacts,
            initial_strength,
            RouteCostModel::default(),
        )
    })
}

pub(crate) fn routeable_output_taps(
    world: &World3D,
    source: Position,
    sink: Position,
) -> Vec<(Position, usize)> {
    let direct_taps = world
        .iter_block()
        .into_iter()
        .filter(|(position, block)| {
            block.kind.is_redstone()
                && detailed_router::target_powers_position(world, source, *position)
        })
        .map(|(position, _)| position)
        .collect::<Vec<_>>();

    let mut taps = powered_redstone_network_taps(world, &direct_taps);
    taps.sort_by_key(|(position, strength)| {
        (
            position.manhattan_distance(&sink),
            std::cmp::Reverse(*strength),
            position.manhattan_distance(&source),
            position.0,
            position.1,
            position.2,
        )
    });
    taps
}

fn powered_redstone_network_taps(world: &World3D, seeds: &[Position]) -> Vec<(Position, usize)> {
    let redstones = world
        .iter_block()
        .into_iter()
        .filter_map(|(position, block)| block.kind.is_redstone().then_some(position))
        .collect::<Vec<_>>();
    let mut strengths = HashMap::<Position, usize>::new();
    let mut queue = VecDeque::new();
    for &seed in seeds {
        if strengths
            .insert(seed, MAX_REDSTONE_STRENGTH)
            .is_none_or(|old| old < MAX_REDSTONE_STRENGTH)
        {
            queue.push_back(seed);
        }
    }

    while let Some(position) = queue.pop_front() {
        let Some(&strength) = strengths.get(&position) else {
            continue;
        };
        if strength <= 1 {
            continue;
        }

        for &next in &redstones {
            if position == next {
                continue;
            }
            if !detailed_router::target_powers_position(world, position, next)
                && !detailed_router::target_powers_position(world, next, position)
            {
                continue;
            }

            let next_strength = strength - 1;
            if strengths.get(&next).is_none_or(|old| *old < next_strength) {
                strengths.insert(next, next_strength);
                queue.push_back(next);
            }
        }
    }

    strengths.into_iter().collect()
}

pub(crate) fn redstone_network_positions(world: &World3D, seeds: &[Position]) -> Vec<Position> {
    let redstones = world
        .iter_block()
        .into_iter()
        .filter_map(|(position, block)| block.kind.is_redstone().then_some(position))
        .collect::<Vec<_>>();
    let mut visited = HashSet::new();
    let mut queue = VecDeque::new();
    for &seed in seeds {
        if visited.insert(seed) {
            queue.push_back(seed);
        }
    }

    while let Some(position) = queue.pop_front() {
        for &next in &redstones {
            if visited.contains(&next) {
                continue;
            }
            if detailed_router::target_powers_position(world, position, next)
                || detailed_router::target_powers_position(world, next, position)
            {
                visited.insert(next);
                queue.push_back(next);
            }
        }
    }

    visited.into_iter().collect()
}

pub(crate) fn isolated_output_repeater_initial_states(
    world: &World3D,
    logical_source: Position,
    route_source: Position,
    sink: Position,
    additional_allowed_contacts: &[Position],
) -> Vec<RouteSearchState> {
    let mut seeds = vec![(route_source, initial_signal_strength(world, route_source))];
    seeds.extend(routeable_output_taps(world, route_source, sink));
    seeds.sort_by_key(|(position, strength)| {
        (
            position.manhattan_distance(&sink),
            std::cmp::Reverse(*strength),
            position.0,
            position.1,
            position.2,
        )
    });
    seeds.dedup_by_key(|(position, _)| *position);

    let mut states = Vec::new();
    let mut output_allowed_contacts = additional_allowed_contacts.to_vec();
    output_allowed_contacts.extend(source_signal_positions(world, logical_source));
    output_allowed_contacts.extend(source_signal_positions(world, route_source));
    output_allowed_contacts.sort();
    output_allowed_contacts.dedup();

    for (seed, _) in seeds {
        if !world.size.bound_on(seed) || !is_route_terminal(world, seed) {
            continue;
        }

        for (adapter_world, driver_position) in
            output_repeater_adapters(world, seed, &output_allowed_contacts)
        {
            states.push(RouteSearchState {
                world: adapter_world,
                terminal: driver_position,
                route: vec![seed, driver_position],
                signal_strength: MAX_REDSTONE_STRENGTH,
                powered_taps: vec![PoweredRouteSource {
                    position: driver_position,
                    strength: MAX_REDSTONE_STRENGTH,
                }],
                pending_bounds: None,
            });
        }

        states.extend(escaped_output_repeater_initial_states(
            world,
            seed,
            sink,
            &output_allowed_contacts,
        ));
    }

    states
}

fn escaped_output_repeater_initial_states(
    world: &World3D,
    seed: Position,
    sink: Position,
    output_allowed_contacts: &[Position],
) -> Vec<RouteSearchState> {
    let goal = RouteGoal::for_sink(world, sink);
    let forbidden_signal_contacts =
        route_forbidden_signal_contact_positions(world, seed, goal, output_allowed_contacts);
    let allowed_shorts = output_allowed_contacts
        .iter()
        .copied()
        .collect::<HashSet<_>>();
    let mut states = Vec::new();
    let mut visited = HashSet::from([seed]);
    let mut queue = VecDeque::from([RouteSearchState {
        world: world.clone(),
        terminal: seed,
        route: vec![seed],
        signal_strength: initial_signal_strength(world, seed),
        powered_taps: vec![PoweredRouteSource {
            position: seed,
            strength: initial_signal_strength(world, seed),
        }],
        pending_bounds: None,
    }]);

    while let Some(state) = queue.pop_front() {
        let escape_steps = state.route.len().saturating_sub(1);
        if escape_steps > 0 {
            for (adapter_world, driver_position) in
                output_repeater_adapters(&state.world, state.terminal, output_allowed_contacts)
            {
                let mut route = state.route.clone();
                route.push(driver_position);
                let mut powered_taps = state.powered_taps.clone();
                powered_taps.push(PoweredRouteSource {
                    position: driver_position,
                    strength: MAX_REDSTONE_STRENGTH,
                });
                states.push(RouteSearchState {
                    world: adapter_world,
                    terminal: driver_position,
                    route,
                    signal_strength: MAX_REDSTONE_STRENGTH,
                    powered_taps,
                    pending_bounds: None,
                });
            }
        }

        if escape_steps >= OUTPUT_ISOLATION_ESCAPE_MAX_STEPS || state.signal_strength <= 1 {
            continue;
        }

        let terminal_node = PlacedNode::new(state.terminal, state.world[state.terminal]);
        for bound in
            route_bounds_for_mode(BoundSearchMode::Propagation, &state.world, &terminal_node)
        {
            if !bound.is_bound_on(&state.world) || goal.rejects_bound_position(bound.position()) {
                continue;
            }

            let PlaceRedstoneResult::Placed(next_world, redstone_node) =
                detailed_router::place_redstone_with_cobble_and_allowed_shorts(
                    &state.world,
                    bound,
                    state.terminal,
                    sink,
                    Some(&allowed_shorts),
                )
            else {
                continue;
            };

            if route_touches_forbidden_existing_signal(
                &next_world,
                redstone_node.position,
                &forbidden_signal_contacts,
            ) {
                continue;
            }

            if visited.insert(redstone_node.position) {
                let mut route = state.route.clone();
                route.push(redstone_node.position);
                let next_strength = state.signal_strength - 1;
                let mut powered_taps = state.powered_taps.clone();
                powered_taps.push(PoweredRouteSource {
                    position: redstone_node.position,
                    strength: next_strength,
                });
                queue.push_back(RouteSearchState {
                    world: next_world,
                    terminal: redstone_node.position,
                    route,
                    signal_strength: next_strength,
                    powered_taps,
                    pending_bounds: None,
                });
            }
        }
    }

    states
}

fn output_repeater_adapters(
    world: &World3D,
    source: Position,
    additional_allowed_contacts: &[Position],
) -> Vec<(World3D, Position)> {
    source
        .cardinal()
        .into_iter()
        .filter_map(|repeater_position| {
            let direction = source.diff(repeater_position).inverse();
            output_repeater_adapter_world(
                world,
                source,
                repeater_position,
                direction,
                additional_allowed_contacts,
            )
        })
        .collect()
}

fn output_repeater_adapter_world(
    world: &World3D,
    source: Position,
    repeater_position: Position,
    direction: Direction,
    additional_allowed_contacts: &[Position],
) -> Option<(World3D, Position)> {
    if !world.size.bound_on(repeater_position) || !world[repeater_position].kind.is_air() {
        return None;
    }
    let driver_position = repeater_position.walk(direction.inverse())?;
    if !world.size.bound_on(driver_position) || !world[driver_position].kind.is_air() {
        return None;
    }
    let repeater_support_position = repeater_position.down()?;
    let driver_support_position = driver_position.down()?;
    if !world.size.bound_on(repeater_support_position)
        || !world.size.bound_on(driver_support_position)
    {
        return None;
    }

    let mut adapter_world = world.clone();
    place_support_cobble_if_needed(&mut adapter_world, repeater_support_position)?;
    place_support_cobble_if_needed(&mut adapter_world, driver_support_position)?;

    let repeater = PlacedNode::new_repeater(repeater_position, direction);
    if repeater.has_conflict(&adapter_world, &[source].into_iter().collect()) {
        return None;
    }
    detailed_router::place_node(&mut adapter_world, repeater);
    if !detailed_router::target_powers_position(&adapter_world, source, repeater_position) {
        return None;
    }
    if adapter_touches_forbidden_existing_signal(
        world,
        &adapter_world,
        repeater_position,
        &output_adapter_allowed_contacts(
            world,
            source,
            additional_allowed_contacts,
            &[source, repeater_position],
        ),
    ) {
        return None;
    }

    let driver = PlacedNode::new_redstone(driver_position);
    if driver.has_conflict(&adapter_world, &[repeater_position].into_iter().collect()) {
        return None;
    }
    detailed_router::place_node(&mut adapter_world, driver);
    if !detailed_router::target_powers_position(&adapter_world, repeater_position, driver_position)
    {
        return None;
    }
    if adapter_touches_forbidden_existing_signal(
        world,
        &adapter_world,
        driver_position,
        &output_adapter_allowed_contacts(
            world,
            source,
            additional_allowed_contacts,
            &[source, repeater_position, driver_position],
        ),
    ) {
        return None;
    }

    Some((adapter_world, driver_position))
}

fn output_adapter_allowed_contacts(
    world: &World3D,
    source: Position,
    additional_allowed_contacts: &[Position],
    local_allowed_contacts: &[Position],
) -> Vec<Position> {
    let mut contacts =
        adapter_allowed_contacts(additional_allowed_contacts, local_allowed_contacts);
    contacts.extend(source_signal_positions(world, source));
    contacts.sort();
    contacts.dedup();
    contacts
}

pub(crate) fn sorted_route_bounds(
    mut bounds: Vec<PlaceBound>,
    world: &World3D,
    sink: Position,
) -> Vec<PlaceBound> {
    bounds.sort_by_key(|bound| {
        let position = bound.position();
        (
            !world.size.bound_on(position),
            world
                .size
                .bound_on(position)
                .then(|| !world[position].kind.is_air())
                .unwrap_or(true),
            position.manhattan_distance(&sink),
            position.0,
            position.1,
            position.2,
        )
    });
    bounds
}

#[derive(Clone, Copy, Debug)]
enum BoundSearchMode {
    Propagation,
    Nearby,
}

fn route_point_to_point_with_bounds_and_initial_strength(
    world: &World3D,
    source: Position,
    sink: Position,
    goal: RouteGoal,
    mode: BoundSearchMode,
    min_z: Option<usize>,
    strategy: GlobalRoutingStrategy,
    additional_allowed_contacts: &[Position],
    initial_strength: usize,
    cost_model: RouteCostModel,
) -> Result<(RoutedNet, World3D), RouteFailure> {
    route_point_to_point_with_initial_queue(
        world,
        source,
        sink,
        goal,
        mode,
        min_z,
        vec![RouteSearchState {
            world: world.clone(),
            terminal: source,
            route: vec![source],
            signal_strength: initial_strength,
            powered_taps: vec![PoweredRouteSource {
                position: source,
                strength: initial_strength,
            }],
            pending_bounds: None,
        }],
        strategy,
        additional_allowed_contacts,
        cost_model,
    )
}

pub(crate) fn route_point_to_point_from_initial_state(
    world: &World3D,
    source: Position,
    sink: Position,
    initial_state: RouteSearchState,
    strategy: GlobalRoutingStrategy,
    additional_allowed_contacts: &[Position],
) -> Result<(RoutedNet, World3D), RouteFailure> {
    route_point_to_point_from_initial_state_with_cost(
        world,
        source,
        sink,
        initial_state,
        strategy,
        additional_allowed_contacts,
        RouteCostModel::default(),
    )
}

pub(crate) fn route_point_to_point_with_cost_model(
    world: &World3D,
    source: Position,
    sink: Position,
    strategy: GlobalRoutingStrategy,
    cost_model: RouteCostModel,
) -> Result<(RoutedNet, World3D), RouteFailure> {
    let initial_strength = initial_signal_strength(world, source);
    let initial_state = RouteSearchState {
        world: world.clone(),
        terminal: source,
        route: vec![source],
        signal_strength: initial_strength,
        powered_taps: vec![PoweredRouteSource {
            position: source,
            strength: initial_strength,
        }],
        pending_bounds: None,
    };

    route_point_to_point_from_initial_state_with_cost(
        world,
        source,
        sink,
        initial_state,
        strategy,
        &[],
        cost_model,
    )
}

fn route_point_to_point_from_initial_state_with_cost(
    world: &World3D,
    source: Position,
    sink: Position,
    initial_state: RouteSearchState,
    strategy: GlobalRoutingStrategy,
    additional_allowed_contacts: &[Position],
    cost_model: RouteCostModel,
) -> Result<(RoutedNet, World3D), RouteFailure> {
    let goal = RouteGoal::for_sink(world, sink);
    route_point_to_point_with_initial_queue(
        world,
        source,
        sink,
        goal,
        BoundSearchMode::Propagation,
        None,
        vec![initial_state.clone()],
        strategy,
        additional_allowed_contacts,
        cost_model,
    )
    .or_else(|_| {
        route_point_to_point_with_initial_queue(
            world,
            source,
            sink,
            goal,
            BoundSearchMode::Nearby,
            None,
            vec![initial_state],
            strategy,
            additional_allowed_contacts,
            cost_model,
        )
    })
}

fn route_point_to_point_with_initial_queue(
    original_world: &World3D,
    source: Position,
    sink: Position,
    goal: RouteGoal,
    mode: BoundSearchMode,
    min_z: Option<usize>,
    initial_states: Vec<RouteSearchState>,
    strategy: GlobalRoutingStrategy,
    additional_allowed_contacts: &[Position],
    cost_model: RouteCostModel,
) -> Result<(RoutedNet, World3D), RouteFailure> {
    let initial_visited = initial_states
        .iter()
        .map(|state| route_visited_key(strategy, state));
    let mut visited = initial_visited.collect::<HashSet<_>>();
    let mut queue = RouteSearchQueue::new(strategy, sink, initial_states, cost_model);
    let forbidden_signal_contacts = route_forbidden_signal_contact_positions(
        original_world,
        source,
        goal,
        additional_allowed_contacts,
    );
    let mut expansions = 0usize;

    while let Some(state) = queue.pop() {
        expansions += 1;
        if route_expansion_limit(strategy).is_some_and(|limit| expansions > limit) {
            break;
        }

        if !is_route_terminal(&state.world, state.terminal) {
            continue;
        }
        if goal.accepts(&state.world, state.terminal) {
            let blocks = added_route_blocks(original_world, &state.world);
            return Ok((
                RoutedNet::new(source, sink, blocks, state.route)
                    .with_powered_taps(state.powered_taps),
                state.world,
            ));
        }

        if state.route.len() > GLOBAL_ROUTE_MAX_STEPS {
            continue;
        }

        let terminal_node = PlacedNode::new(state.terminal, state.world[state.terminal]);
        let allowed_shorts = goal.allowed_short_positions();
        let mut bounds = state
            .pending_bounds
            .unwrap_or_else(|| route_bounds_for_mode(mode, &state.world, &terminal_node));
        bounds.sort_by_key(|bound| {
            let position = bound.position();
            (
                position.manhattan_distance(&sink),
                position.0,
                position.1,
                position.2,
            )
        });
        for bound in bounds {
            if !bound.is_bound_on(&state.world) || goal.rejects_bound_position(bound.position()) {
                continue;
            }
            if min_z.is_some_and(|min_z| {
                bound.position().2 < min_z && bound.position().manhattan_distance(&source) <= 4
            }) {
                continue;
            }

            if state.signal_strength > 1 {
                match detailed_router::place_redstone_with_cobble_and_allowed_shorts(
                    &state.world,
                    bound,
                    state.terminal,
                    goal.placement_target(),
                    allowed_shorts.as_ref(),
                ) {
                    PlaceRedstoneResult::Placed(next_world, redstone_node) => {
                        if route_touches_forbidden_existing_signal(
                            &next_world,
                            redstone_node.position,
                            &forbidden_signal_contacts,
                        ) {
                            continue;
                        }
                        let next_strength = state.signal_strength - 1;
                        if visited.insert((
                            redstone_node.position,
                            route_visited_depth(strategy, state.route.len()),
                            next_strength,
                        )) {
                            let mut next_route = state.route.clone();
                            next_route.push(redstone_node.position);
                            let mut powered_taps = state.powered_taps.clone();
                            powered_taps.push(PoweredRouteSource {
                                position: redstone_node.position,
                                strength: next_strength,
                            });
                            queue.push(RouteSearchState {
                                world: next_world,
                                terminal: redstone_node.position,
                                route: next_route,
                                signal_strength: next_strength,
                                powered_taps,
                                pending_bounds: None,
                            });
                        }
                    }
                    PlaceRedstoneResult::Rejected(_) => {}
                }
            }

            if state.signal_strength <= 2 {
                for direction in Direction::iter_direction_without_top()
                    .into_iter()
                    .filter(|direction| direction.is_cardinal())
                {
                    match detailed_router::place_repeater_with_cobble(
                        &state.world,
                        bound,
                        state.terminal,
                        goal.placement_target(),
                        direction,
                        allowed_shorts.as_ref(),
                    ) {
                        PlaceRepeaterResult::Placed(next_world, repeater_node) => {
                            if route_touches_forbidden_existing_signal(
                                &next_world,
                                repeater_node.position,
                                &forbidden_signal_contacts,
                            ) {
                                continue;
                            }
                            if visited.insert((
                                repeater_node.position,
                                route_visited_depth(strategy, state.route.len()),
                                MAX_REDSTONE_STRENGTH,
                            )) {
                                let mut next_route = state.route.clone();
                                next_route.push(repeater_node.position);
                                let mut powered_taps = state.powered_taps.clone();
                                powered_taps.push(PoweredRouteSource {
                                    position: repeater_node.position,
                                    strength: MAX_REDSTONE_STRENGTH,
                                });
                                queue.push(RouteSearchState {
                                    world: next_world,
                                    terminal: repeater_node.position,
                                    route: next_route,
                                    signal_strength: MAX_REDSTONE_STRENGTH,
                                    powered_taps,
                                    pending_bounds: None,
                                });
                            }
                        }
                        PlaceRepeaterResult::Rejected(_) => {}
                    }
                }
            }
        }
    }

    Err(RouteFailure::Unreachable { source, sink })
}

fn route_forbidden_signal_contact_positions(
    world: &World3D,
    source: Position,
    goal: RouteGoal,
    additional_allowed_contacts: &[Position],
) -> Vec<Position> {
    let mut allowed = HashSet::from([source, goal.placement_target()]);
    allowed.extend(additional_allowed_contacts.iter().copied());
    allowed.extend(source_signal_positions(world, source));
    allowed.extend(goal_contact_positions(world, goal));
    world
        .iter_block()
        .into_iter()
        .filter_map(|(position, block)| {
            (!allowed.contains(&position) && is_signal_terminal_block(block)).then_some(position)
        })
        .collect()
}

fn source_signal_positions(world: &World3D, source: Position) -> HashSet<Position> {
    let mut positions = HashSet::from([source]);
    if !world.size.bound_on(source) {
        return positions;
    }
    if world[source].kind.is_redstone() {
        positions.extend(redstone_network_positions(world, &[source]));
    }

    let direct_taps = world
        .iter_block()
        .into_iter()
        .filter_map(|(position, block)| {
            (block.kind.is_redstone()
                && detailed_router::target_powers_position(world, source, position))
            .then_some(position)
        })
        .collect::<Vec<_>>();
    positions.extend(direct_taps.iter().copied());
    positions.extend(
        powered_redstone_network_taps(world, &direct_taps)
            .into_iter()
            .map(|(position, _)| position),
    );
    positions
}

fn goal_contact_positions(world: &World3D, goal: RouteGoal) -> HashSet<Position> {
    let target = goal.placement_target();
    let mut positions = HashSet::from([target]);
    positions.extend(
        target
            .cardinal()
            .into_iter()
            .filter(|position| world.size.bound_on(*position)),
    );
    let top = target.up();
    if world.size.bound_on(top) {
        positions.insert(top);
    }
    positions
}

fn route_touches_forbidden_existing_signal(
    routed_world: &World3D,
    route_position: Position,
    forbidden_contacts: &[Position],
) -> bool {
    forbidden_contacts.iter().copied().any(|position| {
        if route_position.manhattan_distance(&position) > SIGNAL_CONTACT_SEARCH_RADIUS {
            return false;
        }
        detailed_router::target_powers_position(routed_world, route_position, position)
            || detailed_router::target_powers_position(routed_world, position, route_position)
    })
}

pub(crate) fn is_signal_terminal_block(block: Block) -> bool {
    block.kind.is_redstone()
        || block.kind.is_switch()
        || block.kind.is_torch()
        || block.kind.is_repeater()
        || matches!(block.kind, BlockKind::RedstoneBlock)
}

fn route_bounds_for_mode(
    mode: BoundSearchMode,
    world: &World3D,
    terminal_node: &PlacedNode,
) -> Vec<PlaceBound> {
    match mode {
        BoundSearchMode::Propagation => terminal_node.propagation_bound(Some(world)),
        BoundSearchMode::Nearby
            if terminal_node.block.kind.is_repeater()
                || terminal_node.block.kind.is_torch()
                || terminal_node.block.kind.is_switch() =>
        {
            terminal_node.propagation_bound(Some(world))
        }
        BoundSearchMode::Nearby => nearby_route_bounds(world, terminal_node.position),
    }
}

fn nearby_route_bounds(world: &World3D, position: Position) -> Vec<PlaceBound> {
    let mut result = Vec::new();
    for next in nearby_route_positions(world, position) {
        result.push(PlaceBound(PropagateType::Soft, next, next.diff(position)));
    }
    result
}

fn nearby_route_positions(world: &World3D, position: Position) -> Vec<Position> {
    let mut result = Vec::new();
    let horizontal = [
        (position.0.checked_add(1), Some(position.1)),
        (Some(position.0), position.1.checked_add(1)),
        (position.0.checked_sub(1), Some(position.1)),
        (Some(position.0), position.1.checked_sub(1)),
    ];

    for (next_x, next_y) in horizontal {
        let (Some(next_x), Some(next_y)) = (next_x, next_y) else {
            continue;
        };
        for next_z in [position.2, position.2 + 1] {
            let next = Position(next_x, next_y, next_z);
            if world.size.bound_on(next) {
                result.push(next);
            }
        }
        if let Some(next_z) = position.2.checked_sub(1) {
            let next = Position(next_x, next_y, next_z);
            if world.size.bound_on(next) {
                result.push(next);
            }
        }
    }
    result
}

pub(crate) fn added_route_blocks(before: &World3D, after: &World3D) -> Vec<(Position, Block)> {
    after
        .iter_block()
        .into_iter()
        .filter(|(position, block)| !block.kind.is_air() && before[*position] != *block)
        .collect()
}

pub(crate) fn adapter_allowed_contacts(
    additional_allowed_contacts: &[Position],
    local_allowed_contacts: &[Position],
) -> Vec<Position> {
    let mut contacts = additional_allowed_contacts.to_vec();
    contacts.extend(local_allowed_contacts.iter().copied());
    contacts.sort();
    contacts.dedup();
    contacts
}

pub(crate) fn adapter_touches_forbidden_existing_signal(
    original_world: &World3D,
    adapter_world: &World3D,
    adapter_position: Position,
    allowed_contacts: &[Position],
) -> bool {
    let allowed_contacts = allowed_contacts.iter().copied().collect::<HashSet<_>>();
    original_world
        .iter_block()
        .into_iter()
        .any(|(position, block)| {
            if allowed_contacts.contains(&position) || !is_signal_terminal_block(block) {
                return false;
            }
            detailed_router::target_powers_position(adapter_world, adapter_position, position)
                || detailed_router::target_powers_position(
                    adapter_world,
                    position,
                    adapter_position,
                )
        })
}

pub(crate) fn place_support_cobble_if_needed(
    world: &mut World3D,
    position: Position,
) -> Option<()> {
    if world[position].kind.is_air() {
        let support = PlacedNode::new_cobble(position);
        if support.has_conflict(world, &HashSet::new()) {
            return None;
        }
        detailed_router::place_node(world, support);
    } else if !world[position].kind.is_cobble() {
        return None;
    }
    Some(())
}
