//! Negotiated-congestion post-pass over routed nets (M4.3).
//!
//! After the greedy net loop produces a route for every connection, this pass
//! looks for cells shared by several routes, rips up the routes crossing
//! them, and reroutes those nets with congestion penalties. Only routes with
//! the default simple power contract are rerouted; adapter routes with extra
//! required positions are left untouched. Every accepted reroute must keep
//! its power contract on the assembled world.

use super::congestion::{CongestionConfig, CongestionMap};
use super::cost::RouteCostModel;
use super::engine::route_point_to_point_with_cost_model_and_congestion;
use super::pathfinder::PathfinderConfig;
use super::validation::route_power_contract_holds;
use crate::transform::place_and_route::global_pnr::router::RoutedNet;
use crate::world::World3D;

#[derive(Debug)]
pub(crate) struct NegotiationOutcome {
    pub(crate) routes: Vec<RoutedNet>,
    pub(crate) rerouted: usize,
    pub(crate) overused_cells: usize,
}

pub(crate) fn assemble_world_with_routes(base_world: &World3D, routes: &[RoutedNet]) -> World3D {
    let mut world = assemble_world_with_routes_raw(base_world, routes);
    world.initialize_redstone_states();
    world
}

fn assemble_world_with_routes_raw(base_world: &World3D, routes: &[RoutedNet]) -> World3D {
    let mut world = base_world.clone();
    for route in routes {
        for &(position, block) in &route.blocks {
            if world.size.bound_on(position) {
                world[position] = block;
            }
        }
    }
    world
}

fn redstone_supports_are_consistent(world: &World3D) -> bool {
    world.iter_block().into_iter().all(|(position, block)| {
        if !block.kind.is_redstone() {
            return true;
        }
        position
            .down()
            .is_none_or(|down| world[down].kind.is_cobble())
    })
}

fn is_negotiable(route: &RoutedNet) -> bool {
    route.source != route.sink
        && route.required_powered_positions.len() == 1
        && route.required_powered_positions[0] == route.sink
        && route.required_released_positions.len() == 1
        && route.required_released_positions[0] == route.sink
}

pub(crate) fn negotiate_routed_nets(
    base_world: &World3D,
    routes: &[RoutedNet],
    config: &PathfinderConfig,
) -> NegotiationOutcome {
    let cost_model = RouteCostModel {
        congestion: CongestionConfig {
            present_penalty: config.present_penalty,
            history_penalty: config.history_penalty,
        },
        ..config.cost_model
    };

    let mut current = routes.to_vec();
    let mut map = CongestionMap::new();
    for route in &current {
        map.add_route(&route.path);
    }
    let mut rerouted = 0usize;

    for _ in 0..config.max_iterations.max(1) {
        let overused = map.overused_cells();
        if overused.is_empty() {
            break;
        }
        map.commit_iteration();

        for index in 0..current.len() {
            if !is_negotiable(&current[index])
                || !current[index]
                    .path
                    .iter()
                    .any(|position| overused.contains(position))
            {
                continue;
            }

            let old_path = current[index].path.clone();
            map.rip_route(&old_path);

            let Ok((mut new_route, _)) = route_point_to_point_with_cost_model_and_congestion(
                base_world,
                current[index].source,
                current[index].sink,
                config.strategy,
                cost_model,
                &map,
            ) else {
                map.add_route(&old_path);
                continue;
            };

            new_route.net_id = current[index].net_id;
            new_route.source_endpoint = current[index].source_endpoint.clone();
            new_route.sink_endpoint = current[index].sink_endpoint.clone();
            new_route.source_label = current[index].source_label.clone();
            new_route.sink_label = current[index].sink_label.clone();

            let mut candidate = current.clone();
            candidate[index] = new_route;
            let mut candidate_world = assemble_world_with_routes_raw(base_world, &candidate);
            if !redstone_supports_are_consistent(&candidate_world) {
                map.add_route(&old_path);
                continue;
            }
            candidate_world.initialize_redstone_states();
            if route_power_contract_holds(&candidate_world, &candidate_world, &candidate[index]) {
                map.add_route(&candidate[index].path);
                current = candidate;
                rerouted += 1;
            } else {
                map.add_route(&old_path);
            }
        }
    }

    NegotiationOutcome {
        routes: current,
        rerouted,
        overused_cells: map.overused_cells().len(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::transform::place_and_route::global_pnr::router::GlobalRoutingStrategy;
    use crate::world::block::{Block, BlockKind, Direction};
    use crate::world::position::{DimSize, Position};

    fn cobble() -> Block {
        Block {
            kind: BlockKind::Cobble {
                on_count: 0,
                on_base_count: 0,
            },
            direction: Direction::None,
        }
    }

    fn redstone_block() -> Block {
        Block {
            kind: BlockKind::Redstone {
                on_count: 0,
                state: 0,
                strength: 0,
            },
            direction: Direction::None,
        }
    }

    fn world_with_pairs(size: DimSize, pairs: &[(Position, Position)]) -> World3D {
        let mut world = World3D::new(size);
        for (source, sink) in pairs {
            world[source.down().unwrap()] = cobble();
            world[*source] = redstone_block();
            world[sink.down().unwrap()] = cobble();
            world[*sink] = redstone_block();
        }
        world.initialize_redstone_states();
        world
    }

    fn route(world: &World3D, source: Position, sink: Position) -> RoutedNet {
        route_point_to_point_with_cost_model_and_congestion(
            world,
            source,
            sink,
            GlobalRoutingStrategy::AStar,
            RouteCostModel::default(),
            &CongestionMap::new(),
        )
        .expect("route")
        .0
    }

    #[test]
    fn a_clean_route_set_is_returned_unchanged() {
        let source = Position(0, 1, 1);
        let sink = Position(5, 1, 1);
        let world = world_with_pairs(DimSize(8, 4, 3), &[(source, sink)]);
        let routes = [route(&world, source, sink)];

        let outcome = negotiate_routed_nets(&world, &routes, &PathfinderConfig::default());

        assert_eq!(outcome.rerouted, 0);
        assert_eq!(outcome.overused_cells, 0);
        assert_eq!(outcome.routes[0].path, routes[0].path);
    }

    #[test]
    fn crossing_routes_separate_and_keep_their_contract() {
        let pairs = [
            (Position(0, 0, 1), Position(6, 2, 1)),
            (Position(0, 2, 1), Position(6, 0, 1)),
        ];
        let world = world_with_pairs(DimSize(8, 4, 3), &pairs);
        let routes = [
            route(&world, pairs[0].0, pairs[0].1),
            route(&world, pairs[1].0, pairs[1].1),
        ];
        let config = PathfinderConfig {
            max_iterations: 16,
            present_penalty: 100,
            history_penalty: 50,
            ..Default::default()
        };

        let outcome = negotiate_routed_nets(&world, &routes, &config);

        assert!(outcome.rerouted >= 1, "{outcome:?}");
        assert_eq!(outcome.overused_cells, 0);
        assert_eq!(outcome.routes.len(), 2);
        let assembled = assemble_world_with_routes(&world, &outcome.routes);
        assert!(
            route_power_contract_holds(&assembled, &assembled, &outcome.routes[0])
                && route_power_contract_holds(&assembled, &assembled, &outcome.routes[1])
        );
    }

    #[test]
    fn negotiation_is_deterministic() {
        let pairs = [
            (Position(0, 0, 1), Position(6, 2, 1)),
            (Position(0, 2, 1), Position(6, 0, 1)),
        ];
        let world = world_with_pairs(DimSize(8, 4, 3), &pairs);
        let routes = [
            route(&world, pairs[0].0, pairs[0].1),
            route(&world, pairs[1].0, pairs[1].1),
        ];
        let config = PathfinderConfig {
            max_iterations: 16,
            present_penalty: 100,
            history_penalty: 50,
            ..Default::default()
        };

        let first = negotiate_routed_nets(&world, &routes, &config);
        let second = negotiate_routed_nets(&world, &routes, &config);

        let first_paths = first
            .routes
            .iter()
            .map(|route| route.path.clone())
            .collect::<Vec<_>>();
        let second_paths = second
            .routes
            .iter()
            .map(|route| route.path.clone())
            .collect::<Vec<_>>();
        assert_eq!(first_paths, second_paths);
        assert_eq!(first.rerouted, second.rerouted);
    }
}
