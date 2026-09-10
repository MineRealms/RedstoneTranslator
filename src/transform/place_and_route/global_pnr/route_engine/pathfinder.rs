//! Negotiated-congestion routing loop (M4.2).
//!
//! Routes a flat list of point-to-point nets over one world using the
//! congestion map as the inter-net conflict signal: each pass routes the nets
//! that are missing, folds present overuse into history, then rips up every
//! net crossing an overused cell for the next pass. Nets are routed
//! independently on the original world; physical assembly and shared-world
//! conflicts stay with the router integration.

use crate::transform::place_and_route::global_pnr::route_engine::{
    route_point_to_point_with_cost_model_and_congestion, CongestionConfig, CongestionMap,
    RouteCostModel,
};
use crate::transform::place_and_route::global_pnr::router::{GlobalRoutingStrategy, RoutedNet};
use crate::world::position::Position;
use crate::world::World3D;

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PathfinderNet {
    pub name: String,
    pub source: Position,
    pub sink: Position,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PathfinderConfig {
    pub max_iterations: usize,
    pub present_penalty: usize,
    pub history_penalty: usize,
    pub cost_model: RouteCostModel,
    pub strategy: GlobalRoutingStrategy,
}

impl Default for PathfinderConfig {
    fn default() -> Self {
        Self {
            max_iterations: 8,
            present_penalty: 10,
            history_penalty: 5,
            cost_model: RouteCostModel::default(),
            strategy: GlobalRoutingStrategy::AStar,
        }
    }
}

#[derive(Clone, Debug)]
pub struct PathfinderRoute {
    pub net: usize,
    pub route: Option<RoutedNet>,
}

#[derive(Clone, Debug)]
pub struct PathfinderResult {
    pub routes: Vec<PathfinderRoute>,
    pub iterations: usize,
    pub overused_cells: Vec<Position>,
}

impl PathfinderResult {
    pub fn unrouted(&self) -> usize {
        self.routes
            .iter()
            .filter(|route| route.route.is_none())
            .count()
    }
}

pub fn negotiate_routes(
    world: &World3D,
    nets: &[PathfinderNet],
    config: &PathfinderConfig,
) -> PathfinderResult {
    let cost_model = RouteCostModel {
        congestion: CongestionConfig {
            present_penalty: config.present_penalty,
            history_penalty: config.history_penalty,
        },
        ..config.cost_model
    };

    let mut map = CongestionMap::new();
    let mut routes: Vec<Option<RoutedNet>> = vec![None; nets.len()];
    let mut iterations = 0;

    for iteration in 0..config.max_iterations.max(1) {
        iterations = iteration + 1;

        for (index, net) in nets.iter().enumerate() {
            if routes[index].is_some() {
                continue;
            }
            if let Ok((route, _)) = route_point_to_point_with_cost_model_and_congestion(
                world,
                net.source,
                net.sink,
                config.strategy,
                cost_model,
                &map,
            ) {
                map.add_route(&route.path);
                routes[index] = Some(route);
            }
        }

        let overused = map.overused_cells();
        if overused.is_empty() || iteration + 1 == config.max_iterations.max(1) {
            break;
        }

        map.commit_iteration();
        for route in routes.iter_mut() {
            let Some(current) = route else {
                continue;
            };
            if current
                .path
                .iter()
                .any(|position| overused.contains(position))
            {
                map.rip_route(&current.path);
                *route = None;
            }
        }
    }

    PathfinderResult {
        routes: routes
            .into_iter()
            .enumerate()
            .map(|(net, route)| PathfinderRoute { net, route })
            .collect(),
        iterations,
        overused_cells: map.overused_cells(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::world::block::{Block, BlockKind, Direction};
    use crate::world::position::DimSize;

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

    fn net(name: &str, source: Position, sink: Position) -> PathfinderNet {
        PathfinderNet {
            name: name.to_owned(),
            source,
            sink,
        }
    }

    #[test]
    fn single_net_finishes_in_one_iteration() {
        let source = Position(0, 1, 1);
        let sink = Position(5, 1, 1);
        let world = world_with_pairs(DimSize(8, 4, 3), &[(source, sink)]);
        let nets = [net("n0", source, sink)];

        let result = negotiate_routes(&world, &nets, &PathfinderConfig::default());

        assert_eq!(result.iterations, 1);
        assert_eq!(result.unrouted(), 0);
        assert!(result.overused_cells.is_empty());
    }

    #[test]
    fn identical_nets_report_persistent_overuse() {
        let source = Position(0, 0, 1);
        let sink = Position(5, 0, 1);
        let world = world_with_pairs(DimSize(8, 1, 2), &[(source, sink)]);
        let nets = [net("n0", source, sink), net("n1", source, sink)];
        let config = PathfinderConfig {
            max_iterations: 3,
            ..Default::default()
        };

        let result = negotiate_routes(&world, &nets, &config);

        assert_eq!(result.iterations, 3);
        assert_eq!(result.unrouted(), 0);
        assert!(!result.overused_cells.is_empty());
    }

    #[test]
    fn negotiation_is_deterministic() {
        let pairs = [
            (Position(0, 0, 1), Position(6, 2, 1)),
            (Position(0, 2, 1), Position(6, 0, 1)),
        ];
        let world = world_with_pairs(DimSize(8, 4, 3), &pairs);
        let nets = [
            net("a", pairs[0].0, pairs[0].1),
            net("b", pairs[1].0, pairs[1].1),
        ];

        let first = negotiate_routes(&world, &nets, &PathfinderConfig::default());
        let second = negotiate_routes(&world, &nets, &PathfinderConfig::default());

        let first_paths = first
            .routes
            .iter()
            .map(|route| route.route.as_ref().map(|route| route.path.clone()))
            .collect::<Vec<_>>();
        let second_paths = second
            .routes
            .iter()
            .map(|route| route.route.as_ref().map(|route| route.path.clone()))
            .collect::<Vec<_>>();
        assert_eq!(first_paths, second_paths);
        assert_eq!(first.iterations, second.iterations);
    }

    #[test]
    fn crossing_nets_separate_with_negotiation() {
        let pairs = [
            (Position(0, 0, 1), Position(6, 2, 1)),
            (Position(0, 2, 1), Position(6, 0, 1)),
        ];
        let world = world_with_pairs(DimSize(8, 4, 3), &pairs);
        let nets = [
            net("a", pairs[0].0, pairs[0].1),
            net("b", pairs[1].0, pairs[1].1),
        ];

        let config = PathfinderConfig {
            max_iterations: 16,
            present_penalty: 100,
            history_penalty: 50,
            ..Default::default()
        };
        let result = negotiate_routes(&world, &nets, &config);

        assert_eq!(result.unrouted(), 0);
        assert!(
            result.overused_cells.is_empty(),
            "negotiation left overused cells: {:?} a={:?} b={:?}",
            result.overused_cells,
            result.routes[0]
                .route
                .as_ref()
                .map(|route| route.path.clone()),
            result.routes[1]
                .route
                .as_ref()
                .map(|route| route.path.clone())
        );
    }
}
