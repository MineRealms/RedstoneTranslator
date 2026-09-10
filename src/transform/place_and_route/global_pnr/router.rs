use std::collections::{HashMap, HashSet};

use eyre::ContextCompat;

use crate::output::OutputEndpoint;
use crate::transform::place_and_route::detailed_router;
use crate::transform::place_and_route::global_pnr::heuristics::GlobalHeuristicHooks;
use crate::transform::place_and_route::global_pnr::ir::{
    LayoutCandidate, PhysicalPort, PhysicalPortDirection,
};
use crate::transform::place_and_route::global_pnr::physical_intent::ResolvedPhysicalIntent;
use crate::transform::place_and_route::global_pnr::placer::PlacedModule;
use crate::transform::place_and_route::global_pnr::progress::GlobalPnrProgress;
pub(crate) use crate::transform::place_and_route::global_pnr::route_engine::first_invalid_active_route;
use crate::transform::place_and_route::global_pnr::route_engine::{
    adapter_allowed_contacts, adapter_touches_forbidden_existing_signal, added_route_blocks,
    eager_route_failure_reason, initial_signal_strength, is_route_terminal,
    isolated_output_repeater_initial_states, place_support_cobble_if_needed, powered_route_source,
    redstone_network_positions, route_candidate_powers_sink,
    route_point_to_point_from_initial_state,
    route_point_to_point_with_strategy_and_allowed_contacts,
    route_point_to_point_with_strategy_and_allowed_contacts_and_initial_strength,
    routeable_output_taps, sorted_route_bounds, PoweredRouteSource, RouteSearchState,
    MAX_REDSTONE_STRENGTH,
};
pub use crate::transform::place_and_route::global_pnr::route_engine::{
    route_point_to_point, route_point_to_point_with_strategy,
};
use crate::transform::place_and_route::global_pnr::topology::{
    NetId, ResolvedEndpoint, ResolvedPnrTopology,
};
use crate::transform::place_and_route::placed_node::PlacedNode;
use crate::world::block::{Block, BlockKind, Direction};
use crate::world::position::{DimSize, Position};
use crate::world::World3D;

const GLOBAL_ROUTE_PADDING: usize = 8;
const FANOUT_ROUTE_SOURCE_LIMIT: usize = 8;
const FANOUT_ROUTE_TERMINAL_LIMIT: usize = 24;

#[derive(Clone, Debug)]
struct RoutingTopInput {
    name: String,
    targets: Vec<(String, String)>,
}

#[derive(Clone, Debug)]
pub(crate) struct RoutingConnection {
    pub(crate) source: (String, String),
    pub(crate) target: (String, String),
}

#[derive(Clone, Debug, Default)]
struct RoutingPlan {
    top_inputs: Vec<RoutingTopInput>,
    vars: Vec<RoutingConnection>,
}

impl RoutingPlan {
    fn from_resolved(topology: &ResolvedPnrTopology) -> eyre::Result<Self> {
        let mut plan = Self::default();
        for net in &topology.nets {
            match &net.driver {
                ResolvedEndpoint::TopPort { port } => {
                    let port = topology
                        .port(*port)
                        .context("resolved routing plan has an unknown top input")?;
                    let targets = net
                        .sinks
                        .iter()
                        .filter_map(|sink| resolved_instance_port_pair(topology, sink))
                        .collect::<Vec<_>>();
                    if !targets.is_empty() {
                        plan.top_inputs.push(RoutingTopInput {
                            name: port.name.clone(),
                            targets,
                        });
                    }
                }
                ResolvedEndpoint::InstancePort { .. } => {
                    let source = resolved_instance_port_pair(topology, &net.driver)
                        .context("resolved routing plan has an invalid driver")?;
                    for sink in &net.sinks {
                        let Some(target) = resolved_instance_port_pair(topology, sink) else {
                            continue;
                        };
                        plan.vars.push(RoutingConnection {
                            source: source.clone(),
                            target,
                        });
                    }
                }
            }
        }
        Ok(plan)
    }
}

fn resolved_instance_port_pair(
    topology: &ResolvedPnrTopology,
    endpoint: &ResolvedEndpoint,
) -> Option<(String, String)> {
    let ResolvedEndpoint::InstancePort { instance, port } = endpoint else {
        return None;
    };
    Some((
        topology.instances.get(instance.0)?.display_name.clone(),
        topology.port(*port)?.name.clone(),
    ))
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GlobalRoutingStrategy {
    BreadthFirst,
    AStar,
    DirectGreedy {
        max_steps: usize,
    },
    GreedyBeam {
        beam_width: usize,
        max_expansions: usize,
        /// Deterministically varies equal-cost route choices without changing
        /// the search budget or local/placement decisions.
        variant_seed: u64,
    },
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum NetOrderStrategy {
    #[default]
    Criticality,
    HighestFanoutFirst,
    ReverseCriticality,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum RouteValidationMode {
    /// Validate every accepted branch immediately. This gives the router
    /// feedback at the highest cost.
    #[default]
    Incremental,
    /// Defer dynamic simulation until the complete routed world is evaluated
    /// by global PnR. Geometric and feedback-cycle checks still run eagerly.
    Deferred,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct GlobalRoutingConfig {
    pub strategy: GlobalRoutingStrategy,
    pub validation: RouteValidationMode,
}

impl Default for GlobalRoutingConfig {
    fn default() -> Self {
        Self {
            strategy: GlobalRoutingStrategy::AStar,
            validation: RouteValidationMode::Incremental,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct ResolvedPortTarget {
    position: Position,
    requires_input_diode: bool,
    input_repeater_delay: usize,
}

#[derive(Clone)]
struct InputDiodeAdapter {
    world: World3D,
    driver: Position,
    repeater: Position,
    target: Position,
}

#[derive(Clone)]
struct ExternalInputSource {
    world: World3D,
    switch: Position,
    route_source: Position,
    blocks: Vec<(Position, Block)>,
}

#[derive(Clone, Debug)]
pub struct RoutedNet {
    /// Stable logical identity assigned by the resolved PnR topology. Legacy
    /// router entry points leave this empty; prepared global PnR always fills
    /// it before the route leaves this module.
    pub net_id: Option<NetId>,
    pub source_endpoint: Option<ResolvedEndpoint>,
    pub sink_endpoint: Option<ResolvedEndpoint>,
    pub source_label: Option<String>,
    pub sink_label: Option<String>,
    pub source: Position,
    pub sink: Position,
    pub blocks: Vec<(Position, Block)>,
    pub path: Vec<Position>,
    pub required_powered_positions: Vec<Position>,
    pub required_released_positions: Vec<Position>,
    pub powered_taps: Vec<(Position, usize)>,
}

impl RoutedNet {
    pub(crate) fn new(
        source: Position,
        sink: Position,
        blocks: Vec<(Position, Block)>,
        path: Vec<Position>,
    ) -> Self {
        let powered_taps = path
            .last()
            .copied()
            .map(|position| (position, MAX_REDSTONE_STRENGTH))
            .into_iter()
            .collect();
        Self {
            net_id: None,
            source_endpoint: None,
            sink_endpoint: None,
            source_label: None,
            sink_label: None,
            source,
            sink,
            blocks,
            path,
            required_powered_positions: vec![sink],
            required_released_positions: vec![sink],
            powered_taps,
        }
    }

    fn with_topology_identity(
        mut self,
        net_id: NetId,
        source: ResolvedEndpoint,
        sink: Option<ResolvedEndpoint>,
    ) -> Self {
        self.net_id = Some(net_id);
        self.source_endpoint = Some(source);
        self.sink_endpoint = sink;
        self
    }

    fn with_labels(mut self, source: impl Into<String>, sink: impl Into<String>) -> Self {
        self.source_label = Some(source.into());
        self.sink_label = Some(sink.into());
        self
    }

    fn with_required_powered_positions(mut self, positions: Vec<Position>) -> Self {
        self.required_powered_positions = positions.clone();
        self.required_released_positions = positions;
        self
    }

    fn with_required_released_positions(mut self, positions: Vec<Position>) -> Self {
        self.required_released_positions = positions;
        self
    }

    pub(crate) fn with_powered_taps(mut self, taps: Vec<PoweredRouteSource>) -> Self {
        self.powered_taps = taps
            .into_iter()
            .map(|source| (source.position, source.strength))
            .collect();
        self
    }

    fn powered_route_sources(&self) -> Vec<PoweredRouteSource> {
        self.powered_taps
            .iter()
            .map(|(position, strength)| PoweredRouteSource {
                position: *position,
                strength: *strength,
            })
            .collect()
    }
}

/// Route through the compatibility implementation, then bind every physical
/// branch to the typed topology that initiated the global PnR run. This is the
/// migration boundary: callers no longer need to rediscover net identity from
/// display labels after routing.
pub fn route_resolved_topology_with_order_from_prefix(
    topology: &ResolvedPnrTopology,
    intent: Option<&ResolvedPhysicalIntent>,
    hooks: &GlobalHeuristicHooks,
    candidates: &[LayoutCandidate],
    placed_modules: &[PlacedModule],
    config: &GlobalRoutingConfig,
    order_strategy: NetOrderStrategy,
    progress: &GlobalPnrProgress,
    prefix: &[RoutedNet],
) -> Result<Vec<RoutedNet>, PartialRoutingFailure> {
    let plan = RoutingPlan::from_resolved(topology).map_err(|error| PartialRoutingFailure {
        error,
        routed_nets: prefix.to_vec(),
    })?;
    match route_module_variables_with_order_from_prefix_impl(
        Some(topology),
        intent,
        hooks,
        &plan,
        candidates,
        placed_modules,
        config,
        order_strategy,
        progress,
        prefix,
    ) {
        Ok(mut routes) => {
            bind_route_topology(topology, candidates, placed_modules, &mut routes);
            Ok(routes)
        }
        Err(mut failure) => {
            bind_route_topology(
                topology,
                candidates,
                placed_modules,
                &mut failure.routed_nets,
            );
            Err(failure)
        }
    }
}

fn bind_route_topology(
    topology: &ResolvedPnrTopology,
    candidates: &[LayoutCandidate],
    placed_modules: &[PlacedModule],
    routes: &mut [RoutedNet],
) {
    for route in routes {
        if route.net_id.is_some() {
            continue;
        }
        let Some(source_label) = route.source_label.as_deref() else {
            continue;
        };
        let Some(net) = topology.net_by_driver_label(source_label) else {
            continue;
        };
        let sink = route
            .sink_label
            .as_deref()
            .and_then(|label| topology.sink_by_label(net, label).cloned())
            .or_else(|| {
                net.sinks.iter().find_map(|endpoint| {
                    endpoint_matches_route_sink(
                        topology,
                        endpoint,
                        candidates,
                        placed_modules,
                        route,
                    )
                    .then(|| endpoint.clone())
                })
            });
        *route = route
            .clone()
            .with_topology_identity(net.id, net.driver.clone(), sink);
    }
}

fn endpoint_matches_route_sink(
    topology: &ResolvedPnrTopology,
    endpoint: &ResolvedEndpoint,
    candidates: &[LayoutCandidate],
    placed_modules: &[PlacedModule],
    route: &RoutedNet,
) -> bool {
    let ResolvedEndpoint::InstancePort { instance, port } = endpoint else {
        return false;
    };
    let Some(instance) = topology.instances.get(instance.0) else {
        return false;
    };
    let Some(port) = topology.port(*port) else {
        return false;
    };
    resolve_port_targets(
        candidates,
        placed_modules,
        &instance.display_name,
        &port.name,
    )
    .iter()
    .any(|target| {
        target.position == route.sink || route.required_powered_positions.contains(&target.position)
    })
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum RouteFailure {
    Unreachable { source: Position, sink: Position },
}

#[derive(Debug)]
pub struct PartialRoutingFailure {
    pub error: eyre::Report,
    pub routed_nets: Vec<RoutedNet>,
}

fn route_module_variables_with_order_from_prefix_impl(
    topology: Option<&ResolvedPnrTopology>,
    intent: Option<&ResolvedPhysicalIntent>,
    hooks: &GlobalHeuristicHooks,
    plan: &RoutingPlan,
    candidates: &[LayoutCandidate],
    placed_modules: &[PlacedModule],
    config: &GlobalRoutingConfig,
    order_strategy: NetOrderStrategy,
    progress: &GlobalPnrProgress,
    prefix: &[RoutedNet],
) -> Result<Vec<RoutedNet>, PartialRoutingFailure> {
    let mut route_world = placed_candidate_world(candidates, placed_modules).map_err(|error| {
        PartialRoutingFailure {
            error,
            routed_nets: Vec::new(),
        }
    })?;
    for route in prefix {
        for &(position, block) in &route.blocks {
            if !route_world.size.bound_on(position) {
                return Err(PartialRoutingFailure {
                    error: eyre::eyre!("routed prefix block {position:?} is outside route world"),
                    routed_nets: Vec::new(),
                });
            }
            route_world[position] = block;
        }
    }
    route_world.initialize_redstone_states();
    let mut routes = prefix.to_vec();

    let completed_connections = prefix
        .iter()
        .filter_map(|route| Some((route.source_label.as_ref()?, route.sink_label.as_ref()?)))
        .collect::<HashSet<_>>();
    let completed_typed_connections = prefix
        .iter()
        .filter_map(|route| Some((route.net_id?, route.sink_endpoint.clone()?)))
        .collect::<HashSet<_>>();
    let mut ordered_vars = ordered_module_variables(&plan.vars, order_strategy);
    if let Some(topology) = topology {
        ordered_vars.sort_by_key(|var| {
            let label = format!("{}.{}", var.source.0, var.source.1);
            let priority = topology.net_by_driver_label(&label).map_or(0, |net| {
                intent.map_or(0, |intent| intent.net_priority(net.id))
                    + hooks
                        .net_priority_terms
                        .iter()
                        .map(|hook| (hook.evaluate)(topology, net.id, intent))
                        .sum::<usize>()
            });
            std::cmp::Reverse(priority)
        });
    }
    let vars = ordered_vars
        .into_iter()
        .filter(|var| {
            if let Some(connection) = topology.and_then(|topology| {
                let source = format!("{}.{}", var.source.0, var.source.1);
                let sink = format!("{}.{}", var.target.0, var.target.1);
                let net = topology.net_by_driver_label(&source)?;
                Some((net.id, topology.sink_by_label(net, &sink)?.clone()))
            }) {
                return !completed_typed_connections.contains(&connection);
            }
            let source = format!("{}.{}", var.source.0, var.source.1);
            let sink = format!("{}.{}", var.target.0, var.target.1);
            !completed_connections
                .iter()
                .any(|(completed_source, completed_sink)| {
                    completed_source.as_str() == source && completed_sink.as_str() == sink
                })
        })
        .collect::<Vec<_>>();

    let has_internal_prefix = prefix.iter().any(|route| {
        matches!(
            route.source_endpoint,
            Some(ResolvedEndpoint::InstancePort { .. })
        ) || route
            .source_label
            .as_deref()
            .is_some_and(|label| label.contains('.'))
    });
    if !has_internal_prefix {
        routes.clear();
        route_world = placed_candidate_world(candidates, placed_modules).map_err(|error| {
            PartialRoutingFailure {
                error,
                routed_nets: Vec::new(),
            }
        })?;
    }

    let completed_top_inputs = routes
        .iter()
        .filter_map(|route| route.source_label.as_deref())
        .filter(|label| !label.contains('.'))
        .map(str::to_owned)
        .collect::<HashSet<_>>();
    if let Err(error) = route_top_input_ports(
        &plan.top_inputs,
        candidates,
        placed_modules,
        config,
        progress,
        &completed_top_inputs,
        &mut route_world,
        &mut routes,
    ) {
        return Err(PartialRoutingFailure {
            error,
            routed_nets: routes,
        });
    }

    if let Err(error) = route_internal_module_nets(
        &vars,
        candidates,
        placed_modules,
        config,
        progress,
        &mut route_world,
        &mut routes,
    ) {
        return Err(PartialRoutingFailure {
            error,
            routed_nets: routes,
        });
    }

    if config.validation == RouteValidationMode::Incremental
        && let Some(route) = first_invalid_active_route(&route_world, &routes)
    {
        return Err(PartialRoutingFailure {
            error: eyre::eyre!(
                "routed net from {:?} to {:?} no longer powers its sink in the final routed world",
                route.source,
                route.sink,
            ),
            routed_nets: routes,
        });
    }

    Ok(routes)
}

pub(crate) fn ordered_module_variables(
    vars: &[RoutingConnection],
    strategy: NetOrderStrategy,
) -> Vec<&RoutingConnection> {
    let mut ordered = vars.iter().collect::<Vec<_>>();
    match strategy {
        NetOrderStrategy::Criticality => {
            ordered.sort_by_key(|var| route_variable_priority(var));
        }
        NetOrderStrategy::ReverseCriticality => {
            ordered.sort_by_key(|var| route_variable_priority(var));
            ordered.reverse();
        }
        NetOrderStrategy::HighestFanoutFirst => {
            let fanout = vars.iter().fold(HashMap::new(), |mut counts, var| {
                *counts.entry(var.source.clone()).or_insert(0usize) += 1;
                counts
            });
            ordered.sort_by_key(|var| {
                (
                    std::cmp::Reverse(fanout.get(&var.source).copied().unwrap_or_default()),
                    route_variable_priority(var),
                )
            });
        }
    }
    ordered
}

fn route_internal_module_nets(
    vars: &[&RoutingConnection],
    candidates: &[LayoutCandidate],
    placed_modules: &[PlacedModule],
    config: &GlobalRoutingConfig,
    progress: &GlobalPnrProgress,
    route_world: &mut World3D,
    routes: &mut Vec<RoutedNet>,
) -> eyre::Result<()> {
    let grouped_vars = group_vars_by_source_ordered(vars);
    for (group_index, group_vars) in grouped_vars.iter().enumerate() {
        let source_key = group_vars[0].source.clone();

        let (source_port, source_candidate, source_placed) =
            resolve_port(candidates, placed_modules, &source_key.0, &source_key.1).with_context(
                || {
                    format!(
                        "source port {}.{} is not placed",
                        source_key.0, source_key.1
                    )
                },
            )?;
        let source_positions =
            translate_port_access_positions(source_port, source_candidate, source_placed);
        let source = source_positions.first().copied().unwrap_or_else(|| {
            translate_port_route_position(source_port, source_candidate, source_placed)
        });
        let logical_source =
            translate_candidate_position(source_port.position, source_candidate, source_placed);

        let mut sink_targets = Vec::new();
        for var in group_vars {
            let sinks =
                resolve_port_targets(candidates, placed_modules, &var.target.0, &var.target.1);
            if sinks.is_empty() {
                return Err(eyre::eyre!(
                    "target port {}.{} is not placed",
                    var.target.0,
                    var.target.1
                ));
            }
            for sink in sinks {
                sink_targets.push((*var, sink));
            }
        }
        let all_sinks = sink_targets
            .iter()
            .map(|(_, sink)| *sink)
            .collect::<Vec<_>>();

        let mut route_sources = source_positions
            .iter()
            .copied()
            .chain([logical_source])
            .filter_map(|position| powered_route_source(route_world, position))
            .collect::<Vec<_>>();
        if route_sources.is_empty() {
            route_sources.push(PoweredRouteSource {
                position: source,
                strength: initial_signal_strength(route_world, source),
            });
        }
        let mut route_source_set = route_sources
            .iter()
            .map(|source| source.position)
            .collect::<HashSet<_>>();
        let total_sinks = sink_targets.len();
        let mut routed_sinks = 0usize;
        while !sink_targets.is_empty() {
            sort_internal_sink_targets_by_current_tree(&mut sink_targets, &route_sources);
            let mut selected_route = None;
            let mut last_error = None;

            for (sink_index, (var, sink)) in sink_targets.iter().copied().enumerate() {
                progress.item(
                    group_index + 1,
                    grouped_vars.len(),
                    format!(
                        "route net `{}.{}` -> `{}.{}` sink {}/{}",
                        source_key.0,
                        source_key.1,
                        var.target.0,
                        var.target.1,
                        routed_sinks + 1,
                        total_sinks
                    ),
                );
                let route_result = if routed_sinks == 0 {
                    route_source_to_target_from_access_points(
                        &route_world,
                        source_port,
                        logical_source,
                        &source_positions,
                        sink,
                        &all_sinks,
                        config.strategy,
                    )
                } else {
                    route_to_target_from_powered_network(
                        route_world,
                        &route_sources,
                        sink,
                        &all_sinks,
                        config.strategy,
                    )
                };
                let (route, next_world) = match route_result {
                    Ok(route) => route,
                    Err(failure) => {
                        last_error = Some(eyre::eyre!(
                            "failed to route {}.{} -> {}.{} at {:?}: {failure:?}",
                            source_key.0,
                            source_key.1,
                            var.target.0,
                            var.target.1,
                            sink.position
                        ));
                        continue;
                    }
                };
                if let Some(reason) =
                    eager_route_failure_reason(config.validation, route_world, &next_world, &route)
                {
                    last_error = Some(eyre::eyre!(
                        "routed {}.{} -> {}.{} at {:?}, but route contract failed: {}",
                        source_key.0,
                        source_key.1,
                        var.target.0,
                        var.target.1,
                        sink.position,
                        reason,
                    ));
                    continue;
                }
                selected_route = Some((sink_index, var, sink, route, next_world));
                break;
            }

            let Some((sink_index, var, sink, route, next_world)) = selected_route else {
                return Err(last_error.unwrap_or_else(|| {
                    eyre::eyre!(
                        "failed to route {}.{} after {}/{} sink(s)",
                        source_key.0,
                        source_key.1,
                        routed_sinks,
                        total_sinks
                    )
                }));
            };

            let route = route.with_labels(
                format!("{}.{}", source_key.0, source_key.1),
                format!("{}.{}", var.target.0, var.target.1),
            );
            progress.detail(format!(
                "routed `{}.{}` -> `{}.{}` from {:?} to {:?} with {} block(s)",
                source_key.0,
                source_key.1,
                var.target.0,
                var.target.1,
                route.source,
                route.sink,
                route.blocks.len()
            ));
            for source in route_branch_sources(&next_world, &route, sink.position) {
                if route_source_set.insert(source.position) {
                    route_sources.push(source);
                }
            }
            sink_targets.remove(sink_index);
            prune_powered_route_sources(&mut route_sources, &mut route_source_set, &all_sinks, 0);
            *route_world = next_world;
            routes.push(route);
            routed_sinks += 1;
        }
    }

    Ok(())
}

fn group_vars_by_source_ordered<'a>(
    vars: &[&'a RoutingConnection],
) -> Vec<Vec<&'a RoutingConnection>> {
    let mut groups = Vec::<Vec<&RoutingConnection>>::new();
    for &var in vars {
        if groups
            .last()
            .and_then(|group| group.first())
            .is_some_and(|previous| previous.source == var.source)
        {
            groups.last_mut().expect("checked last group").push(var);
            continue;
        }

        groups.push(vec![var]);
    }

    groups
}

fn route_top_input_ports(
    top_inputs: &[RoutingTopInput],
    candidates: &[LayoutCandidate],
    placed_modules: &[PlacedModule],
    config: &GlobalRoutingConfig,
    progress: &GlobalPnrProgress,
    completed_inputs: &HashSet<String>,
    route_world: &mut World3D,
    routes: &mut Vec<RoutedNet>,
) -> eyre::Result<()> {
    let mut top_input_index = 0;
    for (port_index, port) in top_inputs.iter().enumerate() {
        let sinks = port
            .targets
            .iter()
            .flat_map(|(module, port)| {
                resolve_port_targets(candidates, placed_modules, module, port)
            })
            .collect::<Vec<_>>();
        if sinks.is_empty() {
            progress.item(
                port_index + 1,
                top_inputs.len(),
                format!("skip top input `{}` with no placed sinks", port.name),
            );
            continue;
        }

        let input_index = top_input_index;
        top_input_index += 1;
        if completed_inputs.contains(port.name.as_str()) {
            continue;
        }

        let input_sources = external_input_sources(route_world, input_index, &sinks);
        if input_sources.is_empty() {
            return Err(eyre::eyre!(
                "failed to place top-level input switch `{}`",
                port.name
            ));
        }

        let mut routed = false;
        let mut last_error = None;
        for input_source in input_sources {
            let mut candidate_world = route_world.clone();
            let mut candidate_routes = routes.clone();
            match route_top_input_fanout(
                port.name.as_str(),
                port_index,
                top_inputs.len(),
                input_source,
                sinks.clone(),
                config,
                progress,
                &mut candidate_world,
                &mut candidate_routes,
            ) {
                Ok(()) => {
                    *route_world = candidate_world;
                    *routes = candidate_routes;
                    routed = true;
                    break;
                }
                Err(error) => {
                    last_error = Some(error);
                }
            }
        }

        if !routed {
            let Some(error) = last_error else {
                return Err(eyre::eyre!(
                    "failed to route top-level input `{}` with no source candidates",
                    port.name
                ));
            };
            return Err(error);
        }
    }

    Ok(())
}

fn route_top_input_fanout(
    port_name: &str,
    port_index: usize,
    total_ports: usize,
    input_source: ExternalInputSource,
    mut sinks: Vec<ResolvedPortTarget>,
    config: &GlobalRoutingConfig,
    progress: &GlobalPnrProgress,
    route_world: &mut World3D,
    routes: &mut Vec<RoutedNet>,
) -> eyre::Result<()> {
    let total_sinks = sinks.len();
    *route_world = input_source.world.clone();
    routes.push(
        RoutedNet::new(
            input_source.switch,
            input_source.switch,
            input_source.blocks.clone(),
            vec![input_source.route_source],
        )
        .with_labels(port_name, format!("{port_name}.switch")),
    );

    let mut route_sources = vec![PoweredRouteSource {
        position: input_source.route_source,
        strength: MAX_REDSTONE_STRENGTH,
    }];
    let mut route_source_set = HashSet::from([input_source.route_source]);

    let all_sinks = sinks.clone();
    let mut routed_sinks = 0usize;
    while !sinks.is_empty() {
        sort_top_input_sinks_by_current_tree(&mut sinks, &route_sources);
        let mut selected_route = None;
        let mut last_error = None;

        for (sink_index, sink) in sinks.iter().copied().enumerate() {
            progress.item(
                port_index + 1,
                total_ports,
                format!(
                    "route top input `{}` sink {}/{}",
                    port_name,
                    routed_sinks + 1,
                    total_sinks
                ),
            );
            let route_result = route_top_input_to_target_from_network(
                route_world,
                input_source.switch,
                &route_sources,
                sink,
                &all_sinks,
                config.strategy,
            );
            let (mut route, next_world) = match route_result {
                Ok(route) => route,
                Err(failure) => {
                    last_error = Some(eyre::eyre!(
                        "failed to route top-level input {} -> {:?}: {failure:?}",
                        port_name,
                        sink.position
                    ));
                    continue;
                }
            };
            route.source = input_source.switch;
            if let Some(reason) =
                eager_route_failure_reason(config.validation, route_world, &next_world, &route)
            {
                last_error = Some(eyre::eyre!(
                    "routed top-level input {} -> {:?}, but route contract failed: {}",
                    port_name,
                    sink.position,
                    reason,
                ));
                continue;
            }
            selected_route = Some((sink_index, sink, route, next_world));
            break;
        }

        let Some((sink_index, sink, route, next_world)) = selected_route else {
            return Err(last_error.unwrap_or_else(|| {
                eyre::eyre!(
                    "failed to route top-level input {} after {}/{} sink(s)",
                    port_name,
                    routed_sinks,
                    total_sinks
                )
            }));
        };

        for source in route_branch_sources(&next_world, &route, sink.position) {
            if route_source_set.insert(source.position) {
                route_sources.push(source);
            }
        }
        sinks.remove(sink_index);
        prune_powered_route_sources(&mut route_sources, &mut route_source_set, &sinks, 0);
        *route_world = next_world;
        routes.push(route.with_labels(port_name, format!("{port_name}.sink")));
        routed_sinks += 1;
    }

    Ok(())
}

fn sort_top_input_sinks_by_current_tree(
    sinks: &mut [ResolvedPortTarget],
    route_sources: &[PoweredRouteSource],
) {
    // Top-level fanout은 고정 순서보다 현재 route tree에서 가까운 sink를
    // 우선 시도하는 쪽이 안정적이다. 가까운 후보가 실패하면 같은 단계에서
    // 나머지 sink도 모두 시도하므로, 순서는 탐색 우선순위일 뿐이다.
    sinks.sort_by_key(|sink| {
        let nearest_source = route_sources
            .iter()
            .map(|source| source.position.manhattan_distance(&sink.position))
            .min()
            .unwrap_or(usize::MAX);
        (
            nearest_source,
            sink.position.0,
            sink.position.1,
            sink.position.2,
        )
    });
}

fn sort_internal_sink_targets_by_current_tree(
    sinks: &mut [(&RoutingConnection, ResolvedPortTarget)],
    route_sources: &[PoweredRouteSource],
) {
    sinks.sort_by_key(|(var, sink)| {
        let nearest_source = route_sources
            .iter()
            .map(|source| source.position.manhattan_distance(&sink.position))
            .min()
            .unwrap_or(usize::MAX);
        (
            register_next_target_bit_order(&var.target.0),
            std::cmp::Reverse(nearest_source),
            sink.position.0,
            sink.position.1,
            sink.position.2,
        )
    });
}

fn route_top_input_to_target_from_network(
    world: &World3D,
    logical_source: Position,
    sources: &[PoweredRouteSource],
    sink: ResolvedPortTarget,
    same_net_sinks: &[ResolvedPortTarget],
    strategy: GlobalRoutingStrategy,
) -> Result<(RoutedNet, World3D), RouteFailure> {
    let mut sources = sources.to_vec();
    sources.sort_by_key(|source| source.position.manhattan_distance(&sink.position));
    for source in sources {
        if let Ok(route) = route_logical_source_to_target_position_with_strength(
            world,
            logical_source,
            source.position,
            source.strength,
            sink,
            same_net_sinks,
            strategy,
        ) {
            return Ok(route);
        }
    }

    Err(RouteFailure::Unreachable {
        source: logical_source,
        sink: sink.position,
    })
}

fn route_variable_priority(
    var: &RoutingConnection,
) -> (usize, usize, usize, &str, &str, &str, &str) {
    let is_clock_route =
        var.source.1.contains("clk") || var.target.1.contains("clk") || var.target.1 == "en";
    let target_port_priority = if is_cross_bit_next_input_route(var) {
        0
    } else if is_next_to_master_data_route(var) {
        1
    } else if is_clock_inverter_to_master_enable_route(var) {
        2
    } else if is_master_to_slave_data_route(var) {
        3
    } else if var.target.0.ends_with("_next") {
        4
    } else {
        match (var.target.1.as_str(), is_clock_route) {
            ("d", _) => 5,
            ("en", true) => 5,
            (_, false) => 6,
            _ => 7,
        }
    };
    let feedback_order = register_next_target_bit_order(&var.target.0);
    let source_port_priority = if var.source.1.ends_with("_n") { 0 } else { 1 };
    (
        target_port_priority,
        feedback_order,
        source_port_priority,
        var.source.0.as_str(),
        var.source.1.as_str(),
        var.target.0.as_str(),
        var.target.1.as_str(),
    )
}

fn is_master_to_slave_data_route(var: &RoutingConnection) -> bool {
    var.source.0.ends_with("_master")
        && var.source.1 == "q"
        && var.target.0.ends_with("_slave")
        && var.target.1 == "d"
}

fn is_next_to_master_data_route(var: &RoutingConnection) -> bool {
    var.source.0.ends_with("_next")
        && var.source.1 == "d"
        && var.target.0.ends_with("_master")
        && var.target.1 == "d"
}

fn is_clock_inverter_to_master_enable_route(var: &RoutingConnection) -> bool {
    var.source.0.ends_with("_clk_inv")
        && var.source.1.ends_with("_n")
        && var.target.0.ends_with("_master")
        && var.target.1 == "en"
}

fn is_cross_bit_next_input_route(var: &RoutingConnection) -> bool {
    let Some(source_bit) = register_module_bit(&var.source.0, "_slave") else {
        return false;
    };
    let Some(target_bit) = register_module_bit(&var.target.0, "_next") else {
        return false;
    };
    target_bit > source_bit
}

fn register_next_target_bit_order(target_module: &str) -> usize {
    register_module_bit(target_module, "_next")
        .map(|bit| usize::MAX - bit)
        .unwrap_or(usize::MAX)
}

fn register_module_bit(module: &str, suffix: &str) -> Option<usize> {
    module
        .strip_suffix(suffix)
        .and_then(|bit_name| bit_name.rsplit_once('_'))
        .and_then(|(_, bit)| bit.parse::<usize>().ok())
}

fn route_source_to_target_position(
    world: &World3D,
    source_port: &PhysicalPort,
    logical_source: Position,
    route_source: Position,
    sink: ResolvedPortTarget,
    same_net_sinks: &[ResolvedPortTarget],
    strategy: GlobalRoutingStrategy,
) -> Result<(RoutedNet, World3D), RouteFailure> {
    if source_port.direction == PhysicalPortDirection::Output && source_port.requires_output_diode()
    {
        return route_isolated_output_to_target_position(
            world,
            logical_source,
            route_source,
            sink,
            same_net_sinks,
            strategy,
        );
    }
    if source_port.direction == PhysicalPortDirection::Input {
        let mut sources = redstone_network_positions(world, &[logical_source]);
        sources.push(route_source);
        return route_to_target_from_network(world, &sources, sink, same_net_sinks, strategy);
    }

    route_to_target_position(world, route_source, sink, same_net_sinks, strategy)
}

fn route_source_to_target_from_access_points(
    world: &World3D,
    source_port: &PhysicalPort,
    logical_source: Position,
    route_sources: &[Position],
    sink: ResolvedPortTarget,
    same_net_sinks: &[ResolvedPortTarget],
    strategy: GlobalRoutingStrategy,
) -> Result<(RoutedNet, World3D), RouteFailure> {
    let mut route_sources = route_sources.to_vec();
    route_sources.push(logical_source);
    route_sources.sort_by_key(|source| source.manhattan_distance(&sink.position));
    route_sources.dedup();
    let fallback_source = route_sources.first().copied().unwrap_or(logical_source);
    let mut route_sources = route_sources
        .into_iter()
        .take(FANOUT_ROUTE_SOURCE_LIMIT)
        .collect::<Vec<_>>();
    if !route_sources.contains(&logical_source) {
        route_sources.push(logical_source);
    }

    for route_source in route_sources {
        if !world.size.bound_on(route_source) || !is_route_terminal(world, route_source) {
            continue;
        }
        if let Ok((route, next_world)) = route_source_to_target_position(
            world,
            source_port,
            logical_source,
            route_source,
            sink,
            same_net_sinks,
            strategy,
        ) {
            if route_candidate_powers_sink(world, &next_world, &route, strategy) {
                return Ok((route, next_world));
            }
        }
    }

    Err(RouteFailure::Unreachable {
        source: fallback_source,
        sink: sink.position,
    })
}

fn route_to_target_position(
    world: &World3D,
    source: Position,
    sink: ResolvedPortTarget,
    same_net_sinks: &[ResolvedPortTarget],
    strategy: GlobalRoutingStrategy,
) -> Result<(RoutedNet, World3D), RouteFailure> {
    if sink.requires_input_diode {
        return route_to_redstone_input_through_repeater(
            world,
            source,
            sink.position,
            sink.input_repeater_delay,
            same_net_sinks,
            strategy,
        );
    }

    route_point_to_point_with_strategy_and_allowed_contacts(
        world,
        source,
        sink.position,
        strategy,
        same_net_contact_positions(same_net_sinks),
    )
}

fn route_logical_source_to_target_position_with_strength(
    world: &World3D,
    logical_source: Position,
    route_source: Position,
    route_source_strength: usize,
    sink: ResolvedPortTarget,
    same_net_sinks: &[ResolvedPortTarget],
    strategy: GlobalRoutingStrategy,
) -> Result<(RoutedNet, World3D), RouteFailure> {
    if sink.requires_input_diode {
        return route_to_redstone_input_through_repeater_from_route_source(
            world,
            logical_source,
            route_source,
            route_source_strength,
            sink.position,
            sink.input_repeater_delay,
            same_net_sinks,
            strategy,
        );
    }

    let (route, routed_world) =
        route_point_to_point_with_strategy_and_allowed_contacts_and_initial_strength(
            world,
            route_source,
            sink.position,
            strategy,
            same_net_contact_positions(same_net_sinks),
            route_source_strength,
        )?;
    let powered_taps = route.powered_route_sources();
    let route = RoutedNet::new(logical_source, sink.position, route.blocks, route.path)
        .with_powered_taps(powered_taps);
    if route_candidate_powers_sink(world, &routed_world, &route, strategy) {
        return Ok((route, routed_world));
    }

    Err(RouteFailure::Unreachable {
        source: logical_source,
        sink: sink.position,
    })
}

fn route_to_target_from_network(
    world: &World3D,
    sources: &[Position],
    sink: ResolvedPortTarget,
    same_net_sinks: &[ResolvedPortTarget],
    strategy: GlobalRoutingStrategy,
) -> Result<(RoutedNet, World3D), RouteFailure> {
    let mut sources = sources.to_vec();
    sources.sort_by_key(|source| source.manhattan_distance(&sink.position));
    let fallback_source = sources.first().copied().unwrap_or(sink.position);

    for source in sources.into_iter().take(FANOUT_ROUTE_SOURCE_LIMIT) {
        if !world.size.bound_on(source) || !is_route_terminal(world, source) {
            continue;
        }
        if let Ok((route, next_world)) =
            route_to_target_position(world, source, sink, same_net_sinks, strategy)
        {
            if route_candidate_powers_sink(world, &next_world, &route, strategy) {
                return Ok((route, next_world));
            }
        }
    }

    Err(RouteFailure::Unreachable {
        source: fallback_source,
        sink: sink.position,
    })
}

fn route_to_target_from_powered_network(
    world: &World3D,
    sources: &[PoweredRouteSource],
    sink: ResolvedPortTarget,
    same_net_sinks: &[ResolvedPortTarget],
    strategy: GlobalRoutingStrategy,
) -> Result<(RoutedNet, World3D), RouteFailure> {
    let mut sources = sources.to_vec();
    sources.sort_by_key(|source| source.position.manhattan_distance(&sink.position));
    let fallback_source = sources
        .first()
        .map(|source| source.position)
        .unwrap_or(sink.position);

    for source in sources.into_iter().take(FANOUT_ROUTE_SOURCE_LIMIT) {
        if !world.size.bound_on(source.position) || !is_route_terminal(world, source.position) {
            continue;
        }
        if let Ok((route, next_world)) = route_logical_source_to_target_position_with_strength(
            world,
            source.position,
            source.position,
            source.strength,
            sink,
            same_net_sinks,
            strategy,
        ) {
            if route_candidate_powers_sink(world, &next_world, &route, strategy) {
                return Ok((route, next_world));
            }
        }
    }

    Err(RouteFailure::Unreachable {
        source: fallback_source,
        sink: sink.position,
    })
}

fn same_net_contact_positions(sinks: &[ResolvedPortTarget]) -> Vec<Position> {
    sinks
        .iter()
        .flat_map(|sink| {
            let mut positions = vec![sink.position];
            positions.extend(sink.position.cardinal());
            positions.push(sink.position.up());
            positions
        })
        .collect()
}

fn route_branch_sources(
    world: &World3D,
    route: &RoutedNet,
    sink: Position,
) -> Vec<PoweredRouteSource> {
    let mut sources = route
        .powered_taps
        .iter()
        .copied()
        .filter_map(|(position, strength)| {
            (strength > 1 && world.size.bound_on(position) && is_route_terminal(world, position))
                .then_some(PoweredRouteSource { position, strength })
        })
        .collect::<Vec<_>>();
    sources.sort_by_key(|source| {
        (
            source.position.manhattan_distance(&sink),
            std::cmp::Reverse(source.strength),
            std::cmp::Reverse(source.position.0),
            source.position.1,
            source.position.2,
        )
    });
    sources.truncate(FANOUT_ROUTE_TERMINAL_LIMIT);
    sources
}

fn prune_powered_route_sources(
    route_sources: &mut Vec<PoweredRouteSource>,
    route_source_set: &mut HashSet<Position>,
    sinks: &[ResolvedPortTarget],
    next_sink_index: usize,
) {
    if route_sources.len() <= FANOUT_ROUTE_TERMINAL_LIMIT {
        return;
    }

    route_sources.sort_by_key(|source| {
        let nearest_remaining_sink = sinks
            .iter()
            .skip(next_sink_index)
            .map(|sink| source.position.manhattan_distance(&sink.position))
            .min()
            .unwrap_or(0);
        (
            nearest_remaining_sink,
            std::cmp::Reverse(source.strength),
            source.position.0,
            source.position.1,
            source.position.2,
        )
    });
    route_sources.truncate(FANOUT_ROUTE_TERMINAL_LIMIT);

    route_source_set.clear();
    route_source_set.extend(route_sources.iter().map(|source| source.position));
}

fn route_isolated_output_to_target_position(
    world: &World3D,
    logical_source: Position,
    route_source: Position,
    sink: ResolvedPortTarget,
    same_net_sinks: &[ResolvedPortTarget],
    strategy: GlobalRoutingStrategy,
) -> Result<(RoutedNet, World3D), RouteFailure> {
    let same_net_contacts = same_net_contact_positions(same_net_sinks);
    if sink.requires_input_diode {
        for adapter in redstone_input_repeater_adapters(
            world,
            sink.position,
            sink.input_repeater_delay,
            &same_net_contacts,
        ) {
            let Ok((route, routed_world)) = route_direct_output_to_point(
                &adapter.world,
                logical_source,
                route_source,
                adapter.driver,
                strategy,
                same_net_contacts.clone(),
            ) else {
                continue;
            };
            let driver_route = RoutedNet::new(
                logical_source,
                adapter.driver,
                added_route_blocks(world, &routed_world),
                route.path.clone(),
            );
            if !route_candidate_powers_sink(world, &routed_world, &driver_route, strategy) {
                continue;
            }
            debug_assert!(detailed_router::target_powers_position(
                &routed_world,
                adapter.repeater,
                adapter.target
            ));
            let powered_taps = route.powered_route_sources();
            let route = RoutedNet::new(
                logical_source,
                adapter.target,
                added_route_blocks(world, &routed_world),
                route.path,
            )
            .with_required_powered_positions(vec![adapter.driver, adapter.repeater, adapter.target])
            .with_required_released_positions(vec![adapter.driver, adapter.repeater])
            .with_powered_taps(powered_taps);
            if route_candidate_powers_sink(world, &routed_world, &route, strategy) {
                return Ok((route, routed_world));
            }
        }
    }

    route_isolated_output_to_point(
        world,
        logical_source,
        route_source,
        sink.position,
        strategy,
        same_net_contacts,
    )
}

fn route_to_redstone_input_through_repeater(
    world: &World3D,
    source: Position,
    sink: Position,
    input_repeater_delay: usize,
    same_net_sinks: &[ResolvedPortTarget],
    strategy: GlobalRoutingStrategy,
) -> Result<(RoutedNet, World3D), RouteFailure> {
    route_to_redstone_input_through_repeater_from_route_source(
        world,
        source,
        source,
        initial_signal_strength(world, source),
        sink,
        input_repeater_delay,
        same_net_sinks,
        strategy,
    )
}

fn route_to_redstone_input_through_repeater_from_route_source(
    world: &World3D,
    logical_source: Position,
    route_source: Position,
    route_source_strength: usize,
    sink: Position,
    input_repeater_delay: usize,
    same_net_sinks: &[ResolvedPortTarget],
    strategy: GlobalRoutingStrategy,
) -> Result<(RoutedNet, World3D), RouteFailure> {
    let same_net_contacts = same_net_contact_positions(same_net_sinks);
    for adapter in
        redstone_input_repeater_adapters(world, sink, input_repeater_delay, &same_net_contacts)
    {
        let Ok((route, routed_world)) =
            route_point_to_point_with_strategy_and_allowed_contacts_and_initial_strength(
                &adapter.world,
                route_source,
                adapter.driver,
                strategy,
                same_net_contacts.clone(),
                route_source_strength,
            )
        else {
            continue;
        };
        let driver_route = RoutedNet::new(
            logical_source,
            adapter.driver,
            added_route_blocks(world, &routed_world),
            route.path.clone(),
        );
        if !route_candidate_powers_sink(world, &routed_world, &driver_route, strategy) {
            continue;
        }
        debug_assert!(detailed_router::target_powers_position(
            &routed_world,
            adapter.repeater,
            adapter.target
        ));
        let powered_taps = route.powered_route_sources();
        let route = RoutedNet::new(
            logical_source,
            adapter.target,
            added_route_blocks(world, &routed_world),
            route.path,
        )
        .with_required_powered_positions(vec![adapter.driver, adapter.repeater, adapter.target])
        .with_required_released_positions(vec![adapter.driver, adapter.repeater])
        .with_powered_taps(powered_taps);
        if route_candidate_powers_sink(world, &routed_world, &route, strategy) {
            return Ok((route, routed_world));
        }
    }

    Err(RouteFailure::Unreachable {
        source: logical_source,
        sink,
    })
}

fn route_direct_output_to_point(
    world: &World3D,
    logical_source: Position,
    route_source: Position,
    sink: Position,
    strategy: GlobalRoutingStrategy,
    additional_allowed_contacts: Vec<Position>,
) -> Result<(RoutedNet, World3D), RouteFailure> {
    let source_node = PlacedNode::new(route_source, world[route_source]);
    let initial = RouteSearchState {
        world: world.clone(),
        terminal: route_source,
        route: vec![route_source],
        signal_strength: 2,
        powered_taps: vec![PoweredRouteSource {
            position: route_source,
            strength: 2,
        }],
        pending_bounds: Some(sorted_route_bounds(
            source_node.propagation_bound(Some(world)),
            world,
            sink,
        )),
    };
    if let Ok((route, routed_world)) = route_point_to_point_from_initial_state(
        world,
        logical_source,
        sink,
        initial,
        strategy,
        &additional_allowed_contacts,
    ) {
        if route_candidate_powers_sink(world, &routed_world, &route, strategy) {
            return Ok((route, routed_world));
        }
    }

    for (tap, signal_strength) in routeable_output_taps(world, route_source, sink) {
        let initial = RouteSearchState {
            world: world.clone(),
            terminal: tap,
            route: vec![tap],
            signal_strength,
            powered_taps: vec![PoweredRouteSource {
                position: tap,
                strength: signal_strength,
            }],
            pending_bounds: None,
        };
        let Ok((route, routed_world)) = route_point_to_point_from_initial_state(
            world,
            logical_source,
            sink,
            initial,
            strategy,
            &additional_allowed_contacts,
        ) else {
            continue;
        };
        let powered_taps = route.powered_route_sources();
        let route = RoutedNet::new(
            logical_source,
            sink,
            added_route_blocks(world, &routed_world),
            route.path,
        )
        .with_powered_taps(powered_taps);
        if route_candidate_powers_sink(world, &routed_world, &route, strategy) {
            return Ok((route, routed_world));
        }
    }

    Err(RouteFailure::Unreachable {
        source: logical_source,
        sink,
    })
}

fn redstone_input_repeater_adapters(
    world: &World3D,
    sink: Position,
    input_repeater_delay: usize,
    additional_allowed_contacts: &[Position],
) -> Vec<InputDiodeAdapter> {
    sink.cardinal()
        .into_iter()
        .filter_map(|repeater_position| {
            let direction = repeater_position.diff(sink).inverse();
            input_repeater_adapter_world(
                world,
                sink,
                repeater_position,
                direction,
                input_repeater_delay,
                additional_allowed_contacts,
            )
        })
        .collect()
}

fn input_repeater_adapter_world(
    world: &World3D,
    sink: Position,
    repeater_position: Position,
    direction: Direction,
    input_repeater_delay: usize,
    additional_allowed_contacts: &[Position],
) -> Option<InputDiodeAdapter> {
    if !world.size.bound_on(repeater_position) || !world[repeater_position].kind.is_air() {
        return None;
    }
    let driver_position = repeater_position.walk(direction)?;
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

    let mut repeater = PlacedNode::new_repeater(repeater_position, direction);
    if let BlockKind::Repeater { delay, .. } = &mut repeater.block.kind {
        *delay = input_repeater_delay.clamp(1, 4);
    }
    if repeater.has_conflict(&adapter_world, &[sink].into_iter().collect()) {
        return None;
    }
    detailed_router::place_node(&mut adapter_world, repeater);
    if adapter_touches_forbidden_existing_signal(
        world,
        &adapter_world,
        repeater_position,
        &adapter_allowed_contacts(additional_allowed_contacts, &[sink, repeater_position]),
    ) {
        return None;
    }
    let driver = PlacedNode::new_redstone(driver_position);
    if driver.has_conflict(&adapter_world, &[repeater_position].into_iter().collect()) {
        return None;
    }
    detailed_router::place_node(&mut adapter_world, driver);
    if adapter_touches_forbidden_existing_signal(
        world,
        &adapter_world,
        driver_position,
        &adapter_allowed_contacts(
            additional_allowed_contacts,
            &[sink, repeater_position, driver_position],
        ),
    ) {
        return None;
    }
    if !detailed_router::target_powers_position(&adapter_world, driver_position, repeater_position)
    {
        return None;
    }
    detailed_router::target_powers_position(&adapter_world, repeater_position, sink).then_some(
        InputDiodeAdapter {
            world: adapter_world,
            driver: driver_position,
            repeater: repeater_position,
            target: sink,
        },
    )
}

pub fn collect_topology_output_endpoints(
    topology: &ResolvedPnrTopology,
    candidates: &[LayoutCandidate],
    placed_modules: &[PlacedModule],
) -> Vec<OutputEndpoint> {
    topology
        .nets
        .iter()
        .flat_map(|net| {
            net.sinks.iter().filter_map(|sink| {
                let ResolvedEndpoint::TopPort { port } = sink else {
                    return None;
                };
                let output = topology.port(*port)?;
                let ResolvedEndpoint::InstancePort {
                    instance,
                    port: source_port,
                } = &net.driver
                else {
                    return None;
                };
                let instance = topology.instances.get(instance.0)?;
                let source_port = topology.port(*source_port)?;
                let position = resolve_observable_port_position(
                    candidates,
                    placed_modules,
                    &instance.display_name,
                    &source_port.name,
                )?;
                Some(OutputEndpoint::new(output.name.clone(), position))
            })
        })
        .collect()
}

pub fn collect_topology_input_endpoints(
    topology: &ResolvedPnrTopology,
    routed_nets: &[RoutedNet],
) -> Vec<OutputEndpoint> {
    routed_nets
        .iter()
        .filter(|route| {
            route.source == route.sink
                && route
                    .blocks
                    .first()
                    .is_some_and(|(_, block)| block.kind.is_switch())
        })
        .filter_map(|route| {
            let ResolvedEndpoint::TopPort { port } = route.source_endpoint.as_ref()? else {
                return None;
            };
            Some(OutputEndpoint::new(
                topology.port(*port)?.name.clone(),
                route.source,
            ))
        })
        .collect()
}

fn external_input_sources(
    world: &World3D,
    index: usize,
    sinks: &[ResolvedPortTarget],
) -> Vec<ExternalInputSource> {
    let mut candidates = external_switch_candidates_outside_layout(world, sinks);
    candidates.extend(external_switch_candidates_for_sinks(sinks));
    candidates.sort_by_key(|position| external_input_candidate_cost(*position, sinks));
    candidates.dedup();

    let mut selected = Vec::new();
    selected.extend(candidates.iter().copied().take(8));
    for sink in sinks {
        let mut near_sink = candidates.clone();
        near_sink.sort_by_key(|position| {
            (
                position.manhattan_distance(&sink.position),
                external_input_candidate_cost(*position, sinks),
            )
        });
        selected.extend(near_sink.into_iter().take(4));
    }

    let max_x = world
        .iter_block()
        .into_iter()
        .map(|(position, _)| position.0)
        .max()
        .unwrap_or(0);
    let position = Position(max_x + 2, index * 3 + 1, 1);
    selected.push(position);
    selected.sort_by_key(|position| external_input_candidate_cost(*position, sinks));
    selected.dedup();

    selected
        .into_iter()
        .filter_map(|position| build_external_input_source(world, position))
        .take(24)
        .collect()
}

fn external_input_candidate_cost(
    position: Position,
    sinks: &[ResolvedPortTarget],
) -> (usize, usize, usize, usize) {
    let total_distance = sinks
        .iter()
        .map(|sink| position.manhattan_distance(&sink.position))
        .sum::<usize>();
    let max_distance = sinks
        .iter()
        .map(|sink| position.manhattan_distance(&sink.position))
        .max()
        .unwrap_or(0);
    (total_distance, max_distance, position.0, position.1)
}

fn build_external_input_source(world: &World3D, switch: Position) -> Option<ExternalInputSource> {
    let switch_block = input_switch_block();
    let route_source = switch.walk(switch_block.direction)?;
    let route_source_support = route_source.down()?;
    if !world.size.bound_on(switch)
        || !world.size.bound_on(route_source)
        || !world.size.bound_on(route_source_support)
        || !world[switch].kind.is_air()
        || !world[route_source].kind.is_air()
    {
        return None;
    }

    let mut source_world = world.clone();
    source_world[switch] = switch_block;
    place_support_cobble_if_needed(&mut source_world, route_source_support)?;

    let redstone_node = PlacedNode::new_redstone(route_source);
    if redstone_node.has_conflict(&source_world, &[switch].into_iter().collect()) {
        return None;
    }
    detailed_router::place_node(&mut source_world, redstone_node);
    if !detailed_router::target_powers_position(&source_world, switch, route_source) {
        return None;
    }

    let mut blocks = vec![(switch, switch_block)];
    if world[route_source_support].kind.is_air() {
        blocks.push((route_source_support, source_world[route_source_support]));
    }
    blocks.push((route_source, source_world[route_source]));

    Some(ExternalInputSource {
        world: source_world,
        switch,
        route_source,
        blocks,
    })
}

fn external_switch_candidates_outside_layout(
    world: &World3D,
    sinks: &[ResolvedPortTarget],
) -> Vec<Position> {
    let Some(center) = sink_center_position(sinks) else {
        return Vec::new();
    };
    let Some((min, max)) = occupied_bounds(world) else {
        return Vec::new();
    };

    let mut candidates = Vec::new();
    for distance in 2..=12 {
        candidates.push(Position(max.0 + distance, center.1, center.2));
        candidates.push(Position(center.0, max.1 + distance, center.2));
        if let Some(x) = min.0.checked_sub(distance) {
            candidates.push(Position(x, center.1, center.2));
        }
        if let Some(y) = min.1.checked_sub(distance) {
            candidates.push(Position(center.0, y, center.2));
        }
    }

    candidates.sort_by_key(|position| {
        let total_distance = sinks
            .iter()
            .map(|sink| position.manhattan_distance(&sink.position))
            .sum::<usize>();
        let x_outside = position.0 > max.0 || position.0 < min.0;
        (
            usize::from(!x_outside),
            total_distance,
            position.0,
            position.1,
            position.2,
        )
    });
    candidates
}

fn occupied_bounds(world: &World3D) -> Option<(Position, Position)> {
    let mut blocks = world
        .iter_block()
        .into_iter()
        .filter(|(_, block)| !block.kind.is_air())
        .map(|(position, _)| position);
    let first = blocks.next()?;
    let mut min = first;
    let mut max = first;
    for position in blocks {
        min.0 = min.0.min(position.0);
        min.1 = min.1.min(position.1);
        min.2 = min.2.min(position.2);
        max.0 = max.0.max(position.0);
        max.1 = max.1.max(position.1);
        max.2 = max.2.max(position.2);
    }
    Some((min, max))
}

fn external_switch_candidates_for_sinks(sinks: &[ResolvedPortTarget]) -> Vec<Position> {
    let mut candidates = sinks
        .iter()
        .flat_map(|sink| external_switch_candidates_near_sink(sink.position))
        .collect::<Vec<_>>();

    if let Some(center) = sink_center_position(sinks) {
        candidates.extend(external_switch_candidates_near_sink(center));
    }

    candidates.sort_by_key(|position| {
        let max_distance = sinks
            .iter()
            .map(|sink| position.manhattan_distance(&sink.position))
            .max()
            .unwrap_or(0);
        let total_distance = sinks
            .iter()
            .map(|sink| position.manhattan_distance(&sink.position))
            .sum::<usize>();
        let edge_penalty = usize::from(position.0 < 2 || position.1 < 2) * 16;
        (
            max_distance + edge_penalty,
            total_distance + edge_penalty,
            std::cmp::Reverse(position.0),
            std::cmp::Reverse(position.1),
            position.2,
        )
    });
    candidates.dedup();
    candidates
}

fn sink_center_position(sinks: &[ResolvedPortTarget]) -> Option<Position> {
    let first = sinks.first()?;
    let mut min = first.position;
    let mut max = first.position;
    for sink in sinks {
        min.0 = min.0.min(sink.position.0);
        min.1 = min.1.min(sink.position.1);
        min.2 = min.2.min(sink.position.2);
        max.0 = max.0.max(sink.position.0);
        max.1 = max.1.max(sink.position.1);
        max.2 = max.2.max(sink.position.2);
    }
    Some(Position(
        (min.0 + max.0) / 2,
        (min.1 + max.1) / 2,
        (min.2 + max.2) / 2,
    ))
}

fn external_switch_candidates_near_sink(sink: Position) -> Vec<Position> {
    let mut candidates = Vec::new();
    for distance in 4..=8 {
        candidates.extend([
            Position(sink.0 + distance, sink.1, sink.2),
            Position(sink.0, sink.1 + distance, sink.2),
        ]);
        if let Some(x) = sink.0.checked_sub(distance) {
            candidates.push(Position(x, sink.1, sink.2));
        }
        if let Some(y) = sink.1.checked_sub(distance) {
            candidates.push(Position(sink.0, y, sink.2));
        }
    }
    candidates
}

fn input_switch_block() -> Block {
    Block {
        kind: BlockKind::Switch { is_on: false },
        direction: Direction::West,
    }
}

fn resolve_port_targets(
    candidates: &[LayoutCandidate],
    placed_modules: &[PlacedModule],
    module_name: &str,
    port_name: &str,
) -> Vec<ResolvedPortTarget> {
    let Some((port, candidate, placed)) =
        resolve_port(candidates, placed_modules, module_name, port_name)
    else {
        return Vec::new();
    };

    let position = port.primary_route_position();
    vec![ResolvedPortTarget {
        position: translate_candidate_position(position, candidate, placed),
        requires_input_diode: port.requires_input_diode(),
        input_repeater_delay: 1,
    }]
}

fn resolve_observable_port_position(
    candidates: &[LayoutCandidate],
    placed_modules: &[PlacedModule],
    module_name: &str,
    port_name: &str,
) -> Option<Position> {
    resolve_port(candidates, placed_modules, module_name, port_name).map(
        |(port, candidate, placed)| {
            let position = port
                .routing_access_positions()
                .into_iter()
                .next()
                .unwrap_or_else(|| observable_port_position(candidate, port.position));
            translate_candidate_position(position, candidate, placed)
        },
    )
}

fn observable_port_position(candidate: &LayoutCandidate, position: Position) -> Position {
    if !candidate.world.size.bound_on(position) || !candidate.world[position].kind.is_torch() {
        return position;
    }

    candidate
        .world
        .iter_block()
        .into_iter()
        .filter(|(tap, block)| {
            block.kind.is_redstone()
                && detailed_router::target_powers_position(&candidate.world, position, *tap)
        })
        .map(|(tap, _)| tap)
        .min_by_key(|tap| (position.manhattan_distance(tap), tap.0, tap.1, tap.2))
        .unwrap_or(position)
}

fn translate_port_route_position(
    port: &PhysicalPort,
    candidate: &LayoutCandidate,
    placed: &PlacedModule,
) -> Position {
    translate_candidate_position(port.primary_route_position(), candidate, placed)
}

fn translate_port_access_positions(
    port: &PhysicalPort,
    candidate: &LayoutCandidate,
    placed: &PlacedModule,
) -> Vec<Position> {
    let mut positions = port
        .routing_access_positions()
        .into_iter()
        .map(|position| translate_candidate_position(position, candidate, placed))
        .collect::<Vec<_>>();
    positions.sort();
    positions.dedup();
    positions
}

fn resolve_port<'a>(
    candidates: &'a [LayoutCandidate],
    placed_modules: &'a [PlacedModule],
    module_name: &str,
    port_name: &str,
) -> Option<(&'a PhysicalPort, &'a LayoutCandidate, &'a PlacedModule)> {
    let placed = placed_modules
        .iter()
        .find(|placed| placed.module_name == module_name)?;
    let candidate = candidates.get(placed.candidate_index)?;
    let port = candidate.ports.iter().find(|port| port.name == port_name)?;
    Some((port, candidate, placed))
}

fn placed_candidate_world(
    candidates: &[LayoutCandidate],
    placed_modules: &[PlacedModule],
) -> eyre::Result<World3D> {
    let blocks = translated_candidate_blocks(candidates, placed_modules)?;
    let mut world = World3D::new(route_world_size(&blocks));
    for (position, block) in blocks {
        if !world[position].kind.is_air() {
            eyre::bail!("global route base collision at {position:?}");
        }
        world[position] = block;
    }
    world.initialize_redstone_states();
    Ok(world)
}

fn translated_candidate_blocks(
    candidates: &[LayoutCandidate],
    placed_modules: &[PlacedModule],
) -> eyre::Result<Vec<(Position, Block)>> {
    let mut blocks = Vec::new();
    for placed in placed_modules {
        let candidate = candidates
            .get(placed.candidate_index)
            .with_context(|| format!("missing candidate {}", placed.candidate_index))?;
        blocks.extend(
            candidate
                .world
                .iter_block()
                .into_iter()
                .map(|(position, block)| {
                    (
                        translate_candidate_position(position, candidate, placed),
                        block,
                    )
                }),
        );
    }
    Ok(blocks)
}

fn route_world_size(blocks: &[(Position, Block)]) -> DimSize {
    let mut max = Position(0, 0, 0);
    for (position, _) in blocks {
        max.0 = max.0.max(position.0);
        max.1 = max.1.max(position.1);
        max.2 = max.2.max(position.2);
    }
    DimSize(
        max.0 + GLOBAL_ROUTE_PADDING + 1,
        max.1 + GLOBAL_ROUTE_PADDING + 1,
        max.2 + GLOBAL_ROUTE_PADDING + 1,
    )
}

fn translate_candidate_position(
    position: Position,
    candidate: &LayoutCandidate,
    placed: &PlacedModule,
) -> Position {
    Position(
        placed.origin.0 + position.0 - candidate.bbox.min.0,
        placed.origin.1 + position.1 - candidate.bbox.min.1,
        placed.origin.2 + position.2 - candidate.bbox.min.2,
    )
}

fn route_isolated_output_to_point(
    world: &World3D,
    logical_source: Position,
    route_source: Position,
    sink: Position,
    strategy: GlobalRoutingStrategy,
    additional_allowed_contacts: Vec<Position>,
) -> Result<(RoutedNet, World3D), RouteFailure> {
    let initial_states = isolated_output_repeater_initial_states(
        world,
        logical_source,
        route_source,
        sink,
        &additional_allowed_contacts,
    );
    for initial in initial_states {
        let Ok((route, routed_world)) = route_point_to_point_from_initial_state(
            world,
            logical_source,
            sink,
            initial,
            strategy,
            &additional_allowed_contacts,
        ) else {
            continue;
        };
        if route_candidate_powers_sink(world, &routed_world, &route, strategy) {
            return Ok((route, routed_world));
        }
    }

    Err(RouteFailure::Unreachable {
        source: logical_source,
        sink,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ir::{NetClass, RoutablePortDirection};
    use crate::transform::place_and_route::detailed_router::PlaceRepeaterResult;
    use crate::transform::place_and_route::global_pnr::ir::{
        LayoutCandidate, PhysicalPort, PhysicalPortDirection, PortConnection,
    };
    use crate::transform::place_and_route::global_pnr::placer::{
        place_candidates_on_shelves, GlobalPlacementConfig, PlacedModule,
    };
    use crate::transform::place_and_route::global_pnr::route_engine::validation::{
        active_route_powers_sink, can_validate_active_route_source, route_power_contract_holds,
        switch_route_releases_required_positions_when_off,
    };
    use crate::transform::place_and_route::global_pnr::topology::{
        DefinitionId, DefinitionKey, InstanceId, InstanceKey, NetKey, PortId, ResolvedDefinition,
        ResolvedInstance, ResolvedNet, ResolvedPort,
    };
    use crate::transform::place_and_route::place_bound::{PlaceBound, PropagateType};
    use crate::world::block::{BlockKind, Direction, RedstoneState};
    use crate::world::simulator::Simulator;
    use crate::world::World;

    fn candidate(
        module_name: &str,
        block_position: Position,
        port_name: &str,
        direction: PhysicalPortDirection,
    ) -> LayoutCandidate {
        let mut world = World3D::new(DimSize(2, 1, 2));
        world[block_position.down().unwrap()] = cobble_block();
        world[block_position] = redstone_block();
        world.initialize_redstone_states();
        LayoutCandidate::from_world(
            module_name.to_owned(),
            world,
            vec![PhysicalPort {
                name: port_name.to_owned(),
                direction,
                position: block_position,
                route_position: None,
                access_points: Vec::new(),
                connection: PortConnection::Direct,
            }],
        )
        .unwrap()
    }

    #[test]
    fn resolve_port_targets_uses_exposed_route_position() {
        let mut candidates = vec![candidate(
            "right",
            Position(0, 0, 1),
            "in",
            PhysicalPortDirection::Input,
        )];
        candidates[0].ports[0].route_position = Some(Position(1, 0, 1));
        let placed = vec![PlacedModule {
            module_name: "right".to_owned(),
            candidate_index: 0,
            origin: Position(10, 20, 0),
            bbox: candidates[0].bbox,
        }];

        let targets = resolve_port_targets(&candidates, &placed, "right", "in");

        assert_eq!(targets.len(), 1);
        assert_eq!(targets[0].position, Position(11, 20, 1));
    }

    fn route_test_world(source: Position, sink: Position) -> World3D {
        let mut world = World3D::new(DimSize(8, 4, 3));
        world[source.down().unwrap()] = cobble_block();
        world[source] = redstone_block();
        world[sink.down().unwrap()] = cobble_block();
        world[sink] = redstone_block();
        world.initialize_redstone_states();
        world
    }

    fn route_test_world_with_size(source: Position, sink: Position, size: DimSize) -> World3D {
        let mut world = World3D::new(size);
        world[source.down().unwrap()] = cobble_block();
        world[source] = redstone_block();
        world[sink.down().unwrap()] = cobble_block();
        world[sink] = redstone_block();
        world.initialize_redstone_states();
        world
    }

    fn route_test_world_with_switch_source(
        source: Position,
        sink: Position,
        size: DimSize,
    ) -> World3D {
        let mut world = World3D::new(size);
        world[source] = Block {
            kind: BlockKind::Switch { is_on: false },
            direction: Direction::West,
        };
        world[sink.down().unwrap()] = cobble_block();
        world[sink] = redstone_block();
        world.initialize_redstone_states();
        world
    }

    fn cobble_block() -> Block {
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

    fn silent_progress() -> GlobalPnrProgress {
        GlobalPnrProgress::new(false, "test")
    }

    #[test]
    fn route_point_to_point_places_support_cobble_under_redstone() {
        let source = Position(0, 1, 1);
        let sink = Position(3, 1, 1);
        let world = route_test_world(source, sink);
        let (route, _) = route_point_to_point(&world, source, sink).unwrap();

        assert!(route.blocks.iter().any(|(_, block)| block.kind.is_cobble()));
        assert!(route
            .blocks
            .iter()
            .any(|(_, block)| block.kind.is_redstone()));
    }

    #[test]
    fn route_point_to_point_avoids_blocked_route_position() {
        let source = Position(0, 1, 1);
        let sink = Position(3, 1, 1);
        let mut world = route_test_world(source, sink);
        world[Position(1, 1, 1)] = cobble_block();
        let (route, _) = route_point_to_point(&world, source, sink).unwrap();

        assert!(!route
            .blocks
            .iter()
            .any(|(position, _)| *position == Position(1, 1, 1)));
    }

    #[test]
    fn route_point_to_point_refreshes_long_redstone_with_repeater() {
        let source = Position(0, 1, 1);
        let sink = Position(22, 1, 1);
        let world = route_test_world_with_size(source, sink, DimSize(26, 4, 3));
        let (route, _) = route_point_to_point(&world, source, sink).unwrap();

        assert!(route
            .blocks
            .iter()
            .any(|(_, block)| block.kind.is_repeater()));
    }

    #[test]
    fn route_result_preserves_powered_tap_strengths_for_fanout() {
        let source = Position(0, 1, 1);
        let sink = Position(6, 1, 1);
        let world = route_test_world_with_switch_source(source, sink, DimSize(10, 4, 3));
        let (route, _) = route_point_to_point(&world, source, sink).unwrap();

        assert!(
            route.powered_taps.iter().any(
                |(position, strength)| *position != source && *strength < MAX_REDSTONE_STRENGTH
            ),
            "fanout branch candidates should keep the remaining signal strength from search"
        );
        assert!(route
            .powered_taps
            .iter()
            .all(|(_, strength)| *strength <= MAX_REDSTONE_STRENGTH));
    }

    #[test]
    fn input_diode_route_preserves_powered_taps_for_later_fanout() {
        let source = Position(0, 1, 1);
        let sink = Position(12, 1, 1);
        let world = route_test_world_with_switch_source(source, sink, DimSize(16, 4, 3));

        let (route, _) = route_to_target_position(
            &world,
            source,
            ResolvedPortTarget {
                position: sink,
                requires_input_diode: true,
                input_repeater_delay: 1,
            },
            &[],
            GlobalRoutingStrategy::AStar,
        )
        .unwrap();

        assert!(
            route.powered_taps.len() > 1,
            "diode adapter routes must keep intermediate powered taps for fanout"
        );
    }

    #[test]
    fn output_diode_route_preserves_powered_taps_for_later_fanout() {
        let source = Position(0, 1, 1);
        let sink = Position(12, 1, 1);
        let world = route_test_world_with_size(source, sink, DimSize(16, 4, 3));
        let source_port = PhysicalPort {
            name: "q".to_owned(),
            direction: PhysicalPortDirection::Output,
            position: source,
            route_position: None,
            access_points: Vec::new(),
            connection: PortConnection::OutputDiode,
        };

        let (route, _) = route_source_to_target_position(
            &world,
            &source_port,
            source,
            source,
            ResolvedPortTarget {
                position: sink,
                requires_input_diode: true,
                input_repeater_delay: 1,
            },
            &[],
            GlobalRoutingStrategy::AStar,
        )
        .unwrap();

        assert!(
            route.powered_taps.len() > 1,
            "output isolation must not discard the route taps used for fanout"
        );
    }

    #[test]
    fn output_diode_route_descends_from_high_latch_tap_to_low_input() {
        let source = Position(75, 16, 5);
        let sink = Position(43, 17, 1);
        let world = route_test_world_with_size(source, sink, DimSize(84, 24, 8));
        let source_port = PhysicalPort {
            name: "q".to_owned(),
            direction: PhysicalPortDirection::Output,
            position: source,
            route_position: None,
            access_points: Vec::new(),
            connection: PortConnection::OutputDiode,
        };

        route_source_to_target_position(
            &world,
            &source_port,
            source,
            source,
            ResolvedPortTarget {
                position: sink,
                requires_input_diode: false,
                input_repeater_delay: 1,
            },
            &[],
            GlobalRoutingStrategy::AStar,
        )
        .unwrap();
    }

    #[test]
    fn output_diode_route_descends_to_low_input_diode() {
        let source = Position(33, 16, 5);
        let sink = Position(48, 16, 2);
        let world = route_test_world_with_size(source, sink, DimSize(60, 24, 8));
        let source_port = PhysicalPort {
            name: "q".to_owned(),
            direction: PhysicalPortDirection::Output,
            position: source,
            route_position: None,
            access_points: Vec::new(),
            connection: PortConnection::OutputDiode,
        };

        route_source_to_target_position(
            &world,
            &source_port,
            source,
            source,
            ResolvedPortTarget {
                position: sink,
                requires_input_diode: true,
                input_repeater_delay: 1,
            },
            &[],
            GlobalRoutingStrategy::AStar,
        )
        .unwrap();
    }

    #[test]
    fn output_access_routing_falls_back_to_logical_source() {
        let source = Position(0, 1, 1);
        let sink = Position(5, 1, 1);
        let world = route_test_world_with_size(source, sink, DimSize(8, 4, 3));
        let source_port = PhysicalPort {
            name: "q".to_owned(),
            direction: PhysicalPortDirection::Output,
            position: source,
            route_position: None,
            access_points: vec![Position(99, 99, 99)],
            connection: PortConnection::OutputDiode,
        };

        route_source_to_target_from_access_points(
            &world,
            &source_port,
            source,
            &source_port.access_points,
            ResolvedPortTarget {
                position: sink,
                requires_input_diode: false,
                input_repeater_delay: 1,
            },
            &[],
            GlobalRoutingStrategy::AStar,
        )
        .unwrap();
    }

    #[test]
    fn top_input_fanout_prioritizes_sinks_near_current_route_tree() {
        let mut sinks = vec![
            ResolvedPortTarget {
                position: Position(2, 0, 1),
                requires_input_diode: false,
                input_repeater_delay: 1,
            },
            ResolvedPortTarget {
                position: Position(8, 0, 1),
                requires_input_diode: false,
                input_repeater_delay: 1,
            },
            ResolvedPortTarget {
                position: Position(5, 0, 1),
                requires_input_diode: false,
                input_repeater_delay: 1,
            },
        ];
        let route_sources = vec![PoweredRouteSource {
            position: Position(4, 0, 1),
            strength: MAX_REDSTONE_STRENGTH,
        }];

        sort_top_input_sinks_by_current_tree(&mut sinks, &route_sources);

        assert_eq!(
            sinks.iter().map(|sink| sink.position).collect::<Vec<_>>(),
            vec![Position(5, 0, 1), Position(2, 0, 1), Position(8, 0, 1)]
        );
    }

    #[test]
    fn route_point_to_point_does_not_assume_unpowered_redstone_has_full_strength() {
        let source = Position(0, 1, 1);
        let sink = Position(14, 1, 1);
        let world = route_test_world_with_size(source, sink, DimSize(18, 4, 3));
        let (route, _) = route_point_to_point(&world, source, sink).unwrap();

        assert!(
            route
                .blocks
                .iter()
                .any(|(_, block)| block.kind.is_repeater()),
            "unpowered redstone output ports need a repeater before a long downstream route"
        );
    }

    #[test]
    fn route_point_to_point_astar_handles_long_unpowered_redstone_route() {
        let source = Position(0, 1, 1);
        let sink = Position(44, 1, 1);
        let world = route_test_world_with_size(source, sink, DimSize(48, 4, 3));
        let (route, _) =
            route_point_to_point_with_strategy(&world, source, sink, GlobalRoutingStrategy::AStar)
                .unwrap();

        assert!(
            route
                .blocks
                .iter()
                .filter(|(_, block)| block.kind.is_repeater())
                .count()
                >= 2
        );
    }

    #[test]
    fn route_point_to_point_astar_handles_counter_carry_like_coordinates() {
        let source = Position(26, 10, 3);
        let sink = Position(70, 4, 3);
        let world = route_test_world_with_size(source, sink, DimSize(76, 16, 6));
        let (route, _) =
            route_point_to_point_with_strategy(&world, source, sink, GlobalRoutingStrategy::AStar)
                .unwrap();

        assert!(route
            .blocks
            .iter()
            .any(|(_, block)| block.kind.is_repeater()));
    }

    #[test]
    fn route_point_to_point_greedy_beam_handles_counter_carry_like_coordinates() {
        let source = Position(26, 10, 3);
        let sink = Position(70, 4, 3);
        let world = route_test_world_with_size(source, sink, DimSize(76, 16, 6));
        let (route, _) = route_point_to_point_with_strategy(
            &world,
            source,
            sink,
            GlobalRoutingStrategy::GreedyBeam {
                beam_width: 64,
                max_expansions: 1_024,
                variant_seed: 0,
            },
        )
        .unwrap();

        assert!(route
            .blocks
            .iter()
            .any(|(_, block)| block.kind.is_repeater()));
    }

    #[test]
    fn route_point_to_point_direct_greedy_handles_counter_carry_like_coordinates() {
        let source = Position(26, 10, 3);
        let sink = Position(70, 4, 3);
        let world = route_test_world_with_size(source, sink, DimSize(76, 16, 6));
        let (route, _) = route_point_to_point_with_strategy(
            &world,
            source,
            sink,
            GlobalRoutingStrategy::DirectGreedy { max_steps: 128 },
        )
        .unwrap();

        assert!(route
            .blocks
            .iter()
            .any(|(_, block)| block.kind.is_repeater()));
    }

    #[test]
    fn route_to_redstone_input_handles_counter_fanout_like_coordinates() {
        let source = Position(25, 10, 3);
        let sink = Position(41, 8, 3);
        let world = route_test_world_with_size(source, sink, DimSize(48, 16, 6));

        route_to_target_position(
            &world,
            source,
            ResolvedPortTarget {
                position: sink,
                requires_input_diode: true,
                input_repeater_delay: 1,
            },
            &[],
            GlobalRoutingStrategy::AStar,
        )
        .unwrap();
    }

    #[test]
    fn route_point_to_point_long_route_powers_sink_through_repeater() -> eyre::Result<()> {
        let source = Position(0, 1, 1);
        let sink = Position(22, 1, 1);
        let world = route_test_world_with_switch_source(source, sink, DimSize(26, 4, 3));
        let (_, routed_world) = route_point_to_point(&world, source, sink).unwrap();
        let world = World::from(&routed_world);
        let mut sim = Simulator::from_with_limits_and_trace(&world, 128, 20_000, 0)
            .map_err(|error| eyre::eyre!(error.message().to_owned()))?;

        sim.change_state_with_limits(vec![(source, true)], 128, 20_000)?;

        assert!(
            matches!(sim.world()[sink].kind, BlockKind::Redstone { strength, .. } if strength > 0),
            "sink redstone should be powered through the inserted repeater"
        );
        Ok(())
    }

    #[test]
    fn route_to_redstone_input_finishes_with_repeater_diode() -> eyre::Result<()> {
        let source = Position(10, 1, 1);
        let sink = Position(1, 1, 1);
        let world = route_test_world_with_switch_source(source, sink, DimSize(13, 4, 3));

        let (route, routed_world) = route_to_target_position(
            &world,
            source,
            ResolvedPortTarget {
                position: sink,
                requires_input_diode: true,
                input_repeater_delay: 1,
            },
            &[],
            GlobalRoutingStrategy::BreadthFirst,
        )
        .unwrap();

        assert!(
            route
                .blocks
                .iter()
                .any(|(_, block)| block.kind.is_repeater()),
            "redstone input routes should end through a repeater diode"
        );
        assert!(
            sink.cardinal()
                .into_iter()
                .any(|position| routed_world.size.bound_on(position)
                    && routed_world[position].kind.is_repeater()
                    && detailed_router::target_powers_position(&routed_world, position, sink)),
            "the final repeater should power the input redstone"
        );

        let world = World::from(&routed_world);
        let mut sim = Simulator::from_with_limits_and_trace(&world, 128, 20_000, 0)
            .map_err(|error| eyre::eyre!(error.message().to_owned()))?;
        sim.change_state_with_limits(vec![(source, true)], 128, 20_000)?;

        assert!(
            matches!(sim.world()[sink].kind, BlockKind::Redstone { strength, .. } if strength > 0),
            "input redstone should be powered through the repeater diode"
        );
        Ok(())
    }

    #[test]
    fn route_to_redstone_input_preserves_requested_repeater_delay() -> eyre::Result<()> {
        let source = Position(10, 1, 1);
        let sink = Position(1, 1, 1);
        let world = route_test_world_with_switch_source(source, sink, DimSize(13, 4, 3));

        let (_, routed_world) = route_to_target_position(
            &world,
            source,
            ResolvedPortTarget {
                position: sink,
                requires_input_diode: true,
                input_repeater_delay: 3,
            },
            &[],
            GlobalRoutingStrategy::BreadthFirst,
        )
        .unwrap();

        assert!(sink.cardinal().into_iter().any(|position| {
            matches!(
                routed_world[position].kind,
                BlockKind::Repeater { delay: 3, .. }
            )
        }));
        Ok(())
    }

    #[test]
    fn route_to_cobble_input_finishes_with_repeater_diode() -> eyre::Result<()> {
        let source = Position(10, 1, 1);
        let sink = Position(1, 1, 1);
        let mut world = World3D::new(DimSize(13, 4, 3));
        world[source] = Block {
            kind: BlockKind::Switch { is_on: false },
            direction: Direction::West,
        };
        world[sink] = cobble_block();
        world.initialize_redstone_states();

        let (route, routed_world) = route_to_target_position(
            &world,
            source,
            ResolvedPortTarget {
                position: sink,
                requires_input_diode: true,
                input_repeater_delay: 1,
            },
            &[],
            GlobalRoutingStrategy::BreadthFirst,
        )
        .unwrap();

        assert!(
            route
                .blocks
                .iter()
                .any(|(_, block)| block.kind.is_repeater()),
            "cobble input routes should end through a repeater diode"
        );
        assert!(
            sink.cardinal()
                .into_iter()
                .any(|position| routed_world.size.bound_on(position)
                    && routed_world[position].kind.is_repeater()
                    && detailed_router::target_powers_position(&routed_world, position, sink)),
            "the final repeater should power the input cobble"
        );

        let world = World::from(&routed_world);
        let mut sim = Simulator::from_with_limits_and_trace(&world, 128, 20_000, 0)
            .map_err(|error| eyre::eyre!(error.message().to_owned()))?;
        sim.change_state_with_limits(vec![(source, true)], 128, 20_000)?;

        assert!(
            matches!(sim.world()[sink].kind, BlockKind::Cobble { on_count, .. } if on_count > 0),
            "input cobble should be powered through the repeater diode"
        );
        Ok(())
    }

    #[test]
    fn route_point_to_point_does_not_touch_existing_signal_line() {
        let source = Position(0, 1, 1);
        let sink = Position(6, 1, 1);
        let protected = Position(3, 2, 1);
        let mut world = route_test_world_with_switch_source(source, sink, DimSize(9, 5, 3));
        world[protected.down().unwrap()] = cobble_block();
        world[protected] = redstone_block();
        world.initialize_redstone_states();

        let (_, routed_world) =
            route_point_to_point_with_strategy(&world, source, sink, GlobalRoutingStrategy::AStar)
                .unwrap();

        for (position, block) in routed_world.iter_block() {
            if !block.kind.is_redstone() && !block.kind.is_repeater() {
                continue;
            }
            if position == protected || position == source || position == sink {
                continue;
            }
            assert!(
                !detailed_router::target_powers_position(&routed_world, position, protected)
                    && !detailed_router::target_powers_position(
                        &routed_world,
                        protected,
                        position
                    ),
                "route block {position:?} should not electrically touch protected line {protected:?}"
            );
        }
    }

    #[test]
    fn route_rejects_repeater_floating_above_adjacent_redstone() {
        let prev = Position(17, 7, 4);
        let repeater = Position(18, 7, 5);
        let mut world = World3D::new(DimSize(20, 9, 7));
        world[prev.down().unwrap()] = cobble_block();
        world[prev] = redstone_block();
        world.initialize_redstone_states();

        let result = detailed_router::place_repeater_with_cobble(
            &world,
            PlaceBound(PropagateType::Soft, repeater, Direction::East),
            prev,
            Position(19, 7, 5),
            Direction::East,
            None,
        );

        assert!(matches!(
            result,
            PlaceRepeaterResult::Rejected(detailed_router::RouteRejectReason::DisconnectedRoute)
        ));
    }

    #[test]
    fn route_rejects_repeater_when_previous_redstone_hits_the_wrong_side() {
        let prev = Position(2, 1, 1);
        let repeater = Position(2, 2, 1);
        let mut world = World3D::new(DimSize(5, 5, 3));
        world[prev.down().unwrap()] = cobble_block();
        world[prev] = Block {
            kind: BlockKind::Redstone {
                on_count: 0,
                state: RedstoneState::North as usize,
                strength: 0,
            },
            direction: Direction::None,
        };

        let result = detailed_router::place_repeater_with_cobble(
            &world,
            PlaceBound(PropagateType::Soft, repeater, Direction::South),
            prev,
            Position(2, 3, 1),
            Direction::East,
            None,
        );

        assert!(matches!(
            result,
            PlaceRepeaterResult::Rejected(detailed_router::RouteRejectReason::DisconnectedRoute)
        ));
    }

    #[test]
    fn active_route_validation_checks_off_switch_after_turning_it_on() {
        let source = Position(0, 1, 1);
        let sink = Position(3, 1, 1);
        let before = route_test_world_with_switch_source(source, sink, DimSize(6, 4, 3));
        let mut after = before.clone();
        after[sink.down().unwrap()] = cobble_block();
        after[sink] = redstone_block();
        after.initialize_redstone_states();
        let route = RoutedNet::new(
            source,
            sink,
            vec![(sink, redstone_block())],
            vec![source, sink],
        );

        assert!(
            !active_route_powers_sink(&before, &after, &route),
            "an off top-level switch must still be validated in the active/on state"
        );
    }

    #[test]
    fn active_route_validation_rejects_route_that_turns_active_source_off() {
        let source = Position(0, 1, 1);
        let sink = Position(3, 1, 1);
        let mut before = route_test_world(source, sink);
        before[source] = Block {
            kind: BlockKind::Torch { is_on: true },
            direction: Direction::East,
        };
        let mut after = before.clone();
        after[source] = Block {
            kind: BlockKind::Torch { is_on: false },
            direction: Direction::East,
        };
        let route = RoutedNet::new(
            source,
            sink,
            vec![(sink, redstone_block())],
            vec![source, sink],
        );

        assert!(
            !active_route_powers_sink(&before, &after, &route),
            "a route that disables an already-active source must not be accepted"
        );
    }

    #[test]
    fn active_route_validation_rejects_unpowered_sink_from_active_source() {
        let source = Position(1, 1, 1);
        let sink = Position(4, 1, 1);
        let mut before = route_test_world_with_size(source, sink, DimSize(6, 3, 3));
        before[source] = Block {
            kind: BlockKind::Torch { is_on: true },
            direction: Direction::East,
        };
        before.initialize_redstone_states();
        let after = before.clone();
        let route = RoutedNet::new(source, sink, Vec::new(), vec![source, sink]);

        assert!(
            !active_route_powers_sink(&before, &after, &route),
            "an already-active source must power the routed sink"
        );
    }

    #[test]
    fn active_route_validation_can_check_redstone_sources() {
        let source = Position(1, 1, 1);
        let world = route_test_world_with_size(source, Position(4, 1, 1), DimSize(6, 3, 3));

        assert!(can_validate_active_route_source(&world, source));
    }

    #[test]
    fn active_route_validation_checks_required_powered_positions() {
        let source = Position(0, 1, 1);
        let driver = Position(1, 1, 1);
        let repeater = Position(2, 1, 1);
        let sink = Position(3, 1, 1);
        let mut before = World3D::new(DimSize(5, 3, 3));
        before[source] = Block {
            kind: BlockKind::Switch { is_on: true },
            direction: Direction::West,
        };
        before.initialize_redstone_states();
        let mut after = before.clone();
        after[sink] = Block {
            kind: BlockKind::Switch { is_on: true },
            direction: Direction::West,
        };
        after.initialize_redstone_states();
        let route = RoutedNet::new(
            source,
            sink,
            Vec::new(),
            vec![source, driver, repeater, sink],
        )
        .with_required_powered_positions(vec![driver, repeater, sink]);

        assert!(
            !active_route_powers_sink(&before, &after, &route),
            "isolated routes must validate their diode driver/repeater, not only the logical sink"
        );
    }

    #[test]
    fn route_power_contract_rejects_switch_route_that_stays_on_when_switch_is_off() {
        let source = Position(0, 1, 1);
        let sink = Position(2, 1, 1);
        let mut world = World3D::new(DimSize(4, 3, 3));
        world[source] = Block {
            kind: BlockKind::Switch { is_on: false },
            direction: Direction::West,
        };
        world[sink] = Block {
            kind: BlockKind::RedstoneBlock,
            direction: Direction::None,
        };
        world.initialize_redstone_states();
        let route = RoutedNet::new(source, sink, Vec::new(), vec![source, sink]);

        assert!(
            !route_power_contract_holds(&world, &world, &route),
            "top-level switch routes must not leave required positions powered while the switch is off"
        );
    }

    #[test]
    fn switch_release_contract_can_ignore_independently_powered_diode_target() {
        let source = Position(0, 1, 1);
        let driver = Position(1, 1, 1);
        let sink = Position(2, 1, 1);
        let mut world = World3D::new(DimSize(4, 3, 3));
        world[source] = Block {
            kind: BlockKind::Switch { is_on: false },
            direction: Direction::West,
        };
        world[sink] = Block {
            kind: BlockKind::RedstoneBlock,
            direction: Direction::None,
        };
        world.initialize_redstone_states();
        let route = RoutedNet::new(source, sink, Vec::new(), vec![source, driver, sink])
            .with_required_powered_positions(vec![driver, sink])
            .with_required_released_positions(vec![driver]);

        assert!(
            switch_route_releases_required_positions_when_off(&world, &route),
            "input diode routes only need route-owned driver positions to release; the child-side target may be powered independently"
        );
    }

    #[test]
    fn active_route_set_validation_rechecks_routes_against_latest_world() {
        let source = Position(1, 1, 1);
        let sink = Position(4, 1, 1);
        let mut latest_world = route_test_world_with_size(source, sink, DimSize(6, 3, 3));
        latest_world[source] = Block {
            kind: BlockKind::Torch { is_on: true },
            direction: Direction::East,
        };
        latest_world.initialize_redstone_states();
        let routes = vec![RoutedNet::new(source, sink, Vec::new(), vec![source, sink])];

        assert_eq!(
            first_invalid_active_route(&latest_world, &routes).map(|route| route.sink),
            Some(sink),
            "previously routed active nets must still power their sinks after later routing"
        );
    }

    #[test]
    fn route_priority_routes_high_bit_feedback_first() {
        let q0_feedback = RoutingConnection {
            source: ("q_0_slave".to_owned(), "q".to_owned()),
            target: ("q_0_next".to_owned(), "q_0".to_owned()),
        };
        let q1_feedback = RoutingConnection {
            source: ("q_1_slave".to_owned(), "q".to_owned()),
            target: ("q_1_next".to_owned(), "q_1".to_owned()),
        };

        assert!(route_variable_priority(&q1_feedback) < route_variable_priority(&q0_feedback));
    }

    #[test]
    fn route_priority_routes_cross_bit_next_input_before_own_bit_feedback() {
        let q0_to_own_bit = RoutingConnection {
            source: ("q_0_slave".to_owned(), "q".to_owned()),
            target: ("q_0_next".to_owned(), "q_0".to_owned()),
        };
        let q0_to_next_bit = RoutingConnection {
            source: ("q_0_slave".to_owned(), "q".to_owned()),
            target: ("q_1_next".to_owned(), "q_0".to_owned()),
        };

        assert!(route_variable_priority(&q0_to_next_bit) < route_variable_priority(&q0_to_own_bit));
    }

    #[test]
    fn route_priority_routes_cross_bit_next_input_before_next_to_master_output() {
        let fanin_to_next = RoutingConnection {
            source: ("q_0_slave".to_owned(), "q".to_owned()),
            target: ("q_1_next".to_owned(), "q_0".to_owned()),
        };
        let next_to_master = RoutingConnection {
            source: ("q_1_next".to_owned(), "d".to_owned()),
            target: ("q_1_master".to_owned(), "d".to_owned()),
        };

        assert!(route_variable_priority(&fanin_to_next) < route_variable_priority(&next_to_master));
    }

    #[test]
    fn route_priority_routes_next_to_master_output_before_self_feedback() {
        let self_feedback = RoutingConnection {
            source: ("q_1_slave".to_owned(), "q".to_owned()),
            target: ("q_1_next".to_owned(), "q_1".to_owned()),
        };
        let next_to_master = RoutingConnection {
            source: ("q_1_next".to_owned(), "d".to_owned()),
            target: ("q_1_master".to_owned(), "d".to_owned()),
        };

        assert!(route_variable_priority(&next_to_master) < route_variable_priority(&self_feedback));
    }

    #[test]
    fn route_priority_routes_master_to_slave_handoff_before_feedback() {
        let master_to_slave = RoutingConnection {
            source: ("q_0_master".to_owned(), "q".to_owned()),
            target: ("q_0_slave".to_owned(), "d".to_owned()),
        };
        let self_feedback = RoutingConnection {
            source: ("q_0_slave".to_owned(), "q".to_owned()),
            target: ("q_0_next".to_owned(), "q_0".to_owned()),
        };

        assert!(
            route_variable_priority(&master_to_slave) < route_variable_priority(&self_feedback)
        );
    }

    #[test]
    fn route_priority_routes_clock_inverter_enable_before_self_feedback() {
        let clock_to_master = RoutingConnection {
            source: ("q_0_clk_inv".to_owned(), "clk_n".to_owned()),
            target: ("q_0_master".to_owned(), "en".to_owned()),
        };
        let self_feedback = RoutingConnection {
            source: ("q_0_slave".to_owned(), "q".to_owned()),
            target: ("q_0_next".to_owned(), "q_0".to_owned()),
        };

        assert!(
            route_variable_priority(&clock_to_master) < route_variable_priority(&self_feedback)
        );
    }

    #[test]
    fn route_priority_routes_cross_bit_next_input_before_clock_enable() {
        let cross_bit = RoutingConnection {
            source: ("q_0_slave".to_owned(), "q".to_owned()),
            target: ("q_1_next".to_owned(), "q_0".to_owned()),
        };
        let clock_to_master = RoutingConnection {
            source: ("q_0_clk_inv".to_owned(), "clk_n".to_owned()),
            target: ("q_0_master".to_owned(), "en".to_owned()),
        };

        assert!(route_variable_priority(&cross_bit) < route_variable_priority(&clock_to_master));
    }

    #[test]
    fn route_priority_routes_cross_bit_next_input_before_master_to_slave_handoff() {
        let cross_bit = RoutingConnection {
            source: ("q_0_slave".to_owned(), "q".to_owned()),
            target: ("q_1_next".to_owned(), "q_0".to_owned()),
        };
        let master_to_slave = RoutingConnection {
            source: ("q_0_master".to_owned(), "q".to_owned()),
            target: ("q_0_slave".to_owned(), "d".to_owned()),
        };

        assert!(route_variable_priority(&cross_bit) < route_variable_priority(&master_to_slave));
    }

    #[test]
    fn route_priority_routes_next_to_master_data_before_clock_enable() {
        let next_to_master = RoutingConnection {
            source: ("q_1_next".to_owned(), "d".to_owned()),
            target: ("q_1_master".to_owned(), "d".to_owned()),
        };
        let clock_to_master = RoutingConnection {
            source: ("q_1_clk_inv".to_owned(), "clk_n".to_owned()),
            target: ("q_1_master".to_owned(), "en".to_owned()),
        };

        assert!(
            route_variable_priority(&next_to_master) < route_variable_priority(&clock_to_master)
        );
    }

    #[test]
    fn internal_fanout_routes_far_sink_before_near_sink() {
        let source = PoweredRouteSource {
            position: Position(0, 1, 0),
            strength: MAX_REDSTONE_STRENGTH,
        };
        let far_var = RoutingConnection {
            source: ("q_0_slave".to_owned(), "q".to_owned()),
            target: ("q_1_next".to_owned(), "q_0".to_owned()),
        };
        let near_var = RoutingConnection {
            source: ("q_0_slave".to_owned(), "q".to_owned()),
            target: ("q_0_next".to_owned(), "q_0".to_owned()),
        };
        let far_sink = ResolvedPortTarget {
            position: Position(10, 1, 0),
            requires_input_diode: true,
            input_repeater_delay: 1,
        };
        let near_sink = ResolvedPortTarget {
            position: Position(3, 1, 0),
            requires_input_diode: true,
            input_repeater_delay: 1,
        };
        let mut sinks = vec![(&near_var, near_sink), (&far_var, far_sink)];

        sort_internal_sink_targets_by_current_tree(&mut sinks, &[source]);

        assert_eq!(sinks[0].1.position, far_sink.position);
        assert_eq!(sinks[1].1.position, near_sink.position);
    }

    #[test]
    fn route_var_grouping_preserves_sorted_priority_boundaries() {
        let q0_to_own_bit = RoutingConnection {
            source: ("q_0_slave".to_owned(), "q".to_owned()),
            target: ("q_0_next".to_owned(), "q_0".to_owned()),
        };
        let q1_feedback = RoutingConnection {
            source: ("q_1_slave".to_owned(), "q".to_owned()),
            target: ("q_1_next".to_owned(), "q_1".to_owned()),
        };
        let q0_to_next_bit = RoutingConnection {
            source: ("q_0_slave".to_owned(), "q".to_owned()),
            target: ("q_1_next".to_owned(), "q_0".to_owned()),
        };
        let vars = vec![&q0_to_own_bit, &q1_feedback, &q0_to_next_bit];

        let groups = group_vars_by_source_ordered(&vars);

        assert_eq!(groups.len(), 3);
        assert_eq!(groups[0][0].target, q0_to_own_bit.target);
        assert_eq!(groups[1][0].target, q1_feedback.target);
        assert_eq!(groups[2][0].target, q0_to_next_bit.target);
    }

    #[test]
    fn route_var_grouping_keeps_adjacent_same_source_fanout_together() {
        let q0_to_own_bit = RoutingConnection {
            source: ("q_0_slave".to_owned(), "q".to_owned()),
            target: ("q_0_next".to_owned(), "q_0".to_owned()),
        };
        let q0_to_next_bit = RoutingConnection {
            source: ("q_0_slave".to_owned(), "q".to_owned()),
            target: ("q_1_next".to_owned(), "q_0".to_owned()),
        };
        let vars = vec![&q0_to_own_bit, &q0_to_next_bit];

        let groups = group_vars_by_source_ordered(&vars);

        assert_eq!(groups.len(), 1);
        assert_eq!(
            groups[0]
                .iter()
                .map(|var| var.target.clone())
                .collect::<Vec<_>>(),
            vec![q0_to_own_bit.target, q0_to_next_bit.target]
        );
    }

    #[test]
    fn resolved_routing_binds_physical_branches_to_typed_net_ids() -> eyre::Result<()> {
        let candidates = vec![
            candidate(
                "left",
                Position(0, 0, 1),
                "out",
                PhysicalPortDirection::Output,
            ),
            candidate(
                "right",
                Position(0, 0, 1),
                "in",
                PhysicalPortDirection::Input,
            ),
        ];
        let placed = place_candidates_on_shelves(
            &candidates,
            &GlobalPlacementConfig {
                spacing: 3,
                shelf_width: 16,
                ..Default::default()
            },
        );
        let left_endpoint = ResolvedEndpoint::InstancePort {
            instance: InstanceId(0),
            port: PortId(0),
        };
        let right_endpoint = ResolvedEndpoint::InstancePort {
            instance: InstanceId(1),
            port: PortId(1),
        };
        let topology = ResolvedPnrTopology {
            top: DefinitionId(0),
            definitions: vec![
                ResolvedDefinition {
                    id: DefinitionId(0),
                    key: DefinitionKey("top".to_owned()),
                    display_name: "top".to_owned(),
                    ports: Vec::new(),
                    is_leaf: false,
                },
                ResolvedDefinition {
                    id: DefinitionId(1),
                    key: DefinitionKey("left".to_owned()),
                    display_name: "left".to_owned(),
                    ports: vec![PortId(0)],
                    is_leaf: true,
                },
                ResolvedDefinition {
                    id: DefinitionId(2),
                    key: DefinitionKey("right".to_owned()),
                    display_name: "right".to_owned(),
                    ports: vec![PortId(1)],
                    is_leaf: true,
                },
            ],
            ports: vec![
                ResolvedPort {
                    id: PortId(0),
                    definition: DefinitionId(1),
                    name: "out".to_owned(),
                    direction: RoutablePortDirection::Output,
                },
                ResolvedPort {
                    id: PortId(1),
                    definition: DefinitionId(2),
                    name: "in".to_owned(),
                    direction: RoutablePortDirection::Input,
                },
            ],
            instances: vec![
                ResolvedInstance {
                    id: InstanceId(0),
                    key: InstanceKey("top/left".to_owned()),
                    display_name: "left".to_owned(),
                    definition: DefinitionId(1),
                },
                ResolvedInstance {
                    id: InstanceId(1),
                    key: InstanceKey("top/right".to_owned()),
                    display_name: "right".to_owned(),
                    definition: DefinitionId(2),
                },
            ],
            nets: vec![ResolvedNet {
                id: NetId(0),
                key: NetKey("top/net/data".to_owned()),
                display_name: "data".to_owned(),
                class: NetClass::Data,
                driver: left_endpoint.clone(),
                sinks: vec![right_endpoint.clone()],
            }],
        };

        let routes = route_resolved_topology_with_order_from_prefix(
            &topology,
            None,
            &GlobalHeuristicHooks::default(),
            &candidates,
            &placed,
            &GlobalRoutingConfig::default(),
            NetOrderStrategy::Criticality,
            &silent_progress(),
            &[],
        )
        .map_err(|failure| failure.error)?;

        assert_eq!(routes.len(), 1);
        assert_eq!(routes[0].net_id, Some(NetId(0)));
        assert_eq!(routes[0].source_endpoint, Some(left_endpoint));
        assert_eq!(routes[0].sink_endpoint, Some(right_endpoint));
        Ok(())
    }

    #[test]
    fn route_point_to_point_supports_astar_strategy() {
        let source = Position(0, 1, 1);
        let sink = Position(22, 1, 1);
        let world = route_test_world_with_size(source, sink, DimSize(26, 4, 3));
        let (route, _) =
            route_point_to_point_with_strategy(&world, source, sink, GlobalRoutingStrategy::AStar)
                .unwrap();

        assert!(route
            .blocks
            .iter()
            .any(|(_, block)| block.kind.is_repeater()));
    }
}
