//! M0 benchmark circuits and baseline metrics.
//!
//! The benchmark set is fixed so engine changes can be measured against the
//! same inputs. `measure` lowers a benchmark and records its structural
//! metrics; the baseline place-and-route test is ignored because the current
//! beam-search engine can be very slow on the dense benchmarks.

use std::collections::BTreeMap;

use eyre::{bail, ContextCompat, WrapErr};
use serde::Serialize;

use crate::graph::logic::LogicGraph;
use crate::ir::{
    graph_from_routable_leaf, LogicalDesign, RoutableModule, RoutableModuleBody,
    RoutablePortDirection,
};
use crate::transform::place_and_route::global_pnr::ir::{PhysicalPortDirection, PortConnection};
use crate::transform::place_and_route::global_pnr::topology::{
    ResolvedEndpoint, ResolvedPnrTopology,
};
use crate::transform::place_and_route::placement_ir::{
    MacroPin, MacroRotation, MacroTemplate, PhysicalNet, PinRef, PlacementProblem,
};
use crate::transform::place_and_route::sa_placer::{
    place_annealed, place_initial, AnnealingConfig, InitialPlacementConfig,
};
use crate::world::block::Direction;
use crate::world::position::{DimSize, Position};

const BENCHMARKS: &[(&str, &str)] = &[
    (
        "not_chain",
        include_str!("../../test/benchmarks/not_chain.v"),
    ),
    (
        "full_adder",
        include_str!("../../test/benchmarks/full_adder.v"),
    ),
    (
        "dense_or_cone",
        include_str!("../../test/benchmarks/dense_or_cone.v"),
    ),
    ("fsm_1bit", include_str!("../../test/benchmarks/fsm_1bit.v")),
    ("fsm_2bit", include_str!("../../test/benchmarks/fsm_2bit.v")),
    (
        "random_10",
        include_str!("../../test/benchmarks/random_10.v"),
    ),
    (
        "random_40",
        include_str!("../../test/benchmarks/random_40.v"),
    ),
];

/// Structural metrics of one lowered benchmark.
#[derive(Debug, Clone, Serialize)]
pub struct BenchmarkMetrics {
    pub name: String,
    pub modules: usize,
    pub leaves: usize,
    pub cells: usize,
    pub max_prepared_nodes: usize,
    pub over_limit_leaves: usize,
}

/// Lowers one benchmark and measures its leaves after `prepare_place`, which
/// is the graph the local placer's node limit applies to.
pub fn measure(name: &str, source: &str) -> eyre::Result<BenchmarkMetrics> {
    let logical = LogicalDesign::from_verilog_source_named(source, &format!("{name}.v"))?;
    let cells = logical
        .modules
        .iter()
        .map(|module| module.cells.len())
        .sum();
    let routable = logical.lower_to_routable()?;
    let _topology = ResolvedPnrTopology::from_routable(&routable)
        .with_context(|| format!("benchmark `{name}` does not resolve to a PnR topology"))?;

    let mut leaves = 0;
    let mut max_prepared = 0;
    let mut over_limit = 0;
    for module in &routable.modules {
        if !matches!(module.body, RoutableModuleBody::Leaf { .. }) {
            continue;
        }
        leaves += 1;
        let graph = graph_from_routable_leaf(module)?;
        let prepared = LogicGraph { graph }.prepare_place()?;
        max_prepared = max_prepared.max(prepared.nodes.len());
        if prepared.nodes.len() > 40 {
            over_limit += 1;
        }
    }

    Ok(BenchmarkMetrics {
        name: name.to_owned(),
        modules: routable.modules.len(),
        leaves,
        cells,
        max_prepared_nodes: max_prepared,
        over_limit_leaves: over_limit,
    })
}

/// Placement metrics of one benchmark after the CAD initial placement and
/// simulated annealing refinement.
#[derive(Debug, Clone, Serialize)]
pub struct PlacementBenchmarkMetrics {
    pub name: String,
    pub macros: usize,
    pub instances: usize,
    pub nets: usize,
    pub seed_wire_length: usize,
    pub wire_length: usize,
    pub bounding_box_volume: usize,
    pub legal: bool,
    pub elapsed_ms: u128,
}

/// Builds a placement problem from the benchmark topology with structural
/// macro footprints, then runs the CAD initial placement and annealing
/// refinement.
pub fn measure_placement(name: &str, source: &str) -> eyre::Result<PlacementBenchmarkMetrics> {
    let started = std::time::Instant::now();
    let logical = LogicalDesign::from_verilog_source_named(source, &format!("{name}.v"))?;
    let routable = logical.lower_to_routable()?;
    let topology = ResolvedPnrTopology::from_routable(&routable)
        .with_context(|| format!("benchmark `{name}` does not resolve to a PnR topology"))?;

    let top = routable
        .module(&routable.top)
        .with_context(|| format!("benchmark `{name}` has no top module"))?;
    let RoutableModuleBody::Composite { instances, .. } = &top.body else {
        bail!("benchmark `{name}` top module is not composite");
    };

    let mut problem = PlacementProblem::new();
    let mut seen_macros = BTreeMap::<String, ()>::new();
    for instance in instances {
        if seen_macros.contains_key(&instance.module) {
            continue;
        }
        let child = routable.module(&instance.module).with_context(|| {
            format!(
                "benchmark `{name}` references missing module `{}`",
                instance.module
            )
        })?;
        problem.add_macro(structural_template(child)?);
        seen_macros.insert(instance.module.clone(), ());
    }

    let mut instance_index = BTreeMap::<String, usize>::new();
    for instance in instances {
        let index =
            problem.add_instance(&instance.module, Position(0, 0, 0), MacroRotation::None)?;
        instance_index.insert(instance.name.clone(), index);
    }

    for net in &topology.nets {
        let Some(source) = placement_endpoint(&topology, &net.driver, &instance_index) else {
            continue;
        };
        let sinks = net
            .sinks
            .iter()
            .filter_map(|sink| placement_endpoint(&topology, sink, &instance_index))
            .collect::<Vec<_>>();
        if sinks.is_empty() {
            continue;
        }
        problem.nets.push(PhysicalNet {
            name: net.display_name.clone(),
            source,
            sinks,
            route: None,
            region_sequence: None,
            congestion: 0,
        });
    }

    let total_volume = problem
        .macros
        .values()
        .map(|template| template.size.0 * template.size.1 * template.size.2)
        .sum::<usize>();
    let side = ((total_volume as f64).cbrt().ceil() as usize * 2).max(16);
    let max_height = problem
        .macros
        .values()
        .map(|template| template.size.2)
        .max()
        .unwrap_or(1);
    let config = AnnealingConfig {
        initial: InitialPlacementConfig {
            world: DimSize(side, side, max_height.max(4) + 4),
            ..Default::default()
        },
        ..Default::default()
    };

    let seed = place_initial(&problem, &config.initial)?;
    let solution = place_annealed(&problem, &config)?;

    let mut placed = problem.clone();
    placed.instances = solution.instances.clone();
    let bounding_box_volume = placed
        .bounding_box()
        .map(|(min, max)| (max.0 - min.0 + 1) * (max.1 - min.1 + 1) * (max.2 - min.2 + 1))
        .unwrap_or(0);

    Ok(PlacementBenchmarkMetrics {
        name: name.to_owned(),
        macros: problem.macros.len(),
        instances: problem.instances.len(),
        nets: problem.nets.len(),
        seed_wire_length: seed.wire_length,
        wire_length: solution.wire_length,
        bounding_box_volume,
        legal: solution.is_legal(),
        elapsed_ms: started.elapsed().as_millis(),
    })
}

fn structural_template(module: &RoutableModule) -> eyre::Result<MacroTemplate> {
    let graph = graph_from_routable_leaf(module)?;
    let prepared = LogicGraph { graph }.prepare_place()?;
    let nodes = prepared.nodes.len().max(1);
    let side = ((nodes as f64).sqrt().ceil() as usize).max(1);
    let height = 1 + nodes / 16;
    let size = DimSize(side, side, height);

    let inputs = module
        .ports
        .iter()
        .filter(|port| port.direction == RoutablePortDirection::Input)
        .collect::<Vec<_>>();
    let outputs = module
        .ports
        .iter()
        .filter(|port| port.direction == RoutablePortDirection::Output)
        .collect::<Vec<_>>();

    let mut pins = Vec::with_capacity(inputs.len() + outputs.len());
    for (index, port) in inputs.iter().enumerate() {
        let y = ((index + 1) * size.1 / (inputs.len() + 1)).min(size.1 - 1);
        let position = Position(0, y, 0);
        pins.push(MacroPin {
            name: port.name.clone(),
            position,
            direction: PhysicalPortDirection::Input,
            connection: PortConnection::Direct,
            facing: Direction::West,
            escape: vec![position],
        });
    }
    for (index, port) in outputs.iter().enumerate() {
        let y = ((index + 1) * size.1 / (outputs.len() + 1)).min(size.1 - 1);
        let position = Position(size.0 - 1, y, 0);
        pins.push(MacroPin {
            name: port.name.clone(),
            position,
            direction: PhysicalPortDirection::Output,
            connection: PortConnection::Direct,
            facing: Direction::East,
            escape: vec![position],
        });
    }
    pins.sort_by(|left, right| left.name.cmp(&right.name));

    Ok(MacroTemplate {
        name: module.name.clone(),
        variant: module.name.clone(),
        size,
        blocks: Vec::new(),
        forbidden_routing_cells: Vec::new(),
        pins,
        halo: 0,
        allowed_rotations: vec![MacroRotation::None],
        verified: false,
    })
}

fn placement_endpoint(
    topology: &ResolvedPnrTopology,
    endpoint: &ResolvedEndpoint,
    instance_index: &BTreeMap<String, usize>,
) -> Option<PinRef> {
    let ResolvedEndpoint::InstancePort { instance, port } = endpoint else {
        return None;
    };
    let resolved = topology.instances.get(instance.0)?;
    let problem_index = *instance_index.get(&resolved.display_name)?;
    let port_name = topology.port(*port)?.name.clone();
    Some(PinRef {
        instance: problem_index,
        pin: port_name,
    })
}

#[test]
fn benchmarks_lower_to_valid_topologies() -> eyre::Result<()> {
    let mut failures = Vec::new();
    let mut metrics = Vec::new();
    for (name, source) in BENCHMARKS {
        let measured = measure(name, source)?;
        eprintln!(
            "{:<14} modules={:<3} leaves={:<3} cells={:<3} max_prepared={:<3} over_limit={}",
            measured.name,
            measured.modules,
            measured.leaves,
            measured.cells,
            measured.max_prepared_nodes,
            measured.over_limit_leaves
        );
        if measured.over_limit_leaves > 0 {
            failures.push(measured.name.clone());
        }
        metrics.push(measured);
    }

    // Record the structural baseline for later engine comparisons.
    let baseline = serde_json::json!({
        "format": "redstone-compiler.benchmark-baseline.v1",
        "benchmarks": metrics,
    });
    std::fs::create_dir_all("target")?;
    std::fs::write(
        "target/benchmark-baseline.json",
        serde_json::to_vec_pretty(&baseline)?,
    )?;

    assert!(
        failures.is_empty(),
        "benchmarks exceed the local placer node limit: {failures:?}"
    );
    Ok(())
}

#[test]
#[ignore = "baseline place-and-route; run manually with --release"]
fn benchmark_pnr_baseline() -> eyre::Result<()> {
    use crate::transform::place_and_route::global_pnr::{
        place_and_route_logical_design, GlobalPnrConfig,
    };

    for (name, source) in BENCHMARKS {
        let logical = LogicalDesign::from_verilog_source_named(source, &format!("{name}.v"))?;
        let started = std::time::Instant::now();
        let outcome = std::panic::catch_unwind(|| {
            place_and_route_logical_design(
                &logical,
                &GlobalPnrConfig {
                    show_progress: false,
                    ..Default::default()
                },
            )
        });
        match outcome {
            Ok(Ok(world)) => println!(
                "{name}: PnR ok blocks={} elapsed_ms={}",
                world.iter_block().len(),
                started.elapsed().as_millis()
            ),
            Ok(Err(error)) => println!("{name}: PnR failed: {error}"),
            Err(_) => println!("{name}: PnR panicked"),
        }
    }
    Ok(())
}

#[test]
#[ignore = "full PnR pathfinder comparison; run manually with --release and MCHDL_BENCH"]
fn benchmark_pathfinder_baseline() -> eyre::Result<()> {
    use crate::transform::place_and_route::global_pnr::candidate::UnitCandidateConfig;
    use crate::transform::place_and_route::global_pnr::route_engine::PathfinderConfig;
    use crate::transform::place_and_route::global_pnr::router::{
        GlobalRoutingConfig, GlobalRoutingStrategy, RouteValidationMode,
    };
    use crate::transform::place_and_route::global_pnr::{
        place_and_route_logical_design, GlobalPnrConfig,
    };

    let filter = std::env::var("MCHDL_BENCH").ok();
    let mut results = Vec::new();
    for (name, source) in BENCHMARKS {
        if filter.as_deref().is_some_and(|filter| filter != *name) {
            continue;
        }
        let logical = LogicalDesign::from_verilog_source_named(source, &format!("{name}.v"))?;
        for pathfinder in [None, Some(PathfinderConfig::default())] {
            let routing = GlobalRoutingConfig {
                strategy: GlobalRoutingStrategy::AStar,
                validation: RouteValidationMode::Deferred,
                pathfinder,
            };
            let config = GlobalPnrConfig {
                show_progress: false,
                candidate: UnitCandidateConfig {
                    max_candidates: 1,
                    ..Default::default()
                }
                .into(),
                routing_probe: Some(routing),
                routing,
                ..Default::default()
            };
            let started = std::time::Instant::now();
            let outcome =
                std::panic::catch_unwind(|| place_and_route_logical_design(&logical, &config));
            let elapsed_ms = started.elapsed().as_millis();
            let (ok, blocks, error) = match outcome {
                Ok(Ok(world)) => (true, world.iter_block().len(), None),
                Ok(Err(error)) => (false, 0, Some(error.to_string())),
                Err(_) => (false, 0, Some("panicked".to_owned())),
            };
            println!(
                "{name:<14} pathfinder={} ok={ok} blocks={blocks} elapsed_ms={elapsed_ms} error={}",
                pathfinder.is_some(),
                error.as_deref().unwrap_or("-")
            );
            results.push(serde_json::json!({
                "name": name,
                "pathfinder": pathfinder.is_some(),
                "ok": ok,
                "blocks": blocks,
                "elapsed_ms": elapsed_ms,
                "error": error,
            }));
        }
    }

    let baseline = serde_json::json!({
        "format": "redstone-compiler.benchmark-pathfinder.v1",
        "results": results,
    });
    std::fs::create_dir_all("target")?;
    let path = match filter {
        Some(name) => format!("target/benchmark-pathfinder-{name}.json"),
        None => "target/benchmark-pathfinder.json".to_owned(),
    };
    std::fs::write(path, serde_json::to_vec_pretty(&baseline)?)?;
    Ok(())
}

#[test]
fn random_10_placement_is_legal_and_connected() -> eyre::Result<()> {
    let metrics = measure_placement("random_10", BENCHMARKS[5].1)?;

    assert!(metrics.legal, "{metrics:?}");
    assert!(metrics.instances > 0, "{metrics:?}");
    assert!(metrics.nets > 0, "{metrics:?}");
    assert!(metrics.wire_length > 0, "{metrics:?}");
    Ok(())
}

#[test]
#[ignore = "placement baseline; run manually with --release"]
fn benchmark_placement_baseline() -> eyre::Result<()> {
    let mut metrics = Vec::new();
    for (name, source) in BENCHMARKS {
        match measure_placement(name, source) {
            Ok(measured) => {
                println!(
                    "{:<14} macros={:<3} instances={:<3} nets={:<3} seed_wire={:<5} wire={:<5} bbox_volume={:<6} legal={} elapsed_ms={}",
                    measured.name,
                    measured.macros,
                    measured.instances,
                    measured.nets,
                    measured.seed_wire_length,
                    measured.wire_length,
                    measured.bounding_box_volume,
                    measured.legal,
                    measured.elapsed_ms
                );
                metrics.push(measured);
            }
            Err(error) => println!("{name}: placement failed: {error}"),
        }
    }

    let baseline = serde_json::json!({
        "format": "redstone-compiler.benchmark-placement.v1",
        "benchmarks": metrics,
    });
    std::fs::create_dir_all("target")?;
    std::fs::write(
        "target/benchmark-placement.json",
        serde_json::to_vec_pretty(&baseline)?,
    )?;
    Ok(())
}

#[test]
#[ignore = "full PnR engine comparison; run manually with --release"]
fn benchmark_placement_engines_baseline() -> eyre::Result<()> {
    use crate::transform::place_and_route::global_pnr::candidate::UnitCandidateConfig;
    use crate::transform::place_and_route::global_pnr::placer::{
        GlobalPlacementConfig, PlacementEngine,
    };
    use crate::transform::place_and_route::global_pnr::policy::{
        GlobalPnrPolicies, GlobalSearchBudget, PlacementHeuristic,
    };
    use crate::transform::place_and_route::global_pnr::router::{
        GlobalRoutingConfig, GlobalRoutingStrategy, NetOrderStrategy, RouteValidationMode,
    };
    use crate::transform::place_and_route::global_pnr::{
        place_and_route_logical_design, GlobalPnrConfig, GlobalSearchConfig,
    };

    let filter = std::env::var("MCHDL_BENCH").ok();
    let mut results = Vec::new();
    for (name, source) in BENCHMARKS {
        if filter.as_deref().is_some_and(|filter| filter != *name) {
            continue;
        }
        let logical = LogicalDesign::from_verilog_source_named(source, &format!("{name}.v"))?;
        for engine in [PlacementEngine::Legacy, PlacementEngine::Annealed] {
            let routing = GlobalRoutingConfig {
                strategy: GlobalRoutingStrategy::DirectGreedy { max_steps: 64 },
                validation: RouteValidationMode::Deferred,
                pathfinder: None,
            };
            let config = GlobalPnrConfig {
                show_progress: false,
                candidate: UnitCandidateConfig {
                    max_candidates: 1,
                    ..Default::default()
                }
                .into(),
                placement: GlobalPlacementConfig {
                    engine,
                    spacing: 2,
                    max_attempts: 1,
                    ..Default::default()
                },
                routing_probe: Some(routing),
                routing,
                search: GlobalSearchConfig {
                    budget: GlobalSearchBudget {
                        max_candidates_per_child: 1,
                        max_layout_combinations: 1,
                        max_detailed_routing_attempts: 1,
                        max_refined_routing_attempts: 1,
                        max_refinement_rounds: 1,
                    },
                    policies: GlobalPnrPolicies {
                        placement_heuristics: vec![PlacementHeuristic::Shelf],
                        net_order_strategies: vec![NetOrderStrategy::Criticality],
                    },
                },
                ..Default::default()
            };
            let started = std::time::Instant::now();
            let outcome =
                std::panic::catch_unwind(|| place_and_route_logical_design(&logical, &config));
            let elapsed_ms = started.elapsed().as_millis();
            let (ok, blocks, error) = match outcome {
                Ok(Ok(world)) => (true, world.iter_block().len(), None),
                Ok(Err(error)) => (false, 0, Some(error.to_string())),
                Err(_) => (false, 0, Some("panicked".to_owned())),
            };
            println!(
                "{name:<14} engine={engine:?} ok={ok} blocks={blocks} elapsed_ms={elapsed_ms} error={}",
                error.as_deref().unwrap_or("-")
            );
            results.push(serde_json::json!({
                "name": name,
                "engine": format!("{engine:?}"),
                "ok": ok,
                "blocks": blocks,
                "elapsed_ms": elapsed_ms,
                "error": error,
            }));
        }
    }

    let baseline = serde_json::json!({
        "format": "redstone-compiler.benchmark-placement-engines.v1",
        "results": results,
    });
    std::fs::create_dir_all("target")?;
    let path = match filter {
        Some(name) => format!("target/benchmark-placement-engines-{name}.json"),
        None => "target/benchmark-placement-engines.json".to_owned(),
    };
    std::fs::write(path, serde_json::to_vec_pretty(&baseline)?)?;
    Ok(())
}
