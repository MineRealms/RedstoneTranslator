//! M0 benchmark circuits and baseline metrics.
//!
//! The benchmark set is fixed so engine changes can be measured against the
//! same inputs. `measure` lowers a benchmark and records its structural
//! metrics; the baseline place-and-route test is ignored because the current
//! beam-search engine can be very slow on the dense benchmarks.

use eyre::WrapErr;
use serde::Serialize;

use crate::graph::logic::LogicGraph;
use crate::ir::{graph_from_routable_leaf, LogicalDesign, RoutableModuleBody};
use crate::transform::place_and_route::global_pnr::topology::ResolvedPnrTopology;

const BENCHMARKS: &[(&str, &str)] = &[
    ("not_chain", include_str!("../../test/benchmarks/not_chain.v")),
    ("full_adder", include_str!("../../test/benchmarks/full_adder.v")),
    (
        "dense_or_cone",
        include_str!("../../test/benchmarks/dense_or_cone.v"),
    ),
    ("fsm_1bit", include_str!("../../test/benchmarks/fsm_1bit.v")),
    ("fsm_2bit", include_str!("../../test/benchmarks/fsm_2bit.v")),
    ("random_10", include_str!("../../test/benchmarks/random_10.v")),
    ("random_40", include_str!("../../test/benchmarks/random_40.v")),
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
    let cells = logical.modules.iter().map(|module| module.cells.len()).sum();
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
