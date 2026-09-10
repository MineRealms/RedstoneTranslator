use std::path::PathBuf;

use eyre::WrapErr;
use mimalloc::MiMalloc;
use redstone_compiler::ir::{LogicalDesign, MappingSpec, RcirDocument};
use redstone_compiler::snapshot::{compile_with_snapshot, SnapshotOptions};
use redstone_compiler::transform::place_and_route::compression::{
    default_ladder, place_and_route_logical_design_with_compression,
    place_and_route_with_compression,
};
use redstone_compiler::transform::place_and_route::global_pnr::cell_library::CellLibrary;
use redstone_compiler::transform::place_and_route::global_pnr::topology::ResolvedPnrTopology;
use redstone_compiler::transform::place_and_route::global_pnr::{
    apply_routable_document, emit_prepared_pnr_snapshot, load_prepared_pnr_snapshot,
    place_and_route_logical_design_with_mapping,
    place_and_route_routable_design_with_visualization, run_prepared_pnr_with_visualization,
    GlobalPnrConfig, PhysicalIntent, PnrPrepareConfig,
};
use structopt::StructOpt;

#[global_allocator]
static GLOBAL: MiMalloc = MiMalloc;

#[derive(Debug, StructOpt)]
#[structopt(name = "example", about = "An example of StructOpt usage.")]
pub struct CompilerOption {
    #[structopt(parse(from_os_str))]
    pub input: PathBuf,

    #[structopt(parse(from_os_str))]
    pub output: Option<PathBuf>,

    /// Optional per-design floorplan and routing intent.
    #[structopt(long, parse(from_os_str))]
    pub intent: Option<PathBuf>,

    /// Reuse structurally identical local-placement candidates across runs.
    #[structopt(long, parse(from_os_str))]
    pub candidate_cache: Option<PathBuf>,

    /// Reusable cell library (JSON) applied to local candidate policies.
    /// Replay restores the library embedded in the snapshot instead.
    #[structopt(long, parse(from_os_str))]
    pub cell_library: Option<PathBuf>,

    /// Lowering configuration (JSON): target name and mapping policy.
    #[structopt(long, parse(from_os_str))]
    pub mapping_policy: Option<PathBuf>,

    /// Shrink the design with the compression ladder (replaces `--intent`).
    #[structopt(long)]
    pub compress: bool,
}

fn load_mapping_spec(path: &std::path::Path) -> eyre::Result<MappingSpec> {
    let source = std::fs::read_to_string(path)?;
    MappingSpec::from_json(&source)
        .wrap_err_with(|| format!("load mapping spec {}", path.display()))
}

fn load_cell_library(path: &std::path::Path) -> eyre::Result<CellLibrary> {
    let source = std::fs::read_to_string(path)?;
    CellLibrary::from_json(&source)
        .wrap_err_with(|| format!("load cell library {}", path.display()))
}

fn apply_cell_library(config: &mut GlobalPnrConfig, library: Option<CellLibrary>) {
    if let Some(library) = library {
        config.candidate = config.candidate.clone().with_cell_library(library);
    }
}

fn main() -> eyre::Result<()> {
    tracing_subscriber::fmt::init();
    let opt = CompilerOption::from_args();

    match opt.input.extension().and_then(|ext| ext.to_str()) {
        Some("rcir") => compile_rcir_input(opt),
        Some("rsnap" | "snapshot") => replay_snapshot_input(opt),
        Some("v") => compile_verilog_input(opt),
        _ => eyre::bail!("unsupported input file extension: {:?}", opt.input),
    }
}

fn replay_snapshot_input(opt: CompilerOption) -> eyre::Result<()> {
    // Replay restores the cell library embedded in the snapshot; an explicit
    // `--cell-library` only affects fresh compiles.
    let mut base_config = GlobalPnrConfig::default();
    base_config.candidate_cache_dir = opt.candidate_cache.clone();
    let prepare_config = PnrPrepareConfig::from(&base_config);
    let Some(output) = opt.output else {
        let prepared = load_prepared_pnr_snapshot(&opt.input, &prepare_config)?;
        let (explicit_intent, _) =
            bind_physical_intent(opt.intent.as_deref(), prepared.topology())?;
        let active_intent = explicit_intent
            .as_ref()
            .or_else(|| prepared.snapshot_intent());
        println!(
            "loaded prepared PnR: module={} instances={} candidate_sets={} candidates={} constraints={}",
            prepared.module_name(),
            prepared.summary().instances,
            prepared.summary().unique_candidate_sets,
            prepared.summary().candidates,
            active_intent.map_or(0, |intent| intent.constraints.len()),
        );
        return Ok(());
    };

    let (snapshot_dir, snapshot_archive, options) =
        snapshot_options_without_source(&opt.input, &output);
    let prepared = load_prepared_pnr_snapshot(&opt.input, &prepare_config)?;
    let (physical_intent, intent_source) =
        bind_physical_intent(opt.intent.as_deref(), prepared.topology())?;
    let mut config = base_config;
    prepared.apply_snapshot_config(&mut config)?;
    config.physical_intent = physical_intent.or_else(|| prepared.snapshot_intent().cloned());
    compile_with_snapshot(options, || {
        emit_intent_source(intent_source.as_ref())?;
        emit_prepared_pnr_snapshot(&prepared)?;
        run_prepared_pnr_with_visualization(&prepared, &config)
    })?;
    println!(
        "replayed global PnR without local placement: path={}",
        snapshot_dir.display()
    );
    println!(
        "exported snapshot archive: path={}",
        snapshot_archive.display()
    );
    Ok(())
}

fn compile_verilog_input(opt: CompilerOption) -> eyre::Result<()> {
    let source = std::fs::read_to_string(&opt.input)?;
    let source_name = opt
        .input
        .file_name()
        .and_then(|name| name.to_str())
        .unwrap_or("source.v");
    let logical = LogicalDesign::from_verilog_source_named(&source, source_name)?;
    let cell_library = opt
        .cell_library
        .as_deref()
        .map(load_cell_library)
        .transpose()?;
    let mapping = opt
        .mapping_policy
        .as_deref()
        .map(load_mapping_spec)
        .transpose()?
        .unwrap_or_default();
    let Some(output) = opt.output else {
        let cells = logical
            .modules
            .iter()
            .map(|module| module.cells.len())
            .sum::<usize>();
        let instances = logical
            .modules
            .iter()
            .map(|module| module.instances.len())
            .sum::<usize>();
        println!(
            "loaded Verilog as logical IR: top={} modules={} cells={} instances={}",
            logical.top,
            logical.modules.len(),
            cells,
            instances
        );
        return Ok(());
    };

    let (snapshot_dir, snapshot_archive, options) = snapshot_options(&opt.input, &output);
    if opt.compress && opt.intent.is_some() {
        eyre::bail!(
            "--compress replaces the physical intent; pass only one of --compress and --intent"
        );
    }
    let routable =
        logical.lower_to_routable_with_target(&mapping.target_spec()?, &mapping.policy)?;
    let topology = ResolvedPnrTopology::from_routable(&routable)?;
    let (physical_intent, intent_source) = bind_physical_intent(opt.intent.as_deref(), &topology)?;
    let mut config = GlobalPnrConfig::default();
    config.physical_intent = physical_intent;
    config.candidate_cache_dir = opt.candidate_cache.clone();
    apply_cell_library(&mut config, cell_library);
    compile_with_snapshot(options, || {
        emit_intent_source(intent_source.as_ref())?;
        if opt.compress {
            let compressed = place_and_route_logical_design_with_compression(
                &logical,
                &mapping,
                &default_ladder(),
                &config,
            )?;
            println!(
                "compression: selected box {:?} after {} attempt(s)",
                compressed.box_size,
                compressed.attempts.len()
            );
            Ok(compressed.value)
        } else {
            place_and_route_logical_design_with_mapping(&logical, &mapping, &config)
        }
    })?;

    println!("exported Verilog snapshot: path={}", snapshot_dir.display());
    println!(
        "exported snapshot archive: path={}",
        snapshot_archive.display()
    );

    Ok(())
}

fn compile_rcir_input(opt: CompilerOption) -> eyre::Result<()> {
    let source = std::fs::read_to_string(&opt.input)?;
    let ir: RcirDocument = source.parse()?;
    let cell_library = opt
        .cell_library
        .as_deref()
        .map(load_cell_library)
        .transpose()?;
    let mapping = opt
        .mapping_policy
        .as_deref()
        .map(load_mapping_spec)
        .transpose()?
        .unwrap_or_default();
    let Some(output) = opt.output else {
        match &ir {
            RcirDocument::Logical(design) => println!(
                "loaded logical IR: top={} modules={}",
                design.top,
                design.modules.len()
            ),
            RcirDocument::Routable(document) => println!(
                "loaded routable IR: top={} modules={} target={}",
                document.design.top,
                document.design.modules.len(),
                document.design.target
            ),
        }
        return Ok(());
    };

    let (snapshot_dir, snapshot_archive, options) = snapshot_options(&opt.input, &output);
    if opt.compress && opt.intent.is_some() {
        eyre::bail!(
            "--compress replaces the physical intent; pass only one of --compress and --intent"
        );
    }
    let routable = match &ir {
        RcirDocument::Logical(design) => {
            design.lower_to_routable_with_target(&mapping.target_spec()?, &mapping.policy)?
        }
        RcirDocument::Routable(document) => document.design.clone(),
    };
    let topology = ResolvedPnrTopology::from_routable(&routable)?;
    let (physical_intent, intent_source) = bind_physical_intent(opt.intent.as_deref(), &topology)?;
    let mut config = GlobalPnrConfig::default();
    config.physical_intent = physical_intent.clone();
    config.candidate_cache_dir = opt.candidate_cache.clone();
    if let RcirDocument::Routable(document) = &ir {
        apply_routable_document(document, &mut config)?;
        if physical_intent.is_some() {
            config.physical_intent = physical_intent.clone();
        }
    }
    apply_cell_library(&mut config, cell_library);
    match &ir {
        RcirDocument::Logical(design) => compile_with_snapshot(options, || {
            emit_intent_source(intent_source.as_ref())?;
            if opt.compress {
                let compressed = place_and_route_logical_design_with_compression(
                    design,
                    &mapping,
                    &default_ladder(),
                    &config,
                )?;
                println!(
                    "compression: selected box {:?} after {} attempt(s)",
                    compressed.box_size,
                    compressed.attempts.len()
                );
                Ok(compressed.value)
            } else {
                place_and_route_logical_design_with_mapping(design, &mapping, &config)
            }
        })?,
        RcirDocument::Routable(document) => compile_with_snapshot(options, || {
            emit_intent_source(intent_source.as_ref())?;
            if opt.compress {
                let compressed =
                    place_and_route_with_compression(&document.design, &default_ladder(), &config)?;
                println!(
                    "compression: selected box {:?} after {} attempt(s)",
                    compressed.box_size,
                    compressed.attempts.len()
                );
                Ok(compressed.value)
            } else {
                place_and_route_routable_design_with_visualization(&document.design, &config)
            }
        })?,
    };

    println!("exported rcir snapshot: path={}", snapshot_dir.display());
    println!(
        "exported snapshot archive: path={}",
        snapshot_archive.display()
    );
    Ok(())
}

fn snapshot_options(
    input: &std::path::Path,
    output: &std::path::Path,
) -> (PathBuf, PathBuf, SnapshotOptions) {
    let snapshot_dir = snapshot_output_dir(output);
    let snapshot_archive = snapshot_dir.with_extension("rsnap");
    let design_name = output
        .file_stem()
        .or_else(|| input.file_stem())
        .and_then(|name| name.to_str())
        .unwrap_or("design")
        .to_owned();
    let options = SnapshotOptions::new(&snapshot_dir, design_name).with_source(input);
    (snapshot_dir, snapshot_archive, options)
}

fn bind_physical_intent(
    path: Option<&std::path::Path>,
    topology: &ResolvedPnrTopology,
) -> eyre::Result<(
    Option<redstone_compiler::transform::place_and_route::global_pnr::ResolvedPhysicalIntent>,
    Option<(String, String)>,
)> {
    let Some(path) = path else {
        return Ok((None, None));
    };
    let source = std::fs::read_to_string(path)?;
    let intent: PhysicalIntent = source.parse()?;
    let resolved = intent.bind(topology)?;
    let file_name = path
        .file_name()
        .and_then(|name| name.to_str())
        .unwrap_or("design.rclayout")
        .to_owned();
    Ok((Some(resolved), Some((file_name, source))))
}

fn emit_intent_source(source: Option<&(String, String)>) -> eyre::Result<()> {
    if let Some((file_name, source)) = source {
        redstone_compiler::snapshot::emit_text(format!("intent/{file_name}"), source.clone())?;
    }
    Ok(())
}

fn snapshot_options_without_source(
    input: &std::path::Path,
    output: &std::path::Path,
) -> (PathBuf, PathBuf, SnapshotOptions) {
    let snapshot_dir = snapshot_output_dir(output);
    let snapshot_archive = snapshot_dir.with_extension("rsnap");
    let design_name = output
        .file_stem()
        .or_else(|| input.file_stem())
        .and_then(|name| name.to_str())
        .unwrap_or("design")
        .to_owned();
    let options = SnapshotOptions::new(&snapshot_dir, design_name);
    (snapshot_dir, snapshot_archive, options)
}

fn snapshot_output_dir(output: &std::path::Path) -> PathBuf {
    if output.extension().and_then(|extension| extension.to_str()) == Some("snapshot") {
        output.to_owned()
    } else {
        output.with_extension("snapshot")
    }
}
