# Redstone Compiler Project

Toolkit for Redstone Transfer Level Design: compile Verilog/SystemVerilog into
a Minecraft redstone structure (NBT) through a CAD-style flow.

## Status

- Frontend, Logical/Routable IR, local placement, global P&R, simulation, and
  NBT export are in place. CAD-style placement (simulated annealing), the
  PathFinder routing post-pass, and the compression ladder are implemented
  behind flags; the legacy engines remain the default.
- Current work is the **Electrical Legality Filter** (PECA): candidate
  generation is being moved from "geometrically placeable" to "electrically
  legal" (`docs/electrical_connectivity_analysis.md`).
- Known limits: simple combinational designs compile end to end; `full_adder`
  and `fsm_1bit` currently produce no placement; hierarchical P&R supports
  leaf children of the top module; the simulator is the verification oracle
  and is not a vanilla-Minecraft equivalence test. Details in
  `docs/roadmap.md` and `docs/project_status.md`.

## How to use

```text
cargo run --release --bin redstone-compiler -- input.v out.nbt       # Verilog
cargo run --release --bin redstone-compiler -- input.rcir out.nbt    # Logical/Routable RCIR
cargo run --release --bin redstone-compiler -- input.rsnap out.nbt   # replay a prepared P&R snapshot
```

The second argument names the **snapshot output**. The compiler writes a
directory `out.snapshot/` (IR, PnR config, candidates, instances, routes, and
the final world `out.snapshot/out.nbt`) plus the archive `out.rsnap`. Pass a
path ending in `.snapshot` to control the directory name exactly; the final
NBT is always `<snapshot-dir>/<design>.nbt`.

Input kinds are selected by extension: `.v` (Verilog subset), `.rcir`
(Logical or Routable RCIR), `.rsnap`/`.snapshot` (replay prepared P&R).

| Flag | Effect |
| --- | --- |
| `--intent design.rclayout` | per-design floorplan and routing intent |
| `--cell-library library.json` | reusable cell library applied to candidate policies |
| `--mapping-policy mapping.json` | lowering target and mapping policy |
| `--candidate-cache dir` | reuse structurally identical local-placement candidates |
| `--compress` | run the compression ladder (replaces `--intent`; composite tops) |
| `--placement-engine legacy\|annealed` | global placement engine (default `legacy`) |
| `--memory-budget-mb N` | fail with an error instead of an OOM abort |

Environment variables (see `AGENTS.md` for details): `MCHDL_PERF=1` prints
per-stage clone counters and RSS; `MCHDL_DEBUG_TRUTH_TABLE`,
`MCHDL_DEBUG_PECA`, `MCHDL_DEBUG_ANNEALED`, `MCHDL_DEBUG_PLACEMENT`,
`MCHDL_DEBUG_INPUT_SWITCH`, and `MCHDL_DEBUG_CONNECTIVITY` print diagnostics;
`MCHDL_PECA_ENFORCE=1` enforces `Single` electrical violations; `MCHDL_BENCH`
runs one manual full-flow benchmark per process.

Run the tests with `cargo test --release` (see `AGENTS.md` for the
memory-constrained subset). Benchmarks live in `test/benchmarks/`.

## Viewer

`tools/nbt-viewer` is a local web viewer (Vite + TypeScript + three.js) for
the compiled NBT files. From that directory:

```powershell
npm.cmd install
npm.cmd run prepare:mcmeta   # block assets (falls back to downloading from GitHub)
npm.cmd run dev              # http://127.0.0.1:5173
```

Use "Open NBT" or "Open Folder" to load e.g. `out.snapshot/out.nbt` or the
per-candidate files under `out.snapshot/candidates/`. See
`tools/nbt-viewer/README.md` for details (including the optional Rust
simulator WASM build).

## Compiler Stack

```text
Verilog/SystemVerilog -> Logical IR (RCIR) -> Routable IR (RCIR) -> Place and Route -> World3D -> NBT
```

- Frontend and Logical IR: parse and elaborate the Verilog subset, preserve
  bus width and state intent (`docs/verilog_rtl_interface_design.md`).
- Routable IR: target-mapped scalar netlist with explicit ports, nets, and
  net classes (`docs/intermediate_representation_design.md`).
- Place and Route: local candidate generation plus global placement/routing.
  The CAD-style flow is implemented (placement IR and macros, deterministic
  seed + simulated annealing, extracted A* router, PathFinder negotiated
  congestion, compression ladder); new engines sit behind flags
  (`--placement-engine annealed`, `GlobalRoutingConfig::pathfinder`,
  `--compress`). Design notes in `docs/architecture.md`.
- Electrical legality: PECA derives redstone components and their driving
  terminals from the placed world and checks pin contracts before the
  simulator (`docs/electrical_connectivity_analysis.md`).
- World, World3D: `World` and `World3D` are collections of blocks and
  positions, designed to correspond exactly to the Minecraft world. See
  [world/mod.rs](https://github.com/Redstone-Compiler/redstone-compiler/blob/master/src/world/mod.rs).
- NBT: a blueprint format that can be imported into Minecraft using
  [MCEdit](https://www.mcedit.net/),
  [Litematica](https://www.curseforge.com/minecraft/mc-mods/litematica) or
  similar.

## Documentation

Start at `docs/README.md` (index). The most useful entries:

- `docs/roadmap.md` — living plan and per-commit status log.
- `docs/project_status.md` — done / not done / precise blockers.
- `docs/architecture.md` — CAD migration design and milestone status.
- `docs/electrical_connectivity_analysis.md` — PECA / Electrical Legality Filter.
- `docs/memory_refactor_plan.md` — memory architecture work and measurements.
- `docs/performance_report.md` — memory and compile-performance snapshot.
