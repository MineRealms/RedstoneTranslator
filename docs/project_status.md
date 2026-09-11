# Project Status Report

> Branch: `cad-refactor` · Base commit: `15294b3` · Last update: `53dd417` (M0.8)
> Date: 2026-09-10
>
> This report is the hand-off snapshot before the CAD-style placer refactor.
> It lists what is complete, what is not, and the precise problem the refactor
> must solve. The migration plan lives in `docs/architecture.md`; current
> progress lives in `docs/roadmap.md`, and the memory refactor plan lives in
> `docs/memory_refactor_plan.md`. Sections 1-2.4 describe the pre-refactor
> baseline; section 2.5 tracks the refactor.

## 1. Environment

- Toolchain: `nightly-2025-02-08` (`rust-toolchain.toml`).
- Machine: 32 GB RAM, 24 logical CPUs. Build with `-j 1` or `-j 2`; run tests
  with `--test-threads=1`; skip the eight search-heavy local placer component
  tests (`--skip test_generate_component`) on this machine.
- A release rebuild of the crate takes ~2-3 minutes with `-j 2`.
- Fresh-clone bootstrap: `src/ir/debug.rs` uses `include_str!` on
  `test/*.snapshot/*.v`, which are gitignored. Recreate them from the embedded
  test sources before compiling the lib test target.

## 2. What is complete

### 2.1 IR and lowering (Logical -> Routable)

- `LogicalDesign` / `RoutableDesign` RCIR text formats with deterministic
  round-trips and validation (`src/ir/logical.rs`, `routable.rs`, `text.rs`,
  `logical_text.rs`).
- `LogicalCellKind`: Buffer, Not, And, Or, Xor, Eq, Add, Inc, Mux, DLatch,
  Dff, Register.
- `LogicalValue`: Net, Slice (single-bit select), Constant.
- General mapper `src/ir/mapping/`: bit-blasting, per-bit combinational
  expansion, ripple-carry Add/Inc, Mux and Eq expansion, state decomposition
  (master/slave latches, negedge, multiple state cells, dead-state
  elimination), constants (true = redstone block, false = inverter), cone
  partitioning.
- `TargetSpec`/`TargetOp`/`MappingPolicy` and the serializable
  `MappingSpec` (`redstone-compiler.mapping.v1`), CLI `--mapping-policy`,
  snapshot artifact `ir/mapping.json`.
- Deterministic hierarchy flattening for nested hierarchy, mixed
  cells/instances, and vector-port children (`src/ir/flatten.rs`); one-level
  scalar hierarchy keeps its structural Routable modules.

### 2.2 Verilog frontend

- Modules, ANSI headers, `input/output/reg/wire`, bare `reg`.
- `assign` expressions: `~ & ^ | +`, `==`, `!=`, parentheses, decimal
  constants, single-bit selects, named instance connections.
- `always @(*)`, `always @(posedge clk)`, `always @(negedge clk)`.
- Multi-statement `begin/end`, `if`/`else` with expression conditions,
  `case`/`default` with multiple patterns.
- Multi-assignment clocked processes (one state cell per signal, later
  assignments win, unassigned paths hold).
- Fully assigned combinational processes lower to pure logic; partial
  assignments use the legacy D-latch pattern.

### 2.3 Physical design

- `LayoutCandidate` with a metric vector (blocks, volume, footprint, height,
  ports, access points, blocked cells), `placement_bbox()` halo, Pareto
  frontier retention, cache identity v3 (compiler version + target + shape +
  policy + contract).
- Cell library model (`CellLibrary`, `CellImplementation`,
  `CellPhysicalContract`, JSON `redstone-compiler.cell-library.v1`), CLI
  `--cell-library`, snapshot embedding (`pnr/cell-library.json`) and restore
  on `.rsnap` replay.
- Contract consumption: forced input/output diode isolation and halo-aware
  placement slot sizing/overlap checks.
- Global PnR: preparation boundary, Free3D/shelf/grid/register placement
  heuristics, A*/GreedyBeam routing with rip-up refinement, physical intent.
- World3D, structure NBT, simulator, snapshots, viewer.

### 2.4 Verification

- The non-heavy suite currently passes 362 tests in release (single thread,
  `--skip test_generate_component`; 8 heavy component tests and 10 diagnostic
  tests skipped). The 311 figure in the original snapshot predates M2-M5.
- FSM-class designs lower end to end to a `ResolvedPnrTopology`
  (`ir::mapping::tests::fsm_with_case_and_combinational_output_lowers_to_a_pnr_topology`).
- Constant folding in `prepare_place`
  (`src/transform/logic/fold_constants.rs`) removes identity/absorbing
  operations and constant-fed Or patterns.

### 2.5 CAD refactor progress (branch `cad-refactor`)

- **M0 (commit `40adb27`)**: benchmark set in `test/benchmarks/` plus a
  lowering metrics harness (`src/ir/benchmarks.rs`) that writes
  `target/benchmark-baseline.json`. The benchmarks immediately exposed a real
  bug: flat scalar modules bypassed partitioning, so `dense_or_cone` lowered to
  a single 100-node leaf. The dispatch now falls back to the general
  partitioned mapper when a legacy leaf would exceed the placer limit.

  | Benchmark | Leaves | Max prepared nodes |
  | --- | --- | --- |
  | `not_chain` | 1 | 3 |
  | `full_adder` | 1 | 27 |
  | `dense_or_cone` | 4 | 28 (was 100) |
  | `fsm_1bit` | 4 | 13 |
  | `fsm_2bit` | 6 | 29 |
  | `random_10` | 2 | 27 (was 44) |
  | `random_40` | 11 | 36 |

- **M1 (commit `79b6ff6`)**: `placement_ir.rs` with `MacroTemplate`
  (normalized verified layout, pins, escape cells, forbidden cells, halo,
  rotation set), `MacroInstance`, `PinRef`, `PhysicalNet`, and
  `PlacementProblem` (composition, instantiation into `World3D`, HPWL).
- **M2-M5 (commits `1de49c4`..`1a15e09`)**: router core extraction with reverse
  propagation, cost model, and simulator-backed validator (M2); placement IR
  facings/legality, deterministic initial placement, simulated annealing, and
  the flow adapter behind `PlacementEngine::{Legacy, Annealed}` (M3);
  PathFinder congestion resources, congestion-aware search, negotiated loop,
  router post-pass, and simulator feedback behind
  `GlobalRoutingConfig::pathfinder` (M4); compression ladder and CLI
  `--compress` (M5). Per-commit evidence is in `docs/roadmap.md`.
- **M0.5 (in progress, commits `c7a75ed`..`53dd417`)**: memory instrumentation,
  memory/work budgets, copy-on-write `World3D`, adaptive placement box, and
  the multi-input sampling fix. `not_chain` peak RSS dropped from 265 to
  58 MiB; `full_adder`/`fsm_1bit` now fail gracefully with a work-limit error
  instead of an OOM abort; the annealed placement engine is routable on a
  composite `andnot` chain (4.7 s / 49 MiB versus Legacy 16.6 s / 73 MiB).
  Remaining: the verified macro library and validation/attempt clone
  reduction. Details in `docs/memory_refactor_plan.md`.
- **M0.9-M0.10 (in progress)**: the `state_next` truth-table rejection is
  confirmed as a missing electrical-exclusivity check (a NOT input pin placed
  adjacent to a foreign power source). The fix is the PECA layer
  (`docs/electrical_connectivity_analysis.md`), not a one-off placer patch.
  `full_adder` still produces no placement even with a wide budget.

## 3. What is not complete

- **Local placement/routing reliability** (the blocking problem, see §4).
- Full P&R of the two-bit FSM; the ignored one-bit FSM smoke currently fails
  during `state_next` placement.
- Step 2 leftovers: named implementation variants of target mapping
  (`std.xor -> xor.nor_network`), recipe constraints (port faces, corridors,
  objectives), auto-populated library entries.
- Step 3 leftovers: reset/enable as first-class IR state pins (synchronous
  reset works through Mux lowering), `LoweringMap` provenance.
- Frontend leftovers: `parameter/localparam`, blocking assignments, LHS
  slices/concatenation, arithmetic beyond `+`, shifts/comparisons.
- `Piston` is a stub in NBT export and the simulator; `world_to_logic` does
  not map redstone blocks back to constants.
- Global PnR supports only leaf children of the top module.

## 4. Current problem (precise statement)

### 4.1 Symptom

`LocalPlacer` (`src/transform/place_and_route/local_placer/`) is a beam search
that places gates of one combinational leaf in topological order:

1. For each node, enumerate every placement/route candidate for every state in
   the current beam.
2. Compact duplicate states.
3. Sample the beam down to a fixed width (e.g., `SamplingPolicy::Random(32)`).
4. After the last node, return the surviving states as candidates.

On moderate cones it returns zero candidates, or crashes, at an intermediate
node. Concrete evidence:

- Two-bit FSM next-state cone (`state_next_p0`, 30 prepared nodes, four Or
  merges): the beam was 32 states at step 14; at step 15 (an Or route) **all
  32 states failed to route** and the queue became empty for the remaining 15
  steps. Raising the beam to 64, `max_route_step` from 4 to 16, and route
  sampling to `Random(64)` did not help (11 minutes, still zero candidates).
- Partitioning the cone into smaller chunks (prepared-size target 16) let some
  chunks place (64 candidates) while others still returned zero
  (`state_next_p2`).
- One-bit FSM next cone after constant folding (13 prepared nodes) still
  failed/crashed during placement.

### 4.2 Why it fails (analysis)

- **Greedy commitment without backtracking**: a placement that looks locally
  good (compact, short route) can consume the only space a later Or merge
  needs. Beam search only keeps width-N states and never repairs.
- **Routing and placement are interleaved per gate**: each gate's route is
  fixed before later gates exist, so the router cannot reroute a net when a
  later gate blocks it.
- **Sampling loses diversity**: random sampling keeps N states but can drop
  the only state with free space exactly where the next gate needs it.
- **Cost is local**: compactness plus Manhattan distance to future consumers,
  with no congestion or escape-routing estimate.
- **No rip-up**: once a route is placed it is never removed.

### 4.3 Problem statement for an external expert

> I have a Minecraft redstone "place and route" compiler. One compilation unit
> is a small combinational netlist (roughly 10-40 gates: NOT torches, OR as a
> redstone merge, XOR/AND decomposed into NOT/OR, plus constant redstone
> blocks). Gates are macros with fixed 3D shapes (a torch on a support block,
> an OR merge as a redstone T-junction with specific geometry) that must be
> placed into a bounded 3D grid (e.g. 16x16x6) and wired with redstone.
> Routing has unusual constraints: wire signal strength decays over 15 blocks
> so repeaters are needed; torches and repeaters are directional; each cell
> holds one block; two different nets must never touch; some blocks are
> forbidden routing cells.
>
> The current placer does a beam search: place gates in topological order, at
> each step enumerate all placement/route candidates from every beam state,
> then sample down to a fixed beam (32). On dense cones (e.g. 26-30 graph
> nodes with several multi-input OR merges) the beam dies: at some step every
> sampled state fails to route the next gate, so the search returns zero
> candidates. Increasing the beam to 64, route depth from 4 to 16, and route
> sampling did not help; splitting the cone into smaller chunks helps some
> chunks but others still fail.
>
> What are the standard robust algorithms for this class of problem? I am
> considering: (1) simulated-annealing / force-directed placement followed by
> global + detailed routing with PathFinder-style negotiated congestion and
> rip-up-and-reroute, (2) conflict-directed backtracking with conflict
> learning, (3) precomputed verified macro layouts for common gates (AND, XOR,
> full adder) composed hierarchically, (4) routability-driven placement with
> congestion estimates. Which is most appropriate for a grid-based,
> directionality- and decay-constrained, small-block problem, and are there
> specific papers or algorithms for "routability-driven placement" and
> "escape routing" that apply? Note that density is not the objective;
> routing success rate and determinism are.

## 5. Repository map (where things live)

| Area | Path |
| --- | --- |
| Verilog frontend | `src/verilog/` |
| Logical/Routable IR | `src/ir/` (mapping in `src/ir/mapping/`, flatten in `src/ir/flatten.rs`) |
| Graph transforms | `src/graph/`, `src/transform/logic/` |
| Local placer + local router | `src/transform/place_and_route/local_placer/` |
| Global PnR | `src/transform/place_and_route/global_pnr/` |
| World, blocks, simulator | `src/world/` |
| NBT | `src/nbt/` |
| Snapshots | `src/snapshot.rs`, `global_pnr/prepared_snapshot.rs` |
| Cell library | `global_pnr/cell_library.rs` |
| Docs | `docs/` (roadmap first) |

## 6. Commit history on master (this work)

```
15294b3 Fold constants before placement and fix local placer edge cases
90b4f2b Persist the lowering mapping spec in CLI and snapshots
1b7afc2 Flatten nested and vector-port hierarchy before lowering
f161078 Make the cell library usable from the CLI and snapshots
f3b1e2d Consume cell contracts in candidate generation and placement
bee4efa Add reusable cell library model and physical contracts
4ed378f Add Pareto candidate metrics and harden candidate cache identity
84c89d8 Add general logical-to-routable mapper, constants, and FSM frontend
87274ac Add architecture analysis, mapping design, and project roadmap
```
