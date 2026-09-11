# Project Roadmap and Tracking

This document is the living plan for evolving Redstone Compiler into a
Minecraft FPGA/ASIC-style backend. It records the overall goal, the current
state, the remaining steps, and the working agreements. Update it whenever a
round lands.

## Overall goal

Turn the repository into a compiler that accepts Verilog/SystemVerilog and
emits a playable Minecraft redstone structure, while preserving the author's
IR, place-and-route, world, NBT, snapshot, and simulator assets.

Target user experience:

```text
redstone build cpu.v   ->   cpu.nbt / cpu.schem   ->   runnable in Minecraft
```

The strategic position (see `redstone_compiler_architecture.md`): the value of
this project is the **IR + P&R + World/NBT + simulation loop**, not the
frontend. Extend around that core; do not replace it.

## Guiding principles

- **IR contract**: `Verilog AST -> LogicalDesign -> RoutableDesign -> P&R ->
  World -> NBT`. Lowering must never reconstruct Verilog or re-parse it.
  Logical IR is target-independent and bus-aware; Routable IR is scalar and
  target-mapped.
- **Preserve working assets**: snapshots (`.rsnap`), the local placer, global
  P&R, World3D/NBT, the redstone simulator, and the viewer keep working.
- **No giant files; comments, tests, and a design note for every new module.**
- **`cargo test` must pass.** Search-heavy tests run with `--release`.
- **Commit every round** with the intent in the commit message body.

## Working agreements / environment

- Toolchain: `nightly-2025-02-08` (`rust-toolchain.toml`).
- This machine has 32 GB RAM / 24 logical CPUs. Build with `-j 1` and run
  tests with `--test-threads=1`; skip the eight search-heavy local placer
  component tests with `--skip test_generate_component` when memory matters.
  Do not run cargo commands concurrently.
- Generated test fixtures under `test/*.snapshot/` are gitignored; the lib
  test target needs `test/counter.snapshot/counter.v` and
  `test/d-flip-flop.snapshot/d-flip-flop.v` to exist (recreate from the
  embedded test sources on a fresh clone).
- New docs go under `docs/` and are linked from `AGENTS.md`.

## Roadmap

Route C from the Phase 0 analysis: strengthen the mapping core first, then
extend the frontend, then build the demos.

### CAD refactor track (branch `cad-refactor`)

The beam-search local placer is the blocking bottleneck (see
`docs/project_status.md`). A CAD-style replacement is designed in
`docs/architecture.md`: force-directed initial placement + simulated
annealing, verified macro library (primitive and logic macros), unified 3D A*
router with pin escape, coarse global routing, PathFinder negotiated
congestion, conflict learning, redstone validation, and a compression ladder.
Milestones are M0 benchmarks, M1 placement IR + macros, M2 router extraction,
M3 placement, M4 negotiated congestion, M5 compression. The IR/mapping/frontend work below is
preserved; the CAD track replaces only the physical search engine behind the
`LayoutCandidate` boundary.

M0-M5 are implemented; see the status log below for per-commit evidence. M0.5
(the memory refactor) is in progress: the copy-on-write `World3D`, the local
work budget, the frontier cap, and the adaptive box are done, which removed the
OOM wall (see `docs/memory_refactor_plan.md`). The `state_next` truth-table
rejection is now explained and confirmed as a missing electrical-exclusivity
check: a NOT input pin (support cobble) was placed adjacent to a foreign power
source, so the pin is driven by `state | ~state = 1`. The fix is a physical
electrical connectivity layer (PECA, `docs/electrical_connectivity_analysis.md`,
M0.10), not a one-off placer patch. The remaining M0.5 commits are the verified
macro library and the validation/attempt clone reduction; `full_adder` is still
unplaceable by the legacy placer.

### Step 1 - General mapper + target capabilities + mapping policy (DONE)

- [x] `src/ir/target.rs`: `TargetSpec`/`TargetOp`/`MappingPolicy`
      (register/adder/mux/xor), capability validation.
- [x] `src/ir/mapping/scalar.rs`: `ScalarNets` bit-blasting, `LeafBuilder`.
- [x] `src/ir/mapping/combinational.rs`: per-bit `Not/And/Or/Xor/Add/Inc/Mux`,
      later `Eq`.
- [x] `src/ir/mapping/sequential.rs`: live-state filtering, per-state
      next-value cones, master/slave DFF decomposition, `negedge`, multiple
      state cells, dead-state elimination.
- [x] `src/ir/mapping/mod.rs`: dispatch, shared leaf/composite writers,
      combinational top assembly, cone partitioning.
- [x] `LogicalDesign::lower_to_routable_with_target(target, policy)` API.
- [x] Tests: policy validation, truth-table lowering, negedge, chained/dead
      registers, PnR topology integration.
- [x] `docs/technology_mapping_design.md`.

### Step 2 - Cell library, physical contracts, Pareto candidates (IN PROGRESS)

Goal: move the local placer from "search on every compile" toward "select from
a reusable library of verified physical candidates".

- [x] Extend `LayoutCandidate` metrics from `block_count + bbox_volume` to a
      metric vector (footprint, height, port count, access points, blocked
      cells) with a minimization-objective view.
- [x] Retain a bounded Pareto frontier in candidate generation
      (`pareto_frontier`, dominance on block count / volume / footprint /
      height) instead of truncating generation order.
- [x] Harden the persistent cache identity: compiler version, target name,
      module shape, and candidate policy are all part of the fingerprint.
- [x] Tests: dominance rules, frontier trade-offs, frontier limit, generated
      candidates contain no dominated pair, fingerprint sensitivity.
- [x] Define a stable cell/recipe model (`*.rcell`-like) separate from
      per-design floorplan intent (see `docs/cell_library_design.md`):
      `CellLibrary`/`CellImplementation`/`CellPhysicalContract`, versioned JSON,
      resolved through `CandidatePolicySet` and covered by the preparation
      fingerprint.
- [x] Consume the contract in P&R: forced input/output diode isolation,
      halo reserved by placement slot sizing and overlap validation, and the
      contract included in the preparation fingerprint and persistent cache
      key. `allowed_transforms` and `max_delay` are recorded but not consumed.
- [x] Make the library usable: `--cell-library` CLI flag, `pnr/cell-library.json`
      snapshot artifact, and restore on `.rsnap` replay before the preparation
      fingerprint check.
- [ ] Named implementation variants of the logical target mapping
      (`std.xor -> xor.nor_network`, ...).
- [ ] Auto-populate built-in library entries for the compiler's known special
      cases and cache candidates by implementation name.

### Step 3 - Routable IR expressiveness

Goal: represent what real designs need before the physical flow.

- [ ] `reset` (sync/async) and `enable` on state cells in Logical IR.
- [ ] Carry reset/enable through Logical-to-Routable decomposition.
- [x] Hierarchy: deterministic flattening for nested hierarchy, mixed
      cells/instances, and vector-port children; the legacy one-level scalar
      structural path is preserved for compatibility.
- [x] Embed `MappingPolicy` in snapshots for reproducible lowering:
      `MappingSpec` JSON (`redstone-compiler.mapping.v1`), `--mapping-policy`
      CLI flag, and `ir/mapping.json` snapshot artifact.
- [ ] Optional: `LoweringMap` provenance object (many-to-many mapping).

### Step 4 - Frontend breadth / optional Yosys bridge

Goal: accept realistic Verilog/SystemVerilog or delegate parsing to Yosys.

- [x] `else`, `case`, multi-statement `begin/end`, `negedge`.
- [x] `==`/`!=`, ANSI headers, bare `reg`, single-bit selects.
- [ ] `parameter`/`localparam` including parameterized widths.
- [ ] Blocking assignments, LHS slices/concatenation, `- * / %`, shifts,
      comparisons.
- [ ] Decision: extend the in-house frontend vs. add a Yosys JSON bridge that
      targets `LogicalDesign`. Revisit after Step 2.

### Step 5 - FSM / Counter / RAM / CPU demos

- [x] FSM vertical slice: registered state + `case` next-state +
      fully-assigned combinational output (lowering + PnR topology test).
- [ ] Counter/FSM full P&R to NBT smoke tests (release, single-thread).
- [ ] RAM inference/mapping and a memory-backed demo.
- [ ] 16-bit RISC CPU demo with `redstone build cpu.v` CLI UX.

## Status log

| Round | Scope | Tests |
| --- | --- | --- |
| Phase 0 | Clone, environment, architecture report, `docs/redstone_compiler_architecture.md` | check + 253 non-heavy tests |
| Step 1 | General mapper, target/policy, dispatch, docs | 270 non-heavy tests |
| Step 1.5 | Cone partitioning (leaves below the 40-node placer limit) | 274 non-heavy tests |
| Step 1.6 | Constants end to end (redstone block / inverter expansion) | 278 non-heavy tests |
| Step 4a | FSM frontend: else/case/multi-statement/negedge, `Eq` operator | 283 non-heavy tests |
| Step 4b | `==`/`!=`, ANSI headers, bit selects | 288 non-heavy tests |
| Step 2a | Candidate metric vector, Pareto frontier, cache identity hardening | 293 non-heavy tests |
| Step 2b | Cell library model, physical contract, JSON round-trip, policy resolution | 298 non-heavy tests |
| Step 2c | Contract consumption: isolation, halo placement, fingerprints | 301 non-heavy tests |
| Step 2d | Library CLI, snapshot embedding, replay restore, halo in parity hash | 302 non-heavy tests |
| Step 3a | Deterministic hierarchy flattening (nested, mixed, vector ports) | 306 non-heavy tests |
| Step 3b | MappingSpec persistence: JSON, CLI flag, snapshot artifact | 308 non-heavy tests |
| CAD-M0 | Benchmark set, baseline metrics, oversized-legacy-leaf dispatch fix | 312 non-heavy tests |
| CAD-M1 | Placement IR + macro model (`MacroTemplate`/`MacroInstance`/`PhysicalNet`/`PinRef`) | 315 non-heavy tests |
| CAD-docs | Documentation audit: `docs/README.md` index, M2 extraction plan, removed two obsolete notes | 315 non-heavy tests |
| CAD-M2.0 | Point-to-point router core extracted into `route_engine/` (state, queue, goal, engine); behavior-equivalent | 315 non-heavy tests |
| CAD-M2.1 | Reverse propagation rules completed for torch, repeater, redstone block, and switch | 319 non-heavy tests |
| CAD-M2.2 | Explicit `RouteCostModel` threaded through the search queue; parity defaults keep the legacy priority | 324 non-heavy tests |
| CAD-M2.3 | `RouteValidator` interface with a simulator-backed implementation; validation moved into `route_engine/validation.rs` | 324 non-heavy tests |
| CAD-M3.0 | Placement IR gains pin facings and structured legality (overlap, bounds) plus bounding-box queries | 328 non-heavy tests |
| CAD-M3.1 | Deterministic initial placement: connectivity-ordered shelf seed, barycenter relaxation, overlap repair | 332 non-heavy tests |
| CAD-M3.2 | Simulated annealing: deterministic RNG, translate/swap/spread moves, Metropolis cooling, weighted cost model | 336 non-heavy tests |
| CAD-M3.3 | Placement benchmark harness over the M0 set with structural macros; all composite benchmarks place legally (fsm_2bit wire 142→115) | 337 non-heavy tests |
| CAD-M3.4 | Real-flow adapter behind `PlacementEngine::{Legacy, Annealed}`; selected candidates convert to macros and back to `PlacedModule`s | 338 non-heavy tests |
| CAD-M3.5 | Engine comparison harness (`MCHDL_BENCH` per-process, bounded config); `not_chain` Legacy 516 ms / Annealed 498 ms; composite full-flow runs deferred (32 GB host OOMs) | 338 non-heavy tests |
| CAD-M4.0 | PathFinder congestion resources: per-cell usage, present overuse, accumulated history, deterministic rip-up queries | 342 non-heavy tests |
| CAD-M4.1 | Congestion-aware search: per-state `extra_cost`, present/history penalties in `RouteCostModel`, empty-map parity and detour tests | 344 non-heavy tests |
| CAD-M4.2 | Negotiated-congestion loop: fold present overuse into history, rip up conflicting nets, reroute until clean or budget | 348 non-heavy tests |
| CAD-M4.3 | Router post-pass behind `GlobalRoutingConfig::pathfinder`: contract-gated reroutes, RCIR/JSON/text persistence, manual full-flow harness | 353 non-heavy tests |
| CAD-M4.4 | Simulator feedback: rejected reroutes add history penalties so later passes avoid the failing corridor | 354 non-heavy tests |
| CAD-M5.0 | Compression ladder: descending box iteration, acceptance bookkeeping, generated box-intent integration with the PnR flow | 362 non-heavy tests |
| CAD-M5.1 | CLI `--compress` wiring for Verilog, Logical RCIR, and Routable RCIR inputs (composite tops; replaces `--intent`) | 362 non-heavy tests |
| CAD-perf-report | `docs/performance_report.md`: memory/compile-performance architecture snapshot for external review | - |
| CAD-M0.5 | Memory architecture refactor (`docs/memory_refactor_plan.md`): Commits 1, 2, 3, 5 done; Commits 6 (macro library) and 7 (validation clones) pending | 362 non-heavy tests |
| CAD-M0.5.1 | `perf` module: world-clone counters, stage guards, RSS sampling, memory budget + `--memory-budget-mb`; `not_chain` shows 47,660 clones / 4.1 GB clone bytes | 362 non-heavy tests |
| CAD-M0.5.2 | Adaptive annealed box (volume-derived, no 64 clamp) and compression ladder now shrinks until failure and keeps the smallest valid box | 362 non-heavy tests |
| CAD-M0.5.3 | Copy-on-write `World3D` (per-layer `Arc`): `not_chain` peak RSS 265→58 MiB, candidate prep 394→153 ms, suite 7.3→3.2 s | 362 non-heavy tests |
| CAD-M0.5.5 | Deterministic local work budget + frontier cap; `full_adder`/`fsm_1bit` fail gracefully in ~66 s instead of OOM; wide-limit control confirms the legacy placer is the wall | 362 non-heavy tests |
| CAD-M0.6 | Truth-table rejection diagnosis: fixed the multi-input sampling explosion (`Some(32)` default; `a & ~b` now compiles); `state_next` still 32/32 truth rejects; `full_adder` unplaceable; Annealed composite reaches routing but fails | 362 non-heavy tests |
| CAD-M0.7 | Annealed routability: spacing/pin-access costs, 4-cell margin, 6-cell channels, multi-seed attempts; composite `andnot` chain routes in 4.7 s / 49 MiB vs Legacy 16.6 s / 73 MiB | 362 non-heavy tests |
| CAD-M0.8 | `state_next` truth-rejection reproducer and bisect: minimal failing subgraph is `Not(Not(state))`; failure matches a lost inversion | 362 non-heavy tests |
| CAD-M0.9 | Physical-connectivity trace (`MCHDL_DEBUG_CONNECTIVITY`): per-candidate endpoint map, signal footprints, and block dump; reproducer labels each variant | 362 non-heavy tests |
| CAD-M0.10a | PECA report-only: shared `world/electrical.rs` rules, pin/terminal provenance from the local placer, component extraction, `Single`/`Merge`/`Passive` contracts; mechanically confirms `node=20 ExtraDriver drivers=[(5, state switch)]` on all 32 `tail_n20` candidates; 368 non-heavy tests | 368 non-heavy tests |
| CAD-M0.10b | PECA enforce: reject violating candidates. Blocked on the OR-tap `Merge` reachability false positive | in progress |

All counts are `cargo test --release --lib -- --skip test_generate_component
--test-threads=1`; the eight search-heavy local placer component tests are
excluded on this machine for memory reasons and must be run on a larger box.

## Known gaps and risks

- Constants are materialized but not folded (`and(x, 1)` stays as logic).
- `world_to_logic` does not map redstone blocks back to constant nodes, so
  verifier round-trips on constant designs are unavailable.
- One combinational leaf per state cell before partitioning; shared
  sub-cones are duplicated across state cells.
- Local placer is still the scalability and correctness bottleneck: 40-node
  leaves are expensive, `full_adder` places nothing even with a wide budget,
  and `state_next` produces 32 candidates that all fail the truth-table check
  (minimal failing subgraph `Not(Not(state))`, M0.8). Fixing or replacing the
  leaf realizer is the next high-value step.
- Global PnR only supports leaf children.
- `Piston` is a stub across NBT export and the simulator.
