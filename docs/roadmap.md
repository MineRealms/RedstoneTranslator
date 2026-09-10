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

### Step 2 - Cell library, physical contracts, Pareto candidates (NEXT)

Goal: move the local placer from "search on every compile" toward "select from
a reusable library of verified physical candidates".

- [ ] Define a stable cell/recipe model (`*.rcell`-like) separate from
      per-design floorplan intent (see `physical_design_intent.md`).
- [ ] Extend `LayoutCandidate` metrics from `block_count + bbox_volume` to a
      metric vector (footprint, height, port access, blockage, delay).
- [ ] Retain a bounded Pareto frontier instead of a single best candidate.
- [ ] Add physical-contract fields: access points, halo, legal transforms.
- [ ] Cache verified candidates by target + implementation + recipe + compiler
      version; wire the cache into `UnitCandidateConfig`.
- [ ] Tests: dominance/Pareto filtering, cache identity, candidate reuse.

### Step 3 - Routable IR expressiveness

Goal: represent what real designs need before the physical flow.

- [ ] `reset` (sync/async) and `enable` on state cells in Logical IR.
- [ ] Carry reset/enable through Logical-to-Routable decomposition.
- [ ] Shallow hierarchy flattening beyond the current one-level leaf
      children, or explicitly support composite children in global PnR.
- [ ] Embed `MappingPolicy` in RCIR/snapshots for reproducible lowering.
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

All counts are `cargo test --release --lib -- --skip test_generate_component
--test-threads=1`; the eight search-heavy local placer component tests are
excluded on this machine for memory reasons and must be run on a larger box.

## Known gaps and risks

- Constants are materialized but not folded (`and(x, 1)` stays as logic).
- `world_to_logic` does not map redstone blocks back to constant nodes, so
  verifier round-trips on constant designs are unavailable.
- One combinational leaf per state cell before partitioning; shared
  sub-cones are duplicated across state cells.
- Local placer is still the scalability bottleneck (40-node leaves, expensive
  search); Step 2 targets exactly this.
- Global PnR only supports leaf children.
- `Piston` is a stub across NBT export and the simulator.
