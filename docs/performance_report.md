# Memory and Compile-Performance Architecture

Status snapshot for external review. Branch `cad-refactor` at commit `1a15e09`.
Test baseline: 359 passed / 0 failed (release, `-j1`, single thread, eight heavy
`test_generate_component_*` tests skipped). Host: Windows, 32 GB RAM, 24 logical
cores. This document describes the current implementation only; proposed
optimizations are listed separately and are not implemented.

## 1. Compilation pipeline

```text
Verilog -> LogicalDesign (RCIR) -> RoutableDesign (RCIR)
        -> PreparedPnrDesign (per-child LayoutCandidate sets)
        -> Global PnR (placement attempts x routing attempts)
        -> World3D -> NBT
```

Entry chain (`src/main.rs`): `compile_verilog_input` ->
`LogicalDesign::from_verilog_source_named` -> `lower_to_routable_with_target` ->
`place_and_route_logical_design_with_mapping` ->
`place_and_route_routable_design_with_visualization` ->
`prepare_routable_design_for_global_pnr` -> `run_prepared_pnr_with_visualization`.

Key files: frontend/IR `src/verilog/`, `src/ir/`; candidate generation
`global_pnr/candidate.rs`, `local_placer/`; global PnR
`global_pnr/{mod,placer,router}.rs`; new engines `route_engine/`,
`sa_placer.rs`, `compression.rs`; world/simulator `src/world/`.

## 2. Core data structures

| Structure | Contents | Memory notes |
| --- | --- | --- |
| `Block` | `{ kind: BlockKind, direction }` | Largest variant `Repeater { is_on, is_locked, delay: usize, lock_input1/2: Option<GraphNodeId> }` gives roughly 64 B; not packed |
| `World3D` | `{ size, map: Vec<Vec<Vec<Block>>> }` (`map[z][y][x]`) | `clone()` is a deep copy; 64x64x16 = 65,536 cells is about 4 MB per copy plus 1000+ small `Vec` allocations |
| `World` (simulator) | `blocks: Vec<(Position, Block)>` | `From<&World3D>` copies everything |
| `iter_block()` | returns `Vec<(Position, Block)>` of all non-air blocks | allocates on every call; used on routing hot paths |
| `LayoutCandidate` | `world + bbox + ports + occupied_cells/blocked_cells (HashSet) + cost + halo` | one candidate carries one full world |
| `PreparedPnrDesign` | all candidates of all children | lives for the whole run |
| `ChildCandidateCache` | `{ key, Vec<LayoutCandidate> }` | also lives for the whole run; the disk cache exists but the in-memory index keeps every candidate |
| `RouteSearchState` | `{ world: World3D, terminal, route, signal_strength, powered_taps, pending_bounds, extra_cost }` | every search state embeds a full world copy |

Defaults: `UnitCandidateConfig { dim 16x16x6, max_candidates 16 }`;
`GlobalPlacementConfig { spacing 2, shelf_width 64, max_attempts 16 }`;
`GlobalRoutingConfig { AStar, Incremental, pathfinder: None }`;
`GlobalSearchConfig` uses the Balanced budget
`{ candidates/child 4, layout combinations 16, detailed attempts 16, refined 4, rounds 2 }`.

## 3. Stage behavior

### 3.1 Frontend and lowering (light)

Parsing, mapping, and partitioning are linear in design size. The M0
partitioning fix keeps every leaf at 40 prepared nodes or fewer.

### 3.2 Local candidate generation (heavy: time and memory)

`generate_routable_module_candidates_with_progress_label` ->
`graph_from_routable_leaf` -> `prepare_place` -> beam/sampling search in the
local placer, producing one world per candidate.
`candidate_config_for_routable_child` enables `Random(512)` step sampling and
larger step limits for multi-input combinational children, so the branching
factor is large. The search clones worlds in several places
(`local_placer/routing.rs`). This is the main source of OOM/timeouts for
leaf-top cases such as `full_adder` (27 prepared nodes) and `random_10`.

### 3.3 Global placement (light)

Shelf/free_3d produce `Vec<Vec<PlacedModule>>`; up to 16 placement attempts per
layout combination. The new SA placer (`sa_placer.rs`) searches over coordinates
and is memory-cheap, but `PlacementEngine::Annealed` (default `Legacy`) only
re-places candidates that the legacy local placer already generated, so it does
not reduce candidate-generation cost.

### 3.4 Global routing (heavy)

`route_first_successful_placement` multiplies placement attempts by net-order
strategies by routing stages (probe/routing/refinement). Each attempt routes net
by net. In `route_engine/queue.rs`: A* is capped at 2000 expansions, GreedyBeam
uses a configurable width (128 in tests, max expansions 4096), and
`GLOBAL_ROUTE_MAX_STEPS = 128`. Every queued state embeds a full world, so a
single net frontier can reach gigabytes at 4 MB per world. Each expansion calls
`detailed_router::place_*`, which clones the world again. Incremental validation
settles each route with the simulator (`reset_dynamic_power_states`,
`initialize_redstone_states`, `Simulator::from_preserving...`), adding roughly
one to two world copies per route. The M4 negotiation post-pass
(`GlobalRoutingConfig::pathfinder`, off by default) adds only a
`CongestionMap` (`HashMap<Position, usize>`); reroutes use the same engine.

### 3.5 Assembly and NBT (linear)

`assemble_world` collects candidate blocks and rebuilds the world; `to_nbt()`
is proportional to the final volume and is usually not the peak.

### 3.6 Extra amplification

`ranked_candidate_pools` clones candidates into `ChildCandidatePool` while
`prepared` still owns the originals, so candidate memory is roughly doubled.

## 4. Measurements on the 32 GB host

| Scenario | Result |
| --- | --- |
| `not_chain` (leaf top, 3 prepared nodes), end to end | about 0.5 s, fine |
| `full_adder` (leaf top, 27 nodes), full flow with a bounded config | process OOM (mimalloc abort, 0xc0000409) |
| `fsm_1bit` (4 leaves, 13 nodes), full flow with the Legacy engine | did not finish within 20 minutes (aborted) |
| `random_10` (2 leaves, 27 nodes), candidate generation | 148 s and no candidate produced |
| Eight `test_generate_component_*` tests (DFF/counter class) | OOM |
| Full `cargo test --release` | OOM; the filtered suite passes 359 |

The OOM is a hard abort, not a graceful error.

## 5. Amplification factors, by contribution

1. Search states embed deep `World3D` copies (routing frontier up to 2000 or beam width 128; local placer similar).
2. Deep `World3D` copies are expensive (nested `Vec`, 64 B per `Block`).
3. Attempt multiplication (placement x net order x routing stages).
4. `PreparedPnrDesign` plus `ChildCandidateCache` plus `ranked_candidate_pools` keep all candidate worlds resident (about 2x).
5. `iter_block()` and `World::from` re-allocate/copy on hot paths.
6. Validation and the simulator clone worlds.
7. The initial box is large (the compression ladder starts at 64x64x16).
8. Blocks are unpacked and the nested `Vec` layout fragments allocations.

## 6. Optimization directions not yet implemented

- A. Tight initial box plus adaptive search budgets (flow level, small change).
- B. `World3D` copy-on-write or chunked storage (`Arc<Chunk>`): `clone()` becomes O(chunks), writes copy one chunk.
- C. Flat, packed world (`Vec<u64>` or 8 B `Block`) for a constant-factor win.
- D. Delta/undo-log routing states instead of embedded worlds.
- E. Streaming/disk-backed candidates (keep only top-K or keys in memory).
- F. In-place simulator validation with state snapshot/restore instead of cloning.
- G. Replace leaf candidate generation with SA/macro placement (also fixes exponential time).

## 7. Constraints and review questions

Constraints: IR/snapshot/NBT compatibility must not break; default behavior must
stay identical (359-test baseline); every new engine sits behind a flag
(`PlacementEngine::Annealed`, `GlobalRoutingConfig::pathfinder`, `--compress`);
the host is a single 32 GB machine and builds must use `-j1`/`-j2`.

Questions for review:

1. Priority: is B (copy-on-write) or C (packed storage) the better first step, or should they merge into "chunked and packed chunks"?
2. Which is less invasive and safer for the current code: D (delta states) or B (copy-on-write)?
3. Should the local placer be replaced by the SA/macro path rather than repaired further?
4. Is a global memory-budget scheduler (per-stage caps plus graceful failure) worth building?
5. Is converting `iter_block`/`World::from` to borrowed iterators a good low-risk first win?
6. For a 32 GB target, what design size should realistically be compilable?
