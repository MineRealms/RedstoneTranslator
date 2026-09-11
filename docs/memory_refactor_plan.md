# Memory Architecture Refactor (M0.5)

**Status**: in progress · **Branch**: `cad-refactor` · **Baseline**: 362 passed / 0 failed
(release, `-j1`, `--skip test_generate_component --test-threads=1`)

**Related**: `performance_report.md` (current-state facts), `architecture.md`
(CAD target), `roadmap.md` (milestones).

## 1. Why

Measured on the 32 GB host:

| Scenario | Result |
| --- | --- |
| `not_chain` (leaf top, 3 prepared nodes) | about 0.5 s, fine |
| `full_adder` (leaf top, 27 nodes) | OOM before global routing (`run_prepared_leaf`) |
| `random_10` (2 leaves) | candidate generation 148 s, no candidate |
| `fsm_1bit` (4 leaves) | more than 20 minutes, aborted |
| Eight `test_generate_component_*` tests | OOM |
| Full `cargo test --release` | OOM (mimalloc abort, 0xc0000409) |

Root cause: both search frontiers retain full `World3D` snapshots.

- `PlacerQueue = Vec<(World3D, PlacementState)>` (`local_placer/mod.rs:58`), with
  `LEAK_SAMPLING_QUEUE_THRESHOLD = 10_000` (`local_placer/mod.rs:62`).
- `RouteSearchState { world: World3D, .. }` (`route_engine/state.rs`), A* capped
  at 2000 expansions (`route_engine/queue.rs:11`).

The first wall in practice is local candidate generation, not global routing:
`full_adder` is a leaf top and never reaches the router.

Core principle: search states must be placement/routing decisions (deltas), not
Minecraft world snapshots. This is the incremental state representation that
EDA tools have and this project currently lacks.

## 2. Goals and non-goals

Goals:

- Bound the memory of both search frontiers by representation, not by luck.
- Turn OOM into a clear, deterministic budget error.
- Preserve byte-identical outputs and determinism.

Non-goals (deferred until after this plan):

- `World3D` copy-on-write or packed `Block` storage.
- Physical/Electrical/Simulation layer split.
- Replacing the local placer with placement-first search.
- PathFinder, SA tuning, compression changes.

## 3. Priority summary

| Item | Benefit | Risk | Order |
| --- | --- | --- | --- |
| Memory instrumentation + graceful budget | high | low | 1 |
| Adaptive initial box | high | low | 2 |
| Local placer `World3D` to delta | high | medium | 3 |
| Route state `World3D` to delta | high | medium | 4 |
| Deterministic local work budget | high | medium (behavior) | 5 |
| Verified macro library (fingerprints) | high | low-medium | 6 |
| Validation/attempt clone reduction | medium | medium | 7 |
| Candidate lazy materialization | medium | medium | later |
| COW, packed blocks, layering | long-term | high | later |

## 4. Execution tracker

### [x] Commit 1 — perf: memory instrumentation and graceful budget

Scope: manual `impl Clone for World3D` with atomic clone count and copied bytes;
stage guards (candidate preparation, layout search, routing attempts) with
per-stage deltas and RSS; process working-set sampling via the Windows API;
deterministic memory budget that returns an error instead of aborting; CLI
`--memory-budget-mb` and `MCHDL_PERF=1` verbose output.

Files: `src/perf.rs`, `src/world/mod.rs`, `global_pnr/{mod,candidate}.rs`,
`local_placer/mod.rs`, `route_engine/engine.rs`, `src/main.rs`.

Acceptance met: no behavior change (362 non-heavy tests green);
`--memory-budget-mb 1` on `not_chain` fails with
`memory budget exceeded during candidate preparation: RSS 6 MiB > budget 1 MiB`
and exit code 1 instead of an abort.

Evidence (`MCHDL_PERF=1`, `not_chain`): candidate preparation 394 ms,
**47,660 world clones, 4.1 GB of cloned world bytes**, 16.6 MB allocated,
peak RSS 265 MiB. This confirms that even the smallest benchmark explodes
through world cloning in the local placer, and that Commit 3 is the first big
target.

### [ ] Commit 2 — perf: adaptive initial box

Scope: stop clamping the annealed adapter to `shelf_width` 64; derive the box
from total candidate volume plus a safety margin; start the compression ladder
from the tight estimate and grow on failure.

Files: `global_pnr/annealed.rs`, `compression.rs`.

Acceptance: small designs start near their volume-derived box; non-heavy suite green;
measured world bytes drop.

### [ ] Commit 3 — refactor(local): placement queue world to delta

Scope: `PlacerQueue` entries store a cumulative block delta instead of
`World3D`; materialize one scratch world per popped branch; keep electrical
initialization semantics (`initialize_redstone_states` after materialization).

Files: `local_placer/{mod,state,routing,sequential/*}.rs`.

Acceptance: candidate output byte-identical to the baseline on snapshot tests;
`full_adder` candidate generation completes within budget; non-heavy suite green.

### [ ] Commit 4 — refactor(router): route state world to delta

Scope: `RouteSearchState` stores a cumulative delta (the diff that
`added_route_blocks` already computes); materialize one scratch world per pop for
`detailed_router` calls and validation.

Files: `route_engine/{state,engine}.rs`.

Acceptance: A* parity tests unchanged; existing router tests green; frontier
bytes drop by the predicted factor.

### [ ] Commit 5 — perf(local): deterministic work budget and frontier cap

Scope: explicit caps (queue length, sampled states, total work) with a
documented truncation policy; graceful failure when exceeded.

Acceptance: `fsm_1bit` finishes or fails with a clear budget error; the behavior
change is documented in `roadmap.md` and benchmarked.

### [ ] Commit 6 — perf: verified macro library via fingerprints

Scope: reuse `routable_candidate_shape_fingerprint` and the disk
`candidate_cache` to ship or precompute canonical leaf macros; skip local search
on hits.

Acceptance: repeated leaf shapes hit the cache; candidate generation time drops
on counter/FSM-style designs.

### [ ] Commit 7 — perf: validation and attempt clone reduction

Scope: `validation.rs` world clones (three sites) become snapshot/restore of
dynamic state; avoid rebuilding `assemble_world`/`placed_candidate_world` per
attempt where possible; reuse negotiation assembly.

Acceptance: attempt peak bytes drop; non-heavy suite green.

## 5. Acceptance red lines

- Commits 3 and 4 must keep candidate/route outputs byte-identical (snapshot
  diff plus the non-heavy test baseline).
- Every commit keeps the non-heavy suite green:
  `cargo test --release --lib -j 1 -- --skip test_generate_component --test-threads=1`.
- Builds use `-j 1` or `-j 2`; full-flow benchmarks stay off the 32 GB host.
- Behavior changes are allowed only in Commit 5 and must be recorded in
  `roadmap.md`.

## 6. Code-fact appendix

| Fact | Location |
| --- | --- |
| `PlacerQueue = Vec<(World3D, PlacementState)>` | `local_placer/mod.rs:58` |
| Queue threshold 10,000 entries | `local_placer/mod.rs:62` |
| Local placer world clones | `local_placer/routing.rs:66,125,220,270`; `sequential/macro_routes.rs:58` |
| `RouteSearchState.world` | `route_engine/state.rs` |
| A* cap 2,000 expansions | `route_engine/queue.rs:11` |
| `detailed_router` clones per placement | `detailed_router.rs:97,156` |
| Validation clones | `route_engine/validation.rs:99,123,193` |
| Annealed box clamped to `shelf_width` | `global_pnr/annealed.rs` (`side.max(config.shelf_width.max(16))`) |
| Compression ladder starts at 64x64x16 | `compression.rs` (`default_ladder`) |
| Candidate dim 16x16x6, max 16 | `global_pnr/candidate.rs` (`UnitCandidateConfig::default`) |
| Candidate pools clone worlds | `global_pnr/mod.rs` (`ranked_candidate_pools`) |
| `iter_block()` allocates | `src/world/mod.rs:60` |
| `Block` about 64 B, unpacked | `src/world/block.rs:77-111,221-224` |
| `World3D` nested `Vec<Vec<Vec<Block>>>` | `src/world/mod.rs:31-35` |
| Fingerprint cache key | `global_pnr/mod.rs` (`routable_candidate_shape_fingerprint`) |
| Simulator profile API (never enabled by PnR) | `src/world/simulator.rs:616` |

## 7. Open risks

- Local placer deltas must reproduce electrical states exactly; materialize and
  re-initialize redstone states before any electrical query.
- Materializing one scratch world per pop keeps time roughly equal to today;
  verify with counters rather than assuming.
- Frontier caps change candidate results; keep them opt-in until benchmarked.
- The macro library needs canonical shapes; fingerprints exist, but coverage and
  hit rate must be measured before relying on it.
