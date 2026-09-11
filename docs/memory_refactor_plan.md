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

### [x] Commit 2 — perf: adaptive initial box

Scope: stop clamping the annealed placement box to `shelf_width` 64; derive it
from the total candidate volume (with a floor at the largest macro footprint).
Also fix the compression ladder semantics: it now shrinks until the first
failure and keeps the last valid box, instead of returning the first success
(which was the largest box). Starting the ladder from a volume estimate needs
candidate sizes and is deferred to the candidate-streaming work.

Files: `global_pnr/annealed.rs`, `compression.rs`.

Acceptance met: the annealed adapter no longer forces a 64-wide box; the
ladder now actually compresses; non-heavy suite green (362 tests).

### [x] Commit 3 — copy-on-write World3D (consolidates Commits 3 and 4)

Scope: `World3D` stores per-layer `Arc<Vec<Block>>`; `clone()` is an outer-vector
copy plus Arc bumps, and `IndexMut` copies a layer only when it is shared. This
achieves the delta-state goal (search entries no longer duplicate unchanged
world data) for both the local placer queue and `RouteSearchState` with a
single contained change, and outputs stay byte-identical by construction. New
`perf` counters track actual layer copies separately from logical clone bytes.

Files: `src/world/mod.rs`, `src/world/simulator.rs` (test indexing),
`src/perf.rs`.

Measured on `not_chain` (smallest benchmark): peak RSS 265 MiB to 58 MiB,
candidate preparation 394 ms to 153 ms, actual copied bytes 4.1 GB logical to
748 MB of layer copies; the non-heavy suite drops from 7.3 s to 3.2 s and stays
green (362 tests).

Known gap: `full_adder` (27 prepared nodes) still exhausts memory because a
single `do_step` expands a huge frontier before sampling; that is Commit 5.

### [x] Commit 4 — folded into Commit 3

The route search state keeps its `world` field, but the field is now cheap to
clone and shares unchanged layers, so the separate delta refactor is no longer
required. Revisit only if routing frontiers still dominate after Commit 5.

### [x] Commit 5 — perf(local): deterministic work budget and frontier cap

Scope: a per-step frontier cap (default 16,384 entries, distributed across the
input frontier) plus a deterministic work limit (10M `World3D` clones / 2M
placement generations per local search), both overridable with
`MCHDL_FRONTIER_CAP` and `MCHDL_LOCAL_CLONE_LIMIT`. Large frontiers run the
step sequentially so the cap can stop expansion early; small frontiers keep the
parallel path. Exceeding a limit reports a clear error instead of aborting.

Files: `local_placer/mod.rs`, `perf.rs`, `candidate.rs`.

Acceptance met: `full_adder` and `fsm_1bit` fail gracefully in about 66 s with
`local candidate generation ... exceeded its work limit` and peak RSS around
250 MiB; no OOM abort. Non-heavy suite green (362 tests).

Key finding: a wide-limit control run (`MCHDL_LOCAL_CLONE_LIMIT=60000000`,
`MCHDL_FRONTIER_CAP=131072`, 16 GB budget) still produced zero candidates for
the 13-node `state_next` cone after 429 s / 60M clones at only 2.4 GB RSS. A
second control with the tuned smoke-test config (sampling 32) produced 32
candidates in 13.5 s at 21 MiB RSS, but **all 32 were rejected by the
truth-table check** (`candidate_truth_rejects=32`, zero port rejects). The
legacy local placer is therefore limited by placement quality/correctness, not
memory; the limits bound the damage, they are not the cause. The real fix is
replacing the leaf realizer (verified macro library for repeated shapes plus
the placement-first path).

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

## 4.1 Active investigation — leaf candidate truth-table rejections (M0.6)

**Status**: Phase 1 done, Phase 2 partially done · **Time box**: 30-60 minutes

**Results so far**:

- **Fixed (root cause A)**: the default `UnitCandidateConfig.combinational_sampling_limit`
  was `None`, which left multi-input combinational children on `Random(512)`
  step/route sampling. Even `assign y = a & ~b` burned the 10M clone work limit
  with no result. With the default changed to `Some(32)`, `a & ~b` compiles end
  to end in under a second (162 MiB peak), and `state_next` goes from OOM to 32
  candidates in 13.5 s at 20 MiB.
- **Root cause B (localized)**: all 32 `state_next` candidates are rejected by
  the truth-table check. An exact graph reproducer now lives in the ignored
  test `state_next_graph_candidate_truth_reproducer`
  (`global_pnr/candidate.rs`); it reproduces `candidates=0 truth_rejects=32`
  with the same config as the smoke test. Bisect results: `tail_n8` (Or tail)
  and `tail_n19` (Not of n8) pass; `tail_n20` (`Not(Not(state))` as the output)
  and `tail_n21` fail with 32/32 rejects; `state_direct` and `n8_direct`
  produce no placed worlds. The minimal failing subgraph is the double
  inversion `state -> n16 -> n20` used as the output, and the failure mode
  matches the second inversion being realized as a single inversion
  (`n20 = ~state`): at `go=0,state=1` that yields `n22=1` where the expected
  value is 0, exactly the printed mismatch. Next step: expose the local
  placer's node positions (`PlacementState`) to confirm that the second
  inversion's support is powered by the `state` input instead of `n16`, then
  fix the routing.
- **Limitation (root cause C)**: `full_adder` (27 prepared nodes) produces zero
  placements even with `combinational_sampling_limit = 128` and a 40M clone
  limit; the legacy local placer cannot realize that cone.
- **New engine experiment (fixed)**: a composite `andnot` chain (three
  instances) with `--placement-engine annealed` initially failed to route. The
  causes were the missing spacing/pin-access costs (macros packed flush), no
  placement margin (external input switches could not fit), and a single
  placement attempt per layout combination. After adding halo-aware spacing and
  pin-access costs, a four-cell margin, six-cell channels, and four
  deterministic seed attempts, the Annealed engine completes the design in
  4.7 s at 49 MiB (Legacy: 16.6 s at 73 MiB).

**Context**: the M0.5 work removed the memory wall (COW `World3D`, work budget,
frontier cap). The remaining wall is leaf placement quality/correctness. With
the tuned smoke-test config, the 13-node `state_next` cone produces 32
candidates in 13.5 s at 21 MiB RSS, and **all 32 are rejected by
`candidate_matches_truth_table`** (`candidate_truth_rejects=32`, zero port
rejects). This is the known pre-existing failure recorded in
`project_status.md`.

**Goal**: identify the root cause of the truth-table rejections with evidence,
then either fix it with a regression test or record the decision to replace the
leaf realizer.

### Phase 1 — dump the first failure (about 10 minutes)

Add an env-gated diagnostic (`MCHDL_DEBUG_TRUTH_TABLE=1`) inside
`candidate_matches_truth_table` (`global_pnr/candidate.rs`):

- first fresh-world mismatch: mask, input names and positions, output names and
  positions, expected vs actual `is_powered()` for every output;
- first transition-sequence mismatch: previous mask, current mask, expected vs
  actual.

Run with a 16-minute timeout (about 15 s in practice):

```text
$env:MCHDL_PERF='1'; $env:MCHDL_DEBUG_TRUTH_TABLE='1'
cargo test --release --lib -j 1 fsm_module_generates_world_from_child_layout_candidates -- --ignored --nocapture --test-threads=1
```

Interpretation: a fresh-world failure means combinational behavior is wrong
(missing input, stuck output, short). A transition-only failure means the
dynamic release/hold behavior is wrong (stale simulator state, latch/feedback,
short that only appears after switching).

### Phase 2 — localize to a gate or wire (20-40 minutes)

- Output stuck or input-independent: dump `placed.world` with `{:?}` and follow
  the driver path from `placed.outputs` back to `placed.inputs`; check the
  `detailed_router` short/contact rules.
- Only some masks wrong: flip one input bit at a time, then map the failing
  net back to a leaf graph node and its placement.
- Transition-only failure: print the output level after each `change_state` in
  one simulator to see whether the level fails to release or fails to hold.
- Control: run the same diagnostic on `not_chain` (which passes) to establish
  the passing baseline.

### Phase 3 — fix or pivot

- Placer/routing bug: fix `local_placer/routing.rs` or `detailed_router.rs` and
  add a regression test.
- Verifier too strict: relax `candidate_matches_truth_table` carefully with
  tests; the transition check exists for a reason.
- Search quality: record the evidence and move to the leaf-realizer
  replacement (macro library plus placement-first path).

**Acceptance**: root cause identified with a printed evidence trail; either a
fix with a regression test, or a documented decision to replace the leaf
realizer. Keep the diagnostic env-gated if it is useful long-term, otherwise
remove it before committing.

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
