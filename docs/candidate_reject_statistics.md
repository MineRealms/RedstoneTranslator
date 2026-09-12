# Candidate Reject Statistics (M0.12.0)

Status: implemented · Branch: `cad-refactor` · Prerequisite for M0.12.5 and G1

## 1. Why

We know candidates die in large numbers, but not where. `not_chain` shows
`~2880 placements -> 71 routed` and `state_next` shows `32 placed -> 32 truth
rejects`, while `full_adder` dies before reaching truth at all. Without a
per-stage breakdown, GPU and algorithm priorities are guesses. M0.12.0
instruments the pipeline so every benchmark reports exactly which stage
kills which fraction of candidates.

The numbers directly decide:

- if **geometry/occupancy** dominates -> fix placement enumeration;
- if **electrical** dominates -> extend PECA filtering (G1 GPU evaluator);
- if **routing** dominates -> route-cost fields / constraint-directed
  enumeration (G2/G3);
- if **simulation** dominates -> GPU DC pre-filter (G4) or fewer candidates.

## 2. Counters

Extend `src/perf.rs` (relaxed atomics, always active, same style as the
existing counters):

| Counter | Meaning |
| --- | --- |
| `placements_enumerated` | `(position, direction)` records produced before any check |
| `placements_conflict` | rejected by occupancy/`has_conflict` (geometry) |
| `placements_pruned` | generation-time prune under enforcement (`candidate_drc_pruned`); pre-route violations are counted by `candidate_drc_pre_route` |
| `route_attempts` | candidates entering the exact router |
| `route_failures` | exact router returned no route |
| `route_successes` | candidates that produced at least one routed world |
| `candidates_accepted` | candidates that reached the Pareto frontier |
| `candidate_truth_rejects`, `candidate_port_rejects`, `candidate_drc_rejects`, `candidate_drc_driver_side` | already present |

`route_failures` should be split by reason where available: the local route
debug already records `RouteRejectReason` (isolation, short, conflict, step
budget); aggregate the top reasons into the summary.

## 3. Integration points

- `local_placer/routing.rs`
  - `generate_torch_place_and_routes`: count enumerated, conflict
    (`place_torch_with_cobble` returned `None`), illegal (observer false),
    attempts, failures.
  - `generate_inputs` / `generate_constant_placements`: count enumerated.
  - `generate_or_routes`: count tap routes and isolation rejects.
- `local_placer/mod.rs`
  - `do_step`: per-step aggregate (already emits `input/generated/compacted/
    sampled` via `tracing::trace!`); add the new counters to the trace fields.
- `global_pnr/candidate.rs`
  - `generate_unit_candidates`: `candidates_accepted`; existing truth/port/DRC
    rejects stay.
- `global_pnr/mod.rs`
  - `routing-attempts` stage (line 1089): global route attempts/failures for
    composite designs.

## 4. Output

- Extend the `[perf] summary` line with the new counters (printed under
  `MCHDL_PERF=1`, unchanged otherwise).
- Per-step `tracing::trace!` gains the same fields, so
  `RUST_LOG=redstone_compiler::local_placer=trace` shows the stage breakdown
  per node.
- Optional follow-up: a `[stats]` JSON block per module for benchmark
  comparison; not required for the first cut.

## 5. Constraints

- Zero behavior change: counters are observational; no ordering, sampling, or
  acceptance logic may depend on them.
- Determinism: counts are integers; printing is deterministic.
- Default path stays byte-identical; the counters only add relaxed atomic
  increments.

## 6. Acceptance

- `not_chain` reports a full breakdown
  (enumerated -> conflict -> illegal -> routed -> sampled -> truth -> accepted).
- The `state_next` reproducer reports per-variant geometry/electrical/routing/
  simulation splits.
- `full_adder` and `fsm_1bit` report where they die (they currently produce no
  candidates).
- Non-heavy suite stays green (373 tests) with counters on and off.
- Documented in `docs/gpu_acceleration_plan.md` (G0) and `docs/roadmap.md`.

## 7. Follow-up

M0.12.5 uses these numbers to target the dominant stage with
constraint-directed enumeration; G1 then puts the cheap, deterministic filter
on the GPU (`docs/gpu_acceleration_plan.md`).

## 8. Results

`not_chain` (per step, `RUST_LOG=redstone_compiler::local_placer=trace`):

```
step 1 input switch: enumerated=930  conflict=0    route=0/0/0      generated=930 -> sampled 32
step 2 NOT:          enumerated=1835 conflict=280  route=1555/1484/71 generated=71  -> sampled 32
step 3 output:       enumerated=0
totals: enumerated=2765 conflict=280 route=1555/1484/71 candidates_accepted=32
```

`state_next` reproducer (per variant, counters reset per run):

| Variant | Enumerated | Conflict | Route attempts | Route failures | Route successes | Truth rejects |
| --- | --- | --- | --- | --- | --- | --- |
| `full` | 20570 | 5750 (28%) | 4670 | 2880 (62%) | 1790 | 32 |
| `no_dead` | 12943 | 3868 (30%) | 3457 | 2072 (60%) | 1385 | 32 |
| `tail_n8` | 20570 | 5750 | 4670 | 2880 (62%) | 1790 | 0 |
| `tail_n19` | 20570 | 5750 | 4670 | 2880 (62%) | 1790 | 0 |
| `tail_n20` | 21910 | 6765 (31%) | 5058 | 3162 (63%) | 1896 | 32 |
| `tail_n21` | 20570 | 5750 | 4670 | 2880 (62%) | 1790 | 32 |
| `state_direct` | 10418 | 1881 (18%) | 2686 | 1729 (64%) | 957 | 0 |
| `n8_direct` | 10418 | 2027 (19%) | 2736 | 1493 (55%) | 1243 | 0 |

Findings:

- **Routing dominates**: 55-64% of route attempts fail, and those attempts
  happen only after 18-31% of enumerated placements are already rejected by
  geometry. The G1 filter and constraint-directed enumeration (M0.12.5) should
  target route attempts first.
- Geometry conflict is a significant secondary stage (18-31%).
- DRC (`candidate_drc_pre_route`/`driver_side`) is a small fraction of the
  enumerated volume for these shapes, but it is the only stage that explains
  the `tail_n20`/`tail_n8` truth failures; it is cheap to run before routing.
- `candidates_accepted` equals the final sampled count for `not_chain` (32),
  and the funnel is dominated by the two stages above.

Note: `[perf] summary` lines are wrapped by PowerShell `Out-File`; read the
log with `-Width 4096` or rejoin lines for machine parsing.

### Route goal breakdown (`not_chain`, default `DirectOnly`)

```
route_goal_direct=71   route_goal_redstone=0   route_goal_skipped=1555
route_goal_empty=1484  route_init_empty=0
```

The default `NotRouteStrategy` is `DirectOnly`, so the redstone branch never
runs and the 95% failure rate is not a search failure: the support simply is
not in the source's direct-bound set. The `MCHDL_ROUTE_QUOTA=4` experiment is
a negative result: it is 4.6x faster on the reproducer but biases the
selection (first N successes) and kills the search (0 candidates).

### M0.12.5 first cut: direct-bound pre-check

Under `DirectOnly`, a placement whose support is not a direct bound of the
source can never route. Pre-computing the bound set and skipping those
placements is output-identical, because the skipped placements produced no
route:

| | before | after |
| --- | --- | --- |
| route attempts | 1555 | 71 |
| route failures | 1484 | 0 |
| route successes | 71 | 71 |
| route skipped | - | 1484 |
| final NBT SHA256 | `09B44F4B...` | `09B44F4B...` |

`DirectAndRedstone` (the reproducer configuration) still attempts redstone
routes that fail; that is the next target (route-field pre-filter, G1/G3).

### Redstone branch breakdown (`m06-andnot`, `DirectAndRedstone`)

```
route_attempts=2834  route_failures=2066  route_successes=768
route_goal_direct=207  route_goal_redstone=955  route_goal_empty=2066
route_init_empty=0
```

73% of the attempts produce no route at all, and the init states are never
empty, so the search runs but finds nothing. A sound necessary condition
("the support must have at least one placeable dust powering position") was
added for the redstone branch:

| | before | after |
| --- | --- | --- |
| route attempts | 2834 | 2739 |
| route skipped | - | 95 |
| route failures | 2066 | 1971 |
| route successes | 768 | 768 |
| final NBT SHA256 | `3D8B4B33...` | `3D8B4B33...` |

The filter is output-identical but only explains 95 of the 2066 failures: the
remaining ones fail inside the redstone search (cobble conflicts along the
path, short-circuit rejections, or step/sampling limits), which needs route
internals instrumentation or a route-field pre-filter rather than a local
geometry check.
