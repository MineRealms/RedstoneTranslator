# Candidate Reject Statistics (M0.12.0)

Status: planned · Branch: `cad-refactor` · Prerequisite for M0.12.5 and G1

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
| `placements_illegal` | rejected by pre-route PECA (`candidate_drc_pre_route` today) |
| `placements_pruned` | generation-time prune under enforcement (`candidate_drc_pruned` today) |
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
