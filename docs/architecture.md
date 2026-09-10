# CAD-Style Place-and-Route Architecture (Migration Design)

> Status: **design proposal only — no source changes until approved.**
> Branch: `cad-refactor`. Base: `15294b3`.
>
> Companion document: `docs/project_status.md` (what is done, what is missing,
> and the precise failure of the current beam-search placer).

## 1. Goal

Replace the local beam-search placer/router as the primary physical engine with
a small, real CAD flow specialized for Minecraft redstone:

```text
Routable leaf netlist
  -> logic preprocessing (existing)
  -> macro library expansion (new)
  -> 3D placement (new: simulated annealing)
  -> coarse global routing (new)
  -> detailed 3D routing (new: A* with state)
  -> negotiated congestion + rip-up/reroute (new: PathFinder-style)
  -> conflict feedback into placement (new)
  -> redstone validation (existing simulator + new checks)
  -> compression loop (new)
  -> World3D / NBT (existing)
```

Priorities, in order: **routing success rate**, deterministic valid circuits,
reasonable compactness, maintainable architecture. Density is explicitly not
the objective; Minecraft space is cheap.

## 2. Existing architecture (inventory)

Relevant modules today:

| Layer | Module | Role |
| --- | --- | --- |
| Circuit IR | `src/ir/logical.rs`, `routable.rs`, `mapping/` | Logical/Routable RCIR; the mapper produces scalar leaf modules |
| Graph prep | `src/graph/logic.rs::prepare_place`, `src/transform/logic/*` | Decompose, CSE, constant folding, buffers |
| Candidate boundary | `global_pnr/ir.rs` | `LayoutCandidate`, `PhysicalPort`, metric vector, halo |
| Local engine | `place_and_route/local_placer/` | Beam search: topological placement + immediate routing |
| Global engine | `global_pnr/placer.rs`, `router.rs` | Shelf/grid/Free3D placement, A*/GreedyBeam routing, refinement |
| Library | `global_pnr/cell_library.rs`, `sequential/layout.rs` | Candidate policies, contracts, one hardcoded RS-latch macro |
| World | `src/world/` | World3D, Block, propagation semantics, simulator |
| Output | `src/nbt/`, `src/snapshot.rs` | Structure NBT and replayable snapshots |

What is **preserved as-is**: Logical/Routable IR and their validators, the
Verilog frontend, `World3D`/`Block`/simulator, NBT/snapshots, the viewer, and
the `LayoutCandidate` boundary. The new flow replaces the *search inside a
leaf* (and later the global placer), not the IR.

## 3. Phase 1 — Physical IR for placement

The circuit IR is coordinate-free; the physical IR must be separate.

Proposed module: `src/transform/place_and_route/placement_ir.rs`

```text
MacroInstance
  id, macro: MacroId, position, rotation
  pins: resolved absolute pin positions

PhysicalNet
  source: PinRef
  sinks: Vec<PinRef>
  route: Option<Vec<Position>>          // detailed result
  region_sequence: Option<Vec<RegionId>> // global route result
  congestion: CongestionStats

PinRef
  instance, pin name, absolute position, direction, signal kind

PlacementProblem
  macros: Vec<MacroInstance>
  nets: Vec<PhysicalNet>
  region_grid: RegionGrid
```

The IR is Minecraft-independent: it stores positions and connectivity, not
blocks. Conversion to `LayoutCandidate` (and therefore into the existing
pipeline) happens at the end.

## 4. Phase 2 — Macro library

Today a leaf is placed gate by gate. The new flow places **verified macros**.

Extend the existing cell library instead of adding a parallel one:

- `CellImplementation` gains an optional `macro: MacroTemplate`.
- `MacroTemplate`:
  - `size`, `occupied_blocks`, `forbidden_routing_cells`
  - `pins: Vec<MacroPin>` (relative position, direction, signal kind)
  - `allowed_rotations` (conservative: none or yaw)
  - `variant_name` (e.g. `not.compact`, `or.merge`, `xor.nor_network`)
  - `verified_by: SimulatorFingerprint`
- First macro set: NOT, OR merge, AND (decomposed), XOR, MUX, DFF master/slave
  pair, RS latch (already exists as `SequentialMacro`), full adder.
- Recipes (port faces, corridors, objectives) come from
  `docs/physical_design_intent.md`; start with exact verified templates and
  add partial-relation recipes later.

The existing hardcoded RS-latch macro and the `LayoutCandidate` generator
become macro producers rather than special cases.

## 5. Phase 3 — Simulated annealing placement

Proposed module: `src/transform/place_and_route/sa_placer.rs`

- State: `Map<MacroInstanceId, (Position, Rotation)>` in a bounded box.
- Moves: translate, swap two macros, rotate, spread a congested cluster.
- Cost (all terms normalized to comparable units):

```text
cost = 1.0  * estimated_wire_length        (HPWL over nets)
     + 0.1  * bounding_box_volume
     + 20.0 * routing_congestion_estimate   (per-region demand/capacity)
     + 50.0 * blocked_pin_penalty           (pins with no escape cell)
     + 10.0 * region_pressure               (conflict-learning feedback)
```

- Deterministic: fixed seed sequence, canonical tie-breaking, no wall-clock
  decisions.
- Budget: iteration count and restarts from the config (e.g. fast/balanced/
  thorough presets mirroring `GlobalPnrPreset`).
- Acceptance: Metropolis criterion with a geometric cooling schedule.
- Output: top-K placements by cost, converted to `LayoutCandidate`s.

The existing beam-search placer stays available behind a config switch until
the annealing path passes all benchmarks, then is deprecated.

## 6. Phase 4 — Detailed 3D A* router

The repository already contains an A* router in `global_pnr/router.rs`. The
migration extracts a reusable engine:

Proposed module: `src/transform/place_and_route/route_engine.rs`

- Search state: `(position, direction, signal_strength)`.
- Costs: step 1, turn 3, repeater 8, congestion dynamic, blocked = infinite.
- Heuristic: 3D Manhattan distance to the sink.
- Redstone semantics encoded in the engine: strength decay, torch/repeater
  directionality, forbidden contacts, short-circuit rejection.
- Both the detailed router and the local candidate generator use the engine.

The engine must be pure over `World3D` so it can be unit-tested with the
existing simulator fixtures.

## 7. Phase 5 — Coarse global routing

- Region grid over the placement box (e.g. 8x8x4 cells per region).
- For each net, find a region sequence (A* on the region graph with capacity).
- Detailed routing is then constrained to those regions (with escape margins).
- Replaces the current per-net greedy ordering as the first routing stage;
  region congestion becomes an input to the placement cost.

## 8. Phase 6 — Negotiated congestion (PathFinder-style)

Per leaf (and later per design):

```text
for iteration in 0..max_iterations:
    route every net with the current cost model
    for each overused cell:
        present_cost += present_penalty
        history_cost += history_penalty
    rip up nets crossing overused cells
    reroute them
    stop when no cell is overused or the budget is exhausted
```

- `present_cost(cell) = base + present_penalty * overuse`
- `history_cost(cell)` accumulates across iterations so persistent conflicts
  become expensive everywhere.
- Deterministic iteration order and tie-breaking.
- This is the standard answer to "greedy routing boxes itself in": it repairs
  instead of committing.

## 9. Phase 7 — Conflict learning (placement feedback)

Lightweight because circuits are small:

- When a route fails, record the blocking macro instance and region:
  `Conflict { instance, region, net }`.
- Feed conflicts back as: (a) a soft region cost for the next annealing
  round, (b) an optional hard "avoid region" constraint after repeated
  conflicts, (c) candidate filtering during global PnR.
- Bounded conflict store with deterministic eviction (oldest first).

## 10. Phase 8 — Redstone validation

After each routing attempt:

- Signal strength: no segment exceeds the decay limit; insert repeaters
  automatically.
- Direction: torch/repeater orientation matches the intended flow.
- Short circuit: different nets never touch.
- Connectivity: run the existing `Simulator` on the produced world and check
  every source-to-sink path (reuse `candidate_matches_truth_table` patterns
  and the global verifier hooks).

Validation is the acceptance gate for every engine output; invalid candidates
are never cached or emitted.

## 11. Phase 9 — Compression

Generate a valid circuit first, then shrink:

```text
for box in candidate_boxes_sorted_descending:
    re-run placement/routing inside box
    validate
    keep the first (smallest) valid result
```

Compression is a loop around the flow, not a property of the placer. The
existing `LayoutCandidateCost` metrics (volume, footprint, height, access
points) are the acceptance metrics.

## 12. Compatibility and migration strategy

1. **No IR changes.** `LogicalDesign`, `RoutableDesign`, RCIR, snapshots, and
   NBT stay byte-compatible.
2. **`LayoutCandidate` is the seam.** New engines produce candidates; the
   existing global PnR, cell library, cache, and snapshots consume them
   unchanged.
3. **Feature switch.** `GlobalPnrConfig` gains
   `placement_engine: Beam | Annealing` (default `Beam` until the new path
   passes all benchmarks). No existing test changes behavior by default.
4. **Incremental adoption.** Milestone 1 keeps the old router and only adds
   the IR + macro layer. Milestone 2 adds the A* engine behind a flag.
   Milestone 3 switches leaf candidate generation to annealing for a small
   allowlist of shapes, then expands.
5. **Tests stay green.** All 311 non-heavy tests and the existing snapshots
   must keep passing at every milestone.
6. **Deprecation.** The beam-search local placer is removed only after the
   new flow matches or beats it on every benchmark and the ignored smoke
   tests pass.

## 13. Milestones and acceptance criteria

| Milestone | Scope | Acceptance |
| --- | --- | --- |
| M1 | Placement IR + macro model; old engine unchanged | New IR unit tests; 311 tests green; one macro (NOT) round-trips through the IR |
| M2 | 3D A* route engine extracted; old router delegates | Route engine unit tests vs simulator fixtures; old behavior unchanged |
| M3 | Simulated annealing placer for leaf candidates behind a flag | One-bit FSM leaf places; XOR/full-adder benchmarks place; old engine still default |
| M4 | Global routing + PathFinder + conflict feedback | 26-node dense OR cone and two-bit FSM place; deterministic output; snapshot replay stable |
| M5 | Compression loop + new engine default | Compression improves volume on benchmarks; old engine deprecated |

## 14. Benchmarks and metrics

Benchmark set (checked into `test/` as Verilog or RCIR):

1. `not_chain`: NOT -> NOT -> output.
2. `full_adder`: XOR/AND/OR cone.
3. `dense_or_cone`: 26 nodes, multiple fan-in (the current failure case).
4. `fsm_1bit`, `fsm_2bit`: case-based state machines.
5. Random 10-40 gate combinational netlists with a fixed seed.

Metrics recorded per run: routing success rate, generated volume/footprint,
wire length, repeater count, iterations, wall-clock time, and determinism
(byte-identical output across two runs).

## 15. Risks and open questions

- Macro quality: precomputed verified macros may be larger than ad-hoc
  layouts; the compression loop must recover area.
- Rotation legality: redstone is not rotation-invariant; start with
  translation only and add proven yaw transforms.
- Congestion model: a region demand/capacity model must approximate torch
  directionality and strength decay well enough to guide placement.
- Annealing determinism: parallel evaluation must not affect move order.
- Search budget: annealing plus PathFinder can be slower than beam search;
  presets must keep interactive compile times reasonable.
- When to route globally vs per leaf: the current boundary is per leaf; the
  new flow may want to route across leaf boundaries. Keep the boundary until
  M4 proves otherwise.

## 16. Approval gate

No source file is modified until this document is reviewed. After approval,
implementation starts at M1 and each milestone lands as its own commit with
its acceptance evidence in the message body.
