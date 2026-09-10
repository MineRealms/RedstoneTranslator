# CAD-Style Place-and-Route Architecture (Migration Design)

> Status: **design proposal only — no source changes until approved.**
> Branch: `cad-refactor`. Base: `15294b3`. Revision: v2 (review feedback merged:
> force-directed initial placement, logic macros, pin escape routing,
> Minecraft-specific congestion, M0-M5 ordering).
>
> Companion document: `docs/project_status.md` (what is done, what is missing,
> and the precise failure of the current beam-search placer).

## 1. Goal

Replace the local beam-search placer/router as the primary physical engine with
a small, real CAD flow specialized for Minecraft redstone. The root cause of
the current failures is not search depth; it is the paradigm: incremental
greedy construction must become **global optimization plus repair**.

```text
Routable leaf netlist
  -> logic preprocessing (existing)
  -> macro clustering / library expansion (new)
  -> force-directed initial placement (new)
  -> simulated annealing refinement (new)
  -> global congestion estimate + coarse global routing (new)
  -> A* detailed routing with pin escape (new)
  -> PathFinder negotiated congestion + rip-up/reroute (new)
  -> conflict feedback into placement (new)
  -> redstone validation (existing simulator + new checks)
  -> compression loop (new)
  -> World3D / NBT (existing)
```

Priorities, in order: **routing success rate**, deterministic valid circuits,
reasonable compactness, maintainable architecture. Density is explicitly not
the objective; Minecraft space is cheap. The design must also use the third
dimension actively instead of packing into a thin slab.

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
This is the single most important deliverable: in ASIC terms a cell plus free
routing; in Minecraft the cell *is* the building, so a verified macro is worth
far more than a smarter search over raw blocks.

Extend the existing cell library instead of adding a parallel one:

- `CellImplementation` gains an optional `macro: MacroTemplate`.
- `MacroTemplate`:
  - `size`, `occupied_blocks`, `forbidden_routing_cells`
  - `pins: Vec<MacroPin>` (relative position, direction, signal kind)
  - `routing_halo` (cells that must stay free around the macro)
  - `pin_escape`: precomputed escape directions/cells per pin
  - `allowed_rotations` (conservative: none or yaw)
  - `variant_name` (e.g. `not.compact`, `or.merge`, `xor.nor_network`)
  - `verified_by: SimulatorFingerprint`
- Library tiers:
  1. **Primitive macros**: NOT, OR merge, AND (decomposed), XOR, MUX,
     DFF master/slave pair, RS latch (already exists as `SequentialMacro`).
  2. **Logic macros**: half adder, full adder, mux2, small decoder, register
     bit, counter bit. These are common in hand-built redstone and should be
     verified once and composed everywhere.
- File format: a versioned `redstone.lib` document (JSON DTO first, mirroring
  `CellLibrary`) that carries macros, contracts, and recipes.
- Recipes (port faces, corridors, objectives) come from
  `docs/physical_design_intent.md`; start with exact verified templates and
  add partial-relation recipes later.

The existing hardcoded RS-latch macro and the `LayoutCandidate` generator
become macro producers rather than special cases.

## 5. Phase 3 — Placement: force-directed initial + simulated annealing

Proposed module: `src/transform/place_and_route/sa_placer.rs`

Do **not** start annealing from random positions. The netlists are small and
their topology matters; a random start wastes thousands of iterations.

### 5.1 Initial placement (deterministic)

1. **Topological seed**: place macros in topological order along a space-filling
   curve, respecting macro sizes (this is today's beam search without routing).
2. **Barycenter pass**: repeatedly move each macro toward the average position
   of its connected pins until the movement falls below a threshold.
3. **Force-directed relaxation**: spring attraction along nets, AABB repulsion
   for overlap, optional z-axis bias to spread into the third dimension.
   This naturally pulls `A, B, OR` together before any random search.

The result is the SA starting point.

### 5.2 Simulated annealing refinement

- State: `Map<MacroInstanceId, (Position, Rotation)>` in a bounded box.
- Moves: translate, swap two macros, rotate, spread a congested cluster.
- Cost (all terms normalized to comparable units):

```text
cost = 1.0  * estimated_wire_length        (HPWL over nets)
     + 0.1  * bounding_box_volume
     + 20.0 * routing_congestion_estimate   (per-region demand/capacity)
     + 50.0 * blocked_pin_penalty           (pins with no escape cell)
     + 30.0 * pin_access_penalty            (see escape routing, §6.1)
     + 10.0 * region_pressure               (conflict-learning feedback)
```

- Deterministic: fixed seed sequence, canonical tie-breaking, no wall-clock
  decisions.
- Budget: iteration count and restarts from the config (e.g. fast/balanced/
  thorough presets mirroring `GlobalPnrPreset`).
- Acceptance: Metropolis criterion with a geometric cooling schedule.
- Output: top-K placements by cost, converted to `LayoutCandidate`s.

The existing beam-search placer stays available behind
`GlobalPlacementConfig::engine` (`Legacy` by default, `Annealed` selects the
new path) until the annealing path passes all benchmarks, then is deprecated.

## 6. Phase 4 — Detailed routing: extract the engine from the existing router

M2 is **not** "write a new A*". The repository already routes with a unified
search queue (`BreadthFirst | AStar | DirectGreedy | GreedyBeam`), a
`RouteSearchState`, `PlaceBound` expansion, `detailed_router` block
realization, and simulator-based power contracts. M2 extracts that machinery
into a reusable engine while preserving behavior exactly.

Behavior-equivalence is the first principle: all existing tests must pass
unchanged, and the new engine's default configuration must reproduce the old
point-to-point results. No new heuristic, congestion, or escape scoring lands
in M2.

### 6.1 Module layout

Small files under `src/transform/place_and_route/route_engine/`:

- `state.rs`: `RouteState`, `RouteStep`, `ElectricalState`.
- `cost.rs`: `RouteCostModel`.
- `expansion.rs`: neighbor expansion from `PlaceBound` plus `detailed_router`.
- `queue.rs`: the search queue (A*, beam, BFS) extracted from `router.rs`.
- `engine.rs`: `RouteEngine::route(&RouteProblem)`.
- `validation.rs`: `RouteValidator` trait and the fast rule checks.

`router.rs` keeps net ordering, fanout handling, topology logic, and calls the
engine. `detailed_router.rs` keeps `ElectricalPath -> Minecraft blocks`.

### 6.2 RouteProblem and RouteEndpoint

```rust
pub struct RouteProblem<'a> {
    pub world: &'a World3D,
    pub source: RouteEndpoint,
    pub sink: RouteEndpoint,
    pub net_id: Option<NetId>,
    pub constraints: RouteConstraints,
}

pub struct RouteEndpoint {
    pub position: Position,
    pub allowed_entries: Vec<Direction>,  // pin escape; empty = any
    pub required_power: bool,
}
```

`allowed_entries` carries the macro pin escape positions that M1 already
records. Spatial pin facing stays out of M2; `MacroPin.facing` lands in M3,
where placement costs consume it.

### 6.3 State and electrical mode

```rust
pub struct RouteState {
    pub position: Position,
    pub incoming: Option<Direction>,
    pub electrical: ElectricalState,
    pub cost: RouteCost,
    pub path: Vec<RouteStep>,
}

pub enum ElectricalState {
    Wire { strength: u8 },
    Repeater { delay: u8 },
    Torch,
    HardPowered,
}
```

Do not introduce a second signal-mode enum: the existing `PropagateType`
(`Soft | Hard | Torch | Repeater`) is the mode type and maps directly onto
`ElectricalState`.

### 6.4 Cost model

```rust
pub struct RouteCostModel {
    pub step_cost: usize,
    pub turn_cost: usize,
    pub repeater_cost: usize,
    pub low_strength_penalty: usize,
}
```

Parity defaults replicate the historical priority exactly: `step_cost = 1`,
`turn_cost = 0`, `repeater_cost = 0`, and `low_strength_penalty = 4` applied
when signal strength is at most two. The turn and repeater terms exist but
stay zero until a benchmarked change enables them; congestion stays at zero
until M4.

### 6.5 Two-level validation

- **Level 1 (per expansion)**: cheap rules only — strength decay, direction,
  occupancy, forbidden contacts, short-circuit checks.
- **Level 2 (complete candidate only)**: `route_engine::validation` settles the
  routed world with the existing simulator
  (`Simulator::from_preserving_torch_states_with_limits_and_trace`) and checks
  the power contract: required positions powered while the source is active,
  released when a switch source is off, and no self-sustaining feedback cycle.

Simulating every A* node would explode, so simulate-based penalties feed back
into routing only in M4 (PathFinder).

```rust
pub(crate) trait RouteValidator {
    fn validate(
        &self,
        before: &World3D,
        after: &World3D,
        route: &RoutedNet,
    ) -> RouteValidationResult;
}
```

`SimulatorRouteValidator` is the only implementation today.
`route_candidate_powers_sink` delegates to it for the incremental strategies
and skips the simulation for the cheap DirectGreedy/GreedyBeam probes, which
global PnR validates on the complete routed world.

### 6.6 Mandatory gap: `PlaceBound::propagated_from`

`propagated_from` currently panics (`todo!()`) for Torch, Repeater,
RedstoneBlock, and Switch. Reverse search, escape routing, and bidirectional
heuristics all need predecessor enumeration. M2 implements these rules before
the engine is switched on, covered by dedicated tests.

### 6.7 M2 substeps and exit criteria

- M2.0: extract queue/state/expansion into `route_engine/`; the old router
  calls the extracted code with identical behavior.
- M2.1: implement `propagated_from` for the four missing block kinds.
- M2.2: make `RouteCostModel` explicit; defaults equal the old costs.
- M2.3: add the `RouteValidator` interface and move the simulator contract
  check behind it.

Exit criteria: the new engine replaces the old point-to-point route; old/new
parity on the same `(world, source, sink)` inputs; simulator validation
passes; the cost model is pluggable; `propagated_from` is complete. Not
required in M2: congestion, SA feedback, escape scoring, compression.

### 6.8 Pin escape routing

A* can find a theoretical path while the pin is physically boxed in by
neighbouring macros. Placement must therefore score pin accessibility, not just
pin existence.

- After placement, build a per-pin **escape map**: flood-fill the free cells
  adjacent to the pin (treating other macros as obstacles) and record
  - the number of distinct escape directions,
  - the distance to the nearest free routing channel,
  - whether the pin is reachable from the other endpoints of its net.
- Pin score: `0 = dead end`, `1 = risky`, `2 = normal`, `3+ = excellent`.
- `blocked_pin_penalty` and `pin_access_penalty` in the placement cost consume
  this score; a macro with any pin score 0 is rejected outright.
- The detailed router starts from the pin's escape cells rather than from the
  pin block itself, mirroring PCB escape routing.
- Macros publish precomputed escape directions per pin (`pin_escape`) so the
  map is cheap to build.
- Spatial pin facing (`MacroPin.facing`) is a placement-cost input and lands in
  M3; M2 only needs escape positions, exposed as `RouteEndpoint.allowed_entries`.

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

Implemented in `route_engine::pathfinder` (standalone loop over flat nets) and
`route_engine::negotiation` (the router post-pass). The post-pass runs after
the greedy net loop: it builds the congestion map from the routed paths, folds
present overuse into history, rips up routes crossing overused cells, and
reroutes them with penalties. Only routes with the default simple power
contract are rerouted; adapter routes with extra required positions are left
alone. Every accepted reroute must keep its power contract on the assembled
world, and candidates whose assembly would leave unsupported redstone are
rejected. Rejected candidates add history penalties on their cells, so the
next pass steers away from the failing corridor (simulator feedback).
Enable it with `GlobalRoutingConfig::pathfinder` (default off); the
setting round-trips through RCIR snapshots and the text format.

### 8.1 Minecraft-specific congestion

FPGA congestion counts wire resources; Minecraft congestion must model voxels
and block behavior. The cost of a cell is a weighted sum:

```text
congestion(cell) = w1 * block_occupancy
                 + w2 * signal_corridor_pressure
                 + w3 * direction_conflict
                 + w4 * future_escape_cost
```

- `signal_corridor_pressure`: how many nets want to cross this cell's region.
- `direction_conflict`: opposing torch/repeater directions sharing a corridor.
- `future_escape_cost`: a free cell that is the *only* escape for a macro pin
  has high congestion even though it is empty. This is what prevents the
  placer from sealing a pin with a later macro.
- Weights are preset per flow stage: placement uses a coarse estimate, global
  routing a region-level one, detailed routing the exact one.

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

Generate a valid circuit first, then shrink. Minecraft makes this cheap: start
with abundant space and let the compressor find the smallest valid box.

```text
box ladder: 64x64x16 -> 56x56x14 -> 48x48x12 -> 40x40x10 -> 32x32x8 -> 24x24x6
for box in ladder (descending):
    re-run placement/routing inside box
    validate (redstone checks + simulator)
    keep the first (smallest) valid result
```

Compression is a loop around the flow, not a property of the placer. The
existing `LayoutCandidateCost` metrics (volume, footprint, height, access
points) are the acceptance metrics. The ladder, not a single hard box, is the
search domain.

Implemented in `place_and_route::compression`: `compress` owns the descending
iteration and acceptance bookkeeping, and
`place_and_route_with_compression` runs the full PnR flow once per box,
constraining every top-level instance inside the box through a generated
physical intent (`Inside` constraints over one region). CLI wiring and
benchmark evidence are the remaining M5 work.

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

The ordering below is deliberate: benchmarks first, then the router, then
placement, then repair, then compression. Each milestone is a separate commit
with its acceptance evidence.

| Milestone | Scope | Acceptance |
| --- | --- | --- |
| M0 | Benchmark set + baseline metrics harness | `not_chain`, `full_adder`, `dense_or_cone`, `fsm_1bit`, `fsm_2bit`, `random_10`, `random_40` all lower to a valid topology; baseline metrics recorded (leaves, prepared sizes, P&R success/time) |
| M1 | Placement IR + macro model; old engine unchanged | New IR unit tests; all existing tests green; one primitive macro and one logic macro (full adder) round-trip through the IR |
| M2 | Router core extraction: M2.0 extract queue/state/expansion, M2.1 reverse propagation rules, M2.2 explicit cost model, M2.3 validator interface | Old/new parity on the same `(world, source, sink)` inputs; simulator validation passes; cost model pluggable; `propagated_from` complete for Torch/Repeater/RedstoneBlock/Switch |
| M3 | Macro placement: topological seed + force-directed + SA refinement, behind a flag | One-bit FSM leaf places; full adder and dense OR cone place; old engine still default |
| M4 | Coarse global routing + PathFinder negotiated congestion + conflict feedback | Two-bit FSM and dense OR cone place deterministically; snapshot replay stable |
| M5 | Compression ladder + new engine default | Compression reduces volume on benchmarks; old beam-search engine deprecated |

## 14. Benchmarks and metrics

Benchmark set (checked into `test/benchmarks/` as Verilog):

1. `not_chain.v`: NOT -> NOT -> output.
2. `full_adder.v`: XOR/AND/OR cone.
3. `dense_or_cone.v`: 26+ OR nodes with multiple fan-in (the current failure
   case).
4. `fsm_1bit.v`, `fsm_2bit.v`: case-based state machines.
5. `random_10.v`, `random_40.v`: deterministic mixed-gate netlists.

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
