# Physical Electrical Connectivity Analysis (PECA)

Status: M0.10a done (report-only) · Branch: `cad-refactor` · Owner: M0.10

## 1. Why this layer exists

The `state_next` truth-table rejection was localized to a physical electrical
defect, not a routing reachability bug. A NOT macro's input pin (its support
cobble) was placed directly adjacent to the `state` input switch:

```
(0,5,1) Switch(state)   ── powers ──▶ (0,4,1) cobble (NOT input pin)
(1,4,1) Torch(n16)      ── powers ──▶ (0,4,1) cobble (NOT input pin)
```

Because the simulator counts an adjacent switch as a cobble power source, the
pin is driven by `state | ~state = 1`, the second inverter's torch is always
off, and the output is stuck low. The placer's existing checks only enforce
**occupancy** (no cell overlap) and **reachability** (the intended source can
drive the pin). They do not enforce **exclusivity** (only the intended source
drives the pin). That missing check is the defect.

PECA is the compile-time layer that establishes physical electrical facts and
lets rules (DRC) run on top of them. It is the "Physical Electrical Netlist"
step between routing and simulation in a conventional flow:

```
placement → routing → PECA (facts) → DRC (rules) → simulation → truth table
```

## 2. Non-goals and constraints

- Do not modify `Block`, `World3D`, NBT output, snapshots, or simulator
  semantics. All PECA data is compile-time only and discarded after use.
- Do not modify routing algorithms. PECA consumes the world and the placement
  provenance the local placer already knows; it emits metadata, not routes.
- All output is deterministic (sorted), matching the project convention.

## 3. Shared electrical model (`world/electrical.rs`)

The simulator already owns the redstone electrical rules (how a torch/switch/
repeater/redstone block/redstone dust powers its targets). These rules are
extracted into `world/electrical.rs` so the simulator and PECA share one
source of truth, preventing the classic DRC-vs-simulator rule drift:

- `redstone_propagate_targets(world, pos, state)` — redstone dust target set.
- `power_targets(world, source)` — every target a power source can drive
  (soft/hard), the generalization of the simulator's `cobble_power_inputs`
  construction.
- `cobble_power_sources(world, target)` — possible drivers of a cobble.

A differential test asserts `cobble_power_sources` matches the simulator's
actual power propagation on small hand-built worlds (possible ⊇ actual).

## 4. Facts the local placer emits

The local placer knows the physical meaning of every block it places; PECA must
not re-derive it. Two compile-time sidecar records are emitted alongside each
candidate (they survive `retain_nodes` and cover dead logic):

- `anchors: Vec<(GraphNodeId, Position)>` — the output net of every node maps
  to its anchor position (a terminal for input/constant/NOT, the tap redstone
  for an OR merge).
- `pins: Vec<PinRecord>` — each pin and its electrical contract.

## 5. Pin contracts

```rust
enum PinContract {
    Single { expected_net: GraphNodeId },
    Merge  { input_nets: Vec<GraphNodeId> },
    Passive,
}
```

- `Single(n)` — a NOT/repeater/latch input pin: every possible driver must
  belong to the same electrical unit as `n`'s anchor (its terminal, or a dust
  component whose only driving net is `n`). This is the exclusivity check.
- `Merge(a, b)` — an OR tap: the observed driver set at the tap must be
  exactly `{a, b}` (both branches reach, no foreign net). A branch may reach
  the tap through a cobble that a terminal powers (one relay hop), because the
  simulator's cobble event handling propagates terminal power to adjacent
  dust; dust never relays through a cobble.
- `Passive` — sequential-macro internals are skipped in the first release.

The judgment unit is the **electrical component** (a dust network plus the
terminals that drive it), never a raw position, because fanout and wired-OR
make multi-source components legal.

## 6. Confidence

Two independent axes:

- **Driver confidence** — `Always` (redstone block), `Possible`
  (switch/torch/repeater/powered wire), `Never` (proven constant-off after
  DCE/constant propagation). The first release treats every driver as
  `Possible`.
- **Violation confidence** — `Certain` when the violation follows directly
  from the shared electrical rules (all `Single` violations), and
  `SimulationRequired` when static analysis cannot decide (all `Merge`
  violations). Enforcement only applies to `Certain` violations; `Merge`
  stays report-only until a simulation fallback is added.

## 7. Rollout (report-first)

1. **M0.10a** — report only: run PECA before truth-table validation, print
   `[peca]` violations and count them (`candidate_drc_violations`).
2. **M0.10a.1** — Merge semantics: observed driver set at the tap, with the
   one-hop cobble relay; eliminates the `MissingBranch` false positives.
3. **M0.10a.2** — confidence model: `Single` is `Certain`, `Merge` is
   `SimulationRequired`.
4. **M0.10b** — enforce `Single` only, implemented as an opt-in mode
   (`MCHDL_PECA_ENFORCE=1`). Default-on is blocked on M0.12 generation-time
   avoidance: the legacy placer currently produces only shorted candidates for
   the failing designs, so enforcement alone turns "compiles with a hidden
   short" into "does not compile".
5. **M0.10c** — enforce `Merge`, with the localized simulation fallback.
6. **M0.11** — DCE of dead logic (separate concern; removes the dead-logic
   coupling noise and shrinks the search space).
7. **M0.12** — generation-time rejection (support-position local check, OR tap
   two-branch reach) so bad candidates are pruned early.

## 8. Regression cases

- `double_not` (valid) — passes.
- `foreign_switch_adjacent` (tail_n20 shape) — `Single` violation: `state` and
  `n16` both drive `n20`'s input pin.
- `or_one_sided` (full graph shape) — `Merge` violation: the OR tap is driven
  by only one branch.
- `fanout` — one net driving many pins is not a violation.

## 9. M0.10a results (commits `fa1d6db`, `7a6d8c9`)

- `world/electrical.rs` is the shared rule set; the simulator consumes it
  (27 simulator tests unchanged).
- The local placer emits per-node anchors and per-pin contracts; `analyze`
  runs before truth-table validation and reports without rejecting.
- The confirmed `state_next` defect is reproduced mechanically: all 32
  `tail_n20` candidates report
  `node=20 pin=(0,4,1) ExtraDriver drivers=[(5, (0,5,1))]` — the `state`
  input switch driving the second inverter's support cobble.
- Regression tests: `foreign_switch_adjacent_to_not_input_is_a_single_violation`,
  `double_not_with_single_driver_is_clean`, `fanout_of_a_single_source_is_clean`.
- Non-heavy suite: 368 passed.

### 9.1 Finding: the DRC is stricter than the truth table

Candidates that pass the truth table can still contain electrical shorts. In
`tail_n8`/`tail_n19` the accepted candidates report ~36 violations, partly on
dead logic (removed later by DCE) and partly on live logic where the short
does not flip the tested output. This is the intended value of the layer: the
truth table is necessary but not sufficient.

### 9.2 Merge semantics fixed (M0.10a.1/M0.10a.2)

The `MissingBranch` reports on valid OR merges were not a semantics problem
but an incomplete electrical model: a branch can reach the tap through a
cobble that a terminal powers (the simulator's cobble event handling
propagates terminal power to adjacent dust), and the component walk did not
include that hop. The fix models dust-only components plus a one-hop
terminal -> cobble -> dust relay; dust never relays through a cobble (the
simulator ignores redstone events on cobbles). After the fix the reproducer
reports zero `MissingBranch` violations.

Violations now carry confidence: `Single` is `Certain`, `Merge` is
`SimulationRequired`. The remaining `tail_n8`/`tail_n19` violations are real
couplings: dead-logic nets reaching live components (removed later by DCE)
and live shorts whose extra driver is logically absorbed by the cone (the
truth table passes, but the circuit is fragile). The confirmed `state_next`
`Single` violation persists on all 32 `tail_n20` candidates.

### 9.3 Enforcement measurement (M0.10b)

With `MCHDL_PECA_ENFORCE=1`, every reproducer variant whose candidates carry a
`Certain` violation is rejected before the truth-table check
(`drc_rejects=32`, `truth_rejects=0`). The non-heavy suite stays green (372
passed; it contains no `Certain` violations). Enforcement is safe as a gate,
but the legacy placer cannot yet produce a clean candidate for the
`state_next`/`tail_n8`/`tail_n19` shapes: generation-time avoidance (M0.12) or
DCE (M0.11) is required before default-on.

## 10. M0.12 interface freeze (Electrical Legality Filter)

M0.12 turns candidate generation from "geometrically placeable" into
"electrically legal". The interface is frozen before implementation because it
is consumed by NOT, repeater adapters, sequential macros, the global router,
and the SA cost model.

### 10.1 Module layout

The layer stays under `src/transform/place_and_route/` (never `world/`, which
remains the pure Minecraft model). Target layout once the file grows:
`electrical_drc/{mod.rs, pin.rs, driver.rs, report.rs}`. The shared driver
query already lives in `world/electrical.rs` (M0.12.0, commit `fa1d6db`) and is
used by both the simulator and PECA, so the two can never disagree on what can
power what.

### 10.2 Types

```rust
pub enum PinPort { Named(String), Index(usize) }
pub struct PinId { pub node: GraphNodeId, pub port: PinPort }

pub enum PinIsolationPolicy { SingleSource, Merge, SequentialInput, None }

pub struct PinRecord {
    pub id: PinId,
    pub position: Position,
    pub contract: PinContract,
    pub policy: PinIsolationPolicy,
}

pub enum DrcResult { Pass, Reject(Vec<Violation>) }
```

Primitives use `PinPort::Index` (`in0`, `in1`, `out`); sequential and macro
pins use `PinPort::Named`, reusing the `MacroPin.name` convention. The logical
net identity stays `GraphNodeId` (the truth-table identity); the port only
qualifies the physical pin.

### 10.3 Hooks

- **Pre-route pin check** (`check_pin_before_route`): after
  `place_torch_with_cobble` and before `generate_routes_to_cobble`. It only
  requires `existing_driver \ allowed_source == ∅`, because the route that
  connects the intended source does not exist yet.
- **Driver-side check** (`check_new_driver_against_existing_pins`): whenever a
  new terminal (torch, switch, redstone block, repeater) is placed, verify its
  `power_targets` do not hit an existing foreign pin. This catches coupling
  introduced by later nodes.
- **Post-candidate PECA** remains the complete validator
  (`driver_nets == expected`), including the OR merge.

`allowed_sources` is expressed as `AllowedSource::{NodeOutput, PhysicalTerminal}`
so fanout and future repeaters extend it without reshaping the API.

### 10.4 Enforcement policy

`Certain` (`SingleSource`) violations are hard-rejected. `SimulationRequired`
(`Merge`, near-driver) violations are report-only and may later become a soft
cost term rather than a reject. Soft penalties are never applied to `Certain`
violations, because a deterministic short must not be sampled.

### 10.5 Commit order

1. **M0.12.0** — shared electrical driver query (`fa1d6db`, done).
2. **M0.12.1** — report-only pre-route pin check + `PinId`/`NetIndex` API
   (done).
3. **M0.12.2** — report-only driver-side check (done).
4. **M0.12.3** — enforce `SingleSource` at generation time and candidate level
   (`MCHDL_PECA_ENFORCE=1`); OR stays report-only (done).
5. **M0.12.4** — legality-yield benchmark (done; see 10.7). Default-on remains
   blocked until the generator can find legal layouts.
6. **M0.11** — DCE-lite (plain DAG liveness in `prepare_place`).

### 10.6 M0.12.1/M0.12.2 measurements

On the `state_next` reproducer, with `MCHDL_DEBUG_PECA=1`:

- pre-route pin check: 2014 foreign-driver observations before routing
  (`candidate_drc_pre_route`);
- driver-side check: 1810 observations of a later terminal driving an existing
  foreign pin (`candidate_drc_driver_side`), for example
  `driver_node=20 driver=(11,2,3) pin_node=16 pin=(11,1,3)`.

Both directions are now quantified; the default path pays nothing because the
checks only run when the debug flag is set.

### 10.7 M0.12.3 enforcement and M0.12.4 legal yield

With `MCHDL_PECA_ENFORCE=1` the filter prunes at generation time: a torch
placement whose support has a foreign driver is dropped before routing
(`candidate_drc_pruned`), and a newly placed terminal that would drive an
existing foreign pin drops the branch. The candidate-level reject remains as a
safety net.

Legal yield on the `state_next` reproducer with enforcement on:

| Variant | Candidates | Pruned |
| --- | --- | --- |
| `full` | 0 | 239 |
| `no_dead` | 0 | 217 |
| `tail_n8` | 0 | 239 |
| `tail_n19` | 0 | 239 |
| `tail_n20` | 0 | 190 |
| `tail_n21` | 0 | 239 |
| `state_direct` | 2 | 143 |
| `n8_direct` | 0 | 210 |

The legacy placer cannot find a legal layout for the `state_next` shapes once
the shorted candidates are pruned (`tail_n8`/`tail_n19` previously "compiled"
only with hidden live shorts). Default-on is therefore blocked until the
generator gains placement freedom (orientation/distance strategies, larger
beam/box) or the verified macro library supplies clean layouts.

### 10.8 Placement-freedom experiment

Raising the torch strategy to `AnywhereNonAdjacent` (any distance > 1) with
enforcement on does not finish within a 16-minute budget: each NOT step
enumerates roughly `16*16*6*5 = 7680` `(position, direction)` candidates and
routes each one, so adding freedom by brute force explodes the search. The
reproducer exposes `MCHDL_REPRO_TORCH_STRATEGY=1` to switch strategies for
such measurements.

The next step is constraint-directed generation: enumerate legal support
positions first (no foreign driver, no occupied neighbour) and derive torch
placements from them, instead of enumerate-all-then-filter.
