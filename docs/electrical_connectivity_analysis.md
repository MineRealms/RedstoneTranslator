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
  belong to the same electrical unit as `n`'s anchor (its terminal, or the
  redstone component driven by it). This is the exclusivity check.
- `Merge(a, b)` — an OR tap: the tap's redstone component must have exactly
  `{a, b}` as its driving nets (both branches reach, no foreign net).
- `Passive` — sequential-macro internals are skipped in the first release.

The judgment unit is the **electrical component** (redstone dust network plus
its driving terminals), never a raw position, because fanout and wired-OR make
multi-source components legal.

## 6. Driver confidence

`Always` (redstone block), `Possible` (switch/torch/repeater/powered wire),
`Never` (proven constant-off after DCE/constant propagation). The first
release treats every driver as `Possible` and reports without rejecting.

## 7. Rollout (report-first)

1. **M0.10a** — report only: run PECA before truth-table validation, print
   `[peca]` violations and count them (`candidate_drc_violations`). Metrics:
   violation count, candidate survival rate, truth-reject delta, false-positive
   audit against the truth table.
2. **M0.10b** — enforce: reject violating candidates.
3. **M0.11** — DCE of dead logic (separate concern; shrinks the `Never` set and
   the search space).
4. **M0.12** — generation-time rejection (support-position local check, OR tap
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

### 9.2 Known limitation: OR tap reachability false positives

`Merge` reports `MissingBranch` on valid OR merges (for example `tail_n8`
`node=17 MissingBranch(16)` although the candidate passes the truth table).
The static component walk does not yet capture every path by which a branch
can reach the tap (notably cobble-mediated hops). This must be fixed before
M0.10b enforces violations, otherwise valid candidates would be rejected.
`Single` checks are not affected by this gap and already match the confirmed
root cause.
