# Technology mapping and the general Logical-to-Routable mapper

## Status

Step 1 of the post-Phase-0 roadmap is implemented: a target/policy-aware
lowering path (`src/ir/mapping/`) now handles flat logical modules with buses,
arithmetic, muxes, multiple state cells, and negative-edge state, and splits
large combinational cones into placeable leaves. The legacy special-case
lowering in `src/ir/logical_lowering.rs` is kept as the primary path for the
historical counter/DFF shapes so existing snapshots and structure assertions
stay stable; the general mapper takes over whenever the legacy path cannot
express the module.

## Target capabilities

`src/ir/target.rs` defines:

```rust
TargetSpec { name, capabilities: BTreeSet<TargetOp> }
TargetOp   = Buffer | Not | And | Or | Xor | DLatch | RsLatch
```

- `TargetSpec::redstone_v1()` is the capability set of the current physical
  flow and supports every operation.
- `TargetSpec::require(op)` is called before each primitive is emitted, so a
  reduced target fails with an object-specific diagnostic instead of producing
  an unsupported Routable design.
- The `target` string in `RoutableDesign` is still validated against
  `redstone-v1`; extending the validator to arbitrary target names is future
  work.

## Mapping policy

`MappingPolicy` makes implementation choices explicit and separable from IR
semantics:

| Field | Variants | Meaning |
| --- | --- | --- |
| `register` | `MasterSlaveLatches` | DFF/register = master/slave D latches + inverted clock |
| `adder` | `RippleCarry` | `Add`/`Inc` = per-bit xor/and/or carry chain |
| `mux` | `AndOrNot` | `result = (when_true & select) \| (when_false & ~select)` |
| `xor` | `Direct`, `AndOrNot` | native xor primitive or `(a \| b) & ~(a & b)` |

`MappingPolicy::validate(&target)` rejects a policy whose chosen
implementation the target cannot build (for example `xor = Direct` without the
`Xor` capability) before any circuit is lowered.

## General lowering pipeline

Entry point: `LogicalDesign::lower_to_routable_with_target(&target, &policy)`.
Internally `ir::mapping::lower_flat_module` branches on state:

```text
flat logical module (no instances)
  ├─ no state cells  -> one bit-blasted combinational cone
  └─ state cells     -> live-state filtering
                        -> one next-state cone per live state cell
                        -> clk inverter + master/slave latch leaf per DFF bit
                        -> one latch leaf per D latch bit
                        -> one combinational output cone
                        -> cone partitioning (see below)
                        -> top composite wires typed nets
```

### Cone partitioning

`LeafBuilder::partition` (`src/ir/mapping/scalar.rs`) splits a combinational
cone into leaves that fit the local placer's hard limit of 40 graph nodes:

1. Build the cone as one virtual node list, then compute a deterministic Kahn
   topological order.
2. Greedily assign logic nodes to chunks with a budget of
   `LEAF_NODE_BUDGET = 28` counted over logic nodes and external inputs.
3. Recompute each chunk's real node count (logic + inputs + outputs) and split
   any chunk above `PARTITION_NODE_LIMIT = 40` at its topological midpoint
   until every chunk fits.
4. Cross-chunk producers become `__t{id}` intermediate ports; requested
   outputs keep their names. A cone that fits in one chunk stays a single leaf
   named exactly as before.

A 4-bit add stays one leaf; a 32-bit add register lowers to several
`{q}_next_p{n}` leaves, each below the placer limit, and the design still
resolves to a `ResolvedPnrTopology`.

### Bit-blasting

`ScalarNets` maps `(net, bit)` to the scalar name used by Routable IR. Scalar
nets keep their logical name; vectors use the same `name_bit` spelling that the
Verilog frontend already produces for `name[bit]` selects.

### Combinational expansion

`ir::mapping::combinational::emit_cell` expands each cell per bit:

- `Buffer` is a wire alias (no node),
- `Not`/`And`/`Or`/`Xor` emit scalar primitives (xor follows `policy.xor`),
- `Eq { width }` emits a per-bit XNOR followed by an AND reduction,
- `Add` emits a full-adder chain with shared carry nodes,
- `Inc` emits `~bit0` and an `and`/`xor` carry chain,
- `Mux` emits the and/or/not form from `policy.mux`.

Constants are rejected with a stage-specific error until the Routable IR gains
a constant node.

### State decomposition

`ir::mapping::sequential`:

1. Computes live state cells: backward reachability from module outputs,
   following the data net of every live state cell to a fixed point. State
   cells whose output cannot reach an output are eliminated, so no instance
   port is ever left unconnected.
2. Names every element before wiring (`{q}_next`, `{q}_clk_inv`, `{q}_master`,
   `{q}_slave`, `{q}_latch`, with `_{bit}` for vectors) so state-to-state
   connections resolve forward references.
3. Builds the next-state cone on demand (`ConeEmitter`). Boundary nets
   (module inputs and state outputs) become leaf inputs; the leaf output uses
   the reserved `__next` prefix so it cannot collide with a boundary net name.
4. Wires the composite: clock, inverted clock, master/slave enables (swapped
   for `negedge`), next-state data, and module outputs. Input ports that no
   emitted cone consumes are dropped from the Routable interface.

### Constants

Logical constants are supported end to end:

- `RoutableNodeKind::Constant { value }` is a scalar source node with no
  inputs (`src/ir/routable.rs`), serialized as `node N constant 0|1;`.
- The general mapper materializes constants at their use sites
  (`LeafBuilder::constant`). Constant **one** becomes a `Constant(true)` node;
  constant **zero** is expanded to `not(Constant(true))`, so the physical flow
  only ever places powered sources.
- `LocalPlacer` places `Constant(true)` as a `RedstoneBlock`
  (`constant_node_kind`), which the router already treats as a powered source.
  The expanded zero is a torch held off by that block, which the route power
  contract accepts because the source settles inactive.
- Constant operands are zero-extended to the operation width, matching the
  logical width rules, and constant state data (`q <= 0`) lowers to a
  next-state leaf containing the constant.

This unblocks `q <= 0`/`q <= 1`, constant mux branches, masks, and constant
module outputs. Constant folding/simplification (for example `and(x, 1) -> x`)
is not implemented yet; constants are materialized rather than optimized away.

## Persistence

`MappingSpec` (`redstone-compiler.mapping.v1`) is a versioned JSON document
holding the target name and the mapping policy. It can be supplied with
`--mapping-policy path.json` for `.v` and logical `.rcir` inputs, and
`place_and_route_logical_design_with_mapping` records it in the snapshot as
`ir/mapping.json`, so the lowering configuration that produced a design is
recoverable. The default spec is `redstone-v1` with the default policy.

## Compatibility and migration

Dispatch in `lower_logical_design`:

1. `lower_state_design` handles the historical single-state shapes only when
   `module_is_simple_state_design` is true (all outputs state-driven, all
   combinational cells inside the next-state cone).
2. `requires_general_flat_lowering` forces the new mapper for buses, two or
   more state cells, or `Add`/`Inc`/`Mux`/`Dff`/`Register` cells.
3. Otherwise the legacy scalar leaf writer runs; if it fails, the general
   mapper is retried and its error is reported with the legacy failure as
   context.

This ordering keeps every existing test and snapshot structure intact while
new capabilities flow through the general path. Later steps can invert the
order and delete the special cases once structural compatibility is no longer
required.

## Known limits of this slice

- Constants are materialized as redstone blocks during lowering and folded
  before placement in `prepare_place`
  (`src/transform/logic/fold_constants.rs`), which removes identity/absorbing
  operations and constant-fed Or patterns.
- Constant sources are physical redstone blocks; `world_to_logic` extraction
  does not map them back to constant nodes yet, so verifier round-trips on
  constant designs are not available.
- Nested hierarchy is flattened before lowering
  (`LogicalDesign::flatten_hierarchy`); the PnR flow then accepts one level of
  scalar leaf children.
- Partitioning keeps every leaf under the local placer node limit, but chunk
  boundaries are chosen greedily from topology; placement feedback is not yet
  fed back into the partition.
- Shared sub-cones are duplicated across state cells instead of partitioned
  once and shared.
- `LoweringMap` provenance (many-to-many logical-to-routable mapping) is still
  represented by `IrDebugInfo` only.
- `MappingPolicy` is embedded via `MappingSpec` (`ir/mapping.json` in
  snapshots and the RCIR mapping profile).

## Tests

`src/ir/target.rs`, `src/ir/mapping/{scalar,combinational,sequential,mod}.rs`:

- capability/policy validation,
- vector `and`/`add`/`mux` lowering verified against full truth tables,
- `xor = AndOrNot` removes native xor nodes,
- enabled DFF lowers with a mux next-state leaf and resolves to a PnR topology,
- `negedge` swaps master/slave enables,
- chained registers wire directly without next leaves,
- dead registers are eliminated,
- constant operands lower and match truth tables; constant zero expands
  through an inverter; `q <= 0` produces a constant next-state leaf,
- `LocalPlacer` materializes `Constant(true)` as a redstone block,
- a long chain splits into connected chunks under budget,
- a small cone stays in one named leaf,
- 32-bit combinational add and 32-bit register next state partition into
  multiple leaves, each below the local placer limit, and still resolve to a
  PnR topology.

Run with `cargo test --release` (see AGENTS.md); the search-heavy local placer
component tests can be skipped with `--skip test_generate_component` on
memory-constrained machines.
