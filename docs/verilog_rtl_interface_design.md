# Verilog RTL interface design

## Purpose

The Verilog frontend preserves hardware intent long enough for target mapping
and physical design to make deliberate choices. It does not construct the
physical hierarchy used by PnR directly.

The supported pipeline is:

```text
Verilog source
  -> lexer / parser AST
  -> RTL process model
  -> synthesis cells
  -> LogicalDesign
  -> direct target mapping
  -> RoutableDesign
  -> PreparedPnrDesign
  -> global placement and routing
  -> NBT + compilation snapshot
```

`LogicGraph` remains available for small combinational expressions and as the
node-level input to local placement. It is not a circuit-wide hierarchy or net
identity model.

## Stage responsibilities

### Verilog AST and RTL

The parser owns source syntax. RTL lowering owns procedural semantics such as
clock edges, nonblocking assignments, latch inference, and next-state
expressions. Neither stage knows about Minecraft placement or routing.

### Logical IR

`LogicalDesign` is target-independent and bus-aware. It preserves:

- module definitions and typed instances;
- named nets and vector widths;
- combinational operations;
- registers, DFFs, latches, clock edges, and next-state intent;
- source/provenance locations.

Verilog input and textual Logical RCIR input converge at this boundary.

### Routable IR

Logical lowering constructs `RoutableDesign` directly. Target mapping may
bit-blast buses and expand a register into inverter, next-state, master-latch,
and slave-latch leaves. The result owns explicit definitions, instances, ports,
nets, net classes, and scalar leaf nodes.

The old intermediate graph hierarchy has been removed. In particular, the
compiler must not reconstruct another hierarchy and then convert it back into
Routable IR. This keeps instance identity, fanout, net names, and provenance in
one typed model.

### Local and global PnR

Local placement receives one Routable leaf at a time. Only its scalar node body
is adapted to `Graph`; its typed ports stay authoritative for candidate I/O.

Global PnR receives a validated `RoutableDesign`, resolves it to
`ResolvedPnrTopology`, prepares reusable candidate sets, and then performs
placement and routing. Routing branches carry typed net and endpoint IDs rather
than recovering connectivity from display names.

## Extension rules

When adding Verilog support:

1. Parse syntax without assigning physical meaning in the parser.
2. Represent procedural behavior in RTL/synthesis types.
3. Preserve buses and state intent in Logical IR.
4. Add an explicit target-mapping rule from Logical to Routable IR.
5. Validate both IR stages independently.
6. Add source-map coverage and deterministic text round-trip tests.
7. Exercise local placement only after the Routable leaf schema is stable.

New state elements or arithmetic operations should become Logical cell kinds
before they become redstone macros. New Minecraft implementations should be
selectable target mappings or candidate profiles, not new Verilog syntax.

## Current supported vertical slice

The implemented path covers small combinational modules, structural hierarchy,
D latches, positive-edge and negative-edge DFF expansion, incrementing
registers, and procedural `if`/`else`/`case` state machines.

Frontend details:

- `always` bodies may contain multiple statements in `begin`/`end` blocks.
  Single-statement blocks are normalized away so the legacy AST shape is kept.
- `if (...) ... else ...` and `case (...)`/`default` are parsed. `case` is
  expanded into a priority `if`/`else` chain of equality tests (`Eq`) against
  the selector. `if` conditions are full expressions.
- `==` and `!=` are supported in expressions and lower through `Eq`
  (`!=` becomes `not(Eq)`).
- ANSI-style headers are supported:
  `module m(input clk, input [3:0] a, output reg q, output y);`.
- Single-bit selects of declared vectors are supported on expression
  right-hand sides (`assign y = a[2];`, `q <= a[1];`). `LogicalValue::Slice`
  carries the bit into lowering, where the mapper resolves it to the scalar
  bit. Slices on assignment left-hand sides or instance connections are still
  rejected.
- `always @(posedge clk)` and `always @(negedge clk)` are supported; the
  mapper places master/slave latches with swapped enables for negative edges.
- Bare `reg` declarations declare internal signals.
- A clocked process may assign several signals; each becomes its own DFF or
  register with a per-signal next-value expression. Later assignments take
  priority and unassigned paths hold the previous value.
- A combinational process whose signals are assigned on every path (an
  `if`/`else` or a `case` with `default`) lowers to pure combinational logic
  (`SynthCell::Combinational`). A partially assigned signal still uses the
  legacy D-latch pattern; other partial shapes fail with a diagnostic.

Unsupported constructs must fail during validation or target mapping with a
stage-specific diagnostic.

See also:

- `docs/intermediate_representation_design.md`
- `docs/physical_design_intent.md`
- `docs/compilation_snapshots.md`
