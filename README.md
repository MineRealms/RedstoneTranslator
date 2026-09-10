# Redstone Compiler Project

Toolkit for Redstone Transfer Level Design.

## How to use

```text
cargo run --release -- input.v output.nbt        # compile Verilog
cargo run --release -- input.rcir output.nbt     # compile Logical or Routable RCIR
cargo run --release -- input.rsnap output.nbt    # replay a prepared PnR snapshot
```

Optional flags: `--intent design.rclayout`, `--cell-library library.json`,
`--mapping-policy mapping.json`, `--candidate-cache dir`.

Run the tests with `cargo test --release` (see `AGENTS.md` for the
memory-constrained test subset). Benchmarks live in `test/benchmarks/`.

See `docs/README.md` for the documentation index and `docs/roadmap.md` for the
current state and the CAD-style place-and-route migration plan.

## Compiler Stack

```
Verilog/SystemVerilog -> Logical IR (RCIR) -> Routable IR (RCIR) -> Place and Route -> World3D -> NBT
```

- Frontend and Logical IR: parse and elaborate the Verilog subset, preserve
  bus width and state intent (`docs/verilog_rtl_interface_design.md`).
- Routable IR: target-mapped scalar netlist with explicit ports, nets, and
  net classes (`docs/intermediate_representation_design.md`).
- Place And Route: local candidate generation plus global placement/routing.
  The CAD-style replacement (macro library, force-directed + simulated
  annealing placement, extracted A* routing engine, PathFinder congestion) is
  designed in `docs/architecture.md`.
- World, World3D: `World` and `World3D` are collections of blocks and
  positions, designed to correspond exactly to the Minecraft world. See
  [world/mod.rs](https://github.com/Redstone-Compiler/redstone-compiler/blob/master/src/world/mod.rs).
- NBT: a blueprint format that can be imported into Minecraft using
  [MCEdit](https://www.mcedit.net/),
  [Litematica](https://www.curseforge.com/minecraft/mc-mods/litematica) or
  similar.
