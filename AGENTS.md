# Agent Notes

## Documentation

- Documentation index (read first): `docs/README.md`
- Living roadmap and status log: `docs/roadmap.md`
- CAD-style P&R migration design: `docs/architecture.md`

When asked to create or preserve project documentation, add an appropriate file under `docs/` and register it in `docs/README.md` when it is useful for future agents.

## Git

When committing changes, include the intent behind the change in the commit message body.

## Testing

Run local placer and place-and-route tests with `cargo test --release`; debug builds are too slow for these search-heavy tests.

On memory-constrained machines (e.g. 32 GB), the eight search-heavy `test_generate_component_*` tests can OOM the process. Use a single test thread, or skip them:

```text
cargo test --release -- --test-threads=1
cargo test --release -- --skip test_generate_component --test-threads=1
```

## Build memory

This crate's rustc and linker peak well above 4 GB per job. On a 32 GB machine use at most `-j 2` and prefer `-j 1` for release builds; never run several cargo commands at once.

```text
cargo check --lib -j 1
cargo test --release --lib -j 2 -- --skip test_generate_component --test-threads=1
```

Full-flow PnR benchmark tests (`benchmark_pnr_baseline`, `benchmark_placement_engines_baseline`) are manual and can exhaust 32 GB. Run one benchmark per process with `MCHDL_BENCH=<name>` and only on a larger machine.

## Performance instrumentation

`MCHDL_PERF=1` prints per-stage world-clone counts, cloned bytes, and RSS, plus
a final summary. `--memory-budget-mb <N>` turns the budget into a clear error
instead of an OOM abort. The counters are always active (two relaxed atomic
adds per `World3D` clone). `MCHDL_DEBUG_TRUTH_TABLE=1` prints the first
candidate truth-table mismatch (mask, input/output positions, expected vs
actual) and dumps the leaf graph once per process; `--placement-engine
legacy|annealed` selects the global placement engine.
