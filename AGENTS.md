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
adds per `World3D` clone). `--placement-engine legacy|annealed` selects the
global placement engine.

Local search limits (deterministic; exceeding one reports an error):
`MCHDL_FRONTIER_CAP` (default 16,384 frontier entries per step) and
`MCHDL_LOCAL_CLONE_LIMIT` (default 10M `World3D` clones per local search).

Debug diagnostics (one-line summaries, off by default):

- `MCHDL_DEBUG_TRUTH_TABLE=1`: first candidate truth-table mismatch (mask,
  positions, expected vs actual) and a one-time leaf graph dump.
- `MCHDL_DEBUG_ANNEALED=1`: annealed placement attempts and box decisions.
- `MCHDL_DEBUG_PLACEMENT=1`: global placement macro positions and cost.
- `MCHDL_DEBUG_INPUT_SWITCH=1`: external input switch construction.
- `MCHDL_DEBUG_CONNECTIVITY=1`: per-candidate physical dump (endpoint map,
  signal footprints, non-air blocks) for the local placer.
- `MCHDL_DEBUG_PECA=1`: PECA (physical electrical connectivity) violation
  reports, printed before truth-table validation.
- `MCHDL_PECA_ENFORCE=1`: enforce PECA `Certain` (`Single`) violations:
  generation-time pruning (pre-route and driver-side) plus the candidate-level
  reject before truth-table validation (opt-in; default is report-only).

The ignored test `state_next_graph_candidate_truth_reproducer`
(`global_pnr/candidate.rs`) reproduces the `state_next` truth-table rejection;
run it with `--ignored --nocapture` when working on the leaf realizer.
