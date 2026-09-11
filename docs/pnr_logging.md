# PnR logging

Production placement, routing, and simulation code uses `tracing`. It must not
write progress directly with `println!` or `eprintln!`; callers that do not
install a tracing subscriber remain silent.

## Levels

- `info`: phase transitions, phase timing summaries, selected solution cost,
  long-running routing heartbeats, and the final result. Routing heartbeats are
  emitted at completed-attempt boundaries after roughly ten seconds of work.
- `debug`: layout combinations, routing attempts, cache reuse, and individual
  failure reasons.
- `trace`: local placer steps, candidate counts within a step, individual nets
  and sinks, and simulator events.
- `warn`: exhausted search budgets, fallback behavior, and aggregated terminal
  search failure.
- `error`: unrecoverable failures at an application boundary.

`GlobalPnrConfig::show_progress` controls whether global PnR tracing events are
emitted. It does not print to stderr directly. Its default remains `true`.

## Running the counter smoke test

The ignored sequential smoke tests install an `info` subscriber by default:

```powershell
cargo test --release counter_module_generates_world_from_child_layout_candidates -- --ignored --nocapture
```

Set `RUST_LOG` when more or less detail is needed:

```powershell
$env:RUST_LOG = "debug"
cargo test --release counter_module_generates_world_from_child_layout_candidates -- --ignored --nocapture

$env:RUST_LOG = "trace"
cargo test --release counter_module_generates_world_from_child_layout_candidates -- --ignored --nocapture
```

Library consumers are responsible for installing and configuring their own
subscriber. The compiler CLI installs a formatted subscriber whose default
level is `info`.

## Performance and debug environment variables

These are separate from `RUST_LOG`; they print one-line diagnostics or counters
to stderr and are documented in `AGENTS.md` and `performance_report.md`.

| Variable | Effect |
| --- | --- |
| `MCHDL_PERF=1` | Per-stage world-clone counts, cloned bytes, RSS, final summary |
| `MCHDL_DEBUG_TRUTH_TABLE=1` | First candidate truth-table mismatch plus a leaf graph dump |
| `MCHDL_DEBUG_ANNEALED=1` | Annealed placement attempts and box decisions |
| `MCHDL_DEBUG_PLACEMENT=1` | Global placement macro positions and cost |
| `MCHDL_DEBUG_INPUT_SWITCH=1` | External input switch construction |
| `MCHDL_BENCH=<name>` | Run one manual full-flow benchmark per process |
| `MCHDL_FRONTIER_CAP=<N>` | Local search frontier cap per step (default 16,384) |
| `MCHDL_LOCAL_CLONE_LIMIT=<N>` | Local search `World3D` clone limit (default 10M) |
