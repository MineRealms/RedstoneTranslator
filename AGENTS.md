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
