# Agent Notes

## Documentation

- Project roadmap and tracking (read first): `docs/roadmap.md`
- Project status report (done / not done / current blocker): `docs/project_status.md`
- CAD-style P&R migration design (branch `cad-refactor`): `docs/architecture.md`
- Repository architecture analysis (Phase 0 report): `docs/redstone_compiler_architecture.md`
- Cell library and physical contract design: `docs/cell_library_design.md`
- Target capability and mapping policy design: `docs/technology_mapping_design.md`
- Verilog RTL interface design notes: `docs/verilog_rtl_interface_design.md`
- RCIR language and lowering design: `docs/intermediate_representation_design.md`
- Physical design intent and local-cell recipes: `docs/physical_design_intent.md`
- Physical design intent and local cell recipes: `docs/physical_design_intent.md`
- PnR logging and observability: `docs/pnr_logging.md`
- Compilation snapshot artifacts: `docs/compilation_snapshots.md`

When asked to create or preserve project documentation, add an appropriate file under `docs/` and link it from this file when it is useful for future agents.

## Git

When committing changes, include the intent behind the change in the commit message body.

## Testing

Run local placer and place-and-route tests with `cargo test --release`; debug builds are too slow for these search-heavy tests.
