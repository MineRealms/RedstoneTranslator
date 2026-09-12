# Documentation Index

Entry point for project documentation. Every file has exactly one role: read
the current set first, treat historical files as background evidence, and do
not add new contracts to historical files.

## Reading order

1. `roadmap.md` — living plan and status log (read first).
2. `project_status.md` — pre-refactor snapshot plus CAD refactor progress.
3. `architecture.md` — CAD-style P&R migration design (branch `cad-refactor`).
4. `memory_refactor_plan.md` — M0.5 memory refactor plan and execution tracker (Commits 1-5 done; 6-7 pending).
5. `electrical_connectivity_analysis.md` — PECA (M0.10), the physical electrical fact layer.
6. `gpu_acceleration_plan.md` — CPU/GPU heterogeneous CAD plan (Candidate IR + GPU evaluator).
7. `candidate_reject_statistics.md` — M0.12.0 per-stage rejection counters (active plan).
8. The contracts and designs below for the existing pipeline.

## Current

| Document | Role |
| --- | --- |
| `roadmap.md` | Living roadmap, milestone order, status log, working agreements |
| `project_status.md` | Done / not done / precise blocker; hand-off snapshot |
| `performance_report.md` | Memory and compile-performance architecture snapshot plus M0.5 measured results |
| `memory_refactor_plan.md` | M0.5 memory refactor plan and execution tracker (instrumentation, copy-on-write worlds, budgets, diagnostics; Commits 6-7 pending) |
| `electrical_connectivity_analysis.md` | PECA: physical electrical facts + pin contracts + DRC (M0.10); the exclusivity check that was missing |
| `gpu_acceleration_plan.md` | CPU/GPU heterogeneous CAD plan: Candidate IR + GPU evaluator + CPU exact engine (G0-G4) |
| `candidate_reject_statistics.md` | M0.12.0 plan: per-stage candidate rejection counters that decide GPU/algorithm priorities |
| `architecture.md` | CAD migration design: macro library, placement, routing engine, PathFinder, compression |
| `intermediate_representation_design.md` | RCIR language contract (Logical/Routable IR, lowering, provenance) |
| `verilog_rtl_interface_design.md` | Verilog frontend pipeline and extension rules |
| `technology_mapping_design.md` | TargetSpec/MappingPolicy, general mapper, constants, cone partitioning |
| `cell_library_design.md` | CellLibrary/CellImplementation/contracts, CLI and snapshot integration |
| `physical_design_intent.md` | Floorplan/routing intent model and reusable local cell recipes |
| `compilation_snapshots.md` | Snapshot artifact layout and replay contract |
| `pnr_logging.md` | tracing conventions and log environment variables |
| `sequential_primitives.md` | RS/D latch representation and local placement constraints |

## Historical background

| Document | Status |
| --- | --- |
| `redstone_compiler_architecture.md` | Phase 0 repository analysis; a snapshot at commit `cc99773`, kept as evidence for the migration decisions |
| `hierarchical_placer_design.md` | Rationale for macro composition and staged P&R; the new flow adopts it, `architecture.md` is authoritative |

## Removed in the 2026-09 documentation cleanup

- `local_placer_improvement.md`: working notes for improving the old beam-search
  placer. That engine is being replaced by the CAD flow; the evidence lives in
  `project_status.md` §4.
- `global_routing_visualization.html`: a one-off explainer of an old DFF routing
  failure. Superseded by `project_status.md` §4 and `architecture.md` §6/§8.

## Deletion policy

Delete a document only when the code or contract it describes is removed.
Mark a file as historical here before deleting it.
