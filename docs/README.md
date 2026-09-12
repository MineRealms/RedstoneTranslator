# Documentation Index

Entry point for project documentation. Read the current set first; historical
files are background evidence only. Do not add new contracts to historical
files.

## Reading order

1. `roadmap.md` — living plan, milestone order, status log, working agreements.
2. `architecture.md` — CAD-style P&R design and milestone status.
3. `electrical_connectivity_analysis.md` — PECA (M0.10), the physical electrical fact layer.
4. `gpu_acceleration_plan.md` — CPU/GPU heterogeneous CAD plan (G0-G4).
5. `performance_report.md` — memory/compile measurements, including the M0.5, M0.12 and GPU results.
6. The contracts and designs below for the existing pipeline.

## Current

| Document | Role |
| --- | --- |
| `roadmap.md` | Living roadmap, milestone order, status log, working agreements |
| `architecture.md` | CAD migration design: macro library, placement, routing engine, PathFinder, compression |
| `electrical_connectivity_analysis.md` | PECA: physical electrical facts + pin contracts + DRC (M0.10) |
| `gpu_acceleration_plan.md` | CPU/GPU heterogeneous CAD plan: Candidate IR + GPU evaluator + CPU exact engine (G0-G4) |
| `performance_report.md` | Memory and compile-performance measurements (M0.5 memory work, M0.12 reject statistics, GPU evaluator) |
| `intermediate_representation_design.md` | RCIR language contract (Logical/Routable IR, lowering, provenance) |
| `verilog_rtl_interface_design.md` | Verilog frontend pipeline and extension rules |
| `technology_mapping_design.md` | TargetSpec/MappingPolicy, general mapper, constants, cone partitioning |
| `cell_library_design.md` | CellLibrary/CellImplementation/contracts, CLI and snapshot integration |
| `physical_design_intent.md` | Floorplan/routing intent model and reusable local cell recipes |
| `compilation_snapshots.md` | Snapshot artifact layout and replay contract |
| `pnr_logging.md` | tracing conventions and log environment variables |
| `sequential_primitives.md` | RS/D latch representation and local placement constraints |

## Removed in the 2026-09 documentation cleanup

- `local_placer_improvement.md`, `global_routing_visualization.html`: working
  notes for the old beam-search placer and a one-off DFF routing explainer;
  superseded by the CAD flow and `architecture.md`.
- `project_status.md`: pre-refactor hand-off snapshot; the still-relevant
  blockers now live in the "Known gaps and risks" section of `roadmap.md`.
- `memory_refactor_plan.md`: M0.5 execution tracker; the measured results moved
  to `performance_report.md` and the pending items to `roadmap.md`.
- `candidate_reject_statistics.md`: M0.12.0 tracker; the counter set is in the
  code (`perf.rs`) and the measurements moved to `performance_report.md`.
- `redstone_compiler_architecture.md`, `hierarchical_placer_design.md`: Phase 0
  analysis and early placer rationale; superseded by `architecture.md`.

## Deletion policy

Delete a document when the code, contract, or plan it describes is gone or has
been fully absorbed into a living document. Record the removal here.
