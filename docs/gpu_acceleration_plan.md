# CPU/GPU Heterogeneous CAD Plan

Status: design note · Branch: `cad-refactor` · Related: M0.12, G1-G4

## 1. Principle

The compiler is a discrete-physics CAD flow, not a SIMT workload. The
division of labour:

- **CPU**: irregular search, state management, exact physical construction,
  and verification (the oracle).
- **GPU**: cheap, deterministic, massively parallel filtering and scoring of
  large candidate batches.

Do not GPU-ify the beam queue, A*, or the event-driven simulator. They are
control-flow-heavy and divergent; the win would be negative after transfer and
sync overhead.

## 2. Target architecture

```
Routable IR
  -> Hierarchical flow (leaf modules <= 40 nodes)
  -> CPU Beam Controller        frontier: Vec<(World3D, PlacementState)>
       step(node)
  -> Candidate Generator        enumerate (position, direction) only; no route
  -> Candidate IR (flat batch)
  -> GPU Candidate Evaluator    occupancy, local electrical risk, spacing,
                                cheap route-cost estimate  -> valid + u32 score
  -> CPU Exact Physical Engine  place, route, PECA, world mutation
  -> Candidate Validation       simulator + truth table
  -> next frontier
```

The GPU sits between enumeration and routing. Today the CPU routes every
enumerated placement and only ~2.5% survive (`not_chain` step 2: ~2880
placements -> 71 routed). The evaluator exists to stop routing dead candidates.

## 3. Candidate IR

Enumeration must stop mutating the world. Introduce a compile-time record:

```rust
struct PlacementCandidate {
    entry_id: usize,        // frontier entry that produced it
    node: NodeId,
    kind: CandidateKind,    // Not | Input | Constant | OrTap | ...
    torch_pos: Position,
    direction: Direction,
    support_pos: Position,
    source_pos: Position,   // expected driver terminal
}
```

Batch boundary: flatten `(entry x candidates)` for the current step into one
batch (tens of thousands of records is trivial). Per-entry batches are too
small for the GPU.

The GPU sees a packed, integer-only snapshot, never `World3D`:

```rust
#[repr(C)]
struct GpuCandidate {
    entry_id: u32,
    torch: [i16; 3],
    support: [i16; 3],
    direction: u8,
    neighbor_kind: [u8; 6],
    neighbor_power: [u8; 6],
    source_distance: u16,
}
```

## 4. Evaluator interface

```rust
trait CandidateEvaluator {
    fn evaluate(
        &self,
        candidates: &[PlacementCandidate],
        world: &WorldSnapshot,       // packed SoA, stage-local
    ) -> Vec<CandidateScore>;
}

struct CandidateScore {
    valid: bool,          // conservative hard reject
    drc_penalty: u32,
    estimated_route_cost: u32,
}
```

- `CpuCandidateEvaluator` is the default and the reference implementation.
- `WgpuCandidateEvaluator` is opt-in (`--features gpu`, `MCHDL_GPU=1`).
- The CPU stable-sorts by `(valid, penalty, estimated_route_cost, position,
  direction)` and keeps a top-K per entry before the exact router runs.

## 5. GPU kernel scope (G1)

Conservative local checks only:

1. **Occupancy**: the torch and support cells are free and in bounds.
2. **Local electrical risk**: no foreign power source in the support's six
   neighbours (the `tail_n20` defect class). The authoritative check remains
   the CPU `check_pin` with full `cobble_power_sources`.
3. **Spacing / margin**: halo and channel constraints.
4. **Route-cost estimate**: a multi-source BFS distance field from the source;
   used for ranking, never for hard rejection (redstone decay and
   directionality make it an approximation).

## 6. Determinism

The project requires deterministic outputs.

- Integer scores only (`u32`); no floating-point reductions in the decision.
- No atomics for accept/reject; write per-candidate results, reduce on CPU.
- Fixed batch order and stable sorts; CPU tie-breaks on position/direction.
- GPU paths are feature-gated and off by default; snapshots and candidate
  fingerprints must not depend on GPU scheduling.

## 7. Toolchain

`wgpu` (WGSL compute over DX12/Vulkan) is the first backend: Rust-native,
works with the installed NVIDIA driver, no CUDA toolkit required, portable to
other vendors and the browser. A CUDA backend can be added behind the same
trait if a later stage (for example route fields) needs more performance.

## 8. Phases

1. **G0 (no GPU)** — `M0.12.0` reject statistics (geometry / electrical /
   routing / simulation counters per benchmark) and `M0.12.5`
   constraint-directed enumeration (generate legal supports first; the
   `AnywhereNonAdjacent` experiment shows brute-force freedom explodes to
   ~7680 placements per NOT). Target: ~2880 -> a few hundred candidates.
2. **G1** — Candidate IR + CPU evaluator refactor (done, `03025be`), then the
   wgpu evaluator behind the feature flag (done): build with
   `--features gpu` and run with `MCHDL_GPU=1`. The WGSL kernel mirrors the CPU
   evaluator exactly (cell validity plus the foreign-driver penalty); the
   differential test passes on the RTX 4060 and `not_chain` compiles to the
   identical NBT with the GPU path. Note: wgpu initialization adds roughly
   250 MiB RSS, so the memory budget checks see a higher baseline.
3. **G2** — GPU batch scoring for simulated annealing moves (`sa_placer`).
4. **G3** — GPU route fields / PathFinder congestion maps at the global
   level, where the batch (instances x nets x iterations) is large.
5. **G4** — GPU DC pre-filter for truth tables, last; the CPU simulator stays
   the oracle.

## 9. Interaction with other work

- **Macro cache (M0.5 Commit 6)**: repeated leaf shapes skip search, which
  lowers the value of GPU leaf filtering over time. The large future batch is
  top-level assembly (many instances, nets, and routing iterations), which is
  where G2/G3 matter.
- **Hierarchy**: leaves are capped at 40 prepared nodes
  (`K_MAX_LOCAL_PLACE_NODE_COUNT`), so a single 10k-gate leaf is not a target;
  the flow is hierarchical by construction.
- **PECA**: the GPU evaluator is a conservative pre-filter; PECA, the exact
  router, and the simulator remain the correctness authorities.
