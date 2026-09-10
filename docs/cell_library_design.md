# Cell library design

## Status

Step 2b of the roadmap: a reusable cell implementation model exists in
`src/transform/place_and_route/global_pnr/cell_library.rs`. It is wired into
candidate policy resolution and therefore into candidate generation and the
preparation fingerprint. The built-in `redstone-v1` library starts empty
because the current flow derives policies from the design; designs and targets
add named implementations on top.

## Model

```text
CellLibrary
  format: "redstone-compiler.cell-library.v1"
  name, target
  implementations: [CellImplementation]

CellImplementation
  name:        variant name, e.g. "d_latch.compact"
  definitions: Routable definition names it applies to
  candidate:   UnitCandidateConfig (search box, local placer config, limits)
  contract:    CellPhysicalContract
  priority:    higher wins

CellPhysicalContract
  halo
  requires_input_isolation / requires_output_isolation
  allowed_transforms: [Translation, Yaw]
  max_delay

CellTransform = Translation | Yaw
```

The contract describes only what global P&R may rely on. Per-design floorplan
intent stays in `PhysicalIntent`; the library never contains coordinates.

## Resolution

`CandidatePolicySet::effective_for_definition(definition)` resolves a candidate
policy with this precedence:

1. explicit definition override (`@pnr.candidate` profile or
   `with_definition_override`),
2. best matching cell library implementation (highest priority, ties by
   lexicographically smallest name),
3. the policy set default.

Because `normalized_candidate_policies` and `prepare_config_fingerprint` are
computed from the resolved policies, adding or changing a library entry
automatically invalidates prepared snapshots and persistent candidate caches
that were produced under a different library.

## Serialization

`CellLibrary::to_json` / `CellLibrary::from_json` use a versioned JSON DTO that
reuses `CandidateSpec` from the RCIR sidecar language, so a library entry has
exactly the same expressiveness as an embedded `@pnr.candidate` profile.
`from_json` rejects any other format string. A dedicated `*.rcell` text syntax
can be added later; JSON is the stable artifact for now.

## Contract consumption

The contract is resolved from the library by definition name and is applied
during candidate generation and global placement:

- `requires_input_isolation` / `requires_output_isolation` force the
  corresponding `PortConnection::InputDiode` / `OutputDiode` on generated
  candidate ports, in addition to the sequential-leaf default.
- `halo` is recorded on every generated `LayoutCandidate` and reserved by
  global placement: shelf and grid heuristics size slots with
  `LayoutCandidate::placement_bbox()`, and overlap validation inflates each
  placed module by its halo. The physical world and assembled blocks are
  unchanged.
- The contract is part of both the preparation fingerprint and the persistent
  candidate cache key (`routable-local-candidate-cache-v3`), so a contract-only
  change invalidates prepared snapshots and cached candidates.
- `allowed_transforms` and `max_delay` are recorded but not consumed yet:
  placement does not transform candidates, and routing has no delay model.

## Not implemented yet

- Named implementation variants of the logical target mapping
  (`std.xor -> xor.nor_network`, `xor.buffered`, ...). Today a variant is a
  candidate policy for one Routable definition, not a different internal
  implementation graph.
- Recipe constraints (port faces, ordering, corridors) and Pareto objectives
  per recipe; see `physical_design_intent.md`.
- Auto-populated built-ins for the special cases the compiler already knows
  (for example the D-latch child policy).

## Tests

- built-in library is empty and target-scoped,
- selection prefers priority then smallest name,
- JSON round-trip preserves candidate policy and contract,
- unsupported format strings are rejected,
- a library entry overrides the default policy but not an explicit
  definition override,
- contract resolution returns the library contract for matching definitions,
- generated candidates carry the contract halo and forced diode connections,
- placement overlap validation rejects slots that violate a halo.
