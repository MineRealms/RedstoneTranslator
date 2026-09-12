// Candidate evaluation kernel. Mirrors candidate_eval::evaluate_one:
//  - valid  = torch cell is air and support cell is air or cobble (in bounds)
//  - penalty = 100 per foreign power source next to the support
//  - cost   = manhattan(source, support)
// Cell codes: 0 air, 1 cobble, 2 redstone, 3 torch, 4 repeater, 5 switch,
// 6 redstone block, 7 other.

struct Params {
    dims: vec3<u32>,
    count: u32,
};

@group(0) @binding(0) var<uniform> params: Params;
@group(0) @binding(1) var<storage, read> cells: array<u32>;
@group(0) @binding(2) var<storage, read> candidates: array<u32>;
@group(0) @binding(3) var<storage, read_write> scores: array<u32>;

fn cell_at(p: vec3<u32>) -> u32 {
    if (p.x >= params.dims.x || p.y >= params.dims.y || p.z >= params.dims.z) {
        return 999u;
    }
    let index = p.x + p.y * params.dims.x + p.z * params.dims.x * params.dims.y;
    return cells[index];
}

fn neighbor_penalty(support: vec3<u32>, source: vec3<u32>, shift: vec3<i32>) -> u32 {
    let n = vec3<i32>(support) + shift;
    if (n.x < 0 || n.y < 0 || n.z < 0) {
        return 0u;
    }
    let neighbor = vec3<u32>(n);
    if (all(neighbor == source)) {
        return 0u;
    }
    let cell = cell_at(neighbor);
    if (cell == 3u || cell == 4u || cell == 5u || cell == 6u) {
        return 100u;
    }
    return 0u;
}

@compute @workgroup_size(64)
fn main(@builtin(global_invocation_id) gid: vec3<u32>) {
    let i = gid.x;
    if (i >= params.count) {
        return;
    }

    let base = i * 11u;
    let torch = vec3<u32>(candidates[base + 1u], candidates[base + 2u], candidates[base + 3u]);
    let support = vec3<u32>(candidates[base + 4u], candidates[base + 5u], candidates[base + 6u]);
    let source = vec3<u32>(candidates[base + 8u], candidates[base + 9u], candidates[base + 10u]);

    let torch_cell = cell_at(torch);
    let support_cell = cell_at(support);
    let valid = torch_cell == 0u && (support_cell == 0u || support_cell == 1u);

    var penalty = 0u;
    penalty = penalty + neighbor_penalty(support, source, vec3<i32>(1, 0, 0));
    penalty = penalty + neighbor_penalty(support, source, vec3<i32>(-1, 0, 0));
    penalty = penalty + neighbor_penalty(support, source, vec3<i32>(0, 1, 0));
    penalty = penalty + neighbor_penalty(support, source, vec3<i32>(0, -1, 0));
    penalty = penalty + neighbor_penalty(support, source, vec3<i32>(0, 0, 1));
    penalty = penalty + neighbor_penalty(support, source, vec3<i32>(0, 0, -1));

    let delta = vec3<i32>(source) - vec3<i32>(support);
    let cost = u32(abs(delta.x) + abs(delta.y) + abs(delta.z));

    let out = i * 3u;
    scores[out] = select(0u, 1u, valid);
    scores[out + 1u] = select(10000u, penalty, valid);
    scores[out + 2u] = select(0u, cost, valid);
}
