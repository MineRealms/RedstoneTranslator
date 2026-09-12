//! Optional wgpu backend for the candidate evaluator (feature `gpu`).
//!
//! The GPU evaluates a batch of `PlacementCandidate` records against a packed
//! copy of the world and returns the same `CandidateScore` values as the CPU
//! reference. It is selected with `MCHDL_GPU=1`; any GPU error falls back to
//! the CPU evaluator so compilation never depends on the device.
//!
//! The kernel mirrors `candidate_eval::evaluate_one` exactly; the differential
//! test below guards against drift.

use std::sync::OnceLock;

use wgpu::util::DeviceExt;

use crate::transform::place_and_route::candidate_eval::{
    CandidateEvaluator, CandidateScore, CpuCandidateEvaluator, PlacementCandidate,
};
use crate::world::block::{BlockKind, Direction};
use crate::world::World3D;

/// True when the user opted into the GPU backend.
pub fn enabled() -> bool {
    std::env::var_os("MCHDL_GPU").is_some()
}

struct GpuState {
    device: wgpu::Device,
    queue: wgpu::Queue,
    pipeline: wgpu::ComputePipeline,
    layout: wgpu::BindGroupLayout,
}

static STATE: OnceLock<Option<GpuState>> = OnceLock::new();

fn state() -> Option<&'static GpuState> {
    STATE.get_or_init(|| init().ok()).as_ref()
}

fn init() -> eyre::Result<GpuState> {
    let instance = wgpu::Instance::new(&wgpu::InstanceDescriptor {
        backends: wgpu::Backends::all(),
        ..Default::default()
    });
    let adapter = pollster::block_on(instance.request_adapter(&wgpu::RequestAdapterOptions {
        power_preference: wgpu::PowerPreference::HighPerformance,
        compatible_surface: None,
        force_fallback_adapter: false,
    }))
    .ok_or_else(|| eyre::eyre!("no GPU adapter available"))?;
    let (device, queue) =
        pollster::block_on(adapter.request_device(&wgpu::DeviceDescriptor::default(), None))?;

    let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some("candidate_filter"),
        source: wgpu::ShaderSource::Wgsl(include_str!("candidate_filter.wgsl").into()),
    });

    let storage_entry = |binding: u32, read_only: bool| wgpu::BindGroupLayoutEntry {
        binding,
        visibility: wgpu::ShaderStages::COMPUTE,
        ty: wgpu::BindingType::Buffer {
            ty: wgpu::BufferBindingType::Storage { read_only },
            has_dynamic_offset: false,
            min_binding_size: None,
        },
        count: None,
    };
    let layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("candidate_filter_layout"),
        entries: &[
            wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
            storage_entry(1, true),
            storage_entry(2, true),
            storage_entry(3, false),
        ],
    });
    let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
        label: None,
        bind_group_layouts: &[&layout],
        push_constant_ranges: &[],
    });
    let pipeline = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
        label: Some("candidate_filter"),
        layout: Some(&pipeline_layout),
        module: &shader,
        entry_point: Some("main"),
        compilation_options: Default::default(),
        cache: None,
    });

    Ok(GpuState {
        device,
        queue,
        pipeline,
        layout,
    })
}

/// Evaluate with the GPU when available; fall back to the CPU reference on any
/// initialization or dispatch error.
pub fn evaluate(world: &World3D, candidates: &[PlacementCandidate]) -> Vec<CandidateScore> {
    if candidates.is_empty() {
        return Vec::new();
    }
    let Some(gpu) = state() else {
        return CpuCandidateEvaluator.evaluate(world, candidates);
    };
    evaluate_inner(gpu, world, candidates)
        .unwrap_or_else(|_| CpuCandidateEvaluator.evaluate(world, candidates))
}

fn pack_kind(kind: BlockKind) -> u32 {
    match kind {
        BlockKind::Air => 0,
        BlockKind::Cobble { .. } => 1,
        BlockKind::Redstone { .. } => 2,
        BlockKind::Torch { .. } => 3,
        BlockKind::Repeater { .. } => 4,
        BlockKind::Switch { .. } => 5,
        BlockKind::RedstoneBlock => 6,
        _ => 7,
    }
}

fn direction_code(direction: Direction) -> u32 {
    match direction {
        Direction::None => 0,
        Direction::Bottom => 1,
        Direction::Top => 2,
        Direction::East => 3,
        Direction::West => 4,
        Direction::South => 5,
        Direction::North => 6,
    }
}

fn evaluate_inner(
    gpu: &GpuState,
    world: &World3D,
    candidates: &[PlacementCandidate],
) -> eyre::Result<Vec<CandidateScore>> {
    let (sx, sy, sz) = (world.size.0, world.size.1, world.size.2);
    let mut cells = vec![0u32; sx * sy * sz];
    for (position, block) in world.iter_block() {
        let index = position.0 + position.1 * sx + position.2 * sx * sy;
        cells[index] = pack_kind(block.kind);
    }

    let params = [sx as u32, sy as u32, sz as u32, candidates.len() as u32];
    let mut candidate_data = Vec::with_capacity(candidates.len() * 11);
    for candidate in candidates {
        candidate_data.extend_from_slice(&[
            candidate.entry,
            candidate.torch.0 as u32,
            candidate.torch.1 as u32,
            candidate.torch.2 as u32,
            candidate.support.0 as u32,
            candidate.support.1 as u32,
            candidate.support.2 as u32,
            direction_code(candidate.direction),
            candidate.source.0 as u32,
            candidate.source.1 as u32,
            candidate.source.2 as u32,
        ]);
    }

    let params_buffer = gpu
        .device
        .create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("params"),
            contents: bytemuck_u32(&params),
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        });
    let cells_buffer = gpu
        .device
        .create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("cells"),
            contents: bytemuck_u32(&cells),
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
        });
    let candidates_buffer = gpu
        .device
        .create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("candidates"),
            contents: bytemuck_u32(&candidate_data),
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
        });

    let scores_size = (candidates.len() * 3 * std::mem::size_of::<u32>()) as u64;
    let scores_buffer = gpu.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("scores"),
        size: scores_size,
        usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
        mapped_at_creation: false,
    });
    let readback = gpu.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("scores_readback"),
        size: scores_size,
        usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });

    let bind_group = gpu.device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: None,
        layout: &gpu.layout,
        entries: &[
            wgpu::BindGroupEntry {
                binding: 0,
                resource: params_buffer.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 1,
                resource: cells_buffer.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 2,
                resource: candidates_buffer.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 3,
                resource: scores_buffer.as_entire_binding(),
            },
        ],
    });

    let mut encoder = gpu
        .device
        .create_command_encoder(&wgpu::CommandEncoderDescriptor { label: None });
    {
        let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor::default());
        pass.set_pipeline(&gpu.pipeline);
        pass.set_bind_group(0, &bind_group, &[]);
        let groups = (candidates.len() as u32).div_ceil(64);
        pass.dispatch_workgroups(groups, 1, 1);
    }
    encoder.copy_buffer_to_buffer(&scores_buffer, 0, &readback, 0, scores_size);
    gpu.queue.submit(Some(encoder.finish()));

    let slice = readback.slice(..);
    let (sender, receiver) = std::sync::mpsc::channel();
    slice.map_async(wgpu::MapMode::Read, move |result| {
        let _ = sender.send(result);
    });
    let _ = gpu.device.poll(wgpu::Maintain::Wait);
    receiver
        .recv()
        .map_err(|_| eyre::eyre!("gpu map channel closed"))?
        .map_err(|error| eyre::eyre!("gpu map failed: {error}"))?;

    let bytes = slice.get_mapped_range().to_vec();
    readback.unmap();

    let words: Vec<u32> = bytes
        .chunks_exact(4)
        .map(|chunk| u32::from_le_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]))
        .collect();
    Ok(words
        .chunks_exact(3)
        .map(|word| CandidateScore {
            valid: word[0] != 0,
            drc_penalty: word[1],
            estimated_route_cost: word[2],
        })
        .collect())
}

fn bytemuck_u32(values: &[u32]) -> &[u8] {
    // Safe on all supported platforms: u32 has no padding and the slice lives
    // as long as the borrow.
    unsafe { std::slice::from_raw_parts(values.as_ptr() as *const u8, values.len() * 4) }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::transform::place_and_route::candidate_eval::{
        evaluate as cpu_evaluate, CandidateKind,
    };
    use crate::world::block::{Block, BlockKind, Direction};
    use crate::world::position::{DimSize, Position};
    use crate::world::World;

    fn cobble() -> Block {
        Block {
            kind: BlockKind::Cobble {
                on_count: 0,
                on_base_count: 0,
            },
            direction: Direction::None,
        }
    }

    fn torch(direction: Direction) -> Block {
        Block {
            kind: BlockKind::Torch { is_on: false },
            direction,
        }
    }

    fn candidate(torch: Position, direction: Direction, support: Position) -> PlacementCandidate {
        PlacementCandidate {
            entry: 0,
            kind: CandidateKind::Torch,
            torch,
            direction,
            support,
            source: Position(0, 0, 0),
        }
    }

    #[test]
    fn wgsl_is_valid() {
        let module = naga::front::wgsl::parse_str(include_str!("candidate_filter.wgsl"))
            .unwrap_or_else(|error| panic!("WGSL parse error: {error}"));
        let mut validator = naga::valid::Validator::new(
            naga::valid::ValidationFlags::all(),
            naga::valid::Capabilities::all(),
        );
        validator
            .validate(&module)
            .unwrap_or_else(|error| panic!("WGSL validation error: {error:?}"));
    }

    #[test]
    fn gpu_matches_cpu_on_sample_batch() {
        let Some(gpu) = state() else {
            eprintln!("no GPU adapter available; skipping differential test");
            return;
        };

        let world = World3D::from(&World {
            size: DimSize(8, 8, 4),
            blocks: vec![
                (Position(2, 1, 1), cobble()),
                (Position(3, 1, 1), torch(Direction::West)),
                (Position(1, 1, 1), cobble()),
            ],
        });
        let candidates = vec![
            candidate(Position(1, 1, 2), Direction::East, Position(2, 1, 2)),
            candidate(Position(1, 1, 1), Direction::East, Position(2, 1, 1)),
            candidate(Position(5, 5, 2), Direction::North, Position(5, 6, 2)),
            candidate(Position(1, 1, 2), Direction::East, Position(9, 1, 2)),
        ];

        let cpu = cpu_evaluate(&world, &candidates);
        let gpu_scores = evaluate_inner(gpu, &world, &candidates).expect("gpu evaluate");
        assert_eq!(cpu, gpu_scores);
    }
}
