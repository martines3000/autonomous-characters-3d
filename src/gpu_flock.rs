//! Flocking on the GPU: the whole simulation runs in compute passes over a hashed uniform
//! grid (`assets/shaders/gpu_flock.wgsl`) and writes the instance buffer the instanced draw
//! uses, so boid state never returns to the CPU.
//!
//! `GpuFlock` only needs a `RenderDevice`, a `RenderQueue` and a command encoder, so the same
//! code runs in the render world and in headless tests on a real device.

use bevy::{
    core_pipeline::schedule::camera_driver,
    prelude::*,
    render::{
        extract_resource::{ExtractResource, ExtractResourcePlugin},
        render_resource::{
            binding_types::{storage_buffer_sized, uniform_buffer},
            *,
        },
        renderer::{RenderContext, RenderDevice, RenderGraph, RenderGraphSystems, RenderQueue},
        RenderApp, RenderStartup,
    },
};

use crate::{
    instancing::FlockInstances,
    target::{Target, TargetSelection},
    vehicle, GlobalState, SimulationMode,
};

const SHADER: &str = include_str!("../assets/shaders/gpu_flock.wgsl");
const WORKGROUP: u32 = 256;
// Cells per prefix-sum block; matches SCAN_BLOCK in the shader.
const SCAN_BLOCK: u32 = 512;
// `Boid` in the shader: position/theta, velocity/phi, rotation, separation scratch.
pub(crate) type BoidData = [Vec4; 4];
const BOID_SIZE: u64 = size_of::<BoidData>() as u64;

/// Everything the shader needs for one step; filled each frame from `GlobalState`, the target
/// and the frame time. `count` is 0 when another simulation mode is active.
#[derive(Resource, ExtractResource, Clone, Copy, Default, Debug, ShaderType)]
#[extract_app(RenderApp)]
pub(crate) struct GpuParams {
    pub count: u32,
    // Set by `GpuFlock::encode`.
    pub spawn_from: u32,
    pub table_mask: u32,
    pub frame: u32,
    pub block_count: u32,
    // Alignment/cohesion samples, 0 = unlimited. Separation is never capped.
    pub max_neighbors: u32,
    pub follow: u32,
    pub dt: f32,
    pub cell_size: f32,
    pub size: f32,
    pub mass: f32,
    pub max_speed: f32,
    pub wander_speed: f32,
    pub separation_factor: f32,
    pub alignment_factor: f32,
    pub cohesion_factor: f32,
    pub wander_factor: f32,
    pub wall_avoid_factor: f32,
    pub seek_factor: f32,
    pub separation_distance: f32,
    pub alignment_distance: f32,
    pub cohesion_distance: f32,
    pub wander_distance: f32,
    pub wander_radius: f32,
    pub spawn_size: f32,
    pub walls: Vec3,
    pub target_position: Vec3,
}

pub struct GpuFlockPlugin;

impl Plugin for GpuFlockPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<GpuParams>()
            .add_plugins(ExtractResourcePlugin::<GpuParams>::default())
            .add_systems(
                Update,
                update_gpu_params.after(crate::target::input_controls),
            );
        let Some(render_app) = app.get_sub_app_mut(RenderApp) else {
            return;
        };
        render_app
            .add_systems(RenderStartup, init_gpu_flock)
            .add_systems(
                RenderGraph,
                run_gpu_flock
                    .in_set(RenderGraphSystems::Render)
                    .before(camera_driver),
            );
    }
}

fn update_gpu_params(
    state: Res<GlobalState>,
    targets: Query<(&Transform, &TargetSelection), With<Target>>,
    time: Res<Time>,
    mut params: ResMut<GpuParams>,
) {
    let (target, selection) = targets
        .single()
        .map_or((Vec3::ZERO, false), |(transform, selection)| {
            (transform.translation, selection.0)
        });
    *params = gpu_params(&state, target, selection, time.delta_secs());
}

pub(crate) fn gpu_params(state: &GlobalState, target: Vec3, follow: bool, dt: f32) -> GpuParams {
    let count = if state.mode == SimulationMode::Gpu {
        state.vehicle_count
    } else {
        0
    };
    GpuParams {
        count: count as u32,
        // Sparse wander uses exactly the unlimited path, avoiding an otherwise unconditional
        // fine-grid rebuild and separation pass. The cap targets dense follow-mode flocks.
        max_neighbors: if follow { state.gpu_max_neighbors } else { 0 },
        follow: follow as u32,
        dt,
        cell_size: state.vehicle_size
            * state
                .vehicle_separation_distance
                .max(state.vehicle_alignment_distance)
                .max(state.vehicle_cohesion_distance),
        size: state.vehicle_size,
        mass: state.vehicle_mass,
        max_speed: state.vehicle_max_speed,
        wander_speed: state.vehicle_wander_speed,
        separation_factor: state.vehicle_separation_factor,
        alignment_factor: state.vehicle_alignment_factor,
        cohesion_factor: state.vehicle_cohesion_factor,
        wander_factor: state.vehicle_wander_factor,
        wall_avoid_factor: state.vehicle_wall_avoid_factor,
        seek_factor: state.vehicle_seek_factor,
        separation_distance: state.vehicle_separation_distance,
        alignment_distance: state.vehicle_alignment_distance,
        cohesion_distance: state.vehicle_cohesion_distance,
        wander_distance: state.vehicle_wander_distance,
        wander_radius: state.vehicle_wander_radius,
        spawn_size: vehicle::spawn_size(state.vehicle_count),
        walls: vehicle::walls(state.vehicle_count),
        target_position: target,
        ..default()
    }
}

fn init_gpu_flock(mut commands: Commands, device: Res<RenderDevice>) {
    commands.insert_resource(GpuFlock::new(&device));
}

fn run_gpu_flock(
    mut context: RenderContext,
    mut flock: ResMut<GpuFlock>,
    params: Res<GpuParams>,
    queue: Res<RenderQueue>,
    mut draw: ResMut<FlockInstances>,
) {
    let device = context.render_device().clone();
    flock.encode(&device, &queue, context.command_encoder(), &params);
    if params.count > 0 {
        *draw = FlockInstances {
            buffer: flock
                .buffers
                .as_ref()
                .map(|buffers| buffers.instances.clone()),
            count: flock.count,
        };
    }
}

#[derive(Resource)]
pub(crate) struct GpuFlock {
    layout: BindGroupLayout,
    spawn: ComputePipeline,
    clear_counts: ComputePipeline,
    count_cells: ComputePipeline,
    count_separation: ComputePipeline,
    scan_blocks: ComputePipeline,
    scan_block_sums: ComputePipeline,
    scatter: ComputePipeline,
    separate: ComputePipeline,
    steer: ComputePipeline,
    params: Buffer,
    buffers: Option<FlockBuffers>,
    // Boids alive on the GPU.
    count: u32,
    frame: u32,
}

// The bind group keeps the grid buffers alive; only these two are used directly.
pub(crate) struct FlockBuffers {
    pub boids: Buffer,
    pub instances: Buffer,
    bind_group: BindGroup,
    capacity: u32,
    table_size: u32,
}

impl FlockBuffers {
    fn new(device: &RenderDevice, layout: &BindGroupLayout, params: &Buffer, count: u32) -> Self {
        let capacity = count.next_power_of_two().max(SCAN_BLOCK);
        // At least two hash slots per boid keeps collisions rare.
        let table_size = capacity * 2;
        let buffer = |label, size: u64, extra| {
            device.create_buffer(&BufferDescriptor {
                label: Some(label),
                size,
                usage: BufferUsages::STORAGE | extra,
                mapped_at_creation: false,
            })
        };
        let boids = buffer(
            "gpu_flock_boids",
            capacity as u64 * BOID_SIZE,
            BufferUsages::COPY_SRC | BufferUsages::COPY_DST,
        );
        let sorted = buffer(
            "gpu_flock_sorted",
            capacity as u64 * BOID_SIZE,
            BufferUsages::empty(),
        );
        let slots = buffer(
            "gpu_flock_slots",
            capacity as u64 * 8,
            BufferUsages::empty(),
        );
        let counts = buffer(
            "gpu_flock_counts",
            table_size as u64 * 4,
            BufferUsages::empty(),
        );
        let starts = buffer(
            "gpu_flock_starts",
            table_size as u64 * 4,
            BufferUsages::empty(),
        );
        let block_sums = buffer(
            "gpu_flock_block_sums",
            (table_size / SCAN_BLOCK) as u64 * 4,
            BufferUsages::empty(),
        );
        let instances = buffer(
            "gpu_flock_instances",
            capacity as u64 * size_of::<crate::instancing::InstanceData>() as u64,
            BufferUsages::VERTEX,
        );
        let bind_group = device.create_bind_group(
            "gpu_flock",
            layout,
            &BindGroupEntries::sequential((
                params.as_entire_binding(),
                boids.as_entire_binding(),
                sorted.as_entire_binding(),
                slots.as_entire_binding(),
                counts.as_entire_binding(),
                starts.as_entire_binding(),
                block_sums.as_entire_binding(),
                instances.as_entire_binding(),
            )),
        );
        Self {
            boids,
            instances,
            bind_group,
            capacity,
            table_size,
        }
    }
}

impl GpuFlock {
    pub fn new(device: &RenderDevice) -> Self {
        let entries = BindGroupLayoutEntries::sequential(
            ShaderStages::COMPUTE,
            (
                uniform_buffer::<GpuParams>(false),
                storage_buffer_sized(false, None),
                storage_buffer_sized(false, None),
                storage_buffer_sized(false, None),
                storage_buffer_sized(false, None),
                storage_buffer_sized(false, None),
                storage_buffer_sized(false, None),
                storage_buffer_sized(false, None),
            ),
        );
        let layout = device.create_bind_group_layout("gpu_flock", &entries);
        let module = device.create_and_validate_shader_module(ShaderModuleDescriptor {
            label: Some("gpu_flock"),
            source: ShaderSource::Wgsl(SHADER.into()),
        });
        let pipeline_layout = device.create_pipeline_layout(&PipelineLayoutDescriptor {
            label: Some("gpu_flock"),
            bind_group_layouts: &[Some(&layout)],
            immediate_size: 0,
        });
        let pipeline = |entry_point| {
            device.create_compute_pipeline(&RawComputePipelineDescriptor {
                label: Some(entry_point),
                layout: Some(&pipeline_layout),
                module: &module,
                entry_point: Some(entry_point),
                compilation_options: default(),
                cache: None,
            })
        };
        let params = device.create_buffer(&BufferDescriptor {
            label: Some("gpu_flock_params"),
            size: GpuParams::min_size().get(),
            usage: BufferUsages::UNIFORM | BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        Self {
            spawn: pipeline("spawn"),
            clear_counts: pipeline("clear_counts"),
            count_cells: pipeline("count"),
            count_separation: pipeline("count_separation"),
            scan_blocks: pipeline("scan_blocks"),
            scan_block_sums: pipeline("scan_block_sums"),
            scatter: pipeline("scatter"),
            separate: pipeline("separate"),
            steer: pipeline("steer"),
            layout,
            params,
            buffers: None,
            count: 0,
            frame: 0,
        }
    }

    /// Records one simulation step for `params.count` boids: spawns boids beyond the current
    /// count, then builds the grid and steers. A count of 0 forgets the flock, so the next
    /// non-zero count spawns a fresh one.
    pub fn encode(
        &mut self,
        device: &RenderDevice,
        queue: &RenderQueue,
        encoder: &mut CommandEncoder,
        params: &GpuParams,
    ) {
        let count = params.count;
        if count == 0 {
            self.count = 0;
            return;
        }
        if self
            .buffers
            .as_ref()
            .is_none_or(|buffers| buffers.capacity < count)
        {
            let grown = FlockBuffers::new(device, &self.layout, &self.params, count);
            if let Some(old) = &self.buffers {
                if self.count > 0 {
                    encoder.copy_buffer_to_buffer(
                        &old.boids,
                        0,
                        &grown.boids,
                        0,
                        self.count as u64 * BOID_SIZE,
                    );
                }
            }
            self.buffers = Some(grown);
        }
        let buffers = self.buffers.as_ref().unwrap();
        let spawn_from = self.count.min(count);
        let block_count = buffers.table_size / SCAN_BLOCK;
        let step = GpuParams {
            spawn_from,
            table_mask: buffers.table_size - 1,
            frame: self.frame,
            block_count,
            ..*params
        };
        let mut bytes = encase::UniformBuffer::new(Vec::<u8>::new());
        bytes.write(&step).unwrap();
        queue.write_buffer(&self.params, 0, bytes.as_ref());

        let groups = |items: u32| items.div_ceil(WORKGROUP);
        let mut pass = encoder.begin_compute_pass(&ComputePassDescriptor {
            label: Some("gpu_flock"),
            timestamp_writes: None,
        });
        pass.set_bind_group(0, &*buffers.bind_group, &[]);
        if spawn_from < count {
            pass.set_pipeline(&self.spawn);
            pass.dispatch_workgroups(groups(count - spawn_from), 1, 1);
        }
        if params.max_neighbors != 0 {
            // Exact separation in a fine grid; reuse the grid buffers for the coarse
            // alignment/cohesion pass and carry separation through the second sort.
            for (pipeline, workgroups) in [
                (&self.clear_counts, groups(buffers.table_size)),
                (&self.count_separation, groups(count)),
                (&self.scan_blocks, block_count),
                (&self.scan_block_sums, 1),
                (&self.scatter, groups(count)),
                (&self.separate, groups(count)),
            ] {
                pass.set_pipeline(pipeline);
                pass.dispatch_workgroups(workgroups, 1, 1);
            }
        }
        for (pipeline, workgroups) in [
            (&self.clear_counts, groups(buffers.table_size)),
            (&self.count_cells, groups(count)),
            (&self.scan_blocks, block_count),
            (&self.scan_block_sums, 1),
            (&self.scatter, groups(count)),
            (&self.steer, groups(count)),
        ] {
            pass.set_pipeline(pipeline);
            pass.dispatch_workgroups(workgroups, 1, 1);
        }
        drop(pass);
        self.count = count;
        self.frame = self.frame.wrapping_add(1);
    }

    /// Replaces the flock with `boids`, as if they had been spawned.
    #[cfg(test)]
    pub fn upload(&mut self, device: &RenderDevice, queue: &RenderQueue, boids: &[BoidData]) {
        let count = boids.len() as u32;
        if count == 0 {
            self.count = 0;
            return;
        }
        if self
            .buffers
            .as_ref()
            .is_none_or(|buffers| buffers.capacity < count)
        {
            self.buffers = Some(FlockBuffers::new(device, &self.layout, &self.params, count));
        }
        let bytes: Vec<u8> = boids
            .iter()
            .flat_map(|boid| boid.iter().flat_map(|v| v.to_array()))
            .flat_map(f32::to_le_bytes)
            .collect();
        queue.write_buffer(&self.buffers.as_ref().unwrap().boids, 0, &bytes);
        self.count = count;
    }

    /// Copies the first `count` boids back to the CPU (blocking).
    #[cfg(test)]
    pub fn read_boids(&self, device: &RenderDevice, queue: &RenderQueue) -> Vec<BoidData> {
        if self.count == 0 {
            return Vec::new();
        }
        let size = self.count as u64 * BOID_SIZE;
        let staging = device.create_buffer(&BufferDescriptor {
            label: Some("gpu_flock_readback"),
            size,
            usage: BufferUsages::MAP_READ | BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let mut encoder = device.create_command_encoder(&CommandEncoderDescriptor::default());
        encoder.copy_buffer_to_buffer(&self.buffers.as_ref().unwrap().boids, 0, &staging, 0, size);
        queue.submit([encoder.finish()]);
        staging
            .slice(..)
            .map_async(MapMode::Read, |result| result.unwrap());
        device.poll(PollType::wait_indefinitely()).unwrap();
        let floats: Vec<f32> = staging
            .slice(..)
            .get_mapped_range()
            .expect("GPU flock readback is mapped after polling")
            .as_chunks::<4>()
            .0
            .iter()
            .map(|bytes| f32::from_le_bytes(*bytes))
            .collect();
        floats
            .as_chunks::<16>()
            .0
            .iter()
            .map(|f| {
                [
                    Vec4::new(f[0], f[1], f[2], f[3]),
                    Vec4::new(f[4], f[5], f[6], f[7]),
                    Vec4::new(f[8], f[9], f[10], f[11]),
                    Vec4::new(f[12], f[13], f[14], f[15]),
                ]
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn params_use_live_settings_and_only_enable_gpu_mode() {
        let mut state = GlobalState {
            vehicle_count: 12_345,
            vehicle_size: 2.5,
            vehicle_mass: 123.0,
            vehicle_separation_distance: 40.0,
            vehicle_alignment_distance: 5.0,
            vehicle_cohesion_distance: 9.0,
            gpu_max_neighbors: 128,
            ..default()
        };
        let target = Vec3::new(-10.0, 20.0, 30.0);
        for mode in [
            SimulationMode::Gpu,
            SimulationMode::Naive,
            SimulationMode::Octree,
        ] {
            state.mode = mode;
            let params = gpu_params(&state, target, true, 0.02);
            assert_eq!(
                params.count,
                if mode == SimulationMode::Gpu {
                    12_345
                } else {
                    0
                }
            );
            assert_eq!(params.cell_size, 100.0);
            assert_eq!(params.mass, 123.0);
            assert_eq!(params.size, 2.5);
            assert_eq!(params.dt, 0.02);
            assert_eq!(params.follow, 1);
            assert_eq!(params.target_position, target);
            assert_eq!(params.spawn_size, vehicle::spawn_size(12_345));
            assert_eq!(params.walls, vehicle::walls(12_345));
            assert_eq!(params.max_neighbors, 128);
        }
    }

    #[test]
    fn params_encode_with_wgsl_uniform_offsets() {
        // Naga's Params layout: seven u32s, eighteen f32s, then vec3s at 112 and 128.
        let params = GpuParams {
            count: 1,
            spawn_from: 2,
            table_mask: 3,
            frame: 4,
            block_count: 5,
            max_neighbors: 6,
            follow: 7,
            dt: 8.0,
            cell_size: 9.0,
            size: 10.0,
            mass: 11.0,
            max_speed: 12.0,
            wander_speed: 13.0,
            separation_factor: 14.0,
            alignment_factor: 15.0,
            cohesion_factor: 16.0,
            wander_factor: 17.0,
            wall_avoid_factor: 18.0,
            seek_factor: 19.0,
            separation_distance: 20.0,
            alignment_distance: 21.0,
            cohesion_distance: 22.0,
            wander_distance: 23.0,
            wander_radius: 24.0,
            spawn_size: 25.0,
            walls: Vec3::new(26.0, 27.0, 28.0),
            target_position: Vec3::new(29.0, 30.0, 31.0),
        };
        let mut buffer = encase::UniformBuffer::new(Vec::<u8>::new());
        buffer.write(&params).unwrap();
        let bytes = buffer.as_ref();
        assert_eq!(GpuParams::min_size().get(), 144);
        assert_eq!(bytes.len(), 144);
        for i in 0..7 {
            assert_eq!(&bytes[i * 4..i * 4 + 4], &(i as u32 + 1).to_le_bytes());
        }
        for (offset, value) in (28..100)
            .step_by(4)
            .zip(8..26)
            .chain((112..124).step_by(4).zip(26..29))
            .chain((128..140).step_by(4).zip(29..32))
        {
            assert_eq!(&bytes[offset..offset + 4], &(value as f32).to_le_bytes());
        }
    }

    #[test]
    fn wander_uses_identical_gpu_inputs_for_every_follow_cap() {
        let mut state = GlobalState {
            mode: SimulationMode::Gpu,
            vehicle_count: 1_000_000,
            gpu_max_neighbors: 0,
            ..default()
        };
        let encode = |params: GpuParams| {
            let mut buffer = encase::UniformBuffer::new(Vec::<u8>::new());
            buffer.write(&params).unwrap();
            buffer.into_inner()
        };
        let target = Vec3::new(20.0, 30.0, -10.0);
        let reference = encode(gpu_params(&state, target, false, 1.0 / 60.0));
        for cap in [0, 16, 32, 64, 512] {
            state.gpu_max_neighbors = cap;
            let params = gpu_params(&state, target, false, 1.0 / 60.0);
            assert_eq!(params.max_neighbors, 0);
            // Equal uniforms select the same single-grid dispatches and shader branches.
            assert_eq!(encode(params), reference);
            assert_eq!(
                gpu_params(&state, target, true, 1.0 / 60.0).max_neighbors,
                cap
            );
        }
    }
}
