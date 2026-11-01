//! Flocking on the GPU: the whole simulation runs in compute passes over a hashed uniform
//! grid (`assets/shaders/gpu_flock.wgsl`) and writes the instance buffer the instanced draw
//! uses, so boid state never returns to the CPU.
//!
//! `GpuFlock` only needs a `RenderDevice`, a `RenderQueue` and a command encoder, so the same
//! code runs in the render world and in headless tests on a real device.

use bevy::{
    core_pipeline::schedule::camera_driver,
    ecs::system::SystemParam,
    prelude::*,
    render::{
        diagnostic::RecordDiagnostics,
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
    formation::FormationFrame,
    instancing::FlockInstances,
    predator::{PredatorFrame, PREDATOR_RECORDS},
    swarm::{Obstacle, SwarmScene, MAX_OBSTACLES, OBSTACLE_LOOK_AHEAD},
    target::{Target, TargetSelection},
    vehicle, GlobalState, SimulationMode,
};

const SHADER: &str = include_str!("../assets/shaders/gpu_flock.wgsl");
const WORKGROUP: u32 = 256;
// Cells per prefix-sum block; matches SCAN_BLOCK in the shader.
const SCAN_BLOCK: u32 = 512;
// Test/readback snapshot combines the 64-byte shader Boid and separate 16-byte metadata.
#[cfg(test)]
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub(crate) struct BoidData {
    pub state: [Vec4; 4],
    pub metadata: UVec4,
}

#[cfg(test)]
impl From<[Vec4; 4]> for BoidData {
    fn from(state: [Vec4; 4]) -> Self {
        Self {
            state,
            metadata: UVec4::ZERO,
        }
    }
}
#[cfg(test)]
impl std::ops::Deref for BoidData {
    type Target = [Vec4; 4];
    fn deref(&self) -> &Self::Target {
        &self.state
    }
}
#[cfg(test)]
impl std::ops::DerefMut for BoidData {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.state
    }
}
#[cfg(test)]
impl<'a> IntoIterator for &'a BoidData {
    type Item = &'a Vec4;
    type IntoIter = std::slice::Iter<'a, Vec4>;
    fn into_iter(self) -> Self::IntoIter {
        self.state.iter()
    }
}
const BOID_SIZE: u64 = size_of::<[Vec4; 4]>() as u64;
const METADATA_SIZE: u64 = size_of::<UVec4>() as u64;

/// Everything the shader needs for one step; filled each frame from `GlobalState`, the target
/// and the frame time. `count` is 0 when another simulation mode is active.
#[derive(Resource, ExtractResource, Clone, Copy, Default, Debug, PartialEq, ShaderType)]
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
    pub attractor_position_radius: Vec4,
    // x: signed strength (zero disables); yzw: reserved.
    pub attractor_strength: Vec4,
    // x: active group count (0 = off), y: next spawn ID, z: input order-map base, w: capacity.
    pub groups: UVec4,
    pub group_targets: [Vec4; crate::swarm::MAX_GROUPS],
    // x: arena vec4 offset (set by encode), y: count; zw: reserved.
    pub obstacles: UVec4,
    // strength, look-ahead simulated seconds, world scale, reserved.
    pub obstacle_settings: Vec4,
    // mode, reserved, maintenance block count, arena group-range offset.
    pub formation: UVec4,
    pub formation_center_extent: Vec4,
    // World arrival radius, strength, actual morph, reserved.
    pub formation_settings: Vec4,
    // Arena offset, active count, substep, reserved.
    pub predators: UVec4,
    pub predator_settings: Vec4,
    // mode (0 off, 1 spawn ID, 2 index), selector, simulated-time bits, arena tail offset.
    pub track: UVec4,
    // bank strength, exact legacy flag, response rate, reserved. Render orientation only.
    pub motion: Vec4,
}

impl GpuParams {
    pub(crate) fn with_scene(mut self, scene: &SwarmScene) -> Self {
        self.obstacles = UVec4::ZERO;
        self.obstacle_settings = Vec4::ZERO;
        if !scene.obstacles.is_empty() && scene.obstacle_strength != 0.0 {
            self.obstacles.y = scene.obstacles.len() as u32;
            self.obstacle_settings = Vec4::new(
                scene.obstacle_strength,
                OBSTACLE_LOOK_AHEAD,
                vehicle::world_scale(self.count as usize),
                0.0,
            );
        }
        self
    }

    pub(crate) fn with_formation(mut self, frame: &FormationFrame, cap: u32) -> Self {
        self.formation = UVec4::ZERO;
        self.formation_center_extent = Vec4::ZERO;
        self.formation_settings = Vec4::ZERO;
        if frame.settings.enabled() {
            let scale = vehicle::world_scale(self.count as usize);
            self.formation.x = frame.settings.mode as u32;
            self.formation_center_extent =
                frame.settings.center.extend(frame.settings.extent * scale);
            self.formation_settings = Vec4::new(
                frame.settings.arrival_radius * scale,
                frame.settings.strength,
                frame.morph,
                0.0,
            );
            self.max_neighbors = cap;
        }
        self
    }

    pub(crate) fn with_predators(mut self, frame: &PredatorFrame) -> Self {
        self.predators = UVec4::ZERO;
        self.predator_settings = Vec4::ZERO;
        if frame.active() {
            self.predators.y = frame.count as u32;
            self.predator_settings.x = frame.strength;
        }
        self
    }

    fn simulates(&self) -> bool {
        self.dt > 0.0
    }
}

pub struct GpuFlockPlugin;

impl Plugin for GpuFlockPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<PredatorFrame>()
            .init_resource::<FormationFrame>()
            .init_resource::<SwarmScene>()
            .init_resource::<GpuParams>()
            .add_plugins((
                ExtractResourcePlugin::<GpuParams>::default(),
                ExtractResourcePlugin::<SwarmScene>::default(),
                ExtractResourcePlugin::<FormationFrame>::default(),
                ExtractResourcePlugin::<PredatorFrame>::default(),
            ))
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

pub(crate) fn update_gpu_params(
    state: Res<GlobalState>,
    targets: Query<(&Transform, &TargetSelection), With<Target>>,
    frame: Res<crate::presentation::SimulationFrame>,
    mut params: ResMut<GpuParams>,
    formation: Res<FormationFrame>,
    predators: Res<PredatorFrame>,
) {
    let (target, selection) = targets
        .single()
        .map_or((Vec3::ZERO, false), |(transform, selection)| {
            (transform.translation, selection.0)
        });
    params.set_if_neq(
        gpu_params(&state, target, selection, frame.step.as_secs_f32())
            .with_formation(&formation, state.gpu_max_neighbors)
            .with_predators(&predators),
    );
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
        max_neighbors: if follow || state.groups.has_targets() {
            state.gpu_max_neighbors
        } else {
            0
        },
        groups: UVec4::new(state.groups.count, 0, 0, 0),
        group_targets: state
            .groups
            .table
            .map(|group| group.target.map_or(Vec4::ZERO, |point| point.extend(1.0))),
        follow: follow as u32,
        dt,
        cell_size: state.vehicle_size
            * state
                .vehicle_separation_distance
                .max(state.vehicle_alignment_distance)
                .max(state.vehicle_cohesion_distance),
        size: state.vehicle_size,
        motion: Vec4::new(
            state.motion.bank,
            u32::from(state.motion.classic_heading) as f32,
            crate::motion::RESPONSE,
            0.0,
        ),
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
        attractor_position_radius: state.attractor.point().map_or(Vec4::ZERO, |point| {
            point.extend(state.attractor.world_radius(state.vehicle_count))
        }),
        attractor_strength: Vec4::new(
            if state.attractor.point().is_some() {
                state.attractor.signed_strength()
            } else {
                0.0
            },
            0.0,
            0.0,
            0.0,
        ),
        ..default()
    }
}

fn init_gpu_flock(mut commands: Commands, device: Res<RenderDevice>) {
    commands.insert_resource(GpuFlock::new(&device));
}

#[derive(SystemParam)]
struct FlockFrame<'w> {
    simulation: Res<'w, crate::presentation::SimulationFrame>,
    formation: Res<'w, FormationFrame>,
    predators: Res<'w, PredatorFrame>,
    tracking: Option<Res<'w, crate::follow::TrackBuffer>>,
    readback_buffers:
        Res<'w, bevy::render::render_asset::RenderAssets<bevy::render::storage::GpuShaderBuffer>>,
}

fn run_gpu_flock(
    mut context: RenderContext,
    mut flock: ResMut<GpuFlock>,
    params: Res<GpuParams>,
    frames: FlockFrame,
    queue: Res<RenderQueue>,
    mut draw: ResMut<FlockInstances>,
    scene: Res<SwarmScene>,
) {
    // egui edits in PostUpdate: derive count/settings from the same extracted scene as records.
    let mut params = (*params).with_scene(&scene);
    let frame = &frames.simulation;
    flock.set_scene(&scene);
    flock.set_predators(&frames.predators);
    let device = context.render_device().clone();
    let diagnostics = context.diagnostic_recorder();
    let recorder = diagnostics.as_deref();
    let name = match frame.steps {
        0 => "flock_0_steps",
        1 => "flock_1_steps",
        2 => "flock_2_steps",
        3 => "flock_3_steps",
        _ => "flock_4_steps",
    };
    let span = recorder.time_span(context.command_encoder(), name);
    // A paused frame still permits spawn/truncate maintenance, never sorting or steering.
    if frame.steps > 0
        || params.track.x != 0
        || params.count != flock.count
        || params.groups.x != flock.group_count
        || flock.scene_dirty
        || flock.predators_dirty
        || (params.formation.x != 0) != flock.formation_enabled
    {
        for index in 0..frame.steps.max(1) {
            if params.formation.x != 0 {
                params.formation_settings.z = frames.formation.samples[index as usize];
            }
            if params.predators.y != 0 {
                params.predators.z = index;
            }
            flock.encode(&device, &queue, context.command_encoder(), &params);
        }
    }
    if params.track.x != 0 {
        if let Some(buffer) = frames
            .tracking
            .as_ref()
            .and_then(|track| frames.readback_buffers.get(&track.handle))
        {
            flock.copy_track(context.command_encoder(), &buffer.buffer);
        }
    }
    span.end(context.command_encoder());
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
    update_groups: ComputePipeline,
    gather_metadata: ComputePipeline,
    compact_metadata: ComputePipeline,
    clear_counts: ComputePipeline,
    count_cells: ComputePipeline,
    count_separation: ComputePipeline,
    scan_blocks: ComputePipeline,
    scan_block_sums: ComputePipeline,
    scatter: ComputePipeline,
    scatter_fine: ComputePipeline,
    separate: ComputePipeline,
    formation_count: ComputePipeline,
    formation_scan: ComputePipeline,
    formation_assign: ComputePipeline,
    // Index bits: 0 groups, 1 obstacles, 2 formation, 3 predators. Disabled features compile out of steering.
    steer: [ComputePipeline; 16],
    track: ComputePipeline,
    params: Buffer,
    buffers: Option<FlockBuffers>,
    // Boids alive on the GPU.
    count: u32,
    frame: u32,
    next_id: u32,
    group_count: u32,
    // Vec4 offset (0 or capacity/4) of the order map matching the current boids.
    order_base: u32,
    obstacles: Vec<Obstacle>,
    scene_dirty: bool,
    formation_enabled: bool,
    predator_records: Vec<UVec4>,
    predators_dirty: bool,
    #[cfg(test)]
    predator_uploads: u32,
    #[cfg(test)]
    scene_uploads: u32,
}

// The bind group keeps the grid buffers alive; state, instances and metadata are also used directly.
pub(crate) struct FlockBuffers {
    pub boids: Buffer,
    pub instances: Buffer,
    arena: Buffer,
    bind_group: BindGroup,
    capacity: u32,
    table_size: u32,
}

impl FlockBuffers {
    fn track_offset(&self) -> u32 {
        self.capacity * 5 / 2 + 24 + PREDATOR_RECORDS as u32
    }

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
            BufferUsages::VERTEX | BufferUsages::COPY_SRC | BufferUsages::COPY_DST,
        );
        let arena = buffer(
            "gpu_flock_metadata_arena",
            // Two metadata tables and two packed u32 order maps; scene data shares this binding.
            capacity as u64 * METADATA_SIZE * 5 / 2
                + (MAX_OBSTACLES as u64 * 2
                    + crate::swarm::MAX_GROUPS as u64
                    + PREDATOR_RECORDS as u64
                    + 5)
                    * METADATA_SIZE,
            BufferUsages::COPY_SRC | BufferUsages::COPY_DST,
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
                arena.as_entire_binding(),
            )),
        );
        Self {
            boids,
            instances,
            arena,
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
        let pipeline = |entry_point, groups, obstacles, formation, predators| {
            device.create_compute_pipeline(&RawComputePipelineDescriptor {
                label: Some(entry_point),
                layout: Some(&pipeline_layout),
                module: &module,
                entry_point: Some(entry_point),
                compilation_options: PipelineCompilationOptions {
                    constants: &[
                        ("GROUPS_ENABLED", groups),
                        ("OBSTACLES_ENABLED", obstacles),
                        ("FORMATION_ENABLED", formation),
                        ("PREDATORS_ENABLED", predators),
                    ],
                    ..default()
                },
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
            spawn: pipeline("spawn", 0.0, 0.0, 0.0, 0.0),
            update_groups: pipeline("update_groups", 0.0, 0.0, 0.0, 0.0),
            gather_metadata: pipeline("gather_metadata", 0.0, 0.0, 0.0, 0.0),
            compact_metadata: pipeline("compact_metadata", 0.0, 0.0, 0.0, 0.0),
            clear_counts: pipeline("clear_counts", 0.0, 0.0, 0.0, 0.0),
            count_cells: pipeline("count", 0.0, 0.0, 0.0, 0.0),
            count_separation: pipeline("count_separation", 0.0, 0.0, 0.0, 0.0),
            scan_blocks: pipeline("scan_blocks", 0.0, 0.0, 0.0, 0.0),
            scan_block_sums: pipeline("scan_block_sums", 0.0, 0.0, 0.0, 0.0),
            scatter: pipeline("scatter", 0.0, 0.0, 0.0, 0.0),
            scatter_fine: pipeline("scatter_fine", 0.0, 0.0, 0.0, 0.0),
            separate: pipeline("separate", 0.0, 0.0, 0.0, 0.0),
            formation_count: pipeline("formation_count", 0.0, 0.0, 0.0, 0.0),
            formation_scan: pipeline("formation_scan", 0.0, 0.0, 0.0, 0.0),
            formation_assign: pipeline("formation_assign", 0.0, 0.0, 0.0, 0.0),
            track: pipeline("track", 0.0, 0.0, 0.0, 0.0),
            steer: std::array::from_fn(|flags| {
                pipeline(
                    "steer",
                    (flags & 1 != 0) as u32 as f64,
                    (flags & 2 != 0) as u32 as f64,
                    (flags & 4 != 0) as u32 as f64,
                    (flags & 8 != 0) as u32 as f64,
                )
            }),
            layout,
            params,
            buffers: None,
            count: 0,
            frame: 0,
            next_id: 0,
            group_count: 0,
            order_base: 0,
            obstacles: Vec::new(),
            scene_dirty: false,
            formation_enabled: false,
            predator_records: Vec::new(),
            predators_dirty: false,
            #[cfg(test)]
            predator_uploads: 0,
            #[cfg(test)]
            scene_uploads: 0,
        }
    }

    /// Records one simulation step for `params.count` boids: spawns boids beyond the current
    /// count, then builds the grid and steers. A count of 0 forgets the flock, so the next
    /// non-zero count spawns a fresh one. A zero delta performs count maintenance only:
    /// spawn writes new instances without sorting, steering or advancing the random frame.
    pub(crate) fn set_scene(&mut self, scene: &SwarmScene) {
        assert!(scene.obstacles.len() <= MAX_OBSTACLES);
        if self.obstacles != scene.obstacles {
            self.obstacles.clone_from(&scene.obstacles);
            self.scene_dirty = true;
        }
    }

    pub(crate) fn set_predators(&mut self, frame: &PredatorFrame) {
        let records = frame.records();
        if self.predator_records != records {
            self.predator_records = records;
            self.predators_dirty = true;
        }
    }

    pub fn encode(
        &mut self,
        device: &RenderDevice,
        queue: &RenderQueue,
        encoder: &mut CommandEncoder,
        params: &GpuParams,
    ) {
        let count = params.count;
        let formation = params.formation.x != 0;
        if count == 0 {
            self.count = 0;
            self.next_id = 0;
            self.order_base = 0;
            self.group_count = params.groups.x;
            self.formation_enabled = formation;
            return;
        }
        let remap = params.groups.x != self.group_count;
        let assign_slots = formation && (count != self.count || remap || !self.formation_enabled);
        if !params.simulates()
            && count == self.count
            && !remap
            && !self.scene_dirty
            && !self.predators_dirty
            && formation == self.formation_enabled
            && params.track.x == 0
        {
            self.count = count;
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
                    encoder.copy_buffer_to_buffer(
                        &old.arena,
                        0,
                        &grown.arena,
                        0,
                        self.count as u64 * METADATA_SIZE,
                    );
                    encoder.copy_buffer_to_buffer(
                        &old.arena,
                        (old.capacity as u64 * 2 + self.order_base as u64) * METADATA_SIZE,
                        &grown.arena,
                        grown.capacity as u64 * 2 * METADATA_SIZE,
                        self.count as u64 * 4,
                    );
                    encoder.copy_buffer_to_buffer(
                        &old.instances,
                        0,
                        &grown.instances,
                        0,
                        self.count as u64 * size_of::<crate::instancing::InstanceData>() as u64,
                    );
                }
            }
            self.buffers = Some(grown);
            self.scene_dirty = true;
            self.predators_dirty = true;
            self.order_base = 0;
        }
        let buffers = self.buffers.as_ref().unwrap();
        if self.scene_dirty {
            if !self.obstacles.is_empty() {
                let scene = SwarmScene {
                    obstacles: self.obstacles.clone(),
                    ..default()
                };
                let bytes: Vec<u8> = scene
                    .obstacle_records()
                    .iter()
                    .flat_map(|v| v.to_array().into_iter().flat_map(f32::to_le_bytes))
                    .collect();
                queue.write_buffer(
                    &buffers.arena,
                    buffers.capacity as u64 * 5 / 2 * METADATA_SIZE,
                    &bytes,
                );
                #[cfg(test)]
                {
                    self.scene_uploads += 1;
                }
            }
            self.scene_dirty = false;
        }
        if self.predators_dirty {
            if !self.predator_records.is_empty() {
                let bytes: Vec<u8> = self
                    .predator_records
                    .iter()
                    .flat_map(|v| v.to_array().into_iter().flat_map(u32::to_le_bytes))
                    .collect();
                queue.write_buffer(
                    &buffers.arena,
                    (buffers.capacity as u64 * 5 / 2 + 24) * METADATA_SIZE,
                    &bytes,
                );
                #[cfg(test)]
                {
                    self.predator_uploads += 1;
                }
            }
            self.predators_dirty = false;
        }
        let spawn_from = self.count.min(count);
        let block_count = buffers.table_size / SCAN_BLOCK;
        let step = GpuParams {
            track: UVec4::new(
                params.track.x,
                params.track.y,
                params.track.z,
                buffers.track_offset(),
            ),
            spawn_from,
            table_mask: buffers.table_size - 1,
            frame: self.frame,
            block_count,
            obstacles: UVec4::new(buffers.capacity * 5 / 2, params.obstacles.y, 0, 0),
            predators: UVec4::new(
                buffers.capacity * 5 / 2 + 24 + params.predators.z * 8,
                params.predators.y,
                params.predators.z,
                0,
            ),
            formation: UVec4::new(
                params.formation.x,
                0,
                count.div_ceil(WORKGROUP),
                buffers.capacity * 5 / 2 + MAX_OBSTACLES as u32 * 2,
            ),
            groups: UVec4::new(
                params.groups.x,
                self.next_id,
                self.order_base,
                buffers.capacity,
            ),
            ..*params
        };
        let mut bytes = encase::UniformBuffer::new(Vec::<u8>::new());
        bytes.write(&step).unwrap();
        // Queue writes all happen before submission. Encoder copies keep each substep's
        // uniforms distinct, including its random frame and spawn range.
        let upload = device.create_buffer_with_data(&BufferInitDescriptor {
            label: Some("gpu_flock_step_params"),
            contents: bytes.as_ref(),
            usage: BufferUsages::COPY_SRC,
        });
        encoder.copy_buffer_to_buffer(&upload, 0, &self.params, 0, GpuParams::min_size().get());

        let groups = |items: u32| items.div_ceil(WORKGROUP);
        if params.track.x != 0 {
            encoder.clear_buffer(
                &buffers.arena,
                buffers.track_offset() as u64 * METADATA_SIZE,
                Some(80),
            );
        }
        let mut pass = encoder.begin_compute_pass(&ComputePassDescriptor {
            label: Some("gpu_flock"),
            timestamp_writes: None,
        });
        pass.set_bind_group(0, &*buffers.bind_group, &[]);
        if count != self.count && spawn_from > 0 {
            pass.set_pipeline(&self.gather_metadata);
            pass.dispatch_workgroups(groups(spawn_from), 1, 1);
            pass.set_pipeline(&self.compact_metadata);
            pass.dispatch_workgroups(groups(spawn_from), 1, 1);
        }
        if spawn_from < count {
            pass.set_pipeline(&self.spawn);
            pass.dispatch_workgroups(groups(count - spawn_from), 1, 1);
        }
        if remap {
            pass.set_pipeline(&self.update_groups);
            pass.dispatch_workgroups(groups(count), 1, 1);
        }
        if assign_slots {
            for (pipeline, workgroups) in [
                (&self.formation_count, step.formation.z.div_ceil(64)),
                (&self.formation_scan, 1),
                (&self.formation_assign, groups(count)),
            ] {
                pass.set_pipeline(pipeline);
                pass.dispatch_workgroups(workgroups, 1, 1);
            }
        }
        self.formation_enabled = formation;
        self.next_id = self
            .next_id
            .checked_add(count - spawn_from)
            .expect("GPU spawn ID exhausted");
        self.group_count = params.groups.x;
        if !params.simulates() {
            if params.track.x != 0 {
                pass.set_pipeline(&self.track);
                pass.dispatch_workgroups(groups(count), 1, 1);
            }
            drop(pass);
            self.count = count;
            return;
        }
        if params.max_neighbors != 0 {
            // Exact separation in a fine grid; reuse the grid buffers for the coarse
            // alignment/cohesion pass and carry separation through the second sort.
            for (pipeline, workgroups) in [
                (&self.clear_counts, groups(buffers.table_size)),
                (&self.count_separation, groups(count)),
                (&self.scan_blocks, block_count),
                (&self.scan_block_sums, 1),
                (&self.scatter_fine, groups(count)),
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
            (
                &self.steer[(params.groups.x != 0) as usize
                    | (((params.obstacles.y != 0) as usize) << 1)
                    | ((formation as usize) << 2)
                    | (((params.predators.y != 0) as usize) << 3)],
                groups(count),
            ),
        ] {
            pass.set_pipeline(pipeline);
            pass.dispatch_workgroups(workgroups, 1, 1);
        }
        if params.track.x != 0 {
            pass.set_pipeline(&self.track);
            pass.dispatch_workgroups(groups(count), 1, 1);
        }
        drop(pass);
        if params.max_neighbors == 0 {
            self.order_base = buffers.capacity / 4 - self.order_base;
        }
        self.count = count;
        self.frame = self.frame.wrapping_add(1);
    }

    pub(crate) fn copy_track(&self, encoder: &mut CommandEncoder, destination: &Buffer) {
        if self.count > 0 {
            if let Some(buffers) = &self.buffers {
                encoder.copy_buffer_to_buffer(
                    &buffers.arena,
                    buffers.track_offset() as u64 * METADATA_SIZE,
                    destination,
                    0,
                    80,
                );
            }
        } else {
            encoder.clear_buffer(destination, 0, Some(80));
        }
    }

    #[cfg(test)]
    pub(crate) fn read_track(&self, device: &RenderDevice, queue: &RenderQueue) -> Vec<UVec4> {
        let offset = self.buffers.as_ref().unwrap().track_offset() as u64 * METADATA_SIZE;
        self.read_arena(device, queue, offset, 5)
    }

    /// Replaces the flock with `boids`, as if they had been spawned.
    #[cfg(test)]
    pub fn upload(&mut self, device: &RenderDevice, queue: &RenderQueue, boids: &[BoidData]) {
        let count = boids.len() as u32;
        self.formation_enabled = false;
        if count == 0 {
            self.count = 0;
            self.next_id = 0;
            self.group_count = 0;
            self.order_base = 0;
            return;
        }
        if self
            .buffers
            .as_ref()
            .is_none_or(|buffers| buffers.capacity < count)
        {
            self.buffers = Some(FlockBuffers::new(device, &self.layout, &self.params, count));
            self.scene_dirty = true;
            self.predators_dirty = true;
        }
        let mut bytes = Vec::with_capacity(boids.len() * BOID_SIZE as usize);
        let mut metadata = Vec::with_capacity(boids.len() * METADATA_SIZE as usize);
        for boid in boids {
            for value in boid.state.iter().flat_map(|v| v.to_array()) {
                bytes.extend(value.to_le_bytes());
            }
            for value in boid.metadata.to_array() {
                metadata.extend(value.to_le_bytes());
            }
        }
        let buffers = self.buffers.as_ref().unwrap();
        queue.write_buffer(&buffers.boids, 0, &bytes);
        queue.write_buffer(&buffers.arena, 0, &metadata);
        let order: Vec<u8> = (0..count).flat_map(u32::to_le_bytes).collect();
        queue.write_buffer(
            &buffers.arena,
            buffers.capacity as u64 * 2 * METADATA_SIZE,
            &order,
        );
        self.count = count;
        self.order_base = 0;
        self.next_id = boids
            .iter()
            .map(|b| b.metadata.x)
            .max()
            .unwrap()
            .checked_add(1)
            .unwrap();
        self.group_count = 0;
    }

    /// Read the draw stream to verify stable phase transport after sorting and paused growth.
    #[cfg(test)]
    pub fn read_instances(
        &self,
        device: &RenderDevice,
        queue: &RenderQueue,
    ) -> Vec<crate::instancing::InstanceData> {
        if self.count == 0 {
            return Vec::new();
        }
        let size = self.count as u64 * size_of::<crate::instancing::InstanceData>() as u64;
        let staging = device.create_buffer(&BufferDescriptor {
            label: Some("gpu_instance_readback"),
            size,
            usage: BufferUsages::MAP_READ | BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let mut encoder = device.create_command_encoder(&default());
        encoder.copy_buffer_to_buffer(
            &self.buffers.as_ref().unwrap().instances,
            0,
            &staging,
            0,
            size,
        );
        queue.submit([encoder.finish()]);
        staging
            .slice(..)
            .map_async(MapMode::Read, |result| result.unwrap());
        device.poll(PollType::wait_indefinitely()).unwrap();
        let bytes = staging.slice(..).get_mapped_range().unwrap();
        let floats: Vec<_> = bytes
            .as_chunks::<4>()
            .0
            .iter()
            .map(|v| f32::from_le_bytes(*v))
            .collect();
        floats
            .as_chunks::<12>()
            .0
            .iter()
            .map(|v| {
                [
                    Vec4::new(v[0], v[1], v[2], v[3]),
                    Vec4::new(v[4], v[5], v[6], v[7]),
                    Vec4::new(v[8], v[9], v[10], v[11]),
                ]
            })
            .collect()
    }

    #[cfg(test)]
    fn read_arena(
        &self,
        device: &RenderDevice,
        queue: &RenderQueue,
        offset: u64,
        records: u64,
    ) -> Vec<UVec4> {
        let size = records * METADATA_SIZE;
        let staging = device.create_buffer(&BufferDescriptor {
            label: Some("gpu_swarm_arena_readback"),
            size,
            usage: BufferUsages::MAP_READ | BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let mut encoder = device.create_command_encoder(&CommandEncoderDescriptor::default());
        encoder.copy_buffer_to_buffer(
            &self.buffers.as_ref().unwrap().arena,
            offset,
            &staging,
            0,
            size,
        );
        queue.submit([encoder.finish()]);
        staging
            .slice(..)
            .map_async(MapMode::Read, |result| result.unwrap());
        device.poll(PollType::wait_indefinitely()).unwrap();
        let bytes = staging.slice(..).get_mapped_range().unwrap();
        bytes
            .as_chunks::<16>()
            .0
            .iter()
            .map(|record| {
                UVec4::from_array(std::array::from_fn(|axis| {
                    u32::from_le_bytes(record[axis * 4..axis * 4 + 4].try_into().unwrap())
                }))
            })
            .collect()
    }

    #[cfg(test)]
    pub fn read_obstacle_records(
        &self,
        device: &RenderDevice,
        queue: &RenderQueue,
    ) -> (Vec<Vec4>, u32) {
        let offset = self.buffers.as_ref().unwrap().capacity as u64 * 5 / 2 * METADATA_SIZE;
        let records = self
            .read_arena(device, queue, offset, self.obstacles.len() as u64 * 2)
            .into_iter()
            .map(|record| Vec4::from_array(record.to_array().map(f32::from_bits)))
            .collect();
        (records, self.scene_uploads)
    }

    #[cfg(test)]
    pub fn read_formation_ranges(&self, device: &RenderDevice, queue: &RenderQueue) -> [UVec2; 8] {
        let offset = (self.buffers.as_ref().unwrap().capacity as u64 * 5 / 2
            + MAX_OBSTACLES as u64 * 2)
            * METADATA_SIZE;
        self.read_arena(device, queue, offset, 8)
            .into_iter()
            .map(|record| {
                assert_eq!(record.zw(), UVec2::ZERO);
                record.xy()
            })
            .collect::<Vec<_>>()
            .try_into()
            .unwrap()
    }

    #[cfg(test)]
    pub fn read_predators(&self, device: &RenderDevice, queue: &RenderQueue) -> (Vec<UVec4>, u32) {
        let offset = (self.buffers.as_ref().unwrap().capacity as u64 * 5 / 2 + 24) * METADATA_SIZE;
        (
            self.read_arena(device, queue, offset, PREDATOR_RECORDS as u64),
            self.predator_uploads,
        )
    }

    /// Copies the first `count` boids back to the CPU (blocking).
    #[cfg(test)]
    pub fn read_boids(&self, device: &RenderDevice, queue: &RenderQueue) -> Vec<BoidData> {
        if self.count == 0 {
            return Vec::new();
        }
        let state_size = self.count as u64 * BOID_SIZE;
        let order_offset = state_size + self.count as u64 * METADATA_SIZE;
        let size = order_offset + self.count as u64 * 4;
        let staging = device.create_buffer(&BufferDescriptor {
            label: Some("gpu_flock_readback"),
            size,
            usage: BufferUsages::MAP_READ | BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let mut encoder = device.create_command_encoder(&CommandEncoderDescriptor::default());
        encoder.copy_buffer_to_buffer(
            &self.buffers.as_ref().unwrap().boids,
            0,
            &staging,
            0,
            state_size,
        );
        encoder.copy_buffer_to_buffer(
            &self.buffers.as_ref().unwrap().arena,
            0,
            &staging,
            state_size,
            self.count as u64 * METADATA_SIZE,
        );
        encoder.copy_buffer_to_buffer(
            &self.buffers.as_ref().unwrap().arena,
            (self.buffers.as_ref().unwrap().capacity as u64 * 2 + self.order_base as u64)
                * METADATA_SIZE,
            &staging,
            order_offset,
            self.count as u64 * 4,
        );
        queue.submit([encoder.finish()]);
        staging
            .slice(..)
            .map_async(MapMode::Read, |result| result.unwrap());
        device.poll(PollType::wait_indefinitely()).unwrap();
        let bytes = staging
            .slice(..)
            .get_mapped_range()
            .expect("GPU flock readback is mapped after polling");
        bytes[..state_size as usize]
            .as_chunks::<{ BOID_SIZE as usize }>()
            .0
            .iter()
            .enumerate()
            .map(|(index, record)| {
                let word =
                    |i: usize| u32::from_le_bytes(record[i * 4..i * 4 + 4].try_into().unwrap());
                let state = std::array::from_fn(|i| {
                    Vec4::from_array(std::array::from_fn(|axis| {
                        f32::from_bits(word(i * 4 + axis))
                    }))
                });
                let offset = order_offset as usize + index * 4;
                let record_index =
                    u32::from_le_bytes(bytes[offset..offset + 4].try_into().unwrap()) as usize;
                assert!(
                    record_index < self.count as usize,
                    "invalid GPU metadata order map"
                );
                BoidData {
                    state,
                    metadata: UVec4::from_array(std::array::from_fn(|axis| {
                        let offset =
                            state_size as usize + record_index * METADATA_SIZE as usize + axis * 4;
                        u32::from_le_bytes(bytes[offset..offset + 4].try_into().unwrap())
                    })),
                }
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn presentation_params_skip_paused_steps_and_scale_simulated_delta() {
        let mut app = crate::presentation::tests::clock_app();
        app.init_resource::<PredatorFrame>()
            .init_resource::<FormationFrame>()
            .init_resource::<SwarmScene>()
            .init_resource::<GlobalState>()
            .init_resource::<GpuParams>()
            .add_systems(
                Update,
                update_gpu_params.after(crate::presentation::prepare_simulation),
            );
        for speed in [0.25, 1.0, 2.0] {
            app.world_mut()
                .resource_mut::<crate::presentation::Presentation>()
                .speed = speed;
            app.world_mut()
                .resource_mut::<Time<Real>>()
                .advance_by(std::time::Duration::from_millis(10));
            app.update();
            let params = app.world().resource::<GpuParams>();
            assert!(params.simulates());
            // dt is per step; above 1× the frame splits into ceil(speed) steps.
            assert!((params.dt - 0.01 * speed / speed.ceil()).abs() < 1e-8);
        }
        app.world_mut()
            .resource_mut::<crate::presentation::Presentation>()
            .paused = true;
        app.update();
        let params = app.world().resource::<GpuParams>();
        assert_eq!(params.count, 20_000);
        assert_eq!(params.dt, 0.0);
        assert!(!params.simulates());
        assert_eq!(
            app.world()
                .resource::<crate::presentation::SimulationFrame>()
                .steps,
            0
        );
    }

    #[test]
    fn scene_uniforms_use_the_upload_snapshot_and_clear_stale_enable_fields() {
        let mut scene = SwarmScene::default();
        scene.add_obstacle(Vec3::X, 12.0);
        let base = gpu_params(&GlobalState::default(), Vec3::ZERO, true, 0.01);
        let active = GpuParams { count: 200, ..base }.with_scene(&scene);
        assert_eq!(active.obstacles.y, 1);
        assert_eq!(active.obstacle_settings, Vec4::new(4.0, 1.0, 1.0, 0.0));
        scene.add_obstacle(Vec3::Y, 4.0);
        let edited = GpuParams {
            count: 1_000_000,
            ..active
        }
        .with_scene(&scene);
        assert_eq!(edited.obstacles.y, 2);
        assert_eq!(edited.obstacle_settings.z, vehicle::world_scale(1_000_000));
        scene.obstacle_strength = 0.0;
        let inactive = edited.with_scene(&scene);
        assert_eq!(inactive.obstacles, UVec4::ZERO);
        assert_eq!(inactive.obstacle_settings, Vec4::ZERO);
        scene.obstacle_strength = 8.0;
        scene.obstacles.clear();
        let cleared = edited.with_scene(&scene);
        assert_eq!(cleared.obstacles, UVec4::ZERO);
        assert_eq!(cleared.obstacle_settings, Vec4::ZERO);
        assert_eq!(cleared.dt, base.dt);
        assert_eq!(cleared.group_targets, base.group_targets);
    }

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
    fn attractor_uniforms_share_cpu_point_scale_and_sign() {
        use crate::swarm::AttractorMode;
        let mut state = GlobalState::default();
        let inactive = gpu_params(&state, Vec3::ZERO, false, 0.01);
        assert_eq!(inactive.attractor_position_radius, Vec4::ZERO);
        assert_eq!(inactive.attractor_strength, Vec4::ZERO);
        state.attractor.scripted_point = Some(Vec3::new(30.0, -20.0, 10.0));
        for count in [200, 20_000, 1_000_000] {
            state.vehicle_count = count;
            for mode in [AttractorMode::Attract, AttractorMode::Repel] {
                state.attractor.mode = mode;
                let params = gpu_params(&state, Vec3::ZERO, true, 0.01);
                assert_eq!(
                    params.attractor_position_radius.truncate(),
                    state.attractor.point().unwrap()
                );
                assert_eq!(
                    params.attractor_position_radius.w,
                    state.attractor.world_radius(count)
                );
                assert_eq!(
                    params.attractor_strength.x,
                    state.attractor.signed_strength()
                );
                assert_eq!(params.max_neighbors, state.gpu_max_neighbors);
            }
        }
        state.attractor.mode = AttractorMode::Off;
        let inactive = gpu_params(&state, Vec3::ZERO, false, 0.01);
        assert_eq!(inactive.attractor_position_radius, Vec4::ZERO);
        assert_eq!(inactive.attractor_strength, Vec4::ZERO);
    }

    #[test]
    fn formation_params_preserve_off_bytes_and_enable_the_explicit_cap() {
        let state = GlobalState {
            vehicle_count: 100_000,
            ..default()
        };
        let base = gpu_params(&state, Vec3::X, false, 0.01);
        let mut frame = FormationFrame::default();
        frame.settings.extent = 120.0;
        frame.morph = 0.7;
        assert_eq!(base.with_formation(&frame, 16), base);
        frame
            .settings
            .set_mode(crate::formation::FormationMode::Sphere);
        let active = base.with_formation(&frame, 16);
        assert_eq!(active.max_neighbors, 16);
        assert_eq!(base.with_formation(&frame, 0).max_neighbors, 0);
        assert_eq!(active.formation.x, 1);
        assert_eq!(
            active.formation_center_extent.w,
            120.0 * vehicle::world_scale(100_000)
        );
        assert_eq!(
            active.formation_settings.x,
            10.0 * vehicle::world_scale(100_000)
        );
        // Metadata/order maps, sixteen sphere records, then eight formation ranges.
        for capacity in [512_u32, 131072, 1048576] {
            let spheres = capacity * 5 / 2;
            let ranges = spheres + MAX_OBSTACLES as u32 * 2;
            assert_eq!(ranges - spheres, 16);
            assert!((ranges as u64 + 8) * METADATA_SIZE < 128 * 1024 * 1024);
        }
    }

    #[test]
    fn predator_params_preserve_off_and_neighbor_cap_and_fit_arena_tail() {
        let state = GlobalState {
            vehicle_count: 100_000,
            ..default()
        };
        let base = gpu_params(&state, Vec3::ZERO, false, 0.01);
        assert_eq!(base.with_predators(&PredatorFrame::default()), base);
        let mut settings = crate::predator::Predators::default();
        settings.add(Vec3::ZERO, 20.0);
        let frame = PredatorFrame::default().advance(&settings, 100_000, Vec3::ZERO, &default());
        let active = base.with_predators(&frame);
        assert_eq!(active.predators.y, 1);
        assert_eq!(active.max_neighbors, base.max_neighbors);
        assert_eq!(active.predator_settings.x, 4.0);
        settings.strength = 0.0;
        assert_eq!(
            base.with_predators(&frame.advance(&settings, 100_000, Vec3::ZERO, &default())),
            base
        );
        for capacity in [512_u32, 131072, 1048576] {
            let predators = capacity * 5 / 2 + 24;
            assert_eq!(predators - (capacity * 5 / 2 + 16), 8);
            assert_eq!(PREDATOR_RECORDS, 32);
            assert!(
                (predators as u64 + PREDATOR_RECORDS as u64) * METADATA_SIZE < 128 * 1024 * 1024
            );
        }
    }

    #[test]
    fn params_encode_with_wgsl_uniform_offsets() {
        // Naga's Params layout: seven u32s, eighteen f32s, vec3s at 112/128, Vec4s at 144/160.
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
            attractor_position_radius: Vec4::new(32.0, 33.0, 34.0, 35.0),
            attractor_strength: Vec4::new(36.0, 37.0, 38.0, 39.0),
            groups: UVec4::new(2, 100, 101, 102),
            group_targets: std::array::from_fn(|i| Vec4::splat((40 + i) as f32)),
            obstacles: UVec4::new(9000, 8, 0, 0),
            obstacle_settings: Vec4::new(48.0, 49.0, 50.0, 51.0),
            formation: UVec4::new(1, 0, 4, 9000),
            formation_center_extent: Vec4::new(52.0, 53.0, 54.0, 55.0),
            formation_settings: Vec4::new(56.0, 57.0, 58.0, 59.0),
            predators: UVec4::new(9001, 4, 3, 0),
            predator_settings: Vec4::new(60.0, 61.0, 62.0, 63.0),
            track: UVec4::new(2, 73, 74, 75),
            motion: Vec4::new(0.7, 1.0, 8.0, 0.0),
        };
        let mut buffer = encase::UniformBuffer::new(Vec::<u8>::new());
        buffer.write(&params).unwrap();
        let bytes = buffer.as_ref();
        assert_eq!(GpuParams::min_size().get(), 464);
        assert_eq!(bytes.len(), 464);
        for (axis, value) in [0.7_f32, 1.0, 8.0, 0.0].into_iter().enumerate() {
            assert_eq!(&bytes[448 + axis * 4..452 + axis * 4], &value.to_le_bytes());
        }
        for (axis, value) in [2_u32, 73, 74, 75].into_iter().enumerate() {
            assert_eq!(&bytes[432 + axis * 4..436 + axis * 4], &value.to_le_bytes());
        }
        assert_eq!(size_of::<BoidData>(), 80);
        assert_eq!(BOID_SIZE, 64);
        for (axis, value) in [9001_u32, 4, 3, 0].into_iter().enumerate() {
            assert_eq!(&bytes[400 + axis * 4..404 + axis * 4], &value.to_le_bytes());
        }
        for axis in 0..4 {
            assert_eq!(
                &bytes[416 + axis * 4..420 + axis * 4],
                &((60 + axis) as f32).to_le_bytes()
            );
        }
        for (axis, value) in [1_u32, 0, 4, 9000].into_iter().enumerate() {
            assert_eq!(&bytes[352 + axis * 4..356 + axis * 4], &value.to_le_bytes());
        }
        for axis in 0..8 {
            assert_eq!(
                &bytes[368 + axis * 4..372 + axis * 4],
                &((52 + axis) as f32).to_le_bytes()
            );
        }
        for (axis, value) in [9000_u32, 8, 0, 0].into_iter().enumerate() {
            assert_eq!(&bytes[320 + axis * 4..324 + axis * 4], &value.to_le_bytes());
        }
        for axis in 0..4 {
            assert_eq!(
                &bytes[336 + axis * 4..340 + axis * 4],
                &((48 + axis) as f32).to_le_bytes()
            );
        }
        for (axis, value) in [2_u32, 100, 101, 102].into_iter().enumerate() {
            assert_eq!(&bytes[176 + axis * 4..180 + axis * 4], &value.to_le_bytes());
        }
        for i in 0..8 {
            for axis in 0..4 {
                let offset = 192 + i * 16 + axis * 4;
                assert_eq!(&bytes[offset..offset + 4], &((40 + i) as f32).to_le_bytes());
            }
        }
        for i in 0..7 {
            assert_eq!(&bytes[i * 4..i * 4 + 4], &(i as u32 + 1).to_le_bytes());
        }
        for (offset, value) in (28..100)
            .step_by(4)
            .zip(8..26)
            .chain((112..124).step_by(4).zip(26..29))
            .chain((128..140).step_by(4).zip(29..32))
            .chain((144..176).step_by(4).zip(32..40))
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
