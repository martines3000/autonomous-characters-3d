//! Draws the whole flock with one instanced draw call.
//!
//! Adapted from Bevy 0.20.1's `examples/shader_advanced/custom_shader_instancing.rs`.
//! One entity carries a mesh plus per-vehicle instance data; the render world uploads
//! the data into a reused vertex buffer and draws the mesh once per vehicle.

use std::sync::Arc;

use bevy::{
    core_pipeline::{
        core_3d::{
            Opaque3d, Opaque3dBatchSetKey, Opaque3dBinKey, Transparent3d, TransparentSortingInfo3d,
            CORE_3D_DEPTH_FORMAT,
        },
        prepass::{
            prepass_target_descriptors, DepthPrepass, MotionVectorPrepass, NormalPrepass,
            Opaque3dPrepass, OpaqueNoLightmap3dBatchSetKey, OpaqueNoLightmap3dBinKey,
        },
    },
    ecs::{
        query::QueryItem,
        system::{lifetimeless::SRes, SystemParamItem},
    },
    mesh::{MeshVertexBufferLayoutRef, VertexBufferLayout},
    pbr::{
        MeshPipeline, MeshPipelineKey, MeshPipelineSystems, PrepassPipeline, RenderMeshInstances,
        SetMeshBindGroup, SetMeshViewBindGroup, SetMeshViewBindingArrayBindGroup,
        SetPrepassViewBindGroup, SetPrepassViewEmptyBindGroup, Shadow, ShadowBatchSetKey,
        ShadowBinKey, ShadowView, ViewKeyCache,
    },
    prelude::*,
    render::{
        extract_component::{ExtractComponent, ExtractComponentPlugin},
        extract_resource::{ExtractResource, ExtractResourcePlugin},
        mesh::{allocator::MeshAllocator, RenderMesh, RenderMeshBufferInfo},
        render_asset::RenderAssets,
        render_phase::{
            AddRenderCommand, BinnedRenderPhaseType, DrawFunctions, PhaseItem, PhaseItemExtraIndex,
            RenderCommand, RenderCommandResult, SetItemPipeline, TrackedRenderPass,
            ViewBinnedRenderPhases, ViewSortedRenderPhases,
        },
        render_resource::*,
        renderer::{RenderDevice, RenderQueue},
        sync_component::SyncComponent,
        sync_world::MainEntity,
        view::ExtractedView,
        Render, RenderApp, RenderStartup, RenderSystems,
    },
};

use crate::{
    gpu_flock::GpuParams,
    scene::{Appearance, HeadingPalette},
    vehicle,
    vehicle_model::{BakedAnimation, VehicleModels, ATTRIBUTE_MATERIAL},
    GlobalState,
};

const SHADER_ASSET_PATH: &str = "shaders/vehicle_instancing.wesl";

/// Per-vehicle data: `[position.xyz, packed speed/group]` and rotation `[x, y, z, w]`.
/// Third Vec4 carries a stable phase in x; yzw are reserved.
/// See `vehicle_appearance::pack_speed_group`; physical and render scale live in the uniform.
pub type InstanceData = [Vec4; 3];

/// Instance data for every vehicle, drawn with this entity's `Mesh3d`.
#[derive(Component, Default, Clone)]
pub struct VehicleInstances(pub Vec<InstanceData>);

impl SyncComponent<RenderApp> for VehicleInstances {
    type Target = Self;
}

impl ExtractComponent<RenderApp> for VehicleInstances {
    type QueryData = &'static VehicleInstances;
    type QueryFilter = ();
    type Out = Self;

    fn extract_component(item: QueryItem<'_, '_, Self::QueryData>) -> Option<Self> {
        Some(item.clone())
    }
}

/// Developer measurement aid: skip only the flock draw, retaining simulation and scene work.
#[derive(Resource, ExtractResource, Clone)]
#[extract_app(RenderApp)]
pub(crate) struct FlockDraw(pub bool);

impl Default for FlockDraw {
    fn default() -> Self {
        Self(true)
    }
}

/// Extract only render policy; simulation and instance layouts remain untouched.
#[derive(Resource, ExtractResource, Clone, Default)]
#[extract_app(RenderApp)]
struct FlockRenderSettings {
    shadows: bool,
    opaque: bool,
    count: usize,
    cap: usize,
}
impl FlockRenderSettings {
    fn from_settings(appearance: &Appearance, state: &GlobalState) -> Self {
        Self {
            shadows: appearance.shadows,
            opaque: appearance.shadows || appearance.ambient_occlusion,
            count: state.vehicle_count,
            cap: appearance.shadow_cap,
        }
    }
    fn casts_shadows(&self) -> bool {
        self.shadows && self.count > 0 && self.count <= self.cap
    }
}
fn update_render_settings(
    appearance: Res<Appearance>,
    state: Res<GlobalState>,
    mut settings: ResMut<FlockRenderSettings>,
) {
    *settings = FlockRenderSettings::from_settings(&appearance, &state);
}

pub struct InstancingPlugin;

impl Plugin for InstancingPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<AppearanceUniform>()
            .init_resource::<FlockDraw>()
            .init_resource::<FlockRenderSettings>()
            .init_resource::<AnimationRender>()
            .add_plugins((
                ExtractComponentPlugin::<VehicleInstances>::default(),
                ExtractResourcePlugin::<AppearanceUniform>::default(),
                ExtractResourcePlugin::<FlockDraw>::default(),
                ExtractResourcePlugin::<FlockRenderSettings>::default(),
                ExtractResourcePlugin::<AnimationRender>::default(),
            ))
            .add_systems(
                PostUpdate,
                (update_appearance, update_animation, update_render_settings),
            );

        app.sub_app_mut(RenderApp)
            .add_render_command::<Transparent3d, DrawInstanced>()
            .add_render_command::<Opaque3d, DrawInstancedOpaque>()
            .add_render_command::<Opaque3dPrepass, DrawInstancedDepth>()
            .add_render_command::<Shadow, DrawInstancedDepth>()
            .init_resource::<SpecializedMeshPipelines<InstancingDepthPipeline>>()
            .init_resource::<SpecializedMeshPipelines<InstancingPipeline>>()
            .insert_resource(InstanceBuffer(RawBufferVec::new(BufferUsages::VERTEX)))
            .init_resource::<FlockInstances>()
            .add_systems(
                RenderStartup,
                init_pipeline
                    .after(MeshPipelineSystems)
                    .after(bevy::pbr::init_prepass_pipeline),
            )
            .add_systems(
                Render,
                (
                    // Stock queues remove dirty Mesh3d entries before checking materials.
                    // Run afterwards so they cannot erase our material-free flock bins.
                    queue_instanced
                        .in_set(RenderSystems::QueueMeshes)
                        .after(bevy::pbr::queue_material_meshes),
                    queue_instanced_depth
                        .in_set(RenderSystems::QueueMeshes)
                        .after(bevy::pbr::queue_prepass_material_meshes)
                        .after(bevy::pbr::queue_shadows),
                    prepare_instance_buffer.in_set(RenderSystems::PrepareResources),
                    prepare_appearance.in_set(RenderSystems::PrepareBindGroups),
                ),
            );
    }
}

/// Shared appearance parameters; simulation size is copied here without changing its meaning.
#[derive(Resource, ExtractResource, ShaderType, Clone, Default, PartialEq, Debug)]
#[extract_app(RenderApp)]
struct AppearanceUniform {
    colors: [Vec4; 7],
    // roughness, metallic, rim strength, render-only scale.
    material: Vec4,
    // Base emission, Glow minimum/additional emission, Glow forward stretch coefficient.
    shading: Vec4,
    // palette (0 = original HSV), color mode (0 = heading), Glow specialization flag, reserved.
    modes: UVec4,
    // width, length, width, altitude half-height.
    proportions: Vec4,
    solid_color: Vec4,
    group_colors: [Vec4; crate::swarm::MAX_GROUPS],
    // simulated seconds, procedural amplitude, speed-frequency gain, underwater specialization.
    motion: Vec4,
}

impl AppearanceUniform {
    fn from_settings(appearance: &Appearance, state: &GlobalState) -> Self {
        let look = appearance.style.settings();
        let underwater = appearance.environment == crate::environment::Environment::Underwater;
        Self {
            colors: appearance.palette.colors(),
            material: Vec4::new(
                look.roughness,
                look.metallic,
                look.rim,
                state.vehicle_size * appearance.vehicles.visual_size,
            ),
            shading: if appearance.style == crate::scene::SceneStyle::Glow {
                let floor = crate::vehicle_appearance::emissive_curve(0.0, appearance.glow);
                Vec4::new(
                    look.emission,
                    floor,
                    crate::vehicle_appearance::emissive_curve(1.0, appearance.glow) - floor,
                    crate::vehicle_appearance::streak_scale(1.0, appearance.streak) - 1.0,
                )
            } else {
                Vec4::new(look.emission, 0.0, 0.0, 0.0)
            },
            modes: UVec4::new(
                u32::from(appearance.palette != HeadingPalette::Classic),
                appearance.vehicles.color_mode as u32,
                u32::from(appearance.style == crate::scene::SceneStyle::Glow),
                state.motion.procedural_kind(appearance.vehicles.shape),
            ),
            motion: if state.motion.procedural_kind(appearance.vehicles.shape) != 0 {
                Vec4::new(
                    0.0,
                    state.motion.flap,
                    state.motion.frequency,
                    f32::from(underwater),
                )
            } else {
                Vec4::new(0.0, 0.0, 0.0, f32::from(underwater))
            },
            proportions: Vec4::new(
                appearance.vehicles.width,
                appearance.vehicles.length,
                appearance.vehicles.width,
                vehicle::walls(state.vehicle_count).y,
            ),
            group_colors: state.groups.table.map(|group| {
                Color::srgb_u8(group.color[0], group.color[1], group.color[2])
                    .to_linear()
                    .to_vec4()
            }),
            solid_color: Color::srgb_u8(
                appearance.vehicles.solid_color[0],
                appearance.vehicles.solid_color[1],
                appearance.vehicles.solid_color[2],
            )
            .to_linear()
            .to_vec4(),
        }
    }
}

fn update_appearance(
    appearance: Res<Appearance>,
    state: Res<GlobalState>,
    mut uniform: ResMut<AppearanceUniform>,
    clock: Option<Res<crate::presentation::SimulationClock>>,
) {
    if appearance.is_changed()
        || state.is_changed()
        || uniform.modes.w != 0
        || uniform.motion.w != 0.0
    {
        let mut next = AppearanceUniform::from_settings(&appearance, &state);
        if next.modes.w != 0 || next.motion.w != 0.0 {
            next.motion.x = clock.as_ref().map_or(0.0, |c| c.elapsed_secs());
        }
        uniform.set_if_neq(next);
    }
}

#[derive(Resource, ExtractResource, Clone, Default)]
#[extract_app(RenderApp)]
struct AnimationRender {
    mesh: Option<AssetId<Mesh>>,
    frames: Option<Arc<BakedAnimation>>,
    seconds: f64,
}
impl AnimationRender {
    fn advance(&mut self, models: &VehicleModels, appearance: &Appearance, delta: f64) {
        let same = match (&self.frames, &models.animation) {
            (Some(old), Some(new)) => Arc::ptr_eq(old, new),
            (None, None) => true,
            _ => false,
        };
        if !same {
            self.seconds = 0.0;
        } else if let Some(frames) = &self.frames {
            self.seconds = (self.seconds + delta * models.animation_speed as f64)
                .rem_euclid(frames.duration as f64);
        }
        self.frames = models.animation.clone();
        self.mesh = if appearance.vehicles.shape == crate::vehicle_appearance::VehicleShape::Custom
            && self.frames.is_some()
        {
            models.current.as_ref().map(Handle::id)
        } else {
            None
        };
    }
}
fn update_animation(
    models: Res<VehicleModels>,
    appearance: Res<Appearance>,
    clock: Res<crate::presentation::SimulationClock>,
    mut animation: ResMut<AnimationRender>,
) {
    animation.advance(&models, &appearance, clock.delta_secs_f64());
}

#[derive(ShaderType, Clone, Default)]
struct AnimationUniform {
    // time, duration, sample rate, reserved.
    playback: Vec4,
    // vertices per frame, frames, MeshAllocator base vertex, reserved.
    geometry: UVec4,
}

#[derive(Resource)]
struct AppearanceBinding {
    _buffer: UniformBuffer<AppearanceUniform>,
    _animation: UniformBuffer<AnimationUniform>,
    _frames: Buffer,
    source: Option<Arc<BakedAnimation>>,
    bind_group: BindGroup,
}

#[allow(clippy::too_many_arguments)]
fn prepare_appearance(
    mut commands: Commands,
    uniform: Res<AppearanceUniform>,
    animation: Res<AnimationRender>,
    existing: Option<Res<AppearanceBinding>>,
    pipeline: Res<InstancingPipeline>,
    cache: Res<PipelineCache>,
    device: Res<RenderDevice>,
    queue: Res<RenderQueue>,
    allocator: Res<MeshAllocator>,
) {
    let same = existing
        .as_ref()
        .is_some_and(|old| match (&old.source, &animation.frames) {
            (Some(old), Some(new)) => Arc::ptr_eq(old, new),
            (None, None) => true,
            _ => false,
        });
    // Static meshes rebuild only on change; animated ones refresh time and base vertex each frame.
    if same && !uniform.is_changed() && animation.frames.is_none() {
        return;
    }
    let frames = if same {
        existing.as_ref().unwrap()._frames.clone()
    } else {
        let mut buffer = RawBufferVec::new(BufferUsages::STORAGE);
        if let Some(frames) = &animation.frames {
            buffer.extend(frames.data.iter().copied());
        } else {
            buffer.push([Vec4::ZERO; 2]);
        }
        buffer.write_buffer(&device, &queue);
        buffer.buffer().unwrap().clone()
    };
    let mut buffer = UniformBuffer::from(uniform.clone());
    buffer.write_buffer(&device, &queue);
    let base = animation
        .mesh
        .and_then(|mesh| allocator.mesh_vertex_slice(&mesh))
        .map_or(0, |slice| slice.range.start);
    let mut metadata = UniformBuffer::from(animation.frames.as_ref().map_or_else(
        AnimationUniform::default,
        |frames| AnimationUniform {
            playback: Vec4::new(animation.seconds as f32, frames.duration, 30.0, 0.0),
            geometry: UVec4::new(frames.vertices as u32, frames.frame_count as u32, base, 0),
        },
    ));
    metadata.write_buffer(&device, &queue);
    let bind_group = device.create_bind_group(
        "vehicle_appearance",
        &cache.get_bind_group_layout(&pipeline.appearance_layout),
        &BindGroupEntries::sequential((
            buffer.binding().unwrap(),
            frames.as_entire_binding(),
            metadata.binding().unwrap(),
        )),
    );
    commands.insert_resource(AppearanceBinding {
        _buffer: buffer,
        _animation: metadata,
        _frames: frames,
        source: animation.frames.clone(),
        bind_group,
    });
}

/// The instance buffer and count the flock is drawn with. CPU modes fill it from
/// `VehicleInstances`; GPU mode points it at the buffer its compute pass writes.
#[derive(Resource, Default)]
pub(crate) struct FlockInstances {
    pub buffer: Option<Buffer>,
    pub count: u32,
}

// Reused across frames; it grows only when the flock outgrows it.
#[derive(Resource)]
struct InstanceBuffer(RawBufferVec<InstanceData>);

fn prepare_instance_buffer(
    flocks: Query<&VehicleInstances>,
    mut buffer: ResMut<InstanceBuffer>,
    mut draw: ResMut<FlockInstances>,
    gpu: Res<GpuParams>,
    device: Res<RenderDevice>,
    queue: Res<RenderQueue>,
) {
    if gpu.count > 0 {
        return;
    }
    buffer.0.clear();
    for flock in &flocks {
        buffer.0.extend(flock.0.iter().copied());
    }
    buffer.0.write_buffer(&device, &queue);
    *draw = FlockInstances {
        buffer: buffer.0.buffer().cloned(),
        count: buffer.0.len() as u32,
    };
}

#[allow(clippy::too_many_arguments, clippy::type_complexity)]
fn queue_instanced(
    draw: Res<FlockDraw>,
    settings: Res<FlockRenderSettings>,
    (opaque_functions, mut opaque_phases, allocator): (
        Res<DrawFunctions<Opaque3d>>,
        ResMut<ViewBinnedRenderPhases<Opaque3d>>,
        Res<MeshAllocator>,
    ),
    mut previous: Local<Vec<MainEntity>>,
    animation: Res<AnimationRender>,
    draw_functions: Res<DrawFunctions<Transparent3d>>,
    pipeline: Res<InstancingPipeline>,
    mut pipelines: ResMut<SpecializedMeshPipelines<InstancingPipeline>>,
    pipeline_cache: Res<PipelineCache>,
    meshes: Res<RenderAssets<RenderMesh>>,
    render_mesh_instances: Res<RenderMeshInstances>,
    flocks: Query<(Entity, &MainEntity), With<VehicleInstances>>,
    mut phases: ResMut<ViewSortedRenderPhases<Transparent3d>>,
    views: Query<&ExtractedView>,
    view_key_cache: Res<ViewKeyCache>,
    appearance: Res<AppearanceUniform>,
) {
    // Binned entries persist. Remove our previous entries even when disabled or despawned.
    for phase in opaque_phases.values_mut() {
        for entity in previous.iter() {
            phase.remove(*entity);
        }
    }
    previous.clear();
    previous.extend(flocks.iter().map(|(_, main)| *main));
    if !draw.0 {
        return;
    }
    let draw_function = draw_functions.read().id::<DrawInstanced>();
    for view in &views {
        let Some(phase) = phases.get_mut(&view.retained_view_entity) else {
            continue;
        };
        let Some(&view_key) = view_key_cache.get(&view.retained_view_entity) else {
            continue;
        };
        for (entity, main_entity) in &flocks {
            let Some(mesh_instance) = render_mesh_instances.render_mesh_queue_data(*main_entity)
            else {
                continue;
            };
            let Some(mesh) = meshes.get(mesh_instance.mesh_asset_id()) else {
                continue;
            };
            let key = view_key
                | MeshPipelineKey::from_primitive_topology_and_strip_index(
                    mesh.primitive_topology(),
                    mesh.index_format(),
                );
            let Ok(pipeline) = pipelines.specialize(
                &pipeline_cache,
                &pipeline,
                (
                    flock_pipeline_key(
                        key,
                        animation.mesh == Some(mesh_instance.mesh_asset_id()),
                        appearance.modes.z != 0,
                        appearance.modes.w != 0,
                        appearance.motion.w != 0.0,
                    ),
                    settings.shadows,
                    settings.opaque,
                ),
                &mesh.layout,
            ) else {
                continue;
            };
            if settings.opaque {
                if let (Some(opaque), Some(slabs)) = (
                    opaque_phases.get_mut(&view.retained_view_entity),
                    allocator.mesh_slabs(&mesh_instance.mesh_asset_id()),
                ) {
                    opaque.add(
                        Opaque3dBatchSetKey {
                            pipeline,
                            draw_function: opaque_functions.read().id::<DrawInstancedOpaque>(),
                            material_bind_group_index: None,
                            slabs,
                            lightmap_slab: None,
                        },
                        Opaque3dBinKey {
                            asset_id: mesh_instance.mesh_asset_id().into(),
                        },
                        (entity, *main_entity),
                        mesh_instance.current_uniform_index,
                        BinnedRenderPhaseType::NonMesh,
                    );
                }
                continue;
            }
            // Queued every frame, so the item goes away with the flock entity.
            phase.add_transient(Transparent3d {
                // The flock is opaque and writes depth; sorting it first means
                // blended objects (the target sphere) draw over it, not under it.
                sorting_info: TransparentSortingInfo3d::Sorted {
                    mesh_center: Vec3::ZERO,
                    depth_bias: f32::NEG_INFINITY,
                },
                entity: (entity, *main_entity),
                pipeline,
                draw_function,
                distance: f32::NEG_INFINITY,
                batch_range: 0..1,
                extra_index: PhaseItemExtraIndex::None,
                indexed: true,
            });
        }
    }
}

#[derive(Resource)]
struct InstancingPipeline {
    shader: Handle<Shader>,
    mesh_pipeline: MeshPipeline,
    appearance_layout: BindGroupLayoutDescriptor,
    empty_layout: BindGroupLayoutDescriptor,
}

fn init_pipeline(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    mesh_pipeline: Res<MeshPipeline>,
    prepass: Res<PrepassPipeline>,
) {
    commands.insert_resource(InstancingDepthPipeline {
        shader: asset_server.load(SHADER_ASSET_PATH),
        mesh_pipeline: mesh_pipeline.clone(),
        view_layout: prepass.view_layout_no_motion_vectors.clone(),
        motion_view_layout: prepass.view_layout_motion_vectors.clone(),
        empty_layout: prepass.empty_layout.clone(),
    });
    commands.insert_resource(InstancingPipeline {
        shader: asset_server.load(SHADER_ASSET_PATH),
        mesh_pipeline: mesh_pipeline.clone(),
        appearance_layout: appearance_layout(),
        empty_layout: prepass.empty_layout.clone(),
    });
}

fn appearance_layout() -> BindGroupLayoutDescriptor {
    BindGroupLayoutDescriptor::new(
        "vehicle_appearance",
        &BindGroupLayoutEntries::with_indices(
            ShaderStages::VERTEX_FRAGMENT,
            (
                (0, binding_types::uniform_buffer::<AppearanceUniform>(false)),
                (
                    1,
                    binding_types::storage_buffer_read_only_sized(false, None),
                ),
                (2, binding_types::uniform_buffer::<AnimationUniform>(false)),
            ),
        ),
    )
}

fn model_shader_defs(animated: bool, material: bool) -> impl Iterator<Item = &'static str> {
    [(animated, "ANIMATED"), (material, "MODEL_MATERIALS")]
        .into_iter()
        .filter_map(|(enabled, name)| enabled.then_some(name))
}

fn shadow_shader_defs(receives_shadows: bool) -> impl Iterator<Item = &'static str> {
    receives_shadows
        .then_some("FLOCK_SHADOW_RECEIVER")
        .into_iter()
}

fn underwater_shader_def(enabled: bool) -> Option<&'static str> {
    enabled.then_some("UNDERWATER")
}

fn flock_pipeline_key(
    view: MeshPipelineKey,
    animated: bool,
    glow: bool,
    procedural: bool,
    underwater: bool,
) -> (MeshPipelineKey, bool, bool, bool, bool) {
    (view, animated, glow, procedural, underwater)
}

impl SpecializedMeshPipeline for InstancingPipeline {
    type Key = ((MeshPipelineKey, bool, bool, bool, bool), bool, bool);

    fn specialize(
        &self,
        key: Self::Key,
        layout: &MeshVertexBufferLayoutRef,
    ) -> Result<RenderPipelineDescriptor, SpecializedMeshPipelineError> {
        let (key, receives_shadows, opaque) = key;
        // Preserve ENVIRONMENT_MAP, directional-light and fog definitions from the view key.
        // Both Bevy view bind groups below provide the corresponding PBR textures/uniforms.
        let mut descriptor = self.mesh_pipeline.specialize(key.0, layout)?;
        let material = layout.0.contains(ATTRIBUTE_MATERIAL);
        let mut attributes = vec![
            Mesh::ATTRIBUTE_POSITION.at_shader_location(0),
            Mesh::ATTRIBUTE_NORMAL.at_shader_location(1),
        ];
        if layout.0.contains(Mesh::ATTRIBUTE_COLOR) {
            attributes.push(Mesh::ATTRIBUTE_COLOR.at_shader_location(5));
        }
        if material {
            attributes.push(ATTRIBUTE_MATERIAL.at_shader_location(6));
        }
        descriptor.vertex.buffers[0] = layout.0.get_layout(&attributes)?;
        for name in model_shader_defs(key.1, material) {
            descriptor.vertex.shader_defs.push(name.into());
            descriptor
                .fragment
                .as_mut()
                .unwrap()
                .shader_defs
                .push(name.into());
        }
        if key.2 {
            descriptor.vertex.shader_defs.push("GLOW".into());
            descriptor
                .fragment
                .as_mut()
                .unwrap()
                .shader_defs
                .push("GLOW".into());
        }
        if key.3 {
            descriptor.vertex.shader_defs.push("PROCEDURAL".into());
        }
        descriptor
            .fragment
            .as_mut()
            .unwrap()
            .shader_defs
            .extend(shadow_shader_defs(receives_shadows).map(Into::into));
        if let Some(name) = underwater_shader_def(key.4) {
            descriptor
                .fragment
                .as_mut()
                .unwrap()
                .shader_defs
                .push(name.into());
        }
        if opaque {
            descriptor.layout[2] = self.empty_layout.clone();
        }
        descriptor.layout.push(self.appearance_layout.clone());
        descriptor.vertex.shader = self.shader.clone();
        descriptor.vertex.buffers.push(VertexBufferLayout {
            array_stride: size_of::<InstanceData>() as u64,
            step_mode: VertexStepMode::Instance,
            attributes: vec![
                // Above the locations Bevy uses for mesh attributes (0..=7).
                VertexAttribute {
                    format: VertexFormat::Float32x4,
                    offset: 0,
                    shader_location: 8,
                },
                VertexAttribute {
                    format: VertexFormat::Float32x4,
                    offset: VertexFormat::Float32x4.size(),
                    shader_location: 9,
                },
                VertexAttribute {
                    format: VertexFormat::Float32x4,
                    offset: 32,
                    shader_location: 10,
                },
            ],
        });
        descriptor.fragment.as_mut().unwrap().shader = self.shader.clone();
        Ok(descriptor)
    }
}

/// Depth-only and normal-prepass pipelines reuse the exact color vertex specialization.
#[derive(Resource)]
struct InstancingDepthPipeline {
    shader: Handle<Shader>,
    mesh_pipeline: MeshPipeline,
    view_layout: BindGroupLayoutDescriptor,
    motion_view_layout: BindGroupLayoutDescriptor,
    empty_layout: BindGroupLayoutDescriptor,
}
impl SpecializedMeshPipeline for InstancingDepthPipeline {
    // base vertex key, normal attachment, motion-view layout, shadow depth clamp.
    type Key = ((MeshPipelineKey, bool, bool, bool, bool), bool, bool, bool);
    fn specialize(
        &self,
        key: Self::Key,
        layout: &MeshVertexBufferLayoutRef,
    ) -> Result<RenderPipelineDescriptor, SpecializedMeshPipelineError> {
        let color = InstancingPipeline {
            shader: self.shader.clone(),
            mesh_pipeline: self.mesh_pipeline.clone(),
            appearance_layout: appearance_layout(),
            empty_layout: self.empty_layout.clone(),
        };
        let mut descriptor = color.specialize((key.0, false, false), layout)?;
        descriptor.label = Some("flock_depth_prepass".into());
        descriptor.layout[0] = if key.2 {
            self.motion_view_layout.clone()
        } else {
            self.view_layout.clone()
        };
        descriptor.layout[1] = self.empty_layout.clone();
        descriptor.layout[2] = self.empty_layout.clone();
        let mut defs = descriptor.vertex.shader_defs.clone();
        if key.1 {
            defs.push("FLOCK_NORMAL".into());
        }
        // Directional cascades need unclipped depth for casters between sun and view.
        // Clamp in the fragment shader, matching Bevy's depth-clip-control fallback.
        if key.3 {
            defs.push("FLOCK_SHADOW".into());
        }
        descriptor.fragment = (key.1 || key.3).then(|| FragmentState {
            shader: self.shader.clone(),
            shader_defs: defs,
            entry_point: Some("depth_fragment".into()),
            targets: if key.1 {
                prepass_target_descriptors(true, false, false)
            } else {
                vec![]
            },
            ..default()
        });
        if key.3 {
            descriptor.vertex.shader_defs.push("FLOCK_SHADOW".into());
        }
        descriptor.depth_stencil = Some(DepthStencilState {
            format: CORE_3D_DEPTH_FORMAT,
            depth_write_enabled: Some(true),
            depth_compare: Some(CompareFunction::GreaterEqual),
            stencil: default(),
            bias: default(),
        });
        Ok(descriptor)
    }
}

#[allow(clippy::too_many_arguments, clippy::type_complexity)]
fn queue_instanced_depth(
    draw: Res<FlockDraw>,
    settings: Res<FlockRenderSettings>,
    animation: Res<AnimationRender>,
    appearance: Res<AppearanceUniform>,
    pipeline: Res<InstancingDepthPipeline>,
    mut pipelines: ResMut<SpecializedMeshPipelines<InstancingDepthPipeline>>,
    cache: Res<PipelineCache>,
    meshes: Res<RenderAssets<RenderMesh>>,
    instances: Res<RenderMeshInstances>,
    allocator: Res<MeshAllocator>,
    flocks: Query<(Entity, &MainEntity), With<VehicleInstances>>,
    views: Query<(
        &ExtractedView,
        Has<DepthPrepass>,
        Has<NormalPrepass>,
        Has<MotionVectorPrepass>,
        Has<ShadowView>,
        Option<&Msaa>,
    )>,
    mut prepasses: ResMut<ViewBinnedRenderPhases<Opaque3dPrepass>>,
    mut shadows: ResMut<ViewBinnedRenderPhases<Shadow>>,
    (prepass_functions, shadow_functions): (
        Res<DrawFunctions<Opaque3dPrepass>>,
        Res<DrawFunctions<Shadow>>,
    ),
    mut previous: Local<Vec<MainEntity>>,
) {
    for phase in prepasses.values_mut() {
        for entity in previous.iter() {
            phase.remove(*entity);
        }
    }
    for phase in shadows.values_mut() {
        for entity in previous.iter() {
            phase.remove(*entity);
        }
    }
    previous.clear();
    previous.extend(flocks.iter().map(|(_, main)| *main));
    if !draw.0 {
        return;
    }
    for (view, depth, normal, motion, shadow, msaa) in &views {
        if shadow {
            if !settings.casts_shadows() {
                continue;
            }
        } else if !depth && !normal {
            continue;
        }
        for (entity, main) in &flocks {
            let Some(instance) = instances.render_mesh_queue_data(*main) else {
                continue;
            };
            let mesh_id = instance.mesh_asset_id();
            let (Some(mesh), Some(slabs)) = (meshes.get(mesh_id), allocator.mesh_slabs(&mesh_id))
            else {
                continue;
            };
            let key = MeshPipelineKey::from_msaa_samples(if shadow {
                1
            } else {
                msaa.map_or(1, Msaa::samples)
            }) | MeshPipelineKey::from_primitive_topology_and_strip_index(
                mesh.primitive_topology(),
                mesh.index_format(),
            );
            let Ok(id) = pipelines.specialize(
                &cache,
                &pipeline,
                (
                    flock_pipeline_key(
                        key,
                        animation.mesh == Some(mesh_id),
                        appearance.modes.z != 0,
                        appearance.modes.w != 0,
                        // Caustics only shade the color pass.
                        false,
                    ),
                    normal,
                    motion,
                    shadow,
                ),
                &mesh.layout,
            ) else {
                continue;
            };
            if shadow {
                if let Some(phase) = shadows.get_mut(&view.retained_view_entity) {
                    phase.add(
                        ShadowBatchSetKey {
                            pipeline: id,
                            draw_function: shadow_functions.read().id::<DrawInstancedDepth>(),
                            material_bind_group_index: None,
                            slabs,
                        },
                        ShadowBinKey {
                            asset_id: mesh_id.into(),
                        },
                        (entity, *main),
                        instance.current_uniform_index,
                        BinnedRenderPhaseType::NonMesh,
                    );
                }
            } else if let Some(phase) = prepasses.get_mut(&view.retained_view_entity) {
                phase.add(
                    OpaqueNoLightmap3dBatchSetKey {
                        pipeline: id,
                        draw_function: prepass_functions.read().id::<DrawInstancedDepth>(),
                        material_bind_group_index: None,
                        slabs,
                    },
                    OpaqueNoLightmap3dBinKey {
                        asset_id: mesh_id.into(),
                    },
                    (entity, *main),
                    instance.current_uniform_index,
                    BinnedRenderPhaseType::NonMesh,
                );
            }
        }
    }
}

type DrawInstancedDepth = (
    SetItemPipeline,
    SetPrepassViewBindGroup<0>,
    SetPrepassViewEmptyBindGroup<1>,
    SetPrepassViewEmptyBindGroup<2>,
    SetAppearanceBindGroup,
    DrawMeshInstanced,
);

// Opaque NonMesh bins own their instance buffer; they have no Bevy mesh-uniform batch.
type DrawInstancedOpaque = (
    SetItemPipeline,
    SetMeshViewBindGroup<0>,
    SetMeshViewBindingArrayBindGroup<1>,
    SetPrepassViewEmptyBindGroup<2>,
    SetAppearanceBindGroup,
    DrawMeshInstanced,
);

type DrawInstanced = (
    SetItemPipeline,
    SetMeshViewBindGroup<0>,
    SetMeshViewBindingArrayBindGroup<1>,
    SetMeshBindGroup<2>,
    SetAppearanceBindGroup,
    DrawMeshInstanced,
);

struct SetAppearanceBindGroup;
impl<P: PhaseItem> RenderCommand<P> for SetAppearanceBindGroup {
    type Param = SRes<AppearanceBinding>;
    type ViewQuery = ();
    type ItemQuery = ();
    fn render<'w>(
        _item: &P,
        _view: (),
        _entity: Option<()>,
        binding: SystemParamItem<'w, '_, Self::Param>,
        pass: &mut TrackedRenderPass<'w>,
    ) -> RenderCommandResult {
        pass.set_bind_group(3, &binding.into_inner().bind_group, &[]);
        RenderCommandResult::Success
    }
}

struct DrawMeshInstanced;

impl<P: PhaseItem> RenderCommand<P> for DrawMeshInstanced {
    type Param = (
        SRes<RenderAssets<RenderMesh>>,
        SRes<RenderMeshInstances>,
        SRes<MeshAllocator>,
        SRes<FlockInstances>,
    );
    type ViewQuery = ();
    type ItemQuery = ();

    #[inline]
    fn render<'w>(
        item: &P,
        _view: (),
        _entity: Option<()>,
        (meshes, render_mesh_instances, mesh_allocator, instances): SystemParamItem<
            'w,
            '_,
            Self::Param,
        >,
        pass: &mut TrackedRenderPass<'w>,
    ) -> RenderCommandResult {
        let mesh_allocator = mesh_allocator.into_inner();
        let instances = instances.into_inner();
        let Some(mesh_instance) = render_mesh_instances.render_mesh_queue_data(item.main_entity())
        else {
            return RenderCommandResult::Skip;
        };
        let Some(gpu_mesh) = meshes.into_inner().get(mesh_instance.mesh_asset_id()) else {
            return RenderCommandResult::Skip;
        };
        let (Some(instance_buffer), count) = (&instances.buffer, instances.count) else {
            return RenderCommandResult::Skip;
        };
        if count == 0 {
            return RenderCommandResult::Skip;
        }
        let Some(vertex_slice) = mesh_allocator.mesh_vertex_slice(&mesh_instance.mesh_asset_id())
        else {
            return RenderCommandResult::Skip;
        };

        pass.set_vertex_buffer(0, vertex_slice.buffer.slice(..));
        pass.set_vertex_buffer(1, instance_buffer.slice(..));

        match &gpu_mesh.buffer_info {
            RenderMeshBufferInfo::Indexed {
                index_format,
                count: index_count,
            } => {
                let Some(index_slice) =
                    mesh_allocator.mesh_index_slice(&mesh_instance.mesh_asset_id())
                else {
                    return RenderCommandResult::Skip;
                };
                pass.set_index_buffer(index_slice.buffer.slice(..), *index_format);
                pass.draw_indexed(
                    index_slice.range.start..(index_slice.range.start + index_count),
                    vertex_slice.range.start as i32,
                    0..count,
                );
            }
            RenderMeshBufferInfo::NonIndexed => {
                pass.draw(vertex_slice.range, 0..count);
            }
        }
        RenderCommandResult::Success
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{scene::SceneStyle, GlobalState};

    #[test]
    fn shadow_gating_keeps_default_uniforms_keys_and_skips_over_cap() {
        let mut appearance = Appearance::default();
        let mut state = GlobalState::default();
        let old_uniform = AppearanceUniform::from_settings(&appearance, &state);
        let off = FlockRenderSettings::from_settings(&appearance, &state);
        assert!(!off.opaque && !off.casts_shadows());
        assert!(shadow_shader_defs(off.shadows).next().is_none());
        let key = flock_pipeline_key(
            MeshPipelineKey::from_msaa_samples(4),
            false,
            false,
            false,
            false,
        );
        appearance.shadow_cap = 10;
        assert!(!FlockRenderSettings::from_settings(&appearance, &state).casts_shadows());
        assert_eq!(
            old_uniform,
            AppearanceUniform::from_settings(&appearance, &state)
        );
        appearance.shadows = true;
        for (count, expected) in [
            (0, false),
            (1, true),
            (10, true),
            (11, false),
            (1_000_000, false),
        ] {
            state.vehicle_count = count;
            let settings = FlockRenderSettings::from_settings(&appearance, &state);
            assert!(settings.opaque);
            assert_eq!(settings.casts_shadows(), expected);
        }
        assert_eq!(
            shadow_shader_defs(true).collect::<Vec<_>>(),
            ["FLOCK_SHADOW_RECEIVER"]
        );
        appearance.shadows = false;
        appearance.ambient_occlusion = true;
        let settings = FlockRenderSettings::from_settings(&appearance, &state);
        assert!(settings.opaque && !settings.casts_shadows());
        assert_eq!(
            key,
            flock_pipeline_key(
                MeshPipelineKey::from_msaa_samples(4),
                false,
                false,
                false,
                false
            )
        );
    }

    #[test]
    fn underwater_key_gates_caustics_independently_of_shape_glow_and_ibl() {
        use crate::environment::Environment;
        let view = MeshPipelineKey::ENVIRONMENT_MAP;
        for environment in Environment::ALL {
            for style in SceneStyle::ALL {
                let appearance = Appearance {
                    environment,
                    style,
                    ..default()
                };
                let u = AppearanceUniform::from_settings(&appearance, &GlobalState::default());
                let active = environment == Environment::Underwater;
                let key = flock_pipeline_key(
                    view,
                    true,
                    u.modes.z != 0,
                    u.modes.w != 0,
                    u.motion.w != 0.0,
                );
                assert_eq!(key.0, view);
                assert_eq!(key.4, active);
                assert_eq!(underwater_shader_def(key.4), active.then_some("UNDERWATER"));
            }
        }
        // No deformation: caustics still advance, and pause freezes their simulation clock.
        let mut app = crate::presentation::tests::clock_app();
        app.init_resource::<GlobalState>()
            .insert_resource(Appearance {
                environment: Environment::Underwater,
                ..default()
            })
            .init_resource::<AppearanceUniform>()
            .add_systems(
                Update,
                update_appearance.after(crate::presentation::prepare_simulation),
            );
        let step = |app: &mut App| {
            app.world_mut()
                .resource_mut::<Time<Real>>()
                .advance_by(std::time::Duration::from_millis(20));
            app.update();
        };
        step(&mut app);
        step(&mut app);
        let before = app.world().resource::<AppearanceUniform>().motion.x;
        assert!(before > 0.0);
        assert_eq!(app.world().resource::<AppearanceUniform>().modes.w, 0);
        app.world_mut()
            .resource_mut::<crate::presentation::Presentation>()
            .paused = true;
        step(&mut app);
        step(&mut app);
        assert_eq!(app.world().resource::<AppearanceUniform>().motion.x, before);
        app.world_mut().resource_mut::<Appearance>().environment = Environment::Midday;
        step(&mut app);
        assert_eq!(
            app.world().resource::<AppearanceUniform>().motion,
            Vec4::ZERO
        );
    }

    #[test]
    fn environment_off_keeps_pipeline_keys_and_ibl_keeps_view_specialization() {
        use crate::environment::Environment;
        let view = MeshPipelineKey::from_msaa_samples(4)
            | MeshPipelineKey::from_target_format(TextureFormat::Rgba8UnormSrgb);
        for style in SceneStyle::ALL {
            let appearance = Appearance {
                style,
                environment: Environment::Off,
                ..default()
            };
            let uniform = AppearanceUniform::from_settings(&appearance, &GlobalState::default());
            let key = flock_pipeline_key(view, false, uniform.modes.z != 0, false, false);
            assert_eq!(key, (view, false, style == SceneStyle::Glow, false, false));
            assert!(!key.0.contains(MeshPipelineKey::ENVIRONMENT_MAP));
            let lit_view = view | MeshPipelineKey::ENVIRONMENT_MAP;
            assert!(flock_pipeline_key(lit_view, true, true, true, false)
                .0
                .contains(MeshPipelineKey::ENVIRONMENT_MAP));
        }
    }

    #[test]
    fn glow_keeps_old_uniforms_and_view_keys_and_uses_hdr_msaa_specialization() {
        let state = GlobalState::default();
        let view = MeshPipelineKey::from_msaa_samples(4)
            | MeshPipelineKey::from_target_format(TextureFormat::Rgba8UnormSrgb);
        for style in [SceneStyle::Studio, SceneStyle::Classic] {
            let appearance = Appearance { style, ..default() };
            let uniform = AppearanceUniform::from_settings(&appearance, &state);
            assert_eq!(
                uniform.shading,
                Vec4::new(style.settings().emission, 0.0, 0.0, 0.0)
            );
            assert_eq!(uniform.modes.zw(), UVec2::ZERO);
            assert_eq!(
                flock_pipeline_key(view, false, false, false, false),
                (view, false, false, false, false)
            );
            let adjusted = Appearance {
                glow: 20.0,
                streak: 3.0,
                ..appearance
            };
            assert_eq!(uniform, AppearanceUniform::from_settings(&adjusted, &state));
        }
        let hdr_view = MeshPipelineKey::from_msaa_samples(4)
            | MeshPipelineKey::from_target_format(TextureFormat::Rgba16Float);
        assert_eq!(hdr_view.target_format(), TextureFormat::Rgba16Float);
        assert_ne!(hdr_view, view);
        let appearance = Appearance {
            style: SceneStyle::Glow,
            ..default()
        };
        let uniform = AppearanceUniform::from_settings(&appearance, &state);
        assert_eq!(uniform.modes.z, 1);
        assert_eq!(
            flock_pipeline_key(hdr_view, true, true, false, false),
            (hdr_view, true, true, false, false)
        );
        assert_eq!(hdr_view.msaa_samples(), 4);
        assert_eq!(uniform.shading.y + uniform.shading.z, appearance.glow);
        assert!((uniform.shading.w - appearance.streak).abs() < 1e-6);
    }

    #[test]
    fn procedural_pipeline_gating_and_clock_pause() {
        use crate::vehicle_appearance::VehicleShape;
        let mut state = GlobalState::default();
        let mut appearance = Appearance::default();
        for (shape, kind) in [
            (VehicleShape::Dart, 0),
            (VehicleShape::Bird, 1),
            (VehicleShape::Fish, 2),
            (VehicleShape::Chevron, 3),
            (VehicleShape::Custom, 0),
        ] {
            appearance.vehicles.shape = shape;
            let u = AppearanceUniform::from_settings(&appearance, &state);
            assert_eq!(u.modes.w, kind);
            assert_eq!(
                flock_pipeline_key(MeshPipelineKey::empty(), false, true, kind != 0, false).3,
                kind != 0
            );
        }
        state.motion.flap = 0.0;
        appearance.vehicles.shape = VehicleShape::Bird;
        assert_eq!(
            AppearanceUniform::from_settings(&appearance, &state).motion,
            Vec4::ZERO
        );
        let mut app = crate::presentation::tests::clock_app();
        app.init_resource::<GlobalState>()
            .insert_resource(appearance)
            .init_resource::<AppearanceUniform>()
            .add_systems(
                Update,
                update_appearance.after(crate::presentation::prepare_simulation),
            );
        app.world_mut()
            .resource_mut::<Time<Real>>()
            .advance_by(std::time::Duration::from_millis(20));
        app.update();
        let time = app.world().resource::<AppearanceUniform>().motion.x;
        assert!(time > 0.0);
        app.world_mut()
            .resource_mut::<crate::presentation::Presentation>()
            .paused = true;
        for _ in 0..4 {
            app.world_mut()
                .resource_mut::<Time<Real>>()
                .advance_by(std::time::Duration::from_secs(1));
            app.update();
            assert_eq!(app.world().resource::<AppearanceUniform>().motion.x, time);
        }
    }

    #[test]
    fn vehicle_appearance_changes_preserve_every_simulation_field_and_gpu_parameter() {
        use crate::{
            gpu_flock::gpu_params,
            vehicle_appearance::{ColorMode, VehicleShape},
        };
        let state = GlobalState {
            vehicle_size: 1.75,
            ..default()
        };
        let mut app = App::new();
        app.insert_resource(state.clone())
            .init_resource::<Appearance>()
            .init_resource::<AppearanceUniform>()
            .add_systems(Update, update_appearance);
        let encode = |state: &GlobalState| {
            let mut bytes = encase::UniformBuffer::new(Vec::<u8>::new());
            bytes
                .write(&gpu_params(
                    state,
                    Vec3::new(3.0, 5.0, 7.0),
                    true,
                    1.0 / 60.0,
                ))
                .unwrap();
            bytes.into_inner()
        };
        let before = encode(&state);
        for shape in VehicleShape::ALL.into_iter().chain([VehicleShape::Custom]) {
            for color_mode in ColorMode::ALL {
                {
                    let mut appearance = app.world_mut().resource_mut::<Appearance>();
                    appearance.vehicles.shape = shape;
                    appearance.vehicles.color_mode = color_mode;
                    appearance.vehicles.visual_size = 2.0;
                    appearance.vehicles.length = 2.5;
                    appearance.vehicles.width = 0.5;
                    appearance.vehicles.solid_color = [255, 40, 120];
                }
                app.update();
                assert_eq!(app.world().resource::<GlobalState>(), &state);
                assert_eq!(encode(app.world().resource::<GlobalState>()), before);
                let uniform = app.world().resource::<AppearanceUniform>();
                assert_eq!(uniform.material.w, 3.5);
                assert_eq!(
                    uniform.proportions,
                    Vec4::new(0.5, 2.5, 0.5, vehicle::walls(state.vehicle_count).y)
                );
                assert_eq!(uniform.modes.y, color_mode as u32);
            }
        }
    }

    #[test]
    fn switching_style_and_palette_changes_only_render_parameters() {
        let mut app = App::new();
        app.init_resource::<GlobalState>()
            .init_resource::<Appearance>()
            .init_resource::<AppearanceUniform>()
            .add_systems(Update, update_appearance);
        app.update();
        let aurora = app.world().resource::<AppearanceUniform>().clone();
        app.world_mut().resource_mut::<Appearance>().style = SceneStyle::Classic;
        app.update();
        let classic = app.world().resource::<AppearanceUniform>().clone();
        assert_eq!(classic.colors, aurora.colors);
        assert_eq!(classic.shading.x, 0.0);
        assert_eq!(aurora.shading.x, 0.15);
        assert_eq!(classic.material, Vec4::new(0.6, 0.0, 0.0, 1.0));
        app.world_mut().resource_mut::<Appearance>().palette = HeadingPalette::Dusk;
        app.update();
        let dusk = app.world().resource::<AppearanceUniform>();
        assert_eq!(dusk.material, classic.material);
        assert_ne!(dusk.colors, classic.colors);
        assert_eq!(app.world().resource::<GlobalState>().vehicle_size, 1.0);
        assert_eq!(app.world().resource::<GlobalState>().vehicle_count, 20_000);
        let appearance = Appearance {
            palette: HeadingPalette::Classic,
            ..default()
        };
        assert_eq!(
            AppearanceUniform::from_settings(&appearance, &GlobalState::default())
                .modes
                .x,
            0
        );
        assert_eq!(AppearanceUniform::min_size().get(), 336);
    }

    #[test]
    fn group_colors_extend_uniform_without_changing_existing_modes() {
        let appearance = Appearance::default();
        let mut state = GlobalState::default();
        let before = AppearanceUniform::from_settings(&appearance, &state);
        state.groups.count = 2;
        state.groups.table[1].color = [255, 0, 128];
        let after = AppearanceUniform::from_settings(&appearance, &state);
        assert_eq!(before.colors, after.colors);
        assert_eq!(before.material, after.material);
        assert_eq!(before.modes, after.modes);
        assert_eq!(before.proportions, after.proportions);
        let expected = Color::srgb_u8(255, 0, 128).to_linear().to_vec4();
        assert_eq!(after.group_colors[1], expected);
        let mut buffer =
            bevy::render::render_resource::encase::UniformBuffer::new(Vec::<u8>::new());
        buffer.write(&after).unwrap();
        assert_eq!(buffer.as_ref().len(), 336);
        for (axis, value) in expected.to_array().into_iter().enumerate() {
            let offset = 192 + 16 + axis * 4;
            assert_eq!(&buffer.as_ref()[offset..offset + 4], &value.to_le_bytes());
        }
    }

    #[test]
    fn animation_uses_simulation_delta_pause_speed_and_mesh_specific_specialization() {
        use crate::{
            presentation::{tests::clock_app, Presentation},
            vehicle_appearance::VehicleShape,
        };
        use std::time::Duration;
        let mut app = clock_app();
        app.init_resource::<VehicleModels>()
            .init_resource::<Appearance>()
            .init_resource::<AnimationRender>()
            .add_systems(PostUpdate, update_animation);
        let frames = Arc::new(BakedAnimation {
            data: vec![[Vec4::ZERO; 2]; 31],
            vertices: 1,
            frame_count: 31,
            duration: 1.0,
        });
        {
            let mut models = app.world_mut().resource_mut::<VehicleModels>();
            models.current = Some(Handle::default());
            models.animation = Some(frames);
        }
        app.world_mut().resource_mut::<Appearance>().vehicles.shape = VehicleShape::Custom;
        app.update();
        for (simulation_speed, animation_speed, expected) in
            [(1.0, 1.0, 0.1), (0.5, 2.0, 0.1), (2.0, 0.5, 0.1)]
        {
            app.world_mut().resource_mut::<Presentation>().speed = simulation_speed;
            app.world_mut()
                .resource_mut::<VehicleModels>()
                .animation_speed = animation_speed;
            let before = app.world().resource::<AnimationRender>().seconds;
            app.world_mut()
                .resource_mut::<Time<Real>>()
                .advance_by(Duration::from_millis(100));
            app.update();
            let after = app.world().resource::<AnimationRender>();
            assert!((after.seconds - before - expected).abs() < 1e-6);
            assert!(after.mesh.is_some());
        }
        app.world_mut().resource_mut::<Presentation>().paused = true;
        let before = app.world().resource::<AnimationRender>().seconds;
        for _ in 0..4 {
            app.world_mut()
                .resource_mut::<Time<Real>>()
                .advance_by(Duration::from_secs(2));
            app.update();
            assert_eq!(app.world().resource::<AnimationRender>().seconds, before);
        }
        app.world_mut().resource_mut::<Appearance>().vehicles.shape = VehicleShape::Dart;
        app.update();
        assert!(app.world().resource::<AnimationRender>().mesh.is_none());
        app.world_mut().resource_mut::<VehicleModels>().animation = None;
        app.world_mut().resource_mut::<Appearance>().vehicles.shape = VehicleShape::Custom;
        app.update();
        assert!(app.world().resource::<AnimationRender>().mesh.is_none());
        assert!(model_shader_defs(false, false).next().is_none());
        assert_eq!(
            model_shader_defs(false, true).collect::<Vec<_>>(),
            ["MODEL_MATERIALS"]
        );
        assert_eq!(
            model_shader_defs(true, true).collect::<Vec<_>>(),
            ["ANIMATED", "MODEL_MATERIALS"]
        );
    }
}
