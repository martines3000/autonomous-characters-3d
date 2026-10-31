//! Draws the whole flock with one instanced draw call.
//!
//! Adapted from Bevy 0.20.1's `examples/shader_advanced/custom_shader_instancing.rs`.
//! One entity carries a mesh plus per-vehicle instance data; the render world uploads
//! the data into a reused vertex buffer and draws the mesh once per vehicle.

use bevy::{
    core_pipeline::core_3d::{Transparent3d, TransparentSortingInfo3d},
    ecs::{
        query::QueryItem,
        system::{lifetimeless::SRes, SystemParamItem},
    },
    mesh::{MeshVertexBufferLayoutRef, VertexBufferLayout},
    pbr::{
        MeshPipeline, MeshPipelineKey, MeshPipelineSystems, RenderMeshInstances, SetMeshBindGroup,
        SetMeshViewBindGroup, SetMeshViewBindingArrayBindGroup, ViewKeyCache,
    },
    prelude::*,
    render::{
        extract_component::{ExtractComponent, ExtractComponentPlugin},
        extract_resource::{ExtractResource, ExtractResourcePlugin},
        mesh::{allocator::MeshAllocator, RenderMesh, RenderMeshBufferInfo},
        render_asset::RenderAssets,
        render_phase::{
            AddRenderCommand, DrawFunctions, PhaseItem, PhaseItemExtraIndex, RenderCommand,
            RenderCommandResult, SetItemPipeline, TrackedRenderPass, ViewSortedRenderPhases,
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
};

const SHADER_ASSET_PATH: &str = "shaders/vehicle_instancing.wesl";

/// Per-vehicle data: `[position.xyz, scale]` and the rotation quaternion `[x, y, z, w]`.
pub type InstanceData = [Vec4; 2];

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

pub struct InstancingPlugin;

impl Plugin for InstancingPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<AppearanceUniform>()
            .add_plugins((
                ExtractComponentPlugin::<VehicleInstances>::default(),
                ExtractResourcePlugin::<AppearanceUniform>::default(),
            ))
            .add_systems(PostUpdate, update_appearance);

        app.sub_app_mut(RenderApp)
            .add_render_command::<Transparent3d, DrawInstanced>()
            .init_resource::<SpecializedMeshPipelines<InstancingPipeline>>()
            .insert_resource(InstanceBuffer(RawBufferVec::new(BufferUsages::VERTEX)))
            .init_resource::<FlockInstances>()
            .add_systems(RenderStartup, init_pipeline.after(MeshPipelineSystems))
            .add_systems(
                Render,
                (
                    queue_instanced.in_set(RenderSystems::QueueMeshes),
                    prepare_instance_buffer.in_set(RenderSystems::PrepareResources),
                    prepare_appearance.in_set(RenderSystems::PrepareBindGroups),
                ),
            );
    }
}

/// Shared render-only parameters. Neutral scale/color-mode slots are reserved for stage 2.
#[derive(Resource, ExtractResource, ShaderType, Clone, Default, PartialEq, Debug)]
#[extract_app(RenderApp)]
struct AppearanceUniform {
    colors: [Vec4; 7],
    // roughness, metallic, rim strength, render-only scale.
    material: Vec4,
    // emission strength, reserved for later shading parameters.
    shading: Vec4,
    // palette (0 = original HSV), color mode (0 = heading), reserved, reserved.
    modes: UVec4,
}

impl From<&Appearance> for AppearanceUniform {
    fn from(appearance: &Appearance) -> Self {
        let look = appearance.style.settings();
        Self {
            colors: appearance.palette.colors(),
            material: Vec4::new(look.roughness, look.metallic, look.rim, 1.0),
            shading: Vec4::new(look.emission, 0.0, 0.0, 0.0),
            modes: UVec4::new(
                u32::from(appearance.palette != HeadingPalette::Classic),
                0,
                0,
                0,
            ),
        }
    }
}

fn update_appearance(appearance: Res<Appearance>, mut uniform: ResMut<AppearanceUniform>) {
    if appearance.is_changed() {
        uniform.set_if_neq(AppearanceUniform::from(&*appearance));
    }
}

#[derive(Resource)]
struct AppearanceBinding {
    _buffer: UniformBuffer<AppearanceUniform>,
    bind_group: BindGroup,
}

fn prepare_appearance(
    mut commands: Commands,
    uniform: Res<AppearanceUniform>,
    existing: Option<Res<AppearanceBinding>>,
    pipeline: Res<InstancingPipeline>,
    cache: Res<PipelineCache>,
    device: Res<RenderDevice>,
    queue: Res<RenderQueue>,
) {
    if existing.is_some() && !uniform.is_changed() {
        return;
    }
    let mut buffer = UniformBuffer::from(uniform.clone());
    buffer.write_buffer(&device, &queue);
    let bind_group = device.create_bind_group(
        "vehicle_appearance",
        &cache.get_bind_group_layout(&pipeline.appearance_layout),
        &BindGroupEntries::single(buffer.binding().unwrap()),
    );
    commands.insert_resource(AppearanceBinding {
        _buffer: buffer,
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

#[allow(clippy::too_many_arguments)]
fn queue_instanced(
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
) {
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
            let Ok(pipeline) = pipelines.specialize(&pipeline_cache, &pipeline, key, &mesh.layout)
            else {
                continue;
            };
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
}

fn init_pipeline(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    mesh_pipeline: Res<MeshPipeline>,
) {
    commands.insert_resource(InstancingPipeline {
        shader: asset_server.load(SHADER_ASSET_PATH),
        mesh_pipeline: mesh_pipeline.clone(),
        appearance_layout: BindGroupLayoutDescriptor::new(
            "vehicle_appearance",
            &BindGroupLayoutEntries::single(
                ShaderStages::VERTEX_FRAGMENT,
                binding_types::uniform_buffer::<AppearanceUniform>(false),
            ),
        ),
    });
}

impl SpecializedMeshPipeline for InstancingPipeline {
    type Key = MeshPipelineKey;

    fn specialize(
        &self,
        key: Self::Key,
        layout: &MeshVertexBufferLayoutRef,
    ) -> Result<RenderPipelineDescriptor, SpecializedMeshPipelineError> {
        let mut descriptor = self.mesh_pipeline.specialize(key, layout)?;
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
            ],
        });
        descriptor.fragment.as_mut().unwrap().shader = self.shader.clone();
        Ok(descriptor)
    }
}

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
        assert_eq!(AppearanceUniform::from(&appearance).modes.x, 0);
        assert_eq!(AppearanceUniform::min_size().get(), 160);
    }
}
