//! Sand uses the existing ground entity and restores its original mesh/material on exit.
use bevy::{
    pbr::{ExtendedMaterial, MaterialExtension},
    prelude::*,
    render::render_resource::AsBindGroup,
    shader::ShaderRef,
};

use crate::{
    environment::Environment,
    presentation::SimulationClock,
    scene::{Appearance, StudioSurface},
    vehicle, GlobalState,
};

type SandMaterial = ExtendedMaterial<StandardMaterial, SandCaustics>;

#[derive(Asset, AsBindGroup, Reflect, Debug, Clone)]
struct SandCaustics {
    // Simulated seconds and water-column half-height, shared with the flock shader.
    #[uniform(100)]
    water: Vec4,
}
impl MaterialExtension for SandCaustics {
    fn fragment_shader() -> ShaderRef {
        "shaders/underwater_sand.wesl".into()
    }
}

#[derive(Resource, Default)]
struct SandCache(Option<(Handle<Mesh>, Handle<SandMaterial>)>);

#[derive(Component)]
struct OriginalGround {
    mesh: Handle<Mesh>,
    material: Handle<StandardMaterial>,
}

pub struct UnderwaterPlugin;
impl Plugin for UnderwaterPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(MaterialPlugin::<SandMaterial>::default())
            .init_resource::<SandCache>()
            .add_systems(PostUpdate, sync_sand.after(crate::scene::update_studio));
    }
}

#[allow(clippy::too_many_arguments, clippy::type_complexity)]
fn sync_sand(
    mut commands: Commands,
    appearance: Res<Appearance>,
    state: Res<GlobalState>,
    clock: Res<SimulationClock>,
    mut cache: ResMut<SandCache>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<SandMaterial>>,
    mut originals: ResMut<Assets<StandardMaterial>>,
    grounds: Query<(
        Entity,
        &StudioSurface,
        &Mesh3d,
        Option<&MeshMaterial3d<StandardMaterial>>,
        Option<&OriginalGround>,
    )>,
) {
    let active = appearance.environment == Environment::Underwater;
    for (entity, surface, mesh, material, original) in &grounds {
        if !matches!(surface, StudioSurface::Ground) {
            continue;
        }
        if active {
            let (sand_mesh, sand_material) = cache.0.get_or_insert_with(|| {
                (
                    meshes.add(Plane3d::default().mesh().size(2.0, 2.0)),
                    materials.add(ExtendedMaterial {
                        base: StandardMaterial {
                            base_color: Color::srgb(0.64, 0.57, 0.36),
                            perceptual_roughness: 0.96,
                            reflectance: 0.12,
                            ..default()
                        },
                        extension: SandCaustics { water: Vec4::ZERO },
                    }),
                )
            });
            let next = Vec4::new(
                clock.elapsed_secs(),
                vehicle::walls(state.vehicle_count).y,
                0.0,
                0.0,
            );
            if materials.get(sand_material.id()).unwrap().extension.water != next {
                materials
                    .get_mut(sand_material.id())
                    .unwrap()
                    .extension
                    .water = next;
            }
            if original.is_none() {
                if let Some(material) = material {
                    commands
                        .entity(entity)
                        .insert((
                            OriginalGround {
                                mesh: mesh.0.clone(),
                                material: material.0.clone(),
                            },
                            Mesh3d(sand_mesh.clone()),
                            MeshMaterial3d(sand_material.clone()),
                        ))
                        .remove::<MeshMaterial3d<StandardMaterial>>();
                }
            }
        } else if let Some(original) = original {
            // Style may have changed while the standard material was detached.
            if let Some(mut material) = originals.get_mut(&original.material) {
                material.base_color = if appearance.style == crate::scene::SceneStyle::Glow {
                    Color::srgb(0.25, 0.25, 0.32)
                } else {
                    Color::WHITE
                };
            }
            commands
                .entity(entity)
                .insert((
                    Mesh3d(original.mesh.clone()),
                    MeshMaterial3d(original.material.clone()),
                ))
                .remove::<(OriginalGround, MeshMaterial3d<SandMaterial>)>();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sand_reuses_ground_freezes_on_pause_and_restores_original_assets() {
        let mut app = crate::presentation::tests::clock_app();
        app.insert_resource(Appearance {
            environment: Environment::Underwater,
            ..default()
        })
        .init_resource::<GlobalState>()
        .init_resource::<SandCache>()
        .init_resource::<Assets<Mesh>>()
        .init_resource::<Assets<StandardMaterial>>()
        .init_resource::<Assets<SandMaterial>>()
        .add_systems(PostUpdate, (crate::scene::update_studio, sync_sand).chain());
        let mesh = app
            .world_mut()
            .resource_mut::<Assets<Mesh>>()
            .add(Plane3d::default());
        let material = app
            .world_mut()
            .resource_mut::<Assets<StandardMaterial>>()
            .add(StandardMaterial::default());
        app.world_mut()
            .spawn((Camera3d::default(), Transform::default()));
        let ground = app
            .world_mut()
            .spawn((
                StudioSurface::Ground,
                Transform::default(),
                Visibility::Inherited,
                Mesh3d(mesh.clone()),
                MeshMaterial3d(material.clone()),
            ))
            .id();
        app.update();
        let sand = app
            .world()
            .get::<MeshMaterial3d<SandMaterial>>(ground)
            .unwrap()
            .0
            .clone();
        assert!(!app
            .world()
            .entity(ground)
            .contains::<MeshMaterial3d<StandardMaterial>>());
        app.world_mut()
            .resource_mut::<Time<Real>>()
            .advance_by(std::time::Duration::from_millis(40));
        app.update();
        app.world_mut().resource_mut::<GlobalState>().vehicle_count = 10_000;
        app.update();
        let time = app
            .world()
            .resource::<Assets<SandMaterial>>()
            .get(&sand)
            .unwrap()
            .extension
            .water
            .x;
        assert!(time > 0.0);
        assert_eq!(
            app.world().get::<Transform>(ground).unwrap().translation.y,
            -1.6 * vehicle::walls(10_000).y
        );
        assert_eq!(
            *app.world().get::<Visibility>(ground).unwrap(),
            Visibility::Inherited
        );
        app.world_mut()
            .resource_mut::<crate::presentation::Presentation>()
            .paused = true;
        app.world_mut()
            .resource_mut::<Time<Real>>()
            .advance_by(std::time::Duration::from_secs(1));
        app.update();
        assert_eq!(
            app.world()
                .resource::<Assets<SandMaterial>>()
                .get(&sand)
                .unwrap()
                .extension
                .water
                .x,
            time
        );
        app.world_mut().resource_mut::<Appearance>().style = crate::scene::SceneStyle::Glow;
        app.world_mut().resource_mut::<Appearance>().environment = Environment::Off;
        app.update();
        assert_eq!(app.world().get::<Mesh3d>(ground).unwrap().0, mesh);
        assert_eq!(
            app.world().get::<Transform>(ground).unwrap().translation.y,
            -vehicle::walls(10_000).y
        );
        assert_eq!(
            app.world()
                .get::<MeshMaterial3d<StandardMaterial>>(ground)
                .unwrap()
                .0,
            material
        );
        assert!(!app
            .world()
            .entity(ground)
            .contains::<MeshMaterial3d<SandMaterial>>());
        assert_eq!(
            app.world()
                .resource::<Assets<StandardMaterial>>()
                .get(&material)
                .unwrap()
                .base_color,
            Color::srgb(0.25, 0.25, 0.32)
        );
        app.world_mut().resource_mut::<Appearance>().environment = Environment::Underwater;
        app.update();
        assert_eq!(
            app.world()
                .get::<MeshMaterial3d<SandMaterial>>(ground)
                .unwrap()
                .0,
            sand
        );
        assert_eq!(app.world().resource::<Assets<SandMaterial>>().len(), 1);
    }
}
