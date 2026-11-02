//! Environment-sized participating media; shadow policy stays derived from Appearance.
use bevy::{
    asset::RenderAssetUsages,
    image::ImageSampler,
    light::{FogVolume, Skybox, VolumetricFog, VolumetricLight},
    prelude::*,
    render::render_resource::{Extent3d, TextureDimension, TextureFormat},
    transform::TransformSystems,
};

use crate::{
    environment::Environment,
    scene::{Appearance, FillLight, KeyLight},
    vehicle, GlobalState,
};

pub struct VolumetricPlugin;
impl Plugin for VolumetricPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<WaterParticles>().add_systems(
            PostUpdate,
            sync_volumetric
                .before(bevy::camera::CameraUpdateSystems)
                .before(TransformSystems::Propagate),
        );
    }
}

#[derive(Component)]
pub(crate) struct EnvironmentFog;

#[derive(Component)]
pub(crate) struct PreviousFillVisibility(Visibility);

#[derive(Resource, Default)]
pub(crate) struct WaterParticles(Option<Handle<Image>>);

/// Suspended particles form broad columns along the refracted sun direction.
/// Real shadow-map occlusion from the fish modulates the light inside these columns.
fn water_particles() -> Image {
    const SIDE: u32 = 32;
    let mut image = Image::new_uninit(
        Extent3d {
            width: SIDE,
            height: SIDE,
            depth_or_array_layers: SIDE,
        },
        TextureDimension::D3,
        TextureFormat::R8Unorm,
        RenderAssetUsages::RENDER_WORLD,
    );
    let mut data = Vec::with_capacity((SIDE * SIDE * SIDE) as usize);
    for z in 0..SIDE {
        for y in 0..SIDE {
            for x in 0..SIDE {
                let p = (Vec3::new(x as f32, y as f32, z as f32) + 0.5) / SIDE as f32 - 0.5;
                // World half-extents are 2:2:1; project onto the water surface along the sun.
                let u = p.x + p.y * (0.18 / 0.97);
                let v = p.z + p.y * (0.32 / 0.97);
                let bands =
                    (u * 39.0 + (v * 15.0).sin()).sin() * (v * 31.0 + (u * 17.0).cos()).sin();
                let density = 0.04 + 0.96 * (0.5 + 0.5 * bands).powf(3.0);
                data.push((density * 255.0).round() as u8);
            }
        }
    }
    image.data = Some(data);
    image.sampler = ImageSampler::linear();
    image
}

fn medium(environment: Environment, count: usize) -> FogVolume {
    let (density, tint, intensity, asymmetry) = match environment {
        Environment::Sunset => (0.0015, Color::srgb(1.0, 0.75, 0.48), 1.0, 0.65),
        Environment::Midday => (0.0004, Color::srgb(0.9, 0.96, 1.0), 0.15, 0.55),
        Environment::Underwater => (0.006, Color::srgb(0.2, 0.9, 1.0), 2.5, 0.35),
        Environment::Off => unreachable!("Off never creates a medium"),
    };
    FogVolume {
        density_factor: density / vehicle::world_scale(count),
        fog_color: tint,
        light_intensity: intensity,
        scattering_asymmetry: asymmetry,
        ..default()
    }
}

#[allow(clippy::type_complexity, clippy::too_many_arguments)]
pub(crate) fn sync_volumetric(
    mut commands: Commands,
    appearance: Res<Appearance>,
    state: Res<GlobalState>,
    mut images: ResMut<Assets<Image>>,
    mut particles: ResMut<WaterParticles>,
    cameras: Query<(Entity, Has<Skybox>, Option<&VolumetricFog>), With<Camera3d>>,
    suns: Query<(Entity, Has<VolumetricLight>), With<KeyLight>>,
    volumes: Query<Entity, With<EnvironmentFog>>,
    fills: Query<(Entity, &Visibility, Option<&PreviousFillVisibility>), With<FillLight>>,
) {
    // Do not scatter into a missing/failed environment while its HDRI loads.
    let active = appearance.volumetric_active() && cameras.iter().any(|(_, sky, _)| sky);
    // Bevy 0.20.1 sorts non-volumetric directional lights first, despite expecting
    // volumetric lights first when setting GPU flags. The environment already sets
    // the Studio fill to zero lux; hiding it avoids that extra extracted light.
    for (entity, visibility, previous) in &fills {
        if active && previous.is_none() {
            commands
                .entity(entity)
                .insert((PreviousFillVisibility(*visibility), Visibility::Hidden));
        } else if !active {
            if let Some(previous) = previous {
                commands
                    .entity(entity)
                    .insert(previous.0)
                    .remove::<PreviousFillVisibility>();
            }
        }
    }
    for (entity, sky, fog) in &cameras {
        if active && sky {
            if fog.is_none_or(|fog| fog.step_count != appearance.volumetric_steps) {
                commands.entity(entity).insert(VolumetricFog {
                    step_count: appearance.volumetric_steps,
                    ambient_intensity: 0.0,
                    // No TAA: temporal jitter would shimmer, especially on paused captures.
                    jitter: 0.0,
                    ..default()
                });
            }
        } else if fog.is_some() {
            commands.entity(entity).remove::<VolumetricFog>();
        }
    }
    for (entity, volumetric) in &suns {
        if active && !volumetric {
            commands.entity(entity).insert(VolumetricLight);
        } else if !active && volumetric {
            commands.entity(entity).remove::<VolumetricLight>();
        }
    }
    if !active {
        for entity in &volumes {
            commands.entity(entity).despawn();
        }
    } else if volumes.is_empty() || appearance.is_changed() || state.is_changed() {
        // A FogVolume is a unit cube, scaled to contain the entire simulation world.
        let transform = Transform::from_scale(vehicle::walls(state.vehicle_count) * 2.2);
        let mut fog = medium(appearance.environment, state.vehicle_count);
        if appearance.environment == Environment::Underwater {
            fog.density_texture = Some(
                particles
                    .0
                    .get_or_insert_with(|| images.add(water_particles()))
                    .clone(),
            );
        }
        if volumes.is_empty() {
            commands.spawn((EnvironmentFog, fog, transform));
        } else {
            for entity in &volumes {
                commands.entity(entity).insert((fog.clone(), transform));
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_has_no_volumetric_components_and_environment_off_stays_inert() {
        let mut app = App::new();
        app.init_resource::<Appearance>()
            .init_resource::<GlobalState>()
            .init_resource::<Assets<Image>>()
            .init_resource::<WaterParticles>()
            .add_systems(Update, sync_volumetric);
        let camera = app.world_mut().spawn(Camera3d::default()).id();
        let sun = app.world_mut().spawn(KeyLight).id();
        for requested in [false, true, false] {
            app.world_mut().resource_mut::<Appearance>().volumetric = requested;
            app.update();
            assert!(!app.world().entity(camera).contains::<VolumetricFog>());
            assert!(!app.world().entity(sun).contains::<VolumetricLight>());
            assert_eq!(app.world().resource::<Assets<Image>>().len(), 0);
            assert_eq!(
                app.world_mut()
                    .query::<&FogVolume>()
                    .iter(app.world())
                    .count(),
                0
            );
        }
    }

    #[test]
    fn media_and_steps_follow_environment_size_and_quality_without_duplicates() {
        let mut app = App::new();
        app.init_resource::<Appearance>()
            .init_resource::<GlobalState>()
            .init_resource::<Assets<Image>>()
            .init_resource::<WaterParticles>()
            .add_systems(Update, sync_volumetric);
        let camera = app
            .world_mut()
            .spawn((Camera3d::default(), Skybox::default()))
            .id();
        let sun = app.world_mut().spawn(KeyLight).id();
        let fill = app
            .world_mut()
            .spawn((
                FillLight,
                DirectionalLight::default(),
                Visibility::Inherited,
            ))
            .id();
        for environment in [
            Environment::Sunset,
            Environment::Midday,
            Environment::Underwater,
        ] {
            for steps in [16, 64, 128] {
                {
                    let mut appearance = app.world_mut().resource_mut::<Appearance>();
                    appearance.environment = environment;
                    appearance.volumetric = true;
                    appearance.volumetric_steps = steps;
                }
                app.world_mut().resource_mut::<GlobalState>().vehicle_count = 100_000;
                app.update();
                app.update();
                assert_eq!(
                    app.world().get::<VolumetricFog>(camera).unwrap().step_count,
                    steps
                );
                assert!(app.world().entity(sun).contains::<VolumetricLight>());
                assert!(!app.world().entity(fill).contains::<VolumetricLight>());
                assert_eq!(
                    app.world().get::<Visibility>(fill),
                    Some(&Visibility::Hidden)
                );
                let mut query = app
                    .world_mut()
                    .query_filtered::<(&FogVolume, &Transform), With<EnvironmentFog>>();
                let (fog, transform) = query.single(app.world()).unwrap();
                assert_eq!(
                    fog.density_factor,
                    medium(environment, 100_000).density_factor
                );
                assert_eq!(transform.scale, vehicle::walls(100_000) * 2.2);
                assert_eq!(
                    fog.density_texture.is_some(),
                    environment == Environment::Underwater
                );
                assert_eq!(
                    app.world().resource::<Assets<Image>>().len(),
                    usize::from(environment == Environment::Underwater)
                );
            }
        }
        for environment_off in [false, true] {
            // Exercise both disabling the toggle and leaving an active environment.
            {
                let mut appearance = app.world_mut().resource_mut::<Appearance>();
                appearance.environment = Environment::Underwater;
                appearance.volumetric = true;
            }
            app.update();
            {
                let mut appearance = app.world_mut().resource_mut::<Appearance>();
                appearance.volumetric = environment_off;
                if environment_off {
                    appearance.environment = Environment::Off;
                }
            }
            app.update();
            assert!(!app.world().entity(camera).contains::<VolumetricFog>());
            assert!(!app.world().entity(sun).contains::<VolumetricLight>());
            assert_eq!(
                app.world().get::<Visibility>(fill),
                Some(&Visibility::Inherited)
            );
            assert!(!app
                .world()
                .entity(fill)
                .contains::<PreviousFillVisibility>());
            assert_eq!(
                app.world_mut()
                    .query::<&FogVolume>()
                    .iter(app.world())
                    .count(),
                0
            );
        }
    }
}
