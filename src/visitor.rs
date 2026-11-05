//! Curated visitor scenes. Detailed experiments keep their existing Lab contracts.
use bevy::{camera_controller::pan_orbit_camera::prelude::*, prelude::*};

use crate::{
    environment::Environment,
    presentation::Presentation,
    scene::{self, Appearance, HeadingPalette},
    showcase::{self, Showcase},
    species::{self, Species},
    swarm::SwarmScene,
    vehicle,
    vehicle_appearance::ColorMode,
    GlobalState,
};

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(crate) enum VisitorScene {
    #[default]
    Murmuration,
    ReefDrift,
    SharkHunt,
}

impl VisitorScene {
    pub const ALL: [Self; 3] = [Self::Murmuration, Self::ReefDrift, Self::SharkHunt];

    pub fn name(self) -> &'static str {
        match self {
            Self::Murmuration => "Sunset murmuration",
            Self::ReefDrift => "Reef drift",
            Self::SharkHunt => "Shark hunt",
        }
    }

    pub fn population(self, tier: PopulationTier) -> usize {
        let counts = match self {
            Self::Murmuration => [8_000, 20_000, 40_000],
            Self::ReefDrift => [500, 1_500, 3_000],
            Self::SharkHunt => [5_000, 12_000, 20_000],
        };
        counts[tier as usize]
    }

    fn species(self) -> Species {
        match self {
            Self::Murmuration => Species::Murmuration,
            Self::ReefDrift => Species::ReefDrift,
            Self::SharkHunt => Species::SharkAttack,
        }
    }

    fn environment(self) -> Environment {
        match self {
            Self::Murmuration => Environment::Sunset,
            Self::ReefDrift | Self::SharkHunt => Environment::Underwater,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(crate) enum PopulationTier {
    Low,
    #[default]
    Medium,
    High,
}

impl PopulationTier {
    pub const ALL: [Self; 3] = [Self::Low, Self::Medium, Self::High];

    pub fn name(self) -> &'static str {
        match self {
            Self::Low => "Low",
            Self::Medium => "Medium",
            Self::High => "High",
        }
    }
}

#[derive(Resource, Default)]
pub(crate) struct Visitor {
    pub scene: VisitorScene,
    pub tier: PopulationTier,
    pub reduced_motion: bool,
}

pub(crate) struct VisitorPlugin;
impl Plugin for VisitorPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<Visitor>()
            .add_systems(PostStartup, startup);
    }
}

fn startup(world: &mut World) {
    // Log the exact public tier contract; these counts are scene-specific, not quality settings.
    for scene in VisitorScene::ALL {
        for tier in PopulationTier::ALL {
            debug!("{} / {}: {} animals", scene.name(), tier.name(), scene.population(tier));
        }
    }
    apply(world, VisitorScene::default(), PopulationTier::default());
}

/// Scene selection restores the complete scene while retaining surviving vehicle identities.
/// It does not reseed the simulation. Pause, UI visibility and reduced camera motion survive.
/// Model cancellation renews its generation so a previous scene cannot complete over this one.
pub(crate) fn apply(world: &mut World, scene: VisitorScene, tier: PopulationTier) {
    let (paused, ui_visible) = {
        let presentation = world.resource::<Presentation>();
        (presentation.paused, presentation.ui_visible)
    };
    let reduced_motion = world
        .get_resource::<Visitor>()
        .is_some_and(|visitor| visitor.reduced_motion);
    showcase::apply(world, Showcase::Calm);
    if let Some(mut models) = world.get_resource_mut::<crate::vehicle_model::VehicleModels>() {
        models.clip = "0".into();
        models.animation_speed = 1.0;
    }
    species::apply(world, scene.species());
    world.resource_mut::<GlobalState>().vehicle_count = scene.population(tier);
    // Public scenes never use an invisible threat or an artistic formation.
    world.resource_mut::<SwarmScene>().predators = default();
    {
        let mut appearance = world.resource_mut::<Appearance>();
        appearance.environment = scene.environment();
        appearance.palette = if scene == VisitorScene::Murmuration {
            HeadingPalette::Dusk
        } else {
            HeadingPalette::Ocean
        };
        appearance.vehicles.color_mode = ColorMode::ModelColors;
        appearance.auto_frame = false;
        appearance.shadows = scene != VisitorScene::Murmuration;
    }
    {
        let mut presentation = world.resource_mut::<Presentation>();
        presentation.paused = paused;
        presentation.ui_visible = ui_visible;
        presentation.orbit = !reduced_motion;
        presentation.orbit_speed = 3.0;
    }
    world.insert_resource(Visitor {
        scene,
        tier,
        reduced_motion,
    });
    reset_view(world);
    info!("Visitor scene: {} / {}", scene.name(), tier.name());
}

/// Restore the composed wide shot without changing animals, population, pause or simulation time.
pub(crate) fn reset_view(world: &mut World) {
    let state = world.resource::<GlobalState>();
    let count = state.vehicle_count;
    let scene = world.resource::<Visitor>().scene;
    let (yaw, pitch, fraction): (f32, f32, f32) = match scene {
        VisitorScene::Murmuration => (18.0, 8.0, 0.48),
        VisitorScene::ReefDrift => (15.0, 9.0, 0.32),
        VisitorScene::SharkHunt => (20.0, 12.0, 0.38),
    };
    let rotation = Quat::from_euler(EulerRot::YXZ, yaw.to_radians(), -pitch.to_radians(), 0.0);
    for (mut transform, mut orbit, projection) in world
        .query_filtered::<(&mut Transform, &mut PanOrbitCamera, &Projection), With<Camera3d>>()
        .iter_mut(world)
    {
        let Projection::Perspective(projection) = projection else {
            continue;
        };
        let distance = scene::frame_distance(
            vehicle::walls(count),
            rotation,
            Vec3::ZERO,
            projection.aspect_ratio,
            PerspectiveProjection::default().fov,
        ) * fraction;
        *transform = Transform::from_translation(rotation * Vec3::Z * distance).with_rotation(rotation);
        orbit.current_motion = default();
        orbit.last_anchor_depth = -(distance as f64);
    }
    world.resource_mut::<Presentation>().cancel_camera_transition();
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{formation::FormationMode, predator::PredatorPath, swarm::GroupRole, SimulationMode};

    fn world() -> World {
        let mut world = World::new();
        world.insert_resource(GlobalState::default());
        world.insert_resource(Appearance::default());
        world.insert_resource(Presentation::default());
        world.insert_resource(SwarmScene::default());
        world.insert_resource(Visitor::default());
        world
    }

    fn assert_scene(world: &World, scene: VisitorScene, tier: PopulationTier) {
        let state = world.resource::<GlobalState>();
        let appearance = world.resource::<Appearance>();
        let swarm = world.resource::<SwarmScene>();
        let presentation = world.resource::<Presentation>();
        assert_eq!(state.mode, SimulationMode::Gpu);
        assert_eq!(state.vehicle_count, scene.population(tier));
        assert_eq!(appearance.environment, scene.environment());
        assert_eq!(appearance.vehicles.color_mode, ColorMode::ModelColors);
        assert!(!appearance.auto_frame);
        assert!(!appearance.volumetric);
        assert!(!presentation.motion_blur);
        assert!(!presentation.director.enabled);
        assert!(!presentation.follow.enabled());
        assert!(swarm.predators.objects.is_empty());
        assert_eq!(swarm.formation.mode, FormationMode::Off);
        assert!(swarm.obstacles.is_empty());
        if scene == VisitorScene::SharkHunt {
            assert_eq!(state.groups.count, 2);
            assert_eq!(state.groups.table[1].role, GroupRole::Hunter);
            assert_eq!(state.groups.hunter_count(), 3);
        } else {
            assert_eq!(state.groups.hunter_count(), 0);
            assert!(!state.groups.has_targets());
        }
        assert_eq!(world.resource::<Visitor>().scene, scene);
        assert_eq!(world.resource::<Visitor>().tier, tier);
    }

    #[test]
    fn public_scenes_replace_every_incompatible_lab_family() {
        for scene in VisitorScene::ALL {
            for lab in Showcase::ALL {
                let mut world = world();
                showcase::apply(&mut world, lab);
                world.resource_mut::<GlobalState>().mode = SimulationMode::Octree;
                world.resource_mut::<SwarmScene>().predators.path = PredatorPath::Orbit;
                apply(&mut world, scene, PopulationTier::Medium);
                assert_scene(&world, scene, PopulationTier::Medium);
            }
            for lab in Species::FISH.into_iter().chain(Species::BIRDS).chain(Species::HUNTS) {
                let mut world = world();
                species::apply(&mut world, lab);
                apply(&mut world, scene, PopulationTier::Medium);
                assert_scene(&world, scene, PopulationTier::Medium);
            }
        }
    }

    #[test]
    fn switching_and_repeated_selection_restore_one_scene_contract() {
        let mut world = world();
        for _ in 0..4 {
            for from in VisitorScene::ALL {
                for to in VisitorScene::ALL {
                    for tier in PopulationTier::ALL {
                        apply(&mut world, from, PopulationTier::High);
                        apply(&mut world, to, tier);
                        assert_scene(&world, to, tier);
                        let before = world.resource::<GlobalState>().clone();
                        apply(&mut world, to, tier);
                        assert_eq!(*world.resource::<GlobalState>(), before);
                    }
                }
            }
        }
    }

    #[test]
    fn scene_switch_preserves_viewer_preferences_and_applied_environment() {
        let mut world = world();
        world.resource_mut::<Appearance>().applied_environment = Some(Environment::Sunset);
        world.resource_mut::<Visitor>().reduced_motion = true;
        world.resource_mut::<Presentation>().paused = true;
        world.resource_mut::<Presentation>().ui_visible = false;
        apply(&mut world, VisitorScene::SharkHunt, PopulationTier::Low);
        assert_scene(&world, VisitorScene::SharkHunt, PopulationTier::Low);
        let presentation = world.resource::<Presentation>();
        assert!(presentation.paused);
        assert!(!presentation.ui_visible);
        assert!(!presentation.orbit);
        assert!(world.resource::<Visitor>().reduced_motion);
        let appearance = world.resource::<Appearance>();
        assert_eq!(appearance.active_environment(), Environment::Sunset);
        assert!(!appearance.environment_ready());
    }

    #[test]
    fn reset_view_preserves_simulation_and_stops_subject_camera() {
        let mut world = world();
        apply(&mut world, VisitorScene::ReefDrift, PopulationTier::Medium);
        let before = world.resource::<GlobalState>().clone();
        world.resource_mut::<Presentation>().set_director(true);
        world.resource_mut::<Presentation>().paused = true;
        reset_view(&mut world);
        assert_eq!(*world.resource::<GlobalState>(), before);
        let presentation = world.resource::<Presentation>();
        assert!(presentation.paused);
        assert!(!presentation.director.enabled);
        assert!(!presentation.follow.enabled());
    }
}
