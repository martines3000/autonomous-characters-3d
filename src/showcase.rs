//! Ready-made scenes composed exclusively from existing live settings.
use bevy::{camera_controller::pan_orbit_camera::prelude::*, prelude::*};

use crate::{
    director::Shot,
    environment::Environment,
    formation::{FormationFrame, FormationMode},
    predator::{PredatorFrame, PredatorPath},
    presentation::Presentation,
    scene::{self, Appearance, HeadingPalette, SceneStyle},
    swarm::SwarmScene,
    target::{Target, TargetSelection},
    vehicle,
    vehicle_appearance::{ColorMode, VehicleShape},
    GlobalState,
};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Showcase {
    Starling,
    Fish,
    Swallow,
    Neon,
    Calm,
}

pub(crate) struct ShowcaseSettings {
    pub state: GlobalState,
    pub appearance: Appearance,
    pub presentation: Presentation,
    pub swarm: SwarmScene,
    camera_distance_fraction: f32,
}

impl Showcase {
    pub const ALL: [Self; 5] = [
        Self::Starling,
        Self::Fish,
        Self::Swallow,
        Self::Neon,
        Self::Calm,
    ];

    pub fn name(self) -> &'static str {
        match self {
            Self::Starling => "Starling murmuration",
            Self::Fish => "Fish school",
            Self::Swallow => "Swallow formation",
            Self::Neon => "Neon swarm",
            Self::Calm => "Calm",
        }
    }

    pub fn parse(name: &str) -> Result<Self, String> {
        match name.to_ascii_lowercase().as_str() {
            "starling" | "murmuration" | "starling-murmuration" => Ok(Self::Starling),
            "fish" | "fish-school" => Ok(Self::Fish),
            "swallow" | "swallow-formation" => Ok(Self::Swallow),
            "neon" | "neon-swarm" => Ok(Self::Neon),
            "calm" => Ok(Self::Calm),
            _ => Err(format!(
                "invalid showcase '{name}'; expected starling, fish, swallow, neon or calm"
            )),
        }
    }

    pub fn settings(self) -> ShowcaseSettings {
        let mut settings = ShowcaseSettings {
            state: default(),
            appearance: default(),
            presentation: default(),
            swarm: default(),
            camera_distance_fraction: 1.0,
        };
        if self == Self::Calm {
            return settings;
        }
        let ShowcaseSettings {
            state,
            appearance,
            presentation,
            swarm,
            ..
        } = &mut settings;
        // Explicit showcase composition must survive the count-framing debounce.
        appearance.auto_frame = false;
        match self {
            Self::Starling => {
                state.vehicle_count = 40_000;
                state.vehicle_mass = 50.0;
                state.vehicle_max_speed = 90.0;
                state.vehicle_wander_speed = 60.0;
                state.vehicle_alignment_distance = 100.0;
                state.vehicle_cohesion_distance = 60.0;
                state.vehicle_cohesion_factor = 0.15;
                state.vehicle_separation_distance = 6.0;
                state.vehicle_wander_factor = 0.1;
                state.vehicle_wall_avoid_factor = 3.0;
                state.motion.bank = 0.9;
                state.motion.flap = 0.6;
                appearance.vehicles.shape = VehicleShape::Bird;
                appearance.vehicles.visual_size = 1.6;
                appearance.palette = HeadingPalette::Dusk;
                appearance.environment = Environment::Sunset;
                swarm.predators.add(Vec3::ZERO, 70.0);
                swarm.predators.add(Vec3::ZERO, 70.0);
                swarm.predators.path = PredatorPath::Orbit;
                swarm.predators.path_radius = 40.0;
                swarm.predators.path_speed = 32.0;
                swarm.predators.strength = 5.0;
                swarm.predators.visible = false;
                presentation.set_director(true);
                presentation.director.shot = Shot::Orbit;
                presentation.director.shot_length = 10.0;
            }
            Self::Fish => {
                state.vehicle_mass = 45.0;
                state.vehicle_max_speed = 90.0;
                state.vehicle_alignment_distance = 80.0;
                state.vehicle_cohesion_distance = 65.0;
                state.vehicle_cohesion_factor = 0.2;
                state.vehicle_separation_distance = 5.0;
                state.groups.count = 2;
                state.groups.table[0].color = [35, 195, 220];
                state.groups.table[1].color = [105, 235, 185];
                state.motion.bank = 0.8;
                state.motion.flap = 0.8;
                appearance.vehicles.shape = VehicleShape::Fish;
                appearance.vehicles.visual_size = 1.8;
                appearance.vehicles.color_mode = ColorMode::Group;
                appearance.palette = HeadingPalette::Ocean;
                appearance.environment = Environment::Underwater;
                // The sand floor is visible underwater, so the school's shadows read on it.
                appearance.shadows = true;
                swarm.predators.add(Vec3::ZERO, 85.0);
                swarm.predators.path = PredatorPath::Orbit;
                swarm.predators.path_radius = 35.0;
                swarm.predators.path_speed = 25.0;
                swarm.predators.strength = 5.0;
                swarm.predators.visible = false;
                presentation.set_director(true);
                presentation.director.shot_length = 10.0;
            }
            Self::Swallow => {
                state.vehicle_count = 8_000;
                state.vehicle_mass = 8.0;
                state.vehicle_max_speed = 100.0;
                state.vehicle_separation_distance = 2.5;
                state.vehicle_alignment_distance = 12.0;
                state.vehicle_cohesion_distance = 8.0;
                appearance.vehicles.shape = VehicleShape::Bird;
                appearance.vehicles.visual_size = 1.8;
                appearance.palette = HeadingPalette::Dusk;
                appearance.environment = Environment::Midday;
                swarm.formation.mode = FormationMode::Sphere;
                swarm.formation.morph = 1.0;
                swarm.formation.morph_speed = 0.08;
                presentation.orbit = true;
                presentation.orbit_speed = 8.0;
                settings.camera_distance_fraction = 0.32;
            }
            Self::Neon => {
                state.vehicle_count = 200_000;
                state.vehicle_cohesion_factor = 0.1;
                state.vehicle_alignment_distance = 60.0;
                appearance.style = SceneStyle::Glow;
                appearance.glow = 5.0;
                appearance.streak = 2.0;
                presentation.orbit = true;
                presentation.orbit_speed = 5.0;
                settings.camera_distance_fraction = 0.42;
            }
            Self::Calm => unreachable!(),
        }
        settings
    }
}

/// Both menu buttons and validated scenarios enter here. Vehicle lifecycle stays with
/// the existing count/backend systems; selecting a showcase does not reset retained boids.
pub(crate) fn apply(world: &mut World, showcase: Showcase) {
    let mut settings = showcase.settings();
    // Invalidate old asynchronous camera samples even when switching between Director scenes.
    settings
        .presentation
        .follow
        .restart_after(&world.resource::<Presentation>().follow);
    if let Some(mut models) = world.get_resource_mut::<crate::vehicle_model::VehicleModels>() {
        models.cancel();
    }
    for mut selection in world
        .query_filtered::<&mut TargetSelection, With<Target>>()
        .iter_mut(world)
    {
        selection.0 = false;
    }
    // Enabling normally snaps straight to the requested morph. Prime the existing runtime
    // at Sphere so this scene travels toward Helix, including after another formation.
    let mut formation = FormationFrame::default();
    if showcase == Showcase::Swallow {
        formation.settings = settings.swarm.formation;
        formation.settings.morph = 0.0;
    }
    world.insert_resource(formation);
    world.insert_resource(PredatorFrame::default());
    let rotation = Transform::from_xyz(0.0, 140.0, 450.0)
        .looking_at(Vec3::ZERO, Vec3::Y)
        .rotation;
    for (mut transform, mut orbit, projection) in world
        .query_filtered::<(&mut Transform, &mut PanOrbitCamera, &Projection), With<Camera3d>>()
        .iter_mut(world)
    {
        let Projection::Perspective(projection) = projection else {
            continue;
        };
        let distance = scene::frame_distance(
            vehicle::walls(settings.state.vehicle_count),
            rotation,
            Vec3::ZERO,
            projection.aspect_ratio,
            PerspectiveProjection::default().fov,
        ) * settings.camera_distance_fraction;
        *transform =
            Transform::from_translation(rotation * Vec3::Z * distance).with_rotation(rotation);
        orbit.current_motion = default();
        orbit.last_anchor_depth = -(distance as f64);
    }
    world.insert_resource(settings.state);
    world.insert_resource(settings.appearance);
    world.insert_resource(settings.presentation);
    world.insert_resource(settings.swarm);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        follow::FollowMode, presentation::SimulationFrame, swarm::AttractorMode, SimulationMode,
    };
    use std::time::Duration;

    fn world() -> World {
        let mut world = World::new();
        world.insert_resource(GlobalState::default());
        world.insert_resource(Appearance::default());
        world.insert_resource(Presentation::default());
        world.insert_resource(SwarmScene::default());
        world
    }

    fn assert_settings(world: &World, expected: &ShowcaseSettings) {
        assert_eq!(*world.resource::<GlobalState>(), expected.state);
        assert_eq!(*world.resource::<SwarmScene>(), expected.swarm);
        let actual = world.resource::<Appearance>();
        let wanted = &expected.appearance;
        assert_eq!(actual.style, wanted.style);
        assert_eq!(actual.palette, wanted.palette);
        assert_eq!(actual.vehicles, wanted.vehicles);
        assert_eq!(actual.auto_frame, wanted.auto_frame);
        assert_eq!(actual.frame_requested, wanted.frame_requested);
        assert_eq!(actual.count_editing, wanted.count_editing);
        assert_eq!(actual.glow, wanted.glow);
        assert_eq!(actual.streak, wanted.streak);
        let actual = world.resource::<Presentation>();
        let wanted = &expected.presentation;
        assert_eq!(actual.paused, wanted.paused);
        assert_eq!(actual.speed, wanted.speed);
        assert_eq!(actual.orbit, wanted.orbit);
        assert_eq!(actual.orbit_speed, wanted.orbit_speed);
        assert_eq!(actual.preset, wanted.preset);
        assert_eq!(actual.ui_visible, wanted.ui_visible);
        assert_eq!(actual.camera_delta, wanted.camera_delta);
        assert_eq!(actual.follow.mode, wanted.follow.mode);
        assert_eq!(actual.follow.id, wanted.follow.id);
        assert_eq!(actual.follow.distance, wanted.follow.distance);
        assert_eq!(actual.follow.dof, wanted.follow.dof);
        assert_eq!(actual.follow.aperture, wanted.follow.aperture);
        assert_eq!(actual.director.enabled, wanted.director.enabled);
        assert_eq!(actual.director.shot_length, wanted.director.shot_length);
        assert_eq!(actual.director.shake, wanted.director.shake);
        assert_eq!(actual.director.speed_ramp, wanted.director.speed_ramp);
        assert_eq!(actual.director.shot, wanted.director.shot);
        assert_eq!(actual.director.elapsed, wanted.director.elapsed);
        // Readback generation is runtime identity, intentionally renewed on every application.
    }

    #[test]
    fn names_and_aliases_are_validated() {
        for (names, expected) in [
            (
                &["starling", "murmuration", "starling-murmuration"][..],
                Showcase::Starling,
            ),
            (&["fish", "fish-school"][..], Showcase::Fish),
            (&["swallow", "swallow-formation"][..], Showcase::Swallow),
            (&["neon", "neon-swarm"][..], Showcase::Neon),
            (&["calm", "CALM"][..], Showcase::Calm),
        ] {
            for name in names {
                assert_eq!(Showcase::parse(name).unwrap(), expected);
            }
        }
        for name in ["", "studio", "overview", "starlingg", "fish school"] {
            assert!(Showcase::parse(name).is_err(), "{name}");
        }
    }

    #[test]
    fn scenes_apply_their_key_settings() {
        let mut world = world();
        for preset in Showcase::ALL {
            apply(&mut world, preset);
            let state = world.resource::<GlobalState>();
            let appearance = world.resource::<Appearance>();
            let presentation = world.resource::<Presentation>();
            let swarm = world.resource::<SwarmScene>();
            assert_eq!(state.mode, SimulationMode::Gpu);
            assert_eq!(state.attractor.mode, AttractorMode::Off);
            assert_eq!(state.attractor.scripted_point, None);
            match preset {
                Showcase::Starling => {
                    assert_eq!(appearance.environment, Environment::Sunset);
                    assert_eq!(state.vehicle_count, 40_000);
                    assert_eq!(appearance.vehicles.shape, VehicleShape::Bird);
                    assert_eq!(appearance.palette, HeadingPalette::Dusk);
                    assert_eq!(state.groups.count, 0);
                    assert_eq!(swarm.predators.objects.len(), 2);
                    assert_eq!(swarm.predators.path, PredatorPath::Orbit);
                    assert!(swarm.predators.objects.iter().all(|p| p.mask == 255));
                    assert!(state.motion.bank > 0.0 && state.motion.flap > 0.0);
                    assert!(
                        state.vehicle_cohesion_distance
                            > GlobalState::default().vehicle_cohesion_distance
                    );
                    assert!(presentation.director.enabled);
                    assert_eq!(presentation.director.shot, Shot::Orbit);
                }
                Showcase::Fish => {
                    assert_eq!(appearance.environment, Environment::Underwater);
                    assert!(appearance.shadows);
                    assert_eq!(state.vehicle_count, 20_000);
                    assert_eq!(appearance.vehicles.shape, VehicleShape::Fish);
                    assert_eq!(appearance.palette, HeadingPalette::Ocean);
                    assert_eq!(state.groups.count, 2);
                    assert!(state.groups.table[..2].iter().all(|g| g.target.is_none()));
                    assert!(
                        state.vehicle_cohesion_factor
                            > Showcase::Starling.settings().state.vehicle_cohesion_factor
                    );
                    assert_eq!(swarm.predators.objects.len(), 1);
                    assert_eq!(swarm.predators.path, PredatorPath::Orbit);
                    assert_eq!(swarm.predators.objects[0].mask & 3, 3);
                    assert!(
                        state.vehicle_cohesion_distance
                            > GlobalState::default().vehicle_cohesion_distance
                    );
                    assert!(presentation.director.enabled);
                }
                Showcase::Swallow => {
                    assert_eq!(appearance.environment, Environment::Midday);
                    assert_eq!(appearance.vehicles.shape, VehicleShape::Bird);
                    assert_eq!(appearance.style, SceneStyle::Studio);
                    assert_eq!(swarm.formation.mode, FormationMode::Sphere);
                    assert_eq!(swarm.formation.morph, 1.0);
                    assert!(swarm.formation.morph_speed > 0.0);
                    assert_eq!(world.resource::<FormationFrame>().morph, 0.0);
                    assert!(world.resource::<FormationFrame>().settings.enabled());
                }
                Showcase::Neon => {
                    assert_eq!(appearance.environment, Environment::Off);
                    assert_eq!(state.vehicle_count, 200_000);
                    assert_eq!(appearance.vehicles.shape, VehicleShape::Dart);
                    assert_eq!(appearance.style, SceneStyle::Glow);
                    assert!(appearance.glow > 0.0 && appearance.streak > 0.0);
                    assert!(!presentation.director.enabled);
                }
                Showcase::Calm => assert_eq!(*state, GlobalState::default()),
            }
        }
    }

    #[test]
    fn calm_restores_exact_default_settings_after_dirty_state() {
        let mut world = world();
        apply(&mut world, Showcase::Fish);
        world.resource_mut::<GlobalState>().attractor.scripted_point = Some(Vec3::ONE);
        world.resource_mut::<GlobalState>().attractor.mode = AttractorMode::Attract;
        world
            .resource_mut::<SwarmScene>()
            .add_obstacle(Vec3::ONE, 10.0);
        world.resource_mut::<Presentation>().paused = true;
        world.resource_mut::<Presentation>().ui_visible = false;
        world.resource_mut::<Presentation>().follow.mode = FollowMode::FirstPerson;
        world.resource_mut::<Presentation>().speed = 4.0;
        world.resource_mut::<Appearance>().vehicles.shape = VehicleShape::Custom;
        let target = world.spawn((Target, TargetSelection(true))).id();
        apply(&mut world, Showcase::Calm);
        let defaults = ShowcaseSettings {
            state: GlobalState::default(),
            appearance: Appearance::default(),
            presentation: Presentation::default(),
            swarm: SwarmScene::default(),
            camera_distance_fraction: 1.0,
        };
        assert_settings(&world, &defaults);
        assert!(!world.get::<TargetSelection>(target).unwrap().0);
        assert_eq!(
            *world.resource::<FormationFrame>(),
            FormationFrame::default()
        );
        assert_eq!(*world.resource::<PredatorFrame>(), PredatorFrame::default());
    }

    #[test]
    fn all_preset_orders_are_independent_and_idempotent() {
        let mut world = world();
        for first in Showcase::ALL {
            for second in Showcase::ALL {
                for final_preset in Showcase::ALL {
                    apply(&mut world, first);
                    apply(&mut world, second);
                    apply(&mut world, final_preset);
                    assert_settings(&world, &final_preset.settings());
                    let formation = *world.resource::<FormationFrame>();
                    let predators = *world.resource::<PredatorFrame>();
                    apply(&mut world, final_preset);
                    assert_settings(&world, &final_preset.settings());
                    assert_eq!(*world.resource::<FormationFrame>(), formation);
                    assert_eq!(*world.resource::<PredatorFrame>(), predators);
                }
            }
        }
    }

    #[test]
    fn swallow_starts_at_sphere_and_advances_toward_helix() {
        let mut app = App::new();
        app.insert_resource(Presentation::default());
        apply(app.world_mut(), Showcase::Swallow);
        app.insert_resource(SimulationFrame {
            steps: 1,
            step: Duration::from_secs(1),
        });
        app.add_systems(Update, crate::formation::prepare_frame);
        app.update();
        let frame = app.world().resource::<FormationFrame>();
        assert!((frame.morph - 0.08).abs() < 1e-6);
        assert!(frame.morph < frame.settings.morph);
    }
}
