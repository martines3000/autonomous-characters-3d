//! Bounded, simulation-clock guides for the curated public fish scenes.
//! Local schooling still uses the shared CPU/GPU group steering and separation kernels.
use std::{f32::consts::TAU, time::Duration};

use bevy::{camera_controller::pan_orbit_camera::prelude::PanOrbitCamera, prelude::*};

use crate::{
    presentation::SimulationClock,
    swarm::{Groups, SchoolGuide},
    visitor::VisitorScene,
    GlobalState,
};

#[derive(Resource, Clone, Debug)]
pub(crate) struct SchoolMotion {
    scene: VisitorScene,
    elapsed: Duration,
}

impl Default for SchoolMotion {
    fn default() -> Self {
        Self {
            scene: VisitorScene::ReefDrift,
            elapsed: Duration::ZERO,
        }
    }
}

impl SchoolMotion {
    pub(crate) fn advance(&mut self, state: &mut GlobalState, delta: Duration) {
        self.elapsed = self.elapsed.saturating_add(delta);
        let span = crate::vehicle::spawn_size(state.vehicle_count);
        if self.scene == VisitorScene::SharkHunt {
            let radius = span * 0.34;
            let angular_speed = state.vehicle_max_speed * 0.45 / radius;
            let phase = (self.elapsed.as_secs_f64() * angular_speed as f64) as f32;
            state.groups.table[0].target = None;
            state.groups.table[0].guide = Some(SchoolGuide {
                center: Vec3::new(
                    radius * phase.cos(),
                    span * 0.06 * (phase * 0.5).sin(),
                    span * 0.20 * phase.sin(),
                ),
                velocity: Vec3::new(
                    -radius * angular_speed * phase.sin(),
                    span * 0.03 * angular_speed * (phase * 0.5).cos(),
                    span * 0.20 * angular_speed * phase.cos(),
                ),
                half_length: span * 0.22,
                radius: span * 0.08,
            });
            return;
        }
        let radius = span * 0.30;
        let phase = self.elapsed.as_secs_f64() * (6.0 / radius as f64);
        for group in 0..3 {
            let angle = phase as f32 + group as f32 * TAU / 3.0;
            state.groups.table[group].target = Some(Vec3::new(
                radius * angle.cos(),
                span * 0.07 * (angle * 0.7).sin(),
                span * 0.20 * angle.sin(),
            ));
        }
    }
}

/// Activate public tuning explicitly; Species::ReefDrift remains the loose Lab reference.
pub(crate) fn configure(world: &mut World, scene: VisitorScene) {
    if scene == VisitorScene::Murmuration {
        return;
    }
    let mut motion = SchoolMotion { scene, ..default() };
    let mut state = world.resource_mut::<GlobalState>();
    if scene == VisitorScene::ReefDrift {
        state.groups = Groups {
            count: 3,
            ..default()
        };
        state.vehicle_separation_distance = 7.0;
        state.vehicle_separation_factor = 2.0;
        state.vehicle_alignment_distance = 28.0;
        state.vehicle_alignment_factor = 1.2;
        state.vehicle_cohesion_distance = 34.0;
        state.vehicle_cohesion_factor = 0.5;
        state.vehicle_seek_factor = 0.35;
    } else {
        // The soft corridor leaves its interior free of a central attraction force.
        // Pursuit, lunge, flee and panic/recovery retain their shared kernels.
        state.vehicle_separation_distance = 5.0;
        state.vehicle_separation_factor = 2.2;
        state.vehicle_alignment_distance = 45.0;
        state.vehicle_alignment_factor = 0.15;
        state.vehicle_cohesion_distance = 32.0;
        state.vehicle_cohesion_factor = 0.1;
        state.vehicle_seek_factor = 0.5;
    }
    motion.advance(&mut state, Duration::ZERO);
    world.insert_resource(motion);
}

/// Public guide groups must not become inherited targets in a later Lab species preset.
pub(crate) fn release(world: &mut World) {
    if let Some(motion) = world.remove_resource::<SchoolMotion>() {
        let mut state = world.resource_mut::<GlobalState>();
        if motion.scene == VisitorScene::ReefDrift {
            state.groups = default();
        } else {
            // Keep visible hunters available for Lab inspection, without a hidden moving target.
            state.groups.table[0].target = None;
            state.groups.table[0].guide = None;
        }
    }
}

pub(crate) fn prepare_frame(
    clock: Res<SimulationClock>,
    mut motion: Option<ResMut<SchoolMotion>>,
    mut state: ResMut<GlobalState>,
) {
    if let Some(motion) = &mut motion {
        motion.advance(&mut state, clock.0.delta());
    }
}

/// Scenic follows the encounter's soft envelope; Free, Ride and reduced motion own their views.
pub(crate) fn track_encounter(
    motion: Option<Res<SchoolMotion>>,
    state: Res<GlobalState>,
    visitor: Res<crate::visitor::Visitor>,
    presentation: Res<crate::presentation::Presentation>,
    real: Res<Time<Real>>,
    mut cameras: Query<(&mut Transform, &PanOrbitCamera), With<Camera3d>>,
) {
    if motion.is_none_or(|motion| motion.scene != VisitorScene::SharkHunt)
        || !presentation.orbit
        || presentation.follow.enabled()
        || presentation.director.enabled
        || presentation.camera_override_active()
        || visitor.reduced_motion
    {
        return;
    }
    let Some(guide) = state.groups.guide(0) else {
        return;
    };
    let blend = 1.0 - (-presentation.camera_dt(&real) * 2.0).exp();
    for (mut transform, orbit) in &mut cameras {
        let focus =
            transform.translation + transform.forward() * orbit.last_anchor_depth().abs() as f32;
        transform.translation += (guide.center - focus) * blend;
    }
}

pub(crate) struct SchoolingPlugin;
impl Plugin for SchoolingPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            Update,
            prepare_frame
                .after(crate::presentation::prepare_simulation)
                .before(crate::vehicle::step_cpu)
                .before(crate::gpu_flock::update_gpu_params),
        )
        .add_systems(
            PostUpdate,
            track_encounter
                .after(crate::presentation::animate_camera)
                .before(crate::scene::frame_world),
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::visitor::{PopulationTier, Visitor};

    fn world() -> World {
        let mut world = World::new();
        world.insert_resource(GlobalState::default());
        world.insert_resource(crate::scene::Appearance::default());
        world.insert_resource(crate::presentation::Presentation::default());
        world.insert_resource(crate::swarm::SwarmScene::default());
        world.insert_resource(Visitor::default());
        world
    }

    #[test]
    fn reef_guides_are_separated_bounded_and_move_at_every_tier() {
        for tier in PopulationTier::ALL {
            let mut world = world();
            crate::visitor::apply(&mut world, VisitorScene::ReefDrift, tier);
            let mut motion = world.resource::<SchoolMotion>().clone();
            let mut state = world.resource::<GlobalState>().clone();
            let before = state.groups;
            let bounds = crate::vehicle::walls(state.vehicle_count) * 0.5;
            for _ in 0..120 {
                motion.advance(&mut state, Duration::from_millis(500));
                for i in 0..3 {
                    let point = state.groups.table[i].target.unwrap();
                    assert!(point.is_finite());
                    assert!(point.abs().cmple(bounds).all());
                    for j in 0..i {
                        assert!(point.distance(state.groups.table[j].target.unwrap()) > 30.0);
                    }
                }
            }
            assert_ne!(state.groups, before);
            assert_eq!(state.groups.hunter_count(), 0);
        }
    }

    #[test]
    fn guide_phase_depends_on_simulation_duration_and_zero_delta_is_stable() {
        let mut whole = SchoolMotion::default();
        let mut split = whole.clone();
        let mut a = GlobalState::default();
        let mut b = a.clone();
        whole.advance(&mut a, Duration::from_secs(10));
        for _ in 0..1000 {
            split.advance(&mut b, Duration::from_millis(10));
        }
        assert_eq!(whole.elapsed, split.elapsed);
        assert_eq!(a.groups, b.groups);
        let paused = b.groups;
        split.advance(&mut b, Duration::ZERO);
        assert_eq!(b.groups, paused);
    }

    #[test]
    fn public_restarts_reset_phase_and_lab_presets_release_owned_groups() {
        let mut world = world();
        crate::visitor::apply(&mut world, VisitorScene::ReefDrift, PopulationTier::Medium);
        let initial = world.resource::<GlobalState>().groups;
        world.resource_scope(|world, mut motion: Mut<SchoolMotion>| {
            motion.advance(
                &mut world.resource_mut::<GlobalState>(),
                Duration::from_secs(10),
            );
        });
        crate::visitor::reset_view(&mut world);
        assert_eq!(
            world.resource::<SchoolMotion>().elapsed,
            Duration::from_secs(10)
        );
        crate::visitor::apply(&mut world, VisitorScene::ReefDrift, PopulationTier::Medium);
        assert_eq!(world.resource::<GlobalState>().groups, initial);
        assert_eq!(world.resource::<SchoolMotion>().elapsed, Duration::ZERO);
        crate::visitor::set_lab(&mut world, true);
        assert!(!world.contains_resource::<SchoolMotion>());
        assert_eq!(world.resource::<GlobalState>().groups.count, 0);
        crate::species::apply(&mut world, crate::species::Species::ReefDrift);
        let state = world.resource::<GlobalState>();
        assert_eq!(state.vehicle_wander_factor, 3.0);
        assert_eq!(state.vehicle_cohesion_factor, 0.15);
        assert_eq!(state.groups.count, 0);
        crate::visitor::set_lab(&mut world, false);
        assert!(world.contains_resource::<SchoolMotion>());
        assert_eq!(world.resource::<GlobalState>().groups.count, 3);
        crate::showcase::apply(&mut world, crate::showcase::Showcase::Calm);
        assert!(!world.contains_resource::<SchoolMotion>());
    }

    #[test]
    fn shark_school_travels_inside_bounds_and_keeps_hunter_configuration() {
        for tier in PopulationTier::ALL {
            let mut world = world();
            crate::visitor::apply(&mut world, VisitorScene::SharkHunt, tier);
            let mut motion = world.resource::<SchoolMotion>().clone();
            let mut state = world.resource::<GlobalState>().clone();
            let hunters = state.groups.table[1];
            let start = state.groups.table[0].guide.unwrap().center;
            let bounds = crate::vehicle::walls(state.vehicle_count) * 0.5;
            let mut span = 0.0_f32;
            for _ in 0..120 {
                motion.advance(&mut state, Duration::from_millis(500));
                let guide = state.groups.table[0].guide.unwrap();
                let point = guide.center;
                assert!(point.is_finite() && point.abs().cmple(bounds).all());
                assert!(guide.velocity.is_finite());
                assert_eq!(state.groups.table[0].target, None);
                span = span.max(start.distance(point));
                assert_eq!(state.groups.table[1], hunters);
                assert_eq!(state.groups.hunter_count(), 3);
            }
            assert!(span > crate::vehicle::spawn_size(state.vehicle_count) * 0.5);
            assert_eq!(state.vehicle_cohesion_factor, 0.1);
        }
    }

    #[test]
    fn shark_lab_entry_releases_guide_and_legacy_attack_restores_origin_target() {
        let mut world = world();
        crate::visitor::apply(&mut world, VisitorScene::SharkHunt, PopulationTier::Medium);
        crate::visitor::set_lab(&mut world, true);
        assert!(!world.contains_resource::<SchoolMotion>());
        let state = world.resource::<GlobalState>();
        assert_eq!(state.groups.hunter_count(), 3);
        assert_eq!(state.groups.table[0].target, None);
        assert_eq!(state.groups.table[0].guide, None);
        crate::species::apply(&mut world, crate::species::Species::SharkAttack);
        let state = world.resource::<GlobalState>();
        assert_eq!(state.groups.table[0].target, Some(Vec3::ZERO));
        assert_eq!(state.vehicle_cohesion_factor, 6.0);
        assert_eq!(state.vehicle_seek_factor, 7.0);
        assert_eq!(state.groups.table[1].size_scale, 5.0);
    }

    #[test]
    fn scenic_tracks_the_encounter_but_free_ride_and_reduced_motion_do_not() {
        let mut app = App::new();
        app.insert_resource(Time::<Real>::default());
        app.insert_resource(GlobalState::default());
        app.insert_resource(crate::scene::Appearance::default());
        app.insert_resource(crate::presentation::Presentation::default());
        app.insert_resource(crate::swarm::SwarmScene::default());
        app.insert_resource(Visitor::default());
        crate::visitor::apply(
            app.world_mut(),
            VisitorScene::SharkHunt,
            PopulationTier::Medium,
        );
        let camera = app
            .world_mut()
            .spawn((
                Camera3d::default(),
                Transform::from_xyz(0.0, 0.0, 300.0),
                PanOrbitCamera::default().with_initial_anchor_depth(300.0),
            ))
            .id();
        app.add_systems(Update, track_encounter);
        app.world_mut()
            .resource_mut::<Time<Real>>()
            .advance_by(Duration::from_millis(100));
        let original = *app.world().get::<Transform>(camera).unwrap();
        app.update();
        assert_ne!(*app.world().get::<Transform>(camera).unwrap(), original);
        for mode in 0..3 {
            *app.world_mut().get_mut::<Transform>(camera).unwrap() = original;
            {
                let mut presentation = app
                    .world_mut()
                    .resource_mut::<crate::presentation::Presentation>();
                presentation.orbit = mode != 0;
                presentation.follow.mode = if mode == 1 {
                    crate::follow::FollowMode::FirstPerson
                } else {
                    crate::follow::FollowMode::Off
                };
            }
            app.world_mut().resource_mut::<Visitor>().reduced_motion = mode == 2;
            app.update();
            assert_eq!(*app.world().get::<Transform>(camera).unwrap(), original);
        }
    }
}
