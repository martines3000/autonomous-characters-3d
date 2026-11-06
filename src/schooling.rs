//! Bounded, simulation-clock guides for the curated public fish scenes.
//! Local schooling still uses the shared CPU/GPU group steering and separation kernels.
use std::{f32::consts::TAU, time::Duration};

use bevy::prelude::*;

use crate::{presentation::SimulationClock, swarm::Groups, visitor::VisitorScene, GlobalState};

#[derive(Resource, Clone, Debug, Default)]
pub(crate) struct SchoolMotion {
    elapsed: Duration,
}

impl SchoolMotion {
    pub(crate) fn advance(&mut self, state: &mut GlobalState, delta: Duration) {
        self.elapsed = self.elapsed.saturating_add(delta);
        let span = crate::vehicle::spawn_size(state.vehicle_count);
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
    if scene != VisitorScene::ReefDrift {
        return;
    }
    let mut motion = SchoolMotion::default();
    let mut state = world.resource_mut::<GlobalState>();
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
    motion.advance(&mut state, Duration::ZERO);
    world.insert_resource(motion);
}

/// Public guide groups must not become inherited targets in a later Lab species preset.
pub(crate) fn release(world: &mut World) {
    if world.remove_resource::<SchoolMotion>().is_some() {
        world.resource_mut::<GlobalState>().groups = default();
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

pub(crate) struct SchoolingPlugin;
impl Plugin for SchoolingPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            Update,
            prepare_frame
                .after(crate::presentation::prepare_simulation)
                .before(crate::vehicle::step_cpu)
                .before(crate::gpu_flock::update_gpu_params),
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
}
