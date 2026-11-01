//! Optional camera tracking. GPU samples cross an asynchronous 80-byte Bevy readback;
//! CPU modes sample the live entity. Neither path changes vehicle integration.
use bevy::{
    camera_controller::pan_orbit_camera::prelude::*,
    prelude::*,
    render::{
        extract_resource::{ExtractResource, ExtractResourcePlugin},
        gpu_readback::{Readback, ReadbackComplete},
        render_resource::BufferUsages,
        storage::ShaderBuffer,
        RenderApp,
    },
};
use rand::RngExt;

use crate::{
    gpu_flock::GpuParams,
    presentation::{Presentation, SimulationClock},
    vehicle::{self, Vehicle, VehicleMetadata, VehicleVelocity},
    GlobalState, SimulationMode,
};

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(crate) enum FollowMode {
    #[default]
    Off,
    Chase,
    FirstPerson,
}
impl FollowMode {
    pub(crate) const ALL: [Self; 3] = [Self::Off, Self::Chase, Self::FirstPerson];
    pub(crate) fn name(self) -> &'static str {
        match self {
            Self::Off => "Off",
            Self::Chase => "Chase",
            Self::FirstPerson => "First person",
        }
    }
    pub(crate) fn cycle(self) -> Self {
        match self {
            Self::Off => Self::Chase,
            Self::Chase => Self::FirstPerson,
            Self::FirstPerson => Self::Off,
        }
    }
}

pub(crate) struct Follow {
    pub mode: FollowMode,
    pub id: Option<u32>,
    pub distance: f32,
    revision: u64,
}
impl Default for Follow {
    fn default() -> Self {
        Self {
            mode: FollowMode::Off,
            id: None,
            distance: 6.0,
            revision: 0,
        }
    }
}
impl Follow {
    pub(crate) fn next(&mut self) {
        self.id = None;
        self.revision = self.revision.wrapping_add(1);
    }
    pub(crate) fn select(&mut self, id: u32) {
        self.id = Some(id);
        self.revision = self.revision.wrapping_add(1);
    }
    pub(crate) fn enabled(&self) -> bool {
        self.mode != FollowMode::Off
    }
}

#[derive(Clone, Copy, Debug)]
struct Sample {
    position: Vec3,
    velocity: Vec3,
    rotation: Quat,
    stamp: f32,
}
impl Sample {
    fn predicted(self, elapsed: f32) -> Vec3 {
        self.position + self.velocity * (elapsed - self.stamp).max(0.0)
    }
}

#[derive(Resource, ExtractResource, Clone)]
#[extract_app(RenderApp)]
pub(crate) struct TrackBuffer {
    pub handle: Handle<ShaderBuffer>,
    selector: UVec2,
}

#[derive(Resource, Default)]
struct FollowRuntime {
    mode: FollowMode,
    sample: Option<Sample>,
    readback: Option<Entity>,
    revision: u64,
    gpu: Option<bool>,
    requested_at: f32,
    invalid_samples: u32,
    heading: Vec3,
    position: Vec3,
    spring_velocity: Vec3,
    orbit_yaw: f32,
    blend: Option<(Transform, f32)>,
    projection: Option<PerspectiveProjection>,
}

pub(crate) struct FollowPlugin;
impl Plugin for FollowPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<FollowRuntime>()
            .add_plugins(ExtractResourcePlugin::<TrackBuffer>::default())
            .add_systems(
                Update,
                prepare_tracking
                    .after(crate::gpu_flock::update_gpu_params)
                    .after(crate::vehicle::step_cpu),
            )
            .add_systems(
                PostUpdate,
                follow_camera
                    .before(crate::presentation::animate_camera)
                    .before(bevy::camera::CameraUpdateSystems),
            );
    }
}

// Closed-form critically damped spring for a stationary target. Unlike Euler integration,
// this has the same end state at every frame rate and remains stable after a long frame.
fn spring(position: &mut Vec3, velocity: &mut Vec3, target: Vec3, dt: f32) {
    let displacement = *position - target;
    let impulse = *velocity + displacement * 7.0;
    let decay = (-7.0 * dt).exp();
    *position = target + (displacement + impulse * dt) * decay;
    *velocity = (*velocity - impulse * (7.0 * dt)) * decay;
}

#[allow(clippy::too_many_arguments)]
fn prepare_tracking(
    mut commands: Commands,
    mut presentation: ResMut<Presentation>,
    mut runtime: ResMut<FollowRuntime>,
    state: Res<GlobalState>,
    clock: Res<SimulationClock>,
    real: Res<Time<Real>>,
    mut buffers: ResMut<Assets<ShaderBuffer>>,
    tracking: Option<Res<TrackBuffer>>,
    mut params: ResMut<GpuParams>,
    vehicles: Query<(&Transform, &VehicleVelocity, &VehicleMetadata), With<Vehicle>>,
) {
    params.track = UVec4::ZERO;
    let follow = &mut presentation.follow;
    let gpu = state.mode == SimulationMode::Gpu;
    if runtime.gpu.is_some_and(|previous| previous != gpu) && runtime.revision == follow.revision {
        follow.next();
    }
    runtime.gpu = Some(gpu);
    if !follow.enabled() || state.vehicle_count == 0 {
        if let Some(entity) = runtime.readback.take() {
            commands.entity(entity).despawn();
        }
        commands.remove_resource::<TrackBuffer>();
        runtime.sample = None;
        return;
    }
    let elapsed = clock.elapsed_secs();
    let changed = runtime.revision != follow.revision;
    let stale = !changed
        && (runtime.sample.is_some_and(|s| elapsed - s.stamp > 0.5)
            || runtime.invalid_samples >= 3
            || (runtime.readback.is_some()
                && runtime.sample.is_none()
                && real.elapsed_secs() - runtime.requested_at > 1.0));
    if stale {
        follow.next();
        runtime.sample = None;
    }
    let changed = runtime.revision != follow.revision;
    if changed {
        runtime.sample = None;
        runtime.blend = None;
        runtime.invalid_samples = 0;
        runtime.revision = follow.revision;
    }
    if !gpu {
        let selected = follow
            .id
            .and_then(|id| vehicles.iter().find(|(_, _, m)| m.0.x == id));
        let selected = selected.or_else(|| {
            let count = vehicles.iter().len();
            (count > 0)
                .then(|| vehicles.iter().nth(rand::rng().random_range(0..count)))
                .flatten()
        });
        if let Some((transform, velocity, metadata)) = selected {
            if follow.id != Some(metadata.0.x) {
                runtime.blend = None;
            }
            follow.id = Some(metadata.0.x);
            runtime.sample = Some(Sample {
                position: transform.translation,
                velocity: Vec3::from(velocity.0),
                rotation: transform.rotation,
                stamp: elapsed,
            });
        }
        if let Some(entity) = runtime.readback.take() {
            commands.entity(entity).despawn();
        }
        commands.remove_resource::<TrackBuffer>();
        return;
    }
    if changed || tracking.is_none() || runtime.readback.is_none() {
        if let Some(entity) = runtime.readback.take() {
            commands.entity(entity).despawn();
        }
        let selector = follow.id.map_or_else(
            || UVec2::new(2, rand::rng().random_range(0..state.vehicle_count as u32)),
            |id| UVec2::new(1, id),
        );
        let mut buffer = ShaderBuffer::from(vec![UVec4::ZERO; 5]);
        buffer.buffer_usage |= BufferUsages::COPY_SRC | BufferUsages::COPY_DST;
        let handle = buffers.add(buffer);
        let entity = commands
            .spawn(Readback::buffer(handle.clone()))
            .observe(receive_sample)
            .id();
        commands.insert_resource(TrackBuffer { handle, selector });
        runtime.readback = Some(entity);
        runtime.sample = None;
        runtime.requested_at = real.elapsed_secs();
        runtime.invalid_samples = 0;
        params.track = UVec4::new(selector.x, selector.y, elapsed.to_bits(), 0);
    } else if let Some(tracking) = tracking {
        // After resolving a random index, match its stable spawn ID on subsequent frames.
        let selector = follow.id.map_or(tracking.selector, |id| UVec2::new(1, id));
        params.track = UVec4::new(selector.x, selector.y, elapsed.to_bits(), 0);
    }
}

fn receive_sample(
    event: On<ReadbackComplete>,
    mut presentation: ResMut<Presentation>,
    mut runtime: ResMut<FollowRuntime>,
) {
    if runtime.readback != Some(event.entity)
        || !presentation.follow.enabled()
        || runtime.revision != presentation.follow.revision
    {
        return;
    }
    let records: Vec<UVec4> = event.to_shader_type();
    if records.len() != 5 || records[4].z != 1 {
        runtime.invalid_samples += 1;
        return;
    }
    let id = records[4].x;
    if presentation
        .follow
        .id
        .is_some_and(|expected| expected != id)
    {
        return;
    }
    let decode = |index: usize| records[index].to_array().map(f32::from_bits);
    let sample = Sample {
        position: Vec4::from_array(decode(0)).truncate(),
        velocity: Vec4::from_array(decode(1)).truncate(),
        rotation: Quat::from_array(decode(2)),
        stamp: f32::from_bits(records[4].y),
    };
    if !sample.position.is_finite()
        || !sample.velocity.is_finite()
        || !sample.stamp.is_finite()
        || !sample.rotation.is_finite()
        || runtime.sample.is_some_and(|old| sample.stamp < old.stamp)
    {
        return;
    }
    presentation.follow.id = Some(id);
    runtime.sample = Some(sample);
    runtime.invalid_samples = 0;
}

fn follow_camera(
    presentation: Res<Presentation>,
    mut runtime: ResMut<FollowRuntime>,
    clock: Res<SimulationClock>,
    real: Res<Time<Real>>,
    state: Res<GlobalState>,
    mut cameras: Query<(&mut Transform, &mut Projection, &mut PanOrbitCamera), With<Camera3d>>,
) {
    let Ok((mut transform, mut projection, mut orbit)) = cameras.single_mut() else {
        return;
    };
    let Projection::Perspective(projection) = &mut *projection else {
        return;
    };
    if !presentation.follow.enabled() {
        if let Some(saved) = runtime.projection.take() {
            let aspect_ratio = projection.aspect_ratio;
            let far = projection.far;
            *projection = saved;
            projection.aspect_ratio = aspect_ratio;
            projection.far = far;
        }
        runtime.mode = FollowMode::Off;
        runtime.blend = None;
        return;
    }
    if runtime.mode != presentation.follow.mode {
        runtime.mode = presentation.follow.mode;
        runtime.blend = None;
    }
    // Keep the pan/orbit controller from replaying inertia while we own the camera.
    orbit.current_motion = default();
    let Some(sample) = runtime.sample else {
        return;
    };
    if runtime.projection.is_none() {
        runtime.projection = Some(projection.clone());
    }
    if runtime.blend.is_none() {
        runtime.blend = Some((*transform, 0.0));
        runtime.position = transform.translation;
        runtime.spring_velocity = Vec3::ZERO;
        runtime.heading = sample
            .velocity
            .try_normalize()
            .unwrap_or(sample.rotation * Vec3::Y);
        runtime.orbit_yaw = 0.0;
    }
    let dt_sim = clock.delta_secs_f64() as f32;
    let dt = real.delta_secs();
    if let Some(heading) = sample.velocity.try_normalize() {
        let turn = Quat::from_rotation_arc(runtime.heading, heading);
        runtime.heading = (Quat::IDENTITY.slerp(turn, 1.0 - (-dt_sim / 0.2).exp())
            * runtime.heading)
            .normalize_or_zero();
    }
    let position = sample.predicted(clock.elapsed_secs());
    let size = state.vehicle_size.max(0.001);
    let scale = vehicle::world_scale(state.vehicle_count);
    let heading = runtime.heading;
    let mut desired = *transform;
    match presentation.follow.mode {
        FollowMode::Chase => {
            if presentation.orbit {
                runtime.orbit_yaw += presentation.orbit_speed.to_radians() * dt;
            }
            let distance = presentation.follow.distance.max(2.0) * size * scale;
            let offset = Quat::from_rotation_y(runtime.orbit_yaw)
                * (-heading * distance + Vec3::Y * distance * 0.3);
            let target = position + offset;
            let FollowRuntime {
                position: camera,
                spring_velocity,
                ..
            } = &mut *runtime;
            // Real time keeps bullet-time orbit live. Heading/prediction still use simulated time.
            spring(camera, spring_velocity, target, dt);
            desired.translation = *camera;
            desired.look_at(position + sample.velocity * 0.25, Vec3::Y);
            projection.fov = runtime.projection.as_ref().unwrap().fov;
            // PanOrbitCamera scales near by the old world-view anchor depth. At a
            // boid-sized chase distance that inherited plane would clip the boid away.
            projection.near = size * 0.05;
        }
        FollowMode::FirstPerson => {
            desired.translation = position + heading * (size * 2.0);
            desired.look_to(
                heading,
                if heading.dot(Vec3::Y).abs() > 0.98 {
                    Vec3::Z
                } else {
                    Vec3::Y
                },
            );
            projection.fov = 75.0_f32.to_radians();
            projection.near = size * 0.05;
        }
        FollowMode::Off => return,
    }
    desired.rotation = transform
        .rotation
        .slerp(desired.rotation, 1.0 - (-12.0 * dt).exp());
    let (start, elapsed) = runtime.blend.as_mut().unwrap();
    *elapsed += dt;
    let smooth = crate::presentation::transition_smoothstep(*elapsed);
    transform.translation = start.translation.lerp(desired.translation, smooth);
    transform.rotation = start.rotation.slerp(desired.rotation, smooth);
    orbit.last_anchor_depth = -(transform.translation.distance(position).max(size) as f64);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn spring_converges_with_same_end_state_at_every_frame_rate() {
        let mut states = Vec::new();
        for hz in [30, 60, 144] {
            let mut position = Vec3::new(50.0, -20.0, 7.0);
            let mut velocity = Vec3::new(3.0, -4.0, 10.0);
            let target = Vec3::new(1.0, 2.0, 3.0);
            for _ in 0..hz {
                spring(&mut position, &mut velocity, target, 1.0 / hz as f32);
            }
            states.push((position, velocity));
            for _ in 0..hz * 2 {
                spring(&mut position, &mut velocity, target, 1.0 / hz as f32);
            }
            assert!(position.distance(target) < 0.0001);
            assert!(velocity.length() < 0.0001);
        }
        for pair in states.windows(2) {
            assert!(pair[0].0.distance(pair[1].0) < 0.0001);
            assert!(pair[0].1.distance(pair[1].1) < 0.0001);
        }
    }

    #[test]
    fn tracking_lifecycle_preserves_explicit_ids_rejects_old_events_and_releases_readback() {
        let mut app = crate::presentation::tests::clock_app();
        app.init_resource::<FollowRuntime>()
            .init_resource::<GlobalState>()
            .init_resource::<GpuParams>()
            .init_resource::<Assets<ShaderBuffer>>()
            .add_systems(
                Update,
                prepare_tracking.after(crate::presentation::prepare_simulation),
            );
        app.world_mut()
            .resource_mut::<Time<Real>>()
            .advance_by(std::time::Duration::from_secs(2));
        {
            let mut p = app.world_mut().resource_mut::<Presentation>();
            p.follow.mode = FollowMode::Chase;
            p.follow.select(13);
        }
        app.update();
        assert_eq!(
            app.world()
                .resource::<GpuParams>()
                .track
                .truncate()
                .truncate(),
            UVec2::new(1, 13)
        );
        let old_entity = app.world().resource::<FollowRuntime>().readback.unwrap();
        let valid = |id| {
            [
                UVec4::ZERO,
                UVec4::ZERO,
                UVec4::new(0, 0, 0, 1.0_f32.to_bits()),
                UVec4::ZERO,
                UVec4::new(id, 0.0_f32.to_bits(), 1, 0),
            ]
            .into_iter()
            .flat_map(|v| v.to_array())
            .flat_map(u32::to_le_bytes)
            .collect::<Vec<u8>>()
        };
        app.world_mut().trigger(ReadbackComplete {
            entity: old_entity,
            data: valid(13),
        });
        assert!(app.world().resource::<FollowRuntime>().sample.is_some());
        app.world_mut()
            .resource_mut::<Presentation>()
            .follow
            .select(17);
        app.world_mut().trigger(ReadbackComplete {
            entity: old_entity,
            data: valid(13),
        });
        assert_eq!(app.world().resource::<Presentation>().follow.id, Some(17));
        app.update();
        let entity = app.world().resource::<FollowRuntime>().readback.unwrap();
        assert_ne!(entity, old_entity);
        for _ in 0..3 {
            app.world_mut().trigger(ReadbackComplete {
                entity,
                data: vec![0; 80],
            });
        }
        app.update();
        assert_eq!(app.world().resource::<GpuParams>().track.x, 2);
        app.world_mut().resource_mut::<Presentation>().follow.mode = FollowMode::Off;
        app.update();
        assert_eq!(app.world().resource::<GpuParams>().track, UVec4::ZERO);
        assert!(!app.world().contains_resource::<TrackBuffer>());
        assert_eq!(
            app.world_mut()
                .query::<&Readback>()
                .iter(app.world())
                .count(),
            0
        );
        // Explicit ID wins over automatic repicking on a backend transition.
        app.world_mut().resource_mut::<GlobalState>().mode = SimulationMode::Octree;
        let pose = Transform::from_xyz(7.0, 8.0, 9.0);
        app.world_mut().spawn((
            Vehicle,
            VehicleMetadata(UVec4::new(42, 0, 0, 0)),
            VehicleVelocity(Vec3::X.into()),
            pose,
        ));
        {
            let mut p = app.world_mut().resource_mut::<Presentation>();
            p.follow.mode = FollowMode::Chase;
            p.follow.select(42);
        }
        app.update();
        assert_eq!(app.world().resource::<Presentation>().follow.id, Some(42));
        let sample = app.world().resource::<FollowRuntime>().sample.unwrap();
        assert_eq!(sample.position, pose.translation);
        assert_eq!(
            sample.stamp,
            app.world().resource::<SimulationClock>().elapsed_secs()
        );
        assert!(app.world().resource::<FollowRuntime>().readback.is_none());
    }

    #[test]
    fn first_person_restores_projection_and_paused_chase_orbits() {
        let mut app = crate::presentation::tests::clock_app();
        app.init_resource::<FollowRuntime>()
            .init_resource::<GlobalState>()
            .add_systems(
                Update,
                follow_camera.after(crate::presentation::prepare_simulation),
            );
        let original = PerspectiveProjection {
            fov: 0.9,
            near: 100.0,
            ..default()
        };
        let camera = app
            .world_mut()
            .spawn((
                Camera3d::default(),
                Transform::from_xyz(0.0, 0.0, 100.0),
                Projection::Perspective(original.clone()),
                PanOrbitCamera::default(),
            ))
            .id();
        app.world_mut().resource_mut::<Presentation>().follow.mode = FollowMode::FirstPerson;
        app.world_mut().resource_mut::<FollowRuntime>().sample = Some(Sample {
            position: Vec3::ZERO,
            velocity: Vec3::X,
            rotation: Quat::IDENTITY,
            stamp: 0.0,
        });
        for _ in 0..5 {
            app.world_mut()
                .resource_mut::<Time<Real>>()
                .advance_by(std::time::Duration::from_millis(200));
            app.update();
        }
        let Projection::Perspective(first) = app.world().get::<Projection>(camera).unwrap() else {
            panic!()
        };
        assert_eq!(first.fov, 75.0_f32.to_radians());
        assert_eq!(
            first.near,
            app.world().resource::<GlobalState>().vehicle_size * 0.05
        );
        app.world_mut().resource_mut::<Presentation>().follow.mode = FollowMode::Off;
        app.update();
        let Projection::Perspective(restored) = app.world().get::<Projection>(camera).unwrap()
        else {
            panic!()
        };
        assert_eq!(restored.fov, original.fov);
        assert_eq!(restored.near, original.near);
        {
            let mut presentation = app.world_mut().resource_mut::<Presentation>();
            presentation.follow.mode = FollowMode::Chase;
            presentation.paused = true;
            presentation.orbit = true;
        }
        for _ in 0..20 {
            app.update();
        }
        let Projection::Perspective(chase) = app.world().get::<Projection>(camera).unwrap() else {
            panic!()
        };
        assert_eq!(
            chase.near,
            app.world().resource::<GlobalState>().vehicle_size * 0.05
        );
        let before = *app.world().get::<Transform>(camera).unwrap();
        let position = app
            .world()
            .resource::<FollowRuntime>()
            .sample
            .unwrap()
            .predicted(app.world().resource::<SimulationClock>().elapsed_secs());
        app.update();
        let after = *app.world().get::<Transform>(camera).unwrap();
        assert!(before.translation.distance(after.translation) > 0.01);
        assert!(before.rotation.angle_between(after.rotation) > 0.001);
        assert_eq!(
            app.world()
                .resource::<FollowRuntime>()
                .sample
                .unwrap()
                .predicted(app.world().resource::<SimulationClock>().elapsed_secs()),
            position
        );
    }

    #[test]
    fn extrapolation_freezes_with_simulation_clock() {
        let sample = Sample {
            position: Vec3::ONE,
            velocity: Vec3::X * 10.0,
            rotation: Quat::IDENTITY,
            stamp: 0.0,
        };
        let mut app = crate::presentation::tests::clock_app();
        app.world_mut()
            .resource_mut::<Time<Real>>()
            .advance_by(std::time::Duration::from_secs_f32(0.1));
        app.update();
        app.world_mut().resource_mut::<Presentation>().paused = true;
        let before = sample.predicted(app.world().resource::<SimulationClock>().elapsed_secs());
        for _ in 0..10 {
            app.world_mut()
                .resource_mut::<Time<Real>>()
                .advance_by(std::time::Duration::from_secs(1));
            app.update();
            assert_eq!(
                sample.predicted(app.world().resource::<SimulationClock>().elapsed_secs()),
                before
            );
        }
        assert!(sample.predicted(0.2).distance(Vec3::new(3.0, 1.0, 1.0)) < 0.00001);
    }
}
