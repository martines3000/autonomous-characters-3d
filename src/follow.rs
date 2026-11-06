//! Optional camera tracking. GPU samples cross an asynchronous 80-byte Bevy readback;
//! CPU modes sample the live entity. Neither path changes vehicle integration.
use bevy::{
    camera_controller::pan_orbit_camera::prelude::*,
    ecs::system::SystemParam,
    post_process::dof::{DepthOfField, DepthOfFieldMode},
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

/// The same camera modes can ride a prey spawn ID or a hunter's configured ordinal.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(u32)]
pub(crate) enum FollowSubject {
    #[default]
    Prey = 0,
    Hunter = 1,
}

pub(crate) struct Follow {
    pub mode: FollowMode,
    pub id: Option<u32>,
    pub distance: f32,
    pub dof: bool,
    pub aperture: f32,
    subject: FollowSubject,
    previous_id: Option<u32>,
    revision: u64,
}
impl Default for Follow {
    fn default() -> Self {
        Self {
            mode: FollowMode::Off,
            id: None,
            distance: 6.0,
            dof: true,
            aperture: 2.8,
            subject: FollowSubject::Prey,
            previous_id: None,
            revision: 0,
        }
    }
}
impl Follow {
    /// Preserve the readback generation boundary when replacing all presentation settings.
    pub(crate) fn restart_after(&mut self, previous: &Self) {
        self.revision = previous.revision.wrapping_add(1);
    }
    pub(crate) fn subject(&self) -> FollowSubject {
        self.subject
    }
    pub(crate) fn set_subject(&mut self, subject: FollowSubject) {
        if self.subject != subject {
            self.subject = subject;
            self.id = None;
            self.previous_id = None;
            self.revision = self.revision.wrapping_add(1);
        }
    }
    pub(crate) fn next(&mut self) {
        self.previous_id = self.id.take();
        self.revision = self.revision.wrapping_add(1);
    }
    pub(crate) fn select(&mut self, id: u32) {
        self.id = Some(id);
        self.previous_id = None;
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
    hunter_layout: Vec<(u32, crate::hunter::HunterModel)>,
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
                (follow_camera, followed_marker)
                    .chain()
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
    hunters: Option<Res<crate::hunter::Hunters>>,
) {
    params.track = UVec4::ZERO;
    let follow = &mut presentation.follow;
    let gpu = state.mode == SimulationMode::Gpu;
    let configs = state.groups.hunter_configs();
    let hunter_count = if gpu {
        configs.len()
    } else {
        hunters
            .as_ref()
            .map_or(0, |hunters| hunters.boids.len().min(configs.len()))
    };
    if follow.subject == FollowSubject::Hunter && hunter_count == 0 {
        follow.set_subject(FollowSubject::Prey);
    }
    let layout: Vec<_> = configs
        .iter()
        .map(|(group, settings)| (*group, settings.hunter_model))
        .collect();
    let backend_changed = runtime.gpu.is_some_and(|previous| previous != gpu);
    let hunters_changed =
        follow.subject == FollowSubject::Hunter && runtime.hunter_layout != layout;
    if ((backend_changed || hunters_changed) && runtime.revision == follow.revision)
        || (follow.subject == FollowSubject::Hunter
            && follow.id.is_some_and(|id| id as usize >= hunter_count))
    {
        follow.next();
    }
    runtime.gpu = Some(gpu);
    runtime.hunter_layout = layout;
    let count = if follow.subject == FollowSubject::Hunter {
        hunter_count
    } else {
        state.vehicle_count
    };
    if !follow.enabled() || count == 0 {
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
        if follow.subject == FollowSubject::Hunter {
            let id = follow
                .id
                .unwrap_or_else(|| random_hunter(hunter_count, follow.previous_id));
            let hunter = hunters.as_ref().unwrap().boids[id as usize];
            follow.id = Some(id);
            runtime.sample = Some(Sample {
                position: hunter.position,
                velocity: hunter.velocity,
                rotation: hunter.rotation,
                stamp: elapsed,
            });
        } else {
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
        let selector = if follow.subject == FollowSubject::Hunter {
            UVec2::new(
                3,
                follow
                    .id
                    .unwrap_or_else(|| random_hunter(hunter_count, follow.previous_id)),
            )
        } else {
            follow.id.map_or_else(
                || UVec2::new(2, rand::rng().random_range(0..state.vehicle_count as u32)),
                |id| UVec2::new(1, id),
            )
        };
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
        let selector = follow.id.map_or(tracking.selector, |id| {
            UVec2::new(
                if follow.subject == FollowSubject::Hunter {
                    3
                } else {
                    1
                },
                id,
            )
        });
        params.track = UVec4::new(selector.x, selector.y, elapsed.to_bits(), 0);
    }
}

// Hunter ordinals are stable within one layout. Prefer another hunter when one is available.
fn random_hunter(count: usize, previous: Option<u32>) -> u32 {
    let previous = previous.filter(|id| (*id as usize) < count && count > 1);
    let index = rand::rng().random_range(0..count - usize::from(previous.is_some())) as u32;
    index + u32::from(previous.is_some_and(|previous| index >= previous))
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
    // Tags keep equal-numbered prey/hunter identities from crossing a source boundary.
    if records[4].w != presentation.follow.subject as u32 {
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

fn followed_size(
    follow: &Follow,
    state: &GlobalState,
    appearance: &crate::scene::Appearance,
    sample: &Sample,
) -> f32 {
    let base = state.vehicle_size.max(0.001);
    if follow.subject != FollowSubject::Hunter {
        return base;
    }
    let configs = state.groups.hunter_configs();
    let Some((_, group)) = follow.id.and_then(|id| configs.get(id as usize)) else {
        return base;
    };
    // hunter_mesh normalizes forward length to 2.5. These are the exact multipliers
    // used by vehicle_instancing.wesl, including the optional forward Glow stretch.
    let stretch = if appearance.style == crate::scene::SceneStyle::Glow {
        crate::vehicle_appearance::streak_scale(
            sample.velocity.length() / state.vehicle_max_speed.max(f32::EPSILON),
            appearance.streak,
        )
    } else {
        1.0
    };
    base * appearance.vehicles.visual_size * group.size_scale * appearance.vehicles.length * stretch
}

fn focus_distance(mode: FollowMode, eye: Vec3, boid: Vec3, size: f32) -> Option<f32> {
    match mode {
        FollowMode::Off => None,
        FollowMode::Chase => Some(eye.distance(boid).max(size * 0.1)),
        FollowMode::FirstPerson => Some(size * 10.0),
    }
}

type FollowCameras<'w, 's> = Query<
    'w,
    's,
    (
        Entity,
        &'static mut Transform,
        &'static mut Projection,
        &'static mut PanOrbitCamera,
        Option<&'static mut DepthOfField>,
    ),
    With<Camera3d>,
>;

#[derive(SystemParam)]
struct FollowScene<'w> {
    state: Res<'w, GlobalState>,
    appearance: Res<'w, crate::scene::Appearance>,
}

fn follow_camera(
    mut commands: Commands,
    mut presentation: ResMut<Presentation>,
    mut runtime: ResMut<FollowRuntime>,
    clock: Res<SimulationClock>,
    real: Res<Time<Real>>,
    scene: FollowScene,
    mut cameras: FollowCameras,
) {
    let state = &scene.state;
    let appearance = &scene.appearance;
    let Ok((entity, mut transform, mut projection, mut orbit, mut dof)) = cameras.single_mut()
    else {
        return;
    };
    let Projection::Perspective(projection) = &mut *projection else {
        return;
    };
    if (!presentation.follow.enabled() || !presentation.follow.dof) && dof.is_some() {
        commands.entity(entity).remove::<DepthOfField>();
    }
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
    if runtime.revision != presentation.follow.revision {
        return;
    }
    // Keep the pan/orbit controller from replaying inertia while we own the camera.
    orbit.current_motion = default();
    let Some(sample) = runtime.sample else {
        return;
    };
    if runtime.projection.is_none() {
        runtime.projection = Some(projection.clone());
    }
    if presentation.camera_delta == Some(0.0) {
        return;
    }
    if runtime.blend.is_none() {
        presentation.director.reset_subject();
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
    let dt = presentation.camera_dt(&real);
    if let Some(heading) = sample.velocity.try_normalize() {
        let turn = Quat::from_rotation_arc(runtime.heading, heading);
        runtime.heading = (Quat::IDENTITY.slerp(turn, 1.0 - (-dt_sim / 0.2).exp())
            * runtime.heading)
            .normalize_or_zero();
    }
    let position = sample.predicted(clock.elapsed_secs());
    let size = followed_size(&presentation.follow, state, appearance, &sample);
    let scale = if presentation.follow.subject == FollowSubject::Hunter {
        1.0
    } else {
        vehicle::world_scale(state.vehicle_count)
    };
    let heading = runtime.heading;
    let body_scale = if presentation.follow.subject != FollowSubject::Hunter
        && appearance.style == crate::scene::SceneStyle::Glow
    {
        let speed01 = sample.velocity.length() / state.vehicle_max_speed.max(f32::EPSILON);
        appearance.vehicles.visual_size
            * appearance.vehicles.length
            * crate::vehicle_appearance::streak_scale(speed01, appearance.streak)
    } else {
        1.0
    };
    let mut desired = *transform;
    if presentation.director.enabled {
        let target = presentation.director.eye(position, heading, size * scale);
        let FollowRuntime {
            position: camera,
            spring_velocity,
            ..
        } = &mut *runtime;
        spring(camera, spring_velocity, target, dt);
        desired.translation = *camera;
        desired.look_at(
            position,
            if heading.dot(Vec3::Y).abs() > 0.98 {
                Vec3::Z
            } else {
                Vec3::Y
            },
        );
        projection.fov = runtime.projection.as_ref().unwrap().fov;
        projection.near = size * 0.05;
    } else {
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
                desired.translation = position + heading * (size * body_scale * 2.0);
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
    if presentation.follow.dof {
        let settings = DepthOfField {
            mode: DepthOfFieldMode::Gaussian,
            focal_distance: focus_distance(
                presentation.follow.mode,
                transform.translation,
                position,
                size,
            )
            .unwrap(),
            // A virtual sensor in body units makes the lens useful in this large-unit world.
            sensor_height: size * 0.8,
            aperture_f_stops: presentation.follow.aperture,
            max_circle_of_confusion_diameter: 12.0,
            max_depth: vehicle::walls(state.vehicle_count).max_element() * 10.0,
        };
        if let Some(dof) = dof.as_mut() {
            **dof = settings;
        } else {
            commands.entity(entity).insert(settings);
        }
    }
}

fn followed_marker(
    presentation: Res<Presentation>,
    runtime: Res<FollowRuntime>,
    clock: Res<SimulationClock>,
    state: Res<GlobalState>,
    appearance: Res<crate::scene::Appearance>,
    mut gizmos: Gizmos,
) {
    if runtime.revision != presentation.follow.revision {
        return;
    }
    if presentation.follow.mode == FollowMode::Chase && !presentation.director.enabled {
        if let Some(sample) = runtime.sample {
            let position = sample.predicted(clock.elapsed_secs());
            let normal = sample.velocity.try_normalize().unwrap_or(Vec3::Y);
            gizmos.circle(
                Isometry3d::new(position, Quat::from_rotation_arc(Vec3::Z, normal)),
                followed_size(&presentation.follow, &state, &appearance, &sample) * 1.4,
                Color::srgba(1.0, 0.85, 0.3, 0.65),
            );
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tracking_app() -> App {
        let mut app = crate::presentation::tests::clock_app();
        app.init_resource::<FollowRuntime>()
            .init_resource::<GlobalState>()
            .init_resource::<GpuParams>()
            .init_resource::<Assets<ShaderBuffer>>()
            .init_resource::<crate::hunter::Hunters>()
            .add_systems(
                Update,
                prepare_tracking.after(crate::presentation::prepare_simulation),
            );
        app
    }

    fn hunter_groups() -> crate::swarm::Groups {
        let mut groups = crate::swarm::Groups {
            count: 2,
            ..default()
        };
        groups.table[1].role = crate::swarm::GroupRole::Hunter;
        groups.table[1].hunter_count = 3;
        groups.table[1].size_scale = 8.0;
        groups
    }

    fn sample_bytes(id: u32, subject: FollowSubject) -> Vec<u8> {
        [
            UVec4::new(1.0_f32.to_bits(), 2.0_f32.to_bits(), 3.0_f32.to_bits(), 0),
            UVec4::ZERO,
            UVec4::new(0, 0, 0, 1.0_f32.to_bits()),
            UVec4::ZERO,
            UVec4::new(id, 0.0_f32.to_bits(), 1, subject as u32),
        ]
        .into_iter()
        .flat_map(|record| record.to_array())
        .flat_map(u32::to_le_bytes)
        .collect()
    }

    #[test]
    fn subject_selection_preserves_camera_and_next_stays_with_hunters() {
        let mut follow = Follow {
            mode: FollowMode::FirstPerson,
            ..default()
        };
        follow.set_subject(FollowSubject::Hunter);
        follow.select(1);
        let revision = follow.revision;
        follow.next();
        assert_eq!(follow.subject(), FollowSubject::Hunter);
        assert_eq!(follow.mode, FollowMode::FirstPerson);
        assert_eq!(follow.id, None);
        assert_ne!(follow.revision, revision);
        for _ in 0..100 {
            let id = random_hunter(3, follow.previous_id);
            assert!(id < 3);
            assert_ne!(id, 1);
        }
        assert_eq!(random_hunter(1, Some(0)), 0);
        follow.set_subject(FollowSubject::Prey);
        assert_eq!(follow.mode, FollowMode::FirstPerson);
        assert_eq!(follow.previous_id, None);
    }

    #[test]
    fn cpu_hunters_use_live_poses_repick_and_fall_back_when_missing() {
        let mut app = tracking_app();
        {
            let mut state = app.world_mut().resource_mut::<GlobalState>();
            state.mode = SimulationMode::Octree;
            state.vehicle_count = 32;
            state.groups = hunter_groups();
        }
        let configs = hunter_groups().hunter_configs();
        {
            let mut hunters = app.world_mut().resource_mut::<crate::hunter::Hunters>();
            hunters.boids = crate::hunter::initial(&configs, 1.0);
            hunters.configs = configs;
        }
        {
            let mut presentation = app.world_mut().resource_mut::<Presentation>();
            presentation.paused = true;
            presentation.follow.mode = FollowMode::FirstPerson;
            presentation.follow.set_subject(FollowSubject::Hunter);
            presentation.follow.select(1);
        }
        app.update();
        let expected = app.world().resource::<crate::hunter::Hunters>().boids[1];
        let sample = app.world().resource::<FollowRuntime>().sample.unwrap();
        assert_eq!(sample.position, expected.position);
        assert_eq!(sample.velocity, expected.velocity);
        assert_eq!(sample.rotation, expected.rotation);
        assert!(app.world().resource::<FollowRuntime>().readback.is_none());
        app.world_mut()
            .resource_mut::<crate::hunter::Hunters>()
            .boids[1]
            .position += Vec3::X * 7.0;
        app.update();
        assert_eq!(
            app.world()
                .resource::<FollowRuntime>()
                .sample
                .unwrap()
                .position,
            expected.position + Vec3::X * 7.0
        );
        app.world_mut().resource_mut::<Presentation>().follow.next();
        app.update();
        assert_ne!(app.world().resource::<Presentation>().follow.id, Some(1));
        assert_eq!(
            app.world().resource::<Presentation>().follow.subject(),
            FollowSubject::Hunter
        );
        assert_eq!(
            app.world().resource::<Presentation>().follow.mode,
            FollowMode::FirstPerson
        );
        assert!(app.world().resource::<Presentation>().paused);
        app.world_mut().resource_mut::<GlobalState>().groups.table[1].hunter_count = 1;
        app.world_mut()
            .resource_mut::<crate::hunter::Hunters>()
            .boids
            .truncate(1);
        app.update();
        assert_eq!(app.world().resource::<Presentation>().follow.id, Some(0));
        app.world_mut().resource_mut::<GlobalState>().mode = SimulationMode::Gpu;
        app.update();
        assert_eq!(
            app.world()
                .resource::<GpuParams>()
                .track
                .truncate()
                .truncate(),
            UVec2::new(3, 0)
        );
        assert!(app.world().resource::<FollowRuntime>().readback.is_some());
        app.world_mut().resource_mut::<GlobalState>().mode = SimulationMode::Octree;
        app.update();
        assert_eq!(
            app.world().resource::<Presentation>().follow.subject(),
            FollowSubject::Hunter
        );
        assert_eq!(app.world().resource::<Presentation>().follow.id, Some(0));
        assert!(app.world().resource::<FollowRuntime>().sample.is_some());
        assert!(app.world().resource::<FollowRuntime>().readback.is_none());
        app.world_mut().resource_mut::<GlobalState>().mode = SimulationMode::Gpu;
        app.world_mut().resource_mut::<GlobalState>().groups.table[1].hunter_count = 0;
        app.update();
        assert_eq!(
            app.world().resource::<Presentation>().follow.subject(),
            FollowSubject::Prey
        );
        assert_eq!(app.world().resource::<GpuParams>().track.x, 2);
        app.world_mut().resource_mut::<GlobalState>().vehicle_count = 0;
        app.update();
        assert!(app.world().resource::<FollowRuntime>().sample.is_none());
        assert!(!app.world().contains_resource::<TrackBuffer>());
        assert_eq!(app.world().resource::<GpuParams>().track, UVec4::ZERO);
    }

    #[test]
    fn missing_cpu_hunters_fall_back_to_an_existing_prey() {
        let mut app = tracking_app();
        {
            let mut state = app.world_mut().resource_mut::<GlobalState>();
            state.mode = SimulationMode::Octree;
            state.groups = hunter_groups();
        }
        app.world_mut().spawn((
            Vehicle,
            Transform::from_xyz(4.0, 5.0, 6.0),
            VehicleVelocity(Vec3::X.into()),
            VehicleMetadata(UVec4::new(42, 0, 0, 0)),
        ));
        {
            let mut presentation = app.world_mut().resource_mut::<Presentation>();
            presentation.paused = true;
            presentation.follow.mode = FollowMode::Chase;
            presentation.follow.set_subject(FollowSubject::Hunter);
        }
        app.update();
        let presentation = app.world().resource::<Presentation>();
        assert_eq!(presentation.follow.subject(), FollowSubject::Prey);
        assert_eq!(presentation.follow.id, Some(42));
        assert_eq!(presentation.follow.mode, FollowMode::Chase);
        assert!(presentation.paused);
        assert_eq!(
            app.world()
                .resource::<FollowRuntime>()
                .sample
                .unwrap()
                .position,
            Vec3::new(4.0, 5.0, 6.0)
        );
    }

    #[test]
    fn source_switch_rejects_old_entities_and_equal_numbered_other_species() {
        let mut app = tracking_app();
        app.world_mut().resource_mut::<GlobalState>().groups = hunter_groups();
        {
            let mut presentation = app.world_mut().resource_mut::<Presentation>();
            presentation.follow.mode = FollowMode::Chase;
            presentation.follow.select(1);
        }
        app.update();
        let prey_entity = app.world().resource::<FollowRuntime>().readback.unwrap();
        app.world_mut()
            .resource_mut::<Presentation>()
            .follow
            .set_subject(FollowSubject::Hunter);
        app.update();
        let hunter_entity = app.world().resource::<FollowRuntime>().readback.unwrap();
        assert_ne!(hunter_entity, prey_entity);
        for entity in [prey_entity, hunter_entity] {
            app.world_mut().trigger(ReadbackComplete {
                entity,
                data: sample_bytes(1, FollowSubject::Prey),
            });
            assert!(app.world().resource::<FollowRuntime>().sample.is_none());
        }
        app.world_mut().trigger(ReadbackComplete {
            entity: hunter_entity,
            data: sample_bytes(1, FollowSubject::Hunter),
        });
        assert_eq!(app.world().resource::<Presentation>().follow.id, Some(1));
        assert!(app.world().resource::<FollowRuntime>().sample.is_some());
        app.world_mut()
            .resource_mut::<Presentation>()
            .follow
            .set_subject(FollowSubject::Prey);
        app.update();
        let new_prey_entity = app.world().resource::<FollowRuntime>().readback.unwrap();
        for entity in [hunter_entity, new_prey_entity] {
            app.world_mut().trigger(ReadbackComplete {
                entity,
                data: sample_bytes(1, FollowSubject::Hunter),
            });
            assert!(app.world().resource::<FollowRuntime>().sample.is_none());
        }
        app.world_mut().trigger(ReadbackComplete {
            entity: new_prey_entity,
            data: sample_bytes(1, FollowSubject::Prey),
        });
        assert!(app.world().resource::<FollowRuntime>().sample.is_some());
    }

    #[test]
    fn hunter_camera_uses_draw_scale_and_first_person_eye_clears_the_nose() {
        let mut app = crate::presentation::tests::clock_app();
        app.init_resource::<FollowRuntime>()
            .init_resource::<GlobalState>()
            .init_resource::<crate::scene::Appearance>()
            .add_systems(
                Update,
                follow_camera.after(crate::presentation::prepare_simulation),
            );
        app.world_mut().resource_mut::<GlobalState>().groups = hunter_groups();
        {
            let mut appearance = app.world_mut().resource_mut::<crate::scene::Appearance>();
            appearance.vehicles.visual_size = 1.5;
            appearance.vehicles.length = 1.2;
        }
        {
            let mut presentation = app.world_mut().resource_mut::<Presentation>();
            presentation.paused = true;
            presentation.follow.mode = FollowMode::FirstPerson;
            presentation.follow.set_subject(FollowSubject::Hunter);
            presentation.follow.select(0);
        }
        let revision = app.world().resource::<Presentation>().follow.revision;
        let sample = Sample {
            position: Vec3::ZERO,
            velocity: Vec3::Z * 40.0,
            rotation: crate::motion::initial(Vec3::Z * 40.0),
            stamp: 0.0,
        };
        {
            let mut runtime = app.world_mut().resource_mut::<FollowRuntime>();
            runtime.revision = revision;
            runtime.sample = Some(sample);
        }
        let camera = app
            .world_mut()
            .spawn((
                Camera3d::default(),
                Transform::from_xyz(0.0, 0.0, -100.0),
                Projection::default(),
                PanOrbitCamera::default(),
            ))
            .id();
        for _ in 0..6 {
            app.world_mut()
                .resource_mut::<Time<Real>>()
                .advance_by(std::time::Duration::from_millis(200));
            app.update();
        }
        let unit = 1.5 * 8.0 * 1.2;
        let eye = app.world().get::<Transform>(camera).unwrap().translation;
        assert!((eye.z - 2.0 * unit).abs() < 0.0001);
        assert!(eye.z > 1.25 * unit);
        let Projection::Perspective(projection) = app.world().get::<Projection>(camera).unwrap()
        else {
            panic!()
        };
        assert!((projection.near - unit * 0.05).abs() < 0.0001);
        app.world_mut().resource_mut::<Presentation>().follow.mode = FollowMode::Chase;
        for _ in 0..30 {
            app.world_mut()
                .resource_mut::<Time<Real>>()
                .advance_by(std::time::Duration::from_millis(200));
            app.update();
        }
        let expected = Vec3::new(0.0, 6.0 * unit * 0.3, -6.0 * unit);
        assert!(
            app.world()
                .get::<Transform>(camera)
                .unwrap()
                .translation
                .distance(expected)
                < 0.001
        );
        let presentation = app.world().resource::<Presentation>();
        let state = app.world().resource::<GlobalState>();
        let mut appearance = crate::scene::Appearance::default();
        appearance.style = crate::scene::SceneStyle::Glow;
        appearance.vehicles.visual_size = 1.5;
        appearance.vehicles.length = 1.2;
        appearance.streak = 2.0;
        assert!(
            (followed_size(&presentation.follow, state, &appearance, &sample) - unit * 2.0).abs()
                < 0.0001
        );
    }

    #[test]
    #[ignore = "needs a GPU"]
    fn gpu_hunter_tracking_samples_moving_and_paused_poses_with_zero_prey() {
        use crate::gpu_flock::{gpu_params, GpuFlock};
        use bevy::render::settings::{Backends, RenderResources, WgpuSettings};
        let RenderResources(device, queue, ..) =
            bevy::tasks::block_on(bevy::render::renderer::initialize_renderer(
                Backends::PRIMARY,
                None,
                &WgpuSettings::default(),
            ));
        let mut flock = GpuFlock::new(&device);
        let mut state = GlobalState {
            vehicle_count: 32,
            groups: hunter_groups(),
            ..default()
        };
        let step = |flock: &mut GpuFlock, params: &GpuParams| {
            let mut encoder = device.create_command_encoder(&default());
            flock.encode(&device, &queue, &mut encoder, params);
            queue.submit([encoder.finish()]);
        };
        let check = |flock: &GpuFlock, index: usize| {
            let hunter = flock.read_hunters(&device, &queue)[index];
            let records = flock.read_track(&device, &queue);
            assert_eq!(
                records[4],
                UVec4::new(index as u32, 2.5_f32.to_bits(), 1, 1)
            );
            let decode =
                |index: usize| Vec4::from_array(records[index].to_array().map(f32::from_bits));
            assert!(decode(0).truncate().distance(hunter.position) < 0.00001);
            assert!(decode(1).truncate().distance(hunter.velocity) < 0.00001);
            assert!(Quat::from_array(decode(2).to_array()).angle_between(hunter.rotation) < 0.001);
            hunter.position
        };
        let initial = crate::hunter::initial(&state.groups.hunter_configs(), 1.0)[1].position;
        let mut params = gpu_params(&state, Vec3::ZERO, false, 1.0 / 60.0);
        params.track = UVec4::new(3, 1, 2.5_f32.to_bits(), 0);
        for _ in 0..5 {
            step(&mut flock, &params);
        }
        let moving = check(&flock, 1);
        assert!(moving.distance(initial) > 0.01);
        params.dt = 0.0;
        step(&mut flock, &params);
        assert_eq!(check(&flock, 1), moving);
        state.vehicle_count = 0;
        params = gpu_params(&state, Vec3::ZERO, false, 0.0);
        params.track = UVec4::new(3, 1, 2.5_f32.to_bits(), 0);
        step(&mut flock, &params);
        assert_eq!(check(&flock, 1), moving);
        params.dt = 1.0 / 60.0;
        step(&mut flock, &params);
        assert!(check(&flock, 1).distance(moving) > 0.001);
        state.vehicle_count = 32;
        state.groups = default();
        params = gpu_params(&state, Vec3::ZERO, false, 0.0);
        params.track = UVec4::new(3, 0, 2.5_f32.to_bits(), 0);
        step(&mut flock, &params);
        assert_eq!(flock.read_track(&device, &queue)[4], UVec4::ZERO);
    }

    #[test]
    fn replacing_follow_settings_invalidates_previous_readback_generation() {
        let mut previous = Follow::default();
        previous.next();
        let mut replacement = Follow::default();
        replacement.restart_after(&previous);
        assert_ne!(previous.revision, replacement.revision);
        assert_eq!(replacement.mode, FollowMode::Off);
        assert_eq!(replacement.id, None);
    }

    #[test]
    fn depth_of_field_focus_tracks_actual_camera_and_looks_ahead_in_first_person() {
        let boid = Vec3::new(1.0, 2.0, 3.0);
        assert_eq!(focus_distance(FollowMode::Off, Vec3::ZERO, boid, 1.0), None);
        assert_eq!(
            focus_distance(FollowMode::Chase, boid + Vec3::Z * 24.0, boid, 2.0),
            Some(24.0)
        );
        assert_eq!(
            focus_distance(FollowMode::Chase, boid, boid, 2.0),
            Some(0.2)
        );
        assert_eq!(
            focus_distance(FollowMode::FirstPerson, boid, boid, 2.0),
            Some(20.0)
        );
        assert!(Follow::default().dof);
        assert!(!Follow::default().enabled());
    }

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
            .init_resource::<crate::scene::Appearance>()
            .add_systems(Update, crate::scene::sync_glow_camera)
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
                Msaa::Sample4,
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
        for style in [
            crate::scene::SceneStyle::Glow,
            crate::scene::SceneStyle::Classic,
            crate::scene::SceneStyle::Studio,
        ] {
            app.world_mut()
                .resource_mut::<crate::scene::Appearance>()
                .style = style;
            app.update();
            assert_eq!(
                app.world().get::<bevy::camera::Hdr>(camera).is_some(),
                style == crate::scene::SceneStyle::Glow
            );
            assert!(app.world().get::<DepthOfField>(camera).is_some());
            assert_eq!(*app.world().get::<Msaa>(camera).unwrap(), Msaa::Sample4);
        }
        let dof = app.world().get::<DepthOfField>(camera).unwrap();
        assert_eq!(dof.mode, DepthOfFieldMode::Gaussian);
        assert_eq!(
            dof.focal_distance,
            app.world().resource::<GlobalState>().vehicle_size * 10.0
        );
        app.world_mut().resource_mut::<Presentation>().follow.dof = false;
        app.update();
        assert!(app.world().get::<DepthOfField>(camera).is_none());
        app.world_mut().resource_mut::<Presentation>().follow.dof = true;
        app.update();
        assert!(app.world().get::<DepthOfField>(camera).is_some());
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
        assert!(app.world().get::<DepthOfField>(camera).is_none());
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
        let sample_position = app
            .world()
            .resource::<FollowRuntime>()
            .sample
            .unwrap()
            .predicted(app.world().resource::<SimulationClock>().elapsed_secs());
        let actual_distance = app
            .world()
            .get::<Transform>(camera)
            .unwrap()
            .translation
            .distance(sample_position);
        assert!(
            (app.world()
                .get::<DepthOfField>(camera)
                .unwrap()
                .focal_distance
                - actual_distance)
                .abs()
                < 0.0001
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
