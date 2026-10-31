extern crate nalgebra as na;

// Wall half-extents at REFERENCE_COUNT: wandering vehicles steer back inside.
const WORLD_SIZE: Vec3 = Vec3::new(200.0, 200.0, 100.0);
// Edge length at REFERENCE_COUNT of the cube around the origin where vehicles spawn.
const SPAWN_SIZE: f32 = 100.0;
// Vehicle count whose density was measured to flock well; larger counts scale the world.
const REFERENCE_COUNT: f32 = 200.0;
// Tuning and headless measurements used this reference rate.
const REFERENCE_HZ: f32 = 60.0;
// Largest push from one neighbor, in units of separation factor × max speed; reached
// at a quarter of the separation distance so near-coincident vehicles stay bounded.
const MAX_SEPARATION_PUSH: f32 = 3.0;

use std::{f32::consts::PI, ops::Mul};

use crate::{
    instancing::VehicleInstances,
    octree::{Octree, Point},
    target::{Target, TargetSelection},
    vehicle_appearance::{pack_speed_group, VehicleMeshes},
    GlobalState, SimulationMode,
};
use na::Vector3;

use bevy::{camera::visibility::NoFrustumCulling, prelude::*};
use rand::RngExt;

pub struct VehiclePlugin;

#[derive(bevy::ecs::schedule::ScheduleLabel, Clone, Debug, PartialEq, Eq, Hash)]
struct CpuStep;

// Match Bevy's fixed-schedule convention: kernels read the current simulation step through Time.
// Restore the outer clock before any other Update system runs.
pub(crate) fn step_cpu(world: &mut World) {
    if world.resource::<GlobalState>().mode == SimulationMode::Gpu {
        return;
    }
    let frame = *world.resource::<crate::presentation::SimulationFrame>();
    let original = *world.resource::<Time>();
    for _ in 0..frame.steps {
        let mut time = Time::default();
        time.advance_by(frame.step);
        *world.resource_mut::<Time>() = time;
        world.run_schedule(CpuStep);
    }
    *world.resource_mut::<Time>() = original;
}

#[derive(Component)]
pub struct Vehicle;

#[derive(Component, Default)]
struct VehicleVelocity(pub Vector3<f32>);

#[derive(Component, Default)]
struct VehicleAcceleration(pub Vector3<f32>);

#[derive(Component, Default)]
struct VehicleWanderRotation {
    theta: f32,
    phi: f32,
}

#[derive(Default, Resource)]
struct VehicleSpawner {
    vehicle_count: usize,
}

impl Plugin for VehiclePlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<VehicleSpawner>()
            .add_systems(Startup, spawn_flock)
            .add_systems(CpuStep, (movement, update).chain())
            .add_systems(PostUpdate, crate::vehicle_appearance::update_shape)
            .add_systems(
                Update,
                (vehicle_spawner, vehicle_cleanup, step_cpu, sync_instances)
                    .chain()
                    .after(crate::target::input_controls),
            );
    }
}

impl VehicleAcceleration {
    fn apply_force(&mut self, force: Vector3<f32>, mass: f32) {
        self.0 += force / mass;
    }
}

// One entity draws every vehicle; see `instancing`.
fn spawn_flock(mut commands: Commands, mut meshes: ResMut<Assets<Mesh>>) {
    let shapes = VehicleMeshes::new(&mut meshes);
    commands.spawn((
        // A slim dart reads its heading better than the original round cone.
        Mesh3d(shapes.get(crate::vehicle_appearance::VehicleShape::Dart)),
        VehicleInstances::default(),
        // The shader places the instances; the entity's own bounds mean nothing.
        NoFrustumCulling,
    ));
    commands.insert_resource(shapes);
}

fn sync_instances(
    vehicles: Query<(&Transform, &VehicleVelocity), With<Vehicle>>,
    mut flock: Single<&mut VehicleInstances>,
    state: Res<GlobalState>,
) {
    flock.0.clear();
    flock.0.extend(vehicles.iter().map(|(transform, velocity)| {
        [
            transform.translation.extend(pack_speed_group(
                velocity.0.magnitude() / state.vehicle_max_speed.max(f32::EPSILON),
                0,
            )),
            Vec4::from(transform.rotation),
        ]
    }));
}

fn vehicle_spawner(
    mut commands: Commands,
    mut vehicle_spawner: ResMut<VehicleSpawner>,
    state: Res<GlobalState>,
) {
    while cpu_vehicle_count(&state) > vehicle_spawner.vehicle_count {
        commands
            .spawn(Transform::from_translation(
                (Vec3::new(rand::random(), rand::random(), rand::random()) - Vec3::splat(0.5))
                    * spawn_size(state.vehicle_count),
            ))
            .insert(Vehicle)
            .insert(VehicleVelocity(
                random_direction().mul(state.vehicle_max_speed),
            ))
            .insert(VehicleAcceleration(Vector3::zeros()))
            .insert(VehicleWanderRotation {
                theta: 0.0,
                phi: 0.0,
            });
        vehicle_spawner.vehicle_count += 1;
    }
}

type SteeringQuery<'w, 's> = Query<
    'w,
    's,
    (
        Entity,
        &'static VehicleVelocity,
        &'static mut VehicleAcceleration,
        &'static mut Transform,
        &'static mut VehicleWanderRotation,
    ),
    With<Vehicle>,
>;

// Boids algorithm
fn movement(
    mut vehicle_query: SteeringQuery,
    target_query: Query<(&Transform, &TargetSelection), (With<Target>, Without<Vehicle>)>,
    state: Res<GlobalState>,
    time: Res<Time>,
) {
    let (target_transform, selection) = target_query.single().unwrap();
    let mut wander = true;

    if selection.0 {
        wander = false;
    }

    if !wander {
        let limit_seek = state.vehicle_max_speed * state.vehicle_seek_factor;

        flock(&mut vehicle_query, &state);

        // Seek
        vehicle_query
            .par_iter_mut()
            .for_each(|(_, velocity, mut acceleration, transform, _)| {
                // Calculate force
                let mut force: Vector3<f32> =
                    (target_transform.translation - transform.translation).into();
                force = normalize_or_zero(force).mul(state.vehicle_max_speed);
                force -= velocity.0;

                // Limit force
                limit(&mut force, limit_seek);

                // Apply force
                acceleration.apply_force(force, state.vehicle_mass);
            });
    } else {
        let wander_delta = wander_step_bound(time.delta_secs());

        let limit_wander = state.vehicle_max_speed * state.vehicle_wander_factor;
        let limit_wall_avoid = state.vehicle_max_speed * state.vehicle_wall_avoid_factor;
        let walls = walls(state.vehicle_count);

        flock(&mut vehicle_query, &state);

        // Wander (random steering force within a Sphere)
        vehicle_query.par_iter_mut().for_each(
            |(_, velocity, mut acceleration, transform, mut wander_rotation)| {
                // Check if in bounds
                let fx = transform.translation.x < -walls.x || transform.translation.x > walls.x;
                let fy = transform.translation.y < -walls.y || transform.translation.y > walls.y;

                let fz = transform.translation.z < -walls.z || transform.translation.z > walls.z;

                if !fx && !fy && !fz {
                    let center = normalize_or_zero(velocity.0).mul(state.vehicle_wander_distance)
                        + Into::<Vector3<f32>>::into(transform.translation);
                    let mut rng = rand::rng();

                    let delta_theta = rng.random_range(-wander_delta..=wander_delta);
                    let delta_phi = rng.random_range(-wander_delta..=wander_delta);
                    wander_rotation.theta += delta_theta;
                    wander_rotation.phi += delta_phi;

                    let target = center
                        + Vector3::new(
                            wander_rotation.theta.cos() * wander_rotation.phi.sin(),
                            wander_rotation.theta.sin() * wander_rotation.phi.sin(),
                            wander_rotation.phi.cos(),
                        )
                        .mul(state.vehicle_wander_radius);

                    // Calculate force
                    let mut force: Vector3<f32> =
                        (target - Into::<Vector3<f32>>::into(transform.translation)).into();
                    force = normalize_or_zero(force).mul(state.vehicle_wander_speed);
                    force -= velocity.0;

                    // Limit force
                    limit(&mut force, limit_wander);

                    // Apply force
                    acceleration.apply_force(force, state.vehicle_mass);
                }

                // Calculate force
                let mut force = Vector3::new(
                    if fx {
                        if transform.translation.x < -walls.x {
                            state.vehicle_max_speed
                        } else {
                            -state.vehicle_max_speed
                        }
                    } else {
                        0.0
                    },
                    if fy {
                        if transform.translation.y < -walls.y {
                            state.vehicle_max_speed
                        } else {
                            -state.vehicle_max_speed
                        }
                    } else {
                        0.0
                    },
                    if fz {
                        if transform.translation.z < -walls.z {
                            state.vehicle_max_speed
                        } else {
                            -state.vehicle_max_speed
                        }
                    } else {
                        0.0
                    },
                );

                force -= velocity.0;

                // Limit force
                limit(&mut force, limit_wall_avoid);

                // Apply force
                acceleration.apply_force(force, state.vehicle_mass);
            },
        );
    }
}

// Half-extents of the box that wandering vehicles steer back into.
pub(crate) fn walls(count: usize) -> Vec3 {
    WORLD_SIZE * world_scale(count)
}

// Edge of the cube around the origin where new vehicles appear.
pub(crate) fn spawn_size(count: usize) -> f32 {
    SPAWN_SIZE * world_scale(count)
}

// Vehicles simulated as entities; GPU mode keeps its vehicles on the GPU instead.
fn cpu_vehicle_count(state: &GlobalState) -> usize {
    if state.mode == SimulationMode::Gpu {
        0
    } else {
        state.vehicle_count
    }
}

// Keeps density constant above REFERENCE_COUNT; smaller counts use the reference world.
pub(crate) fn world_scale(count: usize) -> f32 {
    (count as f32 / REFERENCE_COUNT).cbrt().max(1.0)
}

fn wander_step_bound(dt: f32) -> f32 {
    PI / 16.0 * (dt * REFERENCE_HZ).sqrt()
}

fn vehicle_cleanup(
    mut commands: Commands,
    mut vehicle_spawner: ResMut<VehicleSpawner>,
    state: Res<GlobalState>,
    mut query: Query<Entity, With<Vehicle>>,
) {
    let count = cpu_vehicle_count(&state);
    if count >= vehicle_spawner.vehicle_count {
        return;
    }

    for entity in &mut query {
        commands.entity(entity).despawn();
        vehicle_spawner.vehicle_count -= 1;
        if count >= vehicle_spawner.vehicle_count {
            break;
        }
    }
}

fn update(
    mut vehicle_query: Query<
        (
            Entity,
            &mut VehicleVelocity,
            &mut VehicleAcceleration,
            &mut Transform,
        ),
        With<Vehicle>,
    >,
    time: Res<Time>,
    state: Res<GlobalState>,
) {
    vehicle_query
        .par_iter_mut()
        .for_each(|(_, mut velocity, mut acceleration, mut transform)| {
            velocity.0 += acceleration.0 * (time.delta_secs() * REFERENCE_HZ);
            limit(&mut velocity.0, state.vehicle_max_speed);

            transform.translation.x += velocity.0.x * time.delta_secs();
            transform.translation.y += velocity.0.y * time.delta_secs();
            transform.translation.z += velocity.0.z * time.delta_secs();

            let direction = normalize_or_zero(velocity.0);
            if direction != Vector3::zeros() {
                transform.rotation = Quat::from_rotation_arc(Vec3::Y, direction.into());
            }

            acceleration.0 *= 0.0;
        });
}

// Uniform on the unit sphere, so the flock starts disordered.
fn random_direction() -> Vector3<f32> {
    let z: f32 = rand::random_range(-1.0..=1.0);
    let angle: f32 = rand::random_range(0.0..2.0 * PI);
    let radius = (1.0 - z * z).sqrt();
    Vector3::new(radius * angle.cos(), radius * angle.sin(), z)
}

fn normalize_or_zero(vector: Vector3<f32>) -> Vector3<f32> {
    vector
        .try_normalize(f32::EPSILON)
        .unwrap_or(Vector3::zeros())
}

fn limit(data: &mut Vector3<f32>, max: f32) {
    if data.magnitude_squared() > max * max {
        *data = data.normalize() * max;
    }
}

struct OctreeData {
    pub entity: Entity,
    pub transform: Transform,
    pub velocity: Vector3<f32>,
}

fn flock(vehicle_query: &mut SteeringQuery, state: &Res<GlobalState>) {
    if state.mode == SimulationMode::Octree {
        let Some((_, _, _, first_transform, _)) = vehicle_query.iter().next() else {
            return;
        };
        let mut min = first_transform.translation;
        let mut max = min;
        for (_, _, _, transform, _) in vehicle_query.iter() {
            min = min.min(transform.translation);
            max = max.max(transform.translation);
        }
        for axis in 0..3 {
            if min[axis] == max[axis] {
                let padding = 0.001_f32.max(min[axis].abs() * f32::EPSILON);
                min[axis] -= padding;
                max[axis] += padding;
            }
        }
        let center = min * 0.5 + max * 0.5;
        let mut octree = Octree::new(state.octree_size);
        octree.create_root(
            Point {
                x: center.x,
                y: center.y,
                z: center.z,
            },
            Point {
                x: min.x,
                y: min.y,
                z: min.z,
            },
            Point {
                x: max.x,
                y: max.y,
                z: max.z,
            },
        );
        let radius = state
            .vehicle_separation_distance
            .max(state.vehicle_alignment_distance)
            .max(state.vehicle_cohesion_distance)
            * state.vehicle_size;

        // Insert all vehicles into octree
        for (entity, velocity, _, transform, _) in &mut vehicle_query.iter() {
            octree.insert(
                Point {
                    x: transform.translation.x,
                    y: transform.translation.y,
                    z: transform.translation.z,
                },
                OctreeData {
                    entity,
                    transform: transform.clone(),
                    velocity: velocity.0,
                },
            );
        }

        // Find all neighbors for each vehicle
        vehicle_query.par_iter_mut().for_each(
            |(entity, velocity, mut acceleration, transform, _)| {
                let force = flocking_force(
                    transform.translation.into(),
                    velocity.0,
                    |visit| {
                        octree.for_each_in_radius(
                            &Point {
                                x: transform.translation.x,
                                y: transform.translation.y,
                                z: transform.translation.z,
                            },
                            radius,
                            |neighbor| {
                                if neighbor.entity != entity {
                                    visit(neighbor.transform.translation.into(), neighbor.velocity);
                                }
                            },
                        );
                    },
                    state,
                );
                acceleration.apply_force(force, state.vehicle_mass);
            },
        );

        return;
    }

    let others = vehicle_query
        .iter()
        .map(|(entity, velocity, _, transform, _)| (entity.index(), velocity.0, transform.clone()))
        .collect::<Vec<_>>();

    vehicle_query
        .par_iter_mut()
        .for_each(|(entity, velocity, mut acceleration, transform, _)| {
            let force = flocking_force(
                transform.translation.into(),
                velocity.0,
                |visit| {
                    for (other_index, other_velocity, other_transform) in &others {
                        if entity.index() != *other_index {
                            visit(other_transform.translation.into(), *other_velocity);
                        }
                    }
                },
                state,
            );
            acceleration.apply_force(force, state.vehicle_mass);
        });
}

fn flocking_force(
    position: Vector3<f32>,
    velocity: Vector3<f32>,
    neighbors: impl FnOnce(&mut dyn FnMut(Vector3<f32>, Vector3<f32>)),
    state: &GlobalState,
) -> Vector3<f32> {
    let separate_radius = state.vehicle_size * state.vehicle_separation_distance;
    let dist_separate = separate_radius.powi(2);
    let dist_align = (state.vehicle_size * state.vehicle_alignment_distance).powi(2);
    let dist_cohesion = (state.vehicle_size * state.vehicle_cohesion_distance).powi(2);
    let mut separate_sum = Vector3::zeros();
    let mut align_sum = Vector3::zeros();
    let mut cohesion_sum = Vector3::zeros();
    let mut align_count = 0;
    let mut cohesion_count = 0;

    neighbors(&mut |other_position, other_velocity| {
        let diff = position - other_position;
        let distance_squared = diff.magnitude_squared();

        if distance_squared > 0.0 && distance_squared < dist_separate {
            // Pushes grow as neighbors get closer and add up, so a crowd pushes back
            // harder than seek and cohesion can pull it together.
            let distance = distance_squared.sqrt();
            separate_sum +=
                diff / distance * (separate_radius / distance - 1.0).min(MAX_SEPARATION_PUSH);
        }
        if distance_squared < dist_align {
            align_sum += other_velocity;
            align_count += 1;
        }
        if distance_squared < dist_cohesion {
            cohesion_sum += other_position;
            cohesion_count += 1;
        }
    });

    separate_sum *= state.vehicle_max_speed * state.vehicle_separation_factor;
    if align_count > 0 {
        align_sum /= align_count as f32;
        align_sum = normalize_or_zero(align_sum) * state.vehicle_max_speed - velocity;
        limit(
            &mut align_sum,
            state.vehicle_max_speed * state.vehicle_alignment_factor,
        );
    }
    if cohesion_count > 0 {
        let offset = cohesion_sum / cohesion_count as f32 - position;
        let distance = offset.magnitude();
        cohesion_sum =
            normalize_or_zero(offset) * state.vehicle_max_speed * (distance / 10.0).min(1.0)
                - velocity;
        limit(
            &mut cohesion_sum,
            state.vehicle_max_speed * state.vehicle_cohesion_factor,
        );
    }

    separate_sum + align_sum + cohesion_sum
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::instancing::InstanceData;
    use rand::{rngs::StdRng, SeedableRng};
    use std::time::{Duration, Instant};

    #[test]
    #[ignore = "headless performance measurement; run in release with --nocapture --test-threads=1"]
    fn flocking_benchmark() {
        const SEED: u64 = 0xB01D5;
        const WARMUP: usize = 10;
        const STEPS: usize = 60;
        let dt = Duration::from_secs_f64(1.0 / 60.0);

        for count in [200, 1000, 2000, 5000, 10000] {
            let edge = SPAWN_SIZE * world_scale(count);
            let mut rng = StdRng::seed_from_u64(SEED);
            let vehicles: Vec<_> = (0..count)
                .map(|_| {
                    let position = (Vec3::new(rng.random(), rng.random(), rng.random())
                        - Vec3::splat(0.5))
                        * edge;
                    let z: f32 = rng.random_range(-1.0..=1.0);
                    let angle: f32 = rng.random_range(0.0..2.0 * PI);
                    let radius = (1.0 - z * z).sqrt();
                    let direction = Vector3::new(radius * angle.cos(), radius * angle.sin(), z);
                    (position, direction)
                })
                .collect();

            for use_octree in [false, true] {
                let mut app = App::new();
                app.insert_resource(cpu_state()).init_resource::<Time>();
                app.update();
                let (max_speed, octree_size) = {
                    let mut state = app.world_mut().resource_mut::<GlobalState>();
                    state.mode = mode_for(use_octree);
                    state.vehicle_count = count;
                    (state.vehicle_max_speed, state.octree_size)
                };
                if count == 200 && !use_octree {
                    let threads = bevy::tasks::ComputeTaskPool::get().thread_num();
                    println!(
                            "flocking_benchmark: seed={SEED:#x} threads={threads} octree_size={octree_size} dt={:.9}s warmup={WARMUP} steps={STEPS} (large naive cases capped at ~20s)",
                            dt.as_secs_f64()
                        );
                }
                app.add_systems(Update, (movement, update).chain());
                app.world_mut()
                    .spawn((Target, TargetSelection(false), Transform::default()));
                for &(position, direction) in &vehicles {
                    app.world_mut().spawn((
                        Vehicle,
                        VehicleVelocity(direction * max_speed),
                        VehicleAcceleration::default(),
                        VehicleWanderRotation::default(),
                        Transform::from_translation(position),
                    ));
                }
                for _ in 0..WARMUP {
                    app.world_mut().resource_mut::<Time>().advance_by(dt);
                    app.update();
                }
                let mut timings = Vec::with_capacity(STEPS);
                let mut elapsed = Duration::ZERO;
                for _ in 0..STEPS {
                    app.world_mut().resource_mut::<Time>().advance_by(dt);
                    let start = Instant::now();
                    app.update();
                    let duration = start.elapsed();
                    timings.push(duration.as_secs_f64() * 1000.0);
                    elapsed += duration;
                    if !use_octree && count >= 5000 && elapsed >= Duration::from_secs(20) {
                        break;
                    }
                }
                timings.sort_by(f64::total_cmp);
                let steps = timings.len();
                let median = (timings[(steps - 1) / 2] + timings[steps / 2]) / 2.0;
                let p95 = timings[(95 * steps).div_ceil(100) - 1];
                let mode = if use_octree { "octree" } else { "naive" };
                println!(
                    "count={count} mode={mode} median_ms={median:.3} p95_ms={p95:.3} steps={steps}"
                );
            }
        }
    }

    fn presentation_app(mode: SimulationMode) -> (App, Entity) {
        let mut app = crate::presentation::tests::clock_app();
        app.insert_resource(GlobalState {
            mode,
            vehicle_count: 1,
            ..default()
        })
        .add_systems(CpuStep, (movement, update).chain())
        .add_systems(
            Update,
            step_cpu.after(crate::presentation::prepare_simulation),
        );
        app.world_mut().spawn((
            Target,
            TargetSelection(true),
            Transform::from_xyz(10_000.0, 0.0, 0.0),
        ));
        let entity = app
            .world_mut()
            .spawn((
                Vehicle,
                VehicleVelocity::default(),
                VehicleAcceleration::default(),
                VehicleWanderRotation::default(),
                Transform::default(),
            ))
            .id();
        (app, entity)
    }

    #[test]
    fn presentation_speed_matches_simulated_duration_at_different_frame_rates() {
        for mode in [SimulationMode::Naive, SimulationMode::Octree] {
            let mut reference: Option<(Vec3, Vector3<f32>)> = None;
            for (fps, speed) in [
                (60, 1.0),
                (30, 0.5),
                (120, 2.0),
                (240, 4.0),
                (30, 4.0),
                (144, 0.25),
            ] {
                let (mut app, entity) = presentation_app(mode);
                app.world_mut()
                    .resource_mut::<crate::presentation::Presentation>()
                    .speed = speed;
                let frames = (fps as f32 / speed).ceil() as u64;
                for frame in 1..=frames {
                    app.world_mut().resource_mut::<Time<Real>>().advance_to(
                        Duration::from_secs_f64(
                            (frame as f64 / fps as f64).min(1.0 / speed as f64),
                        ),
                    );
                    app.update();
                }
                let position = app.world().get::<Transform>(entity).unwrap().translation;
                let velocity = app.world().get::<VehicleVelocity>(entity).unwrap().0;
                if let Some((p, v)) = reference {
                    assert!(
                        position.distance(p) < 1.5,
                        "{mode:?} {fps} FPS {speed}×: {position:?} vs {p:?}"
                    );
                    assert!(
                        (velocity - v).magnitude() < 1.0,
                        "{mode:?} {fps} FPS {speed}×: {velocity:?} vs {v:?}"
                    );
                } else {
                    reference = Some((position, velocity));
                }
                assert!(position.is_finite());
                assert!(velocity.magnitude() <= 80.0);
            }
        }
    }

    #[test]
    fn pause_preserves_all_cpu_state_bits_and_resume_uses_no_catch_up() {
        for mode in [SimulationMode::Naive, SimulationMode::Octree] {
            let (mut app, entity) = presentation_app(mode);
            app.world_mut()
                .resource_mut::<Time<Real>>()
                .advance_by(Duration::from_millis(16));
            app.update();
            let snapshot = |app: &App| {
                let t = app.world().get::<Transform>(entity).unwrap();
                let v = &app.world().get::<VehicleVelocity>(entity).unwrap().0;
                let a = &app.world().get::<VehicleAcceleration>(entity).unwrap().0;
                let w = app.world().get::<VehicleWanderRotation>(entity).unwrap();
                t.translation
                    .to_array()
                    .into_iter()
                    .chain(t.rotation.to_array())
                    .chain(v.iter().copied())
                    .chain(a.iter().copied())
                    .chain([w.theta, w.phi])
                    .map(f32::to_bits)
                    .collect::<Vec<_>>()
            };
            let before = snapshot(&app);
            app.world_mut()
                .resource_mut::<crate::presentation::Presentation>()
                .paused = true;
            for _ in 0..100 {
                app.world_mut()
                    .resource_mut::<Time<Real>>()
                    .advance_by(Duration::from_millis(100));
                app.update();
                assert_eq!(snapshot(&app), before);
            }
            app.world_mut()
                .resource_mut::<crate::presentation::Presentation>()
                .paused = false;
            app.world_mut()
                .resource_mut::<Time<Real>>()
                .advance_by(Duration::from_millis(16));
            app.update();
            assert_ne!(snapshot(&app), before);
            assert_eq!(
                app.world()
                    .resource::<crate::presentation::SimulationFrame>()
                    .steps,
                1
            );
            // The nested schedule must leave the outer clock untouched.
            assert_eq!(app.world().resource::<Time>().delta(), Duration::ZERO);
        }
    }

    #[test]
    fn constant_acceleration_is_frame_rate_independent() {
        let acceleration = Vector3::new(0.1, -0.2, 0.3);
        let expected = Vector3::new(6.0, -12.0, 18.0);
        for steps in [30_u64, 60, 120, 144] {
            let mut app = App::new();
            app.insert_resource(flocking_state())
                .init_resource::<Time>()
                .add_systems(Update, update);
            let entity = app
                .world_mut()
                .spawn((
                    Vehicle,
                    VehicleVelocity::default(),
                    VehicleAcceleration::default(),
                    Transform::default(),
                ))
                .id();
            for step in 1..=steps {
                app.world_mut()
                    .resource_mut::<Time>()
                    .advance_to(Duration::from_nanos(step * 1_000_000_000 / steps));
                app.world_mut()
                    .get_mut::<VehicleAcceleration>(entity)
                    .unwrap()
                    .0 = acceleration;
                app.update();
            }
            assert_eq!(
                app.world().resource::<Time>().elapsed(),
                Duration::from_secs(1)
            );
            let velocity = app.world().get::<VehicleVelocity>(entity).unwrap().0;
            assert!(
                (velocity - expected).magnitude() < 1e-4,
                "{steps} steps: {velocity:?}"
            );
        }
    }

    #[test]
    fn mass_slider_applies_to_existing_vehicles() {
        let mut app = App::new();
        app.insert_resource(GlobalState {
            vehicle_seek_factor: 7.0,
            ..flocking_state()
        })
        .init_resource::<Time>()
        .add_systems(Update, movement);
        app.world_mut().spawn((
            Target,
            TargetSelection(true),
            Transform::from_translation(Vec3::X * 100.0),
        ));
        let vehicle = app
            .world_mut()
            .spawn((
                Vehicle,
                VehicleVelocity::default(),
                VehicleAcceleration::default(),
                VehicleWanderRotation::default(),
                Transform::default(),
            ))
            .id();
        let mut accelerations = vec![];
        for mass in [1.0, 4.0] {
            app.world_mut().resource_mut::<GlobalState>().vehicle_mass = mass;
            app.world_mut()
                .get_mut::<VehicleAcceleration>(vehicle)
                .unwrap()
                .0 = Vector3::zeros();
            app.update();
            accelerations.push(app.world().get::<VehicleAcceleration>(vehicle).unwrap().0);
        }
        assert!(accelerations[0].x > 0.0);
        assert!((accelerations[0] - accelerations[1] * 4.0).magnitude() < 1e-4);
    }

    #[test]
    fn world_scale_keeps_density_above_the_reference_count() {
        assert_eq!(world_scale(0), 1.0);
        assert_eq!(world_scale(100), 1.0);
        assert_eq!(world_scale(200), 1.0);
        assert!((world_scale(1600) - 2.0).abs() < 1e-5);
        assert!((world_scale(25_000) - 5.0).abs() < 1e-4);
    }

    #[test]
    fn wander_step_bound_scales_with_square_root_of_time() {
        let reference = PI / 16.0;
        assert_eq!(wander_step_bound(1.0 / 60.0), reference);
        assert_eq!(wander_step_bound(0.0), 0.0);
        for (dt, scale) in [
            (1.0 / 30.0, 2.0_f32.sqrt()),
            (1.0 / 120.0, 0.5_f32.sqrt()),
            (1.0 / 144.0, (60.0_f32 / 144.0).sqrt()),
            (0.25, 15.0_f32.sqrt()),
        ] {
            assert!((wander_step_bound(dt) - reference * scale).abs() < 1e-6);
        }
    }

    #[test]
    fn large_time_step_with_large_force_stays_finite_and_within_max_speed() {
        let mut app = App::new();
        app.insert_resource(flocking_state())
            .init_resource::<Time>()
            .add_systems(Update, update);
        let mut acceleration = VehicleAcceleration::default();
        acceleration.apply_force(Vector3::new(1e6, -2e6, 3e6), 1.0);
        let entity = app
            .world_mut()
            .spawn((
                Vehicle,
                VehicleVelocity::default(),
                acceleration,
                Transform::default(),
            ))
            .id();
        app.world_mut()
            .resource_mut::<Time>()
            .advance_by(Duration::from_millis(250));
        app.update();
        let velocity = app.world().get::<VehicleVelocity>(entity).unwrap().0;
        let transform = app.world().get::<Transform>(entity).unwrap();
        assert!(velocity.iter().all(|value| value.is_finite()));
        assert!(
            velocity.magnitude() <= app.world().resource::<GlobalState>().vehicle_max_speed + 1e-3
        );
        assert!(transform.translation.is_finite());
        assert!(transform.rotation.is_finite());
    }

    fn cpu_state() -> GlobalState {
        GlobalState {
            mode: SimulationMode::Octree,
            vehicle_count: 200,
            ..default()
        }
    }

    fn mode_for(use_octree: bool) -> SimulationMode {
        if use_octree {
            SimulationMode::Octree
        } else {
            SimulationMode::Naive
        }
    }

    fn flocking_state() -> GlobalState {
        GlobalState {
            vehicle_count: 200,
            vehicle_size: 1.0,
            vehicle_max_speed: 80.0,
            vehicle_separation_factor: 1.5,
            vehicle_alignment_factor: 3.0,
            vehicle_cohesion_factor: 4.0,
            vehicle_separation_distance: 4.0,
            vehicle_alignment_distance: 30.0,
            vehicle_cohesion_distance: 20.0,
            vehicle_mass: 1.0,
            ..default()
        }
    }

    fn flock_once(vehicles: &[(Vec3, Vector3<f32>)], use_octree: bool) -> Vec<Vector3<f32>> {
        let mut app = App::new();
        app.insert_resource(GlobalState {
            mode: mode_for(use_octree),
            octree_size: 4,
            vehicle_size: 1.25,
            ..flocking_state()
        })
        .add_systems(
            Update,
            |mut query: SteeringQuery, state: Res<GlobalState>| flock(&mut query, &state),
        );
        let entities: Vec<_> = vehicles
            .iter()
            .map(|&(position, velocity)| {
                app.world_mut()
                    .spawn((
                        Vehicle,
                        VehicleVelocity(velocity),
                        VehicleAcceleration::default(),
                        Transform::from_translation(position),
                        VehicleWanderRotation::default(),
                    ))
                    .id()
            })
            .collect();
        app.update();
        entities
            .into_iter()
            .map(|entity| app.world().get::<VehicleAcceleration>(entity).unwrap().0)
            .collect()
    }

    #[test]
    fn octree_flocking_matches_naive_on_seeded_world() {
        let mut rng = StdRng::seed_from_u64(0xB01D5);
        let vehicles: Vec<_> = (0..384)
            .map(|i| {
                let position = match i % 4 {
                    0 => Vec3::new(
                        rng.random_range(-320.0..320.0),
                        rng.random_range(-320.0..320.0),
                        rng.random_range(-160.0..160.0),
                    ),
                    cluster => {
                        let center = match cluster {
                            1 => Vec3::new(240.0, -230.0, 120.0),
                            2 => Vec3::ZERO,
                            _ => Vec3::new(-40.0, 10.0, -5.0),
                        };
                        center
                            + Vec3::new(
                                rng.random_range(-2.0..2.0),
                                rng.random_range(-2.0..2.0),
                                rng.random_range(-2.0..2.0),
                            )
                    }
                };
                let velocity = Vector3::new(
                    rng.random_range(-40.0..40.0),
                    rng.random_range(-40.0..40.0),
                    rng.random_range(-40.0..40.0),
                );
                (position, velocity)
            })
            .collect();
        let naive = flock_once(&vehicles, false);
        let octree = flock_once(&vehicles, true);
        let max_difference = naive
            .iter()
            .zip(&octree)
            .flat_map(|(expected, actual)| {
                (0..3).map(move |axis| (actual[axis] - expected[axis]).abs())
            })
            .fold(0.0_f32, f32::max);
        println!("naive/octree max acceleration component difference: {max_difference}");
        for (i, (expected, actual)) in naive.iter().zip(&octree).enumerate() {
            for axis in 0..3 {
                // Spatial traversal changes summation order, especially for offset clusters.
                let tolerance = 1e-3_f32.max(expected.magnitude() * 1e-5);
                assert!(
                    (actual[axis] - expected[axis]).abs() <= tolerance,
                    "vehicle {i}, axis {axis}: naive={expected:?}, octree={actual:?}, tolerance={tolerance}"
                );
            }
        }
    }

    #[test]
    fn octree_flocking_handles_empty_and_degenerate_bounds() {
        assert!(flock_once(&[], true).is_empty());
        for count in [1, 32] {
            let vehicles = vec![(Vec3::new(240.0, -230.0, 120.0), Vector3::x()); count];
            assert_eq!(flock_once(&vehicles, true), flock_once(&vehicles, false));
        }
    }

    #[test]
    fn normalization_handles_zero_and_near_zero() {
        assert_eq!(normalize_or_zero(Vector3::zeros()), Vector3::zeros());
        assert_eq!(normalize_or_zero(Vector3::repeat(1e-10)), Vector3::zeros());
        assert_eq!(normalize_or_zero(Vector3::new(3.0, 0.0, 0.0)), Vector3::x());
    }

    #[test]
    fn coincident_neighbor_gives_finite_force() {
        let force = flocking_force(
            Vector3::zeros(),
            Vector3::x(),
            |visit| visit(Vector3::zeros(), Vector3::zeros()),
            &flocking_state(),
        );
        assert!(force.iter().all(|value| value.is_finite()));
        assert_eq!(force, Vector3::new(-2.0, 0.0, 0.0));
    }

    #[test]
    fn opposing_neighbor_velocities_give_finite_force() {
        let force = flocking_force(
            Vector3::zeros(),
            Vector3::zeros(),
            |visit| {
                visit(Vector3::x() * 5.0, Vector3::x());
                visit(Vector3::x() * 6.0, -Vector3::x());
            },
            &flocking_state(),
        );
        assert!(force.iter().all(|value| value.is_finite()));
    }

    #[test]
    fn vehicle_at_neighbors_center_gives_finite_force() {
        let force = flocking_force(
            Vector3::zeros(),
            Vector3::zeros(),
            |visit| {
                visit(Vector3::x(), Vector3::y());
                visit(-Vector3::x(), Vector3::y());
            },
            &flocking_state(),
        );
        assert!(force.iter().all(|value| value.is_finite()));
    }

    #[test]
    fn cohesion_arrives_at_half_speed_five_units_away() {
        let state = GlobalState {
            vehicle_separation_distance: 1.0,
            vehicle_alignment_distance: 1.0,
            vehicle_cohesion_factor: 10.0,
            ..flocking_state()
        };
        let force = flocking_force(
            Vector3::zeros(),
            Vector3::zeros(),
            |visit| visit(Vector3::x() * 5.0, Vector3::zeros()),
            &state,
        );
        assert_eq!(force, Vector3::x() * state.vehicle_max_speed * 0.5);
    }

    #[test]
    fn separation_grows_with_closeness_and_crowding() {
        let state = GlobalState {
            vehicle_alignment_distance: 0.01,
            vehicle_cohesion_distance: 0.01,
            ..flocking_state()
        };
        let separation = |distance: f32, count: usize| {
            flocking_force(
                Vector3::zeros(),
                Vector3::zeros(),
                |visit| {
                    for _ in 0..count {
                        visit(Vector3::x() * distance, Vector3::zeros());
                    }
                },
                &state,
            )
        };
        let unit = state.vehicle_max_speed * state.vehicle_separation_factor;
        assert_eq!(separation(2.0, 1), -Vector3::x() * unit);
        assert_eq!(separation(1.0, 1), -Vector3::x() * unit * 3.0);
        // Capped closer in than a quarter of the separation distance.
        assert_eq!(separation(0.1, 1), -Vector3::x() * unit * 3.0);
        assert_eq!(separation(1.0, 10), -Vector3::x() * unit * 30.0);
        assert_eq!(separation(4.5, 1), Vector3::zeros());
    }

    #[test]
    fn followers_keep_apart_around_the_target() {
        let mut app = App::new();
        app.insert_resource(cpu_state()).init_resource::<Time>();
        app.update();
        let count = 300;
        let max_speed = {
            let mut state = app.world_mut().resource_mut::<GlobalState>();
            state.vehicle_count = count;
            state.vehicle_max_speed
        };
        app.add_systems(Update, (movement, update).chain());
        app.world_mut()
            .spawn((Target, TargetSelection(true), Transform::default()));
        let mut rng = StdRng::seed_from_u64(7);
        let edge = SPAWN_SIZE * world_scale(count);
        for _ in 0..count {
            let position =
                (Vec3::new(rng.random(), rng.random(), rng.random()) - Vec3::splat(0.5)) * edge;
            let velocity = Vector3::new(
                rng.random_range(-1.0..1.0),
                rng.random_range(-1.0..1.0),
                rng.random_range(-1.0..1.0),
            ) * max_speed;
            app.world_mut().spawn((
                Vehicle,
                VehicleVelocity(velocity),
                VehicleAcceleration::default(),
                VehicleWanderRotation::default(),
                Transform::from_translation(position),
            ));
        }
        for _ in 0..600 {
            app.world_mut()
                .resource_mut::<Time>()
                .advance_by(Duration::from_secs_f32(1.0 / 60.0));
            app.update();
        }
        let points: Vec<Vec3> = app
            .world_mut()
            .query_filtered::<&Transform, With<Vehicle>>()
            .iter(app.world())
            .map(|transform| transform.translation)
            .collect();
        let crowded = points
            .iter()
            .enumerate()
            .filter(|&(i, a)| {
                points
                    .iter()
                    .enumerate()
                    .any(|(j, b)| i != j && a.distance(*b) < 1.4)
            })
            .count();
        let mean_distance = points.iter().map(|p| p.length()).sum::<f32>() / count as f32;
        // The old averaged separation left almost every follower overlapping a neighbor.
        assert!(crowded * 20 < count, "{crowded} of {count} overlap");
        assert!(
            mean_distance < 40.0,
            "flock is {mean_distance} from the target"
        );
    }

    fn gpu_device() -> (
        bevy::render::renderer::RenderDevice,
        bevy::render::renderer::RenderQueue,
    ) {
        use bevy::render::settings::{Backends, RenderResources, WgpuSettings};
        let RenderResources(device, queue, ..) =
            bevy::tasks::block_on(bevy::render::renderer::initialize_renderer(
                Backends::PRIMARY,
                None,
                &WgpuSettings::default(),
            ));
        (device, queue)
    }

    fn gpu_step(
        flock: &mut crate::gpu_flock::GpuFlock,
        device: &bevy::render::renderer::RenderDevice,
        queue: &bevy::render::renderer::RenderQueue,
        params: &crate::gpu_flock::GpuParams,
    ) {
        let mut encoder = device.create_command_encoder(&default());
        flock.encode(device, queue, &mut encoder, params);
        queue.submit([encoder.finish()]);
    }

    fn gpu_bench_values(name: &str, value: &str, min: u32, max: u32) -> Vec<u32> {
        value
            .split(',')
            .map(|item| {
                let parsed: u32 = item
                    .trim()
                    .parse()
                    .unwrap_or_else(|_| panic!("invalid {name}: {value}"));
                assert!(
                    (min..=max).contains(&parsed),
                    "{name} must be in {min}..={max}: {value}"
                );
                parsed
            })
            .collect()
    }

    fn gpu_bench_modes(value: &str) -> Vec<bool> {
        value
            .split(',')
            .map(|item| match item.trim() {
                "wander" => false,
                "follow" => true,
                _ => panic!("GPU_BENCH_MODES must contain wander or follow: {value}"),
            })
            .collect()
    }

    // Nearest-rank percentiles, with the usual midpoint median for even sample counts.
    fn gpu_bench_percentile(sorted: &[f64], percentile: usize) -> f64 {
        if sorted.is_empty() {
            return f64::NAN;
        }
        if percentile == 50 {
            return (sorted[(sorted.len() - 1) / 2] + sorted[sorted.len() / 2]) / 2.0;
        }
        sorted[(percentile * sorted.len()).div_ceil(100).max(1) - 1]
    }

    struct GpuBenchStats {
        sample_count: usize,
        nn_p1: f64,
        nn_p10: f64,
        nn_median: f64,
        close_percent: f64,
        mean_speed: f64,
        mean_target_distance: f64,
    }

    fn gpu_bench_stats(boids: &[crate::gpu_flock::BoidData], target: Vec3) -> GpuBenchStats {
        assert!(boids.len() >= 2);
        assert!(
            boids.iter().flatten().all(|v| v.is_finite()),
            "nonfinite GPU benchmark state"
        );
        let positions: Vec<_> = boids.iter().map(|boid| boid[0].truncate()).collect();
        let sample_count = boids.len().min(1000);
        // Fixed, evenly spaced indices cover the whole sorted buffer without replacement.
        // Each sampled boid is compared with every other boid, excluding itself.
        let mut nearest: Vec<_> = (0..sample_count)
            .map(|sample| {
                let index = sample * boids.len() / sample_count;
                positions
                    .iter()
                    .enumerate()
                    .filter(|&(other, _)| other != index)
                    .map(|(_, position)| positions[index].distance_squared(*position))
                    .fold(f32::INFINITY, f32::min)
                    .sqrt() as f64
            })
            .collect();
        nearest.sort_by(f64::total_cmp);
        GpuBenchStats {
            sample_count,
            nn_p1: gpu_bench_percentile(&nearest, 1),
            nn_p10: gpu_bench_percentile(&nearest, 10),
            nn_median: gpu_bench_percentile(&nearest, 50),
            close_percent: nearest.iter().filter(|&&distance| distance < 1.4).count() as f64
                / sample_count as f64
                * 100.0,
            mean_speed: boids
                .iter()
                .map(|boid| boid[1].truncate().length() as f64)
                .sum::<f64>()
                / boids.len() as f64,
            mean_target_distance: positions
                .iter()
                .map(|position| position.distance(target) as f64)
                .sum::<f64>()
                / boids.len() as f64,
        }
    }

    #[test]
    fn gpu_benchmark_parsing_and_statistics() {
        use crate::gpu_flock::BoidData;
        assert_eq!(
            gpu_bench_values("counts", "2, 30000,1000000", 2, 1_000_000),
            [2, 30_000, 1_000_000]
        );
        assert_eq!(
            gpu_bench_values("caps", "0,32,64,128", 0, u32::MAX),
            [0, 32, 64, 128]
        );
        assert_eq!(gpu_bench_modes("wander, follow"), [false, true]);
        for value in ["", "0", "-1", "2,", "x", "1000001"] {
            assert!(
                std::panic::catch_unwind(|| gpu_bench_values("counts", value, 2, 1_000_000))
                    .is_err()
            );
        }
        assert!(std::panic::catch_unwind(|| gpu_bench_modes("invalid")).is_err());
        assert!(gpu_bench_percentile(&[], 95).is_nan());
        assert_eq!(gpu_bench_percentile(&[1.0, 2.0, 3.0, 4.0], 50), 2.5);
        assert_eq!(gpu_bench_percentile(&[1.0, 2.0, 3.0, 4.0], 95), 4.0);
        let boids: Vec<BoidData> = [0.0, 1.0, 3.0, 11.0]
            .into_iter()
            .zip([3.0, 4.0, 5.0, 0.0])
            .map(|(x, speed)| {
                [
                    Vec3::new(x, 0.0, 0.0).extend(0.0),
                    Vec3::new(speed, 0.0, 0.0).extend(0.0),
                    Vec4::W,
                    Vec4::ZERO,
                ]
            })
            .collect();
        let stats = gpu_bench_stats(&boids, Vec3::ZERO);
        assert_eq!(stats.sample_count, 4);
        assert_eq!(stats.nn_p1, 1.0);
        assert_eq!(stats.nn_p10, 1.0);
        assert_eq!(stats.nn_median, 1.5);
        assert_eq!(stats.close_percent, 50.0);
        assert_eq!(stats.mean_speed, 3.0);
        assert_eq!(stats.mean_target_distance, 3.75);
        let mut boids: Vec<BoidData> = (0..1001)
            .map(|i| {
                [
                    Vec3::new(i as f32 * 10.0, 0.0, 0.0).extend(0.0),
                    Vec4::ZERO,
                    Vec4::W,
                    Vec4::ZERO,
                ]
            })
            .collect();
        // Index 1000 is not sampled, but must still be considered as index 0's neighbor.
        boids[1000][0].x = 0.25;
        let stats = gpu_bench_stats(&boids, Vec3::ZERO);
        assert_eq!(stats.sample_count, 1000);
        assert_eq!(stats.close_percent, 0.1);
    }

    #[test]
    #[ignore = "headless GPU measurement; run in release with --nocapture --test-threads=1"]
    fn gpu_flocking_benchmark() {
        use crate::gpu_flock::{gpu_params, GpuFlock};
        use bevy::render::render_resource::PollType;
        const WARMUP: usize = 600;
        const STEPS: usize = 60;
        const DT: f32 = 1.0 / 60.0;
        let counts = gpu_bench_values(
            "GPU_BENCH_COUNTS",
            &std::env::var("GPU_BENCH_COUNTS")
                .unwrap_or_else(|_| "10000,30000,100000,300000,1000000".into()),
            2,
            SimulationMode::Gpu.max_vehicles() as u32,
        );
        let caps = gpu_bench_values(
            "GPU_BENCH_CAPS",
            &std::env::var("GPU_BENCH_CAPS").unwrap_or_else(|_| "0,32,64,128".into()),
            0,
            u32::MAX,
        );
        let modes = gpu_bench_modes(
            &std::env::var("GPU_BENCH_MODES").unwrap_or_else(|_| "wander,follow".into()),
        );
        let (device, queue) = gpu_device();
        for count in counts {
            for &following in &modes {
                for &cap in &caps {
                    // Fresh state starts the shader's PCG spawn at frame 0 for every case.
                    let mut flock = GpuFlock::new(&device);
                    let state = GlobalState {
                        mode: SimulationMode::Gpu,
                        vehicle_count: count as usize,
                        gpu_max_neighbors: cap,
                        ..default()
                    };
                    let params = gpu_params(&state, Vec3::ZERO, following, DT);
                    let mut timings = Vec::with_capacity(STEPS);
                    let mut warmup_steps = 0;
                    let mut status = String::from("complete");
                    for step in 0..WARMUP + STEPS {
                        let start = Instant::now();
                        gpu_step(&mut flock, &device, &queue, &params);
                        device.poll(PollType::wait_indefinitely()).unwrap();
                        let elapsed = start.elapsed();
                        if step < WARMUP {
                            warmup_steps += 1;
                        } else {
                            timings.push(elapsed.as_secs_f64() * 1000.0);
                        }
                        if elapsed > Duration::from_millis(250) {
                            let phase = if step < WARMUP { "warmup" } else { "timed" };
                            status = format!(
                                "skipped_remaining phase={phase} slow_step_ms={:.3}",
                                elapsed.as_secs_f64() * 1000.0
                            );
                            break;
                        }
                    }
                    timings.sort_by(f64::total_cmp);
                    let stats = gpu_bench_stats(&flock.read_boids(&device, &queue), Vec3::ZERO);
                    let mode = if following { "follow" } else { "wander" };
                    let target_distance = if following {
                        format!("{:.3}", stats.mean_target_distance)
                    } else {
                        "NA".into()
                    };
                    println!(
                        "gpu_flocking_benchmark count={count} cap={cap} effective_cap={} mode={mode} spawn_frame=0 dt={DT:.9} warmup_steps={warmup_steps}/{WARMUP} timed_steps={}/{STEPS} median_ms={:.3} p95_ms={:.3} nn_sample={} nn_p1={:.3} nn_p10={:.3} nn_median={:.3} nn_lt1_4_pct={:.2} mean_speed={:.3} mean_target_distance={target_distance} status={status}",
                        params.max_neighbors, timings.len(), gpu_bench_percentile(&timings, 50), gpu_bench_percentile(&timings, 95),
                        stats.sample_count, stats.nn_p1, stats.nn_p10, stats.nn_median, stats.close_percent, stats.mean_speed,
                    );
                }
            }
        }
    }

    #[test]
    #[ignore = "needs a GPU"]
    fn gpu_steering_matches_cpu_reference() {
        use crate::gpu_flock::{gpu_params, GpuFlock};
        let count = 3000;
        let dt = 1.0 / 60.0;
        let target = Vec3::new(20.0, 30.0, -10.0);
        let mut rng = StdRng::seed_from_u64(0x6F0C);
        // A sparse region with negative coordinates and a dense cluster, so cells are both
        // empty and crowded.
        let vehicles: Vec<(Vec3, Vector3<f32>)> = (0..count)
            .map(|i| {
                let (center, edge) = if i % 2 == 0 {
                    (Vec3::new(-100.0, 50.0, -30.0), 300.0)
                } else {
                    (Vec3::new(60.0, -20.0, 10.0), 40.0)
                };
                let position = center
                    + (Vec3::new(rng.random(), rng.random(), rng.random()) - Vec3::splat(0.5))
                        * edge;
                let velocity = Vector3::new(
                    rng.random_range(-45.0..45.0),
                    rng.random_range(-45.0..45.0),
                    rng.random_range(-45.0..45.0),
                );
                (position, velocity)
            })
            .collect();

        let (device, queue) = gpu_device();
        for following in [true, false] {
            let mut state = GlobalState {
                mode: SimulationMode::Naive,
                vehicle_count: count,
                // A zero radius makes wander deterministic without changing its force code.
                vehicle_wander_radius: 0.0,
                gpu_max_neighbors: 0,
                ..default()
            };
            let mut app = App::new();
            app.insert_resource(GlobalState { ..state })
                .init_resource::<Time>()
                .add_systems(Update, (movement, update).chain());
            app.world_mut().spawn((
                Target,
                TargetSelection(following),
                Transform::from_translation(target),
            ));
            let mut boids = Vec::new();
            let entities: Vec<_> = vehicles
                .iter()
                .enumerate()
                .map(|(id, &(mut position, velocity))| {
                    // Exercise both signs of every wall, as well as the inside-wall drag.
                    if id < 6 {
                        let axis = id / 2;
                        position[axis] = walls(count)[axis] * if id % 2 == 0 { -1.1 } else { 1.1 };
                    } else if id < 8 {
                        position = Vec3::new(120.0, 0.0, 0.0);
                    } else if id < 10 {
                        // Exactly the alignment radius: strict distance tests must exclude it.
                        position = Vec3::new(300.0 + (id - 8) as f32 * 30.0, 0.0, 0.0);
                    }
                    boids.push([
                        position.extend(id as f32),
                        Vec3::from(velocity).extend(0.0),
                        Vec4::from(Quat::IDENTITY),
                        Vec4::ZERO,
                    ]);
                    app.world_mut()
                        .spawn((
                            Vehicle,
                            VehicleVelocity(velocity),
                            VehicleAcceleration::default(),
                            VehicleWanderRotation {
                                theta: id as f32,
                                phi: 0.0,
                            },
                            Transform::from_translation(position),
                        ))
                        .id()
                })
                .collect();
            let mut flock = GpuFlock::new(&device);
            flock.upload(&device, &queue, &boids);
            state.mode = SimulationMode::Gpu;
            for step in 0..3 {
                app.world_mut()
                    .resource_mut::<Time>()
                    .advance_by(Duration::from_secs_f32(dt));
                app.update();
                let expected: Vec<_> = entities
                    .iter()
                    .map(|&entity| {
                        let transform = app.world().get::<Transform>(entity).unwrap();
                        let velocity = app.world().get::<VehicleVelocity>(entity).unwrap();
                        (
                            transform.translation,
                            Vec3::from(velocity.0),
                            transform.rotation,
                        )
                    })
                    .collect();
                gpu_step(
                    &mut flock,
                    &device,
                    &queue,
                    &gpu_params(&state, target, following, dt),
                );
                let actual = flock.read_boids(&device, &queue);
                assert_eq!(actual.len(), count);
                let mut seen = vec![false; count];
                let mut worst = (0.0_f32, 0.0_f32);
                for boid in &actual {
                    assert!(boid.iter().all(|v| v.is_finite()), "{boid:?}");
                    let id = if following {
                        // Seek leaves theta alone, so it carries identity through the sort.
                        boid[0].w as usize
                    } else {
                        // Wander changes both angles; match its uniquely positioned outputs.
                        expected
                            .iter()
                            .enumerate()
                            .min_by(|(_, a), (_, b)| {
                                boid[0]
                                    .truncate()
                                    .distance_squared(a.0)
                                    .total_cmp(&boid[0].truncate().distance_squared(b.0))
                            })
                            .unwrap()
                            .0
                    };
                    assert!(!seen[id], "boid {id} appears twice");
                    seen[id] = true;
                    let (position, velocity, rotation) = expected[id];
                    worst.0 = worst.0.max(boid[0].truncate().distance(position));
                    worst.1 = worst.1.max(boid[1].truncate().distance(velocity));
                    let actual_rotation = Quat::from_vec4(boid[2]);
                    assert!(actual_rotation.dot(rotation).abs() > 1.0 - 1e-5);
                }
                println!("following={following}, step={step}: gpu/cpu worst position error {}, velocity error {}", worst.0, worst.1);
                assert!(worst.0 < 1e-3 && worst.1 < 1e-2, "{worst:?}");
            }
        }

        // A stationary boid keeps its previous orientation across repeated sorting/updates.
        let rotation = Vec4::from(Quat::from_rotation_x(0.5));
        let mut flock = GpuFlock::new(&device);
        flock.upload(
            &device,
            &queue,
            &[[Vec4::ZERO, Vec4::ZERO, rotation, Vec4::ZERO]],
        );
        let state = GlobalState {
            mode: SimulationMode::Gpu,
            vehicle_count: 1,
            ..default()
        };
        for _ in 0..2 {
            gpu_step(
                &mut flock,
                &device,
                &queue,
                &gpu_params(&state, Vec3::ZERO, true, dt),
            );
            assert_eq!(flock.read_boids(&device, &queue)[0][2], rotation);
        }
        for direction in [Vec3::Y, Vec3::NEG_Y, Vec3::new(0.001, 1.0, 0.0).normalize()] {
            flock.upload(
                &device,
                &queue,
                &[[Vec4::ZERO, direction.extend(0.0), rotation, Vec4::ZERO]],
            );
            gpu_step(
                &mut flock,
                &device,
                &queue,
                // dt = 0 means paused (no steering); a negligible step still steers.
                &gpu_params(&state, Vec3::ZERO, true, 1e-9),
            );
            let actual = Quat::from_vec4(flock.read_boids(&device, &queue)[0][2]);
            let expected = Quat::from_rotation_arc(Vec3::Y, direction);
            assert!(actual.dot(expected).abs() > 1.0 - 1e-6);
        }
    }

    #[test]
    #[ignore = "needs a GPU"]
    fn gpu_neighbor_cap_preserves_complete_separation() {
        use crate::gpu_flock::{gpu_params, GpuFlock};
        let state = GlobalState {
            mode: SimulationMode::Gpu,
            vehicle_count: 96,
            gpu_max_neighbors: 1,
            vehicle_mass: 10_000.0,
            vehicle_alignment_factor: 0.0,
            vehicle_cohesion_factor: 0.0,
            vehicle_seek_factor: 0.0,
            ..default()
        };
        // Many neighbors in one coarse cell, spanning negative/positive fine-cell boundaries.
        let positions: Vec<_> = (0..state.vehicle_count)
            .map(|i| {
                Vec3::new((i % 4) as f32, (i / 4 % 4) as f32, (i / 16) as f32) * 0.6
                    - Vec3::splat(1.1)
            })
            .collect();
        let boids: Vec<_> = positions
            .iter()
            .enumerate()
            .map(|(i, position)| [position.extend(i as f32), Vec4::ZERO, Vec4::W, Vec4::ZERO])
            .collect();
        let (device, queue) = gpu_device();
        let mut flock = GpuFlock::new(&device);
        flock.upload(&device, &queue, &boids);
        let dt = 1.0 / 60.0;
        gpu_step(
            &mut flock,
            &device,
            &queue,
            &gpu_params(&state, Vec3::ZERO, true, dt),
        );
        let actual = flock.read_boids(&device, &queue);
        let mut seen = vec![false; positions.len()];
        for boid in actual {
            let id = boid[0].w as usize;
            assert!(!seen[id]);
            seen[id] = true;
            let position = positions[id];
            let force = flocking_force(
                position.into(),
                Vector3::zeros(),
                |visit| {
                    for (other, point) in positions.iter().enumerate() {
                        if id != other {
                            visit((*point).into(), Vector3::zeros());
                        }
                    }
                },
                &state,
            );
            let mut velocity = force / state.vehicle_mass * (dt * REFERENCE_HZ);
            limit(&mut velocity, state.vehicle_max_speed);
            assert!(boid[1].truncate().distance(Vec3::from(velocity)) < 1e-3);
            assert!(
                boid[0]
                    .truncate()
                    .distance(position + Vec3::from(velocity) * dt)
                    < 1e-4
            );
        }
        assert!(seen.into_iter().all(|seen| seen));
    }

    #[test]
    #[ignore = "needs a GPU"]
    fn gpu_flock_spawns_grows_and_stays_bounded() {
        use crate::gpu_flock::{gpu_params, GpuFlock};
        let (device, queue) = gpu_device();
        let mut flock = GpuFlock::new(&device);
        let mut state = GlobalState::default();
        for (count, steps) in [(1000, 120), (5000, 60), (2000, 60)] {
            state.vehicle_count = count;
            let params = gpu_params(&state, Vec3::ZERO, false, 1.0 / 60.0);
            for _ in 0..steps {
                gpu_step(&mut flock, &device, &queue, &params);
            }
            let boids = flock.read_boids(&device, &queue);
            assert_eq!(boids.len(), count);
            let walls = walls(count);
            for boid in &boids {
                assert!(boid.iter().all(|v| v.is_finite()), "{boid:?}");
                assert!((boid[2].length_squared() - 1.0).abs() < 1e-5);
                assert!(boid[1].truncate().length() <= state.vehicle_max_speed + 1e-3);
                // Wall avoidance turns vehicles back within a few units.
                assert!(
                    (boid[0].truncate().abs() - walls).max_element() < 50.0,
                    "{boid:?} outside {walls}"
                );
            }
        }

        // Growth across 65,536 boids needs more than 256 scan blocks, exercising the
        // multi-block-per-thread path in scan_block_sums as well as buffer preservation.
        let boids: Vec<_> = (0..65_536)
            .map(|id| {
                [
                    Vec3::new(id as f32 * 64.0, -40.0, 10.0).extend((id + 1) as f32),
                    Vec4::ZERO,
                    Vec4::from(Quat::from_rotation_x(0.5)),
                    Vec4::ZERO,
                ]
            })
            .collect();
        flock.upload(&device, &queue, &boids);
        state.vehicle_count = boids.len() + 1;
        gpu_step(
            &mut flock,
            &device,
            &queue,
            &gpu_params(&state, Vec3::ZERO, true, 0.0),
        );
        let actual = flock.read_boids(&device, &queue);
        assert_eq!(actual.len(), state.vehicle_count);
        let mut seen = vec![false; actual.len()];
        for boid in actual {
            let id = boid[0].w as usize;
            assert!(!seen[id], "boid {id} appears twice after growth");
            seen[id] = true;
            if id == 0 {
                assert!(
                    boid[0].truncate().abs().max_element() <= spawn_size(state.vehicle_count) / 2.0
                );
            } else {
                assert_eq!(boid, boids[id - 1]);
            }
        }
        state.vehicle_count = 0;
        gpu_step(
            &mut flock,
            &device,
            &queue,
            &gpu_params(&state, Vec3::ZERO, true, 0.0),
        );
        assert!(flock.read_boids(&device, &queue).is_empty());
        state.vehicle_count = 17;
        gpu_step(
            &mut flock,
            &device,
            &queue,
            &gpu_params(&state, Vec3::ZERO, true, 0.0),
        );
        let fresh = flock.read_boids(&device, &queue);
        assert_eq!(fresh.len(), 17);
        assert!(fresh.iter().all(|boid| boid[0].w == 0.0));
    }

    #[test]
    fn gpu_mode_removes_cpu_entities_and_cpu_mode_respawns_them() {
        let mut app = App::new();
        app.insert_resource(cpu_state())
            .init_resource::<VehicleSpawner>()
            .add_systems(Update, (vehicle_spawner, vehicle_cleanup).chain());
        for (mode, count, expected) in [
            (SimulationMode::Naive, 23, 23),
            (SimulationMode::Gpu, 23, 0),
            (SimulationMode::Octree, 23, 23),
            (SimulationMode::Gpu, 0, 0),
            (SimulationMode::Naive, 7, 7),
        ] {
            let mut state = app.world_mut().resource_mut::<GlobalState>();
            state.mode = mode;
            state.vehicle_count = count;
            app.update();
            assert_eq!(
                app.world().resource::<VehicleSpawner>().vehicle_count,
                expected
            );
            assert_eq!(
                app.world_mut()
                    .query::<&Vehicle>()
                    .iter(app.world())
                    .count(),
                expected
            );
        }
    }

    #[test]
    fn paused_count_and_mode_changes_preserve_pause_and_retained_vehicles() {
        let mut app = crate::presentation::tests::clock_app();
        app.insert_resource(cpu_state())
            .init_resource::<VehicleSpawner>()
            .add_systems(CpuStep, (movement, update).chain())
            .add_systems(
                Update,
                (vehicle_spawner, vehicle_cleanup, step_cpu)
                    .chain()
                    .after(crate::presentation::prepare_simulation),
            );
        app.world_mut()
            .resource_mut::<crate::presentation::Presentation>()
            .paused = true;
        app.world_mut()
            .spawn((Target, TargetSelection(false), Transform::default()));
        let mut retained = std::collections::HashMap::new();
        for (mode, count) in [
            (SimulationMode::Naive, 23),
            (SimulationMode::Octree, 23),
            (SimulationMode::Octree, 30),
            (SimulationMode::Naive, 7),
            (SimulationMode::Gpu, 7),
            (SimulationMode::Octree, 12),
        ] {
            {
                let mut state = app.world_mut().resource_mut::<GlobalState>();
                state.mode = mode;
                state.vehicle_count = count;
            }
            app.world_mut()
                .resource_mut::<Time<Real>>()
                .advance_by(Duration::from_millis(100));
            app.update();
            let current: std::collections::HashMap<_, _> = app
                .world_mut()
                .query_filtered::<(Entity, &Transform, &VehicleVelocity), With<Vehicle>>()
                .iter(app.world())
                .map(|(e, t, v)| (e, (*t, v.0)))
                .collect();
            for (entity, value) in &current {
                if let Some(before) = retained.get(entity) {
                    assert_eq!(value, before);
                }
            }
            assert_eq!(
                current.len(),
                if mode == SimulationMode::Gpu {
                    0
                } else {
                    count
                }
            );
            assert!(
                app.world()
                    .resource::<crate::presentation::Presentation>()
                    .paused
            );
            retained = current;
        }
    }

    #[test]
    fn diagonal_velocity_is_limited_by_magnitude() {
        let mut velocity = Vector3::repeat(80.0);
        limit(&mut velocity, 80.0);
        assert!((velocity.magnitude() - 80.0).abs() <= 1e-3);
    }

    #[test]
    fn stationary_vehicle_keeps_its_rotation() {
        let mut app = App::new();
        app.insert_resource(flocking_state())
            .init_resource::<Time>()
            .add_systems(Update, update);
        let rotation = Quat::from_rotation_x(0.5);
        let entity = app
            .world_mut()
            .spawn((
                Vehicle,
                VehicleVelocity::default(),
                VehicleAcceleration::default(),
                Transform::from_rotation(rotation),
            ))
            .id();
        app.update();
        assert_eq!(
            app.world().get::<Transform>(entity).unwrap().rotation,
            rotation
        );
    }

    #[test]
    fn degenerate_vehicles_stay_finite_and_within_max_speed() {
        for use_octree in [false, true] {
            for following in [true, false] {
                let mut app = App::new();
                app.insert_resource(cpu_state()).init_resource::<Time>();
                app.update();
                {
                    let mut state = app.world_mut().resource_mut::<GlobalState>();
                    state.mode = mode_for(use_octree);
                    state.octree_size = 2;
                    state.vehicle_mass = 1.0;
                }
                app.add_systems(Update, (movement, update).chain());
                let position = Vec3::splat(10.0);
                app.world_mut().spawn((
                    Target,
                    TargetSelection(following),
                    Transform::from_translation(position),
                ));
                for velocity in [
                    Vector3::zeros(),
                    Vector3::repeat(80.0),
                    Vector3::repeat(-80.0),
                ] {
                    app.world_mut().spawn((
                        Vehicle,
                        VehicleVelocity(velocity),
                        VehicleAcceleration::default(),
                        VehicleWanderRotation::default(),
                        Transform::from_translation(position),
                    ));
                }
                for frame in 0..8 {
                    app.world_mut()
                        .resource_mut::<Time>()
                        .advance_by(std::time::Duration::from_secs_f32(1.0 / 60.0));
                    app.update();
                    let max_speed = app.world().resource::<GlobalState>().vehicle_max_speed;
                    for (transform, velocity) in app
                        .world_mut()
                        .query_filtered::<(&Transform, &VehicleVelocity), With<Vehicle>>()
                        .iter(app.world())
                    {
                        assert!(
                            transform.translation.is_finite(),
                            "translation: octree={use_octree}, following={following}, frame={frame}"
                        );
                        assert!(transform.rotation.is_finite());
                        assert!(velocity.0.iter().all(|value| value.is_finite()));
                        assert!(velocity.0.magnitude() <= max_speed + 1e-3);
                    }
                }
            }
        }
    }

    #[test]
    fn vehicles_spawn_in_the_spawn_cube_with_varied_headings() {
        let mut app = App::new();
        app.insert_resource(cpu_state())
            .init_resource::<VehicleSpawner>()
            .add_systems(Update, vehicle_spawner);
        app.update();
        let max_speed = app.world().resource::<GlobalState>().vehicle_max_speed;
        let mut negative_axes = [false; 3];
        for (transform, velocity) in app
            .world_mut()
            .query::<(&Transform, &VehicleVelocity)>()
            .iter(app.world())
        {
            assert!(transform.translation.abs().max_element() <= SPAWN_SIZE / 2.0);
            assert!((velocity.0.magnitude() - max_speed).abs() <= 1e-3);
            for (axis, negative) in negative_axes.iter_mut().enumerate() {
                *negative |= velocity.0[axis] < 0.0;
            }
        }
        assert_eq!(negative_axes, [true; 3]);
    }

    #[test]
    fn shape_switch_keeps_cpu_entities_transforms_velocities_and_follow_state() {
        use crate::{
            scene::Appearance,
            vehicle_appearance::{update_shape, VehicleShape},
        };
        let mut app = App::new();
        app.insert_resource(cpu_state())
            .init_resource::<VehicleSpawner>()
            .init_resource::<Assets<Mesh>>()
            .init_resource::<Appearance>()
            .init_resource::<crate::vehicle_model::VehicleModels>()
            .add_systems(Startup, spawn_flock)
            .add_systems(CpuStep, (movement, update).chain())
            .add_systems(
                Update,
                (vehicle_spawner, vehicle_cleanup, sync_instances).chain(),
            )
            .add_systems(PostUpdate, update_shape);
        let target = app
            .world_mut()
            .spawn((
                Target,
                TargetSelection(true),
                Transform::from_xyz(3.0, 5.0, 7.0),
            ))
            .id();
        app.update();
        let flock = app
            .world_mut()
            .query_filtered::<Entity, With<VehicleInstances>>()
            .single(app.world())
            .unwrap();
        let snapshot = |world: &mut World| {
            world
                .query_filtered::<(Entity, &Transform, &VehicleVelocity), With<Vehicle>>()
                .iter(world)
                .map(|(entity, transform, velocity)| (entity, *transform, velocity.0))
                .collect::<Vec<_>>()
        };
        let before = snapshot(app.world_mut());
        let state = app.world().resource::<GlobalState>().clone();
        let model = app
            .world_mut()
            .resource_mut::<Assets<Mesh>>()
            .add(VehicleShape::Bird.mesh());
        app.world_mut()
            .resource_mut::<crate::vehicle_model::VehicleModels>()
            .current = Some(model.clone());
        for shape in VehicleShape::ALL
            .into_iter()
            .chain([VehicleShape::Custom, VehicleShape::Dart])
        {
            app.world_mut().resource_mut::<Appearance>().vehicles.shape = shape;
            app.update();
            assert_eq!(snapshot(app.world_mut()), before);
            assert_eq!(app.world().resource::<GlobalState>(), &state);
            assert!(app.world().get::<TargetSelection>(target).unwrap().0);
            assert_eq!(
                app.world().get::<Mesh3d>(flock).unwrap().0,
                if shape == VehicleShape::Custom {
                    model.clone()
                } else {
                    app.world().resource::<VehicleMeshes>().get(shape)
                }
            );
            assert_eq!(
                app.world().get::<VehicleInstances>(flock).unwrap().0.len(),
                state.vehicle_count
            );
        }
        assert_eq!(app.world().resource::<Assets<Mesh>>().len(), 5);
    }

    #[test]
    fn instances_follow_vehicle_transforms_and_count() {
        let mut app = App::new();
        app.insert_resource(cpu_state())
            .init_resource::<VehicleSpawner>()
            .init_resource::<Time>()
            .add_systems(
                Update,
                (vehicle_spawner, vehicle_cleanup, update, sync_instances).chain(),
            );
        let flock = app.world_mut().spawn(VehicleInstances::default()).id();
        let sorted = |mut instances: Vec<InstanceData>| {
            instances.sort_by(|a, b| a[0].x.total_cmp(&b[0].x));
            instances
        };
        for count in [200, 3, 0, 5] {
            app.world_mut().resource_mut::<GlobalState>().vehicle_count = count;
            app.world_mut()
                .resource_mut::<Time>()
                .advance_by(Duration::from_secs_f32(1.0 / 60.0));
            app.update();
            let max_speed = app.world().resource::<GlobalState>().vehicle_max_speed;
            let expected: Vec<InstanceData> = app
                .world_mut()
                .query_filtered::<(&Transform, &VehicleVelocity), With<Vehicle>>()
                .iter(app.world())
                .map(|(transform, velocity)| {
                    [
                        transform.translation.extend(pack_speed_group(
                            velocity.0.magnitude() / max_speed.max(f32::EPSILON),
                            0,
                        )),
                        Vec4::from(transform.rotation),
                    ]
                })
                .collect();
            assert_eq!(expected.len(), count);
            // Integration ran before the sync: headings are set, not the identity.
            assert!(expected
                .iter()
                .all(|instance| instance[1] != Vec4::from(Quat::IDENTITY)));
            let instances = app
                .world()
                .get::<VehicleInstances>(flock)
                .unwrap()
                .0
                .clone();
            assert_eq!(sorted(instances), sorted(expected));
        }
    }
}
