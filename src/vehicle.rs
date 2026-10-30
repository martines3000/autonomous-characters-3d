extern crate nalgebra as na;

// Wall half-extents: wandering vehicles steer back inside.
const WORLD_SIZE: Vec3 = Vec3::new(200.0, 200.0, 100.0);
// Edge length of the cube around the origin where vehicles spawn.
const SPAWN_SIZE: f32 = 100.0;

use std::{f32::consts::PI, ops::Mul};

use crate::{
    octree::{Octree, Point},
    target::{Target, TargetSelection},
    GlobalState, RenderState,
};
use na::Vector3;

use bevy::prelude::*;
use rand::RngExt;

pub struct VehiclePlugin;

#[derive(Component)]
pub struct Vehicle;

#[derive(Component, Default)]
struct VehicleVelocity(pub Vector3<f32>);

#[derive(Component, Default)]
struct VehicleAcceleration(pub Vector3<f32>);

#[derive(Component, Default)]
struct VehicleMass(f32);

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
        app.init_resource::<VehicleSpawner>().add_systems(
            Update,
            (vehicle_spawner, vehicle_cleanup, movement, update).chain(),
        );
        // .add_system(benchmark);
    }
}

impl VehicleAcceleration {
    fn apply_force(&mut self, force: Vector3<f32>, mass: &VehicleMass) {
        self.0 += force / mass.0;
    }
}

fn vehicle_spawner(
    mut commands: Commands,
    mut vehicle_spawner: ResMut<VehicleSpawner>,
    state: Res<GlobalState>,
    render_state: Res<RenderState>,
) {
    while state.vehicle_count > vehicle_spawner.vehicle_count {
        // Cube at random position
        commands
            .spawn((
                WorldAssetRoot(render_state.vehicle_scene.clone()),
                Transform::from_translation(
                    (Vec3::new(rand::random(), rand::random(), rand::random()) - Vec3::splat(0.5))
                        * SPAWN_SIZE,
                ),
                if state.benchmark_mode {
                    Visibility::Hidden
                } else {
                    Visibility::Inherited
                },
            ))
            .insert(Vehicle)
            .insert(VehicleVelocity(
                random_direction().mul(state.vehicle_max_speed),
            ))
            .insert(VehicleAcceleration(Vector3::zeros()))
            .insert(VehicleMass(state.vehicle_mass))
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
        &'static VehicleMass,
        &'static mut VehicleWanderRotation,
    ),
    With<Vehicle>,
>;

// Boids algorithm
fn movement(
    mut vehicle_query: SteeringQuery,
    target_query: Query<(&Transform, &TargetSelection), (With<Target>, Without<Vehicle>)>,
    state: Res<GlobalState>,
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
        vehicle_query.par_iter_mut().for_each(
            |(_, velocity, mut acceleration, transform, mass, _)| {
                // Calculate force
                let mut force: Vector3<f32> =
                    (target_transform.translation - transform.translation).into();
                force = normalize_or_zero(force).mul(state.vehicle_max_speed);
                force -= velocity.0;

                // Limit force
                limit(&mut force, limit_seek);

                // Apply force
                acceleration.apply_force(force, mass);
            },
        );
    } else {
        let wander_delta = PI / 16.0;

        let limit_wander = state.vehicle_max_speed * state.vehicle_wander_factor;
        let limit_wall_avoid = state.vehicle_max_speed * state.vehicle_wall_avoid_factor;

        flock(&mut vehicle_query, &state);

        // Wander (random steering force within a Sphere)
        vehicle_query.par_iter_mut().for_each(
            |(_, velocity, mut acceleration, transform, mass, mut wander_rotation)| {
                // Check if in bounds
                let fx = transform.translation.x < -WORLD_SIZE.x
                    || transform.translation.x > WORLD_SIZE.x;
                let fy = transform.translation.y < -WORLD_SIZE.y
                    || transform.translation.y > WORLD_SIZE.y;

                let fz = transform.translation.z < -WORLD_SIZE.z
                    || transform.translation.z > WORLD_SIZE.z;

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
                    acceleration.apply_force(force, mass);
                }

                // Calculate force
                let mut force = Vector3::new(
                    if fx {
                        if transform.translation.x < -WORLD_SIZE.x {
                            state.vehicle_max_speed
                        } else {
                            -state.vehicle_max_speed
                        }
                    } else {
                        0.0
                    },
                    if fy {
                        if transform.translation.y < -WORLD_SIZE.y {
                            state.vehicle_max_speed
                        } else {
                            -state.vehicle_max_speed
                        }
                    } else {
                        0.0
                    },
                    if fz {
                        if transform.translation.z < -WORLD_SIZE.z {
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
                acceleration.apply_force(force, mass);
            },
        );
    }
}

fn vehicle_cleanup(
    mut commands: Commands,
    mut vehicle_spawner: ResMut<VehicleSpawner>,
    state: Res<GlobalState>,
    mut query: Query<Entity, With<Vehicle>>,
) {
    if state.vehicle_count >= vehicle_spawner.vehicle_count {
        return;
    }

    for entity in &mut query {
        commands.entity(entity).despawn();
        vehicle_spawner.vehicle_count -= 1;
        if state.vehicle_count >= vehicle_spawner.vehicle_count {
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
            velocity.0 += acceleration.0;
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

fn benchmark(time: Res<Time>, mut state: ResMut<GlobalState>) {
    if state.benchmark_mode {
        state.benchmark_current_results.push(time.delta_secs());

        // We check for 1010 because we want to skip the first 10 frames
        if state.benchmark_current_results.len() >= 110 {
            let mean = state.benchmark_current_results.iter().skip(10).sum::<f32>() / 100.0;
            state.benchmark_results.push(mean);
            state.benchmark_current_results.clear();
            state.benchmark_step += 1;

            println!(
                "Octree [{}]: {} {}",
                state.use_octree, state.vehicle_count, mean
            );

            if state.vehicle_count >= 100000 {
                if state.use_octree {
                    state.benchmark_mode = false;
                    state.vehicle_count = 0;
                }

                state.use_octree = !state.use_octree;
            }

            let step = if state.vehicle_count < 1000 {
                100
            } else if state.vehicle_count < 10000 {
                1000
            } else {
                10000
            };

            state.vehicle_count += step;
        }
    }
}

fn flock(vehicle_query: &mut SteeringQuery, state: &Res<GlobalState>) {
    if state.use_octree {
        let Some((_, _, _, first_transform, _, _)) = vehicle_query.iter().next() else {
            return;
        };
        let mut min = first_transform.translation;
        let mut max = min;
        for (_, _, _, transform, _, _) in vehicle_query.iter() {
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
            .vehicle_seperation_distance
            .max(state.vehicle_alignment_distance)
            .max(state.vehicle_cohesion_distance)
            * state.vehicle_size;

        // Insert all vehicles into octree
        for (entity, velocity, _, transform, _, _) in &mut vehicle_query.iter() {
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
            |(entity, velocity, mut acceleration, transform, mass, _)| {
                let neighbors = octree.query_radius(
                    &Point {
                        x: transform.translation.x,
                        y: transform.translation.y,
                        z: transform.translation.z,
                    },
                    radius,
                );

                let force = flocking_force(
                    transform.translation.into(),
                    velocity.0,
                    neighbors
                        .iter()
                        .filter(|neighbor| neighbor.entity != entity)
                        .map(|neighbor| (neighbor.transform.translation.into(), neighbor.velocity)),
                    state,
                );
                acceleration.apply_force(force, mass);
            },
        );

        return;
    }

    let others = vehicle_query
        .iter()
        .map(|(entity, velocity, _, transform, _, _)| {
            (entity.index(), velocity.0, transform.clone())
        })
        .collect::<Vec<_>>();

    vehicle_query.par_iter_mut().for_each(
        |(entity, velocity, mut acceleration, transform, mass, _)| {
            let force = flocking_force(
                transform.translation.into(),
                velocity.0,
                others
                    .iter()
                    .filter(|(other_index, _, _)| entity.index() != *other_index)
                    .map(|(_, other_velocity, other_transform)| {
                        (other_transform.translation.into(), *other_velocity)
                    }),
                state,
            );
            acceleration.apply_force(force, mass);
        },
    );
}

fn flocking_force(
    position: Vector3<f32>,
    velocity: Vector3<f32>,
    neighbors: impl Iterator<Item = (Vector3<f32>, Vector3<f32>)>,
    state: &GlobalState,
) -> Vector3<f32> {
    let dist_seperate = (state.vehicle_size * state.vehicle_seperation_distance).powi(2);
    let dist_align = (state.vehicle_size * state.vehicle_alignment_distance).powi(2);
    let dist_cohesion = (state.vehicle_size * state.vehicle_cohesion_distance).powi(2);
    let mut seperate_sum = Vector3::zeros();
    let mut align_sum = Vector3::zeros();
    let mut cohesion_sum = Vector3::zeros();
    let mut seperate_count = 0;
    let mut align_count = 0;
    let mut cohesion_count = 0;

    for (other_position, other_velocity) in neighbors {
        let diff = position - other_position;
        let distance_squared = diff.magnitude_squared();

        if distance_squared > 0.0 && distance_squared < dist_seperate {
            seperate_sum += diff / distance_squared;
            seperate_count += 1;
        }
        if distance_squared < dist_align {
            align_sum += other_velocity;
            align_count += 1;
        }
        if distance_squared < dist_cohesion {
            cohesion_sum += other_position;
            cohesion_count += 1;
        }
    }

    if seperate_count > 0 {
        seperate_sum /= seperate_count as f32;
        seperate_sum = normalize_or_zero(seperate_sum) * state.vehicle_max_speed - velocity;
        limit(
            &mut seperate_sum,
            state.vehicle_max_speed * state.vehicle_seperation_factor,
        );
    }
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

    seperate_sum + align_sum + cohesion_sum
}

#[cfg(test)]
mod tests {
    use super::*;
    use rand::{rngs::StdRng, SeedableRng};

    fn flocking_state() -> GlobalState {
        GlobalState {
            vehicle_size: 1.0,
            vehicle_max_speed: 80.0,
            vehicle_seperation_factor: 1.5,
            vehicle_alignment_factor: 3.0,
            vehicle_cohesion_factor: 4.0,
            vehicle_seperation_distance: 4.0,
            vehicle_alignment_distance: 30.0,
            vehicle_cohesion_distance: 20.0,
            ..default()
        }
    }

    fn flock_once(vehicles: &[(Vec3, Vector3<f32>, f32)], use_octree: bool) -> Vec<Vector3<f32>> {
        let mut app = App::new();
        app.insert_resource(GlobalState {
            use_octree,
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
            .map(|&(position, velocity, mass)| {
                app.world_mut()
                    .spawn((
                        Vehicle,
                        VehicleVelocity(velocity),
                        VehicleAcceleration::default(),
                        Transform::from_translation(position),
                        VehicleMass(mass),
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
                (position, velocity, rng.random_range(0.5..2.0))
            })
            .collect();
        let naive = flock_once(&vehicles, false);
        let octree = flock_once(&vehicles, true);
        for (i, (expected, actual)) in naive.iter().zip(&octree).enumerate() {
            for axis in 0..3 {
                let tolerance = 1e-4_f32.max(expected[axis].abs() * 1e-5);
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
            let vehicles = vec![(Vec3::new(240.0, -230.0, 120.0), Vector3::x(), 1.0); count];
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
            [(Vector3::zeros(), Vector3::zeros())].into_iter(),
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
            [
                (Vector3::x() * 5.0, Vector3::x()),
                (Vector3::x() * 6.0, -Vector3::x()),
            ]
            .into_iter(),
            &flocking_state(),
        );
        assert!(force.iter().all(|value| value.is_finite()));
    }

    #[test]
    fn vehicle_at_neighbors_center_gives_finite_force() {
        let force = flocking_force(
            Vector3::zeros(),
            Vector3::zeros(),
            [(Vector3::x(), Vector3::y()), (-Vector3::x(), Vector3::y())].into_iter(),
            &flocking_state(),
        );
        assert!(force.iter().all(|value| value.is_finite()));
    }

    #[test]
    fn cohesion_arrives_at_half_speed_five_units_away() {
        let state = GlobalState {
            vehicle_seperation_distance: 1.0,
            vehicle_alignment_distance: 1.0,
            vehicle_cohesion_factor: 10.0,
            ..flocking_state()
        };
        let force = flocking_force(
            Vector3::zeros(),
            Vector3::zeros(),
            [(Vector3::x() * 5.0, Vector3::zeros())].into_iter(),
            &state,
        );
        assert_eq!(force, Vector3::x() * state.vehicle_max_speed * 0.5);
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
                app.init_resource::<GlobalState>()
                    .init_resource::<Time>()
                    .add_systems(Startup, crate::configure_global_state);
                app.update();
                {
                    let mut state = app.world_mut().resource_mut::<GlobalState>();
                    state.use_octree = use_octree;
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
                        VehicleMass(1.0),
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
        app.init_resource::<GlobalState>()
            .init_resource::<RenderState>()
            .init_resource::<VehicleSpawner>()
            .add_systems(Startup, crate::configure_global_state)
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
    fn vehicle_count_changes_remove_scene_children() {
        let mut app = App::new();
        app.init_resource::<GlobalState>()
            .init_resource::<RenderState>()
            .init_resource::<VehicleSpawner>()
            .add_systems(Startup, crate::configure_global_state)
            .add_systems(Update, (vehicle_spawner, vehicle_cleanup).chain());
        app.update();
        assert_eq!(
            app.world_mut()
                .query::<&Vehicle>()
                .iter(app.world())
                .count(),
            200
        );
        let parents: Vec<_> = app
            .world_mut()
            .query_filtered::<Entity, With<Vehicle>>()
            .iter(app.world())
            .collect();
        let children: Vec<_> = parents
            .iter()
            .map(|&parent| app.world_mut().spawn(ChildOf(parent)).id())
            .collect();
        app.world_mut().resource_mut::<GlobalState>().vehicle_count = 0;
        app.update();
        assert_eq!(
            app.world_mut()
                .query::<&Vehicle>()
                .iter(app.world())
                .count(),
            0
        );
        assert!(children
            .iter()
            .all(|&child| app.world().get_entity(child).is_err()));
        app.world_mut().resource_mut::<GlobalState>().vehicle_count = 3;
        app.update();
        assert_eq!(
            app.world_mut()
                .query::<&Vehicle>()
                .iter(app.world())
                .count(),
            3
        );
    }
}
