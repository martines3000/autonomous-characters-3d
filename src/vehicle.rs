extern crate nalgebra as na;

// const WORLD_SIZE: Vec3 = Vec3::new(200.0, 200.0, 100.0);
const WORLD_SIZE: Vec3 = Vec3::new(2000.0, 2000.0, 1000.0);

use std::{
    f32::consts::PI,
    ops::{Div, Mul},
};

use crate::{
    octree::{Octree, Point},
    target::Target,
    GlobalState, RenderState,
};
use bevy_mod_picking::Selection;
use na::{SimdPartialOrd, Vector3};

use bevy::prelude::*;
use rand::Rng;

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
        app.init_resource::<VehicleSpawner>()
            .add_system(vehicle_spawner)
            .add_system(vehicle_cleanup)
            .add_system(movement)
            .add_system(update);
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
    meshes: Res<Assets<Mesh>>,
    materials: Res<Assets<StandardMaterial>>,
    mut vehicle_spawner: ResMut<VehicleSpawner>,
    state: Res<GlobalState>,
    render_state: Res<RenderState>,
    assets_server: Res<AssetServer>,
) {
    while state.vehicle_count > vehicle_spawner.vehicle_count {
        // Cube at random position
        commands
            .spawn(SceneBundle {
                // scene: assets_server.load("cone.glb#Scene0"),
                scene: render_state.vehicle_scene.clone(),
                transform: Transform::from_translation(Vec3::new(
                    -WORLD_SIZE.x / 2.0 + rand::random::<f32>() * WORLD_SIZE.x,
                    -WORLD_SIZE.y / 2.0 + rand::random::<f32>() * WORLD_SIZE.y,
                    -WORLD_SIZE.z / 2.0 + rand::random::<f32>() * WORLD_SIZE.z,
                )),
                visibility: Visibility {
                    is_visible: if state.benchmark_mode { false } else { true },
                },
                ..Default::default()
            })
            .insert(Vehicle)
            .insert(VehicleVelocity(
                Vector3::new(
                    rand::random::<f32>() * state.vehicle_max_speed,
                    rand::random::<f32>() * state.vehicle_max_speed,
                    rand::random::<f32>() * state.vehicle_max_speed,
                )
                .normalize()
                .mul(state.vehicle_max_speed),
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

// Boids algorithm
fn movement(
    mut vehicle_query: Query<
        (
            Entity,
            &VehicleVelocity,
            &mut VehicleAcceleration,
            &mut Transform,
            &VehicleMass,
            &mut VehicleWanderRotation,
        ),
        With<Vehicle>,
    >,
    target_query: Query<(&Transform, &Selection), (With<Target>, Without<Vehicle>)>,
    state: Res<GlobalState>,
) {
    let (target_transform, selection) = target_query.get_single().unwrap();
    let mut wander = true;

    if selection.selected() {
        wander = false;
    }

    if !wander {
        let limit_seek = state.vehicle_max_speed * state.vehicle_seek_factor;

        flock(&mut vehicle_query, &state);

        // Seek
        vehicle_query.par_for_each_mut(
            64,
            |(_, velocity, mut acceleration, transform, mass, _)| {
                // Calculate force
                let mut force: Vector3<f32> =
                    (target_transform.translation - transform.translation).into();
                force = force.normalize().mul(state.vehicle_max_speed);
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
        vehicle_query.par_for_each_mut(
            64,
            |(_, velocity, mut acceleration, transform, mass, mut wander_rotation)| {
                // Check if in bounds
                let fx = transform.translation.x < -WORLD_SIZE.x
                    || transform.translation.x > WORLD_SIZE.x;
                let fy = transform.translation.y < -WORLD_SIZE.y
                    || transform.translation.y > WORLD_SIZE.y;

                let fz = transform.translation.z < -WORLD_SIZE.z
                    || transform.translation.z > WORLD_SIZE.z;

                if !fx && !fy && !fz {
                    let center = velocity.0.normalize().mul(state.vehicle_wander_distance)
                        + Into::<Vector3<f32>>::into(transform.translation);
                    let mut rng = rand::thread_rng();

                    let delta_theta = rng.gen_range(-wander_delta..=wander_delta);
                    let delta_phi = rng.gen_range(-wander_delta..=wander_delta);
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
                    force = force.normalize().mul(state.vehicle_wander_speed);
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
        commands.entity(entity).despawn_recursive();
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
    let min = Vector3::from_element(state.vehicle_max_speed * -1.0);
    let max = Vector3::from_element(state.vehicle_max_speed);

    vehicle_query.par_for_each_mut(64, |(_, mut velocity, mut acceleration, mut transform)| {
        velocity.0 = (velocity.0 + acceleration.0).simd_clamp(min, max);

        transform.translation.x += velocity.0.x * time.delta_seconds();
        transform.translation.y += velocity.0.y * time.delta_seconds();
        transform.translation.z += velocity.0.z * time.delta_seconds();

        transform.rotation = Quat::from_rotation_arc(
            Vec3::new(0.0, 1.0, 0.0).into(),
            velocity.0.normalize().into(),
        );

        acceleration.0 *= 0.0;
    });
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
        state.benchmark_current_results.push(time.delta_seconds());

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

fn flock(
    vehicle_query: &mut Query<
        (
            Entity,
            &VehicleVelocity,
            &mut VehicleAcceleration,
            &mut Transform,
            &VehicleMass,
            &mut VehicleWanderRotation,
        ),
        With<Vehicle>,
    >,
    state: &Res<GlobalState>,
) {
    let limit_seperate = state.vehicle_max_speed * state.vehicle_seperation_factor;
    let limit_align = state.vehicle_max_speed * state.vehicle_alignment_factor;
    let limit_cohesion = state.vehicle_max_speed * state.vehicle_cohesion_factor;
    let dist_seperate = (state.vehicle_size * state.vehicle_seperation_distance).powi(2);
    let dist_align = (state.vehicle_size * state.vehicle_alignment_distance).powi(2);
    let dist_cohesion = (state.vehicle_size * state.vehicle_cohesion_distance).powi(2);

    if state.use_octree {
        let mut octree = Octree::new(state.octree_size);
        octree.create_root(
            Point {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            Point {
                x: -WORLD_SIZE.x,
                y: -WORLD_SIZE.y,
                z: -WORLD_SIZE.z,
            },
            Point {
                x: WORLD_SIZE.x,
                y: WORLD_SIZE.y,
                z: WORLD_SIZE.z,
            },
        );

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
        vehicle_query.par_for_each_mut(
            64,
            |(entity, velocity, mut acceleration, transform, mass, _)| {
                let mut seperate_sum = Vector3::new(0.0, 0.0, 0.0);
                let mut align_sum = Vector3::new(0.0, 0.0, 0.0);
                let mut cohesion_sum = Vector3::new(0.0, 0.0, 0.0);
                let mut seperate_count = 0;
                let mut align_count = 0;
                let mut cohesion_count = 0;

                let neighbors = octree.find_neighbors(&Point {
                    x: transform.translation.x,
                    y: transform.translation.y,
                    z: transform.translation.z,
                });

                neighbors.iter().for_each(|neighbor| {
                    let (other_entity, other_transform, other_velocity) =
                        (neighbor.entity, &neighbor.transform, &neighbor.velocity);

                    if entity == other_entity {
                        return;
                    }

                    let distance: f32 = Into::<Vector3<f32>>::into(
                        other_transform.translation - transform.translation,
                    )
                    .magnitude_squared();

                    if distance < dist_seperate {
                        let mut diff = Into::<Vector3<f32>>::into(
                            transform.translation - other_transform.translation,
                        );
                        diff = diff.normalize().div(distance.sqrt());
                        seperate_sum += diff;
                        seperate_count += 1;
                    }

                    if distance < dist_align {
                        align_sum += other_velocity;
                        align_count += 1;
                    }

                    if distance < dist_cohesion {
                        cohesion_sum += Into::<Vector3<f32>>::into(other_transform.translation);
                        cohesion_count += 1;
                    }
                });

                //Apply seperation
                if seperate_count > 0 {
                    seperate_sum /= seperate_count as f32;
                    seperate_sum = seperate_sum.normalize() * state.vehicle_max_speed;
                    seperate_sum -= velocity.0;
                    limit(&mut seperate_sum, limit_seperate);
                    acceleration.apply_force(seperate_sum, mass);
                }

                // Apply alignment
                if align_count > 0 {
                    align_sum /= align_count as f32;
                    align_sum = align_sum.normalize() * state.vehicle_max_speed;
                    align_sum -= velocity.0;
                    limit(&mut align_sum, limit_align);
                    acceleration.apply_force(align_sum, mass);
                }

                // Apply cohesion
                if cohesion_count > 0 {
                    cohesion_sum /= cohesion_count as f32;
                    cohesion_sum = cohesion_sum - Into::<Vector3<f32>>::into(transform.translation);
                    cohesion_sum = cohesion_sum
                        .try_normalize(f32::MIN)
                        .unwrap_or(Vector3::zeros());

                    let dist = cohesion_sum.magnitude();

                    if dist < 10.0 {
                        cohesion_sum *= dist / 10.0;
                    }

                    cohesion_sum *= state.vehicle_max_speed;
                    cohesion_sum -= velocity.0;
                    limit(&mut cohesion_sum, limit_cohesion);
                    acceleration.apply_force(cohesion_sum, mass);
                }
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

    vehicle_query.par_for_each_mut(
        64,
        |(entity, velocity, mut acceleration, transform, mass, _)| {
            let mut seperate_sum = Vector3::new(0.0, 0.0, 0.0);
            let mut align_sum = Vector3::new(0.0, 0.0, 0.0);
            let mut cohesion_sum = Vector3::new(0.0, 0.0, 0.0);
            let mut seperate_count = 0;
            let mut align_count = 0;
            let mut cohesion_count = 0;

            others
                .iter()
                .for_each(|(other_index, other_velocity, other_transform)| {
                    if entity.index() == *other_index {
                        return;
                    }

                    let distance: f32 = Into::<Vector3<f32>>::into(
                        other_transform.translation - transform.translation,
                    )
                    .magnitude_squared();

                    if distance < dist_seperate {
                        let mut diff = Into::<Vector3<f32>>::into(
                            transform.translation - other_transform.translation,
                        );
                        diff = diff.normalize().div(distance.sqrt());
                        seperate_sum += diff;
                        seperate_count += 1;
                    }

                    if distance < dist_align {
                        align_sum += other_velocity;
                        align_count += 1;
                    }

                    if distance < dist_cohesion {
                        cohesion_sum += Into::<Vector3<f32>>::into(other_transform.translation);
                        cohesion_count += 1;
                    }
                });

            //Apply seperation
            if seperate_count > 0 {
                seperate_sum /= seperate_count as f32;
                seperate_sum = seperate_sum.normalize() * state.vehicle_max_speed;
                seperate_sum -= velocity.0;
                limit(&mut seperate_sum, limit_seperate);
                acceleration.apply_force(seperate_sum, mass);
            }

            // Apply alignment
            if align_count > 0 {
                align_sum /= align_count as f32;
                align_sum = align_sum.normalize() * state.vehicle_max_speed;
                align_sum -= velocity.0;
                limit(&mut align_sum, limit_align);
                acceleration.apply_force(align_sum, mass);
            }

            // Apply cohesion
            if cohesion_count > 0 {
                cohesion_sum /= cohesion_count as f32;
                cohesion_sum = cohesion_sum - Into::<Vector3<f32>>::into(transform.translation);
                cohesion_sum = cohesion_sum
                    .try_normalize(f32::MIN)
                    .unwrap_or(Vector3::zeros());

                let dist = cohesion_sum.magnitude();

                if dist < 10.0 {
                    cohesion_sum *= dist / 10.0;
                }

                cohesion_sum *= state.vehicle_max_speed;
                cohesion_sum -= velocity.0;
                limit(&mut cohesion_sum, limit_cohesion);
                acceleration.apply_force(cohesion_sum, mass);
            }
        },
    );
}
