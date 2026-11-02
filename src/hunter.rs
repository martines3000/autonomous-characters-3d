//! A small, optional population. CPU and GPU use the same pursuit/pass contract.
use crate::{
    swarm::Group,
    vehicle::{self, Vehicle, VehicleAcceleration, VehicleMetadata, VehicleVelocity},
    GlobalState, SimulationMode,
};
use bevy::{
    camera::visibility::NoFrustumCulling,
    prelude::*,
    render::{
        extract_component::{ExtractComponent, ExtractComponentPlugin},
        extract_resource::{ExtractResource, ExtractResourcePlugin},
        RenderApp,
    },
};

pub const MAX_HUNTERS: usize = 16;
pub const FLEE_RADIUS: f32 = 24.0;
pub const HUNT_RADIUS: f32 = 180.0;
pub const LUNGE_RADIUS: f32 = 32.0;
pub const LUNGE_TIME: f32 = 0.45;
pub const COOLDOWN: f32 = 0.8;
pub const PANIC_TIME: f32 = 0.65;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum HunterModel {
    #[default]
    Shark,
    Falcon,
    Tuna,
}
impl HunterModel {
    pub fn path(self) -> &'static str {
        match self {
            Self::Shark => "assets/models/shark.glb",
            Self::Falcon => "assets/models/falcon.glb",
            Self::Tuna => "assets/models/tuna.glb",
        }
    }
    pub fn code(self) -> f32 {
        match self {
            Self::Shark => 1.0,
            Self::Falcon => 2.0,
            Self::Tuna => 3.0,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Hunter {
    pub position: Vec3,
    pub velocity: Vec3,
    pub rotation: Quat,
    // Positive: committed lunge. Negative: recovery. Zero: ready to pursue.
    pub timer: f32,
    pub group: u32,
}
impl Hunter {
    pub fn records(self) -> [Vec4; 3] {
        [
            self.position.extend(self.timer),
            self.velocity.extend(self.group as f32),
            Vec4::from(self.rotation),
        ]
    }
    pub fn instance(self, group: Group, speed: f32, phase: f32) -> crate::instancing::InstanceData {
        [
            self.position
                .extend(crate::vehicle_appearance::pack_speed_group(
                    self.velocity.length() / speed.max(f32::EPSILON),
                    self.group as u8,
                )),
            Vec4::from(self.rotation),
            Vec4::new(phase, group.size_scale, group.hunter_model.code(), 0.0),
        ]
    }
}

pub fn initial(configs: &[(u32, Group)], scale: f32) -> Vec<Hunter> {
    configs
        .iter()
        .enumerate()
        .map(|(i, (group, _))| {
            let angle = i as f32 * std::f32::consts::TAU / configs.len().max(1) as f32;
            let position = Vec3::new(angle.cos(), angle.sin(), 0.15) * (45.0 * scale);
            let velocity = -position.normalize() * 40.0;
            Hunter {
                position,
                velocity,
                rotation: crate::motion::initial(velocity),
                timer: 0.0,
                group: *group,
            }
        })
        .collect()
}

pub fn limit(value: Vec3, max: f32) -> Vec3 {
    value.clamp_length_max(max)
}

/// Per-substep physical settings, expressed in world units and simulated seconds.
#[derive(Clone, Copy)]
pub struct HuntStep {
    pub speed: f32,
    pub scale: f32,
    pub dt: f32,
    pub bounds: Vec3,
}

/// Exact nearest prey inside the hunt radius; a committed pass never retargets.
pub fn advance(
    me: Hunter,
    others: &[Hunter],
    prey: &[(u32, Vec3, Vec3)],
    index: usize,
    step: HuntStep,
) -> Hunter {
    let HuntStep {
        speed,
        scale,
        dt,
        bounds,
    } = step;
    let mut timer = me.timer;
    let mut desired = me.velocity.normalize_or_zero() * speed;
    let mut nearest = None;
    let mut best = (HUNT_RADIUS * scale).powi(2);
    let mut nearest_id = u32::MAX;
    for &(id, position, velocity) in prey {
        let distance = me.position.distance_squared(position);
        if distance < best || (distance == best && nearest.is_some() && id < nearest_id) {
            nearest_id = id;
            best = distance;
            nearest = Some((position, velocity));
        }
    }
    if timer > 0.0 {
        timer -= dt;
        desired *= 1.8;
        if timer <= 0.0 {
            timer = -COOLDOWN;
        }
    } else if timer < 0.0 {
        timer = (timer + dt).min(0.0);
    } else if let Some((position, velocity)) = nearest {
        // A small lead aims across the school rather than following its wake.
        desired = (position + velocity * 0.12 - me.position).normalize_or_zero() * speed;
        if best < (LUNGE_RADIUS * scale).powi(2) {
            timer = LUNGE_TIME;
            desired *= 1.8;
        }
    } else {
        let phase = index as f32 * 1.7;
        desired = (me.velocity
            + Vec3::new(
                (me.position.y / scale * 0.03 + phase).sin(),
                (me.position.z / scale * 0.03 + phase).cos(),
                (me.position.x / scale * 0.03 + phase).sin(),
            ) * (speed * 0.15))
            .normalize_or_zero()
            * speed;
    }
    let mut separation = Vec3::ZERO;
    for (i, other) in others.iter().enumerate() {
        if i == index {
            continue;
        }
        let offset = me.position - other.position;
        let distance = offset.length();
        if distance > 0.0 && distance < 12.0 * scale {
            separation += offset / distance * (1.0 - distance / (12.0 * scale));
        }
    }
    for axis in 0..3 {
        if me.position[axis] < -bounds[axis] {
            desired[axis] = speed;
        } else if me.position[axis] > bounds[axis] {
            desired[axis] = -speed;
        }
    }
    let acceleration =
        limit(desired - me.velocity, speed * 3.0) + limit(separation, 1.0) * speed * 2.0;
    let velocity = limit(
        me.velocity + acceleration * dt,
        speed * if timer > 0.0 { 1.8 } else { 1.0 },
    );
    let rotation = crate::motion::initial(velocity);
    Hunter {
        position: me.position + velocity * dt,
        velocity,
        rotation,
        timer,
        ..me
    }
}

pub fn flee(position: Vec3, hunters: &[Hunter], scale: f32, speed: f32) -> (Vec3, bool) {
    let mut sum = Vec3::ZERO;
    let mut threatened = false;
    for hunter in hunters {
        let offset = position - hunter.position;
        let distance = offset.length();
        let weight = 1.0 - (distance / (FLEE_RADIUS * scale)).clamp(0.0, 1.0);
        threatened |= weight > 0.0;
        let normal = if distance > 0.0 {
            offset / distance
        } else {
            Vec3::X
        };
        sum += normal * (weight * weight * (3.0 - 2.0 * weight));
    }
    (limit(sum, 1.0) * (speed * 8.0), threatened)
}

#[derive(Resource, ExtractResource, Clone, Default)]
#[extract_app(RenderApp)]
pub struct Hunters {
    pub boids: Vec<Hunter>,
    pub previous: Vec<Hunter>,
    pub configs: Vec<(u32, Group)>,
    pub instances: Vec<crate::instancing::InstanceData>,
    pub history: Vec<[Vec4; 2]>,
    mode: Option<SimulationMode>,
}

#[derive(Component, ExtractComponent, Clone)]
#[extract_app(RenderApp)]
pub struct HunterDraw {
    pub start: u32,
    pub count: u32,
}

#[derive(Resource, ExtractResource, Clone, Default)]
#[extract_app(RenderApp)]
pub struct HunterSettings(pub crate::swarm::Groups);

pub struct HunterPlugin;
impl Plugin for HunterPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<Hunters>()
            .init_resource::<HunterSettings>()
            .add_plugins((
                ExtractResourcePlugin::<Hunters>::default(),
                ExtractResourcePlugin::<HunterSettings>::default(),
                ExtractComponentPlugin::<HunterDraw>::default(),
            ))
            .add_systems(Update, prepare.before(vehicle::step_cpu))
            .add_systems(PostUpdate, sync_draws);
    }
}
fn prepare(
    state: Res<GlobalState>,
    mut hunters: ResMut<Hunters>,
    mut settings: ResMut<HunterSettings>,
) {
    settings.0 = state.groups;
    let configs = state.groups.hunter_configs();
    if hunters
        .configs
        .iter()
        .map(|(id, _)| id)
        .ne(configs.iter().map(|(id, _)| id))
        || hunters.mode != Some(state.mode)
    {
        hunters.boids = initial(&configs, vehicle::world_scale(state.vehicle_count));
        hunters.previous = hunters.boids.clone();
        hunters.mode = Some(state.mode);
    }
    hunters.configs = configs;
    hunters.previous = hunters.boids.clone();
}

pub(crate) fn step_cpu(world: &mut World, dt: f32) {
    let Some(hunters) = world.get_resource::<Hunters>() else {
        return;
    };
    if hunters.boids.is_empty() {
        return;
    }
    let state = world.resource::<GlobalState>().clone();
    let scale = vehicle::world_scale(state.vehicle_count);
    let prey: Vec<_> = world
        .query_filtered::<(&Transform, &VehicleVelocity, &VehicleMetadata), With<Vehicle>>()
        .iter(world)
        .map(|(p, v, m)| (m.0.x, p.translation, Vec3::from(v.0)))
        .collect();
    let mut hunters = world.resource_mut::<Hunters>();
    let old = hunters.boids.clone();
    for (i, me) in old.iter().copied().enumerate() {
        let speed = state.vehicle_max_speed * hunters.configs[i].1.speed_scale;
        hunters.boids[i] = advance(
            me,
            &old,
            &prey,
            i,
            HuntStep {
                speed,
                scale,
                dt,
                bounds: vehicle::walls(state.vehicle_count),
            },
        );
    }
    let boids = hunters.boids.clone();
    for (transform, mut acceleration, mut metadata, velocity) in world
        .query_filtered::<(
            &Transform,
            &mut VehicleAcceleration,
            &mut VehicleMetadata,
            &VehicleVelocity,
        ), With<Vehicle>>()
        .iter_mut(world)
    {
        let speed = state.vehicle_max_speed * state.groups.speed_scale(metadata.0.y);
        let (force, threatened) = flee(transform.translation, &boids, scale, speed);
        let panic = if threatened {
            PANIC_TIME
        } else {
            (f32::from_bits(metadata.0.w).clamp(0.0, PANIC_TIME) - dt).max(0.0)
        };
        metadata.0.w = panic.to_bits();
        // Panic accelerates along the escape direction as well as raising the speed ceiling.
        let boost = if panic > 0.0 {
            Vec3::from(velocity.0).normalize_or_zero() * speed * 0.5
        } else {
            Vec3::ZERO
        };
        acceleration.0 += nalgebra::Vector3::from(force + boost) / state.vehicle_mass;
    }
}

fn sync_draws(
    mut commands: Commands,
    state: Res<GlobalState>,
    mut hunters: ResMut<Hunters>,
    mut meshes: ResMut<Assets<Mesh>>,
    draws: Query<(Entity, &HunterDraw)>,
    mut cache: Local<Vec<(HunterModel, Handle<Mesh>)>>,
    mut previous: Local<Vec<(u32, Group)>>,
) {
    let configs = state.groups.hunter_configs();
    if previous
        .iter()
        .map(|(id, g)| (*id, g.hunter_model))
        .ne(configs.iter().map(|(id, g)| (*id, g.hunter_model)))
    {
        for (entity, _) in &draws {
            commands.entity(entity).despawn();
        }
        let mut start = 0;
        while start < configs.len() {
            let (group_id, group) = configs[start];
            let count = configs[start..]
                .iter()
                .take_while(|(id, _)| *id == group_id)
                .count();
            let mesh = if let Some((_, mesh)) =
                cache.iter().find(|(model, _)| *model == group.hunter_model)
            {
                Some(mesh.clone())
            } else {
                match crate::vehicle_model::hunter_mesh(group.hunter_model.path()) {
                    Ok(mesh) => {
                        let handle = meshes.add(mesh);
                        cache.push((group.hunter_model, handle.clone()));
                        Some(handle)
                    }
                    Err(error) => {
                        error!("hunter model: {error}");
                        None
                    }
                }
            };
            if let Some(mesh) = mesh {
                commands.spawn((
                    Mesh3d(mesh),
                    Transform::default(),
                    NoFrustumCulling,
                    HunterDraw {
                        start: start as u32,
                        count: count as u32,
                    },
                ));
            }
            start += count;
        }
    }
    *previous = configs;
    if state.mode != SimulationMode::Gpu {
        hunters.instances = hunters
            .boids
            .iter()
            .zip(&hunters.configs)
            .enumerate()
            .map(|(i, (b, (_, g)))| b.instance(*g, state.vehicle_max_speed, i as f32 / 16.0))
            .collect();
        hunters.history = hunters
            .previous
            .iter()
            .map(|b| [b.position.extend(0.0), Vec4::from(b.rotation)])
            .collect();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::swarm::Groups;
    #[test]
    fn pursuit_pass_recovery_and_prey_panic() {
        let me = Hunter {
            velocity: Vec3::Y * 10.0,
            rotation: Quat::IDENTITY,
            ..default()
        };
        let prey = [(0, Vec3::X * 15.0, Vec3::ZERO)];
        let next = advance(
            me,
            &[me],
            &prey,
            0,
            HuntStep {
                speed: 100.0,
                scale: 1.0,
                dt: 0.1,
                bounds: Vec3::splat(200.0),
            },
        );
        assert!(next.velocity.x > 0.0);
        assert_eq!(next.timer, LUNGE_TIME);
        let committed = Hunter {
            timer: 0.01,
            velocity: Vec3::X * 100.0,
            ..me
        };
        let pass = advance(
            committed,
            &[committed],
            &[(0, Vec3::NEG_X, Vec3::ZERO)],
            0,
            HuntStep {
                speed: 100.0,
                scale: 1.0,
                dt: 0.02,
                bounds: Vec3::splat(200.0),
            },
        );
        assert_eq!(pass.timer, -COOLDOWN);
        assert!(pass.velocity.x > 0.0);
        let (force, panic) = flee(Vec3::X, &[me], 1.0, 100.0);
        assert!(panic && force.x > 0.0);
        assert_eq!(flee(Vec3::X * 25.0, &[me], 1.0, 100.0), (Vec3::ZERO, false));
        assert_eq!(flee(Vec3::ZERO, &[], 1.0, 100.0), (Vec3::ZERO, false));
    }
    #[test]
    fn cpu_flee_panics_briefly_and_off_preserves_acceleration() {
        for mode in [SimulationMode::Naive, SimulationMode::Octree] {
            let mut world = World::new();
            let mut state = GlobalState {
                mode,
                vehicle_count: 1,
                ..default()
            };
            state.groups.count = 2;
            state.groups.table[1].role = crate::swarm::GroupRole::Hunter;
            state.groups.table[1].hunter_count = 1;
            let configs = state.groups.hunter_configs();
            let hunters = Hunters {
                boids: vec![Hunter {
                    position: Vec3::ZERO,
                    group: 1,
                    ..default()
                }],
                configs,
                ..default()
            };
            world.insert_resource(state);
            world.insert_resource(hunters);
            let entity = world
                .spawn((
                    Vehicle,
                    Transform::from_translation(Vec3::X),
                    VehicleVelocity(Vec3::X.into()),
                    VehicleAcceleration::default(),
                ))
                .id();
            step_cpu(&mut world, 0.01);
            assert!(world.get::<VehicleAcceleration>(entity).unwrap().0.x > 0.0);
            assert_eq!(
                f32::from_bits(world.get::<VehicleMetadata>(entity).unwrap().0.w),
                PANIC_TIME
            );
            world.get_mut::<Transform>(entity).unwrap().translation = Vec3::splat(10000.0);
            step_cpu(&mut world, 0.1);
            assert!(
                (f32::from_bits(world.get::<VehicleMetadata>(entity).unwrap().0.w)
                    - (PANIC_TIME - 0.1))
                    .abs()
                    < 1e-6
            );
            step_cpu(&mut world, 1.0);
            assert_eq!(world.get::<VehicleMetadata>(entity).unwrap().0.w, 0);
            world.resource_mut::<Hunters>().boids.clear();
            let before = world.get::<VehicleAcceleration>(entity).unwrap().0;
            step_cpu(&mut world, 0.1);
            assert_eq!(world.get::<VehicleAcceleration>(entity).unwrap().0, before);
        }
    }

    #[test]
    fn look_and_speed_edits_preserve_hunter_trajectory_and_roles_remove_it() {
        let mut app = App::new();
        let mut state = GlobalState {
            mode: SimulationMode::Naive,
            ..default()
        };
        state.groups.count = 2;
        state.groups.table[1].role = crate::swarm::GroupRole::Hunter;
        app.insert_resource(state)
            .init_resource::<Hunters>()
            .init_resource::<HunterSettings>()
            .add_systems(Update, prepare);
        app.update();
        app.world_mut().resource_mut::<Hunters>().boids[0].position = Vec3::splat(17.0);
        app.world_mut().resource_mut::<Hunters>().boids[0].timer = 0.2;
        let before = app.world().resource::<Hunters>().boids.clone();
        let mut state = app.world_mut().resource_mut::<GlobalState>();
        state.groups.table[1].color = [1, 2, 3];
        state.groups.table[1].target = Some(Vec3::ONE);
        state.groups.table[1].size_scale = 5.0;
        state.groups.table[1].speed_scale = 1.5;
        app.update();
        assert_eq!(app.world().resource::<Hunters>().boids, before);
        app.world_mut().resource_mut::<GlobalState>().groups.count = 0;
        app.update();
        assert!(app.world().resource::<Hunters>().boids.is_empty());
    }

    #[test]
    fn nearest_ties_use_spawn_id_not_sorted_order() {
        let me = Hunter {
            rotation: Quat::IDENTITY,
            ..default()
        };
        let a = (7, Vec3::X * 10.0, Vec3::ZERO);
        let b = (2, Vec3::NEG_X * 10.0, Vec3::ZERO);
        let first = advance(
            me,
            &[me],
            &[a, b],
            0,
            HuntStep {
                speed: 100.0,
                scale: 1.0,
                dt: 0.1,
                bounds: Vec3::splat(200.0),
            },
        );
        let second = advance(
            me,
            &[me],
            &[b, a],
            0,
            HuntStep {
                speed: 100.0,
                scale: 1.0,
                dt: 0.1,
                bounds: Vec3::splat(200.0),
            },
        );
        assert_eq!(first, second);
        assert!(first.velocity.x < 0.0);
    }

    #[test]
    fn hunters_are_bounded_and_prey_membership_excludes_them() {
        let mut groups = Groups {
            count: 8,
            ..default()
        };
        for group in &mut groups.table[1..] {
            group.role = crate::swarm::GroupRole::Hunter;
            group.hunter_count = 12;
        }
        assert_eq!(groups.hunter_count(), MAX_HUNTERS);
        for id in 0..100 {
            assert_eq!(groups.spawn_group(id), 0);
        }
        assert_eq!(Groups::default().hunter_count(), 0);
        groups.table[1].target = Some(Vec3::X);
        assert!(!groups.has_targets());
        groups.table[0].target = Some(Vec3::ZERO);
        assert!(groups.has_targets());
    }
}
