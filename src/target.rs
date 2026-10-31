use bevy::{light::NotShadowCaster, picking::pointer::PointerButton, prelude::*};
use bevy_egui::EguiContexts;

use crate::{vehicle, GlobalState};

pub struct TargetPlugin;

impl Plugin for TargetPlugin {
    fn build(&self, app: &mut App) {
        app.init_gizmo_group::<TargetMarker>();
        let mut store = app.world_mut().resource_mut::<GizmoConfigStore>();
        let (config, _) = store.config_mut::<TargetMarker>();
        config.depth_bias = -1.0;
        config.line.width = 1.5;
        app.add_systems(Startup, spawn_target)
            .add_systems(
                Update,
                (input_controls, update_target_color, update_target_scale),
            )
            .add_systems(
                PostUpdate,
                draw_target_marker.after(crate::scene::frame_world),
            )
            .add_observer(select_target);
    }
}

#[derive(Component)]
pub struct Target;

#[derive(Component, Default)]
pub struct TargetSelection(pub bool);

#[derive(Default, Reflect, GizmoConfigGroup)]
struct TargetMarker;

// A screen-readable locator remains visible in a dense following flock. The sphere alone is pickable.
fn draw_target_marker(
    mut gizmos: Gizmos<TargetMarker>,
    targets: Query<(&Transform, &TargetSelection), With<Target>>,
    cameras: Query<(&Transform, &Projection), With<Camera3d>>,
    windows: Query<&Window, With<bevy::window::PrimaryWindow>>,
) {
    let (Ok((camera, projection)), Ok(window)) = (cameras.single(), windows.single()) else {
        return;
    };
    let Projection::Perspective(projection) = projection else {
        return;
    };
    for (target, selection) in &targets {
        let depth = -(camera.rotation.inverse() * (target.translation - camera.translation)).z;
        if depth <= 0.0 {
            continue;
        }
        let radius = (depth * (projection.fov * 0.5).tan() * 24.0
            / window.physical_height().max(1) as f32)
            .max(target.scale.x * 3.0);
        let color = if selection.0 {
            FOLLOW_COLOR
        } else {
            IDLE_COLOR
        };
        gizmos
            .circle(
                Isometry3d::new(target.translation, camera.rotation),
                radius,
                color,
            )
            .resolution(32);
        for direction in [Vec3::X, Vec3::NEG_X, Vec3::Y, Vec3::NEG_Y] {
            let axis = camera.rotation * direction * radius;
            gizmos.line(
                target.translation + axis * 1.25,
                target.translation + axis * 1.6,
                color,
            );
        }
    }
}

const IDLE_COLOR: Color = Color::srgb(1.0, 0.18, 0.12);
const FOLLOW_COLOR: Color = Color::srgb(0.15, 1.0, 0.32);
const TARGET_SPEED: f32 = 120.0;

fn spawn_target(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn((
        Mesh3d(meshes.add(Sphere::new(2.0).mesh().ico(3).unwrap())),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: IDLE_COLOR,
            emissive: IDLE_COLOR.to_linear() * 0.4,
            perceptual_roughness: 0.35,
            ..default()
        })),
        Transform::default(),
        NotShadowCaster,
        Target,
        TargetSelection::default(),
        Pickable::default(),
    ));
}

fn update_target_color(
    targets: Query<
        (&TargetSelection, &MeshMaterial3d<StandardMaterial>),
        (With<Target>, Changed<TargetSelection>),
    >,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    for (selection, material) in &targets {
        if let Some(mut material) = materials.get_mut(&material.0) {
            material.base_color = if selection.0 {
                FOLLOW_COLOR
            } else {
                IDLE_COLOR
            };
            material.emissive = material.base_color.to_linear() * 0.4;
        }
    }
}

fn update_target_scale(state: Res<GlobalState>, mut targets: Query<&mut Transform, With<Target>>) {
    let scale = Vec3::splat(vehicle::world_scale(state.vehicle_count));
    for mut transform in &mut targets {
        if transform.scale != scale {
            transform.scale = scale;
        }
    }
}

fn select_target(
    click: On<PointerClick>,
    mut targets: Query<(Entity, &mut TargetSelection), With<Target>>,
    mut contexts: EguiContexts,
) {
    if click.button != PointerButton::Primary
        || contexts
            .ctx_mut()
            .is_ok_and(|ctx| ctx.egui_wants_pointer_input())
    {
        return;
    }
    for (entity, mut selection) in &mut targets {
        selection.0 = entity == click.original_event_target();
    }
}

// Move the selected target relative to the current camera.
pub(crate) fn input_controls(
    keyboard_input: Res<ButtonInput<KeyCode>>,
    mut target_query: Query<(&mut Transform, &TargetSelection), With<Target>>,
    camera_query: Query<&Transform, (With<Camera3d>, Without<Target>)>,
    mut contexts: EguiContexts,
    time: Res<Time>,
) {
    if contexts
        .ctx_mut()
        .is_ok_and(|ctx| ctx.egui_wants_keyboard_input())
    {
        return;
    }
    let (mut transform, selection) = target_query.single_mut().unwrap();
    if !selection.0 {
        return;
    }
    let mut direction = Vec3::ZERO;
    for (key, axis) in [
        (KeyCode::KeyW, Vec3::NEG_Z),
        (KeyCode::KeyS, Vec3::Z),
        (KeyCode::KeyA, Vec3::NEG_X),
        (KeyCode::KeyD, Vec3::X),
        (KeyCode::Space, Vec3::Y),
        (KeyCode::ShiftLeft, Vec3::NEG_Y),
    ] {
        if keyboard_input.pressed(key) {
            direction += axis;
        }
    }
    let camera_transform = camera_query.single().unwrap();
    transform.translation += camera_transform.rotation
        * direction.normalize_or_zero()
        * TARGET_SPEED
        * time.delta_secs();
}

#[cfg(test)]
mod tests {
    use super::*;
    use bevy::{
        camera::NormalizedRenderTarget,
        picking::{
            backend::HitData,
            events::Pointer,
            pointer::{Location, PointerId},
        },
        window::WindowRef,
    };
    use bevy_egui::EguiUserTextures;
    use std::time::Duration;

    #[test]
    fn target_visual_and_pick_mesh_scale_with_live_vehicle_count() {
        let mut app = App::new();
        app.init_resource::<GlobalState>()
            .init_resource::<Assets<Mesh>>()
            .init_resource::<Assets<StandardMaterial>>()
            .add_systems(Startup, spawn_target)
            .add_systems(Update, update_target_scale);
        app.update();
        let entity = app
            .world_mut()
            .query_filtered::<Entity, With<Target>>()
            .single(app.world())
            .unwrap();
        let position = Vec3::new(123.0, -45.0, 67.0);
        let rotation = Quat::from_rotation_y(0.5);
        {
            let mut transform = app.world_mut().get_mut::<Transform>(entity).unwrap();
            transform.translation = position;
            transform.rotation = rotation;
        }
        let mesh = app.world().get::<Mesh3d>(entity).unwrap().0.clone();
        assert!(app.world().get::<Pickable>(entity).is_some());
        for count in [20_000, 1_000_000, 200, 0, 20_000] {
            app.world_mut().resource_mut::<GlobalState>().vehicle_count = count;
            app.update();
            let transform = app.world().get::<Transform>(entity).unwrap();
            assert_eq!(transform.scale, Vec3::splat(vehicle::world_scale(count)));
            assert_eq!(transform.translation, position);
            assert_eq!(transform.rotation, rotation);
            assert_eq!(app.world().get::<Mesh3d>(entity).unwrap().0, mesh);
        }
        assert_eq!(app.world().resource::<Assets<Mesh>>().len(), 1);
    }

    #[test]
    fn held_key_movement_is_frame_rate_independent() {
        for steps in [30_u64, 60, 120, 144] {
            let mut app = App::new();
            app.init_resource::<EguiUserTextures>()
                .init_resource::<ButtonInput<KeyCode>>()
                .init_resource::<Time>()
                .init_resource::<GlobalState>()
                .add_systems(Update, (input_controls, update_target_scale));
            let target = app
                .world_mut()
                .spawn((Target, TargetSelection(true), Transform::default()))
                .id();
            app.world_mut()
                .spawn((Camera3d::default(), Transform::default()));
            app.world_mut()
                .resource_mut::<ButtonInput<KeyCode>>()
                .press(KeyCode::KeyW);
            for step in 1..=steps {
                // Scale changes during movement must not change its world-space speed.
                app.world_mut().resource_mut::<GlobalState>().vehicle_count =
                    if step % 2 == 0 { 1_000_000 } else { 200 };
                app.world_mut()
                    .resource_mut::<Time>()
                    .advance_to(Duration::from_nanos(step * 1_000_000_000 / steps));
                app.update();
            }
            assert_eq!(
                app.world().resource::<Time>().elapsed(),
                Duration::from_secs(1)
            );
            let position = app.world().get::<Transform>(target).unwrap().translation;
            assert!(
                position.distance(Vec3::NEG_Z * 120.0) < 1e-3,
                "{steps} steps: {position:?}"
            );
        }
    }

    #[test]
    fn target_selection_controls_movement_and_color() {
        let mut app = App::new();
        app.init_resource::<EguiUserTextures>()
            .init_resource::<ButtonInput<KeyCode>>()
            .init_resource::<Time>()
            .init_resource::<Assets<StandardMaterial>>()
            .add_systems(Update, (input_controls, update_target_color))
            .add_observer(select_target);
        let window = app.world_mut().spawn_empty().id();
        let material = app
            .world_mut()
            .resource_mut::<Assets<StandardMaterial>>()
            .add(StandardMaterial::default());
        let target = app
            .world_mut()
            .spawn((
                Target,
                TargetSelection::default(),
                Transform::default(),
                MeshMaterial3d(material.clone()),
            ))
            .id();
        app.world_mut()
            .spawn((Camera3d::default(), Transform::default()));
        let click = |entity, button| PointerClick {
            entity,
            pointer: Pointer::new(
                PointerId::Mouse,
                Location {
                    target: NormalizedRenderTarget::Window(
                        WindowRef::Entity(window).normalize(None).unwrap(),
                    ),
                    position: Vec2::ZERO,
                },
            ),
            button,
            hit: HitData::new(window, 1.0, None, None),
            duration: Duration::from_millis(50),
            count: 1,
        };
        app.update();
        assert_eq!(
            app.world()
                .resource::<Assets<StandardMaterial>>()
                .get(&material)
                .unwrap()
                .base_color,
            IDLE_COLOR
        );
        app.world_mut()
            .trigger(click(target, PointerButton::Primary));
        assert!(app.world().get::<TargetSelection>(target).unwrap().0);
        app.world_mut()
            .resource_mut::<ButtonInput<KeyCode>>()
            .press(KeyCode::KeyW);
        app.world_mut()
            .resource_mut::<Time>()
            .advance_by(Duration::from_secs_f32(1.0 / 60.0));
        app.update();
        assert_eq!(
            app.world()
                .resource::<Assets<StandardMaterial>>()
                .get(&material)
                .unwrap()
                .base_color,
            FOLLOW_COLOR
        );
        assert_eq!(
            app.world().get::<Transform>(target).unwrap().translation,
            Vec3::NEG_Z * 2.0
        );
        app.world_mut()
            .trigger(click(window, PointerButton::Secondary));
        assert!(app.world().get::<TargetSelection>(target).unwrap().0);
        app.world_mut()
            .trigger(click(window, PointerButton::Primary));
        assert!(!app.world().get::<TargetSelection>(target).unwrap().0);
        app.update();
        assert_eq!(
            app.world()
                .resource::<Assets<StandardMaterial>>()
                .get(&material)
                .unwrap()
                .base_color,
            IDLE_COLOR
        );
        assert_eq!(
            app.world().get::<Transform>(target).unwrap().translation,
            Vec3::NEG_Z * 2.0
        );
    }
}
