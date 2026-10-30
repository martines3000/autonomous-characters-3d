use bevy::{light::NotShadowCaster, picking::pointer::PointerButton, prelude::*};
use bevy_egui::EguiContexts;

pub struct TargetPlugin;

impl Plugin for TargetPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, spawn_target)
            .add_systems(Update, (input_controls, update_target_color))
            .add_observer(select_target);
    }
}

#[derive(Component)]
pub struct Target;

#[derive(Component, Default)]
pub struct TargetSelection(pub bool);

const IDLE_COLOR: Color = Color::srgba(1.0, 0.0, 0.0, 0.1);
const FOLLOW_COLOR: Color = Color::srgba(0.1, 1.0, 0.2, 0.35);

fn spawn_target(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn((
        Mesh3d(meshes.add(Sphere::new(2.0).mesh().ico(3).unwrap())),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: IDLE_COLOR,
            alpha_mode: AlphaMode::Blend,
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
fn input_controls(
    keyboard_input: Res<ButtonInput<KeyCode>>,
    mut target_query: Query<(&mut Transform, &TargetSelection), With<Target>>,
    camera_query: Query<&Transform, (With<Camera3d>, Without<Target>)>,
    mut contexts: EguiContexts,
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
    transform.translation += camera_transform.rotation * direction.normalize_or_zero() * 2.0;
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
    fn target_selection_controls_movement_and_color() {
        let mut app = App::new();
        app.init_resource::<EguiUserTextures>()
            .init_resource::<ButtonInput<KeyCode>>()
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
