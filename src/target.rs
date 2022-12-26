use bevy::{pbr::NotShadowCaster, prelude::*};
use bevy_mod_picking::{PickableBundle, Selection};

use crate::RenderState;

pub struct TargetPlugin;

impl Plugin for TargetPlugin {
    fn build(&self, app: &mut App) {
        app.add_startup_system(spawn_target)
            .add_system(input_controls);
    }
}

#[derive(Component)]
pub struct Target;

fn spawn_target(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut render_state: ResMut<RenderState>,
) {
    let mesh_handle = meshes.get(&render_state.mesh);

    if mesh_handle.is_none() {
        render_state.mesh = meshes.add(Mesh::from(shape::Icosphere {
            radius: 2.0,
            subdivisions: 3,
        }));
    }

    commands
        .spawn(SpatialBundle {
            transform: Transform::from_translation(Vec3::new(0.0, 0.0, 0.0)),
            ..Default::default()
        })
        .insert(PbrBundle {
            mesh: meshes.get_handle(&render_state.mesh),
            material: materials.add(StandardMaterial {
                base_color: Color::rgba(1.0, 0.0, 0.0, 0.1).into(),
                alpha_mode: AlphaMode::Blend,
                ..Default::default()
            }),
            ..Default::default()
        })
        .insert(NotShadowCaster)
        .insert(Target)
        .insert(PickableBundle::default());
}

// Move 3D object WASD + Space + Shift based on current camera position
fn input_controls(
    keyboard_input: Res<Input<KeyCode>>,
    mut target_query: Query<(&mut Transform, &Selection), With<Target>>,
    camera_query: Query<&Transform, (With<Camera>, Without<Target>)>,
) {
    let (mut transform, selection) = target_query.get_single_mut().unwrap();

    if !selection.selected() {
        return;
    }

    let mut direction = Vec3::ZERO;

    if keyboard_input.pressed(KeyCode::W) {
        direction += Vec3::new(0.0, 0.0, -1.0);
    }

    if keyboard_input.pressed(KeyCode::S) {
        direction += Vec3::new(0.0, 0.0, 1.0);
    }

    if keyboard_input.pressed(KeyCode::A) {
        direction += Vec3::new(-1.0, 0.0, 0.0);
    }

    if keyboard_input.pressed(KeyCode::D) {
        direction += Vec3::new(1.0, 0.0, 0.0);
    }

    if keyboard_input.pressed(KeyCode::Space) {
        direction += Vec3::new(0.0, 1.0, 0.0);
    }

    if keyboard_input.pressed(KeyCode::LShift) {
        direction += Vec3::new(0.0, -1.0, 0.0);
    }

    if direction.length_squared() > 0.0 {
        direction = direction.normalize();
    }

    let camera_transform = camera_query.single();

    transform.translation += camera_transform.rotation * direction * 2.0;
}
