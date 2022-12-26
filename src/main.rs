mod target;
mod vehicle;

use bevy::{
    diagnostic::{Diagnostics, FrameTimeDiagnosticsPlugin},
    prelude::*,
    window::PresentMode,
};
use bevy_mod_picking::*;
use smooth_bevy_cameras::{
    controllers::orbit::{OrbitCameraBundle, OrbitCameraController, OrbitCameraPlugin},
    LookTransformPlugin,
};

use bevy_egui::{egui, EguiContext, EguiPlugin};

fn main() {
    App::new()
        .insert_resource(ClearColor(Color::rgb(0.2, 0.2, 0.2)))
        .insert_resource(Msaa { samples: 4 })
        .init_resource::<GlobalState>()
        .init_resource::<RenderState>()
        .add_startup_system(setup_scene)
        .add_startup_system(configure_global_state)
        .add_startup_system(load_assets)
        .add_system(bevy::window::close_on_esc)
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            window: WindowDescriptor {
                title: "Autonomous characters 3D".to_string(),
                width: 500.0,
                height: 500.0,
                present_mode: PresentMode::AutoNoVsync,
                ..default()
            },
            ..default()
        }))
        .add_plugins(DefaultPickingPlugins)
        .add_plugin(LookTransformPlugin)
        .add_plugin(OrbitCameraPlugin::default())
        .add_plugin(EguiPlugin)
        .add_plugin(FrameTimeDiagnosticsPlugin::default())
        .add_plugin(vehicle::VehiclePlugin)
        .add_plugin(target::TargetPlugin)
        .add_system(fps_update_system)
        .add_system(ui)
        .run();
}

fn setup_scene(mut commands: Commands, asset_server: Res<AssetServer>) {
    // AMBIENT LIGHT
    commands.insert_resource(AmbientLight {
        color: Color::WHITE,
        brightness: 1.0,
    });

    // FPS TEXT
    commands.spawn((
        // Create a TextBundle that has a Text with a list of sections.
        TextBundle::from_sections([
            TextSection::new(
                "FPS: ",
                TextStyle {
                    font: asset_server.load("fonts/FiraSans-Bold.ttf"),
                    font_size: 60.0,
                    color: Color::WHITE,
                },
            ),
            TextSection::from_style(TextStyle {
                font: asset_server.load("fonts/FiraSans-Bold.ttf"),
                font_size: 60.0,
                color: Color::GOLD,
            }),
        ])
        .with_style(Style {
            position_type: PositionType::Absolute,
            position: UiRect {
                top: Val::Px(5.0),
                left: Val::Px(5.0),
                ..default()
            },
            ..Default::default()
        }),
        FpsText,
    ));

    // ORBIT CAMERA
    commands
        .spawn(Camera3dBundle::default())
        .insert(OrbitCameraBundle::new(
            OrbitCameraController {
                mouse_rotate_sensitivity: Vec2 { x: 0.5, y: 0.5 },
                mouse_translate_sensitivity: Vec2 { x: 0.5, y: 0.5 },
                ..Default::default()
            },
            Vec3::new(-2.0, 5.0, 5.0),
            Vec3::new(0., 0., 0.),
        ))
        .insert(PickingCameraBundle::default());
}

#[derive(Default, Resource)]
struct GlobalState {
    // Basic info
    vehicle_count: usize,
    vehicle_size: f32,
    vehicle_mass: f32,
    vehicle_wander_speed: f32,

    // Limits
    vehicle_max_speed: f32,

    // Factors
    vehicle_seperation_factor: f32,
    vehicle_alignment_factor: f32,
    vehicle_cohesion_factor: f32,
    vehicle_wander_factor: f32,
    vehicle_wall_avoid_factor: f32,
    vehicle_seek_factor: f32,

    // Distances
    vehicle_seperation_distance: f32,
    vehicle_alignment_distance: f32,
    vehicle_cohesion_distance: f32,
    vehicle_wander_distance: f32,
    vehicle_wander_radius: f32,
}

fn configure_global_state(mut state: ResMut<GlobalState>) {
    state.vehicle_count = 100;
    state.vehicle_size = 1.0;
    state.vehicle_mass = 60.0;
    state.vehicle_wander_speed = 1.0;

    state.vehicle_max_speed = 1.0;

    state.vehicle_seperation_factor = 1.0;
    state.vehicle_alignment_factor = 1.0;
    state.vehicle_cohesion_factor = 1.0;
    state.vehicle_wander_factor = 1.0;
    state.vehicle_wall_avoid_factor = 1.0;
    state.vehicle_seek_factor = 1.0;

    state.vehicle_seperation_distance = 1.0;
    state.vehicle_alignment_distance = 1.0;
    state.vehicle_cohesion_distance = 1.0;
    state.vehicle_wander_distance = 1.0;
    state.vehicle_wander_radius = 1.0;
}

#[derive(Default, Resource)]
struct RenderState {
    mesh: Handle<Mesh>,
    vehicle_material: Handle<StandardMaterial>,
    vehicle_mesh: Handle<Mesh>,
}

fn load_assets(
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut render_state: ResMut<RenderState>,
) {
    render_state.vehicle_material = materials.add(StandardMaterial {
        base_color: Color::rgba(0.0, 0.0, 1.0, 1.0).into(),
        ..Default::default()
    });

    render_state.vehicle_mesh = meshes.add(Mesh::from(shape::Cube { size: 1.0 }));
}

fn ui(mut egui_context: ResMut<EguiContext>, mut state: ResMut<GlobalState>) {
    egui::Window::new("Menu")
        .default_size([300.0, 100.0])
        .show(egui_context.ctx_mut(), |ui| {
            ui.label("Vehicle count");
            ui.add(egui::Slider::new(&mut state.vehicle_count, 0..=50000).text("count"));
            ui.label("Vehicle mass");
            ui.add(
                egui::Slider::new(&mut state.vehicle_mass, 1.0..=1000.0)
                    .text("mass")
                    .step_by(0.1),
            );
            ui.label("Vehicle max speed");
            ui.add(
                egui::Slider::new(&mut state.vehicle_max_speed, 0.1..=100.0)
                    .text("max speed")
                    .step_by(0.1),
            );
            ui.label("Vehicle seperation factor");
            ui.add(
                egui::Slider::new(&mut state.vehicle_seperation_factor, 0.1..=10.0)
                    .text("separation")
                    .step_by(0.1),
            );
            ui.label("Vehicle alignment factor");
            ui.add(
                egui::Slider::new(&mut state.vehicle_alignment_factor, 0.1..=10.0)
                    .text("alignment")
                    .step_by(0.1),
            );
            ui.label("Vehicle cohesion factor");
            ui.add(
                egui::Slider::new(&mut state.vehicle_cohesion_factor, 0.1..=10.0)
                    .text("cohesion")
                    .step_by(0.1),
            );
            ui.label("Vehicle wander factor");
            ui.add(
                egui::Slider::new(&mut state.vehicle_wander_factor, 0.1..=10.0)
                    .text("wander")
                    .step_by(0.1),
            );
            ui.label("Vehicle wall avoid factor");
            ui.add(
                egui::Slider::new(&mut state.vehicle_wall_avoid_factor, 0.1..=10.0)
                    .text("wall avoid")
                    .step_by(0.1),
            );
            ui.label("Vehicle seek factor");
            ui.add(
                egui::Slider::new(&mut state.vehicle_seek_factor, 0.001..=0.1)
                    .text("seek")
                    .step_by(0.01),
            );
            ui.label("Vehicle seperation distance");
            ui.add(
                egui::Slider::new(&mut state.vehicle_seperation_distance, 1.0..=100.0)
                    .text("separation")
                    .step_by(1.0),
            );
            ui.label("Vehicle alignment distance");
            ui.add(
                egui::Slider::new(&mut state.vehicle_alignment_distance, 1.0..=100.0)
                    .text("alignment")
                    .step_by(1.0),
            );
            ui.label("Vehicle cohesion distance");
            ui.add(
                egui::Slider::new(&mut state.vehicle_cohesion_distance, 1.0..=100.0)
                    .text("cohesion")
                    .step_by(1.0),
            );
            ui.label("Vehicle wander distance");
            ui.add(
                egui::Slider::new(&mut state.vehicle_wander_distance, 1.0..=100.0)
                    .text("wander")
                    .step_by(1.0),
            );
            ui.label("Vehicle wander radius");
            ui.add(
                egui::Slider::new(&mut state.vehicle_wander_radius, 1.0..=100.0)
                    .text("wander")
                    .step_by(1.0),
            );
        });
}

#[derive(Component)]
struct FpsText;

fn fps_update_system(diagnostics: Res<Diagnostics>, mut query: Query<&mut Text, With<FpsText>>) {
    for mut text in &mut query {
        if let Some(fps) = diagnostics.get(FrameTimeDiagnosticsPlugin::FPS) {
            if let Some(value) = fps.smoothed() {
                // Update the value of the second section
                text.sections[1].value = format!("{value:.2}");
            }
        }
    }
}
