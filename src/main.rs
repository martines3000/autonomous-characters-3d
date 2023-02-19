mod octree;
mod target;
mod vehicle;

use bevy::{
    diagnostic::{Diagnostics, FrameTimeDiagnosticsPlugin},
    prelude::*,
};
use bevy_mod_picking::*;
use octree::*;
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
        .add_startup_system_to_stage(StartupStage::PreStartup, load_assets)
        .add_startup_system(setup_scene)
        .add_startup_system(configure_global_state)
        // .add_startup_system(test_octree_visualization)
        .add_system(bevy::window::close_on_esc)
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            window: WindowDescriptor {
                title: "Autonomous characters 3D".to_string(),
                width: 1280.,
                height: 720.,
                canvas: Some("#bevy".to_owned()),
                fit_canvas_to_parent: true,
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

pub fn test_octree_visualization(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let mut octree: Octree<i32> = Octree::new(100);
    octree.create_root(
        Point {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
        Point {
            x: -50.0,
            y: -50.0,
            z: -50.0,
        },
        Point {
            x: 50.0,
            y: 50.0,
            z: 50.0,
        },
    );

    // Insert points
    for i in 0..50000 {
        let point = Point {
            x: -50.0 + rand::random::<f32>() * 100.0,
            y: -50.0 + rand::random::<f32>() * 100.0,
            z: -50.0 + rand::random::<f32>() * 100.0,
        };

        octree.insert(point, i);
    }

    // Print nodes and their points
    for (_, node) in octree.nodes.iter() {
        println!("Bounds: {:?}", node.bounds);

        for point_index in node.points.iter() {
            let point = octree.points.get(*point_index).unwrap();
            println!("Point: {:?}", point.point);
        }
    }

    // Draw nodes
    for (_, node) in octree.nodes.iter() {
        if node.children.len() > 0 {
            continue;
        }
        let (center, min, max) = &node.bounds;

        let mesh = meshes.add(Mesh::from(shape::Cube {
            size: 8.0 * (max.x - min.x) / 10.0,
        }));

        commands.spawn(PbrBundle {
            mesh,
            material: materials.add(Color::rgb(0.0, 0.0, 1.0).into()),
            transform: Transform::from_translation(Vec3::new(center.x, center.y, center.z)),
            ..Default::default()
        });
    }
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

    // Toggle mode
    use_octree: bool,
    octree_size: usize,
    benchmark_mode: bool,
    benchmark_step: usize,
    benchmark_results: Vec<f32>,
    benchmark_current_results: Vec<f32>,
}

fn configure_global_state(mut state: ResMut<GlobalState>) {
    state.vehicle_count = 100;
    state.vehicle_size = 1.0;
    state.vehicle_mass = 60.0;
    state.vehicle_wander_speed = 40.0;

    state.vehicle_max_speed = 80.0;

    state.vehicle_seperation_factor = 1.5;
    state.vehicle_alignment_factor = 3.0;
    state.vehicle_cohesion_factor = 4.0;
    state.vehicle_wander_factor = 1.0;
    state.vehicle_wall_avoid_factor = 1.5;
    state.vehicle_seek_factor = 7.0;

    state.vehicle_seperation_distance = 4.0;
    state.vehicle_alignment_distance = 30.0;
    state.vehicle_cohesion_distance = 20.0;

    state.vehicle_wander_distance = 4.0;
    state.vehicle_wander_radius = 1.5;

    state.use_octree = false;
    state.octree_size = 100;

    state.benchmark_mode = false;
    state.benchmark_step = 0;
    state.benchmark_results = vec![];
    state.benchmark_current_results = vec![];
}

#[derive(Default, Resource)]
struct RenderState {
    mesh: Handle<Mesh>,
    vehicle_material: Handle<StandardMaterial>,
    vehicle_mesh: Handle<Mesh>,
    vehicle_scene: Handle<Scene>,
}

fn load_assets(
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut render_state: ResMut<RenderState>,
    asset_server: Res<AssetServer>,
) {
    render_state.vehicle_material = materials.add(StandardMaterial {
        base_color: Color::rgba(0.0, 0.0, 1.0, 1.0).into(),
        ..Default::default()
    });

    render_state.vehicle_mesh = meshes.add(Mesh::from(shape::Cube { size: 1.0 }));

    render_state.vehicle_scene = asset_server.load("cone.glb#Scene0");
}

fn ui(mut egui_context: ResMut<EguiContext>, mut state: ResMut<GlobalState>) {
    egui::Window::new("Menu")
        .default_size([300.0, 100.0])
        .show(egui_context.ctx_mut(), |ui| {
            ui.label("Vehicle count");
            ui.add(egui::Slider::new(&mut state.vehicle_count, 0..=50000).text("count"));
            ui.label("Vehicle mass");
            ui.add(egui::Slider::new(&mut state.vehicle_mass, 1.0..=1000.0).text("mass"));
            ui.label("Vehicle max speed");
            ui.add(egui::Slider::new(&mut state.vehicle_max_speed, 0.1..=100.0).text("max speed"));
            ui.label("Vehicle wander speed");
            ui.add(
                egui::Slider::new(&mut state.vehicle_wander_speed, 0.1..=100.0)
                    .text("wander speed"),
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

            ui.separator();
            ui.checkbox(&mut state.use_octree, "Use octree");
            ui.add(
                egui::Slider::new(&mut state.octree_size, 2..=500)
                    .text("octree size")
                    .step_by(1.0),
            );

            ui.separator();
            ui.checkbox(&mut state.benchmark_mode, "Benchmark mode");
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
