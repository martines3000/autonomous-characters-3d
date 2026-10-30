mod octree;
mod target;
mod vehicle;

use bevy::{
    app::{TaskPoolOptions, TaskPoolPlugin, TaskPoolThreadAssignmentPolicy},
    camera_controller::pan_orbit_camera::prelude::*,
    diagnostic::{DiagnosticsStore, FrameTimeDiagnosticsPlugin},
    input::mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll, MouseScrollUnit},
    picking::{
        mesh_picking::{MeshPickingCamera, MeshPickingSettings},
        PickingSystems,
    },
    prelude::*,
    text::FontSize,
};
use bevy_egui::{egui, EguiContexts, EguiPlugin, EguiPrimaryContextPass};
use octree::*;

fn main() {
    App::new()
        .insert_resource(ClearColor(Color::srgb(0.2, 0.2, 0.2)))
        .init_resource::<GlobalState>()
        .init_resource::<RenderState>()
        .insert_resource(MeshPickingSettings {
            require_markers: true,
            ..default()
        })
        .add_plugins(
            DefaultPlugins
                .set(TaskPoolPlugin {
                    // Assets and async work are negligible; simulation needs the cores.
                    task_pool_options: TaskPoolOptions {
                        io: TaskPoolThreadAssignmentPolicy {
                            min_threads: 1,
                            max_threads: 1,
                            ..TaskPoolOptions::default().io
                        },
                        async_compute: TaskPoolThreadAssignmentPolicy {
                            min_threads: 1,
                            max_threads: 1,
                            ..TaskPoolOptions::default().async_compute
                        },
                        ..default()
                    },
                })
                .set(WindowPlugin {
                    primary_window: Some(Window {
                        title: "Autonomous characters 3D".to_string(),
                        resolution: (1280, 720).into(),
                        ..default()
                    }),
                    ..default()
                }),
        )
        .add_plugins((
            MeshPickingPlugin,
            DefaultPanOrbitCameraPlugins,
            EguiPlugin::default(),
            FrameTimeDiagnosticsPlugin::default(),
            vehicle::VehiclePlugin,
            target::TargetPlugin,
        ))
        .add_systems(PreStartup, load_assets)
        .add_systems(Startup, (setup_scene, configure_global_state))
        .add_systems(
            PreUpdate,
            camera_input
                .after(PickingSystems::Last)
                .before(PanOrbitCamera::update_camera_positions),
        )
        .add_systems(Update, fps_update_system)
        .add_systems(EguiPrimaryContextPass, ui)
        .run();
}

// The built-in controller deliberately leaves input mapping to the application.
fn camera_input(
    buttons: Res<ButtonInput<MouseButton>>,
    motion: Res<AccumulatedMouseMotion>,
    scroll: Res<AccumulatedMouseScroll>,
    mut contexts: EguiContexts,
    mut cameras: Query<&mut PanOrbitCamera>,
    mut previous_mode: Local<u8>,
) {
    let blocked = contexts
        .ctx_mut()
        .is_ok_and(|ctx| ctx.egui_wants_pointer_input());
    let mode = if blocked {
        0
    } else if buttons.pressed(MouseButton::Right) {
        1
    } else if buttons.pressed(MouseButton::Middle) {
        2
    } else if scroll.delta.y != 0.0 {
        3
    } else {
        0
    };
    for mut camera in &mut cameras {
        if mode != *previous_mode {
            camera.end_move();
            match mode {
                1 => camera.start_orbit(None),
                2 => camera.start_pan(None),
                3 => camera.start_zoom(None),
                _ => (),
            }
        }
        if mode == 1 || mode == 2 {
            camera.send_screenspace_input(motion.delta);
        }
        if !blocked && scroll.delta.y != 0.0 {
            let scale = if scroll.unit == MouseScrollUnit::Line {
                150.0
            } else {
                1.0
            };
            camera.send_zoom_input(scroll.delta.y * scale);
        }
    }
    *previous_mode = mode;
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
    for node in &octree.nodes {
        println!("Bounds: {:?}", node.bounds);

        for point_index in node.points.iter() {
            let point = octree.points.get(*point_index).unwrap();
            println!("Point: {:?}", point.point);
        }
    }

    // Draw nodes
    for node in &octree.nodes {
        if node.children.len() > 0 {
            continue;
        }
        let (center, min, max) = &node.bounds;

        let mesh = meshes.add(Cuboid::from_length(8.0 * (max.x - min.x) / 10.0));

        commands.spawn((
            Mesh3d(mesh),
            MeshMaterial3d(materials.add(Color::srgb(0.0, 0.0, 1.0))),
            Transform::from_translation(Vec3::new(center.x, center.y, center.z)),
        ));
    }
}

fn setup_scene(mut commands: Commands, asset_server: Res<AssetServer>) {
    commands.insert_resource(GlobalAmbientLight {
        brightness: 1000.0,
        ..default()
    });
    let font = TextFont {
        font: asset_server.load("fonts/FiraSans-Bold.ttf").into(),
        font_size: FontSize::Px(60.0),
        ..default()
    };
    commands
        .spawn((
            Text::new("FPS: "),
            font.clone(),
            TextColor(Color::WHITE),
            Node {
                position_type: PositionType::Absolute,
                top: px(5),
                right: px(5),
                ..default()
            },
        ))
        .with_child((
            TextSpan::new(""),
            font,
            TextColor(Color::srgb(1.0, 0.84, 0.0)),
            FpsText,
        ));
    // Frames the spawn cube around the target.
    let eye = Vec3::new(0.0, 60.0, 170.0);
    commands.spawn((
        Camera3d::default(),
        Msaa::Sample4,
        Transform::from_translation(eye).looking_at(Vec3::ZERO, Vec3::Y),
        PanOrbitCamera::default().with_initial_anchor_depth(eye.length() as f64),
        MeshPickingCamera,
    ));
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
}

fn configure_global_state(mut state: ResMut<GlobalState>) {
    state.vehicle_count = 200;
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

    state.use_octree = true;
    state.octree_size = 100;
}

#[derive(Default, Resource)]
struct RenderState {
    vehicle_scene: Handle<WorldAsset>,
}

fn load_assets(mut render_state: ResMut<RenderState>, asset_server: Res<AssetServer>) {
    render_state.vehicle_scene = asset_server.load("cone.glb#Scene0");
}

fn ui(
    mut egui_context: EguiContexts,
    mut state: ResMut<GlobalState>,
    targets: Query<&target::TargetSelection, With<target::Target>>,
) -> Result {
    let ctx = egui_context.ctx_mut()?;
    egui::Window::new("Menu")
        .default_size([300.0, 100.0])
        .show(ctx, |ui| {
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
        });
    let following = targets.iter().any(|selection| selection.0);
    egui::Window::new("Controls")
        .anchor(egui::Align2::RIGHT_BOTTOM, [-12.0, -12.0])
        .resizable(false)
        .collapsible(false)
        .show(ctx, |ui| {
            if following {
                ui.colored_label(egui::Color32::LIGHT_GREEN, "Follow: ON (green target)");
                ui.label("Vehicles follow the selected target.");
            } else {
                ui.colored_label(egui::Color32::LIGHT_RED, "Follow: OFF (red target)");
                ui.label("Vehicles wander freely.");
            }
            ui.separator();
            egui::Grid::new("controls").show(ui, |ui| {
                for (input, action) in [
                    ("Left-click target", "Start following"),
                    ("Left-click background", "Stop following"),
                    ("Right-drag", "Orbit camera"),
                    ("Middle-drag", "Pan camera"),
                    ("Scroll", "Zoom camera"),
                    ("W / A / S / D", "Move selected target"),
                    ("Space / Left Shift", "Move target up / down"),
                ] {
                    ui.label(input);
                    ui.label(action);
                    ui.end_row();
                }
            });
            ui.separator();
            ui.small("Target movement is relative to the camera.");
        });
    Ok(())
}

#[derive(Component)]
struct FpsText;

fn fps_update_system(
    diagnostics: Res<DiagnosticsStore>,
    mut query: Query<&mut TextSpan, With<FpsText>>,
) {
    if let Some(value) = diagnostics
        .get(&FrameTimeDiagnosticsPlugin::FPS)
        .and_then(|fps| fps.smoothed())
    {
        for mut text in &mut query {
            **text = format!("{value:.2}");
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bevy::{
        camera::{CameraProjection, ComputedCameraValues, RenderTargetInfo},
        window::RequestRedraw,
    };
    use bevy_egui::EguiUserTextures;
    use std::time::Duration;

    #[test]
    fn camera_mouse_adapter_applies_orbit_and_scroll() {
        let mut app = App::new();
        app.init_resource::<ButtonInput<MouseButton>>()
            .init_resource::<AccumulatedMouseMotion>()
            .init_resource::<AccumulatedMouseScroll>()
            .init_resource::<EguiUserTextures>()
            .init_resource::<Time<Real>>()
            .add_message::<RequestRedraw>()
            .add_systems(
                Update,
                (camera_input, PanOrbitCamera::update_camera_positions).chain(),
            );
        let eye = Vec3::new(-2.0, 5.0, 5.0);
        let original = Transform::from_translation(eye).looking_at(Vec3::ZERO, Vec3::Y);
        let projection = PerspectiveProjection {
            aspect_ratio: 1280.0 / 720.0,
            ..default()
        };
        let camera = app
            .world_mut()
            .spawn((
                Camera3d::default(),
                Camera {
                    computed: ComputedCameraValues {
                        clip_from_view: projection.get_clip_from_view(),
                        target_info: Some(RenderTargetInfo {
                            physical_size: UVec2::new(1280, 720),
                            scale_factor: 1.0,
                        }),
                        ..default()
                    },
                    ..default()
                },
                Projection::Perspective(projection),
                PanOrbitCamera::default().with_initial_anchor_depth(eye.length() as f64),
                original,
            ))
            .id();
        app.world_mut()
            .resource_mut::<Time<Real>>()
            .advance_by(Duration::from_millis(16));
        app.world_mut()
            .resource_mut::<AccumulatedMouseScroll>()
            .delta
            .y = 1.0;
        app.update();
        let zoomed = *app.world().get::<Transform>(camera).unwrap();
        assert!(zoomed.translation.distance(original.translation) > 0.001);
        app.world_mut()
            .resource_mut::<AccumulatedMouseScroll>()
            .delta = Vec2::ZERO;
        app.world_mut()
            .resource_mut::<ButtonInput<MouseButton>>()
            .press(MouseButton::Right);
        app.world_mut()
            .resource_mut::<AccumulatedMouseMotion>()
            .delta = Vec2::new(50.0, 25.0);
        app.update();
        let orbited = app.world().get::<Transform>(camera).unwrap();
        assert!(orbited.rotation.angle_between(zoomed.rotation) > 0.001);
        assert!(orbited.translation.is_finite());
    }
}
