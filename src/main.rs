mod capture;
mod director;
mod environment;
mod follow;
mod formation;
mod gpu_flock;
mod instancing;
mod motion;
mod octree;
mod predator;
mod presentation;
mod scenario;
mod scene;
mod swarm;
mod target;
mod vehicle;
mod vehicle_appearance;
mod vehicle_model;

use bevy::{
    app::{TaskPoolOptions, TaskPoolPlugin, TaskPoolThreadAssignmentPolicy},
    camera_controller::pan_orbit_camera::prelude::*,
    diagnostic::{DiagnosticsStore, FrameTimeDiagnosticsPlugin},
    input::mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll, MouseScrollUnit},
    picking::{mesh_picking::MeshPickingSettings, PickingSystems},
    prelude::*,
};
use bevy_egui::{egui, EguiContexts, EguiPlugin, EguiPrimaryContextPass};

fn main() {
    App::new()
        .insert_resource(ClearColor(
            scene::SceneStyle::default().settings().background,
        ))
        .init_resource::<GlobalState>()
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
            instancing::InstancingPlugin,
            gpu_flock::GpuFlockPlugin,
            vehicle::VehiclePlugin,
            target::TargetPlugin,
            scene::ScenePlugin,
            vehicle_model::VehicleModelPlugin,
            scenario::ScenarioPlugin,
            presentation::PresentationPlugin,
            follow::FollowPlugin,
            capture::CapturePlugin,
            swarm::SwarmPlugin,
        ))
        .add_systems(
            PreUpdate,
            camera_input
                .after(PickingSystems::Last)
                .before(PanOrbitCamera::update_camera_positions),
        )
        .add_systems(Update, (fps_update_system, sync_ui_visibility))
        .add_systems(EguiPrimaryContextPass, ui)
        .run();
}

fn sync_ui_visibility(
    presentation: Res<presentation::Presentation>,
    mut roots: Query<&mut Visibility, With<Text>>,
) {
    if presentation.is_changed() {
        for mut visibility in &mut roots {
            *visibility = if presentation.ui_visible {
                Visibility::Inherited
            } else {
                Visibility::Hidden
            };
        }
    }
}

// The built-in controller deliberately leaves input mapping to the application.
fn camera_input(
    buttons: Res<ButtonInput<MouseButton>>,
    motion: Res<AccumulatedMouseMotion>,
    scroll: Res<AccumulatedMouseScroll>,
    mut contexts: EguiContexts,
    mut cameras: Query<&mut PanOrbitCamera>,
    mut previous_mode: Local<u8>,
    presentation: Option<Res<presentation::Presentation>>,
) {
    let blocked = contexts
        .ctx_mut()
        .is_ok_and(|ctx| ctx.egui_wants_pointer_input());
    let following = presentation.is_some_and(|p| p.follow.enabled() && !p.director.enabled);
    let mode = if blocked {
        0
    } else if buttons.pressed(MouseButton::Right) {
        1
    } else if buttons.pressed(MouseButton::Middle) {
        2
    } else if !following && scroll.delta.y != 0.0 {
        3
    } else {
        0
    };
    for mut camera in &mut cameras {
        // Framing resets the controller while a drag can still be held.
        if mode != *previous_mode || (mode != 0 && !camera.current_motion.is_user_controlled()) {
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
        if !blocked && !following && scroll.delta.y != 0.0 {
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

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum SimulationMode {
    /// Every vehicle checks every other one on the CPU.
    Naive,
    /// CPU neighbor search through an octree.
    Octree,
    /// Compute shaders; vehicles exist only on the GPU (see `gpu_flock`).
    Gpu,
}

impl SimulationMode {
    // The CPU modes stall beyond this; the GPU mode is limited by GPU memory and time.
    fn max_vehicles(self) -> usize {
        match self {
            SimulationMode::Naive | SimulationMode::Octree => 50_000,
            SimulationMode::Gpu => 1_000_000,
        }
    }
}

#[derive(Resource, Clone, Debug, PartialEq)]
struct GlobalState {
    attractor: swarm::Attractor,
    groups: swarm::Groups,
    motion: motion::MotionSettings,

    // Basic info
    vehicle_count: usize,
    vehicle_size: f32,
    vehicle_mass: f32,
    vehicle_wander_speed: f32,

    // Limits
    vehicle_max_speed: f32,

    // Factors
    vehicle_separation_factor: f32,
    vehicle_alignment_factor: f32,
    vehicle_cohesion_factor: f32,
    vehicle_wander_factor: f32,
    vehicle_wall_avoid_factor: f32,
    vehicle_seek_factor: f32,

    // Distances
    vehicle_separation_distance: f32,
    vehicle_alignment_distance: f32,
    vehicle_cohesion_distance: f32,
    vehicle_wander_distance: f32,
    vehicle_wander_radius: f32,

    mode: SimulationMode,
    // GPU follow-mode alignment/cohesion samples; separation stays complete. Zero is unlimited.
    gpu_max_neighbors: u32,
    octree_size: usize,
}

impl Default for GlobalState {
    fn default() -> Self {
        Self {
            attractor: default(),
            groups: default(),
            motion: default(),
            vehicle_count: 20_000,
            vehicle_size: 1.0,
            vehicle_mass: 60.0,
            vehicle_wander_speed: 40.0,
            vehicle_max_speed: 80.0,
            vehicle_separation_factor: 1.5,
            vehicle_alignment_factor: 3.0,
            vehicle_cohesion_factor: 4.0,
            vehicle_wander_factor: 1.0,
            vehicle_wall_avoid_factor: 1.5,
            vehicle_seek_factor: 7.0,
            vehicle_separation_distance: 4.0,
            vehicle_alignment_distance: 30.0,
            vehicle_cohesion_distance: 20.0,
            vehicle_wander_distance: 4.0,
            vehicle_wander_radius: 1.5,
            mode: SimulationMode::Gpu,
            gpu_max_neighbors: 16,
            octree_size: 100,
        }
    }
}

fn ui(
    mut egui_context: EguiContexts,
    mut state: ResMut<GlobalState>,
    targets: Query<&target::TargetSelection, With<target::Target>>,
    mut appearance: ResMut<scene::Appearance>,
    mut models: ResMut<vehicle_model::VehicleModels>,
    mut presentation: ResMut<presentation::Presentation>,
    mut swarm_ui: swarm::SwarmUi,
) -> Result {
    appearance.count_editing = false;
    if !presentation.ui_visible {
        return Ok(());
    }
    let ctx = egui_context.ctx_mut()?;
    egui::Window::new("Menu")
        .default_size([340.0, 640.0])
        .vscroll(true)
        .max_height(ctx.content_rect().height() - 24.0)
        .show(ctx, |ui| {
            if models.status.starts_with("Error:") {
                ui.colored_label(egui::Color32::LIGHT_RED, &models.status);
            }
            egui::CollapsingHeader::new("Presentation").default_open(true).show(ui, |ui| {
                presentation::ui(ui, &mut presentation);
            });
            egui::CollapsingHeader::new("Simulation").default_open(true).show(ui, |ui| {
            ui.horizontal(|ui| {
                for (mode, label) in [
                    (SimulationMode::Gpu, "GPU"),
                    (SimulationMode::Octree, "Octree"),
                    (SimulationMode::Naive, "Naive"),
                ] {
                    ui.radio_value(&mut state.mode, mode, label);
                }
            });
            let max_vehicles = state.mode.max_vehicles();
            state.vehicle_count = state.vehicle_count.min(max_vehicles);
            ui.label("Vehicle count");
            appearance.count_editing = ui.add(
                egui::Slider::new(&mut state.vehicle_count, 0..=max_vehicles)
                    .logarithmic(true)
                    .text("count"),
            ).dragged();
            if state.mode == SimulationMode::Gpu {
                ui.add(
                    egui::Slider::new(&mut state.gpu_max_neighbors, 0..=512)
                        .text("follow neighbor samples (0 = all)"),
                )
                .on_hover_text("Limits alignment and cohesion while following. Wander uses the full query; separation always checks every nearby vehicle.");
            }
            ui.label("Vehicle mass");
            ui.add(egui::Slider::new(&mut state.vehicle_mass, 1.0..=1000.0).text("mass"));
            ui.label("Vehicle max speed");
            ui.add(egui::Slider::new(&mut state.vehicle_max_speed, 0.1..=100.0).text("max speed"));
            ui.label("Vehicle wander speed");
            ui.add(
                egui::Slider::new(&mut state.vehicle_wander_speed, 0.1..=100.0)
                    .text("wander speed"),
            );
            if state.mode == SimulationMode::Octree {
                ui.separator();
                ui.add(
                    egui::Slider::new(&mut state.octree_size, 2..=500)
                        .text("octree size")
                        .step_by(1.0),
                );
            }
            });
            egui::CollapsingHeader::new("Swarm").default_open(true).show(ui, |ui| {
                swarm::ui(ui, &mut state.attractor);
                ui.separator();
                let mut scene = swarm_ui.scene.clone();
                formation::ui(ui, &mut scene.formation, swarm_ui.orbit_focus(), swarm_ui.formation.morph);
                ui.separator();
                let mut groups = state.groups;
                swarm::groups_ui(ui, &mut groups);
                if groups != state.groups { state.groups = groups; }
                ui.separator();
                swarm::obstacles_ui(ui, &mut scene);
                ui.separator();
                predator::ui(ui, &mut scene.predators, state.groups.count, swarm_ui.orbit_focus());
                if scene != *swarm_ui.scene { *swarm_ui.scene = scene; }
            });
            egui::CollapsingHeader::new("Steering").show(ui, |ui| {
            ui.label("Vehicle separation factor");
            ui.add(
                egui::Slider::new(&mut state.vehicle_separation_factor, 0.1..=10.0)
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

            ui.label("Vehicle separation distance");
            ui.add(
                egui::Slider::new(&mut state.vehicle_separation_distance, 1.0..=100.0)
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
            egui::CollapsingHeader::new("Scene").default_open(true).show(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.label("Style");
                    for style in scene::SceneStyle::ALL {
                        ui.selectable_value(&mut appearance.style, style, style.name());
                    }
                });
                ui.horizontal(|ui| {
                    ui.label("Palette");
                    for palette in scene::HeadingPalette::ALL {
                        ui.selectable_value(&mut appearance.palette, palette, palette.name());
                    }
                });
                egui::ComboBox::from_label("Environment")
                    .selected_text(appearance.environment.name())
                    .show_ui(ui, |ui| {
                        for environment in environment::Environment::ALL {
                            ui.selectable_value(&mut appearance.environment, environment, environment.name());
                        }
                    });
                ui.checkbox(&mut appearance.shadows, "Shadows");
                ui.add(egui::Slider::new(&mut appearance.shadow_cap, 0..=1_000_000).text("Flock shadow cap"));
                if appearance.shadows && state.vehicle_count > appearance.shadow_cap {
                    ui.label("Flock shadows paused: vehicle count exceeds cap");
                }
                ui.checkbox(&mut appearance.ambient_occlusion, "Ambient occlusion");
                if appearance.ambient_occlusion { ui.label("Ambient occlusion uses MSAA Off"); }
                ui.checkbox(&mut appearance.auto_frame, "Auto-frame on count change");
                if ui.button("Frame world").clicked() {
                    appearance.frame_requested = true;
                }
            });
            egui::CollapsingHeader::new("Vehicles").default_open(true).show(ui, |ui| {
                vehicle_model::ui(ui, &mut models, &mut appearance.vehicles);
                vehicle_appearance::ui(ui, &mut appearance.vehicles);
                egui::CollapsingHeader::new("Motion").default_open(true).show(ui, |ui| {
                    ui.checkbox(&mut state.motion.classic_heading, "Classic heading");
                    ui.add(egui::Slider::new(&mut state.motion.bank, 0.0..=1.0).text("Banking"));
                    ui.add(egui::Slider::new(&mut state.motion.flap, 0.0..=1.5).text("Flap / Swim amplitude"));
                    ui.add(egui::Slider::new(&mut state.motion.frequency, 0.0..=3.0).text("speed → frequency"));
                });
                if appearance.style == scene::SceneStyle::Glow {
                    ui.add(egui::Slider::new(&mut appearance.glow, 0.0..=20.0).text("glow"));
                    ui.add(egui::Slider::new(&mut appearance.streak, 0.0..=3.0).text("streak"));
                }
            });
        });
    let following = targets.iter().any(|selection| selection.0);
    egui::Window::new("Controls")
        .anchor(egui::Align2::RIGHT_BOTTOM, [-12.0, -12.0])
        .resizable(false)
        .collapsible(false)
        .show(ctx, |ui| {
            if presentation.paused {
                ui.colored_label(
                    egui::Color32::YELLOW,
                    "PAUSED — target and camera remain live",
                );
            } else {
                ui.label(format!("Simulation: {:.2}×", presentation.speed));
            }
            if following {
                ui.colored_label(egui::Color32::LIGHT_GREEN, "Follow: ON (green target)");
                ui.label(if swarm_ui.scene.formation.enabled() {
                    "Formation overrides following; selection stays stored."
                } else {
                    "Vehicles follow the selected target."
                });
            } else {
                ui.colored_label(egui::Color32::LIGHT_RED, "Follow: OFF (red target)");
                ui.label(if swarm_ui.scene.formation.enabled() {
                    "Vehicles use formation slots."
                } else if state.groups.has_targets() {
                    "Vehicles use group targets where enabled."
                } else {
                    "Vehicles wander freely."
                });
            }
            ui.label(format!("Boid camera: {}", presentation.follow.mode.name()));
            ui.separator();
            egui::Grid::new("controls").show(ui, |ui| {
                for (input, action) in [
                    ("Hold F", "Apply cursor field (Swarm)"),
                    ("P / O", "Pause / auto-orbit"),
                    ("B / N", "Boid camera / next boid"),
                    ("C", "Toggle Director"),
                    (
                        "1 / 2 / 3 / 4 / 5",
                        "Front / side / top / overview / target",
                    ),
                    ("Left-click target", "Start following"),
                    ("Left-click background", "Stop following"),
                    ("Right-drag", "Orbit camera"),
                    ("Middle-drag", "Pan camera"),
                    ("Scroll", "Zoom / chase distance"),
                    ("W / A / S / D", "Move selected target"),
                    ("Space / Left Shift", "Move target up / down"),
                ] {
                    ui.label(input);
                    ui.label(action);
                    ui.end_row();
                }
            });
            ui.separator();
            if state.groups.count > 0 {
                ui.small(format!(
                    "Groups: {} — edit targets/colors in Swarm",
                    state.groups.count
                ));
            }
            ui.small("Target movement is relative to the camera.");
            ui.small(format!(
                "Cursor field: {} — plane through orbit focus",
                state.attractor.mode.name()
            ));
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
    fn startup_camera_frames_the_default_world_without_culling_the_target() {
        // Only assets and scene setup: no renderer, window plugin or native app runner.
        let mut app = App::new();
        app.add_plugins((TaskPoolPlugin::default(), AssetPlugin::default()))
            .init_asset::<bevy::text::Font>()
            .init_asset_loader::<bevy::text::FontLoader>()
            .init_resource::<GlobalState>()
            .init_resource::<scene::Appearance>()
            .add_systems(Startup, scene::setup_scene);
        app.update();
        let state = app.world().resource::<GlobalState>();
        assert_eq!(state.mode, SimulationMode::Gpu);
        assert_eq!(state.vehicle_count, 20_000);
        assert_eq!(state.gpu_max_neighbors, 16);
        let count = state.vehicle_count;
        let mut cameras = app
            .world_mut()
            .query_filtered::<(&Transform, &Projection, &Msaa), With<Camera3d>>();
        let (transform, projection, msaa) = cameras.single(app.world()).unwrap();
        assert_eq!(*msaa, Msaa::Sample4);
        assert!(transform.translation.length() > vehicle::walls(count).length());
        let Projection::Perspective(projection) = projection else {
            panic!("expected perspective camera");
        };
        // The target and the far side of the world must stay within the culling distance.
        assert!(projection.far > transform.translation.length() + vehicle::walls(count).length());
        assert!(
            (transform.rotation * Vec3::NEG_Z).dot(-transform.translation.normalize()) > 0.99999
        );
    }

    #[test]
    fn held_camera_drag_resumes_after_automatic_and_explicit_framing() {
        for button in [MouseButton::Right, MouseButton::Middle] {
            for automatic in [true, false] {
                let mut app = App::new();
                app.init_resource::<GlobalState>()
                    .init_resource::<scene::Appearance>()
                    .init_resource::<ButtonInput<MouseButton>>()
                    .init_resource::<AccumulatedMouseMotion>()
                    .init_resource::<AccumulatedMouseScroll>()
                    .init_resource::<EguiUserTextures>()
                    .init_resource::<Time<Real>>()
                    .add_message::<RequestRedraw>()
                    .add_systems(
                        Update,
                        (
                            camera_input,
                            PanOrbitCamera::update_camera_positions,
                            scene::frame_world,
                        )
                            .chain(),
                    );
                let eye = Vec3::new(0.0, 140.0, 450.0);
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
                        Transform::from_translation(eye).looking_at(Vec3::ZERO, Vec3::Y),
                    ))
                    .id();
                app.update();
                app.world_mut().resource_mut::<GlobalState>().vehicle_count = 200;
                app.world_mut()
                    .resource_mut::<scene::Appearance>()
                    .auto_frame = automatic;
                app.world_mut()
                    .resource_mut::<ButtonInput<MouseButton>>()
                    .press(button);
                app.world_mut()
                    .resource_mut::<AccumulatedMouseMotion>()
                    .delta = Vec2::new(50.0, 25.0);
                app.world_mut()
                    .resource_mut::<Time<Real>>()
                    .advance_by(Duration::from_millis(16));
                app.update();
                app.world_mut()
                    .resource_mut::<Time<Real>>()
                    .advance_by(Duration::from_millis(400));
                if !automatic {
                    app.world_mut()
                        .resource_mut::<scene::Appearance>()
                        .frame_requested = true;
                }
                app.update();
                assert!(!app
                    .world()
                    .get::<PanOrbitCamera>(camera)
                    .unwrap()
                    .current_motion
                    .is_user_controlled());
                let framed = *app.world().get::<Transform>(camera).unwrap();
                app.world_mut()
                    .resource_mut::<Time<Real>>()
                    .advance_by(Duration::from_millis(16));
                app.update();
                let moved = app.world().get::<Transform>(camera).unwrap();
                if button == MouseButton::Right {
                    assert!(
                        moved.rotation.angle_between(framed.rotation) > 0.001,
                        "held orbit stopped after framing (automatic={automatic})"
                    );
                } else {
                    assert!(
                        moved.translation.distance(framed.translation) > 0.001,
                        "held pan stopped after framing (automatic={automatic})"
                    );
                }
            }
        }
    }

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
