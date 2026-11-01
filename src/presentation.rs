//! Presentation controls use real time; vehicle simulation and animation share the dedicated virtual clock.
use std::time::Duration;

use bevy::{
    camera_controller::pan_orbit_camera::prelude::*,
    input::mouse::AccumulatedMouseScroll,
    prelude::*,
    render::{
        extract_resource::{ExtractResource, ExtractResourcePlugin},
        RenderApp,
    },
    time::update_virtual_time,
};
use bevy_egui::{egui, EguiContexts};

use crate::{scene, target::Target, vehicle, GlobalState};

// Existing frame-rate tests exercise 30 Hz. Round upward by less than a nanosecond.
pub const MIN_SPEED: f32 = 0.1;
pub const MAX_SPEED: f32 = 4.0;

#[derive(Resource, Default)]
pub(crate) struct SimulationClock(Time<Virtual>);
impl SimulationClock {
    pub(crate) fn elapsed_secs(&self) -> f32 {
        self.0.elapsed_secs()
    }
    pub(crate) fn delta_secs_f64(&self) -> f64 {
        self.0.delta_secs_f64()
    }
}

/// A frame's simulated duration, split equally for CPU and GPU integration.
#[derive(Resource, ExtractResource, Clone, Copy, Debug, Default)]
#[extract_app(RenderApp)]
pub(crate) struct SimulationFrame {
    pub steps: u32,
    pub step: Duration,
}

impl SimulationFrame {
    // Above 1×, split the frame so no step is longer than a 1× step at the same frame rate.
    // At 1× and below this is today's single step, and GPU work never grows as FPS falls.
    fn from_delta(delta: Duration, speed: f32) -> Self {
        let steps = if delta.is_zero() {
            0
        } else {
            speed.clamp(MIN_SPEED, MAX_SPEED).ceil() as u32
        };
        Self {
            steps,
            step: if steps == 0 {
                Duration::ZERO
            } else {
                delta / steps
            },
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CameraPreset {
    Front,
    Side,
    Top,
    Overview,
    CloseUp,
}

impl CameraPreset {
    pub const ALL: [Self; 5] = [
        Self::Front,
        Self::Side,
        Self::Top,
        Self::Overview,
        Self::CloseUp,
    ];
    pub fn name(self) -> &'static str {
        match self {
            Self::Front => "Front",
            Self::Side => "Side",
            Self::Top => "Top",
            Self::Overview => "Overview",
            Self::CloseUp => "Target close-up",
        }
    }
    pub fn parse(name: &str) -> Result<Self, String> {
        match name {
            "front" => Ok(Self::Front),
            "side" => Ok(Self::Side),
            "top" => Ok(Self::Top),
            "overview" => Ok(Self::Overview),
            "closeup" => Ok(Self::CloseUp),
            _ => Err(format!("invalid camera preset '{name}'")),
        }
    }
}

struct CameraPose {
    transform: Transform,
    distance: f32,
}

impl CameraPose {
    fn preset(
        preset: CameraPreset,
        count: usize,
        target: Vec3,
        projection: &PerspectiveProjection,
    ) -> Self {
        let (yaw, pitch): (f32, f32) = match preset {
            CameraPreset::Front => (0.0, 0.0),
            CameraPreset::Side => (90.0, 0.0),
            CameraPreset::Top => (0.0, 89.9),
            CameraPreset::Overview | CameraPreset::CloseUp => (35.0, 25.0),
        };
        let rotation = Quat::from_euler(EulerRot::YXZ, yaw.to_radians(), -pitch.to_radians(), 0.0);
        let close = preset == CameraPreset::CloseUp;
        let anchor = if close { target } else { Vec3::ZERO };
        let fit = scene::frame_distance(
            vehicle::walls(count),
            rotation,
            Vec3::ZERO,
            projection.aspect_ratio,
            projection.fov,
        );
        let distance = fit * if close { 0.3 } else { 1.0 };
        Self {
            transform: Transform::from_translation(anchor + rotation * Vec3::Z * distance)
                .with_rotation(rotation),
            distance,
        }
    }
}

struct CameraTransition {
    start: Transform,
    start_depth: f32,
    target: CameraPose,
    elapsed: f32,
}

#[derive(Resource)]
pub struct Presentation {
    pub paused: bool,
    pub speed: f32,
    pub orbit: bool,
    pub orbit_speed: f32,
    pub preset: Option<CameraPreset>,
    pub(crate) ui_visible: bool,
    pub(crate) follow: crate::follow::Follow,
    manual_camera: bool,
    transition: Option<CameraTransition>,
}

impl Default for Presentation {
    fn default() -> Self {
        Self {
            paused: false,
            speed: 1.0,
            orbit: false,
            orbit_speed: 10.0,
            preset: None,
            ui_visible: true,
            follow: default(),
            manual_camera: false,
            transition: None,
        }
    }
}

impl Presentation {
    pub fn cancel_camera_transition(&mut self) {
        self.transition = None;
        self.preset = None;
        self.follow.mode = crate::follow::FollowMode::Off;
    }
}

pub struct PresentationPlugin;
impl Plugin for PresentationPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<Presentation>()
            .init_resource::<SimulationClock>()
            .init_resource::<SimulationFrame>()
            .add_plugins(ExtractResourcePlugin::<SimulationFrame>::default())
            .add_systems(PreUpdate, presentation_input.after(crate::camera_input))
            .add_systems(
                Update,
                prepare_simulation
                    .before(crate::vehicle::step_cpu)
                    .before(crate::gpu_flock::update_gpu_params),
            )
            .add_systems(PostUpdate, animate_camera.before(scene::frame_world));
    }
}

pub(crate) fn prepare_simulation(
    presentation: Res<Presentation>,
    real: Res<Time<Real>>,
    mut clock: ResMut<SimulationClock>,
    mut frame: ResMut<SimulationFrame>,
) {
    clock.0.set_relative_speed(presentation.speed);
    if presentation.paused {
        clock.0.pause();
    } else {
        clock.0.unpause();
    }
    let mut generic = Time::default();
    update_virtual_time(&mut generic, &mut clock.0, &real);
    *frame = SimulationFrame::from_delta(clock.0.delta(), presentation.speed);
}

fn presentation_input(
    keys: Res<ButtonInput<KeyCode>>,
    buttons: Res<ButtonInput<MouseButton>>,
    scroll: Res<AccumulatedMouseScroll>,
    mut contexts: EguiContexts,
    mut presentation: ResMut<Presentation>,
) {
    let (keyboard, pointer) = contexts.ctx_mut().map_or((false, false), |ctx| {
        (
            ctx.egui_wants_keyboard_input(),
            ctx.egui_wants_pointer_input(),
        )
    });
    presentation.manual_camera = !pointer
        && (buttons.pressed(MouseButton::Right)
            || buttons.pressed(MouseButton::Middle)
            || (scroll.delta != Vec2::ZERO && !presentation.follow.enabled()));
    if presentation.manual_camera {
        presentation.follow.mode = crate::follow::FollowMode::Off;
    }
    if !pointer && presentation.follow.enabled() {
        presentation.follow.distance =
            (presentation.follow.distance * (-scroll.delta.y * 0.1).exp()).clamp(2.0, 80.0);
    }
    if keyboard {
        return;
    }
    if keys.just_pressed(KeyCode::KeyB) {
        presentation.follow.mode = presentation.follow.mode.cycle();
        presentation.transition = None;
    }
    if keys.just_pressed(KeyCode::KeyN) {
        presentation.follow.next();
    }
    if keys.just_pressed(KeyCode::KeyP) {
        presentation.paused = !presentation.paused;
    }
    if keys.just_pressed(KeyCode::KeyO) {
        presentation.orbit = !presentation.orbit;
    }
    for (key, preset) in [
        KeyCode::Digit1,
        KeyCode::Digit2,
        KeyCode::Digit3,
        KeyCode::Digit4,
        KeyCode::Digit5,
    ]
    .into_iter()
    .zip(CameraPreset::ALL)
    {
        if keys.just_pressed(key) {
            presentation.follow.mode = crate::follow::FollowMode::Off;
            presentation.preset = Some(preset);
        }
    }
}

pub(crate) fn animate_camera(
    mut presentation: ResMut<Presentation>,
    real: Res<Time<Real>>,
    state: Res<GlobalState>,
    targets: Query<&Transform, (With<Target>, Without<Camera3d>)>,
    mut cameras: Query<(&mut Transform, &mut PanOrbitCamera, &Projection), With<Camera3d>>,
) {
    if presentation.follow.enabled() {
        return;
    }
    if presentation.manual_camera {
        presentation.transition = None;
        return;
    }
    let Ok((mut transform, mut orbit, projection)) = cameras.single_mut() else {
        return;
    };
    let Projection::Perspective(projection) = projection else {
        return;
    };
    if let Some(preset) = presentation.preset.take() {
        let target = targets.single().map_or(Vec3::ZERO, |t| t.translation);
        presentation.transition = Some(CameraTransition {
            start: *transform,
            start_depth: orbit.last_anchor_depth().abs() as f32,
            target: CameraPose::preset(preset, state.vehicle_count, target, projection),
            elapsed: 0.0,
        });
        orbit.current_motion = default();
    }
    if let Some(transition) = &mut presentation.transition {
        transition.elapsed += real.delta_secs();
        let t = (transition.elapsed / 0.6).min(1.0);
        let smooth = transition_smoothstep(transition.elapsed);
        transform.translation = transition
            .start
            .translation
            .lerp(transition.target.transform.translation, smooth);
        transform.rotation = transition
            .start
            .rotation
            .slerp(transition.target.transform.rotation, smooth);
        orbit.last_anchor_depth = -(transition
            .start_depth
            .lerp(transition.target.distance, smooth) as f64);
        if t >= 1.0 {
            presentation.transition = None;
        }
        return;
    }
    if presentation.orbit {
        orbit.current_motion = default();
        let focus =
            transform.translation + transform.forward() * orbit.last_anchor_depth().abs() as f32;
        let yaw = Quat::from_rotation_y(presentation.orbit_speed.to_radians() * real.delta_secs());
        transform.translation = focus + yaw * (transform.translation - focus);
        transform.rotation = yaw * transform.rotation;
    }
}

pub(crate) fn transition_smoothstep(elapsed: f32) -> f32 {
    let t = (elapsed / 0.6).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

pub fn ui(ui: &mut egui::Ui, presentation: &mut Presentation) {
    ui.horizontal(|ui| {
        if ui
            .button(if presentation.paused {
                "Resume (P)"
            } else {
                "Pause (P)"
            })
            .clicked()
        {
            presentation.paused = !presentation.paused;
        }
        if presentation.paused {
            ui.colored_label(egui::Color32::YELLOW, "PAUSED");
        }
    });
    ui.add(egui::Slider::new(&mut presentation.speed, MIN_SPEED..=MAX_SPEED).text("simulation ×"));
    ui.horizontal(|ui| {
        for speed in [0.25, 0.5, 1.0, 2.0] {
            if ui.button(format!("{speed}×")).clicked() {
                presentation.speed = speed;
            }
        }
    });
    let previous_follow = presentation.follow.mode;
    ui.horizontal(|ui| {
        ui.label("Boid camera (B)");
        egui::ComboBox::from_id_salt("follow_mode")
            .selected_text(presentation.follow.mode.name())
            .show_ui(ui, |ui| {
                for mode in crate::follow::FollowMode::ALL {
                    ui.selectable_value(&mut presentation.follow.mode, mode, mode.name());
                }
            });
        if ui.button("Next boid (N)").clicked() {
            presentation.follow.next();
        }
    });
    ui.horizontal(|ui| {
        ui.checkbox(&mut presentation.follow.dof, "Depth of field");
        ui.add(
            egui::Slider::new(&mut presentation.follow.aperture, 0.7..=16.0)
                .logarithmic(true)
                .text("aperture f-stop"),
        )
        .on_hover_text("Lower f-stop gives stronger blur; applies only in the boid camera.");
    });
    if previous_follow != presentation.follow.mode {
        presentation.transition = None;
        presentation.preset = None;
    }
    if let Some(id) = presentation
        .follow
        .id
        .filter(|_| presentation.follow.enabled())
    {
        ui.label(format!("Boid ID: {id}"));
    }
    ui.add(egui::Slider::new(&mut presentation.follow.distance, 2.0..=80.0).text("chase distance"));
    ui.checkbox(&mut presentation.orbit, "Auto-orbit (O)");
    ui.add(egui::Slider::new(&mut presentation.orbit_speed, -45.0..=45.0).text("orbit °/s"));
    ui.horizontal_wrapped(|ui| {
        for (index, preset) in CameraPreset::ALL.into_iter().enumerate() {
            if ui
                .button(format!("{} {}", index + 1, preset.name()))
                .clicked()
            {
                presentation.follow.mode = crate::follow::FollowMode::Off;
                presentation.preset = Some(preset);
            }
        }
    });
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    pub(crate) fn clock_app() -> App {
        let mut app = App::new();
        app.init_resource::<Presentation>()
            .init_resource::<SimulationClock>()
            .init_resource::<SimulationFrame>()
            .init_resource::<Time<Real>>()
            .init_resource::<Time>()
            .add_systems(Update, prepare_simulation);
        app
    }

    #[test]
    fn follow_shortcuts_cycle_pick_scroll_and_exit() {
        use crate::follow::FollowMode;
        let mut app = App::new();
        app.init_resource::<Presentation>()
            .init_resource::<ButtonInput<KeyCode>>()
            .init_resource::<ButtonInput<MouseButton>>()
            .init_resource::<AccumulatedMouseScroll>()
            .init_resource::<bevy_egui::EguiUserTextures>()
            .add_systems(Update, presentation_input);
        for expected in [FollowMode::Chase, FollowMode::FirstPerson, FollowMode::Off] {
            app.world_mut()
                .resource_mut::<ButtonInput<KeyCode>>()
                .press(KeyCode::KeyB);
            app.update();
            assert_eq!(app.world().resource::<Presentation>().follow.mode, expected);
            app.world_mut()
                .resource_mut::<ButtonInput<KeyCode>>()
                .reset_all();
        }
        app.world_mut().resource_mut::<Presentation>().follow.mode = FollowMode::Chase;
        app.world_mut()
            .resource_mut::<Presentation>()
            .follow
            .select(123);
        app.world_mut()
            .resource_mut::<ButtonInput<KeyCode>>()
            .press(KeyCode::KeyN);
        app.update();
        assert_eq!(app.world().resource::<Presentation>().follow.id, None);
        app.world_mut()
            .resource_mut::<ButtonInput<KeyCode>>()
            .reset_all();
        let distance = app.world().resource::<Presentation>().follow.distance;
        app.world_mut()
            .resource_mut::<AccumulatedMouseScroll>()
            .delta
            .y = 1.0;
        app.update();
        assert!(app.world().resource::<Presentation>().follow.distance < distance);
        assert_eq!(
            app.world().resource::<Presentation>().follow.mode,
            FollowMode::Chase
        );
        app.world_mut()
            .resource_mut::<AccumulatedMouseScroll>()
            .delta = Vec2::ZERO;
        for key in [
            KeyCode::Digit1,
            KeyCode::Digit2,
            KeyCode::Digit3,
            KeyCode::Digit4,
            KeyCode::Digit5,
        ] {
            app.world_mut().resource_mut::<Presentation>().follow.mode = FollowMode::FirstPerson;
            app.world_mut()
                .resource_mut::<ButtonInput<KeyCode>>()
                .press(key);
            app.update();
            assert_eq!(
                app.world().resource::<Presentation>().follow.mode,
                FollowMode::Off
            );
            app.world_mut()
                .resource_mut::<ButtonInput<KeyCode>>()
                .reset_all();
        }
        for button in [MouseButton::Right, MouseButton::Middle] {
            app.world_mut().resource_mut::<Presentation>().follow.mode = FollowMode::Chase;
            app.world_mut()
                .resource_mut::<ButtonInput<MouseButton>>()
                .press(button);
            app.update();
            assert_eq!(
                app.world().resource::<Presentation>().follow.mode,
                FollowMode::Off
            );
            app.world_mut()
                .resource_mut::<ButtonInput<MouseButton>>()
                .reset_all();
        }
    }

    #[test]
    fn virtual_clock_scales_pauses_and_bounds_substeps() {
        let mut app = clock_app();
        for speed in [0.1, 0.25, 0.5, 1.0, 2.0, 4.0] {
            app.world_mut().resource_mut::<Presentation>().speed = speed;
            app.world_mut()
                .resource_mut::<Time<Real>>()
                .advance_by(Duration::from_millis(20));
            app.update();
            let frame = *app.world().resource::<SimulationFrame>();
            let expected = Duration::from_millis(20).mul_f64(speed as f64);
            assert!(frame.step <= Duration::from_millis(20));
            assert_eq!(frame.steps, speed.ceil() as u32);
            assert!((frame.step * frame.steps).abs_diff(expected) <= Duration::from_nanos(8));
        }
        app.world_mut().resource_mut::<Presentation>().paused = true;
        let elapsed = app.world().resource::<SimulationClock>().0.elapsed();
        for _ in 0..100 {
            app.world_mut()
                .resource_mut::<Time<Real>>()
                .advance_by(Duration::from_secs(1));
            app.update();
            assert_eq!(app.world().resource::<SimulationFrame>().steps, 0);
            assert_eq!(
                app.world().resource::<SimulationClock>().0.elapsed(),
                elapsed
            );
        }
        app.world_mut().resource_mut::<Presentation>().paused = false;
        app.update();
        let frame = app.world().resource::<SimulationFrame>();
        assert_eq!(frame.steps, 4);
        assert_eq!(frame.step * frame.steps, Duration::from_millis(250));
        app.world_mut().resource_mut::<Presentation>().speed = 1.0;
        app.update();
        let frame = app.world().resource::<SimulationFrame>();
        assert_eq!(frame.steps, 1);
        assert_eq!(frame.step, Duration::from_millis(250));
    }

    #[test]
    fn presets_frame_scaled_world_and_close_up_centers_target() {
        let projection = PerspectiveProjection {
            aspect_ratio: 16.0 / 9.0,
            ..default()
        };
        let target = Vec3::new(240.0, -20.0, 70.0);
        for preset in CameraPreset::ALL {
            let reference = CameraPose::preset(preset, 200, target, &projection);
            for count in [200, 20_000, 100_000] {
                let pose = CameraPose::preset(preset, count, target, &projection);
                assert!(
                    (pose.distance / reference.distance - vehicle::world_scale(count)).abs() < 1e-5
                );
                let focus = pose.transform.translation + pose.transform.forward() * pose.distance;
                assert!(
                    focus.distance(if preset == CameraPreset::CloseUp {
                        target
                    } else {
                        Vec3::ZERO
                    }) < 0.001
                );
                if preset != CameraPreset::CloseUp {
                    let fit = scene::frame_distance(
                        vehicle::walls(count),
                        pose.transform.rotation,
                        focus,
                        projection.aspect_ratio,
                        projection.fov,
                    );
                    assert!((fit - pose.distance).abs() < 0.002);
                }
            }
        }
    }

    #[test]
    fn manual_camera_defers_presets_and_orbit_then_resumes_on_real_time() {
        let mut app = App::new();
        app.init_resource::<Presentation>()
            .init_resource::<GlobalState>()
            .init_resource::<Time<Real>>()
            .init_resource::<ButtonInput<KeyCode>>()
            .init_resource::<ButtonInput<MouseButton>>()
            .init_resource::<AccumulatedMouseScroll>()
            .init_resource::<bevy_egui::EguiUserTextures>()
            .add_systems(Update, (presentation_input, animate_camera).chain());
        let camera = app
            .world_mut()
            .spawn((
                Camera3d::default(),
                Transform::from_xyz(0.0, 0.0, 1000.0),
                Projection::Perspective(default()),
                PanOrbitCamera::default().with_initial_anchor_depth(1000.0),
            ))
            .id();
        for key in [KeyCode::KeyP, KeyCode::KeyO, KeyCode::Digit2] {
            app.world_mut()
                .resource_mut::<ButtonInput<KeyCode>>()
                .press(key);
        }
        app.world_mut()
            .resource_mut::<ButtonInput<MouseButton>>()
            .press(MouseButton::Right);
        app.world_mut()
            .resource_mut::<Time<Real>>()
            .advance_by(Duration::from_millis(300));
        app.update();
        assert_eq!(
            app.world().get::<Transform>(camera).unwrap().translation,
            Vec3::Z * 1000.0
        );
        assert_eq!(
            app.world().resource::<Presentation>().preset,
            Some(CameraPreset::Side)
        );
        assert!(app.world().resource::<Presentation>().paused);
        assert!(app.world().resource::<Presentation>().orbit);
        app.world_mut()
            .resource_mut::<ButtonInput<KeyCode>>()
            .reset_all();
        app.world_mut()
            .resource_mut::<ButtonInput<MouseButton>>()
            .release(MouseButton::Right);
        app.update();
        assert!(app.world().resource::<Presentation>().transition.is_some());
        app.update();
        assert!(app.world().resource::<Presentation>().transition.is_none());
        let before = *app.world().get::<Transform>(camera).unwrap();
        app.update();
        let after = app.world().get::<Transform>(camera).unwrap();
        assert!(
            (before.rotation.angle_between(after.rotation) - 3.0_f32.to_radians()).abs() < 1e-4
        );
        assert!((after.translation.length() - before.translation.length()).abs() < 0.001);
        let after = *after;
        app.world_mut()
            .resource_mut::<ButtonInput<MouseButton>>()
            .press(MouseButton::Middle);
        app.update();
        assert_eq!(*app.world().get::<Transform>(camera).unwrap(), after);

        // A focused model-path-style text edit must consume presentation shortcuts.
        let mut context = bevy_egui::EguiContext::default();
        let ctx = context.get_mut();
        let mut text = String::from("model.glb");
        let _ = ctx.run_ui(egui::RawInput::default(), |ui| {
            ui.text_edit_singleline(&mut text).request_focus();
        });
        assert!(ctx.egui_wants_keyboard_input());
        app.world_mut()
            .spawn((context, bevy_egui::PrimaryEguiContext));
        app.world_mut()
            .resource_mut::<ButtonInput<KeyCode>>()
            .press(KeyCode::KeyP);
        app.world_mut()
            .resource_mut::<ButtonInput<KeyCode>>()
            .press(KeyCode::KeyO);
        app.update();
        assert!(app.world().resource::<Presentation>().paused);
        assert!(app.world().resource::<Presentation>().orbit);
    }
}
