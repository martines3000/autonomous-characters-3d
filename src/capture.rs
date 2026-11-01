//! One screenshot in flight: advance once, capture, freeze until the PNG is saved.
use std::{path::PathBuf, time::Duration};

use bevy::{
    app::AppExit,
    camera_controller::pan_orbit_camera::{
        controller::{projections, transform_utils::apply_delta},
        prelude::PanOrbitCamera,
        SyncCameraPosition,
    },
    prelude::*,
    render::view::screenshot::{Screenshot, ScreenshotCaptured},
    window::RequestRedraw,
};

#[derive(Resource, Default)]
pub(crate) struct Capture {
    session: Option<Recording>,
}
struct Recording {
    directory: PathBuf,
    frames: u32,
    saved: u32,
    dt: Duration,
    pending: Option<Entity>,
    capture_this_frame: bool,
}
impl Capture {
    pub(crate) fn active(&self) -> bool {
        self.session.is_some()
    }
    pub(crate) fn start(
        &mut self,
        directory: PathBuf,
        frames: u32,
        fps: u32,
    ) -> Result<(), String> {
        if self.active() {
            return Err("a recording is already active".into());
        }
        if !directory.is_absolute() || frames == 0 || !(1..=240).contains(&fps) {
            return Err(
                "record requires an absolute directory, positive frames and fps 1..=240".into(),
            );
        }
        std::fs::create_dir_all(&directory).map_err(|e| format!("record directory: {e}"))?;
        for index in 0..frames {
            if directory.join(format!("{index:05}.png")).exists() {
                return Err(format!("record refuses existing frame {index:05}.png"));
            }
        }
        self.session = Some(Recording {
            directory,
            frames,
            saved: 0,
            dt: Duration::from_secs_f64(1.0 / fps as f64),
            pending: None,
            capture_this_frame: false,
        });
        Ok(())
    }
    /// None means normal wall time. Some(0) means an asynchronous capture is pending.
    fn next_delta(&self) -> Option<Duration> {
        let session = self.session.as_ref()?;
        Some(
            if session.pending.is_none()
                && !session.capture_this_frame
                && session.saved < session.frames
            {
                session.dt
            } else {
                Duration::ZERO
            },
        )
    }
    pub(crate) fn frame_delta(&mut self) -> Option<Duration> {
        let delta = self.next_delta()?;
        if !delta.is_zero() {
            self.session.as_mut().unwrap().capture_this_frame = true;
        }
        Some(delta)
    }
    fn complete(&mut self, entity: Entity) {
        let Some(session) = &mut self.session else {
            return;
        };
        if session.pending == Some(entity) {
            session.pending = None;
            session.capture_this_frame = false;
            session.saved += 1;
        }
    }
}

pub(crate) struct CapturePlugin;
impl Plugin for CapturePlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<Capture>()
            .add_message::<RequestRedraw>()
            // The stock controller reads wall time. Use its public update API with
            // the capture delta, including for manual camera momentum.
            .configure_sets(PreUpdate, SyncCameraPosition.run_if(not_recording))
            .add_systems(
                PreUpdate,
                (
                    fixed_manual_camera,
                    projections::update_orthographic,
                    projections::update_perspective,
                )
                    .chain()
                    .after(crate::camera_input)
                    .before(SyncCameraPosition)
                    .run_if(recording),
            )
            .add_systems(Last, request_frame);
    }
}
fn recording(capture: Res<Capture>) -> bool {
    capture.active()
}
fn not_recording(capture: Res<Capture>) -> bool {
    !capture.active()
}
fn fixed_manual_camera(
    capture: Res<Capture>,
    mut cameras: Query<(
        &mut Transform,
        &Camera,
        &mut Projection,
        &mut PanOrbitCamera,
    )>,
    mut redraw: MessageWriter<RequestRedraw>,
) {
    let Some(dt) = capture.next_delta().filter(|dt| !dt.is_zero()) else {
        return;
    };
    for (mut transform, camera, projection, mut orbit) in &mut cameras {
        if let Some((translation, rotation)) = orbit.update_transform_and_projection(
            camera,
            &transform.translation.as_dvec3(),
            &transform.rotation.as_dquat(),
            projection,
            &mut redraw,
            dt,
        ) {
            apply_delta(&mut transform, translation, rotation);
        }
    }
}
fn request_frame(
    mut commands: Commands,
    mut capture: ResMut<Capture>,
    screenshots: Query<(), With<Screenshot>>,
) {
    let Some(session) = &mut capture.session else {
        return;
    };
    if session.saved == session.frames {
        info!(
            "record complete: {} frames in {}",
            session.frames,
            session.directory.display()
        );
        capture.session = None;
        return;
    }
    // Bevy accepts only one screenshot per render target per frame. Drain any
    // ordinary scenario screenshot before submitting the already-stepped frame.
    if session.capture_this_frame && screenshots.is_empty() {
        session.pending = Some(
            commands
                .spawn(Screenshot::primary_window())
                .observe(save_frame)
                .id(),
        );
        session.capture_this_frame = false;
    }
}
fn save_frame(
    event: On<ScreenshotCaptured>,
    mut capture: ResMut<Capture>,
    mut exit: MessageWriter<AppExit>,
) {
    let Some(session) = &capture.session else {
        return;
    };
    if session.pending != Some(event.entity) {
        return;
    }
    let path = session.directory.join(format!("{:05}.png", session.saved));
    // Match Bevy's save_to_disk conversion: discard the window's brightness alpha channel.
    // Save before acknowledging, so script exit cannot race encoding or disk writes.
    let result = event
        .image
        .clone()
        .try_into_dynamic()
        .map_err(|e| format!("{e}"))
        .and_then(|image| image.to_rgb8().save(&path).map_err(|e| e.to_string()));
    match result {
        Ok(()) => capture.complete(event.entity),
        Err(error) => {
            error!("record {}: {error}", path.display());
            exit.write(AppExit::error());
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::presentation::{Presentation, SimulationClock};

    #[test]
    fn manual_camera_inertia_uses_capture_delta_and_holds_during_readback() {
        use bevy::camera::{CameraProjection, ComputedCameraValues, RenderTargetInfo};
        use bevy::camera_controller::pan_orbit_camera::controller::{
            momentum::Velocity, motion::CurrentMotion, MinimalPanOrbitCameraPlugin,
        };
        use bevy::math::{DVec2, DVec3};
        let run = |wall: Duration| {
            let mut app = crate::presentation::tests::clock_app();
            app.add_plugins((MinimalPanOrbitCameraPlugin, CapturePlugin))
                .add_message::<AppExit>();
            let projection = PerspectiveProjection::default();
            let mut orbit = PanOrbitCamera::default().with_initial_anchor_depth(100.0);
            orbit.current_motion = CurrentMotion::Momentum {
                velocity: Velocity::Orbit {
                    anchor: DVec3::NEG_Z * 100.0,
                    velocity: DVec2::new(10.0, 5.0),
                },
                momentum_start: bevy::platform::time::Instant::now(),
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
                    Transform::from_xyz(0.0, 0.0, 100.0),
                    Projection::Perspective(projection),
                    orbit,
                ))
                .id();
            let dir = std::env::temp_dir().join(format!("ac3d-inertia-{}", std::process::id()));
            app.world_mut()
                .resource_mut::<Capture>()
                .start(dir.clone(), 3, 30)
                .unwrap();
            for _ in 0..3 {
                app.world_mut()
                    .resource_mut::<Time<Real>>()
                    .advance_by(wall);
                app.update();
                let held = *app.world().get::<Transform>(camera).unwrap();
                app.update();
                assert_eq!(*app.world().get::<Transform>(camera).unwrap(), held);
                let entity = app
                    .world()
                    .resource::<Capture>()
                    .session
                    .as_ref()
                    .unwrap()
                    .pending
                    .unwrap();
                app.world_mut().resource_mut::<Capture>().complete(entity);
                app.world_mut().despawn(entity);
            }
            std::fs::remove_dir(dir).unwrap();
            *app.world().get::<Transform>(camera).unwrap()
        };
        let fast = run(Duration::from_millis(3));
        let slow = run(Duration::from_millis(200));
        assert_eq!(fast, slow);
        assert_ne!(fast.translation, Vec3::Z * 100.0);
    }

    #[test]
    fn recording_drains_other_screenshots_without_advancing_an_extra_step() {
        let mut app = crate::presentation::tests::clock_app();
        app.add_plugins(CapturePlugin).add_message::<AppExit>();
        let dir = std::env::temp_dir().join(format!("ac3d-drain-{}", std::process::id()));
        app.world_mut()
            .resource_mut::<Capture>()
            .start(dir.clone(), 1, 30)
            .unwrap();
        let ordinary = app.world_mut().spawn(Screenshot::primary_window()).id();
        app.update();
        let elapsed = app.world().resource::<SimulationClock>().0.elapsed();
        assert_eq!(elapsed, Duration::from_secs_f64(1.0 / 30.0));
        for _ in 0..3 {
            app.update();
            assert_eq!(
                app.world().resource::<SimulationClock>().0.elapsed(),
                elapsed
            );
            assert!(app
                .world()
                .resource::<Capture>()
                .session
                .as_ref()
                .unwrap()
                .pending
                .is_none());
        }
        app.world_mut().despawn(ordinary);
        app.update();
        assert_eq!(
            app.world().resource::<SimulationClock>().0.elapsed(),
            elapsed
        );
        assert!(app
            .world()
            .resource::<Capture>()
            .session
            .as_ref()
            .unwrap()
            .pending
            .is_some());
        std::fs::remove_dir(dir).unwrap();
    }

    #[test]
    fn screenshot_observer_saves_in_order_and_ignores_duplicate_and_stale_callbacks() {
        let mut app = crate::presentation::tests::clock_app();
        app.add_plugins(CapturePlugin).add_message::<AppExit>();
        let dir = std::env::temp_dir().join(format!("ac3d-png-{}", std::process::id()));
        app.world_mut()
            .resource_mut::<Capture>()
            .start(dir.clone(), 3, 30)
            .unwrap();
        for index in 0..3 {
            app.update();
            let entity = app
                .world()
                .resource::<Capture>()
                .session
                .as_ref()
                .unwrap()
                .pending
                .unwrap();
            // Simulate variable readback latency while both clocks hold the captured state.
            app.update();
            app.update();
            let stale = app.world_mut().spawn_empty().id();
            app.world_mut().trigger(ScreenshotCaptured {
                entity: stale,
                image: Image::default(),
            });
            assert_eq!(
                app.world()
                    .resource::<Capture>()
                    .session
                    .as_ref()
                    .unwrap()
                    .saved,
                index
            );
            for _ in 0..2 {
                app.world_mut().trigger(ScreenshotCaptured {
                    entity,
                    image: Image::default(),
                });
            }
            assert_eq!(
                app.world()
                    .resource::<Capture>()
                    .session
                    .as_ref()
                    .unwrap()
                    .saved,
                index + 1
            );
            app.world_mut().despawn(entity);
            let bytes = std::fs::read(dir.join(format!("{index:05}.png"))).unwrap();
            assert_eq!(&bytes[..8], b"\x89PNG\r\n\x1a\n");
        }
        app.update();
        assert!(!app.world().resource::<Capture>().active());
        assert_eq!(std::fs::read_dir(&dir).unwrap().count(), 3);
        assert!(app
            .world_mut()
            .resource_mut::<Capture>()
            .start(dir.clone(), 3, 30)
            .is_err());
        std::fs::remove_dir_all(dir).unwrap();
    }

    #[test]
    fn fixed_recording_clock_advances_once_per_acknowledged_frame_then_resumes() {
        let mut app = crate::presentation::tests::clock_app();
        app.init_resource::<Capture>()
            .init_resource::<crate::GlobalState>()
            .add_systems(
                Update,
                crate::presentation::animate_camera.after(crate::presentation::prepare_simulation),
            );
        app.world_mut().resource_mut::<Presentation>().orbit = true;
        let camera = app
            .world_mut()
            .spawn((
                Camera3d::default(),
                Transform::from_xyz(0.0, 0.0, 100.0),
                Projection::default(),
                bevy::camera_controller::pan_orbit_camera::prelude::PanOrbitCamera::default()
                    .with_initial_anchor_depth(100.0),
            ))
            .id();
        let dir = std::env::temp_dir().join(format!("ac3d-clock-{}", std::process::id()));
        app.world_mut()
            .resource_mut::<Capture>()
            .start(dir.clone(), 90, 30)
            .unwrap();
        let dt = Duration::from_secs_f64(1.0 / 30.0);
        let initial = app.world().resource::<SimulationClock>().0.elapsed();
        let mut simulated = Duration::ZERO;
        for frame in 0..90 {
            app.world_mut()
                .resource_mut::<Time<Real>>()
                .advance_by(Duration::from_millis(3 + frame));
            app.update();
            simulated += dt;
            assert_eq!(
                app.world().resource::<SimulationClock>().0.elapsed(),
                initial + simulated
            );
            assert_eq!(
                app.world().resource::<Presentation>().camera_delta,
                Some(dt.as_secs_f32())
            );
            let entity = app.world_mut().spawn_empty().id();
            app.world_mut()
                .resource_mut::<Capture>()
                .session
                .as_mut()
                .unwrap()
                .pending = Some(entity);
            let held = *app.world().get::<Transform>(camera).unwrap();
            for _ in 0..3 {
                app.world_mut()
                    .resource_mut::<Time<Real>>()
                    .advance_by(Duration::from_secs(2));
                app.update();
                assert_eq!(
                    app.world().resource::<SimulationClock>().0.elapsed(),
                    initial + simulated
                );
                assert_eq!(
                    app.world().resource::<Presentation>().camera_delta,
                    Some(0.0)
                );
                assert_eq!(*app.world().get::<Transform>(camera).unwrap(), held);
            }
            let mut capture = app.world_mut().resource_mut::<Capture>();
            capture.complete(entity);
            capture.complete(entity); // Duplicate callbacks cannot advance output numbering.
            assert_eq!(capture.session.as_ref().unwrap().saved, frame as u32 + 1);
        }
        assert_eq!(simulated, dt * 90);
        let expected = Quat::from_rotation_y(30.0_f32.to_radians()) * Vec3::Z * 100.0;
        assert!(
            app.world()
                .get::<Transform>(camera)
                .unwrap()
                .translation
                .distance(expected)
                < 0.001
        );
        app.update();
        assert_eq!(
            app.world().resource::<SimulationClock>().0.elapsed(),
            initial + simulated
        );
        app.world_mut().resource_mut::<Capture>().session = None;
        app.world_mut()
            .resource_mut::<Time<Real>>()
            .advance_by(Duration::from_millis(20));
        app.update();
        assert_eq!(
            app.world().resource::<SimulationClock>().0.elapsed(),
            initial + simulated + Duration::from_millis(20)
        );
        std::fs::remove_dir(dir).unwrap();
    }
}
