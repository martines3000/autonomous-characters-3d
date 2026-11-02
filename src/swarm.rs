//! Optional swarm forces share the same arithmetic with gpu_flock.wgsl.
use bevy::{
    camera_controller::pan_orbit_camera::prelude::*,
    ecs::system::SystemParam,
    prelude::*,
    render::{extract_resource::ExtractResource, RenderApp},
    window::PrimaryWindow,
};
use bevy_egui::{egui, EguiContexts};
use nalgebra::Vector3;

use crate::{vehicle, GlobalState};

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum AttractorMode {
    #[default]
    Off,
    Attract,
    Repel,
}

impl AttractorMode {
    pub fn name(self) -> &'static str {
        match self {
            Self::Off => "Off",
            Self::Attract => "Attract",
            Self::Repel => "Repel",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Attractor {
    pub mode: AttractorMode,
    pub strength: f32,
    pub radius: f32,
    pub cursor_point: Option<Vec3>,
    // Scenario override stays active without held input; `attractor mouse` clears it.
    pub scripted_point: Option<Vec3>,
}

impl Default for Attractor {
    fn default() -> Self {
        Self {
            mode: AttractorMode::Off,
            strength: 4.0,
            radius: 60.0,
            cursor_point: None,
            scripted_point: None,
        }
    }
}

impl Attractor {
    pub fn point(&self) -> Option<Vec3> {
        if self.mode == AttractorMode::Off {
            None
        } else {
            self.scripted_point.or(self.cursor_point)
        }
    }

    pub fn world_radius(&self, count: usize) -> f32 {
        self.radius * vehicle::world_scale(count)
    }

    pub fn signed_strength(&self) -> f32 {
        match self.mode {
            AttractorMode::Off => 0.0,
            AttractorMode::Attract => self.strength,
            AttractorMode::Repel => -self.strength,
        }
    }

    pub fn force(&self, position: Vector3<f32>, count: usize, max_speed: f32) -> Vector3<f32> {
        let Some(point) = self.point() else {
            return Vector3::zeros();
        };
        let offset = (Vector3::from(point) - position) / self.world_radius(count);
        let q = offset.magnitude();
        if q >= 1.0 {
            return Vector3::zeros();
        }
        // 6.75*q*(1-q)^2 peaks at one at q=1/3, and vanishes smoothly at the radius.
        offset * (max_speed * self.signed_strength() * 6.75 * (1.0 - q) * (1.0 - q))
    }
}

pub const MAX_GROUPS: usize = 8;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Group {
    pub target: Option<Vec3>,
    pub color: [u8; 3],
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Groups {
    // Zero disables group steering/render metadata while preserving stored membership.
    pub count: u32,
    pub table: [Group; MAX_GROUPS],
}

impl Default for Groups {
    fn default() -> Self {
        let colors = [
            [35, 225, 255],
            [255, 125, 45],
            [185, 95, 255],
            [100, 245, 120],
            [255, 75, 160],
            [255, 230, 65],
            [105, 140, 255],
            [245, 245, 235],
        ];
        Self {
            count: 0,
            table: colors.map(|color| Group {
                target: None,
                color,
            }),
        }
    }
}

impl Groups {
    pub fn spawn_group(&self, id: u32) -> u32 {
        id % if self.count == 0 {
            MAX_GROUPS as u32
        } else {
            self.count
        }
    }

    pub fn mapped_group(&self, group: u32) -> u32 {
        if self.count == 0 {
            group
        } else {
            group % self.count
        }
    }

    pub fn render_group(&self, group: u32) -> u8 {
        if self.count == 0 {
            0
        } else {
            self.mapped_group(group) as u8
        }
    }

    pub fn target(&self, group: u32) -> Option<Vec3> {
        if self.count == 0 {
            None
        } else {
            self.table[self.mapped_group(group) as usize].target
        }
    }

    pub fn has_targets(&self) -> bool {
        self.table[..self.count as usize]
            .iter()
            .any(|group| group.target.is_some())
    }
}

pub fn groups_ui(ui: &mut egui::Ui, groups: &mut Groups) {
    egui::ComboBox::from_label("Groups")
        .selected_text(if groups.count == 0 {
            "Off".into()
        } else {
            groups.count.to_string()
        })
        .show_ui(ui, |ui| {
            ui.selectable_value(&mut groups.count, 0, "Off");
            for count in 2..=MAX_GROUPS as u32 {
                ui.selectable_value(&mut groups.count, count, count.to_string());
            }
        });
    if groups.count == 0 {
        return;
    }
    ui.small("All groups separate; alignment/cohesion stay within each group.");
    ui.small("Group-count edits reassign existing members from their spawn IDs.");
    for (index, group) in groups.table[..groups.count as usize].iter_mut().enumerate() {
        ui.horizontal(|ui| {
            ui.label(format!("Group {index}"));
            ui.color_edit_button_srgb(&mut group.color);
            let mut enabled = group.target.is_some();
            if ui.checkbox(&mut enabled, "Seek target").changed() {
                group.target = enabled.then_some(Vec3::ZERO);
            }
        });
        if let Some(target) = &mut group.target {
            ui.horizontal(|ui| {
                for (axis, value) in ["x", "y", "z"].into_iter().zip(target.as_mut()) {
                    ui.add(
                        egui::DragValue::new(value)
                            .speed(1.0)
                            .range(-1e7..=1e7)
                            .prefix(format!("{axis}: ")),
                    );
                }
            });
        }
    }
    ui.small("Selected global target overrides group targets. Use Vehicles → Color → Group.");
}

pub const MAX_OBSTACLES: usize = 8;
pub const OBSTACLE_LOOK_AHEAD: f32 = 1.0;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Obstacle {
    pub center: Vec3,
    // Reference units; centers remain world coordinates when count changes.
    pub radius: f32,
    pub clearance: f32,
}

#[derive(Resource, ExtractResource, Clone, Debug, PartialEq)]
#[extract_app(RenderApp)]
pub struct SwarmScene {
    pub obstacles: Vec<Obstacle>,
    pub obstacle_strength: f32,
    pub obstacle_clearance: f32,
    pub show_obstacles: bool,
    pub formation: crate::formation::Formation,
    pub predators: crate::predator::Predators,
}

impl Default for SwarmScene {
    fn default() -> Self {
        Self {
            obstacles: Vec::new(),
            obstacle_strength: 4.0,
            obstacle_clearance: 10.0,
            show_obstacles: true,
            formation: default(),
            predators: default(),
        }
    }
}

impl SwarmScene {
    pub fn add_obstacle(&mut self, center: Vec3, radius: f32) {
        if self.obstacles.len() < MAX_OBSTACLES {
            self.obstacles.push(Obstacle {
                center,
                radius,
                clearance: self.obstacle_clearance,
            });
        }
    }

    pub fn set_clearance(&mut self, clearance: f32) {
        self.obstacle_clearance = clearance;
        for obstacle in &mut self.obstacles {
            obstacle.clearance = clearance;
        }
    }

    pub fn obstacle_force(
        &self,
        position: Vector3<f32>,
        velocity: Vector3<f32>,
        count: usize,
        max_speed: f32,
    ) -> Vector3<f32> {
        let scale = vehicle::world_scale(count);
        let segment = velocity * OBSTACLE_LOOK_AHEAD;
        let length_squared = segment.magnitude_squared();
        let mut sum = Vector3::zeros();
        for obstacle in &self.obstacles {
            let center = Vector3::from(obstacle.center);
            let t = if length_squared > 0.0 {
                ((center - position).dot(&segment) / length_squared).clamp(0.0, 1.0)
            } else {
                0.0
            };
            let offset = position + segment * t - center;
            let distance = offset.magnitude();
            let normal = if distance > 0.0 {
                offset / distance
            } else {
                Vector3::x()
            };
            let surface = distance - obstacle.radius * scale;
            let weight = 1.0 - (surface / (obstacle.clearance * scale)).clamp(0.0, 1.0);
            sum += normal * (weight * weight * (3.0 - 2.0 * weight));
        }
        let magnitude = sum.magnitude();
        if magnitude > 1.0 {
            sum /= magnitude;
        }
        sum * (max_speed * self.obstacle_strength)
    }

    pub(crate) fn obstacle_records(&self) -> Vec<Vec4> {
        self.obstacles
            .iter()
            .flat_map(|sphere| {
                [
                    sphere.center.extend(sphere.radius),
                    Vec4::new(sphere.clearance, 0.0, 0.0, 0.0),
                ]
            })
            .collect()
    }
}

pub fn obstacles_ui(ui: &mut egui::Ui, scene: &mut SwarmScene) {
    ui.label("Sphere obstacles (soft avoidance)");
    ui.horizontal(|ui| {
        if ui
            .add_enabled(
                scene.obstacles.len() < MAX_OBSTACLES,
                egui::Button::new("Add at origin"),
            )
            .clicked()
        {
            scene.add_obstacle(Vec3::ZERO, 12.0);
        }
        if ui.button("Clear / Off").clicked() {
            scene.obstacles.clear();
        }
        ui.checkbox(&mut scene.show_obstacles, "Show");
    });
    ui.add(egui::Slider::new(&mut scene.obstacle_strength, 0.0..=20.0).text("obstacle strength"));
    let mut clearance = scene.obstacle_clearance;
    if ui
        .add(egui::Slider::new(&mut clearance, 0.1..=200.0).text("all clearances × world scale"))
        .changed()
    {
        scene.set_clearance(clearance);
    }
    let mut remove = None;
    for (index, obstacle) in scene.obstacles.iter_mut().enumerate() {
        ui.horizontal(|ui| {
            ui.label(format!("Sphere {index}"));
            if ui.button("Remove").clicked() {
                remove = Some(index);
            }
        });
        ui.horizontal(|ui| {
            for (axis, value) in ["x", "y", "z"].into_iter().zip(obstacle.center.as_mut()) {
                ui.add(
                    egui::DragValue::new(value)
                        .speed(1.0)
                        .range(-1e7..=1e7)
                        .prefix(format!("{axis}: ")),
                );
            }
        });
        ui.add(egui::Slider::new(&mut obstacle.radius, 0.1..=200.0).text("radius × world scale"));
        ui.add(
            egui::Slider::new(&mut obstacle.clearance, 0.1..=200.0).text("clearance × world scale"),
        );
    }
    if let Some(index) = remove {
        scene.obstacles.remove(index);
    }
    ui.small("Empty list is Off. One simulated second of look-ahead; penetration is possible.");
}

#[derive(SystemParam)]
pub(crate) struct SwarmUi<'w, 's> {
    pub commands: Commands<'w, 's>,
    pub scene: ResMut<'w, SwarmScene>,
    pub formation: Res<'w, crate::formation::FormationFrame>,
    cameras: Query<'w, 's, (&'static Transform, &'static PanOrbitCamera), With<Camera3d>>,
}

impl SwarmUi<'_, '_> {
    pub fn orbit_focus(&self) -> Option<Vec3> {
        self.cameras.single().ok().map(|(transform, camera)| {
            transform.translation + transform.forward() * camera.last_anchor_depth().abs() as f32
        })
    }
}

pub struct SwarmPlugin;
impl Plugin for SwarmPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<crate::predator::PredatorFrame>()
            .init_resource::<crate::formation::FormationFrame>()
            .init_resource::<SwarmScene>()
            .init_gizmo_group::<SwarmMarker>();
        let mut store = app.world_mut().resource_mut::<GizmoConfigStore>();
        let (config, _) = store.config_mut::<SwarmMarker>();
        config.depth_bias = -1.0;
        config.line.width = 2.0;
        app.add_systems(
            Update,
            crate::formation::prepare_frame
                .after(crate::presentation::prepare_simulation)
                .before(vehicle::step_cpu)
                .before(crate::gpu_flock::update_gpu_params),
        )
        .add_systems(
            Update,
            crate::predator::prepare_frame
                .after(crate::formation::prepare_frame)
                .before(vehicle::step_cpu)
                .before(crate::gpu_flock::update_gpu_params),
        )
        .add_systems(
            Update,
            cursor_input
                .after(crate::target::input_controls)
                .before(vehicle::step_cpu)
                .before(crate::gpu_flock::update_gpu_params),
        )
        .add_systems(
            PostUpdate,
            (draw_attractor, draw_obstacles, crate::predator::draw)
                .after(crate::scene::frame_world),
        );
    }
}

#[derive(Default, Reflect, GizmoConfigGroup)]
pub(crate) struct SwarmMarker;

// Plane through the current orbit focus, perpendicular to the camera's forward direction.
fn cursor_on_plane(ray: Ray3d, camera: &Transform, depth: f32) -> Option<Vec3> {
    let normal = *camera.forward();
    let focus = camera.translation + normal * depth;
    let denominator = ray.direction.dot(normal);
    if denominator.abs() <= f32::EPSILON {
        return None;
    }
    let distance = (focus - ray.origin).dot(normal) / denominator;
    (distance >= 0.0 && distance.is_finite()).then(|| ray.get_point(distance))
}

fn cursor_input(
    keys: Res<ButtonInput<KeyCode>>,
    windows: Query<&Window, With<PrimaryWindow>>,
    cameras: Query<(&Camera, &Transform, &PanOrbitCamera), With<Camera3d>>,
    mut contexts: EguiContexts,
    mut state: ResMut<GlobalState>,
) {
    let point = cursor_point(
        &keys,
        &windows,
        &cameras,
        &mut contexts,
        state.attractor.mode,
    );
    // Write only on change: GlobalState change detection drives the appearance uniform.
    if state.attractor.cursor_point != point {
        state.attractor.cursor_point = point;
    }
}

fn cursor_point(
    keys: &ButtonInput<KeyCode>,
    windows: &Query<&Window, With<PrimaryWindow>>,
    cameras: &Query<(&Camera, &Transform, &PanOrbitCamera), With<Camera3d>>,
    contexts: &mut EguiContexts,
    mode: AttractorMode,
) -> Option<Vec3> {
    let blocked = contexts.ctx_mut().is_ok_and(|ctx| {
        ctx.egui_wants_keyboard_input()
            || ctx.egui_wants_pointer_input()
            || ctx.is_pointer_over_egui()
    });
    if blocked || !keys.pressed(KeyCode::KeyF) || mode == AttractorMode::Off {
        return None;
    }
    let (Ok(window), Ok((camera, transform, orbit))) = (windows.single(), cameras.single()) else {
        return None;
    };
    if !window.focused {
        return None;
    }
    let cursor = window.cursor_position()?;
    let ray = camera
        .viewport_to_world(&GlobalTransform::from(*transform), cursor)
        .ok()?;
    cursor_on_plane(ray, transform, orbit.last_anchor_depth().abs() as f32)
}

fn draw_attractor(
    state: Res<GlobalState>,
    cameras: Query<&Transform, With<Camera3d>>,
    mut gizmos: Gizmos<SwarmMarker>,
) {
    let (Some(point), Ok(camera)) = (state.attractor.point(), cameras.single()) else {
        return;
    };
    let radius = state.attractor.world_radius(state.vehicle_count);
    let color = match state.attractor.mode {
        AttractorMode::Attract => Color::srgb(0.2, 1.0, 0.9),
        AttractorMode::Repel => Color::srgb(1.0, 0.45, 0.12),
        AttractorMode::Off => return,
    };
    gizmos
        .circle(Isometry3d::new(point, camera.rotation), radius, color)
        .resolution(96);
    gizmos.sphere(Isometry3d::from_translation(point), radius * 0.025, color);
    for direction in [Vec3::X, Vec3::Y] {
        let axis = camera.rotation * direction * radius * 0.06;
        gizmos.line(point - axis, point + axis, color);
    }
}

fn draw_obstacles(
    state: Res<GlobalState>,
    scene: Res<SwarmScene>,
    mut gizmos: Gizmos<SwarmMarker>,
) {
    if !scene.show_obstacles {
        return;
    }
    let scale = vehicle::world_scale(state.vehicle_count);
    for sphere in &scene.obstacles {
        gizmos.sphere(
            Isometry3d::from_translation(sphere.center),
            sphere.radius * scale,
            Color::srgb(1.0, 0.55, 0.18),
        );
        gizmos.sphere(
            Isometry3d::from_translation(sphere.center),
            (sphere.radius + sphere.clearance) * scale,
            Color::srgba(1.0, 0.7, 0.3, 0.18),
        );
    }
}

pub fn ui(ui: &mut egui::Ui, attractor: &mut Attractor) {
    ui.horizontal(|ui| {
        ui.label("Cursor field");
        for mode in [
            AttractorMode::Off,
            AttractorMode::Attract,
            AttractorMode::Repel,
        ] {
            ui.selectable_value(&mut attractor.mode, mode, mode.name());
        }
    });
    ui.add(egui::Slider::new(&mut attractor.strength, 0.0..=20.0).text("strength"));
    ui.add(egui::Slider::new(&mut attractor.radius, 1.0..=200.0).text("radius × world scale"));
    ui.small("Hold F over the scene. Plane faces camera through orbit focus.");
    if attractor.scripted_point.is_some() {
        ui.label("Scenario point active");
        if ui.button("Use mouse (hold F)").clicked() {
            attractor.scripted_point = None;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn obstacle_falloff_edges_inside_and_exact_center() {
        let mut scene = SwarmScene {
            obstacle_strength: 2.0,
            ..default()
        };
        scene.add_obstacle(Vec3::ZERO, 10.0);
        scene.set_clearance(4.0);
        let force = |x| scene.obstacle_force(Vector3::x() * x, Vector3::zeros(), 200, 80.0);
        for x in [14.0, 14.001, 100.0] {
            assert_eq!(force(x), Vector3::zeros());
        }
        assert_eq!(force(12.0), Vector3::x() * 80.0);
        for x in [0.0, 1.0, 9.9, 10.0] {
            assert_eq!(force(x), Vector3::x() * 160.0);
        }
        assert_eq!(force(-5.0), -Vector3::x() * 160.0);
        assert!(force(13.999).magnitude() < force(13.99).magnitude() * 0.02);
    }

    #[test]
    fn obstacle_segment_projection_endpoints_and_summed_clamp() {
        let mut scene = SwarmScene {
            obstacle_strength: 2.0,
            ..default()
        };
        scene.add_obstacle(Vec3::ZERO, 10.0);
        let position = Vector3::new(-20.0, 3.0, 0.0);
        let velocity = Vector3::x() * 40.0;
        assert_eq!(
            scene.obstacle_force(position, velocity, 200, 80.0),
            Vector3::y() * 160.0
        );
        assert_eq!(
            scene.obstacle_force(Vector3::x() * -40.0, Vector3::x() * 2.0, 200, 80.0),
            Vector3::zeros()
        );
        scene.obstacles = vec![scene.obstacles[0]; MAX_OBSTACLES];
        assert_eq!(
            scene.obstacle_force(Vector3::x(), Vector3::zeros(), 200, 80.0),
            Vector3::x() * 160.0
        );
        // A closest point exactly at center uses +X, even when the segment crosses the center.
        assert_eq!(
            scene.obstacle_force(-Vector3::x() * 10.0, Vector3::x() * 20.0, 200, 80.0),
            Vector3::x() * 160.0
        );
    }

    #[test]
    fn obstacle_shape_scales_and_empty_or_zero_strength_is_zero() {
        let mut scene = SwarmScene::default();
        let p = Vector3::x() * 15.0;
        assert_eq!(
            scene.obstacle_force(p, Vector3::zeros(), 200, 80.0),
            Vector3::zeros()
        );
        scene.add_obstacle(Vec3::ZERO, 10.0);
        let reference = scene.obstacle_force(p, Vector3::zeros(), 200, 80.0);
        for count in [200, 20_000, 1_000_000] {
            let force = scene.obstacle_force(
                p * vehicle::world_scale(count),
                Vector3::zeros(),
                count,
                80.0,
            );
            assert!((force - reference).magnitude() < 1e-4);
        }
        scene.obstacle_strength = 0.0;
        assert_eq!(
            scene.obstacle_force(p, Vector3::zeros(), 200, 80.0),
            Vector3::zeros()
        );
    }

    #[test]
    fn held_f_requires_scene_cursor_and_defers_to_egui_hover_and_focus() {
        use bevy::camera::{CameraProjection, ComputedCameraValues, RenderTargetInfo};
        let mut app = App::new();
        app.init_resource::<GlobalState>()
            .init_resource::<ButtonInput<KeyCode>>()
            .init_resource::<bevy_egui::EguiUserTextures>()
            .add_systems(Update, cursor_input);
        app.world_mut().resource_mut::<GlobalState>().attractor.mode = AttractorMode::Attract;
        let mut window = Window {
            resolution: (800, 600).into(),
            focused: true,
            ..default()
        };
        window.set_cursor_position(Some(Vec2::new(400.0, 300.0)));
        let window = app.world_mut().spawn((window, PrimaryWindow)).id();
        let projection = PerspectiveProjection {
            aspect_ratio: 4.0 / 3.0,
            ..default()
        };
        app.world_mut().spawn((
            Camera3d::default(),
            Camera {
                computed: ComputedCameraValues {
                    clip_from_view: projection.get_clip_from_view(),
                    target_info: Some(RenderTargetInfo {
                        physical_size: UVec2::new(800, 600),
                        scale_factor: 1.0,
                    }),
                    ..default()
                },
                ..default()
            },
            Transform::from_xyz(0.0, 0.0, 100.0),
            PanOrbitCamera::default().with_initial_anchor_depth(100.0),
        ));
        app.update();
        assert_eq!(
            app.world().resource::<GlobalState>().attractor.point(),
            None
        );
        app.world_mut()
            .resource_mut::<ButtonInput<KeyCode>>()
            .press(KeyCode::KeyF);
        app.update();
        assert!(
            app.world()
                .resource::<GlobalState>()
                .attractor
                .point()
                .unwrap()
                .length()
                < 1e-4
        );
        app.world_mut()
            .resource_mut::<ButtonInput<KeyCode>>()
            .release(KeyCode::KeyF);
        app.update();
        assert_eq!(
            app.world().resource::<GlobalState>().attractor.point(),
            None
        );
        app.world_mut()
            .resource_mut::<ButtonInput<KeyCode>>()
            .press(KeyCode::KeyF);
        app.world_mut()
            .get_mut::<Window>(window)
            .unwrap()
            .set_cursor_position(None);
        app.update();
        assert_eq!(
            app.world().resource::<GlobalState>().attractor.point(),
            None
        );
        app.world_mut()
            .get_mut::<Window>(window)
            .unwrap()
            .set_cursor_position(Some(Vec2::new(400.0, 300.0)));
        app.world_mut().get_mut::<Window>(window).unwrap().focused = false;
        app.update();
        assert_eq!(
            app.world().resource::<GlobalState>().attractor.point(),
            None
        );
        app.world_mut().get_mut::<Window>(window).unwrap().focused = true;
        for keyboard_focus in [false, true] {
            let mut context = bevy_egui::EguiContext::default();
            let ctx = context.get_mut();
            let mut text = String::new();
            for _ in 0..2 {
                let _ = ctx.run_ui(
                    egui::RawInput {
                        events: if keyboard_focus {
                            vec![]
                        } else {
                            vec![egui::Event::PointerMoved(egui::pos2(20.0, 20.0))]
                        },
                        ..default()
                    },
                    |ui| {
                        if keyboard_focus {
                            ui.text_edit_singleline(&mut text).request_focus();
                        } else {
                            egui::Window::new("Menu")
                                .fixed_pos(egui::pos2(0.0, 0.0))
                                .fixed_size(egui::vec2(200.0, 100.0))
                                .show(ui.ctx(), |ui| {
                                    ui.label("Menu hover");
                                });
                        }
                    },
                );
            }
            assert!(if keyboard_focus {
                ctx.egui_wants_keyboard_input()
            } else {
                ctx.is_pointer_over_egui()
            });
            let context = app
                .world_mut()
                .spawn((context, bevy_egui::PrimaryEguiContext))
                .id();
            app.update();
            assert_eq!(
                app.world().resource::<GlobalState>().attractor.point(),
                None
            );
            app.world_mut().despawn(context);
        }
        app.update();
        assert!(app
            .world()
            .resource::<GlobalState>()
            .attractor
            .point()
            .is_some());
    }

    #[test]
    fn field_is_finite_bounded_smooth_at_cutoff_and_repel_negates_attract() {
        let mut field = Attractor {
            mode: AttractorMode::Attract,
            radius: 60.0,
            scripted_point: Some(Vec3::ZERO),
            ..default()
        };
        let speed = 80.0;
        assert_eq!(field.force(Vector3::zeros(), 200, speed), Vector3::zeros());
        for x in [0.00001, 1.0, 20.0, 30.0, 59.9, 59.99, 60.0, 61.0] {
            field.mode = AttractorMode::Attract;
            let force = field.force(Vector3::x() * x, 200, speed);
            assert!(force.iter().all(|v| v.is_finite()));
            assert!(force.x <= 0.0);
            assert!(force.magnitude() <= speed * field.strength + 1e-4);
            if x >= 60.0 {
                assert_eq!(force, Vector3::zeros());
            }
            field.mode = AttractorMode::Repel;
            assert_eq!(field.force(Vector3::x() * x, 200, speed), -force);
        }
        field.mode = AttractorMode::Attract;
        assert!((field.force(Vector3::x() * 20.0, 200, speed).x + 320.0).abs() < 1e-4);
        let near = field.force(Vector3::x() * 59.99, 200, speed).magnitude();
        let farther = field.force(Vector3::x() * 59.9, 200, speed).magnitude();
        assert!(near < farther * 0.011);
        field.strength = 0.0;
        assert_eq!(field.force(Vector3::x(), 200, speed), Vector3::zeros());
        field.strength = 4.0;
        field.mode = AttractorMode::Off;
        assert_eq!(field.force(Vector3::x(), 200, speed), Vector3::zeros());
    }

    #[test]
    fn field_shape_scales_with_count_without_changing_force_bound() {
        let field = Attractor {
            mode: AttractorMode::Attract,
            scripted_point: Some(Vec3::ZERO),
            ..default()
        };
        let reference = field.force(Vector3::new(12.0, -7.0, 3.0), 200, 80.0);
        for count in [200, 20_000, 1_000_000] {
            let position = Vector3::new(12.0, -7.0, 3.0) * vehicle::world_scale(count);
            assert!((field.force(position, count, 80.0) - reference).magnitude() < 1e-4);
        }
    }

    #[test]
    fn cursor_plane_tracks_focus_at_any_zoom_and_camera_orientation() {
        for depth in [1.0, 100.0, 100_000.0] {
            let camera = Transform::from_xyz(43.0, 12.0, 66.0).with_rotation(Quat::from_euler(
                EulerRot::YXZ,
                0.7,
                0.3,
                0.0,
            ));
            let origin = camera.translation;
            let ray = Ray3d::new(origin, camera.forward());
            let focus = origin + camera.forward() * depth;
            assert!(
                cursor_on_plane(ray, &camera, depth)
                    .unwrap()
                    .distance(focus)
                    < 0.02
            );
            let direction = Dir3::new(*camera.forward() + *camera.right() * 0.2).unwrap();
            let point = cursor_on_plane(Ray3d::new(origin, direction), &camera, depth).unwrap();
            assert!((point - focus).dot(*camera.forward()).abs() < 0.02);
        }
        let camera = Transform::default();
        assert!(cursor_on_plane(Ray3d::new(Vec3::ZERO, Dir3::X), &camera, 10.0).is_none());
        assert!(cursor_on_plane(Ray3d::new(Vec3::ZERO, Dir3::Z), &camera, 10.0).is_none());
    }
}
