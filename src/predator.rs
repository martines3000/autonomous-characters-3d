//! Bounded control-object flee fields, sampled once per simulated substep for both backends.
use bevy::{
    prelude::*,
    render::{extract_resource::ExtractResource, RenderApp},
};
use bevy_egui::egui;

use crate::{
    formation::FormationFrame,
    presentation::SimulationFrame,
    swarm::{SwarmMarker, SwarmScene},
    vehicle, GlobalState,
};

pub const MAX_PREDATORS: usize = 4;
pub const PREDATOR_RECORDS: usize = 4 * MAX_PREDATORS * 2;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum PredatorPath {
    #[default]
    Off,
    Orbit,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Predator {
    pub position: Vec3,
    pub radius: f32,
    pub mask: u32,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Predators {
    pub objects: Vec<Predator>,
    pub strength: f32,
    pub visible: bool,
    pub path: PredatorPath,
    pub path_center: Vec3,
    pub path_radius: f32,
    pub path_speed: f32,
    pub default_mask: u32,
}

impl Default for Predators {
    fn default() -> Self {
        Self {
            objects: Vec::new(),
            strength: 4.0,
            visible: true,
            path: PredatorPath::Off,
            path_center: Vec3::ZERO,
            path_radius: 50.0,
            path_speed: 30.0,
            default_mask: 255,
        }
    }
}

impl Predators {
    pub fn add(&mut self, position: Vec3, radius: f32) {
        if self.objects.len() < MAX_PREDATORS {
            self.objects.push(Predator {
                position,
                radius,
                mask: self.default_mask,
            });
        }
    }

    pub fn set_mask(&mut self, mask: u32) {
        self.default_mask = mask;
        for predator in &mut self.objects {
            predator.mask = mask;
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub(crate) struct PredatorSample {
    pub position: Vec3,
    pub radius: f32,
    pub mask: u32,
}

#[derive(Resource, ExtractResource, Clone, Copy, Debug, Default, PartialEq)]
#[extract_app(RenderApp)]
pub(crate) struct PredatorFrame {
    pub samples: [[PredatorSample; MAX_PREDATORS]; 4],
    pub count: usize,
    pub strength: f32,
    pub visible: bool,
    pub phase: f64,
    pub step: usize,
}

impl PredatorFrame {
    pub fn advance(
        &self,
        settings: &Predators,
        count: usize,
        center: Vec3,
        simulation: &SimulationFrame,
    ) -> Self {
        let scale = vehicle::world_scale(count);
        let mut phase = if settings.objects.is_empty() {
            0.0
        } else {
            self.phase
        };
        let mut samples = [[PredatorSample::default(); MAX_PREDATORS]; 4];
        for (step, sample) in samples.iter_mut().enumerate() {
            if settings.path == PredatorPath::Orbit
                && !settings.objects.is_empty()
                && step < simulation.steps as usize
            {
                phase = (phase
                    + (settings.path_speed as f64).to_radians() * simulation.step.as_secs_f64())
                .rem_euclid(std::f64::consts::TAU);
            }
            for (index, predator) in settings.objects.iter().enumerate() {
                let position = if settings.path == PredatorPath::Orbit {
                    let angle = (phase
                        + index as f64 * std::f64::consts::TAU / settings.objects.len() as f64)
                        .rem_euclid(std::f64::consts::TAU) as f32;
                    center
                        + Vec3::new(angle.cos(), angle.sin(), 0.0) * (settings.path_radius * scale)
                } else {
                    predator.position
                };
                sample[index] = PredatorSample {
                    position,
                    radius: predator.radius * scale,
                    mask: predator.mask,
                };
            }
        }
        Self {
            samples,
            count: settings.objects.len(),
            strength: settings.strength,
            visible: settings.visible,
            phase,
            step: simulation.steps.saturating_sub(1) as usize,
        }
    }

    pub fn active(&self) -> bool {
        self.count > 0 && self.strength != 0.0
    }

    pub fn force(&self, position: Vec3, group: u32, speed: f32) -> Vec3 {
        let mut sum = Vec3::ZERO;
        for predator in &self.samples[self.step][..self.count] {
            if predator.mask & (1 << group) == 0 {
                continue;
            }
            let offset = position - predator.position;
            let distance = offset.length();
            let normal = if distance > 0.0 {
                offset / distance
            } else {
                Vec3::X
            };
            let weight = 1.0 - (distance / predator.radius).clamp(0.0, 1.0);
            sum += normal * (weight * weight * (3.0 - 2.0 * weight));
        }
        if sum.length_squared() > 1.0 {
            sum = sum.normalize();
        }
        sum * (speed * self.strength)
    }

    pub fn records(&self) -> Vec<UVec4> {
        if !self.active() {
            return Vec::new();
        }
        self.samples
            .iter()
            .flat_map(|sample| {
                sample.iter().flat_map(|predator| {
                    [
                        UVec4::from_array(
                            predator
                                .position
                                .extend(predator.radius)
                                .to_array()
                                .map(f32::to_bits),
                        ),
                        UVec4::new(predator.mask, 0, 0, 0),
                    ]
                })
            })
            .collect()
    }
}

pub(crate) fn prepare_frame(
    scene: Res<SwarmScene>,
    state: Res<GlobalState>,
    formation: Res<FormationFrame>,
    simulation: Res<SimulationFrame>,
    mut frame: ResMut<PredatorFrame>,
) {
    let center = if formation.settings.enabled() {
        formation.settings.center
    } else {
        scene.predators.path_center
    };
    let next = frame.advance(&scene.predators, state.vehicle_count, center, &simulation);
    frame.set_if_neq(next);
}

pub(crate) fn draw(
    frame: Res<PredatorFrame>,
    cameras: Query<&Transform, With<Camera3d>>,
    mut gizmos: Gizmos<SwarmMarker>,
) {
    if !frame.visible {
        return;
    }
    let Ok(camera) = cameras.single() else {
        return;
    };
    let color = Color::srgb(1.0, 0.12, 0.48);
    for predator in &frame.samples[frame.step][..frame.count] {
        gizmos
            .circle(
                Isometry3d::new(predator.position, camera.rotation),
                predator.radius,
                Color::srgba(1.0, 0.12, 0.48, 0.6),
            )
            .resolution(64);
        gizmos.sphere(
            Isometry3d::from_translation(predator.position),
            predator.radius * 0.08,
            color,
        );
        for axis in [Vec3::X, Vec3::Y, Vec3::Z] {
            gizmos.line(
                predator.position - axis * predator.radius * 0.15,
                predator.position + axis * predator.radius * 0.15,
                color,
            );
        }
    }
}

pub fn ui(ui: &mut egui::Ui, settings: &mut Predators, groups: u32, focus: Option<Vec3>) {
    ui.label("Predators (soft flee fields)");
    ui.horizontal(|ui| {
        if ui
            .add_enabled(
                settings.objects.len() < MAX_PREDATORS,
                egui::Button::new("Add predator"),
            )
            .clicked()
        {
            settings.add(focus.unwrap_or(Vec3::ZERO), 20.0);
        }
        if ui.button("Clear / Off").clicked() {
            settings.objects.clear();
        }
        ui.checkbox(&mut settings.visible, "Show");
    });
    ui.add(egui::Slider::new(&mut settings.strength, 0.0..=20.0).text("flee strength"));
    let previous = settings.path;
    ui.horizontal(|ui| {
        ui.label("Path");
        ui.selectable_value(&mut settings.path, PredatorPath::Off, "Static");
        ui.selectable_value(&mut settings.path, PredatorPath::Orbit, "Orbit");
    });
    if settings.path == PredatorPath::Orbit {
        if previous != settings.path {
            settings.path_center = focus.unwrap_or(Vec3::ZERO);
        }
        if ui
            .add_enabled(focus.is_some(), egui::Button::new("Anchor at orbit focus"))
            .clicked()
        {
            settings.path_center = focus.unwrap();
        }
        ui.add(
            egui::Slider::new(&mut settings.path_radius, 1.0..=200.0)
                .text("path radius × world scale"),
        );
        ui.add(
            egui::Slider::new(&mut settings.path_speed, -180.0..=180.0)
                .text("degrees / simulated s"),
        );
        ui.small("Formation center overrides the anchored orbit center. Orbit overrides static positions.");
    }
    let mut remove = None;
    for (index, predator) in settings.objects.iter_mut().enumerate() {
        ui.horizontal(|ui| {
            ui.label(format!("Predator {index}"));
            if ui.button("Remove").clicked() {
                remove = Some(index);
            }
        });
        ui.horizontal(|ui| {
            for (axis, value) in ["x", "y", "z"].into_iter().zip(predator.position.as_mut()) {
                ui.add(
                    egui::DragValue::new(value)
                        .speed(1.0)
                        .range(-1e7..=1e7)
                        .prefix(format!("{axis}: ")),
                );
            }
        });
        ui.add(
            egui::Slider::new(&mut predator.radius, 1.0..=200.0)
                .text("threat radius × world scale"),
        );
        if groups > 0 {
            ui.horizontal_wrapped(|ui| {
                ui.label("Prey groups:");
                for group in 0..groups {
                    let mut enabled = predator.mask & (1 << group) != 0;
                    if ui.checkbox(&mut enabled, group.to_string()).changed() {
                        if enabled {
                            predator.mask |= 1 << group;
                        } else {
                            predator.mask &= !(1 << group);
                        }
                    }
                }
            });
        }
    }
    if let Some(index) = remove {
        settings.objects.remove(index);
    }
    ui.small("Every predator is tested, including with neighbor caps. Groups Off uses mask bit 0.");
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    #[test]
    fn flee_falloff_coincidence_sum_cap_and_masks() {
        let mut settings = Predators {
            strength: 2.0,
            ..default()
        };
        settings.add(Vec3::ZERO, 10.0);
        settings.set_mask(1);
        let mut frame = PredatorFrame::default().advance(
            &settings,
            200,
            Vec3::ZERO,
            &SimulationFrame::default(),
        );
        assert_eq!(frame.force(Vec3::ZERO, 0, 80.0), Vec3::X * 160.0);
        assert_eq!(frame.force(Vec3::X * 5.0, 0, 80.0), Vec3::X * 80.0);
        for x in [10.0, 10.1, 100.0] {
            assert_eq!(frame.force(Vec3::X * x, 0, 80.0), Vec3::ZERO);
        }
        assert_eq!(frame.force(Vec3::X, 1, 80.0), Vec3::ZERO);
        assert_eq!(frame.force(Vec3::NEG_X * 5.0, 0, 80.0), Vec3::NEG_X * 80.0);
        assert!(
            frame.force(Vec3::X * 9.999, 0, 80.0).length()
                < frame.force(Vec3::X * 9.99, 0, 80.0).length() * 0.02
        );
        settings.objects = vec![settings.objects[0]; MAX_PREDATORS];
        frame = frame.advance(&settings, 200, Vec3::ZERO, &SimulationFrame::default());
        assert_eq!(frame.force(Vec3::X * 5.0, 0, 80.0), Vec3::X * 160.0);
        settings.set_mask(1 << 7);
        frame = frame.advance(&settings, 200, Vec3::ZERO, &SimulationFrame::default());
        assert_eq!(frame.force(Vec3::ZERO, 7, 80.0), Vec3::X * 160.0);
        assert_eq!(frame.force(Vec3::ZERO, 0, 80.0), Vec3::ZERO);
        settings.strength = 0.0;
        frame = frame.advance(&settings, 200, Vec3::ZERO, &SimulationFrame::default());
        assert!(!frame.active());
        assert!(frame.records().is_empty());
    }

    #[test]
    fn orbit_is_deterministic_scaled_and_matches_substep_chunking() {
        let mut settings = Predators {
            path: PredatorPath::Orbit,
            path_speed: 90.0,
            ..default()
        };
        for _ in 0..4 {
            settings.add(Vec3::new(1.0, 2.0, 3.0), 20.0);
        }
        let center = Vec3::new(7.0, -2.0, 3.0);
        let simulation = SimulationFrame {
            steps: 4,
            step: Duration::from_millis(250),
        };
        let frame = PredatorFrame::default().advance(&settings, 200, center, &simulation);
        assert_eq!(
            frame,
            PredatorFrame::default().advance(&settings, 200, center, &simulation)
        );
        assert!((frame.samples[3][0].position - (center + Vec3::Y * 50.0)).length() < 1e-5);
        let mut chunked = PredatorFrame::default();
        for _ in 0..4 {
            chunked = chunked.advance(
                &settings,
                200,
                center,
                &SimulationFrame {
                    steps: 1,
                    ..simulation
                },
            );
        }
        assert_eq!(chunked.samples[0], frame.samples[3]);
        assert_eq!(chunked.phase, frame.phase);
        let scaled = PredatorFrame::default().advance(&settings, 1_000_000, center, &simulation);
        let scale = vehicle::world_scale(1_000_000);
        assert!(
            (scaled.samples[3][0].position
                - center
                - (frame.samples[3][0].position - center) * scale)
                .length()
                < 1e-3
        );
        assert_eq!(scaled.samples[3][0].radius, 20.0 * scale);
        assert_eq!(frame.records().len(), PREDATOR_RECORDS);
        let paused = frame.advance(&settings, 200, center, &SimulationFrame::default());
        assert_eq!(paused.phase, frame.phase);
        assert!(paused
            .samples
            .iter()
            .all(|sample| sample == &frame.samples[3]));
    }

    #[test]
    fn paths_freeze_on_pause_follow_speed_and_never_write_scene() {
        for speed in [0.5, 1.0, 2.0, 4.0] {
            let mut app = crate::presentation::tests::clock_app();
            app.insert_resource(GlobalState {
                vehicle_count: 200,
                ..default()
            })
            .init_resource::<SwarmScene>()
            .init_resource::<FormationFrame>()
            .init_resource::<PredatorFrame>()
            .add_systems(
                Update,
                prepare_frame.after(crate::presentation::prepare_simulation),
            );
            {
                let mut scene = app.world_mut().resource_mut::<SwarmScene>();
                scene.predators.add(Vec3::ZERO, 20.0);
                scene.predators.path = PredatorPath::Orbit;
            }
            app.update();
            app.world_mut()
                .resource_mut::<crate::presentation::Presentation>()
                .paused = true;
            app.world_mut()
                .resource_mut::<Time<Real>>()
                .advance_by(Duration::from_millis(100));
            app.update();
            assert_eq!(app.world().resource::<PredatorFrame>().phase, 0.0);
            app.world_mut()
                .resource_mut::<crate::presentation::Presentation>()
                .paused = false;
            app.world_mut()
                .resource_mut::<crate::presentation::Presentation>()
                .speed = speed;
            app.world_mut()
                .resource_mut::<Time<Real>>()
                .advance_by(Duration::from_millis(20));
            app.update();
            let frame = *app.world().resource::<PredatorFrame>();
            assert!((frame.phase - (30_f64.to_radians() * 0.02 * speed as f64)).abs() < 1e-7);
            assert!(!app
                .world()
                .get_resource_ref::<SwarmScene>()
                .unwrap()
                .is_changed());
            app.world_mut()
                .resource_mut::<crate::presentation::Presentation>()
                .paused = true;
            for _ in 0..3 {
                app.world_mut()
                    .resource_mut::<Time<Real>>()
                    .advance_by(Duration::from_millis(100));
                app.update();
                assert_eq!(app.world().resource::<PredatorFrame>().phase, frame.phase);
            }
            assert!(!app
                .world()
                .get_resource_ref::<PredatorFrame>()
                .unwrap()
                .is_changed());
        }
    }
}
