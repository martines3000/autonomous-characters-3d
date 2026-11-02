//! Analytic formation targets and a shared simulated-substep morph snapshot.
use bevy::{prelude::*, render::extract_resource::ExtractResource, render::RenderApp};
use bevy_egui::egui;

use crate::{presentation::SimulationFrame, swarm::SwarmScene, vehicle};

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum FormationMode {
    #[default]
    Off,
    Sphere,
    Helix,
}

impl FormationMode {
    pub fn name(self) -> &'static str {
        match self {
            Self::Off => "Off",
            Self::Sphere => "Sphere",
            Self::Helix => "Helix",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Formation {
    pub mode: FormationMode,
    pub center: Vec3,
    pub extent: f32,
    pub arrival_radius: f32,
    pub strength: f32,
    // Requested blend, 0 sphere / 1 helix. Actual blend advances in FormationFrame.
    pub morph: f32,
    pub morph_speed: f32,
}

impl Default for Formation {
    fn default() -> Self {
        Self {
            mode: FormationMode::Off,
            center: Vec3::ZERO,
            extent: 60.0,
            arrival_radius: 10.0,
            strength: 4.0,
            morph: 0.0,
            morph_speed: 0.25,
        }
    }
}

impl Formation {
    pub fn enabled(&self) -> bool {
        self.mode != FormationMode::Off
    }

    pub fn set_mode(&mut self, mode: FormationMode) {
        self.mode = mode;
        match mode {
            FormationMode::Sphere => self.morph = 0.0,
            FormationMode::Helix => self.morph = 1.0,
            FormationMode::Off => (),
        }
    }
}

#[derive(Resource, ExtractResource, Clone, Copy, Debug, Default, PartialEq)]
#[extract_app(RenderApp)]
pub(crate) struct FormationFrame {
    pub settings: Formation,
    pub morph: f32,
    pub samples: [f32; 4],
    // CPU-maintained (global start, member count); GPU computes the same ranges in its arena.
    pub ranges: [UVec2; 8],
}

pub(crate) fn prepare_frame(
    scene: Res<SwarmScene>,
    simulation: Res<SimulationFrame>,
    mut frame: ResMut<FormationFrame>,
) {
    let settings = scene.formation;
    let mut morph = if settings.enabled() && !frame.settings.enabled() {
        settings.morph
    } else {
        frame.morph
    };
    let mut samples = [morph; 4];
    for (index, sample) in samples.iter_mut().enumerate() {
        if settings.enabled() && index < simulation.steps as usize {
            let delta = settings.morph - morph;
            let amount = if settings.morph_speed == 0.0 {
                delta.abs()
            } else {
                settings.morph_speed * simulation.step.as_secs_f32()
            };
            morph += delta.signum() * amount.min(delta.abs());
        }
        *sample = morph;
    }
    let next = FormationFrame {
        settings,
        morph,
        samples,
        ..*frame
    };
    frame.set_if_neq(next);
}

// A bounded angle derived with integer arithmetic avoids f32 phase banding at large slot IDs.
pub(crate) fn unit_target(slot: u32, count: u32, morph: f32) -> Vec3 {
    let u = (slot as f32 + 0.5) / count.max(1) as f32;
    let y = 1.0 - 2.0 * u;
    let angle = (slot.wrapping_mul(1640531527) >> 8) as f32 / 16777216.0 * std::f32::consts::TAU;
    let radius = (1.0 - y * y).max(0.0).sqrt();
    let sphere = Vec3::new(radius * angle.cos(), y, radius * angle.sin());
    let angle = 3.0 * std::f32::consts::TAU * u;
    let helix = Vec3::new(0.6 * angle.cos(), 0.8 * y, 0.6 * angle.sin());
    sphere + (helix - sphere) * morph
}

pub(crate) fn group_offset(group: u32, count: u32) -> Vec3 {
    let columns = (count.max(1) as f32).sqrt().ceil() as u32;
    let rows = count.max(1).div_ceil(columns);
    Vec3::new(
        group.rem_euclid(columns) as f32 - (columns - 1) as f32 * 0.5,
        (group / columns) as f32 - (rows - 1) as f32 * 0.5,
        0.0,
    ) * 2.25
}

pub(crate) fn arrival_force(
    offset: Vec3,
    velocity: Vec3,
    speed: f32,
    radius: f32,
    strength: f32,
) -> Vec3 {
    let distance = offset.length();
    let direction = if distance > f32::EPSILON {
        offset / distance
    } else {
        Vec3::ZERO
    };
    let force = direction * speed * (distance / radius).min(1.0) - velocity;
    let cap = speed * strength;
    if force.length_squared() > cap * cap {
        force.normalize() * cap
    } else {
        force
    }
}

pub(crate) fn ranges(groups: &[u32], count: u32) -> [UVec2; 8] {
    let mut counts = [0; 8];
    for &group in groups {
        counts[if count == 0 {
            0
        } else {
            (group % count) as usize
        }] += 1;
    }
    let mut start = 0;
    counts.map(|count| {
        let range = UVec2::new(start, count);
        start += count;
        range
    })
}

impl FormationFrame {
    pub fn target(&self, metadata: UVec4, count: usize, groups: u32) -> Vec3 {
        let group = if groups == 0 { 0 } else { metadata.y % groups };
        let range = self.ranges[group as usize];
        self.settings.center
            + (group_offset(group, groups) + unit_target(metadata.z - range.x, range.y, self.morph))
                * (self.settings.extent * vehicle::world_scale(count))
    }

    #[cfg(test)]
    pub fn fixed(settings: Formation) -> Self {
        Self {
            settings,
            morph: settings.morph,
            samples: [settings.morph; 4],
            ..default()
        }
    }
}

pub fn ui(ui: &mut egui::Ui, settings: &mut Formation, focus: Option<Vec3>, actual: f32) {
    let mut selected = None;
    ui.horizontal(|ui| {
        ui.label("Formation");
        for value in [
            FormationMode::Off,
            FormationMode::Sphere,
            FormationMode::Helix,
        ] {
            if ui
                .selectable_label(settings.mode == value, value.name())
                .clicked()
            {
                selected = Some(value);
            }
        }
    });
    if let Some(mode) = selected {
        settings.set_mode(mode);
    }
    if !settings.enabled() {
        return;
    }
    ui.horizontal(|ui| {
        for (axis, value) in ["x", "y", "z"].into_iter().zip(settings.center.as_mut()) {
            ui.add(
                egui::DragValue::new(value)
                    .speed(1.0)
                    .range(-1e7..=1e7)
                    .prefix(format!("{axis}: ")),
            );
        }
    });
    if ui
        .add_enabled(focus.is_some(), egui::Button::new("Use orbit focus"))
        .clicked()
    {
        settings.center = focus.unwrap();
    }
    ui.add(egui::Slider::new(&mut settings.extent, 1.0..=200.0).text("extent × world scale"));
    ui.add(
        egui::Slider::new(&mut settings.arrival_radius, 0.1..=200.0)
            .text("arrival radius × world scale"),
    );
    ui.add(egui::Slider::new(&mut settings.strength, 0.0..=20.0).text("arrival cap × max speed"));
    ui.add(egui::Slider::new(&mut settings.morph, 0.0..=1.0).text("morph target (sphere → helix)"));
    ui.add(
        egui::Slider::new(&mut settings.morph_speed, 0.0..=2.0)
            .text("morph fraction / simulated s"),
    );
    ui.small(format!(
        "Actual morph: {actual:.3}; zero speed snaps on the next simulation step."
    ));
    ui.small("Overrides seek/wander. Groups get separate shapes; follow selection stays stored.");
    ui.small("Uses follow neighbor samples; 0 is exact. Crowding can prevent exact assembly.");
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    #[test]
    fn arrival_law_slows_inside_radius_brakes_at_slot_and_caps_force() {
        assert_eq!(
            arrival_force(Vec3::X * 5.0, Vec3::ZERO, 80.0, 10.0, 4.0),
            Vec3::X * 40.0
        );
        assert_eq!(
            arrival_force(Vec3::X * 20.0, Vec3::ZERO, 80.0, 10.0, 4.0),
            Vec3::X * 80.0
        );
        assert_eq!(
            arrival_force(Vec3::ZERO, Vec3::Y * 10.0, 80.0, 10.0, 4.0),
            Vec3::NEG_Y * 10.0
        );
        assert_eq!(
            arrival_force(Vec3::X * 20.0, Vec3::NEG_X * 80.0, 80.0, 10.0, 0.25),
            Vec3::X * 20.0
        );
        assert_eq!(
            arrival_force(Vec3::X, Vec3::ZERO, 80.0, 10.0, 0.0),
            Vec3::ZERO
        );
    }

    #[test]
    fn analytic_shapes_and_blends_are_unique_deterministic_finite_and_bounded() {
        for count in [1, 2, 31, 257, 4096, 1_000_000] {
            for morph in [0.0, 0.5, 1.0] {
                let mut previous_y = f32::INFINITY;
                for slot in 0..count {
                    let point = unit_target(slot, count, morph);
                    assert!(point.is_finite() && point.length() <= 1.0 + 1e-6);
                    // Strictly decreasing y proves uniqueness, including at one million slots.
                    assert!(point.y < previous_y);
                    previous_y = point.y;
                    if slot % 997 == 0 {
                        assert_eq!(point, unit_target(slot, count, morph));
                    }
                }
            }
        }
        assert_eq!(group_offset(0, 2), Vec3::NEG_X * 1.125);
        assert_eq!(group_offset(1, 2), Vec3::X * 1.125);
        let mut frame = FormationFrame::fixed(Formation {
            mode: FormationMode::Sphere,
            ..default()
        });
        frame.ranges[0] = UVec2::new(0, 2);
        let reference = frame.target(UVec4::ZERO, 200, 0);
        assert!(
            (frame.target(UVec4::ZERO, 1_000_000, 0) - reference * vehicle::world_scale(1_000_000))
                .length()
                < 1e-3
        );
    }

    #[test]
    fn morph_uses_simulated_substeps_freezes_on_pause_and_does_not_write_scene() {
        let mut app = crate::presentation::tests::clock_app();
        app.init_resource::<SwarmScene>()
            .init_resource::<FormationFrame>()
            .add_systems(
                Update,
                prepare_frame.after(crate::presentation::prepare_simulation),
            );
        app.world_mut()
            .resource_mut::<SwarmScene>()
            .formation
            .set_mode(FormationMode::Sphere);
        app.update();
        app.world_mut()
            .resource_mut::<SwarmScene>()
            .formation
            .set_mode(FormationMode::Helix);
        app.world_mut()
            .resource_mut::<crate::presentation::Presentation>()
            .paused = true;
        app.world_mut()
            .resource_mut::<Time<Real>>()
            .advance_by(Duration::from_millis(100));
        app.update();
        assert_eq!(app.world().resource::<FormationFrame>().morph, 0.0);
        app.world_mut()
            .resource_mut::<crate::presentation::Presentation>()
            .paused = false;
        app.world_mut()
            .resource_mut::<crate::presentation::Presentation>()
            .speed = 2.0;
        app.world_mut()
            .resource_mut::<Time<Real>>()
            .advance_by(Duration::from_millis(100));
        app.update();
        let frame = app.world().resource::<FormationFrame>();
        assert!((frame.samples[0] - 0.025).abs() < 1e-6);
        assert!((frame.samples[1] - 0.05).abs() < 1e-6);
        assert_eq!(frame.morph, frame.samples[1]);
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
            assert!((app.world().resource::<FormationFrame>().morph - 0.05).abs() < 1e-6);
        }
        assert!(!app
            .world()
            .get_resource_ref::<FormationFrame>()
            .unwrap()
            .is_changed());
    }
}
