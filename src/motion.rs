//! Render orientation and procedural-motion settings. Steering never reads these values.
use crate::vehicle_appearance::VehicleShape;
use bevy::prelude::*;

pub(crate) const RESPONSE: f32 = 8.0;
const POLE_CONE: f32 = 0.1;
const MAX_BANK: f32 = std::f32::consts::FRAC_PI_3;

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct MotionSettings {
    pub bank: f32,
    pub classic_heading: bool,
    pub flap: f32,
    pub frequency: f32,
}
impl Default for MotionSettings {
    fn default() -> Self {
        Self {
            bank: 0.7,
            classic_heading: false,
            flap: 0.45,
            frequency: 1.0,
        }
    }
}
impl MotionSettings {
    pub(crate) fn procedural_kind(self, shape: VehicleShape) -> u32 {
        if self.flap == 0.0 {
            return 0;
        }
        match shape {
            VehicleShape::Bird => 1,
            VehicleShape::Fish => 2,
            VehicleShape::Chevron => 3,
            _ => 0,
        }
    }
}

fn tangent(vector: Vec3, heading: Vec3, fallback: Vec3) -> Vec3 {
    (vector - heading * vector.dot(heading))
        .try_normalize()
        .unwrap_or(fallback)
}

fn signed_angle(from: Vec3, to: Vec3, axis: Vec3) -> f32 {
    let sin = axis.dot(from.cross(to));
    let cos = from.dot(to);
    if sin.abs() < 1e-6 && cos < 0.0 {
        std::f32::consts::PI
    } else {
        sin.atan2(cos)
    }
}

pub(crate) fn bank_angle(lateral: f32, max_speed: f32, strength: f32) -> f32 {
    (lateral.atan2((max_speed * 0.35).max(1e-6)) * strength).clamp(-MAX_BANK, MAX_BANK)
}

fn target(
    previous: Quat,
    heading: Vec3,
    acceleration: Vec3,
    max_speed: f32,
    strength: f32,
) -> Quat {
    let arc = Quat::from_rotation_arc(Vec3::Y, heading);
    let arc_up = arc * Vec3::Z;
    let prior_up = tangent(previous * Vec3::Z, heading, arc_up);
    let projected = Vec3::Y - heading * heading.y;
    let up = projected.try_normalize().unwrap_or(prior_up);
    let right = heading.cross(up);
    let lateral = acceleration - heading * acceleration.dot(heading);
    let bank = bank_angle(lateral.dot(right), max_speed, strength);
    let old_roll = signed_angle(arc_up, prior_up, heading);
    let upright_roll = signed_angle(arc_up, up, heading);
    let mut delta = upright_roll + bank - old_roll;
    if delta > std::f32::consts::PI {
        delta -= std::f32::consts::TAU;
    }
    if delta <= -std::f32::consts::PI {
        delta += std::f32::consts::TAU;
    }
    if (delta.abs() - std::f32::consts::PI).abs() < 1e-5 {
        delta = std::f32::consts::PI;
    }
    // Parallel transport at the pole; smoothly restore the world-up horizon outside it.
    let t = (projected.length_squared() / POLE_CONE).clamp(0.0, 1.0);
    let weight = t * t * (3.0 - 2.0 * t);
    (Quat::from_axis_angle(heading, old_roll + delta * weight) * arc).normalize()
}

pub(crate) fn orientation(
    previous: Quat,
    heading: Vec3,
    old_velocity: Vec3,
    velocity: Vec3,
    dt: f32,
    settings: MotionSettings,
    max_speed: f32,
) -> Quat {
    if heading == Vec3::ZERO {
        return previous;
    }
    // Keep the original call and its normalized input unchanged, including float bits.
    if settings.classic_heading {
        return Quat::from_rotation_arc(Vec3::Y, heading);
    }
    if dt <= 0.0 {
        return previous;
    }
    let desired = target(
        previous,
        heading,
        (velocity - old_velocity) / dt,
        max_speed,
        settings.bank,
    );
    previous
        .slerp(desired, 1.0 - (-RESPONSE * dt).exp())
        .normalize()
}

pub(crate) fn initial(velocity: Vec3) -> Quat {
    let heading = velocity.normalize_or_zero();
    if heading == Vec3::ZERO {
        return Quat::IDENTITY;
    }
    let arc = Quat::from_rotation_arc(Vec3::Y, heading);
    target(arc, heading, Vec3::ZERO, 1.0, 0.0)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn bank_turn_sign_clamp_and_speed_change() {
        assert_eq!(bank_angle(0.0, 80.0, 1.0), 0.0);
        assert_eq!(bank_angle(1000.0, 80.0, 0.0), 0.0);
        assert_eq!(bank_angle(1000.0, 80.0, 1.0), MAX_BANK);
        assert_eq!(bank_angle(-1000.0, 80.0, 1.0), -MAX_BANK);
        let q = orientation(
            initial(Vec3::Z),
            Vec3::Z,
            Vec3::Z * 40.0,
            Vec3::Z * 41.0,
            1.0 / 60.0,
            MotionSettings::default(),
            80.0,
        );
        assert!((q * Vec3::Z).distance(Vec3::Y) < 1e-6);
    }
    #[test]
    fn exponential_bank_agrees_at_30_60_144_hz() {
        let mut results = Vec::new();
        for hz in [30, 60, 144] {
            let dt = 1.0 / hz as f32;
            let velocity = Vec3::Z * 40.0;
            let mut q = initial(velocity);
            for _ in 0..hz {
                q = orientation(
                    q,
                    Vec3::Z,
                    velocity + Vec3::X * 20.0 * dt,
                    velocity,
                    dt,
                    MotionSettings::default(),
                    80.0,
                );
            }
            results.push(q);
        }
        for q in &results {
            assert!(q.dot(results[0]).abs() > 1.0 - 1e-6);
        }
        assert!((results[0] * Vec3::Z).dot(Vec3::NEG_X) > 0.3);
    }
    #[test]
    fn stable_up_and_continuous_pole_crossing() {
        let settings = MotionSettings {
            bank: 0.0,
            ..default()
        };
        for heading in [Vec3::X, Vec3::Z, Vec3::NEG_X, Vec3::NEG_Z] {
            let q = initial(heading);
            assert!((q * Vec3::Y).distance(heading) < 1e-6);
            assert!((q * Vec3::Z).distance(Vec3::Y) < 1e-6);
        }
        for pole in [1.0, -1.0] {
            let mut last = initial(Vec3::new(-0.5, pole, 0.0));
            for i in -50..=50 {
                let h = Vec3::new(i as f32 * 0.01, pole, 0.0).normalize();
                let next = orientation(last, h, h * 40.0, h * 40.0, 1.0 / 144.0, settings, 80.0);
                assert!(next.is_finite());
                assert!(next.angle_between(last) < 0.2, "pole={pole} i={i}");
                last = next;
            }
        }
    }
    #[test]
    fn legacy_bits_pause_and_stationary_fallback() {
        let settings = MotionSettings {
            classic_heading: true,
            ..default()
        };
        let previous = Quat::from_rotation_x(0.5);
        for h in [
            Vec3::Y,
            Vec3::NEG_Y,
            Vec3::X,
            Vec3::new(0.1, 0.9, 0.3).normalize(),
        ] {
            assert_eq!(
                orientation(previous, h, Vec3::ZERO, h, 0.1, settings, 80.0)
                    .to_array()
                    .map(f32::to_bits),
                Quat::from_rotation_arc(Vec3::Y, h)
                    .to_array()
                    .map(f32::to_bits)
            );
        }
        assert_eq!(
            orientation(
                previous,
                Vec3::ZERO,
                Vec3::ZERO,
                Vec3::ZERO,
                0.1,
                settings,
                80.0
            ),
            previous
        );
        assert_eq!(
            orientation(
                previous,
                Vec3::X,
                Vec3::X,
                Vec3::X,
                0.0,
                MotionSettings::default(),
                80.0
            ),
            previous
        );
    }
}
