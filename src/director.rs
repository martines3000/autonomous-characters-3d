//! Pure shot scheduling and composition. Subject sampling remains in `follow`.
use bevy::prelude::*;

pub(crate) const MIN_SHOT_LENGTH: f32 = 6.0;
pub(crate) const MAX_SHOT_LENGTH: f32 = 10.0;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(crate) enum Shot {
    #[default]
    Chase,
    Side,
    Flyby,
    Orbit,
    Wide,
}
impl Shot {
    const ALL: [Self; 5] = [
        Self::Chase,
        Self::Side,
        Self::Flyby,
        Self::Orbit,
        Self::Wide,
    ];
    pub(crate) fn name(self) -> &'static str {
        match self {
            Self::Chase => "Close chase",
            Self::Side => "Side tracking",
            Self::Flyby => "Flyby",
            Self::Orbit => "Orbit",
            Self::Wide => "Establishing",
        }
    }
    fn next(self) -> Self {
        Self::ALL[(Self::ALL.iter().position(|&s| s == self).unwrap() + 1) % 5]
    }
}

pub(crate) struct Director {
    pub enabled: bool,
    pub shot_length: f32,
    pub shake: f32,
    pub speed_ramp: bool,
    pub shot: Shot,
    pub elapsed: f32,
    total: f32,
    anchor: Option<(Vec3, Vec3)>,
    pass_gap: f32,
}
impl Default for Director {
    fn default() -> Self {
        Self {
            enabled: false,
            shot_length: 8.0,
            shake: 0.0,
            speed_ramp: true,
            shot: Shot::Chase,
            elapsed: 0.0,
            total: 0.0,
            anchor: None,
            pass_gap: 1.0,
        }
    }
}
impl Director {
    pub(crate) fn start(&mut self) {
        self.enabled = true;
        self.shot = Shot::Chase;
        self.elapsed = 0.0;
        self.total = 0.0;
        self.reset_subject();
    }
    pub(crate) fn reset_subject(&mut self) {
        self.anchor = None;
        self.pass_gap = 1.0;
    }
    /// Returns true when the next shot needs a new subject and transition.
    pub(crate) fn advance(&mut self, dt: f32) -> bool {
        if !self.enabled {
            return false;
        }
        self.elapsed += dt;
        self.total += dt;
        let mut changed = false;
        while self.elapsed >= self.shot_length {
            self.elapsed -= self.shot_length;
            self.shot = self.shot.next();
            self.reset_subject();
            changed = true;
        }
        changed
    }
    pub(crate) fn speed_multiplier(&self) -> f32 {
        if self.enabled && self.speed_ramp && self.shot == Shot::Flyby {
            0.3 + 0.7 * smooth(self.pass_gap.abs())
        } else {
            1.0
        }
    }
    /// Desired world-space eye; the follow spring and transition smooth its motion.
    pub(crate) fn eye(&mut self, subject: Vec3, heading: Vec3, scale: f32) -> Vec3 {
        let side = heading.cross(Vec3::Y).try_normalize().unwrap_or(Vec3::X);
        let t = self.elapsed / self.shot_length;
        let (origin, direction) = *self.anchor.get_or_insert((subject, heading));
        let eye = match self.shot {
            Shot::Chase => {
                subject - heading * (9.0 * scale)
                    + Vec3::Y * (2.5 * scale)
                    + side * ((t - 0.5) * 5.0 * scale)
            }
            Shot::Side => subject + side * (13.0 * scale) + Vec3::Y * (3.0 * scale),
            Shot::Flyby => {
                let eye = origin
                    + direction * (28.0 * scale)
                    + side_for(direction) * (7.0 * scale)
                    + Vec3::Y * (3.0 * scale);
                self.pass_gap = (eye - subject).dot(direction) / (18.0 * scale);
                eye
            }
            Shot::Orbit => {
                subject
                    + Quat::from_rotation_y(t * 1.5) * side * (16.0 * scale)
                    + Vec3::Y * (6.0 * scale)
            }
            Shot::Wide => subject + Vec3::new(130.0, 180.0, 220.0) * scale * (1.0 - t * 0.25),
        };
        eye + shake(self.total) * (self.shake * 0.15 * scale)
    }
}
fn side_for(heading: Vec3) -> Vec3 {
    heading.cross(Vec3::Y).try_normalize().unwrap_or(Vec3::X)
}
fn smooth(t: f32) -> f32 {
    let t = t.clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}
/// Continuous, low-frequency deterministic noise with vector length at most one.
fn shake(t: f32) -> Vec3 {
    Vec3::new(
        (t * 1.7).sin() * 0.35 + (t * 2.9 + 1.1).sin() * 0.15,
        (t * 2.1 + 2.0).sin() * 0.35 + (t * 3.3).sin() * 0.15,
        (t * 1.3 + 4.0).sin() * 0.35 + (t * 2.7).sin() * 0.15,
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn scheduling_preserves_remainder_and_cycles_all_shots() {
        let mut director = Director::default();
        assert!(!director.advance(100.0));
        director.start();
        for shot in [
            Shot::Side,
            Shot::Flyby,
            Shot::Orbit,
            Shot::Wide,
            Shot::Chase,
        ] {
            assert!(!director.advance(7.9));
            assert!(director.advance(0.1));
            assert_eq!(director.shot, shot);
            assert!(director.elapsed.abs() < 0.00001);
        }
        assert!(director.advance(17.0));
        assert_eq!(director.shot, Shot::Flyby);
        assert!((director.elapsed - 1.0).abs() < 0.00001);
        assert_eq!(crate::presentation::transition_smoothstep(0.0), 0.0);
        assert_eq!(crate::presentation::transition_smoothstep(0.3), 0.5);
        assert_eq!(crate::presentation::transition_smoothstep(0.6), 1.0);
    }
    #[test]
    fn shake_is_bounded_smooth_and_deterministic() {
        for i in 0..10000 {
            let t = i as f32 * 0.01;
            assert_eq!(shake(t), shake(t));
            assert!(shake(t).length() <= 1.0);
            assert!(shake(t).distance(shake(t + 0.001)) < 0.004);
        }
    }
    #[test]
    fn flyby_stays_fixed_and_ramp_restores_user_speed() {
        let mut d = Director::default();
        d.start();
        d.shot = Shot::Flyby;
        let eye = d.eye(Vec3::ZERO, Vec3::X, 1.0);
        assert_eq!(d.eye(Vec3::X * 28.0, Vec3::Z, 1.0), eye);
        assert!((d.speed_multiplier() - 0.3).abs() < 0.00001);
        let user_speed = 2.0;
        assert!((user_speed * d.speed_multiplier() - 0.6).abs() < 0.00001);
        d.enabled = false;
        assert_eq!(user_speed * d.speed_multiplier(), user_speed);
        d.enabled = true;
        d.speed_ramp = false;
        assert_eq!(d.speed_multiplier(), 1.0);
        d.speed_ramp = true;
        d.advance(8.0);
        assert_eq!(d.speed_multiplier(), 1.0);
    }
}
