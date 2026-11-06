//! Curated visitor scenes. Detailed experiments keep their existing Lab contracts.
use bevy::{camera_controller::pan_orbit_camera::prelude::*, prelude::*};
use bevy_egui::{egui, EguiContexts};

use crate::{
    environment::Environment,
    presentation::Presentation,
    scene::{Appearance, HeadingPalette},
    showcase::{self, Showcase},
    species::{self, Species},
    swarm::SwarmScene,
    vehicle,
    vehicle_appearance::ColorMode,
    GlobalState,
};

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(crate) enum VisitorScene {
    #[default]
    Murmuration,
    ReefDrift,
    SharkHunt,
}

impl VisitorScene {
    pub const ALL: [Self; 3] = [Self::Murmuration, Self::ReefDrift, Self::SharkHunt];

    pub fn name(self) -> &'static str {
        match self {
            Self::Murmuration => "Sunset murmuration",
            Self::ReefDrift => "Reef drift",
            Self::SharkHunt => "Shark hunt",
        }
    }

    pub fn parse(name: &str) -> Result<Self, String> {
        match name {
            "murmuration" | "starling" => Ok(Self::Murmuration),
            "reef" | "reef-drift" => Ok(Self::ReefDrift),
            "shark" | "shark-hunt" => Ok(Self::SharkHunt),
            _ => Err(format!(
                "invalid scene '{name}'; expected murmuration, reef or shark"
            )),
        }
    }

    pub fn population(self, tier: PopulationTier) -> usize {
        let counts = match self {
            Self::Murmuration => [8_000, 20_000, 40_000],
            Self::ReefDrift => [500, 1_500, 3_000],
            Self::SharkHunt => [5_000, 12_000, 20_000],
        };
        counts[tier as usize]
    }

    fn species(self) -> Species {
        match self {
            Self::Murmuration => Species::Murmuration,
            Self::ReefDrift => Species::ReefDrift,
            Self::SharkHunt => Species::SharkAttack,
        }
    }

    fn environment(self) -> Environment {
        match self {
            Self::Murmuration => Environment::Sunset,
            Self::ReefDrift | Self::SharkHunt => Environment::Underwater,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(crate) enum PopulationTier {
    Low,
    #[default]
    Medium,
    High,
}

impl PopulationTier {
    pub const ALL: [Self; 3] = [Self::Low, Self::Medium, Self::High];

    pub fn parse(name: &str) -> Result<Self, String> {
        match name {
            "low" => Ok(Self::Low),
            "medium" => Ok(Self::Medium),
            "high" => Ok(Self::High),
            _ => Err(format!(
                "invalid population '{name}'; expected low, medium or high"
            )),
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            Self::Low => "Low",
            Self::Medium => "Medium",
            Self::High => "High",
        }
    }
}

#[derive(Resource, Default)]
pub(crate) struct Visitor {
    pub scene: VisitorScene,
    pub tier: PopulationTier,
    pub reduced_motion: bool,
    pub lab: bool,
    pub help: bool,
}

pub(crate) struct VisitorPlugin;
impl Plugin for VisitorPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<Visitor>()
            .add_systems(PostStartup, startup);
    }
}

fn startup(world: &mut World) {
    // Log the exact public tier contract; these counts are scene-specific, not quality settings.
    for scene in VisitorScene::ALL {
        for tier in PopulationTier::ALL {
            debug!(
                "{} / {}: {} animals",
                scene.name(),
                tier.name(),
                scene.population(tier)
            );
        }
    }
    apply(world, VisitorScene::default(), PopulationTier::default());
}

/// Scene and population selection restart a scattered flock inside the selected scene domain.
/// Pause, UI visibility, reduced camera motion and help survive; Lab edits keep their own continuity.
/// Model cancellation renews its generation so a previous scene cannot complete over this one.
pub(crate) fn apply(world: &mut World, scene: VisitorScene, tier: PopulationTier) {
    let (paused, ui_visible) = {
        let presentation = world.resource::<Presentation>();
        (presentation.paused, presentation.ui_visible)
    };
    let reduced_motion = world
        .get_resource::<Visitor>()
        .is_some_and(|visitor| visitor.reduced_motion);
    let help = world
        .get_resource::<Visitor>()
        .is_some_and(|visitor| visitor.help);
    showcase::apply(world, Showcase::Calm);
    if let Some(mut models) = world.get_resource_mut::<crate::vehicle_model::VehicleModels>() {
        models.clip = "0".into();
        models.animation_speed = 1.0;
    }
    species::apply(world, scene.species());
    world.resource_mut::<GlobalState>().vehicle_count = scene.population(tier);
    if scene == VisitorScene::SharkHunt {
        let mut state = world.resource_mut::<GlobalState>();
        // Give wide shots a clear predator silhouette and time to read each committed pass.
        // The shared pursuit, lunge, panic and recovery rules remain the existing CPU/GPU contract.
        state.groups.table[1].size_scale = 8.0;
        state.groups.table[1].speed_scale = 1.0;
    }
    world.init_resource::<crate::gpu_flock::FlockRestart>();
    {
        let mut restart = world.resource_mut::<crate::gpu_flock::FlockRestart>();
        restart.requested = restart.requested.wrapping_add(1);
    }
    // Public scenes never use an invisible threat or an artistic formation.
    world.resource_mut::<SwarmScene>().predators = default();
    {
        let mut appearance = world.resource_mut::<Appearance>();
        appearance.environment = scene.environment();
        appearance.palette = if scene == VisitorScene::Murmuration {
            HeadingPalette::Dusk
        } else {
            HeadingPalette::Ocean
        };
        appearance.vehicles.color_mode = ColorMode::ModelColors;
        appearance.auto_frame = false;
        appearance.shadows = scene != VisitorScene::Murmuration;
    }
    {
        let mut presentation = world.resource_mut::<Presentation>();
        presentation.paused = paused;
        presentation.ui_visible = ui_visible;
        presentation.orbit = !reduced_motion;
        presentation.orbit_speed = 3.0;
    }
    world.insert_resource(Visitor {
        scene,
        tier,
        reduced_motion,
        lab: false,
        help,
    });
    reset_view(world);
    info!("Visitor scene: {} / {}", scene.name(), tier.name());
}

/// Restore the composed wide shot without changing animals, population, pause or simulation time.
pub(crate) fn reset_view(world: &mut World) {
    let state = world.resource::<GlobalState>();
    let count = state.vehicle_count;
    let scene = world.resource::<Visitor>().scene;
    let (yaw, pitch, spawn_distance): (f32, f32, f32) = match scene {
        // The sunset sun is near world azimuth 50 degrees. Look toward it for warm silhouettes.
        VisitorScene::Murmuration => (225.0, -3.0, 0.65),
        VisitorScene::ReefDrift => (20.0, 3.0, 0.95),
        VisitorScene::SharkHunt => (15.0, 3.0, 0.5),
    };
    let rotation = Quat::from_euler(EulerRot::YXZ, yaw.to_radians(), -pitch.to_radians(), 0.0);
    for (mut transform, mut orbit, projection) in world
        .query_filtered::<(&mut Transform, &mut PanOrbitCamera, &Projection), With<Camera3d>>()
        .iter_mut(world)
    {
        let Projection::Perspective(projection) = projection else {
            continue;
        };
        // Compose around the animal volume, rather than the much larger containment walls.
        // Narrow windows pull back enough to retain a useful horizontal view of the encounter.
        let portrait_compensation = (1.1 / projection.aspect_ratio.max(0.3)).clamp(1.0, 2.0);
        let distance = vehicle::spawn_size(count) * spawn_distance * portrait_compensation;
        *transform =
            Transform::from_translation(rotation * Vec3::Z * distance).with_rotation(rotation);
        orbit.current_motion = default();
        orbit.last_anchor_depth = -(distance as f64);
    }
    let reduced_motion = world.resource::<Visitor>().reduced_motion;
    let mut presentation = world.resource_mut::<Presentation>();
    presentation.cancel_camera_transition();
    presentation.orbit = !reduced_motion;
    presentation.orbit_speed = 3.0;
}

/// Leaving Lab reapplies the public contract, so experiments never leak into visitor scenes.
pub(crate) fn set_lab(world: &mut World, enabled: bool) {
    if enabled {
        world.resource_mut::<Visitor>().lab = true;
    } else {
        let visitor = world.resource::<Visitor>();
        let (scene, tier) = (visitor.scene, visitor.tier);
        apply(world, scene, tier);
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum VisitorCamera {
    Scenic,
    Ride,
    Free,
}

impl VisitorCamera {
    fn current(presentation: &Presentation) -> Self {
        if presentation.follow.enabled() || presentation.director.enabled {
            Self::Ride
        } else if presentation.orbit {
            Self::Scenic
        } else {
            Self::Free
        }
    }
}

fn set_camera(world: &mut World, camera: VisitorCamera) {
    if camera == VisitorCamera::Scenic {
        reset_view(world);
        return;
    }
    let mut presentation = world.resource_mut::<Presentation>();
    presentation.cancel_camera_transition();
    presentation.orbit = false;
    if camera == VisitorCamera::Ride {
        presentation.follow.mode = crate::follow::FollowMode::Chase;
        presentation.follow.next();
    }
}

pub(crate) fn ui(
    mut contexts: EguiContexts,
    mut visitor: ResMut<Visitor>,
    mut presentation: ResMut<Presentation>,
    appearance: Res<Appearance>,
    models: Res<crate::vehicle_model::VehicleModels>,
    mut commands: Commands,
) -> Result {
    if !presentation.ui_visible {
        return Ok(());
    }
    let ctx = contexts.ctx_mut()?;
    let width = (ctx.content_rect().width() - 56.0).clamp(200.0, 292.0);
    let frame = egui::Frame::window(&ctx.global_style())
        .fill(egui::Color32::from_rgb(10, 24, 28))
        .stroke(egui::Stroke::new(
            1.0,
            egui::Color32::from_rgb(73, 100, 101),
        ))
        .corner_radius(12)
        .inner_margin(14);
    egui::Window::new("Flock")
        .id(egui::Id::new("visitor_hud"))
        .title_bar(false)
        .resizable(false)
        .collapsible(false)
        .auto_sized()
        .vscroll(true)
        .max_height((ctx.content_rect().height() - 32.0).max(180.0))
        .fixed_pos([16.0, 16.0])
        .min_width(width)
        .max_width(width)
        .frame(frame)
        .show(ctx, |ui| {
            ui.visuals_mut().override_text_color = Some(egui::Color32::from_rgb(225, 237, 229));
            ui.spacing_mut().item_spacing = egui::vec2(8.0, 7.0);
            ui.horizontal(|ui| {
                ui.label(egui::RichText::new("FLOCK").strong().size(23.0));
                ui.label(
                    egui::RichText::new("Nature in motion")
                        .small()
                        .color(egui::Color32::from_rgb(154, 188, 178)),
                );
            });
            if visitor.lab {
                ui.label("Lab · experiments and diagnostics");
                if ui.button("Return to nature showcase").clicked() {
                    commands.queue(|world: &mut World| set_lab(world, false));
                }
                ui.small("Returning restores this scene's animals, light and motion.");
                return;
            }
            let mut scene = visitor.scene;
            egui::ComboBox::from_id_salt("visitor_scene")
                .selected_text(scene.name())
                .width(width)
                .show_ui(ui, |ui| {
                    for candidate in VisitorScene::ALL {
                        ui.selectable_value(&mut scene, candidate, candidate.name());
                    }
                });
            ui.small(match visitor.scene {
                VisitorScene::Murmuration => "Starlings flow together through the evening sky.",
                VisitorScene::ReefDrift => "Slow fish wander through clear, sunlit water.",
                VisitorScene::SharkHunt => {
                    "Three sharks pursue a school. Watch it scatter and regroup."
                }
            });
            ui.horizontal(|ui| {
                ui.label("Flock size");
                let mut tier = visitor.tier;
                for candidate in PopulationTier::ALL {
                    ui.selectable_value(&mut tier, candidate, candidate.name());
                }
                if scene != visitor.scene || tier != visitor.tier {
                    commands.queue(move |world: &mut World| apply(world, scene, tier));
                }
            });
            ui.horizontal(|ui| {
                ui.label("Camera");
                let current = VisitorCamera::current(&presentation);
                for (camera, label) in [
                    (VisitorCamera::Scenic, "Scenic"),
                    (VisitorCamera::Ride, "Ride"),
                    (VisitorCamera::Free, "Free"),
                ] {
                    if ui.selectable_label(current == camera, label).clicked() {
                        commands.queue(move |world: &mut World| set_camera(world, camera));
                    }
                }
            });
            ui.horizontal(|ui| {
                if ui
                    .button(if presentation.paused {
                        "Resume · P"
                    } else {
                        "Pause · P"
                    })
                    .clicked()
                {
                    presentation.paused = !presentation.paused;
                }
                if ui.button("Reset view").clicked() {
                    commands.queue(reset_view);
                }
            });
            if presentation.follow.enabled()
                && ui
                    .button("Another animal · N")
                    .on_hover_text("Follow a random animal while keeping the current camera mode.")
                    .clicked()
            {
                presentation.follow.next();
            }
            if ui
                .checkbox(&mut visitor.reduced_motion, "Reduce camera motion")
                .changed()
            {
                if visitor.reduced_motion {
                    presentation.cancel_camera_transition();
                    presentation.orbit = false;
                } else {
                    commands.queue(reset_view);
                }
            }
            if models.status.starts_with("Error:") {
                ui.colored_label(egui::Color32::LIGHT_RED, "The animals could not load.");
                if ui.button("Try scene again").clicked() {
                    let (scene, tier) = (visitor.scene, visitor.tier);
                    commands.queue(move |world: &mut World| apply(world, scene, tier));
                }
            } else if !appearance.environment_ready() || models.status.starts_with("Loading ") {
                ui.small("Preparing animals and light…");
            }
            ui.separator();
            ui.horizontal(|ui| {
                ui.toggle_value(&mut visitor.help, "Help");
                if ui.small_button("Hide · H").clicked() {
                    presentation.ui_visible = false;
                }
                if ui.small_button("Lab").clicked() {
                    visitor.lab = true;
                }
            });
            if visitor.help {
                ui.small("Right-drag to orbit · scroll to zoom · middle-drag to pan.");
                ui.small(
                    "Ride follows one animal. Another animal or N picks a new subject. Reset view returns to the scene.",
                );
                ui.small("P pauses animals. H hides or restores the interface.");
                ui.small(
                    "Scene and size changes restart the flock. Reset view only moves the camera.",
                );
            }
        });
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        formation::FormationMode, predator::PredatorPath, swarm::GroupRole, SimulationMode,
    };

    fn world() -> World {
        let mut world = World::new();
        world.insert_resource(GlobalState::default());
        world.insert_resource(Appearance::default());
        world.insert_resource(Presentation::default());
        world.insert_resource(SwarmScene::default());
        world.insert_resource(Visitor::default());
        world
    }

    fn assert_scene(world: &World, scene: VisitorScene, tier: PopulationTier) {
        let state = world.resource::<GlobalState>();
        let appearance = world.resource::<Appearance>();
        let swarm = world.resource::<SwarmScene>();
        let presentation = world.resource::<Presentation>();
        assert_eq!(state.mode, SimulationMode::Gpu);
        assert_eq!(state.vehicle_count, scene.population(tier));
        assert_eq!(appearance.environment, scene.environment());
        assert_eq!(appearance.vehicles.color_mode, ColorMode::ModelColors);
        assert!(!appearance.auto_frame);
        assert!(!appearance.volumetric);
        assert!(!presentation.motion_blur);
        assert!(!presentation.director.enabled);
        assert!(!presentation.follow.enabled());
        assert!(swarm.predators.objects.is_empty());
        assert_eq!(swarm.formation.mode, FormationMode::Off);
        assert!(swarm.obstacles.is_empty());
        if scene == VisitorScene::SharkHunt {
            assert_eq!(state.groups.count, 2);
            assert_eq!(state.groups.table[1].role, GroupRole::Hunter);
            assert_eq!(state.groups.hunter_count(), 3);
        } else {
            assert_eq!(state.groups.hunter_count(), 0);
            assert!(!state.groups.has_targets());
        }
        assert_eq!(world.resource::<Visitor>().scene, scene);
        assert_eq!(world.resource::<Visitor>().tier, tier);
    }

    #[test]
    fn public_scenes_replace_every_incompatible_lab_family() {
        for scene in VisitorScene::ALL {
            for lab in Showcase::ALL {
                let mut world = world();
                showcase::apply(&mut world, lab);
                world.resource_mut::<GlobalState>().mode = SimulationMode::Octree;
                world.resource_mut::<SwarmScene>().predators.path = PredatorPath::Orbit;
                apply(&mut world, scene, PopulationTier::Medium);
                assert_scene(&world, scene, PopulationTier::Medium);
            }
            for lab in Species::FISH
                .into_iter()
                .chain(Species::BIRDS)
                .chain(Species::HUNTS)
            {
                let mut world = world();
                species::apply(&mut world, lab);
                apply(&mut world, scene, PopulationTier::Medium);
                assert_scene(&world, scene, PopulationTier::Medium);
            }
        }
    }

    #[test]
    fn switching_and_repeated_selection_restore_one_scene_contract() {
        let mut world = world();
        for _ in 0..4 {
            for from in VisitorScene::ALL {
                for to in VisitorScene::ALL {
                    for tier in PopulationTier::ALL {
                        apply(&mut world, from, PopulationTier::High);
                        apply(&mut world, to, tier);
                        assert_scene(&world, to, tier);
                        let before = world.resource::<GlobalState>().clone();
                        apply(&mut world, to, tier);
                        assert_eq!(*world.resource::<GlobalState>(), before);
                    }
                }
            }
        }
    }

    #[test]
    fn scene_switch_preserves_viewer_preferences_and_applied_environment() {
        let mut world = world();
        world.resource_mut::<Appearance>().applied_environment = Some(Environment::Sunset);
        world.resource_mut::<Visitor>().reduced_motion = true;
        world.resource_mut::<Presentation>().paused = true;
        world.resource_mut::<Presentation>().ui_visible = false;
        apply(&mut world, VisitorScene::SharkHunt, PopulationTier::Low);
        assert_scene(&world, VisitorScene::SharkHunt, PopulationTier::Low);
        let presentation = world.resource::<Presentation>();
        assert!(presentation.paused);
        assert!(!presentation.ui_visible);
        assert!(!presentation.orbit);
        assert!(world.resource::<Visitor>().reduced_motion);
        let appearance = world.resource::<Appearance>();
        assert_eq!(appearance.active_environment(), Environment::Sunset);
        assert!(!appearance.environment_ready());
    }

    #[test]
    fn leaving_lab_restores_public_scene_and_reduced_motion() {
        let mut world = world();
        apply(&mut world, VisitorScene::SharkHunt, PopulationTier::Low);
        world.resource_mut::<Visitor>().reduced_motion = true;
        set_lab(&mut world, true);
        showcase::apply(&mut world, Showcase::Neon);
        world.resource_mut::<GlobalState>().mode = SimulationMode::Naive;
        set_lab(&mut world, false);
        assert_scene(&world, VisitorScene::SharkHunt, PopulationTier::Low);
        assert!(!world.resource::<Visitor>().lab);
        assert!(!world.resource::<Presentation>().orbit);
    }

    #[test]
    fn camera_choices_and_reset_leave_animal_settings_intact() {
        let mut world = world();
        apply(
            &mut world,
            VisitorScene::Murmuration,
            PopulationTier::Medium,
        );
        let camera = world
            .spawn((
                Camera3d::default(),
                Transform::default(),
                PanOrbitCamera::default(),
                Projection::default(),
            ))
            .id();
        let before = world.resource::<GlobalState>().clone();
        set_camera(&mut world, VisitorCamera::Ride);
        assert!(world.resource::<Presentation>().follow.enabled());
        assert!(!world.resource::<Presentation>().orbit);
        set_camera(&mut world, VisitorCamera::Free);
        assert!(!world.resource::<Presentation>().follow.enabled());
        assert!(!world.resource::<Presentation>().orbit);
        set_camera(&mut world, VisitorCamera::Scenic);
        assert!(world.resource::<Presentation>().orbit);
        let transform = *world.get::<Transform>(camera).unwrap();
        assert!(transform.translation.is_finite());
        assert!(transform.translation.length() > 1.0);
        assert!(
            (transform.rotation * Vec3::NEG_Z).dot(-transform.translation.normalize()) > 0.9999
        );
        reset_view(&mut world);
        assert_eq!(*world.get::<Transform>(camera).unwrap(), transform);
        assert_eq!(*world.resource::<GlobalState>(), before);
    }

    #[test]
    fn scene_and_tier_changes_restart_but_camera_reset_does_not() {
        let mut world = world();
        apply(
            &mut world,
            VisitorScene::Murmuration,
            PopulationTier::Medium,
        );
        assert_eq!(
            world.resource::<crate::gpu_flock::FlockRestart>().requested,
            1
        );
        reset_view(&mut world);
        assert_eq!(
            world.resource::<crate::gpu_flock::FlockRestart>().requested,
            1
        );
        apply(&mut world, VisitorScene::Murmuration, PopulationTier::High);
        assert_eq!(
            world.resource::<crate::gpu_flock::FlockRestart>().requested,
            2
        );
        apply(&mut world, VisitorScene::Murmuration, PopulationTier::High);
        assert_eq!(
            world.resource::<crate::gpu_flock::FlockRestart>().requested,
            3
        );
        apply(&mut world, VisitorScene::ReefDrift, PopulationTier::Low);
        assert_eq!(
            world.resource::<crate::gpu_flock::FlockRestart>().requested,
            4
        );
    }

    #[test]
    fn reset_view_preserves_simulation_and_stops_subject_camera() {
        let mut world = world();
        apply(&mut world, VisitorScene::ReefDrift, PopulationTier::Medium);
        let before = world.resource::<GlobalState>().clone();
        world.resource_mut::<Presentation>().set_director(true);
        world.resource_mut::<Presentation>().paused = true;
        reset_view(&mut world);
        assert_eq!(*world.resource::<GlobalState>(), before);
        let presentation = world.resource::<Presentation>();
        assert!(presentation.paused);
        assert!(!presentation.director.enabled);
        assert!(!presentation.follow.enabled());
    }
}
