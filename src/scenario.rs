//! Opt-in desktop scenarios. Parse the entire script before opening the window.
use std::{collections::VecDeque, path::PathBuf};

use bevy::{
    app::AppExit,
    camera_controller::pan_orbit_camera::prelude::*,
    prelude::*,
    render::view::screenshot::{save_to_disk, Screenshot},
    window::{PresentMode, PrimaryWindow},
};

use crate::{
    presentation::{CameraPreset, Presentation, MAX_SPEED, MIN_SPEED},
    scene::{Appearance, HeadingPalette, SceneStyle},
    swarm::AttractorMode,
    target::{Target, TargetSelection},
    vehicle_appearance::{ColorMode, VehicleShape},
    vehicle_model::{ModelAxis, ModelSettings, VehicleModels},
    GlobalState, SimulationMode,
};

pub struct ScenarioPlugin;
impl Plugin for ScenarioPlugin {
    fn build(&self, app: &mut App) {
        let Some(path) = std::env::var_os("AC3D_SCRIPT") else {
            return;
        };
        let script = std::fs::read_to_string(&path).unwrap_or_else(|error| {
            panic!("AC3D_SCRIPT {}: {error}", PathBuf::from(&path).display())
        });
        let commands = parse(&script).unwrap_or_else(|error| {
            panic!("AC3D_SCRIPT {}: {error}", PathBuf::from(&path).display())
        });
        // Opt-in GPU timestamp spans for measuring substep cost in the real application.
        if std::env::var_os("AC3D_GPU_TIMING").is_some() {
            app.add_plugins(bevy::render::diagnostic::RenderDiagnosticsPlugin);
        }
        app.insert_resource(Scenario {
            commands: commands.into(),
            measurement: None,
        })
        .add_systems(
            PreUpdate,
            run_scenario
                .before(crate::camera_input)
                .before(PanOrbitCamera::update_camera_positions),
        );
    }
}

#[derive(Debug, PartialEq)]
struct TimedCommand {
    seconds: f64,
    command: Command,
}

#[derive(Debug, PartialEq)]
enum Command {
    Set(Setting),
    Follow(bool),
    Ui(bool),
    BoidFollow(crate::follow::FollowMode),
    FollowNext,
    FollowId(u32),
    Pause(bool),
    Preset(CameraPreset),
    Target(Vec3),
    Attractor(Option<Vec3>),
    ObstacleAdd(Vec3, f32),
    ObstacleClear,
    PredatorAdd(Vec3, f32),
    PredatorClear,
    Camera { yaw: f32, pitch: f32, distance: f32 },
    Frame,
    Present(bool),
    Screenshot(PathBuf),
    MeasureBegin(String),
    MeasureEnd,
    Exit,
}

#[derive(Debug, PartialEq)]
enum Setting {
    Count(usize),
    Groups(u32),
    FormationMode(crate::formation::FormationMode),
    PredatorStrength(f32),
    PredatorPath(crate::predator::PredatorPath),
    PredatorGroups(u32),
    PredatorPathRadius(f32),
    PredatorPathSpeed(f32),
    PredatorCenter(Vec3),
    PredatorsVisible(bool),
    FormationCenter(Vec3),
    FormationExtent(f32),
    FormationArrival(f32),
    FormationStrength(f32),
    FormationMorph(f32),
    FormationMorphSpeed(f32),
    GroupTarget(usize, Option<Vec3>),
    GroupColor(usize, [u8; 3]),
    AttractorMode(AttractorMode),
    AttractorStrength(f32),
    AttractorRadius(f32),
    ObstacleStrength(f32),
    ObstacleClearance(f32),
    ObstaclesVisible(bool),
    Speed(f32),
    Orbit(bool),
    OrbitSpeed(f32),
    Mode(SimulationMode),
    Style(SceneStyle),
    Glow(f32),
    Streak(f32),
    Dof(bool),
    Palette(HeadingPalette),
    AutoFrame(bool),
    FlockDraw(bool),
    Msaa(Msaa),
    FollowSamples(u32),
    OctreeSize(usize),
    Number(NumberSetting, f32),
    Shape(VehicleShape),
    Model(PathBuf),
    ModelForward(ModelAxis),
    ModelUp(ModelAxis),
    ModelScale(f32),
    AnimationClip(String),
    AnimationSpeed(f32),
    VisualSize(f32),
    Length(f32),
    Width(f32),
    ColorMode(ColorMode),
    SolidColor([u8; 3]),
}

// Each numeric setting keeps its name, range and destination together for later stages.
macro_rules! number_settings {
    ($($variant:ident => $field:ident, $min:expr, $max:expr;)+) => {
        #[derive(Debug, PartialEq)]
        enum NumberSetting { $($variant,)+ }
        impl NumberSetting {
            fn parse(name: &str, value: &str) -> Result<Setting, String> {
                match name {
                    $(stringify!($field) => Ok(Setting::Number(Self::$variant, number(value, $min, $max)?)),)+
                    _ => Err(format!("unknown setting '{name}'")),
                }
            }
            fn apply(self, value: f32, state: &mut GlobalState) {
                match self { $(Self::$variant => state.$field = value,)+ }
            }
        }
    }
}
number_settings! {
    Mass => vehicle_mass, 1.0, 1000.0;
    MaxSpeed => vehicle_max_speed, 0.1, 100.0;
    WanderSpeed => vehicle_wander_speed, 0.1, 100.0;
    SeparationFactor => vehicle_separation_factor, 0.1, 10.0;
    AlignmentFactor => vehicle_alignment_factor, 0.1, 10.0;
    CohesionFactor => vehicle_cohesion_factor, 0.1, 10.0;
    WanderFactor => vehicle_wander_factor, 0.1, 10.0;
    WallAvoidFactor => vehicle_wall_avoid_factor, 0.1, 10.0;
    SeekFactor => vehicle_seek_factor, 0.1, 10.0;
    SeparationDistance => vehicle_separation_distance, 1.0, 100.0;
    AlignmentDistance => vehicle_alignment_distance, 1.0, 100.0;
    CohesionDistance => vehicle_cohesion_distance, 1.0, 100.0;
    WanderDistance => vehicle_wander_distance, 1.0, 100.0;
    WanderRadius => vehicle_wander_radius, 1.0, 100.0;
}

fn number(value: &str, min: f32, max: f32) -> Result<f32, String> {
    let parsed = value
        .parse::<f32>()
        .map_err(|_| format!("invalid number '{value}'"))?;
    if !parsed.is_finite() || !(min..=max).contains(&parsed) {
        return Err(format!("'{value}' must be finite and in {min}..={max}"));
    }
    Ok(parsed)
}

fn integer(value: &str, min: usize, max: usize) -> Result<usize, String> {
    let parsed = value
        .parse::<usize>()
        .map_err(|_| format!("invalid integer '{value}'"))?;
    if !(min..=max).contains(&parsed) {
        return Err(format!("'{value}' must be in {min}..={max}"));
    }
    Ok(parsed)
}

fn switch(value: &str) -> Result<bool, String> {
    match value {
        "on" | "true" => Ok(true),
        "off" | "false" => Ok(false),
        _ => Err(format!("expected on|off, got '{value}'")),
    }
}

fn setting(name: &str, value: &str, mode: SimulationMode) -> Result<Setting, String> {
    Ok(match (name, value) {
        ("predator_strength", _) => Setting::PredatorStrength(number(value, 0.0, 20.0)?),
        ("predator_path", "off") => Setting::PredatorPath(crate::predator::PredatorPath::Off),
        ("predator_path", "orbit") => Setting::PredatorPath(crate::predator::PredatorPath::Orbit),
        ("predator_groups", _) => Setting::PredatorGroups(integer(value, 0, 255)? as u32),
        ("predator_path_radius", _) => Setting::PredatorPathRadius(number(value, 1.0, 200.0)?),
        ("predator_path_speed", _) => Setting::PredatorPathSpeed(number(value, -180.0, 180.0)?),
        ("predators_visible", _) => Setting::PredatorsVisible(switch(value)?),
        ("formation", "off") => Setting::FormationMode(crate::formation::FormationMode::Off),
        ("formation", "sphere") => Setting::FormationMode(crate::formation::FormationMode::Sphere),
        ("formation", "helix") => Setting::FormationMode(crate::formation::FormationMode::Helix),
        ("formation_extent", _) => Setting::FormationExtent(number(value, 1.0, 200.0)?),
        ("formation_arrival_radius", _) => Setting::FormationArrival(number(value, 0.1, 200.0)?),
        ("formation_strength", _) => Setting::FormationStrength(number(value, 0.0, 20.0)?),
        ("formation_morph", _) => Setting::FormationMorph(number(value, 0.0, 1.0)?),
        ("formation_morph_speed", _) => Setting::FormationMorphSpeed(number(value, 0.0, 2.0)?),
        ("groups", "off") => Setting::Groups(0),
        ("groups", _) => Setting::Groups(integer(value, 2, crate::swarm::MAX_GROUPS)? as u32),
        ("attractor_mode", "off") => Setting::AttractorMode(AttractorMode::Off),
        ("attractor_mode", "attract") => Setting::AttractorMode(AttractorMode::Attract),
        ("attractor_mode", "repel") => Setting::AttractorMode(AttractorMode::Repel),
        ("attractor_strength", _) => Setting::AttractorStrength(number(value, 0.0, 20.0)?),
        ("attractor_radius", _) => Setting::AttractorRadius(number(value, 1.0, 200.0)?),
        ("obstacle_strength", _) => Setting::ObstacleStrength(number(value, 0.0, 20.0)?),
        ("obstacle_clearance", _) => Setting::ObstacleClearance(number(value, 0.1, 200.0)?),
        ("obstacles_visible", _) => Setting::ObstaclesVisible(switch(value)?),
        ("speed", _) => Setting::Speed(number(value, MIN_SPEED, MAX_SPEED)?),
        ("orbit", _) => Setting::Orbit(switch(value)?),
        ("orbit_speed", _) => Setting::OrbitSpeed(number(value, -45.0, 45.0)?),
        ("count" | "vehicle_count", _) => Setting::Count(integer(value, 0, mode.max_vehicles())?),
        ("mode", "gpu") => Setting::Mode(SimulationMode::Gpu),
        ("mode", "octree") => Setting::Mode(SimulationMode::Octree),
        ("mode", "naive") => Setting::Mode(SimulationMode::Naive),
        ("style", "studio") => Setting::Style(SceneStyle::Studio),
        ("style", "classic") => Setting::Style(SceneStyle::Classic),
        ("style", "glow") => Setting::Style(SceneStyle::Glow),
        ("glow", _) => Setting::Glow(number(value, 0.0, 20.0)?),
        ("streak", _) => Setting::Streak(number(value, 0.0, 3.0)?),
        ("dof", _) => Setting::Dof(switch(value)?),
        ("palette", "classic") => Setting::Palette(HeadingPalette::Classic),
        ("palette", "ocean") => Setting::Palette(HeadingPalette::Ocean),
        ("palette", "dusk") => Setting::Palette(HeadingPalette::Dusk),
        ("palette", "aurora") => Setting::Palette(HeadingPalette::Aurora),
        ("shape", "dart") => Setting::Shape(VehicleShape::Dart),
        ("shape", "chevron" | "arrow") => Setting::Shape(VehicleShape::Chevron),
        ("shape", "fish") => Setting::Shape(VehicleShape::Fish),
        ("shape", "bird" | "glider") => Setting::Shape(VehicleShape::Bird),
        ("model", _) => Setting::Model(PathBuf::from(value)),
        ("model_forward", _) => Setting::ModelForward(ModelAxis::parse(value)?),
        ("model_up", _) => Setting::ModelUp(ModelAxis::parse(value)?),
        ("animation_clip", _) => Setting::AnimationClip(value.into()),
        ("animation_speed", _) => Setting::AnimationSpeed(number(value, 0.1, 4.0)?),
        ("model_scale", _) => Setting::ModelScale(number(value, 0.1, 5.0)?),
        ("visual_size", _) => Setting::VisualSize(number(value, 0.1, 5.0)?),
        ("length", _) => Setting::Length(number(value, 0.25, 3.0)?),
        ("width", _) => Setting::Width(number(value, 0.25, 3.0)?),
        ("color_mode", "group") => Setting::ColorMode(ColorMode::Group),
        ("color_mode", "heading") => Setting::ColorMode(ColorMode::Heading),
        ("color_mode", "altitude") => Setting::ColorMode(ColorMode::Altitude),
        ("color_mode", "speed") => Setting::ColorMode(ColorMode::Speed),
        ("color_mode", "model" | "model_colors") => Setting::ColorMode(ColorMode::ModelColors),
        ("color_mode", "solid") => Setting::ColorMode(ColorMode::Solid),
        ("solid_color", _) => Setting::SolidColor(hex_color(value)?),
        ("flock_draw", _) => Setting::FlockDraw(switch(value)?),
        ("msaa", "4") => Setting::Msaa(Msaa::Sample4),
        ("msaa", "off") => Setting::Msaa(Msaa::Off),
        ("auto_frame", _) => Setting::AutoFrame(switch(value)?),
        ("gpu_max_neighbors", _) => Setting::FollowSamples(integer(value, 0, 512)? as u32),
        ("octree_size", _) => Setting::OctreeSize(integer(value, 2, 500)?),
        ("mode" | "style" | "palette" | "shape" | "color_mode", _) => {
            return Err(format!("invalid {name} '{value}'"))
        }
        _ => NumberSetting::parse(name, value)?,
    })
}

fn hex_color(value: &str) -> Result<[u8; 3], String> {
    let hex = value.strip_prefix('#').unwrap_or(value);
    if hex.len() != 6 || !hex.bytes().all(|c| c.is_ascii_hexdigit()) {
        return Err(format!(
            "invalid solid_color '{value}'; expected RRGGBB or three sRGB numbers in 0..=1"
        ));
    }
    Ok(std::array::from_fn(|i| {
        u8::from_str_radix(&hex[i * 2..i * 2 + 2], 16).unwrap()
    }))
}

// Preserve ordinary # comments while allowing a single #RRGGBB color argument.
fn script_line(line: &str) -> &str {
    let mut start = 0;
    if let Some(hash) = line.find('#') {
        let prefix: Vec<_> = line[..hash].split_whitespace().collect();
        let tail = &line[hash + 1..];
        if prefix.len() == 3
            && (prefix[1..] == ["set", "solid_color"] || prefix[1] == "group_color")
            && tail.len() >= 6
            && tail.as_bytes()[..6].iter().all(u8::is_ascii_hexdigit)
            && tail.as_bytes().get(6).is_none_or(u8::is_ascii_whitespace)
        {
            start = hash + 7;
        }
    }
    let end = line[start..].find('#').map_or(line.len(), |i| start + i);
    line[..end].trim()
}

fn parse(script: &str) -> Result<Vec<TimedCommand>, String> {
    let mut commands = Vec::new();
    let mut last = 0.0;
    let mut measuring = false;
    let mut mode = GlobalState::default().mode;
    let mut model_settings = ModelSettings::default();
    let mut obstacle_count = 0;
    let mut predator_count = 0;
    for (index, line) in script.lines().enumerate() {
        let line = script_line(line);
        if line.is_empty() {
            continue;
        }
        let result = (|| {
            let words: Vec<_> = line.split_whitespace().collect();
            let seconds = words[0]
                .parse::<f64>()
                .map_err(|_| "invalid timestamp".to_owned())?;
            if !seconds.is_finite() || seconds < last {
                return Err("timestamps must be finite, nonnegative and nondecreasing".into());
            }
            let command = match &words[1..] {
                ["set", "solid_color", r, g, b] => {
                    let values = [
                        number(r, 0.0, 1.0)?,
                        number(g, 0.0, 1.0)?,
                        number(b, 0.0, 1.0)?,
                    ];
                    Command::Set(Setting::SolidColor(
                        values.map(|v| (v * 255.0).round() as u8),
                    ))
                }
                ["set", name, value] => {
                    let value = setting(name, value, mode)?;
                    if let Setting::Mode(next) = value {
                        mode = next;
                    }
                    match &value {
                        Setting::ModelForward(axis) => model_settings.set_forward(*axis),
                        Setting::ModelUp(axis) => {
                            if !model_settings.forward.perpendicular(*axis) {
                                return Err(
                                    "model_up must be perpendicular to model_forward".into()
                                );
                            }
                            model_settings.up = *axis;
                        }
                        _ => (),
                    }
                    Command::Set(value)
                }
                ["predator", "clear"] => {
                    predator_count = 0;
                    Command::PredatorClear
                }
                ["predator", "add", x, y, z, radius] => {
                    if predator_count >= crate::predator::MAX_PREDATORS {
                        return Err("at most four predators; clear before adding more".into());
                    }
                    predator_count += 1;
                    Command::PredatorAdd(
                        Vec3::new(
                            number(x, -1e7, 1e7)?,
                            number(y, -1e7, 1e7)?,
                            number(z, -1e7, 1e7)?,
                        ),
                        number(radius, 1.0, 200.0)?,
                    )
                }
                ["predator_center", x, y, z] => Command::Set(Setting::PredatorCenter(Vec3::new(
                    number(x, -1e7, 1e7)?,
                    number(y, -1e7, 1e7)?,
                    number(z, -1e7, 1e7)?,
                ))),
                ["formation_center", x, y, z] => Command::Set(Setting::FormationCenter(Vec3::new(
                    number(x, -1e7, 1e7)?,
                    number(y, -1e7, 1e7)?,
                    number(z, -1e7, 1e7)?,
                ))),
                ["obstacle", "clear"] => {
                    obstacle_count = 0;
                    Command::ObstacleClear
                }
                ["obstacle", "add", x, y, z, radius] => {
                    if obstacle_count >= crate::swarm::MAX_OBSTACLES {
                        return Err("at most eight obstacles; clear before adding more".into());
                    }
                    obstacle_count += 1;
                    Command::ObstacleAdd(
                        Vec3::new(
                            number(x, -1e7, 1e7)?,
                            number(y, -1e7, 1e7)?,
                            number(z, -1e7, 1e7)?,
                        ),
                        number(radius, 0.1, 200.0)?,
                    )
                }
                ["group_target", group, "off"] => Command::Set(Setting::GroupTarget(
                    integer(group, 0, crate::swarm::MAX_GROUPS - 1)?,
                    None,
                )),
                ["group_target", group, x, y, z] => Command::Set(Setting::GroupTarget(
                    integer(group, 0, crate::swarm::MAX_GROUPS - 1)?,
                    Some(Vec3::new(
                        number(x, -1e7, 1e7)?,
                        number(y, -1e7, 1e7)?,
                        number(z, -1e7, 1e7)?,
                    )),
                )),
                ["group_color", group, color] => Command::Set(Setting::GroupColor(
                    integer(group, 0, crate::swarm::MAX_GROUPS - 1)?,
                    hex_color(color)?,
                )),
                ["ui", value] => Command::Ui(switch(value)?),
                ["pause"] => Command::Pause(true),
                ["resume"] => Command::Pause(false),
                ["preset", name] => Command::Preset(CameraPreset::parse(name)?),
                ["boidcam", "chase"] => Command::BoidFollow(crate::follow::FollowMode::Chase),
                ["boidcam", "first"] => Command::BoidFollow(crate::follow::FollowMode::FirstPerson),
                ["boidcam", "off"] => Command::BoidFollow(crate::follow::FollowMode::Off),
                ["boidcam", "next"] => Command::FollowNext,
                ["boidcam", "id", id] => {
                    Command::FollowId(integer(id, 0, u32::MAX as usize)? as u32)
                }
                ["follow", value @ ("on" | "off")] => Command::Follow(switch(value)?),
                ["attractor", "mouse"] => Command::Attractor(None),
                ["attractor", x, y, z] => Command::Attractor(Some(Vec3::new(
                    number(x, -1e7, 1e7)?,
                    number(y, -1e7, 1e7)?,
                    number(z, -1e7, 1e7)?,
                ))),
                ["target", x, y, z] => Command::Target(Vec3::new(
                    number(x, -1e7, 1e7)?,
                    number(y, -1e7, 1e7)?,
                    number(z, -1e7, 1e7)?,
                )),
                ["camera", yaw, pitch, distance] => Command::Camera {
                    yaw: number(yaw, -360.0, 360.0)?,
                    pitch: number(pitch, -89.9, 89.9)?,
                    distance: number(distance, 1.0, 1e7)?,
                },
                ["frame"] => Command::Frame,
                ["present", "vsync"] => Command::Present(true),
                ["present", "immediate"] => Command::Present(false),
                ["screenshot", path] => Command::Screenshot(PathBuf::from(path)),
                ["measure", "begin", label] if !measuring => {
                    measuring = true;
                    Command::MeasureBegin((*label).into())
                }
                ["measure", "end"] if measuring => {
                    measuring = false;
                    Command::MeasureEnd
                }
                ["exit"] if !measuring => Command::Exit,
                _ => {
                    return Err(format!(
                        "unknown command, wrong arguments, or unpaired measure: '{line}'"
                    ))
                }
            };
            last = seconds;
            Ok(TimedCommand { seconds, command })
        })();
        commands.push(result.map_err(|error: String| format!("line {}: {error}", index + 1))?);
    }
    if measuring {
        return Err(format!(
            "line {}: measure begin has no measure end",
            script.lines().count()
        ));
    }
    Ok(commands)
}

#[derive(Resource)]
struct Scenario {
    commands: VecDeque<TimedCommand>,
    measurement: Option<Measurement>,
}
struct Measurement {
    label: String,
    samples: Vec<f64>,
}

fn run_scenario(world: &mut World) {
    // This system exists only when AC3D_SCRIPT was supplied; no normal-frame harness overhead.
    let now = world.resource::<Time<Real>>().elapsed_secs_f64();
    let delta_ms = world.resource::<Time<Real>>().delta_secs_f64() * 1000.0;
    world.resource_scope(|world, mut scenario: Mut<Scenario>| {
        if let Some(measurement) = &mut scenario.measurement {
            measurement.samples.push(delta_ms);
        }
        while scenario
            .commands
            .front()
            .is_some_and(|entry| entry.seconds <= now)
        {
            let entry = scenario.commands.pop_front().unwrap();
            info!("scenario t={:.3} {:?}", now, entry.command);
            match entry.command {
                Command::Set(setting) => apply_setting(world, setting),
                Command::Ui(visible) => world.resource_mut::<Presentation>().ui_visible = visible,
                Command::Pause(paused) => world.resource_mut::<Presentation>().paused = paused,
                Command::Preset(preset) => {
                    let mut presentation = world.resource_mut::<Presentation>();
                    presentation.follow.mode = crate::follow::FollowMode::Off;
                    presentation.preset = Some(preset)
                }
                Command::BoidFollow(mode) => {
                    let mut presentation = world.resource_mut::<Presentation>();
                    presentation.cancel_camera_transition();
                    presentation.follow.mode = mode;
                }
                Command::FollowNext => world.resource_mut::<Presentation>().follow.next(),
                Command::FollowId(id) => world.resource_mut::<Presentation>().follow.select(id),
                Command::Follow(follow) => {
                    for mut selection in world
                        .query_filtered::<&mut TargetSelection, With<Target>>()
                        .iter_mut(world)
                    {
                        selection.0 = follow;
                    }
                }
                Command::PredatorAdd(position, radius) => {
                    world
                        .resource_mut::<crate::swarm::SwarmScene>()
                        .predators
                        .add(position, radius);
                }
                Command::PredatorClear => {
                    let mut scene = world.resource_mut::<crate::swarm::SwarmScene>();
                    if !scene.predators.objects.is_empty() {
                        scene.predators.objects.clear();
                    }
                }
                Command::ObstacleAdd(center, radius) => {
                    world
                        .resource_mut::<crate::swarm::SwarmScene>()
                        .add_obstacle(center, radius);
                }
                Command::ObstacleClear => {
                    let mut scene = world.resource_mut::<crate::swarm::SwarmScene>();
                    if !scene.obstacles.is_empty() {
                        scene.obstacles.clear();
                    }
                }
                Command::Attractor(point) => {
                    world.resource_mut::<GlobalState>().attractor.scripted_point = point;
                }
                Command::Target(position) => {
                    for mut transform in world
                        .query_filtered::<&mut Transform, With<Target>>()
                        .iter_mut(world)
                    {
                        transform.translation = position;
                    }
                }
                Command::Camera {
                    yaw,
                    pitch,
                    distance,
                } => {
                    world
                        .resource_mut::<Presentation>()
                        .cancel_camera_transition();
                    for (mut transform, mut orbit) in world
                        .query_filtered::<(&mut Transform, &mut PanOrbitCamera), With<Camera3d>>()
                        .iter_mut(world)
                    {
                        let rotation = Quat::from_euler(
                            EulerRot::YXZ,
                            yaw.to_radians(),
                            -pitch.to_radians(),
                            0.0,
                        );
                        *transform = Transform::from_translation(rotation * Vec3::Z * distance)
                            .with_rotation(rotation);
                        orbit.current_motion = default();
                        orbit.last_anchor_depth = -(distance as f64);
                    }
                }
                Command::Frame => world.resource_mut::<Appearance>().frame_requested = true,
                Command::Present(vsync) => {
                    for mut window in world
                        .query_filtered::<&mut Window, With<PrimaryWindow>>()
                        .iter_mut(world)
                    {
                        window.present_mode = if vsync {
                            PresentMode::AutoVsync
                        } else {
                            PresentMode::AutoNoVsync
                        };
                    }
                }
                Command::Screenshot(path) => {
                    if let Some(parent) = path
                        .parent()
                        .filter(|parent| !parent.as_os_str().is_empty())
                    {
                        std::fs::create_dir_all(parent).expect("scenario screenshot directory");
                    }
                    world
                        .spawn(Screenshot::primary_window())
                        .observe(save_to_disk(path));
                }
                Command::MeasureBegin(label) => {
                    for diagnostic in world
                        .resource_mut::<bevy::diagnostic::DiagnosticsStore>()
                        .iter_mut()
                    {
                        if diagnostic.path().as_str().contains("flock_") {
                            diagnostic.clear_history();
                        }
                    }
                    scenario.measurement = Some(Measurement {
                        label,
                        samples: Vec::new(),
                    })
                }
                Command::MeasureEnd => report(
                    world,
                    scenario.measurement.take().expect("validated measurement"),
                ),
                Command::Exit => {
                    world.write_message(AppExit::Success);
                }
            }
        }
    });
}

fn apply_setting(world: &mut World, setting: Setting) {
    match setting {
        value @ (Setting::PredatorStrength(_)
        | Setting::PredatorPath(_)
        | Setting::PredatorGroups(_)
        | Setting::PredatorPathRadius(_)
        | Setting::PredatorPathSpeed(_)
        | Setting::PredatorCenter(_)
        | Setting::PredatorsVisible(_)) => {
            let mut scene = world.resource_mut::<crate::swarm::SwarmScene>();
            let mut settings = scene.predators.clone();
            match value {
                Setting::PredatorStrength(v) => settings.strength = v,
                Setting::PredatorPath(v) => settings.path = v,
                Setting::PredatorGroups(v) => settings.set_mask(v),
                Setting::PredatorPathRadius(v) => settings.path_radius = v,
                Setting::PredatorPathSpeed(v) => settings.path_speed = v,
                Setting::PredatorCenter(v) => settings.path_center = v,
                Setting::PredatorsVisible(v) => settings.visible = v,
                _ => unreachable!(),
            }
            if settings != scene.predators {
                scene.predators = settings;
            }
        }
        value @ (Setting::FormationMode(_)
        | Setting::FormationCenter(_)
        | Setting::FormationExtent(_)
        | Setting::FormationArrival(_)
        | Setting::FormationStrength(_)
        | Setting::FormationMorph(_)
        | Setting::FormationMorphSpeed(_)) => {
            let mut scene = world.resource_mut::<crate::swarm::SwarmScene>();
            let mut formation = scene.formation;
            match value {
                Setting::FormationMode(mode) => formation.set_mode(mode),
                Setting::FormationCenter(value) => formation.center = value,
                Setting::FormationExtent(value) => formation.extent = value,
                Setting::FormationArrival(value) => formation.arrival_radius = value,
                Setting::FormationStrength(value) => formation.strength = value,
                Setting::FormationMorph(value) => formation.morph = value,
                Setting::FormationMorphSpeed(value) => formation.morph_speed = value,
                _ => unreachable!(),
            }
            if scene.formation != formation {
                scene.formation = formation;
            }
        }
        Setting::ObstacleStrength(value) => {
            let mut scene = world.resource_mut::<crate::swarm::SwarmScene>();
            if scene.obstacle_strength != value {
                scene.obstacle_strength = value;
            }
        }
        Setting::ObstacleClearance(value) => {
            let mut scene = world.resource_mut::<crate::swarm::SwarmScene>();
            if scene.obstacle_clearance != value
                || scene
                    .obstacles
                    .iter()
                    .any(|sphere| sphere.clearance != value)
            {
                scene.set_clearance(value);
            }
        }
        Setting::ObstaclesVisible(value) => {
            let mut scene = world.resource_mut::<crate::swarm::SwarmScene>();
            if scene.show_obstacles != value {
                scene.show_obstacles = value;
            }
        }
        Setting::Speed(value) => world.resource_mut::<Presentation>().speed = value,
        Setting::Orbit(value) => world.resource_mut::<Presentation>().orbit = value,
        Setting::OrbitSpeed(value) => world.resource_mut::<Presentation>().orbit_speed = value,
        Setting::FlockDraw(value) => world.resource_mut::<crate::instancing::FlockDraw>().0 = value,
        Setting::Msaa(value) => {
            for mut msaa in world
                .query_filtered::<&mut Msaa, With<Camera3d>>()
                .iter_mut(world)
            {
                *msaa = value;
            }
        }
        Setting::Style(value) => world.resource_mut::<Appearance>().style = value,
        Setting::Glow(value) => world.resource_mut::<Appearance>().glow = value,
        Setting::Streak(value) => world.resource_mut::<Appearance>().streak = value,
        Setting::Dof(value) => world.resource_mut::<Presentation>().follow.dof = value,
        Setting::Palette(value) => world.resource_mut::<Appearance>().palette = value,
        Setting::AutoFrame(value) => world.resource_mut::<Appearance>().auto_frame = value,
        Setting::Shape(value) => {
            world.resource_mut::<VehicleModels>().cancel();
            world.resource_mut::<Appearance>().vehicles.shape = value;
        }
        Setting::Model(path) => world.resource_mut::<VehicleModels>().request(path),
        Setting::ModelForward(axis) => {
            let mut models = world.resource_mut::<VehicleModels>();
            models.settings.set_forward(axis);
            models.reload();
        }
        Setting::ModelUp(axis) => {
            let mut models = world.resource_mut::<VehicleModels>();
            models.settings.up = axis;
            models.reload();
        }
        Setting::AnimationClip(clip) => {
            let mut models = world.resource_mut::<VehicleModels>();
            models.clip = clip;
            models.reload();
        }
        Setting::AnimationSpeed(speed) => {
            world.resource_mut::<VehicleModels>().animation_speed = speed
        }
        Setting::ModelScale(scale) => {
            let mut models = world.resource_mut::<VehicleModels>();
            models.settings.scale = scale;
            models.reload();
        }
        Setting::VisualSize(value) => {
            world.resource_mut::<Appearance>().vehicles.visual_size = value
        }
        Setting::Length(value) => world.resource_mut::<Appearance>().vehicles.length = value,
        Setting::Width(value) => world.resource_mut::<Appearance>().vehicles.width = value,
        Setting::ColorMode(value) => world.resource_mut::<Appearance>().vehicles.color_mode = value,
        Setting::SolidColor(value) => {
            world.resource_mut::<Appearance>().vehicles.solid_color = value
        }
        setting => {
            let mut state = world.resource_mut::<GlobalState>();
            match setting {
                Setting::Groups(count) => {
                    if state.groups.count != count {
                        state.groups.count = count;
                    }
                }
                Setting::GroupTarget(group, target) => {
                    if state.groups.table[group].target != target {
                        state.groups.table[group].target = target;
                    }
                }
                Setting::GroupColor(group, color) => {
                    if state.groups.table[group].color != color {
                        state.groups.table[group].color = color;
                    }
                }
                Setting::AttractorMode(value) => state.attractor.mode = value,
                Setting::AttractorStrength(value) => state.attractor.strength = value,
                Setting::AttractorRadius(value) => state.attractor.radius = value,
                Setting::Count(value) => state.vehicle_count = value.min(state.mode.max_vehicles()),
                Setting::Mode(value) => {
                    state.mode = value;
                    state.vehicle_count = state.vehicle_count.min(value.max_vehicles());
                }
                Setting::FollowSamples(value) => state.gpu_max_neighbors = value,
                Setting::OctreeSize(value) => state.octree_size = value,
                Setting::Number(field, value) => field.apply(value, &mut state),
                _ => unreachable!("appearance settings handled above"),
            }
        }
    }
}

fn report(world: &mut World, mut measurement: Measurement) {
    for diagnostic in world
        .resource::<bevy::diagnostic::DiagnosticsStore>()
        .iter()
    {
        if diagnostic.path().as_str().contains("flock_") && diagnostic.history_len() > 0 {
            let mut values: Vec<_> = diagnostic.values().copied().collect();
            values.sort_by(f64::total_cmp);
            info!(
                "AC3D_COMPUTE label={} path={} samples={} mean_ms={:.4} p95_ms={:.4}",
                measurement.label,
                diagnostic.path().as_str(),
                values.len(),
                diagnostic.average().unwrap_or(0.0),
                values[(values.len() * 95).div_ceil(100) - 1]
            );
        }
    }
    let window = world
        .query_filtered::<&Window, With<PrimaryWindow>>()
        .single(world)
        .expect("primary window");
    let resolution = (window.physical_width(), window.physical_height());
    let draw = world.resource::<crate::instancing::FlockDraw>().0;
    let (transform, msaa) = world
        .query_filtered::<(&Transform, &Msaa), With<Camera3d>>()
        .single(world)
        .expect("camera");
    info!(
        "AC3D_RENDER camera_distance={:.3} camera_xyz={:?} msaa={} flock_draw={}",
        transform.translation.length(),
        transform.translation,
        msaa.samples(),
        draw
    );
    let presentation = world.resource::<Presentation>();
    info!(
        "AC3D_EFFECTS boidcam={} dof={} aperture={} glow={} streak={}",
        presentation.follow.mode.name(),
        presentation.follow.enabled() && presentation.follow.dof,
        presentation.follow.aperture,
        world.resource::<Appearance>().glow,
        world.resource::<Appearance>().streak
    );
    let state = world.resource::<GlobalState>();
    let appearance = world.resource::<Appearance>();
    let frames = measurement.samples.len();
    measurement.samples.sort_by(f64::total_cmp);
    let mean = measurement.samples.iter().sum::<f64>() / frames.max(1) as f64;
    let percentile = |p: f64| {
        measurement
            .samples
            .get(((frames as f64 * p).ceil() as usize).saturating_sub(1))
            .copied()
            .unwrap_or(0.0)
    };
    info!("AC3D_MEASURE label={} frames={} mean_ms={:.3} p50_ms={:.3} p95_ms={:.3} fps={:.2} resolution={}x{} count={} mode={:?} style={} palette={} shape={} color_mode={} visual_size={} length={} width={}",
        measurement.label, frames, mean, percentile(0.5), percentile(0.95), if mean > 0.0 { 1000.0 / mean } else { 0.0 },
        resolution.0, resolution.1, state.vehicle_count, state.mode, appearance.style.name(), appearance.palette.name(), appearance.vehicles.shape.name(), appearance.vehicles.color_mode.name(), appearance.vehicles.visual_size, appearance.vehicles.length, appearance.vehicles.width);
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn predator_commands_validate_masks_capacity_paths_and_changed_only_setters() {
        let commands=parse("0 predator add 1 -2 3 20\n0 set predator_strength 8\n0 set predator_path orbit\n0 set predator_groups 5\n0 set predator_path_radius 50\n0 set predator_path_speed -30\n0 predator_center 4 5 6\n0 set predators_visible off\n1 predator clear").unwrap();
        assert_eq!(
            commands[0].command,
            Command::PredatorAdd(Vec3::new(1.0, -2.0, 3.0), 20.0)
        );
        assert_eq!(commands.last().unwrap().command, Command::PredatorClear);
        for command in [
            "predator add NaN 0 0 10",
            "predator add 0 0 0 0",
            "predator add 0 0 0 201",
            "set predator_path chase",
            "set predator_strength 21",
            "set predator_groups 256",
            "set predator_groups -1",
            "set predator_path_speed inf",
            "predator_center 0 1",
        ] {
            assert!(parse(&format!("0 {command}")).is_err(), "{command}");
        }
        let four = "0 predator add 0 0 0 20\n".repeat(4);
        assert!(parse(&four).is_ok());
        assert!(parse(&format!("{four}0 predator add 0 0 0 20")).is_err());
        assert!(parse(&format!("{four}0 predator clear\n{four}")).is_ok());
        let mut world = World::new();
        world.init_resource::<crate::swarm::SwarmScene>();
        world
            .resource_mut::<crate::swarm::SwarmScene>()
            .predators
            .add(Vec3::ZERO, 20.0);
        apply_setting(&mut world, Setting::PredatorGroups(5));
        assert_eq!(
            world
                .resource::<crate::swarm::SwarmScene>()
                .predators
                .objects[0]
                .mask,
            5
        );
        world.clear_trackers();
        apply_setting(&mut world, Setting::PredatorGroups(5));
        assert!(!world
            .get_resource_ref::<crate::swarm::SwarmScene>()
            .unwrap()
            .is_changed());
    }

    #[test]
    fn formation_commands_validate_ranges_presets_and_unchanged_writes() {
        use crate::formation::FormationMode;
        let commands=parse("0 set formation sphere\n0 formation_center 1 -2 3\n0 set formation_extent 80\n0 set formation_arrival_radius 12\n0 set formation_strength 3\n0 set formation_morph 0.5\n0 set formation_morph_speed 0.2\n1 set formation helix\n2 set formation off").unwrap();
        assert_eq!(
            commands[0].command,
            Command::Set(Setting::FormationMode(FormationMode::Sphere))
        );
        assert_eq!(
            commands[1].command,
            Command::Set(Setting::FormationCenter(Vec3::new(1.0, -2.0, 3.0)))
        );
        for value in [
            "set formation torus",
            "formation_center NaN 0 0",
            "formation_center 1 2",
            "set formation_extent 0",
            "set formation_strength 21",
            "set formation_morph -0.1",
            "set formation_morph 1.1",
            "set formation_morph_speed inf",
            "set formation_arrival_radius 0",
        ] {
            assert!(parse(&format!("0 {value}")).is_err(), "{value}");
        }
        let mut world = World::new();
        world.init_resource::<crate::swarm::SwarmScene>();
        apply_setting(&mut world, Setting::FormationMode(FormationMode::Helix));
        assert_eq!(
            world.resource::<crate::swarm::SwarmScene>().formation.morph,
            1.0
        );
        world.clear_trackers();
        apply_setting(&mut world, Setting::FormationMode(FormationMode::Helix));
        assert!(!world
            .get_resource_ref::<crate::swarm::SwarmScene>()
            .unwrap()
            .is_changed());
        apply_setting(&mut world, Setting::FormationMorph(0.5));
        assert_eq!(
            world.resource::<crate::swarm::SwarmScene>().formation.morph,
            0.5
        );
    }

    #[test]
    fn obstacle_commands_validate_bounds_capacity_and_clearance_edits() {
        let commands = parse("0 set obstacle_strength 8\n0 set obstacle_clearance 12\n0 obstacle add 1 -2 3 10\n1 set obstacles_visible off\n2 obstacle clear").unwrap();
        assert_eq!(
            commands[2].command,
            Command::ObstacleAdd(Vec3::new(1.0, -2.0, 3.0), 10.0)
        );
        assert_eq!(commands[4].command, Command::ObstacleClear);
        for command in [
            "obstacle add NaN 0 0 10",
            "obstacle add 0 0 0 0",
            "obstacle add 0 0 0 201",
            "obstacle add 0 0 0",
            "set obstacle_strength -1",
            "set obstacle_strength inf",
            "set obstacle_clearance 0",
            "set obstacles_visible maybe",
        ] {
            assert!(parse(&format!("0 {command}")).is_err(), "{command}");
        }
        let eight = "0 obstacle add 0 0 0 10\n".repeat(8);
        assert!(parse(&eight).is_ok());
        assert!(parse(&format!("{eight}0 obstacle add 0 0 0 10")).is_err());
        assert!(parse(&format!("{eight}0 obstacle clear\n{eight}")).is_ok());
        let mut world = World::new();
        world.init_resource::<crate::swarm::SwarmScene>();
        world
            .resource_mut::<crate::swarm::SwarmScene>()
            .add_obstacle(Vec3::ZERO, 10.0);
        apply_setting(&mut world, Setting::ObstacleClearance(12.0));
        apply_setting(&mut world, Setting::ObstacleStrength(8.0));
        let scene = world.resource::<crate::swarm::SwarmScene>();
        assert_eq!(scene.obstacle_clearance, 12.0);
        assert_eq!(scene.obstacles[0].clearance, 12.0);
        assert_eq!(scene.obstacle_strength, 8.0);
        world.clear_trackers();
        apply_setting(&mut world, Setting::ObstacleClearance(12.0));
        apply_setting(&mut world, Setting::ObstacleStrength(8.0));
        assert!(!world
            .get_resource_ref::<crate::swarm::SwarmScene>()
            .unwrap()
            .is_changed());
    }

    #[test]
    fn group_commands_validate_and_keep_disabled_defaults() {
        let mut world = World::new();
        world.init_resource::<GlobalState>();
        assert_eq!(world.resource::<GlobalState>().groups.count, 0);
        let commands = parse("0 set groups 8\n0 group_target 0 -100 2 3\n0 group_target 7 off\n0 group_color 0 #1aBcD2 # comment\n0 group_color 7 ff7733\n0 set color_mode group\n1 set groups off").unwrap();
        world.init_resource::<Appearance>();
        for command in commands {
            if let Command::Set(setting) = command.command {
                apply_setting(&mut world, setting);
            }
        }
        let groups = &world.resource::<GlobalState>().groups;
        assert_eq!(groups.count, 0);
        assert_eq!(groups.table[0].target, Some(Vec3::new(-100.0, 2.0, 3.0)));
        assert_eq!(groups.table[7].target, None);
        assert_eq!(groups.table[0].color, [0x1a, 0xbc, 0xd2]);
        assert_eq!(groups.table[7].color, [0xff, 0x77, 0x33]);
        assert_eq!(
            world.resource::<Appearance>().vehicles.color_mode,
            ColorMode::Group
        );
        for bad in [
            "set groups 1",
            "set groups 9",
            "set groups 0",
            "group_target 8 0 0 0",
            "group_target -1 off",
            "group_target 0 NaN 0 0",
            "group_target 0 1 2",
            "group_color 8 ff7733",
            "group_color 0 #xyzxyz",
            "group_color 0 ff773",
        ] {
            assert!(parse(&format!("0 {bad}")).is_err(), "{bad}");
        }
    }

    #[test]
    fn attractor_commands_validate_and_apply_without_changing_other_settings() {
        let commands = parse("0 set attractor_mode attract\n0 set attractor_strength 8\n0 set attractor_radius 70\n0 attractor 1 -2 3\n1 set attractor_mode repel\n2 attractor mouse\n3 set attractor_mode off").unwrap();
        assert_eq!(
            commands[3].command,
            Command::Attractor(Some(Vec3::new(1.0, -2.0, 3.0)))
        );
        assert_eq!(commands[5].command, Command::Attractor(None));
        let mut world = World::new();
        world.init_resource::<GlobalState>();
        let before = world.resource::<GlobalState>().clone();
        for command in commands {
            if let Command::Set(setting) = command.command {
                apply_setting(&mut world, setting);
            }
        }
        let mut after = world.resource::<GlobalState>().clone();
        assert_eq!(after.attractor.mode, AttractorMode::Off);
        assert_eq!(after.attractor.strength, 8.0);
        assert_eq!(after.attractor.radius, 70.0);
        after.attractor = before.attractor;
        assert_eq!(after, before);
        for bad in [
            "set attractor_mode spin",
            "set attractor_radius 0",
            "set attractor_radius 201",
            "set attractor_strength -1",
            "set attractor_strength NaN",
            "set attractor_strength 21",
            "attractor 1 inf 3",
            "attractor 1 2",
            "attractor on",
        ] {
            assert!(parse(&format!("0 {bad}")).is_err(), "{bad}");
        }
    }

    #[test]
    fn glow_dof_and_ui_commands_validate_and_keep_simulation_unchanged() {
        let commands = parse(
            "0 set style glow\n0 set glow 20\n0 set streak 3\n0 set dof off\n0 ui off\n1 ui on",
        )
        .unwrap();
        assert_eq!(
            commands[0].command,
            Command::Set(Setting::Style(SceneStyle::Glow))
        );
        assert_eq!(commands[4].command, Command::Ui(false));
        assert_eq!(commands[5].command, Command::Ui(true));
        for line in [
            "0 set glow -1",
            "0 set glow 21",
            "0 set glow NaN",
            "0 set streak 3.1",
            "0 set streak -0.1",
            "0 set dof maybe",
            "0 ui maybe",
            "0 set style neon",
        ] {
            assert!(parse(line).is_err(), "{line}");
        }
        let mut world = World::new();
        world.init_resource::<Presentation>();
        world.init_resource::<Appearance>();
        world.init_resource::<GlobalState>();
        let before = world.resource::<GlobalState>().clone();
        for setting in [
            Setting::Style(SceneStyle::Glow),
            Setting::Glow(8.0),
            Setting::Streak(1.0),
            Setting::Dof(false),
        ] {
            apply_setting(&mut world, setting);
        }
        assert_eq!(world.resource::<GlobalState>(), &before);
        assert_eq!(world.resource::<Appearance>().glow, 8.0);
        assert!(!world.resource::<Presentation>().follow.dof);
    }

    #[test]
    fn boid_follow_commands_validate_modes_and_spawn_ids() {
        use crate::follow::FollowMode;
        let commands = parse("0 boidcam chase\n0 boidcam id 4294967295\n1 boidcam next\n2 boidcam first\n3 boidcam off\n4 exit").unwrap();
        assert_eq!(commands[0].command, Command::BoidFollow(FollowMode::Chase));
        assert_eq!(commands[1].command, Command::FollowId(u32::MAX));
        assert_eq!(commands[2].command, Command::FollowNext);
        assert_eq!(
            commands[3].command,
            Command::BoidFollow(FollowMode::FirstPerson)
        );
        assert_eq!(commands[4].command, Command::BoidFollow(FollowMode::Off));
        for line in [
            "0 boidcam firstperson",
            "0 boidcam id -1",
            "0 boidcam id 4294967296",
            "0 boidcam id 1.5",
            "0 boidcam id",
            "0 boidcam next 3",
            "0 follow chase",
        ] {
            assert!(parse(line).is_err(), "{line}");
        }
    }

    #[test]
    fn presentation_commands_validate_and_keep_equal_time_order() {
        let commands = parse("0 pause\n0 set speed 0.25\n0 set orbit on\n0 set orbit_speed -15\n0 preset closeup\n1 resume\n2 exit").unwrap();
        assert_eq!(commands[0].command, Command::Pause(true));
        assert_eq!(commands[4].command, Command::Preset(CameraPreset::CloseUp));
        assert_eq!(commands[5].command, Command::Pause(false));
        for bad in [
            "0 set speed 0",
            "0 set speed 4.1",
            "0 set speed NaN",
            "0 set orbit maybe",
            "0 set orbit_speed 46",
            "0 preset unknown",
            "0 pause on",
        ] {
            assert!(parse(bad).is_err(), "{bad}");
        }
        for name in ["front", "side", "top", "overview", "closeup"] {
            assert!(parse(&format!("0 preset {name}")).is_ok());
        }
    }

    #[test]
    fn measurement_controls_validate_and_preserve_default_render_and_simulation() {
        assert!(crate::instancing::FlockDraw::default().0);
        let mut world = World::new();
        world.init_resource::<GlobalState>();
        world.init_resource::<crate::instancing::FlockDraw>();
        let camera = world.spawn((Camera3d::default(), Msaa::Sample4)).id();
        let before = world.resource::<GlobalState>().clone();
        for command in parse("0 set flock_draw off\n0 set msaa off").unwrap() {
            if let Command::Set(setting) = command.command {
                apply_setting(&mut world, setting);
            }
        }
        assert!(!world.resource::<crate::instancing::FlockDraw>().0);
        assert_eq!(*world.get::<Msaa>(camera).unwrap(), Msaa::Off);
        for command in parse("0 set flock_draw on\n0 set msaa 4").unwrap() {
            if let Command::Set(setting) = command.command {
                apply_setting(&mut world, setting);
            }
        }
        assert!(world.resource::<crate::instancing::FlockDraw>().0);
        assert_eq!(*world.get::<Msaa>(camera).unwrap(), Msaa::Sample4);
        assert_eq!(world.resource::<GlobalState>(), &before);
        assert!(parse("0 set flock_draw maybe").is_err());
        assert!(parse("0 set msaa 8").is_err());
    }

    #[test]
    fn parses_all_commands_and_preserves_equal_time_order() {
        let script = "# example\n0 set count 200\n0 set mode gpu\n0 set style classic\n0 set palette dusk # comment\n0 set auto_frame off\n0 set vehicle_mass 70\n0 set gpu_max_neighbors 16\n0 set octree_size 50\n1 follow on\n1 target 1 -2 3\n1 camera 90 20 600\n2 frame\n2 present immediate\n3 screenshot /tmp/shot.png\n4 measure begin sample\n5 measure end\n6 present vsync\n7 follow off\n8 exit\n";
        let parsed = parse(script).unwrap();
        assert_eq!(parsed.len(), 19);
        assert_eq!(parsed[0].command, Command::Set(Setting::Count(200)));
        assert_eq!(
            parsed[9].command,
            Command::Target(Vec3::new(1.0, -2.0, 3.0))
        );
        assert_eq!(parsed.last().unwrap().command, Command::Exit);
        assert_eq!(
            parse("0 set palette aurora").unwrap()[0].command,
            Command::Set(Setting::Palette(HeadingPalette::Aurora))
        );
        assert!(parse("").unwrap().is_empty());
    }

    #[test]
    fn rejects_invalid_commands_values_and_measurement_pairs_at_startup() {
        for bad in [
            "0 nonsense",
            "0",
            "-1 exit",
            "NaN exit",
            "inf exit",
            "0 set count 1000001",
            "0 set count 1.5",
            "0 set mode typo",
            "0 set style typo",
            "0 set palette typo",
            "0 set unknown 1",
            "0 set auto_frame maybe",
            "0 set vehicle_mass NaN",
            "0 set vehicle_max_speed 0",
            "0 set gpu_max_neighbors 513",
            "0 set octree_size 1",
            "0 follow true",
            "0 target 1 inf 3",
            "0 camera 0 90 20",
            "0 camera 0 0 0",
            "0 screenshot",
            "0 present other",
            "0 measure end",
            "0 exit extra",
        ] {
            let error = parse(&format!("# comment\n\n{bad}")).unwrap_err();
            assert!(error.starts_with("line 3:"), "{bad}: {error}");
        }
        assert!(parse("2 frame\n1 exit").unwrap_err().starts_with("line 2:"));
        assert!(parse("0 set mode naive\n1 set count 50001")
            .unwrap_err()
            .starts_with("line 2:"));
        assert!(parse("0 measure begin sample").is_err());
        assert!(parse("0 measure begin a\n1 measure begin b\n2 measure end").is_err());
        assert!(parse("0 measure begin a\n1 exit\n2 measure end").is_err());
    }

    #[test]
    fn appearance_commands_validate_values_and_allow_hex_colors_and_comments() {
        let script = "0 set shape arrow\n0 set shape glider\n0 set visual_size 0.5\n0 set length 3\n0 set width 0.25\n0 set color_mode altitude\n0 set color_mode speed\n0 set color_mode solid\n0 set solid_color #aBcD12 # comment\n0 set solid_color abcD12\n0 set solid_color 1 0.5 0 # comment";
        let commands = parse(script).unwrap();
        assert_eq!(
            commands[8].command,
            Command::Set(Setting::SolidColor([0xab, 0xcd, 0x12]))
        );
        assert_eq!(commands[9].command, commands[8].command);
        assert_eq!(
            commands[10].command,
            Command::Set(Setting::SolidColor([255, 128, 0]))
        );
        assert_eq!(script_line("0 set count 200# comment"), "0 set count 200");
        for bad in [
            "shape sphere",
            "color_mode random",
            "visual_size 0",
            "visual_size inf",
            "length 4",
            "width -1",
            "solid_color #xyz123",
            "solid_color 12345",
            "solid_color 1 2 0",
            "solid_color 0 NaN 0",
        ] {
            assert!(parse(&format!("# header\n0 set {bad}"))
                .unwrap_err()
                .starts_with("line 2:"));
        }
    }

    #[test]
    fn scripted_appearance_changes_preserve_simulation_and_selection() {
        let mut world = World::new();
        world.init_resource::<GlobalState>();
        world.init_resource::<Appearance>();
        let target = world
            .spawn((
                Target,
                TargetSelection(true),
                Transform::from_xyz(1.0, 2.0, 3.0),
            ))
            .id();
        for command in
            parse("0 set style classic\n0 set palette dusk\n0 set auto_frame off").unwrap()
        {
            if let Command::Set(setting) = command.command {
                apply_setting(&mut world, setting);
            }
        }
        assert_eq!(world.resource::<Appearance>().style, SceneStyle::Classic);
        assert_eq!(world.resource::<Appearance>().palette, HeadingPalette::Dusk);
        assert!(!world.resource::<Appearance>().auto_frame);
        assert_eq!(world.resource::<GlobalState>().vehicle_count, 20_000);
        assert_eq!(world.resource::<GlobalState>().vehicle_size, 1.0);
        assert!(world.get::<TargetSelection>(target).unwrap().0);
        assert_eq!(
            world.get::<Transform>(target).unwrap().translation,
            Vec3::new(1.0, 2.0, 3.0)
        );
    }

    #[test]
    fn model_commands_validate_axes_scale_and_color_mode_before_startup() {
        let commands = parse("0 set model_forward -x\n0 set model_up +z\n0 set model_scale 2\n0 set model assets/models/glider.glb\n1 set color_mode model_colors").unwrap();
        assert_eq!(
            commands[0].command,
            Command::Set(Setting::ModelForward(ModelAxis::NegX))
        );
        assert_eq!(
            commands[1].command,
            Command::Set(Setting::ModelUp(ModelAxis::Z))
        );
        assert_eq!(
            commands[3].command,
            Command::Set(Setting::Model("assets/models/glider.glb".into()))
        );
        assert_eq!(
            commands[4].command,
            Command::Set(Setting::ColorMode(ColorMode::ModelColors))
        );
        for bad in [
            "model_forward north",
            "model_up z",
            "model_scale NaN",
            "model_scale 0",
            "model_scale 6",
        ] {
            assert!(parse(&format!("0 set {bad}"))
                .unwrap_err()
                .starts_with("line 1:"));
        }
        assert!(parse("0 set model_forward y\n0 set model_up -y")
            .unwrap_err()
            .contains("line 2: model_up must be perpendicular"));
    }

    #[test]
    fn animation_commands_accept_clip_indices_names_and_bounded_speed() {
        let commands =
            parse("0 set animation_clip Flap\n0 set animation_clip 1\n0 set animation_speed 0.5")
                .unwrap();
        assert_eq!(
            commands[0].command,
            Command::Set(Setting::AnimationClip("Flap".into()))
        );
        assert_eq!(
            commands[1].command,
            Command::Set(Setting::AnimationClip("1".into()))
        );
        assert_eq!(
            commands[2].command,
            Command::Set(Setting::AnimationSpeed(0.5))
        );
        for speed in ["0", "4.1", "NaN", "inf"] {
            assert!(parse(&format!("0 set animation_speed {speed}")).is_err());
        }
    }
}
