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
    scene::{Appearance, HeadingPalette, SceneStyle},
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
    Target(Vec3),
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
    Mode(SimulationMode),
    Style(SceneStyle),
    Palette(HeadingPalette),
    AutoFrame(bool),
    FollowSamples(u32),
    OctreeSize(usize),
    Number(NumberSetting, f32),
    Shape(VehicleShape),
    Model(PathBuf),
    ModelForward(ModelAxis),
    ModelUp(ModelAxis),
    ModelScale(f32),
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
        ("count" | "vehicle_count", _) => Setting::Count(integer(value, 0, mode.max_vehicles())?),
        ("mode", "gpu") => Setting::Mode(SimulationMode::Gpu),
        ("mode", "octree") => Setting::Mode(SimulationMode::Octree),
        ("mode", "naive") => Setting::Mode(SimulationMode::Naive),
        ("style", "studio") => Setting::Style(SceneStyle::Studio),
        ("style", "classic") => Setting::Style(SceneStyle::Classic),
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
        ("model_scale", _) => Setting::ModelScale(number(value, 0.1, 5.0)?),
        ("visual_size", _) => Setting::VisualSize(number(value, 0.1, 5.0)?),
        ("length", _) => Setting::Length(number(value, 0.25, 3.0)?),
        ("width", _) => Setting::Width(number(value, 0.25, 3.0)?),
        ("color_mode", "heading") => Setting::ColorMode(ColorMode::Heading),
        ("color_mode", "altitude") => Setting::ColorMode(ColorMode::Altitude),
        ("color_mode", "speed") => Setting::ColorMode(ColorMode::Speed),
        ("color_mode", "model" | "model_colors") => Setting::ColorMode(ColorMode::ModelColors),
        ("color_mode", "solid") => Setting::ColorMode(ColorMode::Solid),
        ("solid_color", _) => Setting::SolidColor(hex_color(value)?),
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
            && prefix[1..] == ["set", "solid_color"]
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
                ["follow", value @ ("on" | "off")] => Command::Follow(switch(value)?),
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
                Command::Follow(follow) => {
                    for mut selection in world
                        .query_filtered::<&mut TargetSelection, With<Target>>()
                        .iter_mut(world)
                    {
                        selection.0 = follow;
                    }
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
        Setting::Style(value) => world.resource_mut::<Appearance>().style = value,
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
    let window = world
        .query_filtered::<&Window, With<PrimaryWindow>>()
        .single(world)
        .expect("primary window");
    let resolution = (window.physical_width(), window.physical_height());
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
}
