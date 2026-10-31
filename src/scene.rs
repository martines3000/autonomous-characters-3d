//! Scene appearance and camera-aware world presentation; never changes simulation settings.
use std::f32::consts::FRAC_PI_2;

use bevy::{
    asset::RenderAssetUsages,
    camera::visibility::NoFrustumCulling,
    camera_controller::pan_orbit_camera::prelude::*,
    light::{NotShadowCaster, NotShadowReceiver},
    mesh::VertexAttributeValues,
    prelude::*,
    render::{
        render_resource::{Face, PrimitiveTopology},
        view::NoIndirectDrawing,
    },
    text::FontSize,
    transform::TransformSystems,
};

use crate::{vehicle, FpsText, GlobalState};

const SKY_ZENITH: Color = Color::srgb(0.03, 0.05, 0.12);
const SKY_HORIZON: Color = Color::srgb(0.10, 0.16, 0.26);
const FLOOR_COLOR: Color = Color::srgb(0.035, 0.060, 0.10);
const FLOOR_GRID_COLOR: Color = Color::srgb(0.055, 0.090, 0.14);

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum SceneStyle {
    #[default]
    Studio,
    Classic,
}

impl SceneStyle {
    pub const ALL: [Self; 2] = [Self::Studio, Self::Classic];
    pub fn name(self) -> &'static str {
        match self {
            Self::Studio => "Studio",
            Self::Classic => "Classic",
        }
    }
    pub fn settings(self) -> SceneSettings {
        match self {
            Self::Classic => SceneSettings {
                background: Color::srgb(0.035, 0.045, 0.07),
                wall: Color::srgba(0.55, 0.65, 0.85, 0.3),
                grid: Color::srgba(0.55, 0.65, 0.85, 0.12),
                ambient: 400.0,
                key: 6000.0,
                key_color: Color::WHITE,
                fill: 0.0,
                roughness: 0.6,
                metallic: 0.0,
                rim: 0.0,
                emission: 0.0,
            },
            Self::Studio => SceneSettings {
                background: SKY_HORIZON,
                wall: Color::srgba(0.32, 0.48, 0.64, 0.10),
                grid: FLOOR_GRID_COLOR,
                ambient: 650.0,
                key: 10000.0,
                key_color: Color::srgb(1.0, 0.92, 0.80),
                fill: 3500.0,
                roughness: 0.38,
                metallic: 0.18,
                rim: 0.16,
                emission: 0.15,
            },
        }
    }
}

#[derive(Clone, Copy)]
pub struct SceneSettings {
    pub background: Color,
    pub wall: Color,
    pub grid: Color,
    pub ambient: f32,
    pub key: f32,
    pub key_color: Color,
    pub fill: f32,
    pub roughness: f32,
    pub metallic: f32,
    pub rim: f32,
    pub emission: f32,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum HeadingPalette {
    Classic,
    Ocean,
    Dusk,
    #[default]
    Aurora,
}

impl HeadingPalette {
    pub const ALL: [Self; 4] = [Self::Classic, Self::Ocean, Self::Dusk, Self::Aurora];
    pub fn name(self) -> &'static str {
        match self {
            Self::Classic => "Classic",
            Self::Ocean => "Ocean",
            Self::Dusk => "Dusk",
            Self::Aurora => "Aurora",
        }
    }
    /// Linear colors around the heading wheel; Classic uses its original HSV function instead.
    /// Six distinct stops plus the repeated first color make the cyclic endpoint explicit.
    pub fn colors(self) -> [Vec4; 7] {
        let srgb = match self {
            Self::Classic => [[1.0, 1.0, 1.0]; 6],
            Self::Ocean => [
                [0.055, 0.24, 0.80],
                [0.015, 0.88, 1.0],
                [0.12, 0.96, 0.56],
                [1.0, 0.82, 0.40],
                [1.0, 0.25, 0.14],
                [0.58, 0.16, 0.98],
            ],
            Self::Dusk => [
                [0.24, 0.095, 0.66],
                [0.96, 0.06, 0.69],
                [1.0, 0.32, 0.055],
                [1.0, 0.86, 0.10],
                [1.0, 0.24, 0.40],
                [0.63, 0.13, 0.91],
            ],
            Self::Aurora => [
                [0.0, 0.70, 0.67],
                [0.05, 0.96, 0.25],
                [0.78, 1.0, 0.10],
                [0.46, 0.18, 0.94],
                [1.0, 0.06, 0.72],
                [0.08, 0.34, 1.0],
            ],
        };
        std::array::from_fn(|i| {
            let [r, g, b] = srgb[i % 6];
            Color::srgb(r, g, b).to_linear().to_vec4()
        })
    }
}

#[derive(Resource)]
pub struct Appearance {
    pub style: SceneStyle,
    pub palette: HeadingPalette,
    pub vehicles: crate::vehicle_appearance::VehicleAppearance,
    pub auto_frame: bool,
    pub frame_requested: bool,
    pub count_editing: bool,
}

impl Default for Appearance {
    fn default() -> Self {
        Self {
            style: SceneStyle::default(),
            palette: HeadingPalette::default(),
            vehicles: default(),
            auto_frame: true,
            frame_requested: false,
            count_editing: false,
        }
    }
}

pub struct ScenePlugin;
impl Plugin for ScenePlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<Appearance>()
            .add_systems(Startup, (setup_scene, setup_studio).chain())
            .add_systems(
                PostUpdate,
                (frame_world, draw_world, update_studio)
                    .chain()
                    .before(TransformSystems::Propagate),
            );
    }
}

#[derive(Component)]
struct KeyLight;
#[derive(Component)]
struct FillLight;

pub fn setup_scene(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    state: Res<GlobalState>,
    appearance: Res<Appearance>,
) {
    let look = appearance.style.settings();
    commands.insert_resource(GlobalAmbientLight {
        brightness: look.ambient,
        ..default()
    });
    commands.spawn((
        DirectionalLight {
            illuminance: look.key,
            color: look.key_color,
            ..default()
        },
        Transform::from_xyz(1.0, 2.0, 0.6).looking_at(Vec3::ZERO, Vec3::Y),
        KeyLight,
    ));
    commands.spawn((
        DirectionalLight {
            illuminance: look.fill,
            color: Color::srgb(0.55, 0.75, 1.0),
            ..default()
        },
        Transform::from_xyz(-1.0, 0.3, -0.8).looking_at(Vec3::ZERO, Vec3::Y),
        FillLight,
    ));
    let font = TextFont {
        font: asset_server.load("fonts/FiraSans-Bold.ttf").into(),
        font_size: FontSize::Px(60.0),
        ..default()
    };
    commands
        .spawn((
            Text::new("FPS: "),
            font.clone(),
            TextColor(Color::WHITE),
            Node {
                position_type: PositionType::Absolute,
                top: px(5),
                right: px(5),
                ..default()
            },
        ))
        .with_child((
            TextSpan::new(""),
            font,
            TextColor(Color::srgb(1.0, 0.84, 0.0)),
            FpsText,
        ));
    // Far enough back to see the whole default world: flocks spread to the walls within seconds.
    let rotation = Transform::from_xyz(0.0, 140.0, 450.0)
        .looking_at(Vec3::ZERO, Vec3::Y)
        .rotation;
    let distance = frame_distance(
        vehicle::walls(state.vehicle_count),
        rotation,
        Vec3::ZERO,
        1280.0 / 720.0,
        PerspectiveProjection::default().fov,
    );
    let eye = rotation * Vec3::Z * distance;
    commands.spawn((
        Camera3d::default(),
        Projection::Perspective(PerspectiveProjection {
            far: far_wall_distance(eye, vehicle::walls(state.vehicle_count)) * 2.0,
            ..default()
        }),
        Msaa::Sample4,
        // The instanced flock draws directly, not through indirect draw buffers.
        NoIndirectDrawing,
        DistanceFog {
            color: look.background,
            ..default()
        },
        Transform::from_translation(eye).with_rotation(rotation),
        PanOrbitCamera::default().with_initial_anchor_depth(distance as f64),
        bevy::picking::mesh_picking::MeshPickingCamera,
    ));
}

#[derive(Component)]
enum StudioSurface {
    Sky,
    Ground,
}

fn sky_color(height: f32) -> [f32; 4] {
    let horizon = SKY_HORIZON.to_linear().to_vec4();
    let (end, amount) = if height >= 0.0 {
        (SKY_ZENITH, (height / 0.8).clamp(0.0, 1.0))
    } else {
        (FLOOR_COLOR, (-height / 0.65).clamp(0.0, 1.0))
    };
    let smooth = amount * amount * (3.0 - 2.0 * amount);
    horizon.lerp(end.to_linear().to_vec4(), smooth).to_array()
}

fn sky_mesh() -> Mesh {
    let mut mesh = Sphere::new(1.0).mesh().uv(32, 24);
    if let Some(VertexAttributeValues::Float32x3(positions)) =
        mesh.attribute(Mesh::ATTRIBUTE_POSITION)
    {
        let colors: Vec<_> = positions
            .iter()
            .map(|position| sky_color(position[1]))
            .collect();
        mesh.insert_attribute(Mesh::ATTRIBUTE_COLOR, colors);
    }
    mesh
}

fn ground_mesh() -> Mesh {
    let mut positions = Vec::new();
    let mut colors = Vec::new();
    let mut quad = |x0, z0, x1, z1, y, color: Color| {
        positions.extend([
            [x0, y, z0],
            [x0, y, z1],
            [x1, y, z1],
            [x0, y, z0],
            [x1, y, z1],
            [x1, y, z0],
        ]);
        colors.extend([color.to_linear().to_f32_array(); 6]);
    };
    quad(-1.0, -1.0, 1.0, 1.0, 0.0, FLOOR_COLOR);
    // A small fixed grid shares the ground's lighting and fog, with no coplanar surfaces.
    for i in 1..8 {
        let x = -1.0 + i as f32 / 4.0;
        quad(x - 0.0015, -1.0, x + 0.0015, 1.0, 0.02, FLOOR_GRID_COLOR);
    }
    for i in 1..4 {
        let z = -1.0 + i as f32 / 2.0;
        quad(-1.0, z - 0.003, 1.0, z + 0.003, 0.02, FLOOR_GRID_COLOR);
    }
    let normals = vec![[0.0, 1.0, 0.0]; positions.len()];
    Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::default(),
    )
    .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, positions)
    .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, normals)
    .with_inserted_attribute(Mesh::ATTRIBUTE_COLOR, colors)
}

fn setup_studio(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn((
        Mesh3d(meshes.add(sky_mesh())),
        MeshMaterial3d(materials.add(StandardMaterial {
            unlit: true,
            fog_enabled: false,
            cull_mode: Some(Face::Front),
            ..default()
        })),
        StudioSurface::Sky,
        NoFrustumCulling,
        NotShadowCaster,
        NotShadowReceiver,
        Pickable::IGNORE,
    ));
    commands.spawn((
        Mesh3d(meshes.add(ground_mesh())),
        MeshMaterial3d(materials.add(StandardMaterial {
            perceptual_roughness: 1.0,
            metallic: 0.0,
            reflectance: 0.0,
            fog_enabled: true,
            ..default()
        })),
        StudioSurface::Ground,
        NotShadowCaster,
        NotShadowReceiver,
        Pickable::IGNORE,
    ));
}

/// Margin also covers the UV sphere's inscribed polygon faces (not only its vertices).
fn sky_radius(eye: Vec3, walls: Vec3) -> f32 {
    far_wall_distance(eye, walls) * 1.2
}

fn scene_fog_range(style: SceneStyle, eye: Vec3, walls: Vec3) -> (f32, f32) {
    if style == SceneStyle::Classic {
        return fog_range(eye, walls);
    }
    let far = far_wall_distance(eye, walls);
    // At the farthest wall this leaves at least 70% transmission; the world stays readable.
    (far * 0.6, (walls.x * 5.0).max(far * 2.0))
}

fn update_studio(
    state: Res<GlobalState>,
    appearance: Res<Appearance>,
    cameras: Query<&Transform, (With<Camera3d>, Without<StudioSurface>)>,
    mut surfaces: Query<(&StudioSurface, &mut Transform, &mut Visibility)>,
) {
    let Ok(camera) = cameras.single() else {
        return;
    };
    let walls = vehicle::walls(state.vehicle_count);
    for (surface, mut transform, mut visibility) in &mut surfaces {
        *visibility = if appearance.style == SceneStyle::Studio {
            Visibility::Inherited
        } else {
            Visibility::Hidden
        };
        match surface {
            StudioSurface::Sky => {
                transform.translation = camera.translation;
                transform.scale = Vec3::splat(sky_radius(camera.translation, walls));
            }
            StudioSurface::Ground => {
                transform.translation = Vec3::new(0.0, -walls.y, 0.0);
                transform.scale = Vec3::new(walls.x, 1.0, walls.z);
            }
        }
    }
}

fn corners(walls: Vec3) -> impl Iterator<Item = Vec3> {
    [-1.0, 1.0].into_iter().flat_map(move |x| {
        [-1.0, 1.0].into_iter().flat_map(move |y| {
            [-1.0, 1.0]
                .into_iter()
                .map(move |z| walls * Vec3::new(x, y, z))
        })
    })
}

/// Fits all eight corners, including when a user has panned the orbit anchor off center.
pub fn frame_distance(walls: Vec3, rotation: Quat, anchor: Vec3, aspect: f32, fov: f32) -> f32 {
    let tan_y = (fov / 2.0).tan();
    let tan_x = tan_y * aspect.max(0.01);
    corners(walls)
        .map(|corner| {
            let local = rotation.inverse() * (corner - anchor);
            local.z + (local.x.abs() / tan_x).max(local.y.abs() / tan_y)
        })
        .fold(1.0_f32, f32::max)
        * 1.08
}

pub fn far_wall_distance(eye: Vec3, walls: Vec3) -> f32 {
    corners(walls)
        .map(|corner| eye.distance(corner))
        .fold(1.0, f32::max)
}

pub fn fog_range(eye: Vec3, walls: Vec3) -> (f32, f32) {
    let far = far_wall_distance(eye, walls);
    // Keep the enclosed world clear even after manual zoom or disabling auto-frame.
    (far * 1.02, (walls.x * 5.0).max(far * 1.8))
}

#[derive(Default)]
pub(crate) struct CountFrame {
    count: Option<usize>,
    changed_at: f64,
    pending: bool,
}

pub(crate) fn frame_world(
    state: Res<GlobalState>,
    mut appearance: ResMut<Appearance>,
    time: Res<Time<Real>>,
    mut debounce: Local<CountFrame>,
    mut cameras: Query<(&mut Transform, &mut PanOrbitCamera, &Projection), With<Camera3d>>,
) {
    if debounce.count != Some(state.vehicle_count) {
        debounce.pending = true;
        debounce.count = Some(state.vehicle_count);
        debounce.changed_at = time.elapsed_secs_f64();
    }
    let settled = debounce.pending
        && !appearance.count_editing
        && time.elapsed_secs_f64() - debounce.changed_at >= 0.35;
    let requested = appearance.frame_requested || (appearance.auto_frame && settled);
    if settled {
        debounce.pending = false;
    }
    if !requested {
        return;
    }
    appearance.frame_requested = false;
    for (mut transform, mut orbit, projection) in &mut cameras {
        let Projection::Perspective(projection) = projection else {
            continue;
        };
        let anchor =
            transform.translation + transform.forward() * orbit.last_anchor_depth().abs() as f32;
        let distance = frame_distance(
            vehicle::walls(state.vehicle_count),
            transform.rotation,
            anchor,
            projection.aspect_ratio,
            projection.fov,
        );
        transform.translation = anchor + transform.back() * distance;
        orbit.current_motion = default();
        orbit.last_anchor_depth = -(distance as f64);
    }
}

// Wall outline and a floor grid for orientation; fog follows the world size.
#[allow(clippy::too_many_arguments)]
fn draw_world(
    mut gizmos: Gizmos,
    state: Res<GlobalState>,
    appearance: Res<Appearance>,
    mut clear: ResMut<ClearColor>,
    mut ambient: ResMut<GlobalAmbientLight>,
    mut lights: Query<(&mut DirectionalLight, Has<KeyLight>, Has<FillLight>)>,
    mut cameras: Query<(&Transform, &mut DistanceFog, &mut Projection), With<Camera3d>>,
) {
    let look = appearance.style.settings();
    if appearance.is_changed() {
        clear.0 = look.background;
        ambient.brightness = look.ambient;
        for (mut light, key, fill) in &mut lights {
            if key {
                light.illuminance = look.key;
                light.color = look.key_color;
            }
            if fill {
                light.illuminance = look.fill;
            }
        }
    }
    let walls = vehicle::walls(state.vehicle_count);
    if appearance.style == SceneStyle::Classic {
        gizmos.cube(Transform::from_scale(walls * 2.0), look.wall);
        let cell = walls.z / 2.5;
        gizmos.grid(
            Isometry3d::new(
                Vec3::new(0.0, -walls.y, 0.0),
                Quat::from_rotation_x(FRAC_PI_2),
            ),
            UVec2::new((walls.x * 2.0 / cell) as u32, 5),
            Vec2::splat(cell),
            look.grid,
        );
    } else {
        for corner in corners(walls) {
            for axis in [Vec3::X, Vec3::Y, Vec3::Z] {
                gizmos.line(
                    corner,
                    corner - axis * corner.signum() * walls.min_element() * 0.12,
                    look.wall,
                );
            }
        }
    }
    for (transform, mut fog, mut projection) in &mut cameras {
        let (start, end) = scene_fog_range(appearance.style, transform.translation, walls);
        fog.color = look.background;
        fog.falloff = FogFalloff::Linear { start, end };
        if let Projection::Perspective(projection) = &mut *projection {
            projection.far = end
                .max(sky_radius(transform.translation, walls) * 1.1)
                .max(1000.0);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn classic_preserves_original_scene_constants() {
        let look = SceneStyle::Classic.settings();
        assert_eq!(look.background, Color::srgb(0.035, 0.045, 0.07));
        assert_eq!(look.wall, Color::srgba(0.55, 0.65, 0.85, 0.3));
        assert_eq!(look.grid, Color::srgba(0.55, 0.65, 0.85, 0.12));
        assert_eq!((look.ambient, look.key, look.fill), (400.0, 6000.0, 0.0));
        assert_eq!(look.key_color, Color::WHITE);
        assert_eq!((look.roughness, look.metallic, look.rim), (0.6, 0.0, 0.0));
    }

    #[test]
    fn palettes_repeat_the_endpoint_and_span_hues_and_values() {
        for palette in [
            HeadingPalette::Ocean,
            HeadingPalette::Dusk,
            HeadingPalette::Aurora,
        ] {
            let colors = palette.colors();
            assert_eq!(colors.len(), 7);
            assert_eq!(colors[0], colors[6]);
            let mut channel_min = Vec3::ONE;
            let mut channel_max = Vec3::ZERO;
            let mut luminance = Vec::new();
            for pair in colors.windows(2) {
                assert!(pair[0].distance(pair[1]) > 0.1);
                let rgb = pair[0].truncate();
                channel_min = channel_min.min(rgb);
                channel_max = channel_max.max(rgb);
                luminance.push(rgb.dot(Vec3::new(0.2126, 0.7152, 0.0722)));
            }
            assert!(
                (channel_max - channel_min).min_element() > 0.25,
                "{palette:?}"
            );
            let min = luminance.iter().copied().fold(f32::INFINITY, f32::min);
            let max = luminance.iter().copied().fold(0.0_f32, f32::max);
            assert!(max > min * 3.0, "{palette:?}");
        }
        assert_eq!(sky_color(0.0), SKY_HORIZON.to_linear().to_f32_array());
        assert_eq!(SceneStyle::Studio.settings().background, SKY_HORIZON);
        assert_eq!(SceneStyle::Classic.settings().emission, 0.0);
    }

    #[test]
    fn sky_faces_enclose_every_wall_and_stay_inside_far_plane() {
        let mesh = sky_mesh();
        let VertexAttributeValues::Float32x3(positions) =
            mesh.attribute(Mesh::ATTRIBUTE_POSITION).unwrap()
        else {
            panic!("sky positions");
        };
        let indices: Vec<_> = mesh.indices().unwrap().iter().collect();
        for count in [0, 200, 20_000, 1_000_000] {
            let walls = vehicle::walls(count);
            for eye in [
                Vec3::ZERO,
                Vec3::new(0.0, 140.0, 450.0) * vehicle::world_scale(count),
                walls * 20.0,
            ] {
                let radius = sky_radius(eye, walls);
                let (start, far) = scene_fog_range(SceneStyle::Studio, eye, walls);
                assert!(far > radius);
                assert!((far - far_wall_distance(eye, walls)) / (far - start) > 0.70);
                for triangle in indices.as_chunks::<3>().0 {
                    let [a, b, c] = [triangle[0], triangle[1], triangle[2]]
                        .map(|i| Vec3::from_array(positions[i]));
                    let Some(normal) = (b - a).cross(c - a).try_normalize() else {
                        continue;
                    };
                    for corner in corners(walls) {
                        assert!(normal.dot((corner - eye) / radius) < normal.dot(a) + 1e-5);
                    }
                }
            }
        }
    }

    #[test]
    fn studio_surfaces_follow_camera_and_count_and_hide_for_classic() {
        let mut app = App::new();
        app.init_resource::<GlobalState>()
            .init_resource::<Appearance>()
            .init_resource::<Assets<Mesh>>()
            .init_resource::<Assets<StandardMaterial>>()
            .add_systems(Startup, setup_studio)
            .add_systems(Update, update_studio);
        let camera = app
            .world_mut()
            .spawn((Camera3d::default(), Transform::default()))
            .id();
        for count in [0, 200, 20_000, 1_000_000] {
            let eye = Vec3::new(123.0, 140.0, 450.0) * vehicle::world_scale(count);
            app.world_mut()
                .get_mut::<Transform>(camera)
                .unwrap()
                .translation = eye;
            app.world_mut().resource_mut::<GlobalState>().vehicle_count = count;
            app.update();
            let walls = vehicle::walls(count);
            for (entity, surface, transform, material, visibility, pickable) in app
                .world_mut()
                .query::<(
                    Entity,
                    &StudioSurface,
                    &Transform,
                    &MeshMaterial3d<StandardMaterial>,
                    &Visibility,
                    &Pickable,
                )>()
                .iter(app.world())
            {
                assert_eq!(*visibility, Visibility::Inherited);
                assert!(!pickable.is_hoverable && !pickable.should_block_lower);
                let material = app
                    .world()
                    .resource::<Assets<StandardMaterial>>()
                    .get(&material.0)
                    .unwrap();
                match surface {
                    StudioSurface::Sky => {
                        assert_eq!(transform.translation, eye);
                        assert_eq!(transform.scale, Vec3::splat(sky_radius(eye, walls)));
                        assert!(material.unlit && !material.fog_enabled);
                        assert_eq!(material.cull_mode, Some(Face::Front));
                        assert!(app.world().get::<NoFrustumCulling>(entity).is_some());
                    }
                    StudioSurface::Ground => {
                        assert_eq!(transform.translation, Vec3::new(0.0, -walls.y, 0.0));
                        assert_eq!(transform.scale, Vec3::new(walls.x, 1.0, walls.z));
                        assert!(material.fog_enabled && !material.unlit);
                        assert_eq!(
                            (material.perceptual_roughness, material.metallic),
                            (1.0, 0.0)
                        );
                        assert_eq!(material.reflectance, 0.0);
                    }
                }
            }
        }
        for style in [SceneStyle::Classic, SceneStyle::Studio] {
            app.world_mut().resource_mut::<Appearance>().style = style;
            app.update();
            let expected = if style == SceneStyle::Classic {
                Visibility::Hidden
            } else {
                Visibility::Inherited
            };
            for visibility in app
                .world_mut()
                .query_filtered::<&Visibility, With<StudioSurface>>()
                .iter(app.world())
            {
                assert_eq!(*visibility, expected);
            }
        }
    }

    #[test]
    fn frame_and_fog_cover_all_corners_at_every_count_and_aspect() {
        for count in [0, 200, 2000, 20_000, 1_000_000] {
            for aspect in [0.6, 1280.0 / 720.0, 3.0] {
                for rotation in [
                    Quat::IDENTITY,
                    Quat::from_euler(EulerRot::YXZ, 1.2, -0.5, 0.0),
                ] {
                    let walls = vehicle::walls(count);
                    let anchor = walls * Vec3::new(0.3, -0.2, 0.1);
                    let fov = PerspectiveProjection::default().fov;
                    let distance = frame_distance(walls, rotation, anchor, aspect, fov);
                    let eye = anchor + rotation * Vec3::Z * distance;
                    let (start, visibility) = fog_range(eye, walls);
                    assert!(visibility > far_wall_distance(eye, walls));
                    assert!(start > far_wall_distance(eye, walls));
                    for corner in corners(walls) {
                        let local = rotation.inverse() * (corner - eye);
                        assert!(local.z < 0.0);
                        assert!(local.y.abs() / -local.z < (fov / 2.0).tan());
                        assert!(local.x.abs() / -local.z < (fov / 2.0).tan() * aspect);
                    }
                    // Manual zoom far beyond the old fog range must also leave walls visible.
                    assert!(fog_range(eye * 10.0, walls).0 > far_wall_distance(eye * 10.0, walls));
                }
            }
        }
    }

    #[test]
    fn count_framing_waits_for_settle_and_preserves_orientation_and_anchor() {
        let mut app = App::new();
        app.init_resource::<GlobalState>()
            .init_resource::<Appearance>()
            .init_resource::<Time<Real>>()
            .add_systems(Update, frame_world);
        let transform = Transform::from_xyz(0.0, 140.0, 450.0).looking_at(Vec3::ZERO, Vec3::Y);
        let entity = app
            .world_mut()
            .spawn((
                Camera3d::default(),
                transform,
                Projection::default(),
                PanOrbitCamera::default()
                    .with_initial_anchor_depth(transform.translation.length() as f64),
            ))
            .id();
        app.update();
        app.world_mut().resource_mut::<GlobalState>().vehicle_count = 200;
        app.update();
        assert_eq!(*app.world().get::<Transform>(entity).unwrap(), transform);
        app.world_mut().resource_mut::<Appearance>().count_editing = true;
        app.world_mut()
            .resource_mut::<Time<Real>>()
            .advance_by(std::time::Duration::from_millis(400));
        app.update();
        assert_eq!(*app.world().get::<Transform>(entity).unwrap(), transform);
        app.world_mut().resource_mut::<Appearance>().count_editing = false;
        app.update();
        let framed = *app.world().get::<Transform>(entity).unwrap();
        assert_eq!(framed.rotation, transform.rotation);
        assert_ne!(framed.translation, transform.translation);
        let depth = app
            .world()
            .get::<PanOrbitCamera>(entity)
            .unwrap()
            .last_anchor_depth()
            .abs() as f32;
        assert!((framed.translation + framed.forward() * depth).length() < 0.001);
        app.world_mut().resource_mut::<Appearance>().auto_frame = false;
        app.world_mut().resource_mut::<GlobalState>().vehicle_count = 2000;
        app.update();
        app.world_mut()
            .resource_mut::<Time<Real>>()
            .advance_by(std::time::Duration::from_secs(1));
        app.update();
        assert_eq!(*app.world().get::<Transform>(entity).unwrap(), framed);
        app.world_mut().resource_mut::<Appearance>().frame_requested = true;
        app.update();
        assert_ne!(
            app.world().get::<Transform>(entity).unwrap().translation,
            framed.translation
        );
    }
}
