//! Optional HDR skies and cached, CPU-prefiltered image-based lighting.
//! Cubemap storage follows WebGPU's face convention; Bevy flips Z when sampling it.
use std::{
    f32::consts::{PI, TAU},
    str::FromStr,
};

use bevy::{
    asset::{LoadState, RenderAssetUsages},
    camera::{Exposure, Hdr},
    image::ImageSampler,
    light::{EnvironmentMapLight, Skybox},
    prelude::*,
    render::{
        render_resource::{
            Extent3d, TextureDataOrder, TextureDimension, TextureFormat, TextureViewDescriptor,
            TextureViewDimension,
        },
        view::Tonemapping,
    },
    transform::TransformSystems,
};

use crate::{
    scene::{Appearance, FillLight, KeyLight, SceneStyle},
    vehicle, GlobalState,
};

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Environment {
    #[default]
    Off,
    Sunset,
    Midday,
    Underwater,
}
impl Environment {
    pub const ALL: [Self; 4] = [Self::Off, Self::Sunset, Self::Midday, Self::Underwater];
    pub fn name(self) -> &'static str {
        match self {
            Self::Off => "Off",
            Self::Sunset => "Sunset",
            Self::Midday => "Midday",
            Self::Underwater => "Underwater",
        }
    }
    fn source(self) -> Option<&'static str> {
        match self {
            Self::Off | Self::Underwater => None,
            Self::Sunset => Some("environment/kloppenheim_06_puresky_4k.hdr"),
            Self::Midday => Some("environment/kloofendal_48d_partly_cloudy_puresky_2k.hdr"),
        }
    }
    fn sun_lux(self) -> f32 {
        match self {
            Self::Sunset => 12000.0,
            Self::Underwater => 9000.0,
            _ => 24000.0,
        }
    }
    fn sky_brightness(self) -> f32 {
        match self {
            Self::Sunset => 400.0,
            _ => 800.0,
        }
    }
    fn ibl_intensity(self) -> f32 {
        match self {
            Self::Sunset => 650.0,
            Self::Underwater => 850.0,
            _ => 1000.0,
        }
    }
}
impl FromStr for Environment {
    type Err = String;
    fn from_str(value: &str) -> Result<Self, Self::Err> {
        match value {
            "off" => Ok(Self::Off),
            "sunset" => Ok(Self::Sunset),
            "midday" => Ok(Self::Midday),
            "underwater" => Ok(Self::Underwater),
            _ => Err(format!(
                "invalid environment {value:?}; expected off, sunset, midday or underwater"
            )),
        }
    }
}

const FACE_SIZE: u32 = 512;
const ENV_EXPOSURE: Exposure = Exposure { ev100: 10.0 };

struct CachedEnvironment {
    kind: Environment,
    source: Option<Handle<Image>>,
    ready: Option<PreparedEnvironment>,
    failed: bool,
}
struct PreparedEnvironment {
    sky: Handle<Image>,
    light: EnvironmentMapLight,
    sun: Vec3,
    sun_color: Color,
    horizon: Color,
}
#[derive(Resource, Default)]
struct EnvironmentCache(Vec<CachedEnvironment>);

/// Save only the camera values this feature owns, including explicit custom exposure.
#[derive(Component)]
struct PreviousCamera {
    active: Environment,
    exposure: Option<Exposure>,
    tonemapping: Tonemapping,
}

pub struct EnvironmentPlugin;
impl Plugin for EnvironmentPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(crate::underwater::UnderwaterPlugin)
            .init_resource::<EnvironmentCache>()
            .add_systems(
                PostUpdate,
                update_environment
                    .after(crate::scene::draw_world)
                    .after(crate::scene::sync_glow_camera)
                    .before(bevy::camera::CameraUpdateSystems)
                    .before(TransformSystems::Propagate),
            );
    }
}

#[allow(clippy::too_many_arguments, clippy::type_complexity)]
fn update_environment(
    mut commands: Commands,
    appearance: Res<Appearance>,
    state: Res<GlobalState>,
    server: Res<AssetServer>,
    mut images: ResMut<Assets<Image>>,
    mut cache: ResMut<EnvironmentCache>,
    mut cameras: Query<
        (
            Entity,
            &Transform,
            &mut DistanceFog,
            &Tonemapping,
            Option<&Exposure>,
            Option<&PreviousCamera>,
        ),
        With<Camera3d>,
    >,
    mut lights: Query<
        (
            &mut DirectionalLight,
            &mut Transform,
            Has<KeyLight>,
            Has<FillLight>,
        ),
        Without<Camera3d>,
    >,
) {
    let kind = appearance.environment;
    if kind == Environment::Off {
        for (entity, _, _, _, _, previous) in &mut cameras {
            if let Some(previous) = previous {
                let mut camera = commands.entity(entity);
                camera.remove::<(Skybox, EnvironmentMapLight, PreviousCamera)>();
                camera.insert(previous.tonemapping);
                if let Some(exposure) = &previous.exposure {
                    camera.insert(*exposure);
                } else {
                    camera.remove::<Exposure>();
                }
                if appearance.style != SceneStyle::Glow {
                    camera.remove::<Hdr>();
                }
            }
        }
        // The scene restores its light constants when Appearance changes; restore key orientation too.
        if appearance.is_changed() {
            for (_, mut transform, key, _) in &mut lights {
                if key {
                    *transform = Transform::from_xyz(1.0, 2.0, 0.6).looking_at(Vec3::ZERO, Vec3::Y);
                }
            }
        }
        return;
    }
    if !cache.0.iter().any(|entry| entry.kind == kind) {
        cache.0.push(CachedEnvironment {
            kind,
            source: kind.source().map(|path| server.load(path)),
            ready: (kind == Environment::Underwater).then(|| prepare_underwater(&mut images)),
            failed: false,
        });
    }
    let entry = cache.0.iter_mut().find(|entry| entry.kind == kind).unwrap();
    if entry.ready.is_none() && !entry.failed {
        let source = entry.source.as_ref().unwrap();
        if let Some(image) = images.get(source) {
            match Panorama::from_image(image) {
                Ok(panorama) => {
                    let (sun, sun_color) = panorama.sun();
                    let horizon =
                        panorama.horizon() * kind.sky_brightness() * ENV_EXPOSURE.exposure();
                    let sky = images.add(cubemap(FACE_SIZE, 1, |n, _| panorama.sample(n)));
                    let diffuse =
                        images.add(cubemap(32, 1, |n, _| convolve(&panorama, n, 1.0, true)));
                    let specular =
                        images.add(cubemap(FACE_SIZE, FACE_SIZE.ilog2() + 1, |n, roughness| {
                            if roughness == 0.0 {
                                panorama.sample(n)
                            } else {
                                convolve(&panorama, n, roughness, false)
                            }
                        }));
                    entry.ready = Some(PreparedEnvironment {
                        sky,
                        sun,
                        sun_color,
                        horizon: Color::linear_rgb(horizon.x, horizon.y, horizon.z),
                        light: EnvironmentMapLight {
                            diffuse_map: diffuse,
                            specular_map: specular,
                            intensity: kind.ibl_intensity(),
                            ..default()
                        },
                    });
                    entry.source = None;
                    info!("Prepared {} HDRI: 512px sky, diffuse irradiance and GGX mip chain; sun {sun:?}", kind.name());
                }
                Err(error) => {
                    error!("Cannot prepare {}: {error}", kind.name());
                    entry.failed = true;
                }
            }
        } else if matches!(
            server.get_load_state(source.id()),
            Some(LoadState::Failed(_))
        ) {
            error!("Cannot load environment {:?}", kind.source());
            entry.failed = true;
        }
    }
    // Hide the old rig even while loading. A failed asset logs once; Off remains recoverable.
    for (mut light, _, _, _) in &mut lights {
        light.illuminance = 0.0;
    }
    let Some(prepared) = &entry.ready else {
        return;
    };
    for (entity, transform, mut fog, tonemapping, exposure, previous) in &mut cameras {
        if previous.is_none_or(|saved| saved.active != kind) {
            commands.entity(entity).insert(PreviousCamera {
                active: kind,
                exposure: previous.map_or(exposure.copied(), |saved| saved.exposure),
                tonemapping: previous.map_or(*tonemapping, |saved| saved.tonemapping),
            });
            commands.entity(entity).insert((
                Hdr,
                Tonemapping::AgX,
                ENV_EXPOSURE,
                Skybox {
                    image: Some(prepared.sky.clone()),
                    brightness: kind.sky_brightness(),
                    ..default()
                },
                prepared.light.clone(),
            ));
        }
        fog.color = prepared.horizon;
        let far = crate::scene::far_wall_distance(
            transform.translation,
            vehicle::walls(state.vehicle_count),
        );
        fog.falloff = if kind == Environment::Underwater {
            FogFalloff::Exponential {
                density: 0.5 / vehicle::walls(state.vehicle_count).x,
            }
        } else {
            FogFalloff::Linear {
                start: far * 0.35,
                end: far * 3.5,
            }
        };
    }
    for (mut light, mut transform, key, _) in &mut lights {
        if key {
            light.illuminance = kind.sun_lux();
            light.color = prepared.sun_color;
            let up = if prepared.sun.dot(Vec3::Y).abs() > 0.99 {
                Vec3::Z
            } else {
                Vec3::Y
            };
            *transform = Transform::from_translation(prepared.sun).looking_at(Vec3::ZERO, up);
        }
    }
}

/// Smooth water-column radiance, with a broad refracted patch of light overhead.
/// Direction-only detail keeps the six faces continuous and needs no downloaded assets.
fn water_radiance(n: Vec3) -> Vec3 {
    let deep = Vec3::new(0.0005, 0.003, 0.025);
    let horizon = Vec3::new(0.006, 0.075, 0.19);
    let surface = Vec3::new(0.025, 0.85, 0.72);
    if n.y < 0.0 {
        return horizon.lerp(deep, (-n.y).powf(0.7));
    }
    let gradient = horizon.lerp(surface, n.y.powf(1.4));
    let light = n
        .dot(Vec3::new(-0.18, 0.97, -0.16).normalize())
        .max(0.0)
        .powf(20.0);
    let ripple = (n.x * 47.0 + (n.z * 31.0).sin()).sin() * (n.z * 53.0 + (n.x * 29.0).cos()).sin();
    gradient + Vec3::new(0.65, 2.1, 1.5) * light * (0.85 + 0.15 * ripple)
}

fn prepare_underwater(images: &mut Assets<Image>) -> PreparedEnvironment {
    // A low-frequency analytic sky needs only 128px; broad IBL lobes need even less.
    let sky = images.add(cubemap(128, 1, |n, _| water_radiance(n)));
    let panorama = Panorama {
        width: 64,
        height: 32,
        pixels: (0..32)
            .flat_map(|y| {
                (0..64).map(move |x| {
                    water_radiance(panorama_direction(
                        (x as f32 + 0.5) / 64.0,
                        (y as f32 + 0.5) / 32.0,
                    ))
                })
            })
            .collect(),
    };
    let diffuse = images.add(cubemap(16, 1, |n, _| convolve(&panorama, n, 1.0, true)));
    let specular = images.add(cubemap(64, 7, |n, roughness| {
        convolve(&panorama, n, roughness.max(0.04), false)
    }));
    let horizon = water_radiance(Vec3::X)
        * Environment::Underwater.sky_brightness()
        * ENV_EXPOSURE.exposure();
    PreparedEnvironment {
        sky,
        light: EnvironmentMapLight {
            diffuse_map: diffuse,
            specular_map: specular,
            intensity: Environment::Underwater.ibl_intensity(),
            ..default()
        },
        sun: Vec3::new(-0.18, 0.97, -0.16).normalize(),
        sun_color: Color::srgb(0.72, 1.0, 0.86),
        horizon: Color::linear_rgb(horizon.x, horizon.y, horizon.z),
    }
}

struct Panorama {
    width: u32,
    height: u32,
    pixels: Vec<Vec3>,
}
fn luminance(rgb: Vec3) -> f32 {
    rgb.dot(Vec3::new(0.2126, 0.7152, 0.0722))
}
fn panorama_direction(u: f32, v: f32) -> Vec3 {
    let latitude = (0.5 - v) * PI;
    let longitude = (u - 0.5) * TAU;
    Vec3::new(
        latitude.cos() * longitude.cos(),
        latitude.sin(),
        latitude.cos() * longitude.sin(),
    )
}
impl Panorama {
    fn from_image(image: &Image) -> Result<Self, &'static str> {
        if image.texture_descriptor.format != TextureFormat::Rgba32Float {
            return Err("expected linear Rgba32Float HDR");
        }
        let data = image.data.as_ref().ok_or("missing CPU HDR data")?;
        let pixels = data
            .as_chunks::<16>()
            .0
            .iter()
            .map(|bytes| {
                Vec3::new(
                    f32::from_le_bytes(bytes[0..4].try_into().unwrap()),
                    f32::from_le_bytes(bytes[4..8].try_into().unwrap()),
                    f32::from_le_bytes(bytes[8..12].try_into().unwrap()),
                )
                .max(Vec3::ZERO)
            })
            .collect();
        let mut panorama = Self {
            width: image.width(),
            height: image.height(),
            pixels,
        };
        // Normalize upper-hemisphere mean radiance, retaining the HDR highlight range and colors.
        let mut sum = 0.0;
        let mut weight = 0.0;
        for y in 0..panorama.height / 2 {
            let area = (PI * (y as f32 + 0.5) / panorama.height as f32).sin();
            for x in 0..panorama.width {
                sum += luminance(panorama.pixel(x, y)) * area;
                weight += area;
            }
        }
        let scale = weight / sum.max(1e-6);
        for pixel in &mut panorama.pixels {
            *pixel *= scale;
        }
        Ok(panorama)
    }
    fn pixel(&self, x: u32, y: u32) -> Vec3 {
        self.pixels[(y * self.width + x) as usize]
    }
    fn sample(&self, direction: Vec3) -> Vec3 {
        let u = (direction.z.atan2(direction.x) / TAU + 0.5) * self.width as f32 - 0.5;
        let v = ((0.5 - direction.y.clamp(-1.0, 1.0).asin() / PI) * self.height as f32 - 0.5)
            .clamp(0.0, (self.height - 1) as f32);
        let x = u.floor() as i32;
        let y = v.floor() as u32;
        let a = self.pixel(x.rem_euclid(self.width as i32) as u32, y).lerp(
            self.pixel((x + 1).rem_euclid(self.width as i32) as u32, y),
            u.fract().rem_euclid(1.0),
        );
        let b = self
            .pixel(
                x.rem_euclid(self.width as i32) as u32,
                (y + 1).min(self.height - 1),
            )
            .lerp(
                self.pixel(
                    (x + 1).rem_euclid(self.width as i32) as u32,
                    (y + 1).min(self.height - 1),
                ),
                u.fract().rem_euclid(1.0),
            );
        a.lerp(b, v.fract())
    }
    fn horizon(&self) -> Vec3 {
        (0..self.width)
            .map(|x| self.pixel(x, self.height / 2 - 1))
            .sum::<Vec3>()
            / self.width as f32
    }
    fn sun(&self) -> (Vec3, Color) {
        let peak = self.pixels[..(self.width * (self.height / 2)) as usize]
            .iter()
            .map(|p| luminance(*p))
            .fold(0.0, f32::max);
        let mut direction = Vec3::ZERO;
        let mut color = Vec3::ZERO;
        for y in 0..self.height / 2 {
            for x in 0..self.width {
                let n = panorama_direction(
                    (x as f32 + 0.5) / self.width as f32,
                    (y as f32 + 0.5) / self.height as f32,
                );
                let rgb = self.pixel(x, y);
                let weight =
                    (luminance(rgb) - peak * 0.8).max(0.0) * (1.0 - n.y * n.y).max(0.0).sqrt();
                direction += n * weight;
                color += rgb * weight;
            }
        }
        color /= color.max_element().max(1e-6);
        (
            direction.try_normalize().unwrap_or(Vec3::Y),
            Color::linear_rgb(color.x, color.y, color.z),
        )
    }
}

/// WebGPU +X, -X, +Y, -Y, +Z, -Z faces, converted to Bevy world directions.
fn cube_direction(face: u32, u: f32, v: f32) -> Vec3 {
    let n = match face {
        0 => Vec3::new(1.0, -v, -u),
        1 => Vec3::new(-1.0, -v, u),
        2 => Vec3::new(u, 1.0, v),
        3 => Vec3::new(u, -1.0, -v),
        4 => Vec3::new(u, -v, 1.0),
        5 => Vec3::new(-u, -v, -1.0),
        _ => unreachable!(),
    };
    (n * Vec3::new(1.0, 1.0, -1.0)).normalize()
}
fn cubemap(size: u32, mips: u32, sample: impl Fn(Vec3, f32) -> Vec3) -> Image {
    let mut data = Vec::new();
    for mip in 0..mips {
        let side = size >> mip;
        let mut image = Image::new_fill(
            Extent3d {
                width: side,
                height: side,
                depth_or_array_layers: 6,
            },
            TextureDimension::D2,
            &[0; 8],
            TextureFormat::Rgba16Float,
            RenderAssetUsages::RENDER_WORLD,
        );
        for face in 0..6 {
            for y in 0..side {
                for x in 0..side {
                    let n = cube_direction(
                        face,
                        2.0 * (x as f32 + 0.5) / side as f32 - 1.0,
                        2.0 * (y as f32 + 0.5) / side as f32 - 1.0,
                    );
                    let color = sample(n, mip as f32 / (mips - 1).max(1) as f32)
                        .clamp(Vec3::ZERO, Vec3::splat(65504.0));
                    image
                        .set_color_at_3d(x, y, face, Color::linear_rgb(color.x, color.y, color.z))
                        .unwrap();
                }
            }
        }
        data.extend(image.data.take().unwrap());
    }
    let mut image = Image::new_uninit(
        Extent3d {
            width: size,
            height: size,
            depth_or_array_layers: 6,
        },
        TextureDimension::D2,
        TextureFormat::Rgba16Float,
        RenderAssetUsages::RENDER_WORLD,
    );
    image.data = Some(data);
    image.data_order = TextureDataOrder::MipMajor;
    image.texture_descriptor.mip_level_count = mips;
    image.texture_view_descriptor = Some(TextureViewDescriptor {
        dimension: Some(TextureViewDimension::Cube),
        ..default()
    });
    image.sampler = ImageSampler::linear();
    image
}

/// Deterministic cosine-weighted irradiance and split-sum GGX specular convolution.
fn convolve(panorama: &Panorama, n: Vec3, roughness: f32, diffuse: bool) -> Vec3 {
    let up = if n.y.abs() < 0.99 { Vec3::Y } else { Vec3::X };
    let tangent = up.cross(n).normalize();
    let bitangent = n.cross(tangent);
    let mut sum = Vec3::ZERO;
    let mut weight = 0.0;
    const SAMPLES: u32 = 64;
    for i in 0..SAMPLES {
        let u = (i as f32 + 0.5) / SAMPLES as f32;
        let v = i.reverse_bits() as f32 * (1.0 / 4294967296.0);
        let phi = TAU * v;
        let alpha = roughness * roughness;
        let cos_theta = if diffuse {
            (1.0 - u).sqrt()
        } else {
            ((1.0 - u) / (1.0 + (alpha * alpha - 1.0) * u)).sqrt()
        };
        let sin_theta = (1.0 - cos_theta * cos_theta).max(0.0).sqrt();
        let h =
            tangent * (phi.cos() * sin_theta) + bitangent * (phi.sin() * sin_theta) + n * cos_theta;
        let direction = if diffuse { h } else { 2.0 * n.dot(h) * h - n };
        let w = if diffuse {
            1.0
        } else {
            n.dot(direction).max(0.0)
        };
        sum += panorama.sample(direction) * w;
        weight += w;
    }
    sum / weight.max(1e-6)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn underwater_is_procedural_cached_and_restores_the_camera() {
        use bevy::asset::AssetPlugin;
        assert!(water_radiance(Vec3::Y).y > water_radiance(Vec3::NEG_Y).y * 20.0);
        let mut app = App::new();
        app.add_plugins((MinimalPlugins, AssetPlugin::default()))
            .init_asset::<Image>()
            .insert_resource(Appearance {
                environment: Environment::Underwater,
                ..default()
            })
            .init_resource::<GlobalState>()
            .init_resource::<EnvironmentCache>()
            .add_systems(Update, update_environment);
        let camera = app
            .world_mut()
            .spawn((
                Camera3d::default(),
                Transform::default(),
                DistanceFog::default(),
                Tonemapping::TonyMcMapface,
            ))
            .id();
        app.update();
        let entity = app.world().entity(camera);
        assert!(entity.contains::<EnvironmentMapLight>());
        assert!(
            matches!(entity.get::<DistanceFog>().unwrap().falloff, FogFalloff::Exponential { density } if density > 0.0)
        );
        let sky = entity.get::<Skybox>().unwrap().image.clone();
        let image_count = app.world().resource::<Assets<Image>>().len();
        assert_eq!(image_count, 3);
        assert!(app.world().resource::<EnvironmentCache>().0[0]
            .source
            .is_none());
        app.world_mut().resource_mut::<Appearance>().environment = Environment::Off;
        app.update();
        assert!(!app.world().entity(camera).contains::<Skybox>());
        assert!(!app.world().entity(camera).contains::<Hdr>());
        assert_eq!(
            *app.world().get::<Tonemapping>(camera).unwrap(),
            Tonemapping::TonyMcMapface
        );
        app.world_mut().resource_mut::<Appearance>().environment = Environment::Underwater;
        app.update();
        assert_eq!(app.world().get::<Skybox>(camera).unwrap().image, sky);
        assert_eq!(app.world().resource::<Assets<Image>>().len(), image_count);
    }

    #[test]
    fn sun_extraction_uses_hdr_peak_and_world_direction() {
        let (width, height) = (32, 16);
        let mut panorama = Panorama {
            width,
            height,
            pixels: vec![Vec3::splat(0.1); (width * height) as usize],
        };
        for (x, y) in [(2, 6), (16, 1), (27, 7)] {
            panorama.pixels.fill(Vec3::splat(0.1));
            panorama.pixels[(y * width + x) as usize] = Vec3::new(100.0, 50.0, 25.0);
            // Lower-hemisphere highlights are not a sun.
            panorama.pixels[(15 * width) as usize] = Vec3::splat(10000.0);
            let (direction, color) = panorama.sun();
            let expected = panorama_direction(
                (x as f32 + 0.5) / width as f32,
                (y as f32 + 0.5) / height as f32,
            );
            assert!(direction.distance(expected) < 1e-6);
            assert_eq!(color, Color::linear_rgb(1.0, 0.5, 0.25));
        }
    }

    #[test]
    fn cube_faces_match_bevy_sampling_and_share_edges() {
        for (face, expected) in [
            Vec3::X,
            Vec3::NEG_X,
            Vec3::Y,
            Vec3::NEG_Y,
            Vec3::NEG_Z,
            Vec3::Z,
        ]
        .into_iter()
        .enumerate()
        {
            assert_eq!(cube_direction(face as u32, 0.0, 0.0), expected);
        }
        assert!(cube_direction(0, -1.0, 0.3).distance(cube_direction(4, 1.0, 0.3)) < 1e-6);
        assert!(cube_direction(0, 0.2, -1.0).distance(cube_direction(2, 1.0, -0.2)) < 1e-6);
        let colored = Panorama {
            width: 4,
            height: 2,
            pixels: (0..8).map(|i| Vec3::splat(i as f32)).collect(),
        };
        for y in 0..2 {
            for x in 0..4 {
                let n = panorama_direction((x as f32 + 0.5) / 4.0, (y as f32 + 0.5) / 2.0);
                assert!(colored.sample(n).distance(colored.pixel(x, y)) < 1e-5);
            }
        }
        // The longitude seam wraps; poles clamp instead of wrapping vertically.
        assert!(colored.sample(Vec3::NEG_X).distance(Vec3::splat(3.5)) < 1e-5);
        let panorama = Panorama {
            width: 4,
            height: 2,
            pixels: vec![Vec3::ONE; 8],
        };
        for n in [Vec3::Y, Vec3::NEG_Y, Vec3::X, Vec3::NEG_X] {
            assert_eq!(panorama.sample(n), Vec3::ONE);
            assert!((convolve(&panorama, n, 0.5, false) - Vec3::ONE).length() < 1e-6);
            assert!((convolve(&panorama, n, 1.0, true) - Vec3::ONE).length() < 1e-6);
        }
        let image = cubemap(4, 3, |_, _| Vec3::ONE);
        assert_eq!(image.data.as_ref().unwrap().len(), (16 + 4 + 1) * 6 * 8);
        assert_eq!(image.texture_descriptor.mip_level_count, 3);
        assert_eq!(
            image.texture_view_descriptor.unwrap().dimension,
            Some(TextureViewDimension::Cube)
        );
    }

    #[test]
    fn environment_off_preserves_camera_and_round_trip_restores_it() {
        use bevy::asset::AssetPlugin;
        let mut app = App::new();
        app.add_plugins((MinimalPlugins, AssetPlugin::default()))
            .init_asset::<Image>()
            .init_resource::<Appearance>()
            .init_resource::<GlobalState>()
            .init_resource::<EnvironmentCache>()
            .add_systems(
                Update,
                (crate::scene::sync_glow_camera, update_environment).chain(),
            );
        let camera = app
            .world_mut()
            .spawn((
                Camera3d::default(),
                Tonemapping::TonyMcMapface,
                Exposure { ev100: 9.7 },
                Msaa::Sample4,
                DistanceFog::default(),
                Transform::default(),
            ))
            .id();
        for style in [SceneStyle::Studio, SceneStyle::Classic, SceneStyle::Glow] {
            app.world_mut().resource_mut::<Appearance>().style = style;
            app.update();
            let entity = app.world().entity(camera);
            assert_eq!(entity.contains::<Hdr>(), style == SceneStyle::Glow);
            assert_eq!(
                entity.contains::<bevy::post_process::bloom::Bloom>(),
                style == SceneStyle::Glow
            );
            assert!(!entity.contains::<Skybox>());
            assert!(!entity.contains::<EnvironmentMapLight>());
            assert!(!entity.contains::<PreviousCamera>());
            assert_eq!(
                entity.get::<Tonemapping>(),
                Some(&Tonemapping::TonyMcMapface)
            );
            assert_eq!(entity.get::<Exposure>().unwrap().ev100, 9.7);
            assert_eq!(entity.get::<Msaa>(), Some(&Msaa::Sample4));
            assert!(app.world().resource::<EnvironmentCache>().0.is_empty());
        }
        // Inject a prepared map to test lifecycle without loading files or starting a renderer.
        app.world_mut()
            .resource_mut::<EnvironmentCache>()
            .0
            .push(CachedEnvironment {
                kind: Environment::Sunset,
                source: None,
                failed: false,
                ready: Some(PreparedEnvironment {
                    sky: default(),
                    light: EnvironmentMapLight::default(),
                    sun: Vec3::Y,
                    sun_color: Color::WHITE,
                    horizon: Color::WHITE,
                }),
            });
        for style in [SceneStyle::Studio, SceneStyle::Glow, SceneStyle::Classic] {
            {
                let mut appearance = app.world_mut().resource_mut::<Appearance>();
                appearance.style = style;
                appearance.environment = Environment::Sunset;
            }
            app.update();
            assert!(app.world().entity(camera).contains::<Hdr>());
            assert!(app.world().entity(camera).contains::<EnvironmentMapLight>());
            assert_eq!(
                app.world().get::<Tonemapping>(camera),
                Some(&Tonemapping::AgX)
            );
            app.world_mut().resource_mut::<Appearance>().environment = Environment::Off;
            app.update();
            assert_eq!(
                app.world().entity(camera).contains::<Hdr>(),
                style == SceneStyle::Glow
            );
            assert_eq!(
                app.world().get::<Tonemapping>(camera),
                Some(&Tonemapping::TonyMcMapface)
            );
            assert_eq!(app.world().get::<Exposure>(camera).unwrap().ev100, 9.7);
            assert!(!app.world().entity(camera).contains::<Skybox>());
        }
        app.world_mut().resource_mut::<Appearance>().environment = Environment::Sunset;
        app.update();
        app.world_mut()
            .resource_mut::<EnvironmentCache>()
            .0
            .push(CachedEnvironment {
                kind: Environment::Midday,
                source: None,
                ready: None,
                failed: true,
            });
        app.world_mut().resource_mut::<Appearance>().environment = Environment::Midday;
        app.update();
        assert_eq!(
            app.world().get::<PreviousCamera>(camera).unwrap().active,
            Environment::Sunset
        );
        // Simulate an asynchronous asset becoming ready after the selection's change tick.
        let sky = app
            .world_mut()
            .resource_mut::<Assets<Image>>()
            .add(Image::default());
        app.world_mut().resource_mut::<EnvironmentCache>().0[1].ready = Some(PreparedEnvironment {
            sky: sky.clone(),
            light: EnvironmentMapLight::default(),
            sun: Vec3::Y,
            sun_color: Color::WHITE,
            horizon: Color::WHITE,
        });
        app.update();
        assert_eq!(
            app.world().get::<PreviousCamera>(camera).unwrap().active,
            Environment::Midday
        );
        assert_eq!(app.world().get::<Skybox>(camera).unwrap().image, Some(sky));
        assert_eq!(
            app.world()
                .get::<PreviousCamera>(camera)
                .unwrap()
                .tonemapping,
            Tonemapping::TonyMcMapface
        );
    }
}
