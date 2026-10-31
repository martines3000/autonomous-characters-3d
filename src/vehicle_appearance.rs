//! Shared vehicle meshes and render-only settings. No simulation system reads these settings.
use bevy::{
    asset::RenderAssetUsages, mesh::Indices, prelude::*, render::render_resource::PrimitiveTopology,
};
use bevy_egui::egui;

use crate::{instancing::VehicleInstances, scene::Appearance};

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum VehicleShape {
    #[default]
    Dart,
    Chevron,
    Fish,
    Bird,
    Custom,
}

impl VehicleShape {
    pub const ALL: [Self; 4] = [Self::Dart, Self::Chevron, Self::Fish, Self::Bird];
    pub fn name(self) -> &'static str {
        match self {
            Self::Dart => "Dart",
            Self::Chevron => "Chevron",
            Self::Fish => "Fish",
            Self::Bird => "Bird",
            Self::Custom => "Custom model",
        }
    }
    pub fn mesh(self) -> Mesh {
        let mut faces = Faces::default();
        match self {
            Self::Custom => unreachable!("custom meshes come from VehicleModels"),
            Self::Dart => return Cone::new(0.7, 2.5).mesh().resolution(8).build(),
            Self::Chevron => faces.prism(
                &[
                    Vec2::new(0.0, 1.25),
                    Vec2::new(-0.7, -1.25),
                    Vec2::new(0.0, -0.45),
                    Vec2::new(0.7, -1.25),
                ],
                &[[0, 1, 2], [0, 2, 3]],
                0.08,
            ),
            Self::Fish => {
                faces.body(1.25, -0.70, 0.25, 0.46, 0.24);
                faces.prism(
                    &[
                        Vec2::new(0.0, -0.60),
                        Vec2::new(-0.65, -1.25),
                        Vec2::new(0.65, -1.25),
                    ],
                    &[[0, 1, 2]],
                    0.05,
                );
            }
            Self::Bird => {
                faces.body(1.25, -1.25, 0.0, 0.14, 0.12);
                faces.prism(
                    &[
                        Vec2::new(-0.06, 0.55),
                        Vec2::new(-0.90, -0.35),
                        Vec2::new(-0.12, -0.75),
                    ],
                    &[[0, 1, 2]],
                    0.035,
                );
                faces.prism(
                    &[
                        Vec2::new(0.06, 0.55),
                        Vec2::new(0.12, -0.75),
                        Vec2::new(0.90, -0.35),
                    ],
                    &[[0, 1, 2]],
                    0.035,
                );
            }
        }
        faces.mesh()
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(u32)]
pub enum ColorMode {
    #[default]
    Heading,
    Altitude,
    Speed,
    Solid,
    ModelColors,
}
impl ColorMode {
    pub const ALL: [Self; 5] = [
        Self::Heading,
        Self::Altitude,
        Self::Speed,
        Self::Solid,
        Self::ModelColors,
    ];
    pub fn name(self) -> &'static str {
        match self {
            Self::Heading => "Heading",
            Self::Altitude => "Altitude",
            Self::Speed => "Speed",
            Self::Solid => "Solid",
            Self::ModelColors => "Model colors",
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct VehicleAppearance {
    pub shape: VehicleShape,
    pub visual_size: f32,
    pub length: f32,
    pub width: f32,
    pub color_mode: ColorMode,
    /// Picker and scripts use sRGB; the shader receives its linear equivalent.
    pub solid_color: [u8; 3],
}
impl Default for VehicleAppearance {
    fn default() -> Self {
        Self {
            shape: VehicleShape::default(),
            visual_size: 1.0,
            length: 1.0,
            width: 1.0,
            color_mode: ColorMode::default(),
            solid_color: [255, 180, 65],
        }
    }
}

pub fn ui(ui: &mut egui::Ui, appearance: &mut VehicleAppearance) {
    ui.add(egui::Slider::new(&mut appearance.visual_size, 0.1..=5.0).text("Render size"))
        .on_hover_text("Render-only multiplier. Does not change GlobalState::vehicle_size, neighbor radii, separation or steering.");
    ui.add(egui::Slider::new(&mut appearance.length, 0.25..=3.0).text("Length"))
        .on_hover_text("Render-only scale along local +Y (forward).");
    ui.add(egui::Slider::new(&mut appearance.width, 0.25..=3.0).text("Width"))
        .on_hover_text("Render-only scale on both axes perpendicular to forward (local X and Z).");
    ui.horizontal_wrapped(|ui| {
        ui.label("Color");
        for mode in ColorMode::ALL {
            ui.selectable_value(&mut appearance.color_mode, mode, mode.name()).on_hover_text(match mode {
                ColorMode::Heading => "Heading uses the selected cyclic palette; matching directions share a color.",
                ColorMode::Altitude => "Palette ramp from the bottom wall to the top wall, clamped outside the walls.",
                ColorMode::Speed => "Palette ramp from zero speed to the simulation's configured maximum speed.",
                ColorMode::Solid => "One user-chosen sRGB color for every vehicle.",
                ColorMode::ModelColors => "Imported linear base color factors × COLOR_0; built-in meshes fall back to white.",
            });
        }
    });
    if appearance.color_mode == ColorMode::Solid {
        ui.horizontal(|ui| {
            ui.label("Solid color");
            ui.color_edit_button_srgb(&mut appearance.solid_color);
        });
    }
    ui.small("Appearance does not change simulation size or separation.");
}

#[derive(Resource)]
pub(crate) struct VehicleMeshes([Handle<Mesh>; 4]);
impl VehicleMeshes {
    pub(crate) fn new(meshes: &mut Assets<Mesh>) -> Self {
        Self(VehicleShape::ALL.map(|shape| meshes.add(shape.mesh())))
    }
    pub(crate) fn get(&self, shape: VehicleShape) -> Handle<Mesh> {
        self.0[shape as usize].clone()
    }
}

pub(crate) fn update_shape(
    appearance: Res<Appearance>,
    meshes: Res<VehicleMeshes>,
    models: Res<crate::vehicle_model::VehicleModels>,
    mut flocks: Query<&mut Mesh3d, With<VehicleInstances>>,
) {
    let handle = if appearance.vehicles.shape == VehicleShape::Custom {
        let Some(handle) = models.current.as_ref() else {
            return;
        };
        handle.clone()
    } else {
        meshes.get(appearance.vehicles.shape)
    };
    for mut mesh in &mut flocks {
        if mesh.0 != handle {
            mesh.0 = handle.clone();
        }
    }
}

/// Stage 7 transport reservation, not group behavior: mantissa bits 16..22 hold group (0..127),
/// bits 0..15 hold normalized speed (0..65535). Exponent 127 keeps the payload finite and normal.
/// Both simulation backends currently write group zero. Keep this encoding in sync with WGSL.
pub(crate) fn pack_speed_group(speed: f32, group: u8) -> f32 {
    assert!(group < 128);
    let speed = if speed.is_finite() {
        speed.clamp(0.0, 1.0)
    } else {
        0.0
    };
    let quantized = (speed * 65535.0 + 0.5).floor() as u32;
    f32::from_bits(0x3f80_0000 | (u32::from(group) << 16) | quantized)
}

#[cfg(test)]
pub(crate) fn unpack_speed_group(packed: f32) -> (f32, u8) {
    let bits = packed.to_bits();
    (
        (bits & 0xffff) as f32 / 65535.0,
        ((bits >> 16) & 0x7f) as u8,
    )
}

// Separate vertices per triangle keep every custom face flat shaded.
#[derive(Default)]
struct Faces {
    positions: Vec<[f32; 3]>,
    normals: Vec<[f32; 3]>,
}
impl Faces {
    fn triangle(&mut self, a: Vec3, b: Vec3, c: Vec3) {
        let normal = (b - a).cross(c - a).normalize();
        self.positions
            .extend([a.to_array(), b.to_array(), c.to_array()]);
        self.normals.extend([normal.to_array(); 3]);
    }
    fn prism(&mut self, polygon: &[Vec2], triangles: &[[usize; 3]], half: f32) {
        for &[a, b, c] in triangles {
            self.triangle(
                polygon[a].extend(half),
                polygon[b].extend(half),
                polygon[c].extend(half),
            );
            self.triangle(
                polygon[a].extend(-half),
                polygon[c].extend(-half),
                polygon[b].extend(-half),
            );
        }
        for i in 0..polygon.len() {
            let a = polygon[i];
            let b = polygon[(i + 1) % polygon.len()];
            self.triangle(a.extend(-half), b.extend(-half), b.extend(half));
            self.triangle(a.extend(-half), b.extend(half), a.extend(half));
        }
    }
    fn body(&mut self, nose: f32, tail: f32, middle: f32, width: f32, depth: f32) {
        let center = Vec3::new(0.0, middle, 0.0);
        let ring = [
            Vec3::new(width, middle, 0.0),
            Vec3::new(0.0, middle, depth),
            Vec3::new(-width, middle, 0.0),
            Vec3::new(0.0, middle, -depth),
        ];
        for tip in [Vec3::new(0.0, nose, 0.0), Vec3::new(0.0, tail, 0.0)] {
            for i in 0..4 {
                let a = ring[i];
                let b = ring[(i + 1) % 4];
                if (a - tip).cross(b - tip).dot((tip + a + b) / 3.0 - center) > 0.0 {
                    self.triangle(tip, a, b);
                } else {
                    self.triangle(tip, b, a);
                }
            }
        }
    }
    fn mesh(self) -> Mesh {
        let indices: Vec<u32> = (0..self.positions.len() as u32).collect();
        Mesh::new(
            PrimitiveTopology::TriangleList,
            RenderAssetUsages::default(),
        )
        .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, self.positions)
        .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, self.normals)
        .with_inserted_indices(Indices::U32(indices))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bevy::mesh::VertexAttributeValues;

    #[test]
    fn meshes_have_bounded_forward_length_finite_normals_and_correct_winding() {
        for (shape, triangles) in VehicleShape::ALL.into_iter().zip([14, 12, 16, 24]) {
            let mesh = shape.mesh();
            let VertexAttributeValues::Float32x3(positions) =
                mesh.attribute(Mesh::ATTRIBUTE_POSITION).unwrap()
            else {
                panic!("positions");
            };
            let VertexAttributeValues::Float32x3(normals) =
                mesh.attribute(Mesh::ATTRIBUTE_NORMAL).unwrap()
            else {
                panic!("normals");
            };
            assert_eq!(positions.len(), normals.len());
            assert!(positions.iter().flatten().all(|x| x.is_finite()));
            for (position, normal) in positions.iter().zip(normals) {
                let normal = Vec3::from_array(*normal);
                assert!(normal.is_finite());
                if shape == VehicleShape::Dart && normal == Vec3::ZERO {
                    // Bevy deliberately uses zero at the singular cone apex; preserve Classic exactly.
                    assert!((position[1] - 1.25).abs() < 1e-5);
                } else {
                    assert!((normal.length() - 1.0).abs() < 1e-5);
                }
            }
            let min_y = positions.iter().map(|p| p[1]).fold(f32::INFINITY, f32::min);
            let max_y = positions
                .iter()
                .map(|p| p[1])
                .fold(f32::NEG_INFINITY, f32::max);
            assert!((max_y - min_y - 2.5).abs() < 1e-5);
            let nose: Vec<_> = positions.iter().filter(|p| p[1] == max_y).collect();
            assert!(nose.iter().all(|p| p[0].abs() < 1e-5 && p[2].abs() <= 0.1));
            assert!((nose.iter().map(|p| p[2]).sum::<f32>() / nose.len() as f32).abs() < 1e-5);
            let indices: Vec<_> = mesh.indices().unwrap().iter().collect();
            assert_eq!(indices.len() / 3, triangles, "{shape:?}");
            for &[a, b, c] in indices.as_chunks::<3>().0 {
                let geometric = (Vec3::from_array(positions[b]) - Vec3::from_array(positions[a]))
                    .cross(Vec3::from_array(positions[c]) - Vec3::from_array(positions[a]))
                    .normalize();
                assert!(geometric.is_finite());
                for i in [a, b, c] {
                    if Vec3::from_array(normals[i]) == Vec3::ZERO {
                        continue;
                    }
                    assert!(
                        geometric.dot(Vec3::from_array(normals[i])) > 0.5,
                        "{shape:?}"
                    );
                }
            }
        }
    }

    #[test]
    fn instance_speed_and_reserved_group_round_trip_without_stride_growth() {
        assert_eq!(size_of::<crate::instancing::InstanceData>(), 32);
        for group in [0, 1, 7, 63, 127] {
            for speed in [0.0, 0.001, 0.1, 0.3, 0.5, 0.9999, 1.0] {
                let packed = pack_speed_group(speed, group);
                assert!(packed.is_normal() && (1.0..2.0).contains(&packed));
                let (actual, actual_group) = unpack_speed_group(packed);
                assert_eq!(actual_group, group);
                assert!((actual - speed).abs() <= 0.5 / 65535.0 + f32::EPSILON);
            }
        }
        assert_eq!(unpack_speed_group(pack_speed_group(-1.0, 0)).0, 0.0);
        assert_eq!(unpack_speed_group(pack_speed_group(2.0, 0)).0, 1.0);
        assert_eq!(unpack_speed_group(pack_speed_group(f32::NAN, 0)).0, 0.0);
    }
}
