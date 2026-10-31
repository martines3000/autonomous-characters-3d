//! Bounded static glTF import. Background jobs never mutate the running simulation.
use std::{
    fs::File,
    io::Read,
    path::{Path, PathBuf},
};

use bevy::{
    asset::RenderAssetUsages,
    gltf::gltf::{self, accessor::DataType, json::Value, mesh::Mode, Semantic},
    mesh::Indices,
    prelude::*,
    render::render_resource::PrimitiveTopology,
    tasks::{futures::check_ready, IoTaskPool, Task},
};
use bevy_egui::egui;

use crate::{
    scene::Appearance,
    vehicle_appearance::{ColorMode, VehicleShape},
};

const MAX_TRIANGLES: usize = 100_000;
const MAX_VERTICES: usize = MAX_TRIANGLES * 3;
const MAX_BYTES: usize = 64 * 1024 * 1024;
const MAX_NODES: usize = 4096;
const MAX_DEPTH: usize = 128;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ModelAxis {
    X,
    NegX,
    Y,
    NegY,
    Z,
    NegZ,
}
impl ModelAxis {
    pub const ALL: [Self; 6] = [
        Self::X,
        Self::NegX,
        Self::Y,
        Self::NegY,
        Self::Z,
        Self::NegZ,
    ];
    pub fn name(self) -> &'static str {
        match self {
            Self::X => "+X",
            Self::NegX => "-X",
            Self::Y => "+Y",
            Self::NegY => "-Y",
            Self::Z => "+Z",
            Self::NegZ => "-Z",
        }
    }
    pub fn parse(value: &str) -> Result<Self, String> {
        match value {
            "x" | "+x" => Ok(Self::X),
            "-x" => Ok(Self::NegX),
            "y" | "+y" => Ok(Self::Y),
            "-y" => Ok(Self::NegY),
            "z" | "+z" => Ok(Self::Z),
            "-z" => Ok(Self::NegZ),
            _ => Err(format!(
                "invalid model axis '{value}'; expected ±x, ±y or ±z"
            )),
        }
    }
    fn vector(self) -> Vec3 {
        match self {
            Self::X => Vec3::X,
            Self::NegX => Vec3::NEG_X,
            Self::Y => Vec3::Y,
            Self::NegY => Vec3::NEG_Y,
            Self::Z => Vec3::Z,
            Self::NegZ => Vec3::NEG_Z,
        }
    }
    pub fn perpendicular(self, other: Self) -> bool {
        self.vector().dot(other.vector()) == 0.0
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ModelSettings {
    pub forward: ModelAxis,
    pub up: ModelAxis,
    pub scale: f32,
}
impl Default for ModelSettings {
    fn default() -> Self {
        Self {
            forward: ModelAxis::Z,
            up: ModelAxis::Y,
            scale: 1.0,
        }
    }
}
impl ModelSettings {
    pub fn set_forward(&mut self, axis: ModelAxis) {
        self.forward = axis;
        if !axis.perpendicular(self.up) {
            self.up = ModelAxis::ALL
                .into_iter()
                .find(|up| axis.perpendicular(*up))
                .unwrap();
        }
    }
    fn rotation(self) -> Result<Mat3, String> {
        if !self.forward.perpendicular(self.up) {
            return Err(
                "Invalid axes at model.up; expected perpendicular forward and up axes".into(),
            );
        }
        let forward = self.forward.vector();
        let up = self.up.vector();
        // Source right/forward/up become local +X/+Y/+Z without a reflection.
        Ok(Mat3::from_cols(forward.cross(up), forward, up).transpose())
    }
}

#[derive(Debug)]
struct ImportedModel {
    positions: Vec<[f32; 3]>,
    normals: Vec<[f32; 3]>,
    colors: Vec<[f32; 4]>,
    primitives: usize,
}
impl ImportedModel {
    fn bounds(&self) -> (Vec3, Vec3) {
        self.positions.iter().fold(
            (Vec3::splat(f32::INFINITY), Vec3::splat(f32::NEG_INFINITY)),
            |(min, max), p| (min.min(Vec3::from_array(*p)), max.max(Vec3::from_array(*p))),
        )
    }
    fn mesh(mut self, settings: ModelSettings) -> Result<Mesh, String> {
        let rotation = settings.rotation()?;
        if !settings.scale.is_finite() || !(0.1..=5.0).contains(&settings.scale) {
            return Err("Invalid scale at model.scale; expected 0.1..=5".into());
        }
        let (min, max) = self.bounds();
        let length = (max - min).dot(settings.forward.vector().abs());
        let factor = 2.5 / length * settings.scale;
        if !length.is_finite() || length <= 0.0 || !factor.is_finite() {
            return Err(
                "Invalid bounds at model.forward; expected a finite nonzero forward extent".into(),
            );
        }
        let center = min * 0.5 + max * 0.5;
        for position in &mut self.positions {
            let value = (rotation * (Vec3::from_array(*position) - center)) * factor;
            if !value.is_finite() {
                return Err("Non-finite position at model.normalization".into());
            }
            *position = value.to_array();
        }
        for normal in &mut self.normals {
            *normal = (rotation * Vec3::from_array(*normal)).to_array();
        }
        let indices = (0..self.positions.len() as u32).collect();
        Ok(Mesh::new(
            PrimitiveTopology::TriangleList,
            RenderAssetUsages::default(),
        )
        .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, self.positions)
        .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, self.normals)
        .with_inserted_attribute(Mesh::ATTRIBUTE_COLOR, self.colors)
        .with_inserted_indices(Indices::U32(indices)))
    }
}

fn unsupported(feature: &str, path: &str, expected: &str) -> String {
    format!("Unsupported {feature} at {path}; expected {expected}")
}

// Inspect the raw JSON first: gltf's validation otherwise rejects unknown required extensions
// before we can name the unsupported feature. Never use unchecked gltf object access.
fn preflight(value: &Value) -> Result<(), String> {
    if let Some(extension) = value["extensionsRequired"]
        .as_array()
        .and_then(|v| v.first())
    {
        return Err(unsupported(
            &format!("required extension {extension}"),
            "extensionsRequired[0]",
            "no required extensions",
        ));
    }
    for (key, feature) in [("skins", "skins"), ("animations", "animations")] {
        if value[key].as_array().is_some_and(|v| !v.is_empty()) {
            return Err(unsupported(
                feature,
                &format!("{key}[0]"),
                "static unskinned geometry (Stage 6 is deferred)",
            ));
        }
    }
    fn walk(value: &Value, path: &str, depth: usize) -> Result<(), String> {
        if depth > MAX_DEPTH {
            return Err(format!(
                "Limit exceeded at {path}; expected JSON depth <= {MAX_DEPTH}"
            ));
        }
        match value {
            Value::Object(object) => {
                for (key, child) in object {
                    if key == "extras" {
                        continue;
                    }
                    let next = if path.is_empty() {
                        key.clone()
                    } else {
                        format!("{path}.{key}")
                    };
                    match key.as_str() {
                        "extensions" if child.as_object().is_some_and(|o| !o.is_empty()) => {
                            let name = child.as_object().unwrap().keys().next().unwrap();
                            return Err(unsupported(
                                &format!("extension {name}"),
                                &format!("{next}.{name}"),
                                "uncompressed core glTF",
                            ));
                        }
                        "sparse" => {
                            return Err(unsupported("sparse accessor", &next, "dense accessors"))
                        }
                        "targets" if path.contains(".primitives[") => {
                            return Err(unsupported(
                                "morph targets",
                                &next,
                                "static geometry (Stage 6 is deferred)",
                            ))
                        }
                        "weights" if path.starts_with("nodes[") || path.starts_with("meshes[") => {
                            return Err(unsupported("morph weights", &next, "static geometry"))
                        }
                        "skin" if path.starts_with("nodes[") => {
                            return Err(unsupported("skin", &next, "unskinned nodes"))
                        }
                        key if path.starts_with("materials[") && key.ends_with("Texture") => {
                            return Err(unsupported(
                                "texture map",
                                &next,
                                "untextured materials (Stage 6 is deferred)",
                            ))
                        }
                        "alphaMode" if child != "OPAQUE" => {
                            return Err(unsupported("alpha mode", &next, "OPAQUE"))
                        }
                        "doubleSided" if child == true => {
                            return Err(unsupported(
                                "double-sided material",
                                &next,
                                "single-sided materials",
                            ))
                        }
                        "emissiveFactor"
                            if child
                                .as_array()
                                .is_some_and(|a| a.iter().any(|x| x.as_f64() != Some(0.0))) =>
                        {
                            return Err(unsupported("emission", &next, "zero emissiveFactor"))
                        }
                        _ => (),
                    }
                    walk(child, &next, depth + 1)?;
                }
            }
            Value::Array(array) => {
                for (i, child) in array.iter().enumerate() {
                    walk(child, &format!("{path}[{i}]"), depth + 1)?;
                }
            }
            _ => (),
        }
        Ok(())
    }
    walk(value, "", 0)
}

fn read_file(path: &Path, remaining: &mut usize) -> Result<Vec<u8>, String> {
    let mut bytes = Vec::new();
    File::open(path)
        .and_then(|file| file.take(*remaining as u64 + 1).read_to_end(&mut bytes))
        .map_err(|e| format!("Cannot read {}: {e}", path.display()))?;
    if bytes.len() > *remaining {
        return Err(format!(
            "Limit exceeded at {}; expected total input <= 64 MiB",
            path.display()
        ));
    }
    *remaining -= bytes.len();
    Ok(bytes)
}

fn load(path: &Path, settings: ModelSettings) -> Result<(Mesh, usize, usize, usize), String> {
    let mut remaining = MAX_BYTES;
    let bytes = read_file(path, &mut remaining)?;
    let model = parse(&bytes, |uri| {
        read_file(
            &path.parent().unwrap_or(Path::new(".")).join(uri),
            &mut remaining,
        )
    })?;
    let vertices = model.positions.len();
    let primitives = model.primitives;
    Ok((model.mesh(settings)?, vertices, vertices / 3, primitives))
}

fn parse(
    bytes: &[u8],
    mut sidecar: impl FnMut(&str) -> Result<Vec<u8>, String>,
) -> Result<ImportedModel, String> {
    if bytes.len() > MAX_BYTES {
        return Err("Limit exceeded at model; expected input <= 64 MiB".into());
    }
    let json = if bytes.starts_with(b"glTF") {
        gltf::Glb::from_slice(bytes)
            .map_err(|e| format!("Malformed GLB at model: {e}"))?
            .json
    } else {
        std::borrow::Cow::Borrowed(bytes)
    };
    let value: Value = gltf::json::deserialize::from_slice(&json)
        .map_err(|e| format!("Malformed JSON at model: {e}"))?;
    preflight(&value)?;
    let gltf =
        gltf::Gltf::from_slice(bytes).map_err(|e| format!("Malformed glTF at model: {e}"))?;
    let mut total = bytes.len();
    let mut buffers = Vec::new();
    for buffer in gltf.buffers() {
        let path = format!("buffers[{}]", buffer.index());
        if buffer.length() > MAX_BYTES {
            return Err(format!(
                "Limit exceeded at {path}.byteLength; expected <= 64 MiB"
            ));
        }
        let mut data = match buffer.source() {
            gltf::buffer::Source::Bin => gltf
                .blob
                .clone()
                .ok_or_else(|| format!("Missing GLB BIN chunk at {path}"))?,
            gltf::buffer::Source::Uri(uri) => {
                if uri.contains(':')
                    || uri.starts_with('/')
                    || uri.contains('\\')
                    || uri.contains('%')
                {
                    return Err(unsupported(
                        "buffer URI",
                        &format!("{path}.uri"),
                        "local relative sidecar paths without URI schemes or percent encoding",
                    ));
                }
                sidecar(uri).map_err(|e| format!("Missing resource at {path}.uri ({uri}): {e}"))?
            }
        };
        total = total
            .checked_add(data.len())
            .ok_or("Input size overflow at buffers")?;
        if total > MAX_BYTES {
            return Err(format!(
                "Limit exceeded at {path}; expected total input/buffers <= 64 MiB"
            ));
        }
        if data.len() < buffer.length() {
            return Err(format!(
                "Malformed buffer at {path}; byteLength {} exceeds {} bytes",
                buffer.length(),
                data.len()
            ));
        }
        data.truncate(buffer.length());
        buffers.push(data);
    }
    validate_accessors(&gltf, &buffers)?;
    validate_hierarchy(&gltf)?;
    let scene = gltf
        .default_scene()
        .or_else(|| {
            if gltf.scenes().len() == 1 {
                gltf.scenes().next()
            } else {
                None
            }
        })
        .ok_or(
            "Ambiguous or missing scene at scenes; expected default scene or exactly one scene",
        )?;
    let mut model = ImportedModel {
        positions: Vec::new(),
        normals: Vec::new(),
        colors: Vec::new(),
        primitives: 0,
    };
    let mut pending: Vec<_> = scene
        .nodes()
        .map(|node| (node, Mat4::IDENTITY, 0))
        .collect();
    let mut seen = vec![false; gltf.nodes().len()];
    while let Some((node, parent, depth)) = pending.pop() {
        if depth > MAX_DEPTH {
            return Err(format!(
                "Limit exceeded at nodes[{}]; expected hierarchy depth <= {MAX_DEPTH}",
                node.index()
            ));
        }
        if seen[node.index()] {
            return Err(format!(
                "Repeated node at nodes[{}]; expected a tree",
                node.index()
            ));
        }
        seen[node.index()] = true;
        let transform = parent * Mat4::from_cols_array_2d(&node.transform().matrix());
        let linear = Mat3::from_mat4(transform);
        let determinant = linear.determinant();
        if !transform.is_finite()
            || !determinant.is_finite()
            || determinant == 0.0
            || transform.row(3) != Vec4::W
        {
            return Err(format!(
                "Invalid transform at nodes[{}]; expected finite nonsingular affine TRS/matrix",
                node.index()
            ));
        }
        if let Some(mesh) = node.mesh() {
            for primitive in mesh.primitives() {
                append_primitive(&mut model, mesh.index(), primitive, transform, &buffers)?;
            }
        }
        pending.extend(node.children().map(|child| (child, transform, depth + 1)));
    }
    if model.positions.is_empty() {
        return Err("Empty model at scenes; expected at least one triangle".into());
    }
    Ok(model)
}

fn validate_accessors(gltf: &gltf::Gltf, buffers: &[Vec<u8>]) -> Result<(), String> {
    for view in gltf.views() {
        if view
            .offset()
            .checked_add(view.length())
            .is_none_or(|end| end > buffers[view.buffer().index()].len())
        {
            return Err(format!(
                "Malformed buffer view at bufferViews[{}]; range exceeds buffer",
                view.index()
            ));
        }
    }
    for accessor in gltf.accessors() {
        let path = format!("accessors[{}]", accessor.index());
        if accessor.count() == 0 || accessor.count() > MAX_VERTICES {
            return Err(format!(
                "Limit exceeded at {path}.count; expected 1..={MAX_VERTICES}"
            ));
        }
        let view = accessor
            .view()
            .ok_or_else(|| format!("Missing buffer view at {path}.bufferView"))?;
        let size = accessor.size();
        let stride = view.stride().unwrap_or(size);
        let end = (accessor.count() - 1)
            .checked_mul(stride)
            .and_then(|n| n.checked_add(accessor.offset()))
            .and_then(|n| n.checked_add(size));
        if stride < size || end.is_none_or(|end| end > view.length()) {
            return Err(format!(
                "Malformed accessor at {path}; range/stride exceeds buffer view"
            ));
        }
        if accessor.data_type() == DataType::F32 {
            let data = &buffers[view.buffer().index()];
            for i in 0..accessor.count() {
                let start = view.offset() + accessor.offset() + i * stride;
                for bytes in data[start..start + size].as_chunks::<4>().0 {
                    if !f32::from_le_bytes(*bytes).is_finite() {
                        return Err(format!(
                            "Non-finite value at {path}[{i}]; expected finite float32"
                        ));
                    }
                }
            }
        }
    }
    Ok(())
}

fn validate_hierarchy(gltf: &gltf::Gltf) -> Result<(), String> {
    if gltf.nodes().len() > MAX_NODES {
        return Err(format!(
            "Limit exceeded at nodes; expected <= {MAX_NODES} nodes"
        ));
    }
    fn visit(node: gltf::Node, states: &mut [u8], depth: usize) -> Result<(), String> {
        let path = format!("nodes[{}]", node.index());
        if depth > MAX_DEPTH {
            return Err(format!(
                "Limit exceeded at {path}; expected hierarchy depth <= {MAX_DEPTH}"
            ));
        }
        if states[node.index()] == 1 {
            return Err(format!(
                "Cyclic hierarchy at {path}; expected an acyclic tree"
            ));
        }
        if states[node.index()] == 2 {
            return Ok(());
        }
        states[node.index()] = 1;
        for child in node.children() {
            visit(child, states, depth + 1)?;
        }
        states[node.index()] = 2;
        Ok(())
    }
    let mut states = vec![0; gltf.nodes().len()];
    for node in gltf.nodes() {
        visit(node, &mut states, 0)?;
    }
    Ok(())
}

fn append_primitive(
    model: &mut ImportedModel,
    mesh_index: usize,
    primitive: gltf::Primitive,
    transform: Mat4,
    buffers: &[Vec<u8>],
) -> Result<(), String> {
    let path = format!("meshes[{}].primitives[{}]", mesh_index, primitive.index());
    if primitive.mode() != Mode::Triangles {
        return Err(unsupported(
            "primitive mode",
            &format!("{path}.mode"),
            "TRIANGLES (4)",
        ));
    }
    for (semantic, accessor) in primitive.attributes() {
        let supported = match semantic {
            Semantic::Positions | Semantic::Normals => {
                accessor.data_type() == DataType::F32
                    && accessor.dimensions() == gltf::accessor::Dimensions::Vec3
                    && !accessor.normalized()
            }
            Semantic::Colors(0) => {
                matches!(
                    accessor.dimensions(),
                    gltf::accessor::Dimensions::Vec3 | gltf::accessor::Dimensions::Vec4
                ) && ((accessor.data_type() == DataType::F32 && !accessor.normalized())
                    || (matches!(accessor.data_type(), DataType::U8 | DataType::U16)
                        && accessor.normalized()))
            }
            Semantic::Tangents | Semantic::TexCoords(0) => true,
            _ => false,
        };
        if !supported {
            let semantic = semantic.to_string();
            return Err(unsupported(
                &format!("attribute {semantic}"),
                &format!("{path}.attributes.{semantic}"),
                "float32 POSITION/NORMAL, optional COLOR_0; UV0/TANGENT ignored",
            ));
        }
    }
    if let Some(indices) = primitive.indices() {
        if !matches!(
            indices.data_type(),
            DataType::U8 | DataType::U16 | DataType::U32
        ) || indices.dimensions() != gltf::accessor::Dimensions::Scalar
            || indices.normalized()
        {
            return Err(unsupported(
                "index format",
                &format!("{path}.indices"),
                "u8/u16/u32 scalar indices",
            ));
        }
    }
    let reader = primitive.reader(|buffer| Some(buffers[buffer.index()].as_slice()));
    let positions: Vec<_> = reader
        .read_positions()
        .ok_or_else(|| format!("Missing POSITION at {path}.attributes.POSITION"))?
        .collect();
    let normals: Option<Vec<_>> = reader.read_normals().map(Iterator::collect);
    let colors: Option<Vec<_>> = reader.read_colors(0).map(|c| c.into_rgba_f32().collect());
    for (name, count) in [
        ("NORMAL", normals.as_ref().map(Vec::len)),
        ("COLOR_0", colors.as_ref().map(Vec::len)),
    ] {
        if count.is_some_and(|count| count != positions.len()) {
            return Err(format!(
                "Mismatched attribute count at {path}.attributes.{name}; expected {}",
                positions.len()
            ));
        }
    }
    let indices: Vec<_> = reader
        .read_indices()
        .map(|i| i.into_u32().collect())
        .unwrap_or_else(|| (0..positions.len() as u32).collect());
    if indices.is_empty() || !indices.len().is_multiple_of(3) {
        return Err(format!(
            "Invalid triangle count at {path}.indices; expected nonempty triples"
        ));
    }
    if model.positions.len() + indices.len() > MAX_VERTICES {
        return Err(format!("Limit exceeded at {path}; expected <= {MAX_TRIANGLES} triangles / {MAX_VERTICES} expanded vertices"));
    }
    if indices.iter().any(|i| *i as usize >= positions.len()) {
        return Err(format!(
            "Out-of-range index at {path}.indices; expected index < {}",
            positions.len()
        ));
    }
    let factor = Vec4::from_array(
        primitive
            .material()
            .pbr_metallic_roughness()
            .base_color_factor(),
    );
    if !factor.is_finite() || factor.min_element() < 0.0 || factor.max_element() > 1.0 {
        return Err(format!(
            "Invalid base color at {path}.material; expected finite factors in 0..=1"
        ));
    }
    let linear = Mat3::from_mat4(transform);
    let normal_matrix = linear.inverse().transpose();
    for triangle in indices.as_chunks::<3>().0 {
        let mut triangle = *triangle;
        if linear.determinant() < 0.0 {
            triangle.swap(1, 2);
        }
        let points =
            triangle.map(|i| transform.transform_point3(Vec3::from_array(positions[i as usize])));
        if points.iter().any(|p| !p.is_finite()) {
            return Err(format!(
                "Non-finite transformed position at {path}.attributes.POSITION"
            ));
        }
        // Exporters often leave zero-area triangles; they draw nothing, so skip them.
        let Some(flat) = (points[1] - points[0])
            .cross(points[2] - points[0])
            .try_normalize()
            .filter(|n| n.is_finite())
        else {
            continue;
        };
        for (i, point) in triangle.into_iter().zip(points) {
            let normal = if let Some(normals) = &normals {
                (normal_matrix * Vec3::from_array(normals[i as usize])).try_normalize()
                    .filter(|n| n.is_finite()).ok_or_else(|| format!("Invalid normal at {path}.attributes.NORMAL; expected finite nonzero normals"))?
            } else {
                flat
            };
            let color = colors
                .as_ref()
                .map_or(Vec4::ONE, |c| Vec4::from_array(c[i as usize]));
            if !color.is_finite() || color.min_element() < 0.0 || color.max_element() > 1.0 {
                return Err(format!("Invalid vertex color at {path}.attributes.COLOR_0; expected finite values in 0..=1"));
            }
            model.positions.push(point.to_array());
            model.normals.push(normal.to_array());
            model.colors.push((color * factor).to_array());
        }
    }
    model.primitives += 1;
    Ok(())
}

type LoadResult = Result<(Mesh, usize, usize, usize), String>;
struct ModelJob {
    id: u64,
    path: PathBuf,
    task: Task<LoadResult>,
}

#[derive(Resource)]
pub struct VehicleModels {
    pub settings: ModelSettings,
    pub path_input: String,
    pub status: String,
    pub current: Option<Handle<Mesh>>,
    current_path: Option<PathBuf>,
    paths: Vec<PathBuf>,
    scan: Option<Task<Result<Vec<PathBuf>, String>>>,
    scan_status: String,
    pending: Option<ModelJob>,
    generation: u64,
}
impl Default for VehicleModels {
    fn default() -> Self {
        Self {
            settings: default(),
            path_input: "assets/models/glider.glb".into(),
            status: "Choose a local GLB/glTF model.".into(),
            current: None,
            current_path: None,
            paths: Vec::new(),
            scan: None,
            scan_status: String::new(),
            pending: None,
            generation: 0,
        }
    }
}
fn root() -> PathBuf {
    PathBuf::from(
        std::env::var_os("CARGO_MANIFEST_DIR").unwrap_or_else(|| env!("CARGO_MANIFEST_DIR").into()),
    )
}
impl VehicleModels {
    pub fn request(&mut self, path: PathBuf) {
        self.generation = self.generation.wrapping_add(1);
        self.path_input = path.display().to_string();
        let path = root().join(path);
        let settings = self.settings;
        self.status = format!("Loading {}…", path.display());
        let task_path = path.clone();
        self.pending = Some(ModelJob {
            id: self.generation,
            path,
            task: IoTaskPool::get().spawn(async move { load(&task_path, settings) }),
        });
    }
    pub fn cancel(&mut self) {
        self.generation = self.generation.wrapping_add(1);
        if self.pending.take().is_some() {
            self.status = "Model load cancelled; selection preserved.".into();
        }
    }
    pub fn reload(&mut self) {
        let path = self
            .pending
            .as_ref()
            .map(|job| &job.path)
            .or(self.current_path.as_ref())
            .cloned();
        if let Some(path) = path {
            self.request(path);
        }
    }
    fn rescan(&mut self) {
        self.scan_status = "Scanning assets/models/…".into();
        self.scan = Some(IoTaskPool::get().spawn(async {
            let directory = root().join("assets/models");
            let entries = match std::fs::read_dir(directory) {
                Ok(entries) => entries,
                Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Ok(Vec::new()),
                Err(e) => return Err(format!("Cannot scan assets/models/: {e}")),
            };
            let mut paths = Vec::new();
            for entry in entries {
                let entry = entry.map_err(|e| format!("Cannot scan assets/models/: {e}"))?;
                let path = entry.path();
                if path.is_file()
                    && path.extension().and_then(|e| e.to_str()).is_some_and(|e| {
                        e.eq_ignore_ascii_case("glb") || e.eq_ignore_ascii_case("gltf")
                    })
                {
                    paths.push(path);
                }
            }
            paths.sort();
            Ok(paths)
        }));
    }
    fn finish(
        &mut self,
        id: u64,
        path: PathBuf,
        result: LoadResult,
        meshes: &mut Assets<Mesh>,
        appearance: &mut Appearance,
    ) {
        if id != self.generation {
            return;
        }
        match result {
            Ok((mesh, vertices, triangles, primitives)) => {
                self.current = Some(meshes.add(mesh));
                self.current_path = Some(path.clone());
                appearance.vehicles.shape = VehicleShape::Custom;
                appearance.vehicles.color_mode = ColorMode::ModelColors;
                self.status = format!("Loaded {}: {triangles} triangles, {vertices} vertices, {primitives} primitives", path.file_name().unwrap_or_default().to_string_lossy());
            }
            Err(error) => {
                self.status = format!(
                    "Error: {error}. Previous selection kept. Fix file/path and Load again."
                )
            }
        }
        info!("vehicle model: {}", self.status);
    }
}

pub struct VehicleModelPlugin;
impl Plugin for VehicleModelPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<VehicleModels>()
            .add_systems(Startup, |mut models: ResMut<VehicleModels>| models.rescan())
            .add_systems(Update, poll_models);
    }
}
fn poll_models(
    mut models: ResMut<VehicleModels>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut appearance: ResMut<Appearance>,
) {
    if let Some(result) = models.scan.as_mut().and_then(check_ready) {
        models.scan = None;
        match result {
            Ok(paths) => {
                models.scan_status = format!("{} local models", paths.len());
                models.paths = paths;
            }
            Err(error) => models.scan_status = error,
        }
    }
    if let Some(result) = models
        .pending
        .as_mut()
        .and_then(|job| check_ready(&mut job.task))
    {
        let job = models.pending.take().unwrap();
        models.finish(job.id, job.path, result, &mut meshes, &mut appearance);
    }
}

pub fn ui(
    ui: &mut egui::Ui,
    models: &mut VehicleModels,
    appearance: &mut crate::vehicle_appearance::VehicleAppearance,
) {
    let selected = if appearance.shape == VehicleShape::Custom {
        models
            .current_path
            .as_ref()
            .and_then(|p| p.file_name())
            .map_or("Custom model".into(), |p| p.to_string_lossy().into_owned())
    } else {
        appearance.shape.name().into()
    };
    egui::ComboBox::from_label("Shape / model")
        .selected_text(selected)
        .show_ui(ui, |ui| {
            for shape in VehicleShape::ALL {
                if ui
                    .selectable_value(&mut appearance.shape, shape, shape.name())
                    .clicked()
                {
                    models.cancel();
                }
            }
            let mut chosen = None;
            for path in &models.paths {
                let name = path.file_name().unwrap_or_default().to_string_lossy();
                if ui
                    .selectable_label(
                        appearance.shape == VehicleShape::Custom
                            && models.current_path.as_ref() == Some(path),
                        name,
                    )
                    .clicked()
                {
                    chosen = Some(path.clone());
                }
            }
            if let Some(path) = chosen {
                models.request(path);
            }
            if let Some(path) = models.current_path.clone() {
                if !models.paths.contains(&path)
                    && ui
                        .selectable_label(
                            appearance.shape == VehicleShape::Custom,
                            path.display().to_string(),
                        )
                        .clicked()
                {
                    models.request(path);
                }
            }
        });
    ui.horizontal(|ui| {
        ui.add(egui::TextEdit::singleline(&mut models.path_input).desired_width(220.0));
        if ui.button("Load").clicked() {
            models.request(PathBuf::from(&models.path_input));
        }
        if ui.button("Rescan").clicked() {
            models.rescan();
        }
    });
    ui.small(&models.scan_status);
    let before = models.settings;
    ui.horizontal(|ui| {
        egui::ComboBox::from_label("Forward")
            .selected_text(models.settings.forward.name())
            .show_ui(ui, |ui| {
                for axis in ModelAxis::ALL {
                    if ui
                        .selectable_label(models.settings.forward == axis, axis.name())
                        .clicked()
                    {
                        models.settings.set_forward(axis);
                    }
                }
            });
        egui::ComboBox::from_label("Up")
            .selected_text(models.settings.up.name())
            .show_ui(ui, |ui| {
                for axis in ModelAxis::ALL
                    .into_iter()
                    .filter(|axis| models.settings.forward.perpendicular(*axis))
                {
                    ui.selectable_value(&mut models.settings.up, axis, axis.name());
                }
            });
    });
    ui.add(egui::Slider::new(&mut models.settings.scale, 0.1..=5.0).text("Model scale"));
    ui.small("Source forward becomes local +Y; source up becomes local +Z.");
    if models.settings != before {
        models.reload();
    }
    if models.status.starts_with("Error:") {
        ui.colored_label(egui::Color32::LIGHT_RED, &models.status);
    } else {
        ui.label(&models.status);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bevy::{mesh::VertexAttributeValues, render::render_resource::encase};

    fn json(text: &str) -> Value {
        gltf::json::deserialize::from_str(text).unwrap()
    }
    fn fixture() -> (Value, Vec<u8>) {
        let value = json(
            r#"{
            "asset":{"version":"2.0"}, "scene":0, "scenes":[{"nodes":[0]}],
            "nodes":[{"mesh":0}], "meshes":[{"primitives":[{"attributes":{"POSITION":0},"material":0}]}],
            "materials":[{"pbrMetallicRoughness":{"baseColorFactor":[0.5,0.25,1.0,1.0]}}],
            "buffers":[{"byteLength":36,"uri":"mesh.bin"}],
            "bufferViews":[{"buffer":0,"byteLength":36}],
            "accessors":[{"bufferView":0,"componentType":5126,"count":3,"type":"VEC3","min":[-1,0,-1],"max":[1,1,2]}]
        }"#,
        );
        let bytes = [-1.0_f32, 0.0, -1.0, 1.0, 0.0, -1.0, 0.0, 1.0, 2.0]
            .into_iter()
            .flat_map(f32::to_le_bytes)
            .collect();
        (value, bytes)
    }
    fn from_fixture(value: &Value, bytes: &[u8]) -> Result<ImportedModel, String> {
        parse(&gltf::json::serialize::to_vec(value).unwrap(), |uri| {
            assert_eq!(uri, "mesh.bin");
            Ok(bytes.to_vec())
        })
    }
    fn glb(value: &Value, bytes: &[u8]) -> Vec<u8> {
        let mut value = value.clone();
        value["buffers"][0].as_object_mut().unwrap().remove("uri");
        let mut json = gltf::json::serialize::to_vec(&value).unwrap();
        json.resize(json.len().next_multiple_of(4), b' ');
        let mut bin = bytes.to_vec();
        bin.resize(bin.len().next_multiple_of(4), 0);
        let length = (12 + 8 + json.len() + 8 + bin.len()) as u32;
        let mut result = b"glTF".to_vec();
        result.extend(2_u32.to_le_bytes());
        result.extend(length.to_le_bytes());
        result.extend((json.len() as u32).to_le_bytes());
        result.extend(b"JSON");
        result.extend(json);
        result.extend((bin.len() as u32).to_le_bytes());
        result.extend(b"BIN\0");
        result.extend(bin);
        result
    }
    fn positions(mesh: &Mesh) -> &[[f32; 3]] {
        let VertexAttributeValues::Float32x3(p) = mesh.attribute(Mesh::ATTRIBUTE_POSITION).unwrap()
        else {
            panic!("positions");
        };
        p
    }

    #[test]
    fn example_has_expected_parts_counts_bounds_normals_and_linear_colors() {
        let bytes = include_bytes!("../assets/models/glider.glb");
        let model = parse(bytes, |_| panic!("embedded buffers")).unwrap();
        assert_eq!(
            (
                model.primitives,
                model.positions.len(),
                model.positions.len() / 3
            ),
            (2, 72, 24)
        );
        let (min, max) = model.bounds();
        assert!(min.distance(Vec3::new(-1.3, -0.2, -1.0)) < 1e-6);
        assert!(max.distance(Vec3::new(1.3, 0.2, 2.0)) < 1e-6);
        assert!(model.colors.contains(&[1.0, 0.35, 0.025, 1.0]));
        assert!(model.colors.contains(&[0.025, 0.45, 1.0, 1.0]));
        for normal in &model.normals {
            assert!((Vec3::from_array(*normal).length() - 1.0).abs() < 1e-5);
        }
        let mesh = model.mesh(default()).unwrap();
        assert_eq!(mesh.count_vertices(), 72);
        for (_, values) in mesh.attributes() {
            match values {
                VertexAttributeValues::Float32x3(values) => {
                    assert!(values.iter().flatten().all(|v| v.is_finite()))
                }
                VertexAttributeValues::Float32x4(values) => {
                    assert!(values.iter().flatten().all(|v| v.is_finite()))
                }
                _ => panic!("unexpected attribute"),
            }
        }
    }

    #[test]
    fn every_axis_pair_centers_normalizes_and_preserves_handedness() {
        for forward in ModelAxis::ALL {
            for up in ModelAxis::ALL
                .into_iter()
                .filter(|up| forward.perpendicular(*up))
            {
                let settings = ModelSettings {
                    forward,
                    up,
                    scale: 1.0,
                };
                let rotation = settings.rotation().unwrap();
                assert_eq!(rotation * forward.vector(), Vec3::Y);
                assert_eq!(rotation * up.vector(), Vec3::Z);
                assert_eq!(rotation.determinant(), 1.0);
                let model = parse(
                    include_bytes!("../assets/models/glider.glb"),
                    |_| unreachable!(),
                )
                .unwrap();
                let mesh = model.mesh(settings).unwrap();
                let (min, max) = positions(&mesh).iter().fold(
                    (Vec3::splat(f32::INFINITY), Vec3::splat(f32::NEG_INFINITY)),
                    |(min, max), p| (min.min(Vec3::from_array(*p)), max.max(Vec3::from_array(*p))),
                );
                assert!((max.y - min.y - 2.5).abs() < 1e-5);
                assert!((min + max).length() < 1e-5);
            }
        }
        let (value, bytes) = fixture();
        let mesh = from_fixture(&value, &bytes)
            .unwrap()
            .mesh(ModelSettings {
                scale: 2.0,
                ..default()
            })
            .unwrap();
        let min = positions(&mesh)
            .iter()
            .map(|p| p[1])
            .fold(f32::INFINITY, f32::min);
        let max = positions(&mesh)
            .iter()
            .map(|p| p[1])
            .fold(f32::NEG_INFINITY, f32::max);
        assert!((max - min - 5.0).abs() < 1e-5);
    }

    #[test]
    fn sidecar_and_embedded_buffers_match_and_missing_normals_are_flat() {
        let (value, bytes) = fixture();
        let sidecar = from_fixture(&value, &bytes).unwrap();
        let embedded = parse(&glb(&value, &bytes), |_| panic!("sidecar")).unwrap();
        assert_eq!(embedded.positions, sidecar.positions);
        assert_eq!(embedded.colors, vec![[0.5, 0.25, 1.0, 1.0]; 3]);
        let normal = (Vec3::from_array(sidecar.positions[1])
            - Vec3::from_array(sidecar.positions[0]))
        .cross(Vec3::from_array(sidecar.positions[2]) - Vec3::from_array(sidecar.positions[0]))
        .normalize();
        assert!(sidecar
            .normals
            .iter()
            .all(|n| Vec3::from_array(*n).distance(normal) < 1e-6));
    }

    fn add_accessor(
        value: &mut Value,
        bytes: &mut Vec<u8>,
        data: &[u8],
        component: u32,
        dimensions: &str,
        count: usize,
    ) -> usize {
        let views = value["bufferViews"].as_array_mut().unwrap();
        let view = views.len();
        views.push(json(&format!(
            r#"{{"buffer":0,"byteOffset":{},"byteLength":{}}}"#,
            bytes.len(),
            data.len()
        )));
        bytes.extend(data);
        value["buffers"][0]["byteLength"] = Value::from(bytes.len());
        let accessors = value["accessors"].as_array_mut().unwrap();
        let index = accessors.len();
        accessors.push(json(&format!(r#"{{"bufferView":{view},"componentType":{component},"count":{count},"type":"{dimensions}"}}"#)));
        index
    }

    #[test]
    fn all_index_widths_merge_and_color_zero_multiplies_material_factor() {
        for (component, data) in [
            (5121, vec![0, 1, 2]),
            (
                5123,
                [0_u16, 1, 2]
                    .into_iter()
                    .flat_map(u16::to_le_bytes)
                    .collect(),
            ),
            (
                5125,
                [0_u32, 1, 2]
                    .into_iter()
                    .flat_map(u32::to_le_bytes)
                    .collect(),
            ),
        ] {
            let (mut value, mut bytes) = fixture();
            let color = [0.5_f32, 1.0, 0.25, 1.0]
                .repeat(3)
                .into_iter()
                .flat_map(f32::to_le_bytes)
                .collect::<Vec<_>>();
            let color = add_accessor(&mut value, &mut bytes, &color, 5126, "VEC4", 3);
            value["meshes"][0]["primitives"][0]["attributes"]["COLOR_0"] = Value::from(color);
            let indices = add_accessor(&mut value, &mut bytes, &data, component, "SCALAR", 3);
            value["meshes"][0]["primitives"][0]["indices"] = Value::from(indices);
            let primitive = value["meshes"][0]["primitives"][0].clone();
            value["meshes"][0]["primitives"]
                .as_array_mut()
                .unwrap()
                .push(primitive);
            let model = from_fixture(&value, &bytes).unwrap();
            assert_eq!(model.primitives, 2);
            assert_eq!(model.colors, vec![[0.25, 0.25, 0.25, 1.0]; 6]);
        }
        for (component, data) in [
            (5121, [255_u8, 128, 0, 255].repeat(3)),
            (
                5123,
                [65535_u16, 32768, 0, 65535]
                    .repeat(3)
                    .into_iter()
                    .flat_map(u16::to_le_bytes)
                    .collect(),
            ),
        ] {
            let (mut value, mut bytes) = fixture();
            let color = add_accessor(&mut value, &mut bytes, &data, component, "VEC4", 3);
            value["accessors"][color]["normalized"] = Value::Bool(true);
            value["meshes"][0]["primitives"][0]["attributes"]["COLOR_0"] = Value::from(color);
            let model = from_fixture(&value, &bytes).unwrap();
            assert_eq!(model.colors[0][0], 0.5);
            assert!((model.colors[0][1] - 0.125).abs() < 0.001);
        }
    }

    #[test]
    fn hierarchy_bakes_trs_matrix_inverse_normals_and_reflection_winding() {
        let (mut value, bytes) = fixture();
        value["nodes"] = json(
            r#"[{"translation":[3,5,7],"children":[1]}, {"scale":[-2,3,4],"children":[2]}, {"mesh":0,"matrix":[1,0,0,0,0,1,0,0,0,0,1,0,2,0,0,1]}]"#,
        );
        let model = from_fixture(&value, &bytes).unwrap();
        let parent = Mat4::from_translation(Vec3::new(3.0, 5.0, 7.0))
            * Mat4::from_scale(Vec3::new(-2.0, 3.0, 4.0))
            * Mat4::from_translation(Vec3::new(2.0, 0.0, 0.0));
        let original = from_fixture(&fixture().0, &bytes).unwrap();
        assert_eq!(
            Vec3::from_array(model.positions[0]),
            parent.transform_point3(Vec3::from_array(original.positions[0]))
        );
        assert_eq!(
            Vec3::from_array(model.positions[1]),
            parent.transform_point3(Vec3::from_array(original.positions[2]))
        );
        let normal =
            Mat3::from_mat4(parent).inverse().transpose() * Vec3::from_array(original.normals[0]);
        assert!(Vec3::from_array(model.normals[0]).distance(normal.normalize()) < 1e-6);
    }

    #[test]
    fn unsupported_features_name_feature_and_json_path() {
        let cases = [
            (
                "extensionsRequired",
                r#"["KHR_draco_mesh_compression"]"#,
                "required extension",
                "extensionsRequired[0]",
            ),
            (
                "extensionsRequired",
                r#"["EXT_meshopt_compression"]"#,
                "required extension",
                "extensionsRequired[0]",
            ),
            (
                "extensionsRequired",
                r#"["VENDOR_unknown"]"#,
                "required extension",
                "extensionsRequired[0]",
            ),
            ("skins", "[{}]", "skins", "skins[0]"),
            ("animations", "[{}]", "animations", "animations[0]"),
        ];
        for (key, text, feature, path) in cases {
            let (mut value, bytes) = fixture();
            value[key] = json(text);
            let error = from_fixture(&value, &bytes).unwrap_err();
            assert!(
                error.starts_with(&format!("Unsupported {feature}")) && error.contains(path),
                "{error}"
            );
        }
        for (key, text, feature, path) in [
            (
                "alphaMode",
                "\"BLEND\"",
                "alpha mode",
                "materials[0].alphaMode",
            ),
            (
                "alphaMode",
                "\"MASK\"",
                "alpha mode",
                "materials[0].alphaMode",
            ),
            (
                "doubleSided",
                "true",
                "double-sided",
                "materials[0].doubleSided",
            ),
            (
                "emissiveFactor",
                "[1,0,0]",
                "emission",
                "materials[0].emissiveFactor",
            ),
            (
                "normalTexture",
                "{}",
                "texture map",
                "materials[0].normalTexture",
            ),
            (
                "occlusionTexture",
                "{}",
                "texture map",
                "materials[0].occlusionTexture",
            ),
            (
                "emissiveTexture",
                "{}",
                "texture map",
                "materials[0].emissiveTexture",
            ),
        ] {
            let (mut value, bytes) = fixture();
            value["materials"][0][key] = json(text);
            let error = from_fixture(&value, &bytes).unwrap_err();
            assert!(error.contains(feature) && error.contains(path), "{error}");
        }
        for key in ["baseColorTexture", "metallicRoughnessTexture"] {
            let (mut value, bytes) = fixture();
            value["materials"][0]["pbrMetallicRoughness"][key] = json("{}");
            let error = from_fixture(&value, &bytes).unwrap_err();
            assert!(
                error.contains(&format!(
                    "texture map at materials[0].pbrMetallicRoughness.{key}"
                )),
                "{error}"
            );
        }
        for (pointer, key, text, expected) in [
            ("/accessors/0", "sparse", "{}", "sparse accessor at accessors[0].sparse"),
            ("/meshes/0/primitives/0", "targets", "[{}]", "morph targets at meshes[0].primitives[0].targets"),
            ("/meshes/0/primitives/0", "extensions", r#"{"KHR_draco_mesh_compression":{}}"#, "extension KHR_draco_mesh_compression at meshes[0].primitives[0].extensions.KHR_draco_mesh_compression"),
            ("/bufferViews/0", "extensions", r#"{"EXT_meshopt_compression":{}}"#, "extension EXT_meshopt_compression at bufferViews[0].extensions.EXT_meshopt_compression"),
            ("/nodes/0", "skin", "0", "skin at nodes[0].skin"),
            ("/nodes/0", "weights", "[1]", "morph weights at nodes[0].weights"),
            ("/meshes/0", "weights", "[1]", "morph weights at meshes[0].weights"),
        ] {
            let (mut value, bytes) = fixture(); value.pointer_mut(pointer).unwrap()[key] = json(text);
            let error = from_fixture(&value, &bytes).unwrap_err();
            assert!(error.contains(expected), "{error}");
        }
        for mode in [0, 1, 2, 3, 5, 6] {
            let (mut value, bytes) = fixture();
            value["meshes"][0]["primitives"][0]["mode"] = Value::from(mode);
            assert!(from_fixture(&value, &bytes)
                .unwrap_err()
                .contains("primitive mode at meshes[0].primitives[0].mode"));
        }
        for name in ["TEXCOORD_1", "COLOR_1", "JOINTS_0", "WEIGHTS_0"] {
            let (mut value, bytes) = fixture();
            value["meshes"][0]["primitives"][0]["attributes"][name] = Value::from(0);
            let error = from_fixture(&value, &bytes).unwrap_err();
            assert!(error.contains(&format!("attributes.{name}")), "{error}");
        }
    }

    #[test]
    fn buffer_uri_missing_resource_and_malformed_ranges_are_recoverable() {
        for uri in [
            "data:application/octet-stream;base64,AAAA",
            "https://example.org/model.bin",
            "//host/model.bin",
            "file:/tmp/model.bin",
            "mesh%20data.bin",
            "C:\\model.bin",
        ] {
            let (mut value, _) = fixture();
            value["buffers"][0]["uri"] = Value::String(uri.into());
            let error = parse(&gltf::json::serialize::to_vec(&value).unwrap(), |_| {
                panic!("must not resolve URI")
            })
            .unwrap_err();
            assert!(
                error.contains("Unsupported buffer URI at buffers[0].uri"),
                "{error}"
            );
        }
        let (value, bytes) = fixture();
        let error = parse(&gltf::json::serialize::to_vec(&value).unwrap(), |_| {
            Err("not found".into())
        })
        .unwrap_err();
        assert!(error.contains("Missing resource at buffers[0].uri"));
        assert!(from_fixture(&value, &bytes[..20])
            .unwrap_err()
            .contains("Malformed buffer at buffers[0]"));
        for (pointer, key, number, expected) in [
            (
                "/bufferViews/0",
                "byteOffset",
                32,
                "Malformed buffer view at bufferViews[0]",
            ),
            (
                "/accessors/0",
                "byteOffset",
                12,
                "Malformed accessor at accessors[0]",
            ),
            (
                "/bufferViews/0",
                "byteStride",
                4,
                "Malformed accessor at accessors[0]",
            ),
        ] {
            let mut value = value.clone();
            value.pointer_mut(pointer).unwrap()[key] = Value::from(number);
            assert!(from_fixture(&value, &bytes).unwrap_err().contains(expected));
        }
        assert!(parse(b"not json", |_| unreachable!())
            .unwrap_err()
            .contains("Malformed JSON"));
        assert!(parse(b"glTFbad", |_| unreachable!())
            .unwrap_err()
            .contains("Malformed GLB"));
        let mut invalid = value.clone();
        invalid["nodes"][0]["mesh"] = Value::from(99);
        assert!(from_fixture(&invalid, &bytes)
            .unwrap_err()
            .contains("Malformed glTF"));
    }

    #[test]
    fn rejects_nonfinite_binary_values_out_of_range_indices_and_degenerate_geometry() {
        let (value, bytes) = fixture();
        for number in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
            let mut bytes = bytes.clone();
            bytes[..4].copy_from_slice(&number.to_le_bytes());
            assert!(from_fixture(&value, &bytes)
                .unwrap_err()
                .contains("Non-finite value at accessors[0][0]"));
        }
        let (mut value, mut bytes) = fixture();
        let index = add_accessor(&mut value, &mut bytes, &[0, 1, 3], 5121, "SCALAR", 3);
        value["meshes"][0]["primitives"][0]["indices"] = Value::from(index);
        assert!(from_fixture(&value, &bytes)
            .unwrap_err()
            .contains("Out-of-range index at meshes[0].primitives[0].indices"));
        let (value, mut bytes) = fixture();
        bytes.fill(0);
        assert!(from_fixture(&value, &bytes)
            .unwrap_err()
            .contains("Empty model"));
        // A zero-area triangle among valid ones is skipped rather than rejecting the model.
        let (mut value, mut bytes) = fixture();
        let index = add_accessor(
            &mut value,
            &mut bytes,
            &[0, 0, 1, 0, 1, 2],
            5121,
            "SCALAR",
            6,
        );
        value["meshes"][0]["primitives"][0]["indices"] = Value::from(index);
        assert_eq!(from_fixture(&value, &bytes).unwrap().positions.len(), 3);
        let (mut value, mut bytes) = fixture();
        let normal = add_accessor(&mut value, &mut bytes, &[0; 36], 5126, "VEC3", 3);
        value["meshes"][0]["primitives"][0]["attributes"]["NORMAL"] = Value::from(normal);
        assert!(from_fixture(&value, &bytes)
            .unwrap_err()
            .contains("Invalid normal"));
        let (mut value, bytes) = fixture();
        value["nodes"][0]["scale"] = json("[0,1,1]");
        assert!(from_fixture(&value, &bytes)
            .unwrap_err()
            .contains("Invalid transform at nodes[0]"));
    }

    #[test]
    fn scene_selection_empty_cycles_and_limits_report_paths() {
        let (mut value, bytes) = fixture();
        value.as_object_mut().unwrap().remove("scene");
        assert!(from_fixture(&value, &bytes).is_ok());
        value["scenes"]
            .as_array_mut()
            .unwrap()
            .push(json(r#"{"nodes":[]}"#));
        assert!(from_fixture(&value, &bytes)
            .unwrap_err()
            .contains("Ambiguous or missing scene at scenes"));
        value["scene"] = Value::from(0);
        assert!(from_fixture(&value, &bytes).is_ok());
        value["scenes"][0]["nodes"] = json("[]");
        assert!(from_fixture(&value, &bytes)
            .unwrap_err()
            .contains("Empty model at scenes"));
        let (mut value, bytes) = fixture();
        value["nodes"][0]["children"] = json("[0]");
        assert!(from_fixture(&value, &bytes)
            .unwrap_err()
            .contains("Cyclic hierarchy at nodes[0]"));
        let (mut value, bytes) = fixture();
        value["scenes"][0]["nodes"] = json("[0,0]");
        assert!(from_fixture(&value, &bytes)
            .unwrap_err()
            .contains("Repeated node at nodes[0]"));
        let (mut value, bytes) = fixture();
        value["accessors"][0]["count"] = Value::from(MAX_VERTICES + 1);
        assert!(from_fixture(&value, &bytes)
            .unwrap_err()
            .contains("Limit exceeded at accessors[0].count"));
        let (mut value, bytes) = fixture();
        value["nodes"] = Value::Array(vec![json("{}"); MAX_NODES + 1]);
        assert!(from_fixture(&value, &bytes)
            .unwrap_err()
            .contains("Limit exceeded at nodes"));
        let (mut value, bytes) = fixture();
        value["buffers"][0]["byteLength"] = Value::from(MAX_BYTES + 1);
        assert!(from_fixture(&value, &bytes)
            .unwrap_err()
            .contains("Limit exceeded at buffers[0].byteLength"));
        let (mut value, bytes) = fixture();
        value["nodes"] = Value::Array(
            (0..MAX_DEPTH + 2)
                .map(|i| {
                    if i == MAX_DEPTH + 1 {
                        json(r#"{"mesh":0}"#)
                    } else {
                        json(&format!(r#"{{"children":[{}]}}"#, i + 1))
                    }
                })
                .collect(),
        );
        assert!(from_fixture(&value, &bytes)
            .unwrap_err()
            .contains("expected hierarchy depth <= 128"));
        let (mut value, bytes) = fixture();
        value["nodes"] = Value::Array(vec![json(r#"{"mesh":0}"#); 1001]);
        value["scenes"][0]["nodes"] = Value::Array((0..1001).map(Value::from).collect());
        value["meshes"][0]["primitives"] =
            Value::Array(vec![value["meshes"][0]["primitives"][0].clone(); 100]);
        assert!(from_fixture(&value, &bytes)
            .unwrap_err()
            .contains("expected <= 100000 triangles"));
    }

    #[test]
    fn failed_and_stale_results_preserve_previous_mesh_and_simulation() {
        use crate::{
            gpu_flock::gpu_params,
            instancing::VehicleInstances,
            target::{Target, TargetSelection},
            GlobalState,
        };
        let mut app = App::new();
        app.init_resource::<GlobalState>()
            .init_resource::<Appearance>()
            .init_resource::<Assets<Mesh>>()
            .init_resource::<VehicleModels>();
        let entity = app
            .world_mut()
            .spawn((
                Target,
                TargetSelection(true),
                Transform::from_xyz(3.0, 5.0, 7.0),
            ))
            .id();
        let instances = vec![[Vec4::ONE, Vec4::W]; 200];
        let flock = app
            .world_mut()
            .spawn((VehicleInstances(instances.clone()), Transform::default()))
            .id();
        let state = app.world().resource::<GlobalState>().clone();
        let encode = |state: &GlobalState| {
            let mut buffer = encase::UniformBuffer::new(Vec::<u8>::new());
            buffer
                .write(&gpu_params(state, Vec3::ONE, true, 1.0 / 60.0))
                .unwrap();
            buffer.into_inner()
        };
        let params = encode(&state);
        let complete = |world: &mut World, id, result| {
            world.resource_scope(|world, mut models: Mut<VehicleModels>| {
                world.resource_scope(|world, mut meshes: Mut<Assets<Mesh>>| {
                    models.finish(
                        id,
                        "glider.glb".into(),
                        result,
                        &mut meshes,
                        &mut world.resource_mut::<Appearance>(),
                    );
                });
            });
        };
        let loaded = || load(&root().join("assets/models/glider.glb"), default());
        complete(app.world_mut(), 0, loaded());
        let previous = app
            .world()
            .resource::<VehicleModels>()
            .current
            .clone()
            .unwrap();
        assert_eq!(
            app.world().resource::<Appearance>().vehicles.shape,
            VehicleShape::Custom
        );
        assert_eq!(
            app.world().resource::<Appearance>().vehicles.color_mode,
            ColorMode::ModelColors
        );
        complete(
            app.world_mut(),
            0,
            Err("Unsupported texture map at materials[0].normalTexture".into()),
        );
        assert!(app
            .world()
            .resource::<VehicleModels>()
            .status
            .starts_with("Error:"));
        app.world_mut().resource_mut::<VehicleModels>().generation = 2;
        let status = app.world().resource::<VehicleModels>().status.clone();
        complete(app.world_mut(), 1, loaded());
        assert_eq!(app.world().resource::<VehicleModels>().status, status);
        complete(app.world_mut(), 1, Err("stale failure".into()));
        assert_eq!(
            app.world().resource::<VehicleModels>().current,
            Some(previous)
        );
        assert_eq!(
            app.world().resource::<Appearance>().vehicles.shape,
            VehicleShape::Custom
        );
        assert_eq!(app.world().resource::<GlobalState>(), &state);
        assert_eq!(encode(app.world().resource::<GlobalState>()), params);
        assert_eq!(
            app.world().get::<VehicleInstances>(flock).unwrap().0,
            instances
        );
        assert!(app.world().get::<TargetSelection>(entity).unwrap().0);
        assert_eq!(
            app.world().get::<Transform>(entity).unwrap().translation,
            Vec3::new(3.0, 5.0, 7.0)
        );
    }

    #[test]
    fn invalid_attributes_counts_colors_and_settings_report_their_owner() {
        for name in ["POSITION", "NORMAL"] {
            let (mut value, mut bytes) = fixture();
            let index = add_accessor(&mut value, &mut bytes, &[0; 18], 5123, "VEC3", 3);
            value["meshes"][0]["primitives"][0]["attributes"][name] = Value::from(index);
            if name == "POSITION" {
                value["accessors"][index]["min"] = json("[0,0,0]");
                value["accessors"][index]["max"] = json("[1,1,1]");
            }
            let error = from_fixture(&value, &bytes).unwrap_err();
            assert!(
                error.contains(&format!(
                    "Unsupported attribute {name} at meshes[0].primitives[0].attributes.{name}"
                )),
                "{error}"
            );
        }
        for name in ["NORMAL", "COLOR_0"] {
            let (mut value, mut bytes) = fixture();
            let index = add_accessor(&mut value, &mut bytes, &[0; 12], 5126, "VEC3", 1);
            value["meshes"][0]["primitives"][0]["attributes"][name] = Value::from(index);
            assert!(from_fixture(&value, &bytes).unwrap_err().contains(&format!(
                "Mismatched attribute count at meshes[0].primitives[0].attributes.{name}"
            )));
        }
        let (mut value, mut bytes) = fixture();
        let data: Vec<_> = [1.1_f32, 1.0, 1.0]
            .repeat(3)
            .into_iter()
            .flat_map(f32::to_le_bytes)
            .collect();
        let color = add_accessor(&mut value, &mut bytes, &data, 5126, "VEC3", 3);
        value["meshes"][0]["primitives"][0]["attributes"]["COLOR_0"] = Value::from(color);
        assert!(from_fixture(&value, &bytes)
            .unwrap_err()
            .contains("Invalid vertex color at meshes[0].primitives[0].attributes.COLOR_0"));
        value["accessors"][color]["normalized"] = Value::Bool(true);
        assert!(from_fixture(&value, &bytes)
            .unwrap_err()
            .contains("Unsupported attribute COLOR_0"));
        let (mut value, bytes) = fixture();
        value["materials"][0]["pbrMetallicRoughness"]["baseColorFactor"] = json("[2,1,1,1]");
        assert!(from_fixture(&value, &bytes)
            .unwrap_err()
            .contains("Invalid base color"));
        let (value, bytes) = fixture();
        for settings in [
            ModelSettings {
                scale: 0.0,
                ..default()
            },
            ModelSettings {
                scale: f32::NAN,
                ..default()
            },
        ] {
            assert!(from_fixture(&value, &bytes)
                .unwrap()
                .mesh(settings)
                .unwrap_err()
                .contains("Invalid scale at model.scale"));
        }
        assert!(from_fixture(&value, &bytes)
            .unwrap()
            .mesh(ModelSettings {
                up: ModelAxis::NegZ,
                ..default()
            })
            .unwrap_err()
            .contains("Invalid axes at model.up"));
        let mut model = from_fixture(&value, &bytes).unwrap();
        for p in &mut model.positions {
            p[2] = 0.0;
        }
        assert!(model
            .mesh(default())
            .unwrap_err()
            .contains("Invalid bounds at model.forward"));
        let (mut value, mut bytes) = fixture();
        let index = add_accessor(&mut value, &mut bytes, &[0, 1], 5121, "SCALAR", 2);
        value["meshes"][0]["primitives"][0]["indices"] = Value::from(index);
        assert!(from_fixture(&value, &bytes)
            .unwrap_err()
            .contains("Invalid triangle count at meshes[0].primitives[0].indices"));
        value["accessors"][index]["normalized"] = Value::Bool(true);
        assert!(from_fixture(&value, &bytes)
            .unwrap_err()
            .contains("Unsupported index format at meshes[0].primitives[0].indices"));
        let (mut value, bytes) = fixture();
        value["buffers"][0].as_object_mut().unwrap().remove("uri");
        assert!(from_fixture(&value, &bytes)
            .unwrap_err()
            .contains("Missing GLB BIN chunk at buffers[0]"));
        let (mut value, bytes) = fixture();
        value["accessors"][0]
            .as_object_mut()
            .unwrap()
            .remove("bufferView");
        let error = from_fixture(&value, &bytes).unwrap_err();
        assert!(
            error.contains("Malformed glTF") && error.contains("accessors[0].bufferView"),
            "{error}"
        );
    }

    #[test]
    fn supplied_normals_follow_inverse_transpose_under_nonuniform_reflection() {
        let (mut value, mut bytes) = fixture();
        let data: Vec<_> = [0.0_f32, -1.0, 1.0]
            .repeat(3)
            .into_iter()
            .flat_map(f32::to_le_bytes)
            .collect();
        let normal = add_accessor(&mut value, &mut bytes, &data, 5126, "VEC3", 3);
        value["meshes"][0]["primitives"][0]["attributes"]["NORMAL"] = Value::from(normal);
        value["nodes"][0]["scale"] = json("[-2,3,4]");
        let model = from_fixture(&value, &bytes).unwrap();
        let expected = Vec3::new(0.0, -1.0 / 3.0, 1.0 / 4.0).normalize();
        assert!(model
            .normals
            .iter()
            .all(|n| Vec3::from_array(*n).distance(expected) < 1e-6));
    }

    #[test]
    fn asynchronous_requests_scan_cancel_and_recover_without_installing_stale_meshes() {
        use std::time::{Duration, Instant};
        let mut app = App::new();
        app.add_plugins(bevy::app::TaskPoolPlugin::default())
            .init_resource::<Assets<Mesh>>()
            .init_resource::<Appearance>()
            .add_plugins(VehicleModelPlugin);
        let settle = |app: &mut App| {
            let deadline = Instant::now() + Duration::from_secs(3);
            loop {
                app.update();
                let models = app.world().resource::<VehicleModels>();
                if models.pending.is_none() && models.scan.is_none() {
                    break;
                }
                assert!(Instant::now() < deadline, "background job timed out");
                std::thread::sleep(Duration::from_millis(1));
            }
        };
        settle(&mut app);
        assert!(app
            .world()
            .resource::<VehicleModels>()
            .paths
            .iter()
            .any(|p| p.ends_with("glider.glb")));
        app.world_mut()
            .resource_mut::<VehicleModels>()
            .request("assets/models/glider.glb".into());
        assert!(app
            .world()
            .resource::<VehicleModels>()
            .status
            .starts_with("Loading"));
        app.world_mut()
            .resource_mut::<VehicleModels>()
            .request("assets/models/missing-stage3.glb".into());
        settle(&mut app);
        assert!(app.world().resource::<VehicleModels>().current.is_none());
        assert_eq!(
            app.world().resource::<Appearance>().vehicles.shape,
            VehicleShape::Dart
        );
        app.world_mut()
            .resource_mut::<VehicleModels>()
            .request("assets/models/glider.glb".into());
        settle(&mut app);
        let previous = app.world().resource::<VehicleModels>().current.clone();
        assert!(previous.is_some());
        app.world_mut()
            .resource_mut::<VehicleModels>()
            .request("assets/models/missing-stage3.glb".into());
        settle(&mut app);
        assert!(app
            .world()
            .resource::<VehicleModels>()
            .status
            .contains("Cannot read"));
        assert_eq!(app.world().resource::<VehicleModels>().current, previous);
        assert_eq!(
            app.world().resource::<Appearance>().vehicles.shape,
            VehicleShape::Custom
        );
        app.world_mut()
            .resource_mut::<VehicleModels>()
            .request("assets/models/glider.glb".into());
        app.world_mut().resource_mut::<VehicleModels>().cancel();
        app.world_mut().resource_mut::<Appearance>().vehicles.shape = VehicleShape::Fish;
        settle(&mut app);
        assert_eq!(
            app.world().resource::<Appearance>().vehicles.shape,
            VehicleShape::Fish
        );
        assert_eq!(app.world().resource::<VehicleModels>().current, previous);
    }
}
