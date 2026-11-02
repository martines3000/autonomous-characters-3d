//! CPU baking for rigid TRS parts and four-influence skins. No per-vehicle CPU animation.
use super::*;
use gltf::animation::{Interpolation, Property};

pub const SAMPLE_RATE: f32 = 30.0;
pub const MAX_FRAMES: usize = 240;
pub const MAX_BAKED_BYTES: usize = 32 * 1024 * 1024;
const MAX_JOINTS: usize = 128;

#[derive(Debug)]
pub struct BakedAnimation {
    pub data: Vec<[Vec4; 2]>,
    pub vertices: usize,
    pub frame_count: usize,
    pub duration: f32,
}

#[derive(Debug)]
pub(super) struct Channel {
    node: usize,
    property: Property,
    times: Vec<f32>,
    values: Vec<Vec4>,
    step: bool,
}

impl Channel {
    fn sample(&self, time: f32) -> Vec4 {
        let next = self.times.partition_point(|key| *key <= time);
        if next == 0 {
            return self.values[0];
        }
        let last = next - 1;
        if self.step || next == self.times.len() {
            return self.values[last];
        }
        let t = (time - self.times[last]) / (self.times[next] - self.times[last]);
        if self.property == Property::Rotation {
            Vec4::from(
                Quat::from_vec4(self.values[last]).slerp(Quat::from_vec4(self.values[next]), t),
            )
        } else {
            self.values[last].lerp(self.values[next], t)
        }
    }
}

fn channels(animation: gltf::Animation, buffers: &[Vec<u8>]) -> Result<Vec<Channel>, String> {
    let mut result = Vec::new();
    if animation.channels().count() > 256 {
        return Err(format!(
            "Limit exceeded at animations[{}].channels; expected <= 256 channels",
            animation.index()
        ));
    }
    for channel in animation.channels() {
        let path = format!(
            "animations[{}].channels[{}]",
            animation.index(),
            channel.index()
        );
        let sampler = channel.sampler();
        let property = channel.target().property();
        if property == Property::MorphTargetWeights {
            return Err(unsupported(
                "morph animation",
                &format!("{path}.target.path"),
                "translation/rotation/scale",
            ));
        }
        if sampler.interpolation() == Interpolation::CubicSpline {
            return Err(unsupported(
                "CUBICSPLINE",
                &format!(
                    "animations[{}].samplers[{}].interpolation",
                    animation.index(),
                    sampler.index()
                ),
                "LINEAR or STEP",
            ));
        }
        if matches!(
            channel.target().node().transform(),
            gltf::scene::Transform::Matrix { .. }
        ) {
            return Err(unsupported(
                "animated matrix node",
                &format!("nodes[{}].matrix", channel.target().node().index()),
                "TRS animation targets",
            ));
        }
        let input = sampler.input();
        let output = sampler.output();
        let dimensions = if property == Property::Rotation {
            gltf::accessor::Dimensions::Vec4
        } else {
            gltf::accessor::Dimensions::Vec3
        };
        if input.data_type() != DataType::F32
            || input.dimensions() != gltf::accessor::Dimensions::Scalar
            || input.normalized()
            || output.data_type() != DataType::F32
            || output.dimensions() != dimensions
            || output.normalized()
        {
            return Err(unsupported(
                "animation accessor format",
                &format!("{path}.sampler"),
                "float32 scalar times and float32 TRS values",
            ));
        }
        let reader = channel.reader(|buffer| Some(buffers[buffer.index()].as_slice()));
        let times: Vec<_> = reader
            .read_inputs()
            .ok_or_else(|| format!("Missing animation input at {path}.sampler"))?
            .collect();
        if times.is_empty() || times[0] < 0.0 || times.windows(2).any(|p| p[0] >= p[1]) {
            return Err(format!("Invalid key times at {path}.sampler; expected nonnegative strictly increasing times"));
        }
        let mut values: Vec<Vec4> = match reader
            .read_outputs()
            .ok_or_else(|| format!("Missing animation output at {path}.sampler"))?
        {
            gltf::animation::util::ReadOutputs::Translations(values)
            | gltf::animation::util::ReadOutputs::Scales(values) => {
                values.map(|v| Vec3::from_array(v).extend(0.0)).collect()
            }
            gltf::animation::util::ReadOutputs::Rotations(values) => {
                values.into_f32().map(Vec4::from_array).collect()
            }
            _ => return Err(unsupported("morph animation", &path, "TRS animation")),
        };
        if values.len() != times.len() {
            return Err(format!(
                "Mismatched key count at {path}.sampler; expected one value per time"
            ));
        }
        if property == Property::Rotation {
            for value in &mut values {
                if !value.is_finite()
                    || !value.length_squared().is_finite()
                    || value.length_squared() < 1e-12
                {
                    return Err(format!(
                        "Invalid rotation at {path}.sampler; expected finite nonzero quaternions"
                    ));
                }
                *value = value.normalize();
            }
        }
        if property == Property::Scale && values.iter().any(|v| v.truncate().min_element() <= 0.0) {
            return Err(unsupported(
                "nonpositive animated scale",
                &format!("{path}.sampler"),
                "positive scale keys",
            ));
        }
        let node = channel.target().node().index();
        if result
            .iter()
            .any(|c: &Channel| c.node == node && c.property == property)
        {
            return Err(format!("Duplicate animation target at {path}.target; expected one channel per node/property"));
        }
        result.push(Channel {
            node,
            property,
            times,
            values,
            step: sampler.interpolation() == Interpolation::Step,
        });
    }
    if result.is_empty() {
        return Err(format!(
            "Empty animation at animations[{}].channels",
            animation.index()
        ));
    }
    Ok(result)
}

pub(super) fn global_transforms(
    gltf: &gltf::Gltf,
    channels: &[Channel],
    time: f32,
) -> Result<Vec<Mat4>, String> {
    let mut locals: Vec<_> = gltf
        .nodes()
        .map(|node| Mat4::from_cols_array_2d(&node.transform().matrix()))
        .collect();
    let mut poses: Vec<_> = gltf
        .nodes()
        .map(|node| {
            let (translation, rotation, scale) = node.transform().decomposed();
            (
                Vec3::from_array(translation),
                Quat::from_array(rotation),
                Vec3::from_array(scale),
            )
        })
        .collect();
    for channel in channels {
        let value = channel.sample(time);
        let pose = &mut poses[channel.node];
        match channel.property {
            Property::Translation => pose.0 = value.truncate(),
            Property::Rotation => pose.1 = Quat::from_vec4(value),
            Property::Scale => pose.2 = value.truncate(),
            _ => unreachable!("validated TRS property"),
        }
        locals[channel.node] = Mat4::from_scale_rotation_translation(pose.2, pose.1, pose.0);
    }
    let mut parents = vec![None; locals.len()];
    for node in gltf.nodes() {
        for child in node.children() {
            if parents[child.index()].replace(node.index()).is_some() {
                return Err(format!(
                    "Repeated node at nodes[{}]; expected a tree",
                    child.index()
                ));
            }
        }
    }
    fn global(
        index: usize,
        locals: &[Mat4],
        parents: &[Option<usize>],
        result: &mut [Option<Mat4>],
        depth: usize,
    ) -> Result<Mat4, String> {
        if depth > MAX_DEPTH {
            return Err(format!(
                "Limit exceeded at nodes[{index}]; expected hierarchy depth <= {MAX_DEPTH}"
            ));
        }
        if let Some(value) = result[index] {
            return Ok(value);
        }
        let value = if let Some(parent) = parents[index] {
            global(parent, locals, parents, result, depth + 1)? * locals[index]
        } else {
            locals[index]
        };
        if !value.is_finite()
            || value.row(3) != Vec4::W
            || !value.determinant().is_finite()
            || value.determinant() == 0.0
        {
            return Err(format!("Invalid transform at nodes[{index}]; expected finite nonsingular affine TRS/matrix"));
        }
        result[index] = Some(value);
        Ok(value)
    }
    let mut result = vec![None; locals.len()];
    for index in 0..locals.len() {
        global(index, &locals, &parents, &mut result, 0)?;
    }
    Ok(result.into_iter().map(Option::unwrap).collect())
}

pub(super) fn skin_matrices(
    node: &gltf::Node,
    globals: &[Mat4],
    buffers: &[Vec<u8>],
    primitive: &gltf::Primitive,
    count: usize,
    path: &str,
) -> Result<Option<Vec<Mat4>>, String> {
    let reader = primitive.reader(|buffer| Some(buffers[buffer.index()].as_slice()));
    let joints = reader.read_joints(0);
    let weights = reader.read_weights(0);
    let Some(skin) = node.skin() else {
        if joints.is_some() || weights.is_some() {
            return Err(format!("Joint/weight attributes without skin at {path}.attributes; expected a skinned node"));
        }
        return Ok(None);
    };
    let skin_path = format!("skins[{}]", skin.index());
    let joint_nodes: Vec<_> = skin.joints().collect();
    if joint_nodes.is_empty() || joint_nodes.len() > MAX_JOINTS {
        return Err(format!(
            "Limit exceeded at {skin_path}.joints; expected 1..={MAX_JOINTS} joints"
        ));
    }
    if let Some(accessor) = skin.inverse_bind_matrices() {
        if accessor.data_type() != DataType::F32
            || accessor.dimensions() != gltf::accessor::Dimensions::Mat4
            || accessor.normalized()
        {
            return Err(unsupported(
                "inverse bind format",
                &format!("{skin_path}.inverseBindMatrices"),
                "float32 MAT4",
            ));
        }
    }
    let reader = skin.reader(|buffer| Some(buffers[buffer.index()].as_slice()));
    let binds: Vec<_> = reader
        .read_inverse_bind_matrices()
        .map(|values| values.map(|v| Mat4::from_cols_array_2d(&v)).collect())
        .unwrap_or_else(|| vec![Mat4::IDENTITY; joint_nodes.len()]);
    if binds.len() != joint_nodes.len() {
        return Err(format!(
            "Mismatched inverse bind count at {skin_path}.inverseBindMatrices"
        ));
    }
    let matrices: Vec<_> = joint_nodes
        .iter()
        .zip(binds)
        .map(|(joint, bind)| globals[joint.index()] * bind)
        .collect();
    if matrices
        .iter()
        .any(|m| !m.is_finite() || m.row(3) != Vec4::W)
    {
        return Err(format!("Invalid inverse bind matrix at {skin_path}.inverseBindMatrices; expected finite affine matrices"));
    }
    let joints: Vec<_> = joints
        .ok_or_else(|| format!("Missing JOINTS_0 at {path}.attributes.JOINTS_0"))?
        .into_u16()
        .collect();
    let weights: Vec<_> = weights
        .ok_or_else(|| format!("Missing WEIGHTS_0 at {path}.attributes.WEIGHTS_0"))?
        .into_f32()
        .collect();
    if joints.len() != count || weights.len() != count {
        return Err(format!(
            "Mismatched skin attribute count at {path}.attributes"
        ));
    }
    let mut result = Vec::with_capacity(count);
    for (vertex, (joints, weights)) in joints.into_iter().zip(weights).enumerate() {
        if weights.iter().any(|w| !w.is_finite() || *w < 0.0)
            || weights.iter().sum::<f32>() <= 0.0
            || !weights.iter().sum::<f32>().is_finite()
        {
            return Err(format!("Invalid skin weights at {path}.attributes.WEIGHTS_0[{vertex}]; expected nonnegative weights with positive finite sum"));
        }
        if joints.iter().any(|joint| *joint as usize >= matrices.len()) {
            return Err(format!(
                "Out-of-range joint at {path}.attributes.JOINTS_0[{vertex}]"
            ));
        }
        let sum = weights.iter().sum::<f32>();
        let mut matrix = Mat4::ZERO;
        for (joint, weight) in joints.into_iter().zip(weights) {
            matrix += matrices[joint as usize] * (weight / sum);
        }
        if !matrix.is_finite() || matrix.determinant() == 0.0 || !matrix.determinant().is_finite() {
            return Err(format!(
                "Singular skin transform at {path}.attributes.WEIGHTS_0[{vertex}]"
            ));
        }
        result.push(matrix);
    }
    Ok(Some(result))
}

fn budget(vertices: usize, duration: f32, path: &str) -> Result<usize, String> {
    if !duration.is_finite() || duration <= 0.0 {
        return Err(format!(
            "Invalid duration at {path}; expected finite positive duration"
        ));
    }
    if duration * SAMPLE_RATE > (MAX_FRAMES - 1) as f32 {
        return Err(format!(
            "Limit exceeded at {path}; expected <= {MAX_FRAMES} baked frames at 30 Hz"
        ));
    }
    let frame_count = (duration * SAMPLE_RATE).ceil() as usize + 1;
    if frame_count > MAX_FRAMES {
        return Err(format!("Limit exceeded at {path}; expected <= {MAX_FRAMES} baked frames at 30 Hz, got {frame_count}"));
    }
    if vertices
        .checked_mul(frame_count)
        .and_then(|n| n.checked_mul(32))
        .is_none_or(|bytes| bytes > MAX_BAKED_BYTES)
    {
        return Err(format!(
            "Limit exceeded at {path}; expected baked data <= 32 MiB"
        ));
    }
    Ok(frame_count)
}

pub(super) fn load(
    gltf: &gltf::Gltf,
    buffers: &[Vec<u8>],
    settings: ModelSettings,
    choice: &str,
) -> LoadResult {
    let clips: Vec<_> = gltf
        .animations()
        .map(|a| {
            a.name()
                .map_or_else(|| format!("Clip {}", a.index()), str::to_owned)
        })
        .collect();
    let rest = scene_model(gltf, buffers, None)?;
    let triangles = rest.positions.len() / 3;
    let primitives = rest.primitives;
    if clips.is_empty() {
        let mesh = rest.mesh(settings)?;
        return Ok(LoadedModel {
            vertices: mesh.count_vertices(),
            mesh,
            triangles,
            primitives,
            animation: None,
            clips,
            selected_clip: 0,
        });
    }
    let selected_clip = choice
        .parse::<usize>()
        .ok()
        .filter(|index| *index < clips.len())
        .or_else(|| clips.iter().position(|name| name == choice))
        .ok_or_else(|| {
            format!("Unknown animation clip '{choice}' at animations; expected index or clip name")
        })?;
    let clip = gltf.animations().nth(selected_clip).unwrap();
    let path = format!("animations[{selected_clip}]");
    let channels = channels(clip, buffers)?;
    let duration = channels
        .iter()
        .map(|channel| *channel.times.last().unwrap())
        .fold(0.0_f32, f32::max);
    let normalization = rest.normalization(settings)?;
    let mut rest = rest;
    rest.normalize(normalization)?;
    let sources = rest.sources.clone();
    let (mesh, retained) = rest.indexed(true);
    let vertices = retained.len();
    let frame_count = budget(vertices, duration, &path)?;
    let mut data = Vec::with_capacity(vertices * frame_count);
    for frame in 0..frame_count {
        let time = (frame as f32 / SAMPLE_RATE).min(duration);
        let globals = global_transforms(gltf, &channels, time)?;
        let mut pose = scene_model(gltf, buffers, Some(&globals))?;
        if pose.sources != sources {
            return Err(format!("Changing triangle topology at {path}; expected nondegenerate animation frames with constant winding"));
        }
        pose.normalize(normalization)?;
        let next = ((frame + 1) as f32 / SAMPLE_RATE).min(duration);
        // Keep a STEP discontinuity sharp instead of blending toward its next key early.
        let hold = channels.iter().any(|channel| {
            channel.step && channel.times.iter().any(|key| *key > time && *key <= next)
        });
        for &index in &retained {
            data.push([
                Vec3::from_array(pose.positions[index]).extend(if hold { 1.0 } else { 0.0 }),
                Vec3::from_array(pose.normals[index]).extend(0.0),
            ]);
        }
    }
    Ok(LoadedModel {
        mesh,
        vertices,
        triangles,
        primitives,
        animation: Some(Arc::new(BakedAnimation {
            data,
            vertices,
            frame_count,
            duration,
        })),
        clips,
        selected_clip,
    })
}

/// Matches the shader's endpoint-aware, looping 30 Hz sampling.
#[cfg(test)]
fn frame_pair(time: f32, phase: f32, duration: f32, frames: usize) -> (usize, usize, f32) {
    let time = (time / duration + phase).rem_euclid(1.0) * duration;
    let first = ((time * SAMPLE_RATE).floor() as usize).min(frames - 2);
    let next = first + 1;
    let start = first as f32 / SAMPLE_RATE;
    let end = (next as f32 / SAMPLE_RATE).min(duration);
    (first, next, (time - start) / (end - start))
}

#[cfg(test)]
mod tests {
    use super::super::tests::{add_accessor, fixture, json};
    use super::*;

    fn add_rotation(value: &mut Value, bytes: &mut Vec<u8>, node: usize) {
        let times: Vec<_> = [0.0_f32, 1.0, 2.0]
            .into_iter()
            .flat_map(f32::to_le_bytes)
            .collect();
        let input = add_accessor(value, bytes, &times, 5126, "SCALAR", 3);
        let rotations: Vec<_> = [
            Quat::IDENTITY,
            Quat::from_rotation_z(std::f32::consts::FRAC_PI_2),
            Quat::IDENTITY,
        ]
        .into_iter()
        .flat_map(|q| q.to_array())
        .flat_map(f32::to_le_bytes)
        .collect();
        let output = add_accessor(value, bytes, &rotations, 5126, "VEC4", 3);
        value["animations"] = json(&format!(
            r#"[{{"name":"Turn","samplers":[{{"input":{input},"output":{output},"interpolation":"LINEAR"}}],"channels":[{{"sampler":0,"target":{{"node":{node},"path":"rotation"}}}}]}}]"#
        ));
    }
    fn document(value: &Value, bytes: &[u8]) -> (gltf::Gltf, Vec<Vec<u8>>) {
        read_document(&gltf::json::serialize::to_vec(value).unwrap(), |_| {
            Ok(bytes.to_vec())
        })
        .unwrap()
    }

    #[test]
    fn rigid_rotation_matches_known_positions_and_step_holds_keys() {
        let (mut value, mut bytes) = fixture();
        add_rotation(&mut value, &mut bytes, 0);
        let (gltf, buffers) = document(&value, &bytes);
        let channels = channels(gltf.animations().next().unwrap(), &buffers).unwrap();
        for (time, expected) in [
            (0.0, Vec3::new(-1.0, 0.0, -1.0)),
            (0.5, Vec3::new(-0.5_f32.sqrt(), -0.5_f32.sqrt(), -1.0)),
            (1.0, Vec3::new(0.0, -1.0, -1.0)),
        ] {
            let globals = global_transforms(&gltf, &channels, time).unwrap();
            let model = scene_model(&gltf, &buffers, Some(&globals)).unwrap();
            assert!(Vec3::from_array(model.positions[0]).distance(expected) < 1e-5);
        }
        value["animations"][0]["samplers"][0]["interpolation"] = json("\"STEP\"");
        let (gltf, buffers) = document(&value, &bytes);
        let channels = super::channels(gltf.animations().next().unwrap(), &buffers).unwrap();
        assert_eq!(channels[0].sample(0.999), Vec4::from(Quat::IDENTITY));
        assert!(
            Quat::from_vec4(channels[0].sample(1.0))
                .angle_between(Quat::from_rotation_z(std::f32::consts::FRAC_PI_2))
                < 0.001
        );
        let loaded = load(&gltf, &buffers, default(), "0").unwrap();
        let animation = loaded.animation.unwrap();
        assert_eq!(animation.data[29 * animation.vertices][0].w, 1.0);
        assert_eq!(animation.data[30 * animation.vertices][0].w, 0.0);
    }

    fn skinned_fixture() -> (Value, Vec<u8>) {
        let (mut value, mut bytes) = fixture();
        value["nodes"] =
            json(r#"[{"children":[1,2]}, {"translation":[2,0,0]}, {"mesh":0,"skin":0}]"#);
        let joints = add_accessor(
            &mut value,
            &mut bytes,
            &[0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
            5121,
            "VEC4",
            3,
        );
        let weights: Vec<_> = [
            0.5_f32, 0.5, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
        ]
        .into_iter()
        .flat_map(f32::to_le_bytes)
        .collect();
        let weights = add_accessor(&mut value, &mut bytes, &weights, 5126, "VEC4", 3);
        value["meshes"][0]["primitives"][0]["attributes"]["JOINTS_0"] = Value::from(joints);
        value["meshes"][0]["primitives"][0]["attributes"]["WEIGHTS_0"] = Value::from(weights);
        let binds: Vec<_> = [
            Mat4::IDENTITY,
            Mat4::from_translation(Vec3::new(-2.0, 0.0, 0.0)),
        ]
        .into_iter()
        .flat_map(|m| m.to_cols_array())
        .flat_map(f32::to_le_bytes)
        .collect();
        let bind = add_accessor(&mut value, &mut bytes, &binds, 5126, "MAT4", 2);
        value["skins"] = json(&format!(
            r#"[{{"joints":[0,1],"inverseBindMatrices":{bind}}}]"#
        ));
        add_rotation(&mut value, &mut bytes, 1);
        (value, bytes)
    }

    #[test]
    fn two_joint_skin_and_inverse_binds_match_hand_computed_positions() {
        let (value, bytes) = skinned_fixture();
        let (gltf, buffers) = document(&value, &bytes);
        let channels = channels(gltf.animations().next().unwrap(), &buffers).unwrap();
        let bind = scene_model(&gltf, &buffers, None).unwrap();
        assert!(Vec3::from_array(bind.positions[0]).distance(Vec3::new(-1.0, 0.0, -1.0)) < 1e-6);
        let globals = global_transforms(&gltf, &channels, 1.0).unwrap();
        let pose = scene_model(&gltf, &buffers, Some(&globals)).unwrap();
        for (point, expected) in pose.positions.iter().zip([
            Vec3::new(0.5, -1.5, -1.0),
            Vec3::new(1.0, 0.0, -1.0),
            Vec3::new(1.0, -2.0, 2.0),
        ]) {
            assert!(
                Vec3::from_array(*point).distance(expected) < 1e-5,
                "{point:?} != {expected:?}"
            );
        }
        assert!(pose
            .normals
            .iter()
            .all(|n| (Vec3::from_array(*n).length() - 1.0).abs() < 1e-5));
    }

    #[test]
    fn looping_interpolation_uses_partial_last_interval_and_phase() {
        for (time, phase, expected) in [
            (0.05, 0.0, 0.05),
            (1.05, 0.0, 0.05),
            (0.0, 0.25, 0.25),
            (-0.05, 0.0, 0.95),
        ] {
            let (a, b, t) = frame_pair(time, phase, 1.0, 31);
            let actual = (a as f32 / 30.0).lerp(b as f32 / 30.0, t);
            assert!((actual - expected).abs() < 1e-5);
        }
        let (a, b, t) = frame_pair(0.104, 0.0, 0.11, 5);
        assert_eq!((a, b), (3, 4));
        assert!((t - 0.4).abs() < 1e-5);
        assert_eq!(frame_pair(1.0, 0.0, 1.0, 31), (0, 1, 0.0));
    }

    #[test]
    fn baking_reuses_rest_normalization_and_preserves_deformation_identity() {
        let (mut value, mut bytes) = fixture();
        add_rotation(&mut value, &mut bytes, 0);
        let (gltf, buffers) = document(&value, &bytes);
        let rest = scene_model(&gltf, &buffers, None).unwrap();
        let normalization = rest.normalization(default()).unwrap();
        let loaded = load(&gltf, &buffers, default(), "Turn").unwrap();
        let animation = loaded.animation.unwrap();
        assert_eq!(animation.frame_count, 61);
        let expected = (normalization.rotation
            * (Vec3::new(0.0, -1.0, -1.0) - normalization.center))
            * normalization.factor;
        assert!(
            animation.data[30 * animation.vertices][0]
                .truncate()
                .distance(expected)
                < 1e-5
        );
        assert!(animation.data.iter().flatten().all(|v| v.is_finite()));
        assert!(loaded.mesh.attribute(Mesh::ATTRIBUTE_JOINT_INDEX).is_none());
        assert!(loaded
            .mesh
            .attribute(Mesh::ATTRIBUTE_JOINT_WEIGHT)
            .is_none());
        assert!(super::budget(3, 1.0, "animations[0]").is_ok());
    }

    #[test]
    fn budgets_and_unsupported_animation_errors_name_limits_and_paths() {
        assert!(budget(1000, 8.0, "animations[0]")
            .unwrap_err()
            .contains("animations[0]; expected <= 240 baked frames"));
        assert!(budget(300_000, 1.0, "animations[0]")
            .unwrap_err()
            .contains("animations[0]; expected baked data <= 32 MiB"));
        assert!(budget(3, 0.0, "animations[0]")
            .unwrap_err()
            .contains("Invalid duration at animations[0]"));
        let (mut value, mut bytes) = fixture();
        add_rotation(&mut value, &mut bytes, 0);
        value["animations"][0]["samplers"][0]["interpolation"] = json("\"CUBICSPLINE\"");
        let (gltf, buffers) = document(&value, &bytes);
        assert!(load(&gltf, &buffers, default(), "0")
            .unwrap_err()
            .contains("Unsupported CUBICSPLINE at animations[0].samplers[0].interpolation"));
        value["animations"][0]["samplers"][0]["interpolation"] = json("\"LINEAR\"");
        let (gltf, buffers) = document(&value, &bytes);
        assert!(load(&gltf, &buffers, default(), "missing")
            .unwrap_err()
            .contains("Unknown animation clip 'missing' at animations"));
        let (mut value, mut bytes) = skinned_fixture();
        let joints = value["meshes"][0]["primitives"][0]["attributes"]["JOINTS_0"]
            .as_u64()
            .unwrap() as usize;
        let view = value["accessors"][joints]["bufferView"].as_u64().unwrap() as usize;
        let offset = value["bufferViews"][view]["byteOffset"].as_u64().unwrap() as usize;
        bytes[offset] = 2;
        let (gltf, buffers) = document(&value, &bytes);
        assert!(load(&gltf, &buffers, default(), "0")
            .unwrap_err()
            .contains("Out-of-range joint at meshes[0].primitives[0].attributes.JOINTS_0[0]"));
        value["meshes"][0]["primitives"][0]["targets"] = json("[{}]");
        assert!(
            read_document(&gltf::json::serialize::to_vec(&value).unwrap(), |_| Ok(
                bytes.clone()
            ))
            .unwrap_err()
            .contains("Unsupported morph targets at meshes[0].primitives[0].targets")
        );
    }

    #[test]
    fn examples_have_distinct_materials_and_two_valid_small_animation_clips() {
        let bird = load_clip(&root().join("assets/models/bird.glb"), default(), "Flap").unwrap();
        assert_eq!(bird.clips, ["Flap", "Glide"]);
        let animation = bird.animation.unwrap();
        assert_eq!(animation.frame_count, 31);
        assert_eq!(animation.data.len(), animation.vertices * 31);
        assert!(animation.data[..animation.vertices]
            .iter()
            .zip(&animation.data[15 * animation.vertices..16 * animation.vertices])
            .any(|(a, b)| a[0].truncate().distance(b[0].truncate()) > 0.1));
        let glide = load_clip(&root().join("assets/models/bird.glb"), default(), "1").unwrap();
        assert_eq!(glide.selected_clip, 1);
        assert_eq!(glide.animation.unwrap().frame_count, 61);
        let material = load_clip(
            &root().join("assets/models/material-glider.glb"),
            default(),
            "0",
        )
        .unwrap();
        assert!(material.animation.is_none());
        let bevy::mesh::VertexAttributeValues::Float32x2(values) =
            material.mesh.attribute(ATTRIBUTE_MATERIAL).unwrap()
        else {
            panic!("materials");
        };
        assert!(values.contains(&[1.0, 0.12]) && values.contains(&[0.0, 0.95]));
    }

    #[test]
    fn in_memory_clips_enforce_frame_and_memory_limits_before_baking() {
        let (mut value, mut bytes) = fixture();
        add_rotation(&mut value, &mut bytes, 0);
        let input = value["animations"][0]["samplers"][0]["input"]
            .as_u64()
            .unwrap() as usize;
        let view = value["accessors"][input]["bufferView"].as_u64().unwrap() as usize;
        let offset = value["bufferViews"][view]["byteOffset"].as_u64().unwrap() as usize;
        for (i, time) in [0.0_f32, 5.0, 10.0].into_iter().enumerate() {
            bytes[offset + i * 4..offset + i * 4 + 4].copy_from_slice(&time.to_le_bytes());
        }
        let (gltf, buffers) = document(&value, &bytes);
        assert!(load(&gltf, &buffers, default(), "0")
            .unwrap_err()
            .contains("Limit exceeded at animations[0]; expected <= 240 baked frames"));
        for (i, time) in [0.0_f32, 3.0, 6.0].into_iter().enumerate() {
            bytes[offset + i * 4..offset + i * 4 + 4].copy_from_slice(&time.to_le_bytes());
        }
        value["nodes"] = Value::Array(vec![json(r#"{"mesh":0}"#); 2000]);
        value["scenes"][0]["nodes"] = Value::Array((0..2000).map(Value::from).collect());
        let (gltf, buffers) = document(&value, &bytes);
        assert!(load(&gltf, &buffers, default(), "0")
            .unwrap_err()
            .contains("Limit exceeded at animations[0]; expected baked data <= 32 MiB"));
    }

    #[test]
    fn linear_translation_and_scale_channels_compose_in_trs_order() {
        let (value, bytes) = fixture();
        let (gltf, _) = document(&value, &bytes);
        let channels = [
            Channel {
                node: 0,
                property: Property::Translation,
                times: vec![0.0, 1.0],
                values: vec![Vec4::ZERO, Vec4::new(4.0, 2.0, 0.0, 0.0)],
                step: false,
            },
            Channel {
                node: 0,
                property: Property::Scale,
                times: vec![0.0, 1.0],
                values: vec![Vec4::ONE, Vec4::new(3.0, 1.0, 1.0, 0.0)],
                step: false,
            },
        ];
        let globals = global_transforms(&gltf, &channels, 0.5).unwrap();
        assert!(
            globals[0]
                .transform_point3(Vec3::X)
                .distance(Vec3::new(4.0, 1.0, 0.0))
                < 1e-6
        );
    }

    #[test]
    fn zero_and_overflowing_finite_rotation_keys_fail_before_normalization() {
        for component in [0.0_f32, 1e30] {
            let (mut value, mut bytes) = fixture();
            add_rotation(&mut value, &mut bytes, 0);
            let output = value["animations"][0]["samplers"][0]["output"]
                .as_u64()
                .unwrap() as usize;
            let view = value["accessors"][output]["bufferView"].as_u64().unwrap() as usize;
            let offset = value["bufferViews"][view]["byteOffset"].as_u64().unwrap() as usize;
            for i in 0..4 {
                bytes[offset + i * 4..offset + i * 4 + 4].copy_from_slice(&component.to_le_bytes());
            }
            let (gltf, buffers) = document(&value, &bytes);
            assert!(load(&gltf, &buffers, default(), "0")
                .unwrap_err()
                .contains("Invalid rotation at animations[0].channels[0].sampler"));
        }
    }
}
