// One instance per vehicle; lit with Bevy's PBR functions so scene lights and fog apply.
import bevy_pbr::render::{
    view_transformations::position_world_to_clip,
    pbr_types::{pbr_input_new, STANDARD_MATERIAL_FLAGS_FOG_ENABLED_BIT},
    pbr_functions::{apply_pbr_lighting, calculate_view, main_pass_post_lighting_processing},
};

struct Appearance {
    colors: array<vec4<f32>, 7>,
    material: vec4<f32>,
    shading: vec4<f32>,
    modes: vec4<u32>,
    proportions: vec4<f32>,
    solid_color: vec4<f32>,
};
@group(3) @binding(0) var<uniform> appearance: Appearance;

struct Vertex {
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @if(VERTEX_COLORS)
    @location(5) color: vec4<f32>,
    // xyz: world position, w: packed normalized speed/reserved group.
    @location(8) i_position_speed_group: vec4<f32>,
    // Rotation quaternion (x, y, z, w).
    @location(9) i_rotation: vec4<f32>,
};

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) world_position: vec4<f32>,
    @location(1) world_normal: vec3<f32>,
    @location(2) color: vec3<f32>,
};

fn rotate(q: vec4<f32>, v: vec3<f32>) -> vec3<f32> {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}

fn hsv_to_rgb(h: f32, s: f32, v: f32) -> vec3<f32> {
    let k = fract(vec3<f32>(h) + vec3<f32>(0.0, 2.0 / 3.0, 1.0 / 3.0)) * 6.0;
    return v * mix(vec3<f32>(1.0), clamp(abs(k - 3.0) - 1.0, vec3<f32>(0.0), vec3<f32>(1.0)), s);
}

// Hue follows the horizontal heading; climbing vehicles are lighter, diving ones darker.
fn heading_color(heading: vec3<f32>) -> vec3<f32> {
    let hue = atan2(heading.z, heading.x) / 6.2831853 + 0.5;
    if appearance.modes.x != 0u {
        let wheel = fract(hue) * 6.0;
        let index = u32(wheel);
        let color = mix(appearance.colors[index].rgb, appearance.colors[index + 1u].rgb, fract(wheel));
        return color * (0.78 + 0.22 * heading.y);
    }
    let value = 0.8 + 0.2 * heading.y;
    let srgb = hsv_to_rgb(hue, 0.65, value);
    return pow(srgb, vec3<f32>(2.2));
}

// Scalar ramps use the six distinct stops without joining the last color back to the first.
fn palette_ramp(t: f32) -> vec3<f32> {
    let value = clamp(t, 0.0, 1.0);
    if appearance.modes.x == 0u {
        return pow(hsv_to_rgb(value * (5.0 / 6.0), 0.65, 1.0), vec3<f32>(2.2));
    }
    let ramp = value * 5.0;
    let index = u32(ramp);
    return mix(appearance.colors[index].rgb, appearance.colors[index + 1u].rgb, fract(ramp));
}

@vertex
fn vertex(vertex: Vertex) -> VertexOutput {
    let q = vertex.i_rotation;
    let world = rotate(q, vertex.position * appearance.proportions.xyz * appearance.material.w) + vertex.i_position_speed_group.xyz;
    var out: VertexOutput;
    out.world_position = vec4<f32>(world, 1.0);
    out.clip_position = position_world_to_clip(world);
    // Inverse-transpose of the non-uniform local scale, then rotation; fragment normalizes it.
    out.world_normal = rotate(q, vertex.normal / appearance.proportions.xyz);
    // The cone's apex points along local +Y, so that is the heading.
    switch appearance.modes.y {
        case 1u: { out.color = palette_ramp(0.5 + 0.5 * vertex.i_position_speed_group.y / appearance.proportions.w); }
        case 2u: {
            let speed = f32(bitcast<u32>(vertex.i_position_speed_group.w) & 0xffffu) / 65535.0;
            out.color = palette_ramp(speed);
        }
        case 3u: { out.color = appearance.solid_color.rgb; }
        case 4u: {
            out.color = vec3<f32>(1.0);
            @if(VERTEX_COLORS) { out.color = vertex.color.rgb; }
        }
        default: { out.color = heading_color(rotate(q, vec3<f32>(0.0, 1.0, 0.0))); }
    }
    return out;
}

@fragment
fn fragment(in: VertexOutput) -> @location(0) vec4<f32> {
    var pbr_input = pbr_input_new();
    pbr_input.material.base_color = vec4<f32>(in.color, 1.0);
    pbr_input.material.perceptual_roughness = appearance.material.x;
    pbr_input.material.metallic = appearance.material.y;
    pbr_input.material.flags |= STANDARD_MATERIAL_FLAGS_FOG_ENABLED_BIT;
    pbr_input.frag_coord = in.clip_position;
    pbr_input.world_position = in.world_position;
    pbr_input.world_normal = normalize(in.world_normal);
    pbr_input.N = pbr_input.world_normal;
    pbr_input.V = calculate_view(in.world_position, false);
    var lit = apply_pbr_lighting(pbr_input);
    let edge = 1.0 - clamp(dot(pbr_input.N, pbr_input.V), 0.0, 1.0);
    lit = vec4<f32>(lit.rgb + in.color * (edge * edge * appearance.material.z + appearance.shading.x), lit.a);
    return main_pass_post_lighting_processing(pbr_input, lit);
}
