import bevy_pbr::render::{
    forward_io::{VertexOutput, FragmentOutput},
    pbr_fragment::pbr_input_from_standard_material,
    pbr_functions::{apply_pbr_lighting, main_pass_post_lighting_processing},
};
import super::underwater_caustics::caustics;

@group(3) @binding(100) var<uniform> water: vec4<f32>;

@fragment
fn fragment(in: VertexOutput, @builtin(front_facing) front: bool) -> FragmentOutput {
    var pbr = pbr_input_from_standard_material(in, front);
    let p = in.world_position.xz * (45.0 / max(water.y, 1.0));
    let ripples = 0.93 + 0.07 * sin(p.x * 2.0 + sin(p.y * 0.8));
    pbr.material.base_color = vec4<f32>(pbr.material.base_color.rgb * ripples, 1.0);
    var lit = apply_pbr_lighting(pbr);
    let caustic = caustics(in.world_position.xyz, pbr.N, water.x, water.y);
    lit = vec4<f32>(lit.rgb * (1.0 + caustic * 3.5) + caustic * 0.18, 1.0);
    var out: FragmentOutput;
    out.color = main_pass_post_lighting_processing(pbr, lit);
    return out;
}
