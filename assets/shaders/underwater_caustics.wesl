// Project surface wave interference down world XZ. Both fish and sand call this function.
// Broad waves scale with the water column; time comes only from SimulationClock.
fn caustics(world: vec3<f32>, normal: vec3<f32>, seconds: f32, half_height: f32) -> vec3<f32> {
    let p = world.xz * (23.0 / max(half_height, 1.0));
    let t = seconds * 0.32;
    let warp = vec2<f32>(sin(p.y * 1.7 + t), cos(p.x * 1.3 - t)) * 0.65;
    let q = p + warp;
    // Narrow crossing wave ridges form gently moving, broken cellular ribbons.
    let a = sin(q.x * 2.0 + sin(q.y * 1.8 + t) + t);
    let b = sin(q.y * 2.3 + sin(q.x * 1.5 - t) - t * 1.2);
    let ridge = pow(1.0 - abs(a * 0.55 + b * 0.45), 12.0);
    // Body-scale detail stays visible on a fish; derivatives suppress distant floor aliasing.
    let fine = world.xz * 0.65 + warp;
    let detail = pow(1.0 - abs(sin(fine.x * 1.7 - fine.y * 1.3 + sin(fine.y * 1.2 + t) + t * 0.7)), 12.0)
        / (1.0 + dot(fwidth(fine), vec2<f32>(5.0)));
    let depth = max(1.3 - world.y / max(half_height, 1.0), 0.0);
    let attenuation = exp(-0.42 * depth);
    let facing = 0.15 + 0.85 * max(normal.y, 0.0);
    return vec3<f32>(0.50, 1.0, 0.72) * (ridge + detail * 0.5) * attenuation * facing;
}
