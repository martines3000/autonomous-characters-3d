// Params, Boid and Instance are prepended from gpu_flock.wgsl to keep layouts identical.
@group(0) @binding(0) var<uniform> params: Params;
@group(0) @binding(1) var<storage, read_write> prey: array<Boid>;
@group(0) @binding(2) var<storage, read_write> arena: array<vec4<u32>>;
// Two 16-hunter snapshots, each hunter has position/timer, velocity/group and rotation.
@group(0) @binding(3) var<storage, read_write> hunters: array<vec4<f32>>;
// 16 instances followed by 16 previous poses, all in vec4 units.
@group(0) @binding(4) var<storage, read_write> instances: array<vec4<f32>>;
fn unit(v: vec3<f32>) -> vec3<f32> {
    if length(v) <= 1.1920929e-7 { return vec3<f32>(0.0); }
    return normalize(v);
}
fn cap(v: vec3<f32>, max: f32) -> vec3<f32> {
    if dot(v,v) > max*max { return unit(v)*max; }
    return v;
}
// Reuse gpu_flock.wgsl's rotation helpers, matching motion::initial at the poles too.
fn heading(v: vec3<f32>) -> vec4<f32> {
    let direction = unit(v);
    if length(direction) == 0.0 { return vec4<f32>(0.0,0.0,0.0,1.0); }
    return upright_target(heading_rotation(direction),direction,vec3<f32>(0.0));
}
var<workgroup> distances: array<f32,256>;
var<workgroup> prey_indices: array<u32,256>;
var<workgroup> identities: array<u32,256>;
fn identity(index: u32) -> u32 {
    let record = arena[2u*params.groups.w+params.groups.z+index/4u][index%4u];
    return arena[record].x;
}
@compute @workgroup_size(256)
fn hunt(@builtin(workgroup_id) id: vec3<u32>, @builtin(local_invocation_index) lane: u32) {
    let i = id.x;
    let count = u32(params.attractor_strength.y);
    if i >= count { return; }
    let input = params.predators.w + i*3u;
    let output = 48u-params.predators.w + i*3u;
    let old = hunters[input];
    let old_velocity = hunters[input+1u];
    let group = u32(old_velocity.w);
    let settings = params.group_targets[group];
    let scale = params.spawn_size/100.0;
    let speed = params.max_speed*settings.y;
    var position = old.xyz;
    var timer = old.w;
    var velocity = old_velocity.xyz;
    var rotation = hunters[input+2u];
    // Parallel exact search: one 256-lane reduction per hunter, ties use stable spawn ID.
    var best = (180.0*scale)*(180.0*scale);
    var best_index = 0xffffffffu;
    var best_id = 0xffffffffu;
    if timer == 0.0 && params.dt > 0.0 {
        for (var k = lane; k < params.count; k += 256u) {
            let other = prey[k];
            let distance = dot(position-other.position,position-other.position);
            if distance < best || (distance == best && best_index != 0xffffffffu && identity(k) < best_id) {
                best = distance; best_index = k; best_id = identity(k);
            }
        }
    }
    distances[lane] = best;
    prey_indices[lane] = best_index;
    identities[lane] = best_id;
    workgroupBarrier();
    for (var stride = 128u; stride > 0u; stride /= 2u) {
        if lane < stride {
            let other = lane+stride;
            if distances[other] < distances[lane] || (distances[other] == distances[lane] && identities[other] < identities[lane]) {
                distances[lane] = distances[other]; prey_indices[lane] = prey_indices[other]; identities[lane] = identities[other];
            }
        }
        workgroupBarrier();
    }
    if lane != 0u { return; }
    best = distances[0];
    let found = prey_indices[0] != 0xffffffffu;
    var prey_position = vec3<f32>(0.0);
    var target_velocity = vec3<f32>(0.0);
    if found { prey_position = prey[prey_indices[0]].position; target_velocity = prey[prey_indices[0]].velocity; }
    if params.motion.w <= 0.0 || params.dt == 0.0 {
        instances[48u+i*2u] = vec4<f32>(position,0.0);
        instances[49u+i*2u] = rotation;
    }
    if params.dt > 0.0 {
        var desired = unit(velocity)*speed;
        if timer > 0.0 {
            timer -= params.dt;
            desired *= 1.8;
            if timer <= 0.0 { timer = -0.8; }
        } else if timer < 0.0 { timer = min(timer+params.dt,0.0); }
        else if found {
            desired = unit(prey_position+target_velocity*0.12-position)*speed;
            if best < (32.0*scale)*(32.0*scale) { timer = 0.45; desired *= 1.8; }
        } else {
            let phase = f32(i)*1.7;
            desired = unit(velocity+vec3<f32>(sin(position.y/scale*0.03+phase),cos(position.z/scale*0.03+phase),sin(position.x/scale*0.03+phase))*(speed*0.15))*speed;
        }
        var separation = vec3<f32>(0.0);
        for (var k = 0u; k < count; k += 1u) {
            if k == i { continue; }
            let offset = position-hunters[params.predators.w+k*3u].xyz;
            let distance = length(offset);
            if distance > 0.0 && distance < 12.0*scale { separation += offset/distance*(1.0-distance/(12.0*scale)); }
        }
        for (var axis = 0u; axis < 3u; axis += 1u) {
            if position[axis] < -params.walls[axis] { desired[axis] = speed; }
            else if position[axis] > params.walls[axis] { desired[axis] = -speed; }
        }
        let acceleration = cap(desired-velocity,speed*3.0)+cap(separation,1.0)*speed*2.0;
        velocity = cap(velocity+acceleration*params.dt, speed*select(1.0,1.8,timer > 0.0));
        position += velocity*params.dt;
        rotation = heading(velocity);
    }
    hunters[output] = vec4<f32>(position,timer);
    hunters[output+1u] = vec4<f32>(velocity,f32(group));
    hunters[output+2u] = rotation;
    let flee = u32(params.attractor_strength.z)+i*2u;
    arena[flee] = bitcast<vec4<u32>>(vec4<f32>(position,24.0*scale));
    arena[flee+1u] = vec4<u32>(255u,bitcast<vec3<u32>>(velocity));
    let packed = 0x3f800000u | u32(floor(clamp(length(velocity)/max(params.max_speed,1.1920929e-7),0.0,1.0)*65535.0+0.5)) | (group<<16u);
    instances[i*3u] = vec4<f32>(position,bitcast<f32>(packed));
    instances[i*3u+1u] = rotation;
    instances[i*3u+2u] = vec4<f32>(f32(i)/16.0,settings.x,settings.w,0.0);
}
