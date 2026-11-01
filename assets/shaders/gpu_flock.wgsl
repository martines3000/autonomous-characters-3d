// Flocking simulation on the GPU. Mirrors `flocking_force`, `movement` and `update` in
// src/vehicle.rs; see src/gpu_flock.rs for the pass order.
//
// Per frame: [spawn] -> [fine grid -> separate] -> coarse grid -> steer.
// Each grid: clear_counts -> count[_separation] -> scan_blocks -> scan_block_sums -> scatter.
// A nonzero cap limits alignment/cohesion only. The fine grid has separation-sized cells
// and visits every separation neighbor, then carries that force through the coarse sort.
// Cap 0 skips the fine grid and runs the CPU-equivalent unlimited query in steer.
// Neighbors are found with a hashed uniform grid: boids are counted per cell, the counts are
// prefix-summed into cell starts, boids are scattered into cell order (`sorted`), and `steer`
// visits the 27 cells around each boid. `steer` writes the next state back into `boids` in
// sorted order and fills the instance buffer for drawing. Identity/group/slot metadata travels
// with each record; sorted index is never a persistent identity.

struct Params {
    count: u32,
    spawn_from: u32,
    table_mask: u32,
    frame: u32,
    block_count: u32,
    max_neighbors: u32,
    follow: u32,
    dt: f32,
    cell_size: f32,
    size: f32,
    mass: f32,
    max_speed: f32,
    wander_speed: f32,
    separation_factor: f32,
    alignment_factor: f32,
    cohesion_factor: f32,
    wander_factor: f32,
    wall_avoid_factor: f32,
    seek_factor: f32,
    separation_distance: f32,
    alignment_distance: f32,
    cohesion_distance: f32,
    wander_distance: f32,
    wander_radius: f32,
    spawn_size: f32,
    walls: vec3<f32>,
    target_position: vec3<f32>,
    attractor_position_radius: vec4<f32>,
    attractor_strength: vec4<f32>,
    groups: vec4<u32>,
    group_targets: array<vec4<f32>, 8>,
    obstacles: vec4<u32>,
    obstacle_settings: vec4<f32>,
    formation: vec4<u32>,
    formation_center_extent: vec4<f32>,
    formation_settings: vec4<f32>,
    predators: vec4<u32>,
    predator_settings: vec4<f32>,
};

struct Boid {
    position: vec3<f32>,
    wander_theta: f32,
    velocity: vec3<f32>,
    wander_phi: f32,
    // Integration keeps the previous heading when velocity becomes zero, as on the CPU.
    rotation: vec4<f32>,
    // xyz: separation scratch; w: stable spawn phase, preserved through sorting and scratch resets.
    separation: vec4<f32>,
};

struct Instance {
    // xyz: position; w: speed/group payload (normal f32 mantissa), not physical size.
    position_speed_group: vec4<f32>,
    rotation: vec4<f32>,
    phase: vec4<f32>,
};

@group(0) @binding(0) var<uniform> params: Params;
@group(0) @binding(1) var<storage, read_write> boids: array<Boid>;
@group(0) @binding(2) var<storage, read_write> sorted: array<Boid>;
// Per boid: (cell hash, slot within the cell).
@group(0) @binding(3) var<storage, read_write> slots: array<vec2<u32>>;
@group(0) @binding(4) var<storage, read_write> counts: array<atomic<u32>>;
// Exclusive prefix sum of `counts` within each block of SCAN_BLOCK cells.
@group(0) @binding(5) var<storage, read_write> starts: array<u32>;
// Exclusive prefix sum of the block totals.
@group(0) @binding(6) var<storage, read_write> block_sums: array<u32>;
@group(0) @binding(7) var<storage, read_write> instances: array<Instance>;
// Persistent metadata table [0, capacity), maintenance scratch [capacity, 2*capacity),
// then two packed u32 order maps (capacity/4 vec4s each). Sorts carry only table indices.
// Group/slot/ID/reserved values stay in the table; z is the input order-map base, w capacity.
// Scene records begin at 5*capacity/2 and share this eighth binding.
// Spheres use two float-bitcast records: center/radius, clearance/reserved.
@group(0) @binding(8) var<storage, read_write> arena: array<vec4<u32>>;

override GROUPS_ENABLED: bool = false;
override OBSTACLES_ENABLED: bool = false;
override FORMATION_ENABLED: bool = false;
override PREDATORS_ENABLED: bool = false;

fn read_order(base: u32, index: u32) -> u32 {
    return arena[2u * params.groups.w + base + index / 4u][index % 4u];
}

fn write_order(base: u32, index: u32, record: u32) {
    arena[2u * params.groups.w + base + index / 4u][index % 4u] = record;
}

// Maintenance uses current record order, not sorted identity. counts/starts are spare until
// the normal grid build. Fixed-size blocks keep assignment linear with no cross-boid matching.
@compute @workgroup_size(64)
fn formation_count(@builtin(global_invocation_id) id: vec3<u32>) {
    let block = id.x;
    if block >= params.formation.z { return; }
    var histogram = array<u32, 8>(0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u);
    for (var i = block * 256u; i < min((block + 1u) * 256u, params.count); i += 1u) {
        let record = read_order(params.groups.z, i);
        var group = 0u;
        if params.groups.x != 0u { group = arena[record].y; }
        arena[record].z = histogram[group];
        histogram[group] += 1u;
    }
    for (var group = 0u; group < 8u; group += 1u) { atomicStore(&counts[block * 8u + group], histogram[group]); }
}

var<workgroup> formation_totals: array<u32, 8>;

@compute @workgroup_size(8)
fn formation_scan(@builtin(local_invocation_index) group: u32) {
    var total = 0u;
    for (var block = 0u; block < params.formation.z; block += 1u) { total += atomicLoad(&counts[block * 8u + group]); }
    formation_totals[group] = total;
    workgroupBarrier();
    var base = 0u;
    for (var other = 0u; other < group; other += 1u) { base += formation_totals[other]; }
    arena[params.formation.w + group] = vec4<u32>(base, total, 0u, 0u);
    for (var block = 0u; block < params.formation.z; block += 1u) {
        starts[block * 8u + group] = base;
        base += atomicLoad(&counts[block * 8u + group]);
    }
}

@compute @workgroup_size(256)
fn formation_assign(@builtin(global_invocation_id) id: vec3<u32>) {
    if id.x >= params.count { return; }
    let record = read_order(params.groups.z, id.x);
    var group = 0u;
    if params.groups.x != 0u { group = arena[record].y; }
    arena[record].z += starts[(id.x / 256u) * 8u + group];
}

fn formation_target(slot: u32, group: u32) -> vec3<f32> {
    let range = arena[params.formation.w + group].xy;
    let local_slot = slot - range.x;
    let u = (f32(local_slot) + 0.5) / f32(max(range.y, 1u));
    let y = 1.0 - 2.0 * u;
    let angle = f32((local_slot * 1640531527u) >> 8u) / 16777216.0 * (2.0 * PI);
    let radius = sqrt(max(0.0, 1.0 - y * y));
    let sphere = vec3<f32>(radius * cos(angle), y, radius * sin(angle));
    let helix_angle = 3.0 * (2.0 * PI) * u;
    let helix = vec3<f32>(0.6 * cos(helix_angle), 0.8 * y, 0.6 * sin(helix_angle));
    let count = max(params.groups.x, 1u);
    let columns = u32(ceil(sqrt(f32(count))));
    let rows = (count + columns - 1u) / columns;
    let center = vec3<f32>(f32(group % columns) - f32(columns - 1u) * 0.5, f32(group / columns) - f32(rows - 1u) * 0.5, 0.0) * 2.25;
    return params.formation_center_extent.xyz + (center + (sphere + (helix - sphere) * params.formation_settings.z)) * params.formation_center_extent.w;
}

// Count edits compact retained metadata before spawning, making vacant records reusable without
// reusing spawn IDs. Separate dispatches prevent in-place gather/write races.
@compute @workgroup_size(256)
fn gather_metadata(@builtin(global_invocation_id) id: vec3<u32>) {
    if id.x < params.spawn_from {
        arena[params.groups.w + id.x] = arena[read_order(params.groups.z, id.x)];
    }
}

@compute @workgroup_size(256)
fn compact_metadata(@builtin(global_invocation_id) id: vec3<u32>) {
    if id.x < params.spawn_from {
        arena[id.x] = arena[params.groups.w + id.x];
        write_order(params.groups.z, id.x, id.x);
    }
}

const WORKGROUP: u32 = 256u;
const SCAN_BLOCK: u32 = 512u;
const REFERENCE_HZ: f32 = 60.0;
const PI: f32 = 3.14159265;
// Matches MAX_SEPARATION_PUSH in src/vehicle.rs.
const MAX_SEPARATION_PUSH: f32 = 3.0;
// f32::EPSILON, as in `normalize_or_zero`.
const EPSILON: f32 = 1.1920929e-7;

var<workgroup> scratch: array<u32, WORKGROUP>;

fn pcg(value: u32) -> u32 {
    let state = value * 747796405u + 2891336453u;
    let word = ((state >> ((state >> 28u) + 4u)) ^ state) * 277803737u;
    return (word >> 22u) ^ word;
}

// Uniform in [0, 1).
fn random(seed: ptr<function, u32>) -> f32 {
    *seed = pcg(*seed);
    return f32(*seed >> 8u) / 16777216.0;
}

fn normalize_or_zero(v: vec3<f32>) -> vec3<f32> {
    let magnitude = length(v);
    if magnitude <= EPSILON {
        return vec3<f32>(0.0);
    }
    return v / magnitude;
}

fn limit(v: vec3<f32>, cap: f32) -> vec3<f32> {
    if dot(v, v) > cap * cap {
        return normalize(v) * cap;
    }
    return v;
}

fn cell_of(position: vec3<f32>) -> vec3<i32> {
    return vec3<i32>(floor(position / params.cell_size));
}

fn separation_cell(position: vec3<f32>) -> vec3<i32> {
    return vec3<i32>(floor(position / max(params.size * params.separation_distance, EPSILON)));
}

// The same 27 offsets for both grids, with the home cell first.
fn neighbor_offset(n: u32) -> vec3<i32> {
    let cell = (n + 13u) % 27u;
    return vec3<i32>(i32(cell % 3u), i32(cell / 3u % 3u), i32(cell / 9u)) - 1;
}

fn hash_cell(cell: vec3<i32>) -> u32 {
    let u = bitcast<vec3<u32>>(cell);
    return ((u.x * 73856093u) ^ (u.y * 19349663u) ^ (u.z * 83492791u)) & params.table_mask;
}

fn cell_start(hash: u32) -> u32 {
    return starts[hash] + block_sums[hash / SCAN_BLOCK];
}

// Same rotation as `Quat::from_rotation_arc(Vec3::Y, direction)`.
fn heading_rotation(direction: vec3<f32>) -> vec4<f32> {
    let d = direction.y;
    if d > 1.0 - 2.0 * EPSILON {
        return vec4<f32>(0.0, 0.0, 0.0, 1.0);
    }
    if d < -1.0 + 2.0 * EPSILON {
        // Vec3::Y.any_orthonormal_vector() is -Z in glam's rotation arc.
        return vec4<f32>(0.0, 0.0, -1.0, 0.0);
    }
    let axis = cross(vec3<f32>(0.0, 1.0, 0.0), direction);
    return normalize(vec4<f32>(axis, 1.0 + d));
}

@compute @workgroup_size(256)
fn spawn(@builtin(global_invocation_id) id: vec3<u32>) {
    let index = params.spawn_from + id.x;
    if index >= params.count {
        return;
    }
    var seed = pcg(index ^ pcg(params.frame));
    let position = (vec3<f32>(random(&seed), random(&seed), random(&seed)) - 0.5) * params.spawn_size;
    // Uniform on the unit sphere, so the flock starts disordered.
    let z = random(&seed) * 2.0 - 1.0;
    let angle = random(&seed) * 2.0 * PI;
    let radius = sqrt(max(1.0 - z * z, 0.0));
    let direction = vec3<f32>(radius * cos(angle), radius * sin(angle), z);
    let phase = f32(pcg(index ^ pcg(params.frame ^ 0xa511e9b3u)) >> 8u) / 16777216.0;
    let spawn_id = params.groups.y + id.x;
    let group = spawn_id % select(params.groups.x, 8u, params.groups.x == 0u);
    boids[index] = Boid(position, 0.0, direction * params.max_speed, 0.0, vec4<f32>(0.0, 0.0, 0.0, 1.0), vec4<f32>(0.0, 0.0, 0.0, phase));
    arena[index] = vec4<u32>(spawn_id, group, 0u, 0u);
    write_order(params.groups.z, index, index);
    // Spawn-only maintenance must be drawable while the simulation stays paused.
    instances[index] = Instance(vec4<f32>(position, bitcast<f32>(0x3f80ffffu | (select(group, 0u, params.groups.x == 0u) << 16u))), boids[index].rotation, vec4<f32>(phase, 0.0, 0.0, 0.0));
}

// Settings maintenance remains available while paused, without sorting or changing motion/phase.
@compute @workgroup_size(256)
fn update_groups(@builtin(global_invocation_id) id: vec3<u32>) {
    let index = id.x;
    if index >= params.count { return; }
    var group = 0u;
    if params.groups.x != 0u {
        let record = read_order(params.groups.z, index);
        arena[record].y = arena[record].x % params.groups.x;
        group = arena[record].y;
    }
    let speed = bitcast<u32>(instances[index].position_speed_group.w) & 0xffffu;
    instances[index].position_speed_group.w = bitcast<f32>(0x3f800000u | speed | (group << 16u));
}

@compute @workgroup_size(256)
fn clear_counts(@builtin(global_invocation_id) id: vec3<u32>) {
    if id.x <= params.table_mask {
        atomicStore(&counts[id.x], 0u);
    }
}

@compute @workgroup_size(256)
fn count(@builtin(global_invocation_id) id: vec3<u32>) {
    if id.x >= params.count {
        return;
    }
    let hash = hash_cell(cell_of(boids[id.x].position));
    slots[id.x] = vec2<u32>(hash, atomicAdd(&counts[hash], 1u));
}

@compute @workgroup_size(256)
fn count_separation(@builtin(global_invocation_id) id: vec3<u32>) {
    if id.x >= params.count {
        return;
    }
    let hash = hash_cell(separation_cell(boids[id.x].position));
    slots[id.x] = vec2<u32>(hash, atomicAdd(&counts[hash], 1u));
}

// Inclusive scan of `scratch` across the workgroup (Hillis-Steele).
fn scan_scratch(thread: u32) {
    for (var offset = 1u; offset < WORKGROUP; offset <<= 1u) {
        var value = 0u;
        if thread >= offset {
            value = scratch[thread - offset];
        }
        workgroupBarrier();
        scratch[thread] += value;
        workgroupBarrier();
    }
}

// One workgroup per block of SCAN_BLOCK cells; each thread owns two cells.
@compute @workgroup_size(256)
fn scan_blocks(
    @builtin(local_invocation_id) local: vec3<u32>,
    @builtin(workgroup_id) group: vec3<u32>,
) {
    let thread = local.x;
    let first = group.x * SCAN_BLOCK + thread * 2u;
    let a = atomicLoad(&counts[first]);
    let b = atomicLoad(&counts[first + 1u]);
    scratch[thread] = a + b;
    workgroupBarrier();
    scan_scratch(thread);
    let before = scratch[thread] - a - b;
    starts[first] = before;
    starts[first + 1u] = before + a;
    if thread == WORKGROUP - 1u {
        block_sums[group.x] = scratch[thread];
    }
}

// A single workgroup turns the block totals into exclusive block offsets.
@compute @workgroup_size(256)
fn scan_block_sums(@builtin(local_invocation_id) local: vec3<u32>) {
    let thread = local.x;
    let per_thread = (params.block_count + WORKGROUP - 1u) / WORKGROUP;
    let first = min(thread * per_thread, params.block_count);
    let last = min(first + per_thread, params.block_count);
    var total = 0u;
    for (var i = first; i < last; i++) {
        total += block_sums[i];
    }
    scratch[thread] = total;
    workgroupBarrier();
    scan_scratch(thread);
    var running = scratch[thread] - total;
    for (var i = first; i < last; i++) {
        let value = block_sums[i];
        block_sums[i] = running;
        running += value;
    }
}

fn scatter_record(index: u32, source: u32) {
    if index >= params.count { return; }
    let slot = slots[index];
    let destination = cell_start(slot.x) + slot.y;
    sorted[destination] = boids[index];
    write_order(params.groups.w / 4u - source, destination, read_order(source, index));
}

@compute @workgroup_size(256)
fn scatter_fine(@builtin(global_invocation_id) id: vec3<u32>) {
    scatter_record(id.x, params.groups.z);
}

@compute @workgroup_size(256)
fn scatter(@builtin(global_invocation_id) id: vec3<u32>) {
    var source = params.groups.z;
    if params.max_neighbors != 0u { source = params.groups.w / 4u - source; }
    scatter_record(id.x, source);
}

// Two sorts return to the input half; one sort flips it. Matches GpuFlock::encode.
fn stepped_order_base() -> u32 {
    return select(params.groups.w / 4u - params.groups.z, params.groups.z, params.max_neighbors != 0u);
}

@compute @workgroup_size(256)
fn separate(@builtin(global_invocation_id) id: vec3<u32>) {
    let index = id.x;
    if index >= params.count {
        return;
    }
    var me = sorted[index];
    let radius = params.size * params.separation_distance;
    let home = separation_cell(me.position);
    var separation = vec3<f32>(0.0);
    var visited: array<u32, 27>;
    var visited_count = 0u;
    for (var n = 0u; n < 27u; n++) {
        let hash = hash_cell(home + neighbor_offset(n));
        var seen = false;
        for (var v = 0u; v < visited_count; v++) {
            if visited[v] == hash {
                seen = true;
            }
        }
        if seen {
            continue;
        }
        visited[visited_count] = hash;
        visited_count++;
        let start = cell_start(hash);
        let end = start + atomicLoad(&counts[hash]);
        for (var k = start; k < end; k++) {
            if k == index {
                continue;
            }
            let diff = me.position - sorted[k].position;
            let distance_squared = dot(diff, diff);
            if distance_squared > 0.0 && distance_squared < radius * radius {
                let distance = sqrt(distance_squared);
                separation += diff / distance * min(radius / distance - 1.0, MAX_SEPARATION_PUSH);
            }
        }
    }
    me.separation = vec4<f32>(separation, me.separation.w);
    boids[index] = me;
}

@compute @workgroup_size(256)
fn steer(@builtin(global_invocation_id) id: vec3<u32>) {
    let index = id.x;
    if index >= params.count {
        return;
    }
    let me = sorted[index];
    var group = 0u;
    if GROUPS_ENABLED { group = arena[read_order(stepped_order_base(), index)].y; }
    let position = me.position;
    var velocity = me.velocity;

    let separation_radius = params.size * params.separation_distance;
    let separation_squared = separation_radius * separation_radius;
    let alignment_radius = params.size * params.alignment_distance;
    let alignment_squared = alignment_radius * alignment_radius;
    let cohesion_radius = params.size * params.cohesion_distance;
    let cohesion_squared = cohesion_radius * cohesion_radius;
    var neighbor_squared = max(alignment_squared, cohesion_squared);
    if params.max_neighbors == 0u {
        neighbor_squared = max(neighbor_squared, separation_squared);
    }

    var separation = select(vec3<f32>(0.0), me.separation.xyz, params.max_neighbors != 0u);
    var alignment = vec3<f32>(0.0);
    var cohesion = vec3<f32>(0.0);
    var alignment_count = 0u;
    var cohesion_count = 0u;
    var neighbors = 0u;

    let home = cell_of(position);
    var visited: array<u32, 27>;
    var visited_count = 0u;
    // Own cell first. These are samples, not the nearest k; separation was already complete.
    for (var n = 0u; n < 27u; n++) {
        let hash = hash_cell(home + neighbor_offset(n));
        var seen = false;
        for (var v = 0u; v < visited_count; v++) {
            if visited[v] == hash {
                seen = true;
            }
        }
        if seen {
            continue;
        }
        visited[visited_count] = hash;
        visited_count++;

        let start = cell_start(hash);
        let end = start + atomicLoad(&counts[hash]);
        for (var k = start; k < end; k++) {
            if k == index {
                continue;
            }
            let other = sorted[k];
            let diff = position - other.position;
            let distance_squared = dot(diff, diff);
            if distance_squared >= neighbor_squared {
                continue;
            }
            if params.max_neighbors == 0u && distance_squared > 0.0 && distance_squared < separation_squared {
                let distance = sqrt(distance_squared);
                separation += diff / distance * min(separation_radius / distance - 1.0, MAX_SEPARATION_PUSH);
            }
            if GROUPS_ENABLED && group != arena[read_order(stepped_order_base(), k)].y {
                continue;
            }
            if distance_squared < alignment_squared {
                alignment += other.velocity;
                alignment_count++;
            }
            if distance_squared < cohesion_squared {
                cohesion += other.position;
                cohesion_count++;
            }
            neighbors++;
            if params.max_neighbors != 0u && neighbors >= params.max_neighbors {
                break;
            }
        }
        if params.max_neighbors != 0u && neighbors >= params.max_neighbors {
            break;
        }
    }

    var force = separation * (params.max_speed * params.separation_factor);
    if alignment_count > 0u {
        let average = alignment / f32(alignment_count);
        force += limit(
            normalize_or_zero(average) * params.max_speed - velocity,
            params.max_speed * params.alignment_factor,
        );
    }
    if cohesion_count > 0u {
        let offset = cohesion / f32(cohesion_count) - position;
        let distance = length(offset);
        force += limit(
            normalize_or_zero(offset) * params.max_speed * min(distance / 10.0, 1.0) - velocity,
            params.max_speed * params.cohesion_factor,
        );
    }
    if params.attractor_strength.x != 0.0 {
        let offset = (params.attractor_position_radius.xyz - position) / params.attractor_position_radius.w;
        let q = length(offset);
        if q < 1.0 {
            // Mirrors Attractor::force: peak magnitude max_speed * strength at q=1/3.
            force += offset * (params.max_speed * params.attractor_strength.x * 6.75 * (1.0 - q) * (1.0 - q));
        }
    }
    if OBSTACLES_ENABLED {
        let segment = velocity * params.obstacle_settings.y;
        let length_squared = dot(segment, segment);
        var avoidance = vec3<f32>(0.0);
        for (var i = 0u; i < params.obstacles.y; i += 1u) {
            let sphere = bitcast<vec4<f32>>(arena[params.obstacles.x + i * 2u]);
            let clearance = bitcast<vec4<f32>>(arena[params.obstacles.x + i * 2u + 1u]).x * params.obstacle_settings.z;
            var t = 0.0;
            if length_squared > 0.0 { t = clamp(dot(sphere.xyz - position, segment) / length_squared, 0.0, 1.0); }
            let offset = position + segment * t - sphere.xyz;
            let distance = length(offset);
            var normal = vec3<f32>(1.0, 0.0, 0.0);
            if distance > 0.0 { normal = offset / distance; }
            let surface = distance - sphere.w * params.obstacle_settings.z;
            let weight = 1.0 - clamp(surface / clearance, 0.0, 1.0);
            avoidance += normal * (weight * weight * (3.0 - 2.0 * weight));
        }
        force += limit(avoidance, 1.0) * (params.max_speed * params.obstacle_settings.x);
    }
    if FORMATION_ENABLED {
        let slot = arena[read_order(stepped_order_base(), index)].z;
        let offset = formation_target(slot, group) - position;
        let desired = normalize_or_zero(offset) * params.max_speed * min(length(offset) / params.formation_settings.x, 1.0);
        force += limit(desired - velocity, params.max_speed * params.formation_settings.y);
    }
    if PREDATORS_ENABLED {
        var flee = vec3<f32>(0.0);
        for (var i = 0u; i < params.predators.y; i += 1u) {
            let predator = bitcast<vec4<f32>>(arena[params.predators.x + i * 2u]);
            let mask = arena[params.predators.x + i * 2u + 1u].x;
            if (mask & (1u << group)) == 0u { continue; }
            let offset = position - predator.xyz;
            let distance = length(offset);
            var normal = vec3<f32>(1.0, 0.0, 0.0);
            if distance > 0.0 { normal = offset / distance; }
            let weight = 1.0 - clamp(distance / predator.w, 0.0, 1.0);
            flee += normal * (weight * weight * (3.0 - 2.0 * weight));
        }
        force += limit(flee, 1.0) * (params.max_speed * params.predator_settings.x);
    }
    var acceleration = force / params.mass;

    var theta = me.wander_theta;
    var phi = me.wander_phi;
    var seek_target = params.target_position;
    var seeking = params.follow != 0u;
    if !seeking && GROUPS_ENABLED {
        let group_target = params.group_targets[group];
        seeking = group_target.w != 0.0;
        seek_target = group_target.xyz;
    }
    if seeking && !FORMATION_ENABLED {
        let seek = normalize_or_zero(seek_target - position) * params.max_speed - velocity;
        acceleration += limit(seek, params.max_speed * params.seek_factor) / params.mass;
    } else {
        let walls = params.walls;
        let outside = (position < -walls) | (position > walls);
        if !FORMATION_ENABLED && !any(outside) {
            var seed = pcg(index ^ pcg(params.frame * 2654435761u + 1u));
            let bound = PI / 16.0 * sqrt(params.dt * REFERENCE_HZ);
            theta += (random(&seed) * 2.0 - 1.0) * bound;
            phi += (random(&seed) * 2.0 - 1.0) * bound;
            let center = normalize_or_zero(velocity) * params.wander_distance + position;
            let wander_target = center
                + vec3<f32>(cos(theta) * sin(phi), sin(theta) * sin(phi), cos(phi)) * params.wander_radius;
            let wander = normalize_or_zero(wander_target - position) * params.wander_speed - velocity;
            acceleration += limit(wander, params.max_speed * params.wander_factor) / params.mass;
        }
        let desired = select(velocity, select(vec3<f32>(-params.max_speed), vec3<f32>(params.max_speed), position < -walls), outside);
        if any(outside) && (!FORMATION_ENABLED || !seeking) {
            acceleration += limit(desired - velocity, params.max_speed * params.wall_avoid_factor) / params.mass;
        }
    }

    velocity = limit(velocity + acceleration * (params.dt * REFERENCE_HZ), params.max_speed);
    let next = position + velocity * params.dt;
    let direction = normalize_or_zero(velocity);
    var rotation = me.rotation;
    if any(direction != vec3<f32>(0.0)) {
        rotation = heading_rotation(direction);
    }
    boids[index] = Boid(next, theta, velocity, phi, rotation, vec4<f32>(0.0, 0.0, 0.0, me.separation.w));
    // Same 16-bit speed / reserved 7-bit group encoding as pack_speed_group on the CPU.
    let speed = clamp(length(velocity) / max(params.max_speed, EPSILON), 0.0, 1.0);
    let packed = 0x3f800000u | u32(floor(speed * 65535.0 + 0.5)) | (group << 16u);
    instances[index] = Instance(vec4<f32>(next, bitcast<f32>(packed)), rotation, vec4<f32>(me.separation.w, 0.0, 0.0, 0.0));
}
