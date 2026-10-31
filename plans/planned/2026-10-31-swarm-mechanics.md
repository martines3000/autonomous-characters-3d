# Add optional swarm mechanics with one CPU/GPU contract

This ExecPlan is a living document maintained according to `PLANS.md`. It designs all of Stage 7;
this worker implements **7a only**. The orchestrator owns review, commits and authorization of
subsequent workers. Owner order: 7a attractor, 7b groups, 7c obstacles, 7d formations, 7e predators.

## Purpose

Add optional, composable swarm behaviors in a single **Swarm** menu section. Every mechanic starts
off. Users can bend a wandering or following flock with a held mouse-position field, distinguish
several groups, steer around a few obstacles, assemble formations and flee visible predators.
CPU Naive/Octree remain the behavior reference; GPU stays the high-count implementation.

## Progress

- [x] (2026-10-31 20:03Z) Read guidance, umbrella/presentation plans and implementation seams;
  write the shared design before code.
- [x] (2026-10-31 20:12Z) Stage 7a: held cursor field, gizmo, controls, scenario commands and CPU/GPU tests.
- [x] (2026-10-31 20:12Z) Stage 7a: sequential release gates, GPU regressions, desktop proof and repeated 100k timings.
- [ ] Stage 7b: groups and persistent metadata (separate worker).
- [ ] Stage 7c: bounded sphere obstacles (separate worker).
- [ ] Stage 7d: formation slots and arrival (separate worker).
- [ ] Stage 7e: bounded predator objects and fleeing (separate worker).
- [ ] Orchestrator review and commits.

## Surprises & Discoveries

- GPU boids are re-sorted every step, possibly twice. Array indices cannot identify a member.
  `separation.xyz` is scratch; `separation.w` already carries an independent 24-bit animation phase.
- The pinned egui API names its hover check `is_pointer_over_egui`; a real egui Window is needed
  in the hover fixture. The corrected focus/hover/input tests pass.
- Compute already uses seven storage bindings. Separate obstacle and slot buffers would exceed
  the default eight-binding budget when combined; the design below reserves one shared buffer.
- Attraction can create dense wander-mode neighborhoods. The existing unlimited wander query is
  retained deliberately in 7a; a hidden neighbor cap would change steering semantics.

## Decision Log

- Decision: keep all mechanics disabled by default; append conditional terms without changing the
  existing force expression, neighbor laws, wall drag or integration. Test the disabled path explicitly.
  Rationale: correctness and baseline preservation precede performance and convenience.
  Date: 2026-10-31.
- Decision: hold **F**, with Off/Attract/Repel selection; reject Ctrl+left mouse.
  Rationale: F is unused and does not interfere with target picking, camera gestures or target movement.
  Both egui keyboard focus and pointer hover block cursor application. Scenario placement is an
  explicit persistent override, cleared by `attractor mouse`; it intentionally requires no held input.
  Date: 2026-10-31.
- Decision: add a 16-byte `vec4<u32>` metadata lane to Boid in **7b**, not 7a. Components are
  persistent spawn ID, group, formation slot and reserved role. Keep phase untouched.
  Rationale: group packing into phase would lose phase precision or identity space. Neither wander
  angle nor scratch is free. Explicit metadata is easier to audit than float bit payloads. Allocate
  monotonically increasing IDs within each flock lifetime, not sorted indices or current count;
  truncation then growth cannot reuse a retained ID. GPU/CPU transitions reset the lifetime.
  Date: 2026-10-31.
- Decision: preserve unlimited wander and existing follow cap in 7a. No automatic density policy.
  Rationale: cap changes need a separate owner decision and parity contract. Keep exact separation;
  document that sustained attraction may approach quadratic neighbor work. 7d must benchmark dense
  assembly before claiming high-count practicality and seek owner approval for any sampling change.
  Date: 2026-10-31.

## Assumptions and Open Questions

- The first groups implementation supports two groups (table capacity at most eight), one shared
  mesh, same-group alignment/cohesion, all-group separation. Validate mixed-group exact fixtures;
  defer arbitrary interaction matrices and exact population quotas.
- Obstacles are soft steering proxies, not rigid collisions. Validate penetration and clearance at
  named dt/speed settings; do not promise non-penetration. Begin with at most eight spheres.
- Formations use bounded arrival and unique slots, not optimal assignment. Determine convergence
  tolerance and acceptable dense-neighbor cost during 7d; no O(N²) matching.
- One to four predator control objects follow scripted paths or the known target. No capture,
  births/deaths, prey identity locks, centroid readback or unrestricted predator/prey pair search.
- Scaling spatial features cannot make normalized traversal time identical at 200 and 1M while
  preserving the existing fixed world-unit max speed. Validate matching normalized field shape and
  force bounds; do not silently scale vehicle speed or baseline forces to claim that equivalence.

## Outcomes & Retrospective

Design complete before implementation. Stage 7b–7e remain specifications, not delivered behavior.
Stage 7a is implemented and ready for orchestrator review, without commits or dependency changes.
Release suite: 97 passed, 7 ignored (eight new non-ignored tests and one new ignored GPU test).
All five device tests pass, including existing baseline/animation/separation/lifecycle tests and
new attract/repel parity through follow/wander and capped/uncapped paths. New parity worst position
error is 1.53e-5 world units; worst velocity error is 1.17e-5. Formatting, warning-free release build,
native startup smoke, Clippy (seven existing warnings, duplicated for the test target) and diff
checks pass. Held F, release, missing cursor, unfocused window, egui hover/focus and plane geometry
have headless system tests. Native key/pointer injection was unavailable; desktop scenarios use
explicit world placement through the harness.

The 38-second desktop script captures GPU 20k and Octree 1,500 attraction/repulsion before and after
3.5 seconds, plus simultaneous following and attraction. The cyan/orange ring and center remain
visible. A live GPU flock paused with a newly placed field has zero differing scene pixels across
two seconds; resuming produces a nonzero difference. All five scenario apps reached `exit`, and
subsequent process checks found no app, Cargo or rustc. Startup smoke terminated its own instance
normally after ten seconds. HEAD remains `1c0d618` on the requested branch.

Practical cost is material: the 100k attraction scene is about 12.6–12.7 ms versus about 1 ms off.
These timings include the flock becoming dense and the visible ring, rather than isolating field
arithmetic. The existing exact wander query is intentionally retained. Dense 1M attraction and
combinations with later formations/predators are unmeasured; subsequent workers must address density
budgets explicitly, with owner approval before changing sampling or separation semantics.

## Context and Orientation

`src/vehicle.rs::flocking_force` returns flock force, divided by mass by `flock`; `movement` adds
seek or wander/wall acceleration. `update` integrates acceleration with `dt * 60`, clamps velocity
to max speed, then advances position with dt. `step_cpu` repeats this through `CpuStep` using the
presentation `SimulationFrame`. GPU `steer` mirrors that ordering; `GpuParams` is encoded with
encase, and `params_encode_with_wgsl_uniform_offsets` must grow with every uniform edit.
`src/target.rs` owns left-click selection and WASD/Space/Left Shift; `main.rs` owns camera mouse
input; `presentation.rs` owns P/O/1–5 and the virtual simulation clock. Scenario scripts execute
before input. Rendering already reserves bits 16–22 of Instance.w for group; speed uses bits 0–15.

## Plan of Work

### Shared force contract

Let S = `vehicle::world_scale(count)`, V = `vehicle_max_speed`, M = `vehicle_mass`. User radius,
clearance, formation extent and analytic path dimensions are reference-world units multiplied by S.
Explicit scenario positions are already world coordinates and are never silently rescaled. Distance
ratios use these scaled lengths; strength is dimensionless relative to V. Existing physical neighbor
radii retain their current `vehicle_size` rules. Feature forces are evaluated from the current
substep snapshot, added in fixed order, divided by M exactly once, and integrated with `dt * 60`.
Their force units are the repository's reference-tick steering units; acceleration per simulated
second is 60 F/M. Do not multiply feature strength by real dt, speed or S a second time.
Each bounded mechanic uses a cap V times its strength; obstacle/predator lists cap their summed
term. Preserve the existing uncapped crowd separation and per-term limits; there is no new global
force clamp. The final existing speed clamp remains V. New terms enter `flocking_force` before its
mass division in order: attractor, obstacles, formation, predators. Group filtering changes neighbor
selection only when enabled. Port the same arithmetic/order to WGSL.

| Mechanic | Inputs and force shape | Weight, scale and bounds |
| --- | --- | --- |
| 7a attract/repel | Position c, radius R, strength k, sign s. Let u=(c-p)/R and q=length(u). For q<1: F=u * (V*k*s*6.75*(1-q)^2); otherwise zero. | R=radius*S. Peak magnitude V*k at q=1/3; continuous at center, value and derivative zero at cutoff. Repel is exactly negative attract. Default radius 60, strength 4, mode Off. Add to flock force, including while seeking. |
| 7b groups | Carried group membership; separate across all groups, align/cohere within group. Optional per-group target uses the existing seek law. | Existing force/radius factors remain baseline values initially. Group target offsets are reference units*S; explicit world targets stay world units. Disabled means no filtering/table reads. |
| 7c obstacles | Sphere center/radius, clearance, look-ahead horizon; nearest point on segment p to p+v*t. Outward normal at that point, deterministic +X at exact center. Smoothstep of 1-clamp(surface_distance/clearance,0,1), full force inside sphere. | Radius and clearance scale by S; look-ahead seconds are simulated seconds. Sum normal weights then clamp magnitude to V*k. Eight spheres maximum initially. Soft avoidance is additive, including during seek/formation. |
| 7d formations | Unique carried slot indexing a sphere/helix target; center, extent, arrival radius, morph fraction. Desired velocity = normalized offset*V*min(distance/arrival_radius,1); steer = desired-v. | Extent/arrival radius scale by S; cap V*k. Enabled formation replaces global/group seek and wander (one navigation goal), retains existing wall term on the wander branch, flocking and other feature terms. Off uses the original seek/wander branch without extra arithmetic. Morphs advance only in simulated time. |
| 7e predators | At most four visible control objects, position/radius and prey-group mask. Outward smoothstep field; deterministic +X at coincidence. All relevant predators evaluated, never hidden by neighbor sample caps. | Threat radius/path dimensions scale by S; summed flee capped at V*k. Add to existing navigation/flocking. Predators' paths use simulated seconds and stop on pause. No per-boid role cost until an actual boid-role consumer exists. |

7c/7e use smoothstep h(t)=t*t*(3-2*t). These are proposed initial laws; a later worker must document
any behavior revision before code and add independent CPU/GPU fixtures. 7a's law is fixed here.

### Data placement and lifecycle

7a lives in `GlobalState::attractor`: mode, dimensionless strength, reference radius, current cursor
point and optional scenario point. Append two Vec4 uniforms: world point/radius and signed strength
with three reserved lanes. An inactive field encodes signed strength zero and branches before
force evaluation. No Boid/Instance layout changes in 7a, no storage buffers or per-boid field data.

For 7b–7e, add scalar enable/strength/extent controls beside the attractor in `GlobalState`.
A CPU `SwarmScene` resource owns the typed group table, obstacle vector, formation-slot vector and
predator objects/path state; extracted render data contains only the packed uniform/arena snapshot.
CPU kernels read the same scene snapshot for each substep. If a moving obstacle/predator path changes
between substeps, evaluate its simulated-time position analytically from the substep clock on both
backends, rather than advancing it once per rendered frame. Formation target evaluation uses the
same substep morph time. The disabled path never reads these collections.

| Mechanic | CPU data | GPU data |
| --- | --- | --- |
| Attractor | `GlobalState::attractor`, no per-vehicle state | Two appended Vec4 uniforms; no storage or metadata |
| Groups | `SwarmScene` fixed group table; carried ID/group/slot component | Uniform group table and enable/count; Boid metadata; Instance.w group bits |
| Obstacles | `SwarmScene` bounded typed sphere vector and path state | Uniform count/arena offset/strength/clearance; sphere records in shared arena |
| Formations | `SwarmScene` dense Vec<Vec4> slot targets and morph/path parameters; carried slot component | Uniform center/extent/arrival/morph/time and slot offset/count; slot targets in arena; carried Boid slot |
| Predators | `SwarmScene` bounded control objects, group masks and simulated path state | Uniform count/offset/strength/path time; object records in arena; no prey-history buffer |

7b adds CPU metadata components and the 80-byte GPU Boid (currently 64). Scatter, fine separation,
steer, spawn, upload/readback and paused growth copies preserve every metadata lane. Copy group
into the existing Instance.w bits with no instance stride growth (currently 48). ID allocator state
is separate from random frame; spawn-only paused maintenance allocates IDs without advancing motion.
Group count edits deterministically map existing membership modulo new group count on CPU/GPU;
when disabled metadata persists but all vehicles behave as the original one group. Growth assigns
new IDs and groups; sorted-prefix truncation keeps surviving metadata, with no exact quota promise.

7d assigns dense unique slots after count changes by current record order, via an explicit maintenance
pass, and retains them through subsequent sorts. ID survives slot reassignment. CPU does equivalent
one-time assignment. Count zero clears assignments; mode respawns rebuild them. Never use ID modulo
count: truncated/grown lifetimes can have gaps and collisions. Paused count changes may assign slots
without moving existing vehicles. Unique-slot/sort/growth tests are mandatory.

Keep small group tables and feature controls in appended uniform fields (every field gets an encase
byte-offset assertion matching Naga's WGSL layout). For variable data, 7c introduces the eighth and
final storage binding: a read-only `array<vec4<f32>>` shared scene-data arena. Uniform offsets/counts
name disjoint obstacle, formation-slot and predator ranges. CPU keeps typed vectors and packs them
on change; never upload per-boid state each frame. Sphere entries use center/radius plus clearance
and parameters; slots use one Vec4 target each; predator entries include position/radius and a
bitcast group mask in a separate lane. Later workers define and test exact record strides before
binding them. Empty lists have a legal one-record dummy buffer and zero count. Validate ranges and
actual adapter limits; do not add a ninth compute storage binding.

Calculated metadata cost, to measure in 7b: +16 bytes in each of two state arrays = +1 MiB at 20k
(capacity 32,768), +32 MiB at 1M (capacity 1,048,576), plus 25% more bytes per boid state copy.
An 80 MiB capacity-rounded state buffer remains below the default 128 MiB binding limit. One Vec4
slot per capacity entry adds 0.5/16 MiB. Existing instance stride remains 48 bytes. Record actual
adapter allocation/bandwidth/frame results before accepting this tradeoff; no speculative layout
migration in 7a. Animation phase stays bit-identical and independent of group/slot/ID.

### Interaction matrix

All feature settings persist through count/mode changes. CPU↔GPU respawns, Naive↔Octree preserves
vehicles. Pause skips every simulation step, including feature path/morph clocks; settings, cursor,
gizmos, target and camera remain live. Speed uses existing equal SimulationFrame substeps. New
vehicles and metadata may be initialized during pause without stepping retained vehicles.

| Mechanic | Seek/follow | Pause and speed | Groups | Mode/count changes |
| --- | --- | --- | --- | --- |
| Attractor | Additive; seek still runs | Point/gizmo live, motion frozen; every substep samples same frame point | Applies to every group | Keep world point; recompute radius*S; no reset |
| Groups | Global selected target overrides group targets; formation overrides both | Tables live; movement follows simulated time | Same-group align/cohere, all-group separate | Respawn on backend change; retained membership on CPU mode switch; new IDs/groups on growth |
| Obstacles | Additive soft avoidance | Edits live; moving proxy paths freeze | Same avoidance for all groups | Keep proxies/world centers; rescale reference dimensions |
| Formations | Replaces seek/wander; follow selection remains stored for exit | Morph freezes; editing and slot maintenance stay live | Each group gets its own contiguous slot range/center | Reassign dense unique slots on count/group-count edits; no optimal matching |
| Predators | Additive flee, navigation recovers after passing | Paths/flee use simulation steps | Prey mask selects affected groups; predators are separate control objects | Keep object state; rescale reference dimensions; no flock population changes |

| Pair | Composition |
| --- | --- |
| Attractor × groups | Same field affects all groups; no implicit selection mask in 7a. |
| Attractor × obstacles | Add both bounded forces; obstacle steering can bend attraction, no hard collision guarantee. |
| Attractor × formations | Field temporarily deforms assembly; arrival returns members to slots on release. |
| Attractor × predators | Add field and flee; neither silently changes the other's strength. |
| Groups × obstacles | All groups avoid the same proxies; cross-group separation remains complete. |
| Groups × formations | Disjoint slot ranges per group; same formation law and one shared slot arena. |
| Groups × predators | Explicit prey-group mask; no skipped threats due to alignment/cohesion caps. |
| Obstacles × formations | Avoidance competes additively with arrival; blocked slots may never converge. |
| Obstacles × predators | Both terms apply to prey; control-object paths need separate obstacle handling if desired later. |
| Formations × predators | Flee deforms formation; arrival regroups when threats leave. |

### UI and scenario surface

One Swarm collapsible contains attractor controls in 7a; later subsections add Groups, Obstacles,
Formation and Predators without separate top-level windows. Every mode has an explicit Off state.
Controls panel shows hold F, its selected mode and the plane rule. Scenario proposal:

- 7a: `set attractor_mode off|attract|repel`, `set attractor_strength 0..20`,
  `set attractor_radius 1..200`, `attractor x y z` (persistent world override), `attractor mouse`.
- 7b: `set groups off|2` (later up to eight), `group_target group x y z`, `group_color group RRGGBB`.
- 7c: `obstacle add x y z radius`, `obstacle clear`, `set obstacle_strength k`,
  `set obstacle_clearance r`; bounded list and reference radii validated before startup.
- 7d: `set formation off|sphere|helix`, `formation_center x y z`, `set formation_extent r`,
  `set formation_strength k`, `set formation_morph t` (0..1).
- 7e: `predator add x y z radius`, `predator clear`, `set predator_strength k`,
  `set predator_path off|orbit`, `set predator_groups mask`.

Only 7a commands are implemented by this worker. Later command ranges and record layouts are
finalized in their stage checkpoints; parsing always rejects non-finite/out-of-budget data.

## Concrete Steps

Before EVERY Cargo command and app run: `pgrep -a cargo; pgrep -a rustc; pgrep -a autonomous`.
Wait if occupied. Add `$HOME/.cargo/bin` to PATH. Run sequentially, jobs at most four:

1. `CARGO_BUILD_JOBS=4 cargo fmt --all -- --check`
2. `CARGO_BUILD_JOBS=4 cargo test --release --locked`
3. `CARGO_BUILD_JOBS=4 cargo build --release --locked`
4. `python3 scripts/smoke-native.py`
5. `git diff --check`
6. `CARGO_BUILD_JOBS=4 cargo clippy --release --locked --all-targets`
7. `CARGO_BUILD_JOBS=4 cargo test --release --locked gpu_ -- --ignored --nocapture --test-threads=1 --skip gpu_flocking_benchmark`
8. Desktop scripts via `AC3D_SCRIPT` and `CARGO_MANIFEST_DIR`, each ending with `exit`.

## Validation and Acceptance

Default Off takes the original force arithmetic and keeps existing CPU/GPU parity fixtures passing.
CPU tests cover peak/boundary/center, finite force, scale equivalence, zero strength, exact sign
inversion, inactive bit identity, held-key release and egui exclusions, camera plane and pause/speed.
New ignored GPU parity test covers active attract/repel, follow/wander, capped/uncapped paths and
paused preservation. Extend uniform byte-offset tests. Build has no warnings; Clippy stays at seven.
Desktop: before/after attract/repel with ring at 20k GPU and smaller Octree, follow plus field, paused
point/gizmo with identical flock pixels until resume. Repeat on/off frame measurements at 100k GPU,
record profile, adapter, viewport, seed policy, dt, warmup and load. Verify no app remains after each run.

## Idempotence and Recovery

No commit, push, merge, reset or discard by this worker. All evidence/scripts live outside the repo.
Rerun scripts only after checking processes. Orchestrator reviews the uncommitted diff. Disable the
feature to recover original behavior; no data migration or persistent user files are introduced.

## Artifacts and Notes

Evidence directory:
`/tmp/claude-1000/-home-martines-Projects-autonomous-characters-3d/af1dacb4-dc47-4bb5-8e8c-02c6cd1b95e8/scratchpad/shots/stage7a/`.

Evidence files in that directory:

- `desktop.ac3d`, `desktop.log`: all desktop cases and their normal Exit command.
- `gpu-attract-{before,after}.png`, `gpu-repel-{before,after}.png` (20,000 GPU).
- `octree-attract-{before,after}.png`, `octree-repel-{before,after}.png` (1,500 Octree).
- `gpu-follow-attract-{before,after}.png`: green selected target and separate cyan field.
- `gpu-paused-{a,b}.png`, `gpu-resumed.png`; `pixel-comparison.txt`: ImageMagick AE = 0 for the
  paused pair, nonzero after resume, crop `960x920+420+100` excludes UI at 1776×1119.
- `tests.log`, `build.log`, `smoke.log`, `clippy.log`, `gpu.log`, `verification.txt`.
- `timing-{off,on}-{1,2}.ac3d` and corresponding logs; `timing-load.txt`.

Repeated rendered frame intervals: release, 100,000 GPU Dart vehicles, Studio/Aurora, heading color,
physical/render size 1, 4× MSAA, 1776×1119, immediate presentation, camera yaw 0° / pitch 10° /
distance 2,500. AMD Radeon RX 7900 XT (RADV NAVI31), Vulkan, Mesa 26.2.5-arch1.1; Ryzen 9 7900X,
Linux 7.2.8-9-omarchy. Each case starts a fresh app, PCG spawn at frame 0, 1× simulation, real-time
variable dt (not a replayed trajectory or seeded CPU test), three-second warmup, four-second sample.
Attraction is at origin, strength 4, reference radius 60 (world radius about 476.22). GPU scatter
order and wall-clock stepping are nondeterministic. No competing Cargo/app process ran. Load
averages at the four launches: 0.94/1.04/0.86, 1.02/1.05/0.87, 1.38/1.13/0.89, 1.86/1.24/0.93.
No GPU timestamp instrumentation was enabled; these are whole-frame intervals, not compute time.

| Field | Run 1 mean / p50 / p95 ms (frames) | Run 2 mean / p50 / p95 ms (frames) |
| --- | --- | --- |
| Off | 0.990 / 0.962 / 1.219 (4,039) | 1.005 / 0.964 / 1.268 (3,980) |
| Attract | 12.605 / 11.062 / 21.961 (318) | 12.744 / 11.253 / 22.146 (315) |

## Interfaces and Dependencies

7a: new `src/swarm.rs` owns AttractorMode/settings, force kernel, cursor input, gizmo and UI.
`GlobalState` gains the off-default attractor settings. CPU flocking adds a conditional term; GPU
Params appends world point/radius and signed strength Vec4s, mirrored by WGSL. Scenario adds typed
commands. No dependency, Boid stride, Instance stride or baseline simulation-setting changes.

---

Revision notes:

- 2026-10-31: Initial Stage 7 contract and Stage 7a implementation/validation scope, written before code.

- 2026-10-31: Stage 7a complete; recorded release/device checks, desktop pixel proof and repeated rendered frame timings. Later sub-stages remain open.
