# Add optional swarm mechanics with one CPU/GPU contract

This ExecPlan is a living document maintained according to `PLANS.md`. It designs all of Stage 7;
Stage 7a is committed as `d657d5d`; 7b is committed as `fd9921a`; the current worker implements **7c only**. The orchestrator owns review, commits and authorization of
subsequent workers. Owner order: 7a attractor, 7b groups, 7c obstacles, 7d formations, 7e predators.

## Purpose

Add optional, composable swarm behaviors in a single **Swarm** menu section. Every mechanic starts
off. Users can bend a wandering or following flock with a held mouse-position field, distinguish
several groups, steer around a few obstacles, assemble formations and flee visible predators.
CPU Naive/Octree remain the behavior reference; GPU stays the high-count implementation.

## Progress

- [x] (2026-10-31 21:50Z) Stage 7b review blocker: compared field-only loads, split/ping-pong metadata and packed order maps; restored 64-byte state and Off performance.
- [x] (2026-10-31 21:50Z) Review blocker: four Off/two On measurements per case, seven device regressions under eight-binding limits and full gate pass.

- [x] (2026-10-31 20:03Z) Read guidance, umbrella/presentation plans and implementation seams;
  write the shared design before code.
- [x] (2026-10-31 20:12Z) Stage 7a: held cursor field, gizmo, controls, scenario commands and CPU/GPU tests.
- [x] (2026-10-31 20:12Z) Stage 7a: sequential release gates, GPU regressions, desktop proof and repeated 100k timings.
- [x] (2026-10-31 20:33Z) Stage 7b implementation and tests written; 102 CPU tests pass. Device regressions and desktop checks remain below.
- [x] (2026-10-31 20:46Z) Stage 7b: repeated before/after CPU/GPU benchmarks, rendered timings and desktop verification; 102 release tests and all seven device tests pass.
- [x] (2026-10-31 22:17Z) Stage 7c: bounded sphere law, resource/arena lifecycle, UI/gizmos and scenarios.
- [x] (2026-10-31 22:17Z) Stage 7c: 109 release tests and all nine ignored GPU tests pass, including new scene snapshot/parity/arena regressions.
- [x] (2026-10-31 22:17Z) Stage 7c: repeated 7b/Off/eight-sphere benchmarks, rendered/desktop evidence and full gates pass. Awaiting owner review/commit.
- [ ] Stage 7d: formation slots and arrival (separate worker).
- [ ] Stage 7e: bounded predator objects and fleeing (separate worker).
- [ ] Orchestrator review and commits.

## Surprises & Discoveries

- Stage 7c: egui runs its primary pass in PostUpdate (`bevy_egui-0.43.1/src/lib.rs:1145`).
  Extracting a new scene with Update-produced count/settings could mix snapshots during UI edits.
  Derive obstacle uniforms in the render world from the same extracted scene used for uploads;
  preserve the extracted boid count/world scale for the actual step. A regression checks current
  counts, scaling, strength-zero and clear resetting stale enable fields. This moves scene settings
  assembly only; the measured shader and headless step/fixture are unchanged.


- Orchestrator review blocks the original 80-byte layout: optional groups must not impose the
  measured 24–32% Off wander penalty. The previous acceptance decision is superseded. Candidate
  investigation starts with explicit neighbor field loads, then 64-byte state plus split metadata
  if needed. Target: Off 100k/1M wander/follow within roughly 3–5% of the 64-byte baseline.

- Stage 7b measurement: carrying the 80-byte record costs about +24%/+32% in Off wander
  at 100k/1M on this GPU. The explicit identity/phase contract is retained, with this cost exposed
  for owner review. No hidden density approximation was introduced.
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
- Original decision (superseded by the performance review below): add a 16-byte `vec4<u32>` metadata lane to Boid in **7b**, not 7a. Components are
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

- Decision (7b): store the fixed eight-entry group table with its controls in `GlobalState::groups`.
  Reserve `SwarmScene` for variable-length data in 7c. All entries and settings remain Copy.
  Rationale: the table needs no independent lifecycle or extraction resource. Existing pure
  `gpu_params` and CPU parity fixtures read exactly the same settings. UI edits and per-frame
  maintenance write resources/components only when their values change.
  Date: 2026-10-31.
- Decision (7b): dormant spawn membership is spawn ID modulo eight. First enabling maps that
  membership modulo the requested 2–8 count; subsequent count edits map current membership modulo
  the new count. Off preserves stored membership but uses group zero for rendering and no filtering.
  Growth assigns ID modulo current count (eight while off). Increasing a count does not rebalance
  retained members: new groups can remain empty until growth. IDs are monotonic within a nonempty
  backend lifetime; count zero/backend change starts a new lifetime. Slots/reserved initialize zero.
  Rationale: preserve surviving records, avoid sorted-index identity and retain exact Off payloads.
  Date: 2026-10-31.
- Decision (7b): reject other groups after exact separation and before alignment/cohesion or sample
  counting. The fine separation pass remains unfiltered. Follow caps apply when globally following
  or any enabled group has a target; no-target group wander shares that cap in the latter case.
  Without targets, wander remains unlimited. CPU stays exact, as with existing global following.
  Rationale: one group comparison/early continue needs no extra grid or storage, and rejected
  neighbors cannot exhaust the same-group quota. Filling that quota can scan more candidates,
  especially for rare groups. Capped results remain order-dependent samples; dense eight-group
  follow cost is not established by the two-group desktop measurements.
  Date: 2026-10-31.
- Decision (7b): group targets use the unchanged seek force instead of wander/wall steering for
  members with a target. Global selected target overrides all group targets. An unset target leaves
  normal wander/walls; groups Off ignores every group target. Explicit target positions are world
  coordinates; UI provides a per-group enable switch and position editors. Targets can be cleared
  with `group_target group off`. Group indices are zero-based (0–7).
  Rationale: preserve the existing seek law and avoid simultaneously running two navigation goals.
  Date: 2026-10-31.

- Decision (7c, before implementation): `SwarmScene` owns up to eight static spheres, dimensionless
  obstacle strength (default 4), default clearance (10 reference units) and visibility (true).
  Each sphere stores world center, reference radius and reference clearance. A global clearance edit
  updates all existing spheres and the default for new ones; individual sphere editors may override it.
  Empty list is Off. Strength zero also bypasses force evaluation. Fixed look-ahead is one simulated
  second; evaluate the closest point on p to p+v*1s, using current velocity without dt scaling.
  Surface distance is distance(center, closest)-radius*S. Weight is smoothstep of
  1-clamp(surface_distance/(clearance*S),0,1); normal is outward from center at closest point,
  with +X at exact center. Sum weighted normals, clamp magnitude to one, then multiply by V*k.
  Add after attractor and before mass division, with unchanged seek/wander/walls/integration.
  Rationale: direct shared law with bounded O(8N) work; soft avoidance cannot promise non-penetration.
  Date: 2026-10-31.
- Decision (7c): binding-8 tail starts at `5C/2` vec4 units and reserves 16 vec4s (256 bytes).
  Each sphere uses bitcast-u32 center.xyz/reference-radius followed by reference-clearance/three
  reserved float lanes. Append uniform offset/count and strength/look-ahead/world-scale settings.
  Extract `SwarmScene` separately; cache only sphere records in `GpuFlock`. Upload records only
  when that list changes or a new capacity relocates the tail, including while paused. Settings
  and gizmo visibility changes do not upload records or write GlobalState. Retain eight storage
  bindings, 64-byte Boids and 48-byte instances. Specialize Off/On obstacle steering alongside
  the existing group specialization; measure Off against committed 7b before acceptance.
  Rationale: preserve the accepted default path, metadata/order-map identity and later scene arena.
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

Design complete before implementation. Stages 7a–7c are implemented; 7d–7e remain specifications.
Stage handoffs and validation evidence are recorded below.
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

### Stage 7b handoff

Stage 7a was reviewed and committed by the orchestrator as `d657d5d`, including cursor writes only
when the point changes. Stage 7b now implements groups Off/2–8, fixed colors/targets, metadata and
identity transport, and paused membership maintenance. The fixed table lives in `GlobalState`;
variable-length `SwarmScene` storage remains deferred to 7c. Group colors reuse the one draw and
48-byte instance stream. Off preserves baseline force arithmetic and zero render group bits.
No Stage 7c–7e code or dependencies were added. The worker has not committed/pushed/merged/reset
or discarded repository changes; HEAD remains `d657d5d` on the requested branch.

Final gates pass: fmt check; 102 release tests (nine ignored, including benchmarks); warning-free
release build; ten-second native smoke; diff check; Clippy seven pre-existing warnings (seven
repeated for the test target). The ignored GPU suite was run once at the end: all seven passed.
The new mixed fixture covers three groups, two explicit targets plus wander, global seek override,
unlimited and nontruncating capped paths, and carried metadata identity. A deliberately adversarial
cap=1 fixture puts foreign neighbors in the home cell and the only eligible peer in the next cell:
foreign neighbors cannot consume the cap. New group parity worst position error is about 1.53e-5,
velocity error below 1e-5. The metadata fixture covers both sorts, exact phase formula, typed upload,
paused capacity growth, every paused membership mapping, rendering group bits, truncation,
non-reused IDs on regrowth and count-zero lifetime reset. CPU tests cover force filtering,
cross-group separation, membership/lifecycle, global override, paused group seek, and unchanged
component ticks after maintenance settles. No new per-frame input writes were introduced.

Seven desktop screenshots show 20k GPU groups at spawn and after 20 seconds (homogeneous cyan/orange
clusters, without forcing two global islands), split group targets, both groups around the selected
global target despite their still-enabled separate targets, attraction moving both groups to the
cursor field, and 1,500 Octree members split around their targets. All five scripted apps reached
`exit`; final process checks are empty. Eight groups share the same fixed table/pipeline, but dense
8-group performance and exact population balancing remain unclaimed. Increasing group count can
leave newly available groups empty until growth. CPU benchmarks retain their existing practical
counts; 100k/1M measurements are GPU only. Later mechanics remain untouched.

### Stage 7c handoff

Stage 7b was reviewed/committed by the orchestrator as `fd9921a`; its accepted Off overhead and
non-rebalancing group-count policy remain unchanged. Stage 7c now delivers only static sphere
avoidance, scene persistence, change-only arena uploads, paused wireframes, UI and validated commands.
The empty/zero-strength path compiles out. Compared with 7b, repeated 1M wander/follow Off medians
change by -0.08%/-0.53%; short 100k timings vary and do not establish a speedup. Eight-sphere
headless and whole-frame costs are recorded below. No density approximation, wall-drag retuning,
dependency change, formation or predator behavior was added.

Final gates: fmt check; 109 release tests passed (11 ignored, including benchmarks); release build
without warnings; Clippy seven existing warnings, repeated for the test target; ten-second native
startup smoke; diff check. The single final ignored GPU suite passed all nine tests. New mixed
obstacle/group/attractor parity worst position error is 1.53e-5 world units and velocity error
8.81e-6; arena tests verify exact records and upload counts through count/capacity/settings changes.
Desktop scripts and all four final rendered timing apps reached `exit`; paused scene crop has zero
pixel differences across two seconds. No Cargo/rustc/app remains. The worker has not committed,
pushed, merged, reset or discarded repository changes; HEAD remains `fd9921a`.

Soft avoidance allows penetration under competing forces. Only static spheres and a fixed one-second
look-ahead are delivered. Other GPUs and worst-case multi-feature density remain unmeasured.
7d/7e must extend the shared tail after the sphere reservation and preserve the same snapshot,
change-detection and capacity-relocation rules.

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
The fixed group table lives in `GlobalState::groups`; a CPU `SwarmScene` resource introduced in 7c
owns the obstacle vector, formation-slot vector and
predator objects/path state; extracted render data contains only the packed uniform/arena snapshot.
CPU kernels read the same scene snapshot for each substep. If a moving obstacle/predator path changes
between substeps, evaluate its simulated-time position analytically from the substep clock on both
backends, rather than advancing it once per rendered frame. Formation target evaluation uses the
same substep morph time. The disabled path never reads these collections.

| Mechanic | CPU data | GPU data |
| --- | --- | --- |
| Attractor | `GlobalState::attractor`, no per-vehicle state | Two appended Vec4 uniforms; no storage or metadata |
| Groups | `GlobalState::groups` fixed group table; carried ID/group/slot component | Uniform group table and enable/count; separate metadata table resolved through sorted order maps; Instance.w group bits |
| Obstacles | `SwarmScene` bounded typed sphere vector and path state | Uniform count/arena offset/strength/clearance; sphere records in shared arena |
| Formations | `SwarmScene` dense Vec<Vec4> slot targets and morph/path parameters; carried slot component | Uniform center/extent/arrival/morph/time and slot offset/count; slot targets and carried metadata slot in arena |
| Predators | `SwarmScene` bounded control objects, group masks and simulated path state | Uniform count/offset/strength/path time; object records in arena; no prey-history buffer |

7b adds CPU metadata components but keeps the GPU Boid at 64 bytes. A persistent `vec4<u32>`
metadata table stores spawn ID, group, formation slot and reserved role, independently of animation
phase. Each state record carries an indirect table index in a parallel packed `u32` order map.
Scatter copies that index alongside the state on every sort, including while groups are Off.
The Off-specialized steering pipeline performs no metadata/order-map reads. Groups On resolves
its own and candidate group lanes through the order map; separation remains state-only and exact.
Steering/separation never copy metadata back. One sort toggles the current map; two sorts return
to the original half. The host tracks this across capped/uncapped transitions and substeps.

Count changes gather retained metadata into scratch, then compact it into the table in two distinct
dispatches before spawn. This maintenance also runs while paused. Table row indices can change;
spawn ID, membership, slot, reserved and phase do not. New rows initialize all metadata lanes;
monotonic spawn IDs never reuse a survivor's ID. Capacity growth copies state, table, current order
map and instances before compaction/spawn. Upload/readback fixtures resolve the same indirection.
Future metadata consumers must resolve the order map, never treat a sorted index or table row as
spawn identity. Copy group into existing Instance.w bits without changing the 48-byte stride.

ID allocator state is separate from random frame; spawn-only paused maintenance allocates IDs
without advancing motion. Group count edits map existing membership modulo the new count on both
backends. Off preserves membership but ignores filtering/targets and renders group zero. Growth
assigns new IDs/groups; sorted-prefix truncation preserves survivors, without exact population quotas.

7d assigns dense unique slots after count changes by current record order, via an explicit maintenance
pass, and retains them through subsequent sorts. ID survives slot reassignment. CPU does equivalent
one-time assignment. Count zero clears assignments; mode respawns rebuild them. Never use ID modulo
count: truncated/grown lifetimes can have gaps and collisions. Paused count changes may assign slots
without moving existing vehicles. Unique-slot/sort/growth tests are mandatory.

Keep small group tables and feature controls in appended uniform fields (every field gets an encase
byte-offset assertion matching Naga's WGSL layout). Binding 8 is the eighth and final compute storage
binding: one read/write `array<vec4<u32>>` arena. For power-of-two capacity C (at least 512), ranges
in vec4 units are metadata `[0,C)`, maintenance scratch `[C,2C)`, packed order A `[2C,9C/4)`,
packed order B `[9C/4,5C/2)`. Each order map packs four independent scalar indices per vec4.
`Params.groups.z` is the input map base relative to `2C` (0 or C/4); `.w` is C. No group setting
or identity bits are packed into phase. Test/readback `BoidData` remains a logical 80-byte snapshot;
it is not the physical storage stride.

7c–7e append shared scene records starting at `5C/2`, with uniform offsets/counts naming disjoint
obstacle, formation-slot and predator ranges. Bitcast float payloads into the raw u32 lanes. CPU
keeps typed vectors and packs/uploads scene changes; never upload boid metadata each frame.
Capacity growth must relocate/reupload this scene tail when its base changes. Future workers define
and test exact record strides and bounds before use. Empty scene lists have zero count; the metadata
allocation already makes the binding legal. Do not add a ninth storage binding.

Current arena costs 40 bytes per capacity entry (8 bytes more than the rejected two 80-byte
state arrays added; this trades allocation size for less per-step bandwidth): +1.25 MiB at 20k, +5 MiB at 100k, +40 MiB at 1M.
State bindings remain 64 MiB maximum; metadata arena is 40 MiB at 1M capacity. One Vec4 slot per
entry would make the shared arena 56 MiB, below the default 128 MiB binding limit, with bounded
obstacle/predator records still requiring explicit validation. Instance stride remains 48 bytes.
Animation phase stays bit-identical and independent of ID/group/slot. Measured layout alternatives
and the portable eight-binding limit are recorded below.

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
- 7b: `set groups off|2..8`, `group_target group x y z` or `group_target group off`, `group_color group RRGGBB`; zero-based group indices 0–7.
- 7c: `obstacle add x y z radius`, `obstacle clear`, `set obstacle_strength k`,
  `set obstacle_clearance r`; bounded list and reference radii validated before startup.
- 7d: `set formation off|sphere|helix`, `formation_center x y z`, `set formation_extent r`,
  `set formation_strength k`, `set formation_morph t` (0..1).
- 7e: `predator add x y z radius`, `predator clear`, `set predator_strength k`,
  `set predator_path off|orbit`, `set predator_groups mask`.

Only 7a–7b commands are implemented so far. Later command ranges and record layouts are
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

### Stage 7b original benchmark evidence (80-byte candidate rejected)

Evidence root: `/tmp/claude-1000/-home-martines-Projects-autonomous-characters-3d/af1dacb4-dc47-4bb5-8e8c-02c6cd1b95e8/scratchpad/shots/stage7b/`.

Before is committed `d657d5d` with 64-byte Boids; after is this uncommitted Stage 7b implementation
with 80-byte Boids. Groups and attractor are Off in every headless case. No code or fixture tuning
was used between the comparisons. Same AMD Ryzen 9 7900X / AMD Radeon RX 7900 XT (RADV NAVI31),
Vulkan, Mesa 26.2.5-arch1.1, Linux 7.2.8-9-omarchy, pinned Rust, release profile. All commands ran
sequentially with `CARGO_BUILD_JOBS=4`, no app or competing Cargo/rustc process, uptime checked before
every run. GPU before load averages: 0.10/0.47/0.67 and 0.15/0.48/0.67; after:
1.05/0.98/0.78 and 0.89/0.95/0.77. CPU before: 0.19/0.55/0.70 twice; after:
0.95/0.92/0.74 and 1.20/0.97/0.76. Full logs and load snapshots are retained in the evidence root.

GPU command (twice before and twice after):

```sh
GPU_BENCH_COUNTS=100000,1000000 GPU_BENCH_CAPS=16 GPU_BENCH_MODES=wander,follow CARGO_BUILD_JOBS=4 cargo test --release --locked gpu_flocking_benchmark -- --ignored --nocapture --test-threads=1
```

Every case starts fresh at PCG spawn frame 0, uses simulated dt 1/60 s, 600 warmup steps, then 60
encode/submit/blocking-completion samples, with readback/statistics excluded from timing. All cases
completed every step; no 250 ms cutoff fired. Wander uses effective cap 0; follow cap 16 preserves
complete separation. Scatter order and dense follow trajectories vary; these are end-to-end step
costs, not isolated bandwidth microbenchmarks. Before/after density statistics remain in the logs.

| Case | Before run 1 / run 2 median (p95), ms | After run 1 / run 2 median (p95), ms | Change in average of medians |
| --- | --- | --- | --- |
| 100,000 wander | 0.656 (0.706) / 0.684 (0.770) | 0.816 (0.880) / 0.846 (0.937) | +24.0% |
| 100,000 follow | 3.244 (6.663) / 3.275 (4.692) | 3.249 (5.174) / 3.156 (4.789) | -1.7% |
| 1,000,000 wander | 4.576 (4.755) / 4.594 (4.672) | 6.084 (6.225) / 6.006 (6.154) | +31.8% |
| 1,000,000 follow | 7.838 (8.652) / 7.562 (8.718) | 9.031 (9.762) / 9.121 (10.075) | +17.9% |

CPU command (twice before and twice after):

```sh
CARGO_BUILD_JOBS=4 cargo test --release --locked vehicle::tests::flocking_benchmark -- --exact --ignored --nocapture --test-threads=1
```

The existing CPU fixture intentionally covers 200–10,000, below the 50k supported CPU UI ceiling;
100k and 1M are GPU cases. No million-agent CPU promise or infeasible trillion-candidate Naive probe
is introduced. Seed `0xB01D5`, dt 1/60 s, 24 compute threads, octree leaf size 100, 10 warmup steps,
60 timed steps per case. Seed fixes the initial CPU fixtures; thread-local wander RNG is not replayed.
Every case completed all 60; no CPU time cutoff fired.

| Case | Before run 1 / run 2 median (p95), ms | After run 1 / run 2 median (p95), ms |
| --- | --- | --- |
| 200 naive | 0.094 (0.104) / 0.093 (0.103) | 0.089 (0.098) / 0.083 (0.110) |
| 200 octree | 0.099 (0.111) / 0.099 (0.110) | 0.097 (0.109) / 0.071 (0.098) |
| 1,000 naive | 0.340 (0.382) / 0.352 (0.388) | 0.354 (0.390) / 0.438 (0.481) |
| 1,000 octree | 0.252 (0.270) / 0.256 (0.293) | 0.258 (0.297) / 0.235 (0.270) |
| 2,000 naive | 0.817 (0.945) / 0.850 (1.019) | 0.937 (0.991) / 0.942 (1.076) |
| 2,000 octree | 0.431 (0.493) / 0.417 (0.453) | 0.420 (0.448) / 0.418 (0.486) |
| 5,000 naive | 3.941 (4.482) / 3.977 (4.627) | 4.032 (4.487) / 3.954 (4.633) |
| 5,000 octree | 0.929 (1.116) / 0.920 (1.083) | 0.982 (1.144) / 0.984 (1.097) |
| 10,000 naive | 14.555 (15.948) / 14.871 (15.884) | 14.488 (15.766) / 15.153 (16.830) |
| 10,000 octree | 1.795 (2.113) / 1.795 (2.113) | 1.875 (2.125) / 1.822 (2.105) |

Original decision, rejected by orchestrator review: retain the explicitly required 80-byte lane. It preserves the complete animation
phase and gives groups and future formation slots explicit carried identity. This is not a free
change: average Off wander medians increase about 24% at 100k and 32% at 1M (about 0.16/1.46 ms).
Dense 1M follow rises about 18%; 100k follow variation is wider and provides no reliable speedup
claim. CPU 10k medians change about +0.7% Naive / +3.0% Octree, with run variation. The owner can
review the measured tradeoff before committing; no implicit phase packing, reduced precision or
neighbor semantic change was used to hide the cost.

Calculated buffer cost: two state arrays add 32 bytes per capacity entry, or +1 MiB at 20k,
+4 MiB at 100k and +32 MiB at 1M. The maximum individual Boid binding is 80 MiB; instance stride
stays 48 bytes. Compute Params grows 176→320 bytes (groups at byte 176, target array at byte 192),
appearance uniform 192→320 bytes (eight linear group colors at byte 192). Unit byte-offset tests
cover both; the runtime shader passed startup validation. No extra compute storage binding or draw.

### Stage 7b rendered and desktop evidence

Same evidence root as Stage 7b headless measurements. `desktop.ac3d` and `desktop.log` retain the
61-second scenario. Screenshots:

- `gpu-groups-start.png`, `gpu-groups-20s.png`: GPU 20,000, groups 0/1, cyan/orange, no group targets.
- `gpu-group-targets.png`: targets at (-350,0,0) and (350,0,0).
- `gpu-global-follow-override.png`: green global target at origin; both group targets remain enabled.
- `gpu-attractor-both-before.png`, `gpu-attractor-both-after.png`: group targets cleared, attractor
  at (250,0,0), strength 8, reference radius 120, affecting both colored populations.
- `octree-groups.png`: Octree 1,500, targets at (-130,0,0) and (130,0,0).

Rendered timings use four fresh apps with `timing-{off,on}-{1,2}.ac3d` and matching logs, each ending
with `exit`. Groups Off versus two groups, no group/global targets, attractor Off, Group color mode
in both cases, Dart, neutral scales, Studio/Aurora, 100,000 GPU, 4× MSAA, 1776×1119, immediate
presentation, camera yaw 0° / pitch 10° / distance 2,500, simulation 1×. PCG starts at spawn frame 0;
real-time dt/scatter/evolved states vary. Three-second warmup, four-second measurement; no captures
or timestamp instrumentation during measurements. Same hardware/driver as headless tests. Loads
are in `timing-load.txt`; no competing build/test/app ran. Whole-frame intervals, not GPU durations.

| Groups | Run 1 mean / p50 / p95 ms (frames) | Run 2 mean / p50 / p95 ms (frames) |
| --- | --- | --- |
| Off | 1.116 / 1.086 / 1.357 (3,585) | 1.097 / 1.077 / 1.299 (3,647) |
| Two | 1.062 / 1.001 / 1.369 (3,766) | 1.050 / 1.021 / 1.298 (3,808) |

Both sit near this scene's rendering/presentation floor. On changes the eligible neighbor set and
motion; the small interval reduction is not a general speedup claim and does not erase the
headless Off metadata cost. Final `tests.log`, `build.log`, `clippy.log`, `gpu.log`, `smoke.log`,
`desktop.log` and `verification.txt` retain validation evidence.

### Stage 7b Off-path performance review and final layout

The orchestrator rejected the original 80-byte candidate. Final decision: restore the physical
64-byte Boid, retain the complete four-lane metadata in a persistent arena table, and scatter only
32-bit table indices in parallel order maps. Specialize the steering pipeline with a boolean WGSL
override: Off removes metadata reads/filtering/group-target selection at compilation; On retains
all approved group semantics. This preserves identity while Off without copying 16 metadata bytes
on every sort or copying them back from separation/steer. Count edits pay two maintenance dispatches;
normal frames do not. The detailed arena/identity contract above supersedes the original 80-byte
proposal. CPU steering and rendering semantics are unchanged by this correction.

Rejected options remain measured below. Field-only neighbor loads did not recover the 80-byte
scatter cost. Splitting metadata but copying it through every pass retained material overhead.
Full-metadata ping-pong improved this, but carrying four-byte indices avoids twelve extra bytes
per sorted member and leaves the Off neighbor query state-only. Skipping identity transport while
Off was rejected without implementation: it would scramble preserved membership and future slots.
Two separate metadata bindings were rejected on portability grounds; the shared arena uses one.

Bevy limits checked directly in pinned local primary sources:
`~/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/bevy_render-0.20.1/src/settings.rs`
lines 89–104 and 308 (`WgpuSettings::default`, automatic render creation), and `src/renderer/mod.rs`
lines 319–355. Default priority is Functionality, which replaces initial default limits with
`adapter.limits()` before requesting the device. Constraints can lower limits; WebGPU priority
keeps default limits. This RX 7900 XT device requests 1,864,135 storage buffers per shader stage
and a 2,147,483,644-byte storage binding limit. More than eight is therefore available here, but
not guaranteed across supported settings/devices. Final layout requires exactly eight storage
bindings; future scene records share binding 8. Seven final GPU regressions also pass with
`WGPU_SETTINGS_PRIO=webgpu` (eight storage buffers, default 128 MiB storage binding limit).

Evidence: `stage7b/perf-fix/` under the evidence root above. All candidate logs, uptime snapshots,
source probes and gate logs are retained there. Same device, driver, fixed-step fixture, warmup,
counts, caps, sample count and sequential process rules as the original comparison. Every case
completed all 600 warmup and 60 timed steps. Numbers below are per-run median milliseconds, with
all valid runs retained. Four final Off runs were collected because the 100k cases varied; no run
was dropped. Final Off launch load averages were 0.11/0.12/0.16, 0.41/0.19/0.18,
0.07/0.35/0.27 and 0.39/0.41/0.29. Final On launch loads were 0.05/0.13/0.19 and
0.59/0.26/0.23. No competing Cargo, rustc or native app ran.

| Layout / candidate | 100k wander | 100k follow | 1M wander | 1M follow |
| --- | --- | --- | --- | --- |
| Original 64 B | 0.656 / 0.684 | 3.244 / 3.275 | 4.576 / 4.594 | 7.838 / 7.562 |
| First 80 B | 0.816 / 0.846 | 3.249 / 3.156 | 6.084 / 6.006 | 9.031 / 9.121 |
| 80 B explicit neighbor fields | 0.784 / 0.802 | 3.717 / 3.738 | 5.613 / 5.588 | 9.006 / 9.256 |
| 64 B full metadata scatter/copyback | 0.725 / 0.724 | 3.472 / 3.217 | 5.265 / 5.176 | 8.401 / 8.211 |
| 64 B late metadata loads | 0.714 / 0.684 | 3.716 / 3.809 | 4.897 / 4.943 | 8.513 / 8.729 |
| 64 B specialized Off/On | 0.715 / 0.742 | 3.456 / 3.493 | 4.793 / 4.746 | 8.363 / 8.244 |
| 64 B metadata ping-pong, neighbor fields | 0.740 / 0.642 | 3.310 / 3.230 | 4.717 / 4.688 | 8.070 / 8.099 |
| 64 B metadata ping-pong, whole neighbor | 0.734 / 0.681 | 2.929 / 2.977 | 4.712 / 4.683 | 8.084 / 7.912 |
| Final 64 B + packed order maps, Off | 0.736 / 0.627 / 0.739 / 0.711 | 3.247 / 3.672 / 3.145 / 3.202 | 4.621 / 4.599 / 4.637 / 4.658 | 7.813 / 7.765 / 7.985 / 7.832 |
| Fresh archived 64 B control | 0.692 / 0.701 | 3.177 / 3.137 | 4.602 / 4.607 | 7.442 / 7.535 |
| Final packed order maps, two groups | 0.713 / 0.630 | 5.625 / 5.530 | 5.511 / 5.502 | 11.164 / 11.227 |

Averages of all valid per-run medians for final Off versus the original 64-byte baseline:
100k wander 0.70325 vs 0.670 ms (+4.96%); 100k follow 3.3165 vs 3.2595 ms (+1.75%);
1M wander 4.62875 vs 4.585 ms (+0.95%); 1M follow 7.84875 vs 7.700 ms (+1.93%).
These meet the requested roughly 3–5% threshold on this machine. This is not a cross-device guarantee.
Two-group averages are 0.6715 / 5.5775 / 5.5065 / 11.1955 ms in the same case order.
Group filtering changes trajectories; foreign neighbors never consume the same-group cap and can
increase scans in dense following. These On measurements do not isolate metadata arithmetic cost.

An additional contemporary baseline check archived exact committed `d657d5d` into
`/home/martines/.cache/ac3d-stage7b-baseline64-src`, with its own target directory outside this
checkout. Valid `baseline-rebuilt-{1,2}` logs show the old test inventory (103 filtered tests),
confirming the baseline source rebuilt. Fresh baseline run averages are 0.6965 / 3.157 / 4.6045 /
7.4885 ms: variation particularly affects dense follow. Against this contemporary control, final
Off changes are +1.0% / +5.1% / +0.5% / +4.8%; 100k follow sits at the edge of the requested
noise budget, rather than supporting a claim of zero overhead. Initial `baseline-recheck-{1,2}` probes
incorrectly reused copied application fingerprints and executed the current test binary. They are
explicitly invalid controls, retained for audit and excluded from every baseline calculation.
Rebuilding touched archived source corrected this; the shared real target was never used by probes.

Final rendered recheck uses the same four eight-second harness scenarios and scene settings as
original 7b, on the final layout. All four apps reached their scripted `exit`; subsequent process
checks found no app. No screenshots/timestamps ran during measurement. Full logs and uptime
snapshots are `perf-fix/rendered-{off,on}-{1,2}.{log,load}`. Whole-frame intervals remain near the
rendering/presentation floor; groups On changes trajectories and is not an isolated arithmetic test.

| Groups | Run 1 mean / p50 / p95 ms (frames) | Run 2 mean / p50 / p95 ms (frames) |
| --- | --- | --- |
| Off | 1.002 / 0.964 / 1.256 (3990) | 1.035 / 0.983 / 1.319 (3867) |
| Two | 1.012 / 0.987 / 1.218 (3954) | 1.034 / 1.002 / 1.256 (3869) |

Final correction checks: fmt check; 102 release tests passed, nine ignored; release build without
warnings; ten-second startup smoke passed; Clippy seven existing warnings, duplicated for the test
target; diff check passed. Seven ignored GPU regressions passed under WebGPU device limits,
including baseline CPU parity, attractor parity, mixed-group exact/cap filtering and metadata/phase
survival. The metadata test now also sorts while groups are Off, re-enables while paused and
shrinks/grows from the nonidentity order map. This exercises the corrected identity transport
without creating an extra GPU test. Previous approved 7b screenshots remain in the parent directory;
this correction changes storage/dispatch plumbing, not interaction or appearance semantics.

### Stage 7c evidence and lifecycle contract

Scope is static spheres only. `SwarmScene` stores obstacle records plus strength/default-clearance/
visibility settings; these controls live with their scene rather than adding writes to `GlobalState`.
CPU uses the shared scene snapshot in each `CpuStep`; GPU extraction clones changed resources.
Render-world scene uniforms use that same snapshot, including edits made in egui PostUpdate.
The UI edits a local clone and assigns the resource only if values differ. Scenario setters also
avoid unchanged writes. No per-frame record upload or GlobalState input writer was added.

Compute Params grows 320→352 bytes: `obstacles` at byte 320 (absolute vec4 offset, count, reserved,
reserved), `obstacle_settings` at byte 336 (strength, one-second look-ahead, world scale, reserved).
The encase byte-offset test extends the Naga layout assertions. Shader Boid stays 64 bytes;
metadata layout/order maps, animation phase, 48-byte instances, vehicle draw and eight storage
bindings remain unchanged. Four `steer` pipeline specializations cover group/obstacle enable bits.
Empty lists and strength zero select the obstacle-disabled pipeline and perform no obstacle arithmetic
or arena reads in steering. CPU branches before adding any obstacle term, preserving disabled bits.

For capacity C, sphere storage reserves `[5C/2,5C/2+16)` in binding-8 vec4 units (256 bytes).
Each two-vec4 record is `(world center.xyz, reference radius)` and `(reference clearance,0,0,0)`,
written as float bits into the raw-u32 arena. Only the occupied prefix is uploaded. Clearing sets
count zero without erasing unreachable stale records. Later scene arrays begin after this reserved
16-record region; 7d/7e must append their ranges and extend uniform offset/bounds tests. New capacity
relocates/reuploads the sphere tail after copying retained state, metadata, order map and instances.
Count edits within capacity need no record upload; radius/clearance scaling changes through uniforms.
Paused edits upload once without stepping vehicles. Backend/count changes preserve the CPU scene.

New tests: three CPU law tests (falloff/inside/center, segment/endpoints/sum cap, world scaling/Off),
one exact inactive-force/additivity test, one pause/speed/seek test in both CPU modes, one render-scene
uniform snapshot/reset test, and scenario
validation including max-eight, clear/reset, finite/range rejection and unchanged setter writes.
The existing metadata lifecycle fixture now also checks scene survival across CPU modes, growth,
truncation and backend respawn. Two ignored device tests cover mixed groups/attractor/obstacle parity
and exact arena records/upload counts across paused growth, shrink/regrowth, zero count, edits,
settings-only changes, and one/two-sort paths. Existing seven GPU regressions remain unchanged in
behavior and are part of the final nine-test device gate.

Headless comparisons use exact committed `fd9921a` before edits, then this final 7c code, on the same
Ryzen 9 7900X / Radeon RX 7900 XT (RADV NAVI31), Vulkan/Mesa 26.2.5-arch1.1, Linux 7.2.8-9-omarchy,
pinned Rust and release profile. PCG spawn frame zero, dt=1/60 s, 600 warmup steps then 60 timed
encode/submit/wait steps. Counts 100k/1M, configured cap 16, effective wander cap zero and follow
cap 16, groups/attractor Off. Two complete runs per configuration, never concurrent with any build,
test or native app; pgrep and uptime precede every launch. All cases completed every step.

The eight-sphere benchmark uses `GPU_BENCH_OBSTACLES=8`: all combinations of world-scaled reference
centers `(±20,±20,±10)`, reference radius 8, clearance 10, strength 4, look-ahead one simulated second.
It changes trajectories/density as well as arithmetic, so On-minus-Off is not a pure kernel cost.
Readback/statistics and initial one-time record upload are excluded by the warmup. Original and
final timings, p95s, density statistics and uptime snapshots live under
`/tmp/claude-1000/-home-martines-Projects-autonomous-characters-3d/af1dacb4-dc47-4bb5-8e8c-02c6cd1b95e8/scratchpad/shots/stage7c/`.

| Case | 7b run 1 / 2 median (p95) ms | 7c Off run 1 / 2 median (p95) ms | Eight spheres run 1 / 2 median (p95) ms | Off mean-median change |
| --- | --- | --- | --- | --- |
| 100k wander | 0.715 (0.764) / 0.704 (0.755) | 0.650 (0.698) / 0.663 (0.704) | 0.778 (0.857) / 0.719 (0.838) | -7.47% |
| 100k follow | 3.590 (4.836) / 3.231 (4.915) | 3.624 (4.506) / 2.909 (4.108) | 3.543 (4.593) / 3.920 (5.340) | -4.22% |
| 1M wander | 4.682 (4.776) / 4.638 (4.776) | 4.662 (4.796) / 4.651 (4.761) | 5.299 (5.389) / 5.303 (5.425) | -0.08% |
| 1M follow | 7.960 (8.818) / 7.844 (8.788) | 7.906 (8.756) / 7.814 (8.617) | 9.355 (10.929) / 9.643 (11.072) | -0.53% |

No measurable Off regression: large-count wander differs by less than 0.1%, and 1M follow by about
-0.5%. Short 100k runs and dense-follow trajectories vary; the negative changes are not claimed as
speedups. Specialized Off retains the original force expression and neighbor work. No approximate
avoidance/neighbors, density-policy change or wall retuning was introduced.

Launch uptime snapshots (full values retained in matching `.load` files):

- `before-1`: 22:54:08 up  4:34,  1 user,  load average: 0.69, 0.76, 0.50
- `before-2`: 22:54:36 up  4:35,  1 user,  load average: 0.67, 0.74, 0.50
- `after-off-1`: 23:00:59 up  4:41,  1 user,  load average: 0.49, 0.42, 0.41
- `after-off-2`: 23:01:24 up  4:42,  1 user,  load average: 0.32, 0.38, 0.40
- `after-on-1`: 23:02:00 up  4:42,  1 user,  load average: 0.18, 0.34, 0.38
- `after-on-2`: 23:02:53 up  4:43,  1 user,  load average: 0.14, 0.30, 0.36

Rendered 7c measurements use four fresh final-build apps at physical 1280×720, floating test
window, immediate presentation, 100k GPU Dart, Studio/Aurora, heading color, neutral scales,
4× MSAA, 1× simulation, camera yaw 0°/pitch 10°/distance 2,500, no global/group targets or attractor.
On uses the same eight world-scaled sphere centers/radii/settings as the headless fixture, with
wireframes visible. Each script warms up three seconds, samples four seconds, then reaches `exit`
at eight seconds. These are whole-frame intervals, not GPU timestamps; no screenshots during timing.
All scripts/logs/window-sizing transcripts and uptime snapshots are `timing-{off,on}-{1,2}.*` in
the evidence root. Earlier measurements before the render-snapshot correction are retained under
`pre-snapshot-fix/`, excluded from the final rendered table. No test window setting persists after exit.

| Spheres | Run 1 mean / p50 / p95 ms (frames) | Run 2 mean / p50 / p95 ms (frames) |
| --- | --- | --- |
| Off | 1.347 / 1.347 / 1.909 (2,970) | 1.281 / 1.294 / 1.783 (3,123) |
| Eight | 1.354 / 1.361 / 1.917 (2,954) | 1.311 / 1.308 / 1.908 (3,050) |

Desktop evidence is `desktop.ac3d` / `desktop.log` (32 s), plus `octree-check.ac3d` / log (4 s)
for a closer view before the Octree flock reaches the target. Every harness app reaches `exit`.
Temporary floating/resize operations affect only the newly launched test window; no desktop
configuration is changed. Original tiled captures cropped the target and are retained in
`tiled-attempt/`; final captures are 1280×720:

- `gpu-follow-before.png`, `gpu-follow-after.png`: 20k, follow at (500,0,0), spheres at
  (120,0,0), (280,85,0), (340,-65,-20), reference radii 12/11/9, strength 6, clearance 10,
  before resume and after about 4.5 simulated seconds at 1× with vsync (~60 Hz).
- `gpu-wander.png`: following cleared with the same obstacles.
- `gpu-groups-obstacles.png`: groups 0/1 seeking (-400,140,0)/(400,100,0), same spheres.
- `gpu-paused-a.png`, `gpu-paused-b.png`: a fourth sphere added while paused, no simulation motion.
  ImageMagick absolute differing-pixel count is zero in `450x350+420+100`, excluding UI/FPS.
- `octree-obstacles.png`: 1,500 CPU Octree vehicles, two groups, selected target (160,0,0),
  spheres at (-40,0,0)/(80,20,0), radius 10, strength 6; captured about 2.5 s after resume.

No non-penetration promise: avoidance is a bounded additive force, not collision resolution.
Look-ahead is fixed at one simulated second, and spheres are static in 7c. Dynamic paths, formations
and predators remain 7d/7e work. Other GPUs, dense multi-feature combinations and worst-case CPU
counts are unmeasured. No group rebalancing or neighbor/separation policy changed.

## Interfaces and Dependencies

7a: new `src/swarm.rs` owns AttractorMode/settings, force kernel, cursor input, gizmo and UI.
`GlobalState` gains the off-default attractor settings. CPU flocking adds a conditional term; GPU
Params appends world point/radius and signed strength Vec4s, mirrored by WGSL. Scenario adds typed
commands. No dependency, Boid stride, Instance stride or baseline simulation-setting changes in 7a.

7b: `Groups`/`Group` fixed settings table, `VehicleMetadata(UVec4)`, monotonic CPU/GPU spawn counters,
64-byte physical Boids, logical 80-byte test/readback `BoidData`, separate metadata/order arena,
GPU group/count maintenance passes and order tracking. CPU production
kernel is `group_flocking_force`; existing single-group fixtures use the `flocking_force` adapter.
`ColorMode::Group` is appended at value 5, preserving previous mode values. Typed group scenario
settings reuse startup validation. Compute and appearance uniform offsets are extended as recorded
above; binding 8 holds metadata/order maps and reserves its tail for future scene records.
No instance attribute is added.

7c: `SwarmScene` / `Obstacle` / `obstacle_force` / `obstacles_ui` in `swarm.rs`; a changed-resource
extract plus render-snapshot `GpuParams::with_scene`; two appended uniform fields, four steering
specializations and a 256-byte sphere tail in binding 8. CPU `group_flocking_force` takes the same
scene snapshot. Scenario adds validated obstacle add/clear and strength/clearance/visibility setters.
No new crate, storage binding, vehicle instance attribute or physics/collider integration.

---

Revision notes:

- 2026-10-31: Initial Stage 7 contract and Stage 7a implementation/validation scope, written before code.

- 2026-10-31: Stage 7a complete; recorded release/device checks, desktop pixel proof and repeated rendered frame timings. Later sub-stages remain open.

- 2026-10-31: Stage 7b authorized; finalized cap, fixed table, dormant membership and paused maintenance policies before implementation.

- 2026-10-31: Stage 7b CPU suite and repeated before/after headless measurements complete; recorded the measured 80-byte lane tradeoff before desktop/device acceptance.

- 2026-10-31: Stage 7b complete and uncommitted; recorded final CPU/device gates, repeated rendered measurements, screenshots and remaining density/population limits.

- 2026-10-31: Orchestrator rejected the Off-path regression; reopened layout/performance acceptance without changing group semantics.

- 2026-10-31: Resolved the orchestrator performance blocker with 64-byte state, persistent metadata and packed sort indices; retained all rejected candidate measurements and portable binding-budget proof.

- 2026-10-31: Stage 7b committed by orchestrator as fd9921a; authorized 7c only and finalized sphere law, arena stride and upload lifecycle before code.

- 2026-10-31: Stage 7c implemented with a shared sphere law, 256-byte arena tail, coherent render-scene snapshot, change-only uploads, UI/scenarios and repeated headless/rendered/desktop evidence; 109 release tests and the single final nine-test GPU gate pass.
