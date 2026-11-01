# Add optional swarm mechanics with one CPU/GPU contract

This ExecPlan is a living document maintained according to `PLANS.md`. It designs all of Stage 7;
Stage 7a is committed as `d657d5d`; 7b is committed as `fd9921a`; 7c is committed as `4e0d5a6`; 7d is committed as `77406a4`; the current worker implements **7e only**. The orchestrator owns review, commits and authorization of
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
- [x] (2026-10-31 23:09Z) Stage 7d: analytic targets, dense slots, morph clock, UI/scenarios.
- [x] (2026-10-31 23:09Z) Stage 7d: 116 release tests and all 11 ignored GPU tests pass in the single final device run.
- [x] (2026-10-31 23:09Z) Stage 7d: repeated Off/On/cap/60-second convergence measurements, desktop proof and gates pass; awaiting owner review/commit.
- [x] (2026-10-31 23:49Z) Stage 7e: shared flee law, path snapshots, arena upload lifecycle, UI/gizmos/scenarios.
- [x] (2026-10-31 23:49Z) Stage 7e: 122 release tests and the single final 13-test GPU gate pass; parser/path/mask/force/arena regressions included.
- [x] (2026-10-31 23:58Z) Stage 7e: formatting, 122 release tests, warning-free build, Clippy 7, native smoke, desktop/pause proof and diff check. Repeated diagnostics recorded.
- [ ] Stage 7e: required idle-machine Off/On/combined/rendered rerun; unrelated Blender remained active (224% CPU sampled at 23:58Z). Provisional diagnostics cannot close this acceptance condition.
- [ ] Stage 7 complete (all five optional mechanics verified; owner review/commit separate).
- [ ] Orchestrator review and commits.

## Surprises & Discoveries

- Stage 7d: analytic target tests cover one million slots without longitude banding or duplicate y.
  After 11 simulated seconds the 1M flock is still far from assigned slots (median roughly 930–1013
  world units with cap 16). A 60-second check is necessary to distinguish assembly time from failure.
  First long run reaches median 69.4 (wander-wall branch) / 51.7 (stored-follow branch), with a long
  unsettled tail. Flocking and exact separation remain active, so zero slot error is not promised.
- Stage 7d: slot assignment uses the existing counts/starts scratch, plus only 128 persistent arena
  bytes for group ranges. It adds no per-slot Vec4 allocation or edit upload at one million.


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

- Decision (7d, before code): analytic Fibonacci sphere and three-turn helix; no slot-target array.
  For local slot i of n, u=(i+0.5)/n, y=1-2u, angle=2π*f32((i*1640531527 wrapping)>>8)/2^24.
  Sphere=(sqrt(max(0,1-y²))*cos(angle), y, sqrt(max(0,1-y²))*sin(angle)).
  Helix=(0.6*cos(6πu), 0.8*y, 0.6*sin(6πu)). Blend linearly by actual morph fraction.
  The integer angle phase avoids large-float longitude quantization at one million; it never uses
  animation phase. Both shapes and every blend remain within unit extent; monotonic y preserves
  unique slots. Multiply by extent*S and add center plus a group grid offset. Grid columns are
  ceil(sqrt(group count)), rows ceil(count/columns), spacing 2.25*extent*S in the XY plane.
  Default extent 60, arrival radius 10, strength 4, center origin. All explicit centers are world units.
  Rationale: bounded O(1) target evaluation, no 16 MiB edits or new binding, deterministic shared law.
  Date: 2026-10-31.
- Decision (7d): Sphere/Helix select morph targets 0/1. The morph slider/command sets a target fraction;
  actual fraction approaches it at morph speed (default 0.25 fraction/s) in simulated substeps.
  Zero morph speed snaps to the requested fraction on the next simulated step. First enable snaps
  the initial shape to the selected target; switching active presets morphs without slot reassignment.
  A changed-only `FormationFrame` snapshots settings and four substep fractions after the simulation
  clock is prepared. CPU steps/GPU encode consume the same samples; pause freezes actual morph.
  Scene settings change only on user/scenario edits, never because time advances. Navigation force
  adds after obstacles and before flock mass division. Formation suppresses global/group seek and
  wander, but retains wall steering only when the original navigation branch would wander.
  Follow selection and group targets stay stored. Strength zero retains the formation navigation
  selection with zero arrival force; Off explicitly restores normal navigation.
  Rationale: consistent substeps, no per-frame scene uploads and no retained live target array.
  Date: 2026-10-31.
- Decision (7d): assign global dense metadata.z slots by group prefix plus current-order rank within
  that group. Trigger on count edits, group-count edits and Off→On; preserve slots during active
  shape edits/sorts/CPU mode switches. CPU uses two linear passes. GPU uses fixed 256-record blocks:
  count/local-rank, eight-group block-prefix scan, then add block/group bases. Existing counts/starts
  buffers serve as maintenance scratch, overwritten by the next normal grid build. Binding-8 tail
  reserves eight vec4 group ranges immediately after 16 sphere records: `[5C/2+16,5C/2+24)`.
  Range records are `(global start, member count, 0, 0)`; future predator data follows +24.
  Recompute after capacity growth/count changes; no header copy or host readback is needed.
  Rationale: O(N), exact deterministic assignment for identical records, no matching or ninth binding.
  Date: 2026-10-31.
- Decision (7d): enabled formations activate the configured GPU alignment/cohesion cap regardless of
  following; zero remains exact. Separation stays exact and foreign groups never consume samples.
  Eight steer variants specialize group/obstacle/formation enable bits. Benchmark sphere cap 0/16
  at 100k/1M and record convergence/slow-step cutoffs before accepting dense-mode practicality.
  Rationale: formations deliberately gather dense flocks; preserve the existing explicit cap control
  and expose the exact path rather than add a hidden density approximation.
  Date: 2026-10-31.

- Decision (7e, before code): `SwarmScene::predators` owns up to four static positions, reference
  threat radii and eight-bit prey masks, plus strength (default 4), visibility, Off/Orbit path,
  reference path radius (50), angular speed (30 degrees/s), default mask (255) and anchored center.
  Empty list is Off; zero strength also compiles force out. Logical group zero is the entire flock
  when groups are Off; otherwise use the carried mapped group. Each relevant predator contributes
  outward normal (+X at coincidence) times smoothstep(1-clamp(distance/(radius*S),0,1)). Clamp the
  summed normal weights to length one, multiply by V*k, and add after formation before mass division.
  All four fields are evaluated outside neighbor sampling; predators do not themselves enable a cap.
  Rationale: direct bounded shared law, no capture/history or per-boid role data.
  Date: 2026-10-31.
- Decision (7e): Orbit is a deterministic XY circle, with evenly spaced predator phases. It uses
  formation center while formation is enabled, otherwise an anchored orbit center (origin by
  default; UI captures orbit focus when selecting Orbit and offers an explicit focus button).
  Static mode uses each edited world position; Orbit overrides these positions. Threat radius/path
  dimensions scale by world_scale, but explicit static/anchor coordinates do not. Keep phase bounded
  in f64, advancing only during simulated substeps; an empty-list snapshot resets phase, backend/count edits preserve
  settings. Paused camera movement cannot move the anchor. Explicit center/count edits may relocate
  or rescale visible objects while time is frozen, matching other scene editing semantics.
  Rationale: deterministic, pause-safe paths without per-frame camera-driven resource writes.
  Date: 2026-10-31.
- Decision (7e): extracted `PredatorFrame` holds four per-substep snapshots, sampled once in the main
  world after the simulation clock and formation snapshot. CPU/GPU use exactly those positions,
  radii and masks. Binding 8 reserves 32 vec4 records at `[5C/2+24,5C/2+56)`: four substeps × four
  objects × two vec4s. Records are `(position.xyz float bits, world radius bits)` and `(mask,0,0,0)`.
  One small upload per changed frame carries all samples; per-step uniforms select the proper eight-
  vec4 slice. Static/paused frames skip unchanged uploads; capacity growth rewrites the relocated
  tail. Disabled force uploads no records. Add offset/count/substep and strength uniforms. Preserve
  64-byte Boids, 48-byte instances and eight storage bindings. Sixteen prebuilt steer specializations
  cover the four enable bits; no per-frame pipeline creation or disabled-path field work.
  Rationale: exact substeps with at most 512 bytes per moving frame, rather than per-boid uploads.
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

Design completed before implementation. Stages 7a–7d are committed; 7e is implemented and behavior-verified. All five mechanics exist. Overall Stage 7 completion remains open until the required idle-machine performance acceptance and owner review.
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

### Stage 7d handoff

Stage 7c was reviewed/committed by the orchestrator as `4e0d5a6`. Stage 7d now delivers optional
Sphere/Helix formations, analytic targets, dense carried slots with per-group ranges/centers,
simulated-substep morphing, navigation override/restoration, explicit cap policy, Swarm controls
and validated scenario commands. The GPU Boid remains 64 bytes, instance 48 bytes, bindings eight;
the only persistent formation storage addition is the 128-byte group-range header. No dense target
array, dependency, predator code or baseline steering/default retuning was introduced.

Final gate: formatting check; 116 release tests pass (13 ignored: 11 GPU and two benchmarks);
warning-free release build; Clippy seven existing warnings, repeated for the test target; ten-second
native smoke; diff check. The single final ignored GPU suite passed all 11 tests, including mixed
formation/group/obstacle/attractor parity and slot/range survival through sort/growth/truncation/
remapping/Off→On. New formation parity worst position error is 1.53e-05 world units and velocity
error 8.54e-06. CPU lifecycle coverage also checks backend respawn and CPU mode preservation.
Formula tests cover one million slots, all three tested blend fractions and world scaling.

Off headless changes are +3.8% / +1.8% / +0.3% / +1.7% for 100k wander/follow and 1M wander/follow,
within observed run variation; all four final runs remain recorded. Cap/60-second convergence,
whole-frame intervals and nine captures are below. Desktop and all four rendered apps reach `exit`.
Paused mid-morph scene pixels remain identical across two seconds. No Cargo/rustc/app remains.
The worker did not commit, push, merge, reset or discard repository changes; HEAD remains `4e0d5a6`.

Nonzero slot error is an explicit residual: arrival competes with retained flocking, exact separation,
wall drag and soft obstacle avoidance. A blocked slot or goal outside the walls may never converge.
The 1M 60-second p95 tail remains material despite good median convergence. Other GPUs, dense helix
cost at 1M and worst-case multi-group combinations remain unmeasured. Stage 7e must append predator
records after the group-range header and preserve the same change-only/simulated-time contract.

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

The fixed group table lives in `GlobalState::groups`. CPU `SwarmScene` owns obstacle, formation
and predator controls; simulated animation never writes these settings or GlobalState. Changed-only
`FormationFrame` and `PredatorFrame` resources hold up to four substep samples after the shared
simulation clock is prepared. CPU and GPU consume the same snapshots, including every predator
position/radius/mask. The render world extracts these frames plus scene settings; a moving path
uploads one bounded sample batch per frame, never one upload per substep or per boid. Formation
target evaluation uses the same substep morph time. Disabled shader specializations omit force work.

| Mechanic | CPU data | GPU data |
| --- | --- | --- |
| Attractor | `GlobalState::attractor`, no per-vehicle state | Two appended Vec4 uniforms; no storage or metadata |
| Groups | `GlobalState::groups` fixed group table; carried ID/group/slot component | Uniform group table and enable/count; separate metadata table resolved through sorted order maps; Instance.w group bits |
| Obstacles | `SwarmScene` bounded typed sphere vector and path state | Uniform count/arena offset/strength/clearance; sphere records in shared arena |
| Formations | `SwarmScene::formation` controls, changed-only substep `FormationFrame`, carried dense slot | Analytic sphere/helix uniforms; eight group ranges after sphere reservation; metadata.z slot |
| Predators | `SwarmScene::predators` controls; changed-only four-substep `PredatorFrame` | Uniform count/offset/substep/strength; four sample sets in arena at `5C/2+24`; no prey-history buffer |

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
Capacity growth relocates/reuploads this scene tail when its base changes. Spheres reserve
`[5C/2,5C/2+16)`, formation ranges `[5C/2+16,5C/2+24)`, and predator samples
`[5C/2+24,5C/2+56)` vec4s. Total arena allocation is `40C+896` bytes. Empty scene lists have zero count; the metadata
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
| Predators | Additive flee; follow/group goals stay stored, formation retains navigation override | Shared per-substep paths freeze and follow speed; edits/gizmos live; camera movement cannot move anchor | Mask selects mapped groups; Off uses logical group 0; caps never hide threats | Preserve scene/path phase across CPU/backend/count edits; backend respawns flock only; rescale reference radius/path |

| Pair | Composition |
| --- | --- |
| Attractor × groups | Same field affects all groups; no implicit selection mask in 7a. |
| Attractor × obstacles | Add both bounded forces; obstacle steering can bend attraction, no hard collision guarantee. |
| Attractor × formations | Field temporarily deforms assembly; arrival returns members to slots on release. |
| Attractor × predators | Add field and flee; neither silently changes the other's strength. |
| Groups × obstacles | All groups avoid the same proxies; cross-group separation remains complete. |
| Groups × formations | Disjoint carried slot ranges, separate centers and one shared range header; analytic targets. |
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
- 7d: `set formation off|sphere|helix`, `formation_center x y z`, `set formation_extent r` (1..200),
  `set formation_arrival_radius r` (0.1..200), `set formation_strength k` (0..20),
  `set formation_morph t` (0..1), `set formation_morph_speed r` (0..2 fractions/s).
- 7e: `predator add x y z radius`, `predator clear`, `set predator_strength k`,
  `set predator_path off|orbit`, `set predator_groups mask`; plus `predator_center x y z`,
  `set predator_path_radius r`, `set predator_path_speed degrees_per_second`,
  `set predators_visible on|off`. Masks update every existing control and future additions.

Stages 7a–7d commands are implemented. Later command ranges and record layouts are
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

### Stage 7d implementation and headless evidence

`formation.rs` owns Copy settings, bounded analytic formulas, arrival law, and extracted
`FormationFrame`. `SwarmScene::formation` stores edited settings only. `prepare_frame` snapshots
settings after the simulation clock and calculates four morph samples; CPU/GPU use matching samples
at each simulated substep. Only changed values mark resources. The UI uses an aggregate SystemParam
for scene, actual morph and orbit focus; controls stay in Swarm and explain stored following.
No per-frame GlobalState or scene write was added.

CPU metadata assignment runs after spawn/cleanup/membership and before stepping, using current query
record order. GPU assignment runs before sorting, after compaction/spawn/remap. A 64-thread dispatch
handles fixed 256-record blocks serially per invocation, writing local group ranks into metadata.z
and group histograms into existing counts. One eight-thread workgroup scans block totals and writes
range headers; the final 256-thread pass adds group/block prefixes to each slot. All passes are O(N)
with fixed eight-group capacity, and run only on active topology changes or enabling. Normal sorts
carry slots through the existing order maps. Active Sphere/Helix changes preserve slots. Off keeps
latent metadata untouched; enabling establishes valid dense slots, including vehicles spawned Off.
CPU↔GPU respawns, CPU modes retain slots. Existing group membership/rebalancing rules remain intact.

Compute Params is 400 bytes (previously 352): formation mode/reserved/block-count/range-offset at
byte 352, center/extent at 368, arrival/strength/actual-morph/reserved at 384. Tests extend encase/Naga
offsets and buffer bounds. Boids remain 64 bytes and instances 48. Eight specialization variants
remove all inactive formation reads/math. Binding 8 grows only 128 bytes: ranges occupy
`[5C/2+16,5C/2+24)` vec4s, after the 256-byte sphere reservation. Each is u32 `(start,count,0,0)`.
Sphere uploads cannot overwrite these ranges. Growth reassigns ranges from retained/spawned records
rather than copying old headers. Predator records in 7e must begin at `5C/2+24` or later. No dense
slot-target allocation (16 MiB at 1M capacity), host readback, matching or new storage binding.

GPU benchmark comparison: baseline is exact committed `4e0d5a6` before edits. Same Ryzen 9 7900X /
Radeon RX 7900 XT (RADV NAVI31), Vulkan/Mesa 26.2.5-arch1.1, Linux 7.2.8-9-omarchy, pinned Rust,
release profile, PCG frame-zero spawn, dt=1/60 s, 600 warmup plus 60 timed encode/submit/wait samples.
Groups, attractor and obstacles are Off. Formation uses default origin/extent 60/arrival 10/cap-strength
4 and fixed sphere blend zero. Original configured cap is 16; Off wander stays exact. Active sphere
uses configured cap regardless of follow selection; cap zero is the complete query. Separation is
always exact. Stored follow suppresses the wander-wall branch while formation still overrides seek.

Two baseline runs, four final Off runs (two extra repeats for 100k variance), and two On runs per cap;
all valid runs retained. GPU shader/fixture is unchanged when adding optional `GPU_BENCH_WARMUP` for
long convergence checks. No Cargo/rustc/app overlapped. Uptime preceded each command; snapshots and
logs are in the evidence root below. First Off launch follows this worker's compilation (load 5.44,
2.89,2.23); no competing process was running. Process inventory showed idle desktop processes only.
Later Off loads are 2.84/2.59/2.16, 1.17/1.43/1.71, 0.96/1.29/1.63. All requested steps completed;
no 250 ms slow-step cutoff fired. Readback, density and slot statistics stay outside timed samples.

Averages of per-run median milliseconds:

| Case | 7c Off | 7d Off | Sphere cap 16 | Sphere cap 0 | Off change |
| --- | --- | --- | --- | --- | --- |
| 100k wander | 0.673 | 0.699 | 0.744 | 2.407 | +3.83% |
| 100k stored follow | 3.162 | 3.217 | 0.698 | 1.439 | +1.77% |
| 1M wander | 4.622 | 4.635 | 6.853 | 8.021 | +0.28% |
| 1M stored follow | 7.842 | 7.975 | 6.686 | 9.579 | +1.69% |

Off results remain within this run's noise envelope; 1M wander differs by about 0.3% and follow by
1.7%. The short 100k wander sample spans 0.659–0.748 ms; dense 100k follow spans 2.771–3.416 ms.
No fixed-state speedup or exact trajectory replay is claimed: GPU scatter and capped neighbor
selection are nondeterministic. `all-headless-medians.md` retains each median/p95, alongside raw logs.

Cap decision: keep explicit cap activation for formation, with zero exact still available. At 11
simulated seconds cap 16 reduces 1M step cost relative to exact while preserving complete separation.
On versus Off changes navigation, density and wall-branch choice, so it is not an isolated analytic
formula cost. The original baseline steering/density defaults are not retuned.

Convergence is measured against each record's own carried slot, using the same analytic CPU target
and actual group ranges. Default 11-second runs are still assembling at 1M. Additional cap-16 runs
use `GPU_BENCH_WARMUP=3540` plus 60 samples (60 simulated seconds); these show a recognizable shape
with nonzero slot error and a long tail. Both replicas are retained:

| Case / cohort | Simulated seconds | Run 1 / 2 median (p95) slot distance, world units |
| --- | --- | --- |
| 100k wander / on-cap16 | 11.000 | 362.707 (581.925) / 362.394 (583.745) |
| 100k stored follow / on-cap16 | 11.000 | 274.201 (529.398) / 272.841 (531.019) |
| 1M wander / on-cap16 | 11.000 | 1012.920 (1575.149) / 1012.864 (1575.453) |
| 1M stored follow / on-cap16 | 11.000 | 930.088 (1455.942) / 930.116 (1455.714) |
| 100k wander / on-cap0 | 11.000 | 381.789 (588.088) / 381.906 (587.884) |
| 100k stored follow / on-cap0 | 11.000 | 290.313 (536.605) / 290.217 (536.758) |
| 1M wander / on-cap0 | 11.000 | 1037.133 (1592.615) / 1037.167 (1592.770) |
| 1M stored follow / on-cap0 | 11.000 | 951.040 (1480.432) / 950.990 (1480.623) |
| 100k wander / convergence60 | 60.000 | 24.639 (45.015) / 24.723 (44.978) |
| 100k stored follow / convergence60 | 60.000 | 30.088 (45.201) / 30.064 (45.237) |
| 1M wander / convergence60 | 60.000 | 69.391 (726.329) / 69.379 (725.689) |
| 1M stored follow / convergence60 | 60.000 | 51.740 (289.038) / 51.975 (289.578) |

Evidence root:
`/tmp/claude-1000/-home-martines-Projects-autonomous-characters-3d/af1dacb4-dc47-4bb5-8e8c-02c6cd1b95e8/scratchpad/shots/stage7d/`.

Rendered 7d measurements: four fresh apps, physical 1776×1000 floating window, immediate
presentation, 100k GPU Dart, Studio/Aurora, heading color, neutral sizes, 4× MSAA, 1× simulation,
camera yaw 25°/pitch 18°/distance 2,500. Sphere uses default controls, cap 16, no groups/follow/
attractor/obstacles. Three-second warmup, four-second sample, exit at eight seconds. These are
whole-frame intervals during assembly, not steady-state GPU timestamps. No captures during timing.
Scripts, launch uptime and window-size transcripts are `timing-{off,on}-{1,2}.*` in the evidence root.

| Formation | Run 1 mean / p50 / p95 ms (frames) | Run 2 mean / p50 / p95 ms (frames) |
| --- | --- | --- |
| Off | 1.072 / 0.980 / 1.470 (3,729) | 1.002 / 0.970 / 1.260 (3,991) |
| Sphere | 1.058 / 0.988 / 1.369 (3,782) | 1.028 / 0.989 / 1.293 (3,892) |

Desktop: `desktop.ac3d` / log runs for 52 real seconds and reaches `exit`. Native window is temporarily
floated/resized to 1776×1000; no desktop config changes persist. Camera yaw 25°/pitch 18°, distance
1,500 (650 for Octree), vsync about 60 Hz, 4× simulated speed, default flocking/formation settings;
no steering factors were weakened to obtain the shapes. Captures:

- `sphere-assembling.png`: 20k GPU, about four simulated seconds after resume.
- `sphere-assembled.png`: same sphere after about 60 simulated seconds; visible shell, nonzero slot error.
- `morph-midway.png`: paused about halfway through the four-simulated-second sphere→helix target morph;
  actual vehicles lag the moving target shape rather than being teleported.
- `helix.png`: about 32 further simulated seconds after resume; recognizable three-turn coil.
- `groups-formations.png`: two groups, cyan/orange, separate spheres on the XY grid, about 32 simulated
  seconds after group count/slot reassignment. Existing membership policy remains unchanged.
- `formation-obstacle.png`: same pair, reference-radius-14 sphere at (-313,0,180); soft avoidance
  deforms the left formation after about 16 simulated seconds.
- `paused-a.png`, `paused-b.png`: paused during another active morph/assembly; absolute differing pixel
  count is zero in `960x750+420+100`, excluding UI/FPS. Two real seconds apart, no scene motion.
- `octree-formation.png`: 1,500 CPU Octree members, two separate spheres, about 20 simulated seconds
  after backend respawn; scene settings persist and slots rebuild.

Nonzero convergence error is expected with preserved flocking, wall drag and exact separation.
Obstructed/out-of-wall slots may never converge; helix crowding and very large group combinations
remain limits. Measurements cover this RX 7900 XT only. Stage 7e is not implemented.

### Stage 7e implementation, diagnostics and acceptance gap

Stage 7d is the committed baseline `77406a4`. The final mechanic now adds up to four visible,
masked flee controls, static/Orbit motion, shared simulated-substep snapshots, bounded arena
records, sixteen fixed steer variants, Swarm controls and validated scenarios. No Boid/instance
stride, binding count, dependency, wall drag, navigation default or vehicle drawing changes.

CPU/GPU sum outward smoothstep weights, use +X at coincidence, clamp the sum to length one and
multiply by max speed × predator strength. Add after formation and before mass division. Logical
mask group is zero while groups are disabled; otherwise use carried mapped membership. Every
predator is evaluated outside neighbor loops. Predators do not independently enable neighbor caps.
The selected global target still overrides group targets, and formations retain their navigation
override. Flee adds to all other terms; arrival regroups when threats leave. Static/Orbit use one
world snapshot per substep on both backends. Orbit uses an anchored focus or enabled formation
center, not the live camera; explicit edits remain live while paused. Scene settings and GlobalState
are never written by simulated path motion. The extracted frame changes only when its value changes.

The arena remains eight storage bindings: persistent metadata/order region `[0,5C/2)`, sixteen
sphere vec4s `[5C/2,5C/2+16)`, eight group-range vec4s `[5C/2+16,5C/2+24)`, then 32 predator vec4s
`[5C/2+24,5C/2+56)`. Each of four substeps has four two-vec4 `(position.xyz, world_radius)` / mask
records. Uniforms append offset/count/substep and strength at Naga-compatible byte offsets 400/416,
raising Params from 400 to 432 bytes. Arena allocation is `40C+896` bytes. Growth rewrites the tail.
One changed moving frame writes 512 bytes once before all substeps; no per-boid or per-substep
uploads. Static/paused samples skip unchanged writes. Entering pause after multiple substeps may
replace historical samples with the final frozen position once. Strength/visibility alone do not
upload records. The upload fixture proves growth and unchanged-frame counts; moving parity checks
all four slices. The benchmark reports exactly 660 uploads for 600 warmup + 60 timed steps in every
moving case (512 bytes each). Sixteen bounded prebuilt steer variants are the direct four-bit
extension of 7d; they create no runtime pipelines and compile disabled feature work out.

**Performance acceptance is pending an idle-machine rerun.** Both 7d baselines ran with low load.
An unrelated Blender process (PID 389450, another project) started before post-change measurements,
used substantial CPU and stayed active. The worker asked the user for an idle window and did not
alter or terminate that process. A three-second sample at 23:58Z still measured 224% Blender CPU
and 9.78% total machine CPU use; the discrete GPU busy counter read 7%. Post-change numbers below are explicitly provisional diagnostics,
not the requested idle acceptance gate. They show no apparent Off regression, but cannot establish
an uncontaminated 3–5% comparison. Keep Stage 7 completion open until the owner reruns these cases
on an idle machine. Baseline values remain valid; repeat them too if conditions/hardware changed.

RX 7900 XT, release seeded headless benchmark, 600 warmup + 60 timed steps, dt=1/60, medians in ms.
Configured cap 16 is inactive in wander and active in follow. Each case starts at PCG frame zero.

| Count / navigation | 7d baseline 1 / 2 | 7e Off 1 / 2 (provisional) | 4 Orbit On 1 / 2 (provisional) |
| --- | --- | --- | --- |
| 100,000 wander | 0.698 / 0.688 | 0.690 / 0.654 | 0.862 / 0.927 |
| 100,000 follow | 3.417 / 3.094 | 3.304 / 2.932 | 3.788 / 3.297 |
| 1,000,000 wander | 4.714 / 4.656 | 4.587 / 4.606 | 5.394 / 5.361 |
| 1,000,000 follow | 7.745 / 7.976 | 7.976 / 8.024 | 7.917 / 7.916 |

Combined diagnostic, 100k wander: groups 2, four spheres, sphere formation, attractor at origin,
four orbiting predators. Cap 0 median **41.847 ms**, p95 **44.558 ms**; cap 16 median **6.833 ms**,
p95 **10.054 ms**. Both finish 600+60 steps without slow-step cutoff. After 11 simulated seconds,
median slot error is 638.760 / 637.988 world units for cap 0 / 16: competing attraction/fleeing can
prevent slot convergence. No density or force defaults were retuned. This is one run per cap as
requested; it remains non-idle and provisional.

Rendered diagnostics: 100k Dart, Studio/Aurora, Heading, 4× MSAA, 1776×1000, camera yaw25/pitch18/
distance2500, immediate presentation, 1× simulated time, 3 s warmup + 4 s measured. On means four
Orbit controls with radius20, path radius50 and speed30°/simulated s. Fresh apps alternate Off/On;
every script reaches `exit`. Frame intervals include simulation, rendering, UI, gizmos and uploads.

| Rendered 100k | Frames | Mean ms | p50 ms | p95 ms |
| --- | --- | --- | --- | --- |
| off-1 | 3006 | 1.331 | 1.329 | 1.892 |
| on-1 | 2908 | 1.375 | 1.373 | 1.937 |
| off-2 | 3144 | 1.272 | 1.270 | 1.872 |
| on-2 | 2974 | 1.345 | 1.343 | 1.966 |

Recorded local uptime (CET; raw .load files accompany logs):

- `before-1`: 00:17:33 up  5:58,  1 user,  load average: 0.18, 0.23, 0.57
- `before-2`: 00:21:11 up  6:02,  1 user,  load average: 0.02, 0.15, 0.46
- `off-1`: 00:40:40 up  6:21,  1 user,  load average: 1.88, 1.61, 0.85
- `off-2`: 00:50:19 up  6:31,  1 user,  load average: 3.04, 2.83, 1.78
- `on-1`: 00:49:42 up  6:30,  1 user,  load average: 5.71, 3.20, 1.85
- `on-2`: 00:50:45 up  6:31,  1 user,  load average: 2.08, 2.62, 1.74
- `combined`: 00:51:41 up  6:32,  1 user,  load average: 0.86, 2.19, 1.64

Repeat commands on an idle machine (guard cargo/rustc/autonomous before every invocation):

```sh
GPU_BENCH_COUNTS=100000,1000000 GPU_BENCH_CAPS=16 GPU_BENCH_MODES=wander,follow CARGO_BUILD_JOBS=4 cargo test --release --locked vehicle::tests::gpu_flocking_benchmark -- --exact --ignored --nocapture --test-threads=1
GPU_BENCH_PREDATORS=4 GPU_BENCH_COUNTS=100000,1000000 GPU_BENCH_CAPS=16 GPU_BENCH_MODES=wander,follow CARGO_BUILD_JOBS=4 cargo test --release --locked vehicle::tests::gpu_flocking_benchmark -- --exact --ignored --nocapture --test-threads=1
GPU_BENCH_COMBINED=1 GPU_BENCH_COUNTS=100000 GPU_BENCH_CAPS=0,16 GPU_BENCH_MODES=wander CARGO_BUILD_JOBS=4 cargo test --release --locked vehicle::tests::gpu_flocking_benchmark -- --exact --ignored --nocapture --test-threads=1
```

Final gates pass: formatting check; 122 release tests, 15 ignored (13 device tests and two benchmarks);
warning-free release build; Clippy seven existing warnings, repeated for the test target; ten-second
native startup smoke; diff check. No dependencies or historical review records changed.
The single final ignored GPU run passes all 13, under the existing eight-binding test setup. New
mixed parity covers static and orbit paths, all four substep sample slices, three groups, masks
1/5/255/0, an obstacle, attractor and a blended formation, with following both On/Off and unlimited/
nontruncating capped queries. New predator worst position error is 7.630326e-6 world units and
velocity error 1.2361032e-5. Growth/count/paused upload tests check exact arena records and counters.
CPU tests cover falloff edges, coincidence, clamp, masks, Off bytes, group-zero semantics, path
scaling/determinism/substep chunking, pause/speed, and unchanged settings/frame ticks. Existing mode/
membership fixtures now also preserve predator scene settings across CPU/backend/count changes.
Parser tests validate commands, masks, ranges, capacities and unchanged setters.

Desktop artifacts live under
`/tmp/claude-1000/-home-martines-Projects-autonomous-characters-3d/af1dacb4-dc47-4bb5-8e8c-02c6cd1b95e8/scratchpad/shots/stage7e/`:

- `orbit-a.png`, `orbit-b.png`: 20k GPU wander with a moving magenta control.
- `following-orbit-a.png`, `following-orbit-b.png`: 20k follow bends around the opposite sides of the moving threat.
- `formation-deformed.png`, `formation-regrouped.png`: formation deforms, then regroups after clearing.
- `groups-mask.png`: two formations; identical fields affect only cyan group 0, leaving orange group 1 intact.
- `octree.png`: 1,500 CPU vehicles with formation/fleeing.
- `paused-a.png`, `paused-b.png`: Orbit paused; scene crop `960x750+420+100` has **zero** differing pixels across two seconds. UI/FPS remain live outside the crop.
- `combined.png`: actual 100k GPU combined mechanics, cap16.

Scripts, uptime, full timings, verification logs and pixel comparison are beside the images. No
commit, push, merge, reset or discard was performed. No Cargo/rustc/native flock app remains after
checks. Remaining limitations: idle-machine performance acceptance; soft fields permit penetration
and competing forces can prevent formation convergence; no chase/capture, predator obstacle
avoidance or per-boid predator role; other GPU/CPU performance and dense eight-group combinations
are not established. Stage 7 mechanics are implemented and behavior-verified; overall completion
awaits idle performance acceptance and owner review/commit.

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

7d: new `formation.rs` owns `Formation`, `FormationMode`, `FormationFrame`, target/arrival functions
and UI. `SwarmScene` embeds settings; `SwarmUi` supplies scene, actual phase and orbit focus. CPU
`update_formation_slots` runs in maintenance; `group_flocking_force` adds arrival before mass division
and `step_cpu` selects the frame's substep sample. GPU appends three uniforms, three maintenance
pipelines and four formation-enabled steer variants. Group ranges share the existing arena;
`counts`/`starts` are temporary maintenance scratch. Scenario adds mode/center/extent/arrival/cap/
morph/speed setters; the benchmark gains optional formation and warmup controls.

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

- 2026-10-31: Orchestrator committed 7c as 4e0d5a6; authorized 7d only. Finalized analytic formulas, morph/substep snapshot, slot maintenance, arena ranges and explicit formation cap policy before implementation.

- 2026-10-31: Stage 7d CPU gate passes (116 tests), analytic GPU benchmark runs at 100k/1M, and convergence measurement extended to 60 simulated seconds. No target-buffer allocation or per-frame scene write was introduced.

- 2026-10-31: Stage 7d desktop scenarios and repeated rendered intervals pass; recorded cap/convergence limitations and zero-pixel paused proof. The single final 11-test device gate passes.

- 2026-10-31: Stage 7d complete and uncommitted; final 116-test / 11-device-test gate, Clippy baseline, desktop exit/pixel proof and all performance evidence recorded. Stage 7e remains separate owner-authorized work.

- 2026-10-31: Stage 7d committed by orchestrator as 77406a4; 7e only authorized. Finalized flee law, mask semantics, anchored paths, shared substep snapshots and 512-byte arena reservation before code.
