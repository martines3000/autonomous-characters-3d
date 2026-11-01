# Visual foundation and vehicle appearance

This ExecPlan is a living document maintained according to `PLANS.md`. It implements stages 1–2 of
[the umbrella plan](2026-10-31-visual-showcase-and-swarm-features.md). Stage 1 was accepted in `e635c6d`; Stage 2 is now authorized for
this implementation worker. The orchestrator owns review, commits, STATUS and the umbrella plan.

## Purpose

Stage 1 gives the existing instanced darts a readable studio scene, independent heading palettes,
and usable framing at every supported count. A developer scenario script exercises the desktop
without pointer injection and records frame times and screenshots. Stage 2 adds render-only
size/proportions, shared procedural shapes and heading/altitude/speed/solid coloring without
changing the simulation.

## Progress

- [x] (2026-10-31 09:40Z) Read repository instructions, umbrella plan, prior visual plan, roadmap and renderer.
- [x] (2026-10-31 09:56Z) Stage 1: scene styles, palettes, target contrast, fog, framing and collapsible UI.
- [x] (2026-10-31 09:56Z) Stage 1: scenario harness and seven new regression tests.
- [x] (2026-10-31 10:02Z) Stage 1: release checks, nine inspected desktop captures, repeated frame measurements and diff check passed; all scenario launches exited cleanly.
- [x] (2026-10-31 10:24Z) Stage 1 round 2: gradient Studio sky, fogged solid floor/subtle grid, six-stop cyclic palettes, emission, eight captures and repeated focus-monitored timings.
- [x] (2026-10-31 11:41Z) Stage 2: shared Dart/Chevron/Fish/Bird meshes, render-only size/proportions and four color modes.
- [x] (2026-10-31 11:41Z) Stage 2: 32-byte CPU/GPU instance packing, scenario/UI integration and preservation tests.
- [x] (2026-10-31 11:41Z) Stage 2: release checks, 15 inspected close-up captures, live GPU/Octree switches and four per-shape timings.

- [x] (2026-10-31 12:29Z) Approved review fixes: held drags resume after framing; live target bounds extend sky/fog/far clipping. Release checks and requested desktop capture passed.

## Surprises & Discoveries

- Stage 2: Bevy's exact Cone mesh deliberately uses a zero normal at the singular apex, with unit
  side/base normals. Keep it unchanged for Classic; custom meshes use nonzero unit face normals.
  Geometry tests allow that one documented apex exception and verify winding for every triangle.

- Round 2: a dark blue ground base color still appeared gray under the existing lights because of
  neutral dielectric specular reflection. Set ground reflectance to zero; keep roughness 1 and
  metallic 0. Grid strips use the same fogged material, not unfogged gizmos.
- Round 2: uncontrolled desktop focus invalidated the first timing batch. Bevy's default unfocused
  Winit update mode waits 1/60 s, independent of PresentMode. One Classic case was exactly 60 FPS
  at 1662×1390. Repeated all four cases with focus and window-size monitoring; each accepted case
  remained focused at 1667×1390. The application/harness behavior was not changed.
- Round 2: requiring a green-dominant stop was an incorrect palette test for the requested warm
  Dusk family. Regression checks instead require cyclic endpoints, distinct stops, channel range
  and a luminance ratio above 3; Dusk keeps its requested warm hues.

- Before Stage 1, exponential fog was recomputed from count alone, independent of camera position.
- First following screenshot showed that an opaque sphere can still be hidden by the flock. A
  constant-screen-size locator with depth bias -1 now marks it; the pick mesh remains unchanged.
- Enabling a scrollable egui Window retained its old 100-point default height and clipped the menu.
  The initial height is now 440 points, capped by viewport height; expanded sections scroll.
- The orbit controller exposes anchor depth and motion state, so reframing can preserve orientation
  without adding a second camera controller.

## Decision Log

- Decision (stage 2): keep Dart and neutral size/proportions as appearance defaults. Scene style
  changes never reset vehicle shape or coloring; Classic reproduction requires Dart, all multipliers
  1, Heading and Classic palette. Mesh handles are cached once and swapped on the single flock entity.
  Rationale: independent appearance controls must preserve both CPU and GPU state.
  Date: 2026-10-31.
- Decision (stage 2): use low-poly closed prisms for flat silhouettes and faceted bipyramids for
  bodies; all meshes have length 2.5 along local +Y. Inverse-transpose normal scaling handles proportions.
  Rationale: thin volume remains visible edge-on and avoids double-sided rendering.
  Date: 2026-10-31.
- Decision (stage 2): keep InstanceData at 32 bytes. Store normalized speed (16 bits) and a reserved
  group field (7 bits, currently zero) in the mantissa of a normal f32 in [1,2). CPU and WGSL use
  identical bit packing; physical size now comes from the uniform multiplied by render-only size.
  Rationale: avoids stride growth, NaN/subnormal payloads and fractional speed 1 carrying into group.
  Groups are not implemented; their transport capacity is documented for Stage 7.
  Date: 2026-10-31.
- Decision (stage 2): altitude and speed use the palette's non-wrapping first-to-sixth-stop ramp;
  heading retains its full cyclic wheel. Classic non-heading modes use a red-to-violet HSV ramp.
  Rationale: minimum and maximum should have different colors for scalar quantities.
  Date: 2026-10-31.

- Decision (round 2): use an unlit, vertex-colored UV sphere centered on the camera, front-face culled,
  with fog disabled, picking ignored and frustum culling disabled. Its radius exceeds every wall
  corner distance; far clipping exceeds its radius. Classic hides all Studio surfaces.
  Rationale: reuse StandardMaterial rather than introduce another material/shader pipeline.
  Date: 2026-10-31.
- Decision (round 2): one rough nonmetallic ground mesh includes a few slightly raised grid strips.
  Both use the same fogged material; Studio walls become corner brackets. Classic retains its exact gizmos.
  Rationale: bounded geometry, no texture or shader dependencies, and actual fog fading on the grid.
  Date: 2026-10-31.
- Decision (round 2): six distinct palette stops plus a repeated first endpoint; a 15% Studio emission
  term joins the existing rim. Aurora is the default after screenshot comparison: teal/green/lime and magenta remain distinct
  against the navy sky, including a 5-degree view and the 200-vehicle following state.
  Rationale: encode the cyclic endpoint explicitly and recover visible direction changes at distance.
  Date: 2026-10-31.

- Decision (round 1; palette default superseded in round 2): Studio is the default scene; Ocean is the default heading palette. Scene and palette
  switches are independent. Classic scene plus Classic palette restores the old scene constants,
  light transform, HSV function and material roughness. Target visibility and camera-aware fog are
  usability fixes shared by both styles.
  Rationale: preserve the original presentation while allowing palette comparisons under fixed lighting.
  Date: 2026-10-31.
- Decision: one instancing-owned uniform carries palette colors and material parameters. Reserve the
  render-scale and color-mode fields at neutral values for stage 2; no per-vehicle appearance data.
  Rationale: CPU and GPU instances share exactly the same draw and simulation data remain unchanged.
  Date: 2026-10-31.
- Decision: count changes debounce for 0.35 seconds; automatic framing defaults on. Reframe moves
  along camera forward only, preserves orientation and pans, and resets controller momentum.
  Rationale: avoid fighting a live slider; no stage-5 camera presets or transitions.
  Date: 2026-10-31.
- Decision: fog and far clipping use a conservative camera-to-farthest-wall bound every frame.
  Rationale: manual zoom and disabled auto-frame must remain usable too.
  Date: 2026-10-31.
- Decision: no bloom, shadows or additional postprocessing. Use key/fill/ambient and a small material rim.
  Rationale: retain a cheap scene with no extra fullscreen pass or per-vehicle lights.
  Date: 2026-10-31.

## Assumptions and Open Questions

- Assumption: a uniform, one extra unshadowed directional light and a small rim term are affordable at 20k.
  Validation: repeated fresh-launch immediate-present frame measurements with the same settings.
  If wrong: simplify the rim or fill light before handoff.
- Scenario timings use real elapsed time; rendering is interactive and GPU scatter is not bitwise deterministic.
  No deterministic simulation claim is made. Keep screenshot capture outside measurement windows.

## Outcomes & Retrospective

Stage 2 implementation: four cached shared shapes, render size/length/width, Heading/Altitude/Speed/
Solid coloring and color picker, UI and scenario commands are complete. Release tests pass (48,
five ignored); release build has zero compiler warnings, Clippy retains seven existing warnings.
Geometry tests verify triangle counts, finite data, forward bounds, normals and winding. Actual
appearance-system tests preserve every GlobalState field and encoded GPU input, CPU entity IDs,
transforms, velocities and follow state across switches. Desktop and timing evidence is recorded
in the Stage 2 artifact section below. No group behavior or model-loading feature is implemented.

Round 2 implementation adds a true vertical vertex-colored sky, fogged ground/grid, faint corner
brackets, saturated cyclic six-stop palettes with Aurora default, and 15% Studio emission. The
appearance uniform is now 160 bytes (seven colors, material, shading and modes). Classic shader
HSV, scene constants and gizmo geometry remain unchanged; Studio surfaces hide in Classic.
Round 2 passes 43 release tests (five ignored), warning-free release build and exactly seven
pre-existing Clippy warnings. Eight new captures were inspected. Focus-monitored 20k runs show
no measured increase above the approximately 3% frame-time budget. No further rendering passes
or shader assets were needed. All original mechanics and the orchestrator-owned files are preserved.

Round 1 evidence:

Stage 1 implementation passes 40 release tests (five ignored) and a warning-free release build.
Clippy reports exactly seven pre-existing warnings, none added. A source comparison also confirms
`GlobalState::default`, the simulation modules/shader and Cargo manifests are byte-for-byte unchanged.
Seven new tests cover Classic
constants, scheduled style/palette updates, framing/fog math, settling/manual frame behavior and
scenario parsing/application. Stage 2 remains planned for the orchestrator's next worker.

At 20,000 GPU vehicles, Studio mean frame intervals average 1.3% above Classic, with nearly
identical p50 values (see Artifacts). Variable interactive timing prevents attributing this to rendering alone.
The new scene has no additional fullscreen effects; the bounded target locator uses one gizmo group.

## Context and Orientation

`src/main.rs` owns app setup, simulation settings, egui and input. `src/scene.rs` owns appearance, sky/ground geometry,
scene constants, wall/grid drawing, fog and frame-world math. `src/instancing.rs` owns the shared
appearance uniform and draw bind group; `assets/shaders/vehicle_instancing.wesl` consumes it.
`src/target.rs` retains the existing pick mesh and selection behavior, makes its sphere opaque and
adds a screen-sized, depth-independent gizmo locator. `src/scenario.rs` owns the
optional script parser, timed commands and frame summaries. Force laws/defaults stay unchanged;
Stage 2 changes only render-output metadata in the CPU/GPU simulation modules.

## Plan of Work

1. Extract scene setup and introduce independent scene styles and heading palettes.
2. Connect an appearance uniform to the instanced draw; improve target contrast and frame/fog math.
3. Group existing controls without changing their ranges; add appearance/framing controls.
4. Add the opt-in scenario module, startup validation and parser tests. Document its full grammar.
5. Run focused checks sequentially, then scripted desktop checks and repeated measurements.
6. Stage 2: add `vehicle_appearance.rs` with shared mesh generation/cache, appearance settings, UI
   and stream packing helpers. Extend the appearance uniform, CPU/GPU output encoding and shader.
7. Add scenario commands and preservation/geometry/packing tests. Capture close views and live
   switches in GPU/Octree; record one 20k default-framing case per shape. No Stage 3 model loading.

## Concrete Steps

Before every Cargo command: `pgrep -a cargo; pgrep -a rustc`; wait if either reports a process.
Run sequentially with no app running:

1. `CARGO_BUILD_JOBS=4 cargo fmt --all`
2. `CARGO_BUILD_JOBS=4 cargo test --release --locked`
3. `CARGO_BUILD_JOBS=4 cargo build --release --locked`
4. `CARGO_BUILD_JOBS=4 cargo clippy --release --locked --all-targets`
5. `AC3D_SCRIPT=<script> CARGO_MANIFEST_DIR="$PWD" ./target/release/autonomous-characters-3d`
6. `pgrep -a autonomous`
7. `git diff --check`

The orchestrator owns the final gate, ignored GPU tests and benchmarks. No commits by this worker.

## Validation and Acceptance

Classic constant regression; independent scene/palette switches without simulation mutation;
framing and fog at counts 0, 200, 2,000, 20,000 and 1,000,000; script grammar and invalid-value
line numbers. Existing behavior tests pass. Compiler warnings remain zero; Clippy remains at the
seven pre-existing warnings. Capture Studio 20k GPU, Studio 200 after auto-frame, Classic 20k,
every palette and both target states. Repeat 20k GPU frame measurements without capture.
Round 2 also requires no visible Studio sky/ground in Classic; sky mesh faces enclose every wall
and remain within far clipping at 0/200/20k/1M, ignore picking/culling/fog, and follow the camera.
Studio fog matches horizon exactly and retains at least 70% transmission at the farthest wall.
Six distinct palette stops repeat the first endpoint, and the original HSV branch stays unchanged.
Use the same 20k Ocean timing scenario twice per style; budget about 3% added frame time or explain it.

## Idempotence and Recovery

Scripts are opt-in and run once per application launch; use separate output files when keeping runs.
They may replace screenshots at specified paths. Exit scripts explicitly and verify no process remains.
The orchestrator may revert the eventual stage commit; do not reset or discard the shared worktree.

## Artifacts and Notes

Screenshot/log root:
`/tmp/claude-1000/-home-martines-Projects-autonomous-characters-3d/af1dacb4-dc47-4bb5-8e8c-02c6cd1b95e8/scratchpad/shots/stage1/`.
Checks (sequential, process checks before every Cargo command, `CARGO_BUILD_JOBS=4`):

- `cargo fmt --all`: pass.
- `cargo test --release --locked`: 40 passed, zero failed, five ignored (0.10 s).
- `cargo build --release --locked`: pass, zero compiler warnings.
- `cargo clippy --release --locked --all-targets`: seven existing warnings (two octree,
  one unchanged target query type, four vehicle); zero new warnings.
- Ignored GPU tests and benchmarks were not run; final owner gate remains with the orchestrator.

Repeated rendered frame measurements: native release, Linux Omarchy 4.0.6 / kernel
7.2.8-9-omarchy, Ryzen 9 7900X, Radeon RX 7900 XT (RADV NAVI31), Vulkan / Mesa 26.2.5-arch1.1.
Physical window 1667×1390 (Wayland tiled the requested 1280×720 window). Each case starts a fresh
app, GPU mode, 20,000 vehicles wandering, Ocean palette, auto-frame off, camera yaw 0°,
pitch 17.28°, distance 2800. Present mode AutoNoVsync. Built-in GPU PCG spawn at frame zero;
no user seed, no readback, real interactive delta, non-deterministic scatter ordering. Five seconds
warm-up, ten seconds sampled; no screenshots inside or adjacent to a measurement. Four independent
launches in order Classic 1, Studio 1, Studio 2, Classic 2. No Cargo or other app instance overlaps.

| Style / run | Frames | Mean ms | p50 ms | p95 ms | FPS | Load before run (1/5/15 min) |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| Classic / 1 | 9385 | 1.066 | 0.991 | 1.381 | 938.43 | 3.13 / 2.64 / 2.15 |
| Studio / 1 | 9289 | 1.077 | 0.985 | 1.402 | 928.90 | 3.46 / 2.74 / 2.19 |
| Studio / 2 | 9162 | 1.091 | 0.991 | 1.403 | 916.24 | 3.29 / 2.74 / 2.19 |
| Classic / 2 | 9306 | 1.075 | 0.983 | 1.400 | 930.60 | 3.22 / 2.75 / 2.21 |

Mean intervals average to 1.0705 ms Classic and 1.084 ms Studio, a 0.0135 ms (1.3%) increase.
The averaged p50 values are 0.987 ms Classic and 0.988 ms Studio. The mean difference is comparable
to run variation (Studio span 0.014 ms); these runs do not isolate a causal lighting/shader cost.
These include CPU, GPU/present overlap and desktop scheduling, not isolated GPU shader time.
Scripts and full per-case adapter/frame logs are `measure-{classic,studio}-{1,2}.{scenario,log}`
under the screenshot/log root. `tests.log`, `build.log` and `clippy.log` preserve the focused checks.

The final screenshot script `screenshots.scenario` exited normally at 47 seconds. All nine captures
were inspected: clear menu, lit vehicles, readable target marker in both states, correct palettes,
auto-frame at 200, CPU rendering at 2000 and empty world. `desktop.log` has no warnings or shader
errors. `pgrep -a autonomous` returned no process after every launch. A real invalid script launch
also rejected `set palette lava` at startup with `line 2` and nonzero exit (`invalid.scenario`, `invalid.log`).

Final screenshots:

- `studio-20000-ocean-idle.png`: default Studio/Ocean, 20k GPU and idle red target.
- `studio-20000-dusk.png`: Dusk heading palette.
- `studio-20000-classic-palette.png`: original HSV heading palette under Studio lighting.
- `classic-20000.png`: Classic scene and Classic palette.
- `studio-20000-following.png`: following green target in a dense GPU flock.
- `studio-200-auto-frame-idle.png`: count 200 after auto-frame.
- `studio-200-following.png`: follow at count 200.
- `studio-2000-cpu-octree.png`: shared appearance in the CPU path at count 2000.
- `studio-empty.png`: zero-count world and target.

The screenshot scenario changes count 20k → 0 → 200 to inspect a freshly spawned small flock.
A direct reduction preserves/truncates existing boid positions, per the existing simulation rule;
those boids can take time to return inside the smaller walls. Framing fits walls, not outlying boids.

Round 2 artifact root:
`/tmp/claude-1000/-home-martines-Projects-autonomous-characters-3d/af1dacb4-dc47-4bb5-8e8c-02c6cd1b95e8/scratchpad/shots/stage1b/`.

Round 2 checks, sequential with process checks and `CARGO_BUILD_JOBS=4`:

- `cargo fmt --all` and `cargo fmt --all -- --check`: passed.
- `cargo test --release --locked`: 43 passed, zero failed, five ignored (0.11 s).
- `cargo build --release --locked`: passed, zero compiler warnings.
- `cargo clippy --release --locked --all-targets`: seven pre-existing warnings, none added.
- The three new round 2 tests cover palette endpoints/channel/value range; actual sky mesh face
  containment at 0/200/20k/1M with camera at center, standard orbit and far zoom; and Studio surface
  materials/picking/culling, camera/count updates and Classic hide/show. The shared-uniform test now
  checks its 160-byte layout and Studio emission 0.15 versus Classic 0.
- SHA-256 comparisons against the start of round 2 confirm no changes to `main.rs`, `target.rs`,
  simulation modules/shader, Cargo manifests, STATUS or the umbrella plan.
- Ignored GPU tests and benchmarks were not run. No commits, pushes, resets or discards.

Round 2 final timing: same machine/driver, native release, 20k GPU wander, Ocean palette,
AutoNoVsync, auto-frame off, camera yaw 0°, pitch 17.28°, distance 2800. Fresh launch per case,
five-second warmup and ten-second sample, built-in GPU PCG spawn at frame zero, real interactive
delta and nondeterministic scatter, no capture during measurement. Order Classic 1, Studio 1,
Studio 2, Classic 2. Physical resolution 1667×1390 for all four. External `hyprctl` queries sampled
focus and window size every 0.5 seconds; zero violations during each measurement window.
The failed legacy Hyprland focus/resize commands had no effect; accepted windows were naturally
focused at the same size. The saved `measure-focused.py` retains monitoring only.

| Style / run | Frames | Mean ms | p50 ms | p95 ms | FPS | Load before run (1/5/15 min) |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| Classic / 1 | 8605 | 1.162 | 1.119 | 1.463 | 860.51 | 0.85 / 3.60 / 3.24 |
| Studio / 1 | 8931 | 1.120 | 1.036 | 1.447 | 893.09 | 1.74 / 3.67 / 3.26 |
| Studio / 2 | 8776 | 1.139 | 1.063 | 1.452 | 877.62 | 2.38 / 3.69 / 3.28 |
| Classic / 2 | 8624 | 1.159 | 1.121 | 1.460 | 862.45 | 2.80 / 3.72 / 3.30 |

Average of per-run means: Classic 1.1605 ms, Studio 1.1295 ms (Studio 2.7% lower). No measured
increase exceeded the approximately 3% budget on this machine; do not claim a
causal speedup from adding geometry. Interactive load, frame deltas and GPU ordering still vary;
these are frame intervals, not isolated GPU timestamps. Added geometry is constant: one 32×24 UV
sphere (1472 triangles) and one floor/grid mesh (22 triangles), hidden in Classic. No extra lights
or fullscreen postprocessing passes. The shared appearance uniform adds 64 bytes, not per-vehicle data.

Final logs/scripts: `measure-{classic,studio}-{1,2}.{log,scenario}` and `focus-*.json`.
`trial-{classic,studio}.log` preserves the initial gray-floor diagnostic.
`uncontrolled-*.log` preserves the discarded focus-unstable batch; those are not budget evidence.
`fmt.log`, `tests.log`, `build.log`, `clippy.log` and `desktop.log` preserve checks.
`git diff --check` passed. Every scenario exited normally; `pgrep -a autonomous` found no remaining app.

Eight final captures, all inspected, under the round 2 root:

- `studio-20000-aurora.png`: default palette, gradient sky, dark floor, faint corner brackets.
- `studio-20000-ocean.png`: six-stop Ocean wheel.
- `studio-20000-dusk.png`: six-stop Dusk wheel.
- `studio-20000-classic-palette.png`: original HSV under Studio lighting.
- `studio-200-auto-frame-idle.png`: 200 idle after auto-frame.
- `studio-200-following.png`: 200 following with readable green locator and saturated darts.
- `studio-20000-low-camera.png`: pitch 5°, showing horizon, fogged floor and restrained grid.
- `classic-20000.png`: flat Classic, original full box/grid, no Studio sky/floor.

`screenshots.scenario` switches styles/palettes and resets count through zero before the 200 case;
it exits at 41 seconds. The round 2 floor sets reflectance to zero after initial visual QA removed
an unintended gray sheen. Aurora was selected because teal/green/lime and magenta remain distinct
against the navy sky in both the low-angle 20k and 200 views. Palette capture times differ, so
those images are visual acceptance evidence rather than a deterministic simulation comparison.

### Stage 2: vehicle appearance

Artifacts root:
`/tmp/claude-1000/-home-martines-Projects-autonomous-characters-3d/af1dacb4-dc47-4bb5-8e8c-02c6cd1b95e8/scratchpad/shots/stage2/`.

Triangle counts: Dart 14, Chevron 12, Fish 16, Bird 24. All lengths are exactly 2.5 before scaling.
Dart retains Bevy's exact mesh (including its intentional zero normal at the singular apex);
custom shapes have unit flat face normals, explicit indices and thin closed volume. Cached mesh
handles swap on the same flock entity; no per-vehicle mesh entities, regeneration or simulation reset.

Default appearance remains Dart, size/length/width 1, Heading. Scene style never resets vehicle
appearance. Scalar colors use non-wrapping ramps; Heading preserves the Stage 1 wheel exactly.
Solid color uses 8-bit sRGB, converted to linear for lighting. Size/proportion changes are entirely
render-only, with no control linking them to physical size/separation. UI ranges: size 0.1–5,
length/width 0.25–3. The menu starts at 640 points high, still capped by the viewport and scrollable.

Rejected: growing InstanceData, float integer/fraction packing (speed 1 would carry into group),
new per-vehicle entities, double-sided flat planes, model assets, simulation retuning and automatic
shape resets with style. The normal f32 mantissa stores 16-bit speed and reserved 7-bit group;
current group is zero. CPU/WGSL round identically using floor(speed * 65535 + 0.5).

Checks: `cargo fmt --all`, `cargo test --release --locked` (48 passed, five ignored, 0.11 s),
`cargo build --release --locked` (zero warnings), `cargo clippy --release --locked --all-targets`
(seven pre-existing warnings, none added), all sequential after process checks, CARGO_BUILD_JOBS=4.
Final format/diff checks and desktop evidence passed. Ignored GPU tests/benchmarks
remain with the orchestrator; only the requested running-app measurements are used here.

Close-up scripts use 200 wandering vehicles, camera yaw 0°, pitch 17.28°, distance 200, auto-frame
off, immediate presentation. Target is moved below the view with follow off. Captures at 1.2 s
show individual shapes a few dozen pixels long. The following files cover the requested cases:

- `shape-{dart,chevron,fish,bird}.png`: all shapes in Heading/Aurora.
- `size-0.5.png`, `size-2.png`: Bird at the two render sizes, neutral proportions.
- `long-thin.png`: Fish at length 2.5, width 0.3.
- `color-altitude.png`, `color-speed.png`, `color-solid.png`: scalar and solid modes; Heading is
  covered by the shape captures. Solid is #ffad3b, and the picker is visible.
- `classic-dart.png`: Classic style/palette, Dart, Heading, neutral size/proportions.
- `live-gpu-{before,after}.png`, `live-octree-{before,after}.png`: two captures in each single run,
  at 1.0 and 2.2 s, with only a Dart-to-Bird shape command at 1.3 s. Count stays 200, follow stays
  off, no respawn/reset command occurs. Simulation continues between the captures.

Each script ends in exit; each launcher checks that no app remains. Per-case `.scenario` and `.log`
files reproduce the shots. All 15 PNGs were visually inspected, desktop logs contain no errors or
warnings, and the target/controls behavior remains intact. No commits by this worker.

Per-shape measurements, one accepted focused case each as requested: native release, Ryzen 9 7900X,
Radeon RX 7900 XT (RADV NAVI31), Vulkan/Mesa 26.2.5-arch1.1, Linux Omarchy 4.0.6/kernel
7.2.8-9-omarchy. Physical resolution 1667×1390; 20,000 GPU vehicles wandering; Studio/Aurora,
Heading, size/length/width all 1; default camera orientation and Frame world at 1 s. Five seconds
warm-up, ten-second measurement, AutoNoVsync, no screenshots. Built-in PCG spawn at frame zero,
no explicit seed override, real interactive deltas and nondeterministic GPU scatter. Each case
starts a fresh app. External focus/window-size monitoring every 0.5 s found zero violations during
each sample. Uptime and cargo/rustc/app process checks preceded every launch; no runs overlap.

| Shape | Triangles | Vertices | Frames | Mean ms | p50 ms | p95 ms | FPS | Load (1/5/15 min) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| Dart | 14 | 17 | 8493 | 1.178 | 1.130 | 1.503 | 849.22 | 2.89 / 6.90 / 6.12 |
| Chevron | 12 | 36 | 8877 | 1.126 | 1.039 | 1.442 | 887.72 | 3.66 / 6.88 / 6.13 |
| Fish | 16 | 48 | 8051 | 1.242 | 1.256 | 1.556 | 805.08 | 3.89 / 6.77 / 6.10 |
| Bird | 24 | 72 | 7346 | 1.361 | 1.403 | 1.604 | 734.58 | 4.41 / 6.73 / 6.10 |

These single exploratory cases are rendered frame intervals including desktop/CPU/GPU overlap,
not isolated GPU times or a statistically established shape ranking. Flat normals require more
vertices than the smooth cone. Further rendering optimization and repeated controlled comparisons
remain Stage 4 work; no tuning or extra passes were introduced here.

`measure-{dart,chevron,fish,bird}.{scenario,log}`, `focus-*.json` and `measure-focused.py` reproduce
and record timings. `fmt.log`, `tests.log`, `build.log`, `clippy.log` preserve focused checks.
Source comparisons confirm unchanged GlobalState defaults, gpu_flock.rs, octree.rs, target.rs,
Cargo manifests, STATUS, umbrella plan and historical review files. GPU WGSL differences are
restricted to the instance field name/comment and final speed payload output; forces/boid writes
are unchanged. No app remains after the final scenario. All Stage 2 acceptance items are complete
for orchestrator review; no Stage 3 work started.

### Approved review fixes (2026-10-31)

After accepted stages 1–2 (HEAD 3fbdd58), changed only the two approved review findings.
`camera_input` now restarts held input when framing resets controller motion, covering both
settled automatic framing and the explicit button without changing framing timing or anchors.
`held_camera_drag_resumes_after_automatic_and_explicit_framing` drives input, controller update
and framing headlessly, holding right/middle buttons across the reset and verifying continued motion.

Scene sky, fog and far clipping now include the live target's camera distance plus its scaled
sphere radius; the radius constant is shared with target mesh creation. When that extent exceeds
the farthest wall, fog starts beyond both target and walls. Otherwise existing Studio haze and
Classic fog remain unchanged. Target movement remains unclamped. The extended
`sky_faces_enclose_every_wall_and_stay_inside_far_plane` checks all sky triangle planes, target
sphere clearance, walls and fog/far bounds at counts 0/200/20,000/1,000,000, three camera positions,
six axial target directions and a diagonal. `far_target_extends_live_sky_bounds` verifies live
system updates. Headless surface tests register the mesh visibility requirement normally supplied
by VisibilityPlugin, so their surface assertions execute.

Validation: `cargo fmt --all`, release tests (50 passed, 5 ignored), release build (no compiler
warnings), Clippy (the same 7 existing warnings), and `git diff --check` passed. Every Cargo command
used `CARGO_BUILD_JOBS=4` after checking cargo/rustc processes. One desktop scenario used Studio,
200 GPU vehicles, auto-frame off, camera 0/0/1000 and target 0/0/-1000. Inspected the visible red target
and locator in `/tmp/claude-1000/-home-martines-Projects-autonomous-characters-3d/af1dacb4-dc47-4bb5-8e8c-02c6cd1b95e8/scratchpad/shots/fix12/studio-far-target.png`.
The adjacent `far-target.scenario` and `far-target.log` record commands and clean scripted exit;
`pgrep -a autonomous` found no remaining app. No simulation or Cargo manifest changes.

## Interfaces and Dependencies

Stage 2: `vehicle_appearance.rs` owns `VehicleShape`, `ColorMode`, `VehicleAppearance`, cached
`VehicleMeshes`, `update_shape`, mesh builders and packed speed/group helpers. `scene::Appearance`
contains render-only vehicle settings. The instancing uniform grows from 160 to 192 bytes with
proportions/altitude bounds and solid color; InstanceData remains 32 bytes. CPU sync reads velocities
and packs w; GPU steer changes only its final instance output encoding, not boid state or forces.
`GlobalState` gains Clone/Debug/PartialEq for whole-resource regression snapshots; defaults and
fields are unchanged. No Cargo manifest or dependency changes.

New `scene::Appearance`, `SceneStyle`, `HeadingPalette`, frame/fog helpers and `ScenePlugin`.
New `scenario::ScenarioPlugin` and a typed, startup-validated command list.
Stage 1 instancing seam: shared appearance uniform/bind group (160 bytes after round 2,
192 bytes in Stage 2). Studio uses two
constant mesh entities with StandardMaterial, no extra lights, textures or shader assets. No
dependency or simulation-layout changes.

---

Revision notes:

- 2026-10-31: Initial stage 1 implementation plan; stage 2 retained as planned work.
- 2026-10-31: Stage 1 implemented and verified; stage 2 remains planned. No commits by worker.

- 2026-10-31: Owner requested a Stage 1 art-direction revision. Mechanics and Stage 2 scope remain unchanged; round 2 validation pending.

- 2026-10-31: Stage 1 round 2 implemented and validated; Studio art direction revised, Classic and mechanics preserved. Stage 2 remains planned.

- 2026-10-31: Stage 1 accepted in e635c6d. Stage 2 authorized; implementation and evidence pending.

- 2026-10-31 11:41Z: Stage 2 implemented and verified; Stage 1 remains accepted in e635c6d. No commits by worker.

- 2026-10-31 12:29Z: Applied only the two owner-approved review fixes; regressions, release gates and far-target capture verified. No commits by worker.
