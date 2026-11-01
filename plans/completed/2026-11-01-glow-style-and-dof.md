# Add optional Glow rendering and boid-camera depth of field

This ExecPlan is a living document maintained according to `PLANS.md`.

## Purpose

Select Glow for speed-based vehicle emission/streaks and additive bloom. Ride a boid with optional Gaussian depth of field. Studio remains default; Classic/Studio shading and simulation stay unchanged.

## Progress

- [x] (2026-11-01 10:53Z) Read research, repository contracts and Bevy 0.20.1 APIs.
- [x] (2026-11-01 11:18Z) Implement Glow, DoF, UI/scenarios and five regression tests; 135 normal tests pass.
- [x] (2026-11-01 11:30Z) Fresh-app timing pairs at 20k/100k, one heavy case, all seven required screenshots plus UI restoration proof.
- [x] (2026-11-01 11:34Z) fmt check; 135 release tests; warning-free release build; Clippy at seven baseline warnings; native smoke; 14 ignored GPU tests; README, whitespace and final process checks.

## Surprises & Discoveries

- Observation: custom instancing writes main-pass depth despite using Transparent3d.
  Evidence: MeshPipeline specialization uses opaque depth writes; DoF reads multisampled main depth.
- Observation: WESL conditional blocks flatten their declarations into the containing WGSL scope.
  Evidence: the initial desktop check logged duplicate `speed01` declarations. The Glow blocks now
  use distinct local names; the successful showcase and variant sweep have no shader errors.
  The runner rejects rendering errors even if the app exits normally. The failed log remains at
  `/tmp/glow-dof/showcase-before-shader-fix.log`.
- Observation: HDR specialization in Bevy 0.20.1 uses target-format bits, not a `MeshPipelineKey::HDR` flag.
  Evidence: `ViewKeyCache` includes `MeshPipelineKey::from_target_format(view.target_format)` and
  the original queue already carries that whole key into specialization. The regression checks
  the Rgba16Float key differs from LDR and preserves four MSAA samples.

## Decision Log

- Decision: retain TonyMcMapface, specialize Glow in the existing shader, and use Gaussian DoF.
  Rationale: preserve old rendering paths and reuse the cheapest supported effects.
  Date: 2026-11-01.
- Decision: encode the Glow curve endpoints and stretch coefficient in reserved appearance uniform lanes.
  Rationale: no new uniform layout or instance bytes; old styles keep the same zero reserved values.
  A Glow-only specialization strips all added vertex/fragment math and varyings from old styles.
  Date: 2026-11-01.
- Decision: calibrate Gaussian DoF with sensor height `0.8 * vehicle_size`, aperture f/2.8 and
  a 12-pixel blur cap. Chase focuses at actual camera-to-predicted-boid distance; first person
  focuses ten physical size units ahead of the eye.
  Rationale: a literal 18.66 mm sensor barely blurs these world-scale distances. Body units keep
  the presentation useful without touching simulation size or steering. Gaussian supports both
  LDR/HDR and multisampled main depth and runs after Bloom, before tonemapping.
  Date: 2026-11-01.
- Decision: scale Glow first-person eye clearance by rendered length and the current speed streak.
  Rationale: otherwise a full-speed dart can stretch beyond the old eye position. Studio/Classic
  eye placement remains unchanged.
  Date: 2026-11-01.
- Decision: add `ui off|on` and a 20-line chase marker.
  Rationale: hero captures need the scene unobstructed; the ring uses only existing predicted data.
  Date: 2026-11-01.

## Assumptions and Open Questions

- Assumption: a body-scaled virtual lens gives useful blur in this large-unit scene.
  Why reasonable: the default millimeter sensor treats vehicle distances as many meters.
  Validation: compare DoF off/on in Glow and Studio screenshots and timing.
  If wrong: adjust presentation-only lens constants, never simulation.

## Outcomes & Retrospective

Delivered optional Glow and boid-camera Gaussian DoF, reversible camera components, appearance
controls/scenarios, UI-free capture and a small followed-boid ring. Studio remains default. Old
style constants, appearance uniform bytes, view/MSAA/animation key fields and shader instructions
stay unchanged; the extra Glow key field is false and added math/varyings compile out. Simulation
code, dependency files, instance stride, metadata, compute shaders and simulation defaults are untouched.

Desktop tests cover all built-in shapes and color modes, static multi-part material Glider,
animated imported Bird, Classic/Studio/Glow transitions with DoF, first person and 1M. The
photographic choices remain presentation-only. Gaussian can soften silhouette edges; no Bokeh
or additional prepasses were added. Measurements are full app intervals on one shared-machine
RX 7900 XT, not isolated post-process GPU timestamps. Small DoF differences are within noise.
All required verification passes; the app is closed. No required work remains. The final ignored GPU suite ran once after renderer/camera edits were complete.

## Context and Orientation

`scene.rs` owns style/camera/environment settings; `instancing.rs` specializes the flock pipeline; `vehicle_instancing.wesl` handles every mesh, baked animation and color mode. `follow.rs` owns camera pose and sampled boid position. `scenario.rs` validates developer scripts.

## Plan of Work

Extend style/settings and Glow-only shader specialization; add camera effects lifecycle; connect UI/scenarios and hide-UI; test transitions/math and then measure actual desktop views.

## Concrete Steps

From the repository root, sequentially:

Before every Cargo/app command run `pgrep -a cargo; pgrep -a rustc; pgrep -a autonomous`. Run fmt, release tests/build, Clippy, native smoke, desktop scenarios and the required ignored GPU suite sequentially with `CARGO_BUILD_JOBS=4`. Check `uptime` before desktop measurement.

## Validation and Acceptance

Verify parsing/ranges; emission/streak/focus helpers; old pipeline keys/uniforms; effect component transitions; all required screenshots with UI hidden for hero shots; 20k/100k Studio/Glow and Glow chase DoF timings plus heavy 100k Glow.

## Idempotence and Recovery

Every desktop script ends in exit. Repeat commands only with no other build/app running. No commits, resets or discarding work. Style/effect toggles are reversible; dependency files remain untouched.

## Artifacts and Notes

Logs/scripts live in `/tmp/glow-dof/`; screenshots use the owner-specified `shots/glow/` directory.
`showcase.txt` exercises Studio/Glow, Gaussian DoF, first person, animated imported Bird and 1M.
`variants.txt` exercises all four built-in shapes, every color mode, static multi-part material
Glider, and Glow/Classic/Studio/Glow transitions with DoF active. Both finish with exit and no
shader/runtime warnings or errors. `run_cases.py` checks all three required process inventories
and uptime before every app, enforces one app at a time and rejects runtime/shader errors.

Each timing case uses a fresh release app, UI off, AutoNoVsync, fixed count and style, all optional
simulation mechanics off, default 1× GPU wander, and the same [3,13] second window. Overview
uses the existing preset; paired Chase cases select spawn ID 13, default distance and f/2.8.
The heavy case loads the existing stage4 `heavy.glb`, uses Model colors, and runs only once.
It is 250,048 bytes, SHA256 `cef6e3a8fa8c8a5d93d312954c2a93daca3b2a4f499af1370950096752d8cc66`,
with 10,368 input positions and 3,456 triangles; the existing exact importer retains 2,119 vertices.
Regenerate outside the repository using `python3 scripts/make_example_models.py --kind glider --subdivisions 12 --output /tmp/heavy.glb`.
Its retained input is at
`/tmp/claude-1000/-home-martines-Projects-autonomous-characters-3d/af1dacb4-dc47-4bb5-8e8c-02c6cd1b95e8/scratchpad/shots/stage4/heavy.glb`.
Scripts end in exit at 14 seconds. All accepted measurements use 1667×1390 and four MSAA samples. The machine is Ryzen 9 7900X,
Radeon RX 7900 XT, Linux 7.2.8, Vulkan/RADV Mesa 26.2.5. GPU PCG spawning starts at frame zero;
real frame deltas and scatter order are not seeded or fixed. No Cargo/app work overlapped these
runs. `measurement-runs.log` and `measurement-20k-repeat.log` contain every process inventory,
uptime, settings and raw measurements. The accepted one-minute loads range from 3.01 to 5.11
on 24 logical threads.

| Count / view | Frames | Mean ms | p50 ms | p95 ms | FPS |
| --- | --- | --- | --- | --- | --- |
| 20k Studio overview | 8356 | 1.197 | 1.241 | 1.483 | 835.66 |
| 20k Glow overview | 6779 | 1.475 | 1.537 | 1.830 | 677.96 |
| 20k Glow Chase, DoF off | 6735 | 1.485 | 1.572 | 1.790 | 673.50 |
| 20k Glow Chase, DoF on | 6786 | 1.473 | 1.538 | 1.812 | 678.70 |
| 100k Studio overview | 8139 | 1.229 | 1.286 | 1.519 | 813.95 |
| 100k Glow overview | 6653 | 1.503 | 1.566 | 1.846 | 665.29 |
| 100k Glow Chase, DoF off | 6489 | 1.541 | 1.613 | 1.846 | 648.98 |
| 100k Glow Chase, DoF on | 6443 | 1.552 | 1.630 | 1.882 | 644.31 |
| 100k heavy Glider, Glow overview | 857 | 11.656 | 11.657 | 11.997 | 85.79 |

Glow adds 0.278 ms at 20k and 0.274 ms at 100k. The DoF differences, -0.012/+0.011 ms,
are too small to distinguish from app/driver/scatter variation and CPU/GPU overlap. No claim
of zero standalone GPU cost follows from these full-frame numbers.

The first 20k Studio window had desktop stalls (mean 4.788 ms, p50 1.322, p95 16.860). The
viewport also changed from 1662 to 1667 pixels wide during the first 20k group. The entire
20k group was repeated with a stable viewport, using complete ten-second windows and no
individual-frame filtering. Original logs remain as `*-20000.first.log`; all initial values
remain in `measurement-runs.log`. The 100k group and the single heavy run were not repeated.

Successful command sequence, with the runner doing inventories/uptime before every app:

- `python3 /tmp/glow-dof/run_cases.py showcase variants`
- `python3 /tmp/glow-dof/run_cases.py studio-20000 glow-20000 glow-chase-dof-off-20000 glow-chase-dof-on-20000 studio-100000 glow-100000 glow-chase-dof-off-100000 glow-chase-dof-on-100000 heavy-glow-100000`
- `python3 /tmp/glow-dof/run_cases.py studio-20000 glow-20000 glow-chase-dof-off-20000 glow-chase-dof-on-20000`

Screenshots (1662×1390) in
`/tmp/claude-1000/-home-martines-Projects-autonomous-characters-3d/af1dacb4-dc47-4bb5-8e8c-02c6cd1b95e8/scratchpad/shots/glow/`:

- `glow-20k-overview.png`: default Glow strengths, Overview preset.
- `glow-million-overview.png`: 1M, camera yaw 35°, pitch 25°, distance 2500 to frame the flock.
- `glow-chase-dof-slow.png`: spawn ID 13, Chase, 0.1×, Gaussian DoF.
- `glow-first-person.png`: same ID/speed, first person with DoF.
- `studio-chase-dof.png`: same tracked ID with Studio and DoF.
- `glow-imported-animated.png`: animated multi-part Bird, Model colors, Glow Chase with DoF.
- `studio-overview.png`: unchanged Studio sky/floor/material palette path, before Glow is enabled.
- `glow-controls.png`: UI restored after all required UI-free captures.

All required shots have the Menu, Controls and FPS hidden, including both hero shots. The
contact sheet is `/tmp/glow-dof/contact.png`. Live, variable-step flock screenshots are visual
proof, not a pixel-identical replay; old render settings/uniforms/flags are tested separately.

## Interfaces and Dependencies

Add SceneStyle::Glow, appearance glow/streak strengths, Glow shader specialization and optional Hdr/Bloom/DepthOfField components. Reuse reserved uniform lanes. Add boid-camera DoF settings and scenario ui off/on. No dependencies. Verification logs are `/tmp/glow-dof/tests.log`, `build.log`, `clippy.log`, `smoke.log` and `gpu-tests.log`.

---

Revision notes:

- 2026-11-01: Initial plan; completed after shader validation, screenshot review, matched-window desktop measurements and final verification.
