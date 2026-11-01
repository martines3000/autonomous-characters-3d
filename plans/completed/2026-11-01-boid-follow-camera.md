# Follow a boid with an optional chase or first-person camera

This ExecPlan is a living document maintained according to `PLANS.md`.

## Purpose

Ride one stable spawn ID with B (Off/Chase/First person), N (random pick), scroll distance,
and scenario commands. Simulation state and dispatched simulation work stay unchanged when off.

## Progress

- [x] (2026-11-01 01:37Z) Read repository contracts and camera/GPU/identity paths; plan drafted.
- [x] (2026-11-01 01:47Z) Implement optional tracking, camera, controls and scenarios.
- [x] (2026-11-01 02:00Z) Verification complete: fmt; 130 release tests; warning-free release build; Clippy at seven baseline warnings; 10-second native smoke; 14 ignored GPU tests; whitespace and final process checks.
- [x] (2026-11-01 02:00Z) Two headless timing pairs, final 20k desktop off/chase timings, all eight screenshot cases, README and Completed status recorded.

## Surprises & Discoveries

- Observation: `follow on|off` already controls target seeking in scenarios.
  Evidence: `src/scenario.rs` has `Command::Follow(bool)`.
- Observation: an initial no-sample timeout could override an explicit ID requested after startup.
  Evidence: the first desktop run selected ID 1192 instead of scripted ID 13. Timeout now applies
  only to an existing request and never supersedes a new selection; a CPU lifecycle test covers it.
- Observation: the checked-out HEAD advanced externally from `e7ee5e9` to `5028bb5` during work.
  Evidence: the latter adds only the orchestrator's research document; this task never edited it
  or performed any Git mutation.
- Observation: Bevy's pan/orbit controller derives its near plane from anchor depth, with no
  finite upper limit by default. Restoring the world-view near plane during chase clipped out
  the selected boid; the Octree screenshot showed only the distant floor.
  Evidence: `bevy_camera_controller` `projections::update_perspective` uses depth × 0.05. Both
  follow modes now use `0.05 * vehicle_size`; a regression begins with near = 100 and checks
  chase clipping and exit restoration. All screenshots and desktop timing are rerun after this fix.
- Observation: mode changes need a fresh blend and spring origin.
  Evidence: retaining the old chase spring position after first-person travel would jump backward;
  mode transitions now initialize from the current camera transform.

## Decision Log

- Decision: Boid camera scenario commands use `boidcam chase|first|off|next|id n`; `follow on|off` keeps meaning target seeking only.
  Rationale: Orchestrator review: overloading `follow` made `follow off` do two things and clashed with the target-follow label in the UI.
  Date: 2026-11-01.
- Decision: use real-time spring integration and rotation, simulated-time heading and prediction.
  Rationale: the chase camera must continue moving around a paused boid for bullet time. The
  closed-form spring retains frame-rate invariance for a fixed target; simulation is untouched.
  Date: 2026-11-01.
- Decision: clear the 80-byte sample before active tracking and dispatch tracking while paused.
  Rationale: validity zero detects missing IDs even while paused, allowing immediate recovery.
  Reads use the current order map at zero delta and the stepped map after steering.
  Date: 2026-11-01.
- Decision: override near in Chase as well as First person, then restore it on exit.
  Rationale: the inherited world-view near plane can be larger than the entire chase distance.
  The existing presets fit their view using the current FOV; they do not set a separate FOV,
  so save/restore the actual projection rather than hard-coding preset values.
  Date: 2026-11-01.
- Decision: scale chase distance/height by world scale and physical size; scale the first-person
  eye offset only by physical size.
  Rationale: instance body dimensions do not grow with vehicle count. Multiplying the eye offset
  by world scale at 1M would put the camera many body lengths in front of its boid.
  Date: 2026-11-01.
- Decision: omit the optional trail; skip automatic and explicit world framing while following.
  Rationale: keep the view focused on one boid and avoid conflicting camera writers. Presets and
  manual drags provide explicit exits.
  Date: 2026-11-01.

## Assumptions and Open Questions

- Assumption: asynchronous 80-byte readback is adequate for camera prediction.
  Why reasonable: only camera presentation consumes it.
  Validation: ignored GPU identity test and desktop slow-motion screenshots.
  If wrong: record latency and adapt camera smoothing, never simulation.

## Outcomes & Retrospective

Implemented Off/Chase/First person, stable-ID selection, random picks, direct CPU sampling,
optional GPU tracking/readback, prediction, heading smoothing, exact spring integration,
projection restoration, real-time paused orbit, keyboard/UI/scenario controls and documentation.
The render-only feature does not alter steering or simulation defaults. Off adds no tracking
kernel, clear, copy or readback entity. The arena has 80 additional reserved bytes and the
uniform is 448 bytes (previous fields retain their offsets); Boids/instances remain 64/48 bytes.

The close-camera near plane was the important visual discovery: headless tracking correctness
alone did not reveal that the old world-view plane removed nearby vehicles. Both modes now
use a body-scaled near plane before `CameraUpdateSystems` computes the projection. A fresh
blend also prevents jumps when switching between first person and chase.

Remaining limitations: no optional trail; latency distribution is not instrumented; extreme
render-only proportions or unusual custom-model extents can enter the first-person view.
Other nearby boids intentionally remain visible. Desktop captures are 881×1119 because the
window manager tiles the app; performance numbers apply to that resolution and this GPU.

## Context and Orientation

`presentation.rs` owns the simulated clock and camera transitions; `follow.rs` owns tracking
and the moving camera. `gpu_flock.rs` dispatches WGSL passes and maintains metadata order maps.
`vehicle.rs` holds CPU spawn IDs and velocities. `scenario.rs` validates scripts before startup.

## Plan of Work

Add isolated GPU track pass and tail, Bevy readback lifecycle, CPU lookup and camera dynamics;
connect presentation input/UI/scenarios, then test, measure and document.

## Concrete Steps

From the repository root, sequentially:

Before each Cargo/app/benchmark command: `pgrep -a cargo; pgrep -a rustc; pgrep -a autonomous`.
Run the following sequentially, never overlapping another Cargo command or app:

1. `CARGO_BUILD_JOBS=4 cargo fmt --all -- --check`
2. `CARGO_BUILD_JOBS=4 cargo test --release --locked`
3. `CARGO_BUILD_JOBS=4 cargo build --release --locked`
4. `CARGO_BUILD_JOBS=4 cargo clippy --release --locked --all-targets`
5. Benchmarks and scenarios as specified in Artifacts and Notes; each scenario ends in `exit`.
6. `python3 scripts/smoke-native.py`
7. `CARGO_BUILD_JOBS=4 cargo test --release --locked gpu_ -- --ignored --nocapture --test-threads=1 --skip gpu_flocking_benchmark`
8. `git diff --check`, then repeat the process inventory to prove the app is closed.
Use `CARGO_BUILD_JOBS=4` throughout; log exact invocations and results below.

## Validation and Acceptance

CPU spring convergence and frame-rate agreement; pause prediction; B cycle/exits; strict scenarios.
GPU tracking matches full readback through sorts/count edits; existing GPU parity passes.
Required screenshot cases and 20k off/chase frame measurements; 100k/1M kernel measurements.

## Idempotence and Recovery

Tests and scripts may be rerun sequentially. Every script ends in exit. No commits, resets or
discarding work; leave the orchestrator research file untouched.

## Artifacts and Notes

Raw logs and scripts: `/tmp/boid-follow/`.

Headless release measurements: Ryzen 9 7900X, Radeon RX 7900 XT, Vulkan/RADV Mesa 26.2.5,
Linux 7.2.8. Shader PCG spawn starts at frame 0; no other builds/tests/apps overlap. Wander,
cap 0, groups/obstacles/formations/predators off, dt 1/60 s, 600 warmup and 60 timed steps.
Each sample includes encoding, submission and device wait; no CPU readback is timed. Track On
matches ID 13 and adds the kernel plus tail clear. Commands set `GPU_BENCH_COUNTS=100000,1000000`,
`GPU_BENCH_CAPS=0`, `GPU_BENCH_MODES=wander`, `GPU_BENCH_TRACK=0` or `1`, and run
`CARGO_BUILD_JOBS=4 cargo test --release --locked gpu_flocking_benchmark -- --ignored --nocapture --test-threads=1`.

| Count | Off median / p95 ms, run 1 | On median / p95 ms, run 1 | Off median / p95 ms, run 2 | On median / p95 ms, run 2 |
| --- | --- | --- | --- | --- |
| 100,000 | 0.640 / 0.734 | 0.650 / 0.725 | 0.670 / 0.746 | 0.652 / 0.711 |
| 1,000,000 | 4.458 / 4.932 | 4.557 / 5.227 | 4.467 / 5.199 | 4.566 / 5.180 |

One-minute `uptime` loads before off/on/off/on: 3.86, 2.57, 4.82, 3.00 (24 logical threads).
At 100k the difference is below run variance. At 1M both pairs add 0.099 ms, about 2.2%.
GPU atomic scatter order varies between runs, so resulting statistical metrics are not identical
between independently executed benchmarks; the ignored tracking test separately checks exact
state bits with controlled sparse inputs.

Desktop release measurements after the clipping fix, using `/tmp/boid-follow/measure.txt`,
`CARGO_MANIFEST_DIR="$PWD" AC3D_SCRIPT=/tmp/boid-follow/measure.txt target/release/autonomous-characters-3d`.
Uncapped AutoNoVsync, 881×1119, Studio/Aurora/Dart, 4× MSAA, 20,000 GPU wander, 1× speed,
no orbit, all optional simulation mechanics off. Each interval lasts ten real seconds after
three seconds to settle. Scripts have no fixed frame deltas or CPU seed; ID 13 is selected
explicitly for Chase. One-minute load before launch was 3.36. Off uses yaw 35°, pitch 25°,
distance 1200; Chase uses reference distance 6, scaled by world scale and physical size.

| Camera | Frames | Mean ms | p50 ms | p95 ms | FPS |
| --- | --- | --- | --- | --- | --- |
| Off | 7704 | 1.298 | 1.307 | 1.828 | 770.48 |
| Chase | 7530 | 1.328 | 1.336 | 1.849 | 752.96 |

The mean difference is 0.030 ms (2.3%). These intervals include rendering, asynchronous
readback, CPU/GPU overlap and different views; they are not a measurement of the kernel alone.

Final screenshots, all checked at 881×1119, from `/tmp/boid-follow/desktop.txt` (normal exit at
27 seconds). Directory:
`/tmp/claude-1000/-home-martines-Projects-autonomous-characters-3d/af1dacb4-dc47-4bb5-8e8c-02c6cd1b95e8/scratchpad/shots/follow/`

- `chase-1x.png`: 20k GPU, Chase, ID 13, 1×.
- `chase-slow-a.png`, `chase-slow-b.png`: same ID, 0.1×, one real second apart.
- `first-person.png`: same ID, 0.1×, first person.
- `paused-orbit-a.png`, `paused-orbit-b.png`: paused Chase with 10°/s real-time orbit.
  Camera xyz changes from (-260.0266, 40.989433, 80.987) to
  (-267.11627, 40.98942, 79.74094) between captures while simulation time is frozen.
- `octree-chase.png`: 2k CPU Octree, automatically selected living ID 68.
- `million-chase.png`: 1M GPU, ID 13, Chase at 0.1×.

Verification logs are `tests.log`, `build.log`, `clippy.log`, `smoke.log`, `gpu-tests.log` in
`/tmp/boid-follow/`. All required gates pass. Final process inventory has no cargo, rustc or autonomous app process.

## Interfaces and Dependencies

Append `GpuParams.track` and WGSL `Params.track`; add five arena records and optional dispatch.
Follow mode and tracked spawn ID live in `Presentation.follow`. `FollowRuntime` owns camera
smoothing, projection backup and readback lifecycle. `TrackBuffer` extracts the 80-byte buffer
handle; `ReadbackComplete` validates entity/revision/ID/stamp before accepting data. The shader
uses `track.w` for the tail offset assigned by encode. CPU metadata and velocity gain crate
visibility only. `GPU_BENCH_TRACK=1` adds tracking to the existing headless benchmark. No dependency,
instance-format, binding-count or simulation-semantic changes.

---

Revision notes:

- 2026-11-01: Initial plan. Completed after desktop QA exposed and verified the chase near-plane correction.
