# Presentation controls

This ExecPlan is a living document maintained according to `PLANS.md`. Scope is Stage 5 only;
the orchestrator owns review and commits.

## Purpose

Pause or change simulation speed while continuing to position the target and camera. Present any
flock size with real-time auto-orbit and five smoothly animated camera presets.

## Progress

- [x] (2026-10-31 18:50Z) Read repository guidance, simulation, rendering, target, scene and scenario code;
  inspect pinned Bevy time and camera-controller sources.
- [x] (2026-10-31 19:02Z) Implement dedicated virtual simulation clock, bounded steps and pause-safe GPU maintenance.
- [x] (2026-10-31 19:02Z) Implement presentation UI, shortcuts, camera presets, auto-orbit and scenario commands.
- [x] (2026-10-31 19:02Z) Add nine regression tests and update README; 76 release tests pass, 5 ignored.
- [x] (2026-10-31 19:08Z) Complete scripted desktop scenarios, repeated 100k GPU timing,
  startup smoke and fixed-camera pixel comparisons. Every scripted app exits normally.
- [x] (2026-10-31 19:12Z) Final format check, 76 release tests, warning-free release build,
  Clippy (7 existing warnings), startup smoke and diff check pass. Strengthened input/focus test passes.
  No Cargo/compiler/app processes remain.
- [ ] Orchestrator review and commit (owner task).

## Surprises & Discoveries

- Observation: Bevy 0.20.1 clamps virtual time after relative-speed scaling, to 250 ms by default.
  Evidence: `bevy_time/src/virt.rs::advance_with_raw_delta`; `update_virtual_time` copies this clock
  into generic `Time`. Pause/speed setters affect its next advance, not the existing frame delta.
- Observation: the built-in pan-orbit controller and LookTo extension already use `Time<Real>`.
  Evidence: pinned `bevy_camera_controller` component and extension implementations.
- Observation: GPU parameter writes through `queue.write_buffer` cannot distinguish several steps
  encoded before submission. Each step needs an encoder-ordered uniform copy.
  Evidence: the existing encode method uses one uniform buffer for every pass.
- Observation (orchestrator review): the first version split every frame into steps of at most
  1/30 s, so at 1× any frame slower than 30 FPS ran 2–8 steps instead of one. That changed today's
  default behavior and multiplied GPU work exactly when the GPU was already slow (a feedback loop at
  high counts). Jitter also gave 2 or 3 steps at a nominal 60 Hz and 4×.
  Evidence: `SimulationFrame::from_delta` divided by a fixed 1/30 s; the 100k timestamp runs at 4×
  contain `flock_2_steps` and `flock_3_steps` samples.

## Decision Log

- Decision: use a dedicated `Time<Virtual>` advanced from `Time<Real>` after input, leaving Bevy's
  global time alone. CPU substeps temporarily supply generic `Time` inside a dedicated schedule,
  restoring it afterwards, like Bevy's fixed-schedule clock substitution.
  Rationale: immediate pause commands and shortcuts, explicit simulation ownership, existing CPU
  kernel tests unchanged, no effects on other Bevy consumers. Reject global virtual pause because
  it broadens clock ownership and its already-produced delta needs separate same-frame handling.
  Date: 2026-10-31.
- Decision: default speed 1, range 0.1–4. At 1× and below, one step per frame (today's path,
  including low FPS). Above 1×, ceil(speed) equal steps, so no step exceeds the 1× step at the same
  frame rate and GPU work is at most 4× per frame, independent of FPS. Retain Bevy's 250 ms
  simulated-frame cap.
  Rationale: the default must stay unchanged at every frame rate, and slow frames must not trigger
  extra GPU work. Replaces the worker's fixed 1/30 s bound (see Surprises). Reject an
  unconditional fixed tick because it changes today's behavior; reject clamping because it erases
  requested speed at low FPS. A stall beyond the cap discards time rather than accumulating a
  catch-up backlog.
  Date: 2026-10-31.
- Decision: skip CPU steering/integration and GPU sorting/steering/frame increment at pause. Permit
  spawn-only GPU maintenance on count changes, including writing new instances; copy existing
  instances on buffer growth. Shrink truncates. Crossing CPU/GPU respawns; Naive/Octree preserves.
  Rationale: keep existing count/mode semantics and allow settings changes without thawing vehicles.
  Reject dt=0 steering because sorting, random state and frame number would still change.
  Date: 2026-10-31.
- Decision: target keyboard movement stays at 120 units per real second, even while paused/slowed.
  Rationale: a presenter can set up a target; vehicles respond only on resumed simulation steps.
  Date: 2026-10-31.
- Decision: auto-orbit resumes after manual drag/scroll release; a preset request waits for release.
  Presets use a camera-controller target pose, smoothly approached in real time. Frame world cancels
  an active preset; orbit resumes afterwards. Close-up centers on the target at request time.
  Rationale: no competing camera writers or destroyed held drags; no persistent target lock to fight
  subsequent panning. World presets reuse `frame_distance`; close-up scales from the same fit.
  Date: 2026-10-31.
- Decision: keep the preset's target pose in the Presentation controller extension, interpolate
  Transform and PanOrbitCamera anchor depth together, and leave held gestures untouched.
  Rationale: the pinned controller exposes input/current-motion state rather than yaw/radius target
  fields. The existing LookTo extension changes orientation only and resets current_motion when
  receiving a request; it does not supply the required translated target close-up or fit distance.
  Date: 2026-10-31.

## Assumptions and Open Questions

- Assumption: a high-speed step no longer than the 1× step at the same frame rate is as stable as 1× today.
  Why reasonable: steering tuning and wander already use reference-Hz scaling at this rate.
  Validation: speed/frame-rate equivalence and high-speed/low-FPS tests plus desktop checks.
  If wrong: lower the bound with documented behavior/performance evidence.
- Assumption: multiple GPU steps remain practical at 100,000 low-poly vehicles.
  Why reasonable: Stage 4 measured a low rendering floor.
  Validation: repeated in-app 1×/4× comparisons; no ignored tests or benchmarks.
  If wrong: report cost and constrain the control rather than silently dropping virtual steps.

## Outcomes & Retrospective

Implementation complete. The release suite passes 76 tests (nine new), with all five ignored
tests/benchmarks untouched. The release build has no compiler warnings. Clippy retains the seven
pre-existing warnings, with no new warnings. The strengthened input test also exercises P/O/number keys, actual held mouse buttons and an egui
text edit consuming shortcuts. Final gate results are recorded below. Both startup smoke runs pass.

The desktop scripts exercise GPU 20k, CPU Octree 20k and Naive 200, pause/resume, real-time orbit,
0.25×/2× following, every preset at 20k, overview framing at 200/100k, target repositioning during
pause, resumed following, paused count growth across GPU capacity boundaries, and a paused model
load. Fixed-camera pairs from both freshly spawned and previously moving GPU/Octree flocks have
zero changed scene pixels. Orbit pairs differ while paused, as expected.

Residual limits: the 250 ms virtual-frame cap discards excess time on stalls; variable-step wander
is statistically equivalent rather than bit-identical across frame rates. GPU state preservation
has desktop pixel evidence and dispatch/parameter tests, not new GPU readback regression tests;
the owner explicitly excluded ignored GPU tests/benchmarks. Native pointer/keyboard injection is
unavailable here, so input mapping, text focus, held gestures and orbit suspension use headless
system tests; desktop scenarios exercise the same controls through the harness. This plan remains
in planned/ for the orchestrator's review and commit.

## Context and Orientation

`src/presentation.rs` owns controls, clock and camera target state. `src/vehicle.rs` owns a nested
CPU schedule; `src/gpu_flock.rs` extracts the step count and encodes each GPU step. GPU spawn writes
instances independently of integration. `src/scene.rs` owns framing; `src/scenario.rs` owns scripted
real-time commands. Appearance and physical simulation defaults remain unchanged.

## Plan of Work

Add the clock and step subdivision, then wire CPU/GPU pause and maintenance. Add UI, keyboard,
real-time camera animation and typed harness commands. Verify state invariants and speed equivalence,
then run release gates and scripted desktop checks. Record evidence and leave all changes uncommitted.

## Concrete Steps

Before every Cargo command and app run: `pgrep -a cargo; pgrep -a rustc; pgrep -a autonomous`.
Wait for competing processes. Add `$HOME/.cargo/bin` to PATH. Run sequentially:

1. `CARGO_BUILD_JOBS=4 cargo fmt --all -- --check`
2. `CARGO_BUILD_JOBS=4 cargo test --release --locked`
3. `CARGO_BUILD_JOBS=4 cargo build --release --locked`
4. `python3 scripts/smoke-native.py`
5. `git diff --check`
6. `CARGO_BUILD_JOBS=4 cargo clippy --release --locked --all-targets`
7. `CARGO_MANIFEST_DIR="$PWD" AC3D_SCRIPT=<script> ./target/release/autonomous-characters-3d`

## Validation and Acceptance

Existing non-ignored tests pass; pause preserves CPU state bits and skips GPU steering; speed
scales simulated duration consistently. All modes support settings changes while paused. Scripted
screenshots show frozen GPU/Octree pairs, moving real-time camera and target, resumed following,
slow/fast following, every preset at 20k, and scaled framing at 200 and 100k. Compare fixed-camera
paused pixels outside UI. Measure GPU substep cost twice at 100k. No ignored GPU tests/benchmarks.

## Idempotence and Recovery

Scenario files and evidence live outside the repository; every app script ends with `exit`. Repeating
checks is safe after checking processes. The worker will not commit, push, merge, reset or discard.
The orchestrator can review and selectively revise the uncommitted diff.

## Artifacts and Notes

Evidence root: `/tmp/claude-1000/-home-martines-Projects-autonomous-characters-3d/af1dacb4-dc47-4bb5-8e8c-02c6cd1b95e8/scratchpad/shots/stage5/`.

- `desktop.ac3d`: 56-second full scenario, ending with exit; 23 screenshots.
- `pause-live.ac3d`: freeze already-moving 20k GPU/Octree flocks; eight further screenshots.
- Fixed-camera pairs: `gpu-paused-{a,b}.png`, `octree-paused-{a,b}.png`,
  `gpu-live-paused-{a,b}.png`, `octree-live-paused-{a,b}.png`: all zero changed scene pixels.
- Corresponding `*-orbit-{a,b}.png` pairs show real-time camera rotation during pause.
- `following-quarter-speed.png`, `following-double-speed.png`, `target-paused-{before,after}.png`,
  `target-resumed-response.png`, `preset-20k-{front,side,top,overview,closeup}.png`,
  `preset-200-overview.png`, `preset-100k-overview.png`, `paused-grown-100k-model.png`,
  `naive-{paused,resumed}.png`.
- `pixel-comparison.txt`: ImageMagick AE comparisons of scene region `936x1019+420+100` in
  1776×1119 screenshots, excluding Menu, FPS and Controls. Both initial and live paused pairs are
  exactly equal; orbit pairs differ.

Repeated in-app compute timings (release, 100,000 Dart vehicles, wander, 4× MSAA, 1776×1119,
Vulkan on AMD Radeon RX 7900 XT / RADV Mesa 26.2.5, Ryzen 9 7900X). No other Cargo/app process
ran; load averages before the runs were 0.67/0.81/1.02 and 0.71/0.77/0.98. Scripts use the existing
PCG spawn at frame 0, without a seeded/replayed sort or real-time step sequence. Three seconds of
warmup precede the 1× sample; two real seconds after changing to 4× precede that sample. The
window remains vsynced at about 60 FPS; GPU timestamp spans isolate compute including uniform
copies, rather than presenting the vsync interval as compute cost. Diagnostic history retains at
most 120 samples per path. The two speeds observe different evolved flock states; these numbers
are practical in-app costs, not a paired deterministic microbenchmark.

| Steps per rendered frame | Run 1 mean / p95 ms | Run 2 mean / p95 ms |
| --- | --- | --- |
| 1 (1× speed) | 1.1563 / 1.2606 | 1.1391 / 1.2064 |
| 2 (4× speed) | 2.6184 / 3.0456 | 2.5548 / 2.8768 |
| 3 (4× speed, frame jitter) | 3.8941 / 4.4326 | 3.8834 / 4.4895 |
| 0 (paused, timestamp overhead only) | 0.0027 / 0.0028 | 0.0027 / 0.0028 |

Final gate logs: `tests.log` (76 passed, 5 ignored), `build.log` (no compiler warnings),
`clippy.log` (seven existing warnings, repeated for the test target), `smoke.log` (startup pass),
`desktop.log`, `pause-live.log` and `verification.txt`. Final process checks are empty and HEAD
remains `73f5b0a`; no commits, pushes, merges, resets or discarded changes.

The second run uses the final paused-dispatch bypass; the first used the same compute path with
an early-returning encode call during pause. `compute-{1,2}.ac3d` and `compute-{1,2}.log` preserve
scripts and full timing records. The 4× range remains practical for this 100k low-poly scenario;
these measurements do not establish performance for dense following, heavy models or other GPUs.

## Interfaces and Dependencies

New presentation resource, dedicated virtual clock, extracted simulation frame and CPU step schedule.
Opt-in `AC3D_GPU_TIMING=1` with a script enables Bevy render diagnostics and reports compute spans
by substep count. It is off during ordinary runs; GPU timestamps require a supported backend.
No dependency or lockfile changes. New typed harness controls are documented in README.

---

Revision notes:

- 2026-10-31: Initial Stage 5 design and source evidence before implementation.
- 2026-10-31: Implemented Stage 5; recorded pause pixel proof and repeated GPU substep costs.
