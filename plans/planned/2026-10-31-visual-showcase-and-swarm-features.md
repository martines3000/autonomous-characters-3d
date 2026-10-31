# Visual showcase, custom models and swarm mechanics (umbrella)

This ExecPlan is a living document maintained according to `PLANS.md`. It coordinates seven
stages on branch `feat/visual-showcase-and-swarm-features`. Detailed stage plans are linked below and
carry their own evidence; this file records the stage checkpoints and the decisions that cross stages.

## Purpose

Turn the flock into something that is pleasant to look at and to present, and that can show more
than one behavior, without losing the GPU path or the CPU reference modes. When done, a user can:

- pick a visual style (the new default or the original look), palettes, vehicle shapes, sizes and
  coloring, and load their own static or animated glTF models while the flock runs;
- pause, slow down or speed up the simulation, auto-orbit the camera and jump to camera presets that
  frame any flock size;
- enable optional mechanics: a mouse attractor/repulsor, several swarm groups, obstacles, formations
  and predators. All are off in the baseline configuration.

The owner authorized all seven stages on 2026-10-31, in this order, with the roadmap in
`review-report.html` section 12 as background.

## Progress

- [x] (2026-10-31 10:45Z) Branch created from `origin/master` at `57d2eed`; working tree clean.
- [x] (2026-10-31 10:50Z) Umbrella plan drafted.
- [x] (2026-10-31 12:00Z) Stage 1: visual foundation, commit `e635c6d`.
- [x] (2026-10-31 12:50Z) Stage 2: vehicle appearance, commit `f2e3eb0`; ignored GPU regressions pass (3/3).
- [x] (2026-10-31 13:40Z) Review fixes for stages 1–2: owner approved findings 1 (held drag stops after auto-framing) and 2 (far target hidden by sky/culled); both fixed with regression tests. Finding 3 (per-frame `Appearance` change marking) deferred by the owner: cost only.
- [x] (2026-10-31 19:10Z) Stage 3: custom static 3D models (see the custom-vehicle-models plan). Orchestrator review: zero-area triangles are now skipped instead of rejecting the model; axis caption glyphs replaced.
- [x] (2026-10-31 19:55Z) Stage 4: rendering optimization (see the rendering-optimization plan). Import now shares bit-identical vertices through an index buffer: the 3,456-triangle stress model drops from 10,368 to 2,119 GPU vertices and from about 24 to 11.5 ms at 100k. Low-poly shapes sit at a ~1 ms floor; no LOD, impostors, culling or MSAA change was justified. Orchestrator review: accepted; the close-up (distance 100 at 1280×720) does not fill the screen, so fragment/overdraw cost at dense close views stays unmeasured.
- [x] (2026-10-31 20:20Z) Stage 5: presentation controls (see the presentation-controls plan). Orchestrator review: the worker split every frame into steps of at most 1/30 s, which changed 1× behavior below 30 FPS and added GPU work as FPS fell; replaced by ceil(speed) steps above 1× only. The GPU parity test used dt = 0, which now means paused; it now uses a negligible step. Ignored GPU tests pass (3/3); paused flock pixels identical.
- [x] (2026-10-31 21:30Z) Stage 6: animated and multipart models (see the custom-vehicle-models plan). Rigid TRS and skinned clips are baked at 30 Hz into a storage buffer and played in the vertex shader. Per-vehicle phase lives in `separation.w` on the GPU, so it survives sorting; the instance stride grows from 32 to 48 bytes. Orchestrator review: static meshes once again skip the per-frame appearance bind-group rebuild. Ignored GPU tests pass (4/4).
- [x] (2026-10-31 22:40Z) Stage 7a: mouse attractor/repulsor, plus the shared Stage 7 contract (see the swarm-mechanics plan). Hold F; cursor ray meets a camera-facing plane through the orbit focus; smooth falloff, radius × world scale; identical CPU/GPU arithmetic. Dense attraction at 100k costs about 12.6 ms (neighbor density), against 1 ms off. Orchestrator review: cursor input wrote `GlobalState` every frame, marking it changed and rebuilding the appearance uniform; it now writes only on change. Ignored GPU tests pass (5/5).
- [x] (2026-10-31 23:00Z) Stage 7b: multiple swarm groups (see the swarm-mechanics plan). 2–8 groups; separation spans all groups, alignment and cohesion stay within a group; optional per-group targets, and the selected target overrides them while following; a Group color mode. Identity (spawn ID, group, slot) travels in a metadata arena on binding 8, and sorts carry only 32-bit table indices. Orchestrator review: the first version grew the GPU Boid to 80 bytes and made the groups-off step 24–32% slower; it now keeps the 64-byte Boid and a groups-off steer specialization, so off costs +1–5%. Known limit: raising the group count does not rebalance existing members. Ignored GPU tests pass (7/7).
- [ ] Stage 7c: obstacles with avoidance.
- [ ] Stage 7d: formations.
- [ ] Stage 7e: predators with fleeing.
- [ ] Independent review, full gate, GPU regressions, Clippy, repeated performance runs, desktop checks.

## Surprises & Discoveries

- Observation: at reduced vehicle counts the startup camera and fog hide the flock.
  Evidence: the camera starts at `(0, 140, 450) * world_scale(20_000)`, about 2,200 units out, and
  fog visibility is `5 × walls.x`. At 200 vehicles the walls shrink to 200, so fog visibility is
  1,000 units, less than the camera distance (`src/main.rs::setup_scene`, `draw_world`).
- Observation: GPU boids have no stable identity; `steer` writes them back in sorted order.
  Evidence: `assets/shaders/gpu_flock.wgsl` header comment and `steer`. Group and formation data must
  travel with the boid record.

- Observation: review of stage 1 found two defects, not yet fixed.
  Evidence: (1) `scene.rs::frame_world` resets `PanOrbitCamera::current_motion` when auto-framing
  fires, so a right/middle drag that starts within 0.35 s of a count change stops until the button
  is pressed again (`main.rs::camera_input` only restarts motion when its mode changes). Score 3 × 3.
  (2) Sky radius, fog range and far plane use only the wall corners; a target moved far outside the
  walls (no clamp in `target.rs::input_controls`) is hidden by the opaque Studio sky or culled.
  Score 4 × 4.
- Observation: `ui` writes `Appearance::count_editing` every frame, so `Appearance` is always
  marked changed and `draw_world`/`update_appearance` reapply lights and the uniform each frame.
  Evidence: `src/main.rs` (`appearance.count_editing = false`). Cost only; no visible defect. Deferred.

## Decision Log

- Decision: work in the main checkout on a new branch; no worktree.
  Rationale: the tree was clean, and stages touch the same files in a fixed order, so parallel
  worktrees would only add merge cost.
  Date: 2026-10-31.
- Decision: visual size is a render-only setting that multiplies the instance scale.
  `GlobalState::vehicle_size` keeps scaling the neighbor radii as before.
  Rationale: the owner requires that visual geometry not change separation semantics. The roadmap
  (section 12.1) already flags `vehicle_size` as physical.
  Date: 2026-10-31.
- Decision: add a small developer scenario harness (environment-driven script of timed settings,
  camera moves, screenshots and frame-time logging) early, in stage 1.
  Rationale: egui controls cannot be driven from outside the app on this Wayland desktop (no pointer
  injection tool). Stage 4 needs repeatable rendering measurements with fixed settings, and every
  stage needs desktop checks with screenshots.
  Date: 2026-10-31.
- Decision: the simulation reads a simulation clock, while the camera, the UI and target input keep
  real time. Pausing skips GPU stepping instead of stepping with dt = 0.
  Rationale: the roadmap notes that a zero step still re-sorts and advances the GPU frame counter.
  Moving the target while paused lets a presenter set up a shot. Stage 5 records the details.
  Date: 2026-10-31.
- Decision: the new mechanics are optional and off by default. They share one behavior contract
  between `flocking_force`/`movement` on the CPU and `steer` on the GPU, with parity tests where
  practical.
  Rationale: owner requirement. This also matches the existing CPU/GPU parity test pattern.
  Date: 2026-10-31.

## Assumptions and Open Questions

- Assumption: Bevy 0.20.1's enabled `3d` feature and the `gltf` crate already in `Cargo.lock` are
  enough for model import; no new crates.
  Why reasonable: `bevy_gltf` and `gltf 1.4.1` are in the lockfile.
  Validation: the stage 3 plan.
  If wrong: justify any dependency change in the stage 3 plan before editing `Cargo.toml`.
- Assumption: example models can be generated by a script in this repository, so they are original
  work under the repository's terms.
  Why reasonable: it avoids third-party license questions.
  Validation: the stage 3 and stage 6 plans name each asset's origin and license.

## Outcomes & Retrospective

Filled at stage checkpoints and at completion.

## Context and Orientation

- `src/main.rs`: app setup, `GlobalState` defaults, the egui menu, camera input, scene setup
  (`setup_scene`, `draw_world`).
- `src/vehicle.rs`: CPU steering (`movement`, `flock`, `flocking_force`, `update`), spawning, instance
  sync, CPU and GPU tests and benchmarks.
- `src/gpu_flock.rs` and `assets/shaders/gpu_flock.wgsl`: GPU simulation.
- `src/instancing.rs` and `assets/shaders/vehicle_instancing.wesl`: one instanced draw for the flock.
- `src/target.rs`: the selectable, movable target.
- `review-report.html` section 12: feature roadmap with seams, budgets and risks (read-only record).

## Plan of Work

Stages land in the owner's order. Each stage gets a detailed plan in `plans/planned/` when it spans
modules or needs measurement. Each ends with focused tests, a desktop check with screenshots and a
commit. The final gate runs once at the end.

Stage plans:

- Stages 1–2: [visual foundation and vehicle appearance](2026-10-31-visual-foundation-and-appearance.md).
- Stages 3 and 6: [custom glTF vehicle models](2026-10-31-custom-vehicle-models.md).
- Stage 4: [rendering optimization](2026-10-31-rendering-optimization.md).
- Stage 5: [presentation controls](2026-10-31-presentation-controls.md).
- Stage 7: [swarm mechanics](2026-10-31-swarm-mechanics.md).

## Concrete Steps

From the repository root, sequentially, never overlapping another Cargo command or app instance:

1. `CARGO_BUILD_JOBS=4 cargo fmt --all -- --check`
2. `CARGO_BUILD_JOBS=4 cargo test --release --locked`
3. `CARGO_BUILD_JOBS=4 cargo build --release --locked`
4. `python3 scripts/smoke-native.py`
5. `git diff --check`
6. `CARGO_BUILD_JOBS=4 cargo clippy --release --locked --all-targets` (7 known warnings, add none)
7. `CARGO_BUILD_JOBS=4 cargo test --release --locked gpu_ -- --ignored --nocapture --test-threads=1 --skip gpu_flocking_benchmark`

## Validation and Acceptance

Each stage's acceptance is in the owner's brief and repeated in the stage plans. Overall:

- Default settings still produce today's simulation (same `GlobalState` simulation defaults; the
  existing tests keep passing). The new mechanics are off.
- Every stage has been exercised in the running app, with screenshots.
- Rendering results at 20,000 and 100,000 vehicles are recorded twice on an idle machine.

## Idempotence and Recovery

All work is on one branch with stage commits; reverting a stage commit rolls it back. Nothing is
pushed or merged.

## Artifacts and Notes

Large logs and screenshots stay outside the repository (paths in the stage plans). Final screenshots
for review go to `docs/images/` only if they are small and referenced by `README.md`.

## Interfaces and Dependencies

Listed per stage in the stage plans.

---

Revision notes:

- 2026-10-31: Initial umbrella plan.
- 2026-10-31: Stages 1–2 committed and pushed; paused before review fixes and stage 3 at the owner's request.
- 2026-10-31: Review findings 1 and 2 fixed; paused before stage 3.
