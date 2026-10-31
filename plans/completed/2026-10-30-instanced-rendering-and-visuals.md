# Instanced vehicle rendering and a visual pass

This ExecPlan is a living document maintained according to `PLANS.md`.

## Purpose

Large flocks should render cheaply and read clearly. Before this plan each vehicle was a `cone.glb` scene instance: a root entity plus a child mesh entity, all grey, lit only by ambient light on a grey background. At 30,000 vehicles the render world spent about 9.4 ms per frame collecting 30,000 moving meshes (see `completed/2026-10-30-high-count-frame-limit.md`).

When this plan is done:

- The whole flock is drawn by one entity with one per-instance GPU buffer filled from the simulation each frame. Vehicles are plain simulation entities with no mesh or child entities.
- Vehicles are colored by heading, so groups flying together share a color.
- The scene has a directional light, a dark background, distance fog that scales with the world, and a faint outline of the walls with a floor grid.
- 30,000 vehicles run faster than the previous 18.8 ms median frame time (same measurement harness and machine).

## Progress

- [x] (2026-10-30 15:45Z) Plan drafted.
- [x] (2026-10-30 16:05Z) Milestone 1: custom instanced rendering (`src/instancing.rs`, `assets/shaders/vehicle_instancing.wesl`); vehicles have no scene entities; `cone.glb` removed.
- [x] (2026-10-30 16:15Z) Milestone 1 measurement: 30,000 vehicles 18.8 → about 11.7 ms median at load 5.
- [x] (2026-10-30 16:25Z) Milestone 2: visual pass (background, directional light, heading colors, fog, wall box and floor grid, slimmer dart mesh), checked with in-app screenshots at 200, 5,000 and 30,000 vehicles and in follow mode.
- [x] (2026-10-30 17:05Z) Separation retune: investigated with three headless probes; no default changed. Follow-mode collapse deferred to its own task (`plans/STATUS.md`, future task 1).
- [x] (2026-10-30 16:40Z) Independent review (read-only Codex); retained phase item and test gap fixed, multi-camera visibility dismissed.
- [x] (2026-10-30 17:15Z) Validation gate passed; final measurement; README and STATUS updated.

## Surprises & Discoveries

- Observation: OS window capture lacks screen-recording permission, but Bevy's `Screenshot` component writes the rendered frame to a file without it.
  Evidence: screenshots under the scratch directory (see Artifacts); used for every visual check in this plan.
- Observation: the separation and cohesion "factors" are caps on each steering force, not weights. In the measured scenarios the caps never bind, so changing them leaves results unchanged to two decimals.
  Evidence: follow mode, 5,000 vehicles, (separation, cohesion) factors (1.5, 4), (3, 4), (3, 2), (4, 1.5): median nearest-neighbor distance 0.58 in all four.
- Observation: wander mode barely overlaps with the default separation; follow mode collapses whatever the settings.
  Evidence (30 s at 60 Hz, seed 7, octree): wander 5,000 vehicles, separation distance 4: 0.5% of vehicles have a neighbor closer than 1.4 units (the new dart is 1.4 wide); distance 6: 0.2%, but 50% more neighbors within 30 units (higher flocking cost). Follow 5,000: median nearest neighbor 0.23 units, every vehicle within 30 units of every other, at separation distance 4, 6 or 8.
- Observation: seek has no arrival and is capped at 7 × max speed against separation's 1.5 ×, so following vehicles drive into the target point. Adding arrival (radius scaled with cbrt(count)) or seeking only from outside a gather radius fixes 200 vehicles (96% → 12–27% overlapping) but not 1,000 or 5,000: the flock fuses into one dense clump.
  Evidence: `arrival-probe.txt`, `gather-probe.txt`. Separation averages the directions to close neighbors and normalizes the result, so inside a crowd it points in a near-random direction instead of pushing outward. Because every vehicle then has all others in its query radius, following at large counts is also quadratic in cost.
- Observation: two probe scripts silently stopped patching when `main.rs` changed the line they anchored on (the screenshot probe and the frame-time logger). Both now assert their anchors.

## Decision Log

- Decision: custom instancing (one entity, per-instance vertex buffer), following Bevy 0.20.1's `examples/shader_advanced/custom_shader_instancing.rs`.
  Rationale: the measured cost is per-entity mesh collection, which Bevy's automatic batching does not remove. One plain mesh per vehicle would halve entities but keep 30,000 mesh extractions. One entity also makes per-vehicle color free.
  Date: 2026-10-30.
- Decision: compute the color in the shader from the instance rotation (heading), not on the CPU.
  Rationale: no extra per-instance data or CPU work.
  Date: 2026-10-30.
- Decision: no new crates; `Cargo.toml` unchanged. Instance data is `[Vec4; 2]` in Bevy's `RawBufferVec`, which reuses its GPU buffer and grows only when needed.
  Date: 2026-10-30.
- Decision: vehicles do not cast or receive shadows.
  Rationale: the custom draw runs in the main pass only; a shadow-pass draw would double the plumbing for little gain in an open volume without a floor.
  Date: 2026-10-30.
- Decision: draw the opaque flock in `Transparent3d` with a sort distance of negative infinity and queue it as a transient item each frame.
  Rationale: the phase sorts ascending, so the flock draws before blended objects (the target sphere) and they blend over it. Transient items disappear with the entity (review finding).
  Date: 2026-10-30.
- Decision: replace `cone.glb` with Bevy's `Cone` primitive, radius 0.7 and height 2.5 with 8 sides.
  Rationale: same apex axis (+Y); a slimmer dart shows heading better and overlaps less; no asset loading.
  Date: 2026-10-30.
- Decision: keep all simulation defaults and the seek rule unchanged; defer the follow-mode collapse to its own plan.
  Rationale: the probes show it needs a model change (separation formulation and seek arrival), not a retune. `AGENTS.md` forbids changing simulation semantics as a side effect, and a partial fix (gather radius) would help only small flocks.
  Date: 2026-10-30.
- Decision: no in-app toggle for heading colors or the wall outline.
  Rationale: not requested; easy to add later.
  Date: 2026-10-30.

## Assumptions and Open Questions

- Assumption: Bevy's PBR shader functions can light the instanced mesh. Validated: the directional light, ambient light and fog all show on vehicles in screenshots.
- Assumption: the opaque flock in `Transparent3d` draws before the translucent target. Validated by the sort order in `bevy_core_pipeline` (`Transparent3d` sorts ascending; values grow toward the camera) and the review.
- Open: GPU execution time is still unmeasured (no in-pass timestamps on this Mac). The frame time fell by about the measured mesh-collection cost, so the GPU is not the limit at 30,000.

## Outcomes & Retrospective

Delivered:

- One instanced draw for the whole flock; vehicles are bare simulation entities. 30,000 vehicles: 18.8 → 12.1–12.9 ms median (53 → about 80 FPS); 10,000: 7.25 → 4.5 ms (138 → about 220 FPS). Same harness, Apple M3 Pro, load average 5–8 for both baseline and final.
- Heading-colored darts with directional and ambient light, a dark background, distance fog and a wall box with a floor grid; fog and outline scale with the world.
- Test `instances_follow_vehicle_transforms_and_count`: instance data equals the integrated vehicle transforms (non-identity rotations) as the count goes 200 → 3 → 0 → 5.

Not done, by decision: the separation retune. The evidence shows the visible overlap is a follow-mode collapse that tuning cannot fix; it is the first future task in `plans/STATUS.md` with the probe data.

Lessons: probe scripts that patch source by text must assert their anchors. Measure before tuning: three probes showed the sliders were the wrong lever.

Residual risk: the custom render path relies on Bevy 0.20 internals (`MeshPipeline` specialization, `RenderMeshInstances`, `NoIndirectDrawing`), so a Bevy upgrade will need this module revisited. Multiple cameras with different render layers would show the flock in all of them (review finding, not reachable with one camera).

## Context and Orientation

- `src/instancing.rs`: `VehicleInstances` component (per-vehicle `[position + scale, rotation]`), extraction, `RawBufferVec` upload, specialized `MeshPipeline` with an instance vertex buffer at shader locations 8 and 9, and `DrawMeshInstanced`.
- `assets/shaders/vehicle_instancing.wesl`: rotates, scales and translates each vertex and normal by its instance; colors by heading; lights with `apply_pbr_lighting` and `main_pass_post_lighting_processing` (fog, tonemapping).
- `src/vehicle.rs`: `spawn_flock` (Startup) spawns the flock entity with the dart mesh and `NoFrustumCulling`; `sync_instances` runs after `update` in the vehicle chain; `walls(count)` is shared with the scene outline.
- `src/main.rs`: background, lights, `DistanceFog` and `NoIndirectDrawing` on the camera; `draw_world` draws the wall box and floor grid with gizmos and scales the fog.

## Plan of Work

As executed: Milestone 1 (instanced rendering), measurement, Milestone 2 (visual pass and separation investigation), review fixes, gate, documentation.

## Concrete Steps

From the repository root, sequentially, with no other Cargo process running:

1. `CARGO_BUILD_JOBS=4 cargo fmt --all -- --check`
2. `CARGO_BUILD_JOBS=4 cargo test --release --locked` (24 passed, 1 ignored)
3. `CARGO_BUILD_JOBS=4 cargo build --release --locked` (no warnings)
4. `python3 scripts/smoke-native.py` (PASS)
5. `git diff --check`

## Validation and Acceptance

- Gate above passed on 2026-10-30. Clippy: 8 warnings, all in pre-existing code; none in touched lines.
- Screenshots at 200 (wander and follow), 5,000 and 30,000 vehicles show lit, heading-colored darts, the wall box, the floor grid and the target.
- 30,000-vehicle median frame time 12.1–12.9 ms (below 18.8 ms).

## Idempotence and Recovery

Revert the commit to return to scene-per-vehicle rendering (it restores `assets/cone.glb`). Probes and screenshots run in disposable copies with their own `CARGO_TARGET_DIR`.

## Artifacts and Notes

Scratch directory (outside the repository): `/private/tmp/claude-501/-Users-martines-Desktop-git-repos-personal-autonomous-characters-3d/6c3ee9af-73e9-4d9e-baaf-d88371ea7993/scratchpad/codex.ISlk/`.

- `measurements.txt`: every frame-time run with load averages.
- `shots/*.png`: screenshots; `shoot.sh`, `sync-shots.sh`, `apply-shot-probe.py` reproduce them.
- `arrival-probe.txt`, `gather-probe.txt`, `factor-probe.txt`, `separation-probe.txt`: steering probes.
- `review-result.md`: independent review.

Final frame times (profiling copy with `AutoNoVsync`, 5 s warm-up, 10 s window):

```
final n=10000 run=1 load=[4.91 7.20 7.40] median_ms=4.593 p95_ms=5.979
final n=10000 run=2 load=[5.25 7.16 7.38] median_ms=4.465 p95_ms=5.837
final n=30000 run=1 load=[6.60 7.36 7.45] median_ms=12.893 p95_ms=16.170
final n=30000 run=2 load=[7.22 7.46 7.48] median_ms=12.069 p95_ms=13.129
final n=30000 run=3 load=[8.34 7.70 7.56] median_ms=12.439 p95_ms=13.802
```

## Interfaces and Dependencies

- New: `src/instancing.rs` (`InstancingPlugin`, `VehicleInstances`, `InstanceData`), `assets/shaders/vehicle_instancing.wesl`.
- Changed: `vehicle_spawner` (no scene, no `Visibility`), new `spawn_flock`, `sync_instances`, `walls`; `RenderState` and `load_assets` removed; camera gets `NoIndirectDrawing` and `DistanceFog`; new `draw_world` system.
- Removed: `assets/cone.glb`.
- No crate changes.

---

Revision notes:

- 2026-10-30: Initial plan.
- 2026-10-30: Completed. The separation retune became an investigation with no default change; the follow-mode collapse moved to its own future task.
