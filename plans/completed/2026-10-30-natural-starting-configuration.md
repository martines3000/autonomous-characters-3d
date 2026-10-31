# More natural starting configuration

This ExecPlan is a living document maintained according to `PLANS.md`.

## Purpose

At today's defaults the app opens with the camera about 7 units from the target, 100 vehicles scattered over a 2000 × 2000 × 1000 box, and headings drawn from the positive octant. No vehicle has a neighbor within the flocking radii, so flocking never engages, and the few vehicles that pass the camera already point the same way. After this work, the app opens on a visible group that starts disordered and visibly aligns into a flock within seconds, and that stays near the origin instead of drifting out of view.

## Progress

- [x] (2026-10-30 13:45Z) Plan drafted from the measured recommendations in `plans/completed/2026-10-30-numerical-safety.md`.
- [x] (2026-10-30 14:00Z) Milestone 1: wall bounds measured; 200 × 200 × 100 half-extents chosen (see Artifacts and Notes).
- [x] (2026-10-30 14:10Z) Milestone 2: `SPAWN_SIZE` 100, `WORLD_SIZE` 200 × 200 × 100, `random_direction` headings, 200 vehicles, camera eye `(0, 60, 170)`.
- [x] (2026-10-30 14:15Z) Milestone 3: `vehicles_spawn_in_the_spawn_cube_with_varied_headings` added (fails on the old spawner: positions up to ±1000, positive-only headings); gate passed.
- [x] (2026-10-30 14:20Z) Milestone 4: deferred. The app ran 15 s without errors, but window capture failed (`could not create image from window`; this process lacks macOS screen-recording permission), so framing is not visually confirmed. Tracked in `plans/STATUS.md` as a manual check.
- [x] (2026-10-30 14:25Z) Plan closed and moved to `plans/completed/`.

## Surprises & Discoveries

- Observation: with the current 2000 × 2000 × 1000 walls, even a dense flock leaves the camera's neighborhood within 15 s: 0–0.5% of vehicles are within 150 units of the origin at frame 900.
  Evidence: probe scenario W1.
- Observation: walls are soft. With 200 × 200 × 100 half-extents up to 16% of vehicles are briefly outside a wall, by at most 6 units, because wall avoidance is a limited steering force, not a hard boundary.
  Evidence: probe scenario W3.
- Observation: very tight walls (100 × 100 × 50) compress the flock into a dense ball: 100–164 neighbors within 30 units and minimum pair distance 0.97, below the 2-unit cone size.
  Evidence: probe scenario W4.

## Decision Log

- Decision: separate the spawn region from the wall bounds.
  Rationale: `WORLD_SIZE` currently means half-extent for walls and the octree root but full extent for spawning, which is confusing; the measured spawn density (100-unit cube) and the wall distance are independent choices.
  Date: 2026-10-30.
- Decision: keep the walls approach (shrink `WORLD_SIZE`) rather than a camera that follows the flock.
  Rationale: one constant versus a new camera system; the walls already exist and work per axis. Camera follow can be added later if bounded walls look artificial.
  Date: 2026-10-30.
- Decision: wall half-extents 200 × 200 × 100.
  Rationale: the tightest measured bound that keeps the flock near the origin (centroid 66–85 units away after 30 s) without crowding (minimum pair distance 2.0–2.9). 400 × 400 × 200 still lets the flock drift away within 30 s; 100 × 100 × 50 crowds it.
  Date: 2026-10-30.
- Decision: camera eye `(0, 60, 170)`, about 180 units from the target.
  Rationale: frames the 100-unit spawn cube at roughly two-thirds of the window height, with the 2-unit cones about 10 pixels tall at 720p. Provisional until visually checked.
  Date: 2026-10-30.
- Decision: leave speeds, mass, force factors and naive mode unchanged.
  Rationale: forces are still applied per frame; speed tuning belongs after the frame-rate task. Naive mode is cheap at a few hundred vehicles and exact, while octree queries are not yet exact.
  Date: 2026-10-30.

## Assumptions and Open Questions

- Assumption: 200 vehicles stay well within frame budget in naive mode.
  Why reasonable: 40,000 pair checks per frame; historical naive timing at 1,000 vehicles was about 4 ms.
  Validation: startup smoke and FPS overlay during the visual check.
  If wrong: lower to 150 with a proportionally smaller spawn cube.
- Open question: which wall distance keeps the flock in view without visibly bouncing off invisible walls. Answered by Milestone 1.

## Outcomes & Retrospective

Delivered: the app now spawns 200 vehicles in a 100-unit cube around the target with headings uniform on the sphere, walls at ±200 / ±200 / ±100 and a camera about 180 units out. In the headless probe this setup starts disordered (polarization 0.05–0.07), flocks (0.26–0.57 at 5 s, 0.38–0.76 at 30 s) with 98–100% of vehicles near a neighbor, keeps the flock's centroid within 85 units of the origin over 30 s, and seeking still converges (mean distance to a target 150 units away falls to 7 within 5 s). No non-finite values in any run.

Remaining: visual confirmation of camera framing (needs a person or screen-recording permission). Speed defaults are unchanged and should be retuned after the frame-rate task. Camera follow was not needed.

Lessons: the walls, not just spawn density, decided whether the flock stays visible; measuring 30 s rather than 10 s exposed the drift.

## Context and Orientation

- `src/main.rs`: `configure_global_state` sets `vehicle_count` (100); `setup_scene` places the camera at `(-2, 5, 5)` looking at the origin, with a `PanOrbitCamera` anchored at that distance.
- `src/vehicle.rs`: `WORLD_SIZE` (2000 × 2000 × 1000) is used by `vehicle_spawner` (positions uniform in ±`WORLD_SIZE / 2`), by wall avoidance in `movement` (pushes back when a coordinate leaves ±`WORLD_SIZE`, wander mode only), and as the octree root bounds in `flock`. Initial velocity is a positive-octant random vector scaled to max speed.
- `src/target.rs`: the target spawns at the origin.

## Plan of Work

1. Probe wall half-extents 2000/400/200/100 with 200 vehicles in a 100-unit cube and sphere headings (Milestone 1).
2. Add `SPAWN_SIZE` (100-unit cube, full extent) for spawn positions; set `WORLD_SIZE` to the measured wall half-extents; draw headings uniformly on the unit sphere; default `vehicle_count` 200; move the camera so the spawn region is framed.
3. Spawner test: positions inside the spawn cube, speeds equal max speed, headings not confined to one octant.
4. Gate, visual check, close.

## Concrete Steps

From the repository root, sequentially, with no other Cargo process running:

1. `CARGO_BUILD_JOBS=4 cargo fmt --all -- --check`
2. `CARGO_BUILD_JOBS=4 cargo test --release --locked`
3. `CARGO_BUILD_JOBS=4 cargo build --release --locked`
4. `python3 scripts/smoke-native.py`
5. `git diff --check`

## Validation and Acceptance

- Probe with the new defaults: most vehicles have a neighbor within 20 units, polarization rises from near 0 to well above 0.5 within 10 s, and nearly all vehicles stay within the walls over 30 s.
- Spawner test passes; all existing tests pass.
- The app opens with a visible group of vehicles around the target.

## Idempotence and Recovery

Changes are constants and the spawner in `src/main.rs` and `src/vehicle.rs`. Revert the commit to restore the old defaults.

## Artifacts and Notes

Wall-bounds probe (2026-10-30): same disposable workspace and method as the numerical-safety probe, with wall half-extents made a probe parameter. All scenarios: 200 vehicles uniform in a 100-unit cube, sphere headings, wander mode, naive mode, other settings at defaults, 1800 frames at 1/60 s, seeds 1–3 (min–max shown; wander randomness unseeded). The cold build was reused; the probe ran in 2 s. Non-finite values: 0 in every row.

| Walls (half-extents) | Frame | Polarization | % within 150 of origin | % outside walls | Max overshoot | Centroid from origin | Min pair distance |
| --- | --- | --- | --- | --- | --- | --- | --- |
| W1: 2000 × 2000 × 1000 | 900 | 0.60–0.78 | 0–0.5 | 0 | 0 | 220–310 | 2.2–3.6 |
| W1 | 1800 | 0.42–0.56 | 0–0.5 | 0 | 0 | 413–529 | 2.5–3.5 |
| W2: 400 × 400 × 200 | 1800 | 0.39–0.57 | 0 | 2.5–16.5 | 2.2–4.7 | 100–320 | 1.4–3.0 |
| W3: 200 × 200 × 100 (chosen) | 300 | 0.26–0.57 | 58–95 | 0.5–15.5 | 0.5–5.0 | 59–83 | 1.9–2.4 |
| W3 | 900 | 0.51–0.58 | 3.5–57.5 | 0.5–16.0 | 0.0–5.6 | 84–117 | 1.8–3.1 |
| W3 | 1800 | 0.38–0.76 | 14.5–32.5 | 2.5–7.0 | 1.2–5.9 | 66–85 | 2.0–2.9 |
| W4: 100 × 100 × 50 | 1800 | 0.83–0.93 | 100 | 0–4.0 | 0–0.5 | 31–87 | 0.97–1.8 |

Seek check with W3 walls, target fixed at (150, 0, 0): mean distance to target 153–157 at frame 0, 7.0–9.4 at 300, 6.8 at 600.

Gate (2026-10-30, sequential, `CARGO_BUILD_JOBS=4`): `cargo fmt --all -- --check` clean; `cargo test --release --locked` 13 passed; `cargo build --release --locked` with the 2 known warnings; `python3 scripts/smoke-native.py` PASS; `git diff --check` clean.

## Interfaces and Dependencies

No new crates or resources. New constant `SPAWN_SIZE` in `src/vehicle.rs`.

---

Revision notes:

- 2026-10-30: Initial plan.
- 2026-10-30: Recorded probe evidence, implementation and gate; deferred the visual check; closed the plan.
