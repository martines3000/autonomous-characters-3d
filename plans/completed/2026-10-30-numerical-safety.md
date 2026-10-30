# Numerical safety and regression tests

This ExecPlan is a living document maintained according to `PLANS.md`.

## Purpose

Vehicles can acquire NaN velocities or positions from ordinary situations: a vehicle reaching the target, two vehicles at the same position, neighbors flying in opposite directions, or a zero velocity. Once NaN enters a velocity it spreads through flocking sums to every neighbor, and the affected vehicles vanish or stop rendering. Speed is also clamped per axis, so diagonal motion reaches √3 × the configured maximum, and cohesion's slow-down near the group center never triggers because the distance is measured after normalization.

After this work, every steering and integration step produces finite values for any finite input, speed never exceeds the configured maximum, cohesion slows vehicles inside a 10-unit arrival radius, and regression tests prove these properties for both the naive and octree paths. The handoff also records recommendations for a more natural starting configuration, separating measured settings from provisional ones; applying them is future task 1 in `plans/STATUS.md`.

## Progress

- [x] (2026-10-30 12:09Z) Verified original findings F01, F05 and F06 against current `src/vehicle.rs`; recorded finding status in `plans/STATUS.md`.
- [x] (2026-10-30 12:20Z) Planning documents written: `AGENTS.md`, `PLANS.md`, `plans/TEMPLATE.md`, `plans/STATUS.md`, this plan.
- [x] (2026-10-30 12:45Z) Milestone 1: `normalize_or_zero` at every NaN site, shared `flocking_force` for both `flock` branches, magnitude speed clamp and zero-velocity rotation hold in `update`.
- [x] (2026-10-30 12:45Z) Milestone 2: eight new tests in `src/vehicle.rs`. The ECS test `degenerate_vehicles_stay_finite_and_within_max_speed` failed on the old code (non-finite translation at frame 0, naive + following) and passes after the fix.
- [x] (2026-10-30 12:55Z) Milestone 3: gate passed (see Artifacts and Notes).
- [x] (2026-10-30 13:20Z) Milestone 4: starting-configuration recommendations recorded, with a headless probe for the measured ones (see Outcomes and Artifacts). Applying them is deferred to future task 1 in `plans/STATUS.md`.
- [x] (2026-10-30 13:25Z) Clippy: 14 warnings (was 17); none in changed lines. Plan closed and moved to `plans/completed/`.

## Surprises & Discoveries

- Observation: the cohesion code's `try_normalize(f32::MIN)` never returns `None`, because `f32::MIN` is the most negative float, not the smallest positive one. A zero offset therefore normalizes to NaN.
  Evidence: original review harness, `review-progress.md` "Phase 2": `Zero.try_normalize(f32::MIN) -> [NaN,NaN,NaN]`.
- Observation: at the default settings flocking almost never engages. 100 vehicles spawn uniformly in a 2000 × 2000 × 1000 box, so the median nearest-neighbor distance is about 200 units, against cohesion and alignment radii of 20 and 30 units; 99% of vehicles start with no neighbor within 30 units.
  Evidence: see Artifacts and Notes, spawn-density measurement.

## Decision Log

- Decision: fix NaN sources where they are created (zero-length normalization, division by a zero distance) rather than sanitizing non-finite values after the fact.
  Rationale: one guard in the shared math covers every caller; post-hoc sanitizing hides bugs and still lets NaN reach neighbors within the same frame.
  Date: 2026-10-30.
- Decision: extract the duplicated naive/octree flocking accumulation into one function used by both paths.
  Rationale: the two copies are identical; fixing them once avoids divergence and makes the math testable without an ECS world. Neighbor selection (naive scan versus single-leaf octree lookup) stays exactly as it is.
  Date: 2026-10-30.
- Decision: a coincident neighbor (distance exactly zero) contributes nothing to separation; a zero-length desired direction yields a zero desired velocity, so the steering force becomes braking (`-velocity`, then limited).
  Rationale: there is no meaningful direction away from a coincident point; random tie-breaking would make tests nondeterministic. Coincident vehicles still separate once any other force moves them apart.
  Date: 2026-10-30.
- Decision: keep the vehicle's previous rotation when its velocity is zero.
  Rationale: `Quat::from_rotation_arc` needs a unit vector; holding the last orientation is the visually stable choice.
  Date: 2026-10-30.
- Decision: frame-rate dependence (acceleration not scaled by time), octree query exactness and default settings are out of scope.
  Rationale: each changes behavior and tuning and has its own future task; mixing them would make regressions hard to attribute.
  Date: 2026-10-30.

## Assumptions and Open Questions

- Assumption: inputs (positions, velocities, `GlobalState` values) are finite; the egui sliders bound every parameter to a positive range, and mass is at least 1.
  Why reasonable: all values come from the sliders and the spawner.
  Validation: tests use the slider minimums.
  If wrong: add validation where the value enters, not in the steering math.
- Assumption: the magnitude clamp slightly slows diagonal motion compared with today (by up to √3), which is the intended meaning of "max speed".
  Validation: visual check after the change.
  If wrong: tune `vehicle_max_speed` in the starting-configuration task, not by restoring the axis clamp.

## Outcomes & Retrospective

Delivered: steering and integration produce finite values for degenerate but ordinary inputs (vehicle on the target, coincident vehicles, zero and opposing velocities) in naive and octree modes; speed is limited by magnitude; cohesion slows inside its 10-unit arrival radius; the naive and octree paths share one flocking function. Twelve tests pass, eight of them new. Default settings, frame-rate dependence and octree neighbor selection are unchanged.

Behavior changes to expect: diagonal motion is up to √3 slower than before at the same max speed, and cohesion pulls more gently near a group's center.

Residual risk: a coincident pair gets no separation force until another force moves them apart (no observed stuck pairs in the probe; minimum pair distance stayed above 1.9). Non-finite values could still enter through future code that bypasses `normalize_or_zero`.

### Starting-configuration recommendations

Measured with the headless probe below (wander mode, naive mode, all other settings at defaults, 600 frames at a fixed 1/60 s, three position seeds; wander randomness unseeded). Because acceleration is still per frame, these results hold for 60 FPS and must be rechecked after the frame-rate task.

Tested:

1. **Spawn much denser.** With the current 100 vehicles in 2000 × 2000 × 1000, no vehicle ever has a neighbor within 20 units and flocking never engages; polarization decays from 0.89 to 0.06–0.24 over 10 s. 200 vehicles in a 100-unit cube start with about 15 neighbors within 30 units and 98.5–100% of vehicles with a neighbor within 20, and stay that way. 300 vehicles in a 150-unit cube is a workable lower-density alternative (91–95%, 8–19 neighbors).
2. **Randomize initial headings over the whole sphere.** The current spawner draws headings from the positive octant only, so a dense group starts already aligned (polarization 0.89) and flocking is not visible as it emerges. With sphere headings the same group starts disordered (0.05) and aligns to 0.63–0.88 within 10 s.

Provisional (reasoned from the code and the probe, not measured):

3. **Frame the group at startup.** The camera starts about 7 units from the origin and sees only the target. An eye roughly 200 units from the spawn center (for example `(0, 80, 180)` looking at the origin) should frame a 100-unit group.
4. **Keep the flock in view.** Dense flocks drift 140–250 units from the origin in 10 s, while walls sit at ±2000 / ±1000. Either shrink the wall bounds (the source has a commented-out 200 × 200 × 100 `WORLD_SIZE`) or have the camera follow the flock centroid; check wall avoidance at the higher density before choosing.
5. **Revisit speed defaults.** In wander mode the mean speed settles at about 25 regardless of density, far below max speed 80, which only matters when seeking. Tune `vehicle_max_speed` and `vehicle_wander_speed` together after the frame-rate task.
6. **Keep naive mode as the default** until octree queries are exact; at a few hundred vehicles naive cost is small.

Lessons: the unnatural start was mostly a density problem, not a force-tuning problem; a headless probe answered it in under a second of simulation once built.

## Context and Orientation

- `src/main.rs`: `GlobalState` holds every tunable (speeds, factors, distances, octree toggle). `configure_global_state` sets the defaults: 100 vehicles, max speed 80, mass 60, separation/alignment/cohesion distances 4/30/20 (multiplied by `vehicle_size` 1.0).
- `src/vehicle.rs`:
  - `vehicle_spawner` spawns vehicles uniformly in ±`WORLD_SIZE / 2` (`WORLD_SIZE` = 2000 × 2000 × 1000) with a random velocity in the positive octant, normalized to max speed.
  - `movement` calls `flock`, then either seeks the target (when `TargetSelection` is on) or wanders and avoids walls. Forces go through `VehicleAcceleration::apply_force` (`force / mass`).
  - `flock` has two identical copies of the separation/alignment/cohesion math: one over octree neighbors, one over all vehicles.
  - `update` adds acceleration to velocity, clamps each axis, moves the transform by `velocity * delta_secs`, and points the model along the velocity.
  - `limit` clamps a vector's magnitude.
- NaN sources (current line numbers): seek `force.normalize()` (138); wander `velocity.normalize()` and `force.normalize()` (169, 189); separation `diff.normalize().div(distance.sqrt())` with zero distance (435, 526) and `seperate_sum.normalize()` (454, 545); alignment `align_sum.normalize()` (463, 554); cohesion `try_normalize(f32::MIN)` (474, 565); rotation `velocity.normalize()` (288); spawn `normalize()` (91).
- Tests: `src/vehicle.rs` already has an ECS test that builds a minimal `App` with `GlobalState`, `RenderState` and `VehicleSpawner`; follow that pattern.

## Plan of Work

1. Add a NaN-safe normalization helper (zero vector in, zero vector out) and use it at every site listed above.
2. Move the per-vehicle separation/alignment/cohesion accumulation into one function taking the vehicle's position and velocity, an iterator of neighbor positions and velocities (self already excluded), and `&GlobalState`, returning the summed, individually limited steering force. Call it from both branches of `flock`.
3. Separation: skip zero-distance neighbors; weight by `diff / distance_squared` (equivalent to the current `normalize / distance`).
4. Cohesion: measure the offset length before normalizing; scale the desired speed by `min(dist / 10, 1)`.
5. `update`: clamp velocity magnitude with `limit` instead of per-axis `simd_clamp`; update rotation only for non-zero velocity.
6. Tests (Milestone 2), then the gate (Milestone 3), then configuration evidence (Milestone 4).

## Concrete Steps

From the repository root, sequentially, with no other Cargo process running:

1. `CARGO_BUILD_JOBS=4 cargo fmt --all -- --check` — no output.
2. `CARGO_BUILD_JOBS=4 cargo test --release --locked` — all tests pass (4 existing plus the new ones).
3. `CARGO_BUILD_JOBS=4 cargo build --release --locked` — finishes with only the two known dead-code warnings.
4. `python3 scripts/smoke-native.py` — passes after ten seconds.
5. `git diff --check` — no output.

## Validation and Acceptance

- Pure-function tests: zero vector normalizes to zero; flocking with a coincident neighbor, with exactly opposing neighbor velocities, and with the vehicle at the neighbors' center returns a finite force; cohesion toward a neighbor 5 units away asks for half of max speed (before limiting); a velocity of (80, 80, 80) with max speed 80 is clamped to magnitude 80.
- ECS test: vehicles placed at coincident positions with zero and opposing velocities, the target placed on a vehicle, run for several frames in naive and octree modes with following both on and off; every translation, rotation and velocity stays finite and no speed exceeds the maximum (small tolerance).
- Existing tests still pass; app starts in the smoke check.

## Idempotence and Recovery

All changes are in `src/vehicle.rs` plus documentation. Rerunning the gate is safe. To roll back, revert the implementation commit; the planning documents are independent.

## Artifacts and Notes

Gate (2026-10-30, sequential, `CARGO_BUILD_JOBS=4`, Rust 1.99.1, macOS ARM64):

    cargo fmt --all -- --check          # no output
    cargo test --release --locked       # test result: ok. 12 passed; 0 failed
    cargo build --release --locked      # Finished, 2 known dead-code warnings
    python3 scripts/smoke-native.py     # PASS: native release survived 10 seconds
    git diff --check                    # no output

Spawn-density measurement (2026-10-30): uniform spawn positions, 20 seeds (Python `random.Random(0..19)`), Euclidean distances. Script kept outside the repository; the method is fully described here. Radii are the default separation/cohesion/alignment distances 4/20/30.

| Vehicles | Spawn box | Median nearest neighbor | Mean neighbors within 4 / 20 / 30 | No neighbor within 30 |
| --- | --- | --- | --- | --- |
| 100 (default) | 2000 × 2000 × 1000 (default) | 200.2 | 0.00 / 0.00 / 0.01 | 99% |
| 100 | 200 × 200 × 100 | 20.0 | 0.01 / 0.69 / 2.11 | 14% |
| 200 | 100 × 100 × 100 | 9.8 | 0.06 / 5.19 / 15.51 | 0% |
| 300 | 150 × 150 × 150 | 12.7 | 0.03 / 2.51 / 7.81 | 0% |
| 500 | 200 × 200 × 100 | 11.4 | 0.03 / 3.56 / 10.96 | 0% |

This measures only initial neighbor counts, not how the flock evolves.

Headless probe (2026-10-30): a disposable copy of `Cargo.toml`, `Cargo.lock`, `rust-toolchain.toml` and `src/` with one ignored test in `src/vehicle.rs`, built with its own `CARGO_TARGET_DIR` (cold build 5 min 51 s; the probe itself ran 0.6 s). It spawns vehicles directly with a seeded RNG, runs `(movement, update).chain()` with a manually advanced `Time`, and reports metrics. Workspace and raw results were kept outside the repository and are disposable. Ranges are min–max over seeds 1–3. Polarization is the length of the mean unit velocity (0 disordered, 1 aligned). Non-finite values: 0 in every row.

| Scenario | Frame | Neighbors < 30 | % with neighbor < 20 | Polarization | Mean speed | Centroid from origin |
| --- | --- | --- | --- | --- | --- | --- |
| A: 100 in 2000×2000×1000, octant headings (current) | 0 | 0 | 0 | 0.88–0.89 | 80.0 | 67–214 |
| A | 600 | 0–0.02 | 0 | 0.06–0.24 | 19.5–19.9 | 146–309 |
| B: 200 in 100³, octant headings | 0 | 15.3–15.9 | 98.5–100 | 0.89–0.90 | 80.0 | 4–7 |
| B | 600 | 25.8–34.0 | 98.5–100 | 0.87–0.95 | 24.5–25.0 | 250–254 |
| C: 200 in 100³, sphere headings | 0 | 15.3–15.9 | 98.5–100 | 0.05–0.07 | 80.0 | 4–7 |
| C | 120 | 16.8–25.1 | 97.5–98.5 | 0.47–0.80 | 27.2–28.1 | 21–34 |
| C | 600 | 19.6–40.7 | 97.5–98.5 | 0.64–0.88 | 24.5–25.4 | 142–211 |
| D: 300 in 150³, sphere headings | 0 | 7.7–8.0 | 91.3–91.7 | 0.03–0.05 | 80.0 | 2–8 |
| D | 600 | 12.4–19.3 | 92.0–95.3 | 0.55–0.67 | 25.0–25.7 | 125–169 |

Minimum pairwise distance stayed between 1.9 and 3.8 in scenarios B–D.

## Interfaces and Dependencies

No new crates, resources or components. New private functions in `src/vehicle.rs`: a normalization helper and a shared flocking-force function. `flock`, `movement` and `update` keep their system signatures.

---

Revision notes:

- 2026-10-30: Initial plan.
- 2026-10-30: Recorded implementation, gate, probe evidence and configuration recommendations; closed the plan.
