# Frame-rate-independent simulation and target movement

This ExecPlan is a living document maintained according to `PLANS.md`.

## Purpose

Steering forces are added to velocity once per rendered frame, wander angles take one random step per frame, and the selected target moves 2 units per frame. On a 120 Hz display vehicles therefore steer twice as hard and the target moves twice as fast as at 60 Hz, and under heavy load everything slows down. After this work, the same settings produce the same motion per second at any frame rate, matching today's behavior at 60 FPS, which is the rate all tuning and headless measurements used.

## Progress

- [x] (2026-10-30 15:40Z) Plan drafted; finding F03 confirmed in current source (`update` adds acceleration unscaled; wander steps per frame; `input_controls` moves 2 units per frame).
- [x] (2026-10-30 15:40Z) Design settled: variable step scaled to a 60 Hz reference (Decision Log).
- [x] (2026-10-30 16:00Z) Milestone 1: `REFERENCE_HZ` 60; `update` scales acceleration by `dt * 60`; `wander_step_bound(dt)` scales the wander step by `sqrt(dt * 60)`; `TARGET_SPEED` 120 units/s; vehicle chain runs `.after(input_controls)`.
- [x] (2026-10-30 16:00Z) Milestone 2: four new tests (constant acceleration and held-key movement over 1 s at 30/60/120/144 steps, wander step bound, 0.25 s large-force step). The first three fail under the old per-frame formulas (30 steps give half the velocity and 60 instead of 120 units; the old wander bound does not shrink at 120 Hz).
- [x] (2026-10-30 16:15Z) Milestone 3: gate passed; probe at 30/60/144 FPS (Artifacts and Notes).
- [x] (2026-10-30 16:20Z) Plan closed and moved to `plans/completed/`.

## Surprises & Discoveries

- Observation: no surprises in behavior. The only setup change was that `target.rs` tests now need a `Time` resource because `input_controls` reads it.
  Evidence: `target_selection_controls_movement_and_color` gained `Time` setup; its assertions are unchanged.

## Decision Log

- Decision: variable timestep in `Update`, treating today's per-frame quantities as rates at a 60 Hz reference, instead of moving the simulation to `FixedUpdate`.
  Rationale: a fixed 60 Hz step would preserve trajectories exactly, but Bevy 0.20.1 has no built-in transform interpolation, so at 120/144 Hz vehicles would visibly hold poses across frames; and under heavy load catch-up would run the expensive flocking step several times per frame. A variable step updates every frame and is the smaller change. It preserves behavior exactly at 60 FPS and approximately elsewhere (Euler integration and changing neighbors remain step-sensitive). An independent Codex design review reached the same recommendation.
  Date: 2026-10-30.
- Decision: scale acceleration once, in `update` (`velocity += acceleration * dt * 60`), after the existing per-force limits and mass division.
  Rationale: one place covers seek, wander, wall avoidance and both flocking paths without retuning any factor.
  Date: 2026-10-30.
- Decision: scale the wander random-walk step by `sqrt(dt * 60)`.
  Rationale: a random walk's variance grows with the number of steps; scaling the step by the square root of the time ratio keeps the angle variance per second the same at any frame rate.
  Date: 2026-10-30.
- Decision: target keyboard speed 120 units/s (2 units × 60), ordered before vehicle steering.
  Rationale: preserves the 60 FPS feel; the two systems are currently unordered, so steering may read last frame's target position.
  Date: 2026-10-30.
- Decision: speed retuning (wander mode settles near 25 units/s while max speed is 80) stays out of scope.
  Rationale: at 60 FPS behavior is unchanged, so the starting-configuration measurements still hold; retuning is a separate, visual decision recorded as a follow-up.
  Date: 2026-10-30.

## Assumptions and Open Questions

- Assumption: Bevy's virtual-time cap (`max_delta`, 250 ms by default) bounds the largest step.
  Why reasonable: `Time` in `Update` is virtual time.
  Validation: read `bevy_time` 0.20.1 `virt.rs`.
  If wrong: clamp `dt` in `update`.
- Assumption: a 250 ms step (15× the reference) stays finite and bounded because `limit` caps speed after integration.
  Validation: the existing degenerate-input test plus one large-step case.
  If wrong: clamp `dt`.

## Outcomes & Retrospective

Delivered: steering, wander turning and target movement are now defined per second, with today's 60 FPS behavior as the reference. The per-second contracts are tested exactly; the whole flock is approximately frame-rate independent: in the probe, seeking converges identically at 30, 60 and 144 FPS (mean distance 6.8 units after 10 s), and wander-mode metrics at 30 and 144 FPS fall within the seed-to-seed spread at 60 FPS. Finding F03 is resolved. 22 tests pass; Clippy is unchanged at 9 pre-existing warnings.

Remaining by design: trajectories are not bit-identical across frame rates (Euler integration, changing neighbors, unseeded wander). A fixed timestep with interpolation would be needed for that, and nothing currently requires it. Speed retuning stays a follow-up in `plans/STATUS.md`.

Lessons: settling the design with a short independent review before editing kept the change to three formulas and one ordering constraint.

## Context and Orientation

- `src/vehicle.rs`: `VehiclePlugin` registers `(vehicle_spawner, vehicle_cleanup, movement, update).chain()` in `Update`. `movement` computes forces and calls `VehicleAcceleration::apply_force` (`acceleration += force / mass`). In wander mode it adds a random step in ±`PI / 16` to `VehicleWanderRotation { theta, phi }` each call. `update` adds acceleration to velocity, limits speed, moves by `velocity * delta_secs`, and resets acceleration.
- `src/target.rs`: `TargetPlugin` registers `input_controls` and `update_target_color` in `Update`; `input_controls` moves the selected target by `2.0` units per frame in camera-relative directions. Its test drives `input_controls` with `app.update()`.
- Existing tests advance `Time` manually by 1/60 s (`degenerate_vehicles_stay_finite_and_within_max_speed`).

## Plan of Work

1. Add a 60 Hz reference constant in `src/vehicle.rs`; in `update` scale acceleration by `dt * 60`; in `movement` scale the wander step range by `sqrt(dt * 60)`.
2. In `src/target.rs` move the target at 120 units/s times `delta_secs`; make it public enough to order against vehicle steering, and order it before the vehicle chain.
3. Tests: constant acceleration over 1 s at 30/60/120/144 steps per second gives the same final velocity; held key over 1 s moves the target 120 units at each rate; the wander step bound scales with `sqrt(dt * 60)` (deterministic, no RNG statistics); a 250 ms step stays finite and within max speed. Update the target test to advance `Time`.
4. Gate; then a headless probe of the default flock at 30, 60 and 144 FPS for 30 s.

## Concrete Steps

From the repository root, sequentially, with no other Cargo process running:

1. `CARGO_BUILD_JOBS=4 cargo fmt --all -- --check`
2. `CARGO_BUILD_JOBS=4 cargo test --release --locked`
3. `CARGO_BUILD_JOBS=4 cargo build --release --locked`
4. `python3 scripts/smoke-native.py`
5. `git diff --check`

## Validation and Acceptance

- Final velocity after 1 s of constant acceleration agrees across 30/60/120/144 Hz within float tolerance.
- Target displacement after 1 s of a held key is 120 units at every rate.
- Behavior at exactly 1/60 s per frame is unchanged (existing tests pass unmodified apart from `Time` setup).
- Probe: flock metrics (polarization, neighbor counts, centroid drift) at 30 and 144 FPS fall within or near the seed-to-seed spread at 60 FPS.

## Idempotence and Recovery

Changes are local to `update`, the wander step in `movement` and `input_controls`. Revert the commit to restore per-frame behavior.

## Artifacts and Notes

Gate (2026-10-30, sequential, `CARGO_BUILD_JOBS=4`): `cargo fmt --all -- --check` clean; `cargo test --release --locked` 22 passed; `cargo build --release --locked` with 1 known warning; `python3 scripts/smoke-native.py` PASS; `git diff --check` clean; Clippy 9 warnings, all pre-existing.

Frame-rate probe (2026-10-30): disposable copy of the working tree with its own `CARGO_TARGET_DIR`, current defaults (200 vehicles in the 100-unit cube, sphere headings, walls ±200 / ±200 / ±100), wander and naive mode, 30 s of simulated time at fixed steps, seeds 1–3 (min–max; wander unseeded). Non-finite values: 0 in every row.

| Step | Time | Polarization | Neighbors < 30 | % with neighbor < 20 | Mean speed | Centroid from origin | % outside walls |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 1/30 s | 5 s | 0.39–0.60 | 25.0–40.7 | 98.0–98.5 | 24.5–24.7 | 58–87 | 1.0–10.0 |
| 1/60 s | 5 s | 0.39–0.66 | 25.7–41.5 | 97.5–98.5 | 24.0–25.4 | 54–86 | 0–14.5 |
| 1/144 s | 5 s | 0.36–0.58 | 25.5–30.9 | 97.5–99.0 | 24.3–25.2 | 56–82 | 0–20.0 |
| 1/30 s | 30 s | 0.24–0.60 | 42.3–90.3 | 99.0–99.5 | 23.9–24.5 | 82–142 | 1.0–3.0 |
| 1/60 s | 30 s | 0.43–0.64 | 41.9–67.7 | 99.0 | 22.4–23.7 | 118–135 | 9.0–13.5 |
| 1/144 s | 30 s | 0.21–0.53 | 35.3–50.3 | 98.5–100 | 24.3–24.4 | 37–106 | 0–0.5 |

Seek, target fixed at (150, 0, 0), mean distance to target:

| Step | 0 s | 2.5 s | 5 s | 10 s |
| --- | --- | --- | --- | --- |
| 1/30 s | 153–157 | 48.5–59.2 | 7.1–10.6 | 6.8–6.9 |
| 1/60 s | 153–157 | 49.6–60.0 | 6.8–9.4 | 6.8 |
| 1/144 s | 153–157 | 49.7–60.2 | 7.8–10.9 | 6.8 |

## Interfaces and Dependencies

No new crates. `target::input_controls` is now `pub(crate)` so `VehiclePlugin` can run after it. New private items: `REFERENCE_HZ`, `wander_step_bound`, `TARGET_SPEED`.

---

Revision notes:

- 2026-10-30: Initial plan.
- 2026-10-30: Recorded implementation, tests, gate and frame-rate probe; closed.
