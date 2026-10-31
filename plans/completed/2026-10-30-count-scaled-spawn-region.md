# Scale the spawn region with vehicle count

This ExecPlan is a living document maintained according to `PLANS.md`.

## Purpose

The count slider (0–50,000) packs every vehicle into the same 100-unit spawn cube and the same walls, so density grows with the count. At 10,000 vehicles each vehicle has over a thousand neighbors, simulation costs about 26 ms per step in naive mode and 24 ms in octree mode, and the flock is a crowded mass rather than flocking. After this work, the spawn cube and the walls grow with the cube root of the count, keeping density near the 200-vehicles-per-100-unit-cube value measured to flock well, and octree mode is the default, so higher counts stay affordable (about 4 ms per step at 10,000 in the benchmark) and still look like a flock. The default 200-vehicle scene is unchanged.

## Progress

- [x] (2026-10-30 17:30Z) Plan drafted from the benchmark results in `plans/completed/2026-10-30-seeded-headless-benchmarks.md`.
- [x] (2026-10-30 17:45Z) Milestone 1: `REFERENCE_COUNT` 200 and `world_scale(count)`; `vehicle_spawner` and wall avoidance in `movement` scale by it; `use_octree` defaults to true.
- [x] (2026-10-30 17:50Z) Milestone 2: `flocking_benchmark` uses the app rule (sets `vehicle_count` so walls scale); `world_scale_keeps_density_above_the_reference_count` test; gate passed.
- [x] (2026-10-30 18:05Z) Milestone 3: probe at 200/2,000/5,000 and two benchmark runs recorded.
- [x] (2026-10-30 18:10Z) Plan closed and moved to `plans/completed/`.

## Surprises & Discoveries

- Observation: equal starting density does not mean equal flock structure. Larger groups align quickly (polarization 0.78–0.83 at 5 s, higher than 0.49–0.59 at 200), then compress into denser clusters (90–160 neighbors within 30 units at 30 s against 36–53 at 200) and global alignment falls to 0.2–0.4, while the 200-vehicle flock stays at 0.66–0.72. Global polarization drops when several sub-flocks head in different directions, which is plausible here but was not checked visually.
  Evidence: probe table below.
- Observation: the minimum distance between vehicles falls to about 0.9–1.2 units at 5,000 vehicles, below the 2-unit cone height, so some cones overlap in tight clusters. Separation (radius 4) is not strong enough to prevent it at these densities.
  Evidence: probe table below.
- Observation: a load spike during one benchmark run inflated the 10,000-vehicle rows (naive median 98.7 ms with a 420 ms p95; octree 10.2 ms) while smaller counts were unaffected; the 1-minute load average rose from 6.6 to 13.1 between runs.
  Evidence: run 1 versus run 2 below.

## Decision Log

- Decision: scale factor `max(1, (count / 200)^(1/3))`, applied to both the spawn cube edge and the wall half-extents, computed from the current `vehicle_count`.
  Rationale: keeps density, and the wall-to-spawn ratio that kept the flock contained, equal to the measured default. The floor of 1 keeps small counts in the default world instead of shrinking it. Vehicles spawned earlier keep their positions when the slider moves; walls follow the current count, so lowering the count pulls survivors back inside.
  Date: 2026-10-30.
- Decision: make octree mode the default, reversing the benchmark plan's choice.
  Rationale: at 200 vehicles octree costs 0.14 ms against 0.08 ms, a difference invisible in a 16.7 ms frame, while at 10,000 constant-density vehicles it costs 3.8 ms against 26 ms. Both modes compute identical flocking. Defaulting to octree removes the cliff without an automatic mode switch; the checkbox still allows naive mode as a reference.
  Date: 2026-10-30.
- Decision: leave the camera's starting position as is.
  Rationale: the scale factor is 1 at the default count, so the startup framing is unchanged; at larger counts the user zooms out, as with any count change.
  Date: 2026-10-30.
- Decision: the benchmark keeps one workload, the app's own spawn rule.
  Rationale: after this change the app rule is the constant-density workload; the fixed-cube workload no longer describes anything the app does.
  Date: 2026-10-30.

## Assumptions and Open Questions

- Assumption: flocking and containment at larger counts behave like the measured 200-vehicle case, because density and the wall-to-spawn ratio are the same.
  Validation: Milestone 3 probe at 2,000 and 5,000 vehicles.
  If wrong: tune the wall ratio for large counts, or record the limit.

## Outcomes & Retrospective

Delivered: the spawn cube and walls grow with the cube root of the vehicle count (never below the default world), and octree mode is the default. At the default 200 vehicles the scene is unchanged. At larger counts starting density matches the default, the flock stays inside the scaled walls (at most 6% outside, overshoot up to 10 units), and simulation is affordable: about 1.8 ms per step at 5,000 and 3.7 ms at 10,000 vehicles in the default mode, against 6.7 ms and 25.5 ms for naive mode. F10 is resolved for counts up to 10,000; the slider's upper range (to 50,000) is not measured.

Remaining: large flocks cluster more tightly over time and some cones overlap; whether separation strength should scale with group size is a tuning question for the speed-and-force retuning follow-up. Rendering cost at large counts is unmeasured (profiling task).

Lessons: keeping start density constant fixed the cost cliff, but behavior over time still depends on group size; measuring 30 s rather than the start state showed it.

## Context and Orientation

- `src/vehicle.rs`: `WORLD_SIZE` (wall half-extents 200 × 200 × 100) is used only by wall avoidance in `movement`; `SPAWN_SIZE` (100) by `vehicle_spawner`. The octree root already comes from the vehicles' bounding box. `flocking_benchmark` (ignored test) has "app" and "density" workloads.
- `src/main.rs`: `configure_global_state` sets `vehicle_count` 200 and `use_octree` false; the menu slider goes to 50,000.

## Plan of Work

1. Add a `world_scale(count)` function and a named reference count (200) in `src/vehicle.rs`; scale spawn positions and the wall half-extents by it.
2. Set `use_octree = true` in `configure_global_state`.
3. Benchmark: one workload using the app's scaled spawn rule.
4. Tests: `world_scale` values (below, at and above the reference count); spawner test still holds at the default count.
5. Gate; probe at 200, 2,000 and 5,000 vehicles; rerun the benchmark twice.

## Concrete Steps

From the repository root, sequentially, with no other Cargo process running:

1. `CARGO_BUILD_JOBS=4 cargo fmt --all -- --check`
2. `CARGO_BUILD_JOBS=4 cargo test --release --locked`
3. `CARGO_BUILD_JOBS=4 cargo build --release --locked`
4. `python3 scripts/smoke-native.py`
5. `git diff --check`
6. `uptime`, then `CARGO_BUILD_JOBS=4 cargo test --release --locked flocking_benchmark -- --ignored --nocapture --test-threads=1` (twice)

## Validation and Acceptance

- At 200 vehicles nothing changes except the default mode (which computes identical flocking).
- At 2,000 and 5,000 vehicles the probe shows flocking (polarization rising from near zero) and most vehicles inside the scaled walls over 30 s.
- Benchmark at 10,000 vehicles in the default (octree) mode stays near 4 ms per step.

## Idempotence and Recovery

Constants and one function in `src/vehicle.rs`, one default in `src/main.rs`. Revert the commit to restore fixed sizes and the naive default.

## Artifacts and Notes

Gate (2026-10-30, sequential, `CARGO_BUILD_JOBS=4`): `cargo fmt --all -- --check` clean; `cargo test --release --locked` 23 passed, 1 ignored; `cargo build --release --locked` with no warnings; `python3 scripts/smoke-native.py` PASS; `git diff --check` clean; Clippy 8 pre-existing warnings.

Scaled-world probe (2026-10-30): disposable copy of the working tree with its own `CARGO_TARGET_DIR`; `vehicle_count` set to the scenario count; positions in the scaled spawn cube, sphere headings, seeds 1–3 (min–max; wander unseeded); defaults (octree mode); wander mode; 1/60 s steps for 30 s. Centroid distance is divided by the world scale (1, 2.15 and 2.92). Non-finite values: 0 in every row.

| Vehicles | Time | Polarization | Neighbors < 30 | % with neighbor < 20 | Centroid / scale | % outside walls | Max overshoot | Min pair distance |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 200 | 5 s | 0.49–0.59 | 24.9–31.9 | 97.5–98.5 | 57–87 | 0.5–12.0 | 1.3–5.5 | 1.7–2.1 |
| 200 | 30 s | 0.66–0.72 | 36.0–53.4 | 98.5–100 | 94–182 | 2.5–24.5 | 2.5–6.1 | 1.9–2.1 |
| 2,000 | 5 s | 0.79–0.83 | 46.9–58.7 | 98.8–99.2 | 37–39 | 1.1–1.6 | 3.6–10.1 | 1.5–1.7 |
| 2,000 | 30 s | 0.37–0.43 | 91.5–163.3 | 99.2–99.5 | 43–77 | 0.4–5.7 | 2.3–7.8 | 1.2–1.5 |
| 5,000 | 5 s | 0.78–0.83 | 65.4–76.2 | 99.2–99.6 | 25–28 | 0–0.02 | 0–2.6 | 1.2–1.4 |
| 5,000 | 30 s | 0.21–0.35 | 122.5–133.0 | 99.6–99.7 | 62–81 | 1.7–2.0 | 5.2–6.4 | 0.9–1.2 |

Benchmark (2026-10-30, Apple M3 Pro, 11 threads, seed `0xB01D5`, app spawn rule, median ms per step; run 2, with run 1 in parentheses where a load spike disturbed it):

| Vehicles | Naive | Octree (default) |
| --- | --- | --- |
| 200 | 0.07 (0.06) | 0.14 (0.11) |
| 1,000 | 0.59 (0.41) | 0.56 (0.45) |
| 2,000 | 1.46 (1.20) | 0.84 (0.79) |
| 5,000 | 6.75 (7.57) | 1.82 (1.87) |
| 10,000 | 25.53 (98.74, load spike) | 3.74 (10.18, load spike) |

## Interfaces and Dependencies

No new crates. New private `REFERENCE_COUNT` and `world_scale(count: usize) -> f32` in `src/vehicle.rs`; `configure_global_state` sets `use_octree = true`; `flocking_benchmark` prints `count=… mode=…` lines (one workload).

---

Revision notes:

- 2026-10-30: Initial plan.
- 2026-10-30: Recorded implementation, probe, benchmark and clustering finding; closed.
