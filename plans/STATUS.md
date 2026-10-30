# Plan status

## Current work

None. Pick the next item below.

## Prioritized future tasks

No ExecPlan exists yet for these; create one from `TEMPLATE.md` when starting.

1. **Frame-rate-independent simulation and target movement.** Scale acceleration and wander rotation by `Time::delta_secs`, or move simulation to a fixed timestep; scale target keyboard movement by time. Changes behavior and tuning; afterwards rerun the starting-configuration probe and retune `vehicle_max_speed` and `vehicle_wander_speed` together. Finding: F03.
2. **Seeded headless benchmarks with equivalent workloads.** A headless, seeded harness that runs the same agent count and scenario for naive and octree modes (now equivalent). Decide the default mode from it. Replace or remove the disabled in-app `benchmark` system. Findings: F09, F10.
3. **Profiling-guided optimization and optional tooling.** Use `samply` and Bevy tracing against the seeded benchmark; consider Puffin, Criterion or CI only when a measured question needs them. Findings: F10, F12.

Smaller follow-ups (no plan needed): look at the app at startup and confirm the camera frames the flock (automated window capture lacks screen-recording permission); apply the mass slider to existing vehicles (F08); propagate the benchmark-mode visibility checkbox to existing vehicles (F09).

## Original review findings

`review-report.html` audited commit `e7a53d7`. Status against the current branch (`codex/native-modernization`), verified against source on 2026-10-30:

| ID | Finding | Status |
| --- | --- | --- |
| F01 | Zero-vector normalization and division create NaNs | Resolved: numerical-safety plan |
| F02 | Octree query visits one leaf and misses neighbors | Resolved: exact-octree plan |
| F03 | Acceleration, wander and target movement are frame-dependent | Outstanding: future task 1 |
| F04 | Coincident points cause deep octrees and slow rebuilds | Resolved: exact-octree plan (depth limit 16) |
| F05 | Cohesion distance measured after normalization | Resolved: numerical-safety plan |
| F06 | Per-axis speed clamp allows √3 × max speed | Resolved: numerical-safety plan |
| F07 | No explicit system order | Resolved in `262dc33` (chained spawn, cleanup, steering, integration) |
| F08 | Mass slider affects only new vehicles | Outstanding: small follow-up |
| F09 | Benchmark disabled; visibility toggle affects only new vehicles | Outstanding: future task 2 and small follow-up |
| F10 | Naive default mode is quadratic up to the 50,000 slider | Outstanding: future tasks 2 and 3 (octree is now exact, but unmeasured) |
| F11 | Historical dependencies do not build on current toolchains | Resolved in `21769fd` and `262dc33` (Bevy 0.20.1, Rust 1.99.1) |
| F12 | No automated tests or CI | Partly resolved: 18 regression tests; no CI |
| F13 | Unused direct dependencies | Resolved in `262dc33` |
| F14 | Advisory matches in the lockfile | Resolved in `262dc33`: `cargo deny check advisories` passes for the native target |

## Completed

- [Exact octree radius queries](completed/2026-10-30-exact-octree-radius-queries.md) (2026-10-30): `query_radius` with bounds pruning, containing-octant insertion, depth limit 16, root built from vehicle positions; octree and naive accelerations match on a seeded world. Performance unmeasured.
- [More natural starting configuration](completed/2026-10-30-natural-starting-configuration.md) (2026-10-30): 200 vehicles in a 100-unit cube with sphere headings, walls at ±200 / ±200 / ±100, camera about 180 units out; probe shows the flock forms and stays near the origin over 30 s. Visual framing check deferred.
- [Numerical safety and regression tests](completed/2026-10-30-numerical-safety.md) (2026-10-30): finite steering for degenerate inputs in both modes, magnitude speed limit, cohesion arrival fix, shared flocking function, 8 new tests; starting-configuration recommendations with headless probe evidence.

Earlier work (review, native baseline, modernization, controls) is recorded in `review-progress.md`.
