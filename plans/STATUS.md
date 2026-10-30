# Plan status

## Current work

None. Pick the next item below.

## Prioritized future tasks

No ExecPlan exists yet for these; create one from `TEMPLATE.md` when starting.

1. **Scale the spawn region with vehicle count.** The count slider packs every vehicle into a 100-unit cube, so density grows with count and the octree cannot help (about 1.1× faster at 10,000). At constant density it is 6.7× faster at 10,000. Keep density near the measured 200-per-100³ value (edge × (count / 200)^(1/3)), adjust walls and camera to match, and consider selecting octree mode automatically above about 2,000 vehicles. Rerun `flocking_benchmark` and the starting-configuration probe. Finding: F10.
2. **Profiling-guided optimization and optional tooling.** Use `samply` and Bevy tracing against `flocking_benchmark` and the running app (rendering is not in the benchmark); candidates include per-vehicle query allocations and the per-frame octree rebuild. Consider Puffin, Criterion or CI only when a measured question needs them. Findings: F10, F12.

Smaller follow-ups (no plan needed): retune `vehicle_max_speed` and `vehicle_wander_speed` together by eye (wander mode settles near 25 units/s against a max of 80; behavior is now frame-rate independent, so tuning holds at any FPS); look at the app at startup and confirm the camera frames the flock (automated window capture lacks screen-recording permission); apply the mass slider to existing vehicles (F08).

## Original review findings

`review-report.html` audited commit `e7a53d7`. Status against the current branch (`codex/native-modernization`), verified against source on 2026-10-30:

| ID | Finding | Status |
| --- | --- | --- |
| F01 | Zero-vector normalization and division create NaNs | Resolved: numerical-safety plan |
| F02 | Octree query visits one leaf and misses neighbors | Resolved: exact-octree plan |
| F03 | Acceleration, wander and target movement are frame-dependent | Resolved: frame-rate plan (60 Hz reference) |
| F04 | Coincident points cause deep octrees and slow rebuilds | Resolved: exact-octree plan (depth limit 16) |
| F05 | Cohesion distance measured after normalization | Resolved: numerical-safety plan |
| F06 | Per-axis speed clamp allows √3 × max speed | Resolved: numerical-safety plan |
| F07 | No explicit system order | Resolved in `262dc33` (chained spawn, cleanup, steering, integration) |
| F08 | Mass slider affects only new vehicles | Outstanding: small follow-up |
| F09 | Benchmark disabled; visibility toggle affects only new vehicles | Resolved: benchmark plan (headless `flocking_benchmark`; checkbox removed) |
| F10 | Naive default mode is quadratic up to the 50,000 slider | Measured: naive is about 26 ms per step at 10,000; outstanding as future tasks 1 and 2 |
| F11 | Historical dependencies do not build on current toolchains | Resolved in `21769fd` and `262dc33` (Bevy 0.20.1, Rust 1.99.1) |
| F12 | No automated tests or CI | Partly resolved: 22 regression tests; no CI |
| F13 | Unused direct dependencies | Resolved in `262dc33` |
| F14 | Advisory matches in the lockfile | Resolved in `262dc33`: `cargo deny check advisories` passes for the native target |

## Completed

- [Seeded headless benchmarks](completed/2026-10-30-seeded-headless-benchmarks.md) (2026-10-30): `flocking_benchmark` ignored test for both modes on identical seeded inputs; in-app benchmark removed; naive stays default (faster at 200 vehicles); octree 6.7× faster at 10,000 only when density is constant.
- [Frame-rate-independent simulation and target movement](completed/2026-10-30-frame-rate-independence.md) (2026-10-30): acceleration and wander scaled to a 60 Hz reference, target at 120 units/s, target input ordered before steering; probe shows matching seek and overlapping flock metrics at 30/60/144 FPS.
- [Exact octree radius queries](completed/2026-10-30-exact-octree-radius-queries.md) (2026-10-30): `query_radius` with bounds pruning, containing-octant insertion, depth limit 16, root built from vehicle positions; octree and naive accelerations match on a seeded world. Performance unmeasured.
- [More natural starting configuration](completed/2026-10-30-natural-starting-configuration.md) (2026-10-30): 200 vehicles in a 100-unit cube with sphere headings, walls at ±200 / ±200 / ±100, camera about 180 units out; probe shows the flock forms and stays near the origin over 30 s. Visual framing check deferred.
- [Numerical safety and regression tests](completed/2026-10-30-numerical-safety.md) (2026-10-30): finite steering for degenerate inputs in both modes, magnitude speed limit, cohesion arrival fix, shared flocking function, 8 new tests; starting-configuration recommendations with headless probe evidence.

Earlier work (review, native baseline, modernization, controls) is recorded in `review-progress.md`.
