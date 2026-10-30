# Plan status

## Current work

None. Pick the next item below.

## Prioritized future tasks

No ExecPlan exists yet for these; create one from `TEMPLATE.md` when starting.

1. **Follow-mode cost at large counts.** Followers no longer collapse, but they gather into a ball about 20 units across, inside the 30-unit alignment radius, so at 5,000 vehicles every vehicle still visits almost every other one (about 30 ms per simulation step, about 31 FPS in the app). Options: limit each vehicle to its nearest k neighbors (topological interaction, as in real starling flocks), or a smaller alignment radius in follow mode. Either changes query behavior or defaults and needs its own plan.
2. **Flocking on the GPU, if much larger flocks matter.** A compute shader with a uniform grid feeding the instance buffer directly, added as a third mode next to naive and octree. At 30,000 vehicles the CPU simulation (flocking about 9 ms) is now the main cost.

Smaller follow-ups (no plan needed): retune `vehicle_max_speed` and `vehicle_wander_speed` by eye (wander mode settles near 25 units/s against a max of 80; behavior is frame-rate independent, so tuning holds at any FPS); look at the app at startup and confirm the camera frames the flock.

CI (finding F12) is not wanted by the owner.

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
| F08 | Mass slider affects only new vehicles | Resolved: per-vehicle `VehicleMass` removed; steering reads `GlobalState::vehicle_mass` (test `mass_slider_applies_to_existing_vehicles`) |
| F09 | Benchmark disabled; visibility toggle affects only new vehicles | Resolved: benchmark plan (headless `flocking_benchmark`; checkbox removed) |
| F10 | Naive default mode is quadratic up to the 50,000 slider | Resolved up to 30,000 vehicles in wander mode: octree default, count-scaled world, 9 compute threads, instanced rendering (app at about 220 FPS with 10,000 and 80 FPS with 30,000); follow mode at large counts is quadratic (future task 1); above 30,000 unmeasured |
| F11 | Historical dependencies do not build on current toolchains | Resolved in `21769fd` and `262dc33` (Bevy 0.20.1, Rust 1.99.1) |
| F12 | No automated tests or CI | Partly resolved: 24 regression tests; CI not wanted by the owner |
| F13 | Unused direct dependencies | Resolved in `262dc33` |
| F14 | Advisory matches in the lockfile | Resolved in `262dc33`: `cargo deny check advisories` passes for the native target |

## Completed

- [Keep vehicles apart when they follow the target](completed/2026-10-30-follow-mode-separation.md) (2026-10-30): separation pushes add up and grow with closeness; followers at 1,000 and 5,000 go from 100% overlapping to none; wander flocks keep their look with no overlapping pairs; follow-mode cost at large counts deferred.
- [Instanced vehicle rendering and a visual pass](completed/2026-10-30-instanced-rendering-and-visuals.md) (2026-10-30): one instanced draw for the flock; 30,000 vehicles 53 → about 80 FPS, 10,000 about 138 → 220 FPS; heading colors, directional light, fog, wall box and floor grid; separation retune investigated, follow-mode collapse deferred to future task 1.
- [What limits frame time at 30,000 vehicles](completed/2026-10-30-high-count-frame-limit.md) (2026-10-30): CPU critical path, not GPU; 30,000 vehicles at about 53 FPS and 10,000 at about 138 FPS at low machine load; no code change; GPU time unmeasurable with available tools.
- [Profiling-guided optimization](completed/2026-10-30-profiling-guided-optimization.md) (2026-10-30): compute pool 5 → 9 threads; sort- and allocation-free neighbor queries; 10,000 vehicles about 60 → 120–138 FPS and 30,000 about 36 → 53 FPS in the app (see the frame-limit plan for the corrected 30,000 numbers).
- [Scale the spawn region with vehicle count](completed/2026-10-30-count-scaled-spawn-region.md) (2026-10-30): spawn cube and walls scale with cbrt(count / 200); octree default; 1.8 ms per step at 5,000 and 3.7 ms at 10,000 vehicles; large flocks contained but cluster tightly over time.
- [Seeded headless benchmarks](completed/2026-10-30-seeded-headless-benchmarks.md) (2026-10-30): `flocking_benchmark` ignored test for both modes on identical seeded inputs; in-app benchmark removed; naive stays default (faster at 200 vehicles); octree 6.7× faster at 10,000 only when density is constant.
- [Frame-rate-independent simulation and target movement](completed/2026-10-30-frame-rate-independence.md) (2026-10-30): acceleration and wander scaled to a 60 Hz reference, target at 120 units/s, target input ordered before steering; probe shows matching seek and overlapping flock metrics at 30/60/144 FPS.
- [Exact octree radius queries](completed/2026-10-30-exact-octree-radius-queries.md) (2026-10-30): `query_radius` with bounds pruning, containing-octant insertion, depth limit 16, root built from vehicle positions; octree and naive accelerations match on a seeded world. Performance unmeasured.
- [More natural starting configuration](completed/2026-10-30-natural-starting-configuration.md) (2026-10-30): 200 vehicles in a 100-unit cube with sphere headings, walls at ±200 / ±200 / ±100, camera about 180 units out; probe shows the flock forms and stays near the origin over 30 s. Visual framing check deferred.
- [Numerical safety and regression tests](completed/2026-10-30-numerical-safety.md) (2026-10-30): finite steering for degenerate inputs in both modes, magnitude speed limit, cohesion arrival fix, shared flocking function, 8 new tests; starting-configuration recommendations with headless probe evidence.

Earlier work (review, native baseline, modernization, controls) is recorded in `review-progress.md`.
