# Plan status

## Current work

- [Visual showcase, custom models and swarm mechanics](planned/2026-10-31-visual-showcase-and-swarm-features.md) (umbrella, started 2026-10-31, branch `feat/visual-showcase-and-swarm-features`): seven owner-authorized stages, each with a linked stage plan.

## Prioritized future tasks

No ExecPlan exists yet for these; create one from `TEMPLATE.md` when starting.

1. **Follow-mode density at 100,000+ vehicles.** With GPU follow cap 16, 15–42% of sampled vehicles have a neighbor closer than the 1.4-unit dart width after the benchmark warmup (about 19% at 100k, 42% at 300k, 15% at one million). Complete separation does not hold against seek pressure in very large balls. This needs a behavior decision and its own plan, rather than a larger alignment/cohesion sample cap.
2. **Rendering cost in close-up views.** On the M3 Pro, 100,000 wandering vehicles viewed close up reach about 59 FPS, while 300,000 viewed from far away reach about 92 FPS. Investigate fewer triangles per dart or distance impostors, MSAA cost and a depth pre-pass; measure each before changing visuals.
3. **Wall avoidance acts as drag inside the walls.** Both CPU and GPU apply the wall-avoidance term every step, even in bounds, steering toward zero velocity. Wander speed settles near 29 units/s rather than the configured 40. The owner should decide the intended behavior before changing both implementations together.

Smaller follow-ups (no plan needed): retune `vehicle_max_speed` and `vehicle_wander_speed` by eye after the wall-drag decision (wander currently settles near 29 units/s against a max of 80; behavior is frame-rate independent, so tuning holds at any FPS).

CI (finding F12) is not wanted by the owner.

- **Feature expansion roadmap — 2026-10-30:** see [report section 12](../review-report.html#feature-expansion). On 2026-10-31 the owner authorized seven stages (visuals, appearance, custom models, rendering optimization, presentation, animated models, swarm mechanics); see Current work.

## Original review findings

`review-report.html` audited commit `e7a53d7`. Status against `master` (the native modernization was merged in `3f8813a`), verified against source on 2026-10-30:

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
| F10 | Naive default mode is quadratic up to the 50,000 slider | Resolved: GPU default at 20,000, slider up to one million, complete separation with follow cap 16; M3 Pro headless steps 0.67 ms wander / 1.4 ms follow at 20k, 7.4 / 13.2 ms at 300k. Naive/Octree remain CPU references (sliders up to 50k); extreme follow density and close-up rendering cost are future tasks 1–2 |
| F11 | Historical dependencies do not build on current toolchains | Resolved in `21769fd` and `262dc33` (Bevy 0.20.1, Rust 1.99.1) |
| F12 | No automated tests or CI | Partly resolved: 33 non-ignored regression tests, 3 ignored GPU regression tests, and separate ignored CPU/GPU benchmarks; GPU runs need a real device outside restricted sandboxes; CI not wanted by the owner |
| F13 | Unused direct dependencies | Resolved in `262dc33` |
| F14 | Advisory matches in the lockfile | Resolved in `262dc33`: `cargo deny check advisories` passes for the native target |

## Completed

- [Flock on the GPU](completed/2026-10-30-gpu-flocking.md) (2026-10-30): GPU default at 20,000 / follow cap 16; M3 Pro headless wander/follow steps 0.67/1.4 ms at 20k, 7.4/13.2 ms at 300k, 26.1/44.9 ms at one million; default app about 107–120 FPS wandering, scaled target and camera; CPU parity and complete separation tested on the GPU. Large follow density, rendering cost and wall drag remain above.
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
