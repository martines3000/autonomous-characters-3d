# Plan status

## Current work

None. Pick the next item below.

## Prioritized future tasks

No ExecPlan exists yet for these; create one from `TEMPLATE.md` when starting.

1. **More natural starting configuration.** Apply the recommendations in the completed numerical-safety plan ("Starting-configuration recommendations"). Measured: denser spawn (about 200 vehicles in a 100-unit cube) and headings over the whole sphere. Provisional: camera framing, keeping the flock in view (wall bounds or camera follow), speed defaults. Change defaults only with before/after evidence. At the current defaults flocking never engages.
2. **Exact octree radius queries.** Replace the single-leaf `find_neighbors` with a radius query that visits every overlapping node, check it against a naive reference on seeded inputs, and bound subdivision depth for coincident points. Until then, octree mode is not equivalent to naive mode. Findings: F02, F04.
3. **Frame-rate-independent simulation and target movement.** Scale acceleration and wander rotation by `Time::delta_secs`, or move simulation to a fixed timestep; scale target keyboard movement by time. Changes behavior and tuning, so do it after the numerical tests exist. Finding: F03.
4. **Seeded headless benchmarks with equivalent workloads.** A headless, seeded harness that runs the same agent count and scenario for naive and octree modes, after exact queries make them equivalent. Replace or remove the disabled in-app `benchmark` system. Findings: F09, F10.
5. **Profiling-guided optimization and optional tooling.** Use `samply` and Bevy tracing against the seeded benchmark; consider Puffin, Criterion or CI only when a measured question needs them. Findings: F10, F12.

Smaller follow-ups (no plan needed): apply the mass slider to existing vehicles (F08); propagate the benchmark-mode visibility checkbox to existing vehicles (F09).

## Original review findings

`review-report.html` audited commit `e7a53d7`. Status against the current branch (`codex/native-modernization`), verified against source on 2026-10-30:

| ID | Finding | Status |
| --- | --- | --- |
| F01 | Zero-vector normalization and division create NaNs | Resolved: numerical-safety plan |
| F02 | Octree query visits one leaf and misses neighbors | Outstanding: future task 2 |
| F03 | Acceleration, wander and target movement are frame-dependent | Outstanding: future task 3 |
| F04 | Coincident points cause deep octrees and slow rebuilds | Outstanding: future task 2 |
| F05 | Cohesion distance measured after normalization | Resolved: numerical-safety plan |
| F06 | Per-axis speed clamp allows √3 × max speed | Resolved: numerical-safety plan |
| F07 | No explicit system order | Resolved in `262dc33` (chained spawn, cleanup, steering, integration) |
| F08 | Mass slider affects only new vehicles | Outstanding: small follow-up |
| F09 | Benchmark disabled; visibility toggle affects only new vehicles | Outstanding: future task 4 and small follow-up |
| F10 | Naive default mode is quadratic up to the 50,000 slider | Outstanding: future tasks 2, 4 and 5 |
| F11 | Historical dependencies do not build on current toolchains | Resolved in `21769fd` and `262dc33` (Bevy 0.20.1, Rust 1.99.1) |
| F12 | No automated tests or CI | Partly resolved: 12 regression tests; no CI |
| F13 | Unused direct dependencies | Resolved in `262dc33` |
| F14 | Advisory matches in the lockfile | Resolved in `262dc33`: `cargo deny check advisories` passes for the native target |

## Completed

- [Numerical safety and regression tests](completed/2026-10-30-numerical-safety.md) (2026-10-30): finite steering for degenerate inputs in both modes, magnitude speed limit, cohesion arrival fix, shared flocking function, 8 new tests; starting-configuration recommendations with headless probe evidence.

Earlier work (review, native baseline, modernization, controls) is recorded in `review-progress.md`.
