# What limits frame time at 30,000 vehicles

This ExecPlan is a living document maintained according to `PLANS.md`.

## Purpose

After the profiling-guided optimization, the app at 30,000 vehicles still takes about 25–30 ms per frame, yet no CPU thread is saturated (nine compute threads at about 48%, render thread 22%, main thread 4%). CPU sampling cannot see GPU time or show which systems wait on which. This work measures GPU pass times and a per-system frame timeline, names the limit, and fixes it if the app controls it. If the answer is "GPU-bound on work we cannot reduce cheaply", the outcome is a recorded measurement.

## Progress

- [x] (2026-10-30 19:50Z) Plan drafted; tools checked (Xcode `xctrace` 16.0, Bevy `RenderDiagnosticsPlugin`, Bevy `trace_chrome` feature).
- [x] (2026-10-30 20:30Z) Milestone 1: render-pass diagnostics and a `trace_chrome` timeline at 10,000 and 30,000 vehicles. GPU execution time could not be measured (see Surprises).
- [x] (2026-10-30 20:40Z) Milestone 2: the limit is the CPU critical path (simulation in the main world, mesh collection in the render world); no fix in this plan (Decision Log).
- [x] (2026-10-30 20:40Z) Milestone 3: cancelled; no fix warranted within this plan's scope.
- [x] (2026-10-30 20:45Z) Plan closed and moved to `plans/completed/`.

## Surprises & Discoveries

- Observation: GPU execution time cannot be read with Bevy's diagnostics on this Mac. The adapter supports `TIMESTAMP_QUERY` but not timestamps inside passes, which Bevy's recorder needs, so only CPU command-recording times per pass are available (about 0.05 ms per frame in total). A Metal System Trace from `xctrace` stalled while finalizing and its export failed ("Document Missing Template Error").
  Evidence: profiling handback in the disposable workspace (`profile/gpu-handback.txt`).
- Observation: the "after" frame times recorded in the profiling-guided optimization plan were pessimistic because the machine was heavily loaded then. At load averages 5–8, 30,000 vehicles run at 18.7–18.9 ms median (about 53 FPS) and 10,000 at 7.25 ms (about 138 FPS), against 27.8 ms and 16.6 ms before that optimization.
  Evidence: variant A runs below.
- Observation: frame time matches the CPU critical path. With tracing at 30,000, the main-world schedule takes 15.6 ms and the frame 16.1 ms; the render world takes 13.1 ms in parallel (pipelined rendering), and the main world spends almost nothing waiting for it. So the GPU is not what sets frame time, even though its execution time is unmeasured.
  Evidence: variant B timeline below.

## Decision Log

- Decision: no code change in this plan.
  Rationale: the two largest spans are inherent per-vehicle work. Parallel flocking (9.3 ms at 30,000) is already the shared, exact, allocation-free path; cutting it further means fewer neighbors per vehicle, which is a tuning question (large flocks compress to 120+ neighbors within 30 units, see the count-scaled plan) and belongs with the separation retune. Mesh collection in the render world (9.4 ms) is Bevy's per-moving-mesh cost; reducing it would mean custom instanced rendering, a large change for a demo that already runs 30,000 vehicles at about 53 FPS. Smaller levers (lighter `OctreeData` without the cloned `Transform`, a parallel octree build for the 1.4 ms serial span) were judged too small to justify now.
  Date: 2026-10-30.

- Decision: measure in the disposable profiling copy with Bevy's `RenderDiagnosticsPlugin` (GPU timestamp queries per render pass) and the `trace_chrome` feature (per-system spans), plus a Metal System Trace from `xctrace` only if those disagree or leave the GPU picture unclear.
  Rationale: Bevy's own diagnostics name passes and systems directly; `xctrace` traces are large and harder to attribute. Neither belongs in the shipped app, and enabling tracing changes dependencies, which is acceptable only in a disposable copy.
  Date: 2026-10-30.

## Assumptions and Open Questions

- Assumption: raw GPU geometry work is small: the cone has 128 vertices and 62 triangles (about 1.9 million triangles at 30,000 vehicles), the scene has only ambient light (no shadow passes), and MSAA is 4×.
  Validation: GPU pass times from Milestone 1.
  If wrong: the main pass dominates; consider MSAA or mesh/material choices.
- Open question: whether frame time is set by the simulation's critical path (serial octree build, system sync points), render extraction of 30,000 entities, or GPU/presentation.

## Outcomes & Retrospective

Delivered: a named limit. At 30,000 vehicles the app is CPU-bound on its critical path, not GPU-bound: the main world's simulation (15.6 ms, of which parallel flocking 9.3 ms and the serial octree build 1.4 ms) and, in parallel, the render world's mesh collection for 30,000 moving meshes (9.4 ms). Frame time at low machine load is about 18.8 ms (53 FPS); at 10,000 vehicles about 7.3 ms (138 FPS). No code changed.

Not measured: GPU execution time (unsupported timestamps; `xctrace` failed). The frame/CPU match makes a GPU limit unlikely but does not prove the GPU idle.

Follow-ups recorded in `plans/STATUS.md`: retune separation (would also cut neighbor counts and flocking cost at high density); optional instanced rendering if counts beyond 30,000 matter.

Lessons: record the load average with every timing; the same build measured 18.8 ms and 23–31 ms at load 5–8 and 13.

## Context and Orientation

- Previous measurements: `plans/completed/2026-10-30-profiling-guided-optimization.md` (frame times, thread table, group percentages).
- `src/vehicle.rs`: each vehicle is a `WorldAssetRoot` of `cone.glb#Scene0`, so it is a root entity plus a child mesh entity; `flock` builds the octree serially, then runs parallel queries; `movement` runs `flock` then a second parallel pass; `update` integrates in parallel.
- `src/main.rs`: `DefaultPlugins` (pipelined rendering), MSAA 4×, egui, mesh picking (target and camera only), FPS overlay.
- Profiling copy: `scratchpad/codex.ISlk/profile/` (disposable), built into the disposable `probe/target`.

## Plan of Work

1. Resync the profiling copy from `HEAD`; add `RenderDiagnosticsPlugin` and log its GPU/CPU pass times; build a second variant with `bevy/trace_chrome` and capture a short trace per count.
2. Summarize per frame: GPU time per pass and total, CPU time per render pass, and the main-world critical path (longest chain of systems per frame, with their durations).
3. Decide from the data; record in the Decision Log.

## Concrete Steps

Gate in the real repository (only if code changes), sequentially, with no other Cargo process or app running:

1. `CARGO_BUILD_JOBS=4 cargo fmt --all -- --check`
2. `CARGO_BUILD_JOBS=4 cargo test --release --locked`
3. `CARGO_BUILD_JOBS=4 cargo build --release --locked`
4. `python3 scripts/smoke-native.py`
5. `git diff --check`

## Validation and Acceptance

- GPU time per frame and per pass, and the main-world critical path, recorded for 10,000 and 30,000 vehicles with load averages.
- A named limit with evidence; any fix shows a before/after frame-time change at 30,000 and passes the gate.

## Idempotence and Recovery

Measurement does not touch the repository. Revert any fix commit to undo it.

## Artifacts and Notes

Measurements (2026-10-30; Apple M3 Pro, release, app-crate line tables, `AutoNoVsync` requested, 1280×720, defaults; disposable copy synced from `285e410`).

Variant A, render diagnostics (10 s after warm-up):

| Vehicles | Run | Median frame | p95 frame | FPS | Load 1/5/15 |
| --- | --- | --- | --- | --- | --- |
| 10,000 | 1 | 7.27 ms | 9.96 ms | 137.7 | 5.23 / 5.14 / 6.58 |
| 10,000 | 2 | 7.25 ms | 9.91 ms | 137.9 | 6.52 / 5.44 / 6.65 |
| 30,000 | 1 | 18.72 ms | 27.70 ms | 53.4 | 7.35 / 5.66 / 6.71 |
| 30,000 | 2 | 18.93 ms | 30.60 ms | 52.8 | 8.30 / 5.96 / 6.80 |

CPU command recording for all render passes: about 0.05 ms per frame at both counts (largest: clustering 0.016 ms, main opaque 0.014 ms). GPU time: not available.

Variant B, `trace_chrome` timeline (median per frame; spans overlap and include tracing overhead; frame 6.1 ms at 10,000 and 16.1 ms at 30,000):

| Span | 10,000 | 30,000 |
| --- | --- | --- |
| Main-world schedule | 5.71 ms | 15.64 ms |
| `movement` | 3.17 ms | 11.41 ms |
| Serial octree build | 0.45 ms | 1.42 ms |
| Parallel flocking | 2.52 ms | 9.29 ms |
| `PostUpdate` (incl. transform propagation 0.41 / 0.78 ms) | 1.64 ms | 2.37 ms |
| Render extraction | 0.33 ms | 0.39 ms |
| Render-world total | 4.44 ms | 13.09 ms |
| Mesh collection (inclusive) | 0.62 ms | 9.36 ms |
| Render graph and present | 0.92 ms | 1.07 ms |

The `trace_chrome` build took 250 s in the disposable target; the profiling copy and traces stay outside the repository.

## Interfaces and Dependencies

None in the repository. The disposable copy enabled the `bevy/trace_chrome` feature and `RenderDiagnosticsPlugin`.

---

Revision notes:

- 2026-10-30: Initial plan.
- 2026-10-30: Recorded measurements; limit named as CPU critical path; closed without code change.
