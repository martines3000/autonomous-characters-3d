# Profiling-guided optimization

This ExecPlan is a living document maintained according to `PLANS.md`.

## Purpose

The headless benchmark shows simulation is cheap up to 10,000 vehicles (3.7 ms per step in the default octree mode), but nobody has measured the running app: rendering, transform propagation, scene spawning and egui are outside the benchmark, and the count slider goes to 50,000. This work profiles the real app at the default count and at high counts, identifies what dominates frame time, and fixes only that, with before/after numbers. If nothing dominates enough to matter, the outcome is a recorded measurement and no code change.

## Progress

- [x] (2026-10-30 18:20Z) Plan drafted.
- [x] (2026-10-30 18:45Z) Milestone 1: profiled the running app at 200, 10,000 and 30,000 vehicles (Artifacts and Notes).
- [x] (2026-10-30 18:55Z) Milestone 2: fixes chosen (Decision Log): more compute threads, allocation- and sort-free neighbor queries, octree cell size checked by sweep.
- [x] (2026-10-30 19:20Z) Milestone 3: compute pool gets 9 threads; `for_each_in_radius` visitor replaces `query_radius`; octree size sweep kept the default; gate passed; app re-measured.
- [x] (2026-10-30 19:30Z) Plan closed and moved to `plans/completed/`. GPU-side measurement deferred (Outcomes).

## Surprises & Discoveries

- Observation: the app's simulation runs on 5 compute threads on this 11-core machine. Bevy's default task pools reserve 25% each (up to 4 threads) for IO and async compute, and both were idle (0 ms CPU) in every profile. The headless benchmark's `App` uses an 11-thread compute pool, so it overstates what the app gets.
  Evidence: thread table in the 30,000-vehicle profile: five `Compute Task Pool` threads at 77–78% of a core, IO and async pools at 0 ms.
- Observation: up to 10,000 vehicles the app holds 60 FPS (16.6 ms median frames) even with `PresentMode::AutoNoVsync`, so the presentation still waits for the display on this Mac; headroom below 60 FPS is not visible in frame time.
  Evidence: 200 and 10,000 runs at 16.68 and 16.59 ms median.
  Update: after the changes the 10,000-vehicle runs settled at 8.1–8.8 ms, close to a 120 Hz interval, so frame times appear quantized to the display's refresh; treat the 10,000 numbers as "one refresh interval instead of two", not as exact simulation savings.
- Observation: after the changes, the 30,000-vehicle frame no longer saturates the CPU: nine compute threads at 46–49% of a core, render thread 22%, main thread 4%, yet frames still take 23–31 ms. The remaining limit is outside the sampled CPU work (likely GPU or render synchronization; not measured).
  Evidence: after-change thread table below.
- Observation: summation order matters at the 1e-3 level. Without sorting, octree and naive accelerations differ by up to 0.0012 (on accelerations up to hundreds of units); the equivalence test's absolute tolerance went from 1e-4 to 1e-3, still far below the whole-unit errors a missed neighbor causes.
  Evidence: `octree_flocking_matches_naive_on_seeded_world` prints the maximum difference.
- Observation: rendering is not the bottleneck at 30,000 vehicles. Render extraction is 4.6% of process CPU, transform propagation 1.5%, visibility 0.9%; neighbor queries are 31% self time (65% inclusive on the busiest compute thread), sorting their results about 12%, `flocking_force` 12%.
  Evidence: 30,000-vehicle summary groups.

## Decision Log

- Decision: profile in a disposable copy with its own `CARGO_TARGET_DIR`, where the vehicle count is set by an environment variable, VSync is off, and frame times are logged; do not add those knobs to the app.
  Rationale: CUA clicks do not reach egui reliably on this host, so the slider cannot be driven; profiling-only switches would be configuration surface nobody else needs. VSync off exposes real frame cost instead of the display cap.
  Date: 2026-10-30.
- Decision: give the compute pool the idle threads by setting the IO and async-compute pools to one thread each via `TaskPoolPlugin`.
  Rationale: the app loads two assets and runs no async work, so those pools idle while simulation is limited to 5 of 11 cores; this is a configuration change with no algorithmic risk.
  Date: 2026-10-30.
- Decision: make the neighbor query allocation- and sort-free (visit neighbors through a callback) instead of returning a sorted `Vec` per vehicle.
  Rationale: sorting and allocation are about 16% of the busiest thread's CPU. Sorting only kept octree summation order identical to naive mode; a different order changes sums by rounding only, which the equivalence test's tolerance covers.
  Date: 2026-10-30.
- Decision: check the default `octree_size` (100) with a disposable benchmark sweep; change it only for a clear win.
  Rationale: with a 30-unit query radius, 100-point leaves make each query test many points outside the radius; smaller leaves trade that for more nodes. The answer is empirical.
  Date: 2026-10-30.
- Decision: use `samply` (installed, 0.13.1) for CPU sampling, with line tables only for the app crate.
  Rationale: already the documented profiler; per-package debug info avoids rebuilding Bevy with symbols. Bevy function names remain available without line tables.
  Date: 2026-10-30.

## Assumptions and Open Questions

- Assumption: CPU-side costs are visible to `samply`; GPU time is not.
  Validation: compare frame time with summed CPU time on the main and render threads.
  If wrong: record GPU-bound cases as such; GPU profiling is out of scope.

## Outcomes & Retrospective

Delivered: the app's simulation now uses 9 compute threads instead of 5, and neighbor queries no longer sort or allocate. At 10,000 vehicles the running app went from about 60 FPS (16.6 ms) to about 115–124 FPS (8.1–8.8 ms median); at 30,000 the CPU is no longer saturated, sorting and allocation fell from 12.5% and 20% of process CPU to near zero, but frame time is unchanged within noise (23–31 ms against 27.8 ms, with the machine three times as loaded). The octree size sweep (8–100) found the default 100 best or tied, so it stays.

Deferred: what limits 30,000 vehicles now. CPU sampling cannot see GPU time; a Metal capture or Instruments GPU trace would answer it. Recorded as an optional future task in `plans/STATUS.md`. Counts above 30,000 are unmeasured.

Lessons: profiling the real app found what the headless benchmark could not (the 5-thread compute pool). A benchmark's `App` is not the app's thread configuration.

## Context and Orientation

- `src/vehicle.rs`: `vehicle_spawner` spawns each vehicle as a `WorldAssetRoot` of `cone.glb#Scene0` (a scene instance with its own child entities) plus steering components; `movement`, `update` and `flock` run each frame.
- `src/main.rs`: `DefaultPlugins`, egui, mesh picking (only the target and camera opt in), `FrameTimeDiagnosticsPlugin` with an FPS overlay.
- Baseline profile from the old engine version: `target/native-baseline/` (local, ignored; not comparable after the Bevy upgrade).

## Plan of Work

1. Disposable profiling copy: `AC3D_VEHICLES` read in `configure_global_state`, `PresentMode::AutoNoVsync`, frame-time logging (median and p95 after warm-up), and per-package line tables for the app crate.
2. For 200, 10,000 and 30,000 vehicles: one plain run for frame times, then one `samply record` run; summarize per-thread CPU and top self-time functions grouped by system or engine stage.
3. Choose the fix with the largest measured share that the app controls; implement it in the real repository; re-measure the same way.

## Concrete Steps

Gate from the repository root, sequentially, with no other Cargo process running and no other app instance:

1. `CARGO_BUILD_JOBS=4 cargo fmt --all -- --check`
2. `CARGO_BUILD_JOBS=4 cargo test --release --locked`
3. `CARGO_BUILD_JOBS=4 cargo build --release --locked`
4. `python3 scripts/smoke-native.py`
5. `git diff --check`

## Validation and Acceptance

- Frame-time and profile summaries recorded for all three counts, with machine, build and load average.
- Any optimization shows a before/after frame-time improvement at the count where it matters, and does not change simulation results (existing tests, including naive/octree equivalence, pass).

## Idempotence and Recovery

Profiling is read-only for the repository. Revert the optimization commit to undo it.

## Artifacts and Notes

Profile, before changes (2026-10-30; Apple M3 Pro, macOS 26.7.1, release with app-crate line tables, `AutoNoVsync` requested, 1280×720, default settings, unseeded spawn; 10 s after a 5 s warm-up; load averages 3.9–4.3):

| Vehicles | Median frame | p95 frame | Median FPS |
| --- | --- | --- | --- |
| 200 | 16.68 ms | 18.61 ms | 60.0 |
| 10,000 | 16.59 ms | 19.69 ms | 60.3 |
| 30,000 | 27.77 ms | 51.05 ms | 36.0 |

At 30,000: five compute threads busy 77–78% each, render thread 38%, main thread 3%; neighbor queries 31% of all process CPU self time. Profiles and summaries are in the disposable workspace (`scratchpad/codex.ISlk/profile/`), not the repository.

After the changes (same method and build settings; three plain runs per count; load averages 12.7–13.7, about three times the earlier runs):

| Vehicles | Run 1 | Run 2 | Run 3 | Before |
| --- | --- | --- | --- | --- |
| 10,000 median / p95 | 8.80 / 12.86 ms | 8.40 / 13.22 ms | 8.10 / 11.74 ms | 16.59 / 19.69 ms |
| 30,000 median / p95 | 23.40 / 44.42 ms | 30.24 / 54.77 ms | 30.92 / 58.19 ms | 27.77 / 51.05 ms |

At 30,000 after the changes (sampled run, 26.5 ms median): nine compute threads at 46–49% of a core each, render thread 22%, main thread 4%. Process CPU: simulation 59% (was 78%), wait stacks 28% (was 11%), render extraction 5%, allocator 0.6% (was 20%), sorting 0% (was 12.5%). Busiest compute thread: `for_each_in_radius` 32% self, `flocking_force` 27%.

Octree size sweep (disposable copy, octree mode, 11-thread headless benchmark, median of three run medians, load averages 11–12, so treat as rough): 10,000 vehicles: 8 → 6.15 ms, 16 → 4.84, 32 → 4.20, 64 → 3.92, 100 → 3.27; 30,000 vehicles: 8 → 20.81, 16 → 14.85, 32 → 14.62, 64 → 13.35, 100 → 12.73.

Gate (2026-10-30, sequential, `CARGO_BUILD_JOBS=4`): `cargo fmt --all -- --check` clean; `cargo test --release --locked` 23 passed, 1 ignored; `cargo build --release --locked` with no warnings; `python3 scripts/smoke-native.py` PASS; `git diff --check` clean; Clippy 8 pre-existing warnings.

## Interfaces and Dependencies

- `src/main.rs`: `TaskPoolPlugin` with IO and async-compute pools fixed at one thread each.
- `src/octree.rs`: `query_radius(&self, &Point, f32) -> Vec<&T>` replaced by `for_each_in_radius(&self, point: &Point, radius: f32, visit: impl FnMut(&T))`; traversal uses a fixed `[usize; MAX_DEPTH * 7 + 1]` stack.
- `src/vehicle.rs`: `flocking_force` takes `neighbors: impl FnOnce(&mut dyn FnMut(Vector3<f32>, Vector3<f32>))`, shared by naive and octree paths.
- No new crates.

---

Revision notes:

- 2026-10-30: Initial plan.
- 2026-10-30: Recorded profiles, decisions, sweep, before/after frame times; closed with GPU measurement deferred.
- 2026-10-30: Correction from `2026-10-30-high-count-frame-limit.md`: the after-change 30,000-vehicle runs above were taken at load average 13; at load 5–8 the same build measures 18.7–18.9 ms (about 53 FPS), a clear improvement over 27.8 ms. The app is CPU-bound on its critical path there, not GPU-bound.
