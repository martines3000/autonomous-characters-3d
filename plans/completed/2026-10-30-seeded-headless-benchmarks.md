# Seeded headless benchmarks with equivalent workloads

This ExecPlan is a living document maintained according to `PLANS.md`.

## Purpose

The only performance evidence is `meritve.png`, a historical chart that compared naive mode with an octree that did less work (single-leaf neighbor lookups), with no seed, machine or build details. The in-app sweep that produced it is disabled, and its "Benchmark mode" checkbox now only hides vehicles spawned after it is ticked. Naive and octree modes now compute identical flocking, so a fair comparison is possible.

After this work, one command runs a headless, seeded benchmark of the real steering and integration systems for both modes on identical inputs at several vehicle counts, and prints per-step timings. Its results decide the default mode, and they are recorded with scenario, seed, build and machine.

## Progress

- [x] (2026-10-30 16:35Z) Plan drafted; F09 and F10 confirmed in current source (`benchmark` unregistered; checkbox affects only new spawns; naive is the default).
- [x] (2026-10-30 16:55Z) Milestone 1: ignored test `flocking_benchmark`; `benchmark` system, four `GlobalState` fields, the checkbox and the spawn visibility branch removed.
- [x] (2026-10-30 17:00Z) Milestone 2: diff reviewed; gate passed.
- [x] (2026-10-30 17:10Z) Milestone 3: three runs, one discarded for machine contention; results recorded; naive stays the default.
- [x] (2026-10-30 17:15Z) Plan closed and moved to `plans/completed/`.

## Surprises & Discoveries

- Observation: the machine is shared, and contention roughly doubles timings. One run measured 2–7× slower (for example naive at 5,000 vehicles: 20.4 ms against 6.9 ms) while the 1-minute load average was about 20 from other desktop apps.
  Evidence: run 1 versus runs 2 and 3; `uptime` after run 2. Check the load average before trusting a run, and repeat runs.
- Observation: with the app's spawn rule the octree barely helps, because density grows with count: at 10,000 vehicles in a 100-unit cube each vehicle has over a thousand neighbors within 30 units, so the work per vehicle is the neighbor list itself.
  Evidence: "app" workload rows below; octree is slower up to 1,000 vehicles and only about 1.1–1.3× faster at 10,000.
- Observation: at constant density the octree scales as intended: 1.5× faster at 2,000 vehicles, 3.6× at 5,000, 6.7× at 10,000.
  Evidence: "density" workload rows below.

## Decision Log

- Decision: implement the benchmark as an `#[ignore]` test in `src/vehicle.rs`, run with `cargo test --release --locked <name> -- --ignored --nocapture --test-threads=1`.
  Rationale: no new dependency (Criterion would change `Cargo.toml`/`Cargo.lock` and its statistics are not needed for differences this large); the test module can drive the private systems directly; release test builds use the same optimization profile as the app.
  Date: 2026-10-30.
- Decision: time `movement` + `update` (the whole per-frame simulation cost, including octree rebuild) in wander mode with a fixed 1/60 s step, after a warm-up, and report median and 95th-percentile step time.
  Rationale: this is what a frame pays for simulation; wander keeps the flock moving like the default app state. Wander randomness stays unseeded; its effect on work per step is small and shows up in the percentile spread.
  Date: 2026-10-30.
- Decision: two workloads per count, both seeded and shared between modes: "app" (the app's spawn rule: a 100-unit cube regardless of count) and "constant density" (cube edge scaled so density matches 200 vehicles in 100 units).
  Rationale: "app" is what the count slider does today, where density grows with count; "constant density" shows spatial-index scaling and informs a future spawn rule.
  Date: 2026-10-30.
- Decision: keep naive mode as the default.
  Rationale: at the default 200 vehicles naive costs about 0.08 ms per step against 0.14 ms for octree; both are far below a 16.7 ms frame. With the current spawn rule octree only wins from roughly 10,000 vehicles.
  Date: 2026-10-30.
- Decision: remove the unused `benchmark` system, the benchmark fields in `GlobalState` and the "Benchmark mode" checkbox.
  Rationale: the headless benchmark replaces the sweep; the checkbox's only effect was hiding later spawns, which misled (F09). Removing is smaller than repairing it.
  Date: 2026-10-30.

## Assumptions and Open Questions

- Assumption: Bevy's compute task pool behaves the same in a test `App` as in the app, so parallel queries use all cores.
  Why reasonable: existing tests already run `par_iter_mut` systems in a bare `App`.
  Validation: report the thread count the benchmark sees; compare one count against the app's FPS overlay if needed.
  If wrong: initialize the task pool explicitly in the harness.
- Open question: the default mode. Answered by Milestone 3.

## Outcomes & Retrospective

Delivered: a seeded, headless benchmark of the real simulation systems for both modes on identical inputs, runnable with one command in about 10 seconds; the misleading in-app benchmark pieces are gone (F09 resolved), and the release build now has no compiler warnings. Naive stays the default on measured evidence.

F10 (quadratic cost up to the 50,000 slider) is now measured rather than resolved: naive simulation costs about 26 ms per step at 10,000 vehicles, beyond a 60 FPS budget before any rendering. The effective fix is a spawn rule that keeps density constant (so the octree's 6.7× advantage at 10,000 applies), possibly with automatic octree selection above ~2,000 vehicles; recorded as a future task in `plans/STATUS.md`. Counts above 10,000 were not measured.

Not measured: rendering cost (the benchmark is headless), and the effect of `octree_size` (fixed at the default 100).

Lessons: on a shared machine, repeat runs and record the load average; a single run can be off by 2× without any visible error.

## Context and Orientation

- `src/vehicle.rs`: `movement` (steering, calls `flock`), `update` (integration), `flock` (naive scan or octree `query_radius`), spawn rule in `vehicle_spawner` (`SPAWN_SIZE` cube, `random_direction` headings). Existing tests show how to build a minimal `App`, spawn vehicles directly and advance `Time` (`degenerate_vehicles_stay_finite_and_within_max_speed`, `octree_flocking_matches_naive_on_seeded_world`). Unused `benchmark` system near the integration code.
- `src/main.rs`: `GlobalState` has `benchmark_mode`, `benchmark_step`, `benchmark_results`, `benchmark_current_results`; the menu has a "Benchmark mode" checkbox; `vehicle_spawner` reads `benchmark_mode` for visibility.
- Machine for recorded results: Apple M3 Pro, 11 cores, 18 GiB, macOS ARM64, Rust 1.99.1.

## Plan of Work

1. Ignored test `flocking_benchmark` in `src/vehicle.rs`: for each workload and count (200, 1,000, 2,000, 5,000, 10,000) and each mode, build an `App` with defaults, spawn the same seeded vehicles, run warm-up steps, then time each `app.update()` of `(movement, update)`; print one line per case with median and p95 step time in ms, octree size and thread count. Keep it readable; no framework.
2. Remove the in-app benchmark pieces listed in the Decision Log.
3. Gate, then run the benchmark once (owner), record results, decide the default.

## Concrete Steps

From the repository root, sequentially, with no other Cargo process running:

1. `CARGO_BUILD_JOBS=4 cargo fmt --all -- --check`
2. `CARGO_BUILD_JOBS=4 cargo test --release --locked`
3. `CARGO_BUILD_JOBS=4 cargo build --release --locked`
4. `python3 scripts/smoke-native.py`
5. `git diff --check`
6. Benchmark (nothing else running, app closed): `CARGO_BUILD_JOBS=4 cargo test --release --locked flocking_benchmark -- --ignored --nocapture --test-threads=1`

## Validation and Acceptance

- `cargo test` stays fast; the benchmark runs only with `--ignored`.
- Both modes receive identical seeded inputs per case.
- The benchmark prints one line per (workload, count, mode) and finishes in a few minutes.
- Results and the default-mode decision are recorded here and in `README.md`, with the historical chart labeled as such.

## Idempotence and Recovery

The benchmark only reads; rerun it any time. Revert the commit to restore the in-app checkbox.

## Artifacts and Notes

Gate (2026-10-30, sequential, `CARGO_BUILD_JOBS=4`): `cargo fmt --all -- --check` clean; `cargo test --release --locked` 22 passed, 1 ignored; `cargo build --release --locked` with no warnings; `python3 scripts/smoke-native.py` PASS; `git diff --check` clean; Clippy 8 warnings, all pre-existing.

Benchmark (2026-10-30): Apple M3 Pro, 11 compute threads, 18 GiB, macOS ARM64, Rust 1.99.1, release profile; seed `0xB01D5`; default settings (`octree_size` 100); wander mode; fixed 1/60 s step; 10 warm-up steps, 60 timed steps; per-step time of `(movement, update)`. Median ms from run 3, with run 2 in parentheses (run 1 discarded for contention):

| Vehicles | App spawn: naive | App spawn: octree | Constant density: naive | Constant density: octree |
| --- | --- | --- | --- | --- |
| 200 | 0.08 (0.09) | 0.14 (0.16) | 0.08 (0.08) | 0.14 (0.13) |
| 1,000 | 0.45 (0.70) | 0.70 (0.90) | 0.42 (0.42) | 0.45 (0.38) |
| 2,000 | 1.95 (2.89) | 1.68 (4.90) | 1.26 (1.18) | 0.86 (0.69) |
| 5,000 | 6.93 (10.87) | 7.55 (9.32) | 6.53 (6.62) | 1.84 (1.69) |
| 10,000 | 26.96 (29.48) | 23.61 (23.26) | 25.61 (25.57) | 3.77 (3.81) |

"App spawn" places every count in the 100-unit cube, as the app does. "Constant density" scales the cube edge by (count / 200)^(1/3). Command:

    CARGO_BUILD_JOBS=4 cargo test --release --locked flocking_benchmark -- --ignored --nocapture --test-threads=1

## Interfaces and Dependencies

No new crates. `GlobalState` lost `benchmark_mode`, `benchmark_step`, `benchmark_results` and `benchmark_current_results`; the menu lost the "Benchmark mode" checkbox; `vehicle_spawner` always uses `Visibility::Inherited`. New ignored test `vehicle::tests::flocking_benchmark`.

---

Revision notes:

- 2026-10-30: Initial plan.
- 2026-10-30: Recorded harness, results, contention finding and default decision; closed.
