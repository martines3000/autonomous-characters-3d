# Push inward only outside the walls and rebalance edited groups

This ExecPlan is a living document maintained according to `PLANS.md`.

## Purpose

Remove in-bounds wall drag without changing force caps, defaults or navigation gating. Reassign group membership from stable spawn IDs once per group-count edit, including re-enabling. Both backends share the laws.

## Progress

- [x] (2026-11-01 01:23Z) Read guidance and trace both CPU/GPU paths; plan drafted before implementation.
- [x] (2026-11-01 01:26Z) Two baseline initial-world-seeded CPU Octree/GPU measurements at 200 and 20k.
- [x] (2026-11-01 01:27Z) Shared outside-only wall law, ID-based group reassignment and CPU/device regression fixtures implemented.
- [x] (2026-11-01 01:30Z) Two post-change measurements and the single requested 100k/1M wander/follow benchmark recorded.
- [x] (2026-11-01 01:34Z) Formatting, 124 release tests, warning-free release build, Clippy baseline seven, single 13-test GPU gate and diff check pass; documentation completed.

## Surprises & Discoveries

- Current default count is 20,000, so the requested 20k/default cases coincide. Also measure the historical 200-vehicle density reference. CPU wander angles use thread RNG, as in the existing seeded benchmark; the initial world is seeded. GPU wander uses frame/index PCG and sorting can vary traversal order.

- The old adversarial cap fixture used arbitrary groups inconsistent with spawn IDs. Its IDs now preserve the same two rare-group members and 68 foreign neighbors under the new spawn rule; the cap regression remains adversarial.
- Removing wall drag raises settled speed from roughly 29 to 39 units/s without changing the configured wander speed of 40. Largest measured post-fix excursion is 23.399 world units, compared with 17.859 in baseline runs.

## Decision Log

- Decision: keep desired velocity equal to current velocity on in-bounds axes; outside axes desire inward max speed. Skip wall force entirely when fully inside.
  Rationale: owner-selected option (a), preserving cap and seek/formation gating without retuning.
  Date: 2026-11-01.
- Decision: derive edited enabled membership with the existing spawn rule, ID modulo count; disabling preserves latent metadata and renders group zero.
  Rationale: deterministic balanced assignment for dense IDs, no per-frame maintenance or disabled-path tax.
  Date: 2026-11-01.

## Assumptions and Open Questions

- The existing GPU containment fixture remains below 50 world units outside. Validate unchanged assertion in the single final device suite; report any failure without adjusting defaults.
- Sixty simulated seconds with the last ten seconds averaged is sufficient for a settled-speed measurement. Record window means and peak excursion over every step; report nonstationary behavior rather than claim exact equilibrium.

## Outcomes & Retrospective

Both fixes are implemented. Sixty-second repeat measurements show settled speeds near the configured wander speed while preserving soft containment without tuning. Group edits use carried spawn IDs once per change; disabled steady-state group work is unchanged. All requested gates pass: 124 release tests, the single final 13-test GPU suite, formatting, a warning-free release build, Clippy seven pre-existing warnings (repeated for the test target), and diff check. The unchanged `gpu_flock_spawns_grows_and_stays_bounded` assertion remains below 50. Wall regression checks exact zero/boundary behavior, one-axis isolation and force capping. CPU membership tests exercise 2→5→3→Off→5 and verify steady frames do not mark metadata changed; the GPU lifecycle fixture compares actual CPU-maintained records through 2→5→3→Off→2→8, paused growth and sorting. No requested implementation gap remains; optional speed retuning stays owner-by-eye. No app runs were requested or performed. Changes remain uncommitted for owner review.

## Context and Orientation

`vehicle::movement` applies wall force on the wander wall branch; `gpu_flock.wgsl::steer` mirrors it. `vehicle::update_membership` and WGSL `update_groups` maintain carried metadata once per count edit. Formation slot maintenance already follows group maintenance.

## Plan of Work

Add a reusable ignored measurement probe before changing production behavior. Measure seeded initial worlds at 200 and 20k in CPU Octree and GPU for 3,600 fixed 1/60 s steps. Then patch only wall desired components and group maintenance. Extend existing lifecycle fixtures and add a focused wall-force unit test. Update docs and run required gates sequentially.

## Concrete Steps

From the repository root, sequentially:

Before every Cargo command: `pgrep -a cargo; pgrep -a rustc; pgrep -a autonomous`; wait if any are running. Use `PATH="$HOME/.cargo/bin:$PATH"` and `CARGO_BUILD_JOBS=4`. Never overlap invocations. Record `uptime` before measurements.

```sh
# Repeat twice before changing production code, then twice after.
CARGO_BUILD_JOBS=4 cargo test --release --locked vehicle::tests::wall_containment_probe -- --exact --ignored --nocapture --test-threads=1
# One requested post-change GPU benchmark; all optional mechanics Off.
GPU_BENCH_COUNTS=100000,1000000 GPU_BENCH_CAPS=16 GPU_BENCH_MODES=wander,follow CARGO_BUILD_JOBS=4 cargo test --release --locked vehicle::tests::gpu_flocking_benchmark -- --exact --ignored --nocapture --test-threads=1
CARGO_BUILD_JOBS=4 cargo fmt --all -- --check
CARGO_BUILD_JOBS=4 cargo test --release --locked
CARGO_BUILD_JOBS=4 cargo build --release --locked
CARGO_BUILD_JOBS=4 cargo clippy --release --locked --all-targets
# Run once, after all implementation edits.
CARGO_BUILD_JOBS=4 cargo test --release --locked gpu_ -- --ignored --nocapture --test-threads=1 --skip gpu_flocking_benchmark
git diff --check
```

## Validation and Acceptance

Wall force is exactly zero in bounds and changes only outside components; its magnitude retains the existing cap. CPU/GPU parity and unchanged <50 containment assertions pass. Group edits 2→5→3 and Off→On match spawn-rule assignment, preserve other metadata/motion/phase, and run maintenance only on edits. Disabled steady-state group output/cost is unchanged.

## Idempotence and Recovery

Measurement and test invocations are read-only. Check cargo/rustc/autonomous before each invocation and never overlap them. All edits remain uncommitted for owner review; do not reset or discard work.

## Artifacts and Notes

Full logs and parsed tables: `/tmp/wall-containment-2026-11-01/`. Baseline HEAD `189f67d`, branch
`feat/boid-camera-and-fixes`. The baseline includes only the new test probe; production Rust/WGSL
were unchanged until both baseline runs completed. Release profile, Rust 1.99.1 / Bevy 0.20.1,
AMD Ryzen 9 7900X (24 logical CPUs), RX 7900 XT, native Linux. No other Cargo/rustc/flock app
ran concurrently. Uptime and process guards were checked before each run.

Probe settings are every unchanged default, with all optional mechanics Off and no following.
CPU mode is Octree with nine compute threads. Both backends start from identical positions and
velocities generated by `StdRng(0xB01D5)`: a count-scaled spawn cube, unit-sphere headings × max
speed 80, zero initial wander angles. CPU retains native thread-RNG wander perturbations; only its
initial world is seeded (the same convention as the existing seeded CPU benchmark). GPU uses its
frame/index PCG; atomic scatter order can alter trajectories. These are statistical repeats, not
bitwise trajectory replay. No settings were retuned for the probe.

Each run advances 3,600 fixed 1/60 s steps. Settled speed is the arithmetic mean of every vehicle's
speed over steps 3,001–3,600 (simulated seconds 50–60). Peak overshoot is the largest positive
axis distance beyond the walls over **all** vehicles at **every** step, 0–60 s. The probe prints
30–40 / 40–50 / 50–60 s speed windows as a stationarity check. The final two windows differ by
less than 0.29 units/s in these runs. Current default count is 20k, so its requested default and 20k
cases are the same; 200 additionally covers the historical density reference. Speeds are world
units per simulated second; excursions are world units.

| Backend / count | Before speed 1 / 2 | After speed 1 / 2 | Before peak overshoot 1 / 2 | After peak overshoot 1 / 2 |
| --- | --- | --- | --- | --- |
| octree / 200 | 29.148 / 29.126 | 38.779 / 38.848 | 14.641 / 12.597 | 15.015 / 15.869 |
| gpu / 200 | 29.160 / 29.188 | 38.800 / 38.766 | 12.211 / 11.872 | 15.458 / 16.008 |
| octree / 20,000 | 29.308 / 29.332 | 39.210 / 39.206 | 17.369 / 17.859 | 20.621 / 20.960 |
| gpu / 20,000 | 29.315 / 29.296 | 39.145 / 39.213 | 17.559 / 16.805 | 20.179 / 23.399 |

The maximum observed post-change overshoot is 23.399, below 50; this is finite-scenario evidence,
not a hard collision guarantee. Other retained forces still act, so mean speed need not equal 40.
The unchanged existing GPU containment assertion is verified separately in the final device gate.

The requested post-change GPU benchmark ran once, 600 warmup + 60 timed steps (11 simulated
seconds), dt=1/60, PCG spawn frame zero, no optional mechanics. Configured cap16 is inactive in
wander and active in follow. Timings include the existing synchronization/step harness; do not
infer a speedup from differences against historical density trajectories.

| Count / navigation | Median ms | p95 ms | Mean speed after 11 simulated seconds |
| --- | --- | --- | --- |
| 100,000 / wander | 0.604 | 0.652 | 39.411 |
| 100,000 / follow | 3.655 | 5.325 | 24.441 |
| 1,000,000 / wander | 4.571 | 5.001 | 39.478 |
| 1,000,000 / follow | 7.947 | 8.891 | 47.628 |

Recorded uptime (local CET):

- `before-1`: 02:24:33 up  8:05,  1 user,  load average: 0.60, 1.27, 1.87
- `before-2`: 02:25:41 up  8:06,  1 user,  load average: 1.35, 1.42, 1.88
- `after-1`: 02:27:52 up  8:08,  1 user,  load average: 1.05, 1.35, 1.80
- `after-2`: 02:28:34 up  8:09,  1 user,  load average: 1.81, 1.56, 1.86
- `gpu-benchmark`: 02:29:34 up  8:10,  1 user,  load average: 1.99, 1.74, 1.90

Final verification logs: `tests-final.log` (124 passed, 16 ignored), `gpu-tests.log` (13 passed),
`build.log` (no warnings), `clippy.log` (seven existing warnings, seven test-target duplicates),
`fmt.log` (success with no output). Sixteen ignored tests comprise 13 device regressions, the
existing CPU/GPU benchmarks and the new wall measurement probe. No native app/smoke run was made,
as requested. HEAD remains `189f67d`; all changes are uncommitted. Process checks after the gate
show no cargo, rustc or autonomous app. CPU native wander remains stochastic after seeded spawning;
soft steering is not a hard box constraint, and measurements do not prove every possible trajectory
or timestep. These are retained simulation properties, not missing fixes.

## Interfaces and Dependencies

No dependency, buffer layout, resource, default or rendering changes. Private `vehicle::wall_force` takes world position, velocity, bounds, max speed and max force. Test-only `WallProbeStats` / `wall_containment_probe` provide repeatable initial-world measurements. Existing CPU/GPU maintenance triggers and packed metadata layouts are retained.

---

Revision notes:

- 2026-11-01: Initial owner-authorized containment/rebalance follow-up plan.

- 2026-11-01: Both owner-authorized fixes implemented and measured; all gates pass, plan moved to completed for owner review.
