# Measure GPU hunter cost

This ExecPlan is maintained according to `PLANS.md`. The user requested this completed-path filename; the work ran on 2026-11-02.

## Purpose

Measure 0/16 hunters at 20k, 100k, 300k and 1M prey, wander/follow, cap 16. Add a benchmark control and bound large-flock pursuit when overhead exceeds about 0.3 ms. Preserve the disabled GPU path.

## Progress

- [x] (2026-11-02 17:00Z) Read repository rules, inspect benchmark and hunter dispatch, write initial plan.
- [x] (2026-11-02 17:01Z) Capture repeated 1M no-hunter baseline before edits.
- [x] (2026-11-02 17:04Z) Add benchmark option and complete all 16 original-search cases twice.
- [x] (2026-11-02 17:14Z) Implement bounded grid query, preserve small-flock scan, prove target equivalence and unchanged parity.
- [x] (2026-11-02 17:19Z) Final gates pass: 203 release tests, 20 GPU tests, warning-free build, seven baseline Clippy warnings, scripted smoke.
- [x] (2026-11-02 17:22Z) Repeat every final scenario twice; finish measurement table, README and STATUS.
- [x] (2026-11-02 18:25Z) Orchestrator review: dropped the grid search (no measured benefit) and isolated the overhead against two groups without hunters.

## Surprises & Discoveries

- Hunters originally ran before grid construction. Reusing that grid would query stale positions; pursuit now runs after the current coarse scatter and before integration.
- Radius scales with world size. Enumerating the entire hunt-radius box is expensive; expanding cubes and nearest-distance pruning avoid most empty cells. A 3D cursor avoids overflowing a flattened cell-count product.
- A single lane per bucket regressed dense cases. Eight buckets now run concurrently, with 32 lanes sharing each bucket.
- Always using the grid regressed 20k follow from 1.118/1.523 ms to 3.003/3.302 ms and 100k follow from 1.273/1.122 ms to 2.705/2.598 ms. Keep a full scan through 100k prey; large flocks use cells within the hunt sphere.
- Total overhead remains large in 1M wander. These measurements include encode/submit/wait, group steering and changed prey distributions; they cannot isolate pursuit cost. Seeded spawning does not make atomic scatter or evolved trajectories bit-identical.

## Decision Log

- Decision: Measure first and serialize every Cargo/app command with the shared lock.
  Rationale: The explicit task override authorizes benchmarks. Correctness, unchanged defaults and simplicity precede performance.
  Date: 2026-11-02.
- Decision: Reuse current sorted prey, counts, starts, block sums and output order map in the optional hunter pass.
  Rationale: Exact current-position pursuit needs no extra grid dispatch or buffer. Stable-ID ties remain independent of sorting.
  Date: 2026-11-02.
- Decision: Limit the full-scan path to at most 100k prey; use the spatial query above that threshold.
  Rationale: Repeated smaller-case measurements reject an unconditional grid query. This measured fallback preserves preset performance while eliminating the million-prey scan.
  Date: 2026-11-02.
- Decision: Do not change prey steering or simulation defaults to chase remaining total overhead.
  Rationale: The task requests pursuit bounding. Total step measurements do not establish the remaining bottleneck.
  Date: 2026-11-02.

## Assumptions and Open Questions

- This RX 7900 XT result is hardware-specific. Check uptime and repeat on another GPU before generalizing.
- Hunt-radius cells and nearest-distance pruning bound the search region, not milliseconds or worst-case occupancy. Dense/colliding buckets and empty nearby space can still be costly.
- Remaining 1M wander overhead needs separate pass-level profiling. Group/panic work and altered flock geometry are plausible contributors, not measured causes.

## Outcomes & Retrospective

Shipped: the `GPU_BENCH_HUNTERS=0..16` benchmark option and the measurements below. Not shipped: the bounded grid pursuit search. At 1M wander with 16 hunters it measured 31.4/30.3 ms against 32.2/34.6 ms for the full scan, within run-to-run noise, and it regressed 20k–100k follow unless a 100k crossover kept the full scan. It added a second search path and three bindings for no measured gain, so the orchestrator reverted `hunter.wgsl` and `gpu_flock.rs` to master.

Isolation run by the orchestrator (idle machine, load 0.87, 1M wander, cap 16, two runs each, median/p95 ms): two groups without hunters 5.344/5.508 and 5.393/5.608; two groups with 16 hunters, original full scan, 31.611/32.773 and 35.261/37.177. Group steering costs about 1 ms over the single-group 4.4 ms; hunters add about 26–30 ms. Follow (cap 16) shows no hunter overhead, and presets at 30–40k cost about 1 ms extra. The cause is not the pursuit search. Untested hypothesis: panic packs prey densely and wander's uncapped neighbor loop grows with density. Next step is per-pass GPU timing; recorded as a future task in `plans/STATUS.md`.

The sections below describe the worker's grid-search attempt; its code is not on master.

## Context and Orientation

`src/vehicle.rs::tests::gpu_flocking_benchmark` starts fresh shader PCG spawning at frame zero, warms 600 substeps and times 60 at dt=1/60. `src/gpu_flock.rs::GpuFlock::encode` builds the current coarse grid, optionally hunts, then steers prey. Paused hunters still update instances without searching. `assets/shaders/hunter.wgsl::hunt` performs exact distance/lowest-spawn-ID reduction. Its small full scan and large expanding query share pursuit/pass/recovery integration.

## Plan of Work

Completed: capture baseline; add validated benchmark control; measure all sizes/modes twice; implement only the optional pursuit change justified by the threshold; reject the slower unconditional grid variant; verify parity, defaults and final measurements; update README and STATUS.

## Concrete Steps

From the repository root, sequentially, with `CARGO_BUILD_JOBS=4`:

1. Under `flock /tmp/ac3d-heavy.lock`, run `uptime` before each benchmark batch.
2. Run `flock /tmp/ac3d-heavy.lock env CARGO_BUILD_JOBS=4 GPU_BENCH_COUNTS=20000,100000,300000,1000000 GPU_BENCH_CAPS=16 GPU_BENCH_MODES=wander,follow GPU_BENCH_HUNTERS=16 cargo test --release --locked gpu_flocking_benchmark -- --ignored --nocapture --test-threads=1`; repeat with 0, then repeat both batches.
3. Run each under the same shared lock: `cargo fmt --all -- --check`, `cargo test --release --locked`, `cargo build --release --locked`, `cargo clippy --release --locked --all-targets`, `cargo test --release --locked gpu_ -- --ignored --test-threads=1 --skip gpu_flocking_benchmark`.
4. Run `flock /tmp/ac3d-heavy.lock env CARGO_BUILD_JOBS=4 AC3D_SCRIPT=/tmp/ac3d-hunter-scaling/smoke.txt python3 scripts/smoke-native.py`, then `git diff --check`. The smoke script's scenario ends with `12 exit`.

## Validation and Acceptance

All requested original and final benchmark cases complete twice. All gates above pass, including unchanged `gpu_hunters_and_panic_match_cpu_reference`. The new `gpu_bounded_hunter_targets_match_full_scan` compares selected IDs with both a full GPU scan and CPU nearest-distance/ID reference on seed `0x6F0C`: 4096 reordered prey, 16 hunters, both order-map paths (caps 0/16), ties in separate cells, negative coordinates/boundaries, no target and zero distance. It forces the grid below the production cutoff; only test pipelines expose selected IDs in unused instance phase.w. Growth, recovery, paused history and disabled allocation round trips retain existing coverage. Native smoke proves startup only.

## Idempotence and Recovery

Every benchmark creates fresh GPU state. Rerun gates and measurements under the shared lock; no concurrent app/build/test is needed. Optional hunter bindings reuse existing buffers. Review or reverse this patch separately if needed; no reset, discard, commit, push or merge was performed.

## Artifacts and Notes

Machine: Ryzen 9 7900X (24 logical CPUs), Radeon RX 7900 XT, RADV Vulkan, Mesa/vulkan-radeon 26.2.5. Release/locked Rust build; no render window during benchmarks. PCG spawn seed is `pcg(index ^ pcg(0))`; fresh frame 0 per case, dt=1/60, 600 warmup + 60 measured steps. All cases complete without the 250 ms abort. Requested cap 16 becomes effective cap 0 for wander by existing design and 16 for follow. Hunters use one group (group 1), default size/speed 1; prey count excludes hunters. Other benchmark options stay off/default.

Each table cell is run 1/run 2 median milliseconds. Before uses the original full scan; after uses the final hybrid query. Timings are whole substeps, not isolated GPU timestamps.

| Prey | Mode | Before: 0 hunters | Before: 16 | After: 0 hunters | After: 16 |
| ---: | --- | ---: | ---: | ---: | ---: |
| 20,000 | wander | 0.389/0.381 | 1.415/1.513 | 0.394/0.358 | 1.465/1.640 |
| 20,000 | follow | 0.365/0.301 | 1.118/1.523 | 0.311/0.692 | 1.145/1.254 |
| 100,000 | wander | 0.610/0.617 | 3.740/3.032 | 0.566/0.597 | 3.652/3.521 |
| 100,000 | follow | 3.003/2.938 | 1.273/1.122 | 3.393/3.248 | 1.591/1.400 |
| 300,000 | wander | 1.518/1.547 | 8.217/8.823 | 1.518/1.729 | 8.408/9.391 |
| 300,000 | follow | 3.604/3.390 | 1.682/1.713 | 3.665/3.734 | 2.071/1.722 |
| 1,000,000 | wander | 4.390/4.376 | 32.239/34.628 | 4.501/4.470 | 31.434/30.289 |
| 1,000,000 | follow | 7.891/8.044 | 6.729/7.556 | 7.818/7.869 | 7.547/8.776 |

Pre-edit 1M/no-hunter medians: wander 4.461/4.389 ms; follow 7.848/7.951 ms. Their p95 values were 4.634/4.636 and 8.918/9.211 ms respectively. The before-table zero-hunter entries were captured after adding only the benchmark option. The after-table entries are the final no-hunter comparison.

1M wander, 0 hunters p95: 4.774/4.576 ms. 1M wander, 16 hunters p95: 33.460/35.048 ms. 1M follow, 0 hunters p95: 9.151/8.688 ms. 1M follow, 16 hunters p95: 10.276/11.389 ms.

Load averages (1/5/15 minutes): pre-edit 0.98/1.34/1.56; original matrix 1.76/1.54/1.61; final batches 3.12/2.29/1.94, 2.64/2.22/1.92, 1.90/2.09/1.88 and 1.73/2.03/1.87. This is a shared-machine comparison, not an isolated idle-GPU profile. Final 1M wander with 16 hunters remains ~30–31 ms versus ~4.4 ms disabled; pursuit bounding does not resolve the total overhead. Follow timings overlap and fluctuate; do not claim a follow speedup.

Logs and intermediate variants: `/tmp/ac3d-hunter-scaling/`. `before-*` is the pre-edit baseline; `matrix-*` is the original search; `after-*` and `final-*` are rejected unconditional-grid variants; `hybrid-*` verifies the small-flock fallback; `verified-*` is the final table. `*-load.txt` stores uptime before each batch. `fmt.log`, `test.log`, `build.log`, `clippy.log`, `gpu.log`, `smoke.log` and `diff-check.log` hold final gates. Screenshot: `/tmp/ac3d-hunter-scaling/smoke.png` (20k prey/16 hunters; visually checked). Scenario: `/tmp/ac3d-hunter-scaling/smoke.txt`.

## Interfaces and Dependencies

Test-only environment control `GPU_BENCH_HUNTERS`, validated in 0..=16. Positive values reserve group 1 and activate at least two groups; zero preserves existing setup. The optional hunter layout adds three existing grid-buffer bindings; no new buffer allocation. `FULL_SCAN` selects the test reference; `SMALL_FLOCK_LIMIT=100000` is the production crossover, forced to zero in the target-equivalence test. No new crate or public application API.

---

Revision notes:

- 2026-11-02: Initial plan at the explicitly requested completed path. Measurements justified bounding pursuit; rejected slower bucket layout and unconditional small-flock grid queries. Recorded residual total cost rather than claiming a frame-time guarantee.
