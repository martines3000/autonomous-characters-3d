# Flock on the GPU

This ExecPlan follows `PLANS.md`. Completed on 2026-10-30; external GPU/app evidence and the final sandbox gate are recorded below.

## Purpose

The CPU simulation tops out around 30,000 vehicles at 80 FPS (flocking about 9 ms per step) and becomes quadratic when vehicles follow the target. After this work, a "GPU" simulation mode runs steering, neighbor search and integration in compute shaders and writes the instance buffer the renderer already draws, so nothing goes back to the CPU. Someone running the app can pick GPU mode in the menu, raise the vehicle count far beyond the CPU limit, and see the same flocking behavior (wander, walls, follow, separation, alignment, cohesion, sliders). The naive and octree CPU modes remain as the tested reference.

The owner allowed broad changes for performance and asked to remove bad practices introduced earlier; such cleanups are listed in the Plan of Work and the Decision Log.

## Progress

- [x] (2026-10-30 21:00Z) Plan drafted after reading Bevy 0.20.1's compute examples (`compute_shader_game_of_life.rs`, `gpu_readback.rs`) and `bevy_render`'s public renderer API.
- [x] (2026-10-30 22:00Z) Milestone 1: orchestrator confirmed GPU mode works in the app outside the sandbox, with screenshots. Spawn, wander, walls, seek, integration and shared draw path work on the M3 Pro.
- [x] (2026-10-30 22:00Z) Milestone 2: orchestrator ran both GPU tests outside the sandbox; both pass. Worst error over three seek/wander steps: position 1.7e-5, velocity 1.2e-4. Original hashed grid, scans and heading retention validated on the real device.
- [x] (2026-10-30 22:30Z) Milestone 3: external GPU regression/benchmark runs pass; repeat measurements confirm wander uses the unlimited path without its previous cap overhead. GPU / 20,000 vehicles / follow cap 16 selected. Screenshots confirm the scaled camera frames the wall box and distinct flocks. Target scaling is implemented and covered headlessly; final interactive target inspection remains a handback limitation, not unfinished implementation.
- [x] (2026-10-30 21:48Z) Cleanups: existing WIP has `GlobalState` defaults without a startup system, simulation mode enum, separation spelling and dead-code removal. CPU mode transitions now have a regression test.
- [x] (2026-10-30 21:51Z) Sandbox gate: formatting passes; release tests 29 passed / 3 ignored; release build has zero warnings; Clippy reports 7 existing warnings and no new warnings; `git diff --check` passes. GPU tests attempted once, both fail at device creation (see below).
- [x] (2026-10-30 22:03Z) Round-2 sandbox gate: formatting passes; release tests 30 passed / 5 ignored; release build zero warnings; Clippy 7 existing warnings / none added; clean `git diff --check`. Naga validates both new shader entry points. No GPU/device/window attempt in this round.
- [x] (2026-10-30 22:20Z) Round-3 sandbox gate: formatting passes; release tests 32 passed / 5 ignored; release build zero warnings; Clippy 7 existing warnings / none added; clean `git diff --check`. New coverage verifies byte-identical wander GPU uniforms across caps and headless startup camera/target culling. No shader edits, GPU timing, native app, window or device run this round.
- [x] (2026-10-30 22:30Z) Final target sizing: the existing visual/pick mesh follows live world scale; headless tests cover count increases/decreases and unchanged position, rotation and movement speed.
- [x] (2026-10-30 22:30Z) Closure: external screenshots and independent review accepted; README and STATUS updated; final gate 33 passed / 5 ignored, release build zero warnings, Clippy 7 existing warnings / none added, formatting and diff checks clean. Follow density, rendering cost and the wall-drag behavior decision are explicitly deferred to `plans/STATUS.md`; plan moved to `plans/completed/`.

## Surprises & Discoveries

- Observation: `bevy_render` exposes `renderer::initialize_renderer`, `RenderDevice` (wrapping `wgpu::Device`) and `RenderQueue` publicly, so a headless test can create a real GPU device without a window and without new dependencies.
  Evidence: `bevy_render-0.20.1/src/renderer/mod.rs:199`, `settings.rs:167` (`RenderResources(RenderDevice, RenderQueue, ...)`).
- Observation (2026-10-30): wgpu 30's `get_mapped_range()` returns `Result`; failing to unwrap it prevented test compilation. Empty test readbacks now return before a zero-length GPU map; empty uploads also skip unnecessary allocation.
- Observation (2026-10-30): Naga rejected `position < -walls | position > walls`; parenthesizing both comparisons fixes parsing. A standalone probe linked to the existing Naga 30.0.1 rlib validates the complete shader, including uniformity. `Params` spans 144 bytes, with `walls` at 112 and `target_position` at 128; an encase regression test checks these offsets and every scalar.
- Observation (2026-10-30): storing only position, velocity and wander angles loses the CPU's retained orientation at zero velocity. Boid state now also carries rotation (48 bytes rather than 32); instances remain 32 bytes. Rotation-arc singularity thresholds and the opposite-heading axis now match glam.
- Observation (2026-10-30): both GPU tests were attempted once together and failed in `bevy_render-0.20.1/src/renderer/mod.rs:308:36`: `Unable to find a GPU! Make sure you have installed required drivers!` No further device attempts were made; strengthened tests compile but need execution outside the sandbox.
- Observation (2026-10-30, orchestrator's round-2 report): original GPU tests pass outside the sandbox. In release on an M3 Pro with vsync on a 120 Hz display and load average about 2: 200 vehicles reaches 120 FPS; 100,000 wandering with a close camera reaches 59 FPS, while 300,000 wandering with a distant camera reaches 92 FPS. Camera-dependent rasterization cost matters. 30,000 followers reach 31 FPS in a dense ball, exposing the uncapped neighbor traversal. These are reported app observations, not new sandbox measurements.
- Observation (2026-10-30): a first-64 cutoff in a coarse cell does not select the nearest 64 and can discard the strongest separation pushes. A separation-radius grid finds all close neighbors without reading entire alignment-radius cells. It adds a grid rebuild and 16 bytes of separation scratch per boid (64-byte state, still 32-byte instances), reusing all seven storage bindings.
- Observation (2026-10-30): nearest-neighbor diagnostics are deliberately O(1000 × count) on the CPU after GPU readback, outside the measured steps. A unit test verifies self-exclusion and that an unsampled boid still contributes as a nearest neighbor.
- Observation (2026-10-30, round 3): the supplied benchmark shows a large sparse-wander penalty at high counts (cap 16: 37.415 ms vs unlimited 26.300 ms at one million). Inspection confirms that every nonzero-cap step unconditionally records two grid builds and a separate separation dispatch, independent of how many neighbors reach the cap. The benchmark is end-to-end and does not isolate scan/separation/register costs; no per-stage profiling or new GPU execution was possible in this sandbox. Identical wander uniforms now guarantee identical dispatches and shader branches across requested caps.
- Observation (2026-10-30): scaling the camera from the 200-vehicle reference to 20,000 puts it about 2,188 units from the origin. Bevy's default 1,000-unit far culling distance would hide the ordinary target mesh even though the flock bypasses frustum culling. Scale the far distance by the same existing `world_scale` helper; the fog and controller anchor depth already follow world/eye size.

Round-3 evidence supplied by the orchestrator: release build, M3 Pro, load average 1.4–4; fresh GPU PCG spawn at frame 0, dt=1/60, 600 warmup steps, 60 synchronized timed steps. Median ms/step from `gpu-bench1.txt` (rounded):

| Count | wander cap 0 | wander cap 16 | wander cap 32 | follow cap 0 | follow cap 16 | follow cap 32 |
| --- | --- | --- | --- | --- | --- | --- |
| 10,000 | 0.51 | 0.44 | 0.49 | 6.5 | 1.07 | 0.90 |
| 100,000 | 2.51 | 2.51 | 2.30 | 220 | 6.4 | 8.1 |
| 300,000 | 7.50 | 9.79 | 10.36 | skipped | 16.2 | 18.2 |
| 1,000,000 | 26.3 | 37.4 | 42.0 | skipped | 46.4 | 56.2 |

Wander has 0% sampled nearest distances below 1.4 at every cap, with similar speed/spacing statistics. Follow overlap percentages are comparable across caps 16–64 at each count; they are not zero (cap 16: 3.7% at 10k, 19.1% at 100k, 42.6% at 300k, 14.8% at one million). Skipped unlimited follow cases did not complete warmup and are not steady-state comparisons. This is one supplied sweep under varying load, not a new measurement or a repeated controlled comparison. Full log: `/private/tmp/claude-501/-Users-martines-Desktop-git-repos-personal-autonomous-characters-3d/6c3ee9af-73e9-4d9e-baaf-d88371ea7993/scratchpad/codex.ISlk/gpu-bench1.txt`.


Round-4 confirmation supplied by the orchestrator: release, Apple M3 Pro, load average about 3, fresh PCG spawn at frame 0, 600 warmup steps at dt=1/60 followed by 60 synchronized timed steps. `gpu-bench2.txt` confirms requested wander caps 0 and 16 have comparable timings and an effective cap of 0:

| Vehicles | Wander median ms | Follow cap 16 median ms | Follow unlimited | Follow sample closer than 1.4 |
| --- | --- | --- | --- | --- |
| 20,000 | 0.67 | 1.4 | 16.9 ms | 2.2% |
| 100,000 (earlier sweep) | 2.5 | 6.4 | 220 ms | 19% |
| 300,000 | 7.4 | 13.2 | skipped, step >250 ms | 42% |
| 1,000,000 | 26.1 | 44.9 | skipped, step >250 ms | 15% |

All wander cases have 0% sampled nearest distances below 1.4. Percentages describe 1,000 sampled boids versus the whole flock after the fixed warmup, not a full pair count or a steady-state density guarantee. The earlier 100k row has load average 1.4–4. External app observations include rendering and vsync on a 120 Hz display: default 20k about 107–120 FPS wandering and about 127 reported by the FPS counter while following; 100k close-up wander about 59 FPS, 300k distant wander about 92 FPS, and pre-cap 30k follow about 31 FPS. These are observations under different views, not directly comparable headless step measurements.

- Observation (2026-10-30, final round): external screenshots show the camera framing works, but the fixed-radius target became only a few pixels across. Scaling its existing `Transform.scale` by `vehicle::world_scale(current_count)` enlarges both the rendered and picked sphere without changing the seek point or world-space movement speed. The headless test spans counts 0–1,000,000 and confirms the mesh is reused.
- Observation (2026-10-30, independent review reported by orchestrator): no material defects found. A candidate involving cell-coordinate/hash saturation near coordinates of 1e12 was dismissed as unreachable in intended interactive use; the shader still has finite i32 cell coordinates. This is a residual range limit, not a tested guarantee of arbitrary unbounded coordinates.
- Observation (2026-10-30): the old CPU `flocking_benchmark` substring filter also selects `gpu_flocking_benchmark`. README now uses the exact CPU test name and skips the long benchmark in the general GPU regression filter.

## Decision Log

Entries are chronological; later decisions supersede provisional defaults and round-specific scope.

- Decision: write the GPU simulation against `RenderDevice`/`RenderQueue`/`CommandEncoder` directly (pipelines built once from a WGSL file included with `include_str!`), not through Bevy's `PipelineCache` and asset-loaded shaders.
  Rationale: the same code then runs inside the app (render world, `RenderGraph` schedule before `camera_driver`) and in a headless test on a real device, which is the only way to check the shader against the CPU reference with `cargo test`. No asynchronous shader loading state machine is needed. The cost is no shader hot reload, which this project does not use.
  Date: 2026-10-30
- Decision: hashed uniform grid with counting sort: per frame clear counts, count boids per cell with atomics, exclusive prefix sum (per-block scan plus a single-workgroup scan of block sums), scatter boids into cell order, then steer and integrate each boid by visiting the 27 surrounding cells. The cell size is the largest neighbor radius; the hash table has a power-of-two size of at least twice the capacity.
  Rationale: a hash grid works in unbounded space (vehicles leave the walls, the target moves anywhere) with memory proportional to the vehicle count, and needs no GPU tree. Hash collisions only add candidates that the distance test rejects; duplicate cells among the 27 are skipped.
  Date: 2026-10-30
- Decision: boid state lives only on the GPU in GPU mode (no ECS entities). Changing the count spawns new boids on the GPU (or truncates); switching modes respawns the flock.
  Rationale: per-entity ECS work is the CPU cost being removed. Reading GPU state back for a mode switch adds latency and code for little value.
  Date: 2026-10-30
- Decision: the compute pass writes the existing instance format (`[position, scale]`, rotation quaternion) into a buffer the instanced draw uses directly; the draw command takes its buffer and count from a render-world resource filled by whichever mode is active.
  Rationale: keeps one vertex shader and one draw path for all modes.
  Date: 2026-10-30
- Decision: retain a quaternion in each GPU boid and carry it through scatter; preserve it when the integrated velocity normalizes to zero. Explicitly place compute in `RenderGraphSystems::Render` before `camera_driver`.
  Rationale: mirrors CPU heading retention without CPU readback or changing the draw format; the extra 16 bytes per boid are required state. Render-set membership makes the command submission lifecycle explicit.
  Date: 2026-10-30
- Decision: use multiplication for squared radii and the CPU's separation-factor multiplication order; keep the wall force outside the in-bounds wander branch.
  Rationale: `pow(radius, 2.0)` may round differently at strict neighbor cutoffs; the CPU applies wall drag even inside the walls. Tests keep the original position (< 1e-3) and velocity (< 1e-2) tolerances.
  Date: 2026-10-30
- Decision: do not tune the existing WIP's GPU default, count limits or neighbor cap in this round; leave README, measurements and Milestone 3 to the orchestrator.
  Rationale: explicitly outside the assigned implementation scope; the uncapped dense-crowd cost and provisional default still need real-device review.
  Date: 2026-10-30
- Decision: for a nonzero cap, compute complete separation first using 27 cells of width `vehicle_size * vehicle_separation_distance`; rebuild the same grid buffers with the largest neighbor radius and take at most `gpu_max_neighbors` accepted alignment/cohesion samples, own coarse cell first. Cap 0 skips the fine pass and retains the CPU-equivalent unlimited query. Default cap is provisionally 64; expose 0..=512 in a GPU-only menu slider, with 0 meaning unlimited.
  Rationale: arbitrarily capped separation misses the closest boids; finding the nearest k in the original coarse cells still scans the dense crowd. Exact fine-grid separation preserves the existing repulsion law, while the cap removes the large-radius dense-ball traversal. A fine-grid-only search would lose long-range alignment/cohesion in sparse wandering flocks. CPU modes are untouched. This bounds long-range sampling, not pathological all-coincident local density: complete separation can still be quadratic, and hash collisions can add rejected candidates. Measure that tradeoff before choosing the final default.
  Date: 2026-10-30
- Decision: add `gpu_flocking_benchmark`, with fresh normal GPU spawn at PCG frame 0 per case, 600 warmup steps plus 60 timed steps at dt=1/60; each timed interval covers encode, submit and blocking device poll. Abort remaining steps of a case after any completed step exceeds 250 ms, including warmup. Diagnose the final state even for skipped cases and label partial results explicitly.
  Rationale: followers need 10 simulated seconds to gather; timing submission alone hides GPU execution. The slow-step guard bounds the number of further slow dispatches without suppressing evidence. Readback and fixed-sample nearest-neighbor statistics are excluded from timing. Keep the original GPU/CPU parity test explicitly at cap 0 and add a cap-1 dense-cluster separation regression test.
  Date: 2026-10-30
- Decision: use unlimited single-grid steering for wander, regardless of the requested follow cap; retain the complete fine-grid separation plus capped coarse sampling path while following. Expose the control as follow-mode samples and add `effective_cap` to benchmark output. Keep cap 0's CPU-equivalent behavior unchanged in both modes.
  Rationale: this is the cheap alternative permitted by round 3. Removing fine-grid work only after discovering that the cap was not reached would need a density-detection/conditional-dispatch scheme or a new multilevel grid, adding synchronization, storage and sparse-case overhead. Retrying separation in the coarse grid after the cutoff would restore quadratic dense-follow scans. Routing by follow state removes all additional wander work with no GPU readback and preserves every separation neighbor. Tradeoff: unusually dense wander is unlimited, and dispersed followers still pay for the second grid; this is not a universal density-adaptive cap optimization. The benchmark supports this mode split because sparse wander's spacing/speed was insensitive to the cap, while dense follow benefited strongly. Re-measure to confirm end-to-end timings; do not attribute exact percentages to individual GPU stages without profiling.
  Date: 2026-10-30
- Decision: choose follow cap 16, retain GPU as default, and set default count to 20,000 as requested; use `vehicle::world_scale` for camera eye and far culling distance. Pin both CPU test-state helpers to 200 vehicles.
  Rationale: cap 16 has similar follow spacing to 32/64 with lower median cost at 100k–1m; small-count timings are noisy and do not justify the larger caps. Scaling the existing eye preserves relative world framing, and scaling culling keeps the target visible. Legacy tests retain their original spawn cube and walls rather than silently testing a world 4.64× larger.
  Date: 2026-10-30


- Decision: scale only the target's existing transform scale with the live count's `world_scale`, in an Update system; leave its translation, rotation, mesh, selection, seek behavior and 120-unit/s input movement unchanged.
  Rationale: Bevy renders and mesh-picks with the same transformed sphere. This makes the target proportionate to the walls without rebuilding meshes or coupling movement to visual scale. Only assign a new scale when it differs, avoiding needless transform updates.
  Date: 2026-10-30
- Decision: close this plan with the measured native GPU implementation and documented limits; defer large-follow density, rendering cost and the joint CPU/GPU wall-drag decision to the prioritized tasks in `plans/STATUS.md`.
  Rationale: the feature meets its native performance/control goals without silently changing the existing force laws or claiming that all million-vehicle cases reach 60 FPS. The external review's 1e12-coordinate saturation case is recorded rather than expanding the intended coordinate range.
  Date: 2026-10-30

## Assumptions and Open Questions

- Validated on the M3 Pro: wgpu dispatch ordering/barriers are sufficient for the scans, scatter, separation and steering sequence. External GPU/CPU parity, growth/reset and complete-separation tests pass.
- Validated on the M3 Pro: seven storage bindings plus the uniform fit the device limits. Other native adapters were not exercised; GPU tests and normal renderer startup still require a compatible real device. Restricted sandboxes can fail to expose it.
- Answered: the selected defaults are GPU / 20,000 / follow cap 16; wander's effective cap is 0. A repeat benchmark confirms removal of the large-count wander penalty, and external screenshots confirm startup camera framing.
- Deferred, tracked in `plans/STATUS.md`: follow crowding at 100k+, camera-dependent rasterization cost, and whether wall avoidance should act as drag inside the walls. These are behavior/performance initiatives with their own plans, not blockers for this feature.
- Residual limits: complete local separation and dense unlimited wander can remain quadratic; sparse follow still pays for both grids; i32 cell conversion saturates at extreme coordinates (review's dismissed ~1e12 case). No arbitrary-coordinate support is promised.
- Final visual target enlargement and actual desktop ray clicks were not exercised by the sandboxed implementer. Transform/mesh/selection/movement behavior is checked headlessly; an external operator can inspect the resulting target without additional implementation work.

## Outcomes & Retrospective

Delivered on 2026-10-30: a native GPU simulation feeding the existing instanced renderer directly, with no normal-frame simulation readback and no new dependencies. CPU Naive/Octree modes remain the reference. The menu supports counts to one million on the GPU and 50,000 on the CPU, with a follow-only alignment/cohesion sample cap and complete close-neighbor separation. Mode changes discard/respawn GPU state by design; count growth adds boids, reductions truncate and zero resets. Defaults now live in `GlobalState::default`: GPU, 20,000 vehicles, follow cap 16.

Startup camera position, culling distance and orbit anchor frame the larger world. The target now scales its rendered/pickable sphere with the current world while preserving position, selection, seek behavior and movement speed. Its headless regression covers growing/shrinking counts and mesh reuse; the movement test changes scale during held input. README documents controls, modes, CPU/GPU commands, benchmark semantics and qualified performance results. STATUS records completion and the three scoped follow-ups.

External validation: the orchestrator reports all four GPU tests/benchmark passing outside the sandbox, CPU/GPU errors at most 1.7e-5 in position and 1.2e-4 in velocity over three seek/wander steps, and screenshots of distinct flocks inside the full default wall box. Headless median steps at 20k are about 0.67 ms wander / 1.4 ms follow cap 16; at 300k, 7.4 / 13.2 ms; at one million, 26.1 / 44.9 ms. The default app counter reports about 107–120 FPS wandering. These do not imply million-vehicle rendering at 60 FPS. Final sandbox gate: 33 tests passed, 5 ignored; release build zero warnings; Clippy 7 existing warnings and none added; format/diff checks pass. No native app, window or GPU device was run by the implementer during the final round; no commit or Cargo dependency changes were made.

Remaining gaps and residual risk: GPU regression execution requires a GPU exposed outside restricted sandboxes and is ignored in the ordinary test gate; only the M3 Pro was tested externally. Follow crowding remains significant at 100k+ (15–42% sampled below a dart width), complete separation has no hard worst-case cost bound, and sparse followers still use two grids. Rendered FPS depends strongly on camera coverage/MSAA/geometry; no rendering optimization was attempted. CPU/GPU inside-wall drag remains unchanged pending an owner decision. The independent review's ~1e12-coordinate hash/cell saturation case was dismissed as outside intended reachable use, not fixed or guaranteed correct. Desktop clicking/appearance of the final scaled target still needs external inspection if desired.

Lessons: arbitrary caps must not discard the nearest separation forces. A complete fine-grid pass solves that correctness problem but has a real sparse-case cost; routing wander to the original single-grid path removes it without a new density-detection protocol. Timed GPU submission must include completion, and benchmark spacing statistics must accompany timing results. World-scaled defaults also require camera culling and target picking geometry to scale, not just spawn positions and walls.

## Context and Orientation

- `src/gpu_flock.rs`: extracted 144-byte `GpuParams`, `gpu_params` (effective cap 0 for wander), raw compute pipelines and growable GPU buffers. `GpuFlock::encode` spawns, builds grids, steers and integrates; boid state is 64 bytes including rotation and separation scratch, draw instances are 32 bytes. Test-only upload/readback exposes seeded reference scenarios.
- `assets/shaders/gpu_flock.wgsl`: count/prefix-sum/scatter hash grid; complete fine-grid separation before capped coarse alignment/cohesion in follow mode; single-grid exact query for cap 0 or wander. Seek, wander, walls and integration mirror CPU rules, including inside-wall drag and stopped-heading retention.
- `src/vehicle.rs`: CPU spawner/cleanup, `movement`/`flocking_force`/`update`, `world_scale`/`walls`/`spawn_size`, instance sync, ignored CPU/GPU tests and benchmark. GPU mode spawns no ECS vehicles; legacy CPU fixture helpers explicitly use 200 vehicles.
- `src/instancing.rs`: CPU instance upload or direct GPU buffer selection through render-world `FlockInstances`, drawn with one instanced call and the existing `assets/shaders/vehicle_instancing.wesl`.
- `src/main.rs`: `SimulationMode`, `GlobalState::default`, menu/ranges, startup camera/world-scaled culling and camera input.
- `src/target.rs`: target selection/color and camera-relative movement; `update_target_scale` follows live vehicle-count world scaling without changing translation or movement speed.
- `README.md`: final controls, mode semantics, GPU/CPU commands and performance evidence; `plans/STATUS.md`: completed-plan link and density/rendering/wall-drag follow-ups.

## Plan of Work

Implemented sequence:

1. Add `GpuFlock`, extracted params and raw pipelines compiled from included WGSL, so the same encoder works in the render world and headless device tests. Reuse the instanced draw format; run compute in `RenderGraphSystems::Render` before `camera_driver`.
2. Build the hash grid with count, per-512-cell scan, block-sums scan and scatter. Match CPU force/integration rules; retain orientation in boid state and validate uniform layout and WGSL.
3. Introduce Naive/Octree/GPU modes and GPU-only state, suppress CPU entities in GPU mode, implement grow/truncate/reset, and centralize real defaults. Keep CPU tests explicit about the old 200-vehicle reference.
4. Preserve complete separation using a fine grid before capped coarse alignment/cohesion; reuse the same seven storage bindings and carry separation through the second sort. Cap only follow mode to avoid sparse-wander overhead; select cap 16 from measured results.
5. Add ignored device tests plus synchronized benchmark and spacing statistics. External execution confirms parity, lifecycle and performance; ordinary sandbox tests cover parameters, modes, camera, target and numerical behavior.
6. Set 20,000 as the default count, scale camera/culling and live target size with the existing world scale, document results/limits, close the plan and move density/rendering/wall-drag work to STATUS.

## Concrete Steps

From the repository root, sequentially:

1. `CARGO_BUILD_JOBS=4 cargo fmt --all -- --check`
2. `CARGO_BUILD_JOBS=4 cargo test --release --locked`
3. `CARGO_BUILD_JOBS=4 cargo test --release --locked gpu_ -- --ignored --nocapture --test-threads=1 --skip gpu_flocking_benchmark` (orchestrator, outside sandbox; excludes the long benchmark)
4. `GPU_BENCH_COUNTS=20000,100000,300000,1000000 GPU_BENCH_CAPS=0,16 GPU_BENCH_MODES=wander,follow CARGO_BUILD_JOBS=4 cargo test --release --locked gpu_flocking_benchmark -- --ignored --nocapture --test-threads=1` (orchestrator's round-3 comparison; check load and repeat)
5. `CARGO_BUILD_JOBS=4 cargo test --release --locked gpu_flocking_benchmark -- --ignored --nocapture --test-threads=1` (orchestrator, full sweep; check load with `uptime`, repeat before recording results)
6. `CARGO_BUILD_JOBS=4 cargo build --release --locked`
7. `CARGO_BUILD_JOBS=4 cargo clippy --release --locked --all-targets`
8. `python3 scripts/smoke-native.py` (orchestrator only)
9. `git diff --check`

Benchmark configuration: comma-separated `GPU_BENCH_COUNTS` defaults to `10000,30000,100000,300000,1000000` (allowed 2..=1,000,000); `GPU_BENCH_CAPS` defaults to `0,32,64,128` (u32); `GPU_BENCH_MODES` defaults to `wander,follow`. Counts loop outermost, then modes, then caps. Each case prints exactly one benchmark line with count/requested cap/effective cap/mode, spawn frame and dt, completed warmup/timed steps, median/p95 synchronized step milliseconds, nearest sample count, p1/p10/median nearest distance in world units, percentage below 1.4 units, whole-flock mean speed, mean target distance (follow only; otherwise NA), and completion/skip status. Wander's effective cap is always 0 after round 3; the requested follow-cap column remains for comparison. A skipped line names the slow phase and duration; timing percentiles are NaN if warmup aborted before any timed step, otherwise they describe only the completed timed steps. Statistics on skipped cases describe an incompletely warmed state. Nearest sample indices are deterministic and evenly spaced across the final sorted buffer; all boids are searched, excluding self. Percentiles use nearest rank; medians average the two middle values. CPU diagnostics can take time after a GPU case, especially at one million boids, and are excluded from timings.

## Validation and Acceptance

- Passed externally: GPU mode renders distinct flocks in the expected startup wall box; the orchestrator supplied screenshots, mode observations and FPS counters. Target size was then corrected using the same world scale and checked headlessly; final interactive clicking was not run in the sandbox.
- Passed externally: cap-0 seek/deterministic-wander CPU parity within original position <1e-3 / velocity <1e-2 tolerances; stopped headings, growth beyond 256 scan blocks, truncation/reset, bounds and cap-1 complete separation. The GPU benchmark also passes; all need an exposed GPU and remain ignored in the ordinary test gate.
- Met native scale goal: external 300k distant wander about 92 FPS, default 20k about 107–120 FPS; headless steps up to one million are recorded with build, machine, load and warmup. Follow density, close-up fill cost and million-vehicle 60 FPS are not claimed solved.
- Passed final sandbox gate: formatting, 33 non-ignored tests, warning-free release build, no additional Clippy warnings and clean diff checks. Dependency manifests/lockfile are unchanged.
- Documentation/closure complete: README controls/implementation/commands/performance and STATUS completed/future-work entries match the implementation; all Progress items are closed and the file lives in `plans/completed/`.

## Idempotence and Recovery

Normal app/test starts create a fresh in-memory flock; no simulation data persists. Re-running the benchmark creates fresh GPU state per case. The target scale update reuses the existing mesh and is idempotent at a fixed count. Device tests and app runs must remain sequential under the resource rules in `AGENTS.md`.

All work is source/assets/docs and no dependency files changed. To roll back, preserve unrelated uncommitted work and undo this feature as one reviewed change across the module, shader, mode/default changes and renderer integration; restoring only one part can break compilation. The plan was untracked and moved with a plain filesystem rename, without changing the Git index. Disposable probes remain in the named scratch directory and use no shared build output for experimental compilation.

## Artifacts and Notes

- Final gate (2026-10-30 22:30Z), sequential with `CARGO_BUILD_JOBS=4`: `cargo fmt --all` and `cargo fmt --all -- --check` pass; `cargo test --release --locked`: 33 passed, 0 failed, 5 ignored; `cargo build --release --locked`: zero warnings; `cargo clippy --release --locked --all-targets`: 7 pre-existing warnings per target, none added; `git diff --check`: clean. Counts include the new target scaling regression; held-key movement is also exercised while world scale changes.
- External final confirmation from the orchestrator: all four GPU tests/benchmark pass; review found no material defects; screenshots confirm default camera framing. Benchmark logs live in `/private/tmp/claude-501/-Users-martines-Desktop-git-repos-personal-autonomous-characters-3d/6c3ee9af-73e9-4d9e-baaf-d88371ea7993/scratchpad/codex.ISlk/gpu-bench1.txt` and `gpu-bench2.txt` in that same directory. The second sweep confirms the wander bypass and the selected follow cap; rounded tables and conditions are retained above and in README so the result does not depend on the temporary files surviving.

- 2026-10-30 round-3 sandbox gate, sequential with `CARGO_BUILD_JOBS=4`: `cargo fmt --all` and `cargo fmt --all -- --check` pass; `cargo test --release --locked`: 32 passed, 0 failed, 5 ignored; `cargo build --release --locked`: zero warnings; `cargo clippy --release --locked --all-targets`: 7 existing warnings per target, none added; `git diff --check`: clean. Native GPU tests/benchmark remain ignored in the sandbox. The startup regression constructs only a headless Bevy world with asset support and scene setup; it installs no renderer, window plugin or native runner.
- 2026-10-30 round-2 sandbox gate, sequential with `CARGO_BUILD_JOBS=4`: `cargo fmt --all` and `cargo fmt --all -- --check` pass; `cargo test --release --locked`: 30 passed, 0 failed, 5 ignored; `cargo build --release --locked`: zero warnings; `cargo clippy --release --locked --all-targets`: 7 existing warnings per target, none added; `git diff --check`: clean. The existing standalone Naga 30.0.1 probe also validates the new two-grid shader with all validation flags. No new GPU measurements claimed; cap 64 remains provisional.
- 2026-10-30 final sandbox gate, sequential with `CARGO_BUILD_JOBS=4`: `cargo fmt --all` and `cargo fmt --all -- --check` pass; `cargo test --release --locked`: 29 passed, 0 failed, 3 ignored; `cargo build --release --locked`: zero warnings; `cargo clippy --release --locked --all-targets`: 7 existing warnings per target, no new warnings; `git diff --check`: clean. The documented baseline is 8 warnings; the WIP's removed octree visualization contained the additional `len() > 0` lint.
- Naga probe source and executable: `/private/tmp/claude-501/-Users-martines-Desktop-git-repos-personal-autonomous-characters-3d/6c3ee9af-73e9-4d9e-baaf-d88371ea7993/scratchpad/codex.ISlk/gpu-flock-validation/`. It links the checkout's existing Naga 30.0.1 rlib directly, writes no Cargo artifacts into `target/`, and reports `WGSL validates with naga` with all validation flags enabled.
- GPU command attempted exactly once: `CARGO_BUILD_JOBS=4 cargo test --release --locked gpu_ -- --ignored --nocapture --test-threads=1`: 0 passed, 2 failed at renderer initialization, 27 filtered out. The final expanded tests were subsequently compiled by the normal test gate; their runtime assertions were not executed in this sandbox; the orchestrator subsequently ran them successfully outside it.

## Interfaces and Dependencies

No new crates or Cargo changes. New: `gpu_flock` module (`GpuFlock`, `GpuParams`, `GpuFlockPlugin`, 64-byte `BoidData`), `SimulationMode`, render-world `FlockInstances`, and `GlobalState::gpu_max_neighbors` (follow-only, 0 unlimited). `GpuFlock::encode(device, queue, encoder, params)` is the shared native render/test interface. `vehicle::world_scale` is crate-visible and used by the startup camera and `target::update_target_scale`; target movement remains 120 world units/s. `GlobalState` owns its real defaults through `Default` (GPU / 20,000 / cap 16). Removed: `GlobalState::use_octree`, `configure_global_state`, `test_octree_visualization`. Historical separation misspellings were corrected. GPU state is not migrated to/from CPU entities on a mode change.

---

Revision notes:

- 2026-10-30: Initial plan.
- 2026-10-30: Round 2 closes original GPU execution milestones from the orchestrator's evidence; adds complete fine-grid separation with capped coarse alignment/cohesion, a GPU-only cap control and a synchronized benchmark. Final cap selection and visual validation remain with the orchestrator; README, STATUS and rendering changes are explicitly out of scope for this round.
- 2026-10-30: Round 3 records the supplied cap sweep, selects follow cap 16 / count 20,000, and chooses an explicitly follow-only cap to remove the unnecessary wander grid work. Camera eye/far culling scale with the world; sparse-follow adaptivity is deferred as a larger GPU design change. README and STATUS remain for the next round.
- 2026-10-30: Final round adds live target visual/pick scaling, updates README and STATUS, records the repeat benchmark and review limits, closes every Progress item and moves this plan to completed. Rendering/density/wall-drag changes remain separate initiatives.
