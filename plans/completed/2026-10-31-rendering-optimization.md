# Measure and reduce flock rendering cost

This ExecPlan is a living document maintained according to `PLANS.md`. It covers Stage 4 only.
The orchestrator owns review and commits; this worker does not commit, push, merge or discard work.

## Purpose

Measure the styled scene and custom models at 20,000 and 100,000 wandering GPU vehicles, identify
rendering bottlenecks, and make the smallest supported improvement while preserving default visuals
and all simulation settings. Record repeatable before/after evidence and explicit quality tradeoffs.

## Progress

- [x] (2026-10-31 17:47Z) Initial method written before measurement or rendering optimization.
- [x] (2026-10-31 17:50Z) Read source/prior methods; measurement aids and heavy generator built without warnings.
- [x] (2026-10-31 18:14Z) Primary matrix repeated before selection; all 54 baseline samples now accepted, including auxiliary controls.
- [x] (2026-10-31 18:17Z) Exact import indexing; bit-equivalence/seam/status/default/parser tests pass.
- [x] (2026-10-31 18:48Z) All 16 accepted captures inspected; all 32 after timings accepted and paired with baseline.
- [x] (2026-10-31 18:48Z) Sequential release gates, Clippy, smoke, artifact audit and documentation complete; handback for orchestrator review.

## Surprises & Discoveries

- Hyprland currently exposes a 1800×1169 headless Wayland output, scale 1, refresh 60 Hz.
  Tiling is unsuitable for comparison; each app window will float at an exact client size.
- Initial load was 5.71 / 4.45 / 2.60; measurement waited for lower load.
- Hyprland 0.56.2 uses Lua dispatch objects. The legacy string-dispatch setup attempt failed before
  measurement and was terminated; preserved as rejected-setup-lua.*. No accepted timing was lost.
  Runner now uses `hl.dsp.window.float({window="address:...",action="on"})`, resize with x=1280,
  y=720, relative=false, center and `hl.dsp.focus({window="address:..."})`. No persistent config edits.
- Heavy generator output is 3,456 triangles / 10,368 expanded vertices / 250,048 bytes, SHA-256
  cef6e3a8fa8c8a5d93d312954c2a93daca3b2a4f499af1370950096752d8cc66. The default 24-triangle
  glider regenerates byte-for-byte (3ef0d93eaf7d78e26d359d2e7cddf3378fbe2d4b84232af5c7610618e98320d4).
  Independent Python checks cover repeatable bytes/counts/nondegenerate triangles at subdivisions
  1/2/12/16 and rejected values 0/17. No generated benchmark asset added to the repository.

- Initial test expectation used 60 vertices from a Python numerical-equality estimate; actual exact
  import retains 62 because signed-zero attribute bits stay distinct. Corrected the expectation,
  preserving the conservative key and bitwise triangle-equivalence regression. Failed test transcript
  retained as tests-initial.log; this was a test count assumption, not a geometry mismatch.

- Host load reached 7.16 / 4.20 / 3.06 while the runner was waiting, with I/O pressure around
  64% and blocked usb-storage/dmcrypt tasks. No timing was launched above the <=3.0 one-minute
  threshold. Initial builds also require a cooldown before after measurements. This is recorded
  as waiting time, not discarded timing evidence.

- First after-captures exposed external GPU work despite low CPU load: PID 78620, Hunyuan paint
  from another project, started after baseline completion and drove idle-app GPU utilization to 100%
  with about 10 GiB VRAM occupied. All first after-captures are rejected as comparison evidence and
  retained with rejected-gpu-contention-* prefixes. Two captures also closed before readback, logging
  `Failed to send screenshot: sending on a closed channel`; their logs are retained separately.
  No after timing sample had started. Runner now waits for the competing process to finish and idle
  Radeon utilization <=5%, in addition to original load/process rules. Screenshot exit is extended
  from 8 to 12 seconds; the actual timed protocol remains [6,18] with exit at 19 seconds.
  Evidence: external-gpu-contention.txt. No unrelated process was stopped or modified.

- A second external Hunyuan job (PID 93416, --resolution 256 --math-attention) started at 18:27:58Z
  during after-20000-default-glider-4-studio-on-1. The monitor rejected the entire sample even though
  its measured mean was a normal 1.032 ms: resource overlap invalidates it, regardless of result.
  Its log, scenario, uptime and window observations are retained as rejected-gpu-contention-after-*.
  The preceding Dart timing and all recaptured screenshots ended before that job started.
  Runner now preserves/retries any future overlap automatically after the other workload exits.
  All captures were inspected; no LOD exists, so there is no LOD transition to capture.

## Decision Log

- Decision: use a scenario-only flock draw toggle to isolate draw cost while retaining GPU simulation.
  Rationale: Bevy 0.20.1 RenderDiagnosticsPlugin supports Vulkan timestamps but requires enabling
  recorder/features and additional pass instrumentation for this custom compute path. Draw-disabled
  controls require no dependencies and answer the immediate attribution question. Frame differences
  are still overlap-sensitive; do not describe them as isolated GPU timestamps.
  Date: 2026-10-31.
- Decision: add scenario MSAA selection as a measurement aid before the baseline; no UI/default change yet.
  Rationale: compare the existing 4× path with off without editing the renderer between samples.
  Date: 2026-10-31.
- Decision: generate a subdivided glider with unchanged surface, normals and colors.
  Rationale: isolates triangle/vertex cost from silhouette, coverage and shading. It does not represent
  every complex organic model. Do not commit generated benchmark binary; retain it with logs.
  Date: 2026-10-31.

- Decision: deduplicate only bit-identical normalized position/normal/color tuples for imported models.
  Keep triangle order, indices, hard normals, color seams and all shader inputs; no epsilon welding.
  Rationale: primary baseline matrix is complete and geometry cost is decisive. Import currently
  destroys source sharing with identity indices. Recovering it is a small, lossless background-load
  change, with no new draw passes or simulation work. Repeats of the auxiliary no-draw control may
  finish from the unchanged archived baseline during source editing; no Cargo/app overlap is allowed.
  Date: 2026-10-31.
- Decision: retain MSAA 4× and the existing PBR/scene/default Dart path. Do not add an MSAA UI,
  LOD, impostors, a depth prepass, GPU compaction or frustum culling in this stage.
  Rationale: low-poly cases are already near 1 ms; heavy near/far and MSAA off timings are alike.
  The whole-frustum far case is slow too. Extra passes or approximate geometry/shading would add
  complexity or quality loss before trying the exact sharing recoverable from existing input.
  Date: 2026-10-31.
- Decision: after optimization repeat all custom-model cases (including MSAA-off heavy controls)
  and all four default/close Dart cases twice: 32 runs. Bird, Classic and no-draw paths are unchanged.
  Rationale: this covers every affected matrix case plus the explicitly required default-path controls.
  Date: 2026-10-31.

## Assumptions and Open Questions

- Low load means 1-minute load <= 3.0 on this 24-thread Ryzen, and no cargo/rustc/native app before
  each fresh launch. Log uptime (all three averages) for every run; wait rather than overlap work.
- GPU spawn uses the existing frame-zero PCG sequence, no explicit seed setting. Real frame deltas
  and GPU scatter ordering vary. Repeat fresh launches; do not claim identical simulation trajectories.
- A close view `camera 0 17.28 100` sits inside the spawn/flock region at both counts. Inspect captures;
  if coverage is insufficient, choose a new distance and rerun every matching case, documenting why.

## Outcomes & Retrospective

The primary matrix (all model/camera/count cases, MSAA controls and Classic comparison) had two
accepted samples each before optimization selection. Three auxiliary no-draw repeats were delayed
by unrelated host I/O pressure; they subsequently finished from the unchanged archived baseline.
All 54 baseline samples are accepted. Source editing did not change its executable or shader/model
assets, and no Cargo command ran until the last baseline app exited.

Baseline evidence supports geometry/vertex processing as the heavy-model limit: 100k glider is
about 1 ms, but the same surface subdivided to 3,456 triangles costs about 24 ms at both distances
and both MSAA settings. Draw-disabled controls stay near 1 ms. This is differential frame evidence,
not a timestamp attribution to one GPU stage. The low-poly cases sit near a shared simulation/CPU/presentation floor; its exact split is unmeasured.
The heavy model contains only 2,106 numerically equal position/normal/color tuples among 10,368 expanded vertices;
recover indexed sharing before considering an approximate rendering path. The final key preserves
signed zero too: final retained GPU vertex counts are 2,119 for heavy and 62 for the small glider.
Triangle counts stay 3,456 and 24. This reduces heavy-model vertex inputs by 79.6%, without a quality setting.

Delivered: exact indexed sharing for imported models, scenario-only MSAA/draw measurement controls,
actual GPU vertex counts in load status, deterministic subdivided-model generation and regression
coverage. There are 54 accepted before and 32 accepted after timings, two per case. All 16 accepted
screenshots were inspected. The heavy model reaches about 347 FPS at 20k and 86 FPS at 100k, with
mean intervals 45.5–51.9% lower. Default Dart stays near 1 ms with its original render path.

The table below averages each run's mean/p50/p95 separately; these are not pooled percentiles.
FPS is 1000 divided by the averaged mean. Settings are the 4× Studio cases described in the method.

| Count | Camera | Model | Before mean / p50 / p95 ms (FPS) | After mean / p50 / p95 ms (FPS) | Mean reduction |
| ---: | --- | --- | --- | --- | ---: |
| 20,000 | default | dart | 1.038 / 0.982 / 1.329 (963.4) | 1.042 / 0.978 / 1.347 (960.2) | -0.3% |
| 20,000 | default | glider | 1.066 / 0.988 / 1.393 (938.1) | 1.052 / 0.978 / 1.367 (950.6) | 1.3% |
| 20,000 | default | heavy | 5.277 / 5.270 / 5.489 (189.5) | 2.874 / 2.882 / 3.026 (347.9) | 45.5% |
| 20,000 | close | dart | 1.081 / 0.999 / 1.405 (925.5) | 1.028 / 0.972 / 1.332 (972.3) | 4.8% |
| 20,000 | close | glider | 1.043 / 0.982 / 1.345 (958.8) | 1.037 / 0.975 / 1.368 (964.3) | 0.6% |
| 20,000 | close | heavy | 5.295 / 5.290 / 5.533 (188.9) | 2.885 / 2.893 / 3.024 (346.6) | 45.5% |
| 100,000 | default | dart | 1.025 / 0.978 / 1.308 (975.6) | 1.019 / 0.974 / 1.305 (981.4) | 0.6% |
| 100,000 | default | glider | 1.027 / 0.989 / 1.266 (973.7) | 1.019 / 0.976 / 1.304 (980.9) | 0.7% |
| 100,000 | default | heavy | 23.868 / 23.873 / 24.139 (41.9) | 11.538 / 11.546 / 11.691 (86.7) | 51.7% |
| 100,000 | close | dart | 1.027 / 0.973 / 1.333 (973.2) | 1.004 / 0.972 / 1.251 (996.0) | 2.3% |
| 100,000 | close | glider | 1.045 / 0.988 / 1.348 (956.9) | 1.038 / 0.989 / 1.338 (962.9) | 0.6% |
| 100,000 | close | heavy | 24.099 / 24.093 / 24.570 (41.5) | 11.588 / 11.593 / 11.767 (86.3) | 51.9% |

### Attribution and decisions

- **Geometry/vertex processing:** the 24-triangle glider and 3,456-triangle variant have the same
  surface, flat normals, material colors, scale and coverage. At 100k, baseline mean intervals are
  1.03–1.05 ms versus 23.87–24.10 ms. Exact vertex sharing then cuts the heavy model's input
  vertices by 79.6% while retaining all triangles and fragment inputs. Its large frame-time gain
  therefore comes from recovered vertex reuse/geometry work, not a cheaper appearance.
- **Coverage/overdraw:** baseline heavy means differ by about 1% between far and close views;
  both remain expensive. Close-camera Dart stays near 1 ms. These tested views do not support a
  fragment-dominated diagnosis. No claim is made about arbitrary follow balls or larger render sizes.
- **MSAA:** heavy baseline means with MSAA off differ from 4× by at most about 1%. Dart off has
  no consistent gain. Keep 4× as the default and retain off only as a developer scenario control.
- **Shared floor:** no-draw controls retain the same count, simulation and scene, and average
  1.01–1.12 ms. Low-poly cases are around 1.03–1.09 ms. Small negative draw-cost differences are
  noise/overlap, not negative GPU work. Exact CPU, compute, shared-scene and presentation contributions
  cannot be separated with this method; these are frame intervals, not isolated GPU timestamps.
- **Scene:** 100k close Classic Dart averages 1.054 ms versus Studio 1.027 ms. This establishes no
  measurable Studio penalty at the shared floor; it does not prove the scene is free.
- **Rejected alternatives:** MSAA UI/default reduction, fewer Dart triangles and cheaper shading
  have no demonstrated need here. A depth prepass adds geometry work without an observed fill limit.
  Frustum culling cannot fix the expensive default view where the entire world fits. GPU compaction,
  distance LOD and impostors require new buffers/passes or approximate geometry; exact indexing
  already keeps the tested heavy 100k flock usable. Stop at this smaller change.

### Quality and cost tradeoffs

There is no visual quality setting or LOD transition in this change. Indexing preserves every
position/normal/color bit at every triangle corner, including signed zero, hard edges and material
color seams, and keeps triangle order. The regression re-expands indices and compares every bit.
Default Dart, lights, PBR, fog, 4× MSAA and simulation settings retain their existing path.

The tradeoff is bounded hashing/allocation once during background model load, rather than every
frame. Triangle count and per-instance data size do not change. Models without identical full
attribute tuples gain little or nothing; this does not make the 100k-triangle import limit a flock
performance guarantee. The high-poly benchmark is a controlled subdivided glider, not a universal
model survey. Only two accepted fresh launches per case were requested and recorded. Other GPUs,
M3 close-up behavior, higher resolutions, all-unique faceted models and dense follow views remain
outside this matrix. Historical M3 observations are retained separately, not compared as a speedup.


Stage 4 is complete for worker handback, without commits or pushes. Orchestrator review and ignored
GPU tests remain with the owner; this worker did not run them. The plan stays in planned/ as requested.
No Stage 5/6/7 work, dependency changes, simulation changes or default visual changes were made.

## Context and Orientation

`instancing.rs` queues one opaque depth-writing flock draw through Transparent3d. The vertex shader
applies quaternion/proportions/color per vertex; fragment shading uses Bevy PBR and fog.
At baseline, `vehicle_model.rs` expanded imported triangles and assigned identity indices. It now
shares exact normalized position/normal/color tuples through an index buffer. `scene.rs` owns MSAA 4× on the camera, sky, floor and framing. `scenario.rs`
provides timed settings, cameras, screenshots and frame-interval summaries. GPU simulation remains
in `gpu_flock.rs` and its WGSL; no changes to its behavior are allowed.

## Plan of Work

1. Finish reading source and prior evidence. Extend the generator and dev-only measurement controls.
2. Build one baseline with only measurement aids. Preserve its executable outside target for repeatability.
3. Run fresh-launch baseline cases twice: 2 counts × 2 cameras × Dart (14 triangles), Bird (24),
   glider (24), heavy glider (~2,000–5,000). GPU wander, Studio/Aurora, neutral size/proportions,
   Heading for built-ins, Model colors for imported models, MSAA 4×. Add MSAA-off worst cases,
   Classic-vs-Studio Dart controls and draw-disabled controls at both counts and cameras.
4. Name the limit using geometry scaling, distance, MSAA and no-draw comparisons. Select changes only then.
5. Repeat every affected case and the four count/camera Dart cases twice using identical scripts.
6. Document all runs, rejects, decisions, quality tradeoffs, checks and remaining limitations.

## Concrete Steps

All Cargo commands use PATH=$HOME/.cargo/bin:$PATH and CARGO_BUILD_JOBS=4, sequentially.
Before EVERY Cargo command and benchmark launch: `pgrep -a cargo; pgrep -a rustc; pgrep -a autonomous`.
For benchmark launches also run `uptime`, wait until 1-minute load <=3.0, and log uptime/process state.
After external GPU contention was discovered, also require idle Radeon utilization <=5% and no
Hunyuan paint process. The external process started at 18:16Z, after the last accepted baseline
ended at 18:14Z; it invalidates no baseline timing. Retain contention evidence and do not stop the
unrelated process. Reject an after run if that workload reappears during it.
Use Hyprland dispatch on the discovered app address: float, resize exact 1280 720, center and focus.
Poll client size/focus during each run; physical harness resolution MUST be 1280×720.
Reject any measurement-window size/focus violation, wrong model/import error, runtime error,
wrong reported resolution, interrupted run or load above threshold at launch. Retain rejected logs.
Each script requests `present immediate` (AutoNoVsync; retain fallback caveat), sets count/style/model
at 0 s, disables auto-frame, sets default orientation 0°/17.28°, requests Frame world at 1 s for
far cases (record final actual distance) or explicitly sets the close camera at 1 s. Warm up until
6 s; measure [6,18] s (12 s), end, exit at 19 s. No screenshots during timed runs.
Capture separate before/after screenshot scenarios, outside measurements. Scripts always end with exit.

After implementation run sequentially: fmt --check, release tests --locked, release build --locked,
Clippy --release --locked --all-targets, smoke-native.py, git diff --check. No ignored GPU tests.
Confirm no app remains after each run and at handback.

## Validation and Acceptance

Two accepted samples per case, exact resolution and stated camera/model/count/MSAA/style/mode.
Raw tables include mean/p50/p95 ms, FPS, launch load and external log paths. Compare against no-draw
controls without attributing all frame time to GPU work. New setting parsing/default-preservation
and any geometry/LOD/culling math receive tests. Zero compiler warnings and no new Clippy warnings.
Default settings preserve current visuals and simulation; optional tradeoffs must be user-selectable.

## Idempotence and Recovery

Every fresh app launch resets simulation in memory. Measurement scripts end with exit; runners check
process absence and enforce one instance. Generated models/logs/screenshots stay under the artifact
root. No persistent Hyprland config edits. Preserve all uncommitted work for orchestrator review.

## Artifacts and Notes

Artifact root: `/tmp/claude-1000/-home-martines-Projects-autonomous-characters-3d/af1dacb4-dc47-4bb5-8e8c-02c6cd1b95e8/scratchpad/shots/stage4/`.
Raw scripts, load/focus/resolution logs, baseline executable, generator output and screenshots go here.
Hyprland dispatch reference: https://wiki.hypr.land/configuring/core/dispatchers/.
Source check: bevy_render-0.20.1/src/diagnostic/{mod.rs,internal.rs}; recorder checks TIMESTAMP_QUERY
and inside-pass/encoder timestamp features. No GPU timestamps are claimed by this method.


Machine for every table: Ryzen 9 7900X (12 cores / 24 threads), RX 7900 XT (RADV NAVI31),
Vulkan / Mesa 26.2.5-arch1.1, Linux Omarchy 4.0.6 / kernel 7.2.8-9-omarchy, Rust 1.99.1,
Bevy 0.20.1, release --locked. Baseline is HEAD 7e66fd3 plus measurement aids only, preserved in
before-build/autonomous-characters-3d and baseline.patch; after executable is in after-build/.
The GPU uses its normal frame-zero PCG spawn; there is no seed override. Real deltas and scatter
ordering vary, so screenshots are appearance evidence rather than identical simulation snapshots.

Validation, sequential, CARGO_BUILD_JOBS=4 and cargo/rustc/autonomous checks before every Cargo command:

- fmt --check: pass (fmt.log).
- cargo test --release --locked: 67 passed, zero failed, five ignored (tests.log). Initial wrong
  vertex-count expectations are retained in tests-initial.log; bit-equivalence logic passed throughout.
- cargo build --release --locked: pass, zero compiler warnings (build.log).
- cargo clippy --release --locked --all-targets: seven pre-existing warnings per target, none added
  (clippy.log: two octree, one target, four unchanged vehicle warnings).
- Python generator regressions: three passed; deterministic bytes, count, area, winding, bounds and
  input-range checks (generator-tests.log; scripts/test_make_example_models.py).
- Native smoke: survived ten seconds without startup/asset errors (smoke.log).
- Artifact audit: 86 unique accepted timings, every case repeated twice, fixed physical size/focus,
  launch load threshold, no accepted runtime warnings/errors; 16 accepted captures (artifact-audit.log).
- git diff --check: pass. No ignored GPU tests or benchmarks were run.

Artifact names under the root above:

- results.jsonl and tables.md: accepted raw data; summary.md and summarize.py/write_tables.py: arithmetic.
- run_matrix.py: serial process/load/GPU guard, Hyprland floating/resize/focus, repeated scenarios,
  per-run .uptime and .window.json checks, explicit exits and resource-overlap rejection/retry.
- before-runner.log, after-runner.log, after-retry-runner.log: chronological run/wait/rejection records.
- Every raw row maps to `<phase>-<count>-<view>-<model>-<msaa>-<style>-<draw>-<run>.{scenario,log,uptime,window.json}`.
- Accepted screenshots: `shot-{before,after}-{20000,100000}-{default,close}-{dart,heavy}-4-studio-on-1.png`,
  with neighboring scenarios/logs/window checks. captures.jsonl contains exactly these 16 accepted shots.
- Rejected setup: rejected-setup-lua.* (legacy Hyprland dispatch, terminated before measurement).
- Rejected after-capture attempts: rejected-gpu-contention-shot-after-* and
  rejected-capture-channel-shot-after-*; five completed captures discarded as comparison evidence,
  plus two readback failures, all while external GPU work was active. Original metadata also remains
  in captures-with-contested-after.jsonl. Final accepted captures were rerun after that process exited.
- Rejected timing: rejected-gpu-contention-after-20000-default-glider-4-studio-on-1.*;
  mean/p50/p95 1.032 / 0.973 / 1.354 ms, 968.66 FPS, rejected solely for external process overlap.
  Its replacement is in the accepted table. No baseline timings or other after timings were rejected.
- external-gpu-contention.txt: unrelated process start and idle-app GPU load; high host I/O and
  additional export jobs caused waits, not accepted overlapping runs. No unrelated process was stopped.
- heavy.glb: original generated 3,456-triangle benchmark input, 250,048 bytes; retained outside Git.

### Before: raw accepted samples (54)

Every row: 1280×720 physical pixels, GPU mode, wander, Aurora; yaw 0°, pitch 17.28°; all size/proportion/model multipliers 1. Imported models use Model colors; built-ins use Heading. Camera distance is the harness value, not a requested window/camera size. Times are milliseconds; FPS = 1000 / mean interval. Load is launch 1/5/15-minute average. No screenshots in measurement windows.

| Count | Camera / distance | Model / triangles | MSAA | Style | Draw | Run | Mean ms | p50 ms | p95 ms | FPS | Load 1/5/15 |
| ---: | --- | --- | --- | --- | --- | ---: | ---: | ---: | ---: | ---: | --- |
| 20,000 | default / 2851.546 | dart / 14 | 4 | studio | on | 1 | 1.046 | 0.983 | 1.348 | 956.47 | 1.94 / 2.59 / 2.28 |
| 20,000 | default / 2851.546 | bird / 24 | 4 | studio | on | 1 | 1.095 | 1.003 | 1.449 | 912.93 | 2.43 / 2.65 / 2.31 |
| 20,000 | default / 2851.546 | glider / 24 | 4 | studio | on | 1 | 1.061 | 0.989 | 1.377 | 942.83 | 2.63 / 2.69 / 2.33 |
| 20,000 | default / 2851.546 | heavy / 3,456 | 4 | studio | on | 1 | 5.239 | 5.246 | 5.392 | 190.88 | 2.31 / 2.61 / 2.31 |
| 20,000 | close / 100.000 | dart / 14 | 4 | studio | on | 1 | 1.077 | 1.000 | 1.392 | 928.62 | 1.79 / 2.48 / 2.27 |
| 20,000 | close / 100.000 | bird / 24 | 4 | studio | on | 1 | 1.038 | 0.976 | 1.351 | 962.99 | 2.54 / 2.61 / 2.32 |
| 20,000 | close / 100.000 | glider / 24 | 4 | studio | on | 1 | 1.047 | 0.982 | 1.360 | 955.53 | 2.46 / 2.59 / 2.32 |
| 20,000 | close / 100.000 | heavy / 3,456 | 4 | studio | on | 1 | 5.267 | 5.254 | 5.518 | 189.85 | 2.92 / 2.68 / 2.35 |
| 100,000 | default / 4876.075 | dart / 14 | 4 | studio | on | 1 | 1.021 | 0.978 | 1.298 | 979.58 | 2.45 / 2.59 / 2.33 |
| 100,000 | default / 4876.075 | bird / 24 | 4 | studio | on | 1 | 1.041 | 0.991 | 1.334 | 960.44 | 2.54 / 2.60 / 2.34 |
| 100,000 | default / 4876.075 | glider / 24 | 4 | studio | on | 1 | 1.015 | 0.986 | 1.223 | 985.43 | 2.95 / 2.69 / 2.38 |
| 100,000 | default / 4876.075 | heavy / 3,456 | 4 | studio | on | 1 | 23.880 | 23.888 | 24.128 | 41.88 | 2.84 / 2.69 / 2.38 |
| 100,000 | close / 100.000 | dart / 14 | 4 | studio | on | 1 | 1.033 | 0.972 | 1.356 | 967.71 | 2.16 / 2.55 / 2.34 |
| 100,000 | close / 100.000 | bird / 24 | 4 | studio | on | 1 | 1.132 | 1.022 | 1.518 | 883.65 | 2.19 / 2.53 / 2.34 |
| 100,000 | close / 100.000 | glider / 24 | 4 | studio | on | 1 | 1.047 | 0.987 | 1.363 | 955.21 | 2.57 / 2.59 / 2.37 |
| 100,000 | close / 100.000 | heavy / 3,456 | 4 | studio | on | 1 | 24.093 | 24.077 | 24.531 | 41.51 | 2.86 / 2.67 / 2.40 |
| 20,000 | default / 2851.546 | heavy / 3,456 | off | studio | on | 1 | 5.231 | 5.229 | 5.439 | 191.15 | 2.25 / 2.54 / 2.36 |
| 20,000 | close / 100.000 | heavy / 3,456 | off | studio | on | 1 | 5.231 | 5.235 | 5.402 | 191.17 | 1.89 / 2.44 / 2.33 |
| 100,000 | default / 4876.075 | heavy / 3,456 | off | studio | on | 1 | 23.829 | 23.827 | 24.163 | 41.97 | 2.10 / 2.45 / 2.34 |
| 100,000 | close / 100.000 | heavy / 3,456 | off | studio | on | 1 | 23.839 | 23.824 | 24.176 | 41.95 | 1.65 / 2.32 / 2.30 |
| 20,000 | close / 100.000 | dart / 14 | off | studio | on | 1 | 1.055 | 0.985 | 1.373 | 947.96 | 1.47 / 2.24 / 2.27 |
| 100,000 | close / 100.000 | dart / 14 | off | studio | on | 1 | 1.102 | 0.995 | 1.477 | 907.27 | 1.95 / 2.31 / 2.29 |
| 100,000 | close / 100.000 | dart / 14 | 4 | classic | on | 1 | 1.059 | 0.990 | 1.376 | 944.19 | 2.35 / 2.37 / 2.31 |
| 20,000 | default / 2851.546 | dart / 14 | 4 | studio | off | 1 | 1.097 | 0.996 | 1.421 | 911.42 | 2.91 / 2.51 / 2.36 |
| 20,000 | close / 100.000 | dart / 14 | 4 | studio | off | 1 | 1.084 | 0.997 | 1.399 | 922.73 | 2.96 / 2.59 / 2.39 |
| 100,000 | default / 4876.075 | dart / 14 | 4 | studio | off | 1 | 1.047 | 0.980 | 1.363 | 955.12 | 2.95 / 2.62 / 2.41 |
| 100,000 | close / 100.000 | dart / 14 | 4 | studio | off | 1 | 1.035 | 0.969 | 1.355 | 966.50 | 2.96 / 2.67 / 2.43 |
| 20,000 | default / 2851.546 | dart / 14 | 4 | studio | on | 2 | 1.030 | 0.982 | 1.309 | 970.93 | 2.90 / 2.67 / 2.44 |
| 20,000 | default / 2851.546 | bird / 24 | 4 | studio | on | 2 | 1.046 | 0.984 | 1.355 | 956.33 | 2.88 / 2.68 / 2.45 |
| 20,000 | default / 2851.546 | glider / 24 | 4 | studio | on | 2 | 1.071 | 0.986 | 1.408 | 934.12 | 2.55 / 2.61 / 2.43 |
| 20,000 | default / 2851.546 | heavy / 3,456 | 4 | studio | on | 2 | 5.314 | 5.293 | 5.586 | 188.17 | 2.79 / 2.67 / 2.46 |
| 20,000 | close / 100.000 | dart / 14 | 4 | studio | on | 2 | 1.084 | 0.998 | 1.418 | 922.77 | 2.38 / 2.58 / 2.43 |
| 20,000 | close / 100.000 | bird / 24 | 4 | studio | on | 2 | 1.057 | 0.990 | 1.367 | 946.02 | 2.35 / 2.56 / 2.43 |
| 20,000 | close / 100.000 | glider / 24 | 4 | studio | on | 2 | 1.039 | 0.981 | 1.331 | 962.80 | 2.98 / 2.69 / 2.47 |
| 20,000 | close / 100.000 | heavy / 3,456 | 4 | studio | on | 2 | 5.322 | 5.325 | 5.548 | 187.91 | 2.77 / 2.66 / 2.47 |
| 100,000 | default / 4876.075 | dart / 14 | 4 | studio | on | 2 | 1.029 | 0.977 | 1.318 | 972.10 | 2.45 / 2.60 / 2.45 |
| 100,000 | default / 4876.075 | bird / 24 | 4 | studio | on | 2 | 1.034 | 0.987 | 1.318 | 967.46 | 2.59 / 2.62 / 2.46 |
| 100,000 | default / 4876.075 | glider / 24 | 4 | studio | on | 2 | 1.039 | 0.992 | 1.308 | 962.57 | 2.80 / 2.68 / 2.48 |
| 100,000 | default / 4876.075 | heavy / 3,456 | 4 | studio | on | 2 | 23.856 | 23.857 | 24.149 | 41.92 | 2.92 / 2.72 / 2.50 |
| 100,000 | close / 100.000 | dart / 14 | 4 | studio | on | 2 | 1.022 | 0.974 | 1.309 | 978.11 | 2.33 / 2.59 / 2.46 |
| 100,000 | close / 100.000 | bird / 24 | 4 | studio | on | 2 | 1.052 | 1.006 | 1.327 | 950.39 | 2.80 / 2.68 / 2.50 |
| 100,000 | close / 100.000 | glider / 24 | 4 | studio | on | 2 | 1.043 | 0.988 | 1.332 | 958.59 | 2.92 / 2.72 / 2.51 |
| 100,000 | close / 100.000 | heavy / 3,456 | 4 | studio | on | 2 | 24.104 | 24.110 | 24.609 | 41.49 | 2.82 / 2.72 / 2.52 |
| 20,000 | default / 2851.546 | heavy / 3,456 | off | studio | on | 2 | 5.246 | 5.253 | 5.409 | 190.63 | 2.02 / 2.55 / 2.47 |
| 20,000 | close / 100.000 | heavy / 3,456 | off | studio | on | 2 | 5.256 | 5.265 | 5.421 | 190.26 | 1.63 / 2.43 / 2.43 |
| 100,000 | default / 4876.075 | heavy / 3,456 | off | studio | on | 2 | 23.821 | 23.818 | 24.087 | 41.98 | 1.34 / 2.33 / 2.39 |
| 100,000 | close / 100.000 | heavy / 3,456 | off | studio | on | 2 | 23.906 | 23.891 | 24.249 | 41.83 | 1.37 / 2.27 / 2.37 |
| 20,000 | close / 100.000 | dart / 14 | off | studio | on | 2 | 1.115 | 1.043 | 1.422 | 896.90 | 1.38 / 2.22 / 2.35 |
| 100,000 | close / 100.000 | dart / 14 | off | studio | on | 2 | 1.040 | 0.983 | 1.342 | 961.24 | 1.79 / 2.25 / 2.36 |
| 100,000 | close / 100.000 | dart / 14 | 4 | classic | on | 2 | 1.048 | 0.987 | 1.353 | 953.83 | 2.01 / 2.27 / 2.37 |
| 20,000 | default / 2851.546 | dart / 14 | 4 | studio | off | 2 | 1.084 | 1.002 | 1.399 | 922.48 | 2.68 / 2.40 / 2.41 |
| 20,000 | close / 100.000 | dart / 14 | 4 | studio | off | 2 | 1.099 | 1.011 | 1.437 | 909.88 | 2.94 / 3.77 / 3.13 |
| 100,000 | default / 4876.075 | dart / 14 | 4 | studio | off | 2 | 1.195 | 1.093 | 1.499 | 836.69 | 2.98 / 3.69 / 3.13 |
| 100,000 | close / 100.000 | dart / 14 | 4 | studio | off | 2 | 0.994 | 0.952 | 1.275 | 1006.33 | 2.83 / 3.62 / 3.15 |

### After: raw accepted samples (32)

Every row: 1280×720 physical pixels, GPU mode, wander, Aurora; yaw 0°, pitch 17.28°; all size/proportion/model multipliers 1. Imported models use Model colors; built-ins use Heading. Camera distance is the harness value, not a requested window/camera size. Times are milliseconds; FPS = 1000 / mean interval. Load is launch 1/5/15-minute average. No screenshots in measurement windows.

| Count | Camera / distance | Model / triangles | MSAA | Style | Draw | Run | Mean ms | p50 ms | p95 ms | FPS | Load 1/5/15 |
| ---: | --- | --- | --- | --- | --- | ---: | ---: | ---: | ---: | ---: | --- |
| 20,000 | default / 2851.546 | dart / 14 | 4 | studio | on | 1 | 1.037 | 0.976 | 1.346 | 964.28 | 0.94 / 2.05 / 2.72 |
| 20,000 | default / 2851.546 | glider / 24 | 4 | studio | on | 1 | 1.073 | 0.986 | 1.391 | 932.09 | 1.60 / 2.47 / 2.77 |
| 20,000 | default / 2851.546 | heavy / 3,456 | 4 | studio | on | 1 | 2.868 | 2.879 | 3.005 | 348.67 | 2.13 / 2.53 / 2.78 |
| 20,000 | close / 100.000 | dart / 14 | 4 | studio | on | 1 | 1.016 | 0.961 | 1.311 | 984.45 | 1.73 / 2.40 / 2.73 |
| 20,000 | close / 100.000 | glider / 24 | 4 | studio | on | 1 | 1.034 | 0.970 | 1.380 | 967.37 | 1.91 / 2.40 / 2.72 |
| 20,000 | close / 100.000 | heavy / 3,456 | 4 | studio | on | 1 | 2.878 | 2.887 | 3.010 | 347.42 | 1.68 / 2.30 / 2.68 |
| 100,000 | default / 4876.075 | dart / 14 | 4 | studio | on | 1 | 1.029 | 0.978 | 1.324 | 971.42 | 1.43 / 2.20 / 2.64 |
| 100,000 | default / 4876.075 | glider / 24 | 4 | studio | on | 1 | 1.011 | 0.973 | 1.276 | 988.98 | 1.63 / 2.20 / 2.62 |
| 100,000 | default / 4876.075 | heavy / 3,456 | 4 | studio | on | 1 | 11.531 | 11.538 | 11.689 | 86.72 | 2.44 / 2.36 / 2.66 |
| 100,000 | close / 100.000 | dart / 14 | 4 | studio | on | 1 | 1.001 | 0.971 | 1.229 | 998.97 | 1.61 / 2.17 / 2.59 |
| 100,000 | close / 100.000 | glider / 24 | 4 | studio | on | 1 | 1.034 | 0.984 | 1.338 | 966.92 | 2.11 / 2.25 / 2.61 |
| 100,000 | close / 100.000 | heavy / 3,456 | 4 | studio | on | 1 | 11.578 | 11.585 | 11.743 | 86.37 | 2.23 / 2.28 / 2.61 |
| 20,000 | default / 2851.546 | heavy / 3,456 | off | studio | on | 1 | 2.875 | 2.879 | 3.042 | 347.84 | 1.76 / 2.17 / 2.57 |
| 20,000 | close / 100.000 | heavy / 3,456 | off | studio | on | 1 | 2.852 | 2.860 | 2.999 | 350.63 | 1.81 / 2.15 / 2.55 |
| 100,000 | default / 4876.075 | heavy / 3,456 | off | studio | on | 1 | 11.536 | 11.541 | 11.713 | 86.68 | 1.61 / 2.09 / 2.52 |
| 100,000 | close / 100.000 | heavy / 3,456 | off | studio | on | 1 | 11.517 | 11.527 | 11.669 | 86.83 | 1.33 / 1.99 / 2.47 |
| 20,000 | default / 2851.546 | dart / 14 | 4 | studio | on | 2 | 1.046 | 0.980 | 1.348 | 956.08 | 1.06 / 1.87 / 2.42 |
| 20,000 | default / 2851.546 | glider / 24 | 4 | studio | on | 2 | 1.031 | 0.970 | 1.342 | 969.73 | 1.23 / 1.85 / 2.40 |
| 20,000 | default / 2851.546 | heavy / 3,456 | 4 | studio | on | 2 | 2.881 | 2.885 | 3.046 | 347.08 | 1.34 / 1.84 / 2.38 |
| 20,000 | close / 100.000 | dart / 14 | 4 | studio | on | 2 | 1.041 | 0.982 | 1.352 | 960.57 | 1.38 / 1.82 / 2.36 |
| 20,000 | close / 100.000 | glider / 24 | 4 | studio | on | 2 | 1.040 | 0.980 | 1.356 | 961.99 | 1.58 / 1.85 / 2.35 |
| 20,000 | close / 100.000 | heavy / 3,456 | 4 | studio | on | 2 | 2.893 | 2.900 | 3.038 | 345.62 | 1.78 / 1.88 / 2.35 |
| 100,000 | default / 4876.075 | dart / 14 | 4 | studio | on | 2 | 1.009 | 0.969 | 1.286 | 991.38 | 1.60 / 1.84 / 2.32 |
| 100,000 | default / 4876.075 | glider / 24 | 4 | studio | on | 2 | 1.028 | 0.980 | 1.332 | 972.44 | 2.25 / 1.98 / 2.36 |
| 100,000 | default / 4876.075 | heavy / 3,456 | 4 | studio | on | 2 | 11.544 | 11.554 | 11.692 | 86.63 | 2.03 / 1.95 / 2.34 |
| 100,000 | close / 100.000 | dart / 14 | 4 | studio | on | 2 | 1.007 | 0.973 | 1.272 | 993.53 | 1.33 / 1.79 / 2.28 |
| 100,000 | close / 100.000 | glider / 24 | 4 | studio | on | 2 | 1.043 | 0.994 | 1.338 | 959.19 | 2.18 / 1.97 / 2.32 |
| 100,000 | close / 100.000 | heavy / 3,456 | 4 | studio | on | 2 | 11.598 | 11.601 | 11.790 | 86.22 | 2.16 / 1.98 / 2.32 |
| 20,000 | default / 2851.546 | heavy / 3,456 | off | studio | on | 2 | 2.868 | 2.869 | 3.044 | 348.66 | 1.61 / 1.87 / 2.27 |
| 20,000 | close / 100.000 | heavy / 3,456 | off | studio | on | 2 | 2.840 | 2.849 | 2.981 | 352.15 | 1.51 / 1.83 / 2.25 |
| 100,000 | default / 4876.075 | heavy / 3,456 | off | studio | on | 2 | 11.498 | 11.512 | 11.644 | 86.97 | 1.44 / 1.79 / 2.23 |
| 100,000 | close / 100.000 | heavy / 3,456 | off | studio | on | 2 | 11.524 | 11.533 | 11.688 | 86.77 | 1.01 / 1.66 / 2.17 |

## Interfaces and Dependencies

Implemented dev-facing scenario controls: `set flock_draw on|off`, `set msaa 4|off`.
Default flock draw remains enabled; default camera remains Sample4. No dependency changes.
`ImportedModel::indexed_mesh` shares exact ten-f32 position/normal/color tuples after normalization.
Load status now reports actual indexed GPU vertices; expanded-input triangle/vertex limits stay intact.
If Stage 6 adds UVs or other attributes, extend the key before allowing those vertices to share.

---

Revision notes:

- 2026-10-31: Initial method recorded before measurements.

- 2026-10-31 18:12Z: Primary baseline matrix complete; selected lossless import indexing. Three auxiliary control repeats delayed by host I/O; archived baseline unchanged.

- 2026-10-31 18:22Z: Strengthened idle-device guard after unrelated GPU generation invalidated after-captures. Timed scripts and baseline samples unchanged.

- 2026-10-31 18:48Z: Completed repeated measurements, rejection audit, validation and documentation; handed back uncommitted for owner review.
