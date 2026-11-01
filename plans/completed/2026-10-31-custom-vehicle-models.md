# Import custom glTF vehicle models

This ExecPlan is a living document maintained according to `PLANS.md`. It covers Stage 3 static import and
Stage 6 baked animation/materials under [the umbrella plan](2026-10-31-visual-showcase-and-swarm-features.md).
The orchestrator owns review and commits. This implementation worker must not commit or push.

## Purpose

Stage 3 lets users load local static GLB/glTF vehicles while the flock runs, correct their axes and
scale, and retain the previous selection after an import error. Multipart colors use the existing
single instanced draw. Stage 6 adds per-material metallic/roughness and background-baked rigid/skin animation, still using one draw.

## Progress

- [x] (2026-10-31 17:23Z) Read instructions, plans, renderer and dependency source evidence.
- [x] (2026-10-31 17:34Z) Stage 3: bounded importer, background loading, selection, axes, shader colors and scenarios.
- [x] (2026-10-31 17:34Z) Stage 3: deterministic original example asset and importer/preservation regression tests.
- [x] (2026-10-31 17:38Z) Stage 3: sequential release gates, Clippy, inspected desktop captures and repeated focused 20k frame measurements.
- [x] (2026-10-31 19:22Z) Stage 6 authorized; re-read stages 3–5 and verify pinned glTF readers/render seams.
- [x] (2026-10-31 19:41Z) Stage 6: material attributes, rigid/skin baking, playback, stable phases, examples and regression tests; 89 normal tests and four GPU regressions pass.
- [x] (2026-10-31 19:55Z) Stage 6: sequential gates, nine inspected captures, paused/error pixel proof and eight accepted repeated timing cases. No commits by worker.

## Surprises & Discoveries

- Bevy reexports gltf with utils and extensions, but without import. Its gltf-json reexport also
  exposes JSON Value and serialization, allowing precise unsupported-feature errors before validation.
- MeshPipeline already adds VERTEX_COLORS and COLOR at location 5. The custom shader must consume it.
- gltf-json rejects a missing accessor bufferView during its own validation, before the importer
  reaches its defensive missing-view branch. The regression expects Malformed glTF with that JSON
  path; unsupported sparse accessors are detected in preflight before validation.
- The native desktop currently tiles the app at 1776×1119, unlike earlier stages' 1667×1390.
  Keep this resolution difference visible when comparing recorded frame intervals.

## Decision Log

- Decision (Stage 6): retain one draw; carry metallic/roughness as a vertex attribute and include it
  in exact deduplication. Keep textures and morph targets rejected; reject CUBICSPLINE explicitly.
  Rationale: shared PNG support needs UV/sampler/material ownership beyond the accepted single-draw
  material slice. Separate primitive draws from the earlier memo are unnecessary for scalar factors.
  Date: 2026-10-31.
- Decision (Stage 6): bake rigid node TRS and four-influence linear blend skins at 30 Hz, including
  the endpoint, with <=240 frames and <=32 MiB position/normal data. Use one rest-pose normalization.
  Preserve deformation identity when sharing vertices; strip joint/weight inputs from drawn geometry.
  Rationale: animation cost stays independent of vehicle count on the CPU, and rest bounds do not pulse.
  Date: 2026-10-31.
- Decision (Stage 6): store a spawn hash in GPU separation.w, carrying it through both sorts and
  scratch resets; CPU phase hashes stable Entity bits. Add a third Vec4 to the instance stream
  (32 to 48 bytes) without touching speed/group bits. Compare 100k Dart before/after twice.
  Rationale: instance index is unstable, quaternion encoding would affect rotation precision, and
  the Stage 7 group reservation must survive. A separate phase buffer would complicate shared draw
  ownership and paused growth. Steering reads separation.xyz only.
  Date: 2026-10-31.
- Decision (Stage 6): simulation-clock delta advances a render-only playback accumulator, scaled
  by animation speed. Group 3 holds appearance, baked storage and playback metadata/base vertex.
  Specialize ANIMATED only for the active animated mesh; upload baked storage only on model changes.
  Rationale: pause/speed follow Stage 5; static geometry avoids animation shader work.
  Date: 2026-10-31.

- Decision: directly parse through `bevy::gltf::gltf`; resolve local buffers explicitly, reject URI schemes.
  Rationale: no dependency changes; avoid gltf import's disabled base64/urlencoding dependencies.
  Date: 2026-10-31.
- Decision: expand triangles into one indexed mesh, compute missing flat normals, bake hierarchy and
  inverse-transpose normals, and reverse reflected triangle winding. Ignore TANGENT and UV0.
  Rationale: simple, bounded static geometry; preserve colors at material boundaries and one draw.
  Date: 2026-10-31.
- Decision: source forward maps to local +Y; source up maps to local +Z. Default source axes are +Z/+Y.
  Normalize centered bounds to 2.5 units along forward, then apply model scale and existing proportions.
  Rationale: right-handed rotation matching the built-in vehicles; no simulation changes.
  Date: 2026-10-31.
- Decision: successful custom selection defaults to Model colors; built-ins without COLOR_0 use white.
  Rationale: material factors and vertex colors are linear glTF colors, independent of heading palettes.
  Date: 2026-10-31.
- Decision: reject textures, alpha modes, double-sided materials, emission, extensions, skins,
  animation, morph targets, sparse accessors and UV1+. Metallic/roughness remain the scene style's.
  Rationale: explicit static subset avoids silently misleading material rendering. Separate primitive
  draws, image decoding, file dialogs, new crates and animation are rejected for Stage 3 scope.
  Date: 2026-10-31.

## Assumptions and Open Questions

- Assumption: 100,000 triangles / 300,000 expanded vertices, 64 MiB input/buffer budget, 4,096 nodes
  and hierarchy depth 128 are sufficient for vehicle models. Validate errors at boundaries; Stage 4
  may measure tighter rendering budgets. These are sanity limits, not a performance guarantee.
- Stage 6 budgets are now fixed at 30 Hz, 240 frames, 32 MiB baked data, 128 joints and 256 channels. Textures remain unsupported.

## Outcomes & Retrospective

Stage 3 is implemented and verified for orchestrator review, without commits or pushes. Fourteen
new regression tests bring the normal release suite to 64 passed / five ignored. Existing shape
and GPU uniform preservation tests also cover Custom. Native captures prove the same mesh works
in GPU and Octree modes, axis correction, material colors and retained selection after a texture
error. A fresh baseline startup smoke check also passes. No compiler warnings or new Clippy warnings.
No Cargo manifest, simulation implementation/default, historical review or umbrella plan changes.

The static subset is deliberately strict: unsupported material features/extensions fail rather
than silently render differently. Expanding every triangle simplifies flat normals and material
boundaries but increases vertex work; the 100k-triangle limit is a sanity ceiling, not a 20k-flock
performance promise. Stage 4 owns optimization. Stage 6 is implemented and verified for orchestrator review. The final normal release suite has
89 passing tests and six ignored cases; the four ignored GPU regressions passed in one serial run.
Python's five generator tests pass. Release build has zero warnings; Clippy remains at the seven
pre-existing warnings. Nine requested/clip captures were inspected. Paused scene pixels and the
same scene after a rejected CUBICSPLINE load compare with AE zero. Eight accepted timing cases
include repeated 20k/100k animated bird runs and 32/48-byte static Dart controls. Four mistakenly
configured built-in Bird timings remain preserved and explicitly rejected. No Stage 6 blocker.

Residual limits are explicit: no textures, morph targets or CUBICSPLINE; positive animated scales;
fixed 30 Hz bake/interpolation approximation; STEP discontinuities quantized to the next sample and
mixed LINEAR motion held during that interval. Rest bounds can be exceeded by animated wings.
Loop endpoint continuity is the asset author's responsibility. Detailed/long clips must fit the
frame/memory budgets. A low-poly 72-vertex example does not establish heavy animated-model budgets.

## Context and Orientation

`src/vehicle_model.rs` will own parsing, normalization and asynchronous import state.
`vehicle_appearance.rs` owns built-in shapes and mesh switching. `instancing.rs` specializes Bevy's
MeshPipeline; `vehicle_instancing.wesl` performs the actual vertex/fragment shading.
`scenario.rs` provides timed developer commands. No simulation module should change its behavior.

## Plan of Work

1. Add bounded static parsing and normalization, plus an original deterministic GLB generator.
2. Integrate background tasks, scanned model choices, path input, axes/scale and recoverable status.
3. Consume vertex colors conditionally in the existing shader; extend scenario settings.
4. Prove unsupported errors and simulation preservation, document controls and run release gates.
5. Capture wrong/correct axes, colors, GPU/Octree and error recovery; measure 20k twice.
6. Stage 6: retain one mesh/draw with metallic/roughness at a custom vertex attribute, included in
   exact indexing. Bake selected rigid/skin clip on the I/O pool at 30 Hz. Rest normalization applies
   unchanged to every sample. Bind frame-major position/normal storage and playback/base-vertex
   metadata alongside appearance at group 3. Strip joints/weights from drawn geometry.
7. Carry a stable spawn hash in GPU separation.w and through the 48-byte instance stream; CPU
   phases derive from stable Entity bits. Playback uses SimulationClock delta and a separate render
   speed. Prove pause, sorting, growth, material seams and untouched simulation state.
8. Preserve the original memo recommendation as an alternative: separate per-primitive draws sharing
   ONE instance buffer, optional PNG through Bevy Image::from_buffer (JPEG not enabled), and CPU-baked
   vertex animation. Do not duplicate VehicleInstances, because preparation concatenates them.
   The current scalar-material requirement does not need separate draws or textures. The memo's
   instance-index hash was rejected because GPU sorting changes that index every frame. Base-vertex
   correction remains required. 2,000 vertices × 30 samples × 32 bytes is about 1.83 MiB.

## Concrete Steps

From repository root, check `pgrep -a cargo; pgrep -a rustc; pgrep -a autonomous` before EVERY Cargo command or app run and wait if busy.
Run sequentially with CARGO_BUILD_JOBS=4: fmt, release tests --locked, release build --locked,
Clippy --release --locked --all-targets. Then smoke-native.py, explicit exit-ended scenario runs,
process checks and git diff --check. Stage 6 permits one final ignored GPU regression run, excluding gpu_flocking_benchmark. No ignored benchmark runs.

## Validation and Acceptance

Stage 6 adds hand-computed rigid and two-joint skin poses, TRS composition, LINEAR/STEP behavior,
loop/partial-endpoint math, fixed rest normalization, in-memory frame/data budget errors, morph and
CUBICSPLINE rejection, material seam indexing, actual example clips, pause/speed playback and
static/built-in specialization exclusion. Whole GlobalState/GPU parameters/CPU instance state stay
unchanged through animated model installation. An ignored GPU test verifies sorting, phase transport,
group bits and paused buffer growth; existing GPU parity tests remain passing.

Capture two running wing poses with varied phases, a paused pair two seconds apart with zero AE scene
pixels, material close-up, GPU 20k, Octree and unsupported-animation recovery. Repeat 20k/100k bird
frame measurements twice at fixed 1280×720/default framing. Compare 100k Dart at 32/48-byte stride
with two fresh launches each, following Stage 4's six-second warmup/twelve-second sample protocol.


Example counts/bounds and reproducible bytes; finite positions/normals/colors; all 24 perpendicular
signed axis pairs; hierarchy, indexed/nonindexed primitives and material/color multiplication;
precise feature/path rejection tests; stale/failed load recovery; whole GlobalState/GPU parameter
and entity/instance preservation. Release build adds no warnings; Clippy stays at seven existing.
Desktop evidence includes close-up wrong/correct axes, Model colors, running 20k GPU, Octree and
an error with the previous mesh visible. Frame evidence records machine, framing, seed policy and load.

## Idempotence and Recovery

The generator deterministically overwrites its selected output: glider.glb, bird.glb or material-glider.glb by default for that kind. Background requests replace
pending work; only the newest successful completion swaps the mesh. Failed imports retain selection.
Built-in selection cancels pending custom selection. Retry after correcting the file or path.
The orchestrator reviews the uncommitted diff; no reset/discard operations by the worker.

## Artifacts and Notes

Source evidence: bevy_gltf-0.20.1/src/lib.rs:171 and Cargo.toml:122; gltf-1.4.1/src/lib.rs
(Gltf::from_slice); gltf-json-1.4.1/src/lib.rs (Value/deserialize/serialize reexports);
bevy_pbr-0.20.1/src/render/mesh.rs:3465 (COLOR location 5 and VERTEX_COLORS).
Logs/captures: /tmp/claude-1000/-home-martines-Projects-autonomous-characters-3d/af1dacb4-dc47-4bb5-8e8c-02c6cd1b95e8/scratchpad/shots/stage3/.

Stage 3 checks (sequential, process checks before every Cargo command, CARGO_BUILD_JOBS=4):

- cargo fmt --all -- --check: passed.
- cargo test --release --locked: 64 passed, zero failed, five ignored (0.10 s).
- cargo build --release --locked: passed, zero compiler warnings.
- cargo clippy --release --locked --all-targets: seven pre-existing warnings, zero new warnings
  (two octree, one target, four unchanged vehicle implementation warnings).
- python3 scripts/smoke-native.py: release survived ten seconds, no startup/asset errors.
- git diff --check: passed. Ignored GPU tests and all ignored benchmarks were not run.
- Generator SHA-256 before/after rerun: 3ef0d93eaf7d78e26d359d2e7cddf3378fbe2d4b84232af5c7610618e98320d4.

Thirteen importer/lifecycle tests cover the example, all 24 signed perpendicular axis pairs,
source sidecars and embedded buffers, all three index widths, nonindexed triangles, float/u8/u16
colors, factor multiplication, multiple primitives, baked TRS/matrix/reflections/normals, unsupported
features and malformed resources/ranges, nonfinite floats, invalid counts/normals/colors/settings,
empty/ambiguous scenes, cycles/repeated nodes, limits, background scanning/cancellation/stale results,
failed-load recovery and unchanged whole GlobalState/GPU parameters/instances/selection. The new
scenario test covers path/axis/scale/color commands and startup validation. Existing CPU shape
preservation now includes Custom and retains entities, positions, velocities, follow state and count.

Desktop captures, all visually inspected, under the artifact root above:

- wrong-axes.png: 200 GPU vehicles, source +X forward/+Y up (intentionally wrong), distance 200.
- corrected-axes-model-colors.png: same live flock, +Z forward/+Y up, distinct blue wings/gold nose.
- error-previous-model.png: rejected baseColorTexture with visible feature/path/recovery status;
  the previous glider selection and rendering remain. The rejected file is unsupported.gltf.
- custom-heading.png: user can choose Heading after model load.
- octree-model-colors.png: shared glider mesh in Octree at count 200.
- gpu-20000-model-colors.png: running 20k GPU flock with default Frame world framing.

screenshots.scenario changes settings without a reset command for axis/error cases, then switches
CPU/GPU through the existing respawn rule. It exits at 11 s. trial.scenario also exits normally.
Every native scenario ended with exit and no autonomous process remained. desktop.log has no shader,
asset or runtime warnings/errors; the expected import rejection appears as an informational status.
Error UI text remains fully readable in the default scrollable menu. The unrequested trial capture
is retained as diagnostic evidence, not part of the final screenshot set.

Rendered frame intervals, native release on Linux Omarchy 4.0.6/kernel 7.2.8-9-omarchy,
Ryzen 9 7900X, Radeon RX 7900 XT (RADV NAVI31), Vulkan/Mesa 26.2.5-arch1.1. Physical resolution
1776×1119. Each fresh launch uses 20,000 GPU wandering vehicles, Studio/Aurora, glider Model colors,
+Z forward/+Y up, model/render size/length/width all 1, default camera orientation with Frame world
at 1 s, AutoNoVsync, auto-frame off. Built-in PCG spawn starts at frame zero; no explicit seed
setting, real interactive deltas and nondeterministic GPU scatter. Warm up five seconds, measure ten,
exit at 16. No screenshots during or adjacent to measurement. External hyprctl focus/size monitoring
every 0.5 s found no violations in either measurement window. No other build/test/app overlapped.

| Run | Frames | Mean ms | p50 ms | p95 ms | FPS | Load before launch (1/5/15 min) |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| 1 | 9418 | 1.062 | 0.979 | 1.392 | 941.85 | 1.55 / 1.87 / 1.36 |
| 2 | 9434 | 1.060 | 0.983 | 1.375 | 943.39 | 2.20 / 2.00 / 1.41 |

Average per-run mean is 1.061 ms. These include CPU/GPU/presentation overlap and desktop scheduling,
not isolated GPU time or a controlled comparison against earlier stages. measure-monitor.py,
measure-{1,2}.scenario, measure-{1,2}.log, focus-{1,2}.json and uptime-{1,2}.txt reproduce the evidence.
Full tests/build/clippy logs are in the same artifact directory. Source checks confirm unchanged
Cargo.toml/Cargo.lock, gpu_flock.rs, octree.rs, target.rs, GPU simulation shader, GlobalState defaults,
historical review records and umbrella plan. No outstanding Stage 3 blockers; owner review remains.

Additional Stage 6 source evidence: bevy_image-0.20.1/src/image.rs:1589 (Image::from_buffer),
bevy_pbr-0.20.1/src/render/mesh.wesl:15 (first_vertex_index subtraction),
bevy_internal-0.20.1/Cargo.toml's 3d feature enables png; its jpeg feature is separate.
No texture decoding or animation path was implemented in Stage 3.

### Stage 6 evidence (2026-10-31)

Artifact root:
`/tmp/claude-1000/-home-martines-Projects-autonomous-characters-3d/af1dacb4-dc47-4bb5-8e8c-02c6cd1b95e8/scratchpad/shots/stage6/`.

Checks, all sequential after cargo/rustc/autonomous process checks, CARGO_BUILD_JOBS=4:

- cargo fmt --all -- --check: pass.
- cargo test --release --locked: 89 passed, zero failed, six ignored (0.15 s).
- cargo build --release --locked: pass, zero compiler warnings.
- cargo clippy --release --locked --all-targets: seven existing warnings, zero new warnings.
- python3 scripts/test_make_example_models.py: five passed, including byte reproduction, geometry,
  skin inputs/clips and distinct material factors. Original glider bytes remain unchanged.
- One permitted GPU regression run, exact owner command with gpu_flocking_benchmark skipped:
  four passed, zero failed (0.32 s). Stable phases survive coarse/fine sorts, instance transport and
  paused buffer growth. Existing GPU parity/separation/spawn tests still pass. The final subsequent
  CPU-only rotation overflow guard and message/tooltip edits do not change those tested kernels.
- python3 scripts/smoke-native.py: release survived ten seconds without startup/asset errors.
- git diff --check: pass. No ignored benchmarks were run. No app remains after each teardown.

Final generated assets (all original work under repository terms):

- bird.glb: 7,532 bytes, SHA-256 17e931aa1f96fe2126ddf753fcecbe9f48bff3ae36b42a30f14dc2b2192672d4.
  24 triangles / 72 GPU vertices; Flap 31 samples / 71,424 bytes, Glide 61 / 140,544 bytes.
- material-glider.glb: 2,956 bytes, SHA-256 00215a44f5a28aabcd99b28a17341144e1ea57f63a4b69be02af77be5de77d3e.
  24 triangles / 62 GPU vertices; polished gold hull (1 / 0.12), matte blue wings (0 / 0.95).
- glider.glb stays SHA-256 3ef0d93eaf7d78e26d359d2e7cddf3378fbe2d4b84232af5c7610618e98320d4.

Dependency/source evidence: gltf-1.4.1/src/animation/util/mod.rs exposes channel times/TRS output
readers; skin/mod.rs exposes joints and inverse bind matrices (identity when absent). MeshPipeline
specialization already handles COLOR_0; custom material inputs use location 6 and exact indexing.
MeshAllocator draw base vertex is added by draw_indexed and explicitly subtracted before storage
fetch, as Bevy's mesh.wesl subtracts first_vertex_index. Rust/WGSL instance stride is 48; BoidData
remains 64. The pre-change shader read only separation.xyz, so w was available for stable phase.
Shader diffs preserve every steering/force expression and all original random draws.

Desktop captures (physical 1776×1119), all visually inspected:

- bird-pose-1.png, bird-pose-2.png: 200 GPU vehicles, camera yaw 0 / pitch 17.28 / distance 180,
  render size 2.5, model scale/proportions 1, simulation speed 0.25 and animation speed 4. Two times
  0.8/1.1 seconds show changed wing poses and varied phases across the same running flock.
- bird-paused-1.png, bird-paused-2.png: pause at 1.4 seconds, capture at 1.8/3.8. magick compare
  -metric AE on crop 980x640+420+100 (excluding every UI panel/FPS) returns 0 (0).
- error-previous-bird.png: CUBICSPLINE rejected with feature, JSON path and recovery guidance visible
  at the top of the menu. The same crop versus bird-paused-1.png also returns AE 0 (0), proving
  the prior mesh/clip/pose remains rendered. pixel-comparisons.json records exact commands/results.
- bird-glide-clip.png: a valid named clip reload recovers from the error while paused; pose changes
  through appearance only. Clip controls and frame/memory/speed information remain in Vehicles.
- multipart-materials.png: 200 GPU vehicles, same close-up scale/framing, Model colors with the
  gold metallic hull and matte blue wings.
- bird-gpu-20000.png: animated creature, neutral size/proportions, running GPU flock, Frame world.
- bird-octree.png: 200 CPU Octree vehicles, close-up, same shared baked mesh and independent phases.

closeup.scenario exits at 7 s, materials.scenario at 2 s, gpu-octree.scenario at 6 s. All log clean
scripted exits; no shader or runtime warnings/errors appear. The expected CUBICSPLINE import error
is an informational recoverable status. trial-* captures/logs preserve the initial shader/pause probe.
No persistent desktop configuration was changed.

Timings use Stage 4's method on Ryzen 9 7900X, RX 7900 XT (RADV NAVI31), Vulkan/Mesa
26.2.5-arch1.1, Linux Omarchy 4.0.6/kernel 7.2.8-9-omarchy. Native release, exact floating client
1280×720, immediate presentation, MSAA 4, GPU wander, Studio/Aurora. Fixed yaw 0 / pitch 17.28;
Frame world at 1 s gives distance 2851.546 at 20k and 4876.075 at 100k. Neutral model/render
scale and proportions; simulation/animation speed 1, first Flap clip for animated bird. Each fresh
launch uses six seconds warmup, twelve seconds measured, exit at 19. No screenshots overlap samples.
GPU spawn uses the existing frame-zero PCG sequence; real deltas/scatter vary, no explicit seed.
Before each launch, require load <=3.0, idle GPU <=5%, no cargo/rustc/app/competing paint process.
Monitor focus, exact size/floating state and competing GPU jobs every 0.5 s; all accepted cases pass.
These are uncapped frame intervals, not isolated GPU time or monitor refresh rates.

| Case | Count | Run | Frames | Mean ms | p50 ms | p95 ms | FPS | Load (1/5/15 min) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| stride32 / dart | 100,000 | 1 | 11942 | 1.005 | 0.967 | 1.275 | 995.13 | 0.48 / 0.86 / 0.90 |
| stride32 / dart | 100,000 | 2 | 12023 | 0.998 | 0.965 | 1.231 | 1001.93 | 1.11 / 0.99 / 0.94 |
| stride48 / dart | 100,000 | 1 | 11876 | 1.010 | 0.973 | 1.280 | 989.70 | 0.45 / 0.60 / 0.58 |
| stride48 / dart | 100,000 | 2 | 12012 | 0.999 | 0.970 | 1.228 | 1001.00 | 0.83 / 0.68 / 0.61 |
| animated-skinned / bird | 20,000 | 1 | 11633 | 1.032 | 0.984 | 1.313 | 969.44 | 0.95 / 1.20 / 0.88 |
| animated-skinned / bird | 20,000 | 2 | 11702 | 1.025 | 0.974 | 1.323 | 975.13 | 1.32 / 1.28 / 0.92 |
| animated-skinned / bird | 100,000 | 1 | 11448 | 1.048 | 0.997 | 1.366 | 954.01 | 1.70 / 1.38 / 0.96 |
| animated-skinned / bird | 100,000 | 2 | 11637 | 1.031 | 0.996 | 1.273 | 969.72 | 2.20 / 1.53 / 1.02 |

Dart before/after averages are 1.0015 / 1.0045 ms: +0.003 ms (+0.3%), smaller than the per-run
spread. This static control covers transport plus added binding preparation; it does not isolate
GPU memory bandwidth or establish a statistically significant regression. Animated bird means are
1.032/1.025 ms at 20k and 1.048/1.031 ms at 100k. Eight accepted records are in accepted-results.json;
full fields/logs/uptime/window observations and timings.md reproduce them.

The first four cases labeled animated-* accidentally selected the built-in Bird through the reused
Stage 4 runner. Both launcher processes had imported the old code before correction. These are
rejected as animation evidence and retained in rejected-results.json/rejected-builtin-bird.txt and
original logs. The corrected animated-skinned-* scenarios explicitly set model assets/models/bird.glb
and require Loaded bird.glb plus Custom/Model colors metadata. No rejected number appears above.

A final source audit verifies unchanged Cargo files, historical reviews, GlobalState defaults,
CPU steering/integration, GPU force expressions and presentation clock advancement. HEAD stays
2ec4dbe. All work remains uncommitted for the orchestrator.

## Interfaces and Dependencies

Stage 3: VehicleModelPlugin/VehicleModels resource, source-axis/settings types, importer and mesh result.
VehicleShape gains Custom; ColorMode gains ModelColors. Model scale is baked into imported mesh,
so Stage 3 did not grow the 32-byte instance stream or appearance uniform. No crates added.

Stage 6: vehicle_animation.rs owns Channel sampling, skin matrices, BakedAnimation and budgets.
LoadedModel carries optional Arc-backed baked data and clip metadata. ImportedModel carries material
factors and deformation provenance, with both included in exact indexing when applicable.
AnimationRender extracts the immutable frames, selected mesh and bounded f64 playback accumulator.
Group 3 binding 0 remains appearance, binding 1 is read-only baked storage, binding 2 is 32-byte
playback/geometry metadata. The mesh-specific pipeline key adds ANIMATED only for the selected
animated mesh; MODEL_MATERIALS depends on its material vertex attribute. MeshAllocator's base vertex
is sent in metadata. Draw instances grow to three Vec4s (48 bytes); GPU BoidData stays 64 bytes.
SimulationClock exposes its existing delta read-only. No simulation parameter, default, force or
random steering draw changes; phase hashing uses an independent hash. No dependencies added.

---

Revision notes:

- 2026-10-31: Initial Stage 3 implementation plan; Stage 6 explicitly deferred.
- 2026-10-31 17:38Z: Stage 3 implemented and verified; evidence recorded for orchestrator review. No commits by worker.

- 2026-10-31 19:22Z: Stage 6 authorized. One-draw material attributes replace the memo’s per-primitive recommendation; implementation and validation pending.

- 2026-10-31 19:55Z: Stage 6 implemented and verified, budgets/limits and accepted/rejected evidence recorded. No commits by worker.
