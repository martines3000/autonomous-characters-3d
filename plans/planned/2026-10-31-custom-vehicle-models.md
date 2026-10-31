# Import custom glTF vehicle models

This ExecPlan is a living document maintained according to `PLANS.md`. It covers Stage 3 now and
Stage 6 later under [the umbrella plan](2026-10-31-visual-showcase-and-swarm-features.md).
The orchestrator owns review and commits. This implementation worker must not commit or push.

## Purpose

Stage 3 lets users load local static GLB/glTF vehicles while the flock runs, correct their axes and
scale, and retain the previous selection after an import error. Multipart colors use the existing
single instanced draw. Stage 6 will revisit textures, separate material draws and animation.

## Progress

- [x] (2026-10-31 17:23Z) Read instructions, plans, renderer and dependency source evidence.
- [x] (2026-10-31 17:34Z) Stage 3: bounded importer, background loading, selection, axes, shader colors and scenarios.
- [x] (2026-10-31 17:34Z) Stage 3: deterministic original example asset and importer/preservation regression tests.
- [x] (2026-10-31 17:38Z) Stage 3: sequential release gates, Clippy, inspected desktop captures and repeated focused 20k frame measurements.
- [ ] Stage 6 (planned, not authorized for this worker): textures, separate primitive draws and baked animation.

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
- Stage 6 must decide texture formats and material/animation budgets before implementation.

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
performance promise. Stage 4 owns optimization. Stage 6 remains planned and unimplemented;
its unchecked Progress item is explicitly deferred to that owner-authorized future stage.

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
6. Stage 6 later: retain per-primitive meshes sharing ONE instance buffer; do not duplicate
   VehicleInstances (prepare_instance_buffer concatenates them). Appearance already uses group 3
   binding 0, so plan per-material uniforms/textures around that ownership. Decode images through
   Bevy Image::from_buffer (PNG enabled, JPEG not enabled). Bake CPU positions/normals per animation
   frame into storage, strip joints, fetch by vertex_index adjusted for MeshAllocator offset, hash
   per-instance phase and interpolate. 2,000 vertices × 30 frames × 32 bytes is about 1.83 MiB.
   Verify this recommendation against dependency sources again when Stage 6 starts.

## Concrete Steps

From repository root, check `pgrep -a cargo; pgrep -a rustc` before EVERY Cargo command and wait if busy.
Run sequentially with CARGO_BUILD_JOBS=4: fmt, release tests --locked, release build --locked,
Clippy --release --locked --all-targets. Then smoke-native.py, explicit exit-ended scenario runs,
process checks and git diff --check. Do not run ignored tests or benchmarks.

## Validation and Acceptance

Example counts/bounds and reproducible bytes; finite positions/normals/colors; all 24 perpendicular
signed axis pairs; hierarchy, indexed/nonindexed primitives and material/color multiplication;
precise feature/path rejection tests; stale/failed load recovery; whole GlobalState/GPU parameter
and entity/instance preservation. Release build adds no warnings; Clippy stays at seven existing.
Desktop evidence includes close-up wrong/correct axes, Model colors, running 20k GPU, Octree and
an error with the previous mesh visible. Frame evidence records machine, framing, seed policy and load.

## Idempotence and Recovery

The generator deterministically overwrites only assets/models/glider.glb. Background requests replace
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

## Interfaces and Dependencies

New VehicleModelPlugin/VehicleModels resource, source-axis/settings types, importer and mesh result.
VehicleShape gains Custom; ColorMode gains ModelColors. Model scale is baked into imported mesh,
so neither the shared 32-byte instance stream nor appearance uniform needs to grow. No crates added.

---

Revision notes:

- 2026-10-31: Initial Stage 3 implementation plan; Stage 6 explicitly deferred.
- 2026-10-31 17:38Z: Stage 3 implemented and verified; evidence recorded for orchestrator review. No commits by worker.
