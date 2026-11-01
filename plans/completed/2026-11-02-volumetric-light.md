# Volumetric environment lighting

This ExecPlan is a living document maintained according to `PLANS.md`.

## Purpose

Add optional sun shafts, especially through the fish school, and subtle environment sun glare. Startup remains unchanged.

## Progress

- [x] (2026-11-01 23:00Z) Read scene ownership and installed Bevy 0.20.1 volumetric API.
- [x] (2026-11-01 23:20Z) Implement controls, environment tuning, reversible effective shadows and Bloom composition.
- [x] (2026-11-01 23:20Z) Parsing and component lifecycle tests pass.
- [x] (2026-11-01 23:22Z) Locked gates: 175 release tests, 15 GPU tests, warning-free release build; Clippy has seven baseline warnings.
- [x] (2026-11-01 23:22Z) Quick 20k frame intervals measured.
- [x] (2026-11-01 23:24Z) Eight final comparison captures, fish-occlusion proof, exact paused Off round trip and native smoke pass.

## Surprises & Discoveries

- Bevy requires a camera `VolumetricFog`, sun `VolumetricLight`, shadow maps and a transformed unit-cube `FogVolume`.
- Flock shadow policy already has a 200k cap and shared animated depth/shadow passes.
- Bevy 0.20.1 `bevy_pbr/src/render/light.rs` sorts `(volumetric, shadow_maps_enabled, entity)` ascending, but only flags the first volumetric-count entries. The zero-lux Studio fill prevented sun scattering. Temporarily hiding that fill fixes it; visibility is restored on disable.
- Tiny fish shadows produce thin shafts. A cached 32³ R8 density texture strengthens sun-aligned underwater particle columns; the shadow maps still modulate the light.

## Decision Log

- Decision: Derive effective shadows from saved Shadows OR active environment volumetrics; retain the casting cap.
  Rationale: Fish and birds must occlude rays without overwriting the user's shadow preference.
  Date: 2026-11-02.
- Decision: Environment Off has no volumetric effect. Default 64 steps, editable 16–128. Glow retains its existing Bloom values.
  Rationale: Preserve default rendering, bound cost, and reuse camera effect ownership.
  Date: 2026-11-02.

- Decision: Hide only the unused Studio fill while environment volumetrics are active; save its visibility.
  Rationale: Work around the installed Bevy light-order bug without dependencies or changes to default rendering.
  Date: 2026-11-02.

## Assumptions and Open Questions

- Existing animated shadow passes provide the occluders. Native captures validate the installed shader path; fine silhouettes remain limited by cascade resolution and ray steps. No simulation or flock geometry changes.

## Outcomes & Retrospective

Controls, fog lifecycle, per-environment tuning and clean Glow/glare composition are implemented. Default rendering remains component-free. Visual captures show warm Sunset and cyan underwater scattering. Paused on/off round-trip pixels match exactly. Fish-caster removal changes the shafts (MAE 0.002615). Fine silhouettes remain limited by cascade resolution and step count. All required checks and the native smoke pass.

## Context and Orientation

`src/scene.rs` owns Appearance, shadow policy and Bloom. `src/environment.rs` owns aligned HDRI/procedural suns. `src/volumetric.rs` will own fog components. `src/scenario.rs` and `src/main.rs` expose controls. Only the render-policy seam in `src/instancing.rs` needs editing.

## Plan of Work

Add derived shadow policy, camera/volume lifecycle, per-environment medium settings, controls and tests. Validate and tune visually, then document results.

## Concrete Steps

Run sequentially with `CARGO_BUILD_JOBS=4`, each under `flock /tmp/ac3d-heavy.lock`: `cargo fmt --all -- --check`, `cargo test --release --locked`, `cargo build --release --locked`, `cargo clippy --release --locked --all-targets`, and `cargo test --release --locked gpu_ -- --ignored --test-threads=1 --skip gpu_flocking_benchmark`. Native scripts always end with `exit`. Run `git diff --check`.

## Validation and Acceptance

Parser accepts switches and bounded steps. Default has no volumetric components. On/off/environment/style transitions restore saved shadow preference, prepasses and Bloom. Capture UI-free paired fish up/side, Sunset starling and Midday views. Measure 20k Sunset and Underwater off/on frame intervals, not GPU benchmarks.

## Idempotence and Recovery

Components are derived from settings; disabling volumetrics restores saved policy without snapshots. Rerun scripts safely; screenshots overwrite only named artifacts. Revert feature changes through reviewed patches; do not reset the shared worktree.

## Artifacts and Notes

Captures: `/tmp/claude-1000/-home-martines-Projects-autonomous-characters-3d/af1dacb4-dc47-4bb5-8e8c-02c6cd1b95e8/scratchpad/shots/rays/`.

Quick live-app measurements: release, RX 7900 XT/RADV Vulkan + Ryzen 9 7900X, 3816×2110,
4× MSAA, UI hidden, immediate present, 20,000 vehicles, 64 ray steps, three-second windows.
Live GPU spawn/frame-index RNG; no fixed capture timestep or benchmark seed. Load average
before run: 1.49 / 1.72 / 1.39. These are frame intervals, not GPU timings.

| Scene | Off mean / p95 ms | On mean / p95 ms |
| --- | --- | --- |
| Sunset Starling (count overridden to 20k) | 2.679 / 2.982 | 8.686 / 9.002 |
| Underwater Fish (saved Shadows On in both) | 3.916 / 4.708 | 12.111 / 13.480 |

Final UI-free pairs: `underwater-{up,side}-{on,off}.png`, `sunset-starling-{on,off}.png`,
`midday-{on,off}.png`. Additional `occlusion-*.png` captures isolate flock casting;
`underwater-roundtrip-off-{before,after}.png` compare with pixel MAE 0.

Logs and scripts live beside the captures: `measure-20k.{txt,log}`, `capture-final.{txt,log}`,
and `check-{fmt,test,build,clippy,gpu}.log`. No GPU benchmarks ran.

## Interfaces and Dependencies

Appearance adds `volumetric`, `volumetric_steps`, `volumetric_active()` and `effective_shadows()`. No dependency changes.

---

Revision notes:

- 2026-11-02: Initial plan.
