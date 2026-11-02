# Bake environment lighting without blocking frames

This ExecPlan is maintained according to `PLANS.md`.

## Purpose

Keep the previous environment visible while first-selection sky and IBL baking runs on a worker. Scripted stills and recordings wait for a complete environment.

## Progress

- [x] (2026-11-02 09:35Z) Read environment, scene, capture and scenario code; identified synchronous convolution and early scene changes.
- [x] (2026-11-02 09:41Z) Measured baseline; implemented keyed worker jobs, applied scene selection and capture barriers.
- [x] (2026-11-02 16:21Z) Revalidated against master: 193 release tests and 17 ignored GPU regressions pass; warning-free build and seven baseline Clippy warnings. Reviewed volumetric resolution and added pending-medium regression.
- [x] (2026-11-02 16:26Z) Repeated matched-resolution first-switch measurement; inspected same-time captures, paused recordings, startup and rapid reselection; native smoke and diff checks passed. Plan closed.

## Surprises & Discoveries

- Environment selection affects scene surfaces, fog, bloom, sand and instancing beyond the camera sky. All consumers need the applied environment.
- `measure` reports p95 but a single long frame can fall outside it. Added maximum frame milliseconds to expose the stall.
- HDR loader inputs previously used default asset residency. Load CPU-only and move buffers into shared worker input, avoiding a large main-thread clone or unused GPU upload.
- Native paused-pair QA found that CPU-ready images can precede HDR/sand shader readiness. Capture now waits for uploaded maps and two pipeline-settled render reports; clocks hold during that wait.
- Showcase presets replace Appearance, so they must preserve its applied environment; a regression covers that path. New master species mutate vehicle settings only and preserve both environment fields. Lazy motion-blur history passes its GPU regression; native capture QA also covers Rook with blur and volumetric shafts.
- HDR uses `HdrTextureLoaderSettings`, configured through `AssetServer::load_builder().with_settings(...).load(...)`. The desktop check caught an initial settings-type mismatch.

## Decision Log

- Decision: Bake on demand on `AsyncComputeTaskPool`; preserve the old applied environment until completion.
  Rationale: No startup bake or added startup cost; one selection controls all rendering consumers.
  Date: 2026-11-02.
- Decision: Keep capture readiness separate from interactive rendering and reject failed capture requests explicitly.
  Rationale: Interactive frames continue; captures must not silently save the previous environment or hang on failed assets.
  Date: 2026-11-02.

## Assumptions and Open Questions

- Native same-time capture pairs validate uploaded sky/IBL maps and shader readiness. All three environment pairs, both three-frame paused recordings and the Off round trip are byte-identical; images were also visually inspected.
- Timing is a short desktop comparison, not an idle GPU benchmark. Record resolution, default 20k GPU mode, machine and load; simulation is unseeded.

## Outcomes & Retrospective

Delivered on-demand background baking, keyed cancellation, an applied environment shared by all rendering consumers, and still/video readiness barriers. Seven new regressions pass against master `479d9ec`; 193 release tests, 17 GPU regressions, formatting, warning-free release build, Clippy (seven baseline warnings), native smoke and diff checks pass. At default 20k and 3816×2110, repeated maximum frames fall from 2297 / 2010 / 120 ms to at most 6.003 / 3.433 / 2.448 ms for Sunset / Midday / Underwater. Same-time captures show complete looks, including Rook with volumetric shafts and motion blur.

No required work remains. Cancellation discards output but cannot preempt an already-running CPU bake; the previous look stays visible meanwhile. Measurements are short desktop checks, not GPU benchmarks; the accepted baseline predates the latest master changes. Shader readiness is validated on the available RX 7900 XT.

## Context and Orientation

`src/environment.rs::prepare_environment` now polls keyed worker jobs; `update_environment` only applies cached handles and lighting. `src/scenario.rs::run_scenario` dispatches commands in PreUpdate. `src/capture.rs` serializes PNG readback and fixes recording time through `src/presentation.rs`.

## Plan of Work

Measure the baseline. Move CPU preparation into keyed worker jobs, apply ready selections before rendering consumers, and gate still/video capture. Add focused lifecycle tests. Run required checks sequentially under the shared lock, measure again and update README/status.

## Concrete Steps

From the repository root, all heavy commands use `flock /tmp/ac3d-heavy.lock` and `CARGO_BUILD_JOBS=4`:

1. Build release, run `scripts/measure-environment.txt` through `AC3D_SCRIPT`, save baseline logs outside the repository.
2. Run `cargo fmt --all -- --check`, `cargo test --release --locked`, `cargo build --release --locked`, `cargo clippy --release --locked --all-targets`.
3. Run `cargo test --release --locked gpu_ -- --ignored --test-threads=1 --skip gpu_flocking_benchmark`.
4. Run `scripts/capture-environment.txt` through `AC3D_SCRIPT` with a fresh `/tmp/ac3d-env-captures`; compare paused pairs and inspect the images. Repeat the desktop measurement.
5. Run `flock /tmp/ac3d-heavy.lock env CARGO_BUILD_JOBS=4 AC3D_SCRIPT=/tmp/ac3d-env-smoke.txt python3 scripts/smoke-native.py` with `12 exit` in that script; run `git diff --check`.

## Validation and Acceptance

Pending work leaves camera, lights and scene environment unchanged. Completed current jobs switch together; obsolete jobs cannot apply. Same-time environment/capture commands wait. Existing tests pass, release has no warnings, and Clippy adds none beyond seven known warnings. Native runs end with scripted `exit`; no GPU benchmarks.

## Idempotence and Recovery

No dependencies or simulation defaults change. Measurement screenshots under `/tmp` can be replaced. Use fresh directories for recordings. Review the diff to reverse these edits; no commits or destructive Git commands.

## Artifacts and Notes

Reproduction: `scripts/measure-environment.txt`. Release profile, default 20,000 GPU boids, Studio/Dart, immediate presentation, 3816×2110, unseeded simulation, Ryzen 9 7900X / RX 7900 XT, Mesa 26.2.5 Vulkan. Baseline machine load before running: 0.02 / 0.03 / 0.08.

Before maximum / p95 frame milliseconds: Sunset 2296.995 / 1.709, Midday 2010.395 / 1.452, Underwater 120.257 / 1.852. Baseline log: `/tmp/ac3d-env-before.log`; preserved screenshots: `/tmp/ac3d-env-before/ac3d-environment-{sunset,midday,underwater}.png`. Baseline build log: `/tmp/ac3d-env-baseline-build.log`.

After, same profile/settings/machine/resolution as baseline, fresh process per run:

| Environment | Before max / p95 ms | After run 1 max / p95 ms | After run 2 max / p95 ms |
| --- | --- | --- | --- |
| Sunset | 2296.995 / 1.709 | 5.987 / 1.474 | 6.003 / 1.440 |
| Midday | 2010.395 / 1.452 | 3.433 / 1.434 | 3.141 / 1.475 |
| Underwater | 120.257 / 1.852 | 2.448 / 1.903 | 2.211 / 1.853 |

After logs: `/tmp/ac3d-env-after-matched-{1,2}.log`; pre-run loads: 1.58 / 2.09 / 1.65 and 1.99 / 2.15 / 1.69. `/tmp/ac3d-env-measure-sized.py` temporarily floats/resizes only the app window to match 3816×2110; desktop configuration is unchanged. An initial tiled 1699×1318 run is in `/tmp/ac3d-env-after-final.log` and is excluded from this comparison. No seed is available for native spawning; simulation defaults are unchanged.

Capture script: `scripts/capture-environment.txt`. Screenshots: `/tmp/ac3d-env-captures/{off-before,off-after,sunset,sunset-settled,midday,midday-settled,underwater,underwater-settled}.png`; three identical paused recording frames: `/tmp/ac3d-env-captures/video/0000{0,1,2}.png`. Exact PNG hashes match within every pair and the Off round trip. Inspected overview: `/tmp/ac3d-env-capture-contact.png`.

Integration script/log: `/tmp/ac3d-env-integration.{txt,log}`; Rook at 20k, volumetric on, motion blur on then off. All three environment pairs and three paused video frames match exactly. Images: `/tmp/ac3d-env-integration/`; inspected overview: `/tmp/ac3d-env-integration-contact.png`. Startup and rapid cancellation captures: `/tmp/ac3d-env-captures/{startup-sunset,reselected-sunset}.png`; logs `/tmp/ac3d-env-{startup,rapid}-final.log`; inspected overview `/tmp/ac3d-env-edge-contact.png`.

Final gate logs: `/tmp/ac3d-env-final-{test,build,clippy,gpu,smoke}.log`. All heavy commands took `/tmp/ac3d-heavy.lock` with `CARGO_BUILD_JOBS=4`; no GPU benchmark ran. No commit, stash, reset or baseline worktree was needed.

## Interfaces and Dependencies

Bevy tasks and owned CPU image results; no new crate dependencies. Appearance distinguishes requested and applied environment. Scenario and recording paths consult readiness.

---

Revision notes:

- 2026-11-02: Resumed after rebase, reviewed volumetric resolution, added its pending-switch regression, and completed matched-resolution measurements and native capture validation against current master.
