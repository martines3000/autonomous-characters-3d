# HDRI sky environments

This ExecPlan is maintained according to `PLANS.md`.

## Purpose

Add independent Off, Sunset and Midday environments with image-based lighting, aligned sun and horizon fog. Off preserves existing presentation.

## Progress

- [x] (2026-11-01 21:18Z) Read requested source, README, visual research and registry APIs.
- [x] (2026-11-01 21:24Z) Implement cached HDR conversion, CPU filtering and environment lifecycle.
- [x] (2026-11-01 21:34Z) Format check, 147 release tests, warning-free release build, Clippy at seven baseline warnings, 15 GPU correctness tests and native smoke pass.
- [x] (2026-11-01 21:50Z) Inspect all requested captures and prove a paused Off → Sunset → Off round trip is pixel-identical (matching PNG SHA-256).

## Surprises & Discoveries

- The flock already inherits Bevy's environment specialization and both PBR view bind groups.
- GeneratedEnvironmentMapLight filters every frame; CPU prefiltering avoids a render-world completion protocol and ongoing GPU work.
- Asynchronous environment-to-environment switches require tracking the applied variant per camera; selection change ticks alone miss delayed readiness. The lifecycle test covers this.

## Decision Log

- Decision: Environment stays independent of Style; Off loads no HDR assets.
  Rationale: Preserve defaults and allow a future Underwater variant without multiplying styles.
  Date: 2026-11-01.
- Decision: Use 512px Rgba16Float cubemaps, cosine diffuse convolution and GGX specular mip chains, cached per environment.
  Rationale: Filterable HDR textures with no dependencies or ongoing generation cost; loading may briefly stall once.
  Date: 2026-11-01.
- Decision: Use AgX and fixed EV100 10 for environments only.
  Rationale: Preserve cloud highlights and saturated sunset colors without exposure pumping during chase; restore original camera values on Off.
  Date: 2026-11-01.

## Assumptions and Open Questions

- Native captures show a smooth lower sky; no extra ground/ocean plane is needed. No open implementation questions.

## Outcomes & Retrospective

Delivered independent selectors, validated scenarios, cached 512px HDR sky/diffuse/GGX maps, aligned colored sunlight and horizon fog. Glow, chase, depth of field and motion work together. Off restores the original scene and camera; a paused native round trip produced identical PNG files. All required gates pass. First selection briefly blocks for CPU preparation (about 2–3 seconds on this machine); subsequent switches reuse the maps. No Cargo changes, GPU benchmarks or commits.

## Context and Orientation

`src/environment.rs` owns HDR loading, filtering and camera lifecycle. `src/scene.rs` owns the default rig and fog. `src/instancing.rs` inherits Bevy PBR view keys and bindings. `src/scenario.rs` validates scripts.

## Plan of Work

Wire appearance/UI/scenarios; preserve Off camera and shader keys; test direction conventions and capture five requested scenes.

## Concrete Steps

From the repository root, sequentially:

1. `CARGO_BUILD_JOBS=4 flock /tmp/ac3d-heavy.lock cargo fmt --all -- --check`
2. `CARGO_BUILD_JOBS=4 flock /tmp/ac3d-heavy.lock cargo test --release --locked`
3. `CARGO_BUILD_JOBS=4 flock /tmp/ac3d-heavy.lock cargo build --release --locked`
4. `CARGO_BUILD_JOBS=4 flock /tmp/ac3d-heavy.lock cargo clippy --release --locked --all-targets`
5. `CARGO_BUILD_JOBS=4 flock /tmp/ac3d-heavy.lock cargo test --release --locked gpu_ -- --ignored --test-threads=1 --skip gpu_flocking_benchmark`
6. Native showcase and smoke under the same lock, with exit-terminated `AC3D_SCRIPT` files.
7. `git diff --check`

## Validation and Acceptance

Test parsing, synthetic sun extraction, cube directions and Off camera/pipeline preservation. Inspect Sunset/Midday, chase/DoF, Glow and one million birds. Retain CC0 credits.

## Idempotence and Recovery

Conversion occurs once per selected environment; maps remain cached. Off restores the original camera and scene. No Cargo changes or commits.

## Artifacts and Notes

Captures: `/tmp/claude-1000/-home-martines-Projects-autonomous-characters-3d/af1dacb4-dc47-4bb5-8e8c-02c6cd1b95e8/scratchpad/shots/env/`.
Requested scenes: `sunset-20k-bird.png`, `sunset-chase-quarter-speed.png` (also warmer `sunset-chase-id42.png`), `midday-20k-bird.png`, `sunset-glow.png`, `sunset-1m-bird.png`.
Scripts/logs: `/tmp/ac3d-env-{showcase,roundtrip,preview}.txt`, `/tmp/ac3d-env-{tests,clippy,gpu,showcase,roundtrip}.log`.
`/tmp/ac3d-env-off-{before,after}.png` share SHA-256 `236bef42e8d715633073873f101d69f80dd2768d5715de2318e6d354ccf751d6`.

Quick release frame intervals on Ryzen 9 7900X / RX 7900 XT at 3816×2110, 20k Bird, Studio/Aurora, 2× visual size, fixed camera and immediate presentation: Off 0.938 ms; Sunset 1.266 ms. Three-second unseeded windows in one app, no GPU timing or benchmark; no performance claim beyond this sample.

## Interfaces and Dependencies

New Environment enum and private cache; existing HDR image support only. No dependency changes.
