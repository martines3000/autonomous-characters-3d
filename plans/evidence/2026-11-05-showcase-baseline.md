# Nature showcase baseline and art target

This record accompanies A0 of the [approved showcase plan](../completed/2026-11-05-polished-flock-web-showcase.md). Captures are observations by the development agent, not independent owner or new-viewer feedback.

## Reproduction and reference machine

- Source: `3088a34` (application code), with the approved planning-only commit `09c2646`.
- Native release build, pinned Rust 1.99.1 / Bevy 0.20.1, Linux `7.2.8-9-omarchy`, AMD Ryzen 9 7900X, AMD Radeon RX 7900 XT. Browser installed: Chromium 155.0.8059.84, not yet validated at this milestone.
- The compositor gives the requested 1280×720 native window a physical 2131×923 area. All baseline stills use that actual size. Resized review images retain the original aspect ratio.
- `AC3D_SCRIPT` drives screenshots and fixed-step recordings. Scripts do not seed or reseed vehicles. GPU scatter and real-time settling vary; camera comparisons use matched commands, not a claim of identical boid positions.
- Large raw frames, scripts and logs: `/home/martines/Projects/ac3d-evidence/2026-11-05/`. A permanent review-evidence link will be recorded before handoff.
- No other app, Cargo build or benchmark runs during measured sessions. All commands use `/tmp/ac3d-heavy.lock`; Cargo uses four jobs.

The audit first captures unmodified startup, then every Showcase. Each Species/hunt is applied after `showcase calm`, with an appropriate explicit environment and `frame`, so inherited camera/environment problems are visible rather than hidden. All fourteen original choices were inspected.

## Baseline findings and disposition

| Existing choice | Disposition | Observed issue or reason |
| --- | --- | --- |
| Showcase: Starling murmuration | Retune into public Murmuration | Best existing animal silhouettes and layered depth. The scene uses two invisible orbiting flee fields; remove unexplained threats and set a deliberate opening view. |
| Showcase: Fish school | Lab only | Close Director view can point mostly at the caustic sand, with a few fish occupying the center. Public Reef and Shark scenes need independent framing. |
| Showcase: Swallow formation | Lab only; artistic experiment | Sphere/Helix is a geometric animation, not natural swallow behavior. A red target remains visible even with UI hidden. |
| Showcase: Neon swarm | Lab only | Attractive abstract color cloud, but unrelated to the selected nature direction. Keep as an experiment. |
| Showcase: Calm | Lab only; reference defaults | Tiny abstract flock, grid, wall marks, target and large diagnostics dominate startup. Keep unchanged as the internal reference bundle. |
| Species: Sardine bait ball | Lab only | Invisible predators generate the drama. The explicit Shark scene supplies visible roles instead. |
| Species: Tuna stream | Lab only; rename displayed label | Loads `fish.glb`, not `tuna.glb`. Preserve legacy scenario names but label its generic fast-fish behavior honestly. |
| Species: Reef drift | Retune into public Reef drift | Distinct slow, loose swimming fits a quiet scene. World-fit framing makes individual fish invisible; use an immersed camera and a lower default tier. |
| Species: Starling murmuration | Retune into public Murmuration | Same invisible-threat problem; behavior-only application can inherit underwater lighting or an incompatible camera. |
| Species: Swallow hunt | Lab only | Fast independent darting is useful for experiments; the name does not describe a visible predator/prey encounter. |
| Species: Rook roost | Lab only | Correct rook asset and distinct heavy motion, but adds another bird choice before the first three scenes meet the quality bar. |
| Hunt: Shark attack | Retune into public Shark hunt | Correct shark/sardine silhouettes and existing pursuit mechanics. At inherited world-fit framing the three sharks are not readable. |
| Hunt: Falcon strike | Lab only | Correct bird roles, but world-fit framing hides the falcons and the setup inherits environment/camera. Keep for later focused composition. |
| Hunt: Tuna raid | Lab only | Real tuna hunters are present, but twelve simultaneous predators are busy and inherited framing hides their role. |

## Selected visual target

Favor three complete scenes: sunset Murmuration, quiet underwater Reef drift, and a readable Shark hunt. Open with recognizable animals across the central view, layered near/far motion, warm sky or clear water, and restrained lighting. Leave volumetric light and motion blur off until a matched comparison justifies their cost. Existing low-poly animals already have useful silhouettes and coordinated animation; the baseline does not establish a need for a Blender asset change.

The visitor HUD should expose scene, Low/Medium/High population, pause, camera, reset view and brief help. A single Lab entry contains the original tools. FPS, targets, grid diagnostics, imports, backend choices and numeric steering controls start hidden. Reduced camera motion must be available. Population tiers change counts, not rendering quality.

Initial performance target: 60 FPS at 1920×1080 on the recorded desktop GPU. This is a target, not a browser or other-device claim. Select tier counts only after repeat measurements. Report frame intervals, resolution, warmup and load conditions; do not interpret a vsync cap as spare GPU capacity.

## Validation at baseline

`cargo fmt --all -- --check`, `cargo test --release --locked` and `cargo build --release --locked` pass with `CARGO_BUILD_JOBS=4`. Tests: 204 passed, 22 ignored. The release build has no compiler warnings. Baseline startup and all fourteen scripted choices complete without startup/asset errors; rendered captures prove asset loading beyond a ten-second smoke.

The fixed-step baseline recordings cover six simulated seconds each of Starling and Reef, and 24 simulated seconds of Shark. The shark clip remains unreadable even after the legacy Close-up command: the camera still shows mostly empty water and sand. It does not prove visual pursuit/scatter acceptance. Existing regression tests cover pursuit/panic mechanics; the improved public scene still needs a readable complete encounter recording.

Two independent native runs use uncapped presentation, no capture during timing, four seconds or more of settling per scene, and eight-second sample windows. Settings remain fixed within each sample. Timings are rendered frame intervals, not GPU timestamps. Startup loads and environment bake time are outside these windows. The machine's load average before launches was 9.00 / 6.47 on 24 logical CPUs; unrelated activity cannot be ruled out. Camera trajectories and unseeded settling vary, so these numbers establish a baseline range rather than a speedup claim.

| Baseline scenario | Count | Run 1 mean / p95 / max (ms) | Run 2 mean / p95 / max (ms) |
| --- | ---: | --- | --- |
| Starling Showcase, Director | 40,000 | 3.390 / 4.309 / 6.550 | 3.443 / 4.068 / 4.663 |
| Reef Species, underwater close-up | 3,000 | 1.290 / 1.718 / 3.422 | 1.197 / 1.557 / 2.481 |
| Shark attack Species, underwater close-up | 30,000 + 3 sharks | 3.684 / 6.174 / 8.124 | 2.508 / 4.991 / 9.076 |

Exact commands and end-of-window metadata are retained with the scripts/logs in the evidence package. No browser or production readiness is claimed here.
