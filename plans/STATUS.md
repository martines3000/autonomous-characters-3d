# Plan status

## Current work

- AAA look track on `feat/boid-camera-and-fixes`: environment, opaque flock with shadows, hero models, cinematic camera (in progress).

## Prioritized future tasks

Items without a linked ExecPlan need one from `TEMPLATE.md` when starting.

1. **Follow-mode density at 100,000+ vehicles.** With GPU follow cap 16, 15–42% of sampled vehicles have a neighbor closer than the 1.4-unit dart width after the benchmark warmup (about 19% at 100k, 42% at 300k, 15% at one million). Complete separation does not hold against seek pressure in very large balls. This needs a behavior decision and its own plan, rather than a larger alignment/cohesion sample cap.
2. **Rendering budgets for heavy custom models and other GPUs.** [Stage 4](completed/2026-10-31-rendering-optimization.md) measured GPU wander at 1280×720 on the RX 7900 XT: low-poly Dart/Bird/glider cases average about 1 ms, while a 3,456-triangle subdivided glider costs about 24 ms at 100k, near or far; MSAA off barely changes that cost. Exact import indexing now retains 2,119 instead of 10,368 vertices with identical triangle inputs; heavy mean intervals fall from 5.28–5.30 to 2.87–2.89 ms at 20k and 23.87–24.10 to 11.54–11.59 ms at 100k, with 4× MSAA and identical geometry/shading. The historical M3 Pro close-100k observation (~59 FPS) is not remeasured. Fully faceted/unique-attribute models and other GPUs remain open; avoid approximate LOD, culling or extra passes without evidence.

Smaller follow-ups (no plan needed): speed retuning (`vehicle_max_speed` / `vehicle_wander_speed`) remains owner-by-eye after the outside-only wall fix. No defaults were retuned.

CI (finding F12) is not wanted by the owner.

- **Feature expansion roadmap — 2026-10-30:** see [report section 12](../review-report.html#feature-expansion). On 2026-10-31 the owner authorized seven stages (visuals, appearance, custom models, rendering optimization, presentation, animated models, swarm mechanics); see Completed.

## Original review findings

`review-report.html` audited commit `e7a53d7`. Status against `master` (the native modernization was merged in `3f8813a`), verified against source on 2026-10-30:

| ID | Finding | Status |
| --- | --- | --- |
| F01 | Zero-vector normalization and division create NaNs | Resolved: numerical-safety plan |
| F02 | Octree query visits one leaf and misses neighbors | Resolved: exact-octree plan |
| F03 | Acceleration, wander and target movement are frame-dependent | Resolved: frame-rate plan (60 Hz reference) |
| F04 | Coincident points cause deep octrees and slow rebuilds | Resolved: exact-octree plan (depth limit 16) |
| F05 | Cohesion distance measured after normalization | Resolved: numerical-safety plan |
| F06 | Per-axis speed clamp allows √3 × max speed | Resolved: numerical-safety plan |
| F07 | No explicit system order | Resolved in `262dc33` (chained spawn, cleanup, steering, integration) |
| F08 | Mass slider affects only new vehicles | Resolved: per-vehicle `VehicleMass` removed; steering reads `GlobalState::vehicle_mass` (test `mass_slider_applies_to_existing_vehicles`) |
| F09 | Benchmark disabled; visibility toggle affects only new vehicles | Resolved: benchmark plan (headless `flocking_benchmark`; checkbox removed) |
| F10 | Naive default mode is quadratic up to the 50,000 slider | Resolved: GPU default at 20,000, slider up to one million, complete separation with follow cap 16; M3 Pro headless steps 0.67 ms wander / 1.4 ms follow at 20k, 7.4 / 13.2 ms at 300k. Naive/Octree remain CPU references (sliders up to 50k); extreme follow density and close-up rendering cost are future tasks 1–2 |
| F11 | Historical dependencies do not build on current toolchains | Resolved in `21769fd` and `262dc33` (Bevy 0.20.1, Rust 1.99.1) |
| F12 | No automated tests or CI | Partly resolved: 33 non-ignored regression tests, 3 ignored GPU regression tests, and separate ignored CPU/GPU benchmarks; GPU runs need a real device outside restricted sandboxes; CI not wanted by the owner |
| F13 | Unused direct dependencies | Resolved in `262dc33` |
| F14 | Advisory matches in the lockfile | Resolved in `262dc33`: `cargo deny check advisories` passes for the native target |

## Completed

- [Opaque flock and sun shadows](completed/2026-11-01-flock-shadows.md) (2026-11-01): optional cascades, shared animated depth/normal/shadow passes, 200k casting cap and SSAO with reversible MSAA; Off round trip pixel-identical; 161 release and 15 GPU tests pass, Clippy remains at seven baseline warnings.

- [Cinematic Director and fixed-step capture](completed/2026-11-01-director-capture.md) (2026-11-01): five tracked shots, C/UI/scenario controls, subject DoF, shake and reversible flyby ramp; serialized numbered PNG capture with fixed simulation/camera timing. 152 normal and 15 GPU tests pass; five Bird stills and a verified 90-frame/30-fps MP4 captured.
- [HDRI sky environments](completed/2026-11-01-hdri-environment.md) (2026-11-01): independent Off/Sunset/Midday, cached 512px HDR sky and diffuse/GGX IBL, aligned sun, horizon fog, AgX; Off round trip is pixel-identical. 147 release and 15 GPU tests pass; Clippy remains at seven baseline warnings.

- [Natural vehicle motion](completed/2026-11-01-vehicle-motion.md) (2026-11-01): stable-up orientation, turn banking, procedural bird flap and fish swim on simulation time; positions and velocities stay bit-identical; Classic heading restores the old rotation exactly. Benchmarks deferred to an idle machine.
- [Glow style and boid-camera depth of field](completed/2026-11-01-glow-style-and-dof.md) (2026-11-01): optional HDR/additive bloom, speed emission/streak, Gaussian DoF with live focus, a small chase marker and UI-free capture. Studio stays default; old shaders compile out Glow. At 1667×1390 on RX 7900 XT, Glow adds about 0.28 ms at 20k/100k; DoF full-frame differences are within noise; heavy 100k Glider averages 11.656 ms. 135 normal and 14 GPU tests pass.

- [Boid follow camera](completed/2026-11-01-boid-follow-camera.md) (2026-11-01): optional Chase/First person, stable spawn IDs, asynchronous 80-byte GPU samples, direct CPU sampling, simulation-clock prediction and paused real-time orbit. B/N, scroll, UI and reproducible scenario controls; off adds no tracking dispatch/readback. 130 normal and 14 GPU tests pass. RX 7900 XT 1M wander tracking adds about 0.10 ms (2.2%); 20k app means 1.298 ms off / 1.328 ms chase at 881×1119.

- [Outside-only wall containment and group rebalance](completed/2026-11-01-wall-containment.md) (2026-11-01): 20k settled wander speed rises from roughly 29.3 to 39.2 units/s; maximum measured GPU overshoot is 23.399 units and the existing <50 assertion passes. Group-count edits spread existing members by spawn ID; Off handling and carried metadata remain intact.
- [Visual showcase, custom models and swarm mechanics](completed/2026-10-31-visual-showcase-and-swarm-features.md) (umbrella, 2026-11-01): seven stages on `feat/visual-showcase-and-swarm-features`, `e635c6d`..`a0d3ed7`, pushed, not merged; all new features optional and off by default; GPU steps unchanged with them off (1M wander about 4.6 ms, follow about 7.9 ms at cap 16).
- [Swarm mechanics](completed/2026-10-31-swarm-mechanics.md) (2026-11-01): cursor attractor/repulsor, 2–8 groups, up to 8 sphere obstacles, sphere/helix formations and up to 4 predators; one CPU/GPU contract with parity tests; identity and scene records share one storage arena; everything on at 100k costs 7.1 ms at cap 16 (40.6 ms exact). Known group-growth limitation resolved on 2026-11-01: enabling/group-count edits rebalance existing members by spawn ID; no need to wait for vehicle growth.
- [Presentation controls](completed/2026-10-31-presentation-controls.md) (2026-10-31): simulation clock, pause, 0.1–4× speed (ceil(speed) steps above 1×), real-time orbit and five camera presets.
- [Custom glTF vehicle models](completed/2026-10-31-custom-vehicle-models.md) (2026-10-31): static GLB/glTF import, then baked rigid/skinned animation with stable per-vehicle phases and per-part materials in one draw.
- [Rendering optimization](completed/2026-10-31-rendering-optimization.md) (2026-10-31): exact import vertex sharing halves heavy-model frame time (24 to 11.5 ms at 100k); no LOD, culling or MSAA change justified.
- [Visual foundation and vehicle appearance](completed/2026-10-31-visual-foundation-and-appearance.md) (2026-10-31): studio style, palettes, count-aware framing, vehicle shapes, proportions and color modes.

- [Flock on the GPU](completed/2026-10-30-gpu-flocking.md) (2026-10-30): GPU default at 20,000 / follow cap 16; M3 Pro headless wander/follow steps 0.67/1.4 ms at 20k, 7.4/13.2 ms at 300k, 26.1/44.9 ms at one million; default app about 107–120 FPS wandering, scaled target and camera; CPU parity and complete separation tested on the GPU. Large follow density and rendering cost remain above; in-bounds wall drag is resolved by the 2026-11-01 containment fix.
- [Keep vehicles apart when they follow the target](completed/2026-10-30-follow-mode-separation.md) (2026-10-30): separation pushes add up and grow with closeness; followers at 1,000 and 5,000 go from 100% overlapping to none; wander flocks keep their look with no overlapping pairs; follow-mode cost at large counts deferred.
- [Instanced vehicle rendering and a visual pass](completed/2026-10-30-instanced-rendering-and-visuals.md) (2026-10-30): one instanced draw for the flock; 30,000 vehicles 53 → about 80 FPS, 10,000 about 138 → 220 FPS; heading colors, directional light, fog, wall box and floor grid; separation retune investigated, follow-mode collapse deferred to future task 1.
- [What limits frame time at 30,000 vehicles](completed/2026-10-30-high-count-frame-limit.md) (2026-10-30): CPU critical path, not GPU; 30,000 vehicles at about 53 FPS and 10,000 at about 138 FPS at low machine load; no code change; GPU time unmeasurable with available tools.
- [Profiling-guided optimization](completed/2026-10-30-profiling-guided-optimization.md) (2026-10-30): compute pool 5 → 9 threads; sort- and allocation-free neighbor queries; 10,000 vehicles about 60 → 120–138 FPS and 30,000 about 36 → 53 FPS in the app (see the frame-limit plan for the corrected 30,000 numbers).
- [Scale the spawn region with vehicle count](completed/2026-10-30-count-scaled-spawn-region.md) (2026-10-30): spawn cube and walls scale with cbrt(count / 200); octree default; 1.8 ms per step at 5,000 and 3.7 ms at 10,000 vehicles; large flocks contained but cluster tightly over time.
- [Seeded headless benchmarks](completed/2026-10-30-seeded-headless-benchmarks.md) (2026-10-30): `flocking_benchmark` ignored test for both modes on identical seeded inputs; in-app benchmark removed; naive stays default (faster at 200 vehicles); octree 6.7× faster at 10,000 only when density is constant.
- [Frame-rate-independent simulation and target movement](completed/2026-10-30-frame-rate-independence.md) (2026-10-30): acceleration and wander scaled to a 60 Hz reference, target at 120 units/s, target input ordered before steering; probe shows matching seek and overlapping flock metrics at 30/60/144 FPS.
- [Exact octree radius queries](completed/2026-10-30-exact-octree-radius-queries.md) (2026-10-30): `query_radius` with bounds pruning, containing-octant insertion, depth limit 16, root built from vehicle positions; octree and naive accelerations match on a seeded world. Performance unmeasured.
- [More natural starting configuration](completed/2026-10-30-natural-starting-configuration.md) (2026-10-30): 200 vehicles in a 100-unit cube with sphere headings, walls at ±200 / ±200 / ±100, camera about 180 units out; probe shows the flock forms and stays near the origin over 30 s. Visual framing check deferred.
- [Numerical safety and regression tests](completed/2026-10-30-numerical-safety.md) (2026-10-30): finite steering for degenerate inputs in both modes, magnitude speed limit, cohesion arrival fix, shared flocking function, 8 new tests; starting-configuration recommendations with headless probe evidence.

Earlier work (review, native baseline, modernization, controls) is recorded in `review-progress.md`.
