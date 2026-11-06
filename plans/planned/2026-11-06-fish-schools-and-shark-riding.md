# Reef shoals, travelling shark school and shark riding

This ExecPlan is maintained according to `PLANS.md`. The owner authorized implementation on 2026-11-06 after manual review of the showcase and PR #20. It is a new follow-up; the original review-preparation goal remains complete. Prepare small tested PRs after #20. Do not merge, publish registry images, deploy, apply infrastructure, change DNS or trigger production workflows.

## Purpose

Reef drift should show several distinct loose shoals rather than isolated fish. Shark hunt should show a school travelling through the water, splitting around hunters and regrouping without remaining anchored in a central ball. Visitors should be able to ride a shark, change sharks, and retain Chase/First person and Pause. Native and WebGPU use the same simulation and controls.

## Progress

- [x] (2026-11-06 09:55Z) Recorded authorization, scope, ownership and validation strategy. Existing cause confirmed: Reef drift has strong wander/weak schooling; Shark attack sets a fixed origin prey target.
- [x] (2026-11-06 10:08Z) Recorded 25-second native before clips for Reef/Shark and two eight-second medium-tier timing windows per scene at 846×1390. Raw logs/captures are in the evidence directory; load averages are recorded.
- [ ] Prepare first PR with scene-specific Reef shoals and meaningful regressions. Implementation and complete pointer-chord replay pass 233 release tests; the 20 existing GPU regressions pass. All three Reef tiers have approximately 99% same-shoal nearest neighbors in the seeded probe. Headed acceptance passed before the final release/repress edge correction; the integrated final browser gate and PR publication remain.
- [ ] Prepare second PR with a travelling Shark school and behavior/performance evidence.
- [ ] Prepare third PR with CPU/GPU shark riding, visitor controls and recovery tests.
- [ ] Run integrated native/GPU, browser/container and independent review gates; publish evidence and exact PR order.
- [ ] Archive plan and update README, STATUS and morning review index.

## Surprises & Discoveries

- Existing `Group.target` replaces wander and wall avoidance with full-speed seek; static per-shoal targets would create smaller anchored balls. Moving, bounded guides and local alignment/cohesion must be evaluated in motion before accepting their tuning.
- GPU prey poses are sampled through an existing asynchronous 80-byte camera readback. Hunter poses live in the GPU arena in GPU mode and in `Hunters` in CPU modes. Shark riding must sample the actual simulated hunter rather than the CPU initial pose.
- Lab species application preserves non-hunter groups. Public Reef groups therefore require explicit release at Lab/preset entry; merely stopping the route clock would leak group targets into the reference presets.
- The first Shark moving-point trial improved median spacing and reduced panic, but sampled frames still showed a compact travelling ball and cropping. Centroid travel alone is insufficient: the fixed-target baseline already has substantial centroid motion under pursuit. Reject that trial as the final Shark behavior.

## Decision Log

- Decision: Replace Shark's point seek with an explicitly enabled soft capsule-shaped flow guide, shared by CPU/GPU, and follow the moving encounter in Scenic view while respecting Free/Ride/reduced motion.
  Rationale: Inside the corridor there is no force toward a central point; local alignment, separation, cohesion and predator responses determine the school structure. A soft boundary restores stragglers without assigning rigid positions. Existing Lab targets and Reef's validated guides retain their original paths.
  Date: 2026-11-06.
- Decision: Keep the work in three dependent PRs: Reef, travelling Shark school, shark riding.
  Rationale: Each behavior or camera outcome remains independently reviewable. Existing PRs 12–20 and deployment drafts remain unchanged.
  Date: 2026-11-06.
- Decision: First trial a scene-only simulation-clock controller using existing prey groups and bounded travelling guides, with scene-specific steering tuning.
  Rationale: This reuses both CPU/GPU kernels. Accept only after clips and measurements show loose shoals/travel rather than anchored mini-balls; revise the approach if observed behavior fails.
  Date: 2026-11-06.
- Decision: Retain Lab species/reference presets and all global defaults; activate public-school tuning explicitly through visitor scene application.
  Rationale: The owner requested changes to the public Reef and Shark scenes, not an unrelated retuning of all modes or the parked high-count follow case.
  Date: 2026-11-06.

## Assumptions and Open Questions

- Three reef shoals are the initial trial; compare visible separation, cohesion and motion at Low/Medium/High before fixing the count or tuning. These are art/behavior choices, not biological simulation claims.
- A smoothly travelling bounded school guide with reduced cohesion may produce a readable moving school using current kernels. If it still forms a persistent ball, adjust the model based on measured evidence rather than claim completion.
- Shark Ride uses the existing camera modes and random-target action with a simple Fish/Shark choice in Shark hunt. Handle zero hunters, scene/backend/count changes, pause, stale readbacks and model-size-dependent framing.
- Existing assets remain usable; use Blender only if a concrete asset defect appears.
- The recorded Linux/Chromium/RADV configuration is the browser acceptance target. Other browsers/devices remain untested.

## Outcomes & Retrospective

In progress. No new behavior, camera feature or measurement is claimed complete yet.

## Context and Orientation

`src/visitor.rs` applies complete public scenes and draws the visitor HUD. `src/species.rs` contains the preserved Lab behaviors. `src/swarm.rs` defines group roles and targets. CPU steering is in `src/vehicle.rs`; the shared GPU path is in `src/gpu_flock.rs` and `assets/shaders/gpu_flock.wgsl`. `src/hunter.rs` owns hunter simulation and rendering. `src/follow.rs` owns stable camera subjects/readback and `src/presentation.rs` owns camera controls. `src/scenario.rs`, native fixed-step capture, and `scripts/smoke-web.mjs` provide observable evidence.

## Plan of Work

1. Close the review app, record the existing Reef/Shark behavior and repeat medium-tier timing windows without other builds/apps.
2. Introduce a narrow public-school controller, explicit activation/reset/Lab lifecycle, three Reef prey groups and scene-specific steering. Validate phase, bounded routes, pause/speed, group roles and unchanged reference presets. Inspect before/after clips.
3. Extend that controller to Shark hunt using a soft moving flow corridor, preserving three hunters and existing pursuit/panic rules while removing point convergence and adjusting local school structure. Add CPU/GPU guide-force parity and ABI tests. Measure travel, local spacing, school elongation and all public-tier cost without hiding remaining limitations. Keep Scenic composition centered on the moving encounter; Free/Ride/reduced motion retain their ownership.
4. Integrate the independent shark Ride lane: actual CPU/GPU hunter samples, type-preserving random selection, correctly scaled cameras, stale-readback/lifecycle tests, visible Fish/Shark choice and scripted/browser controls.
5. Run the final gates once for the integrated result, publish immutable evidence, push PRs with exact dependencies, update review handoff and archive this plan.

## Concrete Steps

From the repository root, inspect `pgrep -af 'cargo|rustc|autonomous-characters-3d'` before heavy work. All Cargo/app/benchmark work uses `/tmp/ac3d-heavy.lock`, one owner and `CARGO_BUILD_JOBS=4` or fewer. Disposable probes use a separate target directory. Do not overlap native apps, builds or measured browser runs.

```sh
CARGO_BUILD_JOBS=4 cargo fmt --all -- --check
CARGO_BUILD_JOBS=4 cargo test --release --locked
CARGO_BUILD_JOBS=4 cargo build --release --locked
python3 scripts/smoke-native.py
git diff --check
CARGO_BUILD_JOBS=4 cargo test --release --locked gpu_ -- --ignored --nocapture --test-threads=1 --skip gpu_flocking_benchmark
```

Run native captures and repeated timings using README's scenario/record/measure commands. Run web packaging with the pinned wasm-bindgen CLI and reusable cache documented in the evidence; run the static container and headed browser commands from `docs/web-delivery.md`. Use native/WASM Clippy only to compare touched code against the seven known baseline warnings.

## Validation and Acceptance

- Reef shows several loose coherent shoals over time at each public tier, with visible spacing and bounded motion. The scene remains readable immediately and in Ride.
- Shark prey visibly travel rather than stay around the origin, split near hunters, and reform while moving. Before/after video supports the claim; an isolated still or a moving target alone does not.
- School guide phase uses simulation time, freezes during Pause, resets on public scene/tier application and stops owning Lab presets. Public scene/group roles, hunters and finite steering remain correct.
- Fish/Shark Ride selection works in Chase and First person, including N/Another animal, Pause, empty hunters and scene/backend/count changes. GPU samples reflect current hunter motion; stale samples cannot overwrite a new subject.
- Native/GPU tests and headed browser acceptance pass. Record real source, hardware/configuration, resolution, population, warmup and repeated timing distributions. Make no claim for untested devices or biological accuracy.
- Small PRs include images/videos, validation, measured cost, exact dependencies and remaining limits. No production action occurs.

## Idempotence and Recovery

Each public application starts a fresh school phase and existing flock restart. Repeated scene/tier selections are safe; entering Lab or choosing a Lab preset releases school ownership. Camera changes do not restart the simulation. Revert each PR in reverse order to restore prior tuning/camera behavior; deployment/image state is unchanged.

## Artifacts and Notes

Parent review source: `345ce0eb1450ff4c22b11d1873eafe7f3da982a9` (#20). Raw evidence goes under `/home/martines/Projects/ac3d-evidence/2026-11-06-fish-schools/`; selected immutable evidence will be linked from PRs and the morning index.

Ownership: root owns visitor scenes/HUD, public-school controller, main scheduling, docs, evidence and all verification. `/root/scenes` owns shark camera sampling (`follow.rs`, presentation/scenario controls and the WGSL tracking entry point) in an isolated worktree after this plan commit. `/root/browser` researches steering constraints without edits/builds/apps. Coordinate any additional file ownership before editing.

## Interfaces and Dependencies

Reuse existing group targets, simulation clock, hunter storage, camera readback and static WebGPU delivery. No dependency/toolchain upgrade is planned. Exact controller and subject APIs will be recorded after tracing existing boundaries; avoid a generic configuration or path framework.

---

Revision notes:

- 2026-11-06: Initial owner-authorized follow-up plan after manual review and the tested Another animal control.
- 2026-11-06: Rejected the first Shark point-guide trial after native frames retained a compact ball and cropped the moving school. Extended only the explicitly enabled public Shark behavior to a shared soft flow corridor, with force/ABI parity tests and camera ownership checks.
