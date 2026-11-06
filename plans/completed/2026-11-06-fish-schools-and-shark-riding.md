# Reef shoals, travelling shark school and shark riding

This ExecPlan is maintained according to `PLANS.md`. The owner authorized fish-school and shark-riding implementation on 2026-11-06 after manual review of the showcase and PR #20, then added improved web loading/progress and a reveal animation. It is a new follow-up; the original review-preparation goal remains complete. Prepare small tested PRs after #20. Do not merge, publish registry images, deploy, apply infrastructure, change DNS or trigger production workflows.

## Purpose

Reef drift should show several distinct loose shoals rather than isolated fish. Shark hunt should show a school travelling through the water, splitting around hunters and regrouping without remaining anchored in a central ball. Visitors should be able to ride a shark, change sharks, and retain Chase/First person and Pause. Native and WebGPU use the same simulation and controls.

## Progress

- [x] (2026-11-06 09:55Z) Recorded authorization, scope, ownership and validation strategy. Existing cause confirmed: Reef drift has strong wander/weak schooling; Shark attack sets a fixed origin prey target.
- [x] (2026-11-06 10:08Z) Recorded 25-second native before clips for Reef/Shark and two eight-second medium-tier timing windows per scene at 846×1390. Raw logs/captures are in the evidence directory; load averages are recorded.
- [x] (2026-11-06 16:07Z) Prepared ready [PR #22](https://github.com/martines3000/autonomous-characters-3d/pull/22): three Reef shoals, public/Lab lifecycle, pointer regressions and before/after video. Final all-tier same-shoal nearest fractions are 99.64–99.82%.
- [x] (2026-11-06 16:07Z) Prepared ready [PR #23](https://github.com/martines3000/autonomous-characters-3d/pull/23): shared translating/turning guide, coherent initial school volume, local recovery and Scenic framing. Rejected compact/over-dispersed trials; accepted a qualified travelling-school result from all-tier metrics and a 60-second clip.
- [x] (2026-11-06 16:07Z) Prepared ready [PR #24](https://github.com/martines3000/autonomous-characters-3d/pull/24): actual CPU/GPU hunter subjects, type-safe stale-readback rejection, scaled Chase/First person, Fish/Shark selection and target changes. Native/GPU/browser cases and two 12-second camera clips pass.
- [x] (2026-11-06 16:07Z) Prepared ready [PR #25](https://github.com/martines3000/autonomous-characters-3d/pull/25): actual streamed progress, named preparation, rendered-scene reveal, reduced motion and recovery. Nine loader tests, one packaging fixture and headed loading proof/video pass.
- [x] (2026-11-06 16:07Z) Integrated gates pass: 255 release tests, 24 real-device GPU checks, warning-free build, native smoke, seven existing Clippy warnings, 11 headed browser cases, healthy local Docker, 32 byte-identical clean-output files, repeated native windows and all nine browser tiers twice. Independent source/visual reviews complete; immutable evidence pushed.
- [x] (2026-11-06 16:07Z) Updated README, web delivery, STATUS and exact review order; archived this plan. All four PRs remain unmerged. Deployment drafts retain real-image and later health prerequisites.

## Surprises & Discoveries

- Existing `Group.target` replaces wander and wall avoidance with full-speed seek; static per-shoal targets would create smaller anchored balls. Moving, bounded guides and local alignment/cohesion must be evaluated in motion before accepting their tuning.
- GPU prey poses are sampled through an existing asynchronous 80-byte camera readback. Hunter poses live in the GPU arena in GPU mode and in `Hunters` in CPU modes. Shark riding must sample the actual simulated hunter rather than the CPU initial pose.
- Lab species application preserves non-hunter groups. Public Reef groups therefore require explicit release at Lab/preset entry; merely stopping the route clock would leak group targets into the reference presets.
- The first Shark moving-point trial improved median spacing and reduced panic, but sampled frames still showed a compact travelling ball and cropping. Centroid travel alone is insufficient: the fixed-target baseline already has substantial centroid motion under pursuit. Reject that trial as the final Shark behavior.
- Headed acceptance exposed a fast pointer-entry race: PreUpdate reads Egui's prior UI-pass hover state, while a press and movement can cross the HUD boundary before the next UI pass. Current-frame press-event positions now establish initial ownership; ordered full-release/repress and both chord orders have regressions.
- Independent review rejected the first soft-flow result as a complete non-ball solution: Medium/High still contracted into compact clouds. Legacy alignment targets maximum speed rather than matching neighbor speed, pushing fish into the capsule's leading end. Removing that propulsion, carrying the guide's angular velocity and using a gentler circular route improved elongation. Unbounded turn velocity at distant stragglers was then bounded to the nearest capsule point.
- Random cube positions and full-speed random headings are unsuitable initial conditions for a free-interior travelling guide. Public guided prey now need coherent, uniform-volume ellipsoid spawning with the guide's local velocity. Existing target/wander/formation spawning must retain its original draw consumption and behavior.
- At clean source `10acec7`, 250 native tests, 22 GPU regressions, nine loader tests, one packaging test and the full headed browser suite passed. The loading-specific headed proof captured actual partial bytes under a 2 MiB/s throttle, normal/reduced reveals and failure during reveal. Those earlier results did not establish later school corrections; the final integrated source was verified separately.
- Physical desktop input contaminated later Wayland gesture checks while the owner used the PC. A trace retained HUD ownership and frozen simulation time but recorded extra mouse/wheel events. No speculative product fix was made. Full strict acceptance passed in headed Xvfb/X11 on the AMD WebGPU adapter; normal Wayland loading/reveal passed separately. The virtual display requires Chromium's `--disable-vulkan-surface`; without it presentation stalled and that attempt was not accepted.
- A baseline worktree initially reused a newer shared-target test binary. Scenario metadata exposed the mismatch. Forcing compilation of the intended checkout produced the verified origin-target baseline; the cached attempt is excluded. AGENTS now requires source-path/test-metadata checks when sharing Cargo targets across roots.

## Decision Log

- Decision: Add the owner-requested web loader/reveal as a fourth independent PR after the simulation/camera slices. Show actual bytes or determinate progress only when a trustworthy total exists; use named preparation stages otherwise. Prefer a short fade/subtle canvas reveal and respect reduced motion.
  Rationale: This improves the cold-load experience without inventing a global percentage or fighting the 3D camera. Loading/error/retry and device-loss states must remain readable.
  Date: 2026-11-06.
- Decision: Replace Shark's point seek with an explicitly enabled soft capsule-shaped flow guide, shared by CPU/GPU, and follow the moving encounter in Scenic view while respecting Free/Ride/reduced motion.
  Rationale: Inside the corridor there is no force toward a central point; local alignment, separation, cohesion and predator responses determine the school structure. A soft boundary restores stragglers without assigning rigid positions. Existing Lab targets and Reef's validated guides retain their original paths.
  Date: 2026-11-06.
- Decision: Keep the work in four dependent PRs: Reef, travelling Shark school, shark riding, web loading/reveal.
  Rationale: Each behavior or camera outcome remains independently reviewable. Existing application PRs 12–20 and deployment specifications remain unchanged; draft descriptions link the updated review order.
  Date: 2026-11-06.
- Decision: Start only explicitly guided prey within a loose ellipsoid, with translating/rotating guide velocity; retain free motion inside the soft envelope and ordinary predator responses. Increase guide recovery strength and widen the public encounter framing.
  Rationale: This supplies an immediate coherent school without assigning persistent slots or requiring a convergence ball. It also avoids hiding a poor startup behind a long warmup. Legacy spawning remains unchanged when guide navigation does not own the group.
  Date: 2026-11-06.
- Decision: First trial a scene-only simulation-clock controller using existing prey groups and bounded travelling guides, with scene-specific steering tuning.
  Rationale: This reuses both CPU/GPU kernels. Accept only after clips and measurements show loose shoals/travel rather than anchored mini-balls; revise the approach if observed behavior fails.
  Date: 2026-11-06.
- Decision: Retain Lab species/reference presets and all global defaults; activate public-school tuning explicitly through visitor scene application.
  Rationale: The owner requested changes to the public Reef and Shark scenes, not an unrelated retuning of all modes or the parked high-count follow case.
  Date: 2026-11-06.

## Assumptions and Open Questions

- Three Reef shoals are retained after all-tier metrics and footage show distinct coherent groups. This remains an art/behavior choice, not a biological simulation claim.
- Translation alone did not avoid cap packing. Explicit turn flow, coherent spawning and stronger recovery give a travelling school that sometimes compresses. Continuous elongation, hard confinement and visibility of every shark are not acceptance claims.
- Shark Ride uses the existing camera modes and random-target action with a simple Fish/Shark choice in Shark hunt. Handle zero hunters, scene/backend/count changes, pause, stale readbacks and model-size-dependent framing.
- Existing assets remain usable; use Blender only if a concrete asset defect appears.
- The recorded Linux/Chromium/RADV configuration is the browser acceptance target. Other browsers/devices remain untested.

## Outcomes & Retrospective

Four ready PRs (#22–#25) and immutable evidence are pushed after #20. The review handoff is complete.
The public scenes and camera changes share the Rust CPU/WebGPU implementation; ordinary Lab targets,
spawning, defaults and presets retain their paths. No dependency or asset update was needed.

Final native source `4fc7b7b` passes 255 release tests and 24 real-device GPU checks. Clean browser source
`c7fa288` differs only in a README line and produces release `596a0586e1010946ad95daab`; two fresh outputs
match in all 32 files. Eleven strict headed browser cases and separate Wayland loading/reveal proof pass.
See the evidence report for exact source, flags, hardware, loads, timing windows and copy/presentation limits.

Reef same-shoal nearest fractions are 99.64–99.82%. Shark mean elongation is 3.94/2.51/3.68, with wider
spacing and less panic than the pinned origin-target diagnostic. Medium has a real compact phase while
still travelling. A 60-second rendered clip shows later re-extension; some sharks/tails reach frame edges.
No continuous ribbon, hard confinement or biological accuracy claim is made. Different native rasters,
machine loads and virtual-display presentation prevent direct before/after speed comparisons.

Remaining prerequisites are review/merge, a separately authorized build/publication of the actual merged
source, a real registry digest, internal health and a later public rollout/verification session. Other
devices and internet/CRONA capacity remain untested. No production action occurred.

## Context and Orientation

`src/visitor.rs` applies complete public scenes and draws the visitor HUD. `src/species.rs` contains the preserved Lab behaviors. `src/swarm.rs` defines group roles and targets. CPU steering is in `src/vehicle.rs`; the shared GPU path is in `src/gpu_flock.rs` and `assets/shaders/gpu_flock.wgsl`. `src/hunter.rs` owns hunter simulation and rendering. `src/follow.rs` owns stable camera subjects/readback and `src/presentation.rs` owns camera controls. `src/scenario.rs`, native fixed-step capture, and `scripts/smoke-web.mjs` provide observable evidence.

## Plan of Work

1. Close the review app, record the existing Reef/Shark behavior and repeat medium-tier timing windows without other builds/apps.
2. Introduce a narrow public-school controller, explicit activation/reset/Lab lifecycle, three Reef prey groups and scene-specific steering. Validate phase, bounded routes, pause/speed, group roles and unchanged reference presets. Inspect before/after clips.
3. Extend that controller to Shark hunt using a soft moving flow corridor, preserving three hunters and existing pursuit/panic rules while removing point convergence and adjusting local school structure. Add CPU/GPU guide-force parity and ABI tests. Measure travel, local spacing, school elongation and all public-tier cost without hiding remaining limitations. Keep Scenic composition centered on the moving encounter; Free/Ride/reduced motion retain their ownership.
4. Integrate the independent shark Ride lane: actual CPU/GPU hunter samples, type-preserving random selection, correctly scaled cameras, stale-readback/lifecycle tests, visible Fish/Shark choice and scripted/browser controls.
5. Run the final gates once for the integrated result, publish immutable evidence, push PRs with exact dependencies, update review handoff and archive this plan.
6. In the independent web lane, improve the existing HTML/bootstrap loader, use real network/preparation signals, and reveal the actual ready scene with reduced-motion support. Integrate this fourth PR and include loading, failure, retry and reveal in the final headed-browser gate.

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
- Cold and warm browser loads show truthful progress or named stages, then reveal a rendered scene. Known byte totals produce bounded percentages; unknown/compressed lengths remain indeterminate with a byte count. Reduced motion skips spatial animation; failures/unsupported devices and Retry remain accessible and visible.
- Native/GPU tests and headed browser acceptance pass. Record real source, hardware/configuration, resolution, population, warmup and repeated timing distributions. Make no claim for untested devices or biological accuracy.
- Small PRs include images/videos, validation, measured cost, exact dependencies and remaining limits. No production action occurs.

## Idempotence and Recovery

Each public application starts a fresh school phase and existing flock restart. Repeated scene/tier selections are safe; entering Lab or choosing a Lab preset releases school ownership. Camera changes do not restart the simulation. Revert each PR in reverse order to restore prior tuning/camera behavior; deployment/image state is unchanged.

## Artifacts and Notes

Parent review source: `345ce0eb1450ff4c22b11d1873eafe7f3da982a9` (#20). Raw evidence remains under `/home/martines/Projects/ac3d-evidence/2026-11-06-fish-schools/`; selected immutable evidence is linked from PRs and the morning index.

Published evidence: [source, clips, logs and measurements](https://github.com/martines3000/autonomous-characters-3d/tree/8a70142b71f3793f72a4c6549bd307de22fdd06c/fish-schools).
The [follow-up report](../evidence/2026-11-06-fish-schools.md) and [morning index](../evidence/2026-11-06-morning-review.md)
record exact PR order, parent tips and deployment prerequisites. Original numbered PNGs and rejected
diagnostic trials remain local; published logs normalize only trailing blank lines.

Ownership: root owns visitor scenes/HUD, public-school controller, main scheduling, docs, evidence and all verification. `/root/scenes` owns shark camera sampling (`follow.rs`, presentation/scenario controls and the WGSL tracking entry point) in an isolated worktree after this plan commit. `/root/browser` researches steering constraints without edits/builds/apps. Coordinate any additional file ownership before editing.

Later assignments were coordinated explicitly: browser owned metrics/loader and isolated scene geometry;
scenes owned camera, shared turn force and guided spawning; deployment supplied read-only reviews.
Root integrated each lane, owned public framing/docs and ran all builds/tests/apps/measurements serially.

## Interfaces and Dependencies

`SchoolMotion` explicitly owns public-scene guide phase. Optional `SchoolGuide` adds center, velocity,
angular velocity, half-length and radius, a free-interior capsule force, and a guide-only initial-volume
sampler. CPU/WGSL share the contract and precedence; GpuParams appends flows/turns after existing fields
for a tested 720-byte layout. `FollowSubject::{Prey,Hunter}` extends selection and the existing 80-byte
sample contract with type/revision validation. The web catalog adds decoded byte sizes; bootstrap
exports pure progress/reveal helpers and distinguishes Rust readiness from usable reveal completion.
No dependency/toolchain change or generic path/configuration framework was added.

---

Revision notes:

- 2026-11-06: Initial owner-authorized follow-up plan after manual review and the tested Another animal control.
- 2026-11-06: Rejected the first Shark point-guide trial after native frames retained a compact ball and cropped the moving school. Extended only the explicitly enabled public Shark behavior to a shared soft flow corridor, with force/ABI parity tests and camera ownership checks.
- 2026-11-06: Added the owner's web loading/progress and reveal request as a fourth small PR. Existing school/riding work remains active; no production scope was added.
- 2026-11-06: Completed observed-behavior corrections, explicit turn flow and guided initial volume. Recorded desktop-input contamination, isolated headed proof and shared-target source validation. Closed at the four-PR review handoff; production remains outside this plan.
