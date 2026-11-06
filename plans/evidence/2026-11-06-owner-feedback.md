# Owner review follow-up — 6 November 2026

The owner praised the revised presentation and riding camera, questioned sparse Reef drift grouping and the persistent Shark hunt ball, and suggested a visible way to choose another animal.

[PR #20](https://github.com/martines3000/autonomous-characters-3d/pull/20) adds **Another animal · N** while riding. It uses the existing `Follow::next` action and preserves Chase/First person mode and Pause. The N shortcut also works with the HUD hidden. Merge after #19 using parent tip `64d7022d57da50bbe3715b7ccb5d1c676c8820bc`; the [morning index](2026-11-06-morning-review.md) records the complete order.

## Behavior diagnosis and proposed next work

- Reef drift intentionally uses weak alignment/cohesion and high wander (`Species::ReefDrift`), explaining scattered individuals. Proposed next trial: several loose, locally aligned shoals with visible spacing.
- Shark hunt inherits strong bait-ball cohesion and a fixed origin target from `Species::SharkAttack`, explaining the persistent central ball. Proposed next trial: a travelling school with local threat compression, visible splits and slower regrouping.

The fish changes are proposals, not implemented or biologically validated results. A new ExecPlan and before/after clips across public tiers, plus CPU/GPU parity checks, are required before changing steering defaults. They are tracked in [STATUS](../STATUS.md). No fish steering, public population, model, dependency or release-policy change is part of PR #20.

## Verification and evidence

Tested runtime source: `3da6830c30c6abcd46b6a083a742c728c35b9474`; subsequent changes update documentation only. Clean WebGPU release: `fb51caf213f583f7ea6b9a75`, Rust 1.99.1, wasm-bindgen 0.2.130. Local Docker image ID: `sha256:a1a0255c7b439082cbe1e585ce5a3535e3c51528cfc25061cef80886695ec721`; this is not a registry publication digest.

The prescribed native formatting, locked release tests/build, ten-second startup smoke and diff gate pass: **219 release tests**, 23 ignored, no compiler warnings. GPU steering regressions were not rerun for this UI-only change; their previous 20-test result remains tied to the earlier integrated source.

Shared WebGPU packaging, static Docker build and headed Chromium 155 acceptance pass (**11 cases**). The local container is healthy, runs as 101:101 and has a read-only root. The browser runner now changes subjects while paused in Chase and First person, verifies changed views and retained Pause/Ride, and confirms the First person-to-Off cycle remains intact. Existing scene/tier switching, HUD/world input ownership, keyboard focus, resize/high-DPI, reduced motion, failure, real visibility and GPU-loss/reload cases also pass on the recorded Linux/RADV configuration. Other devices and browsers remain untested. No dedicated performance benchmark was rerun; original timing measurements retain their original source.

[Immutable evidence and provenance](https://github.com/martines3000/autonomous-characters-3d/tree/372e05b3e229ff55dc22eeec346d87a41835f487/owner-feedback-2026-11-06) includes [native UI before](https://github.com/martines3000/autonomous-characters-3d/blob/372e05b3e229ff55dc22eeec346d87a41835f487/owner-feedback-2026-11-06/ride-before.png), [after](https://github.com/martines3000/autonomous-characters-3d/blob/372e05b3e229ff55dc22eeec346d87a41835f487/owner-feedback-2026-11-06/ride-after.png), [browser Ride](https://github.com/martines3000/autonomous-characters-3d/blob/372e05b3e229ff55dc22eeec346d87a41835f487/owner-feedback-2026-11-06/camera-ride.png), native/build logs and the [browser report](https://github.com/martines3000/autonomous-characters-3d/blob/372e05b3e229ff55dc22eeec346d87a41835f487/owner-feedback-2026-11-06/smoke-report.json). Camera subjects and real-time frame deltas vary between native screenshots, so these are UI comparisons rather than matched motion/performance captures.

Raw evidence: `/home/martines/Projects/ac3d-evidence/2026-11-06-owner-feedback/`. No merge, registry publication, production workflow dispatch, deployment, infrastructure apply or DNS mutation occurred.

An independent read-only review of runtime commit `3da6830` found no actionable issues; the reviewer ran no competing builds, tests or apps.
