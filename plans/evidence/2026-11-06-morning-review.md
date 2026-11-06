# Morning review — 6 November 2026

The approved review-only goal is complete: application changes are prepared in a tested PR stack, and CRONA workload/routing changes are prepared as drafts. All PRs remain unmerged. No default-branch change, image publication, production workflow dispatch, deployment, infrastructure apply or DNS mutation occurred.

Start with the [before/after evidence](https://github.com/martines3000/autonomous-characters-3d/tree/ece1b2adbae556fbc5cfdb2bee45728bc9d7b983) and the [complete validation report](https://github.com/martines3000/autonomous-characters-3d/blob/ece1b2adbae556fbc5cfdb2bee45728bc9d7b983/verification/FINAL-VALIDATION.md). Local static preview remains at http://127.0.0.1:8088 from the reviewed image; the documented Linux Chromium flags are required on this machine. Browser/encoder/simulation processes are closed.

## PRs and merge order

Merge each application PR into `master` in the order below. Current PR bases point to review parents, so rebase and retarget each child after its parent is squash-merged. The repository permits squash merges only. The review-handoff documentation follows the release tooling; it changes no runtime code.

| Order | PR | Repository | Status | Result / dependency |
| --- | --- | --- | --- | --- |
| 1 | [#12](https://github.com/martines3000/autonomous-characters-3d/pull/12) | autonomous-characters-3d | Ready | Approved plan, startup baseline, all fourteen preset dispositions and initial motion/timings. |
| 2 | [#13](https://github.com/martines3000/autonomous-characters-3d/pull/13) | autonomous-characters-3d | Ready | Complete public scenes, scene-specific tiers and explicit paused-safe flock restart. After #12. |
| 3 | [#14](https://github.com/martines3000/autonomous-characters-3d/pull/14) | autonomous-characters-3d | Ready | Compact visitor HUD, Lab, hidden diagnostics and shared pointer ownership. After #13. |
| 4 | [#15](https://github.com/martines3000/autonomous-characters-3d/pull/15) | autonomous-characters-3d | Ready | Warm animal-volume camera composition and legible HUD. After #14. |
| 5 | [#16](https://github.com/martines3000/autonomous-characters-3d/pull/16) | autonomous-characters-3d | Ready | Readable shark size/pass tuning; legacy Lab settings preserved. After #15. |
| 6 | [#17](https://github.com/martines3000/autonomous-characters-3d/pull/17) | autonomous-characters-3d | Ready | A1/A6 shared WebGPU runtime, assets, loading/errors, input and recovery. After #16. |
| 7 | [#18](https://github.com/martines3000/autonomous-characters-3d/pull/18) | autonomous-characters-3d | Ready | A7 reproducible web/static Docker tooling and manual publication. After #17. |
| 9 | [#1](https://github.com/martines3000/household-cluster/pull/1) | household-cluster | Draft | C1 namespace/workload/service/internal Ingress. Requires merged application, real published registry digest and pull proof. |
| 10 | [#1](https://github.com/martines3000/household-infra/pull/1) | household-infra | Draft | I1 hostname, Tunnel route and proxied CNAME. Requires healthy internal C1 workload and later guarded production plan/apply. |

The deployment gaps are deliberate review boundaries, not completed production tests. Perform publication and rollout only in the separately authorized release session.

### Squash-stack procedure

After merging a parent into `master`, fetch and rebase its child onto the new default branch, using the recorded parent tip below. Then push with a lease, retarget the PR to `master`, and inspect the resulting diff/checks before merging. Continue one child at a time.

```sh
git fetch origin
git rebase --onto origin/master <recorded-parent-tip> <child-branch>
git push --force-with-lease origin <child-branch>
```

| Child PR / branch | Recorded review parent tip |
| --- | --- |
| #13 `codex/showcase-scene-contract` | `6b9f1886e9f53fd56281b63b1ea49d1c0556b647` |
| #14 `codex/showcase-visitor-hud` | `8cf48c2ccdb293d130205d928b9c810b171a547d` |
| #15 `codex/showcase-composition` | `daf0a357a0c333d49401ef4d2e21aaa10bfe7919` |
| #16 `codex/showcase-hunt-readability` | `c35fbf1e54474e6ef1b77a39773356bf0c9c0954` |
| #17 `codex/showcase-webgpu` | `aa418f99410afcc6800dc4e91c5cdf812cb28676` |
| #18 `codex/showcase-web-release` | `da99886929c2e1944db7c3cd171286bd15313291` |

The eventual published source commit will differ from these feature tips after squash/rebase. Build and record that actual merged source; do not copy a review SHA into deployment provenance.

## Visual and behavior evidence

![Composed native first scene](https://raw.githubusercontent.com/martines3000/autonomous-characters-3d/ece1b2adbae556fbc5cfdb2bee45728bc9d7b983/verification/startup-ui.jpg)

- [Original startup](https://github.com/martines3000/autonomous-characters-3d/blob/ece1b2adbae556fbc5cfdb2bee45728bc9d7b983/baseline/startup.jpg), [five original Showcases](https://github.com/martines3000/autonomous-characters-3d/blob/ece1b2adbae556fbc5cfdb2bee45728bc9d7b983/baseline/showcase-contact.jpg), [nine original Species/hunts](https://github.com/martines3000/autonomous-characters-3d/blob/ece1b2adbae556fbc5cfdb2bee45728bc9d7b983/baseline/species-contact.jpg), and the [complete disposition audit](2026-11-05-showcase-baseline.md).
- [Final browser Murmuration](https://github.com/martines3000/autonomous-characters-3d/blob/ece1b2adbae556fbc5cfdb2bee45728bc9d7b983/browser/scene-murmuration.jpg), [Reef](https://github.com/martines3000/autonomous-characters-3d/blob/ece1b2adbae556fbc5cfdb2bee45728bc9d7b983/browser/scene-reef.jpg), [Shark encounter](https://github.com/martines3000/autonomous-characters-3d/blob/ece1b2adbae556fbc5cfdb2bee45728bc9d7b983/browser/scene-shark.jpg), and [real 2× DPI](https://github.com/martines3000/autonomous-characters-3d/blob/ece1b2adbae556fbc5cfdb2bee45728bc9d7b983/browser/visitor-high-dpi.jpg).
- [Six-second Murmuration](https://github.com/martines3000/autonomous-characters-3d/blob/ece1b2adbae556fbc5cfdb2bee45728bc9d7b983/native/murmuration-motion.mp4) and [Reef](https://github.com/martines3000/autonomous-characters-3d/blob/ece1b2adbae556fbc5cfdb2bee45728bc9d7b983/native/reef-motion.mp4) clips; [25-second hunt before](https://github.com/martines3000/autonomous-characters-3d/blob/ece1b2adbae556fbc5cfdb2bee45728bc9d7b983/native/shark-before-motion.mp4) and [after](https://github.com/martines3000/autonomous-characters-3d/blob/ece1b2adbae556fbc5cfdb2bee45728bc9d7b983/native/shark-motion.mp4), with [matched setup/source notes](https://github.com/martines3000/autonomous-characters-3d/blob/ece1b2adbae556fbc5cfdb2bee45728bc9d7b983/native/README.md).
- [Three recognizable sharks beside the disrupted school](https://github.com/martines3000/autonomous-characters-3d/blob/ece1b2adbae556fbc5cfdb2bee45728bc9d7b983/native/shark-encounter.jpg). Sampled full-size frames show local splits and repeated regrouping. Some predators leave or become obscured in the frame. Sparse stills do not measure acceleration or prove a continuous timer sequence; shared regression tests and the complete video support review.
- [Raw browser delivery video](https://github.com/martines3000/autonomous-characters-3d/blob/ece1b2adbae556fbc5cfdb2bee45728bc9d7b983/browser/browser-shark-padded.webm) proves twenty seconds of real rendered motion but includes CDP window padding. It is separate from the polished native behavior clip and from performance samples.

No asset defect warranted Blender changes. No owner or independent human taste approval is claimed; this is the concrete review package for that feedback.

## Verification and performance

Native integrated source `34d3e5dad6bea1be2e9ac797a65564d9248c4697` passes the prescribed fmt → locked release tests → locked release build → ten-second desktop smoke → diff check sequence. Results: **219 release tests passed**, **20 ignored GPU regressions passed**, no compiler warnings, and only the seven existing native/WASM Clippy warnings. The shared pointer fix has a red/green regression plus real browser coverage, including both mouse-button chord orders and numeric Lab focus consuming P/H.

Browser matrix 7 passes scene/tier changes, rapid switching, pause, help/Lab/H, Scenic/Ride/reset, UI/world gesture separation, keyboard focus, resize, real 2× DPI, reduced motion, missing assets, actual hidden-tab pause/return, and actual GPU-process crash/reload. [Exact matrix](https://github.com/martines3000/autonomous-characters-3d/blob/ece1b2adbae556fbc5cfdb2bee45728bc9d7b983/browser/smoke-report.json); [device-loss evidence](https://github.com/martines3000/autonomous-characters-3d/blob/ece1b2adbae556fbc5cfdb2bee45728bc9d7b983/browser/device-loss.jpg).

Reference machine: Ryzen 9 7900X, RX 7900 XT / RADV Mesa 26.2.5, Linux 7.2.8-9-omarchy. Native release, physical 2131×923, UI hidden, uncapped presentation, default Scenic camera, GPU, 1× simulation; eight seconds settling and eight seconds sampled per scene in two independent launches. Motion blur and volumetrics remain off. Load averages before launches: 0.03 and 0.19. PCG frame-0 spawning is retained; real-time frame deltas and atomic scatter are not fixed.

| Native medium scene | Prey | Mean ms, runs 1 / 2 | p95 ms, runs 1 / 2 | Max ms, runs 1 / 2 |
| --- | ---: | --- | --- | --- |
| Murmuration | 20,000 | 2.492 / 2.421 | 2.896 / 2.741 | 3.430 / 3.354 |
| Reef | 1,500 | 1.266 / 1.261 | 1.674 / 1.633 | 2.726 / 2.209 |
| Shark | 12,000 + 3 hunters | 1.813 / 1.918 | 2.175 / 2.641 | 3.098 / 3.380 |

Browser source `9bbe1387638448110bccd29565354b09813aa262`, release `a86835f2931b331568586ec7`: all **nine** scene/tier combinations measured **twice** at 1920×1080. Three seconds settling and six seconds sampled per window; 360/361 intervals each. Means **16.6658–16.6686 ms**, p95 **16.9–17.1 ms**, maxima **17.7–22.5 ms**, near the 60 Hz browser cap. These are application/browser intervals, not GPU timestamps or unused-capacity estimates. Baseline counts/camera paths differ, so no direct speedup is claimed. [All eighteen windows](https://github.com/martines3000/autonomous-characters-3d/blob/ece1b2adbae556fbc5cfdb2bee45728bc9d7b983/browser/measurements.json) and [native logs](https://github.com/martines3000/autonomous-characters-3d/blob/ece1b2adbae556fbc5cfdb2bee45728bc9d7b983/verification/native-measurements.json).

The browser runtime source remains reachable in the release branch. Later native changes fix test-only cubemap fixtures; runtime code is unchanged. Two clean output builds produce **32 byte-identical files**. Local static image ID: `sha256:41020c8ae352b8d3fbbdf0c761f56e93635b751ee538753fdeb191259ff436bd`. This is not a registry manifest digest.

## Supported proof and remaining prerequisites

The tested browser is Chromium 155.0.8059.84 using `/usr/lib/chromium/chromium` with `--enable-unsafe-webgpu --enable-features=Vulkan --use-angle=vulkan --ozone-platform=wayland`. The Arch wrapper overrides feature configuration. Flagless Chromium has no adapter here and shows the tested unsupported state. Windows/macOS, other engines/GPUs and mobile remain untested; no broader support claim is made.

Persistent-cache localhost loading: **1731.5 ms / 29,780,394 transferred bytes cold**; **1285.2 ms / zero transferred bytes warm**. The sunset cold path is roughly 30 MiB. Internet loading and CRONA capacity are unmeasured. Resource requests/limits in C1 are documented initial budgets.

Release prerequisites, outside this goal:

1. Review and merge the application stack into `master` with the squash-stack procedure, then rebuild/validate the actual merged source.
2. Separately authorize and run the manual default-branch publication workflow. Record workflow URL, full source tag and real GHCR manifest digest; verify anonymous pull. Update C1's conspicuous `REQUIRES_PUBLISHED_IMAGE` placeholder with that digest and rerun its checks. A private image needs the established namespace-specific SOPS pull-secret procedure.
3. Review/merge C1 and verify internal rollout, image ID/digest, health, HTML, WASM, shader/font/environment URLs and Host-header routing. The checked-out household configuration requires x86_64, matching the linux/amd64 image; record live node architecture during release.
4. Refresh authoritative DNS and actual zone/Tunnel inventory. The recorded NXDOMAIN for `flock.skippy-ai.com` is point-in-time evidence, not a reservation. Then review I1's guarded main-branch production plan/apply. Public HTTPS/browser/cache/console verification and rollback testing belong to that later session.

[Cluster runbook](https://github.com/martines3000/household-cluster/blob/codex/flock-workload/apps/flock/README.md) and [routing/recovery runbook](https://github.com/martines3000/household-infra/blob/codex/flock-public-route/docs/flock-release.md) preserve the internal-first order, terminal 404 route and host/firewall protections. No public origin port, administrator Access login, database or persistent volume is needed.

Large raw logs/PNG sequences remain under `/home/martines/Projects/ac3d-evidence/2026-11-05/`. Selected evidence is pinned at `ece1b2adbae556fbc5cfdb2bee45728bc9d7b983`. The approved plan is [closed at the review handoff](../completed/2026-11-05-polished-flock-web-showcase.md); production remains outside its completed objective.
