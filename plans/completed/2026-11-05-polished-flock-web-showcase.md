# Build a polished nature showcase and prepare its web release

This ExecPlan is a living document maintained according to `PLANS.md`.

Status: review-preparation goal complete on 2026-11-06. All independently preparable work is finished in unmerged review PRs. C1/I1 remain drafts pending real publication and internal health. No production rollout or public verification was performed.

## Purpose

Turn this project into an inviting, visually cohesive experience that someone can enjoy without understanding boids or simulation settings. The first scene should earn a “this looks cool” reaction through composition, lighting, recognizable animals, and readable motion. Deliver this through small, reviewable improvements, then make the experience available as a website on CRONA.

### Goal for the fresh session

Prepare a tested, reviewable series of small PRs that transforms autonomous-characters-3d into a polished nature showcase, simplifies its visitor UI, improves animal and predator/prey readability, proves browser delivery, and prepares Docker packaging and household-pattern CRONA deployment changes. Follow milestones A0–A7, C1, and I1 below, using subagents for useful independent work. Finish with a morning review index containing PR links, evidence, validation, dependencies, and exact merge order.

This goal ends at the review-preparation handoff. Do not merge, deploy, apply infrastructure, change live DNS, or trigger production workflows. Deployment PRs may remain explicitly marked drafts when a real published image or post-merge internal-health check is unavailable. Complete all independent preparation and identify the exact remaining prerequisite; do not claim readiness using an invented image digest.

Make reasonable decisions, record assumptions and tradeoffs, and continue other work when one lane is blocked. Do not stop for routine design or implementation questions, or after only the first milestone. Ask only when a missing decision or access genuinely prevents further useful authorized work. Update this plan and `plans/STATUS.md` during execution.

### Prompt for a future development chat

Use the following prompt together with this ExecPlan:

> Develop autonomous-characters-3d into a polished, atmospheric nature showcase. Prioritize what a first-time visitor sees, understands, and enjoys. Use this ExecPlan as the staged development goal, and read AGENTS.md, PLANS.md, and plans/STATUS.md before working.
>
> Start by running and recording the current experience. Establish an art direction and a browser feasibility proof before committing to expensive rendering changes. Favor stylized natural scenes, strong animal silhouettes, clear lighting, restrained effects, and deliberate camera composition. Improve one visible aspect at a time. Reuse existing models, animation, environments, cameras, rendering, and capture tools where they fit.
>
> Replace the visitor-facing configuration menu with a small game-like interface: curated scenes, clear flock-size choices, pause/resume, understandable camera controls, reset view, and brief help. Put detailed sliders, diagnostics, backend selection, imports, and experimental features behind one deliberate Lab/developer entry. Keep GPU as the preferred visitor backend. Keep Naive and Octree internally for reference tests and development until browser evidence establishes whether another supported path is needed.
>
> Make each public scene a coherent combination of environment, animals, behavior, population, and camera. Audit every existing preset. Retune, combine, move to Lab, or remove choices that weaken the experience. Favor a few excellent scenes over a large unfinished menu. Low/Medium/High flock-size choices must use scene-specific, measured counts. Keep population and rendering quality distinct.
>
> Re-evaluate natural movement in video, especially predator/prey encounters. A visitor must recognize who hunts whom, see local escape acceleration and dodging, and see recovery and regrouping. Existing pursuit, lunge, panic, and speed-boost mechanics already exist; first determine whether tuning and presentation make them legible. Avoid unexplained invisible threats in public animal scenes. Use Blender when a demonstrated silhouette, animation, or material problem needs an asset change.
>
> Prove browser delivery with the real simulation, instanced renderer, animated models, environments, and input. Prefer a measured Bevy/WASM/WebGPU path that preserves native development. Provide clear loading, failure, and unsupported-device states. Do not promise browser coverage or performance without running it. Package the result reproducibly as static web assets in a Docker image.
>
> Prepare linked application, household-cluster, and household-infra PRs following the household deployment pattern. Propose flock.skippy-ai.com, subject to checking authoritative DNS and existing routes. CRONA should serve the files; the visitor’s device runs the simulation. Keep production release under the owner’s merge/apply control.
>
> Work autonomously across the staged initiative. Parallelize independent research, asset work, and manifest preparation when useful, while respecting shared build and app limits. Make reasonable decisions and record them; continue unblocked work instead of stopping for routine questions. Prepare small PRs with before/after images, motion clips where behavior changes, verification results, performance evidence, tradeoffs, and exact dependency/merge order. Do not merge, deploy, change live DNS, or trigger production workflows. Finish at the review-preparation handoff with a concise morning review index and identify any draft blocked on image publication or deployment access.

## Progress

- [x] (2026-11-05 22:32Z) Created the active review-preparation goal and preserved the approved plan on `codex/showcase-baseline`. The primary agent owns all Cargo verification and app sessions; workers must request the build lock.

- [x] (2026-11-05 21:47Z) Inspected repository guidance, current source, completed plans, and remote household deployment conventions.
- [x] (2026-11-05 21:50Z) Prepared the development prompt, staged PR sequence, proposed defaults, acceptance criteria, and release boundaries.
- [x] (2026-11-05 22:40Z) A0: Captured startup and all fourteen original presets, recorded three motion clips including 24 seconds of Shark, repeated native rendered timings, and documented dispositions/art target in `plans/evidence/2026-11-05-showcase-baseline.md`.
- [x] A1 (completed; early proof and final matrix, 2026-11-05 23:00Z): Warning-free WASM compilation and actual headed AMD WebGPU rendering pass with animated fish, three hunter draws, caustic sand, sky/IBL/shadows and egui. Linux Chromium requires recorded feature flags; flagless has a clear unsupported state. Final committed-source input/capability matrix passes with A6 on runtime 9bbe138.
- [x] (2026-11-05 23:00Z) A2: Complete three-scene contract, scene-specific tiers, stale-model cancellation and explicit public restarts. Integrated A2/A3: 214 release tests pass, warning-free build and focused paused GPU restart regression pass.
- [x] (2026-11-05 23:00Z) A3: Compact visitor HUD, Help/H/P, Scenic/Ride/Free, reduced camera motion and Lab gates implemented; native captures inspected. Existing experimental controls and CPU references remain in Lab.
- [x] (2026-11-06 00:51Z) A4: Animal-volume/warm-sunset composition passes matched capture review and repeated native/browser budgets; no extra effect or asset replacement needed.
- [x] (2026-11-06 00:51Z) A5: Fourteen-choice audit complete; public sharks use 8× scale/1× cruise while legacy Lab stays 5×/1.3×. Matched 25-second clips and original-size frame review show clear silhouettes, local splits and regrouping. Continuous acceleration is not inferred from sparse frames; CPU/GPU mechanics regressions pass.
- [x] (2026-11-06 00:51Z) A6: Final real browser matrix passes controls, chords, keyboard focus, all scenes/tiers, resize/DPI, reduced motion, asset failures, actual visibility and actual GPU crash/reload. All nine tiers measured twice.
- [x] (2026-11-06 00:51Z) A7: Two clean builds produce 32 identical files; local UID 101/read-only static image and HTTP/browser checks pass. Pinned manual master-only publication is prepared, not dispatched. Real registry publication remains C1’s explicit prerequisite.
- [x] (2026-11-05 23:00Z) C1/I1: Prepared and pushed linked draft PRs: household-cluster#1 and household-infra#1. Offline render/policy checks pass; authoritative DNS returned NXDOMAIN for the proposed hostname. Real published image and internal workload health remain explicit post-merge prerequisites, not fabricated readiness.
- [x] (2026-11-06 00:51Z) Completed integrated fmt, 219 release tests, 20 GPU regressions, warning-free build, native smoke, baseline-only Clippy, repeat timings and [morning review index](../evidence/2026-11-06-morning-review.md).

Post-merge public verification and recording the deployed digest are deferred to a separately authorized release follow-up. They are not completion requirements for this review-preparation goal.

## Surprises & Discoveries

- Final QA: Actual HUD dragging exposed pointer fallthrough during held buttons. Current egui area coverage plus gesture-origin ownership fixes it; independent review found and fixed a mixed-button relatch. Red/green regressions and real browser input checks pass.
- Final native gate: The new environment-codec fixture used default RGBA8 images rather than valid half-float cubemaps. The fixture now includes real cube faces/mips and a valid negative-test control; the repeated full gate passes. Runtime payload is unchanged.
- Final delivery: Roughly 30 MiB cold sunset transfer is measured over localhost, not an internet promise. Raw CDP video has padding; full-resolution screenshots and native motion clips are the composition/behavior evidence.

- A1: Headless Chromium reports a ready WebGPU app while CDP captures black; headed capture proves the real rendered scene. The Arch `/usr/bin/chromium` wrapper overrides feature flags, so tested Linux runs use `/usr/lib/chromium/chromium` with recorded flags. Other browser/device combinations remain untested.
- A2: Continuity after Neon 200k → Reef Low retains positions in a domain about 7.4 times larger. Public scene/tier selection now explicitly restarts resident GPU animals. A prepared revision travels with matching GPU uniforms so late HUD changes cannot restart into the previous domain. The paused restart regression verifies fresh bounded positions, cleared panic/hunters, matching motion history, and reused buffers.

- A0: Native baseline passes 204 tests (22 ignored), fmt and warning-free release build. World-fit framing makes species and hunters tiny; the legacy Close-up clip also fails visual encounter readability. Existing low-poly assets do not yet warrant Blender changes. Two measured 30k Shark runs differ materially (3.684/2.508 ms means), so report both rather than claiming a fixed cost.

- This is already a substantial visual application. It has animated fish and birds, shark/falcon/tuna hunters, HDR environments, underwater caustics, shadows, volumetrics, motion blur, depth of field, and cinematic cameras. Adding effects is not the initial problem.
- Showcase applies a full scene, while Species preserves environment, camera, backend, and some surrounding state. Similar-looking controls therefore have different consequences (`src/showcase.rs` and `src/species.rs`).
- Several older fish and murmuration presets use invisible orbiting flee fields. The explicit hunt examples use visible animals, but preserve the previous environment and camera.
- Prey already receive a 35% speed allowance for 0.65 simulated seconds. Hunters already lunge and recover. A second panic system would duplicate existing behavior (`src/hunter.rs`, `src/vehicle.rs`, and `assets/shaders/gpu_flock.wgsl`).
- Current model loading uses `File::open`, local directory scanning, and `CARGO_MANIFEST_DIR`. Browser support requires real asset-loading work, including hunter meshes (`src/vehicle_model.rs`).
- `docs/visual-improvements-research.md` describes an earlier transparent-only pipeline. Current `src/instancing.rs` includes prepass/shadow support; inspect current source before reusing the old limitations.
- CRONA uses K3s/Flux with an internal Traefik ingress. Public HTTPS exposure follows the existing Cloudflare Tunnel and DNS configuration owned by household-infra.

## Decision Log

- Decision: Public scene and population selection restart the flock; Reset view changes only the camera. Lab count/mode edits retain existing continuity.
  Rationale: Scene coherence and an immediate subject outweigh preserving positions from an incompatible world. This is an explicit public behavior change, not a side effect of UI work.
  Date: 2026-11-05.
- Decision: Use animal-volume framing and face the sunset light instead of adding another rendering effect or replacing assets.
  Rationale: Matched native captures show recognizable layered birds and underwater roles with the existing low-poly assets and effects.
  Date: 2026-11-05.

- Decision: Start with three public scenes, preserve other presets as labeled Lab experiments, and fix composition before changing steering.
  Rationale: All fourteen captures show a presentation/consistency gap; existing close-up animal detail and hunt mechanics are usable.
  Date: 2026-11-05.

- Decision: Plan first; do not start implementation during this preparation pass.
  Rationale: The owner explicitly selected preparation of the goal and PR plan.
  Date: 2026-11-05.
- Decision: Favor atmospheric, stylized nature and a small curated scene collection.
  Rationale: Consistent composition and legible animals directly address the requested quality and first impression.
  Date: 2026-11-05.
- Decision: Hide CPU backends and detailed controls in Lab before considering removal.
  Rationale: They retain regression/parity value. Removing them does not improve the visitor experience more than hiding them.
  Date: 2026-11-05.
- Decision: Prove browser compatibility early, alongside visual discovery.
  Rationale: Unsupported shaders, limits, assets, or task execution could otherwise invalidate later polish.
  Date: 2026-11-05.
- Decision: Keep one shared simulation and scene definition across native and browser builds where practical.
  Rationale: Duplicated implementations would increase tuning drift and review cost.
  Date: 2026-11-05.
- Decision: Separate review readiness from production release.
  Rationale: A cluster PR needs a real published image. Public exposure must follow a healthy internal workload.
  Date: 2026-11-05.
- Decision: Preserve the existing decision against general CI. Limit new automation to release tooling needed for this task.
  Rationale: A manually dispatched image-publication workflow can follow household precedent without adding an unrelated CI project.
  Date: 2026-11-05.
- Decision: Execute the approved plan in a fresh session and end the goal at the review-preparation handoff.
  Rationale: The owner's latest instruction requests a fresh session, autonomous work with useful subagents, small PRs, and no merge or deployment.
  Date: 2026-11-05.

## Assumptions and Open Questions

- **Art direction:** Matched captures selected stylized natural scenes with closer animal-volume framing and existing assets. No observed asset defect warranted Blender work. Owner and independent human taste feedback remain part of morning review.
- **Public scene count:** Sunset murmuration opens the app; Reef drift and Shark hunt complete the three public scenes. The fourteen-preset audit records the remaining choices as Lab experiments or replacements.
- **Browser support:** A1/A6 prove the real shared Rust/WASM/WebGPU application on the recorded Linux/Wayland Chromium 155 configuration with RX 7900 XT/RADV and explicit Vulkan flags. Other browsers, GPUs and mobile devices remain untested; see [browser feasibility](../../docs/browser-feasibility.md).
- **Performance:** All nine scene/tier combinations were measured twice at 1920×1080 near the 60 Hz browser cap. Native and cold/warm localhost measurements are recorded in the [morning index](../evidence/2026-11-06-morning-review.md). These measurements do not establish GPU headroom, weaker-device performance or Internet loading times.
- **Population:** Low/Medium/High use 8,000/20,000/40,000 starlings, 500/1,500/3,000 reef fish, and 5,000/12,000/20,000 prey plus three hunters in Shark hunt. There is no public Spectacle tier or numeric slider. Million-vehicle hunter scaling remains separate work.
- **Hostname:** `flock.skippy-ai.com` remains proposed. Read-only repository and point-in-time DNS inspection found no conflict; the later release must refresh actual zone and Tunnel state. The historical `ac3d.skippy-ai.com` hostname is unchanged; no redirect was prepared.
- **Image publication:** The master-only manual GHCR workflow is prepared and untriggered. C1 remains a draft until the application merges, the owner authorizes publication, and a real immutable registry digest and public pull proof exist. The local review image ID is not a registry publication digest.
- **Production access:** This completed goal authorizes review preparation only. Deployment PRs, validation and runbooks are prepared. Publication, internal health checks, guarded infrastructure plan/apply and public verification require the later release session; no production action occurred.

## Outcomes & Retrospective

Execution complete at review handoff. The application stack delivers three coherent GPU scenes, a compact visitor HUD/Lab, explicit safe restarts, composed first views, readable public shark tuning, shared native/WebGPU runtime and reproducible static release tooling. Native 219 tests and 20 GPU regressions pass; both Clippy targets retain seven baseline warnings. All nine browser tiers pass repeated 1080p measurements near 60 Hz. Two builds reproduce 32 identical files. Cluster/routing PRs are validated drafts. See the morning index for PR links, merge order, immutable evidence, device/payload/capture limitations and exact release prerequisites. No production action or human taste approval is claimed.

Earlier A1–A3 interim outcome: real headed browser rendering was feasible and native visitor state/HUD and public restarts were tested. Browser flow and art/performance acceptance were still open at that milestone; they are complete in the final outcome above. C1/I1 retain their explicit release prerequisites.

A0 delivered: complete fourteen-preset disposition audit, native startup/scene captures, fixed-step motion clips, repeat timings, and a three-scene art target. Native baseline checks passed. Browser implementation was pending at that milestone; final browser proof is recorded above.

The initial planning pass delivered a reusable development prompt, repository-backed gap analysis, staged PR plan, observable acceptance criteria, and household release ordering. Runtime and browser measurements were not performed during that initial pass; execution results are recorded above.

The execution goal is complete when the review package and all independently preparable work are finished, with remaining release prerequisites explicitly identified. Final production acceptance belongs to a later release follow-up after the owner’s merges and approved routing apply.

## Context and Orientation

This starting-point inventory informed execution. Final implemented boundaries are recorded under Interfaces and Dependencies.

- `src/main.rs` owns application setup, simulation defaults, the current large egui menu, backend selection, and camera input.
- `src/showcase.rs` owns complete scene bundles; `src/species.rs` owns species tuning and explicit hunt examples. These are the starting points for a single visitor scene-selection contract.
- `src/presentation.rs`, `src/director.rs`, `src/follow.rs`, and `src/scene.rs` own presentation, cinematic shots, riding cameras, and the scene.
- `src/environment.rs`, `src/underwater.rs`, `src/volumetric.rs`, `src/instancing.rs`, and `assets/shaders/` own the existing rendering paths.
- `src/vehicle.rs` and `src/gpu_flock.rs` own CPU/GPU steering. `src/hunter.rs`, `src/predator.rs`, and `src/swarm.rs` distinguish visible hunters from flee fields and groups.
- `src/vehicle_model.rs` and `src/vehicle_animation.rs` own the bounded importer and animation. `scripts/make_*_blender.py` and `assets/models/` provide reproducible animal assets.
- `src/scenario.rs` and `src/capture.rs` provide native scripted inspection and fixed-step frame captures. Their environment/filesystem interfaces need separation from the browser runtime.
- `Cargo.toml` pins Bevy 0.20.1 and bevy_egui 0.43.1; `rust-toolchain.toml` pins Rust 1.99.1. Preserve them unless the browser/dependency milestone requires a documented change.
- `plans/STATUS.md` records known high-count density and hunter costs. Those remain separate investigations unless a selected public tier exposes them.

Deployment sources, inspected on remote default branches on 2026-11-05:

- `household-cluster`: `main` at `812481bebf8dea5c7403c8232960dbb30cb270e1`.
- `household-infra`: `main` at `1d881268a0d2ef082a1631c252598e4870c784b6`. Refresh isolated checkouts before implementation; a local checkout observed during research differs from this remote layout.
- [household-cluster adding-an-app guide](https://github.com/martines3000/household-cluster/blob/main/docs/adding-an-app.md) and [conventions](https://github.com/martines3000/household-cluster/blob/main/docs/conventions.md).
- [Flux apps registration](https://github.com/martines3000/household-cluster/blob/main/clusters/crona/apps.yaml) and [cluster contribution checks](https://github.com/martines3000/household-cluster/blob/main/CONTRIBUTING.md).
- [household-infra architecture](https://github.com/martines3000/household-infra/blob/main/docs/architecture.md), [Cloudflare configuration](https://github.com/martines3000/household-infra/blob/main/cloudflare.tf), and [infra contribution checks](https://github.com/martines3000/household-infra/blob/main/CONTRIBUTING.md).
- [Household image-publication precedent](https://github.com/martines3000/life-os/blob/master/.github/workflows/publish-backend-images.yml) and [production OpenTofu workflow](https://github.com/martines3000/household-infra/blob/main/.github/workflows/opentofu.yml).
- [Bevy browser examples](https://bevy.org/examples-webgpu/) establish that Bevy has a WASM/WebGPU route. [MDN WebGPU](https://developer.mozilla.org/en-US/docs/Web/API/WebGPU_API) documents capability and secure-context requirements. Recheck current support during A1.

## Plan of Work

Each row is a reviewable outcome. Split a row into smaller PRs when independent visual or behavior changes would obscure review. Skip an asset/effect change when the existing implementation already meets acceptance.

| ID | Proposed PR/outcome | Depends on | Required evidence |
| --- | --- | --- | --- |
| A0 | Record the baseline and define the visitor experience and visual target | None | Startup captures, preset audit, short motion clips, initial frame/load budgets |
| A1 | Prove a thin browser slice and isolate native-only interfaces | None; coordinate with A0 | Real animated prey/hunters, compute/instancing, one environment, browser console and capability report |
| A2 | Compose complete public scenes and a deliberate startup scene | A0 | Order-independent preset/reset tests; fresh-start and cross-scene switching captures |
| A3 | Add the visitor HUD, scene-specific population tiers, and Lab access | A2 | Wide/narrow UI captures; keyboard/pointer checks; hidden diagnostics and backend controls |
| A4 | Improve lighting, background, camera composition, and animal presentation in small passes | A0/A2; browser constraints from A1 | Matched before/after stills, motion clips, measured rendering cost |
| A5 | Audit and retune species; make predator/prey encounters readable | A0/A2 | Complete encounter clips, preset disposition table, focused CPU/GPU behavior evidence |
| A6 | Complete browser loading, assets, input, capability handling, and measured quality tiers | A1–A5 | Tested-browser matrix, cold/warm loads, sustained frame times, failure/recovery checks |
| A7 | Add reproducible web builds, static Docker packaging, and manual image publication | A1 for initial tooling; A6 for final release | Clean build/preview commands, local container browser smoke, actual versioned image provenance |
| C1 | Add the internal CRONA workload in household-cluster | A7 and a real published image | Rendered manifests, repository checks, digest, rollout/internal-ingress runbook |
| I1 | Add the public hostname and Tunnel route in household-infra | C1 for final routing; internal health before exposure | Narrow infra diff, policy checks, reviewed production plan and exposure/rollback runbook |

### Experience and behavior details

Audit all five Showcases, six Species, and three hunt examples. Record “keep public”, “retune”, “Lab only”, or “retire”, with a reason and evidence. In particular, label geometric Swallow Sphere/Helix behavior as an artistic experiment if retained. Check that animal names match their models.

The public scene contract must set compatible environment, model/animation, behavior, hunter roles, density tier, and camera framing. Preserve user-owned preferences only when they cannot create contradictory scenes. Define restart/reseed versus continuity explicitly. Test switching from every incompatible scene family, including rapid repeated clicks and delayed asset completion.

Use a compact scene selector and restrained controls. Keep H/hide UI and camera actions discoverable. Separate UI hit areas from world picking and camera gestures. Default developer controls, FPS, wireframes, targets, imports, and numeric steering controls to hidden. Preserve the useful developer workflows behind one deliberate Lab entry.

For natural scenes, review scale, spacing, acceleration, banking, flap/swim cadence, turn radius, coherence, and the visibility of world boundaries. Show distinct behavior between sparse reef movement, schooling fish, and flowing bird clouds. Avoid persistent overlap, wall jitter, rigid rotating balls, or artificial formations presented as natural behavior.

For hunts, use silhouette and scale as well as color. Show pursuit, a committed pass, local scatter/dodge and acceleration, recovery, and regrouping in a 20–30 second clip. Verify that wide framing communicates the encounter and close framing supports the animal detail. Start with existing mechanics; make any simulation changes explicit and maintain parity and frame-rate independence.

### Browser and delivery details

A1 must exercise actual storage-buffer/workgroup limits, shaders, texture formats, render passes, model animation, hunter draws, asynchronous environment preparation, and input. Check random-number/browser dependency configuration and task execution. Adapt browser asset retrieval; do not ship local file paths or assume native task pools imply nonblocking browser work.

The HTML shell owns loading, unsupported-device, and recoverable startup-error messaging. The shared scene/UI code owns visitor interaction. Test resizing, high-DPI scaling, fullscreen if offered, keyboard focus, pointer handling, pause/resume, background-tab return, asset failures, and device loss. Offer reduced camera motion. Record mobile support honestly; full mobile interaction is not a prerequisite for a clearly desktop-targeted first release.

Use one documented web build path selected by A1, with pinned tools and release artifacts. Serve correct WASM MIME types, compressed assets, revalidated HTML, and safe cache behavior for versioned files. Add cross-origin isolation headers only if the chosen runtime needs them. Test the same static image locally that CRONA will run. No database, streaming GPU server, or persistent volume is needed for static hosting.

C1 should add `apps/flock/` and register `flock` in `apps/kustomization.yaml`. Add `platform/namespaces/flock.yaml` and register it in `platform/namespaces/kustomization.yaml`. Use an immutable image, ClusterIP Service, `traefik` Ingress, HTTP probes, resource requests/limits, and the existing non-root/security conventions. Add a pull Secret only if the image is private, using the established SOPS pattern.

I1 should extend the existing hostname configuration, Cloudflare Tunnel routing, and proxied CNAME. Preserve the trailing 404 route, unrelated hostnames, firewall, and host protection. Use the household production workflow for plan/apply. Verify the internal workload before enabling public exposure. The public showcase should not require an administrator’s Cloudflare Access login.

### Parallel work and review handoff

A0 and A1 research can run in parallel. Asset experiments can proceed after the art target is clear. C1/I1 drafts can be prepared while application work finishes. Coordinate changes to shared scene/UI/model files; do not let parallel workers independently invent preset state.

One owner schedules all builds, tests, benchmarks, and app sessions across worktrees. Keep Cargo activity serialized and limited to four jobs. Disposable probes use a separate `CARGO_TARGET_DIR` outside `target/`. Run only one native app at a time; keep measured browser runs isolated from competing app/benchmark work.

Use `codex/` branch names and the repository’s commit-header types. Every future PR should state its observable result, dependencies, exact checks, evidence, and limitations. The morning index must list PR URL, repository, ready/draft status, merge order, short before/after evidence, and any remaining owner action. Keep deployment PRs as drafts until their real prerequisites exist.

## Concrete Steps

The execution recipe below is retained for reproducibility. Completed results and evidence are recorded above and in the morning review index. Production publication and rollout remain future work outside this goal.

1. Inspect `git status --short --branch` and repository guidance before editing. Use isolated worktrees for separate PRs where needed.
2. Before any Cargo command, inspect `pgrep -af 'cargo|rustc'` and resolve active work with its owner. Check for an existing native app. Use the established shared verification lock if one exists.
3. A0: build and launch through the repository’s native commands, then use `AC3D_SCRIPT` scenarios and fixed-step capture commands documented in README. Store large captures outside the repository and attach selected evidence to PRs.
4. A1: select the browser build command from pinned dependency/source research. Record that exact command, target/features, serving command, and expected successful browser result here before promoting the proof into the supported workflow. Do not present an untested guessed command as a working build.
5. At the integrated application review head, run the native gate sequentially from this repository root:

```sh
CARGO_BUILD_JOBS=4 cargo fmt --all -- --check
CARGO_BUILD_JOBS=4 cargo test --release --locked
CARGO_BUILD_JOBS=4 cargo build --release --locked
python3 scripts/smoke-native.py
git diff --check
```

6. Run focused ignored GPU regression tests when steering, buffers, shaders, or GPU rendering contracts change:

```sh
CARGO_BUILD_JOBS=4 cargo test --release --locked gpu_ -- --ignored --nocapture --test-threads=1 --skip gpu_flocking_benchmark
```

7. Keep the release build warning-free. If touched code warrants Clippy, compare against the seven documented existing warnings; Clippy is not a new gate. Benchmark only relevant changes, with `uptime` checked first, no competing work, at least two runs, and the README’s exact benchmark commands.
8. A7 must add exact clean-build, local-container, browser-smoke, and publication commands to README. Use an image built from the recorded source commit, and record its digest after publication.
9. Read each deployment repository’s own `AGENTS.md`/`CONTRIBUTING.md` at execution time. From the household-cluster root, the inspected validation sequence is:

```sh
./scripts/validate.sh
bash -n scripts/*.sh
shellcheck scripts/*.sh
git diff --check
```

10. From the household-infra root, the inspected validation sequence is:

```sh
tofu fmt -check -recursive
bash -n scripts/*.sh tests/*.sh
shellcheck scripts/*.sh tests/*.sh
bash tests/bootstrap-policy.sh
bash tests/backup-policy.sh
bash tests/plan-policy.sh
bash tests/public-ingress-policy.sh
if grep -nE '^[[:space:]]*sed[[:space:]].*--extended-regexp' scripts/*.sh; then
  exit 1
fi
git diff --check
```

Inspect rendered resources as well as exit status. Full production plan/apply uses the infra repository’s main-branch manual workflow. Do not bypass its required apply confirmation or host/firewall protections.

## Validation and Acceptance

- **First impression:** After loading, a recognizable subject and coherent scene are visible immediately. Within ten seconds of watching, the intended activity is apparent. Record owner/new-viewer feedback when available; distinguish human review from scripted proxies.
- **Visitor flow:** Change scene and flock size without knowing boids, steering, or backend terminology. Pause, camera selection, reset view, and help are clear. Default HUD does not cover the main action; developer controls and diagnostics start hidden.
- **Consistency:** Public scenes do not inherit an incompatible environment, model, threat, or camera. Repeated switching, reset, pause, loading failure, and developer-mode entry/exit work predictably.
- **Animal behavior:** Every original preset has an explicit disposition. Public natural scenes have distinct, plausible motion over time. Predator/prey clips communicate roles, local escape acceleration/dodging, recovery, and regrouping without debug rings.
- **Visual iteration:** Each retained visual change has matched captures and a stated reason. Behavior changes include video. New assets have reproducible sources, credits where applicable, and recorded geometry/animation cost.
- **Performance:** Record source commit, build profile, device/GPU, OS/browser/version, resolution, scene, population, effects, warmup, and cold/warm loading. Report frame-time distributions and repeat measurements. Do not extrapolate the RX 7900 XT or historical M3 Pro results to untested browsers.
- **Browser:** At least the declared supported configuration runs the real scene and complete visitor flow. Unsupported configurations show a clear explanation instead of a blank canvas or crash. Record untested browser/device combinations honestly.
- **Engineering:** Native behavior gates and relevant GPU tests pass. CPU reference paths, numerical safety, exact octree behavior, and frame-rate independence retain coverage. Defaults/semantics change only as explicit parts of the relevant PR.
- **Release preparation:** Clean web/container builds reproduce the tested artifact. Ready cluster PRs reference a published immutable image. All PRs show truthful prerequisites and validation; production is not claimed complete at the review-only handoff.
- **Deferred production follow-up:** After separately authorized owner-controlled release, verify HTTPS, HTML/WASM/assets, cache headers, loading, first scene, switches, input/resize, and browser console. Confirm internal workload health and no unintended origin-port exposure. Record URL, source commit, deployed digest, and tested rollback procedure. This is outside the current goal.

## Idempotence and Recovery

Keep each improvement in a small PR with a coherent parent and documented dependency. Preserve existing user work. Scene application must cancel or ignore stale asynchronous results and handle repeated selection safely.

Roll back application behavior or art by reverting the responsible PR and retesting the affected scene. Retain native tooling and CPU reference paths throughout browser adoption.

Roll back a deployed update by restoring the previously verified image digest through GitOps. For a first installation, removal needs a reviewed workload change. Revert only the new public route/DNS configuration if exposure must be withdrawn. The infra plan guard rejects destructive plans, so document its approved recovery procedure instead of assuming DNS deletion can pass ordinary apply.

## Artifacts and Notes

Initial planning checkpoint: `master` at `3088a34`, initially clean. The source/repository audit was read-only; no app session, Cargo build/test, benchmark, Blender change, image publication, or deployment had occurred at that checkpoint.

The completed [morning review index](../evidence/2026-11-06-morning-review.md) links baseline and improved captures, the preset disposition table, tested-browser matrix, performance/load report, source/image provenance, PR order and release prerequisites. Large raw logs and recordings remain outside the working repository; selected immutable evidence is linked for review. No image publication or deployment occurred during execution.

## Interfaces and Dependencies

Implemented ownership changes are `visitor::apply`/`VisitorScene`/`PopulationTier`, `FlockRestart` requested/prepared revisions, shared pointer gesture ownership, `BrowserPlugin`/`BrowserAssetsPlugin`, bounded bundled model import, native lighting export and versioned static release packaging.

The original expected ownership changes were a coherent public scene-application boundary, visitor/developer UI visibility, scene-specific population tiers, and browser-compatible asset/runtime interfaces. The implemented boundaries above reuse the existing simulation and rendering paths without adding a generic configuration framework.

Dependency/toolchain edits belong only to A1/A6/A7 when browser support or build tooling requires them. Do not opportunistically upgrade Bevy or replace the simulation/rendering engine. Keep native-only capture/import tooling available where useful.

This approved initiative restored the optional shared WebGPU target requested by the owner. AGENTS.md now documents both native development and browser delivery. The historical Trunk setup remains retired; the tested static image workflow is documented in [web delivery](../../docs/web-delivery.md).

---

Revision notes:

- 2026-11-05: Initial planning-only goal and PR sequence, grounded in current application code and household deployment guides.
- 2026-11-05: Recorded approval for a fresh-session execution goal, autonomous work and useful subagents, small review PRs, and an explicit stop before merges or production changes. Moved post-release verification outside this goal so review preparation has a reachable completion condition.

- 2026-11-05: Split execution progress into A2/A3/A4/A5, recorded the observed continuity/domain problem and explicit public restart decision, and linked completed deployment draft preparation.

- 2026-11-06: Closed the approved review-only goal after full native/browser/container validation and linked PR preparation. Publication, production plan/apply, rollout, broad-device validation and public verification remain explicitly outside this handoff.
- 2026-11-06: Reconciled completed assumptions, historical planning checkpoints, README coverage counts and setup guidance during the final documentation audit. Runtime code and recorded test results are unchanged.
