# Repository review progress

The original WASM web build was removed on 2026-10-31 (commit 7f8ef75); web-related findings and logs were removed from this record. See the git history for the original text.

Review date: 2026-10-30 (Europe/Ljubljana).
Repository: /Users/martines/Desktop/git_repos/personal/autonomous-characters-3d
Scope: analysis-only review of current implementation, validation, dependencies and modernization; create self-contained review-report.html. Do not change existing application files or Cargo.lock.

## Status
- In progress: initial repository inventory and guidance.
- Pending: architecture/source inspection, validation, current dependency verification, evidence-based findings, HTML report and final verification.

## Commands and evidence
- Read user request from attachment Pasted text.txt.
- Initial inventory found README.md, Cargo.toml and Cargo.lock; Rust/Bevy simulation.
- Git/environment details being collected; see next update.

## Next steps
1. Record branch, commit, initial working tree and environment.
2. Read manifests, documentation and all relevant simulation/octree code.
3. Run existing non-destructive checks preserving locked dependencies.
4. Verify latest stable direct dependencies using official registries/releases.
5. Write and inspect report; update this log with final findings and limitations.

## Phase 1 completed: inventory and static source review
- Branch: feat/post-ai-version; commit e7a53d70c75020e399b4a2d5c1b1153ffbb9075f (2023-03-18). Initial working tree clean. No applicable AGENTS.md found in repository/ancestor paths checked.
- Read all four Rust modules, manifests, toolchain/Cargo config, README and Slovene analysis; viewed meritve.png. No tests/CI found among tracked files.
- Environment: macOS 26.7.1 ARM64, 18 GiB RAM, 11 logical CPUs; existing stable Rust 1.95.0. Repository toolchain selects Rust 1.67.1. First rustup probe automatically downloaded pinned toolchain; initial concurrent cargo check failed with Missing manifest while installation was in progress. Rustup probe later succeeded (1.67.1). This is an environment bootstrap artifact, not a code failure.
- Native formatting with installed stable: `RUSTUP_TOOLCHAIN=stable cargo fmt --all -- --check` PASS.
- Current-stable locked cargo check running; exact logs under /tmp/ac3d-review-e7a53d7/. Preserve lock SHA256 297057ff03baa34b0413676a4d0d95c5be2ed3d5dd7c880f5c4d8cdc6d069b0a.

## Findings under verification
- High: octree.find_neighbors (src/octree.rs:335-374) only visits one leaf; does not accept query radius or adjacent cells. Simulation radius filters cannot recover missing neighbors (vehicle.rs:418-455).
- High: velocity update omits dt on acceleration (vehicle.rs:285), wander random walk advances every frame (180-183), target moves 2 units/frame (target.rs:96).
- High: zero normalization / divide by zero in seeking, separation, alignment and rotation; cohesion uses negative f32::MIN as tolerance (vehicle.rs:479-481,571-573).
- Medium: system registration lacks explicit movement -> update ordering (vehicle.rs:49-53).
- Medium: speed clamped per component instead of magnitude (vehicle.rs:281-285).
- Medium: mass UI updates resource but existing VehicleMass only set at spawn (vehicle.rs:100; main.rs:266).
- Medium: benchmark system disabled (vehicle.rs:54); checkbox only hides subsequently spawned agents (85); dormant benchmark fails to reset count when switching naive to octree (328-345).
- Performance: naive mode is default, quadratic at UI's 50,000 maximum; octree rebuild serial, per-agent neighbor vectors, degenerate tree has no depth/extent bound.
- Architecture: duplicate steering math between backends, unused direct dependencies bevy-inspector-egui/rayon/rustc-hash, no tests/CI/setup instructions.

## Historical benchmark evidence
meritve.png: 1k naive 4.227 ms vs octree 3.259 ms; 10k 242.988 vs 9.106 ms; 30k 1454.913 vs 23.343 ms; 100k octree 79.969 ms. Slovene analysis lines 38-40 explicitly says boids hidden due to rendering bottleneck. No machine/compiler/seed/raw dataset metadata. Octree does less neighbor work, so this is not an equal-semantics speed comparison.

## Next steps
Finish pinned/stable checks; run isolated harness against unmodified octree and nalgebra for neighbor and zero-vector cases; verify official registry current versions/security; construct report with reproducible commands and explicit limitations.

## Phase 2: validation and reproductions
- Current Rust 1.95 `cargo check --locked`: FAIL in dependency coreaudio-sys 0.2.11/bindgen/proc-macro2 1.0.51, invalid generated Ident for unnamed enum from macOS 26.5 SDK. Application not typechecked to completion. Log cargo-check-stable.log.
- Pinned Cargo 1.67.1 check: online git-index fetch stopped after ~1 minute to avoid lengthy legacy index bootstrap. Offline check cannot find bevy in legacy git registry cache; sparse registry probe confirms it requires unstable sparse support. This is a registry/cache limitation; pinned compiler viability remains unverified.
- Native cargo test and Clippy on Rust 1.95 attempted; compiling prerequisites.
- Isolated release harness in /tmp/ac3d-review-e7a53d7/harness imports the repository's unchanged octree.rs and exact nalgebra 0.31.4 + generational-arena 0.2.8 + bevy_ecs 0.9.1. Uses copy of project lock, pruned only inside temporary harness. Initial harness compile had an ambiguous float type; corrected in temporary harness only.
- Harness PASS: neighboring points (-0.01,1,1) / (+0.01,1,1), capacity 1, distance 0.02: querying first point returns only itself. Zero.try_normalize(f32::MIN) -> [NaN,NaN,NaN]; opposing velocities normalize to NaN. Component clamp max80 allows magnitude138.564.
- Cluster reproduction (capacity2, all points1,1,1): n100 ->785 nodes /220us; n1000 ->7985 /15583us; n5000 ->39985 /960588us. Single isolated timings, not simulation benchmark. Confirms linear-depth growth and worsening insertion cost.
- ECS minimal two-system probe happened to run movement then update; no evidence of an observed ordering failure. Missing explicit order is a maintainability/reliability risk, not a demonstrated race. Bevy serializes conflicting systems.

## Phase 3: current versions/security (verified 2026-10-30)
- Primary crates.io API summaries saved in /tmp/ac3d-review-e7a53d7/dependency-summary.json with full per-crate JSON.
- Latest non-prerelease/non-yanked: Bevy0.20.1 (MSRV1.97.1), inspector0.38.0, rand0.10.3, bevy_egui0.43.1, nalgebra0.35.0, smooth-bevy-cameras0.14.0, rayon1.12.0, picking0.20.1, rustc-hash2.1.3, generational-arena0.2.9.
- Official Rust stable channel manifest:1.99.1.
- Confirmed picking archived 2025-03-04 after upstreaming into Bevy0.15. Generational-arena unmaintained RustSec RUSTSEC-2024-0014; alternative slotmap or plain Vec indices since tree rebuilt/no deletions.
- Selected RustSec checked: rand0.8.5 covered by RUSTSEC-2026-0097 (custom logger reentrancy conditions not observed here); webbrowser0.8.7 by RUSTSEC-2026-0257 (Unix BROWSER attacker-controlled URL path not demonstrated); idna0.3.0 by RUSTSEC-2024-0421 (privilege-check scenario absent). Crossbeam-channel0.5.6 is UNAFFECTED by RUSTSEC-2025-0024 (<=0.5.11 unaffected), avoid false positive.
- OSV batch of entire lock in progress; not a reachability proof.

## Exact next steps
1. Collect pending test/Clippy/OSV results and plugin compatibility.
2. Generate review-report.html with 10 required sections, ranked findings, source/registry links, benchmark caveat, commands and embedded evidence.
3. Verify HTML visually and all internal links; check git diff and Cargo.lock SHA unchanged; finalize log.

## Phase 3 completed: dependency/security verification
- OSV querybatch covered all 398 registry lock entries. 16 retained native-related package matches, deduplicating advisory aliases: 7 unmaintained, 2 unsoundness, 7 other security advisories. Full summary and individual records in /tmp/ac3d-review-e7a53d7/. Report embeds full package triage and primary links. Reachability not established.
- Latest published smooth-bevy-cameras0.14.0 targets Bevy0.16; upstream main is unpublished0.15 for Bevy0.17. Latest egui0.43.1 / inspector0.38 target Bevy0.20. A latest-everything upgrade is incompatible without camera/picking work.
- Native test and Clippy finished with dependency build failure (same CoreAudio identifier panic), not a test assertion/lint finding.

## Phase 4: report generated; final verification in progress
- Created self-contained review-report.html (about 300 KB, including embedded historical benchmark image and essential logs/harness source).
- Report has all 10 required sections, 14 findings (5 High, 8 Medium, 1 Low), confidence/effort/evidence/impact/fixes and snapshot source links, exact setup/check commands, date/environment/commit, dependency versions with sources, advisory triage, roadmap and top five actions.
- Structural HTML checks PASS: 25 unique IDs, all internal navigation targets resolve, 122 links, no external scripts/stylesheets, no unfinished placeholders.
- Report generator is temporary /tmp/ac3d-review-e7a53d7/build_report.py; initial append script had nested-quote syntax error, corrected in temporary scripts only. Final report regeneration passes.
- Chrome CUA preview creation timed out after 30 sec; retry visual QA through available local Chrome headless if needed.

## Pending final steps
1. Render report and inspect layout (or explicitly document inability).
2. Validate report source line anchors, git diff, lock SHA and complete progress note.
3. Deliver report/progress paths and concise summary; no application edits.

## Completed review — 2026-10-30

All requested review phases and both deliverables are complete. No implementation fixes, dependency upgrades or edits to tracked project files were made.

### Final findings index
- F01 High / Confirmed: zero-vector normalization/division creates NaNs (vehicle.rs:144,437–481,529–573,291–296).
- F02 High / Confirmed: octree single-leaf query omits adjacent neighbors (octree.rs:335–374; vehicle.rs:418–455).
- F03 High / Confirmed: acceleration/wander/input are frame dependent (vehicle.rs:180–183,281–296; target.rs:90–96).
- F04 High / Confirmed: coincident points cause deep octree/expensive rebuilds (octree.rs:69–105,125–128,282–312).
- F05 Medium / Confirmed: cohesion distance measured after normalization (vehicle.rs:475–491,567–583).
- F06 Medium / Confirmed: axis clamp permits √3 times configured speed (vehicle.rs:281–285).
- F07 Medium / Likely: missing explicit producer/integrator order; no actual inversion observed (vehicle.rs:47–54).
- F08 Medium / Confirmed: mass slider applies only to newly spawned agents (main.rs:265–266; vehicle.rs:100,58–61).
- F09 Medium / Confirmed: benchmark disabled, visibility not propagated, dormant sweep switch count incorrect (vehicle.rs:54,84–86,312–346).
- F10 High / Confirmed: default naive mode scales quadratically up to 50k slider (main.rs:226,263–264; vehicle.rs:500–547).
- F11 Medium / Confirmed on tested modern setup: historical native dependency blockers; pinned historical build unverified (see logs).
- F12 Medium / Confirmed: no automated behavior/spatial tests or CI/setup baseline.
- F13 Low / Confirmed: 3 unused direct dependencies, duplicate GUI graph (Cargo.toml:13–20).
- F14 Medium / Confirmed package-level matches: 16 retained native-related maintenance/security advisory packages; application exploitability unverified.

### Final verification
- Self-contained report includes all required sections, 14 findings, current version/compatibility tables, 398-package advisory screening, historical benchmark image, roadmap and top five actions; essential temporary evidence/reproduction source embedded.
- HTML structural checks passed: unique anchors, internal links, no external scripts/stylesheets or unfinished placeholders. All pinned GitHub source reference line ranges validated against current files.
- Visual QA: headless local Chrome screenshot inspected for desktop executive summary, desktop dependency table, and mobile finding. CUA timed out; CLI screenshots initially raced compositor scrolling, so local DevTools capture used for reliable final inspection. Mobile viewport width420/document width420: no horizontal page overflow. Temporary QA scripts/copies/screenshots are only in /tmp/ac3d-review-e7a53d7/.
- `git diff --exit-code` PASS: no tracked files changed. Final status contains only untracked review-progress.md and review-report.html.
- Cargo.lock SHA-256 unchanged: 297057ff03baa34b0413676a4d0d95c5be2ed3d5dd7c880f5c4d8cdc6d069b0a.

### Deliverables
- /Users/martines/Desktop/git_repos/personal/autonomous-characters-3d/review-report.html
- /Users/martines/Desktop/git_repos/personal/autonomous-characters-3d/review-progress.md

### Remaining limits / future work (not required to complete this review)
No full native app smoke test, historical SDK reconstruction, GPU profiling, equal-semantics simulation benchmark or exhaustive advisory reachability analysis was completed. The report explains these limits. For implementation follow-up, use the final top-five roadmap. If resuming after interruption, first verify commit and lock SHA; the review itself needs no remaining work.

## Extension requested — tooling and progress measurement (2026-10-30)
- User asked to extend the report with researched tools/crates that improve the software or measure progress, including EmbarkStudios.
- Scope remains analysis/report only. No tools installed, crates added or application files changed.
- In progress: primary-source Embark research in background agent per Research skill; main agent researching Bevy-native tracing, benchmark frameworks, CPU/memory/GPU profilers, property testing and performance tracking.
- Planned deliverable: new report section with prioritized recommendations, current versions/MSRV/compatibility caveats, practical adoption plan and explicit measurable acceptance criteria. Persist supporting sources in review-tools-research.md.
- Existing commit unchanged e7a53d70c75020e399b4a2d5c1b1153ffbb9075f. Starting tree has only existing untracked review files.

### Extension research completed
- Primary sources verified; supporting citations in review-tools-research.md, raw registry/repo metadata in /tmp/ac3d-review-e7a53d7/tools/.
- Recommended initial stack: Bevy0.9 trace_chrome/Perfetto, one Criterion headless benchmark harness, samply on this Mac, generated/seeded correctness cases, independent cargo-deny job; Puffin external viewer optional.
- Verified latest metadata and old-stack caveats: Criterion0.8.2 MSRV1.86, Divan0.1.21 MSRV1.80, samply0.13.1 MSRV1.77, proptest1.12 MSRV1.88, debugdump0.17 targetsBevy0.20 / historical0.6 targetsBevy0.9, Puffin0.20 MSRV1.92 and latest puffin_egui incompatible with old egui; cargo-deny0.20.2 MSRV1.88. No compile-compatibility claims beyond metadata.
- Embark agent verified cargo-fetcher archived; recommend against adding it here. cargo-about optional release license output. Profiling abstraction belongs to aclysma, not Embark.
- Deferred: Gungraun Linux instruction metrics/hosted benchmark dashboards; dhat short optional experiment (explicit experimental/maintenance warning); spatial-index comparisons only after oracle parity.
- Now adding report section11, adoption matrix, progress contract and illustrative commands; then HTML/layout and unchanged-source verification.

## Tooling extension complete — 2026-10-30
- Extended review-report.html with section 11, “Tools & measurable progress,” linked from navigation, executive summary and top-five action list. Original 14 findings and original validation evidence preserved.
- Added prioritized tool/problem/artifact/adoption matrix; published Embark package versions/MSRV and historical egui compatibility; benchmark/profiling/memory/spatial-storage choices; illustrative commands; explicit correctness/performance/dependency progress gates; phased adoption and justified deferrals.
- Supporting primary-source record: review-tools-research.md. Research skill’s background agent completed Embark investigation; main appended alternative tools and measurement plan. All recommendations are research-only and are marked as untested integrations.
- Structural verification PASS: 12 sections including conclusion, 26 unique anchors, 180 links, all internal links resolve, no external script/style dependencies, no unfinished placeholders; report 321,030 bytes.
- Visual QA PASS: inspected local Chrome screenshots of section11 at desktop1440 and mobile420 widths. Mobile document width420 equals viewport420; large tables scroll inside their containers. Screenshots under /tmp/ac3d-review-e7a53d7/tools-desktop.png and tools-mobile.png.
- `git diff --exit-code` PASS; source/manifests/toolchain unchanged. Cargo.lock SHA-256 remains 297057ff03baa34b0413676a4d0d95c5be2ed3d5dd7c880f5c4d8cdc6d069b0a.
- No new builds/tests/profiler experiments warranted or run: this extension changes review artifacts only. No recommended tools installed, services connected or application features enabled.

### Final extension deliverables
- review-report.html — complete assessment plus section11 recommendations.
- review-tools-research.md — cited supporting research and published-version checks.
- review-progress.md — ongoing audit/handoff record.

### Outstanding work
None required for the report extension. Future implementation must first address the existing build/correctness gates; historical-version builds, profiler protocol compatibility, actual speedups and CI noise thresholds remain explicitly unverified.

## Native development baseline — 2026-10-30
User changed scope from analysis-only to native local development: first build/run release; explicitly selected “Minimal native compatibility first.” The existing HTML report remains a historical assessment of e7a53d7 and was not updated.

### Branch and minimal changes
- Created branch codex/native-baseline from e7a53d70c75020e399b4a2d5c1b1153ffbb9075f. Changes remain uncommitted.
- Cargo.lock: only coreaudio-sys0.2.11→0.2.12 and bindgen0.61→0.64; no Bevy/plugin/random/math upgrades.
- rust-toolchain.toml: pin tested Rust1.95.0, minimal profile + rustfmt/Clippy. Installed exact compiler and components. rustup also performed its automatic self-update during initial toolchain install.
- src/main.rs: macOS-only CoreStage::First sync from native Window::inner_size to Bevy Windows; old winit0.27 reports invalid/inaccurate visibleRect resize events on this host. Workaround marked for removal after windowing upgrade. Simulation algorithms/settings unchanged.
- README.md: native build/run/check commands, direct-binary asset root, tool versions/commands and compatibility explanation.
- scripts/smoke-native.py: launches release executable with explicit asset root, catches premature exit/worker panics/missing assets, terminates after 10 seconds. Requires desktop graphics; not a numerical/performance suite.

### Reproduction and diagnosis
- Original pinned1.67.1 offline release: blocked by legacy registry cache, same as review.
- Original locked release with installed Rust1.95: fails in coreaudio-sys bindgen invalid anonymous-enum identifier from current Xcode SDK. Upstream exact failure/fix: https://github.com/bevyengine/bevy/issues/10274.
- Disposable /tmp/ac3d-native-baseline/compatibility-probe with exactly two lock changes compiled unchanged simulation in release (51.46s with partially warm artifacts).
- First direct launch lacked assets (Bevy0.9 resolves relative to executable unless CARGO_MANIFEST_DIR is set). Fixed launch environment; original report’s assumption that cwd alone sufficed for direct launch was incorrect. cargo run supplies the correct root.
- Native runtime then reproduced invalid texture dimension4294967295 (initial1280x720 later false resize event) and wgpu panic.
- Minimized winit-only probe0.27.5: initial1280x720; Resized4294967295² while native inner_size remained1280x720, followed by other inaccurate resize events. No Bevy/GPU needed to reproduce faulty event.
- Actual native source diagnosis: winit0.27.5 macos/view.rs frame_did_change uses visibleRect for resize; window.inner_size uses NSView.frame. App syncs authoritative size before engine update/render.

### Validation and tools
- cargo build --release --locked PASS on pinned1.95.0; seven existing unused-variable/dead-code warnings retained.
- cargo test --release --locked PASS, zero Rust tests present.
- cargo fmt --all -- --check PASS; git diff --check PASS.
- New startup smoke verified RED against the original window path (same real wgpu invalid-size failure), then GREEN with guard (10s, no panic/missing assets).
- A shared target-directory collision between disposable and real package builds initially left the original binary at the common output path despite Cargo reporting the real package fresh. Compared hashes, forced recompilation of real source, and reran smoke successfully. Do not rebuild the old probe into the real target output again.
- Visual QA: temporary /tmp/ac3d-native-baseline/AC3D Native Baseline.app wrapper used because CUA could not bind a raw Cargo executable. CUA confirmed “Autonomous characters 3D” window and screenshot showed rendered menu/target. Direct coordinate interaction failed due CUA window coordinate mapping; camera/boid interactions not exhaustively verified. Initial camera is close to target; no flocking/performance improvement claimed. The app wrapper is left running for inspection.
- Installed separate CLIs: cargo-deny0.20.2 and samply0.13.1. No Puffin runtime crate/UI, hosted services or full dependency upgrade.
- Ran cargo deny --locked --format json --target aarch64-apple-darwin check advisories: expected FAIL, 13 advisory errors (7 unmaintained,5 vulnerability,1 unsound) and3 yanked-version warnings. No blanket exceptions/suppressions added. Raw records at /tmp/ac3d-native-baseline/advisories.jsonl.
- Full logs, original lock, disposable probe, old/native minimal reproducer and patch are under /tmp/ac3d-native-baseline/. New source changes are only the authorized native compatibility work; prior review artifacts remain untracked.

### Next local development steps
1. Inspect native app with cargo run --release --locked; use current baseline before broader changes.
2. Record a controlled native CPU/frame baseline using installed samply (source-line info needs release debug metadata); fix correctness invariants/tests before equivalent-work benchmarking claims.
3. Triage cargo-deny debt; coordinate Bevy/egui/camera/picking migration rather than blindly updating every version.
4. Add optional Puffin scopes/viewer once profiling questions and version integration are chosen.

No benchmark/performance guarantee, numerical fix, full interaction suite, Clippy clean result or upgrade-all completion is claimed. Native baseline and starter CLI installation are complete.

## Commit/push and native profiling checkpoint — 2026-10-30
- User requested committing/pushing completed work and continuing native development.
- Committed d3dc75a (review documents) and21769fd (native release startup compatibility, toolchain, README, smoke script); pushed codex/native-baseline with upstream tracking. Remote SHA verified21769fd41a640c82482efe11e68d81103ea13d2a.
- Built symbolized native release: CARGO_PROFILE_RELEASE_DEBUG=1 cargo build --release --locked PASS (2m06s, existing7 warnings).
- Closed the extra temporary native baseline app before capture.
- samply0.13.1 native launch capture succeeded: requested15s/1000Hz,25 threads,656653-byte compressed profile plus415670-byte pre-symbolication sidecar. App terminated intentionally after recording; sampler exit0. No panic or missing asset warnings.
- Local artifacts in target/native-baseline/: native-21769fd.json.gz, native-21769fd.json.syms.json, metadata.json, capture.log, thread-cpu-summary.json and leaf-cpu-summary.json. Files stay local/ignored; no profile upload or service connection.
- Scenario: default100 agents,naive mode,capacity100,1280x720 requested window,VSync/default presentation; random seed uncontrolled, startup included. This is a diagnostic reference, not a deterministic benchmark or comparable FPS result.
- Sampled thread CPU totals: main thread2917.77ms; five compute workers344.81–365.38ms each. These are CPU deltas across a capture, not frame times or GPU measurements. Symbolized leaf attribution is approximate and includes startup.
- Next: continue coordinated native modernization on a separate branch; baseline branch retained for rollback. Research agent checking exact Bevy0.20.1 APIs against primary sources.

## Native modernization — 2026-10-30
- Continued from pushed baseline80c2ded on separate codex/native-modernization branch; baseline branch remains available for rollback and its local CPU profile remains under target/native-baseline/.
- Verified current registry/tagged APIs; research agent wrote docs/native-migration-notes.md with primary sources. Bevy0.20.1 requires substantial camera/picking/text/scene API changes; glTF now uses WorldAssetRoot/WorldAsset.
- Installed/pinned Rust1.99.1 with rustfmt/Clippy. Updated direct dependencies to Bevy0.20.1, bevy_egui0.43.1, rand0.10.3, nalgebra0.35.0 + glam033 conversion; refreshed the complete transitive lockfile. Initial retained profiling1.0.7 could not compile modern epaint; full cargo update resolved it.
- Removed unused bevy-inspector-egui/rayon/rustc-hash dependencies. Replaced smooth-bevy-cameras and bevy_mod_picking with built-in Bevy camera/mesh-picking plugins and app-owned target selection. Picking opts in only target mesh/camera; vehicle scene meshes are excluded.
- Added native right-drag orbit/middle-drag pan/scroll zoom adapter; controls defer to egui capture. Target keyboard movement remains frame-based as before. Explicit vehicle system ordering: spawn, cleanup, steering, integration. Parallel query API uses automatic batching.
- Removed legacy macOS window-size workaround with old winit API. Updated meshes/materials/UI/font/ambient light/window/scene APIs. Ambient brightness calibrated to1000 for the modern photometric model; FPS moved upper-right to avoid the menu.
- Replaced generational-arena with append-only vectors; no node/point deletions or handle reuse existed. Added subdivision/index regression coverage. This storage migration does not fix octree neighbor semantics or numerical issues.
- Added tests for target selection/background deselection/keyboard gating, camera orbit+scroll application, and vehicle count/recursive child removal. Updated smoke script to recognize current error logs case-insensitively.
- cargo check --tests PASS. Final cargo test --release --locked PASS:4 tests,0 failures; cargo build --release --locked PASS. Both retain two dead-code compiler warnings. Initial cold release test build6m12s; subsequent source rebuilds9.14s(test)/6.70s(binary).
- Final native startup smoke PASS:10s, no panic/error/asset load failure, with no legacy resize guard. cargo fmt and git diff --check PASS. Clippy completed with17 warnings (style/type complexity/dead code); no warning-clean claim.
- Native visual inspection: menu, red transparent target, visible vehicle meshes and unobscured FPS overlay render on this Mac. Temporary app bundle under /tmp/ac3d-native-migration/ used for CUA inspection; closed afterward. CUA coordinate actions did not reliably reach egui, so full interactive camera/menu/picking checks remain unverified; automated ECS tests exercise camera transforms and target selection/movement. FPS rendering verified in the default100-agent scene; this is not a controlled benchmark or a speedup claim.
- cargo deny --locked --format json --target aarch64-apple-darwin check advisories PASS: zero errors/zero advisory warnings, no suppressions. Baseline had13 errors/3 yanked warnings. Raw output /tmp/ac3d-native-migration/advisories-final.jsonl. Only native target is assessed.
- No Puffin integration, hosted services, deterministic performance comparison or flocking correctness fixes in this phase.

### Modernization checkpoint
- Native package/API modernization is complete and validated as above. No remaining dependency advisory exceptions are required for the native target.
- Next substantive work: numerical invariants and exact radius-query correctness, then a seeded headless benchmark with fixed workloads and before/after artifacts. Existing frame-based integration/target motion and octree single-leaf neighbor behavior remain known limitations.

## Controls and follow feedback — 2026-10-30
- Added a persistent egui Controls panel at the lower-right, listing target selection/deselection, camera orbit/pan/zoom, WASD and Space/left Shift movement, and camera-relative movement semantics.
- The panel reads the same TargetSelection state as vehicle steering and displays Follow ON/OFF plus a short behavior description. The target changes from translucent red (idle) to brighter translucent green (selected/following), and back to red on background deselection.
- Color synchronization runs only when TargetSelection changes, including the initial spawn. Extended the existing selection/movement regression test to assert initial red, selected green and deselected red material colors.
- Validation ran sequentially with CARGO_BUILD_JOBS=4: cargo test --release --locked PASS (4 tests), cargo build --release --locked PASS, native startup smoke PASS (10s), formatting and git diff --check PASS. Two existing dead-code compiler warnings remain. No dependency or simulation-math changes.
- Native visual inspection confirmed the Controls panel fits alongside the menu and FPS, and the idle status is visible. CUA coordinate clicks remain unreliable on this host; follow color transitions are covered by the automated ECS test, rather than claimed as a completed manual click test.
- Numerical safety remains the next handoff task. The previous prompt's branch/checkpoint description should use the latest HEAD rather than assuming262dc33.

## Planning moved to plans/ — 2026-10-30
- This log is now historical. New work is tracked with ExecPlans: see `AGENTS.md`, `PLANS.md` and `plans/STATUS.md` (which also records the current status of findings F01–F14).


## Feature expansion roadmap — 2026-10-30
- Owner explicitly authorized dated additions to this historical log/report and plans/STATUS.md only; existing historical content remains intact. New report section 12 reflects the post-GPU working tree, not the report's original e7a53d7 snapshot. Link: [Feature expansion & implementation roadmap](review-report.html#feature-expansion).

### Decisions
- First milestone is bounded Showcase: Original default/Ocean/Dusk heading palette, background, fog and light using the existing cone; no simulation changes from appearance. Frame-world, one auto-orbit, UI hide, correct pause (freeze GPU stepping/frame counter without resetting state), simulation speed. Estimated 3–4 focused sessions, Easy–Medium; review appearance and clock changes separately inside it. Defer tracking, bookmarks, transitions and telemetry.
- Optional Phase 1–2 trails lite uses opaque backward-extended, head-anchored silhouettes from instance rotation, fixed length first; speed-dependent length needs speed in instance data. No IDs/history needed. Sampled historical ribbons remain Phase 3 and need stable IDs.
- Groups/roles need metadata preserved through scatter/separation/integration, not stable identity. Introduce IDs only at the first history, formation-assignment or persistent-selection consumer; define uniqueness and lifecycle then.
- Recommended sequence: Showcase → influence brush (repulsion first) → two-group metadata → primitive obstacles → stable IDs when consumed → historical trails / analytic formations / predator scene / LOD / animated theme → baked music / text-logo morphs / SDF terrain / live audio. This is not a strict dependency chain: obstacles can move earlier and simple predator roles do not require IDs.

### Assumptions and limits
- GPU-owned simulation and shared instanced rendering stay intact; CPU modes remain reference implementations at practical counts, not a million-entity target. Whether every new behavior must support CPU modes remains an owner decision.
- Preserve force laws, defaults, mode-switch respawning and count lifecycle unless explicitly approved. Appearance scale is separate from physical vehicle_size; dense fields/formations need an explicit policy beyond the current follow-only cap.
- Only cited performance measurements are those in plans/completed/2026-10-30-gpu-flocking.md. New effort/performance claims and buffer/triangle counts are labeled estimates/calculations. Current capacity-rounded buffers are about 5.75 MiB at 20k and 184 MiB at 1m; full 16-position histories at million capacity are 256 MiB, exceeding the documented default 128 MiB storage-binding size.
- Bevy 0.20.1 local sources support the proposed APIs; Metal built-in render diagnostics report CPU time, not true GPU timestamps.
- Asset/tool availability, licenses and new feature timings were not verified. No new crates/tools/services installed or runtime features implemented.

### Unresolved owner questions
- Preferred count/hardware/frame budget and first visual theme/style; spacing versus target accuracy; soft avoidance versus hard non-penetration.
- Must both CPU modes support every new behavior, or are selected GPU-only features with small reference fixtures acceptable? Which visual effects may be GPU-only?
- Clock policy for target/animation/music/camera and edits while paused; bundled music versus microphone/system audio; whether existing follow-density/wall-drag decisions change milestone order.

### Future implementation and approval gates
- Roadmap approval is not implementation approval. Work on one approved milestone at a time: present precise scope, planned changes and acceptance criteria; wait for explicit approval; implement only that scope.
- When approved, each milestone gets its own ExecPlan under plans/planned/ from plans/TEMPLATE.md, following PLANS.md. No plan files were created in this writing round.
- Validate correctness, performance and existing functionality with headless CPU fixtures, separately authorized device tests, fixed-view screenshots and repeated scenario-matched benchmarks. One verification owner runs all builds/tests/apps sequentially with CARGO_BUILD_JOBS at most 4, per AGENTS.md.
- Summarize results, limitations and remaining improvements; update documentation/progress/STATUS; stop and request approval before the next milestone. Do not hide behavior changes in presentation work.
- Writing-round checks: HTMLParser stack/tag, navigation and historical-content preservation checks plus git diff --check; no Cargo, app, build/test, network or commit. This round is documentation only.
