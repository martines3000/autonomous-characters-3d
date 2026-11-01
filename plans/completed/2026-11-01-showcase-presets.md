# Add one-click showcase scenes

This ExecPlan is maintained according to `PLANS.md`.

## Purpose

Expose five live settings bundles at the top of the menu and through validated `showcase <name>` commands. Keep startup defaults unchanged.

## Progress

- [x] (2026-11-01 21:51Z) Read feature code, defaults and repository rules.
- [x] (2026-11-01 21:51Z) Implement shared preset application and regression tests; 159 normal tests pass, 18 ignored.
- [x] (2026-11-01 22:51Z) Inspect all five UI-hidden eight-second screenshots, menu screenshot and recording samples. Encode 150 frames at 30 FPS.
- [x] (2026-11-01 22:51Z) Complete locked fmt, release tests/build, Clippy, GPU correctness tests and native smoke; finish documentation.

## Surprises & Discoveries

- Predators are visible flee controls, not animal models. Formation enabling snaps to its requested morph, so the showcase must initialize the existing formation frame at the sphere endpoint.
- Initial eight-second captures formed tight pellets. Lowering the existing cohesion force cap produced broad streams; hiding predator debug rings kept the flock readable. Fish close chase showed the built-in shape more clearly than the distant establishing shot.

## Decision Log

- Decision: Construct each bundle from the existing defaults, then apply explicit overrides through one shared path.
  Rationale: Preserve startup behavior and prevent settings leaking between scenes.
  Date: 2026-11-01.
- Decision: Tune cohesion caps, query distances and camera shots from native screenshots; hide predator debug rings in the two natural scenes.
  Rationale: Broad flowing clouds and visible animal silhouettes read better in the first eight seconds than isolated dense balls and influence-radius overlays.
  Date: 2026-11-01.
- Decision: Reuse existing predator, formation, motion and Director controls; add no simulation features or dependencies.
  Rationale: Scope and compatibility take priority over new visual mechanisms.
  Date: 2026-11-01.

## Assumptions and Open Questions

- Existing controls produce readable scenes within eight seconds; confirmed with native screenshots after tuning only bundle values. Director selects subjects randomly, so composition varies between launches.
- Environment integration remains deferred to the orchestrator. The optional environment asset identifier is `None` in every bundle.

## Outcomes & Retrospective

Delivered all five live scenes, validated script names and aliases, unchanged startup defaults, exact Calm defaults, order-independent settings application and the existing sphere-to-helix morph. Tests cover all 125 three-preset sequences and repeated final application. No simulation laws, dependencies or environment code changed. Existing flee controls stand in for falcons/shark; their debug rings are hidden, and no predator animal meshes were added. Director framing varies with its randomly selected subject. Environment wiring remains explicitly deferred.

## Context and Orientation

`src/main.rs` owns defaults and menu; `src/scenario.rs` validates and applies scripted commands. `src/swarm.rs` owns groups and scene controls. Appearance, presentation, formation and predators already have live application paths.

## Plan of Work

Add `src/showcase.rs` with named bundles and a single application path. Queue menu clicks through Bevy commands; scenario commands call the same function. Reset camera/navigation runtime through existing controls, without respawning retained vehicles. Add focused default, parsing, setting and order-independence tests. Capture, tune and document.

## Concrete Steps

Run each heavy command sequentially with `flock /tmp/ac3d-heavy.lock` and `CARGO_BUILD_JOBS=4`: formatting, release tests, release build, Clippy, ignored GPU correctness tests (skip benchmark). Launch scripted native captures under the same lock; every script ends with `exit`. Encode the requested 150-frame recording using ffmpeg. Run `git diff --check`.

## Validation and Acceptance

Five buttons appear above Presentation. Valid names apply expected settings; unknown names fail before startup. Calm equals simulation, appearance and presentation defaults. Any preset sequence ends with the same settings as applying its final preset alone; repeated application is idempotent. Five UI-hidden eight-second screenshots and the murmuration recording exist and have been inspected.

## Idempotence and Recovery

Bundle application replaces all owned settings and clears target selection. Existing count/backend semantics remain in charge of vehicle lifecycle. Capture directories must be fresh because recording refuses existing frame names. No commits or resets; review the working diff to recover edits.

## Artifacts and Notes

Capture root: `/tmp/claude-1000/-home-martines-Projects-autonomous-characters-3d/af1dacb4-dc47-4bb5-8e8c-02c6cd1b95e8/scratchpad/shots/showcase/`.

Final images: `starling.png`, `fish.png`, `swallow.png`, `neon.png`, `calm.png`; extra `menu.png` verifies button placement. Scripts and application logs sit beside them. `murm-rec/00000.png` through `00149.png` encode to `murmuration.mp4`: H.264, yuv420p, 3816×2110, 30 FPS, 5 seconds (verified with ffprobe). Initial/candidate captures are retained alongside the final evidence.

`check-*.log` records fmt success; 159 normal tests passed, 18 ignored; warning-free release build; exactly seven existing Clippy warnings, all outside changed code; 15 ignored GPU correctness tests passed, benchmark skipped; native smoke passed. No GPU benchmark or performance claim. `git diff --check` passed.

## Interfaces and Dependencies

New `showcase` module; no crate or toolchain changes. Optional `environment` field remains unwired. Do not edit environment code in `src/scene.rs`.
