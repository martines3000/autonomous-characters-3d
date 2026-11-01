# Add a procedural underwater environment

This ExecPlan is a living document maintained according to `PLANS.md`.

## Purpose

Add Underwater to the selector and scenario parser: turquoise surface light, deep-blue water, exponential fog, soft sun, IBL, and animated caustics on vehicles and sand. Preserve existing environments, simulation settings and Cargo files. Download nothing.

## Progress

- [x] (2026-11-01 21:55Z) Read project guidance and existing rendering paths.
- [x] (2026-11-01 22:11Z) Implement cached water, shared caustics and reversible sand material.
- [x] (2026-11-01 22:11Z) Focused tests pass for gating, clocks, caching and floor restoration.
- [x] (2026-11-01 22:50Z) All locked gates and startup smoke pass; inspect five final screenshots, prove pixel-identical pause/Off restoration, and complete documentation.

## Surprises & Discoveries

The existing ground uses dark vertex colors and disappears in every sky environment. Its water material overrides those colors and restores the original material on exit. First native screenshots exposed a hard far-clip edge on the enlarged sand plane; extend the water-only far plane beyond the fog horizon. Initial fog also washed out distant schools; lower density and deeper blue radiance retain readable schools. A style change while the standard floor material is detached requires its tint to be refreshed on restoration; a regression test covers that path.

## Decision Log

- Decision: Reuse the environment cache, existing ground entity and motion uniform slots; share caustics through WESL. Omit optional particles.
  Rationale: Preserve existing contracts and keep instancing changes separate from concurrent shadow work.
  Date: 2026-11-01.

## Assumptions and Open Questions

Resolved: relative WESL imports compile through Bevy's loader in the native run. No open implementation questions.

## Outcomes & Retrospective

Delivered the selector/parser path, cached 128px procedural sky, convolved diffuse/GGX IBL, green-tinted overhead sun, count-scaled exponential fog, shared layered-wave caustics and reversible sand floor. Uniform size and instance data are unchanged; only Underwater specializes the new fragment work. All requested checks pass: 160 normal tests, 15 GPU tests, warning-free release build, fmt, diff check and ten-second startup smoke. Clippy retains exactly seven baseline warnings in untouched code. Paused water frames and Off round-trip frames are byte-identical. Optional marine snow and animated surface shimmer were omitted; GPU benchmarks and performance measurements were not run. Visual review used the RX 7900 XT native desktop at 3816×2110; this is visual evidence, not a performance claim.

## Context and Orientation

`src/environment.rs` owns cached sky/IBL, exposure and fog. `src/scene.rs` owns the ground. `src/instancing.rs` owns vehicle shader keys and uniforms. `src/scenario.rs` parses settings.

## Plan of Work

Extend selector and cache; create procedural water radiance; share projected caustics with a sand material; test lifecycle and gating; inspect screenshots; run final checks.

## Concrete Steps

Run sequentially through `flock /tmp/ac3d-heavy.lock env CARGO_BUILD_JOBS=4`: fmt check, release tests, release build, release Clippy, ignored `gpu_` tests excluding `gpu_flocking_benchmark`. Run the native binary under the same lock with `CARGO_MANIFEST_DIR="$PWD"` and an `AC3D_SCRIPT` ending in `exit`. Run `git diff --check`. Startup smoke: `flock /tmp/ac3d-heavy.lock env CARGO_BUILD_JOBS=4 AC3D_SCRIPT=/tmp/underwater-smoke.txt python3 scripts/smoke-native.py` (script ends with `12 exit`).

## Validation and Acceptance

Underwater parses and appears in the selector. Other environments retain their settings. Caustics compile only for Underwater and freeze on pause, including with deformation disabled. Sand restores on exit. Five UI-free screenshots show overview, chase at 0.25×, Director, upward view and Glow. Requested checks pass with no new warnings.

## Idempotence and Recovery

Scripts end with exit and overwrite only their dedicated capture paths. Cached assets are reused. Off restores camera and original floor. Leave changes uncommitted for review.

## Artifacts and Notes

Screenshots, scenario and copied logs: `/tmp/claude-1000/-home-martines-Projects-autonomous-characters-3d/af1dacb4-dc47-4bb5-8e8c-02c6cd1b95e8/scratchpad/shots/water/`.

- `01-fish-20k-overview.png`, `02-fish-chase-quarter-speed.png`, `03-director-10k-fish.png`, `04-looking-up.png`, `05-glow-underwater.png`: UI hidden with `ui off`.
- `07-water-paused-a.png` and `08-water-paused-b.png`: identical SHA-256 `b3202fea32509212093d4e26863c10ec396488358bd3d893b1c0dfaa62cebe8a`.
- `06-off-before-roundtrip.png` and `09-off-after-roundtrip.png`: identical SHA-256 `294a99e9c0fdebdbe44f45e10a89663b5a5c316a930b196686bba90241181170`.
- `underwater-{test,build,clippy,gpu,native,smoke}.log`: gate and native-run evidence; no native shader, asset or validation errors.

## Interfaces and Dependencies

Add `Environment::Underwater`, a cached procedural map, a sand material extension and `UNDERWATER` fragment specialization. Reuse motion time and depth slots. No new dependencies.

---

Revision notes:

- 2026-11-01: Initial plan.
