# Natural low-poly creature animation

This ExecPlan is a living document maintained according to `PLANS.md`.

## Purpose

Give six low-poly creatures smooth, distinct motion within 400 triangles, including a real rook for Rook roost.

## Progress

- [x] (2026-11-02 09:35Z) Read guidance, scripts, importer limits and species integration.
- [x] (2026-11-02 09:42Z) Generated six models; import regression checks budgets, one clip, visible motion and seamless position/normal endpoints.
- [x] (2026-11-02 09:53Z) Captured and inspected all six models at 40,000 instances and in isolated close-ups, with alternate phases.
- [x] (2026-11-02 09:53Z) Locked gates pass: fmt, 185 release tests, warning-free build, seven baseline Clippy warnings, 16 GPU tests and native smoke. Native smoke survives ten seconds without errors. Gates were repeated after the final bird motion revision.

## Surprises & Discoveries

- Existing GLBs contain separate per-object clips. First-clip playback moves only the body. Export must merge all animated objects.
- Importer supports four-weight skins, 128 joints, 30 Hz baking, 240 samples and 32 MiB per clip; cubic curves are rejected.

## Decision Log

- Decision: Sample analytic motion every frame at 30 Hz and export one merged LINEAR clip.
  Rationale: Preserve importer/runtime behavior and coordinate all parts.
  Date: 2026-11-02.
- Decision: Use weighted aquatic spines and articulated rigid bird wings.
  Rationale: Continuous body motion with existing single-draw baking and no dependencies.
  Date: 2026-11-02.

## Assumptions and Open Questions

- Validated: existing baking supports these clips. Actual imports and app captures pass within all limits.

## Outcomes & Retrospective

Six assets and the rook species integration are implemented. Release and GPU regression gates pass. All six models load and animate in the app at 40,000 instances and close range. The merged export fixes the former body-only default clip. Bird recovery raises the shoulders above the body while folding and pitching the hands; skinned swimmers have continuous bends and tail lag. Rook roost uses its own asset; shark/falcon remain assets only. No runtime, steering, dependency or default changes. No implementation gaps remain; GPU performance measurement is deferred to the owner as requested.

## Context and Orientation

Blender scripts own assets; `src/vehicle_animation.rs` bakes them; `src/species.rs` selects them; `src/vehicle_model.rs` tests actual imports.

## Plan of Work

Generate smooth birds and skinned swimmers; wire rook and tests; capture app views; run gates and record evidence.

## Concrete Steps

Use the supplied Blender 4.5.9 invocation. Run every Cargo command and native app under `flock /tmp/ac3d-heavy.lock`, with `CARGO_BUILD_JOBS=4`. Run fmt, release tests/build, Clippy, then ignored `gpu_` tests excluding `gpu_flocking_benchmark`. Every app script ends in `exit`.

## Validation and Acceptance

Six animated assets, each at most 400 triangles, with one coordinated seamless clip. Birds flap twice then glide; swimmers carry a growing body wave. Rook species loads rook. Required gates pass without new warnings. Capture close-up and 40k flock views for all assets.

## Idempotence and Recovery

Rerun generators with the same arguments. Named assets are replaced. Leave changes uncommitted for owner review.

## Artifacts and Notes

Logs, previews and captures: `/tmp/ac3d-natural-model-animation/`. `capture.txt` uses species/model selection, 40,000-instance flock views, then isolated chase/orbit close-ups; screenshots are separated from commands by at least 0.5 seconds, and the script ends with `exit`.

| Model | Triangles | Clip samples | Close-up / flock (under that directory) |
| --- | --- | --- | --- |
| Swallow | 294 | 69 | `swallow-close.png`, `swallow-flock.png` |
| Starling | 270 | 55 | `starling-close.png`, `starling-flock.png` |
| Rook | 334 | 129 | `rook-close.png`, `rook-flock.png` |
| Fish | 270 | 31 | `fish-close.png`, `fish-flock.png` |
| Shark | 254 | 55 | `shark-close.png`, `shark-flock.png` |
| Falcon | 350 | 43 | `falcon-close.png`, `falcon-flock.png` |

Each also has a `-close-phase.png` capture 0.5 seconds later. `test.log`, `build.log`, `clippy.log`, `gpu-tests.log`, `smoke.log` and `app.log` hold gate/runtime evidence. `birds-capture.txt` recaptures the final above-body bird recoveries. `birds-app.log` records that run. `shark-close.txt` captures the asset with slower Reef motion for a stable close-up. Shared Cargo cache: `/home/martines/Projects/autonomous-characters-3d/target`; the build was copied under the lock to a task-local binary before capture, preventing another worktree from replacing it.

## Interfaces and Dependencies

No Rust dependency or runtime interface changes. Shared Blender authoring helpers and three new entry scripts.

---

Revision notes:

- 2026-11-02: Initial plan; requested filename uses 2026-11-03.
