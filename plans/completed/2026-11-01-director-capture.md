# Cinematic Director and fixed-step capture

This ExecPlan is a living document maintained according to `PLANS.md`.

## Purpose

Add optional automatic five-shot cinematography and numbered PNG recording. Preserve simulation defaults and manual controls.

## Progress

- [x] (2026-11-01 21:12Z) Read camera, timing, scenarios and repository guidance.
- [x] (2026-11-01 21:34Z) Add Director, controls, tracked subjects, smoothing and DoF.
- [x] (2026-11-01 21:34Z) Add serialized Screenshot capture and fixed frame timing, including manual camera inertia.
- [x] (2026-11-01 21:34Z) Test pure shot math, restoration, parsing and ECS clock behavior; 152 normal tests pass.
- [x] (2026-11-01 21:39Z) Run locked gates; inspect five stills and 90 frames; encode and verify 3-second MP4.

## Surprises & Discoveries

Screenshot delivery is asynchronous. Waiting updates skip camera and CPU/GPU simulation advancement. Bevy rejects duplicate screenshot targets; recording drains ordinary screenshots before submitting a held frame. The stock pan/orbit controller reads wall time, so recording calls its public update API with the fixed delta. PNG inspection tooling lacked Pillow; PNG dimensions and hashes were verified with Python standard-library header parsing instead.

## Decision Log

- Decision: Reuse follow tracking, spring and lens ownership; isolate pure shot math.
  Rationale: Preserve CPU/GPU contracts and projection restoration.
  Date: 2026-11-01
- Decision: Serialize screenshots and save in completion observers.
  Rationale: Bound memory and associate one fixed step with each output.
  Date: 2026-11-01
- Decision: Multiply effective speed without changing user speed.
  Rationale: Exit restores the latest user setting automatically.
  Date: 2026-11-01

## Assumptions and Open Questions

Recording fixes presentation time; pause and speed still control simulation. Validate N/fps at unpaused 1× without ramp. Verify desktop and ffmpeg through real captures.

## Outcomes & Retrospective

Delivered all five Director shots, C/UI/scenario controls, subject DoF, bounded shake and non-destructive speed ramp. Recording serializes screenshots, freezes waiting simulation/camera frames, drains ordinary screenshots, and preserves fixed timing for manual camera inertia too.

All requested gates passed under the shared lock: fmt, 152 normal release tests, warning-free release build, Clippy with exactly seven baseline warnings, 15 ignored GPU tests excluding the benchmark, native startup smoke, and diff check. No dependencies or scene.rs changes. GPU benchmarks remain for the orchestrator as requested.

Captured 10,000 Bird vehicles with UI hidden: chase.png, side.png, flyby.png, orbit.png and wide.png. The 90 PNGs have contiguous names 00000.png–00089.png and 90 unique SHA256 hashes. ffprobe confirms H.264/yuv420p, 3816×2110, 30 fps, 90 frames and exactly 3.000000 seconds. No known functional gaps; recording remains an offline capture that waits for readback and disk writes.

## Context and Orientation

`src/presentation.rs` owns clocks and camera controls. `src/follow.rs` owns tracking, spring and DoF. `src/scenario.rs` parses scripts. Add `src/director.rs` and `src/capture.rs`. Avoid `src/scene.rs`, edited by another worker.

## Plan of Work

Build shot model, wire controls, serialize capture, test timing, run gates, inspect artifacts.

## Concrete Steps

Sequentially under `flock /tmp/ac3d-heavy.lock` with `CARGO_BUILD_JOBS=4`: cargo fmt check, test release locked, build release locked, clippy release locked all-targets, ignored GPU tests excluding benchmark. Run scripted native captures under the same lock. Finish with `git diff --check`.

## Validation and Acceptance

Five distinct smooth shots; C, UI and scripts toggle Director; manual input and presets exit. Bounded deterministic shake, subject DoF and reversible ramp. Exactly 90 numbered PNGs at 30 fps; ECS proof of fixed steps despite delayed callbacks; MP4 via ffmpeg.

## Idempotence and Recovery

No dependencies or persistent simulation data change. Record into an empty directory; reject existing output names. Review only feature edits for rollback; no commits or destructive git commands.

## Artifacts and Notes

Native scenario and evidence: `director.ac3d`, `smoke.ac3d`, `capture.log`, five stills, `rec/` and `director.mp4`. Recording completed before the scripted exit; no runtime/asset errors appeared. Inspection covered every shot and recording frames 0, 45 and 89.

Artifacts: `/tmp/claude-1000/-home-martines-Projects-autonomous-characters-3d/af1dacb4-dc47-4bb5-8e8c-02c6cd1b95e8/scratchpad/shots/cine/`.

## Interfaces and Dependencies

Presentation Director settings; `director on|off`, `set shot_length`, `set shake`, `set speed_ramp`, `record <abs-dir> <frames> <fps>`. Existing Bevy Screenshot API; no new crates.

---

Revision notes:

- 2026-11-01: Initial plan.
