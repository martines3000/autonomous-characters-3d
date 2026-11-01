# Flock motion vectors and optional motion blur

This ExecPlan is maintained according to `PLANS.md`.

## Purpose

Add cinematic camera and flock motion blur, off by default, including fixed-step capture.

## Progress

- [x] (2026-11-01 23:09Z) Read rendering, simulation, camera, capture and Bevy prepass code.
- [x] (2026-11-01 23:22Z) Implemented previous transforms, motion attachment, controls and regression tests.
- [x] (2026-11-01 23:24Z) Locked fmt, 176 release tests, warning-free build, baseline-only Clippy, 16 GPU tests and ten-second native startup smoke pass. Chase/flyby captures and the 90-frame recording are verified.

## Surprises & Discoveries

- Bevy 0.20.1 uses frame displacement multiplied by shutter fraction; no wall-time delta enters its blur shader.
- Eight compute storage bindings are already occupied. History shares the existing metadata arena.
- The arena can also serve as the previous-transform vertex stream, avoiding another allocation and per-frame copy. Spawn-only zero-delta updates initialize previous poses to current poses.

## Decision Log

- Decision: Keep 48-byte instances and 64-byte GPU boids. Store frame-start transforms by metadata record in the arena, then write a parallel 32-byte previous-transform stream by sorted draw slot.
  Rationale: Sorting cannot mismatch identities; frame-start history survives all speed substeps without another compute binding.
  Date: 2026-11-02.
- Decision: Bind the arena output range directly as a vertex stream.
  Rationale: The existing storage arena supports vertex usage, removing a separate allocation and copy.
  Date: 2026-11-02.
- Decision: Reuse current deformation for both transforms; suppress vectors and shutter while paused. Skip optional TAA.
  Rationale: Avoid thin-wing temporal history artifacts and additional animation storage.
  Date: 2026-11-02.

## Assumptions and Open Questions

- Previous view binding 2 supplies unjittered clip-from-world; confirmed in the installed Bevy prepass sources. Runtime captures validate layout compatibility.

## Outcomes & Retrospective

Orchestrator review note: the frame-start history and sorted-output tails grow the storage arena from 2.5 to 6.5 metadata records per capacity slot. They are always allocated, even with motion blur off: about 64 MiB more VRAM at one million vehicles (2^20 capacity). The compute writes are uniform-gated, so the step cost with blur off is a skipped branch. Allocating the tails only when a motion-vector view exists is a possible follow-up.


Delivered opt-in motion blur, shutter controls and flock motion vectors in all simulation backends. Default camera components, MSAA and color pipeline policy remain unchanged. GPU history adds 64 bytes per capacity slot in the arena; output binds directly as a vertex stream without a copy. Current deformation is reused for both poses; optional TAA is deferred without a ghosting evaluation. No GPU benchmarks were run.

## Context and Orientation

`src/instancing.rs` owns custom color/depth pipelines. `src/gpu_flock.rs` and its WGSL own sorted simulation. `src/vehicle.rs` owns CPU substeps. `src/presentation.rs` owns camera settings, `src/scenario.rs` parses scripts, and `src/scene.rs` owns shared camera depth-prepass policy.

## Plan of Work

Snapshot CPU poses before substeps. Save GPU poses before integration by metadata identity, emit matched previous instances, and attach motion output using Bevy previous view uniforms. Add opt-in controls and tests, then validate real rendering and recording.

## Concrete Steps

Run every Cargo command and native run under `flock /tmp/ac3d-heavy.lock`, with `CARGO_BUILD_JOBS=4`. Run fmt, release tests, release build, Clippy, then ignored GPU tests excluding benchmarks. Capture scripts always end in exit.

## Validation and Acceptance

CPU frame-start ordering, paused zero-vector helper, parser bounds and unchanged Off components/keys pass. GPU identity/substep history passes. Chase and Director captures render without validation errors; 90 numbered PNGs encode at 30 fps.

## Idempotence and Recovery

Settings are reversible. Capture directories must be fresh. Preserve concurrent environment work; no commits, resets or discarded changes.

## Artifacts and Notes

- `/tmp/ac3d-blur-{fmt,tests,build,clippy,gpu,smoke}.log`: final gates; 176 release tests and 16 GPU tests pass; Clippy remains at seven baseline warnings; native startup smoke passes.
- `chase-on.png`, `chase-off.png`, `director-flyby.png`: requested stills; `chase-settled-{on,off}.png` additionally shows a dense target-following flock.
- `paused-{on,off}.png`: byte-identical (SHA-256 `8f91451b842ca33f74577d7b1fe49fd9badd9192bfab8aa4d6fb6256a2983ef6`).
- `cpu-glow-ao.png`, `cpu-naive.png`, `gpu-underwater.png`: additional shader/backend coverage; native runs report no warnings or validation errors.
- `frames/00000.png` through `00089.png`, then `blur.mp4`: ffprobe counts 90 H.264/yuv420p frames, 3816×2110, 30/1 fps, 3.000 seconds. Encoded with the requested ffmpeg command plus `-threads 4`.
- `runtime.txt`, `variants.txt`, `chase.txt`, `spawn.txt`, `smoke.txt`: reproducible scripts ending in exit. Runtime logs live at `/tmp/ac3d-blur-{runtime,variants,chase,spawn}.log`.

Capture root: `/tmp/claude-1000/-home-martines-Projects-autonomous-characters-3d/af1dacb4-dc47-4bb5-8e8c-02c6cd1b95e8/scratchpad/shots/blur/`.

## Interfaces and Dependencies

No dependency changes. Add CPU previous-transform component, parallel render stream, arena history and Presentation motion blur/shutter settings.

---

Revision notes:

- 2026-11-02: Completed; bound the arena output directly as the previous-transform stream and added spawn-only initialization.
