# Give vehicles stable banking and procedural flap/swim

This ExecPlan is a living document maintained according to `PLANS.md`.

## Purpose

Birds and fish keep an upright back, lean into turns, and deform with independent simulation-clock phases. Classic heading remains exact and selectable. Only render rotation and vertices change; positions and velocities remain identical.

## Progress

- [x] (2026-11-01 11:45Z) Read contracts and CPU/GPU/render paths; baseline a44ad80.
- [x] (2026-11-01 11:50Z) Baseline GPU wander, run 1: 100k 0.586 ms, 1M 4.501 ms median. A second run was not recorded before the worker stopped.
- [x] (2026-11-01 12:45Z) Codex implemented orientation, deformation, controls and regression tests. The worker was stopped for a shutdown, and the work was saved as WIP `22fb32e`.
- [x] (2026-11-01 21:20Z) Orchestrator finished the stage: fmt, release build with no warnings, 142 release tests, 15 ignored GPU tests (including banked CPU/GPU rotation parity and position/velocity bit preservation), Clippy at the 7-warning baseline.
- [x] (2026-11-01 21:20Z) Desktop check at 3,000 vehicles, boid chase at 0.25×: birds stay upright, bank into turns and flap out of sync. Fish bent into a zigzag, so the swim wave was cut to half a wavelength and half the amplitude.
- [ ] After-change GPU benchmarks and Bird flap on/off frame times are deferred to an idle machine. Other agent sessions were running.

## Surprises & Discoveries

- Observation: local +Y is forward and local +Z is the back/up axis; world +Y is up.
  Evidence: Bird wings lie in the local XY plane. Existing minimal-arc rotations do not preserve the back direction.
- Observation: `assets/environment/` is already untracked at task start. It is outside this work and remains untouched.

## Decision Log

- Decision: fish swim uses k=π (half a wavelength) and 0.5× amplitude.
  Rationale: k=2.5π with full amplitude bent the 2.5-unit fish into a zigzag in desktop screenshots.
  Date: 2026-11-01.
- Decision: keep all steering/integration expressions intact and calculate orientation afterward.
  Rationale: changes must affect only render state, with CPU/GPU position/velocity bit checks.
  Date: 2026-11-01.

## Assumptions and Open Questions

- Assumption: world-up leveling with transported up near vertical gives smooth pole crossings.
  Why reasonable: prior rotation contains the tangent-plane reference missing at the pole.
  Validation: sweep headings through vertical, bound successive quaternion changes and test CPU/GPU parity.
  If wrong: adjust the presentation frame, never steering.

## Outcomes & Retrospective

Bird flocks now read as birds: upright backs, banking into turns, flapping out of sync. The crude built-in fish mesh still reads poorly at close range; hero models (AAA track step 4) should replace it. Performance numbers are deferred, see Progress.

## Context and Orientation

`vehicle.rs::update` and `gpu_flock.wgsl::steer` integrate motion and write rotation. `gpu_flock.rs` packs compute uniforms and exposes seeded tests/benchmarks. `instancing.rs` specializes the shared `vehicle_instancing.wesl` vertex path. `vehicle_appearance.rs` owns built-in shapes; `scenario.rs` validates capture scripts.

## Plan of Work

Measure baseline first; implement matching Rust/WGSL orientation helpers with an exact legacy branch; append presentation-only motion settings; add specialized procedural deformation after animation/before Glow streak; validate math, parity and bit preservation; measure and capture.

## Concrete Steps

From the repository root, sequentially:

Before every Cargo/app command run `pgrep -a cargo; pgrep -a rustc; pgrep -a autonomous`. Use `CARGO_BUILD_JOBS=4` and never overlap. Run GPU step baseline/after twice after uptime, native scenarios ending in exit, then fmt/release tests/build/Clippy/smoke and the requested ignored GPU suite.

## Validation and Acceptance

Check bank law/clamp, exponential frame-rate agreement, pole continuity, exact legacy CPU bits, banked CPU/GPU parity, untouched position/velocity bits and shader gating. Capture three slow Bird frames, two Fish frames, Glow Bird, legacy comparison and about 30 Bird sequence frames.

## Idempotence and Recovery

Style/motion settings are reversible. Keep exact legacy heading available. No commits, pushes, resets, merges or discarded work. Leave unrelated environment assets untouched; every native scenario ends in exit.

## Artifacts and Notes

Logs/scripts: `/tmp/vehicle-motion/`. Screenshots use the owner-specified `shots/motion/` directory. Timing details and exact settings will be recorded after each milestone.

## Interfaces and Dependencies

Add shared orientation semantics in a small Rust motion module and WGSL helpers, presentation-only motion settings in GlobalState for CPU/GPU access, compute/render uniform fields and a procedural-motion shader specialization. No new dependencies or instance/Boid stride changes.

---

Revision notes:

- 2026-11-01: Initial plan.
