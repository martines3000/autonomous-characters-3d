# Allocate GPU motion history only while blur is enabled

This ExecPlan is maintained according to `PLANS.md`.

## Purpose

Remove the unused history tail when motion blur is off, saving 64 MiB at one million boids after capacity rounding.

## Progress

- [x] (2026-11-02 09:32Z) Read repository rules and trace arena offsets and draw bindings.
- [x] (2026-11-02 09:37Z) Implement conditional allocation, immediate shrinking and current-pose initialization.
- [x] (2026-11-02 09:37Z) Focused GPU tests pass for both sorting paths, including prefix and boid preservation.
- [x] (2026-11-02 09:39Z) Formatting, 185 release tests, warning-free release build, baseline-only Clippy and 17 ignored GPU tests pass.
- [x] (2026-11-02 09:41Z) Native startup/toggles pass; paused Off round trip is byte-identical; plan closed.

## Surprises & Discoveries

- Paused frames skip steering, so enabling history requires one initialization dispatch.
- Queue writes precede encoder copies. Simultaneous scene edits must upload after the preserved prefix using encoder copies.

## Decision Log

- Decision: Shrink immediately on disable and copy the complete arena prefix on either toggle.
  Rationale: Reclaim memory predictably while preserving metadata, both order maps, scene records, formation ranges and tracking verbatim.
  Date: 2026-11-02.
- Decision: Retain all state and scratch buffers; rebuild only the arena and its bind group.
  Rationale: Avoid respawning or unnecessary allocations. Keep shader layout and binding count unchanged.
  Date: 2026-11-02.
- Decision: Initialize previous poses to current poses once on enable, after all live substeps or during paused maintenance.
  Rationale: Ensure zero initial flock motion without adding steady-state dispatches or copies.
  Date: 2026-11-02.

## Assumptions and Open Questions

- Motion-vector views determine whether GPU history is needed. This matches the existing `params.motion.w` contract; native toggles validate it.
- Capacity remains rounded to a power of two. GPU tests check exact arena sizes, not process-wide memory estimates.

## Outcomes & Retrospective

The arena now omits history when blur is off and shrinks immediately on disable. All 185 release tests, 17 ignored GPU tests and native smoke pass. Release build has no warnings; Clippy retains seven baseline warnings. Paused Off round-trip PNGs are byte-identical. GPU benchmarks and timing measurements remain reserved for the orchestrator; no timing improvement is claimed.

## Context and Orientation

`src/gpu_flock.rs` owns arena allocation and render scheduling. The history/output tail follows tracking. `assets/shaders/gpu_flock.wgsl` writes it; `FlockInstances.previous` binds its output only when allocated. GPU regression tests live in `src/vehicle.rs`.

## Plan of Work

Make history optional, preserve the entire prefix with `copy_buffer_to_buffer`, rebuild bindings, initialize first poses, then test toggles and document memory lifetime.

## Concrete Steps

Run sequentially from the repository root. Every Cargo/app command holds `/tmp/ac3d-heavy.lock`; every Cargo command sets `CARGO_BUILD_JOBS=4`.

1. `flock /tmp/ac3d-heavy.lock env CARGO_BUILD_JOBS=4 cargo fmt --all -- --check`
2. `flock /tmp/ac3d-heavy.lock env CARGO_BUILD_JOBS=4 cargo test --release --locked`
3. `flock /tmp/ac3d-heavy.lock env CARGO_BUILD_JOBS=4 cargo build --release --locked`
4. `flock /tmp/ac3d-heavy.lock env CARGO_BUILD_JOBS=4 cargo clippy --release --locked --all-targets`
5. `flock /tmp/ac3d-heavy.lock env CARGO_BUILD_JOBS=4 cargo test --release --locked gpu_ -- --ignored --test-threads=1 --skip gpu_flocking_benchmark`
6. Run the native smoke script under the lock with an `AC3D_SCRIPT` ending in `exit`, then `git diff --check`.

## Validation and Acceptance

GPU toggles preserve bitwise boid state, instance data and the complete prefix, report 41,936 bytes off versus 107,472 bytes on at capacity 1,024, and produce current previous poses when enabled. Both sorting paths and four live substeps are covered. Simultaneous scene/predator edits survive the prefix copy. Release build stays warning-free; Clippy adds no warnings beyond the seven baseline warnings.

## Idempotence and Recovery

Checks can be repeated under the shared lock. Runtime toggles are reversible. Review or revert only this task's patch through normal version control; do not discard other work.

## Artifacts and Notes

Focused GPU command: `flock /tmp/ac3d-heavy.lock env CARGO_BUILD_JOBS=4 cargo test --release --locked gpu_previous_transforms -- --ignored --test-threads=1 --nocapture` — two passed.
Final gate logs: `/tmp/ac3d-lazy-history-{tests,build,clippy,gpu}.log`.
Native script and 3816×2110 captures: `/tmp/ac3d-lazy-history/{smoke.txt,off.png,paused-on.png,off-again.png,live-on.png}`. Smoke log: `/tmp/ac3d-lazy-history-smoke.log`. Both Off PNGs have SHA-256 `e0b25eacc1fb53ee955d97f3aeeafcbe6b9fe99bcc33a016e1cbd3f19906e5c7`. Screenshots were visually inspected. Final `git diff --check` passes.

## Interfaces and Dependencies

Adds history allocation state, arena resizing and an initialization compute entry point. No dependency, uniform layout or storage binding count changes.
