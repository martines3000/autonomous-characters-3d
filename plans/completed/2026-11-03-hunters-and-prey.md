# Hunters and prey

This ExecPlan is a living document maintained according to `PLANS.md`. The requested destination is `plans/completed/` even while implementation is in progress.

## Purpose

Add optional visible hunter groups that slash through prey schools, causing local flight and temporary panic. Existing presets and disabled behavior remain unchanged.

## Progress

- [x] (2026-11-02 16:21Z) Read guidance and traced group, CPU/GPU steering, rendering and species entry points.
- [x] (2026-11-02 16:40Z) Bounded hunter groups, shared pursuit/pass contract, flee and panic implemented.
- [x] (2026-11-02 16:40Z) Small per-group hero mesh draws, role/size/speed controls, Shark attack and Falcon strike scenarios.
- [x] (2026-11-02 16:52Z) Locked fmt check, 196 release tests, warning-free release build, seven baseline Clippy warnings, 19 ignored GPU tests, native smoke and diff check pass. Director/chase, CPU and zero-prey captures inspected.
- [x] (2026-11-02 16:52Z) GPU benchmarks explicitly deferred to the orchestrator; the million-prey measurement remains their follow-up.

## Surprises & Discoveries

- Review caught CPU draw overwrite, stale formation ranges on role edits, depth/color deformation mismatch and pole orientation mismatch; fixed and added regression coverage.
- The flock compute pipeline already uses eight storage bindings. Optional hunter data must reuse the arena or use a separate compute layout.
- Main flock rendering uses one global instance resource. Hunter draws need their own instance selection without changing the prey draw count.

## Decision Log

- Decision: keep hunters as a separate population owned by hunter groups; the vehicle count remains the prey population. Cap hunters at 16 across groups.
  Rationale: preserves large flock buffers and avoids sorting or redistributing millions of prey when toggling hunters. Prey membership skips hunter groups.
  Date: 2026-11-02.
- Decision: use a separate optional GPU hunter pass and GPU-written arena flee records. Keep an equivalent CPU implementation.
  Rationale: avoids readback and extra storage bindings in the main steering pipeline. Disabled pipelines compile out hunter steering.
  Date: 2026-11-02.
- Decision: do not run GPU benchmarks; collect only brief app timing if practical.
  Rationale: explicit shared-machine instruction delegates benchmarking to the orchestrator.
  Date: 2026-11-02.

- Decision: reserve group 0 for prey; hide target controls on hunter groups and retain their stored targets for later role changes.
  Rationale: ensures prey membership always has a destination and keeps pursuit independent of prey targets/formations.
  Date: 2026-11-02.
- Decision: reuse imported hero geometry with procedural hunter swim/flap on the simulation clock, plus the existing lit/depth/shadow/motion-vector pipeline.
  Rationale: gives each small population its own model without duplicating the main flock's global baked-animation binding. Hunter geometry uses the base pose rather than its baked glTF clip.
  Date: 2026-11-02.

## Assumptions and Open Questions

- Assumption: a parallel exact prey search per hunter is acceptable initially. Validate with quick frame timing; retain a clear cost note if measurements are noisy. A spatial search can follow if the owner finds the measured cost excessive.
- Assumption: Group 0 remains prey; UI and scenarios enforce this so membership always has a valid prey group.

## Outcomes & Retrospective

CPU/GPU pursuit and panic, empty-prey cruising, 16-hunter cap, group controls and both examples are implemented. Review fixes cover draw ownership, formation topology, shared pole-safe heading and look-edit continuity. The first capture showed separate sardine clusters, so Shark attack now gives prey a shared origin target to gather the school. Satellite clusters can split and re-form during flight. All requested gates pass; tuned GPU and CPU views show visible sharks cutting through schools, and zero-prey hunters continue cruising. No dependencies changed and no commits were made. Residual risk is performance with many hunters at one million prey; the mandated idle-machine benchmark is deferred to the orchestrator. The quick paused draw comparison cannot bound simulation overhead.

## Context and Orientation

`src/swarm.rs` owns Groups; `src/vehicle.rs` owns CPU substeps. `src/gpu_flock.rs` and `assets/shaders/gpu_flock.wgsl` implement GPU substeps and the shared arena. `src/instancing.rs` owns custom draws. `src/species.rs` and `src/scenario.rs` are preset entry points. Existing invisible control fields remain in `src/predator.rs`.

## Plan of Work

1. Add group role, population, model, size and speed settings plus capped hunter configuration.
2. Implement deterministic pursuit, lunge and recovery, hunter separation, prey flight and panic on CPU and GPU.
3. Render hunters through small separate instance draws using hero meshes; leave the main flock draw unchanged.
4. Add Shark attack and Falcon strike examples, scenario controls and focused tests.
5. Verify under the shared lock, inspect captures, and update this plan and README.

## Concrete Steps

From repository root, run sequentially with `CARGO_BUILD_JOBS=4` and `flock /tmp/ac3d-heavy.lock`: `cargo fmt --all -- --check`, `cargo test --release --locked`, `cargo build --release --locked`, `cargo clippy --release --locked --all-targets`, and `cargo test --release --locked gpu_ -- --ignored --test-threads=1 --skip gpu_flocking_benchmark`. Run only scripted app sessions ending with `exit`; direct launches set `CARGO_MANIFEST_DIR="$PWD"`. Finish with `git diff --check`.

## Validation and Acceptance

CPU pursuit, pass/cooldown, prey flight/panic, capped populations, presets and disabled equivalence have regression coverage. A seeded small case checks GPU/CPU parity. Existing release and ignored GPU tests pass; release build has no warnings; Clippy adds no warnings. Director and chase captures show distinct larger hunters and a responding school. Benchmark remains explicitly deferred.

## Idempotence and Recovery

Population/role changes initialize hunter state deterministically; look and speed edits preserve trajectories. Role removal releases optional rendering/state. Verification is repeatable. Review the final diff; do not reset, discard or commit changes in this worktree.

## Artifacts and Notes

- Logs: `/tmp/ac3d-hunters-{release-tests,gpu-tests,build,clippy,smoke,app,natural,final}.log`; scripts: `/tmp/ac3d-hunters-{captures,natural,final,smoke}.txt`.
- Captures: `/tmp/ac3d-hunters/` includes `shark-tuned-{wide,chase,cpu}.png`, `falcon-natural-{wide,chase}.png`, `falcon-cpu.png` and `sharks-without-prey.png`, plus earlier tuning views. Script timestamps leave at least 0.5 s between changes and screenshots.
- Quick release app draw comparison, RX 7900 XT (RADV Vulkan), Ryzen 7900X, 1699×1318, Studio/Underwater, 30,000 fish, three sharks, unseeded then paused, immediate presentation: 2-second means 1.271 ms on, 1.271 ms off, 1.285 ms on repeat. Extra draw cost is below this short run's noise; this is not a GPU benchmark. Source: `/tmp/ac3d-hunters-natural.log`.
- Discarded the earlier live on/off comparison: changing Hunter to Prey also changed prey membership and size, so it did not isolate cost. The paused comparison used groups Off to preserve prey geometry.
- GPU benchmarks are deferred to the orchestrator by explicit instruction; million-prey pursuit cost remains unmeasured.

## Interfaces and Dependencies

New `src/hunter.rs`, `assets/shaders/hunter.wgsl` and optional GPU state; Group gains role, count, model, size and speed. No crate or toolchain changes. Existing 464-byte uniforms, 64-byte boids and 48-byte instances stay the same size. Hunter-enabled arenas append 512 bytes; the two hunter state snapshots use 1,536 bytes and instance/history storage uses 1,280 bytes. A 256-lane reduction per hunter selects the nearest prey, breaking exact ties by stable spawn ID. Existing group-range records carry size/speed in reserved components. The original 16 steering variants remain the disabled path; additional variants compile in hunters and/or speed scales only when needed.

---

Revision notes:

- 2026-11-02: Initial plan at the owner's requested destination.

- 2026-11-02: Refined exact search to a parallel reduction; shared existing heading helpers, preserved look-edit continuity, and added lifecycle regressions.
- 2026-11-02: Tuned Shark attack to use a shared prey target; recorded paused draw comparison and capture evidence.
- 2026-11-02: Closed after final gates and scripted native runs. Added empty-prey cruising and bounded panic/role lifecycle checks; grouped physical step settings without lint suppressions.
