# Stronger, cheaper volumetric shafts

This ExecPlan is a living document maintained according to `PLANS.md`.

## Purpose

Make optional Sunset and Underwater shafts clearly visible with a target of about +3 ms at 4K/20k. Preserve default Off, showcase settings, and exact Off round trips.

## Progress

- [x] (2026-11-02 09:30Z) Read repository guidance, existing fog ownership and installed Bevy API/shader.
- [x] (2026-11-02 09:37Z) Captured baseline; 4K/20k overhead falls from +6.72/+8.20 ms to +2.66/+2.20 ms.
- [x] (2026-11-02 09:49Z) Inspected before/after and side/up captures; both Off round trips and paused On pairs are byte-identical.
- [x] (2026-11-02 09:49Z) All locked gates and smoke pass; final measurements and documentation complete.

## Surprises & Discoveries

- Bevy jitter uses frame-index interleaved gradient noise, intended for TAA. Existing camera has no temporal reconstruction.
- The shader marches the configured step count across each intersected volume; tighter bounds improve sampling and reduce screen coverage, not steps per intersecting pixel.
- An old Skybox survives while the next HDRI loads. The environment sets sun illuminance to zero during that wait; fog now waits too, preventing a cached texture from using the previous sun. A regression test covers the transition.

## Decision Log

- Decision: Reuse Bevy fog and existing controls; keep temporal jitter zero.
  Rationale: Preserve lifecycle and avoid moving noise without TAA. Correctness and reversibility precede performance.
  Date: 2026-11-02.

- Decision: Default to 16 steps, shrink full volume scale from 2.2× to 1.2× wall half-extents, and cache a 32³ sun-aligned texture for each of Sunset/Underwater.
  Rationale: Fourfold lower march budget; shorter rays resolve flock shadows better. Sparse, feathered columns add contrast instead of a uniform bright veil. Stronger medium scattering/light gain leaves surface lighting untouched. Midday retains its subtle coefficients.
  Date: 2026-11-02.

## Assumptions and Open Questions

- A count-scaled central volume can cover the showcase flock with fewer ray steps. Validate with side/up views; enlarge if clipping appears.
- Measurements are quick frame intervals, not GPU benchmarks. Other work may affect timing; report conditions and limitations.

## Outcomes & Retrospective

Delivered stronger cyan beams through fish and warm shafts through birds, with unchanged default/showcase Off behavior. Final 4K/20k added frame intervals are +2.59 ms Sunset and +1.75 ms Underwater, meeting the approximate +3 ms target on this machine. Both environments restore Off pixels exactly; paused On frames are identical. All gates pass: 186 release tests, 16 GPU tests, warning-free release build, seven existing Clippy warnings, native smoke and diff check. No GPU benchmark ran. The bounded medium deliberately leaves outer-world stragglers unfogged; 16 steps can band fine moving shadows. Other GPUs and longer idle-machine timing remain unmeasured.

## Context and Orientation

`src/volumetric.rs` owns medium texture, components and reversible fill visibility. `src/scene.rs` owns default quality and derived shadow policy. `src/showcase.rs` must retain its existing presets.

## Plan of Work

Capture baseline; reduce ray budget and bound fog; strengthen scattering and sun-aligned density; inspect paired captures, measure 20k, verify Off; run gates and close plan.

## Concrete Steps

Every Cargo/native command uses `CARGO_BUILD_JOBS=4` and `flock /tmp/ac3d-heavy.lock`. Run fmt check, release tests, release build, Clippy and ignored GPU tests excluding the GPU benchmark, sequentially. Native scenarios end in `exit`; direct launches set `CARGO_MANIFEST_DIR`.

## Validation and Acceptance

Sunset starling and Underwater fish captures visibly show shafts. Compare old/new on and new off/on at 20k. Target approximately +3 ms at 4K; record any gap. Existing lifecycle and default tests pass; paused Off capture pixels match exactly; no added compiler/Clippy warnings.

## Idempotence and Recovery

Named `/tmp/ac3d-volumetric-shafts` artifacts can be regenerated. Fog assets remain cached and components derived. Restore only reviewed tuning hunks if rejected; never reset/discard the worktree.

## Artifacts and Notes

Artifacts: `/tmp/ac3d-volumetric-shafts/`.

Release, Ryzen 9 7900X + RX 7900 XT/RADV Vulkan (Mesa 26.2.5), 3816×2110, 4× MSAA, immediate present, UI/Director/boidcam off, 20k custom-model vehicles. Live GPU spawn/frame RNG; no fixed seed. Two-second `measure begin`/`measure end` windows after warmup; these include CPU/GPU overlap, presentation and scheduling, not GPU timestamps. Sunset camera `-116 -5 850`; Underwater `48 -15 650`, saved Shadows On. Load averages were 0.02/0.03/0.08 before baseline build and 4.74/3.29/1.89 before final measurement lock wait; shared-machine load limits precision.

| Scene / revision | Off mean / p95 ms | On mean / p95 ms | Added mean ms |
| --- | --- | --- | --- |
| Sunset, original 64 | 3.197 / 3.510 | 9.919 / 10.268 | 6.722 |
| Sunset, final 16 | 3.260 / 3.542 | 5.849 / 6.227 | 2.589 |
| Underwater, original 64 | 5.030 / 5.338 | 13.228 / 14.658 | 8.198 |
| Underwater, final 16 | 4.932 / 5.285 | 6.685 / 7.206 | 1.753 |

Earlier tuned run: Sunset 3.219→5.878 ms (+2.659); Underwater 5.089→7.292 ms (+2.203). No benchmark or other native app overlapped these runs.

Before/after: `sunset-{baseline,after}.png`, `underwater-{baseline,after}.png`, `underwater-up-{baseline,after}.png`; extra Sunset angles `sunset-{side,oblique}-{baseline,after}.png`. Same showcase/camera/pause timing, independent live simulations. `capture-baseline.txt` and `capture-final.txt` reproduce the primary views. The saved original renderer is `baseline-app`.

`pixel-checks.txt`: both `*-off-{before,after}.png` pairs and both `*-{after,stable}.png` pairs have MAE 0 and identical PNG bytes. Removing flock casters changes On captures (normalized MAE 0.003880 Sunset / 0.001346 Underwater); this includes surface-shadow changes. `baseline.log` and `measure-final.log` contain the timing rows above; `measure-after.log` is the earlier tuned run (its legacy labels still say 64). `check-{fmt,test,build,clippy,gpu,smoke}.log` contain final gates.

## Interfaces and Dependencies

No new dependencies or public controls. `Appearance::volumetric_steps` defaults to 16 and stays editable from 16–128. `EnvironmentParticles` replaces the water-only cache; `particle_columns` receives the loaded sun direction.

---

Revision notes:

- 2026-11-02: Implemented, measured and completed under the requested 2026-11-03 plan filename.
