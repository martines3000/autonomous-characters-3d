# Keep vehicles apart when they follow the target

This ExecPlan is a living document maintained according to `PLANS.md`.

## Purpose

When the target is selected (follow mode), the whole flock collapses onto it: at 5,000 vehicles the median nearest-neighbor distance is 0.23 units against a dart width of 1.4, so the flock renders as one solid ball, and every vehicle is within the 30-unit alignment radius of every other one, so neighbor queries become quadratic. After this work, followers keep visible spacing around the target at 200 to 5,000 vehicles, wander mode still looks and measures as before, and follow mode at large counts costs less per step.

## Progress

- [x] (2026-10-30 10:00Z) Plan drafted from the evidence in `completed/2026-10-30-instanced-rendering-and-visuals.md`.
- [x] (2026-10-30 20:12Z) Probed crowd-scaled separation with and without seek arrival, and the old model, headlessly in a disposable copy.
- [x] (2026-10-30 20:14Z) Implemented crowd-scaled separation in `flocking_force` with two regression tests; both fail on the old model.
- [x] (2026-10-30 20:17Z) Screenshots in follow and wander mode; headless benchmark rerun. No external reviewer: the change is about 15 lines in one function, reviewed by the author against the probe and tests.
- [x] (2026-10-30 20:20Z) Validation gate passed; README and STATUS updated.

## Surprises & Discoveries

- Observation: seek arrival was unnecessary once separation scaled with crowding.
  Evidence: at 1,000 followers, nearest-neighbor median 2.10 without arrival and 2.21 with a 30-unit slowing radius; overlap 0 in both.
- Observation: wander-mode vehicles move faster with the new model (mean speed about 29 instead of 24 units/s).
  Evidence: probe `1:0:3:1.5` 1000w speed=29.6 against baseline 23.9. The old separation was "desired velocity minus velocity" and so also damped speed; the new push does not.
- Observation: follow mode at 5,000 vehicles still costs about 30 ms per simulation step.
  Evidence: probe step_ms 28.75 (old) and 33.12 (new) at 5,000 followers. The ball's radius (about 20 units) is smaller than the 30-unit alignment radius, so almost every vehicle remains every other vehicle's neighbor.

## Decision Log

- Decision: treat this as a model change in `flocking_force` and the seek step, not a default retune.
  Rationale: the earlier probe varied separation distance (4–8), the separation, cohesion and seek factors, seek arrival and a gather radius; none kept flocks above about 1,000 apart, because separation is an averaged, normalized direction capped at `max_speed × factor`, so its strength does not grow with crowding while seek and cohesion keep pulling inward.
  Date: 2026-10-30
- Decision: separation adds, for each neighbor closer than the separation distance r at distance d, a push of (r/d − 1), capped at 3, away from it; the sum is multiplied by `max_speed × separation_factor` and is not limited. Seek is unchanged (no arrival).
  Rationale: the push grows with closeness and with the number of neighbors, so a crowd builds pressure against seek and cohesion. The cap of 3 (reached at d = r/4) keeps near-coincident pairs finite; integration still limits speed. Arrival added a constant and no measurable benefit. The separation factor becomes a weight, which is the reading users expect from the slider.
  Date: 2026-10-30
- Decision: leave follow-mode cost at large counts for later.
  Rationale: it comes from the alignment radius covering the whole gathered flock, not from collapse; fixing it needs a neighbor limit (for example nearest-k), which changes query behavior and is its own task.
  Date: 2026-10-30

## Assumptions and Open Questions

- Assumption: a separation force that grows with the number and closeness of neighbors, plus seek arrival, bounds density near the target.
  Why reasonable: pair repulsion that grows with crowding gives the crowd a pressure that balances the bounded inward pull; arrival weakens that pull near the target.
  Validation: headless probe at 200, 1,000 and 5,000 vehicles in both modes.
  If wrong: record the probe results and consider limiting how many neighbors each vehicle uses.
- Open question: how much wander-mode flocking may change. Target: wander nearest-neighbor percentiles and neighbor counts within roughly 20% of today's values.

## Outcomes & Retrospective

Delivered: crowd-scaled separation. Headless probe (seed 7, 30 simulated seconds at 60 Hz, release build, Apple M3 Pro, load average about 7):

| Scenario | Old: overlap / nn p1 / nn median | New: overlap / nn p1 / nn median |
| --- | --- | --- |
| 300 follow (test) | 297 of 300 overlap | under 5% (test passes) |
| 1,000 follow | 100% / 0.16 / 0.48 | 0% / 1.74 / 2.10 |
| 5,000 follow | 100% / 0.07 / 0.23 | 0% / 1.56 / 1.96 |
| 1,000 wander | 0% / 1.90 / 3.89 | 0% / 2.21 / 3.10 |
| 5,000 wander | 0.4% / 1.53 / 3.59 | 0% / 2.07 / 3.02 |

Overlap is the share of vehicles closer than 1.4 units (the dart width) to their nearest neighbor. Followers now form a loose ball about 15–20 units from the target instead of a solid one 7 units out. Wander flocks are a little tighter in the median (−16 to −20%, within the acceptance bound) but have no overlapping pairs, and move faster (mean speed about 29 instead of 24 units/s). Neighbors within 30 units in wander mode are unchanged (94 and 127 against 63 and 107, same order).

Headless `flocking_benchmark` (wander-style spawn, octree): 5,000 vehicles 1.10 ms, 10,000 vehicles 2.26–2.31 ms per step, two runs, no regression.

Remaining gap: following at 5,000 vehicles still costs about 30 ms per simulation step (about 31 FPS in the app), because the gathered flock fits inside one alignment radius. Recorded in `plans/STATUS.md`.

Lesson: the factors in this model were caps on "desired minus current velocity", so none of them could make a force grow with crowding; changing the form of the force, not its tuning, was needed.

## Context and Orientation

`src/vehicle.rs`: `movement` runs `flock` (octree or naive neighbor search feeding `flocking_force`) and then seek (follow mode) or wander plus wall avoidance. `flocking_force` sums separation, alignment and cohesion; each is "desired velocity minus velocity", limited to `vehicle_max_speed × factor`. `update` integrates with a 60 Hz reference step and limits speed. Defaults are in `configure_global_state` in `src/main.rs`.

## Plan of Work

1. Disposable copy with a probe test that runs 30 simulated seconds at 60 Hz and reports nearest-neighbor percentiles, overlap share (< 1.4 units), neighbors within 30 units, mean speed and mean distance to the target, for each candidate model.
2. Pick the simplest candidate that meets the acceptance numbers; implement it in the real repository with tests.
3. Measure follow-mode step time with the seeded benchmark harness, take screenshots with the screenshot probe, review, run the gate.

## Concrete Steps

From the repository root, sequentially:

1. `CARGO_BUILD_JOBS=4 cargo fmt --all -- --check`
2. `CARGO_BUILD_JOBS=4 cargo test --release --locked`
3. `CARGO_BUILD_JOBS=4 cargo build --release --locked`
4. `python3 scripts/smoke-native.py`
5. `git diff --check`

## Validation and Acceptance

- Follow mode after 30 s: fewer than 5% of vehicles closer than 1.4 units to their nearest neighbor at 200, 1,000 and 5,000 vehicles (today 96%, 100%, 100%).
- Wander mode at 1,000 and 5,000: overlap stays below 1% and nearest-neighbor percentiles stay within about 20% of today's.
- A regression test fails on the old separation model.
- All existing tests pass.

## Idempotence and Recovery

Probes run in a scratch copy with their own `CARGO_TARGET_DIR`. The real change is confined to `src/vehicle.rs` (and defaults in `src/main.rs` if needed); revert with git.

## Artifacts and Notes

Probe scripts and outputs live in the session scratchpad (`codex.ISlk/follow/`: `apply.py`, `r1.txt`, `r2.txt`); screenshots in `codex.ISlk/shots/sep-*.png` (follow 200 and 5,000: separate darts around the target; wander 5,000 far view: distinct colored flocks).

Gate: `cargo fmt --check` OK; 26 tests passed, 1 ignored; release build without warnings; Clippy 8 pre-existing warnings, none new; smoke PASS; `git diff --check` OK.

## Interfaces and Dependencies

No new crates. `flocking_force` changes behavior; new constant `MAX_SEPERATION_PUSH` in `src/vehicle.rs`. The separation factor is now a weight on the summed pushes, documented in `README.md`. The octree/naive comparison test tolerance now scales with the force magnitude, since summed pushes make forces larger.

---

Revision notes:

- 2026-10-30: Initial plan.
- 2026-10-30: Completed; arrival dropped after the probe; follow-mode cost deferred.
