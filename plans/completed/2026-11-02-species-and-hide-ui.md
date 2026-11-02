# Species behaviors and Hide UI

This ExecPlan is a living document maintained according to `PLANS.md`.

## Purpose

The owner asked for an immersive UI toggle and for named animal behaviors with drastic differences instead of slider tuning.

## Progress

- [x] (2026-11-02) H key and Presentation → Hide UI toggle `Presentation::ui_visible` (already used by scenarios).
- [x] (2026-11-02) `src/species.rs`: three fish and three bird behaviors, menu section, `species <name>` scenario command, three tests.
- [x] (2026-11-02) Tuned by eye with Director chase/side/wide captures per species; loosened Reef, Swallow and Rook, which first collapsed into tight balls.
- [x] (2026-11-02) Simulation, Swarm and Vehicles sections start collapsed.
- [x] (2026-11-02) Gates: release tests, fmt, warning-free build, smoke, Clippy baseline.

## Surprises & Discoveries

- Cohesion factors near 1 or higher with short separation make tight spherical balls; that is right for sardines and wrong for loose species, which use 0.15–0.2.
- Scenario commands at the same timestamp run before a screenshot captures, so a `boidcam off` beside a `screenshot` captures without the boid camera.

## Decision Log

- Decision: A species changes only behavior (count, steering, motion, predators, shape/model); environment, camera, Director, groups, obstacles and simulation mode stay.
  Rationale: The owner can combine any species with any environment; showcases remain the full-scene presets.
  Date: 2026-11-02.
- Decision: Keep every slider, collapsed by default.
  Rationale: Fine-tuning, tests and scripts depend on them.
  Date: 2026-11-02.
- Decision: Rook roost reuses the starling model at 2.6 visual size.
  Rationale: Avoids a fourth model until the owner asks for one.
  Date: 2026-11-02.

## Outcomes & Retrospective

All six species hold the 60 FPS vsync cap at 3816×2110 on the RX 7900 XT (mean 16.67 ms, p95 ≤ 16.94 ms) with the Director running. Exact GPU cost was not isolated. Motion was judged from stills and short runs, not video.
