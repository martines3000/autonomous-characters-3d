# Low-poly hero models

This ExecPlan is a living document maintained according to `PLANS.md`.

## Purpose

Give the bird and fish showcase presets recognizable species models, in one consistent style, without LOD or new render paths.

## Progress

- [x] (2026-11-01) Searched CC0 sources; none fit (Quaternius "Bird" is textured and static, "Pigeon" only walks). Modeled the swallow in Blender 4.5 and stopped for owner review.
- [x] (2026-11-02) Owner chose the low-poly direction over a detailed 2–5k-triangle bird.
- [x] (2026-11-02) Merged `feat/aaa-hero-model`; Swallow preset loads `swallow.glb`.
- [x] (2026-11-02) Added `fish.glb` (Fish preset) and `starling.glb` (Starling preset) in the same style.
- [x] (2026-11-02) Gates: 182 release tests, fmt, warning-free release build, smoke, Clippy at seven baseline warnings.

## Surprises & Discoveries

- The fish swim joint showed a light seam where equal rings met. Each body half now tapers inside the other.
- Loading a model switches color mode to Model colors, so the Fish preset's two group colors no longer tint the school; the groups still steer separately.

## Decision Log

- Decision: Rigid node animation only (no skinning), under 400 triangles per model, factor materials only.
  Rationale: The existing importer bakes node animation; at this budget LOD is unnecessary.
  Date: 2026-11-02.
- Decision: Presets keep their built-in shape until the asynchronous load lands; other presets cancel a pending load.
  Rationale: Reuses `VehicleModels::request`/`cancel`; startup and Calm defaults are unchanged.
  Date: 2026-11-02.

## Outcomes & Retrospective

| Model | Triangles | Primitives | Loop | Script |
| --- | --- | --- | --- | --- |
| `swallow.glb` | 294 | 14 | 0.8 s flap | `scripts/make_swallow_blender.py` |
| `fish.glb` | 282 | 11 | 0.8 s swim wave | `scripts/make_fish_blender.py` |
| `starling.glb` | 270 | 14 | 0.6 s flap | `scripts/make_starling_blender.py` |

Starling preset, 40k, 3816×2110, RX 7900 XT: hero model and built-in Bird both hold the 60 FPS vsync cap (mean 16.67 ms). GPU-only cost was not isolated. `Showcase::model()` names each preset's model; `showcase_hero_models_load_low_poly_and_animated` loads them all.
