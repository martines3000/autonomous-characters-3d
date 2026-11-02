# Visual improvement research (2026-11-01)

This note checks visual options against the code on `feat/boid-camera-and-fixes` and against the Bevy 0.20.1 registry source.

- Frame costs are estimates at 1080p with 100,000 vehicles, unless the text says they were measured.
- Registry paths are relative to `~/.cargo/registry/src/index.crates.io-1949cf8c6b5b557f/`.

## Main constraint

The flock is drawn by a custom instanced pipeline, only in the `Transparent3d` phase (`src/instancing.rs`). It never enters the depth, normal or motion-vector prepass, or the shadow phase.

- **Do not work:** effects that need the prepass do not see the vehicles. These are SSAO, contact shadows, TAA, motion blur and SSR.
- **Do work:** effects that read the main HDR target or the main depth buffer. These are bloom, depth of field, the effect stack and tonemapping.

## Ranked options

| # | Option | What it needs | Est. cost | Risk |
|---|---|---|---|---|
| 1 | Speed-based emissive with `Hdr` and `Bloom` | About 10 lines in the vehicle shader and the camera spawn in `src/scene.rs`. Use additive bloom with a prefilter threshold near 1. Works with MSAA. Gate it to a style, because Studio says "no bloom". | 0.3–0.5 ms, independent of count | Low |
| 2 | `DepthOfField` (Gaussian) in the boid camera, focused at the chase distance | Reads the main depth buffer and supports multisampled depth. | 0.3–1 ms (Bokeh 1–3 ms) | Low |
| 3 | Planar projected floor shadow | A second instanced draw flattened along the light, with a dark blend. Needs a pipeline key bit and about 80 lines. | About one extra flock pass (about 1 ms; about 11 ms with the heavy glider, so off there) | Medium |
| 4 | Speed streak: stretch the dart along its velocity | Vertex shader only, about 10 lines. | About 0 | Low |
| 5 | Schooling-fish and murmuration presets | Scenario scripts or settings bundles using groups, predators and formations. Defaults stay unchanged. | 0 | Low |
| 6 | `Vignette`, subtle `ChromaticAberration`, `LensDistortion` | The post-process effect stack. Film grain does not exist in 0.20.1. | About 0.1 ms | Low |
| 7 | `Tonemapping::AgX` as an A/B option | The default is already `TonyMcMapface`. | 0 | Low |
| 8 | Real cascaded shadow maps | A custom queue into the binned `Shadow` phase plus a depth-only flock pipeline. The floor is `NotShadowReceiver` today. | 1–4 ms | High |
| 9 | `Skybox` or `EnvironmentMapLight` | Needs a KTX2 cubemap asset, and none is in the repo. Weak on a matte flock. | Under 0.3 ms | Low–medium |
| 10 | GPU trails for every vehicle | A ring buffer per record that also moves on compaction. About 25.6 MB at 100k with 16 samples; impractical at one million. | 1–3 ms | High |
| 11 | `MotionBlur` or `TemporalAntiAliasing` | Needs a flock prepass with motion vectors. TAA also needs `Msaa::Off` and ghosts thin darts. | 1–2 ms | High |

These options are rejected for now:

- `ScreenSpaceAmbientOcclusion`, `ContactShadows`, `ScreenSpaceReflections`: the flock is absent from the prepass.
- `VolumetricFog`: needs shadow casters.
- `Atmosphere`: clashes with the stylized sky.
- `Smaa` and `Fxaa`: 4× MSAA is better on thin geometry.

## Recommended order

1. Speed streak (4) and emissive bloom (1) as one optional "Glow" style.
2. Depth of field in the boid camera (2).
3. Projected floor shadow (3), off for heavy models.
4. Murmuration and fish-school scenario presets (5).
5. A screenshot key and fixed-step frame-sequence capture for smooth showcase videos (`Screenshot`, `save_to_disk`).

Real shadow maps, all-vehicle trails, motion blur and TAA need a flock prepass first. Do not start them without measured evidence that the cheaper options fall short.

## Verified Bevy 0.20.1 names

- `bevy_post_process-0.20.1/src/`:
  - `bloom/settings.rs`: `Bloom` (requires `Hdr`), `BloomPrefilter`, `BloomCompositeMode`;
  - `dof/mod.rs`: `DepthOfField`, `DepthOfFieldMode`;
  - `motion_blur/mod.rs`: `MotionBlur`, which requires `MotionVectorPrepass`;
  - `auto_exposure/settings.rs`: `AutoExposure`;
  - `effect_stack/`: `Vignette`, `ChromaticAberration`, `LensDistortion`.
- `bevy_camera-0.20.1/src/components.rs`: `Hdr`.
- `bevy_render-0.20.1/src/`:
  - `view/mod.rs`: `Tonemapping`;
  - `view/window/screenshot.rs`: `Screenshot`, `save_to_disk`;
  - `gpu_readback.rs`: `Readback`, `ReadbackComplete`.
- `bevy_anti_alias-0.20.1/src/`: `taa::TemporalAntiAliasing`, `smaa::Smaa`, `fxaa::Fxaa`.
- `bevy_pbr-0.20.1/src/`: `ssao::ScreenSpaceAmbientOcclusion`, `contact_shadows::ContactShadows`, `ssr::ScreenSpaceReflections`, `fog::DistanceFog`.
- `bevy_light-0.20.1/src/`:
  - `cascade.rs`: `CascadeShadowConfigBuilder`;
  - `volumetric.rs`: `VolumetricFog`;
  - `atmosphere.rs`: `Atmosphere`;
  - `probe.rs`: `Skybox`, `EnvironmentMapLight`.

The `3d` feature set in `Cargo.toml` already includes `bevy_post_process`, `bevy_anti_alias`, `tonemapping_luts`, `smaa_luts` and `ktx2`. None of the options above needs a dependency change.
