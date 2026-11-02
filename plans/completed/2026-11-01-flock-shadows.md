# Opaque flock, prepass and sun shadows

This ExecPlan follows `PLANS.md`.

## Purpose

Opt-in matching animated flock shadows, opaque rendering and depth/normal prepasses, with a count budget and unchanged default presentation.

## Progress

- [x] (2026-11-01) Read constraints and local Bevy 0.20.1 binned APIs.
- [x] (2026-11-01 22:19Z) Implemented queues, light/camera lifecycle, cap and settings.
- [x] (2026-11-01 22:22Z) Verified native captures, cap, SSAO and pixel-identical Off round trip.
- [x] (2026-11-01 22:24Z) Final serialized gate: fmt, 161 release tests, warning-free release build, Clippy with the seven baseline warnings, 15 ignored GPU regression tests, and native smoke. `git diff --check` clean.

## Surprises & Discoveries

- Bevy uses persistent binned phases. Custom direct instancing uses NonMesh bins and removes stale entries each frame, including disabled/despawned flocks.
- Stock material queues dequeue dirty Mesh3d entities before checking materials. Custom queues must run after all corresponding stock queues.
- Opaque NonMesh flocks cannot rely on Bevy allocating mesh-uniform batches; their unused group 2 is explicitly empty.
- The Sunset ghost is baked into `assets/environment/kloppenheim_06_puresky_4k.hdr`, near 54.5% x / 50% y. `/tmp/ac3d-sunset-original.png` confirms it independently of rendering; no gizmo or cubemap seam is responsible.

## Decision Log

- Decision: Keep the legacy color path with both features Off; enable Opaque3d for shadows or SSAO.
  Rationale: Preserve default pixel output while providing a proper opaque feature path.
  Date: 2026-11-01.
- Decision: Leave the baked HDRI ghost unchanged and report its source.
  Rationale: A renderer-side angular retouch would hide source content and also alter lighting/reflections. Clean asset repair is separate work.
  Date: 2026-11-01.
- Decision: Share the vertex entry point across color, prepass and depth-only shadow pipelines.
  Rationale: Baked animation, procedural motion, scale and Glow streak cannot drift.
  Date: 2026-11-01.

## Assumptions and Open Questions

- Native desktop and RX 7900 XT were available. Serialized scripted runs validated all requested presentation modes; timings are indicative under shared-machine load.

## Outcomes & Retrospective

Delivered optional sun/Studio shadows, count cap, opaque color and matching depth/normal prepasses, plus SSAO with reversible MSAA. Default rendering stays on its original path. Native paused Off/On/Off captures have exactly identical decoded RGBA bytes before/after. Four changed bytes with cap zero come from the standard target's own shadow; the flock floor shadow disappears. Motion-vector history and contact-shadow controls remain future work. The source HDRI ghost remains: retouching an HDR lighting asset is separate from the renderer fix.

## Context and Orientation

`src/instancing.rs` owns direct flock draws and appearance buffers. `src/scene.rs` owns Studio lights/floor and camera. `src/environment.rs` reuses the key light as HDRI sun. `src/scenario.rs` parses automation.

## Plan of Work

Add gated renderer settings, binned color/depth/shadow queues, depth pipeline variants, camera SSAO lifecycle, count cap, tests and concise README guidance. Investigate horizon artifact separately.

## Concrete Steps

From the repository root, run sequentially:

```sh
flock /tmp/ac3d-heavy.lock env CARGO_BUILD_JOBS=4 cargo fmt --all -- --check
flock /tmp/ac3d-heavy.lock env CARGO_BUILD_JOBS=4 cargo test --release --locked
flock /tmp/ac3d-heavy.lock env CARGO_BUILD_JOBS=4 cargo build --release --locked
flock /tmp/ac3d-heavy.lock env CARGO_BUILD_JOBS=4 cargo clippy --release --locked --all-targets
flock /tmp/ac3d-heavy.lock env CARGO_BUILD_JOBS=4 cargo test --release --locked gpu_ -- --ignored --test-threads=1 --skip gpu_flocking_benchmark
AC3D_SCRIPT=/tmp/ac3d-shadow-smoke.txt flock /tmp/ac3d-heavy.lock env CARGO_BUILD_JOBS=4 python3 scripts/smoke-native.py
git diff --check
```

The smoke script contains `15 exit`; each capture/measurement script also ends in `exit` and runs under the same lock with `CARGO_MANIFEST_DIR="$PWD"`.

## Validation and Acceptance

Default keys stay unchanged; shadow queue rejects Off/over-cap; scenario values validate. Native logs show no shader errors. Capture Sunset overview/chase and Studio floor; capture SSAO pair if implemented. Record only quick in-app frame times at 20k/100k.

## Idempotence and Recovery

Scripts end with exit; runs serialize under shared lock. Features are reversible with toggles. No commits, dependency changes or simulation edits.

## Artifacts and Notes

All screenshots use `ui off`, under:
`/tmp/claude-1000/-home-martines-Projects-autonomous-characters-3d/af1dacb4-dc47-4bb5-8e8c-02c6cd1b95e8/scratchpad/shots/shadow/`.

- `sunset-bird-overview-20k.png`, `sunset-bird-chase-quarter.png` (0.25×), `studio-floor-on-3k.png`.
- `ao-off-3k-close.png` / `ao-on-3k-close.png`: both MSAA Off, paused 3k close chase, DoF Off.
- `studio-floor-off-3k.png` / `studio-off-roundtrip-3k.png`: 0 differing RGBA bytes; `studio-floor-cap-zero-3k.png`: flock shadow absent. Shadow On changes 121,386 channel bytes.

Quick in-app intervals, one two-second sample per condition, release build, unseeded GPU Bird wander, Sunset, 3816×2110, 4× MSAA, immediate presentation, RX 7900 XT Vulkan / Ryzen 9 7900X. Shared-machine load was roughly 2.6–5.1. These are frame intervals, not GPU benchmarks or isolated shadow-pass timings:

| Count | Shadows Off mean / p50 / p95 (ms) | Shadows On mean / p50 / p95 (ms) |
| --- | --- | --- |
| 20,000 | 1.211 / 1.200 / 1.362 | 1.642 / 1.627 / 1.833 |
| 100,000 | 1.686 / 1.670 / 1.856 | 2.829 / 2.831 / 3.013 |

Scripts/logs: `/tmp/ac3d-shadow-{sunset,close,measure}.{txt,log}`. Final gate logs: `/tmp/ac3d-shadow-final-*.log`. No GPU benchmarks were run.

## Interfaces and Dependencies

Appearance gains shadow/SSAO controls and count cap. No new crates. Uses Bevy 0.20.1 Shadow/Opaque3d/Opaque3dPrepass binned phase APIs.
