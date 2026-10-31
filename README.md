# 3D Boids simulation

A 3D flocking (boids) simulation in Rust and [Bevy](https://bevyengine.org/). Vehicles steer
with separation, alignment, cohesion and wander, and can follow a target you move around.

![300,000 vehicles in GPU mode](docs/images/gpu-300000.jpg)

*300,000 vehicles simulated on the GPU at about 92 FPS (Apple M3 Pro).*

## Project history

### 2022–2023: university project

Built in about a week as a bonus assignment for a university course (first commit December 2022,
web build February 2023). Rust 1.66, Bevy 0.9.1, Rayon and nalgebra:

- naive O(n²) neighbor search and an arena-allocated octree, both multithreaded;
- an egui menu for live simulation parameters;
- vehicles drawn as cones, plus a WASM web build ([original demo](https://ac3d.skippy-ai.com/); removed in 2026,
  still in the `master` history);
- a benchmark comparing the two modes ([chart](meritve.png); slower and less exact than today's numbers).

### 2026: revisited with AI coding agents

Almost four years later the project no longer built on current toolchains. It was picked up again to
see how far today's coding agents (OpenAI Codex and Claude Code, with Claude orchestrating Codex
workers later on) get in a handful of prompts. They upgraded the stack, audited the code, found and fixed bugs,
added tests and benchmarks, optimized it, and moved the simulation to the GPU. I wrote the prompts and
made the decisions; the agents wrote the code, plans and checks.

| | Original (2023) | Now (2026) |
| --- | --- | --- |
| Stack | Rust 1.66, Bevy 0.9.1 | Rust 1.99.1, Bevy 0.20.1, bevy_egui 0.43.1 |
| Builds today | No | Yes (verified on macOS; Linux/Windows setup documented) |
| Simulation | CPU naive / octree | **GPU compute shader (default)** plus CPU naive / octree |
| Vehicles | Slider up to 50,000, naive O(n²) by default | 20,000 by default, slider up to 1,000,000 |
| Rendering | One entity per vehicle | One instanced draw, heading colors, lighting, fog |
| Tests | None | 67 regression tests, GPU parity tests, seeded benchmarks |
| Known bugs | 14 audit findings | All fixed except CI, which isn't wanted |

Bugs found and fixed include:
- NaNs from normalizing zero vectors;
- octree queries that checked one leaf and missed neighbors;
- frame-rate-dependent movement;
- a speed clamp that allowed √3× the maximum;
- a mass slider that ignored existing vehicles;
- followers collapsing into a single point.

The audit is [review-report.html](review-report.html). Each fix has an ExecPlan with evidence in
[plans/completed/](plans/completed/), and [plans/STATUS.md](plans/STATUS.md) tracks the open work.

## Run it

Needs [rustup](https://rustup.rs/); `rust-toolchain.toml` installs the pinned Rust 1.99.1. macOS
needs Xcode command-line tools. Linux needs Bevy's system libraries, clang and mold. For a fresh
machine, follow [docs/new-machine-setup.md](docs/new-machine-setup.md).

```sh
CARGO_BUILD_JOBS=4 cargo run --release --locked
```

`cargo run` supplies the asset root. To start the binary directly:
`CARGO_MANIFEST_DIR="$PWD" ./target/release/autonomous-characters-3d`.

### Controls

| Input | Action |
| --- | --- |
| Right-drag / middle-drag / scroll | Orbit / pan / zoom the camera |
| Left-click the red target sphere | Vehicles follow it (target turns green) |
| Left-click the background | Back to wandering |
| W A S D, Space, left Shift | Move the selected target (relative to the camera) |

The on-screen Controls panel shows these and the follow state. Camera and target input defer to the
menu when it has focus.

### Modes and settings

- **GPU** (default) keeps every vehicle in GPU buffers. The count slider goes up to 1,000,000.
  **Follow neighbor samples** (default 16, 0 = all) caps alignment/cohesion samples while following;
  separation always sees every close neighbor.
- **Octree** and **Naive** run on the CPU (up to 50,000) and are the tested reference behavior.
- Switching between CPU and GPU respawns the flock. Switching Naive/Octree keeps it. Lowering the GPU
  count truncates the flock; zero clears it.
- The Menu groups controls under **Simulation**, **Steering**, **Scene**, and **Vehicles**.
- **Studio** (default) has a vertical navy/indigo sky, a lighter blue horizon, warm key/cool fill
  lighting and a dark blue, rough ground plane. A faint ground grid shares the floor's lighting and
  fog; short corner brackets replace the full wall box. The sky follows the camera, encloses the
  walls, stays inside the far plane and ignores fog/picking. No bloom or per-vehicle lights are used.
- **Classic** retains the original flat background, box/grid colors and layout, lighting, material
  roughness and HSV heading palette. It hides the Studio sky and floor.
- Heading palettes are independent of scene style. Six distinct stops plus a repeated first stop
  make the new palettes continuous around the heading wheel:
  **Ocean** (deep blue, cyan, seafoam, sand, coral, violet), **Dusk** (indigo, magenta, orange, gold,
  rose, purple), and **Aurora** (teal, green, yellow-green, violet, magenta, blue).
  **Aurora** is the default: its green/lime and magenta groups contrast well with the navy sky.
  **Classic** keeps the original HSV function unchanged.
  Changing style keeps the selected palette; choose Classic style **and** Classic palette for the
  original presentation. Studio adds 15% base-color emission alongside its restrained rim term to
  keep distant darts visible. Both styles retain the opaque red/green target and a screen marker
  that remains visible inside dense flocks; click the sphere itself to select it.
- **Auto-frame on count change** defaults on. It waits until the count slider stops changing for
  0.35 seconds and is released, then changes orbit distance while preserving orientation and the
  panned anchor. **Frame world** does the same immediately. Both fit every wall corner; they do not
  track the flock centroid. Classic fog starts beyond the farthest wall. Studio uses gentle depth
  haze colored exactly like its horizon, retaining at least 70% transmission at every wall. Fog
  visibility and the far clipping plane follow the camera, even with auto-frame off. The floor and
  grid scale with the world.
- Simulation defaults live in `GlobalState::default` in `src/main.rs`; appearance defaults live in
  `src/scene.rs`. Physical `vehicle_size` still scales neighbor radii and instance size as before.

### Vehicle appearance

The **Vehicles** section changes the running flock without respawning or resetting it. Shapes are
cached procedural meshes sharing the existing single instanced draw; every shape points along
local +Y and measures 2.5 units long before scaling.

| Shape | Triangles | Geometry |
| --- | ---: | --- |
| Dart (default / Classic) | 14 | Original Cone(0.7, 2.5), resolution 8 |
| Chevron | 12 | Thin closed arrow/chevron prism |
| Fish | 16 | Faceted body and tail fin |
| Bird | 24 | Thin body and swept wings |

**Render size** (0.1–5, default 1) is separate from `GlobalState::vehicle_size`. The latter retains
its original physical meaning, scales neighbor radii, and remains 1 by default. Render size never
changes that field, separation, velocities or steering. **Length** and **Width** (0.25–3, default 1)
are also render-only: length scales forward (+Y); width scales both perpendicular axes (X and Z).
The renderer combines these multipliers with the existing physical size for compatibility. There
is no setting linking visual size to separation. Normals account for the non-uniform proportions.

Coloring defaults to **Heading**, retaining the selected Stage 1 cyclic palette. **Altitude** maps
world y from the lower to upper wall to a palette ramp, clamped outside the walls. **Speed** maps
actual speed divided by the configured maximum speed to that ramp. Similar speeds correctly yield
similar colors. Scalar ramps use the first through sixth distinct stops rather than closing the
heading wheel; Classic scalar coloring uses a red-to-violet HSV ramp. **Solid** uses the sRGB color
picker. Scene lighting, rim and Studio emission apply to every coloring mode.

Style changes preserve shape, multipliers, coloring mode and solid color. To reproduce the original
look, select **Classic** style, **Dart**, render size/length/width **1**, **Heading**, and **Classic**
palette. Appearance changes preserve count, positions, velocities, follow state and all physical settings.

### Custom static vehicle models

In **Vehicles**, choose a scanned model in **Shape / model**, or enter a local `.glb` / `.gltf`
path and press **Load**. Relative paths start at the repository root; absolute model paths work too.
**Rescan** refreshes the nonrecursive `assets/models/` list. Reading, parsing, scanning and mesh
construction run on Bevy's I/O task pool while the flock continues. Status shows loading, loaded
triangle/GPU vertex/primitive counts, or a recoverable error. A failed import keeps the previous
shape/model; fix the file/path and press Load again. New requests supersede old ones. Choosing a
built-in shape cancels a pending import. Rescan does not reload files automatically.

Set the model's source **Forward** and perpendicular **Up** axes (±X, ±Y, ±Z). Forward maps to the
flock's local +Y; up maps to local +Z. Changing forward automatically picks a perpendicular up if
necessary. Defaults are +Z forward / +Y up, matching the example. Baked scene bounds are centered
and uniformly scaled to **2.5 units along the selected forward axis**, then **Model scale** (0.1–5,
default 1) applies. Axis/scale edits reload in the background. Existing Render size, Length and Width
still multiply this mesh. Every step is render-only; physical size, separation and simulation state
stay unchanged. A zero forward extent is an error; choose a valid axis and retry.

Successful loads/reloads select **Model colors** by default. This mode uses each material's linear
base color factor multiplied by optional linear `COLOR_0`. Other coloring modes remain available;
choose one after loading completes. Built-in meshes have no vertex colors and use white in Model
colors mode. Metallic/roughness, lighting, rim and emission remain global scene-style settings;
imported metallic/roughness factors do not override them. Vertex/material alpha is ignored because
this renderer is opaque; alpha blend/mask materials are rejected.

Supported subset:

- GLB embedded BIN buffers and local relative `.gltf` sidecar buffers, including `.bin`. Buffer URIs
  with schemes (including `data:`, HTTP and `file:`), absolute paths, backslashes or percent encoding
  are rejected. Use ordinary relative sidecar names. No network requests or image decoding occur.
- Default scene, or exactly one scene if no default exists. Acyclic node trees with finite,
  nonsingular affine TRS/matrix transforms; transforms and inverse-transpose normals are baked.
  Reflected transforms reverse winding. Cameras/lights are not imported.
- One or more TRIANGLES primitives merged into **one mesh and one instanced draw**, with no
  per-vehicle entities. Float32 VEC3 POSITION; optional float32 VEC3 NORMAL. Missing normals become
  flat face normals. Optional RGB/RGBA COLOR_0 supports float32 or normalized u8/u16. Indices support
  u8/u16/u32 or no indices. UV0 and TANGENT are ignored. Import validates expanded triangles,
  then shares bit-identical position/normal/color tuples through an index buffer. Hard edges,
  material/color seams, triangle order and shader inputs stay unchanged; no approximate welding.
- Opaque single-sided base colors; no textures (base color, normal, metallic/roughness, occlusion,
  emission or extension maps), nonzero material emission, double-sided materials, skins, animations,
  morph targets/weights, UV1+, extra color sets, joint/weight attributes or sparse accessors.
  All required and optional extensions are rejected, including Draco and meshopt. Stage 6 will
  decide texture, material draw and animation support.
- Maximum **100,000 triangles / 300,000 expanded vertices** per selected scene, at most 300,000
  elements per accessor, a conservative **64 MiB combined source/buffer budget** (embedded bytes
  count again when resolved), 4,096 nodes and hierarchy depth 128. These are sanity limits;
  small low-poly vehicles are practical for large flocks. Zero-area triangles are skipped. Empty models,
  non-finite data, zero normals, invalid color ranges, bad references, malformed buffer ranges,
  attribute count mismatches and out-of-range indices fail with a feature and JSON path.

Unsupported errors read `Unsupported <feature> at <JSON path>; expected <supported subset>`.
Examples: `Unsupported texture map at materials[0].pbrMetallicRoughness.baseColorTexture`,
`Unsupported sparse accessor at accessors[0].sparse`, and
`Out-of-range index at meshes[0].primitives[0].indices`. Status also explains that the previous
selection was kept and how to retry. Malformed input, missing resources, scene ambiguity and limits
have distinct messages.

`assets/models/glider.glb` is generated original work under the repository's terms. It has a pointed
gold hull and blue swept wings, authored +Z forward / +Y up: two primitives, 72 source vertices (62 indexed GPU vertices),
24 triangles, source bounds (-1.3, -0.2, -1) to (1.3, 0.2, 2). Regenerate it byte-for-byte with
Python 3's standard library:

```sh
python3 scripts/make_example_models.py
# Deterministic rendering stress model, same surface/colors, 3,456 triangles (250,048 bytes):
python3 scripts/make_example_models.py --subdivisions 12 --output /tmp/ac3d-heavy.glb
```

## How it works

```text
src/
├── main.rs          # app setup, settings, egui menu, camera input
├── vehicle.rs       # CPU steering and integration, tests, benchmarks
├── octree.rs        # exact radius queries for the CPU octree mode
├── gpu_flock.rs     # GPU buffers, compute pipelines, parameters
├── instancing.rs    # one instanced draw and shared appearance uniform
├── vehicle_appearance.rs # shared shapes, render-only settings and instance packing
├── vehicle_model.rs # bounded background static glTF import, axes and model choices
├── scene.rs         # styles, palettes, world grid, fog and framing
├── scenario.rs      # optional timed developer scripts and frame logging
└── target.rs        # selectable, movable target
assets/shaders/
├── gpu_flock.wgsl             # GPU simulation
└── vehicle_instancing.wesl    # vehicle rendering
```

Each frame in GPU mode, compute passes do the following:
1. Bin vehicles into a hashed uniform grid (count, prefix sum, scatter).
2. Compute separation in a finer grid.
3. Steer and integrate.
4. Write instance data straight into the buffer the draw call reads, with no CPU readback.

CPU modes run parallel queries and upload instance data each frame. Steering is frame-rate
independent (tuned at 60 Hz). Separation pushes add up and grow stronger with closeness.

## Performance

### RX 7900 XT, Stage 4 (2026-10-31)

Ryzen 9 7900X, Radeon RX 7900 XT / RADV, Vulkan, Mesa 26.2.5-arch1.1, native release,
Hyprland/Wayland. Physical window fixed at **1280×720**, immediate presentation, GPU wander,
Studio/Aurora, **MSAA 4×**, neutral model/render size and proportions. Each case uses two fresh
launches, six seconds warmup and twelve seconds sampled; no captures during timings. Launch
one-minute load stayed <=3.0; competing GPU jobs were excluded. These are uncapped frame intervals,
not GPU timestamp durations or displayed monitor refresh rates.

Camera yaw/pitch is **0° / 17.28°**. Default framing uses Frame world: distance **2851.546** at
20k and **4876.075** at 100k. Close-up distance is **100** at both counts, inside the flock.
Dart has **14 triangles**; heavy is a deterministic **3,456-triangle** subdivision of the example
glider, preserving its surface/normals/colors. Table means average the two per-run means;
FPS below is 1000 / that mean. Full per-run mean/p50/p95, FPS, settings, loads, rejected attempts,
Bird (24 triangles), small glider (24), MSAA-off, Classic and no-draw controls are in the
[Stage 4 plan](plans/planned/2026-10-31-rendering-optimization.md).

| Vehicles | View | Model | Before mean ms / FPS | After mean ms / FPS |
| ---: | --- | --- | ---: | ---: |
| 20,000 | default | Dart | 1.038 / 963.4 | 1.042 / 960.2 |
| 20,000 | default | Heavy | 5.277 / 189.5 | 2.874 / 347.9 |
| 20,000 | close | Dart | 1.081 / 925.5 | 1.028 / 972.3 |
| 20,000 | close | Heavy | 5.295 / 188.9 | 2.885 / 346.6 |
| 100,000 | default | Dart | 1.025 / 975.6 | 1.019 / 981.4 |
| 100,000 | default | Heavy | 23.868 / 41.9 | 11.538 / 86.7 |
| 100,000 | close | Dart | 1.027 / 973.2 | 1.004 / 996.0 |
| 100,000 | close | Heavy | 24.099 / 41.5 | 11.588 / 86.3 |

Exact import indexing cuts heavy-model GPU vertices from **10,368 to 2,119**, retaining all
3,456 triangles and every position/normal/color bit, hard edge and color seam. Heavy mean frame
intervals fall **45.5–51.9%**. Default Dart keeps its existing mesh/shader path; its small timing
differences are run variation. Hashing/allocation happens once during background import. Models
without shared full attribute tuples benefit less; the import triangle ceiling is not a flock
performance guarantee.

The heavy model costs nearly the same near/far and with MSAA off, while no-draw controls retain
simulation/scene work near 1 ms. This supports a geometry/vertex bottleneck for that model;
CPU/compute/presentation contributions at the low-poly floor remain unseparated. Classic versus
Studio establishes no measurable scene penalty at this floor. No approximate LOD, impostors,
culling, extra depth pass, cheaper shading or MSAA default reduction was justified. There is no
visual quality loss or LOD transition; MSAA stays 4×. Developer scenarios can use `set msaa off`
and `set flock_draw off` for comparisons; normal startup draws the flock with 4× MSAA.

### Historical Apple M3 Pro measurements (2026-10-30)

Apple M3 Pro, release build. Headless median milliseconds per simulation step (no rendering):

| Vehicles | CPU naive | CPU octree | GPU wander | GPU follow (cap 16) |
| --- | --- | --- | --- | --- |
| 10,000 | 25.5 | 3.7 | | |
| 20,000 | | | 0.67 | 1.4 |
| 100,000 | | | 2.5 | 6.4 |
| 300,000 | | | 7.4 | 13.2 |
| 1,000,000 | | | 26.1 | 44.9 |

In the app (vsync, 120 Hz), FPS depends on camera distance:
- 20,000 vehicles: about 107–127 FPS;
- 300,000 seen from afar: about 92 FPS;
- 100,000 seen up close: about 59 FPS.

Details and caveats are in [the GPU plan](plans/completed/2026-10-30-gpu-flocking.md) and
[the CPU benchmark plan](plans/completed/2026-10-30-seeded-headless-benchmarks.md).

## Development

Run from the repository root, one Cargo command at a time, with no app running. Before **every**
Cargo command, run `pgrep -a cargo; pgrep -a rustc` and wait if either reports a process:

```sh
CARGO_BUILD_JOBS=4 cargo fmt --all -- --check
CARGO_BUILD_JOBS=4 cargo test --release --locked
CARGO_BUILD_JOBS=4 cargo build --release --locked
python3 scripts/smoke-native.py   # opens the app for 10 s; needs a desktop session
git diff --check
```

Generator regression checks: `python3 -m unittest discover -s scripts -p 'test_make_example_models.py'`.

The normal tests cover:
- steering safety and CPU modes;
- the camera and the target;
- instance data and GPU parameter encoding.

### Desktop scenario scripts

Set `AC3D_SCRIPT` to a UTF-8 plain-text file to drive the running app without pointer injection.
The full script is validated before the window opens; unknown commands, invalid ranges, unordered
or nonfinite timestamps, and unpaired measurements fail with the file and line number.
Without the variable, no scenario resource or system is installed.

Each line is `<seconds-since-start> <command> [args]`. Blank lines and `#` comments are ignored;
`set solid_color #RRGGBB` treats its color token as a value rather than a comment.
Times are real elapsed seconds, nonnegative and nondecreasing. Equal-time commands retain file
order. Names and enum values are lowercase; paths and labels are single whitespace-free tokens.
Commands run on the first frame at or after their timestamp, so separate dependent screenshots
from changes by several frames. Screenshot paths are relative to the launch directory unless absolute;
parent directories are created and existing files can be replaced. Leave time for capture before `exit`.

```text
# /tmp/scene.txt
0 set mode gpu
0 set count 20000
0 set style studio
0 set palette ocean
0 present immediate
0 camera 0 17.28 3000
1 frame
5 measure begin studio-20k
15 measure end
16 screenshot /tmp/studio-20k.png
18 set count 200
20 screenshot /tmp/studio-200.png
21 follow on
21 target 0 0 0
24 screenshot /tmp/follow-200.png
26 exit
```

```sh
AC3D_SCRIPT=/tmp/scene.txt CARGO_MANIFEST_DIR="$PWD" ./target/release/autonomous-characters-3d
pgrep -a autonomous   # no process should remain after exit
```

| Command | Arguments / behavior |
| --- | --- |
| `set count` (alias `vehicle_count`) | Integer, 0–1,000,000 in GPU; 0–50,000 in CPU modes |
| `set mode` | `gpu`, `octree`, `naive`; mode changes retain existing respawn/count-clamp rules |
| `set style` | `studio`, `classic` |
| `set palette` | `classic`, `ocean`, `dusk`, `aurora` |
| `set shape` | `dart`, `chevron` (alias `arrow`), `fish`, `bird` (alias `glider`) |
| `set model` | Local model path, repository-relative or absolute; asynchronous, preserves selection on failure |
| `set model_forward`, `set model_up` | `x`, `+x`, `-x`, `y`, `+y`, `-y`, `z`, `+z`, `-z`; up must be perpendicular to current forward |
| `set model_scale` | Render-only model multiplier, 0.1–5; axes/scale changes reload the current/pending model |
| `set visual_size` | Render-only multiplier, 0.1–5; default 1 |
| `set length`, `set width` | Render-only proportions, 0.25–3; default 1 |
| `set color_mode` | `heading`, `altitude`, `speed`, `solid`, `model_colors` (alias `model`) |
| `set solid_color` | `#RRGGBB`, `RRGGBB`, or three sRGB numbers `r g b` in 0–1; stored as 8-bit sRGB |
| `set msaa` | Developer measurement control: `4` (default) or `off`; changes the camera only |
| `set flock_draw` | Developer measurement control: `on` (default) / `off`; skips only flock drawing, retains GPU simulation and scene |
| `set auto_frame` | `on` / `off` (also `true` / `false`) |
| `set gpu_max_neighbors` | Integer 0–512 |
| `set octree_size` | Integer 2–500 |
| `set vehicle_mass` | 1–1,000 |
| `set vehicle_max_speed`, `set vehicle_wander_speed` | 0.1–100 |
| `set vehicle_{separation,alignment,cohesion,wander,wall_avoid,seek}_factor` | 0.1–10; expand braces to a single field name |
| `set vehicle_{separation,alignment,cohesion,wander}_distance`, `set vehicle_wander_radius` | 1–100 |
| `follow` | `on` / `off`; uses the existing target selection |
| `target` | `x y z`, finite world coordinates, each within ±10,000,000 |
| `camera` | `yaw_deg pitch_deg distance`; yaw ±360°, pitch ±89.9°, distance 1–10,000,000; looks at origin, positive pitch above it |
| `frame` | Request Frame world, preserving orientation and anchor |
| `present` | `vsync` / `immediate` (`AutoVsync` / `AutoNoVsync`, with platform fallback) |
| `screenshot` | Output image path; Bevy Screenshot API, includes UI |
| `measure begin`, `measure end` | Begin takes a label; end logs one `AC3D_MEASURE` line |
| `exit` | Exit normally; end any measurement first |

`AC3D_MEASURE` reports frames, arithmetic mean/p50/p95 frame milliseconds, FPS (`1000 / mean`),
physical window resolution, count, mode, style, palette, shape, color mode and render multipliers. Samples use `Time<Real>` and include
presentation, CPU/GPU overlap and desktop scheduling; these are rendered frame intervals, not GPU
timestamps. Begin excludes the preceding interval; end includes the final interval. Metadata reflects
settings at end, so keep settings, count, follow state, camera and window resolution fixed throughout
a measurement. Keep the app focused: Bevy throttles unfocused updates to about 60 Hz even with
`present immediate`. Warm up first, capture outside the window, check `uptime`, and repeat runs on
an idle machine. Scripts do not seed or reset the simulation; GPU scatter and interactive frame deltas vary.
Model loading is asynchronous: allow several frames after `set model` before screenshots or choosing a
color mode. Put axis/scale commands before `set model` to avoid redundant reloads. Paths remain single
whitespace-free tokens in scripts; the UI path field accepts spaces.
The setting whitelist and ranges live together in `src/scenario.rs`; extend its typed settings for later stages.

The instance stream stays 32 bytes: position plus packed speed/group in one Vec4, and rotation in
another. The first Vec4's w is a normal finite f32 in [1,2): mantissa bits 0–15 encode normalized
speed, bits 16–22 reserve a group id (0–127, currently always zero). Speed endpoints 0 and 1 are exact;
maximum quantization error is half of 1/65535. No groups are implemented. CPU and GPU write the same
encoding; render scale comes from the shared appearance uniform instead of w. No GPU readback is
needed for Speed coloring.


GPU tests and benchmarks are `#[ignore]`d. They need a real GPU, and device creation can fail inside
sandboxes.

```sh
# GPU regression tests: CPU parity, complete separation, spawn/grow/truncate, bounds
CARGO_BUILD_JOBS=4 cargo test --release --locked gpu_ -- --ignored --nocapture --test-threads=1 --skip gpu_flocking_benchmark

# GPU benchmark; cases via GPU_BENCH_COUNTS, GPU_BENCH_CAPS, GPU_BENCH_MODES (comma-separated)
GPU_BENCH_COUNTS=20000,100000 GPU_BENCH_CAPS=0,16 GPU_BENCH_MODES=wander,follow CARGO_BUILD_JOBS=4 cargo test --release --locked gpu_flocking_benchmark -- --ignored --nocapture --test-threads=1

# CPU benchmark (the exact filter skips the GPU benchmark)
CARGO_BUILD_JOBS=4 cargo test --release --locked vehicle::tests::flocking_benchmark -- --exact --ignored --nocapture --test-threads=1
```

The GPU benchmark warms up 600 steps at 1/60 s, then times 60 steps (encode, submit and wait). It
prints median/p95 times, spacing and speed. Steps over 250 ms end a case early. Benchmark on an idle
machine: check `uptime` and repeat runs.

Optional tools:
- `cargo-deny` (`cargo install --locked --version 0.20.2 cargo-deny`):
  `cargo deny --locked --target aarch64-apple-darwin check advisories`.
- `samply` 0.13.1: `CARGO_MANIFEST_DIR="$PWD" samply record ./target/release/autonomous-characters-3d`.
  For source lines, build with `CARGO_PROFILE_RELEASE_DEBUG=1`.
  Keep profiling separate from timing runs.

Agent and planning conventions are in [AGENTS.md](AGENTS.md), [PLANS.md](PLANS.md) and
[plans/STATUS.md](plans/STATUS.md).

The project is native only; the original WASM web build was removed. The `codex/native-baseline`
branch keeps the first native build that compiled again.

## Roadmap

Section 12 of [review-report.html](review-report.html#feature-expansion) proposes next features,
starting with a Showcase milestone: color palettes, auto-orbit camera, pause and simulation speed.
After that come a mouse attractor/repulsor, multiple swarm groups, obstacles, trails, formations and
predators. Each milestone waits for explicit approval. Open issues are listed in
[plans/STATUS.md](plans/STATUS.md):
- crowding in very large follow flocks;
- rendering cost up close;
- wall avoidance acting as drag.

## Sources

- Boids: [Craig Reynolds](http://www.red3d.com/cwr/boids/),
  [The Nature of Code, ch. 6](https://natureofcode.com/book/chapter-6-autonomous-agents/),
  [Reynolds 1987](https://dl.acm.org/doi/10.1145/37401.37406)
- Octree: [Wikipedia](https://en.wikipedia.org/wiki/Octree),
  [Magnum octree demo](https://magnum.graphics/showcase/octree/?spheres=20&sphere-radius=0.1&sphere-velocity=1.0),
  [Castle Game Engine notes](https://castle-engine.io/vrml_engine_doc/output/xsl/html/section.how_octree_works.html)
- Arena-allocated trees in Rust:
  [article](https://dev.to/deciduously/no-more-tears-no-more-knots-arena-allocated-trees-in-rust-44k6),
  [generational-arena](https://github.com/fitzgen/generational-arena)
