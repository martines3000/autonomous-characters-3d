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
| Tests | None | 33 regression tests, GPU parity tests, seeded benchmarks |
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
| Left-click the red target | Vehicles follow it (target turns green) |
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
- Vehicles are colored by heading, so groups flying together share a color. The walls, floor grid
  and fog grow with the vehicle count. Defaults live in `GlobalState::default` in `src/main.rs`.

## How it works

```text
src/
├── main.rs          # app setup, settings, egui menu, camera input
├── vehicle.rs       # CPU steering and integration, tests, benchmarks
├── octree.rs        # exact radius queries for the CPU octree mode
├── gpu_flock.rs     # GPU buffers, compute pipelines, parameters
├── instancing.rs    # one instanced draw for every vehicle
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

Run from the repository root, one Cargo command at a time, with no app running:

```sh
CARGO_BUILD_JOBS=4 cargo fmt --all -- --check
CARGO_BUILD_JOBS=4 cargo test --release --locked
CARGO_BUILD_JOBS=4 cargo build --release --locked
python3 scripts/smoke-native.py   # opens the app for 10 s; needs a desktop session
git diff --check
```

The normal tests cover:
- steering safety and CPU modes;
- the camera and the target;
- instance data and GPU parameter encoding.

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
