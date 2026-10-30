# 3D Boids simulation

[demo](https://ac3d.skippy-ai.com/)

## Local native development

Use `rustup` and run commands from the repository root. `rust-toolchain.toml`
selects the tested Rust 1.99.1 toolchain, including formatting and lint tools.
On macOS, Xcode command-line tools are also required.

```sh
cargo build --release --locked
cargo run --release --locked
cargo fmt --all -- --check
cargo test --release --locked
python3 scripts/smoke-native.py
```

The smoke check opens the app for ten seconds and then closes it. It requires
a desktop session with graphics support; it checks startup, not flocking correctness
or performance. Rust regression tests cover camera input, target selection, vehicle cleanup,
the instance data sent to the renderer, separation that keeps followers apart, and numerical
safety of steering (finite values, magnitude speed limit, cohesion arrival).

Agent and planning conventions are in `AGENTS.md`, `PLANS.md` and `plans/STATUS.md`.

`cargo run` supplies the asset root. When running the binary directly, set it:

```sh
CARGO_MANIFEST_DIR="$PWD" ./target/release/autonomous-characters-3d
```

Native development uses Bevy 0.20.1 and bevy_egui 0.43.1. Camera control and
mesh picking use Bevy's built-in plugins; audio is disabled because the app does
not use it. The original native compatibility build is preserved on the
`codex/native-baseline` branch. The HTML report describes the original codebase;
web builds are outside this migration's scope.

Controls: right-drag to orbit, middle-drag to pan, and scroll to zoom. Left-click
the red target to select it; click the background to return to wandering. Move a
selected target with WASD, Space and left Shift. Camera and target controls defer
to the menu when it consumes input. The on-screen Controls panel lists these
shortcuts and shows Follow ON/OFF. The target turns green while selected and
returns to red when following is off.

Vehicles are colored by heading (hue from the horizontal direction, lighter when
climbing), so groups flying together share a color. The faint box marks the walls
that wandering vehicles steer back into, with a grid on its floor; both, and the
distance fog, grow with the vehicle count.

Separation pushes a vehicle away from each neighbor closer than the separation distance,
harder the closer it is, and the pushes add up; the separation factor scales them. A
crowd therefore pushes back instead of collapsing, so followers form a loose ball around
the target with visible gaps between vehicles. Alignment and cohesion factors still cap
their steering force.

For dependency diagnostics, install the separate CLI with
`cargo install --locked --version 0.20.2 cargo-deny`, then run
`cargo deny --locked --target aarch64-apple-darwin check advisories` on this Mac.
No advisory suppressions are configured. This command does not upgrade packages.
`samply` is an optional native CPU profiler; after installing version 0.13.1, run
`CARGO_MANIFEST_DIR="$PWD" samply record ./target/release/autonomous-characters-3d`.
For source line information, first rebuild with
`CARGO_PROFILE_RELEASE_DEBUG=1 cargo build --release --locked`.
Keep profiling separate from uninstrumented performance measurements.

## Sources

Some sources I used for inspiration and help building this.

### Boids

- http://www.red3d.com/cwr/boids/
- https://natureofcode.com/book/chapter-6-autonomous-agents/
- https://dl.acm.org/doi/10.1145/37401.37406

### Octree

- https://en.wikipedia.org/wiki/Octree
- https://magnum.graphics/showcase/octree/?spheres=20&sphere-radius=0.1&sphere-velocity=1.0
- https://castle-engine.io/vrml_engine_doc/output/xsl/html/section.how_octree_works.html

### Language and libraries used

- Rust 1.99.1 (tested local development toolchain)
- Bevy 0.20.1 (game engine)
- Bevy parallel queries (multithreading)
- Nalgebra 0.35.0 (linear algebra library)

### Implementation

- The Naive and the Octree implementation include multithreading
- Menu for changing the parameters of the simulation
- Octree stores append-only nodes and points in vectors, with indices linking them
- Octree mode answers exact radius queries, so it finds the same neighbors as the naive mode
- IO and async task pools get one thread each, so the simulation's parallel queries get the remaining cores
- The Octree is built in every call of the function "flock" - every time we update the speed of the boids
- All vehicles are drawn with one instanced draw call: one entity holds per-vehicle position and rotation, uploaded to a reused GPU buffer each frame (`src/instancing.rs`, `assets/shaders/vehicle_instancing.wesl`); the shader uses Bevy's PBR lighting and fog
- Steering, wander and target movement are scaled by frame time (tuned at a 60 Hz reference), so behavior does not depend on FPS

### Analysis

Headless simulation benchmark (steering and integration per 1/60 s step, no rendering;
seed `0xB01D5`; Apple M3 Pro, release build; median ms). Both modes get identical inputs
and compute the same flocking. The spawn cube and walls grow with the cube root of the
vehicle count, so density stays constant; octree mode is the default.

| Vehicles | Naive | Octree (default) |
| --- | --- | --- |
| 200 | 0.07 | 0.14 |
| 1,000 | 0.59 | 0.56 |
| 2,000 | 1.46 | 0.84 |
| 5,000 | 6.75 | 1.82 |
| 10,000 | 25.53 | 3.74 |

In the app (window, rendering included, no vsync), wandering flocks run at a median
of about 4.5 ms per frame with 10,000 vehicles and about 12.5 ms with 30,000.
Following the target at large counts is slower (about 30 ms per simulation step at
5,000 vehicles), because the whole flock gathers within the 30-unit alignment radius of
each vehicle, so every vehicle visits almost every other one (see `plans/STATUS.md`).

Run it with nothing else heavy running (timings roughly double on a busy machine):

```sh
cargo test --release --locked flocking_benchmark -- --ignored --nocapture --test-threads=1
```

Details and caveats are in `plans/completed/2026-10-30-seeded-headless-benchmarks.md` and
`plans/completed/2026-10-30-count-scaled-spawn-region.md`.

Historical chart from the original version (single-leaf octree queries that skipped
neighbors, unknown machine and seed; not comparable with the table above):

![meritve](meritve.png)

#### Possible improvements

- Updating of Octree (instead of rebuilding)
- Simulating on the GPU (compute shader with a uniform grid) for flocks of hundreds of thousands
