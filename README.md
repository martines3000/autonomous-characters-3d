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
and numerical safety of steering (finite values, magnitude speed limit, cohesion arrival).

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
- The Octree is built in every call of the function "flock" - every time we update the speed of the boids
- Steering, wander and target movement are scaled by frame time (tuned at a 60 Hz reference), so behavior does not depend on FPS

### Analysis

- Results of benchmarks

![meritve](meritve.png)

#### Possible improvements

- Updating of Octree (instead of rebuilding)
- Faster rendering speed (rendering has issues at higher number of boids)
