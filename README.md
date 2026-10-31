# 3D Boids simulation

[demo](https://ac3d.skippy-ai.com/)

## Local native development

Use `rustup` and run commands from the repository root. `rust-toolchain.toml`
selects the tested Rust 1.99.1 toolchain, including formatting and lint tools.
On macOS, Xcode command-line tools are also required. Run builds and tests
sequentially, with no other Cargo command or native app running.

```sh
CARGO_BUILD_JOBS=4 cargo build --release --locked
CARGO_BUILD_JOBS=4 cargo run --release --locked
CARGO_BUILD_JOBS=4 cargo fmt --all -- --check
CARGO_BUILD_JOBS=4 cargo test --release --locked
python3 scripts/smoke-native.py
```

Close the app before continuing with the checks. The smoke check opens the app
for ten seconds and then closes it. It requires a desktop session with graphics
support; it checks startup, not flocking correctness or performance. The normal
Rust tests cover camera input and framing, target selection and scaling, vehicle
cleanup, instance data, parameter encoding, and numerical safety of steering.
GPU tests and benchmarks are ignored by the normal test command and need a real
GPU; device creation may fail in a sandbox even on a machine with a GPU.

```sh
# GPU regression tests; exclude the long benchmark from the general filter.
CARGO_BUILD_JOBS=4 cargo test --release --locked gpu_ -- --ignored --nocapture --test-threads=1 --skip gpu_flocking_benchmark

# Headless GPU benchmark, run separately.
CARGO_BUILD_JOBS=4 cargo test --release --locked gpu_flocking_benchmark -- --ignored --nocapture --test-threads=1
```

The GPU tests check CPU parity with cap 0, complete separation with cap 1,
spawn/growth/truncation/reset, bounds, and retained headings. Benchmark cases are
configured with comma-separated environment variables: `GPU_BENCH_COUNTS`
(default `10000,30000,100000,300000,1000000`), `GPU_BENCH_CAPS` (default
`0,32,64,128`), and `GPU_BENCH_MODES` (default `wander,follow`). To compare the
selected default cap with unlimited steering:

```sh
GPU_BENCH_COUNTS=20000,100000 GPU_BENCH_CAPS=0,16 GPU_BENCH_MODES=wander,follow CARGO_BUILD_JOBS=4 cargo test --release --locked gpu_flocking_benchmark -- --ignored --nocapture --test-threads=1
```

Each case spawns on the GPU, warms up for 600 steps at 1/60 s, then times 60
steps including encode, submit and blocking device wait. It prints median/p95
milliseconds, requested/effective cap, nearest-distance statistics for 1,000
sampled boids against the whole flock, the percentage below 1.4 units (dart
width), mean speed, and follow-mode mean target distance. Readback and CPU
statistics are outside the timed interval. Any completed step over 250 ms skips
the rest of that case; skipped cases report their partial progress and are not
steady-state comparisons. Check `uptime` and repeat measurements at similar load.

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
returns to red when following is off. Its visible and pickable sphere scales
with the current world size; its position and 120-unit/s movement speed do not.

The menu's mode selector offers **GPU** (default), **Octree** and **Naive**.
Startup uses 20,000 vehicles. The count slider spans 0–1,000,000 in GPU mode and
0–50,000 in either CPU mode; switching to a CPU mode clamps the count to its range.
GPU mode's **follow neighbor samples** slider spans 0–512, defaults to 16, and
limits alignment/cohesion samples only while following. Zero means unlimited.
Separation always sees every close neighbor. Wander uses the full query even
when the follow cap is nonzero. Samples favor the current coarse cell; they are
not a nearest-k selection.

Switching between CPU and GPU discards the previous flock and respawns it in the
selected mode; switching between Naive and Octree keeps the CPU flock. Increasing
the GPU count adds new vehicles, lowering it truncates the existing flock, and
setting it to zero clears it so the next increase spawns a fresh flock. GPU state
is not read back into CPU entities during a mode switch.

Vehicles are colored by heading (hue from the horizontal direction, lighter when
climbing), so groups flying together share a color. The faint box marks the walls
that wandering vehicles steer back into, with a grid on its floor; both, and the
distance fog, grow with the vehicle count. The startup camera frames the default
world; use the camera controls after changing the count.

Separation pushes a vehicle away from each neighbor closer than the separation
distance, harder the closer it is, and the pushes add up. Its factor scales those
pushes; alignment and cohesion factors limit their steering forces. This keeps
moderate follow flocks apart, but very large balls still become crowded under
seek pressure (see the performance table and `plans/STATUS.md`).

For dependency diagnostics, install the separate CLI with
`CARGO_BUILD_JOBS=4 cargo install --locked --version 0.20.2 cargo-deny`, then run
`cargo deny --locked --target aarch64-apple-darwin check advisories` on this Mac.
No advisory suppressions are configured. This command does not upgrade packages.
`samply` is an optional native CPU profiler; after installing version 0.13.1, run
`CARGO_MANIFEST_DIR="$PWD" samply record ./target/release/autonomous-characters-3d`.
For source line information, first rebuild with
`CARGO_BUILD_JOBS=4 CARGO_PROFILE_RELEASE_DEBUG=1 cargo build --release --locked`.
Keep profiling separate from uninstrumented performance measurements.

## Implementation

- `GlobalState` defaults live in its `Default` implementation in `src/main.rs`.
- Naive and Octree use parallel CPU queries. The octree stores append-only nodes
  and points linked by indices, is rebuilt in each call to `flock`, and answers
  exact radius queries matching the naive reference. IO and async task pools get
  one thread each, leaving the remaining cores for simulation queries.
- GPU mode keeps vehicles entirely in GPU buffers (`src/gpu_flock.rs`,
  `assets/shaders/gpu_flock.wgsl`). Compute passes build a hashed uniform grid by
  counting, prefix sum and scatter, then steer and integrate the flock. Capped
  follow mode first computes complete separation in a finer grid, then samples
  alignment/cohesion in the coarse grid. Unlimited mode and wander use one grid.
- All vehicles use one instanced draw call (`src/instancing.rs`,
  `assets/shaders/vehicle_instancing.wesl`), with Bevy's PBR lighting and fog.
  CPU modes upload position/rotation each frame; GPU compute writes the instance
  buffer directly, with no simulation readback during normal rendering.
- Steering, wander and target movement scale with frame time, tuned at a 60 Hz
  reference. The existing wall-avoidance term also applies inside the walls and
  acts as drag: measured wander speed settles near 29 rather than the configured
  40. Changing that behavior is a joint CPU/GPU follow-up.

## Performance

Measurements below were supplied by the orchestrator on an Apple M3 Pro in
release builds. GPU step timings are headless **encode + submit + wait**, not
rendered frame times: fresh PCG spawn at frame 0, 600 warmup steps (10 simulated
seconds at 1/60 s), then 60 timed steps. Median milliseconds, load average about 3:

| Vehicles | Wander | Follow, cap 16 | Follow, unlimited | Follow sample below 1.4 units |
| --- | --- | --- | --- | --- |
| 20,000 | 0.67 | 1.4 | 16.9 | 2.2% |
| 100,000† | 2.5 | 6.4 | 220 | 19% |
| 300,000 | 7.4 | 13.2 | Skipped: step >250 ms | 42% |
| 1,000,000 | 26.1 | 44.9 | Skipped: step >250 ms | 15% |

†100,000 comes from the earlier sweep at load average 1.4–4. Other rows are the
repeat after removing the unnecessary wander fine-grid pass. Requested caps 0
and 16 now use identical wander work, with comparable timings and 0% sampled
nearest distances below 1.4 units. The non-monotonic follow crowding percentages
are snapshots after the fixed warmup, not a claim that the largest flock is less
dense once fully gathered. Detailed evidence is in
[the GPU plan](plans/completed/2026-10-30-gpu-flocking.md).

App observations include rendering, vsync enabled on a 120 Hz display, load
average about 3. At the default GPU/20,000 start, the FPS counter reported about
107–120 wandering and about 127 following. Camera distance matters: 100,000
wandering with close views of the flocks reached about 59 FPS; 300,000 viewed
from far away reached about 92 FPS. Before the cap, 30,000 followers ran at about
31 FPS. Close-up dart coverage suggests fill-rate/rendering cost can exceed the
simulation cost; fewer triangles, distance impostors, MSAA cost and a depth
pre-pass are future investigations, not measured improvements.

### CPU reference

Earlier headless CPU benchmark (steering and integration per 1/60 s step, no
rendering; seed `0xB01D5`; Apple M3 Pro, release build; median ms). Both CPU modes
receive identical inputs and compute the same flocking. The spawn cube and walls
grow with the cube root of the vehicle count, so density stays constant.

| Vehicles | Naive | Octree |
| --- | --- | --- |
| 200 | 0.07 | 0.14 |
| 1,000 | 0.59 | 0.56 |
| 2,000 | 1.46 | 0.84 |
| 5,000 | 6.75 | 1.82 |
| 10,000 | 25.53 | 3.74 |

Earlier Octree app measurements (window/rendering included, no vsync) were about
4.5 ms/frame with 10,000 wandering vehicles and 12.5 ms with 30,000. CPU following
at large counts remains quadratic (about 30 ms/simulation step at 5,000), because
the flock gathers within the 30-unit alignment radius of almost every vehicle.
These CPU results are reference measurements from before the GPU implementation.

Run the CPU benchmark alone with nothing else heavy running (timings roughly
double on a busy machine). The exact filter avoids also running the GPU benchmark:

```sh
CARGO_BUILD_JOBS=4 cargo test --release --locked vehicle::tests::flocking_benchmark -- --exact --ignored --nocapture --test-threads=1
```

Details and caveats are in `plans/completed/2026-10-30-seeded-headless-benchmarks.md`
and `plans/completed/2026-10-30-count-scaled-spawn-region.md`.

Historical chart from the original version (single-leaf octree queries that
skipped neighbors, unknown machine and seed; not comparable with the tables above):

![meritve](meritve.png)

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
