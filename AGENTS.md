# Agent guide

Rust/Bevy 3D boids simulation with native development and optional WebGPU browser delivery. Bevy 0.20.1, bevy_egui 0.43.1, nalgebra 0.35, Rust 1.99.1 (pinned in `rust-toolchain.toml`). Source lives in `src/`: `main.rs` (app setup, `GlobalState` defaults, egui menu, camera input), `vehicle.rs` (spawning, steering, integration), `target.rs` (selectable target), `octree.rs` (spatial index).

## Commands from the repository root

For a fresh clone on another machine, follow [new-machine setup](docs/new-machine-setup.md) before verification.

Run these sequentially, never in parallel, and always with `CARGO_BUILD_JOBS=4` or fewer:

```sh
CARGO_BUILD_JOBS=4 cargo fmt --all -- --check
CARGO_BUILD_JOBS=4 cargo test --release --locked
CARGO_BUILD_JOBS=4 cargo build --release --locked
python3 scripts/smoke-native.py
git diff --check
```

Browser/runtime/tooling changes also need the documented local container and headed-browser checks in `docs/web-delivery.md`. Use the recorded Chromium configuration and serialize builds/apps with the same shared lock. Headless CDP black frames do not establish visual success.

`cargo test` is the behavior gate. The smoke script opens the release binary for ten seconds and needs a desktop session; it proves startup only, not flocking correctness or performance. Clippy (`CARGO_BUILD_JOBS=4 cargo clippy --release --locked --all-targets`) has 7 known pre-existing warnings; it is not a gate, but do not add new warnings in code you touch. The release build has no compiler warnings; keep it that way.

Performance: `CARGO_BUILD_JOBS=4 cargo test --release --locked vehicle::tests::flocking_benchmark -- --exact --ignored --nocapture --test-threads=1` runs the seeded headless CPU benchmark (about 10 s); the GPU tests and benchmark are in `docs/reference.md`. Run it only when no other build, test, benchmark or app is running, check `uptime` first (a busy machine roughly doubles timings), and repeat at least once before recording numbers.

`cargo run --release --locked` supplies the asset root; a direct binary launch needs `CARGO_MANIFEST_DIR="$PWD"`. See `docs/reference.md` for controls, `cargo-deny` and `samply` usage.

## Resource coordination

This machine is shared by several agents and worktrees.

- One owner runs verification for all agents and worktrees. Workers report what they changed and which focused checks they ran; the owner runs the final gate once.
- Never overlap Cargo builds, tests, checks, Clippy, benchmarks or profiling, in this checkout or any other. Before starting one, check that no `cargo`/`rustc` process is running.
- Use `CARGO_BUILD_JOBS=4` or fewer.
- Run only one native app instance at a time (`cargo run`, the smoke script, `samply record` and app bundles all count). Close it before starting another.
- Disposable probes (scratch crates, copied sources, experiments) use their own `CARGO_TARGET_DIR` outside `target/`. A probe that builds into the shared `target/` can overwrite the real binary at the same output path.
- A shared target can also consider another worktree's newer test binary fresh. When changing checkout roots, force this crate to rebuild (`touch src/main.rs` before Cargo), confirm the compiler names the expected worktree, and check the expected test list or scenario metadata. A cached `Finished` line alone does not prove which source ran.

## Scope boundaries

- Native and WASM/WebGPU delivery share the Rust simulation. Follow `docs/web-delivery.md` for the supported browser build, static container and verification path; do not introduce an alternative web implementation as a side effect.
- Preserve the university-project artifacts (`analysis_slovene.md` and `meritve.png`). Keep current documentation focused on usage, maintenance and known limits; keep private infrastructure details and temporary agent logs outside this public repository.
- Change `Cargo.toml`/`Cargo.lock` only in tasks that are about dependencies.
- Do not change simulation semantics (frame-rate dependence, octree query behavior, default settings) as a side effect; each has its own entry in `plans/STATUS.md`.
- Keep `README.md` in sync when commands, controls or behavior change.
- Commit headers use `feat`, `fix`, `docs`, `test`, `refactor`, `perf`, `build` or `chore`.

## Planning

Read `PLANS.md` before multi-step work. `plans/STATUS.md` lists current work, prioritized future tasks and known limits. Open ExecPlans live in `plans/planned/`, closed ones in `plans/completed/`; start new ones from `plans/TEMPLATE.md`. Small, low-risk fixes need no ExecPlan.

Workflow security and publication boundaries are documented in `docs/web-delivery.md`.
