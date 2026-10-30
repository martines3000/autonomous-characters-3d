# Agent guide

Native Rust/Bevy 3D boids simulation. Bevy 0.20.1, bevy_egui 0.43.1, nalgebra 0.35, Rust 1.99.1 (pinned in `rust-toolchain.toml`). Source lives in `src/`: `main.rs` (app setup, `GlobalState` defaults, egui menu, camera input), `vehicle.rs` (spawning, steering, integration), `target.rs` (selectable target), `octree.rs` (spatial index).

## Commands from the repository root

Run these sequentially, never in parallel, and always with `CARGO_BUILD_JOBS=4` or fewer:

```sh
CARGO_BUILD_JOBS=4 cargo fmt --all -- --check
CARGO_BUILD_JOBS=4 cargo test --release --locked
CARGO_BUILD_JOBS=4 cargo build --release --locked
python3 scripts/smoke-native.py
git diff --check
```

`cargo test` is the behavior gate. The smoke script opens the release binary for ten seconds and needs a desktop session; it proves startup only, not flocking correctness or performance. Clippy (`CARGO_BUILD_JOBS=4 cargo clippy --release --locked --all-targets`) has 8 known pre-existing warnings; it is not a gate, but do not add new warnings in code you touch. The release build has no compiler warnings; keep it that way.

Performance: `CARGO_BUILD_JOBS=4 cargo test --release --locked flocking_benchmark -- --ignored --nocapture --test-threads=1` runs the seeded headless benchmark (about 10 s). Run it only when no other build, test, benchmark or app is running, check `uptime` first (a busy machine roughly doubles timings), and repeat at least once before recording numbers.

`cargo run --release --locked` supplies the asset root; a direct binary launch needs `CARGO_MANIFEST_DIR="$PWD"`. See `README.md` for controls, `cargo-deny` and `samply` usage.

## Resource coordination

This machine is shared by several agents and worktrees.

- One owner runs verification for all agents and worktrees. Workers report what they changed and which focused checks they ran; the owner runs the final gate once.
- Never overlap Cargo builds, tests, checks, Clippy, benchmarks or profiling, in this checkout or any other. Before starting one, check that no `cargo`/`rustc` process is running.
- Use `CARGO_BUILD_JOBS=4` or fewer.
- Run only one native app instance at a time (`cargo run`, the smoke script, `samply record` and app bundles all count). Close it before starting another.
- Disposable probes (scratch crates, copied sources, experiments) use their own `CARGO_TARGET_DIR` outside `target/`. A probe that builds into the shared `target/` can overwrite the real binary at the same output path.

## Scope boundaries

- Native development only. `index.html`, `web/`, `Trunk.toml`, `Dockerfile` and the WASM target are legacy and out of scope unless a task says otherwise.
- `review-report.html` and `review-tools-research.md` describe the original pre-modernization code (commit `e7a53d7`). Keep them as historical records; do not edit them. `review-progress.md` is the historical log up to the native modernization; record new work in `plans/`.
- Change `Cargo.toml`/`Cargo.lock` only in tasks that are about dependencies.
- Do not change simulation semantics (frame-rate dependence, octree query behavior, default settings) as a side effect; each has its own entry in `plans/STATUS.md`.
- Keep `README.md` in sync when commands, controls or behavior change.
- Commit headers use `feat`, `fix`, `docs`, `test`, `refactor`, `perf`, `build` or `chore`.

## Planning

Read `PLANS.md` before multi-step work. `plans/STATUS.md` lists current work, prioritized future tasks and the status of the original review findings. Open ExecPlans live in `plans/planned/`, closed ones in `plans/completed/`; start new ones from `plans/TEMPLATE.md`. Small, low-risk fixes need no ExecPlan.

Reference material: `docs/native-migration-notes.md` (Bevy 0.20 API research), `review-report.html` (original findings F01–F14 and tooling recommendations).
