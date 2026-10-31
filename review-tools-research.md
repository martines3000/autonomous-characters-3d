# Tool and crate research for the technical assessment

Research checked **2026-10-30**, against the reviewed Bevy 0.9.1 application at `e7a53d7`. This is a recommendation record, not an implementation or a measured performance result. Application source, manifests, toolchain and lockfile were not changed. Registry responses, published crate manifests and repository metadata are retained under `/tmp/ac3d-review-e7a53d7/tools/` while that directory exists.

## Embark: a short, practical shortlist

**Prioritize `cargo-deny` for dependency triage and Puffin for finding where frame time goes. Add `cargo-about` when packaging a release.** These tools address different evidence gaps; they do not repair steering arithmetic or incomplete neighbor queries. The selection below is an engineering recommendation based on F01/F02/F04/F09/F14, rather than an endorsement of Embark's entire catalog. The [Embark organization](https://github.com/EmbarkStudios) identifies Puffin, cargo-deny and cargo-about as its projects.

| Candidate | Concrete use here | Adoption decision |
| --- | --- | --- |
| **Puffin + `puffin_http` + separate `puffin_viewer`** | CPU scopes around snapshot collection, tree rebuild, neighbor queries, steering and integration; investigate F04 clustered-tree spikes and F09 misleading timings. | Useful after a reproducible native baseline. Prefer a separate viewer to avoid coupling profiling to the application's old egui integration. |
| **`cargo-deny`** | Repeatable advisory, unmaintained-crate, duplicate-version, allowed-source and license checks; turn F14's one-time screen into a maintained record. | High value as a separate modern-toolchain CLI job. Triage the existing baseline before deciding which results should fail CI. |
| **`cargo-about`** | Generate a dependency-license HTML artifact for native distribution. | Optional release task; does not measure runtime progress. |
| **`puffin_egui` embedded UI** | In-app CPU flamegraph. | Defer until UI/toolchain compatibility is settled; current release cannot use the application's egui context. |
| **`cargo-fetcher`** | Accelerates dependency transfer in specialized clean/cloud CI. | Reject for this project now: archived, with no demonstrated dependency-download bottleneck. Ordinary Cargo fetching and a cache are sufficient starting points. |

Puffin's documented scopes collect instrumented CPU durations; its TCP transport supports an external viewer. Enable capture explicitly and mark one frame boundary per rendered frame. This is a diagnostic view, not a statistically controlled benchmark or GPU profiler. Use coarse phase scopes first; instrumenting every neighbor comparison can distort the workload. The proposed scope placement and coarse-first policy are recommendations for this codebase. [Puffin usage and transport](https://github.com/EmbarkStudios/puffin/blob/main/README.md), [Puffin published package](https://crates.io/crates/puffin/0.20.0).

## Version and compatibility evidence

Latest non-yanked stable versions returned by crates.io on the research date:

| Published tool/crate | Latest stable | Declared minimum Rust | Compatibility implication |
| --- | --- | --- | --- |
| `puffin` | 0.20.0 | 1.92.0 | Cannot be compiled by the project's declared Rust 1.67.1 baseline. |
| `puffin_http` | 0.17.0 | 1.92.0 | Same limitation for native app-side transport. |
| `puffin_egui` | 0.30.0 | 1.92.0 | Published dependency is egui `^0.33.3`, versus locked egui 0.20.1 here. |
| `puffin_viewer` | 0.23.0 | 1.92.0 | Can be installed independently using a suitable modern toolchain; app-side capture still needs a compatible build. |
| `cargo-deny` | 0.20.2 | 1.88.0 | Install/run independently; no runtime dependency or Bevy upgrade required. |
| `cargo-about` | 0.9.3 | 1.88.0 | Same CLI separation; published CLI requires the `cli` feature when installing. |

Sources are the first-party registry records: [puffin](https://crates.io/api/v1/crates/puffin/0.20.0), [puffin_http](https://crates.io/api/v1/crates/puffin_http/0.17.0), [puffin_egui](https://crates.io/api/v1/crates/puffin_egui/0.30.0), [puffin_viewer](https://crates.io/api/v1/crates/puffin_viewer/0.23.0), [cargo-deny](https://crates.io/api/v1/crates/cargo-deny/0.20.2), [cargo-about](https://crates.io/api/v1/crates/cargo-about/0.9.3), and [published puffin_egui dependencies](https://crates.io/api/v1/crates/puffin_egui/0.30.0/dependencies). Published `.crate` manifests were also downloaded and inspected. `cargo-about`'s README badge still says Rust 1.81; the published manifest/registry declares **1.88**, which is the value used here.

Do not copy versions or requirements from Puffin's development branch: its [workspace manifest](https://github.com/EmbarkStudios/puffin/blob/main/Cargo.toml) had already advanced to Rust 1.95 and egui 0.37 when checked. Published 0.20.0/0.30.0 packages have the requirements in the table. A second egui version can coexist in Cargo's dependency graph, but its `Context` is a different type; it does not make an embedded profiler compatible with `bevy_egui` 0.18.

If preserving the historical stack is essential, **`puffin_egui` 0.19.2** declares egui `^0.20.1` and puffin `^0.14.2`; **`puffin_http` 0.11.1** also declares puffin `^0.14.2`. Puffin 0.14.3 is a matching historical core candidate. Those published packages do **not declare an MSRV** in the inspected manifests/registry, so this is a dependency-shape match, not a verified Rust 1.67.1 build. Exact direct pins, compatible transitive resolution, target builds and capture/viewer format interoperability still need validation. Do not assume a current viewer can read every historical transport/file version. [Historical egui package](https://crates.io/api/v1/crates/puffin_egui/0.19.2), [historical transport](https://crates.io/api/v1/crates/puffin_http/0.11.1), [historical core](https://crates.io/api/v1/crates/puffin/0.14.3).

**Maintenance check:** GitHub's repository API reported Puffin, cargo-deny and cargo-about as not archived, each pushed on 2026-10-26. That supports considering them maintained candidates; a push date does not prove correctness or support for this old project. cargo-about 0.9.3 was published that same day. [Puffin metadata](https://api.github.com/repos/EmbarkStudios/puffin), [cargo-deny metadata](https://api.github.com/repos/EmbarkStudios/cargo-deny), [cargo-about metadata](https://api.github.com/repos/EmbarkStudios/cargo-about).

## How to measure useful progress with them

- **F01/F02 correctness remains the gate:** count non-finite state and compare sorted neighbor IDs with an exhaustive radius oracle before recording throughput. A faster incomplete query is not an improvement. Keep direct regressions for zero, coincident, symmetric and split-plane cases; no profiler establishes those invariants.
- **F04/F09 performance:** hold seed, initial positions, interaction radii, fixed step, boid count and tree policy constant. Save separate rebuild/query/steering/integration durations and distributions for uniform, tight-cluster and coincident inputs. Record tree nodes/depth, visited candidates and exact accepted neighbors. Puffin explains slow phases; repeated release benchmarks with capture disabled decide whether a change helped. Save hardware, OS, target, compiler, commit and feature set with each result.
- **F14 dependency progress:** archive cargo-deny's machine-readable output, database date and target/feature selection. Track unresolved security, unsoundness and maintenance notices separately, with a reason and review date for each exception. Compare against the reviewed OSV screen with awareness that RustSec and OSV coverage/classification can differ. Lowering a total by suppressing notices is not remediation.

The above metrics and acceptance gates are recommendations, not measurements already performed.

## Illustrative commands, not executed

Run these only after selecting a suitable installed modern Rust toolchain. `+stable` explicitly avoids the repository's dated rustup pin; the toolchain must satisfy the versions above. Install tooling separately from application dependencies. Initialization writes new policy/template files and is shown as future setup.

```sh
cargo +stable install --locked --version 0.20.2 cargo-deny
cargo +stable deny init
cargo +stable deny --locked --format json check advisories
# After selecting a project-specific license/source/duplicate policy:
cargo +stable deny --locked check
```

`--locked` rejects dependency re-resolution instead of silently changing Cargo.lock. cargo-deny supports target filters and an already-generated metadata JSON file if the old project makes metadata resolution problematic. Check native graph selections deliberately; the tool does not prove advisory reachability. [Install/check usage](https://github.com/EmbarkStudios/cargo-deny), [common flags](https://embarkstudios.github.io/cargo-deny/cli/common.html), [advisory configuration](https://embarkstudios.github.io/cargo-deny/checks/advisories/cfg.html).

```sh
cargo +stable install --locked --version 0.9.3 --features cli cargo-about
cargo +stable about init
cargo +stable about generate --locked -o licenses.html about.hbs
```

Review generated license text and handle assets/fonts separately from Rust dependencies. [cargo-about installation](https://github.com/EmbarkStudios/cargo-about#installation), [generate flags](https://embarkstudios.github.io/cargo-about/cli/generate/index.html).

## Deliberate exclusions

- **`cargo-fetcher`:** the [official repository](https://github.com/EmbarkStudios/cargo-fetcher) was archived on 2026-05-21. Its own README targets clean CI and warns against typical developer environments because it bypasses Cargo safety mechanisms such as file locking. This is a poor default for this small demo.
- **`profiling`:** a useful interchangeable instrumentation layer, linked by Puffin, but **owned by aclysma, not Embark**. Add it only if selecting between multiple profiler backends or integrating an existing profiling scheme justifies the abstraction; direct Puffin scopes are smaller for one backend. [Official repository](https://github.com/aclysma/profiling).
- **Quilkin, crash-handling, texture-synthesis and graphics abstractions:** no observed network-server, production crash-dump, procedural-texture or renderer-replacement requirement in the findings. Introducing those systems would not resolve the current numerical, spatial-query or benchmarking defects.

No listed tool was installed or integrated, and no benchmark/profiler experiment was run as part of this research. Compatibility conclusions are based on published metadata and source inspection, with unresolved historical build/transport checks called out above.

## Other tools: recommended measurement stack

These recommendations are tied to this repository, rather than a general Rust tool catalog. Versions were verified from the [official crates.io API](https://crates.io/data-access) on 2026-10-30; raw results are in `tools/alternatives-versions.json` in the temporary evidence directory.

1. **Start with Bevy 0.9.1's built-in trace export and Perfetto.** The engine already supports `bevy/trace_chrome` and `bevy/trace_tracy`; no new profiler abstraction is needed to see systems and render stages. Begin with Chrome-format JSON/Perfetto, then add coarse application spans around snapshot, tree build and steering. Optional tracing dependencies are not in the existing lock, so enabling the feature can require a deliberate profiling-branch lock resolution; a `--locked` attempt must fail rather than silently update it. Tracy requires a client-compatible viewer; do not pair the old tracing client with the newest viewer blindly. These paths are native diagnostics. [Exact Bevy 0.9.1 guide](https://github.com/bevyengine/bevy/blob/v0.9.1/docs/profiling.md), [Bevy 0.9.1 logging manifest](https://github.com/bevyengine/bevy/blob/v0.9.1/crates/bevy_log/Cargo.toml), [Perfetto](https://ui.perfetto.dev).
2. **Use Criterion for reproducible kernel comparisons.** Latest 0.8.2 declares Rust 1.86, above the app's 1.67.1. Use a separately locked modern headless harness or validate a historical crate graph; do not assume a current dev dependency works on the old compiler. Benchmark rebuild + radius query + forces together and separately, on fixed seeded snapshots, after neighbor correctness passes. Named baselines help compare later changes. The maintained upstream is now criterion-rs, not the original bheisler repo. Divan 0.1.21 (Rust 1.80) is a simpler alternative; choose one initially. [Criterion registry](https://crates.io/api/v1/crates/criterion/0.8.2), [maintained repo](https://github.com/criterion-rs/criterion.rs), [baseline flags](https://github.com/criterion-rs/criterion.rs/blob/master/book/src/user_guide/command_line_options.md), [Divan](https://github.com/nvzqz/divan).
3. **Use samply for uninstrumented native CPU stacks.** Version 0.13.1 declares Rust 1.77 for building the CLI; the separately installed profiler can record an older application binary. The official project supports macOS/Linux/Windows. Build release code with line information and record the built executable, rather than including Cargo compilation in the runtime profile. This answers whether time goes into insertion, queries, transforms, rendering preparation or synchronization; it does not supply GPU timing. [Registry](https://crates.io/api/v1/crates/samply/0.13.1), [usage/platforms](https://github.com/mstange/samply).
4. **Use Apple Instruments/Metal capture for platform-specific questions.** This is an ARM Mac: Metal system trace and Xcode GPU capture can distinguish CPU work, GPU work and frame presentation after the native baseline runs. [Apple tools](https://developer.apple.com/metal/tools/), [debug an external executable](https://developer.apple.com/documentation/xcode/capturing-a-metal-workload-in-xcode).
5. **Add generated correctness cases before chasing throughput.** proptest 1.12.0 requires Rust 1.88. It generates cases and shrinks failures; use finite point clouds and deliberate boundary/coincident/symmetric cases against a brute-force oracle. A seeded rand 0.8 snapshot generator and normal Rust tests are already sufficient for the first regressions. Keep old-toolchain tests separate from modern harness tooling. [Registry](https://crates.io/api/v1/crates/proptest/1.12.0), [Proptest book](https://proptest-rs.github.io/proptest/intro.html).
6. **Inspect the schedule with the matching plugin generation.** `bevy_mod_debugdump` latest 0.17.0 targets Bevy 0.20; version 0.6.0 declares Bevy ^0.9. A matching schedule graph can help review F07 but does not prove the force producer runs before integration unless constraints/tests establish it. MSRV is not declared; historical transitive builds remain unverified. [Support table](https://github.com/jakobhellermann/bevy_mod_debugdump), [0.6 dependency evidence](https://crates.io/api/v1/crates/bevy_mod_debugdump/0.6.0/dependencies).
7. **Measure allocation churn only when CPU traces point there.** dhat 0.3.3 supports a replacement global allocator, allocation profiles and allocation-count tests. Its docs explicitly call it experimental and lower-maintenance-priority; registry declares no MSRV. Prefer a short isolated headless experiment and do not ship it enabled. Capture separate allocations/tick and peak bytes, not instrumented FPS. [Registry](https://crates.io/api/v1/crates/dhat/0.3.3), [limitations and setup](https://docs.rs/dhat/0.3.3/dhat/).

## Optional algorithm/storage experiments

- **Kiddo 6.3.0 (Rust 1.89)** provides 3D radius-search candidates. Benchmark complete-radius results and total rebuild-plus-query cost on the same boid distributions; nearest-k or approximate queries change semantics. It is not a measured improvement over a corrected octree. A simple uniform spatial grid is another candidate to compare before adding a dependency. [Registry](https://crates.io/api/v1/crates/kiddo/6.3.0), [official README/query options](https://github.com/sdd/kiddo).
- **slotmap 1.1.1 (Rust 1.58)** is a maintained arena candidate if generational deletion is needed. This implementation rebuilds and never removes nodes, so plain Vec indices remain the smaller first option. Declared compiler minimum is not a tested transitive build guarantee. [Registry](https://crates.io/api/v1/crates/slotmap/1.1.1), [crate documentation](https://docs.rs/slotmap/1.1.1/slotmap/).
- **Gungraun 0.20.0 (Rust 1.88)** is the successor to Iai-Callgrind, useful later for instruction/cache metrics on supported Linux CI. Counts and modeled cycles are not real elapsed time or GPU/parallel-scaling measurements. This ARM Mac is not the initial Valgrind testbed. [Registry](https://crates.io/api/v1/crates/gungraun/0.20.0), [official project](https://github.com/gungraun/gungraun), [Valgrind platforms](https://valgrind.org/info/platforms.html).
- **Bencher/CodSpeed** can retain results and flag regressions once stable benchmarks exist. Start with versioned JSON/CSV and CI artifacts; hosted service setup is optional. Bencher documents thresholds per branch/testbed/measure; CodSpeed documents Rust harness integration. Accounts, current plans, runner compatibility and any upload policy need an implementation-time decision. Neither service was connected. [Bencher thresholds](https://bencher.dev/docs/explanation/thresholds/), [CodSpeed Rust](https://codspeed.io/docs/benchmarks/rust).
- **Defer frame pacing and extra executors.** `bevy_framepace` 0.22.0 targets Bevy ^0.19, and intentionally waiting to pace rendering cannot fix acceleration units in F03 or all-pairs complexity in F10. Keep timing/presentation settings fixed when measuring throughput. The app already uses Bevy parallel queries; adding Rayon work does not by itself improve scaling. [Published framepace dependency](https://crates.io/api/v1/crates/bevy_framepace/0.22.0/dependencies), [design and support table](https://github.com/aevyrie/bevy_framepace).

## Proposed progress contract (not observed results)

Require zero neighbor-set mismatches against the oracle, zero non-finite simulation values on regression cases, a magnitude speed bound and fixed-step replay agreement before comparing performance. Record stage milliseconds, frame median/p95, candidate checks, tree depth/nodes, allocations/tick, peak bytes and visible-scene GPU time separately. Tag results with commit, seed, world dimensions, radii, capacity, population, compiler/target/features, threads, viewport/presentation and hardware. Use identical corrected behavior for the baseline and candidate.

Begin with repeated local runs and CI artifacts. Establish variance before selecting thresholds; an example 5% warning/10% investigation policy is a proposed team choice, not a measured confidence bound. Keep correctness hard gates separate from noisy wall-clock warnings. No profiler, benchmark framework, replacement index or tool installation was performed for this extension.
