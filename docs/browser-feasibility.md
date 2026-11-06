# Browser feasibility and measured support

The declared tested configuration is desktop Linux/Wayland Chromium on the RX 7900 XT with the
explicit experimental WebGPU/Vulkan flags below. It runs the shared simulation, animated animals,
three public scenes and visitor controls. Other browsers and devices remain untested. A successful
compile or an application resource counter alone is not proof of a presented scene.

## Final committed-source acceptance, 2026-11-06 local time

- Runtime/build source: `9bbe1387638448110bccd29565354b09813aa262`, clean tree at packaging.
  Test harness: `6a346f6` adds the final Lab keyboard-focus assertion without changing runtime code.
- Packaged release: `a86835f2931b331568586ec7`, profile `web-release`, Rust **1.99.1**,
  wasm-bindgen **0.2.130**. WASM SHA-256:
  `6e741a40b86214ae13c5495abc266707208adf3c273cd2c554bcffa13744aa1f`.
- Same machine, browser binary, flags and adapter as the early proof below. The actual container
  serves gzip from localhost. No competing Cargo/native app/benchmark/video encoder ran during
  performance windows. Initial load average was **0.77 / 1.17 / 1.02**.
- Real headed screenshots show the visitor HUD, warm starling scene, reef, and three visible shark
  silhouettes among split schools. A 20-second browser Shark hunt recording is separate from
  performance measurements. CDP video contains native-window padding; use the full-resolution PNGs
  for composition and the native hunt clip for polished behavior review. The browser recording
  proves moving presented pixels. Visibility of every shark throughout a clip is not promised.

| Visitor/capability check | Observed result |
| --- | --- |
| Three scenes × Low/Medium/High; rapid repeated switching | Pass; increasing scene-specific counts and latest scene wins |
| Pause/resume, H, Help, Lab entry/return | Pass with real keys and pointer input |
| Lab numeric editor consumes P/H while focused | Pass; pause and HUD visibility remain unchanged |
| Right/middle drag and scroll over HUD; mixed-button chords | Pass; paused world crop remains byte-identical |
| World drag crosses HUD; adding/releasing held buttons | Pass; original world gesture continues |
| Ride, Scenic, reset view, reduced camera motion | Pass; HUD gestures preserve selected camera mode |
| 900×900 / 600×800 resize; real display factor 2 | Pass; high-DPI canvas 1920×1080 at CSS 960×540 |
| No WebGPU / no adapter / insufficient storage limits | Clear unsupported screen and retry action |
| Missing WASM, catalog, shader, font or environment | Visible error and retry action |
| Real background tab and foreground return | Hidden clock **0.6333 → 0.6333 s**; previous running state resumes |
| Actual disposable-browser GPU process crash | Renderer error stops scene; reload restores animated 20,000-bird scene |
| Normal visitor flow | No page, HTTP, failed-request or console errors |

The real visibility/cache/device-loss context uses a disposable persistent profile, no request
routing, and Playwright `connectOverCDP(..., { noDefaults: true })`. Ordinary Playwright contexts
force focus; they do not establish real document visibility. The device-loss check invokes
`Browser.crashGpuProcess` on that disposable browser only, exercising Bevy's actual render-error
handler. It does not destroy an unrelated JavaScript GPU device.

### Repeated public population measurements

Each row has two separate windows after restarting that scene/tier: **3 s warmup + 6 s sample**.
Resolution **1920×1080**, device scale **1**, reduced camera motion, public default effects,
4× MSAA, volumetrics and motion blur off. Shark rows have **three additional hunters**. These are
Bevy frame intervals, including browser/display scheduling, not GPU timestamps. Interactive GPU
scatter is not a seeded replay. Near-60-Hz results establish this reference configuration; they do
not establish unused GPU capacity or performance on another device.

| Scene | Tier / animals | Mean ms, runs 1 / 2 | p95 ms, runs 1 / 2 | Max ms, runs 1 / 2 | Intervals, runs 1 / 2 |
| --- | --- | ---: | ---: | ---: | ---: |
| murmuration | low / 8,000 | 16.667 / 16.667 | 17.1 / 17.1 | 18.4 / 19.3 | 360 / 360 |
| murmuration | medium / 20,000 | 16.666 / 16.666 | 17.0 / 17.0 | 17.8 / 18.1 | 360 / 360 |
| murmuration | high / 40,000 | 16.667 / 16.667 | 17.0 / 17.0 | 21.9 / 22.5 | 360 / 361 |
| reef | low / 500 | 16.666 / 16.666 | 17.1 / 17.0 | 20.3 / 21.2 | 360 / 360 |
| reef | medium / 1,500 | 16.667 / 16.667 | 17.0 / 17.0 | 18.0 / 17.7 | 360 / 360 |
| reef | high / 3,000 | 16.666 / 16.669 | 17.0 / 17.1 | 19.3 / 17.9 | 361 / 360 |
| shark | low / 5,000 | 16.667 / 16.666 | 17.0 / 17.0 | 22.2 / 17.7 | 361 / 360 |
| shark | medium / 12,000 | 16.666 / 16.667 | 16.9 / 17.0 | 18.3 / 17.9 | 361 / 360 |
| shark | high / 20,000 | 16.666 / 16.666 | 17.0 / 16.9 | 17.7 / 18.0 | 360 / 360 |

### Loading and artifact reproduction

An unrouted persistent browser on localhost reached application-ready in **1.732 s cold** and
**1.285 s warm**. Resource timing reported **29,780,394 bytes cold** and **0 bytes warm** transferred.
The fresh profile had no pre-existing cache. These timings do not model internet bandwidth or
latency. A same-context incognito reload in an earlier run re-fetched large files; it was not used
as proof of disk-cache reuse.

| Asset | Raw bytes | Gzip bytes |
| --- | ---: | ---: |
| WASM | 62,237,265 | 13,733,919 |
| Sunset lighting | 29,409,344 | 15,799,195 |
| Underwater lighting | 1,060,928 | 406,723 |

Two separate clean output builds produced **32 byte-identical files**, including metadata and gzip
variants. This proves the recorded source/toolchain's packaged artifact, using existing dependency
caches; it is not a cross-compiler reproducibility claim. Both native exporter and WASM builds were
warning-free. Final WASM Clippy reported the same **seven pre-existing warnings**, with none in the
browser changes. The shared pointer regression failed before its fix, then **21 focused camera
checks passed** (one GPU test intentionally ignored in that focused command). Native full/GPU gates
are recorded separately by the integration owner.

The local Linux/amd64 container passed `nginx -t`, health, UID/GID **101:101**, read-only root,
capability drop and only `/tmp` scratch. HTML/release metadata revalidate; versioned WASM, shader and
font URLs return gzip, correct MIME and immutable cache headers; a missing WASM returns 404. Local
image config: `sha256:739478e49f682963fb3a1e4981b2f88186040830e90409aa25f4aaad8e9cfb21`.
This is not a GHCR publication digest. Image publication and deployment were not performed.

Raw evidence is outside Git under `/home/martines/Projects/ac3d-evidence/2026-11-05/browser/`:
`final-matrix-7/smoke-report.json`, `population-benchmark/measurements.json`,
`behavior-video/browser-shark.webm`, `behavior-video/video-report.json`, `reproducibility.json`,
`container-verification-final.json`, `chord-fixed-build.log`, `reproduced-build.log`,
`wasm-clippy-final.log`, `hud-chord-red.log` and `hud-chord-green.log`. Selected images and video
are linked from the PR and morning review index. The source recorded above remains the measured
pre-merge artifact source; post-merge publication must rebuild and record the default-branch commit.

## Early proof, 2026-11-05

- Source: browser implementation based on planning commit `09c2646`, with uncommitted A1 changes;
  the final PR will identify its tested source revision. Existing Bevy 0.20.1, bevy_egui 0.43.1,
  nalgebra 0.35.0 and Rust 1.99.1 were retained.
- Device: Linux/Omarchy, AMD Radeon RX 7900 XT, RADV Vulkan/Mesa 26.2.5. Chromium
  **155.0.8059.84**, executable `/usr/lib/chromium/chromium`, desktop Wayland.
- Effective application flags: `--enable-unsafe-webgpu --enable-features=Vulkan --use-angle=vulkan
  --ozone-platform=wayland`. Playwright's additional launch defaults are recorded in the raw GPU
  report. This is an explicitly configured experimental Linux browser path.
- WebGPU adapter: AMD `rdna-3`; 16 storage buffers per shader stage, 1024 invocations/workgroup,
  1024 workgroup width, 65536 bytes workgroup storage. The simulation needs 8 storage buffers and
  a uniform, with 256-wide workgroups. No CPU fallback or streamed GPU server was used.
- `cargo build --profile web-release --target wasm32-unknown-unknown` completed in **3m16s** with
  no warnings. Lockfile additions enable WASM/WebGPU/browser randomness; there were no dependency
  upgrades. The native exporter build completed in **38.54s**, also without warnings.
- A real headed browser displayed 30,000 animated fish, underwater sky/IBL, caustic sand, shadows and
  the existing egui interface. The scene configured three shark hunters. The first close camera image
  does not itself establish their visibility or hunt readability; the final wide encounter evidence
  must make the sharks visible.
- First application-ready report: **2.553s** over uncompressed localhost HTTP on this machine. This is
  an early feasibility observation, not an internet loading claim or a sustained performance result.

Raw proof artifacts live outside Git at
`/home/martines/Projects/ac3d-evidence/2026-11-05/browser/`: `proof-shark-headed.png`,
`probe-headed.json`, `real-headed-vulkan.json`, `wasm-build-2.log` and `native-export-build.log`.
The PR/review index links selected evidence.

## Configurations that did not prove delivery

| Configuration | Observed result |
| --- | --- |
| Chromium without feature flags | No WebGPU adapter; clear unsupported-device screen |
| Arch `/usr/bin/chromium` wrapper plus explicit flags | Desktop flag file overrides features; no adapter |
| Direct binary, headless, Vulkan flags | AMD adapter and running application; CDP screenshot black, so presented pixels are not accepted |
| Direct binary, headed Wayland, Vulkan flags | Real presented scene; accepted early proof |

No Safari, Firefox, Edge, Windows, macOS, mobile or second GPU result is implied by this matrix.

## Browser-specific boundaries

Bevy's WASM task pool runs futures on the main thread. The native background environment convolution
therefore cannot be carried over unchanged. Packaging runs the native baker without a window and
ships its exact Rgba16Float cubes; WASM fetches, validates and uploads them. Generation checks keep
obsolete environment results from replacing a new scene. The bundle parser rejects invalid versions,
nonfinite metadata, invalid dimensions/lengths, truncation and trailing bytes, with native regression
tests for exact image/lighting round trips and malformed inputs.

Animal GLBs total roughly 340 KiB and are embedded in WASM. The shared bounded importer and baked
animation preserve the existing asynchronous model-generation checks. Local filesystem scanning and
arbitrary file imports remain available in the native Lab. HTTP assets use a generated catalog;
optional WESL module and metadata probes return absent locally instead of making noisy HTTP 404s.

The early unstripped WASM was about 140 MiB. Section inspection found **83,910,422 bytes** of debug
names, **50,298,807 bytes** of code and **10,834,491 bytes** of data. Release packaging removes the
name and producers sections, then precompresses the exact files with deterministic gzip. The final
packaged sizes and actual transfer are recorded above.

## Primary references checked during execution

- [Bevy setup and web profile guidance](https://bevy.org/learn/quick-start/getting-started/setup/).
- [wasm-bindgen deployment](https://rustwasm.github.io/docs/wasm-bindgen/reference/deployment.html)
  and [the matching 0.2.130 release](https://github.com/wasm-bindgen/wasm-bindgen/releases/tag/0.2.130).
- [getrandom WASM configuration](https://docs.rs/getrandom/0.4.3/getrandom/).
- [WebGPU supported limits](https://developer.mozilla.org/en-US/docs/Web/API/GPUSupportedLimits).
- [Chromium WebGPU troubleshooting](https://developer.chrome.com/docs/web-platform/webgpu/troubleshooting-tips)
  and [documented Linux automation flags](https://developer.chrome.com/blog/supercharge-web-ai-testing).
- Pinned local Bevy sources: `bevy_tasks/src/single_threaded_task_pool.rs`,
  `bevy_asset/src/io/wasm.rs`, `bevy_shader/src/shader.rs`, and
  `bevy_render/src/error_handler.rs` were read to verify task, fetch, optional import and recovery
  behavior. These sources are authoritative for the versions in `Cargo.lock`.
