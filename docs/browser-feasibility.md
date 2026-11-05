# Browser feasibility and measured support

This records A1's early proof, before the visitor HUD and scene tuning are integrated. Final browser
acceptance and release measurements are added after that integration. A successful compile or an
application resource counter alone is not proof of a presented scene.

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
name and producers sections, then precompresses the exact files with deterministic gzip. Final raw
and transferred sizes must be recorded from that packaged artifact.

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
