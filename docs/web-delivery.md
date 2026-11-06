# Browser builds and static image delivery

The browser runs the same Rust simulation, instanced animal renderer, baked model animation, hunter
passes, environments and visitor UI as the native app. The server serves static files. It needs no
graphics device, database or persistent volume.

The first release targets desktop WebGPU. Native development and the CPU reference modes remain
available. WebGL cannot run this compute simulation. Mobile interaction, Safari, Firefox and other
GPUs must not be described as supported until tested. See the measured matrix in
[browser feasibility](browser-feasibility.md).

## Build on Linux x86_64

Use the pinned Rust 1.99.1 toolchain and the native prerequisites in
[new-machine setup](new-machine-setup.md), then install the pinned optional browser tools below.
No driver, desktop or system configuration change is needed by these scripts.

```sh
./scripts/setup-web-tools.sh
export PATH="$PWD/target/web-tools/bin:$PATH"
CARGO_BUILD_JOBS=4 ./scripts/build-web.sh
python3 -m http.server 8080 --bind 127.0.0.1 --directory dist
```

The setup script downloads wasm-bindgen-cli **0.2.130**, verifies the recorded SHA-256, and installs
the `wasm32-unknown-unknown` target. It installs no system packages. On another host, install the
matching CLI and target through that host's normal toolchain setup; the shell build script currently
requires GNU `realpath`, `flock` and `pgrep` and has only been tested on Linux.

The build script holds `/tmp/ac3d-heavy.lock`, rejects another Cargo process, and limits Cargo to four
jobs. Coordinate desktop sessions and performance measurements with other workers as described in
`AGENTS.md`. `CARGO_TARGET_DIR`, `AC3D_BUILD_LOCK` and `AC3D_WEB_OUTPUT` can override the build
directory, shared lock and output directory. `AC3D_NATIVE_TARGET_DIR` can reuse a separate native
cache for the exporter. That cache's release binary is replaced by this source build. Rebuild the
other checkout before launching its binary. Use the same shared lock across worktrees.

The script first builds the native release binary and invokes its headless
`AC3D_EXPORT_WEB_ASSETS` entry point. It exports the exact native half-float sky, irradiance and GGX
cubemaps. It then builds `web-release` for WASM and runs the pinned wasm-bindgen CLI. Removing its
debugging name/producers sections reduces transfer size without changing executable code.

Bundled animal GLBs are embedded only in WASM. They use the existing bounded importer, clips and
stale-request generation checks. Browser Lab cannot open arbitrary local paths; use the native Lab
for imports. Browsers fetch prepared environment bundles over HTTP, avoiding the long CPU bake on
the browser's single-thread task pool. Missing or malformed bundles fail visibly.

`dist/index.html` points at a content-hashed `releases/<hash>/` directory. That directory contains
JS/WASM, the asset catalog, shaders, font, prepared lighting and source metadata. The package includes
deterministic gzip variants and retains existing release directories on a local rebuild so previously
opened HTML can still find its immutable assets. A fresh checkout produces only the current release.
`dist/release.json` records the full source commit, dirty flag, profile, tool versions and content hash.
An uncommitted review build is marked dirty; it is not a published release.

The Python preview server provides a simple local proof. The Docker image below provides the actual
compression, MIME, cache and security headers used by CRONA.

## Run the exact static container locally

```sh
docker build --platform linux/amd64 \
  --build-arg SOURCE_REVISION="$(git rev-parse HEAD)" -t flock:review .
docker run --rm --name flock-review --read-only \
  --cap-drop=ALL --security-opt=no-new-privileges:true \
  --pids-limit=64 --cpus=1 --memory=128m \
  --tmpfs /tmp:rw,noexec,nosuid,size=32m,uid=101,gid=101,mode=1770 \
  -p 127.0.0.1:8080:8080 flock:review
```

The pinned nginx image runs as **101:101** on **8080** with one worker. Only `/tmp` is writable.
`/healthz` returns `200` and `ok`. HTML and release metadata revalidate with `Cache-Control: no-cache`.
Content-hashed assets are immutable for one year; WASM uses `application/wasm`. Precompressed assets
are selected with `Accept-Encoding: gzip` and `Vary: Accept-Encoding`. Do not serve an SPA fallback
for missing WASM or environment files. This runtime does not need cross-origin isolation headers.

```sh
curl -i http://127.0.0.1:8080/healthz
curl -I http://127.0.0.1:8080/index.html
curl -s http://127.0.0.1:8080/release.json
```

## Verify the visitor experience in a real browser

```sh
npm ci --ignore-scripts --prefix web
CHROMIUM_EXECUTABLE=/usr/lib/chromium/chromium \
  node scripts/smoke-web.mjs http://127.0.0.1:8080 /tmp/flock-browser-review
CHROMIUM_EXECUTABLE=/usr/lib/chromium/chromium \
  node scripts/measure-web.mjs http://127.0.0.1:8080 /tmp/flock-web-measurements
```

The smoke runner uses Playwright **1.60.0** and a headed browser so its screenshots prove presented
pixels. On the recorded Linux machine it uses the documented experimental WebGPU/Vulkan flags.
Use the actual Chromium binary: the Arch `/usr/bin/chromium` wrapper can merge desktop flags and
override feature flags. `CHROMIUM_EXECUTABLE` selects a different binary; `AC3D_BROWSER_ARGS` can
provide a JSON array of explicit arguments. The runner records the actual browser version, flags,
adapter, frame intervals, loading times, console messages, response failures and screenshots.
The separate measurement runner repeats all nine public scene/tier combinations: three seconds of
warmup and six seconds of frame intervals, twice each, at 1920×1080. It restarts the interactive GPU
scatter for each window; this is not a seeded replay or a GPU timestamp benchmark. Keep other builds,
simulation apps, benchmarks and video encoders idle during measurements.

The smoke runner also launches one disposable browser profile without Playwright's default focus
emulation. That unrouted persistent context proves real background-tab pause/return and cold/warm
HTTP cache behavior. It crashes only that disposable browser's GPU process through CDP, checks the
actual renderer error path, then reloads and verifies the restored scene. Failure fixtures use their
own contexts. A 2× display-factor browser checks real backing pixels; a context-only DPR override
does not update Chromium's device-pixel ResizeObserver consistently.

The HTML shell handles loading, unsupported adapters and startup errors. Rendering errors, including
device loss, stop rendering and offer reload. The shared visitor UI owns scenes, populations, cameras,
pause, help and Lab. Reduced camera motion follows the browser preference at startup and remains a
visitor control. Background tabs pause simulation and restore the prior pause state on return.
The first release offers no fullscreen control; browser fullscreen remains available through the
browser's own controls.

## Loading progress and scene reveal

The FLOCK shell shows progress for the current resource, followed by named graphics and scene
preparation stages. `asset-list.json` contains the existing asset paths in `assets` and deterministic
raw byte sizes in `decodedBytes`. Those sizes describe the decoded response stream; gzip's smaller
`Content-Length` is not used as its denominator. Unknown or inconsistent totals remain indeterminate
with a received-byte count. There is no guessed overall startup percentage.

The shell reveals the canvas only after Rust reports readiness, an application frame arrives, and
two browser paint opportunities pass. The normal reveal uses a short fade and slight canvas scale;
reduced motion uses opacity only. The main canvas stays inert until the reveal completes. Errors,
unsupported devices and device loss retain the recovery screen and invalidate an unfinished reveal.
`flockReport.rustReadyMs` records Rust readiness; `readyMs` includes the reveal and means the interface
is usable. Preserve this distinction when comparing loading measurements with earlier versions.

```sh
node --test web/loader.test.mjs
python3 -m unittest discover -s scripts -p test_package_web.py
CHROMIUM_EXECUTABLE=/usr/lib/chromium/chromium \
  node scripts/smoke-loading.mjs http://127.0.0.1:8080 /tmp/flock-loading-review
```

The headed loading check uses a recorded network throttle to capture real partial progress, checks
normal and reduced-motion reveals, and injects a clearly labeled failure during the reveal. The
general browser smoke still tests actual GPU-process loss and reload independently.

## Publish after review, under owner control

The `Publish web image` workflow is **manual only** and rejects any ref other than `master`, following
the household repositories. It builds clean source, verifies the recorded commit and clean flag,
then publishes Linux/amd64 as:

```text
ghcr.io/martines3000/autonomous-characters-3d-web:sha-<full-40-character-source-commit>
```

It records the real `image@sha256:...` in its run summary. No image has been published by preparing
this workflow. A source tag is convenient provenance; the cluster workload must pin the **returned
immutable digest**, not an invented value or a mutable tag.

After application PRs merge, the owner may dispatch the workflow from GitHub. Update the linked
household-cluster draft with that actual digest, validate and merge it, then verify internal Traefik
health. Only then prepare/approve the household-infra production plan for public routing.
Publication, cluster rollout, infrastructure apply, public DNS and post-release verification are
outside this review-preparation session. Roll back a future application update by restoring the
previous verified image digest through GitOps; use the household infra runbook for public-route
recovery.

The equivalent explicit owner command is `gh workflow run publish-web-image.yml --ref master`.
Do not run it during review preparation. The pre-merge local image, artifact checksums and tested
source are recorded in [browser feasibility](browser-feasibility.md); none is a published GHCR digest.
