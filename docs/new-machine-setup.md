# Set up a new native development machine

Follow these steps in order. Keep logs outside the checkout. Preserve source, dependency files and
`.cargo/config.toml`; report setup blockers instead of patching them. Legacy `index.html`, `web/`,
`Trunk.toml`, `Dockerfile` and WASM are out of scope; install no Trunk/WASM tooling.

## 1. Discover and report before installing anything

Run the applicable read-only commands; record missing tools without installing them. Record
architecture, logical CPUs, RAM, GPU/driver, and VM/container/sandbox status.

Linux (sh/bash):

```sh
uname -a
cat /etc/os-release
nproc
free -h
lspci -nnk
nvidia-smi
vulkaninfo --summary
printenv DISPLAY WAYLAND_DISPLAY XDG_SESSION_TYPE
loginctl list-sessions
```

macOS:

```sh
sw_vers
uname -m
sysctl hw.logicalcpu hw.memsize
system_profiler SPDisplaysDataType
stat -f '%Su' /dev/console
```

Windows (PowerShell):

```powershell
Get-CimInstance Win32_OperatingSystem | Select-Object Caption,Version,OSArchitecture,TotalVisibleMemorySize
Get-CimInstance Win32_Processor | Select-Object Name,NumberOfLogicalProcessors
Get-CimInstance Win32_VideoController | Select-Object Name,DriverVersion
query session
nvidia-smi
```

Confirm a local console or remote desktop is accessible to this user; display variables alone do not
prove this, and SSH alone supplies no desktop. Report machine facts, missing tools and achievable
verification levels (step 5) to the owner **before any installation**. Propose exact missing system
packages and driver changes; obtain approval before sudo/admin installs, driver changes or reboots.

## 2. Bootstrap tools and clone the native branch

Use existing tools first. Obtain approval for missing bootstrap tools (Git, Python 3, curl) and
platform prerequisites. Defer Linux library/driver installs until step 3's source check. Bevy's core
package lists below were checked against its local 0.20.1 guide; confirm supplementary names/distro
availability on the destination before requesting approval. Install only approved missing packages.

- Ubuntu/Debian candidates: `build-essential clang mold pkg-config git python3 curl`,
  `libasound2-dev libudev-dev libx11-dev libxcursor-dev libxrandr-dev libxi-dev`,
  `libxkbcommon-x11-0 libwayland-dev libxkbcommon-dev libvulkan1 vulkan-tools`; use `sudo apt-get install`.
- Fedora candidates: `gcc-c++ make clang mold pkgconf-pkg-config git python3 curl`,
  `alsa-lib-devel systemd-devel libX11-devel libXcursor-devel libXrandr-devel libXi-devel`,
  `wayland-devel libxkbcommon-devel libxkbcommon-x11 vulkan-loader vulkan-tools`; use `sudo dnf install`.
- Arch candidates: `base-devel clang mold pkgconf git python curl alsa-lib systemd`,
  `libx11 libxcursor libxrandr libxi wayland libxkbcommon libxkbcommon-x11 vulkan-icd-loader vulkan-tools`; use `sudo pacman -S`.
- Linux GPU driver candidates: `mesa-vulkan-drivers` (Ubuntu/Debian/Fedora),
  `vulkan-radeon`/`vulkan-intel` (Arch), or distro-supported NVIDIA proprietary driver
  with matching Vulkan ICD. Select for the hardware/distro and obtain approval.
  Loader/tools alone do not supply a hardware driver.
- Windows: have the owner install Visual Studio Build Tools with **Desktop
  development with C++**, MSVC and a Windows SDK, Git, Python 3 and rustup's
  `rustup-init.exe`. Use an MSVC developer PowerShell and a native MSVC Rust host.
  Approve driver updates separately. For WSL, discover GPU/display access and use Linux steps.
- macOS: check `xcode-select -p`; if absent, ask before `xcode-select --install`.
  Check `git --version`, `python3 --version`; obtain missing Python 3 from the owner.
  Metal uses the macOS GPU driver; no Vulkan package is required.

Install rustup if missing. On Linux/macOS, use the official installer after the initial report
(requires curl and network access on the destination):

```sh
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs -o /tmp/project-rustup-init.sh
sh /tmp/project-rustup-init.sh -y --profile minimal --default-toolchain none
. "$HOME/.cargo/env"
```

On Windows, let the owner complete rustup-init and reopen the developer shell.
Check `rustup --version`, `git --version`, `python3 --version`. If only `python` exists,
verify `python --version` is 3.x; substitute it below and report this.

```sh
git clone --branch codex/native-modernization https://github.com/martines3000/autonomous-characters-3d
cd autonomous-characters-3d
git branch --show-current
git rev-parse HEAD
git status --short
```

Require branch `codex/native-modernization` and a clean checkout; default `master` is not the
requested branch. If cloning fails, report it and resolve access. Read `AGENTS.md`, `README.md`,
`PLANS.md` and `plans/STATUS.md` before proceeding.

## 3. Check the pinned toolchain and Linux prerequisites

Coordinate one verification owner across all agents/worktrees. Before **every** Cargo command, check
`pgrep -fl 'cargo|rustc'` (Linux/macOS) or `Get-Process cargo,rustc -ErrorAction SilentlyContinue`
(PowerShell). Wait for other work; never overlap builds, tests, Clippy, benchmarks or profiling.
Keep one native app at most; close it before verification. Use `CARGO_BUILD_JOBS=4` or fewer. Only
the owner may raise this for the machine; record authorization and the deviation without editing
AGENTS.md.

Commands use sh syntax. In PowerShell, set `$env:CARGO_BUILD_JOBS='4'` and omit its sh prefix.
For GPU benchmark variables set
`$env:GPU_BENCH_COUNTS='20000,100000'`, `$env:GPU_BENCH_CAPS='0,16'`,
`$env:GPU_BENCH_MODES='wander,follow'` before the same Cargo command.
Run individually; check exit codes (`$LASTEXITCODE` in PowerShell); diagnose failures before continuing.

```sh
CARGO_BUILD_JOBS=4 cargo fetch --locked
rustc --version
```

The first Cargo invocation installs the `rust-toolchain.toml` pin if needed: Rust **1.99.1**,
rustfmt and Clippy. Require `rustc --version` to show 1.99.1. Report download/proxy/target failures;
preserve the toolchain and Cargo.lock.

On Linux, inspect `~/.cargo/registry/src/*/bevy-0.20.1/docs/linux_dependencies.md`
(or under `CARGO_HOME`) and its features against this repo's `Cargo.toml`. Defaults are off;
`3d`/`ui` enable `default_platform`: X11, Wayland, gamepads. ALSA is in Bevy's general list;
audio is disabled here. Keep libudev/window libraries for headless builds too.
Verify packages against these sources, obtain approval, install missing prerequisites.
Report an unavailable guide as a verification gap. Check libraries below; run the
clang/mold checks only on x86_64 GNU Linux (other architectures use their native linker):

```sh
pkg-config --modversion libudev x11 wayland-client xkbcommon
command -v clang
command -v mold
ls -l /usr/bin/mold
test -x /usr/bin/mold
```

On `x86_64-unknown-linux-gnu`, `.cargo/config.toml` requires clang and executable `/usr/bin/mold`.
If mold lives elsewhere, report both paths and ask the owner; leave the config unchanged. aarch64
Linux has no override. With a Linux
GPU, rerun `vulkaninfo --summary` (and NVIDIA's `nvidia-smi`); require the intended hardware
adapter/driver. Resolve library/driver failures through approved changes, then retry.

## 4. Run the repository gate in order

From the repository root, run the AGENTS.md sequence sequentially:

```sh
CARGO_BUILD_JOBS=4 cargo fmt --all -- --check
CARGO_BUILD_JOBS=4 cargo test --release --locked
CARGO_BUILD_JOBS=4 cargo build --release --locked
python3 scripts/smoke-native.py
git diff --check
```

At the smoke step, use level C below; explicitly skip it without a usable desktop, then run the diff
check. Require formatting success, passing tests, warning-free release build and clean diff.
Documented baseline: **33 passed, 5 ignored**; report actual counts. On failure, retain errors and
diagnose tools/libraries/access; report source failures instead of patching them during setup.

## 5. Complete and label each verification level

**A — no GPU/display required.** Step 4's fmt, ordinary tests, release build and diff check prove
formatting, CPU/headless behavior and compilation. Also run:

```sh
CARGO_BUILD_JOBS=4 cargo clippy --release --locked --all-targets
```

AGENTS.md states **7** existing Clippy warnings (not a gate). Report the actual count and investigate
additions. Level A does not execute the ignored GPU regressions or benchmarks.

**B — real GPU/driver access, no display required.** Run outside Codex's `workspace-write` sandbox,
which may hide the GPU. If access is unavailable, ask for an unrestricted session and mark B skipped
pending results. Use README.md's filter:

```sh
CARGO_BUILD_JOBS=4 cargo test --release --locked gpu_ -- --ignored --nocapture --test-threads=1 --skip gpu_flocking_benchmark
```

Require all three ignored regressions to pass (`vehicle::tests::` prefix):
`gpu_steering_matches_cpu_reference` (cap-0 CPU parity and retained headings),
`gpu_neighbor_cap_preserves_complete_separation` (cap-1 separation), and
`gpu_flock_spawns_grows_and_stays_bounded` (growth/truncation/reset and bounds). These execute
`src/gpu_flock.rs` compute/readback without a window. For “Unable to find a GPU” or wgpu adapter
errors, check driver, Vulkan ICD, container/sandbox GPU access and user permissions (Linux
render/video groups, `/dev/dri`). Report the cause; obtain approval for driver/group/access changes,
then retry.

Benchmark only on an idle machine with no build, test, benchmark or app running. Run `uptime` before
each run; repeat each benchmark at least twice at comparable load. On Windows, where `uptime`/Unix
load averages may be absent, record the limitation and before each run use:
`Get-CimInstance Win32_OperatingSystem | Select-Object LastBootUpTime` and
`Get-CimInstance Win32_Processor | Select-Object LoadPercentage`.

```sh
uptime
GPU_BENCH_COUNTS=20000,100000 GPU_BENCH_CAPS=0,16 GPU_BENCH_MODES=wander,follow CARGO_BUILD_JOBS=4 cargo test --release --locked gpu_flocking_benchmark -- --ignored --nocapture --test-threads=1
uptime
CARGO_BUILD_JOBS=4 cargo test --release --locked vehicle::tests::flocking_benchmark -- --exact --ignored --nocapture --test-threads=1
```

The exact CPU filter avoids the GPU benchmark; CPU measurement also works at level A.
GPU env vars are comma-separated. Record median/p95, count/mode/cap, completed/skipped steps
and density. GPU timing includes encode + submit + wait: 600 warmup, 60 timed steps at 1/60 s;
steps over 250 ms skip the remainder. Partial cases are not steady-state results.
Compare with README.md's M3 Pro tables and `plans/completed/2026-10-30-gpu-flocking.md`:
GPU wander/follow-cap-16 medians 0.67/1.4 ms at 20k, 2.5/6.4 ms at 100k;
CPU naive/octree 25.53/3.74 ms at 10k. Use these as baselines, not acceptance thresholds.

**C — graphics desktop/GPU required.** Run step 4's smoke script outside the sandbox. It opens
`target/release/autonomous-characters-3d` (`.exe` on Windows), supplies the asset root, waits ten
seconds and closes it. Require PASS: startup/assets only. Then run:
`CARGO_BUILD_JOBS=4 cargo run --release --locked`. Inspect flock, camera, target and mode switching
using README.md's controls. Close the app before continuing. Cargo supplies the asset root;
direct launches need `CARGO_MANIFEST_DIR`. Over SSH without a display, skip both runs and say why.
Request a desktop or VNC/remote desktop with GPU access to prove windowed rendering.

## 6. Hand back evidence and machine-local tooling gaps

The owner installs and logs into Claude Code/Codex CLI. Copy wanted user skills (e.g.
`~/.claude/skills/orchestrate`) and Claude Code project memory manually from the old machine; the
clone does not restore them.

Report: machine/driver/display facts and commit/branch; approved installations; levels A/B/C passed,
failed or skipped with reasons; ordinary and ignored test counts; compiler/Clippy warnings; repeated
benchmark numbers with load average (or its absence), parameters and partial cases; remaining
blockers; and every deviation from AGENTS.md, including skipped desktop checks or an owner-approved
job-limit change. Distinguish measured results from unverified capabilities.
