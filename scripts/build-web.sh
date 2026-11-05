#!/usr/bin/env bash
# Build the shared native/WASM source and package a versioned, static browser release.
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."

jobs=${CARGO_BUILD_JOBS:-4}
if ! [[ "$jobs" =~ ^[1-4]$ ]]; then
  printf '%s\n' 'CARGO_BUILD_JOBS must be between 1 and 4.' >&2
  exit 2
fi
export CARGO_BUILD_JOBS="$jobs"
exec 9>"${AC3D_BUILD_LOCK:-/tmp/ac3d-heavy.lock}"
flock 9
if pgrep -x cargo >/dev/null || pgrep -x rustc >/dev/null; then
  printf '%s\n' 'Another Cargo build is active. Coordinate with its owner before building.' >&2
  exit 2
fi
if [[ "$(wasm-bindgen --version)" != 'wasm-bindgen 0.2.130' ]]; then
  printf '%s\n' 'Use wasm-bindgen-cli 0.2.130 to match Cargo.lock. Run scripts/setup-web-tools.sh.' >&2
  exit 2
fi

task_target=${CARGO_TARGET_DIR:-target}
task_target=$(realpath -m -- "$task_target")
task_native_target=$(realpath -m -- "${AC3D_NATIVE_TARGET_DIR:-$task_target}")
task_output=${AC3D_WEB_OUTPUT:-dist}
task_output=$(realpath -m -- "$task_output")
task_staging=$(mktemp -d)
trap 'python3 -c "import shutil, sys; shutil.rmtree(sys.argv[1])" "$task_staging"' EXIT

CARGO_TARGET_DIR="$task_native_target" cargo build --release --locked
AC3D_EXPORT_WEB_ASSETS="$task_staging/assets/environment" CARGO_MANIFEST_DIR="$PWD" \
  "$task_native_target/release/autonomous-characters-3d"
cargo build --profile web-release --locked --target wasm32-unknown-unknown
wasm-bindgen --target web --no-typescript --remove-name-section --remove-producers-section \
  --out-name flock --out-dir "$task_staging" \
  "$task_target/wasm32-unknown-unknown/web-release/autonomous-characters-3d.wasm"
cp -- web/bootstrap.js "$task_staging/bootstrap.js"
mkdir -p -- "$task_staging/assets"
cp -R -- assets/shaders assets/fonts "$task_staging/assets/"
cp -- assets/environment/CREDITS.md "$task_staging/assets/environment/CREDITS.md"

python3 scripts/package-web.py "$task_staging" "$task_output"
