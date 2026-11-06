#!/usr/bin/env bash
# Install only the pinned browser target and a checksummed, prebuilt wasm-bindgen CLI.
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
if [[ "$(uname -s)" != Linux || "$(uname -m)" != x86_64 ]]; then
  printf '%s\n' 'This bootstrap supports Linux x86_64. Other hosts need wasm-bindgen-cli 0.2.130 and wasm32-unknown-unknown.' >&2
  exit 2
fi
task_tools=${AC3D_WEB_TOOLS_DIR:-$PWD/target/web-tools}
mkdir -p -- "$task_tools/bin"
rustup target add wasm32-unknown-unknown --toolchain 1.99.1
task_archive=$(mktemp)
trap 'unlink -- "$task_archive"' EXIT
curl --proto '=https' --tlsv1.2 --fail --location --silent --show-error \
  https://github.com/wasm-bindgen/wasm-bindgen/releases/download/0.2.130/wasm-bindgen-0.2.130-x86_64-unknown-linux-musl.tar.gz \
  -o "$task_archive"
printf '%s  %s\n' 9687f3bb551b9f66f37ed32a40d7913a00f7962db653054e56857bc306c3227f "$task_archive" | sha256sum --check --status
tar -xzf "$task_archive" -C "$task_tools/bin" --strip-components=1 \
  wasm-bindgen-0.2.130-x86_64-unknown-linux-musl/wasm-bindgen
"$task_tools/bin/wasm-bindgen" --version
printf 'Add %s/bin to PATH before building.\n' "$task_tools"
