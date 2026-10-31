#!/usr/bin/env python3
"""Check that the release app survives startup on a desktop with graphics support."""

import os
from pathlib import Path
import subprocess
import tempfile


root = Path(__file__).resolve().parents[1]
binary = root / "target" / "release" / "autonomous-characters-3d"
if os.name == "nt":
    binary = binary.with_suffix(".exe")
env = dict(os.environ, CARGO_MANIFEST_DIR=str(root), RUST_LOG="warn,wgpu=error")

with tempfile.TemporaryFile(mode="w+") as log:
    process = subprocess.Popen([str(binary)], env=env, stdout=log, stderr=log)
    exited = False
    try:
        try:
            process.wait(timeout=10)
            exited = True
        except subprocess.TimeoutExpired:
            pass
    finally:
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
    log.seek(0)
    output = log.read()

errors = ("panicked at", "error", "failed to load", "path not found")
if exited or any(error in output.lower() for error in errors):
    raise SystemExit(f"Native startup smoke check failed:\n{output}")
print("PASS: native release survived 10 seconds without startup or asset errors.")
