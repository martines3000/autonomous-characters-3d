#!/usr/bin/env python3
"""Package a build as immutable content, with revalidated HTML and source provenance."""

import gzip
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys


def run(*args):
    return subprocess.check_output(args, text=True).strip()


def package(staging: Path, output: Path):
    root = Path(__file__).resolve().parents[1]
    revision = run("git", "-C", str(root), "rev-parse", "HEAD")
    dirty = bool(run("git", "-C", str(root), "status", "--porcelain", "--untracked-files=normal"))
    provenance = {"source": "https://github.com/martines3000/autonomous-characters-3d", "revision": revision, "dirty": dirty, "rust": "1.99.1", "wasm_bindgen": "0.2.130", "profile": "web-release"}
    catalog = sorted(path.relative_to(staging / "assets").as_posix() for path in (staging / "assets").rglob("*") if path.is_file())
    (staging / "asset-list.json").write_text(json.dumps(catalog) + "\n")
    (staging / "source.json").write_text(json.dumps(provenance, indent=2) + "\n")
    files = sorted(path for path in staging.rglob("*") if path.is_file())
    digest = hashlib.sha256()
    for path in files:
        digest.update(path.relative_to(staging).as_posix().encode() + b"\0")
        digest.update(path.read_bytes())
    content = digest.hexdigest()[:24]
    release = output / "releases" / content
    release.mkdir(parents=True, exist_ok=True)
    for path in files:
        target = release / path.relative_to(staging)
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(path, target)
        if target.suffix in {".wasm", ".js", ".wgsl", ".wesl", ".flockenv", ".json", ".md", ".ttf"}:
            target.with_name(target.name + ".gz").write_bytes(gzip.compress(target.read_bytes(), compresslevel=9, mtime=0))
    html = (root / "web/index.html").read_text().replace("__RELEASE_BASE__", f"./releases/{content}/")
    (output / "index.html").write_text(html)
    metadata = dict(provenance, content=content)
    (output / "release.json").write_text(json.dumps(metadata, indent=2) + "\n")
    print(f"Web release {content}: {revision}, dirty={dirty}; output {output}")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        raise SystemExit("Usage: scripts/package-web.py STAGING OUTPUT")
    package(Path(sys.argv[1]).resolve(), Path(sys.argv[2]).resolve())
