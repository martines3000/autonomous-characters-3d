"""Catalog byte accuracy and repeated packaging; no renderer/toolchain needed."""

import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch


SPEC = importlib.util.spec_from_file_location("package_web", Path(__file__).with_name("package-web.py"))
PACKAGE_WEB = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PACKAGE_WEB)


class PackageWebTests(unittest.TestCase):
    def test_raw_catalog_sizes_and_repeated_packaging_are_identical(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            staging = root / "staging"
            (staging / "assets/environment").mkdir(parents=True)
            wasm = b"\0" * 8192
            lighting = b"light" * 1024
            (staging / "flock_bg.wasm").write_bytes(wasm)
            (staging / "assets/environment/sunset.flockenv").write_bytes(lighting)
            # Hold provenance fixed so repeated packaging tests the manifest/content contract.
            with patch.object(PACKAGE_WEB, "run", side_effect=["a" * 40, "", "a" * 40, ""]):
                PACKAGE_WEB.package(staging, root / "first")
                PACKAGE_WEB.package(staging, root / "second")
            release = json.loads((root / "first/release.json").read_text())
            files = root / "first/releases" / release["content"]
            catalog = json.loads((files / "asset-list.json").read_text())
            self.assertEqual(catalog["assets"], ["environment/sunset.flockenv"])
            self.assertEqual(catalog["decodedBytes"]["flock_bg.wasm"], len(wasm))
            self.assertEqual(catalog["decodedBytes"]["assets/environment/sunset.flockenv"], len(lighting))
            self.assertLess((files / "flock_bg.wasm.gz").stat().st_size, catalog["decodedBytes"]["flock_bg.wasm"])
            self.assertNotIn("asset-list.json", catalog["decodedBytes"])

            def contents(output):
                return {p.relative_to(output).as_posix(): p.read_bytes() for p in output.rglob("*") if p.is_file()}

            self.assertEqual(contents(root / "first"), contents(root / "second"))


if __name__ == "__main__":
    unittest.main()
