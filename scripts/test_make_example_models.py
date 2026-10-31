"""Geometry and reproducibility checks for the original benchmark model generator."""
import json
from pathlib import Path
import struct
import unittest

from make_example_models import generate


def parts(blob):
    json_length = struct.unpack_from('<I', blob, 12)[0]
    document = json.loads(blob[20:20 + json_length])
    binary = blob[28 + json_length:]
    result = []
    for primitive in document['meshes'][0]['primitives']:
        attributes = {}
        for name in ('POSITION', 'NORMAL'):
            accessor = document['accessors'][primitive['attributes'][name]]
            view = document['bufferViews'][accessor['bufferView']]
            start = view['byteOffset']
            attributes[name] = list(struct.iter_unpack('<3f', binary[start:start + view['byteLength']]))
        result.append(attributes)
    return result


def area_and_winding(part):
    points, normals = part['POSITION'], part['NORMAL']
    area = 0.0
    for i in range(0, len(points), 3):
        a, b, c = points[i:i + 3]
        ab = [b[k] - a[k] for k in range(3)]
        ac = [c[k] - a[k] for k in range(3)]
        cross = (ab[1] * ac[2] - ab[2] * ac[1],
                 ab[2] * ac[0] - ab[0] * ac[2],
                 ab[0] * ac[1] - ab[1] * ac[0])
        area += sum(v * v for v in cross) ** 0.5 / 2
        if sum(cross[k] * normals[i][k] for k in range(3)) <= 0:
            raise AssertionError('degenerate or reversed triangle')
    return area


class ExampleModelsTest(unittest.TestCase):
    def test_default_asset_is_unchanged(self):
        asset = Path(__file__).resolve().parents[1] / 'assets/models/glider.glb'
        self.assertEqual(generate(), asset.read_bytes())

    def test_subdivision_preserves_surface_bounds_winding_and_bytes(self):
        original = parts(generate())
        for divisions in (2, 12, 16):
            blob = generate(divisions)
            self.assertEqual(blob, generate(divisions))
            subdivided = parts(blob)
            self.assertEqual(sum(len(p['POSITION']) for p in subdivided), 72 * divisions ** 2)
            for base, dense in zip(original, subdivided):
                self.assertAlmostEqual(area_and_winding(base), area_and_winding(dense), places=6)
                for axis in range(3):
                    for bound in (min, max):
                        self.assertEqual(bound(p[axis] for p in base['POSITION']),
                                         bound(p[axis] for p in dense['POSITION']))
            if divisions == 12:
                self.assertLess(len(blob), 300_000)

    def test_subdivision_rejects_out_of_range_values(self):
        for value in (0, -1, 17):
            with self.assertRaises(ValueError):
                generate(value)


if __name__ == '__main__':
    unittest.main()
