"""Geometry and reproducibility checks for the original benchmark model generator."""
import json
from pathlib import Path
import struct
import unittest

from make_example_models import generate, generate_bird, generate_materials, unpack


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


    def test_animated_bird_is_reproducible_small_and_has_valid_skin_and_clips(self):
        blob = generate_bird()
        self.assertEqual(blob, generate_bird())
        asset = Path(__file__).resolve().parents[1] / 'assets/models/bird.glb'
        self.assertEqual(blob, asset.read_bytes())
        self.assertLess(len(blob), 100_000)
        document, binary = unpack(blob)
        self.assertEqual(len(document['skins'][0]['joints']), 3)
        self.assertEqual([clip['name'] for clip in document['animations']], ['Flap', 'Glide'])
        for part in parts(blob):
            self.assertGreater(area_and_winding(part), 0)
        for primitive in document['meshes'][0]['primitives']:
            for name, fmt, width in [('JOINTS_0', 'B', 1), ('WEIGHTS_0', 'f', 4)]:
                accessor = document['accessors'][primitive['attributes'][name]]
                self.assertEqual(accessor['type'], 'VEC4')
                view = document['bufferViews'][accessor['bufferView']]
                start = view['byteOffset']
                values = list(struct.iter_unpack('<4' + fmt, binary[start:start + accessor['count'] * 4 * width]))
                self.assertEqual(len(values), accessor['count'])
                for value in values:
                    if name == 'JOINTS_0':
                        self.assertTrue(all(0 <= joint < 3 for joint in value))
                    else:
                        self.assertAlmostEqual(sum(value), 1.0)
                        self.assertTrue(all(weight >= 0 for weight in value))
        for clip in document['animations']:
            self.assertEqual(len(clip['channels']), 3)
            self.assertTrue(all(channel['target']['path'] in ('rotation', 'translation') for channel in clip['channels']))
            self.assertTrue(all(sampler['interpolation'] == 'LINEAR' for sampler in clip['samplers']))

    def test_multipart_materials_are_reproducible_and_distinct(self):
        blob = generate_materials()
        self.assertEqual(blob, generate_materials())
        asset = Path(__file__).resolve().parents[1] / 'assets/models/material-glider.glb'
        self.assertEqual(blob, asset.read_bytes())
        self.assertLess(len(blob), 100_000)
        document, _ = unpack(blob)
        values = [material['pbrMetallicRoughness'] for material in document['materials']]
        self.assertEqual([v['metallicFactor'] for v in values], [1.0, 0.0])
        self.assertEqual([v['roughnessFactor'] for v in values], [0.12, 0.95])
        self.assertNotEqual(values[0]['baseColorFactor'], values[1]['baseColorFactor'])
        for part in parts(blob):
            self.assertGreater(area_and_winding(part), 0)


if __name__ == '__main__':
    unittest.main()
