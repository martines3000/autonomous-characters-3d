#!/usr/bin/env python3
"""Generate the original, deterministic static glider (repository terms apply)."""
import argparse
import json
import math
from pathlib import Path
import struct


def generate(subdivisions=1):
    """Author +Z forward, +Y up, with a pointed gold hull and blue swept wings."""
    parts = []

    def body():
        ring = [(0.22, 0, 0), (0, 0.20, 0), (-0.22, 0, 0), (0, -0.20, 0)]
        triangles = []
        for tip in [(0, 0, 2.0), (0, 0, -1.0)]:
            for i in range(4):
                triangles.append([tip, ring[i], ring[(i + 1) % 4]])
        return triangles

    def wing(sign):
        a, b, c = (0.12 * sign, 0, 0.55), (1.3 * sign, 0, -0.65), (0.12 * sign, 0, -0.85)
        top = [(x, 0.055, z) for x, _, z in (a, b, c)]
        bottom = [(x, -0.055, z) for x, _, z in (a, b, c)]
        triangles = [top, list(reversed(bottom))]
        for i in range(3):
            j = (i + 1) % 3
            triangles.extend([[bottom[i], bottom[j], top[j]], [bottom[i], top[j], top[i]]])
        return triangles

    def outward(triangles):
        points = [p for triangle in triangles for p in triangle]
        center = tuple(sum(p[i] for p in points) / len(points) for i in range(3))
        positions, normals = [], []
        for a, b, c in triangles:
            ab = [b[i] - a[i] for i in range(3)]
            ac = [c[i] - a[i] for i in range(3)]
            n = [ab[1] * ac[2] - ab[2] * ac[1], ab[2] * ac[0] - ab[0] * ac[2], ab[0] * ac[1] - ab[1] * ac[0]]
            if sum(n[i] * ((a[i] + b[i] + c[i]) / 3 - center[i]) for i in range(3)) < 0:
                b, c = c, b
                n = [-x for x in n]
            length = math.sqrt(sum(x * x for x in n))
            positions.extend([a, b, c])
            normals.extend([tuple(x / length for x in n)] * 3)
        return positions, normals

    parts.append(outward(body()))
    left = outward(wing(-1))
    right = outward(wing(1))
    parts.append((left[0] + right[0], left[1] + right[1]))
    if not 1 <= subdivisions <= 16:
        raise ValueError('subdivisions must be in 1..=16')
    if subdivisions > 1:
        def subdivide(part):
            positions, normals = part
            result, result_normals = [], []
            for offset in range(0, len(positions), 3):
                a, b, c = positions[offset:offset + 3]
                def point(i, j):
                    return tuple(((subdivisions - i - j) * a[k] + i * b[k] + j * c[k]) / subdivisions for k in range(3))
                def triangle(*vertices):
                    result.extend(vertices)
                    result_normals.extend([normals[offset]] * 3)
                for i in range(subdivisions):
                    for j in range(subdivisions - i):
                        triangle(point(i, j), point(i + 1, j), point(i, j + 1))
                        if i + j < subdivisions - 1:
                            triangle(point(i + 1, j), point(i + 1, j + 1), point(i, j + 1))
            return result, result_normals
        parts = [subdivide(part) for part in parts]
    data = bytearray()
    views, accessors, primitives = [], [], []

    def attribute(values, bounds=False):
        offset = len(data)
        for value in values:
            data.extend(struct.pack('<3f', *value))
        views.append({'buffer': 0, 'byteOffset': offset, 'byteLength': len(data) - offset})
        item = {'bufferView': len(views) - 1, 'componentType': 5126, 'count': len(values), 'type': 'VEC3'}
        if bounds:
            item['min'] = [min(p[i] for p in values) for i in range(3)]
            item['max'] = [max(p[i] for p in values) for i in range(3)]
        accessors.append(item)
        return len(accessors) - 1

    for material, (positions, normals) in enumerate(parts):
        primitives.append({'attributes': {'POSITION': attribute(positions, True), 'NORMAL': attribute(normals)}, 'material': material})
    document = {
        'asset': {'version': '2.0', 'generator': 'Original AC3D glider; scripts/make_example_models.py'},
        'scene': 0, 'scenes': [{'nodes': [0]}],
        'nodes': [{'name': 'Glider (+Z forward, +Y up)', 'mesh': 0}],
        'meshes': [{'primitives': primitives}],
        'materials': [
            {'name': 'Gold hull', 'pbrMetallicRoughness': {'baseColorFactor': [1.0, 0.35, 0.025, 1.0]}},
            {'name': 'Blue wings', 'pbrMetallicRoughness': {'baseColorFactor': [0.025, 0.45, 1.0, 1.0]}},
        ],
        'accessors': accessors, 'bufferViews': views, 'buffers': [{'byteLength': len(data)}],
    }
    encoded = json.dumps(document, sort_keys=True, separators=(',', ':'), ensure_ascii=True).encode('ascii')
    encoded += b' ' * (-len(encoded) % 4)
    data += b'\0' * (-len(data) % 4)
    length = 12 + 8 + len(encoded) + 8 + len(data)
    return struct.pack('<4sII', b'glTF', 2, length) + struct.pack('<I4s', len(encoded), b'JSON') + encoded + struct.pack('<I4s', len(data), b'BIN\0') + data


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path(__file__).resolve().parents[1] / 'assets/models/glider.glb')
    parser.add_argument('--subdivisions', type=int, choices=range(1, 17), default=1,
                        help='split each triangle into N*N coplanar triangles; 12 produces 3456 triangles')
    args = parser.parse_args()
    path = args.output
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(generate(args.subdivisions))
    triangles = 24 * args.subdivisions ** 2
    print(f'{path}: {path.stat().st_size} bytes; 2 primitives, {triangles * 3} vertices, {triangles} triangles')
