# Builds the hero shark (assets/models/shark.glb): stylized low-poly shark, skinned travelling-wave swim animation.
# Run with Blender 4.5: blender --background --factory-startup --python scripts/make_shark_blender.py -- assets/models/shark.glb <preview_dir>
# Blender axes: forward = -Y, up = +Z  ->  glTF +Z forward, +Y up (the repo's model convention).
import bpy, bmesh, math, sys

out, preview = sys.argv[sys.argv.index("--") + 1:][:2]
bpy.ops.wm.read_factory_settings(use_empty=True)
scene = bpy.context.scene
scene.render.fps = 30

def material(name, rgb, metallic=0.0, roughness=0.5):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    p = m.node_tree.nodes["Principled BSDF"]
    p.inputs["Base Color"].default_value = (*rgb, 1.0)
    p.inputs["Metallic"].default_value = metallic
    p.inputs["Roughness"].default_value = roughness
    m.diffuse_color = (*rgb, 1.0)  # Workbench preview color
    m.use_backface_culling = True  # closed meshes; the importer rejects doubleSided
    return m

BACK = material("Back", (0.20, 0.25, 0.30), 0.0, 0.55)    # grey back
BELLY = material("Belly", (0.83, 0.85, 0.83), 0.0, 0.65)   # white belly
FIN = material("Fin", (0.18, 0.23, 0.28), 0.0, 0.6)       # grey fins
EYE = material("Eye", (0.02, 0.02, 0.02), 0.0, 0.2)

def obj(name, bm, mats, parent=None, location=(0, 0, 0)):
    mesh = bpy.data.meshes.new(name)
    bm.to_mesh(mesh)
    bm.free()
    for m in mats:
        mesh.materials.append(m)
    o = bpy.data.objects.new(name, mesh)
    scene.collection.objects.link(o)
    o.location = location
    if parent:
        o.parent = parent
    return o

SIDES = 8

def loft(rings, cap_start, cap_end):
    """Closed tube through (y, half-width, half-height, z-center) rings; w=0 makes a point."""
    bm = bmesh.new()
    loops = []
    for y, w, h, zc in rings:
        if w == 0.0:
            loops.append([bm.verts.new((0, y, zc))])
            continue
        loops.append([bm.verts.new((w * math.sin(a), y, zc + h * math.cos(a)))
                      for a in (2 * math.pi * i / SIDES for i in range(SIDES))])
    for a, b in zip(loops, loops[1:]):
        for i in range(SIDES):
            j = (i + 1) % SIDES
            if len(a) == 1:
                bm.faces.new((a[0], b[i], b[j]))
            else:
                bm.faces.new((a[i], a[j], b[j], b[i]))
    if cap_start and len(loops[0]) > 1:
        bm.faces.new(loops[0])
    if cap_end and len(loops[-1]) > 1:
        bm.faces.new(list(reversed(loops[-1])))
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    for f in bm.faces:
        f.material_index = 0 if f.normal.z > 0.1 else 1
    return bm

def plate(points, thickness, vertical=False, mat=0):
    """Extruded flat polygon: points are (x, y) in the XY plane, or (y, z) in the YZ plane."""
    bm = bmesh.new()
    if vertical:
        a = [bm.verts.new((thickness / 2, y, z)) for y, z in points]
        b = [bm.verts.new((-thickness / 2, y, z)) for y, z in points]
    else:
        a = [bm.verts.new((x, y, thickness / 2)) for x, y in points]
        b = [bm.verts.new((x, y, -thickness / 2)) for x, y in points]
    bm.faces.new(a)
    bm.faces.new(list(reversed(b)))
    n = len(points)
    for i in range(n):
        j = (i + 1) % n
        bm.faces.new((a[i], b[i], b[j], a[j]))
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    for f in bm.faces:
        f.material_index = mat
    return bm

# Continuous head-to-peduncle surface, weighted across six spine/tail joints.
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from aquatic_blender import build
from creature_blender import export_model, previews
rig, FRAMES = build(scene, obj, loft, plate, (BACK, BELLY, FIN, EYE), shark=True)
export_model(out, rig, "TravellingWave")
# Workbench previews from several angles and swim phases.
previews(scene, preview, (("three_quarter", (2.6, -2.2, 1.0), 0),
    ("side", (3.4, 0, .2), 8), ("top", (0, 0, 3.6), 8),
    ("top_phase", (0, 0, 3.6), 23), ("front", (0, -3.6, .3), 15)))
