# Builds the hero swallow (assets/models/swallow.glb): stylized low-poly barn swallow, rigid flap animation.
# Run with Blender 4.5: blender --background --factory-startup --python scripts/make_swallow_blender.py -- assets/models/swallow.glb <preview_dir>
# Blender axes: forward = -Y, up = +Z  ->  glTF +Z forward, +Y up (the repo's model convention).
import bpy, bmesh, math, sys
from mathutils import Vector

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

BACK = material("Back", (0.035, 0.07, 0.22), 0.15, 0.35)   # glossy steel blue
BELLY = material("Belly", (0.86, 0.78, 0.66), 0.0, 0.7)     # cream
THROAT = material("Throat", (0.62, 0.20, 0.07), 0.0, 0.6)   # rust
BEAK = material("Beak", (0.03, 0.03, 0.03), 0.0, 0.4)

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

# Body: lofted rings along -Y (nose) to +Y (tail root). (y, half-width, half-height, z-center)
rings = [(-1.00, 0.0, 0.0, 0.10), (-0.93, 0.09, 0.08, 0.11), (-0.82, 0.14, 0.13, 0.11),
         (-0.68, 0.13, 0.12, 0.08), (-0.52, 0.21, 0.21, 0.02), (-0.30, 0.25, 0.25, -0.02),
         (-0.05, 0.22, 0.21, -0.02), (0.22, 0.15, 0.13, 0.0), (0.45, 0.08, 0.06, 0.02),
         (0.58, 0.05, 0.035, 0.03)]
SIDES = 10
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
bm.faces.new(list(reversed(loops[-1])))
bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
for f in bm.faces:
    c = f.calc_center_median()
    if f.normal.z > 0.15:
        f.material_index = 0
    elif c.y < -0.58 and f.normal.z > -0.8:
        f.material_index = 2  # throat
    else:
        f.material_index = 1
# Beak: small pyramid at the nose.
tip = bm.verts.new((0, -1.13, 0.08))
base = [bm.verts.new((0.04 * math.sin(a), -0.97, 0.10 + 0.035 * math.cos(a)))
        for a in (2 * math.pi * i / 4 for i in range(4))]
for i in range(4):
    f = bm.faces.new((base[i], base[(i + 1) % 4], tip))
    f.material_index = 3
body = obj("Body", bm, [BACK, BELLY, THROAT, BEAK])

def plate(points, thickness, mat_top=0, mat_bottom=1):
    """Extruded flat polygon (in XY at z=0) with top/bottom materials."""
    bm = bmesh.new()
    top = [bm.verts.new((x, y, thickness / 2)) for x, y in points]
    bot = [bm.verts.new((x, y, -thickness / 2)) for x, y in points]
    bm.faces.new(top).material_index = mat_top
    bm.faces.new(list(reversed(bot))).material_index = mat_bottom
    n = len(points)
    for i in range(n):
        j = (i + 1) % n
        bm.faces.new((top[i], bot[i], bot[j], top[j])).material_index = mat_top
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    return bm

# Forked tail: two long streamers plus a fan, attached at the tail root.
tail_pts = [(-0.10, 0.0), (0.10, 0.0), (0.30, 0.95), (0.20, 0.98), (0.0, 0.42), (-0.20, 0.98), (-0.30, 0.95)]
tail = obj("Tail", plate(tail_pts, 0.025), [BACK, BELLY], parent=body, location=(0, 0.50, 0.03))

wings = {}
for side, s in (("L", 1.0), ("R", -1.0)):
    # Inner wing (arm): shoulder at x=0, wrist at x=0.62, swept slightly back.
    inner_pts = [(0.0, -0.20), (0.30, -0.18), (0.58, -0.10), (0.60, 0.14), (0.30, 0.22), (0.0, 0.24)]
    # Swallow hand: long, narrow, swept-back crescent ending in a sharp tip.
    outer_pts = [(0.0, -0.10), (0.45, -0.02), (0.90, 0.24), (1.18, 0.58), (0.80, 0.42), (0.38, 0.26), (0.0, 0.14)]
    if s < 0:
        inner_pts = [(-x, y) for x, y in reversed(inner_pts)]
        outer_pts = [(-x, y) for x, y in reversed(outer_pts)]
    inner = obj(f"Wing{side}", plate(inner_pts, 0.04), [BACK, BELLY], parent=body,
                location=(s * 0.16, -0.36, 0.13))
    outer = obj(f"Hand{side}", plate(outer_pts, 0.03), [BACK, BELLY], parent=inner,
                location=(s * 0.58, 0.0, 0.0))
    wings[side] = (inner, outer, s)

# Flap cycle: 24 frames (0.8 s at 30 fps), keyed every 2 frames, exported LINEAR.
FRAMES = 24
for f in range(0, FRAMES + 1, 2):
    t = 2 * math.pi * f / FRAMES
    down = math.sin(t)
    for inner, outer, s in wings.values():
        inner.rotation_euler = (0.0, s * -math.radians(10 + 38 * down), 0.0)
        outer.rotation_euler = (0.0, s * -math.radians(22 * math.sin(t - 0.9)), 0.0)
        inner.keyframe_insert("rotation_euler", frame=f)
        outer.keyframe_insert("rotation_euler", frame=f)
    body.location = (0.0, 0.0, -0.04 * down)
    body.keyframe_insert("location", frame=f)
    tail.rotation_euler = (math.radians(4 * math.sin(t + 1.2)), 0.0, 0.0)
    tail.keyframe_insert("rotation_euler", frame=f)
for o in bpy.data.objects:
    if o.animation_data and o.animation_data.action:
        for fc in o.animation_data.action.fcurves:
            for k in fc.keyframe_points:
                k.interpolation = "LINEAR"
scene.frame_start, scene.frame_end = 0, FRAMES

bpy.ops.object.select_all(action="SELECT")
bpy.context.view_layer.objects.active = body
bpy.ops.export_scene.gltf(filepath=out, export_format="GLB", use_selection=True,
                          use_active_scene=True, export_animations=True,
                          export_frame_range=True, export_force_sampling=True,
                          export_yup=True, export_apply=False, export_texcoords=False,
                          export_materials="EXPORT")
tris = sum(len(p.vertices) - 2 for o in bpy.data.objects if o.type == "MESH" for p in o.data.polygons)
print("PASS exported", out, "triangles", tris)

# Workbench previews from three angles at two flap phases.
cam = bpy.data.objects.new("Cam", bpy.data.cameras.new("Cam"))
scene.collection.objects.link(cam)
scene.camera = cam
scene.render.engine = "BLENDER_WORKBENCH"
scene.display.shading.light = "STUDIO"
scene.display.shading.color_type = "MATERIAL"
scene.render.resolution_x, scene.render.resolution_y = 640, 480
world = bpy.data.worlds.new("W"); world.color = (0.55, 0.62, 0.72); scene.world = world
for name, loc, frame in (("three_quarter_up", (2.6, -2.4, 1.6), 0), ("three_quarter_down", (2.6, -2.4, 1.6), 12),
                         ("side", (3.4, 0.0, 0.3), 6), ("top", (0.0, 0.0, 3.6), 6), ("front", (0.0, -3.6, 0.4), 18)):
    scene.frame_set(frame)
    cam.location = loc
    direction = Vector((0, 0, 0)) - Vector(loc)
    cam.rotation_euler = direction.to_track_quat("-Z", "Y").to_euler()
    scene.render.filepath = f"{preview}/{name}.png"
    bpy.ops.render.render(write_still=True)
print("PASS previews", preview)
