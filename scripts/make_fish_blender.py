# Builds the hero fish (assets/models/fish.glb): stylized low-poly schooling fish, rigid swim animation.
# Run with Blender 4.5: blender --background --factory-startup --python scripts/make_fish_blender.py -- assets/models/fish.glb <preview_dir>
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

BACK = material("Back", (0.02, 0.22, 0.30), 0.3, 0.35)    # deep teal
BELLY = material("Belly", (0.72, 0.78, 0.80), 0.6, 0.3)   # silver
FIN = material("Fin", (0.90, 0.68, 0.18), 0.0, 0.5)       # warm yellow
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

# Front body (head to mid-body). Each half's end tapers inside the other, so the swim joint never gaps.
front = obj("Body", loft([(-1.00, 0.0, 0.0, 0.0), (-0.90, 0.07, 0.10, 0.0), (-0.70, 0.13, 0.22, 0.01),
                          (-0.40, 0.16, 0.30, 0.02), (-0.10, 0.15, 0.28, 0.02), (0.02, 0.145, 0.26, 0.02),
                          (0.12, 0.10, 0.18, 0.02)], False, True), [BACK, BELLY])
# Rear body pivots at y=0: tapers to the caudal peduncle.
rear = obj("Rear", loft([(-0.10, 0.13, 0.24, 0.02), (0.02, 0.15, 0.265, 0.02), (0.20, 0.10, 0.18, 0.02),
                         (0.45, 0.05, 0.09, 0.02), (0.58, 0.03, 0.05, 0.02)], True, True), [BACK, BELLY],
           parent=front)
# Forked caudal fin, pivoting at the peduncle.
tail = obj("Tail", plate([(0.0, 0.06), (0.30, 0.34), (0.40, 0.32), (0.18, 0.0), (0.40, -0.30), (0.30, -0.32),
                          (0.0, -0.05)], 0.02, vertical=True), [FIN], parent=rear, location=(0, 0.55, 0.02))
# Swept dorsal and anal fins.
obj("Dorsal", plate([(-0.40, 0.26), (-0.15, 0.48), (0.08, 0.50), (0.05, 0.24)], 0.02, vertical=True), [FIN],
    parent=front)
obj("Anal", plate([(0.02, -0.20), (0.22, -0.34), (0.32, -0.30), (0.30, -0.12)], 0.02, vertical=True), [FIN],
    parent=rear)
for side, s in (("L", 1.0), ("R", -1.0)):
    pts = [(0.0, 0.0), (0.22, 0.10), (0.24, 0.22), (0.0, 0.12)]
    if s < 0:
        pts = [(-x, y) for x, y in reversed(pts)]
    fin = obj(f"Pectoral{side}", plate(pts, 0.015), [FIN], parent=front, location=(s * 0.13, -0.50, -0.10))
    fin.rotation_euler = (0.0, s * math.radians(25), 0.0)
    eye = bmesh.new()
    bmesh.ops.create_icosphere(eye, subdivisions=1, radius=0.045)
    obj(f"Eye{side}", eye, [EYE], parent=front, location=(s * 0.095, -0.78, 0.06))

# Swim cycle: 24 frames (0.8 s at 30 fps), keyed every 2 frames, exported LINEAR.
# A travelling wave: head yaws slightly, the rear and tail follow with growing lag and amplitude.
FRAMES = 24
for f in range(0, FRAMES + 1, 2):
    t = 2 * math.pi * f / FRAMES
    front.rotation_euler = (0.0, 0.0, math.radians(4 * math.sin(t)))
    rear.rotation_euler = (0.0, 0.0, math.radians(-16 * math.sin(t - 0.9)))
    tail.rotation_euler = (0.0, 0.0, math.radians(-22 * math.sin(t - 1.8)))
    for o in (front, rear, tail):
        o.keyframe_insert("rotation_euler", frame=f)
for o in bpy.data.objects:
    if o.animation_data and o.animation_data.action:
        for fc in o.animation_data.action.fcurves:
            for k in fc.keyframe_points:
                k.interpolation = "LINEAR"
scene.frame_start, scene.frame_end = 0, FRAMES

bpy.ops.object.select_all(action="SELECT")
bpy.context.view_layer.objects.active = front
bpy.ops.export_scene.gltf(filepath=out, export_format="GLB", use_selection=True,
                          use_active_scene=True, export_animations=True,
                          export_frame_range=True, export_force_sampling=True,
                          export_yup=True, export_apply=False, export_texcoords=False,
                          export_materials="EXPORT")
tris = sum(len(p.vertices) - 2 for o in bpy.data.objects if o.type == "MESH" for p in o.data.polygons)
print("PASS exported", out, "triangles", tris)

# Workbench previews from several angles and swim phases.
cam = bpy.data.objects.new("Cam", bpy.data.cameras.new("Cam"))
scene.collection.objects.link(cam)
scene.camera = cam
scene.render.engine = "BLENDER_WORKBENCH"
scene.display.shading.light = "STUDIO"
scene.display.shading.color_type = "MATERIAL"
scene.render.resolution_x, scene.render.resolution_y = 640, 480
world = bpy.data.worlds.new("W"); world.color = (0.30, 0.55, 0.62); scene.world = world
for name, loc, frame in (("three_quarter", (2.6, -2.2, 1.0), 0), ("side", (3.4, 0.0, 0.2), 6),
                         ("top", (0.0, 0.0, 3.6), 6), ("top_phase", (0.0, 0.0, 3.6), 18),
                         ("front", (0.0, -3.6, 0.3), 12)):
    scene.frame_set(frame)
    cam.location = loc
    direction = Vector((0, 0, 0)) - Vector(loc)
    cam.rotation_euler = direction.to_track_quat("-Z", "Y").to_euler()
    scene.render.filepath = f"{preview}/{name}.png"
    bpy.ops.render.render(write_still=True)
print("PASS previews", preview)
