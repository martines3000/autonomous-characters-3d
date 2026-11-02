"""Continuous low-poly aquatic lofts, weighted spines and frame-sampled swimming."""
import math
import bpy
import bmesh
from creature_blender import linear_keys


def build(scene, obj, loft, plate, materials, shark=False):
    back, belly, fin_material, eye_material = materials
    if shark:
        # Broad shoulders, slender caudal peduncle, and a flattened pointed snout.
        rings = [(-1.30, 0, 0, 0), (-1.12, .16, .08, 0), (-.90, .25, .17, 0),
                 (-.62, .30, .24, 0), (-.30, .28, .25, 0), (0, .23, .22, 0),
                 (.25, .17, .17, 0), (.48, .11, .12, .01), (.67, .065, .075, .02),
                 (.83, .035, .05, .03)]
    else:
        rings = [(-1.00, 0, 0, 0), (-.90, .07, .10, 0), (-.70, .13, .22, .01),
                 (-.50, .155, .28, .02), (-.30, .16, .30, .02), (-.10, .15, .28, .02),
                 (.08, .13, .24, .02), (.26, .095, .17, .02), (.43, .055, .10, .02),
                 (.60, .03, .05, .02), (.72, .025, .04, .02)]
    body = obj("Body", loft(rings, False, True), [back, belly])
    # One continuous surface replaces overlapping rigid halves, so no swim joint gaps.
    # Forked caudal fin, pivoting at the peduncle.
    tail_y = .80 if shark else .69
    tail_points = ([(0, .06), (.35, .67), (.48, .74), (.32, .22), (.23, .02),
                    (.40, -.31), (.29, -.28), (0, -.04)] if shark else
                   [(0, .05), (.30, .34), (.40, .32), (.18, 0), (.40, -.30), (.30, -.32), (0, -.04)])
    tail = obj("Tail", plate([(y + tail_y, z + .02) for y, z in tail_points], .022, vertical=True), [fin_material])
    # Swept dorsal and anal fins.
    dorsal_points = ([(-.45, .22), (-.20, .82), (.10, .21)] if shark else
                     [(-.40, .26), (-.15, .48), (.08, .50), (.05, .24)])
    dorsal = obj("Dorsal", plate(dorsal_points, .025, vertical=True), [fin_material])
    anal = obj("Anal", plate([(.16, -.16), (.33, -.32), (.42, -.26), (.40, -.10)], .018, vertical=True), [fin_material])
    pectorals, eyes = [], []
    for side, sign in (("L", 1), ("R", -1)):
        points = ([(0, 0), (.78, .42), (.48, .45), (0, .18)] if shark else
                  [(0, 0), (.22, .10), (.24, .22), (0, .12)])
        if sign < 0:
            points = [(-x, y) for x, y in reversed(points)]
        x = sign * (.24 if shark else .13)
        fin = obj("Pectoral" + side, plate(points, .018), [fin_material], location=(x, -.50, -.10))
        # Bake the attachment offset into vertices before skinning.
        for vertex in fin.data.vertices:
            vertex.co += fin.location
        fin.location = (0, 0, 0)
        pectorals.append((fin, sign, x))
        eye = bmesh.new()
        bmesh.ops.create_icosphere(eye, subdivisions=1, radius=.032 if shark else .045)
        eyes.append(obj("Eye" + side, eye, [eye_material], location=(sign * (.205 if shark else .095), -.88 if shark else -.78, .06)))
    # A travelling wave: head yaws slightly, the rear and tail follow with growing lag and amplitude.
    armature = bpy.data.armatures.new("SwimmingSpine")
    rig = bpy.data.objects.new("SwimmingSpine", armature)
    scene.collection.objects.link(rig)
    bpy.context.view_layer.objects.active = rig
    rig.select_set(True)
    bpy.ops.object.mode_set(mode="EDIT")
    pivots = [-1.05, -.55, -.12, .25, .53, tail_y]
    bones = []
    for i, y in enumerate(pivots):
        bone = armature.edit_bones.new("Spine" + str(i))
        bone.head = (0, y, 0)
        bone.tail = (0, pivots[i + 1] if i + 1 < len(pivots) else y + .25, 0)
        if bones:
            bone.parent = bones[-1]
            bone.use_connect = True
        bones.append(bone)
    for _, sign, x in pectorals:
        bone = armature.edit_bones.new("Pectoral" + str(sign))
        bone.head, bone.tail = (x, -.50, -.10), (x + sign * .20, -.40, -.10)
        bone.parent = bones[1]
    bpy.ops.object.mode_set(mode="OBJECT")

    def skin(mesh, fixed=None):
        groups = {b.name: mesh.vertex_groups.new(name=b.name) for b in rig.data.bones}
        for vertex in mesh.data.vertices:
            if fixed:
                groups[fixed].add([vertex.index], 1.0, "REPLACE")
                continue
            y = vertex.co.y
            i = max(0, min(len(pivots) - 2, next((j - 1 for j, p in enumerate(pivots) if p > y), len(pivots) - 2)))
            weight = max(0.0, min(1.0, (y - pivots[i]) / (pivots[i + 1] - pivots[i])))
            groups["Spine" + str(i)].add([vertex.index], 1 - weight, "REPLACE")
            groups["Spine" + str(i + 1)].add([vertex.index], weight, "REPLACE")
        modifier = mesh.modifiers.new("ContinuousBodyWave", "ARMATURE")
        modifier.object = rig
        mesh.parent = rig

    for mesh in (body, dorsal, anal):
        skin(mesh)
    skin(tail, "Spine5")
    for mesh, sign, _ in pectorals:
        skin(mesh, "Pectoral" + str(sign))
    for mesh in eyes:
        # Eyes follow the head exactly, including their attachment offsets.
        for vertex in mesh.data.vertices:
            vertex.co += mesh.location
        mesh.location = (0, 0, 0)
        skin(mesh, "Spine0")
    frames = 54 if shark else 30
    amplitudes = [0.7, 1.5, 3.0, 5.0, 8.0, 13.0] if shark else [1.0, 2.0, 4.5, 7.0, 10.0, 17.0]
    for frame in range(frames + 1):
        phase = math.tau * frame / frames
        for i, amplitude in enumerate(amplitudes):
            bone = rig.pose.bones["Spine" + str(i)]
            bone.rotation_mode = "XYZ"
            bone.rotation_euler = (0, 0, math.radians(amplitude) * math.sin(phase - i * .65))
            bone.keyframe_insert("rotation_euler", frame=frame)
        for _, sign, _ in pectorals:
            bone = rig.pose.bones["Pectoral" + str(sign)]
            bone.rotation_mode = "XYZ"
            bone.rotation_euler = (math.radians(2 if shark else 7) * math.sin(2 * phase + sign * .4),
                                   0, math.radians(2) * math.sin(phase - .4))
            bone.keyframe_insert("rotation_euler", frame=frame)
    scene.frame_start, scene.frame_end = 0, frames
    linear_keys()
    return rig, frames
