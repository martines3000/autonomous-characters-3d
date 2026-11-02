"""Shared Blender authoring: frame-sampled bird motion and bounded GLB export."""
import math
from pathlib import Path
import bpy
from mathutils import Vector


def bird_loop(scene, body, tail, wings, beat_frames, glide_frames):
    """Two eased strokes, a feathered recovery, then a quiet dihedral glide."""
    frames = 2 * beat_frames + glide_frames
    for f in range(frames + 1):
        active = f < 2 * beat_frames
        phase = 2 * math.pi * (f % beat_frames) / beat_frames if active else 0.0
        # A sine stroke inside a sin^2 window returns to glide with zero seam velocity.
        stroke = math.sin(phase) * math.sin(phase / 2) ** 2 if active else 0.0
        # Recovery occupies the second half; feathering peaks between its endpoints.
        recovery = math.sin(phase) ** 2 if active and phase > math.pi else 0.0
        for inner, outer, side in wings.values():
            inner.rotation_euler = (math.radians(-5 * stroke),
                                    side * math.radians(-9 + 80 * stroke), 0.0)
            outer.rotation_euler = (math.radians(24 * recovery),
                                    side * math.radians(-12 * stroke - 26 * recovery),
                                    side * math.radians(24 * recovery))
            inner.keyframe_insert("rotation_euler", frame=f)
            outer.keyframe_insert("rotation_euler", frame=f)
        body.location = (0.0, 0.0, 0.035 * stroke)
        body.rotation_euler = (math.radians(2 * stroke - 2 * recovery), 0.0, 0.0)
        body.keyframe_insert("location", frame=f)
        body.keyframe_insert("rotation_euler", frame=f)
        tail.rotation_euler = (math.radians(-5 * stroke + 8 * recovery), 0.0, 0.0)
        tail.keyframe_insert("rotation_euler", frame=f)
    scene.frame_start, scene.frame_end = 0, frames
    linear_keys()
    return frames


def linear_keys():
    # Dense analytic samples already supply easing; CUBICSPLINE is unsupported.
    for obj in bpy.data.objects:
        if obj.animation_data and obj.animation_data.action:
            for curve in obj.animation_data.action.fcurves:
                for key in curve.keyframe_points:
                    key.interpolation = "LINEAR"


def export_model(out, active, clip):
    Path(out).parent.mkdir(parents=True, exist_ok=True)
    bpy.ops.object.select_all(action="SELECT")
    bpy.context.view_layer.objects.active = active
    bpy.ops.export_scene.gltf(filepath=out, export_format="GLB", use_selection=True,
        use_active_scene=True, export_animations=True, export_animation_mode="ACTIVE_ACTIONS",
        export_nla_strips_merged_animation_name=clip,
        export_frame_range=True, export_force_sampling=True, export_optimize_animation_size=False,
        export_yup=True, export_apply=False, export_texcoords=False, export_materials="EXPORT")
    tris = sum(len(p.vertices) - 2 for o in bpy.data.objects if o.type == "MESH" for p in o.data.polygons)
    assert tris <= 400, tris
    print("PASS exported", out, "triangles", tris)


def previews(scene, directory, shots):
    Path(directory).mkdir(parents=True, exist_ok=True)
    cam = bpy.data.objects.new("Cam", bpy.data.cameras.new("Cam"))
    scene.collection.objects.link(cam)
    scene.camera = cam
    scene.render.engine = "BLENDER_WORKBENCH"
    scene.display.shading.light = "STUDIO"
    scene.display.shading.color_type = "MATERIAL"
    scene.render.resolution_x, scene.render.resolution_y = 800, 600
    scene.world = bpy.data.worlds.new("Preview")
    scene.world.color = (0.45, 0.52, 0.6)
    for name, location, frame in shots:
        scene.frame_set(frame)
        cam.location = location
        cam.rotation_euler = (-Vector(location)).to_track_quat("-Z", "Y").to_euler()
        scene.render.filepath = str(Path(directory) / (name + ".png"))
        bpy.ops.render.render(write_still=True)
