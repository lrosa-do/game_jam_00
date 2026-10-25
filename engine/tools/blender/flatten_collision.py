import argparse
import os
import sys

import bmesh
import bpy

#blender -b -P /media/projectos/projects/cpp/prisma3d/tools/blender/flatten_collision.py -- /media/projectos/projects/cpp/prisma-racer/assets/burnout/tracks/rk_01_v1.glb /media/projectos/projects/cpp/prisma-racer/assets/burnout/tracks/rk_01_v1_collision --max-height-above-ground 30


def parse():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    parser = argparse.ArgumentParser(prog="flatten_collision.py",
            description="Load a glb, drop materials and textures, join every mesh into one and "
                        "optionally delete everything above a height")
    parser.add_argument("input")
    parser.add_argument("output")
    parser.add_argument("--max-height", type=float, default=None,
            help="delete vertices higher than this (engine Y, the up axis)")
    parser.add_argument("--max-height-above-ground", type=float, default=None,
            help="delete vertices higher than this above the ground level (the height below "
                 "which only 1%% of the vertices are, so buried pieces do not count)")
    parser.add_argument("--name", default="collision", help="name of the joined mesh")
    return parser.parse_args(argv)


def main():
    args = parse()
    if not args.output.lower().endswith(".glb"):
        args.output = args.output + ".glb"
    if os.path.abspath(args.output) == os.path.abspath(args.input):
        print("FLATTEN: the output would overwrite the input, choose another file")
        sys.exit(1)
    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.ops.import_scene.gltf(filepath=args.input)

    for obj in list(bpy.data.objects):
        if obj.type != "MESH":
            bpy.data.objects.remove(obj, do_unlink=True)
    meshes = [o for o in bpy.data.objects if o.type == "MESH"]
    if not meshes:
        print("FLATTEN: no meshes in", args.input)
        sys.exit(1)

    for obj in meshes:
        if obj.data.users > 1:
            obj.data = obj.data.copy()
        matrix = obj.matrix_world.copy()
        obj.parent = None
        obj.matrix_world = matrix
        obj.modifiers.clear()
        obj.data.materials.clear()
        for slot in obj.material_slots:
            slot.material = None
    bpy.ops.object.select_all(action="DESELECT")
    for obj in meshes:
        obj.select_set(True)
    bpy.context.view_layer.objects.active = meshes[0]
    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
    if len(meshes) > 1:
        bpy.ops.object.join()
    joined = bpy.context.view_layer.objects.active
    joined.name = args.name
    joined.data.name = args.name

    mesh = joined.data
    for layer in list(mesh.uv_layers):
        mesh.uv_layers.remove(layer)
    for attribute in list(mesh.color_attributes):
        mesh.color_attributes.remove(attribute)

    bm = bmesh.new()
    bm.from_mesh(mesh)
    before = len(bm.faces)
    limit = args.max_height
    if args.max_height_above_ground is not None and bm.verts:
        heights = sorted(v.co.z for v in bm.verts)
        ground = heights[len(heights) // 100]
        limit = ground + args.max_height_above_ground
        print("FLATTEN: lowest %.3f, ground %.3f, highest %.3f, cut above %.3f"
              % (heights[0], ground, heights[-1], limit))
    if limit is not None:
        doomed = [v for v in bm.verts if v.co.z > limit]
        bmesh.ops.delete(bm, geom=doomed, context="VERTS")
    bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=0.0001)
    loose = [v for v in bm.verts if not v.link_faces]
    bmesh.ops.delete(bm, geom=loose, context="VERTS")
    after = len(bm.faces)
    if after == 0:
        bm.free()
        print("FLATTEN: nothing left after the cut (faces %d -> 0), nothing written" % before)
        sys.exit(1)
    bm.to_mesh(mesh)
    bm.free()

    for material in list(bpy.data.materials):
        bpy.data.materials.remove(material)
    for image in list(bpy.data.images):
        bpy.data.images.remove(image)

    bpy.ops.export_scene.gltf(filepath=args.output, export_format="GLB", use_selection=False,
            export_materials="NONE", export_texcoords=False, export_normals=True)
    limit_text = "none" if limit is None else "%.3f" % limit
    print("FLATTEN: %d meshes joined, faces %d -> %d, height limit %s, written %s"
          % (len(meshes), before, after, limit_text, args.output))


main()
