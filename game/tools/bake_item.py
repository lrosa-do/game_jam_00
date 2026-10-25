# Turns an item GLB that hangs from a holder bone (the Serious Sam 2 pickups) into a static model:
# the mesh in its rest pose without the skeleton and the clips, centred on X/Z with its base at
# y = 0, scaled so its largest side is SIZE metres.
# Usage: blender -b -P tools/bake_item.py -- input.glb output.glb SIZE
import bpy, sys
from mathutils import Vector

args = sys.argv[sys.argv.index('--') + 1:]
source, target, size = args[0], args[1], float(args[2])

bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.import_scene.gltf(filepath=source)

depsgraph = bpy.context.evaluated_depsgraph_get()
meshes = [o for o in bpy.context.scene.objects if o.type == 'MESH' and len(o.data.polygons) > 0
          and not o.name.startswith('Icosphere')]
if not meshes:
    raise SystemExit('no mesh in ' + source)

baked = []
for obj in meshes:
    evaluated = obj.evaluated_get(depsgraph)
    data = bpy.data.meshes.new_from_object(evaluated, depsgraph=depsgraph)
    data.transform(obj.matrix_world)
    copy = bpy.data.objects.new(obj.name, data)
    bpy.context.scene.collection.objects.link(copy)
    baked.append(copy)

for obj in list(bpy.context.scene.objects):
    if obj not in baked:
        bpy.data.objects.remove(obj, do_unlink=True)
for action in list(bpy.data.actions):
    bpy.data.actions.remove(action)

points = [v.co for obj in baked for v in obj.data.vertices]
low = Vector((min(p.x for p in points), min(p.y for p in points), min(p.z for p in points)))
high = Vector((max(p.x for p in points), max(p.y for p in points), max(p.z for p in points)))
extent = high - low
scale = size / max(extent.x, extent.y, extent.z)
# Blender is Z up: the base is the lowest Z.
offset = Vector((-(low.x + high.x) * 0.5, -(low.y + high.y) * 0.5, -low.z))
for obj in baked:
    for v in obj.data.vertices:
        v.co = (v.co + offset) * scale
    obj.data.update()

bpy.ops.export_scene.gltf(filepath=target, export_format='GLB', export_animations=False,
                          export_skins=False, export_yup=True)
print('baked %s: %d meshes, %.3f x %.3f x %.3f m' % (target, len(baked), extent.x * scale,
      extent.y * scale, extent.z * scale))
