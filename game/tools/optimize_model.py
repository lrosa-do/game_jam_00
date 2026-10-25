# Decimates a model and shrinks its textures, for Meshy image-to-3D exports and other heavy files.
# Usage: blender -b -P tools/optimize_model.py -- input.glb output.glb [--tris 12000] [--texture 2048] [--shots dir]
import bpy, os, sys, math, mathutils

args = sys.argv[sys.argv.index('--') + 1:]
source, target = os.path.abspath(args[0]), os.path.abspath(args[1])
def option(name, default):
    return type(default)(args[args.index(name) + 1]) if name in args else default
target_tris = option('--tris', 12000)
texture_size = option('--texture', 2048)
shots = os.path.abspath(args[args.index('--shots') + 1]) if '--shots' in args else None

bpy.ops.wm.read_factory_settings(use_empty=True)
if source.lower().endswith(('.glb', '.gltf')):
    bpy.ops.import_scene.gltf(filepath=source)
else:
    bpy.ops.wm.fbx_import(filepath=source)
meshes = [o for o in bpy.data.objects if o.type == 'MESH' and o.name != 'Icosphere']
total = sum(sum(len(p.vertices) - 2 for p in o.data.polygons) for o in meshes)
print('input triangles', total, 'objects', [o.name for o in meshes])

def render(name):
    if not shots:
        return
    scene = bpy.context.scene
    if not scene.camera:
        cam_data = bpy.data.cameras.new('cam'); cam_data.type = 'ORTHO'
        cam = bpy.data.objects.new('cam', cam_data); scene.collection.objects.link(cam); scene.camera = cam
        scene.render.engine = 'BLENDER_WORKBENCH'; scene.display.shading.light = 'STUDIO'; scene.display.shading.color_type = 'TEXTURE'
        scene.render.resolution_x = 700; scene.render.resolution_y = 900; scene.render.image_settings.file_format = 'PNG'
    cam = scene.camera
    lo = mathutils.Vector((1e9, 1e9, 1e9)); hi = mathutils.Vector((-1e9, -1e9, -1e9))
    for o in meshes:
        for c in o.bound_box:
            p = o.matrix_world @ mathutils.Vector(c)
            lo = mathutils.Vector(map(min, lo, p)); hi = mathutils.Vector(map(max, hi, p))
    center = (lo + hi) / 2
    cam.data.ortho_scale = max(hi.z - lo.z, hi.x - lo.x) * 1.15
    cam.location = (center.x, center.y - 10.0, center.z); cam.rotation_euler = (math.pi / 2, 0.0, 0.0)
    scene.render.filepath = os.path.join(shots, name); bpy.ops.render.render(write_still=True)

render(os.path.splitext(os.path.basename(target))[0] + '_before.png')

for o in meshes:
    tris = sum(len(p.vertices) - 2 for p in o.data.polygons)
    share = tris / total if total else 1.0
    ratio = min(1.0, target_tris * share / max(tris, 1))
    if ratio < 1.0:
        mod = o.modifiers.new('dec', 'DECIMATE')
        mod.ratio = ratio
        bpy.ops.object.select_all(action='DESELECT')
        o.select_set(True); bpy.context.view_layer.objects.active = o
        bpy.ops.object.modifier_apply(modifier='dec')
    for polygon in o.data.polygons:
        polygon.use_smooth = True
for img in bpy.data.images:
    if img.size[0] > texture_size or img.size[1] > texture_size:
        scale = texture_size / max(img.size[0], img.size[1])
        img.scale(max(1, int(img.size[0] * scale)), max(1, int(img.size[1] * scale)))
after = sum(sum(len(p.vertices) - 2 for p in o.data.polygons) for o in meshes)
print('output triangles', after, 'textures', [(i.name, i.size[:]) for i in bpy.data.images][:6])

bpy.ops.object.select_all(action='SELECT')
bpy.ops.export_scene.gltf(filepath=target, export_format='GLB', use_selection=False, export_apply=True, export_yup=True, export_image_format='AUTO')
print('wrote', target, os.path.getsize(target), 'bytes')
render(os.path.splitext(os.path.basename(target))[0] + '_after.png')
