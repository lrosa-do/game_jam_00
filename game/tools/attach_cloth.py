# Fits a loose cloth mesh (coat, skirt) onto a rigged character and binds it to the same skeleton
# with the body's weights, so it follows the hips and legs. The cloth is scaled so that its top
# sits just above the hips and its hem at the given height, and centred on the hips.
# Usage: blender -b -P tools/attach_cloth.py -- rigged.glb cloth.glb output.glb [--waist 0.08] [--hem 0.30] [--shots dir] [--check clip.glb]
import bpy, os, sys, math, mathutils

args = sys.argv[sys.argv.index('--') + 1:]
rigged, cloth_path, target = [os.path.abspath(a) for a in args[:3]]
def option(name, default):
    return type(default)(args[args.index(name) + 1]) if name in args else default
waist = option('--waist', 0.08)
hem = option('--hem', 0.30)
shots = os.path.abspath(args[args.index('--shots') + 1]) if '--shots' in args else None
check = os.path.abspath(args[args.index('--check') + 1]) if '--check' in args else None

def helper(o):
    return o.name.startswith('Icosphere') or any(c.name == 'glTF_not_exported' for c in o.users_collection)

def import_any(path):
    before = set(bpy.data.objects)
    if path.lower().endswith(('.glb', '.gltf')):
        bpy.ops.import_scene.gltf(filepath=path)
    else:
        bpy.ops.wm.fbx_import(filepath=path)
    return [o for o in bpy.data.objects if o not in before and not helper(o)]

def bounds(objects):
    lo = mathutils.Vector((1e9, 1e9, 1e9)); hi = mathutils.Vector((-1e9, -1e9, -1e9))
    for o in objects:
        for c in o.bound_box:
            p = o.matrix_world @ mathutils.Vector(c)
            lo = mathutils.Vector(map(min, lo, p)); hi = mathutils.Vector(map(max, hi, p))
    return lo, hi

bpy.ops.wm.read_factory_settings(use_empty=True)
body_objects = import_any(rigged)
arm = next(o for o in body_objects if o.type == 'ARMATURE')
body = max((o for o in body_objects if o.type == 'MESH'), key=lambda o: len(o.data.polygons))
cloth_objects = import_any(cloth_path)
cloth = max((o for o in cloth_objects if o.type == 'MESH'), key=lambda o: len(o.data.polygons))
for o in cloth_objects:
    if o is not cloth and o.type != 'MESH':
        bpy.data.objects.remove(o, do_unlink=True)
cloth.name = 'Coat'
cloth.data.name = 'Coat'
body.name = 'Body'

hips = next(b for b in arm.data.bones if b.name.endswith('Hips'))
hips_world = arm.matrix_world @ hips.head_local
body_lo, body_hi = bounds([body])
floor = body_lo.z
top_target = hips_world.z + waist
hem_target = floor + hem

bpy.ops.object.select_all(action='DESELECT')
cloth.select_set(True); bpy.context.view_layer.objects.active = cloth
bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
lo, hi = bounds([cloth])
scale = (top_target - hem_target) / max(hi.z - lo.z, 1e-6)
cloth.scale = (scale, scale, scale)
bpy.context.view_layer.update()
lo, hi = bounds([cloth])
center = (lo + hi) / 2
cloth.location = (cloth.location.x + hips_world.x - center.x, cloth.location.y + hips_world.y - center.y, cloth.location.z + top_target - hi.z)
bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
lo, hi = bounds([cloth])
print('cloth fitted: scale %.3f, top %.2f hem %.2f, hips %.2f' % (scale, hi.z, lo.z, hips_world.z))

for group in body.vertex_groups:
    if not cloth.vertex_groups.get(group.name):
        cloth.vertex_groups.new(name=group.name)
transfer = cloth.modifiers.new('weights', 'DATA_TRANSFER')
transfer.object = body
transfer.use_vert_data = True
transfer.data_types_verts = {'VGROUP_WEIGHTS'}
transfer.vert_mapping = 'POLYINTERP_NEAREST'
transfer.layers_vgroup_select_src = 'ALL'
transfer.layers_vgroup_select_dst = 'NAME'
bpy.ops.object.modifier_apply(modifier='weights')
matrix = cloth.matrix_world.copy()
cloth.parent = arm
cloth.matrix_world = matrix
skin = cloth.modifiers.new('Armature', 'ARMATURE')
skin.object = arm
weighted = sum(1 for v in cloth.data.vertices if any(g.weight > 0.0 for g in v.groups))
print('cloth vertices', len(cloth.data.vertices), 'weighted', weighted)

bpy.ops.object.select_all(action='DESELECT')
arm.select_set(True); body.select_set(True); cloth.select_set(True)
bpy.context.view_layer.objects.active = arm
bpy.ops.export_scene.gltf(filepath=target, export_format='GLB', use_selection=True, export_apply=False, export_yup=True, export_animations=False, export_skins=True, export_image_format='AUTO')
print('wrote', target, os.path.getsize(target), 'bytes')

if shots:
    scene = bpy.context.scene
    cam_data = bpy.data.cameras.new('cam'); cam_data.type = 'ORTHO'; cam_data.ortho_scale = 2.3
    cam = bpy.data.objects.new('cam', cam_data); scene.collection.objects.link(cam); scene.camera = cam
    scene.render.engine = 'BLENDER_WORKBENCH'; scene.display.shading.light = 'STUDIO'; scene.display.shading.color_type = 'TEXTURE'
    scene.render.resolution_x = 700; scene.render.resolution_y = 900; scene.render.image_settings.file_format = 'PNG'
    def shoot(name, yaw=12.0):
        cam.location = (hips_world.x + 1.5 * math.sin(math.radians(yaw)) * 5, hips_world.y - 7.0 * math.cos(math.radians(yaw)), 0.85)
        cam.rotation_euler = (math.pi / 2, 0.0, math.radians(yaw))
        scene.render.filepath = os.path.join(shots, name); bpy.ops.render.render(write_still=True)
    shoot('cloth_rest.png', 0.0)
    if check:
        before = set(bpy.data.actions)
        objects = import_any(check)
        action = next(a for a in bpy.data.actions if a not in before)
        for o in objects:
            bpy.data.objects.remove(o, do_unlink=True)
        if not arm.animation_data:
            arm.animation_data_create()
        arm.animation_data.action = action
        if hasattr(arm.animation_data, 'action_slot') and len(action.slots):
            arm.animation_data.action_slot = action.slots[0]
        frames = action.frame_range
        for frac, tag in ((0.3, 'a'), (0.7, 'b')):
            scene.frame_set(int(frames[0] + (frames[1] - frames[0]) * frac))
            shoot('cloth_check_%s.png' % tag, 25.0)
print('done')
