# Binds a Meshy character (mixamorig bone names, T-pose) to the Mixamo skeleton of a clip,
# so Mixamo clips play on it. The Mixamo bones keep their orientation and move to the
# Meshy joint positions; the Meshy vertex weights are kept by bone name.
# Usage: blender -b -P tools/rig_to_mixamo.py -- character.(fbx|glb) clip.(fbx|glb) output.glb [shots_dir] [check_clip ...]
# The character needs Mixamo bone names, with or without the mixamorig prefix (Meshy rig, or a
# Mesh2Motion export with Mixamo naming); the clip may use bare names. The output uses bare names.
import bpy, sys, os, math, mathutils

args = sys.argv[sys.argv.index('--') + 1:]
character, clip, output = [os.path.abspath(a) for a in args[:3]]
shots = os.path.abspath(args[3]) if len(args) > 3 else None
checks = [os.path.abspath(a) for a in args[4:]]

def helper(o):
    return o.name.startswith('Icosphere') or any(c.name == 'glTF_not_exported' for c in o.users_collection)

def import_fbx(path):
    before = set(bpy.data.objects)
    if path.lower().endswith(('.glb', '.gltf')):
        bpy.ops.import_scene.gltf(filepath=path)
    else:
        bpy.ops.wm.fbx_import(filepath=path)
    return [o for o in bpy.data.objects if o not in before and not helper(o)]

def hips_height(arm):
    bone = next((b for b in arm.data.bones if b.name.endswith('Hips')), None)
    if bone is None and len(arm.data.bones):
        bone = arm.data.bones[0]
    return (arm.matrix_world @ bone.head_local).z if bone else None

def normalize(objects):
    arm = next(o for o in objects if o.type == 'ARMATURE')
    height = hips_height(arm)
    factor = 1.0
    while height * factor > 3.0:
        factor *= 0.1
    while height * factor < 0.3:
        factor *= 10.0
    object_scale = arm.scale.x
    for o in objects:
        if o.parent is None:
            o.scale = (o.scale.x * factor, o.scale.y * factor, o.scale.z * factor)
    factor = object_scale * factor
    bpy.ops.object.select_all(action='DESELECT')
    for o in objects:
        o.select_set(True)
    bpy.context.view_layer.objects.active = arm
    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
    action = arm.animation_data.action if arm.animation_data else None
    if action is not None and abs(factor - 1.0) > 1e-6:
        for curve in location_curves(action):
            for key in curve.keyframe_points:
                key.co[1] *= factor
                key.handle_left[1] *= factor
                key.handle_right[1] *= factor
    return arm

def location_curves(action):
    curves = []
    if hasattr(action, 'fcurves'):
        curves = list(action.fcurves)
    else:
        for layer in action.layers:
            for strip in layer.strips:
                for bag in strip.channelbags:
                    curves.extend(bag.fcurves)
    return [c for c in curves if c.data_path.endswith('location')]

def reset_pose(arm):
    for bone in arm.pose.bones:
        bone.matrix_basis = mathutils.Matrix.Identity(4)

def strip_prefix(arm, objects):
    for bone in arm.data.bones:
        for prefix in ('mixamorig:', 'mixamorig'):
            if bone.name.startswith(prefix) and len(bone.name) > len(prefix):
                old_name = bone.name
                bone.name = bone.name[len(prefix):]
                for o in objects:
                    if o.type == 'MESH' and o.vertex_groups.get(old_name):
                        o.vertex_groups[old_name].name = bone.name
                break

bpy.ops.wm.read_factory_settings(use_empty=True)
clip_objects = import_fbx(clip)
rig = normalize(clip_objects)
strip_prefix(rig, [])
rig.animation_data_clear()
reset_pose(rig)
for o in clip_objects:
    if o.type == 'MESH':
        bpy.data.objects.remove(o, do_unlink=True)
rig.name = 'Armature'

character_objects = import_fbx(character)
source = normalize(character_objects)
strip_prefix(source, character_objects)
source.animation_data_clear()
reset_pose(source)
source.data.pose_position = 'REST'
mesh = max((o for o in character_objects if o.type == 'MESH'), key=lambda o: len(o.data.polygons))
bpy.context.view_layer.update()

def world_head(arm, name):
    bone = arm.data.bones.get(name)
    return (arm.matrix_world @ bone.head_local) if bone else None

for name in ('Hips', 'Head', 'LeftHand', 'LeftFoot', 'LeftArm'):
    a, b = world_head(rig, name), world_head(source, name)
    print('joint', name, 'mixamo', tuple(round(v, 3) for v in a) if a else None, 'meshy', tuple(round(v, 3) for v in b) if b else None)

source_bones = {b.name: b for b in source.data.bones}
bpy.ops.object.select_all(action='DESELECT')
rig.select_set(True)
bpy.context.view_layer.objects.active = rig
bpy.ops.object.mode_set(mode='EDIT')
edit = rig.data.edit_bones
original = {b.name: (b.head.copy(), b.tail.copy(), b.roll) for b in edit}
preferred = ('Spine', 'Spine1', 'Spine2', 'Neck', 'Head')
moved = 0
kept = []
def new_head(bone):
    if bone.name in source_bones:
        return world_head(source, bone.name)
    if bone.parent:
        head, tail, roll = original[bone.name]
        parent_head = original[bone.parent.name][0]
        return bone.parent.head + (head - parent_head)
    return bone.head
for bone in edit:
    bone.use_connect = False
for bone in edit:
    head, tail, roll = original[bone.name]
    direction = (tail - head).normalized()
    length = (tail - head).length
    bone.head = new_head(bone)
    children = [c for c in bone.children if c.name in source_bones]
    target = next((c for c in children if c.name in preferred), children[0] if children else None)
    if target is not None:
        length = max((world_head(source, target.name) - bone.head).length, 0.01)
    bone.tail = bone.head + direction * length
    bone.roll = roll
    if bone.name in source_bones:
        moved += 1
    else:
        kept.append(bone.name)
bpy.ops.object.mode_set(mode='OBJECT')
print('bones moved to meshy joints', moved, 'kept with mixamo offsets', kept)

rig_names = {b.name for b in rig.data.bones}
remapped = []
for group in list(mesh.vertex_groups):
    if group.name in rig_names:
        continue
    bone = source_bones.get(group.name)
    target = None
    while bone is not None:
        bone = bone.parent
        if bone is not None and bone.name in rig_names:
            target = bone.name
            break
    if target is None:
        continue
    dest = mesh.vertex_groups.get(target) or mesh.vertex_groups.new(name=target)
    for v in mesh.data.vertices:
        for g in v.groups:
            if g.group == group.index and g.weight > 0.0:
                dest.add([v.index], g.weight, 'ADD')
    remapped.append((group.name, target))
    mesh.vertex_groups.remove(group)
print('vertex groups remapped', remapped)

for m in list(mesh.modifiers):
    if m.type == 'ARMATURE':
        mesh.modifiers.remove(m)
matrix = mesh.matrix_world.copy()
mesh.parent = rig
mesh.matrix_world = matrix
modifier = mesh.modifiers.new('Armature', 'ARMATURE')
modifier.object = rig
mesh.name = 'Body'
bpy.data.objects.remove(source, do_unlink=True)

strip_prefix(rig, [mesh])
bpy.ops.object.select_all(action='DESELECT')
rig.select_set(True)
mesh.select_set(True)
bpy.context.view_layer.objects.active = rig
bpy.ops.export_scene.gltf(filepath=output, export_format='GLB', use_selection=True, export_apply=False, export_yup=True, export_animations=False, export_skins=True, export_image_format='AUTO')
print('wrote', output, os.path.getsize(output), 'bytes')

if shots:
    scene = bpy.context.scene
    cam_data = bpy.data.cameras.new('cam'); cam_data.type = 'ORTHO'; cam_data.ortho_scale = 2.3
    cam = bpy.data.objects.new('cam', cam_data); scene.collection.objects.link(cam); scene.camera = cam
    scene.render.engine = 'BLENDER_WORKBENCH'; scene.display.shading.light = 'STUDIO'; scene.display.shading.color_type = 'TEXTURE'
    scene.render.resolution_x = 700; scene.render.resolution_y = 900; scene.render.image_settings.file_format = 'PNG'
    def shoot(name):
        c = rig.matrix_world @ rig.data.bones['Hips'].head_local
        cam.location = (c.x + 1.5, c.y - 7.0, 0.85); cam.rotation_euler = (math.pi / 2, 0.0, math.radians(12))
        scene.render.filepath = os.path.join(shots, name); bpy.ops.render.render(write_still=True)
    shoot('rig_rest.png')
    for index, path in enumerate(checks):
        objects = import_fbx(path)
        arm = normalize(objects)
        action = arm.animation_data.action if arm.animation_data else None
        if action is None:
            continue
        curves = list(action.fcurves) if hasattr(action, 'fcurves') else [c for layer in action.layers for strip in layer.strips for bag in strip.channelbags for c in bag.fcurves]
        for curve in curves:
            curve.data_path = curve.data_path.replace('mixamorig:', '')
        if not rig.animation_data:
            rig.animation_data_create()
        rig.animation_data.action = action
        if hasattr(rig.animation_data, 'action_slot') and len(action.slots):
            rig.animation_data.action_slot = action.slots[0]
        for o in objects:
            bpy.data.objects.remove(o, do_unlink=True)
        hips = rig.pose.bones.get('Hips')
        frames = action.frame_range
        for frac, tag in ((0.3, 'a'), (0.7, 'b')):
            scene.frame_set(int(frames[0] + (frames[1] - frames[0]) * frac))
            print('   hips pose location', tuple(round(v, 3) for v in hips.location) if hips else None)
            shoot('rig_check_%d_%s.png' % (index, tag))
        print('checked', os.path.basename(path))
print('done')
