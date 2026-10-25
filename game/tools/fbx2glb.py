# Converts an FBX (Mixamo, Meshy) to GLB with Blender in the background.
# Usage: blender -b -P tools/fbx2glb.py -- input.fbx output.glb [--strip-mesh]
# --strip-mesh keeps only the skeleton and the animation (for Mixamo clips), and drops the
# per-bone translation and scale channels (only the hips keep their translation), so the clip
# plays on a skeleton with other bone lengths.
import bpy, os, sys, glob

args = sys.argv[sys.argv.index('--') + 1:]
source, target = os.path.abspath(args[0]), os.path.abspath(args[1])
strip_mesh = '--strip-mesh' in args

bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.wm.fbx_import(filepath=source)

folder = os.path.dirname(source)

def find_texture(image):
    path = bpy.path.abspath(image.filepath) if image.filepath else ''
    if path and os.path.exists(path):
        return path
    base = os.path.basename(path or image.name)
    if '_texture_' in base:
        suffix = base.split('_texture_', 1)[1]
        hits = glob.glob(os.path.join(folder, '*_texture_' + suffix))
        if hits:
            return hits[0]
    hits = glob.glob(os.path.join(folder, base))
    return hits[0] if hits else None

if strip_mesh:
    for o in [o for o in bpy.data.objects if o.type == 'MESH']:
        bpy.data.objects.remove(o, do_unlink=True)
else:
    for image in bpy.data.images:
        if image.packed_file:
            continue
        found = find_texture(image)
        if found:
            image.filepath = found
            image.reload()
            print('texture', image.name, '->', os.path.basename(found))
        else:
            print('texture missing', image.name, image.filepath)
    for material in bpy.data.materials:
        if not material.use_nodes:
            continue
        nodes = material.node_tree.nodes
        links = material.node_tree.links
        principled = next((n for n in nodes if n.type == 'BSDF_PRINCIPLED'), None)
        if not principled:
            continue
        base = next((n for n in nodes if n.type == 'TEX_IMAGE' and n.image), None)
        if not base or not base.image:
            continue
        base_path = bpy.path.abspath(base.image.filepath)
        stem = base_path[:-4] if base_path.lower().endswith('.png') else base_path
        def add_map(suffix, socket, non_color=True, normal=False):
            if principled.inputs[socket].is_linked:
                return
            path = stem + '_' + suffix + '.png'
            if not os.path.exists(path):
                return
            image = bpy.data.images.load(path)
            if non_color:
                image.colorspace_settings.name = 'Non-Color'
            tex = nodes.new('ShaderNodeTexImage')
            tex.image = image
            if normal:
                bump = nodes.new('ShaderNodeNormalMap')
                links.new(tex.outputs['Color'], bump.inputs['Color'])
                links.new(bump.outputs['Normal'], principled.inputs[socket])
            else:
                links.new(tex.outputs['Color'], principled.inputs[socket])
            print('material', material.name, suffix, '->', socket)
        add_map('metallic', 'Metallic')
        add_map('roughness', 'Roughness')
        add_map('normal', 'Normal', normal=True)

def hips_height():
    for o in bpy.data.objects:
        if o.type != 'ARMATURE':
            continue
        bone = o.data.bones.get('mixamorig:Hips') or (o.data.bones[0] if len(o.data.bones) else None)
        if bone:
            return (o.matrix_world @ bone.head_local).z
    return None

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

height = hips_height()
factor = 1.0
if height and height > 0.0:
    while height * factor > 3.0:
        factor *= 0.1
    while height * factor < 0.3:
        factor *= 10.0
roots = [o for o in bpy.data.objects if o.parent is None]
applied = {}
for o in roots:
    applied[o.name] = o.scale.x * factor
    o.scale = (o.scale.x * factor, o.scale.y * factor, o.scale.z * factor)
bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
for o in bpy.data.objects:
    if o.type != 'ARMATURE':
        continue
    root = o
    while root.parent:
        root = root.parent
    total = applied.get(root.name, factor)
    if o.animation_data and o.animation_data.action and abs(total - 1.0) > 1e-6:
        for curve in location_curves(o.animation_data.action):
            for key in curve.keyframe_points:
                key.co[1] *= total
                key.handle_left[1] *= total
                key.handle_right[1] *= total
print('hips height', height, 'factor', factor, 'now', hips_height())

stem = os.path.splitext(os.path.basename(target))[0]
def bare(name):
    for prefix in ('mixamorig:', 'mixamorig'):
        if name.startswith(prefix) and len(name) > len(prefix):
            return name[len(prefix):]
    return name

for o in bpy.data.objects:
    if o.type == 'ARMATURE':
        for bone in o.data.bones:
            if bare(bone.name) != bone.name:
                old_name = bone.name
                bone.name = bare(bone.name)
                for mesh in bpy.data.objects:
                    if mesh.type == 'MESH' and mesh.vertex_groups.get(old_name):
                        mesh.vertex_groups[old_name].name = bone.name
for index, action in enumerate(bpy.data.actions):
    action.name = stem if index == 0 else stem + '_' + str(index + 1)
bpy.ops.object.select_all(action='SELECT')
bpy.ops.export_scene.gltf(filepath=target, export_format='GLB', use_selection=False, export_apply=True, export_yup=True, export_animations=True, export_skins=True, export_image_format='AUTO')

def strip_bone_channels(path):
    import json, struct
    data = open(path, 'rb').read()
    magic, version, length = struct.unpack_from('<III', data, 0)
    json_length, json_type = struct.unpack_from('<II', data, 12)
    document = json.loads(data[20:20 + json_length])
    rest = data[20 + json_length:]
    nodes = document['nodes']
    for animation in document.get('animations', []):
        kept = []
        for channel in animation['channels']:
            path_name = channel['target']['path']
            bone = nodes[channel['target']['node']].get('name', '')
            if path_name == 'scale':
                continue
            if path_name == 'translation' and not bone.endswith('Hips'):
                continue
            kept.append(channel)
        used = sorted({channel['sampler'] for channel in kept})
        remap = {old: new for new, old in enumerate(used)}
        animation['samplers'] = [animation['samplers'][index] for index in used]
        for channel in kept:
            channel['sampler'] = remap[channel['sampler']]
        animation['channels'] = kept
    text = json.dumps(document, separators=(',', ':')).encode('utf-8')
    text += b' ' * ((4 - len(text) % 4) % 4)
    header = struct.pack('<III', magic, version, 20 + len(text) + len(rest))
    open(path, 'wb').write(header + struct.pack('<II', len(text), json_type) + text + rest)

if strip_mesh:
    strip_bone_channels(target)
print('wrote', target, os.path.getsize(target), 'bytes')
