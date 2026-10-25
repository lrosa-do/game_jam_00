# Retargets Max Payne 2 clips (Bip01 skeleton) onto a Mixamo skeleton (bare bone names).
# The Mixamo skeleton is first posed like the MP2 rest pose (A-pose: limb directions matched), then
# every frame each mapped bone gets the world rotation of its MP2 bone relative to the MP2 rest.
# The hips follow the MP2 pelvis scaled by the hip heights; the horizontal drift of the clip is
# removed so loops play in place (--keep-root keeps it).
# The clips come from tools/extract_clips.py with --keep-channels (they need a skin).
# --upper writes an upper-body layer instead (the Spine bone and below it), named <clip>_upper.
# Usage: blender -b -P tools/retarget_mp2.py -- target.glb out_dir clip.glb [clip.glb ...] [--keep-root] [--upper]
import bpy, sys, os, subprocess
from mathutils import Matrix, Quaternion, Vector

args = sys.argv[sys.argv.index('--') + 1:]
keep_root = '--keep-root' in args
upper = '--upper' in args
args = [a for a in args if not a.startswith('--')]
target, out_dir = os.path.abspath(args[0]), os.path.abspath(args[1])
clips = [os.path.abspath(a) for a in args[2:]]
tools = os.path.dirname(os.path.abspath(__file__))
os.makedirs(out_dir, exist_ok=True)

FPS = 30
SIDES = {'L': 'Left', 'R': 'Right'}
MAP = {'Pelvis': 'Hips', 'Spine': 'Spine', 'Spine1': 'Spine2', 'Neck': 'Neck', 'Head': 'Head'}
FINGERS = ['Thumb', 'Index', 'Middle', 'Ring', 'Pinky']
for s, side in SIDES.items():
    MAP.update({s + ' Clavicle': side + 'Shoulder', s + ' UpperArm': side + 'Arm',
                s + ' Forearm': side + 'ForeArm', s + ' Hand': side + 'Hand',
                s + ' Thigh': side + 'UpLeg', s + ' Calf': side + 'Leg', s + ' Foot': side + 'Foot',
                s + ' Toe0': side + 'ToeBase'})
    for f, finger in enumerate(FINGERS):
        for k, suffix in enumerate(['', '1', '2']):
            MAP['%s Finger%d%s' % (s, f, suffix)] = '%sHand%s%d' % (side, finger, k + 1)
MATCH = {'Neck': 'Head'}
for side in SIDES.values():
    MATCH.update({side + 'Shoulder': side + 'Arm', side + 'Arm': side + 'ForeArm',
                  side + 'ForeArm': side + 'Hand', side + 'Hand': side + 'HandMiddle1',
                  side + 'UpLeg': side + 'Leg', side + 'Leg': side + 'Foot', side + 'Foot': side + 'ToeBase'})
    for finger in FINGERS:
        MATCH['%sHand%s1' % (side, finger)] = '%sHand%s2' % (side, finger)
        MATCH['%sHand%s2' % (side, finger)] = '%sHand%s3' % (side, finger)

def helper(o):
    return o.name.startswith('Icosphere') or any(c.name == 'glTF_not_exported' for c in o.users_collection)

def import_glb(path):
    before = set(bpy.data.objects)
    bpy.ops.import_scene.gltf(filepath=path)
    return [o for o in bpy.data.objects if o not in before]

def ordered(arm):
    depth = lambda b: 0 if b.parent is None else 1 + depth(b.parent)
    return sorted(arm.data.bones, key=depth)

bpy.ops.wm.read_factory_settings(use_empty=True)
scene = bpy.context.scene
scene.render.fps = FPS
for o in import_glb(target):
    if o.type != 'ARMATURE':
        bpy.data.objects.remove(o, do_unlink=True)
tgt = next(o for o in bpy.data.objects if o.type == 'ARMATURE')
if tgt.animation_data:
    tgt.animation_data.action = None
for action in list(bpy.data.actions):
    bpy.data.actions.remove(action)

order = [b.name for b in ordered(tgt)]
R = {b.name: b.matrix_local.copy() for b in tgt.data.bones}
parent = {b.name: (b.parent.name if b.parent else None) for b in tgt.data.bones}
root = next(n for n in order if parent[n] is None)

def base_of(name, posed):
    p = parent[name]
    return posed[p] @ R[p].inverted() @ R[name] if p else R[name].copy()

def retarget(clip):
    objects = [o for o in import_glb(clip) if not helper(o)]
    src = next(o for o in objects if o.type == 'ARMATURE')
    action = src.animation_data.action
    name = os.path.splitext(os.path.basename(clip))[0] + ('_upper' if upper else '')
    prefix = 'Bip01 '
    SR = {b.name[len(prefix):]: b.matrix_local.copy() for b in src.data.bones if b.name.startswith(prefix)}
    t2s = {t: s for s, t in MAP.items() if s in SR and t in R}

    matched = {}
    basis = {}
    for n in order:
        base = base_of(n, matched)
        world = base.copy()
        child = MATCH.get(n)
        if n in t2s and child in t2s:
            head = world.translation
            child_head = (world @ R[n].inverted() @ R[child]).translation
            wanted = SR[t2s[child]].translation - SR[t2s[n]].translation
            turn = (child_head - head).rotation_difference(wanted)
            world = Matrix.Translation(head) @ (turn @ world.to_quaternion()).to_matrix().to_4x4()
        matched[n] = world
        basis[n] = (base.inverted() @ world).to_quaternion()
    rest_world = {n: matched[n].to_quaternion() for n in order}

    start, end = int(action.frame_range[0]), int(action.frame_range[1])
    frames = list(range(start, end + 1))
    pelvis = []
    poses = []
    for f in frames:
        scene.frame_set(f)
        pose = {}
        for s in set(t2s.values()):
            pose[s] = src.pose.bones[prefix + s].matrix.to_quaternion()
        poses.append(pose)
        pelvis.append(src.pose.bones[prefix + 'Pelvis'].matrix.translation.copy())
    scale = R[root].translation.z / SR['Pelvis'].translation.z
    drift = pelvis[-1] - pelvis[0]

    tgt.animation_data_create()
    tgt.animation_data.action = None
    previous = {}
    for i, f in enumerate(frames):
        posed = {}
        for n in order:
            base = base_of(n, posed)
            if n in t2s:
                s = t2s[n]
                q = poses[i][s] @ SR[s].to_quaternion().inverted() @ rest_world[n]
                head = base.translation
                if n == root:
                    offset = pelvis[i] - SR['Pelvis'].translation
                    if not keep_root and len(frames) > 1:
                        t = i / (len(frames) - 1)
                        offset = offset - Vector((drift.x * t, drift.y * t, 0.0))
                    head = R[root].translation + offset * scale
                world = Matrix.Translation(head) @ q.to_matrix().to_4x4()
                local = base.inverted() @ world
                rotation = local.to_quaternion()
            else:
                rotation = basis[n]
                world = base @ rotation.to_matrix().to_4x4()
                local = None
            if n in previous and previous[n].dot(rotation) < 0.0:
                rotation.negate()
            previous[n] = rotation
            posed[n] = world
            bone = tgt.pose.bones[n]
            bone.rotation_mode = 'QUATERNION'
            bone.rotation_quaternion = rotation
            bone.keyframe_insert('rotation_quaternion', frame=f - start)
            if n == root:
                bone.location = local.translation
                bone.keyframe_insert('location', frame=f - start)
    for o in objects:
        bpy.data.objects.remove(o, do_unlink=True)
    bpy.data.actions.remove(action)
    tgt.animation_data.action.name = name
    tmp = os.path.join(out_dir, '_' + name + '.glb')
    bpy.ops.object.select_all(action='DESELECT')
    tgt.select_set(True)
    bpy.context.view_layer.objects.active = tgt
    bpy.ops.export_scene.gltf(filepath=tmp, export_format='GLB', use_selection=True, export_yup=True,
                              export_animations=True, export_animation_mode='ACTIONS', export_skins=True)
    subprocess.run(['python3',
                    os.path.join(tools, 'extract_clips.py'), tmp, out_dir, '--root=' + root] + (['--subtree=Spine'] if upper else []), check=True)
    os.remove(tmp)
    done = tgt.animation_data.action
    tgt.animation_data.action = None
    bpy.data.actions.remove(done)
    for bone in tgt.pose.bones:
        bone.rotation_quaternion = Quaternion()
        bone.location = Vector()
    print('retarget:', name, len(frames), 'frames')

for clip in clips:
    retarget(clip)
