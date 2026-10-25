import os
import shutil
import tempfile

import bpy

from .gltf_builder import GltfBuilder, write_gltf
from .import_pmesh import ImportResult, frame_rate, run_gltf_import
from .model import ConvertError
from .pmesh_reader import PMeshModel, read_animations, read_skeleton, resolve_bone
from .pmesh_writer import SKELETON_EXTENSION

MAX_LISTED_BONES = 8
STEM = "animations"


def find_armature(context):
    selected = [o for o in context.selected_objects if o.type == "ARMATURE"]
    if context.object in selected:
        return context.object
    if selected:
        return selected[0]
    return None


def read_file(path):
    try:
        with open(path, "rb") as handle:
            data = handle.read()
    except OSError:
        raise ConvertError("%s: cannot read file" % os.path.basename(path))
    try:
        return read_animations(data)
    except ConvertError as error:
        raise ConvertError("%s: %s" % (os.path.basename(path), error))


class SkeletonCatalog:
    def __init__(self, directory, warnings):
        self.entries = []
        for name in sorted(os.listdir(directory)):
            if not name.lower().endswith("." + SKELETON_EXTENSION):
                continue
            try:
                with open(os.path.join(directory, name), "rb") as handle:
                    skeleton = read_skeleton(handle.read())
            except (OSError, ConvertError) as error:
                warnings.append("skeleton '%s' skipped: %s" % (name, error))
                continue
            self.entries.append((name, skeleton, {bone.name for bone in skeleton.bones}))

    def choose(self, panim_name, bone_names):
        best = None
        best_prefix = -1
        for name, skeleton, names in self.entries:
            if not bone_names <= names:
                continue
            prefix = len(os.path.commonprefix([panim_name, name]))
            if prefix > best_prefix:
                best = (name, skeleton)
                best_prefix = prefix
        return best


class Group:
    def __init__(self, skeleton_name, skeleton):
        self.skeleton_name = skeleton_name
        self.skeleton = skeleton
        self.clips = []


def plan_groups(paths, warnings):
    groups = {}
    catalogs = {}
    for path in paths:
        panim_name = os.path.basename(path)
        directory = os.path.dirname(os.path.abspath(path))
        if directory not in catalogs:
            catalogs[directory] = SkeletonCatalog(directory, warnings)
        for clip in read_file(path):
            names = {track.bone_name for track in clip.tracks if track.bone >= 0}
            if not names:
                warnings.append("%s: clip '%s' has no bone tracks, skipped" % (panim_name, clip.name))
                continue
            chosen = catalogs[directory].choose(panim_name, names)
            if chosen is None:
                raise ConvertError("%s: no .pskel next to the clip has its bones" % panim_name)
            key = (directory, chosen[0])
            if key not in groups:
                groups[key] = Group(chosen[0], chosen[1])
            for track in clip.tracks:
                if track.bone >= 0:
                    track.bone = resolve_bone(chosen[1], track.bone, track.bone_name)
            clip.skeleton = 0
            groups[key].clips.append(clip)
    return list(groups.values())


def missing_bones(group, armature):
    present = set(armature.data.bones.keys())
    missing = []
    for clip in group.clips:
        names = []
        for track in clip.tracks:
            if track.bone >= 0:
                name = group.skeleton.bones[track.bone].name
                if name not in present and name not in names:
                    names.append(name)
        if names:
            listed = ", ".join(names[:MAX_LISTED_BONES])
            if len(names) > MAX_LISTED_BONES:
                listed += ", ..."
            missing.append("clip '%s': bones not in armature '%s': %s" % (clip.name, armature.name, listed))
    return missing


def stash(armature, name, action, slot, frame):
    data = armature.animation_data or armature.animation_data_create()
    track = data.nla_tracks.new(prev=None)
    track.name = name
    strip = track.strips.new(action.name, frame, action)
    if slot is not None:
        strip.action_slot = slot
    strip.action_frame_start = action.frame_range[0]
    strip.action_frame_end = action.frame_range[1]
    track.lock = True
    track.mute = True


def import_group(context, group, armature, result):
    model = PMeshModel()
    model.skeletons = [group.skeleton]
    model.animations = group.clips
    builder = GltfBuilder(model, None, "", STEM, frame_rate())
    document, binary = builder.build()
    result.warnings.extend(builder.warnings)
    if not document.get("animations"):
        return 0

    objects_before = {o.name for o in bpy.data.objects}
    armatures_before = {a.name for a in bpy.data.armatures}
    temporary = tempfile.mkdtemp(prefix="prisma3d_")
    try:
        run_gltf_import(write_gltf(temporary, STEM, document, binary))
    finally:
        shutil.rmtree(temporary, ignore_errors=True)

    created = [o for o in bpy.data.objects if o.name not in objects_before]
    try:
        return stash_created(context, created, armature, result)
    finally:
        for obj in created:
            bpy.data.objects.remove(obj, do_unlink=True)
        for data in [a for a in bpy.data.armatures if a.name not in armatures_before]:
            if data.users == 0:
                bpy.data.armatures.remove(data)


def stash_created(context, created, armature, result):
    tracks = []
    for holder in created:
        if holder.type != "ARMATURE" or not holder.animation_data:
            continue
        for track in holder.animation_data.nla_tracks:
            if track.strips:
                strip = track.strips[0]
                tracks.append((track.name, strip.action, getattr(strip, "action_slot", None)))
    if not tracks:
        raise ConvertError("the glTF import created no animations")
    for name, action, slot in tracks:
        if action.name != name:
            if name not in bpy.data.actions:
                action.name = name
            else:
                result.warnings.append(
                    "action '%s' already exists, the clip was imported as '%s'" % (name, action.name)
                )
        action.use_fake_user = True
        if slot is not None:
            slot.name_display = armature.name
        stash(armature, name, action, slot, context.scene.frame_start)
    return len(tracks)


def import_animations(context, paths, armature):
    result = ImportResult()
    groups = plan_groups(paths, result.warnings)
    if not groups:
        raise ConvertError("there is nothing to import")
    for group in groups:
        result.warnings.extend(missing_bones(group, armature))

    selected = {o.name for o in context.selected_objects}
    active = context.view_layer.objects.active
    try:
        for group in groups:
            result.count += import_group(context, group, armature, result)
    finally:
        for obj in context.view_layer.objects:
            obj.select_set(obj.name in selected)
        context.view_layer.objects.active = active
    return result
