import os
import struct

import numpy as np

from .model import (
    TOPOLOGY_TRIANGLES,
    ConvertError,
)


def fourcc(text):
    return int.from_bytes(text.encode("ascii"), "little")


MAGIC_MESH = fourcc("PMSH")
MAGIC_SKELETON = fourcc("PSKL")
MAGIC_ANIMATION = fourcc("PANM")

VERSION_MAJOR = 1
VERSION_MINOR = 0
VERSION = (VERSION_MAJOR << 16) | VERSION_MINOR

MESH_EXTENSION = "pmesh"
SKELETON_EXTENSION = "pskel"
ANIMATION_EXTENSION = "panim"

CHUNK_MATERIAL_SLOTS = fourcc("MATS")
CHUNK_VERTEX_BUFFER = fourcc("VBUF")
CHUNK_INDEX_BUFFER = fourcc("IBUF")
CHUNK_SUBMESHES = fourcc("SUBM")
CHUNK_SKIN = fourcc("SKIN")
CHUNK_NODES = fourcc("NODE")
CHUNK_CAMERAS = fourcc("CAMS")
CHUNK_SKELETON_LINK = fourcc("SLNK")
CHUNK_ANIMATION_LINK = fourcc("ALNK")
CHUNK_BONES = fourcc("BONE")
CHUNK_CLIP = fourcc("CLIP")
CHUNK_END = fourcc("ENDC")

FLAG_LITTLE_ENDIAN = 1 << 0
FLAG_SKINNED = 1 << 1
HEADER_ATTRIB_SHIFT = 4

HAS_NORMAL = 1 << 0
HAS_TANGENT = 1 << 1
HAS_UV = 1 << 2
HAS_UV2 = 1 << 3
HAS_COLOR = 1 << 4

INDEX16 = 1 << 0
SUBMESH_SKINNED = 1 << 0
SUBMESH_HAS_BOUNDS = 1 << 1

NO_SLOT = 0xFFFFFFFF
MAX_TOPOLOGY = 9
MAX_VERTICES = 100000000
MAX_INDICES = 300000000
MAX_SURFACES = 1 << 20
MAX_MESHES = 1 << 20
MAX_SUBMESHES = 1 << 20
MAX_NODES = 1 << 24
MAX_BONES = 65535
MAX_LINKS = 65536


class Writer:
    def __init__(self):
        self.out = bytearray()

    def header(self, magic, flags):
        self.u32(magic)
        self.u32(VERSION)
        self.u32(flags | FLAG_LITTLE_ENDIAN)

    def begin_chunk(self, chunk_id):
        self.u32(chunk_id)
        marker = len(self.out)
        self.u64(0)
        return marker

    def end_chunk(self, marker):
        size = len(self.out) - marker - 8
        self.out[marker:marker + 8] = struct.pack("<Q", size)

    def u8(self, value):
        self.out += struct.pack("<B", value)

    def u16(self, value):
        self.out += struct.pack("<H", value)

    def u32(self, value):
        self.out += struct.pack("<I", value)

    def s32(self, value):
        self.out += struct.pack("<i", value)

    def u64(self, value):
        self.out += struct.pack("<Q", value)

    def f32(self, value):
        self.out += struct.pack("<f", float(value))

    def bytes(self, data):
        self.out += data

    def string(self, value):
        encoded = value.encode("utf-8", errors="surrogateescape")
        self.u32(len(encoded))
        self.out += encoded

    def floats(self, values):
        self.out += np.ascontiguousarray(values, dtype="<f4").tobytes()


def bad(message):
    raise ConvertError(message)


def file_stem(path):
    begin = 0
    for i, c in enumerate(path):
        if c == "/" or c == "\\":
            begin = i + 1
    end = len(path)
    for i in range(len(path), begin, -1):
        if path[i - 1] == ".":
            end = i - 1
            break
    return path[begin:end]


def directory_part(path):
    slash = -1
    for i, c in enumerate(path):
        if c == "/" or c == "\\":
            slash = i
    if slash == -1:
        return ""
    return path[:slash + 1]


def clean_name(name):
    return "".join("_" if c in '"\n\r' else c for c in name)


def clip_file_name(stem, clip, index, taken):
    safe = bytearray()
    for c in clip.encode("utf-8", errors="surrogateescape"):
        keep = (
            (0x61 <= c <= 0x7A)
            or (0x41 <= c <= 0x5A)
            or (0x30 <= c <= 0x39)
            or c == 0x2D
            or c == 0x5F
        )
        safe.append(c if keep else 0x5F)
    safe_text = safe.decode("ascii")
    if not safe_text:
        safe_text = "clip_%d" % index
    extension = "." + ANIMATION_EXTENSION
    name = stem + "_" + safe_text + extension
    for existing in taken:
        if existing == name:
            return "%s_%s_%d%s" % (stem, safe_text, index, extension)
    return name


def indices_fit16(indices):
    return bool(np.all(indices <= 0xFFFF)) if len(indices) else True


def write_surface(w, surface_id, surface):
    n = len(surface.positions)
    if n == 0 or n > MAX_VERTICES:
        bad("surface %d has no vertices or too many" % surface_id)
    if surface.topology > MAX_TOPOLOGY:
        bad("surface %d has an unknown topology" % surface_id)
    has_normal = surface.normals is not None and len(surface.normals) > 0
    has_tangent = surface.tangents is not None and len(surface.tangents) > 0
    has_uv = surface.uvs is not None and len(surface.uvs) > 0
    has_uv2 = surface.uvs2 is not None and len(surface.uvs2) > 0
    has_color = surface.colors is not None and len(surface.colors) > 0
    if (
        (has_normal and len(surface.normals) != n)
        or (has_tangent and len(surface.tangents) != n)
        or (has_uv and len(surface.uvs) != n)
        or (has_uv2 and len(surface.uvs2) != n)
        or (has_color and len(surface.colors) != n)
    ):
        bad("surface %d has an attribute with the wrong count" % surface_id)
    has_joints = surface.joints is not None and len(surface.joints) > 0
    has_weights = surface.weights is not None and len(surface.weights) > 0
    if has_joints != has_weights or (has_joints and (len(surface.joints) != n or len(surface.weights) != n)):
        bad("surface %d has inconsistent skin data" % surface_id)
    indices = surface.indices if surface.indices is not None else np.zeros(0, dtype=np.uint32)
    if len(indices) > MAX_INDICES or (len(indices) and int(indices.max()) >= n):
        bad("surface %d has an index out of range" % surface_id)

    attribs = 0
    columns = [surface.positions]
    if has_normal:
        attribs |= HAS_NORMAL
        columns.append(surface.normals)
    if has_tangent:
        attribs |= HAS_TANGENT
        columns.append(surface.tangents)
    if has_uv:
        attribs |= HAS_UV
        columns.append(surface.uvs)
    if has_uv2:
        attribs |= HAS_UV2
        columns.append(surface.uvs2)
    if has_color:
        attribs |= HAS_COLOR
        columns.append(surface.colors)

    marker = w.begin_chunk(CHUNK_VERTEX_BUFFER)
    w.u32(surface_id)
    w.u32(attribs)
    w.u32(n)
    w.u32(surface.topology)
    w.floats(np.concatenate([np.asarray(c, dtype=np.float32) for c in columns], axis=1))
    w.end_chunk(marker)

    if len(indices):
        index16 = indices_fit16(indices)
        marker = w.begin_chunk(CHUNK_INDEX_BUFFER)
        w.u32(surface_id)
        w.u32(INDEX16 if index16 else 0)
        w.u32(len(indices))
        if index16:
            w.bytes(np.ascontiguousarray(indices, dtype="<u2").tobytes())
        else:
            w.bytes(np.ascontiguousarray(indices, dtype="<u4").tobytes())
        w.end_chunk(marker)

    if has_joints:
        marker = w.begin_chunk(CHUNK_SKIN)
        w.u32(surface_id)
        w.u32(n)
        record = np.zeros(n, dtype=[("joints", "<u4"), ("weights", "<f4", (4,))])
        record["joints"] = surface.joints
        record["weights"] = surface.weights
        w.bytes(record.tobytes())
        w.end_chunk(marker)
    return attribs, has_joints


def write_strings(w, chunk_id, strings):
    marker = w.begin_chunk(chunk_id)
    w.u32(len(strings))
    for value in strings:
        w.string(value)
    w.end_chunk(marker)


def write_nodes(w, model):
    count = len(model.nodes)
    if count > MAX_NODES:
        bad("too many nodes")
    for i, node in enumerate(model.nodes):
        if (
            node.parent >= count
            or (node.mesh >= 0 and node.mesh >= len(model.meshes))
            or (node.camera >= 0 and node.camera >= len(model.cameras))
            or (node.skeleton >= 0 and node.skeleton >= len(model.skeletons))
        ):
            bad("node %d refers to something that does not exist" % i)
        for child in node.children:
            if child < 0 or child >= count or model.nodes[child].parent != i:
                bad("node %d has a child list that disagrees with the parents" % i)
    for root in model.roots:
        if root < 0 or root >= count:
            bad("a root node does not exist")

    marker = w.begin_chunk(CHUNK_NODES)
    w.u32(count)
    for node in model.nodes:
        w.string(node.name)
        w.s32(node.parent)
        w.s32(node.mesh)
        w.s32(node.camera)
        w.s32(node.skeleton)
        w.floats(node.translation)
        w.floats(node.rotation)
        w.floats(node.scale)
        w.u32(len(node.children))
        for child in node.children:
            w.s32(child)
    w.u32(len(model.roots))
    for root in model.roots:
        w.s32(root)
    w.end_chunk(marker)


def write_cameras(w, cameras):
    marker = w.begin_chunk(CHUNK_CAMERAS)
    w.u32(len(cameras))
    for camera in cameras:
        w.u32(1 if camera.perspective else 0)
        w.f32(camera.yfov)
        w.f32(camera.aspect)
        w.f32(camera.z_near)
        w.f32(camera.z_far)
        w.f32(camera.xmag)
        w.f32(camera.ymag)
    w.end_chunk(marker)


def write_clip(w, clip, skeletons):
    bones = None
    if 0 <= clip.skeleton < len(skeletons):
        bones = skeletons[clip.skeleton].bones
    for track in clip.tracks:
        bone = track.bone
        if bone < -1 or (bone >= 0 and (bones is None or bone >= len(bones))):
            bad("clip '%s' has a bone track that is not in its skeleton" % clip.name)

    marker = w.begin_chunk(CHUNK_CLIP)
    w.string(clip.name)
    w.f32(clip.duration)
    w.s32(clip.skeleton)
    w.u32(clip.loop_mode)
    w.u32(len(clip.tracks))
    w.u32(0)
    w.u32(0)
    w.u32(0)
    w.u32(0)
    w.u32(0)

    for track in clip.tracks:
        w.s32(track.bone)
        w.string(bones[track.bone].name if track.bone >= 0 else "")
        w.u32(len(track.times))
        w.u32(len(track.positions))
        w.u32(len(track.rotations))
        w.u32(len(track.scales))
        w.floats(track.times)
        w.floats(track.positions)
        w.floats(track.rotations)
        w.floats(track.scales)
    w.end_chunk(marker)


def build_skeleton(skeleton):
    if len(skeleton.bones) > MAX_BONES:
        bad("too many bones")
    w = Writer()
    w.header(MAGIC_SKELETON, 0)
    marker = w.begin_chunk(CHUNK_BONES)
    w.u32(len(skeleton.bones))
    for bone in skeleton.bones:
        w.string(bone.name)
        w.s32(bone.parent)
        w.floats(bone.bind_local)
        w.floats(bone.inverse_bind)
    w.end_chunk(marker)
    end = w.begin_chunk(CHUNK_END)
    w.end_chunk(end)
    return bytes(w.out)


def build_animation(clip, skeletons):
    w = Writer()
    w.header(MAGIC_ANIMATION, 0)
    write_clip(w, clip, skeletons)
    end = w.begin_chunk(CHUNK_END)
    w.end_chunk(end)
    return bytes(w.out)


class PMeshFiles:
    def __init__(self):
        self.mesh = b""
        self.skeleton_names = []
        self.skeletons = []
        self.animation_names = []
        self.animations = []


def slot_name(material, index):
    name = clean_name(material.name)
    if not name:
        name = "material_%d" % index
    return name


def build_pmesh(model, converter, stem):
    files = PMeshFiles()
    material_names = [slot_name(material, i) for i, material in enumerate(model.materials)]
    if len(model.meshes) > MAX_MESHES:
        bad("too many meshes")

    w = Writer()
    w.header(MAGIC_MESH, 0)

    marker = w.begin_chunk(CHUNK_MATERIAL_SLOTS)
    w.u32(len(material_names))
    for name in material_names:
        w.string(name)
    w.end_chunk(marker)

    records = [[] for _ in model.meshes]
    surface_id = 0
    attrib_union = 0
    any_skin = False
    for m, mesh in enumerate(model.meshes):
        if len(mesh.primitives) > MAX_SUBMESHES:
            bad("too many sub-meshes")
        for primitive in mesh.primitives:
            if surface_id >= MAX_SURFACES:
                bad("too many surfaces")
            if primitive.material >= 0 and primitive.material >= len(model.materials):
                bad("surface %d has a material that does not exist" % surface_id)

            surface = converter.read_primitive(primitive)
            if (
                surface.tangents is None
                and surface.topology == TOPOLOGY_TRIANGLES
                and surface.normals is not None
                and surface.uvs is not None
            ):
                model.warnings.append(
                    "surface %d has normals and UVs but no tangents; they are not baked here" % surface_id
                )

            attribs, has_joints = write_surface(w, surface_id, surface)
            if primitive.skinned and not has_joints:
                bad("surface %d is skinned but has no joints" % surface_id)

            if primitive.has_aabb:
                bounds_min = primitive.aabb_min
                bounds_max = primitive.aabb_max
            else:
                bounds_min = surface.positions.min(axis=0)
                bounds_max = surface.positions.max(axis=0)
            values = np.concatenate((bounds_min, bounds_max))
            if not bool(np.all(np.isfinite(values))) or bool(np.any(bounds_min > bounds_max)):
                bad("surface %d has invalid bounds" % surface_id)

            flags = (SUBMESH_SKINNED if primitive.skinned else 0) | (
                SUBMESH_HAS_BOUNDS if primitive.has_aabb else 0
            )
            records[m].append(
                (
                    surface_id,
                    NO_SLOT if primitive.material < 0 else primitive.material,
                    flags,
                    bounds_min,
                    bounds_max,
                )
            )
            attrib_union |= attribs
            any_skin = any_skin or has_joints
            surface_id += 1

    marker = w.begin_chunk(CHUNK_SUBMESHES)
    w.u32(len(model.meshes))
    for m, mesh in enumerate(model.meshes):
        w.string(mesh.name)
        w.u32(len(records[m]))
        for surface_index, slot, flags, bounds_min, bounds_max in records[m]:
            w.u32(surface_index)
            w.u32(slot)
            w.u32(flags)
            w.floats(bounds_min)
            w.floats(bounds_max)
    w.end_chunk(marker)

    if model.nodes or model.roots:
        write_nodes(w, model)
    if model.cameras:
        write_cameras(w, model.cameras)

    if model.skeletons:
        for s, skeleton in enumerate(model.skeletons):
            if len(model.skeletons) == 1:
                name = stem + "." + SKELETON_EXTENSION
            else:
                name = "%s_%d.%s" % (stem, s, SKELETON_EXTENSION)
            files.skeleton_names.append(name)
            files.skeletons.append(build_skeleton(skeleton))
        write_strings(w, CHUNK_SKELETON_LINK, files.skeleton_names)
    if model.animations:
        if len(model.animations) > MAX_LINKS:
            bad("too many clips")
        for i, clip in enumerate(model.animations):
            data = build_animation(clip, model.skeletons)
            files.animation_names.append(clip_file_name(stem, clip.name, i, files.animation_names))
            files.animations.append(data)
        write_strings(w, CHUNK_ANIMATION_LINK, files.animation_names)

    end = w.begin_chunk(CHUNK_END)
    w.end_chunk(end)

    flags = FLAG_LITTLE_ENDIAN | (FLAG_SKINNED if any_skin else 0) | (attrib_union << HEADER_ATTRIB_SHIFT)
    w.out[8:12] = struct.pack("<I", flags)
    files.mesh = bytes(w.out)
    return files


def write_staged(items):
    staged = []
    try:
        for path, data in items:
            directory = os.path.dirname(path)
            if directory:
                os.makedirs(directory, exist_ok=True)
            temporary = path + ".tmp"
            if os.path.exists(temporary):
                os.remove(temporary)
            with open(temporary, "wb") as handle:
                handle.write(data)
            staged.append((temporary, path))
        for temporary, path in staged:
            os.replace(temporary, path)
        staged = []
    finally:
        for temporary, _ in staged:
            if os.path.exists(temporary):
                os.remove(temporary)
