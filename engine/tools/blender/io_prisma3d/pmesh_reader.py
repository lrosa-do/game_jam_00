import os
import struct

import numpy as np

from .model import (
    BoneTrack,
    Camera,
    Clip,
    ConvertError,
    Mesh,
    Node,
    Primitive,
    Skeleton,
)
from .pmesh_writer import (
    CHUNK_ANIMATION_LINK,
    CHUNK_BONES,
    CHUNK_CAMERAS,
    CHUNK_CLIP,
    CHUNK_END,
    CHUNK_INDEX_BUFFER,
    CHUNK_MATERIAL_SLOTS,
    CHUNK_NODES,
    CHUNK_SKELETON_LINK,
    CHUNK_SKIN,
    CHUNK_SUBMESHES,
    CHUNK_VERTEX_BUFFER,
    FLAG_LITTLE_ENDIAN,
    FLAG_SKINNED,
    HAS_COLOR,
    HAS_NORMAL,
    HAS_TANGENT,
    HAS_UV,
    HAS_UV2,
    HEADER_ATTRIB_SHIFT,
    INDEX16,
    MAGIC_ANIMATION,
    MAGIC_MESH,
    MAGIC_SKELETON,
    MAX_BONES,
    MAX_INDICES,
    MAX_LINKS,
    MAX_MESHES,
    MAX_NODES,
    MAX_SUBMESHES,
    MAX_SURFACES,
    MAX_TOPOLOGY,
    MAX_VERTICES,
    NO_SLOT,
    SUBMESH_HAS_BOUNDS,
    SUBMESH_SKINNED,
    VERSION_MAJOR,
    directory_part,
    fourcc,
)

CHUNK_SHADOW_MESH = fourcc("SHDW")

KNOWN_ATTRIBS = HAS_NORMAL | HAS_TANGENT | HAS_UV | HAS_UV2 | HAS_COLOR
SKIN_VERTEX_SIZE = 20
HEADER_SIZE = 12
CHUNK_HEADER_SIZE = 12
MAX_STRING = 1 << 20
MAX_CLIPS = 1 << 20
MAX_KEYS = 100000000
MAX_TRACKS = 1 << 24
LOOP_MODE_COUNT = 3
SUBMESH_RECORD_SIZE = 36
NODE_FIXED_SIZE = 4 + 4 * 4 + 40 + 4
CAMERA_RECORD_SIZE = 28
SHADOW_SURFACE_HEADER_SIZE = 16
BONE_FIXED_SIZE = 4 + 4 + 128
CLIP_RECORD_SIZES = (24, 28, 12, 16, 12, 8)


def vertex_attrib_stride(attribs):
    stride = 12
    if attribs & HAS_NORMAL:
        stride += 12
    if attribs & HAS_TANGENT:
        stride += 16
    if attribs & HAS_UV:
        stride += 8
    if attribs & HAS_UV2:
        stride += 8
    if attribs & HAS_COLOR:
        stride += 16
    return stride


def chunk_name(chunk_id):
    text = ""
    for i in range(4):
        c = (chunk_id >> (8 * i)) & 0xFF
        text += chr(c) if 32 <= c < 127 else "?"
    return text


class Reader:
    def __init__(self, data):
        self.data = data
        self.pos = 0
        self.limit = len(data)
        self.parent_limit = 0
        self.chunk_end = 0
        self.chunk_start = 0
        self.chunk_id = 0
        self.in_chunk = False

    def remaining(self):
        return self.limit - self.pos if self.pos <= self.limit else 0

    def available(self, amount):
        return amount <= self.remaining()

    def fits(self, count, record_size):
        return record_size == 0 or count <= self.remaining() // record_size

    def fail(self, what):
        if self.in_chunk:
            raise ConvertError(
                "chunk %s at offset %d, read at offset %d: %s"
                % (chunk_name(self.chunk_id), self.chunk_start, self.pos, what)
            )
        raise ConvertError("at offset %d: %s" % (self.pos, what))

    def u32(self):
        if not self.available(4):
            self.fail("truncated")
        value = struct.unpack_from("<I", self.data, self.pos)[0]
        self.pos += 4
        return value

    def s32(self):
        if not self.available(4):
            self.fail("truncated")
        value = struct.unpack_from("<i", self.data, self.pos)[0]
        self.pos += 4
        return value

    def u64(self):
        if not self.available(8):
            self.fail("truncated")
        value = struct.unpack_from("<Q", self.data, self.pos)[0]
        self.pos += 8
        return value

    def f32(self):
        if not self.available(4):
            self.fail("truncated")
        value = struct.unpack_from("<f", self.data, self.pos)[0]
        self.pos += 4
        return np.float32(value)

    def f32_finite(self):
        value = self.f32()
        if not np.isfinite(value):
            self.fail("value is not finite")
        return value

    def string(self):
        size = self.u32()
        if size > MAX_STRING:
            self.fail("string is too long")
        if not self.available(size):
            self.fail("truncated string")
        value = bytes(self.data[self.pos:self.pos + size]).decode("utf-8", errors="replace")
        self.pos += size
        return value

    def vec3(self):
        return (self.f32_finite(), self.f32_finite(), self.f32_finite())

    def box(self):
        low = self.vec3()
        high = self.vec3()
        if low[0] > high[0] or low[1] > high[1] or low[2] > high[2]:
            self.fail("inverted bounds")
        return low, high

    def floats(self, count):
        if not self.fits(count, 4):
            self.fail("truncated")
        values = np.frombuffer(self.data, dtype="<f4", count=count, offset=self.pos).astype(np.float32)
        self.pos += count * 4
        return values

    def words(self, count):
        if not self.fits(count, 4):
            self.fail("truncated")
        values = np.frombuffer(self.data, dtype="<u4", count=count, offset=self.pos).astype(np.uint32)
        self.pos += count * 4
        return values

    def header(self, expected_magic):
        if not self.available(HEADER_SIZE):
            self.fail("file is smaller than the header")
        magic = self.u32()
        version = self.u32()
        flags = self.u32()
        if magic != expected_magic:
            self.fail("wrong magic number")
        if (version >> 16) != VERSION_MAJOR:
            self.fail("unsupported major version")
        if (flags & FLAG_LITTLE_ENDIAN) == 0:
            self.fail("not a little-endian file")
        return flags

    def next_chunk(self):
        self.chunk_start = self.pos
        if self.remaining() < CHUNK_HEADER_SIZE:
            self.fail("truncated chunk header")
        chunk_id = self.u32()
        size = self.u64()
        self.chunk_id = chunk_id
        self.in_chunk = True
        if size > self.remaining():
            self.fail("chunk is larger than the rest of the file")
        return chunk_id, size

    def enter(self, size):
        if self.chunk_end != 0 or size > self.remaining():
            self.fail("cannot enter chunk")
        self.parent_limit = self.limit
        self.chunk_end = self.pos + size
        self.limit = self.chunk_end

    def leave(self):
        if self.chunk_end == 0:
            return
        self.pos = self.chunk_end
        self.limit = self.parent_limit
        self.parent_limit = 0
        self.chunk_end = 0
        self.in_chunk = False


class SurfaceRef:
    def __init__(self):
        self.vertex_offset = 0
        self.attribs = 0
        self.vertex_count = 0
        self.topology = 0
        self.index_offset = 0
        self.index_count = 0
        self.index16 = False
        self.has_indices = False
        self.skin_offset = 0
        self.has_skin = False


class SubMeshRecord:
    def __init__(self):
        self.surface = 0
        self.material_slot = NO_SLOT
        self.flags = 0
        self.bounds_min = (np.float32(0.0),) * 3
        self.bounds_max = (np.float32(0.0),) * 3


class MeshEntry:
    def __init__(self):
        self.name = ""
        self.sub_meshes = []


class ParsedFile:
    def __init__(self):
        self.flags = 0
        self.materials = []
        self.has_materials = False
        self.meshes = []
        self.has_sub_meshes = False
        self.nodes = []
        self.roots = []
        self.has_nodes = False
        self.cameras = []
        self.has_cameras = False
        self.shadows = []
        self.skeleton_links = []
        self.has_skeleton_links = False
        self.animation_links = []
        self.has_animation_links = False


class PMeshModel:
    def __init__(self):
        self.path = ""
        self.directory = ""
        self.data = b""
        self.surfaces = []
        self.material_slots = []
        self.meshes = []
        self.nodes = []
        self.roots = []
        self.cameras = []
        self.skeletons = []
        self.animations = []
        self.skeleton_files = []
        self.warnings = []


def read_strings(r, limit):
    count = r.u32()
    if count > limit or not r.fits(count, 4):
        r.fail("invalid string count")
    return [r.string() for _ in range(count)]


def read_vertex_buffer(r, surfaces):
    surface = r.u32()
    attribs = r.u32()
    vertex_count = r.u32()
    topology = r.u32()
    if surface != len(surfaces):
        r.fail("surface ids must be sequential")
    if len(surfaces) >= MAX_SURFACES:
        r.fail("too many surfaces")
    if (attribs & ~KNOWN_ATTRIBS) != 0:
        r.fail("unknown vertex attributes")
    if vertex_count == 0 or vertex_count > MAX_VERTICES:
        r.fail("invalid vertex count")
    if topology > MAX_TOPOLOGY:
        r.fail("invalid topology")
    if not r.fits(vertex_count, vertex_attrib_stride(attribs)):
        r.fail("truncated vertex data")
    ref = SurfaceRef()
    ref.vertex_offset = r.pos
    ref.attribs = attribs
    ref.vertex_count = vertex_count
    ref.topology = topology
    surfaces.append(ref)


def read_index_buffer(r, surfaces):
    surface = r.u32()
    flags = r.u32()
    index_count = r.u32()
    if surface >= len(surfaces):
        r.fail("index buffer of an unknown surface")
    ref = surfaces[surface]
    if ref.has_indices:
        r.fail("surface has two index buffers")
    if (flags & ~INDEX16) != 0:
        r.fail("unknown index flags")
    if index_count == 0 or index_count > MAX_INDICES:
        r.fail("invalid index count")
    index16 = (flags & INDEX16) != 0
    if not r.fits(index_count, 2 if index16 else 4):
        r.fail("truncated index data")
    ref.has_indices = True
    ref.index16 = index16
    ref.index_count = index_count
    ref.index_offset = r.pos


def read_skin_chunk(r, surfaces):
    surface = r.u32()
    vertex_count = r.u32()
    if surface >= len(surfaces):
        r.fail("skin of an unknown surface")
    ref = surfaces[surface]
    if ref.has_skin:
        r.fail("surface has two skin chunks")
    if vertex_count != ref.vertex_count:
        r.fail("skin vertex count differs from the surface")
    if not r.fits(vertex_count, SKIN_VERTEX_SIZE):
        r.fail("truncated skin data")
    ref.has_skin = True
    ref.skin_offset = r.pos


def read_sub_meshes(r, parsed):
    mesh_count = r.u32()
    if mesh_count > MAX_MESHES or not r.fits(mesh_count, 8):
        r.fail("invalid mesh count")
    for _ in range(mesh_count):
        mesh = MeshEntry()
        mesh.name = r.string()
        count = r.u32()
        if count > MAX_SUBMESHES or not r.fits(count, SUBMESH_RECORD_SIZE):
            r.fail("invalid sub-mesh count")
        for _ in range(count):
            record = SubMeshRecord()
            record.surface = r.u32()
            record.material_slot = r.u32()
            record.flags = r.u32()
            record.bounds_min, record.bounds_max = r.box()
            mesh.sub_meshes.append(record)
        parsed.meshes.append(mesh)


def read_nodes(r, parsed):
    count = r.u32()
    if count > MAX_NODES or not r.fits(count, NODE_FIXED_SIZE):
        r.fail("invalid node count")
    for _ in range(count):
        node = Node()
        node.name = r.string()
        parent = r.s32()
        mesh = r.s32()
        camera = r.s32()
        skeleton = r.s32()
        if parent < -1 or mesh < -1 or camera < -1 or skeleton < -1:
            r.fail("negative reference in node")
        node.parent = parent
        node.mesh = mesh
        node.camera = camera
        node.skeleton = skeleton
        node.translation = r.vec3()
        node.rotation = (r.f32_finite(), r.f32_finite(), r.f32_finite(), r.f32_finite())
        node.scale = r.vec3()
        child_count = r.u32()
        if not r.fits(child_count, 4):
            r.fail("invalid child count")
        for _ in range(child_count):
            child = r.s32()
            if child < 0:
                r.fail("negative child reference")
            node.children.append(child)
        parsed.nodes.append(node)
    root_count = r.u32()
    if not r.fits(root_count, 4):
        r.fail("invalid root count")
    for _ in range(root_count):
        root = r.s32()
        if root < 0:
            r.fail("negative root reference")
        parsed.roots.append(root)


def read_cameras(r, parsed):
    count = r.u32()
    if count > MAX_NODES or not r.fits(count, CAMERA_RECORD_SIZE):
        r.fail("invalid camera count")
    for _ in range(count):
        camera = Camera()
        perspective = r.u32()
        camera.yfov = r.f32_finite()
        camera.aspect = r.f32_finite()
        camera.z_near = r.f32_finite()
        camera.z_far = r.f32_finite()
        camera.xmag = r.f32_finite()
        camera.ymag = r.f32_finite()
        camera.perspective = perspective != 0
        parsed.cameras.append(camera)


def read_shadow(r, parsed):
    mesh = r.u32()
    count = r.u32()
    if count == 0 or count > MAX_SURFACES or not r.fits(count, SHADOW_SURFACE_HEADER_SIZE):
        r.fail("invalid shadow surface count")
    for _ in range(count):
        topology = r.u32()
        vertex_count = r.u32()
        flags = r.u32()
        index_count = r.u32()
        if topology > MAX_TOPOLOGY:
            r.fail("invalid topology")
        if (
            vertex_count == 0
            or vertex_count > MAX_VERTICES
            or index_count > MAX_INDICES
            or (flags & ~INDEX16) != 0
        ):
            r.fail("invalid shadow surface")
        if not r.fits(vertex_count, 12):
            r.fail("truncated shadow positions")
        r.floats(vertex_count * 3)
        if index_count != 0:
            index16 = (flags & INDEX16) != 0
            if not r.fits(index_count, 2 if index16 else 4):
                r.fail("truncated shadow indices")
            if index16:
                indices = np.frombuffer(r.data, dtype="<u2", count=index_count, offset=r.pos)
                r.pos += index_count * 2
            else:
                indices = r.words(index_count)
            if bool(np.any(indices >= vertex_count)):
                r.fail("shadow index out of range")
    parsed.shadows.append(mesh)


def parse_mesh(data):
    r = Reader(data)
    parsed = ParsedFile()
    surfaces = []
    parsed.flags = r.header(MAGIC_MESH)
    saw_end = False
    while True:
        if r.remaining() == 0:
            r.fail("missing end chunk, the file is truncated")
        chunk_id, size = r.next_chunk()
        r.enter(size)
        if chunk_id == CHUNK_MATERIAL_SLOTS:
            if parsed.has_materials:
                r.fail("duplicate material chunk")
            parsed.materials = read_strings(r, MAX_MESHES)
            parsed.has_materials = True
        elif chunk_id == CHUNK_VERTEX_BUFFER:
            read_vertex_buffer(r, surfaces)
        elif chunk_id == CHUNK_INDEX_BUFFER:
            read_index_buffer(r, surfaces)
        elif chunk_id == CHUNK_SKIN:
            read_skin_chunk(r, surfaces)
        elif chunk_id == CHUNK_SUBMESHES:
            if parsed.has_sub_meshes:
                r.fail("duplicate sub-mesh chunk")
            read_sub_meshes(r, parsed)
            parsed.has_sub_meshes = True
        elif chunk_id == CHUNK_NODES:
            if parsed.has_nodes:
                r.fail("duplicate node chunk")
            read_nodes(r, parsed)
            parsed.has_nodes = True
        elif chunk_id == CHUNK_CAMERAS:
            if parsed.has_cameras:
                r.fail("duplicate camera chunk")
            read_cameras(r, parsed)
            parsed.has_cameras = True
        elif chunk_id == CHUNK_SHADOW_MESH:
            read_shadow(r, parsed)
        elif chunk_id == CHUNK_SKELETON_LINK:
            if parsed.has_skeleton_links:
                r.fail("duplicate skeleton link chunk")
            parsed.skeleton_links = read_strings(r, MAX_LINKS)
            parsed.has_skeleton_links = True
        elif chunk_id == CHUNK_ANIMATION_LINK:
            if parsed.has_animation_links:
                r.fail("duplicate animation link chunk")
            parsed.animation_links = read_strings(r, MAX_LINKS)
            parsed.has_animation_links = True
        elif chunk_id == CHUNK_END:
            saw_end = True
        r.leave()
        if saw_end:
            break
    if r.remaining() != 0:
        r.fail("data after the end chunk")
    return parsed, surfaces


def check_references(parsed, surfaces):
    attribs = 0
    skinned = False
    for surface in surfaces:
        attribs |= surface.attribs
        skinned = skinned or surface.has_skin
    if ((parsed.flags >> HEADER_ATTRIB_SHIFT) & KNOWN_ATTRIBS) != attribs or (
        (parsed.flags & FLAG_SKINNED) != 0
    ) != skinned:
        raise ConvertError("header flags disagree with the surfaces")

    for mesh in parsed.meshes:
        for record in mesh.sub_meshes:
            if record.surface >= len(surfaces):
                raise ConvertError("sub-mesh refers to a missing surface")
            if record.material_slot != NO_SLOT and record.material_slot >= len(parsed.materials):
                raise ConvertError("sub-mesh refers to a missing material slot")
            if (record.flags & SUBMESH_SKINNED) != 0 and not surfaces[record.surface].has_skin:
                raise ConvertError("skinned sub-mesh has a surface without skin data")

    for i, mesh in enumerate(parsed.shadows):
        if mesh >= len(parsed.meshes):
            raise ConvertError("shadow mesh of a missing mesh")
        if mesh in parsed.shadows[:i]:
            raise ConvertError("two shadow meshes for one mesh")

    node_count = len(parsed.nodes)
    for i, node in enumerate(parsed.nodes):
        if (
            node.parent >= node_count
            or (node.mesh >= 0 and node.mesh >= len(parsed.meshes))
            or (node.camera >= 0 and node.camera >= len(parsed.cameras))
            or (node.skeleton >= 0 and node.skeleton >= len(parsed.skeleton_links))
        ):
            raise ConvertError("node refers to something that does not exist")
        for child in node.children:
            if child >= node_count or parsed.nodes[child].parent != i:
                raise ConvertError("node child list disagrees with the parents")
    for root in parsed.roots:
        if root >= node_count or parsed.nodes[root].parent != -1:
            raise ConvertError("root is not a parentless node")

    state = [0] * node_count
    for i in range(node_count):
        if state[i] == 2:
            continue
        current = i
        while True:
            if state[current] == 1:
                raise ConvertError("node hierarchy has a cycle")
            if state[current] == 2:
                break
            state[current] = 1
            parent = parsed.nodes[current].parent
            if parent < 0:
                break
            current = parent
        current = i
        while state[current] == 1:
            state[current] = 2
            parent = parsed.nodes[current].parent
            if parent < 0:
                break
            current = parent


def read_skeleton(data):
    r = Reader(data)
    r.header(MAGIC_SKELETON)
    found = False
    saw_end = False
    loaded = Skeleton()
    while True:
        if r.remaining() == 0:
            r.fail("missing end chunk, the file is truncated")
        chunk_id, size = r.next_chunk()
        r.enter(size)
        if chunk_id == CHUNK_BONES:
            if found:
                r.fail("duplicate bone chunk")
            count = r.u32()
            if count > MAX_BONES or not r.fits(count, BONE_FIXED_SIZE):
                r.fail("invalid bone count")
            for _ in range(count):
                name = r.string()
                parent = r.s32()
                local = r.floats(16)
                inverse = r.floats(16)
                if not loaded.add_bone(name, parent, local, inverse):
                    r.fail("invalid bone")
            found = True
        elif chunk_id == CHUNK_END:
            saw_end = True
        r.leave()
        if saw_end:
            break
    if not found:
        r.fail("no bone chunk")
    if r.remaining() != 0:
        r.fail("data after the end chunk")
    if not loaded.finalize():
        r.fail("bone hierarchy is invalid")
    return loaded


def read_float_array(r, count):
    if count > MAX_KEYS or not r.fits(count, 4):
        r.fail("invalid key count")
    return r.floats(count)


def read_vec_array(r, count, width):
    if count > MAX_KEYS or not r.fits(count, width * 4):
        r.fail("invalid key count")
    return r.floats(count * width).reshape(count, width)


def read_string_array(r, count):
    if count > MAX_KEYS or not r.fits(count, 4):
        r.fail("invalid key count")
    return [r.string() for _ in range(count)]


def read_count(r, record):
    count = r.u32()
    if count > MAX_TRACKS or not r.fits(count, record):
        r.fail("invalid track count")
    return count


def resolve_bone(skeleton, bone, bone_name):
    if bone < len(skeleton.bones) and skeleton.bones[bone].name == bone_name:
        return bone
    return skeleton.find_bone(bone_name)


def read_clip(r, skeletons):
    clip = Clip()
    clip.name = r.string()
    clip.duration = r.f32_finite()
    clip.skeleton = r.s32()
    clip.loop_mode = r.u32()
    if clip.duration < 0.0:
        r.fail("negative clip duration")
    if clip.loop_mode >= LOOP_MODE_COUNT:
        r.fail("invalid loop mode")
    counts = [read_count(r, record) for record in CLIP_RECORD_SIZES]

    bones = None
    if skeletons is not None and 0 <= clip.skeleton < len(skeletons):
        bones = skeletons[clip.skeleton]

    for _ in range(counts[0]):
        bone = r.s32()
        bone_name = r.string()
        if bone < -1:
            r.fail("invalid bone reference")
        if bone >= 0 and skeletons is not None:
            if bones is None:
                r.fail("bone track in a clip without a skeleton")
            bone = resolve_bone(bones, bone, bone_name)
            if bone < 0:
                r.fail("bone track refers to a bone that is not in the skeleton")
        track = BoneTrack(bone)
        track.bone_name = bone_name
        n = [r.u32() for _ in range(4)]
        track.times = read_float_array(r, n[0])
        track.positions = read_vec_array(r, n[1], 3)
        track.rotations = read_vec_array(r, n[2], 4)
        track.scales = read_vec_array(r, n[3], 3)
        clip.tracks.append(track)

    for _ in range(counts[1]):
        r.string()
        n = [r.u32() for _ in range(6)]
        read_float_array(r, n[0])
        read_vec_array(r, n[1], 3)
        read_float_array(r, n[2])
        read_vec_array(r, n[3], 4)
        read_float_array(r, n[4])
        read_vec_array(r, n[5], 3)

    for _ in range(counts[2]):
        r.string()
        n = [r.u32(), r.u32()]
        read_float_array(r, n[0])
        read_string_array(r, n[1])

    for _ in range(counts[3]):
        r.string()
        r.string()
        n = [r.u32(), r.u32()]
        read_float_array(r, n[0])
        read_float_array(r, n[1])

    for _ in range(counts[4]):
        r.string()
        n = [r.u32(), r.u32()]
        read_float_array(r, n[0])
        read_string_array(r, n[1])

    for _ in range(counts[5]):
        r.f32_finite()
        if not r.string():
            r.fail("invalid animation events")
    return clip


def read_animations(data, skeletons=None):
    r = Reader(data)
    r.header(MAGIC_ANIMATION)
    saw_end = False
    loaded = []
    while True:
        if r.remaining() == 0:
            r.fail("missing end chunk, the file is truncated")
        chunk_id, size = r.next_chunk()
        r.enter(size)
        if chunk_id == CHUNK_CLIP:
            if len(loaded) >= MAX_CLIPS:
                r.fail("too many clips")
            loaded.append(read_clip(r, skeletons))
        elif chunk_id == CHUNK_END:
            saw_end = True
        r.leave()
        if saw_end:
            break
    if r.remaining() != 0:
        r.fail("data after the end chunk")
    return loaded


def load_linked(mesh_path, link):
    normalized = link.replace("\\", "/")
    candidates = (directory_part(mesh_path) + normalized, normalized)
    for candidate in candidates:
        if not os.path.exists(candidate):
            continue
        try:
            with open(candidate, "rb") as handle:
                data = handle.read()
        except OSError:
            return None
        return data if data else None
    return None


def load_pmesh(path):
    try:
        with open(path, "rb") as handle:
            data = handle.read()
    except OSError:
        raise ConvertError("cannot read file")
    if not data:
        raise ConvertError("cannot read file")

    parsed, surfaces = parse_mesh(data)
    check_references(parsed, surfaces)

    model = PMeshModel()
    model.path = path
    model.directory = os.path.dirname(os.path.abspath(path))
    model.data = data
    model.surfaces = surfaces

    for link in parsed.skeleton_links:
        linked = load_linked(path, link) if link else None
        if linked is None:
            raise ConvertError("linked skeleton '%s' is missing" % link)
        try:
            model.skeletons.append(read_skeleton(linked))
        except ConvertError as error:
            raise ConvertError("linked skeleton '%s': %s" % (link, error))
        model.skeleton_files.append(link)
    for link in parsed.animation_links:
        linked = load_linked(path, link) if link else None
        if linked is None:
            raise ConvertError("linked animation '%s' is missing" % link)
        try:
            model.animations.extend(read_animations(linked, model.skeletons))
        except ConvertError as error:
            raise ConvertError("linked animation '%s': %s" % (link, error))

    model.material_slots = list(parsed.materials)
    for entry in parsed.meshes:
        mesh = Mesh()
        mesh.name = entry.name
        for record in entry.sub_meshes:
            primitive = Primitive()
            surface = surfaces[record.surface]
            primitive.topology = surface.topology
            primitive.material = -1 if record.material_slot == NO_SLOT else record.material_slot
            primitive.has_aabb = (record.flags & SUBMESH_HAS_BOUNDS) != 0
            primitive.aabb_min = np.array(record.bounds_min, dtype=np.float32)
            primitive.aabb_max = np.array(record.bounds_max, dtype=np.float32)
            primitive.skinned = (record.flags & SUBMESH_SKINNED) != 0
            primitive.has_joints = surface.has_skin
            primitive.source = record.surface
            mesh.primitives.append(primitive)
        model.meshes.append(mesh)
    model.nodes = parsed.nodes
    model.roots = parsed.roots
    model.cameras = parsed.cameras
    return model


class SurfaceData:
    def __init__(self):
        self.positions = None
        self.normals = None
        self.tangents = None
        self.uvs = None
        self.uvs2 = None
        self.colors = None
        self.joints = None
        self.weights = None
        self.indices = None


def read_surface(model, surface_index):
    ref = model.surfaces[surface_index]
    count = ref.vertex_count
    width = vertex_attrib_stride(ref.attribs) // 4
    block = np.frombuffer(model.data, dtype="<f4", count=count * width, offset=ref.vertex_offset)
    block = block.reshape(count, width).astype(np.float32)
    out = SurfaceData()
    column = 0

    def take(size):
        nonlocal column
        values = np.ascontiguousarray(block[:, column:column + size])
        column += size
        return values

    out.positions = take(3)
    if ref.attribs & HAS_NORMAL:
        out.normals = take(3)
    if ref.attribs & HAS_TANGENT:
        out.tangents = take(4)
    if ref.attribs & HAS_UV:
        out.uvs = take(2)
    if ref.attribs & HAS_UV2:
        out.uvs2 = take(2)
    if ref.attribs & HAS_COLOR:
        out.colors = take(4)

    if ref.has_skin:
        record = np.frombuffer(
            model.data,
            dtype=[("joints", "<u4"), ("weights", "<f4", (4,))],
            count=count,
            offset=ref.skin_offset,
        )
        out.joints = record["joints"].astype(np.uint32)
        out.weights = record["weights"].astype(np.float32)

    if ref.has_indices:
        dtype = "<u2" if ref.index16 else "<u4"
        indices = np.frombuffer(
            model.data, dtype=dtype, count=ref.index_count, offset=ref.index_offset
        ).astype(np.uint32)
        if bool(np.any(indices >= count)):
            raise ConvertError("surface %d has an index out of range" % surface_index)
        out.indices = indices
    return out
