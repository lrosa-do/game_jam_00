import numpy as np

TOPOLOGY_TRIANGLES = 0
TOPOLOGY_TRIANGLE_STRIP = 1
TOPOLOGY_LINES = 2
TOPOLOGY_LINE_STRIP = 3
TOPOLOGY_POINTS = 4

ALPHA_OPAQUE = 0
ALPHA_MASK = 1
ALPHA_BLEND = 2


class ConvertError(Exception):
    pass


class Surface:
    def __init__(self):
        self.topology = TOPOLOGY_TRIANGLES
        self.positions = np.zeros((0, 3), dtype=np.float32)
        self.normals = None
        self.tangents = None
        self.colors = None
        self.uvs = None
        self.uvs2 = None
        self.joints = None
        self.weights = None
        self.indices = None


class Primitive:
    def __init__(self):
        self.topology = TOPOLOGY_TRIANGLES
        self.material = -1
        self.has_aabb = False
        self.aabb_min = np.zeros(3, dtype=np.float32)
        self.aabb_max = np.zeros(3, dtype=np.float32)
        self.skinned = False
        self.has_joints = False
        self.source = None


class Mesh:
    def __init__(self):
        self.name = ""
        self.primitives = []


class Node:
    def __init__(self):
        self.name = ""
        self.parent = -1
        self.children = []
        self.translation = (np.float32(0.0), np.float32(0.0), np.float32(0.0))
        self.rotation = (np.float32(0.0), np.float32(0.0), np.float32(0.0), np.float32(1.0))
        self.scale = (np.float32(1.0), np.float32(1.0), np.float32(1.0))
        self.mesh = -1
        self.camera = -1
        self.skeleton = -1


class Camera:
    def __init__(self):
        self.perspective = True
        self.yfov = np.float32(0.9)
        self.aspect = np.float32(0.0)
        self.z_near = np.float32(0.1)
        self.z_far = np.float32(1000.0)
        self.xmag = np.float32(1.0)
        self.ymag = np.float32(1.0)


class Material:
    def __init__(self):
        self.name = ""
        self.base_color = (np.float32(1.0),) * 4
        self.metallic = np.float32(1.0)
        self.roughness = np.float32(1.0)
        self.emissive = (np.float32(0.0),) * 3
        self.alpha_cutoff = np.float32(0.5)
        self.alpha_mode = ALPHA_OPAQUE
        self.double_sided = False
        self.base_color_texture = -1
        self.metallic_roughness_texture = -1
        self.normal_texture = -1
        self.occlusion_texture = -1
        self.emissive_texture = -1


class Texture:
    def __init__(self):
        self.image = -1


class Image:
    def __init__(self):
        self.name = ""
        self.path = ""
        self.mime_type = ""


class Bone:
    def __init__(self, name, parent, bind_local, inverse_bind):
        self.name = name
        self.parent = parent
        self.bind_local = bind_local
        self.inverse_bind = inverse_bind


class Skeleton:
    MAX_BONES = 65535

    def __init__(self):
        self.bones = []

    def find_bone(self, name):
        for i, bone in enumerate(self.bones):
            if bone.name == name:
                return i
        return -1

    def add_bone(self, name, parent, bind_local, inverse_bind):
        if not name or parent < -1 or parent >= self.MAX_BONES or self.find_bone(name) >= 0:
            return False
        if len(self.bones) >= self.MAX_BONES:
            return False
        self.bones.append(Bone(name, parent, bind_local, inverse_bind))
        return True

    def finalize(self):
        count = len(self.bones)
        placed = [False] * count
        placed_count = 0
        progress = True
        while placed_count < count and progress:
            progress = False
            for i in range(count):
                if placed[i]:
                    continue
                parent = self.bones[i].parent
                if parent >= count or parent == i:
                    return False
                if parent < 0 or placed[parent]:
                    placed[i] = True
                    placed_count += 1
                    progress = True
        return placed_count == count


class BoneTrack:
    def __init__(self, bone):
        self.bone = bone
        self.times = np.zeros(0, dtype=np.float32)
        self.positions = np.zeros((0, 3), dtype=np.float32)
        self.rotations = np.zeros((0, 4), dtype=np.float32)
        self.scales = np.zeros((0, 3), dtype=np.float32)


class Clip:
    def __init__(self):
        self.name = ""
        self.duration = np.float32(0.0)
        self.skeleton = 0
        self.loop_mode = 0
        self.tracks = []


class Model:
    def __init__(self):
        self.directory = ""
        self.nodes = []
        self.roots = []
        self.meshes = []
        self.materials = []
        self.textures = []
        self.images = []
        self.cameras = []
        self.skeletons = []
        self.animations = []
        self.warnings = []
