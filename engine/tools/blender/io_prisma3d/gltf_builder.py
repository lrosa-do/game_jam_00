import json
import math
import os
from urllib.parse import quote

import numpy as np

from . import f32 as m
from .gltf_reader import COMPONENT_DTYPE, COMPONENT_FLOAT, COMPONENT_UINT8, COMPONENT_UINT16, COMPONENT_UINT32
from .material_reader import find_material
from .model import (
    TOPOLOGY_LINE_STRIP,
    TOPOLOGY_LINES,
    TOPOLOGY_POINTS,
    TOPOLOGY_TRIANGLE_STRIP,
    TOPOLOGY_TRIANGLES,
    ConvertError,
)
from .pmesh_reader import read_surface

TARGET_ARRAY_BUFFER = 34962
TARGET_ELEMENT_ARRAY_BUFFER = 34963

TOPOLOGY_MODES = {
    TOPOLOGY_POINTS: 0,
    TOPOLOGY_LINES: 1,
    TOPOLOGY_LINE_STRIP: 3,
    TOPOLOGY_TRIANGLES: 4,
    TOPOLOGY_TRIANGLE_STRIP: 5,
}

UNIT_TOLERANCE = 1e-5
FRAME_SNAP_TOLERANCE = 1e-3
PARENT_TOLERANCE = 1e-4
EXTENSION_EMISSIVE_STRENGTH = "KHR_materials_emissive_strength"


class GNode:
    def __init__(self):
        self.name = ""
        self.translation = (0.0, 0.0, 0.0)
        self.rotation = (0.0, 0.0, 0.0, 1.0)
        self.scale = (1.0, 1.0, 1.0)
        self.parent = -1
        self.children = []
        self.mesh = -1
        self.camera = -1
        self.skeleton = -1


def unit_quaternion(values):
    values = [float(v) for v in values]
    length = math.sqrt(sum(v * v for v in values))
    if length < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    if abs(length - 1.0) > UNIT_TOLERANCE:
        values = [v / length for v in values]
    return tuple(values)


def unit_quaternions(values):
    values = np.asarray(values, dtype=np.float32)
    if len(values) == 0:
        return values
    length = np.sqrt(np.sum(values.astype(np.float64) ** 2, axis=1))
    fix = np.abs(length - 1.0) > UNIT_TOLERANCE
    if not bool(np.any(fix)):
        return values
    out = values.copy()
    degenerate = length < 1e-12
    safe = np.where(degenerate, 1.0, length)
    out[fix] = (values[fix] / safe[fix][:, None]).astype(np.float32)
    out[degenerate] = np.array((0.0, 0.0, 0.0, 1.0), dtype=np.float32)
    return out


def clamp01(value):
    return min(1.0, max(0.0, float(value)))


def matrix_of(values):
    return np.asarray(values, dtype=np.float64).reshape(4, 4).T


def bone_order(skeleton):
    count = len(skeleton.bones)
    placed = [False] * count
    order = []
    progress = True
    while len(order) < count and progress:
        progress = False
        for i in range(count):
            if placed[i]:
                continue
            parent = skeleton.bones[i].parent
            if parent < 0 or placed[parent]:
                placed[i] = True
                order.append(i)
                progress = True
    return order


class GltfBuilder:
    def __init__(self, model, library, material_directory, stem, frame_rate=None):
        self.model = model
        self.frame_rate = frame_rate
        self.library = library
        self.material_directory = material_directory
        self.stem = stem
        self.warnings = []
        self.binary = bytearray()
        self.views = []
        self.accessors = []
        self.nodes = []
        self.roots = []
        self.meshes = []
        self.mesh_map = []
        self.skins = []
        self.joint_nodes = []
        self.cameras = []
        self.materials = []
        self.textures = []
        self.images = []
        self.texture_of_path = {}
        self.animations = []
        self.extensions_used = []
        self.surface_cache = {}
        self.root_parents = []

    def add_view(self, data, target=None):
        while len(self.binary) % 4:
            self.binary.append(0)
        view = {"buffer": 0, "byteOffset": len(self.binary), "byteLength": len(data)}
        if target is not None:
            view["target"] = target
        self.binary += data
        self.views.append(view)
        return len(self.views) - 1

    def add_accessor(self, values, kind, component, target=None, bounds=False):
        array = np.ascontiguousarray(values, dtype=COMPONENT_DTYPE[component])
        accessor = {
            "bufferView": self.add_view(array.tobytes(), target),
            "componentType": component,
            "count": int(len(array)),
            "type": kind,
        }
        if bounds:
            low = array.min(axis=0)
            high = array.max(axis=0)
            accessor["min"] = [float(v) for v in np.atleast_1d(low)]
            accessor["max"] = [float(v) for v in np.atleast_1d(high)]
        self.accessors.append(accessor)
        return len(self.accessors) - 1

    def build(self):
        self.make_nodes()
        self.build_meshes()
        self.build_skins()
        self.build_cameras()
        self.build_materials()
        self.build_animations()
        return self.document(), bytes(self.binary)

    def make_nodes(self):
        model = self.model
        for source in model.nodes:
            node = GNode()
            node.name = source.name
            node.translation = tuple(float(v) for v in source.translation)
            node.rotation = unit_quaternion(source.rotation)
            node.scale = tuple(float(v) for v in source.scale)
            node.parent = source.parent
            node.children = list(source.children)
            node.mesh = source.mesh
            node.camera = source.camera
            node.skeleton = source.skeleton
            self.nodes.append(node)
        self.roots = list(model.roots)
        if model.nodes:
            return
        for index, mesh in enumerate(model.meshes):
            node = GNode()
            node.name = mesh.name
            node.mesh = index
            if len(model.skeletons) == 1 and any(
                model.surfaces[p.source].has_skin for p in mesh.primitives
            ):
                node.skeleton = 0
            self.roots.append(len(self.nodes))
            self.nodes.append(node)

    def skinned_meshes(self):
        skinned = set()
        for node in self.nodes:
            if node.mesh >= 0 and 0 <= node.skeleton < len(self.model.skeletons):
                skinned.add(node.mesh)
        return skinned

    def surface_attributes(self, surface_index, with_skin):
        key = (surface_index, with_skin)
        cached = self.surface_cache.get(key)
        if cached is not None:
            return cached
        data = read_surface(self.model, surface_index)
        if not bool(np.all(np.isfinite(data.positions))):
            raise ConvertError("surface %d has positions that are not finite" % surface_index)
        attributes = {
            "POSITION": self.add_accessor(
                data.positions, "VEC3", COMPONENT_FLOAT, TARGET_ARRAY_BUFFER, bounds=True
            )
        }
        if data.normals is not None:
            attributes["NORMAL"] = self.add_accessor(data.normals, "VEC3", COMPONENT_FLOAT, TARGET_ARRAY_BUFFER)
        if data.tangents is not None:
            attributes["TANGENT"] = self.add_accessor(data.tangents, "VEC4", COMPONENT_FLOAT, TARGET_ARRAY_BUFFER)
        if data.uvs is not None:
            attributes["TEXCOORD_0"] = self.add_accessor(data.uvs, "VEC2", COMPONENT_FLOAT, TARGET_ARRAY_BUFFER)
        if data.uvs2 is not None:
            attributes["TEXCOORD_1"] = self.add_accessor(data.uvs2, "VEC2", COMPONENT_FLOAT, TARGET_ARRAY_BUFFER)
        if data.colors is not None:
            attributes["COLOR_0"] = self.add_accessor(data.colors, "VEC4", COMPONENT_FLOAT, TARGET_ARRAY_BUFFER)
        if with_skin and data.joints is not None:
            joints = np.stack(
                [(data.joints >> np.uint32(8 * k)) & np.uint32(0xFF) for k in range(4)], axis=1
            )
            attributes["JOINTS_0"] = self.add_accessor(joints, "VEC4", COMPONENT_UINT8, TARGET_ARRAY_BUFFER)
            attributes["WEIGHTS_0"] = self.add_accessor(
                data.weights, "VEC4", COMPONENT_FLOAT, TARGET_ARRAY_BUFFER
            )
            self.check_joint_range(surface_index, joints)
        indices = None
        if data.indices is not None:
            component = COMPONENT_UINT16 if int(data.indices.max()) < 0xFFFF else COMPONENT_UINT32
            indices = self.add_accessor(data.indices, "SCALAR", component, TARGET_ELEMENT_ARRAY_BUFFER)
        cached = (attributes, indices)
        self.surface_cache[key] = cached
        return cached

    def check_joint_range(self, surface_index, joints):
        limit = min(
            (
                len(self.model.skeletons[node.skeleton].bones)
                for node in self.nodes
                if node.mesh >= 0 and 0 <= node.skeleton < len(self.model.skeletons)
            ),
            default=0,
        )
        if int(joints.max()) >= limit:
            raise ConvertError(
                "surface %d refers to bone %d but its skeleton has %d bones"
                % (surface_index, int(joints.max()), limit)
            )

    def build_meshes(self):
        skinned = self.skinned_meshes()
        for index, mesh in enumerate(self.model.meshes):
            primitives = []
            for primitive in mesh.primitives:
                mode = TOPOLOGY_MODES.get(primitive.topology)
                if mode is None:
                    self.warnings.append(
                        "mesh '%s' has a sub-mesh with a topology that glTF cannot hold; skipped" % mesh.name
                    )
                    continue
                attributes, indices = self.surface_attributes(primitive.source, index in skinned)
                entry = {"attributes": dict(attributes), "mode": mode}
                if indices is not None:
                    entry["indices"] = indices
                if primitive.material >= 0:
                    entry["material"] = primitive.material
                primitives.append(entry)
            if primitives:
                self.mesh_map.append(len(self.meshes))
                self.meshes.append({"name": mesh.name, "primitives": primitives})
            else:
                self.mesh_map.append(-1)

    def find_joint_node(self, name, parent_node, used, by_name):
        for candidate in by_name.get(name, ()):
            if candidate in used:
                continue
            if parent_node >= 0 and self.nodes[candidate].parent != parent_node:
                continue
            return candidate
        return -1

    def add_bone_node(self, bone, parent_node):
        node = GNode()
        node.name = bone.name
        pose = m.pose_from_matrix(bone.bind_local)
        if pose is None:
            self.warnings.append("bone '%s' has a bind pose that is not an affine transform" % bone.name)
        else:
            node.translation = tuple(float(v) for v in pose[0])
            node.rotation = unit_quaternion(pose[1])
            node.scale = tuple(float(v) for v in pose[2])
        node.parent = parent_node
        index = len(self.nodes)
        self.nodes.append(node)
        if parent_node >= 0:
            self.nodes[parent_node].children.append(index)
        else:
            self.roots.append(index)
        return index

    def root_parent(self, bone):
        try:
            bind_world = np.linalg.inv(matrix_of(bone.inverse_bind))
            parent = bind_world @ np.linalg.inv(matrix_of(bone.bind_local))
        except np.linalg.LinAlgError:
            return -1
        tolerance = PARENT_TOLERANCE * max(1.0, float(np.abs(bind_world[:3, 3]).max()))
        if float(np.abs(parent - np.eye(4)).max()) < tolerance:
            return -1
        for known, index in self.root_parents:
            if float(np.abs(known - parent).max()) < tolerance:
                return index
        pose = m.pose_from_matrix(parent.T.reshape(16).astype(np.float32))
        if pose is None:
            return -1
        node = GNode()
        node.name = "skeleton_root"
        node.translation = tuple(float(v) for v in pose[0])
        node.rotation = unit_quaternion(pose[1])
        node.scale = tuple(float(v) for v in pose[2])
        index = len(self.nodes)
        self.nodes.append(node)
        self.roots.append(index)
        self.root_parents.append((parent, index))
        return index

    def build_skins(self):
        by_name = {}
        for index, node in enumerate(self.nodes):
            if node.name:
                by_name.setdefault(node.name, []).append(index)
        for skeleton in self.model.skeletons:
            bones = skeleton.bones
            if not bones:
                raise ConvertError("a linked skeleton has no bones")
            joints = [-1] * len(bones)
            used = set()
            for b in bone_order(skeleton):
                bone = bones[b]
                parent_node = joints[bone.parent] if bone.parent >= 0 else -1
                found = self.find_joint_node(bone.name, parent_node, used, by_name)
                if found < 0:
                    if bone.parent < 0:
                        parent_node = self.root_parent(bone)
                    found = self.add_bone_node(bone, parent_node)
                used.add(found)
                joints[b] = found
            inverse = np.stack([np.asarray(bone.inverse_bind, dtype=np.float32) for bone in bones])
            self.skins.append(
                {
                    "joints": joints,
                    "inverseBindMatrices": self.add_accessor(inverse, "MAT4", COMPONENT_FLOAT),
                }
            )
            self.joint_nodes.append(joints)

    def build_cameras(self):
        for camera in self.model.cameras:
            if camera.perspective:
                data = {
                    "yfov": float(camera.yfov) if camera.yfov > 0.0 else 0.9,
                    "znear": float(camera.z_near) if camera.z_near > 0.0 else 0.1,
                }
                if camera.aspect > 0.0:
                    data["aspectRatio"] = float(camera.aspect)
                if camera.z_far > 0.0:
                    data["zfar"] = float(camera.z_far)
                self.cameras.append({"type": "perspective", "perspective": data})
            else:
                self.cameras.append(
                    {
                        "type": "orthographic",
                        "orthographic": {
                            "xmag": float(camera.xmag),
                            "ymag": float(camera.ymag),
                            "znear": float(camera.z_near),
                            "zfar": float(camera.z_far),
                        },
                    }
                )

    def texture_path(self, definition, key):
        if definition is None:
            return None
        value = definition.values.get(key)
        if value is None:
            return None
        if isinstance(value, list):
            self.warnings.append(
                "material '%s': '%s' is a texture sequence, only its first file is used"
                % (definition.name, key)
            )
            value = value[0]
        normalized = value.replace("\\", "/")
        path = normalized if os.path.isabs(normalized) else os.path.join(self.material_directory, normalized)
        path = os.path.normpath(path)
        if path not in self.texture_of_path:
            if os.path.isfile(path):
                self.images.append({"uri": quote(path.replace(os.sep, "/"), safe="/:")})
                self.textures.append({"source": len(self.images) - 1})
                self.texture_of_path[path] = len(self.textures) - 1
            else:
                self.warnings.append("texture '%s' is missing, skipped" % value)
                self.texture_of_path[path] = -1
        return path if self.texture_of_path[path] >= 0 else None

    def texture_ref(self, path, extra=None):
        reference = {"index": self.texture_of_path[path]}
        if extra:
            reference.update(extra)
        return reference

    def build_materials(self):
        for slot, name in enumerate(self.model.material_slots):
            self.materials.append(self.build_material(name if name else "material_%d" % slot))

    def build_material(self, name):
        definition = find_material(self.library, name) if self.library is not None else None
        values = definition.values if definition is not None else {}
        entry = {"name": name}
        pbr = {}

        color = values.get("color", (1.0, 1.0, 1.0, 1.0))
        pbr["baseColorFactor"] = [clamp01(v) for v in color]
        pbr["metallicFactor"] = clamp01(values.get("metallic", 0.0))
        pbr["roughnessFactor"] = clamp01(values.get("roughness", 1.0))

        albedo = self.texture_path(definition, "albedo")
        if albedo:
            pbr["baseColorTexture"] = self.texture_ref(albedo)

        metallic_map = self.texture_path(definition, "metallic_map")
        roughness_map = self.texture_path(definition, "roughness_map")
        orm = self.texture_path(definition, "orm")
        occlusion = self.texture_path(definition, "occlusion")
        orm_only = orm is not None and metallic_map is None and roughness_map is None
        metallic_source = metallic_map or orm
        roughness_source = roughness_map or orm
        occlusion_source = occlusion or orm
        metallic_channel = "Blue" if orm_only else values.get("metallic_channel", "Red")
        roughness_channel = "Green" if orm_only else values.get("roughness_channel", "Red")
        occlusion_channel = "Red" if orm_only else values.get("occlusion_channel", "Red")

        if metallic_source or roughness_source:
            if (
                metallic_source == roughness_source
                and metallic_channel == "Blue"
                and roughness_channel == "Green"
            ):
                pbr["metallicRoughnessTexture"] = self.texture_ref(metallic_source)
            else:
                self.warnings.append(
                    "material '%s': the metallic and roughness maps are not one texture with "
                    "metallic in blue and roughness in green, skipped" % name
                )
        entry["pbrMetallicRoughness"] = pbr

        normal = self.texture_path(definition, "normal")
        if normal:
            scale = float(values.get("normal_scale", 1.0))
            entry["normalTexture"] = self.texture_ref(normal, {"scale": scale} if scale != 1.0 else None)

        if occlusion_source:
            if occlusion_channel == "Red":
                entry["occlusionTexture"] = self.texture_ref(occlusion_source)
            else:
                self.warnings.append(
                    "material '%s': glTF reads occlusion from the red channel, the texture is skipped" % name
                )

        self.build_emission(entry, definition, values)

        transparency = values.get("transparency", "None")
        if transparency == "Cutout":
            entry["alphaMode"] = "MASK"
            entry["alphaCutoff"] = float(values.get("alpha_cutoff", 0.5))
        elif transparency == "Blend":
            entry["alphaMode"] = "BLEND"
        cull = values.get("cull", "Back")
        if cull == "None":
            entry["doubleSided"] = True
        elif cull == "Front":
            self.warnings.append("material '%s': glTF cannot cull front faces" % name)
        return entry

    def build_emission(self, entry, definition, values):
        color = [float(v) for v in values.get("emission_color", (0.0, 0.0, 0.0))]
        energy = float(values.get("emission_energy", 1.0))
        texture = self.texture_path(definition, "emission")
        if texture:
            if values.get("emission_operator", "Add") == "Add":
                if any(color):
                    self.warnings.append(
                        "material '%s': the engine adds the emission texture to the color, "
                        "it is multiplied here" % definition.name
                    )
                else:
                    color = [1.0, 1.0, 1.0]
            entry["emissiveTexture"] = self.texture_ref(texture)
        scaled = [max(0.0, v) * max(0.0, energy) for v in color]
        peak = max(scaled)
        if peak <= 0.0:
            return
        if peak > 1.0:
            entry["emissiveFactor"] = [v / peak for v in scaled]
            entry["extensions"] = {EXTENSION_EMISSIVE_STRENGTH: {"emissiveStrength": peak}}
            if EXTENSION_EMISSIVE_STRENGTH not in self.extensions_used:
                self.extensions_used.append(EXTENSION_EMISSIVE_STRENGTH)
        else:
            entry["emissiveFactor"] = scaled

    def snap_times(self, times):
        if not self.frame_rate:
            return times
        frames = times.astype(np.float64) * self.frame_rate
        whole = np.round(frames)
        near = np.abs(frames - whole) < FRAME_SNAP_TOLERANCE
        return np.where(near, whole / self.frame_rate, times).astype(np.float32)

    def build_animations(self):
        for index, clip in enumerate(self.model.animations):
            label = clip.name if clip.name else "animation_%d" % index
            if not 0 <= clip.skeleton < len(self.joint_nodes):
                self.warnings.append("clip '%s' has no skeleton, skipped" % label)
                continue
            joints = self.joint_nodes[clip.skeleton]
            channels = []
            samplers = []
            for track in clip.tracks:
                if track.bone < 0:
                    continue
                count = min(len(track.times), len(track.positions), len(track.rotations), len(track.scales))
                if count == 0:
                    continue
                times = self.snap_times(np.asarray(track.times[:count], dtype=np.float32))
                if count > 1 and bool(np.any(times[1:] < times[:-1])):
                    raise ConvertError("clip '%s' has a bone track with times that are not sorted" % label)
                time_accessor = self.add_accessor(times, "SCALAR", COMPONENT_FLOAT, bounds=True)
                for path, values, kind in (
                    ("translation", track.positions[:count], "VEC3"),
                    ("rotation", unit_quaternions(track.rotations[:count]), "VEC4"),
                    ("scale", track.scales[:count], "VEC3"),
                ):
                    samplers.append(
                        {
                            "input": time_accessor,
                            "output": self.add_accessor(values, kind, COMPONENT_FLOAT),
                            "interpolation": "LINEAR",
                        }
                    )
                    channels.append(
                        {
                            "sampler": len(samplers) - 1,
                            "target": {"node": joints[track.bone], "path": path},
                        }
                    )
            if not channels:
                self.warnings.append("clip '%s' has no complete bone track, skipped" % label)
                continue
            self.animations.append({"name": label, "channels": channels, "samplers": samplers})

    def node_entry(self, node):
        entry = {}
        if node.name:
            entry["name"] = node.name
        if node.translation != (0.0, 0.0, 0.0):
            entry["translation"] = list(node.translation)
        if node.rotation != (0.0, 0.0, 0.0, 1.0):
            entry["rotation"] = list(node.rotation)
        if node.scale != (1.0, 1.0, 1.0):
            entry["scale"] = list(node.scale)
        if node.children:
            entry["children"] = list(node.children)
        if node.mesh >= 0 and self.mesh_map[node.mesh] >= 0:
            entry["mesh"] = self.mesh_map[node.mesh]
            if 0 <= node.skeleton < len(self.skins):
                entry["skin"] = node.skeleton
        if node.camera >= 0:
            entry["camera"] = node.camera
        return entry

    def document(self):
        document = {"asset": {"version": "2.0", "generator": "io_prisma3d"}}
        if self.nodes:
            document["scene"] = 0
            document["scenes"] = [{"name": self.stem, "nodes": list(self.roots)}]
            document["nodes"] = [self.node_entry(node) for node in self.nodes]
        for key, value in (
            ("meshes", self.meshes),
            ("skins", self.skins),
            ("cameras", self.cameras),
            ("materials", self.materials),
            ("textures", self.textures),
            ("images", self.images),
            ("animations", self.animations),
            ("accessors", self.accessors),
            ("bufferViews", self.views),
        ):
            if value:
                document[key] = value
        if self.binary:
            document["buffers"] = [{"uri": quote(self.stem + ".bin"), "byteLength": len(self.binary)}]
        if self.extensions_used:
            document["extensionsUsed"] = list(self.extensions_used)
        return document


def write_gltf(directory, stem, document, binary):
    gltf_path = os.path.join(directory, stem + ".gltf")
    try:
        text = json.dumps(document, allow_nan=False)
    except ValueError:
        raise ConvertError("the model holds values that glTF cannot store")
    with open(gltf_path, "w", encoding="utf-8") as handle:
        handle.write(text)
    if binary:
        with open(os.path.join(directory, stem + ".bin"), "wb") as handle:
            handle.write(binary)
    return gltf_path
