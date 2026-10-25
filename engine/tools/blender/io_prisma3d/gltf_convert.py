import os

import numpy as np

from . import f32 as m
from .gltf_reader import (
    COMPONENT_FLOAT,
    COMPONENT_UINT8,
    COMPONENT_UINT16,
    COMPONENT_UINT32,
    AccessorRef,
    GltfDocument,
    decode_uri,
    is_data_uri,
    unpack_floats,
    unpack_indices,
)
from .model import (
    ALPHA_BLEND,
    ALPHA_MASK,
    ALPHA_OPAQUE,
    TOPOLOGY_LINE_STRIP,
    TOPOLOGY_LINES,
    TOPOLOGY_POINTS,
    TOPOLOGY_TRIANGLE_STRIP,
    TOPOLOGY_TRIANGLES,
    BoneTrack,
    Camera,
    Clip,
    ConvertError,
    Image,
    Material,
    Mesh,
    Model,
    Node,
    Primitive,
    Skeleton,
    Surface,
    Texture,
)

F = np.float32

MODE_TOPOLOGY = {
    0: TOPOLOGY_POINTS,
    1: TOPOLOGY_LINES,
    3: TOPOLOGY_LINE_STRIP,
    4: TOPOLOGY_TRIANGLES,
    5: TOPOLOGY_TRIANGLE_STRIP,
}

ATTRIBUTE_SLOTS = {
    ("POSITION", 0): ("positions", 3),
    ("NORMAL", 0): ("normals", 3),
    ("TANGENT", 0): ("tangents", 4),
    ("COLOR", 0): ("colors", 0),
    ("TEXCOORD", 0): ("uvs", 2),
    ("TEXCOORD", 1): ("uvs2", 2),
    ("JOINTS", 0): ("joints", 4),
    ("WEIGHTS", 0): ("weights", 4),
}


class PrimitiveSource:
    def __init__(self):
        self.positions = AccessorRef()
        self.normals = AccessorRef()
        self.tangents = AccessorRef()
        self.colors = AccessorRef()
        self.uvs = AccessorRef()
        self.uvs2 = AccessorRef()
        self.joints = AccessorRef()
        self.weights = AccessorRef()
        self.indices = AccessorRef()


def parse_attribute(name):
    if not name or name.startswith("_"):
        return None
    head, separator, tail = name.partition("_")
    if not separator:
        return head, 0
    if tail.isdigit():
        return head, int(tail)
    return None


def to_float(value, default=0.0):
    if value is None:
        return F(default)
    return F(value)


def float_list(values, size, default):
    out = [F(default)] * size
    if isinstance(values, list):
        for i in range(min(size, len(values))):
            out[i] = F(values[i])
    return tuple(out)


def index_of(entry, key, count, what):
    value = entry.get(key)
    if value is None:
        return -1
    if not isinstance(value, int) or value < 0 or value >= count:
        raise ConvertError("%s refers to something that does not exist" % what)
    return value


def texture_index(view, count):
    if not isinstance(view, dict):
        return -1
    return index_of(view, "index", count, "material")


class GltfConverter:
    def __init__(self, path):
        self.document = GltfDocument(path)
        self.model = Model()
        self.model.directory = self.document.directory

    def convert(self):
        with np.errstate(all="ignore"):
            self.convert_document()
        return self.model

    def convert_images(self):
        document = self.document
        for source in document.array("images"):
            image = Image()
            image.name = source.get("name", "")
            image.mime_type = source.get("mimeType", "")
            uri = source.get("uri")
            if "bufferView" in source:
                raise ConvertError("embedded images are not supported")
            if uri is not None:
                if is_data_uri(uri):
                    raise ConvertError("data URIs are not supported")
                image.path = os.path.join(document.directory, decode_uri(uri))
            else:
                raise ConvertError("image without source")
            self.model.images.append(image)

    def convert_textures(self):
        image_count = len(self.document.array("images"))
        for source in self.document.array("textures"):
            texture = Texture()
            texture.image = index_of(source, "source", image_count, "texture")
            self.model.textures.append(texture)

    def convert_materials(self):
        texture_count = len(self.document.array("textures"))
        for source in self.document.array("materials"):
            material = Material()
            material.name = source.get("name", "")
            if "pbrMetallicRoughness" in source:
                pbr = source["pbrMetallicRoughness"]
                material.base_color = float_list(pbr.get("baseColorFactor"), 4, 1.0)
                material.metallic = to_float(pbr.get("metallicFactor"), 1.0)
                material.roughness = to_float(pbr.get("roughnessFactor"), 1.0)
                material.base_color_texture = texture_index(pbr.get("baseColorTexture"), texture_count)
                material.metallic_roughness_texture = texture_index(
                    pbr.get("metallicRoughnessTexture"), texture_count
                )
            material.emissive = float_list(source.get("emissiveFactor"), 3, 0.0)
            material.alpha_cutoff = to_float(source.get("alphaCutoff"), 0.5)
            mode = source.get("alphaMode")
            if mode == "MASK":
                material.alpha_mode = ALPHA_MASK
            elif mode == "BLEND":
                material.alpha_mode = ALPHA_BLEND
            else:
                material.alpha_mode = ALPHA_OPAQUE
            material.double_sided = bool(source.get("doubleSided", False))
            material.normal_texture = texture_index(source.get("normalTexture"), texture_count)
            material.occlusion_texture = texture_index(source.get("occlusionTexture"), texture_count)
            material.emissive_texture = texture_index(source.get("emissiveTexture"), texture_count)
            self.model.materials.append(material)

    def convert_primitive(self, source):
        document = self.document
        primitive = Primitive()
        accessors = PrimitiveSource()
        if "KHR_draco_mesh_compression" in source.get("extensions", {}):
            raise ConvertError("compressed primitives are not supported")
        mode = source.get("mode", 4)
        if mode not in MODE_TOPOLOGY:
            raise ConvertError("unsupported primitive topology")
        primitive.topology = MODE_TOPOLOGY[mode]
        primitive.material = index_of(source, "material", len(document.array("materials")), "primitive")

        has_position = False
        for name, accessor_index in source.get("attributes", {}).items():
            parsed = parse_attribute(name)
            if parsed is None or parsed not in ATTRIBUTE_SLOTS:
                continue
            slot, expected = ATTRIBUTE_SLOTS[parsed]
            if parsed == ("POSITION", 0):
                has_position = True
            if accessor_index is None:
                raise ConvertError("attribute without accessor")
            target = document.accessor_ref(accessor_index)
            setattr(accessors, slot, target)
            if slot == "colors":
                if target.components != 3 and target.components != 4:
                    raise ConvertError("unexpected attribute type")
            elif target.components != expected:
                raise ConvertError("unexpected attribute type")
            if slot == "joints":
                if target.component_type not in (COMPONENT_UINT8, COMPONENT_UINT16):
                    raise ConvertError("unsupported joint index type")
            if slot == "weights":
                if target.component_type not in (COMPONENT_FLOAT, COMPONENT_UINT8, COMPONENT_UINT16):
                    raise ConvertError("unsupported joint weight type")
            if slot == "positions":
                accessor = document.array("accessors")[accessor_index]
                if "min" in accessor and "max" in accessor:
                    primitive.aabb_min = np.array(
                        (list(accessor["min"]) + [0.0, 0.0, 0.0])[:3], dtype=np.float32
                    )
                    primitive.aabb_max = np.array(
                        (list(accessor["max"]) + [0.0, 0.0, 0.0])[:3], dtype=np.float32
                    )
                    primitive.has_aabb = True
        if not has_position or not accessors.positions.valid():
            raise ConvertError("primitive without POSITION")
        if not accessors.joints.valid() or not accessors.weights.valid():
            accessors.joints = AccessorRef()
            accessors.weights = AccessorRef()
        if source.get("indices") is not None:
            accessors.indices = document.accessor_ref(source["indices"])
            if accessors.indices.components != 1 or accessors.indices.component_type not in (
                COMPONENT_UINT8,
                COMPONENT_UINT16,
                COMPONENT_UINT32,
            ):
                raise ConvertError("unsupported index accessor")
        primitive.has_joints = accessors.joints.valid()
        primitive.source = accessors
        return primitive

    def convert_meshes(self):
        for source in self.document.array("meshes"):
            mesh = Mesh()
            mesh.name = source.get("name", "")
            for primitive_source in source.get("primitives", []):
                mesh.primitives.append(self.convert_primitive(primitive_source))
            self.model.meshes.append(mesh)

    def convert_cameras(self):
        for source in self.document.array("cameras"):
            camera = Camera()
            if "perspective" in source and "orthographic" not in source:
                data = source["perspective"]
                camera.perspective = True
                camera.yfov = to_float(data.get("yfov"))
                camera.aspect = to_float(data.get("aspectRatio"))
                camera.z_near = to_float(data.get("znear"))
                if "zfar" in data and F(data["zfar"]) > F(0.0):
                    camera.z_far = F(data["zfar"])
            elif "orthographic" in source and "perspective" not in source:
                data = source["orthographic"]
                camera.perspective = False
                camera.xmag = to_float(data.get("xmag"))
                camera.ymag = to_float(data.get("ymag"))
                camera.z_near = to_float(data.get("znear"))
                camera.z_far = to_float(data.get("zfar"))
            else:
                raise ConvertError("unsupported camera type")
            self.model.cameras.append(camera)

    def convert_nodes(self):
        document = self.document
        model = self.model
        node_count = len(document.array("nodes"))
        mesh_count = len(document.array("meshes"))
        camera_count = len(document.array("cameras"))
        for source in document.array("nodes"):
            node = Node()
            node.name = source.get("name", "")
            if "matrix" in source:
                values = float_list(source["matrix"], 16, 0.0)
                matrix = np.array(values, dtype=np.float32)
                node.translation, node.rotation, node.scale = m.decompose_matrix(matrix)
            else:
                if "translation" in source:
                    node.translation = float_list(source["translation"], 3, 0.0)
                if "rotation" in source:
                    rotation = float_list(source["rotation"], 4, 0.0)
                    if m.quat_length_squared(rotation) > m.QUAT_EPSILON:
                        node.rotation = m.quat_normalized(rotation)
                if "scale" in source:
                    node.scale = float_list(source["scale"], 3, 0.0)
            node.mesh = index_of(source, "mesh", mesh_count, "node")
            node.camera = index_of(source, "camera", camera_count, "node")
            for child in source.get("children", []):
                if not isinstance(child, int) or child < 0 or child >= node_count:
                    raise ConvertError("node refers to something that does not exist")
                node.children.append(child)
            model.nodes.append(node)

        for i, node in enumerate(model.nodes):
            for child_index in node.children:
                child = model.nodes[child_index]
                if child.parent >= 0:
                    raise ConvertError("node has several parents")
                child.parent = i

        stamp = [-1] * len(model.nodes)
        for i in range(len(model.nodes)):
            current = i
            while current >= 0 and stamp[current] == -1:
                stamp[current] = i
                current = model.nodes[current].parent
            if current >= 0 and stamp[current] == i:
                raise ConvertError("node hierarchy has a cycle")

    def convert_skins(self):
        document = self.document
        model = self.model
        joint_bones = []
        skins = document.array("skins")
        node_count = len(model.nodes)
        for skin in skins:
            joints = skin.get("joints", [])
            if len(joints) == 0:
                raise ConvertError("skin without joints")
            bone_of = [-1] * node_count
            for j, node_index in enumerate(joints):
                if not isinstance(node_index, int) or node_index < 0 or node_index >= node_count:
                    raise ConvertError("skin refers to a node that does not exist")
                if bone_of[node_index] >= 0:
                    raise ConvertError("skin lists a joint twice")
                bone_of[node_index] = j
            joint_bones.append(bone_of)

            inverse_binds = None
            if skin.get("inverseBindMatrices") is not None:
                ref = document.accessor_ref(skin["inverseBindMatrices"])
                if (
                    ref.components != 16
                    or ref.component_type != COMPONENT_FLOAT
                    or ref.count < len(joints)
                ):
                    raise ConvertError("unexpected inverse bind matrices")
                inverse_binds = unpack_floats(document, ref, 16)

            skeleton = Skeleton()
            for j, node_index in enumerate(joints):
                node = model.nodes[node_index]
                name = node.name if node.name else "joint_%d" % j
                if skeleton.find_bone(name) >= 0:
                    name = name + "_%d" % j
                parent = -1
                if node.parent >= 0:
                    parent = bone_of[node.parent]
                if inverse_binds is not None:
                    inverse_bind = inverse_binds[j].copy()
                else:
                    inverse_bind = np.zeros(16, dtype=np.float32)
                    inverse_bind[0] = inverse_bind[5] = inverse_bind[10] = inverse_bind[15] = 1.0
                bind_local = m.compose_local(node.translation, node.rotation, node.scale)
                if not skeleton.add_bone(name, parent, bind_local, inverse_bind):
                    raise ConvertError("invalid skin joint")
            if not skeleton.finalize():
                raise ConvertError("skin joints do not form a hierarchy")
            model.skeletons.append(skeleton)

        for i, source in enumerate(document.array("nodes")):
            if source.get("skin") is not None and source.get("mesh") is not None:
                skin_index = source["skin"]
                if not isinstance(skin_index, int) or skin_index < 0 or skin_index >= len(skins):
                    raise ConvertError("node refers to something that does not exist")
                model.nodes[i].skeleton = skin_index
        for node in model.nodes:
            if node.skeleton < 0 or node.mesh < 0:
                continue
            for primitive in model.meshes[node.mesh].primitives:
                if primitive.has_joints:
                    primitive.skinned = True
        return joint_bones

    def read_channel(self, channel, sampler, raw):
        document = self.document
        input_ref = document.accessor_ref(sampler.get("input"))
        output_ref = document.accessor_ref(sampler.get("output"))
        rotation = channel["target"]["path"] == "rotation"
        output_components = 4 if rotation else 3
        if input_ref.components != 1 or output_ref.components != output_components:
            raise ConvertError("unexpected animation sampler type")
        cubic = sampler.get("interpolation") == "CUBICSPLINE"
        steps_per_key = 3 if cubic else 1
        value_offset = 1 if cubic else 0
        keys = input_ref.count
        if output_ref.count // steps_per_key < keys:
            keys = output_ref.count // steps_per_key
        if keys == 0:
            return

        input_ref.count = keys
        times = unpack_floats(document, input_ref, 1)[:, 0]
        output_ref.count = keys * steps_per_key
        values = unpack_floats(document, output_ref, output_components)
        if keys > 1 and bool(np.any(times[1:] < times[:-1])):
            raise ConvertError("animation times are not sorted")
        values = values[value_offset::steps_per_key][:keys]

        path = channel["target"]["path"]
        if path == "translation":
            raw.position_times = times
            raw.positions = values
        elif path == "scale":
            raw.scale_times = times
            raw.scales = values
        else:
            length_squared = (
                (values[:, 0] * values[:, 0] + values[:, 1] * values[:, 1]) + values[:, 2] * values[:, 2]
            ) + values[:, 3] * values[:, 3]
            inv = (m.ONE / np.sqrt(length_squared)).astype(np.float32)
            normalized = (values * inv[:, None]).astype(np.float32)
            identity = np.array(m.IDENTITY_QUAT, dtype=np.float32)
            raw.rotation_times = times
            raw.rotations = np.where(
                (length_squared > m.QUAT_EPSILON)[:, None], normalized, identity[None, :]
            ).astype(np.float32)

    def convert_animations(self, joint_bones):
        document = self.document
        model = self.model
        nodes_count = len(model.nodes)
        for a, animation in enumerate(document.array("animations")):
            channels = animation.get("channels", [])
            samplers = animation.get("samplers", [])

            votes = [0] * len(model.skeletons)
            for channel in channels:
                node = channel.get("target", {}).get("node")
                if channel.get("sampler") is None or node is None:
                    continue
                if not isinstance(node, int) or node < 0 or node >= nodes_count:
                    raise ConvertError("animation refers to a node that does not exist")
                for s in range(len(votes)):
                    if joint_bones[s][node] >= 0:
                        votes[s] += 1
            skin = -1
            for s in range(len(votes)):
                if votes[s] > 0 and (skin < 0 or votes[s] > votes[skin]):
                    skin = s
            if skin < 0:
                continue

            skeleton = model.skeletons[skin]
            bone_of = joint_bones[skin]
            raw = [RawChannels() for _ in skeleton.bones]
            for channel in channels:
                target = channel.get("target", {})
                node = target.get("node")
                if channel.get("sampler") is None or node is None:
                    continue
                bone = bone_of[node]
                if bone < 0:
                    continue
                path = target.get("path")
                entry = raw[bone]
                if path == "translation":
                    if entry.has_position:
                        continue
                    entry.has_position = True
                elif path == "rotation":
                    if entry.has_rotation:
                        continue
                    entry.has_rotation = True
                elif path == "scale":
                    if entry.has_scale:
                        continue
                    entry.has_scale = True
                else:
                    continue
                sampler_index = channel["sampler"]
                if not isinstance(sampler_index, int) or sampler_index < 0 or sampler_index >= len(samplers):
                    raise ConvertError("animation sampler does not exist")
                self.read_channel(channel, samplers[sampler_index], entry)

            any_key = False
            start = m.ZERO
            for entry in raw:
                for times in (entry.position_times, entry.rotation_times, entry.scale_times):
                    if len(times) == 0:
                        continue
                    if not any_key or times[0] < start:
                        start = times[0]
                    any_key = True
            if not any_key:
                continue

            bind_pose = []
            for bone in skeleton.bones:
                pose = m.pose_from_matrix(bone.bind_local)
                if pose is None:
                    pose = ((m.ZERO, m.ZERO, m.ZERO), m.IDENTITY_QUAT, (m.ONE, m.ONE, m.ONE))
                bind_pose.append(pose)

            clip = Clip()
            name = animation.get("name")
            clip.name = name if name else "animation_%d" % a
            clip.skeleton = skin
            duration = m.ZERO
            for b, entry in enumerate(raw):
                unified = np.sort(
                    np.concatenate((entry.position_times, entry.rotation_times, entry.scale_times))
                ).astype(np.float32)
                if len(unified) == 0:
                    continue
                kept = []
                for value in unified:
                    if not kept or value - kept[-1] >= m.SPAN_EPSILON:
                        kept.append(value)
                times = np.array(kept, dtype=np.float32)

                track = BoneTrack(b)
                bind_position, bind_rotation, bind_scale = bind_pose[b]
                positions = m.sample_vec3(entry.position_times, entry.positions, times)
                if positions is None:
                    positions = np.tile(np.array(bind_position, dtype=np.float32), (len(times), 1))
                rotations = m.sample_quaternion(entry.rotation_times, entry.rotations, times)
                if rotations is None:
                    rotations = np.tile(np.array(bind_rotation, dtype=np.float32), (len(times), 1))
                scales = m.sample_vec3(entry.scale_times, entry.scales, times)
                if scales is None:
                    scales = np.tile(np.array(bind_scale, dtype=np.float32), (len(times), 1))
                track.positions = positions
                track.rotations = rotations
                track.scales = scales
                track.times = (times - start).astype(np.float32)
                last = track.times[-1]
                if last > duration:
                    duration = last
                clip.tracks.append(track)
            clip.duration = duration
            model.animations.append(clip)

    def convert_document(self):
        model = self.model
        self.convert_images()
        self.convert_textures()
        self.convert_materials()
        self.convert_meshes()
        self.convert_cameras()
        self.convert_nodes()

        if len(self.document.array("skins")) > 0:
            joint_bones = self.convert_skins()
            self.convert_animations(joint_bones)

        scenes = self.document.array("scenes")
        scene = None
        scene_index = self.document.json.get("scene")
        if scene_index is not None:
            if not isinstance(scene_index, int) or scene_index < 0 or scene_index >= len(scenes):
                raise ConvertError("the scene does not exist")
            scene = scenes[scene_index]
        elif scenes:
            scene = scenes[0]
        if scene is not None:
            for node_index in scene.get("nodes", []):
                if not isinstance(node_index, int) or node_index < 0 or node_index >= len(model.nodes):
                    raise ConvertError("a root node does not exist")
                model.roots.append(node_index)
        else:
            for i, node in enumerate(model.nodes):
                if node.parent < 0:
                    model.roots.append(i)

    def read_primitive(self, primitive):
        document = self.document
        source = primitive.source
        surface = Surface()
        surface.topology = primitive.topology

        if not source.positions.valid():
            raise ConvertError("could not read a primitive")
        vertex_count = source.positions.count

        def attribute(ref, components):
            if ref.count != vertex_count:
                raise ConvertError("a primitive has an attribute with the wrong count")
            return unpack_floats(document, ref, components)

        with np.errstate(all="ignore"):
            surface.positions = attribute(source.positions, 3)
            if source.normals.valid():
                surface.normals = attribute(source.normals, 3)
            if source.tangents.valid():
                surface.tangents = attribute(source.tangents, 4)
            if source.colors.valid():
                surface.colors = attribute(source.colors, 4)
                if source.colors.components == 3:
                    surface.colors[:, 3] = 1.0
            if source.uvs.valid():
                surface.uvs = attribute(source.uvs, 2)
            if source.uvs2.valid():
                surface.uvs2 = attribute(source.uvs2, 2)
            if source.joints.valid() and source.weights.valid():
                joints = attribute(source.joints, 4)
                if bool(np.any(joints < 0.0)) or bool(np.any(joints > 255.0)):
                    raise ConvertError("a joint index is out of range")
                packed = joints.astype(np.uint32)
                surface.joints = (
                    packed[:, 0]
                    | (packed[:, 1] << np.uint32(8))
                    | (packed[:, 2] << np.uint32(16))
                    | (packed[:, 3] << np.uint32(24))
                ).astype(np.uint32)

                weights = attribute(source.weights, 4)
                total = ((weights[:, 0] + weights[:, 1]) + weights[:, 2]) + weights[:, 3]
                safe = np.where(total > m.ZERO, total, m.ONE).astype(np.float32)
                divided = (weights / safe[:, None]).astype(np.float32)
                surface.weights = np.where((total > m.ZERO)[:, None], divided, weights).astype(np.float32)
            if source.indices.valid():
                surface.indices = unpack_indices(document, source.indices)
            if surface.normals is None and surface.topology == TOPOLOGY_TRIANGLES:
                surface = flat_normals(surface)
        return surface


class RawChannels:
    def __init__(self):
        self.position_times = np.zeros(0, dtype=np.float32)
        self.positions = np.zeros((0, 3), dtype=np.float32)
        self.rotation_times = np.zeros(0, dtype=np.float32)
        self.rotations = np.zeros((0, 4), dtype=np.float32)
        self.scale_times = np.zeros(0, dtype=np.float32)
        self.scales = np.zeros((0, 3), dtype=np.float32)
        self.has_position = False
        self.has_rotation = False
        self.has_scale = False


def flat_normals(surface):
    vertex_count = len(surface.positions)
    corners = vertex_count if surface.indices is None else len(surface.indices)
    if corners < 3:
        return surface
    count = corners - corners % 3
    if surface.indices is None:
        order = np.arange(count, dtype=np.uint32)
    else:
        order = surface.indices[:count]
    triangles = order.reshape(-1, 3)
    triangles = triangles[np.all(triangles < vertex_count, axis=1)]
    flat = Surface()
    flat.topology = surface.topology
    gather = triangles.reshape(-1)
    positions = surface.positions
    a = positions[triangles[:, 0]]
    e1 = positions[triangles[:, 1]] - a
    e2 = positions[triangles[:, 2]] - a
    cross = np.stack(
        (
            e1[:, 1] * e2[:, 2] - e1[:, 2] * e2[:, 1],
            e1[:, 2] * e2[:, 0] - e1[:, 0] * e2[:, 2],
            e1[:, 0] * e2[:, 1] - e1[:, 1] * e2[:, 0],
        ),
        axis=1,
    ).astype(np.float32)
    length_squared = (cross[:, 0] * cross[:, 0] + cross[:, 1] * cross[:, 1]) + cross[:, 2] * cross[:, 2]
    length_squared = np.maximum(length_squared, m.EPSILON_SQUARED).astype(np.float32)
    inv = (m.ONE / np.sqrt(length_squared)).astype(np.float32)
    normals = (cross * inv[:, None]).astype(np.float32)

    flat.positions = positions[gather]
    flat.normals = np.repeat(normals, 3, axis=0)
    flat.indices = np.arange(len(gather), dtype=np.uint32)
    for name in ("tangents", "colors", "uvs", "uvs2", "joints", "weights"):
        values = getattr(surface, name)
        if values is not None:
            setattr(flat, name, values[gather])
    return flat
