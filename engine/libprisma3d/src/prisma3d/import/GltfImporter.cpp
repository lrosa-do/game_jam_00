#include "prisma3d/import/GltfImporter.h"

#include "prisma3d/core/FileSystem.h"

#include "platform.h"

#include "cgltf.h"

#include <ct/sort.hpp>

#include <stdio.h>
#include <string.h>

namespace prisma3d
{

namespace
{

static_assert(sizeof(Math::Vec2) == 2 * sizeof(float), "Vec2 must be tightly packed");
static_assert(sizeof(Math::Vec3) == 3 * sizeof(float), "Vec3 must be tightly packed");
static_assert(sizeof(Math::Vec4) == 4 * sizeof(float), "Vec4 must be tightly packed");
static_assert(sizeof(Math::Mat4) == 16 * sizeof(float), "Mat4 must be tightly packed");

const std::uint32_t kGlbMagic = 0x46546C67u;
const std::uint32_t kChunkJson = 0x4E4F534Au;
const std::uint32_t kChunkBin = 0x004E4942u;
const std::uint32_t kGlbVersion = 2u;
const std::uint64_t kGlbHeaderSize = 12;
const std::uint64_t kChunkHeaderSize = 8;
const std::uint64_t kMaxJsonSize = 512ull * 1024 * 1024;
const int kWrapRepeat = 10497;
const int kFilterNearest = 9728;

std::uint32_t readU32(const std::uint8_t* bytes)
{
    std::uint32_t value;
    memcpy(&value, bytes, sizeof(value));
    return value;
}

bool fail(ModelData& out, const char* message)
{
    out.error = message;
    return false;
}

struct GlbLayout
{
    bool glb = false;
    std::uint64_t jsonOffset = 0;
    std::uint64_t jsonSize = 0;
    bool hasBin = false;
    std::uint64_t binOffset = 0;
    std::uint64_t binSize = 0;
};

bool parseGlbLayout(const std::uint8_t* head, std::uint64_t available, std::uint64_t total,
        GlbLayout& layout, const char*& error)
{
    layout = GlbLayout();
    if (available < kGlbHeaderSize || readU32(head) != kGlbMagic) return true;
    layout.glb = true;
    if (readU32(head + 4) != kGlbVersion)
    {
        error = "unsupported GLB version";
        return false;
    }
    if (available < kGlbHeaderSize + kChunkHeaderSize)
    {
        error = "GLB is truncated";
        return false;
    }
    if (readU32(head + 16) != kChunkJson)
    {
        error = "GLB first chunk is not JSON";
        return false;
    }
    layout.jsonOffset = kGlbHeaderSize + kChunkHeaderSize;
    layout.jsonSize = readU32(head + 12);
    if (layout.jsonSize > kMaxJsonSize || layout.jsonOffset + layout.jsonSize > total)
    {
        error = "GLB JSON chunk exceeds the data";
        return false;
    }
    return true;
}

bool parseBinChunk(const std::uint8_t* header, std::uint64_t total, GlbLayout& layout,
        const char*& error)
{
    if (readU32(header + 4) != kChunkBin) return true;
    layout.hasBin = true;
    layout.binSize = readU32(header);
    layout.binOffset = layout.jsonOffset + layout.jsonSize + kChunkHeaderSize;
    if (layout.binOffset + layout.binSize > total)
    {
        error = "GLB BIN chunk exceeds the data";
        return false;
    }
    return true;
}

ct::String decodeUri(const char* uri)
{
    ct::String result;
    const std::size_t length = strlen(uri);
    for (std::size_t i = 0; i < length; ++i)
    {
        if (uri[i] == '%' && i + 2 < length)
        {
            int value = 0;
            bool valid = true;
            for (int k = 1; k <= 2; ++k)
            {
                const char c = uri[i + k];
                int digit;
                if (c >= '0' && c <= '9') digit = c - '0';
                else if (c >= 'a' && c <= 'f')
                    digit = c - 'a' + 10;
                else if (c >= 'A' && c <= 'F')
                    digit = c - 'A' + 10;
                else
                {
                    valid = false;
                    break;
                }
                value = value * 16 + digit;
            }
            if (valid)
            {
                result.push_back(static_cast<char>(value));
                i += 2;
                continue;
            }
        }
        result.push_back(uri[i]);
    }
    return result;
}

bool isDataUri(const char* uri)
{
    return strncmp(uri, "data:", 5) == 0;
}

GltfComponentType convertComponentType(cgltf_component_type type)
{
    switch (type)
    {
        case cgltf_component_type_r_8:
            return GltfComponentType::Int8;
        case cgltf_component_type_r_8u:
            return GltfComponentType::UInt8;
        case cgltf_component_type_r_16:
            return GltfComponentType::Int16;
        case cgltf_component_type_r_16u:
            return GltfComponentType::UInt16;
        case cgltf_component_type_r_32u:
            return GltfComponentType::UInt32;
        case cgltf_component_type_r_32f:
            return GltfComponentType::Float;
        default:
            return GltfComponentType::Invalid;
    }
}

int componentCount(cgltf_type type)
{
    switch (type)
    {
        case cgltf_type_scalar:
            return 1;
        case cgltf_type_vec2:
            return 2;
        case cgltf_type_vec3:
            return 3;
        case cgltf_type_vec4:
            return 4;
        case cgltf_type_mat4:
            return 16;
        default:
            return 0;
    }
}

bool convertAccessor(const cgltf_data* data, const cgltf_accessor* accessor, const GltfSource& doc,
        GltfAccessorRef& ref, const char*& error)
{
    ref = GltfAccessorRef();
    if (accessor->is_sparse)
    {
        error = "sparse accessors are not supported";
        return false;
    }
    if (!accessor->buffer_view || !accessor->buffer_view->buffer)
    {
        error = "accessor without buffer view";
        return false;
    }
    if (accessor->buffer_view->has_meshopt_compression)
    {
        error = "compressed buffer views are not supported";
        return false;
    }
    const int components = componentCount(accessor->type);
    if (components == 0)
    {
        error = "unsupported accessor type";
        return false;
    }
    const GltfComponentType componentType = convertComponentType(accessor->component_type);
    if (componentType == GltfComponentType::Invalid)
    {
        error = "unsupported component type";
        return false;
    }
    if (accessor->count > 0xFFFFFFFFull)
    {
        error = "accessor count is too large";
        return false;
    }
    const std::uint32_t elementSize =
            static_cast<std::uint32_t>(components) * GltfImporter::componentSize(componentType);
    const cgltf_buffer_view* view = accessor->buffer_view;

    ref.buffer = static_cast<int>(view->buffer - data->buffers);
    ref.offset = static_cast<std::uint64_t>(view->offset) + accessor->offset;
    ref.stride = view->stride ? static_cast<std::uint32_t>(view->stride) : elementSize;
    ref.count = static_cast<std::uint32_t>(accessor->count);
    ref.components = static_cast<std::uint8_t>(components);
    ref.componentType = componentType;
    ref.normalized = accessor->normalized != 0;

    if (ref.stride < elementSize)
    {
        error = "accessor stride is smaller than its element";
        return false;
    }
    if (ref.count > 0)
    {
        const std::uint64_t length = ref.byteLength();
        const std::uint64_t bufferSize = doc.bufferSizes[static_cast<std::size_t>(ref.buffer)];
        if (length > bufferSize || ref.offset > bufferSize - length)
        {
            error = "accessor exceeds its buffer";
            return false;
        }
    }
    return true;
}

bool convertPrimitive(const cgltf_data* data, const cgltf_primitive* source, const GltfSource& doc,
        ModelPrimitiveData& primitive, GltfPrimitiveSource& accessors, const char*& error)
{
    if (source->has_draco_mesh_compression)
    {
        error = "compressed primitives are not supported";
        return false;
    }
    switch (source->type)
    {
        case cgltf_primitive_type_points:
            primitive.topology = prisma::Topology::Points;
            break;
        case cgltf_primitive_type_lines:
            primitive.topology = prisma::Topology::Lines;
            break;
        case cgltf_primitive_type_line_strip:
            primitive.topology = prisma::Topology::LineStrip;
            break;
        case cgltf_primitive_type_triangles:
            primitive.topology = prisma::Topology::Triangles;
            break;
        case cgltf_primitive_type_triangle_strip:
            primitive.topology = prisma::Topology::TriangleStrip;
            break;
        default:
            error = "unsupported primitive topology";
            return false;
    }
    primitive.material =
            source->material ? static_cast<int>(source->material - data->materials) : -1;

    bool hasPosition = false;
    for (cgltf_size i = 0; i < source->attributes_count; ++i)
    {
        const cgltf_attribute& attribute = source->attributes[i];
        GltfAccessorRef* target = nullptr;
        int expected = 0;
        bool colorAttribute = false;
        if (attribute.type == cgltf_attribute_type_position && attribute.index == 0)
        {
            target = &accessors.positions;
            expected = 3;
            hasPosition = true;
        }
        else if (attribute.type == cgltf_attribute_type_normal && attribute.index == 0)
        {
            target = &accessors.normals;
            expected = 3;
        }
        else if (attribute.type == cgltf_attribute_type_tangent && attribute.index == 0)
        {
            target = &accessors.tangents;
            expected = 4;
        }
        else if (attribute.type == cgltf_attribute_type_color && attribute.index == 0)
        {
            target = &accessors.colors;
            colorAttribute = true;
        }
        else if (attribute.type == cgltf_attribute_type_texcoord && attribute.index == 0)
        {
            target = &accessors.uvs;
            expected = 2;
        }
        else if (attribute.type == cgltf_attribute_type_texcoord && attribute.index == 1)
        {
            target = &accessors.uvs2;
            expected = 2;
        }
        else if (attribute.type == cgltf_attribute_type_joints && attribute.index == 0)
        {
            target = &accessors.joints;
            expected = 4;
        }
        else if (attribute.type == cgltf_attribute_type_weights && attribute.index == 0)
        {
            target = &accessors.weights;
            expected = 4;
        }
        if (!target) continue;
        if (!attribute.data)
        {
            error = "attribute without accessor";
            return false;
        }
        if (!convertAccessor(data, attribute.data, doc, *target, error)) return false;
        if (colorAttribute ? (target->components != 3 && target->components != 4)
                           : target->components != expected)
        {
            error = "unexpected attribute type";
            return false;
        }
        if (attribute.type == cgltf_attribute_type_joints)
        {
            if (target->componentType != GltfComponentType::UInt8 &&
                    target->componentType != GltfComponentType::UInt16)
            {
                error = "unsupported joint index type";
                return false;
            }
        }
        if (attribute.type == cgltf_attribute_type_weights)
        {
            if (target->componentType != GltfComponentType::Float &&
                    target->componentType != GltfComponentType::UInt8 &&
                    target->componentType != GltfComponentType::UInt16)
            {
                error = "unsupported joint weight type";
                return false;
            }
        }
        if (attribute.type == cgltf_attribute_type_position)
        {
            if (attribute.data->has_min && attribute.data->has_max)
            {
                primitive.aabb = Math::Box(Math::Vec3(attribute.data->min[0],
                                                   attribute.data->min[1], attribute.data->min[2]),
                        Math::Vec3(attribute.data->max[0], attribute.data->max[1],
                                attribute.data->max[2]));
                primitive.hasAabb = true;
            }
        }
    }
    if (!hasPosition || !accessors.positions.valid())
    {
        error = "primitive without POSITION";
        return false;
    }
    if (!accessors.joints.valid() || !accessors.weights.valid())
    {
        accessors.joints = GltfAccessorRef();
        accessors.weights = GltfAccessorRef();
    }
    if (source->indices)
    {
        if (!convertAccessor(data, source->indices, doc, accessors.indices, error)) return false;
        const GltfComponentType type = accessors.indices.componentType;
        if (accessors.indices.components != 1 ||
                (type != GltfComponentType::UInt8 && type != GltfComponentType::UInt16 &&
                        type != GltfComponentType::UInt32))
        {
            error = "unsupported index accessor";
            return false;
        }
    }
    return true;
}

void decomposeMatrix(const cgltf_float* m, ModelNodeData& node)
{
    const Math::Vec3 x(m[0], m[1], m[2]);
    const Math::Vec3 y(m[4], m[5], m[6]);
    const Math::Vec3 z(m[8], m[9], m[10]);
    const Math::Vec3 scale(x.Length(), y.Length(), z.Length());
    const float epsilon = 1e-8f;

    Math::Mat3 basis;
    basis.col0 = scale.x * scale.x > epsilon ? x / scale.x : Math::Vec3::UnitX;
    basis.col1 = scale.y * scale.y > epsilon ? y / scale.y : Math::Vec3::UnitY;
    basis.col2 = scale.z * scale.z > epsilon ? z / scale.z : Math::Vec3::UnitZ;

    node.translation = Math::Vec3(m[12], m[13], m[14]);
    node.rotation = Math::Quaternion::FromMat3(basis).Normalized();
    node.scale = scale;
}

class ImportReader
{
public:
    ImportReader(const GltfSource& source, const std::uint8_t* memory, std::size_t memorySize)
            : mSource(source),
              mReader(source),
              mMemory(memory),
              mMemorySize(memorySize)
    {
    }

    bool floats(const GltfAccessorRef& accessor, std::uint32_t components, ct::Vector<float>& out)
    {
        ct::Vector<std::uint8_t> bytes;
        if (!readBytes(accessor, bytes)) return false;
        out.resize(static_cast<std::size_t>(accessor.count) * components);
        return GltfImporter::unpackFloats(accessor, bytes.data(), bytes.size(), out.data(),
                components);
    }

private:
    bool readBytes(const GltfAccessorRef& accessor, ct::Vector<std::uint8_t>& out)
    {
        if (!mMemory || !mSource.binary || accessor.buffer != 0) return mReader.read(accessor, out);
        const std::uint64_t length = accessor.byteLength();
        const std::uint64_t begin = mSource.binOffset + accessor.offset;
        if (begin > mMemorySize || length > mMemorySize - begin) return false;
        out.resize(static_cast<std::size_t>(length));
        if (length > 0) memcpy(out.data(), mMemory + begin, static_cast<std::size_t>(length));
        return true;
    }

    const GltfSource& mSource;
    GltfRangeReader mReader;
    const std::uint8_t* mMemory;
    std::size_t mMemorySize;
};

ct::String indexedName(const char* prefix, std::size_t index)
{
    char text[32];
    snprintf(text, sizeof(text), "%s%zu", prefix, index);
    return ct::String(text);
}

bool convertSkins(const cgltf_data* data, const GltfSource& src, ImportReader& reader,
        ModelData& out, ct::Vector<ct::Vector<int>>& jointBones, const char*& error)
{
    jointBones.resize(data->skins_count);
    for (cgltf_size s = 0; s < data->skins_count; ++s)
    {
        const cgltf_skin& skin = data->skins[s];
        if (skin.joints_count == 0)
        {
            error = "skin without joints";
            return false;
        }
        ct::Vector<int>& boneOf = jointBones[s];
        boneOf.resize(data->nodes_count, -1);
        for (cgltf_size j = 0; j < skin.joints_count; ++j)
        {
            const std::size_t node = static_cast<std::size_t>(skin.joints[j] - data->nodes);
            if (boneOf[node] >= 0)
            {
                error = "skin lists a joint twice";
                return false;
            }
            boneOf[node] = static_cast<int>(j);
        }

        ct::Vector<float> inverseBinds;
        if (skin.inverse_bind_matrices)
        {
            GltfAccessorRef ref;
            if (!convertAccessor(data, skin.inverse_bind_matrices, src, ref, error)) return false;
            if (ref.components != 16 || ref.componentType != GltfComponentType::Float ||
                    ref.count < skin.joints_count)
            {
                error = "unexpected inverse bind matrices";
                return false;
            }
            if (!reader.floats(ref, 16, inverseBinds))
            {
                error = "cannot read inverse bind matrices";
                return false;
            }
        }

        Skeleton skeleton;
        for (cgltf_size j = 0; j < skin.joints_count; ++j)
        {
            const std::size_t node = static_cast<std::size_t>(skin.joints[j] - data->nodes);
            const ModelNodeData& nodeData = out.nodes[node];

            ct::String name = nodeData.name.empty() ? indexedName("joint_", j) : nodeData.name;
            if (skeleton.findBone(name.c_str()) >= 0) name = name + indexedName("_", j);

            int parent = -1;
            if (nodeData.parent >= 0) parent = boneOf[static_cast<std::size_t>(nodeData.parent)];

            Math::Mat4 inverseBind = Math::Mat4::Identity();
            if (!inverseBinds.empty())
                memcpy(static_cast<void*>(&inverseBind), &inverseBinds[j * 16],
                        sizeof(inverseBind));

            if (!skeleton.addBone(name, parent, out.nodeLocalTransform(static_cast<int>(node)),
                        inverseBind))
            {
                error = "invalid skin joint";
                return false;
            }
        }
        if (!skeleton.finalize())
        {
            error = "skin joints do not form a hierarchy";
            return false;
        }
        out.skeletons.push_back(skeleton);
    }

    for (std::size_t i = 0; i < out.nodes.size(); ++i)
    {
        const cgltf_node& source = data->nodes[i];
        if (source.skin && source.mesh)
            out.nodes[i].skeleton = static_cast<int>(source.skin - data->skins);
    }
    for (std::size_t i = 0; i < out.nodes.size(); ++i)
    {
        const int mesh = out.nodes[i].mesh;
        if (out.nodes[i].skeleton < 0 || mesh < 0) continue;
        ModelMeshData& meshData = out.meshes[static_cast<std::size_t>(mesh)];
        for (std::size_t p = 0; p < meshData.primitives.size(); ++p)
        {
            ModelPrimitiveData& primitive = meshData.primitives[p];
            if ((primitive.attributeMask & (kVertexJoints | kVertexWeights)) ==
                    (kVertexJoints | kVertexWeights))
                primitive.skinned = true;
        }
    }
    return true;
}

struct RawChannels
{
    ct::Vector<float> positionTimes;
    ct::Vector<Math::Vec3> positions;
    ct::Vector<float> rotationTimes;
    ct::Vector<Math::Quaternion> rotations;
    ct::Vector<float> scaleTimes;
    ct::Vector<Math::Vec3> scales;
    bool hasPosition = false;
    bool hasRotation = false;
    bool hasScale = false;
};

Math::Vec3 sampleVec3(const ct::Vector<float>& times, const ct::Vector<Math::Vec3>& values,
        float time, const Math::Vec3& fallback)
{
    if (times.empty()) return fallback;
    if (times.size() == 1 || time <= times[0]) return values[0];
    if (time >= times[times.size() - 1]) return values[values.size() - 1];
    std::size_t next = 1;
    while (times[next] <= time) ++next;
    const float span = times[next] - times[next - 1];
    const float amount = span > 1e-6f ? (time - times[next - 1]) / span : 0.0f;
    return Math::Vec3::Lerp(values[next - 1], values[next], amount);
}

Math::Quaternion sampleQuaternion(const ct::Vector<float>& times,
        const ct::Vector<Math::Quaternion>& values, float time, const Math::Quaternion& fallback)
{
    if (times.empty()) return fallback;
    if (times.size() == 1 || time <= times[0]) return values[0];
    if (time >= times[times.size() - 1]) return values[values.size() - 1];
    std::size_t next = 1;
    while (times[next] <= time) ++next;
    const float span = times[next] - times[next - 1];
    const float amount = span > 1e-6f ? (time - times[next - 1]) / span : 0.0f;
    return Math::Quaternion::Lerp(values[next - 1], values[next], amount);
}

bool readChannel(const cgltf_data* data, const GltfSource& src, ImportReader& reader,
        const cgltf_animation_channel& channel, RawChannels& raw, const char*& error)
{
    const cgltf_animation_sampler& sampler = *channel.sampler;
    GltfAccessorRef inputRef;
    GltfAccessorRef outputRef;
    if (!convertAccessor(data, sampler.input, src, inputRef, error)) return false;
    if (!convertAccessor(data, sampler.output, src, outputRef, error)) return false;

    const bool rotation = channel.target_path == cgltf_animation_path_type_rotation;
    const std::uint32_t outputComponents = rotation ? 4u : 3u;
    if (inputRef.components != 1 || outputRef.components != outputComponents)
    {
        error = "unexpected animation sampler type";
        return false;
    }
    const std::uint32_t stepsPerKey =
            sampler.interpolation == cgltf_interpolation_type_cubic_spline ? 3u : 1u;
    const std::uint32_t valueOffset = stepsPerKey == 3u ? 1u : 0u;
    std::uint32_t keys = inputRef.count;
    if (outputRef.count / stepsPerKey < keys) keys = outputRef.count / stepsPerKey;
    if (keys == 0) return true;

    ct::Vector<float> times;
    ct::Vector<float> values;
    inputRef.count = keys;
    if (!reader.floats(inputRef, 1, times))
    {
        error = "cannot read animation data";
        return false;
    }
    outputRef.count = keys * stepsPerKey;
    if (!reader.floats(outputRef, outputComponents, values))
    {
        error = "cannot read animation data";
        return false;
    }
    for (std::uint32_t k = 1; k < keys; ++k)
    {
        if (times[k] < times[k - 1])
        {
            error = "animation times are not sorted";
            return false;
        }
    }

    for (std::uint32_t k = 0; k < keys; ++k)
    {
        const float* value = &values[(static_cast<std::size_t>(k) * stepsPerKey + valueOffset) *
                                     outputComponents];
        if (channel.target_path == cgltf_animation_path_type_translation)
        {
            raw.positionTimes.push_back(times[k]);
            raw.positions.push_back(Math::Vec3(value[0], value[1], value[2]));
        }
        else if (channel.target_path == cgltf_animation_path_type_scale)
        {
            raw.scaleTimes.push_back(times[k]);
            raw.scales.push_back(Math::Vec3(value[0], value[1], value[2]));
        }
        else
        {
            Math::Quaternion q(value[0], value[1], value[2], value[3]);
            raw.rotationTimes.push_back(times[k]);
            raw.rotations.push_back(
                    q.LengthSquared() > 1e-12f ? q.Normalized() : Math::Quaternion());
        }
    }
    return true;
}

bool convertAnimations(const cgltf_data* data, const GltfSource& src, ImportReader& reader,
        const ct::Vector<ct::Vector<int>>& jointBones, ModelData& out, const char*& error)
{
    for (cgltf_size a = 0; a < data->animations_count; ++a)
    {
        const cgltf_animation& animation = data->animations[a];

        ct::Vector<int> votes;
        votes.resize(data->skins_count, 0);
        for (cgltf_size c = 0; c < animation.channels_count; ++c)
        {
            const cgltf_animation_channel& channel = animation.channels[c];
            if (!channel.sampler || !channel.target_node) continue;
            const std::size_t node = static_cast<std::size_t>(channel.target_node - data->nodes);
            for (std::size_t s = 0; s < votes.size(); ++s)
            {
                if (jointBones[s][node] >= 0) ++votes[s];
            }
        }
        int skin = -1;
        for (std::size_t s = 0; s < votes.size(); ++s)
        {
            if (votes[s] > 0 && (skin < 0 || votes[s] > votes[static_cast<std::size_t>(skin)]))
                skin = static_cast<int>(s);
        }
        if (skin < 0) continue;

        const Skeleton& skeleton = out.skeletons[static_cast<std::size_t>(skin)];
        const ct::Vector<int>& boneOf = jointBones[static_cast<std::size_t>(skin)];
        ct::Vector<RawChannels> raw;
        raw.resize(skeleton.boneCount());
        for (cgltf_size c = 0; c < animation.channels_count; ++c)
        {
            const cgltf_animation_channel& channel = animation.channels[c];
            if (!channel.sampler || !channel.target_node) continue;
            const int bone = boneOf[static_cast<std::size_t>(channel.target_node - data->nodes)];
            if (bone < 0) continue;
            RawChannels& target = raw[static_cast<std::size_t>(bone)];
            switch (channel.target_path)
            {
                case cgltf_animation_path_type_translation:
                    if (target.hasPosition) continue;
                    target.hasPosition = true;
                    break;
                case cgltf_animation_path_type_rotation:
                    if (target.hasRotation) continue;
                    target.hasRotation = true;
                    break;
                case cgltf_animation_path_type_scale:
                    if (target.hasScale) continue;
                    target.hasScale = true;
                    break;
                default:
                    continue;
            }
            if (!readChannel(data, src, reader, channel, target, error)) return false;
        }

        bool anyKey = false;
        float start = 0.0f;
        for (std::size_t b = 0; b < raw.size(); ++b)
        {
            const ct::Vector<float>* lists[3] = { &raw[b].positionTimes, &raw[b].rotationTimes,
                &raw[b].scaleTimes };
            for (int l = 0; l < 3; ++l)
            {
                if (lists[l]->empty()) continue;
                if (!anyKey || (*lists[l])[0] < start) start = (*lists[l])[0];
                anyKey = true;
            }
        }
        if (!anyKey) continue;

        ct::Vector<LocalPose> bindPose;
        skeleton.bindPose(bindPose);

        AnimationClip clip;
        if (animation.name && animation.name[0]) clip.setName(ct::String(animation.name));
        else
            clip.setName(indexedName("animation_", a));
        clip.setSkeleton(skin);
        float duration = 0.0f;
        for (std::size_t b = 0; b < raw.size(); ++b)
        {
            RawChannels& channels = raw[b];
            ct::Vector<float> unified;
            for (std::size_t k = 0; k < channels.positionTimes.size(); ++k)
                unified.push_back(channels.positionTimes[k]);
            for (std::size_t k = 0; k < channels.rotationTimes.size(); ++k)
                unified.push_back(channels.rotationTimes[k]);
            for (std::size_t k = 0; k < channels.scaleTimes.size(); ++k)
                unified.push_back(channels.scaleTimes[k]);
            if (unified.empty()) continue;
            ct::sort(unified.begin(), unified.end());

            ct::Vector<float> times;
            for (std::size_t k = 0; k < unified.size(); ++k)
            {
                if (times.empty() || unified[k] - times[times.size() - 1] >= 1e-6f)
                    times.push_back(unified[k]);
            }

            BoneTrack track;
            track.bone = static_cast<int>(b);
            for (std::size_t k = 0; k < times.size(); ++k)
            {
                track.positions.push_back(sampleVec3(channels.positionTimes, channels.positions,
                        times[k], bindPose[b].position));
                track.rotations.push_back(sampleQuaternion(channels.rotationTimes,
                        channels.rotations, times[k], bindPose[b].rotation));
                track.scales.push_back(sampleVec3(channels.scaleTimes, channels.scales, times[k],
                        bindPose[b].scale));
                track.times.push_back(times[k] - start);
            }
            const float last = track.times[track.times.size() - 1];
            if (last > duration) duration = last;
            clip.tracks().push_back(track);
        }
        clip.setDuration(duration);
        out.animations.push_back(clip);
    }
    return true;
}

bool convertDocument(const cgltf_data* data, bool binary, const std::uint8_t* memory,
        std::size_t memorySize, ModelData& out, GltfSource& src)
{
    const char* error = "";

    for (cgltf_size i = 0; i < data->buffers_count; ++i)
    {
        const cgltf_buffer& buffer = data->buffers[i];
        if (!buffer.uri)
        {
            if (!binary || i != 0) return fail(out, "buffer without uri");
            if (buffer.size > src.binSize) return fail(out, "buffer is larger than the BIN chunk");
            src.bufferPaths.push_back(out.path);
        }
        else
        {
            if (isDataUri(buffer.uri)) return fail(out, "data URIs are not supported");
            if (binary && i == 0) return fail(out, "GLB buffer 0 must not have a uri");
            src.bufferPaths.push_back(out.directory + decodeUri(buffer.uri));
        }
        src.bufferSizes.push_back(buffer.size);
    }

    for (cgltf_size i = 0; i < data->images_count; ++i)
    {
        const cgltf_image& source = data->images[i];
        ModelImageData image;
        if (source.name) image.name = source.name;
        if (source.mime_type) image.mimeType = source.mime_type;
        GltfImageSource range;
        if (source.buffer_view && source.buffer_view->buffer)
        {
            range.buffer = static_cast<int>(source.buffer_view->buffer - data->buffers);
            range.offset = source.buffer_view->offset;
            image.size = source.buffer_view->size;
            const std::uint64_t bufferSize =
                    src.bufferSizes[static_cast<std::size_t>(range.buffer)];
            if (image.size > bufferSize || range.offset > bufferSize - image.size)
                return fail(out, "image exceeds its buffer");
            out.imageBytes += image.size;
        }
        else if (source.uri)
        {
            if (isDataUri(source.uri)) return fail(out, "data URIs are not supported");
            image.path = out.directory + decodeUri(source.uri);
        }
        else
            return fail(out, "image without source");
        image.source = static_cast<std::uint32_t>(src.images.size());
        src.images.push_back(range);
        out.images.push_back(image);
    }

    for (cgltf_size i = 0; i < data->textures_count; ++i)
    {
        const cgltf_texture& source = data->textures[i];
        ModelTextureData texture;
        texture.image = source.image ? static_cast<int>(source.image - data->images) : -1;
        if (source.sampler)
        {
            texture.repeatU = source.sampler->wrap_s == 0 || source.sampler->wrap_s == kWrapRepeat;
            texture.repeatV = source.sampler->wrap_t == 0 || source.sampler->wrap_t == kWrapRepeat;
            texture.nearest = source.sampler->mag_filter == kFilterNearest;
        }
        out.textures.push_back(texture);
    }

    ct::Vector<int> lightmapTextures;
    for (cgltf_size i = 0; i < data->materials_count; ++i)
    {
        const cgltf_material& source = data->materials[i];
        ModelMaterialData material;
        if (source.name) material.name = source.name;
        if (source.has_pbr_metallic_roughness)
        {
            const cgltf_pbr_metallic_roughness& pbr = source.pbr_metallic_roughness;
            material.baseColor = Math::Vec4(pbr.base_color_factor[0], pbr.base_color_factor[1],
                    pbr.base_color_factor[2], pbr.base_color_factor[3]);
            material.metallic = pbr.metallic_factor;
            material.roughness = pbr.roughness_factor;
            if (pbr.base_color_texture.texture)
                material.baseColorTexture =
                        static_cast<int>(pbr.base_color_texture.texture - data->textures);
            if (pbr.metallic_roughness_texture.texture)
                material.metallicRoughnessTexture =
                        static_cast<int>(pbr.metallic_roughness_texture.texture - data->textures);
        }
        material.emissive = Math::Vec3(source.emissive_factor[0], source.emissive_factor[1],
                source.emissive_factor[2]);
        material.alphaCutoff = source.alpha_cutoff;
        switch (source.alpha_mode)
        {
            case cgltf_alpha_mode_mask:
                material.alphaMode = ModelAlphaMode::Mask;
                break;
            case cgltf_alpha_mode_blend:
                material.alphaMode = ModelAlphaMode::Blend;
                break;
            default:
                material.alphaMode = ModelAlphaMode::Opaque;
                break;
        }
        material.doubleSided = source.double_sided != 0;
        if (source.normal_texture.texture)
            material.normalTexture =
                    static_cast<int>(source.normal_texture.texture - data->textures);
        int lightmapTexture = -1;
        if (source.occlusion_texture.texture)
        {
            const int occlusion =
                    static_cast<int>(source.occlusion_texture.texture - data->textures);
            if (source.occlusion_texture.texcoord == 1) lightmapTexture = occlusion;
            else
                material.occlusionTexture = occlusion;
        }
        lightmapTextures.push_back(lightmapTexture);
        if (source.emissive_texture.texture)
            material.emissiveTexture =
                    static_cast<int>(source.emissive_texture.texture - data->textures);
        out.materials.push_back(material);
    }

    struct ExtraMesh
    {
        std::size_t owner;
        ModelMeshData data;
    };
    ct::Vector<ExtraMesh> pendingExtras;
    for (cgltf_size i = 0; i < data->meshes_count; ++i)
    {
        const cgltf_mesh& source = data->meshes[i];
        ModelMeshData mesh;
        if (source.name) mesh.name = source.name;
        ct::Vector<int> primitiveLightmaps;
        for (cgltf_size p = 0; p < source.primitives_count; ++p)
        {
            ModelPrimitiveData primitive;
            GltfPrimitiveSource accessors;
            if (!convertPrimitive(data, &source.primitives[p], src, primitive, accessors, error))
                return fail(out, error);
            primitive.attributeMask = kVertexPosition;
            if (accessors.normals.valid()) primitive.attributeMask |= kVertexNormal;
            // Without tangents in the file the loader makes them (MeshStore::generateTangents), as
            // Godot does: a normal map can come later from a material override.
            if (accessors.tangents.valid() ||
                    (accessors.uvs.valid() && primitive.topology == prisma::Topology::Triangles))
                primitive.attributeMask |= kVertexTangent;
            if (accessors.colors.valid()) primitive.attributeMask |= kVertexColor;
            if (accessors.uvs.valid()) primitive.attributeMask |= kVertexUv;
            if (accessors.uvs2.valid()) primitive.attributeMask |= kVertexUv2;
            int lightmap = -1;
            if (accessors.uvs2.valid() && primitive.material >= 0 &&
                    static_cast<std::size_t>(primitive.material) < lightmapTextures.size() &&
                    lightmapTextures[static_cast<std::size_t>(primitive.material)] >= 0)
            {
                const cgltf_texture& texture = data->textures[static_cast<std::size_t>(
                        lightmapTextures[static_cast<std::size_t>(primitive.material)])];
                if (texture.image) lightmap = static_cast<int>(texture.image - data->images);
            }
            if (lightmap >= 0 && mesh.lightmapImage < 0) mesh.lightmapImage = lightmap;
            primitiveLightmaps.push_back(lightmap);
            if (accessors.joints.valid()) primitive.attributeMask |= kVertexJoints | kVertexWeights;
            primitive.vertexCount = accessors.positions.count;
            primitive.indexCount = accessors.indices.count;
            // Flat normals give every whole triangle corners of its own (see flatNormals).
            if (!accessors.normals.valid() && primitive.topology == prisma::Topology::Triangles)
            {
                const auto corners =
                        primitive.indexCount > 0 ? primitive.indexCount : primitive.vertexCount;
                if (corners >= 3)
                {
                    primitive.attributeMask |= kVertexNormal;
                    primitive.vertexCount = corners - corners % 3;
                    if (primitive.indexCount > 0) primitive.indexCount = primitive.vertexCount;
                }
            }
            primitive.sourceBytes = accessors.byteLength();
            primitive.source = static_cast<std::uint32_t>(src.primitives.size());
            src.primitives.push_back(accessors);
            out.geometryBytes += primitive.sourceBytes;
            ++out.primitiveCount;
            mesh.primitives.push_back(primitive);
        }
        ModelMeshData kept;
        kept.name = mesh.name;
        kept.lightmapImage = mesh.lightmapImage;
        ct::Vector<ModelMeshData> extras;
        for (std::size_t p = 0; p < mesh.primitives.size(); ++p)
        {
            const int lightmap = primitiveLightmaps[p];
            if (lightmap < 0 || lightmap == mesh.lightmapImage)
            {
                kept.primitives.push_back(mesh.primitives[p]);
                continue;
            }
            std::size_t slot = extras.size();
            for (std::size_t e = 0; e < extras.size(); ++e)
                if (extras[e].lightmapImage == lightmap) slot = e;
            if (slot == extras.size())
            {
                ModelMeshData extra;
                extra.name = mesh.name;
                extra.name += "_lm";
                extra.name.append_number(static_cast<unsigned long long>(extras.size() + 1));
                extra.lightmapImage = lightmap;
                extras.push_back(extra);
            }
            extras[slot].primitives.push_back(mesh.primitives[p]);
        }
        out.meshes.push_back(kept);
        for (std::size_t e = 0; e < extras.size(); ++e)
        {
            ExtraMesh pending;
            pending.owner = static_cast<std::size_t>(i);
            pending.data = extras[e];
            pendingExtras.push_back(pending);
        }
    }
    ct::Vector<ct::Vector<int>> extraMeshes;
    extraMeshes.resize(static_cast<std::size_t>(data->meshes_count));
    for (std::size_t e = 0; e < pendingExtras.size(); ++e)
    {
        extraMeshes[pendingExtras[e].owner].push_back(static_cast<int>(out.meshes.size()));
        out.meshes.push_back(pendingExtras[e].data);
    }

    for (cgltf_size i = 0; i < data->cameras_count; ++i)
    {
        const cgltf_camera& source = data->cameras[i];
        ModelCameraData camera;
        if (source.type == cgltf_camera_type_perspective)
        {
            camera.perspective = true;
            camera.yfov = source.data.perspective.yfov;
            camera.aspect = source.data.perspective.has_aspect_ratio
                                    ? source.data.perspective.aspect_ratio
                                    : 0.0f;
            camera.zNear = source.data.perspective.znear;
            if (source.data.perspective.has_zfar && source.data.perspective.zfar > 0.0f)
                camera.zFar = source.data.perspective.zfar;
        }
        else if (source.type == cgltf_camera_type_orthographic)
        {
            camera.perspective = false;
            camera.xmag = source.data.orthographic.xmag;
            camera.ymag = source.data.orthographic.ymag;
            camera.zNear = source.data.orthographic.znear;
            camera.zFar = source.data.orthographic.zfar;
        }
        else
            return fail(out, "unsupported camera type");
        out.cameras.push_back(camera);
    }

    out.nodes.resize(data->nodes_count);
    for (cgltf_size i = 0; i < data->nodes_count; ++i)
    {
        const cgltf_node& source = data->nodes[i];
        ModelNodeData& node = out.nodes[i];
        if (source.name) node.name = source.name;
        if (source.has_matrix) decomposeMatrix(source.matrix, node);
        else
        {
            if (source.has_translation)
                node.translation = Math::Vec3(source.translation[0], source.translation[1],
                        source.translation[2]);
            if (source.has_rotation)
            {
                const Math::Quaternion rotation(source.rotation[0], source.rotation[1],
                        source.rotation[2], source.rotation[3]);
                if (rotation.LengthSquared() > 1e-12f) node.rotation = rotation.Normalized();
            }
            if (source.has_scale)
                node.scale = Math::Vec3(source.scale[0], source.scale[1], source.scale[2]);
        }
        node.mesh = source.mesh ? static_cast<int>(source.mesh - data->meshes) : -1;
        node.camera = source.camera ? static_cast<int>(source.camera - data->cameras) : -1;
        for (cgltf_size c = 0; c < source.children_count; ++c)
            node.children.push_back(static_cast<int>(source.children[c] - data->nodes));
    }
    for (std::size_t i = 0; i < out.nodes.size(); ++i)
    {
        const ModelNodeData& node = out.nodes[i];
        for (std::size_t c = 0; c < node.children.size(); ++c)
        {
            ModelNodeData& child = out.nodes[static_cast<std::size_t>(node.children[c])];
            if (child.parent >= 0) return fail(out, "node has several parents");
            child.parent = static_cast<int>(i);
        }
    }
    const std::size_t originalNodes = out.nodes.size();
    for (std::size_t i = 0; i < originalNodes; ++i)
    {
        const int mesh = out.nodes[i].mesh;
        if (mesh < 0 || static_cast<std::size_t>(mesh) >= extraMeshes.size()) continue;
        const ct::Vector<int> extras = extraMeshes[static_cast<std::size_t>(mesh)];
        for (std::size_t e = 0; e < extras.size(); ++e)
        {
            ModelNodeData child;
            child.name = out.nodes[i].name;
            child.name += "_lm";
            child.name.append_number(static_cast<unsigned long long>(e + 1));
            child.parent = static_cast<int>(i);
            child.mesh = extras[e];
            out.nodes.push_back(child);
            out.nodes[i].children.push_back(static_cast<int>(out.nodes.size() - 1));
        }
    }
    ct::Vector<int> stamp;
    stamp.resize(out.nodes.size(), -1);
    for (std::size_t i = 0; i < out.nodes.size(); ++i)
    {
        int current = static_cast<int>(i);
        while (current >= 0 && stamp[static_cast<std::size_t>(current)] == -1)
        {
            stamp[static_cast<std::size_t>(current)] = static_cast<int>(i);
            current = out.nodes[static_cast<std::size_t>(current)].parent;
        }
        if (current >= 0 && stamp[static_cast<std::size_t>(current)] == static_cast<int>(i))
            return fail(out, "node hierarchy has a cycle");
    }

    if (data->skins_count > 0)
    {
        ImportReader reader(src, memory, memorySize);
        ct::Vector<ct::Vector<int>> jointBones;
        if (!convertSkins(data, src, reader, out, jointBones, error)) return fail(out, error);
        if (!convertAnimations(data, src, reader, jointBones, out, error)) return fail(out, error);
    }

    const cgltf_scene* scene =
            data->scene ? data->scene : (data->scenes_count ? &data->scenes[0] : nullptr);
    if (scene)
    {
        for (cgltf_size i = 0; i < scene->nodes_count; ++i)
            out.roots.push_back(static_cast<int>(scene->nodes[i] - data->nodes));
    }
    else
    {
        for (std::size_t i = 0; i < out.nodes.size(); ++i)
        {
            if (out.nodes[i].parent < 0) out.roots.push_back(static_cast<int>(i));
        }
    }
    return true;
}

const char* describe(cgltf_result result)
{
    switch (result)
    {
        case cgltf_result_data_too_short:
            return "glTF data is too short";
        case cgltf_result_unknown_format:
            return "unknown glTF format";
        case cgltf_result_invalid_json:
            return "invalid glTF JSON";
        case cgltf_result_invalid_gltf:
            return "invalid glTF";
        case cgltf_result_legacy_gltf:
            return "legacy glTF is not supported";
        case cgltf_result_out_of_memory:
            return "out of memory";
        default:
            return "glTF parse failed";
    }
}

bool convertJson(const std::uint8_t* json, std::size_t jsonSize, const GlbLayout& layout,
        const std::uint8_t* memory, std::size_t memorySize, const ct::String& path, ModelData& out)
{
    out = ModelData();
    out.path = path;
    const std::size_t slash = path.rfind('/');
    if (slash != ct::String::npos) out.directory = ct::String(path.c_str(), slash + 1);
    ct::Unique<GltfSource> owned = ct::make_unique<GltfSource>();
    GltfSource& src = *owned;
    src.binary = layout.hasBin;
    src.binOffset = layout.binOffset;
    src.binSize = layout.binSize;

    cgltf_options options;
    memset(&options, 0, sizeof(options));
    options.type = cgltf_file_type_gltf;
    cgltf_data* data = nullptr;
    const cgltf_result result = cgltf_parse(&options, json, jsonSize, &data);
    if (result != cgltf_result_success) return fail(out, describe(result));
    const bool ok = convertDocument(data, layout.hasBin, memory, memorySize, out, src);
    cgltf_free(data);
    if (ok) out.source = ct::Unique<ModelSource>(static_cast<ct::Unique<GltfSource>&&>(owned));
    return ok;
}

void flatNormals(SurfaceData& surface)
{
    const std::size_t corners =
            surface.indices.empty() ? surface.positions.size() : surface.indices.size();
    if (corners < 3) return;
    if (surface.indices.empty())
    {
        // Unindexed triangles already have corners of their own: only the normals are added.
        const std::size_t count = corners - corners % 3;
        surface.positions.resize(count);
        if (surface.tangents.size() > count) surface.tangents.resize(count);
        if (surface.colors.size() > count) surface.colors.resize(count);
        if (surface.uvs.size() > count) surface.uvs.resize(count);
        if (surface.uvs2.size() > count) surface.uvs2.resize(count);
        if (surface.joints.size() > count) surface.joints.resize(count);
        if (surface.weights.size() > count) surface.weights.resize(count);
        surface.normals.resize(count);
        for (std::size_t t = 0; t < count; t += 3)
        {
            const Math::Vec3& a = surface.positions[t];
            const Math::Vec3 normal = (surface.positions[t + 1] - a)
                                              .Cross(surface.positions[t + 2] - a)
                                              .NormalizedSafe();
            for (std::size_t k = 0; k < 3; ++k) surface.normals[t + k] = normal;
        }
        return;
    }
    SurfaceData flat;
    flat.topology = surface.topology;
    const std::size_t count = corners - corners % 3;
    flat.positions.reserve(count);
    flat.normals.reserve(count);
    for (std::size_t t = 0; t < count; t += 3)
    {
        std::uint32_t corner[3];
        for (std::size_t k = 0; k < 3; ++k)
            corner[k] = surface.indices.empty() ? static_cast<std::uint32_t>(t + k)
                                                : surface.indices[t + k];
        if (corner[0] >= surface.positions.size() || corner[1] >= surface.positions.size() ||
                corner[2] >= surface.positions.size())
            continue;
        const Math::Vec3& a = surface.positions[corner[0]];
        const Math::Vec3 normal = (surface.positions[corner[1]] - a)
                                          .Cross(surface.positions[corner[2]] - a)
                                          .NormalizedSafe();
        for (std::size_t k = 0; k < 3; ++k)
        {
            const std::uint32_t v = corner[k];
            flat.indices.push_back(static_cast<std::uint32_t>(flat.positions.size()));
            flat.positions.push_back(surface.positions[v]);
            flat.normals.push_back(normal);
            if (v < surface.tangents.size()) flat.tangents.push_back(surface.tangents[v]);
            if (v < surface.colors.size()) flat.colors.push_back(surface.colors[v]);
            if (v < surface.uvs.size()) flat.uvs.push_back(surface.uvs[v]);
            if (v < surface.uvs2.size()) flat.uvs2.push_back(surface.uvs2[v]);
            if (v < surface.joints.size()) flat.joints.push_back(surface.joints[v]);
            if (v < surface.weights.size()) flat.weights.push_back(surface.weights[v]);
        }
    }
    surface = static_cast<SurfaceData&&>(flat);
}

float unpackComponent(const std::uint8_t* source, GltfComponentType type, bool normalized)
{
    switch (type)
    {
        case GltfComponentType::Float:
        {
            float value;
            memcpy(&value, source, sizeof(value));
            return value;
        }
        case GltfComponentType::Int8:
        {
            const float value = static_cast<float>(static_cast<std::int8_t>(*source));
            if (!normalized) return value;
            const float scaled = value / 127.0f;
            return scaled < -1.0f ? -1.0f : scaled;
        }
        case GltfComponentType::UInt8:
        {
            const float value = static_cast<float>(*source);
            return normalized ? value / 255.0f : value;
        }
        case GltfComponentType::Int16:
        {
            std::int16_t raw;
            memcpy(&raw, source, sizeof(raw));
            const float value = static_cast<float>(raw);
            if (!normalized) return value;
            const float scaled = value / 32767.0f;
            return scaled < -1.0f ? -1.0f : scaled;
        }
        case GltfComponentType::UInt16:
        {
            std::uint16_t raw;
            memcpy(&raw, source, sizeof(raw));
            const float value = static_cast<float>(raw);
            return normalized ? value / 65535.0f : value;
        }
        case GltfComponentType::UInt32:
        {
            std::uint32_t raw;
            memcpy(&raw, source, sizeof(raw));
            return static_cast<float>(raw);
        }
        default:
            return 0.0f;
    }
}

bool fitsInBytes(const GltfAccessorRef& accessor, std::size_t size, std::uint32_t elementSize)
{
    if (elementSize == 0 || accessor.stride < elementSize) return false;
    if (accessor.count == 0) return true;
    return accessor.byteLength() <= static_cast<std::uint64_t>(size);
}

bool readAttribute(const GltfAccessorRef& accessor, std::uint32_t outComponents,
        std::size_t vertexCount, GltfRangeReader& reader, float* out)
{
    if (accessor.count != vertexCount) return false;
    ct::Vector<std::uint8_t> bytes;
    if (!reader.read(accessor, bytes)) return false;
    return GltfImporter::unpackFloats(accessor, bytes.data(), bytes.size(), out, outComponents);
}

} // namespace

std::uint64_t GltfAccessorRef::byteLength() const
{
    if (count == 0) return 0;
    return static_cast<std::uint64_t>(count - 1) * stride +
           static_cast<std::uint64_t>(components) * GltfImporter::componentSize(componentType);
}

std::uint64_t GltfPrimitiveSource::byteLength() const
{
    return positions.byteLength() + normals.byteLength() + tangents.byteLength() +
           colors.byteLength() + uvs.byteLength() + uvs2.byteLength() + joints.byteLength() +
           weights.byteLength() + indices.byteLength();
}

GltfRangeReader::GltfRangeReader(const GltfSource& source)
        : mSource(source),
          mStreams(),
          mBytesRead(0)
{
    mStreams.resize(source.bufferPaths.size(), nullptr);
}

GltfRangeReader::~GltfRangeReader()
{
    for (std::size_t i = 0; i < mStreams.size(); ++i)
    {
        if (mStreams[i]) io_close(mStreams[i]);
    }
}

IoStream* GltfRangeReader::open(int buffer)
{
    if (buffer < 0 || static_cast<std::size_t>(buffer) >= mStreams.size()) return nullptr;
    IoStream*& stream = mStreams[static_cast<std::size_t>(buffer)];
    if (!stream)
        stream = FileSystem::getSingleton().open(
                mSource.bufferPaths[static_cast<std::size_t>(buffer)]);
    return stream;
}

bool GltfRangeReader::read(int buffer, std::uint64_t offset, std::uint64_t size, void* out)
{
    if (buffer < 0 || static_cast<std::size_t>(buffer) >= mSource.bufferSizes.size()) return false;
    const std::uint64_t bufferSize = mSource.bufferSizes[static_cast<std::size_t>(buffer)];
    if (size > bufferSize || offset > bufferSize - size) return false;
    if (size == 0) return true;
    if (!out) return false;
    IoStream* stream = open(buffer);
    if (!stream) return false;
    const std::uint64_t fileOffset =
            offset + (buffer == 0 && mSource.binary ? mSource.binOffset : 0);
    if (io_seek(stream, static_cast<std::int64_t>(fileOffset), IO_SEEK_SET) !=
            static_cast<std::int64_t>(fileOffset))
        return false;
    if (io_read(stream, out, static_cast<std::size_t>(size)) != static_cast<std::size_t>(size))
        return false;
    mBytesRead += size;
    return true;
}

bool GltfRangeReader::read(const GltfAccessorRef& accessor, ct::Vector<std::uint8_t>& out)
{
    const std::uint64_t length = accessor.byteLength();
    out.resize(static_cast<std::size_t>(length));
    return read(accessor.buffer, accessor.offset, length, out.data());
}

std::uint64_t GltfRangeReader::bytesRead() const
{
    return mBytesRead;
}

const char* GltfImporter::name() const
{
    return "glTF";
}

bool GltfImporter::supports(const char* extension) const
{
    return extension && (strcmp(extension, "gltf") == 0 || strcmp(extension, "glb") == 0);
}

const GltfSource* GltfImporter::source(const ModelData& model)
{
    return static_cast<const GltfSource*>(model.source.get());
}

bool GltfImporter::importMemory(const std::uint8_t* data, std::size_t size, const ct::String& path,
        ModelData& out)
{
    out = ModelData();
    if (!data || size == 0) return fail(out, "empty glTF data");
    GlbLayout layout;
    const char* error = "";
    if (!parseGlbLayout(data, size, size, layout, error)) return fail(out, error);
    if (!layout.glb) return convertJson(data, size, layout, nullptr, 0, path, out);

    const std::uint64_t binHeader = layout.jsonOffset + layout.jsonSize;
    if (size >= binHeader + kChunkHeaderSize &&
            !parseBinChunk(data + binHeader, size, layout, error))
        return fail(out, error);
    return convertJson(data + layout.jsonOffset, static_cast<std::size_t>(layout.jsonSize), layout,
            data, size, path, out);
}

bool GltfImporter::import(const ct::String& path, ModelData& out)
{
    out = ModelData();
    IoStream* stream = FileSystem::getSingleton().open(path);
    if (!stream) return fail(out, "cannot open file");

    const std::int64_t total = io_size(stream);
    if (total < 0)
    {
        io_close(stream);
        return fail(out, "cannot get file size");
    }

    std::uint8_t head[20];
    memset(head, 0, sizeof(head));
    const std::size_t headSize = io_read(stream, head, sizeof(head));

    GlbLayout layout;
    const char* error = "";
    if (!parseGlbLayout(head, headSize, static_cast<std::uint64_t>(total), layout, error))
    {
        io_close(stream);
        return fail(out, error);
    }

    ct::Vector<std::uint8_t> json;
    if (layout.glb)
    {
        json.resize(static_cast<std::size_t>(layout.jsonSize));
        bool ok = io_seek(stream, static_cast<std::int64_t>(layout.jsonOffset), IO_SEEK_SET) ==
                          static_cast<std::int64_t>(layout.jsonOffset) &&
                  io_read(stream, json.data(), json.size()) == json.size();
        if (ok)
        {
            const std::uint64_t binHeader = layout.jsonOffset + layout.jsonSize;
            if (static_cast<std::uint64_t>(total) >= binHeader + kChunkHeaderSize)
            {
                std::uint8_t header[8];
                ok = io_seek(stream, static_cast<std::int64_t>(binHeader), IO_SEEK_SET) ==
                             static_cast<std::int64_t>(binHeader) &&
                     io_read(stream, header, sizeof(header)) == sizeof(header);
                if (ok && !parseBinChunk(header, static_cast<std::uint64_t>(total), layout, error))
                {
                    io_close(stream);
                    return fail(out, error);
                }
            }
        }
        if (!ok)
        {
            io_close(stream);
            return fail(out, "cannot read GLB");
        }
    }
    else
    {
        json.resize(static_cast<std::size_t>(total));
        const bool ok = io_seek(stream, 0, IO_SEEK_SET) == 0 &&
                        io_read(stream, json.data(), json.size()) == json.size();
        if (!ok)
        {
            io_close(stream);
            return fail(out, "cannot read file");
        }
    }
    io_close(stream);
    if (json.empty()) return fail(out, "empty glTF data");
    return convertJson(json.data(), json.size(), layout, nullptr, 0, path, out);
}

bool GltfImporter::readPrimitive(const ModelData& model, const ModelPrimitiveData& description,
        SurfaceData& out) const
{
    const GltfSource* document = source(model);
    if (!document || description.source >= document->primitives.size()) return false;
    const GltfPrimitiveSource& primitive = document->primitives[description.source];
    GltfRangeReader reader(*document);
    out.topology = description.topology;
    out.positions.clear();
    out.normals.clear();
    out.tangents.clear();
    out.colors.clear();
    out.uvs.clear();
    out.uvs2.clear();
    out.joints.clear();
    out.weights.clear();
    out.indices.clear();

    if (!primitive.positions.valid()) return false;
    const std::size_t vertexCount = primitive.positions.count;

    out.positions.resize(vertexCount);
    if (!readAttribute(primitive.positions, 3, vertexCount, reader,
                reinterpret_cast<float*>(out.positions.data())))
        return false;

    if (primitive.normals.valid())
    {
        out.normals.resize(vertexCount);
        if (!readAttribute(primitive.normals, 3, vertexCount, reader,
                    reinterpret_cast<float*>(out.normals.data())))
            return false;
    }
    if (primitive.tangents.valid())
    {
        out.tangents.resize(vertexCount);
        if (!readAttribute(primitive.tangents, 4, vertexCount, reader,
                    reinterpret_cast<float*>(out.tangents.data())))
            return false;
    }
    if (primitive.colors.valid())
    {
        out.colors.resize(vertexCount);
        if (!readAttribute(primitive.colors, 4, vertexCount, reader,
                    reinterpret_cast<float*>(out.colors.data())))
            return false;
        if (primitive.colors.components == 3)
        {
            for (std::size_t i = 0; i < vertexCount; ++i) out.colors[i].w = 1.0f;
        }
    }
    if (primitive.uvs.valid())
    {
        out.uvs.resize(vertexCount);
        if (!readAttribute(primitive.uvs, 2, vertexCount, reader,
                    reinterpret_cast<float*>(out.uvs.data())))
            return false;
    }
    if (primitive.uvs2.valid())
    {
        out.uvs2.resize(vertexCount);
        if (!readAttribute(primitive.uvs2, 2, vertexCount, reader,
                    reinterpret_cast<float*>(out.uvs2.data())))
            return false;
    }
    if (primitive.joints.valid() && primitive.weights.valid())
    {
        ct::Vector<float> joints;
        joints.resize(vertexCount * 4);
        if (!readAttribute(primitive.joints, 4, vertexCount, reader, joints.data())) return false;
        out.joints.resize(vertexCount);
        for (std::size_t i = 0; i < vertexCount; ++i)
        {
            std::uint32_t packed = 0;
            for (std::size_t c = 0; c < 4; ++c)
            {
                const float index = joints[i * 4 + c];
                if (index < 0.0f || index > 255.0f) return false;
                packed |= static_cast<std::uint32_t>(index) << (8 * c);
            }
            out.joints[i] = packed;
        }

        out.weights.resize(vertexCount);
        if (!readAttribute(primitive.weights, 4, vertexCount, reader,
                    reinterpret_cast<float*>(out.weights.data())))
            return false;
        for (std::size_t i = 0; i < vertexCount; ++i)
        {
            Math::Vec4& weight = out.weights[i];
            const float sum = weight.x + weight.y + weight.z + weight.w;
            if (sum > 0.0f)
                weight = Math::Vec4(weight.x / sum, weight.y / sum, weight.z / sum, weight.w / sum);
        }
    }
    if (primitive.indices.valid())
    {
        ct::Vector<std::uint8_t> bytes;
        if (!reader.read(primitive.indices, bytes)) return false;
        if (!unpackIndices(primitive.indices, bytes.data(), bytes.size(), out.indices))
            return false;
    }
    if (out.normals.empty() && out.topology == prisma::Topology::Triangles) flatNormals(out);
    return true;
}

bool GltfImporter::readImage(const ModelData& model, const ModelImageData& image,
        ct::Vector<std::uint8_t>& out) const
{
    out.clear();
    const GltfSource* document = source(model);
    if (image.embedded())
    {
        if (!document || image.source >= document->images.size()) return false;
        const GltfImageSource& range = document->images[image.source];
        GltfRangeReader reader(*document);
        out.resize(static_cast<std::size_t>(image.size));
        return reader.read(range.buffer, range.offset, image.size, out.data());
    }
    if (image.path.empty()) return false;
    IoStream* stream = FileSystem::getSingleton().open(image.path);
    if (!stream) return false;
    const std::int64_t size = io_size(stream);
    bool ok = size >= 0;
    if (ok)
    {
        out.resize(static_cast<std::size_t>(size));
        ok = io_read(stream, out.data(), out.size()) == out.size();
    }
    io_close(stream);
    if (!ok) out.clear();
    return ok;
}

std::uint32_t GltfImporter::componentSize(GltfComponentType type)
{
    switch (type)
    {
        case GltfComponentType::Int8:
        case GltfComponentType::UInt8:
            return 1;
        case GltfComponentType::Int16:
        case GltfComponentType::UInt16:
            return 2;
        case GltfComponentType::UInt32:
        case GltfComponentType::Float:
            return 4;
        default:
            return 0;
    }
}

bool GltfImporter::unpackFloats(const GltfAccessorRef& accessor, const std::uint8_t* bytes,
        std::size_t size, float* out, std::uint32_t outComponents)
{
    const std::uint32_t componentBytes = componentSize(accessor.componentType);
    const std::uint32_t elementSize = accessor.components * componentBytes;
    if (!fitsInBytes(accessor, size, elementSize)) return false;
    if (accessor.count > 0 && (!bytes || !out)) return false;

    const std::uint32_t copied =
            accessor.components < outComponents ? accessor.components : outComponents;
    for (std::uint32_t i = 0; i < accessor.count; ++i)
    {
        const std::uint8_t* element = bytes + static_cast<std::size_t>(i) * accessor.stride;
        float* target = out + static_cast<std::size_t>(i) * outComponents;
        for (std::uint32_t c = 0; c < copied; ++c)
            target[c] = unpackComponent(element + c * componentBytes, accessor.componentType,
                    accessor.normalized);
        for (std::uint32_t c = copied; c < outComponents; ++c) target[c] = 0.0f;
    }
    return true;
}

bool GltfImporter::unpackIndices(const GltfAccessorRef& accessor, const std::uint8_t* bytes,
        std::size_t size, ct::Vector<std::uint32_t>& out)
{
    if (accessor.componentType != GltfComponentType::UInt8 &&
            accessor.componentType != GltfComponentType::UInt16 &&
            accessor.componentType != GltfComponentType::UInt32)
        return false;
    if (accessor.components != 1) return false;
    const std::uint32_t componentBytes = componentSize(accessor.componentType);
    if (!fitsInBytes(accessor, size, componentBytes)) return false;
    if (accessor.count > 0 && !bytes) return false;

    out.resize(accessor.count);
    for (std::uint32_t i = 0; i < accessor.count; ++i)
    {
        const std::uint8_t* source = bytes + static_cast<std::size_t>(i) * accessor.stride;
        std::uint32_t value;
        if (accessor.componentType == GltfComponentType::UInt8) value = *source;
        else if (accessor.componentType == GltfComponentType::UInt16)
        {
            std::uint16_t raw;
            memcpy(&raw, source, sizeof(raw));
            value = raw;
        }
        else
            memcpy(&value, source, sizeof(value));
        out[i] = value;
    }
    return true;
}

} // namespace prisma3d
