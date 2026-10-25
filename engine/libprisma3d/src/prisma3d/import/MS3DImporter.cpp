#include "prisma3d/import/MS3DImporter.h"

#include "prisma3d/core/FileSystem.h"

#include <math.h>
#include <string.h>

namespace prisma3d
{

namespace
{

const std::size_t kHeaderSize = 14;
const std::size_t kVertexSize = 15;
const std::size_t kTriangleSize = 70;
const std::size_t kMaterialSize = 361;
const std::size_t kJointSize = 93;
const std::size_t kKeySize = 16;
const std::size_t kNameSize = 32;
const std::size_t kPathSize = 128;
const float kKeyTolerance = 1e-5f;

bool finite(float value)
{
    return value - value == 0.0f;
}

struct Cursor
{
    const std::uint8_t* data = nullptr;
    std::size_t size = 0;
    std::size_t pos = 0;
    const char* error = nullptr;

    bool fail(const char* message)
    {
        if (!error) error = message;
        return false;
    }

    bool canRead(std::size_t count) const
    {
        return !error && pos <= size && size - pos >= count;
    }

    bool ensure(std::size_t count)
    {
        if (error) return false;
        if (pos > size || size - pos < count) return fail("data is truncated");
        return true;
    }

    bool take(void* out, std::size_t count)
    {
        if (!canRead(count))
        {
            pos = size;
            return fail("data is truncated");
        }
        memcpy(out, data + pos, count);
        pos += count;
        return true;
    }

    std::uint8_t readU8()
    {
        std::uint8_t value = 0;
        take(&value, 1);
        return value;
    }

    std::int8_t readS8()
    {
        return static_cast<std::int8_t>(readU8());
    }

    std::uint16_t readU16()
    {
        std::uint8_t raw[2] = { 0, 0 };
        take(raw, 2);
        return static_cast<std::uint16_t>(raw[0] | (raw[1] << 8));
    }

    std::uint32_t readU32()
    {
        std::uint8_t raw[4] = { 0, 0, 0, 0 };
        take(raw, 4);
        return static_cast<std::uint32_t>(raw[0]) | (static_cast<std::uint32_t>(raw[1]) << 8) |
               (static_cast<std::uint32_t>(raw[2]) << 16) |
               (static_cast<std::uint32_t>(raw[3]) << 24);
    }

    std::int32_t readS32()
    {
        return static_cast<std::int32_t>(readU32());
    }

    float readF32()
    {
        const std::uint32_t raw = readU32();
        float value;
        memcpy(&value, &raw, sizeof(value));
        return value;
    }

    ct::String readFixed(std::size_t length)
    {
        if (!ensure(length)) return ct::String();
        const char* begin = reinterpret_cast<const char*>(data + pos);
        std::size_t used = 0;
        while (used < length && begin[used] != 0) ++used;
        pos += length;
        return ct::String(begin, used);
    }

    void skip(std::size_t count)
    {
        if (ensure(count)) pos += count;
    }
};

struct Vertex
{
    Math::Vec3 position = Math::Vec3(0.0f, 0.0f, 0.0f);
    int bone = -1;
};

struct Triangle
{
    std::uint16_t index[3] = { 0, 0, 0 };
    Math::Vec3 normal[3];
    Math::Vec2 uv[3];
};

struct Group
{
    ct::Vector<std::uint16_t> triangles;
    int material = -1;
};

struct Material
{
    ct::String name;
    Math::Vec3 diffuse = Math::Vec3(1.0f, 1.0f, 1.0f);
    float shininess = 32.0f;
    ct::String texture;
};

struct Key
{
    float time = 0.0f;
    Math::Vec3 value = Math::Vec3(0.0f, 0.0f, 0.0f);
};

struct Joint
{
    ct::String name;
    ct::String parentName;
    Math::Vec3 rotation = Math::Vec3(0.0f, 0.0f, 0.0f);
    Math::Vec3 translation = Math::Vec3(0.0f, 0.0f, 0.0f);
    ct::Vector<Key> rotationKeys;
    ct::Vector<Key> translationKeys;
};

struct File
{
    ct::Vector<Vertex> vertices;
    ct::Vector<Triangle> triangles;
    ct::Vector<Group> groups;
    ct::Vector<Material> materials;
    ct::Vector<Joint> joints;
};

bool readVec3(Cursor& r, Math::Vec3& value)
{
    value.x = r.readF32();
    value.y = r.readF32();
    value.z = r.readF32();
    if (r.error) return false;
    if (!finite(value.x) || !finite(value.y) || !finite(value.z))
        return r.fail("vector is not finite");
    return true;
}

bool readKeys(Cursor& r, std::size_t count, ct::Vector<Key>& keys)
{
    if (!r.ensure(count * kKeySize)) return false;
    keys.reserve(count);
    for (std::size_t i = 0; i < count; ++i)
    {
        Key key;
        key.time = r.readF32();
        if (!finite(key.time)) return r.fail("key time is not finite");
        if (!readVec3(r, key.value)) return false;
        if (!keys.empty() && key.time < keys.back().time)
            return r.fail("keys are not in time order");
        keys.push_back(key);
    }
    return !r.error;
}

bool parseFile(Cursor& r, File& file)
{
    if (!r.ensure(kHeaderSize)) return false;
    if (memcmp(r.data, "MS3D000000", 10) != 0) return r.fail("not a MilkShape file");
    r.pos = 10;
    const std::int32_t version = r.readS32();
    if (version < 3 || version > 4) return r.fail("unsupported version");

    if (!r.ensure(2)) return false;
    const std::size_t vertexCount = r.readU16();
    if (!r.ensure(vertexCount * kVertexSize)) return false;
    file.vertices.reserve(vertexCount);
    for (std::size_t i = 0; i < vertexCount; ++i)
    {
        Vertex vertex;
        r.readU8();
        if (!readVec3(r, vertex.position)) return false;
        vertex.bone = r.readS8();
        r.readU8();
        file.vertices.push_back(vertex);
    }

    if (!r.ensure(2)) return false;
    const std::size_t triangleCount = r.readU16();
    if (!r.ensure(triangleCount * kTriangleSize)) return false;
    file.triangles.reserve(triangleCount);
    for (std::size_t i = 0; i < triangleCount; ++i)
    {
        Triangle triangle;
        r.readU16();
        for (int k = 0; k < 3; ++k)
        {
            triangle.index[k] = r.readU16();
            if (triangle.index[k] >= vertexCount) return r.fail("triangle index is out of range");
        }
        for (int k = 0; k < 3; ++k)
        {
            if (!readVec3(r, triangle.normal[k])) return false;
        }
        float s[3];
        float t[3];
        for (int k = 0; k < 3; ++k) s[k] = r.readF32();
        for (int k = 0; k < 3; ++k) t[k] = r.readF32();
        for (int k = 0; k < 3; ++k) triangle.uv[k] = Math::Vec2(s[k], 1.0f - t[k]);
        r.readU8();
        r.readU8();
        file.triangles.push_back(triangle);
    }

    if (!r.ensure(2)) return false;
    const std::size_t groupCount = r.readU16();
    file.groups.reserve(groupCount);
    for (std::size_t i = 0; i < groupCount; ++i)
    {
        if (!r.ensure(1 + kNameSize + 2)) return false;
        r.readU8();
        r.skip(kNameSize);
        const std::size_t count = r.readU16();
        if (!r.ensure(count * 2 + 1)) return false;
        Group group;
        group.triangles.reserve(count);
        for (std::size_t k = 0; k < count; ++k)
        {
            const std::uint16_t index = r.readU16();
            if (index >= triangleCount) return r.fail("group triangle is out of range");
            group.triangles.push_back(index);
        }
        group.material = r.readS8();
        file.groups.push_back(static_cast<Group&&>(group));
    }

    if (!r.ensure(2)) return false;
    const std::size_t materialCount = r.readU16();
    if (!r.ensure(materialCount * kMaterialSize)) return false;
    file.materials.reserve(materialCount);
    for (std::size_t i = 0; i < materialCount; ++i)
    {
        Material material;
        material.name = r.readFixed(kNameSize);
        r.skip(16);
        material.diffuse.x = r.readF32();
        material.diffuse.y = r.readF32();
        material.diffuse.z = r.readF32();
        r.skip(4);
        r.skip(16);
        r.skip(16);
        material.shininess = r.readF32();
        r.skip(4);
        r.skip(1);
        material.texture = r.readFixed(kPathSize);
        r.skip(kPathSize);
        if (r.error) return false;
        if (!finite(material.diffuse.x) || !finite(material.diffuse.y) ||
                !finite(material.diffuse.z) || !finite(material.shininess))
            return r.fail("material is not finite");
        file.materials.push_back(static_cast<Material&&>(material));
    }

    if (!r.ensure(12)) return false;
    r.skip(12);

    const std::size_t remaining = r.size - r.pos;
    if (remaining == 0) return true;
    if (remaining < 2) return r.fail("data is truncated");
    const std::size_t jointCount = r.readU16();
    if (!r.ensure(jointCount * kJointSize)) return false;
    file.joints.reserve(jointCount);
    for (std::size_t i = 0; i < jointCount; ++i)
    {
        if (!r.ensure(kJointSize)) return false;
        Joint joint;
        r.readU8();
        joint.name = r.readFixed(kNameSize);
        joint.parentName = r.readFixed(kNameSize);
        if (!readVec3(r, joint.rotation) || !readVec3(r, joint.translation)) return false;
        const std::size_t rotationCount = r.readU16();
        const std::size_t translationCount = r.readU16();
        if (!readKeys(r, rotationCount, joint.rotationKeys)) return false;
        if (!readKeys(r, translationCount, joint.translationKeys)) return false;
        file.joints.push_back(static_cast<Joint&&>(joint));
    }

    for (std::size_t i = 0; i < file.vertices.size(); ++i)
    {
        Vertex& vertex = file.vertices[i];
        if (file.joints.empty()) continue;
        if (vertex.bone < 0) vertex.bone = 0;
        if (static_cast<std::size_t>(vertex.bone) >= file.joints.size())
            return r.fail("vertex bone is out of range");
    }
    return !r.error;
}

ct::String directoryOf(const ct::String& path)
{
    std::size_t slash = ct::String::npos;
    for (std::size_t i = 0; i < path.size(); ++i)
    {
        if (path[i] == '/' || path[i] == '\\') slash = i;
    }
    if (slash == ct::String::npos) return ct::String();
    return ct::String(path.c_str(), slash + 1);
}

ct::String baseNameOf(const ct::String& path)
{
    std::size_t begin = 0;
    for (std::size_t i = 0; i < path.size(); ++i)
    {
        if (path[i] == '/' || path[i] == '\\') begin = i + 1;
    }
    std::size_t end = path.size();
    for (std::size_t i = path.size(); i > begin; --i)
    {
        if (path[i - 1] == '.')
        {
            end = i - 1;
            break;
        }
    }
    return ct::String(path.c_str() + begin, end - begin);
}

ct::String textureBasename(const ct::String& reference)
{
    std::size_t begin = 0;
    for (std::size_t i = 0; i < reference.size(); ++i)
    {
        if (reference[i] == '/' || reference[i] == '\\') begin = i + 1;
    }
    return ct::String(reference.c_str() + begin, reference.size() - begin);
}

ct::String joinPath(const ct::String& directory, const ct::String& name)
{
    if (name.empty() || directory.empty()) return name;
    if (name[0] == '/' || name[0] == '\\' || (name.size() > 1 && name[1] == ':')) return name;
    return directory + name;
}

float shininessToRoughness(float shininess)
{
    if (shininess <= 0.0f) return 1.0f;
    const float roughness = sqrtf(2.0f / (shininess + 2.0f));
    return roughness < 0.05f ? 0.05f : (roughness > 1.0f ? 1.0f : roughness);
}

std::uint64_t surfaceBytes(const SurfaceData& surface)
{
    std::uint64_t total = 0;
    total += surface.positions.size() * sizeof(Math::Vec3);
    total += surface.normals.size() * sizeof(Math::Vec3);
    total += surface.tangents.size() * sizeof(Math::Vec4);
    total += surface.colors.size() * sizeof(Math::Vec4);
    total += surface.uvs.size() * sizeof(Math::Vec2);
    total += surface.uvs2.size() * sizeof(Math::Vec2);
    total += surface.joints.size() * sizeof(std::uint32_t);
    total += surface.weights.size() * sizeof(Math::Vec4);
    total += surface.indices.size() * sizeof(std::uint32_t);
    return total;
}

Math::Quaternion eulerToQuat(const Math::Vec3& e)
{
    const Math::Quaternion qx = Math::Quaternion::FromAxisAngle(Math::Vec3(1.0f, 0.0f, 0.0f), e.x);
    const Math::Quaternion qy = Math::Quaternion::FromAxisAngle(Math::Vec3(0.0f, 1.0f, 0.0f), e.y);
    const Math::Quaternion qz = Math::Quaternion::FromAxisAngle(Math::Vec3(0.0f, 0.0f, 1.0f), e.z);
    return qz * qy * qx;
}

Math::Mat4 composeLocal(const Math::Vec3& translation, const Math::Quaternion& rotation)
{
    LocalPose pose;
    pose.position = translation;
    pose.rotation = rotation;
    return pose.toMatrix();
}

bool computeGlobals(const ct::Vector<Math::Mat4>& locals, const ct::Vector<int>& parents,
        ct::Vector<Math::Mat4>& globals)
{
    const std::size_t count = locals.size();
    globals.resize(count);
    ct::Vector<std::uint8_t> state;
    state.resize(count, 0);
    ct::Vector<std::size_t> chain;
    for (std::size_t i = 0; i < count; ++i)
    {
        if (state[i] == 2) continue;
        chain.clear();
        int current = static_cast<int>(i);
        while (current >= 0 && state[static_cast<std::size_t>(current)] != 2)
        {
            const std::size_t index = static_cast<std::size_t>(current);
            if (state[index] == 1) return false;
            state[index] = 1;
            chain.push_back(index);
            current = parents[index];
        }
        for (std::size_t k = chain.size(); k > 0; --k)
        {
            const std::size_t index = chain[k - 1];
            const int parent = parents[index];
            globals[index] = parent >= 0 ? globals[static_cast<std::size_t>(parent)] * locals[index]
                                         : locals[index];
            state[index] = 2;
        }
    }
    return true;
}

ct::String boneName(const Skeleton& skeleton, const ct::String& name, std::size_t index)
{
    if (!name.empty() && skeleton.findBone(name.c_str()) < 0) return name;
    ct::String candidate = name.empty() ? ct::String("joint") : name;
    candidate += "_";
    candidate.append_number(index);
    while (skeleton.findBone(candidate.c_str()) >= 0) candidate += "_";
    return candidate;
}

bool buildSkeleton(const File& file, Skeleton& skeleton, ct::Vector<int>& parents,
        ct::Vector<Math::Mat4>& locals)
{
    const std::size_t count = file.joints.size();
    parents.resize(count, -1);
    locals.resize(count);
    for (std::size_t i = 0; i < count; ++i)
    {
        const Joint& joint = file.joints[i];
        if (!joint.parentName.empty())
        {
            for (std::size_t k = 0; k < count; ++k)
            {
                if (file.joints[k].name == joint.parentName)
                {
                    parents[i] = static_cast<int>(k);
                    break;
                }
            }
        }
        locals[i] = composeLocal(joint.translation, eulerToQuat(joint.rotation));
    }

    ct::Vector<Math::Mat4> globals;
    if (!computeGlobals(locals, parents, globals)) return false;

    for (std::size_t i = 0; i < count; ++i)
    {
        const float determinant = globals[i].Determinant();
        Math::Mat4 inverse = Math::Mat4::Identity();
        if (finite(determinant) && fabsf(determinant) > 1e-20f) inverse = globals[i].Inverse();
        if (!skeleton.addBone(boneName(skeleton, file.joints[i].name, i), parents[i], locals[i],
                    inverse))
            return false;
    }
    return skeleton.finalize();
}

std::size_t upperBound(const ct::Vector<Key>& keys, float time)
{
    std::size_t low = 0;
    std::size_t high = keys.size();
    while (low < high)
    {
        const std::size_t middle = low + (high - low) / 2;
        if (!(time < keys[middle].time)) low = middle + 1;
        else
            high = middle;
    }
    return low;
}

Math::Quaternion sampleRotation(const ct::Vector<Key>& keys, float time)
{
    if (keys.size() == 1 || time <= keys.front().time) return eulerToQuat(keys.front().value);
    if (time >= keys.back().time) return eulerToQuat(keys.back().value);
    const std::size_t next = upperBound(keys, time);
    const std::size_t previous = next - 1;
    const float span = keys[next].time - keys[previous].time;
    const float amount = span > 1e-6f ? (time - keys[previous].time) / span : 0.0f;
    return Math::Quaternion::Slerp(eulerToQuat(keys[previous].value), eulerToQuat(keys[next].value),
            amount);
}

Math::Vec3 sampleTranslation(const ct::Vector<Key>& keys, float time)
{
    if (keys.size() == 1 || time <= keys.front().time) return keys.front().value;
    if (time >= keys.back().time) return keys.back().value;
    const std::size_t next = upperBound(keys, time);
    const std::size_t previous = next - 1;
    const float span = keys[next].time - keys[previous].time;
    const float amount = span > 1e-6f ? (time - keys[previous].time) / span : 0.0f;
    return keys[previous].value + (keys[next].value - keys[previous].value) * amount;
}

void sortTimes(ct::Vector<float>& times)
{
    for (std::size_t i = 1; i < times.size(); ++i)
    {
        const float value = times[i];
        std::size_t k = i;
        while (k > 0 && times[k - 1] > value)
        {
            times[k] = times[k - 1];
            --k;
        }
        times[k] = value;
    }
}

bool buildTrack(int boneIndex, const Joint& joint, const Math::Mat4& bindLocal, BoneTrack& track)
{
    if (joint.rotationKeys.empty() && joint.translationKeys.empty()) return false;

    ct::Vector<float> all;
    for (std::size_t i = 0; i < joint.rotationKeys.size(); ++i)
        all.push_back(joint.rotationKeys[i].time);
    for (std::size_t i = 0; i < joint.translationKeys.size(); ++i)
        all.push_back(joint.translationKeys[i].time);
    sortTimes(all);

    track.bone = boneIndex;
    float last = 0.0f;
    for (std::size_t i = 0; i < all.size(); ++i)
    {
        const float time = all[i];
        if (i > 0 && fabsf(time - last) < kKeyTolerance) continue;
        last = time;
        const Math::Quaternion deltaRotation = joint.rotationKeys.empty()
                                                       ? Math::Quaternion(0.0f, 0.0f, 0.0f, 1.0f)
                                                       : sampleRotation(joint.rotationKeys, time);
        const Math::Vec3 deltaTranslation =
                joint.translationKeys.empty() ? Math::Vec3(0.0f, 0.0f, 0.0f)
                                              : sampleTranslation(joint.translationKeys, time);
        const Math::Mat4 delta = composeLocal(deltaTranslation, deltaRotation);
        LocalPose pose;
        if (!LocalPose::fromMatrix(bindLocal * delta, pose)) pose = LocalPose();
        track.times.push_back(time);
        track.positions.push_back(pose.position);
        track.rotations.push_back(pose.rotation);
        track.scales.push_back(pose.scale);
    }
    return !track.times.empty();
}

struct MS3DSource : ModelSource
{
    ct::Vector<SurfaceData> surfaces;
};

bool fail(ModelData& out, const char* message)
{
    out.error = message;
    return false;
}

bool build(const std::uint8_t* data, std::size_t size, const ct::String& path, ModelData& out)
{
    out = ModelData();
    out.path = path;
    out.directory = directoryOf(path);

    Cursor r;
    r.data = data;
    r.size = size;
    File file;
    if (!parseFile(r, file)) return fail(out, r.error ? r.error : "invalid file");

    const bool skinned = !file.joints.empty();
    const std::size_t materialCount = file.materials.empty() ? 1 : file.materials.size();
    for (std::size_t g = 0; g < file.groups.size(); ++g)
    {
        const int material = file.groups[g].material;
        if (material >= 0 && static_cast<std::size_t>(material) >= materialCount)
            return fail(out, "group material is out of range");
    }

    const ct::String baseName = baseNameOf(path);
    ct::Unique<MS3DSource> owned = ct::make_unique<MS3DSource>();

    ModelMeshData mesh;
    mesh.name = baseName;
    for (std::size_t g = 0; g < file.groups.size(); ++g)
    {
        const Group& group = file.groups[g];
        if (group.triangles.empty()) continue;

        SurfaceData surface;
        surface.topology = prisma::Topology::Triangles;
        for (std::size_t t = 0; t < group.triangles.size(); ++t)
        {
            const Triangle& triangle = file.triangles[group.triangles[t]];
            for (int corner = 0; corner < 3; ++corner)
            {
                const Vertex& vertex = file.vertices[triangle.index[corner]];
                surface.positions.push_back(vertex.position);
                surface.normals.push_back(triangle.normal[corner]);
                surface.uvs.push_back(triangle.uv[corner]);
                if (skinned)
                {
                    const std::uint32_t bone = static_cast<std::uint32_t>(vertex.bone);
                    surface.joints.push_back(bone | (bone << 8) | (bone << 16) | (bone << 24));
                    surface.weights.push_back(Math::Vec4(1.0f, 0.0f, 0.0f, 0.0f));
                }
                surface.indices.push_back(static_cast<std::uint32_t>(surface.positions.size() - 1));
            }
        }

        ModelPrimitiveData primitive;
        primitive.material = group.material >= 0 ? group.material : 0;
        primitive.attributeMask = kVertexPosition | kVertexNormal | kVertexUv;
        if (skinned) primitive.attributeMask |= kVertexJoints | kVertexWeights;
        primitive.vertexCount = static_cast<std::uint32_t>(surface.positions.size());
        primitive.indexCount = static_cast<std::uint32_t>(surface.indices.size());
        primitive.sourceBytes = surfaceBytes(surface);
        primitive.source = static_cast<std::uint32_t>(owned->surfaces.size());
        primitive.aabb = MeshStore::computeAabb(surface.positions);
        primitive.hasAabb = true;
        primitive.skinned = skinned;

        out.geometryBytes += primitive.sourceBytes;
        ++out.primitiveCount;
        mesh.primitives.push_back(primitive);
        owned->surfaces.push_back(static_cast<SurfaceData&&>(surface));
    }
    if (mesh.primitives.empty()) return fail(out, "file has no triangles");
    out.meshes.push_back(static_cast<ModelMeshData&&>(mesh));

    if (file.materials.empty())
    {
        ModelMaterialData material;
        material.name = "default";
        material.metallic = 0.0f;
        out.materials.push_back(static_cast<ModelMaterialData&&>(material));
    }
    for (std::size_t i = 0; i < file.materials.size(); ++i)
    {
        const Material& source = file.materials[i];
        ModelMaterialData material;
        material.name = source.name;
        material.baseColor = Math::Vec4(source.diffuse.x, source.diffuse.y, source.diffuse.z, 1.0f);
        material.metallic = 0.0f;
        material.roughness = shininessToRoughness(source.shininess);
        const ct::String textureFile = textureBasename(source.texture);
        if (!textureFile.empty())
        {
            ModelImageData image;
            image.name = textureFile;
            image.path = joinPath(out.directory, textureFile);
            out.images.push_back(static_cast<ModelImageData&&>(image));
            ModelTextureData texture;
            texture.image = static_cast<int>(out.images.size() - 1);
            out.textures.push_back(texture);
            material.baseColorTexture = static_cast<int>(out.textures.size() - 1);
        }
        out.materials.push_back(static_cast<ModelMaterialData&&>(material));
    }

    ModelNodeData node;
    node.name = baseName;
    node.mesh = 0;
    node.skeleton = skinned ? 0 : -1;
    out.nodes.push_back(static_cast<ModelNodeData&&>(node));
    out.roots.push_back(0);

    if (skinned)
    {
        Skeleton skeleton;
        ct::Vector<int> parents;
        ct::Vector<Math::Mat4> locals;
        if (!buildSkeleton(file, skeleton, parents, locals))
            return fail(out, "skeleton is invalid");

        AnimationClip clip;
        clip.setName(baseName);
        clip.setSkeleton(0);
        float duration = 0.0f;
        for (std::size_t i = 0; i < file.joints.size(); ++i)
        {
            BoneTrack track;
            if (!buildTrack(static_cast<int>(i), file.joints[i], locals[i], track)) continue;
            if (track.times.back() > duration) duration = track.times.back();
            clip.tracks().push_back(static_cast<BoneTrack&&>(track));
        }
        clip.setDuration(duration);
        out.skeletons.push_back(static_cast<Skeleton&&>(skeleton));
        if (!clip.tracks().empty()) out.animations.push_back(static_cast<AnimationClip&&>(clip));
    }

    out.source = ct::Unique<ModelSource>(static_cast<ct::Unique<MS3DSource>&&>(owned));
    return true;
}

} // namespace

const char* MS3DImporter::name() const
{
    return "MilkShape 3D";
}

bool MS3DImporter::supports(const char* extension) const
{
    return extension && strcmp(extension, "ms3d") == 0;
}

bool MS3DImporter::import(const ct::String& path, ModelData& out)
{
    out = ModelData();
    const ct::Vector<std::uint8_t> bytes = FileSystem::getSingleton().readBinary(path);
    if (bytes.empty()) return fail(out, "cannot read file");
    return build(bytes.data(), bytes.size(), path, out);
}

bool MS3DImporter::importMemory(const std::uint8_t* data, std::size_t size,
        const ct::String& virtualName, ModelData& out)
{
    out = ModelData();
    if (!data || size == 0) return fail(out, "empty model data");
    return build(data, size, virtualName, out);
}

bool MS3DImporter::readPrimitive(const ModelData& model, const ModelPrimitiveData& primitive,
        SurfaceData& out) const
{
    const MS3DSource* source = static_cast<const MS3DSource*>(model.source.get());
    if (!source || primitive.source >= source->surfaces.size()) return false;
    out = source->surfaces[primitive.source];
    return true;
}

bool MS3DImporter::readImage(const ModelData&, const ModelImageData&,
        ct::Vector<std::uint8_t>& out) const
{
    out.clear();
    return false;
}

} // namespace prisma3d
