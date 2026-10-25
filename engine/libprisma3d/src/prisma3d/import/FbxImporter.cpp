#include "prisma3d/import/FbxImporter.h"

#include "prisma3d/core/FileSystem.h"

#include "ofbx.h"

#include <ct/hashmap.hpp>
#include <ct/sort.hpp>

#include <math.h>
#include <stdio.h>
#include <string.h>

namespace prisma3d
{

namespace
{

const int kMaxBones = 256;
const int kMaxLayers = 16;
const float kTimeEpsilon = 1e-6f;

typedef ct::Vector<float> Floats;

struct FbxSource : ModelSource
{
    ct::Vector<SurfaceData> surfaces;
    ct::Vector<ct::Vector<std::uint8_t>> embedded;
};

struct BoneMap
{
    ct::Vector<const ofbx::Object*> bones;
    ct::HashMap<std::uint64_t, int> indexById;

    int indexOf(const ofbx::Object* object) const
    {
        if (!object) return -1;
        const int* found = indexById.find(object->id);
        return found ? *found : -1;
    }
};

struct WeightSlots
{
    int ids[4] = { -1, -1, -1, -1 };
    float weights[4] = { 0.0f, 0.0f, 0.0f, 0.0f };

    void add(int bone, float weight)
    {
        if (bone < 0 || weight <= 0.0f) return;
        for (int slot = 0; slot < 4; ++slot)
        {
            if (weight <= weights[slot]) continue;
            for (int k = 3; k > slot; --k)
            {
                ids[k] = ids[k - 1];
                weights[k] = weights[k - 1];
            }
            ids[slot] = bone;
            weights[slot] = weight;
            return;
        }
    }
};

struct Channels
{
    Floats times[3][3];
    Floats values[3][3];
};

struct Context
{
    ofbx::IScene* scene = nullptr;
    ct::String path;
    ct::String directory;
    ct::String baseName;
    BoneMap bones;
    bool hasSkin = false;
    ct::HashMap<std::uint64_t, int> materialSlot;
    ct::HashMap<std::uint64_t, int> textureSlot;
    FbxSource* source = nullptr;
};

std::size_t lastSlash(const ct::String& path)
{
    std::size_t slash = ct::String::npos;
    for (std::size_t i = 0; i < path.size(); ++i)
        if (path[i] == '/' || path[i] == '\\') slash = i;
    return slash;
}

ct::String directoryOf(const ct::String& path)
{
    const std::size_t slash = lastSlash(path);
    return slash == ct::String::npos ? ct::String() : ct::String(path.c_str(), slash + 1);
}

ct::String fileNameOf(const ct::String& path)
{
    const std::size_t slash = lastSlash(path);
    return slash == ct::String::npos ? path : ct::String(path.c_str() + slash + 1);
}

ct::String baseNameOf(const ct::String& path)
{
    const ct::String file = fileNameOf(path);
    std::size_t dot = ct::String::npos;
    for (std::size_t i = 0; i < file.size(); ++i)
        if (file[i] == '.') dot = i;
    return dot == ct::String::npos ? file : ct::String(file.c_str(), dot);
}

bool isAbsolute(const ct::String& name)
{
    if (name.empty()) return false;
    if (name[0] == '/' || name[0] == '\\') return true;
    return name.size() > 1 && name[1] == ':';
}

ct::String joinPath(const ct::String& directory, const ct::String& name)
{
    if (name.empty() || directory.empty() || isAbsolute(name)) return name;
    return directory + name;
}

ct::String resolveTextureFile(const ct::String& directory, const ct::String& embeddedPath)
{
    if (embeddedPath.empty()) return ct::String();
    const FileSystem& files = FileSystem::getSingleton();
    ct::String portable = embeddedPath;
    for (std::size_t i = 0; i < portable.size(); ++i)
        if (portable[i] == '\\') portable[i] = '/';
    const ct::String joined = FileSystem::simplifyPath(joinPath(directory, portable));
    if (files.exists(joined)) return joined;

    static const char* const kFolders[] = { "", "textures/", "Textures/", "images/", "Images/",
        "materials/", "Materials/", "maps/", "Maps/", "tex/", "Tex/" };
    const ct::String name = fileNameOf(embeddedPath);
    ct::String folder = directory;
    while (!folder.empty())
    {
        for (const char* sub: kFolders)
        {
            const ct::String candidate = folder + sub + name;
            if (files.exists(candidate)) return candidate;
        }
        const ct::String parent = directoryOf(ct::String(folder.c_str(), folder.size() - 1));
        if (parent == folder) break;
        folder = parent;
    }
    return joined;
}

ct::String viewString(const ofbx::DataView& view)
{
    if (!view.begin || view.end <= view.begin) return ct::String();
    return ct::String(reinterpret_cast<const char*>(view.begin),
            static_cast<std::size_t>(view.end - view.begin));
}

ct::String objectName(const ofbx::Object& object)
{
    if (object.name[0] != '\0')
    {
        const char* separator = strstr(object.name, "::");
        return ct::String(separator ? separator + 2 : object.name);
    }
    char buffer[48];
    snprintf(buffer, sizeof(buffer), "object_%llu", static_cast<unsigned long long>(object.id));
    return ct::String(buffer);
}

float toFloat(double value)
{
    return static_cast<float>(value);
}

Math::Mat4 toMat4(const ofbx::Matrix& m)
{
    Math::Mat4 out;
    out.col0 = Math::Vec4(toFloat(m.m[0]), toFloat(m.m[1]), toFloat(m.m[2]), toFloat(m.m[3]));
    out.col1 = Math::Vec4(toFloat(m.m[4]), toFloat(m.m[5]), toFloat(m.m[6]), toFloat(m.m[7]));
    out.col2 = Math::Vec4(toFloat(m.m[8]), toFloat(m.m[9]), toFloat(m.m[10]), toFloat(m.m[11]));
    out.col3 = Math::Vec4(toFloat(m.m[12]), toFloat(m.m[13]), toFloat(m.m[14]), toFloat(m.m[15]));
    return out;
}

Math::Vec3 toVec3(const ofbx::Vec3& v)
{
    return Math::Vec3(toFloat(v.x), toFloat(v.y), toFloat(v.z));
}

Math::Vec2 toVec2(const ofbx::Vec2& v)
{
    return Math::Vec2(toFloat(v.x), toFloat(v.y));
}

bool finite(const Math::Vec3& v)
{
    return isfinite(v.x) && isfinite(v.y) && isfinite(v.z);
}

bool finite(const Math::Mat4& m)
{
    const Math::Vec4* columns[4] = { &m.col0, &m.col1, &m.col2, &m.col3 };
    for (int c = 0; c < 4; ++c)
        if (!isfinite(columns[c]->x) || !isfinite(columns[c]->y) || !isfinite(columns[c]->z) ||
                !isfinite(columns[c]->w))
            return false;
    return true;
}

Math::Vec3 xyz(const Math::Vec4& v)
{
    return Math::Vec3(v.x, v.y, v.z);
}

float sampleScalarChannel(const Floats& times, const Floats& values, float time, float fallback)
{
    const std::size_t count = times.size() < values.size() ? times.size() : values.size();
    if (count == 0) return fallback;
    if (count == 1 || time <= times[0]) return values[0];
    if (time >= times[count - 1]) return values[count - 1];
    std::size_t next = 1;
    while (next < count && times[next] <= time) ++next;
    const std::size_t previous = next - 1;
    const float span = times[next] - times[previous];
    const float amount = span > kTimeEpsilon ? (time - times[previous]) / span : 0.0f;
    return values[previous] + (values[next] - values[previous]) * amount;
}

void appendTimes(Floats& destination, const Floats& source)
{
    for (std::size_t i = 0; i < source.size(); ++i) destination.push_back(source[i]);
}

void sortUniqueTimes(Floats& times)
{
    if (times.empty()) return;
    ct::sort(times.begin(), times.end());
    std::size_t kept = 1;
    for (std::size_t i = 1; i < times.size(); ++i)
        if (fabsf(times[i] - times[kept - 1]) >= kTimeEpsilon) times[kept++] = times[i];
    times.resize(kept);
}

bool isSkeletonNode(const ofbx::Object* object)
{
    if (!object) return false;
    return object->getType() == ofbx::Object::Type::LIMB_NODE ||
           object->getType() == ofbx::Object::Type::NULL_NODE;
}

bool buildBoneMap(const ofbx::IScene& scene, BoneMap& result)
{
    ct::HashMap<std::uint64_t, bool> required;
    const auto requireChain = [&](const ofbx::Object* leaf)
    {
        for (const ofbx::Object* object = leaf; isSkeletonNode(object);
                object = object->getParent())
            required.put(object->id, true);
    };

    for (int i = 0; i < scene.getAllObjectCount(); ++i)
    {
        const ofbx::Object* object = scene.getAllObjects()[i];
        if (object && object->getType() == ofbx::Object::Type::LIMB_NODE) requireChain(object);
    }
    for (int mi = 0; mi < scene.getMeshCount(); ++mi)
    {
        const ofbx::Mesh* mesh = scene.getMesh(mi);
        const ofbx::Geometry* geometry = mesh ? mesh->getGeometry() : nullptr;
        const ofbx::Skin* skin = geometry ? geometry->getSkin() : nullptr;
        if (!skin) continue;
        for (int ci = 0; ci < skin->getClusterCount(); ++ci)
        {
            const ofbx::Cluster* cluster = skin->getCluster(ci);
            if (cluster) requireChain(cluster->getLink());
        }
    }

    result = BoneMap();
    while (result.bones.size() < required.size())
    {
        bool progressed = false;
        for (int i = 0; i < scene.getAllObjectCount(); ++i)
        {
            const ofbx::Object* object = scene.getAllObjects()[i];
            if (!object || !required.contains(object->id) || result.indexById.contains(object->id))
                continue;
            const ofbx::Object* parent = object->getParent();
            if (isSkeletonNode(parent) && required.contains(parent->id) &&
                    !result.indexById.contains(parent->id))
                continue;
            if (result.bones.size() >= static_cast<std::size_t>(kMaxBones)) return false;
            result.indexById.put(object->id, static_cast<int>(result.bones.size()));
            result.bones.push_back(object);
            progressed = true;
        }
        if (!progressed) return false;
    }
    return true;
}

bool fail(ModelData& out, const char* message)
{
    out.error = message;
    return false;
}

bool failParse(ModelData& out)
{
    out.error = "cannot parse FBX";
    const char* detail = ofbx::getError();
    if (detail && detail[0])
    {
        out.error += ": ";
        out.error += detail;
    }
    return false;
}

int imageFor(Context& c, ModelData& out, const ofbx::Texture* texture)
{
    if (!texture) return -1;
    if (const int* known = c.textureSlot.find(texture->id)) return *known;

    ct::String name = viewString(texture->getRelativeFileName());
    if (name.empty()) name = viewString(texture->getFileName());

    ModelImageData image;
    const ofbx::DataView embedded = texture->getEmbeddedData();
    if (embedded.begin && embedded.end > embedded.begin + 4)
    {
        ct::Vector<std::uint8_t> bytes;
        bytes.resize(static_cast<std::size_t>(embedded.end - embedded.begin - 4));
        memcpy(bytes.data(), embedded.begin + 4, bytes.size());
        image.name = name.empty() ? ct::String("embedded") : fileNameOf(name);
        image.size = bytes.size();
        image.source = static_cast<std::uint32_t>(c.source->embedded.size());
        c.source->embedded.push_back(static_cast<ct::Vector<std::uint8_t>&&>(bytes));
        out.imageBytes += image.size;
    }
    else
    {
        if (name.empty()) return -1;
        image.name = fileNameOf(name);
        image.path = resolveTextureFile(c.directory, name);
    }
    out.images.push_back(static_cast<ModelImageData&&>(image));

    ModelTextureData slot;
    slot.image = static_cast<int>(out.images.size() - 1);
    out.textures.push_back(slot);
    const int index = static_cast<int>(out.textures.size() - 1);
    c.textureSlot.put(texture->id, index);
    return index;
}

int materialFor(Context& c, ModelData& out, const ofbx::Material* material)
{
    if (!material) return -1;
    if (const int* known = c.materialSlot.find(material->id)) return *known;

    ModelMaterialData data;
    data.name = objectName(*material);
    const ofbx::Color diffuse = material->getDiffuseColor();
    data.baseColor = Math::Vec4(diffuse.r, diffuse.g, diffuse.b, 1.0f);
    data.metallic = 0.0f;
    data.roughness = 1.0f;
    data.baseColorTexture = imageFor(c, out, material->getTexture(ofbx::Texture::DIFFUSE));
    data.normalTexture = imageFor(c, out, material->getTexture(ofbx::Texture::NORMAL));
    out.materials.push_back(static_cast<ModelMaterialData&&>(data));
    const int index = static_cast<int>(out.materials.size() - 1);
    c.materialSlot.put(material->id, index);
    return index;
}

bool contains(const ct::String& text, const char* part)
{
    return strstr(text.c_str(), part) != nullptr;
}

int attachmentBoneFor(const Context& c, const ofbx::Mesh& mesh)
{
    const ct::String meshName = objectName(mesh);
    const bool facial = contains(meshName, "Eye") || contains(meshName, "eye") ||
                        contains(meshName, "Eyelid") || contains(meshName, "Teeth") ||
                        contains(meshName, "Tongue");
    const char* preferred = nullptr;
    if (contains(meshName, "Le_Eye") || contains(meshName, "LeftEye")) preferred = "LeftEye";
    else if (contains(meshName, "Ri_Eye") || contains(meshName, "RightEye"))
        preferred = "RightEye";
    else if (contains(meshName, "Teeth") || contains(meshName, "Tongue"))
        preferred = "Jaw";
    else if (facial)
        preferred = "Head";

    if (preferred)
        for (std::size_t i = 0; i < c.bones.bones.size(); ++i)
            if (objectName(*c.bones.bones[i]) == preferred) return static_cast<int>(i);
    for (const ofbx::Object* parent = mesh.getParent(); parent; parent = parent->getParent())
    {
        const int index = c.bones.indexOf(parent);
        if (index >= 0) return index;
    }
    return -1;
}

std::uint32_t packJoints(int b0, int b1, int b2, int b3)
{
    return static_cast<std::uint32_t>(b0) | (static_cast<std::uint32_t>(b1) << 8) |
           (static_cast<std::uint32_t>(b2) << 16) | (static_cast<std::uint32_t>(b3) << 24);
}

int decodeIndex(int index)
{
    return index < 0 ? -index - 1 : index;
}

bool buildMesh(Context& c, ModelData& out, const ofbx::Mesh& fbxMesh)
{
    const ofbx::Geometry* geometry = fbxMesh.getGeometry();
    if (!geometry) return true;
    const int vertexCount = geometry->getVertexCount();
    const int indexCount = geometry->getIndexCount();
    if (vertexCount <= 0 || indexCount <= 0) return true;

    const ofbx::Vec3* srcPositions = geometry->getVertices();
    const ofbx::Vec3* srcNormals = geometry->getNormals();
    const ofbx::Vec3* srcTangents = geometry->getTangents();
    const ofbx::Vec2* srcUvs = geometry->getUVs(0);
    const ofbx::Vec2* srcUvs2 = geometry->getUVs(1);
    const ofbx::Vec4* srcColors = geometry->getColors();
    const int* srcIndices = geometry->getFaceIndices();
    const int* srcMaterials = geometry->getMaterials();
    const ofbx::Skin* skin = geometry->getSkin();
    const bool skinned = skin && skin->getClusterCount() > 0;

    Math::Mat4 nodeTransform = toMat4(fbxMesh.getGlobalTransform());
    int attachmentBone = -1;
    if (!skinned && c.hasSkin)
    {
        attachmentBone = attachmentBoneFor(c, fbxMesh);
        if (attachmentBone >= 0)
            nodeTransform = toMat4(c.bones.bones[static_cast<std::size_t>(attachmentBone)]
                                            ->getGlobalTransform())
                                    .Inverse() *
                            toMat4(fbxMesh.getGlobalTransform());
    }
    if (skinned) nodeTransform = Math::Mat4::Identity();

    Math::Mat4 finalTransform = nodeTransform * toMat4(fbxMesh.getGeometricMatrix());
    if (!finite(finalTransform)) finalTransform = Math::Mat4::Identity();
    const Math::Mat4 normalTransform =
            skinned ? Math::Mat4::Identity() : finalTransform.Inverse().Transposed();

    ct::Vector<WeightSlots> slots;
    if (skinned)
    {
        slots.resize(static_cast<std::size_t>(vertexCount));
        for (int ci = 0; ci < skin->getClusterCount(); ++ci)
        {
            const ofbx::Cluster* cluster = skin->getCluster(ci);
            if (!cluster || !cluster->getLink()) continue;
            const int bone = c.bones.indexOf(cluster->getLink());
            if (bone < 0) return fail(out, "skin cluster linked to an unknown bone");
            const int* clusterIndices = cluster->getIndices();
            const double* clusterWeights = cluster->getWeights();
            const int count = cluster->getIndicesCount();
            for (int i = 0; i < count; ++i)
            {
                const int index = clusterIndices[i];
                if (index >= 0 && index < vertexCount)
                    slots[static_cast<std::size_t>(index)].add(bone, toFloat(clusterWeights[i]));
            }
        }
    }

    const std::size_t corners = static_cast<std::size_t>(vertexCount);
    ct::Vector<Math::Vec3> positions;
    ct::Vector<Math::Vec3> normals;
    ct::Vector<Math::Vec4> tangents;
    ct::Vector<Math::Vec4> colors;
    ct::Vector<Math::Vec2> uvs;
    ct::Vector<Math::Vec2> uvs2;
    ct::Vector<std::uint32_t> joints;
    ct::Vector<Math::Vec4> weights;
    positions.resize(corners);
    if (srcNormals) normals.resize(corners);
    if (srcTangents) tangents.resize(corners);
    if (srcColors) colors.resize(corners);
    if (srcUvs) uvs.resize(corners);
    if (srcUvs2) uvs2.resize(corners);
    if (c.hasSkin)
    {
        joints.resize(corners);
        weights.resize(corners);
    }

    for (std::size_t vi = 0; vi < corners; ++vi)
    {
        Math::Vec3 p = toVec3(srcPositions[vi]);
        if (!finite(p)) p = Math::Vec3(0.0f, 0.0f, 0.0f);
        Math::Vec3 transformed = xyz(finalTransform * Math::Vec4(p, 1.0f));
        positions[vi] = finite(transformed) ? transformed : p;

        if (srcNormals)
        {
            Math::Vec3 n = toVec3(srcNormals[vi]);
            if (n.Dot(n) > 1e-8f) n = xyz(normalTransform * Math::Vec4(n, 0.0f)).NormalizedSafe();
            normals[vi] = finite(n) ? n : Math::Vec3(0.0f, 1.0f, 0.0f);
        }
        if (srcTangents)
        {
            const Math::Vec3 t = toVec3(srcTangents[vi]);
            tangents[vi] = Math::Vec4(t.x, t.y, t.z, 1.0f);
        }
        if (srcColors)
            colors[vi] = Math::Vec4(toFloat(srcColors[vi].x), toFloat(srcColors[vi].y),
                    toFloat(srcColors[vi].z), toFloat(srcColors[vi].w));
        if (srcUvs)
        {
            Math::Vec2 uv = toVec2(srcUvs[vi]);
            uv.y = 1.0f - uv.y;
            uvs[vi] = uv;
        }
        if (srcUvs2)
        {
            Math::Vec2 uv = toVec2(srcUvs2[vi]);
            uv.y = 1.0f - uv.y;
            uvs2[vi] = uv;
        }
        if (c.hasSkin)
        {
            if (skinned)
            {
                const WeightSlots& s = slots[vi];
                const float sum = s.weights[0] + s.weights[1] + s.weights[2] + s.weights[3];
                if (sum <= 1e-8f)
                {
                    joints[vi] = 0;
                    weights[vi] = Math::Vec4(1.0f, 0.0f, 0.0f, 0.0f);
                }
                else
                {
                    const int b0 = s.ids[0] >= 0 ? s.ids[0] : 0;
                    const int b1 = s.ids[1] >= 0 ? s.ids[1] : b0;
                    const int b2 = s.ids[2] >= 0 ? s.ids[2] : b0;
                    const int b3 = s.ids[3] >= 0 ? s.ids[3] : b0;
                    joints[vi] = packJoints(b0, b1, b2, b3);
                    weights[vi] = Math::Vec4(s.weights[0] / sum, s.weights[1] / sum,
                            s.weights[2] / sum, s.weights[3] / sum);
                }
            }
            else
            {
                const int bone = attachmentBone >= 0 ? attachmentBone : 0;
                joints[vi] = packJoints(bone, bone, bone, bone);
                weights[vi] = Math::Vec4(1.0f, 0.0f, 0.0f, 0.0f);
            }
        }
    }

    const int triangleCount = indexCount / 3;
    ct::Vector<int> triangleSlot;
    ct::Vector<int> order;
    triangleSlot.resize(static_cast<std::size_t>(triangleCount), -1);
    for (int tri = 0; tri < triangleCount; ++tri)
    {
        const int materialIndex = srcMaterials ? srcMaterials[tri] : 0;
        const ofbx::Material* material =
                (materialIndex >= 0 && materialIndex < fbxMesh.getMaterialCount())
                        ? fbxMesh.getMaterial(materialIndex)
                        : nullptr;
        const int slot = materialFor(c, out, material);
        triangleSlot[static_cast<std::size_t>(tri)] = slot;
        bool seen = false;
        for (std::size_t i = 0; i < order.size() && !seen; ++i) seen = order[i] == slot;
        if (!seen) order.push_back(slot);
    }

    ModelMeshData mesh;
    mesh.name = objectName(fbxMesh);
    ct::Vector<int> remap;
    for (std::size_t o = 0; o < order.size(); ++o)
    {
        const int slot = order[o];
        SurfaceData surface;
        remap.clear();
        remap.resize(corners, -1);
        for (int tri = 0; tri < triangleCount; ++tri)
        {
            if (triangleSlot[static_cast<std::size_t>(tri)] != slot) continue;
            int cornerIndex[3];
            bool valid = true;
            for (int k = 0; k < 3; ++k)
            {
                cornerIndex[k] = decodeIndex(srcIndices[tri * 3 + k]);
                valid = valid && cornerIndex[k] >= 0 && cornerIndex[k] < vertexCount;
            }
            if (!valid) continue;
            for (int k = 0; k < 3; ++k)
            {
                const std::size_t corner = static_cast<std::size_t>(cornerIndex[k]);
                int& mapped = remap[corner];
                if (mapped < 0)
                {
                    mapped = static_cast<int>(surface.positions.size());
                    surface.positions.push_back(positions[corner]);
                    if (srcNormals) surface.normals.push_back(normals[corner]);
                    if (srcTangents) surface.tangents.push_back(tangents[corner]);
                    if (srcColors) surface.colors.push_back(colors[corner]);
                    if (srcUvs) surface.uvs.push_back(uvs[corner]);
                    if (srcUvs2) surface.uvs2.push_back(uvs2[corner]);
                    if (c.hasSkin)
                    {
                        surface.joints.push_back(joints[corner]);
                        surface.weights.push_back(weights[corner]);
                    }
                }
                surface.indices.push_back(static_cast<std::uint32_t>(mapped));
            }
        }
        if (surface.indices.empty()) continue;

        ModelPrimitiveData primitive;
        primitive.material = slot;
        primitive.attributeMask = kVertexPosition;
        if (srcNormals) primitive.attributeMask |= kVertexNormal;
        if (srcTangents) primitive.attributeMask |= kVertexTangent;
        if (srcColors) primitive.attributeMask |= kVertexColor;
        if (srcUvs) primitive.attributeMask |= kVertexUv;
        if (srcUvs2) primitive.attributeMask |= kVertexUv2;
        if (c.hasSkin) primitive.attributeMask |= kVertexJoints | kVertexWeights;
        primitive.vertexCount = static_cast<std::uint32_t>(surface.positions.size());
        primitive.indexCount = static_cast<std::uint32_t>(surface.indices.size());
        primitive.skinned = c.hasSkin;
        primitive.aabb = Math::Box(surface.positions[0], surface.positions[0]);
        for (std::size_t i = 1; i < surface.positions.size(); ++i)
            primitive.aabb = primitive.aabb.Expanded(surface.positions[i]);
        primitive.hasAabb = true;
        primitive.source = static_cast<std::uint32_t>(c.source->surfaces.size());
        primitive.sourceBytes =
                static_cast<std::uint64_t>(primitive.vertexCount) *
                        (sizeof(Math::Vec3) * 2 + sizeof(Math::Vec4) * 2 + sizeof(Math::Vec2)) +
                static_cast<std::uint64_t>(primitive.indexCount) * sizeof(std::uint32_t);
        out.geometryBytes += primitive.sourceBytes;
        ++out.primitiveCount;
        mesh.primitives.push_back(primitive);
        c.source->surfaces.push_back(static_cast<SurfaceData&&>(surface));
    }
    if (mesh.primitives.empty()) return true;

    out.meshes.push_back(static_cast<ModelMeshData&&>(mesh));
    ModelNodeData node;
    node.name = objectName(fbxMesh);
    node.mesh = static_cast<int>(out.meshes.size() - 1);
    node.skeleton = c.hasSkin ? 0 : -1;
    out.nodes.push_back(static_cast<ModelNodeData&&>(node));
    out.roots.push_back(static_cast<int>(out.nodes.size() - 1));
    return true;
}

void collectClusterBinds(const Context& c, ct::HashMap<std::uint64_t, Math::Mat4>& bindGlobal)
{
    for (int mi = 0; mi < c.scene->getMeshCount(); ++mi)
    {
        const ofbx::Mesh* mesh = c.scene->getMesh(mi);
        const ofbx::Geometry* geometry = mesh ? mesh->getGeometry() : nullptr;
        const ofbx::Skin* skin = geometry ? geometry->getSkin() : nullptr;
        if (!skin) continue;
        for (int ci = 0; ci < skin->getClusterCount(); ++ci)
        {
            const ofbx::Cluster* cluster = skin->getCluster(ci);
            if (cluster && cluster->getLink())
                bindGlobal.put(cluster->getLink()->id, toMat4(cluster->getTransformLinkMatrix()));
        }
    }
}

bool buildSkeleton(const Context& c, Skeleton& skeleton)
{
    ct::HashMap<std::uint64_t, Math::Mat4> bindGlobalById;
    collectClusterBinds(c, bindGlobalById);

    for (std::size_t i = 0; i < c.bones.bones.size(); ++i)
    {
        const ofbx::Object* object = c.bones.bones[i];
        const ofbx::Object* parent = object->getParent();
        const int parentIndex = c.bones.indexOf(parent);

        Math::Mat4 bindGlobal = toMat4(object->getGlobalTransform());
        const Math::Mat4* clusterBind = bindGlobalById.find(object->id);
        if (clusterBind) bindGlobal = *clusterBind;

        Math::Mat4 parentBindGlobal = Math::Mat4::Identity();
        if (parentIndex >= 0)
        {
            const Math::Mat4* parentBind = bindGlobalById.find(parent->id);
            parentBindGlobal = parentBind ? *parentBind : toMat4(parent->getGlobalTransform());
        }
        const Math::Mat4 bindLocal =
                parentIndex >= 0 ? parentBindGlobal.Inverse() * bindGlobal : bindGlobal;
        const Math::Mat4 inverseBind = clusterBind ? clusterBind->Inverse()
                                                   : toMat4(object->getGlobalTransform()).Inverse();
        if (!skeleton.addBone(objectName(*object), parentIndex, bindLocal, inverseBind))
            return false;
    }
    return skeleton.finalize();
}

const ofbx::AnimationLayer* firstLayerWithCurves(const Context& c,
        const ofbx::AnimationStack& stack)
{
    const char* properties[3] = { "Lcl Translation", "Lcl Rotation", "Lcl Scaling" };
    for (int li = 0; li < kMaxLayers; ++li)
    {
        const ofbx::AnimationLayer* layer = stack.getLayer(li);
        if (!layer) continue;
        for (std::size_t b = 0; b < c.bones.bones.size(); ++b)
            for (int p = 0; p < 3; ++p)
                if (layer->getCurveNode(*c.bones.bones[b], properties[p])) return layer;
    }
    return nullptr;
}

ofbx::Vec3 sampleVec3(const Channels* channels, int type, float time, const ofbx::Vec3& fallback)
{
    if (!channels) return fallback;
    ofbx::Vec3 out;
    out.x = sampleScalarChannel(channels->times[type][0], channels->values[type][0], time,
            toFloat(fallback.x));
    out.y = sampleScalarChannel(channels->times[type][1], channels->values[type][1], time,
            toFloat(fallback.y));
    out.z = sampleScalarChannel(channels->times[type][2], channels->values[type][2], time,
            toFloat(fallback.z));
    return out;
}

bool buildClip(const Context& c, const ofbx::AnimationStack& stack, const Skeleton& skeleton,
        AnimationClip& clip)
{
    const ofbx::AnimationLayer* layer = firstLayerWithCurves(c, stack);
    if (!layer) return false;

    const std::size_t boneCount = c.bones.bones.size();
    const char* properties[3] = { "Lcl Translation", "Lcl Rotation", "Lcl Scaling" };
    ct::Vector<Channels> perBone;
    perBone.resize(boneCount);
    Floats times;
    for (std::size_t b = 0; b < boneCount; ++b)
    {
        const ofbx::Object& bone = *c.bones.bones[b];
        for (int type = 0; type < 3; ++type)
        {
            const ofbx::AnimationCurveNode* node = layer->getCurveNode(bone, properties[type]);
            if (!node) continue;
            for (int axis = 0; axis < 3; ++axis)
            {
                const ofbx::AnimationCurve* curve = node->getCurve(axis);
                if (!curve || curve->getKeyCount() <= 0) continue;
                const ofbx::i64* keyTimes = curve->getKeyTime();
                const float* keyValues = curve->getKeyValue();
                Floats& t = perBone[b].times[type][axis];
                Floats& v = perBone[b].values[type][axis];
                for (int k = 0; k < curve->getKeyCount(); ++k)
                {
                    t.push_back(toFloat(ofbx::fbxTimeToSeconds(keyTimes[k])));
                    v.push_back(keyValues[k]);
                }
                appendTimes(times, t);
            }
        }
    }
    sortUniqueTimes(times);
    if (times.empty()) return false;

    ct::HashMap<std::uint64_t, Math::Mat4> clusterBind;
    collectClusterBinds(c, clusterBind);
    ct::Vector<Math::Mat4> authoredBindLocal;
    authoredBindLocal.resize(boneCount, Math::Mat4::Identity());
    for (std::size_t b = 0; b < boneCount; ++b)
    {
        const ofbx::Object* object = c.bones.bones[b];
        const int parent = c.bones.indexOf(object->getParent());
        const Math::Mat4* bind = clusterBind.find(object->id);
        if (bind && parent >= 0)
        {
            const Math::Mat4* parentBind =
                    clusterBind.find(c.bones.bones[static_cast<std::size_t>(parent)]->id);
            authoredBindLocal[b] = parentBind ? parentBind->Inverse() * *bind : *bind;
        }
        else
            authoredBindLocal[b] = toMat4(object->getLocalTransform());
    }

    ct::Vector<BoneTrack> tracks;
    tracks.resize(boneCount);
    for (std::size_t b = 0; b < boneCount; ++b) tracks[b].bone = static_cast<int>(b);

    for (std::size_t ti = 0; ti < times.size(); ++ti)
    {
        const float time = times[ti];
        for (std::size_t b = 0; b < boneCount; ++b)
        {
            const ofbx::Object* object = c.bones.bones[b];
            const Channels* channels = &perBone[b];
            const ofbx::Vec3 translation =
                    sampleVec3(channels, 0, time, object->getLocalTranslation());
            const ofbx::Vec3 rotation = sampleVec3(channels, 1, time, object->getLocalRotation());
            const ofbx::Vec3 scale = sampleVec3(channels, 2, time, object->getLocalScaling());

            const Math::Mat4 authoredLocal =
                    toMat4(object->evalLocal(translation, rotation, scale));
            const Math::Mat4 deltaLocal = authoredBindLocal[b].Inverse() * authoredLocal;
            const Math::Mat4 correctedLocal =
                    skeleton.bone(static_cast<std::uint32_t>(b)).bindLocal * deltaLocal;

            LocalPose pose;
            if (!LocalPose::fromMatrix(correctedLocal, pose)) continue;
            BoneTrack& track = tracks[b];
            if (!track.rotations.empty())
            {
                const Math::Quaternion& previous = track.rotations.back();
                const float dot = previous.x * pose.rotation.x + previous.y * pose.rotation.y +
                                  previous.z * pose.rotation.z + previous.w * pose.rotation.w;
                if (dot < 0.0f) pose.rotation = -pose.rotation;
            }
            const float length = pose.rotation.Length();
            if (length > 1e-8f) pose.rotation = pose.rotation * (1.0f / length);
            track.times.push_back(time);
            track.positions.push_back(pose.position);
            track.rotations.push_back(pose.rotation);
            track.scales.push_back(pose.scale);
        }
    }

    const float firstKeyTime = times[0];
    const float lastKeyTime = times.back();
    clip = AnimationClip();
    for (std::size_t b = 0; b < boneCount; ++b)
    {
        BoneTrack& track = tracks[b];
        if (track.times.empty()) continue;
        for (std::size_t k = 0; k < track.times.size(); ++k) track.times[k] -= firstKeyTime;
        clip.tracks().push_back(static_cast<BoneTrack&&>(track));
    }
    if (clip.tracks().empty()) return false;

    const ct::String stackName = objectName(stack);
    clip.setName(!stackName.empty() && stackName != "Take 001" ? stackName : c.baseName);
    clip.setSkeleton(0);
    const float duration = lastKeyTime - firstKeyTime;
    clip.setDuration(duration > 0.0f ? duration : 1.0f);
    return true;
}

bool build(const std::uint8_t* data, std::size_t size, const ct::String& path, ModelData& out)
{
    out = ModelData();
    out.path = path;
    out.directory = directoryOf(path);

    ofbx::IScene* scene = ofbx::load(data, static_cast<int>(size),
            static_cast<ofbx::u64>(ofbx::LoadFlags::TRIANGULATE));
    if (!scene) return failParse(out);

    Context c;
    c.scene = scene;
    c.path = path;
    c.directory = out.directory;
    c.baseName = baseNameOf(path);
    ct::Unique<FbxSource> owned = ct::make_unique<FbxSource>();
    c.source = owned.get();

    if (!buildBoneMap(*scene, c.bones))
    {
        scene->destroy();
        return fail(out, "invalid bone hierarchy or more than 256 bones");
    }

    for (int mi = 0; mi < scene->getMeshCount() && !c.hasSkin; ++mi)
    {
        const ofbx::Mesh* mesh = scene->getMesh(mi);
        const ofbx::Geometry* geometry = mesh ? mesh->getGeometry() : nullptr;
        if (geometry && geometry->getSkin() && geometry->getSkin()->getClusterCount() > 0)
            c.hasSkin = true;
    }

    for (int mi = 0; mi < scene->getMeshCount(); ++mi)
    {
        const ofbx::Mesh* mesh = scene->getMesh(mi);
        if (!mesh) continue;
        if (!buildMesh(c, out, *mesh))
        {
            scene->destroy();
            return false;
        }
    }

    if (!c.bones.bones.empty())
    {
        Skeleton skeleton;
        if (!buildSkeleton(c, skeleton))
        {
            scene->destroy();
            return fail(out, "skeleton is invalid");
        }
        out.skeletons.push_back(static_cast<Skeleton&&>(skeleton));

        for (int si = 0; si < scene->getAnimationStackCount(); ++si)
        {
            const ofbx::AnimationStack* stack = scene->getAnimationStack(si);
            if (!stack) continue;
            AnimationClip clip;
            if (buildClip(c, *stack, out.skeletons[0], clip))
                out.animations.push_back(static_cast<AnimationClip&&>(clip));
        }
    }

    scene->destroy();

    if (out.meshes.empty() && out.skeletons.empty()) return fail(out, "no usable geometry");

    out.source = ct::Unique<ModelSource>(static_cast<ct::Unique<FbxSource>&&>(owned));
    return true;
}

} // namespace

const char* FbxImporter::name() const
{
    return "FBX";
}

bool FbxImporter::supports(const char* extension) const
{
    return extension && strcmp(extension, "fbx") == 0;
}

bool FbxImporter::import(const ct::String& path, ModelData& out)
{
    out = ModelData();
    const ct::Vector<std::uint8_t> bytes = FileSystem::getSingleton().readBinary(path);
    if (bytes.empty()) return fail(out, "cannot read file");
    return build(bytes.data(), bytes.size(), path, out);
}

bool FbxImporter::importMemory(const std::uint8_t* data, std::size_t size,
        const ct::String& virtualName, ModelData& out)
{
    out = ModelData();
    if (!data || size == 0) return fail(out, "empty model data");
    return build(data, size, virtualName, out);
}

bool FbxImporter::readPrimitive(const ModelData& model, const ModelPrimitiveData& primitive,
        SurfaceData& out) const
{
    const FbxSource* source = static_cast<const FbxSource*>(model.source.get());
    if (!source || primitive.source >= source->surfaces.size()) return false;
    out = source->surfaces[primitive.source];
    return true;
}

bool FbxImporter::readImage(const ModelData& model, const ModelImageData& image,
        ct::Vector<std::uint8_t>& out) const
{
    out.clear();
    const FbxSource* source = static_cast<const FbxSource*>(model.source.get());
    if (!source || !image.embedded() || image.source >= source->embedded.size()) return false;
    out = source->embedded[image.source];
    return true;
}

} // namespace prisma3d
