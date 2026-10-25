#include "Check.h"

#include "prisma3d/import/OgreMeshImporter.h"

#include "platform.h"

#include <math.h>
#include <string.h>

using namespace prisma3d;

namespace
{

const float kTolerance = 1e-4f;
const char* const kDirectory = PRISMA3D_TEST_DIR "/ogre";

typedef ct::Vector<std::uint8_t> Bytes;

bool near(float a, float b)
{
    return fabsf(a - b) <= kTolerance;
}

bool nearVec3(const Math::Vec3& a, float x, float y, float z)
{
    return near(a.x, x) && near(a.y, y) && near(a.z, z);
}

void putU8(Bytes& bytes, std::uint8_t value)
{
    bytes.push_back(value);
}

void putU16(Bytes& bytes, std::uint16_t value)
{
    bytes.push_back(static_cast<std::uint8_t>(value));
    bytes.push_back(static_cast<std::uint8_t>(value >> 8));
}

void putU32(Bytes& bytes, std::uint32_t value)
{
    for (int i = 0; i < 4; ++i) bytes.push_back(static_cast<std::uint8_t>(value >> (8 * i)));
}

void putF32(Bytes& bytes, float value)
{
    std::uint32_t raw;
    memcpy(&raw, &value, sizeof(raw));
    putU32(bytes, raw);
}

void putVec3(Bytes& bytes, float x, float y, float z)
{
    putF32(bytes, x);
    putF32(bytes, y);
    putF32(bytes, z);
}

void putQuat(Bytes& bytes, float x, float y, float z, float w)
{
    putF32(bytes, x);
    putF32(bytes, y);
    putF32(bytes, z);
    putF32(bytes, w);
}

void putString(Bytes& bytes, const char* text)
{
    for (const char* c = text; *c; ++c) bytes.push_back(static_cast<std::uint8_t>(*c));
    bytes.push_back('\n');
}

std::size_t beginChunk(Bytes& bytes, std::uint16_t id)
{
    const std::size_t start = bytes.size();
    putU16(bytes, id);
    putU32(bytes, 0);
    return start;
}

void patchLength(Bytes& bytes, std::size_t start, std::uint32_t length)
{
    for (int i = 0; i < 4; ++i)
        bytes[start + 2 + static_cast<std::size_t>(i)] =
                static_cast<std::uint8_t>(length >> (8 * i));
}

void endChunk(Bytes& bytes, std::size_t start)
{
    patchLength(bytes, start, static_cast<std::uint32_t>(bytes.size() - start));
}

void putElement(Bytes& bytes, std::uint16_t source, std::uint16_t type, std::uint16_t semantic,
        std::uint16_t offset, std::uint16_t index)
{
    const std::size_t chunk = beginChunk(bytes, 0x5110);
    putU16(bytes, source);
    putU16(bytes, type);
    putU16(bytes, semantic);
    putU16(bytes, offset);
    putU16(bytes, index);
    endChunk(bytes, chunk);
}

std::size_t beginBuffer(Bytes& bytes, std::uint16_t source, std::uint16_t vertexSize)
{
    const std::size_t buffer = beginChunk(bytes, 0x5200);
    putU16(bytes, source);
    putU16(bytes, vertexSize);
    return buffer;
}

void putAssignment(Bytes& bytes, std::uint16_t id, std::uint32_t vertex, std::uint16_t bone,
        float weight)
{
    const std::size_t chunk = beginChunk(bytes, id);
    putU32(bytes, vertex);
    putU16(bytes, bone);
    putF32(bytes, weight);
    endChunk(bytes, chunk);
}

const float kPositions[4][3] = { { 0.0f, 0.0f, 0.0f }, { 1.0f, 0.0f, 0.0f }, { 1.0f, 1.0f, 0.0f },
    { 0.0f, 1.0f, 0.5f } };
const float kNormals[4][3] = { { 0.0f, 0.0f, 1.0f }, { 0.0f, 1.0f, 0.0f }, { 1.0f, 0.0f, 0.0f },
    { 0.0f, 0.6f, 0.8f } };
const std::uint32_t kColors[4] = { 0x80FF4020u, 0xFF000000u, 0x00FFFFFFu, 0xFF102030u };
const float kUvs[4][2] = { { 0.0f, 0.0f }, { 1.0f, 0.0f }, { 1.0f, 1.0f }, { 0.0f, 1.0f } };
const std::uint16_t kBodyIndices[6] = { 0, 1, 2, 0, 2, 3 };
const float kGlassPositions[3][3] = { { 5.0f, 0.0f, 0.0f }, { 6.0f, 0.0f, 0.0f },
    { 6.0f, 2.0f, 0.0f } };

bool gLegacyQuirks = false;

void putLegacyLod(Bytes& mesh, bool strategyName)
{
    const std::size_t lod = beginChunk(mesh, 0x8000);
    if (strategyName) putString(mesh, "Distance");
    putU16(mesh, 2);
    putU8(mesh, 0);
    patchLength(mesh, lod, 6 + 2 + 1);
    const std::size_t usage = beginChunk(mesh, 0x8100);
    putF32(mesh, 100.0f);
    endChunk(mesh, usage);
    for (int sub = 0; sub < 2; ++sub)
    {
        const std::size_t generated = beginChunk(mesh, 0x8120);
        putU32(mesh, 3);
        putU8(mesh, 0);
        for (int i = 0; i < 3; ++i) putU16(mesh, static_cast<std::uint16_t>(i));
        endChunk(mesh, generated);
    }
}

void buildMesh(Bytes& mesh, const char* version, const char* link, int badHandle)
{
    mesh.clear();
    putU16(mesh, 0x1000);
    putString(mesh, version);

    const std::size_t root = beginChunk(mesh, 0x3000);
    putU8(mesh, 1);

    const std::size_t geometry = beginChunk(mesh, 0x5000);
    putU32(mesh, 4);
    const std::size_t declaration = beginChunk(mesh, 0x5100);
    putElement(mesh, 0, 2, 1, 0, 0);
    putElement(mesh, 0, 2, 4, 12, 0);
    putElement(mesh, 1, 10, 5, 0, 0);
    putElement(mesh, 1, 1, 7, 4, 0);
    endChunk(mesh, declaration);

    const std::size_t first = beginBuffer(mesh, 0, 24);
    const std::size_t firstData = beginChunk(mesh, 0x5210);
    for (int v = 0; v < 4; ++v)
    {
        putVec3(mesh, kPositions[v][0], kPositions[v][1], kPositions[v][2]);
        putVec3(mesh, kNormals[v][0], kNormals[v][1], kNormals[v][2]);
    }
    endChunk(mesh, firstData);
    endChunk(mesh, first);

    const std::size_t second = beginBuffer(mesh, 1, 12);
    const std::size_t secondData = beginChunk(mesh, 0x5210);
    for (int v = 0; v < 4; ++v)
    {
        putU32(mesh, kColors[v]);
        putF32(mesh, kUvs[v][0]);
        putF32(mesh, kUvs[v][1]);
    }
    endChunk(mesh, secondData);
    endChunk(mesh, second);
    endChunk(mesh, geometry);

    const std::size_t body = beginChunk(mesh, 0x4000);
    putString(mesh, "Body");
    putU8(mesh, 1);
    putU32(mesh, 6);
    putU8(mesh, 0);
    for (int i = 0; i < 6; ++i) putU16(mesh, kBodyIndices[i]);
    endChunk(mesh, body);

    const std::size_t glass = beginChunk(mesh, 0x4000);
    putString(mesh, "Glass");
    putU8(mesh, 0);
    putU32(mesh, 3);
    putU8(mesh, 1);
    putU32(mesh, 0);
    putU32(mesh, 1);
    putU32(mesh, 2);
    const std::size_t ownGeometry = beginChunk(mesh, 0x5000);
    putU32(mesh, 3);
    const std::size_t ownDeclaration = beginChunk(mesh, 0x5100);
    putElement(mesh, 0, 2, 1, 0, 0);
    endChunk(mesh, ownDeclaration);
    const std::size_t ownBuffer = beginBuffer(mesh, 0, 12);
    const std::size_t ownData = beginChunk(mesh, 0x5210);
    for (int v = 0; v < 3; ++v)
        putVec3(mesh, kGlassPositions[v][0], kGlassPositions[v][1], kGlassPositions[v][2]);
    endChunk(mesh, ownData);
    endChunk(mesh, ownBuffer);
    endChunk(mesh, ownGeometry);
    const std::size_t operation = beginChunk(mesh, 0x4010);
    putU16(mesh, 6);
    endChunk(mesh, operation);
    endChunk(mesh, glass);
    if (gLegacyQuirks)
    {
        patchLength(mesh, body, 6 + 40);
        patchLength(mesh, glass, 5000);
        putLegacyLod(mesh, strcmp(version, "[MeshSerializer_v1.40]") != 0);
    }

    const std::size_t skeletonLink = beginChunk(mesh, 0x6000);
    putString(mesh, link);
    endChunk(mesh, skeletonLink);

    const std::uint16_t armHandle = badHandle >= 0 ? static_cast<std::uint16_t>(badHandle) : 1;
    putAssignment(mesh, 0x7000, 0, 0, 0.75f);
    putAssignment(mesh, 0x7000, 0, 1, 0.25f);
    putAssignment(mesh, 0x7000, 1, 1, 0.25f);
    putAssignment(mesh, 0x7000, 1, 0, 0.75f);
    putAssignment(mesh, 0x7000, 2, 0, 0.75f);
    putAssignment(mesh, 0x7000, 2, armHandle, 0.25f);
    putAssignment(mesh, 0x7000, 3, 0, 0.6f);
    putAssignment(mesh, 0x7000, 3, 1, 0.2f);
    endChunk(mesh, root);
    if (gLegacyQuirks) patchLength(mesh, root, static_cast<std::uint32_t>(mesh.size() + 500));
}

void putBone(Bytes& skeleton, const char* name, std::uint16_t handle, float x, float y, float z)
{
    const std::size_t chunk = beginChunk(skeleton, 0x2000);
    putString(skeleton, name);
    putU16(skeleton, handle);
    putVec3(skeleton, x, y, z);
    putQuat(skeleton, 0.0f, 0.0f, 0.0f, 1.0f);
    patchLength(skeleton, chunk, 6 + 2 + 12 + 16);
}

void buildSkeleton(Bytes& skeleton, bool armFirst)
{
    skeleton.clear();
    putU16(skeleton, 0x1000);
    putString(skeleton, "[Serializer_v1.80]");

    const std::size_t blend = beginChunk(skeleton, 0x1010);
    putU16(skeleton, 0);
    endChunk(skeleton, blend);

    if (armFirst)
    {
        putBone(skeleton, "arm", 1, 0.0f, 1.0f, 0.0f);
        putBone(skeleton, "root", 0, 0.0f, 1.0f, 0.0f);
    }
    else
    {
        putBone(skeleton, "root", 0, 0.0f, 1.0f, 0.0f);
        putBone(skeleton, "arm", 1, 0.0f, 1.0f, 0.0f);
    }

    const std::size_t parent = beginChunk(skeleton, 0x3000);
    putU16(skeleton, 1);
    putU16(skeleton, 0);
    endChunk(skeleton, parent);

    const std::size_t animation = beginChunk(skeleton, 0x4000);
    putString(skeleton, "wave");
    putF32(skeleton, 1.0f);
    const std::size_t track = beginChunk(skeleton, 0x4100);
    putU16(skeleton, 1);
    const float half = 0.70710678f;
    const float times[3] = { 0.0f, 0.5f, 1.0f };
    const float rotations[3][4] = { { 0.0f, 0.0f, 0.0f, 1.0f }, { 0.0f, 0.0f, half, half },
        { 0.0f, 0.0f, 1.0f, 0.0f } };
    const float translations[3][3] = { { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.5f, 0.0f },
        { 0.0f, 0.0f, 0.0f } };
    for (int k = 0; k < 3; ++k)
    {
        const std::size_t key = beginChunk(skeleton, 0x4110);
        putF32(skeleton, times[k]);
        putQuat(skeleton, rotations[k][0], rotations[k][1], rotations[k][2], rotations[k][3]);
        putVec3(skeleton, translations[k][0], translations[k][1], translations[k][2]);
        endChunk(skeleton, key);
    }
    endChunk(skeleton, track);
    endChunk(skeleton, animation);
}

ct::String pathOf(const char* name)
{
    return ct::String(kDirectory) + "/" + name;
}

bool writeFile(const char* name, const Bytes& bytes)
{
    return file_write(pathOf(name).c_str(), bytes.data(), bytes.size());
}

void checkSkeleton(const ModelData& model, bool armFirst)
{
    CHECK(model.skeletons.size() == 1);
    if (model.skeletons.empty()) return;
    const Skeleton& skeleton = model.skeletons[0];
    CHECK(skeleton.boneCount() == 2);
    CHECK(skeleton.finalized());
    const int root = skeleton.findBone("root");
    const int arm = skeleton.findBone("arm");
    CHECK(root == (armFirst ? 1 : 0));
    CHECK(arm == (armFirst ? 0 : 1));
    if (root < 0 || arm < 0) return;
    CHECK(skeleton.bone(static_cast<std::uint32_t>(root)).parent == -1);
    CHECK(skeleton.bone(static_cast<std::uint32_t>(arm)).parent == root);
    const Bone& rootBone = skeleton.bone(static_cast<std::uint32_t>(root));
    const Bone& armBone = skeleton.bone(static_cast<std::uint32_t>(arm));
    CHECK(nearVec3(rootBone.bindLocal.GetTranslation(), 0.0f, 1.0f, 0.0f));
    CHECK(nearVec3(armBone.bindLocal.GetTranslation(), 0.0f, 1.0f, 0.0f));
    CHECK(nearVec3(rootBone.inverseBind.GetTranslation(), 0.0f, -1.0f, 0.0f));
    CHECK(nearVec3(armBone.inverseBind.GetTranslation(), 0.0f, -2.0f, 0.0f));
}

void checkAnimation(const ModelData& model, bool armFirst)
{
    const std::size_t arm = armFirst ? 0 : 1;
    const std::size_t root = armFirst ? 1 : 0;
    CHECK(model.animations.size() == 1);
    if (model.animations.empty()) return;
    const AnimationClip& clip = model.animations[0];
    CHECK(clip.name() == "wave");
    CHECK(near(clip.duration(), 1.0f));
    CHECK(clip.skeleton() == 0);
    CHECK(clip.tracks().size() == 1);
    if (clip.tracks().empty()) return;
    const BoneTrack& track = clip.tracks()[0];
    CHECK(track.bone == static_cast<int>(arm));
    CHECK(track.times.size() == 3);
    CHECK(track.positions.size() == 3);
    CHECK(track.rotations.size() == 3);
    CHECK(track.scales.size() == 3);
    if (track.times.size() != 3 || track.positions.size() != 3 || track.rotations.size() != 3)
        return;
    CHECK(near(track.times[1], 0.5f));
    CHECK(nearVec3(track.positions[1], 0.0f, 1.5f, 0.0f));
    CHECK(near(track.rotations[1].z, 0.70710678f));
    CHECK(near(track.rotations[1].w, 0.70710678f));

    ct::Vector<LocalPose> pose;
    model.skeletons[0].bindPose(pose);
    CHECK(pose.size() == 2);
    if (pose.size() != 2) return;
    clip.sample(0.5f, pose);
    CHECK(nearVec3(pose[arm].position, 0.0f, 1.5f, 0.0f));
    CHECK(near(pose[arm].rotation.z, 0.70710678f));
    CHECK(near(pose[arm].rotation.w, 0.70710678f));
    CHECK(nearVec3(pose[root].position, 0.0f, 1.0f, 0.0f));
}

void checkModel(const ModelImporter& importer, const ModelData& model, const char* nodeName,
        bool armFirst)
{
    CHECK(model.nodes.size() == 1);
    CHECK(model.roots.size() == 1);
    CHECK(model.meshes.size() == 1);
    if (model.nodes.size() != 1 || model.meshes.size() != 1) return;
    CHECK(model.nodes[0].name == nodeName);
    CHECK(model.nodes[0].mesh == 0);
    CHECK(model.nodes[0].parent == -1);
    CHECK(model.nodes[0].skeleton == 0);
    CHECK(model.meshes[0].primitives.size() == 2);
    CHECK(model.primitiveCount == 2);
    CHECK(model.materials.size() == 2);
    CHECK(model.images.empty());
    if (model.meshes[0].primitives.size() != 2 || model.materials.size() != 2) return;

    CHECK(model.materials[0].name == "Body");
    CHECK(model.materials[1].name == "Glass");

    const ModelPrimitiveData& body = model.meshes[0].primitives[0];
    const ModelPrimitiveData& glass = model.meshes[0].primitives[1];
    CHECK(body.material == 0);
    CHECK(glass.material == 1);
    CHECK(body.attributeMask == (kVertexPosition | kVertexNormal | kVertexColor | kVertexUv |
                                        kVertexJoints | kVertexWeights));
    CHECK(glass.attributeMask == kVertexPosition);
    CHECK(body.vertexCount == 4);
    CHECK(body.indexCount == 6);
    CHECK(glass.vertexCount == 3);
    CHECK(glass.indexCount == 3);
    CHECK(body.sourceBytes > 0);
    CHECK(glass.sourceBytes > 0);
    CHECK(model.geometryBytes == body.sourceBytes + glass.sourceBytes);
    CHECK(body.skinned);
    CHECK(!glass.skinned);
    CHECK(body.hasAabb);
    CHECK(nearVec3(body.aabb.min, 0.0f, 0.0f, 0.0f));
    CHECK(nearVec3(body.aabb.max, 1.0f, 1.0f, 0.5f));
    CHECK(nearVec3(glass.aabb.min, 5.0f, 0.0f, 0.0f));
    CHECK(nearVec3(glass.aabb.max, 6.0f, 2.0f, 0.0f));

    checkSkeleton(model, armFirst);
    checkAnimation(model, armFirst);

    SurfaceData surface;
    CHECK(importer.readPrimitive(model, body, surface));
    CHECK(surface.topology == prisma::Topology::Triangles);
    CHECK(surface.positions.size() == 4);
    CHECK(surface.normals.size() == 4);
    CHECK(surface.colors.size() == 4);
    CHECK(surface.uvs.size() == 4);
    CHECK(surface.uvs2.empty());
    CHECK(surface.tangents.empty());
    CHECK(surface.joints.size() == 4);
    CHECK(surface.weights.size() == 4);
    CHECK(surface.indices.size() == 6);
    if (surface.positions.size() == 4 && surface.normals.size() == 4 &&
            surface.colors.size() == 4 && surface.uvs.size() == 4 && surface.joints.size() == 4 &&
            surface.weights.size() == 4 && surface.indices.size() == 6)
    {
        for (int v = 0; v < 4; ++v)
        {
            const std::size_t i = static_cast<std::size_t>(v);
            CHECK(nearVec3(surface.positions[i], kPositions[v][0], kPositions[v][1],
                    kPositions[v][2]));
            CHECK(nearVec3(surface.normals[i], kNormals[v][0], kNormals[v][1], kNormals[v][2]));
            CHECK(near(surface.uvs[i].x, kUvs[v][0]));
            CHECK(near(surface.uvs[i].y, kUvs[v][1]));
            const std::uint32_t packed = kColors[v];
            CHECK(near(surface.colors[i].x, static_cast<float>((packed >> 16) & 0xFF) / 255.0f));
            CHECK(near(surface.colors[i].y, static_cast<float>((packed >> 8) & 0xFF) / 255.0f));
            CHECK(near(surface.colors[i].z, static_cast<float>(packed & 0xFF) / 255.0f));
            CHECK(near(surface.colors[i].w, static_cast<float>((packed >> 24) & 0xFF) / 255.0f));
            CHECK(near(surface.weights[i].x, 0.75f));
            CHECK(near(surface.weights[i].y, 0.25f));
            CHECK(near(surface.weights[i].z, 0.0f));
            CHECK(near(surface.weights[i].w, 0.0f));
            const std::uint32_t rootBone = armFirst ? 1u : 0u;
            const std::uint32_t armBone = armFirst ? 0u : 1u;
            CHECK(surface.joints[i] == (rootBone | (armBone << 8)));
        }
        CHECK(near(surface.colors[0].x, 1.0f));
        CHECK(near(surface.colors[0].y, 64.0f / 255.0f));
        CHECK(near(surface.colors[0].z, 32.0f / 255.0f));
        CHECK(near(surface.colors[0].w, 128.0f / 255.0f));
        for (int i = 0; i < 6; ++i)
            CHECK(surface.indices[static_cast<std::size_t>(i)] == kBodyIndices[i]);
    }

    SurfaceData glassSurface;
    CHECK(importer.readPrimitive(model, glass, glassSurface));
    CHECK(glassSurface.positions.size() == 3);
    CHECK(glassSurface.normals.empty());
    CHECK(glassSurface.colors.empty());
    CHECK(glassSurface.uvs.empty());
    CHECK(glassSurface.joints.empty());
    CHECK(glassSurface.weights.empty());
    CHECK(glassSurface.indices.size() == 3);
    if (glassSurface.positions.size() == 3 && glassSurface.indices.size() == 3)
    {
        CHECK(nearVec3(glassSurface.positions[2], 6.0f, 2.0f, 0.0f));
        CHECK(glassSurface.indices[0] == 0);
        CHECK(glassSurface.indices[1] == 1);
        CHECK(glassSurface.indices[2] == 2);
    }

    ModelPrimitiveData wrong = body;
    wrong.source = 99;
    CHECK(!importer.readPrimitive(model, wrong, surface));

    ModelImageData image;
    ct::Vector<std::uint8_t> imageBytes;
    CHECK(!importer.readImage(model, image, imageBytes));
}

void checkStripAndFan()
{
    Bytes mesh;
    putU16(mesh, 0x1000);
    putString(mesh, "[MeshSerializer_v1.100]");
    const std::size_t root = beginChunk(mesh, 0x3000);
    putU8(mesh, 0);

    for (int kind = 0; kind < 2; ++kind)
    {
        const std::size_t submesh = beginChunk(mesh, 0x4000);
        putString(mesh, kind == 0 ? "Strip" : "Fan");
        putU8(mesh, 0);
        putU32(mesh, 5);
        putU8(mesh, 0);
        for (std::uint16_t i = 0; i < 5; ++i) putU16(mesh, i);
        const std::size_t geometry = beginChunk(mesh, 0x5000);
        putU32(mesh, 5);
        const std::size_t declaration = beginChunk(mesh, 0x5100);
        putElement(mesh, 0, 2, 1, 0, 0);
        endChunk(mesh, declaration);
        const std::size_t buffer = beginBuffer(mesh, 0, 12);
        const std::size_t data = beginChunk(mesh, 0x5210);
        for (int v = 0; v < 5; ++v) putVec3(mesh, static_cast<float>(v), 0.0f, 0.0f);
        endChunk(mesh, data);
        endChunk(mesh, buffer);
        endChunk(mesh, geometry);
        const std::size_t operation = beginChunk(mesh, 0x4010);
        putU16(mesh, kind == 0 ? 5 : 6);
        endChunk(mesh, operation);
        endChunk(mesh, submesh);
    }
    endChunk(mesh, root);

    OgreMeshImporter importer;
    ModelData model;
    CHECK(importer.importMemory(mesh.data(), mesh.size(), pathOf("strips.mesh"), model));
    CHECK(model.skeletons.empty());
    CHECK(model.animations.empty());
    CHECK(model.nodes.size() == 1 && model.nodes[0].skeleton == -1);
    CHECK(model.meshes.size() == 1 && model.meshes[0].primitives.size() == 2);
    if (model.meshes.size() != 1 || model.meshes[0].primitives.size() != 2) return;
    CHECK(!model.meshes[0].primitives[0].skinned);

    SurfaceData strip;
    CHECK(importer.readPrimitive(model, model.meshes[0].primitives[0], strip));
    const std::uint32_t expectedStrip[9] = { 0, 1, 2, 2, 1, 3, 2, 3, 4 };
    CHECK(strip.indices.size() == 9);
    if (strip.indices.size() == 9)
    {
        for (std::size_t i = 0; i < 9; ++i) CHECK(strip.indices[i] == expectedStrip[i]);
    }

    SurfaceData fan;
    CHECK(importer.readPrimitive(model, model.meshes[0].primitives[1], fan));
    const std::uint32_t expectedFan[9] = { 0, 1, 2, 0, 2, 3, 0, 3, 4 };
    CHECK(fan.indices.size() == 9);
    if (fan.indices.size() == 9)
    {
        for (std::size_t i = 0; i < 9; ++i) CHECK(fan.indices[i] == expectedFan[i]);
    }
}

} // namespace

int main()
{
    CHECK(fs_create_directory(kDirectory));

    Bytes mesh;
    Bytes skeleton;
    Bytes swapped;
    buildMesh(mesh, "[MeshSerializer_v1.100]", "rig", -1);
    buildSkeleton(skeleton, false);
    buildSkeleton(swapped, true);

    CHECK(writeFile("body.mesh", mesh));
    CHECK(writeFile("rig.skeleton", skeleton));
    CHECK(writeFile("rig2.skeleton", swapped));

    OgreMeshImporter importer;
    CHECK(strcmp(importer.name(), "Ogre mesh") == 0);
    CHECK(importer.supports("mesh"));
    CHECK(!importer.supports("glb"));
    CHECK(!importer.supports("skeleton"));
    CHECK(!importer.supports(nullptr));

    ModelData model;
    CHECK(importer.import(pathOf("body.mesh"), model));
    CHECK(model.error.empty());
    CHECK(model.path == pathOf("body.mesh"));
    CHECK(model.directory == pathOf(""));
    checkModel(importer, model, "body", false);

    ModelData memoryModel;
    CHECK(importer.importMemory(mesh.data(), mesh.size(), pathOf("memory.mesh"), memoryModel));
    checkModel(importer, memoryModel, "memory", false);

    Bytes remapped;
    buildMesh(remapped, "[MeshSerializer_v1.41]", "rig2.skeleton", -1);
    CHECK(writeFile("remap.mesh", remapped));
    ModelData remapModel;
    CHECK(importer.import(pathOf("remap.mesh"), remapModel));
    checkModel(importer, remapModel, "remap", true);

    const char* const olderVersions[2] = { "[MeshSerializer_v1.8]", "[MeshSerializer_v1.40]" };
    for (int v = 0; v < 2; ++v)
    {
        Bytes older;
        buildMesh(older, olderVersions[v], "rig", -1);
        ModelData olderModel;
        CHECK(importer.importMemory(older.data(), older.size(), pathOf("older.mesh"), olderModel));
        checkModel(importer, olderModel, "older", false);
    }

    gLegacyQuirks = true;
    for (int v = 0; v < 2; ++v)
    {
        Bytes quirky;
        buildMesh(quirky, olderVersions[v], "rig", -1);
        ModelData quirkyModel;
        CHECK(importer.importMemory(quirky.data(), quirky.size(), pathOf("quirky.mesh"),
                quirkyModel));
        checkModel(importer, quirkyModel, "quirky", false);
    }
    Bytes strict;
    buildMesh(strict, "[MeshSerializer_v1.100]", "rig", -1);
    ModelData strictModel;
    CHECK(!importer.importMemory(strict.data(), strict.size(), pathOf("strict.mesh"), strictModel));
    CHECK(!strictModel.error.empty());
    gLegacyQuirks = false;

    ModelLoader loader;
    loader.addImporter(new OgreMeshImporter());
    CHECK(loader.importerFor(pathOf("body.mesh")) != nullptr);
    ModelData loaded;
    CHECK(loader.load(pathOf("body.mesh"), loaded));
    checkModel(*loader.importerFor(pathOf("body.mesh")), loaded, "body", false);
    ModelData loadedMemory;
    CHECK(loader.loadFromMemory(mesh.data(), mesh.size(), pathOf("loader.MESH"), loadedMemory));
    checkModel(*loader.importer(0), loadedMemory, "loader", false);

    ModelData unknown;
    CHECK(!loader.load(pathOf("body.unknown"), unknown));
    CHECK(!unknown.error.empty());

    Bytes badVersion;
    buildMesh(badVersion, "[MeshSerializer_v9.9]", "rig", -1);
    ModelData failed;
    CHECK(!importer.importMemory(badVersion.data(), badVersion.size(), pathOf("version.mesh"),
            failed));
    CHECK(!failed.error.empty());

    Bytes notMesh;
    putU16(notMesh, 0x2222);
    putString(notMesh, "[MeshSerializer_v1.100]");
    CHECK(!importer.importMemory(notMesh.data(), notMesh.size(), pathOf("notmesh.mesh"), failed));
    CHECK(!failed.error.empty());

    Bytes truncated;
    for (std::size_t i = 0; i < mesh.size() / 2; ++i) truncated.push_back(mesh[i]);
    CHECK(!importer.importMemory(truncated.data(), truncated.size(), pathOf("short.mesh"), failed));
    CHECK(!failed.error.empty());
    CHECK(writeFile("short.mesh", truncated));
    CHECK(!importer.import(pathOf("short.mesh"), failed));

    for (std::size_t length = 1; length < mesh.size(); length += 7)
    {
        ModelData partial;
        CHECK(!importer.importMemory(mesh.data(), length, pathOf("partial.mesh"), partial));
    }

    Bytes missing;
    buildMesh(missing, "[MeshSerializer_v1.100]", "nothere", -1);
    CHECK(!importer.importMemory(missing.data(), missing.size(), pathOf("missing.mesh"), failed));
    CHECK(!failed.error.empty());

    Bytes badHandle;
    buildMesh(badHandle, "[MeshSerializer_v1.100]", "rig", 7);
    CHECK(!importer.importMemory(badHandle.data(), badHandle.size(), pathOf("handle.mesh"),
            failed));
    CHECK(!failed.error.empty());

    CHECK(!importer.import(pathOf("absent.mesh"), failed));
    CHECK(!failed.error.empty());
    CHECK(!importer.importMemory(nullptr, 0, pathOf("empty.mesh"), failed));

    Bytes brokenSkeleton = skeleton;
    brokenSkeleton.resize(brokenSkeleton.size() - 5);
    CHECK(writeFile("broken.skeleton", brokenSkeleton));
    Bytes brokenMesh;
    buildMesh(brokenMesh, "[MeshSerializer_v1.100]", "broken", -1);
    CHECK(!importer.importMemory(brokenMesh.data(), brokenMesh.size(), pathOf("broken.mesh"),
            failed));
    CHECK(!failed.error.empty());

    checkStripAndFan();

    if (failures == 0) printf("ogre: ok\n");
    return failures == 0 ? 0 : 1;
}
