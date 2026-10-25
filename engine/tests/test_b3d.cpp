#include "Check.h"

#include "prisma3d/import/B3DImporter.h"

#include "platform.h"

#include <math.h>
#include <string.h>

using namespace prisma3d;

namespace
{

const float kTolerance = 1e-4f;
const char* const kDirectory = PRISMA3D_TEST_DIR "/b3d";

typedef ct::Vector<std::uint8_t> Bytes;

bool near(float a, float b)
{
    return fabsf(a - b) <= kTolerance;
}

bool nearVec3(const Math::Vec3& a, float x, float y, float z)
{
    return near(a.x, x) && near(a.y, y) && near(a.z, z);
}

bool nearQuat(const Math::Quaternion& a, float x, float y, float z, float w)
{
    return near(a.x, x) && near(a.y, y) && near(a.z, z) && near(a.w, w);
}

bool nearIdentity(const Math::Mat4& m)
{
    const Math::Mat4 identity = Math::Mat4::Identity();
    for (int c = 0; c < 4; ++c)
    {
        for (int r = 0; r < 4; ++r)
        {
            if (!near(m[c][r], identity[c][r])) return false;
        }
    }
    return true;
}

void putS32(Bytes& bytes, std::int32_t value)
{
    const std::uint32_t raw = static_cast<std::uint32_t>(value);
    for (int i = 0; i < 4; ++i) bytes.push_back(static_cast<std::uint8_t>(raw >> (8 * i)));
}

void putF32(Bytes& bytes, float value)
{
    std::uint32_t raw;
    memcpy(&raw, &value, sizeof(raw));
    putS32(bytes, static_cast<std::int32_t>(raw));
}

void putVec3(Bytes& bytes, float x, float y, float z)
{
    putF32(bytes, x);
    putF32(bytes, y);
    putF32(bytes, z);
}

void putQuat(Bytes& bytes, float w, float x, float y, float z)
{
    putF32(bytes, w);
    putF32(bytes, x);
    putF32(bytes, y);
    putF32(bytes, z);
}

void putCString(Bytes& bytes, const char* text)
{
    for (const char* c = text; *c; ++c) bytes.push_back(static_cast<std::uint8_t>(*c));
    bytes.push_back(0);
}

std::size_t beginChunk(Bytes& bytes, const char* tag)
{
    const std::size_t start = bytes.size();
    for (int i = 0; i < 4; ++i) bytes.push_back(static_cast<std::uint8_t>(tag[i]));
    putS32(bytes, 0);
    return start;
}

void endChunk(Bytes& bytes, std::size_t start)
{
    const std::uint32_t length = static_cast<std::uint32_t>(bytes.size() - start - 8);
    for (int i = 0; i < 4; ++i)
        bytes[start + 4 + static_cast<std::size_t>(i)] =
                static_cast<std::uint8_t>(length >> (8 * i));
}

struct Options
{
    bool bones = true;
    bool badIndex = false;
    bool unorderedKeys = false;
};

const float kHalfSqrt2 = 0.70710678f;
const float kAngles[3] = { 0.0f, 0.3f, 0.6f };

void buildB3D(Bytes& bytes, const Options& options)
{
    const std::size_t root = beginChunk(bytes, "BB3D");
    putS32(bytes, 1);

    const std::size_t textures = beginChunk(bytes, "TEXS");
    putCString(bytes, "skin.png");
    putS32(bytes, 1);
    putS32(bytes, 2);
    for (int i = 0; i < 5; ++i) putF32(bytes, 0.0f);
    endChunk(bytes, textures);

    const std::size_t brushes = beginChunk(bytes, "BRUS");
    putS32(bytes, 1);
    putCString(bytes, "red");
    putVec3(bytes, 1.0f, 0.0f, 0.0f);
    putF32(bytes, 1.0f);
    putF32(bytes, 0.0f);
    putS32(bytes, 1);
    putS32(bytes, 0);
    putS32(bytes, -1);
    putCString(bytes, "skinned");
    putVec3(bytes, 0.5f, 0.25f, 0.75f);
    putF32(bytes, 1.0f);
    putF32(bytes, 0.0f);
    putS32(bytes, 1);
    putS32(bytes, 0);
    putS32(bytes, 0);
    endChunk(bytes, brushes);

    const std::size_t node = beginChunk(bytes, "NODE");
    putCString(bytes, "root");
    putVec3(bytes, 1.0f, 2.0f, 3.0f);
    putVec3(bytes, 2.0f, 2.0f, 2.0f);
    putQuat(bytes, kHalfSqrt2, 0.0f, kHalfSqrt2, 0.0f);

    const std::size_t mesh = beginChunk(bytes, "MESH");
    putS32(bytes, -1);
    const std::size_t vertices = beginChunk(bytes, "VRTS");
    putS32(bytes, 3);
    putS32(bytes, 1);
    putS32(bytes, 2);
    const float positions[4][3] = { { 0, 0, 0 }, { 1, 0, 0 }, { 1, 1, 0 }, { 0, 1, 0 } };
    for (int i = 0; i < 4; ++i)
    {
        putVec3(bytes, positions[i][0], positions[i][1], positions[i][2]);
        putVec3(bytes, 0.0f, 0.0f, 1.0f);
        putF32(bytes, 0.1f * static_cast<float>(i));
        putF32(bytes, 0.5f);
        putF32(bytes, 0.25f);
        putF32(bytes, 1.0f);
        putF32(bytes, 0.1f * static_cast<float>(i));
        putF32(bytes, 0.2f * static_cast<float>(i));
    }
    endChunk(bytes, vertices);
    const std::size_t first = beginChunk(bytes, "TRIS");
    putS32(bytes, 0);
    putS32(bytes, 0);
    putS32(bytes, 1);
    putS32(bytes, 2);
    endChunk(bytes, first);
    const std::size_t second = beginChunk(bytes, "TRIS");
    putS32(bytes, 1);
    putS32(bytes, 0);
    putS32(bytes, 2);
    putS32(bytes, options.badIndex ? 9 : 3);
    endChunk(bytes, second);
    endChunk(bytes, mesh);

    if (options.bones)
    {
        const std::size_t rootBone = beginChunk(bytes, "BONE");
        putS32(bytes, 2);
        putF32(bytes, 0.25f);
        endChunk(bytes, rootBone);

        const std::size_t anim = beginChunk(bytes, "ANIM");
        putS32(bytes, 0);
        putS32(bytes, 10);
        putF32(bytes, 20.0f);
        endChunk(bytes, anim);
    }

    const std::size_t arm = beginChunk(bytes, "NODE");
    putCString(bytes, "arm");
    putVec3(bytes, 0.0f, 1.0f, 0.0f);
    putVec3(bytes, 1.0f, 1.0f, 1.0f);
    putQuat(bytes, 1.0f, 0.0f, 0.0f, 0.0f);
    if (options.bones)
    {
        const std::size_t armBone = beginChunk(bytes, "BONE");
        putS32(bytes, 1);
        putF32(bytes, 1.0f);
        putS32(bytes, 2);
        putF32(bytes, 0.5f);
        endChunk(bytes, armBone);
        const std::size_t keys = beginChunk(bytes, "KEYS");
        putS32(bytes, 5);
        const int frames[3] = { 0, 5, 9 };
        for (int i = 0; i < 3; ++i)
        {
            const float fi = static_cast<float>(i);
            putS32(bytes, options.unorderedKeys && i == 2 ? 2 : frames[i]);
            putVec3(bytes, fi, 1.0f + fi, 0.0f);
            putQuat(bytes, cosf(kAngles[i]), 0.0f, 0.0f, sinf(kAngles[i]));
        }
        endChunk(bytes, keys);
    }
    endChunk(bytes, arm);

    endChunk(bytes, node);
    endChunk(bytes, root);
}

ct::String pathOf(const char* name)
{
    return ct::String(kDirectory) + "/" + name;
}

bool writeFile(const char* name, const Bytes& bytes)
{
    return file_write(pathOf(name).c_str(), bytes.data(), bytes.size());
}

void checkPrimitive(const ModelImporter& importer, const ModelData& model,
        std::size_t primitiveIndex, bool bones)
{
    CHECK(model.meshes.size() == 1);
    if (model.meshes.empty() || primitiveIndex >= model.meshes[0].primitives.size()) return;
    const ModelPrimitiveData& primitive = model.meshes[0].primitives[primitiveIndex];
    std::uint32_t mask = kVertexPosition | kVertexNormal | kVertexColor | kVertexUv;
    if (bones) mask |= kVertexJoints | kVertexWeights;
    CHECK(primitive.attributeMask == mask);
    CHECK(primitive.vertexCount == 3);
    CHECK(primitive.indexCount == 3);
    CHECK(primitive.skinned == bones);
    CHECK(primitive.hasAabb);
    CHECK(primitive.material == static_cast<int>(primitiveIndex));
    CHECK(primitive.sourceBytes > 0);

    SurfaceData surface;
    CHECK(importer.readPrimitive(model, primitive, surface));
    CHECK(surface.topology == prisma::Topology::Triangles);
    CHECK(surface.positions.size() == 3);
    CHECK(surface.normals.size() == 3);
    CHECK(surface.colors.size() == 3);
    CHECK(surface.uvs.size() == 3);
    CHECK(surface.uvs2.empty());
    CHECK(surface.indices.size() == 3);
    if (surface.positions.size() != 3 || surface.normals.size() != 3 ||
            surface.colors.size() != 3 || surface.uvs.size() != 3 || surface.indices.size() != 3)
        return;

    const int source[2][3] = { { 0, 1, 2 }, { 0, 2, 3 } };
    const float positions[4][2] = { { 0, 0 }, { 1, 0 }, { 1, 1 }, { 0, 1 } };
    for (int i = 0; i < 3; ++i)
    {
        const int vertex = source[primitiveIndex][i];
        CHECK(surface.indices[static_cast<std::size_t>(i)] == static_cast<std::uint32_t>(i));
        CHECK(nearVec3(surface.positions[static_cast<std::size_t>(i)], positions[vertex][0],
                positions[vertex][1], 0.0f));
        CHECK(nearVec3(surface.normals[static_cast<std::size_t>(i)], 0.0f, 0.0f, 1.0f));
        const Math::Vec4 color = surface.colors[static_cast<std::size_t>(i)];
        CHECK(near(color.x, 0.1f * static_cast<float>(vertex)) && near(color.y, 0.5f) &&
                near(color.z, 0.25f) && near(color.w, 1.0f));
        const Math::Vec2 uv = surface.uvs[static_cast<std::size_t>(i)];
        CHECK(near(uv.x, 0.1f * static_cast<float>(vertex)) &&
                near(uv.y, 0.2f * static_cast<float>(vertex)));
    }

    if (!bones)
    {
        CHECK(surface.joints.empty());
        CHECK(surface.weights.empty());
        return;
    }
    CHECK(surface.joints.size() == 3);
    CHECK(surface.weights.size() == 3);
    if (surface.joints.size() != 3 || surface.weights.size() != 3) return;
    for (int i = 0; i < 3; ++i)
    {
        const int vertex = source[primitiveIndex][i];
        const std::size_t k = static_cast<std::size_t>(i);
        const Math::Vec4 weights = surface.weights[k];
        if (vertex == 1)
        {
            CHECK(surface.joints[k] == 0x01010101u);
            CHECK(near(weights.x, 1.0f) && near(weights.y, 0.0f) && near(weights.z, 0.0f) &&
                    near(weights.w, 0.0f));
        }
        else if (vertex == 2)
        {
            CHECK(surface.joints[k] == 0x01010001u);
            CHECK(near(weights.x, 2.0f / 3.0f) && near(weights.y, 1.0f / 3.0f) &&
                    near(weights.z, 0.0f) && near(weights.w, 0.0f));
        }
        else
        {
            CHECK(surface.joints[k] == 0u);
            CHECK(near(weights.x, 1.0f) && near(weights.y, 0.0f));
        }
    }
}

void checkClip(const ModelData& model, const char* stem)
{
    CHECK(model.animations.size() == 1);
    if (model.animations.empty()) return;
    const AnimationClip& clip = model.animations[0];
    CHECK(clip.name() == stem);
    CHECK(clip.skeleton() == 0);
    CHECK(near(clip.duration(), 9.0f / 20.0f));
    CHECK(clip.tracks().size() == 1);
    if (clip.tracks().size() != 1) return;
    const BoneTrack& track = clip.tracks()[0];
    CHECK(track.bone == 1);
    CHECK(track.times.size() == 3);
    CHECK(track.positions.size() == 3);
    CHECK(track.rotations.size() == 3);
    CHECK(track.scales.size() == 3);
    if (track.times.size() != 3 || track.positions.size() != 3 || track.rotations.size() != 3 ||
            track.scales.size() != 3)
        return;
    const int frames[3] = { 0, 5, 9 };
    for (std::size_t i = 0; i < 3; ++i)
    {
        CHECK(near(track.times[i], static_cast<float>(frames[i]) / 20.0f));
        CHECK(nearVec3(track.positions[i], static_cast<float>(i), 1.0f + static_cast<float>(i),
                0.0f));
        CHECK(nearQuat(track.rotations[i], 0.0f, 0.0f, sinf(kAngles[i]), -cosf(kAngles[i])));
        CHECK(nearVec3(track.scales[i], 1.0f, 1.0f, 1.0f));
    }

    ct::Vector<LocalPose> pose;
    model.skeletons[0].bindPose(pose);
    CHECK(pose.size() == 2);
    clip.sample(5.0f / 20.0f, pose);
    CHECK(nearVec3(pose[1].position, 1.0f, 2.0f, 0.0f));
    CHECK(nearQuat(pose[1].rotation, 0.0f, 0.0f, sinf(0.3f), -cosf(0.3f)));
    CHECK(nearVec3(pose[0].position, 1.0f, 2.0f, 3.0f));
    clip.sample(0.0f, pose);
    CHECK(nearVec3(pose[1].position, 0.0f, 1.0f, 0.0f));
}

void checkSkeleton(const ModelData& model)
{
    CHECK(model.skeletons.size() == 1);
    if (model.skeletons.empty()) return;
    const Skeleton& skeleton = model.skeletons[0];
    CHECK(skeleton.boneCount() == 2);
    CHECK(skeleton.finalized());
    CHECK(skeleton.findBone("root") == 0);
    CHECK(skeleton.findBone("arm") == 1);
    if (skeleton.boneCount() != 2) return;
    CHECK(skeleton.bone(0).parent == -1);
    CHECK(skeleton.bone(1).parent == 0);
    CHECK(nearVec3(skeleton.bone(0).bindLocal.GetTranslation(), 1.0f, 2.0f, 3.0f));
    CHECK(nearVec3(skeleton.bone(1).bindLocal.GetTranslation(), 0.0f, 1.0f, 0.0f));
    CHECK(nearIdentity(model.nodeGlobalTransform(0) * skeleton.bone(0).inverseBind));
    CHECK(nearIdentity(model.nodeGlobalTransform(1) * skeleton.bone(1).inverseBind));
}

void checkModel(const ModelImporter& importer, const ModelData& model, const char* stem, bool bones)
{
    CHECK(model.error.empty());
    CHECK(model.nodes.size() == 2);
    CHECK(model.roots.size() == 1);
    if (model.nodes.size() == 2 && model.roots.size() == 1)
    {
        CHECK(model.roots[0] == 0);
        const ModelNodeData& root = model.nodes[0];
        const ModelNodeData& arm = model.nodes[1];
        CHECK(root.name == "root");
        CHECK(root.parent == -1);
        CHECK(root.children.size() == 1 && root.children[0] == 1);
        CHECK(nearVec3(root.translation, 1.0f, 2.0f, 3.0f));
        CHECK(nearVec3(root.scale, 2.0f, 2.0f, 2.0f));
        CHECK(nearQuat(root.rotation, 0.0f, kHalfSqrt2, 0.0f, -kHalfSqrt2));
        CHECK(root.mesh == 0);
        CHECK(root.skeleton == (bones ? 0 : -1));
        CHECK(arm.name == "arm");
        CHECK(arm.parent == 0);
        CHECK(arm.children.empty());
        CHECK(nearVec3(arm.translation, 0.0f, 1.0f, 0.0f));
        CHECK(nearVec3(arm.scale, 1.0f, 1.0f, 1.0f));
        CHECK(nearQuat(arm.rotation, 0.0f, 0.0f, 0.0f, -1.0f));
        CHECK(arm.mesh == -1);
        CHECK(arm.skeleton == -1);
    }

    CHECK(model.meshes.size() == 1);
    if (!model.meshes.empty())
    {
        CHECK(model.meshes[0].name == "root");
        CHECK(model.meshes[0].primitives.size() == 2);
    }
    CHECK(model.primitiveCount == 2);
    CHECK(model.geometryBytes > 0);

    CHECK(model.materials.size() == 2);
    CHECK(model.textures.size() == 1);
    CHECK(model.images.size() == 1);
    if (model.materials.size() == 2)
    {
        CHECK(model.materials[0].name == "red");
        CHECK(near(model.materials[0].baseColor.x, 1.0f) &&
                near(model.materials[0].baseColor.y, 0.0f) &&
                near(model.materials[0].baseColor.w, 1.0f));
        CHECK(model.materials[0].baseColorTexture == -1);
        CHECK(model.materials[1].name == "skinned");
        CHECK(near(model.materials[1].baseColor.x, 0.5f) &&
                near(model.materials[1].baseColor.y, 0.25f) &&
                near(model.materials[1].baseColor.z, 0.75f));
        CHECK(model.materials[1].baseColorTexture == 0);
    }
    if (model.textures.size() == 1) CHECK(model.textures[0].image == 0);
    if (model.images.size() == 1)
    {
        CHECK(model.images[0].path == model.directory + "skin.png");
        CHECK(!model.images[0].embedded());
        ct::Vector<std::uint8_t> pixels;
        CHECK(!importer.readImage(model, model.images[0], pixels));
        CHECK(pixels.empty());
    }

    checkPrimitive(importer, model, 0, bones);
    checkPrimitive(importer, model, 1, bones);

    SurfaceData surface;
    ModelPrimitiveData missing;
    missing.source = 99;
    CHECK(!importer.readPrimitive(model, missing, surface));

    if (bones)
    {
        checkSkeleton(model);
        checkClip(model, stem);
    }
    else
    {
        CHECK(model.skeletons.empty());
        CHECK(model.animations.empty());
    }
}

void checkRejected(ModelImporter& importer, const Bytes& bytes, const char* name)
{
    ModelData failed;
    CHECK(!importer.importMemory(bytes.data(), bytes.size(), pathOf(name), failed));
    CHECK(!failed.error.empty());
}

void putTriangleMesh(Bytes& bytes, float height)
{
    const std::size_t mesh = beginChunk(bytes, "MESH");
    putS32(bytes, -1);
    const std::size_t vertices = beginChunk(bytes, "VRTS");
    putS32(bytes, 0);
    putS32(bytes, 0);
    putS32(bytes, 0);
    putVec3(bytes, 0.0f, height, 0.0f);
    putVec3(bytes, 1.0f, height, 0.0f);
    putVec3(bytes, 0.0f, height, 1.0f);
    endChunk(bytes, vertices);
    const std::size_t triangles = beginChunk(bytes, "TRIS");
    putS32(bytes, -1);
    putS32(bytes, 0);
    putS32(bytes, 1);
    putS32(bytes, 2);
    endChunk(bytes, triangles);
    endChunk(bytes, mesh);
}

void buildSeveralMeshes(Bytes& bytes)
{
    const std::size_t root = beginChunk(bytes, "BB3D");
    putS32(bytes, 1);
    const std::size_t node = beginChunk(bytes, "NODE");
    putCString(bytes, "level");
    putVec3(bytes, 1.0f, 2.0f, 3.0f);
    putVec3(bytes, 1.0f, 1.0f, 1.0f);
    putQuat(bytes, 1.0f, 0.0f, 0.0f, 0.0f);
    putTriangleMesh(bytes, 0.0f);
    putTriangleMesh(bytes, 5.0f);
    putTriangleMesh(bytes, 9.0f);
    endChunk(bytes, node);
    endChunk(bytes, root);
}

void checkSeveralMeshes(ModelImporter& importer)
{
    Bytes bytes;
    buildSeveralMeshes(bytes);
    ModelData model;
    CHECK(importer.importMemory(bytes.data(), bytes.size(), pathOf("several.b3d"), model));
    CHECK(model.error.empty());
    CHECK(model.nodes.size() == 3);
    CHECK(model.meshes.size() == 3);
    CHECK(model.roots.size() == 1);
    if (model.nodes.size() != 3 || model.meshes.size() != 3) return;
    CHECK(model.nodes[0].name == "level");
    CHECK(model.nodes[1].name == "level_1");
    CHECK(model.nodes[2].name == "level_2");
    CHECK(model.nodes[0].children.size() == 2);
    for (int i = 0; i < 3; ++i) CHECK(model.nodes[static_cast<std::size_t>(i)].mesh == i);
    for (int i = 1; i < 3; ++i)
    {
        const ModelNodeData& extra = model.nodes[static_cast<std::size_t>(i)];
        CHECK(extra.parent == 0);
        CHECK(nearVec3(extra.translation, 0.0f, 0.0f, 0.0f));
        CHECK(nearVec3(extra.scale, 1.0f, 1.0f, 1.0f));
        CHECK(nearQuat(extra.rotation, 0.0f, 0.0f, 0.0f, 1.0f));
        CHECK(nearVec3(model.nodeGlobalTransform(i).GetTranslation(), 1.0f, 2.0f, 3.0f));
    }
    for (int i = 0; i < 3; ++i)
    {
        const ModelMeshData& mesh = model.meshes[static_cast<std::size_t>(i)];
        CHECK(mesh.primitives.size() == 1);
        if (mesh.primitives.size() != 1) continue;
        SurfaceData surface;
        CHECK(importer.readPrimitive(model, mesh.primitives[0], surface));
        CHECK(surface.positions.size() == 3);
        if (surface.positions.size() == 3)
            CHECK(near(surface.positions[0].y, i == 0 ? 0.0f : (i == 1 ? 5.0f : 9.0f)));
    }
}

} // namespace

int main()
{
    CHECK(fs_create_directory(kDirectory));

    Bytes full;
    buildB3D(full, Options());
    Options staticOptions;
    staticOptions.bones = false;
    Bytes staticFile;
    buildB3D(staticFile, staticOptions);

    CHECK(writeFile("hero.b3d", full));
    CHECK(writeFile("block.b3d", staticFile));

    B3DImporter importer;
    CHECK(strcmp(importer.name(), "Blitz3D") == 0);
    CHECK(importer.supports("b3d"));
    CHECK(!importer.supports("ms3d"));
    CHECK(!importer.supports(nullptr));

    ModelData model;
    CHECK(importer.import(pathOf("hero.b3d"), model));
    CHECK(model.path == pathOf("hero.b3d"));
    CHECK(model.directory == pathOf(""));
    checkModel(importer, model, "hero", true);

    ModelData staticModel;
    CHECK(importer.import(pathOf("block.b3d"), staticModel));
    checkModel(importer, staticModel, "block", false);

    ModelData memoryModel;
    CHECK(importer.importMemory(full.data(), full.size(), pathOf("memory.b3d"), memoryModel));
    checkModel(importer, memoryModel, "memory", true);

    ModelLoader loader;
    loader.addImporter(new B3DImporter());
    CHECK(loader.importerFor(pathOf("hero.b3d")) != nullptr);
    ModelData loaded;
    CHECK(loader.load(pathOf("hero.b3d"), loaded));
    checkModel(*loader.importerFor(pathOf("hero.b3d")), loaded, "hero", true);
    ModelData loadedMemory;
    CHECK(loader.loadFromMemory(full.data(), full.size(), pathOf("loader.B3D"), loadedMemory));
    checkModel(*loader.importer(0), loadedMemory, "loader", true);

    ModelData unknown;
    CHECK(!loader.load(pathOf("hero.unknown"), unknown));
    CHECK(!unknown.error.empty());

    Bytes badHeader = full;
    badHeader[0] = 'X';
    checkRejected(importer, badHeader, "badheader.b3d");

    Bytes shortLength = full;
    shortLength[4] = 3;
    shortLength[5] = 0;
    shortLength[6] = 0;
    shortLength[7] = 0;
    checkRejected(importer, shortLength, "shortlength.b3d");

    Bytes hugeLength = full;
    hugeLength[7] = 0x7F;
    checkRejected(importer, hugeLength, "hugelength.b3d");

    for (std::size_t length = 1; length < full.size(); ++length)
    {
        ModelData partial;
        CHECK(!importer.importMemory(full.data(), length, pathOf("partial.b3d"), partial));
        CHECK(!partial.error.empty());
    }

    Bytes truncated;
    for (std::size_t i = 0; i < full.size() / 2; ++i) truncated.push_back(full[i]);
    CHECK(writeFile("truncated.b3d", truncated));
    ModelData failed;
    CHECK(!importer.import(pathOf("truncated.b3d"), failed));
    CHECK(!failed.error.empty());

    Options badIndex;
    badIndex.badIndex = true;
    Bytes badIndexFile;
    buildB3D(badIndexFile, badIndex);
    checkRejected(importer, badIndexFile, "badindex.b3d");

    Options unordered;
    unordered.unorderedKeys = true;
    Bytes unorderedFile;
    buildB3D(unorderedFile, unordered);
    checkRejected(importer, unorderedFile, "unordered.b3d");

    CHECK(!importer.import(pathOf("absent.b3d"), failed));
    CHECK(!failed.error.empty());
    CHECK(!importer.importMemory(nullptr, 0, pathOf("empty.b3d"), failed));
    CHECK(!failed.error.empty());

    Bytes noNodes;
    {
        const std::size_t root = beginChunk(noNodes, "BB3D");
        putS32(noNodes, 1);
        endChunk(noNodes, root);
    }
    checkRejected(importer, noNodes, "nonodes.b3d");

    checkSeveralMeshes(importer);

    if (failures == 0) printf("b3d: ok\n");
    return failures == 0 ? 0 : 1;
}
