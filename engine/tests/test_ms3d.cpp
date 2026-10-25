#include "Check.h"

#include "prisma3d/import/MS3DImporter.h"

#include "platform.h"

#include <math.h>
#include <string.h>

using namespace prisma3d;

namespace
{

const float kTolerance = 1e-4f;
const char* const kDirectory = PRISMA3D_TEST_DIR "/ms3d";

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

void putU8(Bytes& bytes, std::uint8_t value)
{
    bytes.push_back(value);
}

void putU16(Bytes& bytes, std::uint16_t value)
{
    bytes.push_back(static_cast<std::uint8_t>(value));
    bytes.push_back(static_cast<std::uint8_t>(value >> 8));
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

void putFixed(Bytes& bytes, const char* text, std::size_t length)
{
    const std::size_t used = strlen(text);
    for (std::size_t i = 0; i < length; ++i)
        bytes.push_back(i < used ? static_cast<std::uint8_t>(text[i]) : 0);
}

void putKey(Bytes& bytes, float time, float x, float y, float z)
{
    putF32(bytes, time);
    putVec3(bytes, x, y, z);
}

struct Options
{
    std::int32_t version = 4;
    bool joints = true;
    bool badTriangle = false;
    bool badMaterial = false;
    bool badBone = false;
    bool cycle = false;
    bool unorderedKeys = false;
};

void buildMS3D(Bytes& bytes, const Options& options, std::size_t& jointsOffset)
{
    for (int i = 0; i < 10; ++i) bytes.push_back(static_cast<std::uint8_t>("MS3D000000"[i]));
    putS32(bytes, options.version);

    putU16(bytes, 3);
    const float positions[3][3] = { { 0, 0, 0 }, { 1, 0, 0 }, { 0, 1, 0 } };
    const int bones[3] = { 0, options.badBone ? 5 : 1, -1 };
    for (int i = 0; i < 3; ++i)
    {
        putU8(bytes, 0);
        putVec3(bytes, positions[i][0], positions[i][1], positions[i][2]);
        putU8(bytes, static_cast<std::uint8_t>(bones[i]));
        putU8(bytes, 0);
    }

    putU16(bytes, 1);
    putU16(bytes, 0);
    putU16(bytes, 0);
    putU16(bytes, 1);
    putU16(bytes, options.badTriangle ? 7 : 2);
    putVec3(bytes, 0.0f, 0.0f, 1.0f);
    putVec3(bytes, 0.0f, 1.0f, 0.0f);
    putVec3(bytes, 1.0f, 0.0f, 0.0f);
    putF32(bytes, 0.1f);
    putF32(bytes, 0.5f);
    putF32(bytes, 0.9f);
    putF32(bytes, 0.2f);
    putF32(bytes, 0.4f);
    putF32(bytes, 0.6f);
    putU8(bytes, 1);
    putU8(bytes, 0);

    putU16(bytes, 1);
    putU8(bytes, 0);
    putFixed(bytes, "body", 32);
    putU16(bytes, 1);
    putU16(bytes, 0);
    putU8(bytes, static_cast<std::uint8_t>(options.badMaterial ? 3 : 0));

    putU16(bytes, 1);
    putFixed(bytes, "skin", 32);
    for (int i = 0; i < 4; ++i) putF32(bytes, 0.1f);
    putF32(bytes, 0.2f);
    putF32(bytes, 0.4f);
    putF32(bytes, 0.6f);
    putF32(bytes, 1.0f);
    for (int i = 0; i < 4; ++i) putF32(bytes, 0.5f);
    for (int i = 0; i < 4; ++i) putF32(bytes, 0.0f);
    putF32(bytes, 30.0f);
    putF32(bytes, 1.0f);
    putU8(bytes, 0);
    putFixed(bytes, "C:\\art\\body.png", 128);
    putFixed(bytes, "", 128);

    putF32(bytes, 24.0f);
    putF32(bytes, 0.0f);
    putS32(bytes, 30);

    jointsOffset = bytes.size();
    if (!options.joints) return;

    putU16(bytes, 2);

    putU8(bytes, 0);
    putFixed(bytes, "root", 32);
    putFixed(bytes, options.cycle ? "arm" : "", 32);
    putVec3(bytes, 0.0f, 0.0f, 0.0f);
    putVec3(bytes, 1.0f, 0.0f, 0.0f);
    putU16(bytes, 2);
    putU16(bytes, 2);
    putKey(bytes, 0.25f, 0.0f, 0.0f, 0.0f);
    putKey(bytes, 0.75f, 0.0f, 0.0f, 0.4f);
    putKey(bytes, 0.25f, 0.0f, 0.0f, 0.0f);
    putKey(bytes, 0.75f, 0.0f, 0.0f, 1.0f);

    putU8(bytes, 0);
    putFixed(bytes, "arm", 32);
    putFixed(bytes, "root", 32);
    putVec3(bytes, 0.0f, 0.0f, 0.2f);
    putVec3(bytes, 0.0f, 1.0f, 0.0f);
    putU16(bytes, 2);
    putU16(bytes, 2);
    putKey(bytes, options.unorderedKeys ? 1.0f : 0.5f, 0.0f, 0.0f, 0.5f);
    putKey(bytes, options.unorderedKeys ? 0.5f : 1.0f, 0.0f, 0.0f, 1.0f);
    putKey(bytes, 0.5f, 0.0f, 0.5f, 0.0f);
    putKey(bytes, 1.0f, 0.0f, 1.0f, 0.0f);
}

ct::String pathOf(const char* name)
{
    return ct::String(kDirectory) + "/" + name;
}

bool writeFile(const char* name, const Bytes& bytes)
{
    return file_write(pathOf(name).c_str(), bytes.data(), bytes.size());
}

void checkGeometry(const ModelImporter& importer, const ModelData& model, bool joints)
{
    CHECK(model.meshes.size() == 1);
    if (model.meshes.empty() || model.meshes[0].primitives.size() != 1)
    {
        CHECK(false);
        return;
    }
    CHECK(model.primitiveCount == 1);
    CHECK(model.geometryBytes > 0);
    const ModelPrimitiveData& primitive = model.meshes[0].primitives[0];
    std::uint32_t mask = kVertexPosition | kVertexNormal | kVertexUv;
    if (joints) mask |= kVertexJoints | kVertexWeights;
    CHECK(primitive.attributeMask == mask);
    CHECK(primitive.vertexCount == 3);
    CHECK(primitive.indexCount == 3);
    CHECK(primitive.skinned == joints);
    CHECK(primitive.hasAabb);
    CHECK(primitive.material == 0);

    SurfaceData surface;
    CHECK(importer.readPrimitive(model, primitive, surface));
    CHECK(surface.topology == prisma::Topology::Triangles);
    CHECK(surface.positions.size() == 3);
    CHECK(surface.normals.size() == 3);
    CHECK(surface.uvs.size() == 3);
    CHECK(surface.indices.size() == 3);
    if (surface.positions.size() != 3 || surface.normals.size() != 3 || surface.uvs.size() != 3 ||
            surface.indices.size() != 3)
        return;

    const float positions[3][3] = { { 0, 0, 0 }, { 1, 0, 0 }, { 0, 1, 0 } };
    const float normals[3][3] = { { 0, 0, 1 }, { 0, 1, 0 }, { 1, 0, 0 } };
    const float uvs[3][2] = { { 0.1f, 0.8f }, { 0.5f, 0.6f }, { 0.9f, 0.4f } };
    for (std::size_t i = 0; i < 3; ++i)
    {
        CHECK(surface.indices[i] == i);
        CHECK(nearVec3(surface.positions[i], positions[i][0], positions[i][1], positions[i][2]));
        CHECK(nearVec3(surface.normals[i], normals[i][0], normals[i][1], normals[i][2]));
        CHECK(near(surface.uvs[i].x, uvs[i][0]) && near(surface.uvs[i].y, uvs[i][1]));
    }

    if (!joints)
    {
        CHECK(surface.joints.empty());
        CHECK(surface.weights.empty());
        return;
    }
    CHECK(surface.joints.size() == 3);
    CHECK(surface.weights.size() == 3);
    if (surface.joints.size() != 3 || surface.weights.size() != 3) return;
    CHECK(surface.joints[0] == 0u);
    CHECK(surface.joints[1] == 0x01010101u);
    CHECK(surface.joints[2] == 0u);
    for (std::size_t i = 0; i < 3; ++i)
    {
        CHECK(near(surface.weights[i].x, 1.0f) && near(surface.weights[i].y, 0.0f) &&
                near(surface.weights[i].z, 0.0f) && near(surface.weights[i].w, 0.0f));
    }

    SurfaceData unused;
    ModelPrimitiveData missing;
    missing.source = 9;
    CHECK(!importer.readPrimitive(model, missing, unused));
}

void checkMaterial(const ModelImporter& importer, const ModelData& model)
{
    CHECK(model.materials.size() == 1);
    CHECK(model.textures.size() == 1);
    CHECK(model.images.size() == 1);
    if (model.materials.size() != 1 || model.textures.size() != 1 || model.images.size() != 1)
        return;
    const ModelMaterialData& material = model.materials[0];
    CHECK(material.name == "skin");
    CHECK(near(material.baseColor.x, 0.2f) && near(material.baseColor.y, 0.4f) &&
            near(material.baseColor.z, 0.6f) && near(material.baseColor.w, 1.0f));
    CHECK(near(material.roughness, 0.25f));
    CHECK(near(material.metallic, 0.0f));
    CHECK(material.baseColorTexture == 0);
    CHECK(model.textures[0].image == 0);
    CHECK(model.images[0].name == "body.png");
    CHECK(model.images[0].path == model.directory + "body.png");
    CHECK(!model.images[0].embedded());
    ct::Vector<std::uint8_t> pixels;
    CHECK(!importer.readImage(model, model.images[0], pixels));
    CHECK(pixels.empty());
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
    CHECK(nearVec3(skeleton.bone(0).bindLocal.GetTranslation(), 1.0f, 0.0f, 0.0f));
    CHECK(nearVec3(skeleton.bone(1).bindLocal.GetTranslation(), 0.0f, 1.0f, 0.0f));
    const Math::Mat4 global = skeleton.bone(0).bindLocal * skeleton.bone(1).bindLocal;
    const Math::Mat4 product = global * skeleton.bone(1).inverseBind;
    const Math::Mat4 identity = Math::Mat4::Identity();
    for (int c = 0; c < 4; ++c)
    {
        for (int r = 0; r < 4; ++r) CHECK(near(product[c][r], identity[c][r]));
    }
}

void checkClip(const ModelData& model, const char* stem)
{
    CHECK(model.animations.size() == 1);
    if (model.animations.empty()) return;
    const AnimationClip& clip = model.animations[0];
    CHECK(clip.name() == stem);
    CHECK(clip.skeleton() == 0);
    CHECK(near(clip.duration(), 1.0f));
    CHECK(clip.tracks().size() == 2);
    if (clip.tracks().size() != 2) return;

    const BoneTrack& root = clip.tracks()[0];
    CHECK(root.bone == 0);
    CHECK(root.times.size() == 2);
    if (root.times.size() == 2 && root.positions.size() == 2 && root.rotations.size() == 2 &&
            root.scales.size() == 2)
    {
        CHECK(near(root.times[0], 0.25f) && near(root.times[1], 0.75f));
        CHECK(nearVec3(root.positions[0], 1.0f, 0.0f, 0.0f));
        CHECK(nearVec3(root.positions[1], 1.0f, 0.0f, 1.0f));
        CHECK(nearQuat(root.rotations[0], 0.0f, 0.0f, 0.0f, 1.0f));
        CHECK(nearQuat(root.rotations[1], 0.0f, 0.0f, sinf(0.2f), cosf(0.2f)));
        CHECK(nearVec3(root.scales[0], 1.0f, 1.0f, 1.0f));
    }

    const BoneTrack& arm = clip.tracks()[1];
    CHECK(arm.bone == 1);
    CHECK(arm.times.size() == 2);
    if (arm.times.size() == 2 && arm.positions.size() == 2 && arm.rotations.size() == 2 &&
            arm.scales.size() == 2)
    {
        CHECK(near(arm.times[0], 0.5f) && near(arm.times[1], 1.0f));
        CHECK(nearVec3(arm.positions[0], -0.5f * sinf(0.2f), 1.0f + 0.5f * cosf(0.2f), 0.0f));
        CHECK(nearVec3(arm.positions[1], -sinf(0.2f), 1.0f + cosf(0.2f), 0.0f));
        CHECK(nearQuat(arm.rotations[0], 0.0f, 0.0f, sinf(0.35f), cosf(0.35f)));
        CHECK(nearQuat(arm.rotations[1], 0.0f, 0.0f, sinf(0.6f), cosf(0.6f)));
        CHECK(nearVec3(arm.scales[1], 1.0f, 1.0f, 1.0f));
    }

    ct::Vector<LocalPose> pose;
    model.skeletons[0].bindPose(pose);
    CHECK(pose.size() == 2);
    if (pose.size() != 2) return;
    clip.sample(0.5f, pose);
    CHECK(nearVec3(pose[1].position, -0.5f * sinf(0.2f), 1.0f + 0.5f * cosf(0.2f), 0.0f));
    CHECK(nearQuat(pose[1].rotation, 0.0f, 0.0f, sinf(0.35f), cosf(0.35f)));
    clip.sample(0.75f, pose);
    CHECK(nearVec3(pose[0].position, 1.0f, 0.0f, 1.0f));
}

void checkModel(const ModelImporter& importer, const ModelData& model, const char* stem,
        bool joints)
{
    CHECK(model.error.empty());
    CHECK(model.nodes.size() == 1);
    CHECK(model.roots.size() == 1);
    if (model.nodes.size() == 1 && model.roots.size() == 1)
    {
        const ModelNodeData& node = model.nodes[0];
        CHECK(model.roots[0] == 0);
        CHECK(node.name == stem);
        CHECK(node.parent == -1);
        CHECK(node.children.empty());
        CHECK(node.mesh == 0);
        CHECK(node.skeleton == (joints ? 0 : -1));
        CHECK(nearVec3(node.translation, 0.0f, 0.0f, 0.0f));
        CHECK(nearVec3(node.scale, 1.0f, 1.0f, 1.0f));
        CHECK(nearQuat(node.rotation, 0.0f, 0.0f, 0.0f, 1.0f));
    }
    if (!model.meshes.empty()) CHECK(model.meshes[0].name == stem);

    checkGeometry(importer, model, joints);
    checkMaterial(importer, model);
    if (joints)
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

} // namespace

int main()
{
    CHECK(fs_create_directory(kDirectory));

    std::size_t jointsOffset = 0;
    Bytes full;
    buildMS3D(full, Options(), jointsOffset);
    const std::size_t staticLength = jointsOffset;
    CHECK(staticLength > 0 && staticLength < full.size());
    CHECK(writeFile("hero.ms3d", full));

    MS3DImporter importer;
    CHECK(strcmp(importer.name(), "MilkShape 3D") == 0);
    CHECK(importer.supports("ms3d"));
    CHECK(!importer.supports("b3d"));
    CHECK(!importer.supports(nullptr));

    ModelData model;
    CHECK(importer.import(pathOf("hero.ms3d"), model));
    CHECK(model.path == pathOf("hero.ms3d"));
    CHECK(model.directory == pathOf(""));
    checkModel(importer, model, "hero", true);

    ModelData memoryModel;
    CHECK(importer.importMemory(full.data(), full.size(), pathOf("memory.ms3d"), memoryModel));
    checkModel(importer, memoryModel, "memory", true);

    Options staticOptions;
    staticOptions.joints = false;
    Bytes staticFile;
    buildMS3D(staticFile, staticOptions, jointsOffset);
    CHECK(staticFile.size() == staticLength);
    CHECK(writeFile("block.ms3d", staticFile));
    ModelData staticModel;
    CHECK(importer.import(pathOf("block.ms3d"), staticModel));
    checkModel(importer, staticModel, "block", false);

    Options version3;
    version3.version = 3;
    Bytes version3File;
    buildMS3D(version3File, version3, jointsOffset);
    ModelData version3Model;
    CHECK(importer.importMemory(version3File.data(), version3File.size(), pathOf("old.ms3d"),
            version3Model));
    checkModel(importer, version3Model, "old", true);

    Options version5;
    version5.version = 5;
    Bytes version5File;
    buildMS3D(version5File, version5, jointsOffset);
    checkRejected(importer, version5File, "future.ms3d");
    Options version2;
    version2.version = 2;
    Bytes version2File;
    buildMS3D(version2File, version2, jointsOffset);
    checkRejected(importer, version2File, "ancient.ms3d");

    ModelLoader loader;
    loader.addImporter(new MS3DImporter());
    CHECK(loader.importerFor(pathOf("hero.ms3d")) != nullptr);
    ModelData loaded;
    CHECK(loader.load(pathOf("hero.ms3d"), loaded));
    checkModel(*loader.importerFor(pathOf("hero.ms3d")), loaded, "hero", true);
    ModelData loadedMemory;
    CHECK(loader.loadFromMemory(full.data(), full.size(), pathOf("loader.MS3D"), loadedMemory));
    checkModel(*loader.importer(0), loadedMemory, "loader", true);
    ModelData unknown;
    CHECK(!loader.load(pathOf("hero.unknown"), unknown));
    CHECK(!unknown.error.empty());

    Bytes badMagic = full;
    badMagic[0] = 'X';
    checkRejected(importer, badMagic, "magic.ms3d");

    for (std::size_t length = 1; length < full.size(); ++length)
    {
        ModelData partial;
        const bool imported =
                importer.importMemory(full.data(), length, pathOf("partial.ms3d"), partial);
        if (length == staticLength)
        {
            CHECK(imported);
            CHECK(partial.skeletons.empty());
        }
        else
        {
            CHECK(!imported);
            CHECK(!partial.error.empty());
        }
    }

    Bytes truncated;
    for (std::size_t i = 0; i < full.size() / 2; ++i) truncated.push_back(full[i]);
    CHECK(writeFile("truncated.ms3d", truncated));
    ModelData failed;
    CHECK(!importer.import(pathOf("truncated.ms3d"), failed));
    CHECK(!failed.error.empty());

    Options badTriangle;
    badTriangle.badTriangle = true;
    Bytes badTriangleFile;
    buildMS3D(badTriangleFile, badTriangle, jointsOffset);
    checkRejected(importer, badTriangleFile, "triangle.ms3d");

    Options badMaterial;
    badMaterial.badMaterial = true;
    Bytes badMaterialFile;
    buildMS3D(badMaterialFile, badMaterial, jointsOffset);
    checkRejected(importer, badMaterialFile, "material.ms3d");

    Options badBone;
    badBone.badBone = true;
    Bytes badBoneFile;
    buildMS3D(badBoneFile, badBone, jointsOffset);
    checkRejected(importer, badBoneFile, "bone.ms3d");

    Options cycle;
    cycle.cycle = true;
    Bytes cycleFile;
    buildMS3D(cycleFile, cycle, jointsOffset);
    checkRejected(importer, cycleFile, "cycle.ms3d");

    Options unordered;
    unordered.unorderedKeys = true;
    Bytes unorderedFile;
    buildMS3D(unorderedFile, unordered, jointsOffset);
    checkRejected(importer, unorderedFile, "unordered.ms3d");

    CHECK(!importer.import(pathOf("absent.ms3d"), failed));
    CHECK(!failed.error.empty());
    CHECK(!importer.importMemory(nullptr, 0, pathOf("empty.ms3d"), failed));
    CHECK(!failed.error.empty());

    if (failures == 0) printf("ms3d: ok\n");
    return failures == 0 ? 0 : 1;
}
