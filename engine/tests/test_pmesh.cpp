#include "Check.h"
#include "PMeshSupport.h"

#include "prisma3d/core/FileSystem.h"
#include "prisma3d/import/B3DImporter.h"
#include "prisma3d/import/GltfImporter.h"
#include "prisma3d/import/ObjImporter.h"
#include "prisma3d/import/OgreMeshImporter.h"
#include "prisma3d/import/PMeshIO.h"
#include "prisma3d/import/PMeshImporter.h"
#include "prisma3d/import/PMeshWriter.h"

#include "platform.h"

#include <stdio.h>
#include <string.h>

using namespace prisma3d;

namespace
{

const char* const kRoot = PRISMA3D_TEST_DIR "/pmesh";
const char* const kAssets = PRISMA3D_ASSETS_DIR "/models";

struct Combo
{
    bool normal;
    bool tangent;
    bool uv;
    bool uv2;
    bool color;
    bool skin;
};

float value(std::uint32_t seed)
{
    const std::uint32_t mixed = seed * 2654435761u + 12345u;
    return static_cast<float>(static_cast<int>(mixed >> 8) - 8000000) * 0.0001f;
}

SurfaceData makeSurface(const Combo& combo, std::uint32_t count, std::uint32_t seed)
{
    SurfaceData s;
    s.topology = prisma::Topology::Triangles;
    for (std::uint32_t i = 0; i < count; ++i)
    {
        s.positions.push_back(Math::Vec3(value(seed + i * 11), value(seed + i * 11 + 1),
                value(seed + i * 11 + 2)));
        if (combo.normal)
            s.normals.push_back(Math::Vec3(value(seed + i * 7), value(seed + i * 7 + 3), -0.0f));
        if (combo.tangent)
            s.tangents.push_back(
                    Math::Vec4(value(seed + i), value(seed + i * 5), value(seed + i * 3), 1.0f));
        if (combo.uv) s.uvs.push_back(Math::Vec2(value(seed + i * 13), value(seed + i * 17)));
        if (combo.uv2) s.uvs2.push_back(Math::Vec2(value(seed + i * 19), value(seed + i * 23)));
        if (combo.color)
            s.colors.push_back(Math::Vec4(value(seed + i * 29), 0.5f, 1.0f, value(seed + i * 31)));
        if (combo.skin)
        {
            s.joints.push_back(i | ((i + 1) << 8) | ((i + 2) << 16) | ((i + 3) << 24));
            s.weights.push_back(Math::Vec4(0.5f, 0.25f, 0.125f, 0.125f));
        }
    }
    if (count >= 4)
    {
        const std::uint32_t indices[6] = { 0, 1, 2, 2, 1, 3 };
        for (std::uint32_t index: indices) s.indices.push_back(index);
    }
    return s;
}

void addMesh(ModelData& model, MemoryImporter& importer, const SurfaceData& surface,
        const char* meshName, int material, bool skinned)
{
    ModelMeshData mesh;
    mesh.name = meshName;
    ModelPrimitiveData primitive;
    primitive.topology = surface.topology;
    primitive.material = material;
    primitive.attributeMask = maskOf(surface);
    primitive.vertexCount = static_cast<std::uint32_t>(surface.positions.size());
    primitive.indexCount = static_cast<std::uint32_t>(surface.indices.size());
    primitive.source = static_cast<std::uint32_t>(importer.surfaces.size());
    primitive.aabb = MeshStore::computeAabb(surface.positions);
    primitive.hasAabb = true;
    primitive.skinned = skinned;
    mesh.primitives.push_back(primitive);
    importer.surfaces.push_back(surface);
    model.meshes.push_back(mesh);
}

void addRoot(ModelData& model, const char* name, int mesh)
{
    ModelNodeData node;
    node.name = name;
    node.mesh = mesh;
    model.nodes.push_back(node);
    model.roots.push_back(static_cast<int>(model.nodes.size() - 1));
}

void addMaterial(ModelData& model, const char* name)
{
    ModelMaterialData material;
    material.name = name;
    model.materials.push_back(material);
}

bool readBack(const ct::Vector<std::uint8_t>& bytes, const char* name, PMeshImporter& importer,
        ModelData& out)
{
    return importer.importMemory(bytes.data(), bytes.size(), name, out);
}

std::size_t expectedSize(const Combo& c, std::uint32_t vertices, std::uint32_t indices,
        std::size_t indexWidth)
{
    std::size_t stride = 12;
    if (c.normal) stride += 12;
    if (c.tangent) stride += 16;
    if (c.uv) stride += 8;
    if (c.uv2) stride += 8;
    if (c.color) stride += 16;
    std::size_t size = 12;
    size += 12 + 4 + 4 + 5;
    size += 12 + 16 + vertices * stride;
    if (indices) size += 12 + 12 + indices * indexWidth;
    if (c.skin) size += 12 + 8 + vertices * 20;
    size += 12 + 4 + (4 + 4) + 4 + 36;
    size += 12 + 4 + (4 + 4 + 16 + 40 + 4) + 4 + 4;
    size += 12;
    return size;
}

void testAttributeCombinations()
{
    const Combo combos[] = {
        { false, false, false, false, false, false },
        { true, false, false, false, false, false },
        { true, true, false, false, false, false },
        { false, false, true, false, false, false },
        { false, false, true, true, false, false },
        { false, false, false, false, true, false },
        { true, false, true, false, false, true },
        { true, true, true, true, true, true },
    };
    std::size_t previous = 0;
    for (std::size_t k = 0; k < sizeof(combos) / sizeof(combos[0]); ++k)
    {
        const Combo& combo = combos[k];
        MemoryImporter source;
        ModelData model;
        addMaterial(model, "mat_a");
        const SurfaceData surface = makeSurface(combo, 4, 100 + static_cast<std::uint32_t>(k));
        addMesh(model, source, surface, "quad", 0, combo.skin);
        addRoot(model, "root", 0);

        PMeshFiles files;
        ct::String error;
        CHECK(buildPMesh(model, source, "quad", PMeshSaveOptions(), files, error));
        CHECK(error.empty());
        CHECK(files.mesh.size() == expectedSize(combo, 4, 6, 2));
        if (k == 1) CHECK(files.mesh.size() == previous + 4 * 12);
        previous = files.mesh.size();
        CHECK(files.skeletons.empty() && files.animations.empty());

        PMeshImporter importer;
        ModelData back;
        CHECK(readBack(files.mesh, "quad.pmesh", importer, back));
        CHECK(back.error.empty());
        CHECK(back.primitiveCount == 1);
        CHECK(back.materials.size() == 1 && back.materials[0].name == "mat_a");
        CHECK(modelDifferences(model, source, back, importer) == 0);
        if (back.meshes.size() == 1 && back.meshes[0].primitives.size() == 1)
        {
            const ModelPrimitiveData& p = back.meshes[0].primitives[0];
            CHECK(p.attributeMask == maskOf(surface));
            CHECK(p.indexCount == 6 && p.vertexCount == 4);
            CHECK(p.skinned == combo.skin);
            CHECK(p.material == 0);
        }
    }
}

void testNoIndicesAndNoMaterial()
{
    MemoryImporter source;
    ModelData model;
    const Combo combo = { true, false, true, false, false, false };
    SurfaceData surface = makeSurface(combo, 3, 5);
    CHECK(surface.indices.empty());
    surface.topology = prisma::Topology::Lines;
    addMesh(model, source, surface, "", -1, false);
    PMeshFiles files;
    ct::String error;
    CHECK(buildPMesh(model, source, "lines", PMeshSaveOptions(), files, error));
    CHECK(files.mesh.size() == 12 + (12 + 4) + (12 + 16 + 3 * 32) + (12 + 4 + 4 + 4 + 36) + 12);
    PMeshImporter importer;
    ModelData back;
    CHECK(readBack(files.mesh, "lines.pmesh", importer, back));
    CHECK(back.materials.empty() && back.nodes.empty());
    CHECK(back.meshes.size() == 1 && back.meshes[0].primitives.size() == 1);
    CHECK(back.meshes[0].primitives[0].material == -1);
    CHECK(back.meshes[0].primitives[0].topology == prisma::Topology::Lines);
    CHECK(modelDifferences(model, source, back, importer) == 0);
}

void testIndexWidth()
{
    const std::uint32_t cases[4][2] = { { 65536, 65535 }, { 65537, 65535 }, { 65537, 65536 },
        { 70000, 69999 } };
    for (std::size_t k = 0; k < 4; ++k)
    {
        const std::uint32_t vertices = cases[k][0];
        const std::uint32_t top = cases[k][1];
        MemoryImporter source;
        ModelData model;
        addMaterial(model, "mat_a");
        SurfaceData surface;
        for (std::uint32_t i = 0; i < vertices; ++i)
            surface.positions.push_back(Math::Vec3(static_cast<float>(i), 0.0f, 0.0f));
        surface.indices.push_back(0);
        surface.indices.push_back(1);
        surface.indices.push_back(top);
        addMesh(model, source, surface, "quad", 0, false);
        addRoot(model, "root", 0);
        PMeshFiles files;
        ct::String error;
        CHECK(buildPMesh(model, source, "big", PMeshSaveOptions(), files, error));
        const Combo none = { false, false, false, false, false, false };
        const std::size_t width = top <= 65535 ? 2 : 4;
        CHECK(files.mesh.size() == expectedSize(none, vertices, 3, width));
        PMeshImporter importer;
        ModelData back;
        CHECK(readBack(files.mesh, "big.pmesh", importer, back));
        CHECK(modelDifferences(model, source, back, importer) == 0);
    }
}

void testWriterRejects()
{
    PMeshSaveOptions options;
    ct::String error;
    PMeshFiles files;
    {
        MemoryImporter source;
        ModelData model;
        addMaterial(model, "m");
        SurfaceData surface = makeSurface({ true, false, false, false, false, false }, 4, 1);
        surface.indices.push_back(4);
        addMesh(model, source, surface, "bad", 0, false);
        CHECK(!buildPMesh(model, source, "bad", options, files, error));
        CHECK(!error.empty());
    }
    {
        MemoryImporter source;
        ModelData model;
        addMaterial(model, "m");
        SurfaceData surface = makeSurface({ true, false, false, false, false, false }, 4, 1);
        surface.normals.pop_back();
        addMesh(model, source, surface, "bad", 0, false);
        CHECK(!buildPMesh(model, source, "bad", options, files, error));
    }
    {
        MemoryImporter source;
        ModelData model;
        SurfaceData surface = makeSurface({ false, false, false, false, false, false }, 4, 1);
        addMesh(model, source, surface, "bad", 3, false);
        CHECK(!buildPMesh(model, source, "bad", options, files, error));
    }
    {
        MemoryImporter source;
        ModelData model;
        SurfaceData surface = makeSurface({ false, false, false, false, false, true }, 4, 1);
        surface.weights.clear();
        addMesh(model, source, surface, "bad", -1, true);
        CHECK(!buildPMesh(model, source, "bad", options, files, error));
    }
    {
        MemoryImporter source;
        ModelData model;
        addMesh(model, source, SurfaceData(), "empty", -1, false);
        CHECK(!buildPMesh(model, source, "bad", options, files, error));
    }
}

Skeleton makeSkeleton()
{
    Skeleton skeleton;
    Math::Mat4 a = Math::Mat4::Identity();
    a.col3 = Math::Vec4(0.0f, 1.0f, 0.0f, 1.0f);
    Math::Mat4 b = Math::Mat4::Identity();
    b.col3 = Math::Vec4(0.5f, 0.25f, 0.0f, 1.0f);
    Math::Mat4 c = Math::Mat4::Identity();
    c.col3 = Math::Vec4(0.0f, 0.0f, 2.0f, 1.0f);
    CHECK(skeleton.addBone("root", -1, a, a.Inverse()));
    CHECK(skeleton.addBone("arm", 0, b, b.Inverse()));
    CHECK(skeleton.addBone("hand", 1, c, c.Inverse()));
    CHECK(skeleton.finalize());
    return skeleton;
}

AnimationClip makeBoneClip()
{
    AnimationClip clip;
    clip.setName("walk");
    clip.setDuration(2.0f);
    clip.setSkeleton(0);
    clip.setLoopMode(LoopMode::PingPong);
    const int bones[2] = { 0, 2 };
    for (int bone: bones)
    {
        BoneTrack track;
        track.bone = bone;
        for (int k = 0; k < 3; ++k)
        {
            track.times.push_back(static_cast<float>(k) * 0.9f);
            track.positions.push_back(
                    Math::Vec3(static_cast<float>(k), static_cast<float>(bone), 0.25f));
            track.rotations.push_back(
                    Math::Quaternion(0.0f, 0.0f, 0.1f * static_cast<float>(k), 1.0f).Normalized());
            track.scales.push_back(Math::Vec3(1.0f, 1.0f + 0.1f * static_cast<float>(k), 1.0f));
        }
        clip.tracks().push_back(track);
    }
    NodeTrack node;
    node.path = "a/b";
    node.positionTimes.push_back(0.0f);
    node.positionTimes.push_back(1.0f);
    node.positions.push_back(Math::Vec3(0.0f, 0.0f, 0.0f));
    node.positions.push_back(Math::Vec3(1.0f, 2.0f, 3.0f));
    node.rotationTimes.push_back(0.5f);
    node.rotations.push_back(Math::Quaternion(0.0f, 0.0f, 0.0f, 1.0f));
    clip.nodeTracks().push_back(node);
    MethodTrack method;
    method.path = "player";
    method.times.push_back(0.25f);
    method.methods.push_back("step");
    clip.methodTracks().push_back(method);
    ValueTrack valueTrack;
    valueTrack.path = "light";
    valueTrack.property = "energy";
    valueTrack.times.push_back(0.0f);
    valueTrack.times.push_back(2.0f);
    valueTrack.values.push_back(1.0f);
    valueTrack.values.push_back(4.0f);
    clip.valueTracks().push_back(valueTrack);
    AnimationTrack animation;
    animation.path = "player";
    animation.times.push_back(1.0f);
    animation.animations.push_back("run");
    clip.animationTracks().push_back(animation);
    clip.addEvent(0.5f, "foot");
    clip.addEvent(0.25f, "early");
    clip.addEvent(0.5f, "foot_again");
    return clip;
}

AnimationClip makeNodeClip()
{
    AnimationClip clip;
    clip.setName("spin");
    clip.setDuration(1.0f);
    clip.setSkeleton(-1);
    clip.setLoopMode(LoopMode::Linear);
    NodeTrack node;
    node.path = "x";
    node.scaleTimes.push_back(0.0f);
    node.scales.push_back(Math::Vec3(2.0f, 2.0f, 2.0f));
    clip.nodeTracks().push_back(node);
    return clip;
}

void buildRichModel(ModelData& model, MemoryImporter& source)
{
    addMaterial(model, "skin");
    addMaterial(model, "metal");
    const Combo skinned = { true, true, true, true, true, true };
    addMesh(model, source, makeSurface(skinned, 4, 7), "body", 0, true);
    ModelPrimitiveData extra;
    const SurfaceData second = makeSurface({ true, false, true, false, false, false }, 4, 9);
    extra = model.meshes[0].primitives[0];
    extra.source = static_cast<std::uint32_t>(source.surfaces.size());
    extra.attributeMask = maskOf(second);
    extra.material = 1;
    extra.skinned = false;
    extra.aabb = MeshStore::computeAabb(second.positions);
    source.surfaces.push_back(second);
    model.meshes[0].primitives.push_back(extra);
    model.meshes[0].shadowSurfaces.push_back(
            makeSurface({ false, false, false, false, false, false }, 4, 3));
    addMesh(model, source, makeSurface({ false, false, true, false, false, false }, 4, 21), "prop",
            1, false);

    ModelNodeData root;
    root.name = "root";
    root.children.push_back(1);
    root.children.push_back(2);
    root.children.push_back(3);
    root.translation = Math::Vec3(1.0f, 2.0f, 3.0f);
    model.nodes.push_back(root);
    ModelNodeData body;
    body.name = "body";
    body.parent = 0;
    body.mesh = 0;
    body.skeleton = 0;
    body.rotation = Math::Quaternion(0.0f, 0.7071068f, 0.0f, 0.7071068f);
    model.nodes.push_back(body);
    ModelNodeData camera;
    camera.name = "";
    camera.parent = 0;
    camera.camera = 0;
    camera.scale = Math::Vec3(2.0f, 2.0f, 2.0f);
    ModelNodeData prop;
    prop.name = "prop";
    prop.parent = 0;
    prop.mesh = 1;
    model.nodes.push_back(camera);
    model.nodes.push_back(prop);
    model.roots.push_back(0);

    ModelCameraData camData;
    camData.perspective = false;
    camData.xmag = 3.0f;
    model.cameras.push_back(camData);
    model.skeletons.push_back(makeSkeleton());
    model.animations.push_back(makeBoneClip());
    model.animations.push_back(makeNodeClip());
}

bool chunkAt(const ct::Vector<std::uint8_t>& bytes, std::uint32_t id, std::size_t& offset,
        std::uint64_t& size)
{
    std::size_t at = pmesh::kHeaderSize;
    while (at + pmesh::kChunkHeaderSize <= bytes.size())
    {
        std::uint32_t chunk = 0;
        std::uint64_t length = 0;
        for (int i = 0; i < 4; ++i) chunk |= static_cast<std::uint32_t>(bytes[at + i]) << (8 * i);
        for (int i = 0; i < 8; ++i)
            length |= static_cast<std::uint64_t>(bytes[at + 4 + i]) << (8 * i);
        if (chunk == id)
        {
            offset = at;
            size = length;
            return true;
        }
        at += pmesh::kChunkHeaderSize + static_cast<std::size_t>(length);
    }
    return false;
}

void patch32(ct::Vector<std::uint8_t>& bytes, std::size_t at, std::uint32_t v)
{
    for (int i = 0; i < 4; ++i) bytes[at + i] = static_cast<std::uint8_t>(v >> (8 * i));
}

void patch64(ct::Vector<std::uint8_t>& bytes, std::size_t at, std::uint64_t v)
{
    for (int i = 0; i < 8; ++i) bytes[at + i] = static_cast<std::uint8_t>(v >> (8 * i));
}

ct::String under(const char* directory, const char* name)
{
    return ct::String(directory) + "/" + name;
}

void testRichRoundTrip()
{
    MemoryImporter source;
    ModelData model;
    buildRichModel(model, source);

    PMeshFiles files;
    ct::String error;
    CHECK(buildPMesh(model, source, "rich", PMeshSaveOptions(), files, error));
    CHECK(error.empty());
    CHECK(files.skeletons.size() == 1 && files.skeletonNames[0] == "rich.pskel");
    CHECK(files.animationName == "rich.panim" && !files.animations.empty());

    const ct::String directory = under(kRoot, "rich");
    CHECK(fs_create_directory(directory.c_str()));
    CHECK(savePMesh(model, source, under(directory.c_str(), "rich.pmesh"), PMeshSaveOptions(),
            error));
    CHECK(FileSystem::getSingleton().exists(under(directory.c_str(), "rich.pskel")));
    CHECK(FileSystem::getSingleton().exists(under(directory.c_str(), "rich.panim")));

    PMeshImporter importer;
    ModelData back;
    CHECK(importer.import(under(directory.c_str(), "rich.pmesh"), back));
    CHECK(back.error.empty());
    CHECK(modelDifferences(model, source, back, importer) == 0);
    CHECK(back.skeletons.size() == 1 && back.animations.size() == 2);
    CHECK(back.animations.size() == 2 && back.animations[0].events().size() == 3);
    CHECK(back.meshes.size() == 2 && back.meshes[0].shadowSurfaces.size() == 1);
}

void testContainerRules()
{
    MemoryImporter source;
    ModelData model;
    buildRichModel(model, source);
    const ct::String directory = under(kRoot, "rules");
    CHECK(fs_create_directory(directory.c_str()));
    ct::String error;
    CHECK(savePMesh(model, source, under(directory.c_str(), "rich.pmesh"), PMeshSaveOptions(),
            error));
    PMeshFiles files;
    CHECK(buildPMesh(model, source, "rich", PMeshSaveOptions(), files, error));
    const ct::String name = under(directory.c_str(), "rich.pmesh");

    PMeshImporter importer;
    ModelData base;
    CHECK(readBack(files.mesh, name.c_str(), importer, base));

    {
        ct::Vector<std::uint8_t> bytes = files.mesh;
        std::size_t offset = 0;
        std::uint64_t size = 0;
        CHECK(chunkAt(bytes, pmesh::SubMeshes, offset, size));
        ct::Vector<std::uint8_t> inserted;
        for (std::size_t i = 0; i < offset; ++i) inserted.push_back(bytes[i]);
        pmesh::Writer w(inserted);
        const std::size_t marker = w.beginChunk(pmesh::fourcc('Z', 'Z', 'Z', 'Z'));
        w.u32(0xDEADBEEFu);
        w.u8(1);
        w.endChunk(marker);
        for (std::size_t i = offset; i < bytes.size(); ++i) inserted.push_back(bytes[i]);
        ModelData loaded;
        CHECK(readBack(inserted, name.c_str(), importer, loaded));
        CHECK(modelDifferences(base, importer, loaded, importer) == 0);
        ModelData noMeshes;
        ct::Vector<std::uint8_t> head;
        for (std::size_t i = 0; i < pmesh::kHeaderSize; ++i) head.push_back(bytes[i]);
        patch32(head, 8, 1u);
        pmesh::Writer hw(head);
        const std::size_t m2 = hw.beginChunk(pmesh::fourcc('Q', 'Q', 'Q', 'Q'));
        hw.endChunk(m2);
        const std::size_t m3 = hw.beginChunk(pmesh::End);
        hw.endChunk(m3);
        CHECK(readBack(head, name.c_str(), importer, noMeshes));
        CHECK(noMeshes.meshes.empty() && noMeshes.nodes.empty());
    }
    {
        ct::Vector<std::uint8_t> bytes = files.mesh;
        patch32(bytes, 4, (static_cast<std::uint32_t>(pmesh::kVersionMajor) << 16) | 7u);
        ModelData loaded;
        CHECK(readBack(bytes, name.c_str(), importer, loaded));
        CHECK(modelDifferences(base, importer, loaded, importer) == 0);
    }
    {
        ct::Vector<std::uint8_t> bytes = files.mesh;
        patch32(bytes, 4, (static_cast<std::uint32_t>(pmesh::kVersionMajor + 1) << 16));
        ModelData loaded;
        CHECK(!readBack(bytes, name.c_str(), importer, loaded));
        CHECK(loaded.error.find("major") != ct::String::npos);
        patch32(bytes, 4, 0u);
        CHECK(!readBack(bytes, name.c_str(), importer, loaded));
    }
    {
        ct::Vector<std::uint8_t> bytes = files.mesh;
        bytes[0] ^= 0x55;
        ModelData loaded;
        CHECK(!readBack(bytes, name.c_str(), importer, loaded));
        CHECK(loaded.error.find("magic") != ct::String::npos);
        bytes = files.mesh;
        patch32(bytes, 8, 0u);
        CHECK(!readBack(bytes, name.c_str(), importer, loaded));
    }
    {
        ModelData loaded;
        CHECK(!importer.importMemory(nullptr, 0, name, loaded));
        CHECK(!loaded.error.empty());
        CHECK(!importer.import(under(directory.c_str(), "missing.pmesh"), loaded));
    }
}

void testCorruption()
{
    MemoryImporter source;
    ModelData model;
    buildRichModel(model, source);
    const ct::String directory = under(kRoot, "rules");
    ct::String error;
    CHECK(savePMesh(model, source, under(directory.c_str(), "rich.pmesh"), PMeshSaveOptions(),
            error));
    PMeshFiles files;
    CHECK(buildPMesh(model, source, "rich", PMeshSaveOptions(), files, error));
    const ct::String name = under(directory.c_str(), "rich.pmesh");
    PMeshImporter importer;

    auto mustFail = [&](const ct::Vector<std::uint8_t>& bytes, const char* what)
    {
        ModelData loaded;
        const bool ok = readBack(bytes, name.c_str(), importer, loaded);
        if (ok)
        {
            printf("FAIL corruption accepted: %s\n", what);
            ++failures;
        }
        else if (loaded.error.empty())
        {
            printf("FAIL corruption without a message: %s\n", what);
            ++failures;
        }
    };

    {
        ct::Vector<std::uint8_t> bytes = files.mesh;
        std::size_t offset = 0;
        std::uint64_t size = 0;
        CHECK(chunkAt(bytes, pmesh::MaterialSlots, offset, size));
        patch64(bytes, offset + 4, 0x00FFFFFFFFFFFFFFull);
        mustFail(bytes, "chunk size larger than the file");
        patch64(bytes, offset + 4, 0xFFFFFFFFFFFFFFFFull);
        mustFail(bytes, "chunk size of 2^64-1");
        patch64(bytes, offset + 4, size + 1);
        mustFail(bytes, "chunk size one byte too large");
    }
    const std::uint32_t counts[] = { 0xFFFFFFFFu, 0x80000000u, 100000001u, 3000000u, 0u };
    for (std::uint32_t count: counts)
    {
        ct::Vector<std::uint8_t> bytes = files.mesh;
        std::size_t offset = 0;
        std::uint64_t size = 0;
        CHECK(chunkAt(bytes, pmesh::VertexBuffer, offset, size));
        patch32(bytes, offset + 12 + 8, count);
        mustFail(bytes, "vertex count");
    }
    const pmesh::Chunk countChunks[] = { pmesh::MaterialSlots, pmesh::SubMeshes, pmesh::Nodes,
        pmesh::Cameras, pmesh::SkeletonLink, pmesh::AnimationLink };
    for (pmesh::Chunk chunk: countChunks)
    {
        ct::Vector<std::uint8_t> bytes = files.mesh;
        std::size_t offset = 0;
        std::uint64_t size = 0;
        CHECK(chunkAt(bytes, chunk, offset, size));
        patch32(bytes, offset + 12, 0xFFFFFFFFu);
        mustFail(bytes, "record count");
        patch32(bytes, offset + 12, 0x01000000u);
        mustFail(bytes, "large record count");
    }
    {
        ct::Vector<std::uint8_t> bytes = files.mesh;
        std::size_t offset = 0;
        std::uint64_t size = 0;
        CHECK(chunkAt(bytes, pmesh::IndexBuffer, offset, size));
        patch32(bytes, offset + 12 + 8, 0xFFFFFFFFu);
        mustFail(bytes, "index count");
        patch32(bytes, offset + 12 + 4, 0xFFu);
        mustFail(bytes, "index flags");
    }
    {
        ct::Vector<std::uint8_t> bytes = files.mesh;
        std::size_t offset = 0;
        std::uint64_t size = 0;
        CHECK(chunkAt(bytes, pmesh::VertexBuffer, offset, size));
        patch32(bytes, offset + 12 + 4, 0xFFFFFFFFu);
        mustFail(bytes, "vertex attributes");
        bytes = files.mesh;
        patch32(bytes, offset + 12 + 12, 99u);
        mustFail(bytes, "topology");
        bytes = files.mesh;
        patch32(bytes, offset + 12, 5u);
        mustFail(bytes, "surface id out of sequence");
    }
    {
        ct::Vector<std::uint8_t> bytes = files.mesh;
        std::size_t offset = 0;
        std::uint64_t size = 0;
        CHECK(chunkAt(bytes, pmesh::SubMeshes, offset, size));
        const std::size_t record = offset + 12 + 4 + (4 + 4) + 4;
        patch32(bytes, record, 77u);
        mustFail(bytes, "sub-mesh surface");
        bytes = files.mesh;
        patch32(bytes, record + 4, 9u);
        mustFail(bytes, "material slot");
        bytes = files.mesh;
        patch32(bytes, record + 12, 0x7F800000u);
        mustFail(bytes, "non-finite bounds");
        bytes = files.mesh;
        patch32(bytes, record + 12, 0x7F7FFFFFu);
        mustFail(bytes, "inverted bounds");
    }
    {
        ct::Vector<std::uint8_t> bytes = files.mesh;
        std::size_t offset = 0;
        std::uint64_t size = 0;
        CHECK(chunkAt(bytes, pmesh::Nodes, offset, size));
        const std::size_t first = offset + 12 + 4 + 4 + 4;
        patch32(bytes, first, 0u);
        mustFail(bytes, "node cycle");
        bytes = files.mesh;
        patch32(bytes, first + 4, 40u);
        mustFail(bytes, "node mesh out of range");
        bytes = files.mesh;
        patch32(bytes, first + 12, 9u);
        mustFail(bytes, "node skeleton out of range");
    }
    {
        ct::Vector<std::uint8_t> bytes = files.mesh;
        bytes.pop_back();
        mustFail(bytes, "last byte missing");
        bytes = files.mesh;
        bytes.push_back(0);
        mustFail(bytes, "trailing byte");
        bytes = files.mesh;
        std::size_t offset = 0;
        std::uint64_t size = 0;
        CHECK(chunkAt(bytes, pmesh::End, offset, size));
        bytes[offset] = 'X';
        mustFail(bytes, "end chunk missing");
        bytes = files.mesh;
        patch32(bytes, 8, 1u | (0x3Fu << pmesh::kHeaderAttribShift));
        mustFail(bytes, "header flags disagree");
    }
    {
        ct::Vector<std::uint8_t> bytes = files.mesh;
        std::size_t offset = 0;
        std::uint64_t size = 0;
        CHECK(chunkAt(bytes, pmesh::IndexBuffer, offset, size));
        const std::size_t first = offset + 12 + 12;
        const std::uint16_t huge = 9;
        bytes[first] = static_cast<std::uint8_t>(huge);
        ModelData loaded;
        CHECK(readBack(bytes, name.c_str(), importer, loaded));
        SurfaceData surface;
        CHECK(!importer.readPrimitive(loaded, loaded.meshes[0].primitives[0], surface));
        SurfaceData ok;
        CHECK(!importer.readPrimitive(loaded, ModelPrimitiveData(), ok));
        ModelPrimitiveData lost;
        lost.source = 1000;
        CHECK(!importer.readPrimitive(loaded, lost, ok));
        ModelData empty;
        CHECK(!importer.readPrimitive(empty, lost, ok));
    }
    {
        ct::Vector<std::uint8_t> skeleton = files.skeletons[0];
        ct::Vector<std::uint8_t> animations = files.animations;
        Skeleton parsed;
        ct::String problem;
        CHECK(readPMeshSkeleton(skeleton.data(), skeleton.size(), parsed, problem));
        ct::Vector<AnimationClip> clips;
        ct::Vector<Skeleton> skeletons;
        skeletons.push_back(parsed);
        CHECK(readPMeshAnimations(animations.data(), animations.size(), skeletons, clips, problem));
        CHECK(clips.size() == 2);
        std::size_t offset = 0;
        std::uint64_t size = 0;
        CHECK(chunkAt(skeleton, pmesh::Bones, offset, size));
        ct::Vector<std::uint8_t> bad = skeleton;
        patch32(bad, offset + 12, 0xFFFFFFFFu);
        Skeleton none;
        CHECK(!readPMeshSkeleton(bad.data(), bad.size(), none, problem) && !problem.empty());
        bad = skeleton;
        const std::size_t parent = offset + 12 + 4 + 4 + 4;
        patch32(bad, parent, 2u);
        CHECK(!readPMeshSkeleton(bad.data(), bad.size(), none, problem));
        ct::Vector<std::uint8_t> animBad = animations;
        CHECK(chunkAt(animBad, pmesh::Clip, offset, size));
        patch32(animBad, offset + 12 + 4 + 4 + 4 + 4 + 4, 0xFFFFFFFFu);
        CHECK(!readPMeshAnimations(animBad.data(), animBad.size(), skeletons, clips, problem));
        ct::Vector<Skeleton> noSkeletons;
        ct::Vector<AnimationClip> orphan;
        CHECK(!readPMeshAnimations(animations.data(), animations.size(), noSkeletons, orphan,
                problem));
        CHECK(orphan.empty());
    }
}

void testTruncation()
{
    MemoryImporter source;
    ModelData model;
    buildRichModel(model, source);
    const ct::String directory = under(kRoot, "truncation");
    CHECK(fs_create_directory(directory.c_str()));
    ct::String error;
    CHECK(savePMesh(model, source, under(directory.c_str(), "rich.pmesh"), PMeshSaveOptions(),
            error));
    PMeshFiles files;
    CHECK(buildPMesh(model, source, "rich", PMeshSaveOptions(), files, error));
    const ct::String name = under(directory.c_str(), "rich.pmesh");
    PMeshImporter importer;

    std::size_t meshAccepted = 0;
    std::size_t meshEmptyError = 0;
    for (std::size_t length = 0; length < files.mesh.size(); ++length)
    {
        ModelData loaded;
        const bool ok = importer.importMemory(files.mesh.data(), length, name, loaded);
        if (ok) ++meshAccepted;
        if (!ok && loaded.error.empty()) ++meshEmptyError;
    }
    CHECK(meshAccepted == 0);
    CHECK(meshEmptyError == 0);
    {
        ModelData loaded;
        CHECK(readBack(files.mesh, name.c_str(), importer, loaded));
    }

    Skeleton parsed;
    CHECK(model.skeletons.size() == 1);
    std::size_t skeletonAccepted = 0;
    for (std::size_t length = 0; length < files.skeletons[0].size(); ++length)
    {
        ct::String problem;
        Skeleton out;
        if (readPMeshSkeleton(files.skeletons[0].data(), length, out, problem)) ++skeletonAccepted;
        else if (problem.empty())
            ++skeletonAccepted;
    }
    CHECK(skeletonAccepted == 0);
    std::size_t animationAccepted = 0;
    for (std::size_t length = 0; length < files.animations.size(); ++length)
    {
        ct::String problem;
        ct::Vector<AnimationClip> out;
        if (readPMeshAnimations(files.animations.data(), length, model.skeletons, out, problem) ||
                problem.empty() || !out.empty())
            ++animationAccepted;
    }
    CHECK(animationAccepted == 0);
    printf("truncation: mesh %zu, skeleton %zu, animation %zu lengths all rejected\n",
            files.mesh.size(), files.skeletons[0].size(), files.animations.size());
}

struct Fixture
{
    const char* path;
    bool animated;
};

void loadFixture(ModelLoader& loader, const char* relative, ModelData& out, bool& ok)
{
    const ct::String path = ct::String(kAssets) + "/" + relative;
    ok = loader.load(path, out);
    CHECK(ok);
}

void poseCheck(const ModelData& a, const ModelData& b, std::size_t& compared)
{
    for (std::size_t c = 0; c < a.animations.size() && c < b.animations.size(); ++c)
    {
        const AnimationClip& x = a.animations[c];
        const AnimationClip& y = b.animations[c];
        const int skeleton = x.skeleton();
        if (skeleton < 0 || static_cast<std::size_t>(skeleton) >= a.skeletons.size()) continue;
        if (x.tracks().empty()) continue;
        const Skeleton& sx = a.skeletons[static_cast<std::size_t>(skeleton)];
        const Skeleton& sy = b.skeletons[static_cast<std::size_t>(skeleton)];
        const float times[2] = { x.duration() * 0.37f, x.duration() * 0.81f };
        for (float time: times)
        {
            ct::Vector<LocalPose> poseX;
            ct::Vector<LocalPose> poseY;
            sx.bindPose(poseX);
            sy.bindPose(poseY);
            x.sample(time, poseX);
            y.sample(time, poseY);
            ct::Vector<Math::Mat4> globalX;
            ct::Vector<Math::Mat4> paletteX;
            ct::Vector<Math::Mat4> globalY;
            ct::Vector<Math::Mat4> paletteY;
            sx.evaluate(poseX, globalX, paletteX);
            sy.evaluate(poseY, globalY, paletteY);
            CHECK(paletteX.size() == sx.boneCount() && paletteY.size() == paletteX.size());
            bool same = paletteX.size() == paletteY.size();
            for (std::size_t i = 0; same && i < paletteX.size(); ++i)
                same = sameMat(paletteX[i], paletteY[i]) && sameMat(globalX[i], globalY[i]);
            CHECK(same);
            ++compared;
        }
    }
}

void testLargeTruncation(const ct::String& path)
{
    ct::Vector<std::uint8_t> bytes = FileSystem::getSingleton().readBinary(path);
    CHECK(!bytes.empty());
    PMeshImporter importer;
    std::size_t tried = 0;
    std::size_t accepted = 0;
    for (std::size_t length = 0; length < bytes.size(); length += 997)
    {
        ModelData loaded;
        ++tried;
        if (importer.importMemory(bytes.data(), length, path, loaded)) ++accepted;
    }
    for (std::size_t back = 1; back <= 64 && back <= bytes.size(); ++back)
    {
        ModelData loaded;
        ++tried;
        if (importer.importMemory(bytes.data(), bytes.size() - back, path, loaded)) ++accepted;
    }
    CHECK(accepted == 0);
    printf("large file truncation: %zu of %zu bytes, %zu lengths tried, %zu accepted\n",
            bytes.size(), bytes.size(), tried, accepted);
}

void testFixtures()
{
    ModelLoader loader;
    loader.addImporter(new GltfImporter());
    loader.addImporter(new OgreMeshImporter());
    loader.addImporter(new ObjImporter());
    loader.addImporter(new PMeshImporter());
    const char* const fixtures[] = { "sinbad/Sinbad.mesh", "sinbad/Sword.mesh",
        "DamagedHelmet/DamagedHelmet.gltf", "BusterDrone/scene.gltf",
        "FlightHelmet/FlightHelmet.gltf", "shader_ball/shader_ball.gltf",
        "shader_ball/shader_ball.obj", "sponza/sponza.obj" };
    std::size_t posesCompared = 0;
    for (const char* relative: fixtures)
    {
        ModelData original;
        bool ok = false;
        loadFixture(loader, relative, original, ok);
        if (!ok) continue;
        ModelImporter* importer = loader.importerFor(ct::String(relative));
        CHECK(importer != nullptr);
        if (!importer) continue;

        ct::String stem;
        for (const char* c = relative; *c; ++c) stem.push_back(*c == '/' || *c == '.' ? '_' : *c);
        const ct::String directory = under(kRoot, "fixtures");
        CHECK(fs_create_directory(directory.c_str()));
        const ct::String target = directory + "/" + stem + ".pmesh";
        ct::String error;
        const bool saved = savePMesh(original, *importer, target, PMeshSaveOptions(), error);
        CHECK(saved);
        if (!saved)
        {
            printf("  %s: %s\n", relative, error.c_str());
            continue;
        }
        ModelData back;
        PMeshImporter reader;
        CHECK(reader.import(target, back));
        const int differences = modelDifferences(original, *importer, back, reader);
        CHECK(differences == 0);
        printf("%s: %zu nodes, %zu meshes, %u primitives, %zu skeletons, %zu animations, "
               "%d differences\n",
                relative, original.nodes.size(), original.meshes.size(), original.primitiveCount,
                original.skeletons.size(), original.animations.size(), differences);
        poseCheck(original, back, posesCompared);
        if (strcmp(relative, "sinbad/Sinbad.mesh") == 0) testLargeTruncation(target);
    }
    CHECK(posesCompared >= 2);
    printf("poses compared: %zu\n", posesCompared);
}

} // namespace

int main()
{
    CHECK(fs_create_directory(kRoot));
    testAttributeCombinations();
    testNoIndicesAndNoMaterial();
    testIndexWidth();
    testWriterRejects();
    testRichRoundTrip();
    testContainerRules();
    testCorruption();
    testTruncation();
    testFixtures();
    return failures ? 1 : 0;
}
