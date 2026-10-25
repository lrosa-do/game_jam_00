#include "Check.h"

#include "prisma3d/import/C3DSImporter.h"

#include "platform.h"

#include <math.h>
#include <string.h>

using namespace prisma3d;

namespace
{

const float kTolerance = 1e-4f;
const char* const kDirectory = PRISMA3D_TEST_DIR "/c3ds";

typedef ct::Vector<std::uint8_t> Bytes;

bool near(float a, float b)
{
    return fabsf(a - b) <= kTolerance;
}

bool nearVec3(const Math::Vec3& a, float x, float y, float z)
{
    return near(a.x, x) && near(a.y, y) && near(a.z, z);
}

bool nearVec2(const Math::Vec2& a, float x, float y)
{
    return near(a.x, x) && near(a.y, y);
}

ct::String pathOf(const char* name)
{
    return ct::String(kDirectory) + "/" + name;
}

bool writeFile(const char* name, const Bytes& bytes)
{
    return file_write(pathOf(name).c_str(), bytes.data(), bytes.size());
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

void putString(Bytes& bytes, const char* text)
{
    for (const char* c = text; *c; ++c) bytes.push_back(static_cast<std::uint8_t>(*c));
    bytes.push_back(0);
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

void putVertices(Bytes& bytes, const float (*positions)[3], std::uint16_t count)
{
    const std::size_t chunk = beginChunk(bytes, 0x4110);
    putU16(bytes, count);
    for (std::uint16_t i = 0; i < count; ++i)
    {
        putF32(bytes, positions[i][0]);
        putF32(bytes, positions[i][1]);
        putF32(bytes, positions[i][2]);
    }
    endChunk(bytes, chunk);
}

void putUvs(Bytes& bytes, const float (*uvs)[2], std::uint16_t count)
{
    const std::size_t chunk = beginChunk(bytes, 0x4140);
    putU16(bytes, count);
    for (std::uint16_t i = 0; i < count; ++i)
    {
        putF32(bytes, uvs[i][0]);
        putF32(bytes, uvs[i][1]);
    }
    endChunk(bytes, chunk);
}

void putFaceGroup(Bytes& bytes, const char* material, const std::uint16_t* faces,
        std::uint16_t count)
{
    const std::size_t chunk = beginChunk(bytes, 0x4130);
    putString(bytes, material);
    putU16(bytes, count);
    for (std::uint16_t i = 0; i < count; ++i) putU16(bytes, faces[i]);
    endChunk(bytes, chunk);
}

struct GroupSpec
{
    const char* material;
    std::uint16_t faces[4];
    std::uint16_t count;
};

void putFaces(Bytes& bytes, const std::uint16_t (*faces)[3], std::uint16_t count,
        const GroupSpec* groups, std::size_t groupCount)
{
    const std::size_t chunk = beginChunk(bytes, 0x4120);
    putU16(bytes, count);
    for (std::uint16_t i = 0; i < count; ++i)
    {
        putU16(bytes, faces[i][0]);
        putU16(bytes, faces[i][1]);
        putU16(bytes, faces[i][2]);
        putU16(bytes, 0x0007);
    }
    for (std::size_t g = 0; g < groupCount; ++g)
        putFaceGroup(bytes, groups[g].material, groups[g].faces, groups[g].count);
    endChunk(bytes, chunk);
}

const float kBoxPositions[4][3] = { { 0, 0, 0 }, { 1, 0, 0 }, { 1, 1, 0 }, { 0, 1, 0 } };
const float kBoxUvs[4][2] = { { 0, 0 }, { 1, 0 }, { 1, 1 }, { 0, 1 } };
const std::uint16_t kBoxFaces[2][3] = { { 0, 1, 2 }, { 0, 2, 3 } };
const float kPairPositions[4][3] = { { 0, 0, 0 }, { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };
const std::uint16_t kPairFaces[2][3] = { { 0, 1, 2 }, { 0, 2, 3 } };
const float kTriPositions[3][3] = { { 0, 0, 0 }, { 0, 1, 0 }, { 1, 0, 0 } };
const std::uint16_t kTriFaces[1][3] = { { 0, 1, 2 } };

void buildModel(Bytes& bytes, std::uint16_t mainId, std::uint16_t badFaceVertex, bool withObjects)
{
    bytes.clear();
    const std::size_t main = beginChunk(bytes, mainId);

    const std::size_t version = beginChunk(bytes, 0x0002);
    putU32(bytes, 3);
    endChunk(bytes, version);

    const std::size_t edit = beginChunk(bytes, 0x3D3D);

    const std::size_t meshVersion = beginChunk(bytes, 0x3D3E);
    putU32(bytes, 3);
    endChunk(bytes, meshVersion);

    const std::size_t material = beginChunk(bytes, 0xAFFF);
    const std::size_t name = beginChunk(bytes, 0xA000);
    putString(bytes, "paint");
    endChunk(bytes, name);
    const std::size_t diffuse = beginChunk(bytes, 0xA020);
    const std::size_t trueColor = beginChunk(bytes, 0x0011);
    putU8(bytes, 255);
    putU8(bytes, 128);
    putU8(bytes, 0);
    endChunk(bytes, trueColor);
    endChunk(bytes, diffuse);
    const std::size_t texMap = beginChunk(bytes, 0xA200);
    const std::size_t percent = beginChunk(bytes, 0x0030);
    putU16(bytes, 100);
    endChunk(bytes, percent);
    const std::size_t mapFile = beginChunk(bytes, 0xA300);
    putString(bytes, "skin.bmp");
    endChunk(bytes, mapFile);
    endChunk(bytes, texMap);
    endChunk(bytes, material);

    const std::size_t second = beginChunk(bytes, 0xAFFF);
    const std::size_t secondName = beginChunk(bytes, 0xA000);
    putString(bytes, "glow");
    endChunk(bytes, secondName);
    const std::size_t secondDiffuse = beginChunk(bytes, 0xA020);
    const std::size_t floatColor = beginChunk(bytes, 0x0010);
    putF32(bytes, 0.25f);
    putF32(bytes, 0.5f);
    putF32(bytes, 0.75f);
    endChunk(bytes, floatColor);
    endChunk(bytes, secondDiffuse);
    endChunk(bytes, second);

    if (withObjects)
    {
        const std::size_t box = beginChunk(bytes, 0x4000);
        putString(bytes, "box");
        const std::size_t boxMesh = beginChunk(bytes, 0x4100);
        putVertices(bytes, kBoxPositions, 4);
        putUvs(bytes, kBoxUvs, 4);
        const GroupSpec boxGroups[1] = { { "paint", { 0, 1, 0, 0 }, 2 } };
        std::uint16_t faces[2][3] = { { kBoxFaces[0][0], kBoxFaces[0][1], kBoxFaces[0][2] },
            { kBoxFaces[1][0], kBoxFaces[1][1], kBoxFaces[1][2] } };
        faces[1][2] = badFaceVertex ? badFaceVertex : faces[1][2];
        putFaces(bytes, faces, 2, boxGroups, 1);
        const std::size_t matrix = beginChunk(bytes, 0x4160);
        for (int i = 0; i < 12; ++i) putF32(bytes, 0.0f);
        endChunk(bytes, matrix);
        endChunk(bytes, boxMesh);
        endChunk(bytes, box);

        const std::size_t pair = beginChunk(bytes, 0x4000);
        putString(bytes, "pair");
        const std::size_t pairMesh = beginChunk(bytes, 0x4100);
        putVertices(bytes, kPairPositions, 4);
        const GroupSpec pairGroups[2] = { { "glow", { 0, 0, 0, 0 }, 1 },
            { "missing", { 1, 0, 0, 0 }, 1 } };
        putFaces(bytes, kPairFaces, 2, pairGroups, 2);
        endChunk(bytes, pairMesh);
        endChunk(bytes, pair);

        const std::size_t tri = beginChunk(bytes, 0x4000);
        putString(bytes, "tri");
        const std::size_t triMesh = beginChunk(bytes, 0x4100);
        putVertices(bytes, kTriPositions, 3);
        putFaces(bytes, kTriFaces, 1, nullptr, 0);
        endChunk(bytes, triMesh);
        endChunk(bytes, tri);

        const std::size_t light = beginChunk(bytes, 0x4000);
        putString(bytes, "lamp");
        const std::size_t lightData = beginChunk(bytes, 0x4600);
        putF32(bytes, 1.0f);
        endChunk(bytes, lightData);
        endChunk(bytes, light);
    }

    endChunk(bytes, edit);
    const std::size_t keyframes = beginChunk(bytes, 0xB000);
    putU32(bytes, 1);
    endChunk(bytes, keyframes);
    endChunk(bytes, main);
}

void checkModel(const ModelImporter& importer, const ModelData& model, const char* rootName)
{
    CHECK(model.error.empty());
    CHECK(model.nodes.size() == 4);
    CHECK(model.roots.size() == 1 && model.roots[0] == 0);
    CHECK(model.meshes.size() == 3);
    CHECK(model.primitiveCount == 4);
    CHECK(model.materials.size() == 4);
    CHECK(model.skeletons.empty());
    CHECK(model.animations.empty());
    if (model.nodes.size() != 4 || model.meshes.size() != 3 || model.materials.size() != 4) return;

    CHECK(model.nodes[0].name == rootName);
    CHECK(model.nodes[0].mesh == -1);
    CHECK(model.nodes[0].parent == -1);
    CHECK(model.nodes[0].children.size() == 3);
    const char* const names[3] = { "box", "pair", "tri" };
    for (std::size_t i = 0; i < 3; ++i)
    {
        CHECK(model.nodes[i + 1].name == names[i]);
        CHECK(model.nodes[i + 1].parent == 0);
        CHECK(model.nodes[i + 1].mesh == static_cast<int>(i));
        CHECK(model.meshes[i].name == names[i]);
        if (model.nodes[0].children.size() == 3)
            CHECK(model.nodes[0].children[i] == static_cast<int>(i + 1));
    }

    CHECK(model.materials[0].name == "paint");
    CHECK(near(model.materials[0].baseColor.x, 1.0f));
    CHECK(near(model.materials[0].baseColor.y, 128.0f / 255.0f));
    CHECK(near(model.materials[0].baseColor.z, 0.0f));
    CHECK(near(model.materials[0].baseColor.w, 1.0f));
    CHECK(model.materials[0].baseColorTexture == 0);
    CHECK(model.materials[1].name == "glow");
    CHECK(near(model.materials[1].baseColor.x, 0.25f));
    CHECK(near(model.materials[1].baseColor.y, 0.5f));
    CHECK(near(model.materials[1].baseColor.z, 0.75f));
    CHECK(model.materials[1].baseColorTexture == -1);
    CHECK(model.materials[2].name == "missing");
    CHECK(near(model.materials[2].baseColor.x, 1.0f));
    CHECK(model.materials[2].baseColorTexture == -1);
    CHECK(model.materials[3].name == "default");
    CHECK(near(model.materials[3].baseColor.x, 1.0f));
    CHECK(model.materials[3].baseColorTexture == -1);

    CHECK(model.textures.size() == 1);
    CHECK(model.images.size() == 1);
    if (model.textures.size() == 1 && model.images.size() == 1)
    {
        CHECK(model.textures[0].image == 0);
        CHECK(model.images[0].path == pathOf("skin.bmp"));
        CHECK(model.images[0].name == "skin.bmp");
        ct::Vector<std::uint8_t> pixels;
        pixels.push_back(1);
        CHECK(!importer.readImage(model, model.images[0], pixels));
        CHECK(pixels.empty());
    }

    CHECK(model.meshes[0].primitives.size() == 1);
    CHECK(model.meshes[1].primitives.size() == 2);
    CHECK(model.meshes[2].primitives.size() == 1);
    if (model.meshes[0].primitives.size() != 1 || model.meshes[1].primitives.size() != 2 ||
            model.meshes[2].primitives.size() != 1)
        return;

    const ModelPrimitiveData& box = model.meshes[0].primitives[0];
    CHECK(box.material == 0);
    CHECK(box.vertexCount == 4);
    CHECK(box.indexCount == 6);
    CHECK(box.hasAabb);
    CHECK((box.attributeMask & kVertexNormal) != 0);
    CHECK((box.attributeMask & kVertexUv) != 0);
    SurfaceData surface;
    CHECK(importer.readPrimitive(model, box, surface));
    CHECK(surface.positions.size() == 4 && surface.normals.size() == 4 && surface.uvs.size() == 4);
    CHECK(surface.indices.size() == 6);
    if (surface.positions.size() == 4 && surface.normals.size() == 4 && surface.uvs.size() == 4 &&
            surface.indices.size() == 6)
    {
        const float uvs[4][2] = { { 0, 1 }, { 1, 1 }, { 1, 0 }, { 0, 0 } };
        const std::uint32_t indices[6] = { 0, 1, 2, 0, 2, 3 };
        for (std::size_t i = 0; i < 4; ++i)
        {
            CHECK(nearVec3(surface.positions[i], kBoxPositions[i][0], kBoxPositions[i][1],
                    kBoxPositions[i][2]));
            CHECK(nearVec3(surface.normals[i], 0.0f, 0.0f, 1.0f));
            CHECK(nearVec2(surface.uvs[i], uvs[i][0], uvs[i][1]));
        }
        for (std::size_t i = 0; i < 6; ++i) CHECK(surface.indices[i] == indices[i]);
    }

    const ModelPrimitiveData& glow = model.meshes[1].primitives[0];
    const ModelPrimitiveData& missing = model.meshes[1].primitives[1];
    CHECK(glow.material == 1);
    CHECK(missing.material == 2);
    CHECK(glow.vertexCount == 3 && glow.indexCount == 3);
    CHECK(missing.vertexCount == 3 && missing.indexCount == 3);
    CHECK((glow.attributeMask & kVertexUv) == 0);
    SurfaceData first;
    SurfaceData other;
    CHECK(importer.readPrimitive(model, glow, first));
    CHECK(importer.readPrimitive(model, missing, other));
    CHECK(first.uvs.empty());
    if (first.normals.size() == 3 && other.normals.size() == 3)
    {
        for (std::size_t i = 0; i < 3; ++i) CHECK(nearVec3(first.normals[i], 0.0f, 0.0f, 1.0f));
        for (std::size_t i = 0; i < 3; ++i) CHECK(nearVec3(other.normals[i], 1.0f, 0.0f, 0.0f));
        CHECK(nearVec3(other.positions[0], 0, 0, 0));
        CHECK(nearVec3(other.positions[1], 0, 1, 0));
        CHECK(nearVec3(other.positions[2], 0, 0, 1));
    }
    else
        CHECK(false);

    const ModelPrimitiveData& tri = model.meshes[2].primitives[0];
    CHECK(tri.material == 3);
    CHECK(tri.indexCount == 3);
    SurfaceData triangle;
    CHECK(importer.readPrimitive(model, tri, triangle));
    CHECK(triangle.normals.size() == 3);
    if (triangle.normals.size() == 3)
    {
        for (std::size_t i = 0; i < 3; ++i) CHECK(nearVec3(triangle.normals[i], 0.0f, 0.0f, -1.0f));
    }

    ModelPrimitiveData invalid = tri;
    invalid.source = 99;
    CHECK(!importer.readPrimitive(model, invalid, surface));
}

void checkRejected(C3DSImporter& importer, const Bytes& bytes, const char* name)
{
    ModelData model;
    CHECK(!importer.importMemory(bytes.data(), bytes.size(), pathOf(name), model));
    CHECK(!model.error.empty());
}

} // namespace

int main()
{
    CHECK(fs_create_directory(kDirectory));

    Bytes bytes;
    buildModel(bytes, 0x4D4D, 0, true);
    CHECK(writeFile("model.3ds", bytes));

    C3DSImporter importer;
    CHECK(strcmp(importer.name(), "3D Studio 3DS") == 0);
    CHECK(importer.supports("3ds"));
    CHECK(!importer.supports("obj"));
    CHECK(!importer.supports(nullptr));

    ModelData model;
    CHECK(importer.import(pathOf("model.3ds"), model));
    CHECK(model.path == pathOf("model.3ds"));
    CHECK(model.directory == pathOf(""));
    checkModel(importer, model, "model");

    ModelData memoryModel;
    CHECK(importer.importMemory(bytes.data(), bytes.size(), pathOf("memory.3ds"), memoryModel));
    checkModel(importer, memoryModel, "memory");

    ModelLoader loader;
    loader.addImporter(new C3DSImporter());
    CHECK(loader.importerFor(pathOf("model.3ds")) != nullptr);
    ModelData loaded;
    CHECK(loader.load(pathOf("model.3ds"), loaded));
    checkModel(*loader.importerFor(pathOf("model.3ds")), loaded, "model");
    ModelData loadedMemory;
    CHECK(loader.loadFromMemory(bytes.data(), bytes.size(), pathOf("loader.3DS"), loadedMemory));
    checkModel(*loader.importer(0), loadedMemory, "loader");

    for (std::size_t length = 1; length < bytes.size(); ++length)
    {
        ModelData partial;
        CHECK(!importer.importMemory(bytes.data(), length, pathOf("partial.3ds"), partial));
        CHECK(!partial.error.empty());
    }

    for (std::size_t length = 7; length < bytes.size(); length += 3)
    {
        Bytes patched;
        for (std::size_t i = 0; i < length; ++i) patched.push_back(bytes[i]);
        patchLength(patched, 0, static_cast<std::uint32_t>(length));
        ModelData partial;
        const bool ok =
                importer.importMemory(patched.data(), patched.size(), pathOf("cut.3ds"), partial);
        CHECK(ok == partial.error.empty());
        CHECK(!ok || partial.primitiveCount > 0);
    }

    Bytes truncated;
    for (std::size_t i = 0; i < bytes.size() / 2; ++i) truncated.push_back(bytes[i]);
    CHECK(writeFile("short.3ds", truncated));
    ModelData failed;
    CHECK(!importer.import(pathOf("short.3ds"), failed));
    CHECK(!failed.error.empty());

    Bytes wrongId;
    buildModel(wrongId, 0x4D4E, 0, true);
    checkRejected(importer, wrongId, "wrong.3ds");

    Bytes badVertex;
    buildModel(badVertex, 0x4D4D, 9, true);
    checkRejected(importer, badVertex, "badvertex.3ds");

    Bytes noObjects;
    buildModel(noObjects, 0x4D4D, 0, false);
    checkRejected(importer, noObjects, "noobjects.3ds");

    Bytes shortLength = bytes;
    patchLength(shortLength, 0, 3);
    checkRejected(importer, shortLength, "shortlength.3ds");

    Bytes oversized = bytes;
    patchLength(oversized, 0, static_cast<std::uint32_t>(bytes.size() + 1));
    checkRejected(importer, oversized, "oversized.3ds");

    Bytes header;
    putU16(header, 0x4D4D);
    checkRejected(importer, header, "header.3ds");

    CHECK(!importer.importMemory(nullptr, 0, pathOf("empty.3ds"), failed));
    CHECK(!failed.error.empty());
    CHECK(!importer.import(pathOf("absent.3ds"), failed));
    CHECK(!failed.error.empty());

    if (failures == 0) printf("c3ds: ok\n");
    return failures == 0 ? 0 : 1;
}
