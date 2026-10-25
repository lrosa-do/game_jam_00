#include "Check.h"

#include "prisma3d/import/GltfImporter.h"

#include "platform.h"

#include <math.h>
#include <string.h>

using namespace prisma3d;

namespace
{

GltfImporter gImporter;

const float kTolerance = 1e-4f;
const std::uint32_t kBinSize = 216;
const std::uint32_t kImageOffset = 200;
const std::uint32_t kImageSize = 16;

bool near(float a, float b)
{
    return fabsf(a - b) <= kTolerance;
}

bool nearVec3(const Math::Vec3& a, float x, float y, float z)
{
    return near(a.x, x) && near(a.y, y) && near(a.z, z);
}

void putU32(ct::Vector<std::uint8_t>& bytes, std::uint32_t value)
{
    for (int i = 0; i < 4; ++i) bytes.push_back(static_cast<std::uint8_t>(value >> (8 * i)));
}

void putU16(ct::Vector<std::uint8_t>& bytes, std::uint16_t value)
{
    bytes.push_back(static_cast<std::uint8_t>(value));
    bytes.push_back(static_cast<std::uint8_t>(value >> 8));
}

void putF32(ct::Vector<std::uint8_t>& bytes, float value)
{
    std::uint32_t raw;
    memcpy(&raw, &value, sizeof(raw));
    putU32(bytes, raw);
}

void putVec3(ct::Vector<std::uint8_t>& bytes, float x, float y, float z)
{
    putF32(bytes, x);
    putF32(bytes, y);
    putF32(bytes, z);
}

void buildBin(ct::Vector<std::uint8_t>& bin)
{
    bin.clear();
    putVec3(bin, 0.0f, 0.0f, 0.0f);
    putVec3(bin, 1.0f, 0.0f, 0.0f);
    putVec3(bin, 0.0f, 1.0f, 0.0f);

    const float uvs[3][2] = { { 0.0f, 0.0f }, { 1.0f, 0.0f }, { 0.0f, 1.0f } };
    for (int i = 0; i < 3; ++i)
    {
        putVec3(bin, 0.0f, 0.0f, 1.0f);
        putF32(bin, uvs[i][0]);
        putF32(bin, uvs[i][1]);
    }

    const std::uint8_t colors[3][4] = { { 255, 128, 0, 255 }, { 0, 255, 128, 255 },
        { 10, 20, 30, 40 } };
    for (int i = 0; i < 3; ++i)
        for (int c = 0; c < 4; ++c) bin.push_back(colors[i][c]);

    const std::uint16_t uv2[3][2] = { { 0, 65535 }, { 65535, 0 }, { 32768, 16384 } };
    for (int i = 0; i < 3; ++i)
    {
        putU16(bin, uv2[i][0]);
        putU16(bin, uv2[i][1]);
    }

    putU16(bin, 0);
    putU16(bin, 1);
    putU16(bin, 2);
    putU16(bin, 0);

    putVec3(bin, 0.0f, 0.0f, 0.0f);
    putVec3(bin, 2.0f, 0.0f, 0.0f);
    putVec3(bin, 2.0f, 2.0f, 0.0f);
    putVec3(bin, 0.0f, 2.0f, 0.0f);

    const std::uint32_t indices[6] = { 0, 1, 2, 0, 2, 3 };
    for (int i = 0; i < 6; ++i) putU32(bin, indices[i]);

    for (std::uint32_t i = 0; i < kImageSize; ++i)
        bin.push_back(static_cast<std::uint8_t>(0xA0 + i));
}

ct::String buildJson(bool glb)
{
    ct::String json =
            "{\"asset\":{\"version\":\"2.0\"},"
            "\"scene\":0,\"scenes\":[{\"nodes\":[0]}],"
            "\"nodes\":["
            "{\"name\":\"A\",\"translation\":[1,2,3],\"children\":[1,2]},"
            "{\"name\":\"B\",\"rotation\":[0,0.70710678,0,0.70710678],\"scale\":[2,2,2],\"mesh\":0}"
            ","
            "{\"name\":\"C\",\"mesh\":1,\"camera\":0}],"
            "\"cameras\":[{\"type\":\"perspective\",\"perspective\":{\"yfov\":0.8,\"aspectRatio\":"
            "1.5,"
            "\"znear\":0.25}}],"
            "\"meshes\":["
            "{\"name\":\"full\",\"primitives\":[{\"attributes\":{\"POSITION\":0,\"NORMAL\":1,"
            "\"TEXCOORD_0\":2,\"COLOR_0\":3,\"TEXCOORD_1\":4},\"indices\":5,\"material\":0}]},"
            "{\"name\":\"plain\",\"primitives\":[{\"attributes\":{\"POSITION\":6},\"indices\":7,"
            "\"material\":1}]}],"
            "\"materials\":["
            "{\"name\":\"wall\",\"pbrMetallicRoughness\":{\"baseColorFactor\":[0.5,0.25,1,0.75],"
            "\"metallicFactor\":0.2,\"roughnessFactor\":0.8,\"baseColorTexture\":{\"index\":0}},"
            "\"emissiveFactor\":[0.1,0.2,0.3],\"alphaMode\":\"MASK\",\"alphaCutoff\":0.3,"
            "\"doubleSided\":true},"
            "{}],"
            "\"textures\":[{\"sampler\":0,\"source\":0}],"
            "\"samplers\":[{\"wrapS\":33071,\"wrapT\":10497,\"magFilter\":9728}],";
    if (glb) json += "\"images\":[{\"name\":\"tex\",\"bufferView\":7,\"mimeType\":\"image/png\"}],";
    else
        json += "\"images\":[{\"name\":\"tex\",\"uri\":\"tex.png\"}],";
    json += glb ? "\"buffers\":[{\"byteLength\":216}],"
                : "\"buffers\":[{\"uri\":\"model%20data.bin\",\"byteLength\":216}],";
    json += "\"bufferViews\":["
            "{\"buffer\":0,\"byteOffset\":0,\"byteLength\":36},"
            "{\"buffer\":0,\"byteOffset\":36,\"byteLength\":60,\"byteStride\":20},"
            "{\"buffer\":0,\"byteOffset\":96,\"byteLength\":12},"
            "{\"buffer\":0,\"byteOffset\":108,\"byteLength\":12},"
            "{\"buffer\":0,\"byteOffset\":120,\"byteLength\":6},"
            "{\"buffer\":0,\"byteOffset\":128,\"byteLength\":48},"
            "{\"buffer\":0,\"byteOffset\":176,\"byteLength\":24},"
            "{\"buffer\":0,\"byteOffset\":200,\"byteLength\":16}],"
            "\"accessors\":["
            "{\"bufferView\":0,\"componentType\":5126,\"count\":3,\"type\":\"VEC3\","
            "\"min\":[0,0,0],\"max\":[1,1,0]},"
            "{\"bufferView\":1,\"componentType\":5126,\"count\":3,\"type\":\"VEC3\"},"
            "{\"bufferView\":1,\"byteOffset\":12,\"componentType\":5126,\"count\":3,\"type\":"
            "\"VEC2\"},"
            "{\"bufferView\":2,\"componentType\":5121,\"normalized\":true,\"count\":3,\"type\":"
            "\"VEC4\"},"
            "{\"bufferView\":3,\"componentType\":5123,\"normalized\":true,\"count\":3,\"type\":"
            "\"VEC2\"},"
            "{\"bufferView\":4,\"componentType\":5123,\"count\":3,\"type\":\"SCALAR\"},"
            "{\"bufferView\":5,\"componentType\":5126,\"count\":4,\"type\":\"VEC3\"},"
            "{\"bufferView\":6,\"componentType\":5125,\"count\":6,\"type\":\"SCALAR\"}]}";
    return json;
}

void buildGlb(const ct::String& json, const ct::Vector<std::uint8_t>& bin,
        ct::Vector<std::uint8_t>& glb)
{
    glb.clear();
    const std::uint32_t jsonLength =
            static_cast<std::uint32_t>((json.size() + 3) & ~std::size_t(3));
    const std::uint32_t binLength = static_cast<std::uint32_t>((bin.size() + 3) & ~std::size_t(3));
    putU32(glb, 0x46546C67u);
    putU32(glb, 2);
    putU32(glb, 12 + 8 + jsonLength + 8 + binLength);
    putU32(glb, jsonLength);
    putU32(glb, 0x4E4F534Au);
    for (std::size_t i = 0; i < json.size(); ++i) glb.push_back(static_cast<std::uint8_t>(json[i]));
    for (std::size_t i = json.size(); i < jsonLength; ++i) glb.push_back(' ');
    putU32(glb, binLength);
    putU32(glb, 0x004E4942u);
    for (std::size_t i = 0; i < bin.size(); ++i) glb.push_back(bin[i]);
    for (std::size_t i = bin.size(); i < binLength; ++i) glb.push_back(0);
}

bool endsWith(const ct::String& text, const char* suffix)
{
    const std::size_t length = strlen(suffix);
    return text.size() >= length &&
           memcmp(text.c_str() + text.size() - length, suffix, length) == 0;
}

void checkAccessor(const GltfAccessorRef& ref, int buffer, std::uint64_t offset,
        std::uint32_t stride, std::uint32_t count, std::uint8_t components, GltfComponentType type,
        bool normalized)
{
    CHECK(ref.buffer == buffer);
    CHECK(ref.offset == offset);
    CHECK(ref.stride == stride);
    CHECK(ref.count == count);
    CHECK(ref.components == components);
    CHECK(ref.componentType == type);
    CHECK(ref.normalized == normalized);
    CHECK(ref.valid());
}

void checkDocument(const ModelData& doc, bool binary)
{
    CHECK(doc.error.empty());
    const GltfSource* src = GltfImporter::source(doc);
    CHECK(src != nullptr);
    if (!src) return;
    CHECK(src->binary == binary);
    CHECK(src->bufferPaths.size() == 1);
    CHECK(src->bufferSizes.size() == 1);
    CHECK(src->bufferSizes[0] == kBinSize);
    CHECK(src->primitives.size() == 2);
    CHECK(src->images.size() == 1);
    CHECK(doc.meshes.size() == 2);
    CHECK(doc.materials.size() == 2);
    CHECK(doc.textures.size() == 1);
    CHECK(doc.images.size() == 1);
    CHECK(doc.cameras.size() == 1);
    CHECK(doc.nodes.size() == 3);
    CHECK(doc.primitiveCount == 2);
    CHECK(doc.geometryBytes == 238);
    if (binary) CHECK(doc.imageBytes == kImageSize);
    else
        CHECK(doc.imageBytes == 0);
    if (doc.meshes.size() != 2 || doc.meshes[0].primitives.size() != 1 ||
            doc.meshes[1].primitives.size() != 1 || doc.nodes.size() != 3 ||
            doc.materials.size() != 2 || doc.textures.size() != 1 || doc.images.size() != 1 ||
            doc.cameras.size() != 1)
        return;

    const ModelPrimitiveData& full = doc.meshes[0].primitives[0];
    CHECK(full.topology == prisma::Topology::Triangles);
    CHECK(full.material == 0);
    CHECK(full.source < src->primitives.size());
    if (full.source >= src->primitives.size()) return;
    const GltfPrimitiveSource& fullSource = src->primitives[full.source];
    CHECK(full.vertexCount == 3);
    CHECK(full.indexCount == 3);
    // UVs and no tangents in the file: the loader generates them.
    CHECK(full.attributeMask == (kVertexPosition | kVertexNormal | kVertexTangent | kVertexColor |
                                        kVertexUv | kVertexUv2));
    checkAccessor(fullSource.positions, 0, 0, 12, 3, 3, GltfComponentType::Float, false);
    checkAccessor(fullSource.normals, 0, 36, 20, 3, 3, GltfComponentType::Float, false);
    checkAccessor(fullSource.uvs, 0, 48, 20, 3, 2, GltfComponentType::Float, false);
    checkAccessor(fullSource.colors, 0, 96, 4, 3, 4, GltfComponentType::UInt8, true);
    checkAccessor(fullSource.uvs2, 0, 108, 4, 3, 2, GltfComponentType::UInt16, true);
    checkAccessor(fullSource.indices, 0, 120, 2, 3, 1, GltfComponentType::UInt16, false);
    CHECK(!fullSource.tangents.valid());
    CHECK(full.hasAabb);
    CHECK(nearVec3(full.aabb.min, 0.0f, 0.0f, 0.0f));
    CHECK(nearVec3(full.aabb.max, 1.0f, 1.0f, 0.0f));
    CHECK(full.sourceBytes == 166);
    CHECK(fullSource.byteLength() == 166);

    const ModelPrimitiveData& plain = doc.meshes[1].primitives[0];
    CHECK(plain.material == 1);
    CHECK(plain.source < src->primitives.size());
    if (plain.source >= src->primitives.size()) return;
    const GltfPrimitiveSource& plainSource = src->primitives[plain.source];
    // No normals: flat ones, so each of the two triangles gets its own three corners.
    CHECK(plain.vertexCount == 6);
    CHECK(plain.indexCount == 6);
    CHECK(plain.attributeMask == (kVertexPosition | kVertexNormal));
    checkAccessor(plainSource.positions, 0, 128, 12, 4, 3, GltfComponentType::Float, false);
    checkAccessor(plainSource.indices, 0, 176, 4, 6, 1, GltfComponentType::UInt32, false);
    CHECK(!plainSource.normals.valid());
    CHECK(!plain.hasAabb);
    CHECK(plain.sourceBytes == 72);

    const ModelMaterialData& wall = doc.materials[0];
    CHECK(wall.name == "wall");
    CHECK(near(wall.baseColor.x, 0.5f) && near(wall.baseColor.y, 0.25f));
    CHECK(near(wall.baseColor.z, 1.0f) && near(wall.baseColor.w, 0.75f));
    CHECK(near(wall.metallic, 0.2f));
    CHECK(near(wall.roughness, 0.8f));
    CHECK(nearVec3(wall.emissive, 0.1f, 0.2f, 0.3f));
    CHECK(wall.alphaMode == ModelAlphaMode::Mask);
    CHECK(near(wall.alphaCutoff, 0.3f));
    CHECK(wall.doubleSided);
    CHECK(wall.baseColorTexture == 0);
    CHECK(wall.normalTexture == -1);
    CHECK(wall.emissiveTexture == -1);

    const ModelMaterialData& plainMaterial = doc.materials[1];
    CHECK(plainMaterial.alphaMode == ModelAlphaMode::Opaque);
    CHECK(near(plainMaterial.alphaCutoff, 0.5f));
    CHECK(!plainMaterial.doubleSided);
    CHECK(plainMaterial.baseColorTexture == -1);
    CHECK(near(plainMaterial.baseColor.x, 1.0f) && near(plainMaterial.baseColor.w, 1.0f));

    const ModelTextureData& texture = doc.textures[0];
    CHECK(texture.image == 0);
    CHECK(!texture.repeatU);
    CHECK(texture.repeatV);
    CHECK(texture.nearest);

    const ModelImageData& image = doc.images[0];
    CHECK(image.name == "tex");
    if (binary)
    {
        CHECK(image.embedded());
        CHECK(image.source < src->images.size());
        if (image.source < src->images.size())
        {
            CHECK(src->images[image.source].buffer == 0);
            CHECK(src->images[image.source].offset == kImageOffset);
        }
        CHECK(image.size == kImageSize);
        CHECK(image.mimeType == "image/png");
    }
    else
    {
        CHECK(!image.embedded());
        CHECK(image.path == doc.directory + "tex.png");
    }

    CHECK(doc.nodes[0].name == "A");
    CHECK(doc.nodes[0].parent == -1);
    CHECK(doc.nodes[0].children.size() == 2);
    CHECK(doc.nodes[0].children[0] == 1 && doc.nodes[0].children[1] == 2);
    CHECK(doc.nodes[1].parent == 0);
    CHECK(doc.nodes[2].parent == 0);
    CHECK(doc.nodes[1].mesh == 0 && doc.nodes[1].camera == -1);
    CHECK(doc.nodes[2].mesh == 1 && doc.nodes[2].camera == 0);
    CHECK(nearVec3(doc.nodes[0].translation, 1.0f, 2.0f, 3.0f));
    CHECK(nearVec3(doc.nodes[1].scale, 2.0f, 2.0f, 2.0f));
    CHECK(doc.roots.size() == 1 && doc.roots[0] == 0);

    const Math::Vec3 local = doc.nodeLocalTransform(1).TransformPoint(Math::Vec3(1.0f, 0.0f, 0.0f));
    CHECK(nearVec3(local, 0.0f, 0.0f, -2.0f));
    const Math::Vec3 global =
            doc.nodeGlobalTransform(1).TransformPoint(Math::Vec3(1.0f, 0.0f, 0.0f));
    CHECK(nearVec3(global, 1.0f, 2.0f, 1.0f));
    CHECK(nearVec3(doc.nodeGlobalTransform(2).GetTranslation(), 1.0f, 2.0f, 3.0f));
    CHECK(nearVec3(doc.nodeGlobalTransform(0).GetTranslation(), 1.0f, 2.0f, 3.0f));

    const ModelCameraData& camera = doc.cameras[0];
    CHECK(camera.perspective);
    CHECK(near(camera.yfov, 0.8f));
    CHECK(near(camera.aspect, 1.5f));
    CHECK(near(camera.zNear, 0.25f));
    CHECK(near(camera.zFar, 1000.0f));
}

void checkReads(const ModelData& doc)
{
    if (doc.meshes.size() != 2) return;
    const GltfSource* src = GltfImporter::source(doc);
    CHECK(src != nullptr);
    if (!src) return;

    SurfaceData surface;
    CHECK(gImporter.readPrimitive(doc, doc.meshes[0].primitives[0], surface));
    CHECK(surface.topology == prisma::Topology::Triangles);
    CHECK(surface.positions.size() == 3);
    CHECK(surface.normals.size() == 3);
    CHECK(surface.uvs.size() == 3);
    CHECK(surface.uvs2.size() == 3);
    CHECK(surface.colors.size() == 3);
    CHECK(surface.tangents.size() == 0);
    CHECK(surface.indices.size() == 3);
    if (surface.positions.size() == 3 && surface.normals.size() == 3 && surface.uvs.size() == 3 &&
            surface.uvs2.size() == 3 && surface.colors.size() == 3 && surface.indices.size() == 3)
    {
        CHECK(nearVec3(surface.positions[1], 1.0f, 0.0f, 0.0f));
        CHECK(nearVec3(surface.positions[2], 0.0f, 1.0f, 0.0f));
        CHECK(nearVec3(surface.normals[2], 0.0f, 0.0f, 1.0f));
        CHECK(near(surface.uvs[1].x, 1.0f) && near(surface.uvs[1].y, 0.0f));
        CHECK(near(surface.uvs[2].x, 0.0f) && near(surface.uvs[2].y, 1.0f));
        CHECK(near(surface.colors[0].x, 1.0f) && near(surface.colors[0].y, 128.0f / 255.0f));
        CHECK(near(surface.colors[0].z, 0.0f) && near(surface.colors[0].w, 1.0f));
        CHECK(near(surface.colors[2].w, 40.0f / 255.0f));
        CHECK(near(surface.uvs2[0].x, 0.0f) && near(surface.uvs2[0].y, 1.0f));
        CHECK(near(surface.uvs2[2].x, 32768.0f / 65535.0f));
        CHECK(near(surface.uvs2[2].y, 16384.0f / 65535.0f));
        CHECK(surface.indices[0] == 0 && surface.indices[1] == 1 && surface.indices[2] == 2);
    }

    CHECK(gImporter.readPrimitive(doc, doc.meshes[1].primitives[0], surface));
    // Flat normals: the corners of the triangles 0 1 2 and 0 2 3, each with the face normal.
    CHECK(surface.positions.size() == 6);
    CHECK(surface.normals.size() == 6);
    CHECK(surface.colors.size() == 0);
    CHECK(surface.uvs.size() == 0);
    CHECK(surface.indices.size() == 6);
    if (surface.positions.size() == 6 && surface.normals.size() == 6 && surface.indices.size() == 6)
    {
        CHECK(nearVec3(surface.positions[2], 2.0f, 2.0f, 0.0f));
        CHECK(nearVec3(surface.positions[4], 2.0f, 2.0f, 0.0f));
        CHECK(nearVec3(surface.positions[5], 0.0f, 2.0f, 0.0f));
        for (int i = 0; i < 6; ++i)
        {
            CHECK(surface.indices[i] == static_cast<std::uint32_t>(i));
            CHECK(near(fabsf(surface.normals[i].z), 1.0f));
        }
    }

    ct::Vector<std::uint8_t> bytes;
    CHECK(gImporter.readImage(doc, doc.images[0], bytes));
    CHECK(bytes.size() == (doc.images[0].embedded() ? kImageSize : 7));
    if (doc.images[0].embedded())
    {
        CHECK(bytes.size() == kImageSize);
        for (std::size_t i = 0; i < bytes.size(); ++i) CHECK(bytes[i] == 0xA0 + i);
    }
    else
        CHECK(memcmp(bytes.data(), "PNGDATA", 7) == 0);

    ModelPrimitiveData invalid = doc.meshes[0].primitives[0];
    invalid.source = static_cast<std::uint32_t>(src->primitives.size());
    CHECK(!gImporter.readPrimitive(doc, invalid, surface));
    ModelImageData invalidImage = doc.images[0];
    if (invalidImage.embedded())
    {
        invalidImage.source = static_cast<std::uint32_t>(src->images.size());
        CHECK(!gImporter.readImage(doc, invalidImage, bytes));
    }
    ModelData empty;
    CHECK(!gImporter.readPrimitive(empty, doc.meshes[0].primitives[0], surface));
    CHECK(!gImporter.readImage(empty, doc.images[0], bytes) || !doc.images[0].embedded());

    GltfRangeReader reader(*src);
    std::uint8_t scratch[32];
    const std::uint64_t before = reader.bytesRead();
    CHECK(!reader.read(0, 210, 20, scratch));
    CHECK(!reader.read(0, kBinSize + 1, 1, scratch));
    CHECK(!reader.read(1, 0, 4, scratch));
    CHECK(!reader.read(-1, 0, 4, scratch));
    CHECK(!reader.read(0, 0, kBinSize + 1, scratch));
    CHECK(reader.bytesRead() == before);
    CHECK(reader.read(0, 4, 4, scratch));
    CHECK(reader.bytesRead() == before + 4);
}

void test_glb(const ct::String& dir)
{
    ct::Vector<std::uint8_t> bin;
    buildBin(bin);
    CHECK(bin.size() == kBinSize);
    const ct::String json = buildJson(true);
    ct::Vector<std::uint8_t> glb;
    buildGlb(json, bin, glb);

    const ct::String path = dir + "/model.glb";
    CHECK(file_write(path.c_str(), glb.data(), glb.size()));

    ModelData doc;
    CHECK(gImporter.import(path, doc));
    CHECK(doc.path == path);
    CHECK(doc.directory == dir + "/");
    const GltfSource* docSource = GltfImporter::source(doc);
    CHECK(docSource != nullptr);
    if (!docSource) return;
    CHECK(docSource->bufferPaths.size() == 1 && docSource->bufferPaths[0] == path);
    const std::uint32_t jsonLength =
            static_cast<std::uint32_t>((json.size() + 3) & ~std::size_t(3));
    CHECK(docSource->binOffset == 12 + 8 + jsonLength + 8);
    CHECK(docSource->binSize == kBinSize);
    checkDocument(doc, true);
    checkReads(doc);

    ModelData memoryDoc;
    CHECK(gImporter.importMemory(glb.data(), glb.size(), path, memoryDoc));
    const GltfSource* memorySource = GltfImporter::source(memoryDoc);
    CHECK(memorySource != nullptr);
    if (!memorySource) return;
    CHECK(memorySource->binOffset == docSource->binOffset);
    CHECK(memorySource->binSize == docSource->binSize);
    checkDocument(memoryDoc, true);
    checkReads(memoryDoc);

    ModelData truncated;
    CHECK(!gImporter.importMemory(glb.data(), glb.size() - 8, path, truncated));
    CHECK(!truncated.error.empty());

    CHECK(fs_remove_path(path.c_str()));
}

void test_gltf_external(const ct::String& dir)
{
    ct::Vector<std::uint8_t> bin;
    buildBin(bin);
    const ct::String binPath = dir + "/model data.bin";
    const ct::String imagePath = dir + "/tex.png";
    const ct::String path = dir + "/model.gltf";
    const ct::String json = buildJson(false);
    CHECK(file_write(binPath.c_str(), bin.data(), bin.size()));
    CHECK(file_write(imagePath.c_str(), "PNGDATA", 7));
    CHECK(file_write(path.c_str(), json.c_str(), json.size()));

    ModelData doc;
    CHECK(gImporter.import(path, doc));
    const GltfSource* docSource = GltfImporter::source(doc);
    CHECK(docSource != nullptr);
    if (!docSource) return;
    CHECK(!docSource->binary);
    CHECK(docSource->binOffset == 0);
    CHECK(docSource->bufferPaths.size() == 1);
    if (docSource->bufferPaths.size() == 1)
    {
        CHECK(endsWith(docSource->bufferPaths[0], "model data.bin"));
        CHECK(docSource->bufferPaths[0] == binPath);
    }
    checkDocument(doc, false);
    checkReads(doc);

    CHECK(fs_remove_path(binPath.c_str()));
    CHECK(fs_remove_path(imagePath.c_str()));
    CHECK(fs_remove_path(path.c_str()));
}

ct::String miniJson(const char* bufferJson, const char* accessorJson, const char* primitiveJson)
{
    ct::String json = "{\"asset\":{\"version\":\"2.0\"},\"buffers\":[";
    json += bufferJson;
    json += "],\"bufferViews\":[{\"buffer\":0,\"byteLength\":36}],\"accessors\":[";
    json += accessorJson;
    json += "],\"meshes\":[{\"primitives\":[";
    json += primitiveJson;
    json += "]}]}";
    return json;
}

bool parseText(const ct::String& json, ModelData& doc)
{
    return gImporter.importMemory(reinterpret_cast<const std::uint8_t*>(json.c_str()), json.size(),
            "/tmp/none/mini.gltf", doc);
}

void test_failures(const ct::String& dir)
{
    const char* goodBuffer = "{\"uri\":\"x.bin\",\"byteLength\":36}";
    const char* goodAccessor =
            "{\"bufferView\":0,\"componentType\":5126,\"count\":3,\"type\":\"VEC3\"}";
    const char* goodPrimitive = "{\"attributes\":{\"POSITION\":0}}";

    ModelData doc;
    CHECK(parseText(miniJson(goodBuffer, goodAccessor, goodPrimitive), doc));
    const GltfSource* miniSource = GltfImporter::source(doc);
    CHECK(miniSource != nullptr);
    if (miniSource)
        CHECK(miniSource->bufferPaths.size() == 1 &&
                miniSource->bufferPaths[0] == "/tmp/none/x.bin");
    CHECK(doc.directory == "/tmp/none/");
    CHECK(doc.roots.size() == 0);

    CHECK(!parseText(
            miniJson(goodBuffer,
                    "{\"bufferView\":0,\"componentType\":5126,\"count\":100,\"type\":\"VEC3\"}",
                    goodPrimitive),
            doc));
    CHECK(!doc.error.empty());

    CHECK(!parseText(
            miniJson(goodBuffer,
                    "{\"bufferView\":0,\"byteOffset\":30,\"componentType\":5126,\"count\":3,"
                    "\"type\":\"VEC3\"}",
                    goodPrimitive),
            doc));
    CHECK(!doc.error.empty());

    CHECK(!parseText(
            miniJson("{\"uri\":\"data:application/octet-stream;base64,AAAA\",\"byteLength\":3}",
                    goodAccessor, goodPrimitive),
            doc));
    CHECK(!doc.error.empty());

    CHECK(!parseText(
            miniJson(goodBuffer,
                    "{\"bufferView\":0,\"componentType\":5126,\"count\":3,\"type\":\"VEC3\","
                    "\"sparse\":{\"count\":1,\"indices\":{\"bufferView\":0,\"componentType\":5123},"
                    "\"values\":{\"bufferView\":0}}}",
                    goodPrimitive),
            doc));
    CHECK(!doc.error.empty());

    CHECK(!parseText(miniJson(goodBuffer, "{\"componentType\":5126,\"count\":3,\"type\":\"VEC3\"}",
                             goodPrimitive),
            doc));
    CHECK(!doc.error.empty());

    CHECK(!parseText(miniJson(goodBuffer, goodAccessor, "{\"attributes\":{\"NORMAL\":0}}"), doc));
    CHECK(!doc.error.empty());

    CHECK(!parseText(
            miniJson(goodBuffer, goodAccessor, "{\"attributes\":{\"POSITION\":0},\"mode\":6}"),
            doc));
    CHECK(!doc.error.empty());
    CHECK(!parseText(
            miniJson(goodBuffer, goodAccessor, "{\"attributes\":{\"POSITION\":0},\"mode\":2}"),
            doc));
    CHECK(parseText(
            miniJson(goodBuffer, goodAccessor, "{\"attributes\":{\"POSITION\":0},\"mode\":5}"),
            doc));
    CHECK(doc.meshes[0].primitives[0].topology == prisma::Topology::TriangleStrip);

    const char* garbage = "this is not json at all, and it is not glTF either";
    const ct::String garbagePath = dir + "/garbage.gltf";
    CHECK(file_write(garbagePath.c_str(), garbage, strlen(garbage)));
    CHECK(!gImporter.import(garbagePath, doc));
    CHECK(!doc.error.empty());
    CHECK(fs_remove_path(garbagePath.c_str()));

    CHECK(!gImporter.import(dir + "/missing.glb", doc));
    CHECK(!doc.error.empty());

    CHECK(!gImporter.importMemory(nullptr, 0, "none", doc));
    CHECK(!doc.error.empty());

    GltfAccessorRef floats;
    floats.buffer = 0;
    floats.stride = 4;
    floats.count = 3;
    floats.components = 1;
    floats.componentType = GltfComponentType::Float;
    std::uint8_t raw[12];
    memset(raw, 0, sizeof(raw));
    ct::Vector<std::uint32_t> indices;
    CHECK(!GltfImporter::unpackIndices(floats, raw, sizeof(raw), indices));
    floats.componentType = GltfComponentType::UInt32;
    CHECK(GltfImporter::unpackIndices(floats, raw, sizeof(raw), indices));
    CHECK(indices.size() == 3);
    CHECK(!GltfImporter::unpackIndices(floats, raw, sizeof(raw) - 1, indices));
    floats.componentType = GltfComponentType::Float;
    float out[3];
    CHECK(GltfImporter::unpackFloats(floats, raw, sizeof(raw), out, 1));
    CHECK(!GltfImporter::unpackFloats(floats, raw, sizeof(raw) - 1, out, 1));
    floats.componentType = GltfComponentType::Invalid;
    CHECK(!GltfImporter::unpackFloats(floats, raw, sizeof(raw), out, 1));

    CHECK(GltfImporter::componentSize(GltfComponentType::Invalid) == 0);
    CHECK(GltfImporter::componentSize(GltfComponentType::Int8) == 1);
    CHECK(GltfImporter::componentSize(GltfComponentType::UInt16) == 2);
    CHECK(GltfImporter::componentSize(GltfComponentType::Float) == 4);

    GltfAccessorRef empty;
    CHECK(empty.byteLength() == 0);
    CHECK(!empty.valid());
}

void test_normalized_unpack()
{
    GltfAccessorRef ref;
    ref.buffer = 0;
    ref.count = 2;
    ref.components = 2;
    ref.normalized = true;

    const std::int8_t bytes8[4] = { -128, 127, 0, -127 };
    ref.componentType = GltfComponentType::Int8;
    ref.stride = 2;
    float out[6];
    CHECK(GltfImporter::unpackFloats(ref, reinterpret_cast<const std::uint8_t*>(bytes8), 4, out,
            2));
    CHECK(near(out[0], -1.0f) && near(out[1], 1.0f) && near(out[2], 0.0f) && near(out[3], -1.0f));

    const std::int16_t bytes16[4] = { -32768, 32767, 0, 16384 };
    ref.componentType = GltfComponentType::Int16;
    ref.stride = 4;
    CHECK(GltfImporter::unpackFloats(ref, reinterpret_cast<const std::uint8_t*>(bytes16), 8, out,
            2));
    CHECK(near(out[0], -1.0f) && near(out[1], 1.0f) && near(out[2], 0.0f));
    CHECK(near(out[3], 16384.0f / 32767.0f));

    ref.normalized = false;
    ref.componentType = GltfComponentType::UInt8;
    ref.stride = 2;
    const std::uint8_t bytesU8[4] = { 1, 2, 3, 4 };
    CHECK(GltfImporter::unpackFloats(ref, bytesU8, 4, out, 3));
    CHECK(near(out[0], 1.0f) && near(out[1], 2.0f) && near(out[2], 0.0f));
    CHECK(near(out[3], 3.0f) && near(out[4], 4.0f) && near(out[5], 0.0f));
}

void test_matrix_node()
{
    ModelData doc;
    const ct::String json = "{\"asset\":{\"version\":\"2.0\"},\"nodes\":["
                            "{\"matrix\":[2,0,0,0, 0,2,0,0, 0,0,2,0, 4,5,6,1],\"children\":[1]},"
                            "{\"translation\":[1,0,0]},{\"name\":\"loose\"}]}";
    CHECK(parseText(json, doc));
    CHECK(doc.nodes.size() == 3);
    CHECK(doc.roots.size() == 2);
    if (doc.nodes.size() != 3 || doc.roots.size() != 2) return;
    CHECK(doc.roots[0] == 0 && doc.roots[1] == 2);
    CHECK(nearVec3(doc.nodes[0].translation, 4.0f, 5.0f, 6.0f));
    CHECK(nearVec3(doc.nodes[0].scale, 2.0f, 2.0f, 2.0f));
    const Math::Quaternion& q = doc.nodes[0].rotation;
    CHECK(near(q.x, 0.0f) && near(q.y, 0.0f) && near(q.z, 0.0f) && near(fabsf(q.w), 1.0f));
    CHECK(nearVec3(doc.nodeGlobalTransform(1).GetTranslation(), 6.0f, 5.0f, 6.0f));

    const ct::String cycle =
            "{\"asset\":{\"version\":\"2.0\"},\"nodes\":[{\"children\":[1]},{\"children\":[0]}]}";
    CHECK(!parseText(cycle, doc));
    CHECK(!doc.error.empty());

    const ct::String twoParents = "{\"asset\":{\"version\":\"2.0\"},\"nodes\":[{\"children\":[2]},{"
                                  "\"children\":[2]},{}]}";
    CHECK(!parseText(twoParents, doc));
    CHECK(!doc.error.empty());
}

void test_big(const ct::String& dir)
{
    const std::uint64_t binBytes = 300000000ull;
    const std::uint64_t accessorOffset = 299000000ull;
    const std::uint32_t count = 1002;

    ct::String json =
            "{\"asset\":{\"version\":\"2.0\"},\"buffers\":[{\"byteLength\":300000000}],"
            "\"bufferViews\":[{\"buffer\":0,\"byteOffset\":299000000,\"byteLength\":12024}],"
            "\"accessors\":[{\"bufferView\":0,\"componentType\":5126,\"count\":1002,\"type\":"
            "\"VEC3\"}],"
            "\"meshes\":[{\"primitives\":[{\"attributes\":{\"POSITION\":0}}]}]}";
    while (json.size() % 4 != 0) json.push_back(' ');

    ct::Vector<std::uint8_t> head;
    putU32(head, 0x46546C67u);
    putU32(head, 2);
    putU32(head, static_cast<std::uint32_t>(12 + 8 + json.size() + 8 + binBytes));
    putU32(head, static_cast<std::uint32_t>(json.size()));
    putU32(head, 0x4E4F534Au);
    for (std::size_t i = 0; i < json.size(); ++i)
        head.push_back(static_cast<std::uint8_t>(json[i]));
    putU32(head, static_cast<std::uint32_t>(binBytes));
    putU32(head, 0x004E4942u);
    const std::uint64_t binStart = head.size();

    ct::Vector<std::uint8_t> payload;
    for (std::uint32_t i = 0; i < count * 3; ++i) putF32(payload, static_cast<float>(i) * 0.5f);

    const ct::String path = dir + "/big.glb";
    IoStream* stream = io_open_file(path.c_str(), "wb");
    CHECK(stream != nullptr);
    if (!stream) return;
    CHECK(io_write(stream, head.data(), head.size()) == head.size());
    const std::int64_t payloadPos = static_cast<std::int64_t>(binStart + accessorOffset);
    CHECK(io_seek(stream, payloadPos, IO_SEEK_SET) == payloadPos);
    CHECK(io_write(stream, payload.data(), payload.size()) == payload.size());
    const std::int64_t lastPos = static_cast<std::int64_t>(binStart + binBytes - 1);
    CHECK(io_seek(stream, lastPos, IO_SEEK_SET) == lastPos);
    const std::uint8_t zero = 0;
    CHECK(io_write(stream, &zero, 1) == 1);
    CHECK(io_close(stream));
    CHECK(file_size(path.c_str()) == static_cast<std::int64_t>(binStart + binBytes));

    ModelData doc;
    const std::uint64_t begin = time_nanos();
    const bool parsed = gImporter.import(path, doc);
    const std::uint64_t elapsed = time_nanos() - begin;
    CHECK(parsed);
    CHECK(elapsed < 500ull * 1000 * 1000);
    printf("big glb parse: %.3f ms\n", static_cast<double>(elapsed) / 1.0e6);
    const GltfSource* src = GltfImporter::source(doc);
    CHECK(src != nullptr);
    if (!src) return;
    CHECK(src->binary);
    CHECK(src->binOffset == binStart);
    CHECK(src->binSize == binBytes);
    CHECK(src->bufferSizes.size() == 1 && src->bufferSizes[0] == binBytes);
    if (parsed && doc.meshes.size() == 1 && doc.meshes[0].primitives.size() == 1 &&
            src->primitives.size() == 1)
    {
        const ModelPrimitiveData& primitive = doc.meshes[0].primitives[0];
        const GltfPrimitiveSource& accessors = src->primitives[primitive.source];
        CHECK(accessors.positions.offset == accessorOffset);
        GltfRangeReader reader(*src);
        ct::Vector<std::uint8_t> bytes;
        CHECK(reader.read(accessors.positions, bytes));
        CHECK(bytes.size() == payload.size());
        CHECK(bytes.size() == payload.size() &&
                memcmp(bytes.data(), payload.data(), bytes.size()) == 0);
        CHECK(reader.bytesRead() == accessors.positions.byteLength());
        CHECK(reader.bytesRead() == count * 12u);
        CHECK(primitive.sourceBytes == count * 12u);
        CHECK(primitive.vertexCount == count);

        SurfaceData surface;
        CHECK(gImporter.readPrimitive(doc, primitive, surface));
        CHECK(surface.positions.size() == count);
        if (surface.positions.size() == count)
            CHECK(near(surface.positions[999].z, 2999.0f * 0.5f));
    }

    CHECK(fs_remove_path(path.c_str()));
}


const std::uint32_t kSkinBinSize = 396;

void putQuat(ct::Vector<std::uint8_t>& bytes, float x, float y, float z, float w)
{
    putF32(bytes, x);
    putF32(bytes, y);
    putF32(bytes, z);
    putF32(bytes, w);
}

void buildSkinBin(ct::Vector<std::uint8_t>& bin)
{
    bin.clear();
    putVec3(bin, 0.0f, 0.0f, 0.0f);
    putVec3(bin, 1.0f, 0.0f, 0.0f);
    putVec3(bin, 0.0f, 1.0f, 0.0f);

    const std::uint8_t joints[3][4] = { { 0, 1, 0, 0 }, { 1, 0, 0, 0 }, { 0, 1, 0, 0 } };
    for (int i = 0; i < 3; ++i)
        for (int c = 0; c < 4; ++c) bin.push_back(joints[i][c]);

    const std::uint8_t weights[3][4] = { { 255, 0, 0, 0 }, { 204, 51, 0, 0 }, { 100, 50, 0, 0 } };
    for (int i = 0; i < 3; ++i)
        for (int c = 0; c < 4; ++c) bin.push_back(weights[i][c]);

    for (int m = 0; m < 2; ++m)
    {
        for (int c = 0; c < 4; ++c)
        {
            for (int r = 0; r < 4; ++r)
            {
                float value = r == c ? 1.0f : 0.0f;
                if (m == 1 && c == 3 && r == 1) value = -1.0f;
                putF32(bin, value);
            }
        }
    }

    putF32(bin, 1.0f);
    putF32(bin, 1.5f);
    putF32(bin, 2.0f);
    putF32(bin, 1.0f);
    putF32(bin, 2.0f);

    putVec3(bin, 0.0f, 1.0f, 0.0f);
    putVec3(bin, 0.0f, 2.0f, 0.0f);
    putVec3(bin, 0.0f, 3.0f, 0.0f);

    putQuat(bin, 0.0f, 0.0f, 0.0f, 1.0f);
    putQuat(bin, 0.0f, 0.0f, 0.38268343f, 0.92387953f);
    putQuat(bin, 0.0f, 0.0f, 0.70710678f, 0.70710678f);

    putQuat(bin, 0.0f, 0.0f, 0.0f, 1.0f);
    putQuat(bin, 0.0f, 0.0f, 0.70710678f, 0.70710678f);

    putVec3(bin, 9.0f, 9.0f, 9.0f);
    putVec3(bin, 0.0f, 5.0f, 0.0f);
    putVec3(bin, 9.0f, 9.0f, 9.0f);
    putVec3(bin, 9.0f, 9.0f, 9.0f);
    putVec3(bin, 0.0f, 7.0f, 0.0f);
    putVec3(bin, 9.0f, 9.0f, 9.0f);
}

ct::String buildSkinJson(bool extras, const char* joints)
{
    ct::String json = "{\"asset\":{\"version\":\"2.0\"},"
                      "\"scene\":0,\"scenes\":[{\"nodes\":[0,1]}],"
                      "\"nodes\":["
                      "{\"name\":\"body\",\"mesh\":0,\"skin\":0},"
                      "{\"name\":\"root\",\"scale\":[2,2,2],\"children\":[2]},"
                      "{\"name\":\"spine\",\"translation\":[0,1,0]}],"
                      "\"skins\":[{\"joints\":";
    json += joints;
    json += ",\"inverseBindMatrices\":3}],"
            "\"meshes\":[{\"name\":\"skinned\",\"primitives\":[{\"attributes\":{\"POSITION\":0,"
            "\"JOINTS_0\":1,\"WEIGHTS_0\":2}}]}],"
            "\"animations\":["
            "{\"name\":\"wave\",\"samplers\":["
            "{\"input\":4,\"output\":6},{\"input\":4,\"output\":7},{\"input\":5,\"output\":8}],"
            "\"channels\":["
            "{\"sampler\":0,\"target\":{\"node\":2,\"path\":\"translation\"}},"
            "{\"sampler\":1,\"target\":{\"node\":2,\"path\":\"rotation\"}},"
            "{\"sampler\":2,\"target\":{\"node\":1,\"path\":\"rotation\"}},"
            "{\"sampler\":0,\"target\":{\"node\":0,\"path\":\"translation\"}}]}";
    if (extras)
        json += ",{\"samplers\":[{\"input\":5,\"output\":9,\"interpolation\":\"CUBICSPLINE\"},"
                "{\"input\":4,\"output\":6}],"
                "\"channels\":[{\"sampler\":0,\"target\":{\"node\":2,\"path\":\"translation\"}}]},"
                "{\"name\":\"loose\",\"samplers\":[{\"input\":4,\"output\":6}],"
                "\"channels\":[{\"sampler\":0,\"target\":{\"node\":0,\"path\":\"translation\"}}]}";
    json += "],\"buffers\":[{\"byteLength\":396}],"
            "\"bufferViews\":["
            "{\"buffer\":0,\"byteOffset\":0,\"byteLength\":36},"
            "{\"buffer\":0,\"byteOffset\":36,\"byteLength\":12},"
            "{\"buffer\":0,\"byteOffset\":48,\"byteLength\":12},"
            "{\"buffer\":0,\"byteOffset\":60,\"byteLength\":128},"
            "{\"buffer\":0,\"byteOffset\":188,\"byteLength\":12},"
            "{\"buffer\":0,\"byteOffset\":200,\"byteLength\":8},"
            "{\"buffer\":0,\"byteOffset\":208,\"byteLength\":36},"
            "{\"buffer\":0,\"byteOffset\":244,\"byteLength\":48},"
            "{\"buffer\":0,\"byteOffset\":292,\"byteLength\":32},"
            "{\"buffer\":0,\"byteOffset\":324,\"byteLength\":72}],"
            "\"accessors\":["
            "{\"bufferView\":0,\"componentType\":5126,\"count\":3,\"type\":\"VEC3\","
            "\"min\":[0,0,0],\"max\":[1,1,0]},"
            "{\"bufferView\":1,\"componentType\":5121,\"count\":3,\"type\":\"VEC4\"},"
            "{\"bufferView\":2,\"componentType\":5121,\"normalized\":true,\"count\":3,\"type\":"
            "\"VEC4\"},"
            "{\"bufferView\":3,\"componentType\":5126,\"count\":2,\"type\":\"MAT4\"},"
            "{\"bufferView\":4,\"componentType\":5126,\"count\":3,\"type\":\"SCALAR\"},"
            "{\"bufferView\":5,\"componentType\":5126,\"count\":2,\"type\":\"SCALAR\"},"
            "{\"bufferView\":6,\"componentType\":5126,\"count\":3,\"type\":\"VEC3\"},"
            "{\"bufferView\":7,\"componentType\":5126,\"count\":3,\"type\":\"VEC4\"},"
            "{\"bufferView\":8,\"componentType\":5126,\"count\":2,\"type\":\"VEC4\"},"
            "{\"bufferView\":9,\"componentType\":5126,\"count\":6,\"type\":\"VEC3\"}]}";
    return json;
}

bool nearQuat(const Math::Quaternion& a, float x, float y, float z, float w)
{
    return near(a.x, x) && near(a.y, y) && near(a.z, z) && near(a.w, w);
}

void checkSkinDocument(const ModelData& doc, bool extras, bool readable)
{
    CHECK(doc.error.empty());
    CHECK(doc.nodes.size() == 3);
    CHECK(doc.meshes.size() == 1);
    CHECK(doc.skeletons.size() == 1);
    if (doc.skeletons.size() != 1 || doc.nodes.size() != 3 || doc.meshes.size() != 1) return;

    const Skeleton& skeleton = doc.skeletons[0];
    CHECK(skeleton.boneCount() == 2);
    CHECK(skeleton.finalized());
    if (skeleton.boneCount() != 2) return;
    CHECK(skeleton.bone(0).name == "root");
    CHECK(skeleton.bone(1).name == "spine");
    CHECK(skeleton.bone(0).parent == -1);
    CHECK(skeleton.bone(1).parent == 0);
    CHECK(skeleton.findBone("spine") == 1);
    CHECK(skeleton.order().size() == 2 && skeleton.order()[0] == 0 && skeleton.order()[1] == 1);
    CHECK(near(skeleton.bone(0).bindLocal.col0.x, 2.0f));
    CHECK(near(skeleton.bone(0).bindLocal.col1.y, 2.0f));
    CHECK(nearVec3(skeleton.bone(1).bindLocal.GetTranslation(), 0.0f, 1.0f, 0.0f));
    CHECK(near(skeleton.bone(0).inverseBind.col0.x, 1.0f));
    CHECK(nearVec3(skeleton.bone(0).inverseBind.GetTranslation(), 0.0f, 0.0f, 0.0f));
    CHECK(nearVec3(skeleton.bone(1).inverseBind.GetTranslation(), 0.0f, -1.0f, 0.0f));
    CHECK(near(skeleton.bone(1).inverseBind.col3.w, 1.0f));

    CHECK(doc.nodes[0].skeleton == 0);
    CHECK(doc.nodes[1].skeleton == -1);
    CHECK(doc.nodes[2].skeleton == -1);

    CHECK(doc.meshes[0].primitives.size() == 1);
    if (doc.meshes[0].primitives.size() != 1) return;
    const ModelPrimitiveData& primitive = doc.meshes[0].primitives[0];
    CHECK(primitive.skinned);
    CHECK(primitive.attributeMask ==
            (kVertexPosition | kVertexNormal | kVertexJoints | kVertexWeights));
    CHECK(primitive.sourceBytes == 60);

    SurfaceData surface;
    CHECK(gImporter.readPrimitive(doc, primitive, surface) == readable);
    CHECK(!readable || surface.positions.size() == 3);
    CHECK(!readable || surface.joints.size() == 3);
    CHECK(!readable || surface.weights.size() == 3);
    if (surface.joints.size() == 3 && surface.weights.size() == 3)
    {
        CHECK(surface.joints[0] == 0x0100u);
        CHECK(surface.joints[1] == 0x0001u);
        CHECK(surface.joints[2] == 0x0100u);
        CHECK(near(surface.weights[0].x, 1.0f) && near(surface.weights[0].y, 0.0f));
        CHECK(near(surface.weights[1].x, 0.8f) && near(surface.weights[1].y, 0.2f));
        CHECK(near(surface.weights[2].x, 2.0f / 3.0f) && near(surface.weights[2].y, 1.0f / 3.0f));
        CHECK(near(surface.weights[2].z, 0.0f) && near(surface.weights[2].w, 0.0f));
        for (int i = 0; i < 3; ++i)
        {
            const Math::Vec4& w = surface.weights[i];
            CHECK(near(w.x + w.y + w.z + w.w, 1.0f));
        }
    }

    CHECK(doc.animations.size() == (extras ? 2u : 1u));
    if (doc.animations.empty()) return;
    const AnimationClip& clip = doc.animations[0];
    CHECK(clip.name() == "wave");
    CHECK(clip.skeleton() == 0);
    CHECK(near(clip.duration(), 1.0f));
    CHECK(clip.tracks().size() == 2);
    if (clip.tracks().size() == 2)
    {
        const BoneTrack& root = clip.tracks()[0];
        const BoneTrack& spine = clip.tracks()[1];
        CHECK(root.bone == 0 && spine.bone == 1);
        CHECK(root.times.size() == 2 && near(root.times[0], 0.0f) && near(root.times[1], 1.0f));
        CHECK(spine.times.size() == 3 && near(spine.times[0], 0.0f) && near(spine.times[1], 0.5f) &&
                near(spine.times[2], 1.0f));
        CHECK(root.positions.size() == 2 && root.rotations.size() == 2 && root.scales.size() == 2);
        CHECK(spine.positions.size() == 3 && spine.rotations.size() == 3 &&
                spine.scales.size() == 3);
        if (root.scales.size() == 2 && root.positions.size() == 2 && root.rotations.size() == 2)
        {
            CHECK(nearVec3(root.scales[0], 2.0f, 2.0f, 2.0f));
            CHECK(nearVec3(root.scales[1], 2.0f, 2.0f, 2.0f));
            CHECK(nearVec3(root.positions[0], 0.0f, 0.0f, 0.0f));
            CHECK(nearQuat(root.rotations[0], 0.0f, 0.0f, 0.0f, 1.0f));
            CHECK(nearQuat(root.rotations[1], 0.0f, 0.0f, 0.70710678f, 0.70710678f));
        }
        if (spine.scales.size() == 3 && spine.positions.size() == 3 && spine.rotations.size() == 3)
        {
            for (int i = 0; i < 3; ++i) CHECK(nearVec3(spine.scales[i], 1.0f, 1.0f, 1.0f));
            CHECK(nearVec3(spine.positions[0], 0.0f, 1.0f, 0.0f));
            CHECK(nearVec3(spine.positions[1], 0.0f, 2.0f, 0.0f));
            CHECK(nearVec3(spine.positions[2], 0.0f, 3.0f, 0.0f));
            CHECK(nearQuat(spine.rotations[1], 0.0f, 0.0f, 0.38268343f, 0.92387953f));
        }

        ct::Vector<LocalPose> pose;
        skeleton.bindPose(pose);
        clip.sample(0.5f, pose);
        CHECK(nearVec3(pose[1].position, 0.0f, 2.0f, 0.0f));
        CHECK(nearQuat(pose[1].rotation, 0.0f, 0.0f, 0.38268343f, 0.92387953f));
        CHECK(nearVec3(pose[1].scale, 1.0f, 1.0f, 1.0f));
        CHECK(nearQuat(pose[0].rotation, 0.0f, 0.0f, 0.38268343f, 0.92387953f));
        CHECK(nearVec3(pose[0].scale, 2.0f, 2.0f, 2.0f));
    }

    if (extras && doc.animations.size() == 2)
    {
        const AnimationClip& cubic = doc.animations[1];
        CHECK(cubic.name() == "animation_1");
        CHECK(cubic.skeleton() == 0);
        CHECK(near(cubic.duration(), 1.0f));
        CHECK(cubic.tracks().size() == 1);
        if (cubic.tracks().size() == 1 && cubic.tracks()[0].positions.size() == 2)
        {
            CHECK(cubic.tracks()[0].bone == 1);
            CHECK(nearVec3(cubic.tracks()[0].positions[0], 0.0f, 5.0f, 0.0f));
            CHECK(nearVec3(cubic.tracks()[0].positions[1], 0.0f, 7.0f, 0.0f));
        }
    }
}

void test_skin(const ct::String& dir)
{
    ct::Vector<std::uint8_t> bin;
    buildSkinBin(bin);
    CHECK(bin.size() == kSkinBinSize);

    for (int variant = 0; variant < 2; ++variant)
    {
        const bool extras = variant == 1;
        const ct::String json = buildSkinJson(extras, "[1,2]");
        ct::Vector<std::uint8_t> glb;
        buildGlb(json, bin, glb);

        const ct::String path = dir + "/skin.glb";
        CHECK(file_write(path.c_str(), glb.data(), glb.size()));
        ModelData doc;
        CHECK(gImporter.import(path, doc));
        checkSkinDocument(doc, extras, true);

        ModelData memoryDoc;
        CHECK(gImporter.importMemory(glb.data(), glb.size(), dir + "/missing.glb", memoryDoc));
        checkSkinDocument(memoryDoc, extras, false);
        CHECK(fs_remove_path(path.c_str()));
    }

    ct::Vector<std::uint8_t> glb;
    buildGlb(buildSkinJson(false, "[1,1]"), bin, glb);
    ModelData duplicate;
    CHECK(!gImporter.importMemory(glb.data(), glb.size(), "dup.glb", duplicate));
    CHECK(!duplicate.error.empty());

    buildGlb(buildSkinJson(false, "[2,1]"), bin, glb);
    ModelData reversed;
    CHECK(gImporter.importMemory(glb.data(), glb.size(), "rev.glb", reversed));
    CHECK(reversed.skeletons.size() == 1 && reversed.skeletons[0].boneCount() == 2);
    if (reversed.skeletons.size() == 1 && reversed.skeletons[0].boneCount() == 2)
    {
        CHECK(reversed.skeletons[0].bone(0).name == "spine");
        CHECK(reversed.skeletons[0].bone(0).parent == 1);
        CHECK(reversed.skeletons[0].bone(1).parent == -1);
        CHECK(reversed.skeletons[0].order()[0] == 1);
    }
}

} // namespace

int main()
{
    const ct::String dir = ct::String(PRISMA3D_TEST_DIR) + "/gltf";
    CHECK(fs_create_directory(dir.c_str()));

    test_glb(dir);
    test_gltf_external(dir);
    test_skin(dir);
    test_failures(dir);
    test_normalized_unpack();
    test_matrix_node();
    test_big(dir);

    fs_remove_path(dir.c_str());

    if (failures) printf("%d check(s) failed\n", failures);
    else
        printf("all checks passed\n");
    return failures != 0;
}
