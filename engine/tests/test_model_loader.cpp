#include "Check.h"
#include "LitTest.h"

#include "prisma3d/Device.h"
#include "prisma3d/core/FileSystem.h"
#include "prisma3d/core/Pixmap.h"
#include "prisma3d/import/ModelImporter.h"
#include "prisma3d/main/Main.h"
#include "prisma3d/render/AssetLoader.h"
#include "prisma3d/render/MaterialStore.h"
#include "prisma3d/render/MeshStore.h"
#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/render/TextureStore.h"
#include "prisma3d/scene/Camera3D.h"
#include "prisma3d/scene/ModelInstance.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/Viewport.h"
#include "prisma3d/scene/VisualInstance3D.h"

#include "platform.h"

#include <math.h>
#include <stdarg.h>
#include <stdint.h>
#include <string.h>

#include <ct/string.hpp>
#include <ct/vector.hpp>

using namespace prisma3d;

namespace
{

typedef ct::Vector<std::uint8_t> Bytes;

const int kWidth = 64;
const int kHeight = 48;
const int kTolerance = 2;
const std::uint64_t kMiB = 1u << 20;

struct Rgb
{
    int r;
    int g;
    int b;
};

const Rgb kClear = { 26, 51, 77 };

std::uint64_t secondsToNanos(unsigned seconds)
{
    return static_cast<std::uint64_t>(seconds) * 1000000000ull;
}

double millis(std::uint64_t nanos)
{
    return static_cast<double>(nanos) / 1.0e6;
}

ct::String path(const char* directory, const char* name)
{
    ct::String result(directory);
    result += "/";
    result += name;
    return result;
}

void appendf(ct::String& text, const char* format, ...)
{
    char buffer[1024];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    text += buffer;
}

void putU32(Bytes& bytes, std::uint32_t value)
{
    for (int i = 0; i < 4; ++i) bytes.push_back(static_cast<std::uint8_t>(value >> (8 * i)));
}

void putU16(Bytes& bytes, std::uint16_t value)
{
    bytes.push_back(static_cast<std::uint8_t>(value));
    bytes.push_back(static_cast<std::uint8_t>(value >> 8));
}

void putF32(Bytes& bytes, float value)
{
    std::uint32_t raw;
    memcpy(&raw, &value, sizeof(raw));
    putU32(bytes, raw);
}

std::uint32_t addView(Bytes& bin, const Bytes& data)
{
    while (bin.size() % 4 != 0) bin.push_back(0);
    const std::uint32_t offset = static_cast<std::uint32_t>(bin.size());
    for (std::size_t i = 0; i < data.size(); ++i) bin.push_back(data[i]);
    return offset;
}

void buildGlb(const ct::String& json, const Bytes& bin, Bytes& glb)
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

bool near(float a, float b)
{
    return fabsf(a - b) < 0.0001f;
}

bool readBuffer(prisma::Driver* driver, prisma::BufferHandle buffer, std::uint32_t offset,
        std::uint32_t size, Bytes& out)
{
    out.resize(size);
    return driver->readBuffer(buffer, offset, size, out.data());
}

bool readTexture(TextureStore& store, TextureId id, unsigned width, unsigned height, Bytes& out)
{
    prisma::RenderTarget target;
    target.texture = store.gpu(id);
    target.mip = 0;
    prisma::Rect rect;
    rect.x = 0;
    rect.y = 0;
    rect.width = width;
    rect.height = height;
    out.resize(static_cast<std::size_t>(width) * height * 4);
    return store.driver()->readPixels(target, rect, out.data());
}

bool readPixel(int x, int y, unsigned char* pixel)
{
    prisma::Rect rect;
    rect.x = x;
    rect.y = y;
    rect.width = 1;
    rect.height = 1;
    return Main::device()->driver()->readPixels(prisma::RenderTarget(), rect, pixel);
}

bool pixelIs(int x, int y, Rgb expected, const char* label)
{
    unsigned char pixel[4] = { 0, 0, 0, 0 };
    if (!readPixel(x, y, pixel))
    {
        printf("%s: pixel (%d,%d) could not be read\n", label, x, y);
        return false;
    }
    const int dr = pixel[0] - expected.r;
    const int dg = pixel[1] - expected.g;
    const int db = pixel[2] - expected.b;
    const bool same = dr >= -kTolerance && dr <= kTolerance && dg >= -kTolerance &&
                      dg <= kTolerance && db >= -kTolerance && db <= kTolerance;
    if (!same)
        printf("%s: pixel (%d,%d) is %d,%d,%d, expected %d,%d,%d\n", label, x, y, pixel[0],
                pixel[1], pixel[2], expected.r, expected.g, expected.b);
    return same;
}

bool pixelIsNot(int x, int y, Rgb unexpected, const char* label)
{
    unsigned char pixel[4] = { 0, 0, 0, 0 };
    if (!readPixel(x, y, pixel))
    {
        printf("%s: pixel (%d,%d) could not be read\n", label, x, y);
        return false;
    }
    const int dr = pixel[0] - unexpected.r;
    const int dg = pixel[1] - unexpected.g;
    const int db = pixel[2] - unexpected.b;
    const bool same = dr >= -kTolerance && dr <= kTolerance && dg >= -kTolerance &&
                      dg <= kTolerance && db >= -kTolerance && db <= kTolerance;
    if (same) printf("%s: pixel (%d,%d) is the clear colour\n", label, x, y);
    return !same;
}

struct Frame
{
    explicit Frame(SceneTree& tree)
    {
        tree.process(1.0 / 60.0);
        Main::rendering()->renderViewports();
    }

    ~Frame()
    {
        Main::rendering()->endFrame(true);
    }
};

const std::uint8_t kPixelBase = 10;

void pngPixel(int x, int y, std::uint8_t* rgba)
{
    rgba[0] = static_cast<std::uint8_t>(kPixelBase + x * 60);
    rgba[1] = static_cast<std::uint8_t>(20 + y * 100);
    rgba[2] = 200;
    rgba[3] = 255;
}

bool buildPng(const ct::String& file, Bytes& out)
{
    Pixmap pixmap(4, 2, 4);
    for (int y = 0; y < 2; ++y)
        for (int x = 0; x < 4; ++x) pngPixel(x, y, pixmap.pixels + (y * 4 + x) * 4);
    if (!pixmap.save(file.c_str())) return false;
    out = FileSystem::getSingleton().readBinary(file);
    return !out.empty();
}

const float kQuad[4][3] = { { -0.5f, -0.5f, 0.0f }, { 0.5f, -0.5f, 0.0f }, { 0.5f, 0.5f, 0.0f },
    { -0.5f, 0.5f, 0.0f } };
const float kQuadUv[4][2] = { { 0.0f, 1.0f }, { 1.0f, 1.0f }, { 1.0f, 0.0f }, { 0.0f, 0.0f } };
const std::uint8_t kQuadColor[4][4] = { { 255, 0, 0, 255 }, { 0, 255, 0, 255 }, { 0, 0, 255, 255 },
    { 255, 255, 255, 255 } };
const std::uint16_t kQuadIndices[6] = { 0, 1, 2, 0, 2, 3 };
const float kTriangle[3][3] = { { 0.0f, 0.0f, 0.0f }, { 1.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f } };
const std::uint32_t kTriangleIndices[3] = { 0, 1, 2 };

bool writeSmallGlb(const ct::String& file, const Bytes& png)
{
    Bytes bin;
    Bytes data;

    for (int i = 0; i < 4; ++i)
        for (int c = 0; c < 3; ++c) putF32(data, kQuad[i][c]);
    const std::uint32_t positionsA = addView(bin, data);
    const std::uint32_t positionsALength = static_cast<std::uint32_t>(data.size());

    data.clear();
    for (int i = 0; i < 4; ++i)
    {
        putF32(data, 0.0f);
        putF32(data, 0.0f);
        putF32(data, 1.0f);
    }
    const std::uint32_t normalsA = addView(bin, data);

    data.clear();
    for (int i = 0; i < 4; ++i)
    {
        putF32(data, kQuadUv[i][0]);
        putF32(data, kQuadUv[i][1]);
    }
    const std::uint32_t uvsA = addView(bin, data);

    data.clear();
    for (int i = 0; i < 4; ++i)
        for (int c = 0; c < 4; ++c) data.push_back(kQuadColor[i][c]);
    const std::uint32_t colorsA = addView(bin, data);

    data.clear();
    for (int i = 0; i < 6; ++i) putU16(data, kQuadIndices[i]);
    const std::uint32_t indicesA = addView(bin, data);

    data.clear();
    for (int i = 0; i < 3; ++i)
        for (int c = 0; c < 3; ++c) putF32(data, kTriangle[i][c]);
    const std::uint32_t positionsB = addView(bin, data);

    data.clear();
    for (int i = 0; i < 3; ++i) putU32(data, kTriangleIndices[i]);
    const std::uint32_t indicesB = addView(bin, data);

    const std::uint32_t image = addView(bin, png);

    ct::String json = "{\"asset\":{\"version\":\"2.0\"},\"scene\":0,\"scenes\":[{\"nodes\":[0]}],";
    json += "\"nodes\":["
            "{\"name\":\"R\",\"translation\":[1,2,3],\"children\":[1,2]},"
            "{\"name\":\"M\",\"rotation\":[0,0.70710678,0,0.70710678],\"scale\":[2,2,2],"
            "\"mesh\":0,\"children\":[3]},"
            "{\"name\":\"C\",\"camera\":0},"
            "{\"name\":\"N\",\"mesh\":1}],"
            "\"cameras\":[{\"type\":\"perspective\",\"perspective\":{\"yfov\":0.8,"
            "\"znear\":0.5,\"zfar\":100}}],"
            "\"meshes\":["
            "{\"name\":\"meshA\",\"primitives\":[{\"attributes\":{\"POSITION\":0,\"NORMAL\":1,"
            "\"TEXCOORD_0\":2,\"COLOR_0\":3},\"indices\":4,\"material\":0}]},"
            "{\"name\":\"meshB\",\"primitives\":[{\"attributes\":{\"POSITION\":5},\"indices\":6}]}"
            "],"
            "\"materials\":["
            "{\"pbrMetallicRoughness\":{\"baseColorFactor\":[0.5,1,0.25,0.75],"
            "\"metallicFactor\":0.25,\"roughnessFactor\":0.75,"
            "\"baseColorTexture\":{\"index\":0},\"metallicRoughnessTexture\":{\"index\":2}},"
            "\"occlusionTexture\":{\"index\":2},\"emissiveFactor\":[0.1,0.2,0.3],"
            "\"alphaMode\":\"MASK\",\"alphaCutoff\":0.3,\"doubleSided\":true},"
            "{\"emissiveTexture\":{\"index\":1},\"normalTexture\":{\"index\":2},"
            "\"alphaMode\":\"BLEND\"}],"
            "\"textures\":[{\"source\":0},{\"sampler\":0,\"source\":0},{\"source\":0}],"
            "\"samplers\":[{\"wrapS\":33071,\"wrapT\":33071}],";
    appendf(json, "\"images\":[{\"bufferView\":7,\"mimeType\":\"image/png\"}],");
    appendf(json, "\"buffers\":[{\"byteLength\":%u}],", static_cast<unsigned>(bin.size()));
    appendf(json,
            "\"bufferViews\":["
            "{\"buffer\":0,\"byteOffset\":%u,\"byteLength\":%u},"
            "{\"buffer\":0,\"byteOffset\":%u,\"byteLength\":48},"
            "{\"buffer\":0,\"byteOffset\":%u,\"byteLength\":32},"
            "{\"buffer\":0,\"byteOffset\":%u,\"byteLength\":16},"
            "{\"buffer\":0,\"byteOffset\":%u,\"byteLength\":12},"
            "{\"buffer\":0,\"byteOffset\":%u,\"byteLength\":36},"
            "{\"buffer\":0,\"byteOffset\":%u,\"byteLength\":12},"
            "{\"buffer\":0,\"byteOffset\":%u,\"byteLength\":%u}],",
            positionsA, positionsALength, normalsA, uvsA, colorsA, indicesA, positionsB, indicesB,
            image, static_cast<unsigned>(png.size()));
    json += "\"accessors\":["
            "{\"bufferView\":0,\"componentType\":5126,\"count\":4,\"type\":\"VEC3\"},"
            "{\"bufferView\":1,\"componentType\":5126,\"count\":4,\"type\":\"VEC3\"},"
            "{\"bufferView\":2,\"componentType\":5126,\"count\":4,\"type\":\"VEC2\"},"
            "{\"bufferView\":3,\"componentType\":5121,\"normalized\":true,\"count\":4,\"type\":"
            "\"VEC4\"},"
            "{\"bufferView\":4,\"componentType\":5123,\"count\":6,\"type\":\"SCALAR\"},"
            "{\"bufferView\":5,\"componentType\":5126,\"count\":3,\"type\":\"VEC3\"},"
            "{\"bufferView\":6,\"componentType\":5125,\"count\":3,\"type\":\"SCALAR\"}]}";

    Bytes glb;
    buildGlb(json, bin, glb);
    return file_write(file.c_str(), glb.data(), glb.size());
}

const std::uint32_t kBigPrimitives = 8;
// A whole number of triangles: unindexed corners left over are dropped with flat normals.
const std::uint32_t kBigVertices = 524286;
const std::uint64_t kBigBinBytes = 3000000000ull;
const std::uint64_t kBigSpread = 350000000ull;

float bigX(std::uint32_t index)
{
    return static_cast<float>(index) * 0.001f;
}

bool writeBigGlb(const ct::String& file)
{
    const std::uint64_t viewBytes = static_cast<std::uint64_t>(kBigVertices) * 12;
    ct::String json = "{\"asset\":{\"version\":\"2.0\"},";
    appendf(json, "\"buffers\":[{\"byteLength\":%llu}],",
            static_cast<unsigned long long>(kBigBinBytes));
    json += "\"bufferViews\":[";
    for (std::uint32_t p = 0; p < kBigPrimitives; ++p)
    {
        if (p) json += ",";
        appendf(json, "{\"buffer\":0,\"byteOffset\":%llu,\"byteLength\":%llu}",
                static_cast<unsigned long long>(p * kBigSpread),
                static_cast<unsigned long long>(viewBytes));
    }
    json += "],\"accessors\":[";
    for (std::uint32_t p = 0; p < kBigPrimitives; ++p)
    {
        if (p) json += ",";
        appendf(json, "{\"bufferView\":%u,\"componentType\":5126,\"count\":%u,\"type\":\"VEC3\"}",
                p, kBigVertices);
    }
    json += "],\"meshes\":[{\"name\":\"big\",\"primitives\":[";
    for (std::uint32_t p = 0; p < kBigPrimitives; ++p)
    {
        if (p) json += ",";
        appendf(json, "{\"attributes\":{\"POSITION\":%u}}", p);
    }
    json += "]}]}";
    while (json.size() % 4 != 0) json.push_back(' ');

    Bytes head;
    putU32(head, 0x46546C67u);
    putU32(head, 2);
    putU32(head, static_cast<std::uint32_t>(12 + 8 + json.size() + 8 + kBigBinBytes));
    putU32(head, static_cast<std::uint32_t>(json.size()));
    putU32(head, 0x4E4F534Au);
    for (std::size_t i = 0; i < json.size(); ++i)
        head.push_back(static_cast<std::uint8_t>(json[i]));
    putU32(head, static_cast<std::uint32_t>(kBigBinBytes));
    putU32(head, 0x004E4942u);
    const std::uint64_t binStart = head.size();

    IoStream* stream = io_open_file(file.c_str(), "wb");
    if (!stream) return false;
    bool ok = io_write(stream, head.data(), head.size()) == head.size();

    const std::uint32_t chunkVertices = 87381;
    Bytes chunk;
    for (std::uint32_t p = 0; ok && p < kBigPrimitives; ++p)
    {
        const std::int64_t position = static_cast<std::int64_t>(binStart + p * kBigSpread);
        ok = io_seek(stream, position, IO_SEEK_SET) == position;
        for (std::uint32_t first = 0; ok && first < kBigVertices; first += chunkVertices)
        {
            const std::uint32_t last =
                    first + chunkVertices < kBigVertices ? first + chunkVertices : kBigVertices;
            chunk.clear();
            for (std::uint32_t i = first; i < last; ++i)
            {
                putF32(chunk, bigX(i));
                putF32(chunk, static_cast<float>(p));
                putF32(chunk, -bigX(i));
            }
            ok = io_write(stream, chunk.data(), chunk.size()) == chunk.size();
        }
    }
    const std::int64_t lastPosition = static_cast<std::int64_t>(binStart + kBigBinBytes - 1);
    if (ok) ok = io_seek(stream, lastPosition, IO_SEEK_SET) == lastPosition;
    const std::uint8_t zero = 0;
    if (ok) ok = io_write(stream, &zero, 1) == 1;
    const bool closed = io_close(stream);
    return ok && closed &&
           file_size(file.c_str()) == static_cast<std::int64_t>(binStart + kBigBinBytes);
}

void checkVertex(const Bytes& bytes, std::uint32_t slot, std::uint32_t index, std::uint32_t prim,
        const char* label)
{
    float vertex[10];
    memcpy(vertex, bytes.data() + slot * 40, sizeof(vertex));
    const bool same = vertex[0] == bigX(index) && vertex[1] == static_cast<float>(prim) &&
                      vertex[2] == -bigX(index) && vertex[3] == 0.0f && vertex[4] == 0.0f &&
                      vertex[5] == 0.0f && vertex[6] == 1.0f && vertex[7] == 1.0f &&
                      vertex[8] == 1.0f && vertex[9] == 1.0f;
    if (!same)
        printf("%s: vertex %u of primitive %u is %g,%g,%g\n", label, index, prim, vertex[0],
                vertex[1], vertex[2]);
    CHECK(same);
}

void testSmallModel(AssetLoader& loader, const char* directory, SceneTree& tree, Window* window,
        Camera3D* camera)
{
    TextureStore& textures = *Main::textures();
    MeshStore& meshes = *Main::meshes();
    prisma::Driver* driver = Main::device()->driver();
    Node* root = tree.getRoot();

    Bytes png;
    CHECK(buildPng(path(directory, "pixels.png"), png));
    const ct::String file = path(directory, "model.glb");
    CHECK(writeSmallGlb(file, png));

    const std::uint32_t totalBefore = loader.totalRequests();
    const ModelId id = loader.requestModel(file);
    CHECK(id.valid());
    CHECK(loader.modelStatus(id) == LoadStatus::InProgress);
    CHECK(loader.totalRequests() == totalBefore + 1);
    CHECK(loader.pendingRequests() == 1);
    const ModelInfo* info = loader.model(id);
    CHECK(info != nullptr);
    if (!info) return;
    CHECK(!info->parsed);
    CHECK(loader.modelProgress(id) == 0.0f);
    CHECK(instantiateModel(root, id) == nullptr);
    CHECK(!loader.freeModel(id));
    CHECK(loader.modelStatus(ModelId()) == LoadStatus::Invalid);
    CHECK(loader.model(ModelId()) == nullptr);
    CHECK(loader.pendingByteLimit() == AssetLoader::kDefaultPendingByteLimit);

    bool renderedEarly = false;
    bool finished = false;
    const std::uint64_t start = time_nanos();
    for (int i = 0; i < 10000 && !finished; ++i)
    {
        if (time_nanos() - start > secondsToNanos(20)) break;
        loader.pump(64);
        CHECK(loader.pendingBytes() <= loader.pendingByteLimit());
        const float progress = loader.modelProgress(id);
        CHECK(progress >= 0.0f && progress <= 1.0f);
        if (info->parsed && !renderedEarly && info->uploadedBytes == 0)
        {
            renderedEarly = true;
            CHECK(info->meshes.size() == 2);
            Node3D* early = instantiateModel(root, id);
            CHECK(early != nullptr);
            if (early)
            {
                {
                    Frame frame(tree);
                    CHECK(pixelIs(32, 24, kClear, "model before upload"));
                }
                early->queueFree();
                tree.process(1.0 / 60.0);
            }
        }
        finished = info->status != LoadStatus::InProgress;
        if (!finished) time_sleep(1);
    }
    CHECK(finished);
    CHECK(renderedEarly);
    CHECK(loader.modelStatus(id) == LoadStatus::Loaded);
    CHECK(loader.modelProgress(id) == 1.0f);
    CHECK(info->parsed);
    CHECK(info->error.empty());
    CHECK(info->pendingJobs == 0 && info->failedJobs == 0);
    CHECK(info->meshes.size() == 2);
    CHECK(info->textures.size() == 3);
    CHECK(loader.pendingRequests() == 0);
    CHECK(loader.pendingBytes() == 0);
    if (info->meshes.size() != 2 || info->textures.size() != 3) return;
    CHECK(info->totalBytes == 268 + 126 + 2 * 32);
    CHECK(info->uploadedBytes == info->totalBytes);

    const MeshId meshA = info->meshes[0];
    const MeshId meshB = info->meshes[1];
    CHECK(meshes.complete(meshA) && meshes.complete(meshB));
    const MeshInfo* infoA = meshes.get(meshA);
    const MeshInfo* infoB = meshes.get(meshB);
    CHECK(infoA && infoB);

    MaterialStore& materials = *Main::materials();
    CHECK(info->materials.size() == 2);
    if (info->materials.size() == 2)
    {
        const MaterialId first = info->materials[0];
        const MaterialId second = info->materials[1];
        CHECK(materials.valid(first) && materials.valid(second) && first != second);
        const MaterialInfo* firstInfo = materials.get(first);
        const MaterialInfo* secondInfo = materials.get(second);
        if (firstInfo && secondInfo)
        {
            CHECK(near(firstInfo->albedo.x, 0.5f) && near(firstInfo->albedo.y, 1.0f) &&
                    near(firstInfo->albedo.z, 0.25f) && near(firstInfo->albedo.w, 0.75f));
            CHECK(near(firstInfo->metallic, 0.25f) && near(firstInfo->roughness, 0.75f));
            CHECK(near(firstInfo->emission.x, 0.1f) && near(firstInfo->emission.y, 0.2f) &&
                    near(firstInfo->emission.z, 0.3f));
            CHECK(firstInfo->transparency == MaterialTransparency::AlphaScissor);
            CHECK(near(firstInfo->alphaScissorThreshold, 0.3f));
            CHECK(firstInfo->cull == MaterialCull::Disabled);
            CHECK(materials.getTexture(first, MaterialTexture::Albedo) == info->textures[0]);
            CHECK(materials.getTexture(first, MaterialTexture::Metallic) == info->textures[2]);
            CHECK(materials.getTexture(first, MaterialTexture::Roughness) == info->textures[2]);
            CHECK(materials.getTexture(first, MaterialTexture::AmbientOcclusion) ==
                    info->textures[2]);
            CHECK(!materials.getTexture(first, MaterialTexture::Normal).valid());
            CHECK(!materials.getTexture(first, MaterialTexture::Emission).valid());
            CHECK(firstInfo->metallicChannel == MaterialTextureChannel::Blue);
            CHECK(firstInfo->roughnessChannel == MaterialTextureChannel::Green);
            CHECK(firstInfo->aoChannel == MaterialTextureChannel::Red);

            CHECK(secondInfo->transparency == MaterialTransparency::AlphaBlend);
            CHECK(secondInfo->cull == MaterialCull::Back);
            CHECK(secondInfo->albedo.x == 1.0f && secondInfo->albedo.w == 1.0f);
            CHECK(materials.getTexture(second, MaterialTexture::Emission) == info->textures[1]);
            CHECK(materials.getTexture(second, MaterialTexture::Normal) == info->textures[2]);
            CHECK(!materials.getTexture(second, MaterialTexture::Albedo).valid());
            CHECK(!materials.getTexture(second, MaterialTexture::Metallic).valid());
        }
        else
            CHECK(false);
        const MeshInfo* surfaceOwner = meshes.get(info->meshes[0]);
        CHECK(surfaceOwner && surfaceOwner->surfaces.size() == 1 &&
                surfaceOwner->surfaces[0].material == first);
        const MeshInfo* plain = meshes.get(info->meshes[1]);
        CHECK(plain && plain->surfaces.size() == 1 && !plain->surfaces[0].material.valid());
    }
    if (infoA && infoA->surfaces.size() == 1)
    {
        const SurfaceInfo& surface = infoA->surfaces[0];
        CHECK(surface.ready);
        CHECK(surface.vertexCount == 4 && surface.indexCount == 6);
        CHECK(surface.vertexStride == 64);
        CHECK(surface.indexFormat == prisma::IndexFormat::UInt16);
        CHECK(surface.attributeMask ==
                (kVertexPosition | kVertexNormal | kVertexColor | kVertexTangent | kVertexUv));
        CHECK(infoA->pendingSurfaces == 0);
        CHECK(near(infoA->aabb.min.x, -0.5f) && near(infoA->aabb.min.y, -0.5f) &&
                near(infoA->aabb.min.z, 0.0f));
        CHECK(near(infoA->aabb.max.x, 0.5f) && near(infoA->aabb.max.y, 0.5f) &&
                near(infoA->aabb.max.z, 0.0f));
        Bytes bytes;
        CHECK(readBuffer(driver, surface.vertexBuffer, 0, 256, bytes));
        if (bytes.size() == 256)
        {
            float vertices[64];
            memcpy(vertices, bytes.data(), sizeof(vertices));
            bool same = true;
            for (int v = 0; v < 4; ++v)
            {
                const float* got = vertices + v * 16;
                for (int c = 0; c < 3; ++c) same = same && got[c] == kQuad[v][c];
                same = same && got[3] == 0.0f && got[4] == 0.0f && got[5] == 1.0f;
                for (int c = 0; c < 4; ++c)
                    same = same && got[6 + c] == (kQuadColor[v][c] == 255 ? 1.0f : 0.0f);
                same = same && near(got[10], 1.0f) && near(got[11], 0.0f) && near(got[12], 0.0f) &&
                       near(got[13], 1.0f);
                same = same && got[14] == kQuadUv[v][0] && got[15] == kQuadUv[v][1];
            }
            CHECK(same);
        }
        CHECK(readBuffer(driver, surface.indexBuffer, 0, 12, bytes));
        if (bytes.size() == 12)
        {
            std::uint16_t indices[6];
            memcpy(indices, bytes.data(), sizeof(indices));
            for (int i = 0; i < 6; ++i) CHECK(indices[i] == kQuadIndices[i]);
        }
    }
    else
        CHECK(false);

    if (infoB && infoB->surfaces.size() == 1)
    {
        const SurfaceInfo& surface = infoB->surfaces[0];
        CHECK(surface.ready);
        CHECK(surface.vertexCount == 3 && surface.indexCount == 3);
        CHECK(surface.vertexStride == 40);
        CHECK(surface.indexFormat == prisma::IndexFormat::UInt16);
        CHECK(near(infoB->aabb.max.x, 1.0f) && near(infoB->aabb.max.y, 1.0f) &&
                near(infoB->aabb.min.x, 0.0f));
        Bytes bytes;
        CHECK(readBuffer(driver, surface.vertexBuffer, 0, 120, bytes));
        if (bytes.size() == 120)
        {
            float vertices[30];
            memcpy(vertices, bytes.data(), sizeof(vertices));
            bool same = true;
            for (int v = 0; v < 3; ++v)
            {
                const float* got = vertices + v * 10;
                for (int c = 0; c < 3; ++c) same = same && got[c] == kTriangle[v][c];
                // No normals in the file: the face normal of the triangle, +Z.
                same = same && got[3] == 0.0f && got[4] == 0.0f && got[5] == 1.0f;
                for (int c = 0; c < 4; ++c) same = same && got[6 + c] == 1.0f;
            }
            CHECK(same);
        }
        CHECK(readBuffer(driver, surface.indexBuffer, 0, 6, bytes));
        if (bytes.size() == 6)
        {
            std::uint16_t indices[3];
            memcpy(indices, bytes.data(), sizeof(indices));
            for (int i = 0; i < 3; ++i) CHECK(indices[i] == kTriangleIndices[i]);
        }
    }
    else
        CHECK(false);

    const TextureId color = info->textures[0];
    const TextureId emissive = info->textures[1];
    const TextureId linear = info->textures[2];
    CHECK(color.valid() && linear.valid());
    CHECK(color == emissive);
    CHECK(linear != color);
    CHECK(loader.textureStatus(color) == LoadStatus::Loaded);
    CHECK(loader.textureStatus(linear) == LoadStatus::Loaded);
    const TextureInfo* colorInfo = textures.get(color);
    const TextureInfo* linearInfo = textures.get(linear);
    CHECK(colorInfo && linearInfo);
    if (colorInfo && linearInfo)
    {
        CHECK(colorInfo->srgb && colorInfo->format == prisma::TextureFormat::RGBA8Srgb);
        CHECK(!linearInfo->srgb && linearInfo->format == prisma::TextureFormat::RGBA8);
        CHECK(colorInfo->width == 4 && colorInfo->height == 2);
        CHECK(colorInfo->mipLevels == 3);
        CHECK(colorInfo->state == TextureState::Ready);
        CHECK(linearInfo->state == TextureState::Ready);
        Bytes pixels;
        CHECK(readTexture(textures, linear, 4, 2, pixels));
        bool same = pixels.size() == 32;
        for (int y = 0; same && y < 2; ++y)
            for (int x = 0; x < 4; ++x)
            {
                std::uint8_t want[4];
                pngPixel(x, y, want);
                const std::uint8_t* got = pixels.data() + (y * 4 + x) * 4;
                if (memcmp(got, want, 4) != 0)
                {
                    printf("texel %d,%d is %d,%d,%d,%d\n", x, y, got[0], got[1], got[2], got[3]);
                    same = false;
                    break;
                }
            }
        CHECK(same);
    }

    Node3D* model = instantiateModel(root, id);
    CHECK(model != nullptr);
    if (!model) return;
    CHECK(strcmp(model->getName().c_str(), "model") == 0);
    CHECK(model->getChildCount() == 1);
    Node3D* nodeR = model->getChildCount() == 1 ? model->getChild(0)->castTo<Node3D>() : nullptr;
    CHECK(nodeR != nullptr);
    if (nodeR)
    {
        CHECK(strcmp(nodeR->getName().c_str(), "R") == 0);
        CHECK(nodeR->castTo<MeshInstance3D>() == nullptr && nodeR->castTo<Camera3D>() == nullptr);
        CHECK(near(nodeR->getPosition().x, 1.0f) && near(nodeR->getPosition().y, 2.0f) &&
                near(nodeR->getPosition().z, 3.0f));
        CHECK(nodeR->getChildCount() == 2);
        MeshInstance3D* nodeM = nodeR->getChildCount() == 2
                                        ? nodeR->getChild(0)->castTo<MeshInstance3D>()
                                        : nullptr;
        Camera3D* nodeC =
                nodeR->getChildCount() == 2 ? nodeR->getChild(1)->castTo<Camera3D>() : nullptr;
        CHECK(nodeM != nullptr && nodeC != nullptr);
        if (nodeM)
        {
            CHECK(strcmp(nodeM->getName().c_str(), "M") == 0);
            CHECK(nodeM->getMesh() == meshA);
            CHECK(near(nodeM->getScale().x, 2.0f) && near(nodeM->getScale().z, 2.0f));
            CHECK(nodeM->getChildCount() == 1);
            MeshInstance3D* nodeN = nodeM->getChildCount() == 1
                                            ? nodeM->getChild(0)->castTo<MeshInstance3D>()
                                            : nullptr;
            CHECK(nodeN != nullptr);
            if (nodeN)
            {
                CHECK(strcmp(nodeN->getName().c_str(), "N") == 0);
                CHECK(nodeN->getMesh() == meshB);
                const Math::Mat4 global = nodeN->getGlobalTransform();
                const Math::Vec3 a = global.TransformPoint(Math::Vec3(1.0f, 0.0f, 0.0f));
                const Math::Vec3 b = global.TransformPoint(Math::Vec3(0.0f, 1.0f, 0.0f));
                CHECK(near(a.x, 1.0f) && near(a.y, 2.0f) && near(a.z, 1.0f));
                CHECK(near(b.x, 1.0f) && near(b.y, 4.0f) && near(b.z, 3.0f));
            }
        }
        if (nodeC)
        {
            CHECK(strcmp(nodeC->getName().c_str(), "C") == 0);
            CHECK(nodeC->getProjection() == CameraProjection::Perspective);
            CHECK(fabsf(nodeC->getFov() - 0.8f * 57.29577951f) < 0.01f);
            CHECK(near(nodeC->getNear(), 0.5f) && near(nodeC->getFar(), 100.0f));
        }
    }

    camera->setPosition(Math::Vec3(5.0f, 2.0f, 3.0f));
    camera->lookAt(Math::Vec3(1.0f, 2.0f, 3.0f));
    CHECK(camera->isCurrent());
    {
        Frame frame(tree);
        CHECK(pixelIsNot(32, 24, kClear, "model centre"));
        CHECK(pixelIs(1, 1, kClear, "model corner"));
    }

    model->queueFree();
    {
        Frame frame(tree);
        CHECK(pixelIs(32, 24, kClear, "model removed"));
    }
    (void) window;

    CHECK(loader.freeModel(id));
    CHECK(loader.model(id) == nullptr);
    CHECK(loader.modelStatus(id) == LoadStatus::Invalid);
    CHECK(!loader.freeModel(id));
    CHECK(meshes.valid(meshA) && meshes.valid(meshB));
    CHECK(textures.valid(color));
    meshes.destroy(meshA);
    meshes.destroy(meshB);
}

void testCollisionOption(AssetLoader& loader, const char* directory, SceneTree& tree)
{
    MeshStore& meshes = *Main::meshes();
    Node* root = tree.getRoot();
    const ct::String file = path(directory, "model.glb");

    ModelOptions options;
    options.collisionType = 2;
    const ModelId solid = loader.requestModel(file, options);
    const ModelId plain = loader.requestModel(file);
    CHECK(solid.valid() && plain.valid());
    loader.waitAll();
    CHECK(loader.modelStatus(solid) == LoadStatus::Loaded);
    CHECK(loader.modelStatus(plain) == LoadStatus::Loaded);
    CHECK(loader.pendingBytes() == 0);

    const ModelInfo* solidInfo = loader.model(solid);
    const ModelInfo* plainInfo = loader.model(plain);
    CHECK(solidInfo != nullptr && plainInfo != nullptr);
    if (!solidInfo || !plainInfo) return;
    CHECK(solidInfo->meshes.size() == 2 && plainInfo->meshes.size() == 2);
    if (solidInfo->meshes.size() != 2 || plainInfo->meshes.size() != 2) return;

    const CollisionMesh* quad = meshes.collision(solidInfo->meshes[0]);
    const CollisionMesh* triangle = meshes.collision(solidInfo->meshes[1]);
    CHECK(quad != nullptr && quad->valid() && quad->triangleCount() == 2);
    CHECK(triangle != nullptr && triangle->valid() && triangle->triangleCount() == 1);
    CHECK(meshes.collision(plainInfo->meshes[0]) == nullptr);
    CHECK(meshes.collision(plainInfo->meshes[1]) == nullptr);
    if (quad)
    {
        CHECK(near(quad->bounds().min.x, -0.5f) && near(quad->bounds().max.y, 0.5f));
        SweepHit hit;
        CHECK(quad->sweepSphere(Math::Vec3(0.0f, 0.0f, 1.0f), 0.1f, Math::Vec3(0.0f, 0.0f, -2.0f),
                hit));
        CHECK(near(hit.t, 0.45f) && near(hit.normal.z, 1.0f));
        CHECK(!quad->sweepSphere(Math::Vec3(3.0f, 0.0f, 1.0f), 0.1f, Math::Vec3(0.0f, 0.0f, -2.0f),
                hit));
    }

    Node3D* solidTree = instantiateModel(root, solid);
    Node3D* plainTree = instantiateModel(root, plain);
    CHECK(solidTree != nullptr && plainTree != nullptr);
    if (solidTree && plainTree)
    {
        MeshInstance3D* solidM = solidTree->getChild(0)->getChild(0)->castTo<MeshInstance3D>();
        MeshInstance3D* plainM = plainTree->getChild(0)->getChild(0)->castTo<MeshInstance3D>();
        CHECK(solidM != nullptr && plainM != nullptr);
        if (solidM && plainM)
        {
            CHECK(solidM->getCollisionType() == 2);
            CHECK(solidM->getCollisionShape() == CollisionShapeType::Mesh);
            CHECK(solidM->getCollisionMesh() == solidInfo->meshes[0]);
            CHECK(plainM->getCollisionType() == 0);
            CHECK(plainM->getCollisionShape() == CollisionShapeType::None);
        }
    }
    if (solidTree) solidTree->queueFree();
    if (plainTree) plainTree->queueFree();
    tree.process(1.0 / 60.0);
    CHECK(loader.freeModel(solid));
    CHECK(loader.freeModel(plain));
}

void testPendingSurface(SceneTree& tree)
{
    MeshStore& meshes = *Main::meshes();
    Node* root = tree.getRoot();

    SurfaceData quad;
    for (int i = 0; i < 4; ++i)
        quad.positions.push_back(Math::Vec3(kQuad[i][0] * 2.0f, kQuad[i][1] * 2.0f, 0.0f));
    for (int i = 0; i < 6; ++i) quad.indices.push_back(kQuadIndices[i]);
    SurfaceLayout layout;
    Bytes vertices;
    Bytes indices;
    CHECK(MeshStore::interleave(quad, layout, vertices, indices));
    CHECK(layout.vertexBytes() == 160 && layout.indexBytes() == 12);
    CHECK(vertices.size() == 160 && indices.size() == 12);

    const MeshId mesh = meshes.create("pending");
    const int surface = meshes.beginSurface(mesh, layout, "pending");
    CHECK(surface == 0);
    CHECK(!meshes.complete(mesh));
    CHECK(meshes.get(mesh)->pendingSurfaces == 1);
    CHECK(!meshes.get(mesh)->surfaces[0].ready);
    CHECK(meshes.uploadVertices(mesh, 0, 0, vertices.data(), 128));
    CHECK(!meshes.uploadVertices(mesh, 0, 128, vertices.data(), 64));
    CHECK(!meshes.uploadVertices(mesh, 1, 0, vertices.data(), 4));
    CHECK(!meshes.uploadIndices(mesh, 0, 8, indices.data(), 8));

    Node3D holder;
    MeshInstance3D* instance = holder.createChild<MeshInstance3D>();
    instance->setMesh(mesh);
    instance->reparent(root);
    instance->setPosition(Math::Vec3(1.0f, 2.0f, 3.0f));
    instance->setRotation(Math::Vec3(0.0f, 1.5707964f, 0.0f));
    {
        Frame frame(tree);
        CHECK(pixelIs(32, 24, kClear, "pending surface"));
    }

    CHECK(meshes.uploadVertices(mesh, 0, 128, vertices.data() + 128, 32));
    CHECK(meshes.uploadIndices(mesh, 0, 0, indices.data(), 12));
    CHECK(meshes.finishSurface(mesh, 0));
    CHECK(!meshes.finishSurface(mesh, 0));
    CHECK(!meshes.uploadVertices(mesh, 0, 0, vertices.data(), 4));
    CHECK(meshes.complete(mesh));
    CHECK(!meshes.complete(MeshId()));
    {
        Frame frame(tree);
        CHECK(pixelIsNot(32, 24, kClear, "finished surface"));
    }
    instance->queueFree();
    tree.process(1.0 / 60.0);
    meshes.destroy(mesh);

    SurfaceLayout empty;
    const MeshId other = meshes.create();
    CHECK(meshes.beginSurface(other, empty) == -1);
    CHECK(meshes.beginSurface(MeshId(), layout) == -1);
    meshes.destroy(other);
}

void testFailures(AssetLoader& loader, const char* directory)
{
    {
        const ModelId missing = loader.requestModel(path(directory, "missing.glb"));
        CHECK(missing.valid());
        loader.waitAll();
        CHECK(loader.modelStatus(missing) == LoadStatus::Failed);
        const ModelInfo* info = loader.model(missing);
        CHECK(info != nullptr && !info->error.empty() && !info->parsed);
        CHECK(loader.modelProgress(missing) == 0.0f);
        Node3D missingHolder;
        CHECK(instantiateModel(&missingHolder, missing) == nullptr);
        CHECK(loader.freeModel(missing));
        CHECK(!loader.requestModel("").valid());
    }

    {
        Bytes bin;
        Bytes data;
        for (int i = 0; i < 9; ++i) putF32(data, 0.0f);
        addView(bin, data);
        const ct::String json =
                "{\"asset\":{\"version\":\"2.0\"},\"buffers\":[{\"byteLength\":36}],"
                "\"bufferViews\":[{\"buffer\":0,\"byteOffset\":0,\"byteLength\":36}],"
                "\"accessors\":[{\"bufferView\":0,\"componentType\":5126,\"count\":100,\"type\":"
                "\"VEC3\"}],"
                "\"meshes\":[{\"primitives\":[{\"attributes\":{\"POSITION\":0}}]}]}";
        Bytes glb;
        buildGlb(json, bin, glb);
        const ct::String file = path(directory, "overrun.glb");
        CHECK(file_write(file.c_str(), glb.data(), glb.size()));
        const ModelId overrun = loader.requestModel(file);
        loader.waitAll();
        CHECK(loader.modelStatus(overrun) == LoadStatus::Failed);
        const ModelInfo* info = loader.model(overrun);
        CHECK(info != nullptr && !info->error.empty());
        CHECK(loader.freeModel(overrun));
        CHECK(fs_remove_path(file.c_str()));
    }

    {
        Bytes present;
        for (int i = 0; i < 3; ++i)
            for (int c = 0; c < 3; ++c) putF32(present, kTriangle[i][c]);
        const ct::String binFile = path(directory, "present.bin");
        CHECK(file_write(binFile.c_str(), present.data(), present.size()));
        const char* json =
                "{\"asset\":{\"version\":\"2.0\"},"
                "\"buffers\":[{\"uri\":\"present.bin\",\"byteLength\":36},"
                "{\"uri\":\"absent.bin\",\"byteLength\":36}],"
                "\"bufferViews\":[{\"buffer\":0,\"byteOffset\":0,\"byteLength\":36},"
                "{\"buffer\":1,\"byteOffset\":0,\"byteLength\":36}],"
                "\"accessors\":[{\"bufferView\":0,\"componentType\":5126,\"count\":3,\"type\":"
                "\"VEC3\"},{\"bufferView\":1,\"componentType\":5126,\"count\":3,\"type\":"
                "\"VEC3\"}],"
                "\"meshes\":[{\"name\":\"good\",\"primitives\":[{\"attributes\":{\"POSITION\":0}}]}"
                ","
                "{\"name\":\"bad\",\"primitives\":[{\"attributes\":{\"POSITION\":1}}]}]}";
        const ct::String file = path(directory, "partial.gltf");
        CHECK(file_write(file.c_str(), json, strlen(json)));
        const ModelId partial = loader.requestModel(file);
        loader.waitAll();
        CHECK(loader.modelStatus(partial) == LoadStatus::Failed);
        const ModelInfo* info = loader.model(partial);
        CHECK(info != nullptr);
        if (info)
        {
            CHECK(info->parsed);
            CHECK(info->failedJobs == 1);
            CHECK(info->pendingJobs == 0);
            CHECK(!info->error.empty());
            CHECK(info->meshes.size() == 2);
            if (info->meshes.size() == 2)
            {
                const MeshInfo* good = Main::meshes()->get(info->meshes[0]);
                CHECK(good != nullptr && good->surfaces.size() == 1 && good->surfaces[0].ready);
                CHECK(Main::meshes()->complete(info->meshes[0]));
                const MeshInfo* bad = Main::meshes()->get(info->meshes[1]);
                CHECK(bad != nullptr && bad->surfaces.empty());
            }
            CHECK(loader.modelProgress(partial) <= 1.0f);
            Node3D holder;
            Node3D* partialTree = instantiateModel(&holder, partial);
            CHECK(partialTree != nullptr);
        }
        CHECK(loader.freeModel(partial));
        CHECK(fs_remove_path(file.c_str()));
        CHECK(fs_remove_path(binFile.c_str()));
    }
    CHECK(loader.pendingRequests() == 0);
    CHECK(loader.pendingBytes() == 0);
}

bool writeFieldGlb(const ct::String& file)
{
    const int cells = 48;
    Bytes positions;
    Bytes indices;
    for (int y = 0; y <= cells; ++y)
    {
        for (int x = 0; x <= cells; ++x)
        {
            const float fx = static_cast<float>(x) * 0.25f;
            const float fy = static_cast<float>(y) * 0.25f;
            putF32(positions, fx);
            putF32(positions, fy);
            putF32(positions, 0.6f * sinf(fx * 0.9f) * cosf(fy * 0.7f));
        }
    }
    for (int y = 0; y < cells; ++y)
    {
        for (int x = 0; x < cells; ++x)
        {
            const std::uint32_t a = static_cast<std::uint32_t>(y * (cells + 1) + x);
            const std::uint32_t c = a + static_cast<std::uint32_t>(cells + 1);
            const std::uint32_t corners[6] = { a, a + 1, c + 1, a, c + 1, c };
            for (int i = 0; i < 6; ++i) putU32(indices, corners[i]);
        }
    }
    Bytes bin;
    const std::uint32_t positionsOffset = addView(bin, positions);
    const std::uint32_t indicesOffset = addView(bin, indices);
    ct::String json = "{\"asset\":{\"version\":\"2.0\"},";
    appendf(json, "\"buffers\":[{\"byteLength\":%u}],", static_cast<unsigned>(bin.size()));
    appendf(json,
            "\"bufferViews\":[{\"buffer\":0,\"byteOffset\":%u,\"byteLength\":%u},"
            "{\"buffer\":0,\"byteOffset\":%u,\"byteLength\":%u}],",
            positionsOffset, static_cast<unsigned>(positions.size()), indicesOffset,
            static_cast<unsigned>(indices.size()));
    appendf(json,
            "\"accessors\":[{\"bufferView\":0,\"componentType\":5126,\"count\":%u,"
            "\"type\":\"VEC3\"},{\"bufferView\":1,\"componentType\":5125,\"count\":%u,"
            "\"type\":\"SCALAR\"}],",
            static_cast<unsigned>(positions.size() / 12),
            static_cast<unsigned>(indices.size() / 4));
    json += "\"meshes\":[{\"name\":\"field\",\"primitives\":[{\"attributes\":{\"POSITION\":0},"
            "\"indices\":1}]}],\"nodes\":[{\"mesh\":0}],\"scenes\":[{\"nodes\":[0]}],\"scene\":0}";
    Bytes glb;
    buildGlb(json, bin, glb);
    return file_write(file.c_str(), glb.data(), glb.size());
}

void testLods(AssetLoader& loader, const char* directory)
{
    MeshStore& meshes = *Main::meshes();
    const ct::String file = path(directory, "field.glb");
    CHECK(writeFieldGlb(file));

    ModelOptions plainOptions;
    plainOptions.generateLods = false;
    const ModelId detailed = loader.requestModel(file);
    const ModelId plain = loader.requestModel(file, plainOptions);
    CHECK(detailed.valid() && plain.valid());
    loader.waitAll();
    CHECK(loader.modelStatus(detailed) == LoadStatus::Loaded);
    CHECK(loader.modelStatus(plain) == LoadStatus::Loaded);
    CHECK(loader.pendingBytes() == 0);

    const ModelInfo* detailedInfo = loader.model(detailed);
    const ModelInfo* plainInfo = loader.model(plain);
    CHECK(detailedInfo != nullptr && plainInfo != nullptr);
    if (!detailedInfo || !plainInfo) return;
    CHECK(detailedInfo->meshes.size() == 1 && plainInfo->meshes.size() == 1);
    if (detailedInfo->meshes.size() != 1 || plainInfo->meshes.size() != 1) return;

    const MeshInfo* plainMesh = meshes.get(plainInfo->meshes[0]);
    CHECK(plainMesh != nullptr && plainMesh->surfaces.size() == 1);
    if (plainMesh && plainMesh->surfaces.size() == 1) CHECK(plainMesh->surfaces[0].lods.empty());
    CHECK(plainInfo->lodLevels == 0 && plainInfo->lodBytes == 0 && plainInfo->lodSeconds == 0.0);

    const MeshInfo* mesh = meshes.get(detailedInfo->meshes[0]);
    CHECK(mesh != nullptr && mesh->surfaces.size() == 1);
    if (!mesh || mesh->surfaces.size() != 1) return;
    const SurfaceInfo& surface = mesh->surfaces[0];
    CHECK(surface.ready);
    CHECK(surface.indexCount == 48 * 48 * 6);
    CHECK(surface.lods.size() >= 3);
    CHECK(detailedInfo->lodLevels == surface.lods.size());
    CHECK(detailedInfo->totalBytes == plainInfo->totalBytes);
    CHECK(detailedInfo->uploadedBytes == plainInfo->uploadedBytes);
    std::uint64_t bytes = 0;
    std::uint32_t previousCount = surface.indexCount;
    float previousEdge = 0.0f;
    const std::uint32_t indexSize = surface.indexFormat == prisma::IndexFormat::UInt16 ? 2u : 4u;
    for (std::size_t l = 0; l < surface.lods.size(); ++l)
    {
        const SurfaceLodInfo& lod = surface.lods[l];
        CHECK(lod.indexBuffer.valid());
        CHECK(lod.indexCount > 0 && lod.indexCount % 3 == 0 && lod.indexCount < previousCount);
        CHECK(lod.edgeLength > 0.0f && lod.edgeLength >= previousEdge);
        previousCount = lod.indexCount;
        previousEdge = lod.edgeLength;
        bytes += static_cast<std::uint64_t>(lod.indexCount) * indexSize;
    }
    CHECK(detailedInfo->lodBytes == bytes);
    CHECK(detailedInfo->lodSeconds > 0.0);
    if (!surface.lods.empty())
    {
        Bytes first;
        const std::uint32_t firstBytes = surface.lods[0].indexCount * indexSize;
        CHECK(readBuffer(Main::device()->driver(), surface.lods[0].indexBuffer, 0, firstBytes,
                first));
        bool inside = true;
        for (std::uint32_t i = 0; i < surface.lods[0].indexCount; ++i)
        {
            std::uint32_t index = 0;
            memcpy(&index, first.data() + static_cast<std::size_t>(i) * indexSize, indexSize);
            if (index >= surface.vertexCount) inside = false;
        }
        CHECK(inside);
    }
    printf("field model: %u levels, %llu bytes, generated in %.2f ms\n", detailedInfo->lodLevels,
            static_cast<unsigned long long>(detailedInfo->lodBytes),
            detailedInfo->lodSeconds * 1000.0);
    CHECK(fs_remove_path(file.c_str()));
}

const std::uint32_t kOrderedTriangles = 7;

bool writeOrderedGlb(const ct::String& file)
{
    const int cells = 48;
    Bytes positions;
    Bytes indices;
    for (int y = 0; y <= cells; ++y)
    {
        for (int x = 0; x <= cells; ++x)
        {
            const float fx = static_cast<float>(x) * 0.25f;
            const float fy = static_cast<float>(y) * 0.25f;
            putF32(positions, fx);
            putF32(positions, fy);
            putF32(positions, 0.6f * sinf(fx * 0.9f) * cosf(fy * 0.7f));
        }
    }
    for (int y = 0; y < cells; ++y)
    {
        for (int x = 0; x < cells; ++x)
        {
            const std::uint32_t a = static_cast<std::uint32_t>(y * (cells + 1) + x);
            const std::uint32_t c = a + static_cast<std::uint32_t>(cells + 1);
            const std::uint32_t corners[6] = { a, a + 1, c + 1, a, c + 1, c };
            for (int i = 0; i < 6; ++i) putU32(indices, corners[i]);
        }
    }
    Bytes bin;
    const std::uint32_t positionsOffset = addView(bin, positions);
    const std::uint32_t indicesOffset = addView(bin, indices);
    std::uint32_t triangleOffsets[kOrderedTriangles];
    for (std::uint32_t t = 0; t < kOrderedTriangles; ++t)
    {
        Bytes triangle;
        const float x = 100.0f * static_cast<float>(t + 1);
        const float corners[3][3] = { { x, 0.0f, 0.0f }, { x + 1.0f, 0.0f, 0.0f },
            { x, 1.0f, 0.0f } };
        for (int v = 0; v < 3; ++v)
            for (int c = 0; c < 3; ++c) putF32(triangle, corners[v][c]);
        triangleOffsets[t] = addView(bin, triangle);
    }

    ct::String json = "{\"asset\":{\"version\":\"2.0\"},";
    appendf(json, "\"buffers\":[{\"byteLength\":%u}],", static_cast<unsigned>(bin.size()));
    appendf(json,
            "\"bufferViews\":[{\"buffer\":0,\"byteOffset\":%u,\"byteLength\":%u},"
            "{\"buffer\":0,\"byteOffset\":%u,\"byteLength\":%u}",
            positionsOffset, static_cast<unsigned>(positions.size()), indicesOffset,
            static_cast<unsigned>(indices.size()));
    for (std::uint32_t t = 0; t < kOrderedTriangles; ++t)
        appendf(json, ",{\"buffer\":0,\"byteOffset\":%u,\"byteLength\":36}", triangleOffsets[t]);
    appendf(json,
            "],\"accessors\":[{\"bufferView\":0,\"componentType\":5126,\"count\":%u,"
            "\"type\":\"VEC3\"},{\"bufferView\":1,\"componentType\":5125,\"count\":%u,"
            "\"type\":\"SCALAR\"}",
            static_cast<unsigned>(positions.size() / 12),
            static_cast<unsigned>(indices.size() / 4));
    for (std::uint32_t t = 0; t < kOrderedTriangles; ++t)
        appendf(json, ",{\"bufferView\":%u,\"componentType\":5126,\"count\":3,\"type\":\"VEC3\"}",
                t + 2);
    json += "],\"meshes\":[{\"name\":\"ordered\",\"primitives\":["
            "{\"attributes\":{\"POSITION\":0},\"indices\":1}";
    for (std::uint32_t t = 0; t < kOrderedTriangles; ++t)
        appendf(json, ",{\"attributes\":{\"POSITION\":%u}}", t + 2);
    json += "]}],\"nodes\":[{\"mesh\":0}],\"scenes\":[{\"nodes\":[0]}],\"scene\":0}";
    Bytes glb;
    buildGlb(json, bin, glb);
    return file_write(file.c_str(), glb.data(), glb.size());
}

bool orderedSurfaces(const MeshInfo* mesh)
{
    if (!mesh || mesh->surfaces.size() != kOrderedTriangles + 1) return false;
    if (mesh->surfaces[0].indexCount != 48 * 48 * 6) return false;
    for (std::uint32_t t = 0; t < kOrderedTriangles; ++t)
    {
        const SurfaceInfo& surface = mesh->surfaces[t + 1];
        if (surface.vertexCount != 3 ||
                !near(surface.aabb.min.x, 100.0f * static_cast<float>(t + 1)))
            return false;
    }
    return true;
}

void testSurfaceOrder(AssetLoader& loader, const char* directory)
{
    MeshStore& meshes = *Main::meshes();
    const ct::String file = path(directory, "ordered.glb");
    CHECK(writeOrderedGlb(file));

    const int rounds = 30;
    ct::Vector<ModelId> models;
    for (int i = 0; i < rounds; ++i) models.push_back(loader.requestModel(file));
    loader.waitAll();
    int ordered = 0;
    for (int i = 0; i < rounds; ++i)
    {
        const ModelInfo* info = loader.model(models[static_cast<std::size_t>(i)]);
        CHECK(info != nullptr && info->status == LoadStatus::Loaded && info->meshes.size() == 1);
        if (!info || info->meshes.size() != 1) continue;
        CHECK(info->nextPrimitives.size() == 1 && info->nextPrimitives[0] == kOrderedTriangles + 1);
        if (orderedSurfaces(meshes.get(info->meshes[0]))) ++ordered;
    }
    if (ordered != rounds) printf("surface order: %d of %d loads in file order\n", ordered, rounds);
    CHECK(ordered == rounds);
    CHECK(loader.pendingBytes() == 0);

    models.clear();
    for (int i = 0; i < 8; ++i) models.push_back(loader.requestModel(file));
    const std::uint64_t start = time_nanos();
    bool finished = false;
    while (!finished && time_nanos() - start < secondsToNanos(60))
    {
        loader.pump(4096, 1);
        finished = true;
        for (std::size_t i = 0; i < models.size(); ++i)
            if (loader.modelStatus(models[i]) == LoadStatus::InProgress) finished = false;
        if (!finished) time_sleep(1);
    }
    CHECK(finished);
    ordered = 0;
    for (std::size_t i = 0; i < models.size(); ++i)
    {
        const ModelInfo* info = loader.model(models[i]);
        CHECK(info != nullptr && info->status == LoadStatus::Loaded);
        if (info && info->meshes.size() == 1 && orderedSurfaces(meshes.get(info->meshes[0])))
            ++ordered;
    }
    CHECK(ordered == static_cast<int>(models.size()));
    CHECK(loader.pendingBytes() == 0);
    CHECK(fs_remove_path(file.c_str()));
}

void testBigModel(AssetLoader& loader, const ct::String& file)
{
    MeshStore& meshes = *Main::meshes();
    prisma::Driver* driver = Main::device()->driver();
    const std::uint64_t limit = 32 * kMiB;
    loader.setPendingByteLimit(limit);
    CHECK(loader.pendingByteLimit() == limit);

    const std::uint64_t wallStart = time_nanos();
    // One surface per primitive: the streaming is what is measured, not the grid split.
    ModelOptions whole;
    whole.splitCellSize = 0.0f;
    const ModelId id = loader.requestModel(file, whole);
    CHECK(id.valid());
    const ModelInfo* info = loader.model(id);
    CHECK(info != nullptr);
    if (!info) return;

    std::uint64_t maxPump = 0;
    std::uint64_t sumPump = 0;
    std::uint64_t pumps = 0;
    std::uint64_t maxPending = 0;
    bool finished = false;
    while (time_nanos() - wallStart < secondsToNanos(120))
    {
        const std::uint64_t begin = time_nanos();
        loader.pump(4 * kMiB);
        const std::uint64_t elapsed = time_nanos() - begin;
        if (elapsed > maxPump) maxPump = elapsed;
        sumPump += elapsed;
        ++pumps;
        const std::uint64_t pending = loader.pendingBytes();
        if (pending > maxPending) maxPending = pending;
        CHECK(pending <= limit);
        if (info->status != LoadStatus::InProgress)
        {
            finished = true;
            break;
        }
        time_sleep(1);
    }
    const std::uint64_t wall = time_nanos() - wallStart;
    CHECK(finished);
    CHECK(loader.modelStatus(id) == LoadStatus::Loaded);
    printf("big model: %llu pumps, max pump %.3f ms, average pump %.3f ms, max pending %.1f MiB, "
           "wall %.1f ms\n",
            static_cast<unsigned long long>(pumps), millis(maxPump),
            pumps ? millis(sumPump) / static_cast<double>(pumps) : 0.0,
            static_cast<double>(maxPending) / static_cast<double>(kMiB), millis(wall));
    CHECK(maxPump < 250ull * 1000 * 1000);
    CHECK(maxPending > 0);
    CHECK(loader.pendingBytes() == 0);
    CHECK(info->failedJobs == 0 && info->pendingJobs == 0);
    CHECK(info->meshes.size() == 1);
    if (info->meshes.size() != 1) return;
    CHECK(info->uploadedBytes == info->totalBytes);
    CHECK(info->totalBytes == static_cast<std::uint64_t>(kBigPrimitives) * kBigVertices * 40);

    const MeshInfo* mesh = meshes.get(info->meshes[0]);
    CHECK(mesh != nullptr && meshes.complete(info->meshes[0]));
    if (mesh && mesh->surfaces.size() == kBigPrimitives)
    {
        bool seen[kBigPrimitives] = { false };
        std::uint32_t checked = 0;
        for (std::uint32_t s = 0; s < kBigPrimitives; ++s)
        {
            const SurfaceInfo& surface = mesh->surfaces[s];
            CHECK(surface.ready);
            CHECK(surface.vertexCount == kBigVertices && surface.indexCount == 0);
            CHECK(!surface.indexBuffer.valid());
            Bytes first;
            CHECK(readBuffer(driver, surface.vertexBuffer, 0, 40, first));
            if (first.size() != 40) continue;
            float y;
            memcpy(&y, first.data() + 4, sizeof(y));
            const std::uint32_t prim = static_cast<std::uint32_t>(y);
            CHECK(prim < kBigPrimitives && !seen[prim]);
            if (prim >= kBigPrimitives) continue;
            seen[prim] = true;
            if (checked >= 2) continue;
            ++checked;
            Bytes sample;
            const std::uint32_t middle = kBigVertices / 2;
            const std::uint32_t last = kBigVertices - 1;
            const std::uint32_t indices[3] = { 0, middle, last };
            for (int k = 0; k < 3; ++k)
            {
                CHECK(readBuffer(driver, surface.vertexBuffer, indices[k] * 40, 40, sample));
                if (sample.size() == 40) checkVertex(sample, 0, indices[k], prim, "big model");
            }
        }
        for (std::uint32_t p = 0; p < kBigPrimitives; ++p) CHECK(seen[p]);
        CHECK(checked == 2);
    }
    else
        CHECK(false);

    meshes.destroy(info->meshes[0]);
    CHECK(loader.freeModel(id));

    loader.setPendingByteLimit(limit);
    const std::uint64_t waitStart = time_nanos();
    const ModelId again = loader.requestModel(file, whole);
    CHECK(again.valid());
    loader.waitAll();
    CHECK(loader.modelStatus(again) == LoadStatus::Loaded);
    CHECK(loader.pendingBytes() == 0);
    CHECK(loader.pendingRequests() == 0);
    const ModelInfo* second = loader.model(again);
    CHECK(second != nullptr && second->meshes.size() == 1);
    if (second && second->meshes.size() == 1)
    {
        CHECK(meshes.complete(second->meshes[0]));
        const MeshInfo* secondMesh = meshes.get(second->meshes[0]);
        CHECK(secondMesh && secondMesh->surfaces.size() == kBigPrimitives);
        meshes.destroy(second->meshes[0]);
    }
    printf("big model waitAll: %.1f ms\n", millis(time_nanos() - waitStart));
    CHECK(loader.freeModel(again));
    loader.setPendingByteLimit(AssetLoader::kDefaultPendingByteLimit);
}

} // namespace

int main(int argc, char** argv)
{
    const bool vulkan = argc > 1 && strcmp(argv[1], "vulkan") == 0;

    CHECK(Device::headlessSupported());
    if (!Device::headlessSupported()) return 1;

    DeviceDesc desc;
    desc.title = "model loader test";
    desc.driver = vulkan ? prisma::DriverType::Vulkan : prisma::DriverType::OpenGL;
    desc.width = kWidth;
    desc.height = kHeight;
    desc.headless = true;

    CHECK(Main::setup(desc));
    if (!Main::device()) return 1;
    printf("%s\n", vulkan ? "Vulkan" : "OpenGL");

    const char* directory =
            vulkan ? PRISMA3D_TEST_DIR "/model_loader_vulkan" : PRISMA3D_TEST_DIR "/model_loader";
    fs_create_directory(directory);
    FileSystem::getSingleton().addSearchPath(directory);

    SceneTree tree;
    CHECK(Main::start(&tree));
    Node* root = tree.getRoot();
    Window* window = root ? root->castTo<Window>() : nullptr;
    CHECK(window != nullptr);
    if (!window) return 1;
    window->setClearColor(Math::Vec4(0.1f, 0.2f, 0.3f, 1.0f));
    RenderingServer::singleton()->scenarioSetEnvironment(window->getScenario(), whiteAmbient());
    Camera3D* camera = root->createChild<Camera3D>();
    camera->setPosition(Math::Vec3(5.0f, 2.0f, 3.0f));
    camera->makeCurrent();

    CHECK(Main::models() != nullptr && Main::models() == ModelLoader::singleton());
    CHECK(Main::models() != nullptr && Main::models()->importerCount() == 8);
    AssetLoader* loader = Main::assets();
    CHECK(loader != nullptr && AssetLoader::singleton() == loader);
    if (!loader) return 1;

    testSmallModel(*loader, directory, tree, window, camera);
    testCollisionOption(*loader, directory, tree);
    testPendingSurface(tree);
    testFailures(*loader, directory);
    testLods(*loader, directory);
    testSurfaceOrder(*loader, directory);

    const ct::String bigFile = path(directory, "big.glb");
    const std::uint64_t writeStart = time_nanos();
    const bool written = writeBigGlb(bigFile);
    CHECK(written);
    printf("big model file written in %.1f ms\n", millis(time_nanos() - writeStart));
    if (written)
    {
        testBigModel(*loader, bigFile);

        AssetLoader* local = new AssetLoader(*Main::textures(), *Main::materials(), *Main::meshes(),
                *Main::models(), 0);
        CHECK(AssetLoader::singleton() == local);
        local->setPendingByteLimit(8 * kMiB);
        const ModelId id = local->requestModel(bigFile);
        CHECK(id.valid());
        for (int i = 0; i < 2; ++i)
        {
            local->pump(1 * kMiB);
            time_sleep(20);
        }
        const std::uint64_t destroyStart = time_nanos();
        delete local;
        const std::uint64_t destroyTime = time_nanos() - destroyStart;
        printf("loader destroyed with work in flight in %.1f ms\n", millis(destroyTime));
        CHECK(destroyTime < secondsToNanos(10));
        CHECK(AssetLoader::singleton() == nullptr);
    }

    fs_remove_path(bigFile.c_str());
    fs_remove_path(path(directory, "pixels.png").c_str());
    fs_remove_path(path(directory, "model.glb").c_str());

    camera->queueFree();
    tree.finalize();
    Main::cleanup();
    fs_remove_path(directory);

    if (failures)
    {
        printf("%d checks failed\n", failures);
        return 1;
    }
    printf("model loader ok\n");
    return 0;
}
