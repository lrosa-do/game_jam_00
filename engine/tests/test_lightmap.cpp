#include "Check.h"
#include "LitTest.h"
#include "TestFiles.h"

#include "prisma3d/Device.h"
#include "prisma3d/core/FileSystem.h"
#include "prisma3d/core/Pixmap.h"
#include "prisma3d/main/Main.h"
#include "prisma3d/render/MaterialStore.h"
#include "prisma3d/render/MeshStore.h"
#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/render/TextureStore.h"
#include "prisma3d/scene/Camera3D.h"
#include "prisma3d/scene/Light3D.h"
#include "prisma3d/scene/LightmapGI.h"
#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/SceneSerializer.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/Viewport.h"
#include "prisma3d/scene/VisualInstance3D.h"
#include "prisma3d/scene/WorldEnvironment.h"

#include <ct/vector.hpp>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

using namespace prisma3d;

namespace
{

const int kWidth = 160;
const int kHeight = 120;
const float kFov = 60.0f;
const float kPlaneDepth = 5.0f;
const float kPlaneHalfWidth = 4.0f;
const float kPlaneHalfHeight = 3.0f;
const int kTolerance = 2;

struct Colour
{
    float r;
    float g;
    float b;
};

struct Image
{
    int width = 0;
    int height = 0;
    ct::Vector<std::uint8_t> pixels;

    int channel(int x, int y, int c) const
    {
        return pixels[(static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
                              static_cast<std::size_t>(x)) *
                              4 +
                      static_cast<std::size_t>(c)];
    }

    bool same(const Image& other) const
    {
        return width == other.width && height == other.height &&
               pixels.size() == other.pixels.size() &&
               memcmp(pixels.data(), other.pixels.data(), pixels.size()) == 0;
    }
};

float linearToSrgb(float value)
{
    if (value <= 0.0f) return 0.0f;
    if (value >= 1.0f) return 1.0f;
    return value <= 0.0031308f ? value * 12.92f : 1.055f * powf(value, 1.0f / 2.4f) - 0.055f;
}

int encode(float linear)
{
    return static_cast<int>(floorf(linearToSrgb(linear) * 255.0f + 0.5f));
}

float clampf(float value, float low, float high)
{
    return value < low ? low : (value > high ? high : value);
}

struct LayerImage
{
    int width = 0;
    int height = 0;
    int layers = 0;
    ct::Vector<float> rgb;

    Colour texel(int layer, int x, int y) const
    {
        const std::size_t at =
                ((static_cast<std::size_t>(layer) * static_cast<std::size_t>(height) +
                         static_cast<std::size_t>(y)) *
                                static_cast<std::size_t>(width) +
                        static_cast<std::size_t>(x)) *
                3;
        return Colour{ rgb[at], rgb[at + 1], rgb[at + 2] };
    }

    Colour sample(int layer, float u, float v) const
    {
        const float fx =
                clampf(u * static_cast<float>(width) - 0.5f, 0.0f, static_cast<float>(width - 1));
        const float fy =
                clampf(v * static_cast<float>(height) - 0.5f, 0.0f, static_cast<float>(height - 1));
        const int x0 = static_cast<int>(floorf(fx));
        const int y0 = static_cast<int>(floorf(fy));
        const int x1 = x0 + 1 < width ? x0 + 1 : x0;
        const int y1 = y0 + 1 < height ? y0 + 1 : y0;
        const float tx = fx - static_cast<float>(x0);
        const float ty = fy - static_cast<float>(y0);
        const Colour a = texel(layer, x0, y0);
        const Colour b = texel(layer, x1, y0);
        const Colour c = texel(layer, x0, y1);
        const Colour d = texel(layer, x1, y1);
        auto mix = [&](float p, float q, float r, float t)
        { return (p * (1.0f - tx) + q * tx) * (1.0f - ty) + (r * (1.0f - tx) + t * tx) * ty; };
        return Colour{ mix(a.r, b.r, c.r, d.r), mix(a.g, b.g, c.g, d.g), mix(a.b, b.b, c.b, d.b) };
    }
};

struct Pixels
{
    int width = 0;
    int height = 0;
    int layers = 0;
    ct::Vector<std::uint16_t> halves;
    LayerImage decoded;

    const std::uint16_t* data() const
    {
        return halves.data();
    }
};

template<typename F>
Pixels makePixels(int width, int height, int layers, F colourAt)
{
    Pixels result;
    result.width = width;
    result.height = height;
    result.layers = layers;
    result.decoded.width = width;
    result.decoded.height = height;
    result.decoded.layers = layers;
    result.halves.resize(static_cast<std::size_t>(width) * static_cast<std::size_t>(height) *
                         static_cast<std::size_t>(layers) * 4);
    result.decoded.rgb.resize(static_cast<std::size_t>(width) * static_cast<std::size_t>(height) *
                              static_cast<std::size_t>(layers) * 3);
    std::size_t at = 0;
    std::size_t decoded = 0;
    for (int layer = 0; layer < layers; ++layer)
    {
        for (int y = 0; y < height; ++y)
        {
            for (int x = 0; x < width; ++x)
            {
                const Colour c = colourAt(layer, x, y, width, height);
                const std::uint16_t r = LightmapGIData::floatToHalf(c.r);
                const std::uint16_t g = LightmapGIData::floatToHalf(c.g);
                const std::uint16_t b = LightmapGIData::floatToHalf(c.b);
                result.halves[at++] = r;
                result.halves[at++] = g;
                result.halves[at++] = b;
                result.halves[at++] = LightmapGIData::floatToHalf(1.0f);
                result.decoded.rgb[decoded++] = LightmapGIData::halfToFloat(r);
                result.decoded.rgb[decoded++] = LightmapGIData::halfToFloat(g);
                result.decoded.rgb[decoded++] = LightmapGIData::halfToFloat(b);
            }
        }
    }
    return result;
}

struct Rig
{
    RenderingServer* server = nullptr;
    prisma::Driver* driver = nullptr;
    TextureStore* textures = nullptr;
    ScenarioId scenario;
    CameraId camera;
    ViewportId view;
    EnvironmentSettings env;
    RenderStats stats;
    int width = kWidth;
    int height = kHeight;

    void apply()
    {
        server->scenarioSetEnvironment(scenario, env);
    }

    bool capture(Image& image, int primes = 2)
    {
        for (int i = 0; i < primes; ++i)
        {
            server->renderViewports();
            server->endFrame(true);
        }
        server->renderViewports();
        stats = server->getStats();
        server->endFrame(true);
        image.width = width;
        image.height = height;
        image.pixels.resize(static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 4);
        prisma::RenderTarget source;
        source.texture = textures->gpu(server->viewportGetTexture(view));
        prisma::Rect rect;
        rect.width = static_cast<std::uint32_t>(width);
        rect.height = static_cast<std::uint32_t>(height);
        return driver->readPixels(source, rect, image.pixels.data());
    }

    float focal() const
    {
        return static_cast<float>(height) * 0.5f / tanf(kFov * 0.5f * 3.14159265358979f / 180.0f);
    }

    float worldX(int x) const
    {
        return (static_cast<float>(x) + 0.5f - static_cast<float>(width) * 0.5f) / focal() *
               kPlaneDepth;
    }

    float worldY(int y) const
    {
        return (static_cast<float>(y) + 0.5f - static_cast<float>(height) * 0.5f) / focal() *
               kPlaneDepth;
    }

    float planeU(int x) const
    {
        return (worldX(x) + kPlaneHalfWidth) / (2.0f * kPlaneHalfWidth);
    }

    float planeV(int y) const
    {
        return (worldY(y) + kPlaneHalfHeight) / (2.0f * kPlaneHalfHeight);
    }
};

SurfaceData planeSurface(const Colour& albedo, bool withUv2, float x0, float x1)
{
    SurfaceData data;
    const Math::Vec4 colour(albedo.r, albedo.g, albedo.b, 1.0f);
    const float y = kPlaneHalfHeight;
    const Math::Vec3 corners[4] = { Math::Vec3(x0, -y, -kPlaneDepth),
        Math::Vec3(x1, -y, -kPlaneDepth), Math::Vec3(x1, y, -kPlaneDepth),
        Math::Vec3(x0, y, -kPlaneDepth) };
    const Math::Vec2 uvs[4] = { Math::Vec2(0.0f, 0.0f), Math::Vec2(1.0f, 0.0f),
        Math::Vec2(1.0f, 1.0f), Math::Vec2(0.0f, 1.0f) };
    for (int i = 0; i < 4; ++i)
    {
        data.positions.push_back(corners[i]);
        data.normals.push_back(Math::Vec3(0.0f, 0.0f, 1.0f));
        data.colors.push_back(colour);
        data.uvs.push_back(uvs[i]);
        if (withUv2) data.uvs2.push_back(uvs[i]);
    }
    const std::uint32_t indices[6] = { 0, 1, 2, 0, 2, 3 };
    for (std::uint32_t index: indices) data.indices.push_back(index);
    return data;
}

int worstError(const Image& image, const Rig& rig, const Colour& albedo,
        const ct::Function<Colour(float, float)>& lightAt, int x0, int x1,
        int* worstChannel = nullptr)
{
    int worst = 0;
    for (int y = 0; y < image.height; ++y)
    {
        for (int x = x0; x < x1; ++x)
        {
            const Colour light = lightAt(rig.planeU(x), rig.planeV(y));
            const int expected[3] = { encode(albedo.r * light.r), encode(albedo.g * light.g),
                encode(albedo.b * light.b) };
            for (int c = 0; c < 3; ++c)
            {
                const int d = abs(image.channel(x, y, c) - expected[c]);
                if (d > worst)
                {
                    worst = d;
                    if (worstChannel) *worstChannel = c;
                }
            }
        }
    }
    return worst;
}

int maxDifference(const Image& a, const Image& b)
{
    int worst = 0;
    for (std::size_t i = 0; i < a.pixels.size(); ++i)
    {
        if ((i & 3) == 3) continue;
        const int d = abs(static_cast<int>(a.pixels[i]) - static_cast<int>(b.pixels[i]));
        if (d > worst) worst = d;
    }
    return worst;
}

void dump(const Image& image, const char* name, const char* tag)
{
    const char* directory = getenv("PRISMA3D_SHOTS");
    if (!directory || !directory[0]) return;
    Pixmap pixmap(image.width, image.height, 4);
    for (int y = 0; y < image.height; ++y)
        for (int x = 0; x < image.width; ++x)
            pixmap.set_pixel(static_cast<std::uint32_t>(x),
                    static_cast<std::uint32_t>(image.height - 1 - y),
                    static_cast<std::uint8_t>(image.channel(x, y, 0)),
                    static_cast<std::uint8_t>(image.channel(x, y, 1)),
                    static_cast<std::uint8_t>(image.channel(x, y, 2)), 255);
    char path[512];
    snprintf(path, sizeof(path), "%s/%s_%s.png", directory, name, tag);
    pixmap.save(path);
}

EnvironmentSettings ambientColour(const Colour& linear)
{
    EnvironmentSettings env = whiteAmbient();
    env.ambientColor =
            Math::Vec3(linearToSrgb(linear.r), linearToSrgb(linear.g), linearToSrgb(linear.b));
    env.background = BackgroundMode::Color;
    env.backgroundColor = Math::Vec3(0.0f, 0.0f, 0.0f);
    return env;
}

EnvironmentSettings ambientNone()
{
    EnvironmentSettings env = whiteAmbient();
    env.ambientSource = AmbientSource::Disabled;
    env.background = BackgroundMode::Color;
    env.backgroundColor = Math::Vec3(0.0f, 0.0f, 0.0f);
    return env;
}

const Colour kAlbedo{ 0.8f, 0.6f, 0.4f };
const Colour kFlat{ 0.25f, 0.5f, 0.1f };
const Math::Vec4 kFullRect(0.0f, 0.0f, 1.0f, 1.0f);

struct Scene
{
    Rig rig;
    MeshId whole;
    MeshId left;
    MeshId right;
    MeshId plain;
    InstanceId plane;
    InstanceId leftPlane;
    InstanceId rightPlane;
    const char* tag = "";
};

struct Fixture
{
    LightmapGIData data;
    LightmapId lightmap;
    Pixels pixels;
};

void hideAll(Scene& scene)
{
    scene.rig.server->instanceSetVisible(scene.plane, false);
    scene.rig.server->instanceSetVisible(scene.leftPlane, false);
    scene.rig.server->instanceSetVisible(scene.rightPlane, false);
}

void showWhole(Scene& scene)
{
    hideAll(scene);
    scene.rig.server->instanceSetVisible(scene.plane, true);
}

void showHalves(Scene& scene)
{
    hideAll(scene);
    scene.rig.server->instanceSetVisible(scene.leftPlane, true);
    scene.rig.server->instanceSetVisible(scene.rightPlane, true);
}

LightmapId makeLightmap(Scene& scene, LightmapGIData& data, bool sh = false)
{
    RenderingServer& server = *scene.rig.server;
    const LightmapId lightmap = server.lightmapCreate();
    server.lightmapSetScenario(lightmap, scene.rig.scenario);
    server.lightmapSetTextures(lightmap, data.getLightTexture(), sh);
    return lightmap;
}

void testDefaultsAndFlat(Scene& scene)
{
    printf("a lightmap replaces the ambient light, times the albedo\n");
    Rig& rig = scene.rig;
    RenderingServer& server = *rig.server;
    showWhole(scene);

    rig.env = ambientColour(Colour{ 1.0f, 1.0f, 1.0f });
    rig.apply();
    Image plain;
    CHECK(rig.capture(plain));
    CHECK(rig.stats.lightmapSurfaces == 0 && rig.stats.lightmapsUsed == 0);
    CHECK(plain.channel(80, 60, 0) == encode(kAlbedo.r) &&
            plain.channel(80, 60, 1) == encode(kAlbedo.g));
    dump(plain, "plain_ambient", scene.tag);

    rig.env = ambientColour(kFlat);
    rig.apply();
    Image reference;
    CHECK(rig.capture(reference));
    CHECK(rig.stats.lightmapSurfaces == 0);

    Fixture f;
    f.pixels = makePixels(8, 8, 1, [](int, int, int, int, int) { return kFlat; });
    CHECK(f.data.setLightImages(8, 8, 1, f.pixels.data()));
    f.lightmap = makeLightmap(scene, f.data);

    rig.env = ambientColour(Colour{ 1.0f, 1.0f, 1.0f });
    rig.apply();
    Image unassigned;
    CHECK(rig.capture(unassigned));
    CHECK(unassigned.same(plain));

    server.instanceGeometrySetLightmap(scene.plane, f.lightmap, kFullRect, 0);
    Image lightmapped;
    CHECK(rig.capture(lightmapped));
    CHECK(rig.stats.lightmapSurfaces == 1 && rig.stats.lightmapsUsed == 1);
    printf("  pixel %d %d %d, ambient reference %d %d %d, difference %d\n",
            lightmapped.channel(80, 60, 0), lightmapped.channel(80, 60, 1),
            lightmapped.channel(80, 60, 2), reference.channel(80, 60, 0),
            reference.channel(80, 60, 1), reference.channel(80, 60, 2),
            maxDifference(reference, lightmapped));
    CHECK(maxDifference(reference, lightmapped) <= 1);
    CHECK(worstError(
                  lightmapped, rig, kAlbedo, [](float, float) { return kFlat; }, 0, kWidth) <= 1);
    dump(lightmapped, "flat_lightmap", scene.tag);

    rig.env = ambientNone();
    rig.apply();
    Image noAmbient;
    CHECK(rig.capture(noAmbient));
    CHECK(maxDifference(reference, noAmbient) <= 1);

    server.instanceGeometrySetLightmap(scene.plane, LightmapId(), Math::Vec4(0, 0, 0, 0), 0);
    rig.env = ambientColour(Colour{ 1.0f, 1.0f, 1.0f });
    rig.apply();
    Image cleared;
    CHECK(rig.capture(cleared));
    CHECK(cleared.same(plain));
    CHECK(rig.stats.lightmapSurfaces == 0);

    server.instanceGeometrySetLightmap(scene.plane, f.lightmap, kFullRect, 0);
    server.lightmapSetVisible(f.lightmap, false);
    Image hidden;
    CHECK(rig.capture(hidden));
    CHECK(hidden.same(plain));
    server.lightmapSetVisible(f.lightmap, true);
    server.lightmapSetTextures(f.lightmap, TextureId(), false);
    Image withoutTexture;
    CHECK(rig.capture(withoutTexture));
    CHECK(withoutTexture.same(plain));

    server.instanceGeometrySetLightmap(scene.plane, LightmapId(), Math::Vec4(0, 0, 0, 0), 0);
    server.lightmapFree(f.lightmap);
}

void testGradient(Scene& scene)
{
    printf("the lightmap is sampled with the second UV set, row 0 at the bottom\n");
    Rig& rig = scene.rig;
    RenderingServer& server = *rig.server;
    showWhole(scene);
    rig.env = ambientNone();
    rig.apply();

    Fixture f;
    f.pixels = makePixels(16, 16, 1,
            [](int, int x, int y, int w, int h)
            {
                return Colour{ (static_cast<float>(x) + 0.5f) / static_cast<float>(w),
                    (static_cast<float>(y) + 0.5f) / static_cast<float>(h), 0.5f };
            });
    CHECK(f.data.setLightImages(16, 16, 1, f.pixels.data()));
    f.lightmap = makeLightmap(scene, f.data);
    server.instanceGeometrySetLightmap(scene.plane, f.lightmap, kFullRect, 0);

    Image image;
    CHECK(rig.capture(image));
    int channel = 0;
    const LayerImage& layer = f.pixels.decoded;
    const int worst = worstError(
            image, rig, kAlbedo, [&layer](float u, float v) { return layer.sample(0, u, v); }, 0,
            kWidth, &channel);
    printf("  gradient worst error %d levels (channel %d)\n", worst, channel);
    CHECK(worst <= kTolerance);
    CHECK(image.channel(10, 60, 0) < image.channel(150, 60, 0));
    CHECK(image.channel(80, 10, 1) < image.channel(80, 110, 1));
    dump(image, "gradient", scene.tag);

    server.setLightmapBicubicFilter(true);
    Image bicubic;
    CHECK(rig.capture(bicubic));
    int interior = 0;
    for (int y = 20; y < kHeight - 20; ++y)
        for (int x = 24; x < kWidth - 24; ++x)
            for (int c = 0; c < 3; ++c)
            {
                const int d = abs(image.channel(x, y, c) - bicubic.channel(x, y, c));
                if (d > interior) interior = d;
            }
    printf("  bicubic against bilinear: %d levels in the interior, %d at the borders\n", interior,
            maxDifference(image, bicubic));
    CHECK(interior <= 2);
    CHECK(maxDifference(image, bicubic) > 2);
    dump(bicubic, "gradient_bicubic", scene.tag);
    server.setLightmapBicubicFilter(false);

    server.instanceGeometrySetLightmap(scene.plane, LightmapId(), Math::Vec4(0, 0, 0, 0), 0);
    server.lightmapFree(f.lightmap);
}

void testSlicesAndScale(Scene& scene)
{
    printf("the slice index and the uv scale select the region\n");
    Rig& rig = scene.rig;
    RenderingServer& server = *rig.server;
    rig.env = ambientNone();
    rig.apply();

    const Colour palette[3] = { Colour{ 0.9f, 0.1f, 0.1f }, Colour{ 0.1f, 0.8f, 0.2f },
        Colour{ 0.2f, 0.3f, 0.9f } };
    Fixture f;
    f.pixels = makePixels(8, 8, 3,
            [&palette](int layer, int, int, int, int) { return palette[layer]; });
    CHECK(f.data.setLightImages(8, 8, 3, f.pixels.data()));
    f.lightmap = makeLightmap(scene, f.data);

    showHalves(scene);
    server.instanceGeometrySetLightmap(scene.leftPlane, f.lightmap, kFullRect, 2);
    server.instanceGeometrySetLightmap(scene.rightPlane, f.lightmap, kFullRect, 1);
    Image slices;
    CHECK(rig.capture(slices));
    CHECK(rig.stats.lightmapSurfaces == 2 && rig.stats.lightmapsUsed == 1);
    int worst = worstError(
            slices, rig, kAlbedo, [&palette](float, float) { return palette[2]; }, 2,
            kWidth / 2 - 2);
    worst = worst > worstError(
                            slices, rig, kAlbedo, [&palette](float, float) { return palette[1]; },
                            kWidth / 2 + 2, kWidth - 2)
                    ? worst
                    : worstError(
                              slices, rig, kAlbedo, [&palette](float, float) { return palette[1]; },
                              kWidth / 2 + 2, kWidth - 2);
    printf("  slices 2 and 1: worst error %d\n", worst);
    CHECK(worst <= kTolerance);
    dump(slices, "slices", scene.tag);

    server.instanceGeometrySetLightmap(scene.leftPlane, f.lightmap, kFullRect, 0);
    Image first;
    CHECK(rig.capture(first));
    CHECK(worstError(
                  first, rig, kAlbedo, [&palette](float, float) { return palette[0]; }, 2,
                  kWidth / 2 - 2) <= kTolerance);

    server.lightmapFree(f.lightmap);
    f.data.clear();

    Fixture halves;
    halves.pixels = makePixels(8, 8, 1, [](int, int x, int, int, int)
            { return x < 4 ? Colour{ 0.9f, 0.2f, 0.1f } : Colour{ 0.1f, 0.3f, 0.8f }; });
    CHECK(halves.data.setLightImages(8, 8, 1, halves.pixels.data()));
    halves.lightmap = makeLightmap(scene, halves.data);
    showWhole(scene);
    const Math::Vec4 rects[3] = { Math::Vec4(0.0f, 0.0f, 0.5f, 1.0f),
        Math::Vec4(0.5f, 0.0f, 0.5f, 1.0f), Math::Vec4(0.25f, 0.5f, 0.5f, 0.5f) };
    for (int r = 0; r < 3; ++r)
    {
        server.instanceGeometrySetLightmap(scene.plane, halves.lightmap, rects[r], 0);
        Image image;
        CHECK(rig.capture(image));
        const LayerImage& layer = halves.pixels.decoded;
        const Math::Vec4 rect = rects[r];
        const int error = worstError(
                image, rig, kAlbedo, [&layer, rect](float u, float v)
                { return layer.sample(0, u * rect.z + rect.x, v * rect.w + rect.y); }, 0, kWidth);
        printf("  uv scale %.2f %.2f %.2f %.2f: worst error %d\n", rect.x, rect.y, rect.z, rect.w,
                error);
        CHECK(error <= kTolerance);
        char name[64];
        snprintf(name, sizeof(name), "uv_scale_%d", r);
        dump(image, name, scene.tag);
    }
    server.instanceGeometrySetLightmap(scene.plane, LightmapId(), Math::Vec4(0, 0, 0, 0), 0);
    server.lightmapFree(halves.lightmap);
}

void testMultipleLightmaps(Scene& scene)
{
    printf("two lightmaps in one frame, each with its own texture\n");
    Rig& rig = scene.rig;
    RenderingServer& server = *rig.server;
    rig.env = ambientNone();
    rig.apply();
    const Colour a{ 0.7f, 0.2f, 0.1f };
    const Colour b{ 0.1f, 0.4f, 0.9f };
    Fixture one;
    Fixture two;
    one.pixels = makePixels(4, 4, 1, [&a](int, int, int, int, int) { return a; });
    two.pixels = makePixels(8, 8, 2, [&b](int layer, int, int, int, int)
            { return layer == 1 ? b : Colour{ 0.0f, 0.0f, 0.0f }; });
    CHECK(one.data.setLightImages(4, 4, 1, one.pixels.data()));
    CHECK(two.data.setLightImages(8, 8, 2, two.pixels.data()));
    one.lightmap = makeLightmap(scene, one.data);
    two.lightmap = makeLightmap(scene, two.data);
    showHalves(scene);
    server.instanceGeometrySetLightmap(scene.leftPlane, one.lightmap, kFullRect, 0);
    server.instanceGeometrySetLightmap(scene.rightPlane, two.lightmap, kFullRect, 1);
    Image image;
    CHECK(rig.capture(image));
    CHECK(rig.stats.lightmapsUsed == 2 && rig.stats.lightmapSurfaces == 2);
    const int left =
            worstError(image, rig, kAlbedo, [&a](float, float) { return a; }, 2, kWidth / 2 - 2);
    const int right = worstError(
            image, rig, kAlbedo, [&b](float, float) { return b; }, kWidth / 2 + 2, kWidth - 2);
    printf("  left error %d, right error %d\n", left, right);
    CHECK(left <= kTolerance && right <= kTolerance);
    dump(image, "two_lightmaps", scene.tag);

    server.lightmapSetBounds(two.lightmap,
            Math::Box(Math::Vec3(100.0f, 100.0f, 100.0f), Math::Vec3(101.0f, 101.0f, 101.0f)));
    Image culled;
    CHECK(rig.capture(culled));
    CHECK(rig.stats.lightmapsUsed == 1 && rig.stats.lightmapSurfaces == 1);
    CHECK(worstError(
                  culled, rig, kAlbedo, [&a](float, float) { return a; }, 2, kWidth / 2 - 2) <=
            kTolerance);
    CHECK(culled.channel(120, 60, 0) == 0 && culled.channel(120, 60, 1) == 0);
    server.lightmapSetBounds(two.lightmap,
            Math::Box(Math::Vec3(-10.0f, -10.0f, -10.0f), Math::Vec3(10.0f, 10.0f, 10.0f)));
    Image inside;
    CHECK(rig.capture(inside));
    CHECK(inside.same(image));

    server.instanceGeometrySetLightmap(scene.leftPlane, LightmapId(), Math::Vec4(0, 0, 0, 0), 0);
    server.instanceGeometrySetLightmap(scene.rightPlane, LightmapId(), Math::Vec4(0, 0, 0, 0), 0);
    server.lightmapFree(one.lightmap);
    server.lightmapFree(two.lightmap);
}

void testSphericalHarmonics(Scene& scene)
{
    printf("the directional lightmap path (four layers per slice)\n");
    Rig& rig = scene.rig;
    RenderingServer& server = *rig.server;
    rig.env = ambientNone();
    rig.apply();
    showWhole(scene);

    const float l1n1 = 0.3f;
    const float l1_0 = 0.5f;
    const float l1p1 = -0.2f;
    Fixture f;
    f.pixels = makePixels(8, 8, 4,
            [&](int layer, int, int, int, int)
            {
                switch (layer)
                {
                    case 0:
                        return kFlat;
                    case 1:
                        return Colour{ 0.5f + l1n1 * 0.5f, 0.5f + l1n1 * 0.5f, 0.5f + l1n1 * 0.5f };
                    case 2:
                        return Colour{ 0.5f + l1_0 * 0.5f, 0.5f + l1_0 * 0.5f, 0.5f + l1_0 * 0.5f };
                    default:
                        return Colour{ 0.5f + l1p1 * 0.5f, 0.5f + l1p1 * 0.5f, 0.5f + l1p1 * 0.5f };
                }
            });
    CHECK(f.data.setLightImages(8, 8, 4, f.pixels.data()));
    f.lightmap = makeLightmap(scene, f.data, true);
    server.instanceGeometrySetLightmap(scene.plane, f.lightmap, kFullRect, 0);

    auto expected = [&](const Math::Vec3& normal)
    {
        const LayerImage& layer = f.pixels.decoded;
        const Colour l0 = layer.texel(0, 0, 0);
        const float factor =
                1.0f + l1n1 * normal.y * 4.0f + l1_0 * normal.z * 4.0f + l1p1 * normal.x * 4.0f;
        return Colour{ l0.r * factor, l0.g * factor, l0.b * factor };
    };

    Image facing;
    CHECK(rig.capture(facing));
    CHECK(rig.stats.lightmapSurfaces == 1);
    const Colour front = expected(Math::Vec3(0.0f, 0.0f, 1.0f));
    int error =
            worstError(facing, rig, kAlbedo, [front](float, float) { return front; }, 0, kWidth);
    printf("  facing the camera: worst error %d\n", error);
    CHECK(error <= kTolerance);
    dump(facing, "directional", scene.tag);

    const Math::Mat4 turned = Math::Mat4::RotationY(0.35f);
    server.cameraSetTransform(rig.camera, turned);
    Image rotated;
    CHECK(rig.capture(rotated));
    CHECK(rotated.channel(80, 60, 0) > 0);
    CHECK(abs(rotated.channel(80, 60, 0) - encode(kAlbedo.r * front.r)) <= kTolerance);
    CHECK(abs(rotated.channel(80, 60, 1) - encode(kAlbedo.g * front.g)) <= kTolerance);
    server.cameraSetTransform(rig.camera, Math::Mat4::Identity());

    const Math::Mat4 lightmapTurn = Math::Mat4::RotationY(1.5707963f);
    server.lightmapSetTransform(f.lightmap, lightmapTurn);
    Image turnedLightmap;
    CHECK(rig.capture(turnedLightmap));
    const Math::Vec3 inLightmap = Math::Mat3(lightmapTurn).Inverse() * Math::Vec3(0, 0, 1);
    const Colour side = expected(inLightmap);
    printf("  lightmap turned a quarter: normal %.2f %.2f %.2f\n", inLightmap.x, inLightmap.y,
            inLightmap.z);
    error = worstError(
            turnedLightmap, rig, kAlbedo, [side](float, float) { return side; }, 0, kWidth);
    CHECK(error <= kTolerance);
    CHECK(maxDifference(facing, turnedLightmap) > 20);
    server.lightmapSetTransform(f.lightmap, Math::Mat4::Identity());

    MaterialStore& materials = *Main::materials();
    const MaterialId glossy = materials.create("lightmap glossy");
    materials.setFloat(glossy, MaterialParam::Roughness, 0.25f);
    materials.setFloat(glossy, MaterialParam::Specular, 1.0f);
    server.instanceSetMaterialOverride(scene.plane, glossy);
    Image diffuseOnly;
    CHECK(rig.capture(diffuseOnly));
    server.lightmapSetSpecularIntensity(f.lightmap, 0.0f);
    Image zero;
    CHECK(rig.capture(zero));
    CHECK(zero.same(diffuseOnly));
    server.lightmapSetSpecularIntensity(f.lightmap, 4.0f);
    Image specular;
    CHECK(rig.capture(specular));
    CHECK(rig.stats.lightmapSurfaces == 1);
    printf("  specular intensity 4: %d levels from the diffuse only image\n",
            maxDifference(diffuseOnly, specular));
    CHECK(maxDifference(diffuseOnly, specular) > 4);
    dump(specular, "directional_specular", scene.tag);
    server.lightmapSetSpecularIntensity(f.lightmap, 0.0f);
    server.instanceSetMaterialOverride(scene.plane, MaterialId());
    materials.destroy(glossy);

    server.instanceGeometrySetLightmap(scene.plane, LightmapId(), Math::Vec4(0, 0, 0, 0), 0);
    server.lightmapFree(f.lightmap);
}

LightId directional(Scene& scene, LightBakeMode mode, float energy)
{
    RenderingServer& server = *scene.rig.server;
    const LightId light = server.lightCreate(LightType::Directional);
    server.lightSetScenario(light, scene.rig.scenario);
    server.lightSetTransform(light, Math::Mat4::Identity());
    server.lightSetParam(light, LightParam::Energy, energy);
    server.lightSetBakeMode(light, mode);
    return light;
}

LightId omni(Scene& scene, LightBakeMode mode, float energy)
{
    RenderingServer& server = *scene.rig.server;
    const LightId light = server.lightCreate(LightType::Omni);
    server.lightSetScenario(light, scene.rig.scenario);
    server.lightSetTransform(light, Math::Mat4::Translation(Math::Vec3(0.0f, 0.0f, -1.0f)));
    server.lightSetParam(light, LightParam::Energy, energy);
    server.lightSetParam(light, LightParam::Range, 20.0f);
    server.lightSetBakeMode(light, mode);
    return light;
}

void testBakeModes(Scene& scene)
{
    printf("a static light that was baked is skipped on lightmapped objects\n");
    Rig& rig = scene.rig;
    RenderingServer& server = *rig.server;
    showWhole(scene);

    Fixture f;
    f.pixels = makePixels(8, 8, 1, [](int, int, int, int, int) { return kFlat; });
    CHECK(f.data.setLightImages(8, 8, 1, f.pixels.data()));
    f.lightmap = makeLightmap(scene, f.data);

    for (int kind = 0; kind < 2; ++kind)
    {
        const bool point = kind == 1;
        const char* name = point ? "omni" : "directional";
        const LightId light = point ? omni(scene, LightBakeMode::Dynamic, 0.4f)
                                    : directional(scene, LightBakeMode::Dynamic, 0.4f);

        rig.env = ambientColour(kFlat);
        rig.apply();
        server.instanceGeometrySetLightmap(scene.plane, LightmapId(), Math::Vec4(0, 0, 0, 0), 0);
        Image referenceLit;
        CHECK(rig.capture(referenceLit));
        server.lightSetVisible(light, false);
        Image referenceDark;
        CHECK(rig.capture(referenceDark));
        server.lightSetVisible(light, true);
        CHECK(maxDifference(referenceLit, referenceDark) > 10);

        rig.env = ambientNone();
        rig.apply();
        server.instanceGeometrySetLightmap(scene.plane, f.lightmap, kFullRect, 0);

        const LightBakeMode modes[3] = { LightBakeMode::Dynamic, LightBakeMode::Disabled,
            LightBakeMode::Static };
        const char* modeNames[3] = { "dynamic", "disabled", "static" };
        for (int m = 0; m < 3; ++m)
        {
            server.lightSetBakeMode(light, modes[m]);
            Image image;
            CHECK(rig.capture(image));
            const Image& expected = m == 2 ? referenceDark : referenceLit;
            const int difference = maxDifference(expected, image);
            printf("  %s light, bake mode %s: %d levels from the %s reference\n", name,
                    modeNames[m], difference, m == 2 ? "lightmap only" : "lightmap plus light");
            CHECK(difference <= 1);
            char label[64];
            snprintf(label, sizeof(label), "bake_%s_%s", name, modeNames[m]);
            dump(image, label, scene.tag);
        }

        server.instanceGeometrySetLightmap(scene.plane, LightmapId(), Math::Vec4(0, 0, 0, 0), 0);
        rig.env = ambientColour(kFlat);
        rig.apply();
        server.lightSetBakeMode(light, LightBakeMode::Static);
        Image unbaked;
        CHECK(rig.capture(unbaked));
        CHECK(maxDifference(referenceLit, unbaked) <= 1);
        server.lightFree(light);
    }
    server.lightmapFree(f.lightmap);
}

void testShadowmask(Scene& scene)
{
    printf("the shadowmask overlay darkens the real time light\n");
    Rig& rig = scene.rig;
    RenderingServer& server = *rig.server;
    showWhole(scene);
    rig.env = ambientNone();
    rig.apply();

    Fixture f;
    f.pixels = makePixels(8, 8, 1, [](int, int, int, int, int) { return kFlat; });
    CHECK(f.data.setLightImages(8, 8, 1, f.pixels.data()));
    f.lightmap = makeLightmap(scene, f.data);
    const LightId light = directional(scene, LightBakeMode::Dynamic, 0.4f);
    server.lightSetShadow(light, true);
    server.instanceGeometrySetLightmap(scene.plane, f.lightmap, kFullRect, 0);

    Image none;
    CHECK(rig.capture(none));

    Pixels lit = makePixels(8, 8, 1, [](int, int, int, int, int) { return Colour{ 1, 1, 1 }; });
    Pixels shadowed =
            makePixels(8, 8, 1, [](int, int, int, int, int) { return Colour{ 0, 0, 0 }; });
    server.lightmapSetShadowmaskMode(f.lightmap, ShadowmaskMode::Overlay);

    CHECK(f.data.setShadowmaskImages(8, 8, 1, lit.data()));
    server.lightmapSetShadowmaskTextures(f.lightmap, f.data.getShadowmaskTexture());
    Image overlayLit;
    CHECK(rig.capture(overlayLit));
    printf("  overlay with a lit mask: %d levels from no mask\n", maxDifference(none, overlayLit));
    CHECK(maxDifference(none, overlayLit) <= 1);

    CHECK(f.data.setShadowmaskImages(8, 8, 1, shadowed.data()));
    server.lightmapSetShadowmaskTextures(f.lightmap, f.data.getShadowmaskTexture());
    Image overlayShadowed;
    CHECK(rig.capture(overlayShadowed));

    server.lightSetVisible(light, false);
    Image lightmapOnly;
    CHECK(rig.capture(lightmapOnly));
    server.lightSetVisible(light, true);
    printf("  overlay with a shadowed mask: %d levels from the lightmap only image, %d from the "
           "lit one\n",
            maxDifference(lightmapOnly, overlayShadowed), maxDifference(none, overlayShadowed));
    CHECK(maxDifference(lightmapOnly, overlayShadowed) <= 1);
    CHECK(maxDifference(none, overlayShadowed) > 10);
    dump(overlayShadowed, "shadowmask_overlay", scene.tag);

    server.lightmapSetShadowmaskMode(f.lightmap, ShadowmaskMode::Only);
    Image only;
    CHECK(rig.capture(only));
    CHECK(maxDifference(lightmapOnly, only) <= 1);
    CHECK(f.data.setShadowmaskImages(8, 8, 1, lit.data()));
    server.lightmapSetShadowmaskTextures(f.lightmap, f.data.getShadowmaskTexture());
    Image onlyLit;
    CHECK(rig.capture(onlyLit));
    CHECK(maxDifference(none, onlyLit) <= 1);

    server.instanceGeometrySetLightmap(scene.plane, LightmapId(), Math::Vec4(0, 0, 0, 0), 0);
    server.lightFree(light);
    server.lightmapFree(f.lightmap);
}

void testScissorMaterial(Scene& scene)
{
    printf("a lightmapped surface with an alpha scissor material\n");
    Rig& rig = scene.rig;
    RenderingServer& server = *rig.server;
    MaterialStore& materials = *Main::materials();
    showWhole(scene);
    rig.env = ambientNone();
    rig.apply();
    const MaterialId material = materials.create("lightmap scissor");
    materials.setTransparency(material, MaterialTransparency::AlphaScissor);
    server.instanceSetMaterialOverride(scene.plane, material);
    Fixture f;
    f.pixels = makePixels(8, 8, 1, [](int, int, int, int, int) { return kFlat; });
    CHECK(f.data.setLightImages(8, 8, 1, f.pixels.data()));
    f.lightmap = makeLightmap(scene, f.data);
    server.instanceGeometrySetLightmap(scene.plane, f.lightmap, kFullRect, 0);
    server.viewportSetMsaa3D(rig.view, ViewportMsaa::Msaa4x);
    Image image;
    CHECK(rig.capture(image));
    CHECK(rig.stats.lightmapSurfaces == 1);
    const int error =
            worstError(image, rig, kAlbedo, [](float, float) { return kFlat; }, 0, kWidth);
    printf("  scissor with MSAA 4x: worst error %d\n", error);
    CHECK(error <= kTolerance);
    server.viewportSetMsaa3D(rig.view, ViewportMsaa::Disabled);
    server.instanceSetMaterialOverride(scene.plane, MaterialId());
    server.instanceGeometrySetLightmap(scene.plane, LightmapId(), Math::Vec4(0, 0, 0, 0), 0);
    server.lightmapFree(f.lightmap);
    materials.destroy(material);
}


SurfaceData quadSurface(const Math::Vec3& a, const Math::Vec3& b, const Math::Vec3& c,
        const Math::Vec3& d, const Math::Vec3& normal, const Colour& albedo)
{
    SurfaceData data;
    const Math::Vec3 corners[4] = { a, b, c, d };
    const Math::Vec2 uvs[4] = { Math::Vec2(0.0f, 0.0f), Math::Vec2(1.0f, 0.0f),
        Math::Vec2(1.0f, 1.0f), Math::Vec2(0.0f, 1.0f) };
    for (int i = 0; i < 4; ++i)
    {
        data.positions.push_back(corners[i]);
        data.normals.push_back(normal);
        data.colors.push_back(Math::Vec4(albedo.r, albedo.g, albedo.b, 1.0f));
        data.uvs.push_back(uvs[i]);
        data.uvs2.push_back(uvs[i]);
    }
    const std::uint32_t indices[6] = { 0, 1, 2, 0, 2, 3 };
    for (std::uint32_t index: indices) data.indices.push_back(index);
    return data;
}

void showcase(Scene& scene)
{
    if (!getenv("PRISMA3D_SHOTS")) return;
    Rig& rig = scene.rig;
    RenderingServer& server = *rig.server;
    MeshStore& meshes = *Main::meshes();
    hideAll(scene);

    const MeshId floorMesh = meshes.create("showcase floor");
    CHECK(meshes.addSurface(floorMesh,
            quadSurface(Math::Vec3(-4, 0, 2), Math::Vec3(4, 0, 2), Math::Vec3(4, 0, -6),
                    Math::Vec3(-4, 0, -6), Math::Vec3(0, 1, 0), Colour{ 0.75f, 0.72f, 0.65f })));
    const MeshId wallMesh = meshes.create("showcase wall");
    CHECK(meshes.addSurface(wallMesh,
            quadSurface(Math::Vec3(-4, 0, -6), Math::Vec3(4, 0, -6), Math::Vec3(4, 4, -6),
                    Math::Vec3(-4, 4, -6), Math::Vec3(0, 0, 1), Colour{ 0.7f, 0.74f, 0.8f })));
    const InstanceId floor = server.instanceCreate(floorMesh, rig.scenario);
    const InstanceId wall = server.instanceCreate(wallMesh, rig.scenario);

    const Pixels baked = makePixels(64, 64, 2,
            [](int layer, int x, int y, int w, int h)
            {
                const float u = (static_cast<float>(x) + 0.5f) / static_cast<float>(w);
                const float v = (static_cast<float>(y) + 0.5f) / static_cast<float>(h);
                if (layer == 0)
                {
                    const float dx = u - 0.35f;
                    const float dy = v - 0.5f;
                    float pool = 1.6f * expf(-(dx * dx + dy * dy) * 6.0f) + 0.12f;
                    const float box = fmaxf(fabsf(u - 0.55f) - 0.12f, fabsf(v - 0.5f) - 0.12f);
                    const float shadow = clampf(box * 14.0f + 0.15f, 0.0f, 1.0f);
                    pool *= 0.25f + 0.75f * shadow;
                    return Colour{ pool * 1.0f, pool * 0.9f, pool * 0.7f };
                }
                const float glow = 0.1f + 0.9f * (1.0f - v) * expf(-(u - 0.35f) * (u - 0.35f) * 5.0f);
                return Colour{ glow * 0.8f, glow * 0.85f, glow };
            });
    LightmapGIData data;
    CHECK(data.setLightImages(64, 64, 2, baked.data()));
    const LightmapId lightmap = makeLightmap(scene, data);

    server.cameraSetTransform(rig.camera,
            Math::Mat4::Translation(Math::Vec3(0.0f, 2.6f, 5.0f)) * Math::Mat4::RotationX(-0.38f));
    rig.env = ambientColour(Colour{ 0.5f, 0.5f, 0.5f });
    rig.apply();
    Image plain;
    CHECK(rig.capture(plain));
    dump(plain, "showcase_plain", scene.tag);
    server.instanceGeometrySetLightmap(floor, lightmap, kFullRect, 0);
    server.instanceGeometrySetLightmap(wall, lightmap, kFullRect, 1);
    Image lit;
    CHECK(rig.capture(lit));
    CHECK(rig.stats.lightmapSurfaces == 2);
    CHECK(maxDifference(plain, lit) > 30);
    dump(lit, "showcase_lightmapped", scene.tag);

    server.instanceGeometrySetLightmap(floor, LightmapId(), Math::Vec4(0, 0, 0, 0), 0);
    server.instanceGeometrySetLightmap(wall, LightmapId(), Math::Vec4(0, 0, 0, 0), 0);
    server.instanceFree(floor);
    server.instanceFree(wall);
    server.lightmapFree(lightmap);
    server.cameraSetTransform(rig.camera, Math::Mat4::Identity());
}

bool captureWindow(SceneTree& tree, Image& image)
{
    for (int i = 0; i < 3; ++i)
    {
        tree.process(1.0 / 60.0);
        Main::rendering()->renderViewports();
        Main::rendering()->endFrame(true);
    }
    tree.process(1.0 / 60.0);
    Main::rendering()->renderViewports();
    image.width = kWidth;
    image.height = kHeight;
    image.pixels.resize(static_cast<std::size_t>(kWidth) * static_cast<std::size_t>(kHeight) * 4);
    prisma::Rect rect;
    rect.width = static_cast<std::uint32_t>(kWidth);
    rect.height = static_cast<std::uint32_t>(kHeight);
    const bool ok =
            Main::device()->driver()->readPixels(prisma::RenderTarget(), rect, image.pixels.data());
    Main::rendering()->endFrame(true);
    return ok;
}

ct::Unique<LightmapGIData> bakedData(const Pixels& pixels, const char* user)
{
    ct::Unique<LightmapGIData> data = ct::make_unique<LightmapGIData>();
    CHECK(data->setLightImages(static_cast<std::uint32_t>(pixels.width),
            static_cast<std::uint32_t>(pixels.height), static_cast<std::uint32_t>(pixels.layers),
            pixels.data()));
    data->addUser(user, kFullRect, 0);
    data->setBounds(Math::Box(Math::Vec3(-10.0f, -10.0f, -10.0f), Math::Vec3(10.0f, 10.0f, 10.0f)));
    data->setBakedExposure(1.0f);
    return data;
}

int centreError(const Image& image, const Colour& light)
{
    int worst = 0;
    const int expected[3] = { encode(kAlbedo.r * light.r), encode(kAlbedo.g * light.g),
        encode(kAlbedo.b * light.b) };
    for (int c = 0; c < 3; ++c)
    {
        const int d = abs(image.channel(80, 60, c) - expected[c]);
        if (d > worst) worst = d;
    }
    return worst;
}

void testNodes(Scene& scene, SceneTree& tree)
{
    printf("the LightmapGI node assigns and clears the lightmaps of its users\n");
    Rig& rig = scene.rig;
    RenderingServer& server = *rig.server;
    hideAll(scene);
    server.viewportSetUpdateMode(rig.view, ViewportUpdateMode::Disabled);
    Node* root = tree.getRoot();
    CHECK(root != nullptr);
    if (!root) return;

    WorldEnvironment* world = root->createChild<WorldEnvironment>();
    EnvironmentSettings env = ambientColour(Colour{ 1.0f, 1.0f, 1.0f });
    world->setEnvironment(env);
    Camera3D* camera = root->createChild<Camera3D>();
    camera->setPerspective(kFov, 0.1f, 100.0f);
    camera->makeCurrent();
    MeshInstance3D* plane = root->createChild<MeshInstance3D>();
    plane->setName("Plane");
    plane->setMesh(scene.whole);

    const Pixels pixels = makePixels(8, 8, 1, [](int, int, int, int, int) { return kFlat; });
    Image plain;
    CHECK(captureWindow(tree, plain));
    CHECK(Main::rendering()->getStats().lightmapSurfaces == 0);
    CHECK(centreError(plain, Colour{ 1.0f, 1.0f, 1.0f }) <= 1);

    LightmapGI* lightmap = root->createChild<LightmapGI>();
    lightmap->setName("Lightmap");
    CHECK(lightmap->getLightData() == nullptr);
    Image empty;
    CHECK(captureWindow(tree, empty));
    CHECK(empty.same(plain));

    lightmap->setLightData(bakedData(pixels, "../Plane"));
    CHECK(lightmap->getLightData() != nullptr);
    Image lit;
    CHECK(captureWindow(tree, lit));
    CHECK(Main::rendering()->getStats().lightmapSurfaces == 1);
    printf("  centre pixel %d %d %d, plain %d %d %d\n", lit.channel(80, 60, 0),
            lit.channel(80, 60, 1), lit.channel(80, 60, 2), plain.channel(80, 60, 0),
            plain.channel(80, 60, 1), plain.channel(80, 60, 2));
    CHECK(centreError(lit, kFlat) <= 1);
    dump(lit, "node_lightmapped", scene.tag);
    dump(plain, "node_plain", scene.tag);

    lightmap->setVisible(false);
    Image hidden;
    CHECK(captureWindow(tree, hidden));
    CHECK(hidden.same(plain));
    lightmap->setVisible(true);
    Image shown;
    CHECK(captureWindow(tree, shown));
    CHECK(shown.same(lit));

    lightmap->setLightData(ct::Unique<LightmapGIData>());
    Image removedData;
    CHECK(captureWindow(tree, removedData));
    CHECK(removedData.same(plain));
    lightmap->setLightData(bakedData(pixels, "../Plane"));
    Image again;
    CHECK(captureWindow(tree, again));
    CHECK(again.same(lit));

    lightmap->queueFree();
    Image freed;
    CHECK(captureWindow(tree, freed));
    CHECK(freed.same(plain));
    CHECK(Main::rendering()->getStats().lightmapSurfaces == 0);

    LightmapGI* second = root->createChild<LightmapGI>();
    second->setLightData(bakedData(pixels, "../Plane"));
    Image back;
    CHECK(captureWindow(tree, back));
    CHECK(back.same(lit));

    second->getParent()->removeChild(second);
    Image detached;
    CHECK(captureWindow(tree, detached));
    CHECK(detached.same(plain));
    root->addChild(second);
    Image reattached;
    CHECK(captureWindow(tree, reattached));
    CHECK(reattached.same(lit));

    printf("a missing user is reported and the others are still assigned\n");
    second->setLightData(bakedData(pixels, "../Missing"));
    Image missing;
    CHECK(captureWindow(tree, missing));
    CHECK(missing.same(plain));
    second->queueFree();

    LightmapGI* partial = root->createChild<LightmapGI>();
    ct::Unique<LightmapGIData> both = bakedData(pixels, "../Missing");
    both->addUser("../Plane", kFullRect, 0);
    partial->setLightData(static_cast<ct::Unique<LightmapGIData>&&>(both));
    Image some;
    CHECK(captureWindow(tree, some));
    CHECK(some.same(lit));

    printf("the light bake mode, gi mode and texel scale are properties\n");
    DirectionalLight3D* sun = root->createChild<DirectionalLight3D>();
    CHECK(sun->getBakeMode() == LightBakeMode::Dynamic);
    CHECK(sun->set("light_bake_mode", PropertyValue::fromInt(1)));
    CHECK(sun->getBakeMode() == LightBakeMode::Static);
    sun->setEnergy(0.4f);
    sun->setGlobalTransform(Math::Mat4::Identity());
    Image skipped;
    CHECK(captureWindow(tree, skipped));
    CHECK(skipped.same(lit));
    CHECK(sun->set("light_bake_mode", PropertyValue::fromInt(2)));
    Image dynamic;
    CHECK(captureWindow(tree, dynamic));
    CHECK(maxDifference(dynamic, lit) > 10);
    sun->queueFree();

    CHECK(plane->getGiMode() == GiMode::Static);
    CHECK(plane->set("gi_mode", PropertyValue::fromInt(2)));
    CHECK(plane->getGiMode() == GiMode::Dynamic);
    CHECK(plane->set("gi_lightmap_texel_scale", PropertyValue::fromSingle(2.5f)));
    CHECK(plane->getLightmapTexelScale() == 2.5f);
    Image gi;
    CHECK(captureWindow(tree, gi));
    CHECK(gi.same(lit));

    partial->queueFree();
    plane->queueFree();
    camera->queueFree();
    world->queueFree();
    tree.process(1.0 / 60.0);
    server.viewportSetUpdateMode(rig.view, ViewportUpdateMode::Always);
}

void testHalfFloats()
{
    printf("half float conversion\n");
    const float exact[] = { 0.0f, 1.0f, 0.5f, 0.25f, 2.0f, 1024.0f, 65504.0f, -1.5f,
        0.000061035156f };
    for (float value: exact)
        CHECK(LightmapGIData::halfToFloat(LightmapGIData::floatToHalf(value)) == value);
    CHECK(LightmapGIData::floatToHalf(1.0f) == 0x3c00);
    CHECK(LightmapGIData::floatToHalf(-2.0f) == 0xc000);
    CHECK(LightmapGIData::floatToHalf(1.0e10f) == 0x7c00);
    CHECK(LightmapGIData::floatToHalf(1.0e-10f) == 0);
    CHECK(LightmapGIData::halfToFloat(0x0001) > 5.9e-8f &&
            LightmapGIData::halfToFloat(0x0001) < 6.0e-8f);
    const float rounded = LightmapGIData::halfToFloat(LightmapGIData::floatToHalf(0.1f));
    CHECK(fabsf(rounded - 0.1f) < 5.0e-5f);
    CHECK(LightmapGIData::halfToFloat(LightmapGIData::floatToHalf(NAN)) !=
            LightmapGIData::halfToFloat(LightmapGIData::floatToHalf(NAN)));
}

ct::String testDirectory()
{
    ct::String directory = PRISMA3D_TEST_DIR;
    directory += "/lightmap_files";
    return directory;
}

void testDataFile(Scene& scene)
{
    printf("the lightmap data file keeps the textures, the users and the settings\n");
    Rig& rig = scene.rig;
    RenderingServer& server = *rig.server;
    const ct::String directory = testDirectory();
    removeTree(directory);
    FileSystem::getSingleton().addSearchPath(directory);

    const Pixels pixels = makePixels(16, 8, 3,
            [](int layer, int x, int y, int w, int h)
            {
                return Colour{ static_cast<float>(x) / static_cast<float>(w) * 0.9f,
                    static_cast<float>(y) / static_cast<float>(h) * 0.9f,
                    static_cast<float>(layer) * 0.4f };
            });
    const Pixels mask = makePixels(16, 8, 3,
            [](int, int x, int, int, int) { return Colour{ x < 8 ? 0.0f : 1.0f, 0.0f, 0.0f }; });
    LightmapGIData data;
    CHECK(data.setLightImages(16, 8, 3, pixels.data()));
    CHECK(data.setShadowmaskImages(16, 8, 3, mask.data()));
    data.setUsesSphericalHarmonics(true);
    data.setInterior(true);
    data.setBakedExposure(1.5f);
    data.setBounds(Math::Box(Math::Vec3(-1.0f, -2.0f, -3.0f), Math::Vec3(4.0f, 5.0f, 6.0f)));
    data.addUser("../Floor", Math::Vec4(0.25f, 0.5f, 0.125f, 0.0625f), 2);
    data.addUser("../Wall/Mesh", Math::Vec4(0.0f, 0.0f, 1.0f, 1.0f), 0, 3);
    CHECK(data.getLayerCount() == 3 && data.hasShadowmaskTextures());

    ct::String error;
    CHECK(data.save(directory + "/missing_directory/level.lmdata", &error));
    CHECK(data.save(directory + "/level.lmdata", &error));
    CHECK(FileSystem::getSingleton().exists("level.lmdata"));
    CHECK(FileSystem::getSingleton().exists("level.light.dds"));
    CHECK(FileSystem::getSingleton().exists("level.shadowmask.dds"));
    const ct::String text = FileSystem::getSingleton().readText("level.lmdata");
    CHECK(text.find("\"format\": \"prisma3d-lightmap\"") != ct::String::npos);
    CHECK(text.find("level.light.dds") != ct::String::npos);

    LightmapGIData loaded;
    CHECK(loaded.load("level.lmdata", &error));
    CHECK(loaded.getLayerCount() == 3 && loaded.hasShadowmaskTextures());
    CHECK(loaded.isUsingSphericalHarmonics() && loaded.isInterior());
    CHECK(loaded.getBakedExposure() == 1.5f);
    CHECK(loaded.getBounds().min.x == -1.0f && loaded.getBounds().max.z == 6.0f);
    CHECK(loaded.getUserCount() == 2);
    if (loaded.getUserCount() == 2)
    {
        CHECK(loaded.getUser(0).path == "../Floor" && loaded.getUser(0).sliceIndex == 2 &&
                loaded.getUser(0).subInstance == -1);
        CHECK(loaded.getUser(0).uvScale.x == 0.25f && loaded.getUser(0).uvScale.w == 0.0625f);
        CHECK(loaded.getUser(1).path == "../Wall/Mesh" && loaded.getUser(1).subInstance == 3);
    }
    const TextureInfo* info = Main::textures()->get(loaded.getLightTexture());
    CHECK(info && info->type == prisma::TextureType::Texture2DArray && info->layers == 3 &&
            info->width == 16 && info->height == 8 &&
            info->format == prisma::TextureFormat::RGBA16F);

    const LightmapId original = makeLightmap(scene, data);
    const LightmapId reread = makeLightmap(scene, loaded);
    rig.env = ambientNone();
    rig.apply();
    showWhole(scene);
    for (int layer = 0; layer < 3; ++layer)
    {
        server.instanceGeometrySetLightmap(scene.plane, original, kFullRect, layer);
        Image before;
        CHECK(rig.capture(before));
        server.instanceGeometrySetLightmap(scene.plane, reread, kFullRect, layer);
        Image after;
        CHECK(rig.capture(after));
        CHECK(rig.stats.lightmapSurfaces == 1);
        CHECK(after.same(before));
        const int error = worstError(
                after, rig, kAlbedo, [&pixels, layer](float u, float v)
                { return pixels.decoded.sample(layer, u, v); }, 0, kWidth);
        printf("  layer %d after the file: worst error %d\n", layer, error);
        CHECK(error <= kTolerance);
    }
    server.instanceGeometrySetLightmap(scene.plane, LightmapId(), Math::Vec4(0, 0, 0, 0), 0);
    server.lightmapFree(original);
    server.lightmapFree(reread);

    printf("the data file refuses what it cannot use\n");
    LightmapGIData broken;
    CHECK(!broken.load("does_not_exist.lmdata", &error) && !error.empty());
    FileSystem::writeText(directory + "/bad.lmdata", "{ not json");
    CHECK(!broken.load("bad.lmdata", &error));
    FileSystem::writeText(directory + "/wrong.lmdata", "{\"format\":\"other\"}");
    CHECK(!broken.load("wrong.lmdata", &error));
    FileSystem::writeText(directory + "/newer.lmdata",
            "{\"format\":\"prisma3d-lightmap\",\"version\":99}");
    CHECK(!broken.load("newer.lmdata", &error));
    FileSystem::writeText(directory + "/layers.lmdata",
            "{\"format\":\"prisma3d-lightmap\",\"version\":1,\"light_texture\":{\"file\":\"level."
            "light.dds\",\"layers\":5}}");
    CHECK(!broken.load("layers.lmdata", &error));
    FileSystem::writeText(directory + "/nolight.lmdata",
            "{\"format\":\"prisma3d-lightmap\",\"version\":1}");
    CHECK(!broken.load("nolight.lmdata", &error));
    CHECK(broken.getLightTexture() == TextureId());
    LightmapGIData unsaved;
    CHECK(!unsaved.save(directory + "/nothing.lmdata", &error));
    LightmapGIData reloaded;
    CHECK(reloaded.load("level.lmdata", &error));
    CHECK(reloaded.save(directory + "/again.lmdata", &error));
    LightmapGIData third;
    CHECK(third.load("again.lmdata", &error));
    CHECK(third.getLayerCount() == 3 && third.hasShadowmaskTextures() && third.getUserCount() == 2);
    data.releaseImages();
    CHECK(!data.save(directory + "/released.lmdata", &error) && !error.empty());

    FileSystem::getSingleton().removeSearchPath(directory);
}

void testSerialization(Scene& scene, SceneTree& tree)
{
    printf("a LightmapGI node and its data file survive saving and loading a scene\n");
    Rig& rig = scene.rig;
    const ct::String directory = testDirectory();
    FileSystem::getSingleton().addSearchPath(directory);
    hideAll(scene);
    rig.server->viewportSetUpdateMode(rig.view, ViewportUpdateMode::Disabled);
    Node* root = tree.getRoot();
    if (!root) return;

    const Pixels pixels = makePixels(8, 8, 1, [](int, int, int, int, int) { return kFlat; });
    {
        ct::Unique<LightmapGIData> baked = bakedData(pixels, "../Plane");
        ct::String error;
        CHECK(baked->save(directory + "/scene.lmdata", &error));
    }

    LightmapGI* plain = root->createChild<LightmapGI>();
    ct::String defaultText;
    CHECK(SceneSerializer::save(plain, defaultText));
    CHECK(defaultText.find("quality") == ct::String::npos);
    CHECK(defaultText.find("light_data") == ct::String::npos);
    plain->queueFree();

    LightmapGI* source = root->createChild<LightmapGI>();
    source->setName("Baked");
    CHECK(source->set("quality", PropertyValue::fromInt(2)));
    CHECK(source->set("bounces", PropertyValue::fromInt(5)));
    CHECK(source->set("directional", PropertyValue::fromBool(true)));
    CHECK(source->set("specular_intensity", PropertyValue::fromSingle(0.75f)));
    CHECK(source->set("shadowmask_mode", PropertyValue::fromInt(1)));
    CHECK(source->set("bias", PropertyValue::fromSingle(0.002f)));
    CHECK(source->set("environment_mode", PropertyValue::fromInt(2)));
    CHECK(source->set("environment_custom_color",
            PropertyValue::fromColor(Math::Vec4(0.2f, 0.4f, 0.6f, 1.0f))));
    CHECK(source->set("generate_probes_subdiv", PropertyValue::fromInt(3)));
    CHECK(source->set("light_data", PropertyValue::fromResourcePath("scene.lmdata")));
    CHECK(source->getLightData() != nullptr);
    CHECK(source->getLightDataPath() == "scene.lmdata");

    ct::String text;
    SceneResult saved;
    CHECK(SceneSerializer::save(source, text, &saved));
    CHECK(saved.success());
    CHECK(text.find("light_data") != ct::String::npos &&
            text.find("scene.lmdata") != ct::String::npos);
    CHECK(text.find("shadowmask_mode") != ct::String::npos);

    SceneResult loadedResult;
    Node* holder = root->createChild<Node>();
    Node* copy = SceneSerializer::load(text, holder, loadedResult);
    CHECK(copy != nullptr && loadedResult.success());
    LightmapGI* loaded = copy ? copy->castTo<LightmapGI>() : nullptr;
    CHECK(loaded != nullptr);
    if (loaded)
    {
        CHECK(loaded->getBakeQuality() == LightmapGI::BakeQuality::High);
        CHECK(loaded->getBounces() == 5 && loaded->isDirectional());
        CHECK(loaded->getSpecularIntensity() == 0.75f);
        CHECK(loaded->getShadowmaskMode() == ShadowmaskMode::Replace);
        CHECK(loaded->getBias() == 0.002f);
        CHECK(loaded->getEnvironmentMode() == LightmapGI::EnvironmentMode::CustomColor);
        CHECK(loaded->getEnvironmentCustomColor().y == 0.4f);
        CHECK(loaded->getGenerateProbes() == LightmapGI::GenerateProbes::Subdiv16);
        CHECK(loaded->getLightDataPath() == "scene.lmdata");
        CHECK(loaded->getLightData() != nullptr);
        if (loaded->getLightData())
        {
            CHECK(loaded->getLightData()->getUserCount() == 1);
            CHECK(loaded->getLightData()->getLightTexture().valid());
        }
        PropertyInfo info;
        CHECK(NodeClassDB::getPropertyInfo("LightmapGI", "specular_intensity", info));
        PropertyValue value;
        CHECK(NodeClassDB::getProperty(loaded, "light_data", value) &&
                value.asString() == "scene.lmdata");
    }

    ct::String again;
    CHECK(copy && SceneSerializer::save(copy, again));
    CHECK(again == text);

    printf("a missing data file keeps its path and does not stop the scene\n");
    LightmapGI* orphan = root->createChild<LightmapGI>();
    CHECK(orphan->set("light_data", PropertyValue::fromResourcePath("not_there.lmdata")));
    CHECK(orphan->getLightData() == nullptr && orphan->getLightDataPath() == "not_there.lmdata");
    ct::String orphanText;
    CHECK(SceneSerializer::save(orphan, orphanText));
    CHECK(orphanText.find("not_there.lmdata") != ct::String::npos);

    holder->queueFree();
    orphan->queueFree();
    source->queueFree();
    tree.process(1.0 / 60.0);
    rig.server->viewportSetUpdateMode(rig.view, ViewportUpdateMode::Always);
    FileSystem::getSingleton().removeSearchPath(directory);
    removeTree(directory);
}

} // namespace

int main(int argc, char** argv)
{
    const bool vulkan = argc > 1 && strcmp(argv[1], "vulkan") == 0;

    CHECK(Device::headlessSupported());
    if (!Device::headlessSupported()) return 1;

    DeviceDesc desc;
    desc.title = "lightmap test";
    desc.driver = vulkan ? prisma::DriverType::Vulkan : prisma::DriverType::OpenGL;
    desc.width = kWidth;
    desc.height = kHeight;
    desc.headless = true;

    CHECK(Main::setup(desc));
    if (!Main::device()) return 1;
    SceneTree tree;
    CHECK(Main::start(&tree));

    Scene scene;
    scene.tag = vulkan ? "vulkan" : "opengl";
    Rig& rig = scene.rig;
    rig.server = Main::rendering();
    rig.driver = Main::device()->driver();
    rig.textures = Main::textures();
    RenderingServer& server = *rig.server;
    MeshStore& meshes = *Main::meshes();

    rig.scenario = server.scenarioCreate();
    rig.env = ambientColour(Colour{ 1.0f, 1.0f, 1.0f });
    rig.apply();

    scene.whole = meshes.create("lightmap plane");
    CHECK(meshes.addSurface(scene.whole,
            planeSurface(kAlbedo, true, -kPlaneHalfWidth, kPlaneHalfWidth)));
    scene.left = meshes.create("lightmap left plane");
    CHECK(meshes.addSurface(scene.left, planeSurface(kAlbedo, true, -kPlaneHalfWidth, 0.0f)));
    scene.right = meshes.create("lightmap right plane");
    CHECK(meshes.addSurface(scene.right, planeSurface(kAlbedo, true, 0.0f, kPlaneHalfWidth)));
    scene.plane = server.instanceCreate(scene.whole, rig.scenario);
    scene.leftPlane = server.instanceCreate(scene.left, rig.scenario);
    scene.rightPlane = server.instanceCreate(scene.right, rig.scenario);

    rig.camera = server.cameraCreate();
    server.cameraSetPerspective(rig.camera, kFov, 0.1f, 100.0f);
    server.cameraSetTransform(rig.camera, Math::Mat4::Identity());
    rig.view = server.viewportCreate();
    server.viewportSetSize(rig.view, kWidth, kHeight);
    server.viewportAttachCamera(rig.view, rig.camera);
    server.viewportSetScenario(rig.view, rig.scenario);
    server.viewportSetClearColor(rig.view, Math::Vec4(0.0f, 0.0f, 0.0f, 1.0f));
    server.viewportSetUpdateMode(rig.view, ViewportUpdateMode::Always);
    CHECK(server.viewportGetTexture(rig.view).valid());
    CHECK(server.getLightmapBicubicFilter());
    server.setLightmapBicubicFilter(false);

    testDefaultsAndFlat(scene);
    testGradient(scene);
    testSlicesAndScale(scene);
    testMultipleLightmaps(scene);
    testSphericalHarmonics(scene);
    testBakeModes(scene);
    testShadowmask(scene);
    testScissorMaterial(scene);
    showcase(scene);
    testHalfFloats();
    testDataFile(scene);
    testNodes(scene, tree);
    testSerialization(scene, tree);

    Main::device()->requestClose();
    tree.finalize();
    Main::cleanup();

    if (failures) printf("%d check(s) failed\n", failures);
    else
        printf("all checks passed\n");
    return failures ? 1 : 0;
}
