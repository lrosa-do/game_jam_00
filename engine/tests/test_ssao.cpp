#include "Check.h"
#include "ColouredBox.h"
#include "LitTest.h"

#include "prisma3d/Device.h"
#include "prisma3d/core/Pixmap.h"
#include "prisma3d/main/Main.h"
#include "prisma3d/render/MaterialStore.h"
#include "prisma3d/render/MeshStore.h"
#include "prisma3d/render/RenderProfiler.h"
#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/render/TextureStore.h"
#include "prisma3d/scene/SceneTree.h"

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
const Math::Vec3 kEye(0.0f, 3.5f, 7.0f);
const Math::Vec3 kTarget(0.0f, 0.5f, 0.0f);

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

    int value(int x, int y) const
    {
        return channel(x, y, 1);
    }

    bool same(const Image& other) const
    {
        return width == other.width && height == other.height &&
               pixels.size() == other.pixels.size() &&
               memcmp(pixels.data(), other.pixels.data(), pixels.size()) == 0;
    }
};

struct Rig
{
    RenderingServer* server = nullptr;
    prisma::Driver* driver = nullptr;
    TextureStore* textures = nullptr;
    MaterialStore* materials = nullptr;
    ScenarioId scenario;
    CameraId camera;
    ViewportId view;
    InstanceId floor;
    InstanceId wall;
    InstanceId box;
    LightId sun;
    EnvironmentSettings env;
    RenderStats stats;
    int width = kWidth;
    int height = kHeight;
    int crease = 0;

    void apply()
    {
        server->scenarioSetEnvironment(scenario, env);
    }

    void draw(int primes)
    {
        for (int i = 0; i < primes; ++i)
        {
            server->renderViewports();
            server->endFrame(true);
        }
        server->renderViewports();
        stats = server->getStats();
    }

    bool capture(Image& image, int primes = 2)
    {
        draw(primes);
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

    void resize(int w, int h)
    {
        width = w;
        height = h;
        server->viewportSetSize(view, static_cast<std::uint32_t>(w), static_cast<std::uint32_t>(h));
    }
};

SurfaceData quad(const Math::Vec3& a, const Math::Vec3& b, const Math::Vec3& c, const Math::Vec3& d,
        const Math::Vec3& normal)
{
    SurfaceData data;
    data.positions.push_back(a);
    data.positions.push_back(b);
    data.positions.push_back(c);
    data.positions.push_back(d);
    for (int i = 0; i < 4; ++i)
    {
        data.colors.push_back(Math::Vec4(1.0f, 1.0f, 1.0f, 1.0f));
        data.normals.push_back(normal);
    }
    data.uvs.push_back(Math::Vec2(0.0f, 0.0f));
    data.uvs.push_back(Math::Vec2(1.0f, 0.0f));
    data.uvs.push_back(Math::Vec2(1.0f, 1.0f));
    data.uvs.push_back(Math::Vec2(0.0f, 1.0f));
    const std::uint32_t indices[6] = { 0, 1, 2, 0, 2, 3 };
    for (std::uint32_t index: indices) data.indices.push_back(index);
    return data;
}

int creaseRow(const Rig& rig)
{
    const Math::Mat4 view = Math::Mat4::LookAt(kEye, kTarget, Math::Vec3(0.0f, 1.0f, 0.0f));
    const Math::Vec4 point = view * Math::Vec4(0.0f, 0.0f, 0.0f, 1.0f);
    const float focal =
            static_cast<float>(rig.height) * 0.5f / tanf(kFov * 0.5f * 3.14159265358979f / 180.0f);
    return static_cast<int>(
            floorf(static_cast<float>(rig.height) * 0.5f + focal * point.y / (-point.z)));
}

int dropAround(const Image& off, const Image& on, int x, int row, int radius)
{
    int best = 0;
    for (int y = row - radius; y <= row + radius; ++y)
    {
        if (y < 0 || y >= off.height) continue;
        const int drop = off.value(x, y) - on.value(x, y);
        if (drop > best) best = drop;
    }
    return best;
}

long totalDifference(const Image& a, const Image& b)
{
    long sum = 0;
    for (std::size_t i = 0; i < a.pixels.size(); ++i)
        sum += labs(static_cast<long>(a.pixels[i]) - static_cast<long>(b.pixels[i]));
    return sum;
}

void dump(const Image& image, const char* name)
{
    const char* directory = getenv("PRISMA3D_SHOTS");
    if (!directory || !directory[0]) return;
    Pixmap pixmap(image.width, image.height, 4);
    for (int y = 0; y < image.height; ++y)
        for (int x = 0; x < image.width; ++x)
            pixmap.set_pixel(static_cast<std::uint32_t>(x), static_cast<std::uint32_t>(y),
                    static_cast<std::uint8_t>(image.channel(x, y, 0)),
                    static_cast<std::uint8_t>(image.channel(x, y, 1)),
                    static_cast<std::uint8_t>(image.channel(x, y, 2)), 255);
    char path[512];
    snprintf(path, sizeof(path), "%s/%s.png", directory, name);
    pixmap.save(path);
}

void dumpTexture(Rig& rig, prisma::TextureHandle texture, const char* name)
{
    const char* directory = getenv("PRISMA3D_SHOTS");
    if (!directory || !directory[0] || !texture.valid()) return;
    Image image;
    image.width = rig.width;
    image.height = rig.height;
    image.pixels.resize(
            static_cast<std::size_t>(rig.width) * static_cast<std::size_t>(rig.height) * 4);
    prisma::RenderTarget source;
    source.texture = texture;
    prisma::Rect rect;
    rect.width = static_cast<std::uint32_t>(rig.width);
    rect.height = static_cast<std::uint32_t>(rig.height);
    if (rig.driver->readPixels(source, rect, image.pixels.data())) dump(image, name);
}

const ProfileSample* gpuSample(const char* name)
{
    RenderProfiler* profiler = Main::profiler();
    if (!profiler) return nullptr;
    for (std::uint32_t i = 0; i < profiler->gpuSampleCount(); ++i)
        if (profiler->gpuSamples()[i].name && strcmp(profiler->gpuSamples()[i].name, name) == 0)
            return &profiler->gpuSamples()[i];
    return nullptr;
}

bool passRegistered(const char* name)
{
    RenderProfiler* profiler = Main::profiler();
    if (!profiler) return false;
    for (std::uint32_t i = 0; i < profiler->passCount(); ++i)
        if (strcmp(profiler->passName(i), name) == 0) return true;
    return false;
}

EnvironmentSettings ambientOnly()
{
    EnvironmentSettings env = whiteAmbient();
    env.background = BackgroundMode::Color;
    env.backgroundColor = Math::Vec3(0.2f, 0.3f, 0.4f);
    return env;
}

void ssaoOn(Rig& rig, float intensity = 2.0f, float radius = 1.0f)
{
    rig.env.ssaoEnabled = true;
    rig.env.ssaoIntensity = intensity;
    rig.env.ssaoRadius = radius;
    rig.apply();
}

void ssaoOff(Rig& rig)
{
    rig.env.ssaoEnabled = false;
    rig.apply();
}

void testDisabled(Rig& rig)
{
    printf("ssao off: no pass, no buffer, the frame does not change\n");
    rig.env = ambientOnly();
    rig.apply();
    Image reference;
    CHECK(rig.capture(reference));
    CHECK(rig.stats.ssaoDispatches == 0);
    CHECK(rig.stats.ssaoTexturesLive == 0);
    CHECK(rig.stats.ssaoTargetsCreated == 0);
    CHECK(rig.stats.normalRoughnessDrawCalls == 0);
    CHECK(rig.stats.normalRoughnessTexturesLive == 0);
    CHECK(!passRegistered("SSAO"));
    CHECK(rig.stats.depthPrePassDrawCalls > 0);

    EnvironmentSettings changed = ambientOnly();
    changed.ssaoEnabled = false;
    changed.ssaoRadius = 8.0f;
    changed.ssaoIntensity = 16.0f;
    changed.ssaoPower = 4.0f;
    changed.ssaoDetail = 5.0f;
    changed.ssaoLightAffect = 1.0f;
    changed.ssaoAoChannelAffect = 1.0f;
    rig.env = changed;
    rig.apply();
    Image other;
    CHECK(rig.capture(other));
    CHECK(other.same(reference));
    CHECK(rig.stats.ssaoDispatches == 0);
    CHECK(rig.stats.ssaoTexturesLive == 0);
    CHECK(rig.stats.normalRoughnessTexturesLive == 0);
    CHECK(!passRegistered("SSAO"));
}

void testCorner(Rig& rig, const char* tag)
{
    printf("ssao darkens the crease and the foot of the box, far pixels stay\n");
    rig.env = ambientOnly();
    rig.apply();
    Image off;
    CHECK(rig.capture(off));
    ssaoOn(rig);
    Image on;
    CHECK(rig.capture(on));
    CHECK(rig.stats.ssaoDispatches > 0);
    CHECK(rig.stats.ssaoTargetsCreated <= 1);
    CHECK(rig.stats.normalRoughnessDrawCalls > 0);
    CHECK(rig.stats.normalRoughnessTexturesLive == 1);
    CHECK(rig.stats.ssaoTexturesLive == 4);
    CHECK(passRegistered("SSAO"));

    const int row = creaseRow(rig);
    const int crease = dropAround(off, on, 40, row, 6);
    printf("  crease row %d, drop %d\n", row, crease);
    CHECK(crease >= 20);
    CHECK(abs(off.value(10, 4) - on.value(10, 4)) <= 3);
    CHECK(abs(off.value(80, kHeight - 3) - on.value(80, kHeight - 3)) <= 3);
    for (int x = 0; x < kWidth; x += 7)
        CHECK(abs(off.value(x, kHeight - 1) - on.value(x, kHeight - 1)) <= 4);
    CHECK(abs(off.value(150, 40) - on.value(150, 40)) <= 3);
    CHECK(totalDifference(off, on) > 2000);
    char name[64];
    snprintf(name, sizeof(name), "ssao_off_%s", tag);
    dump(off, name);
    snprintf(name, sizeof(name), "ssao_on_%s", tag);
    dump(on, name);
    snprintf(name, sizeof(name), "ssao_buffer_%s", tag);
    dumpTexture(rig, rig.server->viewportGetAmbientOcclusion(rig.view), name);
    snprintf(name, sizeof(name), "ssao_normals_%s", tag);
    dumpTexture(rig, rig.server->viewportGetNormalRoughness(rig.view), name);
    ssaoOff(rig);
}

void testIntensity(Rig& rig)
{
    printf("ssao intensity: zero changes nothing, then darker and darker\n");
    rig.env = ambientOnly();
    rig.apply();
    Image off;
    CHECK(rig.capture(off));
    const int row = creaseRow(rig);

    ssaoOn(rig, 0.0f);
    Image zero;
    CHECK(rig.capture(zero));
    CHECK(dropAround(off, zero, 40, row, 6) <= 2);

    int previous = 0;
    const float levels[5] = { 0.25f, 0.5f, 1.0f, 2.0f, 4.0f };
    for (int i = 0; i < 5; ++i)
    {
        ssaoOn(rig, levels[i]);
        Image image;
        CHECK(rig.capture(image));
        const int drop = dropAround(off, image, 40, row, 6);
        printf("  intensity %.2f drop %d\n", levels[i], drop);
        CHECK(drop >= previous);
        if (i > 0) CHECK(drop > previous);
        previous = drop;
    }
    ssaoOff(rig);
}

void testRadius(Rig& rig)
{
    printf("ssao radius: a small radius leaves the far side of the floor alone\n");
    rig.env = ambientOnly();
    rig.apply();
    Image off;
    CHECK(rig.capture(off));
    ssaoOn(rig, 2.0f, 0.2f);
    Image small;
    CHECK(rig.capture(small));
    ssaoOn(rig, 2.0f, 3.0f);
    Image large;
    CHECK(rig.capture(large));
    CHECK(totalDifference(off, large) > totalDifference(off, small));
    ssaoOff(rig);
}

void testQualities(Rig& rig)
{
    printf("every quality, with and without half size\n");
    rig.env = ambientOnly();
    rig.apply();
    Image off;
    CHECK(rig.capture(off));
    const int row = creaseRow(rig);
    ssaoOn(rig);
    const SsaoQuality qualities[5] = { SsaoQuality::VeryLow, SsaoQuality::Low, SsaoQuality::Medium,
        SsaoQuality::High, SsaoQuality::Ultra };
    const char* names[5] = { "very low", "low", "medium", "high", "ultra" };
    std::uint32_t dispatchesFull[5] = {};
    for (int half = 0; half < 2; ++half)
    {
        for (int q = 0; q < 5; ++q)
        {
            rig.server->setSsaoQuality(qualities[q], half != 0, 0.5f, 2, 50.0f, 300.0f);
            Image image;
            CHECK(rig.capture(image));
            const int drop = dropAround(off, image, 40, row, 6);
            printf("  %-8s half %d: drop %d dispatches %u\n", names[q], half, drop,
                    rig.stats.ssaoDispatches);
            CHECK(drop >= 10);
            CHECK(rig.stats.ssaoDispatches > 0);
            CHECK(abs(off.value(10, 4) - image.value(10, 4)) <= 3);
            if (half == 0) dispatchesFull[q] = rig.stats.ssaoDispatches;
            else
                CHECK(rig.stats.ssaoDispatches == dispatchesFull[q]);
            char name[64];
            snprintf(name, sizeof(name), "ssao_q%d_h%d", q, half);
            dump(image, name);
        }
    }
    CHECK(dispatchesFull[4] > dispatchesFull[2]);
    CHECK(dispatchesFull[0] < dispatchesFull[1]);
    rig.server->setSsaoQuality(SsaoQuality::Medium, true, 0.5f, 2, 50.0f, 300.0f);
    ssaoOff(rig);
}

void testToggle(Rig& rig)
{
    printf("turning ssao off frees the buffers and gives back the first image\n");
    rig.env = ambientOnly();
    rig.apply();
    Image off;
    CHECK(rig.capture(off));
    ssaoOn(rig);
    Image on;
    CHECK(rig.capture(on));
    CHECK(rig.stats.ssaoTexturesLive > 0);
    ssaoOff(rig);
    Image again;
    CHECK(rig.capture(again));
    CHECK(again.same(off));
    CHECK(rig.stats.ssaoTexturesLive == 0);
    CHECK(rig.stats.normalRoughnessTexturesLive == 0);
    CHECK(rig.stats.ssaoDispatches == 0);
    ssaoOn(rig);
    Image back;
    CHECK(rig.capture(back));
    CHECK(back.same(on));
}

void testLightAffect(Rig& rig)
{
    printf("ssao light affect and ao channel affect\n");
    EnvironmentSettings env = whiteAmbient();
    env.ambientEnergy = 0.4f;
    env.background = BackgroundMode::Color;
    env.backgroundColor = Math::Vec3(0.2f, 0.3f, 0.4f);
    rig.env = env;
    rig.apply();
    rig.server->lightSetVisible(rig.sun, true);
    Image off;
    CHECK(rig.capture(off));
    const int row = creaseRow(rig);
    ssaoOn(rig);
    Image base;
    CHECK(rig.capture(base));
    rig.env.ssaoLightAffect = 1.0f;
    rig.env.ssaoAoChannelAffect = 1.0f;
    rig.apply();
    Image affected;
    CHECK(rig.capture(affected));
    const int dropBase = dropAround(off, base, 40, row, 6);
    const int dropAffected = dropAround(off, affected, 40, row, 6);
    printf("  drop %d with the light untouched, %d with light affect 1\n", dropBase, dropAffected);
    CHECK(dropAffected >= dropBase);
    CHECK(totalDifference(base, affected) > 0);
    rig.server->lightSetVisible(rig.sun, false);
    ssaoOff(rig);
}

void testNoCompute(Rig& rig)
{
    printf("without compute shaders the pass is skipped and the frame is the plain one\n");
    rig.env = ambientOnly();
    rig.apply();
    Image off;
    CHECK(rig.capture(off));
    ssaoOn(rig);
    rig.server->setScreenSpaceEffectsAllowed(false);
    Image skipped;
    CHECK(rig.capture(skipped));
    CHECK(skipped.same(off));
    CHECK(rig.stats.ssaoDispatches == 0);
    CHECK(rig.stats.ssaoTexturesLive == 0);
    CHECK(rig.stats.normalRoughnessTexturesLive == 0);
    CHECK(rig.env.ssaoEnabled);
    rig.server->setScreenSpaceEffectsAllowed(true);
    Image on;
    CHECK(rig.capture(on));
    CHECK(!on.same(off));
    CHECK(rig.stats.ssaoDispatches > 0);
    ssaoOff(rig);
}

void testProfiler(Rig& rig)
{
    printf("the ssao passes show in the profiler only while they run\n");
    RenderProfiler* profiler = Main::profiler();
    CHECK(profiler != nullptr);
    if (!profiler) return;
    rig.env = ambientOnly();
    rig.apply();
    ssaoOn(rig);
    Image on;
    CHECK(rig.capture(on));
    CHECK(passRegistered("SSAO"));
    CHECK(profiler->passEnabled("SSAO"));
    if (profiler->gpuAvailable())
    {
        const char* names[4] = { "SSAO downsample", "SSAO gather", "SSAO blur", "SSAO interleave" };
        for (int i = 0; i < 60; ++i)
        {
            const ProfileSample* sample = gpuSample("SSAO interleave");
            if (sample && sample->historyCount >= 2) break;
            Main::iteration();
        }
        for (int i = 0; i < 4; ++i)
        {
            const ProfileSample* sample = gpuSample(names[i]);
            CHECK(sample != nullptr);
            if (sample)
            {
                printf("  gpu %-16s %.4f ms\n", names[i], sample->milliseconds);
                CHECK(sample->milliseconds >= 0.0f);
            }
        }
    }
    else
        printf("  gpu timer queries are not available on this driver\n");

    profiler->setPassEnabled("SSAO", false);
    Image disabled;
    CHECK(rig.capture(disabled));
    CHECK(rig.stats.ssaoDispatches == 0);
    CHECK(rig.stats.ssaoTexturesLive == 0);
    profiler->setPassEnabled("SSAO", true);
    Image again;
    CHECK(rig.capture(again));
    CHECK(again.same(on));
    CHECK(rig.stats.ssaoDispatches > 0);
    ssaoOff(rig);
}

void testResize(Rig& rig)
{
    printf("ssao follows the size of the viewport\n");
    rig.env = ambientOnly();
    rig.apply();
    ssaoOn(rig);
    const int sizes[4][2] = { { 97, 61 }, { 160, 120 }, { 33, 17 }, { 160, 120 } };
    for (int i = 0; i < 4; ++i)
    {
        rig.resize(sizes[i][0], sizes[i][1]);
        Image image;
        CHECK(rig.capture(image));
        CHECK(rig.stats.ssaoDispatches > 0);
        CHECK(rig.stats.ssaoTexturesLive == 4);
    }
    ssaoOff(rig);
}

TextureId flatTexture(Rig& rig, std::uint8_t r, std::uint8_t g, std::uint8_t b, bool srgb)
{
    Pixmap pixels(4, 4, 4);
    for (std::uint32_t y = 0; y < 4; ++y)
        for (std::uint32_t x = 0; x < 4; ++x) pixels.set_pixel(x, y, r, g, b, 255);
    return rig.textures->createFromPixmap(pixels, srgb, false, "ssao flat");
}

int columnDrop(const Image& off, const Image& on, int x)
{
    int best = 0;
    for (int y = 0; y < off.height; ++y)
    {
        const int drop = off.value(x, y) - on.value(x, y);
        if (drop > best) best = drop;
    }
    return best;
}

void testMaterials(Rig& rig, MeshId wallMesh, MaterialId grey)
{
    printf("the normal pass follows the material: normal map, cut-out, multimesh, orthogonal\n");
    rig.env = ambientOnly();
    rig.apply();
    rig.server->instanceSetVisible(rig.box, false);
    Image off;
    CHECK(rig.capture(off));
    const int row = creaseRow(rig);

    const MaterialId mapped = rig.materials->create("ssao mapped");
    rig.materials->setColor(mapped, MaterialParam::Albedo, Math::Vec4(0.8f, 0.8f, 0.8f, 1.0f));
    rig.materials->setFloat(mapped, MaterialParam::Specular, 0.0f);
    rig.materials->setFloat(mapped, MaterialParam::Roughness, 1.0f);
    rig.materials->setTexture(mapped, MaterialTexture::Normal,
            flatTexture(rig, 128, 128, 255, false));
    rig.materials->setTexture(mapped, MaterialTexture::Roughness,
            flatTexture(rig, 200, 200, 200, false));
    rig.server->instanceSetMaterialOverride(rig.floor, mapped);
    Image mappedOff;
    CHECK(rig.capture(mappedOff));
    ssaoOn(rig);
    Image mappedOn;
    CHECK(rig.capture(mappedOn));
    printf("  normal map and roughness texture: drop %d\n",
            dropAround(mappedOff, mappedOn, 40, row, 6));
    CHECK(rig.stats.normalRoughnessDrawCalls > 0);
    CHECK(dropAround(mappedOff, mappedOn, 40, row, 6) >= 15);
    ssaoOff(rig);
    rig.server->instanceSetMaterialOverride(rig.floor, grey);

    Pixmap halves(64, 8, 4);
    for (std::uint32_t y = 0; y < 8; ++y)
        for (std::uint32_t x = 0; x < 64; ++x)
            halves.set_pixel(x, y, 255, 255, 255, x < 32 ? 255 : 0);
    const MaterialId cutout = rig.materials->create("ssao cutout");
    rig.materials->setColor(cutout, MaterialParam::Albedo, Math::Vec4(0.8f, 0.8f, 0.8f, 1.0f));
    rig.materials->setFloat(cutout, MaterialParam::Specular, 0.0f);
    rig.materials->setFloat(cutout, MaterialParam::Roughness, 1.0f);
    rig.materials->setTexture(cutout, MaterialTexture::Albedo,
            rig.textures->createFromPixmap(halves, true, false, "ssao halves"));
    rig.materials->setTransparency(cutout, MaterialTransparency::AlphaScissor);
    rig.materials->setFloat(cutout, MaterialParam::AlphaScissorThreshold, 0.5f);
    rig.materials->setCull(cutout, MaterialCull::Disabled);
    rig.server->instanceSetMaterialOverride(rig.wall, cutout);
    Image cutOff;
    CHECK(rig.capture(cutOff));
    ssaoOn(rig);
    Image cutOn;
    CHECK(rig.capture(cutOn));
    const int solid = columnDrop(cutOff, cutOn, 40);
    const int hole = columnDrop(cutOff, cutOn, 110);
    printf("  cut-out wall: drop %d on the solid half, %d on the cut half\n", solid, hole);
    CHECK(solid >= 15);
    CHECK(hole <= 3);
    ssaoOff(rig);
    rig.server->instanceSetMaterialOverride(rig.wall, grey);

    const MultiMeshId mm = rig.server->multimeshCreate();
    rig.server->multimeshAllocateData(mm, 1, false, false);
    rig.server->multimeshSetMesh(mm, wallMesh);
    rig.server->multimeshInstanceSetTransform(mm, 0, Math::Mat4::Identity());
    const InstanceId instance = rig.server->instanceCreate();
    rig.server->instanceSetBase(instance, mm);
    rig.server->instanceSetScenario(instance, rig.scenario);
    rig.server->instanceSetMaterialOverride(instance, grey);
    rig.server->instanceSetVisible(rig.wall, false);
    ssaoOn(rig);
    Image multiOn;
    CHECK(rig.capture(multiOn));
    ssaoOff(rig);
    Image multiOff;
    CHECK(rig.capture(multiOff));
    printf("  multimesh wall: drop %d\n", dropAround(multiOff, multiOn, 40, row, 6));
    CHECK(dropAround(multiOff, multiOn, 40, row, 6) >= 15);
    rig.server->instanceSetVisible(instance, false);
    rig.server->instanceFree(instance);
    rig.server->multimeshFree(mm);
    rig.server->instanceSetVisible(rig.wall, true);

    rig.server->cameraSetOrthogonal(rig.camera, 9.0f, 0.1f, 100.0f);
    rig.server->cameraSetTransform(rig.camera,
            Math::Mat4::LookAt(kEye, kTarget, Math::Vec3(0.0f, 1.0f, 0.0f)).InverseAffine());
    Image orthoOff;
    CHECK(rig.capture(orthoOff));
    ssaoOn(rig);
    Image orthoOn;
    CHECK(rig.capture(orthoOn));
    const int ortho = columnDrop(orthoOff, orthoOn, 40);
    printf("  orthogonal camera: drop %d\n", ortho);
    CHECK(ortho >= 15);
    CHECK(abs(orthoOff.value(10, 4) - orthoOn.value(10, 4)) <= 3);
    CHECK(rig.stats.ssaoDispatches > 0);
    dump(orthoOn, "ssao_ortho");
    ssaoOff(rig);
    rig.server->cameraSetPerspective(rig.camera, kFov, 0.1f, 100.0f);
    rig.server->cameraSetTransform(rig.camera,
            Math::Mat4::LookAt(kEye, kTarget, Math::Vec3(0.0f, 1.0f, 0.0f)).InverseAffine());
    rig.server->instanceSetVisible(rig.box, true);
}

void testMultisampling(Rig& rig)
{
    printf("ssao is skipped while multisampling is on\n");
    rig.env = ambientOnly();
    rig.apply();
    ssaoOn(rig);
    rig.server->viewportSetMsaa3D(rig.view, ViewportMsaa::Msaa4x);
    Image image;
    CHECK(rig.capture(image));
    if (rig.server->viewportGetMsaaSamples(rig.view) > 1)
    {
        CHECK(rig.stats.ssaoDispatches == 0);
        CHECK(rig.stats.ssaoTexturesLive == 0);
    }
    rig.server->viewportSetMsaa3D(rig.view, ViewportMsaa::Disabled);
    Image back;
    CHECK(rig.capture(back));
    CHECK(rig.stats.ssaoDispatches > 0);
    ssaoOff(rig);
}

void showcase(Rig& rig, const char* tag)
{
    const char* directory = getenv("PRISMA3D_SHOTS");
    if (!directory || !directory[0]) return;
    rig.server->instanceSetTransform(rig.box,
            Math::Mat4::Translation(Math::Vec3(-1.2f, 0.0f, 1.6f)) *
                    Math::Mat4::Scale(Math::Vec3(1.6f, 1.6f, 1.6f)));
    rig.server->cameraSetTransform(rig.camera,
            Math::Mat4::LookAt(Math::Vec3(3.2f, 3.0f, 6.0f), Math::Vec3(-0.5f, 0.6f, 0.0f),
                    Math::Vec3(0.0f, 1.0f, 0.0f))
                    .InverseAffine());
    rig.resize(640, 480);
    rig.env = ambientOnly();
    rig.apply();
    Image off;
    rig.capture(off);
    char name[96];
    snprintf(name, sizeof(name), "showcase_off_%s", tag);
    dump(off, name);
    ssaoOn(rig);
    const SsaoQuality qualities[3] = { SsaoQuality::Medium, SsaoQuality::High, SsaoQuality::Ultra };
    const char* names[3] = { "medium", "high", "ultra" };
    for (int q = 0; q < 3; ++q)
    {
        for (int half = 0; half < 2; ++half)
        {
            rig.server->setSsaoQuality(qualities[q], half != 0, 0.5f, 2, 50.0f, 300.0f);
            Image on;
            rig.capture(on);
            snprintf(name, sizeof(name), "showcase_on_%s_%s_%s", names[q], half ? "half" : "full",
                    tag);
            dump(on, name);
        }
    }
    rig.server->setSsaoQuality(SsaoQuality::Medium, true, 0.5f, 2, 50.0f, 300.0f);
    ssaoOff(rig);
}

void cost(Rig& rig)
{
    if (!getenv("PRISMA3D_SSAO_COST")) return;
    RenderProfiler* profiler = Main::profiler();
    if (!profiler || !profiler->gpuAvailable()) return;
    rig.resize(1280, 720);
    const char* names[8] = { "Depth", "Opaque", "SSAO downsample", "SSAO gather", "SSAO blur",
        "SSAO interleave", "Final", "Sky" };
    const SsaoQuality qualities[5] = { SsaoQuality::VeryLow, SsaoQuality::Low, SsaoQuality::Medium,
        SsaoQuality::High, SsaoQuality::Ultra };
    const char* qualityNames[5] = { "very low", "low", "medium", "high", "ultra" };
    for (int mode = 0; mode < 11; ++mode)
    {
        const bool on = mode > 0;
        const int q = on ? (mode - 1) / 2 : 0;
        const bool half = on ? ((mode - 1) % 2) == 0 : false;
        rig.env = ambientOnly();
        rig.env.ssaoEnabled = on;
        rig.apply();
        rig.server->setSsaoQuality(qualities[q], half, 0.5f, 2, 50.0f, 300.0f);
        for (int i = 0; i < 40; ++i) Main::iteration();
        float sum[8] = {};
        int frames = 0;
        for (int i = 0; i < 30; ++i)
        {
            Main::iteration();
            for (int n = 0; n < 8; ++n)
            {
                const ProfileSample* sample = gpuSample(names[n]);
                if (sample) sum[n] += sample->milliseconds;
            }
            ++frames;
        }
        if (on) printf("  ssao %-8s %s:", qualityNames[q], half ? "half" : "full");
        else
            printf("  ssao off              :");
        for (int n = 0; n < 8; ++n)
            printf(" %s %.3f", names[n], sum[n] / static_cast<float>(frames));
        printf("\n");
    }
    rig.server->setSsaoQuality(SsaoQuality::Medium, true, 0.5f, 2, 50.0f, 300.0f);
    ssaoOff(rig);
}

} // namespace

int main(int argc, char** argv)
{
    const bool vulkan = argc > 1 && strcmp(argv[1], "vulkan") == 0;

    CHECK(Device::headlessSupported());
    if (!Device::headlessSupported()) return 1;

    DeviceDesc desc;
    desc.title = "ssao test";
    desc.driver = vulkan ? prisma::DriverType::Vulkan : prisma::DriverType::OpenGL;
    desc.width = kWidth;
    desc.height = kHeight;
    desc.headless = true;

    CHECK(Main::setup(desc));
    if (!Main::device()) return 1;
    SceneTree tree;
    CHECK(Main::start(&tree));

    Rig rig;
    rig.server = Main::rendering();
    rig.driver = Main::device()->driver();
    rig.textures = Main::textures();
    rig.materials = Main::materials();

    RenderingServer& server = *rig.server;
    if (!server.screenSpaceEffectsSupported())
    {
        printf("no compute shaders on this driver, nothing to test\n");
        Main::device()->requestClose();
        tree.finalize();
        Main::cleanup();
        return 0;
    }
    MeshStore& meshes = *Main::meshes();
    rig.scenario = server.scenarioCreate();
    rig.env = ambientOnly();
    rig.apply();

    const MeshId floorMesh = meshes.create("ssao floor");
    CHECK(meshes.addSurface(floorMesh,
            quad(Math::Vec3(-8.0f, 0.0f, 10.0f), Math::Vec3(8.0f, 0.0f, 10.0f),
                    Math::Vec3(8.0f, 0.0f, 0.0f), Math::Vec3(-8.0f, 0.0f, 0.0f),
                    Math::Vec3(0.0f, 1.0f, 0.0f))));
    const MeshId wallMesh = meshes.create("ssao wall");
    CHECK(meshes.addSurface(wallMesh,
            quad(Math::Vec3(-8.0f, 0.0f, 0.0f), Math::Vec3(8.0f, 0.0f, 0.0f),
                    Math::Vec3(8.0f, 8.0f, 0.0f), Math::Vec3(-8.0f, 8.0f, 0.0f),
                    Math::Vec3(0.0f, 0.0f, 1.0f))));
    const MeshId boxMesh = meshes.create("ssao box");
    CHECK(meshes.addSurface(boxMesh, colouredBox()));
    rig.floor = server.instanceCreate(floorMesh, rig.scenario);
    rig.wall = server.instanceCreate(wallMesh, rig.scenario);
    rig.box = server.instanceCreate(boxMesh, rig.scenario);
    server.instanceSetTransform(rig.box, Math::Mat4::Translation(Math::Vec3(2.5f, 0.0f, 3.0f)) *
                                                 Math::Mat4::Scale(Math::Vec3(1.5f, 1.5f, 1.5f)));
    const MaterialId grey = rig.materials->create("ssao grey");
    rig.materials->setColor(grey, MaterialParam::Albedo, Math::Vec4(0.8f, 0.8f, 0.8f, 1.0f));
    rig.materials->setFloat(grey, MaterialParam::Specular, 0.0f);
    rig.materials->setFloat(grey, MaterialParam::Roughness, 1.0f);
    server.instanceSetMaterialOverride(rig.floor, grey);
    server.instanceSetMaterialOverride(rig.wall, grey);
    server.instanceSetMaterialOverride(rig.box, grey);

    rig.sun = server.lightCreate(LightType::Directional);
    server.lightSetScenario(rig.sun, rig.scenario);
    server.lightSetTransform(rig.sun,
            Math::Mat4::LookAt(Math::Vec3(0.0f, 0.0f, 0.0f), Math::Vec3(-0.3f, -1.0f, -0.4f),
                    Math::Vec3(0.0f, 1.0f, 0.0f))
                    .InverseAffine());
    server.lightSetVisible(rig.sun, false);

    rig.camera = server.cameraCreate();
    server.cameraSetPerspective(rig.camera, kFov, 0.1f, 100.0f);
    server.cameraSetTransform(rig.camera,
            Math::Mat4::LookAt(kEye, kTarget, Math::Vec3(0.0f, 1.0f, 0.0f)).InverseAffine());

    rig.view = server.viewportCreate();
    server.viewportSetSize(rig.view, kWidth, kHeight);
    server.viewportAttachCamera(rig.view, rig.camera);
    server.viewportSetScenario(rig.view, rig.scenario);
    server.viewportSetClearColor(rig.view, Math::Vec4(0.0f, 0.0f, 0.0f, 1.0f));
    server.viewportSetUpdateMode(rig.view, ViewportUpdateMode::Always);
    CHECK(server.viewportGetTexture(rig.view).valid());

    testDisabled(rig);
    testCorner(rig, vulkan ? "vulkan" : "opengl");
    testIntensity(rig);
    testRadius(rig);
    testQualities(rig);
    testToggle(rig);
    testLightAffect(rig);
    testNoCompute(rig);
    testProfiler(rig);
    testResize(rig);
    testMaterials(rig, wallMesh, grey);
    testMultisampling(rig);
    showcase(rig, vulkan ? "vulkan" : "opengl");
    cost(rig);

    Main::device()->requestClose();
    tree.finalize();
    Main::cleanup();

    if (failures) printf("%d check(s) failed\n", failures);
    else
        printf("all checks passed\n");
    return failures ? 1 : 0;
}
