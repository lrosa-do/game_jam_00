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
const Math::Vec3 kEye(0.0f, 2.6f, 9.0f);
const Math::Vec3 kTarget(0.0f, 0.0f, 0.0f);
const Math::Vec3 kBoxPosition(0.0f, 0.0f, 3.0f);
const float kBoxSize = 1.5f;

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
    MaterialId mirror;
    LightId sun;
    EnvironmentSettings env;
    RenderStats stats;
    int width = kWidth;
    int height = kHeight;

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

    void lookFrom(const Math::Vec3& eye, const Math::Vec3& target)
    {
        server->cameraSetTransform(camera,
                Math::Mat4::LookAt(eye, target, Math::Vec3(0.0f, 1.0f, 0.0f)).InverseAffine());
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

SurfaceData whiteBox()
{
    SurfaceData data;
    const float h = 0.5f;
    const Math::Vec4 white(1.0f, 1.0f, 1.0f, 1.0f);
    addBoxFace(data, Math::Vec3(-h, 0, h), Math::Vec3(h, 0, h), Math::Vec3(h, 1, h),
            Math::Vec3(-h, 1, h), Math::Vec3(0, 0, 1), white);
    addBoxFace(data, Math::Vec3(h, 0, h), Math::Vec3(h, 0, -h), Math::Vec3(h, 1, -h),
            Math::Vec3(h, 1, h), Math::Vec3(1, 0, 0), white);
    addBoxFace(data, Math::Vec3(h, 0, -h), Math::Vec3(-h, 0, -h), Math::Vec3(-h, 1, -h),
            Math::Vec3(h, 1, -h), Math::Vec3(0, 0, -1), white);
    addBoxFace(data, Math::Vec3(-h, 0, -h), Math::Vec3(-h, 0, h), Math::Vec3(-h, 1, h),
            Math::Vec3(-h, 1, -h), Math::Vec3(-1, 0, 0), white);
    addBoxFace(data, Math::Vec3(-h, 1, h), Math::Vec3(h, 1, h), Math::Vec3(h, 1, -h),
            Math::Vec3(-h, 1, -h), Math::Vec3(0, 1, 0), white);
    addBoxFace(data, Math::Vec3(-h, 0, -h), Math::Vec3(h, 0, -h), Math::Vec3(h, 0, h),
            Math::Vec3(-h, 0, h), Math::Vec3(0, -1, 0), white);
    return data;
}

struct Pixel
{
    float x;
    float y;
};

Pixel project(const Rig& rig, const Math::Vec3& point)
{
    const Math::Mat4 view = Math::Mat4::LookAt(kEye, kTarget, Math::Vec3(0.0f, 1.0f, 0.0f));
    const Math::Vec4 local = view * Math::Vec4(point.x, point.y, point.z, 1.0f);
    const float focal =
            static_cast<float>(rig.height) * 0.5f / tanf(kFov * 0.5f * 3.14159265358979f / 180.0f);
    Pixel pixel;
    pixel.x = static_cast<float>(rig.width) * 0.5f + focal * local.x / (-local.z);
    pixel.y = static_cast<float>(rig.height) * 0.5f + focal * local.y / (-local.z);
    return pixel;
}

struct Region
{
    int x0;
    int y0;
    int x1;
    int y1;
};

Region mirroredBox(const Rig& rig)
{
    float minX = 1e9f, minY = 1e9f, maxX = -1e9f, maxY = -1e9f;
    for (int i = 0; i < 8; ++i)
    {
        const Math::Vec3 corner(kBoxPosition.x + ((i & 1) ? 0.5f : -0.5f) * kBoxSize,
                -((i & 2) ? kBoxSize : 0.0f), kBoxPosition.z + ((i & 4) ? 0.5f : -0.5f) * kBoxSize);
        const Pixel pixel = project(rig, corner);
        if (pixel.x < minX) minX = pixel.x;
        if (pixel.x > maxX) maxX = pixel.x;
        if (pixel.y < minY) minY = pixel.y;
        if (pixel.y > maxY) maxY = pixel.y;
    }
    Region region;
    region.x0 = static_cast<int>(ceilf(minX));
    region.x1 = static_cast<int>(floorf(maxX));
    region.y0 = static_cast<int>(ceilf(minY));
    region.y1 = static_cast<int>(floorf(maxY));
    return region;
}

float excess(const Image& off, const Image& on, const Region& region, int channel)
{
    float sum = 0.0f;
    int count = 0;
    for (int y = region.y0; y <= region.y1; ++y)
    {
        for (int x = region.x0; x <= region.x1; ++x)
        {
            if (x < 0 || y < 0 || x >= off.width || y >= off.height) continue;
            sum += static_cast<float>(on.channel(x, y, channel) - off.channel(x, y, channel));
            ++count;
        }
    }
    return count ? sum / static_cast<float>(count) : 0.0f;
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
            pixmap.set_pixel(static_cast<std::uint32_t>(x),
                    static_cast<std::uint32_t>(image.height - 1 - y),
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

void ssrOn(Rig& rig)
{
    rig.env.ssrEnabled = true;
    rig.apply();
}

void ssrOff(Rig& rig)
{
    rig.env.ssrEnabled = false;
    rig.apply();
}

void setRoughness(Rig& rig, float roughness)
{
    rig.materials->setFloat(rig.mirror, MaterialParam::Roughness, roughness);
}

void testDisabled(Rig& rig)
{
    printf("ssr off: no pass, no buffer, the frame does not change\n");
    rig.env = ambientOnly();
    rig.apply();
    Image reference;
    CHECK(rig.capture(reference));
    CHECK(rig.stats.ssrDispatches == 0);
    CHECK(rig.stats.ssrTexturesLive == 0);
    CHECK(rig.stats.ssrTargetsCreated == 0);
    CHECK(rig.stats.normalRoughnessDrawCalls == 0);
    CHECK(rig.stats.normalRoughnessTexturesLive == 0);
    CHECK(!passRegistered("SSR"));
    CHECK(rig.server->viewportGetReflections(rig.view) == prisma::TextureHandle());
    CHECK(rig.stats.depthPrePassDrawCalls > 0);

    EnvironmentSettings changed = ambientOnly();
    changed.ssrEnabled = false;
    changed.ssrMaxSteps = 512;
    changed.ssrFadeIn = 5.0f;
    changed.ssrFadeOut = 7.0f;
    changed.ssrDepthTolerance = 100.0f;
    rig.env = changed;
    rig.apply();
    Image other;
    CHECK(rig.capture(other));
    CHECK(other.same(reference));
    CHECK(rig.stats.ssrDispatches == 0);
    CHECK(rig.stats.ssrTexturesLive == 0);
    CHECK(rig.stats.normalRoughnessTexturesLive == 0);
    CHECK(!passRegistered("SSR"));
}

void testReflection(Rig& rig, const char* tag)
{
    printf("a mirror floor under a red box shows the box, and only when ssr is on\n");
    rig.env = ambientOnly();
    rig.apply();
    Image off;
    CHECK(rig.capture(off));
    const Region region = mirroredBox(rig);
    printf("  mirrored box on screen: x %d..%d y %d..%d\n", region.x0, region.x1, region.y0,
            region.y1);
    ssrOn(rig);
    Image on;
    CHECK(rig.capture(on));
    CHECK(rig.stats.ssrDispatches > 0);
    CHECK(rig.stats.ssrTargetsCreated <= 1);
    CHECK(rig.stats.normalRoughnessDrawCalls > 0);
    CHECK(rig.stats.normalRoughnessTexturesLive == 1);
    CHECK(rig.stats.ssrTexturesLive == 6);
    CHECK(passRegistered("SSR"));
    CHECK(rig.server->viewportGetReflections(rig.view).valid());

    const float red = excess(off, on, region, 0);
    const float green = excess(off, on, region, 1);
    const float blue = excess(off, on, region, 2);
    printf("  excess over the mirrored box: r %.1f g %.1f b %.1f\n", red, green, blue);
    CHECK(red > 60.0f);
    CHECK(red > green * 2.0f);
    CHECK(red > blue * 2.0f);
    CHECK(totalDifference(off, on) > 5000);

    for (int x = 0; x < kWidth; x += 9) CHECK(abs(on.channel(x, 1, 0) - off.channel(x, 1, 0)) <= 4);
    CHECK(abs(on.channel(8, 70, 0) - off.channel(8, 70, 0)) <= 3);
    CHECK(abs(on.channel(150, 20, 0) - off.channel(150, 20, 0)) <= 6);

    char name[64];
    snprintf(name, sizeof(name), "ssr_off_%s", tag);
    dump(off, name);
    snprintf(name, sizeof(name), "ssr_on_%s", tag);
    dump(on, name);
    snprintf(name, sizeof(name), "ssr_normals_%s", tag);
    dumpTexture(rig, rig.server->viewportGetNormalRoughness(rig.view), name);
    ssrOff(rig);
}

float sharpness(const Image& off, const Image& on, int row, const Region& region)
{
    float peak = 0.0f;
    float steepest = 0.0f;
    float previous = 0.0f;
    for (int x = region.x0 - 12; x <= region.x1 + 12; ++x)
    {
        if (x < 0 || x >= off.width) continue;
        const float e = static_cast<float>(on.channel(x, row, 0) - off.channel(x, row, 0));
        if (e > peak) peak = e;
        const float step = fabsf(e - previous);
        if (x > region.x0 - 12 && step > steepest) steepest = step;
        previous = e;
    }
    return peak > 0.0f ? steepest / peak : 0.0f;
}

void testRoughness(Rig& rig)
{
    printf("roughness: a rougher floor blurs the reflection and loses it near 0.7\n");
    rig.env = ambientOnly();
    rig.apply();
    const Region region = mirroredBox(rig);
    const int row = (region.y0 + region.y1) / 2;
    const float levels[5] = { 0.0f, 0.2f, 0.4f, 0.6f, 0.85f };
    Image off[5];
    Image on[5];
    for (int i = 0; i < 5; ++i)
    {
        setRoughness(rig, levels[i]);
        ssrOff(rig);
        CHECK(rig.capture(off[i]));
        ssrOn(rig);
        CHECK(rig.capture(on[i]));
    }
    float previous = 2.0f;
    float strengths[5];
    for (int i = 0; i < 5; ++i)
    {
        strengths[i] = excess(off[i], on[i], region, 0);
        const float edge = sharpness(off[i], on[i], row, region);
        printf("  roughness %.2f: strength %.1f edge sharpness %.3f\n", levels[i], strengths[i],
                edge);
        if (i < 3)
        {
            CHECK(edge < previous);
            previous = edge;
        }
    }
    CHECK(sharpness(off[0], on[0], row, region) > sharpness(off[2], on[2], row, region) + 0.05f);
    CHECK(strengths[0] > 60.0f);
    CHECK(strengths[3] > 60.0f);
    CHECK(strengths[4] <= 1.0f);
    CHECK(totalDifference(off[4], on[4]) < 600);
    setRoughness(rig, 0.0f);
    ssrOff(rig);
}

float strengthWith(Rig& rig, const Image& off, const Region& region)
{
    Image on;
    CHECK(rig.capture(on));
    return excess(off, on, region, 0);
}

void testSettings(Rig& rig)
{
    printf("max steps, fade and depth tolerance move the reflection one way\n");
    rig.env = ambientOnly();
    rig.apply();
    const Region region = mirroredBox(rig);
    Image off;
    CHECK(rig.capture(off));
    ssrOn(rig);

    const int steps[4] = { 1, 4, 12, 64 };
    float previous = -1.0f;
    for (int i = 0; i < 4; ++i)
    {
        rig.env.ssrMaxSteps = steps[i];
        rig.apply();
        const float strength = strengthWith(rig, off, region);
        printf("  max steps %d: strength %.1f\n", steps[i], strength);
        CHECK(strength >= previous - 0.5f);
        previous = strength;
    }
    rig.env.ssrMaxSteps = 1;
    rig.apply();
    const float oneStep = strengthWith(rig, off, region);
    rig.env.ssrMaxSteps = 64;
    rig.apply();
    const float manySteps = strengthWith(rig, off, region);
    CHECK(manySteps > oneStep + 10.0f);

    const float fadeOuts[4] = { 0.0f, 1.0f, 3.0f, 8.0f };
    previous = 1e9f;
    for (int i = 0; i < 4; ++i)
    {
        rig.env.ssrFadeOut = fadeOuts[i];
        rig.apply();
        const float strength = strengthWith(rig, off, region);
        printf("  fade out %.1f: strength %.1f\n", fadeOuts[i], strength);
        CHECK(strength <= previous + 0.5f);
        previous = strength;
    }
    rig.env.ssrFadeOut = 0.0f;
    rig.apply();
    const float noFadeOut = strengthWith(rig, off, region);
    rig.env.ssrFadeOut = 8.0f;
    rig.apply();
    const float strongFadeOut = strengthWith(rig, off, region);
    CHECK(noFadeOut > strongFadeOut + 10.0f);
    rig.env.ssrFadeOut = 2.0f;

    const float fadeIns[4] = { 0.0f, 0.5f, 1.5f, 4.0f };
    previous = 1e9f;
    for (int i = 0; i < 4; ++i)
    {
        rig.env.ssrFadeIn = fadeIns[i];
        rig.apply();
        const float strength = strengthWith(rig, off, region);
        printf("  fade in %.1f: strength %.1f\n", fadeIns[i], strength);
        CHECK(strength <= previous + 0.5f);
        previous = strength;
    }
    rig.env.ssrFadeIn = 0.0f;
    rig.apply();
    const float noFadeIn = strengthWith(rig, off, region);
    rig.env.ssrFadeIn = 4.0f;
    rig.apply();
    const float strongFadeIn = strengthWith(rig, off, region);
    CHECK(noFadeIn > strongFadeIn + 10.0f);
    rig.env.ssrFadeIn = 0.15f;

    const float tolerances[4] = { 0.01f, 0.1f, 0.5f, 8.0f };
    previous = -1.0f;
    for (int i = 0; i < 4; ++i)
    {
        rig.env.ssrDepthTolerance = tolerances[i];
        rig.apply();
        const float strength = strengthWith(rig, off, region);
        printf("  depth tolerance %.2f: strength %.1f\n", tolerances[i], strength);
        CHECK(strength >= previous - 0.5f);
        previous = strength;
    }
    rig.env.ssrDepthTolerance = 0.01f;
    rig.apply();
    const float tight = strengthWith(rig, off, region);
    rig.env.ssrDepthTolerance = 8.0f;
    rig.apply();
    const float loose = strengthWith(rig, off, region);
    CHECK(loose > tight + 5.0f);
    rig.env.ssrDepthTolerance = 0.5f;
    ssrOff(rig);
}

void testHalfSize(Rig& rig)
{
    printf("half size and full size buffers\n");
    rig.env = ambientOnly();
    rig.apply();
    const Region region = mirroredBox(rig);
    Image off;
    CHECK(rig.capture(off));
    ssrOn(rig);

    rig.server->setSsrHalfSize(true);
    CHECK(rig.server->ssrHalfSize());
    Image half;
    CHECK(rig.capture(half));
    const std::uint32_t halfDispatches = rig.stats.ssrDispatches;
    const std::uint32_t halfTextures = rig.stats.ssrTexturesLive;
    const float halfRed = excess(off, half, region, 0);

    rig.server->setSsrHalfSize(false);
    CHECK(!rig.server->ssrHalfSize());
    Image full;
    CHECK(rig.capture(full));
    const std::uint32_t fullDispatches = rig.stats.ssrDispatches;
    const std::uint32_t fullTextures = rig.stats.ssrTexturesLive;
    const float fullRed = excess(off, full, region, 0);
    printf("  half: %u dispatches, %u textures, red %.1f; full: %u dispatches, %u textures, red "
           "%.1f\n",
            halfDispatches, halfTextures, halfRed, fullDispatches, fullTextures, fullRed);
    CHECK(halfTextures == 6);
    CHECK(fullTextures == 4);
    CHECK(halfDispatches == 14);
    CHECK(fullDispatches == 15);
    CHECK(halfRed > 40.0f);
    CHECK(fullRed > 40.0f);
    CHECK(!half.same(full));
    CHECK(rig.stats.ssrTargetsCreated <= 1);

    rig.server->setSsrHalfSize(true);
    Image again;
    CHECK(rig.capture(again));
    CHECK(again.same(half));
    ssrOff(rig);
}

void testToggle(Rig& rig)
{
    printf("turning ssr off frees the buffers and gives back the first image\n");
    rig.env = ambientOnly();
    rig.apply();
    Image off;
    CHECK(rig.capture(off));
    ssrOn(rig);
    Image on;
    CHECK(rig.capture(on));
    CHECK(rig.stats.ssrTexturesLive > 0);
    CHECK(!on.same(off));
    ssrOff(rig);
    Image again;
    CHECK(rig.capture(again));
    CHECK(again.same(off));
    CHECK(rig.stats.ssrTexturesLive == 0);
    CHECK(rig.stats.normalRoughnessTexturesLive == 0);
    CHECK(rig.stats.ssrDispatches == 0);
    ssrOn(rig);
    Image back;
    CHECK(rig.capture(back));
    CHECK(back.same(on));
    ssrOff(rig);
}

void testNoCompute(Rig& rig)
{
    printf("without compute shaders the pass is skipped and the frame is the plain one\n");
    rig.env = ambientOnly();
    rig.apply();
    Image off;
    CHECK(rig.capture(off));
    ssrOn(rig);
    rig.server->setScreenSpaceEffectsAllowed(false);
    Image skipped;
    CHECK(rig.capture(skipped));
    CHECK(skipped.same(off));
    CHECK(rig.stats.ssrDispatches == 0);
    CHECK(rig.stats.ssrTexturesLive == 0);
    CHECK(rig.stats.normalRoughnessTexturesLive == 0);
    CHECK(rig.env.ssrEnabled);
    rig.server->setScreenSpaceEffectsAllowed(true);
    Image on;
    CHECK(rig.capture(on));
    CHECK(!on.same(off));
    CHECK(rig.stats.ssrDispatches > 0);
    ssrOff(rig);
}

void testProfiler(Rig& rig)
{
    printf("the ssr passes show in the profiler only while they run\n");
    RenderProfiler* profiler = Main::profiler();
    CHECK(profiler != nullptr);
    if (!profiler) return;
    profiler->setEnabled(true);
    rig.env = ambientOnly();
    rig.apply();
    ssrOn(rig);
    Image on;
    CHECK(rig.capture(on, 6));
    CHECK(passRegistered("SSR"));
    CHECK(profiler->passEnabled("SSR"));
    if (profiler->gpuAvailable())
    {
        const char* names[6] = { "SSR downsample", "SSR hi-z", "SSR main", "SSR filter",
            "SSR resolve", "SSR last frame" };
        for (int i = 0; i < 60; ++i)
        {
            const ProfileSample* sample = gpuSample("SSR last frame");
            if (sample && sample->historyCount >= 2) break;
            Main::iteration();
        }
        for (int i = 0; i < 6; ++i)
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

    profiler->setPassEnabled("SSR", false);
    Image disabled;
    CHECK(rig.capture(disabled));
    CHECK(rig.stats.ssrDispatches == 0);
    CHECK(rig.stats.ssrTexturesLive == 0);
    profiler->setPassEnabled("SSR", true);
    Image again;
    CHECK(rig.capture(again, 6));
    CHECK(again.same(on));
    CHECK(rig.stats.ssrDispatches > 0);

    rig.server->setSsrHalfSize(false);
    CHECK(rig.capture(again));
    if (profiler->gpuAvailable())
    {
        for (int i = 0; i < 60; ++i)
        {
            const ProfileSample* sample = gpuSample("SSR copy depth");
            if (sample && sample->historyCount >= 2) break;
            Main::iteration();
        }
        CHECK(gpuSample("SSR copy depth") != nullptr);
    }
    rig.server->setSsrHalfSize(true);
    ssrOff(rig);
}

void testResize(Rig& rig)
{
    printf("ssr follows the size of the viewport\n");
    rig.env = ambientOnly();
    rig.apply();
    ssrOn(rig);
    const int sizes[4][2] = { { 97, 61 }, { 160, 120 }, { 33, 17 }, { 160, 120 } };
    Image first;
    for (int i = 0; i < 4; ++i)
    {
        rig.resize(sizes[i][0], sizes[i][1]);
        Image image;
        CHECK(rig.capture(image));
        CHECK(rig.stats.ssrDispatches > 0);
        CHECK(rig.stats.ssrTexturesLive == 6);
        if (i == 1) first = image;
        if (i == 3) CHECK(image.same(first));
    }
    rig.server->setSsrHalfSize(false);
    for (int i = 0; i < 3; ++i)
    {
        rig.resize(sizes[i][0], sizes[i][1]);
        Image image;
        CHECK(rig.capture(image));
        CHECK(rig.stats.ssrDispatches > 0);
        CHECK(rig.stats.ssrTexturesLive == 4);
    }
    rig.server->setSsrHalfSize(true);
    rig.resize(kWidth, kHeight);
    ssrOff(rig);
}

void testMoving(Rig& rig)
{
    printf("the reflection follows a moving camera\n");
    rig.env = ambientOnly();
    rig.apply();
    ssrOn(rig);
    Image still;
    CHECK(rig.capture(still, 6));
    for (int i = 0; i < 4; ++i)
    {
        const float x = 0.4f * static_cast<float>(i);
        rig.lookFrom(Math::Vec3(kEye.x + x, kEye.y, kEye.z), kTarget);
        rig.draw(0);
        rig.server->endFrame(true);
    }
    rig.lookFrom(kEye, kTarget);
    Image back;
    CHECK(rig.capture(back, 6));
    CHECK(back.same(still));
    CHECK(rig.stats.ssrDispatches > 0);
    ssrOff(rig);
}

void testTogether(Rig& rig)
{
    printf("ssao and ssr together share the normal and roughness buffer\n");
    rig.env = ambientOnly();
    rig.apply();
    const Region region = mirroredBox(rig);
    Image off;
    CHECK(rig.capture(off));
    ssrOn(rig);
    Image onlySsr;
    CHECK(rig.capture(onlySsr));
    rig.env.ssaoEnabled = true;
    rig.apply();
    Image both;
    CHECK(rig.capture(both));
    CHECK(rig.stats.ssaoDispatches > 0);
    CHECK(rig.stats.ssrDispatches > 0);
    CHECK(rig.stats.normalRoughnessTexturesLive == 1);
    CHECK(rig.stats.ssaoTexturesLive > 0);
    CHECK(rig.stats.ssrTexturesLive == 6);
    CHECK(!both.same(onlySsr));
    CHECK(excess(off, both, region, 0) > 40.0f);
    rig.env.ssrEnabled = false;
    rig.apply();
    Image onlySsao;
    CHECK(rig.capture(onlySsao));
    CHECK(rig.stats.ssrDispatches == 0);
    CHECK(rig.stats.ssrTexturesLive == 0);
    CHECK(rig.stats.normalRoughnessTexturesLive == 1);
    CHECK(excess(off, onlySsao, region, 0) < 20.0f);
    rig.env.ssaoEnabled = false;
    rig.apply();
}

void testMultisampling(Rig& rig)
{
    printf("ssr is skipped while multisampling is on\n");
    rig.env = ambientOnly();
    rig.apply();
    ssrOn(rig);
    rig.server->viewportSetMsaa3D(rig.view, ViewportMsaa::Msaa4x);
    Image image;
    CHECK(rig.capture(image));
    if (rig.server->viewportGetMsaaSamples(rig.view) > 1)
    {
        CHECK(rig.stats.ssrDispatches == 0);
        CHECK(rig.stats.ssrTexturesLive == 0);
    }
    rig.server->viewportSetMsaa3D(rig.view, ViewportMsaa::Disabled);
    Image back;
    CHECK(rig.capture(back));
    CHECK(rig.stats.ssrDispatches > 0);
    ssrOff(rig);
}

void testTransparentBackground(Rig& rig)
{
    printf("ssr is skipped in a viewport with a transparent background\n");
    rig.env = ambientOnly();
    rig.apply();
    ssrOn(rig);
    rig.server->viewportSetTransparentBackground(rig.view, true);
    Image image;
    CHECK(rig.capture(image));
    CHECK(rig.stats.ssrDispatches == 0);
    CHECK(rig.stats.ssrTexturesLive == 0);
    rig.server->viewportSetTransparentBackground(rig.view, false);
    Image back;
    CHECK(rig.capture(back));
    CHECK(rig.stats.ssrDispatches > 0);
    ssrOff(rig);
}

void testOrthogonal(Rig& rig)
{
    printf("ssr with an orthogonal camera\n");
    rig.env = ambientOnly();
    rig.apply();
    rig.server->cameraSetOrthogonal(rig.camera, 9.0f, 0.1f, 100.0f);
    rig.lookFrom(kEye, kTarget);
    Image off;
    CHECK(rig.capture(off));
    ssrOn(rig);
    Image on;
    CHECK(rig.capture(on));
    CHECK(rig.stats.ssrDispatches > 0);
    CHECK(totalDifference(off, on) > 3000);
    dump(on, "ssr_ortho");
    ssrOff(rig);
    rig.server->cameraSetPerspective(rig.camera, kFov, 0.1f, 100.0f);
    rig.lookFrom(kEye, kTarget);
}

void showcase(Rig& rig, const char* tag)
{
    const char* directory = getenv("PRISMA3D_SHOTS");
    if (!directory || !directory[0]) return;
    rig.server->instanceSetTransform(rig.box,
            Math::Mat4::Translation(Math::Vec3(-1.2f, 0.0f, 3.2f)) *
                    Math::Mat4::Scale(Math::Vec3(1.6f, 1.6f, 1.6f)));
    rig.server->cameraSetTransform(rig.camera,
            Math::Mat4::LookAt(Math::Vec3(2.6f, 2.6f, 8.0f), Math::Vec3(-0.4f, 0.8f, 1.5f),
                    Math::Vec3(0.0f, 1.0f, 0.0f))
                    .InverseAffine());
    rig.resize(640, 480);
    rig.env = EnvironmentSettings();
    rig.env.background = BackgroundMode::Sky;
    rig.apply();
    rig.server->lightSetVisible(rig.sun, true);
    const float roughnesses[3] = { 0.0f, 0.12f, 0.3f };
    char name[96];
    Image off;
    for (int i = 0; i < 3; ++i)
    {
        setRoughness(rig, roughnesses[i]);
        ssrOff(rig);
        rig.capture(off);
        snprintf(name, sizeof(name), "showcase_off_r%d_%s", i, tag);
        dump(off, name);
        for (int half = 0; half < 2; ++half)
        {
            rig.server->setSsrHalfSize(half != 0);
            ssrOn(rig);
            Image on;
            rig.capture(on);
            snprintf(name, sizeof(name), "showcase_on_r%d_%s_%s", i, half ? "half" : "full", tag);
            dump(on, name);
        }
    }
    rig.server->setSsrHalfSize(true);
    setRoughness(rig, 0.05f);
    rig.server->cameraSetTransform(rig.camera,
            Math::Mat4::LookAt(Math::Vec3(5.0f, 1.1f, 7.5f), Math::Vec3(-1.0f, 0.9f, 0.0f),
                    Math::Vec3(0.0f, 1.0f, 0.0f))
                    .InverseAffine());
    ssrOff(rig);
    rig.capture(off);
    dump(off, tag[0] == 'o' ? "showcase_low_off_opengl" : "showcase_low_off_vulkan");
    ssrOn(rig);
    Image low;
    rig.capture(low);
    dump(low, tag[0] == 'o' ? "showcase_low_on_opengl" : "showcase_low_on_vulkan");
    setRoughness(rig, 0.0f);
    rig.server->lightSetVisible(rig.sun, false);
    ssrOff(rig);
}

void cost(Rig& rig)
{
    if (!getenv("PRISMA3D_SSR_COST")) return;
    RenderProfiler* profiler = Main::profiler();
    if (!profiler || !profiler->gpuAvailable()) return;
    rig.resize(1280, 720);
    const char* names[11] = { "Depth", "Opaque", "SSR downsample", "SSR copy depth", "SSR hi-z",
        "SSR main", "SSR filter", "SSR resolve", "SSR last frame", "Final", "Sky" };
    for (int mode = 0; mode < 3; ++mode)
    {
        const bool on = mode > 0;
        const bool half = mode == 1;
        rig.env = ambientOnly();
        rig.env.ssrEnabled = on;
        rig.apply();
        rig.server->setSsrHalfSize(half);
        for (int i = 0; i < 40; ++i) Main::iteration();
        float sum[11] = {};
        int frames = 0;
        for (int i = 0; i < 30; ++i)
        {
            Main::iteration();
            for (int n = 0; n < 11; ++n)
            {
                const ProfileSample* sample = gpuSample(names[n]);
                if (sample) sum[n] += sample->milliseconds;
            }
            ++frames;
        }
        if (on) printf("  ssr %s:", half ? "half" : "full");
        else
            printf("  ssr off  :");
        for (int n = 0; n < 11; ++n)
            printf(" %s %.3f", names[n], sum[n] / static_cast<float>(frames));
        printf("\n");
    }
    rig.server->setSsrHalfSize(true);
    ssrOff(rig);
}

} // namespace

int main(int argc, char** argv)
{
    const bool vulkan = argc > 1 && strcmp(argv[1], "vulkan") == 0;

    CHECK(Device::headlessSupported());
    if (!Device::headlessSupported()) return 1;

    DeviceDesc desc;
    desc.title = "ssr test";
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

    const MeshId floorMesh = meshes.create("ssr floor");
    CHECK(meshes.addSurface(floorMesh,
            quad(Math::Vec3(-8.0f, 0.0f, 10.0f), Math::Vec3(8.0f, 0.0f, 10.0f),
                    Math::Vec3(8.0f, 0.0f, -2.0f), Math::Vec3(-8.0f, 0.0f, -2.0f),
                    Math::Vec3(0.0f, 1.0f, 0.0f))));
    const MeshId boxMesh = meshes.create("ssr box");
    CHECK(meshes.addSurface(boxMesh, whiteBox()));
    rig.floor = server.instanceCreate(floorMesh, rig.scenario);
    rig.box = server.instanceCreate(boxMesh, rig.scenario);
    server.instanceSetTransform(rig.box,
            Math::Mat4::Translation(kBoxPosition) *
                    Math::Mat4::Scale(Math::Vec3(kBoxSize, kBoxSize, kBoxSize)));

    rig.mirror = rig.materials->create("ssr mirror");
    rig.materials->setColor(rig.mirror, MaterialParam::Albedo, Math::Vec4(1.0f, 1.0f, 1.0f, 1.0f));
    rig.materials->setFloat(rig.mirror, MaterialParam::Metallic, 1.0f);
    rig.materials->setFloat(rig.mirror, MaterialParam::Specular, 0.5f);
    rig.materials->setFloat(rig.mirror, MaterialParam::Roughness, 0.0f);
    const MaterialId red = rig.materials->create("ssr red");
    rig.materials->setColor(red, MaterialParam::Albedo, Math::Vec4(1.0f, 0.1f, 0.05f, 1.0f));
    rig.materials->setFloat(red, MaterialParam::Specular, 0.0f);
    rig.materials->setFloat(red, MaterialParam::Roughness, 1.0f);
    server.instanceSetMaterialOverride(rig.floor, rig.mirror);
    server.instanceSetMaterialOverride(rig.box, red);

    const MeshId wallMesh = meshes.create("ssr wall");
    CHECK(meshes.addSurface(wallMesh,
            quad(Math::Vec3(-8.0f, 0.0f, -2.0f), Math::Vec3(8.0f, 0.0f, -2.0f),
                    Math::Vec3(8.0f, 8.0f, -2.0f), Math::Vec3(-8.0f, 8.0f, -2.0f),
                    Math::Vec3(0.0f, 0.0f, 1.0f))));
    rig.wall = server.instanceCreate(wallMesh, rig.scenario);
    const MaterialId green = rig.materials->create("ssr green");
    rig.materials->setColor(green, MaterialParam::Albedo, Math::Vec4(0.15f, 0.55f, 0.2f, 1.0f));
    rig.materials->setFloat(green, MaterialParam::Specular, 0.0f);
    rig.materials->setFloat(green, MaterialParam::Roughness, 1.0f);
    server.instanceSetMaterialOverride(rig.wall, green);
    server.instanceSetVisible(rig.wall, false);

    rig.sun = server.lightCreate(LightType::Directional);
    server.lightSetScenario(rig.sun, rig.scenario);
    server.lightSetTransform(rig.sun,
            Math::Mat4::LookAt(Math::Vec3(0.0f, 0.0f, 0.0f), Math::Vec3(-0.3f, -1.0f, -0.4f),
                    Math::Vec3(0.0f, 1.0f, 0.0f))
                    .InverseAffine());
    server.lightSetVisible(rig.sun, false);

    rig.camera = server.cameraCreate();
    server.cameraSetPerspective(rig.camera, kFov, 0.1f, 100.0f);
    rig.lookFrom(kEye, kTarget);

    rig.view = server.viewportCreate();
    server.viewportSetSize(rig.view, kWidth, kHeight);
    server.viewportAttachCamera(rig.view, rig.camera);
    server.viewportSetScenario(rig.view, rig.scenario);
    server.viewportSetClearColor(rig.view, Math::Vec4(0.0f, 0.0f, 0.0f, 1.0f));
    server.viewportSetUpdateMode(rig.view, ViewportUpdateMode::Always);
    CHECK(server.viewportGetTexture(rig.view).valid());

    const char* tag = vulkan ? "vulkan" : "opengl";
    testDisabled(rig);
    testReflection(rig, tag);
    testRoughness(rig);
    testSettings(rig);
    testHalfSize(rig);
    testToggle(rig);
    testNoCompute(rig);
    testProfiler(rig);
    testResize(rig);
    testMoving(rig);
    testTogether(rig);
    testMultisampling(rig);
    testTransparentBackground(rig);
    testOrthogonal(rig);
    server.instanceSetVisible(rig.wall, true);
    showcase(rig, tag);
    server.instanceSetVisible(rig.wall, false);
    cost(rig);

    Main::device()->requestClose();
    tree.finalize();
    Main::cleanup();

    if (failures) printf("%d check(s) failed\n", failures);
    else
        printf("all checks passed\n");
    return failures ? 1 : 0;
}
