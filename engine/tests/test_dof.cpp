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
#include "prisma3d/scene/Camera3D.h"
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
const float kNearDepth = 3.0f;
const float kFarDepth = 20.0f;

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

struct Region
{
    int x0;
    int y0;
    int x1;
    int y1;
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
    InstanceId nearWall;
    InstanceId farWall;
    ct::Vector<InstanceId> extras;
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
};

SurfaceData barWall(float x0, float x1, float y0, float y1, float z, float bar,
        const float* centers, int count)
{
    SurfaceData data;
    const Math::Vec4 dark(0.02f, 0.02f, 0.02f, 1.0f);
    const Math::Vec4 white(1.0f, 1.0f, 1.0f, 1.0f);
    const Math::Vec3 normal(0.0f, 0.0f, 1.0f);
    float cursor = x0;
    for (int i = 0; i < count; ++i)
    {
        const float a = centers[i] - bar * 0.5f;
        const float b = centers[i] + bar * 0.5f;
        addBoxFace(data, Math::Vec3(cursor, y0, z), Math::Vec3(a, y0, z), Math::Vec3(a, y1, z),
                Math::Vec3(cursor, y1, z), normal, dark);
        addBoxFace(data, Math::Vec3(a, y0, z), Math::Vec3(b, y0, z), Math::Vec3(b, y1, z),
                Math::Vec3(a, y1, z), normal, white);
        cursor = b;
    }
    addBoxFace(data, Math::Vec3(cursor, y0, z), Math::Vec3(x1, y0, z), Math::Vec3(x1, y1, z),
            Math::Vec3(cursor, y1, z), normal, dark);
    return data;
}

float focalLength(const Rig& rig)
{
    return static_cast<float>(rig.height) * 0.5f / tanf(kFov * 0.5f * 3.14159265358979f / 180.0f);
}

Region wallRegion(const Rig& rig, float x0, float x1, float y0, float y1, float z)
{
    const float focal = focalLength(rig);
    Region region;
    region.x0 = static_cast<int>(ceilf(static_cast<float>(rig.width) * 0.5f + focal * x0 / -z));
    region.x1 = static_cast<int>(floorf(static_cast<float>(rig.width) * 0.5f + focal * x1 / -z));
    region.y0 = static_cast<int>(ceilf(static_cast<float>(rig.height) * 0.5f + focal * y0 / -z));
    region.y1 = static_cast<int>(floorf(static_cast<float>(rig.height) * 0.5f + focal * y1 / -z));
    return region;
}

Region nearRegion(const Rig& rig)
{
    return wallRegion(rig, -8.0f, -0.1f, -0.9f, 0.9f, -kNearDepth);
}

Region farRegion(const Rig& rig)
{
    return wallRegion(rig, 1.5f, 40.0f, -4.5f, 4.5f, -kFarDepth);
}

Region shrink(const Region& region, int by)
{
    Region inner = region;
    inner.x0 += by;
    inner.y0 += by;
    inner.x1 -= by;
    inner.y1 -= by;
    return inner;
}

float contrast(const Image& image, const Region& region)
{
    int steepest = 0;
    for (int y = region.y0; y <= region.y1; ++y)
    {
        for (int x = region.x0; x < region.x1; ++x)
        {
            if (x < 0 || y < 0 || x + 1 >= image.width || y >= image.height) continue;
            const int step = abs(image.channel(x + 1, y, 0) - image.channel(x, y, 0));
            if (step > steepest) steepest = step;
        }
    }
    return static_cast<float>(steepest);
}

int maxDifference(const Image& a, const Image& b, const Region& region)
{
    int worst = 0;
    for (int y = region.y0; y <= region.y1; ++y)
    {
        for (int x = region.x0; x <= region.x1; ++x)
        {
            if (x < 0 || y < 0 || x >= a.width || y >= a.height) continue;
            for (int c = 0; c < 3; ++c)
            {
                const int d = abs(a.channel(x, y, c) - b.channel(x, y, c));
                if (d > worst) worst = d;
            }
        }
    }
    return worst;
}

float meanDifference(const Image& a, const Image& b)
{
    double sum = 0.0;
    for (std::size_t i = 0; i < a.pixels.size(); ++i)
        if ((i & 3) != 3) sum += fabs(static_cast<double>(a.pixels[i]) - b.pixels[i]);
    return static_cast<float>(sum / (static_cast<double>(a.pixels.size()) * 0.75));
}

long totalDifference(const Image& a, const Image& b)
{
    long sum = 0;
    for (std::size_t i = 0; i < a.pixels.size(); ++i)
        sum += labs(static_cast<long>(a.pixels[i]) - static_cast<long>(b.pixels[i]));
    return sum;
}

bool opaque(const Image& image)
{
    for (int y = 0; y < image.height; ++y)
        for (int x = 0; x < image.width; ++x)
            if (image.channel(x, y, 3) != 255) return false;
    return true;
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

void farOnly(Rig& rig, float distance = 8.0f, float transition = 4.0f, float amount = 0.1f)
{
    rig.env = ambientOnly();
    CameraAttributesSettings& a = rig.env.cameraAttributes;
    a.dofBlurFarEnabled = true;
    a.dofBlurFarDistance = distance;
    a.dofBlurFarTransition = transition;
    a.dofBlurAmount = amount;
    rig.apply();
}

void nearOnly(Rig& rig, float distance = 5.0f, float transition = 3.0f, float amount = 0.1f)
{
    rig.env = ambientOnly();
    CameraAttributesSettings& a = rig.env.cameraAttributes;
    a.dofBlurNearEnabled = true;
    a.dofBlurNearDistance = distance;
    a.dofBlurNearTransition = transition;
    a.dofBlurAmount = amount;
    rig.apply();
}

void plain(Rig& rig)
{
    rig.env = ambientOnly();
    rig.apply();
}

const char* shapeName(DofBokehShape shape)
{
    return shape == DofBokehShape::Box       ? "box"
           : shape == DofBokehShape::Hexagon ? "hexagon"
                                             : "circle";
}

const char* qualityName(DofBlurQuality quality)
{
    return quality == DofBlurQuality::VeryLow  ? "very low"
           : quality == DofBlurQuality::Low    ? "low"
           : quality == DofBlurQuality::Medium ? "medium"
                                               : "high";
}

bool computePath(const Rig& rig)
{
    return rig.driver->caps().compute && rig.server->depthOfFieldComputeAllowed();
}

std::uint32_t expectedTextures(const Rig& rig)
{
    return computePath(rig) ? 3u : 7u;
}

std::uint32_t expectedPasses(DofBokehShape shape, DofBlurQuality quality)
{
    if (shape == DofBokehShape::Circle) return 3;
    return quality == DofBlurQuality::VeryLow || quality == DofBlurQuality::Low ? 4 : 3;
}

void testDisabled(Rig& rig, bool first)
{
    printf("dof off: no pass, no buffer, the frame does not change\n");
    plain(rig);
    Image reference;
    CHECK(rig.capture(reference));
    CHECK(rig.stats.dofDispatches == 0 && rig.stats.dofDraws == 0);
    CHECK(rig.stats.dofTexturesLive == 0 && rig.stats.dofTargetsCreated == 0);
    CHECK(!first || !passRegistered("DOF"));
    CHECK(opaque(reference));
    const float nearBase = contrast(reference, shrink(nearRegion(rig), 9));
    const float farBase = contrast(reference, shrink(farRegion(rig), 9));
    printf("  steepest edge of the bars: near %.1f far %.1f\n", nearBase, farBase);
    CHECK(nearBase > 20.0f);
    CHECK(farBase > 20.0f);

    EnvironmentSettings changed = ambientOnly();
    changed.cameraAttributes.dofBlurFarEnabled = false;
    changed.cameraAttributes.dofBlurNearEnabled = false;
    changed.cameraAttributes.dofBlurFarDistance = 1.0f;
    changed.cameraAttributes.dofBlurFarTransition = 30.0f;
    changed.cameraAttributes.dofBlurNearDistance = 30.0f;
    changed.cameraAttributes.dofBlurNearTransition = 30.0f;
    changed.cameraAttributes.dofBlurAmount = 1.0f;
    rig.env = changed;
    rig.apply();
    Image other;
    CHECK(rig.capture(other));
    CHECK(other.same(reference));
    CHECK(rig.stats.dofDispatches == 0 && rig.stats.dofDraws == 0);
    CHECK(rig.stats.dofTexturesLive == 0);
    CHECK(!first || !passRegistered("DOF"));

    farOnly(rig, 8.0f, 4.0f, 0.0f);
    CHECK(rig.capture(other));
    CHECK(other.same(reference));
    CHECK(rig.stats.dofTexturesLive == 0 && rig.stats.dofDispatches == 0);

    plain(rig);
    rig.server->cameraClearAttributes(rig.camera);
    CHECK(rig.capture(other));
    CHECK(other.same(reference));
}

void testFarAndNear(Rig& rig, const char* tag)
{
    printf("far blur blurs the far wall and leaves the near one, near blur the opposite\n");
    plain(rig);
    Image off;
    CHECK(rig.capture(off));
    const Region nearWall = shrink(nearRegion(rig), 9);
    const Region farWall = shrink(farRegion(rig), 9);
    const float nearBase = contrast(off, nearWall);
    const float farBase = contrast(off, farWall);

    farOnly(rig);
    Image far;
    CHECK(rig.capture(far));
    CHECK(rig.stats.dofDispatches + rig.stats.dofDraws > 0);
    CHECK(rig.stats.dofTargetsCreated <= 1);
    CHECK(rig.stats.dofTexturesLive == expectedTextures(rig));
    CHECK(passRegistered("DOF"));
    CHECK(opaque(far));
    const float farBlurred = contrast(far, farWall);
    const float nearKept = contrast(far, nearWall);
    printf("  far blur: far wall %.1f -> %.1f, near wall %.1f -> %.1f (max change %d)\n", farBase,
            farBlurred, nearBase, nearKept, maxDifference(off, far, nearWall));
    CHECK(farBlurred < farBase * 0.5f);
    CHECK(maxDifference(off, far, nearWall) <= 2);
    CHECK(fabsf(nearKept - nearBase) < 1.0f);

    nearOnly(rig);
    Image nearBlur;
    CHECK(rig.capture(nearBlur));
    CHECK(rig.stats.dofDispatches + rig.stats.dofDraws > 0);
    CHECK(opaque(nearBlur));
    const float nearBlurred = contrast(nearBlur, nearWall);
    const float farKept = contrast(nearBlur, farWall);
    printf("  near blur: near wall %.1f -> %.1f, far wall %.1f -> %.1f (max change %d)\n", nearBase,
            nearBlurred, farBase, farKept, maxDifference(off, nearBlur, farWall));
    CHECK(nearBlurred < nearBase * 0.5f);
    CHECK(maxDifference(off, nearBlur, farWall) <= 2);
    CHECK(fabsf(farKept - farBase) < 1.0f);

    rig.env = ambientOnly();
    rig.env.cameraAttributes.dofBlurFarEnabled = true;
    rig.env.cameraAttributes.dofBlurNearEnabled = true;
    rig.env.cameraAttributes.dofBlurFarDistance = 8.0f;
    rig.env.cameraAttributes.dofBlurNearDistance = 5.0f;
    rig.env.cameraAttributes.dofBlurNearTransition = 3.0f;
    rig.apply();
    Image both;
    CHECK(rig.capture(both));
    CHECK(contrast(both, nearWall) < nearBase * 0.5f);
    CHECK(contrast(both, farWall) < farBase * 0.5f);

    char name[64];
    snprintf(name, sizeof(name), "dof_off_%s", tag);
    dump(off, name);
    snprintf(name, sizeof(name), "dof_far_%s", tag);
    dump(far, name);
    snprintf(name, sizeof(name), "dof_near_%s", tag);
    dump(nearBlur, name);
    snprintf(name, sizeof(name), "dof_both_%s", tag);
    dump(both, name);
    plain(rig);
}

void testMonotonic(Rig& rig)
{
    printf("amount, distance and transition move the blur one way\n");
    const Region farWall = shrink(farRegion(rig), 9);
    Image image;

    const float amounts[5] = { 0.01f, 0.03f, 0.06f, 0.1f, 0.3f };
    float previous = 1e9f;
    for (float amount: amounts)
    {
        farOnly(rig, 8.0f, 4.0f, amount);
        CHECK(rig.capture(image));
        const float value = contrast(image, farWall);
        printf("  amount %.2f: contrast %.1f\n", amount, value);
        CHECK(value <= previous + 2.0f);
        previous = value;
    }
    farOnly(rig, 8.0f, 4.0f, 0.01f);
    CHECK(rig.capture(image));
    const float weak = contrast(image, farWall);
    farOnly(rig, 8.0f, 4.0f, 0.3f);
    CHECK(rig.capture(image));
    CHECK(weak > contrast(image, farWall) + 8.0f);

    const float distances[5] = { 2.0f, 8.0f, 14.0f, 18.0f, 24.0f };
    float last = -1.0f;
    for (float distance: distances)
    {
        farOnly(rig, distance, 4.0f, 0.1f);
        CHECK(rig.capture(image));
        const float value = contrast(image, farWall);
        printf("  far distance %.0f: contrast %.1f\n", distance, value);
        CHECK(value >= last - 2.0f);
        last = value;
    }
    plain(rig);
    Image off;
    CHECK(rig.capture(off));
    CHECK(maxDifference(image, off, farWall) <= 1);

    const float transitions[5] = { 0.5f, 4.0f, 8.0f, 14.0f, 40.0f };
    last = -1.0f;
    for (float transition: transitions)
    {
        farOnly(rig, 8.0f, transition, 0.1f);
        CHECK(rig.capture(image));
        const float value = contrast(image, farWall);
        printf("  far transition %.1f: contrast %.1f\n", transition, value);
        CHECK(value >= last - 2.0f);
        last = value;
    }

    const Region nearWall = shrink(nearRegion(rig), 9);
    const float nearDistances[5] = { 1.0f, 3.0f, 4.0f, 6.0f, 12.0f };
    previous = 1e9f;
    for (float distance: nearDistances)
    {
        nearOnly(rig, distance, 3.0f, 0.1f);
        CHECK(rig.capture(image));
        const float value = contrast(image, nearWall);
        printf("  near distance %.0f: contrast %.1f\n", distance, value);
        CHECK(value <= previous + 2.0f);
        previous = value;
    }

    farOnly(rig, 8.0f, -1.0f, 0.1f);
    CHECK(rig.capture(image));
    plain(rig);
    CHECK(rig.capture(off));
    printf("  physical transition: far contrast %.1f against %.1f\n", contrast(image, farWall),
            contrast(off, farWall));
    if (computePath(rig)) CHECK(contrast(image, farWall) < contrast(off, farWall) - 3.0f);
    else
        CHECK(contrast(image, farWall) > contrast(off, farWall) - 3.0f);
    plain(rig);
}

void testShapes(Rig& rig)
{
    printf("every bokeh shape and quality runs and blurs\n");
    plain(rig);
    Image off;
    CHECK(rig.capture(off));
    const Region nearWall = shrink(nearRegion(rig), 9);
    const Region farWall = shrink(farRegion(rig), 9);
    const float farBase = contrast(off, farWall);
    const DofBokehShape shapes[3] = { DofBokehShape::Box, DofBokehShape::Hexagon,
        DofBokehShape::Circle };
    const DofBlurQuality qualities[4] = { DofBlurQuality::VeryLow, DofBlurQuality::Low,
        DofBlurQuality::Medium, DofBlurQuality::High };
    farOnly(rig);
    for (DofBokehShape shape: shapes)
    {
        for (DofBlurQuality quality: qualities)
        {
            rig.server->setDepthOfFieldBokeh(shape, quality, false);
            Image image;
            CHECK(rig.capture(image));
            const std::uint32_t passes = rig.stats.dofDispatches + rig.stats.dofDraws;
            const float blurred = contrast(image, farWall);
            printf("  %-7s %-8s: %u passes, far contrast %.1f\n", shapeName(shape),
                    qualityName(quality), passes, blurred);
            CHECK(passes == expectedPasses(shape, quality));
            const bool noSamples =
                    shape == DofBokehShape::Circle && quality == DofBlurQuality::VeryLow;
            if (!noSamples)
            {
                CHECK(blurred < farBase * 0.6f);
                CHECK(!image.same(off));
            }
            const bool softensNear = !computePath(rig) && quality >= DofBlurQuality::Medium &&
                                     shape != DofBokehShape::Circle;
            if (softensNear)
            {
                printf("    near wall softened to %.1f of %.1f\n", contrast(image, nearWall),
                        contrast(off, nearWall));
                CHECK(contrast(image, nearWall) > contrast(off, nearWall) * 0.35f);
            }
            else
                CHECK(maxDifference(off, image, nearWall) <= 2);
            CHECK(opaque(image));
        }
    }
    rig.server->setDepthOfFieldBokeh(DofBokehShape::Circle, DofBlurQuality::VeryLow, false);
    farOnly(rig, 8.0f, 4.0f, 0.3f);
    Image strong;
    CHECK(rig.capture(strong));
    printf("  circle very low only samples above 8 pixels: amount 0.3 gives contrast %.1f\n",
            contrast(strong, farWall));
    CHECK(contrast(strong, farWall) < farBase * 0.6f);
    rig.server->setDepthOfFieldBokeh(DofBokehShape::Hexagon, DofBlurQuality::Low, false);
    plain(rig);
}

void testJitter(Rig& rig)
{
    printf("jitter changes the bokeh from frame to frame in the compute path only\n");
    farOnly(rig);
    rig.server->setDepthOfFieldBokeh(DofBokehShape::Box, DofBlurQuality::Medium, false);
    Image a;
    Image b;
    CHECK(rig.capture(a));
    CHECK(rig.capture(b));
    CHECK(a.same(b));
    rig.server->setDepthOfFieldBokeh(DofBokehShape::Box, DofBlurQuality::Medium, true);
    CHECK(rig.capture(a));
    CHECK(rig.capture(b));
    const bool compute = computePath(rig);
    printf("  %s: two frames with jitter differ by %ld\n", compute ? "compute" : "raster",
            totalDifference(a, b));
    if (compute) CHECK(!a.same(b));
    else
        CHECK(a.same(b));
    plain(rig);
    Image off;
    CHECK(rig.capture(off));
    CHECK(contrast(a, shrink(farRegion(rig), 9)) < contrast(off, shrink(farRegion(rig), 9)) * 0.5f);
    rig.server->setDepthOfFieldBokeh(DofBokehShape::Hexagon, DofBlurQuality::Low, false);
    plain(rig);
}

void testPaths(Rig& rig)
{
    printf("the compute path and the raster path both blur and give a similar image\n");
    plain(rig);
    Image off;
    CHECK(rig.capture(off));
    const Region farWall = shrink(farRegion(rig), 9);
    const Region nearWall = shrink(nearRegion(rig), 9);
    const DofBokehShape shapes[3] = { DofBokehShape::Box, DofBokehShape::Hexagon,
        DofBokehShape::Circle };
    const DofBlurQuality qualities[2] = { DofBlurQuality::Low, DofBlurQuality::High };
    for (int mode = 0; mode < 3; ++mode)
    {
        if (mode == 0) farOnly(rig);
        else if (mode == 1)
            nearOnly(rig);
        else
        {
            farOnly(rig);
            rig.env.cameraAttributes.dofBlurNearEnabled = true;
            rig.env.cameraAttributes.dofBlurNearDistance = 5.0f;
            rig.env.cameraAttributes.dofBlurNearTransition = 3.0f;
            rig.apply();
        }
        for (DofBokehShape shape: shapes)
        {
            for (DofBlurQuality quality: qualities)
            {
                rig.server->setDepthOfFieldBokeh(shape, quality, false);
                rig.server->setDepthOfFieldComputeAllowed(true);
                Image compute;
                CHECK(rig.capture(compute));
                const std::uint32_t dispatches = rig.stats.dofDispatches;
                const std::uint32_t computeDraws = rig.stats.dofDraws;
                const std::uint32_t computeTextures = rig.stats.dofTexturesLive;
                rig.server->setDepthOfFieldComputeAllowed(false);
                Image raster;
                CHECK(rig.capture(raster));
                const std::uint32_t draws = rig.stats.dofDraws;
                const std::uint32_t rasterDispatches = rig.stats.dofDispatches;
                const std::uint32_t rasterTextures = rig.stats.dofTexturesLive;
                rig.server->setDepthOfFieldComputeAllowed(true);
                const float difference = meanDifference(compute, raster);
                printf("  mode %d %-7s %-8s: compute %u dispatches %u textures, raster %u draws "
                       "%u textures, mean difference %.3f\n",
                        mode, shapeName(shape), qualityName(quality), dispatches, computeTextures,
                        draws, rasterTextures, difference);
                CHECK(dispatches == expectedPasses(shape, quality) && computeDraws == 0);
                CHECK(draws == expectedPasses(shape, quality) && rasterDispatches == 0);
                CHECK(computeTextures == 3 && rasterTextures == 7);
                CHECK(!compute.same(off) && !raster.same(off));
                CHECK(opaque(compute) && opaque(raster));
                CHECK(difference < 12.0f);
                if (mode != 1)
                {
                    CHECK(contrast(compute, farWall) < contrast(off, farWall) * 0.6f);
                    CHECK(contrast(raster, farWall) < contrast(off, farWall) * 0.6f);
                }
                if (mode != 0)
                {
                    CHECK(contrast(compute, nearWall) < contrast(off, nearWall) * 0.6f);
                    CHECK(contrast(raster, nearWall) < contrast(off, nearWall) * 0.6f);
                }
            }
        }
    }
    rig.server->setDepthOfFieldBokeh(DofBokehShape::Hexagon, DofBlurQuality::Low, false);
    rig.server->setDepthOfFieldComputeAllowed(true);
    plain(rig);
}

void testToggle(Rig& rig)
{
    printf("turning dof off frees the buffers and gives back the first image\n");
    plain(rig);
    Image off;
    CHECK(rig.capture(off));
    farOnly(rig);
    Image on;
    CHECK(rig.capture(on));
    CHECK(rig.stats.dofTexturesLive > 0);
    CHECK(!on.same(off));
    plain(rig);
    Image again;
    CHECK(rig.capture(again));
    CHECK(again.same(off));
    CHECK(rig.stats.dofTexturesLive == 0);
    CHECK(rig.stats.dofDispatches == 0 && rig.stats.dofDraws == 0);
    farOnly(rig);
    Image back;
    CHECK(rig.capture(back));
    CHECK(back.same(on));
    const bool allowed = rig.server->depthOfFieldComputeAllowed();
    rig.server->setDepthOfFieldComputeAllowed(!allowed);
    Image other;
    CHECK(rig.capture(other));
    CHECK(rig.stats.dofDispatches + rig.stats.dofDraws > 0);
    CHECK(rig.stats.dofTexturesLive == expectedTextures(rig));
    rig.server->setDepthOfFieldComputeAllowed(allowed);
    Image restored;
    CHECK(rig.capture(restored));
    CHECK(restored.same(on));
    CHECK(rig.stats.dofTexturesLive == expectedTextures(rig));
    plain(rig);
    CHECK(rig.capture(again));
    CHECK(again.same(off));
}

void testProfiler(Rig& rig)
{
    printf("the dof passes show in the profiler only while they run\n");
    RenderProfiler* profiler = Main::profiler();
    CHECK(profiler != nullptr);
    if (!profiler) return;
    farOnly(rig);
    Image on;
    CHECK(rig.capture(on, 6));
    CHECK(passRegistered("DOF"));
    CHECK(profiler->passEnabled("DOF"));
    const char* names[4] = { "DOF blur size", "DOF bokeh 1", "DOF bokeh 2", "DOF composite" };
    if (profiler->gpuAvailable())
    {
        for (int i = 0; i < 60; ++i)
        {
            const ProfileSample* sample = gpuSample("DOF composite");
            if (sample && sample->historyCount >= 2) break;
            Main::iteration();
        }
        for (const char* name: names)
        {
            const ProfileSample* sample = gpuSample(name);
            CHECK(sample != nullptr);
            if (sample)
            {
                printf("  gpu %-14s %.4f ms\n", name, sample->milliseconds);
                CHECK(sample->milliseconds >= 0.0f);
            }
        }
    }
    else
        printf("  gpu timer queries are not available on this driver\n");

    profiler->setPassEnabled("DOF", false);
    Image disabled;
    CHECK(rig.capture(disabled));
    CHECK(rig.stats.dofDispatches == 0 && rig.stats.dofDraws == 0);
    CHECK(rig.stats.dofTexturesLive == 0);
    plain(rig);
    Image off;
    CHECK(rig.capture(off));
    farOnly(rig);
    CHECK(rig.capture(disabled));
    CHECK(disabled.same(off));
    profiler->setPassEnabled("DOF", true);
    Image again;
    CHECK(rig.capture(again, 6));
    CHECK(again.same(on));
    CHECK(rig.stats.dofDispatches > 0);
    plain(rig);
}

void testResize(Rig& rig)
{
    printf("dof follows the size of the viewport\n");
    farOnly(rig);
    const int sizes[5][2] = { { 97, 61 }, { 160, 120 }, { 33, 17 }, { 8, 8 }, { 160, 120 } };
    Image first;
    for (int i = 0; i < 5; ++i)
    {
        rig.resize(sizes[i][0], sizes[i][1]);
        Image image;
        CHECK(rig.capture(image));
        CHECK(rig.stats.dofDispatches + rig.stats.dofDraws > 0);
        CHECK(rig.stats.dofTexturesLive == expectedTextures(rig));
        if (i == 1) first = image;
        if (i == 4) CHECK(image.same(first));
    }
    rig.server->setDepthOfFieldComputeAllowed(false);
    for (int i = 0; i < 4; ++i)
    {
        rig.resize(sizes[i][0], sizes[i][1]);
        Image image;
        CHECK(rig.capture(image));
        CHECK(rig.stats.dofDraws > 0);
        CHECK(rig.stats.dofTexturesLive == 7);
    }
    rig.server->setDepthOfFieldComputeAllowed(true);
    rig.resize(4, 4);
    Image tiny;
    CHECK(rig.capture(tiny));
    CHECK(rig.stats.dofDispatches == 0 && rig.stats.dofDraws == 0);
    CHECK(rig.stats.dofTexturesLive == 0);
    rig.resize(kWidth, kHeight);
    plain(rig);
}

void testTogether(Rig& rig)
{
    printf("dof together with ssao and ssr\n");
    plain(rig);
    Image off;
    CHECK(rig.capture(off));
    const Region farWall = shrink(farRegion(rig), 9);
    const Region nearWall = shrink(nearRegion(rig), 9);
    farOnly(rig);
    rig.env.ssaoEnabled = true;
    rig.env.ssrEnabled = true;
    rig.apply();
    Image all;
    CHECK(rig.capture(all, 4));
    CHECK(rig.stats.dofDispatches + rig.stats.dofDraws > 0);
    if (rig.server->screenSpaceEffectsSupported())
    {
        CHECK(rig.stats.ssaoDispatches > 0);
        CHECK(rig.stats.ssrDispatches > 0);
        CHECK(rig.stats.normalRoughnessTexturesLive == 1);
    }
    CHECK(contrast(all, farWall) < contrast(off, farWall) * 0.5f);
    CHECK(contrast(all, nearWall) > contrast(off, nearWall) * 0.8f);
    dump(all, rig.server->depthOfFieldComputeAllowed() ? "dof_ssao_ssr" : "dof_ssao_ssr_raster");
    rig.server->setDepthOfFieldComputeAllowed(false);
    Image raster;
    CHECK(rig.capture(raster, 4));
    CHECK(rig.stats.dofDraws > 0);
    CHECK(contrast(raster, farWall) < contrast(off, farWall) * 0.5f);
    rig.server->setDepthOfFieldComputeAllowed(true);
    plain(rig);
}

void testCameraAttributes(Rig& rig)
{
    printf("the attributes of the camera replace the ones of the environment\n");
    plain(rig);
    Image off;
    CHECK(rig.capture(off));
    const Region farWall = shrink(farRegion(rig), 9);
    const Region nearWall = shrink(nearRegion(rig), 9);

    farOnly(rig);
    Image world;
    CHECK(rig.capture(world));
    CHECK(contrast(world, farWall) < contrast(off, farWall) * 0.5f);
    CHECK(contrast(world, nearWall) > contrast(off, nearWall) * 0.95f);

    CameraAttributesSettings camera;
    camera.dofBlurNearEnabled = true;
    camera.dofBlurNearDistance = 5.0f;
    camera.dofBlurNearTransition = 3.0f;
    camera.dofBlurAmount = 0.1f;
    rig.server->cameraSetAttributes(rig.camera, camera);
    Image overridden;
    CHECK(rig.capture(overridden));
    CHECK(contrast(overridden, nearWall) < contrast(off, nearWall) * 0.5f);
    CHECK(contrast(overridden, farWall) > contrast(off, farWall) * 0.95f);

    plain(rig);
    Image cameraOnly;
    CHECK(rig.capture(cameraOnly));
    CHECK(cameraOnly.same(overridden));
    CHECK(rig.stats.dofDispatches > 0);

    CameraAttributesSettings none;
    rig.server->cameraSetAttributes(rig.camera, none);
    Image disabled;
    CHECK(rig.capture(disabled));
    CHECK(disabled.same(off));
    CHECK(rig.stats.dofTexturesLive == 0);

    farOnly(rig);
    CHECK(rig.capture(disabled));
    CHECK(disabled.same(off));

    rig.server->cameraClearAttributes(rig.camera);
    Image back;
    CHECK(rig.capture(back));
    CHECK(back.same(world));
    plain(rig);
}

void testMultisampling(Rig& rig)
{
    printf("dof works with multisampling\n");
    farOnly(rig);
    rig.server->viewportSetMsaa3D(rig.view, ViewportMsaa::Msaa4x);
    Image image;
    CHECK(rig.capture(image));
    const Region farWall = shrink(farRegion(rig), 9);
    const Region nearWall = shrink(nearRegion(rig), 9);
    plain(rig);
    Image off;
    CHECK(rig.capture(off));
    farOnly(rig);
    CHECK(rig.capture(image));
    if (rig.server->viewportGetMsaaSamples(rig.view) > 1)
    {
        CHECK(rig.stats.dofDispatches > 0);
        CHECK(contrast(image, farWall) < contrast(off, farWall) * 0.6f);
        CHECK(contrast(image, nearWall) > contrast(off, nearWall) * 0.9f);
        rig.server->setDepthOfFieldComputeAllowed(false);
        Image raster;
        CHECK(rig.capture(raster));
        CHECK(rig.stats.dofDraws > 0);
        CHECK(contrast(raster, farWall) < contrast(off, farWall) * 0.6f);
        rig.server->setDepthOfFieldComputeAllowed(true);
    }
    rig.server->viewportSetMsaa3D(rig.view, ViewportMsaa::Disabled);
    Image back;
    CHECK(rig.capture(back));
    CHECK(contrast(back, farWall) < contrast(off, farWall) * 0.6f);
    plain(rig);
}

void testTransparentBackground(Rig& rig)
{
    printf("dof is skipped in a viewport with a transparent background\n");
    farOnly(rig);
    rig.server->viewportSetTransparentBackground(rig.view, true);
    Image image;
    CHECK(rig.capture(image));
    CHECK(rig.stats.dofDispatches == 0 && rig.stats.dofDraws == 0);
    CHECK(rig.stats.dofTexturesLive == 0);
    rig.server->viewportSetTransparentBackground(rig.view, false);
    Image back;
    CHECK(rig.capture(back));
    CHECK(rig.stats.dofDispatches > 0);
    plain(rig);
}

void testOrthogonal(Rig& rig)
{
    printf("dof with an orthogonal camera\n");
    rig.server->cameraSetOrthogonal(rig.camera, 30.0f, 0.1f, 100.0f);
    plain(rig);
    Image off;
    CHECK(rig.capture(off));
    farOnly(rig, 5.0f, 5.0f, 0.2f);
    Image on;
    CHECK(rig.capture(on));
    CHECK(rig.stats.dofDispatches > 0);
    CHECK(totalDifference(off, on) > 3000);
    dump(on, "dof_ortho");
    rig.server->cameraSetPerspective(rig.camera, kFov, 0.1f, 100.0f);
    plain(rig);
}

void testCustomProjection(Rig& rig)
{
    printf("dof takes the planes from a custom projection\n");
    plain(rig);
    Image off;
    CHECK(rig.capture(off));
    farOnly(rig);
    Image normal;
    CHECK(rig.capture(normal));
    const float aspect = static_cast<float>(rig.width) / static_cast<float>(rig.height);
    rig.server->cameraSetCustomProjection(rig.camera,
            Math::Mat4::Perspective(kFov * 3.14159265358979f / 180.0f, aspect, 0.1f, 100.0f));
    Image custom;
    CHECK(rig.capture(custom));
    CHECK(rig.stats.dofDispatches > 0);
    CHECK(meanDifference(custom, normal) < 1.0f);
    rig.server->cameraSetPerspective(rig.camera, kFov, 0.1f, 100.0f);
    plain(rig);
}

void testAlpha(Rig& rig)
{
    printf("dof gives the viewport texture the alpha of the clear colour\n");
    farOnly(rig);
    for (int mode = 0; mode < 2; ++mode)
    {
        rig.server->setDepthOfFieldComputeAllowed(mode == 0);
        Image on;
        CHECK(rig.capture(on));
        CHECK(rig.stats.dofDispatches + rig.stats.dofDraws > 0);
        CHECK(opaque(on));
    }
    rig.server->viewportSetClearColor(rig.view, Math::Vec4(0.0f, 0.0f, 0.0f, 0.5f));
    for (int mode = 0; mode < 2; ++mode)
    {
        rig.server->setDepthOfFieldComputeAllowed(mode == 0);
        Image on;
        CHECK(rig.capture(on));
        bool uniform = true;
        for (int y = 0; y < on.height; ++y)
            for (int x = 0; x < on.width; ++x)
                if (abs(on.channel(x, y, 3) - 128) > 1) uniform = false;
        CHECK(uniform);
    }
    rig.server->setDepthOfFieldComputeAllowed(true);
    rig.server->viewportSetClearColor(rig.view, Math::Vec4(0.0f, 0.0f, 0.0f, 1.0f));
    plain(rig);
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

void testNodes(Rig& rig, SceneTree& tree, MeshId nearMesh, MeshId farMesh)
{
    printf("the WorldEnvironment and the Camera3D nodes drive the depth of field of the window\n");
    plain(rig);
    Image idle;
    CHECK(rig.capture(idle));
    CHECK(rig.stats.dofTexturesLive == 0);
    rig.server->viewportSetUpdateMode(rig.view, ViewportUpdateMode::Disabled);
    Node* root = tree.getRoot();
    CHECK(root != nullptr);
    if (!root) return;
    WorldEnvironment* world = root->createChild<WorldEnvironment>();
    world->setEnvironment(ambientOnly());
    Camera3D* camera = root->createChild<Camera3D>();
    camera->setPerspective(kFov, 0.1f, 100.0f);
    camera->makeCurrent();
    MeshInstance3D* nearWall = root->createChild<MeshInstance3D>();
    nearWall->setMesh(nearMesh);
    MeshInstance3D* farWall = root->createChild<MeshInstance3D>();
    farWall->setMesh(farMesh);

    const Region nearRegion = shrink(wallRegion(rig, -8.0f, -0.1f, -0.9f, 0.9f, -kNearDepth), 9);
    const Region farRegion = shrink(wallRegion(rig, 1.5f, 40.0f, -4.5f, 4.5f, -kFarDepth), 9);
    Image off;
    CHECK(captureWindow(tree, off));
    CHECK(contrast(off, nearRegion) > 150.0f && contrast(off, farRegion) > 150.0f);
    CHECK(Main::rendering()->getStats().dofDispatches + Main::rendering()->getStats().dofDraws ==
            0);

    CHECK(world->set("dof_blur_far_enabled", PropertyValue::fromBool(true)));
    CHECK(world->set("dof_blur_far_distance", PropertyValue::fromSingle(8.0f)));
    CHECK(world->set("dof_blur_far_transition", PropertyValue::fromSingle(4.0f)));
    Image far;
    CHECK(captureWindow(tree, far));
    const RenderStats stats = Main::rendering()->getStats();
    CHECK(stats.dofDispatches + stats.dofDraws > 0);
    CHECK(contrast(far, farRegion) < contrast(off, farRegion) * 0.5f);
    CHECK(contrast(far, nearRegion) > contrast(off, nearRegion) * 0.95f);

    CHECK(camera->set("attributes_dof_blur_near_enabled", PropertyValue::fromBool(true)));
    CHECK(camera->set("attributes_dof_blur_near_distance", PropertyValue::fromSingle(5.0f)));
    CHECK(camera->set("attributes_dof_blur_near_transition", PropertyValue::fromSingle(3.0f)));
    Image stillWorld;
    CHECK(captureWindow(tree, stillWorld));
    CHECK(stillWorld.same(far));

    CHECK(camera->set("attributes_override", PropertyValue::fromBool(true)));
    Image overridden;
    CHECK(captureWindow(tree, overridden));
    CHECK(contrast(overridden, nearRegion) < contrast(off, nearRegion) * 0.5f);
    CHECK(contrast(overridden, farRegion) > contrast(off, farRegion) * 0.95f);

    CHECK(camera->set("attributes_override", PropertyValue::fromBool(false)));
    Image back;
    CHECK(captureWindow(tree, back));
    CHECK(back.same(far));

    CHECK(world->set("dof_blur_far_enabled", PropertyValue::fromBool(false)));
    Image again;
    CHECK(captureWindow(tree, again));
    CHECK(again.same(off));
    CHECK(Main::rendering()->getStats().dofTexturesLive == 0);

    nearWall->queueFree();
    farWall->queueFree();
    camera->queueFree();
    world->queueFree();
    tree.process(1.0 / 60.0);
    rig.server->viewportSetUpdateMode(rig.view, ViewportUpdateMode::Always);
}

void showcase(Rig& rig, const char* tag)
{
    const char* directory = getenv("PRISMA3D_SHOTS");
    if (!directory || !directory[0]) return;
    rig.resize(640, 480);
    rig.server->cameraSetTransform(rig.camera,
            Math::Mat4::LookAt(Math::Vec3(0.0f, -0.5f, 1.5f), Math::Vec3(0.0f, -0.55f, -10.0f),
                    Math::Vec3(0.0f, 1.0f, 0.0f))
                    .InverseAffine());
    for (InstanceId id: rig.extras) rig.server->instanceSetVisible(id, true);
    rig.server->instanceSetVisible(rig.nearWall, false);
    rig.server->instanceSetVisible(rig.farWall, false);
    const char* kinds[4] = { "off", "far", "near", "both" };
    char name[128];
    for (int mode = 0; mode < 2; ++mode)
    {
        rig.server->setDepthOfFieldComputeAllowed(mode == 0);
        for (int kind = 0; kind < 4; ++kind)
        {
            rig.env = ambientOnly();
            rig.env.background = BackgroundMode::Sky;
            CameraAttributesSettings& a = rig.env.cameraAttributes;
            a.dofBlurFarEnabled = kind == 1 || kind == 3;
            a.dofBlurNearEnabled = kind == 2 || kind == 3;
            a.dofBlurFarDistance = 9.0f;
            a.dofBlurFarTransition = 7.0f;
            a.dofBlurNearDistance = 5.0f;
            a.dofBlurNearTransition = 3.5f;
            a.dofBlurAmount = 0.2f;
            rig.apply();
            Image image;
            rig.capture(image);
            snprintf(name, sizeof(name), "showcase_%s_%s_%s", kinds[kind],
                    mode == 0 ? "compute" : "raster", tag);
            dump(image, name);
        }
    }
    const DofBokehShape shapes[3] = { DofBokehShape::Box, DofBokehShape::Hexagon,
        DofBokehShape::Circle };
    rig.server->setDepthOfFieldComputeAllowed(true);
    for (DofBokehShape shape: shapes)
    {
        rig.server->setDepthOfFieldBokeh(shape, DofBlurQuality::Medium, false);
        Image image;
        rig.capture(image);
        snprintf(name, sizeof(name), "showcase_shape_%s_%s", shapeName(shape), tag);
        dump(image, name);
    }
    rig.server->setDepthOfFieldBokeh(DofBokehShape::Hexagon, DofBlurQuality::Low, false);
    for (InstanceId id: rig.extras) rig.server->instanceSetVisible(id, false);
    rig.server->instanceSetVisible(rig.nearWall, true);
    rig.server->instanceSetVisible(rig.farWall, true);
    rig.server->cameraSetTransform(rig.camera, Math::Mat4::Identity());
    rig.resize(kWidth, kHeight);
    plain(rig);
}

void cost(Rig& rig)
{
    if (!getenv("PRISMA3D_DOF_COST")) return;
    RenderProfiler* profiler = Main::profiler();
    if (!profiler || !profiler->gpuAvailable()) return;
    rig.resize(1280, 720);
    const char* names[8] = { "Opaque", "Sky", "Final", "DOF blur size", "DOF bokeh 1",
        "DOF bokeh 2", "DOF composite", "Transparent" };
    const DofBokehShape shapes[3] = { DofBokehShape::Hexagon, DofBokehShape::Box,
        DofBokehShape::Circle };
    const DofBlurQuality qualities[2] = { DofBlurQuality::Low, DofBlurQuality::Medium };
    for (int mode = 0; mode < 3; ++mode)
    {
        const bool on = mode > 0;
        const bool compute = mode == 1;
        for (int variant = 0; variant < (on ? 6 : 1); ++variant)
        {
            const DofBokehShape shape = shapes[variant / 2];
            const DofBlurQuality quality = qualities[variant % 2];
            if (on) farOnly(rig);
            else
                plain(rig);
            rig.server->setDepthOfFieldBokeh(shape, quality, false);
            rig.server->setDepthOfFieldComputeAllowed(compute);
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
            if (on)
                printf("  dof %-7s %-7s %-6s:", compute ? "compute" : "raster", shapeName(shape),
                        qualityName(quality));
            else
                printf("  dof off                      :");
            for (int n = 0; n < 8; ++n)
                printf(" %s %.3f", names[n], sum[n] / static_cast<float>(frames));
            printf("\n");
        }
    }
    rig.server->setDepthOfFieldBokeh(DofBokehShape::Hexagon, DofBlurQuality::Low, false);
    rig.server->setDepthOfFieldComputeAllowed(true);
    plain(rig);
    rig.resize(kWidth, kHeight);
}

} // namespace

int main(int argc, char** argv)
{
    const bool vulkan = argc > 1 && strcmp(argv[1], "vulkan") == 0;

    CHECK(Device::headlessSupported());
    if (!Device::headlessSupported()) return 1;

    DeviceDesc desc;
    desc.title = "dof test";
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
    const bool hasCompute = rig.driver->caps().compute;
    printf("compute shaders: %s\n", hasCompute ? "yes" : "no, the raster path is the only one");
    MeshStore& meshes = *Main::meshes();
    rig.scenario = server.scenarioCreate();
    rig.env = ambientOnly();
    rig.apply();

    const float nearBars[3] = { -2.4f, -1.5f, -0.6f };
    const float farBars[4] = { 3.0f, 6.0f, 9.0f, 12.0f };
    const MeshId nearMesh = meshes.create("dof near wall");
    CHECK(meshes.addSurface(nearMesh,
            barWall(-8.0f, -0.1f, -0.9f, 0.9f, -kNearDepth, 0.2f, nearBars, 3)));
    const MeshId farMesh = meshes.create("dof far wall");
    CHECK(meshes.addSurface(farMesh,
            barWall(1.5f, 40.0f, -4.5f, 4.5f, -kFarDepth, 0.7f, farBars, 4)));
    rig.nearWall = server.instanceCreate(nearMesh, rig.scenario);
    rig.farWall = server.instanceCreate(farMesh, rig.scenario);

    const MeshId floorMesh = meshes.create("dof showcase floor");
    {
        SurfaceData data;
        const float stripe = 1.0f;
        for (int i = 0; i < 40; ++i)
        {
            const float z0 = 4.0f - stripe * static_cast<float>(i + 1);
            const float z1 = z0 + stripe;
            const Math::Vec4 colour = (i & 1) ? Math::Vec4(0.25f, 0.45f, 0.2f, 1.0f)
                                              : Math::Vec4(0.55f, 0.65f, 0.3f, 1.0f);
            addBoxFace(data, Math::Vec3(-30.0f, -1.5f, z1), Math::Vec3(30.0f, -1.5f, z1),
                    Math::Vec3(30.0f, -1.5f, z0), Math::Vec3(-30.0f, -1.5f, z0),
                    Math::Vec3(0.0f, 1.0f, 0.0f), colour);
        }
        CHECK(meshes.addSurface(floorMesh, data));
    }
    const MeshId boxMesh = meshes.create("dof showcase box");
    CHECK(meshes.addSurface(boxMesh, colouredBox()));
    rig.extras.push_back(server.instanceCreate(floorMesh, rig.scenario));
    const float depths[7] = { 2.2f, 3.5f, 5.0f, 7.5f, 11.0f, 16.0f, 24.0f };
    const float spread[7] = { -0.45f, 0.3f, -0.12f, 0.5f, -0.5f, 0.0f, 0.3f };
    for (int i = 0; i < 7; ++i)
    {
        const InstanceId box = server.instanceCreate(boxMesh, rig.scenario);
        const float size = 0.35f + 0.1f * static_cast<float>(i);
        server.instanceSetTransform(box,
                Math::Mat4::Translation(Math::Vec3(spread[i] * depths[i], -1.5f, -depths[i])) *
                        Math::Mat4::RotationY(0.5f + 0.7f * static_cast<float>(i)) *
                        Math::Mat4::Scale(Math::Vec3(size, size * 2.2f, size)));
        rig.extras.push_back(box);
    }
    for (InstanceId id: rig.extras) server.instanceSetVisible(id, false);

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

    const char* tag = vulkan ? "vulkan" : "opengl";
    CHECK(server.depthOfFieldComputeAllowed());
    CHECK(server.depthOfFieldShape() == DofBokehShape::Hexagon);
    CHECK(server.depthOfFieldQuality() == DofBlurQuality::Low);
    CHECK(!server.depthOfFieldJitter());

    const int rounds = hasCompute ? 2 : 1;
    for (int round = 0; round < rounds; ++round)
    {
        const bool raster = round == 1 || !hasCompute;
        server.setDepthOfFieldComputeAllowed(!raster);
        char pathTag[64];
        snprintf(pathTag, sizeof(pathTag), "%s_%s", raster ? "raster" : "compute", tag);
        printf("--- %s path ---\n", raster ? "raster" : "compute");
        testDisabled(rig, round == 0);
        testFarAndNear(rig, pathTag);
        testMonotonic(rig);
        testShapes(rig);
        testJitter(rig);
        testToggle(rig);
    }
    server.setDepthOfFieldComputeAllowed(hasCompute);
    if (hasCompute) testPaths(rig);
    testProfiler(rig);
    testResize(rig);
    testTogether(rig);
    testCameraAttributes(rig);
    testMultisampling(rig);
    testTransparentBackground(rig);
    testOrthogonal(rig);
    testCustomProjection(rig);
    testAlpha(rig);
    showcase(rig, tag);
    cost(rig);
    testNodes(rig, tree, nearMesh, farMesh);

    Main::device()->requestClose();
    tree.finalize();
    Main::cleanup();

    if (failures) printf("%d check(s) failed\n", failures);
    else
        printf("all checks passed\n");
    return failures ? 1 : 0;
}
