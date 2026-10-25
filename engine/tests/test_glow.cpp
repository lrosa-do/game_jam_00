#include "Check.h"
#include "LitTest.h"

#include "prisma3d/Device.h"
#include "prisma3d/main/Main.h"
#include "prisma3d/render/MaterialStore.h"
#include "prisma3d/render/MeshStore.h"
#include "prisma3d/render/RenderProfiler.h"
#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/render/TextureStore.h"
#include "prisma3d/scene/SceneTree.h"

#include <ct/vector.hpp>

#include <math.h>
#include <stdlib.h>
#include <string.h>

using namespace prisma3d;

namespace
{

const int kSize = 128;
const int kCentre = kSize / 2;
const int kHalfSide = 8;
const int kTolerance = 2;
const float kSquareHalf = 0.7217f;

float toSrgb(float value)
{
    const float clamped = value < 0.0f ? 0.0f : (value > 1.0f ? 1.0f : value);
    return clamped <= 0.0031308f ? clamped * 12.92f : 1.055f * powf(clamped, 1.0f / 2.4f) - 0.055f;
}

int byteOf(float linear)
{
    return static_cast<int>(toSrgb(linear) * 255.0f + 0.5f);
}

float smoothstepOf(float edge0, float edge1, float x)
{
    float t = (x - edge0) / (edge1 - edge0);
    t = t < 0.0f ? 0.0f : (t > 1.0f ? 1.0f : t);
    return t * t * (3.0f - 2.0f * t);
}

struct Image
{
    int width = 0;
    int height = 0;
    ct::Vector<std::uint8_t> pixels;

    int value(int x, int y) const
    {
        return pixels[(static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
                              static_cast<std::size_t>(x)) *
                      4];
    }

    bool same(const Image& other) const
    {
        return width == other.width && height == other.height &&
               pixels.size() == other.pixels.size() &&
               memcmp(pixels.data(), other.pixels.data(), pixels.size()) == 0;
    }
};

bool insideSquare(int x, int y)
{
    return abs(x - kCentre) <= kHalfSide - (x >= kCentre ? 1 : 0) &&
           abs(y - kCentre) <= kHalfSide - (y >= kCentre ? 1 : 0);
}

long outsideSum(const Image& image)
{
    long sum = 0;
    for (int y = 0; y < image.height; ++y)
        for (int x = 0; x < image.width; ++x)
            if (!insideSquare(x, y)) sum += image.value(x, y);
    return sum;
}

int outsideReach(const Image& image, int threshold)
{
    int count = 0;
    for (int x = kCentre + kHalfSide; x < image.width; ++x)
        if (image.value(x, kCentre) >= threshold) ++count;
    return count;
}

struct Rig
{
    RenderingServer* server = nullptr;
    prisma::Driver* driver = nullptr;
    TextureStore* textures = nullptr;
    MaterialStore* materials = nullptr;
    ScenarioId scenario;
    CameraId camera;
    ViewportId view;
    ViewportId screen;
    InstanceId square;
    InstanceId wall;
    MaterialId emission;
    EnvironmentSettings env;
    int width = kSize;
    int height = kSize;
    RenderStats stats;

    void apply()
    {
        server->scenarioSetEnvironment(scenario, env);
    }

    void setEnergy(float energy)
    {
        materials->setFloat(emission, MaterialParam::EmissionEnergy, energy);
    }

    void showSquare()
    {
        server->instanceSetVisible(square, true);
        server->instanceSetVisible(wall, false);
    }

    void showWall()
    {
        server->instanceSetVisible(square, false);
        server->instanceSetVisible(wall, true);
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

    bool capture(Image& image, int primes = 3)
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

    bool captureScreen(Image& image, int primes = 3)
    {
        draw(primes);
        image.width = kSize;
        image.height = kSize;
        image.pixels.resize(static_cast<std::size_t>(kSize) * kSize * 4);
        prisma::Rect rect;
        rect.width = kSize;
        rect.height = kSize;
        const bool read = driver->readPixels(prisma::RenderTarget(), rect, image.pixels.data());
        server->endFrame(true);
        return read;
    }

    void resize(int w, int h)
    {
        width = w;
        height = h;
        server->viewportSetSize(view, static_cast<std::uint32_t>(w), static_cast<std::uint32_t>(h));
    }
};

SurfaceData quadData(float half)
{
    SurfaceData data;
    data.positions.push_back(Math::Vec3(-half, -half, 0.0f));
    data.positions.push_back(Math::Vec3(half, -half, 0.0f));
    data.positions.push_back(Math::Vec3(half, half, 0.0f));
    data.positions.push_back(Math::Vec3(-half, half, 0.0f));
    for (int i = 0; i < 4; ++i)
    {
        data.colors.push_back(Math::Vec4(1.0f, 1.0f, 1.0f, 1.0f));
        data.normals.push_back(Math::Vec3(0.0f, 0.0f, 1.0f));
    }
    data.uvs.push_back(Math::Vec2(0.0f, 0.0f));
    data.uvs.push_back(Math::Vec2(1.0f, 0.0f));
    data.uvs.push_back(Math::Vec2(1.0f, 1.0f));
    data.uvs.push_back(Math::Vec2(0.0f, 1.0f));
    const std::uint32_t indices[6] = { 0, 1, 2, 0, 2, 3 };
    for (std::uint32_t index: indices) data.indices.push_back(index);
    return data;
}

void glowOn(Rig& rig)
{
    rig.env.glowEnabled = true;
    rig.apply();
}

void glowOff(Rig& rig)
{
    rig.env.glowEnabled = false;
    rig.apply();
}

EnvironmentSettings baseEnvironment()
{
    EnvironmentSettings env = whiteAmbient();
    return env;
}

const float kKernelSum = 0.2024f + 2.0f * (0.1790f + 0.1240f + 0.0672f + 0.0285f);

float expectedLevel(float grey, int level, const EnvironmentSettings& env)
{
    const float cap = env.glowHdrLuminanceCap > 6.0f ? env.glowHdrLuminanceCap : 6.0f;
    const float tapped = grey / (1.0f + grey / cap);
    const float blurred = tapped * kKernelSum;
    float value = blurred / (1.0f - blurred / cap);
    value *= env.glowStrength;
    value *= env.tonemapExposure;
    const float feedback = fmaxf(smoothstepOf(env.glowHdrBleedThreshold,
                                         env.glowHdrBleedThreshold + env.glowHdrBleedScale, value),
            env.glowBloom);
    value = fminf(value * feedback, env.glowHdrLuminanceCap);
    for (int i = 1; i <= level; ++i) value = value * kKernelSum * env.glowStrength;
    return value;
}

float expectedGather(float grey, const EnvironmentSettings& env)
{
    float sum = 0.0f;
    for (int i = 0; i < 7; ++i)
        if (env.glowLevels[i] > 0.0001f) sum += expectedLevel(grey, i, env) * env.glowLevels[i];
    return sum;
}

float expectedBlend(GlowBlendMode mode, float colour, const EnvironmentSettings& env, float& gather)
{
    gather = expectedGather(colour, env);
    const float intensity = mode == GlowBlendMode::Mix ? env.glowMix : env.glowIntensity;
    const float glow = gather * intensity;
    switch (mode)
    {
        case GlowBlendMode::Additive:
            return colour + glow;
        case GlowBlendMode::Screen:
        {
            const float clamped = glow < 0.0f ? 0.0f : (glow > 1.0f ? 1.0f : glow);
            return colour + clamped - colour * clamped;
        }
        case GlowBlendMode::SoftLight:
        {
            const float clamped = glow < 0.0f ? 0.0f : (glow > 1.0f ? 1.0f : glow);
            const float curve = colour <= 0.25f
                                        ? ((16.0f * colour - 12.0f) * colour + 4.0f) * colour
                                        : sqrtf(colour);
            return colour > 1.0f ? colour : colour + clamped * (curve - colour);
        }
        case GlowBlendMode::Replace:
            return glow;
        case GlowBlendMode::Mix:
            return colour * (1.0f - intensity) + glow;
    }
    return colour;
}

void testDisabled(Rig& rig)
{
    printf("glow disabled: nothing is created and the frame does not change\n");
    rig.env = baseEnvironment();
    rig.apply();
    rig.setEnergy(4.0f);
    rig.showSquare();

    Image reference;
    CHECK(rig.capture(reference));
    CHECK(reference.value(kCentre, kCentre) == 255);
    CHECK(outsideSum(reference) == 0);
    CHECK(rig.stats.glowPasses == 0);
    CHECK(rig.stats.glowTargetsCreated == 0);
    CHECK(rig.stats.glowTexturesLive == 0);

    EnvironmentSettings changed = baseEnvironment();
    changed.glowEnabled = false;
    changed.glowIntensity = 8.0f;
    changed.glowStrength = 2.0f;
    changed.glowBloom = 1.0f;
    changed.glowBlendMode = GlowBlendMode::Replace;
    changed.glowHdrBleedThreshold = 0.0f;
    for (int i = 0; i < 7; ++i) changed.glowLevels[i] = 4.0f;
    rig.env = changed;
    rig.apply();
    Image other;
    CHECK(rig.capture(other));
    CHECK(other.same(reference));
    CHECK(rig.stats.glowPasses == 0);
    CHECK(rig.stats.glowTargetsCreated == 0);
    CHECK(rig.stats.glowTexturesLive == 0);

    RenderProfiler* profiler = Main::profiler();
    if (profiler)
    {
        bool found = false;
        for (std::uint32_t i = 0; i < profiler->passCount(); ++i)
            if (strcmp(profiler->passName(i), "Glow") == 0) found = true;
        CHECK(!found);
    }
}

void testHalo(Rig& rig)
{
    printf("halo of an emissive square: both blur directions, symmetry, falloff\n");
    rig.env = baseEnvironment();
    glowOn(rig);
    rig.setEnergy(4.0f);
    rig.showSquare();

    Image image;
    CHECK(rig.capture(image, 0));
    CHECK(rig.stats.glowPasses > 0);
    CHECK(rig.stats.glowTargetsCreated == 1);
    CHECK(rig.stats.glowTexturesLive == 2);
    CHECK(image.value(kCentre, kCentre) == 255);

    const int edge = kCentre + kHalfSide;
    printf("  row through the centre, outside the square:");
    for (int x = edge; x < edge + 40; x += 4) printf(" %d", image.value(x, kCentre));
    printf("\n");
    CHECK(image.value(edge, kCentre) > 0);
    CHECK(image.value(edge + 6, kCentre) > 0);
    CHECK(image.value(edge, kCentre) > image.value(edge + 24, kCentre));
    CHECK(image.value(edge + 24, kCentre) > image.value(kSize - 1, kCentre) ||
            image.value(kSize - 1, kCentre) == 0);

    int widestStep = 0;
    int worstLeftRight = 0;
    int worstUpDown = 0;
    int worstTurn = 0;
    for (int x = edge; x < kSize; ++x)
    {
        const int here = image.value(x, kCentre);
        if (x + 1 < kSize)
        {
            const int step = image.value(x + 1, kCentre) - here;
            if (step > widestStep) widestStep = step;
        }
        const int mirror = kSize - 1 - x;
        const int leftRight = abs(here - image.value(mirror, kCentre));
        const int upDown = abs(image.value(kCentre, x) - image.value(kCentre, mirror));
        const int turn = abs(here - image.value(kCentre, x));
        if (leftRight > worstLeftRight) worstLeftRight = leftRight;
        if (upDown > worstUpDown) worstUpDown = upDown;
        if (turn > worstTurn) worstTurn = turn;
    }
    printf("  largest rise outwards %d, worst left-right %d, up-down %d, horizontal-vertical %d\n",
            widestStep, worstLeftRight, worstUpDown, worstTurn);
    CHECK(widestStep <= 1);
    CHECK(worstLeftRight <= kTolerance);
    CHECK(worstUpDown <= kTolerance);
    CHECK(worstTurn <= kTolerance);

    int diagonalWorst = 0;
    for (int k = 1; k < 40; ++k)
    {
        const int a = image.value(kCentre + kHalfSide + k, kCentre + kHalfSide + k);
        const int b = image.value(kCentre - kHalfSide - 1 - k, kCentre + kHalfSide + k);
        const int c = image.value(kCentre + kHalfSide + k, kCentre - kHalfSide - 1 - k);
        const int d = image.value(kCentre - kHalfSide - 1 - k, kCentre - kHalfSide - 1 - k);
        const int worst = abs(a - b) > abs(a - c) ? abs(a - b) : abs(a - c);
        const int all = worst > abs(a - d) ? worst : abs(a - d);
        if (all > diagonalWorst) diagonalWorst = all;
    }
    CHECK(diagonalWorst <= kTolerance);
    CHECK(image.value(kCentre + kHalfSide + 2, kCentre + kHalfSide + 2) > 0);
}

void testIntensity(Rig& rig)
{
    printf("the energy of the halo grows with the intensity\n");
    rig.env = baseEnvironment();
    glowOn(rig);
    rig.setEnergy(4.0f);
    rig.showSquare();
    const float intensities[5] = { 0.0f, 0.1f, 0.3f, 0.6f, 1.2f };
    long previous = -1;
    for (int i = 0; i < 5; ++i)
    {
        rig.env.glowIntensity = intensities[i];
        rig.apply();
        Image image;
        CHECK(rig.capture(image, 1));
        const long sum = outsideSum(image);
        printf("  intensity %.1f: sum of the bytes outside the square %ld\n", intensities[i], sum);
        if (i == 0) CHECK(sum == 0);
        else
            CHECK(sum > previous);
        previous = sum;
    }
}

void testThreshold(Rig& rig)
{
    printf("hdr threshold and bloom\n");
    rig.env = baseEnvironment();
    glowOn(rig);
    rig.showSquare();

    rig.setEnergy(0.5f);
    rig.apply();
    Image below;
    CHECK(rig.capture(below));
    CHECK(abs(below.value(kCentre, kCentre) - byteOf(0.5f)) <= kTolerance);
    CHECK(outsideSum(below) == 0);

    rig.env.glowBloom = 0.5f;
    rig.apply();
    Image bloom;
    CHECK(rig.capture(bloom));
    const long bloomSum = outsideSum(bloom);
    printf("  value 0.5 below the threshold 1: bloom 0 gives %ld, bloom 0.5 gives %ld\n",
            outsideSum(below), bloomSum);
    CHECK(bloomSum > 0);
    CHECK(bloom.value(kCentre + kHalfSide, kCentre) > 0);

    rig.env.glowBloom = 0.0f;
    rig.setEnergy(4.0f);
    rig.apply();
    Image above;
    CHECK(rig.capture(above));
    CHECK(outsideSum(above) > 0);

    rig.env.glowHdrBleedThreshold = 6.0f;
    rig.apply();
    Image raised;
    CHECK(rig.capture(raised));
    CHECK(outsideSum(raised) == 0);

    rig.env.glowHdrBleedThreshold = 0.0f;
    rig.env.glowHdrBleedScale = 8.0f;
    rig.apply();
    Image soft;
    CHECK(rig.capture(soft));
    const long softSum = outsideSum(soft);
    printf("  value 4: threshold 1 scale 2 gives %ld, threshold 6 gives 0, threshold 0 scale 8 "
           "gives %ld\n",
            outsideSum(above), softSum);
    CHECK(softSum > 0 && softSum < outsideSum(above));

    rig.env.glowHdrBleedThreshold = 1.0f;
    rig.env.glowHdrBleedScale = 2.0f;
    rig.env.glowHdrLuminanceCap = 2.0f;
    rig.apply();
    Image capped;
    CHECK(rig.capture(capped));
    printf("  luminance cap 2: %ld against %ld\n", outsideSum(capped), outsideSum(above));
    CHECK(outsideSum(capped) > 0 && outsideSum(capped) < outsideSum(above));
}

void testBlendModes(Rig& rig)
{
    printf("blend modes on a uniform mid grey of 0.5\n");
    rig.env = baseEnvironment();
    glowOn(rig);
    rig.env.glowBloom = 1.0f;
    rig.env.glowMix = 0.2f;
    rig.setEnergy(0.5f);
    rig.showWall();
    const GlowBlendMode modes[5] = { GlowBlendMode::Additive, GlowBlendMode::Screen,
        GlowBlendMode::SoftLight, GlowBlendMode::Replace, GlowBlendMode::Mix };
    const char* names[5] = { "additive", "screen", "soft light", "replace", "mix" };
    for (int i = 0; i < 5; ++i)
    {
        rig.env.glowBlendMode = modes[i];
        rig.apply();
        Image image;
        CHECK(rig.capture(image));
        float gather = 0.0f;
        const float expected = expectedBlend(modes[i], 0.5f, rig.env, gather);
        const int expectedByte = byteOf(expected);
        const int centre = image.value(kCentre, kCentre);
        const int corner = image.value(2, 2);
        const int far = image.value(kSize - 3, kSize - 3);
        printf("  %-10s gather %.5f, expected linear %.5f, byte %3d, read %3d (corner %3d, %3d)\n",
                names[i], gather, expected, expectedByte, centre, corner, far);
        CHECK(abs(centre - expectedByte) <= kTolerance);
        CHECK(abs(corner - expectedByte) <= kTolerance);
        CHECK(abs(far - expectedByte) <= kTolerance);
    }

    rig.env.glowBlendMode = GlowBlendMode::Screen;
    rig.env.glowStrength = 0.5f;
    rig.apply();
    Image strength;
    CHECK(rig.capture(strength));
    float gather = 0.0f;
    const int strengthByte = byteOf(expectedBlend(GlowBlendMode::Screen, 0.5f, rig.env, gather));
    printf("  strength 0.5 (screen): gather %.5f, byte %d, read %d\n", gather, strengthByte,
            strength.value(kCentre, kCentre));
    CHECK(abs(strength.value(kCentre, kCentre) - strengthByte) <= kTolerance);

    rig.env.glowStrength = 1.0f;
    rig.env.glowLevels[0] = 0.0f;
    rig.env.glowLevels[1] = 0.0f;
    rig.env.glowLevels[2] = 0.0f;
    rig.env.glowLevels[3] = 0.0f;
    rig.env.glowLevels[4] = 1.0f;
    rig.env.glowLevels[5] = 0.5f;
    rig.apply();
    Image deep;
    CHECK(rig.capture(deep));
    const int deepByte = byteOf(expectedBlend(GlowBlendMode::Screen, 0.5f, rig.env, gather));
    printf("  levels 5 and 6 only: gather %.5f, byte %d, read %d\n", gather, deepByte,
            deep.value(kCentre, kCentre));
    CHECK(abs(deep.value(kCentre, kCentre) - deepByte) <= kTolerance);

    rig.env.glowLevels[4] = 0.0f;
    rig.env.glowLevels[5] = 0.0f;
    rig.env.glowLevels[1] = 3.0f;
    rig.env.glowLevels[2] = 1.0f;
    rig.env.glowNormalized = true;
    rig.apply();
    Image normalised;
    CHECK(rig.capture(normalised));
    rig.env.glowNormalized = false;
    rig.env.glowLevels[1] = 0.75f;
    rig.env.glowLevels[2] = 0.25f;
    rig.apply();
    Image direct;
    CHECK(rig.capture(direct));
    CHECK(normalised.same(direct));
}

void testLevels(Rig& rig)
{
    printf("per level switches change the radius of the halo\n");
    rig.env = baseEnvironment();
    glowOn(rig);
    rig.env.glowIntensity = 1.0f;
    rig.setEnergy(8.0f);
    rig.showSquare();
    int reach[3] = {};
    const int levels[3] = { 0, 1, 3 };
    for (int i = 0; i < 3; ++i)
    {
        for (int l = 0; l < 7; ++l) rig.env.glowLevels[l] = l == levels[i] ? 1.0f : 0.0f;
        rig.apply();
        Image image;
        CHECK(rig.capture(image));
        reach[i] = outsideReach(image, 3);
        printf("  only level %d: %d pixels of the row at 3 bytes or more\n", levels[i] + 1,
                reach[i]);
        CHECK(image.value(kCentre + kHalfSide, kCentre) > 0);
    }
    CHECK(reach[0] > 0);
    CHECK(reach[0] < reach[1]);
    CHECK(reach[1] < reach[2]);

    Image off;
    glowOff(rig);
    CHECK(rig.capture(off));
    rig.env.glowEnabled = true;
    for (int l = 0; l < 7; ++l) rig.env.glowLevels[l] = 0.0f;
    rig.apply();
    Image none;
    CHECK(rig.capture(none));
    CHECK(none.same(off));
}

void testGlowMap(Rig& rig)
{
    printf("glow map\n");
    rig.env = baseEnvironment();
    glowOn(rig);
    rig.setEnergy(4.0f);
    rig.showSquare();
    Image plain;
    CHECK(rig.capture(plain));
    const long plainSum = outsideSum(plain);
    CHECK(plainSum > 0);

    rig.env.glowMap = rig.textures->black();
    rig.env.glowMapStrength = 0.0f;
    rig.apply();
    Image zero;
    CHECK(rig.capture(zero));
    CHECK(zero.same(plain));

    rig.env.glowMapStrength = 0.5f;
    rig.apply();
    Image half;
    CHECK(rig.capture(half));
    rig.env.glowMapStrength = 1.0f;
    rig.apply();
    Image full;
    CHECK(rig.capture(full));
    printf("  halo with no map %ld, black map at 0.5 %ld, at 1.0 %ld\n", plainSum, outsideSum(half),
            outsideSum(full));
    CHECK(outsideSum(half) > 0 && outsideSum(half) < plainSum);
    CHECK(outsideSum(full) == 0);

    rig.env.glowMap = TextureId();
    rig.apply();
    Image removed;
    CHECK(rig.capture(removed));
    CHECK(removed.same(plain));
}

void testToggle(Rig& rig)
{
    printf("toggling at run time\n");
    rig.env = baseEnvironment();
    rig.setEnergy(4.0f);
    rig.showSquare();
    rig.resize(kSize, kSize);
    rig.apply();

    Image off[3];
    Image on[3];
    for (int round = 0; round < 3; ++round)
    {
        glowOff(rig);
        CHECK(rig.capture(off[round], 1));
        CHECK(rig.stats.glowPasses == 0);
        glowOn(rig);
        CHECK(rig.capture(on[round], 1));
        CHECK(rig.stats.glowPasses > 0);
        CHECK(rig.stats.glowTexturesLive == 2);
        CHECK(rig.stats.glowTargetsCreated == 0);
        CHECK(outsideSum(off[round]) == 0);
        CHECK(outsideSum(on[round]) > 0);
    }
    CHECK(off[0].same(off[1]) && off[1].same(off[2]));
    CHECK(on[0].same(on[1]) && on[1].same(on[2]));
}

void testResize(Rig& rig)
{
    printf("the targets follow the viewport size\n");
    rig.env = baseEnvironment();
    glowOn(rig);
    rig.setEnergy(4.0f);
    rig.showSquare();

    const int sizes[8][2] = { { 96, 96 }, { 128, 128 }, { 100, 75 }, { 33, 17 }, { 128, 128 },
        { 64, 64 }, { 8, 8 }, { 128, 128 } };
    for (int i = 0; i < 8; ++i)
    {
        rig.resize(sizes[i][0], sizes[i][1]);
        Image image;
        CHECK(rig.capture(image, 0));
        const RenderStats first = rig.stats;
        printf("  %3dx%-3d: glow targets created %u, live %u, passes %u\n", sizes[i][0],
                sizes[i][1], first.glowTargetsCreated, first.glowTexturesLive, first.glowPasses);
        CHECK(first.glowTargetsCreated == 1);
        CHECK(first.glowTexturesLive == 2);
        CHECK(first.glowPasses > 0);
        int lit = 0;
        for (int y = 0; y < image.height; ++y)
            for (int x = 0; x < image.width; ++x)
                if (image.value(x, y) > 0) ++lit;
        CHECK(lit > (image.width >= 33 ? 4 : 0));
        CHECK(rig.capture(image, 0));
        CHECK(rig.stats.glowTargetsCreated == 0);
        CHECK(rig.stats.glowTexturesLive == 2);
    }

    rig.resize(6, 6);
    Image small;
    CHECK(rig.capture(small, 0));
    printf("  6x6: live %u, passes %u\n", rig.stats.glowTexturesLive, rig.stats.glowPasses);
    CHECK(rig.stats.glowTexturesLive == 0);
    CHECK(rig.stats.glowPasses == 0);

    rig.resize(kSize, kSize);
    Image back;
    CHECK(rig.capture(back, 0));
    CHECK(rig.stats.glowTexturesLive == 2);
    CHECK(rig.stats.glowTargetsCreated == 1);
}

void testMultisampling(Rig& rig)
{
    printf("multisampling reads the resolved colour\n");
    rig.env = baseEnvironment();
    glowOn(rig);
    rig.setEnergy(4.0f);
    rig.showSquare();
    rig.resize(kSize, kSize);

    Image single;
    CHECK(rig.capture(single));
    rig.server->viewportSetMsaa3D(rig.view, ViewportMsaa::Msaa4x);
    Image multi;
    CHECK(rig.capture(multi));
    printf("  samples %u, halo %ld against %ld\n", rig.server->viewportGetMsaaSamples(rig.view),
            outsideSum(multi), outsideSum(single));
    CHECK(rig.stats.glowPasses > 0);
    CHECK(outsideSum(multi) > 0);
    int worst = 0;
    for (int y = 0; y < kSize; ++y)
        for (int x = 0; x < kSize; ++x)
        {
            const int d = abs(single.value(x, y) - multi.value(x, y));
            if (d > worst) worst = d;
        }
    printf("  largest difference to the single sample frame %d\n", worst);
    CHECK(worst <= kTolerance);
    rig.server->viewportSetMsaa3D(rig.view, ViewportMsaa::Disabled);
}

void testScreen(Rig& rig)
{
    printf("glow on the window\n");
    rig.env = baseEnvironment();
    glowOn(rig);
    rig.setEnergy(4.0f);
    rig.showSquare();
    rig.resize(kSize, kSize);
    Image texture;
    CHECK(rig.capture(texture));

    rig.server->viewportSetActive(rig.view, false);
    rig.screen = rig.server->viewportCreate();
    rig.server->viewportSetSize(rig.screen, kSize, kSize);
    rig.server->viewportAttachToScreen(rig.screen, true);
    rig.server->viewportAttachCamera(rig.screen, rig.camera);
    rig.server->viewportSetScenario(rig.screen, rig.scenario);
    rig.server->viewportSetClearColor(rig.screen, Math::Vec4(0.0f, 0.0f, 0.0f, 1.0f));
    Image window;
    CHECK(rig.captureScreen(window));
    CHECK(rig.stats.glowPasses > 0);
    int worst = 0;
    for (int y = 0; y < kSize; ++y)
        for (int x = 0; x < kSize; ++x)
        {
            const int d = abs(window.value(x, y) - texture.value(x, y));
            if (d > worst) worst = d;
        }
    printf("  largest difference between the window and the viewport texture %d\n", worst);
    CHECK(worst <= kTolerance);
    CHECK(outsideSum(window) > 0);

    glowOff(rig);
    Image windowOff;
    CHECK(rig.captureScreen(windowOff));
    CHECK(rig.stats.glowPasses == 0);
    CHECK(outsideSum(windowOff) == 0);
    rig.server->viewportFree(rig.screen);
    rig.screen = ViewportId();
    rig.server->viewportSetActive(rig.view, true);
}

const ProfileSample* gpuSample(const char* name)
{
    RenderProfiler* profiler = Main::profiler();
    for (std::uint32_t i = 0; i < profiler->gpuSampleCount(); ++i)
        if (strcmp(profiler->gpuSamples()[i].name, name) == 0) return &profiler->gpuSamples()[i];
    return nullptr;
}

void testProfiler(Rig& rig)
{
    printf("the glow passes have a profiler pass and a GPU scope\n");
    RenderProfiler* profiler = Main::profiler();
    CHECK(profiler != nullptr);
    if (!profiler) return;
    rig.env = baseEnvironment();
    glowOn(rig);
    rig.setEnergy(4.0f);
    rig.showSquare();
    rig.resize(kSize, kSize);
    Image on;
    CHECK(rig.capture(on));

    bool found = false;
    for (std::uint32_t i = 0; i < profiler->passCount(); ++i)
        if (strcmp(profiler->passName(i), "Glow") == 0) found = true;
    CHECK(found);
    CHECK(profiler->passEnabled("Glow"));
    CHECK(outsideSum(on) > 0);

    if (profiler->gpuAvailable())
    {
        for (int i = 0; i < 60; ++i)
        {
            if (gpuSample("Glow") && gpuSample("Glow")->historyCount >= 2) break;
            Main::iteration();
        }
        const ProfileSample* glow = gpuSample("Glow");
        CHECK(glow != nullptr);
        if (glow)
        {
            printf("  gpu Glow %.4f ms\n", glow->milliseconds);
            CHECK(glow->milliseconds >= 0.0f);
        }
    }
    else
        printf("  gpu timer queries are not available on this driver\n");

    profiler->setPassEnabled("Glow", false);
    Image disabled;
    CHECK(rig.capture(disabled));
    CHECK(rig.stats.glowPasses == 0);
    CHECK(outsideSum(disabled) == 0);
    profiler->setPassEnabled("Glow", true);
    Image again;
    CHECK(rig.capture(again));
    CHECK(again.same(on));
}

} // namespace

int main(int argc, char** argv)
{
    const bool vulkan = argc > 1 && strcmp(argv[1], "vulkan") == 0;

    CHECK(Device::headlessSupported());
    if (!Device::headlessSupported()) return 1;

    DeviceDesc desc;
    desc.title = "glow test";
    desc.driver = vulkan ? prisma::DriverType::Vulkan : prisma::DriverType::OpenGL;
    desc.width = kSize;
    desc.height = kSize;
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
    MeshStore& meshes = *Main::meshes();
    rig.scenario = server.scenarioCreate();
    rig.env = baseEnvironment();
    rig.apply();

    rig.emission = rig.materials->create("glow emission");
    rig.materials->setColor(rig.emission, MaterialParam::Albedo,
            Math::Vec4(0.0f, 0.0f, 0.0f, 1.0f));
    rig.materials->setColor(rig.emission, MaterialParam::Emission,
            Math::Vec4(1.0f, 1.0f, 1.0f, 1.0f));
    rig.materials->setFloat(rig.emission, MaterialParam::Specular, 0.0f);
    rig.materials->setFloat(rig.emission, MaterialParam::Roughness, 1.0f);
    rig.materials->setFloat(rig.emission, MaterialParam::EmissionEnergy, 4.0f);

    const MeshId squareMesh = meshes.create("glow square");
    CHECK(meshes.addSurface(squareMesh, quadData(kSquareHalf)));
    const MeshId wallMesh = meshes.create("glow wall");
    CHECK(meshes.addSurface(wallMesh, quadData(100.0f)));
    rig.square = server.instanceCreate(squareMesh, rig.scenario);
    rig.wall = server.instanceCreate(wallMesh, rig.scenario);
    server.instanceSetMaterialOverride(rig.square, rig.emission);
    server.instanceSetMaterialOverride(rig.wall, rig.emission);

    rig.camera = server.cameraCreate();
    server.cameraSetPerspective(rig.camera, 60.0f, 0.1f, 100.0f);
    server.cameraSetTransform(rig.camera, Math::Mat4::Translation(Math::Vec3(0.0f, 0.0f, 10.0f)));

    rig.view = server.viewportCreate();
    server.viewportSetSize(rig.view, kSize, kSize);
    server.viewportAttachCamera(rig.view, rig.camera);
    server.viewportSetScenario(rig.view, rig.scenario);
    server.viewportSetClearColor(rig.view, Math::Vec4(0.0f, 0.0f, 0.0f, 1.0f));
    server.viewportSetUpdateMode(rig.view, ViewportUpdateMode::Always);
    CHECK(server.viewportGetTexture(rig.view).valid());

    testDisabled(rig);
    testHalo(rig);
    testIntensity(rig);
    testThreshold(rig);
    testBlendModes(rig);
    testLevels(rig);
    testGlowMap(rig);
    testToggle(rig);
    testMultisampling(rig);
    testScreen(rig);
    testProfiler(rig);
    testResize(rig);

    Main::device()->requestClose();
    tree.finalize();
    Main::cleanup();

    if (failures) printf("%d check(s) failed\n", failures);
    else
        printf("all checks passed\n");
    return failures ? 1 : 0;
}
