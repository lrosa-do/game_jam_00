#include "Check.h"
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
#include <stdlib.h>
#include <string.h>

using namespace prisma3d;

namespace
{

const int kSize = 32;
const int kTolerance = 2;

struct Rgb
{
    float r;
    float g;
    float b;
};

float srgbToLinear(float value)
{
    return value <= 0.04045f ? value / 12.92f : powf((value + 0.055f) / 1.055f, 2.4f);
}

float linearToSrgb(float value)
{
    return value < 0.0031308f ? value * 12.92f : 1.055f * powf(value, 1.0f / 2.4f) - 0.055f;
}

float clamp01(float value)
{
    return value < 0.0f ? 0.0f : (value > 1.0f ? 1.0f : value);
}

float brightnessScale(float brightness)
{
    return brightness < 0.04045f ? brightness * (1.0f / 12.92f)
                                 : powf((brightness + 0.055f) * (1.0f / 1.055f), 2.4f);
}

Rgb adjusted(const Rgb& srgb, float brightness, float contrast, float saturation)
{
    const float scale = brightnessScale(brightness);
    float c[3] = { srgbToLinear(srgb.r) * scale, srgbToLinear(srgb.g) * scale,
        srgbToLinear(srgb.b) * scale };
    for (float& channel: c) channel = linearToSrgb(channel);
    for (float& channel: c) channel = 0.5f + (channel - 0.5f) * contrast;
    const float average = (c[0] + c[1] + c[2]) * (1.0f / 3.0f);
    for (float& channel: c) channel = average + (channel - average) * saturation;
    return { clamp01(c[0]), clamp01(c[1]), clamp01(c[2]) };
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
    MaterialId wall;
    EnvironmentSettings env;
    RenderStats stats;

    void apply()
    {
        server->scenarioSetEnvironment(scenario, env);
    }

    void color(const Rgb& c)
    {
        materials->setColor(wall, MaterialParam::Albedo, Math::Vec4(c.r, c.g, c.b, 1.0f));
    }

    bool read(int& r, int& g, int& b)
    {
        for (int i = 0; i < 2; ++i)
        {
            server->renderViewports();
            server->endFrame(true);
        }
        server->renderViewports();
        stats = server->getStats();
        server->endFrame(true);
        ct::Vector<std::uint8_t> pixels;
        pixels.resize(kSize * kSize * 4);
        prisma::RenderTarget source;
        source.texture = textures->gpu(server->viewportGetTexture(view));
        prisma::Rect rect;
        rect.width = kSize;
        rect.height = kSize;
        if (!driver->readPixels(source, rect, pixels.data())) return false;
        const std::size_t at = (static_cast<std::size_t>(kSize / 2) * kSize + kSize / 2) * 4;
        r = pixels[at];
        g = pixels[at + 1];
        b = pixels[at + 2];
        return true;
    }
};

bool near(int value, float expected)
{
    return abs(value - static_cast<int>(expected * 255.0f + 0.5f)) <= kTolerance;
}

bool check(Rig& rig, const Rgb& input, float brightness, float contrast, float saturation,
        const char* label)
{
    rig.color(input);
    rig.env.adjustmentEnabled = true;
    rig.env.adjustmentBrightness = brightness;
    rig.env.adjustmentContrast = contrast;
    rig.env.adjustmentSaturation = saturation;
    rig.apply();
    int r = 0;
    int g = 0;
    int b = 0;
    CHECK(rig.read(r, g, b));
    const Rgb expected = adjusted(input, brightness, contrast, saturation);
    const bool ok = near(r, expected.r) && near(g, expected.g) && near(b, expected.b);
    if (!ok)
    {
        printf("FAIL %s: input %.2f %.2f %.2f b %.2f c %.2f s %.2f: read %d %d %d, expected "
               "%.1f %.1f %.1f\n",
                label, input.r, input.g, input.b, brightness, contrast, saturation, r, g, b,
                expected.r * 255.0f, expected.g * 255.0f, expected.b * 255.0f);
        ++failures;
    }
    return ok;
}

void testOff(Rig& rig)
{
    printf("adjustments off: the image is the plain one whatever the values are\n");
    const Rgb inputs[3] = { { 0.8f, 0.4f, 0.2f }, { 0.25f, 0.5f, 0.75f }, { 0.5f, 0.5f, 0.5f } };
    for (const Rgb& input: inputs)
    {
        rig.color(input);
        rig.env = whiteAmbient();
        rig.apply();
        int r = 0;
        int g = 0;
        int b = 0;
        CHECK(rig.read(r, g, b));
        CHECK(near(r, input.r) && near(g, input.g) && near(b, input.b));
        rig.env.adjustmentEnabled = false;
        rig.env.adjustmentBrightness = 1.9f;
        rig.env.adjustmentContrast = 1.4f;
        rig.env.adjustmentSaturation = 0.1f;
        rig.apply();
        int r2 = 0;
        int g2 = 0;
        int b2 = 0;
        CHECK(rig.read(r2, g2, b2));
        CHECK(r == r2 && g == g2 && b == b2);
    }
    rig.env = whiteAmbient();
    rig.env.adjustmentEnabled = true;
    rig.apply();
    int r = 0;
    int g = 0;
    int b = 0;
    CHECK(rig.read(r, g, b));
    CHECK(near(r, 0.5f) && near(g, 0.5f) && near(b, 0.5f));
}

void testBrightness(Rig& rig)
{
    printf("brightness: scaled in linear light with the curve of the original\n");
    const float values[6] = { 0.0f, 0.5f, 1.0f, 1.25f, 1.5f, 2.0f };
    const Rgb inputs[2] = { { 0.8f, 0.4f, 0.2f }, { 0.25f, 0.5f, 0.75f } };
    for (const Rgb& input: inputs)
        for (float value: values) check(rig, input, value, 1.0f, 1.0f, "brightness");
}

void testContrast(Rig& rig)
{
    printf("contrast: around the middle grey of the encoded values\n");
    const float values[5] = { 0.0f, 0.75f, 1.0f, 1.25f, 1.5f };
    const Rgb inputs[2] = { { 0.8f, 0.4f, 0.2f }, { 0.25f, 0.5f, 0.75f } };
    for (const Rgb& input: inputs)
        for (float value: values) check(rig, input, 1.0f, value, 1.0f, "contrast");
}

void testSaturation(Rig& rig)
{
    printf("saturation: towards the average of the encoded channels\n");
    const float values[5] = { 0.0f, 0.5f, 1.0f, 1.5f, 2.0f };
    const Rgb inputs[2] = { { 0.8f, 0.4f, 0.2f }, { 0.25f, 0.5f, 0.75f } };
    for (const Rgb& input: inputs)
        for (float value: values) check(rig, input, 1.0f, 1.0f, value, "saturation");
    rig.color({ 0.8f, 0.4f, 0.2f });
    rig.env.adjustmentSaturation = 0.0f;
    rig.apply();
    int r = 0;
    int g = 0;
    int b = 0;
    CHECK(rig.read(r, g, b));
    CHECK(abs(r - g) <= 1 && abs(g - b) <= 1);
}

void testCombined(Rig& rig)
{
    printf("the three together\n");
    const Rgb inputs[3] = { { 0.8f, 0.4f, 0.2f }, { 0.25f, 0.5f, 0.75f }, { 0.1f, 0.9f, 0.3f } };
    const float sets[4][3] = { { 1.2f, 1.1f, 0.8f }, { 0.7f, 1.25f, 1.5f }, { 1.5f, 0.75f, 0.5f },
        { 2.0f, 1.0f, 1.0f } };
    for (const Rgb& input: inputs)
        for (const auto& set: sets) check(rig, input, set[0], set[1], set[2], "combined");
}

TextureId lookup(Rig& rig, bool inverted, const char* name)
{
    Pixmap pixels(256, 1, 4);
    for (int i = 0; i < 256; ++i)
    {
        const std::uint8_t value = static_cast<std::uint8_t>(inverted ? 255 - i : i);
        pixels.set_pixel(static_cast<std::uint32_t>(i), 0, value, value, value, 255);
    }
    return rig.textures->createFromPixmap(pixels, false, false, name);
}

void testColorCorrection(Rig& rig)
{
    printf("color correction: a lookup texture applied to the adjusted colors\n");
    const Rgb input = { 0.8f, 0.4f, 0.2f };
    rig.color(input);
    rig.env = whiteAmbient();
    rig.env.adjustmentEnabled = true;
    rig.env.adjustmentContrast = 1.2f;
    rig.apply();
    int pr = 0;
    int pg = 0;
    int pb = 0;
    CHECK(rig.read(pr, pg, pb));

    rig.env.adjustmentColorCorrection = lookup(rig, false, "identity lookup");
    CHECK(rig.textures->valid(rig.env.adjustmentColorCorrection));
    rig.apply();
    int r = 0;
    int g = 0;
    int b = 0;
    CHECK(rig.read(r, g, b));
    CHECK(abs(r - pr) <= kTolerance && abs(g - pg) <= kTolerance && abs(b - pb) <= kTolerance);

    const TextureId inverted = lookup(rig, true, "inverted lookup");
    rig.env.adjustmentColorCorrection = inverted;
    rig.apply();
    CHECK(rig.read(r, g, b));
    CHECK(abs(r - (255 - pr)) <= kTolerance + 1 && abs(g - (255 - pg)) <= kTolerance + 1 &&
            abs(b - (255 - pb)) <= kTolerance + 1);

    rig.env.adjustmentEnabled = false;
    rig.apply();
    int ar = 0;
    int ag = 0;
    int ab = 0;
    CHECK(rig.read(ar, ag, ab));
    CHECK(near(ar, input.r) && near(ag, input.g) && near(ab, input.b));

    rig.env.adjustmentEnabled = true;
    rig.env.adjustmentColorCorrection = TextureId();
    rig.apply();
    CHECK(rig.read(r, g, b));
    CHECK(abs(r - pr) <= kTolerance && abs(g - pg) <= kTolerance && abs(b - pb) <= kTolerance);
}

void testProfiler(Rig& rig)
{
    printf("the tonemap pass is the only one that runs for adjustments\n");
    rig.env = whiteAmbient();
    rig.env.adjustmentEnabled = true;
    rig.env.adjustmentBrightness = 1.5f;
    rig.apply();
    int r = 0;
    int g = 0;
    int b = 0;
    CHECK(rig.read(r, g, b));
    CHECK(rig.stats.ssaoDispatches == 0);
    CHECK(rig.stats.glowPasses == 0);
    RenderProfiler* profiler = Main::profiler();
    if (profiler)
        for (std::uint32_t i = 0; i < profiler->passCount(); ++i)
        {
            CHECK(strcmp(profiler->passName(i), "SSAO") != 0);
            CHECK(strcmp(profiler->passName(i), "Glow") != 0);
        }
}

} // namespace

int main(int argc, char** argv)
{
    const bool vulkan = argc > 1 && strcmp(argv[1], "vulkan") == 0;

    CHECK(Device::headlessSupported());
    if (!Device::headlessSupported()) return 1;

    DeviceDesc desc;
    desc.title = "adjustments test";
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
    rig.env = whiteAmbient();
    rig.apply();

    rig.wall = rig.materials->create("adjustments wall");
    rig.materials->setShading(rig.wall, MaterialShading::Unshaded);
    rig.color({ 0.5f, 0.5f, 0.5f });

    SurfaceData data;
    data.positions.push_back(Math::Vec3(-100.0f, -100.0f, 0.0f));
    data.positions.push_back(Math::Vec3(100.0f, -100.0f, 0.0f));
    data.positions.push_back(Math::Vec3(100.0f, 100.0f, 0.0f));
    data.positions.push_back(Math::Vec3(-100.0f, 100.0f, 0.0f));
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
    const MeshId mesh = meshes.create("adjustments wall");
    CHECK(meshes.addSurface(mesh, data));
    const InstanceId instance = server.instanceCreate(mesh, rig.scenario);
    server.instanceSetMaterialOverride(instance, rig.wall);

    rig.camera = server.cameraCreate();
    server.cameraSetPerspective(rig.camera, 60.0f, 0.1f, 100.0f);
    server.cameraSetTransform(rig.camera, Math::Mat4::Translation(Math::Vec3(0.0f, 0.0f, 10.0f)));
    rig.view = server.viewportCreate();
    server.viewportSetSize(rig.view, kSize, kSize);
    server.viewportAttachCamera(rig.view, rig.camera);
    server.viewportSetScenario(rig.view, rig.scenario);
    server.viewportSetClearColor(rig.view, Math::Vec4(0.0f, 0.0f, 0.0f, 1.0f));
    server.viewportSetUpdateMode(rig.view, ViewportUpdateMode::Always);
    server.viewportSetSrgbOutput(rig.view, true);
    CHECK(server.viewportGetTexture(rig.view).valid());

    testOff(rig);
    testBrightness(rig);
    testContrast(rig);
    testSaturation(rig);
    testCombined(rig);
    testColorCorrection(rig);
    testProfiler(rig);

    Main::device()->requestClose();
    tree.finalize();
    Main::cleanup();

    if (failures) printf("%d check(s) failed\n", failures);
    else
        printf("all checks passed\n");
    return failures ? 1 : 0;
}
