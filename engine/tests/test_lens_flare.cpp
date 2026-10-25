#include "Check.h"
#include "LitTest.h"

#include "prisma3d/Device.h"
#include "prisma3d/core/Pixmap.h"
#include "prisma3d/main/Main.h"
#include "prisma3d/render/MaterialStore.h"
#include "prisma3d/render/MeshStore.h"
#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/render/TextureStore.h"
#include "prisma3d/scene/SceneTree.h"

#include <ct/vector.hpp>

#include <math.h>
#include <stdio.h>
#include <string.h>

using namespace prisma3d;

namespace
{

const int kSize = 128;
const float kFovDegrees = 60.0f;
const float kTanHalfFov = 0.57735f;
const int kFlareSize = 32;
const int kDiscRadius = 14;
const Math::Vec3 kCameraPosition(0.0f, 0.0f, 10.0f);
const Math::Vec3 kToSun = Math::Vec3(0.3f, 0.2f, -1.0f).Normalized();
const float kOccluderDistance = 5.0f;
const float kOccluderHalf = 0.4f;

float toLinear(int byte)
{
    const float value = static_cast<float>(byte) / 255.0f;
    return value <= 0.04045f ? value / 12.92f : powf((value + 0.055f) / 1.055f, 2.4f);
}

struct Image
{
    int width = 0;
    int height = 0;
    ct::Vector<std::uint8_t> pixels;

    int value(int x, int y) const
    {
        if (x < 0 || y < 0 || x >= width || y >= height) return 0;
        return pixels[(static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
                              static_cast<std::size_t>(x)) *
                      4];
    }

    int peak(int x, int y) const
    {
        int best = 0;
        for (int dy = -1; dy <= 1; ++dy)
            for (int dx = -1; dx <= 1; ++dx)
                if (value(x + dx, y + dy) > best) best = value(x + dx, y + dy);
        return best;
    }

    bool same(const Image& other) const
    {
        return width == other.width && height == other.height &&
               pixels.size() == other.pixels.size() &&
               memcmp(pixels.data(), other.pixels.data(), pixels.size()) == 0;
    }

    bool black() const
    {
        for (std::size_t i = 0; i < pixels.size(); i += 4)
            if (pixels[i] > 1 || pixels[i + 1] > 1 || pixels[i + 2] > 1) return false;
        return true;
    }
};

struct Pixel
{
    int x;
    int y;
};

Pixel project(const Math::Vec3& world, const Math::Mat4& cameraTransform)
{
    const Math::Vec3 local = cameraTransform.InverseAffine().TransformPoint(world);
    const float ndcX = local.x / -local.z / kTanHalfFov;
    const float ndcY = local.y / -local.z / kTanHalfFov;
    Pixel pixel;
    pixel.x = static_cast<int>((ndcX * 0.5f + 0.5f) * static_cast<float>(kSize));
    pixel.y = static_cast<int>((ndcY * 0.5f + 0.5f) * static_cast<float>(kSize));
    return pixel;
}

Pixel ghostOf(const Pixel& light, float offset)
{
    const float ndcX =
            (static_cast<float>(light.x) + 0.5f) / static_cast<float>(kSize) * 2.0f - 1.0f;
    const float ndcY =
            (static_cast<float>(light.y) + 0.5f) / static_cast<float>(kSize) * 2.0f - 1.0f;
    Pixel pixel;
    pixel.x = static_cast<int>(((ndcX * offset) * 0.5f + 0.5f) * static_cast<float>(kSize));
    pixel.y = static_cast<int>(((ndcY * offset) * 0.5f + 0.5f) * static_cast<float>(kSize));
    return pixel;
}

Math::Mat4 lightTowards(const Math::Vec3& toLight)
{
    const Math::Vec3 z = toLight.Normalized();
    const Math::Vec3 x = Math::Vec3(0.0f, 1.0f, 0.0f).Cross(z).Normalized();
    const Math::Vec3 y = z.Cross(x);
    Math::Mat4 matrix = Math::Mat4::Identity();
    matrix.col0 = Math::Vec4(x, 0.0f);
    matrix.col1 = Math::Vec4(y, 0.0f);
    matrix.col2 = Math::Vec4(z, 0.0f);
    return matrix;
}

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

TextureId discTexture(TextureStore& textures, const char* name)
{
    Pixmap pixmap(kFlareSize, kFlareSize, 4);
    for (int y = 0; y < kFlareSize; ++y)
        for (int x = 0; x < kFlareSize; ++x)
        {
            const float dx = static_cast<float>(x) + 0.5f - static_cast<float>(kFlareSize) * 0.5f;
            const float dy = static_cast<float>(y) + 0.5f - static_cast<float>(kFlareSize) * 0.5f;
            const bool inside = dx * dx + dy * dy <= static_cast<float>(kDiscRadius * kDiscRadius);
            pixmap.set_pixel(static_cast<std::uint32_t>(x), static_cast<std::uint32_t>(y), 255, 255,
                    255, inside ? 255 : 0);
        }
    return textures.createFromPixmap(pixmap, true, false, name);
}

struct Rig
{
    RenderingServer* server = nullptr;
    prisma::Driver* driver = nullptr;
    TextureStore* textures = nullptr;
    ScenarioId scenario;
    CameraId camera;
    ViewportId view;
    LightId sun;
    LightId lamp;
    InstanceId occluder;
    TextureId disc;
    TextureId disc2;
    RenderStats stats;

    void draw()
    {
        for (int i = 0; i < 2; ++i)
        {
            server->renderViewports();
            server->endFrame(true);
        }
        server->renderViewports();
        stats = server->getStats();
    }

    bool capture(Image& image)
    {
        draw();
        server->endFrame(true);
        image.width = kSize;
        image.height = kSize;
        image.pixels.resize(static_cast<std::size_t>(kSize) * kSize * 4);
        prisma::RenderTarget source;
        source.texture = textures->gpu(server->viewportGetTexture(view));
        prisma::Rect rect;
        rect.width = kSize;
        rect.height = kSize;
        return driver->readPixels(source, rect, image.pixels.data());
    }

    void placeOccluder(float shiftX)
    {
        const Math::Vec3 centre = kCameraPosition + kToSun * kOccluderDistance;
        server->instanceSetTransform(occluder,
                Math::Mat4::Translation(centre + Math::Vec3(shiftX, 0.0f, 0.0f)));
        server->instanceSetVisible(occluder, true);
    }
};

void testSunFlare(Rig& rig)
{
    const Math::Mat4 cameraTransform = Math::Mat4::Translation(kCameraPosition);
    const Pixel sunPixel = project(kCameraPosition + kToSun * 100.0f, cameraTransform);
    const Pixel ghostPixel = ghostOf(sunPixel, 0.55f);
    CHECK(sunPixel.x > 64 && sunPixel.x < 112 && sunPixel.y > 64 && sunPixel.y < 112);

    Image none;
    CHECK(rig.capture(none));
    CHECK(none.black());
    CHECK(rig.stats.lensFlaresDrawn == 0);

    const TextureId one[1] = { rig.disc };
    rig.server->lightSetLensFlare(rig.sun, one, 1);
    Image single;
    CHECK(rig.capture(single));
    CHECK(rig.stats.lensFlaresDrawn == 1);
    CHECK(single.peak(sunPixel.x, sunPixel.y) > 128);
    CHECK(single.value(sunPixel.x + 24, sunPixel.y) <= 1);
    CHECK(single.peak(ghostPixel.x, ghostPixel.y) <= 1);

    const TextureId two[2] = { rig.disc, rig.disc2 };
    rig.server->lightSetLensFlare(rig.sun, two, 2);
    Image pair;
    CHECK(rig.capture(pair));
    CHECK(rig.stats.lensFlaresDrawn == 2);
    const int ghost = pair.peak(ghostPixel.x, ghostPixel.y);
    CHECK(ghost > 100);
    CHECK(ghost < pair.peak(sunPixel.x, sunPixel.y));
    CHECK(pair.value(1, kSize - 2) <= 1);

    rig.placeOccluder(0.0f);
    Image occluded;
    CHECK(rig.capture(occluded));
    CHECK(occluded.peak(ghostPixel.x, ghostPixel.y) <= 1);
    rig.server->lightSetLensFlare(rig.sun, nullptr, 0);
    Image occluderOnly;
    CHECK(rig.capture(occluderOnly));
    CHECK(rig.stats.lensFlaresDrawn == 0);
    CHECK(occluded.same(occluderOnly));
    CHECK(occluderOnly.peak(sunPixel.x, sunPixel.y) > 1);

    rig.server->lightSetLensFlare(rig.sun, two, 2);
    rig.placeOccluder(kOccluderHalf);
    Image partial;
    CHECK(rig.capture(partial));
    const float ratio = toLinear(partial.peak(ghostPixel.x, ghostPixel.y)) / toLinear(ghost);
    CHECK(ratio > 0.25f && ratio < 0.7f);
    rig.server->instanceSetVisible(rig.occluder, false);

    rig.server->lightSetTransform(rig.sun, lightTowards(Math::Vec3(0.3f, -0.2f, -1.0f)));
    Image below;
    CHECK(rig.capture(below));
    CHECK(below.black());
    CHECK(rig.stats.lensFlaresDrawn == 0);

    rig.server->lightSetTransform(rig.sun, lightTowards(Math::Vec3(0.3f, 0.2f, 1.0f)));
    Image behind;
    CHECK(rig.capture(behind));
    CHECK(behind.black());

    rig.server->lightSetTransform(rig.sun, lightTowards(kToSun));
    rig.server->lightSetVisible(rig.sun, false);
    Image hidden;
    CHECK(rig.capture(hidden));
    CHECK(hidden.black());
    rig.server->lightSetVisible(rig.sun, true);
    rig.server->lightSetLensFlare(rig.sun, nullptr, 0);
}

void testLampFlare(Rig& rig)
{
    const Math::Vec3 lampPosition(-1.5f, 0.8f, 4.0f);
    const Math::Mat4 cameraTransform = Math::Mat4::Translation(kCameraPosition);
    const Pixel lampPixel = project(lampPosition, cameraTransform);
    rig.server->lightSetTransform(rig.lamp, Math::Mat4::Translation(lampPosition));
    rig.server->lightSetVisible(rig.lamp, true);
    const TextureId one[1] = { rig.disc };
    rig.server->lightSetLensFlare(rig.lamp, one, 1);

    Image lit;
    CHECK(rig.capture(lit));
    CHECK(rig.stats.lensFlaresDrawn == 1);
    CHECK(lit.peak(lampPixel.x, lampPixel.y) > 128);

    rig.server->cameraSetTransform(rig.camera,
            Math::Mat4::Translation(kCameraPosition) * Math::Mat4::RotationY(3.14159265f));
    Image away;
    CHECK(rig.capture(away));
    CHECK(away.black());
    CHECK(rig.stats.lensFlaresDrawn == 0);
    rig.server->cameraSetTransform(rig.camera, cameraTransform);
    rig.server->lightSetLensFlare(rig.lamp, nullptr, 0);
    rig.server->lightSetVisible(rig.lamp, false);
}

} // namespace

int main(int argc, char** argv)
{
    const bool vulkan = argc > 1 && strcmp(argv[1], "vulkan") == 0;

    CHECK(Device::headlessSupported());
    if (!Device::headlessSupported()) return 1;

    DeviceDesc desc;
    desc.title = "lens flare test";
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

    RenderingServer& server = *rig.server;
    MeshStore& meshes = *Main::meshes();
    rig.scenario = server.scenarioCreate();
    server.scenarioSetEnvironment(rig.scenario, whiteAmbient());

    rig.disc = discTexture(*rig.textures, "flare disc");
    rig.disc2 = discTexture(*rig.textures, "flare disc 2");
    CHECK(rig.disc.valid() && rig.disc2.valid());

    const MeshId quad = meshes.create("flare occluder");
    CHECK(meshes.addSurface(quad, quadData(kOccluderHalf)));
    rig.occluder = server.instanceCreate(quad, rig.scenario);
    server.instanceSetVisible(rig.occluder, false);

    rig.sun = server.lightCreate(LightType::Directional);
    server.lightSetScenario(rig.sun, rig.scenario);
    server.lightSetTransform(rig.sun, lightTowards(kToSun));
    rig.lamp = server.lightCreate(LightType::Omni);
    server.lightSetScenario(rig.lamp, rig.scenario);
    server.lightSetVisible(rig.lamp, false);

    rig.camera = server.cameraCreate();
    server.cameraSetPerspective(rig.camera, kFovDegrees, 0.1f, 100.0f);
    server.cameraSetTransform(rig.camera, Math::Mat4::Translation(kCameraPosition));

    rig.view = server.viewportCreate();
    server.viewportSetSize(rig.view, kSize, kSize);
    server.viewportAttachCamera(rig.view, rig.camera);
    server.viewportSetScenario(rig.view, rig.scenario);
    server.viewportSetClearColor(rig.view, Math::Vec4(0.0f, 0.0f, 0.0f, 1.0f));
    server.viewportSetUpdateMode(rig.view, ViewportUpdateMode::Always);
    CHECK(server.viewportGetTexture(rig.view).valid());

    testSunFlare(rig);
    testLampFlare(rig);

    Main::device()->requestClose();
    tree.finalize();
    Main::cleanup();

    if (failures) printf("%d check(s) failed\n", failures);
    else
        printf("all checks passed\n");
    return failures ? 1 : 0;
}
