#include "Check.h"
#include "LitTest.h"

#include "prisma3d/Device.h"
#include "prisma3d/render/MaterialStore.h"
#include "prisma3d/render/MeshStore.h"
#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/render/TextureStore.h"

#include <math.h>
#include <string.h>

using namespace prisma3d;

namespace
{

const int kWidth = 64;
const int kHeight = 48;
const std::uint32_t kSize = 32;

struct Rgba
{
    int r;
    int g;
    int b;
    int a;
};

bool readTexel(prisma::Driver* driver, prisma::TextureHandle texture, int x, int y, Rgba& out)
{
    prisma::RenderTarget source;
    source.texture = texture;
    prisma::Rect area;
    area.x = x;
    area.y = y;
    area.width = 1;
    area.height = 1;
    unsigned char pixel[4] = { 0, 0, 0, 0 };
    if (!driver->readPixels(source, area, pixel)) return false;
    out = Rgba{ pixel[0], pixel[1], pixel[2], pixel[3] };
    return true;
}

bool texelIs(prisma::Driver* driver, TextureStore& textures, TextureId texture, int x, int y,
        Rgba expected, int tolerance, const char* label)
{
    Rgba texel = { -1, -1, -1, -1 };
    const bool read = readTexel(driver, textures.gpu(texture), x, y, texel);
    const bool same = read && abs(texel.r - expected.r) <= tolerance &&
                      abs(texel.g - expected.g) <= tolerance &&
                      abs(texel.b - expected.b) <= tolerance &&
                      abs(texel.a - expected.a) <= tolerance;
    if (!same)
        printf("%s: texel (%d,%d) is %d,%d,%d,%d, expected %d,%d,%d,%d\n", label, x, y, texel.r,
                texel.g, texel.b, texel.a, expected.r, expected.g, expected.b, expected.a);
    return same;
}

float toSrgb(float value)
{
    return value <= 0.0031308f ? value * 12.92f : 1.055f * powf(value, 1.0f / 2.4f) - 0.055f;
}

float toLinear(float value)
{
    return value <= 0.04045f ? value / 12.92f : powf((value + 0.055f) / 1.055f, 2.4f);
}

int byteOf(float linear)
{
    return static_cast<int>(toSrgb(linear) * 255.0f + 0.5f);
}

SurfaceData quadData(float half, const Math::Vec4& top, const Math::Vec4& bottom, bool upright)
{
    SurfaceData data;
    data.positions.push_back(Math::Vec3(-half, -half, 0.0f));
    data.positions.push_back(Math::Vec3(half, -half, 0.0f));
    data.positions.push_back(Math::Vec3(half, half, 0.0f));
    data.positions.push_back(Math::Vec3(-half, half, 0.0f));
    data.colors.push_back(bottom);
    data.colors.push_back(bottom);
    data.colors.push_back(top);
    data.colors.push_back(top);
    for (int i = 0; i < 4; ++i) data.normals.push_back(Math::Vec3(0.0f, 0.0f, 1.0f));
    const float low = upright ? 0.0f : 1.0f;
    const float high = 1.0f - low;
    data.uvs.push_back(Math::Vec2(0.0f, low));
    data.uvs.push_back(Math::Vec2(1.0f, low));
    data.uvs.push_back(Math::Vec2(1.0f, high));
    data.uvs.push_back(Math::Vec2(0.0f, high));
    const std::uint32_t indices[6] = { 0, 1, 2, 0, 2, 3 };
    for (std::uint32_t index: indices) data.indices.push_back(index);
    return data;
}

struct Frames
{
    RenderingServer* server = nullptr;
    bool open = false;

    void finish()
    {
        if (!open || !server) return;
        server->endFrame(true);
        open = false;
    }

    void draw(RenderingServer& renderer)
    {
        finish();
        server = &renderer;
        server->renderViewports();
        open = true;
    }
};

} // namespace

int main(int argc, char** argv)
{
    const bool vulkan = argc > 1 && strcmp(argv[1], "vulkan") == 0;

    CHECK(Device::headlessSupported());
    if (!Device::headlessSupported()) return 1;

    DeviceDesc deviceDesc;
    deviceDesc.title = "viewport texture test";
    deviceDesc.driver = vulkan ? prisma::DriverType::Vulkan : prisma::DriverType::OpenGL;
    deviceDesc.width = kWidth;
    deviceDesc.height = kHeight;
    deviceDesc.headless = true;

    Device* device = Device::create(deviceDesc);
    CHECK(device != nullptr);
    if (!device) return 1;
    prisma::Driver* driver = device->driver();

    {
        MeshStore meshes(driver);
        TextureStore textures(driver);
        MaterialStore materials(textures);
        RenderingServer server(driver, meshes, textures, materials);
        Frames frames;

        const Math::Vec4 red(1.0f, 0.0f, 0.0f, 1.0f);
        const Math::Vec4 blue(0.0f, 0.0f, 1.0f, 1.0f);
        const Math::Vec4 white(1.0f, 1.0f, 1.0f, 1.0f);
        const MeshId quad = meshes.create("quad");
        CHECK(meshes.addSurface(quad, quadData(1.0f, red, blue, true)));

        const ScenarioId scenario = server.scenarioCreate();
        const InstanceId instance = server.instanceCreate(quad, scenario);
        const CameraId camera = server.cameraCreate();
        server.cameraSetOrthogonal(camera, 4.0f, 0.1f, 20.0f);
        server.cameraSetTransform(camera, Math::Mat4::Translation(Math::Vec3(0.0f, 0.0f, 3.0f)));

        const ViewportId viewport = server.viewportCreate();
        CHECK(!server.viewportGetTexture(viewport).valid());
        CHECK(!server.viewportRender(viewport));
        server.viewportSetSize(viewport, kSize, kSize);
        CHECK(!server.viewportRender(viewport));
        server.viewportAttachCamera(viewport, camera);
        server.viewportSetScenario(viewport, scenario);
        server.viewportSetTransparentBackground(viewport, true);
        CHECK(server.viewportGetTransparentBackground(viewport));
        CHECK(server.viewportGetUpdateMode(viewport) == ViewportUpdateMode::Disabled);
        CHECK(server.viewportGetDebugDraw(viewport) == ViewportDebugDraw::Disabled);

        TextureId texture = server.viewportGetTexture(viewport);
        CHECK(texture.valid());
        CHECK(server.viewportGetTexture(viewport) == texture);
        {
            const TextureInfo* info = textures.get(texture);
            CHECK(info != nullptr && info->width == kSize && info->height == kSize);
            CHECK(info != nullptr && info->format == prisma::TextureFormat::RGBA8Srgb &&
                    info->srgb);
        }

        frames.draw(server);
        CHECK(server.getStats().viewportsDrawn == 0);

        server.viewportSetDebugDraw(viewport, ViewportDebugDraw::Unshaded);
        server.viewportSetUpdateMode(viewport, ViewportUpdateMode::Once);
        frames.draw(server);
        CHECK(server.getStats().viewportsDrawn == 1);
        CHECK(server.getStats().drawCalls == 1);
        CHECK(server.getStats().triangles == 2);
        CHECK(server.viewportGetUpdateMode(viewport) == ViewportUpdateMode::Disabled);
        CHECK(server.viewportGetTexture(viewport) == texture);
        frames.finish();
        const Rgba clear = { 0, 0, 0, 0 };
        CHECK(texelIs(driver, textures, texture, 16, 4, clear, 0, "transparent below"));
        CHECK(texelIs(driver, textures, texture, 16, 28, clear, 0, "transparent above"));
        CHECK(texelIs(driver, textures, texture, 4, 16, clear, 0, "transparent left"));
        CHECK(texelIs(driver, textures, texture, 28, 16, clear, 0, "transparent right"));
        CHECK(texelIs(driver, textures, texture, 7, 16, clear, 0, "just outside the left edge"));
        CHECK(texelIs(driver, textures, texture, 24, 16, clear, 0, "just outside the right edge"));
        for (int row = 10; row <= 22; row += 4)
        {
            const float y = (static_cast<float>(row) + 0.5f - 16.0f) / 8.0f;
            const float weight = (y + 1.0f) * 0.5f;
            const Rgba expected = { byteOf(weight), 0, byteOf(1.0f - weight), 255 };
            char label[48];
            snprintf(label, sizeof(label), "gradient, row %d from the bottom", row);
            CHECK(texelIs(driver, textures, texture, 16, row, expected, 2, label));
        }

        frames.draw(server);
        CHECK(server.getStats().viewportsDrawn == 0);
        frames.finish();

        server.viewportSetUpdateMode(viewport, ViewportUpdateMode::Disabled);
        server.instanceSetColor(instance, Math::Vec4(0.5f, 0.5f, 0.5f, 1.0f));
        frames.draw(server);
        frames.finish();
        const float centre = (0.0625f + 1.0f) * 0.5f;
        const Rgba kept = { byteOf(centre), 0, byteOf(1.0f - centre), 255 };
        CHECK(texelIs(driver, textures, texture, 16, 16, kept, 2, "disabled viewport keeps"));
        CHECK(server.viewportRender(viewport));
        frames.finish();
        const float tint = toLinear(0.5f);
        const Rgba dimmer = { byteOf(centre * tint), 0, byteOf((1.0f - centre) * tint), 255 };
        CHECK(texelIs(driver, textures, texture, 16, 16, dimmer, 2, "forced render"));
        CHECK(server.getStats().viewportsDrawn == 0);

        server.instanceSetColor(instance, white);
        server.viewportSetUpdateMode(viewport, ViewportUpdateMode::Always);
        for (int i = 0; i < 3; ++i)
        {
            frames.draw(server);
            CHECK(server.getStats().viewportsDrawn == 1);
            CHECK(server.viewportGetUpdateMode(viewport) == ViewportUpdateMode::Always);
        }
        frames.finish();
        server.viewportSetActive(viewport, false);
        frames.draw(server);
        CHECK(server.getStats().viewportsDrawn == 0);
        frames.finish();
        server.viewportSetActive(viewport, true);
        server.viewportSetUpdateMode(viewport, ViewportUpdateMode::Disabled);

        {
            const MaterialId grey = materials.create("grey");
            materials.setColor(grey, MaterialParam::Albedo, Math::Vec4(0.5f, 0.5f, 0.5f, 1.0f));
            const MeshId flat = meshes.create("flat");
            CHECK(meshes.addSurface(flat, quadData(1.0f, white, white, true)));
            const InstanceId panel = server.instanceCreate(flat, scenario);
            server.instanceSetMaterialOverride(panel, grey);
            server.instanceSetVisible(instance, false);
            const LightId sun = server.lightCreate(LightType::Directional);
            server.lightSetScenario(sun, scenario);
            server.lightSetTransform(sun, Math::Mat4::RotationX(-0.7f));
            server.lightSetShadow(sun, true);

            server.viewportSetDebugDraw(viewport, ViewportDebugDraw::Disabled);
            server.viewportSetUpdateMode(viewport, ViewportUpdateMode::Once);
            frames.draw(server);
            frames.finish();
            Rgba lit = {};
            CHECK(readTexel(driver, textures.gpu(texture), 16, 16, lit));
            CHECK(abs(lit.r - 128) > 10);
            CHECK(server.getStats().shadowPasses > 0);

            server.viewportSetDebugDraw(viewport, ViewportDebugDraw::Unshaded);
            server.viewportSetUpdateMode(viewport, ViewportUpdateMode::Once);
            frames.draw(server);
            CHECK(server.getStats().lightsDrawn == 0);
            CHECK(server.getStats().shadowPasses == 0);
            frames.finish();
            CHECK(texelIs(driver, textures, texture, 16, 16, Rgba{ 128, 128, 128, 255 }, 2,
                    "unshaded ignores the light"));

            server.viewportSetDebugDraw(viewport, ViewportDebugDraw::NormalBuffer);
            const TextureId normals = server.viewportGetTexture(viewport);
            CHECK(normals.valid() && normals != texture);
            texture = normals;
            {
                const TextureInfo* info = textures.get(texture);
                CHECK(info != nullptr && info->format == prisma::TextureFormat::RGBA8 &&
                        !info->srgb);
            }
            server.viewportSetUpdateMode(viewport, ViewportUpdateMode::Once);
            frames.draw(server);
            frames.finish();
            CHECK(texelIs(driver, textures, texture, 16, 16, Rgba{ 128, 128, 255, 255 }, 2,
                    "normal of a quad facing the camera"));
            CHECK(texelIs(driver, textures, texture, 4, 16, clear, 0, "normal buffer, outside"));

            const float turn = 0.78539816f;
            server.instanceSetTransform(panel, Math::Mat4::RotationY(turn));
            server.viewportSetUpdateMode(viewport, ViewportUpdateMode::Once);
            frames.draw(server);
            frames.finish();
            const int along = static_cast<int>((0.5f + 0.5f * sinf(turn)) * 255.0f + 0.5f);
            const int toward = static_cast<int>((0.5f + 0.5f * cosf(turn)) * 255.0f + 0.5f);
            CHECK(texelIs(driver, textures, texture, 16, 16, Rgba{ along, 128, toward, 255 }, 2,
                    "normal of a quad turned 45 degrees about Y"));

            server.instanceSetTransform(panel, Math::Mat4::RotationX(-turn));
            server.viewportSetUpdateMode(viewport, ViewportUpdateMode::Once);
            frames.draw(server);
            frames.finish();
            const int up = static_cast<int>((0.5f + 0.5f * sinf(turn)) * 255.0f + 0.5f);
            CHECK(texelIs(driver, textures, texture, 16, 16, Rgba{ 128, up, toward, 255 }, 2,
                    "normal of a quad tipped 45 degrees about X"));

            server.viewportSetDebugDraw(viewport, ViewportDebugDraw::Unshaded);
            texture = server.viewportGetTexture(viewport);
            server.instanceSetTransform(panel, Math::Mat4::Identity());
            server.viewportSetUpdateMode(viewport, ViewportUpdateMode::Once);
            frames.draw(server);
            frames.finish();
            CHECK(texelIs(driver, textures, texture, 16, 16, Rgba{ 128, 128, 128, 255 }, 2,
                    "unshaded after the normals"));

            server.instanceFree(panel);
            server.lightFree(sun);
            server.instanceSetVisible(instance, true);
        }

        {
            const ScenarioId view = server.scenarioCreate();
            server.scenarioSetEnvironment(view, whiteAmbient());
            const MeshId screenQuad = meshes.create("screen quad");
            CHECK(meshes.addSurface(screenQuad, quadData(1.0f, white, white, true)));
            const MaterialId material = materials.create("textured");
            materials.setTexture(material, MaterialTexture::Albedo, texture);
            const InstanceId screenInstance = server.instanceCreate(screenQuad, view);
            server.instanceSetMaterialOverride(screenInstance, material);
            const CameraId screenCamera = server.cameraCreate();
            server.cameraSetOrthogonal(screenCamera, 4.0f, 0.1f, 20.0f);
            server.cameraSetTransform(screenCamera,
                    Math::Mat4::Translation(Math::Vec3(0.0f, 0.0f, 3.0f)));
            const ViewportId screen = server.viewportCreate();
            server.viewportSetSize(screen, kWidth, kHeight);
            server.viewportAttachToScreen(screen, true);
            server.viewportAttachCamera(screen, screenCamera);
            server.viewportSetScenario(screen, view);

            server.viewportSetDebugDraw(viewport, ViewportDebugDraw::Unshaded);
            server.instanceSetColor(instance, white);
            server.viewportSetUpdateMode(viewport, ViewportUpdateMode::Always);
            frames.draw(server);
            CHECK(server.getStats().viewportsDrawn == 2);
            const int rows[3] = { 22, 26, 30 };
            for (int i = 0; i < 3; ++i)
            {
                const float fraction = (36.0f - (static_cast<float>(rows[i]) + 0.5f)) / 24.0f;
                const float t = fraction * 32.0f - 0.5f;
                const int lower = static_cast<int>(floorf(t));
                const float mix = t - static_cast<float>(lower);
                float weights[2] = { 0.0f, 0.0f };
                for (int k = 0; k < 2; ++k)
                {
                    const float y = (static_cast<float>(lower + k) + 0.5f - 16.0f) / 8.0f;
                    weights[k] = (lower + k >= 8 && lower + k < 24) ? (y + 1.0f) * 0.5f : 0.0f;
                }
                const float weight = weights[0] * (1.0f - mix) + weights[1] * mix;
                unsigned char pixel[4] = {};
                prisma::Rect rect;
                rect.x = kWidth / 2;
                rect.y = rows[i];
                rect.width = 1;
                rect.height = 1;
                CHECK(driver->readPixels(prisma::RenderTarget(), rect, pixel));
                const bool inside = weight > 0.0f;
                const int red = inside ? byteOf(weight) : 0;
                const int blue = inside ? byteOf(1.0f - weight) : 0;
                const bool same =
                        abs(pixel[0] - red) <= 3 && pixel[1] == 0 && abs(pixel[2] - blue) <= 3;
                if (!same)
                    printf("screen row %d: %d,%d,%d expected %d,0,%d\n", rows[i], pixel[0],
                            pixel[1], pixel[2], red, blue);
                CHECK(same);
            }
            frames.finish();

            server.viewportSetUpdateMode(viewport, ViewportUpdateMode::Disabled);
            server.viewportFree(viewport);
            CHECK(!textures.valid(texture));
            frames.draw(server);
            CHECK(server.getStats().viewportsDrawn == 1);
            frames.finish();
            server.instanceFree(screenInstance);
            server.viewportFree(screen);
            server.cameraFree(screenCamera);
            server.scenarioFree(view);
        }

        server.viewportFree(ViewportId());
        server.viewportRender(ViewportId());
        CHECK(!server.viewportGetTexture(ViewportId()).valid());
        CHECK(server.viewportGetUpdateMode(ViewportId()) == ViewportUpdateMode::Disabled);
    }

    Device::destroy(device);
    if (failures == 0) printf("viewport texture test: all passed\n");
    return failures == 0 ? 0 : 1;
}
