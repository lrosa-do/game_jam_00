#include "Check.h"

#include "prisma3d/Device.h"
#include "prisma3d/core/Pixmap.h"
#include "prisma3d/render/AnimatedTexture.h"
#include "prisma3d/render/MaterialStore.h"
#include "prisma3d/render/MeshStore.h"
#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/render/TextureStore.h"

#include "platform.h"

#include <math.h>
#include <stdlib.h>
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

const Rgba kRed = { 255, 0, 0, 255 };
const Rgba kGreen = { 0, 255, 0, 255 };
const Rgba kBlue = { 0, 0, 255, 255 };
const Rgba kWhite = { 255, 255, 255, 255 };
const Rgba kMagenta = { 255, 0, 255, 255 };

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
    data.uvs.push_back(Math::Vec2(0.0f, 1.0f));
    data.uvs.push_back(Math::Vec2(1.0f, 1.0f));
    data.uvs.push_back(Math::Vec2(1.0f, 0.0f));
    data.uvs.push_back(Math::Vec2(0.0f, 0.0f));
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

struct Rig
{
    prisma::Driver* driver = nullptr;
    RenderingServer* server = nullptr;
    TextureStore* textures = nullptr;
    ViewportId viewport;
    Frames frames;

    bool texel(int x, int y, Rgba& out)
    {
        const TextureId output = server->viewportGetTexture(viewport);
        prisma::RenderTarget source;
        source.texture = textures->gpu(output);
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

    void render()
    {
        server->viewportSetUpdateMode(viewport, ViewportUpdateMode::Once);
        frames.draw(*server);
        frames.finish();
    }

    bool leftIs(Rgba expected, const char* label)
    {
        return is(8, 16, expected, label);
    }

    bool rightIs(Rgba expected, const char* label)
    {
        return is(24, 16, expected, label);
    }

    bool is(int x, int y, Rgba expected, const char* label)
    {
        Rgba got = { -1, -1, -1, -1 };
        const bool read = texel(x, y, got);
        const bool same = read && abs(got.r - expected.r) <= 2 && abs(got.g - expected.g) <= 2 &&
                          abs(got.b - expected.b) <= 2 && abs(got.a - expected.a) <= 2;
        if (!same)
            printf("%s: texel (%d,%d) is %d,%d,%d,%d, expected %d,%d,%d,%d\n", label, x, y, got.r,
                    got.g, got.b, got.a, expected.r, expected.g, expected.b, expected.a);
        return same;
    }

    bool both(Rgba expected, const char* label)
    {
        render();
        const bool left = leftIs(expected, label);
        const bool right = rightIs(expected, label);
        return left && right;
    }
};

TextureId solid(TextureStore& textures, const Rgba& color, const char* name)
{
    Pixmap pixels(4, 4, 4);
    for (std::uint32_t y = 0; y < 4; ++y)
        for (std::uint32_t x = 0; x < 4; ++x)
            pixels.set_pixel(x, y, static_cast<std::uint8_t>(color.r),
                    static_cast<std::uint8_t>(color.g), static_cast<std::uint8_t>(color.b), 255);
    return textures.createFromPixmap(pixels, true, false, name);
}

MaterialId unshadedMaterial(MaterialStore& materials, TextureId texture, const char* name)
{
    const MaterialId material = materials.create(name);
    materials.setShading(material, MaterialShading::Unshaded);
    materials.setTexture(material, MaterialTexture::Albedo, texture);
    return material;
}

void run(prisma::Driver* driver, bool vulkan)
{
    (void) vulkan;
    MeshStore meshes(driver);
    TextureStore textures(driver);
    MaterialStore materials(textures);
    RenderingServer server(driver, meshes, textures, materials);

    const MeshId quad = meshes.create("quad");
    CHECK(meshes.addSurface(quad, quadData(0.5f)));

    const ScenarioId scenario = server.scenarioCreate();
    const InstanceId left = server.instanceCreate(quad, scenario);
    server.instanceSetTransform(left, Math::Mat4::Translation(Math::Vec3(-1.0f, 0.0f, 0.0f)));
    const InstanceId right = server.instanceCreate(quad, scenario);
    server.instanceSetTransform(right, Math::Mat4::Translation(Math::Vec3(1.0f, 0.0f, 0.0f)));
    const CameraId camera = server.cameraCreate();
    server.cameraSetOrthogonal(camera, 4.0f, 0.1f, 20.0f);
    server.cameraSetTransform(camera, Math::Mat4::Translation(Math::Vec3(0.0f, 0.0f, 3.0f)));

    Rig rig;
    rig.driver = driver;
    rig.server = &server;
    rig.textures = &textures;
    rig.viewport = server.viewportCreate();
    server.viewportSetSize(rig.viewport, kSize, kSize);
    server.viewportAttachCamera(rig.viewport, camera);
    server.viewportSetScenario(rig.viewport, scenario);

    rig.render();
    textures.placeholder();
    const std::size_t baseline = textures.count();

    const TextureId red = solid(textures, kRed, "red");
    const TextureId green = solid(textures, kGreen, "green");
    const TextureId blue = solid(textures, kBlue, "blue");
    CHECK(red.valid() && green.valid() && blue.valid());
    CHECK(textures.count() == baseline + 3);

    AnimatedTexture* animated = new AnimatedTexture(&textures);
    animated->setAutoUpdate(false);
    const TextureId proxy = animated->getTexture();
    CHECK(proxy.valid());
    CHECK(textures.isProxy(proxy));
    CHECK(textures.count() == baseline + 4);
    CHECK(animated->setFrames(3));
    const TextureId frames[3] = { red, green, blue };
    for (int i = 0; i < 3; ++i)
    {
        CHECK(animated->setFrameTexture(i, frames[i]));
        CHECK(animated->setFrameDuration(i, 0.5f));
    }
    CHECK(!animated->setFrameTexture(0, proxy));
    CHECK(animated->getFrameTexture(1) == green);

    const MaterialId materialLeft = unshadedMaterial(materials, proxy, "left");
    const MaterialId materialRight = unshadedMaterial(materials, proxy, "right");
    server.instanceSetMaterialOverride(left, materialLeft);
    server.instanceSetMaterialOverride(right, materialRight);

    CHECK(rig.both(kMagenta, "before the first update the proxy shows the placeholder"));
    const std::uint32_t pipelines = server.scenePipelineCount();
    CHECK(pipelines > 0);

    animated->update(0.0f);
    CHECK(rig.both(kRed, "frame 0"));
    animated->update(0.5f);
    CHECK(animated->getCurrentFrame() == 0);
    CHECK(rig.both(kRed, "frame 0 at the exact duration"));
    animated->update(0.25f);
    CHECK(animated->getCurrentFrame() == 1);
    CHECK(rig.both(kGreen, "frame 1"));
    animated->update(0.25f);
    CHECK(animated->getCurrentFrame() == 1);
    CHECK(rig.both(kGreen, "frame 1 again"));
    animated->update(0.25f);
    CHECK(animated->getCurrentFrame() == 2);
    CHECK(rig.both(kBlue, "frame 2"));
    animated->update(0.5f);
    CHECK(animated->getCurrentFrame() == 0);
    CHECK(rig.both(kRed, "wrapped to frame 0"));
    animated->setPause(true);
    animated->update(5.0f);
    CHECK(rig.both(kRed, "paused"));
    animated->setPause(false);
    CHECK(animated->setCurrentFrame(2));
    animated->update(0.0f);
    CHECK(rig.both(kBlue, "jump to frame 2"));
    CHECK(server.scenePipelineCount() == pipelines);

    CHECK(textures.get(proxy)->width == 4);
    CHECK(textures.get(materials.get(materialLeft)->textures[0]) != nullptr);
    CHECK(materials.get(materialLeft)->version == materials.get(materialRight)->version);

    const std::uint32_t versionBefore = materials.get(materialLeft)->version;
    animated->update(1.0f);
    CHECK(rig.both(kRed, "advanced without touching the material"));
    CHECK(materials.get(materialLeft)->version == versionBefore);
    CHECK(materials.get(materialLeft)->textures[0] == proxy);
    CHECK(server.scenePipelineCount() == pipelines);

    CHECK(animated->setCurrentFrame(1));
    animated->update(0.0f);
    CHECK(rig.both(kGreen, "on the frame about to be destroyed"));
    textures.destroy(green);
    CHECK(!textures.valid(proxy));
    CHECK(rig.both(kWhite, "the frame texture was destroyed"));
    animated->update(0.0f);
    CHECK(rig.both(kWhite, "a destroyed frame does not update the proxy"));
    CHECK(animated->setCurrentFrame(2));
    animated->update(0.0f);
    CHECK(textures.valid(proxy));
    CHECK(rig.both(kBlue, "recovered on a valid frame"));

    delete animated;
    animated = nullptr;
    CHECK(!textures.valid(proxy));
    CHECK(textures.get(proxy) == nullptr);
    CHECK(textures.valid(red) && textures.valid(blue));
    CHECK(rig.both(kWhite, "the animated texture was destroyed"));
    CHECK(textures.count() == baseline + 2);

    {
        AnimatedTexture automatic(&textures);
        CHECK(automatic.getAutoUpdate());
        CHECK(automatic.setFrames(2));
        CHECK(automatic.setFrameTexture(0, red));
        CHECK(automatic.setFrameTexture(1, blue));
        CHECK(automatic.setFrameDuration(0, 0.01f));
        CHECK(automatic.setFrameDuration(1, 100.0f));
        materials.setTexture(materialLeft, MaterialTexture::Albedo, automatic.getTexture());
        materials.setTexture(materialRight, MaterialTexture::Albedo, automatic.getTexture());
        CHECK(rig.both(kRed, "the first frame runs the clock from zero"));
        CHECK(automatic.getCurrentFrame() == 0);
        const std::uint64_t start = time_nanos();
        while (time_nanos() - start < 60000000ull)
        {
        }
        CHECK(rig.both(kBlue, "the clock moved to frame 1 before drawing"));
        CHECK(automatic.getCurrentFrame() == 1);
        automatic.setAutoUpdate(false);
        automatic.setCurrentFrame(0);
        automatic.update(0.0f);
        const std::uint64_t again = time_nanos();
        while (time_nanos() - again < 30000000ull)
        {
        }
        CHECK(rig.both(kRed, "no clock update once disconnected"));
        CHECK(automatic.getCurrentFrame() == 0);
    }
    CHECK(rig.both(kWhite, "after the automatic texture was destroyed"));
    rig.render();

    materials.setTexture(materialLeft, MaterialTexture::Albedo, TextureId());
    materials.setTexture(materialRight, MaterialTexture::Albedo, TextureId());
    server.instanceSetMaterialOverride(left, MaterialId());
    server.instanceSetMaterialOverride(right, MaterialId());
    materials.destroy(materialLeft);
    materials.destroy(materialRight);
    textures.destroy(red);
    textures.destroy(blue);
    CHECK(textures.count() == baseline);

    server.viewportFree(rig.viewport);
    server.cameraFree(camera);
    server.instanceFree(left);
    server.instanceFree(right);
    server.scenarioFree(scenario);
}

} // namespace

int main(int argc, char** argv)
{
    const bool vulkan = argc > 1 && strcmp(argv[1], "vulkan") == 0;

    CHECK(Device::headlessSupported());
    if (!Device::headlessSupported()) return 1;

    DeviceDesc deviceDesc;
    deviceDesc.title = "animated texture render test";
    deviceDesc.driver = vulkan ? prisma::DriverType::Vulkan : prisma::DriverType::OpenGL;
    deviceDesc.width = kWidth;
    deviceDesc.height = kHeight;
    deviceDesc.headless = true;

    Device* device = Device::create(deviceDesc);
    CHECK(device != nullptr);
    if (!device) return 1;

    run(device->driver(), vulkan);

    Device::destroy(device);
    if (failures == 0) printf("animated texture render test: all passed\n");
    return failures == 0 ? 0 : 1;
}
