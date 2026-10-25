#include "Check.h"
#include "LitTest.h"

#include "prisma3d/Device.h"
#include "prisma3d/core/Pixmap.h"
#include "prisma3d/core/Random.h"
#include "prisma3d/render/MaterialStore.h"
#include "prisma3d/render/MeshStore.h"
#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/render/TextureStore.h"
#include "prisma3d/scene/TreeGenerator.h"

#include <math.h>
#include <string.h>

using namespace prisma3d;

namespace
{

const int kWidth = 64;
const int kHeight = 48;
const int kTolerance = 2;
const float kUnit = 13.8564f;

struct Rgb
{
    int r;
    int g;
    int b;
};

const Rgb kClear = { 26, 51, 77 };
const Rgb kRed = { 255, 0, 0 };
const Rgb kGreen = { 0, 255, 0 };
const Rgb kBlue = { 0, 0, 255 };
const Rgb kWhite = { 255, 255, 255 };
const Rgb kBlack = { 0, 0, 0 };

Rgb readPixel(prisma::Driver* driver, int x, int y)
{
    unsigned char pixel[4] = { 0, 0, 0, 0 };
    prisma::Rect rect;
    rect.x = x;
    rect.y = y;
    rect.width = 1;
    rect.height = 1;
    driver->readPixels(prisma::RenderTarget(), rect, pixel);
    return Rgb{ pixel[0], pixel[1], pixel[2] };
}

bool pixelWithin(prisma::Driver* driver, int x, int y, Rgb expected, int tolerance,
        const char* label)
{
    const Rgb pixel = readPixel(driver, x, y);
    const int dr = pixel.r - expected.r;
    const int dg = pixel.g - expected.g;
    const int db = pixel.b - expected.b;
    const bool same = dr >= -tolerance && dr <= tolerance && dg >= -tolerance && dg <= tolerance &&
                      db >= -tolerance && db <= tolerance;
    if (!same)
        printf("%s: pixel (%d,%d) is %d,%d,%d, expected %d,%d,%d\n", label, x, y, pixel.r, pixel.g,
                pixel.b, expected.r, expected.g, expected.b);
    return same;
}

bool pixelIs(prisma::Driver* driver, int x, int y, Rgb expected, const char* label)
{
    return pixelWithin(driver, x, y, expected, kTolerance, label);
}

int screenX(float x)
{
    return static_cast<int>(32.0f + x * kUnit);
}

int screenY(float y)
{
    return static_cast<int>(24.0f - y * kUnit);
}

SurfaceData quadData(float z, float half, const Math::Vec4& color, bool normals)
{
    SurfaceData data;
    data.positions.push_back(Math::Vec3(-half, -half, z));
    data.positions.push_back(Math::Vec3(half, -half, z));
    data.positions.push_back(Math::Vec3(half, half, z));
    data.positions.push_back(Math::Vec3(-half, half, z));
    for (int i = 0; i < 4; ++i) data.colors.push_back(color);
    if (normals)
        for (int i = 0; i < 4; ++i) data.normals.push_back(Math::Vec3(0.0f, 0.0f, 1.0f));
    const std::uint32_t indices[6] = { 0, 1, 2, 0, 2, 3 };
    for (std::uint32_t index: indices) data.indices.push_back(index);
    return data;
}

struct Frames
{
    RenderingServer* server;
    bool open;

    Frames()
            : server(nullptr),
              open(false)
    {
    }

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

Math::Mat4 translation(float x, float y, float z)
{
    return Math::Mat4::Translation(Math::Vec3(x, y, z));
}

Rgb grey(float linear)
{
    const float encoded =
            linear <= 0.0031308f ? linear * 12.92f : 1.055f * powf(linear, 1.0f / 2.4f) - 0.055f;
    const int value = static_cast<int>(encoded * 255.0f + 0.5f);
    return Rgb{ value, value, value };
}

bool near(float a, float b)
{
    return fabsf(a - b) < 0.0001f;
}

bool boxIs(const Math::Box& box, float x0, float y0, float z0, float x1, float y1, float z1)
{
    return near(box.min.x, x0) && near(box.min.y, y0) && near(box.min.z, z0) &&
           near(box.max.x, x1) && near(box.max.y, y1) && near(box.max.z, z1);
}

} // namespace

int main(int argc, char** argv)
{
    const bool vulkan = argc > 1 && strcmp(argv[1], "vulkan") == 0;

    CHECK(Device::headlessSupported());
    if (!Device::headlessSupported()) return 1;

    DeviceDesc deviceDesc;
    deviceDesc.title = "multimesh test";
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

        const Math::Vec4 white(1.0f, 1.0f, 1.0f, 1.0f);
        const MeshId quad = meshes.create("quad");
        CHECK(meshes.addSurface(quad, quadData(0.0f, 0.5f, white, false)));

        const ScenarioId scenario = server.scenarioCreate();
        server.scenarioSetEnvironment(scenario, whiteAmbient());
        const CameraId camera = server.cameraCreate();
        server.cameraSetPerspective(camera, 60.0f, 0.1f, 100.0f);
        server.cameraSetTransform(camera, translation(0.0f, 0.0f, 3.0f));
        const ViewportId viewport = server.viewportCreate();
        server.viewportSetSize(viewport, kWidth, kHeight);
        server.viewportAttachToScreen(viewport, true);
        server.viewportAttachCamera(viewport, camera);
        server.viewportSetScenario(viewport, scenario);
        server.viewportSetClearColor(viewport, Math::Vec4(0.1f, 0.2f, 0.3f, 1.0f));

        const MultiMeshId invalid;
        CHECK(!server.multimeshValid(invalid));
        server.multimeshAllocateData(invalid, 4, true, false);
        server.multimeshSetMesh(invalid, quad);
        server.multimeshInstanceSetTransform(invalid, 0, Math::Mat4::Identity());
        server.multimeshSetVisibleInstances(invalid, 1);
        server.multimeshFree(invalid);
        CHECK(server.multimeshGetInstanceCount(invalid) == 0);
        CHECK(!server.multimeshGetAabb(invalid).Contains(Math::Vec3(0.0f, 0.0f, 0.0f)));

        const MultiMeshId mm = server.multimeshCreate();
        CHECK(mm.valid() && server.multimeshValid(mm));
        CHECK(server.multimeshGetInstanceCount(mm) == 0);
        CHECK(server.multimeshGetVisibleInstances(mm) == -1);
        server.multimeshAllocateData(mm, 3, true, false);
        CHECK(server.multimeshGetInstanceCount(mm) == 3);
        server.multimeshSetMesh(mm, quad);
        CHECK(server.multimeshGetMesh(mm) == quad);

        server.multimeshInstanceSetTransform(mm, 0, translation(-1.5f, 0.0f, 0.0f));
        server.multimeshInstanceSetTransform(mm, 1, translation(0.0f, 0.0f, 0.0f));
        server.multimeshInstanceSetTransform(mm, 2, translation(1.5f, 1.0f, 0.0f));
        server.multimeshInstanceSetColor(mm, 0, Math::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
        server.multimeshInstanceSetColor(mm, 1, Math::Vec4(0.0f, 1.0f, 0.0f, 1.0f));
        server.multimeshInstanceSetColor(mm, 2, Math::Vec4(0.0f, 0.0f, 1.0f, 1.0f));
        server.multimeshInstanceSetCustomData(mm, 0, Math::Vec4(1.0f, 2.0f, 3.0f, 4.0f));
        server.multimeshInstanceSetTransform(mm, 3, translation(9.0f, 9.0f, 9.0f));
        server.multimeshInstanceSetColor(mm, 99, white);

        {
            ct::Vector<float> buffer;
            CHECK(server.multimeshGetBuffer(mm, buffer));
            CHECK(buffer.size() == 48);
            if (buffer.size() == 48)
            {
                CHECK(buffer[0] == 1.0f && buffer[1] == 0.0f && buffer[2] == 0.0f &&
                        buffer[3] == -1.5f);
                CHECK(buffer[4] == 0.0f && buffer[5] == 1.0f && buffer[7] == 0.0f);
                CHECK(buffer[8] == 0.0f && buffer[10] == 1.0f && buffer[11] == 0.0f);
                CHECK(buffer[12] == 1.0f && buffer[13] == 0.0f && buffer[15] == 1.0f);
                CHECK(buffer[16 + 3] == 0.0f && buffer[16 + 13] == 1.0f);
                CHECK(buffer[32 + 3] == 1.5f && buffer[32 + 7] == 1.0f && buffer[32 + 15] == 1.0f);
            }
        }
        CHECK(near(server.multimeshInstanceGetTransform(mm, 2).GetTranslation().x, 1.5f));
        CHECK(near(server.multimeshInstanceGetColor(mm, 1).y, 1.0f));
        CHECK(server.multimeshInstanceGetCustomData(mm, 0).x == 0.0f);

        const InstanceId instance = server.instanceCreate();
        server.instanceSetBase(instance, mm);
        server.instanceSetScenario(instance, scenario);

        frames.draw(server);
        {
            const RenderStats& stats = server.getStats();
            CHECK(stats.viewportsDrawn == 1);
            CHECK(stats.instancesDrawn == 1);
            CHECK(stats.multimeshInstancesDrawn == 3);
            CHECK(stats.surfacesDrawn == 1);
            CHECK(stats.drawCalls == 1);
            CHECK(stats.triangles == 6);
            CHECK(stats.multimeshUploadBytes == 192);
        }
        CHECK(pixelIs(driver, screenX(-1.5f), screenY(0.0f), kRed, "instance 0"));
        CHECK(pixelIs(driver, screenX(0.0f), screenY(0.0f), kGreen, "instance 1"));
        CHECK(pixelIs(driver, screenX(1.5f), screenY(1.0f), kBlue, "instance 2"));
        CHECK(pixelIs(driver, screenX(1.5f), screenY(0.0f), kClear, "below instance 2"));
        CHECK(pixelIs(driver, screenX(-1.5f), screenY(1.0f), kClear, "above instance 0"));
        CHECK(pixelIs(driver, 21, 24, kClear, "gap between 0 and 1"));
        CHECK(pixelIs(driver, 0, 0, kClear, "corner"));
        CHECK(pixelIs(driver, kWidth - 1, kHeight - 1, kClear, "opposite corner"));
        CHECK(pixelIs(driver, screenX(-1.5f) + 6, screenY(0.0f), kRed, "inside the right edge"));
        CHECK(pixelIs(driver, screenX(-1.5f) + 9, screenY(0.0f), kClear, "outside the right edge"));
        CHECK(boxIs(server.multimeshGetAabb(mm), -2.0f, -0.5f, 0.0f, 2.0f, 1.5f, 0.0f));
        CHECK(boxIs(server.instanceGetAabb(instance), -2.0f, -0.5f, 0.0f, 2.0f, 1.5f, 0.0f));

        frames.draw(server);
        CHECK(server.getStats().multimeshUploadBytes == 0);
        CHECK(pixelIs(driver, screenX(0.0f), screenY(0.0f), kGreen, "unchanged frame"));

        server.multimeshInstanceSetTransform(mm, 0, translation(-1.5f, -1.0f, 0.0f));
        frames.draw(server);
        CHECK(server.getStats().multimeshUploadBytes == 192);
        CHECK(pixelIs(driver, screenX(-1.5f), screenY(-1.0f), kRed, "instance 0 moved"));
        CHECK(pixelIs(driver, screenX(-1.5f), screenY(0.0f), kClear, "instance 0 left its place"));
        CHECK(boxIs(server.multimeshGetAabb(mm), -2.0f, -1.5f, 0.0f, 2.0f, 1.5f, 0.0f));

        server.multimeshInstanceSetColor(mm, 1, Math::Vec4(0.0f, 0.0f, 1.0f, 1.0f));
        frames.draw(server);
        CHECK(pixelIs(driver, screenX(0.0f), screenY(0.0f), kBlue, "instance 1 recoloured"));

        server.multimeshInstanceSetTransform(mm, 1,
                Math::Mat4::Scale(Math::Vec3(2.0f, 2.0f, 1.0f)));
        frames.draw(server);
        CHECK(pixelIs(driver, screenX(0.0f) + 10, screenY(0.0f), kBlue, "instance 1 scaled"));
        CHECK(pixelIs(driver, screenX(0.0f) + 15, screenY(0.0f), kClear, "outside scaled edge"));
        server.multimeshInstanceSetTransform(mm, 1, translation(0.0f, 0.0f, 0.0f));
        server.multimeshInstanceSetColor(mm, 1, Math::Vec4(0.0f, 1.0f, 0.0f, 1.0f));

        server.multimeshSetVisibleInstances(mm, 2);
        CHECK(server.multimeshGetVisibleInstances(mm) == 2);
        frames.draw(server);
        CHECK(server.getStats().multimeshInstancesDrawn == 2);
        CHECK(server.getStats().triangles == 4);
        CHECK(pixelIs(driver, screenX(0.0f), screenY(0.0f), kGreen, "visible count, kept"));
        CHECK(pixelIs(driver, screenX(1.5f), screenY(1.0f), kClear, "visible count, cut"));
        CHECK(boxIs(server.multimeshGetAabb(mm), -2.0f, -1.5f, 0.0f, 0.5f, 0.5f, 0.0f));

        server.multimeshSetVisibleInstances(mm, 4);
        server.multimeshSetVisibleInstances(mm, -2);
        CHECK(server.multimeshGetVisibleInstances(mm) == 2);

        server.multimeshSetVisibleInstances(mm, 0);
        frames.draw(server);
        CHECK(server.getStats().drawCalls == 0);
        CHECK(server.getStats().multimeshInstancesDrawn == 0);
        CHECK(pixelIs(driver, screenX(0.0f), screenY(0.0f), kClear, "no visible instances"));
        CHECK(server.instanceGetAabb(instance).min.x > server.instanceGetAabb(instance).max.x);

        server.multimeshSetVisibleInstances(mm, -1);
        frames.draw(server);
        CHECK(server.getStats().multimeshInstancesDrawn == 3);
        CHECK(pixelIs(driver, screenX(1.5f), screenY(1.0f), kBlue, "all visible again"));
        CHECK(boxIs(server.multimeshGetAabb(mm), -2.0f, -1.5f, 0.0f, 2.0f, 1.5f, 0.0f));

        for (int mode = 0; mode < 2; ++mode)
        {
            server.setCullingTreeEnabled(mode == 0);
            server.instanceSetTransform(instance, translation(100.0f, 0.0f, 0.0f));
            frames.draw(server);
            CHECK(server.getStats().instancesCulled == 1);
            CHECK(server.getStats().instancesDrawn == 0);
            CHECK(server.getStats().drawCalls == 0);
            CHECK(pixelIs(driver, screenX(0.0f), screenY(0.0f), kClear, "multimesh culled"));
            CHECK(boxIs(server.instanceGetAabb(instance), 98.0f, -1.5f, 0.0f, 102.0f, 1.5f, 0.0f));

            server.instanceSetTransform(instance, translation(-8.0f, 0.0f, 0.0f));
            frames.draw(server);
            CHECK(server.getStats().instancesCulled == 1);
            CHECK(server.getStats().drawCalls == 0);

            server.instanceSetTransform(instance, translation(-3.5f, 0.0f, 0.0f));
            frames.draw(server);
            CHECK(server.getStats().instancesCulled == 0);
            CHECK(server.getStats().instancesDrawn == 1);
            CHECK(server.getStats().drawCalls == 1);
            CHECK(pixelIs(driver, 5, screenY(1.0f), kBlue, "one instance on screen"));
            CHECK(pixelIs(driver, 32, 24, kClear, "the others off screen"));

            server.multimeshSetCustomAabb(mm,
                    Math::Box(Math::Vec3(-50.0f, -1.0f, -1.0f), Math::Vec3(-40.0f, 1.0f, 1.0f)));
            frames.draw(server);
            CHECK(server.getStats().instancesCulled == 1);
            CHECK(server.getStats().drawCalls == 0);
            CHECK(boxIs(server.multimeshGetAabb(mm), -50.0f, -1.0f, -1.0f, -40.0f, 1.0f, 1.0f));

            server.multimeshSetCustomAabb(mm, Math::Box());
            frames.draw(server);
            CHECK(server.getStats().instancesCulled == 0);
            CHECK(server.getStats().drawCalls == 1);
            CHECK(boxIs(server.multimeshGetAabb(mm), -2.0f, -1.5f, 0.0f, 2.0f, 1.5f, 0.0f));
            server.instanceSetTransform(instance, Math::Mat4::Identity());
            frames.draw(server);
            CHECK(server.getStats().multimeshInstancesDrawn == 3);
        }
        server.setCullingTreeEnabled(true);

        {
            ct::Vector<float> buffer;
            buffer.resize(48);
            for (std::size_t i = 0; i < 48; ++i) buffer[i] = 0.0f;
            const float x[3] = { 1.5f, -1.5f, 0.0f };
            const float y[3] = { -1.0f, 0.0f, 1.0f };
            for (int i = 0; i < 3; ++i)
            {
                buffer[i * 16 + 0] = 1.0f;
                buffer[i * 16 + 3] = x[i];
                buffer[i * 16 + 5] = 1.0f;
                buffer[i * 16 + 7] = y[i];
                buffer[i * 16 + 10] = 1.0f;
                buffer[i * 16 + 12 + i] = 1.0f;
                buffer[i * 16 + 15] = 1.0f;
            }
            CHECK(!server.multimeshSetBuffer(mm, buffer.data(), 47));
            CHECK(server.multimeshSetBuffer(mm, buffer.data(), 48));
            frames.draw(server);
            CHECK(pixelIs(driver, screenX(1.5f), screenY(-1.0f), kRed, "buffer, instance 0"));
            CHECK(pixelIs(driver, screenX(-1.5f), screenY(0.0f), kGreen, "buffer, instance 1"));
            CHECK(pixelIs(driver, screenX(0.0f), screenY(1.0f), kBlue, "buffer, instance 2"));
            CHECK(pixelIs(driver, screenX(0.0f), screenY(0.0f), kClear, "buffer, old place"));
            CHECK(boxIs(server.multimeshGetAabb(mm), -2.0f, -1.5f, 0.0f, 2.0f, 1.5f, 0.0f));
            server.multimeshInstanceSetTransform(mm, 2, translation(0.0f, 1.0f, 0.0f));
            CHECK(near(server.multimeshInstanceGetTransform(mm, 0).GetTranslation().x, 1.5f));
            CHECK(near(server.multimeshInstanceGetColor(mm, 1).y, 1.0f));
            frames.draw(server);
            CHECK(pixelIs(driver, screenX(1.5f), screenY(-1.0f), kRed, "buffer then edit"));
            CHECK(pixelIs(driver, screenX(0.0f), screenY(1.0f), kBlue, "buffer then edit, 2"));
        }

        server.multimeshAllocateData(mm, 3, false, false);
        CHECK(server.multimeshGetInstanceCount(mm) == 3);
        server.multimeshInstanceSetTransform(mm, 0, translation(-1.5f, 0.0f, 0.0f));
        server.multimeshInstanceSetTransform(mm, 1, translation(0.0f, 0.0f, 0.0f));
        server.multimeshInstanceSetTransform(mm, 2, translation(1.5f, 0.0f, 0.0f));
        server.multimeshInstanceSetColor(mm, 0, Math::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
        CHECK(server.multimeshInstanceGetColor(mm, 0).x == 0.0f);
        frames.draw(server);
        CHECK(server.getStats().multimeshUploadBytes == 3 * 12 * 4);
        CHECK(pixelIs(driver, screenX(-1.5f), screenY(0.0f), kWhite, "no colours, 0"));
        CHECK(pixelIs(driver, screenX(0.0f), screenY(0.0f), kWhite, "no colours, 1"));
        CHECK(pixelIs(driver, screenX(1.5f), screenY(0.0f), kWhite, "no colours, 2"));

        server.multimeshAllocateData(mm, 3, false, true);
        server.multimeshInstanceSetTransform(mm, 0, translation(-1.5f, 0.0f, 0.0f));
        server.multimeshInstanceSetTransform(mm, 1, translation(0.0f, 0.0f, 0.0f));
        server.multimeshInstanceSetTransform(mm, 2, translation(1.5f, 0.0f, 0.0f));
        server.multimeshInstanceSetCustomData(mm, 1, Math::Vec4(5.0f, 6.0f, 7.0f, 8.0f));
        {
            ct::Vector<float> buffer;
            CHECK(server.multimeshGetBuffer(mm, buffer));
            CHECK(buffer.size() == 3 * 16);
            if (buffer.size() == 3 * 16)
            {
                CHECK(buffer[16 + 3] == 0.0f && buffer[16 + 12] == 5.0f && buffer[16 + 15] == 8.0f);
                CHECK(buffer[32 + 3] == 1.5f);
            }
        }
        frames.draw(server);
        CHECK(pixelIs(driver, screenX(-1.5f), screenY(0.0f), kWhite, "custom data only, 0"));
        CHECK(pixelIs(driver, screenX(0.0f), screenY(0.0f), kWhite, "custom data only, 1"));
        CHECK(pixelIs(driver, screenX(1.5f), screenY(0.0f), kWhite, "custom data only, 2"));
        CHECK(pixelIs(driver, screenX(-0.75f), screenY(0.0f), kClear, "custom data only, gap"));

        server.multimeshAllocateData(mm, 3, true, true);
        server.multimeshInstanceSetTransform(mm, 0, translation(-1.5f, 0.0f, 0.0f));
        server.multimeshInstanceSetTransform(mm, 1, translation(0.0f, 0.0f, 0.0f));
        server.multimeshInstanceSetTransform(mm, 2, translation(1.5f, 0.0f, 0.0f));
        server.multimeshInstanceSetColor(mm, 0, Math::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
        server.multimeshInstanceSetColor(mm, 1, Math::Vec4(0.0f, 1.0f, 0.0f, 1.0f));
        server.multimeshInstanceSetColor(mm, 2, Math::Vec4(0.0f, 0.0f, 1.0f, 1.0f));
        server.multimeshInstanceSetCustomData(mm, 2, Math::Vec4(9.0f, 9.0f, 9.0f, 9.0f));
        frames.draw(server);
        CHECK(server.getStats().multimeshUploadBytes == 3 * 20 * 4);
        CHECK(pixelIs(driver, screenX(-1.5f), screenY(0.0f), kRed, "colours and data, 0"));
        CHECK(pixelIs(driver, screenX(0.0f), screenY(0.0f), kGreen, "colours and data, 1"));
        CHECK(pixelIs(driver, screenX(1.5f), screenY(0.0f), kBlue, "colours and data, 2"));
        server.multimeshAllocateData(mm, 3, true, false);
        server.multimeshInstanceSetTransform(mm, 0, translation(-1.5f, 0.0f, 0.0f));
        server.multimeshInstanceSetTransform(mm, 1, translation(0.0f, 0.0f, 0.0f));
        server.multimeshInstanceSetTransform(mm, 2, translation(1.5f, 0.0f, 0.0f));
        server.multimeshInstanceSetColor(mm, 0, Math::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
        server.multimeshInstanceSetColor(mm, 1, Math::Vec4(0.0f, 1.0f, 0.0f, 1.0f));
        server.multimeshInstanceSetColor(mm, 2, Math::Vec4(0.0f, 0.0f, 1.0f, 1.0f));
        frames.draw(server);
        CHECK(server.getStats().multimeshUploadBytes == 3 * 16 * 4);
        CHECK(pixelIs(driver, screenX(-1.5f), screenY(0.0f), kRed, "colours again, 0"));
        CHECK(pixelIs(driver, screenX(1.5f), screenY(0.0f), kBlue, "colours again, 2"));

        {
            const MultiMeshId big = server.multimeshCreate();
            const std::uint32_t count = 2000;
            const std::uint32_t regionBytes = 12 * 512 * 4;
            const std::uint32_t totalBytes = 12 * count * 4;
            server.multimeshAllocateData(big, count, false, false);
            server.multimeshSetMesh(big, quad);
            ct::Vector<float> data;
            data.resize(static_cast<std::size_t>(count) * 12);
            for (std::size_t i = 0; i < data.size(); ++i) data[i] = 0.0f;
            for (std::uint32_t i = 0; i < count; ++i)
            {
                data[i * 12 + 0] = 1.0f;
                data[i * 12 + 3] = static_cast<float>(i);
                data[i * 12 + 5] = 1.0f;
                data[i * 12 + 10] = 1.0f;
            }
            CHECK(server.multimeshSetBuffer(big, data.data(), data.size()));
            CHECK(boxIs(server.multimeshGetAabb(big), -0.5f, -0.5f, 0.0f, 1999.5f, 0.5f, 0.0f));
            frames.draw(server);
            CHECK(server.getStats().multimeshUploadBytes == totalBytes);
            frames.draw(server);
            CHECK(server.getStats().multimeshUploadBytes == 0);

            server.multimeshInstanceSetTransform(big, 1500, translation(-7.0f, 2.0f, 0.0f));
            CHECK(near(server.multimeshInstanceGetTransform(big, 1499).GetTranslation().x,
                    1499.0f));
            CHECK(near(server.multimeshInstanceGetTransform(big, 1999).GetTranslation().x,
                    1999.0f));
            frames.draw(server);
            CHECK(server.getStats().multimeshUploadBytes == regionBytes);
            CHECK(boxIs(server.multimeshGetAabb(big), -7.5f, -0.5f, 0.0f, 1999.5f, 2.5f, 0.0f));

            server.multimeshInstanceSetTransform(big, 10, translation(10.0f, 0.0f, 0.0f));
            server.multimeshInstanceSetTransform(big, 600, translation(600.0f, 0.0f, 0.0f));
            frames.draw(server);
            CHECK(server.getStats().multimeshUploadBytes == 2 * regionBytes);

            server.multimeshInstanceSetTransform(big, 10, translation(10.0f, 0.0f, 0.0f));
            server.multimeshInstanceSetTransform(big, 600, translation(600.0f, 0.0f, 0.0f));
            server.multimeshInstanceSetTransform(big, 1900, translation(1900.0f, 0.0f, 0.0f));
            frames.draw(server);
            CHECK(server.getStats().multimeshUploadBytes == totalBytes);

            server.multimeshSetVisibleInstances(big, 600);
            CHECK(boxIs(server.multimeshGetAabb(big), -0.5f, -0.5f, 0.0f, 599.5f, 0.5f, 0.0f));
            server.multimeshInstanceSetTransform(big, 1500, translation(1500.0f, 0.0f, 0.0f));
            frames.draw(server);
            CHECK(server.getStats().multimeshUploadBytes == 0);
            server.multimeshSetVisibleInstances(big, 2000);
            frames.draw(server);
            CHECK(server.getStats().multimeshUploadBytes == totalBytes);
            CHECK(boxIs(server.multimeshGetAabb(big), -0.5f, -0.5f, 0.0f, 1999.5f, 0.5f, 0.0f));
            server.multimeshFree(big);
        }

        {
            const MultiMeshId later = server.multimeshCreate();
            const InstanceId waiting = server.instanceCreate();
            server.instanceSetBase(waiting, later);
            server.instanceSetScenario(waiting, scenario);
            server.multimeshAllocateData(later, 1, false, false);
            frames.draw(server);
            CHECK(server.getStats().multimeshInstancesDrawn == 3);
            server.multimeshSetMesh(later, quad);
            server.multimeshInstanceSetTransform(later, 0, translation(0.0f, 1.2f, 0.0f));
            frames.draw(server);
            CHECK(server.getStats().multimeshInstancesDrawn == 4);
            CHECK(pixelIs(driver, screenX(0.0f), screenY(1.2f), kWhite, "late instance"));
            server.instanceFree(waiting);
            server.multimeshFree(later);
        }

        server.instanceSetVisible(instance, false);

        const ScenarioId lit = server.scenarioCreate();
        {
            EnvironmentSettings dark;
            dark.ambientSource = AmbientSource::Color;
            server.scenarioSetEnvironment(lit, dark);
        }
        server.viewportSetScenario(viewport, lit);
        server.setDirectionalShadowSize(2048, false);

        SurfaceData wallData = quadData(0.0f, 2.0f, white, true);
        const MeshId wallMesh = meshes.create("wall");
        CHECK(meshes.addSurface(wallMesh, wallData));
        const MaterialId matte = materials.create("matte");
        CHECK(materials.setFloat(matte, MaterialParam::Specular, 0.0f));
        CHECK(materials.setFloat(matte, MaterialParam::Roughness, 1.0f));
        const InstanceId wall = server.instanceCreate(wallMesh, lit);
        server.instanceSetMaterialOverride(wall, matte);
        server.instanceSetCastShadows(wall, false);

        const float pi = 3.14159265358979f;
        const MeshId occluder = meshes.create("occluder");
        CHECK(meshes.addSurface(occluder, quadData(1.0f, 0.3f, white, true)));
        const MultiMeshId shadowers = server.multimeshCreate();
        server.multimeshAllocateData(shadowers, 2, false, false);
        server.multimeshSetMesh(shadowers, occluder);
        server.multimeshInstanceSetTransform(shadowers, 0, translation(0.0f, 0.0f, 0.0f));
        server.multimeshInstanceSetTransform(shadowers, 1, translation(0.0f, 1.0f, 0.0f));
        const InstanceId trees = server.instanceCreate();
        server.instanceSetBase(trees, shadowers);
        server.instanceSetScenario(trees, lit);
        server.instanceSetMaterialOverride(trees, matte);

        const LightId sun = server.lightCreate(LightType::Directional);
        server.lightSetScenario(sun, lit);
        server.lightSetTransform(sun, Math::Mat4::RotationY(pi / 6.0f));
        const float lighted = cosf(pi / 6.0f);

        frames.draw(server);
        CHECK(server.getStats().shadowCascades == 0);
        CHECK(server.getStats().multimeshInstancesDrawn == 2);
        CHECK(pixelIs(driver, 24, 24, grey(lighted), "no shadows, first"));
        CHECK(pixelIs(driver, 24, 10, grey(lighted), "no shadows, second"));

        server.lightSetShadow(sun, true);
        frames.draw(server);
        CHECK(server.getStats().shadowCascades == 4);
        CHECK(server.getStats().shadowCasters >= 1);
        CHECK(pixelIs(driver, 24, 24, kBlack, "shadow of the first instance"));
        CHECK(pixelIs(driver, 24, 10, kBlack, "shadow of the second instance"));
        CHECK(pixelIs(driver, 45, 24, grey(lighted), "wall outside of the shadows"));
        CHECK(pixelIs(driver, 24, 38, grey(lighted), "wall below the shadows"));
        const std::uint32_t fullTriangles = server.getStats().shadowTriangles;
        CHECK(fullTriangles > 0);

        server.multimeshSetVisibleInstances(shadowers, 1);
        frames.draw(server);
        CHECK(pixelIs(driver, 24, 24, kBlack, "shadow with one visible instance"));
        CHECK(pixelIs(driver, 24, 10, grey(lighted), "no shadow of the hidden instance"));
        CHECK(server.getStats().shadowTriangles * 2 == fullTriangles);
        server.multimeshSetVisibleInstances(shadowers, -1);

        server.multimeshInstanceSetTransform(shadowers, 1, translation(0.0f, -1.0f, 0.0f));
        frames.draw(server);
        CHECK(pixelIs(driver, 24, 10, grey(lighted), "second instance left"));
        CHECK(pixelIs(driver, 24, 38, kBlack, "second instance shadow moved"));
        server.multimeshInstanceSetTransform(shadowers, 1, translation(0.0f, 1.0f, 0.0f));

        server.instanceSetCastShadows(trees, false);
        frames.draw(server);
        CHECK(pixelIs(driver, 24, 24, grey(lighted), "multimesh that casts no shadow"));
        CHECK(pixelIs(driver, 24, 10, grey(lighted), "multimesh that casts no shadow, second"));
        server.instanceSetCastShadows(trees, true);

        {
            SurfaceData awayData = quadData(1.0f, 0.3f, white, true);
            for (std::size_t i = 0; i < awayData.positions.size(); ++i)
                awayData.positions[i].x += 50.0f;
            const MeshId awayMesh = meshes.create("occluder shadow");
            CHECK(meshes.addSurface(awayMesh, awayData));
            CHECK(meshes.setShadowMesh(occluder, awayMesh));
            frames.draw(server);
            CHECK(server.getStats().shadowTriangles == fullTriangles);
            CHECK(pixelIs(driver, 24, 24, grey(lighted), "shadow mesh moved the shadow"));
            CHECK(pixelIs(driver, 24, 10, grey(lighted), "shadow mesh moved the second shadow"));

            SurfaceData oneTriangle = quadData(1.0f, 0.3f, white, true);
            oneTriangle.indices.resize(3);
            const MeshId smallMesh = meshes.create("occluder small shadow");
            CHECK(meshes.addSurface(smallMesh, oneTriangle));
            CHECK(meshes.setShadowMesh(occluder, smallMesh));
            frames.draw(server);
            CHECK(server.getStats().shadowTriangles * 2 == fullTriangles);

            meshes.destroy(smallMesh);
            meshes.destroy(awayMesh);
            frames.draw(server);
            CHECK(server.getStats().shadowTriangles == fullTriangles);
            CHECK(pixelIs(driver, 24, 24, kBlack, "shadow back after the shadow meshes"));
        }

        {
            server.instanceSetVisible(trees, false);
            server.lightSetShadow(sun, false);

            SurfaceData tangentData = quadData(0.5f, 0.3f, white, true);
            for (int i = 0; i < 4; ++i)
                tangentData.tangents.push_back(Math::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
            tangentData.uvs.push_back(Math::Vec2(0.0f, 0.0f));
            tangentData.uvs.push_back(Math::Vec2(1.0f, 0.0f));
            tangentData.uvs.push_back(Math::Vec2(1.0f, 1.0f));
            tangentData.uvs.push_back(Math::Vec2(0.0f, 1.0f));
            const MeshId tangentMesh = meshes.create("tangent quad");
            CHECK(meshes.addSurface(tangentMesh, tangentData));

            Pixmap flat(4, 4, 4);
            for (std::uint32_t y = 0; y < 4; ++y)
                for (std::uint32_t x = 0; x < 4; ++x) flat.set_pixel(x, y, 128, 128, 255, 255);
            const TextureId flatNormal =
                    textures.createFromPixmap(flat, false, false, "flat normal");
            const MaterialId mapped = materials.create("mapped");
            CHECK(materials.setFloat(mapped, MaterialParam::Specular, 0.0f));
            CHECK(materials.setFloat(mapped, MaterialParam::Roughness, 1.0f));
            CHECK(materials.setTexture(mapped, MaterialTexture::Normal, flatNormal));

            const MultiMeshId mappedMultimesh = server.multimeshCreate();
            server.multimeshAllocateData(mappedMultimesh, 1, true, false);
            server.multimeshSetMesh(mappedMultimesh, tangentMesh);
            server.multimeshInstanceSetTransform(mappedMultimesh, 0, translation(0.0f, 0.0f, 0.0f));
            server.multimeshInstanceSetColor(mappedMultimesh, 0, white);
            const InstanceId mappedInstance = server.instanceCreate();
            server.instanceSetBase(mappedInstance, mappedMultimesh);
            server.instanceSetScenario(mappedInstance, lit);
            server.instanceSetMaterialOverride(mappedInstance, mapped);
            frames.draw(server);
            CHECK(pixelIs(driver, 32, 24, grey(lighted), "normal mapped multimesh"));
            CHECK(pixelIs(driver, 24, 24, grey(lighted),
                    "wall beside the normal mapped multimesh"));

            const MaterialId flatBlue = materials.create("unshaded blue");
            CHECK(materials.setColor(flatBlue, MaterialParam::Albedo,
                    Math::Vec4(0.0f, 0.0f, 1.0f, 1.0f)));
            CHECK(materials.setShading(flatBlue, MaterialShading::Unshaded));
            server.instanceSetMaterialOverride(mappedInstance, flatBlue);
            frames.draw(server);
            CHECK(pixelIs(driver, 32, 24, kBlue, "unshaded multimesh"));
            CHECK(pixelIs(driver, 24, 24, grey(lighted), "wall beside the unshaded multimesh"));

            server.instanceFree(mappedInstance);
            server.multimeshFree(mappedMultimesh);
            server.lightSetShadow(sun, true);
        }

        server.lightFree(sun);

        {
            auto aimed = [](const Math::Vec3& from, const Math::Vec3& to)
            { return Math::Mat4::LookAt(from, to, Math::Vec3(0.0f, 1.0f, 0.0f)).InverseAffine(); };
            auto facing = [&](const Math::Vec3& at, const Math::Vec3& from)
            { return aimed(at, at + (at - from)); };
            auto alongRay = [](const Math::Vec3& from, const Math::Vec3& to, float distance)
            { return from + (to - from).Normalized() * distance; };

            const MeshId blockerMesh = meshes.create("spot blocker");
            CHECK(meshes.addSurface(blockerMesh, quadData(0.0f, 0.3f, white, true)));
            const MultiMeshId blockers = server.multimeshCreate();
            server.multimeshAllocateData(blockers, 2, false, false);
            server.multimeshSetMesh(blockers, blockerMesh);
            const Math::Vec3 origin(0.0f, 0.0f, 0.0f);
            const Math::Vec3 lampPosition(-4.0f, 0.0f, 4.0f);
            const Math::Vec3 high(0.0f, 1.0f, 0.0f);
            server.multimeshInstanceSetTransform(blockers, 0,
                    facing(alongRay(lampPosition, origin, 2.0f), lampPosition));
            server.multimeshInstanceSetTransform(blockers, 1,
                    facing(alongRay(lampPosition, high, 2.0f), lampPosition));
            const InstanceId blocking = server.instanceCreate();
            server.instanceSetBase(blocking, blockers);
            server.instanceSetScenario(blocking, lit);
            server.instanceSetMaterialOverride(blocking, matte);
            server.instanceSetVisible(trees, false);

            const LightId lamp = server.lightCreate(LightType::Spot);
            server.lightSetScenario(lamp, lit);
            server.lightSetParam(lamp, LightParam::Range, 10.0f);
            server.lightSetParam(lamp, LightParam::SpotAngle, 40.0f);
            server.lightSetTransform(lamp, aimed(lampPosition, origin));

            frames.draw(server);
            CHECK(server.getStats().shadowLights == 0);
            CHECK(readPixel(driver, 32, 24).g > 20);
            CHECK(readPixel(driver, 32, 10).g > 20);
            CHECK(readPixel(driver, 32, 36).g > 20);

            server.lightSetShadow(lamp, true);
            frames.draw(server);
            CHECK(server.getStats().shadowLights == 1);
            CHECK(server.getStats().shadowPasses == 1);
            CHECK(server.getStats().shadowCasters >= 1);
            CHECK(pixelIs(driver, 32, 24, kBlack, "spot shadow of the first instance"));
            CHECK(pixelIs(driver, 32, 10, kBlack, "spot shadow of the second instance"));
            CHECK(readPixel(driver, 32, 36).g > 20);
            CHECK(server.getStats().shadowTriangles > 0);

            server.multimeshSetVisibleInstances(blockers, 1);
            frames.draw(server);
            CHECK(pixelIs(driver, 32, 24, kBlack, "spot shadow, one visible"));
            CHECK(readPixel(driver, 32, 10).g > 20);
            server.multimeshSetVisibleInstances(blockers, -1);

            server.instanceSetCastShadows(blocking, false);
            frames.draw(server);
            CHECK(readPixel(driver, 32, 24).g > 20);
            CHECK(readPixel(driver, 32, 10).g > 20);
            server.instanceSetCastShadows(blocking, true);

            server.instanceFree(blocking);
            server.multimeshFree(blockers);
            server.lightFree(lamp);
        }

        {
            const LightId lamp = server.lightCreate(LightType::Directional);
            server.lightSetScenario(lamp, lit);
            server.lightSetTransform(lamp, Math::Mat4::RotationY(pi / 6.0f));
            server.lightSetShadow(lamp, true);
            server.instanceSetVisible(trees, true);
            frames.draw(server);
            CHECK(pixelIs(driver, 24, 24, kBlack, "before freeing"));

            server.multimeshFree(shadowers);
            CHECK(!server.multimeshValid(shadowers));
            server.multimeshFree(shadowers);
            frames.draw(server);
            frames.draw(server);
            CHECK(server.getStats().multimeshInstancesDrawn == 0);
            CHECK(server.getStats().drawCalls == 1);
            CHECK(pixelIs(driver, 24, 24, grey(lighted), "multimesh freed under its instance"));
            CHECK(server.instanceGetAabb(trees).min.x > server.instanceGetAabb(trees).max.x);

            const MultiMeshId again = server.multimeshCreate();
            server.multimeshAllocateData(again, 1, false, false);
            server.multimeshSetMesh(again, occluder);
            server.multimeshInstanceSetTransform(again, 0, translation(0.0f, 0.0f, 0.0f));
            server.instanceSetBase(trees, again);
            frames.draw(server);
            CHECK(server.getStats().multimeshInstancesDrawn == 1);
            CHECK(pixelIs(driver, 24, 24, kBlack, "new multimesh on the old instance"));

            meshes.destroy(occluder);
            frames.draw(server);
            frames.draw(server);
            CHECK(server.getStats().multimeshInstancesDrawn == 0);
            CHECK(pixelIs(driver, 24, 24, grey(lighted), "mesh freed under the multimesh"));

            const MeshId replacement = meshes.create("replacement");
            CHECK(meshes.addSurface(replacement, quadData(1.0f, 0.3f, white, true)));
            server.multimeshSetMesh(again, replacement);
            frames.draw(server);
            CHECK(server.getStats().multimeshInstancesDrawn == 1);
            CHECK(pixelIs(driver, 24, 24, kBlack, "replacement mesh"));

            server.instanceFree(trees);
            frames.draw(server);
            CHECK(pixelIs(driver, 24, 24, grey(lighted), "instance freed before the multimesh"));
            server.multimeshFree(again);
            server.lightFree(lamp);
        }

        {
            const ScenarioId forest = server.scenarioCreate();
            server.scenarioSetEnvironment(forest, whiteAmbient());
            server.viewportSetScenario(viewport, forest);
            server.setDirectionalShadowSize(4096, false);

            SurfaceData ground;
            ground.positions.push_back(Math::Vec3(-200.0f, 0.0f, 200.0f));
            ground.positions.push_back(Math::Vec3(200.0f, 0.0f, 200.0f));
            ground.positions.push_back(Math::Vec3(200.0f, 0.0f, -200.0f));
            ground.positions.push_back(Math::Vec3(-200.0f, 0.0f, -200.0f));
            for (int i = 0; i < 4; ++i)
            {
                ground.colors.push_back(white);
                ground.normals.push_back(Math::Vec3(0.0f, 1.0f, 0.0f));
            }
            const std::uint32_t groundIndices[6] = { 0, 1, 2, 0, 2, 3 };
            for (std::uint32_t index: groundIndices) ground.indices.push_back(index);
            const MeshId groundMesh = meshes.create("forest ground");
            CHECK(meshes.addSurface(groundMesh, ground));
            const InstanceId groundInstance = server.instanceCreate(groundMesh, forest);

            const std::uint32_t presets[3] = { 0, 4, 2 };
            const float heights[3] = { 12.0f, 16.0f, 3.0f };
            const int counts[3] = { 90, 40, 150 };
            MeshId treeMeshes[3];
            MultiMeshId treeMultimeshes[3];
            InstanceId treeInstances[3];
            std::uint32_t expectedTriangles = 2;
            Random random;
            for (int species = 0; species < 3; ++species)
            {
                TreeGeometry geometry;
                TreeGenerator::generate(TreeGenerator::preset(presets[species]).params, geometry);
                const Math::Box box =
                        MeshStore::computeAabb(geometry.bark.positions)
                                .Union(MeshStore::computeAabb(geometry.leaves.positions));
                const float fit = heights[species] / (box.max.y - box.min.y);
                treeMeshes[species] = meshes.create("forest tree");
                CHECK(meshes.addSurface(treeMeshes[species], geometry.bark));
                CHECK(meshes.addSurface(treeMeshes[species], geometry.leaves));
                TreeGeometry shadow;
                TreeGenerator::buildShadowGeometry(geometry, 6, shadow);
                const MeshId shadowMesh = meshes.create("forest tree shadow");
                CHECK(meshes.addSurface(shadowMesh, shadow.bark));
                CHECK(meshes.addSurface(shadowMesh, shadow.leaves));
                CHECK(meshes.setShadowMesh(treeMeshes[species], shadowMesh));
                const MeshInfo* info = meshes.get(treeMeshes[species]);
                CHECK(info != nullptr && info->surfaces.size() == 2);
                if (info)
                    for (const SurfaceInfo& surface: info->surfaces)
                        expectedTriangles += surface.indexCount / 3 * counts[species];

                treeMultimeshes[species] = server.multimeshCreate();
                server.multimeshAllocateData(treeMultimeshes[species], counts[species], true,
                        false);
                server.multimeshSetMesh(treeMultimeshes[species], treeMeshes[species]);
                for (int i = 0; i < counts[species]; ++i)
                {
                    const float angle = random.randfRange(0.0f, 6.2831853f);
                    const float distance = 90.0f * sqrtf(random.randf());
                    const float scale = fit * random.randfRange(0.8f, 1.25f);
                    server.multimeshInstanceSetTransform(treeMultimeshes[species], i,
                            translation(cosf(angle) * distance, 0.0f, sinf(angle) * distance) *
                                    Math::Mat4::RotationY(random.randfRange(0.0f, 6.2831853f)) *
                                    Math::Mat4::Scale(Math::Vec3(scale, scale, scale)));
                    server.multimeshInstanceSetColor(treeMultimeshes[species], i, white);
                }
                treeInstances[species] = server.instanceCreate();
                server.instanceSetBase(treeInstances[species], treeMultimeshes[species]);
                server.instanceSetScenario(treeInstances[species], forest);
            }

            const LightId forestSun = server.lightCreate(LightType::Directional);
            server.lightSetScenario(forestSun, forest);
            server.lightSetShadow(forestSun, true);
            server.lightSetParam(forestSun, LightParam::ShadowMaxDistance, 120.0f);
            server.lightSetTransform(forestSun,
                    Math::Mat4::LookAt(Math::Vec3(11.0f, 16.0f, 9.0f), Math::Vec3(0.0f, 0.0f, 0.0f),
                            Math::Vec3(0.0f, 1.0f, 0.0f))
                            .InverseAffine());
            server.cameraSetPerspective(camera, 60.0f, 0.1f, 500.0f);
            server.cameraSetTransform(camera,
                    Math::Mat4::LookAt(Math::Vec3(0.0f, 8.0f, 100.0f), Math::Vec3(0.0f, 4.0f, 0.0f),
                            Math::Vec3(0.0f, 1.0f, 0.0f))
                            .InverseAffine());

            frames.draw(server);
            frames.draw(server);
            const RenderStats forestStats = server.getStats();
            CHECK(forestStats.drawCalls == 7);
            CHECK(forestStats.multimeshInstancesDrawn == 280);
            CHECK(forestStats.instancesDrawn == 4);
            CHECK(forestStats.triangles == expectedTriangles);
            CHECK(forestStats.shadowCascades == 4);
            CHECK(forestStats.shadowTriangles > 0);
            CHECK(forestStats.multimeshUploadBytes == 0);

            for (int species = 0; species < 3; ++species)
            {
                const MeshId shadowMesh = meshes.getShadowMesh(treeMeshes[species]);
                CHECK(shadowMesh.valid());
                CHECK(meshes.setShadowMesh(treeMeshes[species], MeshId()));
            }
            frames.draw(server);
            const RenderStats fullShadowStats = server.getStats();
            CHECK(fullShadowStats.triangles == expectedTriangles);
            CHECK(fullShadowStats.shadowTriangles > forestStats.shadowTriangles);
            printf("forest, 280 trees in 3 multimeshes: draws %u, instances %u, triangles %u\n",
                    forestStats.drawCalls, forestStats.multimeshInstancesDrawn,
                    forestStats.triangles);
            printf("forest shadows: with shadow meshes %u draws and %u triangles, without %u draws "
                   "and %u triangles\n",
                    forestStats.shadowDrawCalls, forestStats.shadowTriangles,
                    fullShadowStats.shadowDrawCalls, fullShadowStats.shadowTriangles);
            (void) groundInstance;
        }

        frames.finish();
    }

    CHECK(RenderingServer::singleton() == nullptr);

    Device::destroy(device);

    if (failures)
    {
        printf("%d checks failed\n", failures);
        return 1;
    }
    printf("multimesh ok\n");
    return 0;
}
