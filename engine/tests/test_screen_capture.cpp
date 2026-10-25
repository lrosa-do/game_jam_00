#include "Check.h"
#include "LitTest.h"

#include "prisma3d/Device.h"
#include "prisma3d/core/Pixmap.h"
#include "prisma3d/main/Main.h"
#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/scene/SceneTree.h"

#include <stdio.h>
#include <string.h>

using namespace prisma3d;

namespace
{

const int kSize = 96;

long fileSize(const char* path, char* head, int headSize)
{
    FILE* file = fopen(path, "rb");
    if (!file) return -1;
    const size_t read = fread(head, 1, static_cast<size_t>(headSize), file);
    fseek(file, 0, SEEK_END);
    const long size = ftell(file);
    fclose(file);
    return read == static_cast<size_t>(headSize) ? size : -1;
}

int gifFrameCount(const char* path)
{
    FILE* file = fopen(path, "rb");
    if (!file) return -1;
    int frames = 0;
    int previous[2] = { 0, 0 };
    int byte = 0;
    while ((byte = fgetc(file)) != EOF)
    {
        if (previous[0] == 0x21 && previous[1] == 0xF9 && byte == 0x04) ++frames;
        previous[0] = previous[1];
        previous[1] = byte;
    }
    fclose(file);
    return frames;
}

struct Rig
{
    RenderingServer* server = nullptr;
    Device* device = nullptr;

    void frame()
    {
        server->renderViewports();
        device->gifRecordingFrame();
        server->endFrame(true);
    }
};

void testScreenshot(Rig& rig, const char* name)
{
    char path[64];
    snprintf(path, sizeof(path), "capture_%s.png", name);
    remove(path);

    rig.server->renderViewports();
    CHECK(rig.device->takeScreenshot(path));
    rig.server->endFrame(true);

    Pixmap image;
    CHECK(image.load(path));
    CHECK(image.width == kSize && image.height == kSize);
    if (image.width == kSize && image.height == kSize)
    {
        const Color centre = image.get_pixel_color(kSize / 2, kSize / 2);
        CHECK(centre.r() > 200 && centre.g() < 40 && centre.b() < 40);
    }
    remove(path);
}

void testGif(Rig& rig, const char* name)
{
    char path[64];
    snprintf(path, sizeof(path), "capture_%s.gif", name);
    remove(path);

    CHECK(!rig.device->isGifRecording());
    CHECK(!rig.device->gifRecordingStop());
    CHECK(rig.device->gifRecordingStart(path));
    CHECK(rig.device->isGifRecording());
    CHECK(!rig.device->gifRecordingStart(path));
    for (int i = 0; i < 30; ++i) rig.frame();
    CHECK(rig.device->gifRecordingFrames() == 30);
    CHECK(rig.device->gifRecordingStop());
    CHECK(!rig.device->isGifRecording());
    CHECK(rig.device->gifRecordingFrames() == 0);

    char head[6] = {};
    const long size = fileSize(path, head, 6);
    CHECK(size > 100);
    CHECK(memcmp(head, "GIF89a", 6) == 0);
    CHECK(gifFrameCount(path) == 3);

    char headAgain[6] = {};
    remove(path);
    CHECK(fileSize(path, headAgain, 6) < 0);
}

void testAutoNames(Rig& rig)
{
    remove("screenshot000.png");
    remove("screenrec001.gif");
    rig.server->renderViewports();
    CHECK(rig.device->takeScreenshot());
    rig.server->endFrame(true);
    CHECK(rig.device->gifRecordingStart());
    for (int i = 0; i < 10; ++i) rig.frame();
    CHECK(rig.device->gifRecordingStop());
    char head[6] = {};
    CHECK(fileSize("screenshot000.png", head, 6) > 0);
    CHECK(fileSize("screenrec001.gif", head, 6) > 0);
    remove("screenshot000.png");
    remove("screenrec001.gif");
}

} // namespace

int main(int argc, char** argv)
{
    const bool vulkan = argc > 1 && strcmp(argv[1], "vulkan") == 0;

    CHECK(Device::headlessSupported());
    if (!Device::headlessSupported()) return 1;

    DeviceDesc desc;
    desc.title = "screen capture test";
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
    rig.device = Main::device();

    RenderingServer& server = *rig.server;
    const ScenarioId scenario = server.scenarioCreate();
    server.scenarioSetEnvironment(scenario, whiteAmbient());
    const CameraId camera = server.cameraCreate();
    server.cameraSetPerspective(camera, 60.0f, 0.1f, 100.0f);
    const ViewportId screen = server.viewportCreate();
    server.viewportSetSize(screen, kSize, kSize);
    server.viewportAttachCamera(screen, camera);
    server.viewportSetScenario(screen, scenario);
    server.viewportSetClearColor(screen, Math::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
    server.viewportSetUpdateMode(screen, ViewportUpdateMode::Always);
    server.viewportAttachToScreen(screen, true);

    testScreenshot(rig, vulkan ? "vulkan" : "opengl");
    testGif(rig, vulkan ? "vulkan" : "opengl");
    testAutoNames(rig);

    Main::device()->requestClose();
    tree.finalize();
    Main::cleanup();

    if (failures) printf("%d check(s) failed\n", failures);
    else
        printf("all checks passed\n");
    return failures ? 1 : 0;
}
