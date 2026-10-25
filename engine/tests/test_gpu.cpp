#include "Check.h"

#include "prisma3d/Device.h"

#include <string.h>

using namespace prisma3d;

namespace
{

const int kWidth = 64;
const int kHeight = 48;

bool pixelIs(prisma::Driver* driver, int x, int y, int r, int g, int b)
{
    unsigned char pixel[4] = { 0, 0, 0, 0 };
    prisma::Rect rect;
    rect.x = x;
    rect.y = y;
    rect.width = 1;
    rect.height = 1;
    if (!driver->readPixels(prisma::RenderTarget(), rect, pixel))
    {
        printf("pixel (%d,%d) could not be read\n", x, y);
        return false;
    }
    const int dr = pixel[0] - r;
    const int dg = pixel[1] - g;
    const int db = pixel[2] - b;
    const bool same = dr >= -1 && dr <= 1 && dg >= -1 && dg <= 1 && db >= -1 && db <= 1;
    if (!same) printf("pixel (%d,%d) is %d,%d,%d\n", x, y, pixel[0], pixel[1], pixel[2]);
    return same;
}

void clearFrame(Device* device, float r, float g, float b)
{
    prisma::Driver* driver = device->driver();
    device->processEvents();
    driver->beginFrame();
    prisma::RenderPassDesc pass;
    pass.clearColor[0] = r;
    pass.clearColor[1] = g;
    pass.clearColor[2] = b;
    driver->beginRenderPass(pass);
    driver->endRenderPass();
}

void finishFrame(Device* device)
{
    device->driver()->endFrame();
    device->driver()->present();
}

} // namespace

int main(int argc, char** argv)
{
    const bool vulkan = argc > 1 && strcmp(argv[1], "vulkan") == 0;

    CHECK(Device::headlessSupported());
    if (!Device::headlessSupported()) return 1;

    DeviceDesc desc;
    desc.title = "gpu test";
    desc.driver = vulkan ? prisma::DriverType::Vulkan : prisma::DriverType::OpenGL;
    desc.width = kWidth;
    desc.height = kHeight;
    desc.headless = true;

    Device* device = Device::create(desc);
    CHECK(device != nullptr);
    if (!device) return 1;

    CHECK(Device::singleton() == device);
    CHECK(device->isHeadless());
    CHECK(device->driver() != nullptr);
    CHECK(device->driverType() == desc.driver);
    CHECK(device->windowCanDraw());
    CHECK(!device->closeRequested());

    int width = 0;
    int height = 0;
    device->windowGetFramebufferSize(&width, &height);
    CHECK(width == kWidth && height == kHeight);
    device->windowGetSize(&width, &height);
    CHECK(width == kWidth && height == kHeight);
    CHECK(device->getScreenCount() == 0);
    CHECK(device->windowGetScale() == 1.0f);
    device->windowSetTitle("still fine");
    device->windowSetMode(WindowMode::Fullscreen);
    CHECK(device->windowGetMode() == WindowMode::Windowed);
    device->mouseSetMode(MouseMode::Captured);
    CHECK(device->mouseGetMode() == MouseMode::Captured);
    CHECK(device->mouseGetButtonState() == 0);
    CHECK(!device->clipboardHas());

    prisma::Driver* driver = device->driver();
    const prisma::Caps& caps = driver->caps();
    printf("%s %u.%u: max texture %u, compute %d\n",
            driver->type() == prisma::DriverType::Vulkan ? "Vulkan"
            : caps.gles                                  ? "OpenGL ES"
                                                         : "OpenGL",
            caps.versionMajor, caps.versionMinor, caps.maxTextureSize, caps.compute);
    CHECK(caps.maxTextureSize >= 2048);

    clearFrame(device, 0.10f, 0.25f, 0.45f);
    CHECK(pixelIs(driver, 0, 0, 26, 64, 115));
    CHECK(pixelIs(driver, kWidth - 1, kHeight - 1, 26, 64, 115));
    CHECK(pixelIs(driver, kWidth / 2, kHeight / 2, 26, 64, 115));
    finishFrame(device);

    clearFrame(device, 1.0f, 0.5f, 0.0f);
    CHECK(pixelIs(driver, 3, 5, 255, 128, 0));
    finishFrame(device);

    const std::uint64_t before = device->input().getProcessFrames();
    device->processEvents();
    CHECK(device->input().getProcessFrames() == before + 1);
    CHECK(device->getTicksUsec() > 0);

    device->requestClose();
    CHECK(device->closeRequested());
    device->cancelClose();
    CHECK(!device->closeRequested());
    device->requestClose();
    CHECK(device->closeRequested());

    Device::destroy(device);
    CHECK(Device::singleton() == nullptr);

    if (failures) printf("%d check(s) failed\n", failures);
    else
        printf("all checks passed\n");
    return failures ? 1 : 0;
}
