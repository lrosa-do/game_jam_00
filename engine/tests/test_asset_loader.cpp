#include "Check.h"

#include "prisma3d/Device.h"
#include "prisma3d/core/FileSystem.h"
#include "prisma3d/core/Pixmap.h"
#include "prisma3d/import/ModelImporter.h"
#include "prisma3d/render/AssetLoader.h"
#include "prisma3d/render/MaterialStore.h"
#include "prisma3d/render/MeshStore.h"
#include "prisma3d/render/TextureStore.h"

#include "dds.h"
#include "platform.h"

#include <cstdint>
#include <string.h>

#include <ct/string.hpp>
#include <ct/vector.hpp>

using namespace prisma3d;

namespace
{

typedef ct::Vector<std::uint8_t> Bytes;

ct::String path(const char* directory, const char* name)
{
    ct::String result(directory);
    result += "/";
    result += name;
    return result;
}

struct Rgba
{
    int r;
    int g;
    int b;
    int a;
};

const Rgba kRed = { 255, 0, 0, 255 };
const Rgba kBlue = { 0, 0, 255, 255 };
const Rgba kMagenta = { 255, 0, 255, 255 };

bool read(TextureStore& store, TextureId id, unsigned width, unsigned height, Bytes& out)
{
    prisma::RenderTarget target;
    target.texture = store.gpu(id);
    target.mip = 0;
    prisma::Rect rect;
    rect.x = 0;
    rect.y = 0;
    rect.width = width;
    rect.height = height;
    out.resize(static_cast<std::size_t>(width) * height * 4);
    return store.driver()->readPixels(target, rect, out.data());
}

bool expectUniform(TextureStore& store, TextureId id, unsigned width, unsigned height, Rgba color,
        const char* label)
{
    Bytes pixels;
    if (!read(store, id, width, height, pixels))
    {
        printf("%s: readPixels failed\n", label);
        return false;
    }
    for (std::size_t i = 0; i < pixels.size(); i += 4)
    {
        if (pixels[i] != color.r || pixels[i + 1] != color.g || pixels[i + 2] != color.b ||
                pixels[i + 3] != color.a)
        {
            printf("%s: texel %zu is %d,%d,%d,%d, expected %d,%d,%d,%d\n", label, i / 4, pixels[i],
                    pixels[i + 1], pixels[i + 2], pixels[i + 3], color.r, color.g, color.b,
                    color.a);
            return false;
        }
    }
    return true;
}

bool savePng(const char* name, int width, int height, Rgba color)
{
    Pixmap pixmap(width, height, 4);
    pixmap.fill(static_cast<std::uint8_t>(color.r), static_cast<std::uint8_t>(color.g),
            static_cast<std::uint8_t>(color.b), static_cast<std::uint8_t>(color.a));
    return pixmap.save(name);
}

std::uint64_t secondsToNanos(unsigned seconds)
{
    return static_cast<std::uint64_t>(seconds) * 1000000000ull;
}

bool drain(AssetLoader& loader, unsigned seconds)
{
    const std::uint64_t start = time_nanos();
    while (loader.pendingRequests() != 0)
    {
        if (time_nanos() - start > secondsToNanos(seconds)) return false;
        loader.pump();
        time_sleep(1);
    }
    return true;
}

} // namespace

int main(int argc, char** argv)
{
    const bool vulkan = argc > 1 && strcmp(argv[1], "vulkan") == 0;

    CHECK(Device::headlessSupported());
    if (!Device::headlessSupported()) return 1;

    DeviceDesc deviceDesc;
    deviceDesc.title = "asset loader test";
    deviceDesc.driver = vulkan ? prisma::DriverType::Vulkan : prisma::DriverType::OpenGL;
    deviceDesc.width = 64;
    deviceDesc.height = 48;
    deviceDesc.headless = true;

    Device* device = Device::create(deviceDesc);
    CHECK(device != nullptr);
    if (!device) return 1;

    printf("%s\n", vulkan ? "Vulkan" : "OpenGL");

    FileSystem& fileSystem = FileSystem::getSingleton();
    const char* directory =
            vulkan ? PRISMA3D_TEST_DIR "/asset_loader_vulkan" : PRISMA3D_TEST_DIR "/asset_loader";
    fs_create_directory(directory);
    fileSystem.addSearchPath(directory);

    CHECK(savePng(path(directory, "red.png").c_str(), 4, 4, kRed));

    Pixmap gradient(8, 8, 3);
    for (int y = 0; y < 8; ++y)
        for (int x = 0; x < 8; ++x)
        {
            std::uint8_t* texel = gradient.pixels + (y * 8 + x) * 3;
            texel[0] = static_cast<std::uint8_t>(x * 30);
            texel[1] = static_cast<std::uint8_t>(y * 30);
            texel[2] = static_cast<std::uint8_t>((x + y) * 15);
        }
    CHECK(gradient.save(path(directory, "gradient.png").c_str()));

    Bytes ddsFile;
    ddsFile.resize(sizeof(dds::Header) + 16);
    dds::write_header(ddsFile.data(), dds::DXGI_FORMAT_R8G8B8A8_UNORM, 2, 2, 1, 1, false);
    for (int i = 0; i < 4; ++i)
    {
        std::uint8_t* texel = ddsFile.data() + sizeof(dds::Header) + i * 4;
        texel[0] = 0;
        texel[1] = 0;
        texel[2] = 255;
        texel[3] = 255;
    }
    CHECK(file_write(path(directory, "blue.dds").c_str(), ddsFile.data(), ddsFile.size()));

    const std::uint8_t garbage[64] = { 9, 8, 7, 6, 5, 4, 3, 2, 1 };
    CHECK(file_write(path(directory, "broken.png").c_str(), garbage, sizeof(garbage)));

    const char* budgetNames[3] = { "budget0.png", "budget1.png", "budget2.png" };
    const ct::String budgetPaths[3] = { path(directory, "budget0.png"),
        path(directory, "budget1.png"), path(directory, "budget2.png") };
    const char* limitNames[3] = { "limit0.png", "limit1.png", "limit2.png" };
    const ct::String limitPaths[3] = { path(directory, "limit0.png"), path(directory, "limit1.png"),
        path(directory, "limit2.png") };
    for (int i = 0; i < 3; ++i)
    {
        CHECK(savePng(budgetPaths[i].c_str(), 4, 4, Rgba{ 10 * (i + 1), 20, 30, 255 }));
        CHECK(savePng(limitPaths[i].c_str(), 2, 2, Rgba{ 40, 50 * (i + 1), 60, 255 }));
    }

    {
        TextureStore store(device->driver());
        MeshStore meshes(device->driver());
        MaterialStore materials(store);
        ModelLoader importers;
        {
            AssetLoader loader(store, materials, meshes, importers, 2);
            CHECK(AssetLoader::singleton() == &loader);
            CHECK(loader.totalRequests() == 0);
            CHECK(loader.completedRequests() == 0);
            CHECK(loader.pendingRequests() == 0);
            CHECK(loader.uploadedBytes() == 0);

            const std::uint64_t idleStart = time_nanos();
            loader.waitAll();
            loader.pump();
            CHECK(time_nanos() - idleStart < secondsToNanos(1));
            CHECK(loader.completedRequests() == 0);

            CHECK(!loader.requestTexture("", true).valid());
            CHECK(loader.totalRequests() == 0);

            std::uint64_t expectedBytes = 0;

            const TextureId red = loader.requestTexture("red.png", true);
            CHECK(red.valid());
            CHECK(store.valid(red));
            CHECK(loader.textureStatus(red) == LoadStatus::InProgress);
            CHECK(loader.status("red.png", true) == LoadStatus::InProgress);
            CHECK(loader.totalRequests() == 1);
            CHECK(loader.pendingRequests() == 1);
            const TextureInfo* pending = store.get(red);
            CHECK(pending != nullptr);
            if (pending)
            {
                CHECK(pending->state == TextureState::Pending);
                CHECK(pending->srgb);
                CHECK(pending->width == 1 && pending->height == 1);
                CHECK(strcmp(pending->path.c_str(), "red.png") == 0);
            }
            CHECK(expectUniform(store, red, 1, 1, kMagenta, "pending red"));

            const TextureId again = loader.requestTexture("red.png", true);
            CHECK(again == red);
            CHECK(loader.totalRequests() == 1);
            const TextureId redLinear = loader.requestTexture("red.png", false);
            CHECK(redLinear.valid());
            CHECK(redLinear != red);
            CHECK(loader.totalRequests() == 2);
            const TextureId redNoMips = loader.requestTexture("red.png", true, false);
            CHECK(redNoMips.valid());
            CHECK(redNoMips != red);
            CHECK(redNoMips != redLinear);
            CHECK(loader.totalRequests() == 3);
            expectedBytes += 3 * 64;

            const TextureId gradientId = loader.requestTexture("gradient.png", false);
            const TextureId blue = loader.requestTexture("blue.dds", false);
            const TextureId broken = loader.requestTexture("broken.png", true);
            const TextureId missing = loader.requestTexture("missing.png", true);
            CHECK(gradientId.valid() && blue.valid() && broken.valid() && missing.valid());
            CHECK(loader.totalRequests() == 7);
            expectedBytes += 8 * 8 * 4;
            expectedBytes += ddsFile.size();

            CHECK(drain(loader, 10));
            CHECK(loader.pendingRequests() == 0);
            CHECK(loader.completedRequests() == 7);

            CHECK(loader.textureStatus(red) == LoadStatus::Loaded);
            CHECK(loader.textureStatus(redLinear) == LoadStatus::Loaded);
            CHECK(loader.textureStatus(redNoMips) == LoadStatus::Loaded);
            CHECK(loader.textureStatus(gradientId) == LoadStatus::Loaded);
            CHECK(loader.textureStatus(blue) == LoadStatus::Loaded);
            CHECK(loader.textureStatus(broken) == LoadStatus::Failed);
            CHECK(loader.textureStatus(missing) == LoadStatus::Failed);
            CHECK(loader.textureStatus(TextureId()) == LoadStatus::Invalid);
            CHECK(loader.status("red.png", true) == LoadStatus::Loaded);
            CHECK(loader.status("red.png", false) == LoadStatus::Loaded);
            CHECK(loader.status("broken.png", true) == LoadStatus::Failed);
            CHECK(loader.status("broken.png", false) == LoadStatus::Invalid);
            CHECK(loader.status("nothing", true) == LoadStatus::Invalid);

            CHECK(store.valid(red) && store.valid(gradientId) && store.valid(blue));
            CHECK(store.valid(broken) && store.valid(missing));

            const TextureInfo* redInfo = store.get(red);
            CHECK(redInfo != nullptr);
            if (redInfo)
            {
                CHECK(redInfo->state == TextureState::Ready);
                CHECK(redInfo->srgb);
                CHECK(redInfo->width == 4 && redInfo->height == 4);
                CHECK(redInfo->mipLevels > 1);
                CHECK(redInfo->format == prisma::TextureFormat::RGBA8Srgb);
                CHECK(strcmp(redInfo->path.c_str(), "red.png") == 0);
            }
            CHECK(expectUniform(store, red, 4, 4, kRed, "red"));

            const TextureInfo* noMipsInfo = store.get(redNoMips);
            CHECK(noMipsInfo != nullptr);
            if (noMipsInfo)
            {
                CHECK(noMipsInfo->mipLevels == 1);
                CHECK(noMipsInfo->width == 4 && noMipsInfo->height == 4);
                CHECK(noMipsInfo->state == TextureState::Ready);
            }
            const TextureInfo* linearInfo = store.get(redLinear);
            CHECK(linearInfo != nullptr);
            if (linearInfo)
            {
                CHECK(!linearInfo->srgb);
                CHECK(linearInfo->format == prisma::TextureFormat::RGBA8);
            }
            CHECK(expectUniform(store, redLinear, 4, 4, kRed, "red linear"));
            CHECK(expectUniform(store, redNoMips, 4, 4, kRed, "red no mips"));

            const TextureInfo* gradientInfo = store.get(gradientId);
            CHECK(gradientInfo != nullptr);
            if (gradientInfo)
            {
                CHECK(gradientInfo->width == 8 && gradientInfo->height == 8);
                CHECK(gradientInfo->state == TextureState::Ready);
                CHECK(strcmp(gradientInfo->path.c_str(), "gradient.png") == 0);
            }
            Bytes pixels;
            CHECK(read(store, gradientId, 8, 8, pixels));
            bool gradientOk = pixels.size() == 8 * 8 * 4;
            for (int row = 0; gradientOk && row < 8; ++row)
                for (int x = 0; x < 8; ++x)
                {
                    const int y = row;
                    const std::uint8_t* got = pixels.data() + (row * 8 + x) * 4;
                    const std::uint8_t* want = gradient.pixels + (y * 8 + x) * 3;
                    if (got[0] != want[0] || got[1] != want[1] || got[2] != want[2] ||
                            got[3] != 255)
                    {
                        printf("gradient: texel %d,%d is %d,%d,%d,%d\n", x, y, got[0], got[1],
                                got[2], got[3]);
                        gradientOk = false;
                        break;
                    }
                }
            CHECK(gradientOk);

            const TextureInfo* blueInfo = store.get(blue);
            CHECK(blueInfo != nullptr);
            if (blueInfo)
            {
                CHECK(blueInfo->width == 2 && blueInfo->height == 2);
                CHECK(blueInfo->state == TextureState::Ready);
                CHECK(strcmp(blueInfo->path.c_str(), "blue.dds") == 0);
            }
            CHECK(expectUniform(store, blue, 2, 2, kBlue, "blue"));

            CHECK(store.get(broken) != nullptr && store.get(broken)->state == TextureState::Failed);
            CHECK(store.get(missing) != nullptr &&
                    store.get(missing)->state == TextureState::Failed);
            CHECK(store.get(broken)->width == 1 && store.get(missing)->width == 1);
            CHECK(strcmp(store.get(broken)->path.c_str(), "broken.png") == 0);
            CHECK(expectUniform(store, broken, 1, 1, kMagenta, "broken"));
            CHECK(expectUniform(store, missing, 1, 1, kMagenta, "missing"));

            CHECK(loader.uploadedBytes() == expectedBytes);
            CHECK(loader.uploadedBytes() > 0);

            const std::uint32_t totalBefore = loader.totalRequests();
            TextureId budget[3];
            for (int i = 0; i < 3; ++i)
            {
                budget[i] = loader.requestTexture(budgetNames[i], false, false);
                CHECK(budget[i].valid());
                expectedBytes += 64;
            }
            CHECK(loader.totalRequests() == totalBefore + 3);
            CHECK(loader.pendingRequests() == 3);

            const std::uint64_t budgetStart = time_nanos();
            std::uint32_t done = 0;
            bool timedOut = false;
            while (done == 0)
            {
                if (time_nanos() - budgetStart > secondsToNanos(10))
                {
                    timedOut = true;
                    break;
                }
                const std::uint32_t before = loader.completedRequests();
                loader.pump(1, 0);
                const std::uint32_t delta = loader.completedRequests() - before;
                CHECK(delta <= 1);
                done += delta;
                if (delta == 0) time_sleep(1);
            }
            CHECK(!timedOut);
            time_sleep(300);
            for (int i = 0; i < 2 && !timedOut; ++i)
            {
                const std::uint32_t before = loader.completedRequests();
                for (int rows = 0; rows < 4; ++rows)
                {
                    const std::uint32_t during = loader.completedRequests();
                    loader.pump(1, 0);
                    CHECK(loader.completedRequests() - during == (rows == 3 ? 1u : 0u));
                }
                CHECK(loader.completedRequests() - before == 1);
            }
            CHECK(loader.pendingRequests() == 0);
            for (int i = 0; i < 3; ++i)
            {
                CHECK(loader.textureStatus(budget[i]) == LoadStatus::Loaded);
                CHECK(expectUniform(store, budget[i], 4, 4, Rgba{ 10 * (i + 1), 20, 30, 255 },
                        "budget"));
            }

            TextureId limited[3];
            for (int i = 0; i < 3; ++i)
            {
                limited[i] = loader.requestTexture(limitNames[i], false, true);
                CHECK(limited[i].valid());
                expectedBytes += 16;
            }
            CHECK(loader.pendingRequests() == 3);
            const std::uint64_t limitStart = time_nanos();
            while (loader.pendingRequests() == 3)
            {
                if (time_nanos() - limitStart > secondsToNanos(10)) break;
                const std::uint32_t before = loader.completedRequests();
                loader.pump(~0ull, 1);
                CHECK(loader.completedRequests() - before <= 1);
                time_sleep(1);
            }
            CHECK(loader.pendingRequests() == 2);
            time_sleep(300);
            const std::uint32_t beforeLimit = loader.completedRequests();
            loader.pump(~0ull, 2);
            CHECK(loader.completedRequests() - beforeLimit == 2);
            CHECK(loader.pendingRequests() == 0);
            for (int i = 0; i < 3; ++i)
            {
                CHECK(loader.textureStatus(limited[i]) == LoadStatus::Loaded);
                CHECK(expectUniform(store, limited[i], 2, 2, Rgba{ 40, 50 * (i + 1), 60, 255 },
                        "limited"));
            }

            CHECK(loader.uploadedBytes() == expectedBytes);
            CHECK(loader.completedRequests() == loader.totalRequests());
            CHECK(loader.pendingRequests() == 0);

            const std::uint64_t waitStart = time_nanos();
            loader.waitAll();
            CHECK(time_nanos() - waitStart < secondsToNanos(1));

            loader.resetCounters();
            CHECK(loader.totalRequests() == 0);
            CHECK(loader.completedRequests() == 0);
            CHECK(loader.pendingRequests() == 0);
            CHECK(loader.uploadedBytes() == expectedBytes);
            CHECK(loader.requestTexture("red.png", true) == red);
            CHECK(loader.totalRequests() == 0);

            const TextureId late = loader.requestTexture("limit0.png", true, true);
            CHECK(late.valid());
            CHECK(loader.totalRequests() == 1);
            loader.resetCounters();
            CHECK(loader.totalRequests() == 1);
            CHECK(loader.completedRequests() == 0);
            loader.waitAll();
            CHECK(loader.pendingRequests() == 0);
            CHECK(loader.completedRequests() == 1);
            CHECK(loader.textureStatus(late) == LoadStatus::Loaded);
        }
        CHECK(AssetLoader::singleton() == nullptr);

        {
            AssetLoader quick(store, materials, meshes, importers, 1);
            const TextureId left = quick.requestTexture("red.png", false, false);
            const TextureId leftBroken = quick.requestTexture("broken.png", false, false);
            CHECK(left.valid() && leftBroken.valid());
        }
        CHECK(AssetLoader::singleton() == nullptr);
    }

    fileSystem.removeSearchPath(directory);
    Device::destroy(device);
    if (failures) printf("%d failures\n", failures);
    return failures == 0 ? 0 : 1;
}
