#include "Check.h"

#include "prisma3d/Device.h"
#include "prisma3d/core/FileSystem.h"
#include "prisma3d/core/Pixmap.h"
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

struct Rgba
{
    int r;
    int g;
    int b;
    int a;
};

void appendTexels(Bytes& bytes, unsigned count, Rgba color)
{
    for (unsigned i = 0; i < count; ++i)
    {
        bytes.push_back(static_cast<std::uint8_t>(color.r));
        bytes.push_back(static_cast<std::uint8_t>(color.g));
        bytes.push_back(static_cast<std::uint8_t>(color.b));
        bytes.push_back(static_cast<std::uint8_t>(color.a));
    }
}

Bytes makeDds(dds::DXGI_FORMAT format, unsigned width, unsigned height, const Bytes& payload,
        unsigned mips = 1, unsigned arraySize = 1, bool cubemap = false)
{
    Bytes file;
    file.resize(sizeof(dds::Header) + payload.size());
    dds::write_header(file.data(), format, width, height, mips, arraySize, cubemap);
    if (payload.size() > 0)
        memcpy(file.data() + sizeof(dds::Header), payload.data(), payload.size());
    return file;
}

bool read(TextureStore& store, TextureId id, unsigned mip, unsigned width, unsigned height,
        Bytes& out)
{
    prisma::RenderTarget target;
    target.texture = store.gpu(id);
    target.mip = mip;
    prisma::Rect rect;
    rect.x = 0;
    rect.y = 0;
    rect.width = width;
    rect.height = height;
    out.resize(static_cast<std::size_t>(width) * height * 4);
    return store.driver()->readPixels(target, rect, out.data());
}

bool near(int a, int b, int tolerance)
{
    const int d = a - b;
    return d >= -tolerance && d <= tolerance;
}

bool expectUniform(TextureStore& store, TextureId id, unsigned mip, unsigned width, unsigned height,
        Rgba color, int tolerance, const char* label)
{
    Bytes pixels;
    if (!read(store, id, mip, width, height, pixels))
    {
        printf("%s: readPixels failed\n", label);
        return false;
    }
    for (std::size_t i = 0; i < pixels.size(); i += 4)
    {
        if (!near(pixels[i], color.r, tolerance) || !near(pixels[i + 1], color.g, tolerance) ||
                !near(pixels[i + 2], color.b, tolerance) ||
                !near(pixels[i + 3], color.a, tolerance))
        {
            printf("%s: texel %zu is %d,%d,%d,%d, expected %d,%d,%d,%d\n", label, i / 4, pixels[i],
                    pixels[i + 1], pixels[i + 2], pixels[i + 3], color.r, color.g, color.b,
                    color.a);
            return false;
        }
    }
    return true;
}

bool sameBytes(const Bytes& a, const std::uint8_t* b, std::size_t size)
{
    return a.size() == size && memcmp(a.data(), b, size) == 0;
}

prisma::TextureDesc desc2x2(const std::uint8_t* data)
{
    prisma::TextureDesc desc;
    desc.format = prisma::TextureFormat::RGBA8;
    desc.width = 2;
    desc.height = 2;
    desc.mipLevels = 1;
    desc.data = data;
    desc.debugName = "quad";
    return desc;
}

const Rgba kRed = { 255, 0, 0, 255 };
const Rgba kGreen = { 0, 255, 0, 255 };

} // namespace

int main(int argc, char** argv)
{
    const bool vulkan = argc > 1 && strcmp(argv[1], "vulkan") == 0;

    CHECK(Device::headlessSupported());
    if (!Device::headlessSupported()) return 1;

    DeviceDesc deviceDesc;
    deviceDesc.title = "texture store test";
    deviceDesc.driver = vulkan ? prisma::DriverType::Vulkan : prisma::DriverType::OpenGL;
    deviceDesc.width = 64;
    deviceDesc.height = 48;
    deviceDesc.headless = true;

    Device* device = Device::create(deviceDesc);
    CHECK(device != nullptr);
    if (!device) return 1;

    prisma::Driver* driver = device->driver();
    const bool hasBc = driver->caps().textureBC;
    printf("%s: textureBC %d, cubeArrays %d\n", vulkan ? "Vulkan" : "OpenGL", hasBc,
            driver->caps().cubeArrays);

    {
        TextureStore store(driver);
        CHECK(TextureStore::singleton() == &store);
        CHECK(store.driver() == driver);
        CHECK(store.count() == 0);
        CHECK(!store.valid(TextureId()));
        CHECK(store.get(TextureId()) == nullptr);
        CHECK(!store.gpu(TextureId()).valid());

        {
            const std::uint8_t texels[16] = { 255, 0, 0, 255, 0, 255, 0, 255, 0, 0, 255, 255, 10,
                20, 30, 40 };
            const TextureId id = store.create(desc2x2(texels), true);
            CHECK(id.valid());
            CHECK(store.valid(id));
            CHECK(store.count() == 1);
            const TextureInfo* info = store.get(id);
            CHECK(info != nullptr);
            if (info)
            {
                CHECK(info->gpu.valid());
                CHECK(info->type == prisma::TextureType::Texture2D);
                CHECK(info->format == prisma::TextureFormat::RGBA8);
                CHECK(info->width == 2 && info->height == 2 && info->depth == 1);
                CHECK(info->mipLevels == 1 && info->layers == 1);
                CHECK(info->srgb);
                CHECK(info->state == TextureState::Ready);
                CHECK(strcmp(info->name.c_str(), "quad") == 0);
                CHECK(info->path.empty());
            }
            Bytes pixels;
            CHECK(read(store, id, 0, 2, 2, pixels));
            const std::uint8_t topDown[16] = { 255, 0, 0, 255, 0, 255, 0, 255, 0, 0, 255, 255, 10,
                20, 30, 40 };
            CHECK(sameBytes(pixels, topDown, 16));
            store.setState(id, TextureState::Pending);
            CHECK(store.get(id)->state == TextureState::Pending);
            store.setPath(id, "renamed");
            CHECK(strcmp(store.get(id)->path.c_str(), "renamed") == 0);

            prisma::TextureDesc bad;
            bad.width = 0;
            bad.height = 0;
            const TextureId invalid = store.create(bad);
            CHECK(!invalid.valid());
            CHECK(store.count() == 1);
            store.destroy(id);
            CHECK(store.count() == 0);
        }

        Bytes rgbaDds;
        {
            Bytes payload;
            appendTexels(payload, 16, Rgba{ 255, 0, 0, 255 });
            appendTexels(payload, 4, Rgba{ 0, 255, 0, 255 });
            appendTexels(payload, 1, Rgba{ 0, 0, 255, 255 });
            rgbaDds = makeDds(dds::DXGI_FORMAT_R8G8B8A8_UNORM, 4, 4, payload, 3);
            const TextureId id =
                    store.createFromDds(rgbaDds.data(), rgbaDds.size(), false, false, "mips");
            CHECK(id.valid());
            const TextureInfo* info = store.get(id);
            CHECK(info != nullptr);
            if (info)
            {
                CHECK(info->mipLevels == 3);
                CHECK(info->width == 4 && info->height == 4);
                CHECK(info->format == prisma::TextureFormat::RGBA8);
                CHECK(info->layers == 1);
            }
            CHECK(expectUniform(store, id, 0, 4, 4, kRed, 0, "dds mip0"));
            CHECK(expectUniform(store, id, 1, 2, 2, kGreen, 0, "dds mip1"));
            CHECK(expectUniform(store, id, 2, 1, 1, Rgba{ 0, 0, 255, 255 }, 0, "dds mip2"));
            store.destroy(id);

            const TextureId skipped =
                    store.createFromDds(rgbaDds.data(), rgbaDds.size(), false, false, "skip", 1);
            CHECK(skipped.valid());
            const TextureInfo* skipInfo = store.get(skipped);
            CHECK(skipInfo != nullptr);
            if (skipInfo)
            {
                CHECK(skipInfo->mipLevels == 2);
                CHECK(skipInfo->width == 2 && skipInfo->height == 2);
            }
            CHECK(expectUniform(store, skipped, 0, 2, 2, kGreen, 0, "skip mip0"));
            CHECK(expectUniform(store, skipped, 1, 1, 1, Rgba{ 0, 0, 255, 255 }, 0, "skip mip1"));
            store.destroy(skipped);

            const TextureId generated =
                    store.createFromDds(rgbaDds.data(), rgbaDds.size(), true, true, "keep");
            CHECK(generated.valid());
            if (generated.valid())
            {
                CHECK(store.get(generated)->mipLevels == 3);
                CHECK(store.get(generated)->srgb);
                CHECK(store.get(generated)->format == prisma::TextureFormat::RGBA8Srgb);
            }
            store.destroy(generated);
        }

        {
            Bytes single;
            appendTexels(single, 1, Rgba{ 200, 100, 50, 255 });
            Bytes payload;
            appendTexels(payload, 4, Rgba{ 200, 100, 50, 77 });
            const Bytes bgra = makeDds(dds::DXGI_FORMAT_B8G8R8A8_UNORM, 2, 2, payload);
            const TextureId id = store.createFromDds(bgra.data(), bgra.size(), false, false);
            CHECK(id.valid());
            CHECK(expectUniform(store, id, 0, 2, 2, Rgba{ 50, 100, 200, 77 }, 0, "bgra"));
            store.destroy(id);

            const Bytes bgrx = makeDds(dds::DXGI_FORMAT_B8G8R8X8_UNORM, 2, 2, payload);
            const TextureId opaque = store.createFromDds(bgrx.data(), bgrx.size(), false, false);
            CHECK(opaque.valid());
            CHECK(expectUniform(store, opaque, 0, 2, 2, Rgba{ 50, 100, 200, 255 }, 0, "bgrx"));
            store.destroy(opaque);

            const TextureId mipped = store.createFromDds(bgra.data(), bgra.size(), false, true);
            CHECK(mipped.valid());
            CHECK(expectUniform(store, mipped, 0, 2, 2, Rgba{ 50, 100, 200, 77 }, 0,
                    "bgra generated"));
            store.destroy(mipped);
        }

        {
            Bytes block;
            const std::uint8_t bytes[8] = { 0x00, 0xF8, 0xE0, 0x07, 0, 0, 0, 0 };
            for (int i = 0; i < 8; ++i) block.push_back(bytes[i]);
            const Bytes bc1 = makeDds(dds::DXGI_FORMAT_BC1_UNORM, 4, 4, block);

            const TextureId native =
                    store.createFromDds(bc1.data(), bc1.size(), false, false, "bc1");
            CHECK(native.valid());
            if (native.valid())
            {
                CHECK(store.get(native)->width == 4);
                CHECK(hasBc ? store.get(native)->format == prisma::TextureFormat::BC1
                            : store.get(native)->format == prisma::TextureFormat::RGBA8);
                if (!hasBc)
                    CHECK(expectUniform(store, native, 0, 4, 4, kRed, 1, "bc1 decoded fallback"));
            }
            store.destroy(native);

            const TextureId decoded =
                    store.createFromDds(bc1.data(), bc1.size(), true, true, "bc1 decoded");
            CHECK(decoded.valid());
            if (decoded.valid())
            {
                CHECK(store.get(decoded)->format == prisma::TextureFormat::RGBA8Srgb);
                CHECK(store.get(decoded)->mipLevels == 3);
                CHECK(expectUniform(store, decoded, 0, 4, 4, kRed, 1, "bc1 decoded"));
            }
            store.destroy(decoded);

            const Bytes bc4 = makeDds(dds::DXGI_FORMAT_BC4_UNORM, 4, 4, block);
            const TextureId other = store.createFromDds(bc4.data(), bc4.size(), false, false);
            CHECK(other.valid() == hasBc);
            store.destroy(other);
        }

        {
            Bytes payload;
            for (int face = 0; face < 6; ++face)
                appendTexels(payload, 4, Rgba{ face * 40, 0, 0, 255 });
            const Bytes cube = makeDds(dds::DXGI_FORMAT_R8G8B8A8_UNORM, 2, 2, payload, 1, 6, true);
            const TextureId id = store.createFromDds(cube.data(), cube.size(), false, false);
            CHECK(id.valid());
            const TextureInfo* info = store.get(id);
            CHECK(info != nullptr);
            if (info)
            {
                CHECK(info->type == prisma::TextureType::TextureCube);
                CHECK(info->layers == 6);
            }
            store.destroy(id);

            const Bytes array = makeDds(dds::DXGI_FORMAT_R8G8B8A8_UNORM, 2, 2, payload, 1, 6);
            const TextureId arrayId = store.createFromDds(array.data(), array.size(), false, false);
            CHECK(arrayId.valid());
            if (arrayId.valid())
            {
                CHECK(store.get(arrayId)->type == prisma::TextureType::Texture2DArray);
                CHECK(store.get(arrayId)->layers == 6);
            }
            store.destroy(arrayId);

            Bytes bigPayload;
            for (int face = 0; face < 12; ++face)
                appendTexels(bigPayload, 4, Rgba{ 0, face * 20, 0, 255 });
            const Bytes cubeArray =
                    makeDds(dds::DXGI_FORMAT_R8G8B8A8_UNORM, 2, 2, bigPayload, 1, 12, true);
            const TextureId big =
                    store.createFromDds(cubeArray.data(), cubeArray.size(), false, false);
            CHECK(big.valid() == driver->caps().cubeArrays);
            if (big.valid())
            {
                CHECK(store.get(big)->type == prisma::TextureType::TextureCubeArray);
                CHECK(store.get(big)->layers == 12);
            }
            store.destroy(big);
        }

        {
            const ct::String folder = ct::String(PRISMA3D_TEST_DIR) +
                                      (vulkan ? "/texture_store_vulkan" : "/texture_store");
            fs_create_directory(folder.c_str());
            Pixmap pixmap(2, 2, 4);
            pixmap.set_pixel(0, 0, 255, 0, 0, 255);
            pixmap.set_pixel(1, 0, 0, 255, 0, 255);
            pixmap.set_pixel(0, 1, 0, 0, 255, 255);
            pixmap.set_pixel(1, 1, 9, 8, 7, 255);
            CHECK(pixmap.save((folder + "/image.png").c_str()));

            std::size_t size = 0;
            std::uint8_t* png = file_read((folder + "/image.png").c_str(), &size);
            CHECK(png != nullptr && size > 0);
            if (png)
            {
                const TextureId id = store.createFromImage(png, size, false, false, false, "png");
                CHECK(id.valid());
                if (id.valid())
                {
                    CHECK(store.get(id)->width == 2 && store.get(id)->height == 2);
                    CHECK(store.get(id)->format == prisma::TextureFormat::RGBA8);
                    Bytes pixels;
                    CHECK(read(store, id, 0, 2, 2, pixels));
                    const std::uint8_t expected[16] = { 255, 0, 0, 255, 0, 255, 0, 255, 0, 0, 255,
                        255, 9, 8, 7, 255 };
                    CHECK(sameBytes(pixels, expected, 16));
                }
                store.destroy(id);

                const TextureId mipped = store.createFromImage(png, size, true, true, false);
                CHECK(mipped.valid());
                if (mipped.valid())
                {
                    CHECK(store.get(mipped)->format == prisma::TextureFormat::RGBA8Srgb);
                    CHECK(store.get(mipped)->mipLevels == 2);
                }
                store.destroy(mipped);
                fs_free(png);
            }

            Bytes payload;
            appendTexels(payload, 4, Rgba{ 0, 255, 0, 255 });
            const Bytes file = makeDds(dds::DXGI_FORMAT_R8G8B8A8_UNORM, 2, 2, payload);
            CHECK(file_write((folder + "/green.dds").c_str(), file.data(), file.size()));

            FileSystem& fileSystem = FileSystem::getSingleton();
            fileSystem.addSearchPath(folder);

            const TextureId fromPng = store.createFromFile("image.png", false, false);
            CHECK(fromPng.valid());
            if (fromPng.valid())
            {
                CHECK(strcmp(store.get(fromPng)->path.c_str(), "image.png") == 0);
                CHECK(store.get(fromPng)->width == 2);
            }
            const TextureId fromDds = store.createFromFile("green.dds", false, false);
            CHECK(fromDds.valid());
            if (fromDds.valid())
            {
                CHECK(strcmp(store.get(fromDds)->path.c_str(), "green.dds") == 0);
                CHECK(expectUniform(store, fromDds, 0, 2, 2, kGreen, 0, "file dds"));
            }
            store.destroy(fromPng);
            store.destroy(fromDds);
            fileSystem.removeSearchPath(folder);
        }

        {
            Pixmap rgb(2, 2, 3);
            for (int y = 0; y < 2; ++y)
                for (int x = 0; x < 2; ++x)
                {
                    rgb.pixels[(y * 2 + x) * 3] = 10;
                    rgb.pixels[(y * 2 + x) * 3 + 1] = 20;
                    rgb.pixels[(y * 2 + x) * 3 + 2] = 30;
                }
            const TextureId a = store.createFromPixmap(rgb, false, false, "rgb");
            CHECK(a.valid());
            CHECK(expectUniform(store, a, 0, 2, 2, Rgba{ 10, 20, 30, 255 }, 0, "pixmap rgb"));
            store.destroy(a);

            Pixmap rgba(4, 4, 4);
            rgba.fill(1, 2, 3, 4);
            const TextureId b = store.createFromPixmap(rgba, false, true, "rgba");
            CHECK(b.valid());
            if (b.valid()) CHECK(store.get(b)->mipLevels == 3);
            CHECK(expectUniform(store, b, 0, 4, 4, Rgba{ 1, 2, 3, 4 }, 0, "pixmap rgba"));
            store.destroy(b);

            Pixmap empty;
            CHECK(!store.createFromPixmap(empty, false, false).valid());
        }

        {
            const std::uint8_t red[4] = { 255, 0, 0, 255 };
            const std::uint8_t green[4] = { 0, 255, 0, 255 };
            prisma::TextureDesc descA = desc2x2(red);
            descA.width = 1;
            descA.height = 1;
            descA.debugName = "a";
            prisma::TextureDesc descB = descA;
            descB.data = green;
            descB.debugName = "b";
            const TextureId a = store.create(descA);
            const TextureId b = store.create(descB);
            const std::size_t before = store.count();
            const prisma::TextureHandle greenGpu = store.gpu(b);
            CHECK(expectUniform(store, a, 0, 1, 1, kRed, 0, "replace before"));
            CHECK(!store.replace(a, a));
            CHECK(!store.replace(a, TextureId()));
            CHECK(store.replace(a, b));
            CHECK(!store.valid(b));
            CHECK(store.count() == before - 1);
            CHECK(store.gpu(a) == greenGpu);
            CHECK(strcmp(store.get(a)->name.c_str(), "a") == 0);
            CHECK(expectUniform(store, a, 0, 1, 1, kGreen, 0, "replace after"));
            store.destroy(a);
        }

        {
            const TextureId white = store.white();
            CHECK(white.valid());
            CHECK(store.white() == white);
            CHECK(expectUniform(store, white, 0, 1, 1, Rgba{ 255, 255, 255, 255 }, 0, "white"));
            CHECK(strcmp(store.get(white)->name.c_str(), "default.white") == 0);
            CHECK(expectUniform(store, store.black(), 0, 1, 1, Rgba{ 0, 0, 0, 255 }, 0, "black"));
            CHECK(expectUniform(store, store.normal(), 0, 1, 1, Rgba{ 128, 128, 255, 255 }, 0,
                    "normal"));
            CHECK(expectUniform(store, store.placeholder(), 0, 1, 1, Rgba{ 255, 0, 255, 255 }, 0,
                    "placeholder"));
            CHECK(store.count() == 4);
            store.destroy(white);
            CHECK(!store.valid(white));
            CHECK(store.white().valid());
            CHECK(store.count() == 4);
        }

        {
            const std::size_t before = store.count();
            const std::uint8_t garbage[64] = { 1, 2, 3, 4, 5, 6, 7, 8 };
            CHECK(!store.createFromDds(garbage, sizeof(garbage), false, false).valid());
            CHECK(!store.createFromDds(garbage, 0, false, false).valid());
            CHECK(!store.createFromImage(garbage, sizeof(garbage), false, false, false).valid());
            CHECK(!store.createFromImage(garbage, sizeof(garbage), false, false, true).valid());
            CHECK(!store.createFromFile("missing_texture.png", false, false).valid());
            CHECK(!store.createFromFile("missing_texture.dds", false, false).valid());
            store.destroy(TextureId());
            const std::uint8_t pixel[4] = { 1, 2, 3, 4 };
            prisma::TextureDesc one = desc2x2(pixel);
            one.width = 1;
            one.height = 1;
            const TextureId id = store.create(one);
            store.destroy(id);
            store.destroy(id);
            CHECK(store.get(id) == nullptr);
            CHECK(!store.gpu(id).valid());
            CHECK(store.count() == before);
        }

        {
            const std::uint8_t red[4] = { 255, 0, 0, 255 };
            const std::uint8_t green[4] = { 0, 255, 0, 255 };
            prisma::TextureDesc descA = desc2x2(red);
            descA.width = 1;
            descA.height = 1;
            descA.debugName = "proxy a";
            prisma::TextureDesc descB = desc2x2(green);
            descB.width = 4;
            descB.height = 2;
            const std::uint8_t wide[32] = { 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255,
                0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255, 0, 255 };
            descB.data = wide;
            descB.debugName = "proxy b";
            const std::size_t before = store.count();
            const TextureId a = store.create(descA, true);
            const TextureId b = store.create(descB, false);
            CHECK(!store.proxyCreate(TextureId()).valid());
            const TextureId proxy = store.proxyCreate(a);
            CHECK(proxy.valid());
            CHECK(!store.proxyCreate(proxy).valid());
            CHECK(store.count() == before + 3);
            CHECK(store.isProxy(proxy) && !store.isProxy(a) && !store.isProxy(TextureId()));
            CHECK(store.proxyTarget(proxy) == a);
            CHECK(store.valid(proxy));
            CHECK(store.gpu(proxy) == store.gpu(a));
            CHECK(store.get(proxy)->width == 1 && store.get(proxy)->srgb);

            CHECK(!store.proxyUpdate(a, b));
            CHECK(!store.proxyUpdate(proxy, proxy));
            CHECK(!store.proxyUpdate(proxy, TextureId()));
            CHECK(store.proxyTarget(proxy) == a);
            CHECK(store.proxyUpdate(proxy, b));
            CHECK(store.proxyTarget(proxy) == b);
            CHECK(store.gpu(proxy) == store.gpu(b));
            CHECK(store.get(proxy)->width == 4 && store.get(proxy)->height == 2);
            CHECK(!store.get(proxy)->srgb);
            CHECK(expectUniform(store, proxy, 0, 4, 2, kGreen, 0, "proxy shows the target"));

            CHECK(!store.replace(proxy, a));
            CHECK(!store.replace(a, proxy));
            CHECK(store.valid(proxy) && store.valid(a) && store.valid(b));

            const TextureId c = store.create(descA, true);
            const prisma::TextureHandle cGpu = store.gpu(c);
            CHECK(store.replace(b, c));
            CHECK(store.gpu(proxy) == cGpu);
            CHECK(store.proxyTarget(proxy) == b);

            store.destroy(b);
            CHECK(!store.valid(b));
            CHECK(!store.valid(proxy));
            CHECK(!store.gpu(proxy).valid());
            CHECK(!store.proxyTarget(proxy).valid());
            CHECK(store.get(proxy) != nullptr);
            CHECK(store.proxyUpdate(proxy, a));
            CHECK(store.valid(proxy));
            CHECK(store.gpu(proxy) == store.gpu(a));

            store.destroy(proxy);
            CHECK(!store.valid(proxy));
            CHECK(store.valid(a));
            CHECK(expectUniform(store, a, 0, 1, 1, kRed, 0, "target survives its proxy"));

            const TextureId second = store.proxyCreate(a);
            CHECK(second.valid());
            ct::Vector<ct::String> files;
            files.push_back("f0.png");
            files.push_back("f1.png");
            store.setSequence(second, files, 12.0f);
            CHECK(store.get(second)->sequence.size() == 2);
            CHECK(store.get(second)->sequenceFps == 12.0f);
            store.destroy(a);
            store.destroy(c);
            store.clear();
            CHECK(store.count() == 0);
            CHECK(!store.valid(second));
        }

        {
            const TextureId white = store.white();
            CHECK(white.valid());
            store.clear();
            CHECK(store.count() == 0);
            CHECK(!store.valid(white));
            const TextureId again = store.white();
            CHECK(again.valid());
            CHECK(store.count() == 1);
            CHECK(expectUniform(store, again, 0, 1, 1, Rgba{ 255, 255, 255, 255 }, 0,
                    "white again"));
            CHECK(store.black() != again);
            CHECK(store.count() == 2);
        }
    }
    CHECK(TextureStore::singleton() == nullptr);

    Device::destroy(device);
    if (failures) printf("%d failures\n", failures);
    return failures == 0 ? 0 : 1;
}
