#include "Check.h"

#include "prisma3d/Device.h"
#include "prisma3d/core/Pixmap.h"
#include "prisma3d/render/DecalAtlas.h"
#include "prisma3d/render/TextureStore.h"

#include <math.h>
#include <string.h>

using namespace prisma3d;

namespace
{

bool near(float a, float b)
{
    return fabsf(a - b) < 1e-6f;
}

bool nearRect(const Math::Vec4& rect, float x, float y, float w, float h)
{
    return near(rect.x, x) && near(rect.y, y) && near(rect.z, w) && near(rect.w, h);
}

DecalAtlas::Item makeItem(std::uint32_t width, std::uint32_t height)
{
    DecalAtlas::Item item;
    item.pixelWidth = width;
    item.pixelHeight = height;
    return item;
}

void testPack()
{
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    {
        ct::Vector<DecalAtlas::Item> items;
        DecalAtlas::pack(items, 5, width, height);
        CHECK(width == 32 && height == 32);
    }
    {
        ct::Vector<DecalAtlas::Item> items;
        items.push_back(makeItem(64, 64));
        items.push_back(makeItem(128, 64));
        items.push_back(makeItem(256, 256));
        DecalAtlas::pack(items, 5, width, height);
        CHECK(width == 512 && height == 512);
        CHECK(items[0].pixelWidth == 256 && items[0].width == 9 && items[0].height == 9);
        CHECK(items[0].x == 0 && items[0].y == 0);
        CHECK(items[1].pixelWidth == 128 && items[1].width == 5 && items[1].height == 3);
        CHECK(items[1].x == 9 && items[1].y == 0);
        CHECK(items[2].pixelWidth == 64 && items[2].width == 3 && items[2].height == 3);
        CHECK(items[2].x == 9 && items[2].y == 3);
        CHECK(nearRect(DecalAtlas::itemRect(items[0], 5, width, height), 0.03125f, 0.03125f, 0.5f,
                0.5f));
        CHECK(nearRect(DecalAtlas::itemRect(items[1], 5, width, height), 0.59375f, 0.03125f, 0.25f,
                0.125f));
        CHECK(nearRect(DecalAtlas::itemRect(items[2], 5, width, height), 0.59375f, 0.21875f, 0.125f,
                0.125f));
    }
    {
        ct::Vector<DecalAtlas::Item> items;
        for (int i = 0; i < 40; ++i) items.push_back(makeItem(64, 64));
        DecalAtlas::pack(items, 5, width, height);
        CHECK(width == 512 && height == 1024);
        CHECK(items[0].x == 0 && items[0].y == 0);
        CHECK(items[4].x == 12 && items[4].y == 0);
        CHECK(items[5].x == 0 && items[5].y == 3);
        CHECK(items[39].x == 12 && items[39].y == 21);
        for (std::size_t a = 0; a < items.size(); ++a)
        {
            for (std::size_t b = a + 1; b < items.size(); ++b)
            {
                const bool apart = items[a].x + items[a].width <= items[b].x ||
                                   items[b].x + items[b].width <= items[a].x ||
                                   items[a].y + items[a].height <= items[b].y ||
                                   items[b].y + items[b].height <= items[a].y;
                CHECK(apart);
            }
        }
    }
    {
        ct::Vector<DecalAtlas::Item> items;
        items.push_back(makeItem(16, 16));
        DecalAtlas::pack(items, 5, width, height);
        CHECK(width == 256 && height == 64);
        CHECK(items[0].width == 1 && items[0].height == 1);
        CHECK(nearRect(DecalAtlas::itemRect(items[0], 5, width, height), 0.0625f, 0.25f, 0.0625f,
                0.25f));
    }
}

struct Rgba
{
    unsigned char r, g, b, a;
};

bool readTexel(prisma::Driver* driver, prisma::TextureHandle texture, std::uint32_t mip, int x,
        int y, Rgba& out)
{
    prisma::RenderTarget source;
    source.texture = texture;
    source.mip = mip;
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

int countRed(prisma::Driver* driver, prisma::TextureHandle texture, std::uint32_t mip, int width,
        int height)
{
    ct::Vector<unsigned char> pixels;
    pixels.resize(static_cast<std::size_t>(width * height * 4), 0);
    prisma::RenderTarget source;
    source.texture = texture;
    source.mip = mip;
    prisma::Rect area;
    area.width = width;
    area.height = height;
    if (!driver->readPixels(source, area, pixels.data())) return -1;
    int count = 0;
    for (int i = 0; i < width * height; ++i)
    {
        const unsigned char* p = &pixels[static_cast<std::size_t>(i * 4)];
        if (p[0] > 250 && p[1] < 5 && p[2] < 5 && p[3] > 250) ++count;
    }
    return count;
}

bool sameColour(const Rgba& a, const Rgba& b)
{
    return abs(a.r - b.r) <= 2 && abs(a.g - b.g) <= 2 && abs(a.b - b.b) <= 2 && abs(a.a - b.a) <= 2;
}

prisma::TextureHandle makeTarget(prisma::Driver* driver, std::uint32_t size,
        prisma::TextureFormat format)
{
    prisma::TextureDesc desc;
    desc.width = size;
    desc.height = size;
    desc.format = format;
    desc.usage = prisma::kTextureSampled | prisma::kTextureRenderTarget;
    desc.debugName = "atlas test target";
    return driver->createTexture(desc);
}

void testAtlas(prisma::Driver* driver)
{
    TextureStore textures(driver);

    Pixmap quadPixels(64, 64, 4);
    for (std::uint32_t y = 0; y < 64; ++y)
    {
        for (std::uint32_t x = 0; x < 64; ++x)
        {
            const bool right = x >= 32;
            const bool bottom = y >= 32;
            if (!right && !bottom) quadPixels.set_pixel(x, y, 255, 0, 0, 255);
            else if (right && !bottom)
                quadPixels.set_pixel(x, y, 0, 255, 0, 255);
            else if (!right && bottom)
                quadPixels.set_pixel(x, y, 0, 0, 255, 255);
            else
                quadPixels.set_pixel(x, y, 255, 255, 0, 255);
        }
    }
    Pixmap redPixels(128, 32, 4);
    for (std::uint32_t y = 0; y < 32; ++y)
        for (std::uint32_t x = 0; x < 128; ++x) redPixels.set_pixel(x, y, 255, 0, 0, 255);
    Pixmap greyPixels(32, 32, 4);
    for (std::uint32_t y = 0; y < 32; ++y)
        for (std::uint32_t x = 0; x < 32; ++x) greyPixels.set_pixel(x, y, 128, 128, 128, 255);

    const TextureId quad = textures.createFromPixmap(quadPixels, false, false, "quad");
    const TextureId red = textures.createFromPixmap(redPixels, false, false, "red");
    const TextureId grey = textures.createFromPixmap(greyPixels, true, false, "grey");
    CHECK(quad.valid() && red.valid() && grey.valid());

    DecalAtlas atlas;
    CHECK(atlas.create(driver, &textures));
    CHECK(atlas.getMipmaps() == 5);
    CHECK(atlas.isDirty());
    CHECK(atlas.getTextureCount() == 0);
    CHECK(nearRect(atlas.getTextureRect(quad), 0.0f, 0.0f, 0.0f, 0.0f));

    driver->beginFrame();
    CHECK(atlas.update());
    CHECK(!atlas.isDirty());
    CHECK(atlas.getWidth() == 32 && atlas.getHeight() == 32);
    CHECK(atlas.getTexture().valid() && atlas.getTextureSrgb().valid());
    CHECK(!atlas.update());

    atlas.addTexture(quad);
    atlas.addTexture(red);
    atlas.addTexture(grey);
    atlas.addTexture(red);
    CHECK(atlas.getTextureCount() == 3);
    CHECK(atlas.hasTexture(red) && atlas.isDirty());
    CHECK(atlas.update());
    CHECK(atlas.getWidth() == 256 && atlas.getHeight() == 128);
    CHECK(nearRect(atlas.getTextureRect(quad), 0.0625f, 0.125f, 0.25f, 0.5f));
    CHECK(nearRect(atlas.getTextureRect(red), 0.4375f, 0.125f, 0.5f, 0.25f));
    CHECK(nearRect(atlas.getTextureRect(grey), 0.4375f, 0.625f, 0.125f, 0.25f));

    const prisma::TextureHandle built = atlas.getTexture();
    CHECK(!atlas.update());
    CHECK(atlas.getTexture() == built);

    CHECK(countRed(driver, atlas.getTexture(), 0, 256, 128) == 128 * 32 + 32 * 32);
    CHECK(countRed(driver, atlas.getTexture(), 1, 128, 64) == 64 * 16 + 16 * 16);
    CHECK(countRed(driver, atlas.getTexture(), 4, 16, 8) == 8 * 2 + 2 * 2);
    CHECK(countRed(driver, atlas.getTextureSrgb(), 0, 256, 128) == 128 * 32 + 32 * 32);
    CHECK(countRed(driver, atlas.getTextureSrgb(), 4, 16, 8) == 8 * 2 + 2 * 2);

    Rgba corner{ 9, 9, 9, 9 };
    const int cornersX[4] = { 0, 255, 0, 255 };
    const int cornersY[4] = { 0, 0, 127, 127 };
    for (int i = 0; i < 4; ++i)
    {
        CHECK(readTexel(driver, atlas.getTexture(), 0, cornersX[i], cornersY[i], corner));
        CHECK(corner.r == 0 && corner.g == 0 && corner.b == 0 && corner.a == 0);
    }

    const prisma::TextureHandle direct = makeTarget(driver, 2, prisma::TextureFormat::RGBA8);
    const prisma::TextureHandle viaAtlas = makeTarget(driver, 2, prisma::TextureFormat::RGBA8);
    CHECK(direct.valid() && viaAtlas.valid());
    CHECK(atlas.blit(textures.get(quad)->gpu, Math::Vec4(0.0f, 0.0f, 1.0f, 1.0f), direct, false, 0,
            2, 2));
    CHECK(atlas.blit(atlas.getTexture(), atlas.getTextureRect(quad), viaAtlas, false, 0, 2, 2));
    Rgba fromDirect[4];
    Rgba fromAtlas[4];
    for (int i = 0; i < 4; ++i)
    {
        CHECK(readTexel(driver, direct, 0, i % 2, i / 2, fromDirect[i]));
        CHECK(readTexel(driver, viaAtlas, 0, i % 2, i / 2, fromAtlas[i]));
        CHECK(sameColour(fromDirect[i], fromAtlas[i]));
        if (!sameColour(fromDirect[i], fromAtlas[i]))
            printf("texel %d: direct %d,%d,%d atlas %d,%d,%d\n", i, fromDirect[i].r,
                    fromDirect[i].g, fromDirect[i].b, fromAtlas[i].r, fromAtlas[i].g,
                    fromAtlas[i].b);
    }
    for (int a = 0; a < 4; ++a)
        for (int b = a + 1; b < 4; ++b) CHECK(!sameColour(fromDirect[a], fromDirect[b]));

    const prisma::TextureHandle linearOut = makeTarget(driver, 1, prisma::TextureFormat::RGBA8);
    const prisma::TextureHandle srgbOut = makeTarget(driver, 1, prisma::TextureFormat::RGBA8Srgb);
    Rgba texel{ 0, 0, 0, 0 };
    CHECK(atlas.blit(atlas.getTextureSrgb(), atlas.getTextureRect(grey), srgbOut, true, 0, 1, 1));
    CHECK(readTexel(driver, srgbOut, 0, 0, 0, texel));
    CHECK(abs(texel.r - 128) <= 2 && abs(texel.g - 128) <= 2 && abs(texel.b - 128) <= 2);
    CHECK(atlas.blit(atlas.getTextureSrgb(), atlas.getTextureRect(grey), linearOut, false, 0, 1,
            1));
    CHECK(readTexel(driver, linearOut, 0, 0, 0, texel));
    CHECK(abs(texel.r - 55) <= 2 && abs(texel.g - 55) <= 2 && abs(texel.b - 55) <= 2);
    if (abs(texel.r - 55) > 2) printf("srgb atlas read as linear is %d\n", texel.r);

    atlas.removeTexture(red);
    CHECK(atlas.hasTexture(red));
    CHECK(!atlas.isDirty());
    atlas.removeTexture(red);
    CHECK(!atlas.hasTexture(red));
    CHECK(!atlas.isDirty());
    CHECK(atlas.getTextureCount() == 2);
    CHECK(nearRect(atlas.getTextureRect(red), 0.0f, 0.0f, 0.0f, 0.0f));
    CHECK(nearRect(atlas.getTextureRect(quad), 0.0625f, 0.125f, 0.25f, 0.5f));

    atlas.markDirty(red);
    CHECK(!atlas.isDirty());
    atlas.markDirty(quad);
    CHECK(atlas.isDirty());
    CHECK(atlas.update());
    CHECK(atlas.getWidth() == 256 && atlas.getHeight() == 128);
    CHECK(nearRect(atlas.getTextureRect(quad), 0.0625f, 0.125f, 0.25f, 0.5f));
    CHECK(nearRect(atlas.getTextureRect(grey), 0.4375f, 0.125f, 0.125f, 0.25f));
    CHECK(countRed(driver, atlas.getTexture(), 0, 256, 128) == 32 * 32);

    const TextureId missing;
    atlas.addTexture(missing);
    CHECK(atlas.update());
    CHECK(nearRect(atlas.getTextureRect(missing), 0.0f, 0.0f, 0.0f, 0.0f));
    CHECK(nearRect(atlas.getTextureRect(quad), 0.0625f, 0.125f, 0.25f, 0.5f));
    CHECK(!atlas.update());

    driver->endFrame();

    driver->destroy(direct);
    driver->destroy(viaAtlas);
    driver->destroy(linearOut);
    driver->destroy(srgbOut);
    atlas.destroy();
    CHECK(!atlas.getTexture().valid());
}

} // namespace

int main(int argc, char** argv)
{
    const bool vulkan = argc > 1 && strcmp(argv[1], "vulkan") == 0;

    testPack();

    CHECK(Device::headlessSupported());
    if (!Device::headlessSupported()) return 1;

    DeviceDesc deviceDesc;
    deviceDesc.title = "decal atlas test";
    deviceDesc.driver = vulkan ? prisma::DriverType::Vulkan : prisma::DriverType::OpenGL;
    deviceDesc.width = 64;
    deviceDesc.height = 48;
    deviceDesc.headless = true;

    Device* device = Device::create(deviceDesc);
    CHECK(device != nullptr);
    if (!device) return 1;

    testAtlas(device->driver());

    Device::destroy(device);

    if (failures != 0)
    {
        printf("%d checks failed\n", failures);
        return 1;
    }
    printf("decal atlas: all checks passed\n");
    return 0;
}
