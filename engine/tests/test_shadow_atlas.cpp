#include "Check.h"

#include "prisma3d/render/ShadowAtlas.h"

#include <math.h>

using namespace prisma3d;

namespace
{

bool near(float a, float b)
{
    return fabsf(a - b) < 1e-6f;
}

bool slotIs(const ShadowAtlas& atlas, std::uint32_t light, std::uint32_t quadrant,
        std::uint32_t shadow, bool omni)
{
    ShadowAtlas::Slot slot;
    return atlas.getSlot(light, slot) && slot.quadrant == quadrant && slot.shadow == shadow &&
           slot.omni == omni;
}

void testDefaults()
{
    ShadowAtlas atlas;
    CHECK(atlas.getSize() == 4096);
    CHECK(!atlas.getSixteenBits());
    CHECK(atlas.getSubdivision(0) == 2);
    CHECK(atlas.getSubdivision(1) == 2);
    CHECK(atlas.getSubdivision(2) == 4);
    CHECK(atlas.getSubdivision(3) == 8);
    CHECK(atlas.getSmallestSubdivision() == 2);
    CHECK(atlas.getSubdivision(4) == 0);
    CHECK(atlas.getOwnerCount() == 0);
}

void testSubdivisionRounding()
{
    ShadowAtlas atlas;
    atlas.setQuadrantSubdivision(0, 1);
    CHECK(atlas.getSubdivision(0) == 1);
    atlas.setQuadrantSubdivision(0, 2);
    CHECK(atlas.getSubdivision(0) == 2);
    atlas.setQuadrantSubdivision(0, 3);
    CHECK(atlas.getSubdivision(0) == 2);
    atlas.setQuadrantSubdivision(0, 5);
    CHECK(atlas.getSubdivision(0) == 4);
    atlas.setQuadrantSubdivision(0, 256);
    CHECK(atlas.getSubdivision(0) == 16);
    atlas.setQuadrantSubdivision(0, 1024);
    CHECK(atlas.getSubdivision(0) == 32);
    atlas.setQuadrantSubdivision(0, 0);
    CHECK(atlas.getSubdivision(0) == 0);
    atlas.setQuadrantSubdivision(1, 16384);
    CHECK(atlas.getSubdivision(1) == 2);
    atlas.setQuadrantSubdivision(0, 4);
    atlas.setQuadrantSubdivision(4, 4);
    CHECK(atlas.getSubdivision(0) == 2);

    atlas.setQuadrantSubdivision(0, 0);
    atlas.setQuadrantSubdivision(1, 0);
    atlas.setQuadrantSubdivision(2, 0);
    atlas.setQuadrantSubdivision(3, 0);
    CHECK(atlas.getSmallestSubdivision() == 0);
    CHECK(!atlas.updateLight(1, false, 1.0f, 1000, 1));
    atlas.setQuadrantSubdivision(3, 64);
    CHECK(atlas.getSmallestSubdivision() == 8);
    CHECK(atlas.updateLight(1, false, 1.0f, 1000, 1));
    CHECK(slotIs(atlas, 1, 3, 0, false));
}

void testSizes()
{
    ShadowAtlas atlas;
    atlas.setSize(1000, true);
    CHECK(atlas.getSize() == 1024);
    CHECK(atlas.getSixteenBits());
    CHECK(atlas.updateLight(1, false, 1.0f, 1000, 1));
    CHECK(atlas.ownsLight(1));
    atlas.setSize(1024, true);
    CHECK(atlas.ownsLight(1));
    atlas.setSize(2048, true);
    CHECK(!atlas.ownsLight(1));
    CHECK(atlas.getOwnerCount() == 0);
    atlas.setSize(0, false);
    CHECK(atlas.getSize() == 0);
    CHECK(!atlas.updateLight(1, false, 1.0f, 1000, 1));
}

void testPlacement()
{
    ShadowAtlas atlas;

    CHECK(atlas.updateLight(1, false, 1.0f, 1000, 1));
    CHECK(slotIs(atlas, 1, 1, 0, false));
    std::uint32_t pixels[4];
    CHECK(atlas.getPixelRect(1, pixels));
    CHECK(pixels[0] == 2048 && pixels[1] == 0 && pixels[2] == 1024 && pixels[3] == 1024);
    CHECK(atlas.getShadowSize(1) == 1024);
    float rect[4];
    int offset[2];
    CHECK(atlas.getRect(1, rect, offset));
    CHECK(near(rect[0], 0.5f) && near(rect[1], 0.0f) && near(rect[2], 0.25f) &&
            near(rect[3], 0.25f));
    CHECK(offset[0] == 0 && offset[1] == 0);

    CHECK(atlas.updateLight(2, false, 0.01f, 1000, 1));
    CHECK(slotIs(atlas, 2, 3, 0, false));
    CHECK(atlas.getPixelRect(2, pixels));
    CHECK(pixels[0] == 2048 && pixels[1] == 2048 && pixels[2] == 256 && pixels[3] == 256);

    CHECK(atlas.updateLight(3, false, 0.2f, 1000, 1));
    CHECK(slotIs(atlas, 3, 2, 0, false));
    CHECK(atlas.getPixelRect(3, pixels));
    CHECK(pixels[0] == 0 && pixels[1] == 2048 && pixels[2] == 512 && pixels[3] == 512);

    CHECK(atlas.updateLight(4, false, 1.0f, 1000, 1));
    CHECK(slotIs(atlas, 4, 1, 1, false));
    CHECK(atlas.getPixelRect(4, pixels));
    CHECK(pixels[0] == 3072 && pixels[1] == 0);

    CHECK(atlas.updateLight(5, false, 1.0f, 1000, 1));
    CHECK(slotIs(atlas, 5, 1, 2, false));
    CHECK(atlas.getPixelRect(5, pixels));
    CHECK(pixels[0] == 2048 && pixels[1] == 1024);

    CHECK(atlas.updateLight(6, false, 1.0f, 1000, 1));
    CHECK(slotIs(atlas, 6, 1, 3, false));
    CHECK(atlas.getPixelRect(6, pixels));
    CHECK(pixels[0] == 3072 && pixels[1] == 1024);

    CHECK(atlas.updateLight(7, false, 1.0f, 1000, 1));
    CHECK(slotIs(atlas, 7, 0, 0, false));

    CHECK(atlas.updateLight(1, false, 1.0f, 1000, 1));
    CHECK(slotIs(atlas, 1, 1, 0, false));
    CHECK(atlas.getOwnerCount() == 7);
}

void testOmniPairs()
{
    ShadowAtlas atlas;
    atlas.setQuadrantSubdivision(0, 0);
    atlas.setQuadrantSubdivision(2, 0);
    atlas.setQuadrantSubdivision(3, 0);
    CHECK(atlas.updateLight(1, false, 1.0f, 1000, 1));
    CHECK(slotIs(atlas, 1, 1, 0, false));

    CHECK(atlas.updateLight(2, true, 1.0f, 1000, 1));
    CHECK(slotIs(atlas, 2, 1, 1, true));
    float rect[4];
    int offset[2];
    CHECK(atlas.getRect(2, rect, offset));
    CHECK(offset[0] == -1 && offset[1] == 1);
    CHECK(near(rect[0], 0.75f) && near(rect[1], 0.0f));

    CHECK(!atlas.updateLight(3, true, 1.0f, 1000, 1));
    CHECK(!atlas.ownsLight(3));

    atlas.releaseLight(2);
    CHECK(!atlas.ownsLight(2));
    CHECK(atlas.updateLight(4, false, 1.0f, 1000, 1));
    CHECK(slotIs(atlas, 4, 1, 1, false));

    ShadowAtlas small;
    CHECK(small.updateLight(1, true, 0.01f, 1000, 1));
    CHECK(slotIs(small, 1, 3, 0, true));
    CHECK(small.getRect(1, rect, offset));
    CHECK(offset[0] == 1 && offset[1] == 0);
    CHECK(small.updateLight(2, false, 0.01f, 1000, 1));
    CHECK(slotIs(small, 2, 3, 2, false));
    for (std::uint32_t light = 3; light <= 6; ++light)
        CHECK(small.updateLight(light, false, 0.01f, 1000, 1));
    CHECK(slotIs(small, 6, 3, 6, false));
    CHECK(small.updateLight(8, true, 0.01f, 1000, 1));
    CHECK(slotIs(small, 8, 3, 7, true));
    CHECK(small.getRect(8, rect, offset));
    CHECK(offset[0] == -7 && offset[1] == 1);

    ShadowAtlas single;
    single.setQuadrantSubdivision(0, 1);
    single.setQuadrantSubdivision(1, 1);
    single.setQuadrantSubdivision(2, 1);
    single.setQuadrantSubdivision(3, 1);
    CHECK(single.getOwnerCount() == 0);
    CHECK(!single.updateLight(7, true, 1.0f, 1000, 2));
    CHECK(single.updateLight(8, false, 1.0f, 1000, 2));
}

void testStealing()
{
    ShadowAtlas atlas;
    atlas.setQuadrantSubdivision(0, 0);
    atlas.setQuadrantSubdivision(2, 0);
    atlas.setQuadrantSubdivision(3, 0);
    for (std::uint32_t light = 1; light <= 4; ++light)
        CHECK(atlas.updateLight(light, false, 1.0f, 1000, light));
    CHECK(atlas.getOwnerCount() == 4);

    CHECK(!atlas.updateLight(5, false, 1.0f, 1100, 10));
    CHECK(!atlas.ownsLight(5));

    atlas.markVisible(1, 11);
    atlas.markVisible(2, 11);
    atlas.markVisible(3, 11);
    CHECK(!atlas.updateLight(5, false, 1.0f, 1200, 11));
    CHECK(atlas.updateLight(5, false, 1.0f, 2000, 11));
    CHECK(atlas.ownsLight(5));
    CHECK(!atlas.ownsLight(4));
    CHECK(atlas.ownsLight(1) && atlas.ownsLight(2) && atlas.ownsLight(3));
    CHECK(slotIs(atlas, 5, 1, 3, false));

    atlas.markVisible(5, 12);
    atlas.markVisible(1, 12);
    atlas.markVisible(3, 12);
    CHECK(atlas.updateLight(6, false, 1.0f, 3000, 12));
    CHECK(!atlas.ownsLight(2));
    CHECK(slotIs(atlas, 6, 1, 1, false));
}

void testReallocation()
{
    ShadowAtlas atlas;
    CHECK(atlas.updateLight(1, false, 1.0f, 1000, 1));
    CHECK(slotIs(atlas, 1, 1, 0, false));

    CHECK(atlas.updateLight(1, false, 0.01f, 1200, 2));
    CHECK(slotIs(atlas, 1, 1, 0, false));

    CHECK(atlas.updateLight(1, false, 0.01f, 1600, 3));
    CHECK(slotIs(atlas, 1, 3, 0, false));

    CHECK(atlas.updateLight(2, false, 1.0f, 1600, 3));
    CHECK(slotIs(atlas, 2, 1, 0, false));

    CHECK(atlas.updateLight(1, false, 1.0f, 1700, 4));
    CHECK(slotIs(atlas, 1, 3, 0, false));
    CHECK(atlas.updateLight(1, false, 1.0f, 2300, 5));
    CHECK(slotIs(atlas, 1, 1, 1, false));
    CHECK(atlas.getOwnerCount() == 2);

    CHECK(atlas.updateLight(3, true, 0.01f, 2300, 5));
    CHECK(slotIs(atlas, 3, 3, 0, true));
    CHECK(atlas.updateLight(3, true, 1.0f, 3000, 6));
    CHECK(slotIs(atlas, 3, 1, 2, true));
    CHECK(atlas.getOwnerCount() == 3);
    CHECK(atlas.updateLight(4, false, 0.01f, 3000, 6));
    CHECK(slotIs(atlas, 4, 3, 0, false));
}

} // namespace

int main()
{
    testDefaults();
    testSubdivisionRounding();
    testSizes();
    testPlacement();
    testOmniPairs();
    testStealing();
    testReallocation();

    if (failures) printf("%d check(s) failed\n", failures);
    else
        printf("all checks passed\n");
    return failures ? 1 : 0;
}
