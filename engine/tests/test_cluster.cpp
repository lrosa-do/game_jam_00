#include "Check.h"

#include "prisma3d/Device.h"
#include "prisma3d/render/ClusterBuilder.h"

#include <ct/vector.hpp>

#include <math.h>
#include <stdlib.h>
#include <string.h>

using namespace prisma3d;

namespace
{

const std::uint32_t kWidth = 640;
const std::uint32_t kHeight = 480;
const std::uint32_t kMaxElements = 32;
const float kFovDegrees = 60.0f;
const float kNear = 0.1f;
const float kFar = 64.0f;

struct Setup
{
    prisma::Driver* driver;
    Device* device;
    ClusterBuilder* builder;
};

float focalLength()
{
    return (static_cast<float>(kHeight) * 0.5f) / tanf(kFovDegrees * 0.5f * Math::DEG2RAD);
}

float aspect()
{
    return static_cast<float>(kWidth) / static_cast<float>(kHeight);
}

Math::Mat4 projection()
{
    return Math::Mat4::Perspective(kFovDegrees * Math::DEG2RAD, aspect(), kNear, kFar);
}

float screenX(float x, float depth)
{
    const float ndc = x / (depth * aspect() * tanf(kFovDegrees * 0.5f * Math::DEG2RAD));
    return (ndc * 0.5f + 0.5f) * static_cast<float>(kWidth);
}

float screenY(float y, float depth)
{
    const float ndc = y / (depth * tanf(kFovDegrees * 0.5f * Math::DEG2RAD));
    return (ndc * 0.5f + 0.5f) * static_cast<float>(kHeight);
}

int sliceOf(float depth)
{
    int slice = static_cast<int>(floorf(depth / kFar * 32.0f));
    if (slice < 0) slice = 0;
    if (slice > 31) slice = 31;
    return slice;
}

struct Snapshot
{
    ct::Vector<std::uint32_t> words;
    std::uint32_t clusterWidth;
    std::uint32_t clusterHeight;
    std::uint32_t perCell;

    std::uint32_t base(std::uint32_t type, std::uint32_t cx, std::uint32_t cy) const
    {
        return ((cy * clusterWidth + cx) + type * clusterWidth * clusterHeight) * perCell;
    }

    bool bit(std::uint32_t type, std::uint32_t cx, std::uint32_t cy, std::uint32_t index) const
    {
        return ((words[base(type, cx, cy) + (index >> 5)] >> (index & 31)) & 1u) != 0;
    }

    std::uint32_t slice(std::uint32_t type, std::uint32_t cx, std::uint32_t cy,
            std::uint32_t z) const
    {
        return words[base(type, cx, cy) + (perCell - 32) + z];
    }

    bool range(std::uint32_t type, std::uint32_t cx, std::uint32_t cy, std::uint32_t index,
            int& from, int& to) const
    {
        from = -1;
        to = -1;
        for (std::uint32_t z = 0; z < 32; ++z)
        {
            const std::uint32_t entry = slice(type, cx, cy, z);
            const std::uint32_t low = entry & 0xFFFFu;
            const std::uint32_t high = entry >> 16;
            if (low <= index && index < high)
            {
                if (from < 0) from = static_cast<int>(z);
                to = static_cast<int>(z) + 1;
            }
        }
        return from >= 0;
    }

    bool typeEmpty(std::uint32_t type) const
    {
        const std::uint32_t size = clusterWidth * clusterHeight * perCell;
        for (std::uint32_t i = 0; i < size; ++i)
            if (words[type * size + i] != 0) return false;
        return true;
    }

    bool allEmpty() const
    {
        for (std::size_t i = 0; i < words.size(); ++i)
            if (words[i] != 0) return false;
        return true;
    }
};

bool bakeAndRead(const Setup& setup, Snapshot& snapshot)
{
    setup.device->processEvents();
    setup.driver->beginFrame();
    const bool baked = setup.builder->bake();
    setup.driver->endFrame();
    setup.driver->present();
    if (!baked) return false;
    snapshot.clusterWidth = setup.builder->getClusterWidth();
    snapshot.clusterHeight = setup.builder->getClusterHeight();
    snapshot.perCell = setup.builder->getMaxClusterElementCountDiv32() + 32;
    snapshot.words.resize(setup.builder->getClusterBufferSize() / 4);
    return setup.driver->readBuffer(setup.builder->getClusterBuffer(), 0,
            setup.builder->getClusterBufferSize(), snapshot.words.data());
}

Math::Mat4 at(float x, float y, float z)
{
    return Math::Mat4::Translation(Math::Vec3(x, y, z));
}

struct Rect
{
    float left;
    float top;
    float right;
    float bottom;
};

Rect cellRect(std::uint32_t cx, std::uint32_t cy)
{
    Rect rect;
    rect.left = static_cast<float>(cx * 32);
    rect.right = static_cast<float>(cx * 32 + 32);
    rect.top = static_cast<float>(cy * 32);
    rect.bottom = static_cast<float>(cy * 32 + 32);
    return rect;
}

float nearestDistance(const Rect& rect, float px, float py)
{
    const float dx = px < rect.left ? rect.left - px : (px > rect.right ? px - rect.right : 0.0f);
    const float dy = py < rect.top ? rect.top - py : (py > rect.bottom ? py - rect.bottom : 0.0f);
    return sqrtf(dx * dx + dy * dy);
}

float farthestDistance(const Rect& rect, float px, float py)
{
    const float dx = fmaxf(fabsf(px - rect.left), fabsf(px - rect.right));
    const float dy = fmaxf(fabsf(py - rect.top), fabsf(py - rect.bottom));
    return sqrtf(dx * dx + dy * dy);
}

int checkDisc(const Snapshot& snapshot, std::uint32_t type, std::uint32_t index, float px, float py,
        float innerRadius, float outerRadius, const char* label)
{
    int inside = 0;
    int outside = 0;
    for (std::uint32_t cy = 0; cy < snapshot.clusterHeight; ++cy)
        for (std::uint32_t cx = 0; cx < snapshot.clusterWidth; ++cx)
        {
            const Rect rect = cellRect(cx, cy);
            const bool set = snapshot.bit(type, cx, cy, index);
            if (farthestDistance(rect, px, py) <= innerRadius * 0.9f)
            {
                ++inside;
                if (!set)
                {
                    printf("%s: cell (%u,%u) should have element %u\n", label, cx, cy, index);
                    ++failures;
                }
            }
            if (nearestDistance(rect, px, py) >= outerRadius * 1.1f)
            {
                ++outside;
                if (set)
                {
                    printf("%s: cell (%u,%u) should not have element %u\n", label, cx, cy, index);
                    ++failures;
                }
            }
        }
    CHECK(inside >= 4);
    CHECK(outside >= 100);
    return inside;
}

int checkBox(const Snapshot& snapshot, std::uint32_t type, std::uint32_t index, float px, float py,
        float halfWidth, float halfHeight, const char* label)
{
    int inside = 0;
    int outside = 0;
    for (std::uint32_t cy = 0; cy < snapshot.clusterHeight; ++cy)
        for (std::uint32_t cx = 0; cx < snapshot.clusterWidth; ++cx)
        {
            const Rect rect = cellRect(cx, cy);
            const bool set = snapshot.bit(type, cx, cy, index);
            const bool fullyInside =
                    rect.left >= px - halfWidth * 0.9f && rect.right <= px + halfWidth * 0.9f &&
                    rect.top >= py - halfHeight * 0.9f && rect.bottom <= py + halfHeight * 0.9f;
            const bool fullyOutside =
                    rect.right <= px - halfWidth * 1.1f || rect.left >= px + halfWidth * 1.1f ||
                    rect.bottom <= py - halfHeight * 1.1f || rect.top >= py + halfHeight * 1.1f;
            if (fullyInside)
            {
                ++inside;
                if (!set)
                {
                    printf("%s: cell (%u,%u) should have element %u\n", label, cx, cy, index);
                    ++failures;
                }
            }
            if (fullyOutside)
            {
                ++outside;
                if (set)
                {
                    printf("%s: cell (%u,%u) should not have element %u\n", label, cx, cy, index);
                    ++failures;
                }
            }
        }
    CHECK(inside >= 1);
    CHECK(outside >= 100);
    return inside;
}

bool rangeIs(const Snapshot& snapshot, std::uint32_t type, std::uint32_t cx, std::uint32_t cy,
        std::uint32_t index, int expectedFrom, int expectedTo, const char* label)
{
    int from = -1;
    int to = -1;
    const bool found = snapshot.range(type, cx, cy, index, from, to);
    const bool same = found && from == expectedFrom && to == expectedTo;
    if (!same)
        printf("%s: cell (%u,%u) range is %d..%d, expected %d..%d\n", label, cx, cy, from, to,
                expectedFrom, expectedTo);
    return same;
}

void testGeometry(const Setup& setup, const ClusterBuilderShared& shared)
{
    ClusterBuilder& builder = *setup.builder;
    CHECK(builder.getClusterSize() == 32);
    CHECK(builder.getClusterShift() == 5);
    CHECK(builder.getClusterWidth() == 20);
    CHECK(builder.getClusterHeight() == 15);
    CHECK(builder.getMaxClusterElements() == 32);
    CHECK(builder.getMaxClusterElementCountDiv32() == 1);
    CHECK(builder.getClusterTypeSize() == 20u * 15u * 33u);
    CHECK(builder.getClusterBufferSize() == 20u * 15u * 33u * 5u * 4u);
    CHECK(builder.getScreenWidth() == kWidth && builder.getScreenHeight() == kHeight);
    CHECK(shared.sphereOverfit() > 1.0f && shared.sphereOverfit() < 1.2f);
    CHECK(shared.coneOverfit() > 1.0f && shared.coneOverfit() < 1.2f);
}

void testOmni(const Setup& setup, const ClusterBuilderShared& shared)
{
    ClusterBuilder& builder = *setup.builder;
    builder.begin(Math::Mat4::Identity(), projection(), false);
    CHECK(fabsf(builder.getZNear() - kNear) < 0.0001f);
    CHECK(fabsf(builder.getZFar() - kFar) < 0.01f);
    builder.addLight(ClusterBuilder::kLightOmni, at(0.0f, 0.0f, -20.0f), 5.0f, 0.0f);
    CHECK(builder.getElementCount() == 1);
    CHECK(builder.getClusterCountByType(ClusterBuilder::kElementOmniLight) == 1);

    Snapshot snapshot;
    CHECK(bakeAndRead(setup, snapshot));

    const float inner = 5.0f;
    const float outer = 5.0f * shared.sphereOverfit();
    const float focal = focalLength();
    const float innerRadius = focal * inner / sqrtf(20.0f * 20.0f - inner * inner);
    const float outerRadius = focal * outer / sqrtf(20.0f * 20.0f - outer * outer);
    const float px = screenX(0.0f, 20.0f);
    const float py = screenY(0.0f, 20.0f);
    CHECK(fabsf(px - 320.0f) < 0.01f && fabsf(py - 240.0f) < 0.01f);
    checkDisc(snapshot, ClusterBuilder::kElementOmniLight, 0, px, py, innerRadius, outerRadius,
            "omni");

    CHECK(snapshot.bit(0, 10, 7, 0));
    CHECK(snapshot.bit(0, 9, 7, 0));
    CHECK(!snapshot.bit(0, 0, 0, 0));
    CHECK(!snapshot.bit(0, 19, 14, 0));
    CHECK(!snapshot.bit(0, 0, 14, 0));
    CHECK(!snapshot.bit(0, 19, 0, 0));
    CHECK(!snapshot.bit(0, 10, 7, 1));

    const int from = sliceOf(20.0f - outer);
    const int to = sliceOf(20.0f + outer) + 1;
    CHECK(rangeIs(snapshot, 0, 10, 7, 0, from, to, "omni z range"));
    CHECK(rangeIs(snapshot, 0, 9, 7, 0, from, to, "omni z range"));
    CHECK(snapshot.slice(0, 10, 7, from - 1) == 0);
    CHECK(snapshot.slice(0, 10, 7, to) == 0);
    CHECK(snapshot.slice(0, 10, 7, from) == 0x10000u);

    int centerFrom = 0;
    int centerTo = 0;
    CHECK(snapshot.range(0, 10, 7, 0, centerFrom, centerTo));
    const std::uint32_t sideX = 13;
    int sideFrom = 0;
    int sideTo = 0;
    CHECK(snapshot.range(0, sideX, 7, 0, sideFrom, sideTo));
    CHECK(sideFrom >= centerFrom && sideTo <= centerTo && sideFrom < sideTo);

    for (std::uint32_t type = 1; type < ClusterBuilder::kElementTypeMax; ++type)
        CHECK(snapshot.typeEmpty(type));
}

void testSpot(const Setup& setup, const ClusterBuilderShared& shared)
{
    ClusterBuilder& builder = *setup.builder;
    builder.begin(Math::Mat4::Identity(), projection(), false);
    const float aperture = 20.0f;
    const float range = 20.0f;
    builder.addLight(ClusterBuilder::kLightSpot, at(0.0f, 0.0f, -15.0f), range, aperture);
    CHECK(builder.getClusterCountByType(ClusterBuilder::kElementSpotLight) == 1);

    Snapshot snapshot;
    CHECK(bakeAndRead(setup, snapshot));

    const float overfit = shared.coneOverfit();
    const float baseDepth = 15.0f + range * overfit;
    const float baseInner = tanf(aperture * Math::DEG2RAD) * range;
    const float baseOuter = baseInner * overfit * overfit;
    const float focal = focalLength();
    const float innerRadius = focal * baseInner / (15.0f + range * overfit);
    const float outerRadius = focal * baseOuter / (15.0f + range * overfit);
    const float px = screenX(0.0f, 15.0f);
    const float py = screenY(0.0f, 15.0f);
    checkDisc(snapshot, ClusterBuilder::kElementSpotLight, 0, px, py, innerRadius, outerRadius,
            "spot");

    CHECK(snapshot.bit(1, 10, 7, 0));
    CHECK(snapshot.bit(1, 9, 8, 0));
    CHECK(!snapshot.bit(1, 0, 0, 0));
    CHECK(!snapshot.bit(1, 19, 14, 0));
    CHECK(rangeIs(snapshot, 1, 10, 7, 0, sliceOf(15.0f), sliceOf(baseDepth) + 1, "spot z range"));
    CHECK(snapshot.typeEmpty(0));
    for (std::uint32_t type = 2; type < ClusterBuilder::kElementTypeMax; ++type)
        CHECK(snapshot.typeEmpty(type));
}

void testWideSpot(const Setup& setup, const ClusterBuilderShared& shared)
{
    ClusterBuilder& builder = *setup.builder;
    builder.begin(Math::Mat4::Identity(), projection(), false);
    builder.addLight(ClusterBuilder::kLightSpot, at(0.0f, 0.0f, -20.0f), 5.0f, 80.0f);
    CHECK(builder.getClusterCountByType(ClusterBuilder::kElementSpotLight) == 1);

    Snapshot snapshot;
    CHECK(bakeAndRead(setup, snapshot));

    const float inner = 5.0f;
    const float outer = 5.0f * shared.sphereOverfit();
    const float focal = focalLength();
    checkDisc(snapshot, ClusterBuilder::kElementSpotLight, 0, 320.0f, screenY(0.0f, 20.0f),
            focal * inner / sqrtf(400.0f - inner * inner),
            focal * outer / sqrtf(400.0f - outer * outer), "wide spot");
    CHECK(rangeIs(snapshot, 1, 10, 7, 0, sliceOf(20.0f - outer), sliceOf(20.0f + outer) + 1,
            "wide spot z range"));
    CHECK(snapshot.typeEmpty(0));
}

void testDecal(const Setup& setup)
{
    ClusterBuilder& builder = *setup.builder;
    builder.begin(Math::Mat4::Identity(), projection(), false);
    builder.addBox(ClusterBuilder::kBoxDecal, at(0.0f, 0.0f, -21.0f), Math::Vec3(2.0f, 2.0f, 2.0f));
    CHECK(builder.getClusterCountByType(ClusterBuilder::kElementDecal) == 1);

    Snapshot snapshot;
    CHECK(bakeAndRead(setup, snapshot));

    const float half = focalLength() * 2.0f / 19.0f;
    checkBox(snapshot, ClusterBuilder::kElementDecal, 0, 320.0f, 240.0f, half, half, "decal");
    CHECK(snapshot.bit(3, 10, 7, 0));
    CHECK(snapshot.bit(3, 9, 7, 0));
    CHECK(!snapshot.bit(3, 0, 0, 0));
    CHECK(rangeIs(snapshot, 3, 10, 7, 0, sliceOf(19.0f), sliceOf(23.0f) + 1, "decal z range"));
    for (std::uint32_t type = 0; type < ClusterBuilder::kElementTypeMax; ++type)
        if (type != ClusterBuilder::kElementDecal) CHECK(snapshot.typeEmpty(type));

    builder.begin(Math::Mat4::Identity(), projection(), false);
    builder.addBox(ClusterBuilder::kBoxReflectionProbe, at(0.0f, 0.0f, -21.0f),
            Math::Vec3(2.0f, 2.0f, 2.0f));
    CHECK(builder.getClusterCountByType(ClusterBuilder::kElementReflectionProbe) == 1);
    CHECK(bakeAndRead(setup, snapshot));
    CHECK(snapshot.bit(4, 10, 7, 0));
    CHECK(rangeIs(snapshot, 4, 10, 7, 0, sliceOf(19.0f), sliceOf(23.0f) + 1, "probe z range"));
    CHECK(snapshot.typeEmpty(3));
}

void testAreaLight(const Setup& setup)
{
    ClusterBuilder& builder = *setup.builder;
    builder.begin(Math::Mat4::Identity(), projection(), false);
    builder.addLight(ClusterBuilder::kLightArea, at(0.0f, 0.0f, -19.0f), 2.0f, 0.0f,
            Math::Vec2(4.0f, 4.0f));
    CHECK(builder.getClusterCountByType(ClusterBuilder::kElementAreaLight) == 1);

    Snapshot snapshot;
    CHECK(bakeAndRead(setup, snapshot));

    const float half = focalLength() * 4.0f / 19.0f;
    checkBox(snapshot, ClusterBuilder::kElementAreaLight, 0, 320.0f, 240.0f, half, half, "area");
    CHECK(rangeIs(snapshot, 2, 10, 7, 0, sliceOf(19.0f), sliceOf(21.0f) + 1, "area z range"));
    CHECK(snapshot.typeEmpty(0) && snapshot.typeEmpty(1) && snapshot.typeEmpty(3));
}

void testScaledBox(const Setup& setup)
{
    ClusterBuilder& builder = *setup.builder;
    builder.begin(Math::Mat4::Identity(), projection(), false);
    builder.addBox(ClusterBuilder::kBoxDecal,
            at(0.0f, 0.0f, -22.0f) * Math::Mat4::Scale(Math::Vec3(2.0f, 1.0f, 1.0f)),
            Math::Vec3(1.0f, 1.0f, 1.0f));

    Snapshot snapshot;
    CHECK(bakeAndRead(setup, snapshot));

    const float focal = focalLength();
    checkBox(snapshot, ClusterBuilder::kElementDecal, 0, 320.0f, 240.0f, focal * 2.0f / 21.0f,
            focal * 1.0f / 21.0f, "scaled decal");
    CHECK(rangeIs(snapshot, 3, 10, 7, 0, sliceOf(21.0f), sliceOf(23.0f) + 1, "scaled decal z"));
}

void testOrthogonal(const Setup& setup, const ClusterBuilderShared& shared)
{
    ClusterBuilder& builder = *setup.builder;
    const float halfHeight = 10.0f;
    const float halfWidth = halfHeight * aspect();
    builder.begin(Math::Mat4::Identity(),
            Math::Mat4::Ortho(-halfWidth, halfWidth, -halfHeight, halfHeight, kNear, kFar), false);
    CHECK(fabsf(builder.getZNear() - kNear) < 0.01f);
    CHECK(fabsf(builder.getZFar() - kFar) < 0.01f);
    builder.addLight(ClusterBuilder::kLightOmni, at(0.0f, 0.0f, -20.0f), 4.0f, 0.0f);

    Snapshot snapshot;
    CHECK(bakeAndRead(setup, snapshot));

    const float inner = 4.0f / halfHeight * 240.0f;
    const float outer = inner * shared.sphereOverfit();
    checkDisc(snapshot, ClusterBuilder::kElementOmniLight, 0, 320.0f, 240.0f, inner, outer,
            "ortho omni");
    const float radius = 4.0f * shared.sphereOverfit();
    CHECK(rangeIs(snapshot, 0, 10, 7, 0, sliceOf(20.0f - radius), sliceOf(20.0f + radius) + 1,
            "ortho z range"));

    builder.begin(Math::Mat4::Identity(),
            Math::Mat4::Ortho(-halfWidth, halfWidth, -halfHeight, halfHeight, kNear, kFar), false);
    builder.addLight(ClusterBuilder::kLightOmni, at(0.0f, 0.0f, -2.0f), 4.0f, 0.0f);
    CHECK(bakeAndRead(setup, snapshot));
    int from = 0;
    int to = 0;
    CHECK(snapshot.range(0, 10, 7, 0, from, to));
    CHECK(from == 0);
}

void testVertical(const Setup& setup)
{
    ClusterBuilder& builder = *setup.builder;
    builder.begin(Math::Mat4::Identity(), projection(), false);
    builder.addBox(ClusterBuilder::kBoxDecal, at(0.0f, 6.0f, -21.0f), Math::Vec3(2.0f, 2.0f, 2.0f));

    Snapshot snapshot;
    CHECK(bakeAndRead(setup, snapshot));

    const float half = focalLength() * 2.0f / 19.0f;
    const float py = screenY(6.0f, 21.0f);
    checkBox(snapshot, ClusterBuilder::kElementDecal, 0, 320.0f, py, half, half, "decal above");
    const std::uint32_t row = static_cast<std::uint32_t>(py) / 32;
    CHECK(snapshot.bit(3, 10, row, 0));
    CHECK(!snapshot.bit(3, 10, 14 - row, 0));
    CHECK(row > 7);
}

void testTwoElements(const Setup& setup, const ClusterBuilderShared& shared)
{
    ClusterBuilder& builder = *setup.builder;
    builder.begin(Math::Mat4::Identity(), projection(), false);
    builder.addLight(ClusterBuilder::kLightOmni, at(-8.0f, 0.0f, -20.0f), 3.0f, 0.0f);
    builder.addLight(ClusterBuilder::kLightOmni, at(8.0f, 0.0f, -20.0f), 3.0f, 0.0f);
    CHECK(builder.getClusterCountByType(ClusterBuilder::kElementOmniLight) == 2);

    Snapshot snapshot;
    CHECK(bakeAndRead(setup, snapshot));

    const float inner = 3.0f;
    const float outer = 3.0f * shared.sphereOverfit();
    const float focal = focalLength();
    const float innerRadius = focal * inner / sqrtf(400.0f - inner * inner);
    const float outerRadius = focal * outer / sqrtf(400.0f - outer * outer);
    const float py = screenY(0.0f, 20.0f);
    const float leftX = screenX(-8.0f, 20.0f);
    const float rightX = screenX(8.0f, 20.0f);
    checkDisc(snapshot, 0, 0, leftX, py, innerRadius, outerRadius, "left light");
    checkDisc(snapshot, 0, 1, rightX, py, innerRadius, outerRadius, "right light");

    const std::uint32_t leftCell = static_cast<std::uint32_t>(leftX) / 32;
    const std::uint32_t rightCell = static_cast<std::uint32_t>(rightX) / 32;
    CHECK(leftCell < rightCell);
    CHECK(snapshot.bit(0, leftCell, 7, 0));
    CHECK(!snapshot.bit(0, leftCell, 7, 1));
    CHECK(snapshot.bit(0, rightCell, 7, 1));
    CHECK(!snapshot.bit(0, rightCell, 7, 0));
    CHECK(!snapshot.bit(0, 10, 7, 0) && !snapshot.bit(0, 10, 7, 1));

    const int from = sliceOf(20.0f - outer);
    const int to = sliceOf(20.0f + outer) + 1;
    CHECK(rangeIs(snapshot, 0, leftCell, 7, 0, from, to, "left light z range"));
    CHECK(rangeIs(snapshot, 0, rightCell, 7, 1, from, to, "right light z range"));
    CHECK(snapshot.slice(0, leftCell, 7, from) == 0x10000u);
    CHECK(snapshot.slice(0, rightCell, 7, from) == 0x20001u);
}

void testOverlap(const Setup& setup)
{
    ClusterBuilder& builder = *setup.builder;
    builder.begin(Math::Mat4::Identity(), projection(), false);
    builder.addLight(ClusterBuilder::kLightOmni, at(0.0f, 0.0f, -20.0f), 4.0f, 0.0f);
    builder.addLight(ClusterBuilder::kLightOmni, at(0.0f, 0.0f, -30.0f), 4.0f, 0.0f);

    Snapshot snapshot;
    CHECK(bakeAndRead(setup, snapshot));
    CHECK(snapshot.bit(0, 10, 7, 0));
    CHECK(snapshot.bit(0, 10, 7, 1));
    int from0 = 0;
    int to0 = 0;
    int from1 = 0;
    int to1 = 0;
    CHECK(snapshot.range(0, 10, 7, 0, from0, to0));
    CHECK(snapshot.range(0, 10, 7, 1, from1, to1));
    CHECK(from0 < from1);
    CHECK(to0 < to1);
    const std::uint32_t entry = snapshot.slice(0, 10, 7, static_cast<std::uint32_t>(from1));
    CHECK((entry & 0xFFFFu) <= 1u && (entry >> 16) >= 2u);
}

void testBehindCamera(const Setup& setup)
{
    ClusterBuilder& builder = *setup.builder;
    builder.begin(Math::Mat4::Identity(), projection(), false);
    builder.addLight(ClusterBuilder::kLightOmni, at(0.0f, 0.0f, 10.0f), 3.0f, 0.0f);
    builder.addBox(ClusterBuilder::kBoxDecal, at(0.0f, 0.0f, 10.0f), Math::Vec3(2.0f, 2.0f, 2.0f));
    CHECK(builder.getElementCount() == 2);

    Snapshot snapshot;
    CHECK(bakeAndRead(setup, snapshot));
    CHECK(snapshot.allEmpty());
}

void testCameraInside(const Setup& setup, const ClusterBuilderShared& shared)
{
    ClusterBuilder& builder = *setup.builder;
    builder.begin(Math::Mat4::Identity(), projection(), false);
    builder.addLight(ClusterBuilder::kLightOmni, at(0.0f, 0.0f, -1.0f), 20.0f, 0.0f);

    Snapshot snapshot;
    CHECK(bakeAndRead(setup, snapshot));

    int missing = 0;
    int notFromZero = 0;
    for (std::uint32_t cy = 0; cy < 15; ++cy)
        for (std::uint32_t cx = 0; cx < 20; ++cx)
        {
            int from = 0;
            int to = 0;
            if (!snapshot.bit(0, cx, cy, 0) || !snapshot.range(0, cx, cy, 0, from, to)) ++missing;
            else if (from != 0)
                ++notFromZero;
        }
    CHECK(missing == 0);
    CHECK(notFromZero == 0);
    int from = 0;
    int to = 0;
    CHECK(snapshot.range(0, 10, 7, 0, from, to));
    CHECK(to == sliceOf(1.0f + 20.0f * shared.sphereOverfit()) + 1);
}

void testFarPlane(const Setup& setup)
{
    ClusterBuilder& builder = *setup.builder;
    builder.begin(Math::Mat4::Identity(), projection(), false);
    builder.addLight(ClusterBuilder::kLightOmni, at(0.0f, 0.0f, -60.0f), 8.0f, 0.0f);

    Snapshot snapshot;
    CHECK(bakeAndRead(setup, snapshot));
    int from = 0;
    int to = 0;
    CHECK(snapshot.range(0, 10, 7, 0, from, to));
    CHECK(to == 32);
    CHECK(from < 31);
}

void testRotatedAndMoved(const Setup& setup, const ClusterBuilderShared& shared)
{
    ClusterBuilder& builder = *setup.builder;
    const Math::Mat4 camera = Math::Mat4::Translation(Math::Vec3(10.0f, 0.0f, 0.0f)) *
                              Math::Mat4::RotationY(90.0f * Math::DEG2RAD);
    builder.begin(camera, projection(), false);
    builder.addLight(ClusterBuilder::kLightOmni, at(0.0f, 0.0f, 0.0f), 3.0f, 0.0f);

    Snapshot snapshot;
    CHECK(bakeAndRead(setup, snapshot));
    CHECK(snapshot.bit(0, 10, 7, 0));
    CHECK(!snapshot.bit(0, 0, 0, 0));
    const float outer = 3.0f * shared.sphereOverfit();
    CHECK(rangeIs(snapshot, 0, 10, 7, 0, sliceOf(10.0f - outer), sliceOf(10.0f + outer) + 1,
            "moved camera z range"));
}

void testLimits(const Setup& setup)
{
    ClusterBuilder& builder = *setup.builder;
    builder.begin(Math::Mat4::Identity(), projection(), false);
    for (int i = 0; i < 40; ++i)
        builder.addLight(ClusterBuilder::kLightOmni,
                at(static_cast<float>(i % 8) - 4.0f, 0.0f, -20.0f), 1.0f, 0.0f);
    CHECK(builder.getClusterCountByType(ClusterBuilder::kElementOmniLight) == 32);
    CHECK(builder.getElementCount() == 32);
    builder.addBox(ClusterBuilder::kBoxDecal, at(0.0f, 0.0f, -20.0f), Math::Vec3(1.0f, 1.0f, 1.0f));
    CHECK(builder.getClusterCountByType(ClusterBuilder::kElementDecal) == 1);
    CHECK(builder.getElementCount() == 33);

    Snapshot snapshot;
    CHECK(bakeAndRead(setup, snapshot));
    CHECK(snapshot.bit(0, static_cast<std::uint32_t>(screenX(3.0f, 20.0f)) / 32, 7, 31));
    CHECK(snapshot.bit(0, static_cast<std::uint32_t>(screenX(-4.0f, 20.0f)) / 32, 7, 0));
    CHECK(snapshot.bit(3, 10, 7, 0));
}

void testClear(const Setup& setup)
{
    ClusterBuilder& builder = *setup.builder;
    builder.begin(Math::Mat4::Identity(), projection(), false);
    builder.addLight(ClusterBuilder::kLightOmni, at(0.0f, 0.0f, -20.0f), 5.0f, 0.0f);

    Snapshot snapshot;
    CHECK(bakeAndRead(setup, snapshot));
    CHECK(!snapshot.allEmpty());
    CHECK(snapshot.bit(0, 10, 7, 0));

    builder.begin(Math::Mat4::Identity(), projection(), false);
    CHECK(builder.getElementCount() == 0);
    CHECK(bakeAndRead(setup, snapshot));
    CHECK(snapshot.allEmpty());
    CHECK(!snapshot.bit(0, 10, 7, 0));
}

void testResize(const Setup& setup)
{
    ClusterBuilder& builder = *setup.builder;
    CHECK(builder.setup(100, 50, 40));
    CHECK(builder.getClusterWidth() == 4);
    CHECK(builder.getClusterHeight() == 2);
    CHECK(builder.getMaxClusterElements() == 64);
    CHECK(builder.getMaxClusterElementCountDiv32() == 2);
    CHECK(builder.getClusterTypeSize() == 4u * 2u * 34u);
    builder.begin(Math::Mat4::Identity(), projection(), false);
    builder.addLight(ClusterBuilder::kLightOmni, at(0.0f, 0.0f, -10.0f), 2.0f, 0.0f);
    Snapshot snapshot;
    CHECK(bakeAndRead(setup, snapshot));
    CHECK(snapshot.words.size() == 4u * 2u * 34u * 5u);
    CHECK(snapshot.bit(0, 1, 0, 0) || snapshot.bit(0, 1, 1, 0));
    CHECK(!builder.setup(0, 50, 40));
    CHECK(!builder.setup(100, 50, 0));
    CHECK(builder.setup(kWidth, kHeight, kMaxElements));
}

} // namespace

int main(int argc, char** argv)
{
    const bool vulkan = argc > 1 && strcmp(argv[1], "vulkan") == 0;

    CHECK(Device::headlessSupported());
    if (!Device::headlessSupported()) return 1;

    DeviceDesc deviceDesc;
    deviceDesc.title = "cluster test";
    deviceDesc.driver = vulkan ? prisma::DriverType::Vulkan : prisma::DriverType::OpenGL;
    deviceDesc.width = 64;
    deviceDesc.height = 48;
    deviceDesc.headless = true;

    Device* device = Device::create(deviceDesc);
    CHECK(device != nullptr);
    if (!device) return 1;
    prisma::Driver* driver = device->driver();

    ClusterBuilderShared shared;
    CHECK(shared.create(driver));
    CHECK(shared.ready());
    if (!shared.ready()) return 1;

    ClusterBuilder builder;
    CHECK(!builder.setup(kWidth, kHeight, kMaxElements));
    builder.setShared(&shared);
    CHECK(builder.setup(kWidth, kHeight, kMaxElements));

    Setup setup;
    setup.driver = driver;
    setup.device = device;
    setup.builder = &builder;

    testGeometry(setup, shared);
    testOmni(setup, shared);
    testSpot(setup, shared);
    testWideSpot(setup, shared);
    testDecal(setup);
    testAreaLight(setup);
    testScaledBox(setup);
    testVertical(setup);
    testOrthogonal(setup, shared);
    testTwoElements(setup, shared);
    testOverlap(setup);
    testBehindCamera(setup);
    testCameraInside(setup, shared);
    testFarPlane(setup);
    testRotatedAndMoved(setup, shared);
    testLimits(setup);
    testClear(setup);
    testResize(setup);

    builder.destroy();
    CHECK(builder.getClusterBuffer().valid() == false);
    shared.destroy();
    CHECK(!shared.ready());
    Device::destroy(device);

    if (failures)
    {
        printf("%d checks failed\n", failures);
        return 1;
    }
    printf("cluster ok\n");
    return 0;
}
