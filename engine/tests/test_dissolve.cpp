#include "Check.h"
#include "ColouredBox.h"

#include "prisma3d/Device.h"
#include "prisma3d/core/Pixmap.h"
#include "prisma3d/render/MaterialStore.h"
#include "prisma3d/render/MeshStore.h"
#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/render/TextureStore.h"
#include "prisma3d/scene/MultiMesh.h"

#include "platform.h"

#include <math.h>
#include <string.h>

using namespace prisma3d;

namespace
{

const int kSize = 128;
const float kFov = 30.0f;
const float kNear = 0.1f;
const float kFar = 100.0f;
const float kPi = 3.14159265358979f;
const float kDistance = 12.0f;
const float kBoxScale = 1.2f;
const float kSpacing = 1.5f;
const float kRowY = 0.0f;
const float kPlainY = -2.5f;
const float kCameraY = -0.8f;

const int kBayer[8][8] = { { 1, 49, 13, 61, 4, 52, 16, 64 }, { 33, 17, 45, 29, 36, 20, 48, 32 },
    { 9, 57, 5, 53, 12, 60, 8, 56 }, { 41, 25, 37, 21, 44, 28, 40, 24 },
    { 3, 51, 15, 63, 2, 50, 14, 62 }, { 35, 19, 47, 31, 34, 18, 46, 30 },
    { 11, 59, 7, 55, 10, 58, 6, 54 }, { 43, 27, 39, 23, 42, 26, 38, 22 } };

int gErrors = 0;
int checkCount = 0;
bool gFlip = false;

#define TEST(cond)                                                                                 \
    do                                                                                             \
    {                                                                                              \
        ++checkCount;                                                                              \
        CHECK(cond);                                                                               \
    } while (0)

void countErrors(LogLevel level, const char* message, void*)
{
    if (level < LOGLEVEL_ERROR) return;
    ++gErrors;
    printf("logged error: %s\n", message);
}

struct Pix
{
    int r;
    int g;
    int b;
};

struct Image
{
    ct::Vector<std::uint8_t> data;

    Pix at(int x, int y) const
    {
        const std::size_t i = (static_cast<std::size_t>(y) * kSize + x) * 4;
        return Pix{ data[i], data[i + 1], data[i + 2] };
    }
};

int channelDistance(const Pix& a, const Pix& b)
{
    int d = abs(a.r - b.r);
    if (abs(a.g - b.g) > d) d = abs(a.g - b.g);
    if (abs(a.b - b.b) > d) d = abs(a.b - b.b);
    return d;
}

float depthAt(float distance)
{
    const float ndc =
            (kFar + kNear) / (kFar - kNear) - (2.0f * kFar * kNear) / ((kFar - kNear) * distance);
    return ndc * 0.5f + 0.5f;
}

bool near(float a, float b, float eps)
{
    return fabsf(a - b) <= eps;
}

float focal()
{
    return (static_cast<float>(kSize) * 0.5f) / tanf(kFov * 0.5f * kPi / 180.0f);
}

bool keptByPattern(int x, int y, float dissolve)
{
    const int fy = gFlip ? kSize - 1 - y : y;
    const float mask = static_cast<float>(kBayer[x % 8][fy % 8]) / 65.0f;
    return !(mask - dissolve < 0.0f);
}

enum Show
{
    kBoxes = 1,
    kNoCustom = 2,
    kPlain = 4,
    kPlate = 8,
    kShadowBoxes = 16,
    kFloor = 32
};

struct World
{
    RenderingServer* server = nullptr;
    prisma::Driver* driver = nullptr;
    TextureStore* textures = nullptr;
    MaterialStore* materials = nullptr;
    MeshId boxMesh;
    MeshId plateMesh;
    MeshId floorMesh;
    ScenarioId scenario;
    CameraId camera;
    ViewportId viewport;
    LightId sun;
    MultiMeshId boxes;
    MultiMeshId noCustom;
    MultiMeshId plate;
    MultiMeshId shadowBoxes;
    InstanceId boxesInstance;
    InstanceId noCustomInstance;
    InstanceId plainInstance;
    InstanceId plateInstance;
    InstanceId shadowInstance;
    InstanceId floorInstance;
    MaterialId red;
    MaterialId lit;
    MaterialId floorMaterial;
    RenderStats stats;
};

void show(World& w, int mask)
{
    w.server->instanceSetVisible(w.boxesInstance, (mask & kBoxes) != 0);
    w.server->instanceSetVisible(w.noCustomInstance, (mask & kNoCustom) != 0);
    w.server->instanceSetVisible(w.plainInstance, (mask & kPlain) != 0);
    w.server->instanceSetVisible(w.plateInstance, (mask & kPlate) != 0);
    w.server->instanceSetVisible(w.shadowInstance, (mask & kShadowBoxes) != 0);
    w.server->instanceSetVisible(w.floorInstance, (mask & kFloor) != 0);
}

void front(World& w)
{
    w.server->cameraSetTransform(w.camera,
            Math::Mat4::Translation(Math::Vec3(0.0f, kCameraY, kDistance)));
}

Math::Mat4 boxTransform(float x, float y, float z)
{
    return Math::Mat4::Translation(Math::Vec3(x, y, z)) *
           Math::Mat4::Scale(Math::Vec3(kBoxScale, kBoxScale, kBoxScale));
}

float boxX(int index)
{
    return (static_cast<float>(index) - 1.5f) * kSpacing;
}

void setDissolveValues(World& w, MultiMeshId multimesh, const float* values, int count)
{
    for (int i = 0; i < count; ++i)
        w.server->multimeshInstanceSetCustomData(multimesh, static_cast<std::uint32_t>(i),
                Math::Vec4(values[i], 0.0f, 0.0f, 0.0f));
}

Image render(World& w)
{
    w.server->renderViewports();
    w.stats = w.server->getStats();
    w.server->endFrame(true);
    Image image;
    image.data.resize(static_cast<std::size_t>(kSize) * kSize * 4);
    prisma::RenderTarget source;
    source.texture = w.textures->gpu(w.server->viewportGetTexture(w.viewport));
    prisma::Rect rect;
    rect.width = kSize;
    rect.height = kSize;
    CHECK(w.driver->readPixels(source, rect, image.data.data()));
    return image;
}

int countDifferentImages(const Image& a, const Image& b, int tolerance)
{
    int count = 0;
    for (int y = 0; y < kSize; ++y)
        for (int x = 0; x < kSize; ++x)
            if (channelDistance(a.at(x, y), b.at(x, y)) > tolerance) ++count;
    return count;
}

bool isObject(const Image& image, int x, int y, const Pix& background)
{
    return channelDistance(image.at(x, y), background) > 20;
}

int countObject(const Image& image, const Pix& background)
{
    int count = 0;
    for (int y = 0; y < kSize; ++y)
        for (int x = 0; x < kSize; ++x)
            if (isObject(image, x, y, background)) ++count;
    return count;
}

int plainRowTop()
{
    return static_cast<int>(floorf(static_cast<float>(kSize) * 0.5f +
                                   focal() * (kPlainY + kBoxScale + 0.6f - kCameraY) / kDistance));
}

int columnOf(float worldX)
{
    return static_cast<int>(
            floorf(static_cast<float>(kSize) * 0.5f + focal() * worldX / kDistance));
}

int regionOf(int x, int y)
{
    if (y < plainRowTop()) return 4;
    if (x < columnOf(boxX(0) + kSpacing * 0.5f)) return 0;
    if (x < columnOf(boxX(1) + kSpacing * 0.5f)) return 1;
    if (x < columnOf(boxX(2) + kSpacing * 0.5f)) return 2;
    return 3;
}

int patternMismatches(const Image& dissolved, const Image& reference, const Pix& background,
        const float* values, bool flip)
{
    gFlip = flip;
    int mismatches = 0;
    for (int y = 0; y < kSize; ++y)
    {
        for (int x = 0; x < kSize; ++x)
        {
            const int region = regionOf(x, y);
            const float d = region < 4 ? values[region] : 0.0f;
            const bool expected = isObject(reference, x, y, background) && keptByPattern(x, y, d);
            if (expected != isObject(dissolved, x, y, background)) ++mismatches;
        }
    }
    return mismatches;
}

int regionCount(const Image& image, const Pix& background, int region)
{
    int count = 0;
    for (int y = 0; y < kSize; ++y)
        for (int x = 0; x < kSize; ++x)
            if (regionOf(x, y) == region && isObject(image, x, y, background)) ++count;
    return count;
}

void checkMatrix()
{
    printf("bayer matrix\n");
    int seen[65];
    memset(seen, 0, sizeof(seen));
    int below = 0;
    int highest = 0;
    for (int x = 0; x < 8; ++x)
    {
        for (int y = 0; y < 8; ++y)
        {
            ++seen[kBayer[x][y]];
            if (static_cast<float>(kBayer[x][y]) / 65.0f < 0.5f) ++below;
            if (kBayer[x][y] > highest) highest = kBayer[x][y];
        }
    }
    bool permutation = seen[0] == 0;
    for (int k = 1; k <= 64; ++k) permutation = permutation && seen[k] == 1;
    TEST(permutation);
    TEST(below == 32);
    TEST(static_cast<float>(highest) / 65.0f < 0.99f);
    TEST(static_cast<float>(highest) / 65.0f >= 0.98f);
}

void checkAccessors(World& w)
{
    printf("flag accessors\n");
    const MultiMeshId mm = w.server->multimeshCreate();
    TEST(!w.server->multimeshGetDissolve(mm));
    w.server->multimeshSetDissolve(mm, true);
    TEST(w.server->multimeshGetDissolve(mm));
    w.server->multimeshSetDissolve(mm, false);
    TEST(!w.server->multimeshGetDissolve(mm));
    w.server->multimeshSetDissolve(MultiMeshId(), true);
    TEST(!w.server->multimeshGetDissolve(MultiMeshId()));
    w.server->multimeshFree(mm);
    TEST(!w.server->multimeshGetDissolve(mm));

    MultiMesh node;
    TEST(!node.getDissolve());
    node.setDissolve(true);
    TEST(node.getDissolve());
    TEST(w.server->multimeshGetDissolve(node.getId()));
    node.setDissolve(false);
    TEST(!node.getDissolve());
    TEST(!w.server->multimeshGetDissolve(node.getId()));
}

void checkPattern(World& w, const Pix& background)
{
    printf("dissolve pattern on a row of boxes\n");
    front(w);
    show(w, kBoxes | kPlain);
    w.server->setDepthPrePassEnabled(true);
    w.server->multimeshSetDissolve(w.boxes, true);

    const float zeros[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
    const float values[4] = { 0.0f, 0.5f, 0.99f, 1.0f };
    setDissolveValues(w, w.boxes, zeros, 4);
    const Image reference = render(w);
    for (int r = 0; r < 5; ++r) TEST(regionCount(reference, background, r) > 300);
    const int referenceCount[5] = { regionCount(reference, background, 0),
        regionCount(reference, background, 1), regionCount(reference, background, 2),
        regionCount(reference, background, 3), regionCount(reference, background, 4) };

    setDissolveValues(w, w.boxes, values, 4);
    w.server->multimeshSetDissolve(w.boxes, false);
    const Image flagOff = render(w);
    printf("flag off, values set: %d pixels differ from the reference\n",
            countDifferentImages(flagOff, reference, 0));
    TEST(countDifferentImages(flagOff, reference, 0) == 0);

    w.server->multimeshSetDissolve(w.boxes, true);
    const Image dissolved = render(w);
    TEST(w.stats.depthPrePassDrawCalls >= 1);

    const int straight = patternMismatches(dissolved, reference, background, values, false);
    const int flipped = patternMismatches(dissolved, reference, background, values, true);
    printf("pattern mismatches: window origin as read %d, window origin flipped %d\n", straight,
            flipped);
    TEST(straight == 0 || flipped == 0);
    gFlip = straight != 0;
    printf("window origin %s\n", gFlip ? "flipped" : "as read");
    TEST(patternMismatches(dissolved, reference, background, values, gFlip) == 0);

    for (int r = 0; r < 4; ++r)
    {
        int expected = 0;
        for (int y = 0; y < kSize; ++y)
            for (int x = 0; x < kSize; ++x)
                if (regionOf(x, y) == r && isObject(reference, x, y, background) &&
                        keptByPattern(x, y, values[r]))
                    ++expected;
        const int got = regionCount(dissolved, background, r);
        printf("box %d, dissolve %.2f: %d of %d pixels (expected %d)\n", r, values[r], got,
                referenceCount[r], expected);
        TEST(got == expected);
    }
    TEST(regionCount(dissolved, background, 0) == referenceCount[0]);
    TEST(regionCount(dissolved, background, 2) == 0);
    TEST(regionCount(dissolved, background, 3) == 0);
    TEST(regionCount(dissolved, background, 1) > 0);
    TEST(regionCount(dissolved, background, 1) < referenceCount[1]);
    TEST(regionCount(dissolved, background, 4) == referenceCount[4]);

    int blocks = 0;
    int badBlocks = 0;
    for (int by = 0; by < kSize / 8; ++by)
    {
        for (int bx = 0; bx < kSize / 8; ++bx)
        {
            bool inside = true;
            int count = 0;
            for (int y = by * 8; y < by * 8 + 8 && inside; ++y)
            {
                for (int x = bx * 8; x < bx * 8 + 8; ++x)
                {
                    if (regionOf(x, y) != 1 || !isObject(reference, x, y, background))
                    {
                        inside = false;
                        break;
                    }
                }
            }
            if (!inside) continue;
            for (int y = by * 8; y < by * 8 + 8; ++y)
                for (int x = bx * 8; x < bx * 8 + 8; ++x)
                    if (isObject(dissolved, x, y, background)) ++count;
            ++blocks;
            if (count != 32) ++badBlocks;
        }
    }
    printf("%d whole 8x8 blocks at 0.5, %d without exactly 32 pixels\n", blocks, badBlocks);
    TEST(blocks >= 2);
    TEST(badBlocks == 0);
}

void checkPrepass(World& w, const Pix& background)
{
    printf("depth pre-pass on and off\n");
    front(w);
    show(w, kBoxes | kPlain);
    w.server->multimeshSetDissolve(w.boxes, true);
    const float values[4] = { 0.0f, 0.5f, 0.99f, 1.0f };
    setDissolveValues(w, w.boxes, values, 4);

    w.server->setDepthPrePassEnabled(true);
    const Image on = render(w);
    TEST(w.stats.depthPrePassDrawCalls >= 1);
    float keptDepth = -1.0f;
    float hiddenDepth = -1.0f;
    const int centreX = columnOf(boxX(1));
    const int centreY =
            static_cast<int>(floorf(static_cast<float>(kSize) * 0.5f +
                                    focal() * (kRowY + kBoxScale * 0.5f - kCameraY) / kDistance));
    int keptX = -1;
    int keptY = -1;
    int hiddenX = -1;
    int hiddenY = -1;
    for (int y = centreY - 4; y < centreY + 4; ++y)
    {
        for (int x = centreX - 4; x < centreX + 4; ++x)
        {
            if (keptByPattern(x, y, 0.5f))
            {
                if (keptX < 0)
                {
                    keptX = x;
                    keptY = y;
                }
            }
            else if (hiddenX < 0)
            {
                hiddenX = x;
                hiddenY = y;
            }
        }
    }
    TEST(keptX >= 0 && hiddenX >= 0);
    TEST(isObject(on, keptX, keptY, background));
    TEST(!isObject(on, hiddenX, hiddenY, background));
    TEST(w.server->viewportReadDepth(w.viewport, keptX, keptY, keptDepth));
    TEST(w.server->viewportReadDepth(w.viewport, hiddenX, hiddenY, hiddenDepth));
    printf("with the pre-pass: kept %f (expected %f), dissolved %f\n", keptDepth,
            depthAt(kDistance - kBoxScale * 0.5f), hiddenDepth);
    TEST(near(keptDepth, depthAt(kDistance - kBoxScale * 0.5f), 1e-4f));
    TEST(hiddenDepth > 0.9999f);

    w.server->setDepthPrePassEnabled(false);
    const Image off = render(w);
    TEST(w.stats.depthPrePassDrawCalls == 0);
    TEST(w.server->viewportReadDepth(w.viewport, keptX, keptY, keptDepth));
    TEST(w.server->viewportReadDepth(w.viewport, hiddenX, hiddenY, hiddenDepth));
    printf("without the pre-pass: kept %f, dissolved %f\n", keptDepth, hiddenDepth);
    TEST(near(keptDepth, depthAt(kDistance - kBoxScale * 0.5f), 1e-4f));
    TEST(hiddenDepth > 0.9999f);
    const int differing = countDifferentImages(on, off, 1);
    printf("pre-pass on and off differ in %d pixels\n", differing);
    TEST(differing == 0);
    w.server->setDepthPrePassEnabled(true);
}

void checkWithoutCustomData(World& w, const Pix& background)
{
    printf("multimesh without per-instance data\n");
    front(w);
    w.server->multimeshSetDissolve(w.noCustom, true);
    TEST(w.server->multimeshGetDissolve(w.noCustom));
    show(w, kNoCustom | kPlain);
    const Image plain = render(w);
    w.server->multimeshSetDissolve(w.noCustom, false);
    const Image flagOff = render(w);
    TEST(countDifferentImages(plain, flagOff, 0) == 0);
    for (int r = 0; r < 4; ++r) TEST(regionCount(plain, background, r) > 300);
    w.server->multimeshSetDissolve(w.noCustom, true);

    const float zeros[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
    setDissolveValues(w, w.boxes, zeros, 4);
    w.server->multimeshSetDissolve(w.boxes, true);
    show(w, kBoxes | kPlain);
    const Image withData = render(w);
    TEST(countDifferentImages(plain, withData, 0) == 0);
}

void checkScissor(World& w, const Pix& background, MaterialId referenceMaterial, MaterialId cut)
{
    printf("alpha scissor material with dissolve\n");
    w.server->cameraSetTransform(w.camera, Math::Mat4::Translation(Math::Vec3(0.0f, 0.0f, 12.0f)));
    show(w, kPlate);
    const float half[1] = { 0.5f };
    const float none[1] = { 0.0f };
    for (int mode = 0; mode < 2; ++mode)
    {
        w.server->setDepthPrePassEnabled(mode == 0);
        setDissolveValues(w, w.plate, half, 1);
        w.server->multimeshSetDissolve(w.plate, false);
        w.server->instanceSetMaterialOverride(w.plateInstance, referenceMaterial);
        const Image reference = render(w);
        const int referenceCount = countObject(reference, background);
        w.server->instanceSetMaterialOverride(w.plateInstance, cut);
        const Image flagOff = render(w);
        TEST(countObject(flagOff, background) >= countObject(reference, background));
        w.server->multimeshSetDissolve(w.plate, true);
        const Image dissolved = render(w);
        if (mode == 0) TEST(w.stats.depthPrePassDrawCalls == 1);
        else
            TEST(w.stats.depthPrePassDrawCalls == 0);
        int expected = 0;
        int mismatches = 0;
        for (int y = 0; y < kSize; ++y)
        {
            for (int x = 0; x < kSize; ++x)
            {
                const bool want =
                        isObject(reference, x, y, background) && keptByPattern(x, y, 0.5f);
                if (want) ++expected;
                if (want != isObject(dissolved, x, y, background)) ++mismatches;
            }
        }
        const int got = countObject(dissolved, background);
        printf("pre-pass %d: cut plate %d pixels, dissolved %d, expected %d, %d mismatches\n",
                mode == 0, referenceCount, got, expected, mismatches);
        TEST(referenceCount > 2000);
        TEST(got == expected);
        TEST(mismatches == 0);
        TEST(got * 2 < referenceCount + referenceCount / 4);
        TEST(got * 2 > referenceCount - referenceCount / 4);
    }
    w.server->setDepthPrePassEnabled(true);
    w.server->multimeshSetDissolve(w.plate, false);
    setDissolveValues(w, w.plate, none, 1);
}

int intermediateCount(const Image& image, const Pix& background)
{
    int count = 0;
    for (int y = 0; y < kSize; ++y)
    {
        for (int x = 0; x < kSize; ++x)
        {
            const Pix p = image.at(x, y);
            if (p.g > 8 && p.g < background.g - 8) ++count;
        }
    }
    return count;
}

void checkMsaa(World& w, const Pix& background, MaterialId coverage)
{
    printf("multisampling with alpha antialiasing\n");
    if (w.driver->caps().maxSamples < 4)
    {
        printf("no 4x multisampling, skipped\n");
        return;
    }
    const int errorsBefore = gErrors;
    w.server->cameraSetTransform(w.camera, Math::Mat4::Translation(Math::Vec3(0.0f, 0.0f, 12.0f)));
    show(w, kPlate);
    w.server->instanceSetMaterialOverride(w.plateInstance, coverage);
    w.server->viewportSetMsaa3D(w.viewport, ViewportMsaa::Msaa4x);
    const float none[1] = { 0.0f };
    const float quarter[1] = { 0.25f };
    const float half[1] = { 0.5f };
    for (int mode = 0; mode < 2; ++mode)
    {
        w.server->setDepthPrePassEnabled(mode == 0);
        setDissolveValues(w, w.plate, none, 1);
        w.server->multimeshSetDissolve(w.plate, true);
        const Image reference = render(w);
        TEST(w.server->viewportGetMsaaSamples(w.viewport) == 4);
        TEST(w.stats.coverageSurfaces == 1);
        setDissolveValues(w, w.plate, quarter, 1);
        const Image dissolved = render(w);
        TEST(w.stats.coverageSurfaces == 1);
        const int referenceCount = countObject(reference, background);
        const int dissolvedCount = countObject(dissolved, background);
        const int referenceSoft = intermediateCount(reference, background);
        const int dissolvedSoft = intermediateCount(dissolved, background);
        printf("pre-pass %d, dissolve 0.25: %d object pixels (%d without), %d intermediate "
               "(%d without)\n",
                mode == 0, dissolvedCount, referenceCount, dissolvedSoft, referenceSoft);
        TEST(referenceCount > 1000);
        TEST(dissolvedCount > 0);
        TEST(dissolvedCount < referenceCount);
        TEST(dissolvedSoft > 0);
        setDissolveValues(w, w.plate, half, 1);
        const Image gone = render(w);
        printf("pre-pass %d, dissolve 0.5: %d object pixels\n", mode == 0,
                countObject(gone, background));
    }
    w.server->setDepthPrePassEnabled(true);
    w.server->multimeshSetDissolve(w.plate, false);
    setDissolveValues(w, w.plate, none, 1);
    w.server->viewportSetMsaa3D(w.viewport, ViewportMsaa::Disabled);
    w.server->instanceSetMaterialOverride(w.plateInstance, MaterialId());
    TEST(gErrors == errorsBefore);
}

struct Window
{
    int x0;
    int x1;
    int y0;
    int y1;
};

Window windowOf(float x0, float x1, float z0, float z1, float height)
{
    const float scale = focal() / height;
    const float cx = static_cast<float>(kSize) * 0.5f;
    Window window;
    window.x0 = static_cast<int>(floorf(cx + scale * x0));
    window.x1 = static_cast<int>(floorf(cx + scale * x1));
    const int ya = static_cast<int>(floorf(cx - scale * z0));
    const int yb = static_cast<int>(floorf(cx - scale * z1));
    window.y0 = ya < yb ? ya : yb;
    window.y1 = ya < yb ? yb : ya;
    return window;
}

int shadowCount(const Image& image, const Window& window, const Pix& litFloor)
{
    int count = 0;
    for (int y = window.y0; y <= window.y1; ++y)
        for (int x = window.x0; x <= window.x1; ++x)
            if (image.at(x, y).g * 10 < litFloor.g * 8) ++count;
    return count;
}

void checkShadows(World& w)
{
    printf("shadows do not dissolve\n");
    const float height = 14.0f;
    w.server->cameraSetTransform(w.camera, Math::Mat4::Translation(Math::Vec3(0.0f, height, 0.0f)) *
                                                   Math::Mat4::RotationX(-kPi * 0.5f));
    show(w, kShadowBoxes | kFloor);
    w.server->lightSetShadow(w.sun, true);
    w.server->lightSetTransform(w.sun,
            Math::Mat4::RotationY(-kPi * 0.5f) * Math::Mat4::RotationX(-35.0f * kPi / 180.0f));
    const float z = 1.8f;
    w.server->multimeshInstanceSetTransform(w.shadowBoxes, 0, boxTransform(-1.5f, 0.0f, -z));
    w.server->multimeshInstanceSetTransform(w.shadowBoxes, 1, boxTransform(-1.5f, 0.0f, z));
    const float values[2] = { 0.5f, 0.0f };
    const float zeros[2] = { 0.0f, 0.0f };
    setDissolveValues(w, w.shadowBoxes, values, 2);

    const Window upper = windowOf(-0.7f, 3.0f, -z - 0.5f, -z + 0.5f, height);
    const Window lower = windowOf(-0.7f, 3.0f, z - 0.5f, z + 0.5f, height);

    w.server->multimeshSetDissolve(w.shadowBoxes, false);
    const Image off = render(w);
    const Pix litFloor = off.at(columnOf(0.0f) - 40, kSize / 2);
    w.server->multimeshSetDissolve(w.shadowBoxes, true);
    const Image on = render(w);
    TEST(w.stats.shadowPasses >= 1);
    const int offUpper = shadowCount(off, upper, litFloor);
    const int offLower = shadowCount(off, lower, litFloor);
    const int onUpper = shadowCount(on, upper, litFloor);
    const int onLower = shadowCount(on, lower, litFloor);
    printf("floor %d,%d,%d; shadow pixels flag off %d and %d, flag on %d and %d\n", litFloor.r,
            litFloor.g, litFloor.b, offUpper, offLower, onUpper, onLower);
    TEST(onUpper > 100 && onLower > 100);
    TEST(onUpper == offUpper);
    TEST(onLower == offLower);
    const int larger = onUpper > onLower ? onUpper : onLower;
    const int smaller = onUpper > onLower ? onLower : onUpper;
    TEST((larger - smaller) * 50 <= larger);

    int differing = 0;
    for (int y = upper.y0; y <= upper.y1; ++y)
        for (int x = upper.x0; x <= upper.x1; ++x)
            if (channelDistance(on.at(x, y), off.at(x, y)) > 0) ++differing;
    printf("shadow window differs in %d pixels with the flag\n", differing);
    TEST(differing == 0);

    w.server->multimeshSetDissolve(w.shadowBoxes, false);
    setDissolveValues(w, w.shadowBoxes, zeros, 2);
    w.server->lightSetShadow(w.sun, false);
}

SurfaceData whiteBox()
{
    SurfaceData data;
    const float h = 0.5f;
    const Math::Vec4 white(1.0f, 1.0f, 1.0f, 1.0f);
    addBoxFace(data, Math::Vec3(-h, 0, h), Math::Vec3(h, 0, h), Math::Vec3(h, 1, h),
            Math::Vec3(-h, 1, h), Math::Vec3(0, 0, 1), white);
    addBoxFace(data, Math::Vec3(h, 0, h), Math::Vec3(h, 0, -h), Math::Vec3(h, 1, -h),
            Math::Vec3(h, 1, h), Math::Vec3(1, 0, 0), white);
    addBoxFace(data, Math::Vec3(h, 0, -h), Math::Vec3(-h, 0, -h), Math::Vec3(-h, 1, -h),
            Math::Vec3(h, 1, -h), Math::Vec3(0, 0, -1), white);
    addBoxFace(data, Math::Vec3(-h, 0, -h), Math::Vec3(-h, 0, h), Math::Vec3(-h, 1, h),
            Math::Vec3(-h, 1, -h), Math::Vec3(-1, 0, 0), white);
    addBoxFace(data, Math::Vec3(-h, 1, h), Math::Vec3(h, 1, h), Math::Vec3(h, 1, -h),
            Math::Vec3(-h, 1, -h), Math::Vec3(0, 1, 0), white);
    addBoxFace(data, Math::Vec3(-h, 0, -h), Math::Vec3(h, 0, -h), Math::Vec3(h, 0, h),
            Math::Vec3(-h, 0, h), Math::Vec3(0, -1, 0), white);
    return data;
}

SurfaceData floorData(float half)
{
    SurfaceData data;
    const Math::Vec4 white(1.0f, 1.0f, 1.0f, 1.0f);
    addBoxFace(data, Math::Vec3(-half, 0, half), Math::Vec3(half, 0, half),
            Math::Vec3(half, 0, -half), Math::Vec3(-half, 0, -half), Math::Vec3(0, 1, 0), white);
    return data;
}

SurfaceData plateData(float half)
{
    SurfaceData data;
    data.positions.push_back(Math::Vec3(-half, -half, 0.0f));
    data.positions.push_back(Math::Vec3(half, -half, 0.0f));
    data.positions.push_back(Math::Vec3(half, half, 0.0f));
    data.positions.push_back(Math::Vec3(-half, half, 0.0f));
    data.uvs.push_back(Math::Vec2(0.0f, 1.0f));
    data.uvs.push_back(Math::Vec2(1.0f, 1.0f));
    data.uvs.push_back(Math::Vec2(1.0f, 0.0f));
    data.uvs.push_back(Math::Vec2(0.0f, 0.0f));
    for (int i = 0; i < 4; ++i)
    {
        data.normals.push_back(Math::Vec3(0.0f, 0.0f, 1.0f));
        data.colors.push_back(Math::Vec4(1.0f, 1.0f, 1.0f, 1.0f));
    }
    const std::uint32_t indices[6] = { 0, 1, 2, 0, 2, 3 };
    for (std::uint32_t index: indices) data.indices.push_back(index);
    return data;
}

EnvironmentSettings environmentSettings()
{
    EnvironmentSettings environment;
    environment.background = BackgroundMode::ClearColor;
    environment.backgroundColor = Math::Vec3(0.7f, 0.7f, 0.7f);
    environment.ambientSource = AmbientSource::Color;
    environment.ambientColor = Math::Vec3(0.3f, 0.3f, 0.3f);
    environment.reflectionSource = ReflectionSource::Disabled;
    return environment;
}

MaterialId unshadedRed(MaterialStore& materials, const char* name)
{
    const MaterialId material = materials.create(name);
    CHECK(materials.setShading(material, MaterialShading::Unshaded));
    CHECK(materials.setColor(material, MaterialParam::Albedo, Math::Vec4(1.0f, 0.0f, 0.0f, 1.0f)));
    return material;
}

} // namespace

int main(int argc, char** argv)
{
    const bool vulkan = argc > 1 && strcmp(argv[1], "vulkan") == 0;

    CHECK(Device::headlessSupported());
    if (!Device::headlessSupported()) return 1;
    log_set_callback(countErrors, nullptr);

    DeviceDesc deviceDesc;
    deviceDesc.title = "dissolve test";
    deviceDesc.driver = vulkan ? prisma::DriverType::Vulkan : prisma::DriverType::OpenGL;
    deviceDesc.width = kSize;
    deviceDesc.height = kSize;
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

        World w;
        w.server = &server;
        w.driver = driver;
        w.textures = &textures;
        w.materials = &materials;

        w.boxMesh = meshes.create("box");
        CHECK(meshes.addSurface(w.boxMesh, whiteBox()));
        w.plateMesh = meshes.create("plate");
        CHECK(meshes.addSurface(w.plateMesh, plateData(1.0f)));
        w.floorMesh = meshes.create("floor");
        CHECK(meshes.addSurface(w.floorMesh, floorData(6.0f)));

        w.scenario = server.scenarioCreate();
        server.scenarioSetEnvironment(w.scenario, environmentSettings());
        w.camera = server.cameraCreate();
        server.cameraSetPerspective(w.camera, kFov, kNear, kFar);
        w.viewport = server.viewportCreate();
        server.viewportSetSize(w.viewport, kSize, kSize);
        server.viewportSetScenario(w.viewport, w.scenario);
        server.viewportAttachCamera(w.viewport, w.camera);
        server.viewportSetClearColor(w.viewport, Math::Vec4(0.7f, 0.7f, 0.7f, 1.0f));
        server.viewportSetUpdateMode(w.viewport, ViewportUpdateMode::Always);
        server.viewportSetActive(w.viewport, true);

        w.sun = server.lightCreate(LightType::Directional);
        server.lightSetScenario(w.sun, w.scenario);
        server.lightSetTransform(w.sun, Math::Mat4::RotationY(0.6f) * Math::Mat4::RotationX(-0.7f));
        server.lightSetShadow(w.sun, false);

        w.red = unshadedRed(materials, "red");
        w.lit = materials.create("lit");
        CHECK(materials.setColor(w.lit, MaterialParam::Albedo, Math::Vec4(0.8f, 0.8f, 0.8f, 1.0f)));
        w.floorMaterial = materials.create("floor");
        CHECK(materials.setColor(w.floorMaterial, MaterialParam::Albedo,
                Math::Vec4(0.8f, 0.8f, 0.8f, 1.0f)));

        Pixmap cutPixels(64, 64, 4);
        for (std::uint32_t y = 0; y < 64; ++y)
            for (std::uint32_t x = 0; x < 64; ++x)
                cutPixels.set_pixel(x, y, 255, 255, 255, x < 32 ? 255 : 40);
        const TextureId cutTexture = textures.createFromPixmap(cutPixels, false, false, "cut");
        const MaterialId cut = unshadedRed(materials, "cut");
        CHECK(materials.setTexture(cut, MaterialTexture::Albedo, cutTexture));
        CHECK(materials.setTransparency(cut, MaterialTransparency::AlphaScissor));
        CHECK(materials.setFloat(cut, MaterialParam::AlphaScissorThreshold, 0.25f));
        const MaterialId cutReference = unshadedRed(materials, "cut reference");
        CHECK(materials.setTexture(cutReference, MaterialTexture::Albedo, cutTexture));
        CHECK(materials.setTransparency(cutReference, MaterialTransparency::AlphaScissor));
        CHECK(materials.setFloat(cutReference, MaterialParam::AlphaScissorThreshold, 0.5f));

        Pixmap gradientPixels(128, 128, 4);
        for (std::uint32_t y = 0; y < 128; ++y)
        {
            for (std::uint32_t x = 0; x < 128; ++x)
            {
                const float u = (static_cast<float>(x) + 0.5f) / 128.0f;
                const float v = (static_cast<float>(y) + 0.5f) / 128.0f;
                const float alpha = u * 0.8f + v * 0.2f;
                gradientPixels.set_pixel(x, y, 255, 255, 255,
                        static_cast<std::uint8_t>(alpha * 255.0f + 0.5f));
            }
        }
        const TextureId gradientTexture =
                textures.createFromPixmap(gradientPixels, false, false, "gradient");
        const MaterialId coverage = unshadedRed(materials, "coverage");
        CHECK(materials.setTexture(coverage, MaterialTexture::Albedo, gradientTexture));
        CHECK(materials.setTransparency(coverage, MaterialTransparency::AlphaScissor));
        CHECK(materials.setFloat(coverage, MaterialParam::AlphaScissorThreshold, 0.1f));
        CHECK(materials.setAlphaAntialiasing(coverage, MaterialAlphaAntialiasing::AlphaToCoverage,
                0.1f));

        w.boxes = server.multimeshCreate();
        server.multimeshAllocateData(w.boxes, 4, false, true);
        server.multimeshSetMesh(w.boxes, w.boxMesh);
        w.noCustom = server.multimeshCreate();
        server.multimeshAllocateData(w.noCustom, 4, false, false);
        server.multimeshSetMesh(w.noCustom, w.boxMesh);
        for (int i = 0; i < 4; ++i)
        {
            const Math::Mat4 transform = boxTransform(boxX(i), kRowY, 0.0f);
            server.multimeshInstanceSetTransform(w.boxes, static_cast<std::uint32_t>(i), transform);
            server.multimeshInstanceSetTransform(w.noCustom, static_cast<std::uint32_t>(i),
                    transform);
        }
        w.plate = server.multimeshCreate();
        server.multimeshAllocateData(w.plate, 1, false, true);
        server.multimeshSetMesh(w.plate, w.plateMesh);
        server.multimeshInstanceSetTransform(w.plate, 0,
                Math::Mat4::Scale(Math::Vec3(2.0f, 2.0f, 2.0f)));
        w.shadowBoxes = server.multimeshCreate();
        server.multimeshAllocateData(w.shadowBoxes, 2, false, true);
        server.multimeshSetMesh(w.shadowBoxes, w.boxMesh);

        w.boxesInstance = server.instanceCreate();
        server.instanceSetBase(w.boxesInstance, w.boxes);
        server.instanceSetScenario(w.boxesInstance, w.scenario);
        server.instanceSetMaterialOverride(w.boxesInstance, w.red);
        w.noCustomInstance = server.instanceCreate();
        server.instanceSetBase(w.noCustomInstance, w.noCustom);
        server.instanceSetScenario(w.noCustomInstance, w.scenario);
        server.instanceSetMaterialOverride(w.noCustomInstance, w.red);
        w.plainInstance = server.instanceCreate(w.boxMesh, w.scenario);
        server.instanceSetMaterialOverride(w.plainInstance, w.red);
        server.instanceSetTransform(w.plainInstance, boxTransform(boxX(2), kPlainY, 0.0f));
        w.plateInstance = server.instanceCreate();
        server.instanceSetBase(w.plateInstance, w.plate);
        server.instanceSetScenario(w.plateInstance, w.scenario);
        server.instanceSetMaterialOverride(w.plateInstance, cut);
        w.shadowInstance = server.instanceCreate();
        server.instanceSetBase(w.shadowInstance, w.shadowBoxes);
        server.instanceSetScenario(w.shadowInstance, w.scenario);
        server.instanceSetMaterialOverride(w.shadowInstance, w.lit);
        w.floorInstance = server.instanceCreate(w.floorMesh, w.scenario);
        server.instanceSetMaterialOverride(w.floorInstance, w.floorMaterial);
        server.setDepthPrePassEnabled(true);

        show(w, 0);
        front(w);
        const Image empty = render(w);
        const Pix background = empty.at(2, 2);
        printf("background %d,%d,%d\n", background.r, background.g, background.b);
        TEST(countObject(empty, background) == 0);

        checkMatrix();
        checkAccessors(w);
        checkPattern(w, background);
        checkPrepass(w, background);
        checkWithoutCustomData(w, background);
        checkScissor(w, background, cutReference, cut);
        checkMsaa(w, background, coverage);
        checkShadows(w);

        TEST(gErrors == 0);
        printf("dissolve: %d checks\n", checkCount);
    }

    Device::destroy(device);
    log_set_callback(nullptr, nullptr);
    if (failures == 0) printf("dissolve: all checks passed\n");
    return failures == 0 ? 0 : 1;
}
