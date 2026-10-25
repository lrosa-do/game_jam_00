#include "Check.h"
#include "ColouredBox.h"
#include "MarkedBox.h"

#include "prisma3d/Device.h"
#include "prisma3d/render/MaterialStore.h"
#include "prisma3d/render/MeshStore.h"
#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/render/TextureStore.h"

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
const float kScale = 2.0f;
const float kFade = 10.0f;
const float kBoxCentreY = 1.0f;
const float kRadius = 1.7320508f;
const float kPi = 3.14159265358979f;

int gErrors = 0;

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

bool pixelNear(const char* label, const Pix& got, const Pix& want, int tolerance)
{
    if (channelDistance(got, want) <= tolerance) return true;
    printf("%s: got %d,%d,%d expected %d,%d,%d (tolerance %d)\n", label, got.r, got.g, got.b,
            want.r, want.g, want.b, tolerance);
    return false;
}

float toLinear(float value)
{
    return value <= 0.04045f ? value / 12.92f : powf((value + 0.055f) / 1.055f, 2.4f);
}

float toSrgb(float value)
{
    return value <= 0.0031308f ? value * 12.92f : 1.055f * powf(value, 1.0f / 2.4f) - 0.055f;
}

int blendByte(int under, int over, float coverage)
{
    const float a = toLinear(static_cast<float>(under) / 255.0f);
    const float b = toLinear(static_cast<float>(over) / 255.0f);
    return static_cast<int>(toSrgb(a * (1.0f - coverage) + b * coverage) * 255.0f + 0.5f);
}

Pix blend(const Pix& under, const Pix& over, float coverage)
{
    return Pix{ blendByte(under.r, over.r, coverage), blendByte(under.g, over.g, coverage),
        blendByte(under.b, over.b, coverage) };
}

float depthAt(float distance)
{
    const float ndc =
            (kFar + kNear) / (kFar - kNear) - (2.0f * kFar * kNear) / ((kFar - kNear) * distance);
    return ndc * 0.5f + 0.5f;
}

struct World
{
    RenderingServer* server = nullptr;
    prisma::Driver* driver = nullptr;
    TextureStore* textures = nullptr;
    MaterialStore* materials = nullptr;
    MeshId boxMesh;
    MeshId cubeMesh;
    MeshId plateMesh;
    ScenarioId scenario;
    CameraId camera;
    ViewportId viewport;
    LightId sun;
    InstanceId box;
    InstanceId cube;
    InstanceId plate;
    ImpostorId impostor;
    ImpostorSetId set;
    Math::Vec3 eye;
    float yaw = 0.0f;
    RenderStats stats;
};

Math::Mat4 placement(const Math::Vec3& position)
{
    return Math::Mat4::Translation(position) *
           Math::Mat4::Scale(Math::Vec3(kScale, kScale, kScale));
}

void aim(World& w, float azimuth, float distance)
{
    const Math::Vec3 centre(0.0f, kBoxCentreY, 0.0f);
    w.eye = centre + Math::Vec3(sinf(azimuth), 0.0f, cosf(azimuth)) * distance;
    w.yaw = azimuth;
    w.server->cameraSetTransform(w.camera,
            Math::Mat4::Translation(w.eye) * Math::Mat4::RotationY(azimuth));
}

void arrange(World& w, const Math::Vec3& boxAt, bool showBox, const Math::Vec3& cardAt,
        bool showCard)
{
    w.server->instanceSetTransform(w.box, placement(boxAt));
    w.server->instanceSetVisible(w.box, showBox);
    w.server->impostorSetInstance(w.set, 0, w.impostor, placement(cardAt),
            Math::Vec4(1.0f, 1.0f, 1.0f, 1.0f));
    w.server->impostorSetEnabled(w.set, showCard);
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

bool project(const World& w, const Math::Vec3& point, int& x, int& y)
{
    const Math::Vec3 forward(-sinf(w.yaw), 0.0f, -cosf(w.yaw));
    const Math::Vec3 right(cosf(w.yaw), 0.0f, -sinf(w.yaw));
    const Math::Vec3 d = point - w.eye;
    const float depth = d.x * forward.x + d.y * forward.y + d.z * forward.z;
    if (depth <= 0.0f) return false;
    const float focal = (static_cast<float>(kSize) * 0.5f) / tanf(kFov * 0.5f * kPi / 180.0f);
    const float sx = (d.x * right.x + d.y * right.y + d.z * right.z) / depth;
    const float sy = d.y / depth;
    x = static_cast<int>(floorf(static_cast<float>(kSize) * 0.5f + focal * sx));
    y = static_cast<int>(floorf(static_cast<float>(kSize) * 0.5f + focal * sy));
    return true;
}

int countDifferent(const Image& image, const Pix& background, int threshold)
{
    int count = 0;
    for (int y = 0; y < kSize; ++y)
        for (int x = 0; x < kSize; ++x)
            if (channelDistance(image.at(x, y), background) > threshold) ++count;
    return count;
}

int countDifferentImages(const Image& a, const Image& b, int tolerance)
{
    int count = 0;
    for (int y = 0; y < kSize; ++y)
        for (int x = 0; x < kSize; ++x)
            if (channelDistance(a.at(x, y), b.at(x, y)) > tolerance) ++count;
    return count;
}

SurfaceData plateData(float half)
{
    SurfaceData data;
    data.positions.push_back(Math::Vec3(-half, -half, 0.0f));
    data.positions.push_back(Math::Vec3(half, -half, 0.0f));
    data.positions.push_back(Math::Vec3(half, half, 0.0f));
    data.positions.push_back(Math::Vec3(-half, half, 0.0f));
    for (int i = 0; i < 4; ++i)
    {
        data.normals.push_back(Math::Vec3(0.0f, 0.0f, 1.0f));
        data.colors.push_back(Math::Vec4(1.0f, 1.0f, 1.0f, 1.0f));
    }
    const std::uint32_t indices[6] = { 0, 1, 2, 0, 2, 3 };
    for (std::uint32_t index: indices) data.indices.push_back(index);
    return data;
}

EnvironmentSettings baseEnvironment()
{
    EnvironmentSettings environment;
    environment.background = BackgroundMode::ClearColor;
    environment.backgroundColor = Math::Vec3(0.7f, 0.7f, 0.7f);
    environment.ambientSource = AmbientSource::Color;
    environment.ambientColor = Math::Vec3(0.3f, 0.3f, 0.3f);
    environment.reflectionSource = ReflectionSource::Disabled;
    return environment;
}

struct Quadrant
{
    const char* label;
    float x;
    float y;
};

const Quadrant kQuadrants[4] = { { "lower left", -0.5f, 0.5f }, { "lower right", 0.5f, 0.5f },
    { "upper left", -0.5f, 1.5f }, { "upper right", 0.5f, 1.5f } };

bool quadrantPixel(const World& w, int quadrant, float x, float z, int& px, int& py)
{
    return project(w, Math::Vec3(x + kQuadrants[quadrant].x, kQuadrants[quadrant].y, z), px, py);
}

int checkCount = 0;

#define TEST(cond)                                                                                 \
    do                                                                                             \
    {                                                                                              \
        ++checkCount;                                                                              \
        CHECK(cond);                                                                               \
    } while (0)

bool near(float a, float b, float eps)
{
    return fabsf(a - b) <= eps;
}

void checkFront(World& w, const Pix& background)
{
    printf("front view, mesh and cards side by side\n");
    aim(w, 0.0f, 20.0f);
    const Math::Vec3 meshAt(-3.0f, 0.0f, 0.0f);
    const Math::Vec3 cardAt(3.0f, 0.0f, 0.0f);
    arrange(w, meshAt, true, cardAt, true);
    const Image both = render(w);
    TEST(w.stats.impostorSets == 1);

    const Pix wanted[4] = { Pix{ 1, 0, 0 }, Pix{ 1, 0, 1 }, Pix{ 0, 1, 1 }, Pix{ 1, 1, 1 } };
    for (int q = 0; q < 4; ++q)
    {
        int mx = 0;
        int my = 0;
        int cx = 0;
        int cy = 0;
        TEST(quadrantPixel(w, q, meshAt.x, 1.0f, mx, my));
        TEST(quadrantPixel(w, q, cardAt.x, 0.0f, cx, cy));
        const Pix mesh = both.at(mx, my);
        const Pix card = both.at(cx, cy);
        char label[64];
        snprintf(label, sizeof(label), "mesh %s has the marked colour", kQuadrants[q].label);
        const bool marked = (wanted[q].r ? mesh.r > 90 : mesh.r < 40) &&
                            (wanted[q].g ? mesh.g > 90 : mesh.g < 40) &&
                            (wanted[q].b ? mesh.b > 90 : mesh.b < 40);
        if (!marked) printf("%s: got %d,%d,%d\n", label, mesh.r, mesh.g, mesh.b);
        TEST(marked);
        snprintf(label, sizeof(label), "card %s equals the mesh", kQuadrants[q].label);
        TEST(pixelNear(label, card, mesh, 3));
    }

    int cornerX = 0;
    int cornerY = 0;
    for (int c = 0; c < 4; ++c)
    {
        const float dx = (c & 1) ? 1.5f : -1.5f;
        const float dy = (c & 2) ? 1.5f : -1.5f;
        TEST(project(w, Math::Vec3(cardAt.x + dx, kBoxCentreY + dy, 0.0f), cornerX, cornerY));
        TEST(pixelNear("card corner shows the background", both.at(cornerX, cornerY), background,
                0));
        TEST(project(w, Math::Vec3(meshAt.x + dx, kBoxCentreY + dy, 0.0f), cornerX, cornerY));
        TEST(pixelNear("mesh corner shows the background", both.at(cornerX, cornerY), background,
                0));
    }

    int ax = 0;
    int ay = 0;
    int bx = 0;
    int by = 0;
    TEST(project(w, Math::Vec3(cardAt.x - 0.9f, 1.0f, 0.0f), ax, ay));
    TEST(project(w, Math::Vec3(cardAt.x + 0.9f, 1.0f, 0.0f), bx, by));
    TEST(channelDistance(both.at(ax, ay), background) > 20);
    TEST(channelDistance(both.at(bx, by), background) > 20);
    TEST(project(w, Math::Vec3(cardAt.x - 1.2f, 1.0f, 0.0f), ax, ay));
    TEST(pixelNear("just outside the box", both.at(ax, ay), background, 0));
}

struct Azimuth
{
    const char* label;
    float degrees;
    float offsetPixels;
};

void checkAngles(World& w)
{
    printf("four directions and a diagonal\n");
    const Azimuth azimuths[5] = { { "+Z", 0.0f, 4.0f }, { "+X", 90.0f, 4.0f },
        { "-Z", 180.0f, 4.0f }, { "-X", 270.0f, 4.0f }, { "diagonal", 45.0f, 8.0f } };
    const Math::Vec3 origin(0.0f, 0.0f, 0.0f);
    for (const Azimuth& a: azimuths)
    {
        aim(w, a.degrees * kPi / 180.0f, 20.0f);
        arrange(w, origin, true, origin, false);
        const Image mesh = render(w);
        TEST(w.stats.impostorSets == 0);
        arrange(w, origin, false, origin, true);
        const Image card = render(w);
        TEST(w.stats.impostorSets == 1);
        TEST(w.stats.impostorDrawCalls == 1);
        for (int side = -1; side <= 1; side += 2)
        {
            const int x = kSize / 2 + side * static_cast<int>(a.offsetPixels);
            const int y = kSize / 2 + 6;
            char label[64];
            snprintf(label, sizeof(label), "%s side %d", a.label, side);
            TEST(pixelNear(label, card.at(x, y), mesh.at(x, y), 3));
        }
    }
}

void checkLighting(World& w)
{
    printf("sunlit and shadowed faces\n");
    const Math::Vec3 origin(0.0f, 0.0f, 0.0f);
    Pix meshFace[4];
    Pix cardFace[4];
    const float degrees[4] = { 0.0f, 90.0f, 180.0f, 270.0f };
    for (int i = 0; i < 4; ++i)
    {
        aim(w, degrees[i] * kPi / 180.0f, 20.0f);
        arrange(w, origin, true, origin, false);
        meshFace[i] = render(w).at(kSize / 2 - 6, kSize / 2 + 6);
        arrange(w, origin, false, origin, true);
        cardFace[i] = render(w).at(kSize / 2 - 6, kSize / 2 + 6);
        char label[64];
        snprintf(label, sizeof(label), "lit face seen from %.0f degrees", degrees[i]);
        TEST(pixelNear(label, cardFace[i], meshFace[i], 4));
    }
    TEST(meshFace[1].g > meshFace[3].g + 20);
    TEST(cardFace[1].g > cardFace[3].g + 20);
    printf("lit +X %d,%d,%d  shadowed -X %d,%d,%d\n", meshFace[1].r, meshFace[1].g, meshFace[1].b,
            meshFace[3].r, meshFace[3].g, meshFace[3].b);
}

void checkPrepass(World& w, const Pix& background)
{
    printf("depth pre-pass on and off\n");
    const Math::Vec3 meshAt(-3.0f, 0.0f, 0.0f);
    const Math::Vec3 cardAt(3.0f, 0.0f, 0.0f);
    aim(w, 0.0f, 20.0f);
    arrange(w, meshAt, true, cardAt, true);
    w.server->setDepthPrePassEnabled(true);
    const Image on = render(w);
    TEST(w.stats.depthPrePassDrawCalls >= 1);
    TEST(w.stats.impostorSets == 1);
    int cx = 0;
    int cy = 0;
    TEST(quadrantPixel(w, 3, cardAt.x, 0.0f, cx, cy));
    float depthOn = 1.0f;
    TEST(w.server->viewportReadDepth(w.viewport, cx, cy, depthOn));
    printf("card depth with the pre-pass %f, expected %f\n", depthOn, depthAt(20.0f));
    TEST(depthOn < 0.9999f);
    TEST(near(depthOn, depthAt(20.0f), 1e-4f));
    float depthBox = 1.0f;
    int bx = 0;
    int by = 0;
    TEST(quadrantPixel(w, 3, meshAt.x, 1.0f, bx, by));
    TEST(w.server->viewportReadDepth(w.viewport, bx, by, depthBox));
    TEST(near(depthBox, depthAt(19.0f), 1e-4f));
    TEST(near(depthOn, depthAt(20.0f), 1e-4f));

    w.server->setDepthPrePassEnabled(false);
    const Image off = render(w);
    TEST(w.stats.depthPrePassDrawCalls == 0);
    TEST(w.stats.impostorSets == 1);
    float depthOff = 1.0f;
    TEST(w.server->viewportReadDepth(w.viewport, cx, cy, depthOff));
    TEST(near(depthOff, depthOn, 1e-6f));
    const int differing = countDifferentImages(on, off, 1);
    printf("pre-pass on and off differ in %d pixels\n", differing);
    TEST(differing == 0);
    TEST(countDifferent(on, background, 8) > 200);

    arrange(w, meshAt, false, cardAt, true);
    w.server->setDepthPrePassEnabled(true);
    const Image onlyCard = render(w);
    TEST(w.stats.depthPrePassDrawCalls == 0);
    TEST(w.stats.impostorSets == 1);
    TEST(countDifferent(onlyCard, background, 8) > 100);
    w.server->setDepthPrePassEnabled(true);
}

int opaqueCount(World& w, const Pix& background, int markerX, int markerY)
{
    const Image image = render(w);
    int count = 0;
    for (int y = 0; y < kSize; ++y)
    {
        for (int x = 0; x < kSize; ++x)
        {
            if (abs(x - markerX) <= 8 && abs(y - markerY) <= 8) continue;
            if (channelDistance(image.at(x, y), background) > 8) ++count;
        }
    }
    return count;
}

void checkDissolve(World& w, const Pix& background)
{
    printf("dissolve with the distance\n");
    const Math::Vec3 origin(0.0f, 0.0f, 0.0f);
    const Math::Vec3 marker(-2.0f, 2.0f, 0.0f);
    w.server->instanceSetTransform(w.cube,
            Math::Mat4::Translation(marker) * Math::Mat4::Scale(Math::Vec3(0.2f, 0.2f, 0.2f)));
    w.server->instanceSetVisible(w.cube, true);
    arrange(w, origin, false, origin, true);
    const float distances[4] = { kFade - kRadius * 0.5f, kFade - kRadius * 0.25f,
        kFade - kRadius + 0.01f, kFade + 1.0f };
    const float lowest[4] = { 0.40f, 0.62f, 0.0f, 0.99f };
    const float highest[4] = { 0.60f, 0.88f, 0.05f, 1.01f };
    for (int mode = 0; mode < 2; ++mode)
    {
        w.server->setDepthPrePassEnabled(mode == 0);
        for (int i = 0; i < 4; ++i)
        {
            aim(w, 0.0f, distances[i]);
            int markerX = 0;
            int markerY = 0;
            TEST(project(w, marker + Math::Vec3(0.0f, 0.1f, 0.1f), markerX, markerY));
            w.server->impostorSetSwapInDistance(w.impostor, 1.0f);
            const int full = opaqueCount(w, background, markerX, markerY);
            if (mode == 0) TEST(w.stats.depthPrePassDrawCalls == 1);
            else
                TEST(w.stats.depthPrePassDrawCalls == 0);
            w.server->impostorSetSwapInDistance(w.impostor, kFade);
            const int dissolved = opaqueCount(w, background, markerX, markerY);
            const float ratio = static_cast<float>(dissolved) / static_cast<float>(full);
            printf("pre-pass %d, distance %.3f: %d of %d pixels, %.3f\n", mode == 0, distances[i],
                    dissolved, full, ratio);
            TEST(full > 100);
            TEST(ratio >= lowest[i]);
            TEST(ratio <= highest[i]);
        }
    }
    w.server->instanceSetVisible(w.cube, false);
    w.server->setDepthPrePassEnabled(true);
    w.server->impostorSetSwapInDistance(w.impostor, kFade);
}

void checkOcclusion(World& w, const Pix& background)
{
    printf("occlusion against meshes\n");
    const Math::Vec3 origin(0.0f, 0.0f, 0.0f);
    aim(w, 0.0f, 20.0f);
    int px = 0;
    int py = 0;
    TEST(quadrantPixel(w, 3, 0.0f, 0.0f, px, py));
    for (int mode = 0; mode < 2; ++mode)
    {
        w.server->setDepthPrePassEnabled(mode == 0);
        char label[64];
        arrange(w, origin, false, origin, true);
        w.server->instanceSetVisible(w.cube, false);
        const Image cardOnly = render(w);
        float cardDepth = 1.0f;
        TEST(w.server->viewportReadDepth(w.viewport, px, py, cardDepth));
        TEST(near(cardDepth, depthAt(20.0f), 1e-4f));

        w.server->instanceSetTransform(w.cube,
                Math::Mat4::Translation(Math::Vec3(0.0f, -2.0f, 6.0f)) *
                        Math::Mat4::Scale(Math::Vec3(6.0f, 6.0f, 6.0f)));
        w.server->instanceSetVisible(w.cube, true);
        arrange(w, origin, false, origin, false);
        const Image cubeOnly = render(w);
        float cubeDepth = 1.0f;
        TEST(w.server->viewportReadDepth(w.viewport, px, py, cubeDepth));
        TEST(near(cubeDepth, depthAt(11.0f), 1e-4f));

        arrange(w, origin, false, origin, true);
        const Image front = render(w);
        snprintf(label, sizeof(label), "cube in front hides the card (pre-pass %d)", mode == 0);
        TEST(pixelNear(label, front.at(px, py), cubeOnly.at(px, py), 1));
        TEST(channelDistance(front.at(px, py), cardOnly.at(px, py)) > 20);
        TEST(countDifferentImages(front, cubeOnly, 1) == 0);
        float depth = 1.0f;
        TEST(w.server->viewportReadDepth(w.viewport, px, py, depth));
        TEST(near(depth, cubeDepth, 1e-6f));

        w.server->instanceSetTransform(w.cube,
                Math::Mat4::Translation(Math::Vec3(0.0f, -2.0f, -6.0f)) *
                        Math::Mat4::Scale(Math::Vec3(6.0f, 6.0f, 6.0f)));
        arrange(w, origin, false, origin, false);
        const Image behindOnly = render(w);
        float behindDepth = 1.0f;
        TEST(w.server->viewportReadDepth(w.viewport, px, py, behindDepth));
        TEST(near(behindDepth, depthAt(23.0f), 1e-4f));
        arrange(w, origin, false, origin, true);
        const Image behind = render(w);
        snprintf(label, sizeof(label), "card in front hides the cube (pre-pass %d)", mode == 0);
        TEST(pixelNear(label, behind.at(px, py), cardOnly.at(px, py), 1));
        TEST(channelDistance(behind.at(px, py), behindOnly.at(px, py)) > 20);
        TEST(w.server->viewportReadDepth(w.viewport, px, py, depth));
        TEST(near(depth, cardDepth, 1e-6f));
        TEST(!near(depth, behindDepth, 1e-4f));
        TEST(pixelNear("around the card shows the cube", behind.at(38, py), behindOnly.at(38, py),
                1));
        TEST(channelDistance(behind.at(px, py), background) > 20);
        w.server->instanceSetVisible(w.cube, false);
    }
    w.server->setDepthPrePassEnabled(true);
}

void checkTransparent(World& w)
{
    printf("transparent surface drawn after the cards\n");
    const Math::Vec3 origin(0.0f, 0.0f, 0.0f);
    aim(w, 0.0f, 20.0f);
    arrange(w, origin, false, origin, true);
    w.server->instanceSetVisible(w.cube, false);
    w.server->instanceSetVisible(w.plate, false);
    const Image under = render(w);

    const MaterialId glass = w.materials->create("glass");
    CHECK(w.materials->setShading(glass, MaterialShading::Unshaded));
    CHECK(w.materials->setBlendMode(glass, MaterialBlend::Mix));
    CHECK(w.materials->setTransparency(glass, MaterialTransparency::AlphaBlend));
    CHECK(w.materials->setColor(glass, MaterialParam::Albedo, Math::Vec4(0.6f, 0.1f, 0.2f, 1.0f)));
    w.server->instanceSetMaterialOverride(w.plate, glass);
    w.server->instanceSetTransform(w.plate,
            Math::Mat4::Translation(Math::Vec3(0.0f, kBoxCentreY, 5.0f)) *
                    Math::Mat4::Scale(Math::Vec3(1.5f, 1.5f, 1.5f)));
    w.server->instanceSetVisible(w.plate, true);
    const Image solid = render(w);
    TEST(w.stats.impostorSets == 1);

    CHECK(w.materials->setColor(glass, MaterialParam::Albedo, Math::Vec4(0.6f, 0.1f, 0.2f, 0.5f)));
    const Image half = render(w);

    int overCard[2] = { 0, 0 };
    int overBackground[2] = { 0, 0 };
    TEST(quadrantPixel(w, 3, 0.0f, 0.0f, overCard[0], overCard[1]));
    TEST(project(w, Math::Vec3(1.4f, kBoxCentreY + 1.4f, 0.0f), overBackground[0],
            overBackground[1]));
    const Pix covered = solid.at(overCard[0], overCard[1]);
    const Pix cardUnder = under.at(overCard[0], overCard[1]);
    TEST(channelDistance(covered, cardUnder) > 20);
    TEST(pixelNear("half-transparent plate over the card", half.at(overCard[0], overCard[1]),
            blend(cardUnder, covered, 0.5f), 2));
    const Pix coveredBackground = solid.at(overBackground[0], overBackground[1]);
    const Pix backgroundUnder = under.at(overBackground[0], overBackground[1]);
    TEST(channelDistance(coveredBackground, backgroundUnder) > 20);
    TEST(pixelNear("half-transparent plate over the background",
            half.at(overBackground[0], overBackground[1]),
            blend(backgroundUnder, coveredBackground, 0.5f), 2));

    w.server->instanceSetVisible(w.plate, false);
    w.server->instanceSetMaterialOverride(w.plate, MaterialId());
    w.materials->destroy(glass);
}

void checkMsaa(World& w, const Pix& background)
{
    printf("multisampling\n");
    const Math::Vec3 origin(0.0f, 0.0f, 0.0f);
    const int errorsBefore = gErrors;
    aim(w, 0.0f, 20.0f);
    arrange(w, origin, false, origin, true);
    w.server->viewportSetMsaa3D(w.viewport, ViewportMsaa::Disabled);
    const Image plain = render(w);
    w.server->viewportSetMsaa3D(w.viewport, ViewportMsaa::Msaa4x);
    const Image msaa = render(w);
    TEST(w.server->viewportGetMsaaSamples(w.viewport) == 4);
    TEST(w.stats.impostorSets == 1);
    TEST(w.stats.impostorDrawCalls == 1);
    for (int q = 0; q < 4; ++q)
    {
        int x = 0;
        int y = 0;
        TEST(quadrantPixel(w, q, 0.0f, 0.0f, x, y));
        TEST(pixelNear(kQuadrants[q].label, msaa.at(x, y), plain.at(x, y), 4));
    }
    TEST(countDifferent(msaa, background, 8) > 200);

    aim(w, 0.0f, kFade - kRadius * 0.5f);
    w.server->impostorSetSwapInDistance(w.impostor, 1.0f);
    const Image full = render(w);
    w.server->impostorSetSwapInDistance(w.impostor, kFade);
    const Image dissolved = render(w);
    int intermediate = 0;
    int hard = 0;
    for (int y = 0; y < kSize; ++y)
    {
        for (int x = 0; x < kSize; ++x)
        {
            const Pix p = dissolved.at(x, y);
            const bool bg = channelDistance(p, background) <= 6;
            const bool solid = channelDistance(p, full.at(x, y)) <= 6;
            if (!bg && !solid) ++intermediate;
            if (bg && channelDistance(full.at(x, y), background) > 6) ++hard;
        }
    }
    printf("multisampled dissolve: %d intermediate pixels, %d fully removed\n", intermediate, hard);
    TEST(intermediate > 0);
    TEST(countDifferent(dissolved, background, 8) > 0);
    w.server->viewportSetMsaa3D(w.viewport, ViewportMsaa::Disabled);
    w.server->impostorSetSwapInDistance(w.impostor, kFade);
    TEST(gErrors == errorsBefore);
}

void checkFog(World& w)
{
    printf("fog\n");
    const Math::Vec3 origin(0.0f, 0.0f, 0.0f);
    aim(w, 0.0f, 20.0f);
    arrange(w, origin, true, origin, false);
    const Image clearMesh = render(w);
    arrange(w, origin, false, origin, true);
    const Image clearCard = render(w);

    EnvironmentSettings environment = baseEnvironment();
    environment.fogEnabled = true;
    environment.fogMode = FogMode::Exponential;
    environment.fogDensity = 0.01f;
    environment.fogLightColor = Math::Vec3(1.0f, 0.5f, 0.0f);
    w.server->scenarioSetEnvironment(w.scenario, environment);
    arrange(w, origin, true, origin, false);
    const Image fogMesh = render(w);
    arrange(w, origin, false, origin, true);
    const Image fogCard = render(w);
    TEST(w.stats.impostorSets == 1);
    for (int q = 0; q < 4; ++q)
    {
        int mx = 0;
        int my = 0;
        int cx = 0;
        int cy = 0;
        TEST(quadrantPixel(w, q, 0.0f, 1.0f, mx, my));
        TEST(quadrantPixel(w, q, 0.0f, 0.0f, cx, cy));
        char label[64];
        snprintf(label, sizeof(label), "fog %s", kQuadrants[q].label);
        TEST(pixelNear(label, fogCard.at(cx, cy), fogMesh.at(mx, my), 3));
        TEST(channelDistance(fogCard.at(cx, cy), clearCard.at(cx, cy)) > 10);
        TEST(channelDistance(fogMesh.at(mx, my), clearMesh.at(mx, my)) > 10);
    }
    w.server->scenarioSetEnvironment(w.scenario, baseEnvironment());
}

void checkStats(World& w)
{
    printf("statistics\n");
    const Math::Vec3 origin(0.0f, 0.0f, 0.0f);
    aim(w, 0.0f, 20.0f);
    arrange(w, origin, true, origin, true);
    render(w);
    TEST(w.stats.impostorSets == 1);
    TEST(w.stats.impostorDrawCalls == 1);
    arrange(w, origin, true, origin, false);
    render(w);
    TEST(w.stats.impostorSets == 0);
    TEST(w.stats.impostorDrawCalls == 0);
    arrange(w, origin, true, origin, true);
    w.server->setDepthPrePassEnabled(false);
    render(w);
    TEST(w.stats.impostorSets == 1);
    TEST(w.stats.impostorDrawCalls == 1);
    w.server->setDepthPrePassEnabled(true);
}

} // namespace

int main(int argc, char** argv)
{
    const bool vulkan = argc > 1 && strcmp(argv[1], "vulkan") == 0;

    CHECK(Device::headlessSupported());
    if (!Device::headlessSupported()) return 1;
    log_set_callback(countErrors, nullptr);

    DeviceDesc deviceDesc;
    deviceDesc.title = "impostor cards test";
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

        w.boxMesh = meshes.create("marked box");
        CHECK(meshes.addSurface(w.boxMesh, markedBox()));
        w.cubeMesh = meshes.create("cube");
        CHECK(meshes.addSurface(w.cubeMesh, colouredBox()));
        w.plateMesh = meshes.create("plate");
        CHECK(meshes.addSurface(w.plateMesh, plateData(1.0f)));

        w.scenario = server.scenarioCreate();
        server.scenarioSetEnvironment(w.scenario, baseEnvironment());
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

        w.box = server.instanceCreate(w.boxMesh, w.scenario);
        w.cube = server.instanceCreate(w.cubeMesh, w.scenario);
        w.plate = server.instanceCreate(w.plateMesh, w.scenario);
        server.instanceSetVisible(w.cube, false);
        server.instanceSetVisible(w.plate, false);

        const ct::Vector<MaterialId> none;
        w.impostor = server.impostorCreate(w.boxMesh, none, kFade);
        CHECK(w.impostor.valid());
        CHECK(server.impostorRefresh() == 1);
        server.endFrame(true);
        CHECK(server.impostorIsCaptured(w.impostor));
        w.set = server.impostorSetCreate(w.scenario);
        server.impostorSetResize(w.set, 1);

        server.setDepthPrePassEnabled(true);

        aim(w, 0.0f, 20.0f);
        arrange(w, Math::Vec3(0.0f, 0.0f, 0.0f), false, Math::Vec3(0.0f, 0.0f, 0.0f), false);
        const Image empty = render(w);
        const Pix background = empty.at(2, 2);
        printf("background %d,%d,%d\n", background.r, background.g, background.b);
        TEST(countDifferent(empty, background, 0) == 0);

        checkFront(w, background);
        checkAngles(w);
        checkLighting(w);
        checkPrepass(w, background);
        checkDissolve(w, background);
        checkOcclusion(w, background);
        checkTransparent(w);
        checkMsaa(w, background);
        checkFog(w);
        checkStats(w);

        TEST(gErrors == 0);
        printf("impostor cards: %d checks\n", checkCount);
    }

    Device::destroy(device);
    log_set_callback(nullptr, nullptr);
    if (failures == 0) printf("impostor cards: all checks passed\n");
    return failures == 0 ? 0 : 1;
}
