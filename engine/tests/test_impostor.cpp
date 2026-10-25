#include "Check.h"
#include "MarkedBox.h"

#include "prisma3d/Device.h"
#include "prisma3d/render/MaterialStore.h"
#include "prisma3d/render/MeshStore.h"
#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/render/TextureStore.h"

#include <math.h>
#include <string.h>

using namespace prisma3d;

namespace
{

const float kPi = 3.14159265358979f;

struct Rgba
{
    int r;
    int g;
    int b;
    int a;
};

Rgba texel(const ct::Vector<std::uint8_t>& image, int x, int y)
{
    const std::size_t at = (static_cast<std::size_t>(y) * kImpostorTextureDim + x) * 4;
    return Rgba{ image[at], image[at + 1], image[at + 2], image[at + 3] };
}

bool same(const Rgba& a, const Rgba& b, int tolerance)
{
    return abs(a.r - b.r) <= tolerance && abs(a.g - b.g) <= tolerance &&
           abs(a.b - b.b) <= tolerance && abs(a.a - b.a) <= tolerance;
}

bool expect(const char* label, const Rgba& got, const Rgba& want, int tolerance)
{
    if (same(got, want, tolerance)) return true;
    printf("%s: got %d,%d,%d,%d expected %d,%d,%d,%d\n", label, got.r, got.g, got.b, got.a, want.r,
            want.g, want.b, want.a);
    return false;
}

struct Face
{
    Rgba colour;
    Math::Vec3 normal;
};

Face dominantFace(int angle)
{
    const float theta = 2.0f * kPi * static_cast<float>(angle) / 36.0f;
    const float x = -sinf(theta);
    const float z = cosf(theta);
    if (fabsf(z) > fabsf(x))
    {
        if (z > 0) return Face{ Rgba{ 255, 0, 0, 255 }, Math::Vec3(0, 0, 1) };
        return Face{ Rgba{ 0, 0, 255, 255 }, Math::Vec3(0, 0, -1) };
    }
    if (x > 0) return Face{ Rgba{ 0, 255, 0, 255 }, Math::Vec3(1, 0, 0) };
    return Face{ Rgba{ 255, 255, 0, 255 }, Math::Vec3(-1, 0, 0) };
}

int encode(float value)
{
    return static_cast<int>(value * 127.5f + 127.5f + 0.5f);
}

int checkCount = 0;

#define TEST(cond)                                                                                 \
    do                                                                                             \
    {                                                                                              \
        ++checkCount;                                                                              \
        CHECK(cond);                                                                               \
    } while (0)

const float kBoxHalf = 0.5f;
const float kFade = 50.0f;
const std::uint32_t kRed = 0xFF3300FFu;
const std::uint32_t kGreen = 0xFF00FF00u;
const std::uint32_t kBlue = 0xFFFF0000u;

bool near(float a, float b, float eps)
{
    return fabsf(a - b) <= eps;
}

Math::Vec3 crossOf(const Math::Vec3& a, const Math::Vec3& b)
{
    return Math::Vec3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

struct Expected
{
    std::uint32_t slice = 0;
    std::uint32_t dither = 0;
    Math::Vec3 positions[4];
    Math::Vec3 normal;
};

Expected expectedCard(const Math::Vec3& center, float radius, const Math::Vec3& eye,
        std::uint32_t sliceOffset, float fade)
{
    Expected result;
    const Math::Vec3 up(0.0f, 1.0f, 0.0f);
    Math::Vec3 face = eye - center;
    const float dist = face.Length();
    face.y = 0.0f;
    face = face.Normalized();
    const Math::Vec3 right = crossOf(face, up).Normalized();

    float angle = acosf(face.x * 0.0f + face.z * 1.0f) / 3.14159265359f;
    if (crossOf(face, Math::Vec3(0.0f, 0.0f, 1.0f)).y < 0.0f) angle = 2.0f - angle;
    angle *= 0.5f;
    angle = angle - 0.0001f;
    angle = angle < 0.0f ? 0.0f : (angle > 1.0f ? 1.0f : angle);
    result.slice = sliceOffset + static_cast<std::uint32_t>(angle * 36.0f) * 3u;

    const float fadeLeft = fade - dist;
    const float dither = (fadeLeft > 0.0f ? fadeLeft : 0.0f) / radius;
    result.dither = static_cast<std::uint32_t>(dither * 255.0f) & 0xFFu;

    const float corners[4][2] = { { -1.0f, -1.0f }, { 1.0f, -1.0f }, { -1.0f, 1.0f },
        { 1.0f, 1.0f } };
    for (int v = 0; v < 4; ++v)
        result.positions[v] = center + (right * corners[v][0] + up * corners[v][1]) * radius;
    result.normal = face;
    return result;
}

struct Rig
{
    ScenarioId scenario;
    CameraId camera;
    ViewportId viewport;
    ImpostorSetId set;
};

Math::Mat4 lookFrom(const Math::Vec3& eye, const Math::Vec3& target)
{
    const float yaw = atan2f(-(target.x - eye.x), -(target.z - eye.z));
    return Math::Mat4::Translation(eye) * Math::Mat4::RotationY(yaw);
}

bool draw(RenderingServer& server, const Rig& rig, const Math::Vec3& eye, const Math::Vec3& target,
        std::uint32_t& indexCount, ct::Vector<ImpostorCardReadback>& cards)
{
    server.cameraSetTransform(rig.camera, lookFrom(eye, target));
    const bool rendered = server.viewportRender(rig.viewport);
    server.endFrame(true);
    return rendered && server.impostorSetReadCards(rig.set, indexCount, cards);
}

std::uint32_t cardsFor(RenderingServer& server, const Rig& rig, const Math::Vec3& eye,
        const Math::Vec3& target)
{
    std::uint32_t indexCount = 0;
    ct::Vector<ImpostorCardReadback> cards;
    TEST(draw(server, rig, eye, target, indexCount, cards));
    TEST(indexCount % 6 == 0);
    TEST(cards.size() == indexCount / 6);
    return indexCount / 6;
}

bool verifyCard(const char* label, const ImpostorCardReadback& card, const Expected& want,
        std::uint32_t color, int ditherTolerance)
{
    const int failuresBefore = failures;
    ++checkCount;
    if (card.slice != want.slice)
    {
        printf("%s: slice %u expected %u\n", label, card.slice, want.slice);
        ++failures;
    }
    ++checkCount;
    const int ditherByte = static_cast<int>(card.dither * 255.0f + 0.5f);
    if (abs(ditherByte - static_cast<int>(want.dither)) > ditherTolerance)
    {
        printf("%s: dither %d expected %u\n", label, ditherByte, want.dither);
        ++failures;
    }
    ++checkCount;
    if (card.color != color)
    {
        printf("%s: colour %08x expected %08x\n", label, card.color, color);
        ++failures;
    }
    ++checkCount;
    bool positionsOk = true;
    for (int v = 0; v < 4; ++v)
    {
        positionsOk = positionsOk && near(card.positions[v][0], want.positions[v].x, 1e-3f) &&
                      near(card.positions[v][1], want.positions[v].y, 1e-3f) &&
                      near(card.positions[v][2], want.positions[v].z, 1e-3f);
    }
    if (!positionsOk)
    {
        printf("%s: positions differ\n", label);
        for (int v = 0; v < 4; ++v)
            printf("  v%d got %.4f %.4f %.4f want %.4f %.4f %.4f\n", v, card.positions[v][0],
                    card.positions[v][1], card.positions[v][2], want.positions[v].x,
                    want.positions[v].y, want.positions[v].z);
        ++failures;
    }
    ++checkCount;
    if (!near(card.normal[0], want.normal.x, 1e-4f) ||
            !near(card.normal[1], want.normal.y, 1e-4f) ||
            !near(card.normal[2], want.normal.z, 1e-4f))
    {
        printf("%s: normal %.4f %.4f %.4f expected %.4f %.4f %.4f\n", label, card.normal[0],
                card.normal[1], card.normal[2], want.normal.x, want.normal.y, want.normal.z);
        ++failures;
    }
    return failures == failuresBefore;
}

const ImpostorCardReadback* findByColor(const ct::Vector<ImpostorCardReadback>& cards,
        std::uint32_t color)
{
    for (std::size_t i = 0; i < cards.size(); ++i)
        if (cards[i].color == color) return &cards[i];
    return nullptr;
}

Math::Vec3 boxCentre(const Math::Vec3& translation, float scale)
{
    return translation + Math::Vec3(0.0f, kBoxHalf * scale, 0.0f);
}

float boxRadius(float scale, float yaw)
{
    const float extent = (fabsf(cosf(yaw)) + fabsf(sinf(yaw))) * kBoxHalf * scale;
    const float height = kBoxHalf * scale;
    return sqrtf(extent * extent + height * height + extent * extent);
}

Math::Mat4 placement(const Math::Vec3& translation, float yaw, float scale)
{
    return Math::Mat4::Translation(translation) * Math::Mat4::RotationY(yaw) *
           Math::Mat4::Scale(Math::Vec3(scale, scale, scale));
}

void checkPrepare(RenderingServer& server, MeshId box, const ImpostorId impostor)
{
    const std::uint32_t sliceOffset =
            impostorSliceOffset(static_cast<std::uint32_t>(server.impostorGetSlot(impostor)));

    Rig rig;
    rig.scenario = server.scenarioCreate();
    rig.camera = server.cameraCreate();
    server.cameraSetPerspective(rig.camera, 60.0f, 0.1f, 500.0f);
    rig.viewport = server.viewportCreate();
    server.viewportSetSize(rig.viewport, 64, 64);
    server.viewportSetScenario(rig.viewport, rig.scenario);
    server.viewportAttachCamera(rig.viewport, rig.camera);
    server.viewportSetActive(rig.viewport, true);
    rig.set = server.impostorSetCreate(rig.scenario);
    TEST(rig.set.valid());

    const Math::Vec3 origin(0.0f, 0.0f, 0.0f);
    const Math::Vec3 centre = boxCentre(origin, 1.0f);
    const float radius = boxRadius(1.0f, 0.0f);
    TEST(near(radius, sqrtf(0.75f), 1e-6f));

    std::uint32_t indexCount = 0;
    ct::Vector<ImpostorCardReadback> cards;

    TEST(server.impostorSetGetCount(rig.set) == 0);
    TEST(cardsFor(server, rig, Math::Vec3(0.0f, 0.5f, 60.0f), centre) == 0);

    server.impostorSetResize(rig.set, 1);
    TEST(server.impostorSetGetCount(rig.set) == 1);
    server.impostorSetInstance(rig.set, 0, impostor, placement(origin, 0.0f, 1.0f),
            Math::Vec4(1.0f, 0.0f, 0.2f, 1.0f));

    ImpostorInstanceInfo info;
    TEST(server.impostorSetGetInstance(rig.set, 0, info));
    TEST(near(info.center[0], centre.x, 1e-5f) && near(info.center[1], centre.y, 1e-5f) &&
            near(info.center[2], centre.z, 1e-5f));
    TEST(near(info.radius, radius, 1e-5f));
    TEST(info.fadeDistance == kFade);
    TEST(info.sliceOffset == static_cast<int>(sliceOffset));

    {
        const Math::Vec3 eye(0.0f, 0.5f, 60.0f);
        TEST(draw(server, rig, eye, centre, indexCount, cards));
        TEST(indexCount == 6);
        TEST(cards.size() == 1);
        if (cards.size() == 1)
            verifyCard("single", cards[0], expectedCard(centre, radius, eye, sliceOffset, kFade),
                    kRed, 0);
    }

    {
        struct Cardinal
        {
            const char* label;
            Math::Vec3 eye;
            std::uint32_t angle;
        };
        const Cardinal cardinals[4] = {
            { "camera +Z", Math::Vec3(0.0f, 0.5f, 60.0f), 0 },
            { "camera -X", Math::Vec3(-60.0f, 0.5f, 0.0f), 8 },
            { "camera +X", Math::Vec3(60.0f, 0.5f, 0.0f), 26 },
            { "camera -Z", Math::Vec3(0.0f, 0.5f, -60.0f), 17 },
        };
        for (const Cardinal& c: cardinals)
        {
            TEST(draw(server, rig, c.eye, centre, indexCount, cards));
            TEST(cards.size() == 1);
            if (cards.size() != 1) continue;
            TEST(cards[0].slice == sliceOffset + c.angle * 3);
            verifyCard(c.label, cards[0], expectedCard(centre, radius, c.eye, sliceOffset, kFade),
                    kRed, 0);
        }
    }

    {
        int sweepBad = 0;
        for (int degrees = 0; degrees < 360; degrees += 5)
        {
            const float phi = static_cast<float>(degrees) * 3.14159265359f / 180.0f;
            const Math::Vec3 eye = centre + Math::Vec3(sinf(phi), 0.0f, cosf(phi)) * 60.0f;
            TEST(draw(server, rig, eye, centre, indexCount, cards));
            TEST(cards.size() == 1);
            if (cards.size() != 1)
            {
                ++sweepBad;
                continue;
            }
            char label[48];
            snprintf(label, sizeof(label), "azimuth %d", degrees);
            if (!verifyCard(label, cards[0], expectedCard(centre, radius, eye, sliceOffset, kFade),
                        kRed, 0))
                ++sweepBad;
        }
        TEST(sweepBad == 0);
    }

    {
        const Math::Vec3 eye(0.0f, 20.0f, 60.0f);
        TEST(draw(server, rig, eye, centre, indexCount, cards));
        TEST(cards.size() == 1);
        if (cards.size() == 1)
        {
            TEST(near(cards[0].normal[1], 0.0f, 1e-6f));
            verifyCard("raised camera", cards[0],
                    expectedCard(centre, radius, eye, sliceOffset, kFade), kRed, 0);
        }
    }

    {
        const float distances[] = { kFade, kFade - radius * 0.5f, kFade - radius * 0.25f,
            kFade - radius * 0.75f, kFade - radius + 0.01f, kFade + 1.0f, 500.0f };
        const std::uint32_t wantDither[] = { 0, 127, 63, 191, 252, 0, 0 };
        const int tolerance[] = { 0, 0, 1, 1, 1, 0, 0 };
        for (int i = 0; i < 7; ++i)
        {
            const Math::Vec3 eye = centre + Math::Vec3(0.0f, 0.0f, distances[i]);
            TEST(draw(server, rig, eye, centre, indexCount, cards));
            TEST(cards.size() == 1);
            if (cards.size() != 1) continue;
            char label[48];
            snprintf(label, sizeof(label), "fade %d", i);
            Expected want = expectedCard(centre, radius, eye, sliceOffset, kFade);
            TEST(abs(static_cast<int>(want.dither) - static_cast<int>(wantDither[i])) <=
                    tolerance[i]);
            verifyCard(label, cards[0], want, kRed, tolerance[i]);
        }
        const Math::Vec3 culled = centre + Math::Vec3(0.0f, 0.0f, kFade - radius - 1.0f);
        TEST(cardsFor(server, rig, culled, centre) == 0);
        const Math::Vec3 inside = centre + Math::Vec3(0.0f, 0.0f, 10.0f);
        TEST(cardsFor(server, rig, inside, centre) == 0);
    }

    {
        const Math::Vec3 eye(0.0f, 0.5f, 60.0f);
        const Math::Vec3 look = eye + Math::Vec3(0.0f, 0.0f, -1.0f);
        struct Spot
        {
            const char* label;
            Math::Vec3 position;
            std::uint32_t cards;
        };
        const Spot spots[] = {
            { "in front", Math::Vec3(0.0f, 0.0f, 0.0f), 1 },
            { "inside left", Math::Vec3(-20.0f, 0.0f, 0.0f), 1 },
            { "inside right", Math::Vec3(20.0f, 0.0f, 0.0f), 1 },
            { "outside left", Math::Vec3(-200.0f, 0.0f, 0.0f), 0 },
            { "outside right", Math::Vec3(200.0f, 0.0f, 0.0f), 0 },
            { "behind", Math::Vec3(0.0f, 0.0f, 120.0f), 0 },
            { "beyond far", Math::Vec3(0.0f, 0.0f, -600.0f), 0 },
        };
        for (const Spot& spot: spots)
        {
            server.impostorSetInstance(rig.set, 0, impostor, placement(spot.position, 0.0f, 1.0f),
                    Math::Vec4(1.0f, 0.0f, 0.2f, 1.0f));
            const std::uint32_t got = cardsFor(server, rig, eye, look);
            if (got != spot.cards)
                printf("frustum %s: %u cards expected %u\n", spot.label, got, spot.cards);
            TEST(got == spot.cards);
        }
    }

    {
        const Math::Vec3 eye(0.0f, 0.5f, 60.0f);
        const Math::Vec3 look = eye + Math::Vec3(0.0f, 0.0f, -1.0f);
        const Math::Vec3 positions[5] = { Math::Vec3(0.0f, 0.0f, 0.0f),
            Math::Vec3(10.0f, 0.0f, -5.0f), Math::Vec3(-15.0f, 0.0f, 5.0f),
            Math::Vec3(200.0f, 0.0f, 0.0f), Math::Vec3(0.0f, 0.0f, 120.0f) };
        const std::uint32_t colours[5] = { kRed, kGreen, kBlue, 0xFF112233u, 0xFF445566u };
        const Math::Vec4 inputs[5] = { Math::Vec4(1.0f, 0.0f, 0.2f, 1.0f),
            Math::Vec4(0.0f, 1.0f, 0.0f, 1.0f), Math::Vec4(0.0f, 0.0f, 1.0f, 1.0f),
            Math::Vec4(0x33 / 255.0f, 0x22 / 255.0f, 0x11 / 255.0f, 1.0f),
            Math::Vec4(0x66 / 255.0f, 0x55 / 255.0f, 0x44 / 255.0f, 1.0f) };
        server.impostorSetResize(rig.set, 5);
        TEST(server.impostorSetGetCount(rig.set) == 5);
        for (std::uint32_t i = 0; i < 5; ++i)
            server.impostorSetInstance(rig.set, i, impostor, placement(positions[i], 0.0f, 1.0f),
                    inputs[i]);
        TEST(draw(server, rig, eye, look, indexCount, cards));
        TEST(indexCount == 18);
        TEST(cards.size() == 3);
        for (std::uint32_t i = 0; i < 5; ++i)
        {
            const ImpostorCardReadback* card = findByColor(cards, colours[i]);
            if (i < 3)
            {
                TEST(card != nullptr);
                if (!card) continue;
                const Math::Vec3 c = boxCentre(positions[i], 1.0f);
                char label[48];
                snprintf(label, sizeof(label), "multi %u", i);
                verifyCard(label, *card, expectedCard(c, radius, eye, sliceOffset, kFade),
                        colours[i], 0);
            }
            else
                TEST(card == nullptr);
        }

        const Math::Vec3 away(0.0f, 0.5f, 60.0f);
        TEST(cardsFor(server, rig, away, away + Math::Vec3(-1.0f, 0.0f, 0.0f)) == 0);
        TEST(draw(server, rig, eye, look, indexCount, cards));
        TEST(cards.size() == 3);

        server.impostorSetEnabled(rig.set, false);
        TEST(cardsFor(server, rig, eye, look) == 0);
        server.impostorSetEnabled(rig.set, true);
        TEST(cardsFor(server, rig, eye, look) == 3);

        server.impostorSetResize(rig.set, 0);
        TEST(server.impostorSetGetCount(rig.set) == 0);
        TEST(cardsFor(server, rig, eye, look) == 0);
    }

    {
        const Math::Vec3 eye(0.0f, 0.5f, 60.0f);
        const Math::Vec3 look = eye + Math::Vec3(0.0f, 0.0f, -1.0f);
        const ImpostorId stale = server.impostorCreate(box, ct::Vector<MaterialId>(), 50.0f);
        TEST(stale.valid());
        server.impostorFree(stale);
        server.impostorSetResize(rig.set, 3);
        server.impostorSetInstance(rig.set, 0, stale, placement(origin, 0.0f, 1.0f),
                Math::Vec4(1.0f, 0.0f, 0.2f, 1.0f));
        server.impostorSetInstance(rig.set, 1, impostor,
                placement(Math::Vec3(5.0f, 0.0f, 0.0f), 0.0f, 1.0f),
                Math::Vec4(0.0f, 1.0f, 0.0f, 1.0f));
        server.impostorSetInstance(rig.set, 2, ImpostorId(),
                placement(Math::Vec3(-5.0f, 0.0f, 0.0f), 0.0f, 1.0f),
                Math::Vec4(0.0f, 0.0f, 1.0f, 1.0f));
        TEST(!server.impostorSetGetInstance(rig.set, 0, info));
        TEST(draw(server, rig, eye, look, indexCount, cards));
        TEST(cards.size() == 1);
        if (cards.size() == 1) TEST(cards[0].color == kGreen);
    }

    {
        const Math::Vec3 eye(0.0f, 0.5f, 60.0f);
        const Math::Vec3 look = eye + Math::Vec3(0.0f, 0.0f, -1.0f);
        struct Shape
        {
            const char* label;
            Math::Vec3 translation;
            float yaw;
            float scale;
        };
        const Shape shapes[] = {
            { "scale 2", Math::Vec3(0.0f, 0.0f, 0.0f), 0.0f, 2.0f },
            { "scale 0.5", Math::Vec3(3.0f, 1.0f, -4.0f), 0.0f, 0.5f },
            { "yaw 45", Math::Vec3(-6.0f, 0.0f, 2.0f), 0.785398163f, 1.0f },
            { "yaw 30 scale 3", Math::Vec3(4.0f, 2.0f, -3.0f), 0.523598776f, 3.0f },
        };
        server.impostorSetResize(rig.set, 1);
        for (const Shape& shape: shapes)
        {
            server.impostorSetInstance(rig.set, 0, impostor,
                    placement(shape.translation, shape.yaw, shape.scale),
                    Math::Vec4(1.0f, 0.0f, 0.2f, 1.0f));
            const Math::Vec3 c = boxCentre(shape.translation, shape.scale);
            const float r = boxRadius(shape.scale, shape.yaw);
            TEST(server.impostorSetGetInstance(rig.set, 0, info));
            TEST(near(info.radius, r, 1e-4f));
            TEST(near(info.center[1], c.y, 1e-4f));
            TEST(draw(server, rig, eye, look, indexCount, cards));
            TEST(cards.size() == 1);
            if (cards.size() == 1)
                verifyCard(shape.label, cards[0], expectedCard(c, r, eye, sliceOffset, kFade), kRed,
                        0);
        }
        TEST(near(boxRadius(2.0f, 0.0f), 2.0f * boxRadius(1.0f, 0.0f), 1e-5f));
    }

    server.impostorSetFree(rig.set);
    TEST(!server.impostorSetReadCards(rig.set, indexCount, cards));
    server.viewportFree(rig.viewport);
    server.cameraFree(rig.camera);
    server.scenarioFree(rig.scenario);

    printf("prepare: %d checks\n", checkCount);
}

} // namespace

int main(int argc, char** argv)
{
    const bool vulkan = argc > 1 && strcmp(argv[1], "vulkan") == 0;

    CHECK(Device::headlessSupported());
    if (!Device::headlessSupported()) return 1;

    DeviceDesc deviceDesc;
    deviceDesc.title = "impostor test";
    deviceDesc.driver = vulkan ? prisma::DriverType::Vulkan : prisma::DriverType::OpenGL;
    deviceDesc.width = 64;
    deviceDesc.height = 64;
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

        const MeshId box = meshes.create("marked box");
        CHECK(meshes.addSurface(box, markedBox()));
        const ct::Vector<MaterialId> none;

        const ImpostorId impostor = server.impostorCreate(box, none, 50.0f);
        CHECK(impostor.valid());
        CHECK(server.impostorGetSlot(impostor) == 0);
        CHECK(server.impostorGetSwapInDistance(impostor) == 50.0f);
        CHECK(!server.impostorIsCaptured(impostor));
        CHECK(server.impostorRefresh() == 1);
        server.endFrame(true);
        CHECK(server.impostorIsCaptured(impostor));
        printf("capture samples %u, texture memory %llu bytes\n", server.impostorCaptureSamples(),
                static_cast<unsigned long long>(server.impostorTextureBytes()));

        int albedoBad = 0;
        int normalBad = 0;
        int surfaceBad = 0;
        int cornerBad = 0;
        ct::Vector<std::uint8_t> albedo;
        ct::Vector<std::uint8_t> normal;
        ct::Vector<std::uint8_t> surface;
        for (int angle = 0; angle < static_cast<int>(kImpostorCaptureAngles); ++angle)
        {
            CHECK(server.impostorReadSlice(impostor, angle, ImpostorProperty::Albedo, albedo));
            CHECK(server.impostorReadSlice(impostor, angle, ImpostorProperty::Normal, normal));
            CHECK(server.impostorReadSlice(impostor, angle, ImpostorProperty::Surface, surface));
            char label[64];
            const Face face = dominantFace(angle);
            snprintf(label, sizeof(label), "albedo %d", angle);
            if (face.normal.z > 0.5f)
            {
                if (texel(albedo, 64, 64).a != 255) ++albedoBad;
            }
            else if (!expect(label, texel(albedo, 64, 64), face.colour, 2))
                ++albedoBad;
            const Rgba normalWant = Rgba{ encode(face.normal.x), encode(face.normal.y),
                encode(face.normal.z), 255 };
            snprintf(label, sizeof(label), "normal %d", angle);
            if (!expect(label, texel(normal, 64, 64), normalWant, 3)) ++normalBad;
            snprintf(label, sizeof(label), "surface %d", angle);
            if (!expect(label, texel(surface, 64, 64), Rgba{ 255, 255, 0, 128 }, 2)) ++surfaceBad;
            const int corners[4][2] = { { 1, 1 }, { 126, 1 }, { 1, 126 }, { 126, 126 } };
            for (int c = 0; c < 4; ++c)
            {
                snprintf(label, sizeof(label), "corner %d.%d", angle, c);
                const Rgba empty = { 0, 0, 0, 0 };
                if (!expect(label, texel(albedo, corners[c][0], corners[c][1]), empty, 0))
                    ++cornerBad;
            }
        }
        CHECK(albedoBad == 0);
        CHECK(normalBad == 0);
        CHECK(surfaceBad == 0);
        CHECK(cornerBad == 0);

        CHECK(server.impostorReadSlice(impostor, 0, ImpostorProperty::Albedo, albedo));
        CHECK(expect("front lower left", texel(albedo, 44, 44), Rgba{ 255, 0, 0, 255 }, 2));
        CHECK(expect("front lower right", texel(albedo, 84, 44), Rgba{ 255, 0, 255, 255 }, 2));
        CHECK(expect("front upper left", texel(albedo, 44, 84), Rgba{ 0, 255, 255, 255 }, 2));
        CHECK(expect("front upper right", texel(albedo, 84, 84), Rgba{ 255, 255, 128, 255 }, 2));

        CHECK(server.impostorRefresh() == 0);
        server.impostorSetDirty(impostor);
        CHECK(server.impostorRefresh() == 1);
        server.endFrame(true);
        CHECK(server.impostorIsCaptured(impostor));

        checkPrepare(server, box, impostor);

        const ImpostorId second = server.impostorCreate(box, none, 80.0f);
        CHECK(server.impostorGetSlot(second) == 1);
        ImpostorId extra[kMaxImpostorCount];
        std::uint32_t made = 0;
        for (std::uint32_t i = 0; i < kMaxImpostorCount; ++i)
        {
            extra[i] = server.impostorCreate(box, none, 10.0f);
            if (extra[i].valid()) ++made;
        }
        CHECK(made == kMaxImpostorCount - 2);
        server.impostorFree(second);
        const ImpostorId reused = server.impostorCreate(box, none, 10.0f);
        CHECK(reused.valid() && server.impostorGetSlot(reused) == 1);
        CHECK(server.impostorRefresh() >= 1);
        server.endFrame(true);
        CHECK(server.impostorReadSlice(reused, 18, ImpostorProperty::Albedo, albedo));
        CHECK(expect("reused slot back", texel(albedo, 64, 64), Rgba{ 0, 0, 255, 255 }, 2));
        CHECK(server.impostorReadSlice(impostor, 18, ImpostorProperty::Albedo, albedo));
        CHECK(expect("first slot kept", texel(albedo, 64, 64), Rgba{ 0, 0, 255, 255 }, 2));
    }

    Device::destroy(device);
    if (failures == 0) printf("impostor: all checks passed\n");
    return failures == 0 ? 0 : 1;
}
