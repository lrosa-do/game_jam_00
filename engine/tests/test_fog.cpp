#include "Check.h"
#include "ColouredBox.h"
#include "LitTest.h"

#include "prisma3d/Device.h"
#include "prisma3d/core/Pixmap.h"
#include "prisma3d/main/Main.h"
#include "prisma3d/render/MaterialStore.h"
#include "prisma3d/render/MeshStore.h"
#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/render/TextureStore.h"
#include "prisma3d/scene/Camera3D.h"
#include "prisma3d/scene/Forest3D.h"
#include "prisma3d/scene/Grass3D.h"
#include "prisma3d/scene/Light3D.h"
#include "prisma3d/scene/MultiMeshInstance3D.h"
#include "prisma3d/scene/PrimitiveMeshes.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/Terrain3D.h"
#include "prisma3d/scene/Viewport.h"
#include "prisma3d/scene/VisualInstance3D.h"
#include "prisma3d/scene/WorldEnvironment.h"

#include <ct/vector.hpp>

#include <math.h>
#include <stdlib.h>
#include <string.h>

using namespace prisma3d;

namespace
{

const int kWidth = 64;
const int kHeight = 48;
const int kTolerance = 2;
const float kPi = 3.14159265358979f;
const float kAspect = 4.0f / 3.0f;
const float kClearSrgb[3] = { 0.1f, 0.2f, 0.3f };

float toLinear(float value)
{
    return value <= 0.04045f ? value / 12.92f : powf((value + 0.055f) / 1.055f, 2.4f);
}

float toSrgb(float value)
{
    return value <= 0.0031308f ? value * 12.92f : 1.055f * powf(value, 1.0f / 2.4f) - 0.055f;
}

int toByte(float linear)
{
    const float clamped = linear < 0.0f ? 0.0f : (linear > 1.0f ? 1.0f : linear);
    return static_cast<int>(toSrgb(clamped) * 255.0f + 0.5f);
}

float smoothstepOf(float edge0, float edge1, float x)
{
    float t = (x - edge0) / (edge1 - edge0);
    t = t < 0.0f ? 0.0f : (t > 1.0f ? 1.0f : t);
    return t * t * (3.0f - 2.0f * t);
}

struct Lin
{
    float r;
    float g;
    float b;
};

Lin fromSrgb(float r, float g, float b)
{
    return Lin{ toLinear(r), toLinear(g), toLinear(b) };
}

Lin mixLin(const Lin& a, const Lin& b, float t)
{
    return Lin{ a.r * (1.0f - t) + b.r * t, a.g * (1.0f - t) + b.g * t,
        a.b * (1.0f - t) + b.b * t };
}

Lin scaleLin(const Lin& a, float s)
{
    return Lin{ a.r * s, a.g * s, a.b * s };
}

Lin addLin(const Lin& a, const Lin& b)
{
    return Lin{ a.r + b.r, a.g + b.g, a.b + b.b };
}

const Lin kWhiteLin = { 1.0f, 1.0f, 1.0f };

struct Picture
{
    unsigned char pixels[kWidth * kHeight * 4];

    const unsigned char* at(int x, int y) const
    {
        return pixels + (y * kWidth + x) * 4;
    }
};

bool pixelIs(const Picture& picture, const char* label, int x, int y, const Lin& expected)
{
    const unsigned char* p = picture.at(x, y);
    const int er = toByte(expected.r);
    const int eg = toByte(expected.g);
    const int eb = toByte(expected.b);
    const bool same = abs(p[0] - er) <= kTolerance && abs(p[1] - eg) <= kTolerance &&
                      abs(p[2] - eb) <= kTolerance;
    printf("  %-44s (%2d,%2d) is %3d %3d %3d, expected %3d %3d %3d %s\n", label, x, y, p[0], p[1],
            p[2], er, eg, eb, same ? "" : "MISMATCH");
    return same;
}

struct Ray
{
    float x;
    float y;
    float norm;
};

Ray rayAt(int px, int py)
{
    const float tangent = tanf(30.0f * kPi / 180.0f);
    Ray ray;
    ray.x = ((static_cast<float>(px) + 0.5f) / kWidth * 2.0f - 1.0f) * kAspect * tangent;
    ray.y = (1.0f - (static_cast<float>(py) + 0.5f) / kHeight * 2.0f) * tangent;
    ray.norm = sqrtf(1.0f + ray.x * ray.x + ray.y * ray.y);
    return ray;
}

struct Frame
{
    explicit Frame(SceneTree& tree)
    {
        tree.process(1.0 / 60.0);
        Main::rendering()->renderViewports();
    }

    ~Frame()
    {
        Main::rendering()->endFrame(true);
    }
};

bool capture(SceneTree& tree, Picture& picture, int primes = 3)
{
    for (int i = 0; i < primes; ++i)
    {
        Frame prime(tree);
    }
    Frame frame(tree);
    prisma::Rect rect;
    rect.x = 0;
    rect.y = 0;
    rect.width = kWidth;
    rect.height = kHeight;
    return Main::device()->driver()->readPixels(prisma::RenderTarget(), rect, picture.pixels);
}

struct World
{
    SceneTree* tree = nullptr;
    Node* root = nullptr;
    Window* window = nullptr;
    Camera3D* camera = nullptr;
    WorldEnvironment* environment = nullptr;
    MeshInstance3D* wall = nullptr;
    EnvironmentSettings settings;
    MaterialId white;
    MaterialId matte;
    Lin fogColour = { 0.0f, 0.0f, 0.0f };

    void apply()
    {
        environment->setEnvironment(settings);
    }

    void placeWall(float distance)
    {
        wall->setTransform(Math::Mat4::Translation(Math::Vec3(0.0f, 0.0f, -distance)) *
                           Math::Mat4::RotationX(kPi * 0.5f));
    }

    void fogOff()
    {
        settings.fogEnabled = false;
    }

    void fogExponential(float density)
    {
        settings.fogEnabled = true;
        settings.fogMode = FogMode::Exponential;
        settings.fogDensity = density;
        settings.fogHeightDensity = 0.0f;
        settings.fogSunScatter = 0.0f;
        settings.fogAerialPerspective = 0.0f;
        settings.fogSkyAffect = 1.0f;
        settings.fogLightEnergy = 1.0f;
    }

    void setFogColour(float r, float g, float b)
    {
        settings.fogLightColor = Math::Vec3(r, g, b);
        fogColour = fromSrgb(r, g, b);
    }
};

EnvironmentSettings plainEnvironment()
{
    EnvironmentSettings settings = whiteAmbient();
    settings.fogLightColor = Math::Vec3(1.0f, 0.5f, 0.0f);
    return settings;
}

void useSky(World& world, float grey)
{
    ProceduralSkySettings& sky = world.settings.sky;
    sky.skyTopColor = Math::Vec3(grey, grey, grey);
    sky.skyHorizonColor = Math::Vec3(grey, grey, grey);
    sky.groundBottomColor = Math::Vec3(grey, grey, grey);
    sky.groundHorizonColor = Math::Vec3(grey, grey, grey);
    sky.skyEnergy = 1.0f;
    sky.groundEnergy = 1.0f;
    sky.energy = 1.0f;
    sky.debanding = false;
    world.settings.background = BackgroundMode::Sky;
}

void testUnshadedExponential(World& world)
{
    printf("exponential fog on a white unshaded wall\n");
    world.settings = plainEnvironment();
    world.fogOff();
    world.setFogColour(1.0f, 0.5f, 0.0f);
    world.apply();
    world.wall->setMaterialOverride(world.white);
    world.placeWall(10.0f);

    Picture picture;
    CHECK(capture(*world.tree, picture));
    CHECK(pixelIs(picture, "fog off", 32, 24, kWhiteLin));

    world.fogExponential(0.05f);
    world.settings.fogLightColor = Math::Vec3(1.0f, 0.5f, 0.0f);
    world.apply();
    const float distances[2] = { 10.0f, 30.0f };
    for (int i = 0; i < 2; ++i)
    {
        world.placeWall(distances[i]);
        CHECK(capture(*world.tree, picture));
        const int pixels[2][2] = { { 32, 24 }, { 8, 10 } };
        for (int p = 0; p < 2; ++p)
        {
            const Ray ray = rayAt(pixels[p][0], pixels[p][1]);
            const float f = 1.0f - expf(-distances[i] * ray.norm * 0.05f);
            char label[64];
            snprintf(label, sizeof(label), "density 0.05, distance %.0f, f %.4f", distances[i], f);
            CHECK(pixelIs(picture, label, pixels[p][0], pixels[p][1],
                    mixLin(kWhiteLin, world.fogColour, f)));
        }
    }

    world.settings.fogEnabled = false;
    world.settings.fogDensity = 5.0f;
    world.apply();
    CHECK(capture(*world.tree, picture));
    CHECK(pixelIs(picture, "fog disabled with a high density", 32, 24, kWhiteLin));
}

void testDepthMode(World& world)
{
    printf("depth fog\n");
    world.settings = plainEnvironment();
    world.setFogColour(1.0f, 0.5f, 0.0f);
    world.fogExponential(0.8f);
    world.settings.fogMode = FogMode::Depth;
    world.settings.fogDepthBegin = 10.0f;
    world.settings.fogDepthEnd = 50.0f;
    world.settings.fogDepthCurve = 2.0f;
    world.settings.fogLightColor = Math::Vec3(1.0f, 0.5f, 0.0f);
    world.apply();
    world.wall->setMaterialOverride(world.white);

    Picture picture;
    const float distances[3] = { 8.0f, 30.0f, 80.0f };
    for (int i = 0; i < 3; ++i)
    {
        world.placeWall(distances[i]);
        CHECK(capture(*world.tree, picture));
        const Ray ray = rayAt(32, 24);
        const float length = distances[i] * ray.norm;
        const float f = powf(smoothstepOf(10.0f, 50.0f, length), 2.0f) * 0.8f;
        char label[64];
        snprintf(label, sizeof(label), "begin 10 end 50 curve 2, distance %.0f, f %.4f",
                distances[i], f);
        CHECK(pixelIs(picture, label, 32, 24, mixLin(kWhiteLin, world.fogColour, f)));
    }

    world.settings.fogDepthEnd = 0.0f;
    world.settings.fogDepthCurve = 1.0f;
    world.settings.fogDensity = 1.0f;
    world.apply();
    world.placeWall(55.0f);
    CHECK(capture(*world.tree, picture));
    {
        const Ray ray = rayAt(32, 24);
        const float length = 55.0f * ray.norm;
        const float f = smoothstepOf(10.0f, 100.0f, length);
        CHECK(pixelIs(picture, "depth end 0 falls back to the far plane", 32, 24,
                mixLin(kWhiteLin, world.fogColour, f)));
    }
}

void testHeightFog(World& world)
{
    printf("height fog\n");
    world.settings = plainEnvironment();
    world.setFogColour(1.0f, 0.5f, 0.0f);
    world.fogExponential(0.0f);
    world.settings.fogLightColor = Math::Vec3(1.0f, 0.5f, 0.0f);
    world.settings.fogHeight = 0.0f;
    world.settings.fogHeightDensity = 0.2f;
    world.apply();
    world.wall->setMaterialOverride(world.white);
    world.placeWall(20.0f);

    Picture picture;
    CHECK(capture(*world.tree, picture));
    const int rows[2] = { 8, 40 };
    for (int r = 0; r < 2; ++r)
    {
        const Ray ray = rayAt(32, rows[r]);
        const float y = ray.y * 20.0f;
        const float f = 1.0f - expf(fminf(0.0f, y * 0.2f));
        char label[64];
        snprintf(label, sizeof(label), "height 0, density 0.2, world y %.2f", y);
        CHECK(pixelIs(picture, label, 32, rows[r], mixLin(kWhiteLin, world.fogColour, f)));
    }

    world.settings.fogHeight = 10.0f;
    world.apply();
    CHECK(capture(*world.tree, picture));
    for (int r = 0; r < 2; ++r)
    {
        const Ray ray = rayAt(32, rows[r]);
        const float y = ray.y * 20.0f;
        const float f = 1.0f - expf(fminf(0.0f, (y - 10.0f) * 0.2f));
        char label[64];
        snprintf(label, sizeof(label), "height 10, density 0.2, world y %.2f", y);
        CHECK(pixelIs(picture, label, 32, rows[r], mixLin(kWhiteLin, world.fogColour, f)));
    }

    world.settings.fogHeight = 0.0f;
    world.settings.fogHeightDensity = -0.2f;
    world.apply();
    CHECK(capture(*world.tree, picture));
    for (int r = 0; r < 2; ++r)
    {
        const Ray ray = rayAt(32, rows[r]);
        const float y = ray.y * 20.0f;
        const float f = 1.0f - expf(fminf(0.0f, y * -0.2f));
        char label[64];
        snprintf(label, sizeof(label), "height 0, density -0.2, world y %.2f", y);
        CHECK(pixelIs(picture, label, 32, rows[r], mixLin(kWhiteLin, world.fogColour, f)));
    }

    world.settings.fogHeightDensity = 0.2f;
    world.settings.fogDensity = 0.05f;
    world.apply();
    CHECK(capture(*world.tree, picture));
    {
        const Ray ray = rayAt(32, 40);
        const float y = ray.y * 20.0f;
        const float fy = 1.0f - expf(fminf(0.0f, y * 0.2f));
        const float fd = 1.0f - expf(-20.0f * ray.norm * 0.05f);
        const float f = fy > fd ? fy : fd;
        CHECK(pixelIs(picture, "height and distance fog take the larger", 32, 40,
                mixLin(kWhiteLin, world.fogColour, f)));
    }
}

void testLightEnergy(World& world)
{
    printf("fog light colour and energy\n");
    world.settings = plainEnvironment();
    world.setFogColour(1.0f, 0.5f, 0.0f);
    world.fogExponential(10.0f);
    world.settings.fogLightColor = Math::Vec3(1.0f, 0.5f, 0.0f);
    world.wall->setMaterialOverride(world.white);
    world.placeWall(10.0f);

    Picture picture;
    const float energies[3] = { 1.0f, 0.5f, 2.0f };
    for (int i = 0; i < 3; ++i)
    {
        world.settings.fogLightEnergy = energies[i];
        world.apply();
        CHECK(capture(*world.tree, picture));
        char label[64];
        snprintf(label, sizeof(label), "saturated fog, energy %.1f", energies[i]);
        CHECK(pixelIs(picture, label, 32, 24, scaleLin(world.fogColour, energies[i])));
    }
}

void testSunScatter(World& world)
{
    printf("fog sun scatter\n");
    world.settings = plainEnvironment();
    world.setFogColour(0.2f, 0.2f, 0.2f);
    world.fogExponential(10.0f);
    world.settings.fogLightColor = Math::Vec3(0.2f, 0.2f, 0.2f);
    world.settings.fogSunScatter = 0.1f;
    world.apply();
    world.wall->setMaterialOverride(world.white);
    world.placeWall(10.0f);

    DirectionalLight3D* sun = world.root->createChild<DirectionalLight3D>();
    sun->setEnergy(1.0f);

    Picture picture;
    const int pixels[2][2] = { { 32, 24 }, { 8, 10 } };

    sun->setGlobalTransform(Math::Mat4::RotationY(kPi));
    CHECK(capture(*world.tree, picture));
    for (int p = 0; p < 2; ++p)
    {
        const Ray ray = rayAt(pixels[p][0], pixels[p][1]);
        const float facing = 1.0f / ray.norm;
        const float amount = powf(facing, 8.0f);
        const Lin expected = addLin(world.fogColour, scaleLin(kWhiteLin, kPi * 0.1f * amount));
        char label[64];
        snprintf(label, sizeof(label), "sun in front, cosine %.3f", facing);
        CHECK(pixelIs(picture, label, pixels[p][0], pixels[p][1], expected));
    }

    sun->setGlobalTransform(Math::Mat4::Identity());
    CHECK(capture(*world.tree, picture));
    CHECK(pixelIs(picture, "sun behind the camera adds nothing", 32, 24, world.fogColour));

    sun->setGlobalTransform(Math::Mat4::RotationY(kPi));
    world.settings.fogSunScatter = 0.0f;
    world.apply();
    CHECK(capture(*world.tree, picture));
    CHECK(pixelIs(picture, "scatter 0 adds nothing", 32, 24, world.fogColour));

    sun->queueFree();
    {
        Frame frame(*world.tree);
    }
}

void testAerialPerspective(World& world)
{
    printf("aerial perspective takes the fog colour from the sky\n");
    world.settings = plainEnvironment();
    useSky(world, 0.5f);
    world.setFogColour(1.0f, 0.5f, 0.0f);
    world.fogExponential(10.0f);
    world.settings.fogLightColor = Math::Vec3(1.0f, 0.5f, 0.0f);
    world.settings.fogSkyAffect = 0.0f;
    world.apply();
    world.wall->setMaterialOverride(world.white);
    world.placeWall(10.0f);

    const Lin sky = fromSrgb(0.5f, 0.5f, 0.5f);
    Picture picture;
    const float amounts[3] = { 0.0f, 0.5f, 1.0f };
    for (int i = 0; i < 3; ++i)
    {
        world.settings.fogAerialPerspective = amounts[i];
        world.apply();
        CHECK(capture(*world.tree, picture, 6));
        char label[64];
        snprintf(label, sizeof(label), "aerial perspective %.1f", amounts[i]);
        CHECK(pixelIs(picture, label, 32, 24, mixLin(world.fogColour, sky, amounts[i])));
    }
}

void testSkyAffect(World& world)
{
    printf("fog on the sky and on the flat background\n");
    world.settings = plainEnvironment();
    useSky(world, 0.5f);
    world.setFogColour(1.0f, 0.5f, 0.0f);
    world.fogExponential(0.01f);
    world.settings.fogLightColor = Math::Vec3(1.0f, 0.5f, 0.0f);
    world.wall->setVisible(false);

    const Lin sky = fromSrgb(0.5f, 0.5f, 0.5f);
    Picture picture;
    world.fogOff();
    world.apply();
    CHECK(capture(*world.tree, picture, 6));
    CHECK(pixelIs(picture, "sky, fog off", 32, 24, sky));

    world.fogExponential(0.01f);
    world.settings.fogLightColor = Math::Vec3(1.0f, 0.5f, 0.0f);
    const float affects[3] = { 0.0f, 0.5f, 1.0f };
    for (int i = 0; i < 3; ++i)
    {
        world.settings.fogSkyAffect = affects[i];
        world.apply();
        CHECK(capture(*world.tree, picture, 6));
        char label[64];
        snprintf(label, sizeof(label), "sky affect %.1f", affects[i]);
        CHECK(pixelIs(picture, label, 32, 24, mixLin(sky, world.fogColour, affects[i])));
    }

    world.settings.fogSkyAffect = 1.0f;
    world.settings.fogAerialPerspective = 1.0f;
    world.apply();
    CHECK(capture(*world.tree, picture, 6));
    CHECK(pixelIs(picture, "sky affect 1, aerial perspective 1", 32, 24, sky));

    world.settings.fogAerialPerspective = 0.0f;
    world.settings.fogSunScatter = 0.5f;
    world.apply();
    DirectionalLight3D* sun = world.root->createChild<DirectionalLight3D>();
    sun->setEnergy(1.0f);
    sun->setGlobalTransform(Math::Mat4::RotationX(-kPi * 0.25f) * Math::Mat4::RotationY(kPi));
    CHECK(capture(*world.tree, picture, 6));
    {
        const Math::Vec3 toSun =
                Math::Mat3(Math::Mat4::RotationX(-kPi * 0.25f) * Math::Mat4::RotationY(kPi)) *
                Math::Vec3(0.0f, 0.0f, 1.0f);
        const Ray ray = rayAt(32, 24);
        const float cosine = (toSun.x * ray.x + toSun.y * ray.y - toSun.z) / ray.norm;
        const float amount = powf(fmaxf(cosine, 0.0f), 8.0f);
        const Lin expected = addLin(world.fogColour, scaleLin(kWhiteLin, amount * kPi * 0.5f));
        CHECK(pixelIs(picture, "sky sun scatter at 45 degrees", 32, 24, expected));
    }
    sun->queueFree();
    {
        Frame frame(*world.tree);
    }

    world.settings = plainEnvironment();
    world.settings.background = BackgroundMode::ClearColor;
    world.setFogColour(1.0f, 0.5f, 0.0f);
    world.fogExponential(0.01f);
    world.settings.fogLightColor = Math::Vec3(1.0f, 0.5f, 0.0f);
    const Lin clear = fromSrgb(kClearSrgb[0], kClearSrgb[1], kClearSrgb[2]);
    world.fogOff();
    world.apply();
    CHECK(capture(*world.tree, picture));
    CHECK(pixelIs(picture, "clear colour, fog off", 32, 24, clear));
    world.fogExponential(0.0f);
    world.settings.fogLightColor = Math::Vec3(1.0f, 0.5f, 0.0f);
    world.settings.fogSkyAffect = 0.0f;
    world.apply();
    CHECK(capture(*world.tree, picture));
    CHECK(pixelIs(picture, "clear colour, fog on, sky affect 0", 32, 24, clear));
    for (int i = 0; i < 3; ++i)
    {
        world.settings.fogSkyAffect = affects[i];
        world.apply();
        CHECK(capture(*world.tree, picture));
        char label[64];
        snprintf(label, sizeof(label), "clear colour, sky affect %.1f", affects[i]);
        CHECK(pixelIs(picture, label, 32, 24, mixLin(clear, world.fogColour, affects[i])));
    }

    world.settings.background = BackgroundMode::Color;
    world.settings.backgroundColor = Math::Vec3(0.8f, 0.4f, 0.2f);
    world.settings.backgroundEnergy = 0.5f;
    const Lin colour = scaleLin(fromSrgb(0.8f, 0.4f, 0.2f), 0.5f);
    world.settings.fogSkyAffect = 0.0f;
    world.apply();
    CHECK(capture(*world.tree, picture));
    CHECK(pixelIs(picture, "background colour with energy 0.5, affect 0", 32, 24, colour));
    world.settings.fogSkyAffect = 0.5f;
    world.apply();
    CHECK(capture(*world.tree, picture));
    CHECK(pixelIs(picture, "background colour with energy 0.5, affect 0.5", 32, 24,
            mixLin(colour, world.fogColour, 0.5f)));
    world.settings.fogEnabled = false;
    world.apply();
    CHECK(capture(*world.tree, picture));
    CHECK(pixelIs(picture, "background colour, fog off", 32, 24, colour));

    world.wall->setVisible(true);
}

void testBlending(World& world)
{
    printf("transparent and additive materials in fog\n");
    world.settings = plainEnvironment();
    world.setFogColour(1.0f, 0.5f, 0.0f);
    world.fogOff();
    world.apply();
    world.placeWall(10.0f);
    const Lin clear = fromSrgb(kClearSrgb[0], kClearSrgb[1], kClearSrgb[2]);

    MaterialStore& materials = *Main::materials();
    const MaterialId additive = materials.create("additive");
    materials.setColor(additive, MaterialParam::Albedo, Math::Vec4(1.0f, 1.0f, 1.0f, 0.5f));
    materials.setShading(additive, MaterialShading::Unshaded);
    materials.setBlendMode(additive, MaterialBlend::Add);
    const MaterialId mixed = materials.create("mixed");
    materials.setColor(mixed, MaterialParam::Albedo, Math::Vec4(1.0f, 1.0f, 1.0f, 0.5f));
    materials.setShading(mixed, MaterialShading::Unshaded);
    materials.setTransparency(mixed, MaterialTransparency::AlphaBlend);

    Picture picture;
    const float f = 1.0f - expf(-10.0f * rayAt(32, 24).norm * 0.05f);
    const MaterialId both[2] = { additive, mixed };
    const char* names[2] = { "additive", "alpha blended" };
    for (int m = 0; m < 2; ++m)
    {
        world.wall->setMaterialOverride(both[m]);
        world.fogOff();
        world.apply();
        CHECK(capture(*world.tree, picture));
        char label[64];
        const Lin plain =
                m == 0 ? addLin(clear, scaleLin(kWhiteLin, 0.5f)) : mixLin(clear, kWhiteLin, 0.5f);
        snprintf(label, sizeof(label), "%s, fog off", names[m]);
        CHECK(pixelIs(picture, label, 32, 24, plain));

        world.fogExponential(0.05f);
        world.settings.fogLightColor = Math::Vec3(1.0f, 0.5f, 0.0f);
        world.settings.fogSkyAffect = 0.0f;
        world.apply();
        CHECK(capture(*world.tree, picture));
        const Lin surface = mixLin(kWhiteLin, world.fogColour, f);
        const Lin foggy =
                m == 0 ? addLin(clear, scaleLin(surface, 0.5f)) : mixLin(clear, surface, 0.5f);
        snprintf(label, sizeof(label), "%s, fog %.4f", names[m], f);
        CHECK(pixelIs(picture, label, 32, 24, foggy));

        world.settings.fogDensity = 10.0f;
        world.apply();
        CHECK(capture(*world.tree, picture));
        const Lin saturated = m == 0 ? addLin(clear, scaleLin(world.fogColour, 0.5f))
                                     : mixLin(clear, world.fogColour, 0.5f);
        snprintf(label, sizeof(label), "%s, saturated fog", names[m]);
        CHECK(pixelIs(picture, label, 32, 24, saturated));

        world.settings.fogSkyAffect = 1.0f;
        world.apply();
        CHECK(capture(*world.tree, picture));
        const Lin behind = world.fogColour;
        const Lin fogged = m == 0 ? addLin(behind, scaleLin(world.fogColour, 0.5f))
                                  : mixLin(behind, world.fogColour, 0.5f);
        snprintf(label, sizeof(label), "%s, saturated fog, fogged background", names[m]);
        CHECK(pixelIs(picture, label, 32, 24, fogged));
    }
    world.wall->setMaterialOverride(world.white);
    materials.destroy(additive);
    materials.destroy(mixed);
}

void testLit(World& world)
{
    printf("lit surface keeps its shading under the fog\n");
    world.settings = plainEnvironment();
    world.settings.ambientEnergy = 0.5f;
    world.setFogColour(1.0f, 0.5f, 0.0f);
    world.fogOff();
    world.apply();
    world.wall->setMaterialOverride(world.matte);
    world.placeWall(10.0f);

    Picture picture;
    CHECK(capture(*world.tree, picture));
    CHECK(pixelIs(picture, "lit wall, fog off", 32, 24, Lin{ 0.5f, 0.5f, 0.5f }));

    world.fogExponential(0.05f);
    world.settings.fogLightColor = Math::Vec3(1.0f, 0.5f, 0.0f);
    world.apply();
    CHECK(capture(*world.tree, picture));
    const Ray ray = rayAt(32, 24);
    const float f = 1.0f - expf(-10.0f * ray.norm * 0.05f);
    CHECK(pixelIs(picture, "lit wall, ambient 0.5, fog", 32, 24,
            mixLin(Lin{ 0.5f, 0.5f, 0.5f }, world.fogColour, f)));

    world.settings.ambientEnergy = 0.25f;
    world.apply();
    CHECK(capture(*world.tree, picture));
    CHECK(pixelIs(picture, "lit wall, ambient 0.25, fog", 32, 24,
            mixLin(Lin{ 0.25f, 0.25f, 0.25f }, world.fogColour, f)));

    world.settings.fogMode = FogMode::Depth;
    world.settings.fogDepthBegin = 5.0f;
    world.settings.fogDepthEnd = 30.0f;
    world.settings.fogDepthCurve = 1.0f;
    world.settings.fogDensity = 1.0f;
    world.apply();
    CHECK(capture(*world.tree, picture));
    const float depthF = smoothstepOf(5.0f, 30.0f, 10.0f * ray.norm);
    CHECK(pixelIs(picture, "lit wall, depth fog", 32, 24,
            mixLin(Lin{ 0.25f, 0.25f, 0.25f }, world.fogColour, depthF)));

    world.wall->setMaterialOverride(world.white);
}

void testMultiMesh(World& world)
{
    printf("multimesh variant\n");
    world.settings = plainEnvironment();
    world.setFogColour(1.0f, 0.5f, 0.0f);
    world.fogExponential(0.05f);
    world.settings.fogLightColor = Math::Vec3(1.0f, 0.5f, 0.0f);
    world.apply();
    world.wall->setVisible(false);

    const MeshId plane = createPlaneMesh(*Main::meshes(), 400.0f, 400.0f);
    MultiMeshInstance3D* instances = world.root->createChild<MultiMeshInstance3D>();
    instances->setMesh(plane);
    instances->setInstanceCount(1);
    instances->setInstanceTransform(0, Math::Mat4::Translation(Math::Vec3(0.0f, 0.0f, -10.0f)) *
                                               Math::Mat4::RotationX(kPi * 0.5f));
    instances->setMaterialOverride(world.white);

    Picture picture;
    CHECK(capture(*world.tree, picture));
    const float f = 1.0f - expf(-10.0f * rayAt(32, 24).norm * 0.05f);
    CHECK(pixelIs(picture, "multimesh, density 0.05, distance 10", 32, 24,
            mixLin(kWhiteLin, world.fogColour, f)));

    instances->queueFree();
    {
        Frame frame(*world.tree);
    }
    world.wall->setVisible(true);
}

void testImpostor(World& world)
{
    printf("impostor variant\n");
    world.settings = plainEnvironment();
    world.setFogColour(1.0f, 0.5f, 0.0f);
    world.fogOff();
    world.apply();
    world.wall->setVisible(false);

    const MeshId box = Main::meshes()->create("fog box");
    CHECK(Main::meshes()->addSurface(box, colouredBox()));
    Forest3D* forest = world.root->createChild<Forest3D>();
    forest->setSwapDistance(2.0f);
    forest->setDrawDistance(100.0f);
    ForestSpeciesDesc species;
    species.mesh = box;
    CHECK(forest->addSpecies(species) == 0);
    forest->plant(0, Math::Vec3(0.0f, 0.0f, 0.0f), 0.0f, 1.0f);
    world.camera->setGlobalTransform(Math::Mat4::LookAt(Math::Vec3(0.0f, 0.5f, 10.0f),
            Math::Vec3(0.0f, 0.5f, 0.0f), Math::Vec3(0.0f, 1.0f, 0.0f))
                                             .InverseAffine());

    Picture picture;
    CHECK(capture(*world.tree, picture));
    CHECK(forest->getImpostorInstanceCount() == 1);
    CHECK(pixelIs(picture, "impostor card, fog off", 32, 24, Lin{ 1.0f, 0.0f, 0.0f }));

    world.fogExponential(10.0f);
    world.settings.fogLightColor = Math::Vec3(1.0f, 0.5f, 0.0f);
    world.apply();
    CHECK(capture(*world.tree, picture));
    CHECK(pixelIs(picture, "impostor card, saturated fog", 32, 24, world.fogColour));

    forest->queueFree();
    {
        Frame frame(*world.tree);
    }
    world.camera->setGlobalTransform(Math::Mat4::Identity());
    world.wall->setVisible(true);
}

void testTerrain(World& world)
{
    printf("terrain variants\n");
    world.settings = plainEnvironment();
    world.setFogColour(1.0f, 0.5f, 0.0f);
    world.fogOff();
    world.apply();
    world.wall->setVisible(false);

    MaterialStore& materials = *Main::materials();
    const MaterialId red = materials.create("terrain red");
    materials.setColor(red, MaterialParam::Albedo, Math::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
    materials.setFloat(red, MaterialParam::Roughness, 1.0f);

    ct::Vector<float> flat;
    flat.resize(129u * 129u);
    for (std::size_t i = 0; i < flat.size(); ++i) flat[i] = 0.0f;
    TerrainImportData data;
    data.terrainSize = 129;
    data.minBatchSize = 17;
    data.maxBatchSize = 33;
    data.worldSize = 100.0f;
    data.inputFloat = flat.data();

    Terrain3D* terrain = world.root->createChild<Terrain3D>();
    terrain->setLightmapEnabled(false);
    CHECK(terrain->load(data));
    world.camera->setGlobalTransform(Math::Mat4::LookAt(Math::Vec3(0.0f, 10.0f, 0.0f),
            Math::Vec3(0.0f, 0.0f, 0.0f), Math::Vec3(0.0f, 0.0f, -1.0f))
                                             .InverseAffine());

    const Ray ray = rayAt(32, 24);
    const float f = 1.0f - expf(-10.0f * ray.norm * 0.05f);
    Picture picture;
    for (int pass = 0; pass < 2; ++pass)
    {
        terrain->setMaterial(pass == 0 ? red : MaterialId());
        const char* name = pass == 0 ? "terrain, own material" : "terrain, layered material";
        world.fogOff();
        world.apply();
        CHECK(capture(*world.tree, picture));
        const unsigned char* base = picture.at(32, 24);
        const Lin plain = fromSrgb(static_cast<float>(base[0]) / 255.0f,
                static_cast<float>(base[1]) / 255.0f, static_cast<float>(base[2]) / 255.0f);
        char label[64];
        snprintf(label, sizeof(label), "%s, fog off", name);
        printf("  %-44s (32,24) is %3d %3d %3d\n", label, base[0], base[1], base[2]);
        if (pass == 0) CHECK(abs(base[0] - 255) <= 3 && base[1] <= 3 && base[2] <= 3);

        world.fogExponential(0.05f);
        world.settings.fogLightColor = Math::Vec3(1.0f, 0.5f, 0.0f);
        world.apply();
        CHECK(capture(*world.tree, picture));
        snprintf(label, sizeof(label), "%s, fog %.4f", name, f);
        CHECK(pixelIs(picture, label, 32, 24, mixLin(plain, world.fogColour, f)));

        world.settings.fogDensity = 10.0f;
        world.apply();
        CHECK(capture(*world.tree, picture));
        snprintf(label, sizeof(label), "%s, saturated fog", name);
        CHECK(pixelIs(picture, label, 32, 24, world.fogColour));
    }

    terrain->queueFree();
    {
        Frame frame(*world.tree);
    }
    materials.destroy(red);
    world.camera->setGlobalTransform(Math::Mat4::Identity());
    world.wall->setVisible(true);
}

TextureId solidTexture(TextureStore& textures)
{
    Pixmap pixels(16, 16, 4);
    for (int y = 0; y < 16; ++y)
        for (int x = 0; x < 16; ++x)
            pixels.set_pixel(static_cast<std::uint32_t>(x), static_cast<std::uint32_t>(y), 255, 255,
                    255, 255);
    return textures.createFromPixmap(pixels, true, true, "fog grass", 4);
}

void testGrass(World& world)
{
    printf("grass variant\n");
    world.settings = plainEnvironment();
    world.setFogColour(1.0f, 0.5f, 0.0f);
    world.fogOff();
    world.apply();
    world.wall->setVisible(false);

    const TextureId texture = solidTexture(*Main::textures());
    Grass3D* grass = world.root->createChild<Grass3D>();
    grass->setTexture(texture);
    grass->setCellSize(4.0f);
    grass->setDensity(2.0f);
    grass->setDrawDistance(40.0f);
    grass->setReleaseMargin(0.0f);
    grass->setWind(0.0f);
    grass->setExtent(-3.0f, -20.0f, 3.0f, -10.0f);
    world.camera->setGlobalTransform(Math::Mat4::LookAt(Math::Vec3(0.0f, 1.0f, 0.0f),
            Math::Vec3(0.0f, 1.0f, -10.0f), Math::Vec3(0.0f, 1.0f, 0.0f))
                                             .InverseAffine());

    Picture plain;
    CHECK(capture(*world.tree, plain));
    int bestX = -1;
    int bestY = -1;
    int best = 0;
    for (int y = 0; y < kHeight; ++y)
    {
        for (int x = 0; x < kWidth; ++x)
        {
            const unsigned char* p = plain.at(x, y);
            const int away = abs(p[0] - 26) + abs(p[1] - 51) + abs(p[2] - 77);
            if (away > best || (away == best && abs(x - 32) < abs(bestX - 32)))
            {
                best = away;
                bestX = x;
                bestY = y;
            }
        }
    }
    CHECK(best > 40);
    printf("  grass pixel (%d,%d) is %d %d %d without fog\n", bestX, bestY,
            plain.at(bestX, bestY)[0], plain.at(bestX, bestY)[1], plain.at(bestX, bestY)[2]);

    world.fogExponential(10.0f);
    world.settings.fogLightColor = Math::Vec3(1.0f, 0.5f, 0.0f);
    world.apply();
    Picture foggy;
    CHECK(capture(*world.tree, foggy));
    CHECK(pixelIs(foggy, "grass, saturated fog", bestX, bestY, world.fogColour));

    grass->queueFree();
    {
        Frame frame(*world.tree);
    }
    Main::textures()->destroy(texture);
    world.camera->setGlobalTransform(Math::Mat4::Identity());
    world.wall->setVisible(true);
}

} // namespace

void testCurrentEnvironment(World& world)
{
    printf("the first environment in the tree is the current one\n");
    world.wall->setVisible(false);
    world.settings = plainEnvironment();
    world.fogOff();
    world.settings.background = BackgroundMode::Color;
    world.settings.backgroundEnergy = 1.0f;
    world.settings.backgroundColor = Math::Vec3(0.8f, 0.2f, 0.1f);
    world.apply();
    const Lin first = fromSrgb(0.8f, 0.2f, 0.1f);
    const Lin second = fromSrgb(0.1f, 0.7f, 0.3f);
    const Lin clear = fromSrgb(kClearSrgb[0], kClearSrgb[1], kClearSrgb[2]);

    EnvironmentSettings other = world.settings;
    other.backgroundColor = Math::Vec3(0.1f, 0.7f, 0.3f);
    WorldEnvironment* later = world.root->createChild<WorldEnvironment>();
    later->setEnvironment(other);

    Picture picture;
    CHECK(capture(*world.tree, picture));
    CHECK(pixelIs(picture, "two environments, the first wins", 32, 24, first));

    world.root->removeChild(world.environment);
    CHECK(capture(*world.tree, picture));
    CHECK(pixelIs(picture, "first removed, the other takes over", 32, 24, second));

    world.root->addChild(world.environment);
    CHECK(capture(*world.tree, picture));
    CHECK(pixelIs(picture, "added back after the other, the other stays", 32, 24, second));

    later->queueFree();
    CHECK(capture(*world.tree, picture));
    CHECK(pixelIs(picture, "the other freed, the first is back", 32, 24, first));

    world.root->removeChild(world.environment);
    CHECK(capture(*world.tree, picture));
    CHECK(pixelIs(picture, "no environment left, clear colour", 32, 24, clear));
    world.root->addChild(world.environment);

    world.settings = plainEnvironment();
    world.apply();
    world.wall->setVisible(true);
}

int main(int argc, char** argv)
{
    const bool vulkan = argc > 1 && strcmp(argv[1], "vulkan") == 0;

    CHECK(Device::headlessSupported());
    if (!Device::headlessSupported()) return 1;

    DeviceDesc desc;
    desc.title = "fog test";
    desc.driver = vulkan ? prisma::DriverType::Vulkan : prisma::DriverType::OpenGL;
    desc.width = kWidth;
    desc.height = kHeight;
    desc.headless = true;

    CHECK(Main::setup(desc));
    if (!Main::device()) return 1;

    SceneTree tree;
    CHECK(Main::start(&tree));
    Node* root = tree.getRoot();
    Window* window = root ? root->castTo<Window>() : nullptr;
    CHECK(window != nullptr);
    if (!window) return 1;
    window->setClearColor(Math::Vec4(kClearSrgb[0], kClearSrgb[1], kClearSrgb[2], 1.0f));

    World world;
    world.tree = &tree;
    world.root = root;
    world.window = window;
    world.environment = root->createChild<WorldEnvironment>();
    world.settings = plainEnvironment();
    world.apply();
    world.camera = root->createChild<Camera3D>();
    world.camera->setPerspective(60.0f, 0.1f, 100.0f);
    world.camera->makeCurrent();

    MaterialStore& materials = *Main::materials();
    world.white = materials.create("fog white");
    materials.setColor(world.white, MaterialParam::Albedo, Math::Vec4(1.0f, 1.0f, 1.0f, 1.0f));
    materials.setShading(world.white, MaterialShading::Unshaded);
    world.matte = materials.create("fog matte");
    materials.setFloat(world.matte, MaterialParam::Specular, 0.0f);
    materials.setFloat(world.matte, MaterialParam::Roughness, 1.0f);

    world.wall = root->createChild<MeshInstance3D>();
    world.wall->setMesh(createPlaneMesh(*Main::meshes(), 400.0f, 400.0f));
    world.wall->setMaterialOverride(world.white);
    world.placeWall(10.0f);

    testUnshadedExponential(world);
    testDepthMode(world);
    testHeightFog(world);
    testLightEnergy(world);
    testSunScatter(world);
    testAerialPerspective(world);
    testSkyAffect(world);
    testBlending(world);
    testLit(world);
    testMultiMesh(world);
    testImpostor(world);
    testTerrain(world);
    testGrass(world);
    testCurrentEnvironment(world);

    Main::device()->requestClose();
    tree.finalize();
    Main::cleanup();

    if (failures) printf("%d check(s) failed\n", failures);
    else
        printf("all checks passed\n");
    return failures ? 1 : 0;
}
