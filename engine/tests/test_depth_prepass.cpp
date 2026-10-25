#include "Check.h"
#include "LitTest.h"

#include "prisma3d/Device.h"
#include "prisma3d/core/Pixmap.h"
#include "prisma3d/main/Main.h"
#include "prisma3d/render/MaterialStore.h"
#include "prisma3d/render/MeshStore.h"
#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/render/TextureStore.h"
#include "prisma3d/scene/Camera3D.h"
#include "prisma3d/scene/Decal3D.h"
#include "prisma3d/scene/Light3D.h"
#include "prisma3d/scene/MultiMeshInstance3D.h"
#include "prisma3d/scene/PrimitiveMeshes.h"
#include "prisma3d/scene/SceneTree.h"
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

const int kWidth = 128;
const int kHeight = 96;
const float kPi = 3.14159265358979f;

struct Rgb
{
    int r;
    int g;
    int b;
};

const Rgb kClear = { 26, 51, 77 };
const Rgb kRed = { 255, 0, 0 };
const Rgb kGreen = { 0, 255, 0 };
const Rgb kBlue = { 0, 0, 255 };
const Rgb kCyan = { 0, 255, 255 };
const Rgb kBlack = { 0, 0, 0 };

float gFocal = 1.0f;
float gFov = 60.0f;
float gNear = 0.1f;
float gFar = 100.0f;

int column(float x, float distance)
{
    return static_cast<int>(floorf(static_cast<float>(kWidth) * 0.5f + gFocal * x / distance));
}

int row(float y, float distance)
{
    return static_cast<int>(floorf(static_cast<float>(kHeight) * 0.5f - gFocal * y / distance));
}

float depthAt(float distance)
{
    const float ndc =
            (gFar + gNear) / (gFar - gNear) - (2.0f * gFar * gNear) / ((gFar - gNear) * distance);
    return ndc * 0.5f + 0.5f;
}

struct Probe
{
    int x;
    int y;
};

struct Shot
{
    ct::Vector<std::uint8_t> pixels;
    ct::Vector<float> depths;
    RenderStats stats;
};

struct Context
{
    SceneTree* tree = nullptr;
    Node* root = nullptr;
    Node3D* holder = nullptr;
    Camera3D* camera = nullptr;
    WorldEnvironment* environment = nullptr;
    Window* window = nullptr;
    MeshStore* meshes = nullptr;
    TextureStore* textures = nullptr;
    MaterialStore* materials = nullptr;
};

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

void setLens(Context& c, float fov, const Math::Vec3& position)
{
    gFov = fov;
    gNear = 0.05f;
    gFar = 100.0f;
    gFocal = 0.5f * static_cast<float>(kHeight) / tanf(fov * 0.5f * kPi / 180.0f);
    c.camera->setPerspective(fov, gNear, gFar);
    c.camera->setPosition(position);
}

Shot shoot(Context& c, bool prepass, const ct::Vector<Probe>* depthProbes = nullptr)
{
    Main::rendering()->setDepthPrePassEnabled(prepass);
    Shot shot;
    Frame frame(*c.tree);
    shot.pixels.resize(static_cast<std::size_t>(kWidth) * kHeight * 4);
    prisma::Rect rect;
    rect.x = 0;
    rect.y = 0;
    rect.width = kWidth;
    rect.height = kHeight;
    CHECK(Main::device()->driver()->readPixels(prisma::RenderTarget(), rect, shot.pixels.data()));
    shot.stats = Main::rendering()->getStats();
    if (depthProbes)
    {
        for (std::size_t i = 0; i < depthProbes->size(); ++i)
        {
            float depth = -1.0f;
            CHECK(Main::rendering()->viewportReadDepth(c.window->getViewportId(),
                    (*depthProbes)[i].x, (*depthProbes)[i].y, depth));
            shot.depths.push_back(depth);
        }
    }
    return shot;
}

Rgb pixelOf(const Shot& shot, int x, int y)
{
    const std::uint8_t* p = shot.pixels.data() + (static_cast<std::size_t>(y) * kWidth + x) * 4;
    return Rgb{ p[0], p[1], p[2] };
}

bool shotIs(const Shot& shot, int x, int y, Rgb expected, int tolerance, const char* label)
{
    if (x < 0 || y < 0 || x >= kWidth || y >= kHeight)
    {
        printf("%s: pixel (%d,%d) is outside the frame\n", label, x, y);
        return false;
    }
    const Rgb got = pixelOf(shot, x, y);
    const bool same = abs(got.r - expected.r) <= tolerance &&
                      abs(got.g - expected.g) <= tolerance && abs(got.b - expected.b) <= tolerance;
    if (!same)
        printf("%s: pixel (%d,%d) is %d,%d,%d, expected %d,%d,%d\n", label, x, y, got.r, got.g,
                got.b, expected.r, expected.g, expected.b);
    return same;
}

bool shotAgrees(const Shot& shot, int x, int y, const Shot& reference, int tolerance,
        const char* label)
{
    return shotIs(shot, x, y, pixelOf(reference, x, y), tolerance, label);
}

std::size_t differences(const Shot& a, const Shot& b)
{
    std::size_t count = 0;
    for (std::size_t i = 0; i < a.pixels.size() && i < b.pixels.size(); ++i)
        if (a.pixels[i] != b.pixels[i]) ++count;
    return count;
}

bool differsFromClear(const Shot& shot, int x, int y)
{
    const Rgb got = pixelOf(shot, x, y);
    return abs(got.r - kClear.r) > 6 || abs(got.g - kClear.g) > 6 || abs(got.b - kClear.b) > 6;
}

int countPixels(const Shot& shot, Rgb wanted, int tolerance)
{
    int count = 0;
    for (int y = 0; y < kHeight; ++y)
    {
        for (int x = 0; x < kWidth; ++x)
        {
            const Rgb got = pixelOf(shot, x, y);
            if (abs(got.r - wanted.r) <= tolerance && abs(got.g - wanted.g) <= tolerance &&
                    abs(got.b - wanted.b) <= tolerance)
                ++count;
        }
    }
    return count;
}

Shot both(Context& c, const char* label, Shot* off = nullptr,
        const ct::Vector<Probe>* depthProbes = nullptr)
{
    Shot without = shoot(c, false, depthProbes);
    Shot with = shoot(c, true, depthProbes);
    const std::size_t different = differences(with, without);
    if (different != 0) printf("%s: %zu bytes differ with the pre-pass\n", label, different);
    CHECK(different == 0);
    CHECK(without.stats.depthPrePassSurfaces == 0);
    CHECK(without.stats.depthPrePassDrawCalls == 0);
    CHECK(without.stats.depthPrePassTriangles == 0);
    CHECK(with.stats.drawCalls == without.stats.drawCalls);
    CHECK(with.stats.triangles == without.stats.triangles);
    CHECK(with.stats.surfacesDrawn == without.stats.surfacesDrawn);
    CHECK(with.stats.shadowTriangles == without.stats.shadowTriangles);
    for (std::size_t i = 0; i < with.depths.size() && i < without.depths.size(); ++i)
    {
        if (memcmp(&with.depths[i], &without.depths[i], sizeof(float)) != 0)
            printf("%s: depth probe %zu is %.9f with the pre-pass and %.9f without\n", label, i,
                    with.depths[i], without.depths[i]);
        CHECK(memcmp(&with.depths[i], &without.depths[i], sizeof(float)) == 0);
    }
    if (off) *off = static_cast<Shot&&>(without);
    return with;
}

SurfaceData quadSurface(float half, float z, float offsetX)
{
    SurfaceData data;
    data.positions.push_back(Math::Vec3(offsetX - half, -half, z));
    data.positions.push_back(Math::Vec3(offsetX + half, -half, z));
    data.positions.push_back(Math::Vec3(offsetX + half, half, z));
    data.positions.push_back(Math::Vec3(offsetX - half, half, z));
    for (int i = 0; i < 4; ++i) data.normals.push_back(Math::Vec3(0.0f, 0.0f, 1.0f));
    data.uvs.push_back(Math::Vec2(0.0f, 1.0f));
    data.uvs.push_back(Math::Vec2(1.0f, 1.0f));
    data.uvs.push_back(Math::Vec2(1.0f, 0.0f));
    data.uvs.push_back(Math::Vec2(0.0f, 0.0f));
    const std::uint32_t indices[6] = { 0, 1, 2, 0, 2, 3 };
    for (std::uint32_t index: indices) data.indices.push_back(index);
    return data;
}

SurfaceData quadWithoutUvs(float half, float z, float offsetX)
{
    SurfaceData data = quadSurface(half, z, offsetX);
    data.uvs.clear();
    return data;
}

MeshId createQuad(Context& c, float half, float z = 0.0f, float offsetX = 0.0f)
{
    const MeshId mesh = c.meshes->create("quad");
    CHECK(c.meshes->addSurface(mesh, quadSurface(half, z, offsetX)));
    return mesh;
}

TextureId createCutout(Context& c)
{
    Pixmap pixels(64, 8, 4);
    for (std::uint32_t y = 0; y < 8; ++y)
        for (std::uint32_t x = 0; x < 64; ++x)
            pixels.set_pixel(x, y, 255, 255, 255, x < 32 ? 255 : 0);
    return c.textures->createFromPixmap(pixels, true, false, "cutout");
}

TextureId createFlat(Context& c, std::uint8_t r, std::uint8_t g, std::uint8_t b, bool srgb)
{
    Pixmap pixels(4, 4, 4);
    for (std::uint32_t y = 0; y < 4; ++y)
        for (std::uint32_t x = 0; x < 4; ++x) pixels.set_pixel(x, y, r, g, b, 255);
    return c.textures->createFromPixmap(pixels, srgb, false, "flat");
}

MaterialId createFlatMaterial(Context& c, const Math::Vec4& albedo, MaterialShading shading)
{
    const MaterialId material = c.materials->create("flat");
    c.materials->setColor(material, MaterialParam::Albedo, albedo);
    c.materials->setShading(material, shading);
    return material;
}

MaterialId createCutoutMaterial(Context& c, TextureId texture, const Math::Vec4& albedo)
{
    const MaterialId material = c.materials->create("cutout");
    c.materials->setColor(material, MaterialParam::Albedo, albedo);
    c.materials->setTexture(material, MaterialTexture::Albedo, texture);
    c.materials->setTransparency(material, MaterialTransparency::AlphaScissor);
    c.materials->setFloat(material, MaterialParam::AlphaScissorThreshold, 0.5f);
    c.materials->setCull(material, MaterialCull::Disabled);
    return material;
}

struct Nodes
{
    ct::Vector<Node3D*> items;

    template<typename T>
    T* add(Context& c, const Math::Vec3& position)
    {
        T* node = c.holder->createChild<T>();
        node->setPosition(position);
        node->reparent(c.root);
        items.push_back(node);
        return node;
    }

    MeshInstance3D* mesh(Context& c, MeshId mesh, MaterialId material, const Math::Vec3& position)
    {
        MeshInstance3D* node = add<MeshInstance3D>(c, position);
        node->setMesh(mesh);
        node->setMaterialOverride(material);
        return node;
    }

    void release(Context& c)
    {
        for (std::size_t i = 0; i < items.size(); ++i)
        {
            items[i]->reparent(c.holder);
            items[i]->queueFree();
        }
        items.clear();
    }
};

void setBackground(Context& c, const EnvironmentSettings& settings)
{
    c.environment->setEnvironment(settings);
}

EnvironmentSettings darkEnvironment()
{
    EnvironmentSettings dark;
    dark.ambientSource = AmbientSource::Color;
    dark.reflectionSource = ReflectionSource::Disabled;
    return dark;
}

void testMaterials(Context& c)
{
    setBackground(c, whiteAmbient());
    setLens(c, 60.0f, Math::Vec3(0.0f, 0.0f, 6.0f));
    Nodes nodes;

    const MeshId box = createBoxMesh(*c.meshes, Math::Vec3(1.2f, 1.2f, 1.2f));
    const MeshId blueBoxMesh = createBoxMesh(*c.meshes, Math::Vec3(2.0f, 1.0f, 1.0f));
    const MeshId quad = createQuad(c, 1.0f);
    const MeshId overlay = createQuad(c, 0.4f);

    const MaterialId red =
            createFlatMaterial(c, Math::Vec4(1.0f, 0.0f, 0.0f, 1.0f), MaterialShading::Lit);
    const MaterialId blue =
            createFlatMaterial(c, Math::Vec4(0.0f, 0.0f, 1.0f, 1.0f), MaterialShading::Lit);
    const MaterialId unshaded =
            createFlatMaterial(c, Math::Vec4(0.0f, 1.0f, 1.0f, 1.0f), MaterialShading::Unshaded);
    const MaterialId mapped =
            createFlatMaterial(c, Math::Vec4(1.0f, 0.5f, 0.0f, 1.0f), MaterialShading::Lit);
    c.materials->setTexture(mapped, MaterialTexture::Normal, createFlat(c, 128, 128, 255, false));
    const MaterialId cutout =
            createCutoutMaterial(c, createCutout(c), Math::Vec4(0.0f, 1.0f, 0.0f, 1.0f));
    const MaterialId glass =
            createFlatMaterial(c, Math::Vec4(0.0f, 0.0f, 1.0f, 0.5f), MaterialShading::Lit);
    c.materials->setTransparency(glass, MaterialTransparency::AlphaBlend);

    nodes.mesh(c, box, red, Math::Vec3(-3.0f, 1.3f, 0.0f));
    nodes.mesh(c, box, mapped, Math::Vec3(3.0f, 1.3f, 0.0f));
    nodes.mesh(c, box, unshaded, Math::Vec3(-3.0f, -1.3f, 0.0f));
    nodes.mesh(c, quad, cutout, Math::Vec3(0.0f, 0.0f, 0.0f));
    nodes.mesh(c, blueBoxMesh, blue, Math::Vec3(1.6f, 0.0f, -1.5f));
    nodes.mesh(c, overlay, glass, Math::Vec3(-2.1f, 1.3f, 1.5f));

    ct::Vector<Probe> probes;
    const float quadDistance = 6.0f;
    const Probe solid = { column(-0.5f, quadDistance), row(0.0f, quadDistance) };
    const Probe hole = { column(0.25f, quadDistance), row(0.0f, quadDistance) };
    const Probe behind = { column(0.75f, quadDistance), row(0.0f, quadDistance) };
    const Probe redFace = { column(-3.35f, 5.4f), row(1.3f, 5.4f) };
    probes.push_back(solid);
    probes.push_back(hole);
    probes.push_back(behind);
    probes.push_back(redFace);

    Shot off;
    const Shot on = both(c, "materials", &off, &probes);

    CHECK(shotIs(on, solid.x, solid.y, kGreen, 3, "alpha scissor, solid part"));
    CHECK(shotIs(on, hole.x, hole.y, kClear, 3, "alpha scissor, cut part over the background"));
    CHECK(shotIs(on, behind.x, behind.y, kBlue, 3, "alpha scissor, cut part over the box"));
    CHECK(shotIs(on, redFace.x, redFace.y, kRed, 3, "red box"));
    CHECK(shotIs(on, column(-3.0f, 5.4f), row(-1.3f, 5.4f), kCyan, 3, "unshaded box"));
    CHECK(shotIs(on, column(3.0f, 5.4f), row(1.3f, 5.4f), Rgb{ 255, 128, 0 }, 4,
            "normal mapped box"));
    CHECK(shotIs(off, hole.x, hole.y, kClear, 3, "no pre-pass, cut part"));
    CHECK(shotIs(off, behind.x, behind.y, kBlue, 3, "no pre-pass, cut part over the box"));

    const Probe blended = { column(-2.1f, 4.5f), row(1.3f, 4.5f) };
    const Rgb mixed = pixelOf(on, blended.x, blended.y);
    CHECK(mixed.b > 100 && mixed.r > 100 && mixed.g < 20);
    const Rgb glassOverClear = pixelOf(on, column(-1.8f, 4.5f), row(1.3f, 4.5f));
    CHECK(glassOverClear.b > kClear.b &&
            differsFromClear(on, column(-1.8f, 4.5f), row(1.3f, 4.5f)));

    CHECK(on.depths.size() == 4);
    if (on.depths.size() == 4)
    {
        CHECK(fabsf(on.depths[0] - depthAt(quadDistance)) < 1e-4f);
        CHECK(on.depths[1] == 1.0f);
        CHECK(fabsf(on.depths[2] - depthAt(7.0f)) < 1e-4f);
        CHECK(fabsf(on.depths[3] - depthAt(5.4f)) < 1e-4f);
        CHECK(memcmp(&on.depths[1], &off.depths[1], sizeof(float)) == 0);
    }

    CHECK(on.stats.depthPrePassSurfaces == 5);
    CHECK(on.stats.depthPrePassDrawCalls == 5);
    CHECK(on.stats.depthPrePassTriangles == 4 * 12 + 2);
    CHECK(on.stats.surfacesDrawn == 6);
    CHECK(on.stats.drawCalls == 6);

    Main::rendering()->setDepthPrePassEnabled(true);
    CHECK(Main::rendering()->depthPrePassEnabled());
    nodes.release(c);
}

void testSky(Context& c)
{
    EnvironmentSettings sky = whiteAmbient();
    sky.background = BackgroundMode::Sky;
    sky.ambientSource = AmbientSource::Sky;
    setBackground(c, sky);
    setLens(c, 60.0f, Math::Vec3(0.0f, 0.0f, 6.0f));
    Nodes nodes;

    const MeshId box = createBoxMesh(*c.meshes, Math::Vec3(1.2f, 1.2f, 1.2f));
    const MeshId quad = createQuad(c, 1.0f);
    const MaterialId red =
            createFlatMaterial(c, Math::Vec4(1.0f, 0.0f, 0.0f, 1.0f), MaterialShading::Lit);
    const MaterialId cutout =
            createCutoutMaterial(c, createCutout(c), Math::Vec4(0.0f, 1.0f, 0.0f, 1.0f));
    nodes.mesh(c, box, red, Math::Vec3(-3.0f, 0.0f, 0.0f));
    MeshInstance3D* cut = nodes.mesh(c, quad, cutout, Math::Vec3(0.0f, 0.0f, 0.0f));

    const Probe hole = { column(0.25f, 6.0f), row(0.0f, 6.0f) };
    cut->hide();
    const Shot hidden = shoot(c, true);
    cut->show();
    Shot off;
    const Shot on = both(c, "sky", &off);
    CHECK(shotAgrees(on, hole.x, hole.y, hidden, 2, "sky through the cut part"));
    CHECK(shotAgrees(off, hole.x, hole.y, hidden, 2, "sky through the cut part, no pre-pass"));
    CHECK(differsFromClear(on, hole.x, hole.y));
    const Rgb boxed = pixelOf(on, column(-3.0f, 5.4f), row(0.0f, 5.4f));
    CHECK(boxed.r > 60 && boxed.g < 10 && boxed.b < 10);
    CHECK(on.stats.depthPrePassSurfaces == 2);
    nodes.release(c);
}

void testDecal(Context& c)
{
    setBackground(c, whiteAmbient());
    setLens(c, 60.0f, Math::Vec3(0.0f, 0.0f, 6.0f));
    Nodes nodes;

    const MeshId floor = createBoxMesh(*c.meshes, Math::Vec3(8.0f, 0.2f, 8.0f));
    const MaterialId white =
            createFlatMaterial(c, Math::Vec4(1.0f, 1.0f, 1.0f, 1.0f), MaterialShading::Lit);
    nodes.mesh(c, floor, white, Math::Vec3(0.0f, -1.5f, 0.0f));

    Pixmap stainPixels(16, 16, 4);
    for (std::uint32_t y = 0; y < 16; ++y)
        for (std::uint32_t x = 0; x < 16; ++x) stainPixels.set_pixel(x, y, 255, 0, 0, 255);
    const TextureId stain = c.textures->createFromPixmap(stainPixels, true, false, "stain");
    Decal3D* decal = nodes.add<Decal3D>(c, Math::Vec3(0.0f, -1.4f, 0.0f));
    decal->setSize(Math::Vec3(2.0f, 1.0f, 2.0f));
    decal->setTexture(DecalTexture::Albedo, stain);

    Shot off;
    const Shot on = both(c, "decal", &off);
    CHECK(on.stats.decalsDrawn == 1);
    CHECK(on.stats.depthPrePassSurfaces == 1);
    CHECK(countPixels(on, kRed, 6) > 30);
    CHECK(countPixels(off, kRed, 6) == countPixels(on, kRed, 6));
    nodes.release(c);
}

void testMultiMesh(Context& c)
{
    setBackground(c, whiteAmbient());
    setLens(c, 60.0f, Math::Vec3(0.0f, 0.0f, 6.0f));
    Nodes nodes;

    const MeshId box = createBoxMesh(*c.meshes, Math::Vec3(1.0f, 1.0f, 1.0f));
    const MeshId quad = createQuad(c, 1.0f);
    const MaterialId white =
            createFlatMaterial(c, Math::Vec4(1.0f, 1.0f, 1.0f, 1.0f), MaterialShading::Lit);
    const MaterialId cutout =
            createCutoutMaterial(c, createCutout(c), Math::Vec4(0.0f, 1.0f, 0.0f, 1.0f));

    MultiMeshInstance3D* boxes = nodes.add<MultiMeshInstance3D>(c, Math::Vec3(0.0f, 0.0f, 0.0f));
    CHECK(boxes->setUseColors(true));
    boxes->setMesh(box);
    boxes->setInstanceCount(4);
    boxes->setMaterialOverride(white);
    boxes->setInstanceTransform(0, Math::Mat4::Translation(Math::Vec3(-2.0f, 1.2f, 0.0f)));
    boxes->setInstanceTransform(1, Math::Mat4::Translation(Math::Vec3(-1.4f, 1.2f, -1.0f)));
    boxes->setInstanceTransform(2, Math::Mat4::Translation(Math::Vec3(2.0f, 1.2f, 0.0f)));
    boxes->setInstanceTransform(3, Math::Mat4::Translation(Math::Vec3(0.0f, 1.2f, -2.0f)));
    boxes->setInstanceColor(0, Math::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
    boxes->setInstanceColor(1, Math::Vec4(0.0f, 0.0f, 1.0f, 1.0f));
    boxes->setInstanceColor(2, Math::Vec4(0.0f, 1.0f, 0.0f, 1.0f));
    boxes->setInstanceColor(3, Math::Vec4(1.0f, 1.0f, 0.0f, 1.0f));

    MultiMeshInstance3D* cards = nodes.add<MultiMeshInstance3D>(c, Math::Vec3(0.0f, 0.0f, 0.0f));
    cards->setMesh(quad);
    cards->setInstanceCount(2);
    cards->setMaterialOverride(cutout);
    cards->setInstanceTransform(0, Math::Mat4::Translation(Math::Vec3(-1.5f, -1.4f, 0.0f)));
    cards->setInstanceTransform(1, Math::Mat4::Translation(Math::Vec3(1.5f, -1.4f, 0.0f)));

    const MeshId behindMesh = createBoxMesh(*c.meshes, Math::Vec3(0.6f, 0.6f, 0.6f));
    const MaterialId blue =
            createFlatMaterial(c, Math::Vec4(0.0f, 0.0f, 1.0f, 1.0f), MaterialShading::Lit);
    nodes.mesh(c, behindMesh, blue, Math::Vec3(2.57f, -1.4f, -2.0f));

    Shot off;
    const Shot on = both(c, "multimesh", &off);
    CHECK(on.stats.depthPrePassSurfaces == 3);
    CHECK(on.stats.depthPrePassDrawCalls == 3);
    CHECK(on.stats.depthPrePassTriangles == 4 * 12 + 2 * 2 + 12);
    CHECK(on.stats.multimeshInstancesDrawn == 6);
    CHECK(shotIs(on, column(-2.0f, 5.5f), row(1.2f, 5.5f), kRed, 3, "multimesh box, red"));
    CHECK(shotIs(on, column(2.0f, 5.5f), row(1.2f, 5.5f), kGreen, 3, "multimesh box, green"));
    CHECK(shotIs(on, column(-1.5f - 0.5f, 6.0f), row(-1.4f, 6.0f), kGreen, 3,
            "multimesh card, solid part"));
    CHECK(shotIs(on, column(-1.5f + 0.25f, 6.0f), row(-1.4f, 6.0f), kClear, 3,
            "multimesh card, cut part"));
    CHECK(shotIs(on, column(2.0f, 6.0f), row(-1.4f, 7.7f), kBlue, 3,
            "multimesh card, cut part over the box"));
    nodes.release(c);
}

enum class LightKind
{
    Sun,
    Spot,
    OmniCube,
    OmniDual
};

struct ShadowGeometry
{
    float casterZ;
    float casterHalf;
    float solidX;
    float holeX;
};

ShadowGeometry shadowGeometry(LightKind kind)
{
    const float floorZ = -0.9f;
    ShadowGeometry g;
    if (kind == LightKind::Sun)
    {
        g.casterZ = 1.5f;
        g.casterHalf = 0.4f;
        const float shift = g.casterZ - floorZ;
        g.solidX = -g.casterHalf * 0.5f - shift;
        g.holeX = g.casterHalf * 0.5f - shift;
    }
    else
    {
        g.casterZ = 0.3f;
        g.casterHalf = 0.6f;
        const float lampX = -2.5f;
        const float lampZ = 1.5f;
        const float scale = (lampZ - floorZ) / (lampZ - g.casterZ);
        g.solidX = lampX + (-g.casterHalf * 0.5f - lampX) * scale;
        g.holeX = lampX + (g.casterHalf * 0.5f - lampX) * scale;
    }
    return g;
}

Node3D* createLight(Context& c, Nodes& nodes, LightKind kind)
{
    const Math::Vec3 lamp(-2.5f, 0.0f, 1.5f);
    const Math::Vec3 target(2.5f, 0.0f, -0.9f);
    if (kind == LightKind::Sun)
    {
        DirectionalLight3D* sun = nodes.add<DirectionalLight3D>(c, Math::Vec3(0.0f, 0.0f, 0.0f));
        sun->setRotation(Math::Vec3(0.0f, kPi / 4.0f, 0.0f));
        sun->setShadow(true);
        return sun;
    }
    if (kind == LightKind::Spot)
    {
        SpotLight3D* spot = nodes.add<SpotLight3D>(c, lamp);
        spot->setRange(10.0f);
        spot->setSpotAngle(40.0f);
        spot->lookAtFromPosition(lamp, target);
        spot->setShadow(true);
        return spot;
    }
    OmniLight3D* bulb = nodes.add<OmniLight3D>(c, lamp);
    bulb->setRange(10.0f);
    bulb->setShadowMode(
            kind == LightKind::OmniCube ? OmniShadowMode::Cube : OmniShadowMode::DualParaboloid);
    bulb->setShadow(true);
    return bulb;
}

bool shadowSpan(const Shot& lit, const Shot& shadowed, int y, int& first, int& last)
{
    first = -1;
    last = -1;
    for (int x = 0; x < kWidth; ++x)
    {
        const Rgb a = pixelOf(lit, x, y);
        const Rgb b = pixelOf(shadowed, x, y);
        const int drop = a.r - b.r > a.g - b.g ? (a.r - b.r > a.b - b.b ? a.r - b.r : a.b - b.b)
                                               : (a.g - b.g > a.b - b.b ? a.g - b.g : a.b - b.b);
        if (drop > 20)
        {
            if (first < 0) first = x;
            last = x;
        }
    }
    return first >= 0 && last - first >= 12;
}

void setCasterMaterial(VisualInstance3D* caster, bool multimesh, MaterialId material)
{
    if (multimesh) static_cast<MultiMeshInstance3D*>(caster)->setMaterialOverride(material);
    else
        static_cast<MeshInstance3D*>(caster)->setMaterialOverride(material);
}

void testShadowHole(Context& c, LightKind kind, bool multimesh, const char* label)
{
    setBackground(c, darkEnvironment());
    setLens(c, 75.0f, Math::Vec3(0.0f, 0.0f, 3.0f));
    Nodes nodes;
    Main::rendering()->setDirectionalShadowSize(1024, false);
    Main::rendering()->setDirectionalShadowQuality(ShadowQuality::Hard);
    Main::rendering()->setPositionalShadowQuality(ShadowQuality::Hard);
    c.window->setPositionalShadowAtlasSize(1024, false);

    const ShadowGeometry g = shadowGeometry(kind);
    const MaterialId matte = c.materials->create("matte");
    c.materials->setFloat(matte, MaterialParam::Specular, 0.0f);
    c.materials->setFloat(matte, MaterialParam::Roughness, 1.0f);
    const MeshId floorMesh = createBoxMesh(*c.meshes, Math::Vec3(8.0f, 8.0f, 0.2f));
    MeshInstance3D* floor = nodes.mesh(c, floorMesh, matte, Math::Vec3(0.0f, 0.0f, -1.0f));
    floor->setCastShadows(false);

    const MeshId quad = createQuad(c, g.casterHalf);
    const MaterialId cutout =
            createCutoutMaterial(c, createCutout(c), Math::Vec4(0.0f, 1.0f, 0.0f, 1.0f));
    const MaterialId opaque =
            createFlatMaterial(c, Math::Vec4(0.0f, 1.0f, 0.0f, 1.0f), MaterialShading::Lit);
    VisualInstance3D* caster = nullptr;
    if (multimesh)
    {
        MultiMeshInstance3D* cards =
                nodes.add<MultiMeshInstance3D>(c, Math::Vec3(0.0f, 0.0f, 0.0f));
        cards->setMesh(quad);
        cards->setInstanceCount(1);
        cards->setInstanceTransform(0, Math::Mat4::Translation(Math::Vec3(0.0f, 0.0f, g.casterZ)));
        caster = cards;
    }
    else
        caster = nodes.mesh(c, quad, opaque, Math::Vec3(0.0f, 0.0f, g.casterZ));
    createLight(c, nodes, kind);

    const int centre = row(0.0f, 3.9f);

    setCasterMaterial(caster, multimesh, opaque);
    caster->setCastShadows(false);
    const Shot reference = both(c, label);
    CHECK(reference.stats.shadowCasters == 0);
    caster->setCastShadows(true);
    const Shot whole = both(c, label);
    CHECK(whole.stats.shadowCasters > 0);
    int first = -1;
    int last = -1;
    const bool found = shadowSpan(reference, whole, centre, first, last);
    if (!found) printf("%s: the shadow of the opaque quad was not found\n", label);
    CHECK(found);
    if (!found)
    {
        nodes.release(c);
        return;
    }
    const int dark = first + (last - first) / 4;
    const int lit = first + (last - first) * 3 / 4;
    CHECK(shotIs(whole, dark, centre, kBlack, 2, label));
    CHECK(shotIs(whole, lit, centre, kBlack, 2, label));

    setCasterMaterial(caster, multimesh, cutout);
    char text[160];
    Shot off;
    const Shot on = both(c, label, &off);
    CHECK(on.stats.shadowCasters == whole.stats.shadowCasters);
    CHECK(on.stats.shadowTriangles == whole.stats.shadowTriangles);
    snprintf(text, sizeof(text), "%s: floor under the solid part", label);
    CHECK(shotIs(on, dark, centre, kBlack, 2, text));
    CHECK(shotIs(off, dark, centre, kBlack, 2, text));
    snprintf(text, sizeof(text), "%s: floor under the cut part", label);
    CHECK(shotAgrees(on, lit, centre, reference, 6, text));
    CHECK(shotAgrees(off, lit, centre, reference, 6, text));
    const Rgb lightFloor = pixelOf(reference, lit, centre);
    CHECK(lightFloor.r > 20 || lightFloor.g > 20 || lightFloor.b > 20);
    Main::rendering()->setDirectionalShadowQuality(ShadowQuality::SoftLow);
    Main::rendering()->setPositionalShadowQuality(ShadowQuality::SoftLow);
    nodes.release(c);
}

void testShadowMesh(Context& c)
{
    setBackground(c, darkEnvironment());
    setLens(c, 75.0f, Math::Vec3(0.0f, 0.0f, 3.0f));
    Nodes nodes;
    Main::rendering()->setDirectionalShadowSize(1024, false);
    Main::rendering()->setDirectionalShadowQuality(ShadowQuality::Hard);

    const ShadowGeometry g = shadowGeometry(LightKind::Sun);
    const MaterialId matte = c.materials->create("matte");
    c.materials->setFloat(matte, MaterialParam::Specular, 0.0f);
    c.materials->setFloat(matte, MaterialParam::Roughness, 1.0f);
    const MeshId floorMesh = createBoxMesh(*c.meshes, Math::Vec3(8.0f, 8.0f, 0.2f));
    nodes.mesh(c, floorMesh, matte, Math::Vec3(0.0f, 0.0f, -1.0f))->setCastShadows(false);

    const MeshId quad = createQuad(c, g.casterHalf);
    const MeshId displaced = c.meshes->create("displaced quad");
    CHECK(c.meshes->addSurface(displaced, quadWithoutUvs(g.casterHalf, 0.0f, 40.0f)));
    const MeshId proxy = c.meshes->create("proxy quad");
    CHECK(c.meshes->addSurface(proxy, quadWithoutUvs(g.casterHalf, 0.0f, 0.0f)));
    const MeshId twin = createQuad(c, g.casterHalf);

    const MaterialId cutout =
            createCutoutMaterial(c, createCutout(c), Math::Vec4(0.0f, 1.0f, 0.0f, 1.0f));
    const MaterialId opaque =
            createFlatMaterial(c, Math::Vec4(0.0f, 1.0f, 0.0f, 1.0f), MaterialShading::Lit);
    MeshInstance3D* caster = nodes.mesh(c, quad, opaque, Math::Vec3(0.0f, 0.0f, g.casterZ));
    createLight(c, nodes, LightKind::Sun);
    const int centre = row(0.0f, 3.9f);

    caster->setCastShadows(false);
    const Shot reference = both(c, "shadow mesh, reference");
    caster->setCastShadows(true);
    const Shot plain = both(c, "shadow mesh, opaque without it");
    int first = -1;
    int last = -1;
    CHECK(shadowSpan(reference, plain, centre, first, last));
    const int dark = first + (last - first) / 4;
    const int lit = first + (last - first) * 3 / 4;
    CHECK(shotIs(plain, dark, centre, kBlack, 2, "opaque quad, left of the shadow"));
    CHECK(shotIs(plain, lit, centre, kBlack, 2, "opaque quad, right of the shadow"));

    CHECK(c.meshes->setShadowMesh(quad, displaced));
    const Shot substituted = both(c, "shadow mesh, opaque with it");
    CHECK(shotAgrees(substituted, dark, centre, reference, 6, "shadow mesh used, left"));
    CHECK(shotAgrees(substituted, lit, centre, reference, 6, "shadow mesh used, right"));

    caster->setMaterialOverride(cutout);
    const Shot scissored = both(c, "shadow mesh without uvs, alpha scissor");
    CHECK(shotAgrees(scissored, dark, centre, reference, 6, "opaque proxy used, solid part"));
    CHECK(shotAgrees(scissored, lit, centre, reference, 6, "opaque proxy used, cut part"));

    CHECK(c.meshes->setShadowMesh(quad, proxy));
    const Shot proxied = both(c, "shadow mesh without uvs in place, alpha scissor");
    CHECK(shotIs(proxied, dark, centre, kBlack, 2, "opaque proxy in place, solid part"));
    CHECK(shotIs(proxied, lit, centre, kBlack, 2, "opaque proxy in place, cut part"));

    CHECK(c.meshes->setShadowMesh(quad, twin));
    const Shot twinned = both(c, "shadow mesh with uvs, alpha scissor");
    CHECK(shotIs(twinned, dark, centre, kBlack, 2, "shadow mesh with uvs, solid part"));
    CHECK(shotAgrees(twinned, lit, centre, reference, 6, "shadow mesh with uvs, cut part"));

    CHECK(c.meshes->setShadowMesh(quad, MeshId()));
    Main::rendering()->setDirectionalShadowQuality(ShadowQuality::SoftLow);
    nodes.release(c);
}

void testScissorAlpha(Context& c)
{
    setBackground(c, darkEnvironment());
    setLens(c, 75.0f, Math::Vec3(0.0f, 0.0f, 3.0f));
    Nodes nodes;
    Main::rendering()->setDirectionalShadowSize(1024, false);

    const MaterialId matte = c.materials->create("matte");
    c.materials->setFloat(matte, MaterialParam::Specular, 0.0f);
    c.materials->setFloat(matte, MaterialParam::Roughness, 1.0f);
    const MeshId floorMesh = createBoxMesh(*c.meshes, Math::Vec3(8.0f, 8.0f, 0.2f));
    nodes.mesh(c, floorMesh, matte, Math::Vec3(0.0f, 0.0f, -1.0f))->setCastShadows(false);
    const MeshId boxMesh = createBoxMesh(*c.meshes, Math::Vec3(1.0f, 1.0f, 1.0f));
    const MaterialId red =
            createFlatMaterial(c, Math::Vec4(1.0f, 0.0f, 0.0f, 1.0f), MaterialShading::Lit);
    c.materials->setFloat(red, MaterialParam::Specular, 0.0f);
    nodes.mesh(c, boxMesh, red, Math::Vec3(0.0f, 0.6f, 0.0f))->setCastShadows(false);

    const MeshId quad = createQuad(c, 0.25f);
    const MaterialId faint =
            createFlatMaterial(c, Math::Vec4(0.0f, 1.0f, 0.0f, 0.3f), MaterialShading::Lit);
    c.materials->setTransparency(faint, MaterialTransparency::AlphaScissor);
    c.materials->setFloat(faint, MaterialParam::AlphaScissorThreshold, 0.5f);
    const MaterialId solid =
            createFlatMaterial(c, Math::Vec4(0.0f, 1.0f, 0.0f, 1.0f), MaterialShading::Lit);
    c.materials->setTransparency(solid, MaterialTransparency::AlphaScissor);
    c.materials->setFloat(solid, MaterialParam::AlphaScissorThreshold, 0.5f);
    MeshInstance3D* byMaterial = nodes.mesh(c, quad, faint, Math::Vec3(-0.4f, 0.6f, 1.5f));
    MeshInstance3D* byInstance = nodes.mesh(c, quad, solid, Math::Vec3(0.4f, 0.6f, 1.5f));
    byInstance->setColor(Math::Vec4(1.0f, 1.0f, 1.0f, 0.3f));
    createLight(c, nodes, LightKind::Sun);

    byMaterial->hide();
    byInstance->hide();
    const Shot reference = both(c, "alpha sources, reference");
    byMaterial->show();
    byInstance->show();
    const Shot faded = both(c, "alpha sources, faded");
    CHECK(differences(faded, reference) == 0);
    CHECK(faded.stats.shadowCasters >= reference.stats.shadowCasters);
    CHECK(faded.stats.depthPrePassSurfaces == reference.stats.depthPrePassSurfaces + 2);

    byInstance->setColor(Math::Vec4(1.0f, 1.0f, 1.0f, 1.0f));
    const Shot shown = both(c, "alpha sources, instance opaque");
    CHECK(differences(shown, reference) > 0);
    byInstance->setCastShadows(false);
    const Shot unshadowed = both(c, "alpha sources, instance opaque without shadow");
    CHECK(differences(shown, unshadowed) > 0);
    byInstance->setCastShadows(true);
    byMaterial->setMaterialOverride(solid);
    const Shot twice = both(c, "alpha sources, both opaque");
    CHECK(differences(twice, shown) > 0);
    nodes.release(c);
}

void testToggle(Context& c)
{
    setBackground(c, whiteAmbient());
    setLens(c, 60.0f, Math::Vec3(0.0f, 0.0f, 6.0f));
    Nodes nodes;
    const MeshId box = createBoxMesh(*c.meshes, Math::Vec3(1.0f, 1.0f, 1.0f));
    const MaterialId red =
            createFlatMaterial(c, Math::Vec4(1.0f, 0.0f, 0.0f, 1.0f), MaterialShading::Lit);
    nodes.mesh(c, box, red, Math::Vec3(0.0f, 0.0f, 0.0f));
    nodes.mesh(c, box, red, Math::Vec3(0.0f, 0.0f, 0.0f))
            ->setPosition(Math::Vec3(0.0f, 0.0f, -2.0f));

    const Shot on = shoot(c, true);
    CHECK(on.stats.depthPrePassSurfaces == 2);
    CHECK(on.stats.depthPrePassTriangles == 24);
    CHECK(on.stats.depthPrePassDrawCalls >= 1 && on.stats.depthPrePassDrawCalls <= 2);
    CHECK(Main::rendering()->depthPrePassEnabled());
    const Shot off = shoot(c, false);
    CHECK(!Main::rendering()->depthPrePassEnabled());
    CHECK(off.stats.depthPrePassSurfaces == 0 && off.stats.depthPrePassDrawCalls == 0 &&
            off.stats.depthPrePassTriangles == 0);
    CHECK(differences(on, off) == 0);
    Main::rendering()->setDepthPrePassEnabled(true);
    nodes.release(c);
}

} // namespace

int main(int argc, char** argv)
{
    const bool vulkan = argc > 1 && strcmp(argv[1], "vulkan") == 0;

    CHECK(Device::headlessSupported());
    if (!Device::headlessSupported()) return 1;

    DeviceDesc desc;
    desc.title = "depth pre-pass test";
    desc.driver = vulkan ? prisma::DriverType::Vulkan : prisma::DriverType::OpenGL;
    desc.width = kWidth;
    desc.height = kHeight;
    desc.headless = true;

    CHECK(Main::setup(desc));
    if (!Main::device()) return 1;
    CHECK(Main::rendering()->depthPrePassEnabled());

    SceneTree tree;
    CHECK(Main::start(&tree));
    Node* root = tree.getRoot();
    Window* window = root ? root->castTo<Window>() : nullptr;
    CHECK(window != nullptr);
    if (!window) return 1;
    window->setClearColor(Math::Vec4(0.1f, 0.2f, 0.3f, 1.0f));

    Node3D holder;
    Context c;
    c.tree = &tree;
    c.root = root;
    c.holder = &holder;
    c.window = window;
    c.meshes = Main::meshes();
    c.textures = Main::textures();
    c.materials = Main::materials();
    c.environment = holder.createChild<WorldEnvironment>();
    c.environment->setEnvironment(whiteAmbient());
    c.environment->reparent(root);
    c.camera = holder.createChild<Camera3D>();
    c.camera->reparent(root);
    c.camera->makeCurrent();

    testToggle(c);
    testMaterials(c);
    testSky(c);
    testDecal(c);
    testMultiMesh(c);
    testScissorAlpha(c);

    const struct
    {
        LightKind kind;
        const char* name;
    } kinds[4] = { { LightKind::Sun, "sun" }, { LightKind::Spot, "spot" },
        { LightKind::OmniCube, "omni cube" }, { LightKind::OmniDual, "omni paraboloid" } };
    for (int k = 0; k < 4; ++k)
    {
        for (int m = 0; m < 2; ++m)
        {
            char label[64];
            snprintf(label, sizeof(label), "%s shadow, %s", kinds[k].name,
                    m == 0 ? "mesh" : "multimesh");
            testShadowHole(c, kinds[k].kind, m == 1, label);
        }
    }
    testShadowMesh(c);

    Main::device()->requestClose();
    tree.finalize();
    Main::cleanup();

    if (failures == 0) printf("all checks passed\n");
    return failures == 0 ? 0 : 1;
}
