#include "Check.h"
#include "LitTest.h"

#include "prisma3d/Device.h"
#include "prisma3d/main/Engine.h"
#include "prisma3d/main/Main.h"
#include "prisma3d/render/MeshStore.h"
#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/scene/Camera3D.h"
#include "prisma3d/scene/Light3D.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/Viewport.h"
#include "prisma3d/scene/VisualInstance3D.h"

#include <ct/vector.hpp>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

using namespace prisma3d;

namespace
{

const int kWidth = 320;
const int kHeight = 240;
const int kTolerance = 3;
const int kCells = 48;
const std::uint32_t kFieldTriangles = kCells * kCells * 2;
const float kPi = 3.14159265358979323846f;

struct Rgb
{
    int r;
    int g;
    int b;
};

const Rgb kYellow = { 255, 255, 0 };

bool pixelNear(int x, int y, Rgb expected, const char* label)
{
    unsigned char p[4] = { 0, 0, 0, 0 };
    prisma::Rect rect;
    rect.x = x;
    rect.y = y;
    rect.width = 1;
    rect.height = 1;
    if (!Main::device()->driver()->readPixels(prisma::RenderTarget(), rect, p)) return false;
    const int dr = p[0] - expected.r;
    const int dg = p[1] - expected.g;
    const int db = p[2] - expected.b;
    const bool match = dr >= -kTolerance && dr <= kTolerance && dg >= -kTolerance &&
                       dg <= kTolerance && db >= -kTolerance && db <= kTolerance;
    if (!match)
        printf("%s: pixel (%d,%d) is %d,%d,%d, expected %d,%d,%d\n", label, x, y, p[0], p[1], p[2],
                expected.r, expected.g, expected.b);
    return match;
}

RenderStats draw(SceneTree& tree)
{
    tree.process(1.0 / 60.0);
    Main::rendering()->renderViewports();
    const RenderStats stats = Main::rendering()->getStats();
    Main::rendering()->endFrame(true);
    return stats;
}

SurfaceData makeField(float depth)
{
    SurfaceData field;
    for (int y = 0; y <= kCells; ++y)
    {
        for (int x = 0; x <= kCells; ++x)
        {
            const float fx = static_cast<float>(x) * 0.25f;
            const float fy = static_cast<float>(y) * 0.25f;
            field.positions.push_back(Math::Vec3(fx - 6.0f, fy - 6.0f,
                    depth + 0.6f * sinf(fx * 0.9f) * cosf(fy * 0.7f)));
            const float dx = 0.6f * 0.9f * cosf(fx * 0.9f) * cosf(fy * 0.7f);
            const float dy = -0.6f * 0.7f * sinf(fx * 0.9f) * sinf(fy * 0.7f);
            field.normals.push_back(Math::Vec3(-dx, -dy, 1.0f).Normalized());
        }
    }
    for (int y = 0; y < kCells; ++y)
    {
        for (int x = 0; x < kCells; ++x)
        {
            const std::uint32_t a = static_cast<std::uint32_t>(y * (kCells + 1) + x);
            const std::uint32_t c = a + static_cast<std::uint32_t>(kCells + 1);
            const std::uint32_t corners[6] = { a, a + 1, c + 1, a, c + 1, c };
            for (int i = 0; i < 6; ++i) field.indices.push_back(corners[i]);
        }
    }
    return field;
}

bool addFieldSurface(MeshStore& meshes, MeshId mesh, float depth)
{
    const SurfaceData field = makeField(depth);
    if (!meshes.addSurface(mesh, field)) return false;
    const std::uint32_t surface = static_cast<std::uint32_t>(meshes.get(mesh)->surfaces.size() - 1);
    ct::Vector<SurfaceLod> lods;
    if (!MeshStore::generateLods(field, 20.0f, lods)) return false;
    ct::Vector<std::uint8_t> packed;
    for (std::size_t l = 0; l < lods.size(); ++l)
    {
        MeshStore::packIndices(lods[l].indices, meshes.get(mesh)->surfaces[surface].indexFormat,
                packed);
        if (!meshes.addSurfaceLod(mesh, surface, lods[l].edgeLength, packed.data(),
                    static_cast<std::uint32_t>(lods[l].indices.size())))
            return false;
    }
    return lods.size() >= 3;
}

struct View
{
    Math::Vec3 camera;
    float fov = 75.0f;
    float thresholdPixels = 1.0f;
    float scaling = 1.0f;
};

std::uint32_t expectedLod(const SurfaceInfo& surface, const Math::Box& box, float modelScale,
        const View& view)
{
    if (!(view.thresholdPixels > 0.0f)) return 0;
    const float gx = fmaxf(0.0f, fmaxf(box.min.x - view.camera.x, view.camera.x - box.max.x));
    const float gy = fmaxf(0.0f, fmaxf(box.min.y - view.camera.y, view.camera.y - box.max.y));
    const float gz = fmaxf(0.0f, fmaxf(box.min.z - view.camera.z, view.camera.z - box.max.z));
    const float distance = sqrtf(gx * gx + gy * gy + gz * gz);
    const float aspect = static_cast<float>(kWidth) / static_cast<float>(kHeight);
    const float multiplier = 2.0f * aspect * tanf(view.fov * kPi / 360.0f) / view.scaling;
    const float threshold = view.thresholdPixels / static_cast<float>(kWidth);
    std::uint32_t current = 0;
    for (std::size_t i = 0; i < surface.lods.size(); ++i)
    {
        if (surface.lods[i].edgeLength * modelScale / (distance * multiplier) > threshold) break;
        current = static_cast<std::uint32_t>(i) + 1;
    }
    return current;
}

std::uint32_t lodTriangles(const SurfaceInfo& surface, std::uint32_t lod)
{
    return (lod == 0 ? surface.indexCount : surface.lods[lod - 1].indexCount) / 3;
}

} // namespace

int main(int argc, char** argv)
{
    const bool vulkan = argc > 1 && strcmp(argv[1], "vulkan") == 0;

    CHECK(Device::headlessSupported());
    if (!Device::headlessSupported()) return 1;

    DeviceDesc desc;
    desc.title = "mesh lod test";
    desc.driver = vulkan ? prisma::DriverType::Vulkan : prisma::DriverType::OpenGL;
    desc.width = kWidth;
    desc.height = kHeight;
    desc.headless = true;

    CHECK(Main::setup(desc));
    if (!Main::device()) return 1;
    CHECK(Main::rendering() != nullptr);
    if (!Main::rendering()) return 1;

    SceneTree tree;
    CHECK(Main::start(&tree));
    Node* root = tree.getRoot();
    CHECK(root != nullptr);
    if (!root) return 1;
    Window* window = root->castTo<Window>();
    CHECK(window != nullptr);
    if (!window) return 1;
    window->setClearColor(Math::Vec4(0.1f, 0.2f, 0.3f, 1.0f));
    RenderingServer::singleton()->scenarioSetEnvironment(window->getScenario(), whiteAmbient());

    RenderingServer* server = Main::rendering();
    MeshStore& meshes = *Main::meshes();
    CHECK(window->getMeshLodThreshold() == 1.0f);

    Camera3D* camera = root->createChild<Camera3D>();
    camera->makeCurrent();
    CHECK(camera->getFov() == 75.0f);

    const MeshId fieldMesh = meshes.create("field");
    CHECK(addFieldSurface(meshes, fieldMesh, 0.0f));
    const SurfaceInfo& surface = meshes.get(fieldMesh)->surfaces[0];
    CHECK(surface.indexCount == kFieldTriangles * 3);

    MeshInstance3D* field = root->createChild<MeshInstance3D>();
    field->setMesh(fieldMesh);
    field->setColor(Math::Vec4(1.0f, 1.0f, 0.0f, 1.0f));

    View view;
    const float distances[7] = { 1.5f, 3.0f, 10.0f, 30.0f, 100.0f, 300.0f, 1000.0f };
    std::uint32_t previousLod = 0;
    std::uint32_t farLod = 0;
    for (int i = 0; i < 7; ++i)
    {
        view.camera = Math::Vec3(0.0f, 0.0f, distances[i]);
        camera->setPosition(view.camera);
        const RenderStats stats = draw(tree);
        const std::uint32_t lod =
                expectedLod(surface, server->instanceGetAabb(field->getInstance()), 1.0f, view);
        if (stats.triangles != lodTriangles(surface, lod))
            printf("distance %.1f: drew %u triangles, expected level %u with %u\n", distances[i],
                    stats.triangles, lod, lodTriangles(surface, lod));
        CHECK(stats.triangles == lodTriangles(surface, lod));
        CHECK(stats.depthPrePassTriangles == stats.triangles);
        CHECK(stats.lodSurfaces == (lod > 0 ? 1u : 0u));
        CHECK(lod >= previousLod);
        if (i == 0) CHECK(lod == 0);
        previousLod = lod;
        farLod = lod;
    }
    CHECK(farLod >= 3);

    view.camera = Math::Vec3(0.0f, 0.0f, 100.0f);
    camera->setPosition(view.camera);
    {
        tree.process(1.0 / 60.0);
        server->renderViewports();
        CHECK(pixelNear(160, 120, kYellow, "centre of the field at a coarse level"));
        server->endFrame(true);
    }

    view.camera = Math::Vec3(0.0f, 0.0f, 10.0f);
    camera->setPosition(view.camera);
    draw(tree);
    const Math::Box unscaledBox = server->instanceGetAabb(field->getInstance());
    const std::uint32_t baseLod = expectedLod(surface, unscaledBox, 1.0f, view);
    CHECK(baseLod >= 2);

    window->setMeshLodThreshold(0.0f);
    CHECK(window->getMeshLodThreshold() == 0.0f);
    RenderStats stats = draw(tree);
    CHECK(stats.triangles == kFieldTriangles);
    CHECK(stats.lodSurfaces == 0);

    window->setMeshLodThreshold(4.0f);
    view.thresholdPixels = 4.0f;
    stats = draw(tree);
    const std::uint32_t coarseLod =
            expectedLod(surface, server->instanceGetAabb(field->getInstance()), 1.0f, view);
    CHECK(coarseLod > baseLod);
    CHECK(stats.triangles == lodTriangles(surface, coarseLod));
    window->setMeshLodThreshold(1.0f);
    view.thresholdPixels = 1.0f;

    field->setScale(Math::Vec3(4.0f, 4.0f, 4.0f));
    stats = draw(tree);
    const std::uint32_t scaledLod =
            expectedLod(surface, server->instanceGetAabb(field->getInstance()), 4.0f, view);
    CHECK(scaledLod < baseLod);
    CHECK(stats.triangles == lodTriangles(surface, scaledLod));
    field->setScale(Math::Vec3(1.0f, 1.0f, 1.0f));

    server->instanceSetLodBias(field->getInstance(), 4.0f);
    stats = draw(tree);
    const std::uint32_t biasedLod = expectedLod(surface, unscaledBox, 4.0f, view);
    CHECK(biasedLod < baseLod);
    CHECK(stats.triangles == lodTriangles(surface, biasedLod));
    server->instanceSetLodBias(field->getInstance(), 1.0f);

    window->setScaling3D(0.5f);
    view.scaling = 0.5f;
    stats = draw(tree);
    const std::uint32_t halfLod =
            expectedLod(surface, server->instanceGetAabb(field->getInstance()), 1.0f, view);
    CHECK(halfLod > baseLod);
    CHECK(stats.triangles == lodTriangles(surface, halfLod));
    window->setScaling3D(1.0f);
    view.scaling = 1.0f;

    {
        server->setCullingTreeEnabled(false);
        const RenderStats linear = draw(tree);
        server->setCullingTreeEnabled(true);
        const RenderStats withTree = draw(tree);
        CHECK(linear.triangles == lodTriangles(surface, baseLod));
        CHECK(withTree.triangles == linear.triangles);
        CHECK(withTree.lodSurfaces == linear.lodSurfaces);
    }

    {
        MeshInstance3D* twin = root->createChild<MeshInstance3D>();
        twin->setMesh(fieldMesh);
        twin->setColor(Math::Vec4(1.0f, 1.0f, 0.0f, 1.0f));
        view.camera = Math::Vec3(0.0f, 0.0f, 100.0f);
        camera->setPosition(view.camera);
        field->setPosition(Math::Vec3(-20.0f, 0.0f, 0.0f));
        twin->setPosition(Math::Vec3(20.0f, 0.0f, 0.0f));
        stats = draw(tree);
        const std::uint32_t sideLod =
                expectedLod(surface, server->instanceGetAabb(field->getInstance()), 1.0f, view);
        CHECK(sideLod ==
                expectedLod(surface, server->instanceGetAabb(twin->getInstance()), 1.0f, view));
        CHECK(stats.triangles == 2 * lodTriangles(surface, sideLod));
        CHECK(stats.drawCalls == 1);
        CHECK(stats.lodSurfaces == 2);

        twin->setPosition(Math::Vec3(8.0f, 0.0f, 60.0f));
        stats = draw(tree);
        const std::uint32_t nearLod =
                expectedLod(surface, server->instanceGetAabb(twin->getInstance()), 1.0f, view);
        CHECK(nearLod < sideLod);
        CHECK(stats.triangles == lodTriangles(surface, sideLod) + lodTriangles(surface, nearLod));
        CHECK(stats.drawCalls == 2);
        twin->queueFree();
        field->setPosition(Math::Vec3(0.0f, 0.0f, 0.0f));
        draw(tree);
    }

    {
        const MeshId pairMesh = meshes.create("pair");
        CHECK(addFieldSurface(meshes, pairMesh, 0.0f));
        CHECK(addFieldSurface(meshes, pairMesh, -400.0f));
        MeshInstance3D* pair = root->createChild<MeshInstance3D>();
        pair->setMesh(pairMesh);
        field->hide();
        view.camera = Math::Vec3(0.0f, 0.0f, 3.0f);
        camera->setPosition(view.camera);
        stats = draw(tree);
        const MeshInfo* info = meshes.get(pairMesh);
        const Math::Box frontBox = info->surfaces[0].aabb;
        const Math::Box backBox = info->surfaces[1].aabb;
        const std::uint32_t frontLod = expectedLod(info->surfaces[0], frontBox, 1.0f, view);
        const std::uint32_t backLod = expectedLod(info->surfaces[1], backBox, 1.0f, view);
        CHECK(backLod > frontLod);
        CHECK(stats.surfacesDrawn == 2);
        CHECK(stats.triangles == lodTriangles(info->surfaces[0], frontLod) +
                                         lodTriangles(info->surfaces[1], backLod));
        pair->queueFree();
        field->show();
        draw(tree);
    }

    {
        DirectionalLight3D* sun = root->createChild<DirectionalLight3D>();
        sun->setRotation(Math::Vec3(-kPi / 4.0f, 0.0f, 0.0f));
        sun->setShadowMode(DirectionalShadowMode::Orthogonal);
        sun->setShadowMaxDistance(200.0f);
        sun->setShadow(true);
        view.camera = Math::Vec3(0.0f, 0.0f, 60.0f);
        camera->setPosition(view.camera);
        draw(tree);
        stats = draw(tree);
        const std::uint32_t shadowLod =
                expectedLod(surface, server->instanceGetAabb(field->getInstance()), 1.0f, view);
        CHECK(shadowLod >= 1);
        CHECK(stats.triangles == lodTriangles(surface, shadowLod));
        CHECK(stats.shadowPasses >= 1);
        if (stats.shadowTriangles != stats.shadowPasses * lodTriangles(surface, shadowLod))
            printf("shadows: %u triangles in %u passes, expected %u per pass\n",
                    stats.shadowTriangles, stats.shadowPasses, lodTriangles(surface, shadowLod));
        CHECK(stats.shadowTriangles == stats.shadowPasses * lodTriangles(surface, shadowLod));

        window->setMeshLodThreshold(0.0f);
        stats = draw(tree);
        CHECK(stats.shadowTriangles == stats.shadowPasses * kFieldTriangles);
        window->setMeshLodThreshold(1.0f);
        sun->queueFree();
        draw(tree);
    }

    tree.finalize();
    CHECK(tree.getRoot() == nullptr);
    Main::cleanup();
    CHECK(Main::device() == nullptr);

    if (failures != 0)
    {
        printf("%d check(s) failed\n", failures);
        return 1;
    }
    return 0;
}
