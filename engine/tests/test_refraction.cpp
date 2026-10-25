#include "Check.h"
#include "LitTest.h"

#include "prisma3d/Device.h"
#include "prisma3d/main/Main.h"
#include "prisma3d/render/MaterialStore.h"
#include "prisma3d/render/MeshStore.h"
#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/render/TextureStore.h"
#include "prisma3d/scene/SceneTree.h"

#include <ct/vector.hpp>

#include <math.h>
#include <stdio.h>
#include <string.h>

using namespace prisma3d;

namespace
{

const int kSize = 128;
const float kGlassAngle = 35.0f * 3.14159265f / 180.0f;

struct Image
{
    ct::Vector<std::uint8_t> pixels;

    const std::uint8_t* at(int x, int y) const
    {
        return &pixels[(static_cast<std::size_t>(y) * kSize + static_cast<std::size_t>(x)) * 4];
    }

    bool red(int x, int y) const
    {
        const std::uint8_t* p = at(x, y);
        return p[0] > 150 && p[1] < 80 && p[2] < 80;
    }

    bool green(int x, int y) const
    {
        const std::uint8_t* p = at(x, y);
        return p[1] > 150 && p[0] < 80 && p[2] < 80;
    }
};

struct Rig
{
    RenderingServer* server = nullptr;
    prisma::Driver* driver = nullptr;
    TextureStore* textures = nullptr;
    MaterialStore* materials = nullptr;
    ScenarioId scenario;
    CameraId camera;
    ViewportId view;
    InstanceId glass;
    MaterialId glassMaterial;
    RenderStats stats;

    bool capture(Image& image)
    {
        for (int i = 0; i < 2; ++i)
        {
            server->renderViewports();
            server->endFrame(true);
        }
        server->renderViewports();
        stats = server->getStats();
        server->endFrame(true);
        image.pixels.resize(static_cast<std::size_t>(kSize) * kSize * 4);
        prisma::RenderTarget source;
        source.texture = textures->gpu(server->viewportGetTexture(view));
        prisma::Rect rect;
        rect.width = kSize;
        rect.height = kSize;
        return driver->readPixels(source, rect, image.pixels.data());
    }
};

SurfaceData quadData(float half)
{
    SurfaceData data;
    data.positions.push_back(Math::Vec3(-half, -half, 0.0f));
    data.positions.push_back(Math::Vec3(half, -half, 0.0f));
    data.positions.push_back(Math::Vec3(half, half, 0.0f));
    data.positions.push_back(Math::Vec3(-half, half, 0.0f));
    for (int i = 0; i < 4; ++i)
    {
        data.colors.push_back(Math::Vec4(1.0f, 1.0f, 1.0f, 1.0f));
        data.normals.push_back(Math::Vec3(0.0f, 0.0f, 1.0f));
    }
    data.uvs.push_back(Math::Vec2(0.0f, 0.0f));
    data.uvs.push_back(Math::Vec2(1.0f, 0.0f));
    data.uvs.push_back(Math::Vec2(1.0f, 1.0f));
    data.uvs.push_back(Math::Vec2(0.0f, 1.0f));
    const std::uint32_t indices[6] = { 0, 1, 2, 0, 2, 3 };
    for (std::uint32_t index: indices) data.indices.push_back(index);
    return data;
}

MaterialId emissive(MaterialStore& materials, const char* name, const Math::Vec4& colour)
{
    const MaterialId material = materials.create(name);
    materials.setColor(material, MaterialParam::Albedo, Math::Vec4(0.0f, 0.0f, 0.0f, 1.0f));
    materials.setColor(material, MaterialParam::Emission, colour);
    materials.setFloat(material, MaterialParam::EmissionEnergy, 1.0f);
    materials.setFloat(material, MaterialParam::Roughness, 1.0f);
    materials.setFloat(material, MaterialParam::Specular, 0.0f);
    return material;
}

void testRefraction(Rig& rig)
{
    const int x = kSize / 2 + 3;
    const int y = kSize / 2;

    rig.server->instanceSetVisible(rig.glass, false);
    Image plain;
    CHECK(rig.capture(plain));
    CHECK(plain.green(x, y));
    CHECK(plain.red(kSize / 2 - 4, y));
    CHECK(rig.stats.screenCopies == 0);
    const std::uint32_t plainDraws = rig.stats.drawCalls;
    rig.server->instanceSetVisible(rig.glass, true);

    rig.materials->setFloat(rig.glassMaterial, MaterialParam::Refraction, 0.08f);
    Image bent;
    CHECK(rig.capture(bent));
    CHECK(rig.stats.screenCopies == 1);
    CHECK(rig.stats.drawCalls > plainDraws);
    CHECK(bent.red(x, y));
    CHECK(bent.green(kSize / 2 + 12, y));
    CHECK(bent.red(kSize / 2 - 4, y));
    CHECK(bent.green(kSize / 2 + 50, y));

    rig.materials->setFloat(rig.glassMaterial, MaterialParam::Refraction, -0.08f);
    Image opposite;
    CHECK(rig.capture(opposite));
    CHECK(opposite.green(kSize / 2 - 3, y));
    CHECK(opposite.red(kSize / 2 - 12, y));

    rig.materials->setFloat(rig.glassMaterial, MaterialParam::Refraction, 0.0f);
    Image flat;
    CHECK(rig.capture(flat));
    CHECK(rig.stats.screenCopies == 0);
    CHECK(!flat.green(x, y) && !flat.red(x, y));

    rig.server->viewportSetMsaa3D(rig.view, ViewportMsaa::Msaa4x);
    rig.materials->setFloat(rig.glassMaterial, MaterialParam::Refraction, 0.08f);
    Image multisampled;
    CHECK(rig.capture(multisampled));
    CHECK(rig.stats.screenCopies == 1);
    CHECK(multisampled.red(x, y));
    CHECK(multisampled.green(kSize / 2 + 50, y));
    rig.server->viewportSetMsaa3D(rig.view, ViewportMsaa::Disabled);
    rig.materials->setFloat(rig.glassMaterial, MaterialParam::Refraction, 0.0f);
}

} // namespace

int main(int argc, char** argv)
{
    const bool vulkan = argc > 1 && strcmp(argv[1], "vulkan") == 0;

    CHECK(Device::headlessSupported());
    if (!Device::headlessSupported()) return 1;

    DeviceDesc desc;
    desc.title = "refraction test";
    desc.driver = vulkan ? prisma::DriverType::Vulkan : prisma::DriverType::OpenGL;
    desc.width = kSize;
    desc.height = kSize;
    desc.headless = true;

    CHECK(Main::setup(desc));
    if (!Main::device()) return 1;
    SceneTree tree;
    CHECK(Main::start(&tree));

    Rig rig;
    rig.server = Main::rendering();
    rig.driver = Main::device()->driver();
    rig.textures = Main::textures();
    rig.materials = Main::materials();

    RenderingServer& server = *rig.server;
    MeshStore& meshes = *Main::meshes();
    rig.scenario = server.scenarioCreate();
    server.scenarioSetEnvironment(rig.scenario, whiteAmbient());

    const MeshId wall = meshes.create("refraction wall");
    CHECK(meshes.addSurface(wall, quadData(10.0f)));
    const InstanceId left = server.instanceCreate(wall, rig.scenario);
    const InstanceId right = server.instanceCreate(wall, rig.scenario);
    server.instanceSetTransform(left, Math::Mat4::Translation(Math::Vec3(-10.0f, 0.0f, -5.0f)));
    server.instanceSetTransform(right, Math::Mat4::Translation(Math::Vec3(10.0f, 0.0f, -5.0f)));
    server.instanceSetMaterialOverride(left,
            emissive(*rig.materials, "red", Math::Vec4(1.0f, 0.0f, 0.0f, 1.0f)));
    server.instanceSetMaterialOverride(right,
            emissive(*rig.materials, "green", Math::Vec4(0.0f, 1.0f, 0.0f, 1.0f)));

    const MeshId pane = meshes.create("refraction glass");
    CHECK(meshes.addSurface(pane, quadData(3.0f)));
    rig.glass = server.instanceCreate(pane, rig.scenario);
    server.instanceSetTransform(rig.glass, Math::Mat4::Translation(Math::Vec3(0.0f, 0.0f, 2.0f)) *
                                                   Math::Mat4::RotationY(kGlassAngle));
    rig.glassMaterial = rig.materials->create("glass");
    rig.materials->setColor(rig.glassMaterial, MaterialParam::Albedo,
            Math::Vec4(1.0f, 1.0f, 1.0f, 0.0f));
    rig.materials->setFloat(rig.glassMaterial, MaterialParam::Roughness, 0.0f);
    rig.materials->setFloat(rig.glassMaterial, MaterialParam::Specular, 0.0f);
    rig.materials->setCull(rig.glassMaterial, MaterialCull::Disabled);
    server.instanceSetMaterialOverride(rig.glass, rig.glassMaterial);

    rig.camera = server.cameraCreate();
    server.cameraSetPerspective(rig.camera, 60.0f, 0.1f, 100.0f);
    server.cameraSetTransform(rig.camera, Math::Mat4::Translation(Math::Vec3(0.0f, 0.0f, 10.0f)));

    rig.view = server.viewportCreate();
    server.viewportSetSize(rig.view, kSize, kSize);
    server.viewportAttachCamera(rig.view, rig.camera);
    server.viewportSetScenario(rig.view, rig.scenario);
    server.viewportSetClearColor(rig.view, Math::Vec4(0.0f, 0.0f, 0.0f, 1.0f));
    server.viewportSetUpdateMode(rig.view, ViewportUpdateMode::Always);
    CHECK(server.viewportGetTexture(rig.view).valid());

    testRefraction(rig);

    Main::device()->requestClose();
    tree.finalize();
    Main::cleanup();

    if (failures) printf("%d check(s) failed\n", failures);
    else
        printf("all checks passed\n");
    return failures ? 1 : 0;
}
