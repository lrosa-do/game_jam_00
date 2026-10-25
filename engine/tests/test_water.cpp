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
#include "prisma3d/scene/PrimitiveMeshes.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/Viewport.h"
#include "prisma3d/scene/VisualInstance3D.h"
#include "prisma3d/scene/Water3D.h"
#include "prisma3d/scene/WorldEnvironment.h"

#include <ct/vector.hpp>

#include <math.h>
#include <stdio.h>
#include <string.h>

using namespace prisma3d;

namespace
{

const int kSize = 128;
const float kFov = 60.0f;
const Math::Vec3 kEye(0.0f, 8.0f, 0.0f);

struct Image
{
    ct::Vector<std::uint8_t> pixels;

    const std::uint8_t* at(int x, int y) const
    {
        return &pixels[(static_cast<std::size_t>(y) * kSize + static_cast<std::size_t>(x)) * 4];
    }

    int differences(const Image& other) const
    {
        int count = 0;
        for (std::size_t i = 0; i < pixels.size(); i += 4)
        {
            int delta = 0;
            for (std::size_t c = 0; c < 3; ++c)
                delta += abs(
                        static_cast<int>(pixels[i + c]) - static_cast<int>(other.pixels[i + c]));
            if (delta > 24) ++count;
        }
        return count;
    }
};

Math::Mat4 topDownTransform(const Math::Vec3& eye)
{
    Math::Mat4 m = Math::Mat4::Identity();
    m.col0 = Math::Vec4(1.0f, 0.0f, 0.0f, 0.0f);
    m.col1 = Math::Vec4(0.0f, 0.0f, -1.0f, 0.0f);
    m.col2 = Math::Vec4(0.0f, 1.0f, 0.0f, 0.0f);
    m.col3 = Math::Vec4(eye, 1.0f);
    return m;
}

TextureId normalTexture(TextureStore& textures, const char* name, float x, float y)
{
    Pixmap pixmap(4, 4, 4);
    const std::uint8_t r = static_cast<std::uint8_t>((x * 0.5f + 0.5f) * 255.0f + 0.5f);
    const std::uint8_t g = static_cast<std::uint8_t>((y * 0.5f + 0.5f) * 255.0f + 0.5f);
    const float z = sqrtf(fmaxf(0.0f, 1.0f - x * x - y * y));
    const std::uint8_t b = static_cast<std::uint8_t>((z * 0.5f + 0.5f) * 255.0f + 0.5f);
    for (std::uint32_t py = 0; py < 4; ++py)
        for (std::uint32_t px = 0; px < 4; ++px) pixmap.set_pixel(px, py, r, g, b, 255);
    return textures.createFromPixmap(pixmap, false, false, name);
}

struct Rig
{
    RenderingServer* server = nullptr;
    prisma::Driver* driver = nullptr;
    TextureStore* textures = nullptr;
    MaterialStore* materials = nullptr;
    ScenarioId scenario;
    CameraId camera;
    ViewportId view;
    InstanceId floorLeft;
    InstanceId floorRight;
    InstanceId water;
    MaterialId waterMaterial;
    TextureId flatNormal;
    TextureId tiltedNormal;
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

    void floorAt(float height)
    {
        server->instanceSetTransform(floorLeft,
                Math::Mat4::Translation(Math::Vec3(-10.0f, height, 0.0f)));
        server->instanceSetTransform(floorRight,
                Math::Mat4::Translation(Math::Vec3(10.0f, height, 0.0f)));
    }
};

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

void testWater(Rig& rig)
{
    const int x = kSize / 2 - 12;
    const int y = kSize / 2;

    rig.server->instanceSetVisible(rig.water, false);
    Image plain;
    CHECK(rig.capture(plain));
    const std::uint8_t* plainPixel = plain.at(x, y);
    CHECK(plainPixel[0] > 150 && plainPixel[1] < 60);
    CHECK(rig.stats.screenCopies == 0);
    rig.server->instanceSetVisible(rig.water, true);

    rig.materials->setWater(rig.waterMaterial, false);
    Image glass;
    CHECK(rig.capture(glass));
    CHECK(rig.stats.screenCopies == 1);
    const std::uint8_t* glassPixel = glass.at(x, y);
    CHECK(glassPixel[0] > plainPixel[0] - 30);

    WaterMaterialData data;
    data.windSpeed = 0.0f;
    rig.materials->setWater(rig.waterMaterial, true, data);
    Image deep;
    CHECK(rig.capture(deep));
    CHECK(rig.stats.screenCopies == 1);
    const std::uint8_t* deepPixel = deep.at(x, y);
    CHECK(deepPixel[0] > plainPixel[0] / 2);
    CHECK(deepPixel[0] < plainPixel[0] * 4 / 5);
    CHECK(deepPixel[1] < 60);

    rig.floorAt(-0.02f);
    Image shore;
    CHECK(rig.capture(shore));
    const std::uint8_t* shorePixel = shore.at(x, y);
    CHECK(shorePixel[0] > deepPixel[0] + 20);
    rig.floorAt(-1.0f);

    data.depthFade = Math::Vec3(0.0f, 0.0f, 0.0f);
    rig.materials->setWater(rig.waterMaterial, true, data);
    Image clear;
    CHECK(rig.capture(clear));
    const std::uint8_t* clearPixel = clear.at(x, y);
    CHECK(clearPixel[0] > deepPixel[0] + 20);

    rig.materials->setTexture(rig.waterMaterial, MaterialTexture::Normal, rig.tiltedNormal);
    rig.materials->setFloat(rig.waterMaterial, MaterialParam::Refraction, 0.3f);
    Image tilted;
    CHECK(rig.capture(tilted));
    rig.materials->setWater(rig.waterMaterial, false);
    Image tiltedGlass;
    CHECK(rig.capture(tiltedGlass));
    CHECK(tilted.differences(tiltedGlass) > 100);
    rig.materials->setTexture(rig.waterMaterial, MaterialTexture::Normal, rig.flatNormal);
    rig.materials->setFloat(rig.waterMaterial, MaterialParam::Refraction, 0.001f);
}

bool shootWindow(Rig& rig, SceneTree& tree, Image& image)
{
    for (int i = 0; i < 3; ++i)
    {
        tree.process(1.0 / 60.0);
        rig.server->renderViewports();
        if (i == 2)
        {
            image.pixels.resize(static_cast<std::size_t>(kSize) * kSize * 4);
            prisma::Rect rect;
            rect.width = kSize;
            rect.height = kSize;
            if (!rig.driver->readPixels(prisma::RenderTarget(), rect, image.pixels.data()))
                return false;
            rig.stats = rig.server->getStats();
        }
        rig.server->endFrame(true);
    }
    return true;
}

void testNode(Rig& rig, SceneTree& tree)
{
    rig.server->instanceSetVisible(rig.water, false);
    rig.server->viewportSetUpdateMode(rig.view, ViewportUpdateMode::Disabled);

    Node* root = tree.getRoot();
    Window* window = root ? root->castTo<Window>() : nullptr;
    CHECK(window != nullptr);
    if (!window) return;
    window->setClearColor(Math::Vec4(0.0f, 0.0f, 0.0f, 1.0f));
    root->createChild<WorldEnvironment>()->setEnvironment(whiteAmbient());

    Camera3D* camera = root->createChild<Camera3D>();
    camera->setPerspective(kFov, 0.1f, 100.0f);
    camera->setTransform(topDownTransform(kEye));
    camera->makeCurrent();

    MeshStore& meshes = *Main::meshes();
    const MeshId floorMesh = createPlaneMesh(meshes, 20.0f, 40.0f);
    MeshInstance3D* left = root->createChild<MeshInstance3D>();
    left->setMesh(floorMesh);
    left->setPosition(Math::Vec3(-10.0f, -1.0f, 0.0f));
    left->setMaterialOverride(
            emissive(*rig.materials, "node red", Math::Vec4(1.0f, 0.0f, 0.0f, 1.0f)));
    MeshInstance3D* right = root->createChild<MeshInstance3D>();
    right->setMesh(floorMesh);
    right->setPosition(Math::Vec3(10.0f, -1.0f, 0.0f));
    right->setMaterialOverride(
            emissive(*rig.materials, "node green", Math::Vec4(0.0f, 1.0f, 0.0f, 1.0f)));

    Water3D* water = root->createChild<Water3D>();
    water->setSize(Math::Vec2(10.0f, 10.0f));
    water->setNormalTexture(rig.flatNormal);
    water->setColor(Math::Vec4(1.0f, 1.0f, 1.0f, 0.0f));
    water->setRoughness(0.0f);
    water->setSpecular(0.0f);
    water->setRefraction(0.001f);
    WaterMaterialData waves;
    waves.windSpeed = 0.0f;
    water->setWaves(waves);
    CHECK(water->getMaterial().valid());
    CHECK(water->getMesh().valid());
    CHECK(water->getPlanarReflector() == nullptr);
    CHECK(water->getReflection() == WaterReflection::Sky);

    const int x = kSize / 2 - 12;
    const int y = kSize / 2;
    Image deep;
    CHECK(shootWindow(rig, tree, deep));
    CHECK(rig.stats.screenCopies == 1);
    CHECK(rig.stats.reflectorsDrawn == 0);
    const std::uint8_t* deepPixel = deep.at(x, y);
    CHECK(deepPixel[0] > 128 && deepPixel[0] < 204);
    CHECK(deepPixel[1] < 60);

    water->setReflection(WaterReflection::Planar);
    CHECK(water->getPlanarReflector() != nullptr);
    Image mirrored;
    CHECK(shootWindow(rig, tree, mirrored));
    CHECK(rig.stats.reflectorsDrawn == 1);

    water->setReflection(WaterReflection::Sky);
    CHECK(water->getPlanarReflector() == nullptr);
    Image sky;
    CHECK(shootWindow(rig, tree, sky));
    CHECK(rig.stats.reflectorsDrawn == 0);

    water->setRefractionEnabled(false);
    Image opaque;
    CHECK(shootWindow(rig, tree, opaque));
    CHECK(rig.stats.screenCopies == 0);

    water->setRefractionEnabled(true);
    water->setUvTiling(0.05f);
    water->setSize(Math::Vec2(12.0f, 8.0f));
    Image resized;
    CHECK(shootWindow(rig, tree, resized));
    CHECK(rig.stats.screenCopies == 1);
    CHECK(water->getMesh().valid());
}

} // namespace

int main(int argc, char** argv)
{
    const bool vulkan = argc > 1 && strcmp(argv[1], "vulkan") == 0;

    CHECK(Device::headlessSupported());
    if (!Device::headlessSupported()) return 1;

    DeviceDesc desc;
    desc.title = "water test";
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

    const MeshId floorMesh = createPlaneMesh(meshes, 20.0f, 40.0f);
    rig.floorLeft = server.instanceCreate(floorMesh, rig.scenario);
    rig.floorRight = server.instanceCreate(floorMesh, rig.scenario);
    rig.floorAt(-1.0f);
    server.instanceSetMaterialOverride(rig.floorLeft,
            emissive(*rig.materials, "red floor", Math::Vec4(1.0f, 0.0f, 0.0f, 1.0f)));
    server.instanceSetMaterialOverride(rig.floorRight,
            emissive(*rig.materials, "green floor", Math::Vec4(0.0f, 1.0f, 0.0f, 1.0f)));

    rig.flatNormal = normalTexture(*rig.textures, "flat normal", 0.0f, 0.0f);
    rig.tiltedNormal = normalTexture(*rig.textures, "tilted normal", 0.6f, 0.4f);
    CHECK(rig.textures->valid(rig.flatNormal) && rig.textures->valid(rig.tiltedNormal));

    rig.water = server.instanceCreate(createPlaneMesh(meshes, 10.0f, 10.0f), rig.scenario);
    rig.waterMaterial = rig.materials->create("water");
    rig.materials->setColor(rig.waterMaterial, MaterialParam::Albedo,
            Math::Vec4(1.0f, 1.0f, 1.0f, 0.0f));
    rig.materials->setFloat(rig.waterMaterial, MaterialParam::Roughness, 0.0f);
    rig.materials->setFloat(rig.waterMaterial, MaterialParam::Specular, 0.0f);
    rig.materials->setFloat(rig.waterMaterial, MaterialParam::Refraction, 0.001f);
    rig.materials->setTexture(rig.waterMaterial, MaterialTexture::Normal, rig.flatNormal);
    server.instanceSetMaterialOverride(rig.water, rig.waterMaterial);

    rig.camera = server.cameraCreate();
    server.cameraSetPerspective(rig.camera, kFov, 0.1f, 100.0f);
    server.cameraSetTransform(rig.camera, topDownTransform(kEye));

    rig.view = server.viewportCreate();
    server.viewportSetSize(rig.view, kSize, kSize);
    server.viewportAttachCamera(rig.view, rig.camera);
    server.viewportSetScenario(rig.view, rig.scenario);
    server.viewportSetClearColor(rig.view, Math::Vec4(0.0f, 0.0f, 0.0f, 1.0f));
    server.viewportSetUpdateMode(rig.view, ViewportUpdateMode::Always);
    CHECK(server.viewportGetTexture(rig.view).valid());

    testWater(rig);
    testNode(rig, tree);

    Main::device()->requestClose();
    tree.finalize();
    Main::cleanup();

    if (failures) printf("%d check(s) failed\n", failures);
    else
        printf("all checks passed\n");
    return failures ? 1 : 0;
}
