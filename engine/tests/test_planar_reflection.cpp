#include "Check.h"
#include "LitTest.h"

#include "prisma3d/Device.h"
#include "prisma3d/main/Main.h"
#include "prisma3d/render/MaterialStore.h"
#include "prisma3d/render/MeshStore.h"
#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/render/TextureStore.h"
#include "prisma3d/scene/PrimitiveMeshes.h"
#include "prisma3d/scene/SceneTree.h"

#include <ct/vector.hpp>

#include <math.h>
#include <stdio.h>
#include <string.h>

using namespace prisma3d;

namespace
{

const int kSize = 128;
const float kFov = 60.0f;
const Math::Vec3 kEye(0.0f, 3.0f, 6.0f);
const Math::Vec3 kCube(0.0f, 2.0f, 2.0f);

struct Image
{
    ct::Vector<std::uint8_t> pixels;

    const std::uint8_t* at(int x, int y) const
    {
        return &pixels[(static_cast<std::size_t>(y) * kSize + static_cast<std::size_t>(x)) * 4];
    }

    bool reddish(int x, int y) const
    {
        int best = 0;
        int green = 255;
        for (int dy = -1; dy <= 1; ++dy)
            for (int dx = -1; dx <= 1; ++dx)
            {
                const std::uint8_t* p = at(x + dx, y + dy);
                if (p[0] > best)
                {
                    best = p[0];
                    green = p[1];
                }
            }
        return best > 100 && green < 70;
    }
};

Math::Mat4 lookTransform(const Math::Vec3& eye, const Math::Vec3& target)
{
    const Math::Vec3 forward = (target - eye).Normalized();
    const Math::Vec3 right = forward.Cross(Math::Vec3(0.0f, 1.0f, 0.0f)).Normalized();
    const Math::Vec3 up = right.Cross(forward);
    Math::Mat4 m = Math::Mat4::Identity();
    m.col0 = Math::Vec4(right, 0.0f);
    m.col1 = Math::Vec4(up, 0.0f);
    m.col2 = Math::Vec4(forward * -1.0f, 0.0f);
    m.col3 = Math::Vec4(eye, 1.0f);
    return m;
}

void project(const Math::Mat4& cameraTransform, const Math::Vec3& world, int& x, int& y)
{
    const Math::Mat4 view = cameraTransform.InverseAffine();
    const Math::Mat4 projection =
            Math::Mat4::Perspective(kFov * 3.14159265f / 180.0f, 1.0f, 0.1f, 100.0f);
    const Math::Vec4 clip = projection * view * Math::Vec4(world, 1.0f);
    x = static_cast<int>((clip.x / clip.w * 0.5f + 0.5f) * static_cast<float>(kSize));
    y = static_cast<int>((clip.y / clip.w * 0.5f + 0.5f) * static_cast<float>(kSize));
}

struct Rig
{
    RenderingServer* server = nullptr;
    prisma::Driver* driver = nullptr;
    TextureStore* textures = nullptr;
    ScenarioId scenario;
    CameraId camera;
    ViewportId view;
    InstanceId mirror;
    ReflectorId reflector;
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

void testMirror(Rig& rig)
{
    const Math::Mat4 cameraTransform = lookTransform(kEye, Math::Vec3(0.0f, 0.0f, 0.0f));
    const Math::Vec3 mirroredCube(kCube.x, -kCube.y, kCube.z);
    const float t = kEye.y / (kEye.y - mirroredCube.y);
    const Math::Vec3 hit = kEye + (mirroredCube - kEye) * t;
    int x = 0;
    int y = 0;
    project(cameraTransform, hit, x, y);
    CHECK(x > 8 && x < kSize - 8 && y > 8 && y < kSize - 8);
    int cubeX = 0;
    int cubeY = 0;
    project(cameraTransform, kCube, cubeX, cubeY);

    Image reflected;
    CHECK(rig.capture(reflected));
    CHECK(rig.stats.reflectorsDrawn == 1);
    CHECK(rig.stats.viewportsDrawn >= 2);
    CHECK(reflected.reddish(x, y));
    CHECK(reflected.reddish(cubeX, cubeY));
    CHECK(rig.textures->valid(rig.server->reflectorGetTexture(rig.reflector)));
    const std::uint8_t* corner = reflected.at(8, 8);
    CHECK(!(corner[0] > 100 && corner[1] < 70));

    rig.server->reflectorSetVisible(rig.reflector, false);
    Image plain;
    CHECK(rig.capture(plain));
    CHECK(rig.stats.reflectorsDrawn == 0);
    CHECK(!plain.reddish(x, y));
    CHECK(plain.reddish(cubeX, cubeY));
    rig.server->reflectorSetVisible(rig.reflector, true);

    rig.server->instanceSetReflector(rig.mirror, ReflectorId());
    Image detached;
    CHECK(rig.capture(detached));
    CHECK(rig.stats.reflectorsDrawn == 1);
    CHECK(!detached.reddish(x, y));
    rig.server->instanceSetReflector(rig.mirror, rig.reflector);

    rig.server->cameraSetTransform(rig.camera, lookTransform(kEye, Math::Vec3(0.0f, 3.0f, 20.0f)));
    Image away;
    CHECK(rig.capture(away));
    CHECK(rig.stats.reflectorsDrawn == 0);
    rig.server->cameraSetTransform(rig.camera, cameraTransform);

    rig.server->reflectorSetMaxDistance(rig.reflector, 0.001f);
    Image faded;
    CHECK(rig.capture(faded));
    CHECK(rig.stats.reflectorsDrawn == 1);
    CHECK(faded.reddish(x, y));
    rig.server->reflectorSetMaxDistance(rig.reflector, 10.0f);
}

} // namespace

int main(int argc, char** argv)
{
    const bool vulkan = argc > 1 && strcmp(argv[1], "vulkan") == 0;

    CHECK(Device::headlessSupported());
    if (!Device::headlessSupported()) return 1;

    DeviceDesc desc;
    desc.title = "planar reflection test";
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
    MaterialStore& materials = *Main::materials();

    RenderingServer& server = *rig.server;
    MeshStore& meshes = *Main::meshes();
    rig.scenario = server.scenarioCreate();
    server.scenarioSetEnvironment(rig.scenario, whiteAmbient());

    const MeshId plane = meshes.create("mirror plane");
    CHECK(meshes.addSurface(plane, quadData(4.0f)));
    rig.mirror = server.instanceCreate(plane, rig.scenario);
    const Math::Mat4 mirrorTransform = Math::Mat4::RotationX(-3.14159265f * 0.5f);
    server.instanceSetTransform(rig.mirror, mirrorTransform);
    const MaterialId mirrorMaterial = materials.create("mirror");
    materials.setColor(mirrorMaterial, MaterialParam::Albedo, Math::Vec4(1.0f, 1.0f, 1.0f, 1.0f));
    materials.setFloat(mirrorMaterial, MaterialParam::Metallic, 1.0f);
    materials.setFloat(mirrorMaterial, MaterialParam::Roughness, 0.05f);
    server.instanceSetMaterialOverride(rig.mirror, mirrorMaterial);

    rig.reflector = server.reflectorCreate(rig.scenario);
    server.reflectorSetTransform(rig.reflector, mirrorTransform);
    server.reflectorSetHalfSize(rig.reflector, Math::Vec2(4.0f, 4.0f));
    server.reflectorSetResolution(rig.reflector, 128);
    server.reflectorSetMaxDistance(rig.reflector, 10.0f);
    server.instanceSetReflector(rig.mirror, rig.reflector);

    const MeshId cubeMesh = createBoxMesh(meshes, Math::Vec3(1.0f, 1.0f, 1.0f));
    const InstanceId cube = server.instanceCreate(cubeMesh, rig.scenario);
    server.instanceSetTransform(cube, Math::Mat4::Translation(kCube));
    const MaterialId red = materials.create("red");
    materials.setColor(red, MaterialParam::Albedo, Math::Vec4(0.0f, 0.0f, 0.0f, 1.0f));
    materials.setColor(red, MaterialParam::Emission, Math::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
    materials.setFloat(red, MaterialParam::EmissionEnergy, 1.0f);
    server.instanceSetMaterialOverride(cube, red);

    rig.camera = server.cameraCreate();
    server.cameraSetPerspective(rig.camera, kFov, 0.1f, 100.0f);
    server.cameraSetTransform(rig.camera, lookTransform(kEye, Math::Vec3(0.0f, 0.0f, 0.0f)));

    rig.view = server.viewportCreate();
    server.viewportSetSize(rig.view, kSize, kSize);
    server.viewportAttachCamera(rig.view, rig.camera);
    server.viewportSetScenario(rig.view, rig.scenario);
    server.viewportSetClearColor(rig.view, Math::Vec4(0.0f, 0.0f, 0.0f, 1.0f));
    server.viewportSetUpdateMode(rig.view, ViewportUpdateMode::Always);
    CHECK(server.viewportGetTexture(rig.view).valid());

    testMirror(rig);

    Main::device()->requestClose();
    tree.finalize();
    Main::cleanup();

    if (failures) printf("%d check(s) failed\n", failures);
    else
        printf("all checks passed\n");
    return failures ? 1 : 0;
}
