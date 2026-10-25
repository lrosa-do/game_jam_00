#include "Check.h"
#include "LitTest.h"

#include "prisma3d/Device.h"
#include "prisma3d/main/Main.h"
#include "prisma3d/physics/PhysicsServer3D.h"
#include "prisma3d/render/MaterialStore.h"
#include "prisma3d/render/MeshStore.h"
#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/scene/Camera3D.h"
#include "prisma3d/scene/Light3D.h"
#include "prisma3d/scene/PrimitiveMeshes.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/SoftBody3D.h"
#include "prisma3d/scene/Viewport.h"
#include "prisma3d/scene/WorldEnvironment.h"

#include "platform.h"

#include <ct/vector.hpp>

#include <math.h>
#include <string.h>

using namespace prisma3d;

namespace
{

const int kSize = 64;
const double kStep = 1.0 / 60.0;

int gErrors = 0;

void onLog(LogLevel level, const char* message, void*)
{
    if (level < LOGLEVEL_ERROR) return;
    ++gErrors;
    printf("logged error: %s\n", message);
}

struct Picture
{
    ct::Vector<std::uint8_t> pixels;

    const std::uint8_t* at(int worldX, int worldY) const
    {
        const int column = worldX + kSize / 2;
        const int row = kSize / 2 - 1 - worldY;
        return &pixels[(static_cast<std::size_t>(row) * kSize + static_cast<std::size_t>(column)) *
                       4];
    }

    bool covered(int worldX, int worldY) const
    {
        const std::uint8_t* p = at(worldX, worldY);
        return p[0] < 200;
    }
};

struct Context
{
    SceneTree* tree = nullptr;
    Node* root = nullptr;
    Window* window = nullptr;
    Camera3D* camera = nullptr;
    MaterialId material;
};

void step(Context& c, int count)
{
    for (int i = 0; i < count; ++i) c.tree->physicsProcess(kStep);
}

Picture shoot(Context& c)
{
    for (int i = 0; i < 2; ++i)
    {
        c.tree->process(kStep);
        Main::rendering()->renderViewports();
        Main::rendering()->endFrame(true);
    }
    c.tree->process(kStep);
    Main::rendering()->renderViewports();
    Picture picture;
    picture.pixels.resize(static_cast<std::size_t>(kSize) * kSize * 4);
    prisma::Rect rect;
    rect.width = kSize;
    rect.height = kSize;
    CHECK(Main::device()->driver()->readPixels(prisma::RenderTarget(), rect,
            picture.pixels.data()));
    Main::rendering()->endFrame(true);
    return picture;
}

SoftBody3D* makeCloth(Context& c, const Math::Vec3& position)
{
    SoftBody3D* cloth = c.root->createChild<SoftBody3D>();
    cloth->makeCloth(20.0f, 20.0f, 10, 10);
    cloth->setRotationDegrees(Math::Vec3(90.0f, 0.0f, 0.0f));
    cloth->setPosition(position);
    cloth->setMaterialOverride(c.material);
    return cloth;
}

int differing(const Picture& a, const Picture& b, int tolerance)
{
    int count = 0;
    for (std::size_t i = 0; i < a.pixels.size(); ++i)
    {
        const int d = static_cast<int>(a.pixels[i]) - static_cast<int>(b.pixels[i]);
        if (d > tolerance || d < -tolerance) ++count;
    }
    return count;
}

void testMatchesStaticMesh(Context& c)
{
    printf("soft body against the same mesh drawn statically\n");
    Main::physics()->setGravity(Math::Vec3(0.0f, 0.0f, 0.0f));

    SoftBody3D* cloth = makeCloth(c, Math::Vec3(0.0f, 0.0f, 0.0f));
    cloth->pinInBox(Math::Box(Math::Vec3(-30.0f, -30.0f, -30.0f), Math::Vec3(30.0f, 30.0f, 30.0f)));
    step(c, 3);
    CHECK(cloth->isBuilt());
    const Picture soft = shoot(c);
    CHECK(soft.covered(0, 0));
    CHECK(soft.covered(9, 9));
    CHECK(soft.covered(-9, -9));
    CHECK(!soft.covered(12, 0));
    CHECK(!soft.covered(0, -12));
    CHECK(cloth->getMesh().valid());
    CHECK(cloth->getMesh() == cloth->getOwnedMesh());

    cloth->setVisible(false);
    SurfaceData data;
    planeMeshArrays(data, 20.0f, 20.0f, 10, 10);
    const MeshId reference = Main::meshes()->create("reference");
    CHECK(Main::meshes()->addSurface(reference, data));
    MeshInstance3D* flat = c.root->createChild<MeshInstance3D>();
    flat->setMesh(reference);
    flat->setMaterialOverride(c.material);
    flat->setRotationDegrees(Math::Vec3(90.0f, 0.0f, 0.0f));
    const Picture still = shoot(c);
    const int apart = differing(soft, still, 1);
    printf("  %d channel values differ\n", apart);
    CHECK(apart == 0);

    flat->queueFree();
    cloth->setVisible(true);
    cloth->setPosition(Math::Vec3(20.0f, 0.0f, 0.0f));
    step(c, 2);
    const Picture moved = shoot(c);
    CHECK(moved.covered(20, 0));
    CHECK(moved.covered(11, 9));
    CHECK(moved.covered(29, -9));
    CHECK(!moved.covered(0, 0));
    CHECK(!moved.covered(-9, 9));
    cloth->queueFree();
    step(c, 1);
}

void testHangingCorner(Context& c)
{
    printf("soft body hanging from one corner\n");
    Main::physics()->setGravity(Math::Vec3(0.0f, -9.8f, 0.0f));

    SoftBody3D* cloth = makeCloth(c, Math::Vec3(0.0f, 0.0f, 0.0f));
    PhysicsSoftBodyParameters parameters = cloth->getParameters();
    parameters.damping = 0.3f;
    parameters.bendCompliance = 0.01f;
    cloth->setParameters(parameters);
    cloth->pinInSphere(Math::Vec3(-10.0f, 0.0f, -10.0f), 0.5f);
    step(c, 420);
    const Picture hung = shoot(c);
    CHECK(hung.covered(-10, -10));
    CHECK(hung.covered(-20, -4));
    CHECK(hung.covered(-10, 8));
    CHECK(!hung.covered(10, 0));
    CHECK(!hung.covered(0, 5));
    CHECK(!hung.covered(-24, 9));
    cloth->queueFree();
    step(c, 1);
}

} // namespace

int main(int argc, char** argv)
{
    const bool vulkan = argc > 1 && strcmp(argv[1], "vulkan") == 0;

    log_set_callback(onLog, nullptr);
    CHECK(Device::headlessSupported());
    if (!Device::headlessSupported()) return 1;

    DeviceDesc desc;
    desc.title = "soft body render test";
    desc.driver = vulkan ? prisma::DriverType::Vulkan : prisma::DriverType::OpenGL;
    desc.width = kSize;
    desc.height = kSize;
    desc.headless = true;

    CHECK(Main::setup(desc));
    if (!Main::device()) return 1;

    SceneTree tree;
    CHECK(Main::start(&tree));
    Context c;
    c.tree = &tree;
    c.root = tree.getRoot();
    c.window = c.root ? c.root->castTo<Window>() : nullptr;
    CHECK(c.window != nullptr);
    if (!c.window) return 1;

    c.window->setClearColor(Math::Vec4(1.0f, 1.0f, 1.0f, 1.0f));
    WorldEnvironment* environment = c.root->createChild<WorldEnvironment>();
    EnvironmentSettings settings = whiteAmbient();
    settings.ambientColor = Math::Vec3(0.25f, 0.25f, 0.25f);
    environment->setEnvironment(settings);

    c.camera = c.root->createChild<Camera3D>();
    c.camera->makeCurrent();
    c.camera->setOrthogonal(static_cast<float>(kSize), 0.1f, 100.0f);
    c.camera->setPosition(Math::Vec3(0.0f, 0.0f, 10.0f));

    DirectionalLight3D* sun = c.root->createChild<DirectionalLight3D>();
    sun->lookAtFromPosition(Math::Vec3(4.0f, 6.0f, 10.0f), Math::Vec3(0.0f, 0.0f, 0.0f));

    c.material = Main::materials()->create("cloth");
    Main::materials()->setColor(c.material, MaterialParam::Albedo,
            Math::Vec4(0.2f, 0.2f, 0.2f, 1.0f));
    Main::materials()->setCull(c.material, MaterialCull::Disabled);

    testMatchesStaticMesh(c);
    testHangingCorner(c);

    CHECK(gErrors == 0);
    Main::device()->requestClose();
    tree.finalize();
    Main::cleanup();

    if (failures == 0) printf("all checks passed\n");
    return failures == 0 ? 0 : 1;
}
