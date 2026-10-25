#include "Check.h"
#include "LitTest.h"

#include "prisma3d/Device.h"
#include "prisma3d/main/Main.h"
#include "prisma3d/render/MaterialStore.h"
#include "prisma3d/render/MeshStore.h"
#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/scene/Animation.h"
#include "prisma3d/scene/Camera3D.h"
#include "prisma3d/scene/Hair3D.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/Skeleton3D.h"
#include "prisma3d/scene/Viewport.h"
#include "prisma3d/scene/WorldEnvironment.h"

#include "platform.h"

#include <ct/vector.hpp>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

using namespace prisma3d;

namespace
{

const int kSize = 64;
const double kStep = 1.0 / 60.0;
const float kScalpY = -16.0f;
const float kLength = 24.0f;

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
        return at(worldX, worldY)[0] < 200;
    }

    int coveredBetween(int yMin, int yMax) const
    {
        int count = 0;
        for (int y = yMin; y < yMax; ++y)
            for (int x = -kSize / 2; x < kSize / 2; ++x)
                if (covered(x, y)) ++count;
        return count;
    }

    int highestCovered() const
    {
        for (int y = kSize / 2 - 1; y >= -kSize / 2; --y)
            for (int x = -kSize / 2; x < kSize / 2; ++x)
                if (covered(x, y)) return y;
        return -kSize;
    }

    int leftmostCovered(int yMin, int yMax) const
    {
        for (int x = -kSize / 2; x < kSize / 2; ++x)
            for (int y = yMin; y < yMax; ++y)
                if (covered(x, y)) return x;
        return kSize;
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

Picture shoot(Context& c, int frames = 3)
{
    for (int i = 0; i < frames - 1; ++i)
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

SurfaceData scalpData(bool skinned)
{
    SurfaceData data;
    const float half = 16.0f;
    data.positions.push_back(Math::Vec3(-half, kScalpY, -2.0f));
    data.positions.push_back(Math::Vec3(half, kScalpY, -2.0f));
    data.positions.push_back(Math::Vec3(half, kScalpY, 2.0f));
    data.positions.push_back(Math::Vec3(-half, kScalpY, 2.0f));
    const Math::Vec3 normal = Math::Vec3(0.4f, 1.0f, 0.0f).Normalized();
    for (int i = 0; i < 4; ++i)
    {
        data.normals.push_back(normal);
        data.colors.push_back(Math::Vec4(1.0f, 1.0f, 1.0f, 1.0f));
        data.uvs.push_back(Math::Vec2(0.0f, 0.0f));
        if (skinned)
        {
            data.joints.push_back(0u);
            data.weights.push_back(Math::Vec4(1.0f, 0.0f, 0.0f, 0.0f));
        }
    }
    const std::uint32_t indices[6] = { 0, 1, 2, 0, 2, 3 };
    for (std::uint32_t index: indices) data.indices.push_back(index);
    return data;
}

void configure(Hair3D* hair, MaterialId material)
{
    HairParams params;
    params.strandCount = 300;
    params.segmentCount = 3;
    params.billboardCount = 1;
    params.randomSeed = 3;
    params.length = kLength;
    params.randomness = 0.0f;
    params.gravityPower = 0.0f;
    params.stiffness = 0.5f;
    params.width = 0.1f;
    params.viewDistance = 100.0f;
    params.cameraBend = false;
    hair->setParams(params);
    hair->setMaterial(material);
}

void testStrandsDraw(Context& c)
{
    Hair3D* hair = c.root->createChild<Hair3D>();
    configure(hair, c.material);
    hair->setScalp(scalpData(false));

    const Picture picture = shoot(c);
    CHECK(hair->getScalpTriangleCount() == 2);
    CHECK(hair->getStrands().vertexCount() == 300u * 8u);
    const int band = picture.coveredBetween(static_cast<int>(kScalpY) + 1, 0);
    CHECK(band > 100);
    CHECK(picture.coveredBetween(9, kSize / 2) == 0);
    CHECK(picture.highestCovered() >= 2 && picture.highestCovered() <= 7);
    const std::uint32_t drawsWithHair = Main::rendering()->getStats().drawCalls;
    const Math::Box bounds = hair->getAabb();
    CHECK(bounds.max.y > bounds.min.y + 10.0f);

    hair->setVisible(false);
    const Picture hidden = shoot(c);
    CHECK(hidden.coveredBetween(-kSize / 2, kSize / 2) == 0);
    CHECK(Main::rendering()->getStats().drawCalls < drawsWithHair);
    hair->setVisible(true);

    hair->setGravityPower(30.0f);
    const Picture bent = shoot(c, 120);
    CHECK(bent.highestCovered() <= picture.highestCovered() - 2);
    CHECK(bent.coveredBetween(static_cast<int>(kScalpY) - 10, 0) > 30);

    hair->queueFree();
    shoot(c);
}

void testSkinnedScalp(Context& c)
{
    Skeleton3D* skeletonNode = c.root->createChild<Skeleton3D>();
    Skeleton skeleton;
    skeleton.addBone(ct::String("root"), -1, Math::Mat4::Identity(), Math::Mat4::Identity());
    skeletonNode->setSkeleton(skeleton);

    Hair3D* hair = skeletonNode->createChild<Hair3D>();
    configure(hair, c.material);
    hair->setScalp(scalpData(true));
    CHECK(hair->getSkeleton() == skeletonNode);

    const Picture rest = shoot(c);
    const int restLeft = rest.leftmostCovered(static_cast<int>(kScalpY) + 1, 0);
    CHECK(restLeft < -8);

    skeletonNode->setBonePosePosition(0, Math::Vec3(12.0f, 0.0f, 0.0f));
    hair->reset();
    const Picture moved = shoot(c);
    const int movedLeft = moved.leftmostCovered(static_cast<int>(kScalpY) + 1, 0);
    CHECK(movedLeft >= restLeft + 10 && movedLeft <= restLeft + 14);

    skeletonNode->queueFree();
    shoot(c);
}

int agreement(const Picture& a, const Picture& b, int yMin, int yMax, int& coveredA, int& coveredB)
{
    int same = 0;
    int total = 0;
    coveredA = 0;
    coveredB = 0;
    for (int y = yMin; y < yMax; ++y)
        for (int x = -kSize / 2; x < kSize / 2; ++x)
        {
            const bool ca = a.covered(x, y);
            const bool cb = b.covered(x, y);
            coveredA += ca ? 1 : 0;
            coveredB += cb ? 1 : 0;
            if (ca == cb) ++same;
            ++total;
        }
    return total > 0 ? same * 100 / total : 0;
}

void testGpuMatchesCpu(Context& c)
{
    if (!Main::device()->driver()->caps().compute)
    {
        printf("no compute: GPU hair comparison skipped\n");
        return;
    }
    Hair3D* hair = c.root->createChild<Hair3D>();
    configure(hair, c.material);
    hair->setScalp(scalpData(false));
    hair->setGravityPower(20.0f);
    hair->setSimulation(HairSimulation::Cpu);
    const Picture cpu = shoot(c, 40);
    CHECK(!hair->isSimulatingOnGpu());

    hair->setSimulation(HairSimulation::Gpu);
    const Picture gpu = shoot(c, 40);
    CHECK(hair->isSimulatingOnGpu());
    int coveredCpu = 0;
    int coveredGpu = 0;
    const int same = agreement(cpu, gpu, -kSize / 2, kSize / 2, coveredCpu, coveredGpu);
    CHECK(coveredCpu > 100 && coveredGpu > 100);
    CHECK(abs(coveredCpu - coveredGpu) * 100 <= coveredCpu * 15);
    CHECK(same >= 90);
    CHECK(abs(gpu.highestCovered() - cpu.highestCovered()) <= 2);
    const Math::Box bounds = hair->getAabb();
    CHECK(bounds.max.y > bounds.min.y + 10.0f);

    hair->setWind(Math::Vec3(1.0f, 0.0f, 0.0f), 2.0f);
    hair->setSimulation(HairSimulation::Cpu);
    const Picture cpuWind = shoot(c, 40);
    hair->setSimulation(HairSimulation::Gpu);
    const Picture gpuWind = shoot(c, 40);
    const int sameWind = agreement(cpuWind, gpuWind, -kSize / 2, kSize / 2, coveredCpu, coveredGpu);
    CHECK(coveredCpu > 100 && coveredGpu > 100);
    CHECK(sameWind >= 80);

    hair->setVisible(false);
    const Picture hidden = shoot(c);
    CHECK(hidden.coveredBetween(-kSize / 2, kSize / 2) == 0);
    hair->queueFree();
    shoot(c);
}

} // namespace

int main(int argc, char** argv)
{
    const bool vulkan = argc > 1 && strcmp(argv[1], "vulkan") == 0;
    log_set_callback(onLog, nullptr);

    CHECK(Device::headlessSupported());
    if (!Device::headlessSupported()) return 1;

    DeviceDesc desc;
    desc.title = "hair render test";
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

    c.material = Main::materials()->create("hair test");
    Main::materials()->setColor(c.material, MaterialParam::Albedo,
            Math::Vec4(0.2f, 0.2f, 0.2f, 1.0f));
    Main::materials()->setCull(c.material, MaterialCull::Disabled);

    testStrandsDraw(c);
    testSkinnedScalp(c);
    testGpuMatchesCpu(c);

    CHECK(gErrors == 0);
    Main::device()->requestClose();
    tree.finalize();
    Main::cleanup();

    if (failures) printf("%d check(s) failed\n", failures);
    else
        printf("all checks passed\n");
    return failures ? 1 : 0;
}
