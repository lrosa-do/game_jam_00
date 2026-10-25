#include "Check.h"
#include "ColouredBox.h"
#include "LitTest.h"
#include "MarkedBox.h"

#include "prisma3d/Device.h"
#include "prisma3d/core/Random.h"
#include "prisma3d/main/Main.h"
#include "prisma3d/render/Impostor.h"
#include "prisma3d/render/MaterialStore.h"
#include "prisma3d/render/MeshStore.h"
#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/render/TextureStore.h"
#include "prisma3d/scene/Camera3D.h"
#include "prisma3d/scene/Forest3D.h"
#include "prisma3d/scene/Light3D.h"
#include "prisma3d/scene/PrimitiveMeshes.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/TreeGenerator.h"
#include "prisma3d/scene/Viewport.h"
#include "prisma3d/scene/VisualInstance3D.h"
#include "prisma3d/scene/WorldEnvironment.h"

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

bool pixelIs(int x, int y, Rgb expected, const char* label)
{
    unsigned char pixel[4] = { 0, 0, 0, 0 };
    prisma::Rect rect;
    rect.x = x;
    rect.y = y;
    rect.width = 1;
    rect.height = 1;
    if (!Main::device()->driver()->readPixels(prisma::RenderTarget(), rect, pixel))
    {
        printf("%s: pixel (%d,%d) could not be read\n", label, x, y);
        return false;
    }
    const bool same = abs(pixel[0] - expected.r) <= 3 && abs(pixel[1] - expected.g) <= 3 &&
                      abs(pixel[2] - expected.b) <= 3;
    if (!same)
        printf("%s: pixel (%d,%d) is %d,%d,%d, expected %d,%d,%d\n", label, x, y, pixel[0],
                pixel[1], pixel[2], expected.r, expected.g, expected.b);
    return same;
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

    const RenderStats& stats() const
    {
        return Main::rendering()->getStats();
    }
};

const float kScale = 2.0f;
const float kRadius = 1.7320508f;
const float kCentreY = 1.0f;
const float kFade = 20.0f;
const float kDraw = 100.0f;
const float kLens = 10.0f;
const float kNear = 0.5f;
const float kFar = 400.0f;

bool nearTo(float a, float b, float eps)
{
    return fabsf(a - b) <= eps;
}

float depthAt(float distance)
{
    const float ndc =
            (kFar + kNear) / (kFar - kNear) - (2.0f * kFar * kNear) / ((kFar - kNear) * distance);
    return ndc * 0.5f + 0.5f;
}

void testMeshDissolve()
{
    float dither = -1.0f;
    CHECK(Forest3D::meshDissolve(50.0f, 100.0f, 100.0f, dither) && dither == 0.0f);
    CHECK(Forest3D::meshDissolve(100.0f, 100.0f, 100.0f, dither) && dither == 0.0f);
    CHECK(Forest3D::meshDissolve(150.0f, 100.0f, 100.0f, dither) && nearTo(dither, 0.5f, 1e-6f));
    CHECK(Forest3D::meshDissolve(199.0f, 100.0f, 100.0f, dither) && nearTo(dither, 0.99f, 1e-6f));
    CHECK(!Forest3D::meshDissolve(200.0f, 100.0f, 100.0f, dither) && nearTo(dither, 1.0f, 1e-6f));
    CHECK(!Forest3D::meshDissolve(199.5f, 100.0f, 100.0f, dither) && nearTo(dither, 0.995f, 1e-6f));
    CHECK(!Forest3D::meshDissolve(500.0f, 100.0f, 100.0f, dither) && dither > 1.0f);
    CHECK(Forest3D::meshDissolve(0.0f, 100.0f, 100.0f, dither) && dither == 0.0f);

    CHECK(Forest3D::meshDissolve(100.0f, 100.0f, 0.0f, dither) && dither == 0.0f);
    CHECK(!Forest3D::meshDissolve(100.5f, 100.0f, 0.0f, dither) && dither == 0.0f);
    CHECK(Forest3D::meshDissolve(30.0f, 100.0f, 0.0f, dither) && dither == 0.0f);

    const float draw = 500.0f;
    CHECK(Forest3D::meshDissolve(499.0f, draw, 2.0f, dither) && dither == 0.0f);
    CHECK(Forest3D::meshDissolve(501.0f, draw, 2.0f, dither) && nearTo(dither, 0.5f, 1e-4f));
    CHECK(!Forest3D::meshDissolve(502.0f, draw, 2.0f, dither));
    CHECK(!Forest3D::meshDissolve(600.0f, draw, 2.0f, dither));
}

struct Shot
{
    ct::Vector<std::uint8_t> pixels;
    RenderStats stats;
    float depth = 1.0f;

    Rgb at(int x, int y) const
    {
        const std::uint8_t* p = pixels.data() + (static_cast<std::size_t>(y) * kWidth + x) * 4;
        return Rgb{ p[0], p[1], p[2] };
    }

    int countBox() const
    {
        int count = 0;
        for (int y = 0; y < kHeight; ++y)
        {
            for (int x = 0; x < kWidth; ++x)
            {
                const Rgb p = at(x, y);
                const int d = abs(p.r - kClear.r) > abs(p.g - kClear.g) ? abs(p.r - kClear.r)
                                                                        : abs(p.g - kClear.g);
                const int e = d > abs(p.b - kClear.b) ? d : abs(p.b - kClear.b);
                if (e > 20) ++count;
            }
        }
        return count;
    }
};

int differing(const Shot& a, const Shot& b)
{
    int count = 0;
    for (int y = 0; y < kHeight; ++y)
    {
        for (int x = 0; x < kWidth; ++x)
        {
            const Rgb p = a.at(x, y);
            const Rgb q = b.at(x, y);
            if (abs(p.r - q.r) > 3 || abs(p.g - q.g) > 3 || abs(p.b - q.b) > 3) ++count;
        }
    }
    return count;
}

Shot shoot(SceneTree& tree, Window* window, bool readDepth = false)
{
    Frame frame(tree);
    Shot shot;
    shot.pixels.resize(static_cast<std::size_t>(kWidth) * kHeight * 4);
    prisma::Rect rect;
    rect.width = kWidth;
    rect.height = kHeight;
    CHECK(Main::device()->driver()->readPixels(prisma::RenderTarget(), rect, shot.pixels.data()));
    shot.stats = frame.stats();
    if (readDepth)
        CHECK(Main::rendering()->viewportReadDepth(window->getViewportId(), kWidth / 2, kHeight / 2,
                shot.depth));
    return shot;
}

int cardsOnGpu(ImpostorSetId set)
{
    std::uint32_t indexCount = 0;
    ct::Vector<ImpostorCardReadback> cards;
    CHECK(Main::rendering()->impostorSetReadCards(set, indexCount, cards));
    CHECK(indexCount % 6 == 0 && cards.size() == indexCount / 6);
    return static_cast<int>(indexCount / 6);
}

Math::Box treeBox(const TreeGeometry& geometry)
{
    return MeshStore::computeAabb(geometry.bark.positions)
            .Union(MeshStore::computeAabb(geometry.leaves.positions));
}

void plantDemoForest(Forest3D& forest, const float* fits, const int* counts, Random& random)
{
    const float radius = 300.0f;
    int total = 0;
    for (int s = 0; s < 3; ++s) total += counts[s];
    const float area = kPi * radius * radius;
    const float cell = sqrtf(area / static_cast<float>(total));

    ct::Vector<Math::Vec2> spots;
    for (float z = -radius; z < radius; z += cell)
    {
        for (float x = -radius; x < radius; x += cell)
        {
            const float px = x + random.randfRange(0.1f, 0.9f) * cell;
            const float pz = z + random.randfRange(0.1f, 0.9f) * cell;
            if (px * px + pz * pz <= radius * radius) spots.push_back(Math::Vec2(px, pz));
        }
    }
    for (std::size_t i = spots.size(); i > 1; --i)
    {
        const std::size_t j =
                static_cast<std::size_t>(random.randiRange(0, static_cast<int>(i) - 1));
        const Math::Vec2 held = spots[i - 1];
        spots[i - 1] = spots[j];
        spots[j] = held;
    }

    std::size_t next = 0;
    for (int s = 0; s < 3; ++s)
    {
        for (int i = 0; i < counts[s]; ++i)
        {
            Math::Vec2 point(0.0f, 0.0f);
            if (next < spots.size()) point = spots[next++];
            else
            {
                const float angle = random.randfRange(0.0f, 6.2831853f);
                const float distance = radius * sqrtf(random.randf());
                point = Math::Vec2(cosf(angle) * distance, sinf(angle) * distance);
            }
            const float scale = fits[s] * random.randfRange(0.8f, 1.25f);
            const float yaw = random.randfRange(0.0f, 6.2831853f);
            forest.plant(s, Math::Vec3(point.x, 0.0f, point.y), yaw, scale);
        }
    }
}

struct Scene
{
    SceneTree* tree = nullptr;
    Node* root = nullptr;
    Camera3D* camera = nullptr;
};

void place(Scene& scene, const Math::Vec3& position, const Math::Vec3& target)
{
    scene.camera->setGlobalTransform(
            Math::Mat4::LookAt(position, target, Math::Vec3(0.0f, 1.0f, 0.0f)).InverseAffine());
}

void look(Scene& scene, float distance)
{
    place(scene, Math::Vec3(0.0f, kCentreY, distance), Math::Vec3(0.0f, kCentreY, 0.0f));
}

float ditherFor(float distance, float fade, float radius)
{
    return (distance > fade ? distance - fade : 0.0f) / radius;
}

bool recordIs(RenderingServer* server, ImpostorSetId set, int index, const Math::Vec3& centre,
        float radius, float fade, int slot)
{
    ImpostorInstanceInfo info;
    if (!server->impostorSetGetInstance(set, static_cast<std::uint32_t>(index), info))
    {
        printf("record %d could not be read\n", index);
        return false;
    }
    const bool same =
            nearTo(info.center[0], centre.x, 1e-4f) && nearTo(info.center[1], centre.y, 1e-4f) &&
            nearTo(info.center[2], centre.z, 1e-4f) && nearTo(info.radius, radius, 1e-4f) &&
            nearTo(info.fadeDistance, fade, 1e-4f) &&
            info.sliceOffset == static_cast<int>(impostorSliceOffset(slot));
    if (!same)
        printf("record %d: centre %f %f %f radius %f fade %f slice %d\n", index, info.center[0],
                info.center[1], info.center[2], info.radius, info.fadeDistance, info.sliceOffset);
    return same;
}

bool recordMatchesNode(RenderingServer* server, const Forest3D* forest, int index)
{
    ImpostorInstanceInfo info;
    if (!server->impostorSetGetInstance(forest->getImpostorSet(), static_cast<std::uint32_t>(index),
                info))
        return false;
    const Math::Vec3 centre = forest->getInstanceCentre(index);
    return info.center[0] == centre.x && info.center[1] == centre.y && info.center[2] == centre.z &&
           info.radius == forest->getInstanceRadius(index);
}

} // namespace

int main(int argc, char** argv)
{
    const bool vulkan = argc > 1 && strcmp(argv[1], "vulkan") == 0;

    testMeshDissolve();

    CHECK(Device::headlessSupported());
    if (!Device::headlessSupported()) return 1;

    DeviceDesc desc;
    desc.title = "forest test";
    desc.driver = vulkan ? prisma::DriverType::Vulkan : prisma::DriverType::OpenGL;
    desc.width = kWidth;
    desc.height = kHeight;
    desc.headless = true;

    CHECK(Main::setup(desc));
    if (!Main::device()) return 1;

    SceneTree tree;
    CHECK(Main::start(&tree));
    Node* root = tree.getRoot();
    Window* window = root->castTo<Window>();
    CHECK(window != nullptr);
    if (!window) return 1;
    window->setClearColor(Math::Vec4(0.1f, 0.2f, 0.3f, 1.0f));

    Node3D holder;
    WorldEnvironment* environment = holder.createChild<WorldEnvironment>();
    environment->setEnvironment(whiteAmbient());
    environment->reparent(root);

    Scene scene;
    scene.tree = &tree;
    scene.root = root;
    scene.camera = holder.createChild<Camera3D>();
    scene.camera->setPerspective(8.0f, kNear, kFar);
    scene.camera->reparent(root);
    scene.camera->makeCurrent();

    MeshStore& meshes = *Main::meshes();
    TextureStore& textures = *Main::textures();
    MaterialStore& materials = *Main::materials();
    RenderingServer* server = Main::rendering();

    const MeshId box = meshes.create("box");
    CHECK(meshes.addSurface(box, colouredBox()));
    const MeshId marked = meshes.create("marked box");
    CHECK(meshes.addSurface(marked, markedBox()));
    scene.camera->setGlobalTransform(Math::Mat4::Identity());
    materials.defaultMaterial();
    {
        Frame frame(tree);
    }
    const std::size_t meshBaseline = meshes.count();
    const std::size_t textureBaseline = textures.count();
    const std::size_t materialBaseline = materials.count();

    Forest3D* forest = root->createChild<Forest3D>();
    CHECK(forest->castTo<Forest3D>() == forest);
    CHECK(forest->castTo<Node3D>() == forest);
    CHECK(forest->getSpeciesCount() == 0);
    CHECK(forest->getInstanceCount() == 0);
    CHECK(forest->getImpostorSet().valid());
    CHECK(server->impostorSetGetCount(forest->getImpostorSet()) == 0);
    CHECK(nearTo(forest->getSwapDistance(), 120.0f, 1e-6f));
    CHECK(nearTo(forest->getDrawDistance(), 500.0f, 1e-6f) && forest->isImpostorsEnabled());
    forest->setSwapDistance(kFade);
    forest->setDrawDistance(kDraw);
    CHECK(nearTo(forest->getSwapDistance(), kFade, 1e-6f));
    CHECK(nearTo(forest->getDrawDistance(), kDraw, 1e-6f));

    ForestSpeciesDesc invalid;
    CHECK(forest->addSpecies(invalid) == -1);
    CHECK(forest->addInstance(0, Math::Mat4::Identity()) == -1);
    CHECK(!forest->getImpostor(0).valid());
    CHECK(forest->getMeshNode(0) == nullptr);

    ForestSpeciesDesc desc0;
    desc0.mesh = box;
    const int boxSpecies = forest->addSpecies(desc0);
    CHECK(boxSpecies == 0);
    CHECK(forest->getSpeciesCount() == 1);
    const ImpostorId boxImpostor = forest->getImpostor(0);
    CHECK(boxImpostor.valid());
    CHECK(server->impostorGetSlot(boxImpostor) >= 0);
    CHECK(nearTo(server->impostorGetSwapInDistance(boxImpostor), kFade, 1e-6f));
    CHECK(forest->getMeshNode(0) != nullptr);
    CHECK(forest->getMeshNode(0)->castTo<MultiMeshInstance3D>() == forest->getMeshNode(0));
    CHECK(forest->getMeshNode(0)->getMesh() == box);
    CHECK(forest->getMeshNode(0)->getCastShadows());
    CHECK(forest->getMeshNode(0)->getUseCustomData());
    CHECK(forest->getMeshNode(0)->getMultiMesh()->getDissolve());
    CHECK(textures.count() == textureBaseline);
    CHECK(materials.count() == materialBaseline);
    CHECK(meshes.count() == meshBaseline);

    place(scene, Math::Vec3(0.0f, 0.5f, 10.0f), Math::Vec3(0.0f, 0.5f, 0.0f));
    scene.camera->setPerspective(60.0f, kNear, kFar);
    CHECK(forest->plant(0, Math::Vec3(-2.0f, 0.0f, 0.0f), 0.0f, 1.0f) == 0);
    CHECK(forest->plant(0, Math::Vec3(2.0f, 0.0f, -20.0f), 0.0f, 1.0f) == 1);
    CHECK(forest->plant(0, Math::Vec3(0.0f, 0.0f, -12.0f), 0.0f, 1.0f) == 2);
    CHECK(forest->getInstanceCount() == 3 && forest->getInstanceSpecies(2) == 0);
    CHECK(nearTo(forest->getInstanceTransform(1).GetTranslation().z, -20.0f, 1e-6f));
    CHECK(server->impostorSetGetCount(forest->getImpostorSet()) == 3);
    {
        Frame frame(tree);
        CHECK(forest->getMeshInstanceCount() == 1);
        CHECK(forest->getImpostorInstanceCount() == 2);
        CHECK(forest->getMeshInstanceCount(0) == 1 && forest->getImpostorInstanceCount(0) == 2);
        CHECK(forest->getWrittenInstanceCount() == 1);
        CHECK(frame.stats().multimeshInstancesDrawn == 1);
        CHECK(frame.stats().impostorSets == 1);
        CHECK(frame.stats().impostorDrawCalls == 1);
        CHECK(forest->getMeshNode(0)->getVisibleInstanceCount() == 1);
        CHECK(pixelIs(47, 48, kRed, "near tree"));
        CHECK(frame.stats().multimeshUploadBytes > 0);
    }
    CHECK(cardsOnGpu(forest->getImpostorSet()) == 2);
    {
        Frame frame(tree);
        CHECK(forest->getWrittenInstanceCount() == 0);
        CHECK(frame.stats().multimeshUploadBytes == 0);
        CHECK(frame.stats().multimeshInstancesDrawn == 1);
    }

    forest->removeSpecies(0);
    CHECK(forest->getSpeciesCount() == 0 && forest->getInstanceCount() == 0);
    CHECK(server->impostorGetSlot(boxImpostor) == -1);
    CHECK(server->impostorSetGetCount(forest->getImpostorSet()) == 0);
    CHECK(textures.count() == textureBaseline);
    CHECK(materials.count() == materialBaseline);
    CHECK(meshes.count() == meshBaseline);

    ForestSpeciesDesc markedDesc;
    markedDesc.mesh = marked;
    CHECK(forest->addSpecies(markedDesc) == 0);
    const ImpostorId markedImpostor = forest->getImpostor(0);
    const ImpostorSetId set = forest->getImpostorSet();
    CHECK(forest->plant(0, Math::Vec3(0.0f, 0.0f, 0.0f), 0.0f, kScale) == 0);
    scene.camera->setPerspective(kLens, kNear, kFar);
    CHECK(recordIs(server, set, 0, Math::Vec3(0.0f, kCentreY, 0.0f), kRadius, kFade,
            server->impostorGetSlot(markedImpostor)));
    CHECK(recordMatchesNode(server, forest, 0));
    CHECK(nearTo(forest->getInstanceRadius(0), kRadius, 1e-5f));

    {
        const float steps[11] = { 10.0f, kFade - kRadius - 1.0f, kFade - kRadius + 0.05f,
            kFade - kRadius * 0.5f, kFade, kFade + kRadius * 0.25f, kFade + kRadius * 0.5f,
            kFade + kRadius * 0.9f, kFade + kRadius + 1.0f, 60.0f, 10.0f };
        bool wasMesh = false;
        float wasDither = 0.0f;
        for (int i = 0; i < 11; ++i)
        {
            const float d = steps[i];
            look(scene, d);
            const float dither = ditherFor(d, kFade, kRadius);
            const bool wantMesh = d <= kFade + kRadius && dither <= 0.99f;
            const bool wantCard = d >= kFade - kRadius;
            int writes = 0;
            if (wantMesh && !wasMesh) writes = 1;
            else if (wantMesh && wasMesh && fabsf(dither - wasDither) > 1e-6f)
                writes = 1;
            char label[96];
            snprintf(label, sizeof(label), "band step %d (distance %.3f)", i, d);
            {
                Frame frame(tree);
                const bool ok = forest->getMeshInstanceCount() == (wantMesh ? 1 : 0) &&
                                forest->getImpostorInstanceCount() == (wantCard ? 1 : 0);
                if (!ok)
                    printf("%s: mesh %d card %d, expected %d and %d\n", label,
                            forest->getMeshInstanceCount(), forest->getImpostorInstanceCount(),
                            wantMesh ? 1 : 0, wantCard ? 1 : 0);
                CHECK(ok);
                CHECK(frame.stats().multimeshInstancesDrawn == (wantMesh ? 1u : 0u));
                if (forest->getWrittenInstanceCount() != writes)
                    printf("%s: %d writes, expected %d\n", label, forest->getWrittenInstanceCount(),
                            writes);
                CHECK(forest->getWrittenInstanceCount() == writes);
                CHECK((frame.stats().multimeshUploadBytes > 0) == (writes > 0));
                CHECK(frame.stats().impostorSets == 1);
            }
            CHECK(cardsOnGpu(set) == (wantCard ? 1 : 0));
            if (wantMesh)
            {
                const float stored = forest->getMeshNode(0)->getInstanceCustomData(0).x;
                if (!nearTo(stored, dither, 1e-4f))
                    printf("%s: custom data %f, dither %f\n", label, stored, dither);
                CHECK(nearTo(stored, dither, 1e-4f));
            }
            {
                Frame frame(tree);
                CHECK(forest->getWrittenInstanceCount() == 0);
                CHECK(frame.stats().multimeshUploadBytes == 0);
                CHECK(forest->getMeshInstanceCount() == (wantMesh ? 1 : 0));
                CHECK(forest->getImpostorInstanceCount() == (wantCard ? 1 : 0));
            }
            wasMesh = wantMesh;
            wasDither = dither;
        }
    }

    {
        const float half = kFade + kRadius * 0.5f;
        look(scene, half);
        forest->setImpostorsEnabled(false);
        const Shot full = shoot(tree, window, true);
        forest->setImpostorsEnabled(true);
        const Shot both = shoot(tree, window, true);
        server->impostorSetEnabled(set, false);
        const Shot dissolved = shoot(tree, window);
        server->impostorSetEnabled(set, true);
        const int nFull = full.countBox();
        const int nBoth = both.countBox();
        const int nDissolved = dissolved.countBox();
        printf("half dissolve at %.2f: full %d, both %d, dissolved mesh only %d\n", half, nFull,
                nBoth, nDissolved);
        CHECK(nFull > 1500);
        CHECK(full.stats.impostorSets == 0 && both.stats.impostorSets == 1 &&
                dissolved.stats.impostorSets == 0);
        CHECK(both.stats.impostorDrawCalls == 1);
        CHECK(nDissolved > nFull * 40 / 100 && nDissolved < nFull * 60 / 100);
        CHECK(nBoth > nFull * 90 / 100 && nBoth < nFull * 110 / 100);
        CHECK(nBoth > nDissolved * 17 / 10);
        CHECK(nearTo(full.depth, depthAt(half - kScale * 0.5f), 2e-4f));
        CHECK(nearTo(both.depth, depthAt(half - kScale * 0.5f), 2e-4f) ||
                nearTo(both.depth, depthAt(half), 2e-4f));

        const float beyond = kFade + kRadius + 1.0f;
        look(scene, beyond);
        forest->setImpostorsEnabled(false);
        const Shot fullBeyond = shoot(tree, window);
        forest->setImpostorsEnabled(true);
        const Shot cardOnly = shoot(tree, window, true);
        server->impostorSetEnabled(set, false);
        const Shot nothing = shoot(tree, window);
        server->impostorSetEnabled(set, true);
        const int nFullBeyond = fullBeyond.countBox();
        const int nCard = cardOnly.countBox();
        printf("beyond the band at %.2f: mesh %d, card %d, set off %d\n", beyond, nFullBeyond,
                nCard, nothing.countBox());
        CHECK(cardOnly.stats.multimeshInstancesDrawn == 0);
        CHECK(cardOnly.stats.impostorSets == 1 && cardOnly.stats.impostorDrawCalls == 1);
        CHECK(nothing.countBox() == 0);
        CHECK(nCard > nFullBeyond * 80 / 100 && nCard < nFullBeyond * 115 / 100);
        CHECK(nearTo(cardOnly.depth, depthAt(beyond), 2e-4f));

        const float before = kFade - kRadius - 1.0f;
        look(scene, before);
        forest->setImpostorsEnabled(false);
        const Shot fullBefore = shoot(tree, window);
        forest->setImpostorsEnabled(true);
        const Shot meshOnly = shoot(tree, window, true);
        CHECK(cardsOnGpu(set) == 0);
        CHECK(meshOnly.stats.multimeshInstancesDrawn == 1);
        CHECK(differing(fullBefore, meshOnly) == 0);
        CHECK(nearTo(meshOnly.depth, depthAt(before - kScale * 0.5f), 2e-4f));
        CHECK(meshOnly.countBox() > 1500);
    }

    forest->setImpostorsEnabled(false);
    CHECK(!forest->isImpostorsEnabled());
    {
        const float distances[4] = { kDraw - 1.0f, kDraw, kDraw + kRadius * 0.5f,
            kDraw + kRadius + 1.0f };
        const int meshCounts[4] = { 1, 1, 1, 0 };
        for (int i = 0; i < 4; ++i)
        {
            look(scene, distances[i]);
            Frame frame(tree);
            CHECK(forest->getMeshInstanceCount() == meshCounts[i]);
            CHECK(forest->getImpostorInstanceCount() == 0);
            CHECK(frame.stats().impostorSets == 0);
            CHECK(frame.stats().multimeshInstancesDrawn ==
                    static_cast<std::uint32_t>(meshCounts[i]));
            if (i == 2)
                CHECK(nearTo(forest->getMeshNode(0)->getInstanceCustomData(0).x, 0.5f, 1e-4f));
        }
        scene.camera->setPerspective(3.0f, kNear, kFar);
        look(scene, kDraw + kRadius * 0.5f);
        forest->setDrawDistance(500.0f);
        const Shot full = shoot(tree, window);
        forest->setDrawDistance(kDraw);
        const Shot dissolved = shoot(tree, window);
        printf("dissolve at the draw distance: full %d, dissolved %d\n", full.countBox(),
                dissolved.countBox());
        CHECK(full.countBox() > 800);
        CHECK(dissolved.countBox() > full.countBox() * 40 / 100 &&
                dissolved.countBox() < full.countBox() * 60 / 100);
        scene.camera->setPerspective(kLens, kNear, kFar);
    }
    forest->setImpostorsEnabled(true);

    look(scene, 10.0f);
    forest->setSwapDistance(5.0f);
    CHECK(nearTo(server->impostorGetSwapInDistance(markedImpostor), 5.0f, 1e-6f));
    {
        Frame frame(tree);
        CHECK(forest->getMeshInstanceCount() == 0 && forest->getImpostorInstanceCount() == 1);
        CHECK(frame.stats().multimeshInstancesDrawn == 0);
    }
    CHECK(cardsOnGpu(set) == 1);
    CHECK(recordIs(server, set, 0, Math::Vec3(0.0f, kCentreY, 0.0f), kRadius, 5.0f,
            server->impostorGetSlot(markedImpostor)));
    forest->setDrawDistance(3.0f);
    CHECK(nearTo(server->impostorGetSwapInDistance(markedImpostor), 3.0f, 1e-6f));
    forest->setDrawDistance(kDraw);
    forest->setSwapDistance(kFade);
    CHECK(nearTo(server->impostorGetSwapInDistance(markedImpostor), kFade, 1e-6f));
    {
        Frame frame(tree);
        CHECK(forest->getMeshInstanceCount() == 1 && forest->getImpostorInstanceCount() == 0);
    }

    forest->setPosition(Math::Vec3(0.0f, 0.0f, 45.0f));
    place(scene, Math::Vec3(0.0f, kCentreY, 55.0f), Math::Vec3(0.0f, kCentreY, 45.0f));
    {
        Frame frame(tree);
        CHECK(forest->getMeshInstanceCount() == 1 && forest->getImpostorInstanceCount() == 0);
        CHECK(forest->getMeshNode(0)->getAabb().Contains(Math::Vec3(0.0f, 0.5f, 0.0f)));
        CHECK(nearTo(forest->getInstanceCentre(0).z, 45.0f, 1e-4f));
    }
    CHECK(recordIs(server, set, 0, Math::Vec3(0.0f, kCentreY, 45.0f), kRadius, kFade,
            server->impostorGetSlot(markedImpostor)));
    CHECK(recordMatchesNode(server, forest, 0));
    forest->setPosition(Math::Vec3(0.0f, 0.0f, 0.0f));
    look(scene, 10.0f);
    {
        Frame frame(tree);
        CHECK(forest->getMeshInstanceCount() == 1);
    }
    CHECK(recordIs(server, set, 0, Math::Vec3(0.0f, kCentreY, 0.0f), kRadius, kFade,
            server->impostorGetSlot(markedImpostor)));

    forest->clearInstances();
    CHECK(forest->getInstanceCount() == 0 && server->impostorSetGetCount(set) == 0);
    {
        Frame frame(tree);
        CHECK(forest->getMeshInstanceCount() == 0 && forest->getImpostorInstanceCount() == 0);
        CHECK(frame.stats().multimeshInstancesDrawn == 0);
    }

    {
        ForestSpeciesDesc second;
        second.mesh = box;
        const int other = forest->addSpecies(second);
        CHECK(other == 1);
        CHECK(forest->getImpostor(1).valid());
        const ImpostorId boxId = forest->getImpostor(1);
        CHECK(server->impostorGetSlot(boxId) != server->impostorGetSlot(markedImpostor));
        CHECK(forest->plant(0, Math::Vec3(0.0f, 0.0f, 0.0f), 0.0f, kScale) == 0);
        CHECK(forest->plant(0, Math::Vec3(10.0f, 0.0f, 0.0f), 0.0f, 1.0f) == 1);
        CHECK(forest->plant(1, Math::Vec3(0.0f, 0.0f, -5.0f), 0.0f, 3.0f) == 2);
        CHECK(forest->getInstanceCount() == 3);
        CHECK(server->impostorSetGetCount(set) == 3);
        const int markedSlot = server->impostorGetSlot(markedImpostor);
        const int boxSlot = server->impostorGetSlot(boxId);
        CHECK(recordIs(server, set, 0, Math::Vec3(0.0f, 1.0f, 0.0f), kRadius, kFade, markedSlot));
        CHECK(recordIs(server, set, 1, Math::Vec3(10.0f, 0.5f, 0.0f), kRadius * 0.5f, kFade,
                markedSlot));
        CHECK(recordIs(server, set, 2, Math::Vec3(0.0f, 1.5f, -5.0f), kRadius * 1.5f, kFade,
                boxSlot));
        for (int i = 0; i < 3; ++i) CHECK(recordMatchesNode(server, forest, i));

        scene.camera->setPerspective(30.0f, kNear, kFar);
        place(scene, Math::Vec3(5.0f, 1.0f, 60.0f), Math::Vec3(5.0f, 1.0f, 0.0f));
        {
            Frame frame(tree);
            CHECK(forest->getInstanceCount() == 3);
            CHECK(forest->getMeshInstanceCount(0) == 0 && forest->getMeshInstanceCount(1) == 0);
            CHECK(forest->getImpostorInstanceCount(0) == 2);
            CHECK(forest->getImpostorInstanceCount(1) == 1);
            CHECK(forest->getImpostorInstanceCount() == 3);
        }
        CHECK(cardsOnGpu(set) == 3);

        forest->removeSpecies(0);
        CHECK(forest->getSpeciesCount() == 1 && forest->getInstanceCount() == 1);
        CHECK(forest->getInstanceSpecies(0) == 0);
        CHECK(server->impostorGetSlot(markedImpostor) == -1);
        CHECK(forest->getImpostor(0) == boxId);
        CHECK(server->impostorGetSlot(boxId) == boxSlot);
        CHECK(server->impostorSetGetCount(set) == 1);
        CHECK(recordIs(server, set, 0, Math::Vec3(0.0f, 1.5f, -5.0f), kRadius * 1.5f, kFade,
                boxSlot));
        CHECK(recordMatchesNode(server, forest, 0));
        {
            Frame frame(tree);
            CHECK(forest->getMeshInstanceCount() == 0 && forest->getImpostorInstanceCount() == 1);
            CHECK(frame.stats().multimeshInstancesDrawn == 0);
        }
        CHECK(cardsOnGpu(set) == 1);

        forest->removeSpecies(0);
        CHECK(forest->getSpeciesCount() == 0 && forest->getInstanceCount() == 0);
        CHECK(server->impostorGetSlot(boxId) == -1);
        CHECK(server->impostorSetGetCount(set) == 0);
        forest->removeSpecies(5);
        {
            Frame frame(tree);
            CHECK(frame.stats().drawCalls == 0);
            CHECK(frame.stats().impostorDrawCalls == 0);
            CHECK(pixelIs(kWidth / 2, 48, kClear, "no species"));
        }
        CHECK(textures.count() == textureBaseline);
        CHECK(materials.count() == materialBaseline);
        CHECK(meshes.count() == meshBaseline);
    }

    {
        const MeshId loading = meshes.create("loading box");
        ForestSpeciesDesc loadingDesc;
        loadingDesc.mesh = loading;
        CHECK(forest->addSpecies(loadingDesc) == 0);
        const ImpostorId loadingImpostor = forest->getImpostor(0);
        CHECK(forest->plant(0, Math::Vec3(0.0f, 0.0f, 0.0f), 0.0f, kScale) == 0);
        CHECK(forest->getInstanceRadius(0) == 0.0f);
        {
            ImpostorInstanceInfo info;
            CHECK(!server->impostorSetGetInstance(set, 0, info));
        }
        scene.camera->setPerspective(kLens, kNear, kFar);
        look(scene, 10.0f);
        {
            Frame frame(tree);
            CHECK(forest->getInstanceRadius(0) == 0.0f);
            CHECK(forest->getImpostorInstanceCount() == 0);
        }
        {
            ImpostorInstanceInfo info;
            CHECK(!server->impostorSetGetInstance(set, 0, info));
        }
        CHECK(meshes.addSurface(loading, markedBox()));
        forest->update(Math::Vec3(0.0f, kCentreY, 10.0f));
        CHECK(nearTo(forest->getInstanceRadius(0), kRadius, 1e-5f));
        CHECK(nearTo(forest->getInstanceCentre(0).y, kCentreY, 1e-5f));
        CHECK(recordIs(server, set, 0, Math::Vec3(0.0f, kCentreY, 0.0f), kRadius, kFade,
                server->impostorGetSlot(loadingImpostor)));
        CHECK(recordMatchesNode(server, forest, 0));
        CHECK(forest->getMeshInstanceCount() == 1);
        {
            Frame frame(tree);
            CHECK(forest->getMeshInstanceCount() == 1 && forest->getImpostorInstanceCount() == 0);
            CHECK(forest->getWrittenInstanceCount() == 0);
        }
        look(scene, kFade + kRadius * 0.5f);
        {
            Frame frame(tree);
            CHECK(forest->getMeshInstanceCount() == 1 && forest->getImpostorInstanceCount() == 1);
            CHECK(nearTo(forest->getMeshNode(0)->getInstanceCustomData(0).x, 0.5f, 1e-4f));
        }
        forest->removeSpecies(0);
        meshes.destroy(loading);
        CHECK(forest->getSpeciesCount() == 0 && forest->getInstanceCount() == 0);
        CHECK(meshes.count() == meshBaseline);
    }

    {
        CHECK(forest->addSpecies(markedDesc) == 0);
        CHECK(forest->plant(0, Math::Vec3(0.0f, 0.0f, 0.0f), 0.0f, kScale) == 0);
        scene.camera->setPerspective(kLens, kNear, kFar);
        place(scene, Math::Vec3(0.0f, kCentreY, 10.0f), Math::Vec3(0.0f, kCentreY, 20.0f));
        CHECK(forest->isFrustumCullingEnabled());
        {
            Frame frame(tree);
            CHECK(forest->getMeshInstanceCount() == 0);
        }
        forest->setFrustumCullingEnabled(false);
        CHECK(!forest->isFrustumCullingEnabled());
        {
            Frame frame(tree);
            CHECK(forest->getMeshInstanceCount() == 1);
            CHECK(frame.stats().multimeshInstancesDrawn == 0);
        }
        place(scene, Math::Vec3(0.0f, kCentreY, 40.0f), Math::Vec3(0.0f, kCentreY, 80.0f));
        {
            Frame frame(tree);
            CHECK(forest->getMeshInstanceCount() == 0 && forest->getImpostorInstanceCount() == 1);
        }
        CHECK(cardsOnGpu(set) == 0);
        forest->setFrustumCullingEnabled(true);
        {
            Frame frame(tree);
            CHECK(forest->getMeshInstanceCount() == 0 && forest->getImpostorInstanceCount() == 0);
        }

        CHECK(forest->getCastShadows() && forest->getMeshNode(0)->getCastShadows());
        forest->setCastShadows(false);
        CHECK(!forest->getCastShadows() && !forest->getMeshNode(0)->getCastShadows());
        forest->setCastShadows(true);
        CHECK(forest->getMeshNode(0)->getCastShadows());

        look(scene, 10.0f);
        {
            Frame frame(tree);
            CHECK(forest->getMeshInstanceCount() == 1);
        }
        CHECK(textures.count() == textureBaseline);
        forest->queueFree();
        {
            Frame frame(tree);
            CHECK(frame.stats().drawCalls == 0);
            CHECK(frame.stats().instancesDrawn == 0);
            CHECK(frame.stats().impostorSets == 0);
        }
        CHECK(textures.count() == textureBaseline);
        CHECK(materials.count() == materialBaseline);
        CHECK(meshes.count() == meshBaseline);

        ImpostorId taken[kMaxImpostorCount];
        bool distinct = true;
        for (std::uint32_t i = 0; i < kMaxImpostorCount; ++i)
        {
            taken[i] = server->impostorCreate(marked, ct::Vector<MaterialId>(), 10.0f);
            CHECK(taken[i].valid());
            for (std::uint32_t j = 0; j < i; ++j)
                if (server->impostorGetSlot(taken[i]) == server->impostorGetSlot(taken[j]))
                    distinct = false;
        }
        CHECK(distinct);
        for (std::uint32_t i = 0; i < kMaxImpostorCount; ++i) server->impostorFree(taken[i]);
    }

    {
        Forest3D loose;
        const int species = loose.addSpecies(desc0);
        CHECK(species == 0);
        CHECK(!loose.getImpostorSet().valid());
        loose.plant(0, Math::Vec3(0.0f, 0.0f, 0.0f), 0.0f, 1.0f);
        CHECK(loose.update(Math::Vec3(0.0f, 0.5f, 5.0f)));
        CHECK(loose.getMeshInstanceCount() == 1 && loose.getImpostorInstanceCount() == 0);
        CHECK(!loose.update(Math::Vec3(0.0f, 0.5f, 5.5f)));
        CHECK(loose.getWrittenInstanceCount() == 0);
        CHECK(loose.update(Math::Vec3(0.0f, 0.5f, 600.0f)));
        CHECK(loose.getMeshInstanceCount() == 0 && loose.getImpostorInstanceCount() == 1);
        loose.setImpostorsEnabled(false);
        CHECK(loose.update(Math::Vec3(0.0f, 0.5f, 5.0f)));
        CHECK(loose.getMeshInstanceCount() == 1 && loose.getImpostorInstanceCount() == 0);
    }
    CHECK(textures.count() == textureBaseline);
    CHECK(materials.count() == materialBaseline);
    CHECK(meshes.count() == meshBaseline);

    {
        const std::uint32_t presets[3] = { 0, 4, 2 };
        const float heights[3] = { 12.0f, 16.0f, 3.0f };
        const int counts[3] = { 1200, 600, 1800 };
        float fits[3] = { 1.0f, 1.0f, 1.0f };

        Forest3D* big = root->createChild<Forest3D>();
        big->setSwapDistance(120.0f);
        for (int s = 0; s < 3; ++s)
        {
            TreeGeometry geometry;
            TreeGenerator::generate(TreeGenerator::preset(presets[s]).params, geometry);
            const Math::Box extent = treeBox(geometry);
            fits[s] = heights[s] / (extent.max.y - extent.min.y);
            ForestSpeciesDesc description;
            description.mesh = meshes.create("demo tree");
            CHECK(meshes.addSurface(description.mesh, geometry.bark));
            CHECK(meshes.addSurface(description.mesh, geometry.leaves));
            const MaterialId leaves = materials.create("demo leaves");
            materials.setTransparency(leaves, MaterialTransparency::AlphaScissor);
            description.materials.push_back(MaterialId());
            description.materials.push_back(leaves);
            TreeGeometry shadow;
            TreeGenerator::buildShadowGeometry(geometry, 6, shadow);
            description.shadowMesh = meshes.create("demo tree shadow");
            CHECK(meshes.addSurface(description.shadowMesh, shadow.bark));
            CHECK(meshes.addSurface(description.shadowMesh, shadow.leaves));
            CHECK(big->addSpecies(description) == s);
            CHECK(big->getImpostor(s).valid());
        }
        Random random;
        random.seed(20261010u);
        plantDemoForest(*big, fits, counts, random);
        CHECK(big->getInstanceCount() == 3600);

        MeshInstance3D* ground = root->createChild<MeshInstance3D>();
        ground->setMesh(createPlaneMesh(meshes, 1400.0f, 1400.0f));
        DirectionalLight3D* sun = root->createChild<DirectionalLight3D>();
        sun->setShadow(true);
        sun->setShadowMaxDistance(120.0f);
        sun->setRotation(Math::Vec3(-0.9f, 0.6f, 0.0f));
        scene.camera->setPerspective(60.0f, 0.1f, 600.0f);
        place(scene, Math::Vec3(0.0f, 8.0f, 100.0f), Math::Vec3(0.0f, 4.0f, 0.0f));

        CHECK(big->isFrustumCullingEnabled());
        CHECK(fabsf(big->getShadowCullMargin() - 2.0f) < 1e-6f);
        big->setFrustumCullingEnabled(false);
        RenderStats on;
        RenderStats off;
        int meshOn = 0;
        int cardOn = 0;
        int meshOff = 0;
        for (int pass = 0; pass < 2; ++pass)
        {
            big->setImpostorsEnabled(pass == 0);
            {
                Frame frame(tree);
            }
            Frame frame(tree);
            CHECK(big->getWrittenInstanceCount() == 0);
            CHECK(frame.stats().multimeshUploadBytes == 0);
            if (pass == 0)
            {
                on = frame.stats();
                meshOn = big->getMeshInstanceCount();
                cardOn = big->getImpostorInstanceCount();
            }
            else
            {
                off = frame.stats();
                meshOff = big->getMeshInstanceCount();
            }
        }
        printf("demo forest, 3600 trees, camera (0, 8, 100), swap 120:\n");
        printf("  impostors on : mesh %d, cards %d, draws %u, triangles %u, shadow draws %u, "
               "shadow triangles %u, impostor sets %u, impostor draws %u\n",
                meshOn, cardOn, on.drawCalls, on.triangles, on.shadowDrawCalls, on.shadowTriangles,
                on.impostorSets, on.impostorDrawCalls);
        printf("  impostors off: mesh %d, draws %u, triangles %u, shadow draws %u, shadow "
               "triangles %u\n",
                meshOff, off.drawCalls, off.triangles, off.shadowDrawCalls, off.shadowTriangles);
        CHECK(on.impostorSets == 1 && on.impostorDrawCalls == 1);
        CHECK(off.impostorSets == 0 && off.impostorDrawCalls == 0);
        CHECK(server->impostorSetGetCount(big->getImpostorSet()) == 3600);
        CHECK(meshOn > 0 && cardOn > 0 && meshOn < meshOff);
        CHECK(on.triangles < off.triangles);
        CHECK(on.shadowTriangles <= off.shadowTriangles);
        CHECK(meshOff == big->getInstanceCount());

        big->setImpostorsEnabled(true);
        big->setFrustumCullingEnabled(true);
        {
            Frame frame(tree);
        }
        int meshCulled = 0;
        int cardCulled = 0;
        RenderStats culled;
        {
            Frame frame(tree);
            CHECK(big->getWrittenInstanceCount() == 0);
            culled = frame.stats();
            meshCulled = big->getMeshInstanceCount();
            cardCulled = big->getImpostorInstanceCount();
        }
        const int cardsGpu = cardsOnGpu(big->getImpostorSet());
        printf("  with culling : mesh %d, cards %d (compute %d), draws %u, triangles %u, shadow "
               "draws %u, shadow triangles %u, impostor sets %u, impostor draws %u\n",
                meshCulled, cardCulled, cardsGpu, culled.drawCalls, culled.triangles,
                culled.shadowDrawCalls, culled.shadowTriangles, culled.impostorSets,
                culled.impostorDrawCalls);
        CHECK(abs(cardsGpu - cardCulled) <= 2);
        CHECK(culled.impostorSets == 1 && culled.impostorDrawCalls == 1);
        CHECK(meshCulled > 0 && meshCulled < meshOn);
        CHECK(cardCulled > 0 && cardCulled < cardOn);
        CHECK(culled.triangles < on.triangles);
        CHECK(culled.shadowTriangles <= on.shadowTriangles);

        place(scene, Math::Vec3(0.0f, 8.0f, 100.0f), Math::Vec3(0.0f, 4.0f, 900.0f));
        {
            Frame frame(tree);
            CHECK(big->getWrittenInstanceCount() > 0);
        }
        const int meshAway = big->getMeshInstanceCount();
        const int cardAway = big->getImpostorInstanceCount();
        CHECK(meshAway != meshCulled || cardAway != cardCulled);
        CHECK(meshAway + cardAway < meshOn + cardOn);

        big->setShadowCullMargin(0.0f);
        {
            Frame frame(tree);
        }
        CHECK(big->getMeshInstanceCount() <= meshAway);
        big->setShadowCullMargin(-1.0f);
        CHECK(big->getShadowCullMargin() == 0.0f);
        big->queueFree();
        ground->queueFree();
        sun->queueFree();
        {
            Frame frame(tree);
        }
    }

    Main::device()->requestClose();
    tree.finalize();
    Main::cleanup();

    if (failures)
    {
        printf("%d checks failed\n", failures);
        return 1;
    }
    printf("forest ok\n");
    return 0;
}
