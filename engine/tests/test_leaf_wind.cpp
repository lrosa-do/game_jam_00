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
#include "prisma3d/scene/MultiMesh.h"
#include "prisma3d/scene/MultiMeshInstance3D.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/Viewport.h"
#include "prisma3d/scene/VisualInstance3D.h"
#include "prisma3d/scene/WorldEnvironment.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

using namespace prisma3d;

namespace
{

const int kWidth = 128;
const int kHeight = 96;
const float kPi = 3.14159265358979f;
const float kCardHeight = 2.0f;
const float kCameraDistance = 6.0f;

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

struct Image
{
    unsigned char pixels[kWidth * kHeight * 4];

    bool read()
    {
        prisma::Rect rect;
        rect.x = 0;
        rect.y = 0;
        rect.width = kWidth;
        rect.height = kHeight;
        return Main::device()->driver()->readPixels(prisma::RenderTarget(), rect, pixels);
    }

    const unsigned char* at(int x, int y) const
    {
        return pixels + (y * kWidth + x) * 4;
    }

    bool isClear(int x, int y) const
    {
        const unsigned char* p = at(x, y);
        return abs(p[0] - 26) <= 3 && abs(p[1] - 51) <= 3 && abs(p[2] - 77) <= 3;
    }
};

struct Scene
{
    SceneTree* tree = nullptr;
    Node* root = nullptr;
    Camera3D* camera = nullptr;
};

struct Shot
{
    Image image;
    RenderStats stats;
};

bool grab(Scene& scene, Shot& shot)
{
    Frame frame(*scene.tree);
    shot.stats = Main::rendering()->getStats();
    return shot.image.read();
}

int differentPixels(const Image& a, const Image& b)
{
    int different = 0;
    for (int y = 0; y < kHeight; ++y)
        for (int x = 0; x < kWidth; ++x)
            if (memcmp(a.at(x, y), b.at(x, y), 3) != 0) ++different;
    return different;
}

void rows(const Image& image, int& minY, int& maxY)
{
    minY = kHeight;
    maxY = -1;
    for (int y = 0; y < kHeight; ++y)
        for (int x = 0; x < kWidth; ++x)
            if (!image.isClear(x, y))
            {
                if (y < minY) minY = y;
                if (y > maxY) maxY = y;
            }
}

float centroid(const Image& image, int firstRow, int lastRow, int& count)
{
    float sum = 0.0f;
    count = 0;
    for (int y = firstRow; y <= lastRow; ++y)
        for (int x = 0; x < kWidth; ++x)
            if (!image.isClear(x, y))
            {
                sum += static_cast<float>(x);
                ++count;
            }
    return count > 0 ? sum / static_cast<float>(count) : 0.0f;
}

float expectedTopOffset(float instanceX, float strength, float directionX)
{
    float sum = 0.0f;
    for (int corner = 0; corner < 2; ++corner)
    {
        const float x = corner == 0 ? -1.0f : 1.0f;
        const float phase = 0.71f * x + 0.33f * 2.0f + instanceX * 0.12f;
        const float wave = sinf(phase) + 0.5f * sinf(phase * 1.7f);
        sum += directionX * wave * (strength * 1.0f * 1.0f * 0.06f);
    }
    return sum * 0.5f;
}

float pixelsPerMetre()
{
    return static_cast<float>(kHeight) / (2.0f * kCameraDistance * tanf(kPi / 6.0f));
}

MeshId makeCard(MeshStore& meshes)
{
    SurfaceData surface;
    for (int corner = 0; corner < 4; ++corner)
    {
        const float x = (corner & 1) ? 1.0f : -1.0f;
        const float y = (corner >> 1) ? kCardHeight : 0.0f;
        surface.positions.push_back(Math::Vec3(x, y, 0.0f));
        surface.normals.push_back(Math::Vec3(0.0f, 0.0f, 1.0f));
        surface.colors.push_back(Math::Vec4(1.0f, 1.0f, 1.0f, 1.0f));
        surface.uvs.push_back(Math::Vec2(x * 0.5f + 0.5f, 1.0f - y / kCardHeight));
    }
    surface.indices.push_back(0);
    surface.indices.push_back(1);
    surface.indices.push_back(2);
    surface.indices.push_back(2);
    surface.indices.push_back(1);
    surface.indices.push_back(3);
    const MeshId mesh = meshes.create("leaf card");
    CHECK(meshes.addSurface(mesh, surface, "card"));
    return mesh;
}

MaterialId makeLeaf(MaterialStore& materials, TextureId texture, bool wind, float strength)
{
    const MaterialId material = materials.create("leaf");
    CHECK(materials.setTexture(material, MaterialTexture::Albedo, texture));
    CHECK(materials.setTransparency(material, MaterialTransparency::AlphaScissor));
    CHECK(materials.setCull(material, MaterialCull::Disabled));
    CHECK(materials.setFloat(material, MaterialParam::Roughness, 1.0f));
    CHECK(materials.setFloat(material, MaterialParam::Specular, 0.0f));
    if (wind)
        CHECK(materials.setLeafWind(material, true, kCardHeight, strength, 0.0f,
                Math::Vec3(1.0f, 0.0f, 0.0f)));
    return material;
}

void testStore(MaterialStore& materials)
{
    const MaterialId material = materials.create("store");
    const MaterialInfo* info = materials.get(material);
    CHECK(info != nullptr);
    if (!info) return;
    CHECK(!info->leafWind);
    CHECK(info->leafWindStrength == 1.0f && info->leafWindSpeed == 1.0f);
    const std::uint32_t before = info->version;
    CHECK(materials.setLeafWind(material, true, 12.0f, 0.5f, 2.0f, Math::Vec3(0.0f, 0.0f, 1.0f)));
    info = materials.get(material);
    CHECK(info->leafWind && info->leafWindHeight == 12.0f && info->leafWindStrength == 0.5f);
    CHECK(info->leafWindSpeed == 2.0f && info->leafWindDirection.z == 1.0f);
    CHECK(info->version != before);
    CHECK(!materials.setLeafWind(MaterialId(), true, 1.0f));
    materials.destroy(material);
}

void testCard(Scene& scene, MeshStore& meshes, MaterialStore& materials, TextureId texture)
{
    const MeshId card = makeCard(meshes);
    const MaterialId still = makeLeaf(materials, texture, false, 0.0f);
    const MaterialId windy = makeLeaf(materials, texture, true, 12.0f);
    const MaterialId calm = makeLeaf(materials, texture, true, 0.0f);

    MeshInstance3D* node = scene.root->createChild<MeshInstance3D>();
    node->setMesh(card);
    node->setMaterialOverride(still);

    Main::rendering()->setDepthPrePassEnabled(true);
    Shot rest;
    CHECK(grab(scene, rest));
    int top = 0;
    int bottom = 0;
    rows(rest.image, top, bottom);
    CHECK(bottom - top > 20);

    node->setMaterialOverride(calm);
    Shot calmShot;
    CHECK(grab(scene, calmShot));
    const int calmDifferent = differentPixels(rest.image, calmShot.image);
    printf("leaf wind: strength 0 differs from no wind in %d pixels\n", calmDifferent);
    CHECK(calmDifferent == 0);

    node->setMaterialOverride(windy);
    Shot withPrepass;
    CHECK(grab(scene, withPrepass));
    CHECK(withPrepass.stats.depthPrePassDrawCalls > 0);
    Main::rendering()->setDepthPrePassEnabled(false);
    Shot withoutPrepass;
    CHECK(grab(scene, withoutPrepass));
    CHECK(withoutPrepass.stats.depthPrePassDrawCalls == 0);
    Main::rendering()->setDepthPrePassEnabled(true);
    const int prepassDifferent = differentPixels(withPrepass.image, withoutPrepass.image);
    printf("leaf wind: pre-pass on and off differ in %d pixels\n", prepassDifferent);
    CHECK(prepassDifferent == 0);

    int topCount = 0;
    int restTopCount = 0;
    const float restTop = centroid(rest.image, top + 1, top + 3, restTopCount);
    const float windTop = centroid(withPrepass.image, top + 1, top + 3, topCount);
    CHECK(topCount > 0 && restTopCount > 0);
    const float shift = windTop - restTop;
    const float expected = expectedTopOffset(0.0f, 12.0f, 1.0f) * pixelsPerMetre() * 0.9f;
    printf("leaf wind: top rows moved %.2f pixels, expected about %.2f\n", shift, expected);
    CHECK(fabsf(expected) > 2.0f);
    CHECK(shift * expected > 0.0f);
    CHECK(fabsf(shift - expected) < 0.3f * fabsf(expected) + 1.0f);

    int baseCount = 0;
    int restBaseCount = 0;
    const float restBase = centroid(rest.image, bottom - 2, bottom, restBaseCount);
    const float windBase = centroid(withPrepass.image, bottom - 2, bottom, baseCount);
    printf("leaf wind: base rows moved %.2f pixels\n", windBase - restBase);
    CHECK(baseCount > 0 && restBaseCount > 0);
    CHECK(fabsf(windBase - restBase) < 1.0f);

    node->queueFree();
    {
        Frame frame(*scene.tree);
    }
    materials.destroy(still);
    materials.destroy(windy);
    materials.destroy(calm);
    meshes.destroy(card);
}

void testMultiMesh(Scene& scene, MeshStore& meshes, MaterialStore& materials, TextureId texture)
{
    const MeshId card = makeCard(meshes);
    const MaterialId still = makeLeaf(materials, texture, false, 0.0f);
    const MaterialId windy = makeLeaf(materials, texture, true, 12.0f);

    MultiMeshInstance3D* node = scene.root->createChild<MultiMeshInstance3D>();
    MultiMesh* multimesh = node->getMultiMesh();
    CHECK(multimesh != nullptr);
    if (!multimesh) return;
    multimesh->setMesh(card);
    multimesh->setInstanceCount(1);
    multimesh->setInstanceTransform(0, Math::Mat4::Translation(Math::Vec3(2.0f, 0.0f, 0.0f)));
    node->setSurfaceMaterial(0, still);

    Main::rendering()->setDepthPrePassEnabled(true);
    Shot rest;
    CHECK(grab(scene, rest));
    int top = 0;
    int bottom = 0;
    rows(rest.image, top, bottom);
    CHECK(bottom - top > 20);

    node->setSurfaceMaterial(0, windy);
    Shot withPrepass;
    CHECK(grab(scene, withPrepass));
    CHECK(withPrepass.stats.depthPrePassDrawCalls > 0);
    Main::rendering()->setDepthPrePassEnabled(false);
    Shot withoutPrepass;
    CHECK(grab(scene, withoutPrepass));
    Main::rendering()->setDepthPrePassEnabled(true);
    const int prepassDifferent = differentPixels(withPrepass.image, withoutPrepass.image);
    printf("leaf wind: multimesh pre-pass on and off differ in %d pixels\n", prepassDifferent);
    CHECK(prepassDifferent == 0);

    int topCount = 0;
    int restTopCount = 0;
    const float restTop = centroid(rest.image, top + 1, top + 3, restTopCount);
    const float windTop = centroid(withPrepass.image, top + 1, top + 3, topCount);
    CHECK(topCount > 0 && restTopCount > 0);
    const float shift = windTop - restTop;
    const float expected = expectedTopOffset(2.0f, 12.0f, 1.0f) * pixelsPerMetre() * 0.9f;
    printf("leaf wind: multimesh instance at x 2, top rows moved %.2f pixels, expected about "
           "%.2f\n",
            shift, expected);
    CHECK(fabsf(expected) > 2.0f);
    CHECK(shift * expected > 0.0f);
    CHECK(fabsf(shift - expected) < 0.3f * fabsf(expected) + 1.0f);

    node->queueFree();
    {
        Frame frame(*scene.tree);
    }
    materials.destroy(still);
    materials.destroy(windy);
    meshes.destroy(card);
}

} // namespace

int main(int argc, char** argv)
{
    const bool vulkan = argc > 1 && strcmp(argv[1], "vulkan") == 0;

    CHECK(Device::headlessSupported());
    if (!Device::headlessSupported()) return 1;

    DeviceDesc desc;
    desc.title = "leaf wind test";
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
    scene.camera->setPerspective(60.0f, 0.1f, 100.0f);
    scene.camera->reparent(root);
    scene.camera->makeCurrent();
    scene.camera->setGlobalTransform(
            Math::Mat4::LookAt(Math::Vec3(0.0f, kCardHeight * 0.5f, kCameraDistance),
                    Math::Vec3(0.0f, kCardHeight * 0.5f, 0.0f), Math::Vec3(0.0f, 1.0f, 0.0f))
                    .InverseAffine());

    MeshStore& meshes = *Main::meshes();
    TextureStore& textures = *Main::textures();
    MaterialStore& materials = *Main::materials();
    materials.defaultMaterial();
    {
        Frame frame(tree);
    }

    Pixmap pixels(16, 16, 4);
    for (int y = 0; y < 16; ++y)
        for (int x = 0; x < 16; ++x)
            pixels.set_pixel(static_cast<std::uint32_t>(x), static_cast<std::uint32_t>(y), 255, 255,
                    255, 255);
    const TextureId texture = textures.createFromPixmap(pixels, true, true, "leaf", 4);

    const std::size_t meshBaseline = meshes.count();
    const std::size_t materialBaseline = materials.count();

    testStore(materials);
    testCard(scene, meshes, materials, texture);
    testMultiMesh(scene, meshes, materials, texture);

    CHECK(meshes.count() == meshBaseline);
    CHECK(materials.count() == materialBaseline);
    textures.destroy(texture);

    Main::device()->requestClose();
    tree.finalize();
    Main::cleanup();

    if (failures)
    {
        printf("%d checks failed\n", failures);
        return 1;
    }
    printf("leaf wind ok\n");
    return 0;
}
