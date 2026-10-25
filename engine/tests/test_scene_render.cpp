#include "Check.h"
#include "LitTest.h"

#include "prisma3d/Device.h"
#include "prisma3d/core/Pixmap.h"
#include "prisma3d/main/Engine.h"
#include "prisma3d/main/Main.h"
#include "prisma3d/render/DecalAtlas.h"
#include "prisma3d/render/MaterialStore.h"
#include "prisma3d/render/MeshStore.h"
#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/scene/Camera3D.h"
#include "prisma3d/scene/Decal3D.h"
#include "prisma3d/scene/Light3D.h"
#include "prisma3d/scene/MultiMeshInstance3D.h"
#include "prisma3d/scene/PrimitiveMeshes.h"
#include "prisma3d/scene/RibbonTrail3D.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/Viewport.h"
#include "prisma3d/scene/VisualInstance3D.h"
#include "prisma3d/scene/WorldEnvironment.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

using namespace prisma3d;

namespace
{

const int kWidth = 64;
const int kHeight = 48;
const int kTolerance = 2;

struct Rgb
{
    int r;
    int g;
    int b;
};

const Rgb kClear = { 26, 51, 77 };
const Rgb kRed = { 255, 0, 0 };
const Rgb kBlue = { 0, 0, 255 };
const Rgb kGreen = { 0, 255, 0 };

bool pixelWithin(int x, int y, Rgb expected, int tolerance, const char* label)
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
    const int dr = pixel[0] - expected.r;
    const int dg = pixel[1] - expected.g;
    const int db = pixel[2] - expected.b;
    const bool same = dr >= -tolerance && dr <= tolerance && dg >= -tolerance && dg <= tolerance &&
                      db >= -tolerance && db <= tolerance;
    if (!same)
        printf("%s: pixel (%d,%d) is %d,%d,%d, expected %d,%d,%d\n", label, x, y, pixel[0],
                pixel[1], pixel[2], expected.r, expected.g, expected.b);
    return same;
}

bool pixelIs(int x, int y, Rgb expected, const char* label)
{
    return pixelWithin(x, y, expected, kTolerance, label);
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

bool near(float a, float b)
{
    return fabsf(a - b) < 0.0001f;
}

} // namespace

int main(int argc, char** argv)
{
    const bool vulkan = argc > 1 && strcmp(argv[1], "vulkan") == 0;

    CHECK(Device::headlessSupported());
    if (!Device::headlessSupported()) return 1;

    DeviceDesc desc;
    desc.title = "scene render test";
    desc.driver = vulkan ? prisma::DriverType::Vulkan : prisma::DriverType::OpenGL;
    desc.width = kWidth;
    desc.height = kHeight;
    desc.headless = true;

    CHECK(Main::setup(desc));
    if (!Main::device()) return 1;
    CHECK(Main::engine() != nullptr);
    CHECK(Main::meshes() != nullptr);
    CHECK(Main::textures() != nullptr);
    CHECK(Main::materials() != nullptr);
    CHECK(Main::rendering() != nullptr);
    CHECK(Main::assets() != nullptr);
    CHECK(!Main::isStarted());

    {
        SurfaceData plain;
        boxMeshArrays(plain, Math::Vec3(1.0f, 1.0f, 1.0f));
        CHECK(plain.positions.size() == 24);
        CHECK(plain.normals.size() == 24);
        CHECK(plain.tangents.size() == 24);
        CHECK(plain.uvs.size() == 24);
        CHECK(plain.colors.empty());
        CHECK(plain.indices.size() == 36);
        const Math::Box box = MeshStore::computeAabb(plain.positions);
        CHECK(near(box.min.x, -0.5f) && near(box.min.y, -0.5f) && near(box.min.z, -0.5f));
        CHECK(near(box.max.x, 0.5f) && near(box.max.y, 0.5f) && near(box.max.z, 0.5f));
        for (std::size_t i = 0; i < plain.indices.size(); ++i)
            CHECK(plain.indices[i] < plain.positions.size());
        CHECK(near(plain.normals[0].z, 1.0f));
        CHECK(near(plain.uvs[1].x, 2.0f / 3.0f));

        SurfaceData sized;
        boxMeshArrays(sized, Math::Vec3(2.0f, 4.0f, 6.0f));
        const Math::Box sizedBox = MeshStore::computeAabb(sized.positions);
        CHECK(near(sizedBox.min.x, -1.0f) && near(sizedBox.max.y, 2.0f) &&
                near(sizedBox.max.z, 3.0f));

        SurfaceData divided;
        boxMeshArrays(divided, Math::Vec3(1.0f, 1.0f, 1.0f), 1, 1, 1);
        CHECK(divided.positions.size() == 54);
        CHECK(divided.indices.size() == 144);
        for (std::size_t i = 0; i < divided.indices.size(); ++i)
            CHECK(divided.indices[i] < divided.positions.size());
    }

    SceneTree tree;
    CHECK(Main::start(&tree));
    CHECK(Main::isStarted());
    CHECK(Main::tree() == &tree);

    Node* root = tree.getRoot();
    CHECK(root != nullptr);
    if (!root) return 1;
    Window* window = root->castTo<Window>();
    CHECK(window != nullptr);
    if (!window) return 1;
    CHECK(root->castTo<Viewport>() == window);
    CHECK(window->getWidth() == kWidth && window->getHeight() == kHeight);
    CHECK(near(window->getAspect(), 4.0f / 3.0f));
    CHECK(window->getViewportId().valid());
    CHECK(window->getScenario().valid());
    CHECK(Viewport::findViewport(window) == nullptr);
    window->setClearColor(Math::Vec4(0.1f, 0.2f, 0.3f, 1.0f));
    Node3D holder;
    WorldEnvironment* environment = holder.createChild<WorldEnvironment>();
    environment->setEnvironment(whiteAmbient());
    environment->reparent(root);

    Camera3D* camera = holder.createChild<Camera3D>();
    CHECK(camera->getCameraId().valid());
    CHECK(near(camera->getFov(), 75.0f) && near(camera->getNear(), 0.05f) &&
            near(camera->getFar(), 4000.0f));
    CHECK(camera->getCullMask() == 0xffffffffu);
    CHECK(camera->getProjection() == CameraProjection::Perspective);
    camera->setPosition(Math::Vec3(0.0f, 0.0f, 3.0f));
    CHECK(!camera->isCurrent());
    camera->reparent(root);
    CHECK(Viewport::findViewport(camera) == window);
    CHECK(camera->getViewport() == window);
    camera->makeCurrent();
    CHECK(camera->isCurrent());
    CHECK(window->getCamera3D() == camera);

    const MeshId boxMesh = createBoxMesh(*Main::meshes(), Math::Vec3(1.0f, 1.0f, 1.0f));
    CHECK(boxMesh.valid());
    const MeshInfo* info = Main::meshes()->get(boxMesh);
    CHECK(info != nullptr && info->surfaces.size() == 1);

    {
        const MeshId sphere = createSphereMesh(*Main::meshes(), 2.0f, 8, 12);
        CHECK(sphere.valid());
        const MeshInfo* sphereInfo = Main::meshes()->get(sphere);
        CHECK(sphereInfo != nullptr && sphereInfo->surfaces.size() == 1);
        if (sphereInfo)
        {
            CHECK(near(sphereInfo->aabb.min.x, -2.0f) && near(sphereInfo->aabb.max.x, 2.0f));
            CHECK(near(sphereInfo->aabb.min.y, -2.0f) && near(sphereInfo->aabb.max.y, 2.0f));
            CHECK(near(sphereInfo->aabb.min.z, -2.0f) && near(sphereInfo->aabb.max.z, 2.0f));
        }

        const MeshId torus = createTorusMesh(*Main::meshes(), 3.0f, 1.0f, 24, 12);
        CHECK(torus.valid());
        const MeshInfo* torusInfo = Main::meshes()->get(torus);
        CHECK(torusInfo != nullptr && torusInfo->surfaces.size() == 1);
        if (torusInfo)
        {
            CHECK(near(torusInfo->aabb.min.x, -4.0f) && near(torusInfo->aabb.max.x, 4.0f));
            CHECK(near(torusInfo->aabb.min.y, -1.0f) && near(torusInfo->aabb.max.y, 1.0f));
            CHECK(near(torusInfo->aabb.min.z, -4.0f) && near(torusInfo->aabb.max.z, 4.0f));
        }
    }

    MeshInstance3D* box = holder.createChild<MeshInstance3D>();
    CHECK(box->getInstance().valid());
    box->setMesh(boxMesh);
    CHECK(box->getMesh() == boxMesh && box->getBase() == boxMesh);
    box->setColor(Math::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
    CHECK(near(box->getAabb().max.x, 0.5f) && near(box->getAabb().min.z, -0.5f));
    CHECK(box->getLayerMask() == 1);
    box->setLayerMaskValue(3, true);
    CHECK(box->getLayerMaskValue(3) && box->getLayerMaskValue(1) && box->getLayerMask() == 5);
    box->setLayerMaskValue(3, false);
    CHECK(!box->getLayerMaskValue(3) && box->getLayerMask() == 1);
    box->reparent(root);

    {
        Frame frame(tree);
        CHECK(frame.stats().viewportsDrawn == 1);
        CHECK(frame.stats().instancesDrawn == 1);
        CHECK(frame.stats().instancesCulled == 0);
        CHECK(frame.stats().surfacesDrawn == 1);
        CHECK(frame.stats().drawCalls == 1);
        CHECK(pixelIs(32, 24, kRed, "box centre"));
        CHECK(pixelIs(1, 1, kClear, "corner"));
        CHECK(pixelIs(kWidth - 2, kHeight - 2, kClear, "opposite corner"));
    }

    box->setPosition(Math::Vec3(100.0f, 0.0f, 0.0f));
    {
        Frame frame(tree);
        CHECK(frame.stats().instancesCulled == 1);
        CHECK(frame.stats().instancesDrawn == 0);
        CHECK(pixelIs(32, 24, kClear, "culled box"));
    }

    box->setPosition(Math::Vec3(0.0f, 0.0f, 0.0f));
    box->hide();
    CHECK(!box->isVisible());
    {
        Frame frame(tree);
        CHECK(frame.stats().instancesDrawn == 0);
        CHECK(frame.stats().instancesCulled == 0);
        CHECK(pixelIs(32, 24, kClear, "hidden box"));
    }

    box->show();
    box->setColor(Math::Vec4(0.0f, 1.0f, 0.0f, 1.0f));
    {
        Frame frame(tree);
        CHECK(frame.stats().instancesDrawn == 1);
        CHECK(pixelIs(32, 24, kGreen, "shown green box"));
    }

    Camera3D* camera2 = holder.createChild<Camera3D>();
    camera2->reparent(root);
    camera2->setPosition(Math::Vec3(0.0f, 0.0f, -3.0f));
    camera2->lookAt(Math::Vec3(0.0f, 0.0f, 0.0f));
    CHECK(camera->isCurrent());
    CHECK(!camera2->isCurrent());
    CHECK(window->getCamera3D() == camera);
    {
        Frame frame(tree);
        CHECK(frame.stats().instancesDrawn == 1);
        CHECK(pixelIs(32, 24, kGreen, "second camera idle"));
    }

    box->setPosition(Math::Vec3(1.2f, 0.0f, 0.0f));
    {
        Frame frame(tree);
        CHECK(pixelIs(46, 24, kGreen, "first camera, box on the right"));
        CHECK(pixelIs(18, 24, kClear, "first camera, nothing on the left"));
    }

    camera2->makeCurrent();
    CHECK(camera2->isCurrent());
    CHECK(!camera->isCurrent());
    CHECK(window->getCamera3D() == camera2);
    {
        Frame frame(tree);
        CHECK(frame.stats().instancesDrawn == 1);
        CHECK(pixelIs(18, 24, kGreen, "second camera, box on the left"));
        CHECK(pixelIs(46, 24, kClear, "second camera, nothing on the right"));
    }

    camera2->clearCurrent();
    CHECK(camera->isCurrent());
    CHECK(!camera2->isCurrent());
    {
        Frame frame(tree);
        CHECK(pixelIs(46, 24, kGreen, "first camera again"));
        CHECK(pixelIs(18, 24, kClear, "first camera again, left"));
    }

    camera->clearCurrent(false);
    CHECK(!camera->isCurrent() && !camera2->isCurrent());
    CHECK(window->getCamera3D() == nullptr);
    {
        Frame frame(tree);
        CHECK(frame.stats().viewportsDrawn == 0);
    }
    camera->makeCurrent();
    CHECK(camera->isCurrent());

    box->reparent(&holder);
    CHECK(!box->isInsideTree());
    {
        Frame frame(tree);
        CHECK(frame.stats().viewportsDrawn == 1);
        CHECK(frame.stats().instancesDrawn == 0);
        CHECK(frame.stats().instancesCulled == 0);
        CHECK(pixelIs(46, 24, kClear, "removed box"));
    }

    box->reparent(root);
    {
        Frame frame(tree);
        CHECK(frame.stats().instancesDrawn == 1);
        CHECK(pixelIs(46, 24, kGreen, "re-added box"));
    }

    {
        MaterialStore& materials = *Main::materials();
        const MaterialId blue = materials.create("blue");
        const MaterialId red = materials.create("red");
        materials.setColor(blue, MaterialParam::Albedo, Math::Vec4(0.0f, 0.0f, 1.0f, 1.0f));
        materials.setColor(red, MaterialParam::Albedo, Math::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
        box->setColor(Math::Vec4(1.0f, 1.0f, 1.0f, 1.0f));
        CHECK(!box->getMaterialOverride().valid());
        CHECK(!box->getSurfaceMaterial(0).valid());
        {
            Frame frame(tree);
            CHECK(pixelIs(46, 24, Rgb{ 255, 255, 255 }, "box with the default material"));
        }

        box->setSurfaceMaterial(0, red);
        CHECK(box->getSurfaceMaterial(0) == red && !box->getSurfaceMaterial(1).valid());
        {
            Frame frame(tree);
            CHECK(pixelIs(46, 24, kRed, "box with a surface material"));
        }

        box->setMaterialOverride(blue);
        CHECK(box->getMaterialOverride() == blue);
        CHECK(Main::rendering()->instanceResolveMaterial(box->getInstance(), 0) == blue);
        {
            Frame frame(tree);
            CHECK(pixelIs(46, 24, kBlue, "box with a material override"));
        }

        box->reparent(&holder);
        box->reparent(root);
        {
            Frame frame(tree);
            CHECK(pixelIs(46, 24, kBlue, "override kept when re-entering the tree"));
        }

        box->setMaterialOverride(MaterialId());
        box->setSurfaceMaterial(0, MaterialId());
        CHECK(!box->getSurfaceMaterial(0).valid());
        box->setColor(Math::Vec4(0.0f, 1.0f, 0.0f, 1.0f));
        {
            Frame frame(tree);
            CHECK(pixelIs(46, 24, kGreen, "materials cleared"));
        }
    }

    {
        MaterialStore& materials = *Main::materials();
        const MaterialId matte = materials.create("matte");
        materials.setFloat(matte, MaterialParam::Specular, 0.0f);
        materials.setFloat(matte, MaterialParam::Roughness, 1.0f);
        box->setMaterialOverride(matte);
        const float pi = 3.14159265358979f;
        auto grey = [](float linear)
        {
            const float encoded = linear <= 0.0031308f
                                          ? linear * 12.92f
                                          : 1.055f * powf(linear, 1.0f / 2.4f) - 0.055f;
            return static_cast<int>(encoded * 255.0f + 0.5f);
        };
        auto omniAt = [](float distance, float radius)
        {
            float nd = distance / radius;
            nd *= nd;
            nd *= nd;
            nd = 1.0f - nd;
            nd *= nd;
            return nd / distance;
        };

        EnvironmentSettings dark;
        dark.ambientSource = AmbientSource::Color;
        environment->setEnvironment(dark);
        {
            Frame frame(tree);
            CHECK(frame.stats().lightsDrawn == 0);
            CHECK(pixelIs(46, 24, Rgb{ 0, 0, 0 }, "dark environment node"));
        }

        environment->reparent(&holder);
        {
            Frame frame(tree);
            CHECK(pixelIs(46, 24, Rgb{ 0, 51, 0 }, "no environment takes the clear colour"));
        }
        environment->reparent(root);
        {
            Frame frame(tree);
            CHECK(pixelIs(46, 24, Rgb{ 0, 0, 0 }, "environment node added again"));
        }

        DirectionalLight3D* sun = holder.createChild<DirectionalLight3D>();
        CHECK(sun->getLight().valid() && sun->getLightType() == LightType::Directional);
        CHECK(near(sun->getEnergy(), 1.0f) && near(sun->getSpecular(), 0.5f));
        CHECK(sun->getCullMask() == 0xffffffffu && !sun->isNegative());
        CHECK(sun->castTo<Light3D>() == sun && sun->castTo<OmniLight3D>() == nullptr);
        sun->setEnergy(0.5f);
        sun->reparent(root);
        {
            Frame frame(tree);
            CHECK(frame.stats().lightsDrawn == 1);
            CHECK(pixelIs(46, 24, Rgb{ 0, grey(0.5f), 0 }, "directional light node"));
        }

        sun->setRotation(Math::Vec3(0.0f, pi / 2.0f, 0.0f));
        {
            Frame frame(tree);
            CHECK(pixelIs(46, 24, Rgb{ 0, 0, 0 }, "directional light node turned away"));
        }
        sun->setRotation(Math::Vec3(0.0f, 0.0f, 0.0f));
        sun->hide();
        {
            Frame frame(tree);
            CHECK(frame.stats().lightsDrawn == 0);
            CHECK(pixelIs(46, 24, Rgb{ 0, 0, 0 }, "hidden light node"));
        }
        sun->show();
        sun->reparent(&holder);
        {
            Frame frame(tree);
            CHECK(frame.stats().lightsDrawn == 0);
            CHECK(pixelIs(46, 24, Rgb{ 0, 0, 0 }, "light node outside of the tree"));
        }
        sun->reparent(root);
        {
            Frame frame(tree);
            CHECK(pixelIs(46, 24, Rgb{ 0, grey(0.5f), 0 }, "light node back in the tree"));
        }
        sun->reparent(&holder);

        OmniLight3D* bulb = holder.createChild<OmniLight3D>();
        CHECK(bulb->getLightType() == LightType::Omni && near(bulb->getRange(), 5.0f));
        bulb->setRange(10.0f);
        bulb->setPosition(Math::Vec3(1.12f, 0.0f, 2.5f));
        bulb->reparent(root);
        {
            Frame frame(tree);
            CHECK(frame.stats().lightsDrawn == 1);
            CHECK(pixelIs(46, 24, Rgb{ 0, grey(omniAt(2.0f, 10.0f)), 0 }, "omni light node"));
        }
        bulb->setEnergy(0.5f);
        {
            Frame frame(tree);
            CHECK(pixelIs(46, 24, Rgb{ 0, grey(0.5f * omniAt(2.0f, 10.0f)), 0 },
                    "omni light node energy"));
        }
        bulb->reparent(&holder);

        SpotLight3D* lamp = holder.createChild<SpotLight3D>();
        CHECK(lamp->getLightType() == LightType::Spot && near(lamp->getSpotAngle(), 45.0f));
        lamp->setRange(10.0f);
        lamp->setSpotAngle(30.0f);
        lamp->setPosition(Math::Vec3(1.12f, 0.0f, 2.5f));
        lamp->reparent(root);
        {
            Frame frame(tree);
            CHECK(frame.stats().lightsDrawn == 1);
            CHECK(pixelIs(46, 24, Rgb{ 0, grey(omniAt(2.0f, 10.0f)), 0 }, "spot light node"));
        }
        lamp->setRotation(Math::Vec3(0.0f, pi, 0.0f));
        {
            Frame frame(tree);
            CHECK(pixelIs(46, 24, Rgb{ 0, 0, 0 }, "spot light node pointing away"));
        }
        lamp->reparent(&holder);
        lamp->queueFree();
        bulb->queueFree();
        sun->queueFree();

        {
            box->hide();
            EnvironmentSettings shadowed = dark;
            shadowed.reflectionSource = ReflectionSource::Disabled;
            environment->setEnvironment(shadowed);

            MeshInstance3D* floorBox = holder.createChild<MeshInstance3D>();
            floorBox->setMesh(createBoxMesh(*Main::meshes(), Math::Vec3(8.0f, 8.0f, 0.2f)));
            floorBox->setMaterialOverride(matte);
            floorBox->setPosition(Math::Vec3(0.0f, 0.0f, -1.0f));
            floorBox->reparent(root);
            MeshInstance3D* blocker = holder.createChild<MeshInstance3D>();
            blocker->setMesh(createBoxMesh(*Main::meshes(), Math::Vec3(0.6f, 0.6f, 0.6f)));
            blocker->setMaterialOverride(matte);
            blocker->setPosition(Math::Vec3(0.0f, 0.0f, 0.3f));
            blocker->reparent(root);
            CHECK(blocker->getCastShadows());

            DirectionalLight3D* shade = holder.createChild<DirectionalLight3D>();
            shade->setRotation(Math::Vec3(0.0f, pi / 4.0f, 0.0f));
            CHECK(!shade->hasShadow());
            CHECK(shade->getShadowMode() == DirectionalShadowMode::Parallel4Splits);
            CHECK(near(shade->getShadowBias(), 0.1f) && near(shade->getShadowNormalBias(), 2.0f));
            CHECK(near(shade->getShadowOpacity(), 1.0f) && near(shade->getShadowBlur(), 1.0f));
            CHECK(near(shade->getShadowMaxDistance(), 100.0f) &&
                    near(shade->getShadowSplit(2), 0.2f));
            CHECK(near(shade->getShadowFadeStart(), 0.8f) &&
                    near(shade->getShadowPancakeSize(), 20.0f));
            shade->reparent(root);
            Main::rendering()->setDirectionalShadowSize(1024, false);
            {
                Frame frame(tree);
                CHECK(frame.stats().shadowCascades == 0);
                CHECK(pixelIs(21, 24, Rgb{ grey(0.7071f), grey(0.7071f), grey(0.7071f) },
                        "floor lit without shadow"));
            }
            shade->setShadow(true);
            CHECK(shade->hasShadow());
            {
                Frame frame(tree);
                CHECK(frame.stats().shadowCascades == 4);
                CHECK(pixelIs(21, 24, Rgb{ 0, 0, 0 }, "floor in the shadow of the box"));
                CHECK(pixelIs(52, 24, Rgb{ grey(0.7071f), grey(0.7071f), grey(0.7071f) },
                        "floor beside the shadow"));
            }
            blocker->setCastShadows(false);
            CHECK(!blocker->getCastShadows());
            {
                Frame frame(tree);
                CHECK(pixelIs(21, 24, Rgb{ grey(0.7071f), grey(0.7071f), grey(0.7071f) },
                        "box that casts no shadow"));
            }
            blocker->setCastShadows(true);
            shade->setShadowMode(DirectionalShadowMode::Parallel2Splits);
            CHECK(shade->getShadowMode() == DirectionalShadowMode::Parallel2Splits);
            shade->setShadowSplit(1, 0.3f);
            CHECK(near(shade->getShadowSplit(1), 0.3f));
            {
                Frame frame(tree);
                CHECK(frame.stats().shadowCascades == 2);
                CHECK(pixelIs(21, 24, Rgb{ 0, 0, 0 }, "two cascades from the node"));
            }
            shade->reparent(&holder);
            blocker->reparent(&holder);
            floorBox->reparent(&holder);
            shade->queueFree();
            blocker->queueFree();
            floorBox->queueFree();
            box->show();
        }

        {
            box->hide();
            EnvironmentSettings shadowed = dark;
            shadowed.reflectionSource = ReflectionSource::Disabled;
            environment->setEnvironment(shadowed);
            window->setPositionalShadowAtlasSize(1024, false);
            CHECK(window->getPositionalShadowAtlasSize() == 1024);
            CHECK(!window->getPositionalShadowAtlasSixteenBits());
            CHECK(window->getPositionalShadowAtlasQuadrantSubdivision(0) == 4);
            CHECK(window->getPositionalShadowAtlasQuadrantSubdivision(3) == 64);
            window->setPositionalShadowAtlasQuadrantSubdivision(3, 16);
            CHECK(window->getPositionalShadowAtlasQuadrantSubdivision(3) == 16);
            window->setPositionalShadowAtlasQuadrantSubdivision(3, 64);

            MeshInstance3D* floorBox = holder.createChild<MeshInstance3D>();
            floorBox->setMesh(createBoxMesh(*Main::meshes(), Math::Vec3(8.0f, 8.0f, 0.2f)));
            floorBox->setMaterialOverride(matte);
            floorBox->setPosition(Math::Vec3(0.0f, 0.0f, -1.0f));
            floorBox->reparent(root);
            MeshInstance3D* blocker = holder.createChild<MeshInstance3D>();
            blocker->setMesh(createBoxMesh(*Main::meshes(), Math::Vec3(0.6f, 0.6f, 0.6f)));
            blocker->setMaterialOverride(matte);
            blocker->setPosition(Math::Vec3(0.0f, 0.0f, 0.3f));
            blocker->reparent(root);

            const Math::Vec3 lampPosition(-2.5f, 0.0f, 1.5f);
            const Math::Vec3 shadowTarget(2.5f, 0.0f, -0.9f);
            const Rgb unshadowed = Rgb{ grey(0.0622f), grey(0.0622f), grey(0.0622f) };
            const Rgb besideOmni = Rgb{ grey(0.0555f), grey(0.0555f), grey(0.0555f) };
            const Rgb besideSpot = Rgb{ grey(0.0462f), grey(0.0462f), grey(0.0462f) };

            SpotLight3D* lamp = holder.createChild<SpotLight3D>();
            lamp->setRange(10.0f);
            lamp->setSpotAngle(40.0f);
            lamp->lookAtFromPosition(lampPosition, shadowTarget);
            CHECK(!lamp->hasShadow());
            lamp->reparent(root);
            {
                Frame frame(tree);
                CHECK(frame.stats().shadowLights == 0);
                CHECK(pixelWithin(52, 24, unshadowed, 6, "spot node floor without shadow"));
            }
            lamp->setShadow(true);
            {
                Frame frame(tree);
                CHECK(frame.stats().shadowLights == 1);
                CHECK(frame.stats().shadowPasses == 1);
                CHECK(frame.stats().shadowCascades == 0);
                CHECK(pixelIs(52, 24, Rgb{ 0, 0, 0 }, "floor in the shadow of the spot node"));
                CHECK(pixelWithin(52, 10, besideSpot, 8, "floor beside the spot node shadow"));
            }
            lamp->setShadowOpacity(0.0f);
            {
                Frame frame(tree);
                CHECK(frame.stats().shadowLights == 0);
                CHECK(pixelWithin(52, 24, unshadowed, 6, "spot node shadow without opacity"));
            }
            lamp->setShadowOpacity(1.0f);
            blocker->setCastShadows(false);
            {
                Frame frame(tree);
                CHECK(pixelWithin(52, 24, unshadowed, 6, "spot node, blocker without shadow"));
            }
            blocker->setCastShadows(true);
            lamp->reparent(&holder);
            lamp->queueFree();

            OmniLight3D* bulb = holder.createChild<OmniLight3D>();
            CHECK(bulb->getShadowMode() == OmniShadowMode::Cube);
            bulb->setRange(10.0f);
            bulb->setPosition(lampPosition);
            bulb->setShadow(true);
            bulb->reparent(root);
            {
                Frame frame(tree);
                CHECK(frame.stats().shadowLights == 1);
                CHECK(frame.stats().shadowPasses == 6);
                CHECK(pixelIs(52, 24, Rgb{ 0, 0, 0 }, "floor in the cube shadow of the omni node"));
                CHECK(pixelWithin(52, 10, besideOmni, 8, "floor beside the cube shadow"));
            }
            bulb->setShadowMode(OmniShadowMode::DualParaboloid);
            CHECK(bulb->getShadowMode() == OmniShadowMode::DualParaboloid);
            floorBox->setCastShadows(false);
            {
                Frame frame(tree);
                CHECK(frame.stats().shadowLights == 1);
                CHECK(frame.stats().shadowPasses == 2);
                CHECK(pixelIs(52, 24, Rgb{ 0, 0, 0 },
                        "floor in the paraboloid shadow of the omni node"));
                CHECK(pixelWithin(52, 10, besideOmni, 8, "floor beside the paraboloid shadow"));
            }
            bulb->setShadow(false);
            {
                Frame frame(tree);
                CHECK(frame.stats().shadowLights == 0);
                CHECK(frame.stats().shadowPasses == 0);
                CHECK(pixelWithin(52, 24, unshadowed, 6, "omni node shadow disabled"));
            }
            bulb->reparent(&holder);
            blocker->reparent(&holder);
            floorBox->reparent(&holder);
            bulb->queueFree();
            blocker->queueFree();
            floorBox->queueFree();
            box->show();
        }

        box->setMaterialOverride(MaterialId());
        environment->setEnvironment(whiteAmbient());
    }

    {
        box->hide();
        Node3D* sword = root->createChild<Node3D>();
        sword->setName("Sword");
        sword->setPosition(Math::Vec3(-1.5f, 0.0f, 0.0f));
        Node3D* swordBase = sword->createChild<Node3D>();
        swordBase->setName("Base");
        swordBase->setPosition(Math::Vec3(0.0f, -0.5f, 0.0f));
        Node3D* swordTip = sword->createChild<Node3D>();
        swordTip->setName("Tip");
        swordTip->setPosition(Math::Vec3(0.0f, 0.5f, 0.0f));

        RibbonTrail3D* trail = root->createChild<RibbonTrail3D>();
        trail->setTrailLength(0.02f);
        CHECK(trail->getMaterial().valid());
        CHECK(trail->addBladeChain("../Sword/Base", "../Sword/Tip",
                      Math::Vec4(1.0f, 0.0f, 0.0f, 1.0f), Math::Vec4(1.0f, 0.0f, 0.0f, 1.0f), 10.0f,
                      1.0f, 1.0f) == 0);
        {
            Frame frame(tree);
            CHECK(trail->getGeometry().positions.empty());
            CHECK(pixelIs(32, 24, kClear, "trail before the blade moves"));
        }
        sword->setPosition(Math::Vec3(-1.0f, 0.0f, 0.0f));
        {
            Frame frame(tree);
            CHECK(trail->getElementCount(0) == 2);
            CHECK(pixelIs(32, 24, kClear, "trail after the first sample"));
        }
        sword->setPosition(Math::Vec3(1.0f, 0.0f, 0.0f));
        {
            Frame frame(tree);
            CHECK(trail->getElementCount(0) == 3);
            CHECK(trail->getGeometry().positions.size() == 18);
            CHECK(frame.stats().instancesDrawn == 1);
            CHECK(pixelIs(32, 24, Rgb{ 255, 51, 77 }, "additive trail swept across the centre"));
            CHECK(pixelIs(32, 4, kClear, "above the trail"));
        }
        trail->setEmitting(false);
        {
            Frame frame(tree);
            CHECK(pixelIs(32, 24, Rgb{ 255, 51, 77 }, "trail kept while it fades"));
        }
        trail->clearChains();
        {
            Frame frame(tree);
            CHECK(pixelIs(32, 24, kClear, "trail cleared"));
        }
        trail->queueFree();
        sword->queueFree();
        box->show();
    }

    {
        const Math::Mat4 saved = camera->getTransform();
        camera->setIdentity();
        camera->setPosition(Math::Vec3(0.0f, 0.0f, 3.0f));
        camera->setPerspective(60.0f, 0.1f, 100.0f);
        auto nearPoint = [](const Math::Vec3& a, float x, float y, float z)
        { return fabsf(a.x - x) < 1e-3f && fabsf(a.y - y) < 1e-3f && fabsf(a.z - z) < 1e-3f; };
        auto nearScreen = [](const Math::Vec2& a, float x, float y)
        { return fabsf(a.x - x) < 1e-2f && fabsf(a.y - y) < 1e-2f; };
        const Math::Vec2 centre(32.0f, 24.0f);
        const Math::Vec2 right(64.0f, 24.0f);
        const Math::Vec2 top(32.0f, 0.0f);

        CHECK(nearPoint(camera->projectRayOrigin(right), 0.0f, 0.0f, 3.0f));
        CHECK(nearPoint(camera->projectRayNormal(centre), 0.0f, 0.0f, -1.0f));
        CHECK(nearPoint(camera->projectLocalRayNormal(right), 0.6100f, 0.0f, -0.7924f));
        CHECK(nearPoint(camera->projectRayNormal(top), 0.0f, 0.5f, -0.8660f));
        const Math::Ray ray = camera->projectRay(right);
        CHECK(nearPoint(ray.origin, 0.0f, 0.0f, 3.0f));
        CHECK(nearPoint(ray.direction, 0.6100f, 0.0f, -0.7924f));
        CHECK(nearPoint(ray.PointAt(3.0f / 0.7924f), 2.3094f, 0.0f, 0.0f));
        const Math::Ray mouse = camera->projectMouseRay();
        const Math::Ray corner = camera->projectRay(window->getMousePosition());
        CHECK(nearPoint(mouse.direction, corner.direction.x, corner.direction.y,
                corner.direction.z));

        CHECK(nearPoint(camera->projectPosition(centre, 3.0f), 0.0f, 0.0f, 0.0f));
        CHECK(nearPoint(camera->projectPosition(right, 3.0f), 2.3094f, 0.0f, 0.0f));
        CHECK(nearPoint(camera->projectPosition(top, 3.0f), 0.0f, 1.7321f, 0.0f));
        CHECK(nearPoint(camera->projectPosition(right, 0.0f), 0.0f, 0.0f, 3.0f));
        CHECK(nearScreen(camera->unprojectPosition(Math::Vec3(0.0f, 0.0f, 0.0f)), 32.0f, 24.0f));
        CHECK(nearScreen(camera->unprojectPosition(Math::Vec3(2.3094f, 0.0f, 0.0f)), 64.0f, 24.0f));
        CHECK(nearScreen(camera->unprojectPosition(Math::Vec3(0.0f, 1.7321f, 0.0f)), 32.0f, 0.0f));
        const Math::Vec2 somewhere(11.0f, 37.0f);
        CHECK(nearScreen(camera->unprojectPosition(camera->projectPosition(somewhere, 7.5f)), 11.0f,
                37.0f));

        CHECK(camera->isPositionBehind(Math::Vec3(0.0f, 0.0f, 5.0f)));
        CHECK(!camera->isPositionBehind(Math::Vec3(0.0f, 0.0f, 0.0f)));
        CHECK(camera->isPositionInFrustum(Math::Vec3(0.0f, 0.0f, 0.0f)));
        CHECK(camera->isPositionInFrustum(Math::Vec3(2.2f, 0.0f, 0.0f)));
        CHECK(!camera->isPositionInFrustum(Math::Vec3(2.5f, 0.0f, 0.0f)));
        CHECK(!camera->isPositionInFrustum(Math::Vec3(0.0f, 0.0f, 5.0f)));
        CHECK(!camera->isPositionInFrustum(Math::Vec3(0.0f, 0.0f, -200.0f)));

        camera->rotateY(1.57079632679f);
        CHECK(nearPoint(camera->projectRayNormal(centre), -1.0f, 0.0f, 0.0f));
        CHECK(nearPoint(camera->projectPosition(centre, 2.0f), -2.0f, 0.0f, 3.0f));
        CHECK(nearScreen(camera->unprojectPosition(Math::Vec3(-2.0f, 0.0f, 3.0f)), 32.0f, 24.0f));
        camera->setIdentity();
        camera->setPosition(Math::Vec3(0.0f, 0.0f, 3.0f));

        camera->setOrthogonal(4.0f, 0.1f, 100.0f);
        CHECK(nearPoint(camera->projectRayNormal(right), 0.0f, 0.0f, -1.0f));
        CHECK(nearPoint(camera->projectRayOrigin(right), 2.6667f, 0.0f, 2.9f));
        CHECK(nearPoint(camera->projectRayOrigin(top), 0.0f, 2.0f, 2.9f));
        CHECK(nearPoint(camera->projectPosition(right, 3.0f), 2.6667f, 0.0f, 0.0f));
        CHECK(nearScreen(camera->unprojectPosition(Math::Vec3(2.6667f, 0.0f, 0.0f)), 64.0f, 24.0f));
        CHECK(nearScreen(camera->unprojectPosition(Math::Vec3(0.0f, 2.0f, 0.0f)), 32.0f, 0.0f));
        CHECK(camera->isPositionInFrustum(Math::Vec3(2.6f, 1.9f, 0.0f)));
        CHECK(!camera->isPositionInFrustum(Math::Vec3(2.7f, 0.0f, 0.0f)));

        camera->setPerspective(60.0f, 0.1f, 100.0f);
        box->hide();
        MeshInstance3D* target = root->createChild<MeshInstance3D>();
        target->setMesh(boxMesh);
        Math::Vec3 picked;
        {
            Frame frame(tree);
        }
        CHECK(camera->pickDepth(centre, picked));
        CHECK(nearPoint(picked, 0.0f, 0.0f, 0.5f));
        CHECK(camera->pickDepth(Math::Vec2(36.0f, 22.0f), picked));
        CHECK(fabsf(picked.z - 0.5f) < 1e-3f && picked.x > 0.0f && picked.y > 0.0f);
        CHECK(!camera->pickDepth(Math::Vec2(2.0f, 2.0f), picked));
        CHECK(!camera->pickDepth(Math::Vec2(-5.0f, 2.0f), picked));
        target->setPosition(Math::Vec3(0.0f, 0.0f, -4.0f));
        {
            Frame frame(tree);
        }
        CHECK(camera->pickDepth(centre, picked));
        CHECK(nearPoint(picked, 0.0f, 0.0f, -3.5f));

        camera->setOrthogonal(4.0f, 0.1f, 100.0f);
        {
            Frame frame(tree);
        }
        CHECK(camera->pickDepth(centre, picked));
        CHECK(nearPoint(picked, 0.0f, 0.0f, -3.5f));
        CHECK(!camera->pickDepth(Math::Vec2(2.0f, 2.0f), picked));
        target->queueFree();
        box->show();

        camera->setTransform(saved);
    }

    {
        RenderingServer* server = Main::rendering();
        const Math::Mat4 saved = camera->getTransform();
        camera->setRotation(Math::Vec3(0.0f, 0.0f, 0.0f));
        camera->setPosition(Math::Vec3(0.0f, 0.0f, 10.0f));
        auto nearPoint = [](const Math::Vec3& v, float x, float y, float z)
        { return fabsf(v.x - x) < 1e-3f && fabsf(v.y - y) < 1e-3f && fabsf(v.z - z) < 1e-3f; };
        camera->setPerspective(60.0f, 0.1f, 100.0f);

        Pixmap pixels(64, 64, 4);
        for (std::uint32_t y = 0; y < 64; ++y)
            for (std::uint32_t x = 0; x < 64; ++x) pixels.set_pixel(x, y, 255, 0, 0, 255);
        const TextureId stain = Main::textures()->createFromPixmap(pixels, true, false, "stain");
        const TextureId bump = Main::textures()->createFromPixmap(pixels, false, false, "bump");

        Decal3D* decal = root->createChild<Decal3D>();
        CHECK(decal->castTo<Decal3D>() == decal);
        CHECK(decal->getDecal().valid());
        CHECK(nearPoint(decal->getSize(), 2.0f, 2.0f, 2.0f));
        CHECK(near(decal->getEmissionEnergy(), 1.0f) && near(decal->getAlbedoMix(), 1.0f));
        CHECK(near(decal->getUpperFade(), 0.3f) && near(decal->getLowerFade(), 0.3f));
        CHECK(near(decal->getNormalFade(), 0.0f) && !decal->isDistanceFadeEnabled());
        CHECK(near(decal->getDistanceFadeBegin(), 40.0f));
        CHECK(near(decal->getDistanceFadeLength(), 10.0f));
        CHECK(decal->getCullMask() == 0xFFFFFu);
        CHECK(nearPoint(decal->getAabb().min, -1.0f, -1.0f, -1.0f));
        CHECK(nearPoint(decal->getAabb().max, 1.0f, 1.0f, 1.0f));

        {
            Frame frame(tree);
            CHECK(frame.stats().decalsDrawn == 1);
            CHECK(server->getDecalFrameCount() == 1);
            CHECK(server->getDecalFrameData()[0].mask == 0);
        }

        decal->setTexture(DecalTexture::Albedo, stain);
        decal->setTexture(DecalTexture::Normal, bump);
        CHECK(decal->getTexture(DecalTexture::Albedo) == stain);
        CHECK(server->getDecalAtlas().getTextureCount() == 2);
        decal->setSize(Math::Vec3(4.0f, 2.0f, 8.0f));
        decal->setPosition(Math::Vec3(0.0f, 0.0f, -10.0f));
        decal->setModulate(Math::Vec4(1.0f, 1.0f, 1.0f, 0.5f));
        decal->setEmissionEnergy(2.0f);
        decal->setAlbedoMix(0.75f);
        decal->setUpperFade(0.1f);
        decal->setLowerFade(-1.0f);
        decal->setNormalFade(0.25f);
        decal->setCullMask(3);
        CHECK(near(decal->getLowerFade(), 0.0f));
        const DecalParams* params = server->decalGetParams(decal->getDecal());
        CHECK(params != nullptr);
        if (params)
        {
            CHECK(nearPoint(params->size, 4.0f, 2.0f, 8.0f));
            CHECK(near(params->albedoMix, 0.75f) && near(params->emissionEnergy, 2.0f));
            CHECK(near(params->upperFade, 0.1f) && near(params->lowerFade, 0.0f));
            CHECK(near(params->normalFade, 0.25f) && params->cullMask == 3);
            CHECK(params->textures[0] == stain && params->textures[1] == bump);
        }
        {
            Frame frame(tree);
            CHECK(frame.stats().decalsDrawn == 1);
            const DecalDataGpu& dd = server->getDecalFrameData()[0];
            const Math::Vec4 rect = server->getDecalAtlas().getTextureRect(stain);
            CHECK(rect.z > 0.0f && rect.w > 0.0f);
            CHECK(near(dd.albedoRect[0], rect.x) && near(dd.albedoRect[1], rect.y) &&
                    near(dd.albedoRect[2], rect.z) && near(dd.albedoRect[3], rect.w));
            CHECK(dd.normalRect[2] > 0.0f && near(dd.ormRect[2], 0.0f));
            CHECK(dd.mask == 3 && near(dd.modulate[3], 0.5f) && near(dd.albedoMix, 0.75f));
            CHECK(near(dd.emissionEnergy, 2.0f) && near(dd.upperFade, 0.1f));
            CHECK(near(dd.normalFade, 0.25f) && near(dd.normal[1], 1.0f));
            const float x =
                    dd.xform[0] * 2.0f + dd.xform[4] * 1.0f + dd.xform[8] * -16.0f + dd.xform[12];
            const float y =
                    dd.xform[1] * 2.0f + dd.xform[5] * 1.0f + dd.xform[9] * -16.0f + dd.xform[13];
            const float z =
                    dd.xform[2] * 2.0f + dd.xform[6] * 1.0f + dd.xform[10] * -16.0f + dd.xform[14];
            CHECK(near(x, 1.0f) && near(y, 1.0f) && near(z, 1.0f));
        }

        decal->setDistanceFadeEnabled(true);
        decal->setDistanceFadeBegin(5.0f);
        decal->setDistanceFadeLength(5.0f);
        {
            Frame frame(tree);
            CHECK(frame.stats().decalsDrawn == 0);
        }
        decal->setDistanceFadeLength(30.0f);
        {
            Frame frame(tree);
            CHECK(frame.stats().decalsDrawn == 1);
            CHECK(near(server->getDecalFrameData()[0].modulate[3], 0.5f * 0.5f));
        }
        decal->setDistanceFadeEnabled(false);

        decal->hide();
        {
            Frame frame(tree);
            CHECK(frame.stats().decalsDrawn == 0);
        }
        decal->show();
        Decal3D* nearer = root->createChild<Decal3D>();
        nearer->setTexture(DecalTexture::Albedo, stain);
        nearer->setPosition(Math::Vec3(0.0f, 0.0f, 5.0f));
        nearer->setCullMask(9);
        CHECK(server->getDecalAtlas().getTextureCount() == 2);
        {
            Frame frame(tree);
            CHECK(frame.stats().decalsDrawn == 2);
            CHECK(server->getDecalFrameData()[0].mask == 9);
            CHECK(server->getDecalFrameData()[1].mask == 3);
        }
        nearer->setSortingOffset(-100.0f);
        {
            Frame frame(tree);
            CHECK(server->getDecalFrameData()[0].mask == 3);
            CHECK(server->getDecalFrameData()[1].mask == 9);
        }

        decal->queueFree();
        nearer->queueFree();
        {
            Frame frame(tree);
            CHECK(frame.stats().decalsDrawn == 0);
        }
        CHECK(server->getDecalAtlas().getTextureCount() == 0);

        auto readPixel = [](int x, int y, unsigned char* out)
        {
            prisma::Rect rect;
            rect.x = x;
            rect.y = y;
            rect.width = 1;
            rect.height = 1;
            return Main::device()->driver()->readPixels(prisma::RenderTarget(), rect, out);
        };
        box->hide();
        camera->setPosition(Math::Vec3(0.0f, 0.0f, 3.0f));
        MeshInstance3D* wall = root->createChild<MeshInstance3D>();
        wall->setMesh(boxMesh);
        wall->setScale(Math::Vec3(4.0f, 4.0f, 1.0f));
        const MaterialId dark = Main::materials()->create("dark wall");
        Main::materials()->setColor(dark, MaterialParam::Albedo,
                Math::Vec4(0.02f, 0.02f, 0.02f, 1.0f));
        Main::materials()->setFloat(dark, MaterialParam::Roughness, 1.0f);
        wall->setMaterialOverride(dark);
        unsigned char before[4] = { 0, 0, 0, 0 };
        unsigned char beforeSide[4] = { 0, 0, 0, 0 };
        {
            Frame frame(tree);
            CHECK(readPixel(32, 24, before));
            CHECK(readPixel(8, 24, beforeSide));
        }

        Pixmap greenPixels(32, 32, 4);
        for (std::uint32_t y = 0; y < 32; ++y)
            for (std::uint32_t x = 0; x < 32; ++x) greenPixels.set_pixel(x, y, 0, 255, 0, 255);
        const TextureId glow = Main::textures()->createFromPixmap(greenPixels, true, false, "glow");
        Decal3D* mark = root->createChild<Decal3D>();
        mark->setTexture(DecalTexture::Emission, glow);
        mark->setSize(Math::Vec3(0.6f, 1.0f, 0.6f));
        mark->setRotationDegrees(Math::Vec3(90.0f, 0.0f, 0.0f));
        mark->setPosition(Math::Vec3(0.0f, 0.0f, 0.5f));
        mark->setUpperFade(0.0f);
        mark->setLowerFade(0.0f);
        mark->setEmissionEnergy(4.0f);
        unsigned char lit[4] = { 0, 0, 0, 0 };
        unsigned char litSide[4] = { 0, 0, 0, 0 };
        {
            Frame frame(tree);
            CHECK(frame.stats().decalsDrawn == 1);
            CHECK(readPixel(32, 24, lit));
            CHECK(readPixel(8, 24, litSide));
        }
        CHECK(lit[1] > before[1] + 60);
        CHECK(lit[1] > lit[0] + 40 && lit[1] > lit[2] + 40);
        CHECK(abs(litSide[0] - beforeSide[0]) <= 2 && abs(litSide[1] - beforeSide[1]) <= 2 &&
                abs(litSide[2] - beforeSide[2]) <= 2);
        if (lit[1] <= before[1] + 60)
            printf("decal centre %d,%d,%d before %d,%d,%d\n", lit[0], lit[1], lit[2], before[0],
                    before[1], before[2]);

        mark->setCullMask(0);
        {
            Frame frame(tree);
            unsigned char masked[4] = { 0, 0, 0, 0 };
            CHECK(readPixel(32, 24, masked));
            CHECK(abs(masked[1] - before[1]) <= 2);
        }
        mark->setCullMask(0xFFFFFu);
        mark->setPosition(Math::Vec3(1.2f, 0.0f, 0.5f));
        {
            Frame frame(tree);
            unsigned char moved[4] = { 0, 0, 0, 0 };
            CHECK(readPixel(32, 24, moved));
            CHECK(abs(moved[1] - before[1]) <= 2);
        }
        mark->hide();
        {
            Frame frame(tree);
            unsigned char hidden[4] = { 0, 0, 0, 0 };
            CHECK(readPixel(32, 24, hidden));
            CHECK(abs(hidden[1] - before[1]) <= 2);
        }
        mark->queueFree();
        wall->queueFree();
        box->show();

        camera->setTransform(saved);
    }

    {
        box->hide();
        MultiMeshInstance3D* forest = root->createChild<MultiMeshInstance3D>();
        CHECK(forest->castTo<MultiMeshInstance3D>() == forest);
        CHECK(forest->castTo<VisualInstance3D>() == forest);
        CHECK(forest->getInstance().valid());
        CHECK(forest->getMultiMesh() != nullptr && forest->getMultiMesh()->getId().valid());
        CHECK(!forest->getBase().valid());
        CHECK(forest->getInstanceCount() == 0 && forest->getVisibleInstanceCount() == -1);
        CHECK(!forest->getAabb().Contains(Math::Vec3(0.0f, 0.0f, 0.0f)));
        CHECK(forest->setUseColors(true));
        CHECK(forest->getUseColors() && !forest->getUseCustomData());
        forest->setMesh(boxMesh);
        CHECK(forest->getMesh() == boxMesh);
        forest->setInstanceCount(3);
        CHECK(forest->getInstanceCount() == 3);
        CHECK(!forest->setUseColors(false));
        CHECK(!forest->setUseCustomData(true));
        CHECK(forest->getUseColors() && !forest->getUseCustomData());
        forest->setInstanceTransform(0, Math::Mat4::Translation(Math::Vec3(-1.2f, 0.0f, 0.0f)));
        forest->setInstanceTransform(1, Math::Mat4::Identity());
        forest->setInstanceTransform(2, Math::Mat4::Translation(Math::Vec3(1.2f, 0.0f, 0.0f)));
        forest->setInstanceColor(0, Math::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
        forest->setInstanceColor(1, Math::Vec4(0.0f, 1.0f, 0.0f, 1.0f));
        forest->setInstanceColor(2, Math::Vec4(0.0f, 0.0f, 1.0f, 1.0f));
        CHECK(near(forest->getInstanceTransform(2).GetTranslation().x, 1.2f));
        CHECK(near(forest->getInstanceColor(1).y, 1.0f));
        CHECK(!forest->setVisibleInstanceCount(4));
        CHECK(forest->getVisibleInstanceCount() == -1);
        {
            Frame frame(tree);
            CHECK(frame.stats().instancesDrawn == 1);
            CHECK(frame.stats().multimeshInstancesDrawn == 3);
            CHECK(frame.stats().drawCalls == 1);
            CHECK(pixelIs(17, 24, kRed, "multimesh node, left"));
            CHECK(pixelIs(32, 24, kGreen, "multimesh node, centre"));
            CHECK(pixelIs(47, 24, kBlue, "multimesh node, right"));
            CHECK(pixelIs(1, 1, kClear, "multimesh node, corner"));
        }
        {
            const Math::Box aabb = forest->getAabb();
            CHECK(near(aabb.min.x, -1.7f) && near(aabb.max.x, 1.7f));
            CHECK(near(aabb.min.y, -0.5f) && near(aabb.max.y, 0.5f));
            CHECK(near(aabb.min.z, -0.5f) && near(aabb.max.z, 0.5f));
        }
        CHECK(forest->setVisibleInstanceCount(2));
        {
            Frame frame(tree);
            CHECK(frame.stats().multimeshInstancesDrawn == 2);
            CHECK(pixelIs(17, 24, kRed, "multimesh node, visible count, kept"));
            CHECK(pixelIs(47, 24, kClear, "multimesh node, visible count, cut"));
        }
        forest->setVisibleInstanceCount(-1);
        forest->setPosition(Math::Vec3(0.0f, 1.0f, 0.0f));
        {
            Frame frame(tree);
            CHECK(frame.stats().multimeshInstancesDrawn == 3);
            CHECK(pixelIs(32, 24, kClear, "multimesh node moved, old place"));
            CHECK(pixelIs(32, 24 - 12, kGreen, "multimesh node moved"));
        }
        forest->setPosition(Math::Vec3(100.0f, 0.0f, 0.0f));
        {
            Frame frame(tree);
            CHECK(frame.stats().instancesCulled == 1);
            CHECK(frame.stats().drawCalls == 0);
        }
        forest->setPosition(Math::Vec3(0.0f, 0.0f, 0.0f));
        const MaterialId tinted = Main::materials()->create("multimesh tint");
        CHECK(Main::materials()->setColor(tinted, MaterialParam::Albedo,
                Math::Vec4(0.0f, 0.0f, 1.0f, 1.0f)));
        forest->setMaterialOverride(tinted);
        CHECK(forest->getMaterialOverride() == tinted);
        {
            Frame frame(tree);
            CHECK(pixelIs(17, 24, Rgb{ 0, 0, 0 }, "multimesh node, override times red"));
            CHECK(pixelIs(47, 24, kBlue, "multimesh node, override times blue"));
        }
        forest->queueFree();
        {
            Frame frame(tree);
            CHECK(frame.stats().instancesDrawn == 0);
            CHECK(pixelIs(32, 24, kClear, "multimesh node freed"));
        }
        box->show();
    }

    camera->setOrthogonal(4.0f, 0.1f, 100.0f);
    CHECK(camera->getProjection() == CameraProjection::Orthogonal);
    CHECK(near(camera->getCameraProjection().col0.x,
            RenderingServer::orthogonal(4.0f, window->getAspect(), 0.1f, 100.0f).col0.x));
    {
        Frame frame(tree);
        CHECK(frame.stats().instancesDrawn == 1);
        CHECK(pixelIs(32, 24, kClear, "orthogonal, empty centre"));
    }
    camera->setPerspective(75.0f, 0.05f, 4000.0f);

    const std::uint64_t processBefore = Main::engine()->getProcessFrames();
    std::uint64_t ticks = Main::device()->getTicksUsec();
    for (int i = 0; i < 5; ++i)
    {
        ticks += 16667;
        Main::device()->processEvents();
        Main::engine()->iteration(ticks);
    }
    CHECK(Main::engine()->getProcessFrames() > processBefore);

    Main::device()->requestClose();
    bool quit = false;
    for (int i = 0; i < 4 && !quit; ++i) quit = Main::iteration();
    CHECK(quit);

    tree.finalize();
    CHECK(tree.getRoot() == nullptr);
    Main::cleanup();
    CHECK(!Main::isStarted());
    CHECK(Main::device() == nullptr);

    if (failures)
    {
        printf("%d checks failed\n", failures);
        return 1;
    }
    printf("scene render ok\n");
    return 0;
}
