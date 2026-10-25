#include "Check.h"
#include "LitTest.h"

#include "prisma3d/Device.h"
#include "prisma3d/render/MaterialStore.h"
#include "prisma3d/render/MeshStore.h"
#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/render/TextureStore.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

using namespace prisma3d;

namespace
{

const int kWidth = 64;
const int kHeight = 48;
const int kTolerance = 2;
const float kEpsilon = 1e-4f;

struct Rgb
{
    int r;
    int g;
    int b;
};

const Rgb kClear = { 26, 51, 77 };
const Rgb kRed = { 255, 0, 0 };

bool pixelIs(prisma::Driver* driver, int x, int y, Rgb expected, const char* label)
{
    unsigned char pixel[4] = { 0, 0, 0, 0 };
    prisma::Rect rect;
    rect.x = x;
    rect.y = y;
    rect.width = 1;
    rect.height = 1;
    if (!driver->readPixels(prisma::RenderTarget(), rect, pixel))
    {
        printf("%s: pixel (%d,%d) could not be read\n", label, x, y);
        return false;
    }
    const int dr = pixel[0] - expected.r;
    const int dg = pixel[1] - expected.g;
    const int db = pixel[2] - expected.b;
    const bool same = dr >= -kTolerance && dr <= kTolerance && dg >= -kTolerance &&
                      dg <= kTolerance && db >= -kTolerance && db <= kTolerance;
    if (!same)
        printf("%s: pixel (%d,%d) is %d,%d,%d, expected %d,%d,%d\n", label, x, y, pixel[0],
                pixel[1], pixel[2], expected.r, expected.g, expected.b);
    return same;
}

bool near(float a, float b)
{
    return fabsf(a - b) <= kEpsilon;
}

bool nearVec3(const Math::Vec3& a, float x, float y, float z)
{
    return near(a.x, x) && near(a.y, y) && near(a.z, z);
}

std::uint32_t packJoints(unsigned a, unsigned b, unsigned c, unsigned d)
{
    return a | (b << 8) | (c << 16) | (d << 24);
}

SurfaceData skinnedQuad()
{
    SurfaceData data;
    data.positions.push_back(Math::Vec3(0.0f, 0.0f, 0.0f));
    data.positions.push_back(Math::Vec3(1.0f, 0.0f, 0.0f));
    data.positions.push_back(Math::Vec3(0.0f, 1.0f, 0.0f));
    data.positions.push_back(Math::Vec3(1.0f, 1.0f, 0.0f));
    for (int i = 0; i < 4; ++i)
    {
        data.normals.push_back(Math::Vec3(0.0f, 0.0f, 1.0f));
        data.colors.push_back(Math::Vec4(1.0f, 1.0f, 1.0f, 1.0f));
    }
    data.joints.push_back(packJoints(0, 0, 0, 0));
    data.joints.push_back(packJoints(0, 0, 0, 0));
    data.joints.push_back(packJoints(1, 0, 0, 0));
    data.joints.push_back(packJoints(0, 1, 0, 0));
    data.weights.push_back(Math::Vec4(1.0f, 0.0f, 0.0f, 0.0f));
    data.weights.push_back(Math::Vec4(1.0f, 0.0f, 0.0f, 0.0f));
    data.weights.push_back(Math::Vec4(1.0f, 0.0f, 0.0f, 0.0f));
    data.weights.push_back(Math::Vec4(0.5f, 0.5f, 0.0f, 0.0f));
    static const std::uint32_t indices[6] = { 0, 1, 2, 1, 3, 2 };
    for (int i = 0; i < 6; ++i) data.indices.push_back(indices[i]);
    return data;
}

SurfaceData plainQuad()
{
    SurfaceData data = skinnedQuad();
    data.joints.clear();
    data.weights.clear();
    return data;
}

struct Frames
{
    RenderingServer* server;
    bool open;

    Frames()
            : server(nullptr),
              open(false)
    {
    }

    void finish()
    {
        if (!open || !server) return;
        server->endFrame(true);
        open = false;
    }

    void draw(RenderingServer& renderer)
    {
        finish();
        server = &renderer;
        server->renderViewports();
        open = true;
    }
};

struct SkinnedVertices
{
    ct::Vector<std::uint8_t> bytes;
    std::uint32_t stride = 0;
    std::uint32_t mask = 0;

    const float* attribute(std::uint32_t vertex, VertexAttributeBit attribute) const
    {
        return reinterpret_cast<const float*>(
                bytes.data() + vertex * stride + MeshStore::attributeOffset(mask, attribute));
    }

    Math::Vec3 position(std::uint32_t vertex) const
    {
        const float* p = attribute(vertex, kVertexPosition);
        return Math::Vec3(p[0], p[1], p[2]);
    }

    Math::Vec3 normal(std::uint32_t vertex) const
    {
        const float* p = attribute(vertex, kVertexNormal);
        return Math::Vec3(p[0], p[1], p[2]);
    }

    std::uint32_t joints(std::uint32_t vertex) const
    {
        std::uint32_t value = 0;
        memcpy(&value, attribute(vertex, kVertexJoints), sizeof(value));
        return value;
    }
};

bool readSkinned(prisma::Driver* driver, const RenderingServer& server, InstanceId instance,
        const SurfaceInfo& surface, SkinnedVertices& out)
{
    const prisma::BufferHandle buffer = server.instanceSkinnedVertexBuffer(instance, 0);
    if (!buffer.valid()) return false;
    out.stride = surface.vertexStride;
    out.mask = surface.attributeMask;
    out.bytes.resize(surface.vertexCount * surface.vertexStride);
    return driver->readBuffer(buffer, 0, static_cast<std::uint32_t>(out.bytes.size()),
            out.bytes.data());
}

Math::Mat4 translation(float x, float y, float z)
{
    return Math::Mat4::Translation(Math::Vec3(x, y, z));
}

} // namespace

int main(int argc, char** argv)
{
    const bool vulkan = argc > 1 && strcmp(argv[1], "vulkan") == 0;

    CHECK(Device::headlessSupported());
    if (!Device::headlessSupported()) return 1;

    DeviceDesc deviceDesc;
    deviceDesc.title = "skinning test";
    deviceDesc.driver = vulkan ? prisma::DriverType::Vulkan : prisma::DriverType::OpenGL;
    deviceDesc.width = kWidth;
    deviceDesc.height = kHeight;
    deviceDesc.headless = true;

    Device* device = Device::create(deviceDesc);
    CHECK(device != nullptr);
    if (!device) return 1;

    prisma::Driver* driver = device->driver();
    CHECK(driver->caps().compute);

    {
        MeshStore meshes(driver);
        TextureStore textures(driver);
        MaterialStore materials(textures);
        RenderingServer server(driver, meshes, textures, materials);
        Frames frames;

        const MeshId skinnedMesh = meshes.create("skinned quad");
        CHECK(meshes.addSurface(skinnedMesh, skinnedQuad()));
        const MeshInfo* info = meshes.get(skinnedMesh);
        CHECK(info != nullptr && info->surfaces.size() == 1);
        if (!info || info->surfaces.size() != 1) return 1;
        const SurfaceInfo& surface = info->surfaces[0];
        CHECK(surface.skinned());
        CHECK(surface.boneAabbs.size() == 2);
        if (surface.boneAabbs.size() == 2)
        {
            CHECK(nearVec3(surface.boneAabbs[0].min, 0.0f, 0.0f, 0.0f));
            CHECK(nearVec3(surface.boneAabbs[0].max, 1.0f, 1.0f, 0.0f));
            CHECK(nearVec3(surface.boneAabbs[1].min, 0.0f, 1.0f, 0.0f));
            CHECK(nearVec3(surface.boneAabbs[1].max, 1.0f, 1.0f, 0.0f));
        }

        const MeshId plainMesh = meshes.create("plain quad");
        CHECK(meshes.addSurface(plainMesh, plainQuad()));
        const MeshInfo* plainInfo = meshes.get(plainMesh);
        CHECK(plainInfo && !plainInfo->surfaces.empty() && !plainInfo->surfaces[0].skinned());
        CHECK(plainInfo && !plainInfo->surfaces.empty() &&
                plainInfo->surfaces[0].boneAabbs.empty());

        const ScenarioId scenario = server.scenarioCreate();
        server.scenarioSetEnvironment(scenario, whiteAmbient());
        const InstanceId instance = server.instanceCreate(skinnedMesh, scenario);
        server.instanceSetColor(instance, Math::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
        const InstanceId plain = server.instanceCreate(plainMesh, scenario);
        server.instanceSetTransform(plain, translation(0.0f, -10.0f, 0.0f));

        const CameraId camera = server.cameraCreate();
        server.cameraSetOrthogonal(camera, 4.0f, 0.1f, 100.0f);
        server.cameraSetTransform(camera, translation(0.0f, 0.0f, 5.0f));

        const ViewportId viewport = server.viewportCreate();
        server.viewportSetSize(viewport, kWidth, kHeight);
        server.viewportAttachToScreen(viewport, true);
        server.viewportAttachCamera(viewport, camera);
        server.viewportSetScenario(viewport, scenario);
        server.viewportSetClearColor(viewport, Math::Vec4(0.1f, 0.2f, 0.3f, 1.0f));

        const SkeletonId skeleton = server.skeletonCreate();
        CHECK(skeleton.valid());
        CHECK(server.skeletonGetBoneCount(skeleton) == 0);
        server.skeletonAllocateData(skeleton, 2);
        CHECK(server.skeletonGetBoneCount(skeleton) == 2);
        CHECK(server.skeletonBoneGetTransform(skeleton, 1) == Math::Mat4::Identity());

        frames.draw(server);
        CHECK(server.getStats().skinDispatches == 0);
        CHECK(server.getStats().skinnedSurfaces == 0);
        CHECK(!server.instanceSkinnedVertexBuffer(instance, 0).valid());
        CHECK(!server.instanceGetSkeleton(instance).valid());
        CHECK(pixelIs(driver, 38, 18, kRed, "bind pose, no skeleton"));
        frames.finish();

        server.instanceAttachSkeleton(instance, skeleton);
        CHECK(server.instanceGetSkeleton(instance) == skeleton);
        server.instanceAttachSkeleton(plain, skeleton);

        server.skeletonBoneSetTransform(skeleton, 1, translation(0.0f, 0.0f, 2.0f));
        const Math::Mat4 stored = server.skeletonBoneGetTransform(skeleton, 1);
        CHECK(nearVec3(stored.GetTranslation(), 0.0f, 0.0f, 2.0f));

        Math::Box box = server.instanceGetAabb(instance);
        CHECK(nearVec3(box.min, 0.0f, 0.0f, 0.0f));
        CHECK(nearVec3(box.max, 1.0f, 1.0f, 2.0f));
        box = server.instanceGetAabb(plain);
        CHECK(nearVec3(box.min, 0.0f, -10.0f, 0.0f));
        CHECK(nearVec3(box.max, 1.0f, -9.0f, 0.0f));

        frames.draw(server);
        {
            const RenderStats& stats = server.getStats();
            CHECK(stats.skinDispatches == 1);
            CHECK(stats.skinnedSurfaces == 1);
            CHECK(stats.instancesDrawn == 1);
            CHECK(stats.instancesCulled == 1);
        }
        CHECK(!server.instanceSkinnedVertexBuffer(plain, 0).valid());
        CHECK(pixelIs(driver, 38, 18, kRed, "skinned, bone 1 lifted"));
        CHECK(server.getStats().depthPrePassSurfaces == 1);
        CHECK(server.getStats().depthPrePassDrawCalls == 1);
        CHECK(server.getStats().depthPrePassTriangles == 2);
        frames.finish();

        server.setDepthPrePassEnabled(false);
        frames.draw(server);
        CHECK(server.getStats().depthPrePassSurfaces == 0);
        CHECK(server.getStats().skinnedSurfaces == 1);
        CHECK(pixelIs(driver, 38, 18, kRed, "skinned, bone 1 lifted, no pre-pass"));
        CHECK(pixelIs(driver, 2, 2, kClear, "skinned, no pre-pass, corner"));
        frames.finish();
        server.setDepthPrePassEnabled(true);

        SkinnedVertices skinned;
        CHECK(readSkinned(driver, server, instance, surface, skinned));
        if (!skinned.bytes.empty())
        {
            CHECK(nearVec3(skinned.position(0), 0.0f, 0.0f, 0.0f));
            CHECK(nearVec3(skinned.position(1), 1.0f, 0.0f, 0.0f));
            CHECK(nearVec3(skinned.position(2), 0.0f, 1.0f, 2.0f));
            CHECK(nearVec3(skinned.position(3), 1.0f, 1.0f, 1.0f));
            for (std::uint32_t v = 0; v < 4; ++v)
                CHECK(nearVec3(skinned.normal(v), 0.0f, 0.0f, 1.0f));
            CHECK(skinned.joints(3) == packJoints(0, 1, 0, 0));
            const float* weights = skinned.attribute(3, kVertexWeights);
            CHECK(near(weights[0], 0.5f) && near(weights[1], 0.5f));
            const float* color = skinned.attribute(2, kVertexColor);
            CHECK(near(color[0], 1.0f) && near(color[3], 1.0f));
        }

        frames.draw(server);
        CHECK(server.getStats().skinDispatches == 0);
        CHECK(server.getStats().skinnedSurfaces == 1);
        frames.finish();

        server.skeletonBoneSetTransform(skeleton, 0, translation(-1.5f, 0.0f, 0.0f));
        server.skeletonBoneSetTransform(skeleton, 1,
                translation(-1.5f, 0.0f, 0.0f) * Math::Mat4::RotationZ(1.5707963f));
        box = server.instanceGetAabb(instance);
        CHECK(nearVec3(box.min, -2.5f, 0.0f, 0.0f));
        CHECK(nearVec3(box.max, -0.5f, 1.0f, 0.0f));

        frames.draw(server);
        CHECK(server.getStats().skinDispatches == 1);
        CHECK(pixelIs(driver, 38, 18, kClear, "moved away from the bind pose"));
        CHECK(pixelIs(driver, 14, 20, kRed, "moved to the left"));
        frames.finish();
        CHECK(readSkinned(driver, server, instance, surface, skinned));
        if (!skinned.bytes.empty())
        {
            CHECK(nearVec3(skinned.position(0), -1.5f, 0.0f, 0.0f));
            CHECK(nearVec3(skinned.position(1), -0.5f, 0.0f, 0.0f));
            CHECK(nearVec3(skinned.position(2), -2.5f, 0.0f, 0.0f));
            CHECK(nearVec3(skinned.position(3), -1.5f, 1.0f, 0.0f));
            CHECK(nearVec3(skinned.normal(2), 0.0f, 0.0f, 1.0f));
        }

        server.skeletonAllocateData(skeleton, 3);
        CHECK(server.skeletonGetBoneCount(skeleton) == 3);
        CHECK(server.skeletonBoneGetTransform(skeleton, 0) == Math::Mat4::Identity());
        frames.draw(server);
        CHECK(server.getStats().skinDispatches == 1);
        CHECK(pixelIs(driver, 38, 18, kRed, "reallocated to the bind pose"));
        frames.finish();

        server.instanceAttachSkeleton(instance, SkeletonId());
        CHECK(!server.instanceGetSkeleton(instance).valid());
        CHECK(!server.instanceSkinnedVertexBuffer(instance, 0).valid());
        box = server.instanceGetAabb(instance);
        CHECK(nearVec3(box.max, 1.0f, 1.0f, 0.0f));
        frames.draw(server);
        CHECK(server.getStats().skinDispatches == 0);
        CHECK(server.getStats().skinnedSurfaces == 0);
        frames.finish();

        server.instanceAttachSkeleton(instance, skeleton);
        frames.draw(server);
        CHECK(server.getStats().skinDispatches == 1);
        frames.finish();
        server.skeletonFree(skeleton);
        CHECK(!server.instanceGetSkeleton(instance).valid());
        CHECK(!server.instanceGetSkeleton(plain).valid());
        CHECK(server.skeletonGetBoneCount(skeleton) == 0);
        frames.draw(server);
        CHECK(server.getStats().skinDispatches == 0);
        frames.finish();

        const SkeletonId other = server.skeletonCreate();
        server.skeletonAllocateData(other, 1);
        server.instanceAttachSkeleton(instance, other);
        server.instanceSetBase(instance, plainMesh);
        frames.draw(server);
        CHECK(server.getStats().skinDispatches == 0);
        CHECK(!server.instanceSkinnedVertexBuffer(instance, 0).valid());
        frames.finish();
        server.instanceSetBase(instance, skinnedMesh);
        frames.draw(server);
        CHECK(server.getStats().skinDispatches == 1);
        frames.finish();
        server.instanceFree(instance);
        frames.draw(server);
        CHECK(server.getStats().skinDispatches == 0);
        frames.finish();
    }

    Device::destroy(device);

    if (failures == 0) printf("ok\n");
    return failures == 0 ? 0 : 1;
}
