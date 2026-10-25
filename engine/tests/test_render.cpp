#include "Check.h"
#include "LitTest.h"

#include "prisma3d/Device.h"
#include "prisma3d/render/Batch.h"
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

struct Rgb
{
    int r;
    int g;
    int b;
};

const Rgb kClear = { 26, 51, 77 };
const Rgb kRed = { 255, 0, 0 };
const Rgb kGreen = { 0, 255, 0 };
const Rgb kBlue = { 0, 0, 255 };

bool pixelWithin(prisma::Driver* driver, int x, int y, Rgb expected, int tolerance,
        const char* label)
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
    const bool same = dr >= -tolerance && dr <= tolerance && dg >= -tolerance && dg <= tolerance &&
                      db >= -tolerance && db <= tolerance;
    if (!same)
        printf("%s: pixel (%d,%d) is %d,%d,%d, expected %d,%d,%d\n", label, x, y, pixel[0],
                pixel[1], pixel[2], expected.r, expected.g, expected.b);
    return same;
}

bool pixelIs(prisma::Driver* driver, int x, int y, Rgb expected, const char* label)
{
    return pixelWithin(driver, x, y, expected, kTolerance, label);
}

SurfaceData triangleData(const Math::Vec4& color)
{
    SurfaceData data;
    data.positions.push_back(Math::Vec3(-1.0f, -1.0f, 0.0f));
    data.positions.push_back(Math::Vec3(1.0f, -1.0f, 0.0f));
    data.positions.push_back(Math::Vec3(0.0f, 1.0f, 0.0f));
    for (int i = 0; i < 3; ++i) data.colors.push_back(color);
    return data;
}

SurfaceData quadData(float z, float half, const Math::Vec4& color)
{
    SurfaceData data;
    data.positions.push_back(Math::Vec3(-half, -half, z));
    data.positions.push_back(Math::Vec3(half, -half, z));
    data.positions.push_back(Math::Vec3(half, half, z));
    data.positions.push_back(Math::Vec3(-half, half, z));
    for (int i = 0; i < 4; ++i) data.colors.push_back(color);
    const std::uint32_t indices[6] = { 0, 1, 2, 0, 2, 3 };
    for (std::uint32_t index: indices) data.indices.push_back(index);
    return data;
}

struct Frames
{
    prisma::Driver* driver;
    RenderingServer* server;
    bool open;

    explicit Frames(prisma::Driver* d)
            : driver(d),
              server(nullptr),
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


void dfgReference(float nDotV, float roughness, float& outA, float& outB)
{
    const float pi = 3.14159265359f;
    const int samples = 1024;
    const float vx = sqrtf(1.0f - nDotV * nDotV);
    const float vz = nDotV;
    float a = 0.0f;
    float b = 0.0f;
    for (int i = 0; i < samples; ++i)
    {
        unsigned int bits = static_cast<unsigned int>(i);
        bits = (bits << 16u) | (bits >> 16u);
        bits = ((bits & 0x55555555u) << 1u) | ((bits & 0xAAAAAAAAu) >> 1u);
        bits = ((bits & 0x33333333u) << 2u) | ((bits & 0xCCCCCCCCu) >> 2u);
        bits = ((bits & 0x0F0F0F0Fu) << 4u) | ((bits & 0xF0F0F0F0u) >> 4u);
        bits = ((bits & 0x00FF00FFu) << 8u) | ((bits & 0xFF00FF00u) >> 8u);
        const float xi0 = static_cast<float>(i) / static_cast<float>(samples);
        const float xi1 = static_cast<float>(bits) * 2.3283064365386963e-10f;
        const float alpha = roughness * roughness;
        const float phi = 2.0f * pi * xi0;
        const float cosTheta = sqrtf((1.0f - xi1) / (1.0f + (alpha * alpha - 1.0f) * xi1));
        const float sinTheta = sqrtf(1.0f - cosTheta * cosTheta);
        const float hx = cosf(phi) * sinTheta;
        const float hy = sinf(phi) * sinTheta;
        const float hz = cosTheta;
        const float vDotH = vx * hx + vz * hz;
        float lx = 2.0f * vDotH * hx - vx;
        float ly = 2.0f * vDotH * hy;
        float lz = 2.0f * vDotH * hz - vz;
        const float length = sqrtf(lx * lx + ly * ly + lz * lz);
        lz /= length;
        (void) lx;
        (void) ly;
        const float nDotL = lz < 0.0f ? 0.0f : (lz > 1.0f ? 1.0f : lz);
        const float nDotH = hz < 0.0f ? 0.0f : (hz > 1.0f ? 1.0f : hz);
        const float vh = vDotH < 0.0f ? 0.0f : (vDotH > 1.0f ? 1.0f : vDotH);
        if (nDotL <= 0.0f) continue;
        const float k = roughness * roughness / 2.0f;
        const float gv = nDotV / (nDotV * (1.0f - k) + k);
        const float gl = nDotL / (nDotL * (1.0f - k) + k);
        const float visibility = (gv * gl * vh) / (nDotH * nDotV);
        a += powf(1.0f - vh, 5.0f) * visibility;
        b += visibility;
    }
    outA = a / static_cast<float>(samples);
    outB = b / static_cast<float>(samples);
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
    deviceDesc.title = "render test";
    deviceDesc.driver = vulkan ? prisma::DriverType::Vulkan : prisma::DriverType::OpenGL;
    deviceDesc.width = kWidth;
    deviceDesc.height = kHeight;
    deviceDesc.headless = true;

    Device* device = Device::create(deviceDesc);
    CHECK(device != nullptr);
    if (!device) return 1;

    prisma::Driver* driver = device->driver();

    {
        const Math::Vec4 clipNear = RenderingServer::perspective(60.0f, 4.0f / 3.0f, 1.0f, 100.0f) *
                                    Math::Vec4(0.0f, 0.0f, -1.0f, 1.0f);
        const Math::Vec4 clipFar = RenderingServer::perspective(60.0f, 4.0f / 3.0f, 1.0f, 100.0f) *
                                   Math::Vec4(0.0f, 0.0f, -100.0f, 1.0f);
        CHECK(fabsf(clipNear.z / clipNear.w) < 0.001f);
        CHECK(fabsf(clipFar.z / clipFar.w - 1.0f) < 0.001f);
        const float tangent = tanf(30.0f * 3.14159265f / 180.0f);
        const Math::Vec4 edge = RenderingServer::perspective(60.0f, 1.0f, 1.0f, 100.0f) *
                                Math::Vec4(0.0f, tangent, -1.0f, 1.0f);
        CHECK(fabsf(edge.y / edge.w - 1.0f) < 0.001f);

        const Math::Mat4 ortho = RenderingServer::orthogonal(4.0f, 1.0f, 1.0f, 100.0f);
        const Math::Vec4 side = ortho * Math::Vec4(2.0f, 0.0f, -1.0f, 1.0f);
        const Math::Vec4 top = ortho * Math::Vec4(0.0f, 2.0f, -100.0f, 1.0f);
        CHECK(fabsf(side.x - 1.0f) < 0.001f && fabsf(side.z) < 0.001f);
        CHECK(fabsf(top.y - 1.0f) < 0.001f && fabsf(top.z - 1.0f) < 0.001f);
    }

    {
        MeshStore meshes(driver);
        TextureStore textures(driver);
        MaterialStore materials(textures);
        RenderingServer server(driver, meshes, textures, materials);
        Frames frames(driver);
        CHECK(RenderingServer::singleton() == &server);
        CHECK(server.driver() == driver);
        CHECK(&server.meshes() == &meshes);
        CHECK(&server.textures() == &textures);
        CHECK(&server.materials() == &materials);

        const MeshId redTriangle = meshes.create("red triangle");
        CHECK(meshes.addSurface(redTriangle, triangleData(Math::Vec4(1.0f, 0.0f, 0.0f, 1.0f))));
        const MeshId blueQuad = meshes.create("blue quad");
        CHECK(meshes.addSurface(blueQuad,
                quadData(-1.0f, 2.0f, Math::Vec4(0.0f, 0.0f, 1.0f, 1.0f))));
        const MeshId whiteTriangle = meshes.create("white triangle");
        CHECK(meshes.addSurface(whiteTriangle, triangleData(Math::Vec4(1.0f, 1.0f, 1.0f, 1.0f))));

        const ScenarioId scenario = server.scenarioCreate();
        server.scenarioSetEnvironment(scenario, whiteAmbient());
        const InstanceId triangle = server.instanceCreate(redTriangle, scenario);
        CHECK(scenario.valid() && triangle.valid());

        const CameraId camera = server.cameraCreate();
        server.cameraSetPerspective(camera, 60.0f, 0.1f, 100.0f);
        server.cameraSetTransform(camera, translation(0.0f, 0.0f, 3.0f));

        const ViewportId viewport = server.viewportCreate();
        server.viewportSetSize(viewport, kWidth, kHeight);
        server.viewportAttachToScreen(viewport, true);
        server.viewportAttachCamera(viewport, camera);
        server.viewportSetScenario(viewport, scenario);
        server.viewportSetClearColor(viewport, Math::Vec4(0.1f, 0.2f, 0.3f, 1.0f));

        const Math::Box box = server.instanceGetAabb(triangle);
        CHECK(box.min.x == -1.0f && box.min.y == -1.0f && box.min.z == 0.0f);
        CHECK(box.max.x == 1.0f && box.max.y == 1.0f && box.max.z == 0.0f);

        frames.draw(server);
        const RenderStats& stats = server.getStats();
        CHECK(stats.viewportsDrawn == 1);
        CHECK(stats.instancesDrawn == 1);
        CHECK(stats.instancesCulled == 0);
        CHECK(stats.surfacesDrawn == 1);
        CHECK(stats.drawCalls == 1);
        CHECK(pixelIs(driver, 32, 24, kRed, "triangle centre"));
        CHECK(pixelIs(driver, 0, 0, kClear, "triangle corner"));
        CHECK(pixelIs(driver, kWidth - 1, kHeight - 1, kClear, "triangle opposite corner"));
        CHECK(pixelIs(driver, 20, 12, kClear, "outside of the triangle"));
        CHECK(pixelIs(driver, 32, 36, kRed, "triangle base"));
        CHECK(pixelIs(driver, 32, 8, kClear, "above the apex"));

        server.instanceSetVisible(triangle, false);
        frames.draw(server);
        CHECK(stats.viewportsDrawn == 1);
        CHECK(stats.instancesDrawn == 0);
        CHECK(stats.drawCalls == 0);
        CHECK(pixelIs(driver, 32, 24, kClear, "hidden instance"));

        server.instanceSetVisible(triangle, true);
        server.instanceSetTransform(triangle, translation(100.0f, 0.0f, 0.0f));
        frames.draw(server);
        CHECK(stats.instancesCulled == 1);
        CHECK(stats.instancesDrawn == 0);
        CHECK(stats.drawCalls == 0);
        CHECK(pixelIs(driver, 32, 24, kClear, "culled instance"));
        const Math::Box moved = server.instanceGetAabb(triangle);
        CHECK(moved.min.x == 99.0f && moved.max.x == 101.0f);

        server.instanceSetTransform(triangle, translation(0.0f, 0.0f, -200.0f));
        frames.draw(server);
        CHECK(stats.instancesCulled == 1);
        CHECK(pixelIs(driver, 32, 24, kClear, "beyond the far plane"));

        server.instanceSetTransform(triangle, translation(0.0f, 0.0f, 10.0f));
        frames.draw(server);
        CHECK(stats.instancesCulled == 1);
        CHECK(pixelIs(driver, 32, 24, kClear, "behind the camera"));

        server.instanceSetTransform(triangle, Math::Mat4::Identity());
        server.instanceSetColor(triangle, Math::Vec4(0.0f, 1.0f, 0.0f, 1.0f));
        frames.draw(server);
        CHECK(stats.instancesDrawn == 1);
        CHECK(pixelIs(driver, 32, 24, Rgb{ 0, 0, 0 }, "red vertex colour times green"));

        server.instanceSetBase(triangle, whiteTriangle);
        frames.draw(server);
        CHECK(pixelIs(driver, 32, 24, kGreen, "white vertex colour times green"));

        server.instanceSetColor(triangle, Math::Vec4(1.0f, 1.0f, 1.0f, 1.0f));
        server.instanceSetBase(triangle, redTriangle);
        frames.draw(server);
        CHECK(pixelIs(driver, 32, 24, kRed, "restored"));

        server.cameraSetCullMask(camera, 1);
        server.instanceSetLayerMask(triangle, 2);
        frames.draw(server);
        CHECK(stats.instancesDrawn == 0);
        CHECK(stats.instancesCulled == 0);
        CHECK(pixelIs(driver, 32, 24, kClear, "layer not in the camera mask"));
        server.cameraSetCullMask(camera, 2);
        frames.draw(server);
        CHECK(stats.instancesDrawn == 1);
        CHECK(pixelIs(driver, 32, 24, kRed, "layer in the camera mask"));
        server.cameraSetCullMask(camera, 0xffffffff);
        server.instanceSetLayerMask(triangle, 1);

        const InstanceId quad = server.instanceCreate(blueQuad, scenario);
        frames.draw(server);
        CHECK(stats.instancesDrawn == 2);
        CHECK(stats.surfacesDrawn == 2);
        CHECK(stats.drawCalls == 2);
        CHECK(pixelIs(driver, 32, 24, kRed, "triangle in front of the quad"));
        CHECK(pixelIs(driver, 14, 24, kBlue, "quad beside the triangle"));
        CHECK(pixelIs(driver, 50, 24, kBlue, "quad on the other side"));
        CHECK(pixelIs(driver, 32, 6, kBlue, "quad above the triangle"));
        CHECK(pixelIs(driver, 5, 24, kClear, "outside of the quad"));
        CHECK(pixelIs(driver, 0, 0, kClear, "quad corner of the screen"));

        server.instanceSetVisible(triangle, false);
        frames.draw(server);
        CHECK(pixelIs(driver, 32, 24, kBlue, "quad alone"));
        server.instanceSetVisible(triangle, true);

        CHECK(server.instanceGetAabb(quad).min.z == -1.0f);
        server.instanceFree(quad);
        frames.draw(server);
        CHECK(stats.instancesDrawn == 1);
        CHECK(pixelIs(driver, 14, 24, kClear, "freed quad"));
        CHECK(server.instanceGetAabb(quad).min.x > server.instanceGetAabb(quad).max.x);

        server.viewportSetSize(viewport, 0, 0);
        frames.draw(server);
        CHECK(stats.viewportsDrawn == 1);
        CHECK(pixelIs(driver, 32, 24, kRed, "viewport sized from the window"));
        CHECK(stats.targetsCreated == 1);
        frames.draw(server);
        CHECK(stats.targetsCreated == 0);
        server.viewportSetSize(viewport, kWidth, kHeight);

        server.viewportSetSize(viewport, kWidth / 2, kHeight / 2);
        frames.draw(server);
        CHECK(stats.targetsCreated == 1);
        CHECK(pixelIs(driver, 32, 24, kRed, "half resolution stretched to the window, centre"));
        CHECK(pixelIs(driver, 32, 36, kRed, "half resolution stretched to the window, base"));
        CHECK(pixelIs(driver, 20, 12, kClear, "half resolution stretched to the window, outside"));
        CHECK(pixelIs(driver, 0, 0, kClear, "half resolution stretched to the window, corner"));
        frames.draw(server);
        CHECK(stats.targetsCreated == 0);
        server.viewportSetSize(viewport, kWidth / 2, kHeight / 2);
        frames.draw(server);
        CHECK(stats.targetsCreated == 0);
        server.viewportSetSize(viewport, kWidth, kHeight);
        frames.draw(server);
        CHECK(stats.targetsCreated == 1);

        server.cameraSetOrthogonal(camera, 4.0f, 0.1f, 100.0f);
        frames.draw(server);
        CHECK(pixelIs(driver, 32, 24, kRed, "orthogonal centre"));
        CHECK(pixelIs(driver, 32 + 10, 24 + 8, kClear, "orthogonal outside"));
        CHECK(pixelIs(driver, 32, 24 + 10, kRed, "orthogonal base"));
        server.cameraSetPerspective(camera, 60.0f, 0.1f, 100.0f);

        const InstanceId behind = server.instanceCreate(blueQuad, scenario);
        server.instanceSetVisible(triangle, false);
        Math::Mat4 turned =
                Math::Quaternion::FromAxisAngle(Math::Vec3(0.0f, 1.0f, 0.0f), 3.14159265358979f)
                        .ToMat4();
        turned.col3 = Math::Vec4(0.0f, 0.0f, -3.0f, 1.0f);
        server.cameraSetTransform(camera, turned);
        frames.draw(server);
        CHECK(stats.instancesDrawn == 1);
        CHECK(pixelIs(driver, 32, 24, kClear, "quad seen from behind is culled"));
        server.instanceSetVisible(triangle, true);
        server.instanceFree(behind);
        frames.draw(server);
        CHECK(stats.instancesDrawn == 1);
        CHECK(stats.drawCalls == 1);
        CHECK(pixelIs(driver, 32, 24, kClear, "back face is culled"));
        CHECK(pixelIs(driver, 0, 0, kClear, "back face corner"));

        server.cameraSetTransform(camera, translation(0.0f, 0.0f, 3.0f));
        frames.draw(server);
        CHECK(pixelIs(driver, 32, 24, kRed, "camera back in front"));

        const ScenarioId other = server.scenarioCreate();
        server.scenarioSetEnvironment(other, whiteAmbient());
        const InstanceId stray = server.instanceCreate(blueQuad, other);
        frames.draw(server);
        CHECK(stats.instancesDrawn == 1);
        CHECK(pixelIs(driver, 14, 24, kClear, "instance of another scenario"));
        server.viewportSetScenario(viewport, other);
        frames.draw(server);
        CHECK(stats.instancesDrawn == 1);
        CHECK(pixelIs(driver, 32, 24, kBlue, "viewport on the other scenario"));
        server.instanceSetScenario(stray, scenario);
        frames.draw(server);
        CHECK(stats.viewportsDrawn == 1);
        CHECK(stats.instancesDrawn == 0);
        CHECK(pixelIs(driver, 32, 24, kClear, "instance moved out of the scenario"));
        server.viewportSetScenario(viewport, scenario);
        frames.draw(server);
        CHECK(stats.instancesDrawn == 2);
        CHECK(pixelIs(driver, 32, 24, kRed, "both back in the first scenario"));
        server.instanceSetScenario(stray, other);
        server.scenarioFree(other);
        server.viewportSetScenario(viewport, other);
        frames.draw(server);
        CHECK(stats.viewportsDrawn == 0);
        server.viewportSetScenario(viewport, scenario);

        const ViewportId unattached = server.viewportCreate();
        server.viewportSetSize(unattached, kWidth, kHeight);
        server.viewportAttachToScreen(unattached, false);
        server.viewportAttachCamera(unattached, camera);
        server.viewportSetScenario(unattached, scenario);
        frames.draw(server);
        CHECK(stats.viewportsDrawn == 1);
        server.viewportFree(unattached);

        const ViewportId bare = server.viewportCreate();
        server.viewportAttachToScreen(bare, true);
        server.viewportSetScenario(bare, scenario);
        server.viewportSetActive(viewport, false);
        frames.draw(server);
        CHECK(stats.viewportsDrawn == 0);
        server.viewportSetActive(viewport, true);
        server.viewportAttachCamera(bare, camera);
        server.viewportSetActive(viewport, false);
        server.viewportSetClearColor(bare, Math::Vec4(1.0f, 0.0f, 1.0f, 1.0f));
        frames.draw(server);
        CHECK(stats.viewportsDrawn == 1);
        CHECK(pixelIs(driver, 0, 0, Rgb{ 255, 0, 255 }, "second viewport clear"));
        server.viewportFree(bare);
        server.viewportSetActive(viewport, true);

        frames.draw(server);
        CHECK(stats.viewportsDrawn == 1);
        CHECK(pixelIs(driver, 32, 24, kRed, "before freeing"));

        server.cameraFree(camera);
        frames.draw(server);
        CHECK(stats.viewportsDrawn == 0);
        const CameraId again = server.cameraCreate();
        server.cameraSetTransform(again, translation(0.0f, 0.0f, 3.0f));
        server.viewportAttachCamera(viewport, again);
        frames.draw(server);
        CHECK(stats.viewportsDrawn == 1);
        CHECK(pixelIs(driver, 32, 24, kRed, "new camera"));

        server.instanceFree(triangle);
        server.instanceFree(stray);
        server.scenarioFree(scenario);
        server.cameraFree(again);
        server.viewportFree(viewport);
        frames.draw(server);
        CHECK(stats.viewportsDrawn == 0);
        frames.finish();
        server.draw(true, 0.016);
        CHECK(stats.viewportsDrawn == 0);
        CHECK(server.instanceGetAabb(triangle).min.x > server.instanceGetAabb(triangle).max.x);
    }


    {
        MeshStore meshes(driver);
        TextureStore textures(driver);
        MaterialStore materials(textures);
        RenderingServer server(driver, meshes, textures, materials);
        Frames frames(driver);
        const Rgb kWhite = { 255, 255, 255 };

        const MeshId mesh = meshes.create("two surfaces");
        const Math::Vec4 white(1.0f, 1.0f, 1.0f, 1.0f);
        CHECK(meshes.addSurface(mesh, triangleData(white)));
        CHECK(meshes.addSurface(mesh, quadData(-1.0f, 2.0f, white)));
        CHECK(!meshes.get(mesh)->surfaces[0].material.valid());
        CHECK(!meshes.get(mesh)->surfaces[1].material.valid());

        const MaterialId green = materials.create("green");
        const MaterialId blue = materials.create("blue");
        const MaterialId red = materials.create("red");
        CHECK(materials.setColor(green, MaterialParam::Albedo, Math::Vec4(0.0f, 1.0f, 0.0f, 1.0f)));
        CHECK(materials.setColor(blue, MaterialParam::Albedo, Math::Vec4(0.0f, 0.0f, 1.0f, 1.0f)));
        CHECK(materials.setColor(red, MaterialParam::Albedo, Math::Vec4(1.0f, 0.0f, 0.0f, 1.0f)));
        CHECK(&server.materials() == &materials);

        CHECK(!meshes.setSurfaceMaterial(mesh, 2, green));
        CHECK(!meshes.setSurfaceMaterial(MeshId(), 0, green));
        CHECK(meshes.setSurfaceMaterial(mesh, 0, green));
        CHECK(meshes.get(mesh)->surfaces[0].material == green);

        SurfaceLayout layout;
        layout.vertexCount = 3;
        const int pending = meshes.beginSurface(mesh, layout);
        CHECK(pending == 2);
        CHECK(!meshes.get(mesh)->surfaces[2].material.valid());
        CHECK(meshes.setSurfaceMaterial(mesh, 2, blue));
        CHECK(meshes.get(mesh)->surfaces[2].material == blue);
        CHECK(meshes.finishSurface(mesh, 2));
        CHECK(meshes.get(mesh)->surfaces[2].material == blue);
        meshes.clearSurfaces(mesh);
        CHECK(meshes.addSurface(mesh, triangleData(white)));
        CHECK(meshes.addSurface(mesh, quadData(-1.0f, 2.0f, white)));
        CHECK(!meshes.get(mesh)->surfaces[0].material.valid());
        CHECK(meshes.setSurfaceMaterial(mesh, 0, green));

        const ScenarioId scenario = server.scenarioCreate();
        server.scenarioSetEnvironment(scenario, whiteAmbient());
        const InstanceId instance = server.instanceCreate(mesh, scenario);
        const CameraId camera = server.cameraCreate();
        server.cameraSetPerspective(camera, 60.0f, 0.1f, 100.0f);
        server.cameraSetTransform(camera, translation(0.0f, 0.0f, 3.0f));
        const ViewportId viewport = server.viewportCreate();
        server.viewportSetSize(viewport, kWidth, kHeight);
        server.viewportAttachToScreen(viewport, true);
        server.viewportAttachCamera(viewport, camera);
        server.viewportSetScenario(viewport, scenario);
        server.viewportSetClearColor(viewport, Math::Vec4(0.1f, 0.2f, 0.3f, 1.0f));

        const MaterialId fallback = materials.defaultMaterial();
        CHECK(fallback.valid());
        CHECK(server.instanceResolveMaterial(instance, 0) == green);
        CHECK(server.instanceResolveMaterial(instance, 1) == fallback);
        CHECK(server.instanceResolveMaterial(instance, 5) == fallback);
        CHECK(server.instanceResolveMaterial(InstanceId(), 0) == fallback);

        frames.draw(server);
        CHECK(server.getStats().drawCalls == 2);
        CHECK(pixelIs(driver, 32, 24, kGreen, "mesh material green"));
        CHECK(pixelIs(driver, 14, 24, kWhite, "default material on a surface without one"));
        CHECK(pixelIs(driver, 5, 24, kClear, "outside of the quad"));

        server.instanceSetColor(instance, Math::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
        frames.draw(server);
        CHECK(pixelIs(driver, 32, 24, Rgb{ 0, 0, 0 }, "green albedo times red colour"));
        CHECK(pixelIs(driver, 14, 24, kRed, "default albedo times red colour"));
        server.instanceSetColor(instance, white);

        CHECK(materials.setColor(green, MaterialParam::Albedo, Math::Vec4(0.0f, 0.5f, 0.0f, 1.0f)));
        frames.draw(server);
        CHECK(pixelIs(driver, 32, 24, Rgb{ 0, 128, 0 }, "edited albedo"));
        CHECK(materials.setColor(green, MaterialParam::Albedo, Math::Vec4(0.0f, 1.0f, 0.0f, 1.0f)));

        CHECK(materials.setFloat(green, MaterialParam::Metallic, 0.5f));
        frames.draw(server);
        CHECK(pixelIs(driver, 32, 24, Rgb{ 0, 188, 0 }, "half metallic halves the ambient light"));
        CHECK(materials.setFloat(green, MaterialParam::Metallic, 1.0f));
        frames.draw(server);
        CHECK(pixelIs(driver, 32, 24, Rgb{ 0, 0, 0 }, "full metallic has no ambient light"));
        CHECK(materials.setFloat(green, MaterialParam::Metallic, 0.0f));

        const MaterialId glow = materials.create("glow");
        CHECK(materials.setColor(glow, MaterialParam::Albedo, Math::Vec4(0.0f, 0.0f, 0.0f, 1.0f)));
        CHECK(materials.setColor(glow, MaterialParam::Emission,
                Math::Vec4(0.0f, 0.5f, 0.0f, 1.0f)));
        server.instanceSetMaterialOverride(instance, glow);
        frames.draw(server);
        CHECK(pixelIs(driver, 32, 24, Rgb{ 0, 128, 0 }, "emission on a black material"));
        CHECK(materials.setFloat(glow, MaterialParam::EmissionEnergy, 2.0f));
        frames.draw(server);
        CHECK(pixelIs(driver, 32, 24, Rgb{ 0, 175, 0 }, "emission energy"));
        server.instanceSetMaterialOverride(instance, MaterialId());

        {
            const std::uint8_t albedoTexel[4] = { 0, 128, 0, 255 };
            const std::uint8_t normalTexel[4] = { 128, 128, 255, 255 };
            prisma::TextureDesc albedoDesc;
            albedoDesc.format = prisma::TextureFormat::RGBA8Srgb;
            albedoDesc.width = 1;
            albedoDesc.height = 1;
            albedoDesc.data = albedoTexel;
            const TextureId albedoTexture = textures.create(albedoDesc, true);
            prisma::TextureDesc normalDesc;
            normalDesc.format = prisma::TextureFormat::RGBA8;
            normalDesc.width = 1;
            normalDesc.height = 1;
            normalDesc.data = normalTexel;
            const TextureId normalTexture = textures.create(normalDesc, false);
            CHECK(albedoTexture.valid() && normalTexture.valid());

            SurfaceData textured = quadData(0.0f, 1.0f, white);
            const Math::Vec2 corners[4] = { Math::Vec2(0.0f, 0.0f), Math::Vec2(1.0f, 0.0f),
                Math::Vec2(1.0f, 1.0f), Math::Vec2(0.0f, 1.0f) };
            for (int i = 0; i < 4; ++i)
            {
                textured.uvs.push_back(corners[i]);
                textured.normals.push_back(Math::Vec3(0.0f, 0.0f, 1.0f));
                textured.tangents.push_back(Math::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
            }
            const MeshId texturedMesh = meshes.create("textured");
            CHECK(meshes.addSurface(texturedMesh, textured));
            const InstanceId texturedInstance = server.instanceCreate(texturedMesh, scenario);
            server.instanceSetVisible(instance, false);

            const MaterialId withAlbedo = materials.create("with albedo");
            CHECK(materials.setTexture(withAlbedo, MaterialTexture::Albedo, albedoTexture));
            server.instanceSetMaterialOverride(texturedInstance, withAlbedo);
            frames.draw(server);
            CHECK(pixelIs(driver, 32, 24, Rgb{ 0, 128, 0 }, "albedo texture"));

            const MaterialId withNormal = materials.create("with normal");
            CHECK(materials.setTexture(withNormal, MaterialTexture::Albedo, albedoTexture));
            CHECK(materials.setTexture(withNormal, MaterialTexture::Normal, normalTexture));
            server.instanceSetMaterialOverride(texturedInstance, withNormal);
            frames.draw(server);
            CHECK(pixelIs(driver, 32, 24, Rgb{ 0, 128, 0 }, "albedo texture with a normal map"));

            server.instanceFree(texturedInstance);
            server.instanceSetVisible(instance, true);
        }

        server.instanceSetSurfaceMaterial(instance, 1, red);
        CHECK(server.instanceResolveMaterial(instance, 0) == green);
        CHECK(server.instanceResolveMaterial(instance, 1) == red);
        frames.draw(server);
        CHECK(pixelIs(driver, 32, 24, kGreen, "other surface keeps the mesh material"));
        CHECK(pixelIs(driver, 14, 24, kRed, "per-surface override on the quad"));

        server.instanceSetSurfaceMaterial(instance, 0, blue);
        CHECK(server.instanceResolveMaterial(instance, 0) == blue);
        frames.draw(server);
        CHECK(pixelIs(driver, 32, 24, kBlue, "per-surface override wins over the mesh material"));
        CHECK(pixelIs(driver, 14, 24, kRed, "quad override unchanged"));

        server.instanceSetMaterialOverride(instance, green);
        CHECK(server.instanceResolveMaterial(instance, 0) == green);
        CHECK(server.instanceResolveMaterial(instance, 1) == green);
        CHECK(server.instanceResolveMaterial(instance, 9) == green);
        frames.draw(server);
        CHECK(pixelIs(driver, 32, 24, kGreen, "instance override wins over the surface override"));
        CHECK(pixelIs(driver, 14, 24, kGreen, "instance override covers every surface"));

        server.instanceSetMaterialOverride(instance, MaterialId());
        CHECK(server.instanceResolveMaterial(instance, 0) == blue);
        CHECK(server.instanceResolveMaterial(instance, 1) == red);
        frames.draw(server);
        CHECK(pixelIs(driver, 32, 24, kBlue, "override cleared"));

        server.instanceSetSurfaceMaterial(instance, 0, MaterialId());
        server.instanceSetSurfaceMaterial(instance, 1, MaterialId());
        server.instanceSetSurfaceMaterial(instance, 7, MaterialId());
        CHECK(server.instanceResolveMaterial(instance, 0) == green);
        CHECK(server.instanceResolveMaterial(instance, 1) == fallback);
        frames.draw(server);
        CHECK(pixelIs(driver, 32, 24, kGreen, "surface override cleared"));
        CHECK(pixelIs(driver, 14, 24, kWhite, "quad back to the default material"));

        server.instanceSetSurfaceMaterial(instance, 3, red);
        CHECK(server.instanceResolveMaterial(instance, 3) == red);
        CHECK(server.instanceResolveMaterial(instance, 0) == green);
        server.instanceSetSurfaceMaterial(instance, 3, MaterialId());

        materials.destroy(green);
        CHECK(server.instanceResolveMaterial(instance, 0) == fallback);
        frames.draw(server);
        CHECK(pixelIs(driver, 32, 24, kWhite, "destroyed material falls back to the default"));

        const MaterialId again = materials.create("green again");
        CHECK(materials.setColor(again, MaterialParam::Albedo, Math::Vec4(0.0f, 1.0f, 0.0f, 1.0f)));
        CHECK(meshes.setSurfaceMaterial(mesh, 0, again));
        frames.draw(server);
        CHECK(pixelIs(driver, 32, 24, kGreen, "material changed on the mesh surface"));

        {
            server.instanceSetVisible(instance, false);
            const ScenarioId lit = server.scenarioCreate();
            EnvironmentSettings dark;
            dark.ambientSource = AmbientSource::Color;
            server.scenarioSetEnvironment(lit, dark);
            server.viewportSetScenario(viewport, lit);

            SurfaceData wall = quadData(0.0f, 2.0f, white);
            for (int i = 0; i < 4; ++i) wall.normals.push_back(Math::Vec3(0.0f, 0.0f, 1.0f));
            const MeshId wallMesh = meshes.create("wall");
            CHECK(meshes.addSurface(wallMesh, wall));
            const MaterialId matte = materials.create("matte");
            CHECK(materials.setFloat(matte, MaterialParam::Specular, 0.0f));
            CHECK(materials.setFloat(matte, MaterialParam::Roughness, 1.0f));
            const InstanceId wallInstance = server.instanceCreate(wallMesh, lit);
            server.instanceSetMaterialOverride(wallInstance, matte);

            const float pi = 3.14159265358979f;
            auto grey = [](float linear)
            {
                const float encoded = linear <= 0.0031308f
                                              ? linear * 12.92f
                                              : 1.055f * powf(linear, 1.0f / 2.4f) - 0.055f;
                const int value = static_cast<int>(encoded * 255.0f + 0.5f);
                return Rgb{ value, value, value };
            };
            auto omniAt = [](float distance, float radius, float noL)
            {
                float nd = distance / radius;
                nd *= nd;
                nd *= nd;
                nd = 1.0f - nd;
                nd *= nd;
                return nd / distance * noL;
            };
            const Rgb kBlack = { 0, 0, 0 };

            frames.draw(server);
            CHECK(server.getStats().lightsDrawn == 0);
            CHECK(pixelIs(driver, 32, 24, kBlack, "no light and no ambient"));

            const LightId sun = server.lightCreate(LightType::Directional);
            server.lightSetScenario(sun, lit);
            server.lightSetParam(sun, LightParam::Energy, 0.5f);
            frames.draw(server);
            CHECK(server.getStats().lightsDrawn == 1);
            CHECK(pixelIs(driver, 32, 24, grey(0.5f), "directional light facing the wall"));

            server.lightSetParam(sun, LightParam::Energy, 1.0f);
            server.lightSetTransform(sun, Math::Mat4::RotationY(pi / 3.0f));
            frames.draw(server);
            CHECK(pixelIs(driver, 32, 24, grey(0.5f), "directional light at sixty degrees"));

            const Math::Mat4 rolled =
                    translation(0.0f, 0.0f, 3.0f) * Math::Mat4::RotationZ(pi / 2.0f);
            server.cameraSetTransform(camera, rolled);
            frames.draw(server);
            CHECK(pixelIs(driver, 32, 24, grey(0.5f), "directional light seen by a rolled camera"));
            server.cameraSetTransform(camera, translation(0.0f, 0.0f, 3.0f));

            server.lightSetTransform(sun, Math::Mat4::RotationY(pi / 2.0f));
            frames.draw(server);
            CHECK(pixelIs(driver, 32, 24, kBlack, "directional light along the wall"));

            server.lightSetTransform(sun, Math::Mat4::Identity());
            server.lightSetColor(sun, Math::Vec3(1.0f, 0.0f, 0.0f));
            frames.draw(server);
            CHECK(pixelIs(driver, 32, 24, kRed, "red directional light"));
            server.lightSetColor(sun, Math::Vec3(1.0f, 1.0f, 1.0f));

            server.lightSetCullMask(sun, 2);
            frames.draw(server);
            CHECK(pixelIs(driver, 32, 24, kBlack, "light mask outside the instance layer"));
            server.instanceSetLayerMask(wallInstance, 2);
            frames.draw(server);
            CHECK(pixelIs(driver, 32, 24, grey(1.0f), "light mask matching the instance layer"));
            server.instanceSetLayerMask(wallInstance, 1);
            server.lightSetCullMask(sun, 0xffffffff);

            server.lightSetVisible(sun, false);
            frames.draw(server);
            CHECK(pixelIs(driver, 32, 24, kBlack, "hidden light"));
            server.lightSetVisible(sun, true);

            server.lightSetNegative(sun, true);
            frames.draw(server);
            CHECK(pixelIs(driver, 32, 24, kBlack, "negative light"));
            server.lightFree(sun);
            frames.draw(server);
            CHECK(server.getStats().lightsDrawn == 0);
            CHECK(pixelIs(driver, 32, 24, kBlack, "freed light"));

            const LightId bulb = server.lightCreate(LightType::Omni);
            server.lightSetScenario(bulb, lit);
            server.lightSetParam(bulb, LightParam::Range, 10.0f);
            server.lightSetTransform(bulb, translation(0.0f, 0.0f, 2.0f));
            frames.draw(server);
            CHECK(server.getStats().lightsDrawn == 1);
            CHECK(pixelIs(driver, 32, 24, grey(omniAt(2.0f, 10.0f, 1.0f)), "omni light in front"));

            server.cameraSetTransform(camera, rolled);
            frames.draw(server);
            CHECK(pixelIs(driver, 32, 24, grey(omniAt(2.0f, 10.0f, 1.0f)),
                    "omni light seen by a rolled camera"));
            server.cameraSetTransform(camera, translation(0.0f, 0.0f, 3.0f));

            server.lightSetTransform(bulb, translation(1.0f, 0.0f, 2.0f));
            frames.draw(server);
            const float slant = sqrtf(5.0f);
            CHECK(pixelIs(driver, 32, 24, grey(omniAt(slant, 10.0f, 2.0f / slant)),
                    "omni light to the side"));

            server.lightSetTransform(bulb, translation(0.0f, 0.0f, 20.0f));
            frames.draw(server);
            CHECK(pixelIs(driver, 32, 24, kBlack, "omni light beyond its range"));

            server.lightSetTransform(bulb, translation(0.0f, 0.0f, 2.0f));
            server.lightSetParam(bulb, LightParam::Energy, 0.0f);
            frames.draw(server);
            CHECK(pixelIs(driver, 32, 24, kBlack, "omni light without energy"));
            server.lightFree(bulb);

            const LightId lamp = server.lightCreate(LightType::Spot);
            server.lightSetScenario(lamp, lit);
            server.lightSetParam(lamp, LightParam::Range, 10.0f);
            server.lightSetParam(lamp, LightParam::SpotAngle, 10.0f);
            server.lightSetTransform(lamp, translation(0.0f, 0.0f, 2.0f));
            frames.draw(server);
            CHECK(pixelIs(driver, 32, 24, grey(omniAt(2.0f, 10.0f, 1.0f)), "spot light centre"));
            CHECK(pixelIs(driver, 50, 24, kBlack, "outside of the spot cone"));
            server.lightSetTransform(lamp,
                    translation(0.0f, 0.0f, 2.0f) * Math::Mat4::RotationY(pi));
            frames.draw(server);
            CHECK(pixelIs(driver, 32, 24, kBlack, "spot light pointing away"));
            server.lightFree(lamp);

            EnvironmentSettings ambient = dark;
            ambient.ambientColor = Math::Vec3(1.0f, 1.0f, 1.0f);
            ambient.ambientEnergy = 0.5f;
            server.scenarioSetEnvironment(lit, ambient);
            frames.draw(server);
            CHECK(pixelIs(driver, 32, 24, grey(0.5f), "ambient colour and energy"));
            ambient.ambientSource = AmbientSource::Disabled;
            server.scenarioSetEnvironment(lit, ambient);
            frames.draw(server);
            CHECK(pixelIs(driver, 32, 24, kBlack, "ambient disabled"));

            const MaterialId unlit = materials.create("unlit");
            CHECK(materials.setColor(unlit, MaterialParam::Albedo,
                    Math::Vec4(0.0f, 0.5f, 0.0f, 1.0f)));
            CHECK(materials.setFloat(unlit, MaterialParam::Specular, 0.0f));
            CHECK(materials.setFloat(unlit, MaterialParam::Roughness, 1.0f));
            CHECK(materials.setColor(unlit, MaterialParam::Emission,
                    Math::Vec4(1.0f, 0.0f, 0.0f, 1.0f)));
            CHECK(materials.setShading(unlit, MaterialShading::Unshaded));
            server.instanceSetMaterialOverride(wallInstance, unlit);
            frames.draw(server);
            CHECK(pixelIs(driver, 32, 24, Rgb{ 0, 128, 0 }, "unshaded albedo without any light"));
            CHECK(materials.setShading(unlit, MaterialShading::Lit));
            frames.draw(server);
            CHECK(pixelIs(driver, 32, 24, kRed, "back to lit: only the emission is left"));
            CHECK(materials.setShading(unlit, MaterialShading::Unshaded));

            {
                SurfaceData frontData = quadData(1.0f, 0.5f, white);
                for (int i = 0; i < 4; ++i)
                    frontData.normals.push_back(Math::Vec3(0.0f, 0.0f, 1.0f));
                const MeshId frontMesh = meshes.create("front");
                CHECK(meshes.addSurface(frontMesh, frontData));
                const InstanceId front = server.instanceCreate(frontMesh, lit);
                const MaterialId blended = materials.create("blended");
                CHECK(materials.setShading(blended, MaterialShading::Unshaded));
                CHECK(materials.setColor(blended, MaterialParam::Albedo,
                        Math::Vec4(0.5f, 0.0f, 0.0f, 1.0f)));
                server.instanceSetMaterialOverride(front, blended);
                const float half = powf((0.5f + 0.055f) / 1.055f, 2.4f);

                frames.draw(server);
                CHECK(pixelIs(driver, 32, 24, Rgb{ 128, 0, 0 },
                        "opaque quad in front of the wall"));
                CHECK(pixelIs(driver, 8, 24, Rgb{ 0, 128, 0 }, "wall beside the quad"));

                auto depthAt = [](float distance)
                {
                    const float zNear = 0.1f;
                    const float zFar = 100.0f;
                    const float ndc = (zFar + zNear) / (zFar - zNear) -
                                      (2.0f * zFar * zNear) / ((zFar - zNear) * distance);
                    return ndc * 0.5f + 0.5f;
                };
                float depth = 0.0f;
                CHECK(server.viewportReadDepth(viewport, 32, 24, depth));
                if (fabsf(depth - depthAt(2.0f)) >= 1e-4f)
                    printf("depth of the quad is %f, expected %f\n", depth, depthAt(2.0f));
                CHECK(fabsf(depth - depthAt(2.0f)) < 1e-4f);
                CHECK(server.viewportReadDepth(viewport, 8, 24, depth));
                if (fabsf(depth - depthAt(3.0f)) >= 1e-4f)
                    printf("depth of the wall is %f, expected %f\n", depth, depthAt(3.0f));
                CHECK(fabsf(depth - depthAt(3.0f)) < 1e-4f);
                CHECK(!server.viewportReadDepth(viewport, -1, 24, depth));
                CHECK(!server.viewportReadDepth(viewport, 64, 24, depth));
                CHECK(!server.viewportReadDepth(viewport, 32, 48, depth));
                CHECK(!server.viewportReadDepth(ViewportId(), 32, 24, depth));
                frames.draw(server);
                CHECK(pixelIs(driver, 32, 24, Rgb{ 128, 0, 0 }, "frame after a depth read"));

                CHECK(materials.setBlendMode(blended, MaterialBlend::Add));
                frames.draw(server);
                CHECK(pixelIs(driver, 32, 24, Rgb{ 128, 128, 0 }, "additive blend"));
                CHECK(materials.setColor(blended, MaterialParam::Albedo,
                        Math::Vec4(0.5f, 0.0f, 0.0f, 0.5f)));
                frames.draw(server);
                CHECK(pixelIs(driver, 32, 24, Rgb{ grey(half * 0.5f).r, 128, 0 },
                        "additive blend scaled by the alpha"));
                CHECK(materials.setColor(blended, MaterialParam::Albedo,
                        Math::Vec4(0.5f, 0.0f, 0.0f, 1.0f)));

                CHECK(materials.setColor(unlit, MaterialParam::Albedo,
                        Math::Vec4(0.5f, 0.5f, 0.0f, 1.0f)));
                CHECK(materials.setBlendMode(blended, MaterialBlend::Sub));
                frames.draw(server);
                CHECK(pixelIs(driver, 32, 24, Rgb{ 0, 128, 0 }, "subtractive blend"));

                CHECK(materials.setColor(unlit, MaterialParam::Albedo,
                        Math::Vec4(1.0f, 0.5f, 0.0f, 1.0f)));
                CHECK(materials.setColor(blended, MaterialParam::Albedo,
                        Math::Vec4(0.5f, 1.0f, 1.0f, 1.0f)));
                CHECK(materials.setBlendMode(blended, MaterialBlend::Mul));
                frames.draw(server);
                CHECK(pixelIs(driver, 32, 24, Rgb{ 128, 128, 0 }, "multiplicative blend"));

                CHECK(materials.setColor(unlit, MaterialParam::Albedo,
                        Math::Vec4(0.5f, 0.5f, 0.0f, 1.0f)));
                CHECK(materials.setColor(blended, MaterialParam::Albedo,
                        Math::Vec4(0.5f, 0.0f, 0.0f, 1.0f)));
                CHECK(materials.setBlendMode(blended, MaterialBlend::Screen));
                frames.draw(server);
                CHECK(pixelIs(driver, 32, 24, Rgb{ grey(half + half * (1.0f - half)).r, 128, 0 },
                        "screen blend"));

                CHECK(materials.setColor(unlit, MaterialParam::Albedo,
                        Math::Vec4(0.0f, 0.5f, 0.0f, 1.0f)));
                CHECK(materials.setColor(blended, MaterialParam::Albedo,
                        Math::Vec4(0.5f, 0.0f, 0.0f, 1.0f)));
                CHECK(materials.setBlendMode(blended, MaterialBlend::Mix));
                CHECK(materials.setTransparency(blended, MaterialTransparency::AlphaBlend));
                server.instanceSetTransform(front, translation(0.0f, 0.0f, -2.0f));
                frames.draw(server);
                CHECK(pixelIs(driver, 32, 24, Rgb{ 0, 128, 0 }, "quad hidden behind the wall"));
                CHECK(materials.setDepthTest(blended, false));
                frames.draw(server);
                CHECK(pixelIs(driver, 32, 24, Rgb{ 128, 0, 0 },
                        "quad behind the wall without depth test"));

                server.instanceFree(front);
                materials.destroy(blended);
                meshes.destroy(frontMesh);

                SurfaceLayout room;
                room.vertexCount = 4;
                room.indexCount = 6;
                const MeshId trailMesh = meshes.create("trail");
                CHECK(meshes.addDynamicSurface(trailMesh, room) == 0);
                const InstanceId trail = server.instanceCreate(trailMesh, lit);
                const MaterialId trailMaterial = materials.create("trail");
                CHECK(materials.setShading(trailMaterial, MaterialShading::Unshaded));
                server.instanceSetMaterialOverride(trail, trailMaterial);
                frames.draw(server);
                CHECK(pixelIs(driver, 32, 24, Rgb{ 0, 128, 0 }, "empty dynamic surface"));

                CHECK(meshes.updateSurface(trailMesh, 0,
                        quadData(1.0f, 0.5f, Math::Vec4(1.0f, 0.0f, 0.0f, 1.0f))));
                frames.draw(server);
                CHECK(pixelIs(driver, 32, 24, kRed, "dynamic surface after its first update"));

                SurfaceData moved = quadData(1.0f, 0.5f, Math::Vec4(0.0f, 0.0f, 1.0f, 1.0f));
                for (std::size_t i = 0; i < moved.positions.size(); ++i)
                    moved.positions[i].x += 1.0f;
                CHECK(meshes.updateSurface(trailMesh, 0, moved));
                frames.draw(server);
                CHECK(pixelIs(driver, 53, 24, kBlue, "dynamic surface moved and recoloured"));
                CHECK(pixelIs(driver, 32, 24, Rgb{ 0, 128, 0 }, "nothing left where it was"));

                CHECK(meshes.updateSurface(trailMesh, 0, SurfaceData()));
                frames.draw(server);
                CHECK(pixelIs(driver, 53, 24, Rgb{ 0, 128, 0 }, "dynamic surface emptied"));

                server.instanceFree(trail);
                materials.destroy(trailMaterial);
                meshes.destroy(trailMesh);
            }
            server.instanceSetMaterialOverride(wallInstance, matte);

            const MaterialId greenMatte = materials.create("green matte");
            CHECK(materials.setColor(greenMatte, MaterialParam::Albedo,
                    Math::Vec4(0.0f, 1.0f, 0.0f, 1.0f)));
            CHECK(materials.setFloat(greenMatte, MaterialParam::Specular, 0.0f));
            ambient.ambientSource = AmbientSource::Background;
            ambient.background = BackgroundMode::Color;
            ambient.backgroundColor = Math::Vec3(1.0f, 0.0f, 0.0f);
            server.scenarioSetEnvironment(lit, ambient);
            server.instanceSetVisible(wallInstance, false);
            frames.draw(server);
            CHECK(pixelIs(driver, 32, 24, kRed, "background colour"));
            server.instanceSetVisible(wallInstance, true);
            server.instanceSetMaterialOverride(wallInstance, greenMatte);
            frames.draw(server);
            CHECK(pixelIs(driver, 32, 24, kBlack, "green wall in red background ambient"));
            ambient.backgroundEnergy = 0.5f;
            server.scenarioSetEnvironment(lit, ambient);
            server.instanceSetMaterialOverride(wallInstance, matte);
            frames.draw(server);
            CHECK(pixelIs(driver, 32, 24, Rgb{ grey(0.5f).r, 0, 0 },
                    "background energy scales the ambient"));

            server.scenarioClearEnvironment(lit);
            server.viewportSetClearColor(viewport, Math::Vec4(0.1f, 0.2f, 0.3f, 1.0f));
            frames.draw(server);
            CHECK(pixelIs(driver, 32, 24, kClear,
                    "no environment takes the clear colour as ambient"));

            {
                const MaterialId glossy = materials.create("glossy");
                CHECK(materials.setColor(glossy, MaterialParam::Albedo,
                        Math::Vec4(0.0f, 0.0f, 0.0f, 1.0f)));
                CHECK(materials.setFloat(glossy, MaterialParam::Roughness, 0.5f));
                server.instanceSetMaterialOverride(wallInstance, glossy);
                EnvironmentSettings sky;
                sky.background = BackgroundMode::Color;
                sky.backgroundColor = Math::Vec3(1.0f, 1.0f, 1.0f);
                sky.ambientSource = AmbientSource::Color;
                server.scenarioSetEnvironment(lit, sky);
                frames.draw(server);
                float a = 0.0f;
                float b = 0.0f;
                dfgReference(127.5f / 128.0f, 0.5f + 1.0f / 128.0f, a, b);
                const float f0 = 0.04f;
                const float compensation = 1.0f + f0 * (1.0f / b - 1.0f);
                const float expected = compensation * ((1.0f - f0) * a + f0 * b);
                CHECK(pixelIs(driver, 32, 24, grey(expected),
                        "reflected background through the DFG"));
                sky.reflectionSource = ReflectionSource::Disabled;
                server.scenarioSetEnvironment(lit, sky);
                frames.draw(server);
                CHECK(pixelIs(driver, 32, 24, kBlack, "reflection disabled"));
                server.instanceSetMaterialOverride(wallInstance, matte);
            }

            {
                EnvironmentSettings skyEnvironment;
                skyEnvironment.background = BackgroundMode::Sky;
                skyEnvironment.ambientSource = AmbientSource::Color;
                skyEnvironment.sky.debanding = false;
                skyEnvironment.sky.radianceSize = 32;
                server.scenarioSetEnvironment(lit, skyEnvironment);
                server.instanceSetMaterialOverride(wallInstance, matte);
                server.instanceSetTransform(wallInstance,
                        Math::Mat4::Scale(Math::Vec3(0.3f, 0.3f, 1.0f)));

                auto toByte = [](float linear)
                {
                    const float encoded = linear <= 0.0031308f
                                                  ? linear * 12.92f
                                                  : 1.055f * powf(linear, 1.0f / 2.4f) - 0.055f;
                    return static_cast<int>(encoded * 255.0f + 0.5f);
                };
                auto lin = [](float value) {
                    return value < 0.04045f ? value / 12.92f
                                            : powf((value + 0.055f) / 1.055f, 2.4f);
                };
                auto mix = [](float a, float b, float t) { return a + (b - a) * t; };
                auto clamp01 = [](float v) { return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v); };
                const ProceduralSkySettings sky;
                const float skyTop[3] = { lin(sky.skyTopColor.x), lin(sky.skyTopColor.y),
                    lin(sky.skyTopColor.z) };
                const float skyHorizon[3] = { lin(sky.skyHorizonColor.x),
                    lin(sky.skyHorizonColor.y), lin(sky.skyHorizonColor.z) };
                const float groundBottom[3] = { lin(sky.groundBottomColor.x),
                    lin(sky.groundBottomColor.y), lin(sky.groundBottomColor.z) };
                const float groundHorizon[3] = { lin(sky.groundHorizonColor.x),
                    lin(sky.groundHorizonColor.y), lin(sky.groundHorizonColor.z) };
                const float tanHalf = tanf(30.0f * pi / 180.0f);
                const float aspect = static_cast<float>(kWidth) / static_cast<float>(kHeight);
                auto skyAt = [&](int px, int py, bool sunOn, float brightness)
                {
                    float x = ((px + 0.5f) / kWidth * 2.0f - 1.0f) * aspect * tanHalf;
                    float y = (1.0f - (py + 0.5f) / kHeight * 2.0f) * tanHalf;
                    float z = -1.0f;
                    const float length = sqrtf(x * x + y * y + z * z);
                    y /= length;
                    z /= length;
                    float result[3];
                    for (int c = 0; c < 3; ++c)
                    {
                        float skyColor = mix(skyTop[c], skyHorizon[c],
                                clamp01(powf(1.0f - y, 0.6f / sky.skyCurve)));
                        if (sunOn)
                        {
                            const float sunAngle = -z;
                            const float angleMax = cosf(sky.sunAngleMax * pi / 180.0f);
                            if (sunAngle > 1.0f) skyColor = 1.0f;
                            else if (sunAngle > angleMax)
                            {
                                const float c2 = (1.0f - sunAngle) / (1.0f - angleMax);
                                skyColor = mix(skyColor, 1.0f,
                                        clamp01(powf(1.0f - c2, 1.6f / powf(sky.sunCurve, 1.4f))));
                            }
                        }
                        const float groundColor = mix(groundBottom[c], groundHorizon[c],
                                clamp01(powf(1.0f + y, 0.6f / sky.groundCurve)));
                        result[c] = (y >= 0.0f ? skyColor : groundColor) * sky.energy * brightness;
                    }
                    return Rgb{ toByte(result[0]), toByte(result[1]), toByte(result[2]) };
                };

                frames.draw(server);
                CHECK(pixelIs(driver, 32, 24, kBlack, "wall in front of the sky"));
                CHECK(pixelIs(driver, 32, 3, skyAt(32, 3, false, 1.0f), "sky above"));
                CHECK(pixelIs(driver, 5, 12, skyAt(5, 12, false, 1.0f), "sky corner"));
                CHECK(pixelIs(driver, 32, 44, skyAt(32, 44, false, 1.0f), "ground below"));

                {
                    SurfaceData glassData = quadData(1.0f, 0.5f, white);
                    for (int i = 0; i < 4; ++i)
                        glassData.normals.push_back(Math::Vec3(0.0f, 0.0f, 1.0f));
                    const MeshId glassMesh = meshes.create("glass");
                    CHECK(meshes.addSurface(glassMesh, glassData));
                    const InstanceId glass = server.instanceCreate(glassMesh, lit);
                    const MaterialId glassMaterial = materials.create("glass");
                    CHECK(materials.setShading(glassMaterial, MaterialShading::Unshaded));
                    CHECK(materials.setColor(glassMaterial, MaterialParam::Albedo,
                            Math::Vec4(1.0f, 0.0f, 0.0f, 1.0f)));
                    CHECK(materials.setTransparency(glassMaterial,
                            MaterialTransparency::AlphaBlend));
                    server.instanceSetMaterialOverride(glass, glassMaterial);
                    frames.draw(server);
                    CHECK(pixelIs(driver, 32, 24, kRed, "transparent quad drawn after the sky"));
                    CHECK(pixelIs(driver, 32, 3, skyAt(32, 3, false, 1.0f),
                            "sky untouched by the transparent quad"));
                    server.instanceFree(glass);
                    materials.destroy(glassMaterial);
                    meshes.destroy(glassMesh);
                }

                skyEnvironment.sky.debanding = true;
                server.scenarioSetEnvironment(lit, skyEnvironment);
                frames.draw(server);
                CHECK(pixelIs(driver, 32, 3, skyAt(32, 3, false, 1.0f), "sky with debanding"));

                skyEnvironment.sky.debanding = false;
                skyEnvironment.backgroundEnergy = 0.5f;
                server.scenarioSetEnvironment(lit, skyEnvironment);
                frames.draw(server);
                CHECK(pixelIs(driver, 32, 3, skyAt(32, 3, false, 0.5f), "background energy"));
                skyEnvironment.backgroundEnergy = 1.0f;

                const LightId sunLight = server.lightCreate(LightType::Directional);
                server.lightSetScenario(sunLight, lit);
                server.lightSetTransform(sunLight, Math::Mat4::RotationY(pi));
                server.instanceSetVisible(wallInstance, false);
                server.scenarioSetEnvironment(lit, skyEnvironment);
                frames.draw(server);
                CHECK(pixelIs(driver, 32, 3, skyAt(32, 3, true, 1.0f), "sun in the sky"));
                CHECK(pixelIs(driver, 32, 20, skyAt(32, 20, true, 1.0f), "sun near the horizon"));
                server.lightFree(sunLight);

                server.cameraSetTransform(camera,
                        translation(0.0f, 0.0f, 3.0f) * Math::Mat4::RotationX(pi / 2.0f));
                frames.draw(server);
                CHECK(pixelIs(driver, 32, 24,
                        Rgb{ toByte(skyTop[0]), toByte(skyTop[1]), toByte(skyTop[2]) },
                        "camera looking up"));
                server.cameraSetTransform(camera, translation(0.0f, 0.0f, 3.0f));
                server.instanceSetVisible(wallInstance, true);
                server.instanceSetTransform(wallInstance, Math::Mat4::Identity());
            }

            {
                auto toByte = [](float linear)
                {
                    const float encoded = linear <= 0.0031308f
                                                  ? linear * 12.92f
                                                  : 1.055f * powf(linear, 1.0f / 2.4f) - 0.055f;
                    return static_cast<int>(encoded * 255.0f + 0.5f);
                };
                auto lin = [](float value) {
                    return value < 0.04045f ? value / 12.92f
                                            : powf((value + 0.055f) / 1.055f, 2.4f);
                };
                auto pixelAt = [&](int x, int y)
                {
                    unsigned char pixel[4] = { 0, 0, 0, 0 };
                    prisma::Rect rect;
                    rect.x = x;
                    rect.y = y;
                    rect.width = 1;
                    rect.height = 1;
                    driver->readPixels(prisma::RenderTarget(), rect, pixel);
                    return Rgb{ pixel[0], pixel[1], pixel[2] };
                };
                const ProceduralSkySettings sky;
                const Rgb top = { toByte(lin(sky.skyTopColor.x)), toByte(lin(sky.skyTopColor.y)),
                    toByte(lin(sky.skyTopColor.z)) };
                const Rgb horizon = { toByte(lin(sky.skyHorizonColor.x)),
                    toByte(lin(sky.skyHorizonColor.y)), toByte(lin(sky.skyHorizonColor.z)) };

                EnvironmentSettings radianceEnvironment;
                radianceEnvironment.background = BackgroundMode::Sky;
                radianceEnvironment.sky.debanding = false;
                radianceEnvironment.sky.radianceSize = 32;
                server.scenarioSetEnvironment(lit, radianceEnvironment);
                server.instanceSetVisible(wallInstance, true);
                server.instanceSetTransform(wallInstance, Math::Mat4::RotationX(-pi / 4.0f));

                const MaterialId mirror = materials.create("mirror");
                CHECK(materials.setColor(mirror, MaterialParam::Albedo,
                        Math::Vec4(1.0f, 1.0f, 1.0f, 1.0f)));
                CHECK(materials.setFloat(mirror, MaterialParam::Metallic, 1.0f));
                CHECK(materials.setFloat(mirror, MaterialParam::Roughness, 0.0f));
                server.instanceSetMaterialOverride(wallInstance, mirror);
                frames.draw(server);
                CHECK(pixelIs(driver, 32, 24, top, "mirror reflecting the sky above"));

                CHECK(materials.setFloat(mirror, MaterialParam::Roughness, 1.0f));
                frames.draw(server);
                {
                    const Rgb rough = pixelAt(32, 24);
                    CHECK(rough.r > top.r + 4 && rough.r < horizon.r);
                    CHECK(rough.g != top.g || rough.b != top.b);
                }

                const MaterialId diffuse = materials.create("diffuse sky");
                CHECK(materials.setFloat(diffuse, MaterialParam::Specular, 0.0f));
                server.instanceSetMaterialOverride(wallInstance, diffuse);
                frames.draw(server);
                const Rgb lit_up = pixelAt(32, 24);
                server.instanceSetTransform(wallInstance, Math::Mat4::RotationX(pi / 4.0f));
                frames.draw(server);
                const Rgb lit_down = pixelAt(32, 24);
                CHECK(lit_up.b > 20 && lit_up.b <= horizon.b + 2);
                CHECK(lit_up.b > lit_down.b);

                radianceEnvironment.ambientSkyContribution = 0.0f;
                server.scenarioSetEnvironment(lit, radianceEnvironment);
                frames.draw(server);
                CHECK(pixelIs(driver, 32, 24, kBlack, "no sky contribution to the ambient"));
                radianceEnvironment.ambientSkyContribution = 1.0f;

                radianceEnvironment.ambientSource = AmbientSource::Disabled;
                server.scenarioSetEnvironment(lit, radianceEnvironment);
                frames.draw(server);
                CHECK(pixelIs(driver, 32, 24, kBlack, "ambient disabled with a sky"));
                radianceEnvironment.ambientSource = AmbientSource::Background;

                server.instanceSetTransform(wallInstance, Math::Mat4::RotationX(-pi / 4.0f));
                CHECK(materials.setFloat(mirror, MaterialParam::Roughness, 0.0f));
                server.instanceSetMaterialOverride(wallInstance, mirror);
                radianceEnvironment.sky.skyTopColor = Math::Vec3(1.0f, 0.0f, 0.0f);
                server.scenarioSetEnvironment(lit, radianceEnvironment);
                for (int frame = 0; frame < 10; ++frame) frames.draw(server);
                CHECK(pixelIs(driver, 32, 24, kRed, "radiance rebuilt after the sky changed"));

                radianceEnvironment.reflectionSource = ReflectionSource::Disabled;
                server.scenarioSetEnvironment(lit, radianceEnvironment);
                frames.draw(server);
                CHECK(pixelIs(driver, 32, 24, kBlack, "reflection disabled with a sky"));
                server.instanceSetTransform(wallInstance, Math::Mat4::Identity());
            }

            {
                auto toByte = [](float linear)
                {
                    const float encoded = linear <= 0.0031308f
                                                  ? linear * 12.92f
                                                  : 1.055f * powf(linear, 1.0f / 2.4f) - 0.055f;
                    return static_cast<int>(encoded * 255.0f + 0.5f);
                };
                auto grey = [&](float linear)
                {
                    const int value = toByte(linear);
                    return Rgb{ value, value, value };
                };
                EnvironmentSettings darkEnvironment;
                darkEnvironment.ambientSource = AmbientSource::Color;
                darkEnvironment.reflectionSource = ReflectionSource::Disabled;
                server.scenarioSetEnvironment(lit, darkEnvironment);
                server.instanceSetVisible(wallInstance, true);
                server.instanceSetTransform(wallInstance, Math::Mat4::Identity());
                server.instanceSetMaterialOverride(wallInstance, matte);
                server.setDirectionalShadowSize(2048, false);

                SurfaceData occluderData = quadData(1.0f, 0.3f, white);
                for (int i = 0; i < 4; ++i)
                    occluderData.normals.push_back(Math::Vec3(0.0f, 0.0f, 1.0f));
                const MeshId occluderMesh = meshes.create("occluder");
                CHECK(meshes.addSurface(occluderMesh, occluderData));
                const InstanceId occluder = server.instanceCreate(occluderMesh, lit);
                server.instanceSetMaterialOverride(occluder, matte);

                const LightId sun = server.lightCreate(LightType::Directional);
                server.lightSetScenario(sun, lit);
                server.lightSetTransform(sun, Math::Mat4::RotationY(pi / 6.0f));
                const float lighted = cosf(pi / 6.0f);

                frames.draw(server);
                CHECK(server.getStats().shadowCascades == 0);
                CHECK(pixelIs(driver, 24, 24, grey(lighted), "wall lit without shadows"));

                server.lightSetShadow(sun, true);
                frames.draw(server);
                CHECK(server.getStats().shadowCascades == 4);
                CHECK(server.getStats().shadowCasters >= 4);
                CHECK(pixelIs(driver, 24, 24, kBlack, "wall in the shadow of the occluder"));
                CHECK(pixelIs(driver, 45, 24, grey(lighted), "wall outside of the shadow"));
                CHECK(pixelIs(driver, 24, 4, grey(lighted), "wall above the shadow"));
                {
                    const std::uint32_t fullTriangles = server.getStats().shadowTriangles;
                    CHECK(fullTriangles > 0);
                    SurfaceData awayData = occluderData;
                    for (std::size_t i = 0; i < awayData.positions.size(); ++i)
                        awayData.positions[i].x += 50.0f;
                    const MeshId awayMesh = meshes.create("occluder shadow");
                    CHECK(meshes.addSurface(awayMesh, awayData));
                    CHECK(!meshes.setShadowMesh(occluderMesh, occluderMesh));
                    CHECK(meshes.setShadowMesh(occluderMesh, MeshId()));
                    CHECK(meshes.setShadowMesh(occluderMesh, awayMesh));
                    CHECK(meshes.getShadowMesh(occluderMesh) == awayMesh);
                    frames.draw(server);
                    CHECK(server.getStats().shadowTriangles == fullTriangles);
                    CHECK(pixelIs(driver, 24, 24, grey(lighted), "shadow mesh moved the shadow"));

                    SurfaceData oneTriangle = occluderData;
                    oneTriangle.indices.resize(3);
                    const MeshId smallMesh = meshes.create("occluder small shadow");
                    CHECK(meshes.addSurface(smallMesh, oneTriangle));
                    CHECK(meshes.setShadowMesh(occluderMesh, smallMesh));
                    frames.draw(server);
                    CHECK(server.getStats().shadowTriangles < fullTriangles);

                    meshes.destroy(smallMesh);
                    CHECK(!meshes.getShadowMesh(occluderMesh).valid());
                    frames.draw(server);
                    CHECK(server.getStats().shadowTriangles == fullTriangles);
                    CHECK(pixelIs(driver, 24, 24, kBlack, "shadow back after the shadow mesh"));
                    meshes.destroy(awayMesh);
                }

                server.instanceSetCastShadows(occluder, false);
                frames.draw(server);
                CHECK(pixelIs(driver, 24, 24, grey(lighted), "occluder that casts no shadow"));
                server.instanceSetCastShadows(occluder, true);

                const MaterialId additive = materials.create("additive");
                CHECK(materials.setBlendMode(additive, MaterialBlend::Add));
                server.instanceSetMaterialOverride(occluder, additive);
                frames.draw(server);
                CHECK(pixelIs(driver, 24, 24, grey(lighted), "blended occluder casts no shadow"));
                server.instanceSetMaterialOverride(occluder, matte);

                server.lightSetParam(sun, LightParam::ShadowOpacity, 0.5f);
                frames.draw(server);
                CHECK(pixelIs(driver, 24, 24, grey(lighted * 0.5f), "half opaque shadow"));
                server.lightSetParam(sun, LightParam::ShadowOpacity, 1.0f);

                server.lightSetDirectionalShadowMode(sun, DirectionalShadowMode::Orthogonal);
                frames.draw(server);
                CHECK(server.getStats().shadowCascades == 1);
                CHECK(pixelIs(driver, 24, 24, kBlack, "single cascade"));
                CHECK(pixelIs(driver, 45, 24, grey(lighted), "single cascade, lit side"));

                server.lightSetDirectionalShadowMode(sun, DirectionalShadowMode::Parallel2Splits);
                server.lightSetBlendSplits(sun, true);
                frames.draw(server);
                CHECK(server.getStats().shadowCascades == 2);
                CHECK(pixelIs(driver, 24, 24, kBlack, "two cascades blended"));
                server.lightSetBlendSplits(sun, false);
                server.lightSetDirectionalShadowMode(sun, DirectionalShadowMode::Parallel4Splits);

                server.setDirectionalShadowQuality(ShadowQuality::Hard);
                frames.draw(server);
                CHECK(pixelIs(driver, 24, 24, kBlack, "hard shadows"));
                server.setDirectionalShadowQuality(ShadowQuality::SoftHigh);
                frames.draw(server);
                CHECK(pixelIs(driver, 24, 24, kBlack, "high quality shadows"));
                server.setDirectionalShadowQuality(ShadowQuality::SoftLow);

                server.setDirectionalShadowSize(512, false);
                frames.draw(server);
                CHECK(pixelIs(driver, 24, 24, kBlack, "smaller atlas"));
                CHECK(pixelIs(driver, 45, 24, grey(lighted), "smaller atlas, lit side"));
                server.setDirectionalShadowSize(2048, false);

                server.lightSetParam(sun, LightParam::Size, 1.0f);
                frames.draw(server);
                CHECK(pixelIs(driver, 24, 24, kBlack, "soft shadow from the angular size"));
                CHECK(pixelIs(driver, 45, 24, grey(lighted), "soft shadow, lit side"));
                server.lightSetParam(sun, LightParam::Size, 0.0f);

                server.lightSetVisible(sun, false);
                frames.draw(server);
                CHECK(server.getStats().shadowCascades == 0);
                server.lightSetVisible(sun, true);

                server.lightFree(sun);
                server.instanceFree(occluder);
            }

            {
                EnvironmentSettings darkEnvironment;
                darkEnvironment.ambientSource = AmbientSource::Color;
                darkEnvironment.reflectionSource = ReflectionSource::Disabled;
                server.scenarioSetEnvironment(lit, darkEnvironment);
                server.instanceSetVisible(wallInstance, true);
                server.instanceSetTransform(wallInstance, Math::Mat4::Identity());
                server.instanceSetMaterialOverride(wallInstance, matte);
                server.viewportSetPositionalShadowAtlasSize(viewport, 1024, false);

                const float tanHalf = tanf(pi / 6.0f);
                auto wallPoint = [&](int px, int py)
                {
                    const float nx = (static_cast<float>(px) + 0.5f) / kWidth * 2.0f - 1.0f;
                    const float ny = 1.0f - (static_cast<float>(py) + 0.5f) / kHeight * 2.0f;
                    const float half = 3.0f * tanHalf;
                    return Math::Vec3(nx * half * static_cast<float>(kWidth) / kHeight, ny * half,
                            0.0f);
                };
                auto attenuation = [](float distance, float radius)
                {
                    float nd = distance / radius;
                    nd *= nd;
                    nd *= nd;
                    nd = 1.0f - nd;
                    nd *= nd;
                    return nd / distance;
                };
                auto spotAt = [&](const Math::Vec3& point, const Math::Vec3& position,
                                      const Math::Vec3& target, float range, float coneDegrees)
                {
                    const Math::Vec3 toPoint = point - position;
                    const float distance = toPoint.Length();
                    const Math::Vec3 away = toPoint * (1.0f / distance);
                    const Math::Vec3 axis = (target - position).Normalized();
                    const float cone = cosf(coneDegrees * pi / 180.0f);
                    const float scos = fmaxf(Math::Vec3::Dot(away, axis), cone);
                    const float rim = fmaxf(0.0001f, (1.0f - scos) / (1.0f - cone));
                    return attenuation(distance, range) * (1.0f - rim) * (-away.z);
                };
                auto omniAtPoint =
                        [&](const Math::Vec3& point, const Math::Vec3& position, float range)
                {
                    const Math::Vec3 toPoint = point - position;
                    const float distance = toPoint.Length();
                    return attenuation(distance, range) * (-toPoint.z / distance);
                };
                auto aimed = [](const Math::Vec3& from, const Math::Vec3& to) {
                    return Math::Mat4::LookAt(from, to, Math::Vec3(0.0f, 1.0f, 0.0f))
                            .InverseAffine();
                };
                auto facing = [&](const Math::Vec3& at, const Math::Vec3& from)
                { return aimed(at, at + (at - from)); };
                auto alongRay = [](const Math::Vec3& from, const Math::Vec3& to, float distance)
                { return from + (to - from).Normalized() * distance; };
                const int kLooseTolerance = 6;
                const int centreX = 32;
                const int centreY = 24;

                SurfaceData blockerData = quadData(0.0f, 0.3f, white);
                for (int i = 0; i < 4; ++i)
                    blockerData.normals.push_back(Math::Vec3(0.0f, 0.0f, 1.0f));
                const MeshId blockerMesh = meshes.create("positional blocker");
                CHECK(meshes.addSurface(blockerMesh, blockerData));
                const InstanceId blocker = server.instanceCreate(blockerMesh, lit);
                server.instanceSetMaterialOverride(blocker, matte);

                const Math::Vec3 origin(0.0f, 0.0f, 0.0f);
                const Math::Vec3 lampPosition(-4.0f, 0.0f, 4.0f);
                const Math::Vec3 blockerPosition = alongRay(lampPosition, origin, 2.0f);
                server.instanceSetTransform(blocker, facing(blockerPosition, lampPosition));

                const LightId lamp = server.lightCreate(LightType::Spot);
                server.lightSetScenario(lamp, lit);
                server.lightSetParam(lamp, LightParam::Range, 10.0f);
                server.lightSetParam(lamp, LightParam::SpotAngle, 40.0f);
                server.lightSetTransform(lamp, aimed(lampPosition, origin));
                auto litSpot =
                        [&](int px, int py, const Math::Vec3& position, const Math::Vec3& target)
                { return grey(spotAt(wallPoint(px, py), position, target, 10.0f, 40.0f)); };

                frames.draw(server);
                CHECK(server.getStats().shadowLights == 0);
                CHECK(server.getStats().shadowPasses == 0);
                CHECK(pixelWithin(driver, centreX, centreY,
                        litSpot(centreX, centreY, lampPosition, origin), kLooseTolerance,
                        "spot light, no shadow, centre"));
                CHECK(pixelWithin(driver, 32, 4, litSpot(32, 4, lampPosition, origin),
                        kLooseTolerance, "spot light, no shadow, above"));

                server.lightSetShadow(lamp, true);
                frames.draw(server);
                CHECK(server.getStats().shadowLights == 1);
                CHECK(server.getStats().shadowPasses == 1);
                CHECK(server.getStats().shadowCascades == 0);
                CHECK(server.getStats().shadowCasters >= 2);
                CHECK(pixelIs(driver, centreX, centreY, kBlack, "wall in the shadow of the spot"));
                CHECK(pixelWithin(driver, 32, 4, litSpot(32, 4, lampPosition, origin),
                        kLooseTolerance, "wall above the spot shadow"));
                CHECK(pixelWithin(driver, 20, 24, kBlack, 2, "wall inside the spot shadow, left"));
                {
                    std::uint32_t quadrant = 9;
                    std::uint32_t index = 9;
                    bool omni = true;
                    CHECK(server.viewportGetPositionalShadowSlot(viewport, lamp, quadrant, index,
                            omni));
                    CHECK(quadrant < 4 && !omni);
                }

                server.instanceSetCastShadows(blocker, false);
                frames.draw(server);
                CHECK(pixelWithin(driver, centreX, centreY,
                        litSpot(centreX, centreY, lampPosition, origin), kLooseTolerance,
                        "blocker that casts no shadow"));
                server.instanceSetCastShadows(blocker, true);

                server.lightSetParam(lamp, LightParam::ShadowOpacity, 0.5f);
                frames.draw(server);
                {
                    const float half = 0.5f * spotAt(wallPoint(centreX, centreY), lampPosition,
                                                      origin, 10.0f, 40.0f);
                    CHECK(pixelWithin(driver, centreX, centreY, grey(half), kLooseTolerance,
                            "half opaque spot shadow"));
                }
                server.lightSetParam(lamp, LightParam::ShadowOpacity, 1.0f);

                server.lightSetParam(lamp, LightParam::ShadowBlur, 0.0f);
                frames.draw(server);
                CHECK(pixelIs(driver, centreX, centreY, kBlack, "spot shadow without blur"));
                server.lightSetParam(lamp, LightParam::ShadowBlur, 1.0f);

                server.setPositionalShadowQuality(ShadowQuality::Hard);
                frames.draw(server);
                CHECK(pixelIs(driver, centreX, centreY, kBlack, "hard spot shadow"));
                server.setPositionalShadowQuality(ShadowQuality::SoftHigh);
                frames.draw(server);
                CHECK(pixelIs(driver, centreX, centreY, kBlack, "high quality spot shadow"));
                server.setPositionalShadowQuality(ShadowQuality::SoftLow);

                server.lightSetParam(lamp, LightParam::Size, 0.2f);
                frames.draw(server);
                CHECK(pixelIs(driver, centreX, centreY, kBlack, "soft spot shadow from the size"));
                CHECK(pixelWithin(driver, 32, 4, litSpot(32, 4, lampPosition, origin),
                        kLooseTolerance, "soft spot shadow, lit side"));
                server.lightSetParam(lamp, LightParam::Size, 0.0f);

                const Math::Vec3 raised(-4.0f, 1.0f, 4.0f);
                server.lightSetTransform(lamp, aimed(raised, origin));
                frames.draw(server);
                CHECK(pixelWithin(driver, centreX, centreY,
                        litSpot(centreX, centreY, raised, origin), kLooseTolerance,
                        "spot shadow moved away from the centre"));
                CHECK(pixelIs(driver, 32, 42, kBlack, "spot shadow moved to the lower wall"));
                server.lightSetTransform(lamp, aimed(lampPosition, origin));

                server.lightSetShadow(lamp, false);
                frames.draw(server);
                CHECK(server.getStats().shadowPasses == 0);
                CHECK(pixelWithin(driver, centreX, centreY,
                        litSpot(centreX, centreY, lampPosition, origin), kLooseTolerance,
                        "spot shadow disabled again"));
                server.lightSetShadow(lamp, true);

                server.lightSetParam(lamp, LightParam::Range, 2.0f);
                frames.draw(server);
                CHECK(server.getStats().shadowLights == 0);
                CHECK(pixelIs(driver, centreX, centreY, kBlack, "spot range short of the wall"));
                server.lightSetParam(lamp, LightParam::Range, 10.0f);

                const Math::Vec3 otherPosition(4.0f, 0.0f, 4.0f);
                const Math::Vec3 otherTarget(0.0f, 1.0f, 0.0f);
                const Math::Vec3 otherBlockerPosition = alongRay(otherPosition, otherTarget, 2.0f);
                const InstanceId otherBlocker = server.instanceCreate(blockerMesh, lit);
                server.instanceSetMaterialOverride(otherBlocker, matte);
                server.instanceSetTransform(otherBlocker,
                        facing(otherBlockerPosition, otherPosition));
                const LightId other = server.lightCreate(LightType::Spot);
                server.lightSetScenario(other, lit);
                server.lightSetParam(other, LightParam::Range, 10.0f);
                server.lightSetParam(other, LightParam::SpotAngle, 40.0f);
                server.lightSetTransform(other, aimed(otherPosition, otherTarget));
                server.lightSetShadow(other, true);
                frames.draw(server);
                CHECK(server.getStats().shadowLights == 2);
                CHECK(server.getStats().shadowPasses == 2);
                CHECK(pixelWithin(driver, 32, 26,
                        grey(spotAt(wallPoint(32, 26), otherPosition, otherTarget, 10.0f, 40.0f)),
                        kLooseTolerance, "shadowed by the first spot only"));
                CHECK(pixelWithin(driver, 32, 8, litSpot(32, 8, lampPosition, origin),
                        kLooseTolerance, "shadowed by the second spot only"));
                CHECK(pixelIs(driver, 32, 17, kBlack, "shadowed by both spots"));
                {
                    std::uint32_t quadrantA = 0;
                    std::uint32_t indexA = 0;
                    std::uint32_t quadrantB = 0;
                    std::uint32_t indexB = 0;
                    bool omniA = false;
                    bool omniB = false;
                    CHECK(server.viewportGetPositionalShadowSlot(viewport, lamp, quadrantA, indexA,
                            omniA));
                    CHECK(server.viewportGetPositionalShadowSlot(viewport, other, quadrantB, indexB,
                            omniB));
                    CHECK(quadrantA != quadrantB || indexA != indexB);
                }

                server.lightFree(other);
                server.instanceFree(otherBlocker);
                frames.draw(server);
                CHECK(server.getStats().shadowLights == 1);
                {
                    std::uint32_t quadrant = 0;
                    std::uint32_t index = 0;
                    bool omni = false;
                    CHECK(server.viewportGetPositionalShadowSlot(viewport, lamp, quadrant, index,
                            omni));
                    CHECK(pixelIs(driver, centreX, centreY, kBlack,
                            "first spot after the second left"));
                    server.viewportSetPositionalShadowAtlasQuadrantSubdivision(viewport, quadrant,
                            0);
                    frames.draw(server);
                    std::uint32_t movedQuadrant = quadrant;
                    std::uint32_t movedIndex = index;
                    CHECK(server.viewportGetPositionalShadowSlot(viewport, lamp, movedQuadrant,
                            movedIndex, omni));
                    CHECK(movedQuadrant != quadrant);
                    CHECK(pixelIs(driver, centreX, centreY, kBlack,
                            "spot moved to another quadrant"));
                    server.viewportSetPositionalShadowAtlasQuadrantSubdivision(viewport, quadrant,
                            4);
                }
                server.viewportSetPositionalShadowAtlasSize(viewport, 512, false);
                frames.draw(server);
                CHECK(pixelIs(driver, centreX, centreY, kBlack, "spot shadow in a smaller atlas"));
                server.viewportSetPositionalShadowAtlasSize(viewport, 1024, false);

                server.lightFree(lamp);
                frames.draw(server);
                CHECK(server.getStats().shadowLights == 0);

                const LightId bulb = server.lightCreate(LightType::Omni);
                server.lightSetScenario(bulb, lit);
                server.lightSetParam(bulb, LightParam::Range, 12.0f);
                server.instanceSetTransform(blocker, facing(blockerPosition, lampPosition));
                const float turns[4] = { 0.0f, pi, pi / 2.0f, -pi / 2.0f };
                const char* const turnLabels[2][4] = { { "omni shadow, front paraboloid",
                                                           "omni shadow, back paraboloid",
                                                           "omni shadow, light turned a quarter",
                                                           "omni shadow, light turned a quarter "
                                                           "back" },
                    { "cube shadow, front", "cube shadow, back", "cube shadow, turned a quarter",
                        "cube shadow, turned a quarter back" } };
                for (int mode = 0; mode < 2; ++mode)
                {
                    const bool cube = mode == 1;
                    server.instanceSetCastShadows(wallInstance, cube);
                    server.lightSetOmniShadowMode(bulb,
                            cube ? OmniShadowMode::Cube : OmniShadowMode::DualParaboloid);
                    for (int turn = 0; turn < 4; ++turn)
                    {
                        server.lightSetTransform(bulb,
                                translation(lampPosition.x, lampPosition.y, lampPosition.z) *
                                        Math::Mat4::RotationY(turns[turn]));
                        server.lightSetShadow(bulb, false);
                        frames.draw(server);
                        CHECK(pixelWithin(driver, centreX, centreY,
                                grey(omniAtPoint(wallPoint(centreX, centreY), lampPosition, 12.0f)),
                                kLooseTolerance, "omni light, no shadow"));
                        server.lightSetShadow(bulb, true);
                        frames.draw(server);
                        CHECK(server.getStats().shadowLights == 1);
                        CHECK(server.getStats().shadowPasses == (cube ? 6u : 2u));
                        CHECK(pixelIs(driver, centreX, centreY, kBlack, turnLabels[mode][turn]));
                        CHECK(pixelWithin(driver, 32, 4,
                                grey(omniAtPoint(wallPoint(32, 4), lampPosition, 12.0f)),
                                kLooseTolerance, "wall above the omni shadow"));
                    }
                    {
                        std::uint32_t quadrant = 0;
                        std::uint32_t index = 0;
                        bool omni = false;
                        CHECK(server.viewportGetPositionalShadowSlot(viewport, bulb, quadrant,
                                index, omni));
                        CHECK(omni);
                    }

                    server.instanceSetCastShadows(blocker, false);
                    frames.draw(server);
                    CHECK(pixelWithin(driver, centreX, centreY,
                            grey(omniAtPoint(wallPoint(centreX, centreY), lampPosition, 12.0f)),
                            kLooseTolerance, "omni blocker that casts no shadow"));
                    server.instanceSetCastShadows(blocker, true);

                    const Math::Vec3 movedBulb(-4.0f, 1.0f, 4.0f);
                    server.lightSetTransform(bulb,
                            translation(movedBulb.x, movedBulb.y, movedBulb.z));
                    frames.draw(server);
                    CHECK(pixelWithin(driver, centreX, centreY,
                            grey(omniAtPoint(wallPoint(centreX, centreY), movedBulb, 12.0f)),
                            kLooseTolerance, "omni shadow moved away from the centre"));
                    CHECK(pixelIs(driver, 32, 42, kBlack, "omni shadow moved to the lower wall"));
                }

                server.lightSetShadow(bulb, false);
                frames.draw(server);
                CHECK(server.getStats().shadowLights == 0);
                server.lightFree(bulb);
                server.instanceFree(blocker);
                server.instanceSetCastShadows(wallInstance, true);
                server.instanceSetTransform(wallInstance, Math::Mat4::Identity());
            }

            {
                BatchRenderer& batch = server.batch();
                EnvironmentSettings plain;
                plain.ambientSource = AmbientSource::Color;
                plain.reflectionSource = ReflectionSource::Disabled;
                server.scenarioSetEnvironment(lit, plain);
                server.instanceSetVisible(wallInstance, false);
                auto pixelAt = [&](int x, int y)
                {
                    unsigned char pixel[4] = { 0, 0, 0, 0 };
                    prisma::Rect rect;
                    rect.x = x;
                    rect.y = y;
                    rect.width = 1;
                    rect.height = 1;
                    driver->readPixels(prisma::RenderTarget(), rect, pixel);
                    return Rgb{ pixel[0], pixel[1], pixel[2] };
                };

                batch.setColor((unsigned char) 255, (unsigned char) 0, (unsigned char) 0,
                        (unsigned char) 255);
                batch.drawRect(10.0f, 10.0f, 20.0f, 10.0f);
                batch.setColor((unsigned char) 0, (unsigned char) 255, (unsigned char) 0,
                        (unsigned char) 255);
                batch.drawLine(0.0f, 30.5f, 64.0f, 30.5f);
                batch.setColor((unsigned char) 0, (unsigned char) 0, (unsigned char) 255,
                        (unsigned char) 255);
                batch.drawTriangle(40.0f, 4.0f, 60.0f, 4.0f, 60.0f, 24.0f);
                batch.setColor((unsigned char) 255, (unsigned char) 255, (unsigned char) 255,
                        (unsigned char) 255);
                batch.drawRect(2.5f, 36.5f, 10.0f, 8.0f, false);
                frames.draw(server);
                CHECK(server.getStats().batchDrawCalls >= 3);
                CHECK(pixelIs(driver, 15, 14, kRed, "batch rectangle"));
                CHECK(pixelIs(driver, 5, 5, kClear, "batch background"));
                CHECK(pixelIs(driver, 32, 30, kGreen, "batch line"));
                CHECK(pixelIs(driver, 56, 8, kBlue, "batch triangle"));
                CHECK(pixelIs(driver, 42, 20, kClear, "outside of the batch triangle"));
                CHECK(pixelIs(driver, 7, 36, Rgb{ 255, 255, 255 }, "batch rectangle outline"));
                CHECK(pixelIs(driver, 7, 40, kClear, "inside of the batch outline"));
                frames.draw(server);
                CHECK(server.getStats().batchDrawCalls == 0);
                CHECK(pixelIs(driver, 15, 14, kClear, "the batch starts empty each frame"));

                batch.setColor((unsigned char) 255, (unsigned char) 0, (unsigned char) 0,
                        (unsigned char) 255);
                batch.pushMatrix();
                batch.translate(40.0f, 30.0f);
                batch.drawRect(0.0f, 0.0f, 5.0f, 5.0f);
                batch.popMatrix();
                batch.setClipRect(0.0f, 0.0f, 20.0f, 20.0f);
                batch.setColor((unsigned char) 0, (unsigned char) 255, (unsigned char) 0,
                        (unsigned char) 255);
                batch.drawRect(10.0f, 10.0f, 40.0f, 40.0f);
                batch.clearClipRect();
                frames.draw(server);
                CHECK(pixelIs(driver, 42, 32, kRed, "batch matrix stack"));
                CHECK(pixelIs(driver, 15, 15, kGreen, "batch clip, inside"));
                CHECK(pixelIs(driver, 25, 25, kClear, "batch clip, outside"));

                batch.setColor((unsigned char) 255, (unsigned char) 255, (unsigned char) 255,
                        (unsigned char) 128);
                batch.drawRect(0.0f, 0.0f, 64.0f, 48.0f);
                frames.draw(server);
                {
                    const Rgb mixed = pixelAt(32, 24);
                    CHECK(mixed.r > 130 && mixed.r < 150 && mixed.g > 150 && mixed.g < 165 &&
                            mixed.b > 160 && mixed.b < 175);
                }

                batch.setColor((unsigned char) 255, (unsigned char) 255, (unsigned char) 255,
                        (unsigned char) 255);
                batch.drawText(2.0f, 2.0f, 8.0f, "A");
                frames.draw(server);
                {
                    int lit_pixels = 0;
                    for (int y = 2; y < 10; ++y)
                        for (int x = 2; x < 10; ++x)
                            if (pixelAt(x, y).r > 128) ++lit_pixels;
                    CHECK(lit_pixels >= 12 && lit_pixels <= 40);
                }
                CHECK(batch.textWidth(8.0f, "abc\nde") == 24.0f);

                batch.setDepthTest(true);
                batch.setDepthWrite(true);
                batch.setBlend(false);
                batch.setColor((unsigned char) 255, (unsigned char) 0, (unsigned char) 0,
                        (unsigned char) 255);
                batch.drawTriangle3D(Math::Vec3(-1.0f, -1.0f, 1.0f), Math::Vec3(1.0f, -1.0f, 1.0f),
                        Math::Vec3(0.0f, 1.0f, 1.0f));
                frames.draw(server);
                CHECK(server.getStats().batchDrawCalls == 1);
                CHECK(pixelIs(driver, 32, 24, kRed, "batch triangle in the world"));

                server.instanceSetVisible(wallInstance, true);
                server.instanceSetMaterialOverride(wallInstance, matte);
                batch.drawTriangle3D(Math::Vec3(-1.0f, -1.0f, -1.0f),
                        Math::Vec3(1.0f, -1.0f, -1.0f), Math::Vec3(0.0f, 1.0f, -1.0f));
                frames.draw(server);
                CHECK(pixelIs(driver, 32, 24, kBlack, "batch triangle behind the wall"));
                batch.setDepthTest(false);
                batch.drawTriangle3D(Math::Vec3(-1.0f, -1.0f, -1.0f),
                        Math::Vec3(1.0f, -1.0f, -1.0f), Math::Vec3(0.0f, 1.0f, -1.0f));
                frames.draw(server);
                CHECK(pixelIs(driver, 32, 24, kRed, "batch triangle without the depth test"));
                batch.setDefault3DState();
                batch.setDepthTest(false);
                batch.setDepthWrite(false);
                batch.setBlend(true);
                batch.drawWireBox(-1.0f, -1.0f, 0.5f, 1.0f, 1.0f, 1.5f);
                batch.drawAxis(0.0f, 0.0f, 0.0f, 1.0f);
                batch.drawWireGrid(-1.0f, 4, 1.0f);
                frames.draw(server);
                CHECK(server.getStats().batchVertices >= 48);
            }

            server.instanceFree(wallInstance);
            server.instanceSetVisible(instance, true);
        }

        server.instanceFree(instance);
        server.instanceSetSurfaceMaterial(instance, 0, red);
        server.instanceSetMaterialOverride(instance, red);
        CHECK(server.instanceResolveMaterial(instance, 0) == fallback);
        frames.finish();
    }

    CHECK(RenderingServer::singleton() == nullptr);

    Device::destroy(device);

    if (failures) printf("%d check(s) failed\n", failures);
    else
        printf("all checks passed\n");
    return failures ? 1 : 0;
}
