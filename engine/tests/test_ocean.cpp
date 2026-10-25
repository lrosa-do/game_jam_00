#include "Check.h"
#include "LitTest.h"

#include "prisma3d/Device.h"
#include "prisma3d/main/Main.h"
#include "prisma3d/render/Batch.h"
#include "prisma3d/render/MaterialStore.h"
#include "prisma3d/render/MeshStore.h"
#include "prisma3d/render/OceanSimulation.h"
#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/render/TextureStore.h"
#include "prisma3d/scene/Camera3D.h"
#include "prisma3d/scene/Ocean3D.h"
#include "prisma3d/scene/PrimitiveMeshes.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/Viewport.h"
#include "prisma3d/scene/VisualInstance3D.h"
#include "prisma3d/scene/WorldEnvironment.h"

#include <ct/vector.hpp>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

using namespace prisma3d;

namespace
{

const int kN = static_cast<int>(OceanSimulation::kSize);
const int kInWidth = static_cast<int>(OceanSimulation::kInputWidth);

struct Reference
{
    ct::Vector<double> field0;
    ct::Vector<double> field1;
    double cosTable[512];
    double sinTable[512];

    void build(const OceanSimulationParams& params, float time)
    {
        ct::Vector<Math::Vec2> h0;
        ct::Vector<float> omega;
        OceanSimulation::initialSpectrum(params, h0, omega);
        field0.resize(static_cast<std::size_t>(kN) * kN * 2);
        field1.resize(static_cast<std::size_t>(kN) * kN * 2);
        for (int y = 0; y < kN; ++y)
            for (int x = 0; x < kN; ++x)
            {
                const std::size_t in = static_cast<std::size_t>(y) * kInWidth + x;
                const std::size_t mirrored = static_cast<std::size_t>(kN - y) * kInWidth + (kN - x);
                const double phase = static_cast<double>(omega[in]) * time * params.timeScale;
                const double s = sin(phase);
                const double c = cos(phase);
                const double htx =
                        (h0[in].x + h0[mirrored].x) * c - (h0[in].y + h0[mirrored].y) * s;
                const double hty =
                        (h0[in].x - h0[mirrored].x) * s + (h0[in].y - h0[mirrored].y) * c;
                double kx = x - kN * 0.5;
                double ky = y - kN * 0.5;
                const double sqr = kx * kx + ky * ky;
                const double inv = sqr > 1e-12 ? 1.0 / sqrt(sqr) : 0.0;
                kx *= inv;
                ky *= inv;
                const double dtxX = hty * kx;
                const double dtxY = -htx * kx;
                const std::size_t out = (static_cast<std::size_t>(y) * kN + x) * 2;
                field0[out] = htx - dtxY;
                field0[out + 1] = hty + dtxX;
                field1[out] = hty * ky;
                field1[out + 1] = -htx * ky;
            }
        for (int i = 0; i < 512; ++i)
        {
            cosTable[i] = cos(-6.283185307179586 * i / 512.0);
            sinTable[i] = sin(-6.283185307179586 * i / 512.0);
        }
    }

    void transform(const ct::Vector<double>& field, int x, int y, double& re, double& im) const
    {
        re = 0.0;
        im = 0.0;
        for (int j = 0; j < kN; ++j)
            for (int i = 0; i < kN; ++i)
            {
                const int index = (x * i + y * j) & 511;
                const double a = field[(static_cast<std::size_t>(j) * kN + i) * 2];
                const double b = field[(static_cast<std::size_t>(j) * kN + i) * 2 + 1];
                re += a * cosTable[index] - b * sinTable[index];
                im += a * sinTable[index] + b * cosTable[index];
            }
    }
};

void testSpectrum()
{
    OceanSimulationParams params;
    ct::Vector<Math::Vec2> a;
    ct::Vector<Math::Vec2> b;
    ct::Vector<float> omegaA;
    ct::Vector<float> omegaB;
    OceanSimulation::initialSpectrum(params, a, omegaA);
    OceanSimulation::initialSpectrum(params, b, omegaB);
    CHECK(a.size() == OceanSimulation::kInputCount && omegaA.size() == a.size());
    CHECK(memcmp(a.data(), b.data(), a.size() * sizeof(Math::Vec2)) == 0);

    params.seed = 7;
    OceanSimulation::initialSpectrum(params, b, omegaB);
    CHECK(memcmp(a.data(), b.data(), a.size() * sizeof(Math::Vec2)) != 0);
    CHECK(memcmp(omegaA.data(), omegaB.data(), omegaA.size() * sizeof(float)) == 0);

    const std::size_t centre = static_cast<std::size_t>(kN / 2) * kInWidth + kN / 2;
    CHECK(a[centre].x == 0.0f && a[centre].y == 0.0f);
    CHECK(omegaA[centre] == 0.0f);
    const std::size_t next = centre + 1;
    const float k = 2.0f * 3.14159265f / 50.0f;
    CHECK(fabsf(omegaA[next] - sqrtf(981.0f * k)) < 1e-3f);

    double along = 0.0;
    double against = 0.0;
    for (int i = 0; i <= kN; ++i)
        for (int j = 0; j <= kN; ++j)
        {
            const Math::Vec2& h = a[static_cast<std::size_t>(i) * kInWidth + j];
            CHECK(isfinite(h.x) && isfinite(h.y));
            const float kx = static_cast<float>(j - kN / 2);
            const float ky = static_cast<float>(i - kN / 2);
            const float projected = kx * 0.8f + ky * 0.6f;
            const double energy = static_cast<double>(h.x) * h.x + static_cast<double>(h.y) * h.y;
            if (projected > 0.0f) along += energy;
            else if (projected < 0.0f)
                against += energy;
        }
    CHECK(along > 0.0);
    CHECK(against < along * 0.2);
}

bool show(TextureId texture, BatchRenderer::BlendMode blend, ct::Vector<std::uint8_t>& pixels)
{
    RenderingServer& server = *Main::rendering();
    for (int frame = 0; frame < 2; ++frame)
    {
        BatchRenderer& batch = server.batch();
        batch.setSpace(BatchRenderer::Space::Screen);
        batch.setBlendMode(blend);
        batch.setColor(1.0f, 1.0f, 1.0f, 1.0f);
        batch.setTexture(Main::textures()->gpu(texture));
        batch.begin(BatchRenderer::ModeQuads);
        batch.setTexcoord(0.0f, 0.0f);
        batch.vertex2(0.0f, 0.0f);
        batch.setTexcoord(1.0f, 0.0f);
        batch.vertex2(static_cast<float>(kN), 0.0f);
        batch.setTexcoord(1.0f, 1.0f);
        batch.vertex2(static_cast<float>(kN), static_cast<float>(kN));
        batch.setTexcoord(0.0f, 1.0f);
        batch.vertex2(0.0f, static_cast<float>(kN));
        batch.end();
        batch.setBlendMode(BatchRenderer::BlendMode::Alpha);
        server.renderViewports();
        if (frame == 1)
        {
            pixels.resize(static_cast<std::size_t>(kN) * kN * 4);
            prisma::Rect rect;
            rect.width = kN;
            rect.height = kN;
            if (!Main::device()->driver()->readPixels(prisma::RenderTarget(), rect, pixels.data()))
                return false;
        }
        server.endFrame(true);
    }
    return true;
}

void testSimulation(bool vulkan)
{
    RenderingServer& server = *Main::rendering();
    CHECK(OceanSimulation::supported(Main::device()->driver()));
    if (!OceanSimulation::supported(Main::device()->driver())) return;

    OceanSimulationParams params;
    OceanSimulation ocean(*Main::textures());
    CHECK(!ocean.valid());
    CHECK(ocean.create(params));
    CHECK(ocean.valid());
    if (!ocean.valid()) return;
    CHECK(Main::textures()->valid(ocean.displacementMap()));
    CHECK(Main::textures()->valid(ocean.gradientMap()));

    const float time = 12.5f;
    server.openFrame();
    ocean.update(time);
    server.endFrame(true);
    CHECK(ocean.time() == time);

    ct::Vector<float> fields;
    CHECK(ocean.readFields(fields));
    CHECK(fields.size() == static_cast<std::size_t>(kN) * kN * 4);
    if (fields.size() != static_cast<std::size_t>(kN) * kN * 4) return;

    double sum = 0.0;
    double sumSquares = 0.0;
    double imaginary = 0.0;
    double peak = 0.0;
    const std::size_t second = static_cast<std::size_t>(kN) * kN * 2;
    for (std::size_t i = 0; i < static_cast<std::size_t>(kN) * kN; ++i)
    {
        const double height = fields[i * 2] * (((i % kN) + (i / kN)) & 1 ? -1.0 : 1.0);
        CHECK(isfinite(fields[i * 2]) && isfinite(fields[i * 2 + 1]));
        sum += height;
        sumSquares += height * height;
        imaginary += fabs(static_cast<double>(fields[second + i * 2 + 1]));
        if (fabs(height) > peak) peak = fabs(height);
    }
    const double count = static_cast<double>(kN) * kN;
    const double mean = sum / count;
    const double deviation = sqrt(sumSquares / count - mean * mean);
    printf("ocean %s: height mean %.5f deviation %.5f peak %.5f, residual imaginary %.6f\n",
            vulkan ? "vulkan" : "opengl", mean, deviation, peak, imaginary / count);
    CHECK(deviation > 1e-3);
    CHECK(fabs(mean) < deviation * 0.05);
    CHECK(peak < deviation * 12.0);
    CHECK(imaginary / count < deviation * 0.01);

    Reference reference;
    reference.build(params, time);
    const int samples[12][2] = { { 0, 0 }, { 1, 0 }, { 0, 1 }, { 5, 9 }, { 100, 37 }, { 255, 256 },
        { 256, 255 }, { 511, 511 }, { 300, 12 }, { 64, 448 }, { 77, 78 }, { 400, 401 } };
    double worst = 0.0;
    for (int s = 0; s < 12; ++s)
    {
        const int x = samples[s][0];
        const int y = samples[s][1];
        const std::size_t index = (static_cast<std::size_t>(y) * kN + x) * 2;
        double re = 0.0;
        double im = 0.0;
        reference.transform(reference.field0, x, y, re, im);
        worst = fmax(worst, fabs(re - fields[index]));
        worst = fmax(worst, fabs(im - fields[index + 1]));
        reference.transform(reference.field1, x, y, re, im);
        worst = fmax(worst, fabs(re - fields[second + index]));
    }
    printf("ocean %s: worst difference to the reference transform %.6f\n",
            vulkan ? "vulkan" : "opengl", worst);
    CHECK(worst < deviation * 0.02);

    {
        const float grid = static_cast<float>(kN) / params.patchLength;
        const auto at = [&](int x, int y, int c) -> double
        {
            x = (x + kN) % kN;
            y = (y + kN) % kN;
            const std::size_t i = static_cast<std::size_t>(y) * kN + x;
            const double sign = ((x + y) & 1) ? -1.0 : 1.0;
            if (c == 2) return fields[i * 2] * sign;
            if (c == 0) return fields[i * 2 + 1] * sign * params.choppyScale;
            return fields[second + i * 2] * sign * params.choppyScale;
        };
        const auto unit = [](double value) -> double
        { return value < 0.0 ? 0.0 : (value > 1.0 ? 1.0 : value); };

        ct::Vector<std::uint8_t> shown;
        ct::Vector<std::uint8_t> plain;
        ct::Vector<std::uint8_t> weighted;
        CHECK(show(ocean.displacementMap(), BatchRenderer::BlendMode::Alpha, shown));
        CHECK(show(ocean.gradientMap(), BatchRenderer::BlendMode::AddColors, plain));
        CHECK(show(ocean.gradientMap(), BatchRenderer::BlendMode::Alpha, weighted));

        int best[3] = { kN * kN * 3, kN * kN * 2, kN * kN * 2 };
        int folded = 0;
        for (int flip = 0; flip < 2; ++flip)
        {
            int errors[3] = { 0, 0, 0 };
            folded = 0;
            for (int y = 0; y < kN; ++y)
                for (int x = 0; x < kN; ++x)
                {
                    const int row = flip ? kN - 1 - y : y;
                    const std::size_t pixel = (static_cast<std::size_t>(row) * kN + x) * 4;
                    for (int c = 0; c < 3; ++c)
                        if (fabs(unit(at(x, y, c)) * 255.0 - shown[pixel + c]) > 2.5) ++errors[0];

                    const double gx = -(at(x + 1, y, 2) - at(x - 1, y, 2));
                    const double gy = -(at(x, y + 1, 2) - at(x, y - 1, 2));
                    const double scale = params.choppyScale * grid;
                    const double dxx = (at(x + 1, y, 0) - at(x - 1, y, 0)) * scale;
                    const double dxy = (at(x + 1, y, 1) - at(x - 1, y, 1)) * scale;
                    const double dyx = (at(x, y + 1, 0) - at(x, y - 1, 0)) * scale;
                    const double dyy = (at(x, y + 1, 1) - at(x, y - 1, 1)) * scale;
                    const double jacobian = (1.0 + dxx) * (1.0 + dyy) - dxy * dyx;
                    const double fold = unit(1.0 - jacobian);
                    if (fold > 0.05) ++folded;
                    const double gradient[2] = { unit(gx), unit(gy) };
                    for (int c = 0; c < 2; ++c)
                    {
                        if (fabs(gradient[c] * 255.0 - plain[pixel + c]) > 4.0) ++errors[1];
                        if (fabs(gradient[c] * fold * 255.0 - weighted[pixel + c]) > 6.0)
                            ++errors[2];
                    }
                }
            for (int k = 0; k < 3; ++k)
                if (errors[k] < best[k]) best[k] = errors[k];
        }
        printf("ocean %s: texels off: displacement %d, gradient %d, gradient times fold %d; "
               "folded texels %d\n",
                vulkan ? "vulkan" : "opengl", best[0], best[1], best[2], folded);
        CHECK(best[0] < kN * kN / 1000);
        CHECK(best[1] < kN * kN / 200);
        CHECK(best[2] < kN * kN / 200);
        CHECK(folded > 100);
    }

    server.openFrame();
    ocean.update(time);
    server.endFrame(true);
    ct::Vector<float> again;
    CHECK(ocean.readFields(again));
    double repeat = 0.0;
    for (std::size_t i = 0; i < fields.size() && i < again.size(); ++i)
        repeat = fmax(repeat, fabs(static_cast<double>(fields[i]) - again[i]));
    CHECK(repeat < deviation * 1e-3);

    server.openFrame();
    ocean.update(time + 2.0f);
    server.endFrame(true);
    CHECK(ocean.readFields(again));
    double moved = 0.0;
    for (std::size_t i = 0; i < fields.size() && i < again.size(); ++i)
        moved = fmax(moved, fabs(static_cast<double>(fields[i]) - again[i]));
    CHECK(moved > deviation * 0.1);

    ocean.release();
    CHECK(!ocean.valid());
    CHECK(!ocean.readFields(again));
}

struct Shot
{
    ct::Vector<std::uint8_t> pixels;

    const std::uint8_t* at(int x, int y) const
    {
        return &pixels[(static_cast<std::size_t>(y) * kN + x) * 4];
    }
};

bool shoot(SceneTree& tree, Shot& shot, RenderStats& stats, int frames = 3)
{
    RenderingServer& server = *Main::rendering();
    for (int i = 0; i < frames; ++i)
    {
        tree.process(1.0 / 60.0);
        server.renderViewports();
        if (i + 1 == frames)
        {
            shot.pixels.resize(static_cast<std::size_t>(kN) * kN * 4);
            prisma::Rect rect;
            rect.width = kN;
            rect.height = kN;
            if (!Main::device()->driver()->readPixels(prisma::RenderTarget(), rect,
                        shot.pixels.data()))
                return false;
            stats = server.getStats();
        }
        server.endFrame(true);
    }
    return true;
}

MaterialId emissive(const char* name, const Math::Vec4& colour)
{
    MaterialStore& materials = *Main::materials();
    const MaterialId material = materials.create(name);
    materials.setColor(material, MaterialParam::Albedo, Math::Vec4(0.0f, 0.0f, 0.0f, 1.0f));
    materials.setColor(material, MaterialParam::Emission, colour);
    materials.setFloat(material, MaterialParam::EmissionEnergy, 1.0f);
    materials.setFloat(material, MaterialParam::Roughness, 1.0f);
    materials.setFloat(material, MaterialParam::Specular, 0.0f);
    return material;
}

void testNode(SceneTree& tree, Camera3D* camera, bool vulkan)
{
    Node* root = tree.getRoot();
    WorldEnvironment* world = root->createChild<WorldEnvironment>();
    world->setEnvironment(whiteAmbient());
    camera->setPerspective(60.0f, 0.1f, 500.0f);
    camera->lookAtFromPosition(Math::Vec3(0.0f, 6.0f, 0.0f), Math::Vec3(0.0f, 0.0f, -12.0f));

    MeshStore& meshes = *Main::meshes();
    MeshInstance3D* floor = root->createChild<MeshInstance3D>();
    floor->setMesh(createPlaneMesh(meshes, 400.0f, 400.0f));
    floor->setMaterialOverride(emissive("sea floor", Math::Vec4(1.0f, 0.0f, 0.0f, 1.0f)));
    floor->setPosition(Math::Vec3(0.0f, -0.3f, 0.0f));

    RenderStats stats;
    Shot dry;
    CHECK(shoot(tree, dry, stats));
    CHECK(stats.screenCopies == 0);
    const int x = kN / 2;
    const int y = kN / 4;
    CHECK(dry.at(x, y)[0] > 200 && dry.at(x, y)[1] < 40);

    Ocean3D* ocean = root->createChild<Ocean3D>();
    CHECK(ocean->isSupported());
    CHECK(ocean->getMaterial().valid());
    Shot shallow;
    CHECK(shoot(tree, shallow, stats, 4));
    CHECK(ocean->getSimulation() && ocean->getSimulation()->valid());
    CHECK(stats.screenCopies == 1);

    floor->setPosition(Math::Vec3(0.0f, -40.0f, 0.0f));
    Shot deep;
    CHECK(shoot(tree, deep, stats));

    long redDry = 0;
    long redShallow = 0;
    long redDeep = 0;
    long differing = 0;
    for (int row = kN / 8; row < kN / 2; ++row)
        for (int column = 0; column < kN; ++column)
        {
            redDry += dry.at(column, row)[0];
            redShallow += shallow.at(column, row)[0];
            redDeep += deep.at(column, row)[0];
            if (abs(static_cast<int>(shallow.at(column, row)[0]) - deep.at(column, row)[0]) > 12)
                ++differing;
        }
    printf("ocean %s: red sum dry %ld, shallow %ld, deep %ld, texels differing %ld\n",
            vulkan ? "vulkan" : "opengl", redDry, redShallow, redDeep, differing);
    CHECK(redShallow < redDry);
    CHECK(redDeep < redShallow / 2);
    CHECK(differing > kN * kN / 16);

    Math::Vec3 flat = ocean->getDisplacedPosition(Math::Vec3(3.0f, 5.0f, -7.0f));
    CHECK(flat.x == 3.0f && flat.y == 0.0f && flat.z == -7.0f);
    Shot again;
    CHECK(shoot(tree, again, stats));
    double lowest = 1.0e9;
    double highest = -1.0e9;
    for (int i = 0; i < 64; ++i)
    {
        const Math::Vec3 moved = ocean->getDisplacedPosition(
                Math::Vec3(static_cast<float>(i) * 1.7f, 0.0f, static_cast<float>(i) * -2.3f));
        CHECK(isfinite(moved.x) && isfinite(moved.y) && isfinite(moved.z));
        lowest = fmin(lowest, moved.y);
        highest = fmax(highest, moved.y);
    }
    printf("ocean %s: sampled heights from %.3f to %.3f\n", vulkan ? "vulkan" : "opengl", lowest,
            highest);
    CHECK(highest - lowest > 0.5);
    CHECK(highest < 6.0 && lowest > -6.0);

    ocean->setPosition(Math::Vec3(0.0f, 2.0f, 0.0f));
    CHECK(shoot(tree, again, stats));
    CHECK(ocean->getWaterHeight() == 2.0f);
    CHECK(ocean->getDisplacedPosition(Math::Vec3(0.0f, 0.0f, 0.0f)).y > -4.0f);

    ocean->setPlanarReflection(true);
    CHECK(ocean->hasPlanarReflection());
    CHECK(shoot(tree, again, stats));
    CHECK(stats.reflectorsDrawn == 1);
    ocean->setPlanarReflection(false);
    CHECK(shoot(tree, again, stats));
    CHECK(stats.reflectorsDrawn == 0);

    ocean->setVisible(false);
    Shot hidden;
    CHECK(shoot(tree, hidden, stats));
    CHECK(stats.screenCopies == 0);
}

} // namespace

int main(int argc, char** argv)
{
    const bool vulkan = argc > 1 && strcmp(argv[1], "vulkan") == 0;

    testSpectrum();

    CHECK(Device::headlessSupported());
    if (!Device::headlessSupported()) return 1;

    DeviceDesc desc;
    desc.title = "ocean test";
    desc.driver = vulkan ? prisma::DriverType::Vulkan : prisma::DriverType::OpenGL;
    desc.width = kN;
    desc.height = kN;
    desc.headless = true;

    CHECK(Main::setup(desc));
    if (!Main::device()) return 1;
    SceneTree tree;
    CHECK(Main::start(&tree));
    if (Window* window = tree.getRoot() ? tree.getRoot()->castTo<Window>() : nullptr)
        window->setClearColor(Math::Vec4(0.0f, 0.0f, 0.0f, 1.0f));
    Camera3D* camera = tree.getRoot()->createChild<Camera3D>();
    camera->makeCurrent();
    tree.process(1.0 / 60.0);

    testSimulation(vulkan);
    testNode(tree, camera, vulkan);

    Main::device()->requestClose();
    tree.finalize();
    Main::cleanup();

    if (failures) printf("%d check(s) failed\n", failures);
    else
        printf("all checks passed\n");
    return failures ? 1 : 0;
}
