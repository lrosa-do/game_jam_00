#include "Check.h"
#include "LitTest.h"

#include "prisma3d/Device.h"
#include "prisma3d/core/Pixmap.h"
#include "prisma3d/render/AnimatedTexture.h"
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
const std::uint32_t kSize = 32;

struct Rgb
{
    int r;
    int g;
    int b;
};

float decodeValue(float value)
{
    return value <= 0.04045f ? value / 12.92f : powf((value + 0.055f) / 1.055f, 2.4f);
}

float decode(int byte)
{
    return decodeValue(static_cast<float>(byte) / 255.0f);
}

int encode(float linear)
{
    if (!(linear > 0.0f)) return 0;
    if (linear > 1.0f) linear = 1.0f;
    const float value =
            linear <= 0.0031308f ? linear * 12.92f : 1.055f * powf(linear, 1.0f / 2.4f) - 0.055f;
    return static_cast<int>(value * 255.0f + 0.5f);
}

float mixf(float a, float b, float t)
{
    return a * (1.0f - t) + b * t;
}

Rgb expectedDetail(MaterialBlend mode, const float base[3], const float detail[3], float alpha,
        float mask)
{
    int out[3];
    for (int i = 0; i < 3; ++i)
    {
        float applied = detail[i];
        if (mode == MaterialBlend::Add) applied = base[i] + detail[i];
        else if (mode == MaterialBlend::Sub)
            applied = base[i] - detail[i];
        else if (mode == MaterialBlend::Mul)
            applied = base[i] * detail[i];
        const float blended = mixf(base[i], applied, alpha);
        out[i] = encode(mixf(base[i], blended, mask));
    }
    return Rgb{ out[0], out[1], out[2] };
}

SurfaceData quadData(bool withUv2, bool mirroredUv2, bool withTangents)
{
    SurfaceData data;
    const float half = 1.0f;
    data.positions.push_back(Math::Vec3(-half, -half, 0.0f));
    data.positions.push_back(Math::Vec3(half, -half, 0.0f));
    data.positions.push_back(Math::Vec3(half, half, 0.0f));
    data.positions.push_back(Math::Vec3(-half, half, 0.0f));
    for (int i = 0; i < 4; ++i)
    {
        data.colors.push_back(Math::Vec4(1.0f, 1.0f, 1.0f, 1.0f));
        data.normals.push_back(Math::Vec3(0.0f, 0.0f, 1.0f));
        if (withTangents) data.tangents.push_back(Math::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
    }
    data.uvs.push_back(Math::Vec2(0.0f, 1.0f));
    data.uvs.push_back(Math::Vec2(1.0f, 1.0f));
    data.uvs.push_back(Math::Vec2(1.0f, 0.0f));
    data.uvs.push_back(Math::Vec2(0.0f, 0.0f));
    if (withUv2)
    {
        const float left = mirroredUv2 ? 1.0f : 0.0f;
        const float right = 1.0f - left;
        data.uvs2.push_back(Math::Vec2(left, 1.0f));
        data.uvs2.push_back(Math::Vec2(right, 1.0f));
        data.uvs2.push_back(Math::Vec2(right, 0.0f));
        data.uvs2.push_back(Math::Vec2(left, 0.0f));
    }
    const std::uint32_t indices[6] = { 0, 1, 2, 0, 2, 3 };
    for (std::uint32_t index: indices) data.indices.push_back(index);
    return data;
}

struct Frames
{
    RenderingServer* server = nullptr;
    bool open = false;

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

struct Rig
{
    prisma::Driver* driver = nullptr;
    RenderingServer* server = nullptr;
    TextureStore* textures = nullptr;
    MaterialStore* materials = nullptr;
    MeshStore* meshes = nullptr;
    ViewportId viewport;
    InstanceId instance;
    Frames frames;

    bool texel(int x, int y, Rgb& out)
    {
        const TextureId output = server->viewportGetTexture(viewport);
        prisma::RenderTarget source;
        source.texture = textures->gpu(output);
        prisma::Rect area;
        area.x = x;
        area.y = y;
        area.width = 1;
        area.height = 1;
        unsigned char pixel[4] = { 0, 0, 0, 0 };
        if (!driver->readPixels(source, area, pixel)) return false;
        out = Rgb{ pixel[0], pixel[1], pixel[2] };
        return true;
    }

    void render()
    {
        server->viewportSetUpdateMode(viewport, ViewportUpdateMode::Once);
        frames.draw(*server);
        frames.finish();
    }

    bool is(int x, int y, Rgb expected, int tolerance, const char* label)
    {
        Rgb got = { -1, -1, -1 };
        const bool read = texel(x, y, got);
        const bool same = read && abs(got.r - expected.r) <= tolerance &&
                          abs(got.g - expected.g) <= tolerance &&
                          abs(got.b - expected.b) <= tolerance;
        if (!same)
            printf("%s: texel (%d,%d) is %d,%d,%d, expected %d,%d,%d\n", label, x, y, got.r, got.g,
                    got.b, expected.r, expected.g, expected.b);
        return same;
    }

    bool centre(Rgb expected, int tolerance, const char* label)
    {
        render();
        return is(16, 16, expected, tolerance, label);
    }

    bool columns(const Rgb* expected, int tolerance, const char* label)
    {
        render();
        const int xs[4] = { 10, 14, 18, 22 };
        bool all = true;
        for (int i = 0; i < 4; ++i) all = is(xs[i], 16, expected[i], tolerance, label) && all;
        return all;
    }

    bool read(Rgb& out)
    {
        render();
        return texel(16, 16, out);
    }
};

TextureId solid(TextureStore& textures, int r, int g, int b, int a, bool srgb, const char* name)
{
    Pixmap pixels(4, 4, 4);
    for (std::uint32_t y = 0; y < 4; ++y)
        for (std::uint32_t x = 0; x < 4; ++x)
            pixels.set_pixel(x, y, static_cast<std::uint8_t>(r), static_cast<std::uint8_t>(g),
                    static_cast<std::uint8_t>(b), static_cast<std::uint8_t>(a));
    return textures.createFromPixmap(pixels, srgb, false, name);
}

TextureId halves(TextureStore& textures, const int* left, const int* right, bool srgb,
        const char* name)
{
    Pixmap pixels(8, 1, 4);
    for (std::uint32_t x = 0; x < 8; ++x)
    {
        const int* texel = x < 4 ? left : right;
        pixels.set_pixel(x, 0, static_cast<std::uint8_t>(texel[0]),
                static_cast<std::uint8_t>(texel[1]), static_cast<std::uint8_t>(texel[2]), 255);
    }
    return textures.createFromPixmap(pixels, srgb, false, name);
}

const int kBaseBytes[3] = { 128, 102, 77 };
const int kDetailBytes[3] = { 96, 64, 32 };

void blendModes(Rig& rig)
{
    MaterialStore& materials = *rig.materials;
    const float base[3] = { decode(kBaseBytes[0]), decode(kBaseBytes[1]), decode(kBaseBytes[2]) };
    const float detail[3] = { decode(kDetailBytes[0]), decode(kDetailBytes[1]),
        decode(kDetailBytes[2]) };

    const MaterialId material = materials.create("modes");
    materials.setShading(material, MaterialShading::Unshaded);
    materials.setColor(material, MaterialParam::Albedo,
            Math::Vec4(static_cast<float>(kBaseBytes[0]) / 255.0f,
                    static_cast<float>(kBaseBytes[1]) / 255.0f,
                    static_cast<float>(kBaseBytes[2]) / 255.0f, 1.0f));
    rig.server->instanceSetMaterialOverride(rig.instance, material);

    CHECK(rig.centre(Rgb{ encode(base[0]), encode(base[1]), encode(base[2]) }, 1,
            "base colour without detail"));

    const int maskBytes[3] = { 0, 128, 255 };
    const float maskValues[3] = { 0.0f, 128.0f / 255.0f, 1.0f };
    const int alphaBytes[2] = { 255, 128 };
    const float alphaValues[2] = { 1.0f, 128.0f / 255.0f };
    const MaterialBlend modes[4] = { MaterialBlend::Mix, MaterialBlend::Add, MaterialBlend::Sub,
        MaterialBlend::Mul };
    const char* const modeNames[4] = { "mix", "add", "subtract", "multiply" };

    CHECK(materials.setDetail(material, true));
    for (int alphaIndex = 0; alphaIndex < 2; ++alphaIndex)
    {
        const TextureId albedo = solid(*rig.textures, kDetailBytes[0], kDetailBytes[1],
                kDetailBytes[2], alphaBytes[alphaIndex], true, "detail albedo");
        CHECK(materials.setTexture(material, MaterialTexture::DetailAlbedo, albedo));
        for (int maskIndex = 0; maskIndex < 3; ++maskIndex)
        {
            const TextureId mask = solid(*rig.textures, maskBytes[maskIndex], maskBytes[maskIndex],
                    maskBytes[maskIndex], 255, false, "detail mask");
            CHECK(materials.setTexture(material, MaterialTexture::DetailMask, mask));
            for (int m = 0; m < 4; ++m)
            {
                CHECK(materials.setDetailBlendMode(material, modes[m]));
                const Rgb expected = expectedDetail(modes[m], base, detail, alphaValues[alphaIndex],
                        maskValues[maskIndex]);
                char label[96];
                snprintf(label, sizeof(label), "%s, detail alpha %d, mask %d", modeNames[m],
                        alphaBytes[alphaIndex], maskBytes[maskIndex]);
                CHECK(rig.centre(expected, 2, label));
            }
            CHECK(materials.setTexture(material, MaterialTexture::DetailMask, TextureId()));
            rig.textures->destroy(mask);
        }
        CHECK(materials.setTexture(material, MaterialTexture::DetailAlbedo, TextureId()));
        rig.textures->destroy(albedo);
    }

    CHECK(materials.setDetailBlendMode(material, MaterialBlend::Mix));
    CHECK(rig.centre(Rgb{ 255, 255, 255 }, 1, "detail with no textures is white in mix mode"));
    CHECK(materials.setDetailBlendMode(material, MaterialBlend::Add));
    CHECK(rig.centre(Rgb{ 255, 255, 255 }, 1, "detail with no textures is white in add mode"));
    CHECK(materials.setDetailBlendMode(material, MaterialBlend::Mul));
    CHECK(rig.centre(Rgb{ encode(base[0]), encode(base[1]), encode(base[2]) }, 1,
            "detail with no textures leaves the colour in multiply mode"));
    CHECK(materials.setDetail(material, false));
    CHECK(rig.centre(Rgb{ encode(base[0]), encode(base[1]), encode(base[2]) }, 1,
            "detail turned off"));
    rig.server->instanceSetMaterialOverride(rig.instance, MaterialId());
    materials.destroy(material);
}

void uvSets(Rig& rig)
{
    MaterialStore& materials = *rig.materials;
    const int leftBytes[3] = { 220, 40, 40 };
    const int rightBytes[3] = { 40, 40, 220 };
    const Rgb left = { leftBytes[0], leftBytes[1], leftBytes[2] };
    const Rgb right = { rightBytes[0], rightBytes[1], rightBytes[2] };
    const TextureId pattern = halves(*rig.textures, leftBytes, rightBytes, true, "pattern");

    const MaterialId material = materials.create("uv");
    materials.setShading(material, MaterialShading::Unshaded);
    materials.setDetail(material, true);
    materials.setTexture(material, MaterialTexture::DetailAlbedo, pattern);
    rig.server->instanceSetMaterialOverride(rig.instance, material);

    CHECK(materials.setDetailUv(material, MaterialDetailUv::Uv1));
    const Rgb straight[4] = { left, left, right, right };
    CHECK(rig.columns(straight, 2, "detail on UV1"));
    CHECK(materials.setDetailUv(material, MaterialDetailUv::Uv2));
    const Rgb mirrored[4] = { right, right, left, left };
    CHECK(rig.columns(mirrored, 2, "detail on UV2 with the mirrored second set"));

    CHECK(materials.setDetailUv(material, MaterialDetailUv::Uv1));
    CHECK(rig.columns(straight, 2, "back to UV1"));

    const int zero[3] = { 0, 0, 0 };
    const int full[3] = { 255, 255, 255 };
    const TextureId halfMask = halves(*rig.textures, zero, full, false, "half mask");
    CHECK(materials.setTexture(material, MaterialTexture::DetailMask, halfMask));
    const float baseLinear = 1.0f;
    const Rgb white = { encode(baseLinear), encode(baseLinear), encode(baseLinear) };
    const Rgb masked[4] = { white, white, right, right };
    CHECK(rig.columns(masked, 2, "mask on UV1 with detail on UV1"));
    CHECK(materials.setDetailUv(material, MaterialDetailUv::Uv2));
    const Rgb maskedMirrored[4] = { white, white, left, left };
    CHECK(rig.columns(maskedMirrored, 2, "the mask follows UV1 when the detail follows UV2"));
    CHECK(materials.setTexture(material, MaterialTexture::DetailMask, TextureId()));
    rig.textures->destroy(halfMask);

    CHECK(materials.setVec3(material, MaterialParam::Uv2Scale, Math::Vec3(2.0f, 1.0f, 1.0f)));
    const Rgb scaledOnMirror[4] = { right, left, right, left };
    CHECK(rig.columns(scaledOnMirror, 2, "UV2 scale 2 on the mirrored set"));
    CHECK(materials.setVec3(material, MaterialParam::Uv2Scale, Math::Vec3(1.0f, 1.0f, 1.0f)));
    CHECK(rig.columns(mirrored, 2, "UV2 scale back to 1"));

    materials.destroy(material);
    rig.server->instanceSetMaterialOverride(rig.instance, MaterialId());
    rig.textures->destroy(pattern);
}

void unmirroredUv2(Rig& rig, MeshId plain)
{
    MaterialStore& materials = *rig.materials;
    const int leftBytes[3] = { 220, 40, 40 };
    const int rightBytes[3] = { 40, 40, 220 };
    const Rgb left = { leftBytes[0], leftBytes[1], leftBytes[2] };
    const Rgb right = { rightBytes[0], rightBytes[1], rightBytes[2] };
    const TextureId pattern = halves(*rig.textures, leftBytes, rightBytes, true, "pattern");
    const MaterialId material = materials.create("uv scale");
    materials.setShading(material, MaterialShading::Unshaded);
    materials.setDetail(material, true);
    materials.setDetailUv(material, MaterialDetailUv::Uv2);
    materials.setTexture(material, MaterialTexture::DetailAlbedo, pattern);
    rig.server->instanceSetBase(rig.instance, plain);
    rig.server->instanceSetMaterialOverride(rig.instance, material);

    const Rgb identity[4] = { left, left, right, right };
    CHECK(rig.columns(identity, 2, "detail on an identity UV2"));

    CHECK(materials.setVec3(material, MaterialParam::Uv2Scale, Math::Vec3(2.0f, 1.0f, 1.0f)));
    const Rgb scaled[4] = { left, right, left, right };
    CHECK(rig.columns(scaled, 2, "UV2 scale 2"));

    CHECK(materials.setVec3(material, MaterialParam::Uv2Scale, Math::Vec3(1.0f, 1.0f, 1.0f)));
    CHECK(materials.setVec3(material, MaterialParam::Uv2Offset, Math::Vec3(0.5f, 0.0f, 0.0f)));
    const Rgb shifted[4] = { right, right, left, left };
    CHECK(rig.columns(shifted, 2, "UV2 offset 0.5"));

    CHECK(materials.setVec3(material, MaterialParam::Uv2Scale, Math::Vec3(2.0f, 1.0f, 1.0f)));
    const Rgb both[4] = { right, left, right, left };
    CHECK(rig.columns(both, 2, "UV2 scale 2 then offset 0.5"));

    CHECK(materials.setVec3(material, MaterialParam::Uv2Scale, Math::Vec3(1.0f, 1.0f, 1.0f)));
    CHECK(materials.setVec3(material, MaterialParam::Uv2Offset, Math::Vec3(0.0f, 0.0f, 0.0f)));
    CHECK(materials.setDetailUv(material, MaterialDetailUv::Uv1));
    CHECK(materials.setVec3(material, MaterialParam::Uv2Scale, Math::Vec3(3.0f, 3.0f, 1.0f)));
    CHECK(rig.columns(identity, 2, "UV2 scale does not touch a detail on UV1"));

    materials.destroy(material);
    rig.server->instanceSetMaterialOverride(rig.instance, MaterialId());
    rig.textures->destroy(pattern);
}

void missingUv2(Rig& rig, MeshId noSecondSet)
{
    MaterialStore& materials = *rig.materials;
    const int leftBytes[3] = { 220, 40, 40 };
    const int rightBytes[3] = { 40, 40, 220 };
    const TextureId pattern = halves(*rig.textures, leftBytes, rightBytes, true, "pattern");
    const MaterialId material = materials.create("no uv2");
    materials.setShading(material, MaterialShading::Unshaded);
    materials.setDetail(material, true);
    materials.setDetailUv(material, MaterialDetailUv::Uv2);
    materials.setTexture(material, MaterialTexture::DetailAlbedo, pattern);
    materials.setVec3(material, MaterialParam::Uv2Scale, Math::Vec3(7.0f, 7.0f, 1.0f));
    rig.server->instanceSetBase(rig.instance, noSecondSet);
    rig.server->instanceSetMaterialOverride(rig.instance, material);

    const float average = (decode(leftBytes[0]) + decode(rightBytes[0])) * 0.5f;
    const float middle = (decode(leftBytes[1]) + decode(rightBytes[1])) * 0.5f;
    const float blue = (decode(leftBytes[2]) + decode(rightBytes[2])) * 0.5f;
    const Rgb expected = { encode(average), encode(middle), encode(blue) };
    const Rgb all[4] = { expected, expected, expected, expected };
    CHECK(rig.columns(all, 3, "a mesh without a second UV set reads (0, 0)"));

    materials.destroy(material);
    rig.server->instanceSetMaterialOverride(rig.instance, MaterialId());
    rig.textures->destroy(pattern);
}

void pipelines(Rig& rig)
{
    MaterialStore& materials = *rig.materials;
    const MaterialId first = materials.create("first");
    materials.setShading(first, MaterialShading::Unshaded);
    rig.server->instanceSetMaterialOverride(rig.instance, first);
    rig.render();
    const std::uint32_t start = rig.server->scenePipelineCount();
    CHECK(start > 0);

    const MaterialId second = materials.create("second");
    materials.setShading(second, MaterialShading::Unshaded);
    materials.setColor(second, MaterialParam::Albedo, Math::Vec4(0.2f, 0.4f, 0.6f, 1.0f));
    materials.setVec3(second, MaterialParam::Uv2Scale, Math::Vec3(4.0f, 4.0f, 1.0f));
    materials.setDetailUv(second, MaterialDetailUv::Uv2);
    materials.setDetailBlendMode(second, MaterialBlend::Add);
    rig.server->instanceSetMaterialOverride(rig.instance, second);
    rig.render();
    CHECK(rig.server->scenePipelineCount() == start);

    const TextureId one = solid(*rig.textures, 90, 90, 90, 255, true, "detail");
    const TextureId two = solid(*rig.textures, 30, 30, 30, 255, true, "detail two");
    materials.setDetail(second, true);
    rig.render();
    const std::uint32_t withDetail = rig.server->scenePipelineCount();
    CHECK(withDetail == start + 1);

    materials.setTexture(second, MaterialTexture::DetailAlbedo, one);
    rig.render();
    CHECK(rig.server->scenePipelineCount() == withDetail);
    materials.setTexture(second, MaterialTexture::DetailMask, two);
    materials.setTexture(second, MaterialTexture::DetailAlbedo, two);
    materials.setDetailBlendMode(second, MaterialBlend::Mul);
    materials.setDetailUv(second, MaterialDetailUv::Uv1);
    materials.setVec3(second, MaterialParam::Uv2Offset, Math::Vec3(0.3f, 0.3f, 0.0f));
    rig.render();
    CHECK(rig.server->scenePipelineCount() == withDetail);

    rig.server->instanceSetMaterialOverride(rig.instance, first);
    rig.render();
    CHECK(rig.server->scenePipelineCount() == withDetail);

    materials.setDetail(second, false);
    rig.server->instanceSetMaterialOverride(rig.instance, second);
    rig.render();
    CHECK(rig.server->scenePipelineCount() == withDetail);

    rig.server->instanceSetMaterialOverride(rig.instance, MaterialId());
    materials.destroy(first);
    materials.destroy(second);
    rig.textures->destroy(one);
    rig.textures->destroy(two);
}

void animatedDetail(Rig& rig)
{
    MaterialStore& materials = *rig.materials;
    const Rgb colours[3] = { { 255, 0, 0 }, { 0, 255, 0 }, { 0, 0, 255 } };
    TextureId frames[3];
    for (int i = 0; i < 3; ++i)
        frames[i] =
                solid(*rig.textures, colours[i].r, colours[i].g, colours[i].b, 255, true, "frame");

    AnimatedTexture animated(rig.textures);
    animated.setAutoUpdate(false);
    CHECK(animated.setFrames(3));
    for (int i = 0; i < 3; ++i)
    {
        CHECK(animated.setFrameTexture(i, frames[i]));
        CHECK(animated.setFrameDuration(i, 0.5f));
    }
    const MaterialId material = materials.create("animated detail");
    materials.setShading(material, MaterialShading::Unshaded);
    materials.setColor(material, MaterialParam::Albedo, Math::Vec4(0.5f, 0.5f, 0.5f, 1.0f));
    materials.setDetail(material, true);
    materials.setTexture(material, MaterialTexture::DetailAlbedo, animated.getTexture());
    rig.server->instanceSetMaterialOverride(rig.instance, material);

    animated.update(0.0f);
    CHECK(rig.centre(colours[0], 2, "animated detail, frame 0"));
    const std::uint32_t pipelineCount = rig.server->scenePipelineCount();
    const std::uint32_t version = materials.get(material)->version;
    animated.update(0.75f);
    CHECK(rig.centre(colours[1], 2, "animated detail, frame 1"));
    animated.update(0.5f);
    CHECK(rig.centre(colours[2], 2, "animated detail, frame 2"));
    animated.update(0.5f);
    CHECK(rig.centre(colours[0], 2, "animated detail, wrapped"));
    CHECK(rig.server->scenePipelineCount() == pipelineCount);
    CHECK(materials.get(material)->version == version);

    CHECK(materials.setDetailBlendMode(material, MaterialBlend::Add));
    const float base = decodeValue(0.5f);
    CHECK(rig.centre(Rgb{ encode(base + 1.0f), encode(base), encode(base) }, 2,
            "animated detail added to the base"));
    animated.update(0.75f);
    CHECK(rig.centre(Rgb{ encode(base), encode(base + 1.0f), encode(base) }, 2,
            "animated detail added, frame 1"));
    CHECK(rig.server->scenePipelineCount() == pipelineCount);

    rig.server->instanceSetMaterialOverride(rig.instance, MaterialId());
    materials.destroy(material);
    for (int i = 0; i < 3; ++i) rig.textures->destroy(frames[i]);
}

void litNormals(Rig& rig, MeshId tangent)
{
    MaterialStore& materials = *rig.materials;
    const ScenarioId scenario = rig.server->scenarioCreate();
    EnvironmentSettings environment;
    environment.ambientSource = AmbientSource::Color;
    environment.ambientColor = Math::Vec3(0.0f, 0.0f, 0.0f);
    environment.reflectionSource = ReflectionSource::Disabled;
    rig.server->scenarioSetEnvironment(scenario, environment);
    const LightId sun = rig.server->lightCreate(LightType::Directional);
    rig.server->lightSetScenario(sun, scenario);
    rig.server->lightSetTransform(sun, Math::Mat4::RotationY(0.78539816f));

    const CameraId camera = rig.server->cameraCreate();
    rig.server->cameraSetOrthogonal(camera, 4.0f, 0.1f, 20.0f);
    rig.server->cameraSetTransform(camera, Math::Mat4::Translation(Math::Vec3(0.0f, 0.0f, 3.0f)));
    const ViewportId viewport = rig.server->viewportCreate();
    rig.server->viewportSetSize(viewport, kSize, kSize);
    rig.server->viewportAttachCamera(viewport, camera);
    rig.server->viewportSetScenario(viewport, scenario);
    const InstanceId panel = rig.server->instanceCreate(tangent, scenario);

    Rig lit = rig;
    lit.viewport = viewport;
    lit.instance = panel;

    const TextureId towards = solid(*rig.textures, 191, 128, 255, 255, false, "normal towards");
    const TextureId away = solid(*rig.textures, 64, 128, 255, 255, false, "normal away");
    const TextureId white = solid(*rig.textures, 255, 255, 255, 255, true, "white");
    const TextureId clear = solid(*rig.textures, 255, 255, 255, 0, true, "clear");
    const int maskBytes[3] = { 0, 128, 255 };
    TextureId masks[3];
    for (int i = 0; i < 3; ++i)
        masks[i] = solid(*rig.textures, maskBytes[i], maskBytes[i], maskBytes[i], 255, false, "m");

    const MaterialId flat = materials.create("flat");
    materials.setColor(flat, MaterialParam::Albedo, Math::Vec4(0.8f, 0.6f, 0.4f, 1.0f));
    const MaterialId main = materials.create("main normal");
    materials.setColor(main, MaterialParam::Albedo, Math::Vec4(0.8f, 0.6f, 0.4f, 1.0f));
    const MaterialId detail = materials.create("detail normal");
    materials.setColor(detail, MaterialParam::Albedo, Math::Vec4(0.8f, 0.6f, 0.4f, 1.0f));
    materials.setDetail(detail, true);
    materials.setDetailBlendMode(detail, MaterialBlend::Mul);
    materials.setTexture(detail, MaterialTexture::DetailAlbedo, white);

    lit.instance = panel;
    rig.server->instanceSetMaterialOverride(panel, flat);
    Rgb flatPixel = {};
    CHECK(lit.read(flatPixel));
    CHECK(flatPixel.r > 20);

    materials.setTexture(main, MaterialTexture::Normal, towards);
    rig.server->instanceSetMaterialOverride(panel, main);
    Rgb mainTowards = {};
    CHECK(lit.read(mainTowards));
    materials.setTexture(main, MaterialTexture::Normal, away);
    Rgb mainAway = {};
    CHECK(lit.read(mainAway));
    printf("lit flat %d,%d,%d, normal toward the light %d,%d,%d, away %d,%d,%d\n", flatPixel.r,
            flatPixel.g, flatPixel.b, mainTowards.r, mainTowards.g, mainTowards.b, mainAway.r,
            mainAway.g, mainAway.b);
    CHECK(mainTowards.r > flatPixel.r + 15);
    CHECK(mainAway.r < flatPixel.r - 15);

    rig.server->instanceSetMaterialOverride(panel, detail);
    materials.setTexture(detail, MaterialTexture::DetailNormal, towards);
    materials.setTexture(detail, MaterialTexture::DetailMask, masks[2]);
    CHECK(lit.centre(mainTowards, 2, "detail normal at mask 1 equals the main normal"));
    materials.setTexture(detail, MaterialTexture::DetailNormal, away);
    CHECK(lit.centre(mainAway, 2, "detail normal away at mask 1 equals the main normal"));

    materials.setTexture(detail, MaterialTexture::DetailNormal, towards);
    materials.setTexture(detail, MaterialTexture::DetailMask, masks[0]);
    CHECK(lit.centre(flatPixel, 2, "detail normal at mask 0 changes nothing"));
    materials.setTexture(detail, MaterialTexture::DetailMask, masks[1]);
    Rgb halfway = {};
    CHECK(lit.read(halfway));
    CHECK(halfway.r > flatPixel.r + 3 && halfway.r < mainTowards.r - 3);
    materials.setTexture(detail, MaterialTexture::DetailMask, masks[2]);
    materials.setTexture(detail, MaterialTexture::DetailAlbedo, clear);
    CHECK(lit.centre(flatPixel, 2, "detail alpha 0 hides the detail normal and albedo"));
    materials.setTexture(detail, MaterialTexture::DetailAlbedo, white);

    materials.setTexture(main, MaterialTexture::Normal, towards);
    materials.setTexture(detail, MaterialTexture::Normal, towards);
    materials.setTexture(detail, MaterialTexture::DetailNormal, away);
    materials.setTexture(detail, MaterialTexture::DetailMask, masks[0]);
    CHECK(lit.centre(mainTowards, 2, "a main normal map with the detail normal masked out"));
    materials.setTexture(detail, MaterialTexture::DetailMask, masks[2]);
    CHECK(lit.centre(mainAway, 2, "a main normal map replaced by the detail normal at mask 1"));

    materials.destroy(flat);
    materials.destroy(main);
    materials.destroy(detail);
    rig.server->instanceFree(panel);
    rig.server->viewportFree(viewport);
    rig.server->cameraFree(camera);
    rig.server->lightFree(sun);
    rig.server->scenarioFree(scenario);
    rig.textures->destroy(towards);
    rig.textures->destroy(away);
    rig.textures->destroy(white);
    rig.textures->destroy(clear);
    for (int i = 0; i < 3; ++i) rig.textures->destroy(masks[i]);
}

void litAlbedo(Rig& rig)
{
    MaterialStore& materials = *rig.materials;
    const ScenarioId scenario = rig.server->scenarioCreate();
    rig.server->scenarioSetEnvironment(scenario, whiteAmbient());
    const CameraId camera = rig.server->cameraCreate();
    rig.server->cameraSetOrthogonal(camera, 4.0f, 0.1f, 20.0f);
    rig.server->cameraSetTransform(camera, Math::Mat4::Translation(Math::Vec3(0.0f, 0.0f, 3.0f)));
    const ViewportId viewport = rig.server->viewportCreate();
    rig.server->viewportSetSize(viewport, kSize, kSize);
    rig.server->viewportAttachCamera(viewport, camera);
    rig.server->viewportSetScenario(viewport, scenario);
    const MeshId quad = rig.meshes->create("lit quad");
    CHECK(rig.meshes->addSurface(quad, quadData(false, false, false)));
    const InstanceId panel = rig.server->instanceCreate(quad, scenario);

    Rig lit = rig;
    lit.viewport = viewport;
    lit.instance = panel;

    const float base[3] = { decode(kBaseBytes[0]), decode(kBaseBytes[1]), decode(kBaseBytes[2]) };
    const float detail[3] = { decode(kDetailBytes[0]), decode(kDetailBytes[1]),
        decode(kDetailBytes[2]) };
    const MaterialId material = materials.create("lit detail");
    materials.setColor(material, MaterialParam::Albedo,
            Math::Vec4(static_cast<float>(kBaseBytes[0]) / 255.0f,
                    static_cast<float>(kBaseBytes[1]) / 255.0f,
                    static_cast<float>(kBaseBytes[2]) / 255.0f, 1.0f));
    rig.server->instanceSetMaterialOverride(panel, material);
    const TextureId albedo = solid(*rig.textures, kDetailBytes[0], kDetailBytes[1], kDetailBytes[2],
            255, true, "detail albedo");
    const MaterialBlend modes[4] = { MaterialBlend::Mix, MaterialBlend::Add, MaterialBlend::Sub,
        MaterialBlend::Mul };
    CHECK(lit.centre(Rgb{ encode(base[0]), encode(base[1]), encode(base[2]) }, 4,
            "lit material without detail"));
    materials.setDetail(material, true);
    materials.setTexture(material, MaterialTexture::DetailAlbedo, albedo);
    const int maskByte = 255;
    const TextureId mask = solid(*rig.textures, maskByte, maskByte, maskByte, 255, false, "m");
    materials.setTexture(material, MaterialTexture::DetailMask, mask);
    for (int m = 0; m < 4; ++m)
    {
        materials.setDetailBlendMode(material, modes[m]);
        const Rgb expected = expectedDetail(modes[m], base, detail, 1.0f, 1.0f);
        char label[64];
        snprintf(label, sizeof(label), "lit material with detail, mode %d", m);
        CHECK(lit.centre(expected, 4, label));
    }
    CHECK(materials.setDetail(material, false));
    CHECK(lit.centre(Rgb{ encode(base[0]), encode(base[1]), encode(base[2]) }, 4,
            "lit material with detail turned off"));
    materials.destroy(material);
    rig.server->instanceFree(panel);
    rig.server->viewportFree(viewport);
    rig.server->cameraFree(camera);
    rig.server->scenarioFree(scenario);
    rig.textures->destroy(albedo);
    rig.textures->destroy(mask);
}

void run(prisma::Driver* driver)
{
    MeshStore meshes(driver);
    TextureStore textures(driver);
    MaterialStore materials(textures);
    RenderingServer server(driver, meshes, textures, materials);

    const MeshId full = meshes.create("quad with two sets");
    CHECK(meshes.addSurface(full, quadData(true, true, false)));
    const MeshId identity = meshes.create("quad with identity second set");
    {
        SurfaceData data = quadData(true, false, false);
        CHECK(meshes.addSurface(identity, data));
    }
    const MeshId plain = meshes.create("quad with one set");
    CHECK(meshes.addSurface(plain, quadData(false, false, false)));
    const MeshId tangent = meshes.create("quad with tangents");
    CHECK(meshes.addSurface(tangent, quadData(false, false, true)));

    const ScenarioId scenario = server.scenarioCreate();
    const InstanceId instance = server.instanceCreate(full, scenario);
    const CameraId camera = server.cameraCreate();
    server.cameraSetOrthogonal(camera, 4.0f, 0.1f, 20.0f);
    server.cameraSetTransform(camera, Math::Mat4::Translation(Math::Vec3(0.0f, 0.0f, 3.0f)));

    Rig rig;
    rig.driver = driver;
    rig.server = &server;
    rig.textures = &textures;
    rig.materials = &materials;
    rig.meshes = &meshes;
    rig.viewport = server.viewportCreate();
    rig.instance = instance;
    server.viewportSetSize(rig.viewport, kSize, kSize);
    server.viewportAttachCamera(rig.viewport, camera);
    server.viewportSetScenario(rig.viewport, scenario);

    pipelines(rig);
    blendModes(rig);
    uvSets(rig);
    server.instanceSetBase(instance, identity);
    unmirroredUv2(rig, identity);
    missingUv2(rig, plain);
    server.instanceSetBase(instance, full);
    animatedDetail(rig);
    litAlbedo(rig);
    litNormals(rig, tangent);

    server.viewportFree(rig.viewport);
    server.cameraFree(camera);
    server.instanceFree(instance);
    server.scenarioFree(scenario);
}

} // namespace

int main(int argc, char** argv)
{
    const bool vulkan = argc > 1 && strcmp(argv[1], "vulkan") == 0;

    CHECK(Device::headlessSupported());
    if (!Device::headlessSupported()) return 1;

    DeviceDesc deviceDesc;
    deviceDesc.title = "detail render test";
    deviceDesc.driver = vulkan ? prisma::DriverType::Vulkan : prisma::DriverType::OpenGL;
    deviceDesc.width = kWidth;
    deviceDesc.height = kHeight;
    deviceDesc.headless = true;

    Device* device = Device::create(deviceDesc);
    CHECK(device != nullptr);
    if (!device) return 1;

    run(device->driver());

    Device::destroy(device);
    if (failures == 0) printf("detail render test: all passed\n");
    return failures == 0 ? 0 : 1;
}
