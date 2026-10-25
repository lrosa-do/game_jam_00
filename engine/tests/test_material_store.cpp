#include "Check.h"

#include "prisma3d/render/MaterialStore.h"
#include "prisma3d/render/TextureStore.h"

#include <math.h>
#include <string.h>

using namespace prisma3d;

namespace
{

bool approx(float a, float b)
{
    return fabsf(a - b) < 1e-6f;
}

void testTable()
{
    CHECK(MaterialStore::parameterCount() == 16);
    CHECK(strcmp(MaterialStore::parameter(MaterialParam::Refraction).name, "refraction") == 0);
    CHECK(strcmp(MaterialStore::parameter(0u).name, "albedo") == 0);
    CHECK(strcmp(MaterialStore::parameter(MaterialParam::Uv1Offset).name, "uv1_offset") == 0);
    CHECK(strcmp(MaterialStore::parameter(MaterialParam::Uv2Scale).name, "uv2_scale") == 0);
    CHECK(strcmp(MaterialStore::parameter(MaterialParam::Uv2Offset).name, "uv2_offset") == 0);
    CHECK(MaterialStore::parameter(MaterialParam::Uv2Scale).type == MaterialParamType::Vec3);
    CHECK(MaterialStore::parameter(MaterialParam::EmissionEnergy).maximum == 16.0f);
    CHECK(MaterialStore::parameter(MaterialParam::NormalScale).minimum == -16.0f);
    CHECK(MaterialStore::parameter(MaterialParam::Uv1Scale).type == MaterialParamType::Vec3);
    CHECK(MaterialStore::parameter(MaterialParam::Emission).type == MaterialParamType::Color);
    CHECK(MaterialStore::parameter(MaterialParam::Roughness).type == MaterialParamType::Float);

    for (std::uint32_t i = 0; i < MaterialStore::parameterCount(); ++i)
    {
        MaterialParam found = MaterialParam::Count;
        CHECK(MaterialStore::findParameter(MaterialStore::parameter(i).name, found));
        CHECK(static_cast<std::uint32_t>(found) == i);
    }
    MaterialParam unknown = MaterialParam::Metallic;
    CHECK(!MaterialStore::findParameter("shininess", unknown));
    CHECK(!MaterialStore::findParameter("", unknown));
    CHECK(!MaterialStore::findParameter(nullptr, unknown));
    CHECK(unknown == MaterialParam::Metallic);
}

void testLifetime(MaterialStore& store)
{
    CHECK(MaterialStore::singleton() == &store);
    CHECK(store.count() == 0);
    CHECK(!store.valid(MaterialId()));
    CHECK(store.get(MaterialId()) == nullptr);

    const MaterialId first = store.create("paint");
    CHECK(first.valid() && store.valid(first));
    CHECK(store.count() == 1);
    const MaterialInfo* info = store.get(first);
    CHECK(info != nullptr);
    if (!info) return;
    CHECK(strcmp(info->name.c_str(), "paint") == 0);
    CHECK(info->path.empty());
    CHECK(info->albedo.x == 1.0f && info->albedo.y == 1.0f && info->albedo.z == 1.0f &&
            info->albedo.w == 1.0f);
    CHECK(info->metallic == 0.0f && info->specular == 0.5f && info->roughness == 1.0f);
    CHECK(info->emissionEnergy == 1.0f && info->normalScale == 1.0f);
    CHECK(info->clearcoatRoughness == 0.5f && info->alphaScissorThreshold == 0.5f);
    CHECK(info->uv1Scale.x == 1.0f && info->uv1Offset.x == 0.0f);
    CHECK(info->transparency == MaterialTransparency::Opaque);
    CHECK(info->cull == MaterialCull::Back);
    CHECK(info->shading == MaterialShading::Lit);
    CHECK(info->version == 0);

    const MaterialId unnamed = store.create();
    CHECK(unnamed.valid() && unnamed != first);
    CHECK(store.get(unnamed)->name.empty());
    CHECK(store.count() == 2);

    store.setName(first, "metal");
    CHECK(strcmp(store.get(first)->name.c_str(), "metal") == 0);
    store.setPath(first, ct::String("materials/metal.mat"));
    CHECK(strcmp(store.get(first)->path.c_str(), "materials/metal.mat") == 0);

    store.destroy(unnamed);
    CHECK(!store.valid(unnamed));
    CHECK(store.get(unnamed) == nullptr);
    CHECK(store.count() == 1);
    CHECK(store.valid(first));
    store.destroy(unnamed);
    CHECK(store.count() == 1);

    const MaterialId reused = store.create("again");
    CHECK(reused != unnamed);
    CHECK(!store.valid(unnamed));
    store.destroy(reused);

    const MaterialId fallback = store.defaultMaterial();
    CHECK(fallback.valid() && store.valid(fallback));
    CHECK(strcmp(store.get(fallback)->name.c_str(), "default") == 0);
    CHECK(store.count() == 2);
    CHECK(store.defaultMaterial() == fallback);
    CHECK(store.count() == 2);
    store.destroy(fallback);
    CHECK(store.valid(fallback));
    CHECK(store.defaultMaterial() == fallback);
    CHECK(store.count() == 2);

    store.clear();
    CHECK(store.count() == 0);
    CHECK(!store.valid(first));
    CHECK(!store.valid(fallback));
    const MaterialId recreated = store.defaultMaterial();
    CHECK(store.valid(recreated));
    CHECK(store.count() == 1);
    store.clear();
}

void testTypedParameters(MaterialStore& store)
{
    const MaterialId id = store.create("typed");
    std::uint32_t version = store.get(id)->version;

    CHECK(store.setFloat(id, MaterialParam::Metallic, 0.25f));
    float value = 0.0f;
    CHECK(store.getFloat(id, MaterialParam::Metallic, value) && approx(value, 0.25f));
    CHECK(store.get(id)->version == ++version);

    CHECK(store.setFloat(id, MaterialParam::Metallic, 7.0f));
    CHECK(store.getFloat(id, MaterialParam::Metallic, value) && value == 1.0f);
    CHECK(store.setFloat(id, MaterialParam::Metallic, -3.0f));
    CHECK(store.getFloat(id, MaterialParam::Metallic, value) && value == 0.0f);
    version += 2;
    CHECK(store.get(id)->version == version);

    CHECK(store.setFloat(id, MaterialParam::EmissionEnergy, 100.0f));
    CHECK(store.getFloat(id, MaterialParam::EmissionEnergy, value) && value == 16.0f);
    CHECK(store.setFloat(id, MaterialParam::NormalScale, -100.0f));
    CHECK(store.getFloat(id, MaterialParam::NormalScale, value) && value == -16.0f);
    CHECK(store.setFloat(id, MaterialParam::Specular, 0.9f));
    CHECK(store.setFloat(id, MaterialParam::Roughness, 0.3f));
    CHECK(store.setFloat(id, MaterialParam::AoLightAffect, 0.4f));
    CHECK(store.setFloat(id, MaterialParam::Clearcoat, 0.8f));
    CHECK(store.setFloat(id, MaterialParam::ClearcoatRoughness, 0.1f));
    CHECK(store.setFloat(id, MaterialParam::AlphaScissorThreshold, 0.7f));
    const MaterialInfo* info = store.get(id);
    CHECK(approx(info->specular, 0.9f) && approx(info->roughness, 0.3f));
    CHECK(approx(info->aoLightAffect, 0.4f) && approx(info->clearcoat, 0.8f));
    CHECK(approx(info->clearcoatRoughness, 0.1f) && approx(info->alphaScissorThreshold, 0.7f));
    version += 8;

    CHECK(store.setFloat(id, MaterialParam::Metallic, NAN));
    CHECK(store.getFloat(id, MaterialParam::Metallic, value) && value == 0.0f);
    version += 1;

    CHECK(store.get(id)->version == version);
    CHECK(!store.setFloat(id, MaterialParam::Albedo, 0.5f));
    CHECK(!store.setFloat(id, MaterialParam::Uv1Scale, 0.5f));
    CHECK(!store.setFloat(id, MaterialParam::Emission, 0.5f));
    CHECK(!store.setFloat(id, MaterialParam::Count, 0.5f));
    CHECK(!store.getFloat(id, MaterialParam::Albedo, value));
    CHECK(!store.getFloat(id, MaterialParam::Uv1Offset, value));
    CHECK(store.get(id)->version == version);

    CHECK(store.setColor(id, MaterialParam::Albedo, Math::Vec4(0.1f, 2.0f, -1.0f, 0.5f)));
    Math::Vec4 color;
    CHECK(store.getColor(id, MaterialParam::Albedo, color));
    CHECK(approx(color.x, 0.1f) && color.y == 1.0f && color.z == 0.0f && approx(color.w, 0.5f));
    ++version;

    CHECK(store.setColor(id, MaterialParam::Emission, Math::Vec4(0.2f, 0.4f, 3.0f, 0.1f)));
    CHECK(store.getColor(id, MaterialParam::Emission, color));
    CHECK(approx(color.x, 0.2f) && approx(color.y, 0.4f) && color.z == 1.0f && color.w == 1.0f);
    CHECK(approx(store.get(id)->emission.x, 0.2f) && store.get(id)->emission.z == 1.0f);
    ++version;
    CHECK(store.get(id)->version == version);

    CHECK(!store.setColor(id, MaterialParam::Metallic, Math::Vec4(0.0f, 0.0f, 0.0f, 0.0f)));
    CHECK(!store.setColor(id, MaterialParam::Uv1Scale, Math::Vec4(0.0f, 0.0f, 0.0f, 0.0f)));
    CHECK(!store.getColor(id, MaterialParam::Roughness, color));
    CHECK(!store.getColor(id, MaterialParam::Uv1Scale, color));

    CHECK(store.setVec3(id, MaterialParam::Uv1Scale, Math::Vec3(2.0f, -500.0f, 500.0f)));
    Math::Vec3 vector;
    CHECK(store.getVec3(id, MaterialParam::Uv1Scale, vector));
    CHECK(vector.x == 2.0f && vector.y == -100.0f && vector.z == 100.0f);
    CHECK(store.setVec3(id, MaterialParam::Uv1Offset, Math::Vec3(0.5f, 0.25f, 0.0f)));
    CHECK(store.getVec3(id, MaterialParam::Uv1Offset, vector));
    CHECK(vector.x == 0.5f && vector.y == 0.25f && vector.z == 0.0f);
    version += 2;
    CHECK(store.get(id)->version == version);
    CHECK(!store.setVec3(id, MaterialParam::Albedo, Math::Vec3(0.0f, 0.0f, 0.0f)));
    CHECK(!store.setVec3(id, MaterialParam::Emission, Math::Vec3(0.0f, 0.0f, 0.0f)));
    CHECK(!store.setVec3(id, MaterialParam::Roughness, Math::Vec3(0.0f, 0.0f, 0.0f)));
    CHECK(!store.getVec3(id, MaterialParam::Metallic, vector));
    CHECK(store.get(id)->version == version);

    CHECK(!store.setFloat(MaterialId(), MaterialParam::Metallic, 0.5f));
    CHECK(!store.setColor(MaterialId(), MaterialParam::Albedo, Math::Vec4(1.0f, 1.0f, 1.0f, 1.0f)));
    CHECK(!store.setVec3(MaterialId(), MaterialParam::Uv1Scale, Math::Vec3(1.0f, 1.0f, 1.0f)));
    CHECK(!store.getFloat(MaterialId(), MaterialParam::Metallic, value));
    CHECK(!store.getColor(MaterialId(), MaterialParam::Albedo, color));
    CHECK(!store.getVec3(MaterialId(), MaterialParam::Uv1Scale, vector));
    store.clear();
}

void testNamedParameters(MaterialStore& store)
{
    const MaterialId id = store.create("named");
    std::uint32_t version = store.get(id)->version;

    for (std::uint32_t i = 0; i < MaterialStore::parameterCount(); ++i)
    {
        const MaterialParamInfo& description = MaterialStore::parameter(i);
        const bool emission = i == static_cast<std::uint32_t>(MaterialParam::Emission);
        const std::uint32_t count = description.type == MaterialParamType::Float  ? 1u
                                    : description.type == MaterialParamType::Vec3 ? 3u
                                    : emission                                    ? 3u
                                                                                  : 4u;
        float written[4];
        float low[4];
        float high[4];
        float middle = description.minimum + (description.maximum - description.minimum) * 0.25f;
        for (std::uint32_t k = 0; k < 4; ++k)
        {
            written[k] = middle;
            low[k] = description.minimum - 1000.0f;
            high[k] = description.maximum + 1000.0f;
        }
        float read[4] = { -77.0f, -77.0f, -77.0f, -77.0f };

        CHECK(store.setParameter(id, description.name, written, count));
        ++version;
        CHECK(store.get(id)->version == version);
        CHECK(store.getParameter(id, description.name, read, count));
        for (std::uint32_t k = 0; k < count; ++k) CHECK(approx(read[k], middle));
        for (std::uint32_t k = count; k < 4; ++k) CHECK(read[k] == -77.0f);

        CHECK(store.setParameter(id, description.name, low, count));
        CHECK(store.getParameter(id, description.name, read, count));
        for (std::uint32_t k = 0; k < count; ++k) CHECK(read[k] == description.minimum);
        CHECK(store.setParameter(id, description.name, high, count));
        CHECK(store.getParameter(id, description.name, read, count));
        for (std::uint32_t k = 0; k < count; ++k) CHECK(read[k] == description.maximum);
        version += 2;

        if (!emission)
        {
            CHECK(!store.setParameter(id, description.name, written, count + 1));
            CHECK(!store.getParameter(id, description.name, read, count + 1));
        }
        CHECK(!store.setParameter(id, description.name, written, 0));
        CHECK(!store.getParameter(id, description.name, read, 0));
        CHECK(store.get(id)->version == version);
    }

    MaterialParam param;
    CHECK(MaterialStore::findParameter("albedo", param));
    float color[4] = { 0.1f, 0.2f, 0.3f, 0.4f };
    CHECK(store.setParameter(id, "albedo", color, 4));
    ++version;
    CHECK(approx(store.get(id)->albedo.x, 0.1f) && approx(store.get(id)->albedo.w, 0.4f));
    CHECK(!store.setParameter(id, "albedo", color, 3));
    CHECK(store.get(id)->version == version);

    float read[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
    CHECK(store.setParameter(id, "emission", color, 3));
    ++version;
    CHECK(approx(store.get(id)->emission.x, 0.1f) && approx(store.get(id)->emission.z, 0.3f));
    CHECK(store.getParameter(id, "emission", read, 3));
    CHECK(approx(read[0], 0.1f) && approx(read[1], 0.2f) && approx(read[2], 0.3f));
    CHECK(store.getParameter(id, "emission", read, 4));
    CHECK(approx(read[2], 0.3f) && read[3] == 1.0f);
    CHECK(store.setParameter(id, "emission", color, 4));
    ++version;
    CHECK(!store.setParameter(id, "emission", color, 2));
    CHECK(!store.setParameter(id, "emission", color, 1));
    CHECK(store.get(id)->version == version);

    CHECK(!store.setParameter(id, "shininess", color, 1));
    CHECK(!store.getParameter(id, "shininess", read, 1));
    CHECK(!store.setParameter(id, nullptr, color, 1));
    CHECK(!store.setParameter(id, "metallic", nullptr, 1));
    CHECK(!store.getParameter(id, "metallic", nullptr, 1));
    CHECK(!store.setParameter(MaterialId(), "metallic", color, 1));
    CHECK(!store.getParameter(MaterialId(), "metallic", read, 1));
    CHECK(store.get(id)->version == version);
    store.clear();
}

void testState(MaterialStore& store)
{
    const MaterialId id = store.create("state");
    std::uint32_t version = store.get(id)->version;

    CHECK(store.setTransparency(id, MaterialTransparency::AlphaBlend));
    CHECK(store.get(id)->transparency == MaterialTransparency::AlphaBlend);
    CHECK(store.setCull(id, MaterialCull::Disabled));
    CHECK(store.get(id)->cull == MaterialCull::Disabled);
    CHECK(store.setShading(id, MaterialShading::Unshaded));
    CHECK(store.get(id)->shading == MaterialShading::Unshaded);
    version += 3;
    CHECK(store.get(id)->version == version);

    CHECK(store.setTextureChannel(id, MaterialTexture::Metallic, MaterialTextureChannel::Blue));
    CHECK(store.setTextureChannel(id, MaterialTexture::Roughness, MaterialTextureChannel::Green));
    CHECK(store.setTextureChannel(id, MaterialTexture::AmbientOcclusion,
            MaterialTextureChannel::Alpha));
    CHECK(store.get(id)->metallicChannel == MaterialTextureChannel::Blue);
    CHECK(store.get(id)->roughnessChannel == MaterialTextureChannel::Green);
    CHECK(store.get(id)->aoChannel == MaterialTextureChannel::Alpha);
    version += 3;
    CHECK(!store.setTextureChannel(id, MaterialTexture::Albedo, MaterialTextureChannel::Red));
    CHECK(!store.setTextureChannel(id, MaterialTexture::Normal, MaterialTextureChannel::Red));
    CHECK(store.get(id)->version == version);

    CHECK(!store.setTransparency(MaterialId(), MaterialTransparency::Opaque));
    CHECK(!store.setCull(MaterialId(), MaterialCull::Back));
    CHECK(!store.setShading(MaterialId(), MaterialShading::Lit));
    CHECK(!store.setTextureChannel(MaterialId(), MaterialTexture::Metallic,
            MaterialTextureChannel::Red));
    store.clear();
}

void testTextures(MaterialStore& store, TextureStore& textures)
{
    const MaterialId id = store.create("textured");
    const TextureId white = textures.white();
    const TextureId black = textures.black();
    CHECK(white.valid() && black.valid() && white != black);

    for (std::size_t i = 0; i < static_cast<std::size_t>(MaterialTexture::Count); ++i)
        CHECK(!store.get(id)->textures[i].valid());
    CHECK(!store.getTexture(id, MaterialTexture::Albedo).valid());

    std::uint32_t version = store.get(id)->version;
    CHECK(store.setTexture(id, MaterialTexture::Albedo, white));
    CHECK(store.getTexture(id, MaterialTexture::Albedo) == white);
    CHECK(!store.getTexture(id, MaterialTexture::Normal).valid());
    CHECK(store.setTexture(id, MaterialTexture::Normal, black));
    CHECK(store.getTexture(id, MaterialTexture::Normal) == black);
    version += 2;
    CHECK(store.get(id)->version == version);

    CHECK(store.setTexture(id, MaterialTexture::Albedo, TextureId()));
    CHECK(!store.getTexture(id, MaterialTexture::Albedo).valid());
    CHECK(store.getTexture(id, MaterialTexture::Normal) == black);
    ++version;
    CHECK(store.get(id)->version == version);

    const TextureId stale = textures.createEmpty(2, 2, false, false, "stale");
    CHECK(stale.valid());
    textures.destroy(stale);
    CHECK(!textures.valid(stale));
    CHECK(!store.setTexture(id, MaterialTexture::Emission, stale));
    CHECK(!store.getTexture(id, MaterialTexture::Emission).valid());
    CHECK(!store.setTexture(id, MaterialTexture::Count, white));
    CHECK(!store.setTexture(MaterialId(), MaterialTexture::Albedo, white));
    CHECK(!store.getTexture(MaterialId(), MaterialTexture::Albedo).valid());
    CHECK(!store.getTexture(id, MaterialTexture::Count).valid());
    CHECK(store.get(id)->version == version);
    store.clear();
}

void testVariantKey(MaterialStore& store, TextureStore& textures)
{
    const MaterialId a = store.create("a");
    const MaterialId b = store.create("b");
    const std::uint32_t base = store.get(a)->variantKey();
    CHECK(base == 0u);
    CHECK(store.get(b)->variantKey() == base);
    CHECK(store.defaultMaterial() != a);
    CHECK(store.get(store.defaultMaterial())->variantKey() == base);

    store.setFloat(a, MaterialParam::Metallic, 0.9f);
    store.setFloat(a, MaterialParam::Roughness, 0.1f);
    store.setColor(a, MaterialParam::Albedo, Math::Vec4(0.3f, 0.3f, 0.3f, 1.0f));
    CHECK(store.get(a)->variantKey() == base);

    store.setTransparency(a, MaterialTransparency::AlphaScissor);
    CHECK(store.get(a)->variantKey() == 1u);
    store.setTransparency(a, MaterialTransparency::AlphaBlend);
    CHECK(store.get(a)->variantKey() == 2u);
    store.setTransparency(a, MaterialTransparency::Opaque);
    CHECK(store.get(a)->variantKey() == base);

    store.setCull(a, MaterialCull::Front);
    CHECK(store.get(a)->variantKey() == (1u << 2));
    store.setCull(a, MaterialCull::Disabled);
    CHECK(store.get(a)->variantKey() == (2u << 2));
    store.setCull(a, MaterialCull::Back);
    CHECK(store.get(a)->variantKey() == base);

    store.setShading(a, MaterialShading::Unshaded);
    CHECK(store.get(a)->variantKey() == (1u << 4));
    store.setShading(a, MaterialShading::Lit);
    CHECK(store.get(a)->variantKey() == base);

    const TextureId white = textures.white();
    for (std::uint32_t slot = 0; slot < static_cast<std::uint32_t>(MaterialTexture::Count); ++slot)
    {
        const std::uint32_t bit = slot < 8 ? 5 + slot : 22 + (slot - 8);
        store.setTexture(a, static_cast<MaterialTexture>(slot), white);
        CHECK(store.get(a)->variantKey() == (1u << bit));
        store.setTexture(a, static_cast<MaterialTexture>(slot), TextureId());
        CHECK(store.get(a)->variantKey() == base);
    }

    store.setColor(a, MaterialParam::Emission, Math::Vec4(0.0f, 0.0f, 0.0f, 1.0f));
    CHECK(store.get(a)->variantKey() == base);
    store.setColor(a, MaterialParam::Emission, Math::Vec4(0.0f, 0.1f, 0.0f, 1.0f));
    CHECK(store.get(a)->variantKey() == (1u << 13));
    store.setColor(a, MaterialParam::Emission, Math::Vec4(0.0f, 0.0f, 0.0f, 1.0f));
    CHECK(store.get(a)->variantKey() == base);

    store.setFloat(a, MaterialParam::Clearcoat, 0.5f);
    CHECK(store.get(a)->variantKey() == (1u << 14));
    store.setFloat(a, MaterialParam::Clearcoat, 0.0f);
    CHECK(store.get(a)->variantKey() == base);

    CHECK(store.setDetail(a, true));
    CHECK(store.get(a)->variantKey() == (1u << 25));
    CHECK(store.setDetailBlendMode(a, MaterialBlend::Add));
    CHECK(store.get(a)->variantKey() == ((1u << 25) | (1u << 26)));
    CHECK(store.setDetailBlendMode(a, MaterialBlend::Mul));
    CHECK(store.get(a)->variantKey() == ((1u << 25) | (3u << 26)));
    CHECK(store.setDetailUv(a, MaterialDetailUv::Uv2));
    CHECK(store.get(a)->variantKey() == ((1u << 25) | (3u << 26) | (1u << 28)));
    CHECK(store.setDetail(a, false));
    CHECK(store.get(a)->variantKey() == base);
    CHECK(store.setDetailBlendMode(a, MaterialBlend::Mix));
    CHECK(store.setDetailUv(a, MaterialDetailUv::Uv1));
    CHECK(store.get(a)->variantKey() == base);

    store.setTransparency(a, MaterialTransparency::AlphaBlend);
    store.setCull(a, MaterialCull::Disabled);
    store.setShading(a, MaterialShading::Unshaded);
    store.setTexture(a, MaterialTexture::Albedo, white);
    store.setTexture(a, MaterialTexture::Orm, white);
    store.setColor(a, MaterialParam::Emission, Math::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
    store.setFloat(a, MaterialParam::Clearcoat, 1.0f);
    CHECK(store.get(a)->variantKey() ==
            (2u | (2u << 2) | (1u << 4) | (1u << 5) | (1u << 12) | (1u << 13) | (1u << 14)));
    store.clear();
}

void testDetail(MaterialStore& store, TextureStore& textures)
{
    const MaterialId id = store.create("detail");
    const MaterialInfo* info = store.get(id);
    CHECK(info != nullptr);
    if (!info) return;
    CHECK(!info->detail);
    CHECK(info->detailBlend == MaterialBlend::Mix);
    CHECK(info->detailUv == MaterialDetailUv::Uv1);
    CHECK(info->uv2Scale.x == 1.0f && info->uv2Scale.y == 1.0f && info->uv2Scale.z == 1.0f);
    CHECK(info->uv2Offset.x == 0.0f && info->uv2Offset.y == 0.0f && info->uv2Offset.z == 0.0f);

    std::uint32_t version = info->version;
    CHECK(store.setDetail(id, true));
    CHECK(store.get(id)->detail);
    ++version;
    CHECK(store.get(id)->version == version);
    const MaterialBlend modes[] = { MaterialBlend::Mix, MaterialBlend::Add, MaterialBlend::Sub,
        MaterialBlend::Mul };
    for (MaterialBlend mode: modes)
    {
        CHECK(store.setDetailBlendMode(id, mode));
        CHECK(store.get(id)->detailBlend == mode);
        ++version;
    }
    CHECK(!store.setDetailBlendMode(id, MaterialBlend::Screen));
    CHECK(store.get(id)->detailBlend == MaterialBlend::Mul);
    CHECK(store.get(id)->version == version);
    CHECK(store.setDetailUv(id, MaterialDetailUv::Uv2));
    CHECK(store.get(id)->detailUv == MaterialDetailUv::Uv2);
    ++version;
    CHECK(store.get(id)->version == version);

    CHECK(!store.setDetail(MaterialId(), true));
    CHECK(!store.setDetailBlendMode(MaterialId(), MaterialBlend::Add));
    CHECK(!store.setDetailUv(MaterialId(), MaterialDetailUv::Uv2));

    CHECK(store.setVec3(id, MaterialParam::Uv2Scale, Math::Vec3(2.0f, 3.0f, 1.0f)));
    CHECK(store.setVec3(id, MaterialParam::Uv2Offset, Math::Vec3(0.5f, -0.25f, 0.0f)));
    Math::Vec3 vector;
    CHECK(store.getVec3(id, MaterialParam::Uv2Scale, vector));
    CHECK(vector.x == 2.0f && vector.y == 3.0f && vector.z == 1.0f);
    CHECK(store.getVec3(id, MaterialParam::Uv2Offset, vector));
    CHECK(vector.x == 0.5f && vector.y == -0.25f);
    CHECK(store.setVec3(id, MaterialParam::Uv2Scale, Math::Vec3(500.0f, -500.0f, 0.0f)));
    CHECK(store.getVec3(id, MaterialParam::Uv2Scale, vector));
    CHECK(vector.x == 100.0f && vector.y == -100.0f);
    float scale[3] = { 4.0f, 5.0f, 6.0f };
    CHECK(store.setParameter(id, "uv2_scale", scale, 3));
    float back[3] = {};
    CHECK(store.getParameter(id, "uv2_scale", back, 3));
    CHECK(back[0] == 4.0f && back[1] == 5.0f && back[2] == 6.0f);
    float value = 0.0f;
    CHECK(!store.getFloat(id, MaterialParam::Uv2Scale, value));

    const TextureId white = textures.white();
    const MaterialTexture slots[] = { MaterialTexture::DetailAlbedo, MaterialTexture::DetailNormal,
        MaterialTexture::DetailMask };
    for (MaterialTexture slot: slots)
    {
        CHECK(store.setTexture(id, slot, white));
        CHECK(store.getTexture(id, slot) == white);
        CHECK(!store.setTextureChannel(id, slot, MaterialTextureChannel::Green));
    }

    const MaterialId copy = store.duplicate(id, "copy");
    CHECK(copy.valid());
    const MaterialInfo* duplicate = store.get(copy);
    CHECK(duplicate != nullptr);
    if (duplicate)
    {
        CHECK(duplicate->detail && duplicate->detailBlend == MaterialBlend::Mul);
        CHECK(duplicate->detailUv == MaterialDetailUv::Uv2);
        CHECK(duplicate->uv2Scale.x == 4.0f && duplicate->uv2Offset.x == 0.5f);
        CHECK(duplicate->textures[static_cast<std::size_t>(MaterialTexture::DetailMask)] == white);
        CHECK(duplicate->variantKey() == store.get(id)->variantKey());
    }
    store.clear();
}

void testDuplicate(MaterialStore& store, TextureStore& textures)
{
    const MaterialId source = store.create("source");
    store.setPath(source, ct::String("materials/source.mat"));
    store.setColor(source, MaterialParam::Albedo, Math::Vec4(0.2f, 0.4f, 0.6f, 0.8f));
    store.setFloat(source, MaterialParam::Metallic, 0.7f);
    store.setColor(source, MaterialParam::Emission, Math::Vec4(0.1f, 0.2f, 0.3f, 1.0f));
    store.setVec3(source, MaterialParam::Uv1Scale, Math::Vec3(2.0f, 3.0f, 1.0f));
    store.setTexture(source, MaterialTexture::Albedo, textures.white());
    store.setTextureChannel(source, MaterialTexture::Roughness, MaterialTextureChannel::Green);
    store.setTransparency(source, MaterialTransparency::AlphaScissor);
    store.setCull(source, MaterialCull::Disabled);
    store.setShading(source, MaterialShading::Unshaded);

    const MaterialId named = store.duplicate(source, "copy");
    CHECK(named.valid() && named != source && store.valid(named));
    CHECK(store.count() == 2);
    const MaterialInfo* original = store.get(source);
    const MaterialInfo* copy = store.get(named);
    CHECK(strcmp(copy->name.c_str(), "copy") == 0);
    CHECK(copy->path.empty());
    CHECK(strcmp(original->path.c_str(), "materials/source.mat") == 0);
    CHECK(strcmp(original->name.c_str(), "source") == 0);
    CHECK(approx(copy->albedo.y, 0.4f) && approx(copy->albedo.w, 0.8f));
    CHECK(approx(copy->metallic, 0.7f));
    CHECK(approx(copy->emission.z, 0.3f));
    CHECK(copy->uv1Scale.y == 3.0f);
    CHECK(copy->textures[static_cast<std::size_t>(MaterialTexture::Albedo)] == textures.white());
    CHECK(copy->roughnessChannel == MaterialTextureChannel::Green);
    CHECK(copy->transparency == MaterialTransparency::AlphaScissor);
    CHECK(copy->cull == MaterialCull::Disabled);
    CHECK(copy->shading == MaterialShading::Unshaded);
    CHECK(copy->variantKey() == original->variantKey());

    store.setFloat(named, MaterialParam::Metallic, 0.1f);
    CHECK(approx(store.get(source)->metallic, 0.7f));
    CHECK(approx(store.get(named)->metallic, 0.1f));

    const MaterialId same = store.duplicate(source);
    CHECK(same.valid() && same != source && same != named);
    CHECK(strcmp(store.get(same)->name.c_str(), "source") == 0);
    CHECK(store.get(same)->path.empty());

    CHECK(!store.duplicate(MaterialId()).valid());
    store.destroy(named);
    CHECK(!store.duplicate(named).valid());
    CHECK(store.count() == 2);
    store.clear();
}

} // namespace

int main()
{
    testTable();

    prisma::DriverDesc desc;
    desc.type = prisma::DriverType::Null;
    prisma::DriverError error = prisma::DriverError::None;
    prisma::Driver* driver = prisma::createDriver(desc, &error);
    CHECK(driver != nullptr);
    if (!driver) return 1;

    {
        TextureStore textures(driver);
        MaterialStore store(textures);
        testLifetime(store);
        testTypedParameters(store);
        testNamedParameters(store);
        testState(store);
        testTextures(store, textures);
        testVariantKey(store, textures);
        testDetail(store, textures);
        testDuplicate(store, textures);
    }
    CHECK(MaterialStore::singleton() == nullptr);

    prisma::destroyDriver(driver);
    return failures ? 1 : 0;
}
