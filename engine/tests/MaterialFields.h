#pragma once

#include "prisma3d/render/MaterialLibrary.h"

namespace prisma3d
{

constexpr int kMaterialFieldCount = 63;

static_assert(sizeof(MaterialInfo) == 624,
        "MaterialInfo changed: give the new field a key in the material library format, add it to "
        "materialFieldDifferences and everythingMaterial, then update this size; the terrain "
        "maps and the ocean data are run-time state owned by their nodes and have no key");

static_assert(sizeof(MaterialDef) == 1248,
        "MaterialDef changed: give the new field a key in the material library format and add it "
        "to materialDefDifferences, then update this size");

inline bool sameVec3(const Math::Vec3& a, const Math::Vec3& b)
{
    return a.x == b.x && a.y == b.y && a.z == b.z;
}

inline bool sameVec4(const Math::Vec4& a, const Math::Vec4& b)
{
    return a.x == b.x && a.y == b.y && a.z == b.z && a.w == b.w;
}

inline int materialFieldDifferences(const MaterialInfo& a, const MaterialInfo& b)
{
    int differences = 0;
    differences += !sameVec4(a.albedo, b.albedo);
    differences += a.metallic != b.metallic;
    differences += a.specular != b.specular;
    differences += a.roughness != b.roughness;
    differences += !sameVec3(a.emission, b.emission);
    differences += a.emissionEnergy != b.emissionEnergy;
    differences += a.normalScale != b.normalScale;
    differences += a.aoLightAffect != b.aoLightAffect;
    differences += a.clearcoat != b.clearcoat;
    differences += a.clearcoatRoughness != b.clearcoatRoughness;
    differences += a.alphaScissorThreshold != b.alphaScissorThreshold;
    differences += a.alphaAntialiasing != b.alphaAntialiasing;
    differences += a.alphaAntialiasingEdge != b.alphaAntialiasingEdge;
    differences += !sameVec3(a.uv1Scale, b.uv1Scale);
    differences += !sameVec3(a.uv1Offset, b.uv1Offset);
    differences += !sameVec3(a.uv2Scale, b.uv2Scale);
    differences += !sameVec3(a.uv2Offset, b.uv2Offset);
    differences += a.metallicChannel != b.metallicChannel;
    differences += a.roughnessChannel != b.roughnessChannel;
    differences += a.aoChannel != b.aoChannel;
    differences += a.transparency != b.transparency;
    differences += a.cull != b.cull;
    differences += a.emissionOperator != b.emissionOperator;
    differences += a.shading != b.shading;
    differences += a.blend != b.blend;
    differences += a.depthTest != b.depthTest;
    differences += a.detail != b.detail;
    differences += a.detailBlend != b.detailBlend;
    differences += a.detailUv != b.detailUv;
    differences += a.billboard != b.billboard;
    differences += a.billboardKeepScale != b.billboardKeepScale;
    differences += a.receiveShadows != b.receiveShadows;
    differences += !sameVec3(a.backlight, b.backlight);
    differences += a.grass != b.grass;
    differences += a.grassDrawDistance != b.grassDrawDistance;
    differences += a.grassCameraBend != b.grassCameraBend;
    differences += !sameVec3(a.grassTintA, b.grassTintA);
    differences += !sameVec3(a.grassTintB, b.grassTintB);
    differences += !sameVec3(a.grassWindDirection, b.grassWindDirection);
    differences += a.grassWindAmplitude != b.grassWindAmplitude;
    differences += a.grassWindSpeed != b.grassWindSpeed;
    differences += a.leafWind != b.leafWind;
    differences += !sameVec3(a.leafWindDirection, b.leafWindDirection);
    differences += a.leafWindStrength != b.leafWindStrength;
    differences += a.leafWindSpeed != b.leafWindSpeed;
    differences += a.leafWindHeight != b.leafWindHeight;
    differences += a.terrainMorph != b.terrainMorph;
    differences += a.refraction != b.refraction;
    differences += a.refractionChannel != b.refractionChannel;
    differences += a.water != b.water;
    differences += a.waterData.windDirection.x != b.waterData.windDirection.x ||
                   a.waterData.windDirection.y != b.waterData.windDirection.y;
    differences += a.waterData.windSpeed != b.waterData.windSpeed;
    differences += a.waterData.bigWaves.x != b.waterData.bigWaves.x ||
                   a.waterData.bigWaves.y != b.waterData.bigWaves.y;
    differences += a.waterData.midWaves.x != b.waterData.midWaves.x ||
                   a.waterData.midWaves.y != b.waterData.midWaves.y;
    differences += a.waterData.smallWaves.x != b.waterData.smallWaves.x ||
                   a.waterData.smallWaves.y != b.waterData.smallWaves.y;
    differences += a.waterData.waveScale != b.waterData.waveScale;
    differences += a.waterData.waveChop != b.waterData.waveChop;
    differences += a.waterData.bump != b.waterData.bump;
    differences += a.waterData.bump2 != b.waterData.bump2;
    differences += a.waterData.specularMix != b.waterData.specularMix;
    differences += !sameVec3(a.waterData.depthFade, b.waterData.depthFade);
    differences += a.waterData.depthScale != b.waterData.depthScale;
    differences += a.waterData.shoreFade != b.waterData.shoreFade;
    return differences;
}

inline int materialDefDifferences(const MaterialDef& a, const MaterialDef& b)
{
    int differences = materialFieldDifferences(a.info, b.info);
    for (std::size_t slot = 0; slot < kMaterialTextureCount; ++slot)
    {
        differences += !(a.paths[slot] == b.paths[slot]);
        bool sameSequence = a.sequences[slot].size() == b.sequences[slot].size();
        for (std::size_t frame = 0; sameSequence && frame < a.sequences[slot].size(); ++frame)
            sameSequence = a.sequences[slot][frame] == b.sequences[slot][frame];
        differences += !sameSequence;
        differences += a.sequenceFps[slot] != b.sequenceFps[slot];
    }
    return differences;
}

inline MaterialInfo everythingMaterial()
{
    MaterialInfo m;
    m.name = "everything";
    m.transparency = MaterialTransparency::AlphaBlend;
    m.alphaScissorThreshold = 0.25f;
    m.alphaAntialiasing = MaterialAlphaAntialiasing::AlphaToCoverage;
    m.alphaAntialiasingEdge = 0.6f;
    m.blend = MaterialBlend::Screen;
    m.cull = MaterialCull::Front;
    m.depthTest = false;
    m.shading = MaterialShading::Unshaded;
    m.albedo = Math::Vec4(0.1f, 0.2f, 0.3f, 0.4f);
    m.metallic = 0.75f;
    m.metallicChannel = MaterialTextureChannel::Blue;
    m.specular = 0.25f;
    m.roughness = 0.5f;
    m.roughnessChannel = MaterialTextureChannel::Green;
    m.aoChannel = MaterialTextureChannel::Alpha;
    m.aoLightAffect = 0.5f;
    m.emission = Math::Vec3(1.0f, 0.5f, 0.25f);
    m.emissionEnergy = 4.0f;
    m.emissionOperator = MaterialEmissionOperator::Multiply;
    m.normalScale = -2.0f;
    m.clearcoat = 0.75f;
    m.clearcoatRoughness = 0.125f;
    m.uv1Scale = Math::Vec3(2.0f, 3.0f, 4.0f);
    m.uv1Offset = Math::Vec3(0.5f, 0.25f, 0.125f);
    m.uv2Scale = Math::Vec3(3.0f, 2.0f, 1.5f);
    m.uv2Offset = Math::Vec3(0.75f, 0.5f, 0.25f);
    m.detail = true;
    m.detailBlend = MaterialBlend::Mul;
    m.detailUv = MaterialDetailUv::Uv2;
    m.billboard = MaterialBillboard::FixedY;
    m.billboardKeepScale = true;
    m.receiveShadows = false;
    m.backlight = Math::Vec3(0.5f, 0.25f, 0.125f);
    m.grass = true;
    m.grassDrawDistance = 90.0f;
    m.grassCameraBend = 0.25f;
    m.grassTintA = Math::Vec3(0.5f, 0.5f, 0.25f);
    m.grassTintB = Math::Vec3(1.0f, 0.5f, 0.5f);
    m.grassWindDirection = Math::Vec3(0.0f, 0.0f, 1.0f);
    m.grassWindAmplitude = 0.5f;
    m.grassWindSpeed = 2.0f;
    m.leafWind = true;
    m.leafWindDirection = Math::Vec3(0.0f, 0.0f, 1.0f);
    m.leafWindStrength = 0.5f;
    m.leafWindSpeed = 2.0f;
    m.leafWindHeight = 12.0f;
    m.terrainMorph = true;
    m.refraction = 0.3f;
    m.refractionChannel = MaterialTextureChannel::Green;
    m.water = true;
    m.waterData.windDirection = Math::Vec2(1.0f, 0.5f);
    m.waterData.windSpeed = 2.0f;
    m.waterData.bigWaves = Math::Vec2(0.5f, 0.4f);
    m.waterData.midWaves = Math::Vec2(0.25f, 0.2f);
    m.waterData.smallWaves = Math::Vec2(0.125f, 0.0625f);
    m.waterData.waveScale = 10.0f;
    m.waterData.waveChop = 0.5f;
    m.waterData.bump = 2.0f;
    m.waterData.bump2 = 4.0f;
    m.waterData.specularMix = 0.75f;
    m.waterData.depthFade = Math::Vec3(0.1f, 0.05f, 0.025f);
    m.waterData.depthScale = 1.5f;
    m.waterData.shoreFade = 8.0f;
    return m;
}

} // namespace prisma3d
