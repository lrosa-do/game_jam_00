#include "SceneMaterial.h"

#include <math.h>

namespace prisma3d
{

namespace
{
void channelVector(MaterialTextureChannel channel, float* out)
{
    out[0] = out[1] = out[2] = out[3] = 0.0f;
    switch (channel)
    {
        case MaterialTextureChannel::Red:
            out[0] = 1.0f;
            break;
        case MaterialTextureChannel::Green:
            out[1] = 1.0f;
            break;
        case MaterialTextureChannel::Blue:
            out[2] = 1.0f;
            break;
        case MaterialTextureChannel::Alpha:
            out[3] = 1.0f;
            break;
        case MaterialTextureChannel::Grayscale:
            out[0] = out[1] = out[2] = 0.333333f;
            break;
    }
}

} // namespace

float srgbToLinear(float value)
{
    return value < 0.04045f
                   ? value * (1.0f / 12.92f)
                   : powf(static_cast<float>((value + 0.055) * (1.0 / (1.0 + 0.055))), 2.4f);
}

void fillMaterial(const MaterialInfo* material, MaterialDataGpu& out, const TextureStore* textures)
{
    MaterialInfo fallback;
    if (!material) material = &fallback;
    out.albedo[0] = srgbToLinear(material->albedo.x);
    out.albedo[1] = srgbToLinear(material->albedo.y);
    out.albedo[2] = srgbToLinear(material->albedo.z);
    out.albedo[3] = material->albedo.w;
    out.emission[0] = srgbToLinear(material->emission.x);
    out.emission[1] = srgbToLinear(material->emission.y);
    out.emission[2] = srgbToLinear(material->emission.z);
    out.emission[3] =
            material->emissionOperator == MaterialEmissionOperator::Multiply ? 1.0f : 0.0f;
    out.uv1ScaleOffset[0] = material->uv1Scale.x;
    out.uv1ScaleOffset[1] = material->uv1Scale.y;
    out.uv1ScaleOffset[2] = material->uv1Offset.x;
    out.uv1ScaleOffset[3] = material->uv1Offset.y;
    const bool orm =
            !material->textures[static_cast<std::size_t>(MaterialTexture::Metallic)].valid() &&
            !material->textures[static_cast<std::size_t>(MaterialTexture::Roughness)].valid() &&
            material->textures[static_cast<std::size_t>(MaterialTexture::Orm)].valid();
    channelVector(orm ? MaterialTextureChannel::Blue : material->metallicChannel,
            out.metallicChannel);
    channelVector(orm ? MaterialTextureChannel::Green : material->roughnessChannel,
            out.roughnessChannel);
    channelVector(orm ? MaterialTextureChannel::Red : material->aoChannel, out.aoChannel);
    out.surface[0] = material->metallic;
    out.surface[1] = material->specular;
    out.surface[2] = material->roughness;
    out.surface[3] = material->normalScale;
    out.surface2[0] = material->aoLightAffect;
    out.surface2[1] = material->emissionEnergy;
    out.surface2[2] = material->transparency == MaterialTransparency::AlphaScissor
                              ? material->alphaScissorThreshold
                              : 0.0f;
    out.surface2[3] = material->alphaAntialiasingEdge;
    out.refraction[0] = material->refraction;
    out.refraction[1] = material->refraction != 0.0f ? 1.0f : 0.0f;
    out.refraction[2] = 0.0f;
    out.refraction[3] = 0.0f;
    channelVector(material->refractionChannel, out.refractionChannel);
    const WaterMaterialData& water = material->waterData;
    out.water[0] = material->water ? 1.0f : 0.0f;
    out.water[1] = water.waveScale;
    out.water[2] = water.waveChop;
    out.water[3] = water.bump;
    out.waterWind[0] = water.windDirection.x;
    out.waterWind[1] = water.windDirection.y;
    out.waterWind[2] = water.windSpeed;
    out.waterWind[3] = water.bump2;
    out.waterWaves[0] = water.bigWaves.x;
    out.waterWaves[1] = water.bigWaves.y;
    out.waterWaves[2] = water.midWaves.x;
    out.waterWaves[3] = water.midWaves.y;
    out.waterWaves2[0] = water.smallWaves.x;
    out.waterWaves2[1] = water.smallWaves.y;
    out.waterWaves2[2] = water.specularMix;
    out.waterWaves2[3] = water.shoreFade;
    out.waterDepth[0] = water.depthFade.x;
    out.waterDepth[1] = water.depthFade.y;
    out.waterDepth[2] = water.depthFade.z;
    out.waterDepth[3] = water.depthScale;
    out.backlight[0] = srgbToLinear(material->backlight.x);
    out.backlight[1] = srgbToLinear(material->backlight.y);
    out.backlight[2] = srgbToLinear(material->backlight.z);
    out.backlight[3] = 0.0f;
    out.uv2ScaleOffset[0] = material->uv2Scale.x;
    out.uv2ScaleOffset[1] = material->uv2Scale.y;
    out.uv2ScaleOffset[2] = material->uv2Offset.x;
    out.uv2ScaleOffset[3] = material->uv2Offset.y;
    const TextureId mainNormal =
            material->textures[static_cast<std::size_t>(MaterialTexture::Normal)];
    out.detail[0] = static_cast<float>(material->detailBlend);
    out.detail[1] = material->detailUv == MaterialDetailUv::Uv2 ? 1.0f : 0.0f;
    out.detail[2] = 0.0f;
    out.detail[3] = (textures ? textures->valid(mainNormal) : mainNormal.valid()) ? 1.0f : 0.0f;
    out.billboard[0] = material->billboardKeepScale ? 1.0f : 0.0f;
    out.billboard[1] = 0.0f;
    out.billboard[2] = 0.0f;
    out.billboard[3] = 0.0f;
    out.grass[0] = material->grassDrawDistance;
    out.grass[1] = material->grassCameraBend;
    out.grass[2] = 0.0f;
    out.grass[3] = 0.0f;
    out.grassTintA[0] = material->grassTintA.x;
    out.grassTintA[1] = material->grassTintA.y;
    out.grassTintA[2] = material->grassTintA.z;
    out.grassTintA[3] = 0.0f;
    out.grassTintB[0] = material->grassTintB.x;
    out.grassTintB[1] = material->grassTintB.y;
    out.grassTintB[2] = material->grassTintB.z;
    out.grassTintB[3] = 0.0f;
    out.grassWind[0] = material->grassWindDirection.x;
    out.grassWind[1] = material->grassWindDirection.z;
    out.grassWind[2] = material->grassWindAmplitude;
    out.grassWind[3] = material->grassWindSpeed;
    out.leafWind[0] = material->leafWindDirection.x;
    out.leafWind[1] = material->leafWindDirection.z;
    out.leafWind[2] = material->leafWindStrength;
    out.leafWind[3] = material->leafWindSpeed;
    out.leafWindModel[0] = material->leafWindHeight;
    out.leafWindModel[1] = 0.0f;
    out.leafWindModel[2] = 0.0f;
    out.leafWindModel[3] = 0.0f;
    const OceanMaterialData& ocean = material->ocean;
    out.oceanColor[0] = ocean.waterColor.x;
    out.oceanColor[1] = ocean.waterColor.y;
    out.oceanColor[2] = ocean.waterColor.z;
    out.oceanColor[3] = ocean.waterColor.w;
    out.oceanExtinction[0] = 1.0f - ocean.extinctionColor.x;
    out.oceanExtinction[1] = 1.0f - ocean.extinctionColor.y;
    out.oceanExtinction[2] = 1.0f - ocean.extinctionColor.z;
    out.oceanExtinction[3] = ocean.waterHeight;
    out.oceanParams[0] = ocean.patchLength > 0.0f ? 1.0f / ocean.patchLength : 0.0f;
    out.oceanParams[1] = ocean.patchLength / 512.0f;
    out.oceanParams[2] = ocean.displacementTolerance > 1.0f ? ocean.displacementTolerance : 1.0f;
    out.oceanParams[3] = ocean.waveAmplitude;
    out.oceanMisc[0] = ocean.timeScale;
    out.oceanMisc[1] = ocean.gridWidth > 0 ? 1.0f / static_cast<float>(ocean.gridWidth) : 0.0f;
    out.oceanMisc[2] = ocean.gridHeight > 0 ? 1.0f / static_cast<float>(ocean.gridHeight) : 0.0f;
    out.oceanMisc[3] = 0.0f;
    const TerrainMaterialData& terrain = material->terrain;
    out.terrainUv0[0] = terrain.uvMultiplier[0];
    out.terrainUv0[1] = terrain.uvMultiplier[1];
    out.terrainUv0[2] = terrain.uvMultiplier[2];
    out.terrainUv0[3] = terrain.uvMultiplier[3];
    out.terrainUv1[0] = terrain.uvMultiplier[4];
    out.terrainUv1[1] = terrain.parallaxScale;
    out.terrainUv1[2] = terrain.detailStrength;
    out.terrainUv1[3] = 0.0f;
    out.terrainParams[0] = static_cast<float>(terrain.layerCount);
    out.terrainParams[1] = terrain.normalMapping ? 1.0f : 0.0f;
    out.terrainParams[2] = static_cast<float>(terrain.parallax);
    out.terrainParams[3] =
            static_cast<float>((terrain.useLightmap ? 1 : 0) | (terrain.useColourMap ? 2 : 0) |
                               (terrain.detailModulation ? 4 : 0));
}

TextureId sceneTexture(const MaterialInfo& material, MaterialTexture slot)
{
    TextureId texture = material.textures[static_cast<std::size_t>(slot)];
    if (!texture.valid() &&
            (slot == MaterialTexture::Metallic || slot == MaterialTexture::Roughness ||
                    slot == MaterialTexture::AmbientOcclusion))
        texture = material.textures[static_cast<std::size_t>(MaterialTexture::Orm)];
    return texture;
}

std::uint32_t sceneVariant(const MaterialInfo* material, const SurfaceInfo& surface,
        const TextureStore& textures)
{
    if (!material) return 0;
    const bool unshaded = material->shading == MaterialShading::Unshaded;
    std::uint32_t variant = unshaded ? 32u : 0u;
    if (!(surface.attributeMask & kVertexUv)) return variant;
    if (textures.valid(sceneTexture(*material, MaterialTexture::Albedo))) variant |= 1u;
    if (!unshaded)
    {
        if (textures.valid(sceneTexture(*material, MaterialTexture::Metallic)) ||
                textures.valid(sceneTexture(*material, MaterialTexture::Roughness)) ||
                textures.valid(sceneTexture(*material, MaterialTexture::AmbientOcclusion)))
            variant |= 2u;
        if (textures.valid(sceneTexture(*material, MaterialTexture::Emission))) variant |= 4u;
        const bool detailNormal =
                material->detail && textures.valid(material->textures[static_cast<std::size_t>(
                                            MaterialTexture::DetailNormal)]);
        if ((textures.valid(sceneTexture(*material, MaterialTexture::Normal)) || detailNormal) &&
                (surface.attributeMask & kVertexTangent))
            variant |= 8u;
    }
    if (material->transparency == MaterialTransparency::AlphaScissor) variant |= 16u;
    if (material->detail) variant |= 64u;
    return variant;
}


} // namespace prisma3d
