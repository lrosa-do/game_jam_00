#pragma once

#include "SceneData.h"
#include "prisma3d/render/MeshStore.h"
#include "prisma3d/render/TextureStore.h"

namespace prisma3d
{

float srgbToLinear(float value);
void fillMaterial(const MaterialInfo* material, MaterialDataGpu& out,
        const TextureStore* textures = nullptr);
TextureId sceneTexture(const MaterialInfo& material, MaterialTexture slot);
std::uint32_t sceneVariant(const MaterialInfo* material, const SurfaceInfo& surface,
        const TextureStore& textures);

} // namespace prisma3d
