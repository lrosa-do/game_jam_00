#pragma once

#include "prisma3d/render/TerrainMaterialData.h"
#include "prisma3d/render/TextureStore.h"
#include "prisma3d/terrain/Terrain.h"

#include "mathc.h"

#include <cstdint>

namespace prisma3d
{

struct TerrainCompositeInputs
{
    TerrainMaterialData maps;
    Math::Vec3 lightDirection = Math::Vec3(0.70710678f, -0.70710678f, 0.0f);
    Math::Vec3 ambient = Math::Vec3(1.0f, 1.0f, 1.0f);
    Math::Vec3 diffuse = Math::Vec3(1.0f, 1.0f, 1.0f);
};

class TerrainComposite
{
public:
    TerrainComposite();
    ~TerrainComposite();
    TerrainComposite(const TerrainComposite&) = delete;
    TerrainComposite& operator=(const TerrainComposite&) = delete;

    bool create(prisma::Driver* driver, TextureStore* textures);
    void destroy();
    bool isCreated() const;

    bool render(const TerrainCompositeInputs& inputs, TextureId target, const TerrainRect& rect);

private:
    prisma::Driver* mDriver;
    TextureStore* mTextures;
    prisma::ShaderHandle mVertex;
    prisma::ShaderHandle mFragment;
    prisma::PipelineHandle mPipeline;
    prisma::BufferHandle mParams;
    prisma::SamplerHandle mSampler;
    prisma::SamplerHandle mLayerSampler;
};

} // namespace prisma3d
