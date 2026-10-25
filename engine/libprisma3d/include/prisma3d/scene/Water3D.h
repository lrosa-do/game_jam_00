#pragma once

#include "prisma3d/render/MaterialStore.h"
#include "prisma3d/render/MeshStore.h"
#include "prisma3d/scene/VisualInstance3D.h"

namespace prisma3d
{

class PlanarReflector3D;

enum class WaterReflection : std::uint8_t
{
    Sky,
    Planar
};

class Water3D : public VisualInstance3D
{
public:
    PRISMA3D_NODE_CLASS(Water3D, VisualInstance3D);
    static void bindProperties();

    Water3D();
    ~Water3D() override;

    void setSize(const Math::Vec2& size);
    Math::Vec2 getSize() const;
    void setUvTiling(float tilesPerMetre);
    float getUvTiling() const;

    void setNormalTexture(TextureId texture);
    TextureId getNormalTexture() const;
    void setColor(const Math::Vec4& colour);
    Math::Vec4 getColor() const;
    void setRoughness(float roughness);
    float getRoughness() const;
    void setSpecular(float specular);
    float getSpecular() const;
    void setRefraction(float strength);
    float getRefraction() const;
    void setRefractionEnabled(bool enabled);
    bool isRefractionEnabled() const;

    void setWaves(const WaterMaterialData& waves);
    const WaterMaterialData& getWaves() const;
    void setWind(const Math::Vec2& direction, float speed);

    void setReflection(WaterReflection mode);
    WaterReflection getReflection() const;
    void setReflectionResolution(std::uint32_t resolution);
    std::uint32_t getReflectionResolution() const;
    void setReflectionMaxDistance(float distance);
    float getReflectionMaxDistance() const;
    PlanarReflector3D* getPlanarReflector() const;

    MaterialId getMaterial() const;
    MeshId getMesh() const;

private:
    void rebuildMesh();
    void applyMaterial();
    void applyReflection();

    MeshId mMesh;
    MaterialId mMaterial;
    TextureId mNormalTexture;
    Math::Vec2 mSize;
    float mUvTiling;
    Math::Vec4 mColor;
    float mRoughness;
    float mSpecular;
    float mRefraction;
    bool mRefractionEnabled;
    WaterMaterialData mWaves;
    WaterReflection mReflection;
    std::uint32_t mReflectionResolution;
    float mReflectionMaxDistance;
    PlanarReflector3D* mReflector;
};

} // namespace prisma3d
