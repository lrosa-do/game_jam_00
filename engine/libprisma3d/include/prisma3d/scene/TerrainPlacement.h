#pragma once

#include "prisma3d/terrain/TerrainPath.h"

#include "mathc.h"

#include <ct/vector.hpp>

namespace prisma3d
{

class Terrain3D;

class TerrainPlacement
{
public:
    TerrainPlacement();

    void setMaxSlope(float degrees);
    float getMaxSlope() const;
    void addExclusion(const TerrainPath& path, float margin);
    void clearExclusions();
    int getExclusionCount() const;
    bool getExclusion(int index, TerrainPath& path, float& margin) const;

    bool isExcluded(float x, float z) const;
    bool accepts(const Terrain3D& terrain, const Math::Vec3& world) const;

private:
    struct Exclusion
    {
        TerrainPath path;
        float margin = 0.0f;
        float minX = 0.0f;
        float maxX = 0.0f;
        float minZ = 0.0f;
        float maxZ = 0.0f;
    };

    float mMaxSlope;
    ct::Vector<Exclusion> mExclusions;
};

} // namespace prisma3d
