#include "prisma3d/scene/TerrainPlacement.h"

#include "prisma3d/scene/Terrain3D.h"

#include <float.h>

namespace prisma3d
{

TerrainPlacement::TerrainPlacement()
        : mMaxSlope(90.0f),
          mExclusions()
{
}

void TerrainPlacement::setMaxSlope(float degrees)
{
    mMaxSlope = degrees;
}

float TerrainPlacement::getMaxSlope() const
{
    return mMaxSlope;
}

void TerrainPlacement::addExclusion(const TerrainPath& path, float margin)
{
    if (path.points.empty()) return;
    Exclusion exclusion;
    exclusion.path = path;
    exclusion.margin = margin;
    exclusion.minX = FLT_MAX;
    exclusion.minZ = FLT_MAX;
    exclusion.maxX = -FLT_MAX;
    exclusion.maxZ = -FLT_MAX;
    for (std::size_t i = 0; i < path.points.size(); ++i)
    {
        exclusion.minX = path.points[i].x < exclusion.minX ? path.points[i].x : exclusion.minX;
        exclusion.maxX = path.points[i].x > exclusion.maxX ? path.points[i].x : exclusion.maxX;
        exclusion.minZ = path.points[i].z < exclusion.minZ ? path.points[i].z : exclusion.minZ;
        exclusion.maxZ = path.points[i].z > exclusion.maxZ ? path.points[i].z : exclusion.maxZ;
    }
    const float reach = path.getMaxHalfWidth() + margin;
    exclusion.minX -= reach;
    exclusion.maxX += reach;
    exclusion.minZ -= reach;
    exclusion.maxZ += reach;
    mExclusions.push_back(exclusion);
}

void TerrainPlacement::clearExclusions()
{
    mExclusions.clear();
}

int TerrainPlacement::getExclusionCount() const
{
    return static_cast<int>(mExclusions.size());
}

bool TerrainPlacement::getExclusion(int index, TerrainPath& path, float& margin) const
{
    if (index < 0 || static_cast<std::size_t>(index) >= mExclusions.size()) return false;
    path = mExclusions[static_cast<std::size_t>(index)].path;
    margin = mExclusions[static_cast<std::size_t>(index)].margin;
    return true;
}

bool TerrainPlacement::isExcluded(float x, float z) const
{
    for (std::size_t i = 0; i < mExclusions.size(); ++i)
    {
        const Exclusion& exclusion = mExclusions[i];
        if (x < exclusion.minX || x > exclusion.maxX || z < exclusion.minZ || z > exclusion.maxZ)
            continue;
        TerrainPathHit hit;
        if (exclusion.path.nearest(x, z, hit) && hit.clearance < exclusion.margin) return true;
    }
    return false;
}

bool TerrainPlacement::accepts(const Terrain3D& terrain, const Math::Vec3& world) const
{
    if (isExcluded(world.x, world.z)) return false;
    if (mMaxSlope < 90.0f && terrain.getWorldSlopeAt(world) > mMaxSlope) return false;
    return true;
}

} // namespace prisma3d
