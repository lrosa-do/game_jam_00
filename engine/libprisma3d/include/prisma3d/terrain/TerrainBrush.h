#pragma once

#include "prisma3d/terrain/Terrain.h"
#include "prisma3d/terrain/TerrainPath.h"

#include "mathc.h"

#include <cstdint>

namespace prisma3d
{

float terrainBrushWeight(float distance, float radius);

bool terrainBrushHeight(Terrain& terrain, const Math::Vec3& worldCentre, float radius,
        float strength, float timeElapsed, bool lower);
bool terrainBrushBlend(Terrain& terrain, std::uint8_t layer, const Math::Vec3& worldCentre,
        float radius, float strength, float timeElapsed, bool remove);
bool terrainBrushSmooth(Terrain& terrain, const Math::Vec3& worldCentre, float radius,
        float strength, float timeElapsed);
bool terrainBrushFlatten(Terrain& terrain, const Math::Vec3& worldCentre, float radius,
        float targetHeight, float strength, float timeElapsed);
bool terrainBrushColour(Terrain& terrain, const Math::Vec3& worldCentre, float radius,
        const Math::Vec3& colour, float strength, float timeElapsed);

struct TerrainPathBrush
{
    float shoulder = 0.0f;
    float bank = 8.0f;
    float heightOffset = 0.0f;
};

float terrainPathWeight(float clearance, float shoulder, float bank);
std::size_t terrainFlattenAlongPath(Terrain& terrain, const TerrainPath& path,
        const TerrainPathBrush& brush);
std::size_t terrainPaintAlongPath(Terrain& terrain, std::uint8_t layer, const TerrainPath& path,
        const TerrainPathBrush& brush);

} // namespace prisma3d
