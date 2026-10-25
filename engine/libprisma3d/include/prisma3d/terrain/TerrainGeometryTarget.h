#pragma once

#include "prisma3d/terrain/Terrain.h"
#include "prisma3d/terrain/TerrainQuadTreeNode.h"

#include <cstdint>

namespace prisma3d
{

class TerrainGeometryTarget
{
public:
    virtual ~TerrainGeometryTarget() = default;

    virtual void writeVertexRange(const TerrainQuadTreeNode* owner, std::uint32_t first,
            std::uint32_t count, const TerrainVertexData& source) = 0;
    virtual void geometryUpdated(const TerrainRect& rect) = 0;
};

} // namespace prisma3d
