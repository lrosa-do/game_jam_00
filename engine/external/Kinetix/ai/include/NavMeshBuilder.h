#ifndef KINETIX_AI_NAVMESHBUILDER_H
#define KINETIX_AI_NAVMESHBUILDER_H

// Turns the collision of a level (any triangle soup: floors, walls, furniture, ceilings) into the
// walkable surface NavMesh::build() expects, the way Recast does: the triangles are rasterised
// into columns of solid spans, the top of a span is floor where the agent fits under the next span
// above, the floor is pulled back from every drop and wall by the agent radius, and each floor
// cell becomes a quad. Floor under a desk is left out because the desk is a span above it.

#include <kinetix/core/Types.h>

#include <ct/vector.hpp>

namespace kinetix::AI
{

struct NavMeshBuildConfig
{
    f32 cellSize = 0.2f; // XZ side of a cell; smaller follows the walls closer and costs more
    f32 agentHeight = 1.8f;
    f32 agentRadius = 0.3f;
    f32 agentMaxClimb = 0.4f;
    f32 agentMaxSlope = 45.0f; // degrees
    u32 maxCells = 16u * 1024u * 1024u; // the cell size grows until the grid fits
};

struct NavMeshBuildStats
{
    u32 columnsX = 0;
    u32 columnsZ = 0;
    f32 cellSize = 0.0f;
    u32 spans = 0;
    u32 floorCells = 0; // where the agent fits, before the radius
    u32 keptCells = 0; // after the radius
};

class NavMeshBuilder
{
public:
    // Triangles are counter-clockwise seen from their front (glTF); a floor faces up. The output
    // is welded where neighbouring cells share a height, so NavMesh::build() links them directly;
    // cells on a step link through its weld pass (edgeWeldDistance > 0, agentMaxClimb).
    static bool build(const f32* vertices, s32 vertexCount, const s32* indices, s32 triangleCount,
                      const NavMeshBuildConfig& config, ct::Vector<f32>& outVertices,
                      ct::Vector<s32>& outIndices, NavMeshBuildStats* stats = nullptr);
};

}

#endif
