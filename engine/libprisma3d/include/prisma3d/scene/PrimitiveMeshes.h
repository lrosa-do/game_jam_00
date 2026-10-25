#pragma once

#include "prisma3d/render/MeshStore.h"

#include "prisma3d/core/Pixmap.h"

#include "mathc.h"

#include <ct/string.hpp>

#include <cstddef>
#include <cstdint>

namespace prisma3d
{

enum class PrimitiveKind : std::uint8_t
{
    Box,
    Plane,
    Sphere,
    Cylinder,
    Cone,
    Capsule,
    Torus,
    HillsPlane,
    Heightfield
};

enum class PrimitiveParamType : std::uint8_t
{
    Float,
    Count,
    Vec3
};

struct PrimitiveParamDef
{
    const char* key;
    PrimitiveParamType type;
    float fallback;
    float minimum;
    float maximum;
};

struct PrimitiveRecipe
{
    PrimitiveKind kind = PrimitiveKind::Box;
    float values[8] = {};
    ct::String heightmap;
};

void boxMeshArrays(SurfaceData& out, const Math::Vec3& size, int subdivideWidth = 0,
        int subdivideHeight = 0, int subdivideDepth = 0);
MeshId createBoxMesh(MeshStore& meshes, const Math::Vec3& size = Math::Vec3(1.0f, 1.0f, 1.0f),
        const char* name = "BoxMesh");

void planeMeshArrays(SurfaceData& out, float width, float depth, int segmentsX = 1,
        int segmentsZ = 1, float uvTiles = 1.0f);
void sphereMeshArrays(SurfaceData& out, float radius, int rings = 16, int slices = 24);
void cylinderMeshArrays(SurfaceData& out, float radius, float height, int slices = 24);
void coneMeshArrays(SurfaceData& out, float radius, float height, int slices = 24);
void capsuleMeshArrays(SurfaceData& out, float radius, float height, int rings = 8,
        int slices = 24);
void torusMeshArrays(SurfaceData& out, float majorRadius, float minorRadius, int majorSegments = 24,
        int minorSegments = 12);
void hillsPlaneMeshArrays(SurfaceData& out, float width, float depth, int segmentsX, int segmentsZ,
        const Pixmap& heightmap, float heightScale, float uvTiles = 1.0f);
void heightfieldMeshArrays(SurfaceData& out, const float* heights, int width, int height,
        float cellSize, float uvTiles = 1.0f);
void heightfieldMeshArrays(SurfaceData& out, const Pixmap& heightmap, float cellSize,
        float heightScale, float uvTiles = 1.0f);

MeshId createPlaneMesh(MeshStore& meshes, float width, float depth, int segmentsX = 1,
        int segmentsZ = 1, float uvTiles = 1.0f, const char* name = "PlaneMesh");
MeshId createSphereMesh(MeshStore& meshes, float radius, int rings = 16, int slices = 24,
        const char* name = "SphereMesh");
MeshId createCylinderMesh(MeshStore& meshes, float radius, float height, int slices = 24,
        const char* name = "CylinderMesh");
MeshId createConeMesh(MeshStore& meshes, float radius, float height, int slices = 24,
        const char* name = "ConeMesh");
MeshId createCapsuleMesh(MeshStore& meshes, float radius, float height, int rings = 8,
        int slices = 24, const char* name = "CapsuleMesh");
MeshId createTorusMesh(MeshStore& meshes, float majorRadius, float minorRadius,
        int majorSegments = 24, int minorSegments = 12, const char* name = "TorusMesh");
MeshId createHeightfieldMesh(MeshStore& meshes, const Pixmap& heightmap, float cellSize,
        float heightScale, float uvTiles = 1.0f, const char* name = "HeightfieldMesh");

MeshId createHillsPlaneMeshFromFile(MeshStore& meshes, float width, float depth, int segmentsX,
        int segmentsZ, const ct::String& heightmapFile, float heightScale, float uvTiles = 1.0f,
        const char* name = "HillsPlaneMesh");
MeshId createHeightfieldMeshFromFile(MeshStore& meshes, const ct::String& heightmapFile,
        float cellSize, float heightScale, float uvTiles = 1.0f,
        const char* name = "HeightfieldMesh");

std::size_t primitiveKindCount();
const char* primitiveKindName(PrimitiveKind kind);
bool primitiveKindHasHeightmap(PrimitiveKind kind);
int primitiveHeightmapSlot(PrimitiveKind kind);
std::size_t primitiveParamCount(PrimitiveKind kind);
const PrimitiveParamDef& primitiveParam(PrimitiveKind kind, std::size_t index);
void primitiveDefaults(PrimitiveKind kind, PrimitiveRecipe& out);
bool parsePrimitiveRecipe(const ct::String& path, PrimitiveRecipe& out, ct::String& error);
ct::String primitiveRecipePath(const PrimitiveRecipe& recipe);
std::size_t primitiveTriangleCount(const PrimitiveRecipe& recipe);

bool isPrimitivePath(const ct::String& path);
MeshId loadPrimitiveMesh(MeshStore& meshes, const ct::String& path, ct::String& error);

} // namespace prisma3d
