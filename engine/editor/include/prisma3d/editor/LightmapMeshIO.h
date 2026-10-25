#pragma once

#include "prisma3d/render/MeshStore.h"

#include <ct/string.hpp>
#include <ct/vector.hpp>

namespace prisma3d
{

class LightmapMeshIO
{
public:
    static bool readSurfaces(MeshId mesh, ct::Vector<SurfaceData>& out, ct::String& error);
    static bool hasUv2(MeshId mesh);
    static bool isStaticTriangleMesh(MeshId mesh);
    static std::size_t vertexCount(MeshId mesh);
    static bool saveMesh(MeshId source, const ct::Vector<SurfaceData>& surfaces,
            const ct::String& name, const ct::String& path, ct::String& error);
};

} // namespace prisma3d
