#pragma once

#include "prisma3d/render/MeshStore.h"
#include "prisma3d/scene/Collision.h"
#include "prisma3d/scene/VisualInstance3D.h"

#include <ct/vector.hpp>

#include <cstdint>

namespace prisma3d
{

class ManualMesh3D : public VisualInstance3D
{
public:
    PRISMA3D_NODE_CLASS(ManualMesh3D, VisualInstance3D);
    static void bindProperties();

    bool saveData(ct::Json& data, SceneResult& result, const ct::String& path) const override;
    bool loadData(const ct::Json& data, SceneResult& result, const ct::String& path) override;

    ManualMesh3D();
    ~ManualMesh3D() override;

    void setDynamic(std::uint32_t vertexCapacity, std::uint32_t indexCapacity);
    bool isDynamic() const;
    std::uint32_t getVertexCapacity() const;
    std::uint32_t getIndexCapacity() const;

    bool surfaceBegin(prisma::Topology topology, MaterialId material = MaterialId());
    void surfaceSetColor(const Math::Vec4& color);
    void surfaceSetNormal(const Math::Vec3& normal);
    void surfaceSetTangent(const Math::Vec4& tangent);
    void surfaceSetUv(const Math::Vec2& uv);
    void surfaceSetUv2(const Math::Vec2& uv2);
    void surfaceAddVertex(const Math::Vec3& position);
    void surfaceAddIndex(std::uint32_t index);
    bool surfaceEnd();
    bool isSurfaceActive() const;

    int addSurface(const SurfaceData& surface, MaterialId material = MaterialId());
    void clearSurfaces();
    int getSurfaceCount() const;
    const SurfaceData* getSurfaceData(int surface) const;

    void setSurfaceMaterial(int surface, MaterialId material);
    bool setSurfaceMaterialPath(int surface, const ct::String& path);
    MaterialId getSurfaceMaterial(int surface) const;

    MeshId getMesh() const;
    bool buildCollision(CollisionMesh& out) const;

private:
    struct Surface
    {
        SurfaceData data;
        MaterialId material;
    };

    int commitSurface(const SurfaceData& data, MaterialId material);
    void resetActive();
    bool ensureMesh();
    bool uploadStatic(const SurfaceData& data, MaterialId material);
    bool uploadDynamic(const SurfaceData& data, MaterialId material, int index);
    void blankGpuSurface(int index);

    ct::Vector<Surface> mSurfaces;
    ct::Vector<MaterialId> mGpuMaterials;
    MeshId mMesh;
    std::uint32_t mVertexCapacity;
    std::uint32_t mIndexCapacity;
    bool mActive;
    prisma::Topology mActiveTopology;
    MaterialId mActiveMaterial;
    SurfaceData mBuild;
    bool mUsesColors;
    bool mUsesNormals;
    bool mUsesTangents;
    bool mUsesUvs;
    bool mUsesUv2s;
    Math::Vec4 mCurrentColor;
    Math::Vec3 mCurrentNormal;
    Math::Vec4 mCurrentTangent;
    Math::Vec2 mCurrentUv;
    Math::Vec2 mCurrentUv2;
};

} // namespace prisma3d
