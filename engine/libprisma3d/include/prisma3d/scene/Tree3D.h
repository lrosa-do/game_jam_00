#pragma once

#include "prisma3d/render/MeshStore.h"
#include "prisma3d/scene/Collision.h"
#include "prisma3d/scene/TreeGenerator.h"
#include "prisma3d/scene/VisualInstance3D.h"

#include <cstdint>

namespace prisma3d
{

class Tree3D : public VisualInstance3D
{
public:
    PRISMA3D_NODE_CLASS(Tree3D, VisualInstance3D);
    static void bindProperties();

    Tree3D();
    ~Tree3D() override;

    void setParams(const TreeParams& params);
    const TreeParams& getParams() const;
    void setSeed(std::int32_t seed);
    std::int32_t getSeed() const;

    void setBarkMaterial(MaterialId material);
    MaterialId getBarkMaterial() const;
    MaterialId getAssignedBarkMaterial() const;
    void setLeafMaterial(MaterialId material);
    MaterialId getLeafMaterial() const;
    MaterialId getAssignedLeafMaterial() const;

    bool rebuild();
    bool isDirty() const;
    int getBuildCount() const;

    const SurfaceData& getBarkGeometry() const;
    const SurfaceData& getLeafGeometry() const;
    void setShadowLeafStep(int step);
    int getShadowLeafStep() const;
    const TreeGeometry& getShadowGeometry() const;
    bool buildCollision(CollisionMesh& out) const;
    Math::Box getAabb() const override;

protected:
    void notificationHandler(int what) override;

private:
    void upload();
    void applyMaterials();
    void markDirty();
    void connectPreDraw();
    void disconnectPreDraw();

    TreeParams mParams;
    TreeGeometry mGeometry;
    bool mDirty;
    int mBuildCount;
    MeshId mMesh;
    MeshId mShadowMesh;
    TreeGeometry mShadowGeometry;
    int mShadowLeafStep;
    bool mSurfacesReady;
    MaterialId mDefaultBark;
    MaterialId mDefaultLeaf;
    MaterialId mBarkMaterial;
    MaterialId mLeafMaterial;
};

} // namespace prisma3d
