#pragma once

#include "prisma3d/scene/MultiMesh.h"
#include "prisma3d/scene/VisualInstance3D.h"

namespace prisma3d
{

class MultiMeshInstance3D : public VisualInstance3D
{
public:
    PRISMA3D_NODE_CLASS(MultiMeshInstance3D, VisualInstance3D);
    static void bindProperties();

    bool saveData(ct::Json& data, SceneResult& result, const ct::String& path) const override;
    bool loadData(const ct::Json& data, SceneResult& result, const ct::String& path) override;

    MultiMeshInstance3D();
    ~MultiMeshInstance3D() override;

    MultiMesh* getMultiMesh() const;

    void setMesh(MeshId mesh);
    MeshId getMesh() const;
    bool setUseColors(bool enable);
    bool getUseColors() const;
    bool setUseCustomData(bool enable);
    bool getUseCustomData() const;
    void setInstanceCount(int count);
    int getInstanceCount() const;
    bool setVisibleInstanceCount(int count);
    int getVisibleInstanceCount() const;
    void setInstanceTransform(int instance, const Math::Mat4& transform);
    Math::Mat4 getInstanceTransform(int instance) const;
    void setInstanceColor(int instance, const Math::Vec4& color);
    Math::Vec4 getInstanceColor(int instance) const;
    void setInstanceCustomData(int instance, const Math::Vec4& data);
    Math::Vec4 getInstanceCustomData(int instance) const;
    void setSurfaceMaterial(std::uint32_t surface, MaterialId material);
    MaterialId getSurfaceMaterial(std::uint32_t surface) const;
    void setMaterialOverride(MaterialId material);
    MaterialId getMaterialOverride() const;
    Math::Box getAabb() const override;

private:
    MultiMesh* mMultiMesh;
    ct::Vector<MaterialId> mSurfaceMaterials;
    MaterialId mMaterialOverride;
};

} // namespace prisma3d
