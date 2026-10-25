#pragma once

#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/scene/Node3D.h"

#include <ct/string.hpp>
#include <ct/vector.hpp>

namespace prisma3d
{

class Skeleton3D;
class PlanarReflector3D;

enum class GiMode : std::uint8_t
{
    Disabled,
    Static,
    Dynamic
};

class VisualInstance3D : public Node3D
{
public:
    PRISMA3D_NODE_CLASS(VisualInstance3D, Node3D);
    static void bindProperties();
    static constexpr NodeClass kAncestralClass = NodeClass::VisualInstance3D;

    VisualInstance3D();
    ~VisualInstance3D() override;

    InstanceId getInstance() const;
    void setBase(MeshId base);
    void setBase(MultiMeshId base);
    MeshId getBase() const;
    void setLayerMask(std::uint32_t mask);
    std::uint32_t getLayerMask() const;
    void setLayerMaskValue(int layerNumber, bool enable);
    bool getLayerMaskValue(int layerNumber) const;
    void setCastShadows(bool cast);
    bool getCastShadows() const;
    void setTransparency(float transparency);
    float getTransparency() const;
    void setGiMode(GiMode mode);
    GiMode getGiMode() const;
    void setLightmapTexelScale(float scale);
    float getLightmapTexelScale() const;
    void setReflector(PlanarReflector3D* reflector);
    PlanarReflector3D* getReflector() const;
    void setReflectorPath(const ct::String& path);
    ct::String getReflectorPath() const;
    virtual Math::Box getAabb() const;
    static void refreshReflectorChoices(Node* root);

protected:
    void notificationHandler(int what) override;
    void validateProperty(PropertyInfo& info) const override;

private:
    void updateVisibility();
    void resolveReflector();

    InstanceId mInstance;
    MeshId mBase;
    std::uint32_t mLayers;
    bool mVisibleInWorld;
    bool mCastShadows;
    float mTransparency;
    GiMode mGiMode;
    float mLightmapTexelScale;
    PlanarReflector3D* mReflector;
    ct::String mReflectorPath;
};

class MeshInstance3D : public VisualInstance3D
{
public:
    PRISMA3D_NODE_CLASS(MeshInstance3D, VisualInstance3D);
    static void bindProperties();

    bool saveData(ct::Json& data, SceneResult& result, const ct::String& path) const override;
    bool loadData(const ct::Json& data, SceneResult& result, const ct::String& path) override;
    static constexpr NodeClass kAncestralClass = NodeClass::MeshInstance3D;

    MeshInstance3D();
    ~MeshInstance3D() override;

    void setMesh(MeshId mesh);
    MeshId getMesh() const;
    void setColor(const Math::Vec4& color);
    Math::Vec4 getColor() const;
    void setSurfaceMaterial(std::uint32_t surface, MaterialId material);
    MaterialId getSurfaceMaterial(std::uint32_t surface) const;
    void setSurfaceVisible(std::uint32_t surface, bool visible);
    bool isSurfaceVisible(std::uint32_t surface) const;
    void setMaterialOverride(MaterialId material);
    MaterialId getMaterialOverride() const;
    void setSkeletonPath(const ct::String& path);
    const ct::String& getSkeletonPath() const;
    Skeleton3D* getSkeleton() const;
    Math::Box getAabb() const override;

protected:
    void notificationHandler(int what) override;
    void validateProperty(PropertyInfo& info) const override;

private:
    void resolveSkeleton();

    MeshId mMesh;
    Math::Vec4 mColor;
    ct::Vector<MaterialId> mSurfaceMaterials;
    ct::Vector<std::uint8_t> mHiddenSurfaces;
    MaterialId mMaterialOverride;
    ct::String mSkeletonPath;
};

} // namespace prisma3d
