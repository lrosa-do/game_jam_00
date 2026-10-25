#pragma once

#include "prisma3d/render/MaterialStore.h"
#include "prisma3d/render/MeshStore.h"
#include "prisma3d/render/OceanSimulation.h"
#include "prisma3d/scene/VisualInstance3D.h"

namespace prisma3d
{

class PlanarReflector3D;

class Ocean3D : public VisualInstance3D
{
public:
    PRISMA3D_NODE_CLASS(Ocean3D, VisualInstance3D);
    static void bindProperties();

    Ocean3D();
    ~Ocean3D() override;

    bool isSupported() const;

    void setSimulationParams(const OceanSimulationParams& params);
    const OceanSimulationParams& getSimulationParams() const;
    void setWaterColor(const Math::Vec4& colour);
    Math::Vec4 getWaterColor() const;
    void setExtinctionColor(const Math::Vec3& colour);
    Math::Vec3 getExtinctionColor() const;
    void setSurfaceDetail(std::uint32_t detail);
    std::uint32_t getSurfaceDetail() const;
    void setDisplacementTolerance(float tolerance);
    float getDisplacementTolerance() const;
    float getWaterHeight() const;

    void setPlanarReflection(bool enabled);
    bool hasPlanarReflection() const;
    void setReflectionResolution(std::uint32_t resolution);
    std::uint32_t getReflectionResolution() const;

    Math::Vec3 getDisplacedPosition(const Math::Vec3& worldPosition);

    MaterialId getMaterial() const;
    const OceanSimulation* getSimulation() const;
    void update();

protected:
    void notificationHandler(int what) override;

private:
    void rebuildGrid();
    void applyMaterial();
    void applyReflection();
    void applyTransform();

    OceanSimulation* mSimulation;
    OceanSimulationParams mParams;
    MeshId mMesh;
    MaterialId mMaterial;
    Math::Vec4 mWaterColor;
    Math::Vec3 mExtinctionColor;
    std::uint32_t mSurfaceDetail;
    float mTolerance;
    float mWaterHeight;
    bool mPlanar;
    std::uint32_t mReflectionResolution;
    PlanarReflector3D* mReflector;
    bool mDirty;
    ct::Vector<float> mFields;
    std::uint32_t mFieldsRequest;
    bool mFieldsValid;
};

} // namespace prisma3d
