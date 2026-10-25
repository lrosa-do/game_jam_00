#pragma once

#include "prisma3d/render/MaterialStore.h"
#include "prisma3d/render/MeshStore.h"
#include "prisma3d/scene/HairStrands.h"
#include "prisma3d/scene/VisualInstance3D.h"

#include <ct/string.hpp>
#include <ct/vector.hpp>

namespace prisma3d
{

class Skeleton3D;

enum class HairSimulation : std::uint8_t
{
    Auto,
    Cpu,
    Gpu
};

class Hair3D : public VisualInstance3D
{
public:
    PRISMA3D_NODE_CLASS(Hair3D, VisualInstance3D);
    static void bindProperties();

    bool saveData(ct::Json& data, SceneResult& result, const ct::String& path) const override;
    bool loadData(const ct::Json& data, SceneResult& result, const ct::String& path) override;

    Hair3D();
    ~Hair3D() override;

    void setScalp(const SurfaceData& surface);
    void setScalpLengths(const ct::Vector<float>& lengths);
    std::uint32_t getScalpTriangleCount() const;

    void setSkeletonPath(const ct::String& path);
    const ct::String& getSkeletonPath() const;
    Skeleton3D* getSkeleton() const;

    void setParams(const HairParams& params);
    const HairParams& getParams() const;
    void setStrandCount(std::uint32_t count);
    void setSegmentCount(std::uint32_t count);
    void setBillboardCount(std::uint32_t count);
    void setRandomSeed(std::uint32_t seed);
    void setLength(float length);
    void setStiffness(float stiffness);
    void setDrag(float drag);
    void setGravityPower(float power);
    void setRandomness(float randomness);
    void setViewDistance(float distance);
    void setWidth(float width);
    void setUniformity(float uniformity);
    void setCameraBend(bool enabled);
    void setAtlasRects(const ct::Vector<HairAtlasRect>& rects);

    void setMaterial(MaterialId material);
    MaterialId getMaterial() const;
    MaterialId getAssignedMaterial() const;
    void setTexture(TextureId texture);
    TextureId getTexture() const;

    void setWind(const Math::Vec3& direction, float speed = 1.0f, float randomness = 5.0f,
            float waveSize = 1.0f);
    void setWindDirection(const Math::Vec3& direction);
    const Math::Vec3& getWindDirection() const;
    void setWindSpeed(float speed);
    float getWindSpeed() const;
    void setWindRandomness(float randomness);
    float getWindRandomness() const;
    void setWindWaveSize(float waveSize);
    float getWindWaveSize() const;
    void clearColliders();
    void addCollider(const HairCollider& collider);
    std::uint32_t getColliderCount() const;
    void clearForceFields();
    void addForceField(const HairForceField& field);

    void setSimulation(HairSimulation mode);
    HairSimulation getSimulation() const;
    bool isSimulatingOnGpu() const;

    void reset();
    void update(float deltaTime);
    const HairStrands& getStrands() const;

    Math::Box getAabb() const override;

protected:
    void notificationHandler(int what) override;

private:
    struct Gpu;

    void ensureMaterial();
    bool ensureSurface(bool storage);
    void skinScalp();
    void connectPreDraw();
    void disconnectPreDraw();
    void applyIdentityTransform();
    bool wantsGpu() const;
    void fillFrame(HairFrame& frame, float deltaTime);
    void updateCpu(const HairFrame& frame);
    void updateGpu(const HairFrame& frame);
    void releaseGpu();
    void uploadGeometry();

    HairStrands mStrands;
    ct::Vector<Math::Vec3> mScalpPositions;
    ct::Vector<Math::Vec3> mScalpNormals;
    ct::Vector<std::uint32_t> mScalpJoints;
    ct::Vector<Math::Vec4> mScalpWeights;
    ct::Vector<Math::Vec3> mSkinnedPositions;
    ct::Vector<Math::Vec3> mSkinnedNormals;
    ct::Vector<float> mScalpLengths;
    ct::Vector<std::uint32_t> mScalpIndices;
    ct::Vector<HairCollider> mColliders;
    ct::Vector<HairForceField> mForceFields;
    SurfaceData mGeometry;
    ct::String mSkeletonPath;
    MeshId mMesh;
    MaterialId mMaterial;
    MaterialId mDefaultMaterial;
    TextureId mTexture;
    Math::Vec3 mWindDirection;
    float mWindSpeed;
    float mWindRandomness;
    float mWindWaveSize;
    float mTime;
    std::uint32_t mSurfaceVertices;
    std::uint32_t mSurfaceIndices;
    bool mSurfaceReady;
    bool mSurfaceStorage;
    bool mScalpDirty;
    bool mScalpIndicesDirty;
    bool mGpuRegenerate;
    HairSimulation mSimulation;
    Gpu* mGpu;
};

} // namespace prisma3d
