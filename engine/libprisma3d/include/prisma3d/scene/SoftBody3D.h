#pragma once

#include "prisma3d/physics/PhysicsServer3D.h"
#include "prisma3d/render/MeshStore.h"
#include "prisma3d/scene/VisualInstance3D.h"

#include <ct/vector.hpp>

namespace prisma3d
{

class SoftBody3D : public MeshInstance3D
{
public:
    PRISMA3D_NODE_CLASS(SoftBody3D, MeshInstance3D);
    static void bindProperties();

    bool saveData(ct::Json& data, SceneResult& result, const ct::String& path) const override;
    bool loadData(const ct::Json& data, SceneResult& result, const ct::String& path) override;

    SoftBody3D();
    ~SoftBody3D() override;

    void setGeometry(const SurfaceData& geometry);
    const SurfaceData& getGeometry() const;
    void makeCloth(float width, float depth, int segmentsX, int segmentsZ, float uvTiles = 1.0f);
    void makeBall(float radius, int rings = 12, int slices = 18);

    void setParameters(const PhysicsSoftBodyParameters& parameters);
    const PhysicsSoftBodyParameters& getParameters() const;
    void setAttachmentLimit(float multiplier);
    float getAttachmentLimit() const;

    void setPinned(std::size_t vertex, bool pinned);
    bool isPinned(std::size_t vertex) const;
    void pinInSphere(const Math::Vec3& localCenter, float radius, bool pinned = true);
    void pinInBox(const Math::Box& localBox, bool pinned = true);
    void clearPins();

    void setWind(const Math::Vec3& wind);
    const Math::Vec3& getWind() const;
    void addVelocity(const Math::Vec3& delta);
    void setSimulationEnabled(bool enabled);
    bool isSimulationEnabled() const;
    void reset();

    bool isBuilt() const;
    std::size_t getParticleCount() const;
    Math::Vec3 getParticlePosition(std::size_t vertex) const;
    float getWorstStretch() const;
    PhysicsSoftBodyId getSoftBody() const;
    MeshId getOwnedMesh() const;

protected:
    void notificationHandler(int what) override;

private:
    bool build();
    void release();
    void followPins();
    void upload();
    void markDirty();

    SurfaceData mGeometry;
    PhysicsSoftBodyParameters mParameters;
    float mAttachmentLimit;
    Math::Vec3 mWind;
    ct::Vector<std::uint8_t> mPinFlags;
    ct::Vector<std::uint32_t> mVertexParticle;
    ct::Vector<std::uint32_t> mParticleVertex;
    ct::Vector<std::uint8_t> mParticlePinned;
    ct::Vector<Math::Vec3> mPositions;
    ct::Vector<Math::Vec3> mNormals;
    ct::Vector<Math::Vec3> mTangentSums;
    ct::Vector<Math::Vec3> mBitangentSums;
    SurfaceData mUpload;
    PhysicsSoftBodyId mBody;
    MeshId mOwned;
    Math::Mat4 mPinFrame;
    std::uint32_t mVertexCapacity;
    std::uint32_t mIndexCapacity;
    bool mDirty;
    bool mEnabled;
    bool mSurfaceReady;
    bool mPinFrameValid;
};

} // namespace prisma3d
