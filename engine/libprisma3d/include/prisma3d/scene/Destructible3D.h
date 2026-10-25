#pragma once

#include "prisma3d/render/MeshStore.h"
#include "prisma3d/scene/PhysicsBody3D.h"
#include "prisma3d/scene/VisualInstance3D.h"

#include <ct/function.hpp>
#include <ct/vector.hpp>

namespace prisma3d
{

class DebrisMesh3D : public MeshInstance3D
{
public:
    PRISMA3D_NODE_CLASS(DebrisMesh3D, MeshInstance3D);
    static void bindProperties();

    DebrisMesh3D();
    ~DebrisMesh3D() override;

    void adopt(MeshId mesh);
    MeshId getOwnedMesh() const;

private:
    MeshId mOwned;
};

class DebrisBody3D : public RigidBody3D
{
public:
    PRISMA3D_NODE_CLASS(DebrisBody3D, RigidBody3D);
    static void bindProperties();

    DebrisBody3D();

    void setLifetime(float seconds);
    float getLifetime() const;
    float getAge() const;

protected:
    void notificationHandler(int what) override;

private:
    float mLifetime;
    float mAge;
};

class DestructibleBody3D : public RigidBody3D
{
public:
    PRISMA3D_NODE_CLASS(DestructibleBody3D, RigidBody3D);
    static void bindProperties();

    bool saveData(ct::Json& data, SceneResult& result, const ct::String& path) const override;
    bool loadData(const ct::Json& data, SceneResult& result, const ct::String& path) override;

    DestructibleBody3D();

    void setFragmentCount(int count);
    int getFragmentCount() const;
    void setBreakSpeed(float metersPerSecond);
    float getBreakSpeed() const;
    void setDensity(float density);
    float getDensity() const;
    void setDebrisLifetime(float seconds);
    float getDebrisLifetime() const;
    void setPushSpeed(float speed);
    float getPushSpeed() const;
    void setOuterMaterial(MaterialId material);
    MaterialId getOuterMaterial() const;
    void setInnerMaterial(MaterialId material);
    MaterialId getInnerMaterial() const;
    void setSeed(std::uint32_t seed);
    std::uint32_t getSeed() const;
    void setSourcePoints(const Math::Vec3* points, std::size_t count);

    bool breakAt(const Math::Vec3& globalPoint,
            const Math::Vec3& impulse = Math::Vec3(0.0f, 0.0f, 0.0f));
    bool isBroken() const;

    ct::Function<void(const ct::Vector<DebrisBody3D*>& fragments)> broken;

protected:
    void notificationHandler(int what) override;
    void contactStarted(PhysicsBody3D* other, const PhysicsContact& contact) override;
    bool needsContactMonitor() const override;

private:
    bool collectPoints(ct::Vector<Math::Vec3>& out) const;
    float random();

    int mFragments;
    float mBreakSpeed;
    float mDensity;
    float mLifetime;
    float mPush;
    MaterialId mOuter;
    MaterialId mInner;
    std::uint32_t mSeed;
    std::uint32_t mSeedValue;
    ct::Vector<Math::Vec3> mPoints;
    Math::Vec3 mLastVelocity;
    bool mBroken;
};

} // namespace prisma3d
