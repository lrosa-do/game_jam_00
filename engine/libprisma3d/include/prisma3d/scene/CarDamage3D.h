#pragma once

#include "prisma3d/render/AssetLoader.h"
#include "prisma3d/scene/Node3D.h"

#include <ct/string.hpp>
#include <ct/vector.hpp>

namespace prisma3d
{

class CarDamage;
class GroundVehicle3D;
class ImpactFx3D;
class MeshInstance3D;

class CarDamage3D : public Node3D
{
public:
    PRISMA3D_NODE_CLASS(CarDamage3D, Node3D);
    static void bindProperties();

    CarDamage3D();
    ~CarDamage3D() override;

    void setModel(const ct::String& path);
    const ct::String& getModel() const;
    void setSkipPrefix(const ct::String& prefix);
    const ct::String& getSkipPrefix() const;
    void setMinImpulse(float impulse);
    float getMinImpulse() const;
    void setImpactFxPath(const ct::String& path);
    const ct::String& getImpactFxPath() const;

    bool isBuilt() const;
    float getPeak() const;
    float getNoseCrush() const;
    float getTailCrush() const;
    float getStepImpulse() const;
    float getTotalImpulse() const;
    float damageNear(const Math::Vec3& point, float radius) const;
    void hit(const Math::Vec3& point, float impulse);
    void reset();
    CarDamage* getDamage() const;
    GroundVehicle3D* getVehicle() const;

protected:
    void notificationHandler(int what) override;

private:
    bool build();
    void release();
    void listen();
    ImpactFx3D* impactFx() const;
    void smokeCrushed();
    void followShape();
    void anchorChildren();
    void moveAnchors();
    Math::Mat4 chassis() const;

    CarDamage* mDamage;
    ct::Vector<MeshInstance3D*> mShown;
    ct::Vector<MeshId> mMeshes;
    struct Anchor
    {
        ct::String name;
        Math::Mat4 rest;
        std::size_t piece = 0;
        std::size_t part = 0;
        std::size_t vertex = 0;
    };
    ct::Vector<Anchor> mAnchors;
    ct::String mModel;
    ct::String mSkipPrefix;
    ct::String mImpactFxPath;
    float mMinImpulse;
    float mStepImpulse;
    float mTotalImpulse;
    float mEatenNose;
    float mEatenTail;
    unsigned mSmokeTick;
    float mLastSpeed;
    float mSoundWait;
    bool mFailed;
};

} // namespace prisma3d
