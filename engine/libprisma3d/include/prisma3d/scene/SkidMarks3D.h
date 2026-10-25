#pragma once

#include "prisma3d/physics/PhysicsServer3D.h"
#include "prisma3d/render/MeshStore.h"
#include "prisma3d/scene/Node3D.h"

#include <ct/vector.hpp>

namespace prisma3d
{

class MeshInstance3D;

class SkidMarks3D : public Node3D
{
public:
    PRISMA3D_NODE_CLASS(SkidMarks3D, Node3D);
    static void bindProperties();

    static constexpr int kQuads = 2048;
    static constexpr int kWheels = 64;

    SkidMarks3D();
    ~SkidMarks3D() override;

    void setColor(const Math::Vec4& color);
    const Math::Vec4& getColor() const;
    void setLife(float seconds);
    float getLife() const;
    void setFade(float seconds);
    float getFade() const;
    void setRollingStrength(float strength);
    float getRollingStrength() const;

    int claimWheels(int count);
    void touch(int wheel, const Math::Vec3& contact, const Math::Vec3& halfAcross, float strength,
            PhysicsBodyId exclude = PhysicsBodyId());
    void lift(int wheel);
    void clear();

protected:
    void notificationHandler(int what) override;

private:
    struct Wheel
    {
        bool down = false;
        Math::Vec3 left;
        Math::Vec3 right;
        Math::Vec3 place;
        float strength = 0.0f;
    };

    void build();
    void release();
    void update(float step);
    Math::Vec3 onGround(const Math::Vec3& place, PhysicsBodyId exclude) const;

    SurfaceData mSurface;
    ct::Vector<float> mStrength;
    ct::Vector<float> mBorn;
    MeshId mMesh;
    MaterialId mMaterial;
    MeshInstance3D* mShown;
    Wheel mWheels[kWheels];
    Math::Vec4 mColor;
    float mLife;
    float mFade;
    float mRolling;
    int mClaimed;
    int mNext;
    float mTime;
    float mSinceUpload;
    float mSinceFade;
    bool mDirty;
    bool mReady;
};

} // namespace prisma3d
