#pragma once

#include "prisma3d/physics/PhysicsServer3D.h"
#include "prisma3d/scene/AnimationPlayer.h"
#include "prisma3d/scene/Node3D.h"
#include "prisma3d/scene/Skeleton3D.h"

#include <ct/string.hpp>
#include <ct/vector.hpp>

namespace prisma3d
{

enum class RagdollBone : std::uint8_t
{
    Hips,
    Spine,
    Neck,
    Head,
    LeftArm,
    LeftForeArm,
    LeftHand,
    RightArm,
    RightForeArm,
    RightHand,
    LeftUpLeg,
    LeftLeg,
    LeftFoot,
    LeftToes,
    RightUpLeg,
    RightLeg,
    RightFoot,
    RightToes,
    Count
};

class Ragdoll3D : public Node3D
{
public:
    PRISMA3D_NODE_CLASS(Ragdoll3D, Node3D);
    static void bindProperties();

    Ragdoll3D();
    ~Ragdoll3D() override;

    void setSkeleton(Skeleton3D* skeleton);
    Skeleton3D* getSkeleton() const;
    void setSkeletonPath(const ct::String& path);
    ct::String getSkeletonPath() const;
    void setAnimationPlayer(AnimationPlayer* player);
    AnimationPlayer* getAnimationPlayer() const;
    void setAnimationPlayerPath(const ct::String& path);
    ct::String getAnimationPlayerPath() const;

    void setBone(RagdollBone slot, int bone);
    int getBone(RagdollBone slot) const;
    bool setBoneName(RagdollBone slot, const char* name);
    int autoMapBones();

    bool build();
    bool isBuilt() const;
    bool start();
    void stop();
    bool isActive() const;

    void setBlend(float blend);
    float getBlend() const;
    void setCollisionMask(std::uint32_t mask);
    std::uint32_t getCollisionMask() const;
    void setScale(float scale);
    float getScale() const;
    void setFatness(float fatness);
    float getFatness() const;
    void setHeadSize(float headSize);
    float getHeadSize() const;
    float getFacing() const;
    int getPartBone(PhysicsRagdollPart part) const;
    PhysicsRagdollPart getPartParent(PhysicsRagdollPart part) const;
    float getPartMass(PhysicsRagdollPart part) const;
    void applyImpulse(PhysicsRagdollPart part, const Math::Vec3& impulse,
            const Math::Vec3& offset = Math::Vec3(0.0f, 0.0f, 0.0f));
    void applyBlast(const Math::Vec3& origin, float speed);
    Math::Mat4 getPartTransform(PhysicsRagdollPart part) const;
    Math::Vec3 getPartVelocity(PhysicsRagdollPart part) const;
    PhysicsRagdollId getRagdoll() const;

protected:
    void notificationHandler(int what) override;

private:
    void resolveSkeleton();
    void resolveAnimationPlayer();
    void release();
    void follow();
    void writePose();
    void collectGlobalPose(ct::Vector<Math::Mat4>& global) const;

    Skeleton3D* mSkeleton;
    AnimationPlayer* mPlayer;
    ct::String mSkeletonPath;
    ct::String mPlayerPath;
    int mBones[static_cast<std::size_t>(RagdollBone::Count)];
    PhysicsRagdollId mRagdoll;
    ct::Vector<LocalPose> mSavedPose;
    float mBlend;
    float mScale;
    float mFatness;
    float mHeadSize;
    std::uint32_t mMask;
    bool mActive;
    bool mPlayerWasActive;
};

} // namespace prisma3d
