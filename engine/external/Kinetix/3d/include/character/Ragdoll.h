#ifndef KINETIX_PHYSICS_CHARACTER_RAGDOLL_H
#define KINETIX_PHYSICS_CHARACTER_RAGDOLL_H

#include "character/SkeletonView.h"
#include "collision/CollisionFilter.h"
#include "collision/CollisionShape.h"
#include "dynamics/ConeTwistJoint.h"
#include "dynamics/HingeJoint.h"
#include "dynamics/RigidBody.h"
#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

#include <ct/array.hpp>
#include <ct/deque.hpp>
#include <ct/vector.hpp>

namespace kinetix::Physics
{

class PhysicsWorld;

enum class RagdollPart : u8
{
    Pelvis,
    Spine,
    Head,
    LeftUpperLeg,
    LeftLowerLeg,
    RightUpperLeg,
    RightLowerLeg,
    LeftUpperArm,
    LeftLowerArm,
    RightUpperArm,
    RightLowerArm,
    LeftFoot,
    RightFoot,
    Count
};

struct RagdollSettings
{
    f32 scale = 1.0f;
    f32 fatness = 1.0f;
    f32 headSize = 1.0f;
    f32 massScale = 6.0f;
};

class Ragdoll
{
public:
    static constexpr u32 kVelocityIterations = 16;
    static constexpr u32 kPositionIterations = 6;

    Ragdoll() = default;
    // Leaves the world it was added to: the world keeps pointers to the
    // parts' bodies and shapes and to the joints, all of which live here.
    ~Ragdoll();

    // Not copyable or movable. Besides the world's pointers, every part's
    // shape pointer and every joint's body references point into this
    // object, and a copy or a move would carry them over unchanged.
    Ragdoll(const Ragdoll&) = delete;
    Ragdoll& operator=(const Ragdoll&) = delete;
    Ragdoll(Ragdoll&&) = delete;
    Ragdoll& operator=(Ragdoll&&) = delete;

    bool build(const SkeletonView& skeleton, const RagdollBoneMap& bones,
            const ct::Vector<Math::mat4>& restPose, f32 restScale, const RagdollSettings& settings);

    bool valid() const
    {
        return mValid;
    }
    bool inWorld() const
    {
        return mWorld != nullptr;
    }
    bool active() const
    {
        return mActive;
    }
    f32 facing() const
    {
        return mFacing;
    }
    const RagdollSettings& settings() const
    {
        return mSettings;
    }

    s32 boneIndex(RagdollPart part) const
    {
        return mParts[static_cast<usize>(part)].boneIndex;
    }
    RagdollPart parentOf(RagdollPart part) const
    {
        return mParts[static_cast<usize>(part)].parentPart;
    }
    f32 mass(RagdollPart part) const
    {
        return mParts[static_cast<usize>(part)].mass;
    }

    void addToWorld(PhysicsWorld& world, const Math::mat4& ownerWorld,
            const ct::Vector<Math::mat4>& globalPose);
    void removeFromWorld();
    void activate();
    void deactivate();
    bool detach(RagdollPart part);
    bool detached(RagdollPart part) const
    {
        return mParts[static_cast<usize>(part)].detached;
    }

    void setTargetPose(const Math::mat4& ownerWorld, const ct::Vector<Math::mat4>& globalPose);
    void beginStep(f32 duration);
    void endStep();
    void writePose(ct::Vector<LocalPose>& localPose, const Math::mat4& ownerWorld, f32 alpha) const;

    RigidBody* body(RagdollPart part)
    {
        return &mParts[static_cast<usize>(part)].rigidBody;
    }
    const RigidBody* body(RagdollPart part) const
    {
        return &mParts[static_cast<usize>(part)].rigidBody;
    }
    u32 bodyId(RagdollPart part) const
    {
        return mParts[static_cast<usize>(part)].bodyId;
    }

    void setCollisionMask(u32 mask);

private:
    struct Part
    {
        s32 boneIndex = -1;
        RagdollPart parentPart = RagdollPart::Count;
        ct::Vector<s32> parentChain;
        bool rooted = true;

        CapsuleShape capsule{ 0.1f, 0.1f };
        BoxShape box{ Math::vec3(0.1f) };
        CollisionShape* shape = nullptr;
        f32 mass = 1.0f;
        Math::mat3 inertia{ 1.0f };

        Math::vec3 axis{ 0.0f, 1.0f, 0.0f };
        Math::vec3 offset{ 0.0f };
        Math::quat shapeRotation{ 1.0f, 0.0f, 0.0f, 0.0f };
        Math::quat restBasis{ 1.0f, 0.0f, 0.0f, 0.0f };
        Math::vec3 restPosition{ 0.0f };
        Math::vec3 restBodyPosition{ 0.0f };

        RigidBody rigidBody;
        u32 bodyId = 0xFFFFFFFFu;
        Joint* joint = nullptr;
        bool detached = false;

        bool hasTarget = false;
        Math::vec3 targetPosition{ 0.0f };
        Math::quat targetRotation{ 1.0f, 0.0f, 0.0f, 0.0f };
        Math::vec3 previousPosition{ 0.0f };
        Math::quat previousRotation{ 1.0f, 0.0f, 0.0f, 0.0f };
    };

    Part& part(RagdollPart p)
    {
        return mParts[static_cast<usize>(p)];
    }
    const Part& part(RagdollPart p) const
    {
        return mParts[static_cast<usize>(p)];
    }

    void resolveChains(const SkeletonView& skeleton);
    void stabilize();
    void createJoints();
    void attachAll();
    void boneTransform(const Part& p, Math::vec3& position, Math::quat& rotation) const;
    void bodyFromBone(const Part& p, const Math::vec3& bonePosition, const Math::quat& boneRotation,
            Math::vec3& position, Math::quat& rotation) const;

    ct::Array<Part, static_cast<usize>(RagdollPart::Count)> mParts;
    ct::Deque<ConeTwistJoint> mConeTwistJoints;
    ct::Deque<HingeJoint> mHingeJoints;
    PhysicsWorld* mWorld = nullptr;
    RagdollSettings mSettings;
    CollisionFilter mFilter{ 1u, 0xFFFFFFFFu };
    f32 mFacing = 1.0f;
    bool mValid = false;
    bool mActive = false;
};

} // namespace kinetix::Physics

#endif // KINETIX_PHYSICS_CHARACTER_RAGDOLL_H
