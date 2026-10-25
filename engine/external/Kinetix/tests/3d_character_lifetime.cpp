#include "check.h"

#include "character/CharacterRigidBody.h"
#include "character/Ragdoll.h"
#include "collision/CollisionShape.h"
#include "dynamics/PhysicsWorld.h"
#include "dynamics/RigidBody.h"

#include <kinetix/core/Math.h>

#include <type_traits>

using kinetix::f32;
using kinetix::s32;
using kinetix::u32;
using kinetix::Physics::CharacterRigidBody;
using kinetix::Physics::PhysicsWorld;
using kinetix::Physics::Ragdoll;
using RagdollPartId = kinetix::Physics::RagdollPart;

// The world holds raw pointers into both: a copy or a move would leave it
// pointing at the original.
static_assert(!std::is_copy_constructible_v<CharacterRigidBody>, "CharacterRigidBody must not copy");
static_assert(!std::is_copy_assignable_v<CharacterRigidBody>, "CharacterRigidBody must not copy");
static_assert(!std::is_move_constructible_v<CharacterRigidBody>, "CharacterRigidBody must not move");
static_assert(!std::is_copy_constructible_v<Ragdoll>, "Ragdoll must not copy");
static_assert(!std::is_copy_assignable_v<Ragdoll>, "Ragdoll must not copy");
static_assert(!std::is_move_constructible_v<Ragdoll>, "Ragdoll must not move");
static_assert(std::is_default_constructible_v<Ragdoll>, "Ragdoll stays default constructible");

namespace
{

// Hips, spine, neck, head, then left/right arm chains and leg chains, in a
// T-pose facing +z (left shoulder at +x).
enum Bone : s32
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
    BoneCount
};

class TestSkeleton final : public kinetix::Physics::SkeletonView
{
public:
    u32 boneCount() const override
    {
        return BoneCount;
    }
    s32 boneParent(u32 index) const override
    {
        static const s32 parents[BoneCount] = {-1,     Hips,       Spine,      Neck,     Spine,   LeftArm,
                                               LeftForeArm, Spine,  RightArm,   RightForeArm, Hips,    LeftUpLeg,
                                               LeftLeg, LeftFoot,   Hips,       RightUpLeg, RightLeg, RightFoot};
        return parents[index];
    }
};

void buildRagdoll(Ragdoll& ragdoll, ct::Vector<kinetix::Math::mat4>& pose)
{
    static const kinetix::Math::vec3 positions[BoneCount] = {
        {0.0f, 1.0f, 0.0f},    {0.0f, 1.2f, 0.0f},    {0.0f, 1.5f, 0.0f},   {0.0f, 1.65f, 0.0f},
        {0.2f, 1.45f, 0.0f},   {0.5f, 1.45f, 0.0f},   {0.75f, 1.45f, 0.0f}, {-0.2f, 1.45f, 0.0f},
        {-0.5f, 1.45f, 0.0f},  {-0.75f, 1.45f, 0.0f}, {0.1f, 0.95f, 0.0f},  {0.1f, 0.5f, 0.0f},
        {0.1f, 0.08f, 0.0f},   {0.1f, 0.0f, 0.15f},   {-0.1f, 0.95f, 0.0f}, {-0.1f, 0.5f, 0.0f},
        {-0.1f, 0.08f, 0.0f},  {-0.1f, 0.0f, 0.15f}};
    pose.clear();
    for (const kinetix::Math::vec3& position : positions)
        pose.push_back(kinetix::Math::translate(kinetix::Math::mat4(1.0f), position));

    kinetix::Physics::RagdollBoneMap bones;
    bones.hips = Hips;
    bones.spine = Spine;
    bones.neck = Neck;
    bones.head = Head;
    bones.leftArm = LeftArm;
    bones.leftForeArm = LeftForeArm;
    bones.leftHand = LeftHand;
    bones.rightArm = RightArm;
    bones.rightForeArm = RightForeArm;
    bones.rightHand = RightHand;
    bones.leftUpLeg = LeftUpLeg;
    bones.leftLeg = LeftLeg;
    bones.leftFoot = LeftFoot;
    bones.leftToes = LeftToes;
    bones.rightUpLeg = RightUpLeg;
    bones.rightLeg = RightLeg;
    bones.rightFoot = RightFoot;
    bones.rightToes = RightToes;

    TestSkeleton skeleton;
    CHECK(ragdoll.build(skeleton, bones, pose, 1.0f, kinetix::Physics::RagdollSettings()));
}

void characterRigidBodyLeavesTheWorld()
{
    PhysicsWorld world;
    kinetix::Physics::RigidBody ground;
    ground.setBodyType(kinetix::Physics::BodyType::Static);
    ground.setPosition(kinetix::Math::vec3(0.0f, -0.5f, 0.0f));
    ground.calculateDerivedData();
    kinetix::Physics::BoxShape box(kinetix::Math::vec3(50.0f, 0.5f, 50.0f));
    kinetix::Physics::BodyEntry entry;
    entry.body = &ground;
    entry.shape = &box;
    const u32 groundId = world.addBody(entry);

    {
        CharacterRigidBody character;
        character.addToWorld(world, kinetix::Math::vec3(0.0f, 2.0f, 0.0f));
        CHECK(world.bodyCount() == 2);
    }
    CHECK(world.bodyCount() == 1);
    for (int i = 0; i < 3; ++i)
        world.step(1.0f / 60.0f);

    CharacterRigidBody character;
    character.addToWorld(world, kinetix::Math::vec3(0.0f, 2.0f, 0.0f));
    const u32 firstId = character.bodyId();
    character.addToWorld(world, kinetix::Math::vec3(1.0f, 3.0f, 0.0f));
    CHECK(world.bodyCount() == 2);
    CHECK(world.body(firstId) == nullptr || character.bodyId() == firstId);
    CHECK_NEAR(character.position().x, 1.0f, 1.0e-6f);

    kinetix::Physics::RigidBody other;
    other.setMass(1.0f);
    kinetix::Physics::SphereShape sphere(0.5f);
    kinetix::Physics::BodyEntry otherEntry;
    otherEntry.body = &other;
    otherEntry.shape = &sphere;
    const u32 otherId = world.addBody(otherEntry);
    CHECK(character.bindToBody(world, otherId));
    CHECK(world.bodyCount() == 2);
    character.addToWorld(world, kinetix::Math::vec3(0.0f, 2.0f, 0.0f));
    CHECK(world.bodyCount() == 3);
    CHECK(world.body(otherId) != nullptr);
    CHECK(world.body(groundId) != nullptr);
    character.removeFromWorld();
    CHECK(world.bodyCount() == 2);
    world.step(1.0f / 60.0f);
}

void ragdollLeavesTheWorld()
{
    PhysicsWorld world;
    ct::Vector<kinetix::Math::mat4> pose;
    {
        Ragdoll ragdoll;
        buildRagdoll(ragdoll, pose);
        ragdoll.addToWorld(world, kinetix::Math::mat4(1.0f), pose);
        ragdoll.activate();
        CHECK(world.bodyCount() == 13);
        CHECK(world.jointCount() == 12);
        world.step(1.0f / 60.0f);
    }
    CHECK(world.bodyCount() == 0);
    CHECK(world.jointCount() == 0);
    for (int i = 0; i < 3; ++i)
        world.step(1.0f / 60.0f);
}

// Knee hinge [0, 0.8 pi] about cross(forward, thigh) lets the shin swing backwards only; elbow [0, 0.6 pi] about
// cross(upper arm, forward) lets the forearm come forward only. Measured from the bodies, so independent of HingeJoint's sign.
f32 bendAfterSpin(RagdollPartId parent, RagdollPartId child, const kinetix::Math::vec3& spin,
                  const kinetix::Math::vec3& restChild, const kinetix::Math::vec3& towards)
{
    PhysicsWorld world;
    world.setGravity(kinetix::Math::vec3(0.0f));
    ct::Vector<kinetix::Math::mat4> pose;
    Ragdoll ragdoll;
    buildRagdoll(ragdoll, pose);
    ragdoll.addToWorld(world, kinetix::Math::mat4(1.0f), pose);
    ragdoll.activate();

    kinetix::Physics::RigidBody& upper = *ragdoll.body(parent);
    kinetix::Physics::RigidBody& lower = *ragdoll.body(child);
    const kinetix::Math::quat upper0 = upper.orientation();
    const kinetix::Math::quat lower0 = lower.orientation();
    lower.setAngularVelocity(spin);
    for (int i = 0; i < 30; ++i)
        world.step(1.0f / 60.0f);

    const kinetix::Math::quat lowerTurn = lower.orientation() * kinetix::Math::conjugate(lower0);
    const kinetix::Math::quat upperTurn = upper.orientation() * kinetix::Math::conjugate(upper0);
    const kinetix::Math::vec3 local = kinetix::Math::conjugate(upperTurn) * (lowerTurn * restChild);
    return atan2f(kinetix::Math::dot(local, towards), kinetix::Math::dot(local, restChild));
}

void jointsBendTheAnatomicalWay()
{
    using kinetix::Physics::RagdollPart;
    const kinetix::Math::vec3 down(0.0f, -1.0f, 0.0f);
    const kinetix::Math::vec3 forward(0.0f, 0.0f, 1.0f);
    const kinetix::Math::vec3 backward(0.0f, 0.0f, -1.0f);
    const kinetix::Math::vec3 outward(1.0f, 0.0f, 0.0f);

    // Knee: spin the shin each way about the hinge axis (x, facing +z).
    const f32 kneeBack = bendAfterSpin(RagdollPart::LeftUpperLeg, RagdollPart::LeftLowerLeg,
                                       kinetix::Math::vec3(6.0f, 0.0f, 0.0f), down, backward);
    const f32 kneeForward = bendAfterSpin(RagdollPart::LeftUpperLeg, RagdollPart::LeftLowerLeg,
                                          kinetix::Math::vec3(-6.0f, 0.0f, 0.0f), down, backward);
    CHECK(kneeBack > 0.5f);
    CHECK(kneeForward > -0.1f);

    // Left elbow: the forearm points along +x; bending brings it forward.
    const f32 elbowForward = bendAfterSpin(RagdollPart::LeftUpperArm, RagdollPart::LeftLowerArm,
                                           kinetix::Math::vec3(0.0f, -6.0f, 0.0f), outward, forward);
    const f32 elbowBack = bendAfterSpin(RagdollPart::LeftUpperArm, RagdollPart::LeftLowerArm,
                                        kinetix::Math::vec3(0.0f, 6.0f, 0.0f), outward, forward);
    CHECK(elbowForward > 0.5f);
    CHECK(elbowBack > -0.1f);
}

}

int main()
{
    characterRigidBodyLeavesTheWorld();
    ragdollLeavesTheWorld();
    jointsBendTheAnatomicalWay();
    return TEST_RESULT();
}
