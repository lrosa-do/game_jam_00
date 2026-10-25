#ifndef KINETIX_PHYSICS_DYNAMICS_DISTANCEJOINT_H
#define KINETIX_PHYSICS_DYNAMICS_DISTANCEJOINT_H

#include <kinetix/core/Math.h>
#include "dynamics/Joint.h"

namespace kinetix::Physics
{

class DistanceJoint final : public Joint
{
public:
    DistanceJoint(RigidBody& a, const Math::vec3& worldAnchorA, RigidBody& b,
                  const Math::vec3& worldAnchorB);
    DistanceJoint(RigidBody& a, const Math::vec3& localAnchorA, RigidBody& b,
                  const Math::vec3& localAnchorB, f32 minDistance, f32 maxDistance);

    RigidBody* bodyA() const override;
    RigidBody* bodyB() const override;
    void setup(f32 duration) override;
    void warmStart() override;
    void solveVelocity() override;
    void solvePosition(f32 baumgarte) override;

    bool supportsArticulation() const override
    {
        return true;
    }
    u32 articulationLocks(JointRow* rows, f32* errors) override;
    void addArticulationLockImpulses(const f32* impulses) override;
    u32 articulationMotors(ArticulationMotor* motors) override;
    void setArticulationMotorMass(u32 id, f32 inverseMass, f32 gainInverseMass) override;
    f32 solveArticulationMotor(u32 id, f32 velocity) override;
    u32 articulationPositionRows(JointRow* rows, f32* errors) override;

// Axial force from the last step: positive pushes B away from A (a taut rope reads negative).
    f32 axialForce() const;
    Math::vec3 reactionForce() const override;

    const Math::vec3& localAnchorA() const;
    const Math::vec3& localAnchorB() const;
    Math::vec3 worldAnchorA() const;
    Math::vec3 worldAnchorB() const;
    void setDistance(f32 minDistance, f32 maxDistance);
    f32 minDistance() const;
    f32 maxDistance() const;

private:
    void calculateProperties();
    void applyVelocityImpulse(f32 impulse);

    RigidBody* mBodyA;
    RigidBody* mBodyB;
    Math::vec3 mLocalAnchorA;
    Math::vec3 mLocalAnchorB;
    f32 mMinDistance = 0.0f;
    f32 mMaxDistance = 0.0f;
    Math::vec3 mWorldNormal{0.0f, 1.0f, 0.0f};
    Math::vec3 mArmA{0.0f};
    Math::vec3 mArmB{0.0f};
    f32 mEffectiveMass = 0.0f;
    f32 mMinImpulse = 0.0f;
    f32 mMaxImpulse = 0.0f;
    f32 mTotalImpulse = 0.0f;
    bool mActive = false;
    f32 mPreviousDuration = 0.0f;
};

}

#endif
