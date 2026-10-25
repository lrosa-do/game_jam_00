#include "actuator_rig.h"
#include "check.h"

#include <dynamics/ConeTwistJoint.h>
#include <dynamics/DistanceJoint.h>
#include <dynamics/FixedJoint.h>
#include <dynamics/HingeJoint.h>
#include <dynamics/PistonJoint.h>
#include <dynamics/PointJoint.h>
#include <dynamics/SliderJoint.h>
#include <dynamics/UniversalJoint.h>

#include <math.h>
#include <stdio.h>

using namespace actuator_test;

namespace
{

constexpr f32 kMass = 7.0f;

template <typename MakeJoint>
void hangingMassReadsWeight(const char* name, MakeJoint make)
{
    Rig rig(60.0f, true);
    RigidBody& base = rig.base(KM::vec3(0.0f));
    RigidBody& load = rig.box(KM::vec3(0.0f, -1.0f, 0.0f), KM::vec3(0.1f), kMass);
    Joint* joint = make(base, load);
    rig.world.addJoint(joint);
    rig.run(2.0f);
    const KM::vec3 force = joint->reactionForce();
    const KM::vec3 torque = joint->reactionTorque();
    printf("%-10s reaction (%.3f, %.3f, %.3f) N, torque %.2e N m, m g = %.3f\n", name,
           double(force.x), double(force.y), double(force.z), double(KM::length(torque)),
           double(kMass * kG));
    CHECK_NEAR(force.y, kMass * kG, 0.01f * kMass * kG);
    CHECK(fabsf(force.x) < 1.0e-3f * kMass * kG && fabsf(force.z) < 1.0e-3f * kMass * kG);
    CHECK(KM::length(torque) < 1.0e-2f);
    rig.world.removeJoint(joint);
    delete joint;
}

void everyJointReadsAHangingWeight()
{
    const KM::vec3 anchor(0.0f);
    const KM::vec3 up(0.0f, 1.0f, 0.0f);
    hangingMassReadsWeight("hinge", [&](RigidBody& a, RigidBody& b) -> Joint*
                           {
                               return new HingeJoint(a, b, anchor, KM::vec3(0.0f, 0.0f, 1.0f));
                           });
    hangingMassReadsWeight("point", [&](RigidBody& a, RigidBody& b) -> Joint*
                           {
                               return new PointJoint(a, b, anchor);
                           });
    hangingMassReadsWeight("fixed", [&](RigidBody& a, RigidBody& b) -> Joint*
                           {
                               return new FixedJoint(a, b, anchor);
                           });
    hangingMassReadsWeight("universal", [&](RigidBody& a, RigidBody& b) -> Joint*
                           {
                               return new UniversalJoint(a, b, anchor, KM::vec3(1.0f, 0.0f, 0.0f),
                                                         KM::vec3(0.0f, 0.0f, 1.0f));
                           });
    hangingMassReadsWeight("cone-twist", [&](RigidBody& a, RigidBody& b) -> Joint*
                           {
                               return new ConeTwistJoint(a, b, anchor, -up, KM::vec3(0.0f, 0.0f, 1.0f));
                           });
    // The slider and the piston hold along their axis with a locked motor
    // (velocity zero); the distance joint is a rope.
    hangingMassReadsWeight("slider", [&](RigidBody& a, RigidBody& b) -> Joint*
                           {
                               SliderJoint* joint = new SliderJoint(a, b, anchor, up);
                               joint->setMotor(0.0f, 1000.0f);
                               return joint;
                           });
    hangingMassReadsWeight("piston", [&](RigidBody& a, RigidBody& b) -> Joint*
                           {
                               PistonJoint* joint = new PistonJoint(a, b, anchor, up);
                               joint->setLinearMotor(0.0f, 1000.0f);
                               return joint;
                           });
    hangingMassReadsWeight("distance", [&](RigidBody& a, RigidBody& b) -> Joint*
                           {
                               return new DistanceJoint(a, anchor, b, KM::vec3(0.0f, -1.0f, 0.0f));
                           });
}

// The arm of 3d_actuators_servo held out at 0.5 rad by a rigid servo: the
// servo carries the whole moment of the weight about the pivot.
void servoTorqueReadsTheMoment()
{
    Rig rig(60.0f, true);
    RigidBody& base = rig.base(KM::vec3(0.0f));
    RigidBody& arm = rig.box(KM::vec3(0.5f, 0.0f, 0.0f), KM::vec3(0.5f, 0.05f, 0.05f), 10.0f);
    HingeJoint hinge(base, arm, KM::vec3(0.0f), KM::vec3(0.0f, 0.0f, 1.0f));
    rig.world.addJoint(&hinge);
    ServoSettings rigid;
    rigid.maxForce = 500.0f;
    hinge.setServo(0.5f, rigid);
    rig.run(3.0f);
    const f32 expected = 10.0f * kG * 0.5f * cosf(hinge.currentAngle());
    printf("servo torque %.4f N m, m g L cos = %.4f; reaction torque z %.4f; force y %.3f\n",
           double(hinge.motorTorque()), double(expected), double(hinge.reactionTorque().z),
           double(hinge.reactionForce().y));
    CHECK_NEAR(hinge.motorTorque(), expected, 0.02f * expected);
    CHECK_NEAR(hinge.reactionTorque().z, expected, 0.02f * expected);
    CHECK_NEAR(hinge.reactionForce().y, 10.0f * kG, 0.01f * 10.0f * kG);
    CHECK_NEAR(hinge.drive().appliedForce(), hinge.motorTorque(), 1.0e-6f);
    CHECK(fabsf(hinge.limitTorque()) < 1.0e-6f);
    rig.world.removeJoint(&hinge);
}

// Resting on a limit, the limit row carries what the servo did above: the
// weight's moment, pushing up (towards a larger angle).
void limitRowReadsTheLoad()
{
    {
        Rig rig(60.0f, true);
        RigidBody& base = rig.base(KM::vec3(0.0f));
        RigidBody& arm = rig.box(KM::vec3(0.5f, 0.0f, 0.0f), KM::vec3(0.5f, 0.05f, 0.05f), 10.0f);
        HingeJoint hinge(base, arm, KM::vec3(0.0f), KM::vec3(0.0f, 0.0f, 1.0f));
        hinge.setLimits(0.0f, 1.0f);
        rig.world.addJoint(&hinge);
        rig.run(2.0f);
        const f32 expected = 10.0f * kG * 0.5f;
        printf("hinge limit torque %.4f N m (%.4f), angle %.2e\n", double(hinge.limitTorque()),
               double(expected), double(hinge.currentAngle()));
        CHECK_NEAR(hinge.limitTorque(), expected, 0.02f * expected);
        CHECK_NEAR(hinge.reactionTorque().z, expected, 0.02f * expected);
        rig.world.removeJoint(&hinge);
    }
    {
        Rig rig(60.0f, true);
        RigidBody& base = rig.base(KM::vec3(0.0f));
        RigidBody& carriage = rig.box(KM::vec3(0.0f), KM::vec3(0.2f), kMass);
        SliderJoint slider(base, carriage, KM::vec3(0.0f), KM::vec3(0.0f, 1.0f, 0.0f));
        slider.setLimits(0.0f, 1.0f);
        rig.world.addJoint(&slider);
        rig.run(2.0f);
        printf("slider limit force %.4f N (%.4f), position %.2e\n", double(slider.limitForce()),
               double(kMass * kG), double(slider.currentPosition()));
        CHECK_NEAR(slider.limitForce(), kMass * kG, 0.01f * kMass * kG);
        CHECK(fabsf(slider.motorForce()) < 1.0e-6f);
        slider.setMotor(0.25f, 1000.0f);
        rig.run(1.0f);
        printf("slider moving: motor force %.4f N, limit %.4f N, %.4f m/s\n",
               double(slider.motorForce()), double(slider.limitForce()),
               double(slider.currentVelocity()));
        CHECK_NEAR(slider.motorForce(), kMass * kG, 0.01f * kMass * kG);
        CHECK(fabsf(slider.limitForce()) < 1.0e-3f);
        CHECK_NEAR(slider.currentVelocity(), 0.25f, 1.0e-3f);
        rig.world.removeJoint(&slider);
    }
}

// Rows that hold an axis read the moment they hold. An arm on a hinge whose
// axis runs along the arm can only spin about itself; its weight's moment
// is all in the two rotation-lock rows. A fixed cantilever carries both.
void lockRowsReadTheirMoments()
{
    {
        Rig rig(60.0f, true);
        RigidBody& base = rig.base(KM::vec3(0.0f));
        RigidBody& arm = rig.box(KM::vec3(0.5f, 0.0f, 0.0f), KM::vec3(0.5f, 0.05f, 0.05f), 10.0f);
        HingeJoint hinge(base, arm, KM::vec3(0.0f), KM::vec3(1.0f, 0.0f, 0.0f));
        rig.world.addJoint(&hinge);
        rig.run(2.0f);
        const KM::vec3 lock = hinge.lockTorque();
        printf("hinge lock torque (%.4f, %.4f, %.4f), m g L = %.4f\n", double(lock.x),
               double(lock.y), double(lock.z), double(10.0f * kG * 0.5f));
        CHECK_NEAR(lock.z, 10.0f * kG * 0.5f, 0.02f * 10.0f * kG * 0.5f);
        CHECK(fabsf(lock.x) < 1.0e-3f);
        CHECK_NEAR(hinge.lockForce().y, 10.0f * kG, 0.01f * 10.0f * kG);
        rig.world.removeJoint(&hinge);
    }
    {
        Rig rig(60.0f, true);
        RigidBody& base = rig.base(KM::vec3(0.0f));
        RigidBody& arm = rig.box(KM::vec3(0.5f, 0.0f, 0.0f), KM::vec3(0.5f, 0.05f, 0.05f), 10.0f);
        FixedJoint fixed(base, arm, KM::vec3(0.0f));
        rig.world.addJoint(&fixed);
        rig.run(2.0f);
        printf("cantilever: force y %.4f, torque z %.4f\n", double(fixed.reactionForce().y),
               double(fixed.reactionTorque().z));
        CHECK_NEAR(fixed.reactionForce().y, 10.0f * kG, 0.01f * 10.0f * kG);
        CHECK_NEAR(fixed.reactionTorque().z, 10.0f * kG * 0.5f, 0.02f * 10.0f * kG * 0.5f);
        rig.world.removeJoint(&fixed);
    }
}

void velocityReadouts()
{
    Rig rig(60.0f, false);
    RigidBody& base = rig.base(KM::vec3(0.0f));
    RigidBody& rod = rig.box(KM::vec3(0.0f), KM::vec3(0.05f, 0.5f, 0.05f), 2.0f);
    PistonJoint piston(base, rod, KM::vec3(0.0f), KM::vec3(0.0f, 1.0f, 0.0f));
    rig.world.addJoint(&piston);
    piston.setLinearMotor(-0.3f, 100.0f);
    piston.setAngularMotor(1.5f, 100.0f);
    rig.run(1.0f);
    CHECK_NEAR(piston.currentVelocity(), -0.3f, 1.0e-4f);
    CHECK_NEAR(piston.currentAngularVelocity(), 1.5f, 1.0e-4f);
    CHECK_NEAR(piston.currentPosition(), -0.3f, 0.02f);
    CHECK(fabsf(piston.linearMotorForce()) < 1.0e-3f);
    CHECK(fabsf(piston.angularMotorTorque()) < 1.0e-3f);
    rig.world.removeJoint(&piston);
}

}

int main()
{
    everyJointReadsAHangingWeight();
    servoTorqueReadsTheMoment();
    limitRowReadsTheLoad();
    lockRowsReadTheirMoments();
    velocityReadouts();
    return TEST_RESULT();
}
