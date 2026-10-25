#include "actuator_rig.h"
#include "check.h"

#include <dynamics/ConeTwistJoint.h>
#include <dynamics/HingeJoint.h>
#include <dynamics/JointDrive.h>
#include <dynamics/PistonJoint.h>
#include <dynamics/SliderJoint.h>
#include <dynamics/UniversalJoint.h>

#include <math.h>
#include <stdio.h>

using namespace actuator_test;

namespace
{

constexpr f32 kArmMass = 10.0f;
constexpr f32 kArmLength = 1.0f;
const KM::vec3 kArmHalf(0.5f * kArmLength, 0.05f, 0.05f);

// An arm hinged at the origin about +Z, lying along +X: a positive angle
// lifts it towards +Y.
struct Arm
{
    Rig rig;
    RigidBody* base = nullptr;
    RigidBody* arm = nullptr;
    HingeJoint* hinge = nullptr;

    explicit Arm(bool gravity, f32 hz = 60.0f) : rig(hz, gravity)
    {
        base = &rig.base(KM::vec3(0.0f));
        arm = &rig.box(KM::vec3(0.5f * kArmLength, 0.0f, 0.0f), kArmHalf, kArmMass);
        hinge = new HingeJoint(*base, *arm, KM::vec3(0.0f), KM::vec3(0.0f, 0.0f, 1.0f));
        rig.world.addJoint(hinge);
    }
    ~Arm()
    {
        rig.world.removeJoint(hinge);
        delete hinge;
    }
};

f32 pivotInertia()
{
    return kArmMass / 12.0f * (kArmLength * kArmLength + 0.01f) +
           kArmMass * 0.25f * kArmLength * kArmLength;
}

ServoSettings servo(f32 frequency, f32 dampingRatio, f32 maxTorque, f32 maxVelocity = 0.0f)
{
    ServoSettings settings;
    settings.frequency = frequency;
    settings.dampingRatio = dampingRatio;
    settings.maxForce = maxTorque;
    settings.maxVelocity = maxVelocity;
    return settings;
}

f32 stepOvershoot(f32 dampingRatio, f32* settledError = nullptr, f32* errorAt1s5 = nullptr)
{
    Arm a(false);
    a.hinge->setServo(1.0f, servo(1.0f, dampingRatio, 1000.0f));
    f32 peak = 0.0f;
    for (int i = 0; i < 4 * 60; ++i)
    {
        a.rig.world.step(a.rig.step);
        peak = fmaxf(peak, a.hinge->currentAngle());
        if (i == 89 && errorAt1s5)
            *errorAt1s5 = fabsf(a.hinge->currentAngle() - 1.0f);
    }
    if (settledError)
        *settledError = fabsf(a.hinge->currentAngle() - 1.0f);
    return peak - 1.0f;
}

void stepResponseFollowsDampingRatio()
{
    f32 settled = 0.0f;
    f32 at1s5 = 0.0f;
    const f32 critical = stepOvershoot(1.0f, &settled, &at1s5);
    printf("step: zeta 1 overshoot %.4f%%, error at 1.5 s %.5f, at 4 s %.6f rad\n",
           double(critical * 100.0f), double(at1s5), double(settled));
    CHECK(critical < 0.005f);
    // Critically damped at 1 Hz: (1 + w t) e^(-w t) is 0.08% at 1.5 s.
    CHECK(at1s5 < 0.005f);
    CHECK(settled < 1.0e-3f);

    // An underdamped servo overshoots like the mass-spring it is, a little
    // less for the implicit integrator's own damping:
    // exp(-pi zeta / sqrt(1 - zeta^2)) is 37.2% at 0.3 and 16.3% at 0.5.
    const f32 overshoot03 = stepOvershoot(0.3f);
    const f32 overshoot05 = stepOvershoot(0.5f);
    printf("step: zeta 0.3 overshoot %.1f%% (37.2%%), zeta 0.5 %.1f%% (16.3%%)\n",
           double(overshoot03 * 100.0f), double(overshoot05 * 100.0f));
    CHECK(overshoot03 > 0.30f && overshoot03 < 0.38f);
    CHECK(overshoot05 > 0.13f && overshoot05 < 0.17f);
}

// The natural frequency is the arm's about its pivot, not about its centre
// of mass: the period of the undamped servo is 1/f.
void naturalFrequencyUsesPivotInertia()
{
    Arm a(false);
    // An undamped swing this slow is what the world's sleep threshold puts
    // to rest within a few periods; this measures the swing, not that.
    a.arm->setCanSleep(false);
    a.hinge->setServo(0.2f, servo(1.0f, 0.0f, 1000.0f));
    f32 angles[4 * 60];
    for (int i = 0; i < 4 * 60; ++i)
    {
        a.rig.world.step(a.rig.step);
        angles[i] = a.hinge->currentAngle();
    }
    f32 peaks[8];
    int count = 0;
    for (int i = 1; i + 1 < 4 * 60 && count < 8; ++i)
        if (angles[i] > angles[i - 1] && angles[i] >= angles[i + 1])
        {
            const f32 curvature = angles[i - 1] - 2.0f * angles[i] + angles[i + 1];
            const f32 offset = curvature != 0.0f ? 0.5f * (angles[i - 1] - angles[i + 1]) / curvature : 0.0f;
            peaks[count++] = (static_cast<f32>(i) + offset) * a.rig.step;
        }
    const f32 period = count > 1 ? (peaks[count - 1] - peaks[0]) / static_cast<f32>(count - 1) : 0.0f;
    // The implicit integrator's own discrete frequency at h*w = 0.105 is
    // 0.9964 Hz.
    printf("servo period %.4f s at 1 Hz (%d peaks)\n", double(period), count);
    CHECK(count >= 3);
    CHECK_NEAR(period, 1.0036f, 0.01f);
}

// Holding the arm out horizontally against its weight: the steady error of
// a PD servo is the load over its stiffness, k * e = m g L cos(e) with
// k = I_pivot w^2; a rigid servo has none while its torque suffices.
void steadyStateUnderGravity()
{
    {
        Arm a(true);
        a.hinge->setServo(0.0f, servo(2.0f, 1.0f, 500.0f));
        a.rig.run(5.0f);
        const f32 omega = 2.0f * kPi * 2.0f;
        const f32 k = pivotInertia() * omega * omega;
        const f32 load = kArmMass * kG * 0.5f * kArmLength;
        f32 expected = -load / k;
        for (int i = 0; i < 8; ++i)
            expected = -load * cosf(expected) / k;
        printf("soft hold: %.4f rad, expected %.4f\n", double(a.hinge->currentAngle()),
               double(expected));
        CHECK_NEAR(a.hinge->currentAngle(), expected, 0.03f * fabsf(expected));
    }
    {
        Arm a(true);
        ServoSettings rigid;
        rigid.maxForce = 60.0f;
        a.hinge->setServo(0.3f, rigid);
        a.rig.run(4.0f);
        printf("rigid hold at 0.3 rad: error %.2e rad, torque %.2f N m\n",
               double(a.hinge->currentAngle() - 0.3f), double(a.hinge->motorTorque()));
        CHECK_NEAR(a.hinge->currentAngle(), 0.3f, 1.0e-3f);
    }
}

// Not enough torque: from hanging straight down the servo can only lift the
// arm until m g L |cos(theta)| equals its limit, and stays saturated there.
void saturatesAtTorqueLimit()
{
    Arm a(true);
    a.hinge->setServo(-0.5f * kPi, servo(1.0f, 1.0f, 1000.0f));
    a.rig.run(5.0f);
    CHECK_NEAR(a.hinge->currentAngle(), -0.5f * kPi, 0.01f);

    // Saturated: constant torque, the arm a pendulum about the angle where gravity balances it.
    // Kept awake: a swing this slow is otherwise frozen by the sleep threshold.
    const f32 limit = 30.0f;
    a.arm->setDamping(0.3f, 0.3f);
    a.arm->setCanSleep(false);
    a.hinge->setServo(0.0f, servo(2.0f, 1.0f, limit));
    f32 highest = -10.0f;
    for (int i = 0; i < 12 * 60; ++i)
    {
        a.rig.world.step(a.rig.step);
        highest = fmaxf(highest, a.hinge->currentAngle());
    }
    const f32 load = kArmMass * kG * 0.5f * kArmLength;
    const f32 expected = -acosf(limit / load);
    printf("saturated at %.4f rad (expected %.4f), torque %.3f N m\n",
           double(a.hinge->currentAngle()), double(expected), double(a.hinge->motorTorque()));
    CHECK_NEAR(a.hinge->currentAngle(), expected, 0.02f);
    CHECK_NEAR(a.hinge->motorTorque(), limit, 0.01f * limit);
    CHECK(a.hinge->drive().saturated());
    printf("highest angle reached %.3f rad\n", double(highest));
    CHECK(highest < -0.2f);
}

void respectsMaxVelocity()
{
    const f32 frequencies[] = {0.0f, 5.0f};
    for (f32 frequency : frequencies)
    {
        Arm a(false);
        a.hinge->setServo(1.0f, servo(frequency, 1.0f, 1000.0f, 0.5f));
        f32 fastest = 0.0f;
        f32 reached = -1.0f;
        for (int i = 0; i < 5 * 60; ++i)
        {
            a.rig.world.step(a.rig.step);
            fastest = fmaxf(fastest, fabsf(a.hinge->currentAngularVelocity()));
            if (reached < 0.0f && fabsf(a.hinge->currentAngle() - 1.0f) < 0.01f)
                reached = (i + 1) * a.rig.step;
        }
        printf("max velocity 0.5 rad/s, f %.0f Hz: fastest %.4f rad/s, within 0.01 at %.2f s\n",
               double(frequency), double(fastest), double(reached));
        CHECK(fastest <= 0.5f * 1.005f);
        CHECK(reached > 1.8f && reached < 2.6f);
        CHECK_NEAR(a.hinge->currentAngle(), 1.0f, 2.0e-3f);
    }

    Arm a(true);
    a.hinge->setServo(0.5f, servo(4.0f, 1.0f, 1000.0f));
    a.rig.run(3.0f);
    a.hinge->setServo(-1.2f, servo(4.0f, 1.0f, 1000.0f, 0.4f));
    f32 fastest = 0.0f;
    for (int i = 0; i < 6 * 60; ++i)
    {
        a.rig.world.step(a.rig.step);
        fastest = fmaxf(fastest, fabsf(a.hinge->currentAngularVelocity()));
    }
    printf("lowering under gravity at 0.4 rad/s: fastest %.4f\n", double(fastest));
    CHECK(fastest <= 0.4f * 1.01f);
    CHECK_NEAR(a.hinge->currentAngle(), -1.2f, 0.02f);
}

// Across +-pi the short way: from 3.0 rad to -3.0 rad is 0.28 rad through
// pi, not 6 rad back through zero.
void wrapsTheShortWay()
{
    Arm a(false);
    ServoSettings rigid;
    rigid.maxForce = 1000.0f;
    rigid.maxVelocity = 2.0f;
    a.hinge->setServo(3.0f, rigid);
    a.rig.run(3.0f);
    CHECK_NEAR(a.hinge->currentAngle(), 3.0f, 1.0e-3f);

    a.hinge->setServoTarget(-3.0f);
    f32 smallestMagnitude = 10.0f;
    for (int i = 0; i < 2 * 60; ++i)
    {
        a.rig.world.step(a.rig.step);
        smallestMagnitude = fminf(smallestMagnitude, fabsf(a.hinge->currentAngle()));
    }
    printf("wrap: ended at %.4f, never nearer zero than %.3f rad\n",
           double(a.hinge->currentAngle()), double(smallestMagnitude));
    CHECK(smallestMagnitude > 2.9f);
    CHECK_NEAR(a.hinge->currentAngle(), -3.0f, 1.0e-3f);

    Arm b(false);
    b.hinge->setLimits(-3.1f, 3.1f);
    b.hinge->setServo(3.0f, rigid);
    b.rig.run(3.0f);
    b.hinge->setServoTarget(-3.0f);
    b.rig.run(5.0f);
    CHECK_NEAR(b.hinge->currentAngle(), -3.0f, 2.0e-3f);
}

void stableAtVeryHighStiffness()
{
    for (int variant = 0; variant < 2; ++variant)
    {
        Arm a(true);
        ServoSettings settings;
        settings.maxForce = 1.0e5f;
        if (variant == 0)
        {
            settings.frequency = 1000.0f;
            settings.dampingRatio = 0.1f;
        }
        else
        {
            settings.stiffness = 1.0e9f;
            settings.damping = 0.0f;
        }
        a.hinge->setServo(0.7f, settings);
        f32 worstLate = 0.0f;
        bool finite = true;
        for (int i = 0; i < 10 * 60; ++i)
        {
            a.rig.world.step(a.rig.step);
            finite = finite && finiteVec(a.arm->position()) && finiteVec(a.arm->angularVelocity());
            if (i > 60)
                worstLate = fmaxf(worstLate, fabsf(a.hinge->currentAngle() - 0.7f));
        }
        printf("stiff servo (%s): worst error after 1 s %.2e rad, final speed %.2e rad/s\n",
               variant == 0 ? "1 kHz" : "1e9 N m/rad", double(worstLate),
               double(a.hinge->currentAngularVelocity()));
        CHECK(finite);
        CHECK(worstLate < 1.0e-3f);
        CHECK(fabsf(a.hinge->currentAngularVelocity()) < 1.0e-3f);
    }
}

// Feed-forward: the setpoint is the target at the start of the step, moving at the target rate.
void tracksAMovingTarget()
{
    for (int variant = 0; variant < 2; ++variant)
    {
        Arm a(true);
        const ServoSettings settings = variant == 0 ? servo(0.0f, 1.0f, 500.0f) : servo(8.0f, 1.0f, 500.0f);
        a.hinge->setServo(0.0f, settings);
        f32 worst = 0.0f;
        for (int i = 0; i < 6 * 60; ++i)
        {
            const f32 t = i * a.rig.step;
            a.hinge->setServoTarget(0.6f * sinf(2.0f * t), 1.2f * cosf(2.0f * t));
            a.rig.world.step(a.rig.step);
            const f32 reached = 0.6f * sinf(2.0f * (t + a.rig.step));
            if (t > 1.0f)
                worst = fmaxf(worst, fabsf(a.hinge->currentAngle() - reached));
        }
        printf("tracking a sine (%s): worst error %.4f rad\n", variant == 0 ? "rigid" : "8 Hz",
               double(worst));
        CHECK(worst < (variant == 0 ? 2.0e-3f : 0.03f));
    }
}

// Soft servos sag by what the outer links weigh (each joint's frequency scales with its own two bodies' inertia).
// Swinging at 1.5 rad/s the hinges open by the free-swing amount (1.8 cm, joint accuracy at 60 Hz).
void chainOfServos()
{
    for (int variant = 0; variant < 2; ++variant)
    {
        Rig rig(60.0f, true);
        RigidBody& base = rig.base(KM::vec3(0.0f));
        RigidBody* links[3];
        HingeJoint* hinges[3];
        KM::vec3 localParent[3];
        KM::vec3 localChild[3];
        const f32 masses[3] = {40.0f, 20.0f, 10.0f};
        for (u32 i = 0; i < 3; ++i)
        {
            const KM::vec3 pivot(static_cast<f32>(i), 0.0f, 0.0f);
            links[i] = &rig.box(pivot + KM::vec3(0.5f, 0.0f, 0.0f), KM::vec3(0.5f, 0.05f, 0.05f), masses[i]);
            RigidBody& parent = i == 0 ? base : *links[i - 1];
            hinges[i] = new HingeJoint(parent, *links[i], pivot, KM::vec3(0.0f, 0.0f, 1.0f));
            rig.world.addJoint(hinges[i]);
            localParent[i] = parent.pointToLocal(pivot);
            localChild[i] = links[i]->pointToLocal(pivot);
            ServoSettings settings = variant == 0 ? servo(0.0f, 1.0f, 2000.0f, 1.5f) : servo(8.0f, 1.0f, 2000.0f, 1.5f);
            hinges[i]->setServo(0.0f, settings);
        }
        const f32 poses[2][3] = {{0.6f, -0.9f, 0.7f}, {-0.4f, 0.5f, -1.0f}};
        f32 worstSeparation = 0.0f;
        bool finite = true;
        for (u32 pose = 0; pose < 2; ++pose)
        {
            for (u32 i = 0; i < 3; ++i)
                hinges[i]->setServoTarget(poses[pose][i]);
            for (int step = 0; step < 4 * 60; ++step)
            {
                rig.world.step(rig.step);
                for (u32 i = 0; i < 3; ++i)
                {
                    const RigidBody& parent = i == 0 ? base : *links[i - 1];
                    worstSeparation = fmaxf(worstSeparation, KM::length(parent.pointToWorld(localParent[i]) -
                                                                         links[i]->pointToWorld(localChild[i])));
                    finite = finite && finiteVec(links[i]->position());
                }
            }
            for (u32 i = 0; i < 3; ++i)
                CHECK_NEAR(hinges[i]->currentAngle(), poses[pose][i], variant == 0 ? 2.0e-3f : 0.08f);
        }
        f32 holding = 0.0f;
        for (u32 i = 0; i < 3; ++i)
        {
            const RigidBody& parent = i == 0 ? base : *links[i - 1];
            holding = fmaxf(holding, KM::length(parent.pointToWorld(localParent[i]) -
                                                links[i]->pointToWorld(localChild[i])));
        }
        printf("chain of %s servos: angles %.4f %.4f %.4f, worst hinge separation %.2e m, %.2e m "
               "holding\n",
               variant == 0 ? "rigid" : "8 Hz", double(hinges[0]->currentAngle()),
               double(hinges[1]->currentAngle()), double(hinges[2]->currentAngle()),
               double(worstSeparation), double(holding));
        CHECK(finite);
        CHECK(worstSeparation < 2.0e-2f);
        CHECK(holding < 1.0e-3f);
        for (u32 i = 0; i < 3; ++i)
        {
            rig.world.removeJoint(hinges[i]);
            delete hinges[i];
        }
    }
}

void motorAndServoExclusive()
{
    Arm a(false);
    a.hinge->setServo(1.0f, servo(1.0f, 1.0f, 100.0f));
    CHECK(a.hinge->drive().mode() == JointDrive::Mode::Position);
    a.hinge->setMotor(2.0f, 100.0f);
    CHECK(a.hinge->drive().mode() == JointDrive::Mode::Velocity);
    a.rig.run(1.0f);
    CHECK_NEAR(a.hinge->currentAngularVelocity(), 2.0f, 1.0e-3f);
    a.hinge->setServo(0.0f, servo(1.0f, 1.0f, 100.0f));
    CHECK(a.hinge->drive().mode() == JointDrive::Mode::Position);
    a.hinge->disableServo();
    CHECK(!a.hinge->drive().enabled());
    a.hinge->setServo(0.5f, servo(1.0f, 1.0f, 0.0f));
    CHECK(!a.hinge->drive().enabled());
}

void sliderServo()
{
    Rig rig(60.0f, true);
    RigidBody& base = rig.base(KM::vec3(0.0f));
    RigidBody& carriage = rig.box(KM::vec3(0.0f), KM::vec3(0.2f), 5.0f);
    SliderJoint slider(base, carriage, KM::vec3(0.0f), KM::vec3(1.0f, 0.0f, 0.0f));
    slider.setLimits(-1.0f, 1.0f);
    rig.world.addJoint(&slider);
    slider.setServo(0.5f, servo(2.0f, 1.0f, 500.0f));
    f32 peak = 0.0f;
    for (int i = 0; i < 4 * 60; ++i)
    {
        rig.world.step(rig.step);
        peak = fmaxf(peak, slider.currentPosition());
    }
    printf("slider: %.5f m (0.5), peak %.5f\n", double(slider.currentPosition()), double(peak));
    CHECK_NEAR(slider.currentPosition(), 0.5f, 1.0e-3f);
    CHECK(peak < 0.5f * 1.005f);
    slider.setServoTarget(3.0f);
    rig.run(4.0f);
    CHECK_NEAR(slider.currentPosition(), 1.0f, 2.0e-3f);
    rig.world.removeJoint(&slider);
}

void pistonServo()
{
    Rig rig(60.0f, true);
    RigidBody& base = rig.base(KM::vec3(0.0f));
    RigidBody& rod = rig.box(KM::vec3(0.0f, -0.5f, 0.0f), KM::vec3(0.05f, 0.5f, 0.05f), 4.0f);
    PistonJoint piston(base, rod, KM::vec3(0.0f), KM::vec3(0.0f, 1.0f, 0.0f));
    rig.world.addJoint(&piston);
    piston.setLinearServo(-0.3f, servo(2.0f, 1.0f, 500.0f));
    piston.setAngularServo(1.0f, servo(2.0f, 1.0f, 50.0f));
    rig.run(4.0f);
    printf("piston: %.5f m (-0.3), %.5f rad (1.0)\n", double(piston.currentPosition()),
           double(piston.currentAngle()));
    CHECK_NEAR(piston.currentPosition(), -0.3f - 4.0f * kG / (4.0f * 16.0f * kPi * kPi), 2.0e-3f);
    CHECK_NEAR(piston.currentAngle(), 1.0f, 2.0e-3f);
    CHECK(piston.linearDrive().mode() == JointDrive::Mode::Position);
    piston.setLinearMotor(0.0f, 100.0f);
    CHECK(piston.linearDrive().mode() == JointDrive::Mode::Velocity);
    rig.world.removeJoint(&piston);
}

void universalServo()
{
    Rig rig(60.0f, false);
    RigidBody& base = rig.base(KM::vec3(0.0f));
    RigidBody& shaft = rig.box(KM::vec3(0.0f, 0.0f, 0.5f), KM::vec3(0.05f, 0.05f, 0.5f), 2.0f);
    UniversalJoint joint(base, shaft, KM::vec3(0.0f), KM::vec3(1.0f, 0.0f, 0.0f),
                         KM::vec3(0.0f, 1.0f, 0.0f));
    rig.world.addJoint(&joint);
    joint.setServoA(0.4f, servo(2.0f, 1.0f, 100.0f));
    joint.setServoB(-0.3f, servo(2.0f, 1.0f, 100.0f));
    rig.run(4.0f);
    printf("universal: %.4f (0.4), %.4f (-0.3)\n", double(joint.currentAngleA()),
           double(joint.currentAngleB()));
    CHECK_NEAR(joint.currentAngleA(), 0.4f, 0.01f);
    CHECK_NEAR(joint.currentAngleB(), -0.3f, 0.01f);
    rig.world.removeJoint(&joint);
}

void coneTwistServo()
{
    Rig rig(60.0f, true);
    RigidBody& base = rig.base(KM::vec3(0.0f));
    RigidBody& limb = rig.box(KM::vec3(0.0f, -0.5f, 0.0f), KM::vec3(0.06f, 0.5f, 0.08f), 3.0f);
    ConeTwistJoint joint(base, limb, KM::vec3(0.0f), KM::vec3(0.0f, -1.0f, 0.0f),
                         KM::vec3(0.0f, 0.0f, 1.0f));
    rig.world.addJoint(&joint);
    CHECK_NEAR(joint.twistAngle(), 0.0f, 1.0e-5f);
    const KM::quat target = ConeTwistJoint::orientationFromTwistSwing(0.5f, 0.3f, -0.2f);
    joint.setServo(target, servo(3.0f, 1.0f, 200.0f));
    rig.run(4.0f);
    const KM::vec2 swing = joint.swingAngles();
    printf("cone-twist: twist %.4f (0.5), swing %.4f %.4f (0.3 -0.2), error %.2e\n",
           double(joint.twistAngle()), double(swing.x), double(swing.y),
           double(KM::length(joint.servoError())));
    CHECK_NEAR(joint.twistAngle(), 0.5f, 0.02f);
    CHECK_NEAR(swing.x, 0.3f, 0.02f);
    CHECK_NEAR(swing.y, -0.2f, 0.02f);

    ServoSettings rigid;
    rigid.maxForce = 200.0f;
    joint.setServo(target, rigid);
    rig.run(2.0f);
    CHECK(KM::length(joint.servoError()) < 2.0e-3f);
    CHECK(finiteVec(limb.position()));
    rig.world.removeJoint(&joint);
}

}

int main()
{
    stepResponseFollowsDampingRatio();
    naturalFrequencyUsesPivotInertia();
    steadyStateUnderGravity();
    saturatesAtTorqueLimit();
    respectsMaxVelocity();
    wrapsTheShortWay();
    stableAtVeryHighStiffness();
    tracksAMovingTarget();
    chainOfServos();
    motorAndServoExclusive();
    sliderServo();
    pistonServo();
    universalServo();
    coneTwistServo();
    return TEST_RESULT();
}
