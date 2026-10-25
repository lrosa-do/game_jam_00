#include "actuator_rig.h"
#include "check.h"

#include <actuators/ElectricMotor.h>
#include <actuators/Hook.h>
#include <actuators/HydraulicCylinder.h>
#include <actuators/Rope.h>
#include <dynamics/HingeJoint.h>
#include <dynamics/SliderJoint.h>

#include <math.h>
#include <stdio.h>

using namespace actuator_test;

namespace
{

// Steps until `body` sleeps; the time it took, or a negative number.
f32 timeToSleep(Rig& rig, const RigidBody& body, f32 limit)
{
    const int steps = static_cast<int>(limit / rig.step);
    for (int i = 0; i < steps; ++i)
    {
        rig.world.step(rig.step);
        if (!body.awake())
            return (i + 1) * rig.step;
    }
    return -1.0f;
}

bool staysAwake(Rig& rig, const RigidBody& body, f32 seconds)
{
    const int steps = static_cast<int>(seconds / rig.step);
    bool awake = true;
    for (int i = 0; i < steps; ++i)
    {
        rig.world.step(rig.step);
        awake = awake && body.awake();
    }
    return awake;
}

void servoSleepsWhenHoldingAndWakesForATarget()
{
    for (int soft = 0; soft < 2; ++soft)
    {
        Rig rig(60.0f, true);
        RigidBody& base = rig.base(KM::vec3(0.0f));
        RigidBody& arm = rig.box(KM::vec3(0.5f, 0.0f, 0.0f), KM::vec3(0.5f, 0.05f, 0.05f), 10.0f);
        HingeJoint hinge(base, arm, KM::vec3(0.0f), KM::vec3(0.0f, 0.0f, 1.0f));
        rig.world.addJoint(&hinge);
        ServoSettings settings;
        settings.maxForce = 500.0f;
        settings.maxVelocity = 0.3f; // slow: under the sleep threshold
        if (soft)
        {
            settings.frequency = 2.0f;
            settings.dampingRatio = 1.0f;
        }
        hinge.setServo(0.2f, settings);
        const f32 slept = timeToSleep(rig, arm, 10.0f);
        const f32 held = hinge.currentAngle();
        printf("%s servo holding: asleep after %.2f s at %.4f rad\n", soft ? "soft" : "rigid", double(slept),
               double(held));
        CHECK(slept > 0.0f);
        rig.run(1.0f);
        CHECK(!arm.awake());
        CHECK_NEAR(hinge.currentAngle(), held, 1.0e-6f);

        hinge.setServoTarget(-0.6f);
        CHECK(staysAwake(rig, arm, 2.5f));
        rig.run(2.0f);
        CHECK_NEAR(hinge.currentAngle(), -0.6f, soft ? 0.12f : 2.0e-3f);
        rig.world.removeJoint(&hinge);
    }
}

void velocityMotorSleepsOnlyWhenStopped()
{
    Rig rig(60.0f, false);
    RigidBody& base = rig.base(KM::vec3(0.0f));
    RigidBody& wheel = rig.box(KM::vec3(0.0f), KM::vec3(0.3f, 0.3f, 0.05f), 5.0f);
    HingeJoint hinge(base, wheel, KM::vec3(0.0f), KM::vec3(0.0f, 0.0f, 1.0f));
    rig.world.addJoint(&hinge);
    hinge.setMotor(0.2f, 50.0f);
    CHECK(staysAwake(rig, wheel, 5.0f));
    hinge.setMotor(0.0f, 50.0f);
    CHECK(timeToSleep(rig, wheel, 10.0f) > 0.0f);
    hinge.setMotor(1.0f, 50.0f);
    rig.world.step(rig.step);
    CHECK(wheel.awake());
    rig.world.removeJoint(&hinge);
}

void cylinderSleepsWithTheValveClosed()
{
    Rig rig(60.0f, true);
    RigidBody& base = rig.base(KM::vec3(0.0f));
    RigidBody& load = rig.box(KM::vec3(0.0f, 1.2f, 0.0f), KM::vec3(0.3f), 500.0f);
    SliderJoint slider(base, load, KM::vec3(0.0f, 1.2f, 0.0f), KM::vec3(0.0f, 1.0f, 0.0f));
    rig.world.addJoint(&slider);
    HydraulicCylinderParameters p;
    p.initialStroke = 0.2f;
    p.pumpFlow = 3.0f; // 6 mm/s: a crawl
    HydraulicCylinder cylinder(base, KM::vec3(0.0f), load, KM::vec3(0.0f, 1.2f, 0.0f), p);
    rig.world.addJoint(&cylinder);
    const f32 slept = timeToSleep(rig, load, 10.0f);
    printf("cylinder holding: asleep after %.2f s\n", double(slept));
    CHECK(slept > 0.0f);
    cylinder.setValve(1.0f);
    CHECK(staysAwake(rig, load, 3.0f));
    printf("  valve open: %.4f m/s, stroke %.4f\n", double(cylinder.strokeVelocity()), double(cylinder.stroke()));
    CHECK(cylinder.stroke() > 0.2f + 0.01f);
    cylinder.setValve(0.0f);
    CHECK(timeToSleep(rig, load, 10.0f) > 0.0f);
    rig.world.removeJoint(&cylinder);
    rig.world.removeJoint(&slider);
}

void winchSleepsBrakedAndWakesToHoist()
{
    Rig rig(60.0f, true);
    RigidBody& top = rig.base(KM::vec3(0.0f, 10.0f, 0.0f));
    RigidBody& load = rig.box(KM::vec3(0.0f), KM::vec3(0.3f), 300.0f);
    WinchParameters p;
    p.maxLinePull = 10.0e3f;
    p.maxLineSpeed = 0.05f;
    Winch winch(p);
    Rope rope(top, KM::vec3(0.0f, 10.0f, 0.0f), load, KM::vec3(0.0f, 0.3f, 0.0f));
    rope.setWinch(&winch);
    rig.world.addJoint(&rope);
    CHECK(timeToSleep(rig, load, 10.0f) > 0.0f);
    const f32 y0 = load.position().y;
    winch.setCommand(1.0f);
    CHECK(staysAwake(rig, load, 4.0f));
    printf("winch: hoisted %.4f m at 0.05 m/s in 4 s\n", double(load.position().y - y0));
    CHECK_NEAR(load.position().y - y0, 0.2f, 0.01f);
    winch.setCommand(0.0f);
    CHECK(timeToSleep(rig, load, 10.0f) > 0.0f);
    rig.world.removeJoint(&rope);
}

void electricMotorSleepsAtZeroVolts()
{
    Rig rig(60.0f, false);
    RigidBody& base = rig.base(KM::vec3(0.0f));
    RigidBody& wheel = rig.box(KM::vec3(0.0f), KM::vec3(0.3f, 0.3f, 0.05f), 5.0f);
    HingeJoint hinge(base, wheel, KM::vec3(0.0f), KM::vec3(0.0f, 0.0f, 1.0f));
    rig.world.addJoint(&hinge);
    ElectricMotorParameters p;
    p.noLoadSpeed = 10.0f;
    ElectricMotor motor(hinge.drive(), p);
    motor.setVoltage(0.02f);
    CHECK(staysAwake(rig, wheel, 3.0f));
    motor.setVoltage(0.0f);
    CHECK(timeToSleep(rig, wheel, 10.0f) > 0.0f);
    rig.world.removeJoint(&hinge);
}

void hookWakesWhatItTakesAndDrops()
{
    Rig rig(60.0f, true);
    RigidBody& ground = rig.base(KM::vec3(0.0f, -0.5f, 0.0f), true, KM::vec3(5.0f, 0.5f, 5.0f));
    RigidBody& load = rig.box(KM::vec3(0.0f, 0.5f, 0.0f), KM::vec3(0.5f), 100.0f, true);
    RigidBody& crane = rig.base(KM::vec3(0.0f, 1.2f, 0.0f));
    (void)ground;
    rig.world.markStaticBroadphaseDirty();
    CHECK(timeToSleep(rig, load, 10.0f) > 0.0f);
    Hook hook(rig.world, rig.idOf(crane), KM::vec3(0.0f, 1.0f, 0.0f));
    CHECK(hook.attach(rig.idOf(load)));
    CHECK(load.awake());
    CHECK(timeToSleep(rig, load, 10.0f) > 0.0f);
    hook.detach();
    CHECK(load.awake());
}

}

int main()
{
    servoSleepsWhenHoldingAndWakesForATarget();
    velocityMotorSleepsOnlyWhenStopped();
    cylinderSleepsWithTheValveClosed();
    winchSleepsBrakedAndWakesToHoist();
    electricMotorSleepsAtZeroVolts();
    hookWakesWhatItTakesAndDrops();
    return TEST_RESULT();
}
