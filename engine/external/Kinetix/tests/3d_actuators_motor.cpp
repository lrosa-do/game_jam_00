#include "actuator_rig.h"
#include "check.h"

#include <actuators/ElectricMotor.h>
#include <dynamics/HingeJoint.h>
#include <dynamics/UniversalJoint.h>

#include <math.h>
#include <stdio.h>

using namespace actuator_test;

namespace
{

ElectricMotorParameters gearmotor()
{
    ElectricMotorParameters p;
    p.stallTorque = 2.0f;   // N m at the motor
    p.noLoadSpeed = 400.0f; // rad/s at the motor
    p.stallCurrent = 30.0f;
    p.noLoadCurrent = 1.0f;
    p.supplyVoltage = 24.0f;
    p.gearRatio = 50.0f;
    p.efficiency = 0.8f;
    return p;
}

struct Flywheel
{
    Rig rig;
    RigidBody* wheel = nullptr;
    HingeJoint* hinge = nullptr;

    Flywheel() : rig(60.0f, false)
    {
        RigidBody& base = rig.base(KM::vec3(0.0f));
        wheel = &rig.box(KM::vec3(0.0f), KM::vec3(0.3f, 0.3f, 0.05f), 20.0f);
        hinge = new HingeJoint(base, *wheel, KM::vec3(0.0f), KM::vec3(0.0f, 0.0f, 1.0f));
        rig.world.addJoint(hinge);
    }
    ~Flywheel()
    {
        rig.world.removeJoint(hinge);
        delete hinge;
    }

    void run(f32 seconds, f32 loadTorque)
    {
        const int steps = static_cast<int>(seconds / rig.step + 0.5f);
        for (int i = 0; i < steps; ++i)
        {
            wheel->addTorque(KM::vec3(0.0f, 0.0f, -loadTorque));
            rig.world.step(rig.step);
        }
    }
};

void ratingsAtTheOutput()
{
    Flywheel f;
    ElectricMotor motor(f.hinge->drive(), gearmotor());
    CHECK_NEAR(motor.outputStallTorque(), 2.0f * 50.0f * 0.8f, 1.0e-4f);
    CHECK_NEAR(motor.outputNoLoadSpeed(), 400.0f / 50.0f, 1.0e-5f);

    motor.setVoltage(1.0f);
    f.run(3.0f, 0.0f);
    printf("no load: %.4f rad/s (8), %.4f N m, %.3f A, motor %.1f rad/s\n", double(motor.outputSpeed()),
           double(motor.outputTorque()), double(motor.current()), double(motor.motorSpeed()));
    CHECK_NEAR(motor.outputSpeed(), 8.0f, 0.01f * 8.0f);
    CHECK_NEAR(f.hinge->currentAngularVelocity(), 8.0f, 0.01f * 8.0f);
    CHECK(fabsf(motor.outputTorque()) < 0.01f * 80.0f);
    CHECK_NEAR(motor.current(), 1.0f, 0.05f);
    CHECK_NEAR(motor.motorSpeed(), 400.0f, 4.0f);

    motor.setVoltage(0.5f);
    f.run(3.0f, 0.0f);
    CHECK_NEAR(motor.outputSpeed(), 4.0f, 0.01f * 4.0f);
    motor.setVoltage(-1.0f);
    f.run(4.0f, 0.0f);
    CHECK_NEAR(motor.outputSpeed(), -8.0f, 0.01f * 8.0f);
}

void stallTorque()
{
    Flywheel f;
    f.hinge->setLimits(0.0f, 0.0f);
    ElectricMotor motor(f.hinge->drive(), gearmotor());
    motor.setVoltage(1.0f);
    f.run(1.0f, 0.0f);
    printf("stall: %.4f N m (80), motor %.4f N m (2), %.3f A (30), %.1f W\n", double(motor.outputTorque()),
           double(motor.motorTorque()), double(motor.current()), double(motor.electricalPower()));
    CHECK_NEAR(motor.outputTorque(), 80.0f, 0.01f * 80.0f);
    CHECK_NEAR(motor.motorTorque(), 2.0f, 0.01f * 2.0f);
    CHECK_NEAR(motor.current(), 30.0f, 0.3f);
    CHECK_NEAR(motor.electricalPower(), 24.0f * 30.0f, 0.02f * 720.0f);
    CHECK_NEAR(f.hinge->limitTorque(), -80.0f, 0.01f * 80.0f);
}

// Loads of 1/4, 1/2, 3/4 of stall torque settle at 3/4, 1/2, 1/4 of the no-load speed.
void torqueSpeedLine()
{
    const f32 fractions[] = {0.25f, 0.5f, 0.75f};
    for (f32 fraction : fractions)
    {
        Flywheel f;
        ElectricMotor motor(f.hinge->drive(), gearmotor());
        motor.setVoltage(1.0f);
        f.run(4.0f, fraction * 80.0f);
        const f32 expected = 8.0f * (1.0f - fraction);
        printf("load %.0f N m: %.4f rad/s (%.4f), torque %.3f N m, current %.3f A, %.1f W out\n",
               double(fraction * 80.0f), double(motor.outputSpeed()), double(expected),
               double(motor.outputTorque()), double(motor.current()), double(motor.mechanicalPower()));
        CHECK_NEAR(motor.outputSpeed(), expected, 0.01f * 8.0f);
        CHECK_NEAR(motor.outputTorque(), fraction * 80.0f, 0.01f * 80.0f);
        // I = I0 + (Is - I0) T / Ts at the motor shaft.
        CHECK_NEAR(motor.current(), 1.0f + 29.0f * fraction, 0.3f);
    }
    Flywheel f;
    ElectricMotor motor(f.hinge->drive(), gearmotor());
    motor.setVoltage(0.0f);
    f.run(3.0f, 40.0f);
    printf("held against 40 N m at zero volts: %.4f rad/s (-4), %.3f N m\n", double(motor.outputSpeed()),
           double(motor.outputTorque()));
    CHECK_NEAR(motor.outputSpeed(), -4.0f, 0.01f * 8.0f);
}

void positionServoInTheEnvelope()
{
    Rig rig(60.0f, true);
    RigidBody& base = rig.base(KM::vec3(0.0f));
    RigidBody& tool = rig.box(KM::vec3(0.0f, 0.0f, 0.3f), KM::vec3(0.05f, 0.05f, 0.3f), 3.0f);
    UniversalJoint wrist(base, tool, KM::vec3(0.0f), KM::vec3(1.0f, 0.0f, 0.0f), KM::vec3(0.0f, 1.0f, 0.0f));
    rig.world.addJoint(&wrist);
    ElectricMotor pitch(wrist.driveA(), gearmotor());
    ElectricMotor yaw(wrist.driveB(), gearmotor());
    ServoSettings gains;
    gains.frequency = 6.0f;
    gains.dampingRatio = 1.0f;
    pitch.setPositionTarget(0.6f, gains);
    yaw.setPositionTarget(-0.4f, gains);
    f32 fastest = 0.0f;
    f32 strongest = 0.0f;
    for (int i = 0; i < 4 * 60; ++i)
    {
        rig.world.step(rig.step);
        fastest = fmaxf(fastest, fmaxf(fabsf(pitch.outputSpeed()), fabsf(yaw.outputSpeed())));
        strongest = fmaxf(strongest, fmaxf(fabsf(pitch.outputTorque()), fabsf(yaw.outputTorque())));
    }
    printf("wrist: %.4f (0.6), %.4f (-0.4); fastest %.3f rad/s, strongest %.3f N m\n",
           double(wrist.currentAngleA()), double(wrist.currentAngleB()), double(fastest), double(strongest));
    CHECK_NEAR(wrist.currentAngleA(), 0.6f, 0.03f);
    CHECK_NEAR(wrist.currentAngleB(), -0.4f, 0.03f);
    CHECK(fastest <= 8.0f * 1.01f);
    CHECK(strongest <= 80.0f * 1.001f);
    rig.world.removeJoint(&wrist);
}

}

int main()
{
    ratingsAtTheOutput();
    stallTorque();
    torqueSpeedLine();
    positionServoInTheEnvelope();
    return TEST_RESULT();
}
