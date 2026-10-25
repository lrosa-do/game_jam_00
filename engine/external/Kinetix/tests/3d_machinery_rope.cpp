#include "check.h"
#include "actuator_rig.h"

#include <actuators/Rope.h>

using namespace actuator_test;

namespace
{

void wrappedLength()
{
    Rig rig(60.0f, false);
    RigidBody& ground = rig.base(KM::vec3(0.0f, -10.0f, 0.0f));
    RigidBody& load = rig.box(KM::vec3(0.3f, 0.0f, 0.0f), KM::vec3(0.1f), 10.0f);
    const f32 r = 0.3f;
    // Half a turn: both legs vertical, from x = -r and x = +r 5 m below a
    // sheave at (0, 5).
    Rope half(ground, KM::vec3(-r, 0.0f, 0.0f), load, KM::vec3(r, 0.0f, 0.0f));
    half.addSheave(ground, KM::vec3(0.0f, 5.0f, 0.0f), r);
    Rope quarter(ground, KM::vec3(-5.0f, 5.0f + r, 0.0f), load, KM::vec3(r, 0.0f, 0.0f));
    quarter.addSheave(ground, KM::vec3(0.0f, 5.0f, 0.0f), r);
    Rope straight(ground, KM::vec3(-5.0f, 5.0f + r, 0.0f), load, KM::vec3(r, 0.0f, 0.0f));
    straight.addSheave(ground, KM::vec3(0.0f, 5.0f, 0.0f), 0.0f);
    printf("wrap: half turn %.6f m (%.6f), quarter %.6f m (%.6f)\n", double(half.length()),
           double(10.0f + kPi * r), double(quarter.length()), double(10.0f + 0.5f * kPi * r));
    CHECK_NEAR(half.length(), 10.0f + kPi * r, 1.0e-5f);
    CHECK_NEAR(quarter.length(), 10.0f + 0.5f * kPi * r, 1.0e-5f);
    CHECK_NEAR(straight.length(), KM::length(KM::vec3(5.0f, r, 0.0f)) + KM::length(KM::vec3(r, -5.0f, 0.0f)), 1.0e-5f);
}

// m1 heavier: the rope runs towards B and the B side carries 1/eta of the A side.
void atwoodWithFriction(f32 efficiency)
{
    Rig rig(120.0f, true);
    RigidBody& top = rig.base(KM::vec3(0.0f, 10.0f, 0.0f));
    const f32 m1 = 12.0f;
    const f32 m2 = 8.0f;
    const f32 r = 0.2f;
    RigidBody& light = rig.box(KM::vec3(-r, 4.0f, 0.0f), KM::vec3(0.1f), m2);
    RigidBody& heavy = rig.box(KM::vec3(r, 4.0f, 0.0f), KM::vec3(0.1f), m1);
    Rope rope(light, KM::vec3(-r, 4.0f, 0.0f), heavy, KM::vec3(r, 4.0f, 0.0f));
    rope.addSheave(top, KM::vec3(0.0f, 10.0f, 0.0f), r, efficiency);
    rig.world.addJoint(&rope);
    // Taut from the start: let it pick its length, then a running start so
    // the direction is known.
    rig.world.step(rig.step);
    rig.run(1.0f);
    const f32 v0 = heavy.velocity().y;
    rig.run(0.5f);
    const f32 acceleration = (heavy.velocity().y - v0) / 0.5f;
    const f32 expected = -kG * (efficiency * m1 - m2) / (efficiency * m1 + m2);
    const f32 tensionA = rope.segmentTension(0);
    const f32 tensionB = rope.segmentTension(1);
    printf("Atwood over a sheave of eta %.2f: a = %.4f m/s^2 (%.4f); tensions %.3f / %.3f N (ratio %.4f), "
           "m2 (g + a) = %.3f\n",
           double(efficiency), double(acceleration), double(expected), double(tensionA), double(tensionB),
           double(tensionA / tensionB), double(m2 * (kG - expected)));
    CHECK_NEAR(acceleration, expected, 2.0e-3f * fabsf(expected) + 1.0e-3f);
    CHECK_NEAR(tensionA / tensionB, efficiency, 1.0e-3f);
    CHECK_NEAR(tensionA, m2 * (kG - expected), 0.01f * m2 * kG);
    rig.world.removeJoint(&rope);
}

// A winch on the ground, the rope up over two sheaves of eta 0.95 and down
// to a 2 t load: hoisting at constant speed the drum pulls W / eta^2,
// lowering W eta^2; holding, W.
void winchThroughFrictionalSheaves()
{
    Rig rig(60.0f, true);
    RigidBody& ground = rig.base(KM::vec3(0.0f));
    // Both legs vertical: under the outer tangent points of the sheaves.
    RigidBody& load = rig.box(KM::vec3(4.25f, 2.0f, 0.0f), KM::vec3(0.3f), 2000.0f);
    WinchParameters wp;
    wp.drumRadius = 0.3f;
    wp.maxLinePull = 60.0e3f;
    wp.maxLineSpeed = 0.4f;
    Winch winch(wp);
    Rope rope(ground, KM::vec3(-4.25f, 0.5f, 0.0f), load, KM::vec3(4.25f, 2.3f, 0.0f));
    rope.addSheave(ground, KM::vec3(-4.0f, 8.0f, 0.0f), 0.25f, 0.95f);
    rope.addSheave(ground, KM::vec3(4.0f, 8.0f, 0.0f), 0.25f, 0.95f);
    rope.setWinch(&winch);
    rig.world.addJoint(&rope);
    const f32 weight = 2000.0f * kG;
    rig.run(1.0f);
    const f32 holding = rope.tension();
    winch.setCommand(1.0f);
    rig.run(2.0f);
    const f32 hoisting = rope.tension();
    const f32 speedUp = load.velocity().y;
    winch.setCommand(-1.0f);
    rig.run(2.0f);
    const f32 lowering = rope.tension();
    const f32 speedDown = load.velocity().y;
    printf("winch through 2 sheaves of eta 0.95, 2 t: line pull %.0f N holding (W %.0f), %.0f hoisting (W/eta^2 "
           "%.0f) at %.3f m/s, %.0f lowering (W eta^2 %.0f) at %.3f m/s; hook load %.0f\n",
           double(holding), double(weight), double(hoisting), double(weight / (0.95f * 0.95f)), double(speedUp),
           double(lowering), double(weight * 0.95f * 0.95f), double(speedDown), double(rope.hookLoad()));
    CHECK_NEAR(holding, weight, 0.01f * weight);
    CHECK_NEAR(hoisting, weight / (0.95f * 0.95f), 0.005f * weight);
    CHECK_NEAR(lowering, weight * 0.95f * 0.95f, 0.005f * weight);
    CHECK_NEAR(speedUp, 0.4f, 1.0e-3f);
    CHECK_NEAR(speedDown, -0.4f, 1.0e-3f);
    CHECK_NEAR(rope.hookLoad(), weight, 0.005f * weight);
    rig.world.removeJoint(&rope);
}

}

int main()
{
    wrappedLength();
    atwoodWithFriction(1.0f);
    atwoodWithFriction(0.9f);
    winchThroughFrictionalSheaves();
    return TEST_RESULT();
}
