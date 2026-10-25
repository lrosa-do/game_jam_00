#include "actuator_rig.h"
#include "check.h"

#include <actuators/Rope.h>

#include <math.h>
#include <stdio.h>

using namespace actuator_test;

namespace
{

// A drum on the ground at (6, 0, 0), up to a sheave on a static boom tip at
// (0, 20, 0), down to a hook block hanging at (0, 10, 0).
struct Crane
{
    Rig rig;
    RigidBody* ground = nullptr;
    RigidBody* tip = nullptr;
    RigidBody* load = nullptr;
    Winch winch;
    Rope* rope = nullptr;

    Crane(f32 mass, u32 falls, const WinchParameters& parameters, f32 hz = 60.0f,
          f32 hookHeight = 10.0f, f32 tipHeight = 20.0f)
        : rig(hz, true), winch(parameters)
    {
        ground = &rig.base(KM::vec3(6.0f, 0.0f, 0.0f));
        tip = &rig.base(KM::vec3(0.0f, tipHeight, 0.0f));
        load = &rig.box(KM::vec3(0.0f, hookHeight, 0.0f), KM::vec3(0.4f), mass);
        rope = new Rope(*ground, KM::vec3(6.0f, 0.0f, 0.0f), *load, KM::vec3(0.0f, hookHeight + 0.4f, 0.0f));
        rope->addSheave(*tip, KM::vec3(0.0f, tipHeight, 0.0f));
        rope->setFalls(falls);
        rope->setWinch(&winch);
        rig.world.addJoint(rope);
    }
    ~Crane()
    {
        rig.world.removeJoint(rope);
        delete rope;
    }
};

WinchParameters winchParameters(f32 maxPull, f32 maxSpeed)
{
    WinchParameters p;
    p.drumRadius = 0.3f;
    p.maxLinePull = maxPull;
    p.maxLineSpeed = maxSpeed;
    return p;
}

void liftSpeedIsLineSpeedOverFalls()
{
    const u32 fallsList[] = {1, 2, 4};
    for (u32 falls : fallsList)
    {
        Crane c(800.0f, falls, winchParameters(60.0e3f, 1.0f));
        c.rig.run(0.5f);
        c.winch.setCommand(1.0f);
        c.rig.run(1.0f);
        const f32 y0 = c.load->position().y;
        c.rig.run(2.0f);
        const f32 speed = (c.load->position().y - y0) / 2.0f;
        printf("%u falls: hook rises at %.4f m/s (%.4f), line %.4f m/s, tension %.0f N, hook load %.0f N\n",
               falls, double(speed), double(1.0f / falls), double(-c.winch.lineSpeed()),
               double(c.rope->tension()), double(c.rope->hookLoad()));
        CHECK_NEAR(speed, 1.0f / falls, 0.02f / falls);
        CHECK_NEAR(c.winch.lineSpeed(), -1.0f, 0.01f);
        CHECK_NEAR(c.rope->hookLoad(), 800.0f * kG, 0.02f * 800.0f * kG);
        CHECK_NEAR(c.rope->tension(), 800.0f * kG / falls, 0.02f * 800.0f * kG / falls);
        CHECK_NEAR(c.winch.motorTorque(), c.rope->tension() * 0.3f, 0.02f * c.rope->tension() * 0.3f);
    }
}

// 20 kN line pull on two falls lifts up to 40 kN: 3.5 t rises, 4.5 t stalls.
void linePullLimitsTheLift()
{
    const f32 masses[] = {3500.0f, 4500.0f};
    for (f32 mass : masses)
    {
        Crane c(mass, 2, winchParameters(20.0e3f, 0.6f));
        c.rig.run(0.5f);
        const f32 y0 = c.load->position().y;
        c.winch.setCommand(1.0f);
        c.rig.run(3.0f);
        const f32 rise = c.load->position().y - y0;
        printf("%.1f t on 2 falls, 20 kN pull: rose %.3f m, tension %.0f N, motor %.0f N m\n",
               double(mass * 1.0e-3f), double(rise), double(c.rope->tension()),
               double(c.winch.motorTorque()));
        if (mass * kG < 2.0f * 20.0e3f)
        {
            CHECK_NEAR(rise, 0.3f * 3.0f, 0.03f);
            CHECK_NEAR(c.rope->tension(), mass * kG / 2.0f, 0.02f * mass * kG / 2.0f);
        }
        else
        {
            CHECK(rise < 0.0f);
            CHECK_NEAR(c.rope->tension(), 20.0e3f, 0.02f * 20.0e3f);
            CHECK_NEAR(c.winch.motorTorque(), 20.0e3f * 0.3f, 0.02f * 20.0e3f * 0.3f);
        }
    }
}

void slackRopePullsNothing()
{
    Rig rig(60.0f, true);
    RigidBody& anchor = rig.base(KM::vec3(0.0f, 10.0f, 0.0f));
    RigidBody& load = rig.box(KM::vec3(0.0f), KM::vec3(0.2f), 50.0f);
    Rope rope(anchor, KM::vec3(0.0f, 10.0f, 0.0f), load, KM::vec3(0.0f), 11.0f);
    rig.world.addJoint(&rope);
    f32 worstTension = 0.0f;
    for (int i = 0; i < 18; ++i)
    {
        rig.world.step(rig.step);
        worstTension = fmaxf(worstTension, rope.tension());
    }
    const f32 t = 18.0f * rig.step;
    printf("slack: after %.2f s fallen at %.4f m/s (g t = %.4f), worst tension %.3f N\n", double(t),
           double(-load.velocity().y), double(kG * t), double(worstTension));
    CHECK(worstTension == 0.0f);
    CHECK(!rope.taut());
    CHECK_NEAR(-load.velocity().y, kG * t, 0.01f * kG * t);
    rig.run(3.0f);
    printf("caught: stretch %.2e m, tension %.2f N (m g %.2f)\n", double(rope.stretch()),
           double(rope.tension()), double(50.0f * kG));
    CHECK(fabsf(rope.stretch()) < 2.0e-3f);
    CHECK_NEAR(rope.tension(), 50.0f * kG, 0.01f * 50.0f * kG);
    rig.world.removeJoint(&rope);
}

// Two loads on one rope over two sheaves: the heavier goes down, the
// lighter up, at g (m1 - m2) / (m1 + m2), and the rope keeps its length.
void ropeOverSheavesKeepsItsLength()
{
    Rig rig(60.0f, true);
    RigidBody& beam = rig.base(KM::vec3(0.0f, 5.0f, 0.0f));
    RigidBody& heavy = rig.box(KM::vec3(-0.5f, 2.0f, 0.0f), KM::vec3(0.1f), 10.0f);
    RigidBody& light = rig.box(KM::vec3(0.5f, 2.0f, 0.0f), KM::vec3(0.1f), 5.0f);
    Rope rope(heavy, KM::vec3(-0.5f, 2.0f, 0.0f), light, KM::vec3(0.5f, 2.0f, 0.0f));
    rope.addSheave(beam, KM::vec3(-0.5f, 5.0f, 0.0f));
    rope.addSheave(beam, KM::vec3(0.5f, 5.0f, 0.0f));
    rig.world.addJoint(&rope);
    const f32 rest = rope.restLength();
    CHECK_NEAR(rest, 7.0f, 1.0e-5f);
    f32 worst = 0.0f;
    for (int i = 0; i < 60; ++i)
    {
        rig.world.step(rig.step);
        worst = fmaxf(worst, fabsf(rope.length() - rest));
    }
    const f32 expected = kG * (10.0f - 5.0f) / 15.0f;
    printf("Atwood: heavy at %.4f m/s after 1 s (a t = %.4f), worst length change %.2e m, tension %.3f N "
           "(%.3f)\n",
           double(-heavy.velocity().y), double(expected), double(worst), double(rope.tension()),
           double(2.0f * 10.0f * 5.0f * kG / 15.0f));
    CHECK_NEAR(-heavy.velocity().y, expected, 0.02f * expected);
    CHECK_NEAR(light.velocity().y, expected, 0.02f * expected);
    CHECK(worst < 3.0e-3f);
    CHECK_NEAR(rope.tension(), 2.0f * 10.0f * 5.0f * kG / 15.0f, 0.02f * 65.4f);
    rig.world.removeJoint(&rope);
}

void heavyLoadIsStable()
{
    WinchParameters p = winchParameters(150.0e3f, 0.3f);
    p.drumInertia = 40.0f;
    Crane c(10000.0f, 1, p, 60.0f, 10.0f, 40.0f);
    // 6 m across and 40 m up to the tip, 30 m down to the hook.
    f32 worst = 0.0f;
    bool finite = true;
    for (int i = 0; i < 10 * 60; ++i)
    {
        c.rig.world.step(c.rig.step);
        worst = fmaxf(worst, c.rope->stretch());
        finite = finite && finiteVec(c.load->position());
    }
    const f32 y0 = c.load->position().y;
    printf("10 t on 30 m: worst stretch %.2e m, tension %.0f N (m g %.0f), y %.4f\n", double(worst),
           double(c.rope->tension()), double(10000.0f * kG), double(y0));
    CHECK(finite);
    CHECK(worst < 3.0e-3f);
    CHECK_NEAR(c.rope->tension(), 10000.0f * kG, 0.01f * 10000.0f * kG);
    CHECK(fabsf(y0 - 10.0f) < 3.0e-3f);

    c.winch.setCommand(1.0f);
    for (int i = 0; i < 4 * 60; ++i)
    {
        c.rig.world.step(c.rig.step);
        worst = fmaxf(worst, c.rope->stretch());
    }
    c.winch.setCommand(0.0f);
    c.rig.run(3.0f);
    const f32 lifted = c.load->position().y - y0;
    printf("hoisted 4 s at 0.3 m/s: %.4f m, worst stretch %.2e m\n", double(lifted), double(worst));
    CHECK_NEAR(lifted, 1.2f, 0.03f);
    CHECK(worst < 5.0e-3f);
    CHECK(fabsf(c.load->velocity().y) < 1.0e-3f);

    // The same with an elastic rope: it stretches by m g L / EA, and holds.
    Crane e(10000.0f, 1, p, 60.0f, 10.0f, 40.0f);
    const f32 ea = 2.0e8f;
    e.rope->setElasticity(ea, 0.7f);
    e.load->setDamping(0.6f, 0.6f);
    e.rig.run(10.0f);
    const f32 restLength = e.rope->restLength();
    const f32 expected = 10000.0f * kG * restLength / ea;
    printf("elastic: stretch %.4f m (m g L / EA %.4f)\n", double(e.rope->stretch()), double(expected));
    CHECK_NEAR(e.rope->stretch(), expected, 0.03f * expected);
}

void brakeHolds()
{
    Crane c(1000.0f, 1, winchParameters(20.0e3f, 0.5f));
    c.rig.run(1.0f);
    const f32 y0 = c.load->position().y;
    c.rig.run(5.0f);
    printf("brake: drift %.2e m over 5 s, brake torque %.0f N m\n", double(c.load->position().y - y0),
           double(c.winch.brakeTorque()));
    CHECK(fabsf(c.load->position().y - y0) < 1.0e-3f);
    CHECK(c.winch.braking());
    CHECK_NEAR(c.winch.brakeTorque(), 1000.0f * kG * 0.3f, 0.02f * 1000.0f * kG * 0.3f);

    // 30 kN of brake under 4 t slips.
    WinchParameters weak = winchParameters(20.0e3f, 0.5f);
    weak.brakeLinePull = 30.0e3f;
    Crane slipping(4000.0f, 1, weak);
    slipping.rig.run(2.0f);
    printf("brake slipping: hook %.3f m below, tension %.0f N\n",
           double(10.0f - slipping.load->position().y), double(slipping.rope->tension()));
    CHECK(slipping.load->position().y < 9.0f);
    CHECK_NEAR(slipping.rope->tension(), 30.0e3f, 0.02f * 30.0e3f);

    // With no brake and no command, the drum runs free and the load falls,
    // slowed only by the drum's inertia: a = g m / (m + J / r^2).
    WinchParameters free = winchParameters(20.0e3f, 0.5f);
    free.drumInertia = 9.0f; // 100 kg at 0.3 m
    Crane falling(300.0f, 1, free);
    falling.winch.setFreeSpool(true);
    falling.rig.run(0.5f);
    const f32 expected = kG * 300.0f / 400.0f;
    printf("free drum: hook falling at %.4f m/s after 0.5 s (a t = %.4f)\n",
           double(-falling.load->velocity().y), double(expected * 0.5f));
    CHECK_NEAR(-falling.load->velocity().y, expected * 0.5f, 0.02f * expected * 0.5f);
    CHECK_NEAR(falling.winch.lineSpeed(), expected * 0.5f, 0.02f * expected * 0.5f);
    falling.winch.setBrake(true);
    falling.rig.run(1.0f);
    CHECK(fabsf(falling.load->velocity().y) < 1.0e-3f);
}

void lengthLimitsStopTheDrum()
{
    Crane c(500.0f, 1, winchParameters(20.0e3f, 1.0f));
    c.rig.run(0.2f);
    const f32 rest = c.rope->restLength();
    c.rope->setLengthLimits(rest - 1.0f, rest + 1.0f);
    c.winch.setCommand(1.0f);
    c.rig.run(3.0f);
    printf("hoisted to the limit: rest %.4f (limit %.4f), hook speed %.2e\n",
           double(c.rope->restLength()), double(rest - 1.0f), double(c.load->velocity().y));
    CHECK_NEAR(c.rope->restLength(), rest - 1.0f, 1.0e-4f);
    CHECK(fabsf(c.load->velocity().y) < 1.0e-2f);
    c.winch.setCommand(-1.0f);
    c.rig.run(4.0f);
    CHECK_NEAR(c.rope->restLength(), rest + 1.0f, 1.0e-4f);
}

}

int main()
{
    liftSpeedIsLineSpeedOverFalls();
    linePullLimitsTheLift();
    slackRopePullsNothing();
    ropeOverSheavesKeepsItsLength();
    heavyLoadIsStable();
    brakeHolds();
    lengthLimitsStopTheDrum();
    return TEST_RESULT();
}
