#include "check.h"

#include <rotorcraft/Rotor.h>

#include <math.h>

using namespace kinetix::Physics;
using kinetix::f32;
using kinetix::u32;
namespace KM = kinetix::Math;

namespace
{
constexpr f32 kPi = 3.14159265358979f;
constexpr f32 kDeg = kPi / 180.0f;
constexpr f32 kRho = 1.225f;
constexpr f32 kOmega = 200.0f / 4.6f;

Airfoil section()
{
    Airfoil::Parameters p;
    p.liftSlope = 5.73f;
    p.maxLift = 1.35f;
    p.minLift = -1.35f;
    p.minDrag = 0.0095f;
    p.liftDragFactor = 0.008f;
    p.maxDrag = 2.0f;
    return Airfoil(p);
}

Rotor::Description mainRotor()
{
    Rotor::Description d;
    d.radius = 4.6f;
    d.rootCutout = 0.7f;
    d.bladeCount = 3;
    d.rootChord = 0.25f;
    d.tipChord = 0.25f;
    d.twist = -8.0f * kDeg;
    d.hingeOffset = 0.18f;
    d.bladeMass = 22.0f;
    d.position = KM::vec3(0.0f, 1.5f, 0.0f);
    d.shaftAxis = KM::vec3(0.0f, 1.0f, 0.0f);
    d.forward = KM::vec3(0.0f, 0.0f, 1.0f);
    return d;
}

RotorOperatingPoint hover(f32 omega = kOmega, f32 density = kRho)
{
    RotorOperatingPoint p;
    p.angularSpeed = omega;
    p.density = density;
    return p;
}

RotorOperatingPoint forward(f32 speed)
{
    RotorOperatingPoint p = hover();
    p.velocity = KM::vec3(0.0f, 0.0f, speed);
    return p;
}

f32 idealInduced(f32 thrust, const Rotor& rotor, f32 density = kRho)
{
    return sqrtf(thrust / (2.0f * density * rotor.discArea()));
}

// Collective giving `thrust` at the operating point, by bisection.
f32 collectiveFor(Rotor& rotor, const RotorOperatingPoint& point, f32 thrust)
{
    f32 lo = -0.05f;
    f32 hi = 0.35f;
    for (int i = 0; i < 50; ++i)
    {
        const f32 mid = 0.5f * (lo + hi);
        rotor.setControls(mid, 0.0f, 0.0f);
        if (rotor.solve(point).thrust > thrust)
            hi = mid;
        else
            lo = mid;
    }
    rotor.setControls(0.5f * (lo + hi), 0.0f, 0.0f);
    rotor.solve(point);
    return 0.5f * (lo + hi);
}

void derivedProperties()
{
    const Rotor rotor(mainRotor(), section());
    const Rotor::Description& d = rotor.description();
    // gamma = rho a c R^4 / I_beta with I_beta = m (R - e)^2 / 3.
    const f32 flapInertia = d.bladeMass * (d.radius - d.hingeOffset) * (d.radius - d.hingeOffset) / 3.0f;
    const f32 lock = kRho * 5.73f * d.rootChord * powf(d.radius, 4.0f) / flapInertia;
    CHECK_NEAR(rotor.flapInertia(), flapInertia, 1e-3);
    CHECK_NEAR(rotor.lockNumber(kRho), lock, 1e-3 * lock);
    CHECK(lock > 4.0f && lock < 8.0f);
    // nu^2 = 1 + e S / I = 1 + 3 e / (2 (R - e)) with no spring.
    const f32 nu = sqrtf(1.0f + 1.5f * d.hingeOffset / (d.radius - d.hingeOffset));
    CHECK_NEAR(rotor.flapFrequencyRatio(kOmega), nu, 1e-4);
    // A flap spring stiffens it, more at low rotor speed.
    Rotor::Description sprung = mainRotor();
    sprung.flapSpring = 20000.0f;
    const Rotor stiff(sprung, section());
    CHECK(stiff.flapFrequencyRatio(kOmega) > nu);
    CHECK(stiff.flapFrequencyRatio(0.5f * kOmega) > stiff.flapFrequencyRatio(kOmega));
    CHECK_NEAR(rotor.solidity(), 3.0f * 0.25f * (4.6f - 0.7f) / (kPi * 4.6f * 4.6f), 1e-5);
    printf("Lock number %.2f, flap frequency %.3f /rev, polar inertia %.0f kg m^2\n", double(lock),
           double(nu), double(rotor.polarInertia()));
}

void hoverMomentumTheory()
{
    Rotor rotor(mainRotor(), section());
    const f32 weight = 1300.0f * 9.81f;
    const f32 collective = collectiveFor(rotor, hover(), weight);
    const RotorLoads& loads = rotor.loads();
    const f32 vi = idealInduced(loads.thrust, rotor);
    const f32 figureOfMerit = loads.thrust * vi / loads.power;
    printf("hover: T %.0f N at %.2f deg, P %.1f kW, vi %.3f m/s against momentum %.3f (%+.2f%%), "
           "FM %.3f, CT %.5f, coning %.2f deg\n",
           double(loads.thrust), double(collective / kDeg), double(loads.power / 1000.0f),
           double(loads.inducedVelocity), double(vi), double(100.0f * (loads.inducedVelocity / vi - 1.0f)),
           double(figureOfMerit), double(loads.thrustCoefficient), double(loads.coning / kDeg));
    CHECK_NEAR(loads.thrust, weight, 0.002 * weight);
    CHECK_NEAR(loads.inducedVelocity, vi, 0.02 * vi);
    CHECK(figureOfMerit > 0.6f && figureOfMerit < 0.8f);
    CHECK(fabsf(loads.hForce) < 1.0f && fabsf(loads.yForce) < 1.0f);
    CHECK(KM::length(loads.moment) < 1.0f);
    CHECK(fabsf(loads.longitudinalFlapping) < 1e-4f && fabsf(loads.lateralFlapping) < 1e-4f);
    CHECK(loads.coning > 1.0f * kDeg && loads.coning < 6.0f * kDeg);
    CHECK_NEAR(loads.power, loads.torque * kOmega, 1e-3 * loads.power);
    CHECK_NEAR(loads.inflowRatio, loads.inducedVelocity / (kOmega * 4.6f), 1e-5);

    for (f32 deg = 5.0f; deg <= 11.0f; deg += 3.0f)
    {
        rotor.setControls(deg * kDeg, 0.0f, 0.0f);
        const RotorLoads& l = rotor.solve(hover());
        const f32 v = idealInduced(l.thrust, rotor);
        CHECK_NEAR(l.inducedVelocity, v, 0.03 * v);
        const f32 fm = l.thrust * v / l.power;
        CHECK(fm > 0.45f && fm < 0.82f);
    }
}

void thrustScaling()
{
    Rotor rotor(mainRotor(), section());
    rotor.setControls(8.0f * kDeg, 0.0f, 0.0f);
    const f32 reference = rotor.solve(hover()).thrust;
    // At a fixed pitch in hover C_T is fixed: T ~ rho (Omega R)^2.
    const f32 thin = rotor.solve(hover(kOmega, 0.9f)).thrust;
    const f32 slow = rotor.solve(hover(0.9f * kOmega)).thrust;
    printf("thrust scaling: rho 0.9/1.225 -> %.4f (%.4f), omega 0.9 -> %.4f (0.81)\n",
           double(thin / reference), double(0.9f / kRho), double(slow / reference));
    CHECK_NEAR(thin / reference, 0.9f / kRho, 0.01);
    CHECK_NEAR(slow / reference, 0.81, 0.01);
    f32 previous = -1.0e9f;
    for (f32 deg = 0.0f; deg <= 14.0f; deg += 1.0f)
    {
        rotor.setControls(deg * kDeg, 0.0f, 0.0f);
        const f32 thrust = rotor.solve(hover()).thrust;
        CHECK(thrust > previous);
        previous = thrust;
    }
    rotor.setControls(-4.0f * kDeg, 0.0f, 0.0f);
    const RotorLoads& negative = rotor.solve(hover());
    CHECK(negative.thrust < 0.0f);
    CHECK(negative.inducedVelocity < 0.0f);
}

void tipLossReducesThrust()
{
    Rotor lossy(mainRotor(), section());
    Rotor::Description d = mainRotor();
    d.tipLoss = false;
    Rotor ideal(d, section());
    lossy.setControls(8.0f * kDeg, 0.0f, 0.0f);
    ideal.setControls(8.0f * kDeg, 0.0f, 0.0f);
    const RotorLoads a = lossy.solve(hover());
    const RotorLoads b = ideal.solve(hover());
    printf("tip loss: T %.0f N against %.0f N without (%.2f%%)\n", double(a.thrust), double(b.thrust),
           double(100.0f * (a.thrust / b.thrust - 1.0f)));
    CHECK(a.thrust < b.thrust);
    const f32 weight = 1300.0f * 9.81f;
    collectiveFor(lossy, hover(), weight);
    collectiveFor(ideal, hover(), weight);
    CHECK(lossy.loads().power > ideal.loads().power);
    // Without tip loss the uniform-inflow ideal is approached: the mean
    // induced velocity sits at (or below, inboard sections unloaded) the
    // momentum value.
    CHECK(ideal.loads().inducedVelocity <= 1.005f * idealInduced(ideal.loads().thrust, ideal));
}

void translationalLift()
{
    Rotor rotor(mainRotor(), section());
    rotor.setControls(8.0f * kDeg, 0.0f, 0.0f);
    const f32 hovering = rotor.solve(hover()).thrust;
    const f32 at10 = rotor.solve(forward(10.0f)).thrust;
    const f32 at15 = rotor.solve(forward(15.0f)).thrust;
    printf("translational lift at fixed pitch: %.0f N hover, %.0f N at 10 m/s, %.0f N at 15 m/s\n",
           double(hovering), double(at10), double(at15));
    CHECK(at10 > 1.08f * hovering);
    CHECK(at15 > at10);
    // At constant thrust the induced power falls with airspeed (Glauert):
    // the induced velocity is well below hover's by 15 m/s.
    const f32 weight = 1300.0f * 9.81f;
    collectiveFor(rotor, hover(), weight);
    const f32 hoverInduced = rotor.loads().inducedVelocity;
    collectiveFor(rotor, forward(15.0f), weight);
    const f32 forwardInduced = rotor.loads().inducedVelocity;
    CHECK(forwardInduced < 0.6f * hoverInduced);
    // Glauert's high-speed limit: v = T / (2 rho A V).
    collectiveFor(rotor, forward(40.0f), weight);
    const f32 glauert = weight / (2.0f * kRho * rotor.discArea() * 40.0f);
    printf("induced velocity at 40 m/s %.2f m/s, Glauert %.2f m/s\n",
           double(rotor.loads().inducedVelocity), double(glauert));
    CHECK_NEAR(rotor.loads().inducedVelocity, glauert, 0.1 * glauert);
}

void blowback()
{
    Rotor rotor(mainRotor(), section());
    rotor.setControls(8.0f * kDeg, 0.0f, 0.0f);
    f32 previous = 0.0f;
    for (f32 mu = 0.05f; mu <= 0.251f; mu += 0.05f)
    {
        const RotorLoads& l = rotor.solve(forward(mu * kOmega * 4.6f));
        printf("mu %.2f: a1s %.2f deg, b1s %.2f deg, coning %.2f deg\n", double(l.advanceRatio),
               double(l.longitudinalFlapping / kDeg), double(l.lateralFlapping / kDeg),
               double(l.coning / kDeg));
        // The disc blows back (aft) more the faster it goes: the advancing
        // side's extra lift peaks a quarter turn later, over the nose.
        CHECK(l.longitudinalFlapping > previous);
        previous = l.longitudinalFlapping;
        CHECK(l.hForce > 0.0f);
        CHECK(l.discTiltForward < 0.0f);
    }
}

void cyclicSigns(f32 rotation)
{
    Rotor::Description d = mainRotor();
    d.rotation = rotation;
    Rotor rotor(d, section());
    const f32 cyclic = 3.0f * kDeg;

    rotor.setControls(8.0f * kDeg, cyclic, 0.0f);
    const RotorLoads forwardStick = rotor.solve(hover());
    // Disc tilts forward about one for one in hover; thrust forward; hub
    // moment nose down (about body +X, the left axis, by the right-hand
    // rule).
    CHECK_NEAR(forwardStick.discTiltForward, cyclic, 0.15 * cyclic);
    CHECK(fabsf(forwardStick.discTiltRight) < 0.2f * cyclic);
    CHECK(forwardStick.force.z > 0.0f);
    CHECK(forwardStick.moment.x > 0.0f);

    rotor.setControls(8.0f * kDeg, 0.0f, cyclic);
    const RotorLoads rightStick = rotor.solve(hover());
    // Disc tilts right, force to the right (-X), moment rolls right (about
    // +Z, forward).
    CHECK_NEAR(rightStick.discTiltRight, cyclic, 0.15 * cyclic);
    CHECK(fabsf(rightStick.discTiltForward) < 0.2f * cyclic);
    CHECK(rightStick.force.x < 0.0f);
    CHECK(rightStick.moment.z > 0.0f);

    // In forward flight the coning tilts the disc towards the advancing
    // side: right for an anticlockwise rotor, left for a clockwise one.
    rotor.setControls(8.0f * kDeg, 0.0f, 0.0f);
    const RotorLoads fast = rotor.solve(forward(25.0f));
    CHECK(fast.lateralFlapping > 0.0f);
    CHECK(fast.discTiltRight * rotation > 0.0f);
    // The torque reaction is the airframe's business; the rotor reports a
    // torque resisting rotation either way.
    CHECK(fast.torque > 0.0f);
}

void rotationMirrors()
{
    Rotor::Description d = mainRotor();
    Rotor anticlockwise(d, section());
    d.rotation = -1.0f;
    Rotor clockwise(d, section());
    anticlockwise.setControls(8.0f * kDeg, 2.0f * kDeg, -1.0f * kDeg);
    clockwise.setControls(8.0f * kDeg, 2.0f * kDeg, 1.0f * kDeg);
    RotorOperatingPoint p = forward(20.0f);
    p.velocity.x = 3.0f;
    const RotorLoads a = anticlockwise.solve(p);
    p.velocity.x = -3.0f;
    const RotorLoads b = clockwise.solve(p);
    CHECK_NEAR(a.thrust, b.thrust, 1e-3 * a.thrust);
    CHECK_NEAR(a.power, b.power, 1e-3 * fabsf(a.power) + 1.0);
    CHECK_NEAR(a.force.x, -b.force.x, 1e-3 * fabsf(a.force.x) + 0.5);
    CHECK_NEAR(a.force.z, b.force.z, 1e-3 * fabsf(a.force.z) + 0.5);
    CHECK_NEAR(a.moment.x, b.moment.x, 1e-3 * fabsf(a.moment.x) + 0.5);
    CHECK_NEAR(a.moment.z, -b.moment.z, 1e-3 * fabsf(a.moment.z) + 0.5);
    CHECK_NEAR(a.lateralFlapping, b.lateralFlapping, 1e-5);
    CHECK_NEAR(a.discTiltRight, -b.discTiltRight, 1e-5);
}

void rateDamping()
{
    Rotor rotor(mainRotor(), section());
    rotor.setControls(8.0f * kDeg, 0.0f, 0.0f);
    const f32 rate = 0.3f;
    // Nose-up pitch rate is a rotation about the right axis (-X).
    RotorOperatingPoint pitching = hover();
    pitching.angularVelocity = KM::vec3(-rate, 0.0f, 0.0f);
    const RotorLoads pitch = rotor.solve(pitching);
    // The disc lags behind the shaft (tilts forward relative to it) and
    // the hub moment and thrust tilt oppose the motion: nose down.
    CHECK(pitch.discTiltForward > 0.0f);
    CHECK(pitch.moment.x > 0.0f);
    // Classic first-order result: tilt ~ 16 q / (gamma Omega).
    const f32 expected = 16.0f * rate / (rotor.lockNumber(kRho) * kOmega);
    printf("pitch rate %.1f rad/s: disc lag %.3f deg against 16q/(gamma Omega) = %.3f deg\n",
           double(rate), double(pitch.discTiltForward / kDeg), double(expected / kDeg));
    CHECK_NEAR(pitch.discTiltForward, expected, 0.35 * expected);

    // Roll right is a rotation about +Z: the disc lags left and the moment
    // rolls left.
    RotorOperatingPoint rolling = hover();
    rolling.angularVelocity = KM::vec3(0.0f, 0.0f, rate);
    const RotorLoads roll = rotor.solve(rolling);
    CHECK(roll.discTiltRight < 0.0f);
    CHECK(roll.moment.z < 0.0f);
}

void groundEffect()
{
    Rotor rotor(mainRotor(), section());
    const f32 radius = 4.6f;
    const f32 weight = 1300.0f * 9.81f;
    collectiveFor(rotor, hover(), weight);
    const f32 power = rotor.loads().power;
    CHECK_NEAR(rotor.loads().groundEffectFactor, 1.0, 1e-6);

    for (f32 height : {0.5f, 0.75f, 1.0f})
    {
        RotorOperatingPoint near = hover();
        near.groundDistance = height * radius;
        f32 lo = 0.0f;
        f32 hi = 0.3f;
        for (int i = 0; i < 50; ++i)
        {
            const f32 mid = 0.5f * (lo + hi);
            rotor.setControls(mid, 0.0f, 0.0f);
            if (rotor.solve(near).power > power)
                hi = mid;
            else
                lo = mid;
        }
        const RotorLoads& l = rotor.loads();
        const f32 image = 1.0f / (4.0f * height);
        const f32 cheesemanBennett = 1.0f / (1.0f - image * image);
        printf("ground effect z/R %.2f: thrust ratio %.4f at constant power, Cheeseman-Bennett %.4f "
               "(%+.1f%%)\n",
               double(height), double(l.thrust / weight), double(cheesemanBennett),
               double(100.0f * (l.thrust / weight / cheesemanBennett - 1.0f)));
        CHECK_NEAR(l.thrust / weight, cheesemanBennett, 0.05 * cheesemanBennett);
        CHECK_NEAR(l.groundEffectFactor, cheesemanBennett, 1e-4);
    }

    rotor.setControls(8.0f * kDeg, 0.0f, 0.0f);
    RotorOperatingPoint near = hover();
    near.groundDistance = 0.5f * radius;
    const f32 inGround = rotor.solve(near).thrust;
    const f32 outOfGround = rotor.solve(hover()).thrust;
    CHECK(inGround > 1.1f * outOfGround);
    near.velocity = KM::vec3(0.0f, 0.0f, 25.0f);
    const RotorLoads fast = rotor.solve(near);
    CHECK(fast.groundEffectFactor < 1.02f);
}

void vortexRingFlag()
{
    Rotor rotor(mainRotor(), section());
    const f32 weight = 1300.0f * 9.81f;
    collectiveFor(rotor, hover(), weight);
    const f32 vh = idealInduced(weight, rotor);
    printf("vortex ring (v_h %.2f m/s):", double(vh));
    for (f32 ratio : {0.2f, 0.7f, 1.0f, 1.5f, 1.8f, 2.6f})
    {
        RotorOperatingPoint p = hover();
        p.velocity = KM::vec3(0.0f, -ratio * vh, 0.0f);
        collectiveFor(rotor, p, weight);
        const RotorLoads& l = rotor.loads();
        printf(" %.1f v_h -> %s (%.2f)", double(ratio), l.vortexRing ? "VRS" : "clear", double(l.descentRatio));
        CHECK(isfinite(l.thrust) && isfinite(l.inducedVelocity));
        const bool inside = l.descentRatio >= 0.5f && l.descentRatio <= 2.0f;
        CHECK(l.vortexRing == inside);
        if (ratio >= 0.6f && ratio <= 1.9f)
            CHECK(l.vortexRing);
        if (ratio <= 0.25f || ratio >= 2.5f)
            CHECK(!l.vortexRing);
    }
    printf("\n");
    RotorOperatingPoint p = hover();
    p.velocity = KM::vec3(0.0f, -vh, 1.5f * vh);
    CHECK(!rotor.solve(p).vortexRing);
}

void axialDescent()
{
    Rotor rotor(mainRotor(), section());
    rotor.setControls(1.5f * kDeg, 0.0f, 0.0f);
    f32 previousThrust = rotor.solve(hover()).thrust;
    f32 crossing = 0.0f;
    f32 previousTorque = rotor.loads().torque;
    f32 crossingThrust = 0.0f;
    for (f32 descent = 0.25f; descent <= 25.0f; descent += 0.25f)
    {
        RotorOperatingPoint p = hover();
        p.velocity = KM::vec3(0.0f, -descent, 0.0f);
        const RotorLoads& l = rotor.solve(p);
        CHECK(isfinite(l.thrust) && isfinite(l.torque));
        CHECK(fabsf(l.thrust - previousThrust) < 0.06f * fabsf(previousThrust) + 150.0f);
        if (crossing == 0.0f && previousTorque > 0.0f && l.torque <= 0.0f)
        {
            crossing = descent;
            crossingThrust = l.thrust;
        }
        previousThrust = l.thrust;
        previousTorque = l.torque;
    }
    CHECK(crossing > 0.0f);
    const f32 vh = idealInduced(crossingThrust, rotor);
    printf("axial autorotation (zero torque) at %.1f m/s descent, %.2f v_h at that thrust\n",
           double(crossing), double(crossing / vh));
    // Measured: ideal autorotation around 1.7 to 1.8 v_h; with profile drag
    // a little faster.
    CHECK(crossing / vh > 1.4f && crossing / vh < 2.3f);
}

void flappingLag()
{
    Rotor rotor(mainRotor(), section());
    rotor.setControls(8.0f * kDeg, 0.0f, 0.0f);
    rotor.solve(hover());
    Rotor target = rotor;
    target.setControls(8.0f * kDeg, 3.0f * kDeg, 0.0f);
    const f32 goal = target.solve(hover()).discTiltForward;

    rotor.setControls(8.0f * kDeg, 3.0f * kDeg, 0.0f);
    const f32 tau = 16.0f / (rotor.lockNumber(kRho) * kOmega);
    const f32 step = 1.0f / 600.0f;
    f32 time = 0.0f;
    f32 atTau = 0.0f;
    while (time < 10.0f * tau)
    {
        rotor.advance(hover(), step);
        time += step;
        if (atTau == 0.0f && time >= tau)
            atTau = rotor.loads().discTiltForward;
    }
    printf("tip-path-plane lag: tau %.3f s, %.0f%% of the way after tau\n", double(tau),
           double(100.0f * atTau / goal));
    CHECK(atTau > 0.5f * goal && atTau < 0.75f * goal);
    CHECK_NEAR(rotor.loads().discTiltForward, goal, 0.01 * goal);
    CHECK_NEAR(rotor.loads().thrust, target.loads().thrust, 0.002 * target.loads().thrust);

    Rotor coarse(mainRotor(), section());
    coarse.setControls(8.0f * kDeg, 0.0f, 0.0f);
    coarse.solve(hover());
    coarse.setControls(8.0f * kDeg, 3.0f * kDeg, 0.0f);
    f32 previous = 0.0f;
    for (int i = 0; i < 20; ++i)
    {
        const f32 tilt = coarse.advance(hover(), 0.1f).discTiltForward;
        CHECK(tilt >= previous - 1e-5f);
        CHECK(tilt <= 1.01f * goal);
        previous = tilt;
    }
    CHECK_NEAR(previous, goal, 0.02 * goal);
}

void degenerateInputs()
{
    Rotor rotor(mainRotor(), section());
    rotor.setControls(8.0f * kDeg, 0.0f, 0.0f);
    RotorOperatingPoint stopped;
    stopped.angularSpeed = 0.0f;
    stopped.velocity = KM::vec3(0.0f, 0.0f, 20.0f);
    const RotorLoads s = rotor.solve(stopped);
    CHECK(isfinite(s.thrust) && isfinite(s.torque) && isfinite(s.force.x));
    CHECK(s.coning == 0.0f);
    rotor.reset();
    const RotorLoads still = rotor.solve(RotorOperatingPoint());
    CHECK(still.thrust == 0.0f && still.power == 0.0f);
    RotorOperatingPoint bad = hover();
    bad.velocity.x = NAN;
    const RotorLoads nan = rotor.solve(bad);
    CHECK(nan.thrust == 0.0f && isfinite(nan.torque));
    bad = hover();
    bad.density = 0.0f;
    CHECK(rotor.solve(bad).thrust == 0.0f);
    rotor.setControls(NAN, NAN, NAN);
    CHECK(rotor.collective() == 0.0f && rotor.longitudinalCyclic() == 0.0f);
    rotor.setControls(10.0f * kDeg, 0.1f, -0.1f);
    RotorOperatingPoint wild = hover();
    wild.velocity = KM::vec3(30.0f, -40.0f, -60.0f);
    wild.angularVelocity = KM::vec3(3.0f, -2.0f, 4.0f);
    for (int i = 0; i < 30; ++i)
    {
        const RotorLoads& l = rotor.advance(wild, 1.0f / 60.0f);
        CHECK(isfinite(l.thrust) && isfinite(l.torque) && isfinite(l.moment.x) && isfinite(l.moment.z));
        CHECK(fabsf(l.coning) <= 0.351f && fabsf(l.longitudinalFlapping) <= 0.351f);
    }
    Rotor::Description d = mainRotor();
    d.shaftAxis = KM::vec3(0.0f);
    d.radius = -1.0f;
    d.rootCutout = 10.0f;
    const Rotor repaired(d, section());
    CHECK(repaired.description().radius > 0.0f);
    CHECK(repaired.description().rootCutout < repaired.description().radius);
    CHECK_NEAR(KM::length(repaired.shaftAxis()), 1.0, 1e-5);
}
}

int main()
{
    derivedProperties();
    hoverMomentumTheory();
    thrustScaling();
    tipLossReducesThrust();
    translationalLift();
    blowback();
    cyclicSigns(1.0f);
    cyclicSigns(-1.0f);
    rotationMirrors();
    rateDamping();
    groundEffect();
    vortexRingFlag();
    axialDescent();
    flappingLag();
    degenerateInputs();
    return TEST_RESULT();
}
