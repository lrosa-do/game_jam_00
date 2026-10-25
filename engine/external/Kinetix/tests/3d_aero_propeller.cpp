#include "check.h"

#include <dynamics/Atmosphere.h>
#include <dynamics/PistonEngine.h>
#include <dynamics/Propeller.h>
#include <dynamics/RigidBody.h>

#include <math.h>

using namespace kinetix::Physics;
using kinetix::f32;
using kinetix::u32;
namespace KM = kinetix::Math;

namespace
{
constexpr f32 kPi = 3.14159265358979f;
constexpr f32 kRpm = 2.0f * kPi / 60.0f;

Propeller::Description lightPropeller()
{
    Propeller::Description d;
    d.radius = 0.95f;
    d.hubRadius = 0.12f;
    d.bladeCount = 2;
    d.rootChord = 0.17f;
    d.tipChord = 0.09f;
    d.pitch = 1.6f;
    d.position = KM::vec3(0.0f, 0.0f, 2.0f);
    d.axis = KM::vec3(0.0f, 0.0f, 1.0f);
    d.inertia = 2.2f;
    return d;
}

void staticThrust()
{
    const Propeller prop(lightPropeller(), Airfoil::propeller());
    const f32 omega = 2400.0f * kRpm;
    const PropellerLoads loads = prop.evaluate(omega, 0.0f, 1.225f);
    CHECK(isfinite(loads.thrust) && isfinite(loads.torque));
    CHECK(loads.thrust > 1000.0f && loads.thrust < 6000.0f);
    CHECK(loads.torque > 0.0f);
    CHECK_NEAR(loads.power, loads.torque * omega, 1e-3 * loads.power);

    // Momentum theory bounds a hovering disc: the ideal power for thrust T is
    // T^1.5 / sqrt(2 rho A), so the figure of merit must stay below one. Real
    // fixed-pitch propellers manage 0.5 to 0.8 at static.
    const f32 area = kPi * 0.95f * 0.95f;
    const f32 ideal = powf(loads.thrust, 1.5f) / sqrtf(2.0f * 1.225f * area);
    const f32 figureOfMerit = ideal / loads.power;
    printf("static: T %.0f N, Q %.0f N m, P %.1f kW, figure of merit %.2f\n",
           double(loads.thrust), double(loads.torque), double(loads.power / 1000.0f),
           double(figureOfMerit));
    CHECK(figureOfMerit > 0.4f && figureOfMerit < 0.95f);

    // Thrust grows roughly with rpm^2 at static (it is all blade speed).
    const PropellerLoads slower = prop.evaluate(1200.0f * kRpm, 0.0f, 1.225f);
    CHECK_NEAR(loads.thrust / slower.thrust, 4.0, 0.4);

    Propeller::Description fine = lightPropeller();
    fine.elementCount = 48;
    const Propeller finer(fine, Airfoil::propeller());
    CHECK_NEAR(finer.evaluate(omega, 0.0f, 1.225f).thrust, loads.thrust, 0.03 * loads.thrust);
}

// Zero thrust where the advance ratio J = V / (n D) brings every blade
// station to its zero-lift angle: just past pitch / diameter for a cambered
// section, just below it for a symmetric one (profile drag alone).
f32 zeroThrustAdvanceRatio(const Propeller& prop, f32 omega)
{
    f32 previous = prop.evaluate(omega, 0.0f, 1.225f).thrust;
    for (f32 v = 0.5f; v < 200.0f; v += 0.5f)
    {
        const f32 thrust = prop.evaluate(omega, v, 1.225f).thrust;
        if (thrust <= 0.0f)
        {
            const f32 crossing = v - 0.5f * thrust / (thrust - previous);
            return prop.advanceRatio(omega, crossing);
        }
        previous = thrust;
    }
    return -1.0f;
}

void advanceRatio()
{
    const Propeller prop(lightPropeller(), Airfoil::propeller());
    const f32 omega = 2400.0f * kRpm;
    const f32 pitchRatio = 1.6f / 1.9f;

    // Fixed-pitch inner stations are stalled at low J and un-stall as the inflow grows: flat within a few percent of the static value, falling steadily from J ~ 0.3.
    const f32 staticThrust = prop.evaluate(omega, 0.0f, 1.225f).thrust;
    f32 previous = staticThrust;
    bool decreasing = true;
    bool flatAtLowJ = true;
    bool efficient = true;
    f32 bestEfficiency = 0.0f;
    for (f32 v = 1.0f; v < 70.0f; v += 1.0f)
    {
        const PropellerLoads loads = prop.evaluate(omega, v, 1.225f);
        if (prop.advanceRatio(omega, v) > 0.3f && !(loads.thrust < previous))
            decreasing = false;
        if (loads.thrust > 1.05f * staticThrust)
            flatAtLowJ = false;
        previous = loads.thrust;
        CHECK_NEAR(loads.power, loads.torque * omega, 1e-3 * fabsf(loads.power) + 1e-3);
        if (loads.thrust > 0.0f && loads.power > 0.0f)
        {
            const f32 efficiency = loads.thrust * v / loads.power;
            bestEfficiency = fmaxf(bestEfficiency, efficiency);
            if (!(efficiency < 1.0f))
                efficient = false;
        }
    }
    CHECK(decreasing);
    CHECK(flatAtLowJ);
    CHECK(efficient);
    CHECK(bestEfficiency > 0.7f);

    const f32 zero = zeroThrustAdvanceRatio(prop, omega);
    const Propeller symmetric(lightPropeller(), Airfoil::symmetric());
    const f32 zeroSymmetric = zeroThrustAdvanceRatio(symmetric, omega);
    printf("zero thrust at J = %.3f (cambered), %.3f (symmetric); pitch/diameter %.3f; best "
           "efficiency %.2f\n",
           double(zero), double(zeroSymmetric), double(pitchRatio), double(bestEfficiency));
    CHECK(zero > pitchRatio && zero < 1.3f * pitchRatio);
    CHECK(zeroSymmetric > 0.85f * pitchRatio && zeroSymmetric < pitchRatio);
    const f32 atPitch = prop.evaluate(omega, pitchRatio * (omega / (2.0f * kPi)) * 1.9f, 1.225f).thrust;
    CHECK(fabsf(atPitch) < 0.2f * prop.evaluate(omega, 0.0f, 1.225f).thrust);

    CHECK_NEAR(prop.advanceRatio(omega, 0.0f), 0.0, 1e-6);
    CHECK_NEAR(prop.advanceRatio(40.0f * 2.0f * kPi, 38.0f), 0.5, 1e-5);
}

void degenerateOperatingPoints()
{
    const Propeller prop(lightPropeller(), Airfoil::propeller());
    const PropellerLoads idle = prop.evaluate(0.0f, 0.0f, 1.225f);
    CHECK(idle.thrust == 0.0f && idle.torque == 0.0f && idle.power == 0.0f);

    // Stopped in a 30 m/s airstream: pure drag, and a torque that would spin
    // it up - it windmills.
    const PropellerLoads stopped = prop.evaluate(0.0f, 30.0f, 1.225f);
    CHECK(isfinite(stopped.thrust) && isfinite(stopped.torque));
    CHECK(stopped.thrust < 0.0f);
    CHECK(stopped.torque < 0.0f);

    const f32 omega = 1500.0f * kRpm;
    const PropellerLoads braking = prop.evaluate(omega, 120.0f, 1.225f);
    CHECK(isfinite(braking.thrust) && braking.thrust < 0.0f);
    const PropellerLoads reversed = prop.evaluate(omega, -15.0f, 1.225f);
    CHECK(isfinite(reversed.thrust) && reversed.thrust > 0.0f);
    const PropellerLoads crawl = prop.evaluate(1.0e-4f, 1.0e-4f, 1.225f);
    CHECK(isfinite(crawl.thrust) && isfinite(crawl.torque));
    const PropellerLoads garbage = prop.evaluate(NAN, 10.0f, 1.225f);
    CHECK(garbage.thrust == 0.0f && garbage.torque == 0.0f);
    const PropellerLoads vacuum = prop.evaluate(omega, 10.0f, 0.0f);
    CHECK(vacuum.thrust == 0.0f);
}

void reactionOnTheAirframe()
{
    Atmosphere air;
    air.setConstantDensity(1.225f);
    RigidBody body;
    body.setMass(1000.0f);
    body.setInertiaTensor(KM::mat3(1000.0f));

    for (f32 rotation : {1.0f, -1.0f})
    {
        Propeller::Description d = lightPropeller();
        d.rotation = rotation;
        Propeller prop(d, Airfoil::propeller());
        prop.setRpm(2400.0f);
        CHECK(prop.mode() == Propeller::Mode::Rpm);
        CHECK_NEAR(prop.rpm(), 2400.0, 1e-2);

        KM::vec3 force;
        KM::vec3 torque;
        const PropellerLoads loads = prop.computeForces(body, air, force, torque);
        // Thrust along the axis, and the airframe twisted against the
        // rotation by exactly the shaft torque.
        CHECK_NEAR(force.z, loads.thrust, 1e-2);
        CHECK_NEAR(force.x, 0.0, 1e-4);
        CHECK(loads.torque > 0.0f);
        CHECK_NEAR(torque.z, -rotation * loads.torque, 1e-2);
        CHECK(torque.z * rotation < 0.0f);
    }

    Propeller prop(lightPropeller(), Airfoil::propeller());
    prop.setRpm(2400.0f);
    const KM::quat pitchedUp = KM::angleAxis(-0.5f, KM::vec3(1.0f, 0.0f, 0.0f));
    body.setOrientation(pitchedUp);
    KM::vec3 force;
    KM::vec3 torque;
    const PropellerLoads loads = prop.computeForces(body, air, force, torque);
    const KM::vec3 axis = pitchedUp * KM::vec3(0.0f, 0.0f, 1.0f);
    CHECK_NEAR(KM::dot(force, axis), loads.thrust, 1e-2);
    CHECK_NEAR(KM::length(force), loads.thrust, 1e-2);

    // Gyroscopic: a rotor spinning about +Z on a body yawing about +Y needs
    // w x H to turn it; the airframe gets H x w = -I omega X.
    body.setOrientation(KM::quat());
    body.setAngularVelocity(KM::vec3(0.0f, 1.0f, 0.0f));
    prop.computeForces(body, air, force, torque);
    CHECK_NEAR(torque.x, -2.2f * prop.angularSpeed(), 0.01f * 2.2f * prop.angularSpeed());

    body.setAngularVelocity(KM::vec3(0.0f));
    KM::vec3 applied;
    KM::vec3 appliedTorque;
    prop.apply(body, air, 1.0f / 60.0f, &applied, &appliedTorque);
    CHECK_NEAR(applied.z, prop.loads().thrust, 1e-2);
}

void rotorDynamics()
{
    // A test stand: a static body, so only the rotor moves.
    Atmosphere air;
    air.setConstantDensity(1.225f);
    RigidBody stand;
    stand.setBodyType(BodyType::Static);

    Propeller prop(lightPropeller(), Airfoil::propeller());
    prop.setShaftTorque(350.0f);
    CHECK(prop.mode() == Propeller::Mode::Torque);
    CHECK_NEAR(prop.rpm(), 0.0, 1e-6);

    f32 previous = 0.0f;
    bool rising = true;
    for (int i = 0; i < 60 * 20; ++i)
    {
        prop.apply(stand, air, 1.0f / 60.0f);
        if (i < 60 && !(prop.angularSpeed() > previous))
            rising = false;
        previous = prop.angularSpeed();
    }
    CHECK(rising);
    const PropellerLoads loads = prop.evaluate(prop.angularSpeed(), 0.0f, 1.225f);
    printf("350 N m settles at %.0f rpm (aero torque %.1f N m)\n", double(prop.rpm()),
           double(loads.torque));
    CHECK_NEAR(loads.torque, 350.0, 0.5);

    // The airframe feels the shaft torque, not the aerodynamic one.
    KM::vec3 force;
    KM::vec3 torque;
    prop.setShaftTorque(100.0f);
    prop.computeForces(stand, air, force, torque);
    CHECK_NEAR(torque.z, -100.0, 1e-3);

    prop.setShaftTorque(0.0f);
    for (int i = 0; i < 60 * 60; ++i)
        prop.apply(stand, air, 1.0f / 60.0f);
    CHECK(prop.angularSpeed() >= 0.0f);
    CHECK(prop.rpm() < 300.0f);

    // A tiny rotor inertia on a long step does not overshoot into a limit
    // cycle: the aerodynamic torque is treated implicitly.
    Propeller::Description light = lightPropeller();
    light.inertia = 0.01f;
    Propeller twitchy(light, Airfoil::propeller());
    twitchy.setShaftTorque(350.0f);
    for (int i = 0; i < 600; ++i)
        twitchy.apply(stand, air, 1.0f / 30.0f);
    const f32 settled = twitchy.angularSpeed();
    twitchy.apply(stand, air, 1.0f / 30.0f);
    CHECK(isfinite(settled));
    CHECK_NEAR(twitchy.angularSpeed(), settled, 0.01f * settled);
    CHECK_NEAR(twitchy.evaluate(settled, 0.0f, 1.225f).torque, 350.0, 1.0);
}

void pistonEngine()
{
    PistonEngine::Description d;
    d.maxPower = 120000.0f;
    d.ratedRpm = 2700.0f;
    d.maxRpm = 2800.0f;
    PistonEngine engine(d);
    const f32 rated = 2700.0f * kRpm;

    // Full-throttle power before losses: P_max at the rated speed, and less
    // either side of it - the cubic peaks there.
    CHECK_NEAR(engine.fullThrottleTorque(rated) * rated, 120000.0, 1.0);
    CHECK(engine.fullThrottleTorque(0.9f * rated) * 0.9f * rated < 120000.0f);
    CHECK(engine.fullThrottleTorque(1.1f * rated) * 1.1f * rated < 120000.0f);
    CHECK_NEAR(engine.fullThrottleTorque(0.0f), engine.ratedTorque(), 1e-3);
    CHECK_NEAR(engine.fullThrottleTorque(0.5f * rated), 1.25 * engine.ratedTorque(), 1e-2);

    engine.setThrottle(0.0f);
    const f32 idle = engine.shaftTorque(700.0f * kRpm, 1.225f);
    engine.setThrottle(0.5f);
    const f32 half = engine.shaftTorque(700.0f * kRpm, 1.225f);
    engine.setThrottle(1.0f);
    const f32 full = engine.shaftTorque(700.0f * kRpm, 1.225f);
    CHECK(idle > 0.0f);
    CHECK(half > idle && full > half);
    engine.setThrottle(3.0f);
    CHECK_NEAR(engine.throttle(), 1.0, 1e-6);

    // Thinner air, less power: Gagg-Ferrar at 5000 m.
    const f32 sigma = 0.736429f / 1.225f;
    const f32 seaLevel = engine.shaftTorque(2400.0f * kRpm, 1.225f);
    const f32 high = engine.shaftTorque(2400.0f * kRpm, 0.736429f);
    const f32 friction = engine.ratedTorque() *
                         (d.frictionStatic + d.frictionPerRpm * 2400.0f / 2700.0f);
    CHECK_NEAR((high + friction) / (seaLevel + friction), sigma - (1.0f - sigma) / 7.55f, 1e-3);

    CHECK(engine.shaftTorque(2950.0f * kRpm, 1.225f) < 0.0f);
    engine.setThrottle(0.0f);
    PistonEngine::Description weak = d;
    weak.idleThrottle = 0.0f;
    PistonEngine dead(weak);
    CHECK_NEAR(dead.shaftTorque(0.0f, 1.225f), 0.0, 1e-6);
    CHECK_NEAR(dead.shaftTorque(NAN, 1.225f), 0.0, 1e-6);
}
}

int main()
{
    staticThrust();
    advanceRatio();
    degenerateOperatingPoints();
    reactionOnTheAirframe();
    rotorDynamics();
    pistonEngine();
    return TEST_RESULT();
}
