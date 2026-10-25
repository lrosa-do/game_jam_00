#include "check.h"

#include <dynamics/Atmosphere.h>
#include <dynamics/PistonEngine.h>
#include <dynamics/Propeller.h>
#include <dynamics/RigidBody.h>
#include <dynamics/Wing.h>
#include <marine/Hydrofoil.h>
#include <marine/MarinePropeller.h>
#include <marine/WaterSurface.h>

#include <math.h>

using namespace kinetix::Physics;
using kinetix::f32;
using kinetix::u32;
namespace KM = kinetix::Math;

namespace
{
constexpr f32 kPi = 3.14159265358979f;
constexpr f32 kDeg = kPi / 180.0f;
constexpr f32 kRpm = 2.0f * kPi / 60.0f;

bool finiteVec(const KM::vec3& v)
{
    return isfinite(v.x) && isfinite(v.y) && isfinite(v.z);
}

FlatWater testWater()
{
    FlatWater water(0.0f);
    WaterProperties properties;
    properties.density = 1000.0f;
    water.setProperties(properties);
    return water;
}

Wing::Description rudder(f32 top)
{
    Wing::Description d;
    d.root = KM::vec3(0.0f, top, 0.0f);
    d.tip = KM::vec3(0.0f, top - 0.6f, 0.0f);
    d.rootChord = 0.3f;
    d.tipChord = 0.3f;
    d.forward = KM::vec3(0.0f, 0.0f, 1.0f);
    d.up = KM::vec3(1.0f, 0.0f, 0.0f);
    d.elementCount = 12;
    return d;
}

// Body moving through still water at `speed`, flow meeting the rudder at `alpha` (sideslip towards -X is positive alpha).
void move(RigidBody& body, f32 speed, f32 alpha, f32 depth)
{
    body.setPosition(KM::vec3(0.0f, -depth, 0.0f));
    body.setVelocity(KM::vec3(-sinf(alpha), 0.0f, cosf(alpha)) * speed);
    body.calculateDerivedData();
}

void hydrofoilIsWingInWater()
{
    FlatWater water = testWater();
    RigidBody body;
    move(body, 5.0f, 4.0f * kDeg, 3.0f);
    Hydrofoil foil(rudder(0.0f), Airfoil::symmetric());
    const WingState& wet = foil.computeForces(body, water);
    CHECK_NEAR(foil.immersion(), 1.0, 1e-6);

    Wing wing(rudder(0.0f), Airfoil::symmetric());
    Atmosphere dense;
    dense.setConstantDensity(1000.0f);
    const WingState& dry = wing.computeForces(body, dense);
    printf("rudder at 5 m/s, 4 deg: side force %.1f N in water, %.1f N as a wing in rho = 1000\n",
           double(wet.force.x), double(dry.force.x));
    CHECK_NEAR(wet.force.x, dry.force.x, 1e-4 * fabsf(dry.force.x));
    CHECK_NEAR(wet.force.z, dry.force.z, 1e-4 * fabsf(dry.force.x));
    CHECK_NEAR(KM::length(wet.torque - dry.torque), 0.0, 1e-4 * KM::length(dry.torque));
    // Lift towards +X for flow from ahead and to port: alpha positive.
    CHECK(wet.force.x > 0.0f);
    // q S CL_alpha alpha, give or take the drag's share.
    const f32 q = 0.5f * 1000.0f * 25.0f;
    const f32 expected = q * 0.18f * wing.finiteLiftSlope() * 4.0f * kDeg;
    CHECK_NEAR(wet.lift, expected, 0.02 * expected);

    // The same foil in sea-level air: 1/816 of it.
    Atmosphere air;
    const f32 inAir = wing.computeForces(body, air).force.x;
    CHECK_NEAR(wet.force.x / inAir, 1000.0 / 1.225, 0.5);
}

void hydrofoilImmersion()
{
    FlatWater water = testWater();
    RigidBody body;
    Hydrofoil full(rudder(0.0f), Airfoil::symmetric());
    move(body, 4.0f, 5.0f * kDeg, 1.0f);
    const f32 deep = full.computeForces(body, water).force.x;

    move(body, 4.0f, 5.0f * kDeg, -0.3f + 0.0f);
    body.setPosition(KM::vec3(0.0f, 0.0f, 0.0f));
    body.calculateDerivedData();
    Hydrofoil half(rudder(0.3f), Airfoil::symmetric());
    const f32 halfForce = half.computeForces(body, water).force.x;
    printf("rudder half out of the water: immersion %.3f, side force %.1f N of %.1f N\n",
           double(half.immersion()), double(halfForce), double(deep));
    CHECK_NEAR(half.immersion(), 0.5, 0.01);
    CHECK_NEAR(halfForce, 0.5 * deep, 0.02 * deep);

    body.setPosition(KM::vec3(0.0f, 1.0f, 0.0f));
    body.calculateDerivedData();
    const WingState& clear = half.computeForces(body, water);
    CHECK(half.immersion() == 0.0f);
    CHECK(KM::length(clear.force) == 0.0f);

    // A foil lying flat right at the surface switches on gradually: half
    // its load with the surface through its chord line.
    Wing::Description flat;
    flat.root = KM::vec3(0.0f, 0.0f, 0.0f);
    flat.tip = KM::vec3(1.0f, 0.0f, 0.0f);
    flat.rootChord = 0.2f;
    flat.tipChord = 0.2f;
    flat.mirrored = true;
    Hydrofoil foil(flat, Airfoil::symmetric());
    f32 previous = -1.0f;
    bool monotonic = true;
    for (int i = -10; i <= 10; ++i)
    {
        body.setPosition(KM::vec3(0.0f, 0.004f * static_cast<f32>(i), 0.0f));
        body.calculateDerivedData();
        foil.computeForces(body, water);
        if (foil.immersion() > previous + 1.0e-6f && i > -10)
            monotonic = false;
        previous = foil.immersion();
    }
    CHECK(monotonic);
    body.setPosition(KM::vec3(0.0f));
    body.calculateDerivedData();
    foil.computeForces(body, water);
    CHECK_NEAR(foil.immersion(), 0.5, 1e-5);
}

void hydrofoilDeflectionAndCurrent()
{
    FlatWater water = testWater();
    RigidBody body;
    Hydrofoil foil(rudder(0.0f), Airfoil::symmetric());
    foil.setMaxDeflection(30.0f * kDeg);
    move(body, 4.0f, 0.0f, 1.0f);
    CHECK_NEAR(foil.computeForces(body, water).force.x, 0.0, 1e-3);
    // Leading edge towards +X: lift towards +X.
    foil.setDeflection(10.0f * kDeg);
    const WingState& turned = foil.computeForces(body, water);
    CHECK(turned.force.x > 0.0f);
    const f32 deflected = turned.lift;
    // The same lift as meeting the flow at 10 degrees undeflected (the
    // force differs by where the drag points).
    Hydrofoil plain(rudder(0.0f), Airfoil::symmetric());
    move(body, 4.0f, 10.0f * kDeg, 1.0f);
    CHECK_NEAR(plain.computeForces(body, water).lift, deflected, 1e-3 * deflected);
    foil.setDeflection(80.0f * kDeg);
    CHECK_NEAR(foil.deflection(), 30.0 * kDeg, 1e-6);

    // Held still in a 4 m/s current against it: the same as moving at
    // 4 m/s through still water.
    move(body, 4.0f, 5.0f * kDeg, 1.0f);
    const f32 moving = plain.computeForces(body, water).force.x;
    const KM::vec3 velocity = body.velocity();
    body.setVelocity(KM::vec3(0.0f));
    water.setCurrent(-velocity);
    const f32 still = plain.computeForces(body, water).force.x;
    CHECK_NEAR(still, moving, 1e-3 * fabsf(moving));
    body.setVelocity(-velocity);
    CHECK_NEAR(plain.computeForces(body, water).force.x, 0.0, 1e-3);
}

MarinePropeller::Description outboard()
{
    MarinePropeller::Description p;
    p.blades.radius = 0.178f;
    p.blades.hubRadius = 0.045f;
    p.blades.bladeCount = 3;
    p.blades.rootChord = 0.13f;
    p.blades.tipChord = 0.11f;
    p.blades.pitch = 0.43f;
    p.blades.inertia = 0.02f;
    p.blades.elementCount = 10;
    p.blades.position = KM::vec3(0.0f, -0.5f, -3.0f);
    p.gearRatio = 2.0f;
    p.maxSteeringAngle = 30.0f * kDeg;
    return p;
}

void propellerInWater()
{
    MarinePropeller propeller(outboard(), Airfoil::propeller());
    const f32 omega = 2700.0f * kRpm;
    const PropellerLoads water = propeller.evaluate(omega, 8.0f, 1000.0f);
    const PropellerLoads direct = propeller.blades().evaluate(omega, 8.0f, 1000.0f);
    CHECK(water.thrust == direct.thrust && water.torque == direct.torque);
    CHECK(water.thrust > 0.0f && water.torque > 0.0f);
    CHECK_NEAR(water.power, water.torque * omega, 1e-3 * water.power);
    // At the bollard (static), thrust is proportional to density.
    const PropellerLoads wet = propeller.evaluate(omega, 0.0f, 1000.0f);
    const PropellerLoads dry = propeller.evaluate(omega, 0.0f, 1.225f);
    printf("propeller at 2700 rpm: bollard thrust %.0f N in water, %.2f N in air (ratio %.1f), "
           "%.0f N at 8 m/s\n",
           double(wet.thrust), double(dry.thrust), double(wet.thrust / dry.thrust), double(water.thrust));
    CHECK_NEAR(wet.thrust / dry.thrust, 1000.0 / 1.225, 0.01 * 1000.0 / 1.225);

    const PropellerLoads astern = propeller.evaluate(-omega, -2.0f, 1000.0f);
    const PropellerLoads mirrored = propeller.evaluate(omega, 2.0f, 1000.0f);
    CHECK_NEAR(astern.thrust, -0.6 * mirrored.thrust, 1e-3 * mirrored.thrust);
    CHECK_NEAR(astern.torque, -mirrored.torque, 1e-3 * mirrored.torque);
    CHECK(astern.power > 0.0f);

    const PropellerLoads half = propeller.evaluate(omega, 0.0f, 1000.0f, 0.5f);
    CHECK_NEAR(half.thrust, 0.5 * wet.thrust, 1e-3 * wet.thrust);
    const PropellerLoads none = propeller.evaluate(omega, 0.0f, 1000.0f, 0.0f);
    CHECK(none.thrust == 0.0f && none.torque == 0.0f);
}

void propellerImmersionAndSteering()
{
    FlatWater water = testWater();
    MarinePropeller propeller(outboard(), Airfoil::propeller());
    RigidBody body;
    // Hub on the surface: half the disc under.
    body.setPosition(KM::vec3(0.0f, 0.5f, 0.0f));
    body.calculateDerivedData();
    CHECK_NEAR(propeller.immersion(body, water), 0.5, 1e-5);
    body.setPosition(KM::vec3(0.0f, 0.5f - 0.178f, 0.0f));
    body.calculateDerivedData();
    CHECK_NEAR(propeller.immersion(body, water), 1.0, 1e-5);
    body.setPosition(KM::vec3(0.0f, 0.5f + 0.178f, 0.0f));
    body.calculateDerivedData();
    CHECK_NEAR(propeller.immersion(body, water), 0.0, 1e-5);
    // A quarter radius under: the segment formula.
    body.setPosition(KM::vec3(0.0f, 0.5f - 0.25f * 0.178f, 0.0f));
    body.calculateDerivedData();
    const f32 u = 0.25f;
    CHECK_NEAR(propeller.immersion(body, water), (kPi - acosf(u) + u * sqrtf(1.0f - u * u)) / kPi, 1e-4);

    // Steering turns the thrust line about +Y: +30 degrees points it to
    // +X, pushing the stern to port.
    body.setPosition(KM::vec3(0.0f, 0.0f, 0.0f));
    body.calculateDerivedData();
    propeller.setSteering(1.0f);
    CHECK_NEAR(propeller.steering(), 30.0 * kDeg, 1e-6);
    const KM::vec3 axis = propeller.thrustAxis(body);
    CHECK_NEAR(axis.x, sinf(30.0f * kDeg), 1e-5);
    CHECK_NEAR(axis.z, cosf(30.0f * kDeg), 1e-5);
    propeller.setPropellerRpm(2500.0f);
    KM::vec3 force;
    KM::vec3 torque;
    const PropellerLoads loads = propeller.computeForces(body, water, force, torque);
    CHECK(loads.thrust > 0.0f);
    CHECK(force.x > 0.0f);
    // Behind the centre of mass and pushing to +X: the bow swings to -X
    // (starboard), a negative moment about +Y.
    CHECK(torque.y < 0.0f);
    CHECK_NEAR(KM::length(force), loads.thrust * (1.0f - 0.06f), 1e-3 * loads.thrust);
    // The reaction to a right-handed rotor's torque, about -axis.
    propeller.setSteering(0.0f);
    propeller.computeForces(body, water, force, torque);
    CHECK(torque.z < 0.0f);
}

void engineAndGearbox()
{
    FlatWater water = testWater();
    PistonEngine::Description e;
    e.maxPower = 75000.0f;
    e.ratedRpm = 5500.0f;
    e.maxRpm = 6000.0f;
    e.idleThrottle = 0.036f;
    PistonEngine engine(e);
    engine.setThrottle(1.0f);

    for (int lifted = 0; lifted < 2; ++lifted)
    {
        MarinePropeller propeller(outboard(), Airfoil::propeller());
        RigidBody body;
        body.setBodyType(BodyType::Kinematic);
        body.setPosition(KM::vec3(0.0f, lifted ? 3.0f : 0.0f, 0.0f));
        body.calculateDerivedData();
        propeller.setGear(MarinePropeller::Gear::Forward);
        f32 peak = 0.0f;
        f32 worstThrust = 0.0f;
        for (int i = 0; i < 60 * 5; ++i)
        {
            propeller.setEngineTorque(engine.shaftTorque(propeller.engineAngularSpeed(), 1.225f));
            const PropellerLoads& loads = propeller.apply(body, water, 1.0f / 60.0f);
            peak = fmaxf(peak, propeller.engineRpm());
            worstThrust = fmaxf(worstThrust, fabsf(loads.thrust));
            body.clearAccumulators();
        }
        printf("full throttle %s: engine %.0f rpm (peak %.0f), propeller %.0f rpm, thrust %.0f N\n",
               lifted ? "lifted clear" : "at the bollard", double(propeller.engineRpm()), double(peak),
               double(propeller.rpm()), double(propeller.loads().thrust));
        CHECK(isfinite(propeller.engineRpm()));
        CHECK_NEAR(propeller.rpm(), propeller.engineRpm() / 2.0, 1e-2);
        if (lifted)
        {
            CHECK(worstThrust == 0.0f);
            CHECK(propeller.lastImmersion() == 0.0f);
            // Up against the limiter, and no further past its 100 rpm band
            // than one step's overshoot.
            CHECK(propeller.engineRpm() > 5900.0f);
            CHECK(peak < 6150.0f);
        }
        else
        {
            CHECK(propeller.engineRpm() > 2000.0f && propeller.engineRpm() < 5500.0f);
            CHECK(propeller.loads().thrust > 3000.0f);
            const f32 load = propeller.loads().torque / (2.0f * 0.95f);
            CHECK_NEAR(load, engine.shaftTorque(propeller.engineAngularSpeed(), 1.225f), 0.01 * load);
        }

        propeller.setGear(MarinePropeller::Gear::Neutral);
        CHECK(propeller.angularSpeed() == 0.0f);
        propeller.setGear(MarinePropeller::Gear::Reverse);
        CHECK(propeller.rpm() < 0.0f);
    }
}
}

int main()
{
    hydrofoilIsWingInWater();
    hydrofoilImmersion();
    hydrofoilDeflectionAndCurrent();
    propellerInWater();
    propellerImmersionAndSteering();
    engineAndGearbox();
    return TEST_RESULT();
}
