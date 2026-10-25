#include "check.h"

#include <dynamics/Atmosphere.h>
#include <dynamics/RigidBody.h>
#include <dynamics/Wing.h>

#include <math.h>

using namespace kinetix::Physics;
using kinetix::f32;
using kinetix::u32;
namespace KM = kinetix::Math;

namespace
{
constexpr f32 kPi = 3.14159265358979f;
constexpr f32 kDeg = kPi / 180.0f;

// Rectangular, untwisted, symmetric section: 10 m span, 1 m chord, AR 10.
Wing rectangularWing(f32 dihedral = 0.0f, u32 elements = 8)
{
    Wing::Description d;
    d.root = KM::vec3(0.0f, 0.0f, 0.0f);
    d.tip = KM::vec3(5.0f * cosf(dihedral), 5.0f * sinf(dihedral), 0.0f);
    d.rootChord = 1.0f;
    d.tipChord = 1.0f;
    d.mirrored = true;
    d.elementCount = elements;
    d.oswaldEfficiency = 0.8f;
    return Wing(d, Airfoil::symmetric());
}

void prepareBody(RigidBody& body)
{
    body.setMass(100.0f);
    body.setInertiaTensor(KM::mat3(100.0f));
    body.setPosition(KM::vec3(0.0f));
}

// Moving forward (+Z) at `speed` with the air arriving `alpha` from below.
KM::vec3 flightVelocity(f32 speed, f32 alpha)
{
    return KM::vec3(0.0f, -sinf(alpha), cosf(alpha)) * speed;
}

void liftMatchesLiftingLine()
{
    Atmosphere air;
    air.setConstantDensity(1.225f);
    RigidBody body;
    prepareBody(body);

    Wing wing = rectangularWing();
    CHECK_NEAR(wing.area(), 10.0, 1e-4);
    CHECK_NEAR(wing.span(), 10.0, 1e-4);
    CHECK_NEAR(wing.aspectRatio(), 10.0, 1e-3);

    const f32 a0 = 2.0f * kPi;
    const f32 expectedSlope = a0 / (1.0f + a0 / (kPi * 0.8f * 10.0f));
    CHECK_NEAR(wing.finiteLiftSlope(), expectedSlope, 1e-4);

    const f32 speed = 40.0f;
    const f32 alpha = 4.0f * kDeg;
    body.setVelocity(flightVelocity(speed, alpha));
    const WingState state = wing.computeForces(body, air);
    const f32 q = 0.5f * 1.225f * speed * speed;
    const f32 expectedCL = expectedSlope * alpha;
    printf("wing CL %.4f, lifting line %.4f (%.2f%%)\n", double(state.lift / (q * 10.0f)),
           double(expectedCL), double(100.0f * (state.lift / (q * 10.0f) / expectedCL - 1.0f)));
    CHECK_NEAR(state.lift / (q * 10.0f), expectedCL, 0.005 * expectedCL);
    CHECK_NEAR(state.liftCoefficient, expectedCL, 0.005 * expectedCL);
    CHECK_NEAR(state.inducedAngle, expectedCL / (kPi * 0.8f * 10.0f), 1e-4);
    // Drag: profile Cd0 + k Cl^2 plus induced CL^2 / (pi e AR).
    const Airfoil::Parameters& p = wing.airfoil().parameters();
    const f32 sectionCl = a0 * (alpha - state.inducedAngle);
    const f32 expectedCD = p.minDrag + p.liftDragFactor * sectionCl * sectionCl +
                           expectedCL * expectedCL / (kPi * 0.8f * 10.0f);
    CHECK_NEAR(state.drag / (q * 10.0f), expectedCD, 0.03 * expectedCD);

    Wing fine = rectangularWing(0.0f, 32);
    CHECK_NEAR(fine.computeForces(body, air).lift, state.lift, 1e-3 * state.lift);

    // Same total lift from a fin-like, very low aspect ratio wing, where a0 >
    // pi e AR and the plain fixed-point downwash iteration would diverge.
    Wing::Description stubby;
    stubby.root = KM::vec3(0.0f);
    stubby.tip = KM::vec3(1.0f, 0.0f, 0.0f);
    stubby.rootChord = 1.0f;
    stubby.tipChord = 1.0f;
    stubby.mirrored = true;
    stubby.oswaldEfficiency = 0.8f;
    Wing low(stubby, Airfoil::symmetric());
    CHECK_NEAR(low.aspectRatio(), 2.0, 1e-4);
    const WingState lowState = low.computeForces(body, air);
    const f32 lowSlope = a0 / (1.0f + a0 / (kPi * 0.8f * 2.0f));
    CHECK_NEAR(lowState.lift / (q * 2.0f), lowSlope * alpha, 0.005 * lowSlope * alpha);
}

void scaling()
{
    Atmosphere air;
    air.setConstantDensity(1.225f);
    RigidBody body;
    prepareBody(body);
    Wing wing = rectangularWing();
    const f32 alpha = 5.0f * kDeg;

    body.setVelocity(flightVelocity(30.0f, alpha));
    const f32 slow = wing.computeForces(body, air).lift;
    body.setVelocity(flightVelocity(60.0f, alpha));
    const f32 fast = wing.computeForces(body, air).lift;
    CHECK_NEAR(fast / slow, 4.0, 1e-3);

    Atmosphere thin;
    thin.setConstantDensity(0.6125f);
    const f32 halfDensity = wing.computeForces(body, thin).lift;
    CHECK_NEAR(halfDensity / fast, 0.5, 1e-4);

    body.setVelocity(flightVelocity(60.0f, 0.0f));
    const WingState level = wing.computeForces(body, air);
    CHECK_NEAR(level.lift, 0.0, 1e-3);
    CHECK_NEAR(level.force.y, 0.0, 1e-3);
    CHECK(level.force.z < 0.0f);
    CHECK_NEAR(level.torque.x, 0.0, 1e-3);
    CHECK_NEAR(level.torque.z, 0.0, 1e-3);

    body.setVelocity(KM::vec3(0.0f));
    const WingState still = wing.computeForces(body, air);
    CHECK(still.force.x == 0.0f && still.force.y == 0.0f && still.force.z == 0.0f);
}

void forceDirection()
{
    Atmosphere air;
    air.setConstantDensity(1.225f);
    RigidBody body;
    prepareBody(body);
    Wing wing = rectangularWing();

    for (f32 degrees : {3.0f, 8.0f, -6.0f})
    {
        const f32 alpha = degrees * kDeg;
        const KM::vec3 velocity = flightVelocity(50.0f, alpha);
        body.setVelocity(velocity);
        const WingState state = wing.computeForces(body, air);
        const KM::vec3 flow = KM::normalize(-velocity);
        const f32 along = KM::dot(state.force, flow);
        const KM::vec3 across = state.force - flow * along;
        CHECK_NEAR(along, state.drag, 1e-2);
        CHECK(along > 0.0f);
        CHECK_NEAR(KM::dot(across, flow), 0.0, 1e-2);
        CHECK_NEAR(across.x, 0.0, 1e-3);
        CHECK_NEAR(KM::length(across), fabsf(state.lift), 1e-2);
        const KM::vec3 liftDirection(0.0f, cosf(alpha), sinf(alpha));
        CHECK_NEAR(KM::dot(across, liftDirection), state.lift, 1e-2);
        CHECK((state.lift > 0.0f) == (alpha > 0.0f));
    }

    body.setVelocity(flightVelocity(50.0f, 5.0f * kDeg));
    const KM::vec3 reference = wing.computeForces(body, air).force;
    const KM::quat turn = KM::angleAxis(1.1f, KM::normalize(KM::vec3(0.3f, 1.0f, -0.4f)));
    body.setOrientation(turn);
    body.setVelocity(turn * flightVelocity(50.0f, 5.0f * kDeg));
    const KM::vec3 rotated = body.directionToLocal(wing.computeForces(body, air).force);
    CHECK_NEAR(rotated.x, reference.x, 0.05);
    CHECK_NEAR(rotated.y, reference.y, 0.05);
    CHECK_NEAR(rotated.z, reference.z, 0.05);
}

void rollDamping()
{
    Atmosphere air;
    air.setConstantDensity(1.225f);
    RigidBody body;
    prepareBody(body);
    Wing wing = rectangularWing();

    for (f32 rate : {0.5f, -0.5f})
    {
        body.setVelocity(flightVelocity(40.0f, 3.0f * kDeg));
        body.setAngularVelocity(KM::vec3(0.0f, 0.0f, rate));
        const WingState state = wing.computeForces(body, air);
        CHECK(state.torque.z * rate < 0.0f);
        // Strip theory: a strip at x sees its angle change by -p x / V, so
        // the moment is -1/2 rho V a0 c p * integral of x^2 over the span,
        // (2/3) (b/2)^3. The section slope a0, not the finite-wing one: an
        // antisymmetric load leaves the whole-wing CL, and so the uniform
        // downwash, unchanged.
        const f32 expected = -0.5f * 1.225f * 40.0f * 2.0f * kPi * 1.0f * rate * 2.0f * 125.0f / 3.0f;
        printf("roll damping %.0f N m, strip theory %.0f N m\n", double(state.torque.z),
               double(expected));
        CHECK_NEAR(state.torque.z, expected, 0.05f * fabsf(expected));
    }
}

void dihedralEffect()
{
    Atmosphere air;
    air.setConstantDensity(1.225f);
    RigidBody body;
    prepareBody(body);
    const f32 beta = 8.0f * kDeg;
    body.setVelocity(KM::vec3(-40.0f * sinf(beta), -40.0f * sinf(3.0f * kDeg),
                              40.0f * cosf(beta)));

    Wing flat = rectangularWing(0.0f);
    Wing dihedral = rectangularWing(6.0f * kDeg);
    const f32 flatRoll = flat.computeForces(body, air).torque.z;
    const f32 dihedralRoll = dihedral.computeForces(body, air).torque.z;
    // The windward (right) wing rises: rolling left, away from the wind,
    // which is a negative moment about the forward axis.
    CHECK(dihedralRoll < 0.0f);
    CHECK(fabsf(flatRoll) < 0.05f * fabsf(dihedralRoll));

    Wing anhedral = rectangularWing(-6.0f * kDeg);
    CHECK(anhedral.computeForces(body, air).torque.z > 0.0f);
}

void windIsSubtracted()
{
    // At rest in a 40 m/s headwind arriving 4 degrees from below is the same flow as flying through still air.
    Atmosphere still;
    still.setConstantDensity(1.225f);
    Atmosphere windy;
    windy.setConstantDensity(1.225f);
    const KM::vec3 velocity = flightVelocity(40.0f, 4.0f * kDeg);
    windy.setWind(-velocity);

    RigidBody flying;
    prepareBody(flying);
    flying.setVelocity(velocity);
    RigidBody parked;
    prepareBody(parked);

    Wing a = rectangularWing();
    Wing b = rectangularWing();
    const WingState moving = a.computeForces(flying, still);
    const WingState resting = b.computeForces(parked, windy);
    CHECK(moving.lift > 1000.0f);
    CHECK_NEAR(resting.lift, moving.lift, 1e-3 * moving.lift);
    CHECK_NEAR(resting.force.y, moving.force.y, 1e-3 * fabsf(moving.force.y));
    CHECK_NEAR(resting.force.z, moving.force.z, 1e-3 * fabsf(moving.force.z) + 1e-3);

    RigidBody drifting;
    prepareBody(drifting);
    drifting.setVelocity(-velocity);
    const WingState drift = a.computeForces(drifting, windy);
    CHECK_NEAR(KM::length(drift.force), 0.0, 1e-4);
}

void controlSurfaces()
{
    Atmosphere air;
    air.setConstantDensity(1.225f);
    RigidBody body;
    prepareBody(body);
    body.setVelocity(flightVelocity(40.0f, 2.0f * kDeg));

    Wing wing = rectangularWing();
    ControlSurface aileron;
    aileron.spanStart = 0.5f;
    aileron.spanEnd = 1.0f;
    aileron.chordFraction = 0.25f;
    aileron.maxDeflection = 15.0f * kDeg;
    aileron.minDeflection = 15.0f * kDeg;
    const u32 index = wing.addControlSurface(aileron);
    CHECK(index == 0);
    const WingState neutral = wing.computeForces(body, air);

    wing.setControlCommand(index, 1.0f, 1.0f);
    const WingState down = wing.computeForces(body, air);
    CHECK(down.lift > neutral.lift * 1.1f);
    CHECK(down.torque.x > neutral.torque.x);
    CHECK_NEAR(down.torque.z, 0.0, 1.0);

    // Antisymmetric: the primary (+X, left) panel down rolls right, which is
    // a positive moment about forward.
    wing.setControlCommand(index, 1.0f, -1.0f);
    const WingState roll = wing.computeForces(body, air);
    CHECK(roll.torque.z > 100.0f);
    CHECK_NEAR(roll.lift, neutral.lift, 0.05f * neutral.lift);

    CHECK(wing.element(0).control < 0);
    CHECK(wing.element(7).control == 0);
    wing.setControlCommand(5, 1.0f, 1.0f);

    wing.setControlCommand(index, 4.0f, -9.0f);
    CHECK_NEAR(wing.controlSurface(index).command, 1.0, 1e-6);
    CHECK_NEAR(wing.controlSurface(index).mirroredCommand, -1.0, 1e-6);
}

void verticalFin()
{
    Atmosphere air;
    air.setConstantDensity(1.225f);
    RigidBody body;
    prepareBody(body);

    // A fin behind the centre of mass, lift positive to the left (+X).
    Wing::Description d;
    d.root = KM::vec3(0.0f, 0.0f, -4.0f);
    d.tip = KM::vec3(0.0f, 1.5f, -4.0f);
    d.rootChord = 1.2f;
    d.tipChord = 0.8f;
    d.up = KM::vec3(1.0f, 0.0f, 0.0f);
    d.aspectRatio = 2.5f;
    Wing fin(d, Airfoil::symmetric());
    CHECK_NEAR(fin.aspectRatio(), 2.5, 1e-5);
    CHECK_NEAR(fin.area(), 1.5, 1e-4);

    // Wind from the right: the fin is pushed left, which swings the nose
    // right - into the wind. Weathercock stability.
    const f32 beta = 5.0f * kDeg;
    body.setVelocity(KM::vec3(-sinf(beta), 0.0f, cosf(beta)) * 40.0f);
    const WingState state = fin.computeForces(body, air);
    CHECK(state.force.x > 0.0f);
    // Nose right is a negative rotation about +Y.
    CHECK(state.torque.y < 0.0f);
    CHECK_NEAR(state.force.y, 0.0, 0.05f * state.force.x);
}
}

int main()
{
    liftMatchesLiftingLine();
    scaling();
    forceDirection();
    rollDamping();
    dihedralEffect();
    windIsSubtracted();
    controlSurfaces();
    verticalFin();
    return TEST_RESULT();
}
