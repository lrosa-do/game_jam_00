#include "check.h"
#include "actuator_rig.h"

#include <actuators/HydraulicCylinder.h>
#include <dynamics/ArticulationSolver.h>
#include <dynamics/DistanceJoint.h>
#include <dynamics/FixedJoint.h>
#include <dynamics/HingeJoint.h>
#include <dynamics/PointJoint.h>
#include <dynamics/SliderJoint.h>

#include <chrono>

using namespace actuator_test;

namespace
{

double seconds()
{
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

ServoSettings servo(f32 frequency, f32 damping, f32 maxTorque)
{
    ServoSettings s;
    s.frequency = frequency;
    s.dampingRatio = damping;
    s.maxForce = maxTorque;
    return s;
}

f32 freeChain(bool articulated, f32 hz, double* microseconds)
{
    Rig rig(hz, true);
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
        hinges[i]->setArticulated(articulated);
        rig.world.addJoint(hinges[i]);
        localParent[i] = parent.pointToLocal(pivot);
        localChild[i] = links[i]->pointToLocal(pivot);
    }
    f32 worst = 0.0f;
    bool finite = true;
    const int steps = static_cast<int>(4.0f * hz);
    const double start = seconds();
    for (int s = 0; s < steps; ++s)
    {
        rig.world.step(rig.step);
        for (u32 i = 0; i < 3; ++i)
        {
            const RigidBody& parent = i == 0 ? base : *links[i - 1];
            worst = fmaxf(worst, KM::length(parent.pointToWorld(localParent[i]) - links[i]->pointToWorld(localChild[i])));
            finite = finite && finiteVec(links[i]->position());
        }
    }
    if (microseconds)
        *microseconds = (seconds() - start) / steps * 1.0e6;
    CHECK(finite);
    CHECK(rig.world.articulation().groupCount() == (articulated ? 1u : 0u));
    for (u32 i = 0; i < 3; ++i)
    {
        rig.world.removeJoint(hinges[i]);
        delete hinges[i];
    }
    return worst;
}

void freeChainStaysClosed()
{
    double iterativeCost = 0.0;
    double articulatedCost = 0.0;
    const f32 iterative = freeChain(false, 60.0f, &iterativeCost);
    const f32 articulated = freeChain(true, 60.0f, &articulatedCost);
    const f32 articulated120 = freeChain(true, 120.0f, nullptr);
    printf("free 3-link chain at 60 Hz: iterative opens %.3e m (%.1f us/step), articulated %.3e m "
           "(%.1f us/step); articulated at 120 Hz %.3e m\n",
           double(iterative), iterativeCost, double(articulated), articulatedCost, double(articulated120));
    CHECK(iterative > 1.0e-2f);
    CHECK(articulated < 1.0e-4f);
    CHECK(articulated * 100.0f < iterative);
    CHECK(articulated120 < 1.0e-4f);
}

struct Arm
{
    Rig rig;
    RigidBody* base = nullptr;
    RigidBody* links[3] = {};
    HingeJoint* hinges[3] = {};
    HydraulicCylinder* cylinders[3] = {};
    KM::vec3 localAnchorParent[3];
    KM::vec3 localAnchorChild[3];
    f32 valveSign[3] = {1.0f, 1.0f, 1.0f};

    explicit Arm(bool articulated) : rig(60.0f, true)
    {
        const KM::vec3 z(0.0f, 0.0f, 1.0f);
        base = &rig.base(KM::vec3(0.0f));
        const KM::vec3 boomPivot(0.0f, 1.0f, 0.0f);
        const KM::vec3 boomDirection = KM::normalize(KM::vec3(cosf(0.5f), sinf(0.5f), 0.0f));
        const KM::vec3 stickPivot = boomPivot + boomDirection * 3.0f;
        const KM::vec3 stickDirection = KM::normalize(KM::vec3(0.5f, -1.0f, 0.0f));
        const KM::vec3 bucketPivot = stickPivot + stickDirection * 2.0f;
        const KM::vec3 bucketDirection = KM::normalize(KM::vec3(1.0f, -0.3f, 0.0f));
        const KM::vec3 pivots[3] = {boomPivot, stickPivot, bucketPivot};
        const KM::vec3 directions[3] = {boomDirection, stickDirection, bucketDirection};
        const f32 lengths[3] = {3.0f, 2.0f, 0.8f};
        const f32 masses[3] = {900.0f, 450.0f, 300.0f};
        for (u32 i = 0; i < 3; ++i)
        {
            const KM::vec3 centre = pivots[i] + directions[i] * (0.5f * lengths[i]);
            RigidBody& link = rig.box(centre, KM::vec3(0.5f * lengths[i], 0.15f, 0.15f), masses[i]);
            link.setOrientation(KM::angleAxis(atan2f(directions[i].y, directions[i].x), z));
            link.calculateDerivedData();
            links[i] = &link;
            RigidBody& parent = i == 0 ? *base : *links[i - 1];
            hinges[i] = new HingeJoint(parent, link, pivots[i], z);
            hinges[i]->setArticulated(articulated);
            rig.world.addJoint(hinges[i]);
            localAnchorParent[i] = parent.pointToLocal(pivots[i]);
            localAnchorChild[i] = link.pointToLocal(pivots[i]);
        }
        const KM::vec3 boomNormal = KM::cross(z, boomDirection);
        const KM::vec3 stickNormal = KM::cross(z, stickDirection);
        const KM::vec3 barrels[3] = {KM::vec3(0.7f, 0.2f, 0.0f),
                                     boomPivot + boomDirection * 1.6f + boomNormal * 0.5f,
                                     stickPivot + stickDirection * 0.3f + stickNormal * 0.35f};
        const KM::vec3 rods[3] = {boomPivot + boomDirection * 1.2f - boomNormal * 0.2f,
                                  stickPivot - stickDirection * 0.5f + stickNormal * 0.1f,
                                  bucketPivot + stickNormal * 0.35f};
        RigidBody* barrelBodies[3] = {base, links[0], links[1]};
        const f32 bores[3] = {0.16f, 0.13f, 0.11f};
        for (u32 i = 0; i < 3; ++i)
        {
            HydraulicCylinderParameters p;
            p.boreDiameter = bores[i];
            p.rodDiameter = 0.6f * bores[i];
            p.stroke = 1.2f;
            p.initialStroke = 0.6f;
            p.pumpFlow = 120.0f;
            p.reliefPressure = 30.0e6f;
            cylinders[i] = new HydraulicCylinder(*barrelBodies[i], barrels[i], *links[i], rods[i], p);
            cylinders[i]->setArticulated(articulated);
            rig.world.addJoint(cylinders[i]);
            const KM::vec3 line = KM::normalize(rods[i] - barrels[i]);
            valveSign[i] = KM::dot(KM::cross(rods[i] - pivots[i], line), z) > 0.0f ? 1.0f : -1.0f;
        }
    }
    ~Arm()
    {
        for (u32 i = 0; i < 3; ++i)
        {
            rig.world.removeJoint(cylinders[i]);
            rig.world.removeJoint(hinges[i]);
            delete cylinders[i];
            delete hinges[i];
        }
    }
    f32 separation(u32 i) const
    {
        const RigidBody& parent = i == 0 ? *base : *links[i - 1];
        return KM::length(parent.pointToWorld(localAnchorParent[i]) - links[i]->pointToWorld(localAnchorChild[i]));
    }
    void steer(const f32* targets)
    {
        for (u32 i = 0; i < 3; ++i)
            cylinders[i]->setValve(KM::clamp(8.0f * (targets[i] - hinges[i]->currentAngle()) * valveSign[i], -1.0f, 1.0f));
    }
};

void excavatorArm(bool articulated, f32* worstOut, f32* holdingOut, double* microseconds)
{
    Arm arm(articulated);
    const f32 poses[3][3] = {{0.0f, 0.0f, 0.0f}, {0.25f, -0.3f, 0.5f}, {-0.15f, 0.25f, -0.4f}};
    f32 worst = 0.0f;
    bool finite = true;
    const double start = seconds();
    int steps = 0;
    for (u32 pose = 0; pose < 3; ++pose)
    {
        for (int s = 0; s < 6 * 60; ++s, ++steps)
        {
            arm.steer(poses[pose]);
            arm.rig.world.step(arm.rig.step);
            for (u32 i = 0; i < 3; ++i)
            {
                worst = fmaxf(worst, arm.separation(i));
                finite = finite && finiteVec(arm.links[i]->position());
            }
        }
        for (u32 i = 0; i < 3; ++i)
            CHECK_NEAR(arm.hinges[i]->currentAngle(), poses[pose][i], 0.01f);
    }
    *microseconds = (seconds() - start) / steps * 1.0e6;
    for (u32 i = 0; i < 3; ++i)
        arm.cylinders[i]->setValve(0.0f);
    f32 angles[3];
    for (u32 i = 0; i < 3; ++i)
        angles[i] = arm.hinges[i]->currentAngle();
    arm.rig.run(5.0f);
    f32 holding = 0.0f;
    f32 drift = 0.0f;
    for (u32 i = 0; i < 3; ++i)
    {
        holding = fmaxf(holding, arm.separation(i));
        drift = fmaxf(drift, fabsf(arm.hinges[i]->currentAngle() - angles[i]));
    }
    CHECK(finite);
    CHECK(drift < 1.0e-3f);
    *worstOut = worst;
    *holdingOut = holding;
}

void excavatorArmStaysClosed()
{
    f32 iterative = 0.0f;
    f32 iterativeHolding = 0.0f;
    f32 articulated = 0.0f;
    f32 articulatedHolding = 0.0f;
    double iterativeCost = 0.0;
    double articulatedCost = 0.0;
    excavatorArm(false, &iterative, &iterativeHolding, &iterativeCost);
    excavatorArm(true, &articulated, &articulatedHolding, &articulatedCost);
    printf("excavator arm: iterative opens %.3e m (%.3e holding, %.1f us/step), articulated %.3e m "
           "(%.3e holding, %.1f us/step)\n",
           double(iterative), double(iterativeHolding), iterativeCost, double(articulated),
           double(articulatedHolding), articulatedCost);
    CHECK(iterative > 3.0e-3f);
    CHECK(articulated < 1.0e-4f);
    CHECK(articulated * 10.0f < iterative);
    CHECK(articulatedHolding < 1.0e-5f);
}

// A cylinder in an articulated arm still moves at flow / area.
void cylinderSpeedThroughArticulation()
{
    Rig rig(60.0f, true);
    const KM::vec3 z(0.0f, 0.0f, 1.0f);
    RigidBody& base = rig.base(KM::vec3(0.0f));
    RigidBody& boom = rig.box(KM::vec3(1.5f, 1.0f, 0.0f), KM::vec3(1.5f, 0.15f, 0.15f), 1000.0f);
    HingeJoint hinge(base, boom, KM::vec3(0.0f, 1.0f, 0.0f), z);
    hinge.setArticulated(true);
    rig.world.addJoint(&hinge);
    HydraulicCylinderParameters p;
    p.boreDiameter = 0.12f;
    p.rodDiameter = 0.07f;
    p.stroke = 1.0f;
    p.initialStroke = 0.3f;
    p.pumpFlow = 100.0f;
    p.valveResponseTime = 0.0f;
    p.cushionLength = 0.0f;
    HydraulicCylinder cylinder(base, KM::vec3(0.8f, 0.0f, 0.0f), boom, KM::vec3(1.2f, 0.85f, 0.0f), p);
    cylinder.setArticulated(true);
    rig.world.addJoint(&cylinder);
    // The stroke's rate over a quarter of a second (the instantaneous
    // velocity is along the line at the end of the step, which has turned
    // with the boom since the row was built).
    cylinder.setValve(1.0f);
    rig.run(0.25f);
    f32 before = cylinder.stroke();
    rig.run(0.25f);
    const f32 extending = (cylinder.stroke() - before) / 0.25f;
    cylinder.setValve(-1.0f);
    rig.run(0.25f);
    before = cylinder.stroke();
    rig.run(0.25f);
    const f32 retracting = (cylinder.stroke() - before) / 0.25f;
    printf("articulated cylinder: %.5f m/s extending (%.5f), %.5f retracting (%.5f)\n", double(extending),
           double(cylinder.maxExtendSpeed()), double(retracting), double(-cylinder.maxRetractSpeed()));
    CHECK_NEAR(extending, cylinder.maxExtendSpeed(), 1.0e-4f * cylinder.maxExtendSpeed() + 1.0e-6f);
    CHECK_NEAR(retracting, -cylinder.maxRetractSpeed(), 1.0e-4f * cylinder.maxRetractSpeed() + 1.0e-6f);
    cylinder.setValve(0.0f);
    rig.run(0.2f);
    const f32 held = hinge.currentAngle();
    rig.run(3.0f);
    CHECK_NEAR(hinge.currentAngle(), held, 1.0e-5f);
    rig.world.removeJoint(&cylinder);
    rig.world.removeJoint(&hinge);
}

// Planar four-bar: 20 lock rows on 3 bodies leave 1 DOF only if 3 rows are redundant;
// the factorisation finds and drops them.
void fourBarLoop(bool distanceCoupler)
{
    Rig rig(60.0f, true);
    const KM::vec3 z(0.0f, 0.0f, 1.0f);
    RigidBody& ground = rig.base(KM::vec3(0.0f));
    // Ground pivots O2 = (0,0), O4 = (2,0); crank 0.6 m, coupler 2.2 m,
    // rocker 1.2 m: a Grashof crank-rocker.
    const KM::vec3 o2(0.0f, 0.0f, 0.0f);
    const KM::vec3 o4(2.0f, 0.0f, 0.0f);
    const f32 crank = 0.6f;
    const f32 coupler = 2.2f;
    const f32 rocker = 1.2f;
    const KM::vec3 a = o2 + KM::vec3(0.0f, crank, 0.0f);
    // B: |B - A| = coupler, |B - O4| = rocker, above the ground line.
    const f32 d = KM::length(o4 - a);
    const f32 along = (coupler * coupler - rocker * rocker + d * d) / (2.0f * d);
    const f32 across = sqrtf(coupler * coupler - along * along);
    const KM::vec3 axis = (o4 - a) / d;
    const KM::vec3 normal(-axis.y, axis.x, 0.0f);
    const KM::vec3 b = a + axis * along + normal * across;
    auto barBetween = [&](const KM::vec3& p, const KM::vec3& q, f32 mass) -> RigidBody&
    {
        const KM::vec3 delta = q - p;
        RigidBody& bar = rig.box((p + q) * 0.5f, KM::vec3(0.5f * KM::length(delta), 0.04f, 0.04f), mass);
        bar.setOrientation(KM::angleAxis(atan2f(delta.y, delta.x), z));
        bar.calculateDerivedData();
        return bar;
    };
    RigidBody& crankBar = barBetween(o2, a, 5.0f);
    RigidBody& rockerBar = barBetween(o4, b, 8.0f);
    HingeJoint h1(ground, crankBar, o2, z);
    HingeJoint h4(ground, rockerBar, o4, z);
    RigidBody* couplerBar = nullptr;
    HingeJoint* h2 = nullptr;
    HingeJoint* h3 = nullptr;
    DistanceJoint* link = nullptr;
    Joint* joints[4] = {&h1, &h4, nullptr, nullptr};
    if (distanceCoupler)
    {
        link = new DistanceJoint(crankBar, a, rockerBar, b);
        joints[2] = link;
    }
    else
    {
        couplerBar = &barBetween(a, b, 10.0f);
        h2 = new HingeJoint(crankBar, *couplerBar, a, z);
        h3 = new HingeJoint(*couplerBar, rockerBar, b, z);
        joints[2] = h2;
        joints[3] = h3;
    }
    for (Joint* joint : joints)
        if (joint)
        {
            joint->setArticulated(true);
            rig.world.addJoint(joint);
        }
    h1.setMotor(2.0f, 2000.0f);
    const KM::vec3 localA = crankBar.pointToLocal(a);
    const KM::vec3 localB = rockerBar.pointToLocal(b);
    const KM::vec3 couplerA = couplerBar ? couplerBar->pointToLocal(a) : KM::vec3(0.0f);
    const KM::vec3 couplerB = couplerBar ? couplerBar->pointToLocal(b) : KM::vec3(0.0f);
    f32 worstLoop = 0.0f;
    f32 worstRocker = 0.0f;
    bool finite = true;
    for (int s = 0; s < 6 * 60; ++s)
    {
        rig.world.step(rig.step);
        const KM::vec3 pa = crankBar.pointToWorld(localA);
        const KM::vec3 pb = rockerBar.pointToWorld(localB);
        if (couplerBar)
            worstLoop = fmaxf(worstLoop, fmaxf(KM::length(couplerBar->pointToWorld(couplerA) - pa),
                                               KM::length(couplerBar->pointToWorld(couplerB) - pb)));
        else
            worstLoop = fmaxf(worstLoop, fabsf(KM::length(pb - pa) - coupler));
        // Analytic closure: from the crank angle, where the rocker's pin
        // has to be (the same branch, above the ground line).
        const f32 theta = h1.currentAngle() + 0.5f * kPi;
        const KM::vec3 crankPin = o2 + KM::vec3(cosf(theta), sinf(theta), 0.0f) * crank;
        const KM::vec3 toO4 = o4 - crankPin;
        const f32 dd = KM::length(toO4);
        const f32 al = (coupler * coupler - rocker * rocker + dd * dd) / (2.0f * dd);
        const f32 ac = sqrtf(fmaxf(coupler * coupler - al * al, 0.0f));
        const KM::vec3 u = toO4 / dd;
        const KM::vec3 expected = crankPin + u * al + KM::vec3(-u.y, u.x, 0.0f) * ac;
        worstRocker = fmaxf(worstRocker, KM::length(pb - expected));
        finite = finite && finiteVec(rockerBar.position()) && finiteVec(crankBar.angularVelocity());
    }
    printf("four-bar (%s): crank at %.3f rad/s, loop closure %.3e m, rocker pin vs analytic %.3e m, "
           "lock rows %u\n",
           distanceCoupler ? "distance coupler" : "four hinges", double(h1.currentAngularVelocity()),
           double(worstLoop), double(worstRocker), rig.world.articulation().rowCount());
    CHECK(finite);
    CHECK(worstLoop < 1.0e-4f);
    CHECK(worstRocker < 1.0e-3f);
    CHECK_NEAR(h1.currentAngularVelocity(), 2.0f, 0.05f);
    for (Joint* joint : joints)
        if (joint)
            rig.world.removeJoint(joint);
    delete link;
    delete h2;
    delete h3;
}

// Servo gains with the outboard link rigidly held: articulated, the first joint drives both links as one
// body and oscillates at the asked frequency; iterative, the period is sqrt(I_composite / I_own) longer.
f32 servoPeriod(bool articulated)
{
    Rig rig(240.0f, false);
    RigidBody& base = rig.base(KM::vec3(0.0f));
    RigidBody& first = rig.box(KM::vec3(0.5f, 0.0f, 0.0f), KM::vec3(0.5f, 0.05f, 0.05f), 10.0f);
    RigidBody& second = rig.box(KM::vec3(1.5f, 0.0f, 0.0f), KM::vec3(0.5f, 0.05f, 0.05f), 10.0f);
    HingeJoint h0(base, first, KM::vec3(0.0f), KM::vec3(0.0f, 0.0f, 1.0f));
    HingeJoint h1(first, second, KM::vec3(1.0f, 0.0f, 0.0f), KM::vec3(0.0f, 0.0f, 1.0f));
    h0.setArticulated(articulated);
    h1.setArticulated(articulated);
    rig.world.addJoint(&h0);
    rig.world.addJoint(&h1);
    h1.setServo(0.0f, servo(0.0f, 1.0f, 1.0e5f));
    h0.setServo(0.2f, servo(1.0f, 0.05f, 1.0e5f));
    f32 previous = h0.currentAngle() - 0.2f;
    f32 firstCrossing = -1.0f;
    f32 lastCrossing = -1.0f;
    u32 crossings = 0;
    for (int s = 0; s < static_cast<int>(8.0f / rig.step); ++s)
    {
        rig.world.step(rig.step);
        const f32 error = h0.currentAngle() - 0.2f;
        if (previous < 0.0f && error >= 0.0f)
        {
            const f32 t = (s + previous / (previous - error)) * rig.step;
            if (firstCrossing < 0.0f)
                firstCrossing = t;
            lastCrossing = t;
            ++crossings;
        }
        previous = error;
    }
    rig.world.removeJoint(&h1);
    rig.world.removeJoint(&h0);
    return crossings > 1 ? (lastCrossing - firstCrossing) / static_cast<f32>(crossings - 1) : 0.0f;
}

void servoGainsSeeOutboardLinks()
{
    const f32 articulated = servoPeriod(true);
    const f32 iterative = servoPeriod(false);
    // Composite inertia about the first pivot over the first link's own.
    const f32 own = 10.0f / 3.0f;
    const f32 composite = own + 10.0f / 12.0f + 10.0f * 1.5f * 1.5f;
    printf("servo at 1 Hz with an outboard link held rigid: period %.4f s articulated, %.4f s "
           "iterative (sqrt(I_c/I_own) = %.3f)\n",
           double(articulated), double(iterative), double(sqrtf(composite / own)));
    CHECK_NEAR(articulated, 1.0f, 0.03f);
    CHECK_NEAR(iterative / articulated, sqrtf(composite / own), 0.15f * sqrtf(composite / own));
}

// 50 kg arm held horizontal by a rigid servo reads m g up and m g L/2 of torque;
// a hanging 20 kg weight on a slider reads m g on its lock.
void sensorsAndSleep()
{
    Rig rig(60.0f, true);
    RigidBody& base = rig.base(KM::vec3(0.0f));
    RigidBody& arm = rig.box(KM::vec3(1.0f, 0.0f, 0.0f), KM::vec3(1.0f, 0.05f, 0.05f), 50.0f);
    HingeJoint hinge(base, arm, KM::vec3(0.0f), KM::vec3(0.0f, 0.0f, 1.0f));
    hinge.setArticulated(true);
    hinge.setServo(0.0f, servo(0.0f, 1.0f, 5000.0f));
    rig.world.addJoint(&hinge);
    RigidBody& carriage = rig.box(KM::vec3(3.0f, -1.0f, 0.0f), KM::vec3(0.1f), 20.0f);
    SliderJoint slider(base, carriage, KM::vec3(3.0f, -1.0f, 0.0f), KM::vec3(1.0f, 0.0f, 0.0f));
    slider.setArticulated(true);
    rig.world.addJoint(&slider);
    rig.run(2.0f);
    printf("articulated sensors: hinge %.3f N up, %.3f N m; slider lock %.3f N up\n",
           double(hinge.reactionForce().y), double(hinge.motorTorque()), double(slider.reactionForce().y));
    CHECK_NEAR(hinge.reactionForce().y, 50.0f * kG, 1.0e-3f * 50.0f * kG);
    CHECK_NEAR(hinge.motorTorque(), 50.0f * kG * 1.0f, 1.0e-3f * 50.0f * kG);
    CHECK_NEAR(slider.reactionForce().y, 20.0f * kG, 1.0e-3f * 20.0f * kG);
    CHECK_NEAR(hinge.currentAngle(), 0.0f, 1.0e-4f);
    rig.run(4.0f);
    CHECK(!arm.awake());
    CHECK(!carriage.awake());
    hinge.setServoTarget(0.4f);
    rig.run(2.0f);
    CHECK_NEAR(hinge.currentAngle(), 0.4f, 1.0e-3f);
    rig.world.removeJoint(&slider);
    rig.world.removeJoint(&hinge);
}

void mixedJointTypes()
{
    Rig rig(60.0f, true);
    RigidBody& base = rig.base(KM::vec3(0.0f));
    RigidBody& a = rig.box(KM::vec3(0.5f, 0.0f, 0.0f), KM::vec3(0.5f, 0.05f, 0.05f), 30.0f);
    RigidBody& b = rig.box(KM::vec3(1.5f, 0.0f, 0.0f), KM::vec3(0.5f, 0.05f, 0.05f), 30.0f);
    RigidBody& c = rig.box(KM::vec3(2.5f, 0.0f, 0.0f), KM::vec3(0.5f, 0.05f, 0.05f), 30.0f);
    HingeJoint hinge(base, a, KM::vec3(0.0f), KM::vec3(0.0f, 0.0f, 1.0f));
    FixedJoint fixed(a, b, KM::vec3(1.0f, 0.0f, 0.0f));
    PointJoint point(b, c, KM::vec3(2.0f, 0.0f, 0.0f));
    Joint* joints[3] = {&hinge, &fixed, &point};
    for (Joint* joint : joints)
    {
        joint->setArticulated(true);
        rig.world.addJoint(joint);
    }
    const KM::vec3 fixedA = a.pointToLocal(KM::vec3(1.0f, 0.0f, 0.0f));
    const KM::vec3 fixedB = b.pointToLocal(KM::vec3(1.0f, 0.0f, 0.0f));
    const KM::vec3 pointB = b.pointToLocal(KM::vec3(2.0f, 0.0f, 0.0f));
    const KM::vec3 pointC = c.pointToLocal(KM::vec3(2.0f, 0.0f, 0.0f));
    const KM::quat relative = KM::conjugate(a.orientation()) * b.orientation();
    f32 worst = 0.0f;
    f32 twist = 0.0f;
    for (int s = 0; s < 4 * 60; ++s)
    {
        rig.world.step(rig.step);
        worst = fmaxf(worst, KM::length(a.pointToWorld(fixedA) - b.pointToWorld(fixedB)));
        worst = fmaxf(worst, KM::length(b.pointToWorld(pointB) - c.pointToWorld(pointC)));
        const KM::quat now = KM::conjugate(relative) * KM::conjugate(a.orientation()) * b.orientation();
        twist = fmaxf(twist, 2.0f * KM::length(KM::vec3(now.x, now.y, now.z)));
    }
    printf("hinge + fixed + point, articulated: worst opening %.3e m, fixed joint turned %.3e rad\n",
           double(worst), double(twist));
    CHECK(worst < 1.0e-4f);
    CHECK(twist < 1.0e-5f);
    for (Joint* joint : joints)
        rig.world.removeJoint(joint);
}

// 40 links = 200 lock rows, over MaxGroupRows: left to the iterative solver.
void oversizedGroupFallsBack()
{
    Rig rig(60.0f, true);
    RigidBody& base = rig.base(KM::vec3(0.0f));
    constexpr u32 kLinks = 40;
    static_assert(kLinks * 5 > ArticulationSolver::MaxGroupRows, "the chain must be too big");
    ct::Vector<HingeJoint*> hinges;
    // The rig holds 16 bodies; a longer chain gets its own.
    ct::Vector<RigidBody*> links;
    ct::Vector<BoxShape*> shapes;
    ct::Vector<u32> ids;
    RigidBody* parent = &base;
    for (u32 i = 0; i < kLinks; ++i)
    {
        RigidBody* link = new RigidBody();
        link->setBodyType(BodyType::Dynamic);
        link->setMass(1.0f);
        link->setInertiaTensor(Inertia::box(1.0f, KM::vec3(0.1f, 0.02f, 0.02f)));
        link->setPosition(KM::vec3(0.2f * i + 0.1f, 0.0f, 0.0f));
        link->calculateDerivedData();
        BoxShape* shape = new BoxShape(KM::vec3(0.1f, 0.02f, 0.02f));
        BodyEntry entry;
        entry.body = link;
        entry.shape = shape;
        entry.filter.group = 1u << 30;
        entry.filter.mask = 0u;
        ids.push_back(rig.world.addBody(entry));
        HingeJoint* hinge = new HingeJoint(*parent, *link, KM::vec3(0.2f * i, 0.0f, 0.0f), KM::vec3(0.0f, 0.0f, 1.0f));
        hinge->setArticulated(true);
        rig.world.addJoint(hinge);
        hinges.push_back(hinge);
        links.push_back(link);
        shapes.push_back(shape);
        parent = link;
    }
    bool finite = true;
    for (int s = 0; s < 60; ++s)
    {
        rig.world.step(rig.step);
        for (RigidBody* link : links)
            finite = finite && finiteVec(link->position());
    }
    printf("40-link chain: %u articulation groups (iterative fallback), finite %d\n",
           rig.world.articulation().groupCount(), int(finite));
    CHECK(rig.world.articulation().groupCount() == 0);
    CHECK(!hinges[0]->solvedByArticulation());
    CHECK(finite);
    for (HingeJoint* hinge : hinges)
    {
        rig.world.removeJoint(hinge);
        delete hinge;
    }
    for (u32 id : ids)
        rig.world.removeBody(id);
    for (RigidBody* link : links)
        delete link;
    for (BoxShape* shape : shapes)
        delete shape;
}

}

int main()
{
    freeChainStaysClosed();
    excavatorArmStaysClosed();
    cylinderSpeedThroughArticulation();
    fourBarLoop(false);
    fourBarLoop(true);
    servoGainsSeeOutboardLinks();
    sensorsAndSleep();
    mixedJointTypes();
    oversizedGroupFallsBack();
    return TEST_RESULT();
}
