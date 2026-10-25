#include "actuator_rig.h"
#include "check.h"

#include <actuators/HydraulicCylinder.h>
#include <dynamics/HingeJoint.h>
#include <dynamics/SliderJoint.h>

#include <math.h>
#include <stdio.h>

using namespace actuator_test;

namespace
{

HydraulicCylinderParameters cylinder()
{
    HydraulicCylinderParameters p;
    p.boreDiameter = 0.10f;
    p.rodDiameter = 0.05f;
    p.stroke = 1.0f;
    p.initialStroke = 0.2f;
    p.pumpFlow = 100.0f;
    p.reliefPressure = 25.0e6f;
    return p;
}

struct Column
{
    Rig rig;
    RigidBody* base = nullptr;
    RigidBody* load = nullptr;
    SliderJoint* slider = nullptr;
    HydraulicCylinder* cylinder = nullptr;

    Column(f32 mass, const HydraulicCylinderParameters& parameters, bool gravity = true)
        : rig(60.0f, gravity)
    {
        base = &rig.base(KM::vec3(0.0f));
        const f32 height = 1.2f; // closed length 1.0 + initial stroke 0.2
        load = &rig.box(KM::vec3(0.0f, height, 0.0f), KM::vec3(0.3f), mass);
        slider = new SliderJoint(*base, *load, KM::vec3(0.0f, height, 0.0f), KM::vec3(0.0f, 1.0f, 0.0f));
        rig.world.addJoint(slider);
        cylinder = new HydraulicCylinder(*base, KM::vec3(0.0f), *load, KM::vec3(0.0f, height, 0.0f),
                                         parameters);
        rig.world.addJoint(cylinder);
    }
    ~Column()
    {
        rig.world.removeJoint(cylinder);
        rig.world.removeJoint(slider);
        delete cylinder;
        delete slider;
    }
};

void speedIsFlowOverArea()
{
    for (int loaded = 0; loaded < 2; ++loaded)
    {
        Column c(loaded ? 1000.0f : 50.0f, cylinder());
        const f32 capArea = 0.25f * kPi * 0.1f * 0.1f;
        const f32 annulusArea = capArea - 0.25f * kPi * 0.05f * 0.05f;
        const f32 flow = 100.0f / 60000.0f;
        CHECK_NEAR(c.cylinder->maxExtendSpeed(), flow / capArea, 1.0e-6f);
        CHECK_NEAR(c.cylinder->maxRetractSpeed(), flow / annulusArea, 1.0e-6f);

        c.cylinder->setValve(1.0f);
        c.rig.run(1.5f);
        const f32 extend = c.cylinder->strokeVelocity();
        const f32 strokeOut = c.cylinder->stroke();
        c.cylinder->setValve(-1.0f);
        c.rig.run(1.0f);
        const f32 retract = c.cylinder->strokeVelocity();
        printf("%s: extending %.5f m/s (Q/A_cap %.5f) at %.3f m, retracting %.5f (Q/A_ann %.5f)\n",
               loaded ? "1 t" : "50 kg", double(extend), double(flow / capArea), double(strokeOut),
               double(retract), double(-flow / annulusArea));
        CHECK_NEAR(extend, flow / capArea, 0.02f * flow / capArea);
        CHECK_NEAR(retract, -flow / annulusArea, 0.02f * flow / annulusArea);
        CHECK_NEAR(c.cylinder->flow(), 100.0f, 2.0f);

        c.cylinder->setValve(0.5f);
        c.rig.run(0.5f);
        const f32 half = (0.5f - 0.02f) / (1.0f - 0.02f) * flow / capArea;
        CHECK_NEAR(c.cylinder->strokeVelocity(), half, 0.02f * half);
    }
}

// Against a stop it cannot move, the cylinder pushes p * A_cap and pulls
// p * A_annulus, and the relief is open.
void stallsAtReliefPressure()
{
    Column c(200.0f, cylinder());
    // The load's slide ends 0.3 m above where it starts; the cylinder has
    // 0.8 m of stroke left.
    c.slider->setLimits(-0.1f, 0.3f);
    c.cylinder->setValve(1.0f);
    c.rig.run(3.0f);
    const f32 capArea = 0.25f * kPi * 0.1f * 0.1f;
    const f32 push = 25.0e6f * capArea;
    printf("stall pushing: %.1f N (p A_cap %.1f), cap %.3f MPa, relief %d, slide stop %.1f N\n",
           double(c.cylinder->force()), double(push), double(c.cylinder->capPressure() * 1.0e-6f),
           c.cylinder->reliefOpen() ? 1 : 0, double(c.slider->limitForce()));
    CHECK_NEAR(c.cylinder->force(), push, 0.02f * push);
    CHECK_NEAR(c.cylinder->capPressure(), 25.0e6f, 0.02f * 25.0e6f);
    CHECK(c.cylinder->reliefOpen());
    CHECK_NEAR(c.slider->currentPosition(), 0.3f, 2.0e-3f);

    c.cylinder->setValve(-1.0f);
    c.rig.run(4.0f);
    const f32 annulusArea = capArea - 0.25f * kPi * 0.05f * 0.05f;
    const f32 pull = 25.0e6f * annulusArea;
    printf("stall pulling: %.1f N (p A_ann %.1f), rod %.3f MPa\n", double(c.cylinder->force()),
           double(-pull), double(c.cylinder->rodPressure() * 1.0e-6f));
    CHECK_NEAR(c.cylinder->force(), -pull, 0.02f * pull);
    CHECK_NEAR(c.cylinder->rodPressure(), 25.0e6f, 0.02f * 25.0e6f);
    CHECK_NEAR(c.slider->currentPosition(), -0.1f, 2.0e-3f);

    HydraulicCylinderParameters weak = cylinder();
    weak.reliefPressure = 1.0e6f; // 7.85 kN
    Column heavy(1000.0f, weak);
    heavy.rig.run(2.0f);
    printf("1 t on a 7.85 kN relief: stroke %.3f m, relief %d\n", double(heavy.cylinder->stroke()),
           heavy.cylinder->reliefOpen() ? 1 : 0);
    CHECK(heavy.cylinder->stroke() < 0.05f);
    Column light(500.0f, weak);
    light.rig.run(2.0f);
    CHECK_NEAR(light.cylinder->stroke(), 0.2f, 1.0e-3f);
    CHECK(!light.cylinder->reliefOpen());
}

// Oil spring with the bulk modulus of mineral oil: sags by F/k once, then holds.
void holdsTheLoadWithoutDrift()
{
    for (int compressible = 0; compressible < 2; ++compressible)
    {
        HydraulicCylinderParameters p = cylinder();
        if (compressible)
            p.bulkModulus = 1.5e9f;
        Column c(3000.0f, p);
        c.cylinder->setValve(1.0f);
        c.rig.run(1.0f);
        c.cylinder->setValve(0.0f);
        c.rig.run(1.0f);
        const f32 start = c.cylinder->stroke();
        f32 lowest = start;
        f32 highest = start;
        for (int i = 0; i < 10 * 60; ++i)
        {
            c.rig.world.step(c.rig.step);
            lowest = fminf(lowest, c.cylinder->stroke());
            highest = fmaxf(highest, c.cylinder->stroke());
        }
        printf("hold 3 t %s: drift %.3e m over 10 s (range %.3e), force %.1f N, cap %.2f MPa\n",
               compressible ? "compressible" : "rigid", double(c.cylinder->stroke() - start),
               double(highest - lowest), double(c.cylinder->force()),
               double(c.cylinder->capPressure() * 1.0e-6f));
        CHECK(fabsf(c.cylinder->stroke() - start) < 1.0e-3f);
        CHECK(highest - lowest < 1.0e-3f);
        CHECK_NEAR(c.cylinder->force(), 3000.0f * kG, 0.01f * 3000.0f * kG);
    }
}

void cushionsAndStops()
{
    Column c(100.0f, cylinder(), false);
    c.cylinder->setValve(1.0f);
    f32 slowest = 1.0f;
    for (int i = 0; i < 6 * 60; ++i)
    {
        c.rig.world.step(c.rig.step);
        if (c.cylinder->stroke() > 0.99f && c.cylinder->stroke() < 0.999f)
            slowest = fminf(slowest, c.cylinder->strokeVelocity());
    }
    printf("cushion: %.4f m/s in the last centimetre, stroke %.5f, stop force %.1f N\n",
           double(slowest), double(c.cylinder->stroke()), double(c.cylinder->endStopForce()));
    CHECK(slowest < 0.4f * c.cylinder->maxExtendSpeed());
    CHECK_NEAR(c.cylinder->stroke(), 1.0f, 1.0e-3f);
    c.load->addForce(KM::vec3(0.0f, 5.0e5f, 0.0f));
    c.rig.world.step(c.rig.step);
    CHECK(c.cylinder->stroke() < 1.0f + 2.0e-3f);
}

struct Excavator
{
    Rig rig;
    RigidBody* base = nullptr;
    RigidBody* links[3] = {};
    HingeJoint* hinges[3] = {};
    HydraulicCylinder* cylinders[3] = {};
    KM::vec3 localAnchorParent[3];
    KM::vec3 localAnchorChild[3];
    f32 valveSign[3] = {1.0f, 1.0f, 1.0f};

    Excavator() : rig(60.0f, true)
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
            const f32 angle = atan2f(directions[i].y, directions[i].x);
            link.setOrientation(KM::angleAxis(angle, z));
            link.calculateDerivedData();
            links[i] = &link;
            RigidBody& parent = i == 0 ? *base : *links[i - 1];
            hinges[i] = new HingeJoint(parent, link, pivots[i], z);
            rig.world.addJoint(hinges[i]);
            localAnchorParent[i] = parent.pointToLocal(pivots[i]);
            localAnchorChild[i] = link.pointToLocal(pivots[i]);
        }
        // Bucket cylinder is square to its lever so +-0.5 rad stays clear of the dead centre of a direct cylinder.
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
            rig.world.addJoint(cylinders[i]);
            // Extending turns the hinge by the moment of the cylinder's line about the pivot.
            const KM::vec3 line = KM::normalize(rods[i] - barrels[i]);
            const f32 moment = KM::dot(KM::cross(rods[i] - pivots[i], line), z);
            valveSign[i] = moment > 0.0f ? 1.0f : -1.0f;
        }
    }
    ~Excavator()
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
        {
            const f32 error = targets[i] - hinges[i]->currentAngle();
            cylinders[i]->setValve(KM::clamp(8.0f * error * valveSign[i], -1.0f, 1.0f));
        }
    }
};

// Hinge separation under load measures convergence: default 8 velocity iterations vs a stiffer solver.
void excavatorFollowsCommands(u32 velocityIterations, f32 separationLimit)
{
    Excavator e;
    ContactSolverSettings settings;
    settings.velocityIterations = velocityIterations;
    e.rig.world.setSolverSettings(settings);
    const f32 poses[3][3] = {{0.0f, 0.0f, 0.0f}, {0.25f, -0.3f, 0.5f}, {-0.15f, 0.25f, -0.4f}};
    f32 worstSeparation = 0.0f;
    bool finite = true;
    bool movedTheRightWay = true;
    for (u32 pose = 0; pose < 3; ++pose)
    {
        f32 start[3];
        for (u32 i = 0; i < 3; ++i)
            start[i] = e.hinges[i]->currentAngle();
        for (int step = 0; step < 6 * 60; ++step)
        {
            e.steer(poses[pose]);
            e.rig.world.step(e.rig.step);
            for (u32 i = 0; i < 3; ++i)
            {
                worstSeparation = fmaxf(worstSeparation, e.separation(i));
                finite = finite && finiteVec(e.links[i]->position()) &&
                         finiteVec(e.links[i]->angularVelocity());
            }
            if (step == 30)
                for (u32 i = 0; i < 3; ++i)
                {
                    const f32 wanted = poses[pose][i] - start[i];
                    const f32 moved = e.hinges[i]->currentAngle() - start[i];
                    if (fabsf(wanted) > 0.05f && moved * wanted <= 0.0f)
                        movedTheRightWay = false;
                }
        }
        printf("pose %u: boom %.4f (%.2f), stick %.4f (%.2f), bucket %.4f (%.2f); strokes %.3f %.3f %.3f; "
               "forces %.0f %.0f %.0f N\n",
               pose, double(e.hinges[0]->currentAngle()), double(poses[pose][0]),
               double(e.hinges[1]->currentAngle()), double(poses[pose][1]),
               double(e.hinges[2]->currentAngle()), double(poses[pose][2]),
               double(e.cylinders[0]->stroke()), double(e.cylinders[1]->stroke()),
               double(e.cylinders[2]->stroke()), double(e.cylinders[0]->force()),
               double(e.cylinders[1]->force()), double(e.cylinders[2]->force()));
        for (u32 i = 0; i < 3; ++i)
            CHECK_NEAR(e.hinges[i]->currentAngle(), poses[pose][i], 0.01f);
    }
    for (u32 i = 0; i < 3; ++i)
        e.cylinders[i]->setValve(0.0f);
    f32 angles[3];
    for (u32 i = 0; i < 3; ++i)
        angles[i] = e.hinges[i]->currentAngle();
    e.rig.run(5.0f);
    f32 drift = 0.0f;
    for (u32 i = 0; i < 3; ++i)
        drift = fmaxf(drift, fabsf(e.hinges[i]->currentAngle() - angles[i]));
    f32 holding = 0.0f;
    for (u32 i = 0; i < 3; ++i)
        holding = fmaxf(holding, e.separation(i));
    printf("excavator, %u iterations: worst hinge separation %.2e m (%.2e holding), drift with "
           "valves closed %.2e rad\n",
           velocityIterations, double(worstSeparation), double(holding), double(drift));
    CHECK(finite);
    CHECK(movedTheRightWay);
    CHECK(worstSeparation < separationLimit);
    CHECK(holding < 0.5f * separationLimit);
    CHECK(drift < 1.0e-3f);
}

}

int main()
{
    speedIsFlowOverArea();
    stallsAtReliefPressure();
    holdsTheLoadWithoutDrift();
    cushionsAndStops();
    excavatorFollowsCommands(8, 1.0e-2f);
    excavatorFollowsCommands(16, 4.0e-3f);
    return TEST_RESULT();
}
