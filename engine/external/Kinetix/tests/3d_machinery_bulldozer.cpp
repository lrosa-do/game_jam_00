#include "check.h"
#include "machinery_rig.h"

#include <machinery/Bulldozer.h>

using namespace machinery_test;

namespace
{

constexpr u32 kDiggable = 1u << 7;

struct Hand
{
    f32 edge = 0.05f;
    f32 tilt = 0.0f;
    f32 travel = 0.0f;
};

void drive(Scene& scene, Bulldozer& d, const Hand& hand, f32 seconds, KM::vec3* worstForce = nullptr)
{
    const int steps = static_cast<int>(seconds / scene.step + 0.5f);
    for (int i = 0; i < steps; ++i)
    {
        BulldozerControls c;
        c.lift = KM::clamp(8.0f * (hand.edge - d.edgeHeight()), -1.0f, 1.0f);
        c.tilt = KM::clamp(10.0f * (hand.tilt - d.bladeTilt()), -1.0f, 1.0f);
        c.leftTrack = hand.travel;
        c.rightTrack = hand.travel;
        d.setControls(c);
        d.update(scene.step);
        scene.world.step(scene.step);
        if (worstForce)
        {
            const KM::vec3 f = d.bladeForce();
            if (KM::length(f) > KM::length(*worstForce))
                *worstForce = f;
        }
    }
}

void run(Scene& scene, Bulldozer& d, const BulldozerControls& c, f32 seconds)
{
    d.setControls(c);
    const int steps = static_cast<int>(seconds / scene.step + 0.5f);
    for (int i = 0; i < steps; ++i)
    {
        d.update(scene.step);
        scene.world.step(scene.step);
    }
}

f32 machineWeight(const BulldozerParameters& p)
{
    return (p.tractorMass + p.pushFrameMass + p.bladeMass) * kG;
}

// Lift cylinders move at the section's flow over their annuli raising and caps lowering.
void bladeLiftAndTilt()
{
    Scene scene;
    const BulldozerParameters p = BulldozerParameters::crawler18t();
    Bulldozer d(scene.world, p, KM::vec3(0.0f, p.rideHeight, 0.0f));
    BulldozerControls c;
    run(scene, d, c, 0.5f);
    const f32 rest = d.edgeHeight();
    c.lift = 1.0f;
    run(scene, d, c, 0.5f);
    const f32 s0 = d.liftCylinder().stroke();
    run(scene, d, c, 0.5f);
    const f32 raiseSpeed = (d.liftCylinder().stroke() - s0) / 0.5f;
    run(scene, d, c, 2.0f);
    const f32 raised = d.edgeHeight();
    c.lift = 0.0f;
    run(scene, d, c, 0.3f);
    const f32 held = d.edgeHeight();
    run(scene, d, c, 1.5f);
    const f32 drift = d.edgeHeight() - held;
    c.lift = -1.0f;
    run(scene, d, c, 0.5f);
    const f32 s1 = d.liftCylinder().stroke();
    run(scene, d, c, 0.5f);
    const f32 lowerSpeed = (d.liftCylinder().stroke() - s1) / 0.5f;
    const HydraulicCylinderParameters& lift = p.liftCylinder;
    const f32 cap = 0.25f * kPi * lift.boreDiameter * lift.boreDiameter * static_cast<f32>(lift.count);
    const f32 annulus =
        cap - 0.25f * kPi * lift.rodDiameter * lift.rodDiameter * static_cast<f32>(lift.count);
    const f32 flow = p.liftSection.ratedFlow / 60000.0f;
    Hand hand;
    hand.edge = 0.5f;
    drive(scene, d, hand, 2.0f);
    c = BulldozerControls();
    c.tilt = 1.0f;
    run(scene, d, c, 2.5f);
    const f32 tiltUp = d.bladeTilt();
    c.tilt = -1.0f;
    run(scene, d, c, 4.0f);
    const f32 tiltDown = d.bladeTilt();
    printf("blade: rest edge %.3f m, raised to %.3f m (retracting %.4f m/s, Q/A %.4f), held drift %.2e m, lowering "
           "%.4f m/s (Q/A %.4f); tilt %.3f..%.3f rad (tips %+.2f / %+.2f m); hinges %.2e m\n",
           double(rest), double(raised), double(raiseSpeed), double(-flow / annulus), double(drift),
           double(lowerSpeed), double(flow / cap), double(tiltDown), double(tiltUp),
           double(0.5f * p.bladeWidth * sinf(tiltDown)), double(0.5f * p.bladeWidth * sinf(tiltUp)),
           double(d.worstJointSeparation()));
    CHECK_NEAR(rest, p.buildEdgeHeight, 0.01f);
    CHECK(raised > 1.0f);
    CHECK_NEAR(raiseSpeed, -flow / annulus, 0.03f * flow / annulus);
    CHECK_NEAR(lowerSpeed, flow / cap, 0.03f * flow / cap);
    CHECK(fabsf(drift) < 2.0e-3f);
    CHECK(tiltUp > 0.15f);
    CHECK(tiltDown < -0.15f);
    CHECK(d.worstJointSeparation() < 1.0e-4f);
    CHECK(d.finite());
}

// Blade force is bounded by what the tracks can push (mu W) plus the pile's weight on the blade face.
void pushesPile()
{
    Scene scene;
    const BulldozerParameters p = BulldozerParameters::crawler18t();
    Bulldozer d(scene.world, p, KM::vec3(0.0f, p.rideHeight, 0.0f));
    ct::Vector<RigidBody*> pile;
    f32 pileMass = 0.0f;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 2; ++k)
            {
                pile.push_back(&scene.box(KM::vec3(-0.6f + 0.6f * static_cast<f32>(i), 0.25f + 0.5f * static_cast<f32>(k),
                                                   6.0f + 0.6f * static_cast<f32>(j)),
                                          KM::vec3(0.25f), 250.0f, true));
                pileMass += 250.0f;
            }
    auto pileCentre = [&]()
    {
        KM::vec3 sum(0.0f);
        for (RigidBody* b : pile)
            sum += b->position();
        return sum / static_cast<f32>(pile.size());
    };
    const KM::vec3 start = pileCentre();
    Hand hand;
    hand.edge = 0.06f;
    drive(scene, d, hand, 0.5f);
    hand.travel = 1.0f;
    drive(scene, d, hand, 2.0f);
    const f32 freeSpeed = d.tractor().velocity().z;
    KM::vec3 worst(0.0f);
    f32 worstLift = 0.0f;
    f32 worstTilt = 0.0f;
    f32 slowest = 1.0e9f;
    for (int i = 0; i < 6 * 60; ++i)
    {
        drive(scene, d, hand, scene.step, &worst);
        worstLift = fmaxf(worstLift, fabsf(d.liftCylinder().force()));
        worstTilt = fmaxf(worstTilt, fabsf(d.tiltCylinder().force()));
        slowest = fminf(slowest, d.tractor().velocity().z);
    }
    const KM::vec3 moved = pileCentre() - start;
    const f32 grip = SoilParameters::hardSurface().friction * machineWeight(p);
    const HydraulicCylinderParameters& lift = p.liftCylinder;
    const f32 liftLimit = lift.reliefPressure * 0.25f * kPi * lift.boreDiameter * lift.boreDiameter *
                          static_cast<f32>(lift.count);
    const HydraulicCylinderParameters& tilt = p.tiltCylinder;
    const f32 tiltLimit = tilt.reliefPressure * 0.25f * kPi * tilt.boreDiameter * tilt.boreDiameter;
    printf("pushing %.0f kg of boxes: pile moved %.2f m (tractor at %.2f m/s free, slowest %.2f pushing); worst blade "
           "force %.0f N (%.0f along, %.0f up), grip mu W %.0f N; lift cylinders <= %.0f N (relief %.0f), tilt <= %.0f "
           "N (relief %.0f); hinges %.2e m\n",
           double(pileMass), double(moved.z), double(freeSpeed), double(slowest), double(KM::length(worst)),
           double(worst.z), double(worst.y), double(grip), double(worstLift), double(liftLimit), double(worstTilt),
           double(tiltLimit), double(d.worstJointSeparation()));
    CHECK(moved.z > 2.5f);
    CHECK(slowest < freeSpeed - 0.05f);
    CHECK(fabsf(worst.z) < grip + pileMass * kG);
    CHECK(KM::length(worst) < grip + 2.0f * pileMass * kG);
    CHECK(worstLift < 1.01f * liftLimit);
    CHECK(worstTilt < 1.01f * tiltLimit);
    CHECK(d.worstJointSeparation() < 1.0e-3f);
    CHECK(d.finite());
}

// Edge held 0.1 m below grade: it pushes with the soil's cutting resistance (at most its bound).
void cuttingResistance()
{
    Scene scene(60.0f, false);
    RigidBody* floor = new RigidBody();
    floor->setBodyType(BodyType::Static);
    floor->setPosition(KM::vec3(0.0f, -1.0f, 0.0f));
    floor->calculateDerivedData();
    BoxShape* floorShape = new BoxShape(KM::vec3(200.0f, 1.0f, 200.0f));
    BodyEntry entry;
    entry.body = floor;
    entry.shape = floorShape;
    entry.filter.group = kDiggable;
    scene.world.addBody(entry);
    scene.world.markStaticBroadphaseDirty();
    scene.bodies.push_back(floor);
    scene.shapes.push_back(floorShape);

    const BulldozerParameters p = BulldozerParameters::crawler18t();
    Bulldozer d(scene.world, p, KM::vec3(0.0f, p.rideHeight, 0.0f));
    const SoilParameters soil = SoilParameters::heavyClay();
    d.setGroundSoil(SoilParameters::loam());
    SoilBed bed;
    bed.surfaceHeight = 0.0f;
    bed.soil = soil;
    d.setSoilBed(bed);
    Hand hand;
    hand.edge = 0.1f;
    hand.travel = 1.0f;
    drive(scene, d, hand, 2.5f);
    const f32 freeSpeed = d.tractor().velocity().z;
    const f32 freeSlip = d.tracks().side(0).slip;
    hand.edge = -0.1f;
    drive(scene, d, hand, 2.5f);
    f32 force = 0.0f;
    f32 bound = 0.0f;
    f32 depth = 0.0f;
    f32 speed = 0.0f;
    f32 slip = 0.0f;
    const int samples = 60;
    for (int i = 0; i < samples; ++i)
    {
        drive(scene, d, hand, scene.step);
        force += KM::length(d.cuttingEdge().force()) / samples;
        bound += d.cuttingEdge().resistance() / samples;
        depth += d.cuttingEdge().depth() / samples;
        speed += d.tractor().velocity().z / samples;
        slip += d.tracks().side(0).slip / samples;
    }
    const f32 model = SoilCuttingTool::cuttingResistance(soil, p.bladeWidth, depth, speed, p.cuttingAngle);
    printf("dozing heavy clay at %.3f m: edge force %.0f N, bound %.0f N (the model %.0f N at %.2f m/s); tractor %.2f m/s "
           "(free %.2f), slip %.3f (free %.3f)\n",
           double(depth), double(force), double(bound), double(model), double(speed), double(speed), double(freeSpeed),
           double(slip), double(freeSlip));
    CHECK(depth > 0.07f);
    CHECK(force > 0.9f * bound);
    CHECK(force <= 1.01f * bound);
    CHECK_NEAR(bound, model, 0.05f * model);
    CHECK(speed < freeSpeed);
    CHECK(slip > freeSlip);
    CHECK(d.finite());
}

}

int main()
{
    bladeLiftAndTilt();
    pushesPile();
    cuttingResistance();
    return TEST_RESULT();
}
