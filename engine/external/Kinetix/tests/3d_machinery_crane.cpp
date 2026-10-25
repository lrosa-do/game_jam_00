#include "check.h"
#include "machinery_crane_rig.h"

using namespace machinery_test;

namespace
{

void outriggersLiftTheCarrier()
{
    Scene scene;
    MobileCraneParameters p = MobileCraneParameters::allTerrain50t();
    // Quicker than the rated speeds, to keep the test short.
    p.beamSpeed = 1.0f;
    p.jackSpeed = 0.4f;
    MobileCrane crane(scene.world, p, KM::vec3(0.0f, p.chassisHeight, 0.0f));
    run(scene, crane, 1.0f);
    const f32 standing = crane.carrierGroundLoad();
    MobileCraneControls c;
    c.outriggerBeams = 1.0f;
    crane.setControls(c);
    run(scene, crane, 3.0f);
    const f32 beams = crane.outriggerBeam(0).currentPosition();
    c.outriggerJacks = (p.padClearance + 0.2f) / p.jackStroke;
    crane.setControls(c);
    run(scene, crane, 3.0f);
    const f32 weight = (craneMass(p) - 4.0f * p.jackMass) * kG;
    printf("outriggers: standing %.0f N (crane %.0f N less the jacks), beams out %.3f m, jacks down: %.0f / %.0f / "
           "%.0f / %.0f N, sum %.0f, carrier box %.0f N, carrier up %.3f m, tilt %.4f deg\n",
           double(standing), double(weight), double(beams), double(crane.outriggerLoad(0)),
           double(crane.outriggerLoad(1)), double(crane.outriggerLoad(2)), double(crane.outriggerLoad(3)),
           double(jackSum(crane)), double(crane.carrierGroundLoad()), double(crane.chassis().position().y - p.chassisHeight),
           double(crane.tilt() / kDeg));
    CHECK_NEAR(beams, p.outriggerSpread - p.outriggerRetracted, 1.0e-3f);
    CHECK_NEAR(jackSum(crane), weight, 0.01f * weight);
    CHECK(crane.carrierGroundLoad() < 1.0f);
    CHECK(crane.chassis().position().y - p.chassisHeight > 0.15f);
    CHECK(crane.tilt() < 0.2f * kDeg);
    for (u32 i = 0; i < 4; ++i)
        CHECK(crane.outriggerLoad(i) > 0.1f * weight);
    CHECK(crane.finite());
}

struct Lift
{
    f32 tilt = 0.0f;
    f32 utilisation = 0.0f;
    f32 nearSide = 0.0f;
    f32 farSide = 0.0f;
    f32 sum = 0.0f;
    f32 weight = 0.0f;
    f32 rated = 0.0f;
    f32 radius = 0.0f;
    f32 length = 0.0f;
};

void ratedLiftOnOutriggers(MobileCraneParameters p, f32 share, bool lmi, Lift& out)
{
    Scene scene;
    MobileCrane crane(scene.world, p, KM::vec3(0.0f, p.chassisHeight, 0.0f));
    crane.lmi().setEnabled(lmi);
    run(scene, crane, 1.0f);
    const f32 rated = crane.lmi().ratedLoad();
    out.rated = rated;
    out.radius = crane.radius();
    out.length = crane.boomLength();
    const KM::vec3 hook = crane.hook().pointWorld();
    u32 loadId = 0;
    const f32 mass = share * rated - p.hookBlockMass;
    scene.box(hook - KM::vec3(0.0f, 0.6f, 0.0f), KM::vec3(0.6f), mass, false, &loadId);
    crane.hook().attach(loadId, HookMode::Point);
    // Picked up at once, the load bounces on the rope: readings are the last 1.5 s averages.
    f32 worstTilt = 0.0f;
    const int steps = 6 * 60;
    const int averaged = 90;
    for (int i = 0; i < steps; ++i)
    {
        crane.update(scene.step);
        scene.world.step(scene.step);
        worstTilt = fmaxf(worstTilt, crane.tilt());
        if (i < steps - averaged)
            continue;
        out.utilisation += crane.lmi().utilisation() / averaged;
        // Slewed to the left (+X): jacks 0 and 2 are on the load's side.
        out.nearSide += (crane.outriggerLoad(0) + crane.outriggerLoad(2)) / averaged;
        out.farSide += (crane.outriggerLoad(1) + crane.outriggerLoad(3)) / averaged;
        out.sum += jackSum(crane) / averaged;
    }
    out.tilt = worstTilt;
    out.weight = (craneMass(p) - 4.0f * p.jackMass + mass) * kG;
    CHECK(crane.finite());
}

void ratedLiftStaysStable()
{
    MobileCraneParameters p = MobileCraneParameters::allTerrain50t();
    p.buildDeployed = true;
    p.buildSlew = 0.5f * kPi;
    p.buildElevation = 0.9f;
    p.buildExtension = 2.0f;
    Lift lift;
    ratedLiftOnOutriggers(p, 1.0f, true, lift);
    printf("rated load (%.0f kg at %.2f m, %.2f m boom) over the side: utilisation %.3f, tilt <= %.3f deg, jacks %.0f "
           "N (crane + load %.0f N), load side %.0f N, far side %.0f N\n",
           double(lift.rated), double(lift.radius), double(lift.length), double(lift.utilisation),
           double(lift.tilt / kDeg), double(lift.sum), double(lift.weight), double(lift.nearSide), double(lift.farSide));
    CHECK_NEAR(lift.utilisation, 1.0f, 0.02f);
    CHECK(lift.tilt < 0.5f * kDeg);
    CHECK_NEAR(lift.sum, lift.weight, 0.01f * lift.weight);
    CHECK(lift.nearSide > 2.0f * lift.farSide);
    CHECK(lift.farSide > 0.0f);
}

// The chart is 75% of tipping: a load 30% beyond the tipping load (1.73 x rated) tips the crane.
void overloadTips()
{
    MobileCraneParameters p = MobileCraneParameters::allTerrain50t();
    p.buildDeployed = true;
    p.buildSlew = 0.5f * kPi;
    p.buildElevation = 0.9f;
    p.buildExtension = 2.0f;
    const f32 share = 1.3f / p.stabilityFactor;
    Lift lift;
    ratedLiftOnOutriggers(p, share, false, lift);
    LoadMomentIndicator lmi;
    lmi.setChart(MobileCrane::computeChart(p));
    lmi.update(share * lift.rated, lift.radius, lift.length, 0.0f);
    printf("overload x%.2f of rated (%.0f kg at %.2f m), limiter off: tilt %.1f deg, far-side jacks %.0f N; the "
           "limiter reads %.2f and cuts out: %d\n",
           double(share), double(share * lift.rated), double(lift.radius), double(lift.tilt / kDeg),
           double(lift.farSide), double(lmi.utilisation()), int(lmi.cutOut()));
    CHECK(lift.tilt > 5.0f * kDeg);
    // Only the far jacks' own weight: their pads are off the ground.
    CHECK(lift.farSide < 1.1f * 2.0f * p.jackMass * kG);
    CHECK(lmi.cutOut());
}

}

int main()
{
    outriggersLiftTheCarrier();
    ratedLiftStaysStable();
    overloadTips();
    return TEST_RESULT();
}
