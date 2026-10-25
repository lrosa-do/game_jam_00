#include "check.h"
#include "ground_rig.h"

using namespace ground_test;

namespace
{
void minute(bool bumpy, f32 rate)
{
    Scene scene(!bumpy);
    if (bumpy)
        scene.addStatic(makeHeightfield(200, 1.0f, bumpyHeight), KM::vec3(0.0f), KM::quat());
    Rig tractor;
    Rig trailer;
    tractor.layout = buildFarmTractor(tractor.vehicle, tractor.body);
    trailer.layout = buildFarmTrailer(trailer.vehicle, trailer.body);
    SurfaceInfo soil;
    soil.type = SurfaceType::Soil;
    tractor.vehicle.setDefaultSurface(soil);
    trailer.vehicle.setDefaultSurface(soil);
    tractor.place(scene, KM::vec3(0.0f, bumpy ? bumpyHeight(0.0f, 0.0f) : 0.0f, 0.0f));
    CouplingParameters cp;
    cp.type = CouplingType::Ball;
    cp.towPoint = tractor.layout.towPoint;
    cp.trailerPoint = trailer.layout.couplingPoint;
    TrailerCoupling::placeTrailer(tractor.body, trailer.body, cp);
    trailer.addToWorld(scene);
    TrailerCoupling coupling;
    CHECK(coupling.attach(scene.world, tractor.vehicle, trailer.vehicle, cp));
    Rig* rigs[2] = {&tractor, &trailer};
    StabilityRun r;
    stabilityDrive(scene, rigs, 2, &coupling, rate, 3.5f, 0.5f, 6, r);
    reportStability("tractor and trailer", rate, bumpy, r);
    CHECK(r.finite);
    CHECK(r.lowestUp > 0.9f);
    CHECK(r.fastest < 6.0f);
    CHECK(r.fastestWheel < 10.0f);
    CHECK(r.farthest < 60.0f);
    CHECK(r.worstSeparation < 0.01f);
    CHECK(r.stoppedSpeed < 0.05f);
}
}

int main()
{
    minute(false, 60.0f);
    minute(false, 120.0f);
    minute(true, 60.0f);
    minute(true, 120.0f);
    return TEST_RESULT();
}
