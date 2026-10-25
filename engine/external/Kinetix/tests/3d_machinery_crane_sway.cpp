#include "check.h"
#include "machinery_crane_rig.h"

using namespace machinery_test;

namespace
{

f32 slewAndStop(bool shaped, f32* period)
{
    Scene scene;
    MobileCraneParameters p = MobileCraneParameters::allTerrain50t();
    p.buildDeployed = true;
    p.buildElevation = 1.0f;
    p.hookDrop = 4.0f;
    MobileCrane crane(scene.world, p, KM::vec3(0.0f, p.chassisHeight, 0.0f));
    crane.setAntiSway(shaped);
    run(scene, crane, 1.0f);
    MobileCraneControls c = crane.controls();
    c.slew = 0.6f;
    crane.setControls(c);
    const f32 start = crane.slewJoint().currentAngle();
    while (crane.slewJoint().currentAngle() - start < 20.0f * kDeg)
        run(scene, crane, scene.step);
    c.slew = 0.0f;
    crane.setControls(c);
    run(scene, crane, 3.0f);
    f32 swing = 0.0f;
    for (int i = 0; i < 270; ++i)
    {
        crane.update(scene.step);
        scene.world.step(scene.step);
        swing = fmaxf(swing, crane.swingAngle());
    }
    *period = InputShaper::pendulumPeriod(crane.ropeLength());
    CHECK(crane.finite());
    return swing;
}

void antiSway()
{
    f32 period = 0.0f;
    const f32 plain = slewAndStop(false, &period);
    const f32 shaped = slewAndStop(true, &period);
    printf("slewing 20 deg and stopping, rope period %.2f s: residual swing %.4f rad plain, %.4f rad shaped (%.0f%%)\n",
           double(period), double(plain), double(shaped), double(100.0f * shaped / plain));
    CHECK(plain > 0.01f);
    CHECK(shaped < 0.4f * plain);
}

}

int main()
{
    antiSway();
    return TEST_RESULT();
}
