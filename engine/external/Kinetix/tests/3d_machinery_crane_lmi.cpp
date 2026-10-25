#include "check.h"
#include "machinery_crane_rig.h"

using namespace machinery_test;

namespace
{

void lmiCutsOutAtTheChart()
{
    Scene scene;
    MobileCraneParameters p = MobileCraneParameters::allTerrain50t();
    p.buildDeployed = true;
    p.buildSlew = 0.5f * kPi;
    p.buildElevation = 1.05f;
    p.buildExtension = 2.0f;
    MobileCrane crane(scene.world, p, KM::vec3(0.0f, p.chassisHeight, 0.0f));
    run(scene, crane, 1.0f);
    const f32 rated = crane.lmi().ratedLoad();
    const f32 startRadius = crane.radius();
    const KM::vec3 hook = crane.hook().pointWorld();
    u32 loadId = 0;
    const f32 mass = 0.8f * rated - p.hookBlockMass;
    scene.box(hook - KM::vec3(0.0f, 0.6f, 0.0f), KM::vec3(0.6f), mass, false, &loadId);
    crane.hook().attach(loadId, HookMode::Point);
    run(scene, crane, 1.5f);
    MobileCraneControls c = crane.controls();
    c.luff = -0.3f;
    crane.setControls(c);
    f32 cutAt = -1.0f;
    for (int i = 0; i < 30 * 60 && cutAt < 0.0f; ++i)
    {
        crane.update(scene.step);
        scene.world.step(scene.step);
        if (crane.lmi().cutOut())
            cutAt = i * scene.step;
    }
    run(scene, crane, 1.5f);
    const f32 radius = crane.radius();
    const f32 rope = crane.ropeLength();
    const f32 chartAtRadius = crane.lmi().chart().rated(crane.boomLength(), radius);
    const f32 luffSpeed = crane.luffCylinder().strokeVelocity();
    c.hoist = 1.0f;
    crane.setControls(c);
    run(scene, crane, 1.0f);
    const f32 hoisted = rope - crane.ropeLength();
    c.hoist = 0.0f;
    c.luff = 0.3f;
    crane.setControls(c);
    run(scene, crane, 2.0f);
    printf("limiter: %.0f kg (80%% at %.2f m), booming down cut out after %.1f s at %.2f m where the chart rates %.0f "
           "kg (utilisation %.3f), luffing at %.4f m/s; hoist up moved the hook %.4f m; boomed up to %.2f m, "
           "utilisation %.3f\n",
           double(mass + p.hookBlockMass), double(startRadius), double(cutAt), double(radius), double(chartAtRadius),
           double((mass + p.hookBlockMass) / chartAtRadius), double(luffSpeed), double(hoisted), double(crane.radius()),
           double(crane.lmi().utilisation()));
    CHECK(cutAt > 0.0f);
    CHECK(radius > startRadius + 0.5f);
    CHECK_NEAR((mass + p.hookBlockMass) / chartAtRadius, 1.0f, 0.04f);
    CHECK(fabsf(luffSpeed) < 1.0e-3f);
    // Hoisting would have raised it 0.3 m; what is left is the load's bounce on the rope.
    CHECK(fabsf(hoisted) < 0.03f);
    CHECK(crane.radius() < radius - 0.2f);
    CHECK(crane.finite());
}

}

int main()
{
    lmiCutsOutAtTheChart();
    return TEST_RESULT();
}
