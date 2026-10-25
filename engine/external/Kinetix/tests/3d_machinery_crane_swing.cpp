#include "check.h"
#include "machinery_crane_rig.h"

using namespace machinery_test;

namespace
{

// Compound pendulum period about the sheave, against 2 pi sqrt(L/g) with L to the centre of mass.
void pendulumPeriod()
{
    Scene scene;
    MobileCraneParameters p = MobileCraneParameters::allTerrain50t();
    p.buildDeployed = true;
    p.buildElevation = 1.1f;
    p.hookDrop = 4.5f;
    MobileCrane crane(scene.world, p, KM::vec3(0.0f, p.chassisHeight, 0.0f));
    run(scene, crane, 1.0f);
    const KM::vec3 hook = crane.hook().pointWorld();
    const KM::vec3 loadHalf(0.25f);
    u32 loadId = 0;
    RigidBody& load = scene.box(hook - KM::vec3(0.0f, loadHalf.y, 0.0f), loadHalf, 1000.0f, false, &loadId);
    load.setSleepEpsilon(1.0e-4f);
    CHECK(crane.hook().attach(loadId, HookMode::Fixed));
    run(scene, crane, 1.0f);
    // The pivot and the assembly's centre of mass and inertia about it.
    const KM::vec3 pivot = crane.headSheaveWorld();
    const RigidBody& block = crane.hookBlock();
    const f32 mass = block.mass() + load.mass();
    const KM::vec3 centre = (block.position() * block.mass() + load.position() * load.mass()) / mass;
    const f32 length = KM::length(centre - pivot);
    auto about = [&](const RigidBody& body, const KM::vec3& half)
    {
        const f32 m = body.mass();
        const KM::vec3 r = body.position() - pivot;
        // Swinging about the boom's lateral axis: the box's own inertia
        // about it (half-extents across y and z) and the parallel axis.
        return m * (half.y * half.y + half.z * half.z) / 3.0f + m * (r.y * r.y + r.z * r.z);
    };
    const f32 inertia = about(block, crane.halfExtents(MobileCrane::HookBlock)) + about(load, loadHalf);
    const f32 compound = 2.0f * kPi * sqrtf(inertia / (mass * kG * length));
    const f32 simple = 2.0f * kPi * sqrtf(length / kG);
    const KM::vec3 push(0.0f, 0.0f, 0.15f);
    crane.hookBlock().setAwake(true);
    load.setAwake(true);
    crane.hookBlock().setVelocity(crane.hookBlock().velocity() + push);
    load.setVelocity(load.velocity() + push);
    f32 previous = 0.0f;
    f32 first = -1.0f;
    f32 last = -1.0f;
    u32 crossings = 0;
    f32 amplitude = 0.0f;
    const int steps = static_cast<int>(14.5f / scene.step);
    for (int i = 0; i < steps; ++i)
    {
        crane.update(scene.step);
        scene.world.step(scene.step);
        const f32 x = ((block.position() * block.mass() + load.position() * load.mass()) / mass - centre).z;
        amplitude = fmaxf(amplitude, fabsf(x));
        if (previous < 0.0f && x >= 0.0f && i > 0)
        {
            const f32 t = (static_cast<f32>(i) - x / (x - previous)) * scene.step;
            if (first < 0.0f)
                first = t;
            last = t;
            ++crossings;
        }
        previous = x;
    }
    const f32 period = crossings > 1 ? (last - first) / static_cast<f32>(crossings - 1) : 0.0f;
    printf("pendulum: L = %.3f m, period %.4f s over %u swings (compound %.4f, 2 pi sqrt(L/g) %.4f, %+.2f%%), "
           "amplitude %.3f m\n",
           double(length), double(period), crossings - 1, double(compound), double(simple),
           double(100.0f * (period / simple - 1.0f)), double(amplitude));
    CHECK(crossings >= 3);
    CHECK_NEAR(period, compound, 0.02f * compound);
    CHECK_NEAR(period, simple, 0.02f * simple);
    CHECK(crane.finite());
}

}

int main()
{
    pendulumPeriod();
    return TEST_RESULT();
}
