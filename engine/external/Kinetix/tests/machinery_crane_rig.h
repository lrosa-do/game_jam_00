#ifndef KINETIX_TESTS_MACHINERY_CRANE_RIG_H
#define KINETIX_TESTS_MACHINERY_CRANE_RIG_H

#include "machinery_rig.h"

#include <machinery/MobileCrane.h>

namespace machinery_test
{

inline void run(Scene& scene, MobileCrane& crane, f32 seconds)
{
    const int steps = static_cast<int>(seconds / scene.step + 0.5f);
    for (int i = 0; i < steps; ++i)
    {
        crane.update(scene.step);
        scene.world.step(scene.step);
    }
}

inline f32 jackSum(const MobileCrane& crane)
{
    f32 sum = 0.0f;
    for (u32 i = 0; i < 4; ++i)
        sum += crane.outriggerLoad(i);
    return sum;
}

inline f32 craneMass(const MobileCraneParameters& p)
{
    f32 mass = p.chassisMass + 4.0f * (p.beamMass + p.jackMass) + p.superMass + p.baseMass + p.hookBlockMass;
    for (u32 i = 0; i < MobileCraneParameters::SectionCount; ++i)
        mass += p.sectionMass[i];
    return mass;
}

}

#endif // KINETIX_TESTS_MACHINERY_CRANE_RIG_H
