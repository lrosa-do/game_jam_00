#include "Check.h"

#include "prisma3d/render/ReflectionProbeData.h"

#include <math.h>

using namespace prisma3d;

namespace
{

bool near(float a, float b, float tolerance = 1e-4f)
{
    return fabsf(a - b) <= tolerance;
}

void testOrderAndFields()
{
    ReflectionProbeParams small;
    small.size = Math::Vec3(2.0f, 2.0f, 2.0f);
    small.intensity = 0.5f;
    small.interior = true;
    small.boxProjection = true;
    small.blendDistance = 0.25f;
    small.reflectionMask = 0x5u;
    small.originOffset = Math::Vec3(0.0f, 1.0f, 0.0f);
    small.ambientMode = ReflectionProbeAmbientMode::Color;
    small.ambientColor = Math::Vec3(1.0f, 0.5f, 0.0f);
    small.ambientEnergy = 2.0f;

    ReflectionProbeParams large;
    large.size = Math::Vec3(20.0f, 8.0f, 20.0f);

    ReflectionInstanceInput inputs[3];
    inputs[0].probe = &small;
    inputs[0].transform = Math::Mat4::Translation(Math::Vec3(5.0f, 0.0f, 0.0f));
    inputs[0].atlasIndex = 3;
    inputs[1].probe = nullptr;
    inputs[2].probe = &large;
    inputs[2].transform = Math::Mat4::Translation(Math::Vec3(0.0f, 4.0f, 0.0f));
    inputs[2].atlasIndex = 1;

    ct::Vector<ReflectionDataGpu> data;
    ct::Vector<ReflectionSorted> sorted;
    const Math::Mat4 camera = Math::Mat4::Translation(Math::Vec3(0.0f, 0.0f, 10.0f));
    const std::uint32_t count = buildReflectionData(inputs, 3, camera, 1.0f, 8, data, sorted);
    CHECK(count == 2);
    CHECK(data.size() == 2);
    CHECK(sorted.size() == 2);
    CHECK(sorted[0].instance == 2);
    CHECK(sorted[1].instance == 0);

    CHECK(near(data[0].boxExtents[0], 10.0f));
    CHECK(near(data[0].boxExtents[1], 4.0f));
    CHECK(near(data[0].index, 1.0f));
    CHECK(data[0].exterior == 1);
    CHECK(data[0].boxProject == 0);
    CHECK(data[0].ambientMode == 1);

    CHECK(near(data[1].boxExtents[0], 1.0f));
    CHECK(near(data[1].index, 3.0f));
    CHECK(near(data[1].boxOffset[1], 1.0f));
    CHECK(data[1].mask == 0x5u);
    CHECK(near(data[1].intensity, 0.5f));
    CHECK(near(data[1].blendDistance, 0.25f));
    CHECK(data[1].exterior == 0);
    CHECK(data[1].boxProject == 1);
    CHECK(data[1].ambientMode == 2);
    CHECK(near(data[1].ambient[0], 2.0f));
    CHECK(near(data[1].ambient[1], 2.0f * 0.21404114f, 1e-3f));
    CHECK(near(data[1].ambient[2], 0.0f));
    CHECK(near(data[1].exposureNormalization, 1.0f));

    const float* local = data[1].localMatrix;
    CHECK(near(local[0], 1.0f));
    CHECK(near(local[5], 1.0f));
    CHECK(near(local[10], 1.0f));
    CHECK(near(local[12], -5.0f));
    CHECK(near(local[13], 0.0f));
    CHECK(near(local[14], 10.0f));
    CHECK(near(local[15], 1.0f));
}

void testCap()
{
    ReflectionProbeParams probe;
    ReflectionInstanceInput inputs[5];
    for (int i = 0; i < 5; ++i) inputs[i].probe = &probe;
    ct::Vector<ReflectionDataGpu> data;
    ct::Vector<ReflectionSorted> sorted;
    CHECK(buildReflectionData(inputs, 5, Math::Mat4::Identity(), 1.0f, 3, data, sorted) == 3);
    CHECK(buildReflectionData(nullptr, 0, Math::Mat4::Identity(), 1.0f, 3, data, sorted) == 0);
    CHECK(data.empty());
}

void testExposure()
{
    ReflectionProbeParams probe;
    probe.bakedExposure = 2.0f;
    ReflectionInstanceInput input;
    input.probe = &probe;
    ct::Vector<ReflectionDataGpu> data;
    ct::Vector<ReflectionSorted> sorted;
    buildReflectionData(&input, 1, Math::Mat4::Identity(), 4.0f, 8, data, sorted);
    CHECK(near(data[0].exposureNormalization, 2.0f));
}

} // namespace

int main()
{
    testOrderAndFields();
    testCap();
    testExposure();

    if (failures) printf("%d checks failed\n", failures);
    return failures ? 1 : 0;
}
