#include "Check.h"

#include "prisma3d/render/DecalData.h"

#include <math.h>

using namespace prisma3d;

namespace
{

bool near(float a, float b, float tolerance = 1e-4f)
{
    return fabsf(a - b) <= tolerance;
}

Math::Vec3 apply(const DecalDataGpu& dd, float x, float y, float z, float w = 1.0f)
{
    return Math::Vec3(dd.xform[0] * x + dd.xform[4] * y + dd.xform[8] * z + dd.xform[12] * w,
            dd.xform[1] * x + dd.xform[5] * y + dd.xform[9] * z + dd.xform[13] * w,
            dd.xform[2] * x + dd.xform[6] * y + dd.xform[10] * z + dd.xform[14] * w);
}

bool nearVec3(const Math::Vec3& v, float x, float y, float z)
{
    return near(v.x, x) && near(v.y, y) && near(v.z, z);
}

DecalInstanceInput makeInput(const DecalParams& decal, const Math::Vec3& position)
{
    DecalInstanceInput input;
    input.decal = &decal;
    input.transform = Math::Mat4::Translation(position);
    input.rects[static_cast<std::size_t>(DecalTexture::Albedo)] =
            Math::Vec4(0.25f, 0.5f, 0.125f, 0.0625f);
    return input;
}

void testDefaults()
{
    DecalParams decal;
    CHECK(nearVec3(decal.size, 2.0f, 2.0f, 2.0f));
    CHECK(near(decal.emissionEnergy, 1.0f) && near(decal.albedoMix, 1.0f));
    CHECK(decal.cullMask == 0xFFFFFu);
    CHECK(near(decal.upperFade, 0.3f) && near(decal.lowerFade, 0.3f));
    CHECK(!decal.distanceFade);
    CHECK(near(decal.distanceFadeBegin, 40.0f) && near(decal.distanceFadeLength, 10.0f));
    CHECK(near(decal.normalFade, 0.0f));
    CHECK(sizeof(DecalDataGpu) == 240);
}

void testTransform()
{
    ct::Vector<DecalDataGpu> data;
    ct::Vector<DecalSorted> sorted;
    {
        DecalParams decal;
        decal.modulate = Math::Vec4(0.5f, 1.0f, 0.0f, 0.8f);
        decal.emissionEnergy = 3.0f;
        decal.albedoMix = 0.7f;
        decal.cullMask = 5;
        decal.upperFade = 0.1f;
        decal.lowerFade = 0.2f;
        decal.normalFade = 0.4f;
        const DecalInstanceInput input = makeInput(decal, Math::Vec3(0.0f, 0.0f, -5.0f));
        CHECK(buildDecalData(&input, 1, Math::Mat4::Identity(), 16, data, sorted) == 1);
        CHECK(data.size() == 1 && sorted.size() == 1);
        const DecalDataGpu& dd = data[0];
        CHECK(nearVec3(apply(dd, 0.0f, 0.0f, -5.0f), 0.5f, 0.0f, 0.5f));
        CHECK(nearVec3(apply(dd, 1.0f, 1.0f, -4.0f), 1.0f, 1.0f, 1.0f));
        CHECK(nearVec3(apply(dd, -1.0f, -1.0f, -6.0f), 0.0f, -1.0f, 0.0f));
        CHECK(nearVec3(apply(dd, 1.0f, 0.0f, 0.0f, 0.0f), 0.5f, 0.0f, 0.0f));
        CHECK(near(dd.xform[3], 0.0f) && near(dd.xform[7], 0.0f) && near(dd.xform[11], 0.0f) &&
                near(dd.xform[15], 1.0f));
        CHECK(near(dd.normal[0], 0.0f) && near(dd.normal[1], 1.0f) && near(dd.normal[2], 0.0f));
        CHECK(near(dd.normalFade, 0.4f));
        CHECK(near(dd.albedoRect[0], 0.25f) && near(dd.albedoRect[1], 0.5f) &&
                near(dd.albedoRect[2], 0.125f) && near(dd.albedoRect[3], 0.0625f));
        CHECK(near(dd.normalRect[2], 0.0f) && near(dd.ormRect[2], 0.0f) &&
                near(dd.emissionRect[2], 0.0f));
        CHECK(near(dd.modulate[0], 0.21404f) && near(dd.modulate[1], 1.0f) &&
                near(dd.modulate[2], 0.0f) && near(dd.modulate[3], 0.8f));
        CHECK(near(dd.emissionEnergy, 3.0f) && near(dd.albedoMix, 0.7f));
        CHECK(dd.mask == 5);
        CHECK(near(dd.upperFade, 0.1f) && near(dd.lowerFade, 0.2f));
        CHECK(sorted[0].instance == 0);
        CHECK(nearVec3(sorted[0].extents, 1.0f, 1.0f, 1.0f));
    }
    {
        DecalParams decal;
        decal.size = Math::Vec3(4.0f, 2.0f, 8.0f);
        const DecalInstanceInput input = makeInput(decal, Math::Vec3(10.0f, 0.0f, 0.0f));
        const Math::Mat4 camera = Math::Mat4::Translation(Math::Vec3(10.0f, 0.0f, 20.0f));
        buildDecalData(&input, 1, camera, 16, data, sorted);
        const DecalDataGpu& dd = data[0];
        CHECK(nearVec3(apply(dd, 0.0f, 0.0f, -20.0f), 0.5f, 0.0f, 0.5f));
        CHECK(nearVec3(apply(dd, 2.0f, 1.0f, -16.0f), 1.0f, 1.0f, 1.0f));
        CHECK(nearVec3(apply(dd, -2.0f, -1.0f, -24.0f), 0.0f, -1.0f, 0.0f));
        CHECK(nearVec3(sorted[0].extents, 2.0f, 1.0f, 4.0f));
    }
    {
        DecalParams decal;
        DecalInstanceInput input = makeInput(decal, Math::Vec3(0.0f, 0.0f, 0.0f));
        input.transform = Math::Mat4::RotationZ(1.57079632679f);
        input.rects[static_cast<std::size_t>(DecalTexture::Normal)] =
                Math::Vec4(0.5f, 0.5f, 0.25f, 0.25f);
        const Math::Mat4 camera = Math::Mat4::RotationY(1.57079632679f);
        buildDecalData(&input, 1, camera, 16, data, sorted);
        const DecalDataGpu& dd = data[0];
        CHECK(near(dd.normal[0], 0.0f) && near(dd.normal[1], 0.0f) && near(dd.normal[2], -1.0f));
        CHECK(near(dd.normalXform[0], 0.0f) && near(dd.normalXform[1], 1.0f) &&
                near(dd.normalXform[2], 0.0f) && near(dd.normalXform[3], 0.0f));
        CHECK(near(dd.normalXform[4], 0.0f) && near(dd.normalXform[5], 0.0f) &&
                near(dd.normalXform[6], -1.0f));
        CHECK(near(dd.normalXform[8], -1.0f) && near(dd.normalXform[9], 0.0f) &&
                near(dd.normalXform[10], 0.0f));
        CHECK(near(dd.normalRect[0], 0.5f) && near(dd.normalRect[3], 0.25f));
    }
}

void testSortAndFade()
{
    ct::Vector<DecalDataGpu> data;
    ct::Vector<DecalSorted> sorted;
    DecalParams plain;
    DecalParams fading;
    fading.distanceFade = true;
    fading.distanceFadeBegin = 4.0f;
    fading.distanceFadeLength = 2.0f;
    fading.modulate = Math::Vec4(1.0f, 1.0f, 1.0f, 0.8f);
    fading.emissionEnergy = 2.0f;
    {
        DecalInstanceInput inputs[3] = { makeInput(plain, Math::Vec3(0.0f, 0.0f, -5.0f)),
            makeInput(plain, Math::Vec3(0.0f, 0.0f, -3.0f)),
            makeInput(plain, Math::Vec3(0.0f, 0.0f, -9.0f)) };
        CHECK(buildDecalData(inputs, 3, Math::Mat4::Identity(), 16, data, sorted) == 3);
        CHECK(sorted[0].instance == 1 && sorted[1].instance == 0 && sorted[2].instance == 2);
        CHECK(nearVec3(apply(data[0], 0.0f, 0.0f, -3.0f), 0.5f, 0.0f, 0.5f));
        CHECK(nearVec3(apply(data[2], 0.0f, 0.0f, -9.0f), 0.5f, 0.0f, 0.5f));

        inputs[2].sortingOffset = 10.0f;
        buildDecalData(inputs, 3, Math::Mat4::Identity(), 16, data, sorted);
        CHECK(sorted[0].instance == 2 && sorted[1].instance == 1 && sorted[2].instance == 0);

        CHECK(buildDecalData(inputs, 3, Math::Mat4::Identity(), 2, data, sorted) == 2);
        CHECK(sorted[0].instance == 1 && sorted[1].instance == 0);
    }
    {
        DecalInstanceInput inputs[4] = { makeInput(fading, Math::Vec3(0.0f, 0.0f, -3.0f)),
            makeInput(fading, Math::Vec3(0.0f, 0.0f, -5.0f)),
            makeInput(fading, Math::Vec3(0.0f, 0.0f, -7.0f)),
            makeInput(fading, Math::Vec3(0.0f, 0.0f, -5.5f)) };
        CHECK(buildDecalData(inputs, 4, Math::Mat4::Identity(), 16, data, sorted) == 3);
        CHECK(sorted[0].instance == 0 && sorted[1].instance == 1 && sorted[2].instance == 3);
        CHECK(near(data[0].modulate[3], 0.8f) && near(data[0].emissionEnergy, 2.0f));
        CHECK(near(data[1].modulate[3], 0.4f) && near(data[1].emissionEnergy, 1.0f));
        CHECK(near(data[2].modulate[3], 0.8f * 0.15625f) &&
                near(data[2].emissionEnergy, 2.0f * 0.15625f));
    }
    {
        DecalInstanceInput inputs[3] = { makeInput(plain, Math::Vec3(0.0f, 0.0f, -2.0f)),
            makeInput(plain, Math::Vec3(0.0f, 0.0f, -4.0f)),
            makeInput(plain, Math::Vec3(0.0f, 0.0f, -6.0f)) };
        inputs[0].rects[0] = Math::Vec4(0.0f, 0.0f, 0.0f, 0.0f);
        inputs[1].rects[0] = Math::Vec4(0.0f, 0.0f, 0.0f, 0.0f);
        inputs[1].rects[static_cast<std::size_t>(DecalTexture::Emission)] =
                Math::Vec4(0.5f, 0.5f, 0.25f, 0.25f);
        inputs[2].decal = nullptr;
        CHECK(buildDecalData(inputs, 3, Math::Mat4::Identity(), 16, data, sorted) == 2);
        CHECK(data[0].mask == 0 && near(data[0].albedoRect[2], 0.0f) &&
                near(data[0].emissionRect[2], 0.0f) && near(data[0].xform[0], 0.0f));
        CHECK(near(data[1].emissionRect[0], 0.5f) && data[1].mask == 0xFFFFFu);
        CHECK(near(data[1].albedoRect[2], 0.0f));
        CHECK(nearVec3(apply(data[1], 0.0f, 0.0f, -4.0f), 0.5f, 0.0f, 0.5f));
    }
    CHECK(buildDecalData(nullptr, 0, Math::Mat4::Identity(), 16, data, sorted) == 0);
    CHECK(data.empty() && sorted.empty());
}

} // namespace

int main()
{
    testDefaults();
    testTransform();
    testSortAndFade();

    if (failures != 0)
    {
        printf("%d checks failed\n", failures);
        return 1;
    }
    printf("decal data: all checks passed\n");
    return 0;
}
