#include "Check.h"

#include "prisma3d/scene/HairStrands.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

using namespace prisma3d;

namespace
{

const float kEpsilon = 1e-4f;

struct Scalp
{
    ct::Vector<Math::Vec3> positions;
    ct::Vector<Math::Vec3> normals;
    ct::Vector<float> lengths;
    ct::Vector<std::uint32_t> indices;
};

Scalp flatScalp(float half)
{
    Scalp scalp;
    scalp.positions.push_back(Math::Vec3(-half, 0.0f, -half));
    scalp.positions.push_back(Math::Vec3(half, 0.0f, -half));
    scalp.positions.push_back(Math::Vec3(half, 0.0f, half));
    scalp.positions.push_back(Math::Vec3(-half, 0.0f, half));
    for (int i = 0; i < 4; ++i)
    {
        scalp.normals.push_back(Math::Vec3(0.0f, 1.0f, 0.0f));
        scalp.lengths.push_back(1.0f);
    }
    const std::uint32_t indices[6] = { 0, 1, 2, 0, 2, 3 };
    for (std::uint32_t index: indices) scalp.indices.push_back(index);
    return scalp;
}

Scalp tiltedScalp(float half)
{
    Scalp scalp = flatScalp(half);
    for (std::size_t i = 0; i < scalp.normals.size(); ++i)
        scalp.normals[i] = Math::Vec3(0.4f, 1.0f, 0.0f).Normalized();
    return scalp;
}

void apply(HairStrands& hair, const Scalp& scalp)
{
    hair.setScalp(scalp.positions.data(), scalp.normals.data(), scalp.lengths.data(),
            static_cast<std::uint32_t>(scalp.positions.size()), scalp.indices.data(),
            static_cast<std::uint32_t>(scalp.indices.size()));
}

HairParams baseParams()
{
    HairParams params;
    params.strandCount = 16;
    params.segmentCount = 3;
    params.billboardCount = 1;
    params.randomSeed = 7;
    params.length = 0.9f;
    params.randomness = 0.0f;
    params.gravityPower = 0.0f;
    params.stiffness = 0.5f;
    params.drag = 0.1f;
    params.width = 0.1f;
    params.viewDistance = 100.0f;
    params.cameraBend = false;
    return params;
}

HairFrame baseFrame()
{
    HairFrame frame;
    frame.cameraPosition = Math::Vec3(0.0f, 4.0f, 6.0f);
    frame.cameraUp = Math::Vec3(0.0f, 1.0f, 0.0f);
    frame.deltaTime = 0.0f;
    return frame;
}

Math::Vec3 pairMidpoint(const HairStrands& hair, std::uint32_t vertex)
{
    return (hair.positions()[vertex] + hair.positions()[vertex + 1]) * 0.5f;
}

std::uint32_t rootVertex(const HairStrands& hair, std::uint32_t strand, std::uint32_t billboard)
{
    return strand * hair.verticesPerStrand() + billboard * 2;
}

std::uint32_t capVertex(const HairStrands& hair, std::uint32_t strand, std::uint32_t segment,
        std::uint32_t billboard)
{
    const std::uint32_t billboards = hair.params().billboardCount;
    return strand * hair.verticesPerStrand() + 2 * billboards + segment * 2 * billboards +
           billboard * 2;
}

bool near(float a, float b, float epsilon = kEpsilon)
{
    return fabsf(a - b) <= epsilon;
}

bool near(const Math::Vec3& a, const Math::Vec3& b, float epsilon = kEpsilon)
{
    return (a - b).Length() <= epsilon;
}

void testRestPose()
{
    HairStrands hair;
    hair.setParams(baseParams());
    const Scalp scalp = flatScalp(1.0f);
    apply(hair, scalp);
    CHECK(hair.scalpTriangleCount() == 2);
    hair.simulate(baseFrame());

    const HairParams& params = hair.params();
    CHECK(hair.vertexCount() == params.strandCount * (params.segmentCount * 2 + 2));
    CHECK(hair.indexCount() == 6 * params.strandCount * params.segmentCount);

    const float segmentLength = params.length / static_cast<float>(params.segmentCount);
    const float halfWidth = params.width * params.segmentCount * segmentLength * 0.5f;
    bool rootsOnScalp = true;
    bool capsAlongNormal = true;
    bool widthsRight = true;
    bool uvsRight = true;
    for (std::uint32_t strand = 0; strand < params.strandCount; ++strand)
    {
        const std::uint32_t root = rootVertex(hair, strand, 0);
        const Math::Vec3 base = pairMidpoint(hair, root);
        if (fabsf(base.y) > kEpsilon || fabsf(base.x) > 1.0f + kEpsilon ||
                fabsf(base.z) > 1.0f + kEpsilon)
            rootsOnScalp = false;
        const float width = (hair.positions()[root + 1] - hair.positions()[root]).Length();
        if (!near(width, 2.0f * halfWidth, 1e-3f)) widthsRight = false;
        if (!near(hair.uvs()[root].x, 0.0f) || !near(hair.uvs()[root + 1].x, 1.0f) ||
                !near(hair.uvs()[root].y, 1.0f))
            uvsRight = false;
        for (std::uint32_t segment = 0; segment < params.segmentCount; ++segment)
        {
            const std::uint32_t cap = capVertex(hair, strand, segment, 0);
            const Math::Vec3 tip = pairMidpoint(hair, cap);
            const Math::Vec3 expected =
                    base + Math::Vec3(0.0f, 1.0f, 0.0f) *
                                   (segmentLength * static_cast<float>(segment + 1));
            if (!near(tip, expected, 1e-3f)) capsAlongNormal = false;
            if (!near(hair.uvs()[cap].y, 1.0f - static_cast<float>(segment + 1) /
                                                         static_cast<float>(params.segmentCount)))
                uvsRight = false;
            if (!near(hair.normals()[cap], Math::Vec3(0.0f, 1.0f, 0.0f), 1e-3f))
                capsAlongNormal = false;
        }
    }
    CHECK(rootsOnScalp);
    CHECK(capsAlongNormal);
    CHECK(widthsRight);
    CHECK(uvsRight);

    bool indicesValid = true;
    const std::uint32_t billboards = params.billboardCount;
    for (std::uint32_t i = 0; i < hair.indexCount(); i += 6)
    {
        const std::uint32_t* tri = hair.indices() + i;
        for (int k = 0; k < 6; ++k)
            if (tri[k] >= hair.vertexCount()) indicesValid = false;
        if (tri[1] != tri[0] + 1 || tri[2] != tri[0] + 2 * billboards || tri[3] != tri[2] ||
                tri[4] != tri[1] || tri[5] != tri[2] + 1)
            indicesValid = false;
    }
    CHECK(indicesValid);
}

void testDeterminism()
{
    const Scalp scalp = flatScalp(1.0f);
    HairStrands a;
    a.setParams(baseParams());
    apply(a, scalp);
    a.simulate(baseFrame());

    HairStrands b;
    b.setParams(baseParams());
    apply(b, scalp);
    b.simulate(baseFrame());

    CHECK(memcmp(a.positions(), b.positions(), sizeof(Math::Vec3) * a.vertexCount()) == 0);

    HairParams other = baseParams();
    other.randomSeed = 8;
    HairStrands c;
    c.setParams(other);
    apply(c, scalp);
    c.simulate(baseFrame());
    CHECK(memcmp(a.positions(), c.positions(), sizeof(Math::Vec3) * a.vertexCount()) != 0);

    bool allInside = true;
    for (std::uint32_t strand = 0; strand < other.strandCount; ++strand)
    {
        const Math::Vec3 base = pairMidpoint(c, rootVertex(c, strand, 0));
        if (fabsf(base.x) > 1.0f + kEpsilon || fabsf(base.z) > 1.0f + kEpsilon) allInside = false;
    }
    CHECK(allInside);
}

void testGravityKeepsLength()
{
    HairStrands hair;
    HairParams params = baseParams();
    params.gravityPower = 10.0f;
    params.stiffness = 0.2f;
    hair.setParams(params);
    const Scalp scalp = tiltedScalp(1.0f);
    const Math::Vec3 axis = scalp.normals[0];
    apply(hair, scalp);
    HairFrame frame = baseFrame();
    frame.deltaTime = 1.0f / 60.0f;
    for (int i = 0; i < 60; ++i) hair.simulate(frame);

    const float segmentLength = params.length / static_cast<float>(params.segmentCount);
    bool lengthsKept = true;
    bool bent = false;
    bool abovePlane = true;
    for (std::uint32_t strand = 0; strand < params.strandCount; ++strand)
    {
        const Math::Vec3 base = pairMidpoint(hair, rootVertex(hair, strand, 0));
        Math::Vec3 previous = base;
        for (std::uint32_t segment = 0; segment < params.segmentCount; ++segment)
        {
            const Math::Vec3 tip = hair.tail(strand, segment);
            const float along = (tip - previous).Dot(axis);
            const float length = (tip - previous).Length();
            if (length > segmentLength + 1e-3f) lengthsKept = false;
            if (along > 1e-3f && !near(length, segmentLength, 1e-3f)) lengthsKept = false;
            if (along < -1e-3f) abovePlane = false;
            const Math::Vec3 rest = base + axis * (segmentLength * static_cast<float>(segment + 1));
            if (tip.y < rest.y - 0.05f) bent = true;
            previous = tip;
        }
    }
    CHECK(lengthsKept);
    CHECK(bent);
    CHECK(abovePlane);

    HairStrands stiff;
    params.stiffness = 40.0f;
    stiff.setParams(params);
    apply(stiff, scalp);
    for (int i = 0; i < 60; ++i) stiff.simulate(frame);
    float softSum = 0.0f;
    float stiffSum = 0.0f;
    for (std::uint32_t strand = 0; strand < params.strandCount; ++strand)
    {
        softSum += hair.tail(strand, params.segmentCount - 1).y;
        stiffSum += stiff.tail(strand, params.segmentCount - 1).y;
    }
    CHECK(stiffSum > softSum);
}

void testSphereCollider()
{
    HairStrands hair;
    HairParams params = baseParams();
    params.strandCount = 64;
    params.gravityPower = 4.0f;
    params.stiffness = 0.0f;
    hair.setParams(params);
    apply(hair, flatScalp(0.3f));
    const HairCollider collider = HairCollider::sphere(Math::Vec3(0.8f, 0.3f, 0.0f), 0.5f);
    HairFrame frame = baseFrame();
    frame.deltaTime = 1.0f / 60.0f;
    frame.colliders = &collider;
    frame.colliderCount = 1;
    const float segmentLength = params.length / static_cast<float>(params.segmentCount);
    const float halfWidth = params.width * params.segmentCount * segmentLength * 0.5f;
    const float segmentRadius = halfWidth > segmentLength * 0.5f ? halfWidth : segmentLength * 0.5f;
    for (int i = 0; i < 90; ++i) hair.simulate(frame);
    bool outside = true;
    for (std::uint32_t strand = 0; strand < params.strandCount; ++strand)
        for (std::uint32_t segment = 0; segment < params.segmentCount; ++segment)
        {
            const Math::Vec3 tip = pairMidpoint(hair, capVertex(hair, strand, segment, 0));
            if ((tip - collider.position).Length() < collider.radius + segmentRadius - 1e-3f)
                outside = false;
        }
    CHECK(outside);
}

void testDistanceCull()
{
    HairStrands hair;
    HairParams params = baseParams();
    params.viewDistance = 1.0f;
    hair.setParams(params);
    apply(hair, flatScalp(1.0f));
    HairFrame frame = baseFrame();
    frame.cameraPosition = Math::Vec3(50.0f, 0.0f, 0.0f);
    hair.simulate(frame);
    bool collapsed = true;
    for (std::uint32_t strand = 0; strand < params.strandCount; ++strand)
    {
        const std::uint32_t first = strand * hair.verticesPerStrand();
        for (std::uint32_t v = 1; v < hair.verticesPerStrand(); ++v)
            if (!near(hair.positions()[first + v], hair.positions()[first])) collapsed = false;
    }
    CHECK(collapsed);

    frame.cameraPosition = Math::Vec3(0.0f, 0.5f, 0.5f);
    hair.simulate(frame);
    bool expanded = false;
    for (std::uint32_t strand = 0; strand < params.strandCount; ++strand)
    {
        const std::uint32_t first = strand * hair.verticesPerStrand();
        if (!near(hair.positions()[first + 1], hair.positions()[first])) expanded = true;
    }
    CHECK(expanded);
}

void testBillboards()
{
    HairStrands hair;
    HairParams params = baseParams();
    params.billboardCount = 2;
    params.segmentCount = 2;
    hair.setParams(params);
    apply(hair, flatScalp(1.0f));
    hair.simulate(baseFrame());
    CHECK(hair.vertexCount() == params.strandCount * (params.segmentCount * 2 + 2) * 2);
    CHECK(hair.indexCount() == 6 * params.strandCount * params.segmentCount * 2);
    bool rotated = true;
    bool sameBase = true;
    for (std::uint32_t strand = 0; strand < params.strandCount; ++strand)
    {
        const std::uint32_t first = rootVertex(hair, strand, 0);
        const std::uint32_t second = rootVertex(hair, strand, 1);
        const Math::Vec3 sideA = hair.positions()[first + 1] - hair.positions()[first];
        const Math::Vec3 sideB = hair.positions()[second + 1] - hair.positions()[second];
        if (near(sideA.Normalized(), sideB.Normalized(), 1e-2f)) rotated = false;
        if (!near(pairMidpoint(hair, first), pairMidpoint(hair, second), 1e-3f)) sameBase = false;
    }
    CHECK(rotated);
    CHECK(sameBase);
}

void testWind()
{
    HairStrands hair;
    HairParams params = baseParams();
    params.stiffness = 0.0f;
    hair.setParams(params);
    apply(hair, flatScalp(1.0f));
    HairFrame frame = baseFrame();
    frame.deltaTime = 1.0f / 60.0f;
    frame.windDirection = Math::Vec3(1.0f, 0.0f, 0.0f);
    frame.windSpeed = 1.0f;
    float moved = 0.0f;
    for (int i = 0; i < 30; ++i)
    {
        frame.time = static_cast<float>(i) / 60.0f;
        hair.simulate(frame);
    }
    for (std::uint32_t strand = 0; strand < params.strandCount; ++strand)
    {
        const Math::Vec3 base = pairMidpoint(hair, rootVertex(hair, strand, 0));
        const Math::Vec3 tip =
                pairMidpoint(hair, capVertex(hair, strand, params.segmentCount - 1, 0));
        moved += fabsf(tip.x - base.x);
    }
    CHECK(moved > 0.01f);
    const Math::Vec3 none = hair.sampleWind(Math::Vec3(0.3f, 0.2f, 0.1f), 0.0f);
    CHECK(near(none, Math::Vec3(0.0f, 0.0f, 0.0f)));
    const float a = HairStrands::computeWind(Math::Vec3(3.0f, 4.0f, 5.0f), 0.5f, 1.0f, 5.0f, 1.0f);
    const float b = HairStrands::computeWind(Math::Vec3(3.0f, 4.0f, 5.0f), 0.5f, 1.0f, 5.0f, 1.0f);
    CHECK(a == b && a >= -1.0f && a <= 1.0f);

    HairStrands lattice;
    lattice.setParams(params);
    apply(lattice, flatScalp(1.0f));
    HairFrame windy = baseFrame();
    windy.windDirection = Math::Vec3(0.0f, 0.0f, 1.0f);
    windy.time = 2.75f;
    windy.windWaveSize = 1.3f;
    lattice.simulate(windy);
    bool identical = true;
    for (int z = 0; z < 32 && identical; z += 5)
        for (int y = 0; y < 32 && identical; y += 7)
            for (int x = 0; x < 32; x += 3)
            {
                const Math::Vec3 uvw((static_cast<float>(x) + 0.5f) / 32.0f,
                        (static_cast<float>(y) + 0.5f) / 32.0f,
                        (static_cast<float>(z) + 0.5f) / 32.0f);
                const float sampled = lattice.sampleWind(uvw, 1.0f).z;
                const float direct = HairStrands::computeWind(Math::Vec3(static_cast<float>(x),
                                                                      static_cast<float>(y),
                                                                      static_cast<float>(z)),
                        windy.time, windy.windSpeed, windy.windRandomness, windy.windWaveSize);
                if (fabsf(sampled - direct) > 1e-5f) identical = false;
            }
    CHECK(identical);
}

void testEligibleTriangles()
{
    Scalp scalp;
    scalp.positions.push_back(Math::Vec3(-1.0f, 0.0f, 0.0f));
    scalp.positions.push_back(Math::Vec3(0.0f, 0.0f, 1.0f));
    scalp.positions.push_back(Math::Vec3(0.0f, 0.0f, -1.0f));
    scalp.positions.push_back(Math::Vec3(1.0f, 0.0f, 1.0f));
    scalp.positions.push_back(Math::Vec3(1.0f, 0.0f, -1.0f));
    for (int i = 0; i < 5; ++i)
    {
        scalp.normals.push_back(Math::Vec3(0.0f, 1.0f, 0.0f));
        scalp.lengths.push_back(i < 2 ? 1.0f : 0.0f);
    }
    const std::uint32_t indices[6] = { 0, 1, 2, 2, 3, 4 };
    for (std::uint32_t index: indices) scalp.indices.push_back(index);

    HairStrands hair;
    HairParams params = baseParams();
    params.strandCount = 40;
    hair.setParams(params);
    apply(hair, scalp);
    CHECK(hair.scalpTriangleCount() == 1);
    hair.simulate(baseFrame());
    bool onFirst = true;
    bool lengthsVary = false;
    const float full = params.length / static_cast<float>(params.segmentCount);
    for (std::uint32_t strand = 0; strand < params.strandCount; ++strand)
    {
        const Math::Vec3 base = pairMidpoint(hair, rootVertex(hair, strand, 0));
        if (base.x > kEpsilon) onFirst = false;
        const Math::Vec3 tip = pairMidpoint(hair, capVertex(hair, strand, 0, 0));
        const float length = (tip - base).Length();
        if (length < full - 1e-3f) lengthsVary = true;
        if (length > full + 1e-3f) onFirst = false;
    }
    CHECK(onFirst);
    CHECK(lengthsVary);
}

void testAtlasAndBounds()
{
    HairStrands hair;
    HairParams params = baseParams();
    HairAtlasRect small;
    small.texMulAdd = Math::Vec4(0.5f, 1.0f, 0.0f, 0.0f);
    small.size = 0.5f;
    HairAtlasRect large;
    large.texMulAdd = Math::Vec4(0.5f, 1.0f, 0.5f, 0.0f);
    large.size = 2.0f;
    params.atlasRects.push_back(small);
    params.atlasRects.push_back(large);
    params.strandCount = 64;
    hair.setParams(params);
    apply(hair, flatScalp(1.0f));
    hair.simulate(baseFrame());
    const float unit = params.length / static_cast<float>(params.segmentCount);
    bool sizesMatch = true;
    bool uvInRect = true;
    for (std::uint32_t strand = 0; strand < params.strandCount; ++strand)
    {
        const std::uint32_t root = rootVertex(hair, strand, 0);
        const Math::Vec3 base = pairMidpoint(hair, root);
        const float length = (pairMidpoint(hair, capVertex(hair, strand, 0, 0)) - base).Length();
        const bool isSmall = near(length, unit * 0.5f, 1e-3f);
        const bool isLarge = near(length, unit * 2.0f, 1e-3f);
        if (!isSmall && !isLarge) sizesMatch = false;
        const float u0 = hair.uvs()[root].x;
        const float u1 = hair.uvs()[root + 1].x;
        if (isSmall && !(near(u0, 0.0f) && near(u1, 0.5f))) uvInRect = false;
        if (isLarge && !(near(u0, 0.5f) && near(u1, 1.0f))) uvInRect = false;
    }
    CHECK(sizesMatch);
    CHECK(uvInRect);

    Math::Box scalp;
    scalp.min = Math::Vec3(-1.0f, 0.0f, -1.0f);
    scalp.max = Math::Vec3(1.0f, 0.0f, 1.0f);
    const Math::Box bounds = HairStrands::expandBounds(scalp, params,
            Math::Mat4::Translation(Math::Vec3(10.0f, 0.0f, 0.0f)));
    CHECK(near(bounds.min, Math::Vec3(7.0f, -2.0f, -3.0f)));
    CHECK(near(bounds.max, Math::Vec3(13.0f, 2.0f, 3.0f)));
}

void testCameraBend()
{
    HairStrands hair;
    HairParams params = baseParams();
    params.cameraBend = true;
    params.segmentCount = 1;
    hair.setParams(params);
    Scalp scalp = flatScalp(0.5f);
    for (std::size_t i = 0; i < scalp.normals.size(); ++i)
        scalp.normals[i] = Math::Vec3(1.0f, 0.0f, 0.0f);
    apply(hair, scalp);
    hair.simulate(baseFrame());
    const Math::Vec3 expected = Math::Vec3(1.0f, 0.8f, 0.0f).Normalized();
    bool bentToCamera = true;
    for (std::uint32_t strand = 0; strand < params.strandCount; ++strand)
    {
        const Math::Vec3 base = pairMidpoint(hair, rootVertex(hair, strand, 0));
        const Math::Vec3 tip = pairMidpoint(hair, capVertex(hair, strand, 0, 0));
        if (!near((tip - base).Normalized(), expected, 1e-3f)) bentToCamera = false;
        if (!near(hair.normals()[capVertex(hair, strand, 0, 0)], expected, 1e-3f))
            bentToCamera = false;
    }
    CHECK(bentToCamera);
}

void testPlaneCollider()
{
    HairStrands hair;
    HairParams params = baseParams();
    params.gravityPower = 0.5f;
    params.stiffness = 0.0f;
    params.segmentCount = 1;
    params.length = 0.3f;
    params.strandCount = 32;
    hair.setParams(params);
    Scalp scalp = flatScalp(0.02f);
    for (std::size_t i = 0; i < scalp.normals.size(); ++i)
    {
        scalp.normals[i] = Math::Vec3(1.0f, 0.0f, 0.0f);
        scalp.positions[i] = Math::Vec3(0.0f, scalp.positions[i].z + 2.0f, scalp.positions[i].x);
    }
    apply(hair, scalp);
    const float planeY = 1.7f;
    const HairCollider plane =
            HairCollider::plane(Math::Mat4::Translation(Math::Vec3(0.15f, planeY, 0.0f)),
                    Math::Vec3(0.0f, 0.0f, 0.0f), 5.0f);
    CHECK(near(plane.normal, Math::Vec3(0.0f, 1.0f, 0.0f)));
    CHECK(near(plane.position, Math::Vec3(0.15f, planeY, 0.0f)));
    HairFrame frame = baseFrame();
    frame.deltaTime = 1.0f / 60.0f;
    frame.colliders = &plane;
    frame.colliderCount = 1;
    HairStrands free;
    free.setParams(params);
    apply(free, scalp);
    HairFrame freeFrame = frame;
    freeFrame.colliders = nullptr;
    freeFrame.colliderCount = 0;
    for (int i = 0; i < 400; ++i)
    {
        hair.simulate(frame);
        free.simulate(freeFrame);
    }
    bool abovePlane = true;
    bool fell = false;
    bool freeFell = true;
    for (std::uint32_t strand = 0; strand < params.strandCount; ++strand)
    {
        const Math::Vec3 base = pairMidpoint(hair, rootVertex(hair, strand, 0));
        const Math::Vec3 tip = pairMidpoint(hair, capVertex(hair, strand, 0, 0));
        if (tip.y < planeY - 0.02f) abovePlane = false;
        if (tip.y < base.y - 0.05f) fell = true;
        const Math::Vec3 freeTip = pairMidpoint(free, capVertex(free, strand, 0, 0));
        if (freeTip.y > 1.75f) freeFell = false;
    }
    CHECK(abovePlane);
    CHECK(fell);
    CHECK(freeFell);
}

} // namespace

int main()
{
    testRestPose();
    testDeterminism();
    testGravityKeepsLength();
    testSphereCollider();
    testDistanceCull();
    testBillboards();
    testWind();
    testEligibleTriangles();
    testAtlasAndBounds();
    testCameraBend();
    testPlaneCollider();

    if (failures) printf("%d check(s) failed\n", failures);
    else
        printf("all checks passed\n");
    return failures ? 1 : 0;
}
