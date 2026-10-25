#include "Check.h"

#include "prisma3d/core/Pixmap.h"
#include "prisma3d/scene/PrimitiveMeshes.h"

#include <math.h>
#include <stddef.h>

using namespace prisma3d;

namespace
{

const float kEps = 1e-3f;

bool near(float a, float b, float eps = kEps)
{
    return fabsf(a - b) <= eps;
}

void checkStructure(const SurfaceData& s, size_t vertices, size_t indices, float uvTiles)
{
    CHECK(s.positions.size() == vertices);
    CHECK(s.normals.size() == vertices);
    CHECK(s.tangents.size() == vertices);
    CHECK(s.uvs.size() == vertices);
    CHECK(s.colors.empty());
    CHECK(s.uvs2.empty());
    CHECK(s.indices.size() == indices);
    CHECK(indices % 3 == 0);

    bool indicesInRange = true;
    for (size_t i = 0; i < s.indices.size(); ++i)
        if (s.indices[i] >= vertices) indicesInRange = false;
    CHECK(indicesInRange);

    bool normalsUnit = true;
    bool tangentsUnit = true;
    bool uvsInRange = true;
    for (size_t i = 0; i < s.positions.size() && i < s.normals.size() && i < s.tangents.size() &&
                       i < s.uvs.size();
            ++i)
    {
        if (!near(s.normals[i].Length(), 1.0f)) normalsUnit = false;
        const Math::Vec4& t = s.tangents[i];
        if (!near(t.xyz().Length(), 1.0f) || !(near(t.w, 1.0f) || near(t.w, -1.0f)))
            tangentsUnit = false;
        const Math::Vec2& uv = s.uvs[i];
        if (uv.x < -kEps || uv.y < -kEps || uv.x > uvTiles + kEps || uv.y > uvTiles + kEps)
            uvsInRange = false;
    }
    CHECK(normalsUnit);
    CHECK(tangentsUnit);
    CHECK(uvsInRange);
}

bool windingOutward(const SurfaceData& s)
{
    bool ok = true;
    int measured = 0;
    for (size_t i = 0; i + 2 < s.indices.size(); i += 3)
    {
        const std::uint32_t ia = s.indices[i];
        const std::uint32_t ib = s.indices[i + 1];
        const std::uint32_t ic = s.indices[i + 2];
        const Math::Vec3 a = s.positions[ia];
        const Math::Vec3 b = s.positions[ib];
        const Math::Vec3 c = s.positions[ic];
        const Math::Vec3 face = Math::Vec3::Cross(b - a, c - a);
        if (face.Length() < 1e-6f) continue;
        const Math::Vec3 normal = s.normals[ia] + s.normals[ib] + s.normals[ic];
        ++measured;
        if (!(Math::Vec3::Dot(face, normal) > 0.0f)) ok = false;
    }
    return ok && measured > 0;
}

void testPlane()
{
    SurfaceData s;
    planeMeshArrays(s, 4.0f, 2.0f, 4, 2, 3.0f);
    checkStructure(s, 5 * 3, 4 * 2 * 6, 3.0f);
    CHECK(windingOutward(s));
    bool flat = true;
    for (size_t i = 0; i < s.positions.size(); ++i)
    {
        if (s.positions[i].y != 0.0f) flat = false;
        if (!near(s.normals[i].x, 0.0f) || !near(s.normals[i].y, 1.0f) ||
                !near(s.normals[i].z, 0.0f))
            flat = false;
        if (!near(s.tangents[i].x, 1.0f) || !near(s.tangents[i].y, 0.0f) ||
                !near(s.tangents[i].z, 0.0f))
            flat = false;
    }
    CHECK(flat);
    CHECK(near(s.positions[0].x, -2.0f) && near(s.positions[0].z, -1.0f));
    CHECK(near(s.positions[14].x, 2.0f) && near(s.positions[14].z, 1.0f));
    CHECK(near(s.uvs[0].x, 0.0f) && near(s.uvs[0].y, 0.0f));
    CHECK(near(s.uvs[14].x, 3.0f) && near(s.uvs[14].y, 3.0f));

    SurfaceData degenerate;
    planeMeshArrays(degenerate, 1.0f, 1.0f, 0, -3);
    checkStructure(degenerate, 4, 6, 1.0f);
}

void testBox()
{
    SurfaceData s;
    boxMeshArrays(s, Math::Vec3(2.0f, 4.0f, 6.0f));
    checkStructure(s, 24, 36, 1.0f);
    CHECK(windingOutward(s));
    SurfaceData divided;
    boxMeshArrays(divided, Math::Vec3(1.0f, 1.0f, 1.0f), 2, 1, 3);
    CHECK(windingOutward(divided));
}

void testSphere()
{
    SurfaceData s;
    const float radius = 2.0f;
    sphereMeshArrays(s, radius, 5, 7);
    checkStructure(s, 6 * 8, 5 * 7 * 6, 1.0f);
    CHECK(windingOutward(s));
    bool onSphere = true;
    for (size_t i = 0; i < s.positions.size(); ++i)
    {
        if (!near(s.positions[i].Length(), radius)) onSphere = false;
        if (!near((s.positions[i] - s.normals[i] * radius).Length(), 0.0f)) onSphere = false;
    }
    CHECK(onSphere);
    CHECK(near(s.positions[0].y, radius));
    CHECK(near(s.positions[s.positions.size() - 1].y, -radius));

    SurfaceData tiny;
    sphereMeshArrays(tiny, 1.0f, 0, 0);
    checkStructure(tiny, 3 * 4, 2 * 3 * 6, 1.0f);

    SurfaceData defaults;
    sphereMeshArrays(defaults, 1.0f);
    checkStructure(defaults, 17 * 25, 16 * 24 * 6, 1.0f);
}

void testCylinder()
{
    SurfaceData s;
    const float radius = 1.5f;
    const float height = 3.0f;
    const int slices = 8;
    cylinderMeshArrays(s, radius, height, slices);
    checkStructure(s, 4 * slices + 6, 12 * slices, 1.0f);
    CHECK(windingOutward(s));
    bool sideOk = true;
    for (int i = 0; i < 2 * (slices + 1); ++i)
    {
        const Math::Vec3& p = s.positions[i];
        if (!near(sqrtf(p.x * p.x + p.z * p.z), radius)) sideOk = false;
        if (!near(s.normals[i].y, 0.0f)) sideOk = false;
        if (!near(p.y, i < slices + 1 ? 0.0f : height)) sideOk = false;
    }
    CHECK(sideOk);
    bool bounds = true;
    for (size_t i = 0; i < s.positions.size(); ++i)
    {
        const Math::Vec3& p = s.positions[i];
        if (sqrtf(p.x * p.x + p.z * p.z) > radius + kEps) bounds = false;
        if (p.y < -kEps || p.y > height + kEps) bounds = false;
    }
    CHECK(bounds);
    const size_t bottomCenter = 2 * (slices + 1);
    const size_t topCenter = bottomCenter + slices + 2;
    CHECK(near(s.positions[bottomCenter].y, 0.0f) && near(s.normals[bottomCenter].y, -1.0f));
    CHECK(near(s.positions[topCenter].y, height) && near(s.normals[topCenter].y, 1.0f));

    SurfaceData tiny;
    cylinderMeshArrays(tiny, 1.0f, 1.0f, 1);
    checkStructure(tiny, 4 * 3 + 6, 12 * 3, 1.0f);
}

void testCone()
{
    SurfaceData s;
    const float radius = 1.5f;
    const float height = 3.0f;
    const int slices = 8;
    coneMeshArrays(s, radius, height, slices);
    checkStructure(s, 3 * slices + 4, 9 * slices, 1.0f);
    CHECK(windingOutward(s));
    const float slope = (radius / height) / sqrtf(1.0f + (radius / height) * (radius / height));
    bool ok = true;
    for (int i = 0; i < slices + 1; ++i)
    {
        const Math::Vec3& base = s.positions[i];
        const Math::Vec3& apex = s.positions[slices + 1 + i];
        if (!near(sqrtf(base.x * base.x + base.z * base.z), radius) || !near(base.y, 0.0f))
            ok = false;
        if (!near(sqrtf(apex.x * apex.x + apex.z * apex.z), 0.0f) || !near(apex.y, height))
            ok = false;
        if (!near(s.normals[i].y, slope)) ok = false;
    }
    CHECK(ok);
    const size_t capCenter = 2 * (slices + 1);
    CHECK(near(s.normals[capCenter].y, -1.0f));
    CHECK(s.positions.size() == capCenter + slices + 2);
}

void testCapsule()
{
    SurfaceData s;
    const float radius = 1.0f;
    const float height = 2.0f;
    const int rings = 3;
    const int slices = 8;
    capsuleMeshArrays(s, radius, height, rings, slices);
    checkStructure(s, (2 * rings + 2) * (slices + 1), (2 * rings + 1) * slices * 6, 1.0f);
    CHECK(windingOutward(s));
    bool ok = true;
    float minY = 1e9f;
    float maxY = -1e9f;
    for (size_t i = 0; i < s.positions.size(); ++i)
    {
        const Math::Vec3& p = s.positions[i];
        float axisY = p.y;
        if (axisY < radius) axisY = radius;
        if (axisY > radius + height) axisY = radius + height;
        const Math::Vec3 axis(0.0f, axisY, 0.0f);
        if (!near((p - axis).Length(), radius)) ok = false;
        if (!near((p - axis - s.normals[i] * radius).Length(), 0.0f)) ok = false;
        if (p.y < minY) minY = p.y;
        if (p.y > maxY) maxY = p.y;
    }
    CHECK(ok);
    CHECK(near(minY, 0.0f));
    CHECK(near(maxY, height + 2.0f * radius));
    CHECK(near(s.uvs[0].y, 0.0f));
    CHECK(near(s.uvs[s.uvs.size() - 1].y, 1.0f));

    SurfaceData tiny;
    capsuleMeshArrays(tiny, 1.0f, 1.0f, 0, 0);
    checkStructure(tiny, 6 * 4, 5 * 3 * 6, 1.0f);
}

void testTorus()
{
    SurfaceData s;
    const float major = 3.0f;
    const float minor = 1.0f;
    torusMeshArrays(s, major, minor, 10, 6);
    checkStructure(s, 11 * 7, 10 * 6 * 6, 1.0f);
    CHECK(windingOutward(s));
    bool ok = true;
    for (size_t i = 0; i < s.positions.size(); ++i)
    {
        const Math::Vec3& p = s.positions[i];
        const float ringDistance = sqrtf(p.x * p.x + p.z * p.z) - major;
        if (!near(sqrtf(ringDistance * ringDistance + p.y * p.y), minor)) ok = false;
        const Math::Vec3 center(p.x - s.normals[i].x * minor, 0.0f, p.z - s.normals[i].z * minor);
        if (!near(sqrtf(center.x * center.x + center.z * center.z), major)) ok = false;
    }
    CHECK(ok);

    SurfaceData defaults;
    torusMeshArrays(defaults, 2.0f, 0.5f);
    checkStructure(defaults, 25 * 13, 24 * 12 * 6, 1.0f);

    SurfaceData tiny;
    torusMeshArrays(tiny, 2.0f, 0.5f, 1, 1);
    checkStructure(tiny, 4 * 4, 3 * 3 * 6, 1.0f);
}

void fillRamp(Pixmap& map)
{
    for (int y = 0; y < map.height; ++y)
        for (int x = 0; x < map.width; ++x)
        {
            const unsigned char v = static_cast<unsigned char>((x + y * map.width) * 10);
            map.set_pixel(static_cast<std::uint32_t>(x), static_cast<std::uint32_t>(y), v, 0, 0,
                    255);
        }
}

void testHeightfield()
{
    Pixmap map(4, 4, 4);
    CHECK(map.is_valid());
    fillRamp(map);
    const float cell = 2.0f;
    const float scale = 5.1f;
    SurfaceData s;
    heightfieldMeshArrays(s, map, cell, scale, 2.0f);
    checkStructure(s, 16, 9 * 6, 2.0f);
    CHECK(windingOutward(s));
    bool ok = true;
    for (int j = 0; j < 4; ++j)
        for (int i = 0; i < 4; ++i)
        {
            const Math::Vec3& p = s.positions[static_cast<size_t>(j * 4 + i)];
            const float expected = static_cast<float>((i + j * 4) * 10) / 255.0f * scale;
            if (!near(p.x, static_cast<float>(i) * cell) ||
                    !near(p.z, static_cast<float>(j) * cell) || !near(p.y, expected))
                ok = false;
            const Math::Vec2& uv = s.uvs[static_cast<size_t>(j * 4 + i)];
            if (!near(uv.x, static_cast<float>(i) / 3.0f * 2.0f) ||
                    !near(uv.y, static_cast<float>(j) / 3.0f * 2.0f))
                ok = false;
        }
    CHECK(ok);

    float heights[9];
    for (int j = 0; j < 3; ++j)
        for (int i = 0; i < 3; ++i) heights[j * 3 + i] = 0.5f * static_cast<float>(i);
    SurfaceData ramp;
    heightfieldMeshArrays(ramp, heights, 3, 3, 1.0f);
    checkStructure(ramp, 9, 4 * 6, 1.0f);
    const Math::Vec3 expectedNormal = Math::Vec3(-1.0f, 2.0f, 0.0f).Normalized();
    const Math::Vec3 expectedTangent = Math::Vec3(2.0f, 1.0f, 0.0f).Normalized();
    CHECK((ramp.normals[4] - expectedNormal).Length() < kEps);
    CHECK((ramp.tangents[4].xyz() - expectedTangent).Length() < kEps);
    CHECK(near(ramp.positions[5].y, 1.0f) && near(ramp.positions[5].x, 2.0f));

    SurfaceData empty;
    heightfieldMeshArrays(empty, heights, 1, 3, 1.0f);
    CHECK(empty.positions.empty() && empty.indices.empty());
    heightfieldMeshArrays(empty, nullptr, 3, 3, 1.0f);
    CHECK(empty.positions.empty() && empty.indices.empty());
}

void testHillsPlane()
{
    Pixmap flat(4, 4, 4);
    CHECK(flat.is_valid());
    flat.fill(128, 0, 0, 255);
    const float scale = 10.0f;
    const float height = 128.0f / 255.0f * scale;

    SurfaceData plane;
    planeMeshArrays(plane, 4.0f, 2.0f, 4, 2, 2.0f);
    SurfaceData hills;
    hillsPlaneMeshArrays(hills, 4.0f, 2.0f, 4, 2, flat, scale, 2.0f);
    checkStructure(hills, 15, 4 * 2 * 6, 2.0f);
    CHECK(windingOutward(hills));
    CHECK(hills.indices.size() == plane.indices.size());
    bool same = hills.positions.size() == plane.positions.size();
    for (size_t i = 0; same && i < plane.indices.size(); ++i)
        if (hills.indices[i] != plane.indices[i]) same = false;
    for (size_t i = 0; same && i < plane.positions.size(); ++i)
    {
        const Math::Vec3& a = plane.positions[i];
        const Math::Vec3& b = hills.positions[i];
        if (!near(a.x, b.x) || !near(a.z, b.z) || !near(b.y, height)) same = false;
        if ((plane.normals[i] - hills.normals[i]).Length() > kEps) same = false;
        if ((plane.tangents[i] - hills.tangents[i]).Length() > kEps) same = false;
        if ((plane.uvs[i] - hills.uvs[i]).Length() > kEps) same = false;
    }
    CHECK(same);

    Pixmap map(4, 4, 4);
    fillRamp(map);
    SurfaceData sloped;
    hillsPlaneMeshArrays(sloped, 3.0f, 3.0f, 3, 3, map, 25.5f);
    checkStructure(sloped, 16, 9 * 6, 1.0f);
    bool exact = true;
    for (int j = 0; j < 4; ++j)
        for (int i = 0; i < 4; ++i)
        {
            const float expected = static_cast<float>((i + j * 4) * 10) / 255.0f * 25.5f;
            if (!near(sloped.positions[static_cast<size_t>(j * 4 + i)].y, expected)) exact = false;
        }
    CHECK(exact);

    SurfaceData none;
    Pixmap invalid;
    hillsPlaneMeshArrays(none, 1.0f, 1.0f, 2, 2, invalid, 1.0f);
    CHECK(none.positions.empty() && none.indices.empty());
}

} // namespace

void generate(const PrimitiveRecipe& recipe, SurfaceData& s)
{
    const float* v = recipe.values;
    switch (recipe.kind)
    {
        case PrimitiveKind::Box:
            boxMeshArrays(s, Math::Vec3(v[0], v[1], v[2]));
            break;
        case PrimitiveKind::Plane:
            planeMeshArrays(s, v[0], v[1], static_cast<int>(v[2]), static_cast<int>(v[3]), v[4]);
            break;
        case PrimitiveKind::Sphere:
            sphereMeshArrays(s, v[0], static_cast<int>(v[1]), static_cast<int>(v[2]));
            break;
        case PrimitiveKind::Cylinder:
            cylinderMeshArrays(s, v[0], v[1], static_cast<int>(v[2]));
            break;
        case PrimitiveKind::Cone:
            coneMeshArrays(s, v[0], v[1], static_cast<int>(v[2]));
            break;
        case PrimitiveKind::Capsule:
            capsuleMeshArrays(s, v[0], v[1], static_cast<int>(v[2]), static_cast<int>(v[3]));
            break;
        case PrimitiveKind::Torus:
            torusMeshArrays(s, v[0], v[1], static_cast<int>(v[2]), static_cast<int>(v[3]));
            break;
        default:
            break;
    }
}

void testTriangleCount()
{
    const PrimitiveKind kinds[] = { PrimitiveKind::Box, PrimitiveKind::Plane, PrimitiveKind::Sphere,
        PrimitiveKind::Cylinder, PrimitiveKind::Cone, PrimitiveKind::Capsule,
        PrimitiveKind::Torus };
    for (PrimitiveKind kind: kinds)
    {
        PrimitiveRecipe recipe;
        primitiveDefaults(kind, recipe);
        for (int variant = 0; variant < 3; ++variant)
        {
            if (variant == 1)
            {
                for (std::size_t p = 0, at = 0; p < primitiveParamCount(kind); ++p)
                {
                    const PrimitiveParamDef& def = primitiveParam(kind, p);
                    const std::size_t width = def.type == PrimitiveParamType::Vec3 ? 3 : 1;
                    if (def.type == PrimitiveParamType::Count) recipe.values[at] = def.minimum;
                    at += width;
                }
            }
            else if (variant == 2)
            {
                primitiveDefaults(kind, recipe);
                if (kind == PrimitiveKind::Cylinder || kind == PrimitiveKind::Cone)
                    recipe.values[0] = 0.0f;
                if (kind == PrimitiveKind::Plane)
                {
                    recipe.values[2] = 7.0f;
                    recipe.values[3] = 3.0f;
                }
                if (kind == PrimitiveKind::Torus) recipe.values[2] = 31.0f;
            }
            SurfaceData s;
            generate(recipe, s);
            CHECK(s.indices.size() % 3 == 0);
            CHECK(primitiveTriangleCount(recipe) == s.indices.size() / 3);
        }
    }
}

int main()
{
    testPlane();
    testBox();
    testSphere();
    testCylinder();
    testCone();
    testCapsule();
    testTorus();
    testHeightfield();
    testHillsPlane();
    testTriangleCount();

    if (failures)
    {
        printf("%d checks failed\n", failures);
        return 1;
    }
    printf("primitives ok\n");
    return 0;
}
