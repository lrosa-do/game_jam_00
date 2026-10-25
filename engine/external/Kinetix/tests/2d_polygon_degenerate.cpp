#include "check.h"

#include <kx/world.h>

#include <math.h>

using namespace kx;

namespace
{

float PolygonArea(const Polygon& polygon)
{
    float area = 0.0f;
    for (int i = 0; i < polygon.count; ++i)
        area += Cross(polygon.vertices[i], polygon.vertices[(i + 1) % polygon.count]);
    return 0.5f * area;
}

float BodyArea(const Body& body)
{
    float area = 0.0f;
    for (int i = 0; i < body.ShapeCount(); ++i)
        if (body.Shapes()[i].type == ShapeType::Polygon)
            area += PolygonArea(body.Shapes()[i].polygon);
    return area;
}

bool Finite(float value)
{
    return value == value && value - value == 0.0f;
}

}

int main()
{
    {
        Polygon polygon;
        const vec2 tiny[3] = {vec2(100.0f, 100.0f), vec2(100.1f, 100.0f), vec2(100.0f, 100.1f)};
        CHECK(!polygon.Set(tiny, 3));
        CHECK(polygon.count == 0);
        const vec2 line[4] = {vec2(0.0f, 0.0f), vec2(10.0f, 0.0f), vec2(20.0f, 0.0f), vec2(30.0f, 0.01f)};
        CHECK(!polygon.Set(line, 4));
        CHECK(!polygon.Set(line, 2));

        World world(vec2(0.0f, 0.0f));
        Body* body = world.CreateBody(BodyType::Dynamic, vec2(0.0f, 0.0f));
        CHECK(body->AddPolygon(tiny, 3, 1.0f) == 0);
        CHECK(body->AddPolygon(line, 4, 1.0f) == 0);
        CHECK(body->ShapeCount() == 0);
        CHECK(body->AddMesh(line, 4, 1.0f) == 0);
        CHECK(body->ShapeCount() == 0);
    }

    {
        vec2 points[12];
        for (int i = 0; i < 8; ++i)
            points[i] = vec2(-5.0f + float(i), 3.0f - float(i % 3));
        points[8] = vec2(-10.0f, -10.0f);
        points[9] = vec2(10.0f, -10.0f);
        points[10] = vec2(10.0f, 10.0f);
        points[11] = vec2(-10.0f, 10.0f);
        Polygon polygon;
        CHECK(polygon.Set(points, 12));
        CHECK(polygon.count == 4);
        CHECK_NEAR(PolygonArea(polygon), 400.0f, 1e-2f);

        vec2 circle[16];
        for (int i = 0; i < 16; ++i)
            circle[i] = vec2(50.0f * cosf(float(i) * kPi / 8.0f), 50.0f * sinf(float(i) * kPi / 8.0f));
        CHECK(!polygon.Set(circle, 16));
        World world(vec2(0.0f, 0.0f));
        Body* body = world.CreateBody(BodyType::Dynamic, vec2(0.0f, 0.0f));
        CHECK(body->AddPolygon(circle, 16, 1.0f) == 0);
        CHECK(body->AddMesh(circle, 16, 1.0f) > 0);
        float expected = 0.0f;
        for (int i = 0; i < 16; ++i)
            expected += 0.5f * Cross(circle[i], circle[(i + 1) % 16]);
        CHECK_NEAR(BodyArea(*body), expected, 1.0f);
    }

    // A 100-point circle triangulates into 98 triangles; merged into convex
    // pieces it fits a body, with nothing dropped.
    {
        const int count = 100;
        vec2 outline[count];
        float expected = 0.0f;
        for (int i = 0; i < count; ++i)
            outline[i] = vec2(200.0f * cosf(2.0f * kPi * float(i) / count), 200.0f * sinf(2.0f * kPi * float(i) / count));
        for (int i = 0; i < count; ++i)
            expected += 0.5f * Cross(outline[i], outline[(i + 1) % count]);
        World world(vec2(0.0f, 0.0f));
        Body* body = world.CreateBody(BodyType::Dynamic, vec2(0.0f, 0.0f));
        const int added = body->AddMesh(outline, count, 1.0f);
        CHECK(added > 0 && added <= Body::kMaxShapes);
        CHECK_NEAR(BodyArea(*body), expected, expected * 1e-3f);
    }

    {
        const int teeth = 40;
        vec2 comb[teeth * 2 + 2];
        int n = 0;
        for (int i = 0; i < teeth; ++i)
        {
            comb[n++] = vec2(float(i) * 20.0f, 100.0f);
            comb[n++] = vec2(float(i) * 20.0f + 10.0f, 10.0f);
        }
        comb[n++] = vec2(float(teeth) * 20.0f, 0.0f);
        comb[n++] = vec2(0.0f, 0.0f);
        World world(vec2(0.0f, 0.0f));
        Body* body = world.CreateBody(BodyType::Dynamic, vec2(0.0f, 0.0f));
        body->AddBox(5.0f, 5.0f, vec2(0.0f, 0.0f), 1.0f);
        const float massBefore = body->Mass();
        CHECK(body->AddMesh(comb, n, 1.0f) == 0);
        CHECK(body->ShapeCount() == 1);
        CHECK_NEAR(body->Mass(), massBefore, 1e-3f);
    }

    {
        Polygon flat;
        flat.count = 3;
        flat.vertices[0] = vec2(0.0f, 0.0f);
        flat.vertices[1] = vec2(1.0f, 0.0f);
        flat.vertices[2] = vec2(2.0f, 0.0f);
        MassData mass = flat.ComputeMass(1.0f);
        CHECK(Finite(mass.mass) && Finite(mass.center.x) && Finite(mass.center.y) && Finite(mass.I));
    }

    {
        World world(vec2(0.0f, 0.0f));
        Body* box = world.CreateBox(vec2(0.0f, 0.0f), 10.0f, 10.0f, 1.0f);
        Body* positive = nullptr;
        Body* negative = nullptr;
        // 0.05 px from the right side: that side would be a degenerate sliver.
        CHECK(!Slice(world, box, vec2(9.95f, 0.0f), vec2(1.0f, 0.0f), &positive, &negative));
        CHECK(world.BodyCount() == 1);
    }

    return TEST_RESULT();
}
