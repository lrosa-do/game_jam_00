#include "check.h"

#include <kx/world.h>

#include <math.h>

using namespace kx;

namespace
{

float BodyArea(const Body& body)
{
    float area = 0.0f;
    for (int s = 0; s < body.ShapeCount(); ++s)
    {
        const Polygon& polygon = body.Shapes()[s].polygon;
        if (body.Shapes()[s].type != ShapeType::Polygon)
            continue;
        for (int i = 0; i < polygon.count; ++i)
            area += 0.5f * Cross(polygon.vertices[i], polygon.vertices[(i + 1) % polygon.count]);
    }
    return area;
}

}

int main()
{
    // Shaving one vertex off an octagon: the rest is a 9-gon. No area lost.
    {
        World world(vec2(0.0f, 0.0f));
        vec2 octagon[8];
        for (int i = 0; i < 8; ++i)
            octagon[i] = vec2(50.0f * cosf(float(i) * kPi / 4.0f), 50.0f * sinf(float(i) * kPi / 4.0f));
        Body* body = world.CreatePolygon(vec2(0.0f, 0.0f), octagon, 8, 1.0f);
        const float area = BodyArea(*body);
        Body* positive = nullptr;
        Body* negative = nullptr;
        CHECK(Slice(world, body, vec2(45.0f, 0.0f), vec2(1.0f, 0.0f), &positive, &negative));
        CHECK(positive && negative);
        if (positive && negative)
        {
            CHECK_NEAR(BodyArea(*positive) + BodyArea(*negative), area, 1.0f);
            CHECK_NEAR(BodyArea(*positive), 60.36f, 0.5f);
        }
    }

    {
        World world(vec2(0.0f, 0.0f));
        Body* body = world.CreateBody(BodyType::Dynamic, vec2(0.0f, 0.0f));
        body->AddBox(10.0f, 10.0f, vec2(-20.0f, 0.0f), 1.0f);
        body->AddBox(10.0f, 10.0f, vec2(20.0f, 0.0f), 1.0f);
        body->SetShapeFilter(0, 0x0002, 0x00F0, 3);
        body->SetShapeFilter(1, 0x0004, 0x0F00, -2);
        Body* positive = nullptr;
        Body* negative = nullptr;
        CHECK(Slice(world, body, vec2(0.0f, 0.0f), vec2(0.0f, 1.0f), &positive, &negative));
        if (positive && negative)
        {
            CHECK(positive->ShapeCount() == 2);
            for (int s = 0; s < positive->ShapeCount(); ++s)
            {
                const Filter filter = positive->ShapeFilter(s);
                const bool left = positive->Shapes()[s].polygon.centroid.x < 0.0f;
                CHECK(filter.category == (left ? 0x0002 : 0x0004));
                CHECK(filter.mask == (left ? 0x00F0 : 0x0F00));
                CHECK(filter.group == (left ? 3 : -2));
            }
        }
    }

    // A spinning body: each piece moves with v + w x (c_piece - c_source).
    {
        World world(vec2(0.0f, 0.0f));
        Body* body = world.CreateBox(vec2(0.0f, 0.0f), 20.0f, 10.0f, 1.0f);
        body->SetVelocity(vec2(5.0f, 0.0f));
        body->SetAngularVelocity(2.0f);
        Body* positive = nullptr;
        Body* negative = nullptr;
        CHECK(Slice(world, body, vec2(0.0f, 0.0f), vec2(1.0f, 0.0f), &positive, &negative));
        if (positive && negative)
        {
            // Piece centres at x = +-10: w x r = 2 * (-0, 10) = (0, +-20).
            CHECK_NEAR(positive->WorldCenter().x, 10.0f, 1e-3f);
            CHECK_NEAR(positive->Velocity().x, 5.0f, 1e-3f);
            CHECK_NEAR(positive->Velocity().y, 20.0f, 1e-3f);
            CHECK_NEAR(negative->Velocity().y, -20.0f, 1e-3f);
            CHECK_NEAR(positive->AngularVelocity(), 2.0f, 1e-6f);
        }
    }

    {
        World world(vec2(0.0f, 0.0f));
        vec2 points[4] = {vec2(-60.0f, 0.0f), vec2(-20.0f, 0.0f), vec2(20.0f, 0.0f), vec2(60.0f, 0.0f)};
        Body* chain = world.CreateChain(points, 4, false);
        Body* positive = nullptr;
        Body* negative = nullptr;
        CHECK(Slice(world, chain, vec2(0.0f, 0.0f), vec2(1.0f, 0.0f), &positive, &negative));
        if (positive && negative)
        {
            CHECK(positive->ShapeCount() == 2);
            for (int s = 0; s < positive->ShapeCount(); ++s)
                CHECK(positive->Shapes()[s].edge.oneSided);
            for (int s = 0; s < negative->ShapeCount(); ++s)
                CHECK(negative->Shapes()[s].edge.oneSided);
        }
    }

    return TEST_RESULT();
}
