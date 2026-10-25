#include "check.h"

#include <kinetix/core/Color.h>
#include <kx/joints/distancejoint.h>
#include <kx/joints/wheeljoint.h>
#include <kx/tilemapcollider.h>
#include <kx/world.h>

#include <math.h>
#include <string.h>

using namespace kx;

namespace
{

float RunWheel(const vec2& axis)
{
    World world(vec2(0.0f, 500.0f));
    Body* chassis = world.CreateBody(BodyType::Static, vec2(0.0f, 0.0f));
    chassis->AddBox(20.0f, 5.0f, vec2(0.0f, 0.0f), 1.0f);
    Body* wheel = world.CreateCircle(vec2(0.0f, 30.0f), 8.0f, 1.0f);
    world.AddJoint(new WheelJoint(chassis, wheel, vec2(0.0f, 30.0f), axis, 4.0f, 0.7f));
    for (int i = 0; i < 30; ++i)
        world.Step(1.0f / 60.0f);
    return wheel->Position().y;
}

float Distance(const Body* a, const Body* b)
{
    vec2 d = b->Position() - a->Position();
    return sqrtf(Dot(d, d));
}

}

int main()
{
    CHECK_NEAR(RunWheel(vec2(0.0f, 10.0f)), RunWheel(vec2(0.0f, 1.0f)), 1e-3f);

    {
        World world(vec2(0.0f, 500.0f));
        Body* anchor = world.CreateBody(BodyType::Static, vec2(0.0f, 0.0f));
        Body* bob = world.CreateCircle(vec2(0.0f, 100.0f), 5.0f, 1.0f);
        DistanceJoint* spring = new DistanceJoint(anchor, bob, vec2(0.0f, 0.0f), vec2(0.0f, 100.0f));
        spring->SetSpring(1.0f, 0.1f);
        world.AddJoint(spring);
        float longest = 0.0f;
        for (int i = 0; i < 60; ++i)
        {
            world.Step(1.0f / 60.0f);
            if (Distance(anchor, bob) > longest)
                longest = Distance(anchor, bob);
        }
        CHECK(longest > 110.0f);
    }

    // Rest length clamped into the range: a 100 px joint limited to
    // [20, 50] pulls in to 50 at most (rigid joint, no spring).
    {
        World world(vec2(0.0f, 0.0f));
        Body* anchor = world.CreateBody(BodyType::Static, vec2(0.0f, 0.0f));
        Body* bob = world.CreateCircle(vec2(0.0f, 100.0f), 5.0f, 1.0f);
        DistanceJoint* joint = new DistanceJoint(anchor, bob, vec2(0.0f, 0.0f), vec2(0.0f, 100.0f));
        joint->SetSpring(2.0f, 1.0f);
        joint->SetLengthRange(20.0f, 50.0f);
        world.AddJoint(joint);
        for (int i = 0; i < 240; ++i)
            world.Step(1.0f / 60.0f);
        CHECK(Distance(anchor, bob) <= 51.0f);
        CHECK(Distance(anchor, bob) >= 19.0f);
    }

    {
        World world(vec2(0.0f, 0.0f));
        TileMapCollider map(world);
        map.SetMapSize(4, 3);
        map.SetSolid(1, 0, true);
        map.SetSolid(3, 2, true);
        map.SetSolid(0, 1, true);
        map.SetMapSize(6, 4);
        CHECK(map.IsSolid(1, 0));
        CHECK(map.IsSolid(3, 2));
        CHECK(map.IsSolid(0, 1));
        int solid = 0;
        for (int y = 0; y < 4; ++y)
            for (int x = 0; x < 6; ++x)
                solid += map.IsSolid(x, y) ? 1 : 0;
        CHECK(solid == 3);
        map.SetMapSize(2, 2);
        CHECK(map.IsSolid(1, 0));
        CHECK(map.IsSolid(0, 1));
        CHECK(!map.IsSolid(1, 1));
    }

    {
        const int size = 64;
        unsigned char pixels[size * size];
        memset(pixels, 0, sizeof(pixels));
        for (int y = 0; y < size; ++y)
            for (int x = 0; x < size; ++x)
            {
                const float dx = float(x) + 0.5f - size * 0.5f;
                const float dy = float(y) + 0.5f - size * 0.5f;
                const float r = sqrtf(dx * dx + dy * dy);
                if (r < 28.0f && r > 14.0f)
                    pixels[y * size + x] = 255;
            }
        World world(vec2(0.0f, 0.0f));
        Body* ring = world.CreateFromImage(vec2(0.0f, 0.0f), pixels, size, size, 1, 128, 1.0f, 1.0f, 2.0f);
        CHECK(ring != nullptr);
        if (ring)
        {
            CHECK(world.BodyAtPoint(vec2(0.0f, 0.0f)) == nullptr);
            CHECK(world.BodyAtPoint(vec2(5.0f, 3.0f)) == nullptr);
            CHECK(world.BodyAtPoint(vec2(21.0f, 0.0f)) == ring);
            CHECK(world.BodyAtPoint(vec2(0.0f, -21.0f)) == ring);
            float area = 0.0f;
            for (int s = 0; s < ring->ShapeCount(); ++s)
            {
                const Polygon& polygon = ring->Shapes()[s].polygon;
                for (int i = 0; i < polygon.count; ++i)
                    area += 0.5f * Cross(polygon.vertices[i], polygon.vertices[(i + 1) % polygon.count]);
            }
            const float ringArea = kPi * (28.0f * 28.0f - 14.0f * 14.0f);
            CHECK_NEAR(area, ringArea, 0.15f * ringArea);
        }
    }

    {
        kinetix::Color over = kinetix::Color::fromHSV(0.0f, 1.0f, 2.0f, 3.0f);
        CHECK(over.r() == 255);
        CHECK(over.a() == 255);
        kinetix::Color under = kinetix::Color::fromHSV(0.5f, -1.0f, -0.5f, -1.0f);
        CHECK(under.r() == 0 && under.g() == 0 && under.b() == 0 && under.a() == 0);
        kinetix::Color nan = kinetix::Color::fromHSV(NAN, NAN, NAN, NAN);
        CHECK(nan.r() == 0 && nan.a() == 0);
        kinetix::Color red = kinetix::Color::fromHSV(0.0f, 1.0f, 1.0f, 1.0f);
        CHECK(red.r() == 255 && red.g() == 0 && red.b() == 0 && red.a() == 255);
    }

    return TEST_RESULT();
}
