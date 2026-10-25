#include "check.h"

#include <kx/joints/distancejoint.h>
#include <kx/world.h>

using namespace kx;

int main()
{
    {
        World world(vec2(0.0f, 0.0f));
        Body* wall = world.CreateStaticBox(vec2(50.0f, 0.0f), 2.0f, 100.0f);
        wall->SetFilter(0x0002, 0xFFFF);
        Body* bullet = world.CreateCircle(vec2(0.0f, 0.0f), 2.0f, 1.0f);
        bullet->SetFilter(0x0001, 0xFFFF & ~0x0002);
        bullet->SetBullet(true);
        bullet->SetVelocity(vec2(6000.0f, 0.0f));
        for (int i = 0; i < 3; ++i)
            world.Step(1.0f / 60.0f);
        CHECK_NEAR(bullet->Position().x, 300.0f, 1.0f);
    }

    {
        World world(vec2(0.0f, 0.0f));
        world.CreateStaticBox(vec2(50.0f, 0.0f), 2.0f, 100.0f);
        Body* bullet = world.CreateCircle(vec2(0.0f, 0.0f), 2.0f, 1.0f);
        bullet->SetBullet(true);
        bullet->SetVelocity(vec2(6000.0f, 0.0f));
        for (int i = 0; i < 3; ++i)
            world.Step(1.0f / 60.0f);
        CHECK(bullet->Position().x < 48.0f);
        CHECK(bullet->Velocity().x < 1.0f);
    }

    {
        World world(vec2(0.0f, 0.0f));
        Body* wall = world.CreateStaticBox(vec2(50.0f, 0.0f), 2.0f, 100.0f);
        Body* bullet = world.CreateCircle(vec2(0.0f, 0.0f), 2.0f, 1.0f);
        bullet->SetBullet(true);
        DistanceJoint* rope = new DistanceJoint(wall, bullet, vec2(50.0f, 100.0f), vec2(0.0f, 0.0f));
        rope->SetLengthRange(0.0f, 1000.0f);
        world.AddJoint(rope);
        bullet->SetVelocity(vec2(6000.0f, 0.0f));
        world.Step(1.0f / 60.0f);
        CHECK(bullet->Position().x > 90.0f);
    }

    // One-sided chain: (50,100) -> (50,-100) has its solid side at x < 50.
    {
        World world(vec2(0.0f, 0.0f));
        vec2 points[2] = {vec2(50.0f, 100.0f), vec2(50.0f, -100.0f)};
        world.CreateChain(points, 2, false);
        Body* bullet = world.CreateCircle(vec2(80.0f, 0.0f), 2.0f, 1.0f);
        bullet->SetBullet(true);
        bullet->SetVelocity(vec2(-3000.0f, 0.0f));
        for (int i = 0; i < 3; ++i)
            world.Step(1.0f / 60.0f);
        CHECK_NEAR(bullet->Position().x, -70.0f, 1.0f);

        RayCastHit hit;
        CHECK(!world.RayCastClosest(vec2(80.0f, 0.0f), vec2(-100.0f, 0.0f), hit));
        ct::Vector<RayCastHit> hits;
        world.RayCastAll(vec2(80.0f, 0.0f), vec2(-100.0f, 0.0f), hits);
        CHECK(hits.empty());

        CHECK(world.RayCastClosest(vec2(20.0f, 0.0f), vec2(100.0f, 0.0f), hit));
        CHECK_NEAR(hit.point.x, 50.0f, 0.01f);
        CHECK_NEAR(hit.normal.x, -1.0f, 1e-4f);
    }

    {
        World world(vec2(0.0f, 0.0f));
        world.CreateEdge(vec2(50.0f, 100.0f), vec2(50.0f, -100.0f));
        RayCastHit hit;
        CHECK(world.RayCastClosest(vec2(80.0f, 0.0f), vec2(-100.0f, 0.0f), hit));
        CHECK(world.RayCastClosest(vec2(20.0f, 0.0f), vec2(100.0f, 0.0f), hit));
    }

    return TEST_RESULT();
}
