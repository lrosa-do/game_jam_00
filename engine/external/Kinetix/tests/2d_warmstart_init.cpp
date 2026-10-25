#include "check.h"

#include <kx/world.h>

#include <new>
#include <string.h>

using namespace kx;

int main()
{
    {
        alignas(Manifold) unsigned char storage[sizeof(Manifold)];
        memset(storage, 0xAB, sizeof(storage));
        Manifold* manifold = new (storage) Manifold;
        CHECK(manifold->pointCount == 0);
        for (int i = 0; i < kMaxManifoldPoints; ++i)
        {
            CHECK(manifold->points[i].normalImpulse == 0.0f);
            CHECK(manifold->points[i].tangentImpulse == 0.0f);
            CHECK(manifold->points[i].id.key == 0u);
        }
        manifold->~Manifold();
    }

    {
        World world(vec2(0.0f, 500.0f));
        world.CreateStaticBox(vec2(0.0f, 0.0f), 200.0f, 10.0f);
        Body* box = world.CreateBox(vec2(0.0f, -21.0f), 10.0f, 10.0f, 1.0f);
        for (int i = 0; i < 120; ++i)
            world.Step(1.0f / 60.0f);
        CHECK(!box->IsAwake());
        world.Step(1.0f / 60.0f);
        box->ApplyLinearImpulseToCenter(vec2(10.0f, 0.0f));
        world.Step(1.0f / 60.0f);
        const vec2 v = box->Velocity();
        CHECK(v.x == v.x && v.y == v.y);
        CHECK(fabsf(v.x) < 50.0f);
        CHECK(fabsf(v.y) < 50.0f);
        CHECK(box->Position().y > -25.0f && box->Position().y < -15.0f);
    }

    return TEST_RESULT();
}
