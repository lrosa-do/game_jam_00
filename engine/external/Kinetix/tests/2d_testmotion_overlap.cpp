#include "check.h"

#include <kx/world.h>

using namespace kx;

int main()
{
    World world(vec2(0.0f, 0.0f));
    world.CreateStaticBox(vec2(0.0f, 0.0f), 100.0f, 10.0f); // top face at y = -10

    const float gaps[] = {5.0f, 1.0f, 0.5f, 0.0f, -1.0f};
    for (float gap : gaps)
    {
        Body* box = world.CreateKinematicBox(vec2(0.0f, -20.0f - gap), 10.0f, 10.0f);
        MotionResult result;
        CHECK(world.TestMotion(*box, vec2(0.0f, 50.0f), result));
        CHECK(result.hit);
        CHECK(result.travel.y <= (gap > 0.0f ? gap : 0.0f) + 1e-3f);
        CHECK(result.travel.y >= 0.0f);
        CHECK(result.normal.y < -0.9f);

        MotionResult placed;
        if (world.TestPosition(*box, box->Position(), placed))
        {
            CHECK_NEAR(result.travel.y, 0.0f, 1e-4f);
            CHECK_NEAR(result.remainder.y, 50.0f, 1e-3f);
            CHECK_NEAR(result.fraction, 0.0f, 1e-6f);
        }

        MotionResult up;
        CHECK(!world.TestMotion(*box, vec2(0.0f, -50.0f), up));
        CHECK_NEAR(up.travel.y, -50.0f, 1e-4f);
        MotionResult side;
        CHECK(!world.TestMotion(*box, vec2(30.0f, 0.0f), side));
        CHECK_NEAR(side.travel.x, 30.0f, 1e-4f);

        world.Destroy(box);
    }

    return TEST_RESULT();
}
