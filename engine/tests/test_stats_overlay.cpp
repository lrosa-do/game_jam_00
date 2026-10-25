#include "Check.h"

#include "prisma3d/main/StatsOverlay.h"

#include <math.h>

using namespace prisma3d;

namespace
{

bool near(float a, float b, float tolerance)
{
    return fabsf(a - b) <= tolerance;
}

struct Clock
{
    std::uint64_t ticks = 1000000;

    void advance(StatsOverlay& overlay, int frames, std::uint64_t micros)
    {
        for (int i = 0; i < frames; ++i)
        {
            overlay.frame(ticks);
            ticks += micros;
        }
    }
};

void testToggle()
{
    StatsOverlay overlay;
    CHECK(!overlay.visible());
    overlay.toggle();
    CHECK(overlay.visible());
    overlay.toggle();
    CHECK(!overlay.visible());
    overlay.setVisible(true);
    CHECK(overlay.visible());
}

void testSteady()
{
    StatsOverlay overlay;
    Clock clock;
    clock.advance(overlay, 120, 16667);
    CHECK(near(overlay.fps(), 60.0f, 0.5f));
    CHECK(near(overlay.minFps(), 60.0f, 0.5f));
    CHECK(near(overlay.maxFps(), 60.0f, 0.5f));
    CHECK(near(overlay.frameMilliseconds(), 16.667f, 0.1f));
}

void testSpikeAndRecovery()
{
    StatsOverlay overlay;
    Clock clock;
    clock.advance(overlay, 60, 16667);
    clock.advance(overlay, 1, 50000);
    clock.advance(overlay, 30, 16667);
    CHECK(near(overlay.minFps(), 20.0f, 0.5f));
    CHECK(near(overlay.maxFps(), 60.0f, 0.5f));

    clock.advance(overlay, 240, 16667);
    CHECK(near(overlay.minFps(), 60.0f, 0.5f));
    CHECK(near(overlay.fps(), 60.0f, 0.5f));
}

void testAverageIsRecent()
{
    StatsOverlay overlay;
    Clock clock;
    clock.advance(overlay, 180, 33333);
    CHECK(near(overlay.fps(), 30.0f, 0.5f));
    clock.advance(overlay, 240, 8333);
    CHECK(near(overlay.fps(), 120.0f, 1.0f));
    CHECK(near(overlay.maxFps(), 120.0f, 1.0f));
}

void testFastRateFitsTheWindow()
{
    StatsOverlay overlay;
    Clock clock;
    clock.advance(overlay, 20000, 100);
    CHECK(near(overlay.fps(), 10000.0f, 100.0f));
    CHECK(near(overlay.minFps(), 10000.0f, 100.0f));
}

} // namespace

int main()
{
    testToggle();
    testSteady();
    testSpikeAndRecovery();
    testAverageIsRecent();
    testFastRateFitsTheWindow();

    if (failures) printf("%d checks failed\n", failures);
    return failures ? 1 : 0;
}
