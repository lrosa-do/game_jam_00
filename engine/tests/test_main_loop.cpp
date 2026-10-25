#include "Check.h"

#include "prisma3d/main/Engine.h"
#include "prisma3d/main/MainLoop.h"

#include <math.h>

using namespace prisma3d;

namespace
{

const std::uint64_t kFrame = 16666;

class CountingLoop : public MainLoop
{
public:
    int initializeCalls = 0;
    int prepareCalls = 0;
    int physicsCalls = 0;
    int endCalls = 0;
    int processCalls = 0;
    double lastPhysicsDelta = 0.0;
    double lastProcessDelta = 0.0;
    double processSum = 0.0;
    bool physicsReturns = false;
    bool processReturns = false;
    bool inPhysicsSeen = true;
    Engine* engine = nullptr;

    void initialize() override
    {
        ++initializeCalls;
    }

    void iterationPrepare() override
    {
        ++prepareCalls;
    }

    bool physicsProcess(double time) override
    {
        ++physicsCalls;
        lastPhysicsDelta = time;
        if (engine && !engine->isInPhysicsFrame())
        {
            inPhysicsSeen = false;
        }
        return physicsReturns;
    }

    void iterationEnd() override
    {
        ++endCalls;
    }

    bool process(double time) override
    {
        ++processCalls;
        lastProcessDelta = time;
        processSum += time;
        return processReturns;
    }
};

struct DrawRecord
{
    int calls = 0;
    bool lastWantsPresent = false;
    double lastStep = 0.0;
};

void onDraw(bool wantsPresent, double scaledStep, void* user)
{
    DrawRecord* record = static_cast<DrawRecord*>(user);
    ++record->calls;
    record->lastWantsPresent = wantsPresent;
    record->lastStep = scaledStep;
}

int sleepCalls = 0;
std::uint32_t sleepRequests[8];

void onSleep(std::uint32_t usec)
{
    if (sleepCalls < 8)
    {
        sleepRequests[sleepCalls] = usec;
    }
    ++sleepCalls;
}

bool near(double a, double b, double tolerance)
{
    return fabs(a - b) <= tolerance;
}

void test_defaults()
{
    CHECK(Engine::singleton() == nullptr);
    {
        Engine engine;
        CHECK(Engine::singleton() == &engine);
        CHECK(engine.getPhysicsTicksPerSecond() == 60);
        CHECK(engine.getMaxPhysicsStepsPerFrame() == 8);
        CHECK(engine.getPhysicsJitterFix() == 0.5);
        CHECK(engine.getMaxFps() == 0);
        CHECK(engine.getFrameDelay() == 0);
        CHECK(engine.getTimeScale() == 1.0);
        CHECK(engine.getFixedFps() == -1);
        CHECK(!engine.isInLowProcessorUsageMode());
        CHECK(engine.getLowProcessorUsageModeSleepUsec() == 6900);
        CHECK(engine.getFramesDrawn() == 0);
        CHECK(engine.getPhysicsFrames() == 0);
        CHECK(engine.getProcessFrames() == 0);
        CHECK(!engine.isInPhysicsFrame());
        CHECK(engine.getProcessStep() == 0.0);
        CHECK(engine.getPhysicsInterpolationFraction() == 0.0);
        CHECK(engine.getFramesPerSecond() == 0.0);
        CHECK(engine.getMainLoop() == nullptr);
    }
    CHECK(Engine::singleton() == nullptr);
}

void test_start_and_steady()
{
    Engine engine;
    CountingLoop loop;
    loop.engine = &engine;
    engine.setMainLoop(&loop);
    CHECK(engine.getMainLoop() == &loop);
    engine.start(0);
    CHECK(loop.initializeCalls == 1);

    std::uint64_t ticks = 0;
    bool exit = false;
    for (int i = 0; i < 60; ++i)
    {
        ticks += kFrame;
        exit = engine.iteration(ticks) || exit;
        CHECK(!engine.isInPhysicsFrame());
        CHECK(engine.getPhysicsInterpolationFraction() >= 0.0);
        CHECK(engine.getPhysicsInterpolationFraction() <= 1.0);
    }
    CHECK(!exit);
    CHECK(loop.processCalls == 60);
    CHECK(near(loop.processSum, 1.0, 0.02));
    CHECK(loop.physicsCalls >= 58 && loop.physicsCalls <= 62);
    CHECK(loop.prepareCalls == loop.physicsCalls);
    CHECK(loop.endCalls == loop.physicsCalls);
    CHECK(engine.getPhysicsFrames() == static_cast<std::uint64_t>(loop.physicsCalls));
    CHECK(engine.getProcessFrames() == 60);
    CHECK(loop.inPhysicsSeen);
    CHECK(near(loop.lastPhysicsDelta, 1.0 / 60.0, 1e-9));
}

void test_huge_gap()
{
    Engine engine;
    CountingLoop loop;
    engine.setMainLoop(&loop);
    engine.start(0);
    std::uint64_t ticks = 0;
    for (int i = 0; i < 10; ++i)
    {
        ticks += kFrame;
        engine.iteration(ticks);
    }
    const int before = loop.physicsCalls;
    ticks += 5000000;
    engine.iteration(ticks);
    CHECK(loop.physicsCalls - before == engine.getMaxPhysicsStepsPerFrame());
    CHECK(engine.getProcessStep() > 0.0);
    CHECK(engine.getProcessStep() < 5.0);
    CHECK(near(loop.lastProcessDelta, engine.getProcessStep(), 1e-12));
}

void test_fixed_fps()
{
    Engine engineA;
    Engine engineB;
    CountingLoop loopA;
    CountingLoop loopB;
    engineA.setMainLoop(&loopA);
    engineA.setFixedFps(30);
    CHECK(engineA.getFixedFps() == 30);
    engineA.start(0);
    std::uint64_t ticksA = 0;
    const std::uint64_t feeds[] = { 1000, 50000, 16666, 3, 900000, 16666, 7000, 120000 };
    const int count = 8;
    double sumA = 0.0;
    int physicsA = 0;
    for (int i = 0; i < 90; ++i)
    {
        ticksA += feeds[i % count];
        const int before = loopA.physicsCalls;
        engineA.iteration(ticksA);
        sumA += loopA.lastProcessDelta;
        physicsA += loopA.physicsCalls - before;
        if (i >= 12)
        {
            CHECK(loopA.physicsCalls - before == 2);
            CHECK(near(loopA.lastProcessDelta, 1.0 / 30.0, 1e-3));
        }
    }
    CHECK(loopA.processCalls == 90);
    CHECK(near(sumA, 3.0, 1.0 / 30.0));
    CHECK(physicsA >= 178 && physicsA <= 182);

    engineB.setMainLoop(&loopB);
    engineB.setFixedFps(30);
    engineB.start(0);
    std::uint64_t ticksB = 0;
    ticksA = 0;
    bool same = true;
    Engine engineC;
    CountingLoop loopC;
    engineC.setMainLoop(&loopC);
    engineC.setFixedFps(30);
    engineC.start(0);
    for (int i = 0; i < 90; ++i)
    {
        ticksB += kFrame;
        engineB.iteration(ticksB);
        ticksA += feeds[i % count];
        engineC.iteration(ticksA);
        same = same && loopB.lastProcessDelta == loopC.lastProcessDelta &&
               loopB.physicsCalls == loopC.physicsCalls;
    }
    CHECK(same);
}

void test_time_scale_and_physics_rate()
{
    {
        Engine engine;
        CountingLoop loop;
        engine.setMainLoop(&loop);
        engine.setTimeScale(2.0);
        CHECK(engine.getTimeScale() == 2.0);
        engine.start(0);
        std::uint64_t ticks = 0;
        for (int i = 0; i < 30; ++i)
        {
            ticks += kFrame;
            engine.iteration(ticks);
        }
        CHECK(near(loop.lastProcessDelta, engine.getProcessStep() * 2.0, 1e-12));
        CHECK(near(loop.lastPhysicsDelta, 2.0 / 60.0, 1e-9));
        CHECK(near(loop.processSum, 1.0, 0.02));
    }
    {
        Engine engine;
        CountingLoop loop;
        engine.setMainLoop(&loop);
        engine.setPhysicsTicksPerSecond(120);
        CHECK(engine.getPhysicsTicksPerSecond() == 120);
        engine.start(0);
        std::uint64_t ticks = 0;
        for (int i = 0; i < 60; ++i)
        {
            ticks += kFrame;
            engine.iteration(ticks);
        }
        CHECK(loop.physicsCalls >= 116 && loop.physicsCalls <= 124);
        CHECK(near(loop.lastPhysicsDelta, 1.0 / 120.0, 1e-9));
    }
}

void test_exit()
{
    {
        Engine engine;
        CountingLoop loop;
        engine.setMainLoop(&loop);
        engine.start(0);
        CHECK(!engine.iteration(kFrame));
        loop.processReturns = true;
        CHECK(engine.iteration(2 * kFrame));
    }
    {
        Engine engine;
        CountingLoop loop;
        engine.setMainLoop(&loop);
        engine.start(0);
        loop.physicsReturns = true;
        std::uint64_t ticks = 0;
        bool exit = false;
        for (int i = 0; i < 5 && !exit; ++i)
        {
            ticks += kFrame;
            exit = engine.iteration(ticks);
        }
        CHECK(exit);
        CHECK(!engine.isInPhysicsFrame());
        CHECK(loop.physicsCalls >= 1);
    }
}

void test_draw()
{
    Engine engine;
    CountingLoop loop;
    DrawRecord record;
    engine.setMainLoop(&loop);
    engine.setDrawFunction(onDraw, &record);
    engine.setTimeScale(0.5);
    engine.start(0);
    std::uint64_t ticks = 0;
    for (int i = 0; i < 5; ++i)
    {
        ticks += kFrame;
        engine.iteration(ticks);
        CHECK(record.calls == i + 1);
        CHECK(record.lastWantsPresent);
        CHECK(near(record.lastStep, engine.getProcessStep() * 0.5, 1e-12));
    }
    CHECK(engine.getFramesDrawn() == 5);

    engine.setLowProcessorUsageMode(true);
    CHECK(engine.isInLowProcessorUsageMode());
    for (int i = 0; i < 3; ++i)
    {
        ticks += kFrame;
        engine.iteration(ticks);
    }
    CHECK(record.calls == 5);
    CHECK(engine.getFramesDrawn() == 5);

    engine.forceRedraw();
    ticks += kFrame;
    engine.iteration(ticks);
    CHECK(record.calls == 6);
    CHECK(engine.getFramesDrawn() == 6);

    ticks += kFrame;
    engine.iteration(ticks);
    CHECK(record.calls == 6);
}

void test_frame_delay()
{
    {
        Engine engine;
        engine.setMaxFps(100);
        CHECK(engine.getMaxFps() == 100);
        sleepCalls = 0;
        engine.addFrameDelay(true, 0, onSleep);
        CHECK(sleepCalls == 1);
        CHECK(sleepRequests[0] == 10000);
        engine.addFrameDelay(true, 10000 + 1000, onSleep);
        CHECK(sleepCalls == 2);
        CHECK(sleepRequests[1] >= 8900 && sleepRequests[1] <= 9100);
    }
    {
        Engine engine;
        engine.setFrameDelay(5);
        CHECK(engine.getFrameDelay() == 5);
        sleepCalls = 0;
        engine.addFrameDelay(true, 0, onSleep);
        CHECK(sleepCalls == 1);
        CHECK(sleepRequests[0] == 5000);
    }
    {
        Engine engine;
        sleepCalls = 0;
        engine.addFrameDelay(true, 1000, onSleep);
        CHECK(sleepCalls == 0);
        engine.addFrameDelay(false, 0, onSleep);
        CHECK(sleepCalls == 1);
        CHECK(sleepRequests[0] == 6900);
    }
}

void test_fps()
{
    Engine engine;
    CountingLoop loop;
    engine.setMainLoop(&loop);
    engine.start(0);
    CHECK(engine.getFramesPerSecond() == 0.0);
    std::uint64_t ticks = 0;
    for (int i = 0; i < 70; ++i)
    {
        ticks += kFrame;
        engine.iteration(ticks);
    }
    CHECK(near(engine.getFramesPerSecond(), 60.0, 2.0));
}

} // namespace

int main()
{
    test_defaults();
    test_start_and_steady();
    test_huge_gap();
    test_fixed_fps();
    test_time_scale_and_physics_rate();
    test_exit();
    test_draw();
    test_frame_delay();
    test_fps();
    if (failures) printf("%d failure(s)\n", failures);
    return failures ? 1 : 0;
}
