#include "prisma3d/main/Engine.h"

#include "MainTimerSync.h"
#include "prisma3d/Device.h"
#include "prisma3d/main/MainLoop.h"
#include "prisma3d/render/RenderProfiler.h"

#include "platform.h"

namespace prisma3d
{

struct Engine::TimerSync
{
    MainTimerSync sync;
};

Engine* Engine::sSingleton = nullptr;

namespace
{

std::uint64_t currentTicksUsec()
{
    Device* device = Device::singleton();
    if (device)
    {
        return device->getTicksUsec();
    }
    return time_nanos() / 1000;
}

void sleepUsec(std::uint32_t usec)
{
    Device* device = Device::singleton();
    if (device)
    {
        device->delayUsec(usec);
        return;
    }
    time_sleep((usec + 999) / 1000);
}

} // namespace

Engine::Engine()
        : mTimerSync(new TimerSync()),
          mMainLoop(nullptr),
          mDraw(nullptr),
          mDrawUser(nullptr),
          mPhysicsTicksPerSecond(60),
          mMaxPhysicsStepsPerFrame(8),
          mPhysicsJitterFix(0.5),
          mMaxFps(0),
          mFrameDelay(0),
          mTimeScale(1.0),
          mFixedFps(-1),
          mLowProcessorUsageMode(false),
          mLowProcessorUsageModeSleepUsec(6900),
          mEditorHint(false),
          mFramesDrawn(0),
          mPhysicsFrames(0),
          mProcessFrames(0),
          mInPhysics(false),
          mProcessStep(0.0),
          mPhysicsInterpolationFraction(0.0),
          mFps(0.0),
          mFrameTicks(0),
          mLastTicks(0),
          mTargetTicks(0),
          mFpsLastTicks(0),
          mFpsFrames(0),
          mForceRedraw(false),
          mStarted(false)
{
    sSingleton = this;
}

Engine::~Engine()
{
    if (sSingleton == this)
    {
        sSingleton = nullptr;
    }
    delete mTimerSync;
}

Engine* Engine::singleton()
{
    return sSingleton;
}

void Engine::setPhysicsTicksPerSecond(int ticks)
{
    if (ticks <= 0)
    {
        return;
    }
    mPhysicsTicksPerSecond = ticks;
}

int Engine::getPhysicsTicksPerSecond() const
{
    return mPhysicsTicksPerSecond;
}

void Engine::setMaxPhysicsStepsPerFrame(int steps)
{
    if (steps <= 0)
    {
        return;
    }
    mMaxPhysicsStepsPerFrame = steps;
}

int Engine::getMaxPhysicsStepsPerFrame() const
{
    return mMaxPhysicsStepsPerFrame;
}

void Engine::setPhysicsJitterFix(double threshold)
{
    if (threshold < 0)
    {
        threshold = 0;
    }
    mPhysicsJitterFix = threshold;
}

double Engine::getPhysicsJitterFix() const
{
    return mPhysicsJitterFix;
}

void Engine::setMaxFps(int fps)
{
    mMaxFps = fps >= 0 ? fps : 0;
}

int Engine::getMaxFps() const
{
    return mMaxFps;
}

void Engine::setFrameDelay(std::uint32_t msec)
{
    mFrameDelay = msec;
}

std::uint32_t Engine::getFrameDelay() const
{
    return mFrameDelay;
}

void Engine::setTimeScale(double scale)
{
    mTimeScale = scale;
}

double Engine::getTimeScale() const
{
    return mTimeScale;
}

void Engine::setFixedFps(int fps)
{
    mFixedFps = fps;
}

int Engine::getFixedFps() const
{
    return mFixedFps;
}

void Engine::setLowProcessorUsageMode(bool enabled)
{
    mLowProcessorUsageMode = enabled;
}

bool Engine::isInLowProcessorUsageMode() const
{
    return mLowProcessorUsageMode;
}

void Engine::setEditorHint(bool enabled)
{
    mEditorHint = enabled;
}

bool Engine::isEditorHint() const
{
    return mEditorHint;
}

void Engine::setLowProcessorUsageModeSleepUsec(int usec)
{
    mLowProcessorUsageModeSleepUsec = usec;
}

int Engine::getLowProcessorUsageModeSleepUsec() const
{
    return mLowProcessorUsageModeSleepUsec;
}

std::uint64_t Engine::getFramesDrawn() const
{
    return mFramesDrawn;
}

std::uint64_t Engine::getPhysicsFrames() const
{
    return mPhysicsFrames;
}

std::uint64_t Engine::getProcessFrames() const
{
    return mProcessFrames;
}

bool Engine::isInPhysicsFrame() const
{
    return mInPhysics;
}

double Engine::getProcessStep() const
{
    return mProcessStep;
}

double Engine::getPhysicsInterpolationFraction() const
{
    return mPhysicsInterpolationFraction;
}

double Engine::getFramesPerSecond() const
{
    return mFps;
}

void Engine::setMainLoop(MainLoop* loop)
{
    mMainLoop = loop;
}

MainLoop* Engine::getMainLoop() const
{
    return mMainLoop;
}

void Engine::setDrawFunction(DrawFunction function, void* user)
{
    mDraw = function;
    mDrawUser = user;
}

void Engine::forceRedraw()
{
    mForceRedraw = true;
}

void Engine::start(std::uint64_t ticksUsec)
{
    mTimerSync->sync.init(ticksUsec);
    mLastTicks = ticksUsec;
    mFpsLastTicks = ticksUsec;
    mStarted = true;
    if (mMainLoop)
    {
        mMainLoop->initialize();
    }
}

bool Engine::iteration(std::uint64_t ticksUsec)
{
    const std::uint64_t ticks = ticksUsec;
    mTimerSync->sync.setCpuTicksUsec(ticks);
    mTimerSync->sync.setFixedFps(mFixedFps);

    const std::uint64_t ticksElapsed = ticks - mLastTicks;

    const int physicsTicksPerSecond = mPhysicsTicksPerSecond;
    const double physicsStep = 1.0 / physicsTicksPerSecond;

    const double timeScale = mTimeScale;

    MainFrameTime advance = mTimerSync->sync.advance(physicsStep, physicsTicksPerSecond);
    double processStep = advance.processStep;
    double scaledStep = processStep * timeScale;

    mProcessStep = processStep;
    mPhysicsInterpolationFraction = advance.interpolationFraction;

    mFrameTicks += ticksElapsed;

    mLastTicks = ticks;

    const int maxPhysicsSteps = mMaxPhysicsStepsPerFrame;
    if (mFixedFps == -1 && advance.physicsSteps > maxPhysicsSteps)
    {
        processStep -= (advance.physicsSteps - maxPhysicsSteps) * physicsStep;
        advance.physicsSteps = maxPhysicsSteps;
        mProcessStep = processStep;
    }

    bool exit = false;

    {
        ProfileScope physicsScope("Physics");
        for (int iters = 0; iters < advance.physicsSteps; ++iters)
        {
            mInPhysics = true;
            mPhysicsFrames++;

            if (mMainLoop)
            {
                mMainLoop->iterationPrepare();

                if (mMainLoop->physicsProcess(physicsStep * timeScale))
                {
                    mInPhysics = false;
                    exit = true;
                    break;
                }

                mMainLoop->iterationEnd();
            }

            mInPhysics = false;
        }
    }

    {
        ProfileScope processScope("Process");
        if (mMainLoop && mMainLoop->process(processStep * timeScale))
        {
            exit = true;
        }
    }

    if (mDraw)
    {
        Device* device = Device::singleton();
        bool wantsPresent = device ? device->windowCanDraw() : true;

        wantsPresent |= mForceRedraw;
        if (mForceRedraw || !mLowProcessorUsageMode)
        {
            {
                ProfileScope drawScope("Draw");
                mDraw(wantsPresent, scaledStep, mDrawUser);
            }
            mFramesDrawn++;
            mForceRedraw = false;
        }
    }

    mFpsFrames++;
    mProcessFrames++;

    if (mFrameTicks > 1000000)
    {
        mFps = mFpsFrames;
        mFrameTicks %= 1000000;
        mFpsFrames = 0;
    }

    return exit;
}

bool Engine::iteration()
{
    const bool exit = iteration(currentTicksUsec());

    if (mFixedFps != -1)
    {
        return exit;
    }

    Device* device = Device::singleton();
    addFrameDelay(device ? device->windowCanDraw() : true, currentTicksUsec(), sleepUsec);

    return exit;
}

void Engine::addFrameDelay(bool canDraw, std::uint64_t ticksUsec, void (*delayUsec)(std::uint32_t))
{
    std::uint64_t currentTicks = ticksUsec;

    const std::uint32_t frameDelay = mFrameDelay;
    if (frameDelay)
    {
        delayUsec(frameDelay * 1000);
        currentTicks += static_cast<std::uint64_t>(frameDelay) * 1000;
    }

    std::uint64_t dynamicDelay = 0;
    if (mLowProcessorUsageMode || !canDraw)
    {
        dynamicDelay = static_cast<std::uint64_t>(mLowProcessorUsageModeSleepUsec);
    }
    const int maxFps = mMaxFps;
    if (maxFps > 0)
    {
        const std::uint64_t maxFpsDelay = static_cast<std::uint64_t>(1000000 / maxFps);
        dynamicDelay = dynamicDelay > maxFpsDelay ? dynamicDelay : maxFpsDelay;
    }

    if (dynamicDelay > 0)
    {
        mTargetTicks += dynamicDelay;

        if (currentTicks < mTargetTicks)
        {
            const std::uint64_t slept = mTargetTicks - currentTicks;
            delayUsec(static_cast<std::uint32_t>(slept));
            currentTicks += slept;
        }

        const std::uint64_t lower = currentTicks - dynamicDelay;
        const std::uint64_t upper = currentTicks + dynamicDelay;
        const std::uint64_t clamped = mTargetTicks > lower ? mTargetTicks : lower;
        mTargetTicks = clamped < upper ? clamped : upper;
    }
}

} // namespace prisma3d
