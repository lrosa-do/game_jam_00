#pragma once

#include <cstdint>

namespace prisma3d
{

class MainLoop;

struct MainFrameTime
{
    double processStep = 0.0;
    int physicsSteps = 0;
    double interpolationFraction = 0.0;

    void clampProcessStep(double minProcessStep, double maxProcessStep);
};

class Engine
{
public:
    typedef void (*DrawFunction)(bool wantsPresent, double scaledStep, void* user);

    Engine();
    ~Engine();
    Engine(const Engine&) = delete;
    Engine& operator=(const Engine&) = delete;

    static Engine* singleton();

    void setPhysicsTicksPerSecond(int ticks);
    int getPhysicsTicksPerSecond() const;
    void setMaxPhysicsStepsPerFrame(int steps);
    int getMaxPhysicsStepsPerFrame() const;
    void setPhysicsJitterFix(double threshold);
    double getPhysicsJitterFix() const;
    void setMaxFps(int fps);
    int getMaxFps() const;
    void setFrameDelay(std::uint32_t msec);
    std::uint32_t getFrameDelay() const;
    void setTimeScale(double scale);
    double getTimeScale() const;
    void setFixedFps(int fps);
    int getFixedFps() const;
    void setLowProcessorUsageMode(bool enabled);
    bool isInLowProcessorUsageMode() const;
    void setLowProcessorUsageModeSleepUsec(int usec);
    int getLowProcessorUsageModeSleepUsec() const;
    void setEditorHint(bool enabled);
    bool isEditorHint() const;

    std::uint64_t getFramesDrawn() const;
    std::uint64_t getPhysicsFrames() const;
    std::uint64_t getProcessFrames() const;
    bool isInPhysicsFrame() const;
    double getProcessStep() const;
    double getPhysicsInterpolationFraction() const;
    double getFramesPerSecond() const;

    void setMainLoop(MainLoop* loop);
    MainLoop* getMainLoop() const;
    void setDrawFunction(DrawFunction function, void* user);

    void start(std::uint64_t ticksUsec);
    bool iteration(std::uint64_t ticksUsec);
    bool iteration();
    void addFrameDelay(bool canDraw, std::uint64_t ticksUsec, void (*delayUsec)(std::uint32_t));

    void forceRedraw();

private:
    static Engine* sSingleton;

    struct TimerSync;

    TimerSync* mTimerSync;
    MainLoop* mMainLoop;
    DrawFunction mDraw;
    void* mDrawUser;

    int mPhysicsTicksPerSecond;
    int mMaxPhysicsStepsPerFrame;
    double mPhysicsJitterFix;
    int mMaxFps;
    std::uint32_t mFrameDelay;
    double mTimeScale;
    int mFixedFps;
    bool mLowProcessorUsageMode;
    int mLowProcessorUsageModeSleepUsec;
    bool mEditorHint;

    std::uint64_t mFramesDrawn;
    std::uint64_t mPhysicsFrames;
    std::uint64_t mProcessFrames;
    bool mInPhysics;
    double mProcessStep;
    double mPhysicsInterpolationFraction;
    double mFps;
    std::uint64_t mFrameTicks;
    std::uint64_t mLastTicks;
    std::uint64_t mTargetTicks;
    std::uint64_t mFpsLastTicks;
    std::uint32_t mFpsFrames;
    bool mForceRedraw;
    bool mStarted;
};

} // namespace prisma3d
