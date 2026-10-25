#pragma once

#include "prisma3d/main/Engine.h"

#include <cstdint>

namespace prisma3d
{

class MainTimerSync
{
    class DeltaSmoother
    {
    public:
        std::int64_t smoothDelta(std::int64_t delta);
        void setAllowed(bool allowed);

    private:
        void madeNewEstimate();
        void updateRefreshRateEstimator(std::int64_t delta);
        bool fpsAllowsSmoothing(std::int64_t delta);

        std::int64_t mVsyncDelta = 16666;
        std::int64_t mLeftoverTime = 0;

        static const int kMeasureFpsOverNumFrames = 64;

        std::int64_t mMeasurementTime = 0;
        std::int64_t mMeasurementFrameCount = 0;
        std::int64_t mMeasurementEndFrame = kMeasureFpsOverNumFrames;
        std::int64_t mMeasurementStartTime = 0;
        bool mMeasurementAllowsSmoothing = true;

        std::int32_t mEstimatedFps = 0;
        std::int32_t mHitsAtEstimated = 0;
        std::int32_t mHitsAboveEstimated = 0;
        std::int32_t mHitsBelowEstimated = 0;
        std::int32_t mHitsOneAboveEstimated = 0;
        std::int32_t mHitsOneBelowEstimated = 0;
        bool mEstimateComplete = false;
        bool mEstimateLocked = false;

        std::int64_t mEstimatorTotalDelta = 0;
        std::int32_t mEstimatorDeltaReadings = 0;
        bool mAllowed = false;
    } mDeltaSmoother;

    std::uint64_t mLastCpuTicksUsec = 0;
    std::uint64_t mCurrentCpuTicksUsec = 0;

    double mTimeAccum = 0;
    double mTimeDeficit = 0;

    static const int kControlSteps = 12;

    int mAccumulatedPhysicsSteps[kControlSteps];
    int mTypicalPhysicsSteps[kControlSteps];

    int mFixedFps = 0;

protected:
    double getPhysicsJitterFix();
    int getAveragePhysicsSteps(double& minSteps, double& maxSteps);
    MainFrameTime advanceCore(double physicsStep, int physicsTicksPerSecond, double processStep);
    MainFrameTime advanceChecked(double physicsStep, int physicsTicksPerSecond, double processStep);
    double getCpuProcessStep();

public:
    MainTimerSync();

    void init(std::uint64_t cpuTicksUsec);
    void setCpuTicksUsec(std::uint64_t cpuTicksUsec);
    void setFixedFps(int fixedFps);
    void setDeltaSmoothing(bool enabled);

    MainFrameTime advance(double physicsStep, int physicsTicksPerSecond);
};

} // namespace prisma3d
