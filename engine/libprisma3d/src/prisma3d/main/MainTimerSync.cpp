#include "MainTimerSync.h"

#include <cmath>

namespace prisma3d
{

namespace
{

template<typename T>
T maxOf(T a, T b)
{
    return a > b ? a : b;
}

} // namespace

void MainFrameTime::clampProcessStep(double minProcessStep, double maxProcessStep)
{
    if (processStep < minProcessStep)
    {
        processStep = minProcessStep;
    }
    else if (processStep > maxProcessStep)
    {
        processStep = maxProcessStep;
    }
}

void MainTimerSync::DeltaSmoother::setAllowed(bool allowed)
{
    mAllowed = allowed;
}

void MainTimerSync::DeltaSmoother::madeNewEstimate()
{
    mHitsAboveEstimated = 0;
    mHitsAtEstimated = 0;
    mHitsBelowEstimated = 0;
    mHitsOneAboveEstimated = 0;
    mHitsOneBelowEstimated = 0;

    mEstimateComplete = false;
}

void MainTimerSync::DeltaSmoother::updateRefreshRateEstimator(std::int64_t delta)
{
    if (mEstimateLocked)
    {
        return;
    }

    mEstimatorTotalDelta += delta;
    mEstimatorDeltaReadings++;

    const int numReadings = 60;

    if (mEstimatorDeltaReadings < numReadings)
    {
        return;
    }

    delta = mEstimatorTotalDelta / numReadings;

    mEstimatorDeltaReadings = 0;
    mEstimatorTotalDelta = 0;

    int fps = static_cast<int>(std::round(1000000.0 / static_cast<double>(delta)));

    if (mEstimatedFps == 0)
    {
        if (fps >= 50)
        {
            mEstimatedFps = fps;
        }
        else
        {
            return;
        }
    }

    if (fps == mEstimatedFps)
    {
        mHitsAtEstimated++;

        if (mEstimateComplete && mHitsAtEstimated == 20)
        {
            mEstimateLocked = true;
            return;
        }

        if ((!mEstimateComplete) && (mHitsAtEstimated > 2))
        {
            if (mEstimatedFps)
            {
                mEstimateComplete = true;
                mVsyncDelta = 1000000 / mEstimatedFps;
            }
        }

        return;
    }

    const int significanceUp = 1;
    const int significanceDown = 2;

    if (fps < mEstimatedFps)
    {
        if (fps == (mEstimatedFps - 1))
        {
            mHitsOneBelowEstimated++;

            if ((mHitsOneBelowEstimated > mHitsAtEstimated) &&
                    (mHitsOneBelowEstimated > significanceDown))
            {
                mEstimatedFps--;
                madeNewEstimate();
            }

            return;
        }
        else
        {
            mHitsBelowEstimated++;

            bool established = mEstimateComplete && (mHitsAtEstimated > 10);

            if (!established)
            {
                if (((mHitsBelowEstimated / 8) > mHitsAtEstimated) &&
                        (mHitsBelowEstimated > significanceDown))
                {
                    mEstimatedFps--;
                    madeNewEstimate();
                }
            }

            return;
        }
    }

    if (fps == (mEstimatedFps + 1))
    {
        mHitsOneAboveEstimated++;

        if ((mHitsOneAboveEstimated > mHitsAtEstimated) &&
                (mHitsOneAboveEstimated > significanceUp))
        {
            mEstimatedFps++;
            madeNewEstimate();
        }
        return;
    }
    else
    {
        mHitsAboveEstimated++;

        if ((mHitsAboveEstimated > mHitsAtEstimated) && (mHitsAboveEstimated > significanceUp))
        {
            int change = fps - mEstimatedFps;
            change /= 2;
            change = maxOf(1, change);

            mEstimatedFps += change;
            madeNewEstimate();
        }
        return;
    }
}

bool MainTimerSync::DeltaSmoother::fpsAllowsSmoothing(std::int64_t delta)
{
    mMeasurementTime += delta;
    mMeasurementFrameCount++;

    if (mMeasurementFrameCount == mMeasurementEndFrame)
    {
        if (mEstimateComplete)
        {
            std::int64_t timePassed = mMeasurementTime - mMeasurementStartTime;

            timePassed /= kMeasureFpsOverNumFrames;

            if (timePassed)
            {
                double fps = 1000000.0 / static_cast<double>(timePassed);
                double ratio = fps / static_cast<double>(mEstimatedFps);

                if ((ratio > 0.95) && (ratio < 1.05))
                {
                    mMeasurementAllowsSmoothing = true;
                }
                else
                {
                    mMeasurementAllowsSmoothing = false;
                }
            }
        }

        mMeasurementStartTime = mMeasurementTime;
        mMeasurementEndFrame += kMeasureFpsOverNumFrames;
    }

    return mMeasurementAllowsSmoothing;
}

std::int64_t MainTimerSync::DeltaSmoother::smoothDelta(std::int64_t delta)
{
    if (!mAllowed)
    {
        return delta;
    }

    if (delta > 1000000)
    {
        return delta;
    }

    if (!fpsAllowsSmoothing(delta))
    {
        return delta;
    }

    if (delta < 1000)
    {
        return delta;
    }

    updateRefreshRateEstimator(delta);

    if (!mEstimateComplete)
    {
        return delta;
    }

    mLeftoverTime += delta;

    std::int64_t units = mLeftoverTime / mVsyncDelta;

    units = maxOf<std::int64_t>(units, 1);

    mLeftoverTime -= units * mVsyncDelta;

    return units * mVsyncDelta;
}

double MainTimerSync::getPhysicsJitterFix()
{
    return Engine::singleton()->getPhysicsJitterFix();
}

int MainTimerSync::getAveragePhysicsSteps(double& minSteps, double& maxSteps)
{
    minSteps = mTypicalPhysicsSteps[0];
    maxSteps = minSteps + 1;

    for (int i = 1; i < kControlSteps; ++i)
    {
        const double typicalLower = mTypicalPhysicsSteps[i];
        const double currentMin = typicalLower / (i + 1);
        if (currentMin > maxSteps)
        {
            return i;
        }
        else if (currentMin > minSteps)
        {
            minSteps = currentMin;
        }
        const double currentMax = (typicalLower + 1) / (i + 1);
        if (currentMax < minSteps)
        {
            return i;
        }
        else if (currentMax < maxSteps)
        {
            maxSteps = currentMax;
        }
    }

    return kControlSteps;
}

MainFrameTime MainTimerSync::advanceCore(double physicsStep, int physicsTicksPerSecond,
        double processStep)
{
    MainFrameTime ret;

    ret.processStep = processStep;

    mTimeAccum += ret.processStep;
    ret.physicsSteps = static_cast<int>(std::floor(mTimeAccum * physicsTicksPerSecond));

    int minTypicalSteps = mTypicalPhysicsSteps[0];
    int maxTypicalSteps = minTypicalSteps + 1;

    bool updateTypical = false;

    for (int i = 0; i < kControlSteps - 1; ++i)
    {
        int stepsLeftToMatchTypical = mTypicalPhysicsSteps[i + 1] - mAccumulatedPhysicsSteps[i];
        if (stepsLeftToMatchTypical > maxTypicalSteps ||
                stepsLeftToMatchTypical + 1 < minTypicalSteps)
        {
            updateTypical = true;
            break;
        }

        if (stepsLeftToMatchTypical > minTypicalSteps)
        {
            minTypicalSteps = stepsLeftToMatchTypical;
        }
        if (stepsLeftToMatchTypical + 1 < maxTypicalSteps)
        {
            maxTypicalSteps = stepsLeftToMatchTypical + 1;
        }
    }

    if (ret.physicsSteps < minTypicalSteps)
    {
        const int maxPossibleSteps = static_cast<int>(
                std::floor((mTimeAccum) *physicsTicksPerSecond + getPhysicsJitterFix()));
        if (maxPossibleSteps < minTypicalSteps)
        {
            ret.physicsSteps = maxPossibleSteps;
            updateTypical = true;
        }
        else
        {
            ret.physicsSteps = minTypicalSteps;
        }
    }
    else if (ret.physicsSteps > maxTypicalSteps)
    {
        const int minPossibleSteps = static_cast<int>(
                std::floor((mTimeAccum) *physicsTicksPerSecond - getPhysicsJitterFix()));
        if (minPossibleSteps > maxTypicalSteps)
        {
            ret.physicsSteps = minPossibleSteps;
            updateTypical = true;
        }
        else
        {
            ret.physicsSteps = maxTypicalSteps;
        }
    }

    if (ret.physicsSteps < 0)
    {
        ret.physicsSteps = 0;
    }

    mTimeAccum -= ret.physicsSteps * physicsStep;

    for (int i = kControlSteps - 2; i >= 0; --i)
    {
        mAccumulatedPhysicsSteps[i + 1] = mAccumulatedPhysicsSteps[i] + ret.physicsSteps;
    }
    mAccumulatedPhysicsSteps[0] = ret.physicsSteps;

    if (updateTypical)
    {
        for (int i = kControlSteps - 1; i >= 0; --i)
        {
            if (mTypicalPhysicsSteps[i] > mAccumulatedPhysicsSteps[i])
            {
                mTypicalPhysicsSteps[i] = mAccumulatedPhysicsSteps[i];
            }
            else if (mTypicalPhysicsSteps[i] < mAccumulatedPhysicsSteps[i] - 1)
            {
                mTypicalPhysicsSteps[i] = mAccumulatedPhysicsSteps[i] - 1;
            }
        }
    }

    return ret;
}

MainFrameTime MainTimerSync::advanceChecked(double physicsStep, int physicsTicksPerSecond,
        double processStep)
{
    if (mFixedFps != -1)
    {
        processStep = 1.0 / mFixedFps;
    }

    float minOutputStep = static_cast<float>(processStep / 8);
    minOutputStep = maxOf(minOutputStep, 1E-6f);

    processStep += mTimeDeficit;

    MainFrameTime ret = advanceCore(physicsStep, physicsTicksPerSecond, processStep);

    const double processMinusAccum = ret.processStep - mTimeAccum;

    {
        double minAveragePhysicsSteps, maxAveragePhysicsSteps;
        int consistentSteps =
                getAveragePhysicsSteps(minAveragePhysicsSteps, maxAveragePhysicsSteps);
        if (consistentSteps > 3)
        {
            ret.clampProcessStep(minAveragePhysicsSteps * physicsStep,
                    maxAveragePhysicsSteps * physicsStep);
        }
    }

    double maxClockDeviation = getPhysicsJitterFix() * physicsStep;
    ret.clampProcessStep(processStep - maxClockDeviation, processStep + maxClockDeviation);

    ret.clampProcessStep(processMinusAccum, processMinusAccum + physicsStep);

    if (ret.processStep < minOutputStep)
    {
        ret.processStep = minOutputStep;
    }

    mTimeAccum = ret.processStep - processMinusAccum;

    if (mTimeAccum > physicsStep)
    {
        const int extraPhysicsSteps =
                static_cast<int>(std::floor(mTimeAccum * physicsTicksPerSecond));
        mTimeAccum -= extraPhysicsSteps * physicsStep;
        ret.physicsSteps += extraPhysicsSteps;
    }

    mTimeDeficit = processStep - ret.processStep;

    ret.interpolationFraction = mTimeAccum / physicsStep;

    return ret;
}

double MainTimerSync::getCpuProcessStep()
{
    std::uint64_t cpuTicksElapsed = mCurrentCpuTicksUsec - mLastCpuTicksUsec;
    mLastCpuTicksUsec = mCurrentCpuTicksUsec;

    cpuTicksElapsed = static_cast<std::uint64_t>(
            mDeltaSmoother.smoothDelta(static_cast<std::int64_t>(cpuTicksElapsed)));

    return cpuTicksElapsed / 1000000.0;
}

MainTimerSync::MainTimerSync()
{
    for (int i = kControlSteps - 1; i >= 0; --i)
    {
        mTypicalPhysicsSteps[i] = i;
        mAccumulatedPhysicsSteps[i] = i;
    }
}

void MainTimerSync::init(std::uint64_t cpuTicksUsec)
{
    mCurrentCpuTicksUsec = mLastCpuTicksUsec = cpuTicksUsec;
}

void MainTimerSync::setCpuTicksUsec(std::uint64_t cpuTicksUsec)
{
    mCurrentCpuTicksUsec = cpuTicksUsec;
}

void MainTimerSync::setFixedFps(int fixedFps)
{
    mFixedFps = fixedFps;
}

void MainTimerSync::setDeltaSmoothing(bool enabled)
{
    mDeltaSmoother.setAllowed(enabled);
}

MainFrameTime MainTimerSync::advance(double physicsStep, int physicsTicksPerSecond)
{
    double cpuProcessStep = getCpuProcessStep();

    return advanceChecked(physicsStep, physicsTicksPerSecond, cpuProcessStep);
}

} // namespace prisma3d
