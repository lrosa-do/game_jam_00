#pragma once

#include "prisma3d/render/Batch.h"
#include "prisma3d/render/RenderProfiler.h"
#include "prisma3d/render/RenderingServer.h"

#include <cstdint>

namespace prisma3d
{

class StatsOverlay
{
public:
    static constexpr float kAverageSeconds = 1.0f;
    static constexpr float kExtremesSeconds = 3.0f;
    static constexpr float kRefreshSeconds = 0.25f;

    StatsOverlay();

    void setVisible(bool visible);
    bool visible() const;
    void toggle();

    void frame(std::uint64_t ticksUsec);
    void draw(BatchRenderer& batch, const RenderStats& stats, const RenderProfiler* profiler,
            int width, int height);

    float fps() const;
    float minFps() const;
    float maxFps() const;
    float frameMilliseconds() const;

private:
    static constexpr std::uint32_t kCapacity = 4096;

    void refresh();

    bool mVisible;
    std::uint64_t mLastTicks;
    std::uint32_t mHead;
    std::uint32_t mCount;
    double mClock;
    float mSinceRefresh;
    float mFps;
    float mMinFps;
    float mMaxFps;
    float mFrameMilliseconds;
    std::uint32_t mBuildTotal;
    float mBuildMilliseconds;
    float mDelta[kCapacity];
    double mStamp[kCapacity];
};

} // namespace prisma3d
