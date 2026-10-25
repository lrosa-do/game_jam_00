#include "prisma3d/main/StatsOverlay.h"

#include <stdio.h>

namespace prisma3d
{

namespace
{

const float kTextSize = 14.0f;
const float kLineHeight = 18.0f;
const float kPadding = 8.0f;
const float kMargin = 8.0f;
const int kLines = 4;

} // namespace

StatsOverlay::StatsOverlay()
        : mVisible(false),
          mLastTicks(0),
          mHead(0),
          mCount(0),
          mClock(0.0),
          mSinceRefresh(0.0f),
          mFps(0.0f),
          mMinFps(0.0f),
          mMaxFps(0.0f),
          mFrameMilliseconds(0.0f),
          mBuildTotal(0),
          mBuildMilliseconds(0.0f)
{
}

void StatsOverlay::setVisible(bool visible)
{
    mVisible = visible;
}

bool StatsOverlay::visible() const
{
    return mVisible;
}

void StatsOverlay::toggle()
{
    mVisible = !mVisible;
}

void StatsOverlay::frame(std::uint64_t ticksUsec)
{
    if (mLastTicks == 0 || ticksUsec <= mLastTicks)
    {
        mLastTicks = ticksUsec;
        return;
    }
    const float delta = static_cast<float>(ticksUsec - mLastTicks) * 1e-6f;
    mLastTicks = ticksUsec;
    mClock += static_cast<double>(delta);

    if (mCount == kCapacity)
    {
        mHead = (mHead + 1) % kCapacity;
        --mCount;
    }
    const std::uint32_t slot = (mHead + mCount) % kCapacity;
    mDelta[slot] = delta;
    mStamp[slot] = mClock;
    ++mCount;

    while (mCount > 1 && mClock - mStamp[mHead] > static_cast<double>(kExtremesSeconds))
    {
        mHead = (mHead + 1) % kCapacity;
        --mCount;
    }

    mSinceRefresh += delta;
    if (mFps == 0.0f || mSinceRefresh >= kRefreshSeconds)
    {
        mSinceRefresh = 0.0f;
        refresh();
    }
}

void StatsOverlay::refresh()
{
    if (mCount == 0) return;
    float sum = 0.0f;
    std::uint32_t frames = 0;
    float shortest = mDelta[mHead];
    float longest = mDelta[mHead];
    for (std::uint32_t i = 0; i < mCount; ++i)
    {
        const std::uint32_t slot = (mHead + mCount - 1 - i) % kCapacity;
        const float delta = mDelta[slot];
        if (delta < shortest) shortest = delta;
        if (delta > longest) longest = delta;
        if (frames == 0 || mClock - mStamp[slot] < static_cast<double>(kAverageSeconds))
        {
            sum += delta;
            ++frames;
        }
    }
    mFps = sum > 0.0f ? static_cast<float>(frames) / sum : 0.0f;
    mFrameMilliseconds = frames > 0 ? sum * 1000.0f / static_cast<float>(frames) : 0.0f;
    mMinFps = longest > 0.0f ? 1.0f / longest : 0.0f;
    mMaxFps = shortest > 0.0f ? 1.0f / shortest : 0.0f;
}

float StatsOverlay::fps() const
{
    return mFps;
}

float StatsOverlay::minFps() const
{
    return mMinFps;
}

float StatsOverlay::maxFps() const
{
    return mMaxFps;
}

float StatsOverlay::frameMilliseconds() const
{
    return mFrameMilliseconds;
}

void StatsOverlay::draw(BatchRenderer& batch, const RenderStats& stats,
        const RenderProfiler* profiler, int width, int height)
{
    if (!mVisible || width <= 0 || height <= 0) return;

    char lines[kLines][112];
    snprintf(lines[0], sizeof(lines[0]), "FPS %.0f   min %.0f   max %.0f",
            static_cast<double>(mFps), static_cast<double>(mMinFps), static_cast<double>(mMaxFps));
    if (profiler && profiler->gpuAvailable())
        snprintf(lines[1], sizeof(lines[1]), "Frame %.2f ms   CPU %.2f ms   GPU %.2f ms",
                static_cast<double>(mFrameMilliseconds),
                static_cast<double>(profiler->cpuFrameMilliseconds()),
                static_cast<double>(profiler->gpuFrameMilliseconds()));
    else if (profiler)
        snprintf(lines[1], sizeof(lines[1]), "Frame %.2f ms   CPU %.2f ms",
                static_cast<double>(mFrameMilliseconds),
                static_cast<double>(profiler->cpuFrameMilliseconds()));
    else
        snprintf(lines[1], sizeof(lines[1]), "Frame %.2f ms",
                static_cast<double>(mFrameMilliseconds));
    snprintf(lines[2], sizeof(lines[2]), "Draws %u   Instances %u   Triangles %u   Batch %u",
            stats.drawCalls, stats.instancesDrawn, stats.triangles, stats.batchDrawCalls);
    if (stats.pipelinesBuilt > 0)
    {
        mBuildTotal += stats.pipelinesBuilt;
        mBuildMilliseconds += stats.pipelineBuildMilliseconds;
    }
    snprintf(lines[3], sizeof(lines[3]), "Pipelines built %u   %.0f ms total   %u this frame",
            mBuildTotal, mBuildMilliseconds, stats.pipelinesBuilt);

    float widest = 0.0f;
    for (int i = 0; i < kLines; ++i)
    {
        const float lineWidth = batch.textWidth(kTextSize, lines[i]);
        if (lineWidth > widest) widest = lineWidth;
    }
    const float panelWidth = widest + kPadding * 2.0f;
    const float panelHeight = kLineHeight * static_cast<float>(kLines) + kPadding * 2.0f - 4.0f;
    const float x = static_cast<float>(width) - panelWidth - kMargin;
    const float y = kMargin;

    batch.pushMatrix();
    batch.loadIdentity();
    batch.setSpace(BatchRenderer::Space::Screen);
    batch.setBlendMode(BatchRenderer::BlendMode::Alpha);
    batch.setDepthTest(false);
    batch.setDepthWrite(false);
    batch.setBlend(true);
    batch.setCullFace(false);
    batch.clearClipRect();

    batch.setColor(static_cast<unsigned char>(0), static_cast<unsigned char>(0),
            static_cast<unsigned char>(0), static_cast<unsigned char>(170));
    batch.drawRect(x, y, panelWidth, panelHeight);
    batch.setColor(static_cast<unsigned char>(255), static_cast<unsigned char>(255),
            static_cast<unsigned char>(255), static_cast<unsigned char>(255));
    for (int i = 0; i < kLines; ++i)
        batch.drawText(x + kPadding, y + kPadding + kLineHeight * static_cast<float>(i), kTextSize,
                lines[i]);

    batch.popMatrix();
    batch.setTexture(batch.whiteTexture());
    batch.setColor(static_cast<unsigned char>(255), static_cast<unsigned char>(255),
            static_cast<unsigned char>(255), static_cast<unsigned char>(255));
}

} // namespace prisma3d
