#pragma once

#include "prisma/rhi/Driver.h"

#include <cstdint>

namespace prisma3d
{

struct ProfileSample
{
    static constexpr std::uint32_t kHistorySize = 120;

    const char* name = nullptr;
    std::uint32_t depth = 0;
    float milliseconds = 0.0f;
    float display = 0.0f;
    float average = 0.0f;
    float maximum = 0.0f;
    float history[kHistorySize] = {};
    std::uint32_t historyCount = 0;
    std::uint32_t historyCursor = 0;
};

class RenderProfiler
{
public:
    static constexpr std::uint32_t kMaxSamples = 32;
    static constexpr std::uint32_t kMaxDepth = 16;
    static constexpr std::uint32_t kMaxGpuDepth = 8;
    static constexpr std::uint32_t kMaxGpuScopesPerFrame = 64;
    static constexpr std::uint32_t kFrameDelay = 3;
    static constexpr std::uint32_t kMaxPasses = 16;
    static constexpr double kRefreshSeconds = 0.25;

    explicit RenderProfiler(prisma::Driver* driver);
    ~RenderProfiler();
    RenderProfiler(const RenderProfiler&) = delete;
    RenderProfiler& operator=(const RenderProfiler&) = delete;

    static RenderProfiler* singleton();

    void setEnabled(bool enabled);
    bool enabled() const;
    void request();

    void setSpikeThreshold(float milliseconds);
    float spikeThreshold() const;

    void beginFrame();
    void endFrame();

    bool begin(const char* name);
    void end();
    void addSample(const char* name, float milliseconds);

    bool beginGpu(const char* name);
    void endGpu();

    const ProfileSample* cpuSamples() const;
    std::uint32_t cpuSampleCount() const;
    float cpuFrameMilliseconds() const;

    bool gpuAvailable() const;
    const ProfileSample* gpuSamples() const;
    std::uint32_t gpuSampleCount() const;
    float gpuFrameMilliseconds() const;

    float fps() const;
    std::uint64_t frameNumber() const;

    bool registerPass(const char* name);
    void setPassEnabled(const char* name, bool enabled);
    bool passEnabled(const char* name) const;
    std::uint32_t passCount() const;
    const char* passName(std::uint32_t index) const;

private:
    struct ActiveScope
    {
        std::uint32_t sample = 0;
        std::uint64_t startNanos = 0;
    };

    struct GpuScope
    {
        std::uint32_t sample = 0;
        std::uint32_t depth = 0;
        prisma::QueryHandle query;
        std::uint64_t nanos = 0;
        bool resolved = false;
    };

    struct GpuFrame
    {
        GpuScope scopes[kMaxGpuScopesPerFrame];
        std::uint32_t count = 0;
        bool pending = false;
    };

    struct Pass
    {
        const char* name = nullptr;
        bool enabled = true;
    };

    static RenderProfiler* sSingleton;

    static std::uint32_t findOrCreate(ProfileSample* samples, std::uint32_t& count,
            const char* name);
    static void pushHistory(ProfileSample* samples, std::uint32_t count, bool refresh);
    bool resolveGpu(GpuFrame& frame);
    void harvestGpu(GpuFrame& frame);
    int findPass(const char* name) const;
    void logSpike(const char* kind, std::uint64_t frame, float total, const ProfileSample* samples,
            std::uint32_t count) const;

    prisma::Driver* mDriver;
    bool mEnabled;
    std::uint64_t mRequestFrame;
    bool mGpuAvailable;
    float mSpikeThreshold;

    ProfileSample mCpuSamples[kMaxSamples];
    ActiveScope mCpuStack[kMaxDepth];
    std::uint32_t mCpuSampleCount;
    std::uint32_t mCpuDepth;
    std::uint64_t mFrameStartNanos;
    std::uint64_t mLastRefreshNanos;
    float mCpuFrameMilliseconds;
    float mCpuDisplayFrameMilliseconds;
    bool mOverflowWarned;
    bool mDepthOverflowWarned;

    ProfileSample mGpuSamples[kMaxSamples];
    GpuFrame mGpuFrames[kFrameDelay];
    std::uint32_t mGpuStack[kMaxGpuDepth];
    std::uint32_t mGpuSampleCount;
    std::uint32_t mGpuCursor;
    std::uint32_t mGpuDepth;
    bool mGpuRecording;
    std::uint64_t mGpuLastRefreshNanos;
    float mGpuFrameMilliseconds;
    float mGpuDisplayFrameMilliseconds;

    Pass mPasses[kMaxPasses];
    std::uint32_t mPassCount;

    std::uint64_t mFrameNumber;
    std::uint64_t mFpsWindowStartNanos;
    std::uint32_t mFpsWindowFrames;
    float mFps;
};

class ProfileScope
{
public:
    explicit ProfileScope(const char* name);
    ~ProfileScope();
    ProfileScope(const ProfileScope&) = delete;
    ProfileScope& operator=(const ProfileScope&) = delete;

private:
    bool mActive;
};

class GpuProfileScope
{
public:
    explicit GpuProfileScope(const char* name);
    ~GpuProfileScope();
    GpuProfileScope(const GpuProfileScope&) = delete;
    GpuProfileScope& operator=(const GpuProfileScope&) = delete;

private:
    bool mActive;
};

} // namespace prisma3d
