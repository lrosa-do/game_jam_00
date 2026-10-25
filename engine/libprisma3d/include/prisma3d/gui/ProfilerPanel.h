#pragma once

#include "prisma3d/render/RenderProfiler.h"
#include "prisma3d/render/RenderingServer.h"

#include <ct/hashset.hpp>
#include <igui/Gui.hpp>

namespace prisma3d
{

class ProfilerPanel
{
public:
    ProfilerPanel();

    void draw(ig::Context& context, RenderProfiler& profiler, const RenderStats& stats);
    void draw(ig::Context& context, RenderProfiler& profiler, RenderingServer& server);

    bool open = true;
    bool showCpu = true;
    bool showGpu = true;
    bool showHistory = true;
    bool showPasses = true;
    bool showCulling = true;

private:
    void drawSections(ig::Context& context, RenderProfiler& profiler, RenderingServer* server);
    void drawSamples(ig::Context& context, const char* id, const ProfileSample* samples,
            std::uint32_t count, float* weights);
    void drawHistory(ig::Context& context, const ProfileSample* cpuFrame,
            const ProfileSample* gpuFrame);
    void drawBody(ig::Context& context, RenderProfiler& profiler, const RenderStats& stats,
            RenderingServer* server);

    std::uint64_t mHeaderRefreshNanos;
    float mHeaderFps;
    float mHeaderFrameMilliseconds;
    float mHeaderGpuMilliseconds;
    RenderStats mHeaderStats;
    ct::HashSet<std::uint64_t> mCollapsed;
    float mCpuWeights[4];
    float mGpuWeights[4];
};

} // namespace prisma3d
