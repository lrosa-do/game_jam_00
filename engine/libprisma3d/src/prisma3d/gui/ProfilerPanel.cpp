#include "prisma3d/gui/ProfilerPanel.h"

#include "platform.h"

#include <stdio.h>
#include <string.h>

namespace prisma3d
{

namespace
{

const std::uint64_t kHeaderRefreshNanos =
        static_cast<std::uint64_t>(RenderProfiler::kRefreshSeconds * 1000000000.0);
const float kHistoryHeight = 90.0f;
const float kPlotTop = 14.0f;
const float kPlotHeight = 56.0f;
const float kFrameBudgetMilliseconds = 1000.0f / 60.0f;

const ProfileSample* findSample(const ProfileSample* samples, std::uint32_t count, const char* name)
{
    for (std::uint32_t i = 0; i < count; ++i)
    {
        if (samples[i].name && strcmp(samples[i].name, name) == 0) return &samples[i];
    }
    return nullptr;
}

float historyValue(const ProfileSample& sample, std::uint32_t index)
{
    const std::uint32_t start =
            sample.historyCount < ProfileSample::kHistorySize ? 0 : sample.historyCursor;
    return sample.history[(start + index) % ProfileSample::kHistorySize];
}

void drawPolyline(ig::Context& ui, const ProfileSample& sample, float width, float scale,
        const ig::Color& color)
{
    if (sample.historyCount < 2) return;
    const float stepX = width / static_cast<float>(ProfileSample::kHistorySize - 1);
    const float offsetX = width - stepX * static_cast<float>(sample.historyCount - 1);
    ig::Vec2 previous;
    for (std::uint32_t i = 0; i < sample.historyCount; ++i)
    {
        float value = historyValue(sample, i) / scale;
        if (value > 1.0f) value = 1.0f;
        const ig::Vec2 point(offsetX + stepX * static_cast<float>(i),
                kPlotTop + kPlotHeight * (1.0f - value));
        if (i > 0) ui.drawLine(previous, point, color, 1.0f);
        previous = point;
    }
}

} // namespace

ProfilerPanel::ProfilerPanel()
        : mHeaderRefreshNanos(0),
          mHeaderFps(0.0f),
          mHeaderFrameMilliseconds(0.0f),
          mHeaderGpuMilliseconds(0.0f),
          mCpuWeights{ 3.0f, 1.0f, 1.0f, 1.0f },
          mGpuWeights{ 3.0f, 1.0f, 1.0f, 1.0f }
{
}

void ProfilerPanel::draw(ig::Context& context, RenderProfiler& profiler, const RenderStats& stats)
{
    drawBody(context, profiler, stats, nullptr);
}

void ProfilerPanel::draw(ig::Context& context, RenderProfiler& profiler, RenderingServer& server)
{
    drawBody(context, profiler, server.getStats(), &server);
}

void ProfilerPanel::drawBody(ig::Context& context, RenderProfiler& profiler,
        const RenderStats& stats, RenderingServer* server)
{
    if (!open) return;
    profiler.request();
    if (context.beginWindow("Profiler", ig::Rect(320.0f, 10.0f, 420.0f, 520.0f), &open))
    {
        const std::uint64_t now = time_nanos();
        if (now - mHeaderRefreshNanos > kHeaderRefreshNanos)
        {
            mHeaderRefreshNanos = now;
            mHeaderFps = profiler.fps();
            mHeaderFrameMilliseconds = mHeaderFps > 0.0f ? 1000.0f / mHeaderFps : 0.0f;
            mHeaderGpuMilliseconds = profiler.gpuFrameMilliseconds();
            mHeaderStats = stats;
        }

        char text[160];
        snprintf(text, sizeof(text), "%.0f FPS   Frame %.2f ms", mHeaderFps,
                mHeaderFrameMilliseconds);
        context.label(text);
        if (profiler.gpuAvailable())
            snprintf(text, sizeof(text), "CPU %.2f ms   GPU %.2f ms",
                    profiler.cpuFrameMilliseconds(), mHeaderGpuMilliseconds);
        else
            snprintf(text, sizeof(text), "CPU %.2f ms   GPU n/a", profiler.cpuFrameMilliseconds());
        context.label(text);
        snprintf(text, sizeof(text),
                "Draws %u   Instances %u   Surfaces %u   Culled %u   Viewports %u",
                mHeaderStats.drawCalls, mHeaderStats.instancesDrawn, mHeaderStats.surfacesDrawn,
                mHeaderStats.instancesCulled, mHeaderStats.viewportsDrawn);
        context.label(text);
        snprintf(text, sizeof(text), "Triangles %u   Skinned surfaces %u   Skin dispatches %u",
                mHeaderStats.triangles, mHeaderStats.skinnedSurfaces, mHeaderStats.skinDispatches);
        context.label(text);

        const float bodyHeight = context.availableHeight();
        if (bodyHeight > 0.0f && context.beginChild("profiler.body", bodyHeight, false))
        {
            drawSections(context, profiler, server);
            context.endChild();
        }
    }
    context.endWindow();
}

void ProfilerPanel::drawSections(ig::Context& context, RenderProfiler& profiler,
        RenderingServer* server)
{
    char text[160];
    if (context.collapsingHeader("CPU", showCpu))
        drawSamples(context, "profiler.cpu", profiler.cpuSamples(), profiler.cpuSampleCount(),
                mCpuWeights);

    if (context.collapsingHeader("GPU", showGpu))
    {
        if (!profiler.gpuAvailable()) context.label("timer queries unavailable");
        else
        {
            snprintf(text, sizeof(text), "Measured %.2f ms of %.2f ms",
                    profiler.gpuFrameMilliseconds(), mHeaderFrameMilliseconds);
            context.label(text);
            drawSamples(context, "profiler.gpu", profiler.gpuSamples(), profiler.gpuSampleCount(),
                    mGpuWeights);
        }
    }

    if (context.collapsingHeader("History", showHistory))
    {
        drawHistory(context, findSample(profiler.cpuSamples(), profiler.cpuSampleCount(), "Frame"),
                findSample(profiler.gpuSamples(), profiler.gpuSampleCount(), "Frame"));
    }

    if (context.collapsingHeader("Passes", showPasses))
    {
        for (std::uint32_t i = 0; i < profiler.passCount(); ++i)
        {
            const char* name = profiler.passName(i);
            bool enabled = profiler.passEnabled(name);
            const bool before = enabled;
            context.checkbox(name, enabled);
            if (enabled != before) profiler.setPassEnabled(name, enabled);
        }
    }

    if (context.collapsingHeader("Culling", showCulling))
    {
        bool treeEnabled = server ? server->cullingTreeEnabled() : mHeaderStats.cullTree;
        if (server)
        {
            const bool before = treeEnabled;
            context.checkbox("Tree", treeEnabled);
            if (treeEnabled != before) server->setCullingTreeEnabled(treeEnabled);
        }
        snprintf(text, sizeof(text), "State: %s", treeEnabled ? "tree" : "linear");
        context.label(text);
        snprintf(text, sizeof(text), "Entries %u   Nodes %u   Depth %u", mHeaderStats.cullEntries,
                mHeaderStats.cullNodes, mHeaderStats.cullDepth);
        context.label(text);
        snprintf(text, sizeof(text), "Visited %u   Candidates %u   Accepted %u",
                mHeaderStats.cullNodesVisited, mHeaderStats.cullCandidates,
                mHeaderStats.cullAccepted);
        context.label(text);
        snprintf(text, sizeof(text), "Quality %.1f%s",
                static_cast<double>(mHeaderStats.cullQuality),
                mHeaderStats.cullRebuildPending ? "   (rebuilding)" : "");
        context.label(text);
    }
}

void ProfilerPanel::drawSamples(ig::Context& context, const char* id, const ProfileSample* samples,
        std::uint32_t count, float* weights)
{
    ig::TableStyle style;
    style.alternateRows = true;
    style.borders = true;
    style.rowHeight = 18.0f;
    style.rowSpacing = 0.0f;
    if (!context.beginTable(id, ig::Span<float>(weights, 4), style)) return;

    context.tableNextColumn();
    context.tableHeader("Scope");
    context.tableNextColumn();
    context.tableHeader("ms");
    context.tableNextColumn();
    context.tableHeader("avg");
    context.tableNextColumn();
    context.tableHeader("max");

    char text[96];
    const std::uint32_t kKeyDepth = 32;
    std::uint64_t keys[kKeyDepth];
    bool skipping = false;
    std::uint32_t skipDepth = 0;
    for (std::uint32_t i = 0; i < count; ++i)
    {
        const std::uint32_t depth = samples[i].depth < kKeyDepth ? samples[i].depth : kKeyDepth - 1;
        if (skipping)
        {
            if (depth > skipDepth) continue;
            skipping = false;
        }
        const char* name = samples[i].name ? samples[i].name : "";
        std::uint64_t key = depth > 0 ? keys[depth - 1] : 14695981039346656037ull;
        for (const char* c = name; *c; ++c)
            key = (key ^ static_cast<std::uint8_t>(*c)) * 1099511628211ull;
        keys[depth] = key;

        const bool hasChildren = i + 1 < count && samples[i + 1].depth > samples[i].depth;
        bool expanded = !mCollapsed.contains(key);
        context.tableNextColumn();
        if (context.tableTreeCell(name, static_cast<int>(depth), hasChildren, expanded))
        {
            if (expanded) mCollapsed.erase(key);
            else
                mCollapsed.insert(key);
        }
        if (hasChildren && !expanded)
        {
            skipping = true;
            skipDepth = depth;
        }

        context.tableNextColumn();
        snprintf(text, sizeof(text), "%.3f", samples[i].display);
        context.label(text);
        context.tableNextColumn();
        snprintf(text, sizeof(text), "%.3f", samples[i].average);
        context.label(text);
        context.tableNextColumn();
        snprintf(text, sizeof(text), "%.3f", samples[i].maximum);
        context.label(text);
    }
    context.endTable();
}

void ProfilerPanel::drawHistory(ig::Context& context, const ProfileSample* cpuFrame,
        const ProfileSample* gpuFrame)
{
    if (!context.beginChild("profiler.history", kHistoryHeight)) return;

    const float width = context.availableWidth();
    float scale = kFrameBudgetMilliseconds;
    if (cpuFrame && cpuFrame->maximum > scale) scale = cpuFrame->maximum;
    if (gpuFrame && gpuFrame->maximum > scale) scale = gpuFrame->maximum;

    const ig::Color cpuColor(90, 200, 120, 255);
    const ig::Color gpuColor(240, 170, 60, 255);
    const ig::Color budgetColor(200, 80, 80, 200);
    const ig::Color textColor(190, 190, 190, 255);

    const float budgetY = kPlotTop + kPlotHeight * (1.0f - kFrameBudgetMilliseconds / scale);
    context.drawLine(ig::Vec2(0.0f, budgetY), ig::Vec2(width, budgetY), budgetColor, 1.0f);
    if (cpuFrame) drawPolyline(context, *cpuFrame, width, scale, cpuColor);
    if (gpuFrame) drawPolyline(context, *gpuFrame, width, scale, gpuColor);

    char text[48];
    snprintf(text, sizeof(text), "%.1f ms", scale);
    context.drawText(text, ig::Vec2(0.0f, 0.0f), textColor);
    snprintf(text, sizeof(text), "%.1f", kFrameBudgetMilliseconds);
    context.drawText(text, ig::Vec2(width - 40.0f, budgetY - 14.0f), budgetColor);
    context.drawText("CPU", ig::Vec2(width * 0.5f - 40.0f, 0.0f), cpuColor);
    context.drawText("GPU", ig::Vec2(width * 0.5f + 10.0f, 0.0f), gpuColor);

    context.dummy(width, kPlotTop + kPlotHeight);
    context.endChild();
}

} // namespace prisma3d
