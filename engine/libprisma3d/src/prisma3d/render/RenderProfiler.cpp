#include "prisma3d/render/RenderProfiler.h"

#include "platform.h"

#include <stdio.h>
#include <string.h>

namespace prisma3d
{

namespace
{

const double kNanosPerMillisecond = 1000000.0;
const float kSpikeDetailMilliseconds = 0.5f;
const std::uint64_t kRefreshNanos =
        static_cast<std::uint64_t>(RenderProfiler::kRefreshSeconds * 1000000000.0);

float toMilliseconds(std::uint64_t nanos)
{
    return static_cast<float>(static_cast<double>(nanos) / kNanosPerMillisecond);
}

} // namespace

RenderProfiler* RenderProfiler::sSingleton = nullptr;

RenderProfiler::RenderProfiler(prisma::Driver* driver)
        : mDriver(driver),
          mEnabled(false),
          mRequestFrame(0),
          mGpuAvailable(driver != nullptr && driver->caps().timerQueries),
          mSpikeThreshold(0.0f),
          mCpuSampleCount(0),
          mCpuDepth(0),
          mFrameStartNanos(0),
          mLastRefreshNanos(0),
          mCpuFrameMilliseconds(0.0f),
          mCpuDisplayFrameMilliseconds(0.0f),
          mOverflowWarned(false),
          mDepthOverflowWarned(false),
          mGpuSampleCount(0),
          mGpuCursor(0),
          mGpuDepth(0),
          mGpuRecording(false),
          mGpuLastRefreshNanos(0),
          mGpuFrameMilliseconds(0.0f),
          mGpuDisplayFrameMilliseconds(0.0f),
          mPassCount(0),
          mFrameNumber(0),
          mFpsWindowStartNanos(0),
          mFpsWindowFrames(0),
          mFps(0.0f)
{
    for (std::uint32_t i = 0; i < kMaxGpuDepth; ++i) mGpuStack[i] = 0;
    sSingleton = this;
}

RenderProfiler::~RenderProfiler()
{
    if (mDriver)
    {
        for (GpuFrame& frame: mGpuFrames)
        {
            for (GpuScope& scope: frame.scopes)
            {
                if (!scope.query.valid()) continue;
                mDriver->destroy(scope.query);
                scope.query = prisma::QueryHandle();
            }
        }
    }
    if (sSingleton == this) sSingleton = nullptr;
}

RenderProfiler* RenderProfiler::singleton()
{
    return sSingleton;
}

void RenderProfiler::setEnabled(bool enabled)
{
    mEnabled = enabled;
}

void RenderProfiler::request()
{
    mRequestFrame = mFrameNumber + 1;
}

bool RenderProfiler::enabled() const
{
    return mEnabled || mFrameNumber <= mRequestFrame + 1;
}

std::uint32_t RenderProfiler::findOrCreate(ProfileSample* samples, std::uint32_t& count,
        const char* name)
{
    for (std::uint32_t i = 0; i < count; ++i)
    {
        if (strcmp(samples[i].name, name) == 0) return i;
    }
    if (count >= kMaxSamples) return kMaxSamples;

    samples[count].name = name;
    return count++;
}

void RenderProfiler::pushHistory(ProfileSample* samples, std::uint32_t count, bool refresh)
{
    for (std::uint32_t i = 0; i < count; ++i)
    {
        ProfileSample& sample = samples[i];
        if (refresh) sample.display = sample.milliseconds;
        sample.history[sample.historyCursor] = sample.milliseconds;
        sample.historyCursor = (sample.historyCursor + 1) % ProfileSample::kHistorySize;
        if (sample.historyCount < ProfileSample::kHistorySize) ++sample.historyCount;

        float total = 0.0f;
        sample.maximum = 0.0f;
        for (std::uint32_t j = 0; j < sample.historyCount; ++j)
        {
            total += sample.history[j];
            if (sample.history[j] > sample.maximum) sample.maximum = sample.history[j];
        }
        sample.average =
                sample.historyCount ? total / static_cast<float>(sample.historyCount) : 0.0f;
        if (sample.average > sample.maximum) sample.average = sample.maximum;
    }
}

void RenderProfiler::beginFrame()
{
    for (std::uint32_t i = 0; i < mCpuSampleCount; ++i) mCpuSamples[i].milliseconds = 0.0f;
    mCpuDepth = 0;
    mGpuDepth = 0;
    mGpuRecording = false;
    mFrameStartNanos = time_nanos();

    if (!enabled() || !mGpuAvailable) return;

    mGpuCursor = (mGpuCursor + 1) % kFrameDelay;
    GpuFrame& frame = mGpuFrames[mGpuCursor];
    if (frame.pending)
    {
        const std::uint64_t resolveStart = time_nanos();
        const bool resolved = resolveGpu(frame);
        addSample("GpuResolve", toMilliseconds(time_nanos() - resolveStart));
        if (!resolved) return;
        harvestGpu(frame);
        frame.pending = false;
    }
    frame.count = 0;
    mGpuRecording = true;
}

void RenderProfiler::endFrame()
{
    const std::uint64_t now = time_nanos();

    if (mGpuRecording)
    {
        GpuFrame& frame = mGpuFrames[mGpuCursor];
        frame.pending = frame.count > 0;
        mGpuRecording = false;
    }

    ++mFrameNumber;
    if (mFpsWindowStartNanos == 0) mFpsWindowStartNanos = now;
    else
    {
        ++mFpsWindowFrames;
        const std::uint64_t elapsed = now - mFpsWindowStartNanos;
        if (elapsed >= kRefreshNanos)
        {
            mFps = static_cast<float>(static_cast<double>(mFpsWindowFrames) * 1000000000.0 /
                                      static_cast<double>(elapsed));
            mFpsWindowFrames = 0;
            mFpsWindowStartNanos = now;
        }
    }

    if (!enabled()) return;

    mCpuFrameMilliseconds = toMilliseconds(now - mFrameStartNanos);
    const std::uint32_t frameSample = findOrCreate(mCpuSamples, mCpuSampleCount, "Frame");
    if (frameSample < kMaxSamples) mCpuSamples[frameSample].milliseconds = mCpuFrameMilliseconds;

    const bool refresh = (now - mLastRefreshNanos) > kRefreshNanos;
    if (refresh)
    {
        mLastRefreshNanos = now;
        mCpuDisplayFrameMilliseconds = mCpuFrameMilliseconds;
    }
    pushHistory(mCpuSamples, mCpuSampleCount, refresh);

    if (mSpikeThreshold > 0.0f && mCpuFrameMilliseconds > mSpikeThreshold)
        logSpike("cpu", mFrameNumber, mCpuFrameMilliseconds, mCpuSamples, mCpuSampleCount);
}

void RenderProfiler::setSpikeThreshold(float milliseconds)
{
    mSpikeThreshold = milliseconds > 0.0f ? milliseconds : 0.0f;
}

float RenderProfiler::spikeThreshold() const
{
    return mSpikeThreshold;
}

void RenderProfiler::logSpike(const char* kind, std::uint64_t frame, float total,
        const ProfileSample* samples, std::uint32_t count) const
{
    char line[1024];
    std::size_t used = static_cast<std::size_t>(snprintf(line, sizeof(line),
            "spike: frame %llu %s %.2f ms:", static_cast<unsigned long long>(frame), kind, total));
    for (std::uint32_t i = 0; i < count && used + 1 < sizeof(line); ++i)
    {
        const ProfileSample& sample = samples[i];
        if (!sample.name || sample.milliseconds < kSpikeDetailMilliseconds) continue;
        if (strcmp(sample.name, "Frame") == 0) continue;
        const int written = snprintf(line + used, sizeof(line) - used, " %s %.2f", sample.name,
                sample.milliseconds);
        if (written <= 0) break;
        used += static_cast<std::size_t>(written);
    }
    log_warn("%s", line);
}

bool RenderProfiler::begin(const char* name)
{
    if (!enabled() || !name) return false;
    if (mCpuDepth >= kMaxDepth)
    {
        if (!mDepthOverflowWarned)
        {
            log_warn("profiler: kMaxDepth (%u) exceeded at '%s'; deeper scopes are not timed",
                    kMaxDepth, name);
            mDepthOverflowWarned = true;
        }
        return false;
    }
    const std::uint32_t sample = findOrCreate(mCpuSamples, mCpuSampleCount, name);
    if (sample >= kMaxSamples)
    {
        if (!mOverflowWarned)
        {
            log_warn("profiler: kMaxSamples (%u) exceeded; '%s' and further scopes are not tracked",
                    kMaxSamples, name);
            mOverflowWarned = true;
        }
        return false;
    }
    mCpuSamples[sample].depth = mCpuDepth;
    mCpuStack[mCpuDepth].sample = sample;
    mCpuStack[mCpuDepth].startNanos = time_nanos();
    ++mCpuDepth;
    return true;
}

void RenderProfiler::end()
{
    if (mCpuDepth == 0) return;
    const std::uint64_t now = time_nanos();
    const ActiveScope scope = mCpuStack[--mCpuDepth];
    const float elapsed = toMilliseconds(now - scope.startNanos);
    mCpuSamples[scope.sample].milliseconds += elapsed;
    if (mCpuDepth > 0) mCpuSamples[mCpuStack[mCpuDepth - 1].sample].milliseconds -= elapsed;
}

void RenderProfiler::addSample(const char* name, float milliseconds)
{
    if (!enabled() || !name) return;
    const std::uint32_t sample = findOrCreate(mCpuSamples, mCpuSampleCount, name);
    if (sample < kMaxSamples) mCpuSamples[sample].milliseconds += milliseconds;
}

bool RenderProfiler::resolveGpu(GpuFrame& frame)
{
    if (frame.count > 0)
    {
        GpuScope& last = frame.scopes[frame.count - 1];
        std::uint64_t nanos = 0;
        if (!last.resolved)
        {
            if (!mDriver->queryResult(last.query, &nanos)) return false;
            last.nanos = nanos;
            last.resolved = true;
        }
    }
    bool ready = true;
    for (std::uint32_t i = 0; i < frame.count; ++i)
    {
        GpuScope& scope = frame.scopes[i];
        if (scope.resolved) continue;
        std::uint64_t nanos = 0;
        if (mDriver->queryResult(scope.query, &nanos))
        {
            scope.nanos = nanos;
            scope.resolved = true;
        }
        else
            ready = false;
    }
    return ready;
}

void RenderProfiler::harvestGpu(GpuFrame& frame)
{
    for (std::uint32_t i = 0; i < mGpuSampleCount; ++i) mGpuSamples[i].milliseconds = 0.0f;
    mGpuFrameMilliseconds = 0.0f;

    for (std::uint32_t i = 0; i < frame.count; ++i)
    {
        const GpuScope& scope = frame.scopes[i];
        if (!scope.resolved) continue;

        const float milliseconds = toMilliseconds(scope.nanos);
        mGpuSamples[scope.sample].milliseconds += milliseconds;
        mGpuSamples[scope.sample].depth = scope.depth;
        if (scope.depth == 0) mGpuFrameMilliseconds += milliseconds;
        for (std::uint32_t parent = i; scope.depth > 0 && parent-- > 0;)
        {
            if (frame.scopes[parent].depth != scope.depth - 1) continue;
            if (frame.scopes[parent].resolved)
                mGpuSamples[frame.scopes[parent].sample].milliseconds -= milliseconds;
            break;
        }
    }

    const std::uint64_t now = time_nanos();
    const bool refresh = (now - mGpuLastRefreshNanos) > kRefreshNanos;
    if (refresh)
    {
        mGpuLastRefreshNanos = now;
        mGpuDisplayFrameMilliseconds = mGpuFrameMilliseconds;
    }
    pushHistory(mGpuSamples, mGpuSampleCount, refresh);

    if (mSpikeThreshold > 0.0f && mGpuFrameMilliseconds > mSpikeThreshold)
    {
        const std::uint64_t frameNumber =
                mFrameNumber > kFrameDelay ? mFrameNumber - kFrameDelay : 0;
        logSpike("gpu", frameNumber, mGpuFrameMilliseconds, mGpuSamples, mGpuSampleCount);
    }
}

bool RenderProfiler::beginGpu(const char* name)
{
    if (!enabled() || !mGpuAvailable || !mGpuRecording || !name || mGpuDepth >= kMaxGpuDepth)
        return false;

    GpuFrame& frame = mGpuFrames[mGpuCursor];
    if (frame.pending || frame.count >= kMaxGpuScopesPerFrame) return false;

    const std::uint32_t sample = findOrCreate(mGpuSamples, mGpuSampleCount, name);
    if (sample >= kMaxSamples) return false;

    GpuScope& scope = frame.scopes[frame.count];
    if (!scope.query.valid())
    {
        scope.query = mDriver->createQuery(prisma::QueryType::Time);
        if (!scope.query.valid()) return false;
    }
    scope.sample = sample;
    scope.depth = mGpuDepth;
    scope.nanos = 0;
    scope.resolved = false;

    mDriver->beginQuery(scope.query);
    mGpuStack[mGpuDepth++] = frame.count++;
    return true;
}

void RenderProfiler::endGpu()
{
    if (!mGpuAvailable || mGpuDepth == 0) return;
    const std::uint32_t index = mGpuStack[--mGpuDepth];
    mDriver->endQuery(mGpuFrames[mGpuCursor].scopes[index].query);
}

const ProfileSample* RenderProfiler::cpuSamples() const
{
    return mCpuSamples;
}

std::uint32_t RenderProfiler::cpuSampleCount() const
{
    return mCpuSampleCount;
}

float RenderProfiler::cpuFrameMilliseconds() const
{
    return mCpuDisplayFrameMilliseconds;
}

bool RenderProfiler::gpuAvailable() const
{
    return mGpuAvailable;
}

const ProfileSample* RenderProfiler::gpuSamples() const
{
    return mGpuSamples;
}

std::uint32_t RenderProfiler::gpuSampleCount() const
{
    return mGpuSampleCount;
}

float RenderProfiler::gpuFrameMilliseconds() const
{
    return mGpuDisplayFrameMilliseconds;
}

float RenderProfiler::fps() const
{
    return mFps;
}

std::uint64_t RenderProfiler::frameNumber() const
{
    return mFrameNumber;
}

int RenderProfiler::findPass(const char* name) const
{
    if (!name) return -1;
    for (std::uint32_t i = 0; i < mPassCount; ++i)
    {
        if (strcmp(mPasses[i].name, name) == 0) return static_cast<int>(i);
    }
    return -1;
}

bool RenderProfiler::registerPass(const char* name)
{
    if (!name) return false;
    if (findPass(name) >= 0) return true;
    if (mPassCount >= kMaxPasses) return false;
    mPasses[mPassCount].name = name;
    mPasses[mPassCount].enabled = true;
    ++mPassCount;
    return true;
}

void RenderProfiler::setPassEnabled(const char* name, bool enabled)
{
    const int index = findPass(name);
    if (index >= 0) mPasses[index].enabled = enabled;
}

bool RenderProfiler::passEnabled(const char* name) const
{
    const int index = findPass(name);
    return index < 0 ? true : mPasses[index].enabled;
}

std::uint32_t RenderProfiler::passCount() const
{
    return mPassCount;
}

const char* RenderProfiler::passName(std::uint32_t index) const
{
    return index < mPassCount ? mPasses[index].name : nullptr;
}

ProfileScope::ProfileScope(const char* name)
        : mActive(false)
{
    RenderProfiler* profiler = RenderProfiler::singleton();
    mActive = profiler != nullptr && profiler->begin(name);
}

ProfileScope::~ProfileScope()
{
    if (!mActive) return;
    RenderProfiler* profiler = RenderProfiler::singleton();
    if (profiler) profiler->end();
}

GpuProfileScope::GpuProfileScope(const char* name)
        : mActive(false)
{
    RenderProfiler* profiler = RenderProfiler::singleton();
    mActive = profiler != nullptr && profiler->beginGpu(name);
}

GpuProfileScope::~GpuProfileScope()
{
    if (!mActive) return;
    RenderProfiler* profiler = RenderProfiler::singleton();
    if (profiler) profiler->endGpu();
}

} // namespace prisma3d
