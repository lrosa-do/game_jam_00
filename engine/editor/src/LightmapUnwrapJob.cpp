#include "prisma3d/editor/LightmapUnwrapJob.h"

namespace prisma3d
{

LightmapUnwrapJob::LightmapUnwrapJob()
        : mPercent(0),
          mRunning(false),
          mFinished(false),
          mSucceeded(false),
          mCancelled(false)
{
}

LightmapUnwrapJob::~LightmapUnwrapJob()
{
    cancel();
    join();
}

void LightmapUnwrapJob::join()
{
    if (mThread.joinable()) mThread.join();
}

bool LightmapUnwrapJob::start(const ct::Vector<SurfaceData>& input,
        const LightmapUnwrapSettings& settings)
{
    if (running()) return false;
    join();

    mInput = input;
    mOutput.clear();
    mSettings = settings;
    mSettings.progress = &LightmapUnwrapJob::onProgress;
    mSettings.progressUserData = this;
    {
        ct::LockGuard lock(mMutex);
        mStage = "starting";
        mError.clear();
        mResult = LightmapUnwrapResult();
        mPercent = 0;
        mRunning = true;
        mFinished = false;
        mSucceeded = false;
        mCancelled = false;
    }
    mThread.start([this] { run(); });
    return true;
}

void LightmapUnwrapJob::run()
{
    LightmapUnwrapper unwrapper;
    ct::Vector<SurfaceData> output;
    LightmapUnwrapResult atlas;
    ct::String error;
    const bool ok = unwrapper.unwrap(mInput, output, mSettings, &atlas, &error);

    ct::LockGuard lock(mMutex);
    mResult = atlas;
    mOutput = static_cast<ct::Vector<SurfaceData>&&>(output);
    mError = error;
    mSucceeded = ok;
    mFinished = true;
    mRunning = false;
}

bool LightmapUnwrapJob::onProgress(const char* stage, std::uint32_t percent, void* userData)
{
    LightmapUnwrapJob& job = *static_cast<LightmapUnwrapJob*>(userData);
    ct::LockGuard lock(job.mMutex);
    if (stage) job.mStage = stage;
    job.mPercent = percent;
    return !job.mCancelled;
}

bool LightmapUnwrapJob::running() const
{
    ct::LockGuard lock(mMutex);
    return mRunning;
}

void LightmapUnwrapJob::cancel()
{
    ct::LockGuard lock(mMutex);
    mCancelled = true;
}

bool LightmapUnwrapJob::cancelled() const
{
    ct::LockGuard lock(mMutex);
    return mCancelled;
}

std::uint32_t LightmapUnwrapJob::percent() const
{
    ct::LockGuard lock(mMutex);
    return mPercent;
}

ct::String LightmapUnwrapJob::stage() const
{
    ct::LockGuard lock(mMutex);
    return mStage;
}

bool LightmapUnwrapJob::collect(ct::Vector<SurfaceData>& output, bool& succeeded)
{
    bool finished = false;
    {
        ct::LockGuard lock(mMutex);
        finished = mFinished;
        if (finished)
        {
            output = static_cast<ct::Vector<SurfaceData>&&>(mOutput);
            mOutput.clear();
            succeeded = mSucceeded;
            mFinished = false;
        }
    }
    if (finished)
    {
        join();
        mInput.clear();
    }
    return finished;
}

const LightmapUnwrapResult& LightmapUnwrapJob::result() const
{
    return mResult;
}

const ct::String& LightmapUnwrapJob::error() const
{
    return mError;
}

} // namespace prisma3d
