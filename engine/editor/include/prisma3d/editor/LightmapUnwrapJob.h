#pragma once

#include "prisma3d/editor/LightmapUnwrapper.h"

#include <ct/string.hpp>
#include <ct/thread.hpp>
#include <ct/vector.hpp>

#include <cstdint>

namespace prisma3d
{

class LightmapUnwrapJob
{
public:
    LightmapUnwrapJob();
    ~LightmapUnwrapJob();
    LightmapUnwrapJob(const LightmapUnwrapJob&) = delete;
    LightmapUnwrapJob& operator=(const LightmapUnwrapJob&) = delete;

    bool start(const ct::Vector<SurfaceData>& input, const LightmapUnwrapSettings& settings);

    bool running() const;
    void cancel();

    std::uint32_t percent() const;
    ct::String stage() const;

    bool collect(ct::Vector<SurfaceData>& output, bool& succeeded);

    const LightmapUnwrapResult& result() const;
    const ct::String& error() const;
    bool cancelled() const;

private:
    static bool onProgress(const char* stage, std::uint32_t percent, void* userData);
    void run();
    void join();

    mutable ct::Mutex mMutex;
    ct::Thread mThread;
    ct::Vector<SurfaceData> mInput;
    ct::Vector<SurfaceData> mOutput;
    LightmapUnwrapSettings mSettings;
    LightmapUnwrapResult mResult;
    ct::String mStage;
    ct::String mError;
    std::uint32_t mPercent;
    bool mRunning;
    bool mFinished;
    bool mSucceeded;
    bool mCancelled;
};

} // namespace prisma3d
