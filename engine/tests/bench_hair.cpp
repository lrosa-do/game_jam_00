#include "prisma3d/scene/HairStrands.h"

#include "platform.h"

#include <ct/threadpool.hpp>

#include <stdio.h>

using namespace prisma3d;

namespace
{

struct Scalp
{
    ct::Vector<Math::Vec3> positions;
    ct::Vector<Math::Vec3> normals;
    ct::Vector<std::uint32_t> indices;
};

Scalp makeScalp()
{
    Scalp scalp;
    const int side = 16;
    for (int z = 0; z <= side; ++z)
        for (int x = 0; x <= side; ++x)
        {
            scalp.positions.push_back(
                    Math::Vec3(static_cast<float>(x) * 0.02f, 0.0f, static_cast<float>(z) * 0.02f));
            scalp.normals.push_back(Math::Vec3(0.2f, 1.0f, 0.1f).Normalized());
        }
    for (int z = 0; z < side; ++z)
        for (int x = 0; x < side; ++x)
        {
            const std::uint32_t a = static_cast<std::uint32_t>(z * (side + 1) + x);
            const std::uint32_t b = a + 1;
            const std::uint32_t c = a + static_cast<std::uint32_t>(side + 1);
            const std::uint32_t d = c + 1;
            scalp.indices.push_back(a);
            scalp.indices.push_back(b);
            scalp.indices.push_back(d);
            scalp.indices.push_back(a);
            scalp.indices.push_back(d);
            scalp.indices.push_back(c);
        }
    return scalp;
}

double measure(HairStrands& hair, HairFrame& frame, ct::ThreadPool* pool, int frames)
{
    const std::uint32_t strands = hair.params().strandCount;
    const std::uint64_t start = time_nanos();
    for (int i = 0; i < frames; ++i)
    {
        frame.time += frame.deltaTime;
        hair.beginFrame(frame);
        std::uint32_t windBegin = 0;
        std::uint32_t windEnd = 0;
        hair.windSlicesToCompute(windBegin, windEnd);
        if (pool && windEnd > windBegin)
        {
            HairStrands* work = &hair;
            pool->parallel_for(
                    windBegin, windEnd,
                    [work](std::size_t slice)
                    {
                        work->computeWindSlices(static_cast<std::uint32_t>(slice),
                                static_cast<std::uint32_t>(slice) + 1);
                    },
                    1);
        }
        else
        {
            hair.computeWindSlices(windBegin, windEnd);
        }
        if (pool)
        {
            const std::uint32_t kChunk = 128;
            const std::uint32_t chunks = (strands + kChunk - 1) / kChunk;
            HairStrands* work = &hair;
            pool->parallel_for(0, chunks,
                    [work, strands](std::size_t chunk)
                    {
                        const std::uint32_t begin = static_cast<std::uint32_t>(chunk) * kChunk;
                        const std::uint32_t end =
                                begin + kChunk < strands ? begin + kChunk : strands;
                        work->simulateRange(begin, end);
                    });
        }
        else
        {
            hair.simulateRange(0, strands);
        }
        hair.endFrame();
    }
    return static_cast<double>(time_nanos() - start) * 1e-6 / static_cast<double>(frames);
}

void run(std::uint32_t strands, std::uint32_t segments, bool wind, ct::ThreadPool* pool,
        std::uint32_t windSlices = 32)
{
    HairStrands hair;
    hair.setWindSlicesPerFrame(windSlices);
    HairParams params;
    params.strandCount = strands;
    params.segmentCount = segments;
    params.length = 0.3f;
    params.width = 0.02f;
    params.gravityPower = 2.0f;
    hair.setParams(params);
    const Scalp scalp = makeScalp();
    hair.setScalp(scalp.positions.data(), scalp.normals.data(), nullptr,
            static_cast<std::uint32_t>(scalp.positions.size()), scalp.indices.data(),
            static_cast<std::uint32_t>(scalp.indices.size()));
    HairFrame frame;
    frame.cameraPosition = Math::Vec3(0.0f, 1.0f, 2.0f);
    frame.deltaTime = 1.0f / 60.0f;
    if (wind) frame.windDirection = Math::Vec3(1.0f, 0.0f, 0.3f);
    measure(hair, frame, pool, 5);
    const double milliseconds = measure(hair, frame, pool, 60);
    printf("%6u strands x %u segments  wind %s (%2u slices)  %s: %.2f ms per frame (%u vertices)\n",
            strands, segments, wind ? "on " : "off", windSlices, pool ? "pool  " : "single",
            milliseconds, hair.vertexCount());
}

} // namespace

int main()
{
    ct::ThreadPool pool;
    run(2000, 4, false, nullptr);
    run(2000, 4, true, nullptr);
    run(8000, 6, false, nullptr);
    run(8000, 6, true, nullptr);
    run(8000, 6, false, &pool);
    run(8000, 6, true, &pool);
    run(8000, 6, true, &pool, 8);
    run(2000, 4, true, nullptr, 8);
    run(20000, 6, false, nullptr);
    run(20000, 6, false, &pool);
    return 0;
}
