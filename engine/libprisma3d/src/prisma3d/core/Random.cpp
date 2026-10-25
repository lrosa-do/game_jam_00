// *Really* minimal PCG32 code / (c) 2014 M.E. O'Neill / pcg-random.org
// Licensed under Apache License 2.0 (NO WARRANTY, etc. see website)

#include "prisma3d/core/Random.h"

#include "platform.h"

#include <math.h>
#include <time.h>

#if defined(_MSC_VER)
#include <intrin.h>
#endif

namespace prisma3d
{

namespace
{

int countLeadingZeros(std::uint32_t value)
{
#if defined(__GNUC__)
    return __builtin_clz(value);
#else
    unsigned long index;
    _BitScanReverse(&index, value);
    return 31 - static_cast<int>(index);
#endif
}

} // namespace

Random::Random(std::uint64_t seedValue, std::uint64_t inc)
        : mState(0),
          mInc(0),
          mSequence(inc),
          mSeed(0)
{
    seed(seedValue);
}

void Random::seed(std::uint64_t seedValue)
{
    mSeed = seedValue;
    mState = 0U;
    mInc = (mSequence << 1u) | 1u;
    rand();
    mState += seedValue;
    rand();
}

std::uint64_t Random::getSeed() const
{
    return mSeed;
}

void Random::setState(std::uint64_t state)
{
    mState = state;
}

std::uint64_t Random::getState() const
{
    return mState;
}

void Random::randomize()
{
    seed((static_cast<std::uint64_t>(time(nullptr)) + time_nanos() / 1000u) * mState + kDefaultInc);
}

std::uint32_t Random::rand()
{
    const std::uint64_t oldState = mState;
    mState = oldState * 6364136223846793005ULL + (mInc | 1);
    const std::uint32_t xorshifted =
            static_cast<std::uint32_t>(((oldState >> 18u) ^ oldState) >> 27u);
    const std::uint32_t rot = static_cast<std::uint32_t>(oldState >> 59u);
    return (xorshifted >> rot) | (xorshifted << ((-rot) & 31));
}

std::uint32_t Random::rand(std::uint32_t bounds)
{
    const std::uint32_t threshold = -bounds % bounds;
    for (;;)
    {
        const std::uint32_t r = rand();
        if (r >= threshold) return r % bounds;
    }
}

double Random::randd()
{
    const std::uint32_t protoExpOffset = rand();
    if (protoExpOffset == 0) return 0;
    const std::uint64_t high = static_cast<std::uint64_t>(rand()) << 32;
    const std::uint64_t significand = high | rand() | 0x8000000000000001ULL;
    return ldexp(static_cast<double>(significand), -64 - countLeadingZeros(protoExpOffset));
}

float Random::randf()
{
    const std::uint32_t protoExpOffset = rand();
    if (protoExpOffset == 0) return 0;
    return ldexpf(static_cast<float>(rand() | 0x80000001U),
            -32 - countLeadingZeros(protoExpOffset));
}

double Random::randdRange(double from, double to)
{
    return randd() * (to - from) + from;
}

float Random::randfRange(float from, float to)
{
    return randf() * (to - from) + from;
}

int Random::randiRange(int from, int to)
{
    if (from == to) return from;

    const std::int64_t low = from < to ? from : to;
    const std::int64_t high = from < to ? to : from;
    const std::uint32_t diff = static_cast<std::uint32_t>(high - low);

    if (diff == UINT32_MAX) return static_cast<int>(static_cast<std::int64_t>(rand()) + low);

    return static_cast<int>(static_cast<std::int64_t>(rand(diff + 1U)) + low);
}

} // namespace prisma3d
