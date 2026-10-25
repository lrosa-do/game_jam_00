#pragma once

#include <cstdint>

namespace prisma3d
{

class Random
{
public:
    static constexpr std::uint64_t kDefaultSeed = 12047754176567800795ULL;
    static constexpr std::uint64_t kDefaultInc = 1442695040888963407ULL;

    explicit Random(std::uint64_t seed = kDefaultSeed, std::uint64_t inc = kDefaultInc);

    void seed(std::uint64_t seed);
    std::uint64_t getSeed() const;
    void setState(std::uint64_t state);
    std::uint64_t getState() const;
    void randomize();

    std::uint32_t rand();
    std::uint32_t rand(std::uint32_t bounds);
    double randd();
    float randf();
    double randdRange(double from, double to);
    float randfRange(float from, float to);
    int randiRange(int from, int to);

private:
    std::uint64_t mState;
    std::uint64_t mInc;
    std::uint64_t mSequence;
    std::uint64_t mSeed;
};

} // namespace prisma3d
