#pragma once

#include "prisma3d/core/Random.h"
#include "prisma3d/render/MeshStore.h"

#include <ct/vector.hpp>

#include <cstdint>

namespace prisma3d
{

struct TreeParams
{
    float clumpMax = 0.8f;
    float clumpMin = 0.5f;
    float lengthFalloffFactor = 0.85f;
    float lengthFalloffPower = 1.0f;
    float branchFactor = 2.0f;
    float radiusFalloffRate = 0.6f;
    float taperRate = 0.95f;
    float maxRadius = 0.25f;
    float climbRate = 1.5f;
    float trunkKink = 0.0f;
    float twistRate = 13.0f;
    float trunkLength = 2.5f;
    float initialBranchLength = 0.85f;
    float dropAmount = 0.0f;
    float growAmount = 0.0f;
    float sweepAmount = 0.0f;
    float vMultiplier = 0.2f;
    float twigScale = 2.0f;
    std::uint32_t segments = 6;
    std::uint32_t levels = 3;
    std::uint32_t trunkSteps = 2;
    std::int32_t seed = 10;

    bool operator==(const TreeParams& other) const;
    bool operator!=(const TreeParams& other) const;
};

struct TreePreset
{
    const char* name;
    TreeParams params;
    std::uint32_t twigTexture;
};

struct TreeGeometry
{
    SurfaceData bark;
    SurfaceData leaves;
    ct::Vector<std::uint32_t> trunkIndices;

    void clear();
};

class TreeGenerator
{
public:
    static constexpr std::uint32_t kMaxSegments = 64;
    static constexpr std::uint32_t kMaxLevels = 12;
    static constexpr std::uint32_t kMaxTrunkSteps = 128;
    static constexpr std::uint64_t kMaxBudget = 1ull << 19;

    static TreeParams sanitize(const TreeParams& params);
    static void generate(const TreeParams& params, TreeGeometry& out);
    static void buildShadowGeometry(const TreeGeometry& full, int leafStep, TreeGeometry& out);

    static std::uint32_t presetCount();
    static const TreePreset& preset(std::uint32_t index);
    static TreeParams randomParams(Random& random);
};

} // namespace prisma3d
