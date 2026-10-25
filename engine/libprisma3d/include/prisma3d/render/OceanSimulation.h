#pragma once

#include "prisma3d/render/TextureStore.h"

#include "mathc.h"

#include <ct/vector.hpp>

#include <cstdint>

namespace prisma3d
{

struct OceanSimulationParams
{
    float patchLength = 50.0f;
    float timeScale = 0.3f;
    float waveAmplitude = 1000.0f;
    Math::Vec2 windDirection = Math::Vec2(0.8f, 0.6f);
    float windSpeed = 600.0f;
    float windDependency = 0.07f;
    float choppyScale = 1.3f;
    std::uint32_t seed = 1;
};

class OceanSimulation
{
public:
    static constexpr std::uint32_t kSize = 512;
    static constexpr std::uint32_t kInputWidth = kSize + 4;
    static constexpr std::uint32_t kInputCount = kInputWidth * (kSize + 1);
    static constexpr std::uint32_t kFieldCount = kSize * kSize;

    explicit OceanSimulation(TextureStore& textures);
    ~OceanSimulation();
    OceanSimulation(const OceanSimulation&) = delete;
    OceanSimulation& operator=(const OceanSimulation&) = delete;

    static bool supported(prisma::Driver* driver);
    static void initialSpectrum(const OceanSimulationParams& params, ct::Vector<Math::Vec2>& h0,
            ct::Vector<float>& omega);

    bool create(const OceanSimulationParams& params);
    void release();
    bool valid() const;
    const OceanSimulationParams& params() const;

    void update(float time);
    float time() const;

    TextureId displacementMap() const;
    TextureId gradientMap() const;
    TextureId perlinMap() const;
    bool readFields(ct::Vector<float>& out);

private:
    struct Gpu;

    TextureStore& mTextures;
    OceanSimulationParams mParams;
    Gpu* mGpu;
    float mTime;
};

} // namespace prisma3d
