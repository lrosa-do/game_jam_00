#pragma once

#include "prisma3d/render/MeshStore.h"

#include "mathc.h"

#include <ct/string.hpp>
#include <ct/vector.hpp>

#include <cstdint>

namespace prisma3d
{

struct LightmapUnwrapSettings
{
    using ProgressCallback = bool (*)(const char* stage, std::uint32_t percent, void* userData);

    std::uint32_t resolution = 1024;
    std::uint32_t padding = 4;
    float texelsPerUnit = 0.0f;
    Math::Vec3 scale = Math::Vec3(1.0f, 1.0f, 1.0f);
    ProgressCallback progress = nullptr;
    void* progressUserData = nullptr;
};

struct LightmapUnwrapResult
{
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    std::uint32_t chartCount = 0;
};

class LightmapUnwrapper
{
public:
    bool unwrap(const ct::Vector<SurfaceData>& input, ct::Vector<SurfaceData>& output,
            const LightmapUnwrapSettings& settings = LightmapUnwrapSettings(),
            LightmapUnwrapResult* result = nullptr, ct::String* error = nullptr) const;

    static float worldSurfaceArea(const ct::Vector<SurfaceData>& surfaces, const Math::Vec3& scale);
    static float fittedTexelsPerUnit(float area, std::uint32_t atlasResolution);
};

} // namespace prisma3d
