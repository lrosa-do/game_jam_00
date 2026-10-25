#pragma once

#include <ct/slotmap32.hpp>

#include <cstdint>

namespace prisma3d
{

struct ImpostorTag;
struct ImpostorSetTag;
using ImpostorId = ct::Handle32<ImpostorTag>;
using ImpostorSetId = ct::Handle32<ImpostorSetTag>;

constexpr std::uint32_t kImpostorCaptureAngles = 36;
constexpr std::uint32_t kImpostorTextureDim = 128;
constexpr std::uint32_t kMaxImpostorCount = 8;
constexpr std::uint32_t kImpostorProperties = 3;

enum class ImpostorProperty : std::uint8_t
{
    Albedo,
    Normal,
    Surface
};

struct ImpostorInstanceInfo
{
    float center[3] = { 0.0f, 0.0f, 0.0f };
    float radius = 0.0f;
    float fadeDistance = 0.0f;
    int sliceOffset = -1;
};

struct ImpostorCardReadback
{
    std::uint32_t slice = 0;
    float dither = 0.0f;
    std::uint32_t color = 0;
    float positions[4][3] = {};
    float normal[3] = {};
};

constexpr std::uint32_t impostorSliceOffset(std::uint32_t slot)
{
    return slot * kImpostorCaptureAngles * kImpostorProperties;
}

constexpr std::uint32_t impostorSlice(std::uint32_t slot, std::uint32_t angle,
        ImpostorProperty property)
{
    return impostorSliceOffset(slot) + angle * kImpostorProperties +
           static_cast<std::uint32_t>(property);
}

} // namespace prisma3d
