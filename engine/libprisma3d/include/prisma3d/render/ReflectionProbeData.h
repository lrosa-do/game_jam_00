#pragma once

#include "mathc.h"

#include <ct/vector.hpp>

#include <cstdint>

namespace prisma3d
{

enum class ReflectionProbeUpdateMode : std::uint8_t
{
    Once,
    Always
};

enum class ReflectionProbeAmbientMode : std::uint8_t
{
    Disabled,
    Environment,
    Color
};

struct ReflectionProbeParams
{
    ReflectionProbeUpdateMode updateMode = ReflectionProbeUpdateMode::Once;
    float intensity = 1.0f;
    float blendDistance = 1.0f;
    ReflectionProbeAmbientMode ambientMode = ReflectionProbeAmbientMode::Environment;
    Math::Vec3 ambientColor = Math::Vec3(0.0f, 0.0f, 0.0f);
    float ambientEnergy = 1.0f;
    float maxDistance = 0.0f;
    Math::Vec3 size = Math::Vec3(20.0f, 20.0f, 20.0f);
    Math::Vec3 originOffset = Math::Vec3(0.0f, 0.0f, 0.0f);
    bool interior = false;
    bool boxProjection = false;
    bool shadows = false;
    std::uint32_t cullMask = (1u << 20) - 1u;
    std::uint32_t reflectionMask = (1u << 20) - 1u;
    float meshLodThreshold = 0.01f;
    float bakedExposure = 1.0f;
};

struct ReflectionDataGpu
{
    float boxExtents[3];
    float index;
    float boxOffset[3];
    std::uint32_t mask;
    float ambient[3];
    float intensity;
    float blendDistance;
    std::uint32_t exterior;
    std::uint32_t boxProject;
    std::uint32_t ambientMode;
    float exposureNormalization;
    std::uint32_t pad0;
    std::uint32_t pad1;
    std::uint32_t pad2;
    float localMatrix[16];
};

struct ReflectionInstanceInput
{
    const ReflectionProbeParams* probe = nullptr;
    Math::Mat4 transform = Math::Mat4::Identity();
    int atlasIndex = -1;
};

struct ReflectionSorted
{
    std::uint32_t instance = 0;
    Math::Mat4 transform = Math::Mat4::Identity();
    Math::Vec3 extents = Math::Vec3(0.0f, 0.0f, 0.0f);
};

std::uint32_t buildReflectionData(const ReflectionInstanceInput* instances,
        std::uint32_t instanceCount, const Math::Mat4& cameraTransform, float cameraExposure,
        std::uint32_t maxReflections, ct::Vector<ReflectionDataGpu>& data,
        ct::Vector<ReflectionSorted>& sorted);

} // namespace prisma3d
