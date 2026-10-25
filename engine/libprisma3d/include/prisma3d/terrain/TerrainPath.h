#pragma once

#include "mathc.h"

#include <ct/vector.hpp>

#include <cstddef>

namespace prisma3d
{

struct TerrainPathHit
{
    float distance = 0.0f;
    float clearance = 0.0f;
    float height = 0.0f;
    float halfWidth = 0.0f;
};

struct TerrainPath
{
    ct::Vector<Math::Vec3> points;
    ct::Vector<float> halfWidths;
    bool closed = false;

    void clear();
    void add(const Math::Vec3& point, float halfWidth);
    std::size_t getSegmentCount() const;
    float getMaxHalfWidth() const;
    bool nearest(float x, float z, TerrainPathHit& hit) const;
};

} // namespace prisma3d
