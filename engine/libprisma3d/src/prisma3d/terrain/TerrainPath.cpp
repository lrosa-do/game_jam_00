#include "prisma3d/terrain/TerrainPath.h"

#include <float.h>
#include <math.h>

namespace prisma3d
{

void TerrainPath::clear()
{
    points.clear();
    halfWidths.clear();
    closed = false;
}

void TerrainPath::add(const Math::Vec3& point, float halfWidth)
{
    points.push_back(point);
    halfWidths.push_back(halfWidth);
}

std::size_t TerrainPath::getSegmentCount() const
{
    if (points.size() < 2) return 0;
    return closed && points.size() > 2 ? points.size() : points.size() - 1;
}

float TerrainPath::getMaxHalfWidth() const
{
    float widest = 0.0f;
    for (std::size_t i = 0; i < halfWidths.size(); ++i)
        widest = halfWidths[i] > widest ? halfWidths[i] : widest;
    return widest;
}

bool TerrainPath::nearest(float x, float z, TerrainPathHit& hit) const
{
    if (points.empty()) return false;
    if (points.size() == 1)
    {
        const float dx = x - points[0].x;
        const float dz = z - points[0].z;
        hit.distance = sqrtf(dx * dx + dz * dz);
        hit.halfWidth = halfWidths[0];
        hit.clearance = hit.distance - hit.halfWidth;
        hit.height = points[0].y;
        return true;
    }

    const std::size_t segments = getSegmentCount();
    float bestClearance = FLT_MAX;
    for (std::size_t i = 0; i < segments; ++i)
    {
        const std::size_t j = (i + 1) % points.size();
        const Math::Vec3& a = points[i];
        const Math::Vec3& b = points[j];
        const float ex = b.x - a.x;
        const float ez = b.z - a.z;
        const float length2 = ex * ex + ez * ez;
        float t = 0.0f;
        if (length2 > 1.0e-12f) t = ((x - a.x) * ex + (z - a.z) * ez) / length2;
        t = t < 0.0f ? 0.0f : (t > 1.0f ? 1.0f : t);
        const float px = a.x + ex * t - x;
        const float pz = a.z + ez * t - z;
        const float distance = sqrtf(px * px + pz * pz);
        const float halfWidth = halfWidths[i] + (halfWidths[j] - halfWidths[i]) * t;
        const float clearance = distance - halfWidth;
        if (clearance < bestClearance)
        {
            bestClearance = clearance;
            hit.distance = distance;
            hit.clearance = clearance;
            hit.halfWidth = halfWidth;
            hit.height = a.y + (b.y - a.y) * t;
        }
    }
    return true;
}

} // namespace prisma3d
