#pragma once

#include "mathc.h"

#include <float.h>
#include <math.h>

namespace prisma3d
{
namespace terrain_math
{

struct Hit
{
    bool hit;
    float distance;
};

inline bool isNullBox(const Math::Box& box)
{
    return box.min.x > box.max.x || box.min.y > box.max.y || box.min.z > box.max.z;
}

inline Math::Vec3 faceNormalWithoutNormalize(const Math::Vec3& v1, const Math::Vec3& v2,
        const Math::Vec3& v3)
{
    return (v2 - v1).Cross(v3 - v1);
}

inline Math::Vec4 faceNormalPlaneWithoutNormalize(const Math::Vec3& v1, const Math::Vec3& v2,
        const Math::Vec3& v3)
{
    const Math::Vec3 normal = faceNormalWithoutNormalize(v1, v2, v3);
    return Math::Vec4(normal.x, normal.y, normal.z, -normal.Dot(v1));
}

inline Math::Vec3 basicFaceNormal(const Math::Vec3& v1, const Math::Vec3& v2, const Math::Vec3& v3)
{
    return faceNormalWithoutNormalize(v1, v2, v3).NormalizedSafe();
}

inline Hit intersectsPlane(const Math::Ray& ray, const Math::Vec4& plane)
{
    const Math::Vec3 normal(plane.x, plane.y, plane.z);
    const float denom = normal.Dot(ray.direction);
    if (fabsf(denom) < FLT_EPSILON) return { false, 0.0f };
    const float nom = normal.Dot(ray.origin) + plane.w;
    const float t = -(nom / denom);
    return { t >= 0.0f, t };
}

inline bool strictlyInside(const Math::Vec3& point, const Math::Vec3& lo, const Math::Vec3& hi)
{
    return point.x > lo.x && point.y > lo.y && point.z > lo.z && point.x < hi.x && point.y < hi.y &&
           point.z < hi.z;
}

inline Hit intersectsBox(const Math::Ray& ray, const Math::Box& box)
{
    if (isNullBox(box)) return { false, 0.0f };

    float lowt = 0.0f;
    bool hit = false;
    const Math::Vec3& lo = box.min;
    const Math::Vec3& hi = box.max;
    const Math::Vec3& rayorig = ray.origin;
    const Math::Vec3& raydir = ray.direction;

    if (strictlyInside(rayorig, lo, hi)) return { true, 0.0f };

    if (rayorig.x <= lo.x && raydir.x > 0.0f)
    {
        const float t = (lo.x - rayorig.x) / raydir.x;
        const Math::Vec3 p = rayorig + raydir * t;
        if (p.y >= lo.y && p.y <= hi.y && p.z >= lo.z && p.z <= hi.z && (!hit || t < lowt))
        {
            hit = true;
            lowt = t;
        }
    }
    if (rayorig.x >= hi.x && raydir.x < 0.0f)
    {
        const float t = (hi.x - rayorig.x) / raydir.x;
        const Math::Vec3 p = rayorig + raydir * t;
        if (p.y >= lo.y && p.y <= hi.y && p.z >= lo.z && p.z <= hi.z && (!hit || t < lowt))
        {
            hit = true;
            lowt = t;
        }
    }
    if (rayorig.y <= lo.y && raydir.y > 0.0f)
    {
        const float t = (lo.y - rayorig.y) / raydir.y;
        const Math::Vec3 p = rayorig + raydir * t;
        if (p.x >= lo.x && p.x <= hi.x && p.z >= lo.z && p.z <= hi.z && (!hit || t < lowt))
        {
            hit = true;
            lowt = t;
        }
    }
    if (rayorig.y >= hi.y && raydir.y < 0.0f)
    {
        const float t = (hi.y - rayorig.y) / raydir.y;
        const Math::Vec3 p = rayorig + raydir * t;
        if (p.x >= lo.x && p.x <= hi.x && p.z >= lo.z && p.z <= hi.z && (!hit || t < lowt))
        {
            hit = true;
            lowt = t;
        }
    }
    if (rayorig.z <= lo.z && raydir.z > 0.0f)
    {
        const float t = (lo.z - rayorig.z) / raydir.z;
        const Math::Vec3 p = rayorig + raydir * t;
        if (p.x >= lo.x && p.x <= hi.x && p.y >= lo.y && p.y <= hi.y && (!hit || t < lowt))
        {
            hit = true;
            lowt = t;
        }
    }
    if (rayorig.z >= hi.z && raydir.z < 0.0f)
    {
        const float t = (hi.z - rayorig.z) / raydir.z;
        const Math::Vec3 p = rayorig + raydir * t;
        if (p.x >= lo.x && p.x <= hi.x && p.y >= lo.y && p.y <= hi.y && (!hit || t < lowt))
        {
            hit = true;
            lowt = t;
        }
    }
    return { hit, lowt };
}

} // namespace terrain_math
} // namespace prisma3d
