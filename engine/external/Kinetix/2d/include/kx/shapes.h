#pragma once

#include "math.h"
#include <stdint.h>

#include "common.h"

namespace kx
{

    struct AABB
    {
        vec2 lowerBound;
        vec2 upperBound;

        vec2 GetCenter() const { return 0.5f * (lowerBound + upperBound); }
        vec2 GetExtents() const { return 0.5f * (upperBound - lowerBound); }

        float GetPerimeter() const
        {
            float wx = upperBound.x - lowerBound.x;
            float wy = upperBound.y - lowerBound.y;
            return 2.0f * (wx + wy);
        }

        void Combine(const AABB &aabb)
        {
            lowerBound = Min(lowerBound, aabb.lowerBound);
            upperBound = Max(upperBound, aabb.upperBound);
        }

        void Combine(const AABB &aabb1, const AABB &aabb2)
        {
            lowerBound = Min(aabb1.lowerBound, aabb2.lowerBound);
            upperBound = Max(aabb1.upperBound, aabb2.upperBound);
        }

        bool Contains(const AABB &aabb) const
        {
            return lowerBound.x <= aabb.lowerBound.x &&
                   lowerBound.y <= aabb.lowerBound.y &&
                   aabb.upperBound.x <= upperBound.x &&
                   aabb.upperBound.y <= upperBound.y;
        }
    };

    inline bool TestOverlap(const AABB &a, const AABB &b)
    {
        vec2 d1 = b.lowerBound - a.upperBound;
        vec2 d2 = a.lowerBound - b.upperBound;

        if (d1.x > 0.0f || d1.y > 0.0f)
            return false;
        if (d2.x > 0.0f || d2.y > 0.0f)
            return false;
        return true;
    }

    struct MassData
    {
        float mass;
        vec2 center;
        float I;
    };

    struct Circle
    {
        vec2 center = vec2(0.0f, 0.0f);
        float radius = 0.0f;

        AABB ComputeAABB(const Transform &xf) const;
        MassData ComputeMass(float density) const;
    };

    struct Edge
    {
        Edge() : radius(kPolygonRadius), oneSided(false)
        {
            vertex0 = vec2(0.0f, 0.0f);
            vertex3 = vec2(0.0f, 0.0f);
        }

        void SetTwoSided(const vec2 &v1, const vec2 &v2);

        AABB ComputeAABB(const Transform &xf) const;
        MassData ComputeMass(float density) const;

        vec2 vertex1, vertex2;
        vec2 vertex0, vertex3;
        float radius;
        bool oneSided;
    };

    constexpr int32_t kMaxPolygonVertices = 8;

    struct Polygon
    {
        Polygon() : count(0), radius(kPolygonRadius) { centroid = vec2(0.0f, 0.0f); }

        // Builds the convex hull of the points. Returns false - and leaves the
        // polygon untouched - when the input is degenerate (fewer than three
        // distinct points, collinear, near-zero area) or when the hull needs
        // more than kMaxPolygonVertices vertices. Any number of input points
        // is accepted; only the hull has to fit.
        bool Set(const vec2 *points, int32_t pointCount);
        void SetAsBox(float halfWidth, float halfHeight);
        void SetAsBox(float halfWidth, float halfHeight, const vec2 &center, float angle);

        AABB ComputeAABB(const Transform &xf) const;
        MassData ComputeMass(float density) const;

        vec2 centroid;
        vec2 vertices[kMaxPolygonVertices];
        vec2 normals[kMaxPolygonVertices];
        int32_t count;
        float radius;
    };

} 