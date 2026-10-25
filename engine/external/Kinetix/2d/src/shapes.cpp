#include "kx/shapes.h"

#include <ct/vector.hpp>
#include <math.h>

namespace kx
{

    AABB Circle::ComputeAABB(const Transform &xf) const
    {
        vec2 p = xf.Transform(center);
        AABB aabb;
        aabb.lowerBound = vec2(p.x - radius, p.y - radius);
        aabb.upperBound = vec2(p.x + radius, p.y + radius);
        return aabb;
    }

    MassData Circle::ComputeMass(float density) const
    {
        MassData massData;
        massData.mass = density * kPi * radius * radius;
        massData.center = center;
        massData.I = massData.mass * (0.5f * radius * radius + Dot(center, center));
        return massData;
    }

    void Edge::SetTwoSided(const vec2 &v1, const vec2 &v2)
    {
        vertex1 = v1;
        vertex2 = v2;
        oneSided = false;
    }

    AABB Edge::ComputeAABB(const Transform &xf) const
    {
        vec2 v1 = xf.Transform(vertex1);
        vec2 v2 = xf.Transform(vertex2);

        vec2 lower = Min(v1, v2);
        vec2 upper = Max(v1, v2);

        vec2 r(radius, radius);
        AABB aabb;
        aabb.lowerBound = lower - r;
        aabb.upperBound = upper + r;
        return aabb;
    }

    MassData Edge::ComputeMass(float) const
    {
        MassData massData;
        massData.mass = 0.0f;
        massData.center = 0.5f * (vertex1 + vertex2);
        massData.I = 0.0f;
        return massData;
    }

    void Polygon::SetAsBox(float halfWidth, float halfHeight)
    {
        count = 4;
        vertices[0] = vec2(-halfWidth, -halfHeight);
        vertices[1] = vec2(halfWidth, -halfHeight);
        vertices[2] = vec2(halfWidth, halfHeight);
        vertices[3] = vec2(-halfWidth, halfHeight);
        normals[0] = vec2(0.0f, -1.0f);
        normals[1] = vec2(1.0f, 0.0f);
        normals[2] = vec2(0.0f, 1.0f);
        normals[3] = vec2(-1.0f, 0.0f);
        centroid = vec2(0.0f, 0.0f);
    }

    void Polygon::SetAsBox(float halfWidth, float halfHeight, const vec2 &boxCenter, float angle)
    {
        SetAsBox(halfWidth, halfHeight);
        centroid = boxCenter;

        Transform xf = MakeTransform(boxCenter, angle);

        for (int32_t i = 0; i < count; ++i)
        {
            vertices[i] = xf.Transform(vertices[i]);
            normals[i] = vec2(xf.a * normals[i].x + xf.c * normals[i].y,
                                    xf.b * normals[i].x + xf.d * normals[i].y);
        }
    }

    namespace
    {

        // Below this area (px^2) a polygon has no usable centroid, normals or mass.
        constexpr float kMinPolygonArea = 0.5f * kLinearSlop * kLinearSlop;

        vec2 AverageVertex(const vec2 *vs, int32_t vertCount)
        {
            vec2 c(0.0f, 0.0f);
            for (int32_t i = 0; i < vertCount; ++i)
                c += vs[i];
            return vertCount > 0 ? c * (1.0f / (float)vertCount) : c;
        }

        vec2 ComputePolygonCentroid(const vec2 *vs, int32_t vertCount)
        {
            vec2 c(0.0f, 0.0f);
            float area = 0.0f;

            vec2 s = vs[0];
            const float inv3 = 1.0f / 3.0f;

            for (int32_t i = 0; i < vertCount; ++i)
            {
                vec2 p1 = vs[0] - s;
                vec2 p2 = vs[i] - s;
                vec2 p3 = i + 1 < vertCount ? vs[i + 1] - s : vs[0] - s;

                vec2 e1 = p2 - p1;
                vec2 e2 = p3 - p1;

                float triangleArea = 0.5f * Cross(e1, e2);
                area += triangleArea;
                c += triangleArea * inv3 * (p1 + p2 + p3);
            }

            if (area <= kEpsilon)
                return AverageVertex(vs, vertCount);
            c = (1.0f / area) * c + s;
            return c;
        }

        float PolygonArea(const vec2 *vs, int32_t vertCount)
        {
            float area = 0.0f;
            for (int32_t i = 1; i + 1 < vertCount; ++i)
                area += 0.5f * Cross(vs[i] - vs[0], vs[i + 1] - vs[0]);
            return area;
        }

        // Gift-wrapping hull of ps[0..n), counter-clockwise, collinear points
        // dropped. Returns the hull size, or 0 when the hull has fewer than
        // three vertices or more than kMaxPolygonVertices.
        int32_t ComputeHull(const vec2 *ps, int32_t n, vec2 *out)
        {
            if (n < 3)
                return 0;

            int32_t i0 = 0;
            float x0 = ps[0].x;
            for (int32_t i = 1; i < n; ++i)
            {
                float x = ps[i].x;
                if (x > x0 || (x == x0 && ps[i].y < ps[i0].y))
                {
                    i0 = i;
                    x0 = x;
                }
            }

            // Room for nearly collinear vertices that the pass below removes;
            // a hull still bigger than this cannot shrink to a polygon. The
            // bound also stops the loop if the wrap never closes.
            const int32_t kScratch = 4 * kMaxPolygonVertices;
            vec2 hull[kScratch];
            int32_t m = 0;
            int32_t ih = i0;

            for (;;)
            {
                if (m >= kScratch)
                    return 0;
                hull[m] = ps[ih];

                int32_t ie = 0;
                for (int32_t j = 1; j < n; ++j)
                {
                    if (ie == ih)
                    {
                        ie = j;
                        continue;
                    }

                    vec2 r = ps[ie] - hull[m];
                    vec2 v = ps[j] - hull[m];
                    float c = Cross(r, v);
                    if (c < 0.0f)
                        ie = j;
                    if (c == 0.0f && (v.x * v.x + v.y * v.y) > (r.x * r.x + r.y * r.y))
                        ie = j;
                }

                ++m;
                ih = ie;
                if (ie == i0)
                    break;
            }

            // Drop vertices that sit on the line through their neighbours.
            // Gift wrapping skips exactly collinear points, not nearly
            // collinear ones, and those give zero-length normals.
            const float tolerance = 0.5f * kLinearSlop;
            bool removed = true;
            while (removed && m >= 3)
            {
                removed = false;
                for (int32_t i = 0; i < m; ++i)
                {
                    vec2 prev = hull[(i + m - 1) % m];
                    vec2 next = hull[(i + 1) % m];
                    vec2 edge = next - prev;
                    float length = sqrtf(Dot(edge, edge));
                    float distance = length > kEpsilon ? Cross(edge, hull[i] - prev) / length : 0.0f;
                    if (fabsf(distance) < tolerance)
                    {
                        for (int32_t k = i; k + 1 < m; ++k)
                            hull[k] = hull[k + 1];
                        --m;
                        removed = true;
                        break;
                    }
                }
            }

            if (m < 3 || m > kMaxPolygonVertices)
                return 0;
            if (PolygonArea(hull, m) < kMinPolygonArea)
                return 0;

            for (int32_t i = 0; i < m; ++i)
                out[i] = hull[i];
            return m;
        }

    } 

    bool Polygon::Set(const vec2 *points, int32_t pointCount)
    {
        if (!points || pointCount < 3)
            return false;

        vec2 local[kMaxPolygonVertices];
        ct::Vector<vec2> heap;
        vec2 *ps = local;
        if (pointCount > kMaxPolygonVertices)
        {
            heap.resize((size_t)pointCount);
            ps = heap.data();
        }

        int32_t n = 0;
        const float weldSquared = (0.5f * kLinearSlop) * (0.5f * kLinearSlop);
        for (int32_t i = 0; i < pointCount; ++i)
        {
            vec2 v = points[i];
            if (!(v.x == v.x && v.y == v.y))
                return false;

            bool unique = true;
            for (int32_t j = 0; j < n; ++j)
            {
                if (DistanceSquared(v, ps[j]) < weldSquared)
                {
                    unique = false;
                    break;
                }
            }
            if (unique)
                ps[n++] = v;
        }

        vec2 hull[kMaxPolygonVertices];
        int32_t m = ComputeHull(ps, n, hull);
        if (m == 0)
            return false;

        count = m;
        for (int32_t i = 0; i < m; ++i)
            vertices[i] = hull[i];

        for (int32_t i = 0; i < m; ++i)
        {
            int32_t i1 = i;
            int32_t i2 = i + 1 < m ? i + 1 : 0;
            vec2 edge = vertices[i2] - vertices[i1];
            normals[i] = Normalize(Cross(edge, 1.0f));
        }

        centroid = ComputePolygonCentroid(vertices, m);
        return true;
    }

    AABB Polygon::ComputeAABB(const Transform &xf) const
    {
        vec2 lower = xf.Transform(vertices[0]);
        vec2 upper = lower;

        for (int32_t i = 1; i < count; ++i)
        {
            vec2 v = xf.Transform(vertices[i]);
            lower = Min(lower, v);
            upper = Max(upper, v);
        }

        vec2 r(radius, radius);
        AABB aabb;
        aabb.lowerBound = lower - r;
        aabb.upperBound = upper + r;
        return aabb;
    }

    MassData Polygon::ComputeMass(float density) const
    {
        if (count < 3)
        {
            MassData empty;
            empty.mass = 0.0f;
            empty.center = AverageVertex(vertices, count);
            empty.I = 0.0f;
            return empty;
        }

        vec2 center(0.0f, 0.0f);
        float area = 0.0f;
        float I = 0.0f;

        vec2 s = vertices[0];
        const float inv3 = 1.0f / 3.0f;

        for (int32_t i = 0; i < count; ++i)
        {
            vec2 e1 = vertices[i] - s;
            vec2 e2 = i + 1 < count ? vertices[i + 1] - s : vertices[0] - s;

            float D = Cross(e1, e2);
            float triangleArea = 0.5f * D;
            area += triangleArea;
            center += triangleArea * inv3 * (e1 + e2);

            float ex1 = e1.x, ey1 = e1.y;
            float ex2 = e2.x, ey2 = e2.y;
            float intx2 = ex1 * ex1 + ex2 * ex1 + ex2 * ex2;
            float inty2 = ey1 * ey1 + ey2 * ey1 + ey2 * ey2;

            I += (0.25f * inv3 * D) * (intx2 + inty2);
        }

        MassData massData;
        if (area <= kEpsilon)
        {
            // A hand-filled polygon must not divide by zero in a release build.
            massData.mass = 0.0f;
            massData.center = AverageVertex(vertices, count);
            massData.I = 0.0f;
            return massData;
        }
        massData.mass = density * area;

        center *= 1.0f / area;
        massData.center = center + s;

        massData.I = density * I;
        massData.I += massData.mass * (Dot(massData.center, massData.center) - Dot(center, center));

        return massData;
    }

} 