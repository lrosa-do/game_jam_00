#include "kx/internal/imageshape.h"
#include "kx/world.h"

#include <math.h>

namespace kx
{

    namespace
    {

        float SampleDensity(const unsigned char *pixels, int width, int height, int bpp, int x, int y)
        {
            if (x < 0 || y < 0 || x >= width || y >= height)
                return 0.0f;
            if (bpp == 4)
                return (float)pixels[(y * width + x) * 4 + 3];
            return (float)pixels[y * width + x];
        }

        void CollectSegment(ct::Vector<Polyline> &lines, const vec2 &v0, const vec2 &v1)
        {
            int before = -1;
            int after = -1;
            for (size_t i = 0; i < lines.size(); ++i)
            {
                const ct::Vector<vec2> &pts = lines[i].points;
                if (before < 0 && pts.back() == v0)
                    before = (int)i;
                if (after < 0 && pts[0] == v1)
                    after = (int)i;
            }

            if (before >= 0 && after >= 0)
            {
                if (before == after)
                {
                    lines[(size_t)before].points.push_back(v1);
                }
                else
                {
                    ct::Vector<vec2> &lbefore = lines[(size_t)before].points;
                    const ct::Vector<vec2> &lafter = lines[(size_t)after].points;
                    for (size_t i = 0; i < lafter.size(); ++i)
                        lbefore.push_back(lafter[i]);
                    lines[(size_t)after] = lines.back();
                    lines.pop_back();
                }
            }
            else if (before >= 0)
            {
                lines[(size_t)before].points.push_back(v1);
            }
            else if (after >= 0)
            {
                ct::Vector<vec2> &pts = lines[(size_t)after].points;
                ct::Vector<vec2> shifted;
                shifted.reserve(pts.size() + 1);
                shifted.push_back(v0);
                for (size_t i = 0; i < pts.size(); ++i)
                    shifted.push_back(pts[i]);
                pts = shifted;
            }
            else
            {
                Polyline line;
                line.points.push_back(v0);
                line.points.push_back(v1);
                lines.push_back(line);
            }
        }

        void Seg(ct::Vector<Polyline> &lines, const vec2 &a, const vec2 &b)
        {
            if (a != b)
                CollectSegment(lines, b, a);
        }

        void Segs(ct::Vector<Polyline> &lines, const vec2 &a, const vec2 &b, const vec2 &c)
        {
            Seg(lines, b, c);
            Seg(lines, a, b);
        }

        void MarchCellHard(ct::Vector<Polyline> &lines,
                           float a, float b, float c, float d, float t,
                           float x0, float x1, float y0, float y1)
        {
            float xm = 0.5f * (x0 + x1);
            float ym = 0.5f * (y0 + y1);

            int code = (a > t ? 1 : 0) | (b > t ? 2 : 0) | (c > t ? 4 : 0) | (d > t ? 8 : 0);

            switch (code)
            {
            case 0x1:
                Segs(lines, vec2(x0, ym), vec2(xm, ym), vec2(xm, y0));
                break;
            case 0x2:
                Segs(lines, vec2(xm, y0), vec2(xm, ym), vec2(x1, ym));
                break;
            case 0x3:
                Seg(lines, vec2(x0, ym), vec2(x1, ym));
                break;
            case 0x4:
                Segs(lines, vec2(xm, y1), vec2(xm, ym), vec2(x0, ym));
                break;
            case 0x5:
                Seg(lines, vec2(xm, y1), vec2(xm, y0));
                break;
            case 0x6:
                Segs(lines, vec2(xm, y0), vec2(xm, ym), vec2(x0, ym));
                Segs(lines, vec2(xm, y1), vec2(xm, ym), vec2(x1, ym));
                break;
            case 0x7:
                Segs(lines, vec2(xm, y1), vec2(xm, ym), vec2(x1, ym));
                break;
            case 0x8:
                Segs(lines, vec2(x1, ym), vec2(xm, ym), vec2(xm, y1));
                break;
            case 0x9:
                Segs(lines, vec2(x1, ym), vec2(xm, ym), vec2(xm, y0));
                Segs(lines, vec2(x0, ym), vec2(xm, ym), vec2(xm, y1));
                break;
            case 0xA:
                Seg(lines, vec2(xm, y0), vec2(xm, y1));
                break;
            case 0xB:
                Segs(lines, vec2(x0, ym), vec2(xm, ym), vec2(xm, y1));
                break;
            case 0xC:
                Seg(lines, vec2(x1, ym), vec2(x0, ym));
                break;
            case 0xD:
                Segs(lines, vec2(x1, ym), vec2(xm, ym), vec2(xm, y0));
                break;
            case 0xE:
                Segs(lines, vec2(xm, y0), vec2(xm, ym), vec2(x0, ym));
                break;
            default:
                break;
            }
        }

        void MarchImage(const unsigned char *pixels, int width, int height, int bpp,
                        unsigned char threshold, ct::Vector<Polyline> &lines)
        {
            float t = (float)threshold;

            ct::Vector<float> buffer;
            buffer.reserve((size_t)(width + 1));
            for (int i = 0; i <= width; ++i)
                buffer.push_back(SampleDensity(pixels, width, height, bpp, i, 0));

            for (int j = 0; j < height; ++j)
            {
                float y0 = (float)j;
                float y1 = (float)(j + 1);

                float bVal = buffer[0];
                float dVal = SampleDensity(pixels, width, height, bpp, 0, j + 1);
                buffer[0] = dVal;

                for (int i = 0; i < width; ++i)
                {
                    float x0 = (float)i;
                    float x1 = (float)(i + 1);

                    float aVal = bVal;
                    bVal = buffer[(size_t)(i + 1)];
                    float cVal = dVal;
                    dVal = SampleDensity(pixels, width, height, bpp, i + 1, j + 1);
                    buffer[(size_t)(i + 1)] = dVal;

                    MarchCellHard(lines, aVal, bVal, cVal, dVal, t, x0, x1, y0, y1);
                }
            }
        }

    } 

    int TraceImageOutlines(const unsigned char *pixels, int width, int height, int bpp,
                           unsigned char threshold, ct::Vector<Polyline> &outLines)
    {
        outLines.clear();

        if (!pixels || width <= 0 || height <= 0 || (bpp != 4 && bpp != 1))
            return 0;

        MarchImage(pixels, width, height, bpp, threshold, outLines);
        return (int)outLines.size();
    }

    namespace
    {

        float Sharpness(const vec2 &a, const vec2 &b, const vec2 &c)
        {
            return Dot(Normalize(a - b), Normalize(c - b));
        }

        bool IsClosed(const Polyline &line)
        {
            return line.points.size() > 1 && line.points[0] == line.points.back();
        }

    } 

    void SimplifyOutline(const Polyline &line, float toleranceRadians, Polyline &outSimplified)
    {
        outSimplified.points.clear();

        const ct::Vector<vec2> &src = line.points;
        if (src.size() < 3)
        {
            outSimplified.points = src;
            return;
        }

        ct::Vector<vec2> reduced;
        reduced.push_back(src[0]);
        reduced.push_back(src[1]);

        float minSharp = -cosf(toleranceRadians);

        for (size_t i = 2; i < src.size(); ++i)
        {
            const vec2 &vert = src[i];
            float sharp = Sharpness(reduced[reduced.size() - 2], reduced.back(), vert);

            if (sharp <= minSharp)
                reduced[reduced.size() - 1] = vert;
            else
                reduced.push_back(vert);
        }

        if (IsClosed(line) &&
            Sharpness(reduced[reduced.size() - 2], reduced[0], reduced[1]) < minSharp)
        {
            reduced[0] = reduced[reduced.size() - 2];
            reduced.pop_back();
        }

        outSimplified.points = reduced;
    }

    namespace
    {

        void SimplifySpan(const ct::Vector<vec2> &loop, size_t first, size_t last, float toleranceSquared,
                          ct::Vector<unsigned char> &keep)
        {
            // Douglas-Peucker over loop[first..last] (indices wrap).
            const size_t n = loop.size();
            const vec2 a = loop[first % n];
            const vec2 b = loop[last % n];
            const vec2 ab = b - a;
            const float lengthSquared = Dot(ab, ab);
            float worst = -1.0f;
            size_t worstIndex = first;
            for (size_t k = first + 1; k < last; ++k)
            {
                const vec2 p = loop[k % n];
                float distanceSquared;
                if (lengthSquared > kEpsilon)
                {
                    const float c = Cross(ab, p - a);
                    distanceSquared = c * c / lengthSquared;
                }
                else
                {
                    distanceSquared = DistanceSquared(p, a);
                }
                if (distanceSquared > worst)
                {
                    worst = distanceSquared;
                    worstIndex = k;
                }
            }
            if (worst > toleranceSquared)
            {
                keep[worstIndex % n] = 1;
                SimplifySpan(loop, first, worstIndex, toleranceSquared, keep);
                SimplifySpan(loop, worstIndex, last, toleranceSquared, keep);
            }
        }

        // Closed-loop Douglas-Peucker, split at vertex 0 and the vertex
        // farthest from it.
        void SimplifyLoop(const ct::Vector<vec2> &loop, float tolerance, ct::Vector<vec2> &out)
        {
            out.clear();
            const size_t n = loop.size();
            if (n < 3)
                return;
            size_t far = 0;
            float farthest = -1.0f;
            for (size_t i = 1; i < n; ++i)
            {
                const float d = DistanceSquared(loop[i], loop[0]);
                if (d > farthest)
                {
                    farthest = d;
                    far = i;
                }
            }
            ct::Vector<unsigned char> keep;
            keep.resize(n, 0);
            keep[0] = 1;
            keep[far] = 1;
            SimplifySpan(loop, 0, far, tolerance * tolerance, keep);
            SimplifySpan(loop, far, n, tolerance * tolerance, keep);
            for (size_t i = 0; i < n; ++i)
                if (keep[i])
                    out.push_back(loop[i]);
            if (out.size() < 3)
                out.clear();
        }

        bool PointInLoop(const ct::Vector<vec2> &loop, const vec2 &point)
        {
            bool inside = false;
            const size_t n = loop.size();
            for (size_t i = 0, j = n - 1; i < n; j = i++)
            {
                const vec2 &a = loop[i];
                const vec2 &b = loop[j];
                if ((a.y > point.y) != (b.y > point.y) &&
                    point.x < (b.x - a.x) * (point.y - a.y) / (b.y - a.y) + a.x)
                    inside = !inside;
            }
            return inside;
        }

        double OutlineArea2(const Polyline &line)
        {
            const ct::Vector<vec2> &pts = line.points;
            size_t n = pts.size();
            if (n < 3)
                return 0.0;

            double area = 0.0;
            for (size_t i = 0; i < n; ++i)
            {
                size_t j = (i + 1) % n;
                area += (double)pts[i].x * pts[j].y - (double)pts[j].x * pts[i].y;
            }
            return area < 0.0 ? -area : area;
        }

    } 

    int Body::AddFromImage(const unsigned char *pixels, int width, int height, int bpp,
                           unsigned char threshold, float density, float scale, float simplifyDegrees)
    {
        if (!pixels || width <= 0 || height <= 0 || (bpp != 4 && bpp != 1))
            return 0;

        ct::Vector<Polyline> rawLines;
        int lineCount = TraceImageOutlines(pixels, width, height, bpp, threshold, rawLines);
        if (lineCount == 0)
            return 0;

        float toleranceRadians = simplifyDegrees * (3.14159265359f / 180.0f);

        ct::Vector<Polyline> simplified;
        simplified.reserve(rawLines.size());
        for (size_t i = 0; i < rawLines.size(); ++i)
        {
            Polyline out;
            SimplifyOutline(rawLines[i], toleranceRadians, out);
            if (out.points.size() >= 3)
                simplified.push_back(out);
        }

        if (simplified.size() == 0)
            return 0;

        for (size_t i = 0; i < simplified.size(); ++i)
        {
            size_t best = i;
            double bestArea = OutlineArea2(simplified[i]);
            for (size_t j = i + 1; j < simplified.size(); ++j)
            {
                double area = OutlineArea2(simplified[j]);
                if (area > bestArea)
                {
                    bestArea = area;
                    best = j;
                }
            }
            if (best != i)
            {
                Polyline tmp = simplified[i];
                simplified[i] = simplified[best];
                simplified[best] = tmp;
            }
        }

        vec2 center((float)width * 0.5f, (float)height * 0.5f);

        // Every traced outline, closed and in body space.
        ct::Vector<ct::Vector<vec2>> loops;
        for (size_t i = 0; i < simplified.size(); ++i)
        {
            const ct::Vector<vec2> &pts = simplified[i].points;
            int loopCount = (int)pts.size();
            if (loopCount > 0 && pts[0] == pts.back())
                --loopCount;
            if (loopCount < 3)
                continue;

            ct::Vector<vec2> local;
            local.resize((size_t)loopCount);
            for (int k = 0; k < loopCount; ++k)
                local[(size_t)k] = (pts[(size_t)k] - center) * scale;
            loops.push_back(local);
        }

        // Outlines never cross, so one vertex tells whether a loop lies inside
        // another. Depth even = solid, odd = hole of the loop one level up.
        const size_t loopCount = loops.size();
        ct::Vector<int> depth;
        depth.resize(loopCount, 0);
        for (size_t i = 0; i < loopCount; ++i)
            for (size_t j = 0; j < loopCount; ++j)
                if (i != j && PointInLoop(loops[j], loops[i][0]))
                    ++depth[i];

        const int shapesBefore = mShapeCount;
        const float pixel = scale > 0.0f ? scale : -scale;
        ct::Vector<ct::Vector<vec2>> work;
        float tolerance = 0.0f;
        for (int attempt = 0; attempt < 5; ++attempt, tolerance = tolerance > 0.0f ? tolerance * 2.0f : 0.5f * pixel)
        {
            // A marching-squares staircase has a reflex vertex at every notch; too many do not fit
            // a body's shapes even after merging, so coarser outlines are tried. Hole topology
            // stays that of the traced outlines.
            work.clear();
            for (size_t i = 0; i < loopCount; ++i)
            {
                work.push_back(ct::Vector<vec2>());
                if (tolerance > 0.0f)
                    SimplifyLoop(loops[i], tolerance, work.back());
                else
                    work.back() = loops[i];
            }

            int added = 0;
            bool fits = true;
            for (size_t i = 0; i < loopCount && fits; ++i)
            {
                if (depth[i] % 2 != 0 || work[i].size() < 3)
                    continue;

                ct::Vector<const vec2 *> holes;
                ct::Vector<int> holeCounts;
                for (size_t j = 0; j < loopCount; ++j)
                {
                    if (depth[j] == depth[i] + 1 && work[j].size() >= 3 && PointInLoop(loops[i], loops[j][0]))
                    {
                        holes.push_back(work[j].data());
                        holeCounts.push_back((int)work[j].size());
                    }
                }

                const vec2 *outline = work[i].data();
                const int outlineCount = (int)work[i].size();
                int pieces = AddMeshPieces(outline, outlineCount, holes.data(), holeCounts.data(),
                                           (int)holes.size(), density);
                // Hole loops that touch the outline defeat the constrained
                // triangulation; the solid without its holes beats no solid.
                if (pieces == -1 && !holes.empty())
                    pieces = AddMeshPieces(outline, outlineCount, nullptr, nullptr, 0, density);
                if (pieces == -2)
                    fits = false;
                else if (pieces > 0)
                    added += pieces;
            }

            if (fits)
                return added;

            // All or nothing, like AddMesh.
            mShapeCount = shapesBefore;
            RecomputeMass();
        }

        return 0;
    }

    Body *World::CreateFromImage(const vec2 &pos, const unsigned char *pixels, int width, int height, int bpp,
                                 unsigned char threshold, float density, float scale, float simplifyDegrees)
    {
        Body *body = CreateBody(BodyType::Dynamic, pos);
        int added = body->AddFromImage(pixels, width, height, bpp, threshold, density, scale, simplifyDegrees);

        if (added == 0)
        {
            Destroy(body);
            return nullptr;
        }

        return body;
    }

} 