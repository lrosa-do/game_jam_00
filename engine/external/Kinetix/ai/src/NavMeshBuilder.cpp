#include <kinetix/core/Containers.h>
#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

#include "NavMeshBuilder.h"

#include <math.h>

namespace kinetix::AI
{

namespace
{

constexpr f32 kFloatMax = 3.402823466e+38F;

// Spans closer than this are one solid: a floor and the wall standing on it merge.
constexpr f32 kSpanTouch = 0.01f;

// A clipped triangle has at most 3 + 4 corners.
constexpr s32 kClipMax = 12;

struct Span
{
    f32 lo = 0.0f;
    f32 hi = 0.0f;
    s32 next = -1;
    bool walkable = false;
};

struct Cell
{
    f32 y = 0.0f;
    f32 ceiling = 0.0f;
    u32 column = 0;
    s32 neighbor[4] = {-1, -1, -1, -1}; // -x, +z, +x, -z
};

// Keeps the part of a polygon on one side of x = bound (axis 0) or z = bound (axis 2).
s32 clipPolygon(const Math::vec3* in, s32 count, Math::vec3* out, s32 axis, f32 bound,
                bool keepAbove)
{
    s32 written = 0;
    for (s32 i = 0; i < count; ++i)
    {
        const Math::vec3& a = in[i];
        const Math::vec3& b = in[(i + 1) % count];
        const f32 da = (axis == 0 ? a.x : a.z) - bound;
        const f32 db = (axis == 0 ? b.x : b.z) - bound;
        const bool insideA = keepAbove ? da >= 0.0f : da <= 0.0f;
        const bool insideB = keepAbove ? db >= 0.0f : db <= 0.0f;
        if (insideA && written < kClipMax)
            out[written++] = a;
        if (insideA != insideB && written < kClipMax)
        {
            const f32 t = da / (da - db);
            out[written++] = a + (b - a) * t;
        }
    }
    return written;
}

class Heightfield
{
public:
    Heightfield(u32 columnsX, u32 columnsZ, f32 mergeClimb)
        : mColumnsX(columnsX), mMergeClimb(mergeClimb)
    {
        mHeads.resize(static_cast<usize>(columnsX) * columnsZ, -1);
    }

    // Adds a solid interval to a column, merged with the spans it overlaps. The merged top is
    // walkable when the higher of the two tops is, or either is when the tops are within a climb.
    void add(u32 x, u32 z, f32 lo, f32 hi, bool walkable)
    {
        const usize column = static_cast<usize>(z) * mColumnsX + x;
        s32 previous = -1;
        s32 current = mHeads[column];
        while (current != -1)
        {
            Span& span = mSpans[static_cast<usize>(current)];
            if (span.lo > hi + kSpanTouch)
                break;
            if (span.hi < lo - kSpanTouch)
            {
                previous = current;
                current = span.next;
                continue;
            }
            if (fabsf(span.hi - hi) <= mMergeClimb)
                walkable = walkable || span.walkable;
            else if (span.hi > hi)
                walkable = span.walkable;
            lo = Math::min(lo, span.lo);
            hi = Math::max(hi, span.hi);
            const s32 next = span.next;
            if (previous == -1)
                mHeads[column] = next;
            else
                mSpans[static_cast<usize>(previous)].next = next;
            current = next;
        }
        Span added;
        added.lo = lo;
        added.hi = hi;
        added.walkable = walkable;
        added.next = current;
        mSpans.push_back(added);
        const s32 index = static_cast<s32>(mSpans.size() - 1);
        if (previous == -1)
            mHeads[column] = index;
        else
            mSpans[static_cast<usize>(previous)].next = index;
    }

    s32 head(usize column) const { return mHeads[column]; }
    Span& span(s32 index) { return mSpans[static_cast<usize>(index)]; }
    usize columns() const { return mHeads.size(); }

    u32 liveSpans() const
    {
        u32 count = 0;
        for (usize c = 0; c < mHeads.size(); ++c)
            for (s32 s = mHeads[c]; s != -1; s = mSpans[static_cast<usize>(s)].next)
                ++count;
        return count;
    }

private:
    u32 mColumnsX;
    f32 mMergeClimb;
    ct::Vector<s32> mHeads;
    ct::Vector<Span> mSpans;
};

u64 vertexKey(u32 x, u32 z, f32 y)
{
    const s32 height = static_cast<s32>(floorf(y * 50.0f + 0.5f)); // 2 cm
    return (static_cast<u64>(x & 0xFFFFFu) << 44) | (static_cast<u64>(z & 0xFFFFFu) << 24) |
           static_cast<u64>(static_cast<u32>(height) & 0xFFFFFFu);
}

}

bool NavMeshBuilder::build(const f32* vertices, s32 vertexCount, const s32* indices,
                           s32 triangleCount, const NavMeshBuildConfig& config,
                           ct::Vector<f32>& outVertices, ct::Vector<s32>& outIndices,
                           NavMeshBuildStats* stats)
{
    outVertices.clear();
    outIndices.clear();
    if (!vertices || !indices || vertexCount <= 0 || triangleCount <= 0 ||
        config.cellSize <= 0.0f || config.agentHeight <= 0.0f)
        return false;

    Math::vec3 low(kFloatMax);
    Math::vec3 high(-kFloatMax);
    for (s32 i = 0; i < vertexCount; ++i)
    {
        const Math::vec3 p(vertices[i * 3 + 0], vertices[i * 3 + 1], vertices[i * 3 + 2]);
        low = Math::min(low, p);
        high = Math::max(high, p);
    }

    f32 cell = config.cellSize;
    u32 columnsX = 0;
    u32 columnsZ = 0;
    for (;;)
    {
        columnsX = static_cast<u32>(floorf((high.x - low.x) / cell)) + 1u;
        columnsZ = static_cast<u32>(floorf((high.z - low.z) / cell)) + 1u;
        if (static_cast<u64>(columnsX) * columnsZ <= config.maxCells)
            break;
        cell *= 1.25f;
    }
    const f32 inverseCell = 1.0f / cell;

    const f32 climb = Math::max(config.agentMaxClimb, 0.0f);
    Heightfield field(columnsX, columnsZ, climb);
    const f32 slopeCos = cosf(Math::radians(config.agentMaxSlope));

    Math::vec3 row[kClipMax];
    Math::vec3 band[kClipMax];
    Math::vec3 piece[kClipMax];
    Math::vec3 clipped[kClipMax];
    for (s32 t = 0; t < triangleCount; ++t)
    {
        const s32 i0 = indices[t * 3 + 0];
        const s32 i1 = indices[t * 3 + 1];
        const s32 i2 = indices[t * 3 + 2];
        if (i0 < 0 || i0 >= vertexCount || i1 < 0 || i1 >= vertexCount || i2 < 0 ||
            i2 >= vertexCount)
            continue;
        const Math::vec3 corners[3] = {
            Math::vec3(vertices[i0 * 3], vertices[i0 * 3 + 1], vertices[i0 * 3 + 2]),
            Math::vec3(vertices[i1 * 3], vertices[i1 * 3 + 1], vertices[i1 * 3 + 2]),
            Math::vec3(vertices[i2 * 3], vertices[i2 * 3 + 1], vertices[i2 * 3 + 2])};
        const Math::vec3 normal =
            Math::cross(corners[1] - corners[0], corners[2] - corners[0]);
        const f32 length = Math::length(normal);
        if (length < 1e-12f)
            continue;
        const bool walkable = normal.y / length >= slopeCos;

        const Math::vec3 lo = Math::min(Math::min(corners[0], corners[1]), corners[2]);
        const Math::vec3 hi = Math::max(Math::max(corners[0], corners[1]), corners[2]);
        const s32 x0 = Math::max(static_cast<s32>(floorf((lo.x - low.x) * inverseCell)), 0);
        const s32 x1 = Math::min(static_cast<s32>(floorf((hi.x - low.x) * inverseCell)),
                                 static_cast<s32>(columnsX) - 1);
        const s32 z0 = Math::max(static_cast<s32>(floorf((lo.z - low.z) * inverseCell)), 0);
        const s32 z1 = Math::min(static_cast<s32>(floorf((hi.z - low.z) * inverseCell)),
                                 static_cast<s32>(columnsZ) - 1);

        for (s32 z = z0; z <= z1; ++z)
        {
            const f32 zMin = low.z + static_cast<f32>(z) * cell;
            s32 count = clipPolygon(corners, 3, row, 2, zMin, true);
            count = clipPolygon(row, count, band, 2, zMin + cell, false);
            if (count < 3)
                continue;
            for (s32 x = x0; x <= x1; ++x)
            {
                const f32 xMin = low.x + static_cast<f32>(x) * cell;
                s32 n = clipPolygon(band, count, piece, 0, xMin, true);
                n = clipPolygon(piece, n, clipped, 0, xMin + cell, false);
                if (n < 3)
                    continue;
                f32 bottom = clipped[0].y;
                f32 top = clipped[0].y;
                for (s32 k = 1; k < n; ++k)
                {
                    bottom = Math::min(bottom, clipped[k].y);
                    top = Math::max(top, clipped[k].y);
                }
                field.add(static_cast<u32>(x), static_cast<u32>(z), bottom, top, walkable);
            }
        }
    }

    // A ledge, kerb or the riser of a step no higher than a climb above walkable ground is
    // walkable too, as Recast's low hanging obstacle filter does.
    for (usize column = 0; column < field.columns(); ++column)
    {
        bool belowWalkable = false;
        f32 belowTop = 0.0f;
        for (s32 s = field.head(column); s != -1; s = field.span(s).next)
        {
            Span& span = field.span(s);
            const bool was = span.walkable;
            if (!span.walkable && belowWalkable && span.hi - belowTop <= climb)
                span.walkable = true;
            belowWalkable = was;
            belowTop = span.hi;
        }
    }

    ct::Vector<Cell> cells;
    ct::Vector<u32> firstCell(field.columns() + 1, 0u);
    for (usize column = 0; column < field.columns(); ++column)
    {
        firstCell[column] = static_cast<u32>(cells.size());
        for (s32 s = field.head(column); s != -1; s = field.span(s).next)
        {
            const Span& span = field.span(s);
            if (!span.walkable)
                continue;
            const f32 ceiling = span.next != -1 ? field.span(span.next).lo : kFloatMax;
            if (ceiling - span.hi < config.agentHeight)
                continue;
            Cell floor;
            floor.y = span.hi;
            floor.ceiling = ceiling;
            floor.column = static_cast<u32>(column);
            cells.push_back(floor);
        }
    }
    firstCell[field.columns()] = static_cast<u32>(cells.size());

    const s32 offsetX[4] = {-1, 0, 1, 0};
    const s32 offsetZ[4] = {0, 1, 0, -1};
    for (usize c = 0; c < cells.size(); ++c)
    {
        Cell& here = cells[c];
        const s32 x = static_cast<s32>(here.column % columnsX);
        const s32 z = static_cast<s32>(here.column / columnsX);
        for (s32 d = 0; d < 4; ++d)
        {
            const s32 nx = x + offsetX[d];
            const s32 nz = z + offsetZ[d];
            if (nx < 0 || nz < 0 || nx >= static_cast<s32>(columnsX) ||
                nz >= static_cast<s32>(columnsZ))
                continue;
            const usize column = static_cast<usize>(nz) * columnsX + static_cast<usize>(nx);
            f32 best = kFloatMax;
            for (u32 k = firstCell[column]; k < firstCell[column + 1]; ++k)
            {
                const Cell& other = cells[k];
                const f32 step = fabsf(other.y - here.y);
                const f32 gap = Math::min(other.ceiling, here.ceiling) - Math::max(other.y, here.y);
                if (step > climb || gap < config.agentHeight || step >= best)
                    continue;
                best = step;
                here.neighbor[d] = static_cast<s32>(k);
            }
        }
    }

    // Chamfer distance to the nearest edge cell (2 across, 3 diagonally), two passes as in Recast.
    constexpr u16 kFar = 0xFFFF;
    ct::Vector<u16> distance(cells.size(), kFar);
    for (usize c = 0; c < cells.size(); ++c)
        for (s32 d = 0; d < 4; ++d)
            if (cells[c].neighbor[d] < 0)
                distance[c] = 0;
    auto relax = [&](usize c, s32 straight, s32 turn) {
        const s32 n = cells[c].neighbor[straight];
        if (n < 0)
            return;
        u32 best = static_cast<u32>(distance[static_cast<usize>(n)]) + 2u;
        const s32 corner = cells[static_cast<usize>(n)].neighbor[turn];
        if (corner >= 0)
            best = Math::min(best, static_cast<u32>(distance[static_cast<usize>(corner)]) + 3u);
        if (best < distance[c])
            distance[c] = static_cast<u16>(Math::min(best, static_cast<u32>(kFar)));
    };
    for (usize c = 0; c < cells.size(); ++c)
    {
        relax(c, 0, 3); // -x, then -z
        relax(c, 3, 2); // -z, then +x
    }
    for (usize c = cells.size(); c > 0; --c)
    {
        relax(c - 1, 2, 1); // +x, then +z
        relax(c - 1, 1, 0); // +z, then -x
    }

    const u32 radiusCells = static_cast<u32>(ceilf(Math::max(config.agentRadius, 0.0f) * inverseCell));
    HashMap<u64, s32> vertexOf;
    auto vertex = [&](u32 x, u32 z, f32 y) {
        const u64 key = vertexKey(x, z, y);
        const s32* found = vertexOf.find(key);
        if (found)
            return *found;
        const s32 index = static_cast<s32>(outVertices.size() / 3);
        outVertices.push_back(low.x + static_cast<f32>(x) * cell);
        outVertices.push_back(y);
        outVertices.push_back(low.z + static_cast<f32>(z) * cell);
        vertexOf[key] = index;
        return index;
    };
    u32 kept = 0;
    for (usize c = 0; c < cells.size(); ++c)
    {
        if (distance[c] < radiusCells * 2u)
            continue;
        ++kept;
        const Cell& here = cells[c];
        const u32 x = here.column % columnsX;
        const u32 z = here.column / columnsX;
        // Counter-clockwise from above, so the normal of both triangles points up.
        const s32 a = vertex(x, z, here.y);
        const s32 b = vertex(x, z + 1, here.y);
        const s32 cc = vertex(x + 1, z + 1, here.y);
        const s32 d = vertex(x + 1, z, here.y);
        outIndices.push_back(a);
        outIndices.push_back(b);
        outIndices.push_back(cc);
        outIndices.push_back(a);
        outIndices.push_back(cc);
        outIndices.push_back(d);
    }

    if (stats)
    {
        stats->columnsX = columnsX;
        stats->columnsZ = columnsZ;
        stats->cellSize = cell;
        stats->spans = field.liveSpans();
        stats->floorCells = static_cast<u32>(cells.size());
        stats->keptCells = kept;
    }
    return !outIndices.empty();
}

}
