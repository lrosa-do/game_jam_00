#include "prisma3d/terrain/TerrainBrush.h"

#include "prisma3d/terrain/TerrainLayerBlendMap.h"

#include <ct/vector.hpp>

#include <float.h>
#include <math.h>

namespace prisma3d
{

namespace
{

float clamp01(float value)
{
    return value < 0.0f ? 0.0f : (value > 1.0f ? 1.0f : value);
}

float smoothStep(float edge0, float edge1, float value)
{
    const float t = clamp01((value - edge0) / (edge1 - edge0));
    return t * t * (3.0f - 2.0f * t);
}

struct Footprint
{
    int minX = 0;
    int minY = 0;
    int maxX = -1;
    int maxY = -1;
    float centreX = 0.0f;
    float centreY = 0.0f;
    float scale = 1.0f;
};

Footprint footprint(const Terrain& terrain, const Math::Vec3& worldCentre, float radius)
{
    Footprint result;
    Math::Vec3 ts;
    terrain.getTerrainPosition(worldCentre, ts);
    const float last = static_cast<float>(terrain.getSize() - 1);
    result.scale = terrain.getWorldSize() / last;
    result.centreX = ts.x * last;
    result.centreY = ts.y * last;
    const float points = radius / result.scale;
    const int limit = terrain.getSize() - 1;
    auto clampIndex = [limit](float v)
    { return v < 0.0f ? 0 : (v > limit ? limit : static_cast<int>(v)); };
    result.minX = clampIndex(floorf(result.centreX - points));
    result.maxX = clampIndex(ceilf(result.centreX + points));
    result.minY = clampIndex(floorf(result.centreY - points));
    result.maxY = clampIndex(ceilf(result.centreY + points));
    return result;
}

struct Lattice
{
    int width = 0;
    int height = 0;
    float x0 = 0.0f;
    float dx = 1.0f;
    float z0 = 0.0f;
    float dz = 1.0f;
};

struct FootprintField
{
    Lattice lattice;
    ct::Vector<float> clearance;
    ct::Vector<float> height;
};

void latticeRange(float origin, float step, int count, float low, float high, int& first, int& last)
{
    float a = (low - origin) / step;
    float b = (high - origin) / step;
    if (a > b)
    {
        const float swap = a;
        a = b;
        b = swap;
    }
    first = static_cast<int>(floorf(a)) - 1;
    last = static_cast<int>(ceilf(b)) + 1;
    first = first < 0 ? 0 : first;
    last = last > count - 1 ? count - 1 : last;
}

void computeFootprint(const TerrainPath& path, const Lattice& lattice, float reach,
        FootprintField& out)
{
    out.lattice = lattice;
    const std::size_t count = static_cast<std::size_t>(lattice.width) * lattice.height;
    out.clearance.clear();
    out.clearance.resize(count, FLT_MAX);
    out.height.clear();
    out.height.resize(count, 0.0f);

    const std::size_t segments = path.getSegmentCount();
    for (std::size_t i = 0; i < segments; ++i)
    {
        const std::size_t j = (i + 1) % path.points.size();
        const Math::Vec3& a = path.points[i];
        const Math::Vec3& b = path.points[j];
        const float segmentReach =
                reach - path.getMaxHalfWidth() +
                (path.halfWidths[i] > path.halfWidths[j] ? path.halfWidths[i] : path.halfWidths[j]);
        const float minX = (a.x < b.x ? a.x : b.x) - segmentReach;
        const float maxX = (a.x > b.x ? a.x : b.x) + segmentReach;
        const float minZ = (a.z < b.z ? a.z : b.z) - segmentReach;
        const float maxZ = (a.z > b.z ? a.z : b.z) + segmentReach;
        int firstX = 0;
        int lastX = -1;
        int firstZ = 0;
        int lastZ = -1;
        latticeRange(lattice.x0, lattice.dx, lattice.width, minX, maxX, firstX, lastX);
        latticeRange(lattice.z0, lattice.dz, lattice.height, minZ, maxZ, firstZ, lastZ);

        const float ex = b.x - a.x;
        const float ez = b.z - a.z;
        const float length2 = ex * ex + ez * ez;
        for (int row = firstZ; row <= lastZ; ++row)
        {
            const float z = lattice.z0 + lattice.dz * static_cast<float>(row);
            for (int column = firstX; column <= lastX; ++column)
            {
                const float x = lattice.x0 + lattice.dx * static_cast<float>(column);
                float t = 0.0f;
                if (length2 > 1.0e-12f) t = ((x - a.x) * ex + (z - a.z) * ez) / length2;
                t = clamp01(t);
                const float px = a.x + ex * t - x;
                const float pz = a.z + ez * t - z;
                const float distance = sqrtf(px * px + pz * pz);
                const float halfWidth =
                        path.halfWidths[i] + (path.halfWidths[j] - path.halfWidths[i]) * t;
                const float clearance = distance - halfWidth;
                const std::size_t slot = static_cast<std::size_t>(row) * lattice.width + column;
                if (clearance < out.clearance[slot])
                {
                    out.clearance[slot] = clearance;
                    out.height[slot] = a.y + (b.y - a.y) * t;
                }
            }
        }
    }
}

} // namespace

float terrainPathWeight(float clearance, float shoulder, float bank)
{
    const float beyond = clearance - shoulder;
    if (beyond <= 0.0f) return 1.0f;
    if (!(bank > 0.0f) || beyond >= bank) return 0.0f;
    return terrainBrushWeight(beyond, bank);
}

std::size_t terrainFlattenAlongPath(Terrain& terrain, const TerrainPath& path,
        const TerrainPathBrush& brush)
{
    if (!terrain.isPrepared() || terrain.getAlignment() != TerrainAlignment::AlignXZ ||
            path.getSegmentCount() == 0)
        return 0;

    const int size = terrain.getSize();
    Math::Vec3 first;
    Math::Vec3 last;
    terrain.getPoint(0, 0, 0.0f, first);
    terrain.getPoint(static_cast<std::uint32_t>(size - 1), static_cast<std::uint32_t>(size - 1),
            0.0f, last);
    const Math::Vec3& origin = terrain.getPosition();
    Lattice lattice;
    lattice.width = size;
    lattice.height = size;
    lattice.x0 = first.x + origin.x;
    lattice.dx = (last.x - first.x) / static_cast<float>(size - 1);
    lattice.z0 = first.z + origin.z;
    lattice.dz = (last.z - first.z) / static_cast<float>(size - 1);

    const float bank = brush.bank > 0.0f ? brush.bank : 0.0f;
    const float reach = path.getMaxHalfWidth() + brush.shoulder + bank;
    FootprintField field;
    computeFootprint(path, lattice, reach, field);

    std::size_t written = 0;
    for (int y = 0; y < size; ++y)
    {
        for (int x = 0; x < size; ++x)
        {
            const std::size_t slot = static_cast<std::size_t>(y) * size + x;
            if (field.clearance[slot] == FLT_MAX) continue;
            const float weight = terrainPathWeight(field.clearance[slot], brush.shoulder, bank);
            if (!(weight > 0.0f)) continue;
            const float current = terrain.getHeightAtPoint(static_cast<std::uint32_t>(x),
                    static_cast<std::uint32_t>(y));
            const float target = field.height[slot] + brush.heightOffset - origin.y;
            const float value = current + (target - current) * weight;
            if (value == current) continue;
            terrain.setHeightAtPoint(static_cast<std::uint32_t>(x), static_cast<std::uint32_t>(y),
                    value);
            ++written;
        }
    }
    return written;
}

std::size_t terrainPaintAlongPath(Terrain& terrain, std::uint8_t layer, const TerrainPath& path,
        const TerrainPathBrush& brush)
{
    if (!terrain.isPrepared() || terrain.getAlignment() != TerrainAlignment::AlignXZ ||
            path.getSegmentCount() == 0)
        return 0;
    TerrainLayerBlendMap* map = terrain.getLayerBlendMap(layer);
    if (!map) return 0;

    const int size = terrain.getLayerBlendMapSize();
    Math::Vec3 first;
    Math::Vec3 last;
    map->convertUVToWorldSpace(0.0f, 0.0f, &first);
    map->convertUVToWorldSpace(1.0f, 1.0f, &last);
    Lattice lattice;
    lattice.width = size;
    lattice.height = size;
    const Math::Vec3& origin = terrain.getPosition();
    lattice.x0 = first.x + origin.x;
    lattice.dx = (last.x - first.x) / static_cast<float>(size - 1);
    lattice.z0 = first.z + origin.z;
    lattice.dz = (last.z - first.z) / static_cast<float>(size - 1);

    const float bank = brush.bank > 0.0f ? brush.bank : 0.0f;
    const float reach = path.getMaxHalfWidth() + brush.shoulder + bank;
    FootprintField field;
    computeFootprint(path, lattice, reach, field);

    const std::uint8_t layers = terrain.getLayerCount();
    std::size_t written = 0;
    for (int y = 0; y < size; ++y)
    {
        for (int x = 0; x < size; ++x)
        {
            const std::size_t slot = static_cast<std::size_t>(y) * size + x;
            if (field.clearance[slot] == FLT_MAX) continue;
            const float weight = terrainPathWeight(field.clearance[slot], brush.shoulder, bank);
            if (!(weight > 0.0f)) continue;
            const std::uint32_t ux = static_cast<std::uint32_t>(x);
            const std::uint32_t uy = static_cast<std::uint32_t>(y);
            const float current = map->getBlendValue(ux, uy);
            map->setBlendValue(ux, uy, current + (1.0f - current) * weight);
            for (std::uint8_t above = static_cast<std::uint8_t>(layer + 1); above < layers; ++above)
            {
                TerrainLayerBlendMap* other = terrain.getLayerBlendMap(above);
                if (other)
                    other->setBlendValue(ux, uy, other->getBlendValue(ux, uy) * (1.0f - weight));
            }
            ++written;
        }
    }
    map->update();
    for (std::uint8_t above = static_cast<std::uint8_t>(layer + 1); above < layers; ++above)
    {
        TerrainLayerBlendMap* other = terrain.getLayerBlendMap(above);
        if (other) other->update();
    }
    return written;
}

float terrainBrushWeight(float distance, float radius)
{
    float weight = distance / radius;
    weight = weight < 1.0f ? weight : 1.0f;
    return 1.0f - weight * weight;
}

bool terrainBrushHeight(Terrain& terrain, const Math::Vec3& worldCentre, float radius,
        float strength, float timeElapsed, bool lower)
{
    if (!terrain.isPrepared() || !(radius > 0.0f)) return false;

    Math::Vec3 tsPos;
    terrain.getTerrainPosition(worldCentre, tsPos);
    const float brushSizeTerrainSpace = 2.0f * radius / terrain.getWorldSize();
    const float terrainSize = static_cast<float>(terrain.getSize() - 1);
    int startx = static_cast<int>((tsPos.x - brushSizeTerrainSpace) * terrainSize);
    int starty = static_cast<int>((tsPos.y - brushSizeTerrainSpace) * terrainSize);
    int endx = static_cast<int>((tsPos.x + brushSizeTerrainSpace) * terrainSize);
    int endy = static_cast<int>((tsPos.y + brushSizeTerrainSpace) * terrainSize);
    startx = startx > 0 ? startx : 0;
    starty = starty > 0 ? starty : 0;
    endx = endx < static_cast<int>(terrainSize) ? endx : static_cast<int>(terrainSize);
    endy = endy < static_cast<int>(terrainSize) ? endy : static_cast<int>(terrainSize);

    bool changed = false;
    for (int y = starty; y <= endy; ++y)
    {
        for (int x = startx; x <= endx; ++x)
        {
            const float tsXdist = (static_cast<float>(x) / terrainSize) - tsPos.x;
            const float tsYdist = (static_cast<float>(y) / terrainSize) - tsPos.y;

            float weight =
                    sqrtf(tsYdist * tsYdist + tsXdist * tsXdist) / (0.5f * brushSizeTerrainSpace);
            weight = weight < 1.0f ? weight : 1.0f;
            weight = 1.0f - (weight * weight);
            if (!(weight > 0.0f)) continue;

            const float addedHeight = weight * strength * timeElapsed;
            const float height = terrain.getHeightAtPoint(static_cast<std::uint32_t>(x),
                    static_cast<std::uint32_t>(y));
            terrain.setHeightAtPoint(static_cast<std::uint32_t>(x), static_cast<std::uint32_t>(y),
                    lower ? height - addedHeight : height + addedHeight);
            changed = true;
        }
    }
    return changed;
}

bool terrainBrushBlend(Terrain& terrain, std::uint8_t layer, const Math::Vec3& worldCentre,
        float radius, float strength, float timeElapsed, bool remove)
{
    if (!terrain.isPrepared() || !(radius > 0.0f)) return false;
    TerrainLayerBlendMap* map = terrain.getLayerBlendMap(layer);
    if (!map) return false;

    Math::Vec3 tsPos;
    terrain.getTerrainPosition(worldCentre, tsPos);
    const float brushSizeTerrainSpace = 2.0f * radius / terrain.getWorldSize();
    const float imgSize = static_cast<float>(terrain.getLayerBlendMapSize());
    int startx = static_cast<int>((tsPos.x - brushSizeTerrainSpace) * imgSize);
    int starty = static_cast<int>((tsPos.y - brushSizeTerrainSpace) * imgSize);
    int endx = static_cast<int>((tsPos.x + brushSizeTerrainSpace) * imgSize);
    int endy = static_cast<int>((tsPos.y + brushSizeTerrainSpace) * imgSize);
    startx = startx > 0 ? startx : 0;
    starty = starty > 0 ? starty : 0;
    endx = endx < static_cast<int>(imgSize) ? endx : static_cast<int>(imgSize);
    endy = endy < static_cast<int>(imgSize) ? endy : static_cast<int>(imgSize);

    bool changed = false;
    for (int y = starty; y <= endy; ++y)
    {
        for (int x = startx; x <= endx; ++x)
        {
            const float tsXdist = (static_cast<float>(x) / imgSize) - tsPos.x;
            const float tsYdist = (static_cast<float>(y) / imgSize) - tsPos.y;

            float weight =
                    sqrtf(tsYdist * tsYdist + tsXdist * tsXdist) / (0.5f * brushSizeTerrainSpace);
            weight = weight < 1.0f ? weight : 1.0f;
            weight = 1.0f - (weight * weight);
            if (!(weight > 0.0f)) continue;

            const float paint = weight * strength * timeElapsed;
            const int imgY = static_cast<int>(imgSize) - y;
            if (x >= static_cast<int>(imgSize) || imgY >= static_cast<int>(imgSize)) continue;
            float val = map->getBlendValue(static_cast<std::uint32_t>(x),
                                static_cast<std::uint32_t>(imgY)) +
                        (remove ? -paint : paint);
            val = clamp01(val);
            map->setBlendValue(static_cast<std::uint32_t>(x), static_cast<std::uint32_t>(imgY),
                    val);
            changed = true;
        }
    }

    map->update();
    return changed;
}

bool terrainBrushColour(Terrain& terrain, const Math::Vec3& worldCentre, float radius,
        const Math::Vec3& colour, float strength, float timeElapsed)
{
    if (!terrain.isPrepared() || !(radius > 0.0f) || !terrain.getGlobalColourMapEnabled())
        return false;
    std::uint8_t* pixels = terrain.globalColourMapData();
    if (!pixels) return false;

    Math::Vec3 tsPos;
    terrain.getTerrainPosition(worldCentre, tsPos);
    const float brushSizeTerrainSpace = 2.0f * radius / terrain.getWorldSize();
    const int size = terrain.getGlobalColourMapSize();
    const float imgSize = static_cast<float>(size);
    int startx = static_cast<int>((tsPos.x - brushSizeTerrainSpace) * imgSize);
    int starty = static_cast<int>((tsPos.y - brushSizeTerrainSpace) * imgSize);
    int endx = static_cast<int>((tsPos.x + brushSizeTerrainSpace) * imgSize);
    int endy = static_cast<int>((tsPos.y + brushSizeTerrainSpace) * imgSize);
    startx = startx > 0 ? startx : 0;
    starty = starty > 0 ? starty : 0;
    endx = endx < size ? endx : size;
    endy = endy < size ? endy : size;

    const float target[3] = { clamp01(colour.x) * 255.0f, clamp01(colour.y) * 255.0f,
        clamp01(colour.z) * 255.0f };
    bool changed = false;
    int minX = size, minY = size, maxX = -1, maxY = -1;
    for (int y = starty; y <= endy; ++y)
    {
        for (int x = startx; x <= endx; ++x)
        {
            const float tsXdist = (static_cast<float>(x) / imgSize) - tsPos.x;
            const float tsYdist = (static_cast<float>(y) / imgSize) - tsPos.y;
            float weight =
                    sqrtf(tsYdist * tsYdist + tsXdist * tsXdist) / (0.5f * brushSizeTerrainSpace);
            weight = weight < 1.0f ? weight : 1.0f;
            weight = 1.0f - (weight * weight);
            if (!(weight > 0.0f)) continue;
            const float amount = clamp01(weight * strength * timeElapsed);
            const int imgY = size - y;
            if (x >= size || imgY >= size || imgY < 0) continue;
            std::uint8_t* pixel = &pixels[(static_cast<std::size_t>(imgY) * size + x) * 3];
            for (int c = 0; c < 3; ++c)
            {
                const float value = static_cast<float>(pixel[c]) +
                                    (target[c] - static_cast<float>(pixel[c])) * amount;
                const std::uint8_t rounded = static_cast<std::uint8_t>(value + 0.5f);
                if (rounded != pixel[c])
                {
                    pixel[c] = rounded;
                    changed = true;
                }
            }
            if (x < minX) minX = x;
            if (x > maxX) maxX = x;
            if (imgY < minY) minY = imgY;
            if (imgY > maxY) maxY = imgY;
        }
    }
    if (changed) terrain.dirtyGlobalColourMapRect(TerrainRect(minX, minY, maxX + 1, maxY + 1));
    return changed;
}

bool terrainBrushSmooth(Terrain& terrain, const Math::Vec3& worldCentre, float radius,
        float strength, float timeElapsed)
{
    if (!terrain.isPrepared() || !(radius > 0.0f)) return false;
    const float amount = clamp01(strength * timeElapsed);
    if (!(amount > 0.0f)) return false;

    const Footprint area = footprint(terrain, worldCentre, radius);
    const int size = terrain.getSize();
    const int marginX0 = area.minX > 0 ? area.minX - 1 : 0;
    const int marginY0 = area.minY > 0 ? area.minY - 1 : 0;
    const int marginX1 = area.maxX < size - 1 ? area.maxX + 1 : size - 1;
    const int marginY1 = area.maxY < size - 1 ? area.maxY + 1 : size - 1;
    const int width = marginX1 - marginX0 + 1;
    ct::Vector<float> source;
    source.resize(static_cast<std::size_t>(width) * (marginY1 - marginY0 + 1));
    for (int y = marginY0; y <= marginY1; ++y)
        for (int x = marginX0; x <= marginX1; ++x)
            source[static_cast<std::size_t>(y - marginY0) * width + (x - marginX0)] =
                    terrain.getHeightAtPoint(static_cast<std::uint32_t>(x),
                            static_cast<std::uint32_t>(y));

    bool changed = false;
    for (int y = area.minY; y <= area.maxY; ++y)
    {
        for (int x = area.minX; x <= area.maxX; ++x)
        {
            const float dx = (static_cast<float>(x) - area.centreX) * area.scale;
            const float dy = (static_cast<float>(y) - area.centreY) * area.scale;
            const float distance = sqrtf(dx * dx + dy * dy);
            if (distance > radius) continue;
            const float falloff = 1.0f - smoothStep(0.0f, radius, distance);

            float average = 0.0f;
            int samples = 0;
            for (int oy = -1; oy <= 1; ++oy)
            {
                for (int ox = -1; ox <= 1; ++ox)
                {
                    int sx = x + ox;
                    int sy = y + oy;
                    sx = sx < 0 ? 0 : (sx > size - 1 ? size - 1 : sx);
                    sy = sy < 0 ? 0 : (sy > size - 1 ? size - 1 : sy);
                    average += source[static_cast<std::size_t>(sy - marginY0) * width +
                                      (sx - marginX0)];
                    ++samples;
                }
            }
            const float current =
                    source[static_cast<std::size_t>(y - marginY0) * width + (x - marginX0)];
            const float t = amount * falloff;
            terrain.setHeightAtPoint(static_cast<std::uint32_t>(x), static_cast<std::uint32_t>(y),
                    current + (average / static_cast<float>(samples) - current) * t);
            changed = true;
        }
    }
    return changed;
}

bool terrainBrushFlatten(Terrain& terrain, const Math::Vec3& worldCentre, float radius,
        float targetHeight, float strength, float timeElapsed)
{
    if (!terrain.isPrepared() || !(radius > 0.0f)) return false;
    const float amount = clamp01(strength * timeElapsed);
    if (!(amount > 0.0f)) return false;

    const Footprint area = footprint(terrain, worldCentre, radius);
    bool changed = false;
    for (int y = area.minY; y <= area.maxY; ++y)
    {
        for (int x = area.minX; x <= area.maxX; ++x)
        {
            const float dx = (static_cast<float>(x) - area.centreX) * area.scale;
            const float dy = (static_cast<float>(y) - area.centreY) * area.scale;
            const float distance = sqrtf(dx * dx + dy * dy);
            if (distance > radius) continue;
            const float falloff = 1.0f - smoothStep(0.0f, radius, distance);
            const float height = terrain.getHeightAtPoint(static_cast<std::uint32_t>(x),
                    static_cast<std::uint32_t>(y));
            const float t = clamp01(amount * falloff);
            terrain.setHeightAtPoint(static_cast<std::uint32_t>(x), static_cast<std::uint32_t>(y),
                    height + (targetHeight - height) * t);
            changed = true;
        }
    }
    return changed;
}

} // namespace prisma3d
