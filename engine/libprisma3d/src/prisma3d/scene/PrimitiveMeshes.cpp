#include "prisma3d/scene/PrimitiveMeshes.h"

#include "prisma3d/core/FileSystem.h"
#include "prisma3d/core/Pixmap.h"

#include "platform.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace prisma3d
{

namespace
{

void addVertex(SurfaceData& out, const Math::Vec3& position, const Math::Vec3& normal,
        const Math::Vec4& tangent, const Math::Vec2& uv)
{
    out.positions.push_back(position);
    out.normals.push_back(normal);
    out.tangents.push_back(tangent);
    out.uvs.push_back(uv);
}

void addQuadIndices(SurfaceData& out, int prevRow, int thisRow, int i2)
{
    const int first[6] = { prevRow + i2 - 2, thisRow + i2 - 2, prevRow + i2, prevRow + i2,
        thisRow + i2 - 2, thisRow + i2 };
    const int second[6] = { prevRow + i2 - 1, thisRow + i2 - 1, prevRow + i2 + 1, prevRow + i2 + 1,
        thisRow + i2 - 1, thisRow + i2 + 1 };
    for (int k = 0; k < 6; ++k) out.indices.push_back(static_cast<std::uint32_t>(first[k]));
    for (int k = 0; k < 6; ++k) out.indices.push_back(static_cast<std::uint32_t>(second[k]));
}

Math::Vec4 revolveTangent(float theta)
{
    return Math::Vec4(-sinf(theta), 0.0f, cosf(theta), 1.0f);
}

void appendGridIndices(SurfaceData& out, int cellsX, int cellsZ, int stride)
{
    for (int j = 0; j < cellsZ; ++j)
    {
        for (int i = 0; i < cellsX; ++i)
        {
            const int a = j * stride + i;
            const int b = a + 1;
            const int c = a + stride;
            const int d = c + 1;
            const int quad[6] = { a, c, b, b, c, d };
            for (int k = 0; k < 6; ++k) out.indices.push_back(static_cast<std::uint32_t>(quad[k]));
        }
    }
}

void appendRingIndices(SurfaceData& out, int rows, int slices, const int (&order)[6])
{
    for (int r = 0; r < rows; ++r)
    {
        for (int s = 0; s < slices; ++s)
        {
            const int a = r * (slices + 1) + s;
            const int b = a + slices + 1;
            const int corner[4] = { a, a + 1, b, b + 1 };
            for (int k = 0; k < 6; ++k)
                out.indices.push_back(static_cast<std::uint32_t>(corner[order[k]]));
        }
    }
}

void buildTube(SurfaceData& out, float radius, float height, int slices, float topScale)
{
    const float slope = height > 0.0f ? (radius - radius * topScale) / height : 0.0f;
    for (int cap = 0; cap <= 1; ++cap)
    {
        const float y = cap ? height : 0.0f;
        const float r = cap ? radius * topScale : radius;
        for (int s = 0; s <= slices; ++s)
        {
            const float u = static_cast<float>(s) / static_cast<float>(slices);
            const float theta = u * 2.0f * Math::PI;
            const float cx = cosf(theta);
            const float cz = sinf(theta);
            addVertex(out, Math::Vec3(cx * r, y, cz * r), Math::Vec3(cx, slope, cz).Normalized(),
                    revolveTangent(theta), Math::Vec2(u, static_cast<float>(cap)));
        }
    }
    for (int s = 0; s < slices; ++s)
    {
        const int a = s;
        const int b = s + slices + 1;
        const int quad[6] = { a, b, a + 1, a + 1, b, b + 1 };
        for (int k = 0; k < 6; ++k) out.indices.push_back(static_cast<std::uint32_t>(quad[k]));
    }

    for (int cap = 0; cap <= 1; ++cap)
    {
        const float r = cap ? radius * topScale : radius;
        if (r < 1e-5f) continue;
        const float y = cap ? height : 0.0f;
        const float ny = cap ? 1.0f : -1.0f;
        const int center = static_cast<int>(out.positions.size());
        addVertex(out, Math::Vec3(0.0f, y, 0.0f), Math::Vec3(0.0f, ny, 0.0f),
                Math::Vec4(1.0f, 0.0f, 0.0f, 1.0f), Math::Vec2(0.5f, 0.5f));
        for (int s = 0; s <= slices; ++s)
        {
            const float theta =
                    static_cast<float>(s) / static_cast<float>(slices) * 2.0f * Math::PI;
            addVertex(out, Math::Vec3(cosf(theta) * r, y, sinf(theta) * r),
                    Math::Vec3(0.0f, ny, 0.0f), revolveTangent(theta),
                    Math::Vec2(cosf(theta) * 0.5f + 0.5f, sinf(theta) * 0.5f + 0.5f));
        }
        for (int s = 0; s < slices; ++s)
        {
            const int a = center + 1 + s;
            const int b = a + 1;
            out.indices.push_back(static_cast<std::uint32_t>(center));
            out.indices.push_back(static_cast<std::uint32_t>(cap ? b : a));
            out.indices.push_back(static_cast<std::uint32_t>(cap ? a : b));
        }
    }
}

void addCapsuleRing(SurfaceData& out, int slices, float v, float y, float ringRadius, float ny,
        float nRingScale)
{
    for (int s = 0; s <= slices; ++s)
    {
        const float u = static_cast<float>(s) / static_cast<float>(slices);
        const float theta = u * 2.0f * Math::PI;
        const float cx = cosf(theta);
        const float cz = sinf(theta);
        addVertex(out, Math::Vec3(cx * ringRadius, y, cz * ringRadius),
                Math::Vec3(cx * nRingScale, ny, cz * nRingScale).Normalized(),
                revolveTangent(theta), Math::Vec2(u, v));
    }
}

float sampleHeightmap(const Pixmap& image, float u, float v, float scale)
{
    const float cu = u < 0.0f ? 0.0f : (u > 1.0f ? 1.0f : u);
    const float cv = v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v);
    const float px = cu * static_cast<float>(image.width - 1);
    const float pz = cv * static_cast<float>(image.height - 1);
    const int x0 = static_cast<int>(px);
    const int z0 = static_cast<int>(pz);
    const int x1 = x0 + 1 < image.width ? x0 + 1 : x0;
    const int z1 = z0 + 1 < image.height ? z0 + 1 : z0;
    const float fx = px - static_cast<float>(x0);
    const float fz = pz - static_cast<float>(z0);

    const unsigned char* pixels = image.pixels;
    const int stride = image.components;
    const float h00 = static_cast<float>(pixels[(z0 * image.width + x0) * stride]);
    const float h10 = static_cast<float>(pixels[(z0 * image.width + x1) * stride]);
    const float h01 = static_cast<float>(pixels[(z1 * image.width + x0) * stride]);
    const float h11 = static_cast<float>(pixels[(z1 * image.width + x1) * stride]);
    const float top = h00 + (h10 - h00) * fx;
    const float bottom = h01 + (h11 - h01) * fx;
    return (top + (bottom - top) * fz) / 255.0f * scale;
}

bool heightmapUsable(const Pixmap& image)
{
    return image.pixels != nullptr && image.width > 0 && image.height > 0 && image.components > 0;
}

MeshId createFromSurface(MeshStore& meshes, const SurfaceData& surface, const char* name)
{
    const MeshId mesh = meshes.create(name);
    if (!mesh.valid()) return mesh;
    meshes.addSurface(mesh, surface, name);
    return mesh;
}

} // namespace

void boxMeshArrays(SurfaceData& out, const Math::Vec3& size, int subdivideWidth,
        int subdivideHeight, int subdivideDepth)
{
    out = SurfaceData();

    int i, j, prevRow, thisRow, point;
    float x, y, z;
    const float oneThird = 1.0f / 3.0f;
    const float twoThirds = 2.0f / 3.0f;

    const Math::Vec3 startPos = size * -0.5f;

    const std::size_t numPoints = static_cast<std::size_t>(subdivideHeight + 2) *
                                  static_cast<std::size_t>(subdivideWidth + 2) * 6;
    out.positions.reserve(numPoints);
    out.normals.reserve(numPoints);
    out.tangents.reserve(numPoints);
    out.uvs.reserve(numPoints);
    out.indices.reserve(static_cast<std::size_t>(subdivideHeight + 1) *
                        static_cast<std::size_t>(subdivideWidth + 1) * 6 * 6);
    point = 0;

    y = startPos.y;
    thisRow = point;
    prevRow = 0;
    for (j = 0; j <= subdivideHeight + 1; j++)
    {
        float v = static_cast<float>(j);
        v /= (2.0f * (subdivideHeight + 1.0f));

        x = startPos.x;
        for (i = 0; i <= subdivideWidth + 1; i++)
        {
            float u = static_cast<float>(i);
            u /= (3.0f * (subdivideWidth + 1.0f));

            addVertex(out, Math::Vec3(x, -y, -startPos.z), Math::Vec3(0.0f, 0.0f, 1.0f),
                    Math::Vec4(1.0f, 0.0f, 0.0f, 1.0f), Math::Vec2(u, v));
            point++;

            addVertex(out, Math::Vec3(-x, -y, startPos.z), Math::Vec3(0.0f, 0.0f, -1.0f),
                    Math::Vec4(-1.0f, 0.0f, 0.0f, 1.0f), Math::Vec2(twoThirds + u, v));
            point++;

            if (i > 0 && j > 0) addQuadIndices(out, prevRow, thisRow, i * 2);

            x += size.x / (subdivideWidth + 1.0f);
        }

        y += size.y / (subdivideHeight + 1.0f);
        prevRow = thisRow;
        thisRow = point;
    }

    y = startPos.y;
    thisRow = point;
    prevRow = 0;
    for (j = 0; j <= (subdivideHeight + 1); j++)
    {
        float v = static_cast<float>(j);
        v /= (2.0f * (subdivideHeight + 1.0f));

        z = startPos.z;
        for (i = 0; i <= (subdivideDepth + 1); i++)
        {
            float u = static_cast<float>(i);
            u /= (3.0f * (subdivideDepth + 1.0f));

            addVertex(out, Math::Vec3(-startPos.x, -y, -z), Math::Vec3(1.0f, 0.0f, 0.0f),
                    Math::Vec4(0.0f, 0.0f, -1.0f, 1.0f), Math::Vec2(oneThird + u, v));
            point++;

            addVertex(out, Math::Vec3(startPos.x, -y, z), Math::Vec3(-1.0f, 0.0f, 0.0f),
                    Math::Vec4(0.0f, 0.0f, 1.0f, 1.0f), Math::Vec2(u, 0.5f + v));
            point++;

            if (i > 0 && j > 0) addQuadIndices(out, prevRow, thisRow, i * 2);

            z += size.z / (subdivideDepth + 1.0f);
        }

        y += size.y / (subdivideHeight + 1.0f);
        prevRow = thisRow;
        thisRow = point;
    }

    z = startPos.z;
    thisRow = point;
    prevRow = 0;
    for (j = 0; j <= (subdivideDepth + 1); j++)
    {
        float v = static_cast<float>(j);
        v /= (2.0f * (subdivideDepth + 1.0f));

        x = startPos.x;
        for (i = 0; i <= (subdivideWidth + 1); i++)
        {
            float u = static_cast<float>(i);
            u /= (3.0f * (subdivideWidth + 1.0f));

            addVertex(out, Math::Vec3(-x, -startPos.y, -z), Math::Vec3(0.0f, 1.0f, 0.0f),
                    Math::Vec4(-1.0f, 0.0f, 0.0f, 1.0f), Math::Vec2(oneThird + u, 0.5f + v));
            point++;

            addVertex(out, Math::Vec3(x, startPos.y, -z), Math::Vec3(0.0f, -1.0f, 0.0f),
                    Math::Vec4(1.0f, 0.0f, 0.0f, 1.0f), Math::Vec2(twoThirds + u, 0.5f + v));
            point++;

            if (i > 0 && j > 0) addQuadIndices(out, prevRow, thisRow, i * 2);

            x += size.x / (subdivideWidth + 1.0f);
        }

        z += size.z / (subdivideDepth + 1.0f);
        prevRow = thisRow;
        thisRow = point;
    }
}

void planeMeshArrays(SurfaceData& out, float width, float depth, int segmentsX, int segmentsZ,
        float uvTiles)
{
    out = SurfaceData();
    const int segX = segmentsX < 1 ? 1 : segmentsX;
    const int segZ = segmentsZ < 1 ? 1 : segmentsZ;
    const int nx = segX + 1;
    const int nz = segZ + 1;
    const float x0 = -width * 0.5f;
    const float z0 = -depth * 0.5f;
    const float dx = width / static_cast<float>(segX);
    const float dz = depth / static_cast<float>(segZ);

    const std::size_t count = static_cast<std::size_t>(nx) * static_cast<std::size_t>(nz);
    out.positions.reserve(count);
    out.normals.reserve(count);
    out.tangents.reserve(count);
    out.uvs.reserve(count);
    out.indices.reserve(static_cast<std::size_t>(segX) * static_cast<std::size_t>(segZ) * 6);
    for (int j = 0; j < nz; ++j)
    {
        for (int i = 0; i < nx; ++i)
        {
            addVertex(out,
                    Math::Vec3(x0 + static_cast<float>(i) * dx, 0.0f,
                            z0 + static_cast<float>(j) * dz),
                    Math::Vec3(0.0f, 1.0f, 0.0f), Math::Vec4(1.0f, 0.0f, 0.0f, 1.0f),
                    Math::Vec2(static_cast<float>(i) / static_cast<float>(segX),
                            static_cast<float>(j) / static_cast<float>(segZ)) *
                            uvTiles);
        }
    }
    appendGridIndices(out, segX, segZ, nx);
}

void sphereMeshArrays(SurfaceData& out, float radius, int rings, int slices)
{
    out = SurfaceData();
    rings = rings < 2 ? 2 : rings;
    slices = slices < 3 ? 3 : slices;

    const std::size_t count =
            static_cast<std::size_t>(rings + 1) * static_cast<std::size_t>(slices + 1);
    out.positions.reserve(count);
    out.normals.reserve(count);
    out.tangents.reserve(count);
    out.uvs.reserve(count);
    out.indices.reserve(static_cast<std::size_t>(rings) * static_cast<std::size_t>(slices) * 6);
    for (int r = 0; r <= rings; ++r)
    {
        const float v = static_cast<float>(r) / static_cast<float>(rings);
        const float phi = v * Math::PI;
        for (int s = 0; s <= slices; ++s)
        {
            const float u = static_cast<float>(s) / static_cast<float>(slices);
            const float theta = u * 2.0f * Math::PI;
            const Math::Vec3 n(sinf(phi) * cosf(theta), cosf(phi), sinf(phi) * sinf(theta));
            addVertex(out, n * radius, n, revolveTangent(theta), Math::Vec2(u, v));
        }
    }
    const int order[6] = { 0, 1, 2, 1, 3, 2 };
    appendRingIndices(out, rings, slices, order);
}

void cylinderMeshArrays(SurfaceData& out, float radius, float height, int slices)
{
    out = SurfaceData();
    slices = slices < 3 ? 3 : slices;
    buildTube(out, radius, height, slices, 1.0f);
}

void coneMeshArrays(SurfaceData& out, float radius, float height, int slices)
{
    out = SurfaceData();
    slices = slices < 3 ? 3 : slices;
    buildTube(out, radius, height, slices, 0.0f);
}

void capsuleMeshArrays(SurfaceData& out, float radius, float height, int rings, int slices)
{
    out = SurfaceData();
    rings = rings < 2 ? 2 : rings;
    slices = slices < 3 ? 3 : slices;

    const int totalRings = rings * 2 + 2;
    const float vScale = 1.0f / static_cast<float>(totalRings - 1);
    const std::size_t count =
            static_cast<std::size_t>(totalRings) * static_cast<std::size_t>(slices + 1);
    out.positions.reserve(count);
    out.normals.reserve(count);
    out.tangents.reserve(count);
    out.uvs.reserve(count);
    out.indices.reserve(
            static_cast<std::size_t>(totalRings - 1) * static_cast<std::size_t>(slices) * 6);

    int row = 0;
    for (int r = 0; r <= rings; ++r)
    {
        const float phi =
                Math::PI * (1.0f - 0.5f * static_cast<float>(r) / static_cast<float>(rings));
        addCapsuleRing(out, slices, static_cast<float>(row) * vScale, radius + radius * cosf(phi),
                radius * sinf(phi), cosf(phi), sinf(phi));
        ++row;
    }
    addCapsuleRing(out, slices, static_cast<float>(row) * vScale, radius + height, radius, 0.0f,
            1.0f);
    ++row;
    for (int r = 1; r <= rings; ++r)
    {
        const float phi =
                Math::PI * 0.5f * (1.0f - static_cast<float>(r) / static_cast<float>(rings));
        addCapsuleRing(out, slices, static_cast<float>(row) * vScale,
                radius + height + radius * cosf(phi), radius * sinf(phi), cosf(phi), sinf(phi));
        ++row;
    }
    const int order[6] = { 0, 2, 1, 1, 2, 3 };
    appendRingIndices(out, totalRings - 1, slices, order);
}

void torusMeshArrays(SurfaceData& out, float majorRadius, float minorRadius, int majorSegments,
        int minorSegments)
{
    out = SurfaceData();
    majorSegments = majorSegments < 3 ? 3 : majorSegments;
    minorSegments = minorSegments < 3 ? 3 : minorSegments;

    const std::size_t count = static_cast<std::size_t>(majorSegments + 1) *
                              static_cast<std::size_t>(minorSegments + 1);
    out.positions.reserve(count);
    out.normals.reserve(count);
    out.tangents.reserve(count);
    out.uvs.reserve(count);
    out.indices.reserve(
            static_cast<std::size_t>(majorSegments) * static_cast<std::size_t>(minorSegments) * 6);
    for (int i = 0; i <= majorSegments; ++i)
    {
        const float u = static_cast<float>(i) / static_cast<float>(majorSegments);
        const float theta = u * 2.0f * Math::PI;
        const float rx = cosf(theta);
        const float rz = sinf(theta);
        const Math::Vec3 center(rx * majorRadius, 0.0f, rz * majorRadius);
        for (int j = 0; j <= minorSegments; ++j)
        {
            const float v = static_cast<float>(j) / static_cast<float>(minorSegments);
            const float phi = v * 2.0f * Math::PI;
            const Math::Vec3 n(rx * cosf(phi), sinf(phi), rz * cosf(phi));
            addVertex(out, center + n * minorRadius, n, revolveTangent(theta), Math::Vec2(u, v));
        }
    }
    const int order[6] = { 0, 1, 2, 1, 3, 2 };
    appendRingIndices(out, majorSegments, minorSegments, order);
}

void hillsPlaneMeshArrays(SurfaceData& out, float width, float depth, int segmentsX, int segmentsZ,
        const Pixmap& heightmap, float heightScale, float uvTiles)
{
    out = SurfaceData();
    if (!heightmapUsable(heightmap)) return;
    const int segX = segmentsX < 1 ? 1 : segmentsX;
    const int segZ = segmentsZ < 1 ? 1 : segmentsZ;
    const int nx = segX + 1;
    const int nz = segZ + 1;
    const float x0 = -width * 0.5f;
    const float z0 = -depth * 0.5f;
    const float dx = width / static_cast<float>(segX);
    const float dz = depth / static_cast<float>(segZ);

    const std::size_t count = static_cast<std::size_t>(nx) * static_cast<std::size_t>(nz);
    out.positions.reserve(count);
    out.normals.reserve(count);
    out.tangents.reserve(count);
    out.uvs.reserve(count);
    out.indices.reserve(static_cast<std::size_t>(segX) * static_cast<std::size_t>(segZ) * 6);
    for (int j = 0; j < nz; ++j)
    {
        for (int i = 0; i < nx; ++i)
        {
            const float wx = x0 + static_cast<float>(i) * dx;
            const float wz = z0 + static_cast<float>(j) * dz;
            const float h =
                    sampleHeightmap(heightmap, (wx - x0) / width, (wz - z0) / depth, heightScale);
            out.positions.push_back(Math::Vec3(wx, h, wz));
            out.normals.push_back(Math::Vec3(0.0f, 1.0f, 0.0f));
            out.tangents.push_back(Math::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
            out.uvs.push_back(Math::Vec2(static_cast<float>(i) / static_cast<float>(segX),
                                      static_cast<float>(j) / static_cast<float>(segZ)) *
                              uvTiles);
        }
    }
    for (int j = 0; j < nz; ++j)
    {
        for (int i = 0; i < nx; ++i)
        {
            const std::size_t v = static_cast<std::size_t>(j) * static_cast<std::size_t>(nx) +
                                  static_cast<std::size_t>(i);
            const Math::Vec3 p = out.positions[v];
            const float hl = sampleHeightmap(heightmap, (p.x - dx - x0) / width, (p.z - z0) / depth,
                    heightScale);
            const float hr = sampleHeightmap(heightmap, (p.x + dx - x0) / width, (p.z - z0) / depth,
                    heightScale);
            const float hd = sampleHeightmap(heightmap, (p.x - x0) / width, (p.z - dz - z0) / depth,
                    heightScale);
            const float hu = sampleHeightmap(heightmap, (p.x - x0) / width, (p.z + dz - z0) / depth,
                    heightScale);
            out.normals[v] = Math::Vec3(hl - hr, 2.0f * ((dx + dz) * 0.5f), hd - hu).Normalized();
            out.tangents[v] = Math::Vec4(Math::Vec3(2.0f * dx, hr - hl, 0.0f).Normalized(), 1.0f);
        }
    }
    appendGridIndices(out, segX, segZ, nx);
}

void heightfieldMeshArrays(SurfaceData& out, const float* heights, int width, int height,
        float cellSize, float uvTiles)
{
    out = SurfaceData();
    if (heights == nullptr || width < 2 || height < 2) return;

    const std::size_t count = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
    out.positions.reserve(count);
    out.normals.reserve(count);
    out.tangents.reserve(count);
    out.uvs.reserve(count);
    out.indices.reserve(
            static_cast<std::size_t>(width - 1) * static_cast<std::size_t>(height - 1) * 6);
    for (int j = 0; j < height; ++j)
    {
        for (int i = 0; i < width; ++i)
        {
            const std::size_t v = static_cast<std::size_t>(j) * static_cast<std::size_t>(width) +
                                  static_cast<std::size_t>(i);
            const float hl = heights[j * width + (i > 0 ? i - 1 : i)];
            const float hr = heights[j * width + (i < width - 1 ? i + 1 : i)];
            const float hd = heights[(j > 0 ? j - 1 : j) * width + i];
            const float hu = heights[(j < height - 1 ? j + 1 : j) * width + i];
            addVertex(out,
                    Math::Vec3(static_cast<float>(i) * cellSize, heights[v],
                            static_cast<float>(j) * cellSize),
                    Math::Vec3(hl - hr, 2.0f * cellSize, hd - hu).Normalized(),
                    Math::Vec4(Math::Vec3(2.0f * cellSize, hr - hl, 0.0f).Normalized(), 1.0f),
                    Math::Vec2(static_cast<float>(i) / static_cast<float>(width - 1),
                            static_cast<float>(j) / static_cast<float>(height - 1)) *
                            uvTiles);
        }
    }
    appendGridIndices(out, width - 1, height - 1, width);
}

void heightfieldMeshArrays(SurfaceData& out, const Pixmap& heightmap, float cellSize,
        float heightScale, float uvTiles)
{
    out = SurfaceData();
    if (!heightmapUsable(heightmap)) return;
    const std::size_t count =
            static_cast<std::size_t>(heightmap.width) * static_cast<std::size_t>(heightmap.height);
    ct::Vector<float> world;
    world.reserve(count);
    for (std::size_t i = 0; i < count; ++i)
    {
        world.push_back(
                static_cast<float>(
                        heightmap.pixels[i * static_cast<std::size_t>(heightmap.components)]) /
                255.0f * heightScale);
    }
    heightfieldMeshArrays(out, world.data(), heightmap.width, heightmap.height, cellSize, uvTiles);
}


namespace
{

struct KindDef
{
    const char* name;
    bool heightmap;
    int count;
    PrimitiveParamDef params[8];
};

const float kMaxExtent = 1.0e7f;
const float kMaxCount = 4096.0f;
const float kMaxGridCells = 4194304.0f;
const char kPrimitivePrefix[] = "primitive:";

const KindDef kKinds[] = {
    { "box", false, 1, { { "size", PrimitiveParamType::Vec3, 1.0f, 0.0f, kMaxExtent } } },
    { "plane", false, 5,
        { { "width", PrimitiveParamType::Float, 1.0f, 0.0f, kMaxExtent },
            { "depth", PrimitiveParamType::Float, 1.0f, 0.0f, kMaxExtent },
            { "segments_x", PrimitiveParamType::Count, 1.0f, 1.0f, kMaxCount },
            { "segments_z", PrimitiveParamType::Count, 1.0f, 1.0f, kMaxCount },
            { "uv_tiles", PrimitiveParamType::Float, 1.0f, 0.0f, kMaxExtent } } },
    { "sphere", false, 3,
        { { "radius", PrimitiveParamType::Float, 0.5f, 0.0f, kMaxExtent },
            { "rings", PrimitiveParamType::Count, 16.0f, 2.0f, kMaxCount },
            { "slices", PrimitiveParamType::Count, 24.0f, 3.0f, kMaxCount } } },
    { "cylinder", false, 3,
        { { "radius", PrimitiveParamType::Float, 0.5f, 0.0f, kMaxExtent },
            { "height", PrimitiveParamType::Float, 1.0f, 0.0f, kMaxExtent },
            { "slices", PrimitiveParamType::Count, 24.0f, 3.0f, kMaxCount } } },
    { "cone", false, 3,
        { { "radius", PrimitiveParamType::Float, 0.5f, 0.0f, kMaxExtent },
            { "height", PrimitiveParamType::Float, 1.0f, 0.0f, kMaxExtent },
            { "slices", PrimitiveParamType::Count, 24.0f, 3.0f, kMaxCount } } },
    { "capsule", false, 4,
        { { "radius", PrimitiveParamType::Float, 0.5f, 0.0f, kMaxExtent },
            { "height", PrimitiveParamType::Float, 1.0f, 0.0f, kMaxExtent },
            { "rings", PrimitiveParamType::Count, 8.0f, 2.0f, kMaxCount },
            { "slices", PrimitiveParamType::Count, 24.0f, 3.0f, kMaxCount } } },
    { "torus", false, 4,
        { { "major_radius", PrimitiveParamType::Float, 1.0f, 0.0f, kMaxExtent },
            { "minor_radius", PrimitiveParamType::Float, 0.25f, 0.0f, kMaxExtent },
            { "major_segments", PrimitiveParamType::Count, 24.0f, 3.0f, kMaxCount },
            { "minor_segments", PrimitiveParamType::Count, 12.0f, 3.0f, kMaxCount } } },
    { "hills_plane", true, 6,
        { { "width", PrimitiveParamType::Float, 1.0f, 0.0f, kMaxExtent },
            { "depth", PrimitiveParamType::Float, 1.0f, 0.0f, kMaxExtent },
            { "segments_x", PrimitiveParamType::Count, 1.0f, 1.0f, kMaxCount },
            { "segments_z", PrimitiveParamType::Count, 1.0f, 1.0f, kMaxCount },
            { "height_scale", PrimitiveParamType::Float, 1.0f, -kMaxExtent, kMaxExtent },
            { "uv_tiles", PrimitiveParamType::Float, 1.0f, 0.0f, kMaxExtent } } },
    { "heightfield", true, 3,
        { { "cell_size", PrimitiveParamType::Float, 1.0f, 0.0f, kMaxExtent },
            { "height_scale", PrimitiveParamType::Float, 1.0f, -kMaxExtent, kMaxExtent },
            { "uv_tiles", PrimitiveParamType::Float, 1.0f, 0.0f, kMaxExtent } } },
};

const KindDef& kindDef(PrimitiveKind kind)
{
    return kKinds[static_cast<int>(kind)];
}

void appendFloat(ct::String& out, float value)
{
    char buffer[48];
    if (value == floorf(value) && fabsf(value) < 1.0e9f)
        snprintf(buffer, sizeof(buffer), "%.0f", static_cast<double>(value));
    else
    {
        for (int precision = 1; precision <= 9; ++precision)
        {
            snprintf(buffer, sizeof(buffer), "%.*g", precision, static_cast<double>(value));
            if (strtof(buffer, nullptr) == value) break;
        }
    }
    out += buffer;
}

void appendEncoded(ct::String& out, const ct::String& text)
{
    static const char digits[] = "0123456789ABCDEF";
    for (std::size_t i = 0; i < text.size(); ++i)
    {
        const unsigned char c = static_cast<unsigned char>(text[i]);
        if (c == '%' || c == '&' || c == '?' || c == '=' || c < 0x20)
        {
            out += '%';
            out += digits[c >> 4];
            out += digits[c & 15];
        }
        else
            out += static_cast<char>(c);
    }
}

int hexValue(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

bool decode(const ct::String& text, ct::String& out)
{
    out.clear();
    for (std::size_t i = 0; i < text.size(); ++i)
    {
        if (text[i] != '%')
        {
            out += text[i];
            continue;
        }
        if (i + 2 >= text.size()) return false;
        const int high = hexValue(text[i + 1]);
        const int low = hexValue(text[i + 2]);
        if (high < 0 || low < 0) return false;
        out += static_cast<char>(high * 16 + low);
        i += 2;
    }
    return true;
}

ct::String recipePath(const PrimitiveRecipe& recipe)
{
    const KindDef& def = kindDef(recipe.kind);
    ct::String path = kPrimitivePrefix;
    path += def.name;
    char separator = '?';
    int value = 0;
    int heightmapAt = def.heightmap ? (recipe.kind == PrimitiveKind::HillsPlane ? 4 : 0) : -1;
    for (int p = 0; p < def.count; ++p)
    {
        if (p == heightmapAt)
        {
            path += separator;
            separator = '&';
            path += "heightmap=";
            appendEncoded(path, recipe.heightmap);
        }
        path += separator;
        separator = '&';
        path += def.params[p].key;
        path += '=';
        if (def.params[p].type == PrimitiveParamType::Vec3)
        {
            for (int c = 0; c < 3; ++c)
            {
                if (c > 0) path += ',';
                appendFloat(path, recipe.values[value++]);
            }
        }
        else
            appendFloat(path, recipe.values[value++]);
    }
    return path;
}

ct::String describePath(const ct::String& path)
{
    ct::String text = "'";
    text += path;
    text += '\'';
    return text;
}

bool parseNumber(const ct::String& text, float& out)
{
    if (text.empty()) return false;
    char* end = nullptr;
    const float value = strtof(text.c_str(), &end);
    if (end == text.c_str() || *end != '\0') return false;
    if (value != value || value == INFINITY || value == -INFINITY) return false;
    out = value;
    return true;
}

void applyDefaults(PrimitiveRecipe& recipe)
{
    const KindDef& def = kindDef(recipe.kind);
    int value = 0;
    for (int p = 0; p < def.count; ++p)
    {
        if (def.params[p].type == PrimitiveParamType::Vec3)
        {
            for (int c = 0; c < 3; ++c) recipe.values[value++] = def.params[p].fallback;
        }
        else
            recipe.values[value++] = def.params[p].fallback;
    }
    recipe.heightmap.clear();
}

bool parseRecipe(const ct::String& path, PrimitiveRecipe& recipe, ct::String& error)
{
    error.clear();
    if (!path.starts_with(kPrimitivePrefix))
    {
        error = ct::String("not a primitive mesh path: ") + describePath(path);
        return false;
    }
    const std::size_t prefixLength = sizeof(kPrimitivePrefix) - 1;
    const std::size_t question = path.find('?', prefixLength);
    const ct::String kindName = question == ct::String::npos
                                        ? path.substr(prefixLength)
                                        : path.substr(prefixLength, question - prefixLength);
    int kindIndex = -1;
    for (int k = 0; k < static_cast<int>(sizeof(kKinds) / sizeof(kKinds[0])); ++k)
        if (kindName == kKinds[k].name) kindIndex = k;
    if (kindIndex < 0)
    {
        error = ct::String("unknown primitive mesh '") + kindName + "' in " + describePath(path);
        return false;
    }
    recipe.kind = static_cast<PrimitiveKind>(kindIndex);
    const KindDef& def = kKinds[kindIndex];

    bool seen[9] = {};
    bool heightmapSeen = false;
    applyDefaults(recipe);

    std::size_t at = question == ct::String::npos ? path.size() : question + 1;
    while (at < path.size())
    {
        std::size_t end = path.find('&', at);
        if (end == ct::String::npos) end = path.size();
        const ct::String item = path.substr(at, end - at);
        at = end + 1;
        if (item.empty()) continue;
        const std::size_t equals = item.find('=');
        if (equals == ct::String::npos)
        {
            error = ct::String("'") + item + "' has no value in " + describePath(path);
            return false;
        }
        const ct::String key = item.substr(0, equals);
        const ct::String text = item.substr(equals + 1);
        if (def.heightmap && key == "heightmap")
        {
            if (heightmapSeen || !decode(text, recipe.heightmap) || recipe.heightmap.empty())
            {
                error = ct::String("bad heightmap in ") + describePath(path);
                return false;
            }
            heightmapSeen = true;
            continue;
        }
        int value = 0;
        int found = -1;
        for (int p = 0; p < def.count; ++p)
        {
            if (key == def.params[p].key)
            {
                found = p;
                break;
            }
            value += def.params[p].type == PrimitiveParamType::Vec3 ? 3 : 1;
        }
        if (found < 0)
        {
            error = ct::String("unknown parameter '") + key + "' for " + def.name + " in " +
                    describePath(path);
            return false;
        }
        if (seen[found])
        {
            error = ct::String("repeated parameter '") + key + "' in " + describePath(path);
            return false;
        }
        seen[found] = true;
        const PrimitiveParamDef& param = def.params[found];
        const int components = param.type == PrimitiveParamType::Vec3 ? 3 : 1;
        std::size_t from = 0;
        for (int c = 0; c < components; ++c)
        {
            const bool last = c == components - 1;
            const std::size_t comma = last ? ct::String::npos : text.find(',', from);
            if (!last && comma == ct::String::npos)
            {
                error = ct::String("'") + key + "' needs three numbers in " + describePath(path);
                return false;
            }
            const std::size_t stop = last ? text.size() : comma;
            float number = 0.0f;
            bool ok = parseNumber(text.substr(from, stop - from), number);
            if (ok && param.type == PrimitiveParamType::Count)
                ok = number == floorf(number) && number >= param.minimum && number <= param.maximum;
            else if (ok)
                ok = number >= param.minimum && number <= param.maximum;
            if (!ok)
            {
                error = ct::String("bad value for '") + key + "' in " + describePath(path);
                return false;
            }
            recipe.values[value + c] = number;
            from = stop + 1;
        }
    }
    if (def.heightmap && !heightmapSeen)
    {
        error = ct::String("the heightmap is missing in ") + describePath(path);
        return false;
    }
    if (recipe.kind == PrimitiveKind::Plane || recipe.kind == PrimitiveKind::HillsPlane)
    {
        if (recipe.values[2] * recipe.values[3] > kMaxGridCells)
        {
            error = ct::String("too many cells in ") + describePath(path);
            return false;
        }
    }
    return true;
}

bool loadHeightmap(const ct::String& file, Pixmap& out, ct::String& error)
{
    const ct::Vector<std::uint8_t> bytes = FileSystem::getSingleton().readBinary(file);
    if (bytes.empty())
    {
        error = ct::String("the heightmap '") + file + "' was not found";
        return false;
    }
    if (!out.load_from_memory(bytes.data(), static_cast<std::uint32_t>(bytes.size())) ||
            !heightmapUsable(out))
    {
        error = ct::String("the heightmap '") + file + "' is not a usable image";
        return false;
    }
    return true;
}

MeshId buildRecipe(MeshStore& meshes, const PrimitiveRecipe& recipe, const char* name, bool strict,
        ct::String& error)
{
    SurfaceData surface;
    const float* v = recipe.values;
    switch (recipe.kind)
    {
        case PrimitiveKind::Box:
            boxMeshArrays(surface, Math::Vec3(v[0], v[1], v[2]));
            break;
        case PrimitiveKind::Plane:
            planeMeshArrays(surface, v[0], v[1], static_cast<int>(v[2]), static_cast<int>(v[3]),
                    v[4]);
            break;
        case PrimitiveKind::Sphere:
            sphereMeshArrays(surface, v[0], static_cast<int>(v[1]), static_cast<int>(v[2]));
            break;
        case PrimitiveKind::Cylinder:
            cylinderMeshArrays(surface, v[0], v[1], static_cast<int>(v[2]));
            break;
        case PrimitiveKind::Cone:
            coneMeshArrays(surface, v[0], v[1], static_cast<int>(v[2]));
            break;
        case PrimitiveKind::Capsule:
            capsuleMeshArrays(surface, v[0], v[1], static_cast<int>(v[2]), static_cast<int>(v[3]));
            break;
        case PrimitiveKind::Torus:
            torusMeshArrays(surface, v[0], v[1], static_cast<int>(v[2]), static_cast<int>(v[3]));
            break;
        case PrimitiveKind::HillsPlane:
        {
            Pixmap heightmap;
            if (!loadHeightmap(recipe.heightmap, heightmap, error)) return MeshId();
            hillsPlaneMeshArrays(surface, v[0], v[1], static_cast<int>(v[2]),
                    static_cast<int>(v[3]), heightmap, v[4], v[5]);
            break;
        }
        case PrimitiveKind::Heightfield:
        {
            Pixmap heightmap;
            if (!loadHeightmap(recipe.heightmap, heightmap, error)) return MeshId();
            heightfieldMeshArrays(surface, heightmap, v[0], v[1], v[2]);
            break;
        }
    }
    if (strict && surface.positions.empty())
    {
        error = ct::String("the primitive mesh ") + describePath(recipePath(recipe)) +
                " has no geometry";
        return MeshId();
    }
    const MeshId mesh = createFromSurface(meshes, surface, name);
    if (mesh.valid()) meshes.setPath(mesh, recipePath(recipe));
    return mesh;
}

void clampCounts(PrimitiveRecipe& recipe)
{
    const KindDef& def = kindDef(recipe.kind);
    int value = 0;
    for (int p = 0; p < def.count; ++p)
    {
        const PrimitiveParamDef& param = def.params[p];
        if (param.type == PrimitiveParamType::Vec3)
        {
            value += 3;
            continue;
        }
        float& number = recipe.values[value++];
        if (param.type != PrimitiveParamType::Count) continue;
        if (number < param.minimum) number = param.minimum;
        if (number > param.maximum) number = param.maximum;
    }
}

MeshId create(MeshStore& meshes, PrimitiveRecipe recipe, const char* name)
{
    clampCounts(recipe);
    ct::String error;
    const MeshId mesh = buildRecipe(meshes, recipe, name, false, error);
    if (!mesh.valid() && !error.empty()) log_error("primitive: %s", error.c_str());
    return mesh;
}

} // namespace

std::size_t primitiveKindCount()
{
    return sizeof(kKinds) / sizeof(kKinds[0]);
}

const char* primitiveKindName(PrimitiveKind kind)
{
    return kindDef(kind).name;
}

bool primitiveKindHasHeightmap(PrimitiveKind kind)
{
    return kindDef(kind).heightmap;
}

int primitiveHeightmapSlot(PrimitiveKind kind)
{
    if (!kindDef(kind).heightmap) return -1;
    return kind == PrimitiveKind::HillsPlane ? 4 : 0;
}

std::size_t primitiveParamCount(PrimitiveKind kind)
{
    return static_cast<std::size_t>(kindDef(kind).count);
}

const PrimitiveParamDef& primitiveParam(PrimitiveKind kind, std::size_t index)
{
    return kindDef(kind).params[index];
}

void primitiveDefaults(PrimitiveKind kind, PrimitiveRecipe& out)
{
    out.kind = kind;
    applyDefaults(out);
}

bool parsePrimitiveRecipe(const ct::String& path, PrimitiveRecipe& out, ct::String& error)
{
    return parseRecipe(path, out, error);
}

ct::String primitiveRecipePath(const PrimitiveRecipe& recipe)
{
    return recipePath(recipe);
}

std::size_t primitiveTriangleCount(const PrimitiveRecipe& recipe)
{
    const float* v = recipe.values;
    const auto count = [](float value, int minimum)
    {
        const int number = static_cast<int>(value);
        return static_cast<std::size_t>(number < minimum ? minimum : number);
    };
    const bool capped = v[0] >= 1e-5f;
    switch (recipe.kind)
    {
        case PrimitiveKind::Box:
            return 12;
        case PrimitiveKind::Plane:
        case PrimitiveKind::HillsPlane:
            return count(v[2], 1) * count(v[3], 1) * 2;
        case PrimitiveKind::Sphere:
            return count(v[1], 2) * count(v[2], 3) * 2;
        case PrimitiveKind::Cylinder:
            return count(v[2], 3) * (capped ? 4 : 2);
        case PrimitiveKind::Cone:
            return count(v[2], 3) * (capped ? 3 : 2);
        case PrimitiveKind::Capsule:
            return (count(v[2], 2) * 2 + 1) * count(v[3], 3) * 2;
        case PrimitiveKind::Torus:
            return count(v[2], 3) * count(v[3], 3) * 2;
        case PrimitiveKind::Heightfield:
            break;
    }
    return 0;
}

MeshId createBoxMesh(MeshStore& meshes, const Math::Vec3& size, const char* name)
{
    PrimitiveRecipe recipe;
    recipe.kind = PrimitiveKind::Box;
    recipe.values[0] = size.x;
    recipe.values[1] = size.y;
    recipe.values[2] = size.z;
    return create(meshes, recipe, name);
}

MeshId createPlaneMesh(MeshStore& meshes, float width, float depth, int segmentsX, int segmentsZ,
        float uvTiles, const char* name)
{
    PrimitiveRecipe recipe;
    recipe.kind = PrimitiveKind::Plane;
    recipe.values[0] = width;
    recipe.values[1] = depth;
    recipe.values[2] = static_cast<float>(segmentsX);
    recipe.values[3] = static_cast<float>(segmentsZ);
    recipe.values[4] = uvTiles;
    return create(meshes, recipe, name);
}

MeshId createSphereMesh(MeshStore& meshes, float radius, int rings, int slices, const char* name)
{
    PrimitiveRecipe recipe;
    recipe.kind = PrimitiveKind::Sphere;
    recipe.values[0] = radius;
    recipe.values[1] = static_cast<float>(rings);
    recipe.values[2] = static_cast<float>(slices);
    return create(meshes, recipe, name);
}

MeshId createCylinderMesh(MeshStore& meshes, float radius, float height, int slices,
        const char* name)
{
    PrimitiveRecipe recipe;
    recipe.kind = PrimitiveKind::Cylinder;
    recipe.values[0] = radius;
    recipe.values[1] = height;
    recipe.values[2] = static_cast<float>(slices);
    return create(meshes, recipe, name);
}

MeshId createConeMesh(MeshStore& meshes, float radius, float height, int slices, const char* name)
{
    PrimitiveRecipe recipe;
    recipe.kind = PrimitiveKind::Cone;
    recipe.values[0] = radius;
    recipe.values[1] = height;
    recipe.values[2] = static_cast<float>(slices);
    return create(meshes, recipe, name);
}

MeshId createCapsuleMesh(MeshStore& meshes, float radius, float height, int rings, int slices,
        const char* name)
{
    PrimitiveRecipe recipe;
    recipe.kind = PrimitiveKind::Capsule;
    recipe.values[0] = radius;
    recipe.values[1] = height;
    recipe.values[2] = static_cast<float>(rings);
    recipe.values[3] = static_cast<float>(slices);
    return create(meshes, recipe, name);
}

MeshId createTorusMesh(MeshStore& meshes, float majorRadius, float minorRadius, int majorSegments,
        int minorSegments, const char* name)
{
    PrimitiveRecipe recipe;
    recipe.kind = PrimitiveKind::Torus;
    recipe.values[0] = majorRadius;
    recipe.values[1] = minorRadius;
    recipe.values[2] = static_cast<float>(majorSegments);
    recipe.values[3] = static_cast<float>(minorSegments);
    return create(meshes, recipe, name);
}

MeshId createHeightfieldMesh(MeshStore& meshes, const Pixmap& heightmap, float cellSize,
        float heightScale, float uvTiles, const char* name)
{
    SurfaceData surface;
    heightfieldMeshArrays(surface, heightmap, cellSize, heightScale, uvTiles);
    return createFromSurface(meshes, surface, name);
}

MeshId createHillsPlaneMeshFromFile(MeshStore& meshes, float width, float depth, int segmentsX,
        int segmentsZ, const ct::String& heightmapFile, float heightScale, float uvTiles,
        const char* name)
{
    PrimitiveRecipe recipe;
    recipe.kind = PrimitiveKind::HillsPlane;
    recipe.values[0] = width;
    recipe.values[1] = depth;
    recipe.values[2] = static_cast<float>(segmentsX);
    recipe.values[3] = static_cast<float>(segmentsZ);
    recipe.values[4] = heightScale;
    recipe.values[5] = uvTiles;
    recipe.heightmap = heightmapFile;
    return create(meshes, recipe, name);
}

MeshId createHeightfieldMeshFromFile(MeshStore& meshes, const ct::String& heightmapFile,
        float cellSize, float heightScale, float uvTiles, const char* name)
{
    PrimitiveRecipe recipe;
    recipe.kind = PrimitiveKind::Heightfield;
    recipe.values[0] = cellSize;
    recipe.values[1] = heightScale;
    recipe.values[2] = uvTiles;
    recipe.heightmap = heightmapFile;
    return create(meshes, recipe, name);
}

bool isPrimitivePath(const ct::String& path)
{
    return path.starts_with(kPrimitivePrefix);
}

MeshId loadPrimitiveMesh(MeshStore& meshes, const ct::String& path, ct::String& error)
{
    PrimitiveRecipe recipe;
    if (!parseRecipe(path, recipe, error)) return MeshId();
    const ct::String canonical = recipePath(recipe);
    const MeshId existing = meshes.find(canonical);
    if (existing.valid()) return existing;
    const KindDef& def = kindDef(recipe.kind);
    ct::String name = def.name;
    name += "-mesh";
    return buildRecipe(meshes, recipe, name.c_str(), true, error);
}

} // namespace prisma3d
