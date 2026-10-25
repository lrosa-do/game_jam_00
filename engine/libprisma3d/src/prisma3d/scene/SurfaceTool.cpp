#include "prisma3d/scene/SurfaceTool.h"

#include <ct/hashmap.hpp>

#include <math.h>
#include <string.h>

namespace prisma3d
{

namespace
{

bool sameVec(const Math::Vec2& a, const Math::Vec2& b)
{
    return a.x == b.x && a.y == b.y;
}

bool sameVec(const Math::Vec3& a, const Math::Vec3& b)
{
    return a.x == b.x && a.y == b.y && a.z == b.z;
}

bool sameVec(const Math::Vec4& a, const Math::Vec4& b)
{
    return a.x == b.x && a.y == b.y && a.z == b.z && a.w == b.w;
}

std::uint64_t mixFloat(std::uint64_t seed, float value)
{
    value += 0.0f;
    std::uint32_t bits = 0;
    memcpy(&bits, &value, sizeof(bits));
    return ct::hash_combine(seed, bits);
}

std::uint64_t mixVec(std::uint64_t seed, const Math::Vec2& v)
{
    return mixFloat(mixFloat(seed, v.x), v.y);
}

std::uint64_t mixVec(std::uint64_t seed, const Math::Vec3& v)
{
    return mixFloat(mixFloat(mixFloat(seed, v.x), v.y), v.z);
}

std::uint64_t mixVec(std::uint64_t seed, const Math::Vec4& v)
{
    return mixFloat(mixFloat(mixFloat(mixFloat(seed, v.x), v.y), v.z), v.w);
}

Math::Vec3 normalizedOrZero(const Math::Vec3& v)
{
    const float lengthSquared = v.LengthSquared();
    if (lengthSquared == 0.0f) return Math::Vec3(0.0f, 0.0f, 0.0f);
    const float inverse = 1.0f / sqrtf(lengthSquared);
    return Math::Vec3(v.x * inverse, v.y * inverse, v.z * inverse);
}

Math::Vec3 planeNormal(const Math::Vec3& a, const Math::Vec3& b, const Math::Vec3& c)
{
    return normalizedOrZero((b - a).Cross(c - a));
}

struct SmoothKey
{
    Math::Vec3 vertex;
    std::uint32_t smoothGroup = 0;

    bool operator==(const SmoothKey& other) const
    {
        return sameVec(vertex, other.vertex) && smoothGroup == other.smoothGroup;
    }

    std::uint64_t hash() const
    {
        return ct::hash_combine(mixVec(0, vertex), smoothGroup);
    }
};

} // namespace

bool SurfaceTool::Vertex::operator==(const Vertex& other) const
{
    return sameVec(vertex, other.vertex) && sameVec(uv, other.uv) && sameVec(uv2, other.uv2) &&
           sameVec(normal, other.normal) && sameVec(tangent, other.tangent) &&
           sameVec(color, other.color) && smoothGroup == other.smoothGroup;
}

std::uint64_t SurfaceTool::Vertex::hash() const
{
    std::uint64_t h = smoothGroup;
    h = mixVec(h, color);
    h = mixVec(h, normal);
    h = mixVec(h, tangent);
    h = mixVec(h, uv);
    h = mixVec(h, uv2);
    h = mixVec(h, vertex);
    return h;
}

SurfaceTool::SurfaceTool()
        : mBegun(false),
          mFirst(false),
          mTopology(prisma::Topology::Lines),
          mFormat(0),
          mVertices(),
          mIndices(),
          mLastColor(0.0f, 0.0f, 0.0f, 1.0f),
          mLastNormal(0.0f, 0.0f, 0.0f),
          mLastUv(0.0f, 0.0f),
          mLastUv2(0.0f, 0.0f),
          mLastTangent(0.0f, 0.0f, 0.0f, 0.0f),
          mLastSmoothGroup(0)
{
}

void SurfaceTool::begin(prisma::Topology topology)
{
    clear();

    mTopology = topology;
    mBegun = true;
    mFirst = true;
}

void SurfaceTool::addVertex(const Math::Vec3& position)
{
    if (!mBegun) return;

    Vertex vertex;
    vertex.vertex = position;
    vertex.color = mLastColor;
    vertex.normal = mLastNormal;
    vertex.uv = mLastUv;
    vertex.uv2 = mLastUv2;
    vertex.tangent = mLastTangent;
    vertex.smoothGroup = mLastSmoothGroup;

    mVertices.push_back(vertex);
    mFirst = false;
}

void SurfaceTool::setColor(const Math::Vec4& color)
{
    if (!mBegun) return;
    if (!mFirst && !(mFormat & kFormatColor)) return;

    mFormat |= kFormatColor;
    mLastColor = color;
}

void SurfaceTool::setNormal(const Math::Vec3& normal)
{
    if (!mBegun) return;
    if (!mFirst && !(mFormat & kFormatNormal)) return;

    mFormat |= kFormatNormal;
    mLastNormal = normal;
}

void SurfaceTool::setTangent(const Math::Vec4& tangent)
{
    if (!mBegun) return;
    if (!mFirst && !(mFormat & kFormatTangent)) return;

    mFormat |= kFormatTangent;
    mLastTangent = tangent;
}

void SurfaceTool::setUv(const Math::Vec2& uv)
{
    if (!mBegun) return;
    if (!mFirst && !(mFormat & kFormatUv)) return;

    mFormat |= kFormatUv;
    mLastUv = uv;
}

void SurfaceTool::setUv2(const Math::Vec2& uv2)
{
    if (!mBegun) return;
    if (!mFirst && !(mFormat & kFormatUv2)) return;

    mFormat |= kFormatUv2;
    mLastUv2 = uv2;
}

void SurfaceTool::setSmoothGroup(std::uint32_t group)
{
    mLastSmoothGroup = group;
}

void SurfaceTool::addTriangle(const Math::Vec3& a, const Math::Vec3& b, const Math::Vec3& c)
{
    if (!mBegun || mTopology != prisma::Topology::Triangles) return;

    addVertex(a);
    addVertex(b);
    addVertex(c);
}

void SurfaceTool::addQuad(const Math::Vec3& a, const Math::Vec3& b, const Math::Vec3& c,
        const Math::Vec3& d)
{
    if (!mBegun || mTopology != prisma::Topology::Triangles) return;

    addTriangle(a, b, c);
    addTriangle(a, c, d);
}

void SurfaceTool::addTriangleFan(const ct::Vector<Math::Vec3>& positions,
        const ct::Vector<Math::Vec2>& uvs, const ct::Vector<Math::Vec4>& colors,
        const ct::Vector<Math::Vec2>& uvs2, const ct::Vector<Math::Vec3>& normals,
        const ct::Vector<Math::Vec4>& tangents)
{
    if (!mBegun || mTopology != prisma::Topology::Triangles) return;
    if (positions.size() < 3) return;

    auto addPoint = [&](std::size_t n)
    {
        if (colors.size() > n) setColor(colors[n]);
        if (uvs.size() > n) setUv(uvs[n]);
        if (uvs2.size() > n) setUv2(uvs2[n]);
        if (normals.size() > n) setNormal(normals[n]);
        if (tangents.size() > n) setTangent(tangents[n]);
        addVertex(positions[n]);
    };

    for (std::size_t i = 0; i + 2 < positions.size(); ++i)
    {
        addPoint(0);
        addPoint(i + 1);
        addPoint(i + 2);
    }
}

void SurfaceTool::addIndex(std::uint32_t index)
{
    if (!mBegun) return;

    mFormat |= kFormatIndex;
    mIndices.push_back(index);
}

void SurfaceTool::index()
{
    if (!mIndices.empty()) return;

    ct::HashMap<Vertex, std::uint32_t> indices;
    indices.reserve(mVertices.size());

    std::size_t newSize = 0;
    for (std::size_t i = 0; i < mVertices.size(); ++i)
    {
        const Vertex vertex = mVertices[i];
        std::uint32_t index = 0;
        if (const std::uint32_t* found = indices.find(vertex))
        {
            index = *found;
        }
        else
        {
            index = static_cast<std::uint32_t>(indices.size());
            mVertices[newSize] = vertex;
            indices.put(vertex, index);
            ++newSize;
        }

        mIndices.push_back(index);
    }

    mVertices.resize(newSize);
    mFormat |= kFormatIndex;
}

void SurfaceTool::deindex()
{
    if (mIndices.empty()) return;

    for (std::size_t i = 0; i < mIndices.size(); ++i)
        if (mIndices[i] >= mVertices.size()) return;

    ct::Vector<Vertex> old = mVertices;
    mVertices.clear();
    for (std::size_t i = 0; i < mIndices.size(); ++i) mVertices.push_back(old[mIndices[i]]);

    mFormat &= ~static_cast<std::uint32_t>(kFormatIndex);
    mIndices.clear();
}

void SurfaceTool::generateNormals(bool flip)
{
    if (mTopology != prisma::Topology::Triangles) return;

    const bool wasIndexed = !mIndices.empty();

    deindex();

    if ((mVertices.size() % 3) != 0) return;

    ct::HashMap<SmoothKey, Math::Vec3> smooth;
    smooth.reserve(mVertices.size());

    for (std::size_t vi = 0; vi < mVertices.size(); vi += 3)
    {
        Vertex* v = &mVertices[vi];

        Math::Vec3 normal;
        if (!flip) normal = planeNormal(v[0].vertex, v[1].vertex, v[2].vertex);
        else
            normal = planeNormal(v[2].vertex, v[1].vertex, v[0].vertex);

        for (int i = 0; i < 3; ++i)
        {
            if (v[i].smoothGroup != kFlatShading)
            {
                SmoothKey key;
                key.vertex = v[i].vertex;
                key.smoothGroup = v[i].smoothGroup;
                if (Math::Vec3* sum = smooth.find(key)) *sum += normal;
                else
                    smooth.put(key, normal);
            }
            else
            {
                v[i].normal = normal;
            }
        }
    }

    for (std::size_t i = 0; i < mVertices.size(); ++i)
    {
        Vertex& vertex = mVertices[i];
        if (vertex.smoothGroup == kFlatShading) continue;

        SmoothKey key;
        key.vertex = vertex.vertex;
        key.smoothGroup = vertex.smoothGroup;
        const Math::Vec3* sum = smooth.find(key);
        vertex.normal = sum ? normalizedOrZero(*sum) : Math::Vec3(0.0f, 0.0f, 0.0f);
    }

    mFormat |= kFormatNormal;

    if (wasIndexed) index();
}

bool SurfaceTool::generateTangents()
{
    if (!(mFormat & kFormatUv)) return false;
    if (!(mFormat & kFormatNormal)) return false;
    if (mTopology != prisma::Topology::Triangles) return false;

    if (mVertices.empty())
    {
        mFormat |= kFormatTangent;
        return true;
    }

    const std::size_t cornerCount = mIndices.size() > 0 ? mIndices.size() : mVertices.size();
    if (cornerCount % 3 != 0) return false;

    SurfaceData data;
    data.topology = mTopology;
    data.positions.reserve(mVertices.size());
    data.normals.reserve(mVertices.size());
    data.uvs.reserve(mVertices.size());
    for (std::size_t i = 0; i < mVertices.size(); ++i)
    {
        data.positions.push_back(mVertices[i].vertex);
        data.normals.push_back(mVertices[i].normal);
        data.uvs.push_back(mVertices[i].uv);
    }
    data.indices = mIndices;

    if (!MeshStore::generateTangents(data)) return false;

    for (std::size_t i = 0; i < mVertices.size(); ++i) mVertices[i].tangent = data.tangents[i];

    mFormat |= kFormatTangent;
    return true;
}

void SurfaceTool::clear()
{
    mBegun = false;
    mTopology = prisma::Topology::Lines;
    mFormat = 0;
    mIndices.clear();
    mVertices.clear();
    mLastSmoothGroup = 0;
}

bool SurfaceTool::commit(SurfaceData& out) const
{
    if (mVertices.empty()) return false;

    const std::size_t count = mVertices.size();

    out = SurfaceData();
    out.topology = mTopology;
    out.positions.resize(count);
    if (mFormat & kFormatNormal) out.normals.resize(count);
    if (mFormat & kFormatTangent) out.tangents.resize(count);
    if (mFormat & kFormatColor) out.colors.resize(count);
    if (mFormat & kFormatUv) out.uvs.resize(count);
    if (mFormat & kFormatUv2) out.uvs2.resize(count);

    for (std::size_t i = 0; i < count; ++i)
    {
        const Vertex& vertex = mVertices[i];
        out.positions[i] = vertex.vertex;
        if (mFormat & kFormatNormal) out.normals[i] = vertex.normal;
        if (mFormat & kFormatTangent) out.tangents[i] = vertex.tangent;
        if (mFormat & kFormatColor) out.colors[i] = vertex.color;
        if (mFormat & kFormatUv) out.uvs[i] = vertex.uv;
        if (mFormat & kFormatUv2) out.uvs2[i] = vertex.uv2;
    }

    if (mFormat & kFormatIndex) out.indices = mIndices;
    return true;
}

bool SurfaceTool::isBegun() const
{
    return mBegun;
}

prisma::Topology SurfaceTool::getTopology() const
{
    return mTopology;
}

std::size_t SurfaceTool::getVertexCount() const
{
    return mVertices.size();
}

std::size_t SurfaceTool::getIndexCount() const
{
    return mIndices.size();
}

} // namespace prisma3d
