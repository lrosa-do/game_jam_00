#pragma once

#include "prisma3d/render/MeshStore.h"

#include "mathc.h"

#include <ct/vector.hpp>

#include <cstddef>
#include <cstdint>

namespace prisma3d
{

class SurfaceTool
{
public:
    static constexpr std::uint32_t kFlatShading = 0xFFFFFFFFu;

    SurfaceTool();

    void begin(prisma::Topology topology);
    void setColor(const Math::Vec4& color);
    void setNormal(const Math::Vec3& normal);
    void setTangent(const Math::Vec4& tangent);
    void setUv(const Math::Vec2& uv);
    void setUv2(const Math::Vec2& uv2);
    void setSmoothGroup(std::uint32_t group);
    void addVertex(const Math::Vec3& position);
    void addIndex(std::uint32_t index);
    void addTriangle(const Math::Vec3& a, const Math::Vec3& b, const Math::Vec3& c);
    void addQuad(const Math::Vec3& a, const Math::Vec3& b, const Math::Vec3& c,
            const Math::Vec3& d);
    void addTriangleFan(const ct::Vector<Math::Vec3>& positions,
            const ct::Vector<Math::Vec2>& uvs = ct::Vector<Math::Vec2>(),
            const ct::Vector<Math::Vec4>& colors = ct::Vector<Math::Vec4>(),
            const ct::Vector<Math::Vec2>& uvs2 = ct::Vector<Math::Vec2>(),
            const ct::Vector<Math::Vec3>& normals = ct::Vector<Math::Vec3>(),
            const ct::Vector<Math::Vec4>& tangents = ct::Vector<Math::Vec4>());
    void index();
    void deindex();
    void generateNormals(bool flip = false);
    bool generateTangents();
    void clear();
    bool commit(SurfaceData& out) const;

    bool isBegun() const;
    prisma::Topology getTopology() const;
    std::size_t getVertexCount() const;
    std::size_t getIndexCount() const;

private:
    enum FormatBit : std::uint32_t
    {
        kFormatNormal = 1u << 0,
        kFormatTangent = 1u << 1,
        kFormatColor = 1u << 2,
        kFormatUv = 1u << 3,
        kFormatUv2 = 1u << 4,
        kFormatIndex = 1u << 5
    };

    struct Vertex
    {
        std::uint32_t smoothGroup = 0;
        Math::Vec4 color = Math::Vec4(0.0f, 0.0f, 0.0f, 1.0f);
        Math::Vec3 normal = Math::Vec3(0.0f, 0.0f, 0.0f);
        Math::Vec4 tangent = Math::Vec4(0.0f, 0.0f, 0.0f, 0.0f);
        Math::Vec2 uv = Math::Vec2(0.0f, 0.0f);
        Math::Vec2 uv2 = Math::Vec2(0.0f, 0.0f);
        Math::Vec3 vertex = Math::Vec3(0.0f, 0.0f, 0.0f);

        bool operator==(const Vertex& other) const;
        std::uint64_t hash() const;
    };

    bool mBegun;
    bool mFirst;
    prisma::Topology mTopology;
    std::uint32_t mFormat;
    ct::Vector<Vertex> mVertices;
    ct::Vector<std::uint32_t> mIndices;
    Math::Vec4 mLastColor;
    Math::Vec3 mLastNormal;
    Math::Vec2 mLastUv;
    Math::Vec2 mLastUv2;
    Math::Vec4 mLastTangent;
    std::uint32_t mLastSmoothGroup;
};

} // namespace prisma3d
