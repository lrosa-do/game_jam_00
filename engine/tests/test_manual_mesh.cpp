#include "Check.h"

#include "prisma3d/Device.h"
#include "prisma3d/render/MaterialStore.h"
#include "prisma3d/render/MeshStore.h"
#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/render/TextureStore.h"
#include "prisma3d/scene/Collision.h"
#include "prisma3d/scene/ManualMesh3D.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/SurfaceTool.h"

#include <math.h>
#include <string.h>

using namespace prisma3d;

namespace
{

const float kTolerance = 1e-5f;

bool near(float a, float b)
{
    return fabsf(a - b) <= kTolerance;
}

bool isVec3(const Math::Vec3& v, float x, float y, float z)
{
    return near(v.x, x) && near(v.y, y) && near(v.z, z);
}

bool isVec2(const Math::Vec2& v, float x, float y)
{
    return near(v.x, x) && near(v.y, y);
}

bool isVec4(const Math::Vec4& v, float x, float y, float z, float w)
{
    return near(v.x, x) && near(v.y, y) && near(v.z, z) && near(v.w, w);
}

const Math::Vec3 kP0(0.0f, 0.0f, 0.0f);
const Math::Vec3 kP1(1.0f, 0.0f, 0.0f);
const Math::Vec3 kP2(1.0f, 1.0f, 0.0f);
const Math::Vec3 kP3(0.0f, 1.0f, 0.0f);

void testAttributes()
{
    SurfaceTool tool;
    tool.setColor(Math::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
    tool.addVertex(kP0);
    CHECK(!tool.isBegun());
    CHECK(tool.getVertexCount() == 0);

    tool.begin(prisma::Topology::Triangles);
    CHECK(tool.isBegun());
    CHECK(tool.getTopology() == prisma::Topology::Triangles);
    tool.setNormal(Math::Vec3(0.0f, 0.0f, 1.0f));
    tool.setUv(Math::Vec2(0.5f, 0.25f));
    tool.addVertex(kP0);
    tool.setColor(Math::Vec4(0.0f, 1.0f, 0.0f, 1.0f));
    tool.setUv2(Math::Vec2(1.0f, 1.0f));
    tool.setTangent(Math::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
    tool.setUv(Math::Vec2(0.75f, 0.5f));
    tool.addVertex(kP1);
    tool.addVertex(kP3);

    SurfaceData out;
    CHECK(tool.commit(out));
    CHECK(out.topology == prisma::Topology::Triangles);
    CHECK(out.positions.size() == 3);
    CHECK(isVec3(out.positions[0], 0.0f, 0.0f, 0.0f));
    CHECK(isVec3(out.positions[1], 1.0f, 0.0f, 0.0f));
    CHECK(isVec3(out.positions[2], 0.0f, 1.0f, 0.0f));
    CHECK(out.normals.size() == 3);
    CHECK(out.uvs.size() == 3);
    CHECK(out.colors.empty());
    CHECK(out.tangents.empty());
    CHECK(out.uvs2.empty());
    CHECK(out.indices.empty());
    CHECK(isVec3(out.normals[2], 0.0f, 0.0f, 1.0f));
    CHECK(isVec2(out.uvs[0], 0.5f, 0.25f));
    CHECK(isVec2(out.uvs[1], 0.75f, 0.5f));
    CHECK(isVec2(out.uvs[2], 0.75f, 0.5f));

    SurfaceTool empty;
    empty.begin(prisma::Topology::Lines);
    SurfaceData untouched;
    untouched.positions.push_back(kP0);
    CHECK(!empty.commit(untouched));
    CHECK(untouched.positions.size() == 1);

    tool.clear();
    CHECK(!tool.isBegun());
    CHECK(tool.getVertexCount() == 0);
    CHECK(!tool.commit(out));

    SurfaceTool colored;
    colored.begin(prisma::Topology::Points);
    colored.setColor(Math::Vec4(0.25f, 0.5f, 0.75f, 1.0f));
    colored.setUv2(Math::Vec2(2.0f, 3.0f));
    colored.setTangent(Math::Vec4(0.0f, 1.0f, 0.0f, -1.0f));
    colored.addVertex(kP2);
    CHECK(colored.commit(out));
    CHECK(out.topology == prisma::Topology::Points);
    CHECK(out.colors.size() == 1 && isVec4(out.colors[0], 0.25f, 0.5f, 0.75f, 1.0f));
    CHECK(out.uvs2.size() == 1 && isVec2(out.uvs2[0], 2.0f, 3.0f));
    CHECK(out.tangents.size() == 1 && isVec4(out.tangents[0], 0.0f, 1.0f, 0.0f, -1.0f));
    CHECK(out.normals.empty());
    CHECK(out.uvs.empty());
}

void testTrianglesAndQuads()
{
    SurfaceTool tool;
    tool.begin(prisma::Topology::Triangles);
    tool.addTriangle(kP0, kP1, kP3);
    CHECK(tool.getVertexCount() == 3);
    tool.addQuad(kP0, kP1, kP2, kP3);
    CHECK(tool.getVertexCount() == 9);

    SurfaceData out;
    CHECK(tool.commit(out));
    const Math::Vec3 expected[9] = { kP0, kP1, kP3, kP0, kP1, kP2, kP0, kP2, kP3 };
    for (int i = 0; i < 9; ++i) CHECK(isVec3(out.positions[i], expected[i].x, expected[i].y, 0.0f));

    SurfaceTool lines;
    lines.begin(prisma::Topology::Lines);
    lines.addTriangle(kP0, kP1, kP3);
    lines.addQuad(kP0, kP1, kP2, kP3);
    CHECK(lines.getVertexCount() == 0);

    SurfaceTool indexed;
    indexed.begin(prisma::Topology::Triangles);
    indexed.addVertex(kP0);
    indexed.addVertex(kP1);
    indexed.addVertex(kP2);
    indexed.addVertex(kP3);
    const std::uint32_t order[6] = { 0, 1, 2, 0, 2, 3 };
    for (int i = 0; i < 6; ++i) indexed.addIndex(order[i]);
    CHECK(indexed.getIndexCount() == 6);
    CHECK(indexed.commit(out));
    CHECK(out.positions.size() == 4);
    CHECK(out.indices.size() == 6);
    for (int i = 0; i < 6; ++i) CHECK(out.indices[i] == order[i]);
}

void testTriangleFan()
{
    ct::Vector<Math::Vec3> positions;
    positions.push_back(kP0);
    positions.push_back(kP1);
    positions.push_back(kP2);
    positions.push_back(kP3);
    ct::Vector<Math::Vec2> uvs;
    uvs.push_back(Math::Vec2(0.0f, 0.0f));
    uvs.push_back(Math::Vec2(1.0f, 0.0f));
    uvs.push_back(Math::Vec2(1.0f, 1.0f));
    uvs.push_back(Math::Vec2(0.0f, 1.0f));

    SurfaceTool tool;
    tool.begin(prisma::Topology::Triangles);
    tool.addTriangleFan(positions, uvs);
    CHECK(tool.getVertexCount() == 6);

    SurfaceData out;
    CHECK(tool.commit(out));
    const int order[6] = { 0, 1, 2, 0, 2, 3 };
    CHECK(out.uvs.size() == 6);
    CHECK(out.normals.empty() && out.colors.empty());
    for (int i = 0; i < 6; ++i)
    {
        CHECK(isVec3(out.positions[i], positions[order[i]].x, positions[order[i]].y, 0.0f));
        CHECK(isVec2(out.uvs[i], uvs[order[i]].x, uvs[order[i]].y));
    }

    positions.push_back(Math::Vec3(-1.0f, 0.5f, 0.0f));
    tool.begin(prisma::Topology::Triangles);
    tool.addTriangleFan(positions);
    CHECK(tool.getVertexCount() == 9);
    CHECK(tool.commit(out));
    CHECK(out.uvs.empty());
    CHECK(isVec3(out.positions[6], 0.0f, 0.0f, 0.0f));
    CHECK(isVec3(out.positions[7], 0.0f, 1.0f, 0.0f));
    CHECK(isVec3(out.positions[8], -1.0f, 0.5f, 0.0f));

    ct::Vector<Math::Vec3> two;
    two.push_back(kP0);
    two.push_back(kP1);
    tool.begin(prisma::Topology::Triangles);
    tool.addTriangleFan(two);
    CHECK(tool.getVertexCount() == 0);

    tool.begin(prisma::Topology::Lines);
    tool.addTriangleFan(positions);
    CHECK(tool.getVertexCount() == 0);

    ct::Vector<Math::Vec3> normals;
    for (int i = 0; i < 4; ++i) normals.push_back(Math::Vec3(0.0f, 0.0f, 1.0f));
    ct::Vector<Math::Vec4> colors;
    colors.push_back(Math::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
    colors.push_back(Math::Vec4(0.0f, 1.0f, 0.0f, 1.0f));
    colors.push_back(Math::Vec4(0.0f, 0.0f, 1.0f, 1.0f));
    ct::Vector<Math::Vec3> four;
    four.push_back(kP0);
    four.push_back(kP1);
    four.push_back(kP2);
    four.push_back(kP3);
    tool.begin(prisma::Topology::Triangles);
    tool.addTriangleFan(four, ct::Vector<Math::Vec2>(), colors, ct::Vector<Math::Vec2>(), normals);
    CHECK(tool.commit(out));
    CHECK(out.colors.size() == 6 && out.normals.size() == 6);
    CHECK(isVec4(out.colors[0], 1.0f, 0.0f, 0.0f, 1.0f));
    CHECK(isVec4(out.colors[1], 0.0f, 1.0f, 0.0f, 1.0f));
    CHECK(isVec4(out.colors[2], 0.0f, 0.0f, 1.0f, 1.0f));
}

void testIndexAndDeindex()
{
    ct::Vector<Math::Vec3> positions;
    positions.push_back(kP0);
    positions.push_back(kP1);
    positions.push_back(kP2);
    positions.push_back(kP3);
    ct::Vector<Math::Vec2> uvs;
    uvs.push_back(Math::Vec2(0.0f, 0.0f));
    uvs.push_back(Math::Vec2(1.0f, 0.0f));
    uvs.push_back(Math::Vec2(1.0f, 1.0f));
    uvs.push_back(Math::Vec2(0.0f, 1.0f));
    ct::Vector<Math::Vec3> normals;
    for (int i = 0; i < 4; ++i) normals.push_back(Math::Vec3(0.0f, 0.0f, 1.0f));

    SurfaceTool tool;
    tool.begin(prisma::Topology::Triangles);
    tool.addTriangleFan(positions, uvs, ct::Vector<Math::Vec4>(), ct::Vector<Math::Vec2>(),
            normals);
    CHECK(tool.getVertexCount() == 6);
    CHECK(tool.getIndexCount() == 0);

    SurfaceData soup;
    CHECK(tool.commit(soup));

    tool.index();
    CHECK(tool.getVertexCount() == 4);
    CHECK(tool.getIndexCount() == 6);
    SurfaceData merged;
    CHECK(tool.commit(merged));
    CHECK(merged.positions.size() == 4);
    CHECK(merged.uvs.size() == 4 && merged.normals.size() == 4);
    const std::uint32_t expectedIndices[6] = { 0, 1, 2, 0, 2, 3 };
    CHECK(merged.indices.size() == 6);
    for (int i = 0; i < 6; ++i) CHECK(merged.indices[i] == expectedIndices[i]);
    for (int i = 0; i < 4; ++i)
    {
        CHECK(isVec3(merged.positions[i], positions[i].x, positions[i].y, 0.0f));
        CHECK(isVec2(merged.uvs[i], uvs[i].x, uvs[i].y));
    }

    tool.index();
    CHECK(tool.getVertexCount() == 4);
    CHECK(tool.getIndexCount() == 6);

    tool.deindex();
    CHECK(tool.getVertexCount() == 6);
    CHECK(tool.getIndexCount() == 0);
    SurfaceData restored;
    CHECK(tool.commit(restored));
    CHECK(restored.indices.empty());
    CHECK(restored.positions.size() == 6);
    for (int i = 0; i < 6; ++i)
    {
        CHECK(isVec3(restored.positions[i], soup.positions[i].x, soup.positions[i].y,
                soup.positions[i].z));
        CHECK(isVec2(restored.uvs[i], soup.uvs[i].x, soup.uvs[i].y));
        CHECK(isVec3(restored.normals[i], 0.0f, 0.0f, 1.0f));
    }

    tool.deindex();
    CHECK(tool.getVertexCount() == 6);

    SurfaceTool different;
    different.begin(prisma::Topology::Triangles);
    different.setNormal(Math::Vec3(0.0f, 0.0f, 1.0f));
    different.addVertex(kP0);
    different.addVertex(kP1);
    different.addVertex(kP3);
    different.setNormal(Math::Vec3(0.0f, 1.0f, 0.0f));
    different.addVertex(kP0);
    different.addVertex(kP1);
    different.addVertex(kP3);
    different.index();
    CHECK(different.getVertexCount() == 6);
    CHECK(different.getIndexCount() == 6);

    SurfaceTool groups;
    groups.begin(prisma::Topology::Triangles);
    groups.addVertex(kP0);
    groups.setSmoothGroup(7);
    groups.addVertex(kP0);
    groups.setSmoothGroup(0);
    groups.addVertex(kP0);
    groups.index();
    CHECK(groups.getVertexCount() == 2);
    SurfaceData grouped;
    CHECK(groups.commit(grouped));
    CHECK(grouped.indices.size() == 3);
    CHECK(grouped.indices[0] == 0 && grouped.indices[1] == 1 && grouped.indices[2] == 0);

    SurfaceTool broken;
    broken.begin(prisma::Topology::Triangles);
    broken.addVertex(kP0);
    broken.addVertex(kP1);
    broken.addIndex(0);
    broken.addIndex(1);
    broken.addIndex(5);
    broken.deindex();
    CHECK(broken.getVertexCount() == 2);
    CHECK(broken.getIndexCount() == 3);
}

void addTriangleXY(SurfaceTool& tool, const Math::Vec3& a, const Math::Vec3& b, const Math::Vec3& c)
{
    tool.addVertex(a);
    tool.addVertex(b);
    tool.addVertex(c);
}

void testGenerateNormals()
{
    SurfaceTool tool;
    tool.begin(prisma::Topology::Triangles);
    addTriangleXY(tool, kP0, Math::Vec3(2.0f, 0.0f, 0.0f), Math::Vec3(0.0f, 2.0f, 0.0f));
    tool.generateNormals();
    SurfaceData out;
    CHECK(tool.commit(out));
    CHECK(out.normals.size() == 3);
    for (int i = 0; i < 3; ++i) CHECK(isVec3(out.normals[i], 0.0f, 0.0f, 1.0f));

    tool.generateNormals(true);
    CHECK(tool.commit(out));
    for (int i = 0; i < 3; ++i) CHECK(isVec3(out.normals[i], 0.0f, 0.0f, -1.0f));

    SurfaceTool flat;
    flat.begin(prisma::Topology::Triangles);
    flat.setSmoothGroup(SurfaceTool::kFlatShading);
    addTriangleXY(flat, kP0, kP1, kP3);
    flat.generateNormals();
    CHECK(flat.commit(out));
    for (int i = 0; i < 3; ++i) CHECK(isVec3(out.normals[i], 0.0f, 0.0f, 1.0f));

    const Math::Vec3 a(0.0f, 0.0f, 0.0f);
    const Math::Vec3 b(1.0f, 0.0f, 0.0f);
    const Math::Vec3 up(0.0f, 1.0f, 0.0f);
    const Math::Vec3 back(0.0f, 0.0f, 1.0f);
    const float diagonal = 0.70710678f;

    SurfaceTool smooth;
    smooth.begin(prisma::Topology::Triangles);
    addTriangleXY(smooth, a, b, up);
    addTriangleXY(smooth, b, a, back);
    smooth.generateNormals();
    CHECK(smooth.commit(out));
    CHECK(out.normals.size() == 6);
    CHECK(isVec3(out.normals[0], 0.0f, diagonal, diagonal));
    CHECK(isVec3(out.normals[1], 0.0f, diagonal, diagonal));
    CHECK(isVec3(out.normals[2], 0.0f, 0.0f, 1.0f));
    CHECK(isVec3(out.normals[3], 0.0f, diagonal, diagonal));
    CHECK(isVec3(out.normals[4], 0.0f, diagonal, diagonal));
    CHECK(isVec3(out.normals[5], 0.0f, 1.0f, 0.0f));

    SurfaceTool sharp;
    sharp.begin(prisma::Topology::Triangles);
    sharp.setSmoothGroup(SurfaceTool::kFlatShading);
    addTriangleXY(sharp, a, b, up);
    addTriangleXY(sharp, b, a, back);
    sharp.generateNormals();
    CHECK(sharp.commit(out));
    for (int i = 0; i < 3; ++i) CHECK(isVec3(out.normals[i], 0.0f, 0.0f, 1.0f));
    for (int i = 3; i < 6; ++i) CHECK(isVec3(out.normals[i], 0.0f, 1.0f, 0.0f));

    SurfaceTool groups;
    groups.begin(prisma::Topology::Triangles);
    groups.setSmoothGroup(1);
    addTriangleXY(groups, a, b, up);
    groups.setSmoothGroup(2);
    addTriangleXY(groups, b, a, back);
    groups.generateNormals();
    CHECK(groups.commit(out));
    for (int i = 0; i < 3; ++i) CHECK(isVec3(out.normals[i], 0.0f, 0.0f, 1.0f));
    for (int i = 3; i < 6; ++i) CHECK(isVec3(out.normals[i], 0.0f, 1.0f, 0.0f));

    SurfaceTool indexed;
    indexed.begin(prisma::Topology::Triangles);
    indexed.addVertex(kP0);
    indexed.addVertex(kP1);
    indexed.addVertex(kP2);
    indexed.addVertex(kP3);
    const std::uint32_t order[6] = { 0, 1, 2, 0, 2, 3 };
    for (int i = 0; i < 6; ++i) indexed.addIndex(order[i]);
    indexed.generateNormals();
    CHECK(indexed.getVertexCount() == 4);
    CHECK(indexed.getIndexCount() == 6);
    CHECK(indexed.commit(out));
    CHECK(out.normals.size() == 4);
    for (int i = 0; i < 4; ++i) CHECK(isVec3(out.normals[i], 0.0f, 0.0f, 1.0f));
    for (int i = 0; i < 6; ++i) CHECK(out.indices[i] == order[i]);

    SurfaceTool degenerate;
    degenerate.begin(prisma::Topology::Triangles);
    addTriangleXY(degenerate, kP0, kP0, kP1);
    degenerate.generateNormals();
    CHECK(degenerate.commit(out));
    for (int i = 0; i < 3; ++i) CHECK(isVec3(out.normals[i], 0.0f, 0.0f, 0.0f));

    SurfaceTool lines;
    lines.begin(prisma::Topology::Lines);
    lines.addVertex(kP0);
    lines.addVertex(kP1);
    lines.generateNormals();
    CHECK(lines.commit(out));
    CHECK(out.normals.empty());

    SurfaceTool partial;
    partial.begin(prisma::Topology::Triangles);
    partial.addVertex(kP0);
    partial.addVertex(kP1);
    partial.generateNormals();
    CHECK(partial.commit(out));
    CHECK(out.normals.empty());
}

void testGenerateTangents()
{
    SurfaceTool tool;
    tool.begin(prisma::Topology::Triangles);
    tool.setNormal(Math::Vec3(0.0f, 0.0f, 1.0f));
    CHECK(!tool.generateTangents());

    ct::Vector<Math::Vec3> positions;
    positions.push_back(kP0);
    positions.push_back(kP1);
    positions.push_back(kP2);
    positions.push_back(kP3);
    ct::Vector<Math::Vec2> uvs;
    uvs.push_back(Math::Vec2(0.0f, 1.0f));
    uvs.push_back(Math::Vec2(1.0f, 1.0f));
    uvs.push_back(Math::Vec2(1.0f, 0.0f));
    uvs.push_back(Math::Vec2(0.0f, 0.0f));
    ct::Vector<Math::Vec3> normals;
    for (int i = 0; i < 4; ++i) normals.push_back(Math::Vec3(0.0f, 0.0f, 1.0f));

    tool.begin(prisma::Topology::Triangles);
    tool.addTriangleFan(positions, uvs, ct::Vector<Math::Vec4>(), ct::Vector<Math::Vec2>(),
            normals);
    tool.index();
    CHECK(tool.generateTangents());

    SurfaceData out;
    CHECK(tool.commit(out));
    CHECK(out.tangents.size() == 4);
    for (int i = 0; i < 4; ++i) CHECK(isVec4(out.tangents[i], 1.0f, 0.0f, 0.0f, 1.0f));

    SurfaceTool noNormal;
    noNormal.begin(prisma::Topology::Triangles);
    noNormal.setUv(Math::Vec2(0.0f, 0.0f));
    addTriangleXY(noNormal, kP0, kP1, kP3);
    CHECK(!noNormal.generateTangents());

    SurfaceTool lines;
    lines.begin(prisma::Topology::Lines);
    lines.setNormal(Math::Vec3(0.0f, 0.0f, 1.0f));
    lines.setUv(Math::Vec2(0.0f, 0.0f));
    lines.addVertex(kP0);
    lines.addVertex(kP1);
    CHECK(!lines.generateTangents());

    SurfaceTool noVertices;
    noVertices.begin(prisma::Topology::Triangles);
    noVertices.setNormal(Math::Vec3(0.0f, 0.0f, 1.0f));
    noVertices.setUv(Math::Vec2(0.0f, 0.0f));
    CHECK(noVertices.generateTangents());
}

MeshId gStaticMesh;

MeshInfo infoOf(MeshStore& store, MeshId mesh, bool& ok)
{
    const MeshInfo* info = store.get(mesh);
    ok = info != nullptr;
    return info ? *info : MeshInfo();
}

void testNodeStatic(MeshStore& store, MaterialStore& materials)
{
    SceneTree tree;
    tree.initialize();
    ManualMesh3D* node = tree.getRoot()->createChild<ManualMesh3D>();

    CHECK(node->castTo<ManualMesh3D>() == node);
    CHECK(node->castTo<VisualInstance3D>() == node);
    CHECK(node->castTo<MeshInstance3D>() == nullptr);
    CHECK(!node->isDynamic());
    CHECK(node->getSurfaceCount() == 0);
    CHECK(!node->getMesh().valid());
    CHECK(!node->isSurfaceActive());

    CHECK(!node->surfaceEnd());
    node->surfaceAddVertex(kP0);
    node->surfaceSetColor(Math::Vec4(1.0f, 0.0f, 0.0f, 1.0f));

    const MaterialId red = materials.create("red");
    CHECK(node->surfaceBegin(prisma::Topology::Triangles, red));
    CHECK(node->isSurfaceActive());
    CHECK(!node->surfaceBegin(prisma::Topology::Lines));
    CHECK(!node->surfaceEnd());
    CHECK(node->isSurfaceActive());
    CHECK(node->addSurface(SurfaceData()) == -1);
    node->surfaceSetNormal(Math::Vec3(0.0f, 0.0f, 1.0f));
    node->surfaceAddVertex(Math::Vec3(-1.0f, 0.0f, 0.0f));
    node->surfaceSetUv(Math::Vec2(0.5f, 0.5f));
    node->surfaceAddVertex(Math::Vec3(2.0f, 0.0f, 0.0f));
    node->surfaceSetUv(Math::Vec2(1.0f, 0.0f));
    node->surfaceAddVertex(Math::Vec3(0.0f, 3.0f, 1.0f));
    CHECK(node->surfaceEnd());
    CHECK(!node->isSurfaceActive());
    CHECK(node->getSurfaceCount() == 1);
    CHECK(node->getMesh().valid());
    CHECK(node->getBase() == node->getMesh());
    CHECK(node->getSurfaceMaterial(0) == red);

    const SurfaceData* first = node->getSurfaceData(0);
    CHECK(first != nullptr);
    if (first)
    {
        CHECK(first->topology == prisma::Topology::Triangles);
        CHECK(first->positions.size() == 3);
        CHECK(first->normals.size() == 3);
        CHECK(first->uvs.size() == 3);
        CHECK(first->colors.empty() && first->tangents.empty() && first->uvs2.empty());
        CHECK(isVec2(first->uvs[0], 0.5f, 0.5f));
        CHECK(isVec2(first->uvs[1], 0.5f, 0.5f));
        CHECK(isVec2(first->uvs[2], 1.0f, 0.0f));
        CHECK(isVec3(first->normals[0], 0.0f, 0.0f, 1.0f));
    }
    CHECK(node->getSurfaceData(1) == nullptr);
    CHECK(node->getSurfaceData(-1) == nullptr);

    bool ok = false;
    MeshInfo info = infoOf(store, node->getMesh(), ok);
    CHECK(ok);
    CHECK(info.surfaces.size() == 1);
    if (info.surfaces.size() == 1)
    {
        CHECK(!info.surfaces[0].dynamic);
        CHECK(info.surfaces[0].vertexCount == 3);
        CHECK(info.surfaces[0].indexCount == 0);
        CHECK(info.surfaces[0].topology == prisma::Topology::Triangles);
        CHECK(info.surfaces[0].material == red);
        CHECK((info.surfaces[0].attributeMask & kVertexUv) != 0);
        CHECK((info.surfaces[0].attributeMask & kVertexUv2) == 0);
    }
    CHECK(isVec3(info.aabb.min, -1.0f, 0.0f, 0.0f));
    CHECK(isVec3(info.aabb.max, 2.0f, 3.0f, 1.0f));
    const Math::Box bounds = node->getAabb();
    CHECK(isVec3(bounds.min, -1.0f, 0.0f, 0.0f));
    CHECK(isVec3(bounds.max, 2.0f, 3.0f, 1.0f));

    CHECK(node->surfaceBegin(prisma::Topology::Lines));
    node->surfaceAddVertex(Math::Vec3(0.0f, 0.0f, 0.0f));
    node->surfaceSetColor(Math::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
    node->surfaceAddVertex(Math::Vec3(0.0f, 0.0f, 10.0f));
    node->surfaceSetColor(Math::Vec4(0.0f, 1.0f, 0.0f, 1.0f));
    node->surfaceAddVertex(Math::Vec3(0.0f, 5.0f, 10.0f));
    node->surfaceAddIndex(0);
    node->surfaceAddIndex(1);
    node->surfaceAddIndex(1);
    node->surfaceAddIndex(2);
    CHECK(node->surfaceEnd());
    CHECK(node->getSurfaceCount() == 2);
    CHECK(!node->getSurfaceMaterial(1).valid());
    const SurfaceData* lines = node->getSurfaceData(1);
    CHECK(lines != nullptr);
    if (lines)
    {
        CHECK(lines->topology == prisma::Topology::Lines);
        CHECK(lines->colors.size() == 3);
        CHECK(isVec4(lines->colors[0], 1.0f, 0.0f, 0.0f, 1.0f));
        CHECK(isVec4(lines->colors[1], 1.0f, 0.0f, 0.0f, 1.0f));
        CHECK(isVec4(lines->colors[2], 0.0f, 1.0f, 0.0f, 1.0f));
        CHECK(lines->indices.size() == 4);
    }
    info = infoOf(store, node->getMesh(), ok);
    CHECK(info.surfaces.size() == 2);
    if (info.surfaces.size() == 2)
    {
        CHECK(info.surfaces[1].topology == prisma::Topology::Lines);
        CHECK(info.surfaces[1].indexCount == 4);
    }
    CHECK(isVec3(info.aabb.max, 2.0f, 5.0f, 10.0f));

    CHECK(node->surfaceBegin(prisma::Topology::Triangles));
    node->surfaceAddVertex(kP0);
    node->surfaceAddVertex(kP1);
    node->surfaceAddVertex(kP2);
    node->surfaceAddIndex(0);
    node->surfaceAddIndex(1);
    node->surfaceAddIndex(7);
    CHECK(!node->surfaceEnd());
    CHECK(!node->isSurfaceActive());
    CHECK(node->getSurfaceCount() == 2);

    const MaterialId green = materials.create("green");
    node->setSurfaceMaterial(1, green);
    CHECK(node->getSurfaceMaterial(1) == green);
    info = infoOf(store, node->getMesh(), ok);
    if (info.surfaces.size() == 2) CHECK(info.surfaces[1].material == green);
    node->setSurfaceMaterial(0, MaterialId());
    CHECK(!node->getSurfaceMaterial(0).valid());
    node->setSurfaceMaterial(5, green);
    node->setSurfaceMaterial(-1, green);
    CHECK(!node->getSurfaceMaterial(5).valid());

    SurfaceTool tool;
    tool.begin(prisma::Topology::Triangles);
    tool.addQuad(kP0, kP1, kP2, kP3);
    SurfaceData made;
    CHECK(tool.commit(made));
    CHECK(node->addSurface(made, red) == 2);
    CHECK(node->getSurfaceCount() == 3);
    CHECK(node->getSurfaceMaterial(2) == red);
    SurfaceData bad;
    bad.positions.push_back(kP0);
    bad.indices.push_back(4);
    CHECK(node->addSurface(bad) == -1);
    CHECK(node->getSurfaceCount() == 3);

    const MeshId mesh = node->getMesh();
    node->clearSurfaces();
    CHECK(node->getSurfaceCount() == 0);
    CHECK(store.valid(mesh));
    info = infoOf(store, mesh, ok);
    CHECK(info.surfaces.empty());
    CHECK(node->getMesh() == mesh);

    CHECK(node->surfaceBegin(prisma::Topology::Triangles));
    node->surfaceAddVertex(kP0);
    node->clearSurfaces();
    CHECK(!node->isSurfaceActive());
    CHECK(!node->surfaceEnd());

    CHECK(node->addSurface(made) == 0);
    CHECK(node->getSurfaceCount() == 1);

    gStaticMesh = node->getMesh();
}

void testNodeDynamic(MeshStore& store)
{
    SceneTree tree;
    tree.initialize();
    ManualMesh3D* node = tree.getRoot()->createChild<ManualMesh3D>();

    node->setDynamic(8, 12);
    CHECK(node->isDynamic());
    CHECK(node->getVertexCapacity() == 8);
    CHECK(node->getIndexCapacity() == 12);

    CHECK(node->surfaceBegin(prisma::Topology::Triangles));
    node->surfaceSetNormal(Math::Vec3(0.0f, 0.0f, 1.0f));
    node->surfaceAddVertex(kP0);
    node->surfaceAddVertex(kP1);
    node->surfaceAddVertex(kP3);
    CHECK(node->surfaceEnd());
    CHECK(node->getSurfaceCount() == 1);

    const MeshId mesh = node->getMesh();
    CHECK(mesh.valid());
    bool ok = false;
    MeshInfo info = infoOf(store, mesh, ok);
    CHECK(ok);
    CHECK(info.surfaces.size() == 1);
    if (info.surfaces.size() == 1)
    {
        CHECK(info.surfaces[0].dynamic);
        CHECK(info.surfaces[0].vertexCapacity == 8);
        CHECK(info.surfaces[0].indexCapacity == 12);
        CHECK(info.surfaces[0].vertexCount == 3);
    }
    CHECK(isVec3(info.aabb.min, 0.0f, 0.0f, 0.0f));
    CHECK(isVec3(info.aabb.max, 1.0f, 1.0f, 0.0f));

    node->clearSurfaces();
    CHECK(node->getSurfaceCount() == 0);
    info = infoOf(store, mesh, ok);
    CHECK(info.surfaces.size() == 1);
    if (info.surfaces.size() == 1)
    {
        CHECK(info.surfaces[0].vertexCount == 0);
        CHECK(info.surfaces[0].indexCount == 0);
    }

    CHECK(node->surfaceBegin(prisma::Topology::Triangles));
    node->surfaceSetNormal(Math::Vec3(0.0f, 0.0f, 1.0f));
    node->surfaceAddVertex(Math::Vec3(0.0f, 0.0f, 0.0f));
    node->surfaceAddVertex(Math::Vec3(4.0f, 0.0f, 0.0f));
    node->surfaceAddVertex(Math::Vec3(4.0f, 2.0f, 0.0f));
    node->surfaceAddVertex(Math::Vec3(0.0f, 2.0f, 0.0f));
    const std::uint32_t order[6] = { 0, 1, 2, 0, 2, 3 };
    for (int i = 0; i < 6; ++i) node->surfaceAddIndex(order[i]);
    CHECK(node->surfaceEnd());
    CHECK(node->getSurfaceCount() == 1);
    CHECK(node->getMesh() == mesh);
    info = infoOf(store, mesh, ok);
    CHECK(info.surfaces.size() == 1);
    if (info.surfaces.size() == 1)
    {
        CHECK(info.surfaces[0].vertexCount == 4);
        CHECK(info.surfaces[0].indexCount == 6);
    }
    CHECK(isVec3(info.aabb.max, 4.0f, 2.0f, 0.0f));
    const Math::Box bounds = node->getAabb();
    CHECK(isVec3(bounds.max, 4.0f, 2.0f, 0.0f));

    node->clearSurfaces();
    for (int s = 0; s < 2; ++s)
    {
        CHECK(node->surfaceBegin(prisma::Topology::Triangles));
        node->surfaceSetNormal(Math::Vec3(0.0f, 0.0f, 1.0f));
        node->surfaceAddVertex(Math::Vec3(0.0f, 0.0f, static_cast<float>(s)));
        node->surfaceAddVertex(Math::Vec3(1.0f, 0.0f, static_cast<float>(s)));
        node->surfaceAddVertex(Math::Vec3(0.0f, 1.0f, static_cast<float>(s)));
        CHECK(node->surfaceEnd());
    }
    CHECK(node->getSurfaceCount() == 2);
    info = infoOf(store, mesh, ok);
    CHECK(info.surfaces.size() == 2);

    node->clearSurfaces();
    info = infoOf(store, mesh, ok);
    CHECK(info.surfaces.size() == 2);
    CHECK(info.surfaces[0].vertexCount == 0 && info.surfaces[1].vertexCount == 0);

    CHECK(node->surfaceBegin(prisma::Topology::Triangles));
    node->surfaceSetNormal(Math::Vec3(0.0f, 0.0f, 1.0f));
    node->surfaceAddVertex(Math::Vec3(0.0f, 0.0f, 5.0f));
    node->surfaceAddVertex(Math::Vec3(1.0f, 0.0f, 5.0f));
    node->surfaceAddVertex(Math::Vec3(0.0f, 1.0f, 5.0f));
    CHECK(node->surfaceEnd());
    info = infoOf(store, mesh, ok);
    CHECK(info.surfaces.size() == 2);
    CHECK(info.surfaces[0].vertexCount == 3);
    CHECK(info.surfaces[1].vertexCount == 0);
    CHECK(isVec3(info.aabb.min, 0.0f, 0.0f, 5.0f));
    CHECK(isVec3(info.aabb.max, 1.0f, 1.0f, 5.0f));

    node->clearSurfaces();
    CHECK(node->surfaceBegin(prisma::Topology::Triangles));
    node->surfaceSetNormal(Math::Vec3(0.0f, 0.0f, 1.0f));
    for (int i = 0; i < 9; ++i)
        node->surfaceAddVertex(Math::Vec3(static_cast<float>(i), 0.0f, 0.0f));
    CHECK(!node->surfaceEnd());
    CHECK(!node->isSurfaceActive());
    CHECK(node->getSurfaceCount() == 0);
    info = infoOf(store, mesh, ok);
    CHECK(info.surfaces.size() == 2);
    CHECK(info.surfaces[0].vertexCount == 0);

    CHECK(node->surfaceBegin(prisma::Topology::Triangles));
    node->surfaceSetNormal(Math::Vec3(0.0f, 0.0f, 1.0f));
    node->surfaceAddVertex(kP0);
    node->surfaceAddVertex(kP1);
    node->surfaceAddVertex(kP3);
    for (int i = 0; i < 13; ++i) node->surfaceAddIndex(0);
    CHECK(!node->surfaceEnd());
    CHECK(node->getSurfaceCount() == 0);

    CHECK(node->surfaceBegin(prisma::Topology::Triangles));
    node->surfaceSetUv(Math::Vec2(0.0f, 0.0f));
    node->surfaceAddVertex(kP0);
    node->surfaceAddVertex(kP1);
    node->surfaceAddVertex(kP3);
    CHECK(!node->surfaceEnd());
    CHECK(node->getSurfaceCount() == 0);

    CHECK(node->surfaceBegin(prisma::Topology::Triangles));
    node->surfaceSetNormal(Math::Vec3(0.0f, 0.0f, 1.0f));
    node->surfaceAddVertex(kP0);
    node->surfaceAddVertex(kP2);
    node->surfaceAddVertex(kP3);
    CHECK(node->surfaceEnd());
    CHECK(node->getSurfaceCount() == 1);
    CHECK(node->getMesh() == mesh);
    info = infoOf(store, mesh, ok);
    CHECK(info.surfaces[0].vertexCount == 3);

    node->setDynamic(0, 0);
    CHECK(!node->isDynamic());
    CHECK(node->getSurfaceCount() == 0);
    info = infoOf(store, mesh, ok);
    CHECK(info.surfaces.empty());
}

void testNodeCollision(MeshStore& store)
{
    (void) store;
    SceneTree tree;
    tree.initialize();
    ManualMesh3D* node = tree.getRoot()->createChild<ManualMesh3D>();

    CollisionMesh none;
    CHECK(!node->buildCollision(none));
    CHECK(!none.valid());

    SurfaceTool tool;
    tool.begin(prisma::Topology::Triangles);
    tool.addQuad(Math::Vec3(-1.0f, 0.0f, -1.0f), Math::Vec3(-1.0f, 0.0f, 1.0f),
            Math::Vec3(1.0f, 0.0f, 1.0f), Math::Vec3(1.0f, 0.0f, -1.0f));
    tool.generateNormals();
    SurfaceData floor;
    CHECK(tool.commit(floor));
    CHECK(floor.normals.size() == 6);
    for (int i = 0; i < 6; ++i) CHECK(isVec3(floor.normals[i], 0.0f, 1.0f, 0.0f));
    CHECK(node->addSurface(floor) == 0);

    node->surfaceBegin(prisma::Topology::Lines);
    node->surfaceAddVertex(Math::Vec3(0.0f, 0.0f, 0.0f));
    node->surfaceAddVertex(Math::Vec3(0.0f, 9.0f, 0.0f));
    CHECK(node->surfaceEnd());
    CHECK(node->getSurfaceCount() == 2);

    CollisionMesh collision;
    CHECK(node->buildCollision(collision));
    CHECK(collision.valid());
    CHECK(collision.triangleCount() == 2);
    CHECK(isVec3(collision.bounds().min, -1.0f, 0.0f, -1.0f));
    CHECK(isVec3(collision.bounds().max, 1.0f, 0.0f, 1.0f));

    SweepHit hit;
    CHECK(collision.raycast(Math::Ray(Math::Vec3(0.5f, 3.0f, 0.25f), Math::Vec3(0.0f, -1.0f, 0.0f)),
            10.0f, hit));
    CHECK(near(hit.t, 3.0f));
    CHECK(isVec3(hit.normal, 0.0f, 1.0f, 0.0f));
    CHECK(!collision.raycast(Math::Ray(Math::Vec3(5.0f, 3.0f, 0.0f), Math::Vec3(0.0f, -1.0f, 0.0f)),
            10.0f, hit));
}

} // namespace

int main(int argc, char** argv)
{
    const bool vulkan = argc > 1 && strcmp(argv[1], "vulkan") == 0;

    testAttributes();
    testTrianglesAndQuads();
    testTriangleFan();
    testIndexAndDeindex();
    testGenerateNormals();
    testGenerateTangents();

    CHECK(Device::headlessSupported());
    if (!Device::headlessSupported())
    {
        printf("%d checks failed\n", failures);
        return 1;
    }

    DeviceDesc deviceDesc;
    deviceDesc.title = "manual mesh test";
    deviceDesc.driver = vulkan ? prisma::DriverType::Vulkan : prisma::DriverType::OpenGL;
    deviceDesc.width = 64;
    deviceDesc.height = 48;
    deviceDesc.headless = true;

    Device* device = Device::create(deviceDesc);
    CHECK(device != nullptr);
    if (!device) return 1;

    {
        MeshStore meshes(device->driver());
        TextureStore textures(device->driver());
        MaterialStore materials(textures);
        RenderingServer server(device->driver(), meshes, textures, materials);

        testNodeStatic(meshes, materials);
        CHECK(gStaticMesh.valid());
        CHECK(!meshes.valid(gStaticMesh));
        testNodeDynamic(meshes);
        testNodeCollision(meshes);
    }

    Device::destroy(device);

    if (failures) printf("%d checks failed\n", failures);
    else
        printf("ok\n");
    return failures ? 1 : 0;
}
