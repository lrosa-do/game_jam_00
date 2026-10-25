#include "Check.h"

#include "prisma3d/Device.h"
#include "prisma3d/render/MeshStore.h"
#include "prisma3d/scene/Collision.h"

#include <cstdint>
#include <math.h>
#include <string.h>

using namespace prisma3d;

namespace
{

bool near(float a, float b)
{
    return fabsf(a - b) < 1e-4f;
}

SurfaceData triangle()
{
    SurfaceData data;
    data.positions.push_back(Math::Vec3(-1.0f, -2.0f, 0.0f));
    data.positions.push_back(Math::Vec3(1.0f, -2.0f, 0.5f));
    data.positions.push_back(Math::Vec3(0.0f, 3.0f, -0.5f));
    return data;
}

SurfaceData fullTriangle()
{
    SurfaceData data = triangle();
    for (int i = 0; i < 3; ++i)
    {
        data.normals.push_back(Math::Vec3(0.0f, 0.0f, 1.0f));
        data.colors.push_back(Math::Vec4(0.5f, 0.25f, 0.125f, 1.0f));
        data.tangents.push_back(Math::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
        data.uvs.push_back(Math::Vec2(0.0f, 1.0f));
        data.uvs2.push_back(Math::Vec2(1.0f, 0.0f));
    }
    data.indices.push_back(0);
    data.indices.push_back(1);
    data.indices.push_back(2);
    return data;
}

SurfaceData boxSurface()
{
    SurfaceData data;
    for (int i = 0; i < 8; ++i)
    {
        data.positions.push_back(
                Math::Vec3((i & 1) ? 1.0f : -1.0f, (i & 2) ? 2.0f : -2.0f, (i & 4) ? 3.0f : -3.0f));
    }
    static const std::uint32_t indices[36] = { 1, 3, 7, 1, 7, 5, 0, 4, 6, 0, 6, 2, 2, 6, 7, 2, 7, 3,
        0, 1, 5, 0, 5, 4, 4, 5, 7, 4, 7, 6, 0, 2, 3, 0, 3, 1 };
    for (int i = 0; i < 36; ++i) data.indices.push_back(indices[i]);
    return data;
}

bool sameVec(const Math::Vec3& a, float x, float y, float z)
{
    return a.x == x && a.y == y && a.z == z;
}

bool readBuffer(prisma::Driver* driver, prisma::BufferHandle buffer, std::uint32_t size,
        ct::Vector<std::uint8_t>& out)
{
    out.resize(size);
    return driver->readBuffer(buffer, 0, size, out.data());
}

} // namespace

int main(int argc, char** argv)
{
    const bool vulkan = argc > 1 && strcmp(argv[1], "vulkan") == 0;

    CHECK(Device::headlessSupported());
    if (!Device::headlessSupported()) return 1;

    DeviceDesc deviceDesc;
    deviceDesc.title = "mesh store test";
    deviceDesc.driver = vulkan ? prisma::DriverType::Vulkan : prisma::DriverType::OpenGL;
    deviceDesc.width = 64;
    deviceDesc.height = 48;
    deviceDesc.headless = true;

    Device* device = Device::create(deviceDesc);
    CHECK(device != nullptr);
    if (!device) return 1;

    prisma::Driver* driver = device->driver();

    {
        MeshStore store(driver);
        CHECK(MeshStore::singleton() == &store);
        CHECK(store.driver() == driver);
        CHECK(store.count() == 0);

        CHECK(MeshStore::vertexStride(kVertexPosition | kVertexNormal | kVertexColor) == 40);
        CHECK(MeshStore::vertexStride(kVertexPosition | kVertexNormal | kVertexColor |
                                      kVertexTangent | kVertexUv | kVertexUv2) == 72);
        CHECK(MeshStore::vertexStride(kVertexPosition) == 12);
        const std::uint32_t full = kVertexPosition | kVertexNormal | kVertexColor | kVertexTangent |
                                   kVertexUv | kVertexUv2;
        CHECK(MeshStore::attributeOffset(full, kVertexPosition) == 0);
        CHECK(MeshStore::attributeOffset(full, kVertexNormal) == 12);
        CHECK(MeshStore::attributeOffset(full, kVertexColor) == 24);
        CHECK(MeshStore::attributeOffset(full, kVertexTangent) == 40);
        CHECK(MeshStore::attributeOffset(full, kVertexUv) == 56);
        CHECK(MeshStore::attributeOffset(full, kVertexUv2) == 64);
        CHECK(MeshStore::attributeOffset(kVertexPosition | kVertexNormal | kVertexColor | kVertexUv,
                      kVertexUv) == 40);

        const std::uint32_t skinned = full | kVertexJoints | kVertexWeights;
        CHECK(MeshStore::vertexStride(skinned) == MeshStore::vertexStride(full) + 20);
        CHECK(MeshStore::vertexStride(skinned) == 92);
        CHECK(MeshStore::attributeOffset(skinned, kVertexJoints) == 72);
        CHECK(MeshStore::attributeOffset(skinned, kVertexWeights) == 76);
        CHECK(MeshStore::attributeOffset(skinned, kVertexUv2) == 64);
        CHECK(MeshStore::attributeOffset(kVertexPosition | kVertexJoints | kVertexWeights,
                      kVertexWeights) == 16);

        const MeshId mesh = store.create("triangle");
        CHECK(mesh.valid());
        CHECK(store.valid(mesh));
        CHECK(store.count() == 1);
        const MeshInfo* info = store.get(mesh);
        CHECK(info != nullptr);
        if (info)
        {
            CHECK(info->surfaces.empty());
            CHECK(strcmp(info->name.c_str(), "triangle") == 0);
            CHECK(info->path.empty());
        }
        CHECK(!store.valid(MeshId()));
        CHECK(store.get(MeshId()) == nullptr);

        CHECK(store.addSurface(mesh, triangle(), "positions only"));
        info = store.get(mesh);
        CHECK(info && info->surfaces.size() == 1);
        if (info && info->surfaces.size() == 1)
        {
            const SurfaceInfo& surface = info->surfaces[0];
            CHECK(surface.attributeMask == (kVertexPosition | kVertexNormal | kVertexColor));
            CHECK(surface.vertexStride == 40);
            CHECK(surface.vertexCount == 3);
            CHECK(surface.indexCount == 0);
            CHECK(!surface.indexBuffer.valid());
            CHECK(surface.vertexBuffer.valid());
            CHECK(surface.topology == prisma::Topology::Triangles);
            CHECK(sameVec(surface.aabb.min, -1.0f, -2.0f, -0.5f));
            CHECK(sameVec(surface.aabb.max, 1.0f, 3.0f, 0.5f));
            CHECK(sameVec(info->aabb.min, -1.0f, -2.0f, -0.5f));

            ct::Vector<std::uint8_t> bytes;
            CHECK(readBuffer(driver, surface.vertexBuffer, 3 * 40, bytes));
            if (bytes.size() == 120)
            {
                float vertex[10];
                memcpy(vertex, bytes.data() + 40, sizeof(vertex));
                CHECK(vertex[0] == 1.0f && vertex[1] == -2.0f && vertex[2] == 0.5f);
                CHECK(vertex[3] == 0.0f && vertex[4] == 0.0f && vertex[5] == 0.0f);
                CHECK(vertex[6] == 1.0f && vertex[7] == 1.0f && vertex[8] == 1.0f &&
                        vertex[9] == 1.0f);
            }
        }

        const MeshId second = store.create();
        CHECK(second.valid() && second != mesh);
        CHECK(store.count() == 2);
        CHECK(store.addSurface(second, fullTriangle(), "full"));
        const MeshInfo* secondInfo = store.get(second);
        CHECK(secondInfo && secondInfo->surfaces.size() == 1);
        if (secondInfo && secondInfo->surfaces.size() == 1)
        {
            const SurfaceInfo& surface = secondInfo->surfaces[0];
            CHECK(surface.attributeMask == (kVertexPosition | kVertexNormal | kVertexColor |
                                                   kVertexTangent | kVertexUv | kVertexUv2));
            CHECK(surface.vertexStride == 72);
            CHECK(surface.indexCount == 3);
            CHECK(surface.indexBuffer.valid());
            CHECK(surface.indexFormat == prisma::IndexFormat::UInt16);

            ct::Vector<std::uint8_t> bytes;
            CHECK(readBuffer(driver, surface.vertexBuffer, 3 * 72, bytes));
            if (bytes.size() == 216)
            {
                float vertex[18];
                memcpy(vertex, bytes.data(), sizeof(vertex));
                CHECK(vertex[0] == -1.0f && vertex[1] == -2.0f && vertex[2] == 0.0f);
                CHECK(vertex[5] == 1.0f);
                CHECK(vertex[6] == 0.5f && vertex[7] == 0.25f && vertex[8] == 0.125f);
                CHECK(vertex[10] == 1.0f && vertex[13] == 1.0f);
                CHECK(vertex[14] == 0.0f && vertex[15] == 1.0f);
                CHECK(vertex[16] == 1.0f && vertex[17] == 0.0f);
            }
            ct::Vector<std::uint8_t> indices;
            CHECK(readBuffer(driver, surface.indexBuffer, 6, indices));
            if (indices.size() == 6)
            {
                std::uint16_t values[3];
                memcpy(values, indices.data(), sizeof(values));
                CHECK(values[0] == 0 && values[1] == 1 && values[2] == 2);
            }
        }

        {
            SurfaceData rigged = triangle();
            rigged.joints.push_back(0x04030201u);
            rigged.joints.push_back(0x00000007u);
            rigged.joints.push_back(0xAABBCCDDu);
            rigged.weights.push_back(Math::Vec4(1.0f, 0.0f, 0.0f, 0.0f));
            rigged.weights.push_back(Math::Vec4(0.5f, 0.5f, 0.0f, 0.0f));
            rigged.weights.push_back(Math::Vec4(0.25f, 0.25f, 0.25f, 0.25f));
            const MeshId skin = store.create("skin");
            CHECK(store.addSurface(skin, rigged, "skin"));
            const MeshInfo* skinInfo = store.get(skin);
            CHECK(skinInfo && skinInfo->surfaces.size() == 1);
            if (skinInfo && skinInfo->surfaces.size() == 1)
            {
                const SurfaceInfo& surface = skinInfo->surfaces[0];
                CHECK(surface.attributeMask == (kVertexPosition | kVertexNormal | kVertexColor |
                                                       kVertexJoints | kVertexWeights));
                CHECK(surface.vertexStride == 60);

                ct::Vector<std::uint8_t> bytes;
                CHECK(readBuffer(driver, surface.vertexBuffer, 3 * 60, bytes));
                if (bytes.size() == 180)
                {
                    const std::uint32_t jointsOffset =
                            MeshStore::attributeOffset(surface.attributeMask, kVertexJoints);
                    const std::uint32_t weightsOffset =
                            MeshStore::attributeOffset(surface.attributeMask, kVertexWeights);
                    CHECK(jointsOffset == 40 && weightsOffset == 44);
                    CHECK(bytes[jointsOffset] == 0x01 && bytes[jointsOffset + 1] == 0x02 &&
                            bytes[jointsOffset + 2] == 0x03 && bytes[jointsOffset + 3] == 0x04);
                    std::uint32_t packed;
                    memcpy(&packed, bytes.data() + 60 + jointsOffset, sizeof(packed));
                    CHECK(packed == 7u);
                    memcpy(&packed, bytes.data() + 120 + jointsOffset, sizeof(packed));
                    CHECK(packed == 0xAABBCCDDu);
                    float weights[4];
                    memcpy(weights, bytes.data() + 60 + weightsOffset, sizeof(weights));
                    CHECK(weights[0] == 0.5f && weights[1] == 0.5f && weights[2] == 0.0f &&
                            weights[3] == 0.0f);
                    memcpy(weights, bytes.data() + 120 + weightsOffset, sizeof(weights));
                    CHECK(weights[0] == 0.25f && weights[3] == 0.25f);
                    float position[3];
                    memcpy(position, bytes.data() + 60, sizeof(position));
                    CHECK(position[0] == 1.0f && position[1] == -2.0f && position[2] == 0.5f);
                }
            }
            store.destroy(skin);

            SurfaceData bare = triangle();
            bare.joints = rigged.joints;
            const MeshId rejected = store.create("rejected");
            CHECK(!store.addSurface(rejected, bare));
            bare = triangle();
            bare.weights = rigged.weights;
            CHECK(!store.addSurface(rejected, bare));
            bare = rigged;
            bare.joints.pop_back();
            CHECK(!store.addSurface(rejected, bare));
            bare = rigged;
            bare.weights.pop_back();
            CHECK(!store.addSurface(rejected, bare));
            bare = rigged;
            bare.weights.push_back(Math::Vec4(1.0f, 0.0f, 0.0f, 0.0f));
            CHECK(!store.addSurface(rejected, bare));
            CHECK(store.get(rejected)->surfaces.empty());
            store.destroy(rejected);
        }

        {
            SurfaceData large;
            for (int i = 0; i < 70000; ++i)
                large.positions.push_back(Math::Vec3(static_cast<float>(i), 0.0f, 0.0f));
            large.indices.push_back(0);
            large.indices.push_back(1);
            large.indices.push_back(69999);
            const MeshId big = store.create("big");
            CHECK(store.addSurface(big, large));
            const MeshInfo* bigInfo = store.get(big);
            CHECK(bigInfo && bigInfo->surfaces.size() == 1);
            if (bigInfo && bigInfo->surfaces.size() == 1)
                CHECK(bigInfo->surfaces[0].indexFormat == prisma::IndexFormat::UInt32);

            SurfaceData edge;
            for (int i = 0; i < 65536; ++i)
                edge.positions.push_back(Math::Vec3(static_cast<float>(i), 0.0f, 0.0f));
            edge.indices.push_back(65535);
            edge.indices.push_back(0);
            edge.indices.push_back(1);
            const MeshId edgeMesh = store.create("edge");
            CHECK(store.addSurface(edgeMesh, edge));
            const MeshInfo* edgeInfo = store.get(edgeMesh);
            CHECK(edgeInfo && edgeInfo->surfaces.size() == 1);
            if (edgeInfo && edgeInfo->surfaces.size() == 1)
                CHECK(edgeInfo->surfaces[0].indexFormat == prisma::IndexFormat::UInt16);

            edge.indices.push_back(65536);
            CHECK(!store.addSurface(edgeMesh, edge));
            CHECK(store.get(edgeMesh)->surfaces.size() == 1);
            store.destroy(big);
            store.destroy(edgeMesh);
        }

        {
            const MeshId bad = store.create("bad");
            SurfaceData data = triangle();
            data.normals.push_back(Math::Vec3(0.0f, 0.0f, 1.0f));
            CHECK(!store.addSurface(bad, data));
            data = triangle();
            data.colors.push_back(Math::Vec4(1.0f, 1.0f, 1.0f, 1.0f));
            CHECK(!store.addSurface(bad, data));
            data = triangle();
            data.uvs.push_back(Math::Vec2(0.0f, 0.0f));
            data.uvs.push_back(Math::Vec2(0.0f, 0.0f));
            CHECK(!store.addSurface(bad, data));
            data = triangle();
            data.tangents.push_back(Math::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
            CHECK(!store.addSurface(bad, data));
            data = triangle();
            data.uvs2.push_back(Math::Vec2(0.0f, 0.0f));
            CHECK(!store.addSurface(bad, data));
            data = triangle();
            data.indices.push_back(3);
            CHECK(!store.addSurface(bad, data));
            CHECK(!store.addSurface(bad, SurfaceData()));
            CHECK(!store.addSurface(MeshId(), triangle()));
            CHECK(store.get(bad)->surfaces.empty());
            store.destroy(bad);
        }

        {
            const MeshId lines = store.create("lines");
            SurfaceData data = triangle();
            data.topology = prisma::Topology::Lines;
            CHECK(store.addSurface(lines, data));
            CHECK(store.get(lines)->surfaces[0].topology == prisma::Topology::Lines);
            store.destroy(lines);
        }

        {
            const MeshId merged = store.create("merged");
            SurfaceData a = triangle();
            SurfaceData b;
            b.positions.push_back(Math::Vec3(5.0f, 5.0f, 5.0f));
            b.positions.push_back(Math::Vec3(6.0f, 7.0f, 8.0f));
            b.positions.push_back(Math::Vec3(5.5f, 5.5f, 5.5f));
            CHECK(store.addSurface(merged, a));
            CHECK(store.addSurface(merged, b));
            const MeshInfo* mergedInfo = store.get(merged);
            CHECK(mergedInfo && mergedInfo->surfaces.size() == 2);
            if (mergedInfo)
            {
                CHECK(sameVec(mergedInfo->aabb.min, -1.0f, -2.0f, -0.5f));
                CHECK(sameVec(mergedInfo->aabb.max, 6.0f, 7.0f, 8.0f));
            }
            store.clearSurfaces(merged);
            mergedInfo = store.get(merged);
            CHECK(mergedInfo && mergedInfo->surfaces.empty());
            CHECK(store.valid(merged));
            CHECK(store.addSurface(merged, b));
            CHECK(sameVec(store.get(merged)->aabb.min, 5.0f, 5.0f, 5.0f));
            CHECK(sameVec(store.get(merged)->aabb.max, 6.0f, 7.0f, 8.0f));
        }

        {
            ct::Vector<Math::Vec3> points;
            points.push_back(Math::Vec3(1.0f, 2.0f, 3.0f));
            points.push_back(Math::Vec3(-4.0f, 0.0f, 9.0f));
            const Math::Box box = MeshStore::computeAabb(points);
            CHECK(sameVec(box.min, -4.0f, 0.0f, 3.0f));
            CHECK(sameVec(box.max, 1.0f, 2.0f, 9.0f));
        }

        {
            SurfaceData quad;
            quad.positions.push_back(Math::Vec3(0.0f, 0.0f, 0.0f));
            quad.positions.push_back(Math::Vec3(1.0f, 0.0f, 0.0f));
            quad.positions.push_back(Math::Vec3(1.0f, 1.0f, 0.0f));
            quad.positions.push_back(Math::Vec3(0.0f, 1.0f, 0.0f));
            for (int i = 0; i < 4; ++i) quad.normals.push_back(Math::Vec3(0.0f, 0.0f, 1.0f));
            quad.uvs.push_back(Math::Vec2(0.0f, 1.0f));
            quad.uvs.push_back(Math::Vec2(1.0f, 1.0f));
            quad.uvs.push_back(Math::Vec2(1.0f, 0.0f));
            quad.uvs.push_back(Math::Vec2(0.0f, 0.0f));
            const std::uint32_t order[] = { 0, 1, 2, 0, 2, 3 };
            for (int i = 0; i < 6; ++i) quad.indices.push_back(order[i]);

            SurfaceData down = quad;
            CHECK(MeshStore::generateTangents(down));
            CHECK(down.tangents.size() == 4);
            for (std::size_t i = 0; i < down.tangents.size(); ++i)
            {
                CHECK(near(down.tangents[i].x, 1.0f) && near(down.tangents[i].y, 0.0f) &&
                        near(down.tangents[i].z, 0.0f));
                CHECK(near(down.tangents[i].w, 1.0f));
            }

            SurfaceData up = quad;
            for (std::size_t i = 0; i < up.uvs.size(); ++i) up.uvs[i].y = 1.0f - up.uvs[i].y;
            CHECK(MeshStore::generateTangents(up));
            for (std::size_t i = 0; i < up.tangents.size(); ++i)
                CHECK(near(up.tangents[i].x, 1.0f) && near(up.tangents[i].w, -1.0f));

            SurfaceData mirrored = quad;
            for (std::size_t i = 0; i < mirrored.uvs.size(); ++i)
                mirrored.uvs[i].x = 1.0f - mirrored.uvs[i].x;
            CHECK(MeshStore::generateTangents(mirrored));
            for (std::size_t i = 0; i < mirrored.tangents.size(); ++i)
                CHECK(near(mirrored.tangents[i].x, -1.0f) && near(mirrored.tangents[i].w, -1.0f));

            SurfaceData soup;
            for (int i = 0; i < 6; ++i)
            {
                soup.positions.push_back(quad.positions[order[i]]);
                soup.normals.push_back(quad.normals[order[i]]);
                soup.uvs.push_back(quad.uvs[order[i]]);
            }
            CHECK(MeshStore::generateTangents(soup));
            CHECK(soup.tangents.size() == 6);
            for (std::size_t i = 0; i < soup.tangents.size(); ++i)
                CHECK(near(soup.tangents[i].x, 1.0f) && near(soup.tangents[i].w, 1.0f));

            {
                const int cells = 48;
                SurfaceData field;
                for (int y = 0; y <= cells; ++y)
                {
                    for (int x = 0; x <= cells; ++x)
                    {
                        const float fx = static_cast<float>(x) * 0.25f;
                        const float fy = static_cast<float>(y) * 0.25f;
                        field.positions.push_back(
                                Math::Vec3(fx, fy, 0.6f * sinf(fx * 0.9f) * cosf(fy * 0.7f)));
                        const float dx = 0.6f * 0.9f * cosf(fx * 0.9f) * cosf(fy * 0.7f);
                        const float dy = -0.6f * 0.7f * sinf(fx * 0.9f) * sinf(fy * 0.7f);
                        field.normals.push_back(Math::Vec3(-dx, -dy, 1.0f).Normalized());
                        field.uvs.push_back(Math::Vec2(fx / 12.0f, fy / 12.0f));
                    }
                }
                for (int y = 0; y < cells; ++y)
                {
                    for (int x = 0; x < cells; ++x)
                    {
                        const std::uint32_t a = static_cast<std::uint32_t>(y * (cells + 1) + x);
                        const std::uint32_t b = a + 1;
                        const std::uint32_t c = a + static_cast<std::uint32_t>(cells + 1);
                        const std::uint32_t d = c + 1;
                        const std::uint32_t corners[6] = { a, b, d, a, d, c };
                        for (int i = 0; i < 6; ++i) field.indices.push_back(corners[i]);
                    }
                }

                auto wellFormed =
                        [](const SurfaceData& surface, const ct::Vector<SurfaceLod>& levels)
                {
                    std::size_t previousCount = surface.indices.size();
                    float previousEdge = 0.0f;
                    for (std::size_t l = 0; l < levels.size(); ++l)
                    {
                        const SurfaceLod& level = levels[l];
                        if (level.indices.empty() || level.indices.size() % 3 != 0) return false;
                        if (level.indices.size() * 4 >= previousCount * 3) return false;
                        if (level.edgeLength <= 0.0f || level.edgeLength < previousEdge)
                            return false;
                        for (std::size_t i = 0; i < level.indices.size(); ++i)
                            if (level.indices[i] >= surface.positions.size()) return false;
                        previousCount = level.indices.size();
                        previousEdge = level.edgeLength;
                    }
                    return true;
                };

                ct::Vector<SurfaceLod> lods;
                CHECK(MeshStore::generateLods(field, 20.0f, lods));
                CHECK(lods.size() >= 3);
                CHECK(wellFormed(field, lods));
                if (!lods.empty())
                {
                    CHECK(lods[0].indices.size() * 2 <= field.indices.size() + 6);
                    CHECK(lods[0].edgeLength < lods.back().edgeLength);
                    printf("lods of a %d-triangle field:",
                            static_cast<int>(field.indices.size() / 3));
                    for (std::size_t l = 0; l < lods.size(); ++l)
                        printf(" %d (%.4f)", static_cast<int>(lods[l].indices.size() / 3),
                                lods[l].edgeLength);
                    printf("\n");
                }

                ct::Vector<SurfaceLod> again;
                CHECK(MeshStore::generateLods(field, 20.0f, again));
                CHECK(again.size() == lods.size());
                for (std::size_t l = 0; l < lods.size() && l < again.size(); ++l)
                {
                    CHECK(again[l].edgeLength == lods[l].edgeLength);
                    CHECK(again[l].indices.size() == lods[l].indices.size());
                }

                SurfaceData soup;
                for (std::size_t i = 0; i < field.indices.size(); ++i)
                {
                    soup.positions.push_back(field.positions[field.indices[i]]);
                    soup.normals.push_back(field.normals[field.indices[i]]);
                    soup.uvs.push_back(field.uvs[field.indices[i]]);
                    soup.indices.push_back(static_cast<std::uint32_t>(i));
                }
                ct::Vector<SurfaceLod> soupLods;
                CHECK(MeshStore::generateLods(soup, 20.0f, soupLods));
                CHECK(soupLods.size() == lods.size());
                CHECK(wellFormed(soup, soupLods));
                for (std::size_t l = 0; l < lods.size() && l < soupLods.size(); ++l)
                    CHECK(soupLods[l].indices.size() == lods[l].indices.size());

                SurfaceData islands = soup;
                for (std::size_t i = 0; i < islands.uvs.size(); ++i)
                    islands.uvs[i] = Math::Vec2(static_cast<float>(i), static_cast<float>(i % 3));
                ct::Vector<SurfaceLod> islandLods;
                CHECK(MeshStore::generateLods(islands, 20.0f, islandLods));
                CHECK(islandLods.size() < lods.size());

                SurfaceData coloured = field;
                for (std::size_t i = 0; i < coloured.positions.size(); ++i)
                    coloured.colors.push_back(Math::Vec4(coloured.positions[i].x / 12.0f, 0.5f,
                            coloured.positions[i].y / 12.0f, 1.0f));
                ct::Vector<SurfaceLod> colouredLods;
                CHECK(MeshStore::generateLods(coloured, 20.0f, colouredLods));
                CHECK(colouredLods.size() >= 3);
                CHECK(wellFormed(coloured, colouredLods));

                SurfaceData unlit = field;
                unlit.normals.clear();
                ct::Vector<SurfaceLod> unlitLods;
                CHECK(MeshStore::generateLods(unlit, 20.0f, unlitLods));
                CHECK(unlitLods.size() >= 3);
                CHECK(wellFormed(unlit, unlitLods));

                SurfaceData unindexed = field;
                unindexed.indices.clear();
                CHECK(!MeshStore::generateLods(unindexed, 20.0f, lods));
                CHECK(lods.empty());
                SurfaceData ragged = field;
                ragged.indices.pop_back();
                CHECK(!MeshStore::generateLods(ragged, 20.0f, lods));
                SurfaceData outside = field;
                outside.indices[5] = static_cast<std::uint32_t>(outside.positions.size());
                CHECK(!MeshStore::generateLods(outside, 20.0f, lods));
                SurfaceData lineList = field;
                lineList.topology = prisma::Topology::Lines;
                CHECK(!MeshStore::generateLods(lineList, 20.0f, lods));

                ct::Vector<SurfaceLod> quadLods;
                CHECK(MeshStore::generateLods(quad, 20.0f, quadLods));
                CHECK(quadLods.empty());

                ct::Vector<SurfaceLod> levels;
                CHECK(MeshStore::generateLods(field, 20.0f, levels));
                const MeshId lodMesh = store.create("lods");
                CHECK(store.addSurface(lodMesh, field));
                const MeshInfo* lodInfo = store.get(lodMesh);
                CHECK(lodInfo != nullptr && lodInfo->surfaces.size() == 1);
                if (lodInfo && lodInfo->surfaces.size() == 1 && levels.size() >= 3)
                {
                    const SurfaceInfo& stored = lodInfo->surfaces[0];
                    CHECK(stored.lods.empty());
                    CHECK(stored.indexFormat == prisma::IndexFormat::UInt16);
                    ct::Vector<std::uint8_t> packed;
                    MeshStore::packIndices(levels[0].indices, stored.indexFormat, packed);
                    CHECK(packed.size() == levels[0].indices.size() * 2);
                    const std::uint32_t firstCount =
                            static_cast<std::uint32_t>(levels[0].indices.size());
                    CHECK(!store.addSurfaceLod(lodMesh, 1, levels[0].edgeLength, packed.data(),
                            firstCount));
                    CHECK(!store.addSurfaceLod(MeshId(), 0, levels[0].edgeLength, packed.data(),
                            firstCount));
                    CHECK(!store.addSurfaceLod(lodMesh, 0, levels[0].edgeLength, nullptr,
                            firstCount));
                    CHECK(!store.addSurfaceLod(lodMesh, 0, 0.0f, packed.data(), firstCount));
                    CHECK(!store.addSurfaceLod(lodMesh, 0, levels[0].edgeLength, packed.data(),
                            firstCount - 1));
                    CHECK(!store.addSurfaceLod(lodMesh, 0, levels[0].edgeLength, packed.data(),
                            stored.indexCount));
                    CHECK(stored.lods.empty());
                    for (std::size_t l = 0; l < levels.size(); ++l)
                    {
                        MeshStore::packIndices(levels[l].indices, stored.indexFormat, packed);
                        CHECK(store.addSurfaceLod(lodMesh, 0, levels[l].edgeLength, packed.data(),
                                static_cast<std::uint32_t>(levels[l].indices.size())));
                    }
                    CHECK(stored.lods.size() == levels.size());
                    for (std::size_t l = 0; l < stored.lods.size() && l < levels.size(); ++l)
                    {
                        CHECK(stored.lods[l].indexBuffer.valid());
                        CHECK(stored.lods[l].indexCount == levels[l].indices.size());
                        CHECK(stored.lods[l].edgeLength == levels[l].edgeLength);
                    }
                    MeshStore::packIndices(levels[0].indices, stored.indexFormat, packed);
                    CHECK(!store.addSurfaceLod(lodMesh, 0, levels[0].edgeLength, packed.data(), 3));
                    CHECK(!store.addSurfaceLod(lodMesh, 0, levels.back().edgeLength * 2.0f,
                            packed.data(), firstCount));
                    CHECK(stored.lods.size() == levels.size());

                    ct::Vector<std::uint32_t> wide;
                    wide.push_back(1);
                    wide.push_back(70000);
                    wide.push_back(3);
                    MeshStore::packIndices(wide, prisma::IndexFormat::UInt32, packed);
                    CHECK(packed.size() == 12);
                    std::uint32_t middle = 0;
                    memcpy(&middle, packed.data() + 4, 4);
                    CHECK(middle == 70000);
                }
                if (lodInfo && lodInfo->surfaces.size() == 1 &&
                        lodInfo->surfaces[0].lods.size() == levels.size() && !levels.empty())
                {
                    const SurfaceInfo& stored = lodInfo->surfaces[0];
                    CHECK(store.transform(lodMesh,
                            Math::Mat4::Scale(Math::Vec3(2.0f, 3.0f, 1.0f))));
                    for (std::size_t l = 0; l < levels.size(); ++l)
                        CHECK(stored.lods[l].edgeLength == levels[l].edgeLength * 3.0f);
                    const std::uint32_t count = stored.lods[0].indexCount;
                    ct::Vector<std::uint8_t> before;
                    CHECK(readBuffer(driver, stored.lods[0].indexBuffer, count * 2, before));
                    CHECK(store.transform(lodMesh,
                            Math::Mat4::Scale(Math::Vec3(-1.0f, 1.0f, 1.0f))));
                    CHECK(stored.lods[0].edgeLength == levels[0].edgeLength * 3.0f);
                    ct::Vector<std::uint8_t> after;
                    CHECK(readBuffer(driver, stored.lods[0].indexBuffer, count * 2, after));
                    bool flipped = before.size() == after.size() && !before.empty();
                    for (std::uint32_t i = 0; flipped && i + 2 < count; i += 3)
                    {
                        flipped = memcmp(before.data() + i * 2, after.data() + i * 2, 2) == 0 &&
                                  memcmp(before.data() + (i + 1) * 2, after.data() + (i + 2) * 2,
                                          2) == 0 &&
                                  memcmp(before.data() + (i + 2) * 2, after.data() + (i + 1) * 2,
                                          2) == 0;
                    }
                    CHECK(flipped);
                    std::uint16_t first[3];
                    memcpy(first, before.data(), sizeof(first));
                    CHECK(first[1] != first[2]);
                }
                store.destroy(lodMesh);
                CHECK(!store.valid(lodMesh));
            }

            SurfaceData bare = quad;
            bare.uvs.clear();
            CHECK(!MeshStore::generateTangents(bare));
            CHECK(bare.tangents.empty());
            SurfaceData flat = quad;
            flat.normals.clear();
            CHECK(!MeshStore::generateTangents(flat));
            SurfaceData lines = quad;
            lines.topology = prisma::Topology::Lines;
            CHECK(!MeshStore::generateTangents(lines));
            SurfaceData odd = quad;
            odd.indices.push_back(0);
            CHECK(!MeshStore::generateTangents(odd));
            SurfaceData outside = quad;
            outside.indices[5] = 4;
            CHECK(!MeshStore::generateTangents(outside));
            SurfaceData empty;
            CHECK(!MeshStore::generateTangents(empty));

            SurfaceData collapsed = quad;
            for (std::size_t i = 0; i < collapsed.uvs.size(); ++i)
                collapsed.uvs[i] = Math::Vec2(0.25f, 0.25f);
            CHECK(MeshStore::generateTangents(collapsed));
            for (std::size_t i = 0; i < collapsed.tangents.size(); ++i)
            {
                const Math::Vec4& t = collapsed.tangents[i];
                const Math::Vec3& n = collapsed.normals[i];
                const float length = sqrtf(t.x * t.x + t.y * t.y + t.z * t.z);
                CHECK(length > 0.99f && length < 1.01f);
                CHECK(fabsf(t.x * n.x + t.y * n.y + t.z * n.z) < 0.01f);
                CHECK(t.w == 1.0f || t.w == -1.0f);
            }
        }

        {
            const int columns = 8;
            SurfaceData strip;
            for (int i = 0; i <= columns; ++i)
            {
                const float x = static_cast<float>(i) * 10.0f;
                strip.positions.push_back(Math::Vec3(x, 0.0f, 0.0f));
                strip.positions.push_back(Math::Vec3(x, 0.0f, 10.0f));
                strip.normals.push_back(Math::Vec3(0.0f, 1.0f, 0.0f));
                strip.normals.push_back(Math::Vec3(0.0f, 1.0f, 0.0f));
                strip.uvs.push_back(Math::Vec2(x, 0.0f));
                strip.uvs.push_back(Math::Vec2(x, 1.0f));
            }
            for (int i = 0; i < columns; ++i)
            {
                const std::uint32_t a = static_cast<std::uint32_t>(i * 2);
                const std::uint32_t corners[6] = { a, a + 1, a + 2, a + 2, a + 1, a + 3 };
                for (int c = 0; c < 6; ++c) strip.indices.push_back(corners[c]);
            }

            ct::Vector<SurfaceData> pieces;
            CHECK(MeshStore::splitByGrid(strip, 20.0f, 32, pieces));
            CHECK(pieces.size() == 4);
            std::size_t triangles = 0;
            float covered = 0.0f;
            for (std::size_t p = 0; p < pieces.size(); ++p)
            {
                const SurfaceData& piece = pieces[p];
                CHECK(piece.positions.size() == 6 && piece.normals.size() == 6 &&
                        piece.uvs.size() == 6 && piece.tangents.empty());
                CHECK(piece.indices.size() == 12);
                triangles += piece.indices.size() / 3;
                const Math::Box box = MeshStore::computeAabb(piece.positions);
                CHECK(box.Size().x > 19.9f && box.Size().x < 20.1f);
                covered += box.Size().x;
                for (std::size_t i = 0; i < piece.indices.size(); ++i)
                {
                    CHECK(piece.indices[i] < piece.positions.size());
                    const std::uint32_t v = piece.indices[i];
                    CHECK(piece.uvs[v].x == piece.positions[v].x);
                }
            }
            CHECK(triangles == static_cast<std::size_t>(columns) * 2);
            CHECK(covered > 79.9f && covered < 80.1f);

            CHECK(MeshStore::splitByGrid(strip, 20.0f, 2, pieces));
            CHECK(pieces.size() == 2);
            CHECK(!MeshStore::splitByGrid(strip, 100.0f, 32, pieces));
            CHECK(pieces.empty());
            CHECK(!MeshStore::splitByGrid(strip, 0.0f, 32, pieces));

            SurfaceData soup;
            for (std::size_t i = 0; i < strip.indices.size(); ++i)
                soup.positions.push_back(strip.positions[strip.indices[i]]);
            CHECK(MeshStore::splitByGrid(soup, 20.0f, 32, pieces));
            CHECK(pieces.size() == 4);

            SurfaceData broken = strip;
            broken.indices[3] = 999;
            CHECK(!MeshStore::splitByGrid(broken, 20.0f, 32, pieces));
            SurfaceData skinned = strip;
            skinned.joints.resize(strip.positions.size(), 0u);
            skinned.weights.resize(strip.positions.size(), Math::Vec4(1.0f, 0.0f, 0.0f, 0.0f));
            CHECK(!MeshStore::splitByGrid(skinned, 20.0f, 32, pieces));
            SurfaceData lines = strip;
            lines.topology = prisma::Topology::Lines;
            CHECK(!MeshStore::splitByGrid(lines, 20.0f, 32, pieces));
        }

        {
            const MeshId solid = store.create("collision");
            CHECK(store.collision(solid) == nullptr);
            CHECK(store.collision(MeshId()) == nullptr);
            CHECK(store.get(solid)->collision == nullptr);
            CHECK(!store.addCollisionSurface(MeshId(), boxSurface()));
            SurfaceData broken = triangle();
            broken.positions.pop_back();
            CHECK(!store.addCollisionSurface(solid, broken));
            CHECK(store.collision(solid) == nullptr);

            SurfaceData lines = boxSurface();
            lines.topology = prisma::Topology::Lines;
            CHECK(!store.addCollisionSurface(solid, lines));
            CHECK(store.collision(solid) == nullptr);

            CHECK(store.addCollisionSurface(solid, boxSurface()));
            const CollisionMesh* collision = store.collision(solid);
            CHECK(collision != nullptr);
            CHECK(store.get(solid)->collision == collision);
            if (collision)
            {
                CHECK(collision->valid());
                CHECK(collision->triangleCount() == 12);
                CHECK(sameVec(collision->bounds().min, -1.0f, -2.0f, -3.0f));
                CHECK(sameVec(collision->bounds().max, 1.0f, 2.0f, 3.0f));
            }

            CHECK(store.addCollisionSurface(solid, boxSurface()));
            collision = store.collision(solid);
            CHECK(collision && collision->triangleCount() == 24);

            store.clearSurfaces(solid);
            CHECK(store.valid(solid));
            CHECK(store.collision(solid) == nullptr);

            CHECK(store.addCollisionSurface(solid, boxSurface()));
            CHECK(store.setCollision(solid, nullptr));
            CHECK(store.collision(solid) == nullptr);

            CollisionMesh* owned = new CollisionMesh();
            CHECK(owned->buildBox(Math::Vec3(-1.0f), Math::Vec3(1.0f)));
            CHECK(store.setCollision(solid, owned));
            CHECK(store.collision(solid) == owned);
            CollisionMesh* replacement = new CollisionMesh();
            CHECK(replacement->buildBox(Math::Vec3(-2.0f), Math::Vec3(2.0f)));
            CHECK(store.setCollision(solid, replacement));
            CHECK(store.collision(solid) == replacement);
            CHECK(store.setCollision(solid, replacement));
            CHECK(store.collision(solid) == replacement);
            CHECK(!store.setCollision(MeshId(), new CollisionMesh()));

            store.destroy(solid);
            CHECK(!store.valid(solid));
            CHECK(store.collision(solid) == nullptr);
            CHECK(!store.setCollision(solid, nullptr));

            const MeshId kept = store.create("kept");
            CHECK(store.addCollisionSurface(kept, boxSurface()));
            CHECK(store.collision(kept) != nullptr);
        }

        {
            SurfaceData source = fullTriangle();
            const MeshId moved = store.create("moved");
            CHECK(store.addSurface(moved, source, "moved"));
            CHECK(store.addCollisionSurface(moved, source));
            const std::uint32_t versionBefore = store.get(moved)->version;
            const Math::Mat4 matrix = Math::Mat4::Translation(Math::Vec3(1.0f, 2.0f, 3.0f)) *
                                      Math::Mat4::RotationY(1.5707964f) *
                                      Math::Mat4::Scale(Math::Vec3(2.0f, 2.0f, 2.0f));
            CHECK(store.transform(moved, matrix));
            const MeshInfo* movedInfo = store.get(moved);
            CHECK(movedInfo && movedInfo->version == versionBefore + 1);
            if (movedInfo && movedInfo->surfaces.size() == 1)
            {
                const SurfaceInfo& surface = movedInfo->surfaces[0];
                ct::Vector<std::uint8_t> bytes;
                CHECK(readBuffer(driver, surface.vertexBuffer, 3 * surface.vertexStride, bytes));
                if (bytes.size() == 3 * surface.vertexStride)
                {
                    for (std::uint32_t v = 0; v < 3; ++v)
                    {
                        const Math::Vec3 original = source.positions[v];
                        const Math::Vec3 expected = matrix.TransformPoint(original);
                        float position[3];
                        memcpy(position, bytes.data() + v * surface.vertexStride, sizeof(position));
                        CHECK(near(position[0], expected.x) && near(position[1], expected.y) &&
                                near(position[2], expected.z));

                        float normal[3];
                        memcpy(normal, bytes.data() + v * surface.vertexStride + 12,
                                sizeof(normal));
                        CHECK(near(normal[0], 1.0f) && near(normal[1], 0.0f) &&
                                near(normal[2], 0.0f));

                        const std::uint32_t tangentOffset =
                                MeshStore::attributeOffset(surface.attributeMask, kVertexTangent);
                        float tangent[4];
                        memcpy(tangent, bytes.data() + v * surface.vertexStride + tangentOffset,
                                sizeof(tangent));
                        CHECK(near(tangent[0], 0.0f) && near(tangent[1], 0.0f) &&
                                near(tangent[2], -1.0f) && near(tangent[3], 1.0f));
                    }
                }
                const Math::Vec3 first = matrix.TransformPoint(source.positions[0]);
                const Math::Vec3 second = matrix.TransformPoint(source.positions[1]);
                const Math::Vec3 third = matrix.TransformPoint(source.positions[2]);
                CHECK(near(surface.aabb.min.x, fminf(first.x, fminf(second.x, third.x))));
                CHECK(near(surface.aabb.max.y, fmaxf(first.y, fmaxf(second.y, third.y))));
                CHECK(near(movedInfo->aabb.min.z, fminf(first.z, fminf(second.z, third.z))));
            }
            const CollisionMesh* collision = store.collision(moved);
            CHECK(collision != nullptr && collision->valid());
            if (collision)
            {
                const Math::Vec3 a = matrix.TransformPoint(source.positions[0]);
                const Math::Vec3 b = matrix.TransformPoint(source.positions[1]);
                const Math::Vec3 c = matrix.TransformPoint(source.positions[2]);
                CHECK(near(collision->bounds().max.y, fmaxf(a.y, fmaxf(b.y, c.y))));
                CHECK(near(collision->bounds().min.z, fminf(a.z, fminf(b.z, c.z))));
            }

            const MeshId mirror = store.create("mirror");
            CHECK(store.addSurface(mirror, source, "mirror"));
            CHECK(store.transform(mirror, Math::Mat4::Scale(Math::Vec3(-1.0f, 1.0f, 1.0f))));
            const MeshInfo* mirrorInfo = store.get(mirror);
            if (mirrorInfo && mirrorInfo->surfaces.size() == 1)
            {
                ct::Vector<std::uint8_t> indices;
                CHECK(readBuffer(driver, mirrorInfo->surfaces[0].indexBuffer, 6, indices));
                if (indices.size() == 6)
                {
                    std::uint16_t order[3];
                    memcpy(order, indices.data(), sizeof(order));
                    CHECK(order[0] == 0 && order[1] == 2 && order[2] == 1);
                }
                ct::Vector<std::uint8_t> bytes;
                CHECK(readBuffer(driver, mirrorInfo->surfaces[0].vertexBuffer,
                        mirrorInfo->surfaces[0].vertexStride, bytes));
                const std::uint32_t tangentOffset = MeshStore::attributeOffset(
                        mirrorInfo->surfaces[0].attributeMask, kVertexTangent);
                if (bytes.size() >= tangentOffset + 16)
                {
                    float tangent[4];
                    memcpy(tangent, bytes.data() + tangentOffset, sizeof(tangent));
                    CHECK(near(tangent[0], -1.0f) && near(tangent[3], -1.0f));
                }
            }

            CHECK(!store.transform(mirror, Math::Mat4::Scale(Math::Vec3(0.0f, 1.0f, 1.0f))));
            CHECK(!store.transform(MeshId(), matrix));

            SurfaceData rigged = triangle();
            rigged.joints.push_back(1u);
            rigged.joints.push_back(1u);
            rigged.joints.push_back(1u);
            for (int i = 0; i < 3; ++i)
                rigged.weights.push_back(Math::Vec4(1.0f, 0.0f, 0.0f, 0.0f));
            const MeshId skin = store.create("skin transform");
            CHECK(store.addSurface(skin, rigged, "skin transform"));
            CHECK(!store.transform(skin, matrix));

            SurfaceLayout layout;
            layout.vertexCount = 3;
            const MeshId loading = store.create("loading");
            CHECK(store.beginSurface(loading, layout) >= 0);
            CHECK(!store.transform(loading, matrix));
        }

        {
            SurfaceLayout room;
            room.attributeMask = kVertexPosition | kVertexNormal | kVertexColor | kVertexUv;
            room.vertexCount = 4;
            room.indexCount = 6;
            const MeshId strip = store.create("strip");
            CHECK(store.addDynamicSurface(MeshId(), room) == -1);
            SurfaceLayout none = room;
            none.vertexCount = 0;
            CHECK(store.addDynamicSurface(strip, none) == -1);
            SurfaceLayout rigged = room;
            rigged.attributeMask |= kVertexJoints | kVertexWeights;
            CHECK(store.addDynamicSurface(strip, rigged) == -1);

            CHECK(store.addDynamicSurface(strip, room, "strip") == 0);
            const MeshInfo* stripInfo = store.get(strip);
            CHECK(stripInfo != nullptr && stripInfo->surfaces.size() == 1);
            CHECK(store.complete(strip));
            const SurfaceInfo& dynamic = stripInfo->surfaces[0];
            CHECK(dynamic.dynamic && dynamic.ready);
            CHECK(dynamic.vertexCount == 0 && dynamic.indexCount == 0);
            CHECK(dynamic.vertexCapacity == 4 && dynamic.indexCapacity == 6);
            CHECK(dynamic.indexFormat == prisma::IndexFormat::UInt16);
            CHECK(dynamic.vertexStride == 48);

            SurfaceData quad;
            quad.positions.push_back(Math::Vec3(0.0f, 0.0f, 0.0f));
            quad.positions.push_back(Math::Vec3(2.0f, 0.0f, 0.0f));
            quad.positions.push_back(Math::Vec3(2.0f, 1.0f, 0.0f));
            quad.positions.push_back(Math::Vec3(0.0f, 1.0f, 0.0f));
            for (int i = 0; i < 4; ++i) quad.uvs.push_back(Math::Vec2(0.25f * i, 1.0f));
            const std::uint32_t quadIndices[6] = { 0, 1, 2, 0, 2, 3 };
            for (std::uint32_t index: quadIndices) quad.indices.push_back(index);

            const std::uint32_t version = stripInfo->version;
            CHECK(store.updateSurface(strip, 0, quad));
            CHECK(stripInfo->version == version + 1);
            CHECK(dynamic.vertexCount == 4 && dynamic.indexCount == 6);
            CHECK(near(dynamic.aabb.max.x, 2.0f) && near(dynamic.aabb.max.y, 1.0f));
            CHECK(near(stripInfo->aabb.max.x, 2.0f) && near(stripInfo->aabb.min.x, 0.0f));
            ct::Vector<std::uint8_t> bytes;
            CHECK(readBuffer(driver, dynamic.vertexBuffer, 4 * 48, bytes));
            float second[3];
            memcpy(second, bytes.data() + 48, sizeof(second));
            CHECK(near(second[0], 2.0f) && near(second[1], 0.0f) && near(second[2], 0.0f));
            ct::Vector<std::uint8_t> indexBytes;
            CHECK(readBuffer(driver, dynamic.indexBuffer, 12, indexBytes));
            std::uint16_t readIndices[6];
            memcpy(readIndices, indexBytes.data(), sizeof(readIndices));
            CHECK(readIndices[1] == 1 && readIndices[4] == 2 && readIndices[5] == 3);

            SurfaceData smaller;
            smaller.positions.push_back(Math::Vec3(5.0f, 0.0f, 0.0f));
            smaller.positions.push_back(Math::Vec3(6.0f, 0.0f, 0.0f));
            smaller.positions.push_back(Math::Vec3(6.0f, 3.0f, 0.0f));
            for (int i = 0; i < 3; ++i) smaller.uvs.push_back(Math::Vec2(0.0f, 0.0f));
            for (std::uint32_t i = 0; i < 3; ++i) smaller.indices.push_back(i);
            CHECK(store.updateSurface(strip, 0, smaller));
            CHECK(dynamic.vertexCount == 3 && dynamic.indexCount == 3);
            CHECK(near(stripInfo->aabb.min.x, 5.0f) && near(stripInfo->aabb.max.y, 3.0f));
            CHECK(readBuffer(driver, dynamic.vertexBuffer, 48, bytes));
            memcpy(second, bytes.data(), sizeof(second));
            CHECK(near(second[0], 5.0f));

            SurfaceData tooBig = quad;
            tooBig.positions.push_back(Math::Vec3(9.0f, 9.0f, 9.0f));
            tooBig.uvs.push_back(Math::Vec2(0.0f, 0.0f));
            CHECK(!store.updateSurface(strip, 0, tooBig));
            SurfaceData otherLayout = quad;
            otherLayout.uvs.clear();
            CHECK(!store.updateSurface(strip, 0, otherLayout));
            CHECK(dynamic.vertexCount == 3);
            CHECK(!store.updateSurface(strip, 1, quad));
            CHECK(!store.updateSurface(mesh, 0, quad));

            CHECK(store.updateSurface(strip, 0, SurfaceData()));
            CHECK(dynamic.vertexCount == 0 && dynamic.indexCount == 0);
            CHECK(store.updateSurface(strip, 0, quad));
            CHECK(dynamic.vertexCount == 4);
            store.destroy(strip);
        }

        const std::size_t before = store.count();
        store.destroy(second);
        CHECK(!store.valid(second));
        CHECK(store.get(second) == nullptr);
        CHECK(store.count() == before - 1);
        store.destroy(second);
        CHECK(store.count() == before - 1);
        CHECK(store.valid(mesh));

        store.clear();
        CHECK(store.count() == 0);
        CHECK(!store.valid(mesh));
        CHECK(store.addSurface(mesh, triangle()) == false);
    }

    CHECK(MeshStore::singleton() == nullptr);

    Device::destroy(device);

    if (failures) printf("%d check(s) failed\n", failures);
    else
        printf("all checks passed\n");
    return failures ? 1 : 0;
}
