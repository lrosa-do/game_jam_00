#include "check.h"

#include <NavMesh.h>
#include <NavMeshQuery.h>

#include <ct/vector.hpp>

using kinetix::AI::NavMesh;
using kinetix::AI::NavMeshConfig;
using kinetix::AI::NavMeshQuery;
using kinetix::AI::PathQueryState;

namespace
{

namespace KM = kinetix::Math;

struct Mesh
{
    ct::Vector<float> verts;
    ct::Vector<int> indices;
};

// rows[z][x] == '#' is a walkable cell of `scale` metres, vertices shared on
// a regular grid. Each cell is split along (x,z)-(x+1,z+1), or along the
// other diagonal when `otherDiagonal` is set.
Mesh makeGrid(const char* const* rows, int height, bool otherDiagonal, float scale = 1.0f)
{
    int width = 0;
    while (rows[0][width])
        ++width;
    Mesh mesh;
    for (int z = 0; z <= height; ++z)
        for (int x = 0; x <= width; ++x)
        {
            mesh.verts.push_back(static_cast<float>(x) * scale);
            mesh.verts.push_back(0.0f);
            mesh.verts.push_back(static_cast<float>(z) * scale);
        }
    const auto id = [width](int x, int z) { return z * (width + 1) + x; };
    for (int z = 0; z < height; ++z)
        for (int x = 0; x < width; ++x)
        {
            if (rows[z][x] != '#')
                continue;
            const int a = id(x, z), b = id(x, z + 1), c = id(x + 1, z + 1), d = id(x + 1, z);
            const int split[2][6] = {{a, b, c, a, c, d}, {a, b, d, b, c, d}};
            for (int i = 0; i < 6; ++i)
                mesh.indices.push_back(split[otherDiagonal ? 1 : 0][i]);
        }
    return mesh;
}

bool build(NavMesh& nav, const Mesh& mesh, const NavMeshConfig& config = NavMeshConfig())
{
    return nav.build(mesh.verts.data(), static_cast<int>(mesh.verts.size() / 3), mesh.indices.data(),
                     static_cast<int>(mesh.indices.size() / 3), config);
}

float distanceXZ(const KM::vec3& a, const KM::vec3& b)
{
    const float dx = a.x - b.x;
    const float dz = a.z - b.z;
    return sqrtf(dx * dx + dz * dz);
}

}

int main()
{
    // agentRadius must not depend on how the floor is triangulated: a 2 m
    // corridor of 1 m quads stays connected for any radius up to 1 m, and
    // every corner of the route keeps the radius from both side walls.
    {
        const char* rows[] = {"##", "##", "##", "##", "##", "##"};
        const float radii[] = {0.5f, 0.7f, 1.0f};
        for (int diagonal = 0; diagonal < 2; ++diagonal)
            for (float radius : radii)
            {
                NavMeshConfig config;
                config.agentRadius = radius;
                NavMesh nav;
                CHECK(build(nav, makeGrid(rows, 6, diagonal != 0), config));
                ct::Vector<KM::vec3> path;
                bool partial = true;
                CHECK(nav.findPath(KM::vec3(1.0f, 0.0f, 0.5f), KM::vec3(1.0f, 0.0f, 5.5f), path,
                                   KM::vec3(2.0f, 4.0f, 2.0f), &partial));
                CHECK(!partial);
                CHECK(path.size() >= 2);
                CHECK(!path.empty() && path[path.size() - 1].z > 4.9f);
                for (kinetix::usize i = 1; i < path.size(); ++i)
                {
                    CHECK(path[i].x >= radius - 1e-2f);
                    CHECK(2.0f - path[i].x >= radius - 1e-2f);
                }
            }

        NavMeshConfig config;
        config.agentRadius = 1.1f;
        NavMesh nav;
        CHECK(build(nav, makeGrid(rows, 6, false), config));
        ct::Vector<KM::vec3> path;
        bool partial = false;
        CHECK(nav.findPath(KM::vec3(1.0f, 0.0f, 0.5f), KM::vec3(1.0f, 0.0f, 5.5f), path,
                           KM::vec3(2.0f, 4.0f, 2.0f), &partial));
        CHECK(partial);
    }

    // A 10 m room made of only two triangles still keeps the route off its
    // walls: the goal next to a wall is pushed out to agentRadius, and no
    // corner hugs a wall.
    {
        const char* rows[] = {"#"};
        NavMeshConfig config;
        config.agentRadius = 1.0f;
        NavMesh nav;
        CHECK(build(nav, makeGrid(rows, 1, false, 10.0f), config));
        ct::Vector<KM::vec3> path;
        CHECK(nav.findPath(KM::vec3(5.0f, 0.0f, 5.0f), KM::vec3(9.9f, 0.0f, 0.2f), path));
        CHECK(path.size() >= 2);
        for (kinetix::usize i = 1; i < path.size(); ++i)
        {
            const KM::vec3& p = path[i];
            const float clearance = kinetix::Min(kinetix::Min(p.x, 10.0f - p.x),
                                                 kinetix::Min(p.z, 10.0f - p.z));
            CHECK(clearance >= 1.0f - 1e-2f);
        }
    }

    {
        const char* rows[] = {"##..##"};
        NavMesh nav;
        CHECK(build(nav, makeGrid(rows, 1, false)));
        ct::Vector<KM::vec3> path;
        bool partial = false;
        CHECK(nav.findPath(KM::vec3(0.5f, 0.0f, 0.5f), KM::vec3(5.5f, 0.0f, 0.5f), path,
                           KM::vec3(2.0f, 4.0f, 2.0f), &partial));
        CHECK(partial);
        CHECK(nav.findPath(KM::vec3(0.5f, 0.0f, 0.5f), KM::vec3(1.5f, 0.0f, 0.5f), path,
                           KM::vec3(2.0f, 4.0f, 2.0f), &partial));
        CHECK(!partial);

        NavMeshQuery query;
        CHECK(query.start(nav, KM::vec3(0.5f, 0.0f, 0.5f), KM::vec3(5.5f, 0.0f, 0.5f)));
        while (query.step(1) == PathQueryState::Searching)
        {
        }
        CHECK(query.state() == PathQueryState::Ready);
        CHECK(query.partial());
    }

    // Funnel: a start lying exactly on the first portal must not produce a
    // corner behind it (L corridor, both diagonal splits, both directions).
    {
        const char* rows[] = {"#####", "....#", "....#", "....#", "....#"};
        for (int diagonal = 0; diagonal < 2; ++diagonal)
        {
            NavMesh nav;
            CHECK(build(nav, makeGrid(rows, 5, diagonal != 0)));
            const KM::vec3 ends[2] = {KM::vec3(4.5f, 0.0f, 4.5f), KM::vec3(0.5f, 0.0f, 0.5f)};
            for (int direction = 0; direction < 2; ++direction)
            {
                ct::Vector<KM::vec3> path;
                CHECK(nav.findPath(ends[direction], ends[1 - direction], path));
                CHECK(path.size() == 3);
                if (path.size() == 3)
                    CHECK(distanceXZ(path[1], KM::vec3(4.0f, 0.0f, 1.0f)) < 1e-3f);
            }
        }
    }

    {
        const char* rows[] = {"###", "###", "###"};
        NavMesh nav;
        CHECK(build(nav, makeGrid(rows, 3, false)));
        KM::vec3 out;
        CHECK(nav.moveAlongSurface(KM::vec3(0.25f, 0.0f, 0.75f), KM::vec3(1.75f, 0.0f, 1.25f), out));
        CHECK(distanceXZ(out, KM::vec3(1.75f, 0.0f, 1.25f)) < 1e-3f);
        CHECK(nav.moveAlongSurface(KM::vec3(0.5f, 0.0f, 1.5f), KM::vec3(1.5f, 0.0f, 0.5f), out));
        CHECK(distanceXZ(out, KM::vec3(1.5f, 0.0f, 0.5f)) < 1e-3f);
    }

    // raycast and distanceToWall agree on which way a wall faces: into the
    // walkable area.
    {
        const char* rows[] = {"#####", "....#", "....#", "....#", "....#"};
        NavMesh nav;
        CHECK(build(nav, makeGrid(rows, 5, false)));
        float fraction = 1.0f;
        KM::vec3 hitNormal;
        const KM::vec3 from(0.5f, 0.0f, 0.5f);
        const KM::vec3 to(0.5f, 0.0f, 3.0f);
        CHECK(nav.raycast(from, to, fraction, hitNormal));
        CHECK(fraction < 1.0f);
        CHECK(hitNormal.z < -0.99f); // the wall at z = 1, facing back to z < 1

        float distance = 0.0f;
        KM::vec3 wallNormal;
        CHECK(nav.distanceToWall(KM::vec3(2.5f, 0.0f, 0.8f), 1.0f, distance, wallNormal));
        CHECK_NEAR(distance, 0.2f, 1e-3f);
        CHECK(KM::dot(wallNormal, hitNormal) > 0.99f);
    }

    {
        const char* rows[] = {"######", "######"};
        NavMesh nav;
        CHECK(build(nav, makeGrid(rows, 2, false)));
        const KM::vec3 start(0.5f, 0.0f, 0.5f);
        const KM::vec3 goal(5.5f, 0.0f, 1.5f);

        NavMeshQuery query;
        query.setMaxNodes(100000);
        CHECK(query.start(nav, start, goal));
        while (query.step(1) == PathQueryState::Searching)
        {
        }
        CHECK(query.state() == PathQueryState::Ready);
        const kinetix::u32 needed = query.nodesExpanded();
        ct::Vector<KM::vec3> path;
        CHECK(query.result(path));
        const kinetix::usize corners = path.size();
        CHECK(query.result(path));
        CHECK(path.size() == corners);

        ct::Vector<KM::vec3> direct;
        CHECK(nav.findPath(start, goal, direct));
        CHECK(direct.size() == corners);

        query.setMaxNodes(needed);
        CHECK(query.start(nav, start, goal));
        while (query.step(1) == PathQueryState::Searching)
        {
        }
        CHECK(query.state() == PathQueryState::Ready);

        query.setMaxNodes(needed - 1);
        CHECK(query.start(nav, start, goal));
        while (query.step(1) == PathQueryState::Searching)
        {
        }
        CHECK(query.state() == PathQueryState::Failed);
        CHECK(!query.result(path));
        CHECK(path.empty());
    }

    {
        const float verts[] = {0.0f, 0.0f, 0.0f,  2.0f, 0.0f, 0.0f, 1.0f, 0.0f, -2.0f,
                               1.0f, 0.0f, 2.0f,  1.0f, 0.5f, 3.0f};
        const int indices[] = {0, 1, 2, 1, 0, 3, 1, 0, 4};
        NavMesh nav;
        CHECK(nav.build(verts, 5, indices, 3, NavMeshConfig()));
        const KM::vec3 points[3] = {KM::vec3(1.0f, 0.0f, -1.0f), KM::vec3(1.0f, 0.0f, 1.0f),
                                    KM::vec3(1.0f, 0.45f, 2.8f)};
        const KM::vec3 extents(0.5f, 0.3f, 0.5f);
        for (int i = 0; i < 3; ++i)
            for (int j = i + 1; j < 3; ++j)
            {
                ct::Vector<KM::vec3> path;
                CHECK(nav.findPath(points[i], points[j], path, extents));
                const bool forward = !path.empty() && distanceXZ(path[path.size() - 1], points[j]) < 1e-2f;
                CHECK(nav.findPath(points[j], points[i], path, extents));
                const bool backward = !path.empty() && distanceXZ(path[path.size() - 1], points[i]) < 1e-2f;
                CHECK(forward == backward);
            }
    }

    return TEST_RESULT();
}
