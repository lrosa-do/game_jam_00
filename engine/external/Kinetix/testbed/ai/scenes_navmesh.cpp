#include "scene.h"

#include "render_ai.h"

#include <NavMesh.h>
#include <NavMeshQuery.h>

#include <kinetix/core/Algorithm.h>

#include <GL/gl.h>

#include <math.h>
#include <stdio.h>

namespace testbed
{

using namespace kinetix::AI;

using kinetix::f32;
using kinetix::s32;
using kinetix::u32;
namespace M = kinetix::Math;

class MeshBuilder
{
public:
    void quad(f32 x0, f32 z0, f32 x1, f32 z1, f32 y)
    {
        const s32 base = s32(mVerts.size() / 3);
        push(x0, y, z0);
        push(x0, y, z1);
        push(x1, y, z1);
        push(x1, y, z0);
        mIndices.push_back(base);
        mIndices.push_back(base + 1);
        mIndices.push_back(base + 2);
        mIndices.push_back(base);
        mIndices.push_back(base + 2);
        mIndices.push_back(base + 3);
    }

    void ramp(f32 x0, f32 z0, f32 x1, f32 z1, f32 y0, f32 y1)
    {
        const s32 base = s32(mVerts.size() / 3);
        push(x0, y0, z0);
        push(x0, y1, z1);
        push(x1, y1, z1);
        push(x1, y0, z0);
        mIndices.push_back(base);
        mIndices.push_back(base + 1);
        mIndices.push_back(base + 2);
        mIndices.push_back(base);
        mIndices.push_back(base + 2);
        mIndices.push_back(base + 3);
    }

    void floorWithHole(f32 x0, f32 z0, f32 x1, f32 z1, f32 y, f32 hx0, f32 hz0, f32 hx1, f32 hz1,
                       f32 cell)
    {
        for (f32 x = x0; x < x1 - 1e-3f; x += cell)
            for (f32 z = z0; z < z1 - 1e-3f; z += cell)
            {
                const f32 cx = x + cell * 0.5f;
                const f32 cz = z + cell * 0.5f;
                if (cx > hx0 && cx < hx1 && cz > hz0 && cz < hz1)
                    continue;
                quad(x, z, x + cell, z + cell, y);
            }
    }

    bool build(NavMesh& mesh, const NavMeshConfig& config, const M::vec3* seed = nullptr) const
    {
        return mesh.build(mVerts.data(), s32(mVerts.size() / 3), mIndices.data(),
                          s32(mIndices.size() / 3), config, seed);
    }

private:
    void push(f32 x, f32 y, f32 z)
    {
        mVerts.push_back(x);
        mVerts.push_back(y);
        mVerts.push_back(z);
    }

    ct::Vector<f32> mVerts;
    ct::Vector<s32> mIndices;
};

class NavScene : public Scene
{
public:
    bool is3D() const override { return true; }
    Camera3D camera3D() const override { return mCamera; }

    void handleInput(const Input& input) override
    {
        if (!input.grabPressed || fabsf(input.rayDirection.y) < 1e-4f)
            return;
        const f32 t = -input.rayOrigin.y / input.rayDirection.y;
        if (t <= 0.0f)
            return;
        mGoal = input.rayOrigin + input.rayDirection * t;
        repath();
    }

protected:
    NavMesh mMesh;
    Camera3D mCamera;
    float mElapsed = 0.0f;
    M::vec3 mStart{0.0f};
    M::vec3 mGoal{0.0f};
    ct::Vector<M::vec3> mPath;
    bool mPathFound = false;

    M::vec3 mWalker{0.0f};
    M::vec3 mWalkerHeading{0.0f, 0.0f, 1.0f};
    M::vec3 mDrawHeading{0.0f, 0.0f, 1.0f};
    usize mNextCorner = 1;
    bool mArrived = false;
    f32 mTimeSinceArrived = 0.0f;
    f32 mWalkSpeed = 6.0f;

    void repath()
    {
        mPath.clear();
        mPathFound = mMesh.findPath(mStart, mGoal, mPath, M::vec3(4.0f, 8.0f, 4.0f));
        mWalker = mStart;
        mNextCorner = 1;
        mArrived = false;
        mTimeSinceArrived = 0.0f;
        if (mPath.size() > 1)
            mDrawHeading = M::normalize(mPath[1] - mPath[0]);
    }

    void advanceWalker(float dt)
    {
        mElapsed += dt;
        if (!mPathFound || mNextCorner >= mPath.size())
        {
            mArrived = mPathFound;
            // Time since arrival, not mElapsed (which never resets): gating on mElapsed made repath() fire every frame.
            mTimeSinceArrived += dt;
            if (mArrived && mTimeSinceArrived > 1.0f)
                repath();
            return;
        }

        f32 budget = mWalkSpeed * dt;
        while (budget > 0.0f && mNextCorner < mPath.size())
        {
            M::vec3 toCorner = mPath[mNextCorner] - mWalker;
            const f32 distance = M::length(toCorner);
            if (distance < 1e-4f)
            {
                ++mNextCorner;
                continue;
            }
            // The exact heading moves the walker; smoothing it would drift off the path. mWalkerHeading is only drawn.
            mWalkerHeading = toCorner / distance;
            if (distance <= budget)
            {
                mWalker = mPath[mNextCorner];
                budget -= distance;
                ++mNextCorner;
            }
            else
            {
                mWalker = mWalker + mWalkerHeading * budget;
                budget = 0.0f;
            }
        }

        // Blended: a fast walker can cross several short corners in one call.
        if (M::length(mWalkerHeading) > 1e-6f)
        {
            const float blend = 1.0f - powf(0.001f, dt);
            mDrawHeading =
                M::normalize(mDrawHeading + (M::normalize(mWalkerHeading) - mDrawHeading) * blend);
        }

        // Loop. The first frame the walker reaches the end lands here, so mTimeSinceArrived starts at zero here.
        if (mNextCorner >= mPath.size())
            mArrived = true;
    }

    void drawCommon() const
    {
        setupLightingAI();
        drawNavMeshSolid(mMesh.debugTriangles());
        drawPath(mPath, 0.35f, 0.85f, 1.0f);

        AgentStyle style;
        style.r = 0.35f; style.g = 0.55f; style.b = 0.65f;
        style.radius = 0.6f;
        M::vec3 heading(0.0f, 0.0f, 1.0f);
        if (mPath.size() > 1)
            heading = mPath[1] - mPath[0];
        drawAgent(mStart, heading, style);

        style.r = 0.4f; style.g = 1.0f; style.b = 0.5f;
        style.radius = 0.8f;
        drawAgent(mWalker, mDrawHeading, style);
        drawVelocity(mWalker, mWalkerHeading * mWalkSpeed, 0.25f);

        drawGoalMarker(mGoal, mElapsed);
    }
};

// The straight line between the two ends goes through a wall.
class TwoRooms final : public NavScene
{
public:
    TwoRooms()
    {
        MeshBuilder builder;
        builder.floorWithHole(0.0f, 0.0f, 20.0f, 20.0f, 0.0f, -1.0f, -1.0f, -1.0f, -1.0f, 2.0f);
        builder.floorWithHole(0.0f, 28.0f, 20.0f, 48.0f, 0.0f, -1.0f, -1.0f, -1.0f, -1.0f, 2.0f);
        for (f32 z = 20.0f; z < 28.0f; z += 2.0f)
            builder.quad(18.0f, z, 20.0f, z + 2.0f, 0.0f);

        // The corridor is 2m wide (x 18..20), so agentRadius 0.4m keeps the route clear of both walls.
        NavMeshConfig config;
        config.agentRadius = 0.4f;
        mBuilt = builder.build(mMesh, config);

        mStart = M::vec3(3.0f, 0.0f, 3.0f);
        mGoal = M::vec3(3.0f, 0.0f, 45.0f);
        repath();

        mCamera.target = M::vec3(10.0f, 0.0f, 24.0f);
        mCamera.distance = 62.0f;
        mCamera.pitch = 1.05f;

        addCheck("the mesh builds");
        addCheck("a route exists between the rooms");
        addCheck("it goes through the corridor, not the wall");
    }

    const char* name() const override { return "AI: two rooms"; }
    const char* description() const override
    {
        return "Two rooms joined by one corridor at the right-hand edge. The straight\n"
               "line between start and goal crosses the gap between them, so any route\n"
               "shorter than the detour is a route through a wall.\n\n"
               "Left click moves the goal and re-paths.";
    }

    void step(float dt) override { advanceWalker(dt); }
    void draw() override { drawCommon(); }

    void evaluate() override
    {
        report(0, mBuilt);
        report(1, mPathFound && mPath.size() >= 2);

        f32 length = 0.0f;
        bool insideGap = false;
        for (usize i = 0; i < mPath.size(); ++i)
        {
            if (i)
                length += M::distance(mPath[i - 1], mPath[i]);
            if (mPath[i].z > 20.5f && mPath[i].z < 27.5f && mPath[i].x < 17.5f)
                insideGap = true;
        }
        const f32 straight = M::distance(mStart, mGoal);

        char buf[96];
        snprintf(buf, sizeof(buf), "%.1f m vs %.1f straight", double(length), double(straight));
        report(2, mPathFound && !insideGap && length > straight * 1.15f, buf);
    }

private:
    bool mBuilt = false;
};

// The hole's edges must not be welded shut by the adjacency pass.
class Courtyard final : public NavScene
{
public:
    Courtyard()
    {
        MeshBuilder builder;
        builder.floorWithHole(0.0f, 0.0f, 30.0f, 30.0f, 0.0f, 10.0f, 10.0f, 20.0f, 20.0f, 2.0f);

        NavMeshConfig config;
        config.agentRadius = 0.5f;
        mBuilt = builder.build(mMesh, config);

        mStart = M::vec3(3.0f, 0.0f, 15.0f);
        mGoal = M::vec3(27.0f, 0.0f, 15.0f);
        repath();

        mCamera.target = M::vec3(15.0f, 0.0f, 15.0f);
        mCamera.distance = 48.0f;
        mCamera.pitch = 1.1f;

        addCheck("the mesh builds");
        addCheck("a route exists around the hole");
        addCheck("no corner sits inside the hole");
    }

    const char* name() const override { return "AI: hole in the floor"; }
    const char* description() const override
    {
        return "A courtyard with a square hole through it. Start and goal are on opposite\n"
               "sides, so the route has to commit to one way round. A corner inside the\n"
               "hole means the adjacency pass welded across a gap it should not have.\n\n"
               "Left click moves the goal and re-paths.";
    }

    void step(float dt) override { advanceWalker(dt); }
    void draw() override { drawCommon(); }

    void evaluate() override
    {
        report(0, mBuilt);
        report(1, mPathFound && mPath.size() >= 2);

        int inside = 0;
        for (usize i = 0; i < mPath.size(); ++i)
            if (mPath[i].x > 10.5f && mPath[i].x < 19.5f && mPath[i].z > 10.5f &&
                mPath[i].z < 19.5f)
                ++inside;

        char buf[64];
        snprintf(buf, sizeof(buf), "%d corners in the hole", inside);
        report(2, inside == 0, buf);
    }

private:
    bool mBuilt = false;
};

// A* has to expand into the dead end and come back out; a stuck search returns a partial path.
class DeadEnd final : public NavScene
{
public:
    DeadEnd()
    {
        MeshBuilder builder;
        for (f32 z = 0.0f; z < 40.0f; z += 2.0f)
            builder.quad(0.0f, z, 6.0f, z + 2.0f, 0.0f);
        for (f32 x = 6.0f; x < 26.0f; x += 2.0f)
            builder.quad(x, 8.0f, x + 2.0f, 14.0f, 0.0f);

        NavMeshConfig config;
        mBuilt = builder.build(mMesh, config);

        mStart = M::vec3(3.0f, 0.0f, 2.0f);
        mGoal = M::vec3(3.0f, 0.0f, 38.0f);
        repath();

        NavMeshQuery query;
        if (query.start(mMesh, mStart, mGoal, M::vec3(4.0f, 8.0f, 4.0f)))
        {
            while (query.step(1) == PathQueryState::Searching)
            {
            }
            query.result(mIncremental);
            mNodes = query.nodesExpanded();
        }

        mCamera.target = M::vec3(10.0f, 0.0f, 20.0f);
        mCamera.distance = 58.0f;
        mCamera.pitch = 1.1f;

        addCheck("the mesh builds");
        addCheck("the route reaches the goal");
        addCheck("it does not wander down the dead end");
        addCheck("one node per step gives the same route");
    }

    const char* name() const override { return "AI: dead end"; }
    const char* description() const override
    {
        return "A corridor with a branch that leads nowhere. A* has to expand into the\n"
               "branch and back out - a search that settles for the dead end returns a\n"
               "path that stops short of the goal.\n\n"
               "Left click moves the goal and re-paths.";
    }

    void step(float dt) override { advanceWalker(dt); }
    void draw() override { drawCommon(); }

    void evaluate() override
    {
        report(0, mBuilt);

        const bool reached =
            mPathFound && mPath.size() >= 2 && M::distance(mPath[mPath.size() - 1], mGoal) < 2.0f;
        char buf[80];
        snprintf(buf, sizeof(buf), "%d corners, ends %.1f m from goal", int(mPath.size()),
                 mPath.empty() ? -1.0 : double(M::distance(mPath[mPath.size() - 1], mGoal)));
        report(1, reached, buf);

        int inBranch = 0;
        for (usize i = 0; i < mPath.size(); ++i)
            if (mPath[i].x > 7.0f)
                ++inBranch;
        report(2, inBranch == 0);

        bool same = mIncremental.size() == mPath.size();
        for (usize i = 0; same && i < mPath.size(); ++i)
            if (M::distance(mIncremental[i], mPath[i]) > 1e-3f)
                same = false;
        snprintf(buf, sizeof(buf), "%u nodes expanded", mNodes);
        report(3, same, buf);
    }

private:
    bool mBuilt = false;
    ct::Vector<M::vec3> mIncremental;
    u32 mNodes = 0;
};

// The adjacency pass welds the two floors into one surface; there is no off-mesh link.
class Steps final : public NavScene
{
public:
    Steps()
    {
        MeshBuilder builder;
        for (int level = 0; level < 5; ++level)
        {
            const f32 z0 = float(level) * 8.0f;
            const f32 y = float(level) * 0.5f;
            for (f32 x = 0.0f; x < 12.0f; x += 2.0f)
                for (f32 z = z0; z < z0 + 8.0f; z += 2.0f)
                    builder.quad(x, z, x + 2.0f, z + 2.0f, y);
        }

        NavMeshConfig config;
        config.agentMaxClimb = 0.9f;
        mBuilt = builder.build(mMesh, config);

        mStart = M::vec3(6.0f, 0.0f, 2.0f);
        mGoal = M::vec3(6.0f, 2.0f, 38.0f);
        repath();

        mCamera.target = M::vec3(6.0f, 1.0f, 20.0f);
        mCamera.distance = 52.0f;
        mCamera.pitch = 0.75f;

        addCheck("the mesh builds");
        addCheck("the route climbs all five levels");
        addCheck("it follows the surface, not a straight line");
    }

    const char* name() const override { return "AI: stepped levels"; }
    const char* description() const override
    {
        return "Five platforms, each half a metre above the last, with no ramp between\n"
               "them. They are one walkable surface only because the adjacency pass welds\n"
               "open edges within agentMaxClimb - raise the steps past it and the route\n"
               "disappears.\n\nLeft click moves the goal and re-paths.";
    }

    void step(float dt) override { advanceWalker(dt); }
    void draw() override { drawCommon(); }

    void evaluate() override
    {
        report(0, mBuilt);

        const bool reached =
            mPathFound && !mPath.empty() && mPath[mPath.size() - 1].z > 34.0f;
        char buf[80];
        snprintf(buf, sizeof(buf), "ends at z=%.1f y=%.2f", mPath.empty() ? -1.0 : double(mPath[mPath.size() - 1].z),
                 mPath.empty() ? -1.0 : double(mPath[mPath.size() - 1].y));
        report(1, reached, buf);

        f32 highest = -1e9f;
        for (usize i = 0; i < mPath.size(); ++i)
            if (mPath[i].y > highest)
                highest = mPath[i].y;
        report(2, mPathFound && highest > 1.5f);
    }

private:
    bool mBuilt = false;
};

// Same courtyard but with agentRadius set: the route must stay agentRadius clear of the hole,
// measured against its known rectangle rather than NavMesh::distanceToWall().
class AgentClearance final : public NavScene
{
public:
    AgentClearance()
    {
        MeshBuilder builder;
        builder.floorWithHole(0.0f, 0.0f, 30.0f, 30.0f, 0.0f, 10.0f, 10.0f, 20.0f, 20.0f, 2.0f);

        NavMeshConfig config;
        config.agentRadius = kAgentRadius;
        mBuilt = builder.build(mMesh, config);

        mStart = M::vec3(3.0f, 0.0f, 15.0f);
        mGoal = M::vec3(27.0f, 0.0f, 15.0f);
        repath();

        mCamera.target = M::vec3(15.0f, 0.0f, 15.0f);
        mCamera.distance = 48.0f;
        mCamera.pitch = 1.1f;

        addCheck("the mesh builds");
        addCheck("a route exists around the hole");
        addCheck("every corner stays agentRadius clear of the hole");
        addCheck("every corner stays agentRadius clear of the outer wall");
    }

    const char* name() const override { return "AI: agent clearance"; }
    const char* description() const override
    {
        return "The same square hole as 'hole in the floor', but built with agentRadius\n"
               "set: every portal is cut back to the stretch at least that far from\n"
               "every wall - the hole's edge and the outer boundary alike - so the\n"
               "route clears both instead of hugging them.\n\n"
               "Left click moves the goal and re-paths.";
    }

    void step(float dt) override { advanceWalker(dt); }
    void draw() override { drawCommon(); }

    void evaluate() override
    {
        report(0, mBuilt);
        report(1, mPathFound && mPath.size() >= 2);

        f32 worstHole = 1.0e30f;
        f32 worstOuter = 1.0e30f;
        for (usize i = 0; i < mPath.size(); ++i)
        {
            const M::vec3& p = mPath[i];
            // Distance to the nearest edge of the hole rectangle [10,20]x[10,20]; <= 0 means inside.
            const f32 dxHole = kinetix::Max(10.0f - p.x, p.x - 20.0f);
            const f32 dzHole = kinetix::Max(10.0f - p.z, p.z - 20.0f);
            const f32 toHole = (dxHole < 0.0f && dzHole < 0.0f)
                                   ? kinetix::Max(dxHole, dzHole) // inside: least-negative axis
                                   : sqrtf(kinetix::Max(dxHole, 0.0f) * kinetix::Max(dxHole, 0.0f) +
                                           kinetix::Max(dzHole, 0.0f) * kinetix::Max(dzHole, 0.0f));
            if (toHole < worstHole)
                worstHole = toHole;

            const f32 toOuter =
                kinetix::Min(kinetix::Min(p.x, 30.0f - p.x), kinetix::Min(p.z, 30.0f - p.z));
            if (toOuter < worstOuter)
                worstOuter = toOuter;
        }

        // Slack: clearance is enforced on the portals, and a segment inside one triangle is not
        // re-checked (see NavMesh::computePortalSpans()).
        const f32 tolerance = 0.35f;
        char buf[64];
        snprintf(buf, sizeof(buf), "closest corner %.2f m (radius %.2f m)", double(worstHole),
                 double(kAgentRadius));
        report(2, mPathFound && worstHole >= kAgentRadius - tolerance, buf);

        snprintf(buf, sizeof(buf), "closest corner %.2f m (radius %.2f m)", double(worstOuter),
                 double(kAgentRadius));
        report(3, mPathFound && worstOuter >= kAgentRadius - tolerance, buf);
    }

private:
    static constexpr f32 kAgentRadius = 1.5f;
    bool mBuilt = false;
};

REGISTER_SCENE(TwoRooms, "AI: two rooms", Category::AI)
REGISTER_SCENE(Courtyard, "AI: hole in the floor", Category::AI)
REGISTER_SCENE(DeadEnd, "AI: dead end", Category::AI)
REGISTER_SCENE(Steps, "AI: stepped levels", Category::AI)
REGISTER_SCENE(AgentClearance, "AI: agent clearance", Category::AI)

} // namespace testbed
