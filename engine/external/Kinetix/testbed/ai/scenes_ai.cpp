#include "scene.h"

#include "render_ai.h"

#include <Group.h>
#include <NavMesh.h>
#include <NavMeshBehavior.h>
#include <NavMeshQuery.h>
#include <ProximityGrid.h>
#include <SquadEntity.h>
#include <World.h>

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

// Integer coordinates put an agent exactly on a shared vertex, which the surface walk must survive.
void buildGrid(int n, ct::Vector<f32>& verts, ct::Vector<s32>& indices)
{
    for (int z = 0; z <= n; ++z)
        for (int x = 0; x <= n; ++x)
        {
            verts.push_back(f32(x));
            verts.push_back(0.0f);
            verts.push_back(f32(z));
        }

    auto at = [n](int x, int z) { return z * (n + 1) + x; };
    for (int z = 0; z < n; ++z)
        for (int x = 0; x < n; ++x)
        {
            indices.push_back(at(x, z));
            indices.push_back(at(x, z + 1));
            indices.push_back(at(x + 1, z + 1));
            indices.push_back(at(x, z));
            indices.push_back(at(x + 1, z + 1));
            indices.push_back(at(x + 1, z));
        }
}

class BaseAI : public Scene
{
public:
    bool is3D() const override { return true; }
    Camera3D camera3D() const override { return mCamera; }

protected:
    NavMesh mMesh;
    Camera3D mCamera;
    float mElapsed = 0.0f;

    void drawMesh() const
    {
        setupLightingAI();
        drawNavMeshSolid(mMesh.debugTriangles());
    }

    static void drawMarker(const M::vec3& p, float size, float r, float g, float b)
    {
        AgentStyle style;
        style.r = r; style.g = g; style.b = b; style.radius = size;
        drawAgent(p, M::vec3(0.0f, 0.0f, 1.0f), style);
    }
};

// The incremental query must produce findPath()'s path whatever budget it is given.
class QueryEquivalence final : public BaseAI
{
public:
    QueryEquivalence()
    {
        ct::Vector<f32> verts;
        ct::Vector<s32> indices;
        buildGrid(40, verts, indices);
        NavMeshConfig config;
        mMesh.build(verts.data(), s32(verts.size() / 3), indices.data(), s32(indices.size() / 3),
                    config);

        mStart = M::vec3(1.0f, 0.0f, 1.0f);
        mEnd = M::vec3(38.5f, 0.0f, 38.5f);
        mMesh.findPath(mStart, mEnd, mReference);

        mCamera.target = M::vec3(19.0f, 0.0f, 19.0f);
        mCamera.pitch = 0.95f;
        mCamera.distance = 46.0f;

        addCheck("findPath returns a route");
        addCheck("every budget gives the same corners");
        addCheck("every budget expands the same nodes");
    }

    const char* name() const override { return "AI: incremental A*"; }
    const char* description() const override
    {
        return "The same search, run to completion and then again one budget at a time\n"
               "(1, 7, 64, 1000 nodes per step). The thin line is the reference path,\n"
               "found once; the thick line is what each budget just found - a yellow dot\n"
               "appears off to the left as each budget passes. Spreading a search over\n"
               "frames must not change what it finds - only when it finishes, so the two\n"
               "lines should never separate.";
    }

    void step(float dt) override
    {
        mElapsed += dt;
        if (mDone)
            return;

        static const u32 kBudgets[] = {1u, 7u, 64u, 1000u};
        if (mBudgetIndex >= 4)
        {
            mDone = true;
            return;
        }

        NavMeshQuery query;
        if (!query.start(mMesh, mStart, mEnd))
        {
            mBudgetIndex = 4;
            return;
        }
        while (query.step(kBudgets[mBudgetIndex]) == PathQueryState::Searching)
        {
        }

        ct::Vector<M::vec3> path;
        query.result(path);
        mLast = kinetix::Move(path);
        mLastNodes = query.nodesExpanded();
        if (mFirstNodes == 0)
            mFirstNodes = mLastNodes;

        if (!matchesReference())
            mMismatch = true;
        if (mLastNodes != mFirstNodes)
            mNodeMismatch = true;

        ++mBudgetIndex;
    }

    void draw() override
    {
        drawMesh();
        drawPath(mReference, 0.35f, 0.45f, 1.0f);
        if (!mLast.empty())
            drawPath(mLast, mMismatch ? 1.0f : 0.35f, mMismatch ? 0.3f : 1.0f, 0.35f, 5.0f);
        drawMarker(mStart, 0.8f, 0.3f, 1.0f, 0.3f);
        drawMarker(mEnd, 0.8f, 1.0f, 0.4f, 0.3f);

        for (usize i = 0; i < mBudgetIndex && i < 4; ++i)
        {
            AgentStyle style;
            style.radius = 0.4f;
            style.r = 0.9f;
            style.g = 0.75f;
            style.b = 0.2f;
            drawAgent(M::vec3(-3.0f, 0.0f, 1.0f + float(i) * 1.5f), M::vec3(1.0f, 0.0f, 0.0f),
                     style);
        }
    }

    void evaluate() override
    {
        report(0, mReference.size() >= 2);

        char buf[96];
        snprintf(buf, sizeof(buf), "%d budgets checked", int(mBudgetIndex));
        report(1, !mMismatch, buf);

        snprintf(buf, sizeof(buf), "%u nodes", mLastNodes);
        report(2, !mNodeMismatch, buf);
    }

private:
    bool matchesReference() const
    {
        if (mLast.size() != mReference.size())
            return false;
        for (usize i = 0; i < mLast.size(); ++i)
        {
            const M::vec3 d = mLast[i] - mReference[i];
            if (fabsf(d.x) > 1e-4f || fabsf(d.y) > 1e-4f || fabsf(d.z) > 1e-4f)
                return false;
        }
        return true;
    }

    static void drawPath(const ct::Vector<M::vec3>& path, float r, float g, float b,
                        float lineWidth = 1.0f)
    {
        if (path.size() < 2)
            return;
        glLineWidth(lineWidth);
        glColor3f(r, g, b);
        glBegin(GL_LINE_STRIP);
        for (usize i = 0; i < path.size(); ++i)
            glVertex3f(path[i].x, path[i].y + 0.35f, path[i].z);
        glEnd();
        glLineWidth(1.0f);
    }

    M::vec3 mStart;
    M::vec3 mEnd;
    ct::Vector<M::vec3> mReference;
    ct::Vector<M::vec3> mLast;
    usize mBudgetIndex = 0;
    u32 mFirstNodes = 0;
    u32 mLastNodes = 0;
    bool mMismatch = false;
    bool mNodeMismatch = false;
    bool mDone = false;
};

// Agents starting on integer coordinates stand on shared vertices.
class Crowd final : public BaseAI
{
public:
    Crowd()
    {
        ct::Vector<f32> verts;
        ct::Vector<s32> indices;
        buildGrid(40, verts, indices);
        NavMeshConfig config;
        mMesh.build(verts.data(), s32(verts.size() / 3), indices.data(), s32(indices.size() / 3),
                    config);

        NavMeshBehavior::Settings settings;
        settings.searchNodesPerFrame = 64;
        // goalRadius alone is too small for sixteen 0.55m agents converging on one point; arrivalRadius
        // ramps turnRate down before the brake.
        settings.goalRadius = 3.0f;
        settings.arrivalRadius = 8.0f;
        settings.avoidDistance = 1.2f;
        mBehavior = new NavMeshBehavior(mMesh, settings);

        mGroup = new Group(mWorld);
        mWorld.add(*mGroup);

        mGoal = M::vec3(38.0f, 0.0f, 38.0f);
        for (int i = 0; i < kAgents; ++i)
        {
            Entity::Settings entitySettings;
            entitySettings.maxSpeed = 6.0f;
            entitySettings.desiredSpeed = 4.0f;
            SquadEntity* agent = new SquadEntity(mWorld, entitySettings);
            agent->setPosition(M::vec3(1.0f + f32(i % 4), 0.0f, 1.0f + f32(i / 4)));
            // A per-agent offset gives each agent its own arrival spot inside the shared goal marker.
            agent->setGoal(mGoal + slotFor(i));
            agent->addBehavior(*mBehavior);
            mGroup->add(*agent);
            mAgents.push_back(agent);
        }

        mCamera.target = M::vec3(19.0f, 0.0f, 19.0f);
        mCamera.pitch = 0.95f;
        mCamera.distance = 46.0f;

        addCheck("every agent leaves its starting vertex");
        addCheck("nobody leaves the walkable surface");
        addCheck("all reach the goal");
    }

    ~Crowd() override { delete mBehavior; }

    const char* name() const override { return "AI: crowd to a goal"; }
    const char* description() const override
    {
        return "Sixteen agents placed on exact grid vertices, walking to the far corner.\n"
               "An agent that never moves means the surface walk is refusing to leave a\n"
               "vertex; one that stops the whole frame means the funnel is looping.";
    }

    void step(float dt) override
    {
        mWorld.iterate(dt);
        mElapsed += dt;

        if (mHeading.size() != mAgents.size())
            mHeading.resize(mAgents.size(), M::vec3(0.0f, 0.0f, 1.0f));

        for (usize i = 0; i < mAgents.size(); ++i)
        {
            const M::vec3 p = mAgents[i]->position();
            if (p.x < -1.0f || p.z < -1.0f || p.x > 41.0f || p.z > 41.0f)
                mLeftSurface = true;

            // No behavior aligns the entity, so draw from a blend of velocity() to avoid snapping.
            const M::vec3 v = mAgents[i]->velocity();
            if (M::length(v) > 0.05f)
            {
                const float blend = 1.0f - powf(0.001f, dt);
                mHeading[i] = M::normalize(mHeading[i] + (M::normalize(v) - mHeading[i]) * blend);
            }
        }
    }

    void draw() override
    {
        drawMesh();
        drawGoalMarker(mGoal, mElapsed);

        for (usize i = 0; i < mAgents.size(); ++i)
        {
            SquadEntity* agent = mAgents[i];
            AgentStyle style;
            style.radius = 0.55f;
            const float togo = M::distance(agent->position(), mGoal) / 52.0f;
            style.r = 0.35f + togo * 0.5f;
            style.g = 0.85f - togo * 0.35f;
            style.b = 1.0f - togo * 0.5f;
            const M::vec3 heading = i < mHeading.size() ? mHeading[i] : agent->velocity();
            drawAgent(agent->position(), heading, style);
            drawVelocity(agent->position(), agent->velocity(), 0.4f);
        }
    }

    void evaluate() override
    {
        int moved = 0;
        int arrived = 0;
        for (int i = 0; i < kAgents; ++i)
        {
            const M::vec3 p = mAgents[usize(i)]->position();
            if (fabsf(p.x - (1.0f + f32(i % 4))) > 0.3f || fabsf(p.z - (1.0f + f32(i / 4))) > 0.3f)
                ++moved;
            // Each agent's own slot. Slack over NavMeshBehavior::Settings::goalRadius (3.0m): a braked
            // agent nudged by avoidance settles at 3.0-3.4m, not strictly inside it.
            if (M::distance(p, mGoal + slotFor(i)) < 4.0f)
                ++arrived;
        }

        char buf[80];
        snprintf(buf, sizeof(buf), "%d of %d moved", moved, kAgents);
        if (mElapsed > 1.0f)
            report(0, moved == kAgents, buf);

        report(1, !mLeftSurface);

        snprintf(buf, sizeof(buf), "%d of %d arrived", arrived, kAgents);
        if (mElapsed > 18.0f)
            report(2, arrived == kAgents, buf);
    }

private:
    static constexpr int kAgents = 16;

    // Same spread as the start grid: each agent has its own arrival slot, used for the goal and in evaluate().
    static M::vec3 slotFor(int i)
    {
        return M::vec3(f32(i % 4) - 1.5f, 0.0f, f32(i / 4) - 1.5f);
    }

    World mWorld;
    Group* mGroup = nullptr;
    NavMeshBehavior* mBehavior = nullptr;
    ct::Vector<SquadEntity*> mAgents;
    ct::Vector<M::vec3> mHeading;
    M::vec3 mGoal;
    bool mLeftSurface = false;
};

// Checks the grid query against a brute-force sweep every step.
class Proximity final : public BaseAI
{
public:
    Proximity()
    {
        mGroup = new Group(mWorld);
        mWorld.add(*mGroup);

        unsigned seed = 12345u;
        auto nextRandom = [&seed]() {
            seed = seed * 1103515245u + 12345u;
            return f32((seed >> 16) & 0x7fff) / f32(0x7fff);
        };

        for (int i = 0; i < kAgents; ++i)
        {
            Entity::Settings settings;
            settings.senseRange = 4.0f;
            Entity* agent = new Entity(mWorld, settings);
            agent->setPosition(M::vec3(nextRandom() * 40.0f, 0.0f, nextRandom() * 40.0f));
            mGroup->add(*agent);
            mAgents.push_back(agent);
        }

        mCamera.target = M::vec3(19.0f, 0.0f, 19.0f);
        mCamera.pitch = 0.95f;
        mCamera.distance = 46.0f;

        addCheck("grid answers match a brute-force sweep");
        addCheck("queries stay bounded by the neighbourhood");
    }

    const char* name() const override { return "AI: proximity grid"; }
    const char* description() const override
    {
        return "A hundred agents, one asked every step (in turn) who is within sense\n"
               "range - the bright one, ringed at its actual sense radius. Green dots are\n"
               "who the grid answered; dim ones are everybody else. A brute-force sweep\n"
               "asks the same question every step, and the two answers have to agree -\n"
               "the grid is an index, not an approximation.";
    }

    void step(float dt) override
    {
        mWorld.iterate(dt);
        mElapsed += dt;

        mCurrentProbe = mAgents[mProbe % mAgents.size()];
        ++mProbe;

        mLastNeighbours.clear();
        mWorld.proximityGrid().query(mCurrentProbe->position(), mCurrentProbe->senseRange(),
                                     mLastNeighbours);

        ct::Vector<Entity*> brute;
        for (Entity* other : mAgents)
        {
            const M::vec3 d = other->position() - mCurrentProbe->position();
            if (d.x * d.x + d.z * d.z <= mCurrentProbe->senseRange() * mCurrentProbe->senseRange())
                brute.push_back(other);
        }

        mLastGrid = mLastNeighbours.size();
        mLastBrute = brute.size();
        if (mLastNeighbours.size() != brute.size() || !sameSet(mLastNeighbours, brute))
            mMismatch = true;

        if (mLastNeighbours.size() > mWorstQuery)
            mWorstQuery = mLastNeighbours.size();
    }

    void draw() override
    {
        drawGrid3D(0.0f, 60, 1.0f);

        for (Entity* agent : mAgents)
            if (agent != mCurrentProbe)
                drawMarker(agent->position(), 0.22f, 0.32f, 0.36f, 0.42f);

        for (Entity* neighbour : mLastNeighbours)
            if (neighbour != mCurrentProbe)
                drawMarker(neighbour->position(), 0.32f, 1.0f, 0.55f, 0.55f);

        if (mCurrentProbe)
        {
            drawMarker(mCurrentProbe->position(), 0.4f, 1.0f, 0.9f, 0.3f);
            drawSenseRing(mCurrentProbe->position(), mCurrentProbe->senseRange());
        }
    }

    void evaluate() override
    {
        char buf[96];
        snprintf(buf, sizeof(buf), "grid %d, brute %d", int(mLastGrid), int(mLastBrute));
        report(0, !mMismatch, buf);

        snprintf(buf, sizeof(buf), "worst %d of %d agents", int(mWorstQuery), kAgents);
        report(1, mWorstQuery < usize(kAgents), buf);
    }

private:
    static constexpr int kAgents = 100;

    static bool sameSet(const ct::Vector<Entity*>& a, const ct::Vector<Entity*>& b)
    {
        for (Entity* entity : a)
        {
            bool found = false;
            for (Entity* other : b)
                if (entity == other)
                {
                    found = true;
                    break;
                }
            if (!found)
                return false;
        }
        return true;
    }

    static void drawSenseRing(const M::vec3& centre, float radius)
    {
        glDisable(GL_LIGHTING);
        glColor3f(1.0f, 0.9f, 0.3f);
        glBegin(GL_LINE_LOOP);
        for (int i = 0; i < 32; ++i)
        {
            const float a = 2.0f * 3.14159265f * float(i) / 32.0f;
            glVertex3f(centre.x + radius * cosf(a), centre.y + 0.08f, centre.z + radius * sinf(a));
        }
        glEnd();
    }

    World mWorld;
    Group* mGroup = nullptr;
    ct::Vector<Entity*> mAgents;
    Entity* mCurrentProbe = nullptr;
    ct::Vector<Entity*> mLastNeighbours;
    usize mProbe = 0;
    usize mLastGrid = 0;
    usize mLastBrute = 0;
    usize mWorstQuery = 0;
    bool mMismatch = false;
};

REGISTER_SCENE(QueryEquivalence, "AI: incremental A*", Category::AI)
REGISTER_SCENE(Crowd, "AI: crowd to a goal", Category::AI)
REGISTER_SCENE(Proximity, "AI: proximity grid", Category::AI)

} // namespace testbed
