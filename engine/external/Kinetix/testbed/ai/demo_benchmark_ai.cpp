#include "scene.h"

#include "render_ai.h"

#include <Behavior.h>
#include <Entity.h>
#include <Group.h>
#include <World.h>

#include <math.h>
#include <stdio.h>
#include <time.h>

namespace testbed
{

using namespace kinetix::AI;

using kinetix::f32;
namespace M = kinetix::Math;

namespace
{
double monotonicSeconds()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return double(ts.tv_sec) + double(ts.tv_nsec) * 1.0e-9;
}
} // namespace

// Separation + alignment + cohesion on every entity walks the ProximityGrid each frame.
class BenchmarkAI final : public Scene
{
public:
    BenchmarkAI()
    {
        for (int i = 0; i < kPhaseCount; ++i)
        {
            char label[48];
            snprintf(label, sizeof(label), "%d agents", kAgentCounts[i]);
            addCheck(label);
        }
        rebuild();

        mCamera.target = M::vec3(0.0f, 0.0f, 0.0f);
        mCamera.pitch = 0.95f;
    }

    ~BenchmarkAI() override { clearAgents(); }

    const char* name() const override { return "AI: benchmark"; }
    const char* description() const override
    {
        return "A flock of growing size (separation + alignment + cohesion, all sensed\n"
               "through the ProximityGrid), ms per World::iterate() at each size. Compare\n"
               "against 2D: benchmark and 3D: benchmark for a same-machine number.";
    }

    bool is3D() const override { return true; }
    Camera3D camera3D() const override { return mCamera; }

    void step(float dt) override
    {
        const double t0 = monotonicSeconds();
        mWorld.iterate(dt);
        const double stepMs = (monotonicSeconds() - t0) * 1000.0;

        mAccumMs += stepMs;
        ++mAccumSteps;
        mElapsedInPhase += dt;

        if (mElapsedInPhase > kPhaseSeconds)
        {
            const double avgMs = mAccumMs / double(mAccumSteps > 0 ? mAccumSteps : 1);
            char buf[48];
            snprintf(buf, sizeof(buf), "%.3f ms/iterate avg", avgMs);
            report(mPhase < kPhaseCount ? mPhase : kPhaseCount - 1, true, buf);

            ++mPhase;
            mAccumMs = 0.0;
            mAccumSteps = 0;
            mElapsedInPhase = 0.0f;
            if (mPhase < kPhaseCount)
                rebuild();
        }
    }

    void draw() override
    {
        setupLightingAI();
        for (Entity* entity : mAgents)
        {
            AgentStyle style;
            style.radius = 0.35f;
            drawAgent(entity->position(), entity->forward(), style);
        }
    }

    void evaluate() override
    {
    }

private:
    static constexpr int kPhaseCount = 5;
    static constexpr float kPhaseSeconds = 3.0f;
    static constexpr int kAgentCounts[kPhaseCount] = {20, 60, 120, 250, 500};

    void clearAgents()
    {
        if (mGroup)
        {
            // Remove the group from the World first, or it keeps a dangling pointer.
            mWorld.remove(*mGroup);
            delete mGroup;
        }
        mGroup = nullptr;
        mAgents.clear();
    }

    void rebuild()
    {
        clearAgents();
        mGroup = new Group(mWorld);
        mWorld.add(*mGroup);

        const int n = kAgentCounts[mPhase < kPhaseCount ? mPhase : kPhaseCount - 1];
        // Area grows with the count so agent density stays constant across phases.
        const f32 halfExtent = sqrtf(float(n)) * 2.0f;

        for (int i = 0; i < n; ++i)
        {
            Entity::Settings settings;
            settings.maxSpeed = 4.0f;
            settings.desiredSpeed = 3.0f;
            settings.maxVelocityChange = 6.0f;
            settings.senseRange = 5.0f;
            Entity* agent = new Entity(mWorld, settings);

            const f32 angle = float(i) * 2.399963f;
            const f32 radius = sqrtf(float(i) / float(n)) * halfExtent;
            agent->setPosition(M::vec3(cosf(angle) * radius, 0.0f, sinf(angle) * radius));
            agent->setVelocity(M::vec3(sinf(angle), 0.0f, cosf(angle)));

            agent->addBehavior(mSeparation);
            agent->addBehavior(mAlignment);
            agent->addBehavior(mCohesion);

            mGroup->add(*agent);
            mAgents.push_back(agent);
        }

        mCamera.distance = 8.0f + halfExtent * 1.6f;
    }

    World mWorld;
    Group* mGroup = nullptr;
    ct::Vector<Entity*> mAgents;

    SeparationBehavior mSeparation{1.2f, 0.4f, 1.2f};
    AlignmentBehavior mAlignment{2.0f};
    CohesionBehavior mCohesion{1.5f};

    Camera3D mCamera;

    int mPhase = 0;
    double mAccumMs = 0.0;
    int mAccumSteps = 0;
    float mElapsedInPhase = 0.0f;
};

constexpr int BenchmarkAI::kAgentCounts[BenchmarkAI::kPhaseCount];

REGISTER_SCENE(BenchmarkAI, "AI: benchmark", Category::AI)

} // namespace testbed
