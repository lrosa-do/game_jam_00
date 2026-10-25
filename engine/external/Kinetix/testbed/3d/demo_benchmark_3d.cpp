#include "scene.h"

#include "render3d.h"

#include <collision/CollisionShape.h>
#include <dynamics/PhysicsWorld.h>
#include <dynamics/RigidBody.h>

#include <kinetix/core/DebugDraw3D.h>

#include <math.h>
#include <stdio.h>
#include <time.h>

namespace testbed
{

using namespace kinetix::Physics;

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

// Contact-heavy pile (a box stack) timed with the same clock as Benchmark2D and BenchmarkAI.
class Benchmark3D final : public Scene
{
public:
    Benchmark3D()
    {
        for (int i = 0; i < kPhaseCount; ++i)
        {
            char label[48];
            const int n = kSizePerPhase[i];
            snprintf(label, sizeof(label), "%d bodies", n * n + 1);
            addCheck(label);
        }
        rebuild();
    }

    ~Benchmark3D() override { clearBodies(); }

    const char* name() const override { return "3D: benchmark"; }
    const char* description() const override
    {
        return "Growing grids of stacked boxes, ms per step() at each size. Compare\n"
               "against 2D: benchmark and AI: benchmark for a same-machine number.";
    }

    bool is3D() const override { return true; }
    Camera3D camera3D() const override { return mCamera; }

    void step(float dt) override
    {
        const double t0 = monotonicSeconds();
        mWorld.step(dt);
        const double stepMs = (monotonicSeconds() - t0) * 1000.0;

        mAccumMs += stepMs;
        ++mAccumSteps;
        mElapsedInPhase += dt;

        if (mElapsedInPhase > kPhaseSeconds)
        {
            const double avgMs = mAccumMs / double(mAccumSteps > 0 ? mAccumSteps : 1);
            char buf[64];
            snprintf(buf, sizeof(buf), "%.3f ms/step avg (%d contacts)", avgMs,
                     int(mWorld.contactCount()));
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
        setupLighting3D();
        drawGround(0.0f, mGroundExtent, 2.0f);
        drawWorldSolid(mWorld, mBodies, mShapes);
    }

    void evaluate() override
    {
    }

private:
    static constexpr int kPhaseCount = 5;
    static constexpr float kPhaseSeconds = 3.0f;
    // kSizePerPhase^2 boxes, two layers tall, plus the ground: n^2 grows the body count faster than
    // a row count and stresses the broadphase.
    static constexpr int kSizePerPhase[kPhaseCount] = {3, 5, 7, 9, 11};

    void clearBodies()
    {
        for (RigidBody* body : mBodies)
            delete body;
        for (CollisionShape* shape : mShapes)
            delete shape;
        mBodies.clear();
        mShapes.clear();
    }

    RigidBody* addBox(const M::vec3& position, const M::vec3& halfExtents, bool isStatic,
                      f32 mass = 1.0f)
    {
        RigidBody* body = new RigidBody();
        body->setPosition(position);
        body->setBodyType(isStatic ? BodyType::Static : BodyType::Dynamic);
        if (!isStatic)
        {
            body->setMass(mass);
            body->setInertiaTensor(Inertia::box(mass, halfExtents));
        }
        BoxShape* shape = new BoxShape(halfExtents);

        BodyEntry entry;
        entry.body = body;
        entry.shape = shape;
        entry.friction = 0.6f;
        mWorld.addBody(entry);

        mBodies.push_back(body);
        mShapes.push_back(shape);
        return body;
    }

    void rebuild()
    {
        mWorld.clear();
        clearBodies();
        mWorld.setGravity(M::vec3(0.0f, -9.81f, 0.0f));

        const int n = kSizePerPhase[mPhase < kPhaseCount ? mPhase : kPhaseCount - 1];
        const f32 spacing = 1.1f;
        const f32 half = spacing * 0.5f;
        const f32 extent = float(n) * spacing * 0.5f + 4.0f;
        addBox(M::vec3(0.0f, -1.0f, 0.0f), M::vec3(extent, 1.0f, extent), true);

        for (int layer = 0; layer < 2; ++layer)
            for (int z = 0; z < n; ++z)
                for (int x = 0; x < n; ++x)
                {
                    const f32 px = (float(x) - float(n - 1) * 0.5f) * spacing;
                    const f32 pz = (float(z) - float(n - 1) * 0.5f) * spacing;
                    const f32 py = half + float(layer) * spacing;
                    addBox(M::vec3(px, py, pz), M::vec3(half * 0.9f), false);
                }

        mGroundExtent = extent;
        mCamera.target = M::vec3(0.0f, 0.5f, 0.0f);
        mCamera.distance = 10.0f + float(n) * 1.6f;
        mCamera.pitch = 0.9f;
    }

    PhysicsWorld mWorld;
    ct::Vector<RigidBody*> mBodies;
    ct::Vector<CollisionShape*> mShapes;
    Camera3D mCamera;
    float mGroundExtent = 20.0f;

    int mPhase = 0;
    double mAccumMs = 0.0;
    int mAccumSteps = 0;
    float mElapsedInPhase = 0.0f;
};

constexpr int Benchmark3D::kSizePerPhase[Benchmark3D::kPhaseCount];

REGISTER_SCENE(Benchmark3D, "3D: benchmark", Category::Physics3D)

} // namespace testbed
