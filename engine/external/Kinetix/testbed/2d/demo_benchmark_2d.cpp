#include "scene.h"

#include "debugdraw_gl.h"

#include <kx/body.h>
#include <kx/debugdraw.h>
#include <kx/world.h>

#include <math.h>
#include <stdio.h>
#include <time.h>

namespace testbed
{

namespace
{
double monotonicSeconds()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return double(ts.tv_sec) + double(ts.tv_nsec) * 1.0e-9;
}
} // namespace

constexpr float kGravityPx = -980.0f;

// A pyramid sets the body count: contact solving dominates the 2D step.
class Benchmark2D final : public Scene
{
public:
    Benchmark2D()
    {
        for (int i = 0; i < kPhaseCount; ++i)
        {
            char label[48];
            snprintf(label, sizeof(label), "%d bodies", kRowCounts[i] * (kRowCounts[i] + 1) / 2);
            addCheck(label);
        }
        rebuild();
    }
    ~Benchmark2D() override { delete mWorld; }

    const char* name() const override { return "2D: benchmark"; }
    const char* description() const override
    {
        return "Stacked pyramids of growing size, ms per Step() at each size. Doubles the\n"
               "body count every few seconds; watch where the curve stops being flat.\n"
               "Compare against 3D: benchmark and AI: benchmark for a same-machine,\n"
               "same-day cross-library number, not an absolute one.";
    }

    bool is3D() const override { return false; }
    Camera2D camera2D() const override { return mCamera; }

    void step(float dt) override
    {
        const double t0 = monotonicSeconds();
        mWorld->Step(dt);
        const double stepMs = (monotonicSeconds() - t0) * 1000.0;

        mAccumMs += stepMs;
        ++mAccumSteps;
        mElapsedInPhase += dt;

        if (mElapsedInPhase > kPhaseSeconds)
        {
            mLastAvgMs = mAccumMs / double(mAccumSteps > 0 ? mAccumSteps : 1);
            mLastBodyCount = int(mWorld->BodyCount());
            char buf[64];
            snprintf(buf, sizeof(buf), "%.3f ms/step avg (%d contacts)", mLastAvgMs,
                     int(mWorld->ContactCount()));
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
        GLDebugDraw2D draw;
        kx::Draw(*mWorld, draw, kx::DebugDrawShapes);
    }

    void evaluate() override
    {
    }

private:
    static constexpr int kPhaseCount = 5;
    static constexpr float kPhaseSeconds = 3.0f;
    static constexpr int kRowCounts[kPhaseCount] = {4, 8, 12, 18, 24};

    void rebuild()
    {
        delete mWorld;
        mWorld = new kx::World(kx::vec2(0.0f, kGravityPx));
        mWorld->CreateStaticBox(kx::vec2(0.0f, -20.0f), 4000.0f, 20.0f);

        const int rows = kRowCounts[mPhase < kPhaseCount ? mPhase : kPhaseCount - 1];
        const float half = 15.0f;
        for (int row = 0; row < rows; ++row)
        {
            const int count = rows - row;
            const float rowY = half + float(row) * (half * 2.0f + 1.0f);
            const float rowX = -float(count - 1) * half;
            for (int i = 0; i < count; ++i)
            {
                kx::Body* box =
                    mWorld->CreateBox(kx::vec2(rowX + float(i) * half * 2.0f, rowY), half, half, 1.0f);
                box->SetFriction(0.6f);
            }
        }

        mCamera.centerY = float(rows) * half * 0.7f;
        mCamera.zoom = 1.0f / (1.0f + float(rows) * 0.05f);
    }

    kx::World* mWorld = nullptr;
    Camera2D mCamera;

    int mPhase = 0;
    double mAccumMs = 0.0;
    int mAccumSteps = 0;
    float mElapsedInPhase = 0.0f;
    double mLastAvgMs = 0.0;
    int mLastBodyCount = 0;
};

constexpr int Benchmark2D::kRowCounts[Benchmark2D::kPhaseCount];

REGISTER_SCENE(Benchmark2D, "2D: benchmark", Category::Physics2D)

} // namespace testbed
