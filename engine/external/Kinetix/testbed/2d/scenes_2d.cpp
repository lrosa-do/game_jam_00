#include "scene.h"

#include "debugdraw_gl.h"

#include <kx/body.h>
#include <kx/debugdraw.h>
#include <kx/world.h>

#include <math.h>
#include <stdio.h>

namespace testbed
{

// kLinearSlop is 0.5 and shapes carry a 1-unit skin (kPolygonRadius), so scenes are sized in pixels.
constexpr float kGravityPx = -980.0f;

class Base2D : public Scene
{
public:
    const char* const* overlayNames() const override
    {
        static const char* names[] = {"AABBs", "contacts", nullptr};
        return names;
    }

protected:
    unsigned overlayFlags() const
    {
        return ((debugFlags & 1u) ? kx::DebugDrawAABBs : 0u) |
               ((debugFlags & 2u) ? kx::DebugDrawContacts : 0u);
    }

public:
    bool is3D() const override { return false; }
    Camera2D camera2D() const override { return mCamera; }

protected:
    kx::World* mWorld = nullptr;
    Camera2D mCamera;
    float mElapsed = 0.0f;

    ~Base2D() override { delete mWorld; }

    void drawWorld()
    {
        GLDebugDraw2D draw;
        kx::Draw(*mWorld, draw, kx::DebugDrawShapes | kx::DebugDrawJoints | overlayFlags());
    }
};

class RestingBox final : public Base2D
{
public:
    RestingBox()
    {
        mWorld = new kx::World(kx::vec2(0.0f, kGravityPx));
        mWorld->CreateStaticBox(kx::vec2(0.0f, -20.0f), 600.0f, 20.0f); // top at y=0
        mBox = mWorld->CreateBox(kx::vec2(0.0f, 300.0f), 16.0f, 16.0f, 1.0f);
        mCamera.centerY = 150.0f;
        mCamera.zoom = 1.2f;
        addCheck("settles on the floor (y = 16 +/- 3)");
        addCheck("never crosses the floor (y > 0)");
        addCheck("falls at all (left y = 300)");
    }

    const char* name() const override { return "2D: box at rest"; }
    const char* description() const override
    {
        return "A 32px box dropped onto the floor. It should settle at y = 16 (its own\n"
               "half-height) and stay there. Watch for it sinking, jittering, or\n"
               "floating above the surface.";
    }

    void step(float dt) override
    {
        mWorld->Step(dt);
        mElapsed += dt;
    }
    void draw() override { drawWorld(); }

    void evaluate() override
    {
        const float y = mBox->Position().y;
        char buf[64];
        snprintf(buf, sizeof(buf), "y = %.2f", double(y));
        if (mElapsed > 1.5f)
            report(0, fabsf(y - 16.0f) < 3.0f, buf);
        report(1, y > 0.0f, buf);
        report(2, mElapsed < 0.2f || y < 299.0f, buf);
    }

private:
    kx::Body* mBox = nullptr;
};

class Stack final : public Base2D
{
public:
    Stack()
    {
        mWorld = new kx::World(kx::vec2(0.0f, kGravityPx));
        mWorld->CreateStaticBox(kx::vec2(0.0f, -20.0f), 600.0f, 20.0f);
        for (int i = 0; i < kCount; ++i)
            mBoxes[i] = mWorld->CreateBox(kx::vec2(0.0f, 20.0f + float(i) * 34.0f), 16.0f, 16.0f, 1.0f);
        mCamera.centerY = 150.0f;
        mCamera.zoom = 1.2f;
        addCheck("stays in order, bottom to top");
        addCheck("does not collapse (top above 100)");
        addCheck("bottom box rests near y = 16");
        addCheck("stops moving (top box drifts < 0.5px/s at 8s)");
    }

    const char* name() const override { return "2D: stack of 6"; }
    const char* description() const override
    {
        return "Six boxes stacked. They should keep their order, stay upright, and go\n"
               "to sleep. A stack that sinks into itself or trembles forever is the\n"
               "solver failing, not the scene.";
    }

    void step(float dt) override
    {
        mWorld->Step(dt);
        mElapsed += dt;
    }
    void draw() override { drawWorld(); }

    void evaluate() override
    {
        bool ordered = true;
        for (int i = 1; i < kCount; ++i)
            if (mBoxes[i]->Position().y <= mBoxes[i - 1]->Position().y)
                ordered = false;
        report(0, ordered);

        const float top = mBoxes[kCount - 1]->Position().y;
        char buf[64];
        snprintf(buf, sizeof(buf), "top y = %.1f", double(top));
        if (mElapsed > 1.0f)
            report(1, top > 100.0f, buf);

        if (mElapsed > 2.0f)
        {
            char b2[64];
            snprintf(b2, sizeof(b2), "bottom y = %.2f", double(mBoxes[0]->Position().y));
            report(2, fabsf(mBoxes[0]->Position().y - 16.0f) < 4.0f, b2);
        }

        if (mElapsed > 8.0f)
        {
            float fastest = 0.0f;
            for (int i = 0; i < kCount; ++i)
            {
                const kx::vec2 v = mBoxes[i]->Velocity();
                const float speed = sqrtf(v.x * v.x + v.y * v.y);
                if (speed > fastest)
                    fastest = speed;
            }
            char b3[64];
            snprintf(b3, sizeof(b3), "fastest = %.3f px/s", double(fastest));
            report(3, fastest < 0.5f, b3);
        }
    }

private:
    static constexpr int kCount = 6;
    kx::Body* mBoxes[kCount] = {};
};

class Bounce final : public Base2D
{
public:
    Bounce()
    {
        mWorld = new kx::World(kx::vec2(0.0f, kGravityPx));
        mWorld->CreateStaticBox(kx::vec2(0.0f, -20.0f), 600.0f, 20.0f);
        mBall = mWorld->CreateCircle(kx::vec2(0.0f, 400.0f), 20.0f, 1.0f);
        mStartY = 400.0f;
        mCamera.centerY = 200.0f;
        addCheck("never rises above where it started");
        addCheck("does not fall through the floor");
        addCheck("ends up resting on the floor");
    }

    const char* name() const override { return "2D: falling circle"; }
    const char* description() const override
    {
        return "A circle dropped onto the floor with no restitution. It must never end\n"
               "up higher than it began - energy appearing out of the solver is the\n"
               "failure this catches.";
    }

    void step(float dt) override
    {
        mWorld->Step(dt);
        mElapsed += dt;
        const float y = mBall->Position().y;
        if (y > mPeak)
            mPeak = y;
    }
    void draw() override { drawWorld(); }

    void evaluate() override
    {
        char buf[80];
        snprintf(buf, sizeof(buf), "peak %.1f, now %.1f", double(mPeak), double(mBall->Position().y));
        report(0, mPeak <= mStartY + 1.0f, buf);
        report(1, mBall->Position().y > 10.0f, buf);
        if (mElapsed > 4.0f)
            report(2, fabsf(mBall->Position().y - 20.0f) < 4.0f, buf);
    }

private:
    kx::Body* mBall = nullptr;
    float mStartY = 0.0f;
    float mPeak = -1e9f;
};

class Ramp final : public Base2D
{
public:
    Ramp()
    {
        mWorld = new kx::World(kx::vec2(0.0f, kGravityPx));
        mWorld->CreateStaticBox(kx::vec2(0.0f, -20.0f), 900.0f, 20.0f);
        mWorld->CreateEdge(kx::vec2(-300.0f, 300.0f), kx::vec2(250.0f, 20.0f));
        mBox = mWorld->CreateBox(kx::vec2(-260.0f, 300.0f), 14.0f, 14.0f, 1.0f);
        mStartX = -260.0f;
        mCamera.centerY = 150.0f;
        mCamera.zoom = 0.8f;
        addCheck("slides downhill (x increases)");
        addCheck("stays above the ramp, never through it");
    }

    const char* name() const override { return "2D: box on a ramp"; }
    const char* description() const override
    {
        return "A box on a sloped edge. Gravity should carry it down the slope and the\n"
               "edge should hold it - a box that tunnels through a thin edge shape is\n"
               "exactly what this is watching for.";
    }

    void step(float dt) override
    {
        mWorld->Step(dt);
        mElapsed += dt;
        if (mBox->Position().y < mLowest)
            mLowest = mBox->Position().y;
    }
    void draw() override { drawWorld(); }

    void evaluate() override
    {
        char buf[80];
        snprintf(buf, sizeof(buf), "x = %.1f, y = %.1f", double(mBox->Position().x),
                 double(mBox->Position().y));
        if (mElapsed > 1.0f)
            report(0, mBox->Position().x > mStartX + 5.0f, buf);
        report(1, mLowest > -40.0f, buf);
    }

private:
    kx::Body* mBox = nullptr;
    float mStartX = 0.0f;
    float mLowest = 1e9f;
};

class Tunnelling final : public Base2D
{
public:
    Tunnelling()
    {
        mWorld = new kx::World(kx::vec2(0.0f, kGravityPx));
        mWorld->CreateStaticBox(kx::vec2(0.0f, -20.0f), 600.0f, 20.0f);

        mPlain = mWorld->CreateBox(kx::vec2(-80.0f, 900.0f), 10.0f, 10.0f, 1.0f);
        mBullet = mWorld->CreateBox(kx::vec2(80.0f, 900.0f), 10.0f, 10.0f, 1.0f);
        mBullet->SetBullet(true);

        mPlain->SetVelocity(kx::vec2(0.0f, -4000.0f));
        mBullet->SetVelocity(kx::vec2(0.0f, -4000.0f));

        mCamera.centerY = 300.0f;
        mCamera.zoom = 0.5f;
        addCheck("bullet body is stopped by the floor");
        addCheck("bullet body ends up resting on it");
    }

    const char* name() const override { return "2D: tunnelling"; }
    const char* description() const override
    {
        return "Two identical boxes fired at the floor at 4000px/s. The right one has\n"
               "SetBullet(true) and is swept against the floor; the left one is not,\n"
               "and is the reason that flag exists. Only the right one is checked -\n"
               "the left is there to be watched.";
    }

    void step(float dt) override
    {
        mWorld->Step(dt);
        mElapsed += dt;
        if (mBullet->Position().y < mBulletLowest)
            mBulletLowest = mBullet->Position().y;
        if (mPlain->Position().y < mPlainLowest)
            mPlainLowest = mPlain->Position().y;
    }
    void draw() override { drawWorld(); }

    void evaluate() override
    {
        char buf[96];
        snprintf(buf, sizeof(buf), "bullet low %.1f, plain low %.1f", double(mBulletLowest),
                 double(mPlainLowest));
        report(0, mBulletLowest > -20.0f, buf);
        if (mElapsed > 2.0f)
            report(1, fabsf(mBullet->Position().y - 10.0f) < 6.0f, buf);
    }

private:
    kx::Body* mPlain = nullptr;
    kx::Body* mBullet = nullptr;
    float mBulletLowest = 1e9f;
    float mPlainLowest = 1e9f;
};

REGISTER_SCENE(RestingBox, "2D: box at rest", Category::Physics2D)
REGISTER_SCENE(Stack, "2D: stack of 6", Category::Physics2D)
REGISTER_SCENE(Bounce, "2D: falling circle", Category::Physics2D)
REGISTER_SCENE(Ramp, "2D: box on a ramp", Category::Physics2D)
REGISTER_SCENE(Tunnelling, "2D: tunnelling", Category::Physics2D)

} // namespace testbed
