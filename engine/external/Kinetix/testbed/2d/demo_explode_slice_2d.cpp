#include "scene.h"

#include "debugdraw_gl.h"

#include <kx/body.h>
#include <kx/debugdraw.h>
#include <kx/joints/mousejoint.h>
#include <kx/world.h>

#include <math.h>
#include <stdio.h>

namespace testbed
{

constexpr float kGravityPx = -980.0f;

// Pyramid to poke at: left-click drags a body (Demo2D's mouse joint), C slices the body
// under the cursor, F/middle-click explodes bodies away from the cursor.
class ExplodeSlice2D final : public Scene
{
public:
    ExplodeSlice2D()
    {
        mWorld = new kx::World(kx::vec2(0.0f, kGravityPx));
        // Slicing the grabbed body deletes its mouse joint in the world: forget it then.
        mWorld->SetJointDestroyedCallback(&ExplodeSlice2D::onJointDestroyed, this);
        mWorld->CreateStaticBox(kx::vec2(0.0f, -20.0f), 900.0f, 20.0f);

        const float half = 18.0f;
        for (int row = 0; row < kRows; ++row)
        {
            const int count = kRows - row;
            const float rowY = half + float(row) * (half * 2.0f + 1.0f);
            const float rowX = -float(count - 1) * half;
            for (int i = 0; i < count; ++i)
            {
                kx::Body* box = mWorld->CreateBox(
                    kx::vec2(rowX + float(i) * half * 2.0f, rowY), half, half, 1.0f);
                box->SetFriction(0.6f);
            }
        }

        mCamera.centerY = 130.0f;
        mCamera.zoom = 0.85f;

        addCheck("slicing a body replaces it with two, mass conserved");
        addCheck("exploding scatters bodies away from the blast point");
    }

    const char* name() const override { return "2D demo: explode & slice"; }
    const char* description() const override
    {
        return "A pyramid to break apart. Left-click a body to drag it; C slices whatever\n"
               "body is under the cursor in two; F or middle-click blows everything\n"
               "nearby away from the cursor. kx::Slice/kx::Explode - never exercised by\n"
               "any other demo.";
    }

    bool is3D() const override { return false; }
    Camera2D camera2D() const override { return mCamera; }

    const char* const* overlayNames() const override
    {
        static const char* names[] = {"AABBs", "contacts", nullptr};
        return names;
    }

    void handleInput(const Input& input) override
    {
        const kx::vec2 cursor(input.worldPoint.x, input.worldPoint.y);

        if (input.grabPressed && !mMouseJoint)
        {
            if (kx::Body* body = mWorld->BodyAtPoint(cursor, true))
            {
                mMouseJoint = new kx::MouseJoint(body, cursor, 4000.0f * body->Mass());
                mWorld->AddJoint(mMouseJoint);
            }
        }

        if (input.cutPressed)
            sliceColumnAt(cursor.x);

        if (mMouseJoint)
        {
            if (input.grabHeld)
                mMouseJoint->SetTarget(cursor);
            else
            {
                mWorld->DestroyJoint(mMouseJoint);
                mMouseJoint = nullptr;
            }
        }

        if (input.firePressed)
        {
            const usize before = mWorld->BodyCount();
            kx::Explode(*mWorld, cursor, 140.0f, 3.5e6f, 1.0f);
            mExploded = mExploded || before == mWorld->BodyCount();
        }
    }

    void step(float dt) override
    {
        mWorld->Step(dt);
        mElapsed += dt;

        float fastest = 0.0f;
        for (kx::Body* body : mWorld->Bodies())
        {
            const float speed = body->Velocity().Length();
            if (speed > fastest)
                fastest = speed;
        }
        if (fastest > mFastestSeen)
            mFastestSeen = fastest;
    }

    void draw() override
    {
        GLDebugDraw2D draw;
        unsigned flags = kx::DebugDrawShapes | kx::DebugDrawJoints;
        flags |= (debugFlags & 1u) ? kx::DebugDrawAABBs : 0u;
        flags |= (debugFlags & 2u) ? kx::DebugDrawContacts : 0u;
        kx::Draw(*mWorld, draw, flags);
    }

    void evaluate() override
    {
        char buf[64];
        snprintf(buf, sizeof(buf), "%d bodies, %d slices made", int(mWorld->BodyCount()), mSliceCount);
        if (mSliceCount > 0)
            report(0, true, buf);

        snprintf(buf, sizeof(buf), "fastest body %.0f px/s", double(mFastestSeen));
        if (mExploded)
            report(1, mFastestSeen > 200.0f, buf);
    }

private:
    static void onJointDestroyed(kx::Joint* joint, void* context)
    {
        ExplodeSlice2D* self = static_cast<ExplodeSlice2D*>(context);
        if (joint == self->mMouseJoint)
            self->mMouseJoint = nullptr;
    }

    void sliceColumnAt(float x)
    {
        // Centre-distance test over every body's AABB, not a real column overlap.
        kx::Body* target = nullptr;
        float bestY = -1.0e30f;
        const float reach = 30.0f;
        for (kx::Body* body : mWorld->Bodies())
        {
            if (body->Type() != kx::BodyType::Dynamic)
                continue;
            const kx::vec2 c = body->Position();
            if (x > c.x - reach && x < c.x + reach && c.y > bestY)
            {
                bestY = c.y;
                target = body;
            }
        }
        if (!target)
            return;

        kx::Body* positive = nullptr;
        kx::Body* negative = nullptr;
        if (kx::Slice(*mWorld, target, kx::vec2(x, target->Position().y),
                      kx::vec2(1.0f, 0.0f), &positive, &negative, 40.0f))
            ++mSliceCount;
    }

    kx::World* mWorld = nullptr;
    kx::MouseJoint* mMouseJoint = nullptr;
    Camera2D mCamera;
    float mElapsed = 0.0f;

    static constexpr int kRows = 6;
    int mSliceCount = 0;
    bool mExploded = false;
    float mFastestSeen = 0.0f;

    ~ExplodeSlice2D() override { delete mWorld; }
};

REGISTER_SCENE(ExplodeSlice2D, "2D demo: explode & slice", Category::Physics2D)

} // namespace testbed
