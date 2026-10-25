#include "scene.h"

#include "debugdraw_gl.h"

#include <kx/body.h>
#include <kx/debugdraw.h>
#include <kx/joints/distancejoint.h>
#include <kx/joints/mousejoint.h>
#include <kx/joints/revolutejoint.h>
#include <kx/joints/wheeljoint.h>
#include <kx/world.h>

#include <math.h>
#include <stdio.h>

namespace testbed
{

constexpr float kGravityPx = -980.0f;

class Demo2D : public Scene
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

    void handleInput(const Input& input) override
    {
        if (!mWorld)
            return;
        const kx::vec2 cursor(input.worldPoint.x, input.worldPoint.y);

        if (input.grabPressed && !mMouseJoint)
        {
            if (kx::Body* body = mWorld->BodyAtPoint(cursor, true))
            {
                mMouseJoint = new kx::MouseJoint(body, cursor, 4000.0f * body->Mass());
                mWorld->AddJoint(mMouseJoint);
            }
        }
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
            kx::Body* shot = mWorld->CreateCircle(cursor, 12.0f, 2.0f);
            shot->SetVelocity(kx::vec2(0.0f, -900.0f));
            shot->SetBullet(true);
            shot->SetRestitution(0.3f);
        }
    }

protected:
    kx::World* mWorld = nullptr;
    kx::MouseJoint* mMouseJoint = nullptr;
    Camera2D mCamera;
    float mElapsed = 0.0f;

    ~Demo2D() override { delete mWorld; }

    void drawWorld()
    {
        GLDebugDraw2D draw;
        kx::Draw(*mWorld, draw, drawFlags() | overlayFlags());
    }

    virtual unsigned drawFlags() const
    {
        return kx::DebugDrawShapes | kx::DebugDrawJoints;
    }
};

class Ragdoll2D final : public Demo2D
{
public:
    Ragdoll2D()
    {
        mWorld = new kx::World(kx::vec2(0.0f, kGravityPx));
        mWorld->CreateStaticBox(kx::vec2(0.0f, -20.0f), 900.0f, 20.0f);

        kx::Body* anchor = mWorld->CreateStaticBox(kx::vec2(0.0f, 420.0f), 8.0f, 8.0f);
        kx::Body* previous = anchor;
        float x = 0.0f;

        for (int i = 0; i < kLinks; ++i)
        {
            x += kLinkLength;
            kx::Body* link = mWorld->CreateBox(kx::vec2(x, 420.0f), kLinkLength * 0.5f, 5.0f, 1.0f);
            const kx::vec2 anchorPoint(x - kLinkLength * 0.5f, 420.0f);
            mWorld->AddJoint(new kx::RevoluteJoint(previous, link, anchorPoint));
            previous = link;
            mLinks[i] = link;
        }
        mTip = previous;

        mCamera.centerX = 150.0f;
        mCamera.centerY = 250.0f;
        mCamera.zoom = 0.9f;

        addCheck("the chain does not come apart");
        addCheck("it swings down under gravity");
        addCheck("the tip stays above the floor");
    }

    const char* name() const override { return "2D demo: rope"; }
    const char* description() const override
    {
        return "Twelve links on revolute joints, anchored at the left. Watch it fall and\n"
               "settle. Links pulling away from each other mean the joint solver is\n"
               "losing the constraint under its own weight.";
    }

    void step(float dt) override
    {
        mWorld->Step(dt);
        mElapsed += dt;
    }
    void draw() override { drawWorld(); }

    void evaluate() override
    {
        float worstGap = 0.0f;
        for (int i = 1; i < kLinks; ++i)
        {
            const kx::vec2 d = mLinks[i]->Position() - mLinks[i - 1]->Position();
            const float gap = sqrtf(d.x * d.x + d.y * d.y);
            if (gap > worstGap)
                worstGap = gap;
        }
        char buf[64];
        snprintf(buf, sizeof(buf), "worst link gap %.1f px", double(worstGap));
        report(0, worstGap < kLinkLength * 2.0f, buf);

        if (mElapsed > 1.0f)
            report(1, mTip->Position().y < 400.0f);
        report(2, mTip->Position().y > -10.0f);
    }

private:
    static constexpr int kLinks = 12;
    static constexpr float kLinkLength = 30.0f;
    kx::Body* mLinks[kLinks] = {};
    kx::Body* mTip = nullptr;
};

class Vehicle2D final : public Demo2D
{
public:
    Vehicle2D()
    {
        mWorld = new kx::World(kx::vec2(0.0f, kGravityPx));

        float x = -600.0f;
        float y = 0.0f;
        kx::vec2 previous(x, y);
        for (int i = 0; i < 200; ++i)
        {
            x += 40.0f;
            // 45-degree walls are unclimbable; the bumps are meant for the suspension.
            y = 12.0f * sinf(float(i) * 0.4f) * (i > 4 ? 1.0f : 0.0f);
            const kx::vec2 next(x, y);
            mWorld->CreateEdge(previous, next);
            previous = next;
        }

        mChassis = mWorld->CreateBox(kx::vec2(-500.0f, 120.0f), 55.0f, 18.0f, 1.0f);

        const kx::vec2 axis(0.0f, 1.0f);
        mRear = mWorld->CreateCircle(kx::vec2(-540.0f, 80.0f), 22.0f, 1.5f);
        mFront = mWorld->CreateCircle(kx::vec2(-460.0f, 80.0f), 22.0f, 1.5f);
        mRear->SetFriction(1.4f);
        mFront->SetFriction(1.4f);

        kx::WheelJoint* rearJoint =
            new kx::WheelJoint(mChassis, mRear, mRear->Position(), axis, 4.0f, 0.7f);
        kx::WheelJoint* frontJoint =
            new kx::WheelJoint(mChassis, mFront, mFront->Position(), axis, 4.0f, 0.7f);
        // Pixel-scale inertia: a 22px wheel at density 1.5 has ~5.5e5, so spinning it up in one step
        // needs a torque budget ~1e8. Speed is capped low: past ~10 rad/s the chassis flips.
        rearJoint->SetMotor(true, -7.0f, 5.0e8f);
        frontJoint->SetMotor(true, -7.0f, 5.0e8f);
        mWorld->AddJoint(rearJoint);
        mWorld->AddJoint(frontJoint);

        mStartX = mChassis->Position().x;
        mFurthest = mStartX;
        mCamera.centerY = 100.0f;
        mCamera.zoom = 0.55f;

        addCheck("the car drives forward");
        addCheck("it stays on its wheels");
        addCheck("wheels stay attached to the chassis");
    }

    const char* name() const override { return "2D demo: vehicle"; }
    const char* description() const override
    {
        return "Two wheel joints with motors and suspension, over rolling ground. The\n"
               "camera follows the chassis. A car that digs in, flips, or leaves its\n"
               "wheels behind is a friction or joint problem, not a scene problem.";
    }

    void step(float dt) override
    {
        mWorld->Step(dt);
        mElapsed += dt;
        mCamera.centerX = mChassis->Position().x;
        if (mChassis->Position().x > mFurthest)
            mFurthest = mChassis->Position().x;
        if (fabsf(mChassis->Angle()) > mWorstTilt)
            mWorstTilt = fabsf(mChassis->Angle());
    }
    void draw() override { drawWorld(); }

    void evaluate() override
    {
        char buf[80];
        snprintf(buf, sizeof(buf), "travelled %.0f px", double(mFurthest - mStartX));
        if (mElapsed > 2.0f)
            report(0, mFurthest > mStartX + 100.0f, buf);

        // Angle() accumulates rather than wrapping: this is "did it ever roll over".
        snprintf(buf, sizeof(buf), "worst tilt %.2f rad", double(mWorstTilt));
        report(1, mWorstTilt < 1.5f, buf);

        const kx::vec2 toRear = mRear->Position() - mChassis->Position();
        const kx::vec2 toFront = mFront->Position() - mChassis->Position();
        const float rearGap = sqrtf(toRear.x * toRear.x + toRear.y * toRear.y);
        const float frontGap = sqrtf(toFront.x * toFront.x + toFront.y * toFront.y);
        snprintf(buf, sizeof(buf), "wheel offsets %.0f / %.0f", double(rearGap), double(frontGap));
        report(2, rearGap < 140.0f && frontGap < 140.0f, buf);
    }

private:
    kx::Body* mChassis = nullptr;
    kx::Body* mRear = nullptr;
    kx::Body* mFront = nullptr;
    float mStartX = 0.0f;
    float mFurthest = -1e9f;
    float mWorstTilt = 0.0f;
};

class Pyramid2D final : public Demo2D
{
public:
    Pyramid2D()
    {
        mWorld = new kx::World(kx::vec2(0.0f, kGravityPx));
        mWorld->CreateStaticBox(kx::vec2(0.0f, -20.0f), 900.0f, 20.0f);

        const float half = 15.0f;
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
                mBoxes.push_back(box);
                if (row == 0)
                    mBase.push_back(box);
            }
        }

        mCamera.centerY = 150.0f;
        mCamera.zoom = 1.0f;

        addCheck("the pyramid stands");
        addCheck("the base does not spread");
        addCheck("nothing sinks through the floor");
    }

    const char* name() const override { return "2D demo: pyramid"; }
    const char* description() const override
    {
        return "Twenty-one boxes stacked into a pyramid. Each row leans on the one below,\n"
               "so this leans on friction as much as on contact. A pyramid that slumps\n"
               "outward is friction going soft under stacked load.";
    }

    void step(float dt) override
    {
        mWorld->Step(dt);
        mElapsed += dt;

        for (kx::Body* box : mBoxes)
            if (box->Position().y < mLowest)
                mLowest = box->Position().y;

        float widest = 0.0f;
        for (kx::Body* box : mBase)
            if (fabsf(box->Position().x) > widest)
                widest = fabsf(box->Position().x);
        if (widest > mWidestBase)
            mWidestBase = widest;
    }
    void draw() override { drawWorld(); }

    void evaluate() override
    {
        char buf[80];

        float highest = -1e9f;
        for (kx::Body* box : mBoxes)
            if (box->Position().y > highest)
                highest = box->Position().y;
        snprintf(buf, sizeof(buf), "apex at %.0f px", double(highest));
        if (mElapsed > 2.0f)
            report(0, highest > 100.0f, buf);

        snprintf(buf, sizeof(buf), "base half-width %.0f px", double(mWidestBase));
        report(1, mWidestBase < 130.0f, buf);

        snprintf(buf, sizeof(buf), "lowest %.1f px", double(mLowest));
        report(2, mLowest > 0.0f, buf);
    }

private:
    static constexpr int kRows = 6;
    ct::Vector<kx::Body*> mBoxes;
    ct::Vector<kx::Body*> mBase;
    float mLowest = 1e9f;
    float mWidestBase = 0.0f;
};

REGISTER_SCENE(Ragdoll2D, "2D demo: rope", Category::Physics2D)
REGISTER_SCENE(Vehicle2D, "2D demo: vehicle", Category::Physics2D)
REGISTER_SCENE(Pyramid2D, "2D demo: pyramid", Category::Physics2D)

} // namespace testbed
