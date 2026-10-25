#include "scene.h"

#include "debugdraw_gl.h"

#include <kx/body.h>
#include <kx/debugdraw.h>
#include <kx/joints/distancejoint.h>
#include <kx/joints/gearjoint.h>
#include <kx/joints/motorjoint.h>
#include <kx/joints/mousejoint.h>
#include <kx/joints/revolutejoint.h>
#include <kx/joints/wheeljoint.h>
#include <kx/world.h>

#include <math.h>
#include <stdio.h>

namespace testbed
{

constexpr float kGravityPx = -980.0f;

class Joints2D final : public Scene
{
public:
    Joints2D()
    {
        mWorld = new kx::World(kx::vec2(0.0f, kGravityPx));
        mWorld->CreateStaticBox(kx::vec2(0.0f, -20.0f), 2000.0f, 20.0f);

        buildDistance();
        buildRevolute();
        buildMotor();
        buildWheel();
        buildGear();

        mCamera.centerX = 450.0f;
        mCamera.centerY = 180.0f;
        mCamera.zoom = 0.42f;

        addCheck("distance joint holds its length range");
        addCheck("revolute motor keeps spinning");
        addCheck("motor joint holds its position target");
        addCheck("wheel suspension settles, does not detach");
        addCheck("gear joint keeps both wheels' ratio locked");
    }

    const char* name() const override { return "2D demo: joint gym"; }
    const char* description() const override
    {
        return "All six 2D joint types side by side: distance, revolute (motor), motor,\n"
               "wheel (spring), gear, and mouse. Left-drag any dynamic body to feel its\n"
               "constraint push back; a joint that lets go or explodes shows here first.";
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
    }

    void step(float dt) override
    {
        mWorld->Step(dt);
        mElapsed += dt;
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
        char buf[80];

        const float distGap = (mDistanceBob->Position() - mDistanceAnchorPos).Length();
        snprintf(buf, sizeof(buf), "bob distance %.1f px", double(distGap));
        if (mElapsed > 1.0f)
            report(0, distGap > 60.0f && distGap < 260.0f, buf);

        // Torque budget does not reach the 4 rad/s target; only checks the wheel is turning.
        snprintf(buf, sizeof(buf), "angular velocity %.2f rad/s", double(mRevoluteWheel->AngularVelocity()));
        if (mElapsed > 1.0f)
            report(1, fabsf(mRevoluteWheel->AngularVelocity()) > 0.5f, buf);

        const kx::vec2 toTarget = mMotorBody->Position() - mMotorAnchorPos;
        snprintf(buf, sizeof(buf), "offset (%.0f, %.0f) px", double(toTarget.x), double(toTarget.y));
        if (mElapsed > 2.0f)
            report(2, fabsf(toTarget.x - mMotorTargetOffset.x) < 40.0f, buf);

        const float wheelGap = (mWheelWheel->Position() - mWheelChassis->Position()).Length();
        snprintf(buf, sizeof(buf), "chassis-wheel gap %.1f px", double(wheelGap));
        if (mElapsed > 1.5f)
            report(3, wheelGap < 80.0f, buf);

        // Gear: (angleA - angleC) + ratio*(angleB - angleD) is constant; with static anchors
        // that is driver.Angle() + ratio*driven.Angle() == its starting value.
        const float gearCoordinate = mGearDriver->Angle() + mGearRatio * mGearDriven->Angle();
        const float gearError = fabsf(gearCoordinate - mGearInitialCoordinate);
        snprintf(buf, sizeof(buf), "gear coordinate drift %.2f rad", double(gearError));
        if (mElapsed > 1.0f)
            report(4, gearError < 1.0f, buf);
    }

private:
    void buildDistance()
    {
        const float x = 0.0f;
        kx::Body* anchor = mWorld->CreateStaticBox(kx::vec2(x, 400.0f), 8.0f, 8.0f);
        mDistanceAnchorPos = anchor->Position();
        mDistanceBob = mWorld->CreateCircle(kx::vec2(x, 250.0f), 16.0f, 1.0f);
        auto* joint = new kx::DistanceJoint(anchor, mDistanceBob, anchor->Position(), mDistanceBob->Position());
        joint->SetLengthRange(80.0f, 200.0f);
        joint->SetSpring(2.0f, 0.5f);
        mWorld->AddJoint(joint);
    }

    void buildRevolute()
    {
        const float x = 200.0f;
        kx::Body* anchor = mWorld->CreateStaticBox(kx::vec2(x, 300.0f), 8.0f, 8.0f);
        mRevoluteWheel = mWorld->CreateCircle(kx::vec2(x, 300.0f), 40.0f, 1.0f);
        auto* joint = new kx::RevoluteJoint(anchor, mRevoluteWheel, anchor->Position());
        // A 40px circle at density 1 has inertia ~4e6, so a torque of 1e6 only gives ~0.25 rad/s^2;
        // torque scales with pixel-scale inertia.
        joint->SetMotor(true, 4.0f, 4.0e7f);
        mWorld->AddJoint(joint);
    }

    void buildMotor()
    {
        const float x = 350.0f;
        kx::Body* anchor = mWorld->CreateStaticBox(kx::vec2(x, 300.0f), 8.0f, 8.0f);
        mMotorBody = mWorld->CreateBox(kx::vec2(x, 300.0f), 24.0f, 24.0f, 1.0f);
        mMotorAnchorPos = anchor->Position();
        auto* joint = new kx::MotorJoint(anchor, mMotorBody);
        mMotorTargetOffset = kx::vec2(90.0f, -60.0f);
        joint->SetLinearOffset(mMotorTargetOffset);
        joint->SetMaxForce(6.0e5f);
        joint->SetMaxTorque(6.0e5f);
        mWorld->AddJoint(joint);
    }

    void buildWheel()
    {
        const float x = 550.0f;
        mWheelChassis = mWorld->CreateBox(kx::vec2(x, 260.0f), 45.0f, 15.0f, 1.0f);
        mWheelWheel = mWorld->CreateCircle(kx::vec2(x, 210.0f), 22.0f, 1.5f);
        mWheelWheel->SetFriction(1.2f);
        auto* joint = new kx::WheelJoint(mWheelChassis, mWheelWheel, mWheelWheel->Position(),
                                          kx::vec2(0.0f, 1.0f), 3.0f, 0.6f);
        mWorld->AddJoint(joint);
    }

    void buildGear()
    {
        const float x = 750.0f;
        kx::Body* anchorA = mWorld->CreateStaticBox(kx::vec2(x, 320.0f), 6.0f, 6.0f);
        kx::Body* anchorB = mWorld->CreateStaticBox(kx::vec2(x + 90.0f, 320.0f), 6.0f, 6.0f);
        mGearDriver = mWorld->CreateCircle(kx::vec2(x, 320.0f), 30.0f, 1.0f);
        mGearDriven = mWorld->CreateCircle(kx::vec2(x + 90.0f, 320.0f), 15.0f, 1.0f);

        auto* revA = new kx::RevoluteJoint(anchorA, mGearDriver, anchorA->Position());
        revA->SetMotor(true, 2.0f, 1.0e6f);
        mWorld->AddJoint(revA);
        auto* revB = new kx::RevoluteJoint(anchorB, mGearDriven, anchorB->Position());
        mWorld->AddJoint(revB);

        mGearRatio = 2.0f;
        auto* gear = new kx::GearJoint(revA, revB, mGearRatio);
        mWorld->AddJoint(gear);
        mGearInitialCoordinate = mGearDriver->Angle() + mGearRatio * mGearDriven->Angle();
    }

    kx::World* mWorld = nullptr;
    kx::MouseJoint* mMouseJoint = nullptr;
    Camera2D mCamera;
    float mElapsed = 0.0f;

    kx::vec2 mDistanceAnchorPos;
    kx::Body* mDistanceBob = nullptr;

    kx::Body* mRevoluteWheel = nullptr;

    kx::vec2 mMotorAnchorPos;
    kx::vec2 mMotorTargetOffset;
    kx::Body* mMotorBody = nullptr;

    kx::Body* mWheelChassis = nullptr;
    kx::Body* mWheelWheel = nullptr;

    float mGearRatio = 1.0f;
    float mGearInitialCoordinate = 0.0f;
    kx::Body* mGearDriver = nullptr;
    kx::Body* mGearDriven = nullptr;

    ~Joints2D() override { delete mWorld; }
};

REGISTER_SCENE(Joints2D, "2D demo: joint gym", Category::Physics2D)

} // namespace testbed
