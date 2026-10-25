#include "scene.h"

#include "render3d.h"

#include <collision/CollisionShape.h>
#include <dynamics/DistanceJoint.h>
#include <dynamics/FixedJoint.h>
#include <dynamics/HingeJoint.h>
#include <dynamics/MouseJoint.h>
#include <dynamics/PhysicsWorld.h>
#include <dynamics/PointJoint.h>
#include <dynamics/RigidBody.h>
#include <dynamics/SliderJoint.h>
#include <dynamics/UniversalJoint.h>
#include <dynamics/WheelJoint.h>

#include <kinetix/core/DebugDraw3D.h>

#include <GL/gl.h>

#include <math.h>
#include <stdio.h>

namespace testbed
{

using namespace kinetix::Physics;

using kinetix::f32;
namespace M = kinetix::Math;

// Every 3D joint type, one rig each, spread along X.
class Joints3D final : public Scene
{
public:
    bool is3D() const override { return true; }
    Camera3D camera3D() const override { return mCamera; }

    void handleInput(const Input& input) override
    {
        const kinetix::Ray ray{input.rayOrigin, input.rayDirection};

        if (input.grabPressed && !mMouseJoint)
        {
            WorldRayHit hit;
            QueryFilter filter;
            if (mWorld.raycast(ray, 500.0f, filter, hit))
            {
                for (RigidBody* body : mBodies)
                {
                    if (!body || body->bodyType() != BodyType::Dynamic)
                        continue;
                    if (M::distance(body->position(), hit.point) > 2.0f)
                        continue;
                    mMouseJoint = new MouseJoint(*body, hit.point);
                    mWorld.addJoint(mMouseJoint);
                    mGrabDistance = hit.distance;
                    break;
                }
            }
        }

        if (mMouseJoint)
        {
            if (input.grabHeld)
            {
                mGrabTarget = input.rayOrigin + input.rayDirection * mGrabDistance;
                mMouseJoint->setTarget(mGrabTarget);
            }
            else
            {
                mWorld.removeJoint(mMouseJoint);
                delete mMouseJoint;
                mMouseJoint = nullptr;
            }
        }
    }

    Joints3D()
    {
        mWorld.setGravity(M::vec3(0.0f, -9.81f, 0.0f));
        addBox(M::vec3(0.0f, -1.0f, 0.0f), M::vec3(40.0f, 1.0f, 8.0f), true);

        buildDistance(-15.0f);
        buildFixed(-10.0f);
        buildHinge(-5.0f);
        buildPoint(0.0f);
        buildSlider(5.0f);
        buildUniversal(10.0f);
        buildWheel(16.0f);

        mCamera.target = M::vec3(0.0f, 2.0f, 0.0f);
        mCamera.distance = 42.0f;
        mCamera.yaw = 1.3f;

        addCheck("distance joint holds its range");
        addCheck("fixed joint holds both bodies rigid");
        addCheck("hinge motor swings the arm");
        addCheck("point joint holds its anchor");
        addCheck("slider stays on its axis");
        addCheck("universal joint keeps its perpendicularity");
        addCheck("wheel suspension settles, does not detach");
    }

    const char* name() const override { return "3D demo: joint gym"; }
    const char* description() const override
    {
        return "All seven 3D joint types side by side: distance, fixed, hinge (limited\n"
               "swing with motor), point, slider, universal, and wheel (suspension +\n"
               "motors). Left-click and drag any dynamic body to feel its constraint\n"
               "push back.";
    }

    void step(float dt) override
    {
        mWorld.step(dt);
        mElapsed += dt;
    }

    void draw() override { drawWorld(); }

    void evaluate() override
    {
        char buf[96];

        const f32 distGap = M::distance(mDistanceBob->position(), mDistanceAnchorPos);
        snprintf(buf, sizeof(buf), "bob distance %.2f m", double(distGap));
        if (mElapsed > 1.0f)
            report(0, distGap > 1.0f && distGap < 6.0f, buf);

        const f32 fixedGap = M::distance(mFixedB->position(), mFixedAnchorPos);
        snprintf(buf, sizeof(buf), "fixed body offset from anchor %.2f m", double(fixedGap));
        if (mElapsed > 1.0f)
            report(1, fixedGap < 2.5f, buf);

        // The motor bounces the arm between its limits, so test "still moving", not "still fast".
        const f32 hingeSpeed = M::length(mHingeArm->angularVelocity());
        if (hingeSpeed > mHingeMaxSpeedSeen)
            mHingeMaxSpeedSeen = hingeSpeed;
        snprintf(buf, sizeof(buf), "peak angular speed %.2f rad/s", double(mHingeMaxSpeedSeen));
        if (mElapsed > 2.0f)
            report(2, mHingeMaxSpeedSeen > 0.5f, buf);

        const f32 pointGap = M::distance(mPointBody->position(), mPointAnchorPos);
        snprintf(buf, sizeof(buf), "point body distance from anchor %.2f m", double(pointGap));
        if (mElapsed > 1.0f)
            report(3, pointGap < 3.0f, buf);

        // The carriage must move only along its axis (X); lateral drift means the lock rows failed.
        const f32 lateral = fabsf(mSliderBody->position().z - mSliderAnchorPos.z) +
                            fabsf(mSliderBody->position().y - mSliderAnchorPos.y);
        snprintf(buf, sizeof(buf), "slider lateral drift %.2f m", double(lateral));
        if (mElapsed > 1.0f)
            report(4, lateral < 1.0f, buf);

        const f32 uniGap = M::distance(mUniversalB->position(), mUniversalAnchorPos);
        snprintf(buf, sizeof(buf), "universal body distance from anchor %.2f m", double(uniGap));
        if (mElapsed > 1.0f)
            report(5, uniGap < 3.0f, buf);

        const f32 wheelGap = M::distance(mWheelWheel->position(), mWheelChassis->position());
        snprintf(buf, sizeof(buf), "chassis-wheel gap %.2f m", double(wheelGap));
        if (mElapsed > 1.5f)
            report(6, wheelGap < 3.0f, buf);
    }

private:
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

    RigidBody* addSphere(const M::vec3& position, f32 radius, bool isStatic, f32 mass = 1.0f)
    {
        RigidBody* body = new RigidBody();
        body->setPosition(position);
        body->setBodyType(isStatic ? BodyType::Static : BodyType::Dynamic);
        if (!isStatic)
        {
            body->setMass(mass);
            body->setInertiaTensor(Inertia::solidSphere(mass, radius));
        }
        SphereShape* shape = new SphereShape(radius);

        BodyEntry entry;
        entry.body = body;
        entry.shape = shape;
        entry.friction = 0.4f;
        mWorld.addBody(entry);

        mBodies.push_back(body);
        mShapes.push_back(shape);
        return body;
    }

    void buildDistance(f32 x)
    {
        RigidBody* anchor = addBox(M::vec3(x, 8.0f, 0.0f), M::vec3(0.2f, 0.2f, 0.2f), true);
        mDistanceAnchorPos = anchor->position();
        mDistanceBob = addSphere(M::vec3(x, 5.0f, 0.0f), 0.4f, false);
        auto* joint = new DistanceJoint(*anchor, anchor->position(), *mDistanceBob,
                                        mDistanceBob->position());
        joint->setDistance(2.0f, 4.0f);
        mWorld.addJoint(joint);
        mJoints.push_back(joint);
    }

    void buildFixed(f32 x)
    {
        RigidBody* anchor = addBox(M::vec3(x, 8.0f, 0.0f), M::vec3(0.2f, 0.2f, 0.2f), true);
        mFixedB = addBox(M::vec3(x, 6.5f, 0.0f), M::vec3(0.3f, 1.0f, 0.3f), false);
        mFixedAnchorPos = anchor->position();
        auto* joint = new FixedJoint(*anchor, *mFixedB, M::vec3(x, 7.5f, 0.0f));
        mWorld.addJoint(joint);
        mJoints.push_back(joint);
    }

    void buildHinge(f32 x)
    {
        // Hinge axis +Z, arm hanging along -Y (as Chain3D). An unlimited motor occasionally spikes
        // into the thousands of rad/s as the arm crosses a full turn, so limits keep it swinging like a gate.
        RigidBody* anchor = addBox(M::vec3(x, 7.0f, 0.0f), M::vec3(0.2f, 0.2f, 0.2f), true);
        mHingeArm = addBox(M::vec3(x, 6.4f, 0.0f), M::vec3(0.1f, 0.5f, 0.1f), false, 0.3f);
        auto* joint = new HingeJoint(*anchor, *mHingeArm, M::vec3(x, 7.0f, 0.0f),
                                     M::vec3(0.0f, 0.0f, 1.0f));
        joint->setLimits(-1.0f, 1.0f);
        joint->setMotor(2.5f, 20.0f);
        mWorld.addJoint(joint);
        mJoints.push_back(joint);
    }

    void buildPoint(f32 x)
    {
        RigidBody* anchor = addBox(M::vec3(x, 8.0f, 0.0f), M::vec3(0.2f, 0.2f, 0.2f), true);
        mPointAnchorPos = anchor->position();
        mPointBody = addBox(M::vec3(x, 6.0f, 0.0f), M::vec3(0.4f, 0.4f, 0.4f), false);
        auto* joint = new PointJoint(*anchor, *mPointBody, M::vec3(x, 7.5f, 0.0f));
        mWorld.addJoint(joint);
        mJoints.push_back(joint);
    }

    void buildSlider(f32 x)
    {
        RigidBody* anchor = addBox(M::vec3(x, 6.0f, 0.0f), M::vec3(0.2f, 0.2f, 0.2f), true);
        mSliderAnchorPos = anchor->position();
        mSliderBody = addBox(M::vec3(x, 6.0f, 0.0f), M::vec3(0.4f, 0.4f, 0.4f), false);
        auto* joint = new SliderJoint(*anchor, *mSliderBody, anchor->position(),
                                      M::vec3(1.0f, 0.0f, 0.0f));
        joint->setLimits(-3.0f, 3.0f);
        joint->setMotor(1.5f, 200.0f);
        mWorld.addJoint(joint);
        mJoints.push_back(joint);
    }

    void buildUniversal(f32 x)
    {
        RigidBody* anchor = addBox(M::vec3(x, 8.0f, 0.0f), M::vec3(0.2f, 0.2f, 0.2f), true);
        mUniversalAnchorPos = anchor->position();
        mUniversalB = addBox(M::vec3(x, 6.5f, 0.0f), M::vec3(0.3f, 1.0f, 0.3f), false);
        auto* joint = new UniversalJoint(*anchor, *mUniversalB, M::vec3(x, 7.5f, 0.0f),
                                         M::vec3(1.0f, 0.0f, 0.0f), M::vec3(0.0f, 0.0f, 1.0f));
        mWorld.addJoint(joint);
        mJoints.push_back(joint);
    }

    void buildWheel(f32 x)
    {
        mWheelChassis = addBox(M::vec3(x, 4.0f, 0.0f), M::vec3(1.2f, 0.3f, 0.6f), false, 20.0f);
        mWheelWheel = addSphere(M::vec3(x, 3.0f, 0.0f), 0.5f, false, 5.0f);
        auto* joint = new WheelJoint(*mWheelChassis, *mWheelWheel, mWheelWheel->position(),
                                     M::vec3(0.0f, -1.0f, 0.0f), M::vec3(1.0f, 0.0f, 0.0f));
        joint->setSuspension(1.0f, 8000.0f, 400.0f);
        joint->setSpinMotor(4.0f, 40.0f);
        mWorld.addJoint(joint);
        mJoints.push_back(joint);
    }

    void drawWorld()
    {
        setupLighting3D();
        drawGround(0.0f, mGroundExtent, 2.0f);
        drawWorldSolid(mWorld, mBodies, mShapes);

        kinetix::DebugDraw3D& debug = kinetix::DebugDraw3D::getSingleton();
        debug.clear();
        mWorld.debugDraw();
        if (debugFlags & 1u)
        {
            glDisable(GL_LIGHTING);
            glBegin(GL_LINES);
            for (const kinetix::DebugLine3D& line : debug.lines())
            {
                glColor4ub(line.color.r(), line.color.g(), line.color.b(), line.color.a());
                glVertex3f(line.from.x, line.from.y, line.from.z);
                glVertex3f(line.to.x, line.to.y, line.to.z);
            }
            glEnd();
        }
    }

    const char* const* overlayNames() const override
    {
        static const char* names[] = {"wireframe", nullptr};
        return names;
    }

    PhysicsWorld mWorld;
    MouseJoint* mMouseJoint = nullptr;
    M::vec3 mGrabTarget{0.0f};
    f32 mGrabDistance = 10.0f;
    Camera3D mCamera;
    float mElapsed = 0.0f;
    float mGroundExtent = 42.0f;
    ct::Vector<RigidBody*> mBodies;
    ct::Vector<CollisionShape*> mShapes;
    ct::Vector<Joint*> mJoints;

    M::vec3 mDistanceAnchorPos{0.0f};
    RigidBody* mDistanceBob = nullptr;

    M::vec3 mFixedAnchorPos{0.0f};
    RigidBody* mFixedB = nullptr;

    RigidBody* mHingeArm = nullptr;
    f32 mHingeMaxSpeedSeen = 0.0f;

    M::vec3 mPointAnchorPos{0.0f};
    RigidBody* mPointBody = nullptr;

    M::vec3 mSliderAnchorPos{0.0f};
    RigidBody* mSliderBody = nullptr;

    M::vec3 mUniversalAnchorPos{0.0f};
    RigidBody* mUniversalB = nullptr;

    RigidBody* mWheelChassis = nullptr;
    RigidBody* mWheelWheel = nullptr;

    ~Joints3D() override
    {
        for (Joint* joint : mJoints)
            delete joint;
        for (RigidBody* body : mBodies)
            delete body;
        for (CollisionShape* shape : mShapes)
            delete shape;
    }
};

REGISTER_SCENE(Joints3D, "3D demo: joint gym", Category::Physics3D)

} // namespace testbed
