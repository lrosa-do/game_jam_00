#include "scene.h"

#include "render3d.h"

#include <collision/CollisionShape.h>
#include <dynamics/HingeJoint.h>
#include <dynamics/MouseJoint.h>
#include <dynamics/PhysicsWorld.h>
#include <dynamics/RigidBody.h>

#include <kinetix/core/DebugDraw3D.h>

#include <GL/gl.h>

#include <math.h>
#include <stdio.h>

namespace testbed
{

using namespace kinetix::Physics;

using kinetix::f32;
using kinetix::u32;
namespace M = kinetix::Math;

class Demo3D : public Scene
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
                    if (M::distance(body->position(), hit.point) > 3.0f)
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

        if (input.firePressed)
            fireProjectile(input.rayOrigin, input.rayDirection);
    }

protected:
    PhysicsWorld mWorld;
    MouseJoint* mMouseJoint = nullptr;
    M::vec3 mGrabTarget{0.0f};
    f32 mGrabDistance = 10.0f;
    Camera3D mCamera;
    float mElapsed = 0.0f;
    ct::Vector<RigidBody*> mBodies;
    ct::Vector<CollisionShape*> mShapes;
    ct::Vector<Joint*> mJoints;

    ~Demo3D() override
    {
        for (Joint* joint : mJoints)
            delete joint;
        for (RigidBody* body : mBodies)
            delete body;
        for (CollisionShape* shape : mShapes)
            delete shape;
    }

    void fireProjectile(const M::vec3& origin, const M::vec3& direction)
    {
        RigidBody* body = new RigidBody();
        body->setPosition(origin + direction * 2.0f);
        body->setBodyType(BodyType::Dynamic);
        body->setMass(12.0f);
        body->setInertiaTensor(Inertia::solidSphere(12.0f, 0.4f));
        body->setVelocity(direction * 26.0f);
        SphereShape* shape = new SphereShape(0.4f);

        BodyEntry entry;
        entry.body = body;
        entry.shape = shape;
        entry.friction = 0.4f;
        mWorld.addBody(entry);

        mBodies.push_back(body);
        mShapes.push_back(shape);
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
            // setMass() leaves the inertia tensor at 1 on every axis; a heavy body that resists rotation
            // like a 1kg one makes the contact solver diverge (past ~40kg).
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

    float mGroundExtent = 25.0f;
};

class Chain3D final : public Demo3D
{
public:
    Chain3D()
    {
        mWorld.setGravity(M::vec3(0.0f, -9.81f, 0.0f));
        addBox(M::vec3(0.0f, -1.0f, 0.0f), M::vec3(20.0f, 1.0f, 20.0f), true);

        RigidBody* anchor = addBox(M::vec3(0.0f, 10.0f, 0.0f), M::vec3(0.3f, 0.3f, 0.3f), true);
        RigidBody* previous = anchor;
        f32 y = 10.0f;

        for (int i = 0; i < kLinks; ++i)
        {
            y -= kLinkLength;
            RigidBody* link =
                addBox(M::vec3(0.0f, y, 0.0f), M::vec3(0.15f, kLinkLength * 0.4f, 0.15f), false);
            HingeJoint* joint = new HingeJoint(*previous, *link,
                                               M::vec3(0.0f, y + kLinkLength * 0.5f, 0.0f),
                                               M::vec3(0.0f, 0.0f, 1.0f));
            mWorld.addJoint(joint);
            mJoints.push_back(joint);
            previous = link;
            mLinks[i] = link;
        }

        mLinks[kLinks - 1]->setVelocity(M::vec3(4.0f, 0.0f, 0.0f));

        mCamera.target = M::vec3(0.0f, 5.0f, 0.0f);
        mCamera.distance = 22.0f;

        addCheck("links stay joined");
        addCheck("the chain swings");
        addCheck("it hangs below the anchor");
    }

    const char* name() const override { return "3D demo: hanging chain"; }
    const char* description() const override
    {
        return "Eight links on hinge joints, hung from a static anchor and pushed\n"
               "sideways. Links drifting apart mean the joint solver is losing the\n"
               "constraint; a chain that never settles means it is adding energy.";
    }

    void step(float dt) override
    {
        mWorld.step(dt);
        mElapsed += dt;

        for (int i = 0; i < kLinks; ++i)
        {
            const f32 x = mLinks[i]->position().x;
            if (x > mMaxSwing)
                mMaxSwing = x;
        }
    }
    void draw() override { drawWorld(); }

    void evaluate() override
    {
        f32 worstGap = 0.0f;
        for (int i = 1; i < kLinks; ++i)
        {
            const f32 gap = M::distance(mLinks[i]->position(), mLinks[i - 1]->position());
            if (gap > worstGap)
                worstGap = gap;
        }
        char buf[64];
        snprintf(buf, sizeof(buf), "worst gap %.2f m", double(worstGap));
        report(0, worstGap < kLinkLength * 2.5f, buf);

        snprintf(buf, sizeof(buf), "max swing %.2f m", double(mMaxSwing));
        if (mElapsed > 1.0f)
            report(1, mMaxSwing > 0.2f, buf);

        f32 highest = -1e9f;
        for (int i = 0; i < kLinks; ++i)
            if (mLinks[i]->position().y > highest)
                highest = mLinks[i]->position().y;
        report(2, highest < 10.5f);
    }

private:
    static constexpr int kLinks = 8;
    static constexpr f32 kLinkLength = 0.8f;
    RigidBody* mLinks[kLinks] = {};
    f32 mMaxSwing = -1e9f;
};

class Demolition3D final : public Demo3D
{
public:
    Demolition3D()
    {
        mWorld.setGravity(M::vec3(0.0f, -9.81f, 0.0f));
        addBox(M::vec3(0.0f, -1.0f, 0.0f), M::vec3(30.0f, 1.0f, 30.0f), true);

        for (int row = 0; row < kRows; ++row)
            for (int col = 0; col < kCols; ++col)
            {
                const f32 x = (float(col) - float(kCols) * 0.5f) * 1.05f +
                              (row % 2 ? 0.5f : 0.0f);
                RigidBody* brick =
                    addBox(M::vec3(x, 0.25f + float(row) * 0.55f, 0.0f),
                           M::vec3(0.5f, 0.25f, 0.25f), false);
                mBricks.push_back(brick);
            }

        RigidBody* ball = new RigidBody();
        ball->setPosition(M::vec3(0.0f, 1.5f, 12.0f));
        ball->setBodyType(BodyType::Dynamic);
        ball->setMass(40.0f);
        ball->setInertiaTensor(Inertia::solidSphere(40.0f, 0.8f));
        // 10 m/s: no continuous collision here, so a faster sphere passes through the wall.
        ball->setVelocity(M::vec3(0.0f, 0.0f, -10.0f));
        SphereShape* shape = new SphereShape(0.8f);

        BodyEntry entry;
        entry.body = ball;
        entry.shape = shape;
        entry.friction = 0.4f;
        mWorld.addBody(entry);
        mBodies.push_back(ball);
        mShapes.push_back(shape);
        mBall = ball;

        mCamera.target = M::vec3(0.0f, 1.5f, 0.0f);
        mCamera.distance = 24.0f;

        addCheck("the wall stands before impact");
        addCheck("the ball knocks it down");
        addCheck("no brick falls through the floor");
    }

    const char* name() const override { return "3D demo: demolition"; }
    const char* description() const override
    {
        return "A brick wall and a heavy sphere thrown at it. Bricks should scatter and\n"
               "settle into a pile; bricks sinking into the floor or through each other\n"
               "is the contact solver giving up under a sudden load.";
    }

    void step(float dt) override
    {
        mWorld.step(dt);
        mElapsed += dt;

        if (mElapsed < 0.4f)
        {
            mStandingBefore = 0;
            for (RigidBody* brick : mBricks)
                if (brick->position().y > 0.2f)
                    ++mStandingBefore;
        }

        for (RigidBody* brick : mBricks)
            if (brick->position().y < mLowest)
                mLowest = brick->position().y;
    }
    void draw() override { drawWorld(); }

    void evaluate() override
    {
        report(0, mStandingBefore == int(mBricks.size()));

        int disturbed = 0;
        for (usize i = 0; i < mBricks.size(); ++i)
            if (fabsf(mBricks[i]->position().z) > 0.6f)
                ++disturbed;

        char buf[80];
        snprintf(buf, sizeof(buf), "%d of %d moved", disturbed, int(mBricks.size()));
        // Nothing happens before ~1.2 s (ball starts 12m out at 10 m/s); judging earlier records a
        // failure that cannot clear.
        if (mElapsed > 6.0f)
            report(1, disturbed > int(mBricks.size()) / 4, buf);

        snprintf(buf, sizeof(buf), "lowest %.2f m", double(mLowest));
        report(2, mLowest > -0.4f, buf);
    }

private:
    static constexpr int kRows = 6;
    static constexpr int kCols = 8;
    ct::Vector<RigidBody*> mBricks;
    RigidBody* mBall = nullptr;
    int mStandingBefore = 0;
    f32 mLowest = 1e9f;
};

REGISTER_SCENE(Chain3D, "3D demo: hanging chain", Category::Physics3D)
REGISTER_SCENE(Demolition3D, "3D demo: demolition", Category::Physics3D)

} // namespace testbed
