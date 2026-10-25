#include "scene.h"

#include "render3d.h"

#include <collision/CollisionShape.h>
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
using kinetix::usize;
namespace M = kinetix::Math;

class Base3D : public Scene
{
public:
    bool is3D() const override { return true; }
    Camera3D camera3D() const override { return mCamera; }

protected:
    PhysicsWorld mWorld;
    Camera3D mCamera;
    float mElapsed = 0.0f;
    ct::Vector<RigidBody*> mBodies;
    ct::Vector<CollisionShape*> mShapes;

    ~Base3D() override
    {
        for (RigidBody* b : mBodies)
            delete b;
        for (CollisionShape* s : mShapes)
            delete s;
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
        entry.friction = 0.55f;
        mWorld.addBody(entry);

        mBodies.push_back(body);
        mShapes.push_back(shape);
        return body;
    }

    RigidBody* addSphere(const M::vec3& position, f32 radius, f32 mass = 1.0f)
    {
        RigidBody* body = new RigidBody();
        body->setPosition(position);
        body->setBodyType(BodyType::Dynamic);
        body->setMass(mass);
        body->setInertiaTensor(Inertia::solidSphere(mass, radius));
        SphereShape* shape = new SphereShape(radius);

        BodyEntry entry;
        entry.body = body;
        entry.shape = shape;
        entry.friction = 0.55f;
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

class RestingBox3D final : public Base3D
{
public:
    RestingBox3D()
    {
        mWorld.setGravity(M::vec3(0.0f, -9.81f, 0.0f));
        addBox(M::vec3(0.0f, -1.0f, 0.0f), M::vec3(20.0f, 1.0f, 20.0f), true); // top at y=0
        mBox = addBox(M::vec3(0.0f, 8.0f, 0.0f), M::vec3(0.5f, 0.5f, 0.5f), false);
        mCamera.target = M::vec3(0.0f, 2.0f, 0.0f);
        mCamera.distance = 18.0f;
        addCheck("settles at y = 0.5 (+/- 0.15)");
        addCheck("never sinks below y = 0.3");
        addCheck("actually falls");
    }

    const char* name() const override { return "3D: box at rest"; }
    const char* description() const override
    {
        return "A 1m box dropped onto a static floor. Unlike the 2D solver, this one\n"
               "works in metres, so it should settle at y = 0.5 with no visible gap.";
    }

    void step(float dt) override
    {
        mWorld.step(dt);
        mElapsed += dt;
        if (mBox->position().y < mLowest)
            mLowest = mBox->position().y;
    }
    void draw() override { drawWorld(); }

    void evaluate() override
    {
        const f32 y = mBox->position().y;
        char buf[64];
        snprintf(buf, sizeof(buf), "y = %.3f", double(y));
        if (mElapsed > 3.0f)
            report(0, fabsf(y - 0.5f) < 0.15f, buf);
        report(1, mLowest > 0.3f, buf);
        report(2, mElapsed < 0.3f || y < 7.9f, buf);
    }

private:
    RigidBody* mBox = nullptr;
    f32 mLowest = 1e9f;
};

class Stack3D final : public Base3D
{
public:
    Stack3D()
    {
        mWorld.setGravity(M::vec3(0.0f, -9.81f, 0.0f));
        addBox(M::vec3(0.0f, -1.0f, 0.0f), M::vec3(20.0f, 1.0f, 20.0f), true);
        for (int i = 0; i < kCount; ++i)
            mStack[i] = addBox(M::vec3(0.0f, 0.5f + float(i) * 1.05f, 0.0f),
                               M::vec3(0.5f, 0.5f, 0.5f), false);
        mCamera.target = M::vec3(0.0f, 2.0f, 0.0f);
        mCamera.distance = 16.0f;
        addCheck("stays in order, bottom to top");
        addCheck("does not collapse");
        addCheck("does not drift sideways");
    }

    const char* name() const override { return "3D: stack of 5"; }
    const char* description() const override
    {
        return "Five boxes stacked in 3D. Order, height and lateral drift are all\n"
               "checked - a stack that slowly walks off its base is a solver problem\n"
               "that a screenshot alone would not show.\n\n"
               "Friction here is 0.55 because that is inside the only band this solver\n"
               "keeps a stack standing in: below 0.55 and above 0.60 it collapses. More\n"
               "friction making a stack worse is not physical - see ARCHITECTURE.md.";
    }

    void step(float dt) override
    {
        mWorld.step(dt);
        mElapsed += dt;
    }
    void draw() override { drawWorld(); }

    void evaluate() override
    {
        bool ordered = true;
        for (int i = 1; i < kCount; ++i)
            if (mStack[i]->position().y <= mStack[i - 1]->position().y)
                ordered = false;
        report(0, ordered);

        const f32 top = mStack[kCount - 1]->position().y;
        char buf[64];
        snprintf(buf, sizeof(buf), "top y = %.2f", double(top));
        if (mElapsed > 1.0f)
            report(1, top > 3.0f, buf);

        f32 drift = 0.0f;
        for (int i = 0; i < kCount; ++i)
        {
            const f32 d = sqrtf(mStack[i]->position().x * mStack[i]->position().x +
                                mStack[i]->position().z * mStack[i]->position().z);
            if (d > drift)
                drift = d;
        }
        char b2[64];
        snprintf(b2, sizeof(b2), "drift = %.3f", double(drift));
        report(2, drift < 1.0f, b2);
    }

private:
    static constexpr int kCount = 5;
    RigidBody* mStack[kCount] = {};
};

class SphereDrop final : public Base3D
{
public:
    SphereDrop()
    {
        mWorld.setGravity(M::vec3(0.0f, -9.81f, 0.0f));
        addBox(M::vec3(0.0f, -1.0f, 0.0f), M::vec3(20.0f, 1.0f, 20.0f), true);
        for (int i = 0; i < kCount; ++i)
            mBalls[i] = addSphere(M::vec3(float(i % 3) * 1.4f - 1.4f, 6.0f + float(i) * 1.6f,
                                             float(i / 3) * 1.4f - 1.0f),
                                  0.5f);
        mCamera.target = M::vec3(0.0f, 1.5f, 0.0f);
        mCamera.distance = 20.0f;
        addCheck("none falls through the floor");
        addCheck("all come to rest on it");
    }

    const char* name() const override { return "3D: spheres"; }
    const char* description() const override
    {
        return "Nine spheres dropped together - they collide with each other on the way\n"
               "down. None should end up below the floor, and all should settle on it.";
    }

    void step(float dt) override
    {
        mWorld.step(dt);
        mElapsed += dt;
        for (int i = 0; i < kCount; ++i)
            if (mBalls[i]->position().y < mLowest)
                mLowest = mBalls[i]->position().y;
    }
    void draw() override { drawWorld(); }

    void evaluate() override
    {
        char buf[64];
        snprintf(buf, sizeof(buf), "lowest y = %.3f", double(mLowest));
        report(0, mLowest > 0.2f, buf);

        if (mElapsed > 6.0f)
        {
            bool settled = true;
            for (int i = 0; i < kCount; ++i)
                if (mBalls[i]->position().y > 3.0f)
                    settled = false;
            report(1, settled);
        }
    }

private:
    static constexpr int kCount = 9;
    RigidBody* mBalls[kCount] = {};
    f32 mLowest = 1e9f;
};

REGISTER_SCENE(RestingBox3D, "3D: box at rest", Category::Physics3D)
REGISTER_SCENE(Stack3D, "3D: stack of 5", Category::Physics3D)
REGISTER_SCENE(SphereDrop, "3D: spheres", Category::Physics3D)

} // namespace testbed
