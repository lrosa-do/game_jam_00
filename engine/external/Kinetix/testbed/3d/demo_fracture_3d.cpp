#include "scene.h"

#include "render3d.h"

#include <collision/CollisionShape.h>
#include <dynamics/MouseJoint.h>
#include <dynamics/PhysicsWorld.h>
#include <dynamics/RigidBody.h>
#include <geometry/VoronoiShatter.h>

#include <kinetix/core/Algorithm.h>
#include <kinetix/core/DebugDraw3D.h>

#include <GL/gl.h>

#include <math.h>
#include <stdio.h>

namespace testbed
{

using namespace kinetix::Physics;
using kinetix::Geometry::Shard;
using kinetix::Geometry::VoronoiShatter;

using kinetix::f32;
using kinetix::u32;
namespace M = kinetix::Math;

// A tower of boxes and a wrecking ball: a hard enough impact replaces the box hit with a handful
// of VoronoiShatter shards, each its own dynamic ConvexHullShape body.
class Fracture3D final : public Scene
{
public:
    Fracture3D()
    {
        mWorld.setGravity(M::vec3(0.0f, -9.81f, 0.0f));
        addBox(M::vec3(0.0f, -1.0f, 0.0f), M::vec3(20.0f, 1.0f, 20.0f), true);

        for (int i = 0; i < kBlocks; ++i)
        {
            addBox(M::vec3(0.0f, 0.5f + float(i) * 1.05f, 0.0f), M::vec3(0.5f), false, 8.0f);
            mUnbroken.push_back(mLastBodyId);
        }

        RigidBody* ball = new RigidBody();
        ball->setPosition(M::vec3(0.0f, 2.5f, 12.0f));
        ball->setBodyType(BodyType::Dynamic);
        ball->setMass(60.0f);
        ball->setInertiaTensor(Inertia::solidSphere(60.0f, 0.9f));
        ball->setVelocity(M::vec3(0.0f, 0.0f, -14.0f));
        SphereShape* ballShape = new SphereShape(0.9f);
        BodyEntry ballEntry;
        ballEntry.body = ball;
        ballEntry.shape = ballShape;
        ballEntry.friction = 0.4f;
        mBallId = mWorld.addBody(ballEntry);
        mBodies.push_back(ball);
        mShapes.push_back(ballShape);

        mWorld.setEventCallback(&Fracture3D::onContact, this);

        mCamera.target = M::vec3(0.0f, 2.0f, 0.0f);
        mCamera.distance = 26.0f;
        mCamera.pitch = 0.85f;

        addCheck("the ball hits the tower");
        addCheck("at least one block shatters into shards");
        addCheck("shards fall and settle instead of exploding outward");
    }

    ~Fracture3D() override
    {
        for (Joint* joint : mJoints)
            delete joint;
        for (RigidBody* body : mBodies)
            delete body;
        for (CollisionShape* shape : mShapes)
            delete shape;
    }

    const char* name() const override { return "3D demo: fracture"; }
    const char* description() const override
    {
        return "A tower of blocks and a wrecking ball. A hard enough hit pulls the block\n"
               "out of the world and replaces it with kinetix::Geometry::VoronoiShatter\n"
               "shards - real ConvexHullShape bodies, not a visual effect. Watch for\n"
               "shards that fly apart instead of tumbling and settling.";
    }

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
                for (usize i = 0; i < mBodies.size(); ++i)
                {
                    RigidBody* body = mBodies[i];
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
                mMouseJoint->setTarget(input.rayOrigin + input.rayDirection * mGrabDistance);
            else
            {
                mWorld.removeJoint(mMouseJoint);
                delete mMouseJoint;
                mMouseJoint = nullptr;
            }
        }
    }

    void step(float dt) override
    {
        mWorld.step(dt);
        mElapsed += dt;

        for (usize i = 0; i < mShardIds.size(); ++i)
        {
            const BodyEntry* entry = mWorld.body(mShardIds[i]);
            if (!entry)
                continue;
            const f32 speed = M::length(entry->body->velocity());
            if (speed > mFastestShardSpeed)
                mFastestShardSpeed = speed;
        }

        // Fracturing mid-step (inside the event callback) would touch the solver's references; the
        // callback only records what to break and it happens here, after step() has returned.
        for (u32 id : mPendingShatter)
            shatterBlock(id);
        mPendingShatter.clear();
    }

    void draw() override
    {
        setupLighting3D();
        drawGround(0.0f, 22.0f, 2.0f);
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

    void evaluate() override
    {
        // Nothing happens before ~0.85 s (ball starts 12 m out at 14 m/s); judging earlier records a
        // failure that cannot clear.
        if (mElapsed > 1.2f)
            report(0, mHitDetected);

        char buf[48];
        snprintf(buf, sizeof(buf), "%d block(s) shattered", mShatterCount);
        if (mElapsed > 2.0f)
            report(1, mShatterCount > 0, buf);

        snprintf(buf, sizeof(buf), "fastest shard %.1f m/s", double(mFastestShardSpeed));
        // A shard leaving its block at speed means Voronoi cells with volume/centroid outside the
        // block (geometry, not physics). 25 m/s is headroom over the 14 m/s impact.
        if (mShatterCount > 0 && mElapsed > 3.0f)
            report(2, mFastestShardSpeed < 25.0f, buf);
    }

private:
    const char* const* overlayNames() const override
    {
        static const char* names[] = {"wireframe", nullptr};
        return names;
    }

    static void onContact(const ContactEventInfo& info, void* userData)
    {
        auto* self = static_cast<Fracture3D*>(userData);
        if (info.event != ContactEvent::Enter)
            return;

        // Either side might be the ball; the other, if still unbroken, is recorded and shattered in step().
        if (info.bodyA != self->mBallId && info.bodyB != self->mBallId)
            return;
        const u32 other = info.bodyA == self->mBallId ? info.bodyB : info.bodyA;
        const BodyEntry* ball = self->mWorld.body(self->mBallId);
        const BodyEntry* block = self->mWorld.body(other);
        if (!ball || !block || !ball->body || !block->body)
            return;
        // A hard hit shows as closing speed, not depth: a speculative contact stops
        // the ball at the surface.
        const f32 approach =
            fabsf(M::dot(ball->body->velocity() - block->body->velocity(), info.normal));
        self->considerShatter(other, approach);
    }

    void considerShatter(u32 bodyId, f32 approach)
    {
        if (approach < kShatterSpeed)
            return;
        for (u32 id : mUnbroken)
            if (id == bodyId)
            {
                mPendingShatter.push_back(bodyId);
                mHitDetected = true;
                return;
            }
    }

    void shatterBlock(u32 bodyId)
    {
        BodyEntry* entry = mWorld.body(bodyId);
        if (!entry || !entry->body)
            return;

        const M::vec3 position = entry->body->position();
        const M::vec3 half(0.5f);

        // Source point cloud: the block's own 8 corners, local space (as VoronoiShatter::shatter wants).
        ct::Vector<M::vec3> sourceVerts;
        for (int i = 0; i < 8; ++i)
            sourceVerts.push_back(M::vec3((i & 1) ? half.x : -half.x, (i & 2) ? half.y : -half.y,
                                          (i & 4) ? half.z : -half.z));

        // Voronoi sites inside the block, seeded from its id so a --headless run is reproducible.
        ct::Vector<M::vec3> sites;
        u32 seed = bodyId * 2654435761u + 1u;
        for (int i = 0; i < kShardsPerBlock; ++i)
        {
            auto rnd = [&seed]() {
                seed = seed * 1103515245u + 12345u;
                return f32((seed >> 16) & 0x7fffu) / f32(0x7fffu) - 0.5f;
            };
            sites.push_back(M::vec3(rnd(), rnd(), rnd()) * half * 1.6f);
        }

        ct::Vector<Shard> shards;
        VoronoiShatter::shatter(sourceVerts, sites, shards);

        mWorld.removeBody(bodyId);
        for (u32& id : mUnbroken)
            if (id == bodyId)
                id = 0xFFFFFFFFu;

        for (const Shard& shard : shards)
        {
            if (shard.vertices.empty())
                continue;

            RigidBody* body = new RigidBody();
            body->setPosition(position + shard.centroid);
            body->setBodyType(BodyType::Dynamic);
            const f32 mass = kinetix::Max(shard.volume * kDensity, 0.05f);
            body->setMass(mass);

            ConvexHullShape* shape = new ConvexHullShape(shard);
            body->setInertiaTensor(shape->inertia(mass));

            BodyEntry newEntry;
            newEntry.body = body;
            newEntry.shape = shape;
            newEntry.friction = 0.6f;
            const u32 id = mWorld.addBody(newEntry);

            mBodies.push_back(body);
            mShapes.push_back(shape);
            mShardIds.push_back(id);
        }

        ++mShatterCount;
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
        // The world's id, not an index into mBodies (ContactEventInfo reports pairs by id).
        mLastBodyId = mWorld.addBody(entry);

        mBodies.push_back(body);
        mShapes.push_back(shape);
        return body;
    }

    static constexpr int kBlocks = 5;
    static constexpr int kShardsPerBlock = 6;
    static constexpr f32 kShatterSpeed = 4.0f;
    static constexpr f32 kDensity = 3.0f;

    PhysicsWorld mWorld;
    MouseJoint* mMouseJoint = nullptr;
    f32 mGrabDistance = 10.0f;
    Camera3D mCamera;
    float mElapsed = 0.0f;

    ct::Vector<RigidBody*> mBodies;
    ct::Vector<CollisionShape*> mShapes;
    ct::Vector<Joint*> mJoints;
    ct::Vector<u32> mUnbroken;
    ct::Vector<u32> mShardIds;
    ct::Vector<u32> mPendingShatter;
    u32 mBallId = 0;
    u32 mLastBodyId = 0;

    bool mHitDetected = false;
    int mShatterCount = 0;
    f32 mFastestShardSpeed = 0.0f;
};

REGISTER_SCENE(Fracture3D, "3D demo: fracture", Category::Physics3D)

} // namespace testbed
