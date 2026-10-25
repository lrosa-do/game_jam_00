#include "scene.h"

#include "render3d.h"

#include <collision/CollisionShape.h>
#include <dynamics/PhysicsWorld.h>
#include <dynamics/RaycastVehicle.h>
#include <dynamics/RigidBody.h>
#include <effects/ParticleSystem.h>

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

class VehicleBase : public Scene
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

    ~VehicleBase() override
    {
        for (RigidBody* body : mBodies)
            delete body;
        for (CollisionShape* shape : mShapes)
            delete shape;
    }

    RigidBody* addBox(const M::vec3& position, const M::vec3& halfExtents, bool isStatic,
                      f32 mass, u32* outId = nullptr)
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
        entry.friction = 0.8f;
        const u32 id = mWorld.addBody(entry);
        if (outId)
            *outId = id;

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

// A RaycastVehicle's wheels are rays cast down from the chassis; they have no collision of their own.
class RaycastCar final : public VehicleBase
{
public:
    RaycastCar()
    {
        mWorld.setGravity(M::vec3(0.0f, -9.81f, 0.0f));
        addBox(M::vec3(0.0f, -1.0f, 0.0f), M::vec3(400.0f, 1.0f, 400.0f), true, 0.0f);

        for (int i = 0; i < 4; ++i)
        {
            const f32 z = 22.0f + float(i) * 26.0f;
            // Negative angle about X so a car driving along +z meets the low side first. The car drives
            // on the top face, so the near TOP corner sits at floor level: centreY + h*cos(a) - L*sin(a).
            constexpr f32 kAngle = 0.16f;
            constexpr f32 kHalfHeight = 0.3f;
            constexpr f32 kHalfLength = 4.0f;
            const f32 centreY = kHalfLength * sinf(kAngle) - kHalfHeight * cosf(kAngle);

            RigidBody* ramp = addBox(M::vec3(0.0f, centreY, z),
                                     M::vec3(5.0f, kHalfHeight, kHalfLength), true, 0.0f);
            ramp->setOrientation(M::angleAxis(-kAngle, M::vec3(1.0f, 0.0f, 0.0f)));
        }
        for (int i = 0; i < 14; ++i)
        {
            const f32 z = 8.0f + float(i) * 12.0f;
            addBox(M::vec3(-7.0f, 0.4f, z), M::vec3(0.4f, 0.4f, 0.4f), true, 0.0f);
            addBox(M::vec3(7.0f, 0.4f, z), M::vec3(0.4f, 0.4f, 0.4f), true, 0.0f);
        }
        mWorld.markStaticBroadphaseDirty();

        u32 chassisId = 0;
        mChassis = addBox(M::vec3(0.0f, 1.2f, 0.0f), M::vec3(1.0f, 0.3f, 2.0f), false, 800.0f,
                          &chassisId);
        mGroundExtent = 120.0f;

        mVehicle = new RaycastVehicle(*mChassis, &mWorld, chassisId);

        RaycastVehicle::Tuning tuning;
        const f32 restLength = 0.4f;
        const f32 radius = 0.4f;
        const M::vec3 down(0.0f, -1.0f, 0.0f);
        const M::vec3 axle(-1.0f, 0.0f, 0.0f);

        mVehicle->addWheel(M::vec3(0.9f, -0.2f, -1.4f), down, axle, restLength, radius, tuning, true);
        mVehicle->addWheel(M::vec3(-0.9f, -0.2f, -1.4f), down, axle, restLength, radius, tuning, true);
        mVehicle->addWheel(M::vec3(0.9f, -0.2f, 1.4f), down, axle, restLength, radius, tuning, false);
        mVehicle->addWheel(M::vec3(-0.9f, -0.2f, 1.4f), down, axle, restLength, radius, tuning, false);

        mStartZ = mChassis->position().z;

        mCamera.target = M::vec3(0.0f, 1.0f, 0.0f);
        mCamera.distance = 11.0f;
        mCamera.pitch = 0.35f;
        mCamera.yaw = 3.14159265f;

        addCheck("the car drives forward");
        addCheck("it stays upright");
        addCheck("it does not sink through the ground");
    }

    ~RaycastCar() override { delete mVehicle; }

    const char* name() const override { return "3D demo: raycast vehicle"; }
    const char* description() const override
    {
        return "A four-wheel raycast vehicle driving at a ramp. The wheels are rays, not\n"
               "bodies: suspension is the ray length and grip comes from a friction\n"
               "model, so there are no wheel collisions to tunnel or jitter.\n"
               "Manual mode (P): drive it with the arrow keys.";
    }

    bool drivable() const override { return true; }
    const char* controlsHelp() const override
    {
        return "Up / Down (or W / S)\tdrive / brake, then reverse\n"
               "Left / Right (or A / D)\tsteer\n"
               "X (hold)\tbrake\n"
               "R\tstart over";
    }

    void control(const ControlInput& input, float dt) override
    {
        if (input.hit(Key::R))
        {
            restartRequested = true;
            return;
        }
        mDrive = input.pair(Key::Down, Key::Up, Key::S, Key::W);
        mBrakeHeld = input.down(Key::X);
        mSteer = rampControl(mSteer, input.pair(Key::Left, Key::Right, Key::A, Key::D), 0.0f, 2.0f, 3.0f, dt);
    }

    void readouts(ct::Vector<Readout>& out) const override
    {
        const M::vec3 forward = mChassis->directionToWorld(M::vec3(0.0f, 0.0f, 1.0f));
        addReadout(out, "speed", "%.1f km/h", double(M::dot(mChassis->velocity(), forward) * 3.6f));
        addReadout(out, "steering", "%+.0f%%", double(mSteer * 100.0f));
        addReadout(out, "heading", "%.0f deg", double(atan2f(-forward.x, forward.z) * 57.29578f));
        addReadout(out, "travelled", "%.1f m along the route", double(mChassis->position().z - mStartZ));
    }

    void step(float dt) override
    {
        if (mManual)
            driveByHand();
        else
        {
            // Raycast vehicles need engine force applied every step.
            for (u32 i = 2; i < 4; ++i)
                mVehicle->setEngineForce(900.0f, i);
        }

        mWorld.step(dt);
        mVehicle->update(dt);
        mElapsed += dt;

        if (mChassis->position().y < mLowest)
            mLowest = mChassis->position().y;
        if (mChassis->position().z > mFurthest)
            mFurthest = mChassis->position().z;

        mCamera.target = mChassis->position() + M::vec3(0.0f, 0.6f, 0.0f);
        mCamera.chase = mManual;
        if (mManual)
        {
            const M::vec3 forward = mChassis->directionToWorld(M::vec3(0.0f, 0.0f, 1.0f));
            if (forward.x * forward.x + forward.z * forward.z > 1.0e-4f)
                mCamera.yaw = atan2f(-forward.x, -forward.z);
        }
    }

    void draw() override
    {
        drawWorld();

        // The wheels are rays, not bodies: draw from the transform the vehicle reports so steering
        // and suspension travel show.
        for (u32 i = 0; i < mVehicle->wheelCount(); ++i)
        {
            const RaycastVehicle::Wheel& wheel = mVehicle->wheel(i);
            const M::mat4& m = wheel.worldTransform;
            const M::vec3 centre(m[3][0], m[3][1], m[3][2]);
            const M::vec3 axle(m[0][0], m[0][1], m[0][2]);
            const M::vec3 up(m[1][0], m[1][1], m[1][2]);
            const M::vec3 forward(m[2][0], m[2][1], m[2][2]);

            glDisable(GL_LIGHTING);
            glColor3f(0.95f, 0.75f, 0.20f);
            for (int side = -1; side <= 1; side += 2)
            {
                const M::vec3 offset = axle * (0.12f * float(side));
                glBegin(GL_LINE_LOOP);
                for (int seg = 0; seg < 20; ++seg)
                {
                    const float a = 2.0f * 3.14159265f * float(seg) / 20.0f;
                    const M::vec3 rim =
                        centre + offset + up * (0.4f * sinf(a)) + forward * (0.4f * cosf(a));
                    glVertex3f(rim.x, rim.y, rim.z);
                }
                glEnd();
            }
            glBegin(GL_LINES);
            for (int spoke = 0; spoke < 4; ++spoke)
            {
                const float a = 3.14159265f * 0.5f * float(spoke);
                const M::vec3 rim = centre + up * (0.4f * sinf(a)) + forward * (0.4f * cosf(a));
                glVertex3f(centre.x, centre.y, centre.z);
                glVertex3f(rim.x, rim.y, rim.z);
            }
            glEnd();
        }
    }

    void evaluate() override
    {
        char buf[80];
        snprintf(buf, sizeof(buf), "travelled %.1f m", double(mFurthest - mStartZ));
        if (mElapsed > 3.0f)
            report(0, mFurthest > mStartZ + 3.0f, buf);

        const M::vec3 up = mChassis->orientation() * M::vec3(0.0f, 1.0f, 0.0f);
        snprintf(buf, sizeof(buf), "up.y = %.2f", double(up.y));
        report(1, up.y > 0.3f, buf);

        snprintf(buf, sizeof(buf), "lowest y %.2f m", double(mLowest));
        report(2, mLowest > -0.5f, buf);
    }

private:
    // Wheels 2 and 3 lead (the car drives along +Z): they drive and steer.
    // Down brakes while it is still rolling forward, then reverses.
    void driveByHand()
    {
        const M::vec3 forward = mChassis->directionToWorld(M::vec3(0.0f, 0.0f, 1.0f));
        const f32 speed = M::dot(mChassis->velocity(), forward);
        const bool braking = mBrakeHeld || (mDrive < 0.0f && speed > 0.5f) || (mDrive > 0.0f && speed < -0.5f);
        for (u32 i = 0; i < 4; ++i)
        {
            const bool driven = i >= 2;
            mVehicle->setEngineForce(!braking && driven ? mDrive * (mDrive > 0.0f ? 1500.0f : 800.0f) : 0.0f, i);
            mVehicle->setBrake(braking ? 20.0f : 0.0f, i);
            if (driven)
                mVehicle->setSteering(-mSteer * 0.35f, i);
        }
    }

    RigidBody* mChassis = nullptr;
    RaycastVehicle* mVehicle = nullptr;
    f32 mDrive = 0.0f;
    f32 mSteer = 0.0f;
    bool mBrakeHeld = false;
    f32 mStartZ = 0.0f;
    f32 mFurthest = -1e9f;
    f32 mLowest = 1e9f;
};

// Particles are swept points that collide with the world, each with its own response; cheap enough for thousands.
class Particles3D final : public VehicleBase
{
public:
    Particles3D()
    {
        mWorld.setGravity(M::vec3(0.0f, -9.81f, 0.0f));
        addBox(M::vec3(0.0f, -1.0f, 0.0f), M::vec3(30.0f, 1.0f, 30.0f), true, 0.0f);

        addBox(M::vec3(0.0f, 2.0f, 0.0f), M::vec3(3.0f, 0.25f, 3.0f), true, 0.0f);
        mWorld.markStaticBroadphaseDirty();

        mParticles = new ParticleSystem(mWorld);

        mCamera.target = M::vec3(0.0f, 2.0f, 0.0f);
        mCamera.distance = 20.0f;

        addCheck("particles are emitted");
        addCheck("they expire rather than accumulating forever");
    }

    ~Particles3D() override { delete mParticles; }

    const char* name() const override { return "3D demo: particles"; }
    const char* description() const override
    {
        return "A fountain of bouncing particles over a static slab. Particles are swept\n"
               "points with their own lifetime and restitution - not rigid bodies - so a\n"
               "few thousand cost what a handful of bodies would.";
    }

    void step(float dt) override
    {
        mElapsed += dt;

        mEmitTimer += dt;
        while (mEmitTimer > 0.02f)
        {
            mEmitTimer -= 0.02f;
            for (int i = 0; i < 6; ++i)
            {
                // Deterministic spread: a scene that differs run to run cannot be checked.
                mSeed = mSeed * 1103515245u + 12345u;
                const float a = float((mSeed >> 16) & 0x7fff) / float(0x7fff) * 6.2831853f;
                mSeed = mSeed * 1103515245u + 12345u;
                const float r = float((mSeed >> 16) & 0x7fff) / float(0x7fff);

                ParticleSpawn spawn;
                spawn.position = M::vec3(0.0f, 6.0f, 0.0f);
                spawn.velocity = M::vec3(cosf(a) * r * 4.0f, 2.0f, sinf(a) * r * 4.0f);
                spawn.radius = 0.05f;
                spawn.life = 3.0f;
                spawn.restitution = 0.4f;
                spawn.response = ParticleResponse::Bounce;
                mParticles->emit(spawn);
                ++mEmitted;
            }
        }

        mWorld.step(dt);
        mParticles->step(dt);

        if (mParticles->count() > mPeakAlive)
            mPeakAlive = mParticles->count();
    }

    void draw() override
    {
        drawWorld();

        glDisable(GL_LIGHTING);
        glPointSize(4.0f);
        glBegin(GL_POINTS);
        for (usize i = 0; i < mParticles->count(); ++i)
        {
            const ParticleSystem::View p = mParticles->particle(i);
            const float t = p.position.y / 7.0f;
            glColor3f(1.0f, 0.45f + t * 0.4f, 0.25f + t * 0.5f);
            glVertex3f(p.position.x, p.position.y, p.position.z);
        }
        glEnd();
        glPointSize(1.0f);
    }

    void evaluate() override
    {
        char buf[96];
        snprintf(buf, sizeof(buf), "%d emitted, %d alive", int(mEmitted),
                 int(mParticles->count()));
        if (mElapsed > 0.5f)
            report(0, mEmitted > 0 && mParticles->count() > 0, buf);

        // Life 3 s and 300/s emission level the population off near 900; checks it stopped growing.
        snprintf(buf, sizeof(buf), "alive %d, emitted %d", int(mParticles->count()),
                 int(mEmitted));
        if (mElapsed > 8.0f)
            report(1, mParticles->count() <= mPeakAlive && mParticles->count() < 2000, buf);
    }

private:
    ParticleSystem* mParticles = nullptr;
    float mEmitTimer = 0.0f;
    usize mEmitted = 0;
    usize mPeakAlive = 0;
    unsigned mSeed = 987654321u;
};

REGISTER_SCENE(RaycastCar, "3D demo: raycast vehicle", Category::Physics3D)
REGISTER_SCENE(Particles3D, "3D demo: particles", Category::Physics3D)

} // namespace testbed
