#include "scene.h"

#include "render3d.h"

#include <collision/CollisionShape.h>
#include <dynamics/PhysicsWorld.h>
#include <dynamics/RigidBody.h>
#include <geometry/ConvexHullComputer.h>
#include <marine/Boat.h>
#include <marine/WaterSurface.h>

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

namespace
{
constexpr f32 kPi = 3.14159265f;
constexpr f32 kDeg = kPi / 180.0f;

constexpr f32 kThrottleTime = 2.0f;
constexpr f32 kTurnTime = 6.0f;
constexpr f32 kThrottle = 0.8f;
constexpr f32 kRudder = 0.7f;
constexpr f32 kVerdictTime = 9.5f;

constexpr f32 kLeverRate = 0.5f;
constexpr f32 kHelmRate = 1.2f;
constexpr f32 kHelmReturn = 1.5f;

constexpr int kWaterCells = 48;
constexpr f32 kWaterCell = 2.0f;

const M::vec3 kDockHalfExtents(2.0f, 2.5f, 15.0f);
const M::vec3 kDockCentre(6.0f, -1.0f, 5.0f);
} // namespace

// The default motorboat on a moderate sea, moored beside a pier. Buoyancy integrates hydrostatic
// pressure over the hull mesh clipped at the waterline; thrust from blade element momentum theory.
class Boat3D final : public Scene
{
public:
    Boat3D()
    {
        mWorld.setGravity(M::vec3(0.0f, -9.81f, 0.0f));
        mWorld.setFixedStep(1.0f / 60.0f);

        GerstnerWaves::Wave swell;
        swell.amplitude = 0.3f;
        swell.wavelength = 32.0f;
        swell.direction = M::vec2(0.15f, -1.0f);
        swell.steepness = 0.8f;
        mWater.addWave(swell);
        GerstnerWaves::Wave sea;
        sea.amplitude = 0.12f;
        sea.wavelength = 11.0f;
        sea.direction = M::vec2(1.0f, -0.4f);
        sea.phase = 1.3f;
        sea.steepness = 0.7f;
        mWater.addWave(sea);
        GerstnerWaves::Wave chop;
        chop.amplitude = 0.04f;
        chop.wavelength = 4.5f;
        chop.direction = M::vec2(-0.6f, -0.8f);
        chop.phase = 2.4f;
        chop.steepness = 0.6f;
        mWater.addWave(chop);

        mDock.setBodyType(BodyType::Static);
        mDock.setPosition(kDockCentre);
        BodyEntry dock;
        dock.body = &mDock;
        dock.shape = &mDockShape;
        dock.friction = 0.6f;
        mWorld.addBody(dock);
        mWorld.markStaticBroadphaseDirty();

        mLayout = buildSmallMotorboat(mBoat, mBody);
        mBoat.setWater(&mWater);
        // Collides as the convex hull of its hull mesh: the mesh is not centred on the centre of mass,
        // so a box would need an offset the world cannot give it.
        const HullMesh& hull = mBoat.hull().hull();
        kinetix::Geometry::ConvexHullComputer computer;
        computer.compute(&hull.vertices()[0].x, sizeof(M::vec3), static_cast<int>(hull.vertexCount()), 0.0f,
                         0.0f);
        mHullShape = new ConvexHullShape(computer.vertices.data(), static_cast<u32>(computer.vertices.size()),
                                         computer.edges.data(), static_cast<u32>(computer.edges.size()),
                                         computer.faces.data(), static_cast<u32>(computer.faces.size()));
        BodyEntry entry;
        entry.body = &mBody;
        entry.shape = mHullShape;
        entry.friction = 0.4f;
        mWorld.addBody(entry);
        place();

        mCamera.target = mBody.position();
        mCamera.distance = 20.0f;
        mCamera.pitch = 0.32f;
        mCamera.yaw = kPi + 0.5f;

        addCheck("floats at a sane draft");
        addCheck("drives forward under throttle");
        addCheck("turns to starboard with the rudder");
        addCheck("stays upright and finite");
    }

    ~Boat3D() override
    {
        delete mHullShape;
    }

    const char* name() const override
    {
        return "3D demo: boat";
    }
    const char* description() const override
    {
        return "A 6 m, 1250 kg outboard motorboat on a moderate sea, beside a pier. It drifts for "
               "two seconds, runs out at 80% throttle and then turns to starboard.\n"
               "Buoyancy is the waves' hydrostatic pressure integrated over the hull mesh, clipped "
               "at the waterline (red). Drag and planing lift come from every wetted triangle, "
               "thrust from blade element momentum theory on the propeller (yellow: thrust). "
               "C restarts.\n"
               "Manual mode (P): take the helm - see the controls panel.";
    }

    bool drivable() const override
    {
        return true;
    }
    void setManual(bool manual) override
    {
        if (manual == mManual)
            return;
        mManual = manual;
        if (manual)
        {
            mThrottle = mBoat.controls().throttle;
            mRudder = mBoat.controls().rudder;
        }
        else
            restart();
    }
    const char* controlsHelp() const override
    {
        return "W / S, Up / Down\tthrottle ahead / astern (the lever stays set)\n"
               "X\tthrottle to neutral\n"
               "A / D, Left / Right\thelm to port / starboard\n"
               "R or C\tback beside the pier\n"
               "gamepad\ttriggers: throttle, left stick: helm";
    }

    void control(const ControlInput& input, float dt) override
    {
        if (input.hit(Key::R))
        {
            restart();
            return;
        }
        const f32 lever = input.pair(Key::S, Key::W, Key::Down, Key::Up) + input.axis(PadAxis::RightTrigger) -
                          input.axis(PadAxis::LeftTrigger);
        mThrottle = M::clamp(mThrottle + lever * kLeverRate * dt, -1.0f, 1.0f);
        if (input.hit(Key::X))
            mThrottle = 0.0f;
        const f32 keys = input.pair(Key::A, Key::D, Key::Left, Key::Right);
        mRudder = rampControl(mRudder, keys != 0.0f ? keys : input.axis(PadAxis::LeftX), 0.0f, kHelmRate,
                              kHelmReturn, dt);
    }

    void readouts(ct::Vector<Readout>& out) const override
    {
        const BoatTelemetry& t = mBoat.telemetry();
        addReadout(out, "speed", "%.1f kn  (%.1f m/s through the water)", double(t.forwardSpeed * 1.94384f),
                   double(t.forwardSpeed));
        addReadout(out, "over the ground", "%.1f kn", double(t.groundSpeed * 1.94384f));
        addReadout(out, "engine", "%.0f rpm, thrust %.1f kN", double(t.engineRpm), double(t.thrust / 1000.0f));
        addReadout(out, "throttle / helm", "%+.0f%% / %+.2f", double(mBoat.controls().throttle * 100.0f),
                   double(mBoat.controls().rudder));
        addReadout(out, "heading", "%.0f deg, turning %.1f deg/s", double(t.heading / kDeg),
                   double(t.yawRate / kDeg));
        addReadout(out, "heel / trim", "%.1f / %.1f deg", double(t.heel / kDeg), double(t.trim / kDeg));
        addReadout(out, "draft", "%.2f m", double(t.draft));
        addReadout(out, "propeller immersion", "%.0f%%", double(t.immersion * 100.0f));
    }

    bool is3D() const override
    {
        return true;
    }
    Camera3D camera3D() const override
    {
        return mCamera;
    }

    void handleInput(const Input& input) override
    {
        if (input.cutPressed)
            restart();
    }

    void step(float dt) override
    {
        mElapsed += dt;
        if (mManual)
        {
            BoatControls c;
            c.throttle = mThrottle;
            c.rudder = mRudder;
            mBoat.setControls(c);
        }
        else
            helm();
        mBoat.applyForces(dt);
        mWorld.step(dt);
        mWater.advance(dt);

        const M::vec3 p = mBody.position();
        const BoatTelemetry& t = mBoat.telemetry();
        mFinite = mFinite && isfinite(p.x) && isfinite(p.y) && isfinite(p.z) && isfinite(t.engineRpm) &&
                  isfinite(mBody.angularVelocity().x);
        mLowestUp = fminf(mLowestUp, mBody.directionToWorld(M::vec3(0.0f, 1.0f, 0.0f)).y);
        if (mElapsed >= kTurnTime)
        {
            f32 delta = t.heading - mPreviousHeading;
            if (delta > kPi)
                delta -= 2.0f * kPi;
            if (delta < -kPi)
                delta += 2.0f * kPi;
            mTurned += delta;
        }
        mPreviousHeading = t.heading;
        mCamera.target = p;
        mCamera.chase = mManual;
        if (mManual)
        {
            const M::vec3 forward = mBody.directionToWorld(M::vec3(0.0f, 0.0f, 1.0f));
            if (forward.x * forward.x + forward.z * forward.z > 1.0e-4f)
                mCamera.yaw = atan2f(-forward.x, -forward.z);
        }
    }

    void draw() override
    {
        setupLighting3D();
        mBodies.clear();
        mShapes.clear();
        mBodies.push_back(&mDock);
        mShapes.push_back(&mDockShape);
        drawWorldSolid(mWorld, mBodies, mShapes);
        drawHull();
        drawWaterline();
        drawPropeller();
        drawWater();
    }

    void evaluate() override
    {
        const BoatTelemetry& t = mBoat.telemetry();
        char buffer[128];

        if (mElapsed >= kThrottleTime - 0.1f && !mDraftJudged)
        {
            mDraftJudged = true;
            mDraftOk = t.draft > 0.2f && t.draft < 0.55f && fabsf(t.heel) < 10.0f * kDeg &&
                       fabsf(t.trim) < 10.0f * kDeg;
            snprintf(mDraftDetail, sizeof(mDraftDetail), "draft %.2f m, heel %.1f, trim %.1f deg",
                     double(t.draft), double(t.heel / kDeg), double(t.trim / kDeg));
        }
        if (mDraftJudged)
            report(0, mDraftOk, mDraftDetail);

        if (mElapsed >= kTurnTime - 0.1f && !mRunJudged)
        {
            mRunJudged = true;
            const f32 run = mBody.position().z - mStart.z;
            mRunOk = t.forwardSpeed > 5.0f && run > 10.0f;
            snprintf(mRunDetail, sizeof(mRunDetail), "%.1f m/s through the water, %.0f m run, %.0f rpm",
                     double(t.forwardSpeed), double(run), double(t.engineRpm));
        }
        if (mRunJudged)
            report(1, mRunOk, mRunDetail);

        if (mElapsed >= kVerdictTime && !mTurnJudged)
        {
            mTurnJudged = true;
            mTurnOk = mTurned > 30.0f * kDeg && t.yawRate > 0.1f;
            snprintf(mTurnDetail, sizeof(mTurnDetail), "turned %.0f deg, %.2f rad/s, heel %.1f deg",
                     double(mTurned / kDeg), double(t.yawRate), double(t.heel / kDeg));
        }
        if (mTurnJudged)
            report(2, mTurnOk, mTurnDetail);

        snprintf(buffer, sizeof(buffer), "lowest up %.3f", double(mLowestUp));
        report(3, mFinite && mLowestUp > 0.8f, buffer);
    }

private:
    void place()
    {
        mStart = M::vec3(0.0f, mWater.height(0.0f, 0.0f) + mLayout.restHeight, 0.0f);
        mBody.setPosition(mStart);
        mBody.setOrientation(M::quat());
        mBody.setVelocity(M::vec3(0.0f));
        mBody.setAngularVelocity(M::vec3(0.0f));
        mBody.clearAccumulators();
        mBody.calculateDerivedData();
    }

    void restart()
    {
        mWater.setTime(0.0f);
        mBoat = Boat();
        mLayout = buildSmallMotorboat(mBoat, mBody);
        mBoat.setWater(&mWater);
        place();
        mElapsed = 0.0f;
        mTurned = 0.0f;
        mLowestUp = 1.0f;
        mFinite = true;
        mDraftJudged = mRunJudged = mTurnJudged = false;
        resetChecks();
        mThrottle = 0.0f;
        mRudder = 0.0f;
    }

    void helm()
    {
        BoatControls c;
        if (mElapsed >= kThrottleTime)
            c.throttle = kThrottle;
        if (mElapsed >= kTurnTime)
            c.rudder = kRudder;
        mBoat.setControls(c);
    }

    void drawHull()
    {
        const HullMesh& hull = mBoat.hull().hull();
        const M::mat4& transform = mBody.transform();
        glEnable(GL_LIGHTING);
        glPushMatrix();
        glMultMatrixf(&transform[0][0]);
        glBegin(GL_TRIANGLES);
        for (u32 t = 0; t < hull.triangleCount(); ++t)
        {
            const M::vec3& n = hull.triangleNormal(t);
            if (n.y > 0.7f)
                glColor3f(0.72f, 0.58f, 0.40f);
            else if (n.y < -0.2f)
                glColor3f(0.55f, 0.20f, 0.16f);
            else
                glColor3f(0.92f, 0.92f, 0.90f);
            glNormal3f(n.x, n.y, n.z);
            for (u32 k = 0; k < 3; ++k)
            {
                const M::vec3& v = hull.vertices()[hull.indices()[3 * t + k]];
                glVertex3f(v.x, v.y, v.z);
            }
        }
        glEnd();
        glPopMatrix();
        glDisable(GL_LIGHTING);
    }

    void drawWaterline()
    {
        const ct::Vector<M::vec3>& line = mBoat.hull().waterline();
        glLineWidth(2.0f);
        glColor3f(0.9f, 0.1f, 0.1f);
        glBegin(GL_LINES);
        for (const M::vec3& p : line)
            glVertex3f(p.x, p.y + 0.01f, p.z);
        glEnd();
        glLineWidth(1.0f);
    }

    void drawPropeller()
    {
        const MarinePropeller& prop = mBoat.propeller(0);
        const M::vec3 hub = prop.hubPosition(mBody);
        const M::vec3 axis = prop.thrustAxis(mBody);
        const M::vec3 top = hub + mBody.directionToWorld(M::vec3(0.0f, 0.75f, 0.0f));
        glColor3f(0.15f, 0.15f, 0.18f);
        glLineWidth(3.0f);
        glBegin(GL_LINES);
        glVertex3f(top.x, top.y, top.z);
        glVertex3f(hub.x, hub.y, hub.z);
        glEnd();
        glLineWidth(1.0f);
        const M::vec3 thrust = hub - axis * (prop.loads().thrust * 4.0e-4f);
        glColor3f(1.0f, 0.75f, 0.1f);
        glBegin(GL_LINES);
        glVertex3f(hub.x, hub.y, hub.z);
        glVertex3f(thrust.x, thrust.y, thrust.z);
        glEnd();
    }

    void drawWater()
    {
        // Rest-position grid snapped to the cell size so the mesh does not swim with the boat.
        const M::vec3 centre = mBody.position();
        const f32 x0 = floorf(centre.x / kWaterCell) * kWaterCell - 0.5f * kWaterCells * kWaterCell;
        const f32 z0 = floorf(centre.z / kWaterCell) * kWaterCell - 0.5f * kWaterCells * kWaterCell;
        const f32 time = mWater.time();
        for (int j = 0; j <= kWaterCells; ++j)
            for (int i = 0; i <= kWaterCells; ++i)
                mGrid[j][i] = mWater.surfacePoint(x0 + i * kWaterCell, z0 + j * kWaterCell, time);

        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glEnable(GL_LIGHTING);
        glDepthMask(GL_FALSE);
        glBegin(GL_TRIANGLES);
        for (int j = 0; j < kWaterCells; ++j)
        {
            for (int i = 0; i < kWaterCells; ++i)
            {
                const M::vec3& a = mGrid[j][i];
                const M::vec3& b = mGrid[j][i + 1];
                const M::vec3& c = mGrid[j + 1][i + 1];
                const M::vec3& d = mGrid[j + 1][i];
                waterTriangle(a, d, c);
                waterTriangle(a, c, b);
            }
        }
        glEnd();
        glDepthMask(GL_TRUE);
        glDisable(GL_LIGHTING);

        glColor4f(0.75f, 0.88f, 0.95f, 0.35f);
        glBegin(GL_LINES);
        for (int j = 0; j <= kWaterCells; ++j)
        {
            for (int i = 0; i < kWaterCells; ++i)
            {
                const M::vec3& a = mGrid[j][i];
                const M::vec3& b = mGrid[j][i + 1];
                glVertex3f(a.x, a.y + 0.01f, a.z);
                glVertex3f(b.x, b.y + 0.01f, b.z);
                const M::vec3& c = mGrid[i][j];
                const M::vec3& d = mGrid[i + 1][j];
                glVertex3f(c.x, c.y + 0.01f, c.z);
                glVertex3f(d.x, d.y + 0.01f, d.z);
            }
        }
        glEnd();
        glDisable(GL_BLEND);
    }

    static void waterTriangle(const M::vec3& a, const M::vec3& b, const M::vec3& c)
    {
        M::vec3 n = M::cross(b - a, c - a);
        if (n.y < 0.0f)
            n = -n;
        const f32 lift = M::clamp((a.y + b.y + c.y) / 3.0f * 0.8f, -0.25f, 0.25f);
        glColor4f(0.10f + lift * 0.4f, 0.32f + lift * 0.5f, 0.52f + lift * 0.4f, 0.78f);
        glNormal3f(n.x, n.y, n.z);
        glVertex3f(a.x, a.y, a.z);
        glVertex3f(b.x, b.y, b.z);
        glVertex3f(c.x, c.y, c.z);
    }

    PhysicsWorld mWorld;
    Camera3D mCamera;
    GerstnerWaves mWater;
    RigidBody mDock;
    BoxShape mDockShape{kDockHalfExtents};
    RigidBody mBody;
    ConvexHullShape* mHullShape = nullptr;
    Boat mBoat;
    SmallMotorboatLayout mLayout;
    ct::Vector<RigidBody*> mBodies;
    ct::Vector<CollisionShape*> mShapes;
    M::vec3 mGrid[kWaterCells + 1][kWaterCells + 1];

    M::vec3 mStart{0.0f};
    f32 mElapsed = 0.0f;
    f32 mPreviousHeading = 0.0f;
    f32 mTurned = 0.0f;
    f32 mLowestUp = 1.0f;
    bool mFinite = true;
    bool mDraftJudged = false;
    bool mDraftOk = false;
    bool mRunJudged = false;
    bool mRunOk = false;
    bool mTurnJudged = false;
    bool mTurnOk = false;
    char mDraftDetail[96] = {};
    char mRunDetail[96] = {};
    char mTurnDetail[96] = {};

    f32 mThrottle = 0.0f;
    f32 mRudder = 0.0f;
};

REGISTER_SCENE(Boat3D, "3D demo: boat", Category::Physics3D)

} // namespace testbed
