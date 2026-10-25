#include "scene.h"

#include "render3d.h"

#include <collision/CollisionShape.h>
#include <dynamics/Aircraft.h>
#include <dynamics/PhysicsWorld.h>
#include <dynamics/RaycastVehicle.h>
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

namespace
{
constexpr f32 kPi = 3.14159265f;
constexpr f32 kDeg = kPi / 180.0f;

// A 10 m/s (20 kt) wind straight down the runway and a brakes-release start at full power put
// the liftoff inside the ten seconds the headless run gives each scene.
constexpr f32 kHeadwind = 10.0f;
constexpr f32 kRunUpRpm = 2500.0f;
constexpr f32 kRotateSpeed = 20.0f;   // m/s indicated
constexpr f32 kCircuitHeight = 150.0f;
constexpr f32 kRunwayLength = 1500.0f;
constexpr f32 kRunwayWidth = 30.0f;
constexpr f32 kVerdictTime = 9.5f;

constexpr f32 kThrottleRate = 0.5f; // full travel in two seconds
constexpr f32 kStickRate = 1.6f;
constexpr f32 kStickReturn = 2.5f;
constexpr f32 kTrimRate = 0.25f;
constexpr f32 kFlapRate = 0.3f;
constexpr f32 kFlapNotches[] = {0.0f, 0.33f, 0.67f, 1.0f};
constexpr int kFlapNotchCount = int(sizeof(kFlapNotches) / sizeof(kFlapNotches[0]));
// Brake impulse per main wheel per step (N s): about 0.3 g on a dry runway.
constexpr f32 kBrakeImpulse = 30.0f;
} // namespace

// The default light aircraft on a RaycastVehicle undercarriage, taking off under a scripted
// autopilot and flying a circuit. Lift per wing strip and propeller blade element momentum thrust
// are computed every step from the air it moves through, wind included.
class Airplane3D final : public Scene
{
public:
    Airplane3D()
    {
        mWorld.setGravity(M::vec3(0.0f, -9.81f, 0.0f));
        mWorld.setFixedStep(1.0f / 60.0f);

        // Static slab whose top is the ground plane, running away along +Z.
        mRunway.setBodyType(BodyType::Static);
        mRunway.setPosition(M::vec3(0.0f, -1.0f, 0.5f * kRunwayLength - 50.0f));
        BodyEntry runway;
        runway.body = &mRunway;
        runway.shape = &mRunwayShape;
        runway.friction = 0.8f;
        mWorld.addBody(runway);
        mWorld.markStaticBroadphaseDirty();

        mLayout = buildLightAircraft(mAircraft, mBody);
        mFuselageShape = BoxShape(mLayout.fuselageHalfExtents);
        mAircraft.ownAtmosphere().setWind(M::vec3(0.0f, 0.0f, -kHeadwind));
        mBody.setPosition(M::vec3(0.0f, mLayout.restHeight, 0.0f));
        BodyEntry entry;
        entry.body = &mBody;
        entry.shape = &mFuselageShape;
        entry.friction = 0.5f;
        mBodyId = mWorld.addBody(entry);

        mGear = new RaycastVehicle(mBody, &mWorld, mBodyId);
        addLightAircraftGear(*mGear, mLayout);
        mAircraft.propeller(0).setAngularSpeed(kRunUpRpm * 2.0f * kPi / 60.0f);

        mCamera.target = mBody.position();
        mCamera.distance = 28.0f;
        mCamera.pitch = 0.22f;
        mCamera.yaw = kPi;

        addCheck("becomes airborne");
        addCheck("climbs away");
        addCheck("stays on the centreline during the roll");
        addCheck("stays finite");
    }

    ~Airplane3D() override
    {
        delete mGear;
    }

    const char* name() const override
    {
        return "3D demo: airplane";
    }
    const char* description() const override
    {
        return "A 1000 kg single-engine light aircraft on a runway, into a 10 m/s headwind.\n"
               "A scripted autopilot holds the centreline, rotates at 20 m/s and climbs to\n"
               "a 150 m circuit. Lift, drag and moments come from strip theory on each wing\n"
               "with lifting-line downwash; thrust from blade element momentum theory.\n"
               "Strips turn red as they stall. C restarts the takeoff.\n"
               "Manual mode (P): fly it yourself - see the controls panel.";
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
            const ControlInputs& c = mAircraft.controls();
            mThrottle = c.throttle;
            mElevator = c.elevator;
            mAileron = c.aileron;
            mRudder = c.rudder;
            mFlaps = c.flaps;
            mFlapNotch = 0;
            for (int i = 0; i < kFlapNotchCount; ++i)
                if (fabsf(kFlapNotches[i] - c.flaps) < fabsf(kFlapNotches[mFlapNotch] - c.flaps))
                    mFlapNotch = i;
            mTrim = c.elevatorTrim;
        }
        else
            restart();
    }
    const char* controlsHelp() const override
    {
        return "W / S, PgUp / PgDn\tthrottle up / down (stays set)\n"
               "Down / Up\televator: pull (nose up) / push\n"
               "Left / Right\tailerons: roll left / right\n"
               "Q / E\trudder left / right (nose wheel on the ground)\n"
               "Home / End\televator trim nose up / down\n"
               "G / V\tflaps down / up one notch\n"
               "B (hold)\twheel brakes\n"
               "R or C\tback to the runway\n"
               "gamepad\tleft stick: pitch and roll, right stick: rudder, triggers: throttle";
    }

    void control(const ControlInput& input, float dt) override
    {
        if (input.hit(Key::R))
        {
            restart();
            return;
        }
        const f32 throttleCommand = input.pair(Key::S, Key::W, Key::PageDown, Key::PageUp) +
                                    input.axis(PadAxis::RightTrigger) - input.axis(PadAxis::LeftTrigger);
        mThrottle = M::clamp(mThrottle + throttleCommand * kThrottleRate * dt, 0.0f, 1.0f);

        auto command = [](f32 keys, f32 stick)
        {
            return keys != 0.0f ? keys : stick;
        };
        mElevator = rampControl(mElevator, command(input.pair(Key::Up, Key::Down), input.axis(PadAxis::LeftY)), 0.0f,
                                kStickRate, kStickReturn, dt);
        mAileron = rampControl(mAileron, command(input.pair(Key::Left, Key::Right), input.axis(PadAxis::LeftX)), 0.0f,
                               kStickRate, kStickReturn, dt);
        mRudder = rampControl(mRudder, command(input.pair(Key::Q, Key::E), input.axis(PadAxis::RightX)), 0.0f,
                              kStickRate, kStickReturn, dt);
        mTrim = M::clamp(mTrim + input.pair(Key::End, Key::Home) * kTrimRate * dt, -0.5f, 0.5f);

        if (input.hit(Key::G) && mFlapNotch + 1 < kFlapNotchCount)
            ++mFlapNotch;
        if (input.hit(Key::V) && mFlapNotch > 0)
            --mFlapNotch;
        mFlaps = approach(mFlaps, kFlapNotches[mFlapNotch], kFlapRate, dt);
        mBrakes = input.down(Key::B);
    }

    void readouts(ct::Vector<Readout>& out) const override
    {
        const AircraftTelemetry& t = mAircraft.telemetry();
        const ControlInputs& c = mAircraft.controls();
        addReadout(out, "airspeed (IAS)", "%.1f m/s  %.0f kt", double(t.equivalentAirspeed),
                   double(t.equivalentAirspeed * 1.94384f));
        addReadout(out, "height", "%.1f m", double(mBody.position().y - mLayout.restHeight));
        addReadout(out, "vertical speed", "%+.1f m/s", double(t.verticalSpeed));
        addReadout(out, "angle of attack", "%.1f deg", double(t.angleOfAttack / kDeg));
        addReadout(out, "pitch / bank", "%.1f / %.1f deg", double(t.pitch / kDeg), double(t.bank / kDeg));
        addReadout(out, "heading", "%.0f deg", double(t.heading / kDeg));
        addReadout(out, "propeller", "%.0f rpm", double(mAircraft.propeller(0).angularSpeed() * 60.0f / (2.0f * kPi)));
        addReadout(out, "throttle", "%.0f%%", double(c.throttle * 100.0f));
        addReadout(out, "elev / ail / rud", "%+.2f / %+.2f / %+.2f", double(c.elevator), double(c.aileron),
                   double(c.rudder));
        addReadout(out, "trim / flaps", "%+.2f / %.0f%%", double(c.elevatorTrim), double(c.flaps * 100.0f));
        addReadout(out, "gear", "%s%s", onGround() ? "on the ground" : "airborne",
                   mManual && mBrakes ? ", brakes" : "");
        addReadout(out, "position x / z", "%.1f / %.0f m", double(mBody.position().x), double(mBody.position().z));
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
            fly();
        else
            autopilot();
        mAircraft.applyForces(dt);
        mWorld.step(dt);
        mGear->update(dt);

        const M::vec3 p = mBody.position();
        mFinite = mFinite && isfinite(p.x) && isfinite(p.y) && isfinite(p.z) &&
                  isfinite(mAircraft.propeller(0).angularSpeed());
        if (onGround())
        {
            mAirborneTime = 0.0f;
            mWorstLateral = fmaxf(mWorstLateral, fabsf(p.x));
        }
        else
        {
            mAirborneTime += dt;
            if (mAirborneTime > 0.5f && !mAirborne)
            {
                mAirborne = true;
                mLiftoffTime = mElapsed - mAirborneTime;
                mGroundRoll = p.z;
                mLiftoffSpeed = mAircraft.telemetry().equivalentAirspeed;
            }
        }
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
        drawGround(-0.02f, 1200.0f, 20.0f);
        mBodies.clear();
        mShapes.clear();
        mBodies.push_back(&mRunway);
        mShapes.push_back(&mRunwayShape);
        mBodies.push_back(&mBody);
        mShapes.push_back(&mFuselageShape);
        drawWorldSolid(mWorld, mBodies, mShapes);

        drawWings();
        drawPropeller();
        drawGear();
        drawRunwayMarkings();
    }

    void evaluate() override
    {
        const AircraftTelemetry& t = mAircraft.telemetry();
        const f32 height = mBody.position().y - mLayout.restHeight;
        char buffer[128];

        if (mAirborne)
            snprintf(buffer, sizeof(buffer), "liftoff at %.1f s after %.0f m, %.1f m/s IAS",
                     double(mLiftoffTime), double(mGroundRoll), double(mLiftoffSpeed));
        else
            snprintf(buffer, sizeof(buffer), "rolling, %.1f m/s IAS", double(t.equivalentAirspeed));
        if (mAirborne || mElapsed >= kVerdictTime)
            report(0, mAirborne, buffer);

        // Verdict at kVerdictTime: it climbs away after liftoff, not that it never levels off.
        if (mElapsed >= kVerdictTime && !mClimbJudged)
        {
            mClimbJudged = true;
            mClimbing = mAirborne && height > 1.5f && t.verticalSpeed > 0.5f;
            snprintf(mClimbDetail, sizeof(mClimbDetail), "%.1f m up, climbing %.1f m/s",
                     double(height), double(t.verticalSpeed));
        }
        if (mClimbJudged)
            report(1, mClimbing, mClimbDetail);

        snprintf(buffer, sizeof(buffer), "worst %.2f m off", double(mWorstLateral));
        report(2, mWorstLateral < 3.0f, buffer);

        report(3, mFinite);
    }

private:
    bool onGround() const
    {
        for (u32 i = 0; i < mGear->wheelCount(); ++i)
            if (mGear->wheel(i).inContact)
                return true;
        return false;
    }

    void restart()
    {
        mBody.setPosition(M::vec3(0.0f, mLayout.restHeight, 0.0f));
        mBody.setOrientation(M::quat());
        mBody.setVelocity(M::vec3(0.0f));
        mBody.setAngularVelocity(M::vec3(0.0f));
        mBody.clearAccumulators();
        mGear->resetSuspension();
        mAircraft.setControls(ControlInputs());
        mAircraft.propeller(0).setAngularSpeed(kRunUpRpm * 2.0f * kPi / 60.0f);
        mElapsed = 0.0f;
        mAirborne = false;
        mAirborneTime = 0.0f;
        mWorstLateral = 0.0f;
        mBankTarget = 0.0f;
        mClimbJudged = false;
        mClimbing = false;
        resetChecks();
        mThrottle = 0.0f;
        mElevator = mAileron = mRudder = 0.0f;
        mTrim = 0.0f;
        mFlaps = 0.0f;
        mFlapNotch = 0;
        mBrakes = false;
        for (u32 i = 0; i < mGear->wheelCount(); ++i)
            mGear->setBrake(0.0f, i);
    }

    void fly()
    {
        ControlInputs c;
        c.throttle = mThrottle;
        c.elevator = mElevator;
        c.aileron = mAileron;
        c.rudder = mRudder;
        c.flaps = mFlaps;
        c.elevatorTrim = mTrim;
        mAircraft.setControls(c);
        mGear->setSteering(-mRudder * 0.3f, 0);
        // Wheels 1 and 2 are the mains (addLightAircraftGear's order).
        for (u32 i = 1; i < mGear->wheelCount(); ++i)
            mGear->setBrake(mBrakes ? kBrakeImpulse : 0.0f, i);
    }

    // Takeoff roll, rotation, climb, then a level circuit at 150 m (PD loops on telemetry).
    void autopilot()
    {
        const AircraftTelemetry& t = mAircraft.telemetry();
        const f32 height = mBody.position().y - mLayout.restHeight;
        const bool grounded = onGround();

        ControlInputs c;
        c.throttle = 1.0f;
        // Takeoff flap until clear of the ground, then cleaned up between 30 and 60 m; all at once costs lift.
        c.flaps = 0.67f * M::clamp((60.0f - height) / 30.0f, 0.0f, 1.0f);

        f32 pitchTarget = 0.0f;
        if (!grounded || t.equivalentAirspeed > kRotateSpeed)
            pitchTarget = 11.0f * kDeg;
        if (height > 30.0f)
            pitchTarget = 9.0f * kDeg;
        if (height > kCircuitHeight - 30.0f)
        {
            pitchTarget = M::clamp(2.0f * kDeg + (kCircuitHeight - height) * 0.004f -
                                       t.verticalSpeed * 0.01f,
                                   -5.0f * kDeg, 10.0f * kDeg);
            c.throttle = 0.75f;
            if (height > kCircuitHeight - 10.0f)
                mBankTarget = 20.0f * kDeg;
        }
        const bool pitching = !grounded || t.equivalentAirspeed > kRotateSpeed;
        c.elevator = pitching ? M::clamp((pitchTarget - t.pitch) * 3.0f - t.bodyRates.y, -1.0f, 1.0f)
                              : 0.0f;
        if (mBankTarget > 0.0f)
            c.elevator = M::clamp(c.elevator + 0.15f, -1.0f, 1.0f);

        c.aileron = M::clamp((mBankTarget - t.bank) * 2.0f - t.bodyRates.x * 0.5f, -1.0f, 1.0f);
        if (mBankTarget == 0.0f)
        {
            c.rudder = M::clamp(-t.heading * 3.0f - t.bodyRates.z - mBody.position().x * 0.05f,
                                -1.0f, 1.0f);
        }
        else
        {
            c.rudder = M::clamp(-t.sideslip * 4.0f, -1.0f, 1.0f);
        }
        mAircraft.setControls(c);
        mGear->setSteering(-c.rudder * 0.3f, 0);
    }

    void drawWings()
    {
        glDisable(GL_LIGHTING);
        for (u32 w = 0; w < mAircraft.wingCount(); ++w)
        {
            const Wing& wing = mAircraft.wing(w);
            for (u32 e = 0; e < wing.elementCount(); ++e)
            {
                const Wing::Element& element = wing.element(e);
                const M::vec3 centre = mBody.pointToWorld(element.position);
                const M::vec3 chord = mBody.directionToWorld(element.chordAxis) * element.chord;
                const M::vec3 span =
                    mBody.directionToWorld(element.pitchAxis) * (0.5f * element.area / element.chord);
                const M::vec3 leading = centre + chord * 0.25f;
                const M::vec3 trailing = centre - chord * 0.75f;
                const f32 s = element.separation;
                const f32 shade = element.control >= 0 ? 0.8f : 1.0f;
                glColor3f(shade * (0.85f + 0.15f * s), shade * (0.85f - 0.6f * s),
                          shade * (0.80f - 0.6f * s));
                glBegin(GL_QUADS);
                const M::vec3 a = leading + span;
                const M::vec3 b = leading - span;
                const M::vec3 c = trailing - span;
                const M::vec3 d = trailing + span;
                glVertex3f(a.x, a.y, a.z);
                glVertex3f(b.x, b.y, b.z);
                glVertex3f(c.x, c.y, c.z);
                glVertex3f(d.x, d.y, d.z);
                glEnd();
                glColor3f(0.2f, 0.2f, 0.25f);
                glBegin(GL_LINE_LOOP);
                glVertex3f(a.x, a.y, a.z);
                glVertex3f(b.x, b.y, b.z);
                glVertex3f(c.x, c.y, c.z);
                glVertex3f(d.x, d.y, d.z);
                glEnd();
            }
        }
    }

    void drawPropeller()
    {
        const Propeller& prop = mAircraft.propeller(0);
        const Propeller::Description& d = prop.description();
        const M::vec3 hub = mBody.pointToWorld(d.position);
        const M::vec3 axis = mBody.directionToWorld(d.axis);
        const M::vec3 side = mBody.directionToWorld(M::vec3(1.0f, 0.0f, 0.0f));
        const M::vec3 up = M::cross(axis, side);
        mBladeAngle += prop.angularSpeed() / 60.0f * 0.25f;
        glDisable(GL_LIGHTING);
        glColor3f(0.6f, 0.6f, 0.65f);
        glBegin(GL_LINE_LOOP);
        for (int i = 0; i < 32; ++i)
        {
            const f32 a = 2.0f * kPi * static_cast<f32>(i) / 32.0f;
            const M::vec3 rim = hub + (side * cosf(a) + up * sinf(a)) * d.radius;
            glVertex3f(rim.x, rim.y, rim.z);
        }
        glEnd();
        glColor3f(0.15f, 0.15f, 0.15f);
        glBegin(GL_LINES);
        for (u32 blade = 0; blade < d.bladeCount; ++blade)
        {
            const f32 a = mBladeAngle + 2.0f * kPi * static_cast<f32>(blade) / static_cast<f32>(d.bladeCount);
            const M::vec3 tip = hub + (side * cosf(a) + up * sinf(a)) * d.radius;
            glVertex3f(hub.x, hub.y, hub.z);
            glVertex3f(tip.x, tip.y, tip.z);
        }
        glEnd();
    }

    void drawGear()
    {
        glDisable(GL_LIGHTING);
        glColor3f(0.1f, 0.1f, 0.1f);
        for (u32 i = 0; i < mGear->wheelCount(); ++i)
        {
            const RaycastVehicle::Wheel& wheel = mGear->wheel(i);
            const M::mat4& m = wheel.worldTransform;
            const M::vec3 centre(m[3][0], m[3][1], m[3][2]);
            const M::vec3 up(m[1][0], m[1][1], m[1][2]);
            const M::vec3 forward(m[2][0], m[2][1], m[2][2]);
            glBegin(GL_LINE_LOOP);
            for (int seg = 0; seg < 16; ++seg)
            {
                const f32 a = 2.0f * kPi * static_cast<f32>(seg) / 16.0f;
                const M::vec3 rim = centre + (up * sinf(a) + forward * cosf(a)) * wheel.radius;
                glVertex3f(rim.x, rim.y, rim.z);
            }
            glEnd();
            glBegin(GL_LINES);
            glVertex3f(wheel.hardPointWorld.x, wheel.hardPointWorld.y, wheel.hardPointWorld.z);
            glVertex3f(centre.x, centre.y, centre.z);
            glEnd();
        }
    }

    void drawRunwayMarkings()
    {
        glDisable(GL_LIGHTING);
        glColor3f(0.95f, 0.95f, 0.95f);
        glBegin(GL_QUADS);
        for (f32 z = 0.0f; z < kRunwayLength - 100.0f; z += 50.0f)
        {
            glVertex3f(-0.4f, 0.01f, z);
            glVertex3f(0.4f, 0.01f, z);
            glVertex3f(0.4f, 0.01f, z + 25.0f);
            glVertex3f(-0.4f, 0.01f, z + 25.0f);
        }
        glEnd();
    }

    PhysicsWorld mWorld;
    Camera3D mCamera;
    RigidBody mRunway;
    BoxShape mRunwayShape{M::vec3(0.5f * kRunwayWidth, 1.0f, 0.5f * kRunwayLength)};
    RigidBody mBody;
    BoxShape mFuselageShape{M::vec3(0.5f)};
    Aircraft mAircraft;
    LightAircraftLayout mLayout;
    RaycastVehicle* mGear = nullptr;
    u32 mBodyId = 0;
    ct::Vector<RigidBody*> mBodies;
    ct::Vector<CollisionShape*> mShapes;

    f32 mElapsed = 0.0f;
    f32 mAirborneTime = 0.0f;
    f32 mLiftoffTime = 0.0f;
    f32 mGroundRoll = 0.0f;
    f32 mLiftoffSpeed = 0.0f;
    f32 mWorstLateral = 0.0f;
    f32 mBankTarget = 0.0f;
    f32 mBladeAngle = 0.0f;
    bool mAirborne = false;
    bool mFinite = true;
    bool mClimbJudged = false;
    bool mClimbing = false;
    char mClimbDetail[96] = {};

    f32 mThrottle = 0.0f;
    f32 mElevator = 0.0f;
    f32 mAileron = 0.0f;
    f32 mRudder = 0.0f;
    f32 mTrim = 0.0f;
    f32 mFlaps = 0.0f;
    int mFlapNotch = 0;
    bool mBrakes = false;
};

REGISTER_SCENE(Airplane3D, "3D demo: airplane", Category::Physics3D)

} // namespace testbed
