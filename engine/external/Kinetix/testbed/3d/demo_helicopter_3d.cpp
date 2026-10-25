#include "scene.h"

#include "render3d.h"

#include <collision/CollisionShape.h>
#include <dynamics/PhysicsWorld.h>
#include <dynamics/RigidBody.h>
#include <rotorcraft/Helicopter.h>
#include <rotorcraft/HelicopterAutopilot.h>

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

// Five 1/60 s steps per frame on the scripted run (the flight takes ~45 s simulated, the headless
// run gives each scene ten); by hand it runs one step per step.
constexpr int kStepsPerFrame = 5;

// Manual flying: lever and stick rates, and the cyclic/pedal travel the keys reach either side
// of the hover trim (full travel is more than anyone can fly from a keyboard).
constexpr f32 kCollectiveRate = 0.25f;
constexpr f32 kStickRate = 1.5f;
constexpr f32 kStickReturn = 2.5f;
constexpr f32 kCyclicAuthority = 0.35f;
constexpr f32 kPedalAuthority = 0.5f;
constexpr f32 kAssistSpeed = 8.0f;   // m/s over the ground
constexpr f32 kAssistClimb = 2.0f;   // m/s
constexpr f32 kAssistTurnRate = 0.5f; // rad/s of heading target

constexpr f32 kLiftTime = 1.0f;       // collective up, hover 5 m over the pad
constexpr f32 kHoverJudged = 7.0f;     // hover held from here ...
constexpr f32 kTransitionTime = 10.0f; // ... to here; then forward
constexpr f32 kReturnTime = 15.0f;     // turn back for the pad
constexpr f32 kLandingJudged = 48.0f;  // down and settled by here
constexpr f32 kHoverHeight = 5.0f;
constexpr f32 kCruiseHeight = 20.0f;
constexpr f32 kCruiseSpeed = 16.0f;
constexpr f32 kReturnSpeed = 14.0f;

const M::vec3 kGroundHalfExtents(200.0f, 1.0f, 200.0f);
constexpr f32 kPadHalfSize = 5.0f;
} // namespace

// The default light helicopter on a pad: lifts off, hovers, accelerates away, turns back and
// lands under the autopilot. Blade element theory on a polar grid with momentum inflow; rotor
// speed from the governor against rotor torque through the drive train. C restarts.
class Helicopter3D final : public Scene
{
public:
    Helicopter3D()
    {
        mWorld.setGravity(M::vec3(0.0f, -9.81f, 0.0f));
        mWorld.setFixedStep(1.0f / 60.0f);

        mGround.setBodyType(BodyType::Static);
        mGround.setPosition(M::vec3(0.0f, -kGroundHalfExtents.y, 0.0f));
        BodyEntry ground;
        ground.body = &mGround;
        ground.shape = &mGroundShape;
        ground.friction = 0.8f;
        mWorld.addBody(ground);
        mWorld.markStaticBroadphaseDirty();

        mLayout = buildLightHelicopter(mHelicopter, mBody);
        mHullShape = BoxShape(mLayout.hullHalfExtents);
        BodyEntry entry;
        entry.body = &mBody;
        entry.shape = &mHullShape;
        entry.friction = 0.5f;
        mBodyId = mWorld.addBody(entry);
        mHelicopter.setGround(&mWorld, mBodyId);

        // The hover trim, found once on a scratch copy well clear of the ground: the autopilot's
        // integrator start (on the skids the airframe sits level whatever a hover needs).
        {
            RigidBody body;
            Helicopter helicopter;
            buildLightHelicopter(helicopter, body);
            body.setPosition(M::vec3(0.0f, 300.0f, 0.0f));
            mHoverTrim = helicopter.trim(HelicopterTrimTarget());
        }

        mCamera.distance = 30.0f;
        mCamera.pitch = 0.3f;
        mCamera.yaw = 0.8f * kPi;

        addCheck("lifts off the pad");
        addCheck("hover holds within a metre");
        addCheck("transitions to forward flight");
        addCheck("returns and lands upright on the pad");
        addCheck("rotor speed governed");
        addCheck("stays finite");
        restart();
    }

    const char* name() const override
    {
        return "3D demo: helicopter";
    }
    const char* description() const override
    {
        return "A 1300 kg light turbine helicopter on a pad, flown by the autopilot helper:\n"
               "lift off, hover at 5 m, accelerate to 16 m/s, turn back and land on the pad.\n"
               "Rotor: blade elements on a polar grid, momentum inflow with tip loss,\n"
               "quasi-steady flapping (blades drawn coned and tilted), ground effect.\n"
               "The scripted run is shown at 5x speed. C restarts.\n"
               "Manual mode (P): fly it yourself in real time - see the controls panel.";
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
            seedFromControls();
            mHold = false;
            mEngineOn = mHelicopter.controls().engineOn;
            mSasOn = true;
            applySas();
        }
        else
            restart();
    }
    const char* controlsHelp() const override
    {
        return "W / S (hold)\tcollective up / down (the lever stays set)\n"
               "Up / Down\tcyclic forward / back\n"
               "Left / Right\tcyclic left / right\n"
               "Q / E\tpedals left / right\n"
               "I\tengine start / stop\n"
               "H\thover-hold assist on / off (then W/S climb, arrows fly, Q/E turn)\n"
               "K\tstability augmentation on / off (on by default)\n"
               "R or C\tback on the pad\n"
               "gamepad\tleft stick: cyclic, right stick x: pedals, triggers: collective";
    }

    void control(const ControlInput& input, float dt) override
    {
        if (input.hit(Key::R))
        {
            restart();
            return;
        }
        if (input.hit(Key::I))
            mEngineOn = !mEngineOn;
        if (input.hit(Key::K))
        {
            mSasOn = !mSasOn;
            applySas();
        }
        if (input.hit(Key::H))
        {
            mHold = !mHold;
            if (mHold)
            {
                mAutopilot = HelicopterAutopilot();
                HelicopterAutopilotGains gains;
                gains.maxSpeed = kAssistSpeed;
                mAutopilot.setGains(gains);
                mAutopilot.engage(mHelicopter);
                mHoldPoint = mBody.position();
                mHoldHeading = mHelicopter.telemetry().heading;
                mAssistMoving = false;
                mAssistClimbing = false;
            }
            else
                seedFromControls();
        }

        const f32 forward = input.pair(Key::Down, Key::Up) - input.axis(PadAxis::LeftY);
        const f32 right = input.pair(Key::Left, Key::Right) + input.axis(PadAxis::LeftX);
        const f32 pedal = input.pair(Key::Q, Key::E) + input.axis(PadAxis::RightX);
        const f32 lever = input.pair(Key::S, Key::W) + input.axis(PadAxis::RightTrigger) -
                          input.axis(PadAxis::LeftTrigger);
        if (mHold)
        {
            assist(forward, right, pedal, lever, dt);
            return;
        }
        mCollective = M::clamp(mCollective + lever * kCollectiveRate * dt, 0.0f, 1.0f);
        mLongitudinal = rampControl(mLongitudinal, M::clamp(forward, -1.0f, 1.0f), 0.0f, kStickRate, kStickReturn, dt);
        mLateral = rampControl(mLateral, M::clamp(right, -1.0f, 1.0f), 0.0f, kStickRate, kStickReturn, dt);
        mPedal = rampControl(mPedal, M::clamp(pedal, -1.0f, 1.0f), 0.0f, kStickRate, kStickReturn, dt);
    }

    void readouts(ct::Vector<Readout>& out) const override
    {
        const HelicopterTelemetry& t = mHelicopter.telemetry();
        const HelicopterControls& c = mHelicopter.controls();
        addReadout(out, "rotor", "%.0f rpm  %.1f%%", double(t.rotorRpm), double(t.rotorSpeedPercent));
        addReadout(out, "rotor torque", "%.1f kN m", double(t.mainRotorTorque / 1000.0f));
        addReadout(out, "engine", "%s, Ng %.0f%%, %.0f kW", c.engineOn ? "on" : "off",
                   double(t.gasGeneratorPercent), double(t.enginePower / 1000.0f));
        addReadout(out, "collective", "%.0f%%", double(c.collective * 100.0f));
        addReadout(out, "cyclic lon / lat", "%+.2f / %+.2f", double(c.cyclicLongitudinal), double(c.cyclicLateral));
        addReadout(out, "pedals", "%+.2f", double(c.pedals));
        addReadout(out, "height", "%.1f m", double(t.onGround ? 0.0f : t.heightAboveGround));
        addReadout(out, "vertical speed", "%+.2f m/s", double(t.verticalSpeed));
        addReadout(out, "airspeed / ground", "%.1f / %.1f m/s", double(t.trueAirspeed), double(t.groundSpeed));
        addReadout(out, "pitch / bank", "%.1f / %.1f deg", double(t.pitch / kDeg), double(t.bank / kDeg));
        addReadout(out, "heading", "%.0f deg", double(t.heading / kDeg));
        if (mManual)
            addReadout(out, "assist", "%s, SAS %s", mHold ? "hover hold" : "off", mSasOn ? "on" : "off");
        addReadout(out, "skids", "%s", t.onGround ? "on the ground" : "airborne");
        const M::vec3 p = mBody.position();
        addReadout(out, "position x / z", "%.1f / %.1f m", double(p.x), double(p.z));
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
        const int steps = mManual ? 1 : kStepsPerFrame;
        for (int i = 0; i < steps; ++i)
            simulate(dt);
        mCamera.target = mBody.position();
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
        drawGround(0.0f, 300.0f, 10.0f);
        drawPad();
        drawSolidBox(mBody.transform(), mLayout.hullHalfExtents, Shade{0.85f, 0.35f, 0.2f});
        M::mat4 boom = mBody.transform();
        boom[3] = M::vec4(mBody.pointToWorld(M::vec3(0.0f, 0.3f, -3.6f)), 1.0f);
        drawSolidBox(boom, M::vec3(0.15f, 0.18f, 2.1f), Shade{0.85f, 0.35f, 0.2f});
        drawSurfaces();
        drawSkids();
        drawRotor(mHelicopter.mainRotor(), mMainAngle, Shade{0.15f, 0.15f, 0.18f});
        drawRotor(mHelicopter.tailRotor(), mTailAngle, Shade{0.2f, 0.2f, 0.2f});
    }

    void evaluate() override
    {
        const HelicopterTelemetry& t = mHelicopter.telemetry();
        char buffer[160];

        if (mLiftedOff || mSim > kHoverJudged)
        {
            snprintf(buffer, sizeof(buffer), "off the skids at %.1f s", double(mLiftoffTime));
            report(0, mLiftedOff, buffer);
        }
        if (mSim > kTransitionTime)
        {
            snprintf(buffer, sizeof(buffer), "worst %.2f m from the hover point", double(mWorstHover));
            report(1, mWorstHover < 1.0f, buffer);
        }
        if (mSim > kReturnTime)
        {
            snprintf(buffer, sizeof(buffer), "%.1f m/s reached", double(mFastest));
            report(2, mFastest > 0.8f * kCruiseSpeed, buffer);
        }
        if (mLanded || mSim > kLandingJudged)
        {
            const f32 offset = M::length(M::vec3(mBody.position().x, 0.0f, mBody.position().z));
            const bool upright = fabsf(t.pitch) < 3.0f * kDeg && fabsf(t.bank) < 3.0f * kDeg;
            snprintf(buffer, sizeof(buffer), "down at %.1f s, %.2f m off the centre, pitch %.1f bank %.1f deg",
                     double(mLandTime), double(offset), double(t.pitch / kDeg), double(t.bank / kDeg));
            report(3, mLanded && upright && offset < kPadHalfSize, buffer);
        }
        snprintf(buffer, sizeof(buffer), "%.1f%% to %.1f%%", double(mLowestRotor), double(mHighestRotor));
        report(4, mLowestRotor > 90.0f && mHighestRotor < 106.0f, buffer);
        report(5, mFinite);
    }

private:
    void restart()
    {
        mBody.setPosition(M::vec3(0.0f, mLayout.restHeight, 0.0f));
        mBody.setOrientation(M::quat());
        mBody.setVelocity(M::vec3(0.0f));
        mBody.setAngularVelocity(M::vec3(0.0f));
        mBody.clearAccumulators();
        HelicopterControls idle;
        idle.collective = 0.0f;
        mHelicopter.setControls(idle);
        mHelicopter.startRunning();
        mAutopilot = HelicopterAutopilot();
        HelicopterAutopilotGains gains;
        gains.maxSpeed = kReturnSpeed;
        gains.maxYawRate = 0.6f;
        mAutopilot.setGains(gains);
        mEngaged = false;
        mSim = 0.0f;
        mLiftedOff = false;
        mLiftoffTime = 0.0f;
        mWorstHover = 0.0f;
        mFastest = 0.0f;
        mDescending = false;
        mLanded = false;
        mLandTime = 0.0f;
        mLowestRotor = 1.0e9f;
        mHighestRotor = 0.0f;
        mFinite = true;
        resetChecks();
        mCollective = 0.0f;
        mLongitudinal = mLateral = mPedal = 0.0f;
        mHold = false;
        mEngineOn = true;
        mSasOn = true;
        applySas();
    }

    // Rate damping is the pilot's: on in manual mode unless switched off, never on the scripted run.
    void applySas()
    {
        StabilityAugmentation sas = mHelicopter.stabilityAugmentation();
        sas.enabled = mManual && mSasOn;
        mHelicopter.setStabilityAugmentation(sas);
    }

    void seedFromControls()
    {
        const HelicopterControls& c = mHelicopter.controls();
        mCollective = c.collective;
        mLongitudinal = M::clamp((c.cyclicLongitudinal - mHoverTrim.cyclicLongitudinal) / kCyclicAuthority, -1.0f, 1.0f);
        mLateral = M::clamp((c.cyclicLateral - mHoverTrim.cyclicLateral) / kCyclicAuthority, -1.0f, 1.0f);
        mPedal = M::clamp((c.pedals - mHoverTrim.pedals) / kPedalAuthority, -1.0f, 1.0f);
    }

    // Hover hold: arrows fly over the ground in the helicopter's frame, W/S climb, Q/E turn;
    // released, each holds where it is.
    void assist(f32 forward, f32 right, f32 pedal, f32 lever, f32 dt)
    {
        mHoldHeading += M::clamp(pedal, -1.0f, 1.0f) * kAssistTurnRate * dt;
        const M::vec3 p = mBody.position();
        if (lever != 0.0f)
        {
            mAutopilot.holdVerticalSpeed(M::clamp(lever, -1.0f, 1.0f) * kAssistClimb);
            mAssistClimbing = true;
        }
        else
        {
            if (mAssistClimbing)
                mHoldPoint.y = p.y;
            mAssistClimbing = false;
            mAutopilot.holdAltitude(mHoldPoint.y);
        }
        if (forward != 0.0f || right != 0.0f)
        {
            const M::vec3 ahead(-sinf(mHoldHeading), 0.0f, cosf(mHoldHeading));
            const M::vec3 side = M::cross(ahead, M::vec3(0.0f, 1.0f, 0.0f));
            mAutopilot.holdVelocity((ahead * M::clamp(forward, -1.0f, 1.0f) + side * M::clamp(right, -1.0f, 1.0f)) *
                                        kAssistSpeed,
                                    mHoldHeading);
            mAssistMoving = true;
        }
        else if (mAssistMoving)
        {
            // Brake to a stop first, then hold: holding where the key came up would overshoot.
            mAutopilot.holdVelocity(M::vec3(0.0f), mHoldHeading);
            if (mHelicopter.telemetry().groundSpeed < 1.5f)
            {
                const M::vec3 v = mBody.velocity();
                mAssistMoving = false;
                mHoldPoint.x = p.x + 0.7f * v.x;
                mHoldPoint.z = p.z + 0.7f * v.z;
            }
        }
        else
            mAutopilot.holdPosition(mHoldPoint, mHoldHeading);
    }

    void fly(f32 dt)
    {
        HelicopterControls c;
        if (mHold)
            c = mAutopilot.update(mHelicopter, dt);
        else
        {
            c.collective = mCollective;
            c.cyclicLongitudinal = mHoverTrim.cyclicLongitudinal + mLongitudinal * kCyclicAuthority;
            c.cyclicLateral = mHoverTrim.cyclicLateral + mLateral * kCyclicAuthority;
            c.pedals = mHoverTrim.pedals + mPedal * kPedalAuthority;
        }
        c.engineOn = mEngineOn;
        mHelicopter.setControls(c);
    }

    void plan()
    {
        const f32 hoverY = mLayout.restHeight + kHoverHeight;
        if (mSim < kLiftTime)
            return;
        if (!mEngaged)
        {
            mAutopilot.engage(mHoverTrim, 0.0f);
            mEngaged = true;
        }
        if (mSim < kTransitionTime)
        {
            mAutopilot.holdPosition(M::vec3(0.0f, hoverY, 0.0f), 0.0f);
        }
        else if (mSim < kReturnTime)
        {
            mAutopilot.holdVelocity(M::vec3(0.0f, 0.0f, kCruiseSpeed), 0.0f);
            mAutopilot.holdAltitude(mLayout.restHeight + kCruiseHeight);
        }
        else if (!mDescending)
        {
            mAutopilot.holdAltitude(hoverY);
            mAutopilot.holdPosition(M::vec3(0.0f, hoverY, 0.0f), kPi);
            const M::vec3 p = mBody.position();
            const f32 offset = M::length(M::vec3(p.x, 0.0f, p.z));
            if (offset < 1.5f && M::length(mBody.velocity()) < 1.0f && fabsf(p.y - hoverY) < 1.0f)
            {
                mDescending = true;
                mAutopilot.holdVerticalSpeed(-1.5f);
            }
        }
        else if (mHelicopter.telemetry().onGround)
        {
            mAutopilot.holdVerticalSpeed(-3.0f);
        }
        if (mLanded)
        {
            HelicopterControls idle;
            idle.collective = 0.0f;
            mHelicopter.setControls(idle);
            return;
        }
        mHelicopter.setControls(mAutopilot.update(mHelicopter, 1.0f / 60.0f));
    }

    void simulate(f32 dt)
    {
        if (mManual)
            fly(dt);
        else
            plan();
        mHelicopter.applyForces(dt);
        mWorld.step(dt);
        mSim += dt;

        const HelicopterTelemetry& t = mHelicopter.telemetry();
        const M::vec3 p = mBody.position();
        mFinite = mFinite && isfinite(p.x) && isfinite(p.y) && isfinite(p.z) &&
                  isfinite(mHelicopter.transmission().rotorSpeed());
        if (!mLiftedOff && !t.onGround && mSim > kLiftTime)
        {
            mLiftedOff = true;
            mLiftoffTime = mSim;
        }
        if (mSim > kHoverJudged && mSim < kTransitionTime)
        {
            const M::vec3 hover(0.0f, mLayout.restHeight + kHoverHeight, 0.0f);
            mWorstHover = fmaxf(mWorstHover, M::length(p - hover));
        }
        mFastest = fmaxf(mFastest, t.trueAirspeed);
        if (mLiftedOff)
        {
            mLowestRotor = fminf(mLowestRotor, t.rotorSpeedPercent);
            mHighestRotor = fmaxf(mHighestRotor, t.rotorSpeedPercent);
        }
        if (mDescending && !mLanded && t.onGround && M::length(mBody.velocity()) < 0.05f)
        {
            bool all = true;
            for (u32 i = 0; i < mHelicopter.skidContactCount(); ++i)
                all = all && mHelicopter.skidContactState(i).inContact;
            if (all)
            {
                mLanded = true;
                mLandTime = mSim;
            }
        }

        // The drawn blades turn at a fraction of the true speed, which would only strobe.
        mMainAngle += mHelicopter.transmission().rotorSpeed() * dt * 0.08f;
        mTailAngle += mHelicopter.transmission().tailRotorSpeed() * dt * 0.02f;
    }

    void drawPad()
    {
        glDisable(GL_LIGHTING);
        glColor3f(0.35f, 0.35f, 0.38f);
        glBegin(GL_QUADS);
        glVertex3f(-kPadHalfSize, 0.01f, -kPadHalfSize);
        glVertex3f(kPadHalfSize, 0.01f, -kPadHalfSize);
        glVertex3f(kPadHalfSize, 0.01f, kPadHalfSize);
        glVertex3f(-kPadHalfSize, 0.01f, kPadHalfSize);
        glEnd();
        glColor3f(0.95f, 0.95f, 0.95f);
        glBegin(GL_QUADS);
        const f32 bars[3][4] = {{-2.0f, -1.5f, -2.5f, 2.5f}, {1.5f, 2.0f, -2.5f, 2.5f}, {-1.5f, 1.5f, -0.25f, 0.25f}};
        for (const auto& bar : bars)
        {
            glVertex3f(bar[0], 0.02f, bar[2]);
            glVertex3f(bar[1], 0.02f, bar[2]);
            glVertex3f(bar[1], 0.02f, bar[3]);
            glVertex3f(bar[0], 0.02f, bar[3]);
        }
        glEnd();
    }

    // beta(psi) = a0 - a1s cos(psi) - b1s sin(psi), and the tip-path circle.
    void drawRotor(const Rotor& rotor, f32 angle, Shade shade)
    {
        const Rotor::Description& d = rotor.description();
        const RotorLoads& l = rotor.loads();
        const M::vec3 hub = mBody.pointToWorld(d.position);
        const M::vec3 shaft = mBody.directionToWorld(rotor.shaftAxis());
        const M::vec3 aft = mBody.directionToWorld(rotor.aftAxis());
        const M::vec3 advancing = mBody.directionToWorld(rotor.advancingAxis());
        auto tip = [&](f32 psi)
        {
            const f32 beta = l.coning - l.longitudinalFlapping * cosf(psi) - l.lateralFlapping * sinf(psi);
            const M::vec3 radial = aft * cosf(psi) + advancing * sinf(psi);
            return hub + (radial * cosf(beta) + shaft * sinf(beta)) * d.radius;
        };
        glDisable(GL_LIGHTING);
        glColor3f(0.6f, 0.6f, 0.65f);
        glBegin(GL_LINE_LOOP);
        for (int i = 0; i < 48; ++i)
        {
            const M::vec3 p = tip(2.0f * kPi * static_cast<f32>(i) / 48.0f);
            glVertex3f(p.x, p.y, p.z);
        }
        glEnd();
        glLineWidth(3.0f);
        glColor3f(shade.r, shade.g, shade.b);
        glBegin(GL_LINES);
        for (u32 b = 0; b < d.bladeCount; ++b)
        {
            const f32 psi = angle + 2.0f * kPi * static_cast<f32>(b) / static_cast<f32>(d.bladeCount);
            const M::vec3 p = tip(psi);
            glVertex3f(hub.x, hub.y, hub.z);
            glVertex3f(p.x, p.y, p.z);
        }
        glEnd();
        glLineWidth(1.0f);
    }

    void drawSkids()
    {
        glDisable(GL_LIGHTING);
        glLineWidth(2.0f);
        glColor3f(0.2f, 0.2f, 0.2f);
        glBegin(GL_LINES);
        for (u32 side = 0; side < 2; ++side)
        {
            const M::vec3 front = mBody.pointToWorld(mLayout.skidPoints[2 * side]);
            const M::vec3 rear = mBody.pointToWorld(mLayout.skidPoints[2 * side + 1]);
            glVertex3f(front.x, front.y, front.z);
            glVertex3f(rear.x, rear.y, rear.z);
            for (const M::vec3& local : {mLayout.skidPoints[2 * side], mLayout.skidPoints[2 * side + 1]})
            {
                const M::vec3 foot = mBody.pointToWorld(local * M::vec3(1.0f, 1.0f, 0.6f));
                const M::vec3 floor = mBody.pointToWorld(M::vec3(local.x * 0.5f, -mLayout.hullHalfExtents.y, local.z * 0.6f));
                glVertex3f(foot.x, foot.y, foot.z);
                glVertex3f(floor.x, floor.y, floor.z);
            }
        }
        glEnd();
        glLineWidth(1.0f);
    }

    void drawSurfaces()
    {
        glDisable(GL_LIGHTING);
        for (u32 w = 0; w < mHelicopter.wingCount(); ++w)
        {
            const Wing& wing = mHelicopter.wing(w);
            for (u32 e = 0; e < wing.elementCount(); ++e)
            {
                const Wing::Element& element = wing.element(e);
                const M::vec3 centre = mBody.pointToWorld(element.position);
                const M::vec3 chord = mBody.directionToWorld(element.chordAxis) * element.chord;
                const M::vec3 span = mBody.directionToWorld(element.pitchAxis) * (0.5f * element.area / element.chord);
                const M::vec3 leading = centre + chord * 0.25f;
                const M::vec3 trailing = centre - chord * 0.75f;
                const f32 s = element.separation;
                glColor3f(0.9f, 0.9f - 0.6f * s, 0.85f - 0.6f * s);
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
            }
        }
    }

    PhysicsWorld mWorld;
    Camera3D mCamera;
    RigidBody mGround;
    BoxShape mGroundShape{kGroundHalfExtents};
    RigidBody mBody;
    BoxShape mHullShape{M::vec3(0.5f)};
    Helicopter mHelicopter;
    LightHelicopterLayout mLayout;
    HelicopterAutopilot mAutopilot;
    HelicopterTrimResult mHoverTrim;
    u32 mBodyId = 0;

    f32 mSim = 0.0f;
    bool mEngaged = false;
    bool mLiftedOff = false;
    f32 mLiftoffTime = 0.0f;
    f32 mWorstHover = 0.0f;
    f32 mFastest = 0.0f;
    bool mDescending = false;
    bool mLanded = false;
    f32 mLandTime = 0.0f;
    f32 mLowestRotor = 1.0e9f;
    f32 mHighestRotor = 0.0f;
    bool mFinite = true;
    f32 mMainAngle = 0.0f;
    f32 mTailAngle = 0.0f;

    f32 mCollective = 0.0f;
    f32 mLongitudinal = 0.0f;
    f32 mLateral = 0.0f;
    f32 mPedal = 0.0f;
    bool mEngineOn = true;
    bool mSasOn = true;
    bool mHold = false;
    M::vec3 mHoldPoint{0.0f};
    f32 mHoldHeading = 0.0f;
    bool mAssistMoving = false;
    bool mAssistClimbing = false;
};

REGISTER_SCENE(Helicopter3D, "3D demo: helicopter", Category::Physics3D)

} // namespace testbed
