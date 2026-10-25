#include "scene.h"

#include "render3d.h"

#include <actuators/Hook.h>
#include <actuators/HydraulicCylinder.h>
#include <actuators/Rope.h>
#include <collision/CollisionShape.h>
#include <dynamics/FixedJoint.h>
#include <dynamics/HingeJoint.h>
#include <dynamics/PhysicsWorld.h>
#include <dynamics/RigidBody.h>

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
// The digger: boom, stick and an L-shaped bucket on a pedestal, three hydraulic cylinders between
// pins on the links. Lengths along each link, metres.
constexpr f32 kBoomLength = 3.0f;
constexpr f32 kStickLength = 2.2f;
const M::vec3 kBoomPivot(0.0f, 1.5f, 0.0f);
// The bucket hangs from its pivot: wall straight down, floor running back towards the machine.
const M::vec3 kWallHalf(0.03f, 0.35f, 0.12f);
const M::vec3 kFloorHalf(0.35f, 0.025f, 0.12f);
const M::vec3 kFloorOffset(-0.38f, -0.675f, 0.0f);
const M::vec3 kBoxCentre(3.0f, 0.5f, 0.0f);
const M::vec3 kBoxHalf(0.25f);
const M::vec3 kSleeperHalf(0.25f, 0.125f, 0.05f);

// Bucket pivot position and bucket angle over the script.
struct Waypoint
{
    f32 time;
    f32 x;
    f32 y;
    f32 curl;
};
const Waypoint kScript[] = {
    {0.0f, 4.08f, 1.395f, 0.0f}, {1.5f, 4.08f, 0.795f, 0.0f}, {4.5f, 3.38f, 0.795f, 0.0f},
    {6.0f, 3.38f, 0.795f, -0.6f}, {8.0f, 3.38f, 1.795f, -0.6f}, {10.0f, 3.38f, 1.795f, -0.6f}};
constexpr u32 kWaypoints = sizeof(kScript) / sizeof(kScript[0]);

const M::vec3 kWinchPoint(-4.0f, 0.5f, 0.0f);
const M::vec3 kSheavePoint(-6.0f, 7.0f, 0.0f);
const M::vec3 kHookStart(-6.0f, 4.0f, 0.0f);
const M::vec3 kHookHalf(0.15f, 0.2f, 0.15f);
const M::vec3 kLoadCentre(-6.0f, 0.3f, 0.0f);
const M::vec3 kLoadHalf(0.4f, 0.3f, 0.3f);
constexpr f32 kHoistHeight = 2.0f;
constexpr f32 kReleaseTime = 7.5f;
} // namespace

class Actuators3D final : public Scene
{
public:
    Actuators3D()
    {
        mWorld.setGravity(M::vec3(0.0f, -9.81f, 0.0f));
        mWorld.setFixedStep(1.0f / 60.0f);
        buildGround();
        buildDigger();
        buildCrane();
        mWorld.markStaticBroadphaseDirty();

        mCamera.target = M::vec3(-1.0f, 2.0f, 0.0f);
        mCamera.distance = 17.0f;
        mCamera.pitch = 0.3f;
        mCamera.yaw = 0.15f;

        addCheck("digger arm follows its poses");
        addCheck("bucket scoops the box and lifts it");
        addCheck("crane hooks the load and hoists it");
        addCheck("load released and back on the ground");
        addCheck("everything finite, joints holding");
    }

    ~Actuators3D() override
    {
        delete mHook;
        for (Joint* joint : mJoints)
        {
            mWorld.removeJoint(joint);
            delete joint;
        }
        mWorld.clear();
        for (RigidBody* body : mOwnedBodies)
            delete body;
        for (CollisionShape* shape : mOwnedShapes)
            delete shape;
    }

    const char* name() const override
    {
        return "3D demo: actuators";
    }
    const char* description() const override
    {
        return "Right: a digger arm on a pedestal - boom, stick and an L-shaped bucket on three hinges, "
               "each turned by a hydraulic cylinder pinned between the two links (orange: barrel, grey: "
               "rod). The operator turns the bucket's path into joint angles and opens each valve for "
               "the stroke speed that follows them; the pump flow over the piston area sets the "
               "speed, the relief pressure the force. The bucket "
               "goes down, slides its floor under a box on two sleepers, curls and lifts it.\n"
               "Left: a winch on the ground, its rope over a mast-top sheave and down two falls to a "
               "hook block. It pays out to the load, hooks it by capture radius, hoists it 2 m, holds "
               "it on the brake and lets it go.\n"
               "Manual mode (P): open the valves and run the winch yourself - see the controls panel.";
    }

    bool drivable() const override
    {
        return true;
    }
    const char* controlsHelp() const override
    {
        return "Up / Down\tdigger: boom cylinder out / in\n"
               "I / K\tdigger: stick cylinder out / in\n"
               "J / L\tdigger: bucket cylinder out / in\n"
               "W / S\tcrane: winch hoist / lower (let go: on the brake)\n"
               "G\tcrane: hook the nearest load / let it go\n"
               "R\tstart over";
    }

    void control(const ControlInput& input, float dt) override
    {
        if (input.hit(Key::R))
        {
            restartRequested = true;
            return;
        }
        const f32 commands[3] = {input.pair(Key::Down, Key::Up), input.pair(Key::K, Key::I), input.pair(Key::L, Key::J)};
        for (u32 i = 0; i < 3; ++i)
            mValves[i] = rampControl(mValves[i], commands[i], 0.0f, 4.0f, 6.0f, dt);
        mWinchCommand = rampControl(mWinchCommand, input.pair(Key::S, Key::W), 0.0f, 3.0f, 6.0f, dt);
        if (input.hit(Key::G))
        {
            if (mHook->attached())
                mHook->detach();
            else
                mHook->attachNearest(0.3f, HookMode::Point);
        }
    }

    void readouts(ct::Vector<Readout>& out) const override
    {
        const char* names[3] = {"boom cylinder", "stick cylinder", "bucket cylinder"};
        for (u32 i = 0; i < 3; ++i)
        {
            const HydraulicCylinder& c = *mCylinders[i];
            addReadout(out, names[i], "valve %+.2f, %.2f m stroke, %.0f bar", double(c.valve()), double(c.stroke()),
                       double(c.capPressure() / 1.0e5f));
        }
        addReadout(out, "joint angles", "%.0f / %.0f / %.0f deg", double(mHinges[0]->currentAngle() * 57.29578f),
                   double(mHinges[1]->currentAngle() * 57.29578f), double(mHinges[2]->currentAngle() * 57.29578f));
        addReadout(out, "box", "%.2f m up", double(mBox->position().y));
        addReadout(out, "winch", "command %+.2f, line %+.2f m/s", double(mWinch.command()),
                   double(mWinch.lineSpeed()));
        addReadout(out, "hook", "%s, rope load %.0f N", mHook->attached() ? "loaded" : "free",
                   double(mRope->hookLoad()));
        addReadout(out, "load", "%.2f m up", double(mLoad->position().y));
    }

    bool is3D() const override
    {
        return true;
    }
    Camera3D camera3D() const override
    {
        return mCamera;
    }

    void step(float dt) override
    {
        mElapsed += dt;
        if (mManual)
        {
            for (u32 i = 0; i < 3; ++i)
                mCylinders[i]->setValve(mValves[i]);
            mWinch.setCommand(mWinchCommand);
        }
        else
        {
            operateDigger(dt);
            operateCrane();
        }
        mWorld.step(dt);
        observe();
    }

    void draw() override
    {
        setupLighting3D();
        drawWorldSolid(mWorld, mBodies, mShapes);
        drawCylinders();
        drawRope();
    }

    void evaluate() override
    {
        char buffer[160];
        if (mElapsed >= 9.0f && !mPosesJudged)
        {
            mPosesJudged = true;
            snprintf(mPosesDetail, sizeof(mPosesDetail), "bucket pivot within %.3f m of the script, bucket within %.3f rad",
                     double(mWorstTracking), double(mWorstCurl));
        }
        if (mPosesJudged)
            report(0, mWorstTracking < 0.05f && mWorstCurl < 0.05f, mPosesDetail);

        if (mElapsed >= 9.5f && !mScoopJudged)
        {
            mScoopJudged = true;
            const M::vec3 floor = mFloor->position();
            mScoopOk = mBox->position().y > 1.0f && M::length(mBox->position() - floor) < 0.6f;
            snprintf(mScoopDetail, sizeof(mScoopDetail), "box at %.2f m, %.2f m from the bucket floor",
                     double(mBox->position().y), double(M::length(mBox->position() - floor)));
        }
        if (mScoopJudged)
            report(1, mScoopOk, mScoopDetail);

        if (mElapsed >= kReleaseTime - 0.1f && !mHoistJudged)
        {
            mHoistJudged = true;
            mHoistOk = mHookedAt > 0.0f && mHighestLoad - kLoadCentre.y > 0.85f * kHoistHeight;
            snprintf(mHoistDetail, sizeof(mHoistDetail), "hooked at %.2f s, lifted %.2f m, hook load %.0f N",
                     double(mHookedAt), double(mHighestLoad - kLoadCentre.y), double(mRope->hookLoad()));
        }
        if (mHoistJudged)
            report(2, mHoistOk, mHoistDetail);

        if (mElapsed >= 9.5f && !mReleaseJudged)
        {
            mReleaseJudged = true;
            mReleaseOk = !mHook->attached() && mLoad->position().y < kLoadCentre.y + 0.05f;
            snprintf(mReleaseDetail, sizeof(mReleaseDetail), "load at %.2f m after the release",
                     double(mLoad->position().y));
        }
        if (mReleaseJudged)
            report(3, mReleaseOk, mReleaseDetail);

        snprintf(buffer, sizeof(buffer), "worst hinge opening %.1f mm, worst rope stretch %.1f mm",
                 double(mWorstSeparation * 1000.0f), double(mWorstStretch * 1000.0f));
        report(4, mFinite && mWorstSeparation < 0.03f && mWorstStretch < 0.01f, buffer);
    }

private:
    RigidBody& addBody(const M::vec3& centre, const M::vec3& halfExtents, f32 mass, bool collide,
                       u32* idOut = nullptr)
    {
        RigidBody* body = new RigidBody();
        if (mass > 0.0f)
        {
            body->setBodyType(BodyType::Dynamic);
            body->setMass(mass);
            body->setInertiaTensor(Inertia::box(mass, halfExtents));
        }
        else
            body->setBodyType(BodyType::Static);
        body->setPosition(centre);
        body->calculateDerivedData();
        BoxShape* shape = new BoxShape(halfExtents);
        BodyEntry entry;
        entry.body = body;
        entry.shape = shape;
        entry.friction = 0.8f;
        if (!collide)
        {
            entry.filter.group = 1u << 30;
            entry.filter.mask = 0u;
        }
        const u32 id = mWorld.addBody(entry);
        if (idOut)
            *idOut = id;
        mOwnedBodies.push_back(body);
        mOwnedShapes.push_back(shape);
        mBodies.push_back(body);
        mShapes.push_back(shape);
        return *body;
    }

    void addJoint(Joint* joint)
    {
        mWorld.addJoint(joint);
        mJoints.push_back(joint);
    }

    void buildGround()
    {
        addBody(M::vec3(0.0f, -0.5f, 0.0f), M::vec3(14.0f, 0.5f, 6.0f), 0.0f, true);
    }

    // Two-link inverse kinematics in the arm's plane, elbow up: absolute boom and stick angles.
    static void solveArm(f32 x, f32 y, f32& boom, f32& stick)
    {
        const M::vec3 target(x - kBoomPivot.x, y - kBoomPivot.y, 0.0f);
        const f32 distance = M::length(target);
        const f32 cosine =
            M::clamp((kBoomLength * kBoomLength + distance * distance - kStickLength * kStickLength) /
                         (2.0f * kBoomLength * distance),
                     -1.0f, 1.0f);
        boom = atan2f(target.y, target.x) + acosf(cosine);
        const M::vec3 elbow(kBoomLength * cosf(boom), kBoomLength * sinf(boom), 0.0f);
        stick = atan2f(target.y - elbow.y, target.x - elbow.x);
    }

    void buildDigger()
    {
        const M::vec3 z(0.0f, 0.0f, 1.0f);
        RigidBody& pedestal = addBody(M::vec3(0.0f, 0.75f, 0.0f), M::vec3(0.4f, 0.75f, 0.4f), 0.0f, false);
        addBody(M::vec3(kBoxCentre.x, kSleeperHalf.y, 0.2f), kSleeperHalf, 0.0f, true);
        addBody(M::vec3(kBoxCentre.x, kSleeperHalf.y, -0.2f), kSleeperHalf, 0.0f, true);
        mBox = &addBody(kBoxCentre, kBoxHalf, 40.0f, true);

        solveArm(kScript[0].x, kScript[0].y, mRestAngles[0], mRestAngles[1]);
        mRestAngles[2] = 0.0f;
        const M::vec3 boomDirection(cosf(mRestAngles[0]), sinf(mRestAngles[0]), 0.0f);
        const M::vec3 stickPivot = kBoomPivot + boomDirection * kBoomLength;
        const M::vec3 stickDirection(cosf(mRestAngles[1]), sinf(mRestAngles[1]), 0.0f);
        const M::vec3 bucketPivot = stickPivot + stickDirection * kStickLength;

        RigidBody& boom = addBody(kBoomPivot + boomDirection * (0.5f * kBoomLength),
                                  M::vec3(0.5f * kBoomLength, 0.14f, 0.14f), 600.0f, false);
        boom.setOrientation(M::angleAxis(mRestAngles[0], z));
        boom.calculateDerivedData();
        RigidBody& stick = addBody(stickPivot + stickDirection * (0.5f * kStickLength),
                                   M::vec3(0.5f * kStickLength, 0.1f, 0.1f), 300.0f, false);
        stick.setOrientation(M::angleAxis(mRestAngles[1], z));
        stick.calculateDerivedData();
        u32 wallId = 0;
        u32 floorId = 0;
        RigidBody& wall =
            addBody(bucketPivot + M::vec3(0.0f, -kWallHalf.y, 0.0f), kWallHalf, 60.0f, true, &wallId);
        mFloor = &addBody(bucketPivot + kFloorOffset, kFloorHalf, 40.0f, true, &floorId);
        addJoint(new FixedJoint(wall, *mFloor, bucketPivot + M::vec3(-0.03f, -0.675f, 0.0f)));
        mWorld.setPairIgnored(wallId, floorId, true);

        mLinks[0] = &boom;
        mLinks[1] = &stick;
        mLinks[2] = &wall;
        RigidBody* parents[3] = {&pedestal, &boom, &stick};
        const M::vec3 pivots[3] = {kBoomPivot, stickPivot, bucketPivot};
        for (u32 i = 0; i < 3; ++i)
        {
            mHinges[i] = new HingeJoint(*parents[i], *mLinks[i], pivots[i], z);
            addJoint(mHinges[i]);
            mParents[i] = parents[i];
            mLocalParent[i] = parents[i]->pointToLocal(pivots[i]);
            mLocalChild[i] = mLinks[i]->pointToLocal(pivots[i]);
        }

        // Cylinder pins: boom's from the pedestal ahead of its pivot to the boom's underside; stick's
        // from the boom top to a lever behind the stick pivot; bucket's along the stick to a lever square to it.
        const M::vec3 boomNormal = M::cross(z, boomDirection);
        const M::vec3 stickNormal = M::cross(z, stickDirection);
        const M::vec3 barrels[3] = {M::vec3(0.6f, 0.4f, 0.0f),
                                    kBoomPivot + boomDirection * 1.4f + boomNormal * 0.45f,
                                    stickPivot + stickDirection * 0.3f + stickNormal * 0.3f};
        const M::vec3 rods[3] = {kBoomPivot + boomDirection * 1.1f - boomNormal * 0.18f,
                                 stickPivot - stickDirection * 0.45f + stickNormal * 0.1f,
                                 bucketPivot + stickNormal * 0.3f};
        RigidBody* barrelBodies[3] = {&pedestal, &boom, &stick};
        const f32 bores[3] = {0.14f, 0.12f, 0.09f};
        for (u32 i = 0; i < 3; ++i)
        {
            HydraulicCylinderParameters p;
            p.boreDiameter = bores[i];
            p.rodDiameter = 0.6f * bores[i];
            p.stroke = 1.4f;
            p.initialStroke = 0.7f;
            p.pumpFlow = 150.0f;
            p.reliefPressure = 30.0e6f;
            mCylinders[i] = new HydraulicCylinder(*barrelBodies[i], barrels[i], *mLinks[i], rods[i], p);
            addJoint(mCylinders[i]);
            mBarrelLocal[i] = barrelBodies[i]->pointToLocal(barrels[i]);
            mRodLocal[i] = mLinks[i]->pointToLocal(rods[i]);
        }
    }

    void buildCrane()
    {
        // The mast is drawn, not simulated.
        mWinchHouse = &addBody(kWinchPoint, M::vec3(0.4f, 0.4f, 0.4f), 0.0f, false);
        mMastTop = &addBody(kSheavePoint + M::vec3(0.0f, 0.2f, 0.0f), M::vec3(0.2f), 0.0f, false);
        addBody(M::vec3(kSheavePoint.x - 0.5f, 0.5f * kSheavePoint.y, 0.0f),
                M::vec3(0.15f, 0.5f * kSheavePoint.y, 0.15f), 0.0f, false);
        mHookBlock = &addBody(kHookStart, kHookHalf, 30.0f, true, &mHookBlockId);
        mLoad = &addBody(kLoadCentre, kLoadHalf, 250.0f, true, &mLoadId);

        WinchParameters p;
        p.drumRadius = 0.25f;
        p.maxLinePull = 6.0e3f;
        p.maxLineSpeed = 2.0f;
        mWinch = Winch(p);
        mRope = new Rope(*mWinchHouse, kWinchPoint, *mHookBlock, kHookStart + M::vec3(0.0f, kHookHalf.y, 0.0f));
        mRope->addSheave(*mMastTop, kSheavePoint);
        mRope->setFalls(2);
        mRope->setWinch(&mWinch);
        addJoint(mRope);
        mHook = new Hook(mWorld, mHookBlockId, kHookStart - M::vec3(0.0f, kHookHalf.y, 0.0f));
    }

    // The digger's operator: script pivot through inverse kinematics into hinge angles, each valve
    // opened for the stroke speed that gives the target joint rate, plus a proportional correction.
    void operateDigger(f32 dt)
    {
        u32 segment = 0;
        while (segment + 2 < kWaypoints && mElapsed >= kScript[segment + 1].time)
            ++segment;
        const Waypoint& a = kScript[segment];
        const Waypoint& b = kScript[segment + 1];
        const f32 t = M::clamp((mElapsed - a.time) / (b.time - a.time), 0.0f, 1.0f);
        mTargetX = a.x + (b.x - a.x) * t;
        mTargetY = a.y + (b.y - a.y) * t;
        const f32 curl = a.curl + (b.curl - a.curl) * t;
        mTargetCurl = curl;
        f32 boom = 0.0f;
        f32 stick = 0.0f;
        solveArm(mTargetX, mTargetY, boom, stick);
        const f32 absolute[3] = {boom, stick, curl};
        f32 targets[3];
        targets[0] = absolute[0] - mRestAngles[0];
        targets[1] = (absolute[1] - absolute[0]) - (mRestAngles[1] - mRestAngles[0]);
        targets[2] = (absolute[2] - absolute[1]) - (mRestAngles[2] - mRestAngles[1]);
        const M::vec3 z(0.0f, 0.0f, 1.0f);
        for (u32 i = 0; i < 3; ++i)
        {
            const f32 rate = mStarted && dt > 0.0f ? (targets[i] - mPreviousTargets[i]) / dt : 0.0f;
            mPreviousTargets[i] = targets[i];
            const M::vec3 barrel = mParents[i]->pointToWorld(mBarrelLocal[i]);
            const M::vec3 rod = mLinks[i]->pointToWorld(mRodLocal[i]);
            const M::vec3 pivot = mLinks[i]->pointToWorld(mLocalChild[i]);
            // Stroke per radian: the moment arm of the cylinder's line about the pivot.
            const f32 arm = M::dot(M::cross(rod - pivot, M::normalize(rod - barrel)), z);
            const f32 error = targets[i] - mHinges[i]->currentAngle();
            const f32 strokeSpeed = arm * (rate + 3.0f * error);
            const HydraulicCylinder& c = *mCylinders[i];
            const f32 full = strokeSpeed > 0.0f ? c.maxExtendSpeed() : c.maxRetractSpeed();
            const f32 deadband = c.parameters().valveDeadband;
            f32 valve = strokeSpeed / full;
            if (valve != 0.0f)
                valve = (valve > 0.0f ? 1.0f : -1.0f) * (deadband + fabsf(valve) * (1.0f - deadband));
            mCylinders[i]->setValve(M::clamp(valve, -1.0f, 1.0f));
        }
        mStarted = true;
    }

    void operateCrane()
    {
        if (mElapsed >= kReleaseTime)
        {
            if (mHook->attached())
                mHook->detach();
            mWinch.setCommand(0.0f);
            return;
        }
        if (!mHook->attached() && mHookedAt < 0.0f)
        {
            if (mHook->attachNearest(0.15f, HookMode::Point) != 0xFFFFFFFFu)
                mHookedAt = mElapsed;
            else
            {
                mWinch.setCommand(-1.0f);
                return;
            }
        }
        const bool high = mLoad->position().y - kLoadCentre.y >= kHoistHeight;
        mWinch.setCommand(high ? 0.0f : 1.0f);
    }

    void observe()
    {
        // Bucket pivot and angle errors against the script, once the arm has had 0.5 s to catch up.
        if (mElapsed > 0.5f)
        {
            const M::vec3 pivot = mLinks[2]->pointToWorld(mLocalChild[2]);
            mWorstTracking = fmaxf(mWorstTracking, M::length(pivot - M::vec3(mTargetX, mTargetY, 0.0f)));
            const M::vec3 wall = mLinks[2]->directionToWorld(M::vec3(0.0f, 1.0f, 0.0f));
            mWorstCurl = fmaxf(mWorstCurl, fabsf(atan2f(-wall.x, wall.y) - mTargetCurl));
        }
        for (u32 i = 0; i < 3; ++i)
            mWorstSeparation = fmaxf(mWorstSeparation, M::length(mParents[i]->pointToWorld(mLocalParent[i]) -
                                                                 mLinks[i]->pointToWorld(mLocalChild[i])));
        mWorstStretch = fmaxf(mWorstStretch, mRope->stretch());
        if (mHook->attached())
            mHighestLoad = fmaxf(mHighestLoad, mLoad->position().y);
        for (const RigidBody* body : mBodies)
        {
            const M::vec3& p = body->position();
            mFinite = mFinite && isfinite(p.x) && isfinite(p.y) && isfinite(p.z);
        }
    }

    void drawCylinders()
    {
        glDisable(GL_LIGHTING);
        for (u32 i = 0; i < 3; ++i)
        {
            const HydraulicCylinder& c = *mCylinders[i];
            const M::vec3 a = mParents[i]->pointToWorld(mBarrelLocal[i]);
            const M::vec3 b = mLinks[i]->pointToWorld(mRodLocal[i]);
            const M::vec3 axis = M::normalize(b - a);
            const M::vec3 barrelEnd = a + axis * (0.9f * c.parameters().closedLength);
            const f32 front = 0.2f;
            glLineWidth(9.0f);
            glColor3f(0.95f, 0.55f, 0.1f);
            glBegin(GL_LINES);
            glVertex3f(a.x, a.y, a.z + front);
            glVertex3f(barrelEnd.x, barrelEnd.y, barrelEnd.z + front);
            glEnd();
            glLineWidth(4.0f);
            glColor3f(0.8f, 0.8f, 0.85f);
            glBegin(GL_LINES);
            glVertex3f(barrelEnd.x, barrelEnd.y, barrelEnd.z + front);
            glVertex3f(b.x, b.y, b.z + front);
            glEnd();
        }
        glLineWidth(1.0f);
    }

    void drawRope()
    {
        glDisable(GL_LIGHTING);
        glLineWidth(2.0f);
        glColor3f(0.15f, 0.15f, 0.15f);
        glBegin(GL_LINES);
        const M::vec3 drum = mRope->pointWorld(0);
        const M::vec3 sheave = mRope->pointWorld(1);
        const M::vec3 block = mRope->pointWorld(2);
        glVertex3f(drum.x, drum.y, drum.z);
        glVertex3f(sheave.x, sheave.y, sheave.z);
        for (int side = -1; side <= 1; side += 2)
        {
            glVertex3f(sheave.x + 0.06f * side, sheave.y, sheave.z);
            glVertex3f(block.x + 0.06f * side, block.y, block.z);
        }
        if (mHook->attached())
        {
            const M::vec3 hook = mHook->pointWorld();
            const M::vec3 top = mLoad->pointToWorld(M::vec3(0.0f, kLoadHalf.y, 0.0f));
            glVertex3f(hook.x, hook.y, hook.z);
            glVertex3f(top.x, top.y, top.z);
        }
        glEnd();
        glLineWidth(1.0f);
    }

    PhysicsWorld mWorld;
    Camera3D mCamera;
    ct::Vector<RigidBody*> mOwnedBodies;
    ct::Vector<CollisionShape*> mOwnedShapes;
    ct::Vector<RigidBody*> mBodies;
    ct::Vector<CollisionShape*> mShapes;
    ct::Vector<Joint*> mJoints;

    RigidBody* mLinks[3] = {};
    RigidBody* mParents[3] = {};
    HingeJoint* mHinges[3] = {};
    HydraulicCylinder* mCylinders[3] = {};
    M::vec3 mLocalParent[3];
    M::vec3 mLocalChild[3];
    M::vec3 mBarrelLocal[3];
    M::vec3 mRodLocal[3];
    f32 mPreviousTargets[3] = {};
    bool mStarted = false;
    f32 mRestAngles[3] = {};
    RigidBody* mFloor = nullptr;
    RigidBody* mBox = nullptr;
    f32 mTargetX = 0.0f;
    f32 mTargetY = 0.0f;

    RigidBody* mWinchHouse = nullptr;
    RigidBody* mMastTop = nullptr;
    RigidBody* mHookBlock = nullptr;
    RigidBody* mLoad = nullptr;
    u32 mHookBlockId = 0;
    u32 mLoadId = 0;
    Winch mWinch;
    Rope* mRope = nullptr;
    Hook* mHook = nullptr;

    f32 mElapsed = 0.0f;
    f32 mTargetCurl = 0.0f;
    f32 mWorstTracking = 0.0f;
    f32 mWorstCurl = 0.0f;
    f32 mWorstSeparation = 0.0f;
    f32 mWorstStretch = 0.0f;
    f32 mHookedAt = -1.0f;
    f32 mHighestLoad = -1.0f;
    bool mFinite = true;
    bool mPosesJudged = false;
    bool mScoopJudged = false;
    bool mScoopOk = false;
    bool mHoistJudged = false;
    bool mHoistOk = false;
    bool mReleaseJudged = false;
    bool mReleaseOk = false;
    char mPosesDetail[96] = {};
    char mScoopDetail[96] = {};
    char mHoistDetail[96] = {};
    char mReleaseDetail[96] = {};

    f32 mValves[3] = {};
    f32 mWinchCommand = 0.0f;
};

REGISTER_SCENE(Actuators3D, "3D demo: actuators", Category::Physics3D)

} // namespace testbed
