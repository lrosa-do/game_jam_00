#include "scene.h"

#include "render3d.h"

#include <actuators/Hook.h>
#include <actuators/HydraulicCylinder.h>
#include <actuators/Rope.h>
#include <collision/CollisionShape.h>
#include <dynamics/PhysicsWorld.h>
#include <dynamics/RigidBody.h>
#include <machinery/Bulldozer.h>
#include <machinery/Excavator.h>
#include <machinery/MobileCrane.h>

#include <GL/gl.h>

#include <math.h>
#include <stdio.h>
#include <string.h>

namespace testbed
{

using namespace kinetix::Physics;

using kinetix::f32;
using kinetix::u32;
namespace M = kinetix::Math;

namespace
{

constexpr f32 kGravity = 9.81f;
constexpr f32 kVerdictTime = 9.5f;
constexpr u32 kDiggableGroup = 1u << 7;

const Shade kMachineYellow{0.93f, 0.72f, 0.12f};
const Shade kSteel{0.45f, 0.46f, 0.50f};
const Shade kTrackBlack{0.16f, 0.16f, 0.17f};
const Shade kSoilBrown{0.50f, 0.36f, 0.22f};

void drawLine(const M::vec3& a, const M::vec3& b)
{
    glVertex3f(a.x, a.y, a.z);
    glVertex3f(b.x, b.y, b.z);
}

void drawCylinder(const HydraulicCylinder& c)
{
    const M::vec3 a = c.barrelPinWorld();
    const M::vec3 b = c.rodPinWorld();
    const M::vec3 axis = M::normalize(b - a);
    const M::vec3 barrelEnd = a + axis * (0.9f * c.parameters().closedLength);
    glLineWidth(8.0f);
    glColor3f(0.95f, 0.55f, 0.1f);
    glBegin(GL_LINES);
    drawLine(a, barrelEnd);
    glEnd();
    glLineWidth(4.0f);
    glColor3f(0.85f, 0.85f, 0.9f);
    glBegin(GL_LINES);
    drawLine(barrelEnd, b);
    glEnd();
    glLineWidth(1.0f);
}

void drawTracks(const RigidBody& hull, const TrackParameters& t)
{
    const f32 length = t.idlerZ - t.sprocketZ + t.idlerRadius + t.sprocketRadius;
    const f32 height = 2.0f * t.sprocketRadius;
    for (int side = 0; side < 2; ++side)
    {
        const f32 x = (side == 0 ? 0.5f : -0.5f) * t.gauge;
        const M::vec3 local(x, t.bottomHeight + 0.5f * height, 0.5f * (t.idlerZ + t.sprocketZ));
        M::mat4 transform = hull.transform();
        const M::vec3 centre = hull.pointToWorld(local);
        transform[3][0] = centre.x;
        transform[3][1] = centre.y;
        transform[3][2] = centre.z;
        drawSolidBox(transform, M::vec3(0.5f * t.shoeWidth, 0.5f * height, 0.5f * length), kTrackBlack);
    }
}

class MachineryScene : public Scene
{
public:
    MachineryScene()
    {
        mWorld.setGravity(M::vec3(0.0f, -kGravity, 0.0f));
        mWorld.setFixedStep(1.0f / 60.0f);
    }
    ~MachineryScene() override
    {
        mWorld.clear();
        for (RigidBody* body : mBodies)
            delete body;
        for (CollisionShape* shape : mShapes)
            delete shape;
    }

    bool is3D() const override
    {
        return true;
    }
    Camera3D camera3D() const override
    {
        return mCamera;
    }
    bool drivable() const override
    {
        return true;
    }

protected:
    // 1/2/3 switch machine (separate scenes), R restarts. True when the scene is going away.
    bool commonKeys(const ControlInput& input)
    {
        static const char* const machines[3] = {"3D demo: excavator", "3D demo: crane", "3D demo: bulldozer"};
        const Key keys[3] = {Key::Num1, Key::Num2, Key::Num3};
        for (int i = 0; i < 3; ++i)
            if (input.hit(keys[i]) && strcmp(machines[i], name()) != 0)
                requestedScene = machines[i];
        if (input.hit(Key::R))
            restartRequested = true;
        return requestedScene || restartRequested;
    }

    static f32 lever(f32 value, f32 command, f32 dt)
    {
        return rampControl(value, command, 0.0f, 4.0f, 6.0f, dt);
    }

    static void tracks(f32 drive, f32 turn, f32& left, f32& right)
    {
        left = M::clamp(drive - turn, -1.0f, 1.0f);
        right = M::clamp(drive + turn, -1.0f, 1.0f);
    }

    static void pumpReadouts(ct::Vector<Readout>& out, const HydraulicSystem& h)
    {
        for (u32 i = 0; i < h.pumpCount(); ++i)
        {
            char label[24];
            snprintf(label, sizeof(label), "pump %u", i + 1);
            addReadout(out, label, "%.0f bar, %.0f l/min%s", double(h.pump(i).pressure / 1.0e5f),
                       double(h.pump(i).flow), h.pump(i).saturated ? " (saturated)" : "");
        }
        addReadout(out, "engine", "%.0f rpm", double(h.engine().rpm()));
    }

    RigidBody& addBox(const M::vec3& centre, const M::vec3& halfExtents, f32 mass, u32 group = 1u)
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
        entry.friction = 0.7f;
        entry.filter.group = group;
        mWorld.addBody(entry);
        mBodies.push_back(body);
        mShapes.push_back(shape);
        return *body;
    }

    void drawBoxes()
    {
        drawWorldSolid(mWorld, mBodies, mShapes);
    }

    PhysicsWorld mWorld;
    Camera3D mCamera;
    ct::Vector<RigidBody*> mBodies;
    ct::Vector<CollisionShape*> mShapes;
    f32 mElapsed = 0.0f;
    bool mFinite = true;
    f32 mWorstSeparation = 0.0f;
};

} // namespace

class Excavator3D final : public MachineryScene
{
public:
    Excavator3D()
    {
        addBox(M::vec3(0.0f, -1.0f, -47.0f), M::vec3(50.0f, 1.0f, 50.0f), 0.0f);
        addBox(M::vec3(0.0f, -1.0f, 29.0f), M::vec3(50.0f, 1.0f, 20.0f), 0.0f);
        addBox(M::vec3(23.0f, -1.0f, 6.0f), M::vec3(20.0f, 1.0f, 3.0f), 0.0f);
        addBox(M::vec3(-23.0f, -1.0f, 6.0f), M::vec3(20.0f, 1.0f, 3.0f), 0.0f);
        mSoil = &addBox(M::vec3(0.0f, -1.5f, 6.0f), M::vec3(3.0f, 1.5f, 3.0f), 0.0f, kDiggableGroup);
        mWorld.markStaticBroadphaseDirty();

        mExcavator = new Excavator(mWorld, ExcavatorParameters::crawler20t(), M::vec3(0.0f, 0.745f, 0.0f));
        mExcavator->setGroundSoil(SoilParameters::loam());
        SoilBed bed;
        bed.minX = -3.0f;
        bed.maxX = 3.0f;
        bed.minZ = 3.0f;
        bed.maxZ = 9.0f;
        bed.surfaceHeight = 0.0f;
        bed.soil = SoilParameters::heavyClay();
        mExcavator->setSoilBed(bed);

        mCamera.target = M::vec3(0.0f, 2.0f, 3.0f);
        mCamera.distance = 20.0f;
        mCamera.pitch = 0.35f;
        mCamera.yaw = 0.9f;

        addCheck("teeth go into the pit and the clay resists the cut");
        addCheck("bucket curled with the load, slewed a quarter turn");
        addCheck("bucket opened to dump");
        addCheck("everything finite, arm holding together");
    }
    ~Excavator3D() override
    {
        delete mExcavator;
    }

    const char* name() const override
    {
        return "3D demo: excavator";
    }
    const char* description() const override
    {
        return "A 20 t crawler excavator on tracks (Bekker / Janosi-Hanamoto soil contact), its upper "
               "structure slewing on a hydraulic motor, boom, stick and bucket on cylinders - the bucket "
               "through its four-bar linkage - all from two load-sensing pumps on one diesel. The arm "
               "and the linkage are one articulated block (solved directly, not iterated). The tooth "
               "assist turns a target for the teeth into cylinder lengths: into the pit of heavy clay "
               "(brown), drag and curl through it against the cutting resistance, raise, slew 90 deg "
               "and open the bucket.\n"
               "Manual mode (P): work the levers yourself - see the controls panel.";
    }

    void setManual(bool manual) override
    {
        mManual = manual;
        if (manual)
        {
            mExcavator->clearToothTarget();
            mExcavator->clearSwingTarget();
            mControls = ExcavatorControls();
        }
    }
    const char* controlsHelp() const override
    {
        return "1 / 2 / 3\texcavator / crane / bulldozer\n"
               "Left / Right\tslew left / right\n"
               "Up / Down\tboom raise / lower\n"
               "I / K\tstick out / in\n"
               "J / L\tbucket curl / open\n"
               "W / S\ttracks forward / back\n"
               "A / D\tturn on the tracks left / right\n"
               "R\tstart over";
    }

    void control(const ControlInput& input, float dt) override
    {
        if (commonKeys(input))
            return;
        ExcavatorControls& c = mControls;
        c.swing = lever(c.swing, input.pair(Key::Right, Key::Left), dt);
        c.boom = lever(c.boom, input.pair(Key::Down, Key::Up), dt);
        c.stick = lever(c.stick, input.pair(Key::I, Key::K), dt);
        c.bucket = lever(c.bucket, input.pair(Key::L, Key::J), dt);
        mDrive = lever(mDrive, input.pair(Key::S, Key::W), dt);
        mTurn = lever(mTurn, input.pair(Key::D, Key::A), dt);
        tracks(mDrive, mTurn, c.leftTrack, c.rightTrack);
    }

    void readouts(ct::Vector<Readout>& out) const override
    {
        Excavator& e = *mExcavator;
        pumpReadouts(out, e.hydraulics());
        addReadout(out, "boom cylinder", "%.2f m stroke, %.0f bar", double(e.boomCylinder().stroke()),
                   double(e.boomCylinder().capPressure() / 1.0e5f));
        addReadout(out, "stick cylinder", "%.2f m stroke, %.0f bar", double(e.stickCylinder().stroke()),
                   double(e.stickCylinder().capPressure() / 1.0e5f));
        addReadout(out, "bucket cylinder", "%.2f m stroke, %.0f bar", double(e.bucketCylinder().stroke()),
                   double(e.bucketCylinder().capPressure() / 1.0e5f));
        addReadout(out, "slew", "%.0f deg", double(e.swingJoint().currentAngle() * 57.29578f));
        addReadout(out, "teeth", "%.2f m deep, soil force %.1f kN", double(e.teeth().depth()),
                   double(M::length(e.teeth().force()) / 1000.0f));
    }

    void step(float dt) override
    {
        mElapsed += dt;
        if (mManual)
            mExcavator->setControls(mControls);
        else
            script();
        mExcavator->update(dt);
        mWorld.step(dt);
        observe();
    }

    void draw() override
    {
        setupLighting3D();
        drawBoxes();
        const Excavator& e = *mExcavator;
        for (u32 i = 0; i < Excavator::BodyCount; ++i)
            drawSolidBox(e.body(i).transform(), e.shape(i).halfExtents(), i < 2 ? kMachineYellow : kSteel);
        drawTracks(e.body(Excavator::Undercarriage), e.parameters().tracks);
        glDisable(GL_LIGHTING);
        drawCylinder(mExcavator->boomCylinder());
        drawCylinder(mExcavator->stickCylinder());
        drawCylinder(mExcavator->bucketCylinder());
    }

    void evaluate() override
    {
        char buffer[160];
        // Each verdict once its part of the cycle is over; a check reported failing stays failed.
        if (mElapsed >= 5.0f)
        {
            snprintf(buffer, sizeof(buffer), "teeth %.2f m deep, soil force up to %.0f N", double(mDeepest),
                     double(mStrongest));
            report(0, mDeepest > 0.25f && mStrongest > 10000.0f, buffer);
        }
        if (mElapsed >= 7.5f && !mCarryJudged)
        {
            mCarryJudged = true;
            mCarryStroke = mExcavator->bucketCylinder().stroke();
            mCarrySwing = mExcavator->swingJoint().currentAngle();
        }
        if (mCarryJudged)
        {
            snprintf(buffer, sizeof(buffer), "bucket cylinder %.2f of %.2f m, slewed %.2f rad", double(mCarryStroke),
                     double(mExcavator->bucketCylinder().parameters().stroke), double(mCarrySwing));
            report(1, mCarryStroke > 0.7f * mExcavator->bucketCylinder().parameters().stroke && mCarrySwing > 1.4f,
                   buffer);
        }
        if (mElapsed >= kVerdictTime && !mDumpJudged)
        {
            mDumpJudged = true;
            mDumpStroke = mExcavator->bucketCylinder().stroke();
        }
        if (mDumpJudged)
        {
            snprintf(buffer, sizeof(buffer), "bucket cylinder %.2f m", double(mDumpStroke));
            report(2, mDumpStroke < mCarryStroke - 0.25f, buffer);
        }
        snprintf(buffer, sizeof(buffer), "worst pin opening %.1e m", double(mWorstSeparation));
        report(3, mFinite && mWorstSeparation < 1.0e-3f, buffer);
    }

private:
    void script()
    {
        Excavator& e = *mExcavator;
        const int phase = mElapsed < 2.0f ? 0 : mElapsed < 3.3f ? 1 : mElapsed < 4.6f ? 2 : mElapsed < 5.0f ? 3 :
                                                                                       mElapsed < 7.6f ? 4 : 5;
        if (phase == mPhase)
            return;
        mPhase = phase;
        ExcavatorControls idle;
        switch (phase)
        {
        case 0:
            e.setToothTarget(ArmPoint{7.5f, -0.8f}, -1.9f);
            break;
        case 1:
            e.setToothTarget(ArmPoint{7.5f, -1.45f}, -1.9f);
            break;
        case 2:
        {
            e.clearToothTarget();
            ExcavatorControls c;
            c.stick = 0.8f;
            c.boom = 0.15f;
            c.bucket = 0.3f;
            e.setControls(c);
            break;
        }
        case 3:
            e.setControls(idle);
            e.setToothTarget(ArmPoint{6.0f, 1.0f}, 0.6f);
            break;
        case 4:
            e.setSwingTarget(0.5f * 3.14159265f);
            break;
        default:
            e.setToothTarget(ArmPoint{6.5f, 1.5f}, -1.9f);
            break;
        }
    }

    void observe()
    {
        Excavator& e = *mExcavator;
        mDeepest = fmaxf(mDeepest, e.teeth().depth());
        mStrongest = fmaxf(mStrongest, M::length(e.teeth().force()));
        mWorstSeparation = fmaxf(mWorstSeparation, e.worstJointSeparation());
        mFinite = mFinite && e.finite();
    }

    Excavator* mExcavator = nullptr;
    RigidBody* mSoil = nullptr;
    ExcavatorControls mControls;
    f32 mDrive = 0.0f;
    f32 mTurn = 0.0f;
    int mPhase = -1;
    f32 mDeepest = 0.0f;
    f32 mStrongest = 0.0f;
    bool mCarryJudged = false;
    f32 mCarryStroke = 0.0f;
    f32 mCarrySwing = 0.0f;
    bool mDumpJudged = false;
    f32 mDumpStroke = 0.0f;
};

REGISTER_SCENE(Excavator3D, "3D demo: excavator", Category::Physics3D)

class Crane3D final : public MachineryScene
{
public:
    Crane3D()
    {
        addBox(M::vec3(0.0f, -1.0f, 0.0f), M::vec3(60.0f, 1.0f, 60.0f), 0.0f);
        mWorld.markStaticBroadphaseDirty();

        MobileCraneParameters p = MobileCraneParameters::allTerrain50t();
        p.buildDeployed = true;
        p.buildElevation = 1.0f;
        p.buildExtension = 2.0f;
        const M::vec3 head = MobileCrane::buildHeadLocal(p) + M::vec3(0.0f, p.chassisHeight, 0.0f);
        p.hookDrop = head.y - 2.0f * kLoadHalf - 0.8f - 0.05f;
        mParameters = p;
        mCrane = new MobileCrane(mWorld, p, M::vec3(0.0f, p.chassisHeight, 0.0f));
        mCrane->setAntiSway(true);
        const M::vec3 hook = mCrane->hook().pointWorld();
        mLoad = &addBox(M::vec3(hook.x, kLoadHalf, hook.z), M::vec3(kLoadHalf), kLoadMass);

        mCamera.target = M::vec3(0.0f, 7.0f, 2.0f);
        mCamera.distance = 38.0f;
        mCamera.pitch = 0.3f;
        mCamera.yaw = 0.8f;

        addCheck("hooks the load and hoists it 1 m");
        addCheck("slews 35 deg with it, the shaper keeping the swing under 8 deg");
        addCheck("limiter reads the load, well inside the chart");
        addCheck("outriggers carry crane and load");
        addCheck("everything finite, boom holding together");
    }
    ~Crane3D() override
    {
        delete mCrane;
    }

    const char* name() const override
    {
        return "3D demo: crane";
    }
    const char* description() const override
    {
        return "A 50 t all-terrain crane on its outriggers (beams out, jacks down, the carrier lifted): "
               "the superstructure slews on a hydraulic motor, the boom luffs on a cylinder and "
               "telescopes, the winch reeves 4 falls over the head sheave (radius and friction) to the "
               "hook block. The load moment indicator reads the hook load against the chart computed "
               "from the crane's stability. It hooks 5 t, hoists it 1 m and slews 35 deg, the slew "
               "lever through the anti-sway input shaper (tuned to the rope's pendulum period) so the "
               "load hardly swings.\n"
               "Manual mode (P): operate it yourself - see the controls panel.";
    }

    void setManual(bool manual) override
    {
        mManual = manual;
        if (manual)
        {
            mControls = mCrane->controls();
            mControls.slew = mControls.luff = mControls.telescope = mControls.hoist = 0.0f;
        }
    }
    const char* controlsHelp() const override
    {
        return "1 / 2 / 3\texcavator / crane / bulldozer\n"
               "Left / Right\tslew left / right\n"
               "Up / Down\tboom up / down (luff)\n"
               "E / Q\ttelescope out / in\n"
               "W / S\thoist up / down\n"
               "G\thook the nearest load / let it go\n"
               "B\tanti-sway shaper on / off\n"
               "M / N\toutrigger beams out / in\n"
               "X / Z\toutrigger jacks down / up\n"
               "R\tstart over";
    }

    void control(const ControlInput& input, float dt) override
    {
        if (commonKeys(input))
            return;
        MobileCraneControls& c = mControls;
        c.slew = lever(c.slew, input.pair(Key::Right, Key::Left), dt);
        c.luff = lever(c.luff, input.pair(Key::Down, Key::Up), dt);
        c.telescope = lever(c.telescope, input.pair(Key::Q, Key::E), dt);
        c.hoist = lever(c.hoist, input.pair(Key::S, Key::W), dt);
        c.outriggerBeams = M::clamp(c.outriggerBeams + input.pair(Key::N, Key::M) * 0.2f * dt, 0.0f, 1.0f);
        c.outriggerJacks = M::clamp(c.outriggerJacks + input.pair(Key::Z, Key::X) * 0.2f * dt, 0.0f, 1.0f);
        if (input.hit(Key::G))
        {
            if (mCrane->hook().attached())
                mCrane->hook().detach();
            else
                mHooked = mCrane->hook().attachNearest(0.6f, HookMode::Point) != 0xFFFFFFFFu;
        }
        if (input.hit(Key::B))
            mCrane->setAntiSway(!mCrane->antiSway());
    }

    void readouts(ct::Vector<Readout>& out) const override
    {
        MobileCrane& crane = *mCrane;
        const LoadMomentIndicator& lmi = crane.lmi();
        addReadout(out, "LMI load / rated", "%.0f / %.0f kg", double(lmi.load()), double(lmi.ratedLoad()));
        addReadout(out, "LMI utilisation", "%.0f%%%s", double(lmi.utilisation() * 100.0f),
                   lmi.cutOut() ? "  CUT-OUT" : (lmi.warning() ? "  WARNING" : ""));
        addReadout(out, "radius", "%.1f m", double(crane.radius()));
        addReadout(out, "boom", "%.1f m at %.0f deg", double(crane.boomLength()),
                   double(crane.boomElevation() * 57.29578f));
        addReadout(out, "slew", "%.0f deg", double(crane.slewJoint().currentAngle() * 57.29578f));
        addReadout(out, "hook", "%s, load swing %.1f deg", crane.hook().attached() ? "loaded" : "free",
                   double(crane.swingAngle() * 57.29578f));
        addReadout(out, "anti-sway", "%s", crane.antiSway() ? "on" : "off");
        addReadout(out, "outriggers", "beams %.0f%%, jacks %.0f%%", double(crane.controls().outriggerBeams * 100.0f),
                   double(crane.controls().outriggerJacks * 100.0f));
        addReadout(out, "jack loads", "%.0f %.0f %.0f %.0f kN", double(crane.outriggerLoad(0) / 1000.0f),
                   double(crane.outriggerLoad(1) / 1000.0f), double(crane.outriggerLoad(2) / 1000.0f),
                   double(crane.outriggerLoad(3) / 1000.0f));
        addReadout(out, "carrier on tyres", "%.0f kN", double(crane.carrierGroundLoad() / 1000.0f));
        addReadout(out, "luff cylinder", "%.0f bar", double(crane.luffCylinder().capPressure() / 1.0e5f));
    }

    void step(float dt) override
    {
        mElapsed += dt;
        if (mManual)
            mCrane->setControls(mControls);
        else
        {
            if (!mHooked && mElapsed > 0.3f)
                mHooked = mCrane->hook().attachNearest(0.3f, HookMode::Point) != 0xFFFFFFFFu;
            MobileCraneControls c = mCrane->controls();
            c.hoist = mHooked && mLoad->position().y - kLoadHalf < 1.0f ? 1.0f : 0.0f;
            c.slew = mElapsed > 2.5f && mCrane->slewJoint().currentAngle() < 0.6f ? 1.0f : 0.0f;
            mCrane->setControls(c);
        }
        mCrane->update(dt);
        mWorld.step(dt);
        observe();
    }

    void draw() override
    {
        setupLighting3D();
        drawBoxes();
        const MobileCrane& crane = *mCrane;
        for (u32 i = 0; i < MobileCrane::BodyCount; ++i)
        {
            const bool steel = i >= MobileCrane::Beam0 && i < MobileCrane::Superstructure;
            drawSolidBox(crane.body(i).transform(), crane.halfExtents(i), steel ? kSteel : kMachineYellow);
        }
        glDisable(GL_LIGHTING);
        drawCylinder(mCrane->luffCylinder());
        const Rope& rope = mCrane->rope();
        glLineWidth(2.0f);
        glColor3f(0.1f, 0.1f, 0.1f);
        glBegin(GL_LINES);
        for (u32 i = 0; i + 1 < rope.pointCount(); ++i)
            drawLine(rope.pointWorld(i), rope.pointWorld(i + 1));
        if (mCrane->hook().attached())
            drawLine(mCrane->hook().pointWorld(), mLoad->pointToWorld(M::vec3(0.0f, kLoadHalf, 0.0f)));
        glEnd();
        glLineWidth(1.0f);
    }

    void evaluate() override
    {
        char buffer[160];
        if (mElapsed >= 3.0f)
        {
            snprintf(buffer, sizeof(buffer), "hooked %d, load raised %.2f m", int(mHooked), double(mHighest));
            report(0, mHooked && mHighest > 0.8f, buffer);
        }
        if (mElapsed >= kVerdictTime && !mJudged)
        {
            mJudged = true;
            mSlew = mCrane->slewJoint().currentAngle();
            mLmiLoad = mCrane->lmi().load();
            mUtilisation = mCrane->lmi().utilisation();
        }
        if (!mJudged)
            return;
        snprintf(buffer, sizeof(buffer), "slewed %.2f rad, worst swing %.1f deg", double(mSlew),
                 double(mWorstSwing * 57.29578f));
        report(1, mSlew > 0.6f && mWorstSwing < 8.0f * 0.0174533f, buffer);
        const f32 hooked = kLoadMass + mParameters.hookBlockMass;
        snprintf(buffer, sizeof(buffer), "limiter %.0f kg (load + block %.0f kg), utilisation %.2f", double(mLmiLoad),
                 double(hooked), double(mUtilisation));
        report(2, fabsf(mLmiLoad - hooked) < 0.05f * hooked && mUtilisation < 0.9f && !mCrane->lmi().cutOut(),
               buffer);
        f32 weight = mParameters.chassisMass + 4.0f * mParameters.beamMass + mParameters.superMass +
                     mParameters.baseMass + mParameters.hookBlockMass + kLoadMass;
        for (u32 i = 0; i < MobileCraneParameters::SectionCount; ++i)
            weight += mParameters.sectionMass[i];
        weight *= kGravity;
        snprintf(buffer, sizeof(buffer), "jacks %.0f N (crane and load %.0f N), carrier on the ground %.0f N",
                 double(mJackAverage), double(weight), double(mCrane->carrierGroundLoad()));
        report(3, fabsf(mJackAverage - weight) < 0.03f * weight && mCrane->carrierGroundLoad() < 1.0f, buffer);
        snprintf(buffer, sizeof(buffer), "worst pin opening %.1e m", double(mWorstSeparation));
        report(4, mFinite && mWorstSeparation < 1.0e-3f, buffer);
    }

private:
    static constexpr f32 kLoadHalf = 0.6f;
    static constexpr f32 kLoadMass = 5000.0f;

    void observe()
    {
        MobileCrane& crane = *mCrane;
        if (crane.hook().attached())
            mHighest = fmaxf(mHighest, mLoad->position().y - kLoadHalf);
        if (mElapsed > 2.5f)
            mWorstSwing = fmaxf(mWorstSwing, crane.swingAngle());
        if (mElapsed > kVerdictTime - 1.0f && mElapsed <= kVerdictTime)
        {
            f32 sum = 0.0f;
            for (u32 i = 0; i < 4; ++i)
                sum += crane.outriggerLoad(i);
            mJackAverage += sum / 60.0f;
        }
        mWorstSeparation = fmaxf(mWorstSeparation, crane.worstJointSeparation());
        mFinite = mFinite && crane.finite();
    }

    MobileCraneParameters mParameters;
    MobileCrane* mCrane = nullptr;
    MobileCraneControls mControls;
    RigidBody* mLoad = nullptr;
    bool mHooked = false;
    f32 mHighest = 0.0f;
    f32 mWorstSwing = 0.0f;
    f32 mJackAverage = 0.0f;
    bool mJudged = false;
    f32 mSlew = 0.0f;
    f32 mLmiLoad = 0.0f;
    f32 mUtilisation = 0.0f;
};

REGISTER_SCENE(Crane3D, "3D demo: crane", Category::Physics3D)

class Bulldozer3D final : public MachineryScene
{
public:
    Bulldozer3D()
    {
        addBox(M::vec3(0.0f, -1.0f, 10.0f), M::vec3(40.0f, 1.0f, 40.0f), 0.0f);
        mWorld.markStaticBroadphaseDirty();
        const BulldozerParameters p = BulldozerParameters::crawler18t();
        mDozer = new Bulldozer(mWorld, p, M::vec3(0.0f, p.rideHeight, 0.0f));
        mDozer->setGroundSoil(SoilParameters::loam());
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                for (int k = 0; k < 2; ++k)
                {
                    mPile.push_back(&addBox(M::vec3(-0.6f + 0.6f * static_cast<f32>(i), 0.25f + 0.5f * static_cast<f32>(k),
                                                    5.0f + 0.6f * static_cast<f32>(j)),
                                            M::vec3(0.25f), 250.0f));
                    mPileMass += 250.0f;
                }
        mPileStart = pileCentre();

        mCamera.target = M::vec3(0.0f, 1.0f, 5.0f);
        mCamera.distance = 18.0f;
        mCamera.pitch = 0.35f;
        mCamera.yaw = 1.1f;

        addCheck("pushes the pile ahead of the blade");
        addCheck("blade forces bounded by what the tracks can push");
        addCheck("blade raised after the push");
        addCheck("everything finite, push frame holding together");
    }
    ~Bulldozer3D() override
    {
        delete mDozer;
    }

    const char* name() const override
    {
        return "3D demo: bulldozer";
    }
    const char* description() const override
    {
        return "An 18 t crawler dozer, hydrostatic travel (a pump and a motor a track) on loam: the push "
               "frame pivots on trunnions and is raised by two lift cylinders, the blade tilts on a "
               "hinge by a third. The operator holds the blade just off the ground and drives into a "
               "pile of 18 boxes (4.5 t), pushing it ahead, then raises the blade.\n"
               "Manual mode (P): drive it yourself - see the controls panel.";
    }

    const char* controlsHelp() const override
    {
        return "1 / 2 / 3\texcavator / crane / bulldozer\n"
               "W / S\ttracks forward / back\n"
               "A / D\tturn left / right\n"
               "Up / Down\tblade raise / lower\n"
               "Left / Right\tblade tilt (left end up / down)\n"
               "R\tstart over";
    }

    void control(const ControlInput& input, float dt) override
    {
        if (commonKeys(input))
            return;
        mControls.lift = lever(mControls.lift, input.pair(Key::Down, Key::Up), dt);
        mControls.tilt = lever(mControls.tilt, input.pair(Key::Right, Key::Left), dt);
        mDrive = lever(mDrive, input.pair(Key::S, Key::W), dt);
        mTurn = lever(mTurn, input.pair(Key::D, Key::A), dt);
        tracks(mDrive, mTurn, mControls.leftTrack, mControls.rightTrack);
    }

    void readouts(ct::Vector<Readout>& out) const override
    {
        Bulldozer& d = *mDozer;
        pumpReadouts(out, d.hydraulics());
        addReadout(out, "blade edge", "%.2f m up, tilt %.1f deg", double(d.edgeHeight()),
                   double(d.bladeTilt() * 57.29578f));
        const M::vec3 forward = d.tractor().directionToWorld(M::vec3(0.0f, 0.0f, 1.0f));
        addReadout(out, "blade force", "%.1f kN", double(fabsf(M::dot(d.bladeForce(), forward)) / 1000.0f));
        addReadout(out, "lift cylinder", "%.0f bar", double(d.liftCylinder().capPressure() / 1.0e5f));
        addReadout(out, "speed", "%.1f km/h", double(M::dot(d.tractor().velocity(), forward) * 3.6f));
        addReadout(out, "pile moved", "%.2f m", double(pileCentre().z - mPileStart.z));
    }

    void step(float dt) override
    {
        mElapsed += dt;
        Bulldozer& d = *mDozer;
        if (mManual)
            d.setControls(mControls);
        else
        {
            const f32 edge = mElapsed < 7.5f ? 0.06f : 1.2f;
            BulldozerControls c;
            c.lift = M::clamp(8.0f * (edge - d.edgeHeight()), -1.0f, 1.0f);
            c.tilt = M::clamp(-10.0f * d.bladeTilt(), -1.0f, 1.0f);
            c.leftTrack = mElapsed > 0.5f && mElapsed < 7.5f ? 1.0f : 0.0f;
            c.rightTrack = c.leftTrack;
            d.setControls(c);
        }
        d.update(dt);
        mWorld.step(dt);
        observe();
    }

    void draw() override
    {
        setupLighting3D();
        drawBoxes();
        const Bulldozer& d = *mDozer;
        for (u32 i = 0; i < Bulldozer::BodyCount; ++i)
            drawSolidBox(d.body(i).transform(), d.shape(i).halfExtents(), i == Bulldozer::PushFrame ? kSteel :
                                                                                                     kMachineYellow);
        drawTracks(d.body(Bulldozer::Tractor), d.parameters().tracks);
        glDisable(GL_LIGHTING);
        drawCylinder(mDozer->liftCylinder());
        drawCylinder(mDozer->tiltCylinder());
    }

    void evaluate() override
    {
        char buffer[160];
        if (mElapsed >= 7.4f && !mPushJudged)
        {
            mPushJudged = true;
            mMoved = pileCentre().z - mPileStart.z;
        }
        if (mPushJudged)
        {
            snprintf(buffer, sizeof(buffer), "pile moved %.2f m", double(mMoved));
            report(0, mMoved > 2.0f, buffer);
        }
        const BulldozerParameters& p = mDozer->parameters();
        const f32 weight = (p.tractorMass + p.pushFrameMass + p.bladeMass) * kGravity;
        const f32 grip = SoilParameters::loam().cohesion * 2.0f * p.tracks.shoeWidth * p.tracks.contactLength +
                         weight * tanf(SoilParameters::loam().frictionAngle);
        snprintf(buffer, sizeof(buffer), "worst along the blade %.0f N, tracks' limit %.0f N", double(mWorstPush),
                 double(grip));
        report(1, mWorstPush < grip + mPileMass * kGravity, buffer);
        if (mElapsed >= kVerdictTime && !mRaiseJudged)
        {
            mRaiseJudged = true;
            mRaised = mDozer->edgeHeight();
        }
        if (mRaiseJudged)
        {
            snprintf(buffer, sizeof(buffer), "edge %.2f m up", double(mRaised));
            report(2, mRaised > 0.5f, buffer);
        }
        snprintf(buffer, sizeof(buffer), "worst pin opening %.1e m", double(mWorstSeparation));
        report(3, mFinite && mWorstSeparation < 1.0e-3f, buffer);
    }

private:
    M::vec3 pileCentre() const
    {
        M::vec3 sum(0.0f);
        for (const RigidBody* b : mPile)
            sum += b->position();
        return sum / static_cast<f32>(mPile.size());
    }

    void observe()
    {
        Bulldozer& d = *mDozer;
        const M::vec3 forward = d.tractor().directionToWorld(M::vec3(0.0f, 0.0f, 1.0f));
        mWorstPush = fmaxf(mWorstPush, fabsf(M::dot(d.bladeForce(), forward)));
        mWorstSeparation = fmaxf(mWorstSeparation, d.worstJointSeparation());
        mFinite = mFinite && d.finite();
    }

    Bulldozer* mDozer = nullptr;
    BulldozerControls mControls;
    f32 mDrive = 0.0f;
    f32 mTurn = 0.0f;
    ct::Vector<RigidBody*> mPile;
    f32 mPileMass = 0.0f;
    M::vec3 mPileStart{0.0f};
    bool mPushJudged = false;
    f32 mMoved = 0.0f;
    f32 mWorstPush = 0.0f;
    bool mRaiseJudged = false;
    f32 mRaised = 0.0f;
};

REGISTER_SCENE(Bulldozer3D, "3D demo: bulldozer", Category::Physics3D)

} // namespace testbed
