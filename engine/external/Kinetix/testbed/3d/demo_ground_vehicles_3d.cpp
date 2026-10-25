#include "scene.h"

#include "render3d.h"

#include <collision/CollisionShape.h>
#include <dynamics/PhysicsWorld.h>
#include <dynamics/RigidBody.h>
#include <ground/Ground.h>

#include <GL/gl.h>

#include <math.h>
#include <stdio.h>

namespace testbed
{

using namespace kinetix::Physics;

using kinetix::f32;
using kinetix::s32;
using kinetix::u32;
namespace M = kinetix::Math;

namespace
{
constexpr f32 kPi = 3.14159265f;
constexpr f32 kDeg = kPi / 180.0f;
constexpr f32 kRpmToRadians = 2.0f * kPi / 60.0f;
constexpr f32 kVerdictTime = 9.5f;

// Vehicle bodies are in this collision group; the wheel rays skip it so a trailer's rays miss its tractor.
constexpr u32 kVehicleGroup = 1u << 4;

constexpr f32 kCarLane = 0.0f;
constexpr f32 kTruckLane = -45.0f;
constexpr f32 kTractorLane = 45.0f;

// Tractor lane heightfield: flat soil, a 4 m long 0.25 m high cosine hump under the left wheel
// track, then a 15% ramp up onto a plateau 0.75 m high.
constexpr int kFieldCellsX = 64;
constexpr int kFieldCellsZ = 160;
constexpr f32 kFieldCell = 0.25f;
constexpr f32 kFieldStartZ = -8.0f;
constexpr f32 kBumpZ = 0.0f;
constexpr f32 kBumpHalfLength = 2.0f;
constexpr f32 kBumpHeight = 0.25f;
constexpr f32 kRampStartZ = 4.0f;
constexpr f32 kRampLength = 5.0f;
constexpr f32 kRampGrade = 0.15f;

f32 fieldHeight(f32 x, f32 z)
{
    f32 h = 0.0f;
    const f32 dz = fabsf(z - kBumpZ);
    const f32 dx = fabsf(x - 0.94f);
    if (dz < kBumpHalfLength)
    {
        f32 across = 0.0f;
        if (dx < 0.25f)
            across = 1.0f;
        else if (dx < 0.45f)
            across = 0.5f * (1.0f + cosf(kPi * (dx - 0.25f) / 0.2f));
        h += kBumpHeight * 0.5f * (1.0f + cosf(kPi * (z - kBumpZ) / kBumpHalfLength)) * across;
    }
    const f32 along = z - kRampStartZ;
    if (along > 0.0f)
        h += kRampGrade * M::min(along, kRampLength);
    return h;
}

SurfaceInfo laneSurface(u32, const M::vec3& point, void*)
{
    SurfaceInfo s;
    if (point.x > 0.5f * kTractorLane)
        s.type = SurfaceType::Soil;
    return s;
}

constexpr f32 kPedalRate = 2.5f;
constexpr f32 kPedalRelease = 4.0f;
constexpr f32 kSteerRate = 1.2f;
constexpr f32 kSteerReturn = 2.0f;
constexpr f32 kCoupleDistance = 0.5f;
// Landing legs of a dropped semi-trailer: one stiff spring-damper foot with friction a metre
// behind the kingpin, kingpin height below it (buildSemiTrailer's default coupling height).
constexpr f32 kKingpinHeight = 1.2f;
constexpr f32 kLegStiffness = 1.0e6f; // N/m
constexpr f32 kLegDamping = 1.5e5f;   // N s/m
constexpr f32 kLegFriction = 0.6f;

f32 wrap(f32 angle)
{
    while (angle > kPi)
        angle -= 2.0f * kPi;
    while (angle < -kPi)
        angle += 2.0f * kPi;
    return angle;
}

struct Unit
{
    RigidBody body;
    BoxShape shape{M::vec3(1.0f)};
    GroundVehicle vehicle;
    GroundVehicleLayout layout;
    u32 id = 0;
    Shade shade{0.8f, 0.8f, 0.8f};

    void addTo(PhysicsWorld& world)
    {
        shape = BoxShape(layout.collisionHalfExtents);
        BodyEntry entry;
        entry.body = &body;
        entry.shape = &shape;
        entry.friction = 0.5f;
        entry.filter.group = kVehicleGroup;
        id = world.addBody(entry);
        vehicle.setBody(&body, id);
        vehicle.setWorld(&world);
        QueryFilter rays;
        rays.collision.mask = ~kVehicleGroup;
        vehicle.setRayFilter(rays);
        vehicle.setSurfaceCallback(laneSurface, nullptr);
    }

    void place(const M::vec3& ground)
    {
        body.setPosition(ground + M::vec3(0.0f, layout.restHeight, 0.0f));
        body.setOrientation(M::quat());
        body.setVelocity(M::vec3(0.0f));
        body.setAngularVelocity(M::vec3(0.0f));
        body.clearAccumulators();
        body.calculateDerivedData();
    }
};
} // namespace

// Three vehicles, one per lane: a car through its gears and a corner; a 6x4 tractor unit pulling
// a laden semi-trailer into a right turn on the fifth wheel; a farm tractor over a bump and up a
// 15% soil ramp, front axle swinging on its pivot. Transient tyre model; drivetrain solved as one
// implicit system per step.
class GroundVehicles3D final : public Scene
{
public:
    GroundVehicles3D()
    {
        mWorld.setGravity(M::vec3(0.0f, -9.81f, 0.0f));
        mWorld.setFixedStep(1.0f / 60.0f);

        mGround.setBodyType(BodyType::Static);
        mGround.setPosition(M::vec3(0.0f, -1.0f, 0.0f));
        BodyEntry ground;
        ground.body = &mGround;
        ground.shape = &mGroundShape;
        ground.friction = 0.8f;
        mWorld.addBody(ground);

        buildField();
        mFieldBody.setBodyType(BodyType::Static);
        mFieldBody.setPosition(M::vec3(kTractorLane, 0.0f, 0.0f));
        BodyEntry field;
        field.body = &mFieldBody;
        field.shape = mField;
        field.friction = 0.8f;
        mWorld.addBody(field);
        mWorld.markStaticBroadphaseDirty();

        PassengerCarOptions car;
        car.gearbox = GearboxMode::Automatic;
        mCar.layout = buildPassengerCar(mCar.vehicle, mCar.body, car);
        mCar.shade = Shade{0.80f, 0.18f, 0.15f};
        mTruck.layout = buildTruck6x4(mTruck.vehicle, mTruck.body);
        mTruck.shade = Shade{0.20f, 0.40f, 0.75f};
        mTrailer.layout = buildSemiTrailer(mTrailer.vehicle, mTrailer.body);
        mTrailer.shade = Shade{0.82f, 0.82f, 0.78f};
        mTractor.layout = buildFarmTractor(mTractor.vehicle, mTractor.body);
        mTractor.shade = Shade{0.20f, 0.55f, 0.22f};
        mCar.addTo(mWorld);
        mTruck.addTo(mWorld);
        mTrailer.addTo(mWorld);
        mTractor.addTo(mWorld);
        mCoupling.type = CouplingType::FifthWheel;
        mCoupling.towPoint = mTruck.layout.towPoint;
        mCoupling.trailerPoint = mTrailer.layout.couplingPoint;
        place();

        mCamera.distance = 22.0f;
        mCamera.pitch = 0.35f;
        mCamera.yaw = kPi + 0.6f;
        followCamera();

        addCheck("car: accelerates to 14 m/s");
        addCheck("car: brakes and turns");
        addCheck("truck: pulls away and turns, trailer coupled");
        addCheck("tractor: over the bump on four wheels, up the ramp");
        addCheck("everything upright and finite");
    }

    ~GroundVehicles3D() override
    {
        mHitch.detach();
        delete mField;
    }

    const char* name() const override
    {
        return "3D demo: ground vehicles";
    }
    const char* description() const override
    {
        return "Three vehicles on transient tyres and implicit drivetrains. Middle: a 1400 kg automatic\n"
               "car runs up through the gears, brakes hard for a right-hander and powers out. Left: a 6x4\n"
               "tractor unit pulls a 19 t semi-trailer away on its fifth wheel and turns. Right: a\n"
               "farm tractor in fourth crosses a bump under its left wheels - the front axle swings\n"
               "on its pivot - and climbs a 15% soil ramp. Lines show the tyre forces (yellow) and\n"
               "loads (cyan). C moves the camera to the next vehicle.\n"
               "Manual mode (P): drive any of them - see the controls panel.";
    }

    bool drivable() const override
    {
        return true;
    }
    void setManual(bool manual) override
    {
        mManual = manual;
        if (manual)
            mDriven = mFollow;
    }
    const char* controlsHelp() const override
    {
        return "1 / 2 / 3\tdrive the car / the truck / the tractor\n"
               "W or Up\tthrottle\n"
               "S or Down\tbrake\n"
               "A / D, Left / Right\tsteer left / right\n"
               "X (hold)\thandbrake\n"
               "E / Q\tgear up / down (automatic: N, R, D)\n"
               "L\tdifferential locks on / off\n"
               "T\ttruck: uncouple / couple the trailer (back the fifth wheel under it)\n"
               "R\tstart over\n"
               "gamepad\tleft stick: steer, right trigger: throttle, left trigger: brake";
    }

    void control(const ControlInput& input, float dt) override
    {
        if (input.hit(Key::R))
        {
            restartRequested = true;
            return;
        }
        const int previous = mDriven;
        if (input.hit(Key::Num1))
            mDriven = 0;
        if (input.hit(Key::Num2))
            mDriven = 1;
        if (input.hit(Key::Num3))
            mDriven = 2;
        if (mDriven != previous)
            mThrottle = mBrake = mSteer = 0.0f;
        mFollow = mDriven;
        Unit& u = driven();

        const bool gas = input.down(Key::W) || input.down(Key::Up);
        const bool brake = input.down(Key::S) || input.down(Key::Down);
        mThrottle = approach(mThrottle, gas ? 1.0f : input.axis(PadAxis::RightTrigger), gas ? kPedalRate : kPedalRelease,
                             dt);
        mBrake = approach(mBrake, brake ? 1.0f : input.axis(PadAxis::LeftTrigger), brake ? kPedalRate : kPedalRelease,
                          dt);
        const f32 steer = input.pair(Key::A, Key::D, Key::Left, Key::Right);
        mSteer = rampControl(mSteer, steer != 0.0f ? steer : input.axis(PadAxis::LeftX), 0.0f, kSteerRate,
                             kSteerReturn, dt);
        mHandbrake = input.down(Key::X);

        Gearbox& box = u.vehicle.gearbox();
        const s32 target = box.targetGear();
        if (input.hit(Key::E))
        {
            if (box.mode() == GearboxMode::Automatic || target < 1)
                u.vehicle.selectGear(target + 1);
            else
                u.vehicle.shiftUp();
        }
        if (input.hit(Key::Q))
        {
            if (box.mode() == GearboxMode::Automatic)
                u.vehicle.selectGear(target > 0 ? 0 : -1);
            else
                u.vehicle.shiftDown();
        }
        if (input.hit(Key::L))
        {
            const s32 diffs[4] = {u.layout.frontDifferential, u.layout.rearDifferential, u.layout.centreDifferential,
                                  u.layout.secondRearDifferential};
            const bool lock = !locked(u);
            for (s32 d : diffs)
                if (d >= 0)
                    u.vehicle.setDifferentialLock(static_cast<u32>(d), lock);
        }
        if (input.hit(Key::T) && mDriven == 1)
        {
            if (mHitch.attached())
            {
                mHitch.detach();
                // The bodies' boxes overlap where a real tractor unit backs under the trailer, so they keep ignoring each other.
                mWorld.setPairIgnored(mTruck.id, mTrailer.id, true);
            }
            else if (couplingGap() < kCoupleDistance)
                mHitch.attach(mWorld, mTruck.vehicle, mTrailer.vehicle, mCoupling);
        }
    }

    void readouts(ct::Vector<Readout>& out) const override
    {
        const int which = mManual ? mDriven : mFollow;
        const Unit& u = which == 0 ? mCar : (which == 1 ? mTruck : mTractor);
        const VehicleTelemetry& t = u.vehicle.telemetry();
        const char* names[3] = {"car (automatic)", "6x4 truck + trailer", "farm tractor"};
        addReadout(out, mManual ? "driving" : "camera on", "%s", names[which]);
        addReadout(out, "speed", "%.1f km/h", double(t.forwardSpeed * 3.6f));
        addReadout(out, "engine", "%.0f rpm%s", double(t.engineRpm), t.engineRunning ? "" : " (stalled)");
        const s32 g = t.gear;
        const s32 target = u.vehicle.gearbox().targetGear();
        char gear[16];
        if (target < 0)
            snprintf(gear, sizeof(gear), "R");
        else if (target == 0)
            snprintf(gear, sizeof(gear), "N");
        else if (u.vehicle.gearbox().mode() == GearboxMode::Automatic)
            snprintf(gear, sizeof(gear), "D%d", t.shifting ? target : g);
        else
            snprintf(gear, sizeof(gear), "%d", target);
        addReadout(out, "gear", "%s%s", gear, t.shifting ? " (shifting)" : "");
        const VehicleControls& c = u.vehicle.controls();
        addReadout(out, "throttle / brake", "%.0f%% / %.0f%%%s", double(c.throttle * 100.0f), double(c.brake * 100.0f),
                   c.handbrake > 0.0f ? ", handbrake" : "");
        addReadout(out, "steering", "%+.1f deg", double(t.steeringAngle / kDeg));
        addReadout(out, "diff locks", "%s", locked(u) ? "locked" : "open");
        addReadout(out, "heading", "%.0f deg", double(t.heading / kDeg));
        if (which == 1)
        {
            if (mHitch.attached())
                addReadout(out, "trailer", "coupled, articulation %.1f deg",
                           double(mHitch.articulationAngle() / kDeg));
            else
                addReadout(out, "trailer", "uncoupled, fifth wheel %.2f m from the kingpin", double(couplingGap()));
        }
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
            mFollow = (mFollow + 1) % 3;
    }

    void step(float dt) override
    {
        mElapsed += dt;
        if (mManual)
            driveByHand();
        else
        {
            driveCar();
            driveTruck();
            driveTractor();
        }
        mCar.vehicle.applyForces(dt);
        mTruck.vehicle.applyForces(dt);
        mTrailer.vehicle.applyForces(dt);
        mTractor.vehicle.applyForces(dt);
        if (!mHitch.attached())
            landingLegs();
        mWorld.step(dt);
        measure(dt);
        followCamera();
        mCamera.chase = mManual;
        if (mManual)
        {
            const M::vec3 forward = driven().body.directionToWorld(M::vec3(0.0f, 0.0f, 1.0f));
            if (forward.x * forward.x + forward.z * forward.z > 1.0e-4f)
                mCamera.yaw = atan2f(-forward.x, -forward.z);
        }
    }

    void draw() override
    {
        setupLighting3D();
        drawGround(0.0f, 150.0f, 5.0f);
        drawField();
        drawUnit(mCar);
        drawUnit(mTruck);
        drawUnit(mTrailer);
        drawUnit(mTractor);
        const M::vec3 a = mTruck.body.pointToWorld(mCoupling.towPoint);
        drawLine3D(a, a + M::vec3(0.0f, 0.6f, 0.0f), 1.0f, 0.3f, 0.1f);
        if (!mHitch.attached())
        {
            const M::vec3 b = mTrailer.body.pointToWorld(mCoupling.trailerPoint);
            drawLine3D(b, b - M::vec3(0.0f, 0.6f, 0.0f), 0.2f, 1.0f, 0.3f);
        }
    }

    void evaluate() override
    {
        char buffer[160];
        if (mElapsed < kVerdictTime || mJudged)
        {
            if (mJudged)
                for (usize i = 0; i < mVerdicts.size(); ++i)
                    report(i, mVerdicts[i].ok, mVerdicts[i].detail.c_str());
            return;
        }
        mJudged = true;
        mVerdicts.clear();

        snprintf(buffer, sizeof(buffer), "top %.1f m/s, gear %d at the top", double(mCarTop), mCarTopGear);
        addVerdict(mCarTop > 14.0f, buffer);

        snprintf(buffer, sizeof(buffer), "braked from %.1f to %.1f m/s, turned %.0f deg", double(mCarBrakeFrom),
                 double(mCarBrakeTo), double(mCarTurned / kDeg));
        addVerdict(mCarBrakeFrom - mCarBrakeTo > 4.0f && mCarTurned > 60.0f * kDeg, buffer);

        snprintf(buffer, sizeof(buffer), "%.1f m/s, turned %.0f deg, articulation %.1f deg, hitch %.2f mm",
                 double(mTruckTop), double(mTruckTurned / kDeg), double(mWorstArticulation / kDeg),
                 double(1000.0f * mWorstSeparation));
        addVerdict(mTruckTop > 3.5f && mTruckTurned > 10.0f * kDeg && mWorstArticulation < 45.0f * kDeg &&
                       mWorstSeparation < 0.01f,
                   buffer);

        const f32 climbed = mTractor.body.position().y - mTractor.layout.restHeight;
        snprintf(buffer, sizeof(buffer), "lightest wheel on the bump %.1f kN, axle roll %.1f deg, %.2f m up, z %.1f",
                 double(mTractorLightest / 1000.0f), double(mTractorRoll / kDeg), double(climbed),
                 double(mTractor.body.position().z));
        addVerdict(mTractorBumpSeen && mTractorLightest > 500.0f && climbed > 0.8f * kRampGrade * kRampLength &&
                       mTractor.body.position().z > kRampStartZ + kRampLength,
                   buffer);

        snprintf(buffer, sizeof(buffer), "lowest up %.3f", double(mLowestUp));
        addVerdict(mFinite && mLowestUp > 0.85f, buffer);
        for (usize i = 0; i < mVerdicts.size(); ++i)
            report(i, mVerdicts[i].ok, mVerdicts[i].detail.c_str());
    }

private:
    struct Verdict
    {
        bool ok = false;
        ct::String detail;
    };

    void addVerdict(bool ok, const char* detail)
    {
        Verdict v;
        v.ok = ok;
        v.detail = ct::String(detail);
        mVerdicts.push_back(v);
    }

    void buildField()
    {
        const f32 halfX = 0.5f * kFieldCellsX * kFieldCell;
        for (int j = 0; j <= kFieldCellsZ; ++j)
        {
            for (int i = 0; i <= kFieldCellsX; ++i)
            {
                const f32 x = -halfX + i * kFieldCell;
                const f32 z = kFieldStartZ + j * kFieldCell;
                mFieldVertices.push_back(M::vec3(x, fieldHeight(x, z), z));
            }
        }
        for (int j = 0; j < kFieldCellsZ; ++j)
        {
            for (int i = 0; i < kFieldCellsX; ++i)
            {
                const u32 a = static_cast<u32>(j * (kFieldCellsX + 1) + i);
                const u32 b = a + 1;
                const u32 c = a + static_cast<u32>(kFieldCellsX + 1);
                const u32 d = c + 1;
                const u32 tri[6] = {a, c, b, b, c, d};
                for (u32 k : tri)
                    mFieldIndices.push_back(k);
            }
        }
        mField = new TrimeshShape(mFieldVertices.data(), static_cast<u32>(mFieldVertices.size()),
                                  mFieldIndices.data(), static_cast<u32>(mFieldIndices.size()));
    }

    void place()
    {
        mCar.place(M::vec3(kCarLane, 0.0f, -30.0f));
        mTruck.place(M::vec3(kTruckLane, 0.0f, -10.0f));
        TrailerCoupling::placeTrailer(mTruck.body, mTrailer.body, mCoupling);
        mHitch.attach(mWorld, mTruck.vehicle, mTrailer.vehicle, mCoupling);
        mTractor.place(M::vec3(kTractorLane, 0.0f, -4.0f));
        mCar.vehicle.selectGear(1);
        mTruck.vehicle.selectGear(1);
        mTractor.vehicle.selectGear(4);
        mTractor.vehicle.setDifferentialLock(static_cast<u32>(mTractor.layout.rearDifferential), true);
        mCarHeading = mCar.vehicle.telemetry().heading;
        mTruckHeading = mTruck.vehicle.telemetry().heading;
    }

    void driveCar()
    {
        VehicleControls c;
        const VehicleTelemetry& t = mCar.vehicle.telemetry();
        if (mElapsed < 5.0f)
        {
            c.throttle = 1.0f;
        }
        else if (mElapsed < 6.0f)
        {
            c.brake = 0.8f;
        }
        else if (mElapsed < 8.5f)
        {
            c.steering = 0.45f;
            c.throttle = kinetix::Clamp(0.25f + 0.3f * (8.0f - t.forwardSpeed), 0.0f, 1.0f);
        }
        else
        {
            c.throttle = 1.0f;
        }
        mCar.vehicle.setControls(c);
    }

    Unit& driven()
    {
        return mDriven == 0 ? mCar : (mDriven == 1 ? mTruck : mTractor);
    }

    static bool locked(const Unit& u)
    {
        const s32 diffs[4] = {u.layout.rearDifferential, u.layout.frontDifferential, u.layout.centreDifferential,
                              u.layout.secondRearDifferential};
        for (s32 d : diffs)
            if (d >= 0 && u.vehicle.differential(static_cast<u32>(d)).locked())
                return true;
        return false;
    }

    f32 couplingGap() const
    {
        return M::length(mTruck.body.pointToWorld(mCoupling.towPoint) -
                         mTrailer.body.pointToWorld(mCoupling.trailerPoint));
    }

    void landingLegs()
    {
        RigidBody& body = mTrailer.body;
        const M::vec3 foot = body.pointToWorld(mCoupling.trailerPoint + M::vec3(0.0f, -kKingpinHeight, -1.0f));
        const f32 penetration = -foot.y;
        if (penetration <= 0.0f)
            return;
        const M::vec3 v = body.velocityAtPoint(foot);
        const f32 normal = M::max(kLegStiffness * penetration - kLegDamping * v.y, 0.0f);
        // Friction as a stiff damper on the sliding speed, capped by mu N.
        M::vec3 slide(v.x, 0.0f, v.z);
        M::vec3 friction = slide * (-kLegDamping);
        const f32 limit = kLegFriction * normal;
        const f32 size = M::length(friction);
        if (size > limit && size > 0.0f)
            friction = friction * (limit / size);
        body.addForceAtPoint(M::vec3(0.0f, normal, 0.0f) + friction, foot);
    }

    // The driver's controls act on the current vehicle; the others stand on their parking brakes.
    void driveByHand()
    {
        Unit* units[3] = {&mCar, &mTruck, &mTractor};
        for (int i = 0; i < 3; ++i)
        {
            VehicleControls c;
            if (i == mDriven)
            {
                c.throttle = mThrottle;
                c.brake = mBrake;
                c.steering = mSteer;
                c.handbrake = mHandbrake ? 1.0f : 0.0f;
            }
            else
                c.handbrake = 1.0f;
            units[i]->vehicle.setControls(c);
        }
        VehicleControls trailer;
        if (!mHitch.attached())
        {
            trailer.brake = 1.0f;
            trailer.handbrake = 1.0f;
        }
        mTrailer.vehicle.setControls(trailer);
    }

    void driveTruck()
    {
        VehicleControls c;
        const f32 v = mTruck.vehicle.telemetry().forwardSpeed;
        c.throttle = mElapsed < 5.0f ? 1.0f : kinetix::Clamp(0.3f + 0.4f * (5.0f - v), 0.0f, 1.0f);
        c.steering = mElapsed > 4.0f ? 0.6f : 0.0f;
        mTruck.vehicle.setControls(c);
    }

    void driveTractor()
    {
        VehicleControls c;
        c.throttle = 1.0f;
        mTractor.vehicle.setControls(c);
    }

    void measure(f32 dt)
    {
        (void)dt;
        const Unit* units[4] = {&mCar, &mTruck, &mTrailer, &mTractor};
        for (const Unit* u : units)
        {
            const M::vec3 p = u->body.position();
            mFinite = mFinite && isfinite(p.x) && isfinite(p.y) && isfinite(p.z) &&
                      isfinite(u->vehicle.telemetry().forwardSpeed);
            mLowestUp = M::min(mLowestUp, u->body.directionToWorld(M::vec3(0.0f, 1.0f, 0.0f)).y);
        }
        const VehicleTelemetry& car = mCar.vehicle.telemetry();
        if (car.forwardSpeed > mCarTop)
        {
            mCarTop = car.forwardSpeed;
            mCarTopGear = car.gear;
        }
        if (mElapsed >= 5.0f && mCarBrakeFrom < 0.0f)
            mCarBrakeFrom = car.forwardSpeed;
        if (mElapsed >= 6.0f && mCarBrakeTo < 0.0f)
            mCarBrakeTo = car.forwardSpeed;
        mCarTurned += wrap(car.heading - mCarHeading);
        mCarHeading = car.heading;

        const VehicleTelemetry& truck = mTruck.vehicle.telemetry();
        mTruckTop = M::max(mTruckTop, truck.forwardSpeed);
        mTruckTurned += wrap(truck.heading - mTruckHeading);
        mTruckHeading = truck.heading;
        mWorstArticulation = M::max(mWorstArticulation, fabsf(mHitch.articulationAngle()));
        mWorstSeparation = M::max(mWorstSeparation, mHitch.separation());

        bool onBump = false;
        for (u32 w = 0; w < mTractor.vehicle.wheelCount(); ++w)
        {
            const M::vec3& c = mTractor.vehicle.wheel(w).contactPoint;
            if (fabsf(c.z - kBumpZ) < kBumpHalfLength && c.x - kTractorLane > 0.5f)
                onBump = true;
        }
        if (onBump)
        {
            mTractorBumpSeen = true;
            for (u32 w = 0; w < mTractor.vehicle.wheelCount(); ++w)
                mTractorLightest = M::min(mTractorLightest, mTractor.vehicle.wheel(w).load);
            mTractorRoll = M::max(mTractorRoll, fabsf(mTractor.vehicle.telemetry().axleRoll[0]));
        }
    }

    void followCamera()
    {
        const Unit* u = mFollow == 0 ? &mCar : (mFollow == 1 ? &mTruck : &mTractor);
        mCamera.target = u->body.position() + M::vec3(0.0f, 0.5f, 0.0f);
        mCamera.distance = mFollow == 1 ? 32.0f : (mFollow == 2 ? 14.0f : 22.0f);
    }

    void drawField()
    {
        glEnable(GL_LIGHTING);
        glColor3f(0.45f, 0.35f, 0.22f);
        glBegin(GL_TRIANGLES);
        for (usize t = 0; t + 2 < mFieldIndices.size(); t += 3)
        {
            const M::vec3 a = mFieldVertices[mFieldIndices[t]] + M::vec3(kTractorLane, 0.01f, 0.0f);
            const M::vec3 b = mFieldVertices[mFieldIndices[t + 1]] + M::vec3(kTractorLane, 0.01f, 0.0f);
            const M::vec3 c = mFieldVertices[mFieldIndices[t + 2]] + M::vec3(kTractorLane, 0.01f, 0.0f);
            M::vec3 n = M::cross(b - a, c - a);
            if (n.y < 0.0f)
                n = -n;
            glNormal3f(n.x, n.y, n.z);
            glVertex3f(a.x, a.y, a.z);
            glVertex3f(b.x, b.y, b.z);
            glVertex3f(c.x, c.y, c.z);
        }
        glEnd();
        glDisable(GL_LIGHTING);
    }

    void drawUnit(const Unit& u)
    {
        drawSolidBox(u.body.transform(), u.shape.halfExtents(), u.shade);
        for (u32 w = 0; w < u.vehicle.wheelCount(); ++w)
        {
            const WheelTelemetry& t = u.vehicle.wheel(w);
            const WheelParameters& p = u.vehicle.axle(w / 2).wheel;
            drawWheel(t.worldTransform, p.radius, p.width);
            if (!t.inContact)
                continue;
            const M::mat4& m = t.worldTransform;
            const M::vec3 heading(m[2][0], m[2][1], m[2][2]);
            const M::vec3 axle(m[0][0], m[0][1], m[0][2]);
            const f32 scale = 1.0f / (u.layout.mass * 9.81f);
            const M::vec3 force = (heading * t.longitudinalForce + axle * t.lateralForce) * (6.0f * scale);
            drawLine3D(t.contactPoint, t.contactPoint + force, 1.0f, 0.85f, 0.1f);
            drawLine3D(t.contactPoint, t.contactPoint + t.contactNormal * (t.load * 3.0f * scale), 0.2f, 0.9f, 0.95f);
        }
    }

    static void drawWheel(const M::mat4& m, f32 radius, f32 width)
    {
        const M::vec3 centre(m[3][0], m[3][1], m[3][2]);
        const M::vec3 axle(m[0][0], m[0][1], m[0][2]);
        const M::vec3 up(m[1][0], m[1][1], m[1][2]);
        const M::vec3 forward(m[2][0], m[2][1], m[2][2]);
        const int segments = 18;
        glEnable(GL_LIGHTING);
        glColor3f(0.12f, 0.12f, 0.13f);
        glBegin(GL_QUADS);
        for (int s = 0; s < segments; ++s)
        {
            const f32 a0 = 2.0f * kPi * s / segments;
            const f32 a1 = 2.0f * kPi * (s + 1) / segments;
            const M::vec3 r0 = up * cosf(a0) + forward * sinf(a0);
            const M::vec3 r1 = up * cosf(a1) + forward * sinf(a1);
            const M::vec3 n = r0 + r1;
            glNormal3f(n.x, n.y, n.z);
            const M::vec3 p[4] = {centre + r0 * radius + axle * (0.5f * width),
                                  centre + r1 * radius + axle * (0.5f * width),
                                  centre + r1 * radius - axle * (0.5f * width),
                                  centre + r0 * radius - axle * (0.5f * width)};
            for (const M::vec3& q : p)
                glVertex3f(q.x, q.y, q.z);
        }
        glEnd();
        glDisable(GL_LIGHTING);
        glColor3f(0.85f, 0.85f, 0.85f);
        glBegin(GL_LINES);
        for (int s = 0; s < 4; ++s)
        {
            const f32 a = 0.5f * kPi * s;
            const M::vec3 rim = centre + (up * cosf(a) + forward * sinf(a)) * (0.8f * radius);
            for (int side = -1; side <= 1; side += 2)
            {
                const M::vec3 offset = axle * (0.5f * width * side + 0.01f * side);
                glVertex3f(centre.x + offset.x, centre.y + offset.y, centre.z + offset.z);
                glVertex3f(rim.x + offset.x, rim.y + offset.y, rim.z + offset.z);
            }
        }
        glEnd();
    }

    PhysicsWorld mWorld;
    Camera3D mCamera;
    RigidBody mGround;
    BoxShape mGroundShape{M::vec3(150.0f, 1.0f, 150.0f)};
    RigidBody mFieldBody;
    TrimeshShape* mField = nullptr;
    ct::Vector<M::vec3> mFieldVertices;
    ct::Vector<u32> mFieldIndices;

    Unit mCar;
    Unit mTruck;
    Unit mTrailer;
    Unit mTractor;
    CouplingParameters mCoupling;
    TrailerCoupling mHitch;
    int mFollow = 0;

    int mDriven = 0;
    f32 mThrottle = 0.0f;
    f32 mBrake = 0.0f;
    f32 mSteer = 0.0f;
    bool mHandbrake = false;

    f32 mElapsed = 0.0f;
    bool mFinite = true;
    f32 mLowestUp = 1.0f;
    f32 mCarTop = 0.0f;
    s32 mCarTopGear = 0;
    f32 mCarBrakeFrom = -1.0f;
    f32 mCarBrakeTo = -1.0f;
    f32 mCarHeading = 0.0f;
    f32 mCarTurned = 0.0f;
    f32 mTruckTop = 0.0f;
    f32 mTruckHeading = 0.0f;
    f32 mTruckTurned = 0.0f;
    f32 mWorstArticulation = 0.0f;
    f32 mWorstSeparation = 0.0f;
    bool mTractorBumpSeen = false;
    f32 mTractorLightest = 1.0e9f;
    f32 mTractorRoll = 0.0f;
    bool mJudged = false;
    ct::Vector<Verdict> mVerdicts;
};

REGISTER_SCENE(GroundVehicles3D, "3D demo: ground vehicles", Category::Physics3D)

} // namespace testbed
