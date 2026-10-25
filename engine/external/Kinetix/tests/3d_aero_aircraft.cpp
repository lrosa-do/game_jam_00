#include "check.h"

#include <collision/CollisionShape.h>
#include <dynamics/Aircraft.h>
#include <dynamics/PhysicsWorld.h>
#include <dynamics/RigidBody.h>

#include <math.h>

using namespace kinetix::Physics;
using kinetix::f32;
using kinetix::u32;
namespace KM = kinetix::Math;

namespace
{
constexpr f32 kPi = 3.14159265358979f;
constexpr f32 kDeg = kPi / 180.0f;

bool finiteVec(const KM::vec3& v)
{
    return isfinite(v.x) && isfinite(v.y) && isfinite(v.z);
}

struct Flight
{
    PhysicsWorld world;
    RigidBody body;
    Aircraft aircraft;
    BoxShape shape{KM::vec3(0.5f)};
    LightAircraftLayout layout;

    explicit Flight(f32 altitude = 1000.0f)
    {
        layout = buildLightAircraft(aircraft, body);
        shape = BoxShape(layout.fuselageHalfExtents);
        body.setPosition(KM::vec3(0.0f, altitude, 0.0f));
        BodyEntry entry;
        entry.body = &body;
        entry.shape = &shape;
        world.addBody(entry);
    }

    void run(f32 seconds, f32 rate = 60.0f)
    {
        const f32 step = 1.0f / rate;
        const int steps = static_cast<int>(seconds * rate + 0.5f);
        for (int i = 0; i < steps; ++i)
        {
            aircraft.applyForces(step);
            world.step(step);
        }
    }
};

void trimmedLevelFlight()
{
    Flight flight;
    TrimTarget target;
    target.airspeed = 55.0f;
    const TrimResult trim = flight.aircraft.trim(target);
    printf("cruise trim 55 m/s: converged %d in %u, alpha %.2f deg, elevator %.3f, aileron %.4f, "
           "rudder %.4f, throttle %.3f, %.0f rpm\n",
           int(trim.converged), trim.iterations, double(trim.angleOfAttack / kDeg),
           double(trim.elevator), double(trim.aileron), double(trim.rudder), double(trim.throttle),
           double(trim.rpm));
    CHECK(trim.converged);
    CHECK(trim.residual < 1e-4f);
    CHECK(trim.angleOfAttack > 0.0f && trim.angleOfAttack < 5.0f * kDeg);
    CHECK(trim.throttle > 0.4f && trim.throttle < 1.0f);
    CHECK(trim.rpm > 1800.0f && trim.rpm < 2800.0f);
    CHECK(fabsf(trim.elevator) < 0.5f);
    // A clockwise propeller rolls the aircraft left; the trim holds it with a
    // little right aileron.
    CHECK(trim.aileron > 0.0f && trim.aileron < 0.1f);
    CHECK(fabsf(trim.sideslip) < 1.0f * kDeg);
    CHECK(flight.aircraft.controls().elevator == 0.0f);
    CHECK_NEAR(flight.aircraft.controls().elevatorTrim, trim.elevator, 1e-6);

    flight.aircraft.applyForces(0.0f);
    const AircraftTelemetry& start = flight.aircraft.telemetry();
    CHECK_NEAR(start.trueAirspeed, 55.0, 1e-3);
    CHECK_NEAR(start.angleOfAttack, trim.angleOfAttack, 1e-4);
    CHECK_NEAR(start.verticalSpeed, 0.0, 1e-4);

    f32 lowest = 1.0e9f;
    f32 highest = -1.0e9f;
    f32 worstBank = 0.0f;
    for (int i = 0; i < 30 * 4; ++i)
    {
        flight.run(0.25f);
        lowest = fminf(lowest, flight.body.position().y);
        highest = fmaxf(highest, flight.body.position().y);
        worstBank = fmaxf(worstBank, fabsf(flight.aircraft.telemetry().bank));
    }
    printf("hands-off 30 s: altitude %.2f .. %.2f m, airspeed %.2f m/s, worst bank %.2f deg\n",
           double(lowest), double(highest), double(flight.aircraft.telemetry().trueAirspeed),
           double(worstBank / kDeg));
    CHECK(lowest > 995.0f && highest < 1005.0f);
    CHECK_NEAR(flight.aircraft.telemetry().trueAirspeed, 55.0, 1.0);
    CHECK(worstBank < 2.0f * kDeg);
}

void phugoidIsDamped()
{
    Flight flight;
    TrimTarget target;
    target.airspeed = 55.0f;
    CHECK(flight.aircraft.trim(target).converged);
    flight.body.setVelocity(flight.body.velocity() + KM::vec3(0.0f, 0.0f, 6.0f));

    f32 firstHalf = 0.0f;
    f32 secondHalf = 0.0f;
    f32 lowest = 1.0e9f;
    for (int i = 0; i < 120 * 4; ++i)
    {
        flight.run(0.25f);
        const f32 deviation = fabsf(flight.body.position().y - 1000.0f);
        if (i < 60 * 4)
            firstHalf = fmaxf(firstHalf, deviation);
        else
            secondHalf = fmaxf(secondHalf, deviation);
        lowest = fminf(lowest, flight.body.position().y);
    }
    printf("phugoid after +6 m/s: largest swing %.1f m in the first minute, %.1f m in the "
           "second\n",
           double(firstHalf), double(secondHalf));
    CHECK(firstHalf > 10.0f && firstHalf < 100.0f);
    CHECK(secondHalf < 0.8f * firstHalf);
    CHECK(lowest > 990.0f);
}

// Half a control for half a second from trimmed flight at 50 m/s. The rates
// are the cockpit's: roll right, nose up, nose right positive.
void controlResponse()
{
    struct Case
    {
        f32 aileron;
        f32 elevator;
        f32 rudder;
    };
    const Case cases[6] = {{0.5f, 0.0f, 0.0f}, {-0.5f, 0.0f, 0.0f}, {0.0f, 0.5f, 0.0f},
                           {0.0f, -0.5f, 0.0f}, {0.0f, 0.0f, 0.5f}, {0.0f, 0.0f, -0.5f}};
    for (const Case& c : cases)
    {
        Flight flight;
        TrimTarget target;
        target.airspeed = 50.0f;
        CHECK(flight.aircraft.trim(target).converged);
        ControlInputs controls = flight.aircraft.controls();
        controls.aileron = c.aileron;
        controls.elevator = c.elevator;
        controls.rudder = c.rudder;
        flight.aircraft.setControls(controls);
        flight.run(0.5f);
        const AircraftTelemetry& t = flight.aircraft.telemetry();
        const KM::vec3 rates = t.bodyRates;
        if (c.aileron != 0.0f)
        {
            CHECK(rates.x * c.aileron > 0.15f);
            CHECK(t.bank * c.aileron > 5.0f * kDeg);
            CHECK(fabsf(rates.y) < 0.2f * fabsf(rates.x));
        }
        if (c.elevator != 0.0f)
        {
            CHECK(rates.y * c.elevator > 0.1f);
            CHECK((t.pitch - 2.5f * kDeg) * c.elevator > 5.0f * kDeg);
            CHECK(fabsf(rates.x) < 0.2f * fabsf(rates.y));
        }
        if (c.rudder != 0.0f)
        {
            CHECK(rates.z * c.rudder > 0.1f);
            CHECK(t.heading * c.rudder > 3.0f * kDeg);
            // Yaw rolls it the same way: the wing moving forward lifts more.
            CHECK(rates.x * c.rudder > 0.0f);
        }
    }
}

void stall()
{
    Flight flight;
    Aircraft& aircraft = flight.aircraft;
    RigidBody& body = flight.body;
    const f32 speed = 30.0f;
    f32 peak = 0.0f;
    f32 peakAlpha = 0.0f;
    f32 liftAt[41];
    for (int degrees = 0; degrees <= 40; ++degrees)
    {
        const f32 alpha = static_cast<f32>(degrees) * kDeg;
        body.setOrientation(KM::angleAxis(-alpha, KM::vec3(1.0f, 0.0f, 0.0f)));
        body.setVelocity(KM::vec3(0.0f, 0.0f, speed));
        KM::vec3 force;
        KM::vec3 torque;
        aircraft.computeForces(force, torque);
        CHECK_NEAR(aircraft.telemetry().angleOfAttack, alpha, 1e-4);
        liftAt[degrees] = force.y;
        if (force.y > peak)
        {
            peak = force.y;
            peakAlpha = alpha;
        }
        const f32 stalled = aircraft.wing(flight.layout.mainWing).state().stalledFraction;
        if (degrees == 4)
            CHECK(stalled < 0.01f);
        if (degrees == 25)
            CHECK(stalled > 0.9f);
    }
    printf("stall at 30 m/s: peak lift %.0f N (%.2f g) at %.0f deg; %.0f N at %.0f deg\n",
           double(peak), double(peak / 9810.0f), double(peakAlpha / kDeg),
           double(liftAt[24]), 24.0);
    // Peak lift near the stall angle of the wing (12-17 degrees with washout).
    CHECK(peakAlpha > 11.0f * kDeg && peakAlpha < 18.0f * kDeg);
    CHECK(liftAt[24] < 0.75f * peak);
    // The 1 g stall speed this implies is in the light-aircraft range.
    const f32 stallSpeed = speed * sqrtf(9810.0f / peak);
    CHECK(stallSpeed > 22.0f && stallSpeed < 32.0f);

    ControlInputs flaps;
    flaps.flaps = 1.0f;
    aircraft.setControls(flaps);
    f32 flapPeak = 0.0f;
    for (int degrees = 0; degrees <= 30; ++degrees)
    {
        body.setOrientation(
            KM::angleAxis(-static_cast<f32>(degrees) * kDeg, KM::vec3(1.0f, 0.0f, 0.0f)));
        body.setVelocity(KM::vec3(0.0f, 0.0f, speed));
        KM::vec3 force;
        KM::vec3 torque;
        aircraft.computeForces(force, torque);
        flapPeak = fmaxf(flapPeak, force.y);
    }
    CHECK(flapPeak > 1.1f * peak);

    Flight slow;
    TrimTarget target;
    target.airspeed = 35.0f;
    CHECK(slow.aircraft.trim(target).converged);
    ControlInputs back = slow.aircraft.controls();
    back.throttle = 0.0f;
    back.elevator = 1.0f;
    slow.aircraft.setControls(back);
    f32 highestAlpha = 0.0f;
    f32 mostStalled = 0.0f;
    for (int i = 0; i < 8 * 60; ++i)
    {
        slow.run(1.0f / 60.0f);
        highestAlpha = fmaxf(highestAlpha, slow.aircraft.telemetry().angleOfAttack);
        mostStalled = fmaxf(mostStalled,
                            slow.aircraft.wing(slow.layout.mainWing).state().stalledFraction);
    }
    printf("full aft stick from 35 m/s: alpha reached %.1f deg, %.0f%% of the wing stalled\n",
           double(highestAlpha / kDeg), double(100.0f * mostStalled));
    CHECK(highestAlpha > 14.0f * kDeg);
    CHECK(mostStalled > 0.3f);
    CHECK(finiteVec(slow.body.position()) && finiteVec(slow.body.angularVelocity()));
}

void glide()
{
    Flight flight(1500.0f);
    TrimTarget target;
    target.airspeed = 36.0f;
    target.throttle = 0.0f;
    const TrimResult trim = flight.aircraft.trim(target);
    CHECK(trim.converged);
    CHECK(trim.climbAngle < 0.0f);
    flight.run(5.0f);
    const KM::vec3 start = flight.body.position();
    flight.run(30.0f);
    const KM::vec3 end = flight.body.position();
    const f32 distance = sqrtf((end.x - start.x) * (end.x - start.x) +
                               (end.z - start.z) * (end.z - start.z));
    const f32 ratio = distance / (start.y - end.y);
    printf("idle glide at 36 m/s: trim says %.1f:1, flown %.1f:1, %.0f rpm windmilling\n",
           double(-1.0f / tanf(trim.climbAngle)), double(ratio),
           double(flight.aircraft.propeller(0).rpm()));
    CHECK(ratio > 5.0f && ratio < 15.0f);
    CHECK_NEAR(ratio, -1.0f / tanf(trim.climbAngle), 1.0);
    CHECK_NEAR(flight.aircraft.telemetry().trueAirspeed, 36.0, 2.0);
}

void telemetry()
{
    Flight flight;
    Aircraft& aircraft = flight.aircraft;
    RigidBody& body = flight.body;
    KM::vec3 force;
    KM::vec3 torque;

    // Parked in a 20 m/s wind from straight ahead: 20 m/s of airspeed, no
    // angle of attack, no sideslip.
    aircraft.ownAtmosphere().setWind(KM::vec3(0.0f, 0.0f, -20.0f));
    aircraft.computeForces(force, torque);
    CHECK_NEAR(aircraft.telemetry().trueAirspeed, 20.0, 1e-4);
    CHECK_NEAR(aircraft.telemetry().angleOfAttack, 0.0, 1e-5);
    CHECK_NEAR(aircraft.telemetry().sideslip, 0.0, 1e-5);
    const f32 windLift = force.y;
    aircraft.ownAtmosphere().setWind(KM::vec3(0.0f));
    body.setVelocity(KM::vec3(0.0f, 0.0f, 20.0f));
    aircraft.computeForces(force, torque);
    CHECK(windLift > 100.0f);
    CHECK_NEAR(force.y, windLift, 1e-3f * windLift);

    // Wind from the right is positive sideslip.
    body.setVelocity(KM::vec3(0.0f));
    aircraft.ownAtmosphere().setWind(KM::vec3(10.0f, 0.0f, -30.0f));
    aircraft.computeForces(force, torque);
    CHECK(aircraft.telemetry().sideslip > 0.0f);
    CHECK_NEAR(aircraft.telemetry().sideslip, asinf(10.0f / sqrtf(1000.0f)), 1e-4);
    aircraft.ownAtmosphere().setWind(KM::vec3(0.0f));

    Atmosphere shared;
    shared.setWind(KM::vec3(0.0f, 0.0f, -15.0f));
    aircraft.setAtmosphere(&shared);
    aircraft.computeForces(force, torque);
    CHECK_NEAR(aircraft.telemetry().trueAirspeed, 15.0, 1e-4);
    aircraft.setAtmosphere(nullptr);

    const KM::quat roll = KM::angleAxis(30.0f * kDeg, KM::vec3(0.0f, 0.0f, 1.0f));
    body.setOrientation(roll);
    body.setVelocity(KM::vec3(0.0f, 0.0f, 40.0f));
    body.setAngularVelocity(roll * KM::vec3(-0.2f, -0.3f, 0.1f));
    aircraft.computeForces(force, torque);
    CHECK_NEAR(aircraft.telemetry().bank, 30.0 * kDeg, 1e-4);
    CHECK_NEAR(aircraft.telemetry().bodyRates.x, 0.1, 1e-5);  // roll
    CHECK_NEAR(aircraft.telemetry().bodyRates.y, 0.2, 1e-5);  // pitch: -X is right
    CHECK_NEAR(aircraft.telemetry().bodyRates.z, 0.3, 1e-5);  // yaw: -Y is down
    body.setOrientation(KM::angleAxis(-20.0f * kDeg, KM::vec3(0.0f, 1.0f, 0.0f)));
    body.setAngularVelocity(KM::vec3(0.0f));
    aircraft.computeForces(force, torque);
    CHECK_NEAR(aircraft.telemetry().heading, 20.0 * kDeg, 1e-4);

    // High up the air is thinner: equivalent airspeed below true.
    body.setOrientation(KM::quat());
    body.setPosition(KM::vec3(0.0f, 5000.0f, 0.0f));
    body.setVelocity(KM::vec3(0.0f, -2.0f, 60.0f));
    aircraft.computeForces(force, torque);
    CHECK_NEAR(aircraft.telemetry().altitude, 5000.0, 1e-2);
    CHECK_NEAR(aircraft.telemetry().verticalSpeed, -2.0, 1e-5);
    CHECK_NEAR(aircraft.telemetry().equivalentAirspeed / aircraft.telemetry().trueAirspeed,
               sqrtf(0.736429f / 1.225f), 2e-3);
    CHECK_NEAR(aircraft.telemetry().mach, aircraft.telemetry().trueAirspeed / 320.529f, 2e-3);

    const f32 omega = aircraft.propeller(0).angularSpeed();
    aircraft.computeForces(force, torque);
    CHECK_NEAR(aircraft.propeller(0).angularSpeed(), omega, 1e-6);
    CHECK_NEAR(body.velocity().z, 60.0, 1e-6);
}

void downwashLink()
{
    Flight flight;
    Aircraft& aircraft = flight.aircraft;
    const u32 tail = flight.layout.horizontalTail;
    flight.body.setOrientation(KM::angleAxis(-6.0f * kDeg, KM::vec3(1.0f, 0.0f, 0.0f)));
    flight.body.setVelocity(KM::vec3(0.0f, 0.0f, 45.0f));
    KM::vec3 force;
    KM::vec3 torque;
    aircraft.computeForces(force, torque);
    const f32 withWake = aircraft.wing(tail).state().lift;
    const f32 inducedAngle = aircraft.wing(flight.layout.mainWing).state().inducedAngle;
    CHECK(inducedAngle > 0.0f);
    // The main wing's wake pushes down 1.7 * alpha_i * V on the tail.
    CHECK_NEAR(aircraft.wing(tail).inducedFlow().y, -1.7f * inducedAngle * 45.0f, 1e-3);

    aircraft.setDownwash(tail, -1);
    aircraft.computeForces(force, torque);
    const f32 cleanAir = aircraft.wing(tail).state().lift;
    CHECK(aircraft.wing(tail).inducedFlow().y == 0.0f);
    CHECK(cleanAir > withWake + 100.0f);

    aircraft.setDownwash(tail, static_cast<kinetix::s32>(tail));
    aircraft.setDownwash(tail, 17);
    aircraft.setDownwash(42, 0);
    aircraft.computeForces(force, torque);
    CHECK(aircraft.wing(tail).inducedFlow().y == 0.0f);
}

void stepCallbackTiming()
{
    Flight manual;
    Flight callback;
    TrimTarget target;
    target.airspeed = 50.0f;
    CHECK(manual.aircraft.trim(target).converged);
    CHECK(callback.aircraft.trim(target).converged);
    ControlInputs controls = manual.aircraft.controls();
    controls.aileron = 0.3f;
    controls.elevator = 0.2f;
    manual.aircraft.setControls(controls);
    callback.aircraft.setControls(controls);

    callback.world.setFixedStep(1.0f / 60.0f);
    callback.world.setStepCallback(&Aircraft::stepCallback, &callback.aircraft);
    callback.aircraft.applyForces(1.0f / 60.0f);
    for (int frame = 0; frame < 300; ++frame)
    {
        callback.world.update(1.0f / 30.0f);
        manual.run(1.0f / 30.0f);
    }
    const KM::vec3 a = manual.body.position();
    const KM::vec3 b = callback.body.position();
    printf("callback vs manual after 10 s: %.6f m apart\n", double(KM::length(a - b)));
    CHECK(KM::length(a - b) < 1e-3f);
    CHECK(KM::length(a - KM::vec3(0.0f, 1000.0f, 0.0f)) > 100.0f);
}

// The script is aerobatic, not a flight plan: it ends several kilometres below where it started; altitude is not checked.
void longRuns()
{
    for (f32 rate : {60.0f, 120.0f})
    {
        Flight flight(4000.0f);
        TrimTarget target;
        target.airspeed = 50.0f;
        CHECK(flight.aircraft.trim(target).converged);
        bool finite = true;
        f32 fastest = 0.0f;
        f32 fastestSpin = 0.0f;
        const f32 step = 1.0f / rate;
        for (int i = 0; i < static_cast<int>(180.0f * rate); ++i)
        {
            const f32 t = static_cast<f32>(i) * step;
            ControlInputs c = flight.aircraft.controls();
            c.aileron = 0.6f * sinf(0.21f * t);
            const f32 heightError = 4000.0f - flight.body.position().y;
            c.elevator = 0.4f * sinf(0.13f * t + 1.0f) + KM::clamp(heightError * 0.002f, -0.4f, 0.4f);
            c.rudder = 0.4f * sinf(0.07f * t);
            c.throttle = 0.5f + 0.5f * sinf(0.05f * t);
            c.flaps = t > 60.0f && t < 90.0f ? 1.0f : 0.0f;
            flight.aircraft.setControls(c);
            if (i == static_cast<int>(120.0f * rate))
            {
                flight.body.setVelocity(KM::vec3(5.0f, -10.0f, -25.0f));
                flight.body.setAngularVelocity(KM::vec3(2.0f, -3.0f, 4.0f));
            }
            flight.aircraft.applyForces(step);
            flight.world.step(step);
            const KM::vec3 v = flight.body.velocity();
            const KM::vec3 w = flight.body.angularVelocity();
            if (!finiteVec(v) || !finiteVec(w) || !finiteVec(flight.body.position()) ||
                !isfinite(flight.aircraft.propeller(0).angularSpeed()))
                finite = false;
            fastest = fmaxf(fastest, KM::length(v));
            fastestSpin = fmaxf(fastestSpin, KM::length(w));
        }
        printf("%.0f Hz, 3 min: fastest %.1f m/s, fastest rotation %.2f rad/s, ended %.0f m below the start\n",
               double(rate), double(fastest), double(fastestSpin),
               double(4000.0f - flight.body.position().y));
        CHECK(finite);
        CHECK(fastest < 150.0f);
        CHECK(fastestSpin < 8.0f);
    }
}

void degenerateUse()
{
    Aircraft empty;
    KM::vec3 force(1.0f);
    KM::vec3 torque(1.0f);
    empty.applyForces(1.0f / 60.0f);
    empty.computeForces(force, torque);
    CHECK(force.x == 0.0f && torque.x == 0.0f);
    CHECK(!empty.trim(TrimTarget()).converged);
    Aircraft::stepCallback(1.0f / 60.0f, nullptr);

    ControlInputs wild;
    wild.aileron = 5.0f;
    wild.elevator = -7.0f;
    wild.throttle = NAN;
    wild.flaps = -1.0f;
    empty.setControls(wild);
    CHECK(empty.controls().aileron == 1.0f);
    CHECK(empty.controls().elevator == -1.0f);
    CHECK(empty.controls().throttle == 0.0f);
    CHECK(empty.controls().flaps == 0.0f);

    const u32 prop = empty.addPropeller(Propeller(), 3);
    CHECK(empty.propellerEngine(prop) == -1);

    Flight flight;
    const KM::vec3 before = flight.body.position();
    TrimTarget impossible;
    impossible.airspeed = 15.0f;
    const TrimResult result = flight.aircraft.trim(impossible);
    CHECK(!result.converged);
    CHECK_NEAR(KM::length(flight.body.position() - before), 0.0, 1e-6);
    CHECK_NEAR(KM::length(flight.body.velocity()), 0.0, 1e-6);
}
}

int main()
{
    trimmedLevelFlight();
    phugoidIsDamped();
    controlResponse();
    stall();
    glide();
    telemetry();
    downwashLink();
    stepCallbackTiming();
    longRuns();
    degenerateUse();
    return TEST_RESULT();
}
