#include "check.h"

#include <dynamics/RigidBody.h>
#include <rotorcraft/Helicopter.h>

#include <math.h>

using namespace kinetix::Physics;
using kinetix::f32;
using kinetix::u32;
namespace KM = kinetix::Math;

namespace
{
constexpr f32 kDeg = 3.14159265358979f / 180.0f;

struct Trimmed
{
    RigidBody body;
    Helicopter helicopter;
    LightHelicopterLayout layout;

    explicit Trimmed(f32 height = 300.0f)
    {
        layout = buildLightHelicopter(helicopter, body);
        body.setPosition(KM::vec3(0.0f, height, 0.0f));
    }
};

void hoverTrim()
{
    Trimmed t;
    HelicopterTrimTarget target;
    const HelicopterTrimResult r = t.helicopter.trim(target);
    printf("hover trim: converged %d in %u, collective %.3f, cyclic %.3f / %.3f, pedals %.3f, "
           "pitch %.2f deg, bank %.2f deg, %.1f kW (main %.1f, tail %.1f)\n",
           int(r.converged), r.iterations, double(r.collective), double(r.cyclicLongitudinal),
           double(r.cyclicLateral), double(r.pedals), double(r.pitch / kDeg), double(r.bank / kDeg),
           double(r.powerRequired / 1000.0f), double(r.mainRotorPower / 1000.0f),
           double(r.tailRotorPower / 1000.0f));
    CHECK(r.converged);
    CHECK_NEAR(r.pedals, t.layout.hoverPedals, 0.05);
    CHECK_NEAR(r.collective, t.layout.hoverCollective, 0.05);
    // 140 to 200 kW for a 1300 kg light helicopter out of ground effect.
    CHECK(r.powerRequired > 140000.0f && r.powerRequired < 210000.0f);
    // The tail rotor carries the main rotor's torque: thrust times its arm
    // about the centre of mass, give or take the fin and the tail rotor's
    // own torque.
    const RotorLoads& main = t.helicopter.mainRotor().loads();
    const RotorLoads& tail = t.helicopter.tailRotor().loads();
    const f32 arm = -t.helicopter.tailRotor().description().position.z;
    printf("anti-torque: main rotor torque %.0f N m, tail rotor thrust %.0f N x %.2f m = %.0f N m\n",
           double(main.torque), double(tail.thrust), double(arm), double(tail.thrust * arm));
    CHECK_NEAR(tail.thrust * arm, main.torque, 0.08 * main.torque);
    // Hanging nose up a few degrees (the shaft is tilted forward) and left
    // side low, leaning into the tail rotor's push to the right.
    CHECK(r.pitch > 1.0f * kDeg && r.pitch < 7.0f * kDeg);
    CHECK(r.bank < 0.0f && r.bank > -5.0f * kDeg);
    CHECK(r.cyclicLateral < 0.0f);
    KM::vec3 force;
    KM::vec3 torque;
    t.helicopter.computeForces(force, torque);
    const f32 weight = t.body.mass() * 9.81f;
    CHECK_NEAR(force.y, weight, 0.002 * weight);
    CHECK(KM::length(torque) < 0.002f * weight * 4.6f);
    CHECK_NEAR(t.helicopter.transmission().rotorSpeed(), t.helicopter.ratedRotorSpeed(), 1e-3);
    CHECK(t.helicopter.engine().running());
}

void powerBucket()
{
    Trimmed t;
    constexpr u32 kCount = 25;
    f32 speeds[kCount];
    f32 power[kCount];
    u32 best = 0;
    printf("power required:");
    for (u32 i = 0; i < kCount; ++i)
    {
        HelicopterTrimTarget target;
        target.airspeed = 2.5f * static_cast<f32>(i);
        const HelicopterTrimResult r = t.helicopter.trim(target);
        CHECK(r.converged);
        speeds[i] = target.airspeed;
        power[i] = r.powerRequired;
        if (power[i] < power[best])
            best = i;
        if (i % 4 == 0)
            printf(" %.0f m/s %.0f kW,", double(speeds[i]), double(power[i] / 1000.0f));
    }
    printf("\nbucket minimum %.1f kW at %.1f m/s (hover %.1f kW, 60 m/s %.1f kW)\n",
           double(power[best] / 1000.0f), double(speeds[best]), double(power[0] / 1000.0f),
           double(power[kCount - 1] / 1000.0f));
    // The bucket: its bottom is the best-climb speed Vy of light helicopters (50 to 65 kt).
    for (u32 i = 1; i <= best; ++i)
        CHECK(power[i] < power[i - 1]);
    for (u32 i = best + 1; i < kCount; ++i)
        CHECK(power[i] > power[i - 1]);
    CHECK(speeds[best] >= 25.0f && speeds[best] <= 40.0f);
    CHECK(power[best] < 0.65f * power[0]);
    CHECK(power[kCount - 1] > 1.5f * power[best]);
}

void cruiseTrim()
{
    Trimmed t;
    HelicopterTrimTarget hover;
    const HelicopterTrimResult h = t.helicopter.trim(hover);
    HelicopterTrimTarget cruise;
    cruise.airspeed = 50.0f;
    const HelicopterTrimResult r = t.helicopter.trim(cruise);
    printf("cruise trim at 50 m/s: converged %d, collective %.3f, cyclic long %.3f lat %.3f, pedals "
           "%.3f, pitch %.2f deg, bank %.2f deg, %.1f kW, a1s %.2f deg, b1s %.2f deg\n",
           int(r.converged), double(r.collective), double(r.cyclicLongitudinal), double(r.cyclicLateral),
           double(r.pedals), double(r.pitch / kDeg), double(r.bank / kDeg), double(r.powerRequired / 1000.0f),
           double(r.longitudinalFlapping / kDeg), double(r.lateralFlapping / kDeg));
    CHECK(r.converged);
    CHECK(r.cyclicLongitudinal > h.cyclicLongitudinal + 0.2f);
    CHECK(r.pitch < h.pitch);
    CHECK(t.helicopter.mainRotor().loads().force.z > 0.0f);
    // The fin takes over much of the tail rotor's work: less left pedal,
    // far less tail rotor power than in hover.
    CHECK(r.pedals > h.pedals);
    CHECK(r.tailRotorPower < 0.5f * h.tailRotorPower);
    CHECK(fabsf(r.cyclicLongitudinal) < 0.8f && fabsf(r.cyclicLateral) < 0.8f);
    CHECK(r.collective < 0.9f);
    CHECK(r.powerRequired < 0.8f * t.helicopter.engine().description().maxPower);
    CHECK_NEAR(t.body.velocity().z, 50.0f, 1e-3);
    CHECK_NEAR(t.helicopter.telemetry().trueAirspeed, 50.0f, 1e-3);

    // A 2 m/s climb at the same speed costs about weight times climb rate
    // more (a bit less: the climb also eases the induced power).
    HelicopterTrimTarget climb = cruise;
    climb.climbRate = 2.0f;
    const HelicopterTrimResult c = t.helicopter.trim(climb);
    CHECK(c.converged);
    const f32 extra = c.powerRequired - r.powerRequired;
    const f32 potential = t.body.mass() * 9.81f * 2.0f;
    CHECK(extra > 0.7f * potential && extra < 1.3f * potential);
}

void altitude()
{
    Trimmed low;
    Trimmed high;
    high.helicopter.ownAtmosphere().setOriginAltitude(2200.0f);
    HelicopterTrimTarget hover;
    const HelicopterTrimResult a = low.helicopter.trim(hover);
    const HelicopterTrimResult b = high.helicopter.trim(hover);
    printf("hover out of ground effect at 300 m %.1f kW, at 2500 m %.1f kW\n",
           double(a.powerRequired / 1000.0f), double(b.powerRequired / 1000.0f));
    CHECK(a.converged && b.converged);
    CHECK(b.collective > a.collective);
    CHECK(b.powerRequired > 1.05f * a.powerRequired);

    // In ground effect, a metre above the pad (the ground plane is at y = 0):
    // much less power, and the hover trim still converges.
    Trimmed pad(low.layout.restHeight + 1.0f);
    const HelicopterTrimResult c = pad.helicopter.trim(hover);
    printf("hover 1 m above the ground %.1f kW (ground effect factor %.3f)\n",
           double(c.powerRequired / 1000.0f), double(pad.helicopter.mainRotor().loads().groundEffectFactor));
    CHECK(c.converged);
    CHECK(c.powerRequired < 0.9f * a.powerRequired);
    CHECK(pad.helicopter.mainRotor().loads().groundEffectFactor > 1.1f);
}

// In hover the rotor's wake beats down on the cabin roof and the
// stabiliser: the rotor carries a few percent more than the weight. At
// cruise the wake is blown back clear of them.
void wakeDownload()
{
    Trimmed with;
    Trimmed without;
    without.helicopter.setRotorWakeOnAirframe(false);
    HelicopterTrimTarget hover;
    CHECK(with.helicopter.trim(hover).converged);
    CHECK(without.helicopter.trim(hover).converged);
    const f32 ratio = with.helicopter.mainRotor().loads().thrust / without.helicopter.mainRotor().loads().thrust;
    printf("hover download: the rotor carries %.1f%% more with its wake on the airframe\n",
           double(100.0f * (ratio - 1.0f)));
    CHECK(ratio > 1.02f && ratio < 1.08f);
    HelicopterTrimTarget cruise;
    cruise.airspeed = 50.0f;
    CHECK(with.helicopter.trim(cruise).converged);
    CHECK(without.helicopter.trim(cruise).converged);
    const f32 fast = with.helicopter.mainRotor().loads().thrust / without.helicopter.mainRotor().loads().thrust;
    CHECK(fabsf(fast - 1.0f) < 0.01f);
}

void trimRefusesNonsense()
{
    Trimmed t;
    const KM::vec3 before = t.body.position();
    HelicopterTrimTarget bad;
    bad.airspeed = NAN;
    CHECK(!t.helicopter.trim(bad).converged);
    HelicopterTrimTarget fast;
    fast.airspeed = 120.0f;
    const HelicopterTrimResult r = t.helicopter.trim(fast);
    CHECK(!r.converged);
    CHECK(t.body.position().y == before.y);
    CHECK(KM::length(t.body.velocity()) == 0.0f);
}
}

int main()
{
    hoverTrim();
    powerBucket();
    cruiseTrim();
    altitude();
    wakeDownload();
    trimRefusesNonsense();
    return TEST_RESULT();
}
