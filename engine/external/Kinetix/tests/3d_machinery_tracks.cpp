#include "check.h"
#include "machinery_rig.h"

#include <dynamics/SliderJoint.h>
#include <machinery/Terramechanics.h>
#include <machinery/TrackedUndercarriage.h>

using namespace machinery_test;

namespace
{

constexpr f32 kHullMass = 20000.0f;

TrackParameters tracks(u32 samples = 16)
{
    TrackParameters p;
    p.gauge = 2.2f;
    p.shoeWidth = 0.6f;
    p.contactLength = 3.6f;
    p.bottomHeight = -0.8f;
    p.samples = samples;
    p.beltMass = 1200.0f;
    p.internalResistance = 0.0f;
    return p;
}

struct Machine
{
    RigidBody* hull = nullptr;
    u32 hullId = 0;
    TrackedUndercarriage* undercarriage = nullptr;
    Scene* scene = nullptr;

    Machine(Scene& s, const KM::vec3& position, const KM::quat& orientation, const SoilParameters& soil,
            const TrackParameters& p = tracks())
        : scene(&s)
    {
        hull = &s.box(position, KM::vec3(1.4f, 0.4f, 2.2f), kHullMass, false, &hullId);
        hull->setOrientation(orientation);
        hull->calculateDerivedData();
        undercarriage = new TrackedUndercarriage(s.world, hullId, p);
        undercarriage->setSoil(soil);
        s.world.addJoint(undercarriage);
    }
    ~Machine()
    {
        scene->world.removeJoint(undercarriage);
        delete undercarriage;
    }
    void drive(f32 left, f32 right, f32 force = 2.0e5f)
    {
        undercarriage->drive(TrackedUndercarriage::Left).setVelocity(left, force);
        undercarriage->drive(TrackedUndercarriage::Right).setVelocity(right, force);
    }
    f32 meanSinkage() const
    {
        return 0.5f * (undercarriage->side(0).meanSinkage + undercarriage->side(1).meanSinkage);
    }
};

// Static sinkage: W over two contact patches b x L, z = (p / (kc/b + kphi))^(1/n).
void staticSinkage()
{
    struct Case
    {
        const char* name;
        SoilParameters soil;
    };
    const Case cases[] = {{"dry sand", SoilParameters::drySand()},
                          {"loam", SoilParameters::loam()},
                          {"clay", SoilParameters::clay()},
                          {"snow", SoilParameters::snow()},
                          {"hard", SoilParameters::hardSurface(0.7f)}};
    const TrackParameters p = tracks();
    for (const Case& c : cases)
    {
        Scene scene;
        Machine m(scene, KM::vec3(0.0f, 0.82f, 0.0f), KM::quat(), c.soil);
        scene.run(6.0f);
        const f32 pressure = kHullMass * kG / (2.0f * p.shoeWidth * p.contactLength);
        const f32 expected = Terramechanics::sinkage(c.soil, pressure, p.shoeWidth);
        const f32 sinkage = m.meanSinkage();
        f32 spread = 0.0f;
        for (u32 side = 0; side < 2; ++side)
            for (u32 k = 0; k < m.undercarriage->sampleCount(); ++k)
                spread = fmaxf(spread, fabsf(m.undercarriage->sample(side, k).sinkage - expected));
        const f32 load = m.undercarriage->side(0).normalLoad + m.undercarriage->side(1).normalLoad;
        printf("%-8s: sinkage %.4f m (Bekker %.4f, %+.2f%%), worst sample off by %.2e m; load %.0f N (W %.0f); "
               "pressure %.1f kPa\n",
               c.name, double(sinkage), double(expected), double(100.0f * (sinkage / expected - 1.0f)),
               double(spread), double(load), double(kHullMass * kG), double(pressure * 1.0e-3f));
        CHECK_NEAR(sinkage, expected, 0.05f * expected);
        CHECK_NEAR(load, kHullMass * kG, 0.005f * kHullMass * kG);
        CHECK(finiteVec(m.hull->position()));
        CHECK(fabsf(m.hull->velocity().y) < 1.0e-3f);
    }
}

// Dynamometer: a carriage at the ground speed holds the hull through a vertical slider while the belts
// run faster (a set slip); the thrust is compared with H(i).
void tractiveEffortCurve()
{
    const SoilParameters soil = SoilParameters::loam();
    const TrackParameters p = tracks(24);
    const f32 slips[] = {0.02f, 0.05f, 0.1f, 0.2f, 0.4f, 0.7f, 0.9f};
    f32 previous = 0.0f;
    bool monotonic = true;
    f32 saturated = 0.0f;
    f32 maximum = 0.0f;
    for (f32 slip : slips)
    {
        Scene scene;
        Machine m(scene, KM::vec3(0.0f, 0.81f, 0.0f), KM::quat(), soil, p);
        u32 carriageId = 0;
        RigidBody& carriage = scene.box(KM::vec3(0.0f, 0.81f, 0.0f), KM::vec3(0.2f), 1.0f, false, &carriageId);
        carriage.setBodyType(BodyType::Kinematic);
        SliderJoint dyno(carriage, *m.hull, KM::vec3(0.0f, 0.81f, 0.0f), KM::vec3(0.0f, 1.0f, 0.0f));
        scene.world.addJoint(&dyno);
        scene.run(2.0f);
        const f32 groundSpeed = 0.5f;
        const f32 beltSpeed = groundSpeed / (1.0f - slip);
        carriage.setVelocity(KM::vec3(0.0f, 0.0f, groundSpeed));
        m.drive(beltSpeed, beltSpeed, 1.0e6f);
        scene.run(10.0f);
        f32 thrust = 0.0f;
        f32 load = 0.0f;
        f32 measuredSlip = 0.0f;
        for (u32 side = 0; side < 2; ++side)
        {
            thrust += m.undercarriage->side(side).thrust;
            load += m.undercarriage->side(side).normalLoad;
            measuredSlip += 0.5f * m.undercarriage->side(side).slip;
        }
        const f32 expected = 2.0f * Terramechanics::tractiveEffort(soil, 0.5f * load, p.shoeWidth, p.contactLength, slip);
        maximum = 2.0f * Terramechanics::maxTractiveEffort(soil, 0.5f * load, p.shoeWidth, p.contactLength);
        printf("slip %.2f (measured %.4f): thrust %.0f N, Janosi-Hanamoto %.0f N (%+.2f%%), of A c + W tan phi = %.0f\n",
               double(slip), double(measuredSlip), double(thrust), double(expected),
               double(100.0f * (thrust / expected - 1.0f)), double(maximum));
        CHECK_NEAR(measuredSlip, slip, 2.0e-3f);
        CHECK_NEAR(thrust, expected, 0.03f * expected + 200.0f);
        monotonic = monotonic && thrust > previous;
        previous = thrust;
        saturated = thrust;
        scene.world.removeJoint(&dyno);
    }
    CHECK(monotonic);
    CHECK(saturated > 0.95f * maximum);
    CHECK(saturated <= maximum * 1.005f);
}

void pivotTurnAndStraightRun()
{
    Scene scene;
    Machine m(scene, KM::vec3(0.0f, 0.802f, 0.0f), KM::quat(), SoilParameters::hardSurface(0.7f));
    scene.run(1.0f);
    const KM::vec3 start = m.hull->position();
    m.drive(0.4f, -0.4f);
    f32 heading = 0.0f;
    KM::vec3 lastForward = m.hull->directionToWorld(KM::vec3(0.0f, 0.0f, 1.0f));
    f32 drift = 0.0f;
    for (int i = 0; i < 8 * 60; ++i)
    {
        scene.world.step(scene.step);
        const KM::vec3 forward = m.hull->directionToWorld(KM::vec3(0.0f, 0.0f, 1.0f));
        heading += atan2f(KM::dot(KM::cross(lastForward, forward), KM::vec3(0.0f, 1.0f, 0.0f)),
                          KM::dot(lastForward, forward));
        lastForward = forward;
        const KM::vec3 offset = m.hull->position() - start;
        drift = fmaxf(drift, sqrtf(offset.x * offset.x + offset.z * offset.z));
    }
    const f32 yawRate = m.hull->angularVelocity().y;
    const f32 ideal = 2.0f * 0.4f / tracks().gauge;
    printf("pivot turn on concrete: %.1f deg in 8 s, yaw rate %.4f rad/s (no-slip %.4f), centre drifted %.3f m, slips "
           "%.3f / %.3f\n",
           double(heading / kDeg), double(yawRate), double(-ideal), double(drift),
           double(m.undercarriage->side(0).slip), double(m.undercarriage->side(1).slip));
    // Left forward, right back: turning right, yaw negative about +Y.
    CHECK(heading < -60.0f * kDeg);
    CHECK(fabsf(yawRate) > 0.5f * ideal);
    CHECK(fabsf(yawRate) <= ideal * 1.01f);
    CHECK(drift < 0.15f);

    m.drive(0.6f, 0.6f);
    scene.run(4.0f);
    const KM::vec3 forward = m.hull->directionToWorld(KM::vec3(0.0f, 0.0f, 1.0f));
    const f32 speed = KM::dot(m.hull->velocity(), forward);
    printf("straight on concrete: %.4f m/s at belts 0.6, slip %.4f\n", double(speed), double(m.undercarriage->side(0).slip));
    CHECK_NEAR(speed, 0.6f, 0.02f);
    CHECK(fabsf(m.hull->angularVelocity().y) < 1.0e-3f);
}

// A clay ramp: c = 4.1 kPa, phi = 13 deg; traction limit (A c + W cos t tan phi >= W sin t) is 18.1 deg.
f32 climb(f32 slope, f32* slipOut)
{
    Scene scene(60.0f, false);
    const KM::quat tilt = KM::angleAxis(-slope, KM::vec3(1.0f, 0.0f, 0.0f));
    const u32 ramp =
        scene.addStatic(new BoxShape(KM::vec3(20.0f, 1.0f, 60.0f)), tilt * KM::vec3(0.0f, -1.0f, 0.0f), tilt);
    (void)ramp;
    const KM::vec3 up = tilt * KM::vec3(0.0f, 1.0f, 0.0f);
    const KM::vec3 along = tilt * KM::vec3(0.0f, 0.0f, 1.0f);
    Machine m(scene, up * 0.81f, tilt, SoilParameters::clay());
    m.drive(0.0f, 0.0f, 5.0e5f);
    scene.run(2.0f);
    const KM::vec3 start = m.hull->position();
    m.drive(0.5f, 0.5f, 5.0e5f);
    scene.run(10.0f);
    const f32 progress = KM::dot(m.hull->position() - start, along);
    *slipOut = m.undercarriage->side(0).slip;
    CHECK(finiteVec(m.hull->position()));
    return progress;
}

void slopeTractionLimit()
{
    const SoilParameters clay = SoilParameters::clay();
    const TrackParameters p = tracks();
    const f32 area = 2.0f * p.shoeWidth * p.contactLength;
    f32 limit = 0.0f;
    for (f32 t = 0.0f; t < 40.0f * kDeg; t += 0.01f * kDeg)
    {
        const f32 w = kHullMass * kG;
        if (area * clay.cohesion + w * cosf(t) * tanf(clay.frictionAngle) < w * sinf(t))
        {
            limit = t;
            break;
        }
    }
    f32 slipLow = 0.0f;
    f32 slipHigh = 0.0f;
    const f32 low = climb(8.0f * kDeg, &slipLow);
    f32 slipNear = 0.0f;
    const f32 near = climb(15.0f * kDeg, &slipNear);
    const f32 high = climb(22.0f * kDeg, &slipHigh);
    printf("clay slope (traction limit %.1f deg): 8 deg climbs %.2f m in 10 s (slip %.3f), 22 deg %.2f m (slip %.3f)\n",
           double(limit / kDeg), double(low), double(slipLow), double(high), double(slipHigh));
    printf("   15 deg, just inside the limit: %.2f m (slip %.3f)\n", double(near), double(slipNear));
    CHECK(limit > 12.0f * kDeg && limit < 22.0f * kDeg);
    CHECK(near > 0.5f);
    CHECK(slipNear > slipLow);
    CHECK(low > 3.0f);
    CHECK(slipLow < 0.25f);
    CHECK(high < 0.0f);
    CHECK(slipHigh > 0.9f);
}

}

int main()
{
    staticSinkage();
    tractiveEffortCurve();
    pivotTurnAndStraightRun();
    slopeTractionLimit();
    return TEST_RESULT();
}
