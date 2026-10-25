#ifndef KINETIX_TESTS_MACHINERY_STABILITY_H
#define KINETIX_TESTS_MACHINERY_STABILITY_H

#include "check.h"
#include "machinery_rig.h"

#include <machinery/Bulldozer.h>
#include <machinery/Excavator.h>
#include <machinery/MobileCrane.h>

#include <stdio.h>

namespace machinery_test
{

inline f32 lever(f32 time, f32 period, f32 phase = 0.0f)
{
    const f32 s = sinf(2.0f * kPi * (time / period + phase));
    return KM::clamp(1.6f * s, -1.0f, 1.0f);
}

struct StabilityResult
{
    bool finite = true;
    f32 worstSpeed = 0.0f;
    f32 worstSeparation = 0.0f;
    f32 worstTilt = 0.0f;
};

inline void track(StabilityResult& r, const RigidBody& body)
{
    const KM::vec3& v = body.velocity();
    const KM::vec3& w = body.angularVelocity();
    if (!finiteVec(body.position()) || !finiteVec(v) || !finiteVec(w))
        r.finite = false;
    r.worstSpeed = fmaxf(r.worstSpeed, KM::length(v));
}

inline f32 tiltOf(const RigidBody& body)
{
    const KM::vec3 up = body.directionToWorld(KM::vec3(0.0f, 1.0f, 0.0f));
    return acosf(KM::clamp(up.y, -1.0f, 1.0f));
}

inline void report(const char* name, f32 hz, const StabilityResult& r)
{
    printf("%s at %.0f Hz for 60 s: finite %d, fastest body %.2f m/s, worst joint opening %.2e m, worst tilt %.2f deg\n",
           name, double(hz), int(r.finite), double(r.worstSpeed), double(r.worstSeparation), double(r.worstTilt / kDeg));
}

inline void excavatorStability(f32 hz)
{
    Scene scene(hz);
    Excavator e(scene.world, ExcavatorParameters::crawler20t(), KM::vec3(0.0f, 0.745f, 0.0f));
    e.setGroundSoil(SoilParameters::loam());
    StabilityResult r;
    const int steps = static_cast<int>(60.0f * hz + 0.5f);
    for (int i = 0; i < steps; ++i)
    {
        const f32 t = static_cast<f32>(i) * scene.step;
        ExcavatorControls c;
        // The boom mostly up and the stick not fully out, so the bucket
        // works the air rather than levering the machine off the ground.
        c.boom = 0.4f + 0.6f * lever(t, 7.0f);
        c.stick = 0.7f * lever(t, 5.0f, 0.3f);
        c.bucket = lever(t, 3.0f, 0.1f);
        c.swing = lever(t, 11.0f);
        c.leftTrack = 0.6f * lever(t, 13.0f);
        c.rightTrack = 0.6f * lever(t, 17.0f, 0.25f);
        e.setControls(c);
        e.update(scene.step);
        scene.world.step(scene.step);
        for (u32 b = 0; b < Excavator::BodyCount; ++b)
            track(r, e.body(b));
        r.worstSeparation = fmaxf(r.worstSeparation, e.worstJointSeparation());
        r.worstTilt = fmaxf(r.worstTilt, tiltOf(e.undercarriage()));
    }
    report("excavator", hz, r);
    CHECK(r.finite);
    CHECK(e.finite());
    CHECK(r.worstSpeed < 20.0f);
    CHECK(r.worstSeparation < 1.0e-3f);
    CHECK(r.worstTilt < 15.0f * kDeg);
}

inline void craneStability(f32 hz)
{
    Scene scene(hz);
    MobileCraneParameters p = MobileCraneParameters::allTerrain50t();
    p.buildDeployed = true;
    p.buildElevation = 0.9f;
    p.buildExtension = 2.0f;
    MobileCrane crane(scene.world, p, KM::vec3(0.0f, p.chassisHeight, 0.0f));
    for (int i = 0; i < static_cast<int>(0.5f * hz); ++i)
    {
        crane.update(scene.step);
        scene.world.step(scene.step);
    }
    u32 loadId = 0;
    const KM::vec3 hook = crane.hook().pointWorld();
    RigidBody& load = scene.box(hook - KM::vec3(0.0f, 0.5f, 0.0f), KM::vec3(0.5f), 3000.0f, true, &loadId);
    crane.hook().attach(loadId, HookMode::Point);
    StabilityResult r;
    const int steps = static_cast<int>(60.0f * hz + 0.5f);
    for (int i = 0; i < steps; ++i)
    {
        const f32 t = static_cast<f32>(i) * scene.step;
        MobileCraneControls c = crane.controls();
        c.slew = 0.5f * lever(t, 19.0f);
        c.luff = 0.5f * lever(t, 13.0f, 0.5f);
        c.telescope = lever(t, 23.0f, 0.5f);
        c.hoist = 0.5f * lever(t, 7.0f, 0.5f);
        crane.setControls(c);
        crane.update(scene.step);
        scene.world.step(scene.step);
        for (u32 b = 0; b < MobileCrane::BodyCount; ++b)
            track(r, crane.body(b));
        track(r, load);
        r.worstSeparation = fmaxf(r.worstSeparation, crane.worstJointSeparation());
        r.worstTilt = fmaxf(r.worstTilt, crane.tilt());
    }
    report("crane", hz, r);
    CHECK(r.finite);
    CHECK(crane.finite());
    CHECK(r.worstSpeed < 20.0f);
    CHECK(r.worstSeparation < 1.0e-3f);
    CHECK(r.worstTilt < 2.0f * kDeg);
}

inline void bulldozerStability(f32 hz)
{
    Scene scene(hz);
    const BulldozerParameters p = BulldozerParameters::crawler18t();
    Bulldozer d(scene.world, p, KM::vec3(0.0f, p.rideHeight, 0.0f));
    d.setGroundSoil(SoilParameters::loam());
    for (int i = 0; i < 4; ++i)
        scene.box(KM::vec3(-1.5f + 1.0f * static_cast<f32>(i), 0.3f, 5.0f), KM::vec3(0.3f), 300.0f, true);
    StabilityResult r;
    const int steps = static_cast<int>(60.0f * hz + 0.5f);
    for (int i = 0; i < steps; ++i)
    {
        const f32 t = static_cast<f32>(i) * scene.step;
        BulldozerControls c;
        c.lift = lever(t, 6.0f, 0.5f);
        c.tilt = lever(t, 4.0f);
        c.leftTrack = lever(t, 15.0f, 0.1f);
        c.rightTrack = lever(t, 11.0f, 0.1f);
        d.setControls(c);
        d.update(scene.step);
        scene.world.step(scene.step);
        for (u32 b = 0; b < Bulldozer::BodyCount; ++b)
            track(r, d.body(b));
        r.worstSeparation = fmaxf(r.worstSeparation, d.worstJointSeparation());
        r.worstTilt = fmaxf(r.worstTilt, tiltOf(d.tractor()));
    }
    report("bulldozer", hz, r);
    CHECK(r.finite);
    CHECK(d.finite());
    CHECK(r.worstSpeed < 20.0f);
    CHECK(r.worstSeparation < 1.0e-3f);
    CHECK(r.worstTilt < 20.0f * kDeg);
}

}

#endif // KINETIX_TESTS_MACHINERY_STABILITY_H
