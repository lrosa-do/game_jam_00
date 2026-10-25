#include "actuator_rig.h"
#include "check.h"

#include <actuators/Hook.h>
#include <dynamics/Joint.h>

#include <math.h>
#include <stdio.h>

using namespace actuator_test;

namespace
{

struct Yard
{
    Rig rig;
    RigidBody* ground = nullptr;
    RigidBody* hook = nullptr;
    RigidBody* load = nullptr;
    u32 hookId = 0;
    u32 loadId = 0;

    Yard() : rig(60.0f, true)
    {
        ground = &rig.base(KM::vec3(0.0f, -0.5f, 0.0f), true, KM::vec3(10.0f, 0.5f, 10.0f));
        load = &rig.box(KM::vec3(0.0f, 0.5f, 0.0f), KM::vec3(0.5f), 200.0f, true);
        hook = &rig.box(KM::vec3(0.0f, 1.3f, 0.0f), KM::vec3(0.1f, 0.3f, 0.1f), 10.0f, true);
        hook->setBodyType(BodyType::Kinematic);
        hookId = rig.idOf(*hook);
        loadId = rig.idOf(*load);
        rig.world.markStaticBroadphaseDirty();
    }
};

void attachesWithoutAJump()
{
    for (int fixed = 0; fixed < 2; ++fixed)
    {
        Yard y;
        y.rig.run(1.0f);
        Hook hook(y.rig.world, y.hookId, KM::vec3(0.0f, 1.0f, 0.0f));
        const KM::vec3 before = y.load->position();
        CHECK(hook.attach(y.loadId, fixed ? HookMode::Fixed : HookMode::Point));
        CHECK(hook.attached());
        CHECK(hook.load() == y.load);
        CHECK(y.rig.world.pairIgnored(y.hookId, y.loadId));
        y.rig.world.step(y.rig.step);
        const f32 jump = KM::length(y.load->position() - before);
        printf("%s: moved %.2e m and at %.2e m/s the step after attaching\n", fixed ? "fixed" : "point",
               double(jump), double(KM::length(y.load->velocity())));
        CHECK(jump < 1.0e-3f);
        CHECK(KM::length(y.load->velocity()) < 0.05f);

        y.hook->setVelocity(KM::vec3(0.0f, 0.5f, 0.0f));
        y.rig.run(2.0f);
        printf("  lifted to %.4f m (1.5), at %.4f m/s, hook pulls %.1f N\n", double(y.load->position().y),
               double(y.load->velocity().y), double(hook.loadForce().y));
        CHECK_NEAR(y.load->position().y, 1.5f, 0.02f);
        CHECK_NEAR(y.load->velocity().y, 0.5f, 0.01f);
        CHECK_NEAR(hook.loadForce().y, -200.0f * kG, 0.02f * 200.0f * kG);
        if (fixed)
        {
            y.hook->setVelocity(KM::vec3(0.0f));
            y.hook->setAngularVelocity(KM::vec3(0.0f, 1.0f, 0.0f));
            y.rig.run(0.5f);
            CHECK_NEAR(y.load->angularVelocity().y, 1.0f, 0.02f);
        }

        y.hook->setVelocity(KM::vec3(0.0f));
        y.hook->setAngularVelocity(KM::vec3(0.0f));
        hook.detach();
        CHECK(!hook.attached());
        CHECK(!y.rig.world.pairIgnored(y.hookId, y.loadId));
        y.rig.run(0.2f);
        printf("  released: falling at %.4f m/s after 0.2 s\n", double(-y.load->velocity().y));
        CHECK(y.load->velocity().y < -1.5f);
    }
}

void captureFindsTheNearest()
{
    Rig rig(60.0f, false);
    RigidBody& crane = rig.base(KM::vec3(0.0f, 5.0f, 0.0f), true, KM::vec3(0.2f));
    // A small box 1.0 m below the hook (surface 0.8 m away), a beam whose
    // end is 0.5 m away (centre 3 m away), and one out of reach.
    RigidBody& box = rig.box(KM::vec3(0.0f, 4.0f, 0.0f), KM::vec3(0.2f), 10.0f, true);
    RigidBody& beam = rig.box(KM::vec3(3.0f, 5.0f, 0.0f), KM::vec3(2.5f, 0.2f, 0.2f), 50.0f, true);
    rig.box(KM::vec3(0.0f, 5.0f, 4.0f), KM::vec3(0.2f), 10.0f, true);
    rig.base(KM::vec3(-0.3f, 5.0f, 0.0f), true, KM::vec3(0.2f));
    rig.world.markStaticBroadphaseDirty();
    Hook hook(rig.world, rig.idOf(crane), KM::vec3(0.0f, 5.0f, 0.0f));
    CHECK(hook.findNearest(0.3f) == 0xFFFFFFFFu);
    CHECK(hook.findNearest(0.6f) == rig.idOf(beam));
    CHECK(hook.findNearest(2.0f) == rig.idOf(beam));
    rig.world.body(rig.idOf(beam))->filter.group = 1u << 3;
    QueryFilter withoutBeams;
    withoutBeams.collision.mask = ~(1u << 3);
    CHECK(hook.findNearest(2.0f, withoutBeams) == rig.idOf(box));
    CHECK(hook.attachNearest(2.0f, HookMode::Point, withoutBeams) == rig.idOf(box));
    CHECK(hook.loadId() == rig.idOf(box));
    CHECK(hook.attachNearest(2.0f) == 0xFFFFFFFFu);
    hook.detach();
}

struct CallbackScript
{
    Yard* yard = nullptr;
    Hook* hook = nullptr;
    int step = 0;
    u32 attachedAt = 0;
    f32 heightAfterRelease = 0.0f;
    bool removed = false;
};

void scriptedStep(f32 /*duration*/, void* user)
{
    CallbackScript& s = *static_cast<CallbackScript*>(user);
    ++s.step;
    if (s.step == 30)
        s.attachedAt = s.hook->attachNearest(0.5f);
    if (s.step == 31)
        s.yard->hook->setVelocity(KM::vec3(0.0f, 1.0f, 0.0f));
    if (s.step == 90)
    {
        s.yard->hook->setVelocity(KM::vec3(0.0f));
        s.hook->detach();
    }
    if (s.step == 91)
        s.heightAfterRelease = s.yard->load->position().y;
    if (s.step == 150)
        s.hook->attach(s.yard->loadId);
    if (s.step == 160 && s.hook->attached())
    {
        s.yard->rig.world.removeBody(s.yard->loadId);
        s.removed = true;
    }
    if (s.step == 161)
        s.hook->detach();
}

void safeFromTheStepCallback()
{
    Yard y;
    Hook hook(y.rig.world, y.hookId, KM::vec3(0.0f, 1.0f, 0.0f));
    CallbackScript script;
    script.yard = &y;
    script.hook = &hook;
    y.rig.world.setStepCallback(scriptedStep, &script);
    y.rig.run(1.0f);
    CHECK(script.attachedAt == y.loadId);
    CHECK(hook.attached());
    y.rig.run(1.0f);
    printf("callback: lifted to %.3f m by the release, then at %.3f m/s back on the ground\n",
           double(script.heightAfterRelease), double(-y.load->velocity().y));
    CHECK(script.heightAfterRelease > 1.3f);
    CHECK(!hook.attached());
    y.rig.run(1.0f);
    CHECK(script.removed);
    CHECK(!hook.attached());
    CHECK(hook.load() == nullptr);
    CHECK(y.rig.world.jointCount() == 0);
    y.rig.world.setStepCallback(nullptr, nullptr);
    y.rig.run(0.5f);
    CHECK(finiteVec(y.hook->position()));
}

void destructorDetaches()
{
    Yard y;
    {
        Hook hook(y.rig.world, y.hookId, KM::vec3(0.0f, 1.0f, 0.0f));
        CHECK(hook.attach(y.loadId));
        CHECK(y.rig.world.jointCount() == 1);
        y.rig.run(0.1f);
    }
    CHECK(y.rig.world.jointCount() == 0);
    CHECK(!y.rig.world.pairIgnored(y.hookId, y.loadId));
    Hook hook(y.rig.world, y.hookId, KM::vec3(0.0f, 1.0f, 0.0f));
    CHECK(!hook.attach(y.rig.idOf(*y.ground)));
    CHECK(!hook.attach(y.hookId));
    CHECK(!hook.attach(12345u));
    CHECK(!hook.attached());
}

}

int main()
{
    attachesWithoutAJump();
    captureFindsTheNearest();
    safeFromTheStepCallback();
    destructorDetaches();
    return TEST_RESULT();
}
