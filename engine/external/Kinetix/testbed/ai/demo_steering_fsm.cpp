#include "scene.h"

#include "render_ai.h"

#include <Action.h>
#include <Entity.h>
#include <Group.h>
#include <State.h>
#include <StateMachine.h>
#include <Steering.h>
#include <Transition.h>
#include <World.h>

#include <GL/gl.h>

#include <math.h>
#include <stdio.h>

namespace testbed
{

using namespace kinetix::AI;

using kinetix::f32;
namespace M = kinetix::Math;

namespace
{
struct HunterContext
{
    Entity* hunter = nullptr;
    M::vec3* targetPosition = nullptr;
    SteerBehavior* seek = nullptr;
    SteerBehavior* flee = nullptr;
    WanderBehavior* wander = nullptr;
    class HoldBehavior* hold = nullptr;
    // Set by HoldBehavior; the Hold -> Flee transition only reads it.
    f32 distanceOnHoldEnter = 0.0f;

    f32 distanceToTarget() const
    {
        return M::distance(hunter->position(), *targetPosition);
    }
};

// Arrival-aware seek/flee: SteerLibrary::seek()/flee() never ease off near the target.
// `sign` is +1 to seek, -1 to flee; `ease` is where the speed ramp starts.
SteerBehavior::SteerFunc buildArrival(const M::vec3* target, f32 ease, f32 sign)
{
    return [target, ease, sign](SteerLibrary& steer, float) -> M::vec3 {
        const Entity& vehicle = steer.vehicle();
        M::vec3 toTarget = (*target - vehicle.position()) * sign;
        const f32 distance = M::length(toTarget);
        if (distance < 1e-4f)
            return M::vec3(0.0f);

        const f32 speedScale = distance < ease ? distance / ease : 1.0f;
        const M::vec3 desiredVelocity = (toTarget / distance) * vehicle.maxSpeed() * speedScale;
        return desiredVelocity - vehicle.velocity();
    };
}

// Brakes the entity and tracks whether the target is closing in since Hold was entered.
// Must be a real Behavior: Entity::iterate() zeroes desiredMove each call, so a State action
// running from StateMachine::iterate() (after World::iterate()) would be discarded.
class HoldBehavior final : public Behavior
{
public:
    HoldBehavior(HunterContext& context, f32 gracePeriod) : mContext(context), mGracePeriod(gracePeriod)
    {
    }

    void onEnter()
    {
        mContext.distanceOnHoldEnter = mContext.distanceToTarget();
        mElapsedInHold = 0.0f;
    }

    void iterate(float timeDelta, Entity& entity) override
    {
        entity.setDesiredMove(entity.desiredMove() - entity.velocity());
        mElapsedInHold += timeDelta;
        // Slide the baseline forward through the grace period: on entry the hunter still carries
        // Seek's speed and Entity has no damping, so an entry-only baseline would read momentum as closing in.
        if (mElapsedInHold < mGracePeriod)
            mContext.distanceOnHoldEnter = mContext.distanceToTarget();
    }

    const char* name() const override { return "Hold Behavior"; }

private:
    HunterContext& mContext;
    f32 mGracePeriod;
    f32 mElapsedInHold = 0.0f;
};
} // namespace

// One entity, four states: Wander when the target is far, Seek once in range, Hold once close
// enough to arrive, Flee if the target keeps closing in while held. Exactly one steering
// behavior drives the entity at a time (no blending).
//
// Hold exists because arrival alone leaves a stationary target's hunter orbiting forever
// (Seek eases in, crosses Flee's threshold, Flee kicks it out, repeat); proximity alone is never
// a reason to flee.
class SteeringFSM final : public Scene
{
public:
    SteeringFSM()
    {
        mGroup = new Group(mWorld);
        mWorld.add(*mGroup);

        Entity::Settings settings;
        settings.maxSpeed = 6.0f;
        settings.desiredSpeed = 4.0f;
        settings.maxVelocityChange = 8.0f;
        settings.senseRange = 30.0f;
        mHunter = new Entity(mWorld, settings);
        mHunter->setPosition(M::vec3(-10.0f, 0.0f, 0.0f));
        mGroup->add(*mHunter);

        // Outside kSeekRange so the demo opens in Wander.
        mTargetPosition = M::vec3(10.0f, 0.0f, 0.0f);

        mContext.hunter = mHunter;
        mContext.targetPosition = &mTargetPosition;
        mContext.seek = new SteerBehavior(buildArrival(&mTargetPosition, kSeekArrivalRadius, 1.0f));
        mContext.flee = new SteerBehavior(buildArrival(&mTargetPosition, kFleeEase, -1.0f));
        mContext.wander = new WanderBehavior();
        mContext.hold = new HoldBehavior(mContext, kHoldGracePeriod);

        buildStateMachine();

        mCamera.target = M::vec3(0.0f, 0.0f, 0.0f);
        mCamera.pitch = 0.9f;
        mCamera.distance = 24.0f;

        addCheck("starts wandering with the target far away");
        addCheck("switches to seek once the target is in range");
        addCheck("settles into hold instead of orbiting forever");
        addCheck("switches to flee if the target keeps closing in");
    }

    ~SteeringFSM() override
    {
        delete mFSM;
        delete mContext.seek;
        delete mContext.flee;
        delete mContext.wander;
        delete mContext.hold;
    }

    const char* name() const override { return "AI: steering FSM"; }
    const char* description() const override
    {
        return "One entity, four states, three Steering.h-shaped behaviors: it wanders\n"
               "when alone, seeks the target once in range, holds position once close\n"
               "enough, and flees if the target keeps closing in while held. Left-click\n"
               "and drag the orange marker.";
    }

    bool is3D() const override { return true; }
    Camera3D camera3D() const override { return mCamera; }

    void handleInput(const Input& input) override
    {
        // Intersect the pick ray with the ground plane (y = 0); the target is not a physics body.
        if (input.grabPressed)
            mDraggingTarget = true;
        if (input.grabReleased)
            mDraggingTarget = false;

        if (mDraggingTarget && fabsf(input.rayDirection.y) > 1.0e-4f)
        {
            const f32 t = -input.rayOrigin.y / input.rayDirection.y;
            if (t > 0.0f)
                mTargetPosition = input.rayOrigin + input.rayDirection * t;
        }
    }

    void step(float dt) override
    {
        // seek/flee are buildArrival() lambdas capturing &mTargetPosition; no per-frame push needed.
        mWorld.iterate(dt);
        mFSM->iterate();
        mElapsed += dt;

        if (mFSM->currentState() == mWanderState)
            mSeenWander = true;
        else if (mFSM->currentState() == mSeekState)
            mSeenSeek = true;
        else if (mFSM->currentState() == mHoldState)
            mSeenHold = true;
        else if (mFSM->currentState() == mFleeState)
            mSeenFlee = true;

        // Longest uninterrupted stay in Hold, to tell "settled" from "passing through".
        if (mFSM->currentState() == mHoldState)
        {
            mHoldStreak += dt;
            if (mHoldStreak > mLongestHoldStreak)
                mLongestHoldStreak = mHoldStreak;
        }
        else
        {
            mHoldStreak = 0.0f;
        }

        // No behavior calls alignWithVelocity(), so forward() never turns; blend the drawn heading
        // against velocity() instead.
        const M::vec3 v = mHunter->velocity();
        if (M::length(v) > 0.05f)
        {
            const float blend = 1.0f - powf(0.001f, dt);
            mHeading = M::normalize(mHeading + (M::normalize(v) - mHeading) * blend);
        }
    }

    void draw() override
    {
        setupLightingAI();
        drawGroundPlane();

        AgentStyle style;
        style.radius = 0.6f;
        if (mFSM->currentState() == mSeekState)
            style.r = 0.3f, style.g = 0.9f, style.b = 0.4f;
        else if (mFSM->currentState() == mHoldState)
            style.r = 0.9f, style.g = 0.8f, style.b = 0.25f;
        else if (mFSM->currentState() == mFleeState)
            style.r = 0.95f, style.g = 0.3f, style.b = 0.3f;
        else
            style.r = 0.5f, style.g = 0.6f, style.b = 0.95f;
        drawAgent(mHunter->position(), mHeading, style);
        drawVelocity(mHunter->position(), mHunter->velocity());

        drawGoalMarker(mTargetPosition, mElapsed);
    }

    void evaluate() override
    {
        char buf[80];
        snprintf(buf, sizeof(buf), "state: %s", mFSM->currentState() ? mFSM->currentState()->name().c_str() : "?");
        // Only checked at the start, before Wander can carry the hunter anywhere.
        if (mElapsed > 0.05f && mFirstState.empty())
            mFirstState = mFSM->currentState() ? mFSM->currentState()->name() : ct::String();
        report(0, mFirstState.empty() || mFirstState == "Wander", buf);

        // Never fail, only confirm reachability.
        if (mSeenSeek)
            report(1, true, "seek reached");
        if (mSeenFlee)
            report(3, true, "flee reached");

        // With the target never dragged the hunter must sit in Hold, not just pass through. Judged
        // only after 10 s: a check that fails once stays failed, and the streak starts at 0.
        snprintf(buf, sizeof(buf), "longest hold streak %.1fs", double(mLongestHoldStreak));
        if (mElapsed > 10.0f)
            report(2, mLongestHoldStreak > 2.0f, buf);
    }

private:
    void buildStateMachine()
    {
        mFSM = new StateMachine();

        mWanderState = new State("Wander");
        mSeekState = new State("Seek");
        mHoldState = new State("Hold");
        mFleeState = new State("Flee");
        mFSM->addState(mWanderState);
        mFSM->addState(mSeekState);
        mFSM->addState(mHoldState);
        mFSM->addState(mFleeState);

        HunterContext* ctx = &mContext;
        Entity* hunter = mHunter;

        mWanderState->addEnterAction(new CallbackAction(mWanderState, [hunter, ctx](State&) {
            hunter->addBehavior(*ctx->wander);
        }));
        mWanderState->addExitAction(new CallbackAction(mWanderState, [hunter, ctx](State&) {
            hunter->removeBehavior(*ctx->wander);
        }));

        mSeekState->addEnterAction(new CallbackAction(mSeekState, [hunter, ctx](State&) {
            hunter->addBehavior(*ctx->seek);
        }));
        mSeekState->addExitAction(new CallbackAction(mSeekState, [hunter, ctx](State&) {
            hunter->removeBehavior(*ctx->seek);
        }));

        mHoldState->addEnterAction(new CallbackAction(mHoldState, [hunter, ctx](State&) {
            hunter->addBehavior(*ctx->hold);
            ctx->hold->onEnter();
        }));
        mHoldState->addExitAction(new CallbackAction(mHoldState, [hunter, ctx](State&) {
            hunter->removeBehavior(*ctx->hold);
        }));

        mFleeState->addEnterAction(new CallbackAction(mFleeState, [hunter, ctx](State&) {
            hunter->addBehavior(*ctx->flee);
        }));
        mFleeState->addExitAction(new CallbackAction(mFleeState, [hunter, ctx](State&) {
            hunter->removeBehavior(*ctx->flee);
        }));

        mWanderState->addTransition(new CallbackTransition(mWanderState, mSeekState,
            [ctx](State&) { return ctx->distanceToTarget() < kSeekRange; }));

        mSeekState->addTransition(new CallbackTransition(mSeekState, mHoldState,
            [ctx](State&) { return ctx->distanceToTarget() < kHoldRadius; }));
        mSeekState->addTransition(new CallbackTransition(mSeekState, mWanderState,
            [ctx](State&) { return ctx->distanceToTarget() > kSeekRange; }));

        // Hold -> Flee: the target closed in since the baseline was last taken. During the grace
        // period the baseline slides forward, so Seek's carried-over speed never triggers it.
        mHoldState->addTransition(new CallbackTransition(mHoldState, mFleeState, [ctx](State&) {
            return ctx->distanceOnHoldEnter - ctx->distanceToTarget() > kApproachThreshold;
        }));
        mHoldState->addTransition(new CallbackTransition(mHoldState, mSeekState,
            [ctx](State&) { return ctx->distanceToTarget() > kHoldRadius; }));

        // Flee -> Hold, not Seek, so a target oscillating at the boundary re-checks from a fresh baseline.
        mFleeState->addTransition(new CallbackTransition(mFleeState, mHoldState,
            [ctx](State&) { return ctx->distanceToTarget() > kFleeExitRange; }));

        mWanderState->enter();
    }

    void drawGroundPlane() const
    {
        glDisable(GL_LIGHTING);
        glColor3f(0.16f, 0.18f, 0.2f);
        glBegin(GL_LINES);
        for (int i = -20; i <= 20; ++i)
        {
            glVertex3f(float(i), 0.0f, -20.0f);
            glVertex3f(float(i), 0.0f, 20.0f);
            glVertex3f(-20.0f, 0.0f, float(i));
            glVertex3f(20.0f, 0.0f, float(i));
        }
        glEnd();
    }

    static constexpr f32 kSeekRange = 14.0f;
    static constexpr f32 kSeekArrivalRadius = 10.0f;
    static constexpr f32 kHoldRadius = 5.0f;
    static constexpr f32 kApproachThreshold = 1.5f;
    static constexpr float kHoldGracePeriod = 0.5f;
    static constexpr f32 kFleeEase = 4.0f;
    static constexpr f32 kFleeExitRange = 6.0f;

    World mWorld;
    Group* mGroup = nullptr;
    Entity* mHunter = nullptr;
    HunterContext mContext;

    StateMachine* mFSM = nullptr;
    State* mWanderState = nullptr;
    State* mSeekState = nullptr;
    State* mHoldState = nullptr;
    State* mFleeState = nullptr;

    M::vec3 mHeading{1.0f, 0.0f, 0.0f};
    M::vec3 mTargetPosition{0.0f};
    bool mDraggingTarget = false;

    Camera3D mCamera;
    float mElapsed = 0.0f;
    bool mSeenWander = false;
    bool mSeenSeek = false;
    bool mSeenHold = false;
    bool mSeenFlee = false;
    float mHoldStreak = 0.0f;
    float mLongestHoldStreak = 0.0f;
    ct::String mFirstState;
};

REGISTER_SCENE(SteeringFSM, "AI: steering FSM", Category::AI)

} // namespace testbed
