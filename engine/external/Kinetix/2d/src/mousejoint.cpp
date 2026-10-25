#include "kx/joints/mousejoint.h"

#include "kx/body.h"

#include <math.h>

namespace kx
{

    MouseJoint::MouseJoint(Body *body, const vec2 &target, float maxForce,
                           float frequencyHz, float dampingRatio)
        : Joint(JointType::Mouse, nullptr, body),
          mTarget(target),
          mMaxForce(maxForce),
          mRB(0.0f, 0.0f),
          mC(0.0f, 0.0f),
          mImpulse(0.0f, 0.0f),
          mGamma(0.0f),
          mMass00(0.0f), mMass01(0.0f), mMass10(0.0f), mMass11(0.0f)
    {
        mLocalAnchor = InvTransformPoint(body->GetTransform(), target);

        float mass = body->Mass();
        float omega = 2.0f * kPi * frequencyHz;
        mStiffness = mass * omega * omega;
        mDamping = 2.0f * mass * dampingRatio * omega;

        // A sleeping body skips the solver, so grabbing one must wake it.
        body->SetAwake(true);
    }

    vec2 MouseJoint::AnchorB() const
    {
        return mBodyB->GetTransform().Transform(mLocalAnchor);
    }

    void MouseJoint::SetTarget(const vec2 &target)
    {
        if (target != mTarget)
            WakeBodies();
        mTarget = target;
    }

    void MouseJoint::InitVelocity(float dt)
    {
        Body *b = mBodyB;
        float invMass = b->InvMass();
        float invI = b->InvI();

        float h = dt;
        mGamma = h * (mDamping + h * mStiffness);
        if (mGamma != 0.0f)
            mGamma = 1.0f / mGamma;
        float beta = h * mStiffness * mGamma;

        Transform xf = b->GetTransform();
        mRB = xf.Transform(mLocalAnchor) - b->WorldCenter();

        float k00 = invMass + invI * mRB.y * mRB.y + mGamma;
        float k01 = -invI * mRB.x * mRB.y;
        float k11 = invMass + invI * mRB.x * mRB.x + mGamma;
        float det = k00 * k11 - k01 * k01;
        if (det != 0.0f)
            det = 1.0f / det;
        mMass00 = det * k11;
        mMass01 = -det * k01;
        mMass10 = -det * k01;
        mMass11 = det * k00;

        mC = (b->WorldCenter() + mRB - mTarget) * beta;

        float damp = 1.0f - 0.02f * (60.0f * dt);
        if (damp < 0.0f)
            damp = 0.0f;
        SolverSetAngularVelocity(b, b->AngularVelocity() * damp);

        SolverSetVelocity(b, b->Velocity() + invMass * mImpulse);
        SolverSetAngularVelocity(b, b->AngularVelocity() + invI * Cross(mRB, mImpulse));
    }

    void MouseJoint::SolveVelocity(float dt)
    {
        Body *b = mBodyB;

        vec2 cdot = b->Velocity() + Cross(b->AngularVelocity(), mRB);
        vec2 rhs = -(cdot + mC + mGamma * mImpulse);
        vec2 impulse(mMass00 * rhs.x + mMass01 * rhs.y,
                          mMass10 * rhs.x + mMass11 * rhs.y);

        vec2 oldImpulse = mImpulse;
        mImpulse += impulse;
        float maxImpulse = dt * mMaxForce;
        float lenSq = mImpulse.x * mImpulse.x + mImpulse.y * mImpulse.y;
        if (lenSq > maxImpulse * maxImpulse)
            mImpulse *= maxImpulse / sqrtf(lenSq);
        impulse = mImpulse - oldImpulse;

        SolverSetVelocity(b, b->Velocity() + b->InvMass() * impulse);
        SolverSetAngularVelocity(b, b->AngularVelocity() + b->InvI() * Cross(mRB, impulse));
    }

} 