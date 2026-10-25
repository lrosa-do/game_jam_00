#include "kx/joints/distancejoint.h"

#include "kx/body.h"

#include <math.h>

namespace kx
{

    DistanceJoint::DistanceJoint(Body *a, Body *b, const vec2 &worldAnchorA, const vec2 &worldAnchorB)
        : Joint(JointType::Distance, a, b),
          mRangeSet(false), mStiffness(0.0f), mDamping(0.0f),
          mGamma(0.0f), mBias(0.0f), mImpulse(0.0f), mLowerImpulse(0.0f), mUpperImpulse(0.0f),
          mU(0.0f, 0.0f), mRA(0.0f, 0.0f), mRB(0.0f, 0.0f),
          mCurrentLength(0.0f), mMass(0.0f), mSoftMass(0.0f)
    {
        mLocalAnchorA = InvTransformPoint(a->GetTransform(), worldAnchorA);
        mLocalAnchorB = InvTransformPoint(b->GetTransform(), worldAnchorB);

        vec2 d = worldAnchorB - worldAnchorA;
        mLength = sqrtf(Dot(d, d));
        if (mLength < kLinearSlop)
            mLength = kLinearSlop;
        mMinLength = mLength;
        mMaxLength = mLength;
    }

    void DistanceJoint::SetSpring(float frequencyHz, float dampingRatio)
    {
        float massA = mBodyA->Mass();
        float massB = mBodyB->Mass();
        float mass;
        if (massA > 0.0f && massB > 0.0f)
            mass = massA * massB / (massA + massB);
        else if (massA > 0.0f)
            mass = massA;
        else
            mass = massB;

        float omega = 2.0f * kPi * frequencyHz;
        mStiffness = mass * omega * omega;
        mDamping = 2.0f * mass * dampingRatio * omega;
        if (mStiffness <= 0.0f)
            mImpulse = 0.0f;
        WakeBodies();
    }

    void DistanceJoint::SetLengthRange(float minLength, float maxLength)
    {
        mMinLength = minLength < kLinearSlop ? kLinearSlop : minLength;
        mMaxLength = maxLength < mMinLength ? mMinLength : maxLength;
        mRangeSet = true;
        mLength = Clamp(mLength, mMinLength, mMaxLength);
        mLowerImpulse = 0.0f;
        mUpperImpulse = 0.0f;
        WakeBodies();
    }

    vec2 DistanceJoint::AnchorA() const
    {
        return mBodyA->GetTransform().Transform(mLocalAnchorA);
    }

    vec2 DistanceJoint::AnchorB() const
    {
        return mBodyB->GetTransform().Transform(mLocalAnchorB);
    }

    void DistanceJoint::InitVelocity(float dt)
    {
        Body *a = mBodyA;
        Body *b = mBodyB;

        Transform xfA = a->GetTransform();
        Transform xfB = b->GetTransform();

        vec2 worldAnchorA = xfA.Transform(mLocalAnchorA);
        vec2 worldAnchorB = xfB.Transform(mLocalAnchorB);
        mRA = worldAnchorA - a->WorldCenter();
        mRB = worldAnchorB - b->WorldCenter();
        mU = worldAnchorB - worldAnchorA;

        mCurrentLength = sqrtf(Dot(mU, mU));
        if (mCurrentLength > kLinearSlop)
        {
            mU *= 1.0f / mCurrentLength;
        }
        else
        {
            mU = vec2(0.0f, 0.0f);
            mMass = 0.0f;
            mImpulse = 0.0f;
            mLowerImpulse = 0.0f;
            mUpperImpulse = 0.0f;
        }

        float mA = a->InvMass(), mB = b->InvMass();
        float iA = a->InvI(), iB = b->InvI();

        float crAu = Cross(mRA, mU);
        float crBu = Cross(mRB, mU);
        float invMass = mA + iA * crAu * crAu + mB + iB * crBu * crBu;
        mMass = invMass != 0.0f ? 1.0f / invMass : 0.0f;

        if (mStiffness > 0.0f)
        {
            float C = mCurrentLength - mLength;

            float h = dt;
            mGamma = h * (mDamping + h * mStiffness);
            mGamma = mGamma != 0.0f ? 1.0f / mGamma : 0.0f;
            mBias = C * h * mStiffness * mGamma;

            invMass += mGamma;
            mSoftMass = invMass != 0.0f ? 1.0f / invMass : 0.0f;
        }
        else
        {
            mGamma = 0.0f;
            mBias = 0.0f;
            mSoftMass = mMass;
        }

        vec2 P = (mImpulse + mLowerImpulse - mUpperImpulse) * mU;
        SolverSetVelocity(a, a->Velocity() - mA * P);
        SolverSetAngularVelocity(a, a->AngularVelocity() - iA * Cross(mRA, P));
        SolverSetVelocity(b, b->Velocity() + mB * P);
        SolverSetAngularVelocity(b, b->AngularVelocity() + iB * Cross(mRB, P));
    }

    void DistanceJoint::SolveVelocity(float dt)
    {
        Body *a = mBodyA;
        Body *b = mBodyB;

        float mA = a->InvMass(), mB = b->InvMass();
        float iA = a->InvI(), iB = b->InvI();

        vec2 vA = a->Velocity();
        float wA = a->AngularVelocity();
        vec2 vB = b->Velocity();
        float wB = b->AngularVelocity();

        // Rigid only without a spring and with no slack in the range.
        const bool rigid = mStiffness <= 0.0f && mMinLength >= mMaxLength;
        if (!rigid)
        {
            if (mStiffness > 0.0f)
            {
                vec2 vpA = vA + Cross(wA, mRA);
                vec2 vpB = vB + Cross(wB, mRB);
                float Cdot = Dot(mU, vpB - vpA);

                float impulse = -mSoftMass * (Cdot + mBias + mGamma * mImpulse);
                mImpulse += impulse;

                vec2 P = impulse * mU;
                vA -= mA * P;
                wA -= iA * Cross(mRA, P);
                vB += mB * P;
                wB += iB * Cross(mRB, P);
            }

            if (mRangeSet)
            {
                float C = mCurrentLength - mMinLength;
                float bias = (C > 0.0f ? C : 0.0f) / dt;

                vec2 vpA = vA + Cross(wA, mRA);
                vec2 vpB = vB + Cross(wB, mRB);
                float Cdot = Dot(mU, vpB - vpA);

                float impulse = -mMass * (Cdot + bias);
                float oldImpulse = mLowerImpulse;
                mLowerImpulse = mLowerImpulse + impulse > 0.0f ? mLowerImpulse + impulse : 0.0f;
                impulse = mLowerImpulse - oldImpulse;
                vec2 P = impulse * mU;

                vA -= mA * P;
                wA -= iA * Cross(mRA, P);
                vB += mB * P;
                wB += iB * Cross(mRB, P);
            }

            if (mRangeSet)
            {
                float C = mMaxLength - mCurrentLength;
                float bias = (C > 0.0f ? C : 0.0f) / dt;

                vec2 vpA = vA + Cross(wA, mRA);
                vec2 vpB = vB + Cross(wB, mRB);
                float Cdot = Dot(mU, vpA - vpB);

                float impulse = -mMass * (Cdot + bias);
                float oldImpulse = mUpperImpulse;
                mUpperImpulse = mUpperImpulse + impulse > 0.0f ? mUpperImpulse + impulse : 0.0f;
                impulse = mUpperImpulse - oldImpulse;
                vec2 P = -impulse * mU;

                vA -= mA * P;
                wA -= iA * Cross(mRA, P);
                vB += mB * P;
                wB += iB * Cross(mRB, P);
            }
        }
        else
        {
            vec2 vpA = vA + Cross(wA, mRA);
            vec2 vpB = vB + Cross(wB, mRB);
            float Cdot = Dot(mU, vpB - vpA);

            float impulse = -mMass * Cdot;
            mImpulse += impulse;

            vec2 P = impulse * mU;
            vA -= mA * P;
            wA -= iA * Cross(mRA, P);
            vB += mB * P;
            wB += iB * Cross(mRB, P);
        }

        SolverSetVelocity(a, vA);
        SolverSetAngularVelocity(a, wA);
        SolverSetVelocity(b, vB);
        SolverSetAngularVelocity(b, wB);
    }

    bool DistanceJoint::SolvePosition()
    {
        Body *a = mBodyA;
        Body *b = mBodyB;

        Transform xfA = a->GetTransform();
        Transform xfB = b->GetTransform();
        vec2 worldAnchorA = xfA.Transform(mLocalAnchorA);
        vec2 worldAnchorB = xfB.Transform(mLocalAnchorB);
        vec2 rA = worldAnchorA - a->WorldCenter();
        vec2 rB = worldAnchorB - b->WorldCenter();
        vec2 u = worldAnchorB - worldAnchorA;

        float length = sqrtf(Dot(u, u));
        if (length > kEpsilon)
            u *= 1.0f / length;

        float C;
        if (mStiffness > 0.0f && !mRangeSet)
            return true;
        if (mMinLength == mMaxLength)
            C = length - mMinLength;
        else if (length < mMinLength)
            C = length - mMinLength;
        else if (mMaxLength < length)
            C = length - mMaxLength;
        else
            return true;

        float impulse = -mMass * C;
        vec2 P = impulse * u;

        a->ShiftCenter(-a->InvMass() * P, -a->InvI() * Cross(rA, P));
        b->ShiftCenter(b->InvMass() * P, b->InvI() * Cross(rB, P));

        return fabsf(C) < kLinearSlop;
    }

} 