#pragma once

#include "../math.h"

#include "joint.h"

namespace kx
{

    class DistanceJoint : public Joint
    {
    public:
        DistanceJoint(Body *a, Body *b, const vec2 &worldAnchorA, const vec2 &worldAnchorB);

        // A spring (frequencyHz > 0) pulls toward the rest length on its own;
        // without one the joint is rigid. SetLengthRange adds hard lower and
        // upper limits - and clamps the rest length into them - with or
        // without a spring.
        void SetSpring(float frequencyHz, float dampingRatio);
        void SetLengthRange(float minLength, float maxLength);

        vec2 AnchorA() const override;
        vec2 AnchorB() const override;

    protected:
        void InitVelocity(float dt) override;
        void SolveVelocity(float dt) override;
        bool SolvePosition() override;

    private:
        vec2 mLocalAnchorA;
        vec2 mLocalAnchorB;
        float mLength;
        float mMinLength;
        float mMaxLength;
        bool mRangeSet;

        float mStiffness;
        float mDamping;

        float mGamma;
        float mBias;
        float mImpulse;
        float mLowerImpulse;
        float mUpperImpulse;

        vec2 mU;
        vec2 mRA, mRB;
        float mCurrentLength;
        float mMass;
        float mSoftMass;
    };

} 