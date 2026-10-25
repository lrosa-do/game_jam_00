#pragma once

#include "../math.h"

#include "joint.h"

namespace kx
{

    class MouseJoint : public Joint
    {
    public:
        MouseJoint(Body *body, const vec2 &target, float maxForce,
                   float frequencyHz = 5.0f, float dampingRatio = 0.7f);

        void SetTarget(const vec2 &target);
        const vec2 &Target() const { return mTarget; }

        vec2 AnchorA() const override { return mTarget; }
        vec2 AnchorB() const override;

    protected:
        void InitVelocity(float dt) override;
        void SolveVelocity(float dt) override;

    private:
        vec2 mTarget;
        vec2 mLocalAnchor;
        float mMaxForce;
        float mStiffness;
        float mDamping;

        vec2 mRB;
        vec2 mC;
        vec2 mImpulse;
        float mGamma;
        float mMass00, mMass01, mMass10, mMass11;
    };

} 