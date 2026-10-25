#pragma once

#include "../math.h"

#include "joint.h"

namespace kx
{

    class WheelJoint : public Joint
    {
    public:
        WheelJoint(Body *chassis, Body *wheel, const vec2 &worldAnchor, const vec2 &worldAxis,
                  float frequencyHz = 4.0f, float dampingRatio = 0.7f);

        void SetMotor(bool enabled, float speed, float maxTorque);
        void SetSpring(float frequencyHz, float dampingRatio);

        vec2 AnchorA() const override;
        vec2 AnchorB() const override;

    protected:
        void InitVelocity(float dt) override;
        void SolveVelocity(float dt) override;
        bool SolvePosition() override;

    private:
        vec2 mLocalAnchorA;
        vec2 mLocalAnchorB;
        vec2 mLocalXAxisA;
        vec2 mLocalYAxisA;

        float mImpulse;
        float mMotorImpulse;
        float mSpringImpulse;

        float mMaxMotorTorque;
        float mMotorSpeed;
        bool mEnableMotor;

        float mStiffness;
        float mDamping;

        vec2 mAx, mAy;
        float mSAx, mSBx;
        float mSAy, mSBy;

        float mMass;
        float mMotorMass;
        float mAxialMass;
        float mSpringMass;

        float mBias;
        float mGamma;
    };

} 