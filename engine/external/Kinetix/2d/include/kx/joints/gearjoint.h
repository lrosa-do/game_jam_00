#pragma once

#include "../math.h"

#include "joint.h"

namespace kx
{

    class RevoluteJoint;

    class GearJoint : public Joint
    {
    public:
        GearJoint(RevoluteJoint *jointA, RevoluteJoint *jointB, float ratio);

        void SetRatio(float ratio) { mRatio = ratio; }
        float Ratio() const { return mRatio; }

        vec2 AnchorA() const override;
        vec2 AnchorB() const override;

    protected:
        void InitVelocity(float dt) override;
        void SolveVelocity(float dt) override;
        bool SolvePosition() override;

        bool DependsOnBody(const Body *body) const override
        {
            return body == mBodyC || body == mBodyD;
        }
        bool DependsOnJoint(const Joint *joint) const override;
        int ConnectedBodies(Body *out[4]) const override;

    private:
        RevoluteJoint *mJoint1;
        RevoluteJoint *mJoint2;

        Body *mBodyC;
        Body *mBodyD;

        vec2 mLocalAnchorA;
        vec2 mLocalAnchorB;

        float mReferenceAngleA;
        float mReferenceAngleB;

        float mConstant;
        float mRatio;
        float mImpulse;
        float mMass;
    };

} 