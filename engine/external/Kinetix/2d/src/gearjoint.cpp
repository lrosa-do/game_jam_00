#include "kx/joints/gearjoint.h"
#include "kx/joints/revolutejoint.h"
#include "kx/body.h"

#include <math.h>

namespace kx
{

    GearJoint::GearJoint(RevoluteJoint *jointA, RevoluteJoint *jointB, float ratio)
        : Joint(JointType::Gear, jointA->BodyB(), jointB->BodyB()),
          mJoint1(jointA), mJoint2(jointB),
          mBodyC(jointA->BodyA()), mBodyD(jointB->BodyA()),
          mReferenceAngleA(jointA->mReferenceAngle), mReferenceAngleB(jointB->mReferenceAngle),
          mConstant(0.0f), mRatio(ratio), mImpulse(0.0f), mMass(0.0f)
    {
        mLocalAnchorA = jointA->mLocalAnchorB;
        mLocalAnchorB = jointB->mLocalAnchorB;

        float coordinateA = mBodyA->Angle() - mBodyC->Angle() - mReferenceAngleA;
        float coordinateB = mBodyB->Angle() - mBodyD->Angle() - mReferenceAngleB;
        mConstant = coordinateA + mRatio * coordinateB;
    }

    bool GearJoint::DependsOnJoint(const Joint *joint) const
    {
        return joint == static_cast<const Joint *>(mJoint1) ||
               joint == static_cast<const Joint *>(mJoint2);
    }

    int GearJoint::ConnectedBodies(Body *out[4]) const
    {
        out[0] = mBodyA;
        out[1] = mBodyB;
        out[2] = mBodyC;
        out[3] = mBodyD;
        return 4;
    }

    vec2 GearJoint::AnchorA() const
    {
        return mBodyA->GetTransform().Transform(mLocalAnchorA);
    }

    vec2 GearJoint::AnchorB() const
    {
        return mBodyB->GetTransform().Transform(mLocalAnchorB);
    }

    void GearJoint::InitVelocity(float dt)
    {
        (void)dt;

        float iA = mBodyA->InvI(), iB = mBodyB->InvI(), iC = mBodyC->InvI(), iD = mBodyD->InvI();
        float massInv = iA + iC + mRatio * mRatio * (iB + iD);
        mMass = massInv > 0.0f ? 1.0f / massInv : 0.0f;

        SolverSetAngularVelocity(mBodyA, mBodyA->AngularVelocity() + iA * mImpulse);
        SolverSetAngularVelocity(mBodyC, mBodyC->AngularVelocity() - iC * mImpulse);
        SolverSetAngularVelocity(mBodyB, mBodyB->AngularVelocity() + iB * mImpulse * mRatio);
        SolverSetAngularVelocity(mBodyD, mBodyD->AngularVelocity() - iD * mImpulse * mRatio);
    }

    void GearJoint::SolveVelocity(float dt)
    {
        (void)dt;

        float iA = mBodyA->InvI(), iB = mBodyB->InvI(), iC = mBodyC->InvI(), iD = mBodyD->InvI();
        float wA = mBodyA->AngularVelocity(), wB = mBodyB->AngularVelocity();
        float wC = mBodyC->AngularVelocity(), wD = mBodyD->AngularVelocity();

        float Cdot = (wA - wC) + mRatio * (wB - wD);
        float impulse = -mMass * Cdot;
        mImpulse += impulse;

        SolverSetAngularVelocity(mBodyA, wA + iA * impulse);
        SolverSetAngularVelocity(mBodyC, wC - iC * impulse);
        SolverSetAngularVelocity(mBodyB, wB + iB * impulse * mRatio);
        SolverSetAngularVelocity(mBodyD, wD - iD * impulse * mRatio);
    }

    bool GearJoint::SolvePosition()
    {
        float iA = mBodyA->InvI(), iB = mBodyB->InvI(), iC = mBodyC->InvI(), iD = mBodyD->InvI();
        float mass = iA + iC + mRatio * mRatio * (iB + iD);

        float coordinateA = mBodyA->Angle() - mBodyC->Angle() - mReferenceAngleA;
        float coordinateB = mBodyB->Angle() - mBodyD->Angle() - mReferenceAngleB;
        float C = (coordinateA + mRatio * coordinateB) - mConstant;

        float impulse = mass > 0.0f ? -C / mass : 0.0f;

        mBodyA->ShiftCenter(vec2(0.0f, 0.0f), iA * impulse);
        mBodyC->ShiftCenter(vec2(0.0f, 0.0f), -iC * impulse);
        mBodyB->ShiftCenter(vec2(0.0f, 0.0f), iB * impulse * mRatio);
        mBodyD->ShiftCenter(vec2(0.0f, 0.0f), -iD * impulse * mRatio);

        return fabsf(C) < kAngularSlop;
    }

} 