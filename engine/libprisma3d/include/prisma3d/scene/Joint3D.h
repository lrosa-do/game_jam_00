#pragma once

#include "prisma3d/physics/PhysicsServer3D.h"
#include "prisma3d/scene/Node3D.h"
#include "prisma3d/scene/PhysicsBody3D.h"

#include <ct/string.hpp>
#include <ct/vector.hpp>

namespace prisma3d
{

class Joint3D : public Node3D
{
public:
    PRISMA3D_NODE_CLASS(Joint3D, Node3D);
    static void bindProperties();

    ~Joint3D() override;

    void setNodeA(const ct::String& path);
    const ct::String& getNodeA() const;
    void setNodeB(const ct::String& path);
    const ct::String& getNodeB() const;
    void setBodies(PhysicsBody3D* bodyA, PhysicsBody3D* bodyB);

    PhysicsBody3D* getBodyA() const;
    PhysicsBody3D* getBodyB() const;

    void setExcludeNodesFromCollision(bool exclude);
    bool getExcludeNodesFromCollision() const;

    PhysicsJointId getJoint() const;
    bool isConfigured() const;
    const ct::String& getConfigurationWarning() const;
    void setJointEnabled(bool enabled);
    bool isJointEnabled() const;

protected:
    explicit Joint3D(PhysicsJointType type);

    virtual void fillFrame(PhysicsJointFrame& frame) const;
    virtual void applyParameters(PhysicsServer3D& server);
    void refreshParameters();
    void rebuildJoint();

    void notificationHandler(int what) override;
    PhysicsServer3D* server() const;
    const PhysicsJointType mType;

private:
    friend class PhysicsBody3D;

    void rebuild(bool onlyFree);
    void releaseServerJoint();
    PhysicsBody3D* resolve(const ct::String& path, PhysicsBody3D* direct) const;
    void track(PhysicsBody3D* body);
    void untrack(PhysicsBody3D* body);
    void bodyGone(PhysicsBody3D* body);
    void bodyLeftTree();

    ct::String mPathA;
    ct::String mPathB;
    PhysicsBody3D* mDirectA;
    PhysicsBody3D* mDirectB;
    PhysicsBody3D* mBodyA;
    PhysicsBody3D* mBodyB;
    ct::Vector<PhysicsBody3D*> mTracked;
    PhysicsJointId mJoint;
    PhysicsBodyId mExceptionA;
    PhysicsBodyId mExceptionB;
    ct::String mWarning;
    bool mExclude;
    bool mConfigured;
    bool mEnabled;
};

class PinJoint3D : public Joint3D
{
public:
    PRISMA3D_NODE_CLASS(PinJoint3D, Joint3D);
    static void bindProperties();

    PinJoint3D();

    void setMotor(const Math::Vec3& localAxis, float targetAngularVelocity, float maxTorque);
    void disableMotor();
    void setMotorEnabled(bool enabled);
    bool isMotorEnabled() const;
    void setMotorAxis(const Math::Vec3& localAxis);
    const Math::Vec3& getMotorAxis() const;
    void setMotorTargetVelocity(float velocity);
    float getMotorTargetVelocity() const;
    void setMotorMaxTorque(float torque);
    float getMotorMaxTorque() const;

protected:
    void applyParameters(PhysicsServer3D& server) override;

private:
    bool mMotor;
    Math::Vec3 mAxis;
    float mVelocity;
    float mTorque;
};

class HingeJoint3D : public Joint3D
{
public:
    PRISMA3D_NODE_CLASS(HingeJoint3D, Joint3D);
    static void bindProperties();

    HingeJoint3D();

    void setUseLimit(bool use);
    bool getUseLimit() const;
    void setLimit(float lower, float upper);
    float getLimitLower() const;
    float getLimitUpper() const;
    void setMotor(float targetVelocity, float maxTorque);
    void disableMotor();
    void setMotorEnabled(bool enabled);
    bool isMotorEnabled() const;
    void setMotorTargetVelocity(float velocity);
    float getMotorTargetVelocity() const;
    void setMotorMaxTorque(float torque);
    float getMotorMaxTorque() const;
    float getAngle() const;

protected:
    void applyParameters(PhysicsServer3D& server) override;

private:
    bool mUseLimit;
    float mLower;
    float mUpper;
    bool mMotor;
    float mVelocity;
    float mTorque;
};

class SliderJoint3D : public Joint3D
{
public:
    PRISMA3D_NODE_CLASS(SliderJoint3D, Joint3D);
    static void bindProperties();

    SliderJoint3D();

    void setUseLimit(bool use);
    bool getUseLimit() const;
    void setLimit(float lower, float upper);
    float getLimitLower() const;
    float getLimitUpper() const;
    void setMotor(float targetVelocity, float maxForce);
    void disableMotor();
    void setMotorEnabled(bool enabled);
    bool isMotorEnabled() const;
    void setMotorTargetVelocity(float velocity);
    float getMotorTargetVelocity() const;
    void setMotorMaxForce(float force);
    float getMotorMaxForce() const;
    float getPosition() const;

protected:
    void fillFrame(PhysicsJointFrame& frame) const override;
    void applyParameters(PhysicsServer3D& server) override;

private:
    bool mUseLimit;
    float mLower;
    float mUpper;
    bool mMotor;
    float mVelocity;
    float mForce;
};

class DistanceJoint3D : public Joint3D
{
public:
    PRISMA3D_NODE_CLASS(DistanceJoint3D, Joint3D);
    static void bindProperties();

    DistanceJoint3D();

    void setOffsetB(const Math::Vec3& offset);
    const Math::Vec3& getOffsetB() const;
    void setDistance(float minimum, float maximum);
    float getDistanceMinimum() const;
    float getDistanceMaximum() const;
    void setDistanceMinimum(float minimum);
    void setDistanceMaximum(float maximum);
    void setCustomDistance(bool custom);
    bool isCustomDistance() const;

protected:
    void fillFrame(PhysicsJointFrame& frame) const override;
    void applyParameters(PhysicsServer3D& server) override;

private:
    Math::Vec3 mOffsetB;
    bool mCustomDistance;
    float mMinimum;
    float mMaximum;
};

class FixedJoint3D : public Joint3D
{
public:
    PRISMA3D_NODE_CLASS(FixedJoint3D, Joint3D);
    static void bindProperties();

    FixedJoint3D();
};

class PistonJoint3D : public Joint3D
{
public:
    PRISMA3D_NODE_CLASS(PistonJoint3D, Joint3D);
    static void bindProperties();

    PistonJoint3D();

    void setLinearLimit(float lower, float upper);
    void setAngularLimit(float lower, float upper);
    void setLinearMotor(float targetVelocity, float maxForce);
    void setAngularMotor(float targetVelocity, float maxTorque);
    void disableLinearMotor();
    void disableAngularMotor();
    void setLinearLimitEnabled(bool enabled);
    bool isLinearLimitEnabled() const;
    void setLinearLimitLower(float lower);
    float getLinearLimitLower() const;
    void setLinearLimitUpper(float upper);
    float getLinearLimitUpper() const;
    void setAngularLimitEnabled(bool enabled);
    bool isAngularLimitEnabled() const;
    void setAngularLimitLower(float lower);
    float getAngularLimitLower() const;
    void setAngularLimitUpper(float upper);
    float getAngularLimitUpper() const;
    void setLinearMotorEnabled(bool enabled);
    bool isLinearMotorEnabled() const;
    void setLinearMotorTargetVelocity(float velocity);
    float getLinearMotorTargetVelocity() const;
    void setLinearMotorMaxForce(float force);
    float getLinearMotorMaxForce() const;
    void setAngularMotorEnabled(bool enabled);
    bool isAngularMotorEnabled() const;
    void setAngularMotorTargetVelocity(float velocity);
    float getAngularMotorTargetVelocity() const;
    void setAngularMotorMaxTorque(float torque);
    float getAngularMotorMaxTorque() const;
    float getPosition() const;
    float getAngle() const;

protected:
    void fillFrame(PhysicsJointFrame& frame) const override;
    void applyParameters(PhysicsServer3D& server) override;

private:
    bool mLinearLimit;
    float mLinearLower;
    float mLinearUpper;
    bool mAngularLimit;
    float mAngularLower;
    float mAngularUpper;
    bool mLinearMotor;
    float mLinearVelocity;
    float mLinearForce;
    bool mAngularMotor;
    float mAngularVelocity;
    float mAngularTorque;
};

class UniversalJoint3D : public Joint3D
{
public:
    PRISMA3D_NODE_CLASS(UniversalJoint3D, Joint3D);
    static void bindProperties();

    UniversalJoint3D();

    void setLimitX(float lower, float upper);
    void setLimitY(float lower, float upper);
    void setMotorX(float targetVelocity, float maxTorque);
    void setMotorY(float targetVelocity, float maxTorque);
    void disableMotorX();
    void disableMotorY();
    void setLimitXEnabled(bool enabled);
    bool isLimitXEnabled() const;
    void setLimitXLower(float lower);
    float getLimitXLower() const;
    void setLimitXUpper(float upper);
    float getLimitXUpper() const;
    void setLimitYEnabled(bool enabled);
    bool isLimitYEnabled() const;
    void setLimitYLower(float lower);
    float getLimitYLower() const;
    void setLimitYUpper(float upper);
    float getLimitYUpper() const;
    void setMotorXEnabled(bool enabled);
    bool isMotorXEnabled() const;
    void setMotorXTargetVelocity(float velocity);
    float getMotorXTargetVelocity() const;
    void setMotorXMaxTorque(float torque);
    float getMotorXMaxTorque() const;
    void setMotorYEnabled(bool enabled);
    bool isMotorYEnabled() const;
    void setMotorYTargetVelocity(float velocity);
    float getMotorYTargetVelocity() const;
    void setMotorYMaxTorque(float torque);
    float getMotorYMaxTorque() const;
    float getAngleX() const;
    float getAngleY() const;

protected:
    void fillFrame(PhysicsJointFrame& frame) const override;
    void applyParameters(PhysicsServer3D& server) override;

private:
    bool mLimitX;
    float mLowerX;
    float mUpperX;
    bool mLimitY;
    float mLowerY;
    float mUpperY;
    bool mMotorX;
    float mVelocityX;
    float mTorqueX;
    bool mMotorY;
    float mVelocityY;
    float mTorqueY;
};

class WheelJoint3D : public Joint3D
{
public:
    PRISMA3D_NODE_CLASS(WheelJoint3D, Joint3D);
    static void bindProperties();

    WheelJoint3D();

    void setSuspension(float restLength, float stiffness, float damping);
    void setSteeringLimit(float lower, float upper);
    void setSteeringServo(float targetAngle, float maxTorque, float maxAngularVelocity);
    void disableSteeringServo();
    void setSpinMotor(float targetVelocity, float maxTorque);
    void disableSpinMotor();
    void setSuspensionRestLength(float restLength);
    float getSuspensionRestLength() const;
    void setSuspensionStiffness(float stiffness);
    float getSuspensionStiffness() const;
    void setSuspensionDamping(float damping);
    float getSuspensionDamping() const;
    void setSteeringLimitEnabled(bool enabled);
    bool isSteeringLimitEnabled() const;
    void setSteeringLimitLower(float lower);
    float getSteeringLimitLower() const;
    void setSteeringLimitUpper(float upper);
    float getSteeringLimitUpper() const;
    void setSteeringServoEnabled(bool enabled);
    bool isSteeringServoEnabled() const;
    void setSteeringServoTargetAngle(float angle);
    float getSteeringServoTargetAngle() const;
    void setSteeringServoMaxTorque(float torque);
    float getSteeringServoMaxTorque() const;
    void setSteeringServoMaxAngularVelocity(float velocity);
    float getSteeringServoMaxAngularVelocity() const;
    void setSpinMotorEnabled(bool enabled);
    bool isSpinMotorEnabled() const;
    void setSpinMotorTargetVelocity(float velocity);
    float getSpinMotorTargetVelocity() const;
    void setSpinMotorMaxTorque(float torque);
    float getSpinMotorMaxTorque() const;
    float getSteeringAngle() const;
    float getSuspensionTravel() const;

protected:
    void fillFrame(PhysicsJointFrame& frame) const override;
    void applyParameters(PhysicsServer3D& server) override;

private:
    float mRestLength;
    float mStiffness;
    float mDamping;
    bool mSteeringLimit;
    float mSteeringLower;
    float mSteeringUpper;
    bool mServo;
    float mServoAngle;
    float mServoTorque;
    float mServoVelocity;
    bool mSpin;
    float mSpinVelocity;
    float mSpinTorque;
};

class ConeTwistJoint3D : public Joint3D
{
public:
    PRISMA3D_NODE_CLASS(ConeTwistJoint3D, Joint3D);
    static void bindProperties();

    ConeTwistJoint3D();

    void setSwingLimits(float halfAngleY, float halfAngleZ);
    float getSwingLimitY() const;
    float getSwingLimitZ() const;
    void setTwistLimits(float lower, float upper);
    float getTwistLower() const;
    float getTwistUpper() const;

protected:
    void applyParameters(PhysicsServer3D& server) override;

private:
    float mSwingY;
    float mSwingZ;
    float mTwistLower;
    float mTwistUpper;
};

class MouseJoint3D : public Joint3D
{
public:
    PRISMA3D_NODE_CLASS(MouseJoint3D, Joint3D);
    static void bindProperties();

    MouseJoint3D();

    void setTarget(const Math::Vec3& globalTarget);
    void setTargetPoint(const Math::Vec3& globalTarget);
    const Math::Vec3& getTarget() const;
    void setUseTarget(bool use);
    bool isUsingTarget() const;
    void setSpring(float maxForce, float frequencyHz, float dampingRatio);
    void setSpringMaxForce(float maxForce);
    float getSpringMaxForce() const;
    void setSpringFrequency(float frequencyHz);
    float getSpringFrequency() const;
    void setSpringDampingRatio(float dampingRatio);
    float getSpringDampingRatio() const;

protected:
    void applyParameters(PhysicsServer3D& server) override;

private:
    Math::Vec3 mTarget;
    bool mHasTarget;
    float mMaxForce;
    float mFrequency;
    float mDampingRatio;
};

} // namespace prisma3d
