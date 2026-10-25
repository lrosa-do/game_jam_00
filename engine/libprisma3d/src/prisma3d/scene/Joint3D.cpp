#include "prisma3d/scene/Joint3D.h"

#include "prisma3d/scene/NodeClassDB.h"

#include <math.h>

namespace prisma3d
{

namespace
{

const float kPi = 3.14159265358979f;

Math::Vec3 columnOf(const Math::Vec4& column)
{
    return Math::Vec3(column.x, column.y, column.z);
}

float infinity()
{
    return INFINITY;
}

} // namespace

Joint3D::Joint3D(PhysicsJointType type)
        : Node3D(),
          mType(type),
          mPathA(),
          mPathB(),
          mDirectA(nullptr),
          mDirectB(nullptr),
          mBodyA(nullptr),
          mBodyB(nullptr),
          mTracked(),
          mJoint(),
          mExceptionA(),
          mExceptionB(),
          mWarning(),
          mExclude(true),
          mConfigured(false),
          mEnabled(true)
{
}

Joint3D::~Joint3D()
{
    releaseServerJoint();
    for (std::size_t i = 0; i < mTracked.size(); ++i) mTracked[i]->detachJoint(this);
    mTracked.clear();
}

PhysicsServer3D* Joint3D::server() const
{
    return PhysicsServer3D::singleton();
}

void Joint3D::setNodeA(const ct::String& path)
{
    if (mPathA == path && !mDirectA) return;
    mPathA = path;
    mDirectA = nullptr;
    rebuild(false);
}

const ct::String& Joint3D::getNodeA() const
{
    return mPathA;
}

void Joint3D::setNodeB(const ct::String& path)
{
    if (mPathB == path && !mDirectB) return;
    mPathB = path;
    mDirectB = nullptr;
    rebuild(false);
}

const ct::String& Joint3D::getNodeB() const
{
    return mPathB;
}

void Joint3D::setBodies(PhysicsBody3D* bodyA, PhysicsBody3D* bodyB)
{
    mPathA.clear();
    mPathB.clear();
    mDirectA = bodyA;
    mDirectB = bodyB;
    track(bodyA);
    track(bodyB);
    rebuild(false);
}

PhysicsBody3D* Joint3D::getBodyA() const
{
    return mBodyA;
}

PhysicsBody3D* Joint3D::getBodyB() const
{
    return mBodyB;
}

void Joint3D::setExcludeNodesFromCollision(bool exclude)
{
    if (mExclude == exclude) return;
    mExclude = exclude;
    rebuild(false);
}

bool Joint3D::getExcludeNodesFromCollision() const
{
    return mExclude;
}

PhysicsJointId Joint3D::getJoint() const
{
    return mJoint;
}

bool Joint3D::isConfigured() const
{
    return mConfigured;
}

const ct::String& Joint3D::getConfigurationWarning() const
{
    return mWarning;
}

void Joint3D::setJointEnabled(bool enabled)
{
    mEnabled = enabled;
    PhysicsServer3D* physics = server();
    if (physics && mConfigured) physics->jointSetEnabled(mJoint, enabled);
}

bool Joint3D::isJointEnabled() const
{
    return mEnabled;
}

void Joint3D::refreshParameters()
{
    if (PhysicsServer3D* physics = server())
        if (mConfigured) applyParameters(*physics);
}

void Joint3D::rebuildJoint()
{
    rebuild(false);
}

void Joint3D::fillFrame(PhysicsJointFrame& frame) const
{
    const Math::Mat4 global = getGlobalTransform();
    frame.anchor = columnOf(global.col3);
    frame.anchorB = frame.anchor;
    frame.axis = columnOf(global.col2);
    frame.axisB = columnOf(global.col0);
}

void Joint3D::applyParameters(PhysicsServer3D& physics)
{
    (void) physics;
}

PhysicsBody3D* Joint3D::resolve(const ct::String& path, PhysicsBody3D* direct) const
{
    if (direct) return direct;
    if (path.empty()) return nullptr;
    Node* node = getNode(path);
    return node ? node->castTo<PhysicsBody3D>() : nullptr;
}

void Joint3D::track(PhysicsBody3D* body)
{
    if (!body) return;
    for (std::size_t i = 0; i < mTracked.size(); ++i)
        if (mTracked[i] == body) return;
    mTracked.push_back(body);
    body->attachJoint(this);
}

void Joint3D::untrack(PhysicsBody3D* body)
{
    for (std::size_t i = 0; i < mTracked.size(); ++i)
    {
        if (mTracked[i] != body) continue;
        mTracked.erase(mTracked.begin() + static_cast<std::ptrdiff_t>(i));
        return;
    }
}

void Joint3D::bodyGone(PhysicsBody3D* body)
{
    untrack(body);
    if (mDirectA == body) mDirectA = nullptr;
    if (mDirectB == body) mDirectB = nullptr;
    releaseServerJoint();
}

void Joint3D::bodyLeftTree()
{
    releaseServerJoint();
    setPhysicsProcess(true);
}

void Joint3D::releaseServerJoint()
{
    PhysicsServer3D* physics = server();
    if (physics)
    {
        if (mExceptionA.valid() && mExceptionB.valid())
            physics->bodyRemoveCollisionException(mExceptionA, mExceptionB);
        if (mJoint.valid()) physics->jointFree(mJoint);
    }
    mExceptionA = PhysicsBodyId();
    mExceptionB = PhysicsBodyId();
    mJoint = PhysicsJointId();
    mBodyA = nullptr;
    mBodyB = nullptr;
    mConfigured = false;
}

void Joint3D::rebuild(bool onlyFree)
{
    releaseServerJoint();
    mWarning.clear();
    if (onlyFree || !isInsideTree())
    {
        setPhysicsProcess(false);
        return;
    }

    PhysicsServer3D* physics = server();
    if (!physics) return;

    PhysicsBody3D* bodyA = resolve(mPathA, mDirectA);
    PhysicsBody3D* bodyB = resolve(mPathB, mDirectB);
    const bool namedA = !mPathA.empty() || mDirectA;
    const bool namedB = !mPathB.empty() || mDirectB;

    if (namedA && !bodyA && namedB && !bodyB) mWarning = "Node A and Node B must be PhysicsBody3Ds";
    else if (namedA && !bodyA)
        mWarning = "Node A must be a PhysicsBody3D";
    else if (namedB && !bodyB)
        mWarning = "Node B must be a PhysicsBody3D";
    else if (!bodyA && !bodyB)
        mWarning = "Joint is not connected to any PhysicsBody3Ds";
    else if (bodyA == bodyB)
        mWarning = "Node A and Node B must be different PhysicsBody3Ds";
    if (!mWarning.empty())
    {
        setPhysicsProcess(false);
        return;
    }

    track(bodyA);
    track(bodyB);

    if ((bodyA && !bodyA->isSimulated()) || (bodyB && !bodyB->isSimulated()))
    {
        setPhysicsProcess(true);
        return;
    }

    if (!bodyA)
    {
        bodyA = bodyB;
        bodyB = nullptr;
    }

    bodyA->syncToServer();
    if (bodyB) bodyB->syncToServer();

    PhysicsJointFrame frame;
    fillFrame(frame);
    const PhysicsJointId joint = physics->jointCreate(mType, bodyA->getBody(),
            bodyB ? bodyB->getBody() : PhysicsBodyId(), frame);
    if (!physics->jointValid(joint))
    {
        mWarning = "The physics server could not create the joint";
        setPhysicsProcess(true);
        return;
    }

    mJoint = joint;
    mBodyA = bodyA;
    mBodyB = bodyB;
    mConfigured = true;
    applyParameters(*physics);
    if (!mEnabled) physics->jointSetEnabled(mJoint, false);
    if (mExclude && bodyB)
    {
        mExceptionA = bodyA->getBody();
        mExceptionB = bodyB->getBody();
        physics->bodyAddCollisionException(mExceptionA, mExceptionB);
    }
    setPhysicsProcess(false);
}

void Joint3D::notificationHandler(int what)
{
    Node3D::notificationHandler(what);
    switch (what)
    {
        case kNotificationPostEnterTree:
            rebuild(false);
            break;
        case kNotificationExitTree:
            rebuild(true);
            break;
        case kNotificationPhysicsProcess:
            if (!mConfigured && mWarning.empty()) rebuild(false);
            break;
        default:
            break;
    }
}

PinJoint3D::PinJoint3D()
        : Joint3D(PhysicsJointType::Pin),
          mMotor(false),
          mAxis(Math::Vec3(0.0f, 1.0f, 0.0f)),
          mVelocity(0.0f),
          mTorque(0.0f)
{
}

void PinJoint3D::setMotor(const Math::Vec3& localAxis, float targetAngularVelocity, float maxTorque)
{
    mMotor = true;
    mAxis = localAxis;
    mVelocity = targetAngularVelocity;
    mTorque = maxTorque;
    if (PhysicsServer3D* physics = server())
        if (isConfigured()) applyParameters(*physics);
}

void PinJoint3D::disableMotor()
{
    mMotor = false;
    if (PhysicsServer3D* physics = server())
        if (isConfigured()) physics->jointDisableMotor(getJoint());
}

void PinJoint3D::setMotorEnabled(bool enabled)
{
    mMotor = enabled;
    refreshParameters();
}

bool PinJoint3D::isMotorEnabled() const
{
    return mMotor;
}

void PinJoint3D::setMotorAxis(const Math::Vec3& localAxis)
{
    mAxis = localAxis;
    refreshParameters();
}

const Math::Vec3& PinJoint3D::getMotorAxis() const
{
    return mAxis;
}

void PinJoint3D::setMotorTargetVelocity(float velocity)
{
    mVelocity = velocity;
    refreshParameters();
}

float PinJoint3D::getMotorTargetVelocity() const
{
    return mVelocity;
}

void PinJoint3D::setMotorMaxTorque(float torque)
{
    mTorque = torque;
    refreshParameters();
}

float PinJoint3D::getMotorMaxTorque() const
{
    return mTorque;
}

void PinJoint3D::applyParameters(PhysicsServer3D& physics)
{
    if (!mMotor)
    {
        physics.jointDisableMotor(getJoint());
        return;
    }
    const Math::Mat4 global = getGlobalTransform();
    const Math::Vec3 world = columnOf(global.col0) * mAxis.x + columnOf(global.col1) * mAxis.y +
                             columnOf(global.col2) * mAxis.z;
    physics.jointSetPinMotor(getJoint(), world, mVelocity, mTorque);
}

HingeJoint3D::HingeJoint3D()
        : Joint3D(PhysicsJointType::Hinge),
          mUseLimit(false),
          mLower(-kPi * 0.5f),
          mUpper(kPi * 0.5f),
          mMotor(false),
          mVelocity(1.0f),
          mTorque(1.0f)
{
}

void HingeJoint3D::setUseLimit(bool use)
{
    mUseLimit = use;
    if (PhysicsServer3D* physics = server())
        if (isConfigured()) applyParameters(*physics);
}

bool HingeJoint3D::getUseLimit() const
{
    return mUseLimit;
}

void HingeJoint3D::setLimit(float lower, float upper)
{
    mLower = lower;
    mUpper = upper;
    if (PhysicsServer3D* physics = server())
        if (isConfigured()) applyParameters(*physics);
}

float HingeJoint3D::getLimitLower() const
{
    return mLower;
}

float HingeJoint3D::getLimitUpper() const
{
    return mUpper;
}

void HingeJoint3D::setMotor(float targetVelocity, float maxTorque)
{
    mMotor = true;
    mVelocity = targetVelocity;
    mTorque = maxTorque;
    if (PhysicsServer3D* physics = server())
        if (isConfigured()) applyParameters(*physics);
}

void HingeJoint3D::disableMotor()
{
    mMotor = false;
    if (PhysicsServer3D* physics = server())
        if (isConfigured()) physics->jointDisableMotor(getJoint());
}

void HingeJoint3D::setMotorEnabled(bool enabled)
{
    mMotor = enabled;
    refreshParameters();
}

bool HingeJoint3D::isMotorEnabled() const
{
    return mMotor;
}

void HingeJoint3D::setMotorTargetVelocity(float velocity)
{
    mVelocity = velocity;
    refreshParameters();
}

float HingeJoint3D::getMotorTargetVelocity() const
{
    return mVelocity;
}

void HingeJoint3D::setMotorMaxTorque(float torque)
{
    mTorque = torque;
    refreshParameters();
}

float HingeJoint3D::getMotorMaxTorque() const
{
    return mTorque;
}

float HingeJoint3D::getAngle() const
{
    PhysicsServer3D* physics = server();
    return physics && isConfigured() ? physics->jointGetPosition(getJoint()) : 0.0f;
}

void HingeJoint3D::applyParameters(PhysicsServer3D& physics)
{
    if (mUseLimit) physics.jointSetLimits(getJoint(), mLower, mUpper);
    else
        physics.jointSetLimits(getJoint(), -kPi, kPi);
    if (mMotor) physics.jointSetMotor(getJoint(), mVelocity, mTorque);
    else
        physics.jointDisableMotor(getJoint());
}

SliderJoint3D::SliderJoint3D()
        : Joint3D(PhysicsJointType::Slider),
          mUseLimit(false),
          mLower(-1.0f),
          mUpper(1.0f),
          mMotor(false),
          mVelocity(0.0f),
          mForce(0.0f)
{
}

void SliderJoint3D::setUseLimit(bool use)
{
    mUseLimit = use;
    if (PhysicsServer3D* physics = server())
        if (isConfigured()) applyParameters(*physics);
}

bool SliderJoint3D::getUseLimit() const
{
    return mUseLimit;
}

void SliderJoint3D::setLimit(float lower, float upper)
{
    mLower = lower;
    mUpper = upper;
    if (PhysicsServer3D* physics = server())
        if (isConfigured()) applyParameters(*physics);
}

float SliderJoint3D::getLimitLower() const
{
    return mLower;
}

float SliderJoint3D::getLimitUpper() const
{
    return mUpper;
}

void SliderJoint3D::setMotor(float targetVelocity, float maxForce)
{
    mMotor = true;
    mVelocity = targetVelocity;
    mForce = maxForce;
    if (PhysicsServer3D* physics = server())
        if (isConfigured()) applyParameters(*physics);
}

void SliderJoint3D::disableMotor()
{
    mMotor = false;
    if (PhysicsServer3D* physics = server())
        if (isConfigured()) physics->jointDisableMotor(getJoint());
}

void SliderJoint3D::setMotorEnabled(bool enabled)
{
    mMotor = enabled;
    refreshParameters();
}

bool SliderJoint3D::isMotorEnabled() const
{
    return mMotor;
}

void SliderJoint3D::setMotorTargetVelocity(float velocity)
{
    mVelocity = velocity;
    refreshParameters();
}

float SliderJoint3D::getMotorTargetVelocity() const
{
    return mVelocity;
}

void SliderJoint3D::setMotorMaxForce(float force)
{
    mForce = force;
    refreshParameters();
}

float SliderJoint3D::getMotorMaxForce() const
{
    return mForce;
}

float SliderJoint3D::getPosition() const
{
    PhysicsServer3D* physics = server();
    return physics && isConfigured() ? physics->jointGetPosition(getJoint()) : 0.0f;
}

void SliderJoint3D::fillFrame(PhysicsJointFrame& frame) const
{
    Joint3D::fillFrame(frame);
    frame.axis = columnOf(getGlobalTransform().col0);
}

void SliderJoint3D::applyParameters(PhysicsServer3D& physics)
{
    if (mUseLimit) physics.jointSetLimits(getJoint(), mLower, mUpper);
    else
        physics.jointSetLimits(getJoint(), -infinity(), infinity());
    if (mMotor) physics.jointSetMotor(getJoint(), mVelocity, mForce);
    else
        physics.jointDisableMotor(getJoint());
}

DistanceJoint3D::DistanceJoint3D()
        : Joint3D(PhysicsJointType::Distance),
          mOffsetB(Math::Vec3(0.0f, 0.0f, 0.0f)),
          mCustomDistance(false),
          mMinimum(0.0f),
          mMaximum(0.0f)
{
}

void DistanceJoint3D::setOffsetB(const Math::Vec3& offset)
{
    mOffsetB = offset;
}

const Math::Vec3& DistanceJoint3D::getOffsetB() const
{
    return mOffsetB;
}

void DistanceJoint3D::setDistance(float minimum, float maximum)
{
    mCustomDistance = true;
    mMinimum = minimum;
    mMaximum = maximum;
    if (PhysicsServer3D* physics = server())
        if (isConfigured()) applyParameters(*physics);
}

float DistanceJoint3D::getDistanceMinimum() const
{
    return mMinimum;
}

float DistanceJoint3D::getDistanceMaximum() const
{
    return mMaximum;
}

void DistanceJoint3D::setDistanceMinimum(float minimum)
{
    mMinimum = minimum;
    refreshParameters();
}

void DistanceJoint3D::setDistanceMaximum(float maximum)
{
    mMaximum = maximum;
    refreshParameters();
}

void DistanceJoint3D::setCustomDistance(bool custom)
{
    if (mCustomDistance == custom) return;
    mCustomDistance = custom;
    if (custom) refreshParameters();
    else
        rebuildJoint();
}

bool DistanceJoint3D::isCustomDistance() const
{
    return mCustomDistance;
}

void DistanceJoint3D::fillFrame(PhysicsJointFrame& frame) const
{
    Joint3D::fillFrame(frame);
    const Math::Mat4 global = getGlobalTransform();
    frame.anchorB = frame.anchor + columnOf(global.col0) * mOffsetB.x +
                    columnOf(global.col1) * mOffsetB.y + columnOf(global.col2) * mOffsetB.z;
}

void DistanceJoint3D::applyParameters(PhysicsServer3D& physics)
{
    if (mCustomDistance) physics.jointSetDistance(getJoint(), mMinimum, mMaximum);
}

FixedJoint3D::FixedJoint3D()
        : Joint3D(PhysicsJointType::Fixed)
{
}

PistonJoint3D::PistonJoint3D()
        : Joint3D(PhysicsJointType::Piston),
          mLinearLimit(false),
          mLinearLower(-1.0f),
          mLinearUpper(1.0f),
          mAngularLimit(false),
          mAngularLower(-kPi * 0.5f),
          mAngularUpper(kPi * 0.5f),
          mLinearMotor(false),
          mLinearVelocity(0.0f),
          mLinearForce(0.0f),
          mAngularMotor(false),
          mAngularVelocity(0.0f),
          mAngularTorque(0.0f)
{
}

void PistonJoint3D::setLinearLimit(float lower, float upper)
{
    mLinearLimit = true;
    mLinearLower = lower;
    mLinearUpper = upper;
    if (PhysicsServer3D* physics = server())
        if (isConfigured()) applyParameters(*physics);
}

void PistonJoint3D::setAngularLimit(float lower, float upper)
{
    mAngularLimit = true;
    mAngularLower = lower;
    mAngularUpper = upper;
    if (PhysicsServer3D* physics = server())
        if (isConfigured()) applyParameters(*physics);
}

void PistonJoint3D::setLinearMotor(float targetVelocity, float maxForce)
{
    mLinearMotor = true;
    mLinearVelocity = targetVelocity;
    mLinearForce = maxForce;
    if (PhysicsServer3D* physics = server())
        if (isConfigured()) applyParameters(*physics);
}

void PistonJoint3D::setAngularMotor(float targetVelocity, float maxTorque)
{
    mAngularMotor = true;
    mAngularVelocity = targetVelocity;
    mAngularTorque = maxTorque;
    if (PhysicsServer3D* physics = server())
        if (isConfigured()) applyParameters(*physics);
}

void PistonJoint3D::disableLinearMotor()
{
    mLinearMotor = false;
    if (PhysicsServer3D* physics = server())
        if (isConfigured()) physics->jointDisableMotor(getJoint());
}

void PistonJoint3D::disableAngularMotor()
{
    mAngularMotor = false;
    if (PhysicsServer3D* physics = server())
        if (isConfigured()) physics->jointDisableSecondaryMotor(getJoint());
}

void PistonJoint3D::setLinearLimitEnabled(bool enabled)
{
    mLinearLimit = enabled;
    refreshParameters();
}

bool PistonJoint3D::isLinearLimitEnabled() const
{
    return mLinearLimit;
}

void PistonJoint3D::setLinearLimitLower(float value)
{
    mLinearLower = value;
    refreshParameters();
}

float PistonJoint3D::getLinearLimitLower() const
{
    return mLinearLower;
}

void PistonJoint3D::setLinearLimitUpper(float value)
{
    mLinearUpper = value;
    refreshParameters();
}

float PistonJoint3D::getLinearLimitUpper() const
{
    return mLinearUpper;
}

void PistonJoint3D::setAngularLimitEnabled(bool enabled)
{
    mAngularLimit = enabled;
    refreshParameters();
}

bool PistonJoint3D::isAngularLimitEnabled() const
{
    return mAngularLimit;
}

void PistonJoint3D::setAngularLimitLower(float value)
{
    mAngularLower = value;
    refreshParameters();
}

float PistonJoint3D::getAngularLimitLower() const
{
    return mAngularLower;
}

void PistonJoint3D::setAngularLimitUpper(float value)
{
    mAngularUpper = value;
    refreshParameters();
}

float PistonJoint3D::getAngularLimitUpper() const
{
    return mAngularUpper;
}

void PistonJoint3D::setLinearMotorEnabled(bool enabled)
{
    mLinearMotor = enabled;
    refreshParameters();
}

bool PistonJoint3D::isLinearMotorEnabled() const
{
    return mLinearMotor;
}

void PistonJoint3D::setLinearMotorTargetVelocity(float value)
{
    mLinearVelocity = value;
    refreshParameters();
}

float PistonJoint3D::getLinearMotorTargetVelocity() const
{
    return mLinearVelocity;
}

void PistonJoint3D::setLinearMotorMaxForce(float value)
{
    mLinearForce = value;
    refreshParameters();
}

float PistonJoint3D::getLinearMotorMaxForce() const
{
    return mLinearForce;
}

void PistonJoint3D::setAngularMotorEnabled(bool enabled)
{
    mAngularMotor = enabled;
    refreshParameters();
}

bool PistonJoint3D::isAngularMotorEnabled() const
{
    return mAngularMotor;
}

void PistonJoint3D::setAngularMotorTargetVelocity(float value)
{
    mAngularVelocity = value;
    refreshParameters();
}

float PistonJoint3D::getAngularMotorTargetVelocity() const
{
    return mAngularVelocity;
}

void PistonJoint3D::setAngularMotorMaxTorque(float value)
{
    mAngularTorque = value;
    refreshParameters();
}

float PistonJoint3D::getAngularMotorMaxTorque() const
{
    return mAngularTorque;
}

float PistonJoint3D::getPosition() const
{
    PhysicsServer3D* physics = server();
    return physics && isConfigured() ? physics->jointGetPosition(getJoint()) : 0.0f;
}

float PistonJoint3D::getAngle() const
{
    PhysicsServer3D* physics = server();
    return physics && isConfigured() ? physics->jointGetSecondaryPosition(getJoint()) : 0.0f;
}

void PistonJoint3D::fillFrame(PhysicsJointFrame& frame) const
{
    Joint3D::fillFrame(frame);
    frame.axis = columnOf(getGlobalTransform().col0);
}

void PistonJoint3D::applyParameters(PhysicsServer3D& physics)
{
    if (mLinearLimit) physics.jointSetLimits(getJoint(), mLinearLower, mLinearUpper);
    else
        physics.jointSetLimits(getJoint(), -infinity(), infinity());
    if (mAngularLimit) physics.jointSetSecondaryLimits(getJoint(), mAngularLower, mAngularUpper);
    else
        physics.jointSetSecondaryLimits(getJoint(), -kPi, kPi);
    if (mLinearMotor) physics.jointSetMotor(getJoint(), mLinearVelocity, mLinearForce);
    else
        physics.jointDisableMotor(getJoint());
    if (mAngularMotor) physics.jointSetSecondaryMotor(getJoint(), mAngularVelocity, mAngularTorque);
    else
        physics.jointDisableSecondaryMotor(getJoint());
}

UniversalJoint3D::UniversalJoint3D()
        : Joint3D(PhysicsJointType::Universal),
          mLimitX(false),
          mLowerX(-kPi * 0.5f),
          mUpperX(kPi * 0.5f),
          mLimitY(false),
          mLowerY(-kPi * 0.5f),
          mUpperY(kPi * 0.5f),
          mMotorX(false),
          mVelocityX(0.0f),
          mTorqueX(0.0f),
          mMotorY(false),
          mVelocityY(0.0f),
          mTorqueY(0.0f)
{
}

void UniversalJoint3D::setLimitX(float lower, float upper)
{
    mLimitX = true;
    mLowerX = lower;
    mUpperX = upper;
    if (PhysicsServer3D* physics = server())
        if (isConfigured()) applyParameters(*physics);
}

void UniversalJoint3D::setLimitY(float lower, float upper)
{
    mLimitY = true;
    mLowerY = lower;
    mUpperY = upper;
    if (PhysicsServer3D* physics = server())
        if (isConfigured()) applyParameters(*physics);
}

void UniversalJoint3D::setMotorX(float targetVelocity, float maxTorque)
{
    mMotorX = true;
    mVelocityX = targetVelocity;
    mTorqueX = maxTorque;
    if (PhysicsServer3D* physics = server())
        if (isConfigured()) applyParameters(*physics);
}

void UniversalJoint3D::setMotorY(float targetVelocity, float maxTorque)
{
    mMotorY = true;
    mVelocityY = targetVelocity;
    mTorqueY = maxTorque;
    if (PhysicsServer3D* physics = server())
        if (isConfigured()) applyParameters(*physics);
}

void UniversalJoint3D::disableMotorX()
{
    mMotorX = false;
    if (PhysicsServer3D* physics = server())
        if (isConfigured()) physics->jointDisableMotor(getJoint());
}

void UniversalJoint3D::disableMotorY()
{
    mMotorY = false;
    if (PhysicsServer3D* physics = server())
        if (isConfigured()) physics->jointDisableSecondaryMotor(getJoint());
}

void UniversalJoint3D::setLimitXEnabled(bool enabled)
{
    mLimitX = enabled;
    refreshParameters();
}

bool UniversalJoint3D::isLimitXEnabled() const
{
    return mLimitX;
}

void UniversalJoint3D::setLimitXLower(float value)
{
    mLowerX = value;
    refreshParameters();
}

float UniversalJoint3D::getLimitXLower() const
{
    return mLowerX;
}

void UniversalJoint3D::setLimitXUpper(float value)
{
    mUpperX = value;
    refreshParameters();
}

float UniversalJoint3D::getLimitXUpper() const
{
    return mUpperX;
}

void UniversalJoint3D::setLimitYEnabled(bool enabled)
{
    mLimitY = enabled;
    refreshParameters();
}

bool UniversalJoint3D::isLimitYEnabled() const
{
    return mLimitY;
}

void UniversalJoint3D::setLimitYLower(float value)
{
    mLowerY = value;
    refreshParameters();
}

float UniversalJoint3D::getLimitYLower() const
{
    return mLowerY;
}

void UniversalJoint3D::setLimitYUpper(float value)
{
    mUpperY = value;
    refreshParameters();
}

float UniversalJoint3D::getLimitYUpper() const
{
    return mUpperY;
}

void UniversalJoint3D::setMotorXEnabled(bool enabled)
{
    mMotorX = enabled;
    refreshParameters();
}

bool UniversalJoint3D::isMotorXEnabled() const
{
    return mMotorX;
}

void UniversalJoint3D::setMotorXTargetVelocity(float value)
{
    mVelocityX = value;
    refreshParameters();
}

float UniversalJoint3D::getMotorXTargetVelocity() const
{
    return mVelocityX;
}

void UniversalJoint3D::setMotorXMaxTorque(float value)
{
    mTorqueX = value;
    refreshParameters();
}

float UniversalJoint3D::getMotorXMaxTorque() const
{
    return mTorqueX;
}

void UniversalJoint3D::setMotorYEnabled(bool enabled)
{
    mMotorY = enabled;
    refreshParameters();
}

bool UniversalJoint3D::isMotorYEnabled() const
{
    return mMotorY;
}

void UniversalJoint3D::setMotorYTargetVelocity(float value)
{
    mVelocityY = value;
    refreshParameters();
}

float UniversalJoint3D::getMotorYTargetVelocity() const
{
    return mVelocityY;
}

void UniversalJoint3D::setMotorYMaxTorque(float value)
{
    mTorqueY = value;
    refreshParameters();
}

float UniversalJoint3D::getMotorYMaxTorque() const
{
    return mTorqueY;
}

float UniversalJoint3D::getAngleX() const
{
    PhysicsServer3D* physics = server();
    return physics && isConfigured() ? physics->jointGetPosition(getJoint()) : 0.0f;
}

float UniversalJoint3D::getAngleY() const
{
    PhysicsServer3D* physics = server();
    return physics && isConfigured() ? physics->jointGetSecondaryPosition(getJoint()) : 0.0f;
}

void UniversalJoint3D::fillFrame(PhysicsJointFrame& frame) const
{
    const Math::Mat4 global = getGlobalTransform();
    frame.anchor = columnOf(global.col3);
    frame.anchorB = frame.anchor;
    frame.axis = columnOf(global.col0);
    frame.axisB = columnOf(global.col1);
}

void UniversalJoint3D::applyParameters(PhysicsServer3D& physics)
{
    if (mLimitX) physics.jointSetLimits(getJoint(), mLowerX, mUpperX);
    else
        physics.jointSetLimits(getJoint(), -kPi, kPi);
    if (mLimitY) physics.jointSetSecondaryLimits(getJoint(), mLowerY, mUpperY);
    else
        physics.jointSetSecondaryLimits(getJoint(), -kPi, kPi);
    if (mMotorX) physics.jointSetMotor(getJoint(), mVelocityX, mTorqueX);
    else
        physics.jointDisableMotor(getJoint());
    if (mMotorY) physics.jointSetSecondaryMotor(getJoint(), mVelocityY, mTorqueY);
    else
        physics.jointDisableSecondaryMotor(getJoint());
}

WheelJoint3D::WheelJoint3D()
        : Joint3D(PhysicsJointType::Wheel),
          mRestLength(0.0f),
          mStiffness(2000.0f),
          mDamping(200.0f),
          mSteeringLimit(false),
          mSteeringLower(-kPi * 0.25f),
          mSteeringUpper(kPi * 0.25f),
          mServo(false),
          mServoAngle(0.0f),
          mServoTorque(0.0f),
          mServoVelocity(0.0f),
          mSpin(false),
          mSpinVelocity(0.0f),
          mSpinTorque(0.0f)
{
}

void WheelJoint3D::setSuspension(float restLength, float stiffness, float damping)
{
    mRestLength = restLength;
    mStiffness = stiffness;
    mDamping = damping;
    if (PhysicsServer3D* physics = server())
        if (isConfigured()) applyParameters(*physics);
}

void WheelJoint3D::setSteeringLimit(float lower, float upper)
{
    mSteeringLimit = true;
    mSteeringLower = lower;
    mSteeringUpper = upper;
    if (PhysicsServer3D* physics = server())
        if (isConfigured()) applyParameters(*physics);
}

void WheelJoint3D::setSteeringServo(float targetAngle, float maxTorque, float maxAngularVelocity)
{
    mServo = true;
    mServoAngle = targetAngle;
    mServoTorque = maxTorque;
    mServoVelocity = maxAngularVelocity;
    if (PhysicsServer3D* physics = server())
        if (isConfigured()) applyParameters(*physics);
}

void WheelJoint3D::disableSteeringServo()
{
    mServo = false;
    if (PhysicsServer3D* physics = server())
        if (isConfigured()) physics->jointDisableSteeringServo(getJoint());
}

void WheelJoint3D::setSpinMotor(float targetVelocity, float maxTorque)
{
    mSpin = true;
    mSpinVelocity = targetVelocity;
    mSpinTorque = maxTorque;
    if (PhysicsServer3D* physics = server())
        if (isConfigured()) applyParameters(*physics);
}

void WheelJoint3D::disableSpinMotor()
{
    mSpin = false;
    if (PhysicsServer3D* physics = server())
        if (isConfigured()) physics->jointDisableMotor(getJoint());
}

void WheelJoint3D::setSuspensionRestLength(float value)
{
    mRestLength = value;
    refreshParameters();
}

float WheelJoint3D::getSuspensionRestLength() const
{
    return mRestLength;
}

void WheelJoint3D::setSuspensionStiffness(float value)
{
    mStiffness = value;
    refreshParameters();
}

float WheelJoint3D::getSuspensionStiffness() const
{
    return mStiffness;
}

void WheelJoint3D::setSuspensionDamping(float value)
{
    mDamping = value;
    refreshParameters();
}

float WheelJoint3D::getSuspensionDamping() const
{
    return mDamping;
}

void WheelJoint3D::setSteeringLimitEnabled(bool enabled)
{
    mSteeringLimit = enabled;
    refreshParameters();
}

bool WheelJoint3D::isSteeringLimitEnabled() const
{
    return mSteeringLimit;
}

void WheelJoint3D::setSteeringLimitLower(float value)
{
    mSteeringLower = value;
    refreshParameters();
}

float WheelJoint3D::getSteeringLimitLower() const
{
    return mSteeringLower;
}

void WheelJoint3D::setSteeringLimitUpper(float value)
{
    mSteeringUpper = value;
    refreshParameters();
}

float WheelJoint3D::getSteeringLimitUpper() const
{
    return mSteeringUpper;
}

void WheelJoint3D::setSteeringServoEnabled(bool enabled)
{
    mServo = enabled;
    refreshParameters();
}

bool WheelJoint3D::isSteeringServoEnabled() const
{
    return mServo;
}

void WheelJoint3D::setSteeringServoTargetAngle(float value)
{
    mServoAngle = value;
    refreshParameters();
}

float WheelJoint3D::getSteeringServoTargetAngle() const
{
    return mServoAngle;
}

void WheelJoint3D::setSteeringServoMaxTorque(float value)
{
    mServoTorque = value;
    refreshParameters();
}

float WheelJoint3D::getSteeringServoMaxTorque() const
{
    return mServoTorque;
}

void WheelJoint3D::setSteeringServoMaxAngularVelocity(float value)
{
    mServoVelocity = value;
    refreshParameters();
}

float WheelJoint3D::getSteeringServoMaxAngularVelocity() const
{
    return mServoVelocity;
}

void WheelJoint3D::setSpinMotorEnabled(bool enabled)
{
    mSpin = enabled;
    refreshParameters();
}

bool WheelJoint3D::isSpinMotorEnabled() const
{
    return mSpin;
}

void WheelJoint3D::setSpinMotorTargetVelocity(float value)
{
    mSpinVelocity = value;
    refreshParameters();
}

float WheelJoint3D::getSpinMotorTargetVelocity() const
{
    return mSpinVelocity;
}

void WheelJoint3D::setSpinMotorMaxTorque(float value)
{
    mSpinTorque = value;
    refreshParameters();
}

float WheelJoint3D::getSpinMotorMaxTorque() const
{
    return mSpinTorque;
}

float WheelJoint3D::getSteeringAngle() const
{
    PhysicsServer3D* physics = server();
    return physics && isConfigured() ? physics->jointGetSecondaryPosition(getJoint()) : 0.0f;
}

float WheelJoint3D::getSuspensionTravel() const
{
    PhysicsServer3D* physics = server();
    return physics && isConfigured() ? physics->jointGetPosition(getJoint()) : 0.0f;
}

void WheelJoint3D::fillFrame(PhysicsJointFrame& frame) const
{
    const Math::Mat4 global = getGlobalTransform();
    frame.anchor = columnOf(global.col3);
    frame.anchorB = frame.anchor;
    frame.axis = Math::Vec3(0.0f, 0.0f, 0.0f) - columnOf(global.col1);
    frame.axisB = columnOf(global.col0);
}

void WheelJoint3D::applyParameters(PhysicsServer3D& physics)
{
    physics.jointSetSuspension(getJoint(), mRestLength, mStiffness, mDamping);
    if (mSteeringLimit) physics.jointSetLimits(getJoint(), mSteeringLower, mSteeringUpper);
    else
        physics.jointSetLimits(getJoint(), -kPi, kPi);
    if (mServo)
        physics.jointSetSteeringServo(getJoint(), mServoAngle, mServoTorque, mServoVelocity);
    else
        physics.jointDisableSteeringServo(getJoint());
    if (mSpin) physics.jointSetMotor(getJoint(), mSpinVelocity, mSpinTorque);
    else
        physics.jointDisableMotor(getJoint());
}

ConeTwistJoint3D::ConeTwistJoint3D()
        : Joint3D(PhysicsJointType::ConeTwist),
          mSwingY(kPi),
          mSwingZ(kPi),
          mTwistLower(-kPi),
          mTwistUpper(kPi)
{
}

void ConeTwistJoint3D::setSwingLimits(float halfAngleY, float halfAngleZ)
{
    mSwingY = halfAngleY;
    mSwingZ = halfAngleZ;
    if (PhysicsServer3D* physics = server())
        if (isConfigured()) applyParameters(*physics);
}

float ConeTwistJoint3D::getSwingLimitY() const
{
    return mSwingY;
}

float ConeTwistJoint3D::getSwingLimitZ() const
{
    return mSwingZ;
}

void ConeTwistJoint3D::setTwistLimits(float lower, float upper)
{
    mTwistLower = lower;
    mTwistUpper = upper;
    if (PhysicsServer3D* physics = server())
        if (isConfigured()) applyParameters(*physics);
}

float ConeTwistJoint3D::getTwistLower() const
{
    return mTwistLower;
}

float ConeTwistJoint3D::getTwistUpper() const
{
    return mTwistUpper;
}

void ConeTwistJoint3D::applyParameters(PhysicsServer3D& physics)
{
    physics.jointSetSecondaryLimits(getJoint(), mSwingY, mSwingZ);
    physics.jointSetLimits(getJoint(), mTwistLower, mTwistUpper);
}

MouseJoint3D::MouseJoint3D()
        : Joint3D(PhysicsJointType::Mouse),
          mTarget(Math::Vec3(0.0f, 0.0f, 0.0f)),
          mHasTarget(false),
          mMaxForce(500.0f),
          mFrequency(5.0f),
          mDampingRatio(0.7f)
{
}

void MouseJoint3D::setTarget(const Math::Vec3& globalTarget)
{
    mTarget = globalTarget;
    mHasTarget = true;
    if (PhysicsServer3D* physics = server())
        if (isConfigured()) physics->jointSetTarget(getJoint(), mTarget);
}

void MouseJoint3D::setTargetPoint(const Math::Vec3& globalTarget)
{
    mTarget = globalTarget;
    if (PhysicsServer3D* physics = server())
        if (isConfigured() && mHasTarget) physics->jointSetTarget(getJoint(), mTarget);
}

const Math::Vec3& MouseJoint3D::getTarget() const
{
    return mTarget;
}

void MouseJoint3D::setUseTarget(bool use)
{
    mHasTarget = use;
    refreshParameters();
}

bool MouseJoint3D::isUsingTarget() const
{
    return mHasTarget;
}

void MouseJoint3D::setSpringMaxForce(float value)
{
    mMaxForce = value;
    refreshParameters();
}

float MouseJoint3D::getSpringMaxForce() const
{
    return mMaxForce;
}

void MouseJoint3D::setSpringFrequency(float value)
{
    mFrequency = value;
    refreshParameters();
}

float MouseJoint3D::getSpringFrequency() const
{
    return mFrequency;
}

void MouseJoint3D::setSpringDampingRatio(float value)
{
    mDampingRatio = value;
    refreshParameters();
}

float MouseJoint3D::getSpringDampingRatio() const
{
    return mDampingRatio;
}

void MouseJoint3D::setSpring(float maxForce, float frequencyHz, float dampingRatio)
{
    mMaxForce = maxForce;
    mFrequency = frequencyHz;
    mDampingRatio = dampingRatio;
    if (PhysicsServer3D* physics = server())
        if (isConfigured()) applyParameters(*physics);
}

void MouseJoint3D::applyParameters(PhysicsServer3D& physics)
{
    physics.jointSetMouseSpring(getJoint(), mMaxForce, mFrequency, mDampingRatio);
    physics.jointSetTarget(getJoint(), mHasTarget ? mTarget : getGlobalPosition());
}

void Joint3D::bindProperties()
{
    using DB = NodeClassDB;
    using P = Joint3D;
    DB::addProperty<P>(PropertyInfo::nodePath("node_a", "PhysicsBody3D"), &P::setNodeA,
            &P::getNodeA);
    DB::addProperty<P>(PropertyInfo::nodePath("node_b", "PhysicsBody3D"), &P::setNodeB,
            &P::getNodeB);
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "enabled"), &P::setJointEnabled,
            &P::isJointEnabled);

    DB::addPropertyGroup<P>("Collision", "exclude_nodes_from_");
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "exclude_nodes_from_collision"),
            &P::setExcludeNodesFromCollision, &P::getExcludeNodesFromCollision);
}

void PinJoint3D::bindProperties()
{
    using DB = NodeClassDB;
    using P = PinJoint3D;
    DB::addPropertyGroup<P>("Motor", "motor/");
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "motor/enable"), &P::setMotorEnabled,
            &P::isMotorEnabled);
    DB::addProperty<P>(PropertyInfo(PropertyType::Vec3, "motor/axis"), &P::setMotorAxis,
            &P::getMotorAxis);
    PropertyInfo velocity = PropertyInfo::range(PropertyType::Float, "motor/target_velocity",
            -200.0, 200.0, 0.01, true, true);
    velocity.radiansAsDegrees = true;
    DB::addProperty<P>(velocity, &P::setMotorTargetVelocity, &P::getMotorTargetVelocity);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "motor/max_torque", 0.0, 100000.0, 0.01, true),
            &P::setMotorMaxTorque, &P::getMotorMaxTorque);
}

void HingeJoint3D::bindProperties()
{
    using DB = NodeClassDB;
    using P = HingeJoint3D;
    DB::addPropertyGroup<P>("Angular Limit", "angular_limit/");
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "angular_limit/enable"), &P::setUseLimit,
            &P::getUseLimit);
    PropertyInfo upper = PropertyInfo::range(PropertyType::Float, "angular_limit/upper", -180.0,
            180.0, 0.1, true, true);
    upper.radiansAsDegrees = true;
    DB::addProperty<P>(
            upper, [](P* joint, const PropertyValue& value)
            { joint->setLimit(joint->getLimitLower(), static_cast<float>(value.asFloat())); },
            &P::getLimitUpper);
    PropertyInfo lower = PropertyInfo::range(PropertyType::Float, "angular_limit/lower", -180.0,
            180.0, 0.1, true, true);
    lower.radiansAsDegrees = true;
    DB::addProperty<P>(
            lower, [](P* joint, const PropertyValue& value)
            { joint->setLimit(static_cast<float>(value.asFloat()), joint->getLimitUpper()); },
            &P::getLimitLower);

    DB::addPropertyGroup<P>("Motor", "motor/");
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "motor/enable"), &P::setMotorEnabled,
            &P::isMotorEnabled);
    PropertyInfo velocity = PropertyInfo::range(PropertyType::Float, "motor/target_velocity",
            -200.0, 200.0, 0.01, true, true);
    velocity.radiansAsDegrees = true;
    DB::addProperty<P>(velocity, &P::setMotorTargetVelocity, &P::getMotorTargetVelocity);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "motor/max_torque", 0.0, 100000.0, 0.01, true),
            &P::setMotorMaxTorque, &P::getMotorMaxTorque);
}

void SliderJoint3D::bindProperties()
{
    using DB = NodeClassDB;
    using P = SliderJoint3D;
    DB::addPropertyGroup<P>("Linear Limit", "linear_limit/");
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "linear_limit/enable"), &P::setUseLimit,
            &P::getUseLimit);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "linear_limit/upper_distance", -1024.0, 1024.0,
                    0.01, true, true),
            [](P* joint, const PropertyValue& value)
            { joint->setLimit(joint->getLimitLower(), static_cast<float>(value.asFloat())); },
            &P::getLimitUpper);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "linear_limit/lower_distance", -1024.0, 1024.0,
                    0.01, true, true),
            [](P* joint, const PropertyValue& value)
            { joint->setLimit(static_cast<float>(value.asFloat()), joint->getLimitUpper()); },
            &P::getLimitLower);

    DB::addPropertyGroup<P>("Motor", "motor/");
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "motor/enable"), &P::setMotorEnabled,
            &P::isMotorEnabled);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "motor/target_velocity", -1024.0,
                               1024.0, 0.01, true, true),
            &P::setMotorTargetVelocity, &P::getMotorTargetVelocity);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "motor/force_limit", 0.0, 100000.0,
                               0.01, true),
            &P::setMotorMaxForce, &P::getMotorMaxForce);
}

void DistanceJoint3D::bindProperties()
{
    using DB = NodeClassDB;
    using P = DistanceJoint3D;
    DB::addProperty<P>(PropertyInfo(PropertyType::Vec3, "offset_b"), &P::setOffsetB,
            &P::getOffsetB);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "distance_minimum", 0.0, 4096.0, 0.01, true),
            &P::setDistanceMinimum, &P::getDistanceMinimum);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "distance_maximum", 0.0, 4096.0, 0.01, true),
            &P::setDistanceMaximum, &P::getDistanceMaximum);
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "custom_distance"), &P::setCustomDistance,
            &P::isCustomDistance);
}

void FixedJoint3D::bindProperties()
{
}

void PistonJoint3D::bindProperties()
{
    using DB = NodeClassDB;
    using P = PistonJoint3D;
    const auto distance = [](const char* name)
    { return PropertyInfo::range(PropertyType::Float, name, -1024.0, 1024.0, 0.01, true, true); };
    const auto angle = [](const char* name)
    {
        PropertyInfo info =
                PropertyInfo::range(PropertyType::Float, name, -180.0, 180.0, 0.1, true, true);
        info.radiansAsDegrees = true;
        return info;
    };
    const auto force = [](const char* name)
    { return PropertyInfo::range(PropertyType::Float, name, 0.0, 100000.0, 0.01, true); };

    DB::addPropertyGroup<P>("Linear Limit", "linear_limit/");
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "linear_limit/enabled"),
            &P::setLinearLimitEnabled, &P::isLinearLimitEnabled);
    DB::addProperty<P>(distance("linear_limit/upper_distance"), &P::setLinearLimitUpper,
            &P::getLinearLimitUpper);
    DB::addProperty<P>(distance("linear_limit/lower_distance"), &P::setLinearLimitLower,
            &P::getLinearLimitLower);

    DB::addPropertyGroup<P>("Angular Limit", "angular_limit/");
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "angular_limit/enabled"),
            &P::setAngularLimitEnabled, &P::isAngularLimitEnabled);
    DB::addProperty<P>(angle("angular_limit/upper_angle"), &P::setAngularLimitUpper,
            &P::getAngularLimitUpper);
    DB::addProperty<P>(angle("angular_limit/lower_angle"), &P::setAngularLimitLower,
            &P::getAngularLimitLower);

    DB::addPropertyGroup<P>("Linear Motor", "linear_motor/");
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "linear_motor/enabled"),
            &P::setLinearMotorEnabled, &P::isLinearMotorEnabled);
    DB::addProperty<P>(distance("linear_motor/target_velocity"), &P::setLinearMotorTargetVelocity,
            &P::getLinearMotorTargetVelocity);
    DB::addProperty<P>(force("linear_motor/force_limit"), &P::setLinearMotorMaxForce,
            &P::getLinearMotorMaxForce);

    DB::addPropertyGroup<P>("Angular Motor", "angular_motor/");
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "angular_motor/enabled"),
            &P::setAngularMotorEnabled, &P::isAngularMotorEnabled);
    DB::addProperty<P>(angle("angular_motor/target_velocity"), &P::setAngularMotorTargetVelocity,
            &P::getAngularMotorTargetVelocity);
    DB::addProperty<P>(force("angular_motor/force_limit"), &P::setAngularMotorMaxTorque,
            &P::getAngularMotorMaxTorque);
}

void UniversalJoint3D::bindProperties()
{
    using DB = NodeClassDB;
    using P = UniversalJoint3D;
    const auto angle = [](const char* name)
    {
        PropertyInfo info =
                PropertyInfo::range(PropertyType::Float, name, -180.0, 180.0, 0.1, true, true);
        info.radiansAsDegrees = true;
        return info;
    };
    const auto force = [](const char* name)
    { return PropertyInfo::range(PropertyType::Float, name, 0.0, 100000.0, 0.01, true); };

    DB::addPropertyGroup<P>("Angular Limit X", "angular_limit_x/");
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "angular_limit_x/enabled"),
            &P::setLimitXEnabled, &P::isLimitXEnabled);
    DB::addProperty<P>(angle("angular_limit_x/upper_angle"), &P::setLimitXUpper,
            &P::getLimitXUpper);
    DB::addProperty<P>(angle("angular_limit_x/lower_angle"), &P::setLimitXLower,
            &P::getLimitXLower);

    DB::addPropertyGroup<P>("Angular Limit Y", "angular_limit_y/");
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "angular_limit_y/enabled"),
            &P::setLimitYEnabled, &P::isLimitYEnabled);
    DB::addProperty<P>(angle("angular_limit_y/upper_angle"), &P::setLimitYUpper,
            &P::getLimitYUpper);
    DB::addProperty<P>(angle("angular_limit_y/lower_angle"), &P::setLimitYLower,
            &P::getLimitYLower);

    DB::addPropertyGroup<P>("Angular Motor X", "angular_motor_x/");
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "angular_motor_x/enabled"),
            &P::setMotorXEnabled, &P::isMotorXEnabled);
    DB::addProperty<P>(angle("angular_motor_x/target_velocity"), &P::setMotorXTargetVelocity,
            &P::getMotorXTargetVelocity);
    DB::addProperty<P>(force("angular_motor_x/force_limit"), &P::setMotorXMaxTorque,
            &P::getMotorXMaxTorque);

    DB::addPropertyGroup<P>("Angular Motor Y", "angular_motor_y/");
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "angular_motor_y/enabled"),
            &P::setMotorYEnabled, &P::isMotorYEnabled);
    DB::addProperty<P>(angle("angular_motor_y/target_velocity"), &P::setMotorYTargetVelocity,
            &P::getMotorYTargetVelocity);
    DB::addProperty<P>(force("angular_motor_y/force_limit"), &P::setMotorYMaxTorque,
            &P::getMotorYMaxTorque);
}

void WheelJoint3D::bindProperties()
{
    using DB = NodeClassDB;
    using P = WheelJoint3D;
    const auto angle = [](const char* name)
    {
        PropertyInfo info =
                PropertyInfo::range(PropertyType::Float, name, -180.0, 180.0, 0.1, true, true);
        info.radiansAsDegrees = true;
        return info;
    };
    const auto force = [](const char* name)
    { return PropertyInfo::range(PropertyType::Float, name, 0.0, 100000.0, 0.01, true); };

    DB::addPropertyGroup<P>("Suspension", "suspension/");
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "suspension/rest_length", 0.0,
                               100.0, 0.001, true),
            &P::setSuspensionRestLength, &P::getSuspensionRestLength);
    DB::addProperty<P>(force("suspension/stiffness"), &P::setSuspensionStiffness,
            &P::getSuspensionStiffness);
    DB::addProperty<P>(force("suspension/damping"), &P::setSuspensionDamping,
            &P::getSuspensionDamping);

    DB::addPropertyGroup<P>("Steering Limit", "steering_limit/");
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "steering_limit/enabled"),
            &P::setSteeringLimitEnabled, &P::isSteeringLimitEnabled);
    DB::addProperty<P>(angle("steering_limit/upper_angle"), &P::setSteeringLimitUpper,
            &P::getSteeringLimitUpper);
    DB::addProperty<P>(angle("steering_limit/lower_angle"), &P::setSteeringLimitLower,
            &P::getSteeringLimitLower);

    DB::addPropertyGroup<P>("Steering Servo", "steering_servo/");
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "steering_servo/enabled"),
            &P::setSteeringServoEnabled, &P::isSteeringServoEnabled);
    DB::addProperty<P>(angle("steering_servo/target_angle"), &P::setSteeringServoTargetAngle,
            &P::getSteeringServoTargetAngle);
    DB::addProperty<P>(force("steering_servo/force_limit"), &P::setSteeringServoMaxTorque,
            &P::getSteeringServoMaxTorque);
    DB::addProperty<P>(angle("steering_servo/max_angular_velocity"),
            &P::setSteeringServoMaxAngularVelocity, &P::getSteeringServoMaxAngularVelocity);

    DB::addPropertyGroup<P>("Spin Motor", "spin_motor/");
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "spin_motor/enabled"),
            &P::setSpinMotorEnabled, &P::isSpinMotorEnabled);
    DB::addProperty<P>(angle("spin_motor/target_velocity"), &P::setSpinMotorTargetVelocity,
            &P::getSpinMotorTargetVelocity);
    DB::addProperty<P>(force("spin_motor/force_limit"), &P::setSpinMotorMaxTorque,
            &P::getSpinMotorMaxTorque);
}

void ConeTwistJoint3D::bindProperties()
{
    using DB = NodeClassDB;
    using P = ConeTwistJoint3D;
    PropertyInfo swingY =
            PropertyInfo::range(PropertyType::Float, "swing_limit_y", 0.0, 180.0, 0.1);
    swingY.radiansAsDegrees = true;
    DB::addProperty<P>(
            swingY,
            [](P* joint, const PropertyValue& value) {
                joint->setSwingLimits(static_cast<float>(value.asFloat()), joint->getSwingLimitZ());
            },
            &P::getSwingLimitY);
    PropertyInfo swingZ =
            PropertyInfo::range(PropertyType::Float, "swing_limit_z", 0.0, 180.0, 0.1);
    swingZ.radiansAsDegrees = true;
    DB::addProperty<P>(
            swingZ,
            [](P* joint, const PropertyValue& value) {
                joint->setSwingLimits(joint->getSwingLimitY(), static_cast<float>(value.asFloat()));
            },
            &P::getSwingLimitZ);
    PropertyInfo twistLower =
            PropertyInfo::range(PropertyType::Float, "twist_lower", -180.0, 180.0, 0.1, true, true);
    twistLower.radiansAsDegrees = true;
    DB::addProperty<P>(
            twistLower, [](P* joint, const PropertyValue& value)
            { joint->setTwistLimits(static_cast<float>(value.asFloat()), joint->getTwistUpper()); },
            &P::getTwistLower);
    PropertyInfo twistUpper =
            PropertyInfo::range(PropertyType::Float, "twist_upper", -180.0, 180.0, 0.1, true, true);
    twistUpper.radiansAsDegrees = true;
    DB::addProperty<P>(
            twistUpper, [](P* joint, const PropertyValue& value)
            { joint->setTwistLimits(joint->getTwistLower(), static_cast<float>(value.asFloat())); },
            &P::getTwistUpper);
}

void MouseJoint3D::bindProperties()
{
    using DB = NodeClassDB;
    using P = MouseJoint3D;
    DB::addProperty<P>(PropertyInfo(PropertyType::Vec3, "target"), &P::setTargetPoint,
            &P::getTarget);
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "use_target"), &P::setUseTarget,
            &P::isUsingTarget);

    DB::addPropertyGroup<P>("Spring", "spring/");
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "spring/max_force", 0.0, 1000000.0,
                               0.01, true),
            &P::setSpringMaxForce, &P::getSpringMaxForce);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "spring/frequency", 0.0, 1000.0, 0.01, true),
            &P::setSpringFrequency, &P::getSpringFrequency);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "spring/damping_ratio", 0.0, 100.0,
                               0.01, true),
            &P::setSpringDampingRatio, &P::getSpringDampingRatio);
}

} // namespace prisma3d
