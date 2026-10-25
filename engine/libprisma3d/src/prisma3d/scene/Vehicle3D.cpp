#include "prisma3d/scene/Vehicle3D.h"

#include "prisma3d/scene/NodeClassDB.h"

#include <math.h>

namespace prisma3d
{

namespace
{

const float kTwoPi = 6.28318530717958647692f;

Math::Vec3 columnOf(const Math::Vec4& column)
{
    return Math::Vec3(column.x, column.y, column.z);
}

} // namespace

VehicleWheel3D::VehicleWheel3D()
        : Node3D(),
          mBody(nullptr),
          mIndex(-1),
          mFrameLocked(false),
          mConnection(Math::Vec3(0.0f, 0.0f, 0.0f)),
          mDirection(Math::Vec3(0.0f, -1.0f, 0.0f)),
          mAxle(Math::Vec3(1.0f, 0.0f, 0.0f)),
          mRadius(0.5f),
          mRestLength(0.15f),
          mTravel(5.0f),
          mStiffness(5.88f),
          mMaxForce(6000.0f),
          mDampingCompression(0.83f),
          mDampingRelaxation(0.88f),
          mFrictionSlip(10.5f),
          mRollInfluence(0.1f),
          mEngineForce(0.0f),
          mBrake(0.0f),
          mSteering(0.0f),
          mTraction(false),
          mSteers(false),
          mState(),
          mPreviousRotation(0.0f),
          mRpm(0.0f)
{
    setNotifyLocalTransform(true);
}

VehicleWheel3D::~VehicleWheel3D()
{
    if (mBody) mBody->removeWheel(this);
}

PhysicsWheelParameters VehicleWheel3D::parameters() const
{
    PhysicsWheelParameters p;
    p.connection = mConnection;
    p.direction = mDirection;
    p.axle = mAxle;
    p.restLength = mRestLength;
    p.travel = mTravel;
    p.radius = mRadius;
    p.stiffness = mStiffness;
    p.dampingCompression = mDampingCompression;
    p.dampingRelaxation = mDampingRelaxation;
    p.frictionSlip = mFrictionSlip;
    p.maxForce = mMaxForce;
    p.rollInfluence = mRollInfluence;
    p.front = mSteers;
    return p;
}

void VehicleWheel3D::captureFrame()
{
    if (mFrameLocked) return;
    const Math::Mat4 local = getTransform();
    mConnection = columnOf(local.col3);
    mDirection = (Math::Vec3(0.0f, 0.0f, 0.0f) - columnOf(local.col1)).NormalizedSafe();
    mAxle = columnOf(local.col0).NormalizedSafe();
}

void VehicleWheel3D::pushParameters()
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (!server || !mBody || mIndex < 0) return;
    server->vehicleSetWheelParameters(mBody->getVehicle(), mIndex, parameters());
}

void VehicleWheel3D::setRadius(float radius)
{
    mRadius = radius;
    pushParameters();
}

float VehicleWheel3D::getRadius() const
{
    return mRadius;
}

void VehicleWheel3D::setSuspensionRestLength(float length)
{
    mRestLength = length;
    pushParameters();
}

float VehicleWheel3D::getSuspensionRestLength() const
{
    return mRestLength;
}

void VehicleWheel3D::setSuspensionTravel(float length)
{
    mTravel = length;
    pushParameters();
}

float VehicleWheel3D::getSuspensionTravel() const
{
    return mTravel;
}

void VehicleWheel3D::setSuspensionStiffness(float value)
{
    mStiffness = value;
    pushParameters();
}

float VehicleWheel3D::getSuspensionStiffness() const
{
    return mStiffness;
}

void VehicleWheel3D::setSuspensionMaxForce(float value)
{
    mMaxForce = value;
    pushParameters();
}

float VehicleWheel3D::getSuspensionMaxForce() const
{
    return mMaxForce;
}

void VehicleWheel3D::setDampingCompression(float value)
{
    mDampingCompression = value;
    pushParameters();
}

float VehicleWheel3D::getDampingCompression() const
{
    return mDampingCompression;
}

void VehicleWheel3D::setDampingRelaxation(float value)
{
    mDampingRelaxation = value;
    pushParameters();
}

float VehicleWheel3D::getDampingRelaxation() const
{
    return mDampingRelaxation;
}

void VehicleWheel3D::setFrictionSlip(float value)
{
    mFrictionSlip = value;
    pushParameters();
}

float VehicleWheel3D::getFrictionSlip() const
{
    return mFrictionSlip;
}

void VehicleWheel3D::setRollInfluence(float value)
{
    mRollInfluence = value;
    pushParameters();
}

float VehicleWheel3D::getRollInfluence() const
{
    return mRollInfluence;
}

void VehicleWheel3D::setEngineForce(float force)
{
    mEngineForce = force;
}

float VehicleWheel3D::getEngineForce() const
{
    return mEngineForce;
}

void VehicleWheel3D::setBrake(float brake)
{
    mBrake = brake;
}

float VehicleWheel3D::getBrake() const
{
    return mBrake;
}

void VehicleWheel3D::setSteering(float steering)
{
    mSteering = steering;
}

float VehicleWheel3D::getSteering() const
{
    return mSteering;
}

void VehicleWheel3D::setUseAsTraction(bool enable)
{
    mTraction = enable;
}

bool VehicleWheel3D::isUsedAsTraction() const
{
    return mTraction;
}

void VehicleWheel3D::setUseAsSteering(bool enable)
{
    mSteers = enable;
    pushParameters();
}

bool VehicleWheel3D::isUsedAsSteering() const
{
    return mSteers;
}

bool VehicleWheel3D::isInContact() const
{
    return mState.inContact;
}

PhysicsBody3D* VehicleWheel3D::getContactBody() const
{
    return mState.inContact ? PhysicsBody3D::fromBody(mState.groundBody) : nullptr;
}

Math::Vec3 VehicleWheel3D::getContactPoint() const
{
    return mState.contactPoint;
}

Math::Vec3 VehicleWheel3D::getContactNormal() const
{
    return mState.contactNormal;
}

float VehicleWheel3D::getSkidInfo() const
{
    return mState.skidInfo;
}

float VehicleWheel3D::getRpm() const
{
    return mRpm;
}

void VehicleWheel3D::notificationHandler(int what)
{
    Node3D::notificationHandler(what);
    switch (what)
    {
        case kNotificationEnterTree:
        {
            Node* parent = getParent();
            VehicleBody3D* body = parent ? parent->castTo<VehicleBody3D>() : nullptr;
            if (!body) break;
            captureFrame();
            body->addWheel(this);
            break;
        }
        case kNotificationLocalTransformChanged:
            captureFrame();
            if (mBody) mBody->mDirty = !mFrameLocked ? true : mBody->mDirty;
            break;
        case kNotificationExitTree:
            if (mBody) mBody->removeWheel(this);
            break;
        default:
            break;
    }
}

VehicleBody3D::VehicleBody3D()
        : RigidBody3D(),
          mVehicle(),
          mWheels(),
          mEngineForce(0.0f),
          mBrake(0.0f),
          mSteering(0.0f),
          mDirty(false),
          mMotorcycle(false),
          mSteerInput(0.0f),
          mMaxSteer(0.7f),
          mCaster(0.0f),
          mMaxLean(0.7853982f),
          mLeanK(5000.0f),
          mLeanC(1000.0f),
          mLeanEnabled(true),
          mStep(1.0f / 60.0f)
{
    setPhysicsProcess(true);
}

VehicleBody3D::~VehicleBody3D()
{
    releaseVehicle();
    for (std::size_t i = 0; i < mWheels.size(); ++i)
    {
        mWheels[i]->mBody = nullptr;
        mWheels[i]->mIndex = -1;
    }
    mWheels.clear();
}

void VehicleBody3D::setEngineForce(float force)
{
    mEngineForce = force;
    for (std::size_t i = 0; i < mWheels.size(); ++i)
        if (mWheels[i]->mTraction) mWheels[i]->mEngineForce = force;
}

float VehicleBody3D::getEngineForce() const
{
    return mEngineForce;
}

void VehicleBody3D::setBrake(float brake)
{
    mBrake = brake;
    for (std::size_t i = 0; i < mWheels.size(); ++i) mWheels[i]->mBrake = brake;
}

float VehicleBody3D::getBrake() const
{
    return mBrake;
}

void VehicleBody3D::setSteering(float steering)
{
    mSteering = steering;
    for (std::size_t i = 0; i < mWheels.size(); ++i)
        if (mWheels[i]->mSteers) mWheels[i]->mSteering = steering;
}

float VehicleBody3D::getSteering() const
{
    return mSteering;
}

PhysicsVehicleId VehicleBody3D::getVehicle() const
{
    return mVehicle;
}

float VehicleBody3D::getSpeedKmh() const
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    return server ? server->vehicleGetSpeedKmh(mVehicle) : 0.0f;
}

std::size_t VehicleBody3D::getWheelCount() const
{
    return mWheels.size();
}

VehicleWheel3D* VehicleBody3D::getWheel(std::size_t index) const
{
    return index < mWheels.size() ? mWheels[index] : nullptr;
}

void VehicleBody3D::addWheel(VehicleWheel3D* wheel)
{
    for (std::size_t i = 0; i < mWheels.size(); ++i)
        if (mWheels[i] == wheel) return;
    wheel->mBody = this;
    mWheels.push_back(wheel);
    mDirty = true;
}

void VehicleBody3D::removeWheel(VehicleWheel3D* wheel)
{
    for (std::size_t i = 0; i < mWheels.size(); ++i)
    {
        if (mWheels[i] != wheel) continue;
        mWheels.erase(mWheels.begin() + static_cast<std::ptrdiff_t>(i));
        break;
    }
    wheel->mBody = nullptr;
    wheel->mIndex = -1;
    mDirty = true;
}

void VehicleBody3D::releaseVehicle()
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (server && mVehicle.valid()) server->vehicleFree(mVehicle);
    mVehicle = PhysicsVehicleId();
    for (std::size_t i = 0; i < mWheels.size(); ++i) mWheels[i]->mIndex = -1;
}

void VehicleBody3D::rebuild()
{
    releaseVehicle();
    mDirty = false;
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (!server || mWheels.empty() || !isSimulated()) return;
    mVehicle = server->vehicleCreate(getBody());
    if (!server->vehicleValid(mVehicle))
    {
        mVehicle = PhysicsVehicleId();
        return;
    }
    for (std::size_t i = 0; i < mWheels.size(); ++i)
    {
        mWheels[i]->mIndex = server->vehicleAddWheel(mVehicle, mWheels[i]->parameters());
        mWheels[i]->mFrameLocked = true;
    }
    applyMotorcycle();
}

void VehicleBody3D::applyMotorcycle()
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (!server || !server->vehicleValid(mVehicle)) return;
    server->vehicleEnableMotorcycle(mVehicle, mMotorcycle);
    if (!mMotorcycle) return;
    server->motorcycleSetMaxSteerAngle(mVehicle, mMaxSteer);
    server->motorcycleSetCasterAngle(mVehicle, mCaster);
    server->motorcycleSetMaxLeanAngle(mVehicle, mMaxLean);
    server->motorcycleSetLeanSpring(mVehicle, mLeanK, mLeanC);
    server->motorcycleSetLeanControllerEnabled(mVehicle, mLeanEnabled);
    server->motorcycleSetSteerInput(mVehicle, mSteerInput);
}

void VehicleBody3D::pushControls()
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (!server || !server->vehicleValid(mVehicle)) return;
    for (std::size_t i = 0; i < mWheels.size(); ++i)
    {
        const VehicleWheel3D* wheel = mWheels[i];
        if (wheel->mIndex < 0) continue;
        server->vehicleSetEngineForce(mVehicle, wheel->mIndex, wheel->mEngineForce);
        server->vehicleSetBrake(mVehicle, wheel->mIndex, wheel->mBrake);
        if (!(mMotorcycle && wheel->mSteers))
            server->vehicleSetSteering(mVehicle, wheel->mIndex, wheel->mSteering);
    }
}

void VehicleBody3D::setMotorcycle(bool enabled)
{
    mMotorcycle = enabled;
    applyMotorcycle();
}

bool VehicleBody3D::isMotorcycle() const
{
    return mMotorcycle;
}

void VehicleBody3D::setSteerInput(float input)
{
    mSteerInput = input;
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (server && mMotorcycle) server->motorcycleSetSteerInput(mVehicle, input);
}

float VehicleBody3D::getSteerInput() const
{
    return mSteerInput;
}

void VehicleBody3D::setMaxSteerAngle(float angle)
{
    mMaxSteer = angle;
    applyMotorcycle();
}

float VehicleBody3D::getMaxSteerAngle() const
{
    return mMaxSteer;
}

void VehicleBody3D::setCasterAngle(float angle)
{
    mCaster = angle;
    applyMotorcycle();
}

float VehicleBody3D::getCasterAngle() const
{
    return mCaster;
}

void VehicleBody3D::setMaxLeanAngle(float angle)
{
    mMaxLean = angle;
    applyMotorcycle();
}

float VehicleBody3D::getMaxLeanAngle() const
{
    return mMaxLean;
}

void VehicleBody3D::setLeanSpringConstant(float springConstant)
{
    setLeanSpring(springConstant, mLeanC);
}

float VehicleBody3D::getLeanSpringConstant() const
{
    return mLeanK;
}

void VehicleBody3D::setLeanDamping(float damping)
{
    setLeanSpring(mLeanK, damping);
}

float VehicleBody3D::getLeanDamping() const
{
    return mLeanC;
}

void VehicleBody3D::setLeanSpring(float springConstant, float damping)
{
    mLeanK = springConstant;
    mLeanC = damping;
    applyMotorcycle();
}

void VehicleBody3D::setLeanControllerEnabled(bool enabled)
{
    mLeanEnabled = enabled;
    applyMotorcycle();
}

bool VehicleBody3D::isLeanControllerEnabled() const
{
    return mLeanEnabled;
}

float VehicleBody3D::getLeanAngle() const
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    return server ? server->motorcycleGetLeanAngle(mVehicle) : 0.0f;
}

void VehicleBody3D::bodyMoved(const Math::Mat4& frame)
{
    RigidBody3D::bodyMoved(frame);
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (!server || !server->vehicleValid(mVehicle)) return;
    const Math::Mat4 inverse = frame.InverseAffine();
    for (std::size_t i = 0; i < mWheels.size(); ++i)
    {
        VehicleWheel3D* wheel = mWheels[i];
        PhysicsWheelState state;
        if (wheel->mIndex < 0 || !server->vehicleGetWheelState(mVehicle, wheel->mIndex, state))
            continue;
        wheel->mState = state;
        wheel->setTransform(inverse * state.worldTransform);
        wheel->mRpm = (state.rotation - wheel->mPreviousRotation) / mStep * 60.0f / kTwoPi;
        wheel->mPreviousRotation = state.rotation;
    }
}

void VehicleBody3D::notificationHandler(int what)
{
    RigidBody3D::notificationHandler(what);
    switch (what)
    {
        case kNotificationPhysicsProcess:
        {
            PhysicsServer3D* server = PhysicsServer3D::singleton();
            if (!server) break;
            mStep = static_cast<float>(getPhysicsProcessDeltaTime());
            if (mStep <= 0.0f) mStep = 1.0f / 60.0f;
            if (mDirty || (!server->vehicleValid(mVehicle) && !mWheels.empty() && isSimulated()))
                rebuild();
            pushControls();
            break;
        }
        case kNotificationExitTree:
            releaseVehicle();
            mDirty = true;
            break;
        default:
            break;
    }
}

void VehicleWheel3D::bindProperties()
{
    using DB = NodeClassDB;
    using P = VehicleWheel3D;
    DB::addPropertyGroup<P>("Per-Wheel Motion", "");
    DB::addProperty<P>(PropertyInfo(PropertyType::Float, "engine_force"), &P::setEngineForce,
            &P::getEngineForce);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "brake", 0.0, 1000.0, 0.01, true),
            &P::setBrake, &P::getBrake);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "steering", -1.5708, 1.5708, 0.001,
                               true, true),
            &P::setSteering, &P::getSteering);
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "use_as_traction"), &P::setUseAsTraction,
            &P::isUsedAsTraction);
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "use_as_steering"), &P::setUseAsSteering,
            &P::isUsedAsSteering);

    DB::addPropertyGroup<P>("Wheel", "wheel_");
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "wheel_roll_influence", 0.0, 1.0, 0.01, true),
            &P::setRollInfluence, &P::getRollInfluence);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "wheel_radius", 0.001, 100.0, 0.001, true),
            &P::setRadius, &P::getRadius);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "wheel_rest_length", 0.0, 100.0, 0.001, true),
            &P::setSuspensionRestLength, &P::getSuspensionRestLength);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "wheel_friction_slip", 0.0, 1000.0,
                               0.01, true),
            &P::setFrictionSlip, &P::getFrictionSlip);

    DB::addPropertyGroup<P>("Suspension", "suspension_");
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "suspension_travel", 0.0, 100.0, 0.001, true),
            &P::setSuspensionTravel, &P::getSuspensionTravel);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "suspension_stiffness", 0.0, 1000.0,
                               0.01, true),
            &P::setSuspensionStiffness, &P::getSuspensionStiffness);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "suspension_max_force", 0.0,
                               100000.0, 0.01, true),
            &P::setSuspensionMaxForce, &P::getSuspensionMaxForce);

    DB::addPropertyGroup<P>("Damping", "damping_");
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "damping_compression", 0.0, 10.0, 0.01, true),
            &P::setDampingCompression, &P::getDampingCompression);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "damping_relaxation", 0.0, 10.0, 0.01, true),
            &P::setDampingRelaxation, &P::getDampingRelaxation);
}

void VehicleBody3D::bindProperties()
{
    using DB = NodeClassDB;
    using P = VehicleBody3D;
    DB::addPropertyGroup<P>("Motion", "");
    DB::addProperty<P>(PropertyInfo(PropertyType::Float, "engine_force"), &P::setEngineForce,
            &P::getEngineForce);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "brake", 0.0, 1000.0, 0.01, true),
            &P::setBrake, &P::getBrake);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "steering", -1.5708, 1.5708, 0.001,
                               true, true),
            &P::setSteering, &P::getSteering);

    DB::addPropertyGroup<P>("Motorcycle", "motorcycle_");
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "motorcycle_enabled"), &P::setMotorcycle,
            &P::isMotorcycle);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "motorcycle_steer_input", -1.0, 1.0,
                               0.01, true, true),
            &P::setSteerInput, &P::getSteerInput);
    PropertyInfo maxSteer = PropertyInfo::range(PropertyType::Float, "motorcycle_max_steer_angle",
            0.0, 90.0, 0.1, true);
    maxSteer.radiansAsDegrees = true;
    DB::addProperty<P>(maxSteer, &P::setMaxSteerAngle, &P::getMaxSteerAngle);
    PropertyInfo caster = PropertyInfo::range(PropertyType::Float, "motorcycle_caster_angle", -90.0,
            90.0, 0.1, true, true);
    caster.radiansAsDegrees = true;
    DB::addProperty<P>(caster, &P::setCasterAngle, &P::getCasterAngle);
    PropertyInfo maxLean = PropertyInfo::range(PropertyType::Float, "motorcycle_max_lean_angle",
            0.0, 90.0, 0.1, true);
    maxLean.radiansAsDegrees = true;
    DB::addProperty<P>(maxLean, &P::setMaxLeanAngle, &P::getMaxLeanAngle);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "motorcycle_lean_spring", 0.0,
                               1000000.0, 0.1, true),
            &P::setLeanSpringConstant, &P::getLeanSpringConstant);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "motorcycle_lean_damping", 0.0,
                               1000000.0, 0.1, true),
            &P::setLeanDamping, &P::getLeanDamping);
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "motorcycle_lean_controller_enabled"),
            &P::setLeanControllerEnabled, &P::isLeanControllerEnabled);
}

} // namespace prisma3d
