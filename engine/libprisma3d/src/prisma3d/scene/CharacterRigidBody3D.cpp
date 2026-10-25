#include "prisma3d/scene/CharacterRigidBody3D.h"

#include "prisma3d/scene/NodeClassDB.h"

namespace prisma3d
{

CharacterRigidBody3D::CharacterRigidBody3D()
        : RigidBody3D()
{
    setLockRotation(true);
    setFriction(0.2f);
    setMass(80.0f);
    setPhysicsProcess(true);
}

CharacterRigidBody3D::~CharacterRigidBody3D()
{
    release();
}

void CharacterRigidBody3D::release()
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (server && mController.valid()) server->characterRigidFree(mController);
    mController = PhysicsCharacterRigidId();
}

void CharacterRigidBody3D::applyParameters()
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (server && server->characterRigidValid(mController))
        server->characterRigidSetParameters(mController, mParameters);
}

void CharacterRigidBody3D::setMaxSlopeAngle(float degrees)
{
    mParameters.maxSlopeAngle = degrees;
    applyParameters();
}

float CharacterRigidBody3D::getMaxSlopeAngle() const
{
    return mParameters.maxSlopeAngle;
}

void CharacterRigidBody3D::setSupportingHeight(float height)
{
    mParameters.supportingHeight = height;
    applyParameters();
}

float CharacterRigidBody3D::getSupportingHeight() const
{
    return mParameters.supportingHeight;
}

void CharacterRigidBody3D::setMaxSeparation(float distance)
{
    mParameters.maxSeparation = distance;
    applyParameters();
}

float CharacterRigidBody3D::getMaxSeparation() const
{
    return mParameters.maxSeparation;
}

void CharacterRigidBody3D::setUpDirection(const Math::Vec3& up)
{
    mParameters.up = up;
    applyParameters();
}

const Math::Vec3& CharacterRigidBody3D::getUpDirection() const
{
    return mParameters.up;
}

void CharacterRigidBody3D::probeGround()
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (!server || !isSimulated())
    {
        mGround = PhysicsGroundInfo();
        return;
    }
    if (!server->characterRigidValid(mController))
        mController = server->characterRigidCreate(getBody(), mParameters);
    mGround = server->characterRigidProbe(mController);
}

PhysicsGroundState CharacterRigidBody3D::getGroundState() const
{
    return mGround.state;
}

bool CharacterRigidBody3D::isOnGround() const
{
    return mGround.state == PhysicsGroundState::OnGround;
}

bool CharacterRigidBody3D::isSupported() const
{
    return mGround.state == PhysicsGroundState::OnGround ||
           mGround.state == PhysicsGroundState::OnSteepGround;
}

const Math::Vec3& CharacterRigidBody3D::getGroundNormal() const
{
    return mGround.normal;
}

const Math::Vec3& CharacterRigidBody3D::getGroundPosition() const
{
    return mGround.position;
}

const Math::Vec3& CharacterRigidBody3D::getGroundVelocity() const
{
    return mGround.velocity;
}

PhysicsBody3D* CharacterRigidBody3D::getGroundBody() const
{
    return PhysicsBody3D::fromBody(mGround.body);
}

void CharacterRigidBody3D::notificationHandler(int what)
{
    RigidBody3D::notificationHandler(what);
    switch (what)
    {
        case kNotificationPhysicsProcess:
            probeGround();
            break;
        case kNotificationExitTree:
            release();
            mGround = PhysicsGroundInfo();
            break;
        default:
            break;
    }
}

void CharacterRigidBody3D::bindProperties()
{
    using DB = NodeClassDB;
    using P = CharacterRigidBody3D;
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "max_slope_angle", 0.0, 90.0, 0.1),
            &P::setMaxSlopeAngle, &P::getMaxSlopeAngle);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "supporting_height", 0.0, 10.0, 0.001, true),
            &P::setSupportingHeight, &P::getSupportingHeight);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "max_separation", 0.0, 10.0, 0.001, true),
            &P::setMaxSeparation, &P::getMaxSeparation);
    DB::addProperty<P>(PropertyInfo(PropertyType::Vec3, "up_direction"), &P::setUpDirection,
            &P::getUpDirection);
}

} // namespace prisma3d
