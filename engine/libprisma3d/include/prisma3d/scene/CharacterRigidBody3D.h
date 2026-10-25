#pragma once

#include "prisma3d/physics/PhysicsServer3D.h"
#include "prisma3d/scene/PhysicsBody3D.h"

namespace prisma3d
{

class CharacterRigidBody3D : public RigidBody3D
{
public:
    PRISMA3D_NODE_CLASS(CharacterRigidBody3D, RigidBody3D);
    static void bindProperties();

    CharacterRigidBody3D();
    ~CharacterRigidBody3D() override;

    void setMaxSlopeAngle(float degrees);
    float getMaxSlopeAngle() const;
    void setSupportingHeight(float height);
    float getSupportingHeight() const;
    void setMaxSeparation(float distance);
    float getMaxSeparation() const;
    void setUpDirection(const Math::Vec3& up);
    const Math::Vec3& getUpDirection() const;

    void probeGround();
    PhysicsGroundState getGroundState() const;
    bool isOnGround() const;
    bool isSupported() const;
    const Math::Vec3& getGroundNormal() const;
    const Math::Vec3& getGroundPosition() const;
    const Math::Vec3& getGroundVelocity() const;
    PhysicsBody3D* getGroundBody() const;

protected:
    void notificationHandler(int what) override;

private:
    void release();
    void applyParameters();

    PhysicsCharacterRigidParameters mParameters;
    PhysicsCharacterRigidId mController;
    PhysicsGroundInfo mGround;
};

} // namespace prisma3d
