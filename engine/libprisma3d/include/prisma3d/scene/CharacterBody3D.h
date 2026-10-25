#pragma once

#include "prisma3d/physics/PhysicsServer3D.h"
#include "prisma3d/scene/PhysicsBody3D.h"

namespace prisma3d
{

class CharacterBody3D : public PhysicsBody3D
{
public:
    PRISMA3D_NODE_CLASS(CharacterBody3D, PhysicsBody3D);
    static void bindProperties();

    CharacterBody3D();
    ~CharacterBody3D() override;

    void setVelocity(const Math::Vec3& velocity);
    const Math::Vec3& getVelocity() const;
    bool moveAndSlide(float deltaTime = 0.0f);

    bool isOnFloor() const;
    bool isOnWall() const;
    bool isOnCeiling() const;
    const Math::Vec3& getFloorNormal() const;
    const Math::Vec3& getWallNormal() const;
    const Math::Vec3& getCeilingNormal() const;
    float getFloorAngle() const;
    bool isNearGround(float distance) const;

    void setCapsule(float radius, float height);
    float getRadius() const;
    float getHeight() const;
    void setSlopeLimit(float degrees);
    float getSlopeLimit() const;
    void setStepOffset(float offset);
    float getStepOffset() const;
    void setGroundSnap(float distance);
    float getGroundSnap() const;
    void setSkinWidth(float width);
    float getSkinWidth() const;
    void setWallMinSlideAngle(float degrees);
    float getWallMinSlideAngle() const;
    void setUpDirection(const Math::Vec3& up);
    const Math::Vec3& getUpDirection() const;
    void setFloorStopOnSlope(bool enabled);
    bool isFloorStopOnSlope() const;
    void setSlideOnCeiling(bool enabled);
    bool isSlideOnCeiling() const;
    void setPushForce(float force);
    float getPushForce() const;
    void setMaxSlides(int slides);
    int getMaxSlides() const;

    std::size_t getSlideCollisionCount() const;
    PhysicsBody3D* getSlideCollider(std::size_t index) const;
    Math::Vec3 getSlideNormal(std::size_t index) const;

    PhysicsCharacterId getCharacter() const;

protected:
    void notificationHandler(int what) override;

private:
    void release();
    void gatherShape();

    PhysicsCharacterParameters mParameters;
    PhysicsCharacterId mCharacter;
    PhysicsCharacterState mState;
    Math::Vec3 mVelocity;
    float mPushForce;
    bool mHasShape;
};

} // namespace prisma3d
