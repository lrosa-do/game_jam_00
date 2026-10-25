#pragma once

#include "prisma3d/physics/PhysicsServer3D.h"
#include "prisma3d/scene/PhysicsBody3D.h"

#include <ct/string.hpp>
#include <ct/vector.hpp>

namespace prisma3d
{


class GroundVehicle3D : public RigidBody3D
{
public:
    PRISMA3D_NODE_CLASS(GroundVehicle3D, RigidBody3D);
    static void bindProperties();

    static constexpr int kWheels = 4;

    GroundVehicle3D();
    ~GroundVehicle3D() override;

    void setDrive(int drive);
    int getDrive() const;
    void setAutomatic(bool automatic);
    bool isAutomatic() const;
    void setAntiLock(bool antiLock);
    bool isAntiLock() const;
    void setWheelRayMask(std::uint32_t mask);
    std::uint32_t getWheelRayMask() const;
    void setWheelRadius(float radius);
    float getWheelRadiusSetting() const;
    void setTopSpeedKmh(float kmh);
    float getTopSpeedKmh() const;
    void setPower(float power);
    float getPower() const;
    void setGrip(float grip);
    float getGrip() const;
    void setRollInfluence(float influence);
    float getRollInfluence() const;
    void setHandbrakeTorque(float torque);
    float getHandbrakeTorque() const;
    void setAxleGrip(int axle, float scale);

    void setThrottle(float throttle);
    float getThrottle() const;
    void setBrake(float brake);
    float getBrake() const;
    void setHandbrake(float handbrake);
    float getHandbrake() const;
    void setSteering(float steering);
    float getSteering() const;

    void setWheelPath(int wheel, const ct::String& path);
    ct::String getWheelPath(int wheel) const;
    void setWheelFrontLeft(const ct::String& path);
    ct::String getWheelFrontLeft() const;
    void setWheelFrontRight(const ct::String& path);
    ct::String getWheelFrontRight() const;
    void setWheelRearLeft(const ct::String& path);
    ct::String getWheelRearLeft() const;
    void setWheelRearRight(const ct::String& path);
    ct::String getWheelRearRight() const;
    void setSkidMarksPath(const ct::String& path);
    const ct::String& getSkidMarksPath() const;
    void setCollisionMesh(const ct::String& path);
    const ct::String& getCollisionMesh() const;
    void setCollisionScale(const Math::Vec3& scale);
    const Math::Vec3& getCollisionScale() const;
    void setCollisionOffset(const Math::Vec3& offset);
    const Math::Vec3& getCollisionOffset() const;

    PhysicsGroundVehicleId getVehicle() const;
    const PhysicsGroundVehicleLayout& getLayout() const;
    Math::Vec3 getCollisionHalfExtents() const;
    float getRestHeight() const;
    float getSpeedKmh() const;
    float getForwardSpeed() const;
    float getRpm() const;
    int getGear() const;
    int getWheelCount() const;
    bool isWheelInContact(int wheel) const;
    float getWheelSlipRatio(int wheel) const;
    float getWheelSlipAngle(int wheel) const;
    float getWheelSlide(int wheel) const;
    float getWheelRadius(int wheel) const;
    Math::Mat4 getWheelLocalTransform(int wheel) const;
    bool isWheelDetached(int wheel) const;
    void setWheelDetached(int wheel, bool detached);
    void setWheelRadiusScale(int wheel, float scale);
    void stopWheels();
    void placeOnGround(const Math::Vec3& at, const Math::Vec3& forward);
    void park();
    bool isReversing() const;
    void setLengthwiseExtent(float rear, float front);

protected:
    void notificationHandler(int what) override;
    void bodyMoved(const Math::Mat4& frame) override;
    void bindShape(const CollisionShape3D* ignore) override;

private:
    void build();
    void release();
    void pushControls();
    void placeWheels();
    void fitAxles();
    void updateSkids();
    void loadHull();
    PhysicsShapeId makeHull(float rear, float front) const;
    Node3D* wheelNode(int wheel) const;

    PhysicsGroundVehicleId mVehicle;
    PhysicsGroundVehicleLayout mLayout;
    PhysicsShapeId mShape;
    PhysicsGroundVehicleControls mControls;
    ct::String mWheelPaths[kWheels];
    ct::String mSkidMarksPath;
    ct::String mCollisionMesh;
    Math::Vec3 mCollisionScale;
    Math::Vec3 mCollisionOffset;
    ct::Vector<Math::Vec3> mHullPoints;
    bool mHullLoaded;
    float mReverseHold;
    int mSkidSlot;
    std::uint32_t mRayMask;
    float mWheelRadius;
    float mPower;
    float mTopSpeedKmh;
    float mGrip;
    float mRollInfluence;
    float mHandbrakeTorque;
    int mDrive;
    bool mAutomatic;
    bool mAntiLock;
    bool mDirty;
    bool mParked;
    bool mReverse;
};

} // namespace prisma3d
