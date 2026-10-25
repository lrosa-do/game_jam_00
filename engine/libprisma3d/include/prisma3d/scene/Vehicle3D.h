#pragma once

#include "prisma3d/physics/PhysicsServer3D.h"
#include "prisma3d/scene/Node3D.h"
#include "prisma3d/scene/PhysicsBody3D.h"

#include <ct/vector.hpp>

namespace prisma3d
{

class VehicleBody3D;

class VehicleWheel3D : public Node3D
{
public:
    PRISMA3D_NODE_CLASS(VehicleWheel3D, Node3D);
    static void bindProperties();

    VehicleWheel3D();
    ~VehicleWheel3D() override;

    void setRadius(float radius);
    float getRadius() const;
    void setSuspensionRestLength(float length);
    float getSuspensionRestLength() const;
    void setSuspensionTravel(float length);
    float getSuspensionTravel() const;
    void setSuspensionStiffness(float value);
    float getSuspensionStiffness() const;
    void setSuspensionMaxForce(float value);
    float getSuspensionMaxForce() const;
    void setDampingCompression(float value);
    float getDampingCompression() const;
    void setDampingRelaxation(float value);
    float getDampingRelaxation() const;
    void setFrictionSlip(float value);
    float getFrictionSlip() const;
    void setRollInfluence(float value);
    float getRollInfluence() const;

    void setEngineForce(float force);
    float getEngineForce() const;
    void setBrake(float brake);
    float getBrake() const;
    void setSteering(float steering);
    float getSteering() const;
    void setUseAsTraction(bool enable);
    bool isUsedAsTraction() const;
    void setUseAsSteering(bool enable);
    bool isUsedAsSteering() const;

    bool isInContact() const;
    PhysicsBody3D* getContactBody() const;
    Math::Vec3 getContactPoint() const;
    Math::Vec3 getContactNormal() const;
    float getSkidInfo() const;
    float getRpm() const;

protected:
    void notificationHandler(int what) override;

private:
    friend class VehicleBody3D;

    PhysicsWheelParameters parameters() const;
    void pushParameters();
    void captureFrame();

    VehicleBody3D* mBody;
    int mIndex;
    bool mFrameLocked;
    Math::Vec3 mConnection;
    Math::Vec3 mDirection;
    Math::Vec3 mAxle;
    float mRadius;
    float mRestLength;
    float mTravel;
    float mStiffness;
    float mMaxForce;
    float mDampingCompression;
    float mDampingRelaxation;
    float mFrictionSlip;
    float mRollInfluence;
    float mEngineForce;
    float mBrake;
    float mSteering;
    bool mTraction;
    bool mSteers;
    PhysicsWheelState mState;
    float mPreviousRotation;
    float mRpm;
};

class VehicleBody3D : public RigidBody3D
{
public:
    PRISMA3D_NODE_CLASS(VehicleBody3D, RigidBody3D);
    static void bindProperties();

    VehicleBody3D();
    ~VehicleBody3D() override;

    void setEngineForce(float force);
    float getEngineForce() const;
    void setBrake(float brake);
    float getBrake() const;
    void setSteering(float steering);
    float getSteering() const;

    PhysicsVehicleId getVehicle() const;
    float getSpeedKmh() const;
    std::size_t getWheelCount() const;
    VehicleWheel3D* getWheel(std::size_t index) const;

    void setMotorcycle(bool enabled);
    bool isMotorcycle() const;
    void setSteerInput(float input);
    float getSteerInput() const;
    void setMaxSteerAngle(float angle);
    float getMaxSteerAngle() const;
    void setCasterAngle(float angle);
    float getCasterAngle() const;
    void setMaxLeanAngle(float angle);
    float getMaxLeanAngle() const;
    void setLeanSpring(float springConstant, float damping);
    void setLeanSpringConstant(float springConstant);
    float getLeanSpringConstant() const;
    void setLeanDamping(float damping);
    float getLeanDamping() const;
    void setLeanControllerEnabled(bool enabled);
    bool isLeanControllerEnabled() const;
    float getLeanAngle() const;

protected:
    void notificationHandler(int what) override;
    void bodyMoved(const Math::Mat4& frame) override;

private:
    friend class VehicleWheel3D;

    void addWheel(VehicleWheel3D* wheel);
    void removeWheel(VehicleWheel3D* wheel);
    void rebuild();
    void releaseVehicle();
    void pushControls();
    void applyMotorcycle();

    PhysicsVehicleId mVehicle;
    ct::Vector<VehicleWheel3D*> mWheels;
    float mEngineForce;
    float mBrake;
    float mSteering;
    bool mDirty;
    bool mMotorcycle;
    float mSteerInput;
    float mMaxSteer;
    float mCaster;
    float mMaxLean;
    float mLeanK;
    float mLeanC;
    bool mLeanEnabled;
    float mStep;
};

} // namespace prisma3d
