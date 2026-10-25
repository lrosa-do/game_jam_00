#pragma once

#include "prisma3d/physics/PhysicsServer3D.h"
#include "prisma3d/scene/CollisionShape3D.h"
#include "prisma3d/scene/Node3D.h"

#include <ct/function.hpp>

namespace prisma3d
{

class Joint3D;

class PhysicsBody3D : public Node3D
{
public:
    PRISMA3D_NODE_CLASS(PhysicsBody3D, Node3D);
    static void bindProperties();
    static constexpr NodeClass kAncestralClass = NodeClass::PhysicsBody3D;

    ~PhysicsBody3D() override;

    PhysicsBodyId getBody() const;
    PhysicsBodyMode getMode() const;
    bool isSimulated() const;

    void setCollisionLayer(std::uint32_t layer);
    std::uint32_t getCollisionLayer() const;
    void setCollisionMask(std::uint32_t mask);
    std::uint32_t getCollisionMask() const;
    void setFriction(float friction);
    float getFriction() const;
    void setBounce(float bounce);
    float getBounce() const;

    void setContactMonitor(bool enabled, int maxContacts = 8);
    bool isContactMonitor() const;
    void setMaxContacts(int maxContacts);
    int getMaxContacts() const;

    static PhysicsBody3D* fromBody(PhysicsBodyId body);

    ct::Function<void(PhysicsBody3D* other, const PhysicsContact& contact)> bodyEntered;
    ct::Function<void(PhysicsBody3D* other, const PhysicsContact& contact)> bodyExited;
    ct::Function<void(bool sleeping)> sleepingChanged;

protected:
    explicit PhysicsBody3D(PhysicsBodyMode mode);

    void notificationHandler(int what) override;
    void validateProperty(PropertyInfo& info) const override;
    virtual void bodyMoved(const Math::Mat4& frame);
    virtual void contactStarted(PhysicsBody3D* other, const PhysicsContact& contact);

    void writeTransform(const Math::Mat4& frame);
    bool ensureBody();
    virtual void pushState();
    virtual bool needsContactMonitor() const;
    void refreshContactMonitor();
    virtual void bindShape(const CollisionShape3D* ignore);

private:
    friend class CollisionShape3D;
    friend class Joint3D;

    class Listener : public PhysicsBodyListener
    {
    public:
        Listener()
                : owner(nullptr)
        {
        }

        void physicsBodyMoved(const Math::Mat4& frame) override;
        void physicsSleepingChanged(bool sleeping) override;
        void physicsContactStarted(const PhysicsContact& contact) override;
        void physicsContactEnded(const PhysicsContact& contact) override;

        PhysicsBody3D* owner;
    };

    bool createBody();
    void enterWorld();
    void pushTransform();
    void syncToServer();
    void attachJoint(Joint3D* joint);
    void detachJoint(Joint3D* joint);
    void releaseJoints(bool forget);

    Listener mListener;
    PhysicsBodyId mBody;
    PhysicsBodyMode mMode;
    Math::Mat4 mSynced;
    bool mHasSynced;
    bool mContactMonitor;
    int mMaxContacts;
    std::uint32_t mLayer;
    std::uint32_t mMask;
    float mFriction;
    float mBounce;
    ct::Vector<Joint3D*> mJoints;
};

class StaticBody3D : public PhysicsBody3D
{
public:
    PRISMA3D_NODE_CLASS(StaticBody3D, PhysicsBody3D);
    static void bindProperties();

    StaticBody3D();
};

class AnimatableBody3D : public PhysicsBody3D
{
public:
    PRISMA3D_NODE_CLASS(AnimatableBody3D, PhysicsBody3D);
    static void bindProperties();

    AnimatableBody3D();
};

class RigidBody3D : public PhysicsBody3D
{
public:
    PRISMA3D_NODE_CLASS(RigidBody3D, PhysicsBody3D);
    static void bindProperties();

    RigidBody3D();

    void setMass(float mass);
    float getMass() const;
    void setInertia(const Math::Vec3& inertia);
    Math::Vec3 getInertia() const;
    Math::Vec3 getComputedInertia() const;
    void setLockRotation(bool lock);
    bool isLockRotation() const;
    void setLinearDamp(float damp, PhysicsDampMode mode = PhysicsDampMode::Combine);
    float getLinearDamp() const;
    void setLinearDampMode(PhysicsDampMode mode);
    PhysicsDampMode getLinearDampMode() const;
    void setAngularDamp(float damp, PhysicsDampMode mode = PhysicsDampMode::Combine);
    float getAngularDamp() const;
    void setAngularDampMode(PhysicsDampMode mode);
    PhysicsDampMode getAngularDampMode() const;
    void setCanSleep(bool canSleep);
    bool canSleep() const;
    void setSleeping(bool sleeping);
    bool isSleeping() const;

    void setLinearVelocity(const Math::Vec3& velocity);
    Math::Vec3 getLinearVelocity() const;
    void setAngularVelocity(const Math::Vec3& velocity);
    Math::Vec3 getAngularVelocity() const;
    Math::Vec3 getVelocityAtPosition(const Math::Vec3& globalPosition) const;

    void applyCentralImpulse(const Math::Vec3& impulse);
    void applyImpulse(const Math::Vec3& impulse, const Math::Vec3& offset);
    void applyTorqueImpulse(const Math::Vec3& impulse);
    void applyCentralForce(const Math::Vec3& force);
    void applyForce(const Math::Vec3& force, const Math::Vec3& offset);
    void applyTorque(const Math::Vec3& torque);
    void setConstantForce(const Math::Vec3& force);
    Math::Vec3 getConstantForce() const;
    void setConstantTorque(const Math::Vec3& torque);
    Math::Vec3 getConstantTorque() const;

    void setInterpolated(bool interpolated);
    bool isInterpolated() const;

    int getContactCount() const;
    void getCollidingBodies(ct::Vector<PhysicsBody3D*>& out) const;

protected:
    void notificationHandler(int what) override;
    void pushState() override;

private:
    float mMass;
    Math::Vec3 mInertia;
    bool mLockRotation;
    float mLinearDamp;
    float mAngularDamp;
    PhysicsDampMode mLinearDampMode;
    PhysicsDampMode mAngularDampMode;
    bool mCanSleep;
    bool mSleeping;
    Math::Vec3 mLinearVelocity;
    Math::Vec3 mAngularVelocity;
    Math::Vec3 mConstantForce;
    Math::Vec3 mConstantTorque;
    bool mInterpolated;
};

} // namespace prisma3d
