#pragma once

#include "k2d/Component.h"
#include "k2d/ShapeDesc2D.h"

#include <kx/body.h>

#include <cstddef>
#include <limits>

namespace k2d
{

    class PhysicsWorld2D;

    class RigidBody2D : public Component
    {
    public:
        static const ComponentType Type = ComponentType::RigidBody;

        RigidBody2D();
        ~RigidBody2D() override;

        int addBoxShape(const Math::Vec2 &size, const Math::Vec2 &offset = Math::Vec2(0.0f, 0.0f), float density = 1.0f);
        int addCircleShape(float radius, const Math::Vec2 &offset = Math::Vec2(0.0f, 0.0f), float density = 1.0f);
        int addPolygonShape(const Math::Vec2 *points, int count, const Math::Vec2 &offset = Math::Vec2(0.0f, 0.0f), float density = 1.0f);
        int addEdgeShape(const Math::Vec2 &start, const Math::Vec2 &end, const Math::Vec2 &offset = Math::Vec2(0.0f, 0.0f));
        int addChainShape(const Math::Vec2 *points, int count, bool loop, const Math::Vec2 &offset = Math::Vec2(0.0f, 0.0f));
        void removeShape(int index);
        void clearShapes();

        void setShapeOffset(int index, const Math::Vec2 &offset);
        void setShapeDensity(int index, float density);
        void setShapeSensor(int index, bool sensor);
        void setShapeFilter(int index, uint16_t category, uint16_t mask);

        size_t shapeCount() const { return mShapes.size(); }
        const ShapeDesc2D &shapeAt(int index) const { return mShapes[index]; }
        const ct::Vector<ShapeDesc2D> &shapes() const { return mShapes; }

        kx::BodyType bodyType() const { return mBodyType; }
        void setBodyType(kx::BodyType type);

        float density() const { return mDensity; }
        void setDensity(float density);
        float friction() const { return mFriction; }
        void setFriction(float friction);
        float restitution() const { return mRestitution; }
        void setRestitution(float restitution);
        float linearDamping() const { return mLinearDamping; }
        void setLinearDamping(float damping);
        float angularDamping() const { return mAngularDamping; }
        void setAngularDamping(float damping);
        float gravityScale() const { return mGravityScale; }
        void setGravityScale(float scale);
        bool fixedRotation() const { return mFixedRotation; }
        void setFixedRotation(bool fixed);
        bool bullet() const { return mBullet; }
        void setBullet(bool bullet);

        Math::Vec2 velocity() const;
        void setVelocity(const Math::Vec2 &velocity);
        float angularVelocity() const;
        void setAngularVelocity(float degreesPerSecond);

        void applyForce(const Math::Vec2 &force);
        void applyImpulse(const Math::Vec2 &impulse);
        void applyTorque(float torque);
        void wake();

        kx::Body *body() const { return mBody; }
        bool inWorld() const { return mBody != nullptr; }

    private:
        friend class PhysicsWorld2D;

        void markShapesDirty();

        kx::Body *mBody;
        PhysicsWorld2D *mWorld;
        kx::BodyType mBodyType;
        float mDensity;
        float mFriction;
        float mRestitution;
        float mLinearDamping;
        float mAngularDamping;
        float mGravityScale;
        bool mFixedRotation;
        bool mBullet;

        Math::Vec2 mPendingVelocity;
        float mPendingAngularVelocity;
        bool mHasPendingVelocity;
        bool mHasPendingAngularVelocity;
        bool mNeedsRebuild;
        static constexpr std::size_t InvalidWorldIndex = (std::numeric_limits<std::size_t>::max)();
        std::size_t mBodyIndex;
        std::size_t mPendingIndex;
        std::size_t mDirtyIndex;
        ct::Vector<ShapeDesc2D> mShapes;
    };

}
