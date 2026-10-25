#include "k2d/RigidBody2D.h"

#include "k2d/PhysicsWorld2D.h"

#include <cmath>

namespace k2d
{

    namespace
    {
        constexpr float kDegToRad = 0.01745329251994329577f;
        constexpr float kRadToDeg = 57.29577951308232088f;
    }

    RigidBody2D::RigidBody2D()
        : Component(Type, ComponentEventNone), mBody(nullptr), mWorld(nullptr),
          mBodyType(kx::BodyType::Dynamic), mDensity(1.0f), mFriction(0.3f), mRestitution(0.0f),
          mLinearDamping(0.0f), mAngularDamping(0.0f), mGravityScale(1.0f), mFixedRotation(false),
          mBullet(false), mPendingVelocity(0.0f, 0.0f), mPendingAngularVelocity(0.0f),
          mHasPendingVelocity(false), mHasPendingAngularVelocity(false), mNeedsRebuild(false),
          mBodyIndex(InvalidWorldIndex), mPendingIndex(InvalidWorldIndex),
          mDirtyIndex(InvalidWorldIndex)
    {
        if (PhysicsWorld2D *world = PhysicsWorld2D::Active())
        {
            mWorld = world;
            mPendingIndex = world->mPending.size();
            world->mPending.push_back(this);
        }
    }

    RigidBody2D::~RigidBody2D()
    {
        if (mWorld)
            mWorld->detach(*this);
    }

    void RigidBody2D::setBodyType(kx::BodyType type)
    {
        if (mBodyType == type)
            return;
        mBodyType = type;
        if (mWorld)
            mWorld->markDirty(*this);
        else
            mNeedsRebuild = true;
    }

    void RigidBody2D::setDensity(float density)
    {
        const float clamped = density > 0.0f ? density : 0.0f;
        if (mDensity == clamped)
            return;
        mDensity = clamped;
        if (mWorld)
            mWorld->markDirty(*this);
        else
            mNeedsRebuild = true;
    }

    void RigidBody2D::setFriction(float friction)
    {
        mFriction = friction > 0.0f ? friction : 0.0f;
        if (mBody)
            mBody->SetFriction(mFriction);
    }

    void RigidBody2D::setRestitution(float restitution)
    {
        mRestitution = restitution > 0.0f ? restitution : 0.0f;
        if (mBody)
            mBody->SetRestitution(mRestitution);
    }

    void RigidBody2D::setLinearDamping(float damping)
    {
        mLinearDamping = damping > 0.0f ? damping : 0.0f;
        if (mBody)
            mBody->SetLinearDamping(mLinearDamping);
    }

    void RigidBody2D::setAngularDamping(float damping)
    {
        mAngularDamping = damping > 0.0f ? damping : 0.0f;
        if (mBody)
            mBody->SetAngularDamping(mAngularDamping);
    }

    void RigidBody2D::setGravityScale(float scale)
    {
        mGravityScale = scale;
        if (mBody)
            mBody->SetGravityScale(mGravityScale);
    }

    void RigidBody2D::setFixedRotation(bool fixed)
    {
        mFixedRotation = fixed;
        if (mBody)
            mBody->SetFixedRotation(fixed);
    }

    void RigidBody2D::setBullet(bool bullet)
    {
        mBullet = bullet;
        if (mBody)
            mBody->SetBullet(bullet);
    }

    Math::Vec2 RigidBody2D::velocity() const
    {
        if (mBody)
            return mBody->Velocity();
        return mHasPendingVelocity ? mPendingVelocity : Math::Vec2(0.0f, 0.0f);
    }

    void RigidBody2D::setVelocity(const Math::Vec2 &velocity)
    {
        if (mBody)
        {
            mBody->SetVelocity(velocity);
            return;
        }
        mPendingVelocity = velocity;
        mHasPendingVelocity = true;
    }

    float RigidBody2D::angularVelocity() const
    {
        if (mBody)
            return mBody->AngularVelocity() * kRadToDeg;
        return mHasPendingAngularVelocity ? mPendingAngularVelocity : 0.0f;
    }

    void RigidBody2D::setAngularVelocity(float degreesPerSecond)
    {
        if (mBody)
        {
            mBody->SetAngularVelocity(degreesPerSecond * kDegToRad);
            return;
        }
        mPendingAngularVelocity = degreesPerSecond;
        mHasPendingAngularVelocity = true;
    }

    void RigidBody2D::applyForce(const Math::Vec2 &force)
    {
        if (mBody)
            mBody->ApplyForceToCenter(force);
    }

    void RigidBody2D::applyImpulse(const Math::Vec2 &impulse)
    {
        if (mBody)
            mBody->ApplyLinearImpulseToCenter(impulse);
    }

    void RigidBody2D::applyTorque(float torque)
    {
        if (mBody)
            mBody->ApplyTorque(torque);
    }

    void RigidBody2D::wake()
    {
        if (mBody)
            mBody->SetAwake(true);
    }

    void RigidBody2D::markShapesDirty()
    {
        if (mWorld)
            mWorld->markDirty(*this);
        else
            mNeedsRebuild = true;
    }

    int RigidBody2D::addBoxShape(const Math::Vec2 &size, const Math::Vec2 &offset, float density)
    {
        ShapeDesc2D shape;
        shape.kind = ShapeKind2D::Box;
        shape.size = size;
        shape.offset = offset;
        shape.density = density;
        mShapes.push_back(shape);
        markShapesDirty();
        return (int)mShapes.size() - 1;
    }

    int RigidBody2D::addCircleShape(float radius, const Math::Vec2 &offset, float density)
    {
        ShapeDesc2D shape;
        shape.kind = ShapeKind2D::Circle;
        shape.radius = radius;
        shape.offset = offset;
        shape.density = density;
        mShapes.push_back(shape);
        markShapesDirty();
        return (int)mShapes.size() - 1;
    }

    int RigidBody2D::addPolygonShape(const Math::Vec2 *points, int count, const Math::Vec2 &offset, float density)
    {
        ShapeDesc2D shape;
        shape.kind = ShapeKind2D::Polygon;
        for (int i = 0; i < count; ++i)
            shape.points.push_back(points[i]);
        shape.offset = offset;
        shape.density = density;
        mShapes.push_back(shape);
        markShapesDirty();
        return (int)mShapes.size() - 1;
    }

    int RigidBody2D::addEdgeShape(const Math::Vec2 &start, const Math::Vec2 &end, const Math::Vec2 &offset)
    {
        ShapeDesc2D shape;
        shape.kind = ShapeKind2D::Edge;
        shape.start = start;
        shape.end = end;
        shape.offset = offset;
        mShapes.push_back(shape);
        markShapesDirty();
        return (int)mShapes.size() - 1;
    }

    int RigidBody2D::addChainShape(const Math::Vec2 *points, int count, bool loop, const Math::Vec2 &offset)
    {
        ShapeDesc2D shape;
        shape.kind = ShapeKind2D::Chain;
        for (int i = 0; i < count; ++i)
            shape.points.push_back(points[i]);
        shape.loop = loop;
        shape.offset = offset;
        mShapes.push_back(shape);
        markShapesDirty();
        return (int)mShapes.size() - 1;
    }

    void RigidBody2D::removeShape(int index)
    {
        if (index < 0 || (size_t)index >= mShapes.size())
            return;
        mShapes.erase(mShapes.begin() + index);
        markShapesDirty();
    }

    void RigidBody2D::clearShapes()
    {
        if (mShapes.empty())
            return;
        mShapes.clear();
        markShapesDirty();
    }

    void RigidBody2D::setShapeOffset(int index, const Math::Vec2 &offset)
    {
        if (index < 0 || (size_t)index >= mShapes.size())
            return;
        mShapes[index].offset = offset;
        markShapesDirty();
    }

    void RigidBody2D::setShapeDensity(int index, float density)
    {
        if (index < 0 || (size_t)index >= mShapes.size())
            return;
        mShapes[index].density = density;
        markShapesDirty();
    }

    void RigidBody2D::setShapeSensor(int index, bool sensor)
    {
        if (index < 0 || (size_t)index >= mShapes.size())
            return;
        mShapes[index].sensor = sensor;
        markShapesDirty();
    }

    void RigidBody2D::setShapeFilter(int index, uint16_t category, uint16_t mask)
    {
        if (index < 0 || (size_t)index >= mShapes.size())
            return;
        mShapes[index].category = category;
        mShapes[index].mask = mask;
        markShapesDirty();
    }

}
