#pragma once

#include "math.h"
#include <stdint.h>

#include "common.h"
#include "shapes.h"

namespace kx
{

    struct ContactEvent;
    typedef void (*ContactCallback)(const ContactEvent &event, void *context);

    enum class BodyType : unsigned char
    {
        Static,
        Kinematic,
        Dynamic
    };

    enum class ShapeType : unsigned char
    {
        Circle,
        Polygon,
        Edge
    };

    struct Filter
    {
        uint16_t category;
        uint16_t mask;
        int16_t group;

        Filter() : category(1), mask(0xFFFF), group(0)
        {
        }
    };

    inline bool ShouldCollide(const Filter &a, const Filter &b)
    {
        if (a.group == b.group && a.group != 0)
            return a.group > 0;
        return (a.mask & b.category) != 0 && (b.mask & a.category) != 0;
    }

    struct Shape
    {
        ShapeType type;
        float density;
        Filter filter;
        bool isSensor;
        void *userData;
        Circle circle;
        Polygon polygon;
        Edge edge;
    };

    inline float ShapeRadius(const Shape &shape)
    {
        switch (shape.type)
        {
        case ShapeType::Circle:
            return shape.circle.radius;
        case ShapeType::Polygon:
            return shape.polygon.radius;
        default:
            return shape.edge.radius;
        }
    }

    class Body
    {
    public:
        static constexpr int kMaxShapes = 32;

        Body();

        Transform GetTransform() const { return MakeTransform(mPosition, mAngle); }

        const vec2 &Position() const { return mPosition; }
        void SetPosition(const vec2 &position) { mPosition = position; SetAwake(true); }

        float Angle() const { return mAngle; }
        void SetAngle(float angle) { mAngle = angle; SetAwake(true); }

        const vec2 &Velocity() const { return mLinearVelocity; }
        void SetVelocity(const vec2 &v) { mLinearVelocity = v; SetAwake(true); }

        float AngularVelocity() const { return mAngularVelocity; }
        void SetAngularVelocity(float w) { mAngularVelocity = w; SetAwake(true); }

        bool FixedRotation() const { return mFixedRotation; }
        void SetFixedRotation(bool fixed)
        {
            mFixedRotation = fixed;
            SetAwake(true);
            RecomputeMass();
        }

        bool IsBullet() const { return mBullet; }
        void SetBullet(bool bullet) { mBullet = bullet; }

        bool IsAwake() const { return mAwake; }
        void SetAwake(bool awake)
        {
            mAwake = awake;
            if (awake)
                mSleepTime = 0.0f;
            else
            {
                mLinearVelocity = vec2(0.0f);
                mAngularVelocity = 0.0f;
            }
        }

        float Friction() const { return mFriction; }
        void SetFriction(float friction) { mFriction = friction; }

        float Restitution() const { return mRestitution; }
        void SetRestitution(float restitution) { mRestitution = restitution; }

        float LinearDamping() const { return mLinearDamping; }
        void SetLinearDamping(float damping) { mLinearDamping = damping > 0.0f ? damping : 0.0f; }

        float AngularDamping() const { return mAngularDamping; }
        void SetAngularDamping(float damping) { mAngularDamping = damping > 0.0f ? damping : 0.0f; }

        float GravityScale() const { return mGravityScale; }
        void SetGravityScale(float scale) { mGravityScale = scale; }

        float Mass() const { return mInvMass > 0.0f ? 1.0f / mInvMass : 0.0f; }
        float InvMass() const { return mInvMass; }
        float InvI() const { return mInvI; }

        vec2 WorldCenter() const { return GetTransform().Transform(mLocalCenter); }

        void ApplyImpulse(const vec2 &impulse, const vec2 &point)
        {
            if (Dot(impulse, impulse) > 0.0f)
                SetAwake(true);
            mLinearVelocity += mInvMass * impulse;
            mAngularVelocity += mInvI * Cross(point - WorldCenter(), impulse);
        }

        void ApplyLinearImpulseToCenter(const vec2 &impulse, bool wake = true)
        {
            if (wake && Dot(impulse, impulse) > 0.0f)
                SetAwake(true);
            mLinearVelocity += mInvMass * impulse;
        }

        void ApplyAngularImpulse(float impulse, bool wake = true)
        {
            if (wake && impulse != 0.0f)
                SetAwake(true);
            mAngularVelocity += mInvI * impulse;
        }

        void ApplyForce(const vec2 &force, const vec2 &point, bool wake = true)
        {
            if (mType != BodyType::Dynamic)
                return;
            if (wake)
                SetAwake(true);
            if (!mAwake)
                return;
            mForce += force;
            mTorque += Cross(point - WorldCenter(), force);
        }

        void ApplyForceToCenter(const vec2 &force, bool wake = true)
        {
            if (mType != BodyType::Dynamic)
                return;
            if (wake)
                SetAwake(true);
            if (!mAwake)
                return;
            mForce += force;
        }

        void ApplyTorque(float torque, bool wake = true)
        {
            if (mType != BodyType::Dynamic)
                return;
            if (wake)
                SetAwake(true);
            if (!mAwake)
                return;
            mTorque += torque;
        }

        BodyType Type() const { return mType; }
        uint32_t Id() const { return mId; }

        void SetUserData(void *userData) { mUserData = userData; }
        void *UserData() const { return mUserData; }

        void SetContactCallback(ContactCallback callback, void *context = nullptr)
        {
            mContactCallback = callback;
            mContactContext = context;
        }

        bool SetSensor(int shapeIndex, bool sensor)
        {
            if (shapeIndex < 0 || shapeIndex >= mShapeCount)
                return false;
            mShapes[shapeIndex].isSensor = sensor;
            SetAwake(true);
            return true;
        }

        bool IsSensor(int shapeIndex) const
        {
            return shapeIndex >= 0 && shapeIndex < mShapeCount && mShapes[shapeIndex].isSensor;
        }

        bool SetShapeUserData(int shapeIndex, void *userData)
        {
            if (shapeIndex < 0 || shapeIndex >= mShapeCount)
                return false;
            mShapes[shapeIndex].userData = userData;
            return true;
        }

        void *ShapeUserData(int shapeIndex) const
        {
            return shapeIndex >= 0 && shapeIndex < mShapeCount ? mShapes[shapeIndex].userData : nullptr;
        }

        const Shape *Shapes() const { return mShapes; }
        int ShapeCount() const { return mShapeCount; }
        const AABB &TightAABB() const { return mTightAABB; }

        void SetFilter(uint16_t category, uint16_t mask, int16_t group = 0)
        {
            mDefaultFilter.category = category;
            mDefaultFilter.mask = mask;
            mDefaultFilter.group = group;
            for (int i = 0; i < mShapeCount; ++i)
                mShapes[i].filter = mDefaultFilter;
        }

        bool SetShapeFilter(int shapeIndex, uint16_t category, uint16_t mask, int16_t group = 0)
        {
            if (shapeIndex < 0 || shapeIndex >= mShapeCount)
                return false;
            mShapes[shapeIndex].filter.category = category;
            mShapes[shapeIndex].filter.mask = mask;
            mShapes[shapeIndex].filter.group = group;
            SetAwake(true);
            return true;
        }

        Filter ShapeFilter(int shapeIndex) const
        {
            return shapeIndex >= 0 && shapeIndex < mShapeCount ? mShapes[shapeIndex].filter
                                                              : Filter();
        }

        int AddCircle(const vec2 &localCenter, float radius, float density);
        int AddBox(float halfWidth, float halfHeight, const vec2 &localCenter, float density);
        int AddEdge(const vec2 &localA, const vec2 &localB);
        // Adds a copy of an edge as it is, ghost vertices and one-sidedness
        // included (AddEdge(a, b) always makes a two-sided edge).
        int AddEdge(const Edge &edge);

        int AddChain(const vec2 *points, int count, bool loop);

        // AddPolygon uses the convex hull of the points. It returns 0 and adds
        // nothing when the points are degenerate or their hull has more than
        // kMaxPolygonVertices vertices.
        int AddPolygon(const vec2 *points, int count, float density);
        // AddMesh and AddFromImage are all-or-nothing: they return the number of convex
        // pieces added, or 0 with the body unchanged when the outline cannot be
        // triangulated or the pieces do not fit in the remaining shapes. AddFromImage
        // retries with coarser outlines (Douglas-Peucker, 0.5 to 8 pixels) before failing.
        int AddMesh(const vec2 *outline, int count, float density);
        int AddFromImage(const unsigned char *pixels, int width, int height, int bpp,
                         unsigned char threshold, float density, float scale = 1.0f, float simplifyDegrees = 2.0f);

        // True once World::Destroy was called during a step or event dispatch: the body
        // stays allocated until the destruction is applied, but gets no more contact
        // callbacks and cannot be sliced or destroyed again.
        bool IsDestroyPending() const { return mDestroyPending; }

        void IntegrateVelocity(const vec2 &gravity, float dt);
        void IntegratePosition(float dt);
        void ShiftCenter(const vec2 &deltaCenter, float deltaAngle);

    private:
        void RecomputeMass();
        // Pieces added; 0 when every piece was a degenerate sliver; -1 when
        // the triangulation failed, -2 when the pieces do not fit in the
        // shapes left (body unchanged in both cases).
        int AddMeshPieces(const vec2 *outline, int count, const vec2 *const *holes, const int *holeCounts,
                          int holeCount, float density);

        BodyType mType;

        vec2 mPosition;
        float mAngle;

        vec2 mLinearVelocity;
        float mAngularVelocity;
        float mSleepTime;
        bool mAwake;
        bool mFixedRotation;
        bool mBullet;

        float mInvMass;
        float mInvI;
        vec2 mLocalCenter;
        float mFriction;
        float mRestitution;
        float mLinearDamping;
        float mAngularDamping;
        float mGravityScale;
        vec2 mForce;
        float mTorque;
        uint32_t mId;
        void *mUserData;
        ContactCallback mContactCallback;
        void *mContactContext;

        int32_t mProxyId;
        vec2 mProxyPosition;
        AABB mTightAABB;

        Filter mDefaultFilter;
        Shape mShapes[kMaxShapes];
        int mShapeCount;

        int32_t mIslandIndex;
        bool mDestroyPending;

        friend class World;
        friend class Joint;
    };

} 