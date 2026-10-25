#pragma once

#include "../math.h"

namespace kx
{

    class Body;

    enum class JointType : unsigned char
    {
        Distance,
        Wheel,
        Mouse,
        Revolute,
        Gear,
        Motor
    };

    class Joint
    {
    public:
        virtual ~Joint();

        JointType Type() const { return mType; }
        Body *BodyA() const { return mBodyA; }
        Body *BodyB() const { return mBodyB; }

        virtual vec2 AnchorA() const = 0;
        virtual vec2 AnchorB() const = 0;

        bool CollideConnected() const { return mCollideConnected; }
        void SetCollideConnected(bool collide) { mCollideConnected = collide; }

    protected:
        Joint(JointType type, Body *a, Body *b);

        virtual void InitVelocity(float dt) = 0;
        virtual void SolveVelocity(float dt) = 0;
        virtual bool SolvePosition() { return true; }

        virtual bool DependsOnBody(const Body *body) const
        {
            (void)body;
            return false;
        }
        virtual bool DependsOnJoint(const Joint *joint) const
        {
            (void)joint;
            return false;
        }

        // Every body the joint's solver writes to (at most four; null bodies
        // are left out). The world joins them into one sleep island.
        virtual int ConnectedBodies(Body *out[4]) const;

        // The solver's velocity writes. Body::SetVelocity would reset the sleep timer and
        // keep jointed bodies awake; sleeping is the world's island decision.
        // Wakes the connected bodies; for setters that change what the joint drives.
        void WakeBodies();

        static void SolverSetVelocity(Body *body, const vec2 &velocity);
        static void SolverSetAngularVelocity(Body *body, float angularVelocity);

        JointType mType;
        Body *mBodyA;
        Body *mBodyB;
        bool mCollideConnected;

        friend class World;
    };

} 