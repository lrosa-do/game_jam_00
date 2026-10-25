#include "kx/joints/joint.h"

#include "kx/body.h"

namespace kx
{

    Joint::Joint(JointType type, Body *a, Body *b)
        : mType(type), mBodyA(a), mBodyB(b), mCollideConnected(false)
    {
    }

    Joint::~Joint()
    {
    }

    int Joint::ConnectedBodies(Body *out[4]) const
    {
        int count = 0;
        if (mBodyA)
            out[count++] = mBodyA;
        if (mBodyB)
            out[count++] = mBodyB;
        return count;
    }

    void Joint::WakeBodies()
    {
        Body *bodies[4];
        const int count = ConnectedBodies(bodies);
        for (int i = 0; i < count; ++i)
            if (bodies[i]->Type() == BodyType::Dynamic)
                bodies[i]->SetAwake(true);
    }

    void Joint::SolverSetVelocity(Body *body, const vec2 &velocity)
    {
        body->mLinearVelocity = velocity;
    }

    void Joint::SolverSetAngularVelocity(Body *body, float angularVelocity)
    {
        body->mAngularVelocity = angularVelocity;
    }

} 