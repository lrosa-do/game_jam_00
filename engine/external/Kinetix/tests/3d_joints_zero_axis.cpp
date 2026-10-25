#include "check.h"

#include <dynamics/RigidBody.h>
#include <dynamics/SliderJoint.h>
#include <dynamics/UniversalJoint.h>

using namespace kinetix;
using namespace kinetix::Physics;

namespace
{

bool finite(const kinetix::Math::vec3& v)
{
    return isfinite(v.x) && isfinite(v.y) && isfinite(v.z);
}

bool finite(const kinetix::Math::quat& q)
{
    return isfinite(q.w) && isfinite(q.x) && isfinite(q.y) && isfinite(q.z);
}

void makeDynamicBox(RigidBody& body)
{
    body.setBodyType(BodyType::Dynamic);
    body.setMass(1.0f);
    body.setInertiaTensor(Inertia::box(1.0f, kinetix::Math::vec3(0.5f)));
}

template <typename JointType>
void stepJoint(JointType& joint)
{
    for (int i = 0; i < 4; ++i)
    {
        joint.setup(1.0f / 60.0f);
        joint.warmStart();
        joint.solveVelocity();
        joint.solvePosition(0.2f);
    }
}

bool bodyIsFinite(const RigidBody& body)
{
    return finite(body.position()) && finite(body.orientation()) && finite(body.velocity()) &&
           finite(body.angularVelocity());
}

void sliderWorldAxis()
{
    RigidBody a;
    a.setBodyType(BodyType::Static);
    RigidBody b;
    makeDynamicBox(b);
    b.setPosition(kinetix::Math::vec3(0.0f, -1.0f, 0.0f));
    b.setVelocity(kinetix::Math::vec3(1.0f, 2.0f, 3.0f));
    SliderJoint joint(a, b, kinetix::Math::vec3(0.0f), kinetix::Math::vec3(0.0f));
    joint.setLimits(-0.5f, 0.5f);
    stepJoint(joint);
    CHECK(bodyIsFinite(b));
    CHECK(isfinite(joint.currentPosition()));
}

void sliderLocalAxes()
{
    RigidBody a;
    a.setBodyType(BodyType::Static);
    RigidBody b;
    makeDynamicBox(b);
    b.setPosition(kinetix::Math::vec3(0.0f, -1.0f, 0.0f));
    b.setVelocity(kinetix::Math::vec3(1.0f, 2.0f, 3.0f));
    SliderJoint joint(a, kinetix::Math::vec3(0.0f), kinetix::Math::vec3(0.0f),
                      kinetix::Math::vec3(0.0f), b, kinetix::Math::vec3(0.0f, 1.0f, 0.0f),
                      kinetix::Math::vec3(0.0f), kinetix::Math::vec3(0.0f));
    stepJoint(joint);
    CHECK(bodyIsFinite(b));
    CHECK(isfinite(joint.currentPosition()));
}

void universalWorldAxes()
{
    RigidBody a;
    a.setBodyType(BodyType::Static);
    RigidBody b;
    makeDynamicBox(b);
    b.setPosition(kinetix::Math::vec3(0.0f, -1.0f, 0.0f));
    b.setAngularVelocity(kinetix::Math::vec3(1.0f, 2.0f, 3.0f));
    UniversalJoint joint(a, b, kinetix::Math::vec3(0.0f), kinetix::Math::vec3(0.0f),
                         kinetix::Math::vec3(0.0f));
    joint.setLimitsA(-0.5f, 0.5f);
    stepJoint(joint);
    CHECK(bodyIsFinite(b));
    CHECK(isfinite(joint.currentAngleA()));
    CHECK(isfinite(joint.currentAngleB()));
}

void universalLocalAxes()
{
    RigidBody a;
    a.setBodyType(BodyType::Static);
    RigidBody b;
    makeDynamicBox(b);
    b.setPosition(kinetix::Math::vec3(0.0f, -1.0f, 0.0f));
    b.setAngularVelocity(kinetix::Math::vec3(1.0f, 2.0f, 3.0f));
    UniversalJoint joint(a, kinetix::Math::vec3(0.0f), kinetix::Math::vec3(0.0f), b,
                         kinetix::Math::vec3(0.0f, 1.0f, 0.0f), kinetix::Math::vec3(0.0f));
    stepJoint(joint);
    CHECK(bodyIsFinite(b));
}

}

int main()
{
    sliderWorldAxis();
    sliderLocalAxes();
    universalWorldAxes();
    universalLocalAxes();
    return TEST_RESULT();
}
