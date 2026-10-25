#include "check.h"

#include <dynamics/PistonJoint.h>
#include <dynamics/RigidBody.h>

// A motor enabled on a joint whose bodies sleep must wake them, or the solver keeps skipping them.

using namespace kinetix;
using namespace kinetix::Physics;

namespace
{

void makeDynamicBox(RigidBody& body, const kinetix::Math::vec3& halfExtents, f32 mass)
{
    body.setBodyType(BodyType::Dynamic);
    body.setMass(mass);
    body.setInertiaTensor(Inertia::box(mass, halfExtents));
}

bool motorWakes(bool linear)
{
    RigidBody a;
    makeDynamicBox(a, kinetix::Math::vec3(0.5f), 1.0f);
    RigidBody b;
    makeDynamicBox(b, kinetix::Math::vec3(0.5f), 1.0f);
    b.setPosition(kinetix::Math::vec3(1.0f, 0.0f, 0.0f));
    PistonJoint joint(a, b, kinetix::Math::vec3(0.5f, 0.0f, 0.0f),
                      kinetix::Math::vec3(1.0f, 0.0f, 0.0f));
    a.setAwake(false);
    b.setAwake(false);
    if (linear)
        joint.setLinearMotor(1.0f, 100.0f);
    else
        joint.setAngularMotor(1.0f, 100.0f);
    joint.setup(1.0f / 60.0f);
    return a.awake() && b.awake();
}

bool idleJointLeavesSleepersAlone()
{
    RigidBody a;
    makeDynamicBox(a, kinetix::Math::vec3(0.5f), 1.0f);
    RigidBody b;
    makeDynamicBox(b, kinetix::Math::vec3(0.5f), 1.0f);
    b.setPosition(kinetix::Math::vec3(1.0f, 0.0f, 0.0f));
    PistonJoint joint(a, b, kinetix::Math::vec3(0.5f, 0.0f, 0.0f),
                      kinetix::Math::vec3(1.0f, 0.0f, 0.0f));
    a.setAwake(false);
    b.setAwake(false);
    joint.setup(1.0f / 60.0f);
    return !a.awake() && !b.awake();
}

}

int main()
{
    CHECK(motorWakes(true));
    CHECK(motorWakes(false));
    CHECK(idleJointLeavesSleepersAlone());
    return TEST_RESULT();
}
