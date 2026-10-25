#include "dynamics/Joint.h"

#include "dynamics/RigidBody.h"

namespace kinetix::Physics
{

void Joint::wakeBody(RigidBody* body)
{
    body->setAwake(true);
}

}
