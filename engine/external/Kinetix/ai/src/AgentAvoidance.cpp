#include "AgentAvoidance.h"

#include "Entity.h"
#include "Group.h"
#include "World.h"

#include <kinetix/core/Math.h>

namespace kinetix::AI::detail
{

void applyAgentAvoidance(Entity& entity, float avoidDistance, float turnRate)
{
    if (avoidDistance <= 0.0f)
        return;

    Math::vec3 repulsion(0.0f);
    const Math::vec3 position = entity.position();

    // Only the agents the radius can reach, from the grid the world rebuilt this frame.
    ct::Vector<Entity*> neighbours;
    entity.world().proximityGrid().query(position, avoidDistance, neighbours);

    for (Entity* other : neighbours)
    {
        if (other == &entity)
            continue;

        Math::vec3 away = position - other->position();
        away.y = 0.0f;
        const float distance = Math::length(away);
        if (distance >= avoidDistance)
            continue;

        if (distance > 1e-5f)
            repulsion += (away / distance) * (1.0f - distance / avoidDistance);
        else
            // Coincident entities need opposite deterministic directions,
            // or both pick the same escape and stay stuck together.
            repulsion += (&entity < other) ? entity.side() : -entity.side();
    }

    const float repulsionLength = Math::length(repulsion);
    if (repulsionLength <= 1e-5f)
        return;

    Math::vec3 desired = entity.desiredMove();
    const float desiredLength = Math::length(desired);
    const Math::vec3 escape = repulsion / repulsionLength;
    const float weight = Math::clamp(turnRate, 0.0f, 1.0f);

    if (desiredLength > 1e-5f)
    {
        const Math::vec3 direction = desired / desiredLength;
        Math::vec3 blended = direction * (1.0f - weight) + escape * weight;
        if (Math::length(blended) <= 1e-5f)
        {
            // Heading straight into the neighbour with an even blend: the two cancel and
            // normalising nothing is a NaN. Step aside instead - the escape turned a quarter about
            // up, or the entity's own side if the escape is vertical.
            blended = Math::vec3(-escape.z, 0.0f, escape.x);
            if (Math::length(blended) <= 1e-5f)
                blended = entity.side();
        }
        desired = Math::normalize(blended) * desiredLength;
    }
    else
        desired = escape * turnRate;

    entity.setDesiredMove(desired);
}

}
