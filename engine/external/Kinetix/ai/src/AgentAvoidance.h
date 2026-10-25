#ifndef KINETIX_AI_AGENT_AVOIDANCE_H
#define KINETIX_AI_AGENT_AVOIDANCE_H

// Internal to the AI implementation, not a public header.

namespace kinetix::AI
{
class Entity;
}

namespace kinetix::AI::detail
{

// Pushes `entity` away from every other entity inside `avoidDistance`, then bends its desired move
// toward that escape by `turnRate` (0 leaves the move alone, 1 replaces it). Repulsion is summed
// over all neighbours and weighted (1 - d/r), so it grows as they close; two entities in exactly
// the same place take deterministically opposite directions.
void applyAgentAvoidance(Entity& entity, float avoidDistance, float turnRate);

}

#endif
