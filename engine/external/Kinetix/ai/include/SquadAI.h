#ifndef KINETIX_AI_SQUADAI_H
#define KINETIX_AI_SQUADAI_H

// Ownership of the returned StateMachine passes to the caller: assign it with
// SquadEntity::setStateMachine and delete it before the entity dies (the callbacks capture the
// entity by reference).

#include "SquadEntity.h"
#include "StateMachine.h"

namespace kinetix::AI
{

// Leader machine: AwaitingSquadTaskCompletion <-> CommandSquadToPOI <-> StandingGround.
StateMachine* buildLeaderStateMachine(SquadLeaderEntity& leader);

// Member machine: WaitingForCommand -> MovingToGoal -> WaypointReached.
StateMachine* buildMemberStateMachine(SquadEntity& member);

}

#endif
