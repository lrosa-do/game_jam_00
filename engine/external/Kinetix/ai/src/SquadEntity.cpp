#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>
#include <kinetix/core/Log.h>

#include "SquadEntity.h"

#include "PointOfInterest.h"
#include "StateMachine.h"
#include "WaypointNetwork.h"

#include <stdlib.h>

#include <kinetix/core/Algorithm.h>

namespace kinetix::AI
{

WaypointID selectRandomWaypoint(const WaypointNetwork& network)
{
    const auto& waypoints = network.waypoints();
    if (waypoints.empty())
        return 0;

    ct::Vector<WaypointID> ids;
    ids.reserve(waypoints.size());
    for (const auto& entry : waypoints)
        ids.push_back(entry.key);
    return ids[rand() % static_cast<int>(ids.size())];
}

SquadEntity::SquadEntity(World& world, const Settings& settings) : Entity(world, settings)
{
    mKind = EntityKind::Squad;
}

SquadEntity::~SquadEntity()
{
    if (mSquadLeader)
        mSquadLeader->removeSquadMember(this);
}

void SquadEntity::iterate(float timeDelta)
{
    if (mStateMachine)
        mStateMachine->iterate();
    Entity::iterate(timeDelta);
}

namespace
{
constexpr float kEntityRadius = 0.75f;
}

bool SquadEntity::waypointReached()
{
    if (mNextWaypoint != 0 && mWaypointNetwork)
    {
        Waypoint* wp = mWaypointNetwork->findWaypoint(mNextWaypoint);
        if (wp)
        {
            Math::vec3 vec = wp->position() - position();
            vec.y = 0.0f; // XZ only
            float distToWP = Math::length(vec);
            return (distToWP - kEntityRadius) < wp->radius();
        }
    }
    return false;
}

void SquadEntity::onWaypointReached()
{
    // Intentionally empty hook.
}

bool SquadEntity::goalReached()
{
    Math::vec3 vec = mGoalPosition - position();
    vec.y = 0.0f; // XZ only
    return Math::length(vec) < mGoalRadius;
}

void SquadEntity::onGoalReached()
{
}

void SquadEntity::onWaitingForCommand()
{
}

void SquadEntity::setCommand(SquadCommand command)
{
    mCommand = command;
    if (mCommand == SquadCommand::StandGround)
    {
        setGoal(position());
        setNextWaypoint(0);
        mPath.clear();
    }
}

SquadLeaderEntity::SquadLeaderEntity(World& world, const Settings& settings)
    : SquadEntity(world, settings)
{
}

SquadLeaderEntity::~SquadLeaderEntity()
{
    clearSquadMembers();
}

void SquadLeaderEntity::setSelectedPointOfInterest(PointOfInterest* poi)
{
    mSelectedPointOfInterest = poi;
    mSelectedPointOfInterestId = poi ? poi->id() : 0;
}

PointOfInterest* SquadLeaderEntity::selectedPointOfInterest() const
{
    if (!mSelectedPointOfInterest || !mPointsOfInterest)
        return mSelectedPointOfInterest;
    return mPointsOfInterest->find(mSelectedPointOfInterestId);
}

void SquadLeaderEntity::setSelectedWaypoint(Waypoint* wp)
{
    mSelectedWaypoint = wp;
    mSelectedWaypointId = wp ? wp->id() : 0;
}

Waypoint* SquadLeaderEntity::selectedWaypoint() const
{
    if (!mSelectedWaypoint || !mWaypointNetwork)
        return mSelectedWaypoint;
    return mWaypointNetwork->findWaypoint(mSelectedWaypointId);
}

void SquadLeaderEntity::addSquadMember(SquadEntity* member)
{
    if (!member)
        return;
    if (kinetix::Find(mSquadMembers.begin(), mSquadMembers.end(), member) != mSquadMembers.end())
        return;
    if (member->squadLeader() && member->squadLeader() != this)
        member->squadLeader()->removeSquadMember(member);
    mSquadMembers.push_back(member);
    member->setSquadLeader(this);
}

void SquadLeaderEntity::clearSquadMembers()
{
    for (SquadEntity* member : mSquadMembers)
    {
        if (member && member->squadLeader() == this)
            member->setSquadLeader(nullptr);
    }
    mSquadMembers.clear();
}

void SquadLeaderEntity::setCommand(SquadCommand command)
{
    mLastCommand = this->command(); // the command we are leaving
    SquadEntity::setCommand(command);

    if (command == SquadCommand::AttackTarget)
        sendSquadToTarget();
    else
    {
        for (SquadEntity* member : mSquadMembers)
            member->setCommand(command);
    }
}

void SquadLeaderEntity::sendSquadToTarget()
{
    if (mSquadMembers.empty())
        return;
    PointOfInterest* poi = selectedPointOfInterest();
    if (!poi)
        return;

    Path emptyPath;
    for (SquadEntity* member : mSquadMembers)
    {
        member->setNextWaypoint(0);
        member->setPath(emptyPath);
        member->setGoal(poi->position());
    }
}

void SquadLeaderEntity::sendSquadToRandomPOI()
{
    if (!mPointsOfInterest || mSquadMembers.empty())
        return;

    PointOfInterest* closest = mPointsOfInterest->findNearest(mSquadMembers[0]->goal());
    PointOfInterest* poi = mPointsOfInterest->selectRandom(closest ? closest->id() : 0);
    if (!poi)
        return;
    setSelectedPointOfInterest(poi);

    Path emptyPath;
    for (SquadEntity* member : mSquadMembers)
    {
        member->setNextWaypoint(0);
        member->setPath(emptyPath);
        member->setGoal(poi->position());
    }
}

void SquadLeaderEntity::sendSquadToRandomWaypoint()
{
    if (mSquadMembers.empty() || !mWaypointNetwork)
        return;

    const WaypointID wpID = selectRandomWaypoint(*mWaypointNetwork);
    Waypoint* wp = mWaypointNetwork->findWaypoint(wpID);
    if (!wp)
        return;
    setSelectedWaypoint(wp);

    Path emptyPath;
    for (SquadEntity* member : mSquadMembers)
    {
        member->setNextWaypoint(0);
        member->setPath(emptyPath);
        member->setGoal(wp->position());
    }
}

void SquadLeaderEntity::commandSquadToRallyOnLeader()
{
    if (mSquadMembers.empty())
        return;

    Path emptyPath;
    for (SquadEntity* member : mSquadMembers)
    {
        member->setNextWaypoint(0);
        member->setPath(emptyPath);
        member->setGoal(position());
    }
}

void SquadLeaderEntity::removeSquadMember(SquadEntity* member)
{
    auto it = kinetix::Find(mSquadMembers.begin(), mSquadMembers.end(), member);
    if (it != mSquadMembers.end())
        mSquadMembers.erase(it);
    if (member && member->squadLeader() == this)
        member->setSquadLeader(nullptr);
}

}
