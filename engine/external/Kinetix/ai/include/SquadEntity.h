#ifndef KINETIX_AI_SQUADENTITY_H
#define KINETIX_AI_SQUADENTITY_H

// Does NOT own its state machine; the caller must delete it, and not let it outlive the entity
// (its callbacks capture the entity).

#include "Entity.h"
#include "FormationBehavior.h"
#include "PointOfInterest.h"
#include "Waypoint.h"
#include "WaypointNetwork.h"

#include <ct/vector.hpp>

namespace kinetix::AI
{

class PointOfInterest;
class PointsOfInterest;
class SquadLeaderEntity;
class StateMachine;
class Waypoint;
class WaypointNetwork;

enum class SquadCommand
{
    PatrolWaypointNetwork,
    PatrolPointsOfInterest,
    RallyToLeaderPosition,
    StandGround,
    AttackTarget,
    FlankTarget,
    SuppressTarget,
    Regroup,
    Retreat,
    DefendPosition,
    MoveToPoint,
    SearchArea,
    EscortTarget,
    Count
};

WaypointID selectRandomWaypoint(const WaypointNetwork& network);

class SquadEntity : public Entity
{
public:
    // The entity as a SquadEntity, or null when it is another kind.
    static SquadEntity* from(Entity& entity)
    {
        return entity.kind() == EntityKind::Squad ? static_cast<SquadEntity*>(&entity) : nullptr;
    }

    SquadEntity(World& world, const Settings& settings);
    // Leaves its leader's squad, so the leader never holds a dead member.
    ~SquadEntity() override;

    void iterate(float timeDelta) override;

    void setPath(const Path& path)
    {
        mPath = path;
    }
    Path& path()
    {
        return mPath;
    }
    const Path& path() const
    {
        return mPath;
    }

    void setGoal(const Math::vec3& goal)
    {
        mGoalPosition = goal;
    }
    const Math::vec3& goal() const
    {
        return mGoalPosition;
    }

    void setNextWaypoint(WaypointID wp)
    {
        mNextWaypoint = wp;
    }
    WaypointID nextWaypoint() const
    {
        return mNextWaypoint;
    }
    void setCurrentWaypoint(WaypointID wp)
    {
        mCurrentWaypoint = wp;
    }
    WaypointID currentWaypoint() const
    {
        return mCurrentWaypoint;
    }

    void setWaypointNetwork(WaypointNetwork* network)
    {
        mWaypointNetwork = network;
    }
    WaypointNetwork* waypointNetwork() const
    {
        return mWaypointNetwork;
    }

    void setStateMachine(StateMachine* machine)
    {
        mStateMachine = machine;
    }
    StateMachine* stateMachine() const
    {
        return mStateMachine;
    }

    bool hasValidWaypoint() const
    {
        return mNextWaypoint != 0;
    }
    bool hasValidPath() const
    {
        return !mPath.empty();
    }

    bool waypointReached();
    bool goalReached();
    void onWaypointReached();
    void onGoalReached();
    void onWaitingForCommand();

    void resetTimeSinceWaypointReached()
    {
        mTimeSinceNextWaypointReached = 0.0f;
    }
    void incrementTimeSinceWaypointReached(float deltaTime)
    {
        mTimeSinceNextWaypointReached += deltaTime;
    }
    float timeSinceWaypointReached() const
    {
        return mTimeSinceNextWaypointReached;
    }
    void resetTimeSinceGoalReached()
    {
        mTimeSinceGoalReached = 0.0f;
    }
    void incrementTimeSinceGoalReached(float deltaTime)
    {
        mTimeSinceGoalReached += deltaTime;
    }
    float timeSinceGoalReached() const
    {
        return mTimeSinceGoalReached;
    }
    void resetTimeSinceLOSTest()
    {
        mTimeSinceLOSTested = 0.0f;
    }
    void incrementTimeSinceLOSTest(float deltaTime)
    {
        mTimeSinceLOSTested += deltaTime;
    }
    float timeSinceLOSTest() const
    {
        return mTimeSinceLOSTested;
    }

    void setLOSStatus(bool status)
    {
        mLOSStatus = status;
    }
    bool losStatus() const
    {
        return mLOSStatus;
    }

    void setSquadId(int id)
    {
        mSquadId = id;
    }
    int squadId() const
    {
        return mSquadId;
    }

    // Set by SquadLeaderEntity::addSquadMember()/removeSquadMember(). FormationBehavior reads this
    // because scanning for squadId 0 finds the last id-0 entity anywhere, not this entity's leader.
    void setSquadLeader(SquadLeaderEntity* leader)
    {
        mSquadLeader = leader;
    }
    SquadLeaderEntity* squadLeader() const
    {
        return mSquadLeader;
    }

    void setGoalRadius(float radius)
    {
        mGoalRadius = radius;
    }
    float goalRadius() const
    {
        return mGoalRadius;
    }

    void setSquadFormation(int formation)
    {
        mSquadFormation = formation;
    }
    int squadFormation() const
    {
        return mSquadFormation;
    }

    virtual void setCommand(SquadCommand command);
    SquadCommand command() const
    {
        return mCommand;
    }

protected:
    WaypointNetwork* mWaypointNetwork = nullptr;
    WaypointID mNextWaypoint = 0;
    WaypointID mCurrentWaypoint = 0;
    Math::vec3 mGoalPosition = Math::vec3(0.0f);
    Path mPath;
    StateMachine* mStateMachine = nullptr;
    SquadCommand mCommand = SquadCommand::PatrolPointsOfInterest;
    bool mLOSStatus = false;
    float mTimeSinceNextWaypointReached = 0.0f;
    float mTimeSinceGoalReached = 0.0f;
    float mTimeSinceLOSTested = 100.0f; // first LOS test fires immediately
    float mGoalRadius = 25.0f;
    int mSquadId = -1;
    int mSquadFormation = static_cast<int>(SquadFormation::Abreast);
    SquadLeaderEntity* mSquadLeader = nullptr;
};

class SquadLeaderEntity final : public SquadEntity
{
public:
    SquadLeaderEntity(World& world, const Settings& settings);
    ~SquadLeaderEntity() override;

    void setCommand(SquadCommand command) override;

    bool hasCommandChanged() const
    {
        return !mCommandAcknowledged || command() != mLastCommand;
    }

    void acknowledgeCommand()
    {
        mLastCommand = command();
        mCommandAcknowledged = true;
    }

    void sendSquadToRandomPOI();
    void sendSquadToRandomWaypoint();
    void commandSquadToRallyOnLeader();
    void sendSquadToTarget();

    // The selection is remembered by id and looked up again in the owning registry on every read,
    // so a POI or waypoint removed from it reads back as nullptr instead of dangling. Without a
    // registry set, the pointer is returned as given and its lifetime is the caller's.
    void setSelectedPointOfInterest(PointOfInterest* poi);
    PointOfInterest* selectedPointOfInterest() const;
    void setSelectedWaypoint(Waypoint* wp);
    Waypoint* selectedWaypoint() const;
    void setPointsOfInterest(PointsOfInterest* pois)
    {
        mPointsOfInterest = pois;
    }
    PointsOfInterest* pointsOfInterest() const
    {
        return mPointsOfInterest;
    }

    // Squad members are NOT owned by the leader. The member<->leader link is
    // kept on both sides: a member joining here leaves any previous leader,
    // and removing or clearing members resets their squadLeader().
    void addSquadMember(SquadEntity* member);
    void removeSquadMember(SquadEntity* member);
    void clearSquadMembers();
    ct::Vector<SquadEntity*>& squadMembers()
    {
        return mSquadMembers;
    }
    const ct::Vector<SquadEntity*>& squadMembers() const
    {
        return mSquadMembers;
    }

private:
    ct::Vector<SquadEntity*> mSquadMembers;
    PointsOfInterest* mPointsOfInterest = nullptr;
    PointOfInterest* mSelectedPointOfInterest = nullptr;
    PointOfInterestID mSelectedPointOfInterestId = 0;
    Waypoint* mSelectedWaypoint = nullptr;
    WaypointID mSelectedWaypointId = 0;
    SquadCommand mLastCommand = SquadCommand::PatrolPointsOfInterest;
    bool mCommandAcknowledged = false;
};

class SquadMemberEntity final : public SquadEntity
{
public:
    using SquadEntity::SquadEntity;
};

}

#endif
