// All offset arithmetic is done on the XZ plane; a zero-length guard protects the final
// orientation build. The per-formation helpers static_cast the entity: iterate() has already
// checked it is a SquadEntity.

#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>
#include <kinetix/core/Log.h>

#include "FormationBehavior.h"

#include "Entity.h"
#include "Group.h"
#include "SquadEntity.h"
#include "World.h"

#include <math.h>

namespace kinetix::AI
{

FormationBehavior::FormationBehavior(float goalRadius, float formationRadius)
    : mGoalRadius(goalRadius), mFormationRadius(formationRadius)
{
}

void FormationBehavior::iterate(float timeDelta, Entity& entity)
{
    (void)timeDelta;

    SquadEntity* squadmatePtr = SquadEntity::from(entity);
    if (!squadmatePtr)
        return;
    SquadEntity& squadmate = *squadmatePtr;

    if (squadmate.squadId() == 0)
        return;

    // The leader comes from this entity's own squad link: scanning for squadId 0 finds the last
    // such entity anywhere, the wrong leader once a second squad exists.
    mSquadLeader = squadmate.squadLeader();
    mPointMan = nullptr;
    if (!mSquadLeader)
        return;

    for (SquadEntity* member : mSquadLeader->squadMembers())
    {
        if (member && member->squadId() == 1)
        {
            mPointMan = member;
            break;
        }
    }

    if (!mPointMan || !mSquadLeader)
        return;

    // Orientation basis vectors: right = +X, up = +Y, forward (look) = +Z.
    Math::mat3 pointManBasis = Math::mat3_cast(mPointMan->orientation());
    mPointManLook = pointManBasis[2];
    mPointManRight = pointManBasis[0];
    Math::mat3 leaderBasis = Math::mat3_cast(mSquadLeader->orientation());
    mLeaderLook = leaderBasis[2];
    mLeaderRight = leaderBasis[0];

    Math::vec3 goal(0.0f);
    Math::vec3 dir(0.0f);
    switch (static_cast<SquadFormation>(mSquadLeader->squadFormation()))
    {
    case SquadFormation::Pentagon:
        pentagon(entity, goal, dir);
        break;
    case SquadFormation::Diamond:
        diamond(entity, goal, dir);
        break;
    case SquadFormation::Abreast:
        abreast(entity, goal, dir);
        break;
    case SquadFormation::SingleFile:
        singleFile(entity, goal, dir);
        break;
    case SquadFormation::Wedge:
        wedge(entity, goal, dir);
        break;
    case SquadFormation::V:
        vFormation(entity, goal, dir);
        break;
    case SquadFormation::Circle:
        circle(entity, goal, dir);
        break;
    default:
        goal = squadmate.goal();
        dir = goal - squadmate.position();
        break;
    }

    squadmate.setGoalRadius(mFormationRadius);
    mPointMan->setGoalRadius(mGoalRadius);

    dir.y = 0.0f;
    Math::vec3 up(0.0f, 1.0f, 0.0f);
    Math::vec3 lk = dir;
    float lkLen = Math::length(lk);
    // Near a formation slot the direction is floating-point noise; rebuilding the quaternion from
    // it makes a stopped agent flip between two headings every frame.
    if (lkLen > 0.1f)
    {
        lk /= lkLen;
        Math::vec3 rt = Math::normalize(Math::cross(up, lk));
        Math::mat3 basis(rt, up, lk); // columns: right, up, forward
        squadmate.setGoal(goal);
        squadmate.setOrientation(Math::quat_cast(basis));
    }
    else
    {
        // No direction yet - keep the goal, leave the current orientation.
        squadmate.setGoal(goal);
    }
}

void FormationBehavior::diamond(Entity& entity, Math::vec3& goal, Math::vec3& dir) const
{
    const SquadEntity& squadmate = static_cast<const SquadEntity&>(entity);
    switch (squadmate.squadId())
    {
    case 1: // point man
        goal = squadmate.goal();
        dir = goal - squadmate.position();
        break;
    case 2: // right flank
        goal = mPointMan->position() - (mPointManLook * 20.0f) + (mPointManRight * 30.0f);
        dir = mPointManRight;
        break;
    case 3: // left flank
        goal = mPointMan->position() - (mPointManLook * 20.0f) - (mPointManRight * 30.0f);
        dir = -mPointManRight;
        break;
    case 4: // rear guard
        goal = mPointMan->position() - (mPointManLook * 90.0f);
        dir = -mPointManLook;
        break;
    default: // anyone else just heads to the goal
        goal = squadmate.goal();
        dir = goal - squadmate.position();
        break;
    }
}

void FormationBehavior::abreast(Entity& entity, Math::vec3& goal, Math::vec3& dir) const
{
    const SquadEntity& squadmate = static_cast<const SquadEntity&>(entity);
    switch (squadmate.squadId())
    {
    case 1: // point man
        goal = squadmate.goal();
        break;
    case 2: // right flank
        goal = mPointMan->position() + (mPointManRight * 40.0f);
        break;
    case 3: // left flank
        goal = mPointMan->position() + (mPointManRight * 85.0f);
        break;
    case 4: // rear guard
        goal = mPointMan->position() + (mPointManRight * 120.0f);
        break;
    default:
        goal = squadmate.goal();
        break;
    }

    dir = goal - squadmate.position();
}

void FormationBehavior::singleFile(Entity& entity, Math::vec3& goal, Math::vec3& dir) const
{
    const SquadEntity& squadmate = static_cast<const SquadEntity&>(entity);
    switch (squadmate.squadId())
    {
    case 1: // point man
        goal = squadmate.goal();
        dir = goal - squadmate.position();
        break;
    case 2:
        goal = mPointMan->position() - (mPointManLook * 100.0f);
        dir = mPointManLook;
        break;
    case 3:
        goal = mPointMan->position() - (mPointManLook * 190.0f);
        dir = mPointManLook;
        break;
    case 4:
        goal = mPointMan->position() - (mPointManLook * 240.0f);
        dir = mPointManLook;
        break;
    default:
        goal = squadmate.goal();
        dir = goal - squadmate.position();
        break;
    }
}

void FormationBehavior::pentagon(Entity& entity, Math::vec3& goal, Math::vec3& dir) const
{
    const SquadEntity& squadmate = static_cast<const SquadEntity&>(entity);

    // Rotate the leader's look by 45 degrees about the world up axis. The opposite diagonal is
    // rotated in the leader's local frame; mirroring the global X component only works while the
    // leader faces +Z.
    Math::vec3 v1 = Math::angleAxis(Math::radians(45.0f), Math::vec3(0.0f, 1.0f, 0.0f)) * mLeaderLook;
    Math::vec3 v2 = Math::angleAxis(Math::radians(-45.0f), Math::vec3(0.0f, 1.0f, 0.0f)) * mLeaderLook;

    switch (squadmate.squadId())
    {
    case 2: // right flank
        goal = mSquadLeader->position() - (mLeaderLook * 30.0f) + (mLeaderRight * 50.0f);
        dir = v1;
        break;
    case 3: // left flank
        goal = mSquadLeader->position() - (mLeaderLook * 30.0f) - (mLeaderRight * 50.0f);
        dir = v2;
        break;
    case 1: // point man (rear guard 2)
        goal = mSquadLeader->position() - (mLeaderLook * 90.0f) - (mLeaderRight * 25.0f);
        dir = -v1;
        break;
    case 4: // rear guard
        goal = mSquadLeader->position() - (mLeaderLook * 90.0f) + (mLeaderRight * 25.0f);
        dir = -v2;
        break;
    default:
        goal = squadmate.goal();
        dir = goal - squadmate.position();
        break;
    }
}

void FormationBehavior::wedge(Entity& entity, Math::vec3& goal, Math::vec3& dir) const
{
    const SquadEntity& member = static_cast<const SquadEntity&>(entity);
    const Math::vec3& leader = mSquadLeader->position();
    switch (member.squadId())
    {
    case 1: goal = leader + mLeaderLook * 25.0f; dir = mLeaderLook; break;
    case 2: goal = leader - mLeaderLook * 20.0f + mLeaderRight * 25.0f; dir = mLeaderLook; break;
    case 3: goal = leader - mLeaderLook * 20.0f - mLeaderRight * 25.0f; dir = mLeaderLook; break;
    case 4: goal = leader - mLeaderLook * 55.0f; dir = mLeaderLook; break;
    default: goal = member.goal(); dir = goal - member.position(); break;
    }
}

void FormationBehavior::vFormation(Entity& entity, Math::vec3& goal, Math::vec3& dir) const
{
    const SquadEntity& member = static_cast<const SquadEntity&>(entity);
    const Math::vec3& leader = mSquadLeader->position();
    switch (member.squadId())
    {
    case 1: goal = leader + mLeaderLook * 25.0f; break;
    case 2: goal = leader - mLeaderLook * 15.0f + mLeaderRight * 30.0f; break;
    case 3: goal = leader - mLeaderLook * 15.0f - mLeaderRight * 30.0f; break;
    case 4: goal = leader - mLeaderLook * 55.0f + mLeaderRight * 55.0f; break;
    default: goal = member.goal(); break;
    }
    dir = goal - member.position();
}

void FormationBehavior::circle(Entity& entity, Math::vec3& goal, Math::vec3& dir) const
{
    const SquadEntity& member = static_cast<const SquadEntity&>(entity);
    const float angle = Math::radians(90.0f * static_cast<float>(member.squadId() - 1));
    const Math::vec3 offset = mLeaderRight * (cosf(angle) * 45.0f) +
                             mLeaderLook * (sinf(angle) * 45.0f);
    goal = mSquadLeader->position() + offset;
    dir = mSquadLeader->position() - member.position();
}

}
