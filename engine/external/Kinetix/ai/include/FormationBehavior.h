#ifndef KINETIX_AI_FORMATIONBEHAVIOR_H
#define KINETIX_AI_FORMATIONBEHAVIOR_H

// Acts only on a SquadEntity with squadId() != 0 (the leader, id 0, is player controlled and skips
// the formation). Local frame: right = +X, up = +Y, forward (look) = +Z.

#include "Behavior.h"

#include <kinetix/core/Math.h>

namespace kinetix::AI
{

enum class SquadFormation
{
    Pentagon,
    Diamond,
    Abreast,
    SingleFile,
    Wedge,
    V,
    Circle,
    Count
};

class SquadEntity;
class SquadLeaderEntity;

class FormationBehavior final : public Behavior
{
public:
    FormationBehavior(float goalRadius, float formationRadius);

    void iterate(float timeDelta, Entity& entity) override;
    const char* name() const override
    {
        return "Formation Behavior";
    }

private:
    void singleFile(Entity& entity, Math::vec3& goal, Math::vec3& dir) const;
    void abreast(Entity& entity, Math::vec3& goal, Math::vec3& dir) const;
    void diamond(Entity& entity, Math::vec3& goal, Math::vec3& dir) const;
    void pentagon(Entity& entity, Math::vec3& goal, Math::vec3& dir) const;
    void wedge(Entity& entity, Math::vec3& goal, Math::vec3& dir) const;
    void vFormation(Entity& entity, Math::vec3& goal, Math::vec3& dir) const;
    void circle(Entity& entity, Math::vec3& goal, Math::vec3& dir) const;

    float mGoalRadius;
    float mFormationRadius;

    SquadLeaderEntity* mSquadLeader = nullptr; // non-owning
    SquadEntity* mPointMan = nullptr;          // non-owning

    Math::vec3 mPointManLook;
    Math::vec3 mPointManRight;
    Math::vec3 mLeaderLook;
    Math::vec3 mLeaderRight;
};

}

#endif
