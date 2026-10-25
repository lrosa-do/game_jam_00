#ifndef KINETIX_AI_WORLD_H
#define KINETIX_AI_WORLD_H

#include "ProximityGrid.h"

#include <ct/vector.hpp>

namespace kinetix::AI
{

class Group;
class Entity;

class World
{
public:
    World() = default;
    virtual ~World();

    // Takes ownership: group must be heap-allocated. add() rejects a group already in this world;
    // one added to two worlds, or a stack object, cannot be detected.
    void add(Group& group);
    void remove(Group& group);

    ct::Vector<Group*>& groups()
    {
        return mGroups;
    }
    const ct::Vector<Group*>& groups() const
    {
        return mGroups;
    }

    virtual void iterate(float timeDelta);

    // Rebuilt at the top of every iterate(), so behaviors asking "who is near me" get this frame's
    // answer. Behaviors read it; nothing else should write to it.
    const ProximityGrid& proximityGrid() const
    {
        return mProximityGrid;
    }

protected:
    ct::Vector<Group*> mGroups;
    ProximityGrid mProximityGrid;
    // Scratch for the rebuild, kept so a frame costs no allocation.
    ct::Vector<Entity*> mGridScratch;
};

}

#endif
