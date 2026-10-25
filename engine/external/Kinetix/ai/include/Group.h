#ifndef KINETIX_AI_GROUP_H
#define KINETIX_AI_GROUP_H

// Owns the entities added via add() and deletes them on destruction.

#include <ct/vector.hpp>

namespace kinetix::AI
{

class Entity;
class World;

class Group
{
public:
    explicit Group(World& world) : mWorld(world)
    {
    }
    virtual ~Group();

    // Takes ownership: entity must be heap-allocated. add() rejects one that already has a group;
    // a stack object cannot be detected.
    void add(Entity& entity);
    void remove(Entity& entity);

    ct::Vector<Entity*>& entities()
    {
        return mEntities;
    }
    const ct::Vector<Entity*>& entities() const
    {
        return mEntities;
    }

    virtual void iterate(float timeDelta);

protected:
    ct::Vector<Entity*> mEntities;
    World& mWorld;
};

}

#endif
