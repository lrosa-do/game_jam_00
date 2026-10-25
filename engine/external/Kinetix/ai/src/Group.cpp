#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>
#include <kinetix/core/Log.h>

#include "Group.h"

#include "Entity.h"

#include <kinetix/core/Algorithm.h>

namespace kinetix::AI
{

Group::~Group()
{
    // Unlinked before the delete, so the entity's destructor does not reach
    // back into a list this loop is walking.
    ct::Vector<Entity*> entities = kinetix::Move(mEntities);
    mEntities.clear();
    for (Entity* entity : entities)
    {
        entity->setCurrentGroup(nullptr);
        delete entity;
    }
}

void Group::add(Entity& entity)
{
    // Rejects an entity already owned by a group (itself or another): adding it twice would delete
    // it twice, and a second group would orphan it in the first, which would still delete it.
    if (entity.currentGroup())
    {
        Log::warning("Group: add() called with an entity already owned by a group; ignored");
        return;
    }
    mEntities.push_back(&entity);
    entity.setCurrentGroup(this);
}

void Group::remove(Entity& entity)
{
    auto it = kinetix::Find(mEntities.begin(), mEntities.end(), &entity);
    if (it == mEntities.end())
        return; // not this group's entity - do not touch its currentGroup()
    mEntities.erase(it);
    entity.setCurrentGroup(nullptr);
}

void Group::iterate(float timeDelta)
{
    for (Entity* entity : mEntities)
        entity->iterate(timeDelta);
}

}
