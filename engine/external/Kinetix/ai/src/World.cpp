#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>
#include <kinetix/core/Log.h>

#include "World.h"

#include "Group.h"

#include <kinetix/core/Algorithm.h>

namespace kinetix::AI
{

World::~World()
{
    for (Group* group : mGroups)
        delete group;
    mGroups.clear();
}

void World::add(Group& group)
{
    // Rejects a group already in this world: it would be deleted twice.
    if (kinetix::Find(mGroups.begin(), mGroups.end(), &group) != mGroups.end())
    {
        Log::warning("World: add() called with a group already owned by this world; ignored");
        return;
    }
    mGroups.push_back(&group);
}

void World::remove(Group& group)
{
    auto it = kinetix::Find(mGroups.begin(), mGroups.end(), &group);
    if (it != mGroups.end())
        mGroups.erase(it);
}

void World::iterate(float timeDelta)
{
    // One pass to file everyone before any behavior asks who is nearby; positions change during the
    // step below, so behaviors see where the others were at the top of the frame.
    mGridScratch.clear();
    for (Group* group : mGroups)
        for (Entity* entity : group->entities())
            mGridScratch.push_back(entity);
    mProximityGrid.rebuild(mGridScratch);

    for (Group* group : mGroups)
        group->iterate(timeDelta);
}

}
