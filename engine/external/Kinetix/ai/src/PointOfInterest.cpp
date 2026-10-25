#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>
#include <kinetix/core/Log.h>

#include "PointOfInterest.h"

#include <float.h>
#include <stdlib.h>

namespace kinetix::AI
{

namespace
{
PointOfInterestID nextPointOfInterestId()
{
    static PointOfInterestID counter = 1; // 0 is reserved as the null/invalid id
    return counter++;
}
}

// PointOfInterest id is assigned lazily on first use to keep the default
// constructor trivial; add() is the only place that needs it.
PointOfInterestID PointOfInterest::id() const
{
    if (mId == 0)
        mId = nextPointOfInterestId();
    return mId;
}

bool PointsOfInterest::add(PointOfInterest* poi)
{
    PointOfInterestID id = poi->id();
    if (mPois.find(id) == nullptr)
    {
        mPois[id] = poi;
        return true;
    }
    return false;
}

bool PointsOfInterest::remove(PointOfInterestID poiID)
{
    PointOfInterest** slot = mPois.find(poiID);
    if (!slot)
        return false;

    delete *slot;
    mPois.erase(poiID);
    return true;
}

void PointsOfInterest::clear()
{
    for (auto& entry : mPois)
        delete entry.value;
    mPois.clear();
}

PointOfInterest* PointsOfInterest::find(PointOfInterestID poiID) const
{
    PointOfInterest* const* slot = mPois.find(poiID);
    return slot ? *slot : nullptr;
}

PointOfInterest* PointsOfInterest::selectRandom(PointOfInterestID current) const
{
    if (mPois.empty())
        return nullptr;
    if (mPois.size() == 1)
        return mPois.begin()->value;

    ct::Vector<PointOfInterestID> ids;
    ids.reserve(mPois.size());
    for (const auto& entry : mPois)
        ids.push_back(entry.key);

    PointOfInterestID newId = current;
    while (newId == current)
        newId = ids[rand() % static_cast<int>(ids.size())];

    return find(newId);
}

PointOfInterest* PointsOfInterest::findNearest(const Math::vec3& position) const
{
    PointOfInterest* closest = nullptr;
    float closestDist = FLT_MAX;
    for (const auto& entry : mPois)
    {
        PointOfInterest* poi = entry.value;
        float dist = Math::length(poi->position() - position);
        if (dist < closestDist)
        {
            closestDist = dist;
            closest = poi;
        }
    }
    return closest;
}

}
