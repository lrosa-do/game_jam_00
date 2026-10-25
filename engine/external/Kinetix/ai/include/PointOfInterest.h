#ifndef KINETIX_AI_POINTOFINTEREST_H
#define KINETIX_AI_POINTOFINTEREST_H

#include <kinetix/core/Types.h>

#include <kinetix/core/Math.h>
#include <ct/hashmap.hpp>
#include <ct/vector.hpp>

namespace kinetix::AI
{

using PointOfInterestID = kinetix::u32;

class PointOfInterest
{
public:
    PointOfInterest() = default;
    PointOfInterest(const Math::vec3& position, float radius) : mPosition(position), mRadius(radius)
    {
    }

    // Assigned lazily on first call so default-constructed POIs never waste
    // an id until they are actually registered.
    PointOfInterestID id() const;

    const Math::vec3& position() const
    {
        return mPosition;
    }
    void setPosition(const Math::vec3& position)
    {
        mPosition = position;
    }
    float radius() const
    {
        return mRadius;
    }
    void setRadius(float radius)
    {
        mRadius = radius;
    }

private:
    mutable PointOfInterestID mId = 0;
    Math::vec3 mPosition = Math::vec3(0.0f);
    float mRadius = 1.0f;
};

// Registry of points of interest. OWNS the POIs added via add() and deletes
// them on remove/clear/destruction.
class PointsOfInterest
{
public:
    using Map = ct::HashMap<PointOfInterestID, PointOfInterest*>;

    PointsOfInterest() = default;
    ~PointsOfInterest()
    {
        clear();
    }

    bool add(PointOfInterest* poi);
    bool remove(PointOfInterestID poiID);
    void clear();
    PointOfInterest* find(PointOfInterestID poiID) const;

    PointOfInterest* selectRandom(PointOfInterestID current) const;

    PointOfInterest* findNearest(const Math::vec3& position) const;

    const Map& map() const
    {
        return mPois;
    }

private:
    Map mPois;
};

}

#endif
