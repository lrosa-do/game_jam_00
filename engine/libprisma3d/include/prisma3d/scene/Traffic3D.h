#pragma once

#include "prisma3d/scene/Node3D.h"

#include <ct/string.hpp>
#include <ct/vector.hpp>

#include <cstdint>

namespace prisma3d
{

class CarDamage3D;
class GroundVehicle3D;
class Path3D;
class VehicleDriver3D;

class Traffic3D : public Node3D
{
public:
    PRISMA3D_NODE_CLASS(Traffic3D, Node3D);
    static void bindProperties();

    Traffic3D();
    ~Traffic3D() override;

    void setPathNode(const ct::String& path);
    const ct::String& getPathNode() const;
    void setTarget(const ct::String& path);
    const ct::String& getTarget() const;
    void setAheadDistance(float distance);
    float getAheadDistance() const;
    void setBehindDistance(float distance);
    float getBehindDistance() const;
    void setRailLayer(std::uint32_t layer);
    std::uint32_t getRailLayer() const;
    void setRailMask(std::uint32_t mask);
    std::uint32_t getRailMask() const;

    int getActiveCount() const;

protected:
    void notificationHandler(int what) override;

private:
    struct Car
    {
        GroundVehicle3D* vehicle = nullptr;
        VehicleDriver3D* driver = nullptr;
        CarDamage3D* damage = nullptr;
        float offset = 0.0f;
        float lane = 0.0f;
        float speed = 0.0f;
        std::uint32_t layer = 1;
        std::uint32_t mask = 1;
        bool active = true;
    };

    bool gather();
    void update(float step);
    void setActive(Car& car, bool active, const Path3D& path);
    void putOnLane(Car& car, const Path3D& path) const;
    Math::Vec3 laneDirection(const Path3D& path, float offset) const;

    ct::String mPathNode;
    ct::String mTarget;
    ct::Vector<Car> mCars;
    float mAheadDistance;
    float mBehindDistance;
    std::uint32_t mRailLayer;
    std::uint32_t mRailMask;
    unsigned mSpread;
    bool mGathered;
};

} // namespace prisma3d
