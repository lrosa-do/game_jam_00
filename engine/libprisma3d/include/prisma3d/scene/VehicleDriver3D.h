#pragma once

#include "prisma3d/scene/Node.h"

#include "mathc.h"

#include <ct/string.hpp>
#include <ct/vector.hpp>

namespace prisma3d
{

class GroundVehicle3D;

class VehicleDriver3D : public Node
{
public:
    PRISMA3D_NODE_CLASS(VehicleDriver3D, Node);
    static void bindProperties();

    VehicleDriver3D();
    ~VehicleDriver3D() override;

    void setPathNode(const ct::String& path);
    const ct::String& getPathNode() const;
    void setCornering(float cornering);
    float getCornering() const;
    void setTopSpeed(float speed);
    float getTopSpeed() const;
    void setLaneOffset(float offset);
    float getLaneOffset() const;
    void setActive(bool active);
    bool isActive() const;

    float getProgress() const;
    bool wantsBoost() const;
    GroundVehicle3D* getVehicle() const;

protected:
    void notificationHandler(int what) override;

private:
    bool loadPath();
    float follow(const Math::Vec3& place, const Math::Vec3& ahead);
    void drive(GroundVehicle3D& vehicle, float step);
    void putOnRoad(GroundVehicle3D& vehicle, const Math::Vec3& at, const Math::Vec3& along) const;
    int nearest(const Math::Vec3& place) const;
    int wrap(int index) const;
    const Math::Vec3& point(int index) const;
    int steps(float metres) const;

    ct::String mPathNode;
    ct::Vector<Math::Vec3> mPath;
    float mSpacing;
    bool mLoop;
    bool mLoaded;
    int mIndex;
    int mDirection;
    float mProgress;
    float mCornering;
    float mTopSpeed;
    float mLaneOffset;
    float mStuck;
    float mBacking;
    bool mActive;
    bool mBoost;
};

} // namespace prisma3d
