#pragma once

#include "prisma3d/scene/Node3D.h"

#include <ct/string.hpp>
#include <ct/vector.hpp>

namespace prisma3d
{

class ManualMesh3D;

// Walkable surface of a level for agents that walk: built from the static collision under the
// source node (concave meshes and boxes of static bodies), with the floor under furniture and
// within the agent radius of walls and drops left out. Paths follow the surface; the region joins
// the "navigation" group so scripts find it without a path.
class NavigationRegion3D : public Node3D
{
public:
    PRISMA3D_NODE_CLASS(NavigationRegion3D, Node3D);
    static void bindProperties();

    NavigationRegion3D();
    ~NavigationRegion3D() override;

    void setSourcePath(const ct::String& path);
    ct::String getSourcePath() const;
    void setSeedPath(const ct::String& path);
    ct::String getSeedPath() const;
    void setCellSize(float size);
    float getCellSize() const;
    void setAgentHeight(float height);
    float getAgentHeight() const;
    void setAgentRadius(float radius);
    float getAgentRadius() const;
    void setAgentMaxClimb(float climb);
    float getAgentMaxClimb() const;
    void setAgentMaxSlope(float degrees);
    float getAgentMaxSlope() const;
    void setBakeOnReady(bool bake);
    bool getBakeOnReady() const;
    void setShowDebug(bool show);
    bool getShowDebug() const;

    // Collects the collision and builds the surface now; false when there is no floor.
    bool bake();
    void clear();
    bool isBaked() const;
    int getTriangleCount() const;
    double getBakeMilliseconds() const;

    // Every query answers in global space and fails (empty path, false) before a bake.
    bool findPath(const Math::Vec3& from, const Math::Vec3& to, ct::Vector<Math::Vec3>& path,
            bool* partial = nullptr) const;
    bool nearestPoint(const Math::Vec3& point, Math::Vec3& out) const;
    bool moveAlongSurface(const Math::Vec3& from, const Math::Vec3& to, Math::Vec3& out) const;
    bool raycast(const Math::Vec3& from, const Math::Vec3& to, float& fraction) const;
    bool randomPoint(Math::Vec3& out);

protected:
    void notificationHandler(int what) override;

private:
    struct State;

    void collect(Node* node, ct::Vector<float>& vertices, ct::Vector<int>& indices) const;
    void updateDebug();

    State* mState;
    ManualMesh3D* mDebug;
    ct::String mSourcePath;
    ct::String mSeedPath;
    float mCellSize;
    float mAgentHeight;
    float mAgentRadius;
    float mAgentMaxClimb;
    float mAgentMaxSlope;
    bool mBakeOnReady;
    bool mShowDebug;
    float mWaited;
    unsigned mRandomSeed;
};

} // namespace prisma3d
