#pragma once

#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/scene/Node3D.h"

namespace prisma3d
{

class PlanarReflector3D : public Node3D
{
public:
    PRISMA3D_NODE_CLASS(PlanarReflector3D, Node3D);
    static void bindProperties();

    PlanarReflector3D();
    ~PlanarReflector3D() override;

    ReflectorId getReflector() const;

    void setHalfSize(const Math::Vec2& halfSize);
    Math::Vec2 getHalfSize() const;
    void setResolution(std::uint32_t resolution);
    std::uint32_t getResolution() const;
    void setMaxDistance(float distance);
    float getMaxDistance() const;
    TextureId getTexture() const;

protected:
    void notificationHandler(int what) override;

private:
    void updateVisibility();

    ReflectorId mReflector;
    Math::Vec2 mHalfSize;
    std::uint32_t mResolution;
    float mMaxDistance;
    bool mVisibleInWorld;
};

} // namespace prisma3d
