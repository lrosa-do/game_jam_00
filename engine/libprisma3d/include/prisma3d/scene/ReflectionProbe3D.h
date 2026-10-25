#pragma once

#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/scene/Node3D.h"

namespace prisma3d
{

class ReflectionProbe3D : public Node3D
{
public:
    PRISMA3D_NODE_CLASS(ReflectionProbe3D, Node3D);
    static void bindProperties();

    ReflectionProbe3D();
    ~ReflectionProbe3D() override;

    ReflectionProbeId getProbe() const;

    void setUpdateMode(ReflectionProbeUpdateMode mode);
    ReflectionProbeUpdateMode getUpdateMode() const;
    void setIntensity(float intensity);
    float getIntensity() const;
    void setBlendDistance(float distance);
    float getBlendDistance() const;
    void setAmbientMode(ReflectionProbeAmbientMode mode);
    ReflectionProbeAmbientMode getAmbientMode() const;
    void setAmbientColor(const Math::Vec3& color);
    const Math::Vec3& getAmbientColor() const;
    void setAmbientEnergy(float energy);
    float getAmbientEnergy() const;
    void setMaxDistance(float distance);
    float getMaxDistance() const;
    void setSize(const Math::Vec3& size);
    const Math::Vec3& getSize() const;
    void setOriginOffset(const Math::Vec3& offset);
    const Math::Vec3& getOriginOffset() const;
    void setInterior(bool interior);
    bool isInterior() const;
    void setBoxProjection(bool enabled);
    bool isBoxProjection() const;
    void setShadows(bool enabled);
    bool isShadows() const;
    void setCullMask(std::uint32_t mask);
    std::uint32_t getCullMask() const;
    void setReflectionMask(std::uint32_t mask);
    std::uint32_t getReflectionMask() const;
    void setMeshLodThreshold(float pixels);
    float getMeshLodThreshold() const;
    const ReflectionProbeParams& getParams() const;
    Math::Box getAabb() const;

protected:
    void notificationHandler(int what) override;
    void validateProperty(PropertyInfo& info) const override;

private:
    void push();
    void updateVisibility();

    ReflectionProbeId mProbe;
    ReflectionProbeParams mParams;
    bool mVisibleInWorld;
};

} // namespace prisma3d
