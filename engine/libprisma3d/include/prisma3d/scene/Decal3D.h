#pragma once

#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/scene/Node3D.h"

namespace prisma3d
{

class Decal3D : public Node3D
{
public:
    PRISMA3D_NODE_CLASS(Decal3D, Node3D);
    static void bindProperties();

    Decal3D();
    ~Decal3D() override;

    DecalId getDecal() const;

    void setSize(const Math::Vec3& size);
    const Math::Vec3& getSize() const;
    void setTexture(DecalTexture type, TextureId texture);
    TextureId getTexture(DecalTexture type) const;
    void setEmissionEnergy(float energy);
    float getEmissionEnergy() const;
    void setAlbedoMix(float mix);
    float getAlbedoMix() const;
    void setModulate(const Math::Vec4& modulate);
    const Math::Vec4& getModulate() const;
    void setUpperFade(float fade);
    float getUpperFade() const;
    void setLowerFade(float fade);
    float getLowerFade() const;
    void setNormalFade(float fade);
    float getNormalFade() const;
    void setDistanceFadeEnabled(bool enabled);
    bool isDistanceFadeEnabled() const;
    void setDistanceFadeBegin(float distance);
    float getDistanceFadeBegin() const;
    void setDistanceFadeLength(float distance);
    float getDistanceFadeLength() const;
    void setCullMask(std::uint32_t mask);
    std::uint32_t getCullMask() const;
    void setSortingOffset(float offset);
    float getSortingOffset() const;
    Math::Box getAabb() const;

protected:
    void notificationHandler(int what) override;
    void validateProperty(PropertyInfo& info) const override;

private:
    void updateVisibility();

    DecalId mDecal;
    DecalParams mParams;
    float mSortingOffset;
    bool mVisibleInWorld;
};

} // namespace prisma3d
