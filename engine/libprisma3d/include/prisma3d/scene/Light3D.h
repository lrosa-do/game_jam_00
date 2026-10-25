#pragma once

#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/scene/Node3D.h"

namespace prisma3d
{

class Light3D : public Node3D
{
public:
    PRISMA3D_NODE_CLASS(Light3D, Node3D);
    static void bindProperties();

    bool saveData(ct::Json& data, SceneResult& result, const ct::String& path) const override;
    bool loadData(const ct::Json& data, SceneResult& result, const ct::String& path) override;

    ~Light3D() override;

    LightId getLight() const;
    LightType getLightType() const;

    void setColor(const Math::Vec3& color);
    Math::Vec3 getColor() const;
    void setEnergy(float energy);
    float getEnergy() const;
    void setSpecular(float specular);
    float getSpecular() const;
    void setCullMask(std::uint32_t mask);
    std::uint32_t getCullMask() const;
    void setNegative(bool negative);
    bool isNegative() const;
    void setShadow(bool enabled);
    bool hasShadow() const;
    void setBakeMode(LightBakeMode mode);
    LightBakeMode getBakeMode() const;
    void setShadowBias(float bias);
    float getShadowBias() const;
    void setShadowNormalBias(float bias);
    float getShadowNormalBias() const;
    void setShadowBlur(float blur);
    float getShadowBlur() const;
    void setShadowOpacity(float opacity);
    float getShadowOpacity() const;
    void setLensFlare(const TextureId* textures, std::uint32_t count);
    void setLensFlare(const ct::Vector<TextureId>& textures);
    void clearLensFlare();
    std::uint32_t getLensFlareCount() const;
    TextureId getLensFlareTexture(std::uint32_t index) const;

protected:
    explicit Light3D(LightType type);

    void setParam(LightParam param, float value);
    float getParam(LightParam param) const;
    void notificationHandler(int what) override;
    void validateProperty(PropertyInfo& info) const override;

private:
    void updateVisibility();

    LightId mLight;
    LightType mType;
    Math::Vec3 mColor;
    float mParams[static_cast<std::size_t>(LightParam::Count)];
    std::uint32_t mCullMask;
    bool mNegative;
    bool mShadow;
    LightBakeMode mBakeMode;
    bool mVisibleInWorld;
    TextureId mLensFlare[kMaxLensFlareElements];
    std::uint32_t mLensFlareCount;
};

class DirectionalLight3D : public Light3D
{
public:
    PRISMA3D_NODE_CLASS(DirectionalLight3D, Light3D);
    static void bindProperties();

    DirectionalLight3D();

    void setAngularDistance(float degrees);
    float getAngularDistance() const;
    void setShadowMode(DirectionalShadowMode mode);
    DirectionalShadowMode getShadowMode() const;
    void setBlendSplits(bool blend);
    bool getBlendSplits() const;
    void setShadowMaxDistance(float distance);
    float getShadowMaxDistance() const;
    void setShadowSplit(int index, float offset);
    float getShadowSplit(int index) const;
    void setShadowFadeStart(float start);
    float getShadowFadeStart() const;
    void setShadowPancakeSize(float size);
    float getShadowPancakeSize() const;

protected:
    void validateProperty(PropertyInfo& info) const override;

private:
    DirectionalShadowMode mShadowMode;
    bool mBlendSplits;
};

class OmniLight3D : public Light3D
{
public:
    PRISMA3D_NODE_CLASS(OmniLight3D, Light3D);
    static void bindProperties();

    OmniLight3D();

    void setRange(float range);
    float getRange() const;
    void setAttenuation(float attenuation);
    float getAttenuation() const;
    void setSize(float size);
    float getSize() const;
    void setShadowMode(OmniShadowMode mode);
    OmniShadowMode getShadowMode() const;

private:
    OmniShadowMode mShadowMode;
};

class SpotLight3D : public Light3D
{
public:
    PRISMA3D_NODE_CLASS(SpotLight3D, Light3D);
    static void bindProperties();

    SpotLight3D();

    void setRange(float range);
    float getRange() const;
    void setAttenuation(float attenuation);
    float getAttenuation() const;
    void setSize(float size);
    float getSize() const;
    void setSpotAngle(float degrees);
    float getSpotAngle() const;
    void setSpotAttenuation(float attenuation);
    float getSpotAttenuation() const;
};

} // namespace prisma3d
