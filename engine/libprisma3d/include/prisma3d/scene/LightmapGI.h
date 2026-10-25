#pragma once

#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/scene/Node3D.h"

#include <ct/ptr.hpp>
#include <ct/string.hpp>
#include <ct/vector.hpp>

#include <cstddef>
#include <cstdint>

namespace prisma3d
{

class VisualInstance3D;

class LightmapGIData
{
public:
    static constexpr const char* kFormatName = "prisma3d-lightmap";
    static constexpr std::uint32_t kFormatVersion = 1;

    struct User
    {
        ct::String path;
        std::int32_t subInstance = -1;
        Math::Vec4 uvScale = Math::Vec4(0.0f, 0.0f, 1.0f, 1.0f);
        std::int32_t sliceIndex = 0;
    };

    LightmapGIData();
    ~LightmapGIData();
    LightmapGIData(const LightmapGIData&) = delete;
    LightmapGIData& operator=(const LightmapGIData&) = delete;

    void addUser(const ct::String& path, const Math::Vec4& uvScale, int sliceIndex,
            int subInstance = -1);
    std::size_t getUserCount() const;
    const User& getUser(std::size_t index) const;
    void clearUsers();

    bool setLightImages(std::uint32_t width, std::uint32_t height, std::uint32_t layers,
            const std::uint16_t* rgba16f);
    bool setShadowmaskImages(std::uint32_t width, std::uint32_t height, std::uint32_t layers,
            const std::uint16_t* rgba16f);
    void clearShadowmaskTextures();
    void clearLightTextures();
    bool hasShadowmaskTextures() const;
    TextureId getLightTexture() const;
    TextureId getShadowmaskTexture() const;
    std::uint32_t getLayerCount() const;
    std::uint32_t getLightWidth() const;
    std::uint32_t getLightHeight() const;
    const ct::Vector<std::uint16_t>& getLightPixels() const;

    void setUsesSphericalHarmonics(bool enable);
    bool isUsingSphericalHarmonics() const;
    void setBounds(const Math::Box& bounds);
    const Math::Box& getBounds() const;
    bool setProbeGrid(const Math::Vec3& origin, const Math::Vec3& cell, const std::uint32_t dims[3],
            const ct::Vector<float>& sh);
    bool hasProbes() const;
    const Math::Vec3& getProbeOrigin() const;
    const Math::Vec3& getProbeCell() const;
    const std::uint32_t* getProbeDims() const;
    const ct::Vector<float>& getProbeSh() const;
    void setInterior(bool interior);
    bool isInterior() const;
    void setBakedExposure(float exposure);
    float getBakedExposure() const;

    void releaseImages();
    bool save(const ct::String& path, ct::String* error = nullptr);
    bool load(const ct::String& path, ct::String* error = nullptr);
    void clear();

    static std::uint16_t floatToHalf(float value);
    static float halfToFloat(std::uint16_t value);

private:
    struct Layers
    {
        TextureId texture;
        std::uint32_t width = 0;
        std::uint32_t height = 0;
        std::uint32_t layers = 0;
        ct::Vector<std::uint16_t> pixels;
        ct::String source;
    };

    static bool setImages(Layers& target, std::uint32_t width, std::uint32_t height,
            std::uint32_t layers, const std::uint16_t* rgba16f, const char* name);
    static bool loadLayers(Layers& target, const ct::String& file, std::uint32_t layers,
            const char* name, ct::String* error);
    static bool saveLayers(Layers& source, const ct::String& file, ct::String* error);
    static void freeLayers(Layers& target);

    Layers mLight;
    Layers mShadowmask;
    ct::Vector<User> mUsers;
    bool mUsesSphericalHarmonics;
    bool mInterior;
    float mBakedExposure;
    Math::Box mBounds;
    Math::Vec3 mProbeOrigin;
    Math::Vec3 mProbeCell;
    std::uint32_t mProbeDims[3];
    ct::Vector<float> mProbeSh;
};

class LightmapGI : public Node3D
{
public:
    PRISMA3D_NODE_CLASS(LightmapGI, Node3D);
    static void bindProperties();

    enum class BakeQuality : std::uint8_t
    {
        Low,
        Medium,
        High,
        Ultra
    };

    enum class GenerateProbes : std::uint8_t
    {
        Disabled,
        Subdiv4,
        Subdiv8,
        Subdiv16,
        Subdiv32
    };

    enum class EnvironmentMode : std::uint8_t
    {
        Disabled,
        Scene,
        CustomColor
    };

    struct AtlasCursor
    {
        int layer = -1;
        int shelfX = 0;
        int shelfY = 0;
        int shelfHeight = 0;
    };

    LightmapGI();
    ~LightmapGI() override;

    static void placeBlock(AtlasCursor& cursor, int width, int height, int pageWidth,
            int pageHeight, int gutter, int& x, int& y);
    void setModelAtlas(const AtlasCursor& cursor);
    bool isModelAtlas() const;

    LightmapId getLightmap() const;

    void setLightData(ct::Unique<LightmapGIData> data, const ct::String& path = ct::String());
    LightmapGIData* getLightData() const;
    void updateLightData();
    void setLightDataPath(const ct::String& path);
    const ct::String& getLightDataPath() const;

    void setBakeQuality(BakeQuality quality);
    BakeQuality getBakeQuality() const;
    void setUseDenoiser(bool enable);
    bool isUsingDenoiser() const;
    void setDenoiserStrength(float strength);
    float getDenoiserStrength() const;
    void setDenoiserRange(int range);
    int getDenoiserRange() const;
    void setDirectional(bool enable);
    bool isDirectional() const;
    void setShadowmaskMode(ShadowmaskMode mode);
    ShadowmaskMode getShadowmaskMode() const;
    void setUseTextureForBounces(bool enable);
    bool isUsingTextureForBounces() const;
    void setInterior(bool interior);
    bool isInterior() const;
    void setEnvironmentMode(EnvironmentMode mode);
    EnvironmentMode getEnvironmentMode() const;
    void setEnvironmentCustomColor(const Math::Vec3& color);
    const Math::Vec3& getEnvironmentCustomColor() const;
    void setEnvironmentCustomEnergy(float energy);
    float getEnvironmentCustomEnergy() const;
    void setBounces(int bounces);
    int getBounces() const;
    void setBounceIndirectEnergy(float energy);
    float getBounceIndirectEnergy() const;
    void setBias(float bias);
    float getBias() const;
    void setTexelScale(float scale);
    float getTexelScale() const;
    void setMaxTextureSize(int size);
    int getMaxTextureSize() const;
    void setSupersamplingEnabled(bool enable);
    bool isSupersamplingEnabled() const;
    void setSupersamplingFactor(float factor);
    float getSupersamplingFactor() const;
    void setGenerateProbes(GenerateProbes probes);
    GenerateProbes getGenerateProbes() const;
    void setSpecularIntensity(float intensity);
    float getSpecularIntensity() const;

    Math::Box getAabb() const;

protected:
    void notificationHandler(int what) override;
    void validateProperty(PropertyInfo& info) const override;

private:
    void push();
    void updateVisibility();
    void assignLightmaps();
    void clearLightmaps();
    VisualInstance3D* resolveUser(std::size_t index) const;
    bool mergeIntoSharedAtlas();
    bool mergeInto(LightmapGI& atlas);
    static ct::Vector<LightmapGI*>& atlases();

    LightmapId mLightmap;
    ct::Unique<LightmapGIData> mData;
    ct::String mDataPath;
    bool mVisibleInWorld;
    bool mAssigned;
    BakeQuality mBakeQuality;
    bool mUseDenoiser;
    float mDenoiserStrength;
    int mDenoiserRange;
    int mBounces;
    float mBounceIndirectEnergy;
    float mBias;
    float mTexelScale;
    int mMaxTextureSize;
    bool mSupersamplingEnabled;
    float mSupersamplingFactor;
    bool mInterior;
    EnvironmentMode mEnvironmentMode;
    Math::Vec3 mEnvironmentCustomColor;
    float mEnvironmentCustomEnergy;
    bool mDirectional;
    bool mUseTextureForBounces;
    ShadowmaskMode mShadowmaskMode;
    GenerateProbes mGenerateProbes;
    float mSpecularIntensity;
    bool mModelAtlas;
    bool mMerged;
    AtlasCursor mCursor;
};

} // namespace prisma3d
