#pragma once

#include "mathc.h"

#include <prisma/rhi/Driver.h>

#include <ct/vector.hpp>

#include <cstdint>

namespace prisma3d
{

class ClusterBuilder;

class ClusterBuilderShared
{
public:
    ClusterBuilderShared();
    ~ClusterBuilderShared();
    ClusterBuilderShared(const ClusterBuilderShared&) = delete;
    ClusterBuilderShared& operator=(const ClusterBuilderShared&) = delete;

    bool create(prisma::Driver* driver);
    void destroy();

    bool ready() const;
    prisma::Driver* driver() const;
    float sphereOverfit() const;
    float coneOverfit() const;

private:
    friend class ClusterBuilder;

    struct Mesh
    {
        prisma::BufferHandle vertexBuffer;
        prisma::BufferHandle indexBuffer;
        std::uint32_t indexCount = 0;
    };

    bool createMesh(Mesh& mesh, const float* vertices, std::uint32_t vertexCount,
            const std::uint16_t* indices, std::uint32_t indexCount, const char* name);

    prisma::Driver* mDriver;
    Mesh mSphere;
    Mesh mCone;
    Mesh mBox;
    float mSphereOverfit;
    float mConeOverfit;
    prisma::ShaderHandle mRenderVertexShader;
    prisma::ShaderHandle mRenderFragmentShader;
    prisma::PipelineHandle mRenderPipeline;
    prisma::ShaderHandle mStoreShader;
    prisma::PipelineHandle mStorePipeline;
    prisma::ShaderHandle mClearShader;
    prisma::PipelineHandle mClearPipeline;
};

class ClusterBuilder
{
public:
    enum LightType : std::uint32_t
    {
        kLightOmni,
        kLightSpot,
        kLightArea
    };

    enum BoxType : std::uint32_t
    {
        kBoxReflectionProbe,
        kBoxDecal
    };

    enum ElementType : std::uint32_t
    {
        kElementOmniLight,
        kElementSpotLight,
        kElementAreaLight,
        kElementDecal,
        kElementReflectionProbe,
        kElementTypeMax
    };

    static constexpr float kWideSpotAngleThresholdDeg = 60.0f;

    ClusterBuilder();
    ~ClusterBuilder();
    ClusterBuilder(const ClusterBuilder&) = delete;
    ClusterBuilder& operator=(const ClusterBuilder&) = delete;

    void setShared(ClusterBuilderShared* shared);
    bool setup(std::uint32_t screenWidth, std::uint32_t screenHeight, std::uint32_t maxElements);
    void destroy();

    void begin(const Math::Mat4& cameraTransform, const Math::Mat4& projection, bool flipY);
    void addLight(LightType type, const Math::Mat4& transform, float radius, float spotAperture,
            const Math::Vec2& areaSize = Math::Vec2(0.0f, 0.0f));
    void addBox(BoxType type, const Math::Mat4& transform, const Math::Vec3& halfSize);
    bool bake();

    prisma::BufferHandle getClusterBuffer() const;
    std::uint32_t getClusterBufferSize() const;
    std::uint32_t getClusterSize() const;
    std::uint32_t getClusterShift() const;
    std::uint32_t getClusterWidth() const;
    std::uint32_t getClusterHeight() const;
    std::uint32_t getClusterTypeSize() const;
    std::uint32_t getMaxClusterElements() const;
    std::uint32_t getMaxClusterElementCountDiv32() const;
    std::uint32_t getScreenWidth() const;
    std::uint32_t getScreenHeight() const;
    std::uint32_t getClusterCountByType(ElementType type) const;
    std::uint32_t getElementCount() const;
    float getZNear() const;
    float getZFar() const;

private:
    struct RenderElementData
    {
        std::uint32_t type;
        std::uint32_t touchesNear;
        std::uint32_t touchesFar;
        std::uint32_t originalIndex;
        float transformInv[12];
        float scale[3];
        std::uint32_t hasWideSpotAngle;
    };

    struct StateUniform
    {
        float projection[16];
        float invZFar;
        std::uint32_t screenToClustersShift;
        std::uint32_t clusterScreenWidth;
        std::uint32_t clusterDataSize;
        std::uint32_t clusterDepthOffset;
        std::uint32_t pad0;
        std::uint32_t pad1;
        std::uint32_t pad2;
    };

    struct DrawParams
    {
        std::uint32_t baseIndex;
        std::uint32_t pad0;
        std::uint32_t pad1;
        std::uint32_t pad2;
    };

    struct StoreParams
    {
        std::uint32_t clusterRenderDataSize;
        std::uint32_t maxRenderElementCountDiv32;
        std::uint32_t clusterScreenSize[2];
        std::uint32_t renderElementCountDiv32;
        std::uint32_t maxClusterElementCountDiv32;
        std::uint32_t pad1;
        std::uint32_t pad2;
    };

    struct ClearParams
    {
        std::uint32_t wordCount;
        std::uint32_t rowWidth;
        std::uint32_t pad0;
        std::uint32_t pad1;
    };

    void dispatchClear(prisma::BufferHandle buffer, std::uint32_t bytes, std::uint32_t slot);
    bool fits(ElementType type) const;

    ClusterBuilderShared* mShared;
    prisma::Driver* mDriver;

    std::uint32_t mClusterCountByType[kElementTypeMax];
    std::uint32_t mMaxElementsByType;
    ct::Vector<RenderElementData> mRenderElements;
    std::uint32_t mRenderElementCount;
    std::uint32_t mRenderElementMax;

    Math::Mat4 mViewTransform;
    Math::Mat4 mAdjustedProjection;
    Math::Mat4 mProjection;
    float mZFar;
    float mZNear;
    bool mCameraOrthogonal;

    std::uint32_t mClusterSize;
    std::uint32_t mDivisor;
    std::uint32_t mScreenWidth;
    std::uint32_t mScreenHeight;
    std::uint32_t mClusterWidth;
    std::uint32_t mClusterHeight;

    prisma::TextureHandle mTarget;
    prisma::BufferHandle mClusterRenderBuffer;
    prisma::BufferHandle mClusterBuffer;
    prisma::BufferHandle mElementBuffer;
    prisma::BufferHandle mStateBuffer;
    prisma::BufferHandle mDrawParamsBuffer;
    prisma::BufferHandle mStoreParamsBuffer;
    prisma::BufferHandle mClearParamsBuffer;
    std::uint32_t mClusterRenderBufferSize;
    std::uint32_t mClusterBufferSize;
    std::uint32_t mUniformStride;
    ct::Vector<unsigned char> mDrawParamsData;
};

} // namespace prisma3d
