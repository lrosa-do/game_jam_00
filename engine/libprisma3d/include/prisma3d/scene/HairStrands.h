#pragma once

#include "mathc.h"

#include <ct/vector.hpp>

#include <cstdint>

namespace prisma3d
{

struct HairAtlasRect
{
    Math::Vec4 texMulAdd = Math::Vec4(1.0f, 1.0f, 0.0f, 0.0f);
    float size = 1.0f;
};

struct HairParams
{
    std::uint32_t strandCount = 0;
    std::uint32_t segmentCount = 1;
    std::uint32_t billboardCount = 1;
    std::uint32_t randomSeed = 1;
    float length = 1.0f;
    float stiffness = 0.5f;
    float drag = 0.1f;
    float gravityPower = 0.0f;
    float randomness = 0.2f;
    float viewDistance = 200.0f;
    float width = 1.0f;
    float uniformity = 1.0f;
    float textureAspect = 1.0f;
    bool cameraBend = true;
    ct::Vector<HairAtlasRect> atlasRects;
};

enum class HairColliderType : std::uint8_t
{
    Sphere,
    Capsule,
    Plane
};

struct HairCollider
{
    HairColliderType type = HairColliderType::Sphere;
    Math::Vec3 position;
    Math::Vec3 tip;
    Math::Vec3 normal = Math::Vec3(0.0f, 1.0f, 0.0f);
    float radius = 1.0f;
    Math::Mat4 planeProjection = Math::Mat4::Identity();

    static HairCollider sphere(const Math::Vec3& center, float radius);
    static HairCollider capsule(const Math::Vec3& base, const Math::Vec3& tip, float radius);
    static HairCollider plane(const Math::Mat4& world, const Math::Vec3& offset, float radius);
};

enum class HairForceType : std::uint8_t
{
    Point,
    Plane
};

struct HairForceField
{
    HairForceType type = HairForceType::Point;
    Math::Vec3 position;
    Math::Vec3 direction = Math::Vec3(0.0f, 1.0f, 0.0f);
    float gravity = 0.0f;
    float range = 1.0f;
};

struct HairFrame
{
    Math::Mat4 transform = Math::Mat4::Identity();
    Math::Vec3 cameraPosition;
    Math::Vec3 cameraUp = Math::Vec3(0.0f, 1.0f, 0.0f);
    float deltaTime = 0.0f;
    float time = 0.0f;
    Math::Vec3 windDirection;
    float windSpeed = 1.0f;
    float windRandomness = 5.0f;
    float windWaveSize = 1.0f;
    const HairCollider* colliders = nullptr;
    std::uint32_t colliderCount = 0;
    const HairForceField* forces = nullptr;
    std::uint32_t forceCount = 0;
};

class HairStrands
{
public:
    static constexpr std::uint32_t kWindTextureSize = 32;

    void setParams(const HairParams& params);
    const HairParams& params() const;

    void setScalp(const Math::Vec3* positions, const Math::Vec3* normals, const float* lengths,
            std::uint32_t vertexCount, const std::uint32_t* indices, std::uint32_t indexCount);
    void updateScalp(const Math::Vec3* positions, const Math::Vec3* normals);
    std::uint32_t scalpTriangleCount() const;
    const std::uint32_t* scalpIndices() const;
    const float* scalpLengths() const;

    void reset();
    void simulate(const HairFrame& frame);
    void beginFrame(const HairFrame& frame);
    void windSlicesToCompute(std::uint32_t& begin, std::uint32_t& end) const;
    void computeWindSlices(std::uint32_t begin, std::uint32_t end);
    void simulateRange(std::uint32_t begin, std::uint32_t end);
    void endFrame();
    void setWindSlicesPerFrame(std::uint32_t slices);
    std::uint32_t windSlicesPerFrame() const;

    std::uint32_t vertexCount() const;
    std::uint32_t indexCount() const;
    std::uint32_t verticesPerStrand() const;
    const Math::Vec3* positions() const;
    const Math::Vec3* normals() const;
    const Math::Vec2* uvs() const;
    const std::uint32_t* indices() const;
    Math::Vec3 tail(std::uint32_t strand, std::uint32_t segment) const;

    static Math::Box expandBounds(const Math::Box& scalpBounds, const HairParams& params,
            const Math::Mat4& world);

    static float computeWind(const Math::Vec3& texel, float time, float speed, float randomness,
            float waveSize);
    static float noiseGradient3D(const Math::Vec3& p);
    Math::Vec3 sampleWind(const Math::Vec3& position, float weight) const;

private:
    struct Tail
    {
        Math::Vec3 previous;
        Math::Vec3 current;
    };

    struct WindLattice
    {
        ct::Vector<Math::Vec3> gradients;
        int base = 0;
        int size = 0;
        bool valid = false;
    };

    static constexpr std::uint32_t kWindOctaves = 6;
    static constexpr int kWindLatticeLimit = 48;

    void prepareWindLattices();
    float latticeNoise(const Math::Vec3& p, const WindLattice& lattice) const;

    void ensureBuffers();
    void simulateStrand(std::uint32_t strand);
    void buildIndices();

    HairParams mParams;
    ct::Vector<Math::Vec3> mScalpPositions;
    ct::Vector<Math::Vec3> mScalpNormals;
    ct::Vector<float> mScalpLengths;
    ct::Vector<std::uint32_t> mScalpIndices;
    ct::Vector<Tail> mTails;
    ct::Vector<Math::Vec3> mPositions;
    ct::Vector<Math::Vec3> mNormals;
    ct::Vector<Math::Vec2> mUvs;
    ct::Vector<std::uint32_t> mIndices;
    ct::Vector<float> mWind;
    WindLattice mWindLattices[kWindOctaves];
    std::uint32_t mWindSlicesPerFrame = kWindTextureSize;
    std::uint32_t mWindCursor = 0;
    std::uint32_t mWindBegin = 0;
    std::uint32_t mWindEnd = 0;
    HairFrame mFrame;
    Math::Mat4 mNormalTransform = Math::Mat4::Identity();
    float mAspect = 1.0f;
    bool mWindActive = false;
    bool mRegenerate = true;
    bool mIndicesDirty = true;
    std::uint32_t mBuiltStrands = 0;
    std::uint32_t mBuiltSegments = 0;
    std::uint32_t mBuiltBillboards = 0;
};

} // namespace prisma3d
