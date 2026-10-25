#pragma once

#include "prisma3d/core/BoundsTree.h"
#include "prisma3d/render/MeshStore.h"

#include "mathc.h"

#include <ct/vector.hpp>

#include <cstdint>

namespace prisma3d
{

class Node3D;

enum class CollisionShapeType : std::uint8_t
{
    None,
    Ellipsoid,
    Box,
    Mesh
};

enum class CollisionMethod : std::uint8_t
{
    Sphere,
    Polygon,
    Box
};

enum class CollisionResponse : std::uint8_t
{
    Stop,
    Slide,
    SlideNoSlope
};

struct SweepHit
{
    std::uint32_t triangle = 0;
    float t = 0.0f;
    Math::Vec3 normal = Math::Vec3(0.0f, 1.0f, 0.0f);
};

class CollisionMesh
{
public:
    bool build(const Math::Vec3* vertices, std::uint32_t vertexCount, const std::uint32_t* indices,
            std::uint32_t indexCount, const Math::Vec3& scale = Math::Vec3(1.0f, 1.0f, 1.0f));
    bool build(const SurfaceData& surface, const Math::Vec3& scale = Math::Vec3(1.0f, 1.0f, 1.0f));
    void append(const Math::Vec3* vertices, std::uint32_t vertexCount, const std::uint32_t* indices,
            std::uint32_t indexCount, const Math::Vec3& scale = Math::Vec3(1.0f, 1.0f, 1.0f));
    bool append(const SurfaceData& surface, const Math::Vec3& scale = Math::Vec3(1.0f, 1.0f, 1.0f));
    bool finish();
    bool buildBox(const Math::Vec3& min, const Math::Vec3& max);
    bool transform(const Math::Mat4& matrix);
    void clear();

    bool valid() const;
    std::uint32_t triangleCount() const;
    std::uint32_t partCount() const;
    int partOfTriangle(std::uint32_t triangle) const;
    void triangle(std::uint32_t index, Math::Vec3& a, Math::Vec3& b, Math::Vec3& c) const;
    const Math::Box& bounds() const;

    bool sweepEllipsoid(const Math::Vec3& localCentre, const Math::Vec3& radii,
            const Math::Vec3& velocity, SweepHit& hit) const;
    bool sweepSphere(const Math::Vec3& localCentre, float radius, const Math::Vec3& velocity,
            SweepHit& hit) const;
    bool raycast(const Math::Ray& localRay, float maxDistance, SweepHit& hit) const;

private:
    ct::Vector<Math::Vec3> mVertices;
    ct::Vector<std::uint32_t> mIndices;
    ct::Vector<std::uint32_t> mPartStarts;
    ct::Vector<Math::Box> mTriangleBounds;
    BoundsTree mTree;
    Math::Box mBounds;
    bool mFinished = false;
    mutable ct::Vector<std::uint32_t> mCandidates;
};

struct CollisionHit
{
    Node3D* node = nullptr;
    Math::Vec3 point;
    Math::Vec3 normal = Math::Vec3(0.0f, 1.0f, 0.0f);
    std::uint32_t triangle = 0;
    float time = 0.0f;
};

struct CollisionPair
{
    int source = 0;
    int destination = 0;
    CollisionMethod method = CollisionMethod::Polygon;
    CollisionResponse response = CollisionResponse::Slide;
};

class CollisionWorld
{
public:
    static constexpr std::uint32_t kMaxSlideIterations = 8;

    CollisionWorld();
    ~CollisionWorld();
    CollisionWorld(const CollisionWorld&) = delete;
    CollisionWorld& operator=(const CollisionWorld&) = delete;

    static CollisionWorld* singleton();

    void setPair(int source, int destination, CollisionMethod method, CollisionResponse response);
    void clearPairs();
    std::uint32_t pairCount() const;
    const CollisionPair& pair(std::uint32_t index) const;

    void setUpDirection(const Math::Vec3& up);
    const Math::Vec3& upDirection() const;
    void setSlopeLimit(float degrees);
    float slopeLimit() const;
    void setSkinWidth(float width);
    float skinWidth() const;

    void update();
    bool raycast(const Math::Ray& ray, float maxDistance, CollisionHit& hit, int type = 0);

    std::uint32_t nodeCount() const;
    std::uint32_t sweepCount() const;

private:
    friend class Node3D;

    struct Target
    {
        Node3D* node = nullptr;
        const CollisionPair* pair = nullptr;
        const CollisionMesh* mesh = nullptr;
        Math::Mat3 rotation;
        Math::Mat3 inverseRotation;
        Math::Vec3 origin;
        float radius = 0.0f;
    };

    struct Contact
    {
        std::uint32_t target = 0;
        SweepHit hit;
        Math::Vec3 normal;
    };

    void add(Node3D* node);
    void remove(Node3D* node);
    void updateNode(Node3D* node);
    bool isSource(int type) const;
    const CollisionPair* findPair(int source, int destination) const;
    void collectTargets(const Node3D* source, int sourceType);
    bool findContact(const Math::Vec3& centre, const Math::Vec3& radii, const Math::Vec3& velocity,
            Contact& contact);
    Math::Vec3 contactCentre(const Contact& contact, const Math::Vec3& centre,
            const Math::Vec3& velocity, const Math::Vec3& radii) const;
    CollisionHit makeHit(const Contact& contact, const Math::Vec3& contactCentre,
            const Math::Vec3& radii) const;

    ct::Vector<Node3D*> mNodes;
    ct::Vector<Target> mTargets;
    ct::Vector<CollisionPair> mPairs;
    Math::Vec3 mUp;
    float mSlopeLimitDegrees;
    float mSlopeLimitCosine;
    float mSkinWidth;
    std::uint32_t mSweeps;
};

} // namespace prisma3d
