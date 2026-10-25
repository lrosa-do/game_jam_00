#include "prisma3d/scene/Collision.h"

#include "prisma3d/scene/Node3D.h"

#include <math.h>

namespace prisma3d
{

namespace
{

const float kPi = 3.14159265358979f;
const float kMinSpeedSquared = 1e-12f;

CollisionWorld* gWorld = nullptr;

bool pointInTriangle(const Math::Vec3& p, const Math::Vec3& v1, const Math::Vec3& v2,
        const Math::Vec3& v3, const Math::Vec3& normal)
{
    Math::Vec3 e = v2 - v1;
    Math::Vec3 d = v1 - p;
    if (d.Dot(e.Cross(normal)) < 0.0f) return false;
    e = v3 - v2;
    d = v2 - p;
    if (d.Dot(e.Cross(normal)) < 0.0f) return false;
    e = v1 - v3;
    d = v3 - p;
    if (d.Dot(e.Cross(normal)) < 0.0f) return false;
    return true;
}

bool solveCollision(float a, float b, float c, float& t)
{
    const float disc = b * b - 4.0f * a * c;
    if (disc < 0.0f) return false;
    const float sq = sqrtf(disc);
    const float inv = 1.0f / (2.0f * a);
    float t0 = (-b - sq) * inv;
    float t1 = (-b + sq) * inv;
    if (t1 < t0)
    {
        const float swap = t0;
        t0 = t1;
        t1 = swap;
    }
    if (t1 < 0.0f) return false;
    t = (t0 < 0.0f) ? t1 : t0;
    return true;
}

bool sphereIntersectPlane(const Math::Vec3& center, float radius, const Math::Vec3& velocity,
        const Math::Vec3& planeNormal, const Math::Vec3& planePoint, float& tMax)
{
    const float numer = (center - planePoint).Dot(planeNormal) - radius;
    const float denom = velocity.Dot(planeNormal);

    if (numer < 0.0f || denom > -1e-7f)
    {
        if (denom > -1e-5f) return false;
        if (numer < -radius) return false;
        tMax = numer;
        return true;
    }

    const float t = -(numer / denom);
    if (t < 0.0f || t > tMax) return false;
    tMax = t;
    return true;
}

bool sphereIntersectPoint(const Math::Vec3& center, float radius, const Math::Vec3& velocity,
        const Math::Vec3& point, float& tMax, Math::Vec3& normal)
{
    const Math::Vec3 l = center - point;
    const float l2 = l.Dot(l);
    const float a = velocity.Dot(velocity);
    const float b = 2.0f * velocity.Dot(l);
    const float c = l2 - radius * radius;

    if (c < 0.0f)
    {
        const float len = sqrtf(l2);
        const float t = len - radius;
        if (tMax < t) return false;
        normal = l / len;
        tMax = t;
        return true;
    }

    if (tMax < 0.0f) return false;
    float t = 0.0f;
    if (!solveCollision(a, b, c, t)) return false;
    if (t > tMax) return false;
    normal = (center + velocity * t - point).NormalizedSafe();
    tMax = t;
    return true;
}

bool sphereIntersectSegment(const Math::Vec3& center, float radius, const Math::Vec3& velocity,
        const Math::Vec3& v1, const Math::Vec3& v2, float& tMax, Math::Vec3& normal)
{
    Math::Vec3 e = v2 - v1;
    const Math::Vec3 l = center - v1;
    const float eLen = e.Length();
    if (eLen < 1e-5f) return false;
    e /= eLen;

    const Math::Vec3 x = l.Cross(e);
    const Math::Vec3 y = velocity.Cross(e);
    const float a = y.Dot(y);
    const float b = 2.0f * x.Dot(y);
    const float c = x.Dot(x) - radius * radius;

    if (c < 0.0f)
    {
        const float d = l.Dot(e);
        if (d < 0.0f) return sphereIntersectPoint(center, radius, velocity, v1, tMax, normal);
        if (d > eLen) return sphereIntersectPoint(center, radius, velocity, v2, tMax, normal);
        const Math::Vec3 onEdge = v1 + e * d;
        normal = (center - onEdge).NormalizedSafe();
        const float t = (center - onEdge).Length() - radius;
        if (tMax < t) return false;
        tMax = t;
        return true;
    }

    if (tMax < 0.0f) return false;
    float t = 0.0f;
    if (!solveCollision(a, b, c, t)) return false;
    if (t > tMax) return false;

    const Math::Vec3 collCenter = center + velocity * t;
    const float d = (collCenter - v1).Dot(e);
    if (d < 0.0f) return sphereIntersectPoint(center, radius, velocity, v1, tMax, normal);
    if (d > eLen) return sphereIntersectPoint(center, radius, velocity, v2, tMax, normal);

    normal = (collCenter - (v1 + e * d)).NormalizedSafe();
    tMax = t;
    return true;
}

bool sphereIntersectTriangle(const Math::Vec3& center, float radius, const Math::Vec3& velocity,
        const Math::Vec3& v0, const Math::Vec3& v1, const Math::Vec3& v2, const Math::Vec3& normal,
        float& tMax, Math::Vec3& outNormal)
{
    float t = tMax;
    if (!sphereIntersectPlane(center, radius, velocity, normal, v0, t)) return false;

    const Math::Vec3 collCenter = (t < 0.0f) ? center - normal * t : center + velocity * t;
    if (pointInTriangle(collCenter, v0, v1, v2, normal))
    {
        outNormal = normal;
        tMax = t;
        return true;
    }

    bool hit = false;
    hit |= sphereIntersectSegment(center, radius, velocity, v0, v1, tMax, outNormal);
    hit |= sphereIntersectSegment(center, radius, velocity, v1, v2, tMax, outNormal);
    hit |= sphereIntersectSegment(center, radius, velocity, v2, v0, tMax, outNormal);
    return hit;
}

bool rayIntersectTriangle(const Math::Ray& ray, const Math::Vec3& v0, const Math::Vec3& v1,
        const Math::Vec3& v2, float& t)
{
    const Math::Vec3 edge1 = v1 - v0;
    const Math::Vec3 edge2 = v2 - v0;

    const Math::Vec3 h = ray.direction.Cross(edge2);
    const float a = edge1.Dot(h);
    if (fabsf(a) < 1e-8f) return false;

    const float f = 1.0f / a;
    const Math::Vec3 s = ray.origin - v0;
    const float u = f * s.Dot(h);
    if (u < 0.0f || u > 1.0f) return false;

    const Math::Vec3 q = s.Cross(edge1);
    const float v = f * ray.direction.Dot(q);
    if (v < 0.0f || u + v > 1.0f) return false;

    const float hit = f * edge2.Dot(q);
    if (hit < 0.0f) return false;

    t = hit;
    return true;
}

Math::Mat3 orthonormalBasis(const Math::Mat4& transform)
{
    Math::Vec3 x = transform.col0.xyz();
    Math::Vec3 y = transform.col1.xyz();
    Math::Vec3 z = transform.col2.xyz();
    x = x.LengthSquared() > 1e-12f ? x.Normalized() : Math::Vec3(1.0f, 0.0f, 0.0f);
    y = y.LengthSquared() > 1e-12f ? y.Normalized() : Math::Vec3(0.0f, 1.0f, 0.0f);
    z = z.LengthSquared() > 1e-12f ? z.Normalized() : Math::Vec3(0.0f, 0.0f, 1.0f);
    return Math::Mat3(x, y, z);
}

} // namespace

bool CollisionMesh::build(const Math::Vec3* vertices, std::uint32_t vertexCount,
        const std::uint32_t* indices, std::uint32_t indexCount, const Math::Vec3& scale)
{
    clear();
    append(vertices, vertexCount, indices, indexCount, scale);
    return finish();
}

bool CollisionMesh::build(const SurfaceData& surface, const Math::Vec3& scale)
{
    clear();
    append(surface, scale);
    return finish();
}

void CollisionMesh::append(const Math::Vec3* vertices, std::uint32_t vertexCount,
        const std::uint32_t* indices, std::uint32_t indexCount, const Math::Vec3& scale)
{
    mPartStarts.push_back(triangleCount());
    if (!vertices || !indices || vertexCount == 0 || indexCount == 0 || indexCount % 3 != 0) return;
    for (std::uint32_t i = 0; i < indexCount; ++i)
    {
        if (indices[i] >= vertexCount) return;
    }

    const std::uint32_t base = static_cast<std::uint32_t>(mVertices.size());
    const std::size_t indexBase = mIndices.size();
    mVertices.resize(base + vertexCount);
    for (std::uint32_t i = 0; i < vertexCount; ++i) mVertices[base + i] = vertices[i] * scale;

    const bool mirrored = scale.x * scale.y * scale.z < 0.0f;
    mIndices.resize(indexBase + indexCount);
    for (std::uint32_t i = 0; i < indexCount; i += 3)
    {
        mIndices[indexBase + i] = base + indices[i];
        mIndices[indexBase + i + 1] = base + (mirrored ? indices[i + 2] : indices[i + 1]);
        mIndices[indexBase + i + 2] = base + (mirrored ? indices[i + 1] : indices[i + 2]);
    }
    mFinished = false;
}

bool CollisionMesh::append(const SurfaceData& surface, const Math::Vec3& scale)
{
    const std::uint32_t vertexCount = static_cast<std::uint32_t>(surface.positions.size());
    if (surface.topology != prisma::Topology::Triangles || vertexCount == 0)
    {
        mPartStarts.push_back(triangleCount());
        return false;
    }
    const std::size_t before = mIndices.size();
    if (!surface.indices.empty())
    {
        append(surface.positions.data(), vertexCount, surface.indices.data(),
                static_cast<std::uint32_t>(surface.indices.size()), scale);
        return mIndices.size() > before;
    }

    ct::Vector<std::uint32_t> sequence;
    sequence.resize(vertexCount);
    for (std::uint32_t i = 0; i < vertexCount; ++i) sequence[i] = i;
    append(surface.positions.data(), vertexCount, sequence.data(), vertexCount, scale);
    return mIndices.size() > before;
}

bool CollisionMesh::finish()
{
    mFinished = false;
    mTree.clear();
    mTriangleBounds.clear();
    mBounds = Math::Box();
    if (mIndices.empty()) return false;

    const std::uint32_t triangles = static_cast<std::uint32_t>(mIndices.size() / 3);
    mTriangleBounds.resize(triangles);
    for (std::uint32_t i = 0; i < triangles; ++i)
    {
        const Math::Vec3& a = mVertices[mIndices[i * 3]];
        const Math::Vec3& b = mVertices[mIndices[i * 3 + 1]];
        const Math::Vec3& c = mVertices[mIndices[i * 3 + 2]];
        Math::Box box(Math::Vec3::Min(a, Math::Vec3::Min(b, c)),
                Math::Vec3::Max(a, Math::Vec3::Max(b, c)));
        mTriangleBounds[i] = box;
        mBounds = mBounds.Union(box);
    }

    mTree.setLeafCapacity(4);
    mTree.build(mTriangleBounds.data(), triangles);
    mFinished = true;
    return true;
}

bool CollisionMesh::buildBox(const Math::Vec3& min, const Math::Vec3& max)
{
    if (!(min.x < max.x && min.y < max.y && min.z < max.z))
    {
        clear();
        return false;
    }

    Math::Vec3 corners[8];
    for (int i = 0; i < 8; ++i)
    {
        corners[i] = Math::Vec3((i & 1) ? max.x : min.x, (i & 2) ? max.y : min.y,
                (i & 4) ? max.z : min.z);
    }

    static const std::uint32_t indices[36] = { 1, 3, 7, 1, 7, 5, 0, 4, 6, 0, 6, 2, 2, 6, 7, 2, 7, 3,
        0, 1, 5, 0, 5, 4, 4, 5, 7, 4, 7, 6, 0, 2, 3, 0, 3, 1 };
    return build(corners, 8, indices, 36);
}

bool CollisionMesh::transform(const Math::Mat4& matrix)
{
    if (mIndices.empty()) return false;
    for (std::size_t i = 0; i < mVertices.size(); ++i)
        mVertices[i] = matrix.TransformPoint(mVertices[i]);
    if (matrix.UpperLeft3x3().Determinant() < 0.0f)
    {
        for (std::size_t i = 0; i + 2 < mIndices.size(); i += 3)
        {
            const std::uint32_t swap = mIndices[i + 1];
            mIndices[i + 1] = mIndices[i + 2];
            mIndices[i + 2] = swap;
        }
    }
    return finish();
}

void CollisionMesh::clear()
{
    mVertices.clear();
    mIndices.clear();
    mPartStarts.clear();
    mTriangleBounds.clear();
    mTree.clear();
    mBounds = Math::Box();
    mCandidates.clear();
    mFinished = false;
}

bool CollisionMesh::valid() const
{
    return mFinished && !mIndices.empty();
}

std::uint32_t CollisionMesh::partCount() const
{
    return static_cast<std::uint32_t>(mPartStarts.size());
}

int CollisionMesh::partOfTriangle(std::uint32_t triangle) const
{
    if (triangle >= triangleCount() || mPartStarts.empty()) return -1;
    std::size_t low = 0;
    std::size_t high = mPartStarts.size();
    while (high - low > 1)
    {
        const std::size_t middle = (low + high) / 2;
        if (mPartStarts[middle] <= triangle) low = middle;
        else
            high = middle;
    }
    return static_cast<int>(low);
}

std::uint32_t CollisionMesh::triangleCount() const
{
    return static_cast<std::uint32_t>(mIndices.size() / 3);
}

void CollisionMesh::triangle(std::uint32_t index, Math::Vec3& a, Math::Vec3& b, Math::Vec3& c) const
{
    const std::uint32_t base = index * 3;
    a = mVertices[mIndices[base]];
    b = mVertices[mIndices[base + 1]];
    c = mVertices[mIndices[base + 2]];
}

const Math::Box& CollisionMesh::bounds() const
{
    return mBounds;
}

bool CollisionMesh::sweepEllipsoid(const Math::Vec3& localCentre, const Math::Vec3& radii,
        const Math::Vec3& velocity, SweepHit& hit) const
{
    if (!valid() || radii.x <= 0.0f || radii.y <= 0.0f || radii.z <= 0.0f) return false;

    float largest = radii.x > radii.y ? radii.x : radii.y;
    if (radii.z > largest) largest = radii.z;
    const Math::Vec3 pad(largest + 0.01f);
    Math::Box swept;
    swept = swept.Expanded(localCentre - pad);
    swept = swept.Expanded(localCentre + pad);
    swept = swept.Expanded(localCentre + velocity - pad);
    swept = swept.Expanded(localCentre + velocity + pad);

    mTree.queryCandidates(swept, mCandidates);
    if (mCandidates.empty()) return false;

    const Math::Vec3 inverseRadii(1.0f / radii.x, 1.0f / radii.y, 1.0f / radii.z);
    const Math::Vec3 eCentre = localCentre * inverseRadii;
    const Math::Vec3 eVelocity = velocity * inverseRadii;

    float best = 1.0f;
    bool found = false;
    for (std::size_t i = 0; i < mCandidates.size(); ++i)
    {
        const std::uint32_t index = mCandidates[i];
        const std::uint32_t base = index * 3;
        const Math::Vec3& v0 = mVertices[mIndices[base]];
        const Math::Vec3& v1 = mVertices[mIndices[base + 1]];
        const Math::Vec3& v2 = mVertices[mIndices[base + 2]];
        const Math::Vec3 raw = (v1 - v0).Cross(v2 - v0);
        if (raw.Length() < 1.0e-12f) continue;

        const Math::Vec3 e0 = v0 * inverseRadii;
        const Math::Vec3 e1 = v1 * inverseRadii;
        const Math::Vec3 e2 = v2 * inverseRadii;
        const Math::Vec3 eNormal = (raw.NormalizedSafe() * radii).NormalizedSafe();

        float t = best;
        Math::Vec3 eHitNormal(0.0f);
        if (!sphereIntersectTriangle(eCentre, 1.0f, eVelocity, e0, e1, e2, eNormal, t, eHitNormal))
            continue;
        best = t;
        hit.t = t;
        hit.triangle = index;
        hit.normal = (eHitNormal * inverseRadii).NormalizedSafe();
        found = true;
    }
    return found;
}

bool CollisionMesh::sweepSphere(const Math::Vec3& localCentre, float radius,
        const Math::Vec3& velocity, SweepHit& hit) const
{
    return sweepEllipsoid(localCentre, Math::Vec3(radius), velocity, hit);
}

bool CollisionMesh::raycast(const Math::Ray& localRay, float maxDistance, SweepHit& hit) const
{
    if (!valid()) return false;

    mTree.queryCandidates(localRay, maxDistance, mCandidates);
    if (mCandidates.empty()) return false;

    bool found = false;
    float nearest = maxDistance;
    for (std::size_t i = 0; i < mCandidates.size(); ++i)
    {
        const std::uint32_t index = mCandidates[i];
        const std::uint32_t base = index * 3;
        const Math::Vec3& v0 = mVertices[mIndices[base]];
        const Math::Vec3& v1 = mVertices[mIndices[base + 1]];
        const Math::Vec3& v2 = mVertices[mIndices[base + 2]];
        float distance = 0.0f;
        if (!rayIntersectTriangle(localRay, v0, v1, v2, distance) || distance >= nearest) continue;
        nearest = distance;
        hit.triangle = index;
        hit.t = distance;
        const Math::Vec3 raw = (v1 - v0).Cross(v2 - v0);
        const float length = raw.Length();
        hit.normal = length > 1.0e-8f ? raw / length : Math::Vec3(0.0f, 1.0f, 0.0f);
        found = true;
    }
    return found;
}

CollisionWorld::CollisionWorld()
        : mNodes(),
          mTargets(),
          mPairs(),
          mUp(0.0f, 1.0f, 0.0f),
          mSlopeLimitDegrees(45.0f),
          mSlopeLimitCosine(cosf(45.0f * kPi / 180.0f)),
          mSkinWidth(0.01f),
          mSweeps(0)
{
    gWorld = this;
}

CollisionWorld::~CollisionWorld()
{
    if (gWorld == this) gWorld = nullptr;
}

CollisionWorld* CollisionWorld::singleton()
{
    return gWorld;
}

void CollisionWorld::setPair(int source, int destination, CollisionMethod method,
        CollisionResponse response)
{
    for (std::size_t i = 0; i < mPairs.size(); ++i)
    {
        if (mPairs[i].source == source && mPairs[i].destination == destination)
        {
            mPairs[i].method = method;
            mPairs[i].response = response;
            return;
        }
    }
    CollisionPair pair;
    pair.source = source;
    pair.destination = destination;
    pair.method = method;
    pair.response = response;
    mPairs.push_back(pair);
}

void CollisionWorld::clearPairs()
{
    mPairs.clear();
}

std::uint32_t CollisionWorld::pairCount() const
{
    return static_cast<std::uint32_t>(mPairs.size());
}

const CollisionPair& CollisionWorld::pair(std::uint32_t index) const
{
    return mPairs[index];
}

void CollisionWorld::setUpDirection(const Math::Vec3& up)
{
    if (up.LengthSquared() < 1e-12f) return;
    mUp = up.Normalized();
}

const Math::Vec3& CollisionWorld::upDirection() const
{
    return mUp;
}

void CollisionWorld::setSlopeLimit(float degrees)
{
    mSlopeLimitDegrees = degrees;
    mSlopeLimitCosine = cosf(degrees * kPi / 180.0f);
}

float CollisionWorld::slopeLimit() const
{
    return mSlopeLimitDegrees;
}

void CollisionWorld::setSkinWidth(float width)
{
    mSkinWidth = width < 0.0f ? 0.0f : width;
}

float CollisionWorld::skinWidth() const
{
    return mSkinWidth;
}

std::uint32_t CollisionWorld::nodeCount() const
{
    return static_cast<std::uint32_t>(mNodes.size());
}

std::uint32_t CollisionWorld::sweepCount() const
{
    return mSweeps;
}

void CollisionWorld::add(Node3D* node)
{
    for (std::size_t i = 0; i < mNodes.size(); ++i)
    {
        if (mNodes[i] == node) return;
    }
    mNodes.push_back(node);
}

void CollisionWorld::remove(Node3D* node)
{
    for (std::size_t i = 0; i < mNodes.size(); ++i)
    {
        if (mNodes[i] == node)
        {
            mNodes.erase(mNodes.begin() + i);
            break;
        }
    }

    for (std::size_t i = 0; i < mNodes.size(); ++i)
    {
        ct::Vector<CollisionHit>& hits = mNodes[i]->mCollision->hits;
        for (std::size_t h = hits.size(); h > 0; --h)
        {
            if (hits[h - 1].node == node) hits.erase(hits.begin() + (h - 1));
        }
    }
}

bool CollisionWorld::isSource(int type) const
{
    for (std::size_t i = 0; i < mPairs.size(); ++i)
    {
        if (mPairs[i].source == type) return true;
    }
    return false;
}

const CollisionPair* CollisionWorld::findPair(int source, int destination) const
{
    for (std::size_t i = 0; i < mPairs.size(); ++i)
    {
        if (mPairs[i].source == source && mPairs[i].destination == destination) return &mPairs[i];
    }
    return nullptr;
}

bool CollisionWorld::raycast(const Math::Ray& ray, float maxDistance, CollisionHit& hit, int type)
{
    bool found = false;
    float nearest = maxDistance;
    for (std::size_t i = 0; i < mNodes.size(); ++i)
    {
        Node3D* node = mNodes[i];
        Node3D::CollisionData& data = *node->mCollision;
        if (type != 0 && data.type != type) continue;
        const CollisionShapeType shape = node->effectiveCollisionShape();
        if (shape == CollisionShapeType::None) continue;

        const Math::Mat4 inverse = node->getGlobalTransform().Inverse();
        const Math::Ray local(inverse.TransformPoint(ray.origin),
                inverse.TransformVector(ray.direction));

        SweepHit sweep;
        bool touched = false;
        switch (shape)
        {
            case CollisionShapeType::Mesh:
            {
                MeshStore* store = MeshStore::singleton();
                const CollisionMesh* mesh =
                        store ? store->collision(node->effectiveCollisionMesh()) : nullptr;
                touched = mesh && mesh->valid() && mesh->raycast(local, nearest, sweep);
                break;
            }
            case CollisionShapeType::Box:
                if (!data.boxMesh)
                {
                    CollisionMesh* box = new CollisionMesh();
                    if (!box->buildBox(data.boxMin, data.boxMax))
                    {
                        delete box;
                        break;
                    }
                    data.boxMesh = box;
                }
                touched = data.boxMesh->raycast(local, nearest, sweep);
                break;
            case CollisionShapeType::Ellipsoid:
            {
                const Math::Vec3 radii = data.radii;
                if (radii.x <= 0.0f || radii.y <= 0.0f || radii.z <= 0.0f) break;
                const Math::Vec3 origin(local.origin.x / radii.x, local.origin.y / radii.y,
                        local.origin.z / radii.z);
                const Math::Vec3 direction(local.direction.x / radii.x, local.direction.y / radii.y,
                        local.direction.z / radii.z);
                const float a = direction.Dot(direction);
                const float b = origin.Dot(direction);
                const float c = origin.Dot(origin) - 1.0f;
                const float discriminant = b * b - a * c;
                if (a <= 1.0e-12f || discriminant < 0.0f) break;
                const float root = sqrtf(discriminant);
                float t = (-b - root) / a;
                if (t < 0.0f) t = (-b + root) / a;
                if (t < 0.0f || t >= nearest) break;
                const Math::Vec3 unit = origin + direction * t;
                sweep.t = t;
                sweep.triangle = 0;
                sweep.normal = Math::Vec3(unit.x / radii.x, unit.y / radii.y, unit.z / radii.z);
                touched = true;
                break;
            }
            default:
                break;
        }
        if (!touched || sweep.t >= nearest) continue;

        nearest = sweep.t;
        found = true;
        hit.node = node;
        hit.time = sweep.t;
        hit.triangle = sweep.triangle;
        hit.point = ray.PointAt(sweep.t);
        const Math::Vec3 normal = Math::Mat3(inverse).Transposed() * sweep.normal;
        const float length = normal.Length();
        hit.normal = length > 1.0e-8f ? normal / length : Math::Vec3(0.0f, 1.0f, 0.0f);
    }
    return found;
}

void CollisionWorld::collectTargets(const Node3D* source, int sourceType)
{
    mTargets.clear();
    for (std::size_t i = 0; i < mNodes.size(); ++i)
    {
        Node3D* node = mNodes[i];
        if (node == source) continue;

        Node3D::CollisionData& data = *node->mCollision;
        const CollisionPair* pair = findPair(sourceType, data.type);
        if (!pair) continue;

        Target target;
        target.node = node;
        target.pair = pair;

        switch (pair->method)
        {
            case CollisionMethod::Polygon:
            {
                if (node->effectiveCollisionShape() != CollisionShapeType::Mesh) continue;
                MeshStore* store = MeshStore::singleton();
                const CollisionMesh* mesh =
                        store ? store->collision(node->effectiveCollisionMesh()) : nullptr;
                if (!mesh || !mesh->valid()) continue;
                target.mesh = mesh;
                break;
            }
            case CollisionMethod::Box:
                if (node->effectiveCollisionShape() != CollisionShapeType::Box) continue;
                if (!data.boxMesh)
                {
                    CollisionMesh* box = new CollisionMesh();
                    if (!box->buildBox(data.boxMin, data.boxMax))
                    {
                        delete box;
                        continue;
                    }
                    data.boxMesh = box;
                }
                target.mesh = data.boxMesh;
                break;
            case CollisionMethod::Sphere:
                if (node->effectiveCollisionShape() != CollisionShapeType::Ellipsoid) continue;
                target.radius = data.radii.x;
                break;
        }

        const Math::Mat4 transform = node->getGlobalTransform();
        target.origin = transform.GetTranslation();
        target.rotation = orthonormalBasis(transform);
        target.inverseRotation = target.rotation.Transposed();
        mTargets.push_back(target);
    }
}

bool CollisionWorld::findContact(const Math::Vec3& centre, const Math::Vec3& radii,
        const Math::Vec3& velocity, Contact& contact)
{
    bool found = false;
    for (std::size_t i = 0; i < mTargets.size(); ++i)
    {
        const Target& target = mTargets[i];
        ++mSweeps;

        Contact candidate;
        candidate.target = static_cast<std::uint32_t>(i);

        if (target.mesh)
        {
            if (!target.mesh->sweepEllipsoid(target.inverseRotation * (centre - target.origin),
                        radii, target.inverseRotation * velocity, candidate.hit))
                continue;
            candidate.normal = (target.rotation * candidate.hit.normal).NormalizedSafe();
        }
        else
        {
            const Math::Vec3 offset = centre - target.origin;
            if (offset.LengthSquared() < 1e-12f) continue;
            float tMax = 1.0f;
            Math::Vec3 normal;
            if (!sphereIntersectPoint(centre, radii.x + target.radius, velocity, target.origin,
                        tMax, normal))
                continue;
            candidate.hit.t = tMax;
            candidate.hit.triangle = 0;
            candidate.hit.normal = normal;
            candidate.normal = normal;
        }

        if (!found || candidate.hit.t < contact.hit.t)
        {
            contact = candidate;
            found = true;
        }
    }
    return found;
}

Math::Vec3 CollisionWorld::contactCentre(const Contact& contact, const Math::Vec3& centre,
        const Math::Vec3& velocity, const Math::Vec3& radii) const
{
    if (contact.hit.t > 0.0f) return centre + velocity * contact.hit.t;

    const Target& target = mTargets[contact.target];
    if (!target.mesh) return centre - contact.normal * contact.hit.t;

    const Math::Vec3 ellipsoidNormal = (contact.hit.normal * radii).NormalizedSafe();
    return centre - target.rotation * (ellipsoidNormal * radii) * contact.hit.t;
}

CollisionHit CollisionWorld::makeHit(const Contact& contact, const Math::Vec3& contactCentre,
        const Math::Vec3& radii) const
{
    const Target& target = mTargets[contact.target];

    CollisionHit hit;
    hit.node = target.node;
    hit.normal = contact.normal;
    hit.triangle = contact.hit.triangle;
    hit.time = contact.hit.t;

    if (target.mesh)
    {
        const Math::Vec3 ellipsoidCentre =
                (target.inverseRotation * (contactCentre - target.origin)) / radii;
        const Math::Vec3 ellipsoidNormal = (contact.hit.normal * radii).NormalizedSafe();
        const Math::Vec3 local = (ellipsoidCentre - ellipsoidNormal) * radii;
        hit.point = target.rotation * local + target.origin;
    }
    else
    {
        hit.point = target.origin + contact.normal * target.radius;
    }
    return hit;
}

void CollisionWorld::update()
{
    mSweeps = 0;
    for (std::size_t i = 0; i < mNodes.size(); ++i) updateNode(mNodes[i]);
}

void CollisionWorld::updateNode(Node3D* node)
{
    Node3D::CollisionData& data = *node->mCollision;
    data.hits.clear();

    if (node->effectiveCollisionShape() != CollisionShapeType::Ellipsoid || !isSource(data.type))
        return;

    const Math::Vec3 current = node->getGlobalPosition();
    if (!data.hasPrevious)
    {
        data.previousPosition = current;
        data.hasPrevious = true;
        return;
    }

    const Math::Vec3 displacement = current - data.previousPosition;
    if (displacement.LengthSquared() < kMinSpeedSquared)
    {
        data.previousPosition = current;
        return;
    }

    collectTargets(node, data.type);

    const Math::Vec3 radii = data.radii;
    Math::Vec3 centre = data.previousPosition;
    Math::Vec3 velocity = displacement;
    Math::Vec3 endpoint = current;

    bool resolvedOverlap = false;
    for (std::uint32_t iteration = 0; iteration < kMaxSlideIterations; ++iteration)
    {
        if (velocity.LengthSquared() < kMinSpeedSquared) break;

        Contact contact;
        if (mTargets.empty() || !findContact(centre, radii, velocity, contact))
        {
            centre = centre + velocity;
            break;
        }

        if (contact.hit.t < 0.0f)
        {
            if (resolvedOverlap) break;
            resolvedOverlap = true;
        }

        const Math::Vec3 n = contact.normal;
        const Math::Vec3 contactCentre = this->contactCentre(contact, centre, velocity, radii);
        data.hits.push_back(makeHit(contact, contactCentre, radii));

        const CollisionResponse response = mTargets[contact.target].pair->response;
        const float upDot = n.Dot(mUp);
        const bool flatten =
                response == CollisionResponse::SlideNoSlope && upDot >= mSlopeLimitCosine;

        centre = flatten ? contactCentre + mUp * (mSkinWidth / upDot)
                         : contactCentre + n * mSkinWidth;

        if (response == CollisionResponse::Stop) break;

        if (flatten)
        {
            const Math::Vec3 remaining = endpoint - centre;
            endpoint = centre + (remaining - mUp * remaining.Dot(mUp));
        }
        else
        {
            endpoint = endpoint - n * (endpoint - centre).Dot(n);
        }
        velocity = endpoint - centre;
    }

    if ((centre - current).LengthSquared() > 1e-12f) node->setGlobalPosition(centre);
    data.previousPosition = centre;
}

} // namespace prisma3d
