#include "prisma3d/scene/Road3D.h"

#include "prisma3d/scene/Curve3D.h"
#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/Path3D.h"
#include "prisma3d/scene/ResourceProperties.h"
#include "prisma3d/scene/SceneJson.h"
#include "prisma3d/scene/Terrain3D.h"
#include "prisma3d/terrain/TerrainPath.h"

#include "PathMath.h"

#include <ct/json.hpp>

#include <float.h>
#include <math.h>

namespace prisma3d
{

using namespace pathmath;

namespace
{

const char* const kMeshName = "road";
constexpr float kMinimumInterval = 0.01f;
constexpr float kMinimumWidth = 0.001f;
constexpr int kMaxSegments = 100000;
constexpr std::uint32_t kRoadAttributes =
        kVertexPosition | kVertexNormal | kVertexTangent | kVertexUv;

bool sameVec3(const Math::Vec3& a, const Math::Vec3& b)
{
    return a.x == b.x && a.y == b.y && a.z == b.z;
}

Math::Vec3 pointOf(const Math::Mat3& basis, const Math::Vec3& origin, float x, float y)
{
    return origin + basis.col0 * x + basis.col1 * y;
}

} // namespace

Road3D::Road3D()
        : VisualInstance3D(),
          mPath(nullptr),
          mWidth(4.0f),
          mProfile(),
          mPointWidths(),
          mIntervalType(IntervalType::Distance),
          mInterval(1.0f),
          mSimplifyAngle(0.0f),
          mRotationMode(RotationMode::PathFollow),
          mRotationAccurate(false),
          mTextureRepeat(4.0f),
          mDirty(true),
          mBuildCount(0),
          mGeometry(),
          mMesh(),
          mSurfaceReady(false),
          mUploadedEmpty(true),
          mVertexCapacity(0),
          mIndexCapacity(0),
          mDefaultMaterial(),
          mMaterial(),
          mGround(),
          mTerrain(nullptr),
          mTerrainPath(),
          mGroundOffset(0.06f),
          mGroundTilt(true),
          mGroundBounds{ 0.0f, 0.0f, 0.0f, 0.0f },
          mGroundBoundsValid(false)
{

    if (MaterialStore* materials = MaterialStore::singleton())
    {
        mDefaultMaterial = materials->create(kMeshName);
        materials->setFloat(mDefaultMaterial, MaterialParam::Roughness, 0.9f);
        mMaterial = mDefaultMaterial;
    }
}

Road3D::~Road3D()
{
    disconnectPreDraw();
    if (mTerrain) mTerrain->removeObserver(this);

    if (RenderingServer* server = RenderingServer::singleton())
        server->instanceSetBase(getInstance(), MeshId());
    if (MeshStore* store = MeshStore::singleton())
        if (mMesh.valid()) store->destroy(mMesh);
    if (MaterialStore* materials = MaterialStore::singleton())
        if (mDefaultMaterial.valid()) materials->destroy(mDefaultMaterial);
}

void Road3D::setWidth(float width)
{
    if (!isfinite(width)) return;
    if (width < kMinimumWidth) width = kMinimumWidth;
    if (width == mWidth) return;
    mWidth = width;
    markDirty();
}

float Road3D::getWidth() const
{
    return mWidth;
}

void Road3D::setProfile(const ct::Vector<Math::Vec2>& profile)
{
    mProfile = profile;
    markDirty();
}

void Road3D::clearProfile()
{
    mProfile.clear();
    markDirty();
}

const ct::Vector<Math::Vec2>& Road3D::getProfile() const
{
    return mProfile;
}

void Road3D::setPointWidth(int index, float width)
{
    if (index < 0 || !isfinite(width)) return;
    if (width < kMinimumWidth) width = kMinimumWidth;
    const std::size_t slot = static_cast<std::size_t>(index);
    if (slot >= mPointWidths.size()) mPointWidths.resize(slot + 1, -1.0f);
    if (mPointWidths[slot] == width) return;
    mPointWidths[slot] = width;
    markDirty();
}

float Road3D::getPointWidth(int index) const
{
    if (index < 0) return mWidth;
    const std::size_t slot = static_cast<std::size_t>(index);
    if (slot >= mPointWidths.size() || mPointWidths[slot] <= 0.0f) return mWidth;
    return mPointWidths[slot];
}

void Road3D::clearPointWidths()
{
    mPointWidths.clear();
    markDirty();
}

void Road3D::setIntervalType(IntervalType type)
{
    if (type == mIntervalType) return;
    mIntervalType = type;
    markDirty();
}

Road3D::IntervalType Road3D::getIntervalType() const
{
    return mIntervalType;
}

void Road3D::setInterval(float interval)
{
    if (!isfinite(interval)) return;
    if (interval < kMinimumInterval) interval = kMinimumInterval;
    if (interval == mInterval) return;
    mInterval = interval;
    markDirty();
}

float Road3D::getInterval() const
{
    return mInterval;
}

void Road3D::setSimplifyAngle(float degrees)
{
    if (!isfinite(degrees)) return;
    if (degrees < 0.0f) degrees = 0.0f;
    if (degrees > 180.0f) degrees = 180.0f;
    if (degrees == mSimplifyAngle) return;
    mSimplifyAngle = degrees;
    markDirty();
}

float Road3D::getSimplifyAngle() const
{
    return mSimplifyAngle;
}

void Road3D::setRotationMode(RotationMode mode)
{
    if (mode == mRotationMode) return;
    mRotationMode = mode;
    markDirty();
}

Road3D::RotationMode Road3D::getRotationMode() const
{
    return mRotationMode;
}

void Road3D::setRotationAccurate(bool accurate)
{
    if (accurate == mRotationAccurate) return;
    mRotationAccurate = accurate;
    markDirty();
}

bool Road3D::isRotationAccurate() const
{
    return mRotationAccurate;
}

void Road3D::setTextureRepeat(float metres)
{
    if (!isfinite(metres)) return;
    if (metres < 0.0f) metres = 0.0f;
    if (metres == mTextureRepeat) return;
    mTextureRepeat = metres;
    markDirty();
}

float Road3D::getTextureRepeat() const
{
    return mTextureRepeat;
}

void Road3D::setMaterial(MaterialId material)
{
    mMaterial = material.valid() ? material : mDefaultMaterial;
    applyMaterial();
}

MaterialId Road3D::getMaterial() const
{
    return mMaterial;
}

MaterialId Road3D::getAssignedMaterial() const
{
    return mMaterial == mDefaultMaterial ? MaterialId() : mMaterial;
}

void Road3D::onCurveChanged()
{
    markDirty();
}

void Road3D::setGround(const GroundFunction& ground)
{
    mGround = ground;
    markDirty();
}

void Road3D::clearGround()
{
    mGround = GroundFunction();
    markDirty();
}

bool Road3D::hasGround() const
{
    return static_cast<bool>(mGround) || mTerrain != nullptr;
}

void Road3D::setTerrain(Terrain3D* terrain)
{
    mTerrainPath = ct::String();
    if (terrain == mTerrain) return;
    if (mTerrain) mTerrain->removeObserver(this);
    mTerrain = terrain;
    if (mTerrain) mTerrain->addObserver(this);
    markDirty();
}

Terrain3D* Road3D::getTerrain() const
{
    return mTerrain;
}

void Road3D::setTerrainPath(const ct::String& path)
{
    mTerrainPath = path;
    resolveTerrain();
}

ct::String Road3D::getTerrainPath() const
{
    if (mTerrain) return getPathTo(mTerrain);
    return mTerrainPath;
}

void Road3D::resolveTerrain()
{
    Terrain3D* terrain = nullptr;
    if (!mTerrainPath.empty())
    {
        Node* node = getNode(mTerrainPath);
        terrain = node ? node->castTo<Terrain3D>() : nullptr;
    }
    if (terrain)
    {
        setTerrain(terrain);
        return;
    }
    const ct::String pending = mTerrainPath;
    setTerrain(nullptr);
    mTerrainPath = pending;
}

void Road3D::setGroundOffset(float offset)
{
    if (!isfinite(offset) || offset == mGroundOffset) return;
    mGroundOffset = offset;
    markDirty();
}

float Road3D::getGroundOffset() const
{
    return mGroundOffset;
}

void Road3D::setGroundTilt(bool tilt)
{
    if (tilt == mGroundTilt) return;
    mGroundTilt = tilt;
    markDirty();
}

bool Road3D::isGroundTilt() const
{
    return mGroundTilt;
}

void Road3D::onGroundChanged(const Math::Box& worldBox)
{
    if (!hasGround()) return;
    if (!mGroundBoundsValid)
    {
        markDirty();
        return;
    }
    if (worldBox.max.x < mGroundBounds[0] || worldBox.min.x > mGroundBounds[2] ||
            worldBox.max.z < mGroundBounds[1] || worldBox.min.z > mGroundBounds[3])
        return;
    markDirty();
}

void Road3D::onTerrainChanged(const Math::Box& worldBox)
{
    onGroundChanged(worldBox);
}

void Road3D::onTerrainDestroyed()
{
    mTerrain = nullptr;
    markDirty();
}

bool Road3D::buildFootprint(TerrainPath& out, float spacing) const
{
    out.clear();
    if (!mPath || !(spacing > 0.0f)) return false;
    const Curve3D& curve = mPath->getCurve();
    if (curve.getPointCount() < 2) return false;
    const float length = curve.getBakedLength();
    if (!(length > 1.0e-6f)) return false;

    const bool closed = curve.isClosed();
    int segments = static_cast<int>(ceilf(length / spacing));
    const int least = closed ? 3 : 1;
    if (segments < least) segments = least;
    if (segments > kMaxSegments) segments = kMaxSegments;

    float profileHalf = mWidth * 0.5f;
    if (mProfile.size() >= 2)
    {
        profileHalf = 0.0f;
        for (std::size_t i = 0; i < mProfile.size(); ++i)
        {
            const float x = fabsf(mProfile[i].x);
            profileHalf = x > profileHalf ? x : profileHalf;
        }
    }

    ct::Vector<float> pointOffsets;
    computePointOffsets(curve, pointOffsets);
    const Math::Mat4 world = getGlobalTransform();
    const int samples = closed ? segments : segments + 1;
    for (int i = 0; i < samples; ++i)
    {
        const float offset = length * static_cast<float>(i) / static_cast<float>(segments);
        const float scale = widthScaleAt(curve, offset, pointOffsets);
        out.add(world.TransformPoint(curve.sampleBaked(offset)), profileHalf * scale);
    }
    out.closed = closed;
    return true;
}

bool Road3D::flush()
{
    if (!mDirty) return false;
    rebuild();
    return true;
}

void Road3D::rebuild()
{
    mDirty = false;
    ++mBuildCount;

    mGroundBoundsValid = hasGround();
    mGroundBounds[0] = FLT_MAX;
    mGroundBounds[1] = FLT_MAX;
    mGroundBounds[2] = -FLT_MAX;
    mGroundBounds[3] = -FLT_MAX;

    ct::Vector<Ring> rings;
    if (mPath) buildRings(mPath->getCurve(), rings);
    buildGeometry(rings);
    uploadGeometry();
}

bool Road3D::isDirty() const
{
    return mDirty;
}

int Road3D::getBuildCount() const
{
    return mBuildCount;
}

const SurfaceData& Road3D::getGeometry() const
{
    return mGeometry;
}

bool Road3D::buildCollision(CollisionMesh& out) const
{
    out.clear();
    if (mGeometry.positions.empty() || mGeometry.indices.empty()) return false;
    if (!out.append(mGeometry)) return false;
    return out.finish();
}

Math::Box Road3D::getAabb() const
{
    return MeshStore::computeAabb(mGeometry.positions);
}

void Road3D::notificationHandler(int what)
{
    VisualInstance3D::notificationHandler(what);

    switch (what)
    {
        case kNotificationEnterTree:
        {
            Node* parent = getParent();
            mPath = parent ? parent->castTo<Path3D>() : nullptr;
            if (!mTerrainPath.empty()) resolveTerrain();
            markDirty();
            connectPreDraw();
            break;
        }
        case kNotificationExitTree:
            mPath = nullptr;
            disconnectPreDraw();
            break;
        case kNotificationTransformChanged:
            if (hasGround()) markDirty();
            break;
        default:
            break;
    }
}

int Road3D::countSegments(const Curve3D& curve, bool closed) const
{
    const float length = curve.getBakedLength();
    int extrusions = 0;
    if (mIntervalType == IntervalType::Distance)
    {
        const float count = ceilf(length / mInterval);
        extrusions = (count > 1.0f ? static_cast<int>(count > 1.0e6f ? 1.0e6f : count) : 1) + 1;
    }
    else
    {
        extrusions = static_cast<int>(ceilf(static_cast<float>(curve.getPointCount()) / mInterval));
    }

    int segments = extrusions - 1;
    const int least = closed ? 3 : 1;
    if (segments < least) segments = least;
    if (segments > kMaxSegments) segments = kMaxSegments;
    return segments;
}

void Road3D::computePointOffsets(const Curve3D& curve, ct::Vector<float>& out) const
{
    const int count = curve.getPointCount();
    const bool closed = curve.isClosed();
    const ct::Vector<Math::Vec3>& baked = curve.getBakedPoints();
    const ct::Vector<float>& distances = curve.getBakedDistCache();

    out.clear();
    out.resize(static_cast<std::size_t>(closed ? count + 1 : count), 0.0f);

    std::size_t previous = 0;
    for (int i = 1; i < count; ++i)
    {
        const Math::Vec3 position = curve.getPointPosition(i);
        float found = out[static_cast<std::size_t>(i - 1)];
        for (std::size_t j = previous + 1; j < baked.size() && j < distances.size(); ++j)
        {
            if (!sameVec3(baked[j], position)) continue;
            found = distances[j];
            previous = j;
            break;
        }
        out[static_cast<std::size_t>(i)] = found;
    }
    if (closed) out[static_cast<std::size_t>(count)] = curve.getBakedLength();
}

float Road3D::widthScaleAt(const Curve3D& curve, float offset,
        const ct::Vector<float>& pointOffsets) const
{
    const int count = curve.getPointCount();
    if (pointOffsets.size() < 2 || count < 2) return 1.0f;

    std::size_t segment = 0;
    while (segment + 2 < pointOffsets.size() && pointOffsets[segment + 1] <= offset) ++segment;

    const float begin = pointOffsets[segment];
    const float end = pointOffsets[segment + 1];
    float t = 0.0f;
    if (end - begin > 1.0e-6f) t = clamp((offset - begin) / (end - begin), 0.0f, 1.0f);

    const int first = static_cast<int>(segment) % count;
    const int second = static_cast<int>(segment + 1) % count;
    const float width = lerp(getPointWidth(first), getPointWidth(second), t);
    return width / mWidth;
}

Road3D::Ring Road3D::makeRing(const Curve3D& curve, float offset, float step, float length,
        const ct::Vector<float>& pointOffsets) const
{
    Ring ring;
    ring.distance = offset;
    ring.scale = widthScaleAt(curve, offset, pointOffsets);

    Math::Vec3 point(0.0f, 0.0f, 0.0f);
    Math::Vec3 direction(0.0f, 0.0f, -1.0f);
    Math::Vec3 up(0.0f, 1.0f, 0.0f);

    if (mRotationMode == RotationMode::Fixed)
    {
        point = curve.sampleBaked(offset);
    }
    else
    {
        if (!mRotationAccurate)
        {
            point = curve.sampleBaked(offset);
            const float nextOffset = offset + step < length ? offset + step : length;
            const float lastOffset = offset - step > 0.0f ? offset - step : 0.0f;
            direction = curve.sampleBaked(nextOffset) - curve.sampleBaked(lastOffset);
        }
        else
        {
            const Math::Mat4 sample = curve.sampleBakedWithRotation(offset);
            point = sample.GetTranslation();
            direction = -sample.col2.xyz();
        }

        if (mRotationMode == RotationMode::PathFollow && curve.isUpVectorEnabled())
            up = curve.sampleBakedUpVector(offset, true);
    }

    if (direction.Dot(direction) < 1.0e-12f) direction = Math::Vec3(0.0f, 0.0f, -1.0f);

    ring.basis = lookingAt(direction, up);
    ring.origin = point;
    return ring;
}

void Road3D::buildRings(const Curve3D& curve, ct::Vector<Ring>& rings) const
{
    rings.clear();
    if (curve.getPointCount() < 2) return;

    const float length = curve.getBakedLength();
    if (!(length > 1.0e-6f)) return;

    const bool closed = curve.isClosed();
    const int segments = countSegments(curve, closed);
    const float step = length / static_cast<float>(segments);

    ct::Vector<float> pointOffsets;
    computePointOffsets(curve, pointOffsets);

    rings.push_back(makeRing(curve, 0.0f, step, length, pointOffsets));

    const float angleDot = cosf(mSimplifyAngle * (kPi / 180.0f));
    Math::Vec3 previousDirection(0.0f, 0.0f, 0.0f);

    for (int x0 = 0; x0 < segments; ++x0)
    {
        const bool closing = closed && x0 == segments - 1;
        float previousOffset = static_cast<float>(x0) * step;
        float currentOffset = static_cast<float>(x0 + 1) * step;
        if (previousOffset > length) previousOffset = length;
        if (currentOffset > length) currentOffset = length;

        const Math::Vec3 previousPoint = curve.sampleBaked(previousOffset);
        const Math::Vec3 currentPoint = curve.sampleBaked(closing ? 0.0f : currentOffset);
        const Math::Vec3 direction = normalized(currentPoint - previousPoint);

        if (mSimplifyAngle > 0.0f && x0 > 0 && rings.size() > 1 &&
                previousDirection.Dot(direction) > angleDot)
            rings.pop_back();
        else
            previousDirection = direction;

        if (closing)
        {
            Ring joined = rings[0];
            joined.distance = length;
            rings.push_back(joined);
        }
        else
        {
            rings.push_back(makeRing(curve, currentOffset, step, length, pointOffsets));
        }
    }
}

void Road3D::buildGeometry(const ct::Vector<Ring>& rings)
{
    mGeometry.topology = prisma::Topology::Triangles;
    mGeometry.positions.clear();
    mGeometry.normals.clear();
    mGeometry.tangents.clear();
    mGeometry.colors.clear();
    mGeometry.uvs.clear();
    mGeometry.uvs2.clear();
    mGeometry.indices.clear();

    if (rings.size() < 2) return;

    ct::Vector<Math::Vec2> profile;
    if (mProfile.size() >= 2)
    {
        profile = mProfile;
    }
    else
    {
        profile.push_back(Math::Vec2(-mWidth * 0.5f, 0.0f));
        profile.push_back(Math::Vec2(mWidth * 0.5f, 0.0f));
    }

    const std::size_t sides = profile.size() - 1;
    ct::Vector<float> across;
    across.resize(profile.size(), 0.0f);
    for (std::size_t i = 1; i < profile.size(); ++i)
    {
        const float dx = profile[i].x - profile[i - 1].x;
        const float dy = profile[i].y - profile[i - 1].y;
        across[i] = across[i - 1] + sqrtf(dx * dx + dy * dy);
    }
    const float acrossTotal = across[across.size() - 1];
    for (std::size_t i = 0; i < across.size(); ++i)
    {
        if (acrossTotal > 1.0e-6f) across[i] /= acrossTotal;
        else
            across[i] = static_cast<float>(i) / static_cast<float>(sides);
    }

    const float totalDistance = rings[rings.size() - 1].distance;
    const std::size_t verticesPerRing = sides * 2;
    mGeometry.positions.reserve(rings.size() * verticesPerRing);
    mGeometry.normals.reserve(rings.size() * verticesPerRing);
    mGeometry.uvs.reserve(rings.size() * verticesPerRing);

    for (std::size_t r = 0; r < rings.size(); ++r)
    {
        const Ring& ring = rings[r];
        float v = 0.0f;
        if (mTextureRepeat > 0.0f) v = ring.distance / mTextureRepeat;
        else if (totalDistance > 1.0e-6f)
            v = ring.distance / totalDistance;

        for (std::size_t s = 0; s < sides; ++s)
        {
            const float ax = profile[s].x * ring.scale;
            const float ay = profile[s].y;
            const float bx = profile[s + 1].x * ring.scale;
            const float by = profile[s + 1].y;

            float nx = -(by - ay);
            float ny = bx - ax;
            const float nl = sqrtf(nx * nx + ny * ny);
            if (nl > 1.0e-9f)
            {
                nx /= nl;
                ny /= nl;
            }
            else
            {
                nx = 0.0f;
                ny = 1.0f;
            }
            const Math::Vec3 normal = (ring.basis.col0 * nx + ring.basis.col1 * ny).Normalized();

            mGeometry.positions.push_back(pointOf(ring.basis, ring.origin, ax, ay));
            mGeometry.normals.push_back(normal);
            mGeometry.uvs.push_back(Math::Vec2(across[s], v));
            mGeometry.positions.push_back(pointOf(ring.basis, ring.origin, bx, by));
            mGeometry.normals.push_back(normal);
            mGeometry.uvs.push_back(Math::Vec2(across[s + 1], v));
        }
    }

    mGeometry.indices.reserve((rings.size() - 1) * sides * 6);
    for (std::size_t r = 0; r + 1 < rings.size(); ++r)
    {
        for (std::size_t s = 0; s < sides; ++s)
        {
            const std::uint32_t previousLeft =
                    static_cast<std::uint32_t>(r * verticesPerRing + s * 2);
            const std::uint32_t previousRight = previousLeft + 1;
            const std::uint32_t currentLeft =
                    previousLeft + static_cast<std::uint32_t>(verticesPerRing);
            const std::uint32_t currentRight = currentLeft + 1;
            mGeometry.indices.push_back(previousLeft);
            mGeometry.indices.push_back(currentRight);
            mGeometry.indices.push_back(currentLeft);
            mGeometry.indices.push_back(previousLeft);
            mGeometry.indices.push_back(previousRight);
            mGeometry.indices.push_back(currentRight);
        }
    }

    applyGround(profile, rings, sides);

    if (!MeshStore::generateTangents(mGeometry))
    {
        mGeometry.tangents.clear();
        mGeometry.tangents.resize(mGeometry.positions.size(), Math::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
    }
}

void Road3D::applyGround(const ct::Vector<Math::Vec2>& profile, const ct::Vector<Ring>& rings,
        std::size_t sides)
{
    const bool terrain = mTerrain && mTerrain->isLoaded();
    if (!mGround && !terrain) return;

    const Math::Mat4 world = getGlobalTransform();
    const Math::Mat4 inverse = world.InverseAffine();
    const std::size_t verticesPerRing = sides * 2;
    ct::Vector<float> heights;
    ct::Vector<Math::Vec3> flat;
    heights.resize(profile.size(), 0.0f);
    flat.resize(profile.size(), Math::Vec3(0.0f, 0.0f, 0.0f));

    for (std::size_t r = 0; r < rings.size(); ++r)
    {
        const Ring& ring = rings[r];
        float level = -FLT_MAX;
        for (std::size_t k = 0; k < profile.size(); ++k)
        {
            flat[k] = world.TransformPoint(
                    pointOf(ring.basis, ring.origin, profile[k].x * ring.scale, 0.0f));
            heights[k] = mGround ? mGround(flat[k]) : mTerrain->getWorldHeightAt(flat[k]);
            level = heights[k] > level ? heights[k] : level;
        }

        for (std::size_t s = 0; s < sides; ++s)
        {
            for (std::size_t e = 0; e < 2; ++e)
            {
                const std::size_t k = s + e;
                const std::size_t index = r * verticesPerRing + s * 2 + e;
                const Math::Vec3 full = world.TransformPoint(mGeometry.positions[index]);
                const float relief = full.y - flat[k].y;
                const float base = mGroundTilt ? heights[k] : level;
                const Math::Vec3 placed(full.x, base + mGroundOffset + relief, full.z);
                mGeometry.positions[index] = inverse.TransformPoint(placed);
                mGroundBounds[0] = placed.x < mGroundBounds[0] ? placed.x : mGroundBounds[0];
                mGroundBounds[1] = placed.z < mGroundBounds[1] ? placed.z : mGroundBounds[1];
                mGroundBounds[2] = placed.x > mGroundBounds[2] ? placed.x : mGroundBounds[2];
                mGroundBounds[3] = placed.z > mGroundBounds[3] ? placed.z : mGroundBounds[3];
            }
        }
    }

    const bool closed =
            rings.size() > 2 && sameVec3(rings[0].origin, rings[rings.size() - 1].origin);
    for (std::size_t r = 0; r < rings.size(); ++r)
    {
        std::size_t next = r + 1 < rings.size() ? r + 1 : (closed ? 1 : r);
        std::size_t previous = r > 0 ? r - 1 : (closed ? rings.size() - 2 : r);
        for (std::size_t s = 0; s < sides; ++s)
        {
            const std::size_t at = r * verticesPerRing + s * 2;
            const Math::Vec3 across = mGeometry.positions[at + 1] - mGeometry.positions[at];
            for (std::size_t e = 0; e < 2; ++e)
            {
                const Math::Vec3 along =
                        mGeometry.positions[next * verticesPerRing + s * 2 + e] -
                        mGeometry.positions[previous * verticesPerRing + s * 2 + e];
                const Math::Vec3 normal = across.Cross(along);
                if (normal.Dot(normal) > 1.0e-18f) mGeometry.normals[at + e] = normal.Normalized();
            }
        }
    }
}

void Road3D::uploadGeometry()
{
    MeshStore* store = MeshStore::singleton();
    if (!store) return;

    if (!store->valid(mMesh))
    {
        mMesh = store->create(kMeshName);
        mSurfaceReady = false;
        setBase(mMesh);
    }

    const std::size_t vertexCount = mGeometry.positions.size();
    const std::size_t indexCount = mGeometry.indices.size();

    if (mSurfaceReady && (vertexCount > mVertexCapacity || indexCount > mIndexCapacity))
        invalidateSurface();

    if (!mSurfaceReady && vertexCount > 0)
    {
        mVertexCapacity = static_cast<std::uint32_t>(vertexCount + vertexCount / 2 + 64);
        mIndexCapacity = static_cast<std::uint32_t>(indexCount + indexCount / 2 + 96);
        SurfaceLayout layout;
        layout.topology = prisma::Topology::Triangles;
        layout.attributeMask = kRoadAttributes;
        layout.vertexCount = mVertexCapacity;
        layout.indexCount = mIndexCapacity;
        layout.indexFormat = prisma::IndexFormat::UInt32;
        mSurfaceReady = store->addDynamicSurface(mMesh, layout, kMeshName) >= 0;
        mUploadedEmpty = true;
        applyMaterial();
    }

    const bool empty = vertexCount == 0;
    if (mSurfaceReady && !(empty && mUploadedEmpty))
    {
        store->updateSurface(mMesh, 0, mGeometry);
        mUploadedEmpty = empty;
    }
}

void Road3D::invalidateSurface()
{
    MeshStore* store = MeshStore::singleton();
    if (store && store->valid(mMesh)) store->clearSurfaces(mMesh);
    mSurfaceReady = false;
}

void Road3D::applyMaterial()
{
    MeshStore* store = MeshStore::singleton();
    if (!store || !mSurfaceReady || !store->valid(mMesh) || !mMaterial.valid()) return;
    store->setSurfaceMaterial(mMesh, 0, mMaterial);
}

void Road3D::markDirty()
{
    mDirty = true;
}

void Road3D::connectPreDraw()
{
    RenderingServer* server = RenderingServer::singleton();
    if (!server) return;
    server->framePreDrawConnect(this, [this]() { flush(); });
}

void Road3D::disconnectPreDraw()
{
    RenderingServer* server = RenderingServer::singleton();
    if (server) server->framePreDrawDisconnect(this);
}

void Road3D::bindProperties()
{
    using DB = NodeClassDB;
    using P = Road3D;
    resourceprop::add<P, MaterialId>(PropertyInfo::resource("material", "Material"),
            &P::setMaterial, &P::getAssignedMaterial);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "width", 0.001, 1000.0, 0.001, true),
            &P::setWidth, &P::getWidth);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "texture_repeat", 0.0, 1000.0, 0.01, true),
            &P::setTextureRepeat, &P::getTextureRepeat);

    DB::addPropertyGroup<P>("Sampling", "");
    DB::addProperty<P>(PropertyInfo::enumeration("interval_type", "Distance,Subdivide"),
            &P::setIntervalType, &P::getIntervalType);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "interval", 0.01, 1000.0, 0.01, true),
            &P::setInterval, &P::getInterval);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "simplify_angle", 0.0, 180.0, 0.1),
            &P::setSimplifyAngle, &P::getSimplifyAngle);
    DB::addProperty<P>(PropertyInfo::enumeration("rotation_mode", "Fixed,Path,Path Follow"),
            &P::setRotationMode, &P::getRotationMode);
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "rotation_accurate"),
            &P::setRotationAccurate, &P::isRotationAccurate);

    DB::addPropertyGroup<P>("Ground", "ground_");
    DB::addProperty<P>(PropertyInfo::nodePath("terrain", "Terrain3D"), &P::setTerrainPath,
            &P::getTerrainPath);
    DB::addProperty<P>(PropertyInfo(PropertyType::Float, "ground_offset"), &P::setGroundOffset,
            &P::getGroundOffset);
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "ground_tilt"), &P::setGroundTilt,
            &P::isGroundTilt);
}

namespace
{

constexpr std::int64_t kRoad3DDataVersion = 1;

}

bool Road3D::saveData(ct::Json& data, SceneResult& result, const ct::String& path) const
{
    (void) result;
    (void) path;
    if (mProfile.empty() && mPointWidths.empty()) return false;

    data["version"] = ct::Json(static_cast<long long>(kRoad3DDataVersion));
    if (!mProfile.empty())
    {
        ct::Json profile = ct::Json::array();
        for (std::size_t i = 0; i < mProfile.size(); ++i)
            profile.push_back(scenejson::fromVec2(mProfile[i]));
        data["profile"] = profile;
    }
    if (!mPointWidths.empty())
        data["point_widths"] = scenejson::fromFloats(mPointWidths.data(), mPointWidths.size());
    return true;
}

bool Road3D::loadData(const ct::Json& data, SceneResult& result, const ct::String& path)
{
    if (!scenejson::checkVersion(data, kRoad3DDataVersion, result, path)) return false;

    ct::Vector<Math::Vec2> profile;
    if (const ct::Json* field = scenejson::arrayField(data, "profile", result, path))
    {
        for (std::size_t i = 0; i < field->size(); ++i)
        {
            Math::Vec2 point;
            if (!scenejson::toVec2(field->at(i), point))
            {
                result.addError(scenejson::element(path, "profile", i), "expected [x, y]");
                continue;
            }
            profile.push_back(point);
        }
    }
    setProfile(profile);

    clearPointWidths();
    if (const ct::Json* field = scenejson::arrayField(data, "point_widths", result, path))
    {
        for (std::size_t i = 0; i < field->size(); ++i)
        {
            float width = 0.0f;
            if (!scenejson::toFloat(field->at(i), width))
            {
                result.addError(scenejson::element(path, "point_widths", i), "expected a number");
                continue;
            }
            if (width > 0.0f) setPointWidth(static_cast<int>(i), width);
        }
    }
    return true;
}

} // namespace prisma3d
