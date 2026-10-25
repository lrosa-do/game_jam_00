#include "prisma3d/scene/SoftBody3D.h"

#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/PrimitiveMeshes.h"
#include "prisma3d/scene/SceneJson.h"
#include "prisma3d/scene/SceneResources.h"

#include <ct/hashmap.hpp>
#include <ct/json.hpp>

#include <math.h>

namespace prisma3d
{

namespace
{

const char* const kMeshName = "SoftBody";
const std::uint32_t kBitsPerAxis = 21;
const std::uint64_t kAxisMask = (1ull << kBitsPerAxis) - 1ull;

std::uint64_t cellKey(const Math::Vec3& point, const Math::Vec3& origin, float inverseCell)
{
    const Math::Vec3 offset = point - origin;
    const std::uint64_t x = static_cast<std::uint64_t>(floorf(offset.x * inverseCell + 0.5f));
    const std::uint64_t y = static_cast<std::uint64_t>(floorf(offset.y * inverseCell + 0.5f));
    const std::uint64_t z = static_cast<std::uint64_t>(floorf(offset.z * inverseCell + 0.5f));
    return (x & kAxisMask) | ((y & kAxisMask) << kBitsPerAxis) |
           ((z & kAxisMask) << (2 * kBitsPerAxis));
}

} // namespace

SoftBody3D::SoftBody3D()
        : MeshInstance3D(),
          mAttachmentLimit(1.0f),
          mWind(0.0f, 0.0f, 0.0f),
          mPinFrame(Math::Mat4::Identity()),
          mVertexCapacity(0),
          mIndexCapacity(0),
          mDirty(true),
          mEnabled(true),
          mSurfaceReady(false),
          mPinFrameValid(false)
{
    setProcess(true);
    setPhysicsProcess(true);
}

SoftBody3D::~SoftBody3D()
{
    release();
    setMesh(MeshId());
    MeshStore* store = MeshStore::singleton();
    if (store && mOwned.valid()) store->destroy(mOwned);
}

void SoftBody3D::markDirty()
{
    mDirty = true;
}

void SoftBody3D::setGeometry(const SurfaceData& geometry)
{
    mGeometry = geometry;
    mPinFlags.assign(geometry.positions.size(), 0);
    markDirty();
}

const SurfaceData& SoftBody3D::getGeometry() const
{
    return mGeometry;
}

void SoftBody3D::makeCloth(float width, float depth, int segmentsX, int segmentsZ, float uvTiles)
{
    SurfaceData data;
    planeMeshArrays(data, width, depth, segmentsX, segmentsZ, uvTiles);
    setGeometry(data);
}

void SoftBody3D::makeBall(float radius, int rings, int slices)
{
    SurfaceData data;
    sphereMeshArrays(data, radius, rings, slices);
    setGeometry(data);
}

void SoftBody3D::setParameters(const PhysicsSoftBodyParameters& parameters)
{
    mParameters = parameters;
    markDirty();
}

const PhysicsSoftBodyParameters& SoftBody3D::getParameters() const
{
    return mParameters;
}

void SoftBody3D::setAttachmentLimit(float multiplier)
{
    mAttachmentLimit = multiplier;
    markDirty();
}

float SoftBody3D::getAttachmentLimit() const
{
    return mAttachmentLimit;
}

void SoftBody3D::setPinned(std::size_t vertex, bool pinned)
{
    if (vertex >= mPinFlags.size()) return;
    mPinFlags[vertex] = pinned ? 1 : 0;
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (!mDirty && server && server->softBodyValid(mBody) && vertex < mVertexParticle.size())
    {
        const std::uint32_t particle = mVertexParticle[vertex];
        mParticlePinned[particle] = pinned ? 1 : 0;
        server->softBodySetPinned(mBody, particle, pinned);
        mPinFrameValid = false;
    }
}

bool SoftBody3D::isPinned(std::size_t vertex) const
{
    return vertex < mPinFlags.size() && mPinFlags[vertex] != 0;
}

void SoftBody3D::pinInSphere(const Math::Vec3& localCenter, float radius, bool pinned)
{
    const float limit = radius * radius;
    for (std::size_t i = 0; i < mGeometry.positions.size(); ++i)
    {
        const Math::Vec3 offset = mGeometry.positions[i] - localCenter;
        if (offset.Dot(offset) <= limit) setPinned(i, pinned);
    }
}

void SoftBody3D::pinInBox(const Math::Box& localBox, bool pinned)
{
    for (std::size_t i = 0; i < mGeometry.positions.size(); ++i)
        if (localBox.Contains(mGeometry.positions[i])) setPinned(i, pinned);
}

void SoftBody3D::clearPins()
{
    for (std::size_t i = 0; i < mPinFlags.size(); ++i) setPinned(i, false);
}

void SoftBody3D::setWind(const Math::Vec3& wind)
{
    mWind = wind;
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (server && mBody.valid()) server->softBodySetWind(mBody, wind);
}

const Math::Vec3& SoftBody3D::getWind() const
{
    return mWind;
}

void SoftBody3D::addVelocity(const Math::Vec3& delta)
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (server && mBody.valid()) server->softBodyAddVelocity(mBody, delta);
}

void SoftBody3D::setSimulationEnabled(bool enabled)
{
    mEnabled = enabled;
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (server && mBody.valid()) server->softBodySetEnabled(mBody, enabled);
}

bool SoftBody3D::isSimulationEnabled() const
{
    return mEnabled;
}

void SoftBody3D::reset()
{
    markDirty();
}

bool SoftBody3D::isBuilt() const
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    return server && server->softBodyValid(mBody) && !mDirty;
}

std::size_t SoftBody3D::getParticleCount() const
{
    return mParticleVertex.size();
}

Math::Vec3 SoftBody3D::getParticlePosition(std::size_t vertex) const
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (!server || !server->softBodyValid(mBody) || vertex >= mVertexParticle.size())
        return Math::Vec3(0.0f, 0.0f, 0.0f);
    Math::Vec3 position(0.0f, 0.0f, 0.0f);
    const std::size_t particle = mVertexParticle[vertex];
    ct::Vector<Math::Vec3> all;
    all.resize(particle + 1);
    server->softBodyGetPositions(mBody, all.data(), all.size());
    position = all[particle];
    return position;
}

float SoftBody3D::getWorstStretch() const
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    return server ? server->softBodyGetWorstStretch(mBody) : 0.0f;
}

PhysicsSoftBodyId SoftBody3D::getSoftBody() const
{
    return mBody;
}

MeshId SoftBody3D::getOwnedMesh() const
{
    return mOwned;
}

void SoftBody3D::release()
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (server && mBody.valid()) server->softBodyFree(mBody);
    mBody = PhysicsSoftBodyId();
}

bool SoftBody3D::build()
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    release();
    mVertexParticle.clear();
    mParticleVertex.clear();
    mParticlePinned.clear();
    mDirty = false;
    mPinFrameValid = false;
    if (!server) return false;

    const std::size_t vertexCount = mGeometry.positions.size();
    if (vertexCount < 3 || mGeometry.indices.size() < 3) return false;

    Math::Vec3 low = mGeometry.positions[0];
    Math::Vec3 high = low;
    for (std::size_t i = 1; i < vertexCount; ++i)
    {
        const Math::Vec3& p = mGeometry.positions[i];
        if (p.x < low.x) low.x = p.x;
        if (p.y < low.y) low.y = p.y;
        if (p.z < low.z) low.z = p.z;
        if (p.x > high.x) high.x = p.x;
        if (p.y > high.y) high.y = p.y;
        if (p.z > high.z) high.z = p.z;
    }
    const Math::Vec3 span = high - low;
    float extent = span.x > span.y ? span.x : span.y;
    if (span.z > extent) extent = span.z;
    float cell = extent / static_cast<float>(kAxisMask - 1);
    if (cell < 1.0e-5f) cell = 1.0e-5f;

    ct::HashMap<std::uint64_t, std::uint32_t> welded;
    ct::Vector<Math::Vec3> rest;
    mVertexParticle.resize(vertexCount);
    for (std::size_t i = 0; i < vertexCount; ++i)
    {
        const std::uint64_t key = cellKey(mGeometry.positions[i], low, 1.0f / cell);
        if (const std::uint32_t* found = welded.find(key))
        {
            mVertexParticle[i] = *found;
            continue;
        }
        const std::uint32_t particle = static_cast<std::uint32_t>(rest.size());
        welded[key] = particle;
        mVertexParticle[i] = particle;
        mParticleVertex.push_back(static_cast<std::uint32_t>(i));
        rest.push_back(mGeometry.positions[i]);
    }

    ct::Vector<std::uint32_t> triangles;
    triangles.reserve(mGeometry.indices.size());
    for (std::size_t i = 0; i + 2 < mGeometry.indices.size(); i += 3)
    {
        const std::uint32_t a = mGeometry.indices[i];
        const std::uint32_t b = mGeometry.indices[i + 1];
        const std::uint32_t c = mGeometry.indices[i + 2];
        if (a >= vertexCount || b >= vertexCount || c >= vertexCount) return false;
        const std::uint32_t pa = mVertexParticle[a];
        const std::uint32_t pb = mVertexParticle[b];
        const std::uint32_t pc = mVertexParticle[c];
        if (pa == pb || pb == pc || pa == pc) continue;
        triangles.push_back(pa);
        triangles.push_back(pb);
        triangles.push_back(pc);
    }
    if (triangles.size() < 3) return false;

    const Math::Mat4 frame = getGlobalTransform();
    ct::Vector<Math::Vec3> world;
    world.resize(rest.size());
    for (std::size_t i = 0; i < rest.size(); ++i) world[i] = frame.TransformPoint(rest[i]);

    mBody = server->softBodyCreate(world.data(), world.size(), triangles.data(), triangles.size(),
            mParameters);
    if (!mBody.valid()) return false;

    mParticlePinned.assign(rest.size(), 0);
    for (std::size_t i = 0; i < vertexCount && i < mPinFlags.size(); ++i)
        if (mPinFlags[i]) mParticlePinned[mVertexParticle[i]] = 1;
    bool anyPinned = false;
    for (std::size_t i = 0; i < mParticlePinned.size(); ++i)
        if (mParticlePinned[i])
        {
            server->softBodySetPinned(mBody, i, true);
            anyPinned = true;
        }
    if (anyPinned && mAttachmentLimit > 0.0f)
        server->softBodyBuildAttachments(mBody, mAttachmentLimit);

    server->softBodySetWind(mBody, mWind);
    server->softBodySetEnabled(mBody, mEnabled);
    mPositions.resize(rest.size());
    return true;
}

void SoftBody3D::followPins()
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (!server || !server->softBodyValid(mBody)) return;
    const Math::Mat4 frame = getGlobalTransform();
    if (mPinFrameValid && frame == mPinFrame) return;
    mPinFrame = frame;
    mPinFrameValid = true;
    for (std::size_t i = 0; i < mParticlePinned.size(); ++i)
    {
        if (!mParticlePinned[i]) continue;
        const Math::Vec3 local = mGeometry.positions[mParticleVertex[i]];
        server->softBodySetParticlePosition(mBody, i, frame.TransformPoint(local));
    }
}

void SoftBody3D::upload()
{
    MeshStore* store = MeshStore::singleton();
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (!store || !store->driver() || !server || !server->softBodyValid(mBody)) return;

    const std::size_t particleCount = mParticleVertex.size();
    const std::size_t vertexCount = mGeometry.positions.size();
    mPositions.resize(particleCount);
    server->softBodyGetPositions(mBody, mPositions.data(), particleCount);

    mNormals.assign(particleCount, Math::Vec3(0.0f, 0.0f, 0.0f));
    const bool hasUvs = mGeometry.uvs.size() == vertexCount;
    const bool hasTangents = hasUvs && mGeometry.tangents.size() == vertexCount;
    if (hasTangents)
    {
        mTangentSums.assign(vertexCount, Math::Vec3(0.0f, 0.0f, 0.0f));
        mBitangentSums.assign(vertexCount, Math::Vec3(0.0f, 0.0f, 0.0f));
    }

    for (std::size_t i = 0; i + 2 < mGeometry.indices.size(); i += 3)
    {
        const std::uint32_t v[3] = { mGeometry.indices[i], mGeometry.indices[i + 1],
            mGeometry.indices[i + 2] };
        if (v[0] >= vertexCount || v[1] >= vertexCount || v[2] >= vertexCount) continue;
        const std::uint32_t p[3] = { mVertexParticle[v[0]], mVertexParticle[v[1]],
            mVertexParticle[v[2]] };
        const Math::Vec3 e1 = mPositions[p[1]] - mPositions[p[0]];
        const Math::Vec3 e2 = mPositions[p[2]] - mPositions[p[0]];
        const Math::Vec3 face = e1.Cross(e2);
        for (int k = 0; k < 3; ++k) mNormals[p[k]] = mNormals[p[k]] + face;
        if (!hasTangents) continue;
        const Math::Vec2 u1 = mGeometry.uvs[v[1]] - mGeometry.uvs[v[0]];
        const Math::Vec2 u2 = mGeometry.uvs[v[2]] - mGeometry.uvs[v[0]];
        const float determinant = u1.x * u2.y - u2.x * u1.y;
        if (fabsf(determinant) < 1.0e-12f) continue;
        const float scale = 1.0f / determinant;
        const Math::Vec3 tangent = (e1 * u2.y - e2 * u1.y) * scale;
        const Math::Vec3 bitangent = (e2 * u1.x - e1 * u2.x) * scale;
        for (int k = 0; k < 3; ++k)
        {
            mTangentSums[v[k]] = mTangentSums[v[k]] + tangent;
            mBitangentSums[v[k]] = mBitangentSums[v[k]] + bitangent;
        }
    }

    const Math::Mat4 global = getGlobalTransform();
    const Math::Mat4 toLocal = global.InverseAffine();
    const Math::Mat4 normalMatrix = global.Transposed();

    mUpload.topology = mGeometry.topology;
    mUpload.positions.resize(vertexCount);
    mUpload.normals.resize(vertexCount);
    if (hasTangents) mUpload.tangents.resize(vertexCount);
    for (std::size_t i = 0; i < vertexCount; ++i)
    {
        const std::uint32_t particle = mVertexParticle[i];
        mUpload.positions[i] = toLocal.TransformPoint(mPositions[particle]);
        Math::Vec3 normal = mNormals[particle];
        const float length = normal.Length();
        normal = length > 1.0e-12f ? normal * (1.0f / length) : Math::Vec3(0.0f, 1.0f, 0.0f);
        const Math::Vec3 localNormal = normalMatrix.TransformVector(normal);
        mUpload.normals[i] = localNormal.Normalized();
        if (hasTangents)
        {
            Math::Vec3 tangent = mTangentSums[i];
            tangent = tangent - normal * normal.Dot(tangent);
            const float tangentLength = tangent.Length();
            tangent = tangentLength > 1.0e-12f ? tangent * (1.0f / tangentLength)
                                               : mGeometry.tangents[i].xyz();
            const float handedness =
                    normal.Cross(tangent).Dot(mBitangentSums[i]) < 0.0f ? -1.0f : 1.0f;
            const Math::Vec3 localTangent = toLocal.TransformVector(tangent).Normalized();
            mUpload.tangents[i] =
                    Math::Vec4(localTangent.x, localTangent.y, localTangent.z, handedness);
        }
    }
    mUpload.colors = mGeometry.colors;
    mUpload.uvs = mGeometry.uvs;
    mUpload.uvs2 = mGeometry.uvs2;
    mUpload.indices = mGeometry.indices;

    if (!mOwned.valid() || !store->valid(mOwned))
    {
        mOwned = store->create(kMeshName);
        mSurfaceReady = false;
        setMesh(mOwned);
    }

    if (mSurfaceReady &&
            (vertexCount > mVertexCapacity || mGeometry.indices.size() > mIndexCapacity))
    {
        store->clearSurfaces(mOwned);
        mSurfaceReady = false;
    }

    if (!mSurfaceReady)
    {
        mVertexCapacity = static_cast<std::uint32_t>(vertexCount);
        mIndexCapacity = static_cast<std::uint32_t>(mGeometry.indices.size());
        SurfaceLayout layout;
        layout.topology = mUpload.topology;
        layout.attributeMask = kVertexPosition | kVertexNormal;
        if (hasTangents) layout.attributeMask |= kVertexTangent;
        if (!mUpload.colors.empty()) layout.attributeMask |= kVertexColor;
        if (!mUpload.uvs.empty()) layout.attributeMask |= kVertexUv;
        if (!mUpload.uvs2.empty()) layout.attributeMask |= kVertexUv2;
        layout.vertexCount = mVertexCapacity;
        layout.indexCount = mIndexCapacity;
        layout.indexFormat = prisma::IndexFormat::UInt32;
        mSurfaceReady = store->addDynamicSurface(mOwned, layout, kMeshName) >= 0;
    }
    if (mSurfaceReady) store->updateSurface(mOwned, 0, mUpload);
}

void SoftBody3D::notificationHandler(int what)
{
    MeshInstance3D::notificationHandler(what);
    switch (what)
    {
        case kNotificationPhysicsProcess:
            if (mDirty) build();
            followPins();
            break;
        case kNotificationProcess:
            if (!mDirty) upload();
            break;
        case kNotificationExitTree:
            release();
            mDirty = true;
            break;
        default:
            break;
    }
}

void SoftBody3D::bindProperties()
{
    using DB = NodeClassDB;
    using P = SoftBody3D;
    auto parameters = &P::getParameters;
    auto setParameters = &P::setParameters;
    using S = PhysicsSoftBodyParameters;
    DB::addFieldProperty<P>(
            PropertyInfo::range(PropertyType::Float, "mass", 0.001, 1000.0, 0.001, true),
            parameters, setParameters, &S::mass);
    DB::addFieldProperty<P>(PropertyInfo::range(PropertyType::Float, "structural_compliance", 0.0,
                                    1.0, 0.000001, true),
            parameters, setParameters, &S::structuralCompliance);
    DB::addFieldProperty<P>(
            PropertyInfo::range(PropertyType::Float, "bend_compliance", 0.0, 1.0, 0.000001, true),
            parameters, setParameters, &S::bendCompliance);
    DB::addFieldProperty<P>(PropertyInfo::enumeration("bend_constraint", "Distance,Dihedral"),
            parameters, setParameters, &S::bend);
    DB::addFieldProperty<P>(
            PropertyInfo::range(PropertyType::Float, "pressure", 0.0, 1000.0, 0.01, true),
            parameters, setParameters, &S::pressure);
    DB::addFieldProperty<P>(
            PropertyInfo::range(PropertyType::Int, "substeps", 1.0, 64.0, 1.0, true), parameters,
            setParameters, &S::substeps);
    DB::addFieldProperty<P>(PropertyInfo::range(PropertyType::Float, "damping", 0.0, 1.0, 0.001),
            parameters, setParameters, &S::damping);
    DB::addFieldProperty<P>(
            PropertyInfo::range(PropertyType::Float, "collision_margin", 0.0, 1.0, 0.001, true),
            parameters, setParameters, &S::collisionMargin);

    DB::addPropertyGroup<P>("Collision", "collision_");
    DB::addFieldProperty<P>(PropertyInfo::layers3DPhysics("collision_layer"), parameters,
            setParameters, &S::collisionGroup);
    DB::addFieldProperty<P>(PropertyInfo::layers3DPhysics("collision_mask"), parameters,
            setParameters, &S::collisionMask);

    DB::addPropertyGroup<P>("Simulation", "");
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "attachment_limit", 0.0, 100.0, 0.01, true),
            &P::setAttachmentLimit, &P::getAttachmentLimit);
    DB::addProperty<P>(PropertyInfo(PropertyType::Vec3, "wind"), &P::setWind, &P::getWind);
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "simulation_enabled"),
            &P::setSimulationEnabled, &P::isSimulationEnabled);
}

namespace
{

constexpr std::int64_t kSoftBodyDataVersion = 1;

}

bool SoftBody3D::saveData(ct::Json& data, SceneResult& result, const ct::String& path) const
{
    ct::Json base = ct::Json::object();
    const bool hasBase = MeshInstance3D::saveData(base, result, path);
    if (hasBase) data = base;
    if (mGeometry.positions.empty() && !hasBase) return false;

    data["version"] = ct::Json(static_cast<long long>(kSoftBodyDataVersion));
    if (mGeometry.positions.empty()) return true;

    data["geometry"] = scenejson::fromSurface(mGeometry);
    ct::Json pinned = ct::Json::array();
    for (std::size_t i = 0; i < mPinFlags.size(); ++i)
        if (mPinFlags[i] != 0) pinned.push_back(ct::Json(static_cast<long long>(i)));
    if (pinned.size() > 0) data["pinned"] = pinned;
    return true;
}

bool SoftBody3D::loadData(const ct::Json& data, SceneResult& result, const ct::String& path)
{
    if (!scenejson::checkVersion(data, kSoftBodyDataVersion, result, path)) return false;
    MeshInstance3D::loadData(data, result, path);

    if (const ct::Json* geometry = data.find("geometry"))
    {
        SurfaceData surface;
        if (!scenejson::toSurface(*geometry, surface, result, scenejson::field(path, "geometry")))
            return false;
        setGeometry(surface);
        if (const ct::Json* pinned = scenejson::arrayField(data, "pinned", result, path))
        {
            for (std::size_t i = 0; i < pinned->size(); ++i)
            {
                std::int64_t vertex = 0;
                if (!scenejson::toInt(pinned->at(i), vertex) || vertex < 0 ||
                        vertex >= static_cast<std::int64_t>(surface.positions.size()))
                {
                    result.addError(scenejson::element(path, "pinned", i),
                            "not a vertex of the geometry");
                    continue;
                }
                setPinned(static_cast<std::size_t>(vertex), true);
            }
        }
    }
    return true;
}

} // namespace prisma3d
