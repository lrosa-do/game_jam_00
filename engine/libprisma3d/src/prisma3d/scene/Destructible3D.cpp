#include "prisma3d/scene/Destructible3D.h"

#include "prisma3d/scene/CollisionShape3D.h"
#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/ResourceProperties.h"
#include "prisma3d/scene/SceneJson.h"
#include "prisma3d/scene/SceneResources.h"

#include <ct/json.hpp>

#include <math.h>

namespace prisma3d
{

namespace
{

struct Plane
{
    Math::Vec3 normal;
    float offset;
};

Math::Vec3 columnOf(const Math::Vec4& column)
{
    return Math::Vec3(column.x, column.y, column.z);
}

bool onPlane(const Math::Vec3& point, const ct::Vector<Plane>& planes, float tolerance)
{
    for (std::size_t i = 0; i < planes.size(); ++i)
        if (fabsf(Math::Vec3::Dot(planes[i].normal, point) + planes[i].offset) <= tolerance)
            return true;
    return false;
}

} // namespace

DebrisMesh3D::DebrisMesh3D()
        : MeshInstance3D(),
          mOwned()
{
}

DebrisMesh3D::~DebrisMesh3D()
{
    setMesh(MeshId());
    MeshStore* store = MeshStore::singleton();
    if (store && mOwned.valid()) store->destroy(mOwned);
}

void DebrisMesh3D::adopt(MeshId mesh)
{
    mOwned = mesh;
    setMesh(mesh);
}

MeshId DebrisMesh3D::getOwnedMesh() const
{
    return mOwned;
}

DebrisBody3D::DebrisBody3D()
        : RigidBody3D(),
          mLifetime(0.0f),
          mAge(0.0f)
{
    setProcess(true);
}

void DebrisBody3D::setLifetime(float seconds)
{
    mLifetime = seconds;
}

float DebrisBody3D::getLifetime() const
{
    return mLifetime;
}

float DebrisBody3D::getAge() const
{
    return mAge;
}

void DebrisBody3D::notificationHandler(int what)
{
    RigidBody3D::notificationHandler(what);
    if (what != kNotificationProcess || mLifetime <= 0.0f) return;
    mAge += static_cast<float>(getProcessDeltaTime());
    if (mAge >= mLifetime)
    {
        queueFree();
        return;
    }
    const float shrinkStart = mLifetime * 0.8f;
    if (mAge > shrinkStart)
    {
        const float t = (mAge - shrinkStart) / (mLifetime - shrinkStart);
        const float scale = 1.0f - t * 0.95f;
        setScale(Math::Vec3(scale, scale, scale));
    }
}

DestructibleBody3D::DestructibleBody3D()
        : RigidBody3D(),
          mFragments(12),
          mBreakSpeed(0.0f),
          mDensity(500.0f),
          mLifetime(8.0f),
          mPush(2.0f),
          mOuter(),
          mInner(),
          mSeed(2463534242u),
          mSeedValue(2463534242u),
          mPoints(),
          mLastVelocity(Math::Vec3(0.0f, 0.0f, 0.0f)),
          mBroken(false)
{
    setPhysicsProcess(true);
}

void DestructibleBody3D::setFragmentCount(int count)
{
    mFragments = count < 2 ? 2 : (count > 64 ? 64 : count);
}

int DestructibleBody3D::getFragmentCount() const
{
    return mFragments;
}

void DestructibleBody3D::setBreakSpeed(float metersPerSecond)
{
    mBreakSpeed = metersPerSecond;
    refreshContactMonitor();
}

bool DestructibleBody3D::needsContactMonitor() const
{
    return mBreakSpeed > 0.0f;
}

float DestructibleBody3D::getBreakSpeed() const
{
    return mBreakSpeed;
}

void DestructibleBody3D::setDensity(float density)
{
    mDensity = density > 1.0f ? density : 1.0f;
}

float DestructibleBody3D::getDensity() const
{
    return mDensity;
}

void DestructibleBody3D::setDebrisLifetime(float seconds)
{
    mLifetime = seconds;
}

float DestructibleBody3D::getDebrisLifetime() const
{
    return mLifetime;
}

void DestructibleBody3D::setPushSpeed(float speed)
{
    mPush = speed;
}

float DestructibleBody3D::getPushSpeed() const
{
    return mPush;
}

void DestructibleBody3D::setOuterMaterial(MaterialId material)
{
    mOuter = material;
}

MaterialId DestructibleBody3D::getOuterMaterial() const
{
    return mOuter;
}

void DestructibleBody3D::setInnerMaterial(MaterialId material)
{
    mInner = material;
}

MaterialId DestructibleBody3D::getInnerMaterial() const
{
    return mInner;
}

void DestructibleBody3D::setSeed(std::uint32_t seed)
{
    mSeedValue = seed == 0 ? 2463534242u : seed;
    mSeed = mSeedValue;
}

std::uint32_t DestructibleBody3D::getSeed() const
{
    return mSeedValue;
}

void DestructibleBody3D::setSourcePoints(const Math::Vec3* points, std::size_t count)
{
    mPoints.clear();
    if (points && count > 0) mPoints.assign(points, points + count);
}

bool DestructibleBody3D::isBroken() const
{
    return mBroken;
}

float DestructibleBody3D::random()
{
    mSeed ^= mSeed << 13;
    mSeed ^= mSeed >> 17;
    mSeed ^= mSeed << 5;
    return static_cast<float>(mSeed & 0xFFFFFFu) / static_cast<float>(0x1000000u);
}

namespace
{

bool solidShard(const PhysicsShard& shard, const Math::Vec3& scale)
{
    Math::Vec3 low = shard.vertices[0];
    Math::Vec3 high = shard.vertices[0];
    for (std::size_t v = 1; v < shard.vertices.size(); ++v)
    {
        const Math::Vec3& p = shard.vertices[v];
        low = Math::Vec3(fminf(low.x, p.x), fminf(low.y, p.y), fminf(low.z, p.z));
        high = Math::Vec3(fmaxf(high.x, p.x), fmaxf(high.y, p.y), fmaxf(high.z, p.z));
    }
    const Math::Vec3 extent((high.x - low.x) * scale.x, (high.y - low.y) * scale.y,
            (high.z - low.z) * scale.z);
    const float thinnest = fminf(extent.x, fminf(extent.y, extent.z));
    return thinnest >= 0.015f && shard.volume * scale.x * scale.y * scale.z >= 1e-6f;
}

} // namespace

bool DestructibleBody3D::collectPoints(ct::Vector<Math::Vec3>& out) const
{
    out.clear();
    if (mPoints.size() >= 4)
    {
        out = mPoints;
        return true;
    }
    for (std::size_t i = 0; i < getChildCount(); ++i)
    {
        const CollisionShape3D* shape = getChild(i)->castTo<CollisionShape3D>();
        if (!shape || shape->isDisabled()) continue;
        ct::Vector<Math::Vec3> local;
        if (!shape->getHullPoints(local)) continue;
        const Math::Mat4 frame = shape->getTransform();
        for (std::size_t p = 0; p < local.size(); ++p)
            out.push_back(columnOf(frame.col0) * local[p].x + columnOf(frame.col1) * local[p].y +
                          columnOf(frame.col2) * local[p].z + columnOf(frame.col3));
        return out.size() >= 4;
    }
    return false;
}

void DestructibleBody3D::notificationHandler(int what)
{
    RigidBody3D::notificationHandler(what);
    if (what == kNotificationPhysicsProcess) mLastVelocity = getLinearVelocity();
}

void DestructibleBody3D::contactStarted(PhysicsBody3D* other, const PhysicsContact& contact)
{
    if (mBroken || mBreakSpeed <= 0.0f) return;
    Math::Vec3 relative = mLastVelocity;
    if (RigidBody3D* body = other ? other->castTo<RigidBody3D>() : nullptr)
        relative = relative - body->getLinearVelocity();
    const float approach = fabsf(Math::Vec3::Dot(relative, contact.normal));
    if (approach >= mBreakSpeed) breakAt(contact.point, relative * getMass());
}

bool DestructibleBody3D::breakAt(const Math::Vec3& globalPoint, const Math::Vec3& impulse)
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    Node* parent = getParent();
    if (mBroken || !server || !parent || !isInsideTree()) return false;
    ct::Vector<Math::Vec3> points;
    if (!collectPoints(points)) return false;

    const Math::Mat4 global = getGlobalTransform();
    const Math::Vec3 scale(columnOf(global.col0).Length(), columnOf(global.col1).Length(),
            columnOf(global.col2).Length());
    const Math::Vec3 impactLocal = toLocal(globalPoint);

    ct::Vector<Math::Vec3> sites;
    sites.reserve(static_cast<std::size_t>(mFragments));
    const std::size_t count = points.size();
    for (int i = 0; i < mFragments; ++i)
    {
        float weights[4];
        std::size_t picks[4];
        float sum = 0.0f;
        for (int k = 0; k < 4; ++k)
        {
            picks[k] = static_cast<std::size_t>(random() * static_cast<float>(count)) % count;
            weights[k] = 0.05f + random();
            sum += weights[k];
        }
        Math::Vec3 site(0.0f, 0.0f, 0.0f);
        for (int k = 0; k < 4; ++k) site = site + points[picks[k]] * (weights[k] / sum);
        if ((i & 1) == 0) site = site + (impactLocal - site) * (random() * 0.6f);
        sites.push_back(site);
    }

    ct::Vector<PhysicsShard> shards;
    if (!server->shatter(points.data(), points.size(), sites.data(), sites.size(), shards))
        return false;

    ct::Vector<Math::Vec3> hullVertices;
    ct::Vector<std::uint32_t> hullIndices;
    ct::Vector<Plane> planes;
    if (server->convexHullMesh(points.data(), points.size(), hullVertices, hullIndices))
    {
        for (std::size_t t = 0; t + 2 < hullIndices.size(); t += 3)
        {
            const Math::Vec3 a = hullVertices[hullIndices[t]];
            const Math::Vec3 b = hullVertices[hullIndices[t + 1]];
            const Math::Vec3 c = hullVertices[hullIndices[t + 2]];
            Math::Vec3 normal = Math::Vec3::Cross(b - a, c - a);
            if (normal.LengthSquared() < 1e-14f) continue;
            normal = normal.Normalized();
            Plane plane;
            plane.normal = normal;
            plane.offset = -Math::Vec3::Dot(normal, a);
            planes.push_back(plane);
        }
    }
    float extent = 0.0f;
    for (std::size_t i = 0; i < points.size(); ++i) extent = fmaxf(extent, points[i].Length());
    const float tolerance = fmaxf(extent * 1e-3f, 1e-5f);

    MeshStore* meshes = MeshStore::singleton();
    if (meshes && !meshes->driver()) meshes = nullptr;
    MaterialId outer = mOuter;
    if (!outer.valid())
        for (std::size_t i = 0; i < getChildCount() && !outer.valid(); ++i)
            if (const MeshInstance3D* visual = getChild(i)->castTo<MeshInstance3D>())
                outer = visual->getMaterialOverride();
    const MaterialId inner = mInner.valid() ? mInner : outer;

    const Math::Vec3 linear = getLinearVelocity();
    const Math::Vec3 angular = getAngularVelocity();
    const Math::Vec3 center = getGlobalPosition();
    const float totalVolume = [&]()
    {
        float v = 0.0f;
        for (std::size_t i = 0; i < shards.size(); ++i) v += shards[i].volume;
        return v > 1e-9f ? v : 1.0f;
    }();

    ct::Vector<DebrisBody3D*> pieces;
    for (std::size_t s = 0; s < shards.size(); ++s)
    {
        const PhysicsShard& shard = shards[s];
        if (shard.vertices.size() < 4 || !solidShard(shard, scale)) continue;
        // Built outside the tree and placed before it enters: a body that enters at the
        // origin is pulled back there by the physics sync before its transform is pushed.
        DebrisBody3D* piece = new DebrisBody3D();
        piece->setInternal(true);
        Math::Mat4 frame = global;
        frame.col3 =
                Math::Vec4(global.col0.x * shard.centroid.x + global.col1.x * shard.centroid.y +
                                   global.col2.x * shard.centroid.z + global.col3.x,
                        global.col0.y * shard.centroid.x + global.col1.y * shard.centroid.y +
                                global.col2.y * shard.centroid.z + global.col3.y,
                        global.col0.z * shard.centroid.x + global.col1.z * shard.centroid.y +
                                global.col2.z * shard.centroid.z + global.col3.z,
                        1.0f);
        const Node3D* parent3D = parent->castTo<Node3D>();
        piece->setTransform(parent3D ? parent3D->getGlobalTransform().InverseAffine() * frame : frame);

        ct::Vector<Math::Vec3> scaled;
        scaled.reserve(shard.vertices.size());
        for (std::size_t v = 0; v < shard.vertices.size(); ++v)
            scaled.push_back(Math::Vec3(shard.vertices[v].x * scale.x,
                    shard.vertices[v].y * scale.y, shard.vertices[v].z * scale.z));
        CollisionShape3D* shape = piece->createChild<CollisionShape3D>();
        shape->setConvexFromPoints(scaled.data(), scaled.size());
        piece->setMass(fmaxf(shard.volume * scale.x * scale.y * scale.z * mDensity, 0.05f));
        piece->setFriction(getFriction());
        piece->setBounce(getBounce());
        piece->setCollisionLayer(getCollisionLayer());
        piece->setCollisionMask(getCollisionMask());
        piece->setLifetime(mLifetime);

        const Math::Vec3 worldCentroid = columnOf(frame.col3);
        Math::Vec3 away = worldCentroid - globalPoint;
        away = away.LengthSquared() > 1e-8f ? away.Normalized() : Math::Vec3(0.0f, 1.0f, 0.0f);
        Math::Vec3 velocity = linear + Math::Vec3::Cross(angular, worldCentroid - center) +
                              away * (mPush * (0.5f + random()));
        velocity = velocity + impulse * (1.0f / (getMass() > 0.0f ? getMass() : 1.0f)) *
                                      (shard.volume / totalVolume);
        piece->setLinearVelocity(velocity);
        piece->setAngularVelocity(angular);

        if (meshes)
        {
            SurfaceData outerData;
            SurfaceData innerData;
            for (std::size_t t = 0; t + 2 < shard.indices.size(); t += 3)
            {
                const Math::Vec3 a = shard.vertices[shard.indices[t]];
                const Math::Vec3 b = shard.vertices[shard.indices[t + 1]];
                const Math::Vec3 c = shard.vertices[shard.indices[t + 2]];
                const Math::Vec3 la = a + shard.centroid;
                const Math::Vec3 lb = b + shard.centroid;
                const Math::Vec3 lc = c + shard.centroid;
                const bool exterior = onPlane(la, planes, tolerance) &&
                                      onPlane(lb, planes, tolerance) &&
                                      onPlane(lc, planes, tolerance);
                SurfaceData& target = exterior ? outerData : innerData;
                Math::Vec3 normal = Math::Vec3::Cross(b - a, c - a);
                normal = normal.LengthSquared() > 1e-14f ? normal.Normalized()
                                                         : Math::Vec3(0.0f, 1.0f, 0.0f);
                const float ax = fabsf(normal.x);
                const float ay = fabsf(normal.y);
                const float az = fabsf(normal.z);
                const Math::Vec3 corners[3] = { a, b, c };
                const Math::Vec3 locals[3] = { la, lb, lc };
                for (int k = 0; k < 3; ++k)
                {
                    const std::uint32_t index = static_cast<std::uint32_t>(target.positions.size());
                    target.positions.push_back(Math::Vec3(corners[k].x * scale.x,
                            corners[k].y * scale.y, corners[k].z * scale.z));
                    target.normals.push_back(normal);
                    const Math::Vec3& l = locals[k];
                    Math::Vec2 uv(l.z, l.y);
                    if (ay >= ax && ay >= az) uv = Math::Vec2(l.x, l.z);
                    else if (az >= ax && az >= ay)
                        uv = Math::Vec2(l.x, l.y);
                    target.uvs.push_back(uv * 0.5f + Math::Vec2(0.5f, 0.5f));
                    target.indices.push_back(index);
                }
            }
            const MeshId mesh = meshes->create("fragment");
            std::uint32_t surface = 0;
            bool ok = false;
            if (!outerData.indices.empty() && meshes->addSurface(mesh, outerData, "outer"))
            {
                meshes->setSurfaceMaterial(mesh, surface++, outer);
                ok = true;
            }
            if (!innerData.indices.empty() && meshes->addSurface(mesh, innerData, "inner"))
            {
                meshes->setSurfaceMaterial(mesh, surface++, inner);
                ok = true;
            }
            if (ok)
            {
                DebrisMesh3D* visual = piece->createChild<DebrisMesh3D>();
                visual->adopt(mesh);
            }
            else
                meshes->destroy(mesh);
        }
        parent->addChild(piece);
        pieces.push_back(piece);
    }

    mBroken = true;
    if (broken) broken(pieces);
    queueFree();
    return true;
}

void DebrisMesh3D::bindProperties()
{
}

void DebrisBody3D::bindProperties()
{
    using DB = NodeClassDB;
    using P = DebrisBody3D;
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "lifetime", 0.0, 3600.0, 0.01, true),
            &P::setLifetime, &P::getLifetime);
}

void DestructibleBody3D::bindProperties()
{
    using DB = NodeClassDB;
    using P = DestructibleBody3D;
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Int, "fragment_count", 2.0, 256.0, 1.0, true),
            &P::setFragmentCount, &P::getFragmentCount);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "break_speed", 0.0, 1000.0, 0.01, true),
            &P::setBreakSpeed, &P::getBreakSpeed);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "density", 0.001, 100000.0, 0.001, true),
            &P::setDensity, &P::getDensity);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "debris_lifetime", 0.0, 3600.0, 0.01, true),
            &P::setDebrisLifetime, &P::getDebrisLifetime);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "push_speed", 0.0, 1000.0, 0.01, true),
            &P::setPushSpeed, &P::getPushSpeed);
    DB::addProperty<P>(PropertyInfo(PropertyType::Int, "seed"), &P::setSeed, &P::getSeed);
    resourceprop::add<P, MaterialId>(PropertyInfo::resource("outer_material", "Material"),
            &P::setOuterMaterial, &P::getOuterMaterial);
    resourceprop::add<P, MaterialId>(PropertyInfo::resource("inner_material", "Material"),
            &P::setInnerMaterial, &P::getInnerMaterial);
}

namespace
{

constexpr std::int64_t kDestructibleDataVersion = 1;

}

bool DestructibleBody3D::saveData(ct::Json& data, SceneResult& result, const ct::String& path) const
{
    (void) result;
    (void) path;
    if (mPoints.empty()) return false;

    data["version"] = ct::Json(static_cast<long long>(kDestructibleDataVersion));
    ct::Json points = ct::Json::array();
    for (std::size_t i = 0; i < mPoints.size(); ++i)
        points.push_back(scenejson::fromVec3(mPoints[i]));
    data["source_points"] = points;
    return true;
}

bool DestructibleBody3D::loadData(const ct::Json& data, SceneResult& result, const ct::String& path)
{
    if (!scenejson::checkVersion(data, kDestructibleDataVersion, result, path)) return false;

    const ct::Json* field = scenejson::arrayField(data, "source_points", result, path);
    if (!field) return true;
    ct::Vector<Math::Vec3> points;
    for (std::size_t i = 0; i < field->size(); ++i)
    {
        Math::Vec3 point;
        if (!scenejson::toVec3(field->at(i), point))
        {
            result.addError(scenejson::element(path, "source_points", i), "expected [x, y, z]");
            continue;
        }
        points.push_back(point);
    }
    setSourcePoints(points.data(), points.size());
    return true;
}

} // namespace prisma3d
