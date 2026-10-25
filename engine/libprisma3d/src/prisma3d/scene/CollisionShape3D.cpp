#include "prisma3d/scene/CollisionShape3D.h"

#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/PhysicsBody3D.h"
#include "prisma3d/scene/SceneJson.h"
#include "prisma3d/scene/SceneResources.h"
#include "prisma3d/scene/Terrain3D.h"

#include "platform.h"

#include <ct/json.hpp>

#include <math.h>
#include <string.h>

namespace prisma3d
{

CollisionShape3D::CollisionShape3D()
        : Node3D(),
          mShape(),
          mKind(CollisionShapeKind::None),
          mDisabled(false),
          mBox(1.0f, 1.0f, 1.0f),
          mSphereRadius(0.5f),
          mCapsuleRadius(0.5f),
          mCapsuleHeight(2.0f),
          mCylinderRadius(0.5f),
          mCylinderHeight(2.0f),
          mPlaneNormal(0.0f, 1.0f, 0.0f),
          mPlaneDistance(0.0f),
          mConcaveMesh()
{
    setNotifyLocalTransform(true);
}

CollisionShape3D::~CollisionShape3D()
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (server && mShape.valid()) server->shapeFree(mShape);
}

bool CollisionShape3D::prepare(CollisionShapeKind kind, PhysicsShapeType type)
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (!server) return false;
    if (kind != CollisionShapeKind::ConcaveMesh) mConcaveMesh = MeshId();
    if (mKind == kind && server->shapeValid(mShape)) return true;
    if (mShape.valid()) server->shapeFree(mShape);
    mShape = server->shapeCreate(type);
    const CollisionShapeKind before = mKind;
    mKind = mShape.valid() ? kind : CollisionShapeKind::None;
    if (mKind != before) notifyPropertyListChanged();
    return mShape.valid();
}

void CollisionShape3D::validateProperty(PropertyInfo& info) const
{
    Node3D::validateProperty(info);
    CollisionShapeKind owner = CollisionShapeKind::None;
    if (info.name == "box_size") owner = CollisionShapeKind::Box;
    else if (info.name == "sphere_radius")
        owner = CollisionShapeKind::Sphere;
    else if (info.name == "capsule_radius" || info.name == "capsule_height")
        owner = CollisionShapeKind::Capsule;
    else if (info.name == "cylinder_radius" || info.name == "cylinder_height")
        owner = CollisionShapeKind::Cylinder;
    else if (info.name == "plane_normal" || info.name == "plane_distance")
        owner = CollisionShapeKind::Plane;
    else if (info.name == "source_mesh")
    {
        if (mKind != CollisionShapeKind::ConvexHull && mKind != CollisionShapeKind::ConcaveMesh)
            info.usage = PropertyUsage::NoEditor;
        return;
    }
    else if (info.name == "source_terrain")
    {
        if (mKind != CollisionShapeKind::HeightMap) info.usage = PropertyUsage::NoEditor;
        return;
    }
    else
        return;
    if (mKind != owner) info.usage = PropertyUsage::NoEditor;
}

bool CollisionShape3D::select(CollisionShapeKind kind)
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    switch (kind)
    {
        case CollisionShapeKind::Box:
            if (server && !prepare(kind, PhysicsShapeType::Box)) return false;
            break;
        case CollisionShapeKind::Sphere:
            if (server && !prepare(kind, PhysicsShapeType::Sphere)) return false;
            break;
        case CollisionShapeKind::Capsule:
            if (server && !prepare(kind, PhysicsShapeType::Capsule)) return false;
            break;
        case CollisionShapeKind::Cylinder:
            if (server && !prepare(kind, PhysicsShapeType::Cylinder)) return false;
            break;
        case CollisionShapeKind::Plane:
            if (server && !prepare(kind, PhysicsShapeType::WorldBoundary)) return false;
            break;
        default:
            return false;
    }
    const bool changed = mKind != kind;
    mKind = kind;
    if (changed) notifyPropertyListChanged();
    return true;
}

void CollisionShape3D::applyDimensions()
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (!server || !server->shapeValid(mShape)) return;
    switch (mKind)
    {
        case CollisionShapeKind::Box:
            server->shapeSetBox(mShape, mBox * 0.5f);
            break;
        case CollisionShapeKind::Sphere:
            server->shapeSetSphere(mShape, mSphereRadius);
            break;
        case CollisionShapeKind::Capsule:
            server->shapeSetCapsule(mShape, mCapsuleRadius, mCapsuleHeight);
            break;
        case CollisionShapeKind::Cylinder:
            server->shapeSetCylinder(mShape, mCylinderRadius, mCylinderHeight);
            break;
        case CollisionShapeKind::Plane:
            server->shapeSetWorldBoundary(mShape, mPlaneNormal, mPlaneDistance);
            break;
        default:
            break;
    }
}

bool CollisionShape3D::ensureShape()
{
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (!server) return false;
    if (server->shapeValid(mShape)) return true;
    mShape = PhysicsShapeId();
    switch (mKind)
    {
        case CollisionShapeKind::Box:
        case CollisionShapeKind::Sphere:
        case CollisionShapeKind::Capsule:
        case CollisionShapeKind::Cylinder:
        case CollisionShapeKind::Plane:
        {
            const CollisionShapeKind kind = mKind;
            mKind = CollisionShapeKind::None;
            if (!select(kind)) return false;
            applyDimensions();
            return true;
        }
        case CollisionShapeKind::ConvexHull:
        {
            const CollisionShapeKind kind = mKind;
            mKind = CollisionShapeKind::None;
            if (mPoints.empty() || !prepare(kind, PhysicsShapeType::ConvexHull))
            {
                mKind = kind;
                return false;
            }
            server->shapeSetConvexHull(mShape, mPoints.data(), mPoints.size());
            return true;
        }
        default:
            return false;
    }
}

void CollisionShape3D::notifyBody(bool leaving)
{
    Node* parent = getParent();
    PhysicsBody3D* body = parent ? parent->castTo<PhysicsBody3D>() : nullptr;
    if (body) body->bindShape(leaving ? this : nullptr);
}

void CollisionShape3D::setKind(CollisionShapeKind kind)
{
    if (kind == mKind && (!PhysicsServer3D::singleton() || ensureShape())) return;
    if (kind == CollisionShapeKind::None)
    {
        PhysicsServer3D* server = PhysicsServer3D::singleton();
        if (server && mShape.valid()) server->shapeFree(mShape);
        mShape = PhysicsShapeId();
        const bool changed = mKind != CollisionShapeKind::None;
        mKind = CollisionShapeKind::None;
        notifyBody(false);
        if (changed) notifyPropertyListChanged();
        return;
    }
    if (kind == CollisionShapeKind::ConvexHull || kind == CollisionShapeKind::ConcaveMesh ||
            kind == CollisionShapeKind::HeightMap)
    {
        if (kind == mKind) return;
        PhysicsServer3D* server = PhysicsServer3D::singleton();
        if (server && mShape.valid()) server->shapeFree(mShape);
        mShape = PhysicsShapeId();
        mKind = kind;
        notifyBody(false);
        notifyPropertyListChanged();
        applySource();
        return;
    }
    if (!select(kind)) return;
    applyDimensions();
    notifyBody(false);
}

void CollisionShape3D::setSourceMeshPath(const ct::String& path)
{
    if (mSourceMesh == path) return;
    mSourceMesh = path;
    applySource();
}

const ct::String& CollisionShape3D::getSourceMeshPath() const
{
    return mSourceMesh;
}

void CollisionShape3D::setSourceTerrainPath(const ct::String& path)
{
    if (mSourceTerrain == path) return;
    mSourceTerrain = path;
    applySource();
}

const ct::String& CollisionShape3D::getSourceTerrainPath() const
{
    return mSourceTerrain;
}

bool CollisionShape3D::applySource()
{
    if (mKind == CollisionShapeKind::HeightMap)
    {
        if (mSourceTerrain.empty()) return false;
        Node* node = getNode(mSourceTerrain);
        const Terrain3D* terrain = node ? node->castTo<Terrain3D>() : nullptr;
        return terrain && setHeightMapFromTerrain(*terrain);
    }
    if (mKind != CollisionShapeKind::ConvexHull && mKind != CollisionShapeKind::ConcaveMesh)
        return false;
    if (mSourceMesh.empty()) return false;
    sceneres::clearError();
    const MeshId mesh = sceneres::loadMesh(mSourceMesh);
    if (!mesh.valid())
    {
        log_error("collision shape: %s", sceneres::takeError().c_str());
        return false;
    }
    return mKind == CollisionShapeKind::ConvexHull ? setConvexFromMesh(mesh)
                                                   : setConcaveFromMesh(mesh);
}

void CollisionShape3D::setBox(const Math::Vec3& size)
{
    mBox = Math::Vec3(fabsf(size.x), fabsf(size.y), fabsf(size.z));
    if (!select(CollisionShapeKind::Box)) return;
    applyDimensions();
    notifyBody(false);
}

void CollisionShape3D::setBoxSize(const Math::Vec3& size)
{
    mBox = Math::Vec3(fabsf(size.x), fabsf(size.y), fabsf(size.z));
    if (mKind != CollisionShapeKind::Box) return;
    applyDimensions();
    notifyBody(false);
}

Math::Vec3 CollisionShape3D::getBoxSize() const
{
    return mBox;
}

void CollisionShape3D::setSphere(float radius)
{
    mSphereRadius = fabsf(radius);
    if (!select(CollisionShapeKind::Sphere)) return;
    applyDimensions();
    notifyBody(false);
}

void CollisionShape3D::setSphereRadius(float radius)
{
    mSphereRadius = fabsf(radius);
    if (mKind != CollisionShapeKind::Sphere) return;
    applyDimensions();
    notifyBody(false);
}

float CollisionShape3D::getSphereRadius() const
{
    return mSphereRadius;
}

void CollisionShape3D::setCapsule(float radius, float height)
{
    mCapsuleRadius = fabsf(radius);
    mCapsuleHeight = fabsf(height);
    if (!select(CollisionShapeKind::Capsule)) return;
    applyDimensions();
    notifyBody(false);
}

void CollisionShape3D::setCapsuleRadius(float radius)
{
    mCapsuleRadius = fabsf(radius);
    if (mKind != CollisionShapeKind::Capsule) return;
    applyDimensions();
    notifyBody(false);
}

void CollisionShape3D::setCapsuleHeight(float height)
{
    mCapsuleHeight = fabsf(height);
    if (mKind != CollisionShapeKind::Capsule) return;
    applyDimensions();
    notifyBody(false);
}

float CollisionShape3D::getCapsuleRadius() const
{
    return mCapsuleRadius;
}

float CollisionShape3D::getCapsuleHeight() const
{
    return mCapsuleHeight;
}

void CollisionShape3D::setCylinder(float radius, float height)
{
    mCylinderRadius = fabsf(radius);
    mCylinderHeight = fabsf(height);
    if (!select(CollisionShapeKind::Cylinder)) return;
    applyDimensions();
    notifyBody(false);
}

void CollisionShape3D::setCylinderRadius(float radius)
{
    mCylinderRadius = fabsf(radius);
    if (mKind != CollisionShapeKind::Cylinder) return;
    applyDimensions();
    notifyBody(false);
}

void CollisionShape3D::setCylinderHeight(float height)
{
    mCylinderHeight = fabsf(height);
    if (mKind != CollisionShapeKind::Cylinder) return;
    applyDimensions();
    notifyBody(false);
}

float CollisionShape3D::getCylinderRadius() const
{
    return mCylinderRadius;
}

float CollisionShape3D::getCylinderHeight() const
{
    return mCylinderHeight;
}

void CollisionShape3D::setPlane(const Math::Vec3& normal, float distance)
{
    mPlaneNormal = normal;
    mPlaneDistance = distance;
    if (!select(CollisionShapeKind::Plane)) return;
    applyDimensions();
    notifyBody(false);
}

void CollisionShape3D::setPlaneNormal(const Math::Vec3& normal)
{
    mPlaneNormal = normal;
    if (mKind != CollisionShapeKind::Plane) return;
    applyDimensions();
    notifyBody(false);
}

const Math::Vec3& CollisionShape3D::getPlaneNormal() const
{
    return mPlaneNormal;
}

void CollisionShape3D::setPlaneDistance(float distance)
{
    mPlaneDistance = distance;
    if (mKind != CollisionShapeKind::Plane) return;
    applyDimensions();
    notifyBody(false);
}

float CollisionShape3D::getPlaneDistance() const
{
    return mPlaneDistance;
}

bool CollisionShape3D::setConvexFromPoints(const Math::Vec3* points, std::size_t count)
{
    if (!prepare(CollisionShapeKind::ConvexHull, PhysicsShapeType::ConvexHull)) return false;
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    mPoints.clear();
    if (points && count > 0) mPoints.assign(points, points + count);
    server->shapeSetConvexHull(mShape, points, count);
    notifyBody(false);
    return server->shapeGetConvexVertexCount(mShape) >= 4;
}

bool CollisionShape3D::setConvexFromMesh(MeshId mesh)
{
    MeshStore* store = MeshStore::singleton();
    const CollisionMesh* collision = store ? store->collision(mesh) : nullptr;
    if (!collision || collision->triangleCount() == 0) return false;
    ct::Vector<Math::Vec3> points;
    points.reserve(static_cast<std::size_t>(collision->triangleCount()) * 3);
    for (std::uint32_t t = 0; t < collision->triangleCount(); ++t)
    {
        Math::Vec3 a, b, c;
        collision->triangle(t, a, b, c);
        points.push_back(a);
        points.push_back(b);
        points.push_back(c);
    }
    return setConvexFromPoints(points.data(), points.size());
}

bool CollisionShape3D::setConcaveFromMesh(MeshId mesh)
{
    MeshStore* store = MeshStore::singleton();
    const CollisionMesh* collision = store ? store->collision(mesh) : nullptr;
    if (!collision || collision->triangleCount() == 0) return false;
    if (!prepare(CollisionShapeKind::ConcaveMesh, PhysicsShapeType::ConcavePolygon)) return false;
    PhysicsServer3D::singleton()->shapeSetConcaveCollision(mShape, *collision);
    mConcaveMesh = mesh;
    notifyBody(false);
    return true;
}

MeshId CollisionShape3D::getConcaveMesh() const
{
    return mConcaveMesh;
}

int CollisionShape3D::getSurfaceOfTriangle(int triangle) const
{
    if (triangle < 0 || mKind != CollisionShapeKind::ConcaveMesh || !mConcaveMesh.valid())
        return -1;
    const MeshStore* store = MeshStore::singleton();
    const CollisionMesh* collision = store ? store->collision(mConcaveMesh) : nullptr;
    return collision ? collision->partOfTriangle(static_cast<std::uint32_t>(triangle)) : -1;
}

bool CollisionShape3D::setHeightMapFromTerrain(const Terrain3D& terrain)
{
    if (!terrain.isLoaded() || !terrain.getTerrain().isPrepared()) return false;
    if (!prepare(CollisionShapeKind::HeightMap, PhysicsShapeType::HeightMap)) return false;
    PhysicsServer3D::singleton()->shapeSetHeightMapFromTerrain(mShape, terrain.getTerrain());
    notifyBody(false);
    return true;
}

void CollisionShape3D::setDisabled(bool disabled)
{
    if (mDisabled == disabled) return;
    mDisabled = disabled;
    notifyBody(false);
}

bool CollisionShape3D::isDisabled() const
{
    return mDisabled;
}

bool CollisionShape3D::getHullPoints(ct::Vector<Math::Vec3>& out) const
{
    out.clear();
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (!server || !server->shapeValid(mShape)) return false;
    switch (mKind)
    {
        case CollisionShapeKind::Box:
        {
            const Math::Vec3 half = server->shapeGetBox(mShape);
            for (int i = 0; i < 8; ++i)
                out.push_back(Math::Vec3((i & 1) ? half.x : -half.x, (i & 2) ? half.y : -half.y,
                        (i & 4) ? half.z : -half.z));
            return true;
        }
        case CollisionShapeKind::Sphere:
        case CollisionShapeKind::Capsule:
        {
            float radius = 0.0f;
            float offset = 0.0f;
            if (mKind == CollisionShapeKind::Sphere) radius = server->shapeGetSphere(mShape);
            else
            {
                radius = server->shapeGetCapsuleRadius(mShape);
                offset = server->shapeGetCapsuleHeight(mShape) * 0.5f - radius;
                if (offset < 0.0f) offset = 0.0f;
            }
            const float d = 0.57735027f;
            const Math::Vec3 directions[14] = { Math::Vec3(1, 0, 0), Math::Vec3(-1, 0, 0),
                Math::Vec3(0, 1, 0), Math::Vec3(0, -1, 0), Math::Vec3(0, 0, 1),
                Math::Vec3(0, 0, -1), Math::Vec3(d, d, d), Math::Vec3(-d, d, d),
                Math::Vec3(d, -d, d), Math::Vec3(-d, -d, d), Math::Vec3(d, d, -d),
                Math::Vec3(-d, d, -d), Math::Vec3(d, -d, -d), Math::Vec3(-d, -d, -d) };
            for (int i = 0; i < 14; ++i)
            {
                Math::Vec3 point = directions[i] * radius;
                point.y += point.y >= 0.0f ? offset : -offset;
                out.push_back(point);
            }
            return true;
        }
        case CollisionShapeKind::Cylinder:
        {
            const float radius = server->shapeGetCylinderRadius(mShape);
            const float half = server->shapeGetCylinderHeight(mShape) * 0.5f;
            for (int i = 0; i < 16; ++i)
            {
                const float angle = 6.28318531f * static_cast<float>(i) / 16.0f;
                out.push_back(Math::Vec3(cosf(angle) * radius, half, sinf(angle) * radius));
                out.push_back(Math::Vec3(cosf(angle) * radius, -half, sinf(angle) * radius));
            }
            return true;
        }
        case CollisionShapeKind::ConvexHull:
            out = mPoints;
            return out.size() >= 4;
        default:
            return false;
    }
}

CollisionShapeKind CollisionShape3D::getKind() const
{
    return mKind;
}

PhysicsShapeId CollisionShape3D::getShape() const
{
    return mShape;
}

void CollisionShape3D::notificationHandler(int what)
{
    Node3D::notificationHandler(what);
    switch (what)
    {
        case kNotificationEnterTree:
            ensureShape();
            notifyBody(false);
            break;
        case kNotificationLocalTransformChanged:
            notifyBody(false);
            break;
        case kNotificationExitTree:
            notifyBody(true);
            break;
        default:
            break;
    }
}

void CollisionShape3D::bindProperties()
{
    using DB = NodeClassDB;
    using P = CollisionShape3D;
    DB::addProperty<P>(PropertyInfo::enumeration("shape_type",
                               "None,Box,Sphere,Capsule,Cylinder,Plane,Convex Hull,Concave "
                               "Mesh,Height Map"),
            &P::setKind, &P::getKind);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Vec3, "box_size", 0.001, 1024.0, 0.001, true),
            &P::setBoxSize, &P::getBoxSize);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "sphere_radius", 0.001, 1024.0, 0.001, true),
            &P::setSphereRadius, &P::getSphereRadius);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "capsule_radius", 0.001, 1024.0, 0.001, true),
            &P::setCapsuleRadius, &P::getCapsuleRadius);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "capsule_height", 0.001, 1024.0, 0.001, true),
            &P::setCapsuleHeight, &P::getCapsuleHeight);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "cylinder_radius", 0.001, 1024.0, 0.001, true),
            &P::setCylinderRadius, &P::getCylinderRadius);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "cylinder_height", 0.001, 1024.0, 0.001, true),
            &P::setCylinderHeight, &P::getCylinderHeight);
    DB::addProperty<P>(PropertyInfo(PropertyType::Vec3, "plane_normal"), &P::setPlaneNormal,
            &P::getPlaneNormal);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "plane_distance", -1024.0, 1024.0,
                               0.001, true, true),
            &P::setPlaneDistance, &P::getPlaneDistance);
    DB::addProperty<P>(PropertyInfo::resource("source_mesh", "Mesh"), &P::setSourceMeshPath,
            &P::getSourceMeshPath);
    DB::addProperty<P>(PropertyInfo::nodePath("source_terrain", "Terrain3D"),
            &P::setSourceTerrainPath, &P::getSourceTerrainPath);
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "disabled"), &P::setDisabled,
            &P::isDisabled);
}

namespace
{

constexpr std::int64_t kCollisionShapeDataVersion = 1;

}

bool CollisionShape3D::saveData(ct::Json& data, SceneResult& result, const ct::String& path) const
{
    if (mKind != CollisionShapeKind::ConvexHull && mKind != CollisionShapeKind::ConcaveMesh &&
            mKind != CollisionShapeKind::HeightMap)
        return false;

    data["version"] = ct::Json(static_cast<long long>(kCollisionShapeDataVersion));
    data["kind"] = ct::Json(
            mKind == CollisionShapeKind::ConvexHull
                    ? "convex_hull"
                    : (mKind == CollisionShapeKind::ConcaveMesh ? "concave_mesh" : "height_map"));
    if (mKind == CollisionShapeKind::ConvexHull)
    {
        ct::Json points = ct::Json::array();
        for (std::size_t i = 0; i < mPoints.size(); ++i)
            points.push_back(scenejson::fromVec3(mPoints[i]));
        data["points"] = points;
        return true;
    }

    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (!server || !server->shapeValid(mShape))
    {
        result.addWarning(path, "the shape data lives in the physics server, which is not "
                                "available; the shape was not saved");
        return false;
    }
    if (mKind == CollisionShapeKind::ConcaveMesh)
    {
        ct::Vector<Math::Vec3> vertices;
        ct::Vector<std::uint32_t> indices;
        if (!server->shapeGetConcaveFaces(mShape, vertices, indices)) return false;
        ct::Json positions = ct::Json::array();
        for (std::size_t i = 0; i < vertices.size(); ++i)
            positions.push_back(scenejson::fromVec3(vertices[i]));
        data["vertices"] = positions;
        ct::Json faces = ct::Json::array();
        for (std::size_t i = 0; i < indices.size(); ++i)
            faces.push_back(ct::Json(static_cast<long long>(indices[i])));
        data["indices"] = faces;
        return true;
    }

    ct::Vector<float> heights;
    if (!server->shapeGetHeightMap(mShape, heights)) return false;
    data["width"] = ct::Json(static_cast<long long>(server->shapeGetHeightMapWidth(mShape)));
    data["depth"] = ct::Json(static_cast<long long>(server->shapeGetHeightMapDepth(mShape)));
    data["scale"] = scenejson::fromVec3(server->shapeGetHeightMapScale(mShape));
    data["offset"] = scenejson::fromVec3(server->shapeGetHeightMapOffset(mShape));
    data["heights"] = scenejson::fromBlock(heights.data(), heights.size() * sizeof(float));
    return true;
}

bool CollisionShape3D::loadData(const ct::Json& data, SceneResult& result, const ct::String& path)
{
    if (!scenejson::checkVersion(data, kCollisionShapeDataVersion, result, path)) return false;

    const ct::String kind = scenejson::getString(data, "kind", ct::String());
    if (kind != "convex_hull" && kind != "concave_mesh" && kind != "height_map")
    {
        result.addError(scenejson::field(path, "kind"),
                "expected convex_hull, concave_mesh or height_map");
        return false;
    }

    if (kind == "convex_hull")
    {
        const ct::Json* field = scenejson::arrayField(data, "points", result, path);
        if (!field)
        {
            result.addError(scenejson::field(path, "points"), "missing points");
            return false;
        }
        ct::Vector<Math::Vec3> points;
        for (std::size_t i = 0; i < field->size(); ++i)
        {
            Math::Vec3 point;
            if (!scenejson::toVec3(field->at(i), point))
            {
                result.addError(scenejson::element(path, "points", i), "expected [x, y, z]");
                return false;
            }
            points.push_back(point);
        }
        if (!setConvexFromPoints(points.data(), points.size()) && PhysicsServer3D::singleton())
            result.addWarning(path, "the points do not make a hull");
        else if (!PhysicsServer3D::singleton())
            mPoints = points;
        return true;
    }

    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (!server)
    {
        result.addWarning(path, "the physics server is not available; the shape data was not "
                                "loaded");
        return true;
    }

    if (kind == "concave_mesh")
    {
        const ct::Json* positions = scenejson::arrayField(data, "vertices", result, path);
        const ct::Json* faces = scenejson::arrayField(data, "indices", result, path);
        if (!positions || !faces)
        {
            result.addError(path, "missing vertices or indices");
            return false;
        }
        ct::Vector<Math::Vec3> vertices;
        for (std::size_t i = 0; i < positions->size(); ++i)
        {
            Math::Vec3 vertex;
            if (!scenejson::toVec3(positions->at(i), vertex))
            {
                result.addError(scenejson::element(path, "vertices", i), "expected [x, y, z]");
                return false;
            }
            vertices.push_back(vertex);
        }
        ct::Vector<std::uint32_t> indices;
        for (std::size_t i = 0; i < faces->size(); ++i)
        {
            std::int64_t index = 0;
            if (!scenejson::toInt(faces->at(i), index) || index < 0 ||
                    index >= static_cast<std::int64_t>(vertices.size()))
            {
                result.addError(scenejson::element(path, "indices", i), "not a vertex index");
                return false;
            }
            indices.push_back(static_cast<std::uint32_t>(index));
        }
        if (!prepare(CollisionShapeKind::ConcaveMesh, PhysicsShapeType::ConcavePolygon))
            return false;
        server->shapeSetConcaveFaces(mShape, vertices.data(), vertices.size(), indices.data(),
                indices.size());
        notifyBody(false);
        return true;
    }

    {
        const std::int64_t width = scenejson::getInt(data, "width", 0);
        const std::int64_t depth = scenejson::getInt(data, "depth", 0);
        ct::Vector<std::uint8_t> bytes;
        const ct::Json* block = data.find("heights");
        if (width < 2 || depth < 2 || !block || !scenejson::toBlock(*block, bytes) ||
                bytes.size() != static_cast<std::size_t>(width * depth) * sizeof(float))
        {
            result.addError(path, "the height map needs width, depth and width * depth heights");
            return false;
        }
        ct::Vector<float> heights;
        heights.resize(bytes.size() / sizeof(float));
        memcpy(heights.data(), bytes.data(), bytes.size());
        Math::Vec3 scale(1.0f, 1.0f, 1.0f);
        Math::Vec3 offset(0.0f, 0.0f, 0.0f);
        scenejson::readVec3Field(data, "scale", scale, result, path);
        scenejson::readVec3Field(data, "offset", offset, result, path);
        if (!prepare(CollisionShapeKind::HeightMap, PhysicsShapeType::HeightMap)) return false;
        server->shapeSetHeightMap(mShape, static_cast<int>(width), static_cast<int>(depth),
                heights.data());
        server->shapeSetHeightMapTransform(mShape, scale, offset);
        notifyBody(false);
        return true;
    }
}

} // namespace prisma3d
