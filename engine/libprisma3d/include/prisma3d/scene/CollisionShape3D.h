#pragma once

#include "prisma3d/physics/PhysicsServer3D.h"
#include "prisma3d/render/MeshStore.h"
#include "prisma3d/scene/Node3D.h"

#include <ct/vector.hpp>

namespace prisma3d
{

class Terrain3D;

enum class CollisionShapeKind : std::uint8_t
{
    None,
    Box,
    Sphere,
    Capsule,
    Cylinder,
    Plane,
    ConvexHull,
    ConcaveMesh,
    HeightMap
};

class CollisionShape3D : public Node3D
{
public:
    PRISMA3D_NODE_CLASS(CollisionShape3D, Node3D);
    static void bindProperties();

    bool saveData(ct::Json& data, SceneResult& result, const ct::String& path) const override;
    bool loadData(const ct::Json& data, SceneResult& result, const ct::String& path) override;

    CollisionShape3D();
    ~CollisionShape3D() override;

    void setBox(const Math::Vec3& size);
    void setBoxSize(const Math::Vec3& size);
    Math::Vec3 getBoxSize() const;
    void setSphere(float radius);
    void setSphereRadius(float radius);
    float getSphereRadius() const;
    void setCapsule(float radius, float height);
    void setCapsuleRadius(float radius);
    void setCapsuleHeight(float height);
    float getCapsuleRadius() const;
    float getCapsuleHeight() const;
    void setCylinder(float radius, float height);
    void setCylinderRadius(float radius);
    void setCylinderHeight(float height);
    float getCylinderRadius() const;
    float getCylinderHeight() const;
    void setPlane(const Math::Vec3& normal, float distance);
    void setPlaneNormal(const Math::Vec3& normal);
    const Math::Vec3& getPlaneNormal() const;
    void setPlaneDistance(float distance);
    float getPlaneDistance() const;
    void setKind(CollisionShapeKind kind);
    bool setConvexFromPoints(const Math::Vec3* points, std::size_t count);
    bool setConvexFromMesh(MeshId mesh);
    bool setConcaveFromMesh(MeshId mesh);
    MeshId getConcaveMesh() const;
    int getSurfaceOfTriangle(int triangle) const;
    bool setHeightMapFromTerrain(const Terrain3D& terrain);
    void setSourceMeshPath(const ct::String& path);
    const ct::String& getSourceMeshPath() const;
    void setSourceTerrainPath(const ct::String& path);
    const ct::String& getSourceTerrainPath() const;
    bool applySource();

    void setDisabled(bool disabled);
    bool isDisabled() const;

    CollisionShapeKind getKind() const;
    PhysicsShapeId getShape() const;
    bool getHullPoints(ct::Vector<Math::Vec3>& out) const;

protected:
    void notificationHandler(int what) override;
    void validateProperty(PropertyInfo& info) const override;

private:
    friend class PhysicsBody3D;

    bool prepare(CollisionShapeKind kind, PhysicsShapeType type);
    bool select(CollisionShapeKind kind);
    bool ensureShape();
    void applyDimensions();
    void notifyBody(bool leaving);

    PhysicsShapeId mShape;
    CollisionShapeKind mKind;
    bool mDisabled;
    Math::Vec3 mBox;
    float mSphereRadius;
    float mCapsuleRadius;
    float mCapsuleHeight;
    float mCylinderRadius;
    float mCylinderHeight;
    Math::Vec3 mPlaneNormal;
    float mPlaneDistance;
    ct::Vector<Math::Vec3> mPoints;
    ct::String mSourceMesh;
    ct::String mSourceTerrain;
    MeshId mConcaveMesh;
};

} // namespace prisma3d
