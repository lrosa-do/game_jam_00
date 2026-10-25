#pragma once

#include "prisma3d/scene/Collision.h"
#include "prisma3d/scene/Node.h"

#include "mathc.h"

namespace prisma3d
{

class Node3D : public Node
{
public:
    PRISMA3D_NODE_CLASS(Node3D, Node);
    static void bindProperties();
    static constexpr NodeClass kAncestralClass = NodeClass::Node3D;

    enum Notification3D : int
    {
        kNotificationEnterWorld = 41,
        kNotificationExitWorld = 42,
        kNotificationVisibilityChanged = 43,
        kNotificationLocalTransformChanged = 44
    };

    Node3D();
    ~Node3D() override;

    Node3D* getParentNode3D() const;

    void setPosition(const Math::Vec3& position);
    Math::Vec3 getPosition() const;
    void setRotation(const Math::Vec3& eulerRadians);
    Math::Vec3 getRotation() const;
    void setRotationDegrees(const Math::Vec3& eulerDegrees);
    Math::Vec3 getRotationDegrees() const;
    void setQuaternion(const Math::Quaternion& quaternion);
    Math::Quaternion getQuaternion() const;
    void setScale(const Math::Vec3& scale);
    Math::Vec3 getScale() const;

    void setTransform(const Math::Mat4& transform);
    Math::Mat4 getTransform() const;
    void setGlobalTransform(const Math::Mat4& transform);
    Math::Mat4 getGlobalTransform() const;
    void setGlobalPosition(const Math::Vec3& position);
    Math::Vec3 getGlobalPosition() const;
    void setBasis(const Math::Mat3& basis);
    Math::Mat3 getBasis() const;
    void setGlobalBasis(const Math::Mat3& basis);
    Math::Mat3 getGlobalBasis() const;
    void setGlobalRotation(const Math::Vec3& eulerRadians);
    Math::Vec3 getGlobalRotation() const;
    void setGlobalRotationDegrees(const Math::Vec3& eulerDegrees);
    Math::Vec3 getGlobalRotationDegrees() const;

    void rotate(const Math::Vec3& axis, float angle);
    void rotateX(float angle);
    void rotateY(float angle);
    void rotateZ(float angle);
    void translate(const Math::Vec3& offset);
    void translateObjectLocal(const Math::Vec3& offset);
    void rotateObjectLocal(const Math::Vec3& axis, float angle);
    void scale(const Math::Vec3& ratio);
    void scaleObjectLocal(const Math::Vec3& scale);
    void globalRotate(const Math::Vec3& axis, float angle);
    void globalScale(const Math::Vec3& scale);
    void globalTranslate(const Math::Vec3& offset);
    void orthonormalize();
    void setIdentity();
    void lookAt(const Math::Vec3& target, const Math::Vec3& up = Math::Vec3(0.0f, 1.0f, 0.0f));
    void lookAtFromPosition(const Math::Vec3& position, const Math::Vec3& target,
            const Math::Vec3& up = Math::Vec3(0.0f, 1.0f, 0.0f));
    Math::Vec3 toLocal(const Math::Vec3& global) const;
    Math::Vec3 toGlobal(const Math::Vec3& local) const;

    void setNotifyTransform(bool enabled);
    bool isTransformNotificationEnabled() const;
    void setNotifyLocalTransform(bool enabled);
    bool isLocalTransformNotificationEnabled() const;

    void setVisible(bool visible);
    void show();
    void hide();
    bool isVisible() const;
    bool isVisibleInTree() const;

    void forceUpdateTransform();

    void setCollisionType(int type);
    int getCollisionType() const;
    void setCollisionEnabled(bool enabled);
    bool isCollisionEnabled() const;
    void setCollisionRadius(float radius);
    void setCollisionRadius(float radiusX, float radiusY);
    void setCollisionCapsule(float radius, float height);
    Math::Vec3 getCollisionRadii() const;
    void setCollisionBox(const Math::Vec3& min, const Math::Vec3& max);
    void setCollisionRadii(const Math::Vec3& radii);
    void setCollisionBoxMin(const Math::Vec3& min);
    Math::Vec3 getCollisionBoxMin() const;
    void setCollisionBoxMax(const Math::Vec3& max);
    Math::Vec3 getCollisionBoxMax() const;
    void setCollisionShape(CollisionShapeType shape);
    void setCollisionMesh(MeshId mesh);
    MeshId getCollisionMesh() const;
    void setStoredCollisionMesh(MeshId mesh);
    MeshId getStoredCollisionMesh() const;
    CollisionShapeType getCollisionShape() const;
    CollisionShapeType getStoredCollisionShape() const;
    std::uint32_t countCollisions() const;
    const CollisionHit& getCollision(std::uint32_t index) const;
    Node3D* collidedWith(int type) const;
    void resetCollision();

protected:
    void notificationHandler(int what) override;
    void validateProperty(PropertyInfo& info) const override;

private:
    friend class SceneTree;

    enum DirtyFlags : std::uint32_t
    {
        kDirtyNone = 0,
        kDirtyLocalTransform = 1u << 1,
        kDirtyGlobalTransform = 1u << 2,
        kDirtyEuler = 1u << 3
    };

    friend class CollisionWorld;

    struct CollisionData
    {
        CollisionData() = default;
        CollisionData(const CollisionData&) = delete;
        CollisionData& operator=(const CollisionData&) = delete;
        ~CollisionData()
        {
            delete boxMesh;
        }

        int type = 0;
        int lastType = 0;
        CollisionShapeType shape = CollisionShapeType::None;
        Math::Vec3 radii = Math::Vec3(1.0f, 1.0f, 1.0f);
        Math::Vec3 boxMin = Math::Vec3(-0.5f, -0.5f, -0.5f);
        Math::Vec3 boxMax = Math::Vec3(0.5f, 0.5f, 0.5f);
        MeshId mesh;
        CollisionMesh* boxMesh = nullptr;
        Math::Vec3 previousPosition;
        bool hasPrevious = false;
        ct::Vector<CollisionHit> hits;
    };

    CollisionData& collisionData();
    CollisionShapeType effectiveCollisionShape() const;
    MeshId effectiveCollisionMesh() const;

    void propagateTransformChanged(Node3D* origin);
    void updateLocalTransform() const;
    void propagateVisibilityChanged();
    void notifyDirty();

    mutable Math::Mat4 mLocalTransform;
    mutable Math::Mat4 mGlobalTransform;
    Math::Vec3 mPosition;
    Math::Quaternion mRotation;
    mutable Math::Vec3 mEulerRotation;
    Math::Vec3 mScale;
    mutable std::uint32_t mDirty;
    bool mInsideWorld;
    bool mVisible;
    bool mNotifyTransform;
    bool mNotifyLocalTransform;
    bool mTransformChangedQueued;
    CollisionData* mCollision;
};

} // namespace prisma3d
