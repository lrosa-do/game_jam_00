#include "prisma3d/scene/Node3D.h"

#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/ResourceProperties.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/VisualInstance3D.h"

namespace prisma3d
{

namespace
{

const float kEpsilon = 1e-8f;
const float kDegreesToRadians = 0.017453292519943295f;
const float kRadiansToDegrees = 57.29577951308232f;

Math::Mat3 orthonormalized(const Math::Mat3& basis)
{
    Math::Vec3 x = basis.col0;
    Math::Vec3 y = basis.col1;
    Math::Vec3 z = basis.col2;
    x = x.Normalized();
    y = (y - x * x.Dot(y)).Normalized();
    z = (z - x * x.Dot(z) - y * y.Dot(z)).Normalized();
    return Math::Mat3(x, y, z);
}

constexpr std::uint32_t clearBits(std::uint32_t value, std::uint32_t bits)
{
    return value & ~bits;
}

} // namespace

Node3D::Node3D()
        : Node(),
          mLocalTransform(Math::Mat4::Identity()),
          mGlobalTransform(Math::Mat4::Identity()),
          mPosition(0.0f, 0.0f, 0.0f),
          mRotation(Math::Quaternion::Identity()),
          mEulerRotation(0.0f, 0.0f, 0.0f),
          mScale(1.0f, 1.0f, 1.0f),
          mDirty(kDirtyNone),
          mInsideWorld(false),
          mVisible(true),
          mNotifyTransform(false),
          mNotifyLocalTransform(false),
          mTransformChangedQueued(false),
          mCollision(nullptr)
{
    defineAncestry(NodeClass::Node3D);
}

void Node3D::bindProperties()
{
    using DB = NodeClassDB;
    DB::addPropertyGroup<Node3D>("Transform", "");
    DB::addProperty<Node3D>(PropertyInfo::range(PropertyType::Vec3, "position", -99999.0, 99999.0,
                                    0.001, true, true),
            &Node3D::setPosition, &Node3D::getPosition);
    PropertyInfo rotation =
            PropertyInfo::range(PropertyType::Vec3, "rotation", -360.0, 360.0, 0.1, true, true);
    rotation.radiansAsDegrees = true;
    DB::addProperty<Node3D>(rotation, &Node3D::setRotation, &Node3D::getRotation);
    DB::addProperty<Node3D>(PropertyInfo(PropertyType::Vec3, "rotation_degrees", PropertyHint::None,
                                    "", PropertyUsage::None),
            &Node3D::setRotationDegrees, &Node3D::getRotationDegrees);
    DB::addProperty<Node3D>(PropertyInfo(PropertyType::Quat, "quaternion", PropertyHint::None, "",
                                    PropertyUsage::Editor),
            &Node3D::setQuaternion, &Node3D::getQuaternion);
    DB::addProperty<Node3D>(PropertyInfo(PropertyType::Vec3, "scale"), &Node3D::setScale,
            &Node3D::getScale);
    DB::addProperty<Node3D>(PropertyInfo(PropertyType::Vec3, "global_position", PropertyHint::None,
                                    "", PropertyUsage::None),
            &Node3D::setGlobalPosition, &Node3D::getGlobalPosition);
    DB::addProperty<Node3D>(PropertyInfo(PropertyType::Vec3, "global_rotation", PropertyHint::None,
                                    "", PropertyUsage::None),
            &Node3D::setGlobalRotation, &Node3D::getGlobalRotation);
    DB::addProperty<Node3D>(PropertyInfo(PropertyType::Vec3, "global_rotation_degrees",
                                    PropertyHint::None, "", PropertyUsage::None),
            &Node3D::setGlobalRotationDegrees, &Node3D::getGlobalRotationDegrees);

    DB::addPropertyGroup<Node3D>("Visibility", "");
    DB::addProperty<Node3D>(PropertyInfo(PropertyType::Bool, "visible"), &Node3D::setVisible,
            &Node3D::isVisible);

    DB::addPropertyGroup<Node3D>("Collision", "collision_");
    DB::addProperty<Node3D>(PropertyInfo(PropertyType::Bool, "collision_enabled",
                                    PropertyHint::GroupEnable, "", PropertyUsage::Editor),
            &Node3D::setCollisionEnabled, &Node3D::isCollisionEnabled);
    DB::addProperty<Node3D>(PropertyInfo(PropertyType::Int, "collision_type"),
            &Node3D::setCollisionType, &Node3D::getCollisionType);
    DB::addProperty<Node3D>(PropertyInfo::enumeration("collision_shape", "None,Ellipsoid,Box,Mesh"),
            &Node3D::setCollisionShape, &Node3D::getStoredCollisionShape);
    DB::addProperty<Node3D>(
            PropertyInfo::range(PropertyType::Vec3, "collision_radii", 0.0, 1024.0, 0.001, true),
            &Node3D::setCollisionRadii, &Node3D::getCollisionRadii);
    DB::addProperty<Node3D>(PropertyInfo(PropertyType::Vec3, "collision_box_min"),
            &Node3D::setCollisionBoxMin, &Node3D::getCollisionBoxMin);
    DB::addProperty<Node3D>(PropertyInfo(PropertyType::Vec3, "collision_box_max"),
            &Node3D::setCollisionBoxMax, &Node3D::getCollisionBoxMax);
    resourceprop::add<Node3D, MeshId>(PropertyInfo::resource("collision_mesh", "Mesh"),
            &Node3D::setStoredCollisionMesh, &Node3D::getStoredCollisionMesh);
}

Node3D::~Node3D()
{
    if (mCollision)
    {
        if (getTree()) getTree()->getCollisionWorld().remove(this);
        delete mCollision;
        mCollision = nullptr;
    }

    if (mTransformChangedQueued && getTree())
    {
        ct::Vector<Node3D*>& queue = getTree()->mTransformChanged;
        for (std::size_t i = 0; i < queue.size(); ++i)
        {
            if (queue[i] == this) queue[i] = nullptr;
        }
        mTransformChangedQueued = false;
    }
}

Node3D* Node3D::getParentNode3D() const
{
    Node* parent = getParent();
    return parent ? parent->castTo<Node3D>() : nullptr;
}

void Node3D::setPosition(const Math::Vec3& position)
{
    mPosition = position;
    mDirty |= kDirtyLocalTransform;
    propagateTransformChanged(this);
    if (mNotifyLocalTransform) notification(kNotificationLocalTransformChanged);
}

Math::Vec3 Node3D::getPosition() const
{
    return mPosition;
}

void Node3D::setRotation(const Math::Vec3& eulerRadians)
{
    mEulerRotation = eulerRadians;
    mRotation = Math::Quaternion::FromEulerAngles(eulerRadians.x, eulerRadians.y, eulerRadians.z);
    mDirty = clearBits(mDirty, kDirtyEuler) | kDirtyLocalTransform;
    propagateTransformChanged(this);
    if (mNotifyLocalTransform) notification(kNotificationLocalTransformChanged);
}

Math::Vec3 Node3D::getRotation() const
{
    if (mDirty & kDirtyEuler)
    {
        mEulerRotation = mRotation.ToEulerAngles();
        mDirty = clearBits(mDirty, kDirtyEuler);
    }
    return mEulerRotation;
}

void Node3D::setQuaternion(const Math::Quaternion& quaternion)
{
    mRotation = quaternion.Normalized();
    mEulerRotation = mRotation.ToEulerAngles();
    mDirty = clearBits(mDirty, kDirtyEuler) | kDirtyLocalTransform;
    propagateTransformChanged(this);
    if (mNotifyLocalTransform) notification(kNotificationLocalTransformChanged);
}

Math::Quaternion Node3D::getQuaternion() const
{
    return mRotation;
}

void Node3D::setScale(const Math::Vec3& scale)
{
    mScale = scale;
    mDirty |= kDirtyLocalTransform;
    propagateTransformChanged(this);
    if (mNotifyLocalTransform) notification(kNotificationLocalTransformChanged);
}

Math::Vec3 Node3D::getScale() const
{
    return mScale;
}

void Node3D::setTransform(const Math::Mat4& transform)
{
    const Math::Vec3 x = transform.col0.xyz();
    const Math::Vec3 y = transform.col1.xyz();
    const Math::Vec3 z = transform.col2.xyz();

    Math::Vec3 scale(x.Length(), y.Length(), z.Length());
    if (Math::Mat3(x, y, z).Determinant() < 0.0f) scale.x = -scale.x;

    Math::Mat3 basis;
    basis.col0 = scale.x * scale.x > kEpsilon ? x / scale.x : Math::Vec3::UnitX;
    basis.col1 = scale.y * scale.y > kEpsilon ? y / scale.y : Math::Vec3::UnitY;
    basis.col2 = scale.z * scale.z > kEpsilon ? z / scale.z : Math::Vec3::UnitZ;

    mPosition = transform.GetTranslation();
    mRotation = Math::Quaternion::FromMat3(basis).Normalized();
    mScale = scale;
    mLocalTransform = transform;
    mDirty = clearBits(mDirty, kDirtyLocalTransform) | kDirtyEuler;

    propagateTransformChanged(this);
    if (mNotifyLocalTransform) notification(kNotificationLocalTransformChanged);
}

Math::Mat4 Node3D::getTransform() const
{
    if (mDirty & kDirtyLocalTransform) updateLocalTransform();
    return mLocalTransform;
}

void Node3D::setGlobalTransform(const Math::Mat4& transform)
{
    Node3D* parent = getParentNode3D();
    setTransform(parent ? parent->getGlobalTransform().InverseAffine() * transform : transform);
}

Math::Mat4 Node3D::getGlobalTransform() const
{
    Node3D* parent = getParentNode3D();

    if (!isInsideTree())
        return parent ? parent->getGlobalTransform() * getTransform() : getTransform();

    if (mDirty & kDirtyGlobalTransform)
    {
        if (mDirty & kDirtyLocalTransform) updateLocalTransform();

        mGlobalTransform =
                parent ? parent->getGlobalTransform() * mLocalTransform : mLocalTransform;
        mDirty = clearBits(mDirty, kDirtyGlobalTransform);
    }

    return mGlobalTransform;
}

void Node3D::setGlobalPosition(const Math::Vec3& position)
{
    Math::Mat4 transform = getGlobalTransform();
    transform.col3 = Math::Vec4(position, 1.0f);
    setGlobalTransform(transform);
}

Math::Vec3 Node3D::getGlobalPosition() const
{
    return getGlobalTransform().GetTranslation();
}

void Node3D::setRotationDegrees(const Math::Vec3& eulerDegrees)
{
    setRotation(eulerDegrees * kDegreesToRadians);
}

Math::Vec3 Node3D::getRotationDegrees() const
{
    return getRotation() * kRadiansToDegrees;
}

void Node3D::setBasis(const Math::Mat3& basis)
{
    setTransform(Math::Mat4(basis, getTransform().GetTranslation()));
}

Math::Mat3 Node3D::getBasis() const
{
    return getTransform().UpperLeft3x3();
}

void Node3D::setGlobalBasis(const Math::Mat3& basis)
{
    setGlobalTransform(Math::Mat4(basis, getGlobalPosition()));
}

Math::Mat3 Node3D::getGlobalBasis() const
{
    return getGlobalTransform().UpperLeft3x3();
}

void Node3D::setGlobalRotation(const Math::Vec3& eulerRadians)
{
    const Math::Mat3 current = getGlobalBasis();
    Math::Vec3 scale(current.col0.Length(), current.col1.Length(), current.col2.Length());
    if (current.Determinant() < 0.0f) scale = scale * -1.0f;
    const Math::Mat3 rotation =
            Math::Quaternion::FromEulerAngles(eulerRadians.x, eulerRadians.y, eulerRadians.z)
                    .ToMat3();
    setGlobalBasis(rotation * Math::Mat3::Scale(scale));
}

Math::Vec3 Node3D::getGlobalRotation() const
{
    return Math::Quaternion::FromMat3(orthonormalized(getGlobalBasis()))
            .Normalized()
            .ToEulerAngles();
}

void Node3D::setGlobalRotationDegrees(const Math::Vec3& eulerDegrees)
{
    setGlobalRotation(eulerDegrees * kDegreesToRadians);
}

Math::Vec3 Node3D::getGlobalRotationDegrees() const
{
    return getGlobalRotation() * kRadiansToDegrees;
}

void Node3D::rotate(const Math::Vec3& axis, float angle)
{
    if (axis.LengthSquared() < kEpsilon) return;
    setQuaternion(Math::Quaternion::FromAxisAngle(axis.Normalized(), angle) * mRotation);
}

void Node3D::rotateX(float angle)
{
    rotate(Math::Vec3(1.0f, 0.0f, 0.0f), angle);
}

void Node3D::rotateY(float angle)
{
    rotate(Math::Vec3(0.0f, 1.0f, 0.0f), angle);
}

void Node3D::rotateZ(float angle)
{
    rotate(Math::Vec3(0.0f, 0.0f, 1.0f), angle);
}

void Node3D::translate(const Math::Vec3& offset)
{
    translateObjectLocal(offset);
}

void Node3D::translateObjectLocal(const Math::Vec3& offset)
{
    setPosition(mPosition + mRotation * (offset * mScale));
}

void Node3D::rotateObjectLocal(const Math::Vec3& axis, float angle)
{
    if (axis.LengthSquared() < kEpsilon) return;
    setBasis(getBasis() * Math::Quaternion::FromAxisAngle(axis.Normalized(), angle).ToMat3());
}

void Node3D::scale(const Math::Vec3& ratio)
{
    setBasis(Math::Mat3::Scale(ratio) * getBasis());
}

void Node3D::scaleObjectLocal(const Math::Vec3& scale)
{
    setBasis(getBasis() * Math::Mat3::Scale(scale));
}

void Node3D::globalScale(const Math::Vec3& scale)
{
    setGlobalBasis(Math::Mat3::Scale(scale) * getGlobalBasis());
}

void Node3D::orthonormalize()
{
    setBasis(orthonormalized(getBasis()));
}

void Node3D::setIdentity()
{
    setTransform(Math::Mat4::Identity());
}

void Node3D::globalRotate(const Math::Vec3& axis, float angle)
{
    if (axis.LengthSquared() < kEpsilon) return;
    Math::Mat4 transform = getGlobalTransform();
    const Math::Mat3 rotation = Math::Quaternion::FromAxisAngle(axis.Normalized(), angle).ToMat3();
    const Math::Mat3 basis = rotation * transform.UpperLeft3x3();
    transform.col0 = Math::Vec4(basis.col0, 0.0f);
    transform.col1 = Math::Vec4(basis.col1, 0.0f);
    transform.col2 = Math::Vec4(basis.col2, 0.0f);
    setGlobalTransform(transform);
}

void Node3D::globalTranslate(const Math::Vec3& offset)
{
    setGlobalPosition(getGlobalPosition() + offset);
}

void Node3D::lookAt(const Math::Vec3& target, const Math::Vec3& up)
{
    lookAtFromPosition(getGlobalPosition(), target, up);
}

void Node3D::lookAtFromPosition(const Math::Vec3& position, const Math::Vec3& target,
        const Math::Vec3& up)
{
    const Math::Vec3 forward = target - position;
    if (forward.LengthSquared() < kEpsilon || up.LengthSquared() < kEpsilon) return;
    if (up.Cross(forward).LengthSquared() < kEpsilon) return;

    const Math::Quaternion rotation = Math::Quaternion::LookRotation(-forward, up);
    const Math::Vec3 originalScale = mScale;
    setGlobalTransform(Math::Mat4(rotation.ToMat3(), position));
    setScale(originalScale);
}

Math::Vec3 Node3D::toLocal(const Math::Vec3& global) const
{
    return getGlobalTransform().InverseAffine().TransformPoint(global);
}

Math::Vec3 Node3D::toGlobal(const Math::Vec3& local) const
{
    return getGlobalTransform().TransformPoint(local);
}

void Node3D::setNotifyTransform(bool enabled)
{
    mNotifyTransform = enabled;
}

bool Node3D::isTransformNotificationEnabled() const
{
    return mNotifyTransform;
}

void Node3D::setNotifyLocalTransform(bool enabled)
{
    mNotifyLocalTransform = enabled;
}

bool Node3D::isLocalTransformNotificationEnabled() const
{
    return mNotifyLocalTransform;
}

void Node3D::setVisible(bool visible)
{
    if (mVisible == visible) return;

    mVisible = visible;

    if (!isInsideTree()) return;

    propagateVisibilityChanged();
}

void Node3D::show()
{
    setVisible(true);
}

void Node3D::hide()
{
    setVisible(false);
}

bool Node3D::isVisible() const
{
    return mVisible;
}

bool Node3D::isVisibleInTree() const
{
    const Node3D* node = this;
    while (node)
    {
        if (!node->mVisible) return false;
        node = node->getParentNode3D();
    }
    return true;
}

void Node3D::forceUpdateTransform()
{
    if (!isInsideTree() || !mTransformChangedQueued) return;

    ct::Vector<Node3D*>& queue = getTree()->mTransformChanged;
    for (std::size_t i = 0; i < queue.size(); ++i)
    {
        if (queue[i] == this) queue[i] = nullptr;
    }
    mTransformChangedQueued = false;

    notification(kNotificationTransformChanged);
}

void Node3D::setCollisionType(int type)
{
    if (type == 0 && !mCollision) return;

    const bool wasOn = getCollisionType() != 0;
    CollisionData& data = collisionData();
    data.type = type;
    if (wasOn != (type != 0)) notifyPropertyListChanged();
    SceneTree* tree = getTree();
    if (!tree || !isInsideTree()) return;

    if (type != 0) tree->getCollisionWorld().add(this);
    else
    {
        tree->getCollisionWorld().remove(this);
        data.hits.clear();
        data.hasPrevious = false;
    }
}

int Node3D::getCollisionType() const
{
    return mCollision ? mCollision->type : 0;
}

void Node3D::setCollisionEnabled(bool enabled)
{
    if (enabled == (getCollisionType() != 0)) return;
    CollisionData& data = collisionData();
    if (enabled) setCollisionType(data.lastType != 0 ? data.lastType : 1);
    else
    {
        data.lastType = data.type;
        setCollisionType(0);
    }
}

bool Node3D::isCollisionEnabled() const
{
    return getCollisionType() != 0;
}

void Node3D::setCollisionRadius(float radius)
{
    setCollisionRadius(radius, radius);
}

void Node3D::setCollisionRadius(float radiusX, float radiusY)
{
    CollisionData& data = collisionData();
    data.radii = Math::Vec3(radiusX, radiusY, radiusX);
    const bool changed = data.shape != CollisionShapeType::Ellipsoid;
    data.shape = CollisionShapeType::Ellipsoid;
    if (changed) notifyPropertyListChanged();
}

void Node3D::setCollisionCapsule(float radius, float height)
{
    setCollisionRadius(radius, height * 0.5f + radius);
}

Math::Vec3 Node3D::getCollisionRadii() const
{
    return mCollision ? mCollision->radii : Math::Vec3(1.0f, 1.0f, 1.0f);
}

void Node3D::setCollisionBox(const Math::Vec3& min, const Math::Vec3& max)
{
    CollisionData& data = collisionData();
    data.boxMin = Math::Vec3::Min(min, max);
    data.boxMax = Math::Vec3::Max(min, max);
    delete data.boxMesh;
    data.boxMesh = nullptr;
    const bool changed = data.shape != CollisionShapeType::Box;
    data.shape = CollisionShapeType::Box;
    if (changed) notifyPropertyListChanged();
}

void Node3D::setCollisionRadii(const Math::Vec3& radii)
{
    collisionData().radii = radii;
}

void Node3D::setCollisionBoxMin(const Math::Vec3& min)
{
    CollisionData& data = collisionData();
    data.boxMin = min;
    delete data.boxMesh;
    data.boxMesh = nullptr;
}

Math::Vec3 Node3D::getCollisionBoxMin() const
{
    return mCollision ? mCollision->boxMin : Math::Vec3(-0.5f, -0.5f, -0.5f);
}

void Node3D::setCollisionBoxMax(const Math::Vec3& max)
{
    CollisionData& data = collisionData();
    data.boxMax = max;
    delete data.boxMesh;
    data.boxMesh = nullptr;
}

Math::Vec3 Node3D::getCollisionBoxMax() const
{
    return mCollision ? mCollision->boxMax : Math::Vec3(0.5f, 0.5f, 0.5f);
}

void Node3D::setCollisionShape(CollisionShapeType shape)
{
    if (!mCollision && shape == CollisionShapeType::None) return;
    CollisionData& data = collisionData();
    const bool changed = data.shape != shape;
    data.shape = shape;
    if (changed) notifyPropertyListChanged();
}

void Node3D::setCollisionMesh(MeshId mesh)
{
    CollisionData& data = collisionData();
    data.mesh = mesh;
    const CollisionShapeType shape =
            mesh.valid() ? CollisionShapeType::Mesh : CollisionShapeType::None;
    const bool changed = data.shape != shape;
    data.shape = shape;
    if (changed) notifyPropertyListChanged();
}

void Node3D::validateProperty(PropertyInfo& info) const
{
    Node::validateProperty(info);
    const CollisionShapeType shape = getStoredCollisionShape();
    if (getCollisionType() == 0 &&
            (info.name == "collision_type" || info.name == "collision_shape" ||
                    info.name == "collision_radii" || info.name == "collision_box_min" ||
                    info.name == "collision_box_max" || info.name == "collision_mesh"))
        info.usage = PropertyUsage::NoEditor;
    else if (info.name == "collision_radii")
    {
        if (shape != CollisionShapeType::Ellipsoid) info.usage = PropertyUsage::NoEditor;
    }
    else if (info.name == "collision_box_min" || info.name == "collision_box_max")
    {
        if (shape != CollisionShapeType::Box) info.usage = PropertyUsage::NoEditor;
    }
    else if (info.name == "collision_mesh")
    {
        if (shape != CollisionShapeType::Mesh) info.usage = PropertyUsage::NoEditor;
    }
}

MeshId Node3D::getCollisionMesh() const
{
    return effectiveCollisionMesh();
}

void Node3D::setStoredCollisionMesh(MeshId mesh)
{
    if (!mCollision && !mesh.valid()) return;
    collisionData().mesh = mesh;
}

MeshId Node3D::getStoredCollisionMesh() const
{
    return mCollision ? mCollision->mesh : MeshId();
}

CollisionShapeType Node3D::getCollisionShape() const
{
    return effectiveCollisionShape();
}

CollisionShapeType Node3D::getStoredCollisionShape() const
{
    return mCollision ? mCollision->shape : CollisionShapeType::None;
}

CollisionShapeType Node3D::effectiveCollisionShape() const
{
    if (!mCollision) return CollisionShapeType::None;
    if (mCollision->shape == CollisionShapeType::None && mCollision->type != 0 &&
            isClass(NodeClass::MeshInstance3D))
        return CollisionShapeType::Mesh;
    return mCollision->shape;
}

MeshId Node3D::effectiveCollisionMesh() const
{
    if (!mCollision) return MeshId();
    if (mCollision->mesh.valid()) return mCollision->mesh;
    if (effectiveCollisionShape() == CollisionShapeType::Mesh && isClass(NodeClass::MeshInstance3D))
        return static_cast<const MeshInstance3D*>(this)->getMesh();
    return MeshId();
}

std::uint32_t Node3D::countCollisions() const
{
    return mCollision ? static_cast<std::uint32_t>(mCollision->hits.size()) : 0;
}

const CollisionHit& Node3D::getCollision(std::uint32_t index) const
{
    static const CollisionHit none;
    if (!mCollision || index >= mCollision->hits.size()) return none;
    return mCollision->hits[index];
}

Node3D* Node3D::collidedWith(int type) const
{
    if (!mCollision) return nullptr;
    for (std::size_t i = 0; i < mCollision->hits.size(); ++i)
    {
        Node3D* node = mCollision->hits[i].node;
        if (node && node->getCollisionType() == type) return node;
    }
    return nullptr;
}

void Node3D::resetCollision()
{
    if (!mCollision) return;
    mCollision->hasPrevious = false;
    mCollision->hits.clear();
}

Node3D::CollisionData& Node3D::collisionData()
{
    if (!mCollision) mCollision = new CollisionData();
    return *mCollision;
}

void Node3D::notificationHandler(int what)
{
    Node::notificationHandler(what);

    switch (what)
    {
        case kNotificationEnterTree:
            mDirty |= kDirtyGlobalTransform;
            notifyDirty();
            notification(kNotificationEnterWorld);
            if (mCollision)
            {
                mCollision->hasPrevious = false;
                if (mCollision->type != 0 && getTree()) getTree()->getCollisionWorld().add(this);
            }
            break;
        case kNotificationExitTree:
            notification(kNotificationExitWorld, true);
            if (mCollision)
            {
                if (getTree()) getTree()->getCollisionWorld().remove(this);
                mCollision->hits.clear();
                mCollision->hasPrevious = false;
            }
            if (mTransformChangedQueued && getTree())
            {
                ct::Vector<Node3D*>& queue = getTree()->mTransformChanged;
                for (std::size_t i = 0; i < queue.size(); ++i)
                {
                    if (queue[i] == this) queue[i] = nullptr;
                }
                mTransformChangedQueued = false;
            }
            break;
        case kNotificationEnterWorld:
            mInsideWorld = true;
            break;
        case kNotificationExitWorld:
            mInsideWorld = false;
            break;
        default:
            break;
    }
}

void Node3D::propagateTransformChanged(Node3D* origin)
{
    if (!isInsideTree()) return;

    for (std::size_t i = 0; i < getChildCount(); ++i)
    {
        Node3D* child = getChild(i)->castTo<Node3D>();
        if (child) child->propagateTransformChanged(origin);
    }

    notifyDirty();

    mDirty |= kDirtyGlobalTransform;
}

void Node3D::updateLocalTransform() const
{
    const Math::Mat3 rotation = mRotation.ToMat3();
    const Math::Mat3 basis(rotation.col0 * mScale.x, rotation.col1 * mScale.y,
            rotation.col2 * mScale.z);
    mLocalTransform = Math::Mat4(basis, mPosition);
    mDirty = clearBits(mDirty, kDirtyLocalTransform);
}

void Node3D::propagateVisibilityChanged()
{
    notification(kNotificationVisibilityChanged);

    for (std::size_t i = 0; i < getChildCount(); ++i)
    {
        Node3D* child = getChild(i)->castTo<Node3D>();
        if (child && child->mVisible) child->propagateVisibilityChanged();
    }
}

void Node3D::notifyDirty()
{
    if (mNotifyTransform && !mTransformChangedQueued && getTree())
        getTree()->queueTransformChanged(this);
}

} // namespace prisma3d
