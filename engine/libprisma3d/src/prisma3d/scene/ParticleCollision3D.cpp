#include "prisma3d/scene/ParticleCollision3D.h"

#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/SceneTree.h"

#include <math.h>

namespace prisma3d
{

namespace
{

constexpr float kEpsilon = 0.001f;

float signOf(float value)
{
    if (value > 0.0f) return 1.0f;
    if (value < 0.0f) return -1.0f;
    return 0.0f;
}

} // namespace

ParticleCollision3D::ParticleCollision3D(Shape shape)
        : Node3D(),
          mShape(shape),
          mExtents(1.0f, 1.0f, 1.0f)
{
}

ParticleCollision3D::~ParticleCollision3D()
{
}

void ParticleCollision3D::notificationHandler(int what)
{
    Node3D::notificationHandler(what);

    switch (what)
    {
        case kNotificationEnterTree:
            if (SceneTree* tree = getTree()) tree->addParticleCollider(this);
            break;
        case kNotificationExitTree:
            if (SceneTree* tree = getTree()) tree->removeParticleCollider(this);
            break;
        default:
            break;
    }
}

bool ParticleCollision3D::collide(const Math::Vec3& position, float particleSize,
        Math::Vec3& normal, float& depth) const
{
    const Math::Mat4 transform = getGlobalTransform();
    Math::Vec3 axisX(transform.col0.x, transform.col0.y, transform.col0.z);
    Math::Vec3 axisY(transform.col1.x, transform.col1.y, transform.col1.z);
    Math::Vec3 axisZ(transform.col2.x, transform.col2.y, transform.col2.z);
    const Math::Vec3 scale(axisX.Length(), axisY.Length(), axisZ.Length());
    if (scale.x <= 0.0f || scale.y <= 0.0f || scale.z <= 0.0f) return false;

    axisX = axisX * (1.0f / scale.x);
    axisY = (axisY - axisX * Math::Vec3::Dot(axisX, axisY)).Normalized();
    axisZ = (axisZ - axisX * Math::Vec3::Dot(axisX, axisZ) - axisY * Math::Vec3::Dot(axisY, axisZ))
                    .Normalized();

    const Math::Vec3 origin(transform.col3.x, transform.col3.y, transform.col3.z);
    const Math::Vec3 relative = position - origin;

    if (mShape == Shape::Sphere)
    {
        const float radius = mExtents.x * (scale.x + scale.y + scale.z) / 3.0f;
        const float length = relative.Length();
        const float distance = length - (particleSize + radius);
        if (distance > kEpsilon) return false;
        depth = -distance;
        normal = length > 0.0f ? relative * (1.0f / length) : Math::Vec3(0.0f, 1.0f, 0.0f);
        return true;
    }

    const Math::Vec3 local(Math::Vec3::Dot(relative, axisX), Math::Vec3::Dot(relative, axisY),
            Math::Vec3::Dot(relative, axisZ));
    const Math::Vec3 extents(mExtents.x * scale.x, mExtents.y * scale.y, mExtents.z * scale.z);
    const Math::Vec3 absolute(fabsf(local.x), fabsf(local.y), fabsf(local.z));
    const Math::Vec3 sign(signOf(local.x), signOf(local.y), signOf(local.z));

    Math::Vec3 localNormal(0.0f, 0.0f, 0.0f);
    if (absolute.x > extents.x || absolute.y > extents.y || absolute.z > extents.z)
    {
        const Math::Vec3 closest(fminf(absolute.x, extents.x), fminf(absolute.y, extents.y),
                fminf(absolute.z, extents.z));
        const Math::Vec3 outside = absolute - closest;
        const float distance = outside.Length() - particleSize;
        if (distance > kEpsilon) return false;
        localNormal = outside.Normalized();
        depth = -distance;
    }
    else
    {
        const Math::Vec3 axisLength = extents - absolute;
        if (axisLength.x < axisLength.y && axisLength.x < axisLength.z)
        {
            localNormal = Math::Vec3(1.0f, 0.0f, 0.0f);
            depth = axisLength.x + particleSize;
        }
        else if (axisLength.y < axisLength.x && axisLength.y < axisLength.z)
        {
            localNormal = Math::Vec3(0.0f, 1.0f, 0.0f);
            depth = axisLength.y + particleSize;
        }
        else
        {
            localNormal = Math::Vec3(0.0f, 0.0f, 1.0f);
            depth = axisLength.z + particleSize;
        }
    }

    localNormal =
            Math::Vec3(localNormal.x * sign.x, localNormal.y * sign.y, localNormal.z * sign.z);
    normal = axisX * localNormal.x + axisY * localNormal.y + axisZ * localNormal.z;
    return true;
}

ParticleCollisionSphere3D::ParticleCollisionSphere3D()
        : ParticleCollision3D(Shape::Sphere)
{
}

void ParticleCollisionSphere3D::setRadius(float radius)
{
    mExtents = Math::Vec3(radius, radius, radius);
}

float ParticleCollisionSphere3D::getRadius() const
{
    return mExtents.x;
}

ParticleCollisionBox3D::ParticleCollisionBox3D()
        : ParticleCollision3D(Shape::Box)
{
    setSize(Math::Vec3(2.0f, 2.0f, 2.0f));
}

void ParticleCollisionBox3D::setSize(const Math::Vec3& size)
{
    mExtents = size * 0.5f;
}

Math::Vec3 ParticleCollisionBox3D::getSize() const
{
    return mExtents * 2.0f;
}

void ParticleCollision3D::bindProperties()
{
}

void ParticleCollisionSphere3D::bindProperties()
{
    using DB = NodeClassDB;
    using P = ParticleCollisionSphere3D;
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "radius", 0.001, 1024.0, 0.001, true),
            &P::setRadius, &P::getRadius);
}

void ParticleCollisionBox3D::bindProperties()
{
    using DB = NodeClassDB;
    using P = ParticleCollisionBox3D;
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Vec3, "size", 0.001, 1024.0, 0.001, true),
            &P::setSize, &P::getSize);
}

} // namespace prisma3d
