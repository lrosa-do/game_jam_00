#pragma once

#include "prisma3d/scene/Node3D.h"

namespace prisma3d
{

class ParticleCollision3D : public Node3D
{
public:
    PRISMA3D_NODE_CLASS(ParticleCollision3D, Node3D);
    static void bindProperties();

    ~ParticleCollision3D() override;

    bool collide(const Math::Vec3& position, float particleSize, Math::Vec3& normal,
            float& depth) const;

protected:
    enum class Shape : std::uint8_t
    {
        Sphere,
        Box
    };

    explicit ParticleCollision3D(Shape shape);

    void notificationHandler(int what) override;

    Shape mShape;
    Math::Vec3 mExtents;
};

class ParticleCollisionSphere3D : public ParticleCollision3D
{
public:
    PRISMA3D_NODE_CLASS(ParticleCollisionSphere3D, ParticleCollision3D);
    static void bindProperties();

    ParticleCollisionSphere3D();

    void setRadius(float radius);
    float getRadius() const;
};

class ParticleCollisionBox3D : public ParticleCollision3D
{
public:
    PRISMA3D_NODE_CLASS(ParticleCollisionBox3D, ParticleCollision3D);
    static void bindProperties();

    ParticleCollisionBox3D();

    void setSize(const Math::Vec3& size);
    Math::Vec3 getSize() const;
};

} // namespace prisma3d
