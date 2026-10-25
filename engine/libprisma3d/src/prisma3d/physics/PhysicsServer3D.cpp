#include "prisma3d/physics/PhysicsServer3D.h"

#include "prisma3d/render/Batch.h"
#include "prisma3d/scene/Collision.h"
#include "prisma3d/terrain/Terrain.h"

#include <kinetix/core/DebugDraw3D.h>

#include "character/CharacterBody.h"
#include "character/CharacterRigidBody.h"
#include "character/Ragdoll.h"
#include "collision/CollisionShape.h"
#include "dynamics/DistanceJoint.h"
#include "dynamics/FixedJoint.h"
#include "dynamics/HingeJoint.h"
#include "dynamics/MotorcycleController.h"
#include "dynamics/MouseJoint.h"
#include "dynamics/PhysicsWorld.h"
#include "dynamics/PistonJoint.h"
#include "dynamics/PointJoint.h"
#include "dynamics/RaycastVehicle.h"
#include "dynamics/RigidBody.h"
#include "dynamics/SliderJoint.h"
#include "dynamics/UniversalJoint.h"
#include "dynamics/WheelJoint.h"
#include "geometry/ConvexHullComputer.h"
#include "geometry/VoronoiShatter.h"
#include "ground/GroundVehicle.h"
#include "ground/GroundVehiclePresets.h"
#include "softbody/SoftBody.h"

#include "platform.h"

#include <ct/sort.hpp>

#include <math.h>

namespace prisma3d
{

namespace kp = kinetix::Physics;

namespace
{

constexpr std::uint32_t kNoLibId = 0xFFFFFFFFu;

Math::Vec3 vec3Of(const Math::Vec4& v)
{
    return Math::Vec3(v.x, v.y, v.z);
}

Math::Vec3 absVec(const Math::Vec3& v)
{
    return Math::Vec3(fabsf(v.x), fabsf(v.y), fabsf(v.z));
}

float maxComponent(const Math::Vec3& v)
{
    return fmaxf(v.x, fmaxf(v.y, v.z));
}

Math::Quaternion rotationOf(const Math::Mat4& m)
{
    const Math::Vec3 x = vec3Of(m.col0).NormalizedSafe();
    const Math::Vec3 y = vec3Of(m.col1).NormalizedSafe();
    const Math::Vec3 z = Math::Vec3::Cross(x, y).NormalizedSafe();
    const Math::Vec3 yy = Math::Vec3::Cross(z, x);
    return Math::Quaternion::FromMat3(Math::Mat3(x, yy, z)).Normalized();
}

Math::Mat4 composeFrame(const Math::Quaternion& rotation, const Math::Vec3& position)
{
    return Math::Mat4(rotation.ToMat3(), position);
}

kinetix::Math::quat toKinetix(const Math::Quaternion& q)
{
    return kinetix::Math::quat(q);
}

Math::Quaternion fromKinetix(const kinetix::Math::quat& q)
{
    return static_cast<const Math::Quaternion&>(q);
}

float dampRetention(float damp)
{
    return damp > 0.0f ? expf(-damp) : 1.0f;
}

} // namespace

struct PhysicsShapeInstance
{
    Math::Vec3 scale = Math::Vec3(1.0f, 1.0f, 1.0f);
    Math::Vec3 center = Math::Vec3(0.0f, 0.0f, 0.0f);
    kp::CollisionShape* shape = nullptr;
    mutable kp::TrimeshShape* proxy = nullptr;
    int uses = 0;
};

struct PhysicsShapeTag
{
    PhysicsShapeType type = PhysicsShapeType::Box;
    Math::Vec3 halfExtents = Math::Vec3(0.5f, 0.5f, 0.5f);
    float radius = 0.5f;
    float height = 2.0f;
    Math::Vec3 planeNormal = Math::Vec3(0.0f, 1.0f, 0.0f);
    float planeDistance = 0.0f;
    ct::Vector<Math::Vec3> vertices;
    ct::Vector<std::uint32_t> indices;
    int mapWidth = 2;
    int mapDepth = 2;
    ct::Vector<float> heights;
    Math::Vec3 mapScale = Math::Vec3(1.0f, 1.0f, 1.0f);
    Math::Vec3 mapOffset = Math::Vec3(0.0f, 0.0f, 0.0f);
    ct::Vector<Math::Vec3> hullVertices;
    ct::Vector<kinetix::Geometry::ConvexHullComputer::Edge> hullEdges;
    ct::Vector<int> hullFaces;
    Math::Vec3 hullCenter = Math::Vec3(0.0f, 0.0f, 0.0f);
    ct::Vector<PhysicsShapeInstance*> instances;
    bool dying = false;
};

struct PhysicsBodyTag
{
    kp::RigidBody* body = nullptr;
    std::uint32_t libId = kNoLibId;
    PhysicsBodyMode mode = PhysicsBodyMode::Static;
    PhysicsShapeId shape;
    PhysicsShapeInstance* instance = nullptr;
    Math::Vec3 offsetPosition = Math::Vec3(0.0f, 0.0f, 0.0f);
    Math::Quaternion offsetRotation = Math::Quaternion::Identity();
    Math::Vec3 localScale = Math::Vec3(1.0f, 1.0f, 1.0f);
    bool inWorld = true;
    bool enabled = true;
    bool dying = false;
    bool hasTarget = false;
    bool everPlaced = false;
    bool wasAwake = true;
    bool lockRotation = false;
    bool canSleep = true;
    bool interpolated = true;
    bool monitor = false;
    bool impulseMonitor = false;
    int maxContacts = 0;
    std::uint32_t layer = 1;
    std::uint32_t mask = 1;
    float friction = 1.0f;
    float bounce = 0.0f;
    float mass = 1.0f;
    Math::Vec3 customInertia = Math::Vec3(0.0f, 0.0f, 0.0f);
    float linearDamp = 0.0f;
    float angularDamp = 0.0f;
    PhysicsDampMode linearMode = PhysicsDampMode::Combine;
    PhysicsDampMode angularMode = PhysicsDampMode::Combine;
    Math::Vec3 constantForce = Math::Vec3(0.0f, 0.0f, 0.0f);
    Math::Vec3 constantTorque = Math::Vec3(0.0f, 0.0f, 0.0f);
    Math::Vec3 targetPosition = Math::Vec3(0.0f, 0.0f, 0.0f);
    Math::Quaternion targetRotation = Math::Quaternion::Identity();
    Math::Vec3 previousPosition = Math::Vec3(0.0f, 0.0f, 0.0f);
    Math::Quaternion previousRotation = Math::Quaternion::Identity();
    PhysicsBodyListener* listener = nullptr;
    void* userData = nullptr;
    ct::Vector<std::uint32_t> touching;
    ct::Vector<PhysicsContactImpulse> impulses;
};

struct PhysicsJointTag
{
    kp::Joint* joint = nullptr;
    PhysicsJointType type = PhysicsJointType::Pin;
    PhysicsBodyId bodyA;
    PhysicsBodyId bodyB;
    bool dying = false;
};

struct PhysicsException
{
    std::uint32_t a = 0;
    std::uint32_t b = 0;
};

struct PhysicsVehicleTag
{
    kp::RaycastVehicle* vehicle = nullptr;
    kp::MotorcycleController* motorcycle = nullptr;
    PhysicsBodyId chassis;
    bool dying = false;
};

struct PhysicsGroundVehicleTag
{
    kp::GroundVehicle* vehicle = nullptr;
    PhysicsBodyId chassis;
    ct::Vector<Math::Mat4> wheelLocal;
    kp::EngineParameters engine;
    kp::GearboxParameters gearbox;
    kp::ClutchParameters clutch;
    ct::Vector<float> tyreFriction;
    ct::Vector<float> handbrakeTorque;
    float restHeight = 0.0f;
    bool dying = false;
};

struct PhysicsCharacterTag
{
    kp::CharacterBody* body = nullptr;
    PhysicsCharacterParameters parameters;
    PhysicsBodyId ignored;
    ct::Vector<PhysicsCharacterHit> hits;
    bool dying = false;
};

struct PhysicsCharacterRigidTag
{
    kp::CharacterRigidBody* controller = nullptr;
    PhysicsBodyId body;
    PhysicsCharacterRigidParameters parameters;
    bool dying = false;
};

struct PhysicsSoftBodyTag
{
    kp::SoftBody* body = nullptr;
    int substeps = 4;
    bool enabled = true;
    bool dying = false;
};

struct PhysicsRagdollTag
{
    kp::Ragdoll* ragdoll = nullptr;
    bool counted = false;
    bool dying = false;
};

struct PhysicsRawContactEvent
{
    std::uint32_t a = 0;
    std::uint32_t b = 0;
    bool entered = false;
    Math::Vec3 point = Math::Vec3(0.0f, 0.0f, 0.0f);
    Math::Vec3 normal = Math::Vec3(0.0f, 1.0f, 0.0f);
    float depth = 0.0f;
};

struct PhysicsServer3D::Impl
{
    class WorldSweep;

    explicit Impl(PhysicsServer3D* server)
            : anchor(new kp::RigidBody()),
              owner(server)
    {
        anchor->setBodyType(kp::BodyType::Static);
        world.setEventCallback(&Impl::onContactEvent, this);
        world.setStepCallback(&Impl::onStepEnd, this);
    }

    ~Impl()
    {
        for (std::size_t i = 0; i < joints.size(); ++i) releaseJoint(joints.items()[i]);
        for (std::size_t i = 0; i < vehicles.size(); ++i)
        {
            delete vehicles.items()[i].motorcycle;
            delete vehicles.items()[i].vehicle;
        }
        for (std::size_t i = 0; i < groundVehicles.size(); ++i)
            delete groundVehicles.items()[i].vehicle;
        for (std::size_t i = 0; i < characterRigids.size(); ++i)
            delete characterRigids.items()[i].controller;
        for (std::size_t i = 0; i < characters.size(); ++i) delete characters.items()[i].body;
        for (std::size_t i = 0; i < softBodies.size(); ++i) delete softBodies.items()[i].body;
        for (std::size_t i = 0; i < ragdolls.size(); ++i)
        {
            ragdolls.items()[i].ragdoll->removeFromWorld();
            delete ragdolls.items()[i].ragdoll;
        }
        for (std::size_t i = 0; i < bodies.size(); ++i)
        {
            PhysicsBodyTag& record = bodies.items()[i];
            if (record.libId != kNoLibId) world.removeBody(record.libId);
            releaseInstance(record);
            delete record.body;
        }
        for (std::size_t i = 0; i < shapes.size(); ++i) destroyShapeRecord(shapes.items()[i]);
        delete anchor;
    }

    kp::PhysicsWorld world;
    kp::RigidBody* anchor;
    PhysicsServer3D* owner;
    ct::SlotMap32<PhysicsShapeTag> shapes;
    ct::SlotMap32<PhysicsBodyTag> bodies;
    ct::SlotMap32<PhysicsJointTag> joints;
    ct::Vector<PhysicsException> exceptions;
    ct::SlotMap32<PhysicsVehicleTag> vehicles;
    ct::SlotMap32<PhysicsGroundVehicleTag> groundVehicles;
    ct::SlotMap32<PhysicsRagdollTag> ragdolls;
    ct::SlotMap32<PhysicsCharacterTag> characters;
    ct::Vector<PhysicsCharacterId> characterFreeQueue;
    ct::SlotMap32<PhysicsCharacterRigidTag> characterRigids;
    ct::Vector<PhysicsCharacterRigidId> characterRigidFreeQueue;
    ct::SlotMap32<PhysicsSoftBodyTag> softBodies;
    ct::Vector<PhysicsSoftBodyId> softBodyFreeQueue;
    ct::Vector<PhysicsRagdollId> ragdollFreeQueue;
    ct::Vector<std::uint32_t> libToBody;
    ct::Vector<PhysicsRawContactEvent> events;
    ct::Vector<PhysicsBodyId> freeQueue;
    ct::Vector<PhysicsJointId> jointFreeQueue;
    ct::Vector<PhysicsVehicleId> vehicleFreeQueue;
    ct::Vector<PhysicsGroundVehicleId> groundVehicleFreeQueue;
    Math::Vec3 gravity = Math::Vec3(0.0f, -9.8f, 0.0f);
    float defaultLinearDamp = 0.1f;
    float defaultAngularDamp = 0.1f;
    std::uint32_t velocityIterations = 8;
    std::uint32_t positionIterations = 3;
    std::uint32_t substeps = 2;
    std::uint32_t ragdollsActive = 0;
    std::uint32_t impulseMonitors = 0;
    float contactMargin = 0.04f;
    bool active = true;
    bool locked = false;
    float lastStep = 1.0f / 60.0f;
    PhysicsStats stats;
    double stepAccumulator = 0.0;

    static void onContactEvent(const kp::ContactEventInfo& info, void* user)
    {
        if (info.event == kp::ContactEvent::Stay) return;
        Impl* self = static_cast<Impl*>(user);
        PhysicsRawContactEvent event;
        event.a = info.bodyA;
        event.b = info.bodyB;
        event.entered = info.event == kp::ContactEvent::Enter;
        event.point = info.point;
        event.normal = info.normal;
        event.depth = info.penetration;
        self->events.push_back(event);
    }

    static void onStepEnd(float step, void* user)
    {
        Impl* self = static_cast<Impl*>(user);
        for (std::size_t i = 0; i < self->vehicles.size(); ++i)
        {
            PhysicsVehicleTag& record = self->vehicles.items()[i];
            if (record.dying || !record.vehicle) continue;
            if (record.motorcycle) record.motorcycle->preUpdate(step, self->gravity);
            record.vehicle->update(step);
            if (record.motorcycle) record.motorcycle->postUpdate(step);
        }
    }

    PhysicsBodyTag* body(PhysicsBodyId id)
    {
        PhysicsBodyTag* record = bodies.get(id);
        return record && !record->dying ? record : nullptr;
    }

    const PhysicsBodyTag* body(PhysicsBodyId id) const
    {
        const PhysicsBodyTag* record = bodies.get(id);
        return record && !record->dying ? record : nullptr;
    }

    PhysicsShapeTag* shape(PhysicsShapeId id)
    {
        PhysicsShapeTag* record = shapes.get(id);
        return record && !record->dying ? record : nullptr;
    }

    const PhysicsShapeTag* shape(PhysicsShapeId id) const
    {
        const PhysicsShapeTag* record = shapes.get(id);
        return record && !record->dying ? record : nullptr;
    }

    PhysicsBodyId handleOfLib(std::uint32_t libId) const
    {
        if (libId >= libToBody.size() || libToBody[libId] == 0) return PhysicsBodyId();
        return PhysicsBodyId::from_bits(libToBody[libId]);
    }

    float totalLinearDamp(const PhysicsBodyTag& record) const
    {
        return record.linearMode == PhysicsDampMode::Combine ? defaultLinearDamp + record.linearDamp
                                                             : record.linearDamp;
    }

    float totalAngularDamp(const PhysicsBodyTag& record) const
    {
        return record.angularMode == PhysicsDampMode::Combine
                       ? defaultAngularDamp + record.angularDamp
                       : record.angularDamp;
    }

    void applyDamping(PhysicsBodyTag& record)
    {
        record.body->setDamping(dampRetention(totalLinearDamp(record)),
                dampRetention(totalAngularDamp(record)));
    }

    void applyDampingAll()
    {
        for (std::size_t i = 0; i < bodies.size(); ++i) applyDamping(bodies.items()[i]);
    }

    static void destroyInstance(PhysicsShapeInstance* instance)
    {
        delete instance->proxy;
        delete instance->shape;
        delete instance;
    }

    void destroyShapeRecord(PhysicsShapeTag& record)
    {
        for (std::size_t i = 0; i < record.instances.size(); ++i)
            destroyInstance(record.instances[i]);
        record.instances.clear();
    }

    PhysicsShapeInstance* buildInstance(const PhysicsShapeTag& record, const Math::Vec3& scale)
    {
        const Math::Vec3 s = absVec(scale);
        kp::CollisionShape* built = nullptr;
        switch (record.type)
        {
            case PhysicsShapeType::Box:
                built = new kp::BoxShape(Math::Vec3(record.halfExtents.x * s.x,
                        record.halfExtents.y * s.y, record.halfExtents.z * s.z));
                break;
            case PhysicsShapeType::Sphere:
                built = new kp::SphereShape(record.radius * maxComponent(s));
                break;
            case PhysicsShapeType::Capsule:
            {
                const float radius = record.radius * fmaxf(s.x, s.z);
                const float total = fmaxf(record.height * s.y, radius * 2.0f);
                built = new kp::CapsuleShape(radius, total * 0.5f - radius);
                break;
            }
            case PhysicsShapeType::Cylinder:
            {
                const float radius = record.radius * fmaxf(s.x, s.z);
                built = new kp::CylinderShape(radius, record.height * s.y * 0.5f);
                break;
            }
            case PhysicsShapeType::WorldBoundary:
                built = new kp::PlaneShape(record.planeNormal, record.planeDistance);
                break;
            case PhysicsShapeType::ConcavePolygon:
            {
                if (record.indices.size() < 3) return nullptr;
                ct::Vector<Math::Vec3> scaled;
                scaled.reserve(record.vertices.size());
                for (std::size_t i = 0; i < record.vertices.size(); ++i)
                    scaled.push_back(record.vertices[i] * scale);
                {
                    kp::TrimeshShape* mesh = new kp::TrimeshShape(scaled.data(),
                            static_cast<std::uint32_t>(scaled.size()), record.indices.data(),
                            static_cast<std::uint32_t>(record.indices.size()));
                    // Wound consistently: the front the renderer shows is the only side with a
                    // surface.
                    ct::Vector<std::uint32_t> wrongWay;
                    mesh->windingErrors(wrongWay);
                    mesh->setOneSided(wrongWay.empty());
                    built = mesh;
                }
                break;
            }
            case PhysicsShapeType::ConvexHull:
                break;
            case PhysicsShapeType::HeightMap:
            {
                const int w = record.mapWidth;
                const int d = record.mapDepth;
                if (w < 2 || d < 2 || record.heights.size() < static_cast<std::size_t>(w * d))
                    return nullptr;
                const Math::Vec3 total = record.mapScale * scale;
                ct::Vector<Math::Vec3> grid;
                grid.reserve(static_cast<std::size_t>(w * d));
                for (int j = 0; j < d; ++j)
                    for (int i = 0; i < w; ++i)
                        grid.push_back(Math::Vec3((static_cast<float>(i) - (w - 1) * 0.5f) *
                                                          total.x,
                                record.heights[static_cast<std::size_t>(j * w + i)] * total.y,
                                (static_cast<float>(j) - (d - 1) * 0.5f) * total.z));
                ct::Vector<std::uint32_t> triangles;
                triangles.reserve(static_cast<std::size_t>((w - 1) * (d - 1) * 6));
                for (int j = 0; j < d - 1; ++j)
                    for (int i = 0; i < w - 1; ++i)
                    {
                        const std::uint32_t v00 = static_cast<std::uint32_t>(j * w + i);
                        const std::uint32_t v10 = v00 + 1;
                        const std::uint32_t v01 = v00 + static_cast<std::uint32_t>(w);
                        const std::uint32_t v11 = v01 + 1;
                        triangles.push_back(v00);
                        triangles.push_back(v01);
                        triangles.push_back(v10);
                        triangles.push_back(v10);
                        triangles.push_back(v01);
                        triangles.push_back(v11);
                    }
                {
                    kp::TrimeshShape* mesh = new kp::TrimeshShape(grid.data(),
                            static_cast<std::uint32_t>(grid.size()), triangles.data(),
                            static_cast<std::uint32_t>(triangles.size()));
                    mesh->setOneSided(true);
                    built = mesh;
                }
                break;
            }
        }
        Math::Vec3 center(0.0f, 0.0f, 0.0f);
        if (record.type == PhysicsShapeType::ConvexHull)
        {
            if (record.hullVertices.size() < 4) return nullptr;
            ct::Vector<Math::Vec3> scaled;
            scaled.reserve(record.hullVertices.size());
            for (std::size_t i = 0; i < record.hullVertices.size(); ++i)
                scaled.push_back(record.hullVertices[i] * s);
            built = new kp::ConvexHullShape(scaled.data(),
                    static_cast<std::uint32_t>(scaled.size()), record.hullEdges.data(),
                    static_cast<std::uint32_t>(record.hullEdges.size()), record.hullFaces.data(),
                    static_cast<std::uint32_t>(record.hullFaces.size()));
            center = record.hullCenter * s;
        }
        if (!built) return nullptr;
        PhysicsShapeInstance* instance = new PhysicsShapeInstance();
        instance->scale = scale;
        instance->center = center;
        instance->shape = built;
        return instance;
    }

    PhysicsShapeInstance* acquireInstance(PhysicsShapeTag& record, const Math::Vec3& scale)
    {
        for (std::size_t i = 0; i < record.instances.size(); ++i)
        {
            PhysicsShapeInstance* instance = record.instances[i];
            if (instance->scale == scale)
            {
                ++instance->uses;
                return instance;
            }
        }
        PhysicsShapeInstance* instance = buildInstance(record, scale);
        if (!instance) return nullptr;
        instance->uses = 1;
        record.instances.push_back(instance);
        return instance;
    }

    void releaseInstance(PhysicsBodyTag& record)
    {
        if (!record.instance) return;
        PhysicsShapeInstance* instance = record.instance;
        record.instance = nullptr;
        if (--instance->uses > 0) return;
        PhysicsShapeTag* owner = shapes.get(record.shape);
        if (owner)
        {
            for (std::size_t i = 0; i < owner->instances.size(); ++i)
                if (owner->instances[i] == instance)
                {
                    owner->instances.erase(owner->instances.begin() + i);
                    break;
                }
        }
        destroyInstance(instance);
    }

    bool allowedInWorld(const PhysicsBodyTag& record) const
    {
        if (!record.inWorld || !record.enabled || !record.instance) return false;
        if (record.mode == PhysicsBodyMode::Rigid)
        {
            const kp::ShapeType type = record.instance->shape->type();
            if (type == kp::ShapeType::Trimesh || type == kp::ShapeType::Plane) return false;
        }
        return true;
    }

    void removeFromWorld(PhysicsBodyTag& record)
    {
        if (record.libId == kNoLibId) return;
        if (record.libId < libToBody.size()) libToBody[record.libId] = 0;
        world.removeBody(record.libId);
        record.libId = kNoLibId;
        if (record.mode == PhysicsBodyMode::Static) world.markStaticBroadphaseDirty();
    }

    void syncWorld(PhysicsBodyId id, PhysicsBodyTag& record)
    {
        const bool wanted = allowedInWorld(record);
        if (!wanted)
        {
            removeFromWorld(record);
            return;
        }
        if (record.libId == kNoLibId)
        {
            kp::BodyEntry entry;
            entry.body = record.body;
            entry.shape = record.instance->shape;
            entry.filter.group = record.layer;
            entry.filter.mask = record.mask;
            entry.friction = record.friction;
            entry.restitution = record.bounce;
            entry.enabled = true;
            record.libId = world.addBody(entry);
            if (record.libId == 0xFFFFFFFFu) return;
            if (record.libId >= libToBody.size()) libToBody.resize(record.libId + 1, 0);
            libToBody[record.libId] = id.bits();
            for (std::size_t g = 0; g < groundVehicles.size(); ++g)
            {
                PhysicsGroundVehicleTag& vehicle = groundVehicles.items()[g];
                if (!vehicle.dying && vehicle.vehicle && vehicle.chassis == id)
                    vehicle.vehicle->setBody(record.body, record.libId);
            }
            for (std::size_t e = 0; e < exceptions.size(); ++e)
            {
                const PhysicsException& pair = exceptions[e];
                if (pair.a != id.bits() && pair.b != id.bits()) continue;
                const PhysicsBodyTag* other =
                        bodies.get(PhysicsBodyId::from_bits(pair.a == id.bits() ? pair.b : pair.a));
                if (other && other->libId != kNoLibId)
                    world.setPairIgnored(record.libId, other->libId, true);
            }
            for (std::size_t i = 0; i < joints.size(); ++i)
            {
                PhysicsJointTag& joint = joints.items()[i];
                if (!joint.dying && (joint.bodyA == id || joint.bodyB == id))
                    world.addJoint(joint.joint);
            }
        }
        else
        {
            kp::BodyEntry* entry = world.body(record.libId);
            if (entry)
            {
                entry->shape = record.instance->shape;
                entry->filter.group = record.layer;
                entry->filter.mask = record.mask;
                entry->friction = record.friction;
                entry->restitution = record.bounce;
            }
        }
        if (record.mode == PhysicsBodyMode::Static) world.markStaticBroadphaseDirty();
    }

    void applyMass(PhysicsBodyTag& record)
    {
        record.body->setMass(fmaxf(record.mass, 0.001f));
        if (record.instance)
        {
            if (record.customInertia.x > 0.0f && record.customInertia.y > 0.0f &&
                    record.customInertia.z > 0.0f)
                record.body->setInertiaTensor(Math::Mat3(Math::Vec3(record.customInertia.x, 0, 0),
                        Math::Vec3(0, record.customInertia.y, 0),
                        Math::Vec3(0, 0, record.customInertia.z)));
            else
                record.body->setInertiaTensor(record.instance->shape->inertia(record.body->mass()));
        }
        if (record.lockRotation) record.body->setInverseInertiaTensor(Math::Mat3(0.0f));
    }

    Math::Vec3 bodyOffset(const PhysicsBodyTag& record) const
    {
        if (!record.instance) return record.offsetPosition;
        return record.offsetPosition + record.offsetRotation * record.instance->center;
    }

    Math::Vec3 bodyPosition(const PhysicsBodyTag& record) const
    {
        return record.body->position();
    }

    Math::Quaternion bodyRotation(const PhysicsBodyTag& record) const
    {
        return fromKinetix(record.body->orientation());
    }

    Math::Mat4 frameOf(const PhysicsBodyTag& record, const Math::Vec3& position,
            const Math::Quaternion& rotation) const
    {
        const Math::Quaternion frameRotation = rotation * record.offsetRotation.Conjugate();
        const Math::Vec3 framePosition = position - frameRotation * bodyOffset(record);
        return composeFrame(frameRotation, framePosition);
    }

    void libPoseOf(const PhysicsBodyTag& record, const Math::Mat4& frame, Math::Vec3& position,
            Math::Quaternion& rotation) const
    {
        const Math::Quaternion frameRotation = rotationOf(frame);
        position = vec3Of(frame.col3) + frameRotation * bodyOffset(record);
        rotation = (frameRotation * record.offsetRotation).Normalized();
    }

    void releaseJoint(PhysicsJointTag& record)
    {
        if (!record.joint) return;
        world.removeJoint(record.joint);
        delete record.joint;
        record.joint = nullptr;
    }

    void destroyJointNow(PhysicsJointId id)
    {
        PhysicsJointTag* record = joints.get(id);
        if (!record) return;
        releaseJoint(*record);
        joints.erase(id);
    }

    void destroyGroundVehicleNow(PhysicsGroundVehicleId id)
    {
        PhysicsGroundVehicleTag* record = groundVehicles.get(id);
        if (!record) return;
        delete record->vehicle;
        groundVehicles.erase(id);
    }

    void applyGroundVehicles(float step)
    {
        for (std::size_t i = 0; i < groundVehicles.size(); ++i)
        {
            PhysicsGroundVehicleTag& record = groundVehicles.items()[i];
            if (record.dying || !record.vehicle) continue;
            const PhysicsBodyTag* chassis = body(record.chassis);
            if (!chassis || chassis->libId == kNoLibId || !chassis->enabled ||
                    chassis->mode != PhysicsBodyMode::Rigid)
                continue;
            record.vehicle->applyForces(step);
            const kp::RigidBody* rigid = chassis->body;
            const Math::Mat4 toChassis =
                    (Math::Mat4::Translation(rigid->position()) * rigid->orientation().ToMat4())
                            .InverseAffine();
            const std::uint32_t count = record.vehicle->wheelCount();
            record.wheelLocal.resize(count, Math::Mat4::Identity());
            for (std::uint32_t w = 0; w < count; ++w)
                record.wheelLocal[w] = toChassis * record.vehicle->wheel(w).worldTransform;
        }
    }

    void destroyVehicleNow(PhysicsVehicleId id)
    {
        PhysicsVehicleTag* record = vehicles.get(id);
        if (!record) return;
        delete record->motorcycle;
        delete record->vehicle;
        vehicles.erase(id);
    }

    void destroyCharacterRigidNow(PhysicsCharacterRigidId id)
    {
        PhysicsCharacterRigidTag* record = characterRigids.get(id);
        if (!record) return;
        delete record->controller;
        characterRigids.erase(id);
    }

    void destroyCharacterNow(PhysicsCharacterId id)
    {
        PhysicsCharacterTag* record = characters.get(id);
        if (!record) return;
        delete record->body;
        characters.erase(id);
    }

    void destroySoftBodyNow(PhysicsSoftBodyId id)
    {
        PhysicsSoftBodyTag* record = softBodies.get(id);
        if (!record) return;
        delete record->body;
        softBodies.erase(id);
    }

    void applySolverSettings()
    {
        std::uint32_t velocity = velocityIterations;
        std::uint32_t position = positionIterations;
        if (ragdollsActive > 0)
        {
            if (velocity < kp::Ragdoll::kVelocityIterations)
                velocity = kp::Ragdoll::kVelocityIterations;
            if (position < kp::Ragdoll::kPositionIterations)
                position = kp::Ragdoll::kPositionIterations;
        }
        kp::ContactSolverSettings settings = kp::ContactSolverSettings();
        settings.velocityIterations = velocity;
        settings.positionIterations = position;
        world.setSolverSettings(settings);
    }

    void setRagdollCounted(PhysicsRagdollTag& record, bool counted)
    {
        if (record.counted == counted) return;
        record.counted = counted;
        if (counted) ++ragdollsActive;
        else
            --ragdollsActive;
        applySolverSettings();
    }

    void destroyRagdollNow(PhysicsRagdollId id)
    {
        PhysicsRagdollTag* record = ragdolls.get(id);
        if (!record) return;
        record->ragdoll->removeFromWorld();
        setRagdollCounted(*record, false);
        delete record->ragdoll;
        ragdolls.erase(id);
    }

    void detachContacts(PhysicsBodyId id, PhysicsBodyTag& record, bool notifyPartners)
    {
        ct::Vector<std::uint32_t> partners;
        partners.swap(record.touching);
        if (!notifyPartners) return;
        for (std::size_t i = 0; i < partners.size(); ++i)
        {
            const PhysicsBodyId otherId = PhysicsBodyId::from_bits(partners[i]);
            PhysicsBodyTag* other = body(otherId);
            if (!other) continue;
            for (std::size_t j = 0; j < other->touching.size(); ++j)
                if (other->touching[j] == id.bits())
                {
                    other->touching.erase(other->touching.begin() + j);
                    break;
                }
            if (other->listener)
            {
                PhysicsContact contact;
                contact.other = id;
                other->listener->physicsContactEnded(contact);
            }
        }
    }

    void collectImpulses()
    {
        const ct::Vector<kp::Contact>& contacts = world.contacts();
        for (std::size_t n = 0; n < contacts.size(); ++n)
        {
            const kp::Contact& contact = contacts[n];
            for (int side = 0; side < 2; ++side)
            {
                const PhysicsBodyId self = handleOfLib(side == 0 ? contact.idA : contact.idB);
                PhysicsBodyTag* record = self.valid() ? body(self) : nullptr;
                if (!record || !record->impulseMonitor) continue;
                const PhysicsBodyId other = handleOfLib(side == 0 ? contact.idB : contact.idA);
                const Math::Vec3 normal =
                        side == 0 ? contact.manifold.normal : -contact.manifold.normal;
                for (std::uint32_t k = 0; k < contact.manifold.count; ++k)
                {
                    const kp::ContactPoint& point = contact.manifold.points[k];
                    if (point.normalImpulse <= 0.0f) continue;
                    PhysicsContactImpulse entry;
                    entry.other = other;
                    entry.point = point.position;
                    entry.normal = normal;
                    entry.impulse = point.normalImpulse;
                    record->impulses.push_back(entry);
                }
            }
        }
    }

    void destroyBodyNow(PhysicsBodyId id)
    {
        PhysicsBodyTag* record = bodies.get(id);
        if (!record) return;
        if (record->impulseMonitor) --impulseMonitors;
        for (std::size_t i = 0; i < exceptions.size();)
        {
            if (exceptions[i].a == id.bits() || exceptions[i].b == id.bits())
                exceptions.erase(exceptions.begin() + i);
            else
                ++i;
        }
        for (std::size_t i = 0; i < joints.size();)
        {
            PhysicsJointTag& joint = joints.items()[i];
            if (joint.bodyA == id || joint.bodyB == id) destroyJointNow(joints.handle_at(i));
            else
                ++i;
        }
        for (std::size_t i = 0; i < vehicles.size();)
        {
            if (vehicles.items()[i].chassis == id) destroyVehicleNow(vehicles.handle_at(i));
            else
                ++i;
        }
        for (std::size_t i = 0; i < groundVehicles.size();)
        {
            if (groundVehicles.items()[i].chassis == id)
                destroyGroundVehicleNow(groundVehicles.handle_at(i));
            else
                ++i;
        }
        record = bodies.get(id);
        record->listener = nullptr;
        record->dying = true;
        detachContacts(id, *record, true);
        record = bodies.get(id);
        removeFromWorld(*record);
        releaseInstance(*record);
        delete record->body;
        record->body = nullptr;
        bodies.erase(id);
    }

    void flushQueues()
    {
        while (!characterRigidFreeQueue.empty())
        {
            const PhysicsCharacterRigidId id = characterRigidFreeQueue.back();
            characterRigidFreeQueue.pop_back();
            destroyCharacterRigidNow(id);
        }
        while (!characterFreeQueue.empty())
        {
            const PhysicsCharacterId id = characterFreeQueue.back();
            characterFreeQueue.pop_back();
            destroyCharacterNow(id);
        }
        while (!softBodyFreeQueue.empty())
        {
            const PhysicsSoftBodyId id = softBodyFreeQueue.back();
            softBodyFreeQueue.pop_back();
            destroySoftBodyNow(id);
        }
        while (!ragdollFreeQueue.empty())
        {
            const PhysicsRagdollId id = ragdollFreeQueue.back();
            ragdollFreeQueue.pop_back();
            destroyRagdollNow(id);
        }
        while (!vehicleFreeQueue.empty())
        {
            const PhysicsVehicleId id = vehicleFreeQueue.back();
            vehicleFreeQueue.pop_back();
            destroyVehicleNow(id);
        }
        while (!groundVehicleFreeQueue.empty())
        {
            const PhysicsGroundVehicleId id = groundVehicleFreeQueue.back();
            groundVehicleFreeQueue.pop_back();
            destroyGroundVehicleNow(id);
        }
        while (!jointFreeQueue.empty())
        {
            const PhysicsJointId id = jointFreeQueue.back();
            jointFreeQueue.pop_back();
            destroyJointNow(id);
        }
        while (!freeQueue.empty())
        {
            const PhysicsBodyId id = freeQueue.back();
            freeQueue.pop_back();
            destroyBodyNow(id);
        }
    }

    void dispatchContacts()
    {
        ct::Vector<PhysicsRawContactEvent> batch;
        batch.swap(events);
        stats.contactEvents = static_cast<std::uint32_t>(batch.size());
        for (std::size_t n = 0; n < batch.size(); ++n)
        {
            const PhysicsRawContactEvent event = batch[n];
            const PhysicsBodyId idA = handleOfLib(event.a);
            const PhysicsBodyId idB = handleOfLib(event.b);
            if (!idA.valid() || !idB.valid())
            {
                continue;
            }
            if (!body(idA) || !body(idB)) continue;

            if (owner->contactReported)
            {
                PhysicsContactEvent report;
                report.bodyA = idA;
                report.bodyB = idB;
                report.entered = event.entered;
                report.point = event.point;
                report.normal = event.normal;
                report.depth = event.depth;
                owner->contactReported(report);
            }

            for (int side = 0; side < 2; ++side)
            {
                const PhysicsBodyId self = side == 0 ? idA : idB;
                const PhysicsBodyId other = side == 0 ? idB : idA;
                PhysicsBodyTag* record = body(self);
                if (!record || !record->monitor || record->maxContacts <= 0) continue;
                PhysicsContact contact;
                contact.other = other;
                contact.point = event.point;
                contact.normal = side == 0 ? event.normal : -event.normal;
                contact.depth = event.depth;
                if (event.entered)
                {
                    if (static_cast<int>(record->touching.size()) >= record->maxContacts) continue;
                    record->touching.push_back(other.bits());
                    if (record->listener) record->listener->physicsContactStarted(contact);
                }
                else
                {
                    bool found = false;
                    for (std::size_t j = 0; j < record->touching.size(); ++j)
                        if (record->touching[j] == other.bits())
                        {
                            record->touching.erase(record->touching.begin() + j);
                            found = true;
                            break;
                        }
                    if (found && record->listener) record->listener->physicsContactEnded(contact);
                }
            }
        }
    }
};

PhysicsServer3D* PhysicsServer3D::sSingleton = nullptr;

PhysicsServer3D::PhysicsServer3D()
        : contactReported(),
          mImpl(new Impl(this))
{
    if (!sSingleton) sSingleton = this;
}

PhysicsServer3D::~PhysicsServer3D()
{
    if (sSingleton == this) sSingleton = nullptr;
    delete mImpl;
}

PhysicsServer3D* PhysicsServer3D::singleton()
{
    return sSingleton;
}

void PhysicsServer3D::step(double delta)
{
    Impl& s = *mImpl;
    if (s.locked || !s.active) return;
    const float dt = static_cast<float>(delta);
    if (!(dt > 0.0f)) return;

    const std::uint64_t start = time_nanos();
    s.locked = true;
    s.lastStep = dt;

    const std::size_t count = s.bodies.size();
    for (std::size_t i = 0; i < count; ++i)
    {
        PhysicsBodyTag& record = s.bodies.items()[i];
        if (record.libId == kNoLibId) continue;
        record.previousPosition = s.bodyPosition(record);
        record.previousRotation = s.bodyRotation(record);
        if (record.mode == PhysicsBodyMode::Kinematic)
        {
            if (record.hasTarget)
            {
                record.body->setVelocity((record.targetPosition - record.body->position()) / dt);
                Math::Quaternion change =
                        record.targetRotation * s.bodyRotation(record).Conjugate();
                if (change.w < 0.0f) change = -change;
                const float vectorLength = Math::Vec3(change.x, change.y, change.z).Length();
                Math::Vec3 angular(0.0f, 0.0f, 0.0f);
                if (vectorLength > 1e-7f)
                {
                    const float angle = 2.0f * atan2f(vectorLength, change.w);
                    angular =
                            Math::Vec3(change.x, change.y, change.z) * (angle / vectorLength / dt);
                }
                record.body->setAngularVelocity(angular);
            }
            else
            {
                record.body->setVelocity(Math::Vec3(0.0f, 0.0f, 0.0f));
                record.body->setAngularVelocity(Math::Vec3(0.0f, 0.0f, 0.0f));
            }
        }
        else if (record.mode == PhysicsBodyMode::Rigid)
            record.wasAwake = record.body->awake();
    }

    for (std::size_t i = 0; i < s.ragdolls.size(); ++i)
    {
        PhysicsRagdollTag& record = s.ragdolls.items()[i];
        if (!record.dying && record.ragdoll) record.ragdoll->beginStep(dt);
    }

    if (s.impulseMonitors > 0)
        for (std::size_t i = 0; i < count; ++i)
        {
            PhysicsBodyTag& record = s.bodies.items()[i];
            if (record.impulseMonitor) record.impulses.clear();
        }

    s.world.setGravity(s.gravity);
    const std::uint32_t substeps = s.substeps > 0 ? s.substeps : 1;
    const float substep = dt / static_cast<float>(substeps);
    for (std::uint32_t pass = 0; pass < substeps; ++pass)
    {
        for (std::size_t i = 0; i < count; ++i)
        {
            PhysicsBodyTag& record = s.bodies.items()[i];
            if (record.libId == kNoLibId || record.mode != PhysicsBodyMode::Rigid) continue;
            if (record.constantForce.LengthSquared() > 0.0f)
                record.body->addForce(record.constantForce);
            if (record.constantTorque.LengthSquared() > 0.0f)
                record.body->addTorque(record.constantTorque);
        }
        if (s.groundVehicles.size() > 0) s.applyGroundVehicles(substep);
        s.world.step(substep);
        if (s.impulseMonitors > 0) s.collectImpulses();
    }

    for (std::size_t i = 0; i < s.ragdolls.size(); ++i)
    {
        PhysicsRagdollTag& record = s.ragdolls.items()[i];
        if (!record.dying && record.ragdoll) record.ragdoll->endStep();
    }

    for (std::size_t i = 0; i < s.softBodies.size(); ++i)
    {
        PhysicsSoftBodyTag& soft = s.softBodies.items()[i];
        if (soft.dying || !soft.enabled || !soft.body) continue;
        soft.body->setGravity(s.gravity);
        soft.body->setCollisionWorld(&s.world);
        soft.body->step(dt, static_cast<std::uint32_t>(soft.substeps));
    }

    for (std::size_t i = 0; i < count; ++i)
    {
        PhysicsBodyTag& record = s.bodies.items()[i];
        if (record.libId == kNoLibId) continue;
        if (record.mode == PhysicsBodyMode::Kinematic && record.hasTarget)
        {
            record.body->setPosition(record.targetPosition);
            record.body->setOrientation(toKinetix(record.targetRotation));
            record.hasTarget = false;
        }
    }

    for (std::size_t i = 0; i < s.bodies.size(); ++i)
    {
        PhysicsBodyTag* record = &s.bodies.items()[i];
        if (record->dying || record->libId == kNoLibId || record->mode != PhysicsBodyMode::Rigid)
            continue;
        const bool awake = record->body->awake();
        if (!record->wasAwake && !awake) continue;
        PhysicsBodyListener* listener = record->listener;
        const bool changed = awake != record->wasAwake;
        record->wasAwake = awake;
        if (!listener) continue;
        listener->physicsBodyMoved(
                s.frameOf(*record, s.bodyPosition(*record), s.bodyRotation(*record)));
        if (changed) listener->physicsSleepingChanged(!awake);
    }

    s.dispatchContacts();

    s.locked = false;
    s.flushQueues();

    std::uint32_t awakeCount = 0;
    std::uint32_t staticCount = 0;
    std::uint32_t kinematicCount = 0;
    const auto bodies = s.bodies.items();
    for (std::size_t i = 0; i < bodies.size(); ++i)
    {
        const PhysicsBodyTag& record = bodies[i];
        if (record.mode == PhysicsBodyMode::Static) ++staticCount;
        else if (record.mode == PhysicsBodyMode::Kinematic)
            ++kinematicCount;
        else if (record.libId != kNoLibId && record.body->awake())
            ++awakeCount;
    }
    const double ms = static_cast<double>(time_nanos() - start) * 1e-6;
    s.stats.bodies = static_cast<std::uint32_t>(s.bodies.size());
    s.stats.awakeBodies = awakeCount;
    s.stats.staticBodies = staticCount;
    s.stats.kinematicBodies = kinematicCount;
    s.stats.shapes = static_cast<std::uint32_t>(s.shapes.size());
    s.stats.joints = static_cast<std::uint32_t>(s.joints.size());
    s.stats.vehicles = static_cast<std::uint32_t>(s.vehicles.size());
    s.stats.contacts = static_cast<std::uint32_t>(s.world.contactCount());
    ++s.stats.steps;
    s.stats.stepMs = ms;
    s.stepAccumulator += ms;
    s.stats.averageStepMs = s.stepAccumulator / static_cast<double>(s.stats.steps);
    if (ms > s.stats.maxStepMs) s.stats.maxStepMs = ms;
}

void PhysicsServer3D::setActive(bool active)
{
    mImpl->active = active;
}

bool PhysicsServer3D::isActive() const
{
    return mImpl->active;
}

bool PhysicsServer3D::isStepping() const
{
    return mImpl->locked;
}

void PhysicsServer3D::flushQueries()
{
    if (!mImpl->locked) mImpl->flushQueues();
}

void PhysicsServer3D::setGravity(const Math::Vec3& gravity)
{
    mImpl->gravity = gravity;
    mImpl->world.setGravity(gravity);
}

const Math::Vec3& PhysicsServer3D::getGravity() const
{
    return mImpl->gravity;
}

void PhysicsServer3D::setDefaultLinearDamp(float damp)
{
    mImpl->defaultLinearDamp = fmaxf(damp, 0.0f);
    mImpl->applyDampingAll();
}

float PhysicsServer3D::getDefaultLinearDamp() const
{
    return mImpl->defaultLinearDamp;
}

void PhysicsServer3D::setDefaultAngularDamp(float damp)
{
    mImpl->defaultAngularDamp = fmaxf(damp, 0.0f);
    mImpl->applyDampingAll();
}

float PhysicsServer3D::getDefaultAngularDamp() const
{
    return mImpl->defaultAngularDamp;
}

void PhysicsServer3D::setSolverIterations(std::uint32_t velocityIterations,
        std::uint32_t positionIterations)
{
    Impl& s = *mImpl;
    s.velocityIterations = velocityIterations > 0 ? velocityIterations : 1;
    s.positionIterations = positionIterations > 0 ? positionIterations : 1;
    s.applySolverSettings();
}

void PhysicsServer3D::setSubsteps(std::uint32_t substeps)
{
    mImpl->substeps = substeps < 1 ? 1 : (substeps > 16 ? 16 : substeps);
}

std::uint32_t PhysicsServer3D::getSubsteps() const
{
    return mImpl->substeps;
}

std::uint32_t PhysicsServer3D::getVelocityIterations() const
{
    return mImpl->velocityIterations;
}

std::uint32_t PhysicsServer3D::getPositionIterations() const
{
    return mImpl->positionIterations;
}

void PhysicsServer3D::setContactMargin(float margin)
{
    mImpl->contactMargin = fmaxf(margin, 0.0f);
    mImpl->world.setContactMargin(mImpl->contactMargin);
}

float PhysicsServer3D::getContactMargin() const
{
    return mImpl->contactMargin;
}

PhysicsShapeId PhysicsServer3D::shapeCreate(PhysicsShapeType type)
{
    PhysicsShapeTag record;
    record.type = type;
    if (type == PhysicsShapeType::HeightMap)
    {
        record.heights.assign(4, 0.0f);
    }
    return mImpl->shapes.insert(static_cast<PhysicsShapeTag&&>(record));
}

void PhysicsServer3D::shapeFree(PhysicsShapeId shape)
{
    Impl& s = *mImpl;
    PhysicsShapeTag* record = s.shape(shape);
    if (!record) return;
    for (std::size_t i = 0; i < s.bodies.size(); ++i)
    {
        PhysicsBodyTag& body = s.bodies.items()[i];
        if (body.shape != shape) continue;
        const PhysicsBodyId id = s.bodies.handle_at(i);
        s.removeFromWorld(body);
        s.releaseInstance(body);
        body.shape = PhysicsShapeId();
        (void) id;
    }
    s.destroyShapeRecord(*record);
    s.shapes.erase(shape);
}

bool PhysicsServer3D::shapeValid(PhysicsShapeId shape) const
{
    return mImpl->shape(shape) != nullptr;
}

PhysicsShapeType PhysicsServer3D::shapeGetType(PhysicsShapeId shape) const
{
    const PhysicsShapeTag* record = mImpl->shape(shape);
    return record ? record->type : PhysicsShapeType::Box;
}

namespace
{

void rebuildBodiesOfShape(PhysicsServer3D::Impl& s, PhysicsShapeId shape)
{
    PhysicsShapeTag* record = s.shape(shape);
    if (!record) return;
    ct::Vector<PhysicsBodyId> affected;
    for (std::size_t i = 0; i < s.bodies.size(); ++i)
        if (s.bodies.items()[i].shape == shape) affected.push_back(s.bodies.handle_at(i));
    for (std::size_t n = 0; n < affected.size(); ++n)
    {
        PhysicsBodyTag* body = s.bodies.get(affected[n]);
        if (!body) continue;
        s.removeFromWorld(*body);
        s.releaseInstance(*body);
    }
    for (std::size_t n = 0; n < affected.size(); ++n)
    {
        PhysicsBodyTag* body = s.bodies.get(affected[n]);
        if (!body) continue;
        body->instance = s.acquireInstance(*record, body->localScale);
        if (body->mode == PhysicsBodyMode::Rigid) s.applyMass(*body);
        s.syncWorld(affected[n], *body);
    }
}

} // namespace

void PhysicsServer3D::shapeSetBox(PhysicsShapeId shape, const Math::Vec3& halfExtents)
{
    PhysicsShapeTag* record = mImpl->shape(shape);
    if (!record || record->type != PhysicsShapeType::Box) return;
    record->halfExtents = absVec(halfExtents);
    rebuildBodiesOfShape(*mImpl, shape);
}

Math::Vec3 PhysicsServer3D::shapeGetBox(PhysicsShapeId shape) const
{
    const PhysicsShapeTag* record = mImpl->shape(shape);
    return record ? record->halfExtents : Math::Vec3(0.0f, 0.0f, 0.0f);
}

void PhysicsServer3D::shapeSetSphere(PhysicsShapeId shape, float radius)
{
    PhysicsShapeTag* record = mImpl->shape(shape);
    if (!record || record->type != PhysicsShapeType::Sphere) return;
    record->radius = fmaxf(radius, 0.0001f);
    rebuildBodiesOfShape(*mImpl, shape);
}

float PhysicsServer3D::shapeGetSphere(PhysicsShapeId shape) const
{
    const PhysicsShapeTag* record = mImpl->shape(shape);
    return record ? record->radius : 0.0f;
}

void PhysicsServer3D::shapeSetCapsule(PhysicsShapeId shape, float radius, float height)
{
    PhysicsShapeTag* record = mImpl->shape(shape);
    if (!record || record->type != PhysicsShapeType::Capsule) return;
    record->radius = fmaxf(radius, 0.0001f);
    record->height = fmaxf(height, record->radius * 2.0f);
    rebuildBodiesOfShape(*mImpl, shape);
}

void PhysicsServer3D::shapeSetCylinder(PhysicsShapeId shape, float radius, float height)
{
    PhysicsShapeTag* record = mImpl->shape(shape);
    if (!record || record->type != PhysicsShapeType::Cylinder) return;
    record->radius = fmaxf(radius, 0.0001f);
    record->height = fmaxf(height, 0.0001f);
    rebuildBodiesOfShape(*mImpl, shape);
}

float PhysicsServer3D::shapeGetCylinderRadius(PhysicsShapeId shape) const
{
    const PhysicsShapeTag* record = mImpl->shape(shape);
    return record && record->type == PhysicsShapeType::Cylinder ? record->radius : 0.0f;
}

float PhysicsServer3D::shapeGetCylinderHeight(PhysicsShapeId shape) const
{
    const PhysicsShapeTag* record = mImpl->shape(shape);
    return record && record->type == PhysicsShapeType::Cylinder ? record->height : 0.0f;
}

float PhysicsServer3D::shapeGetCapsuleRadius(PhysicsShapeId shape) const
{
    const PhysicsShapeTag* record = mImpl->shape(shape);
    return record ? record->radius : 0.0f;
}

float PhysicsServer3D::shapeGetCapsuleHeight(PhysicsShapeId shape) const
{
    const PhysicsShapeTag* record = mImpl->shape(shape);
    return record ? record->height : 0.0f;
}

void PhysicsServer3D::shapeSetWorldBoundary(PhysicsShapeId shape, const Math::Vec3& normal,
        float distance)
{
    PhysicsShapeTag* record = mImpl->shape(shape);
    if (!record || record->type != PhysicsShapeType::WorldBoundary) return;
    record->planeNormal = normal.NormalizedSafe();
    record->planeDistance = distance;
    rebuildBodiesOfShape(*mImpl, shape);
}

void PhysicsServer3D::shapeSetConcaveFaces(PhysicsShapeId shape, const Math::Vec3* vertices,
        std::size_t vertexCount, const std::uint32_t* indices, std::size_t indexCount)
{
    PhysicsShapeTag* record = mImpl->shape(shape);
    if (!record || record->type != PhysicsShapeType::ConcavePolygon) return;
    record->vertices.clear();
    record->indices.clear();
    if (vertices && vertexCount) record->vertices.assign(vertices, vertices + vertexCount);
    if (indices && indexCount)
        record->indices.assign(indices, indices + indexCount - indexCount % 3);
    rebuildBodiesOfShape(*mImpl, shape);
}

namespace
{

struct WeldEntry
{
    Math::Vec3 position;
    std::uint32_t corner;
};

bool weldLess(const WeldEntry& a, const WeldEntry& b)
{
    if (a.position.x != b.position.x) return a.position.x < b.position.x;
    if (a.position.y != b.position.y) return a.position.y < b.position.y;
    if (a.position.z != b.position.z) return a.position.z < b.position.z;
    return a.corner < b.corner;
}

} // namespace

void PhysicsServer3D::shapeSetConcaveCollision(PhysicsShapeId shape, const CollisionMesh& mesh)
{
    PhysicsShapeTag* record = mImpl->shape(shape);
    if (!record || record->type != PhysicsShapeType::ConcavePolygon) return;
    const std::uint32_t triangles = mesh.triangleCount();
    ct::Vector<WeldEntry> corners;
    corners.reserve(static_cast<std::size_t>(triangles) * 3);
    for (std::uint32_t t = 0; t < triangles; ++t)
    {
        Math::Vec3 a, b, c;
        mesh.triangle(t, a, b, c);
        corners.push_back({ a, t * 3 });
        corners.push_back({ b, t * 3 + 1 });
        corners.push_back({ c, t * 3 + 2 });
    }
    ct::sort(corners.begin(), corners.end(), weldLess);
    record->vertices.clear();
    record->indices.assign(static_cast<std::size_t>(triangles) * 3, 0);
    for (std::size_t i = 0; i < corners.size(); ++i)
    {
        if (i == 0 || !(corners[i].position == corners[i - 1].position))
            record->vertices.push_back(corners[i].position);
        record->indices[corners[i].corner] =
                static_cast<std::uint32_t>(record->vertices.size() - 1);
    }
    rebuildBodiesOfShape(*mImpl, shape);
}

void PhysicsServer3D::shapeSetConvexHull(PhysicsShapeId shape, const Math::Vec3* points,
        std::size_t count)
{
    PhysicsShapeTag* record = mImpl->shape(shape);
    if (!record || record->type != PhysicsShapeType::ConvexHull) return;
    record->hullVertices.clear();
    record->hullEdges.clear();
    record->hullFaces.clear();
    record->hullCenter = Math::Vec3(0.0f, 0.0f, 0.0f);
    if (points && count >= 4)
    {
        kinetix::Geometry::ConvexHullComputer hull;
        const float shrunk = hull.compute(&points[0].x, static_cast<int>(sizeof(Math::Vec3)),
                static_cast<int>(count), 0.0f, 0.0f);
        if (shrunk >= 0.0f && hull.vertices.size() >= 4 && hull.faces.size() >= 4)
        {
            float volume = 0.0f;
            Math::Vec3 weighted(0.0f, 0.0f, 0.0f);
            const Math::Vec3 origin = hull.vertices[0];
            for (std::size_t f = 0; f < hull.faces.size(); ++f)
            {
                const auto* start = &hull.edges[static_cast<std::size_t>(hull.faces[f])];
                const Math::Vec3 a =
                        hull.vertices[static_cast<std::size_t>(start->getSourceVertex())];
                const auto* edge = start->getNextEdgeOfFace();
                while (edge != start && edge->getNextEdgeOfFace() != start)
                {
                    const Math::Vec3 b =
                            hull.vertices[static_cast<std::size_t>(edge->getSourceVertex())];
                    const Math::Vec3 c = hull.vertices[static_cast<std::size_t>(
                            edge->getNextEdgeOfFace()->getSourceVertex())];
                    const float tetra =
                            Math::Vec3::Dot(a - origin, Math::Vec3::Cross(b - origin, c - origin)) /
                            6.0f;
                    volume += tetra;
                    weighted = weighted + (origin + a + b + c) * (tetra * 0.25f);
                    edge = edge->getNextEdgeOfFace();
                }
            }
            Math::Vec3 center = origin;
            if (fabsf(volume) > 1e-12f) center = weighted * (1.0f / volume);
            record->hullCenter = center;
            record->hullVertices.reserve(hull.vertices.size());
            for (std::size_t i = 0; i < hull.vertices.size(); ++i)
                record->hullVertices.push_back(hull.vertices[i] - center);
            record->hullEdges = hull.edges;
            record->hullFaces = hull.faces;
        }
    }
    rebuildBodiesOfShape(*mImpl, shape);
}

std::size_t PhysicsServer3D::shapeGetConvexVertexCount(PhysicsShapeId shape) const
{
    const PhysicsShapeTag* record = mImpl->shape(shape);
    return record ? record->hullVertices.size() : 0;
}

Math::Vec3 PhysicsServer3D::shapeGetConvexCenter(PhysicsShapeId shape) const
{
    const PhysicsShapeTag* record = mImpl->shape(shape);
    return record ? record->hullCenter : Math::Vec3(0.0f, 0.0f, 0.0f);
}

void PhysicsServer3D::shapeSetHeightMap(PhysicsShapeId shape, int width, int depth,
        const float* heights)
{
    PhysicsShapeTag* record = mImpl->shape(shape);
    if (!record || record->type != PhysicsShapeType::HeightMap || width < 2 || depth < 2 ||
            !heights)
        return;
    record->mapWidth = width;
    record->mapDepth = depth;
    record->heights.assign(heights, heights + static_cast<std::size_t>(width) * depth);
    record->mapScale = Math::Vec3(1.0f, 1.0f, 1.0f);
    record->mapOffset = Math::Vec3(0.0f, 0.0f, 0.0f);
    rebuildBodiesOfShape(*mImpl, shape);
}

void PhysicsServer3D::shapeSetHeightMapFromTerrain(PhysicsShapeId shape, const Terrain& terrain)
{
    PhysicsShapeTag* record = mImpl->shape(shape);
    if (!record || record->type != PhysicsShapeType::HeightMap || !terrain.isPrepared()) return;
    if (terrain.getAlignment() != TerrainAlignment::AlignXZ)
    {
        log_error("physics: only terrains aligned to X-Z become height maps");
        return;
    }
    const int n = static_cast<int>(terrain.getSize());
    record->mapWidth = n;
    record->mapDepth = n;
    record->heights.resize(static_cast<std::size_t>(n) * n);
    for (int j = 0; j < n; ++j)
        for (int i = 0; i < n; ++i)
            record->heights[static_cast<std::size_t>(j * n + i)] = terrain.getHeightAtPoint(
                    static_cast<std::uint32_t>(i), static_cast<std::uint32_t>(n - 1 - j));
    const float cell = terrain.getScale();
    const float centre = terrain.getBase() + (n - 1) * cell * 0.5f;
    record->mapScale = Math::Vec3(cell, 1.0f, cell);
    record->mapOffset = Math::Vec3(centre, 0.0f, -centre);
    rebuildBodiesOfShape(*mImpl, shape);
}

void PhysicsServer3D::shapeSetHeightMapTransform(PhysicsShapeId shape, const Math::Vec3& scale,
        const Math::Vec3& offset)
{
    PhysicsShapeTag* record = mImpl->shape(shape);
    if (!record || record->type != PhysicsShapeType::HeightMap) return;
    record->mapScale = scale;
    record->mapOffset = offset;
    rebuildBodiesOfShape(*mImpl, shape);
}

bool PhysicsServer3D::shapeGetHeightMap(PhysicsShapeId shape, ct::Vector<float>& heights) const
{
    const PhysicsShapeTag* record = mImpl->shape(shape);
    if (!record || record->type != PhysicsShapeType::HeightMap) return false;
    heights = record->heights;
    return true;
}

bool PhysicsServer3D::shapeGetConcaveFaces(PhysicsShapeId shape, ct::Vector<Math::Vec3>& vertices,
        ct::Vector<std::uint32_t>& indices) const
{
    const PhysicsShapeTag* record = mImpl->shape(shape);
    if (!record || record->type != PhysicsShapeType::ConcavePolygon) return false;
    vertices = record->vertices;
    indices = record->indices;
    return true;
}

int PhysicsServer3D::shapeGetHeightMapWidth(PhysicsShapeId shape) const
{
    const PhysicsShapeTag* record = mImpl->shape(shape);
    return record ? record->mapWidth : 0;
}

int PhysicsServer3D::shapeGetHeightMapDepth(PhysicsShapeId shape) const
{
    const PhysicsShapeTag* record = mImpl->shape(shape);
    return record ? record->mapDepth : 0;
}

Math::Vec3 PhysicsServer3D::shapeGetHeightMapScale(PhysicsShapeId shape) const
{
    const PhysicsShapeTag* record = mImpl->shape(shape);
    return record ? record->mapScale : Math::Vec3(1.0f, 1.0f, 1.0f);
}

Math::Vec3 PhysicsServer3D::shapeGetHeightMapOffset(PhysicsShapeId shape) const
{
    const PhysicsShapeTag* record = mImpl->shape(shape);
    return record ? record->mapOffset : Math::Vec3(0.0f, 0.0f, 0.0f);
}

std::size_t PhysicsServer3D::shapeGetTriangleCount(PhysicsShapeId shape) const
{
    const PhysicsShapeTag* record = mImpl->shape(shape);
    if (!record) return 0;
    if (record->type == PhysicsShapeType::HeightMap)
        return static_cast<std::size_t>((record->mapWidth - 1) * (record->mapDepth - 1) * 2);
    return record->indices.size() / 3;
}

PhysicsBodyId PhysicsServer3D::bodyCreate()
{
    PhysicsBodyTag record;
    record.body = new kp::RigidBody();
    record.body->setBodyType(kp::BodyType::Static);
    const PhysicsBodyId id = mImpl->bodies.insert(static_cast<PhysicsBodyTag&&>(record));
    mImpl->applyDamping(*mImpl->bodies.get(id));
    return id;
}

void PhysicsServer3D::bodyFree(PhysicsBodyId body)
{
    Impl& s = *mImpl;
    PhysicsBodyTag* record = s.body(body);
    if (!record) return;
    if (s.locked)
    {
        record->listener = nullptr;
        record->dying = true;
        s.freeQueue.push_back(body);
        return;
    }
    s.destroyBodyNow(body);
}

bool PhysicsServer3D::bodyValid(PhysicsBodyId body) const
{
    return mImpl->body(body) != nullptr;
}

void PhysicsServer3D::bodySetMode(PhysicsBodyId id, PhysicsBodyMode mode)
{
    Impl& s = *mImpl;
    PhysicsBodyTag* record = s.body(id);
    if (!record || record->mode == mode) return;
    const bool wasStatic = record->mode == PhysicsBodyMode::Static;
    record->mode = mode;
    record->hasTarget = false;
    record->everPlaced = false;
    switch (mode)
    {
        case PhysicsBodyMode::Static:
            record->body->setBodyType(kp::BodyType::Static);
            break;
        case PhysicsBodyMode::Kinematic:
            record->body->setBodyType(kp::BodyType::Kinematic);
            break;
        case PhysicsBodyMode::Rigid:
            record->body->setBodyType(kp::BodyType::Dynamic);
            s.applyMass(*record);
            break;
    }
    if (mode != PhysicsBodyMode::Rigid)
    {
        record->body->setVelocity(Math::Vec3(0.0f, 0.0f, 0.0f));
        record->body->setAngularVelocity(Math::Vec3(0.0f, 0.0f, 0.0f));
    }
    s.removeFromWorld(*record);
    if (wasStatic || mode == PhysicsBodyMode::Static) s.world.markStaticBroadphaseDirty();
    s.syncWorld(id, *record);
}

PhysicsBodyMode PhysicsServer3D::bodyGetMode(PhysicsBodyId body) const
{
    const PhysicsBodyTag* record = mImpl->body(body);
    return record ? record->mode : PhysicsBodyMode::Static;
}

void PhysicsServer3D::bodySetShape(PhysicsBodyId id, PhysicsShapeId shape,
        const Math::Mat4& localTransform)
{
    Impl& s = *mImpl;
    PhysicsBodyTag* record = s.body(id);
    if (!record) return;
    s.removeFromWorld(*record);
    s.releaseInstance(*record);
    record->shape = PhysicsShapeId();
    PhysicsShapeTag* shapeRecord = s.shape(shape);
    if (!shapeRecord)
    {
        record->offsetPosition = Math::Vec3(0.0f, 0.0f, 0.0f);
        record->offsetRotation = Math::Quaternion::Identity();
        return;
    }
    const Math::Vec3 scale = Math::Vec3(vec3Of(localTransform.col0).Length(),
            vec3Of(localTransform.col1).Length(), vec3Of(localTransform.col2).Length());
    record->shape = shape;
    record->localScale = scale;
    record->offsetRotation = rotationOf(localTransform);
    record->offsetPosition = vec3Of(localTransform.col3);
    if (shapeRecord->type == PhysicsShapeType::HeightMap)
        record->offsetPosition = record->offsetPosition + shapeRecord->mapOffset * scale;
    record->instance = s.acquireInstance(*shapeRecord, scale);
    if (record->mode == PhysicsBodyMode::Rigid) s.applyMass(*record);
    s.syncWorld(id, *record);
}

PhysicsShapeId PhysicsServer3D::bodyGetShape(PhysicsBodyId body) const
{
    const PhysicsBodyTag* record = mImpl->body(body);
    return record ? record->shape : PhysicsShapeId();
}

bool PhysicsServer3D::bodyIsSimulated(PhysicsBodyId body) const
{
    const PhysicsBodyTag* record = mImpl->body(body);
    return record && record->libId != kNoLibId;
}

void PhysicsServer3D::bodySetListener(PhysicsBodyId body, PhysicsBodyListener* listener)
{
    PhysicsBodyTag* record = mImpl->body(body);
    if (record) record->listener = listener;
}

void PhysicsServer3D::bodySetUserData(PhysicsBodyId body, void* data)
{
    PhysicsBodyTag* record = mImpl->body(body);
    if (record) record->userData = data;
}

void* PhysicsServer3D::bodyGetUserData(PhysicsBodyId body) const
{
    const PhysicsBodyTag* record = mImpl->body(body);
    return record ? record->userData : nullptr;
}

void PhysicsServer3D::bodySetEnabled(PhysicsBodyId id, bool enabled)
{
    PhysicsBodyTag* record = mImpl->body(id);
    if (!record || record->enabled == enabled) return;
    record->enabled = enabled;
    mImpl->syncWorld(id, *record);
}

bool PhysicsServer3D::bodyIsEnabled(PhysicsBodyId body) const
{
    const PhysicsBodyTag* record = mImpl->body(body);
    return record && record->enabled;
}

void PhysicsServer3D::bodySetInWorld(PhysicsBodyId id, bool inWorld)
{
    Impl& s = *mImpl;
    PhysicsBodyTag* record = s.body(id);
    if (!record || record->inWorld == inWorld) return;
    record->inWorld = inWorld;
    if (!inWorld) s.detachContacts(id, *record, true);
    s.syncWorld(id, *record);
}

void PhysicsServer3D::bodySetTransform(PhysicsBodyId id, const Math::Mat4& frame)
{
    Impl& s = *mImpl;
    PhysicsBodyTag* record = s.body(id);
    if (!record) return;
    Math::Vec3 position;
    Math::Quaternion rotation;
    s.libPoseOf(*record, frame, position, rotation);
    if (record->mode == PhysicsBodyMode::Kinematic && record->everPlaced)
    {
        record->hasTarget = true;
        record->targetPosition = position;
        record->targetRotation = rotation;
        return;
    }
    record->everPlaced = true;
    record->body->setPosition(position);
    record->body->setOrientation(toKinetix(rotation));
    record->previousPosition = position;
    record->previousRotation = rotation;
    record->hasTarget = false;
    if (record->mode == PhysicsBodyMode::Static) s.world.markStaticBroadphaseDirty();
    else if (record->mode == PhysicsBodyMode::Rigid)
        record->body->setAwake(true);
}

Math::Mat4 PhysicsServer3D::bodyGetTransform(PhysicsBodyId id) const
{
    const Impl& s = *mImpl;
    const PhysicsBodyTag* record = s.body(id);
    if (!record) return Math::Mat4::Identity();
    return s.frameOf(*record, s.bodyPosition(*record), s.bodyRotation(*record));
}

Math::Mat4 PhysicsServer3D::bodyGetInterpolatedTransform(PhysicsBodyId id, float fraction) const
{
    const Impl& s = *mImpl;
    const PhysicsBodyTag* record = s.body(id);
    if (!record) return Math::Mat4::Identity();
    const float t = fraction < 0.0f ? 0.0f : (fraction > 1.0f ? 1.0f : fraction);
    const Math::Vec3 position =
            Math::Vec3::Lerp(record->previousPosition, s.bodyPosition(*record), t);
    const Math::Quaternion rotation =
            Math::Quaternion::Slerp(record->previousRotation, s.bodyRotation(*record), t);
    return s.frameOf(*record, position, rotation);
}

Math::Vec3 PhysicsServer3D::bodyGetCenterOfMass(PhysicsBodyId body) const
{
    const PhysicsBodyTag* record = mImpl->body(body);
    return record ? mImpl->bodyPosition(*record) : Math::Vec3(0.0f, 0.0f, 0.0f);
}

void PhysicsServer3D::bodySetCollisionLayer(PhysicsBodyId id, std::uint32_t layer)
{
    PhysicsBodyTag* record = mImpl->body(id);
    if (!record) return;
    record->layer = layer;
    mImpl->syncWorld(id, *record);
}

std::uint32_t PhysicsServer3D::bodyGetCollisionLayer(PhysicsBodyId body) const
{
    const PhysicsBodyTag* record = mImpl->body(body);
    return record ? record->layer : 0;
}

void PhysicsServer3D::bodySetCollisionMask(PhysicsBodyId id, std::uint32_t mask)
{
    PhysicsBodyTag* record = mImpl->body(id);
    if (!record) return;
    record->mask = mask;
    mImpl->syncWorld(id, *record);
}

std::uint32_t PhysicsServer3D::bodyGetCollisionMask(PhysicsBodyId body) const
{
    const PhysicsBodyTag* record = mImpl->body(body);
    return record ? record->mask : 0;
}

void PhysicsServer3D::bodySetFriction(PhysicsBodyId id, float friction)
{
    PhysicsBodyTag* record = mImpl->body(id);
    if (!record) return;
    record->friction = fmaxf(friction, 0.0f);
    mImpl->syncWorld(id, *record);
}

float PhysicsServer3D::bodyGetFriction(PhysicsBodyId body) const
{
    const PhysicsBodyTag* record = mImpl->body(body);
    return record ? record->friction : 0.0f;
}

void PhysicsServer3D::bodySetBounce(PhysicsBodyId id, float bounce)
{
    PhysicsBodyTag* record = mImpl->body(id);
    if (!record) return;
    record->bounce = fminf(fmaxf(bounce, 0.0f), 1.0f);
    mImpl->syncWorld(id, *record);
}

float PhysicsServer3D::bodyGetBounce(PhysicsBodyId body) const
{
    const PhysicsBodyTag* record = mImpl->body(body);
    return record ? record->bounce : 0.0f;
}

void PhysicsServer3D::bodySetMass(PhysicsBodyId id, float mass)
{
    PhysicsBodyTag* record = mImpl->body(id);
    if (!record) return;
    record->mass = fmaxf(mass, 0.001f);
    mImpl->applyMass(*record);
}

float PhysicsServer3D::bodyGetMass(PhysicsBodyId body) const
{
    const PhysicsBodyTag* record = mImpl->body(body);
    return record ? record->mass : 0.0f;
}

void PhysicsServer3D::bodySetInertia(PhysicsBodyId id, const Math::Vec3& inertia)
{
    PhysicsBodyTag* record = mImpl->body(id);
    if (!record) return;
    record->customInertia = inertia;
    mImpl->applyMass(*record);
}

Math::Vec3 PhysicsServer3D::bodyGetInertia(PhysicsBodyId body) const
{
    const PhysicsBodyTag* record = mImpl->body(body);
    if (!record) return Math::Vec3(0.0f, 0.0f, 0.0f);
    if (record->customInertia.x > 0.0f) return record->customInertia;
    if (!record->instance) return Math::Vec3(0.0f, 0.0f, 0.0f);
    const Math::Mat3 tensor = record->instance->shape->inertia(record->mass);
    return Math::Vec3(tensor.col0.x, tensor.col1.y, tensor.col2.z);
}

void PhysicsServer3D::bodySetLockRotation(PhysicsBodyId id, bool lock)
{
    PhysicsBodyTag* record = mImpl->body(id);
    if (!record || record->lockRotation == lock) return;
    record->lockRotation = lock;
    mImpl->applyMass(*record);
}

bool PhysicsServer3D::bodyIsLockRotation(PhysicsBodyId body) const
{
    const PhysicsBodyTag* record = mImpl->body(body);
    return record && record->lockRotation;
}

void PhysicsServer3D::bodySetLinearDamp(PhysicsBodyId id, float damp, PhysicsDampMode mode)
{
    PhysicsBodyTag* record = mImpl->body(id);
    if (!record) return;
    record->linearDamp = fmaxf(damp, 0.0f);
    record->linearMode = mode;
    mImpl->applyDamping(*record);
}

void PhysicsServer3D::bodySetAngularDamp(PhysicsBodyId id, float damp, PhysicsDampMode mode)
{
    PhysicsBodyTag* record = mImpl->body(id);
    if (!record) return;
    record->angularDamp = fmaxf(damp, 0.0f);
    record->angularMode = mode;
    mImpl->applyDamping(*record);
}

void PhysicsServer3D::bodySetInterpolated(PhysicsBodyId id, bool interpolated)
{
    if (PhysicsBodyTag* record = mImpl->body(id)) record->interpolated = interpolated;
}

void PhysicsServer3D::bodySetCanSleep(PhysicsBodyId id, bool canSleep)
{
    PhysicsBodyTag* record = mImpl->body(id);
    if (!record) return;
    record->canSleep = canSleep;
    record->body->setCanSleep(canSleep);
}

bool PhysicsServer3D::bodyCanSleep(PhysicsBodyId body) const
{
    const PhysicsBodyTag* record = mImpl->body(body);
    return record && record->canSleep;
}

void PhysicsServer3D::bodySetSleeping(PhysicsBodyId id, bool sleeping)
{
    PhysicsBodyTag* record = mImpl->body(id);
    if (!record || record->mode != PhysicsBodyMode::Rigid) return;
    record->body->setAwake(!sleeping);
}

bool PhysicsServer3D::bodyIsSleeping(PhysicsBodyId body) const
{
    const PhysicsBodyTag* record = mImpl->body(body);
    return record && record->mode == PhysicsBodyMode::Rigid && !record->body->awake();
}

void PhysicsServer3D::bodySetLinearVelocity(PhysicsBodyId id, const Math::Vec3& velocity)
{
    PhysicsBodyTag* record = mImpl->body(id);
    if (!record) return;
    if (record->mode == PhysicsBodyMode::Rigid && velocity.LengthSquared() > 0.0f)
        record->body->setAwake(true);
    record->body->setVelocity(velocity);
}

Math::Vec3 PhysicsServer3D::bodyGetLinearVelocity(PhysicsBodyId body) const
{
    const PhysicsBodyTag* record = mImpl->body(body);
    return record ? record->body->velocity() : Math::Vec3(0.0f, 0.0f, 0.0f);
}

void PhysicsServer3D::bodySetAngularVelocity(PhysicsBodyId id, const Math::Vec3& velocity)
{
    PhysicsBodyTag* record = mImpl->body(id);
    if (!record) return;
    if (record->mode == PhysicsBodyMode::Rigid && velocity.LengthSquared() > 0.0f)
        record->body->setAwake(true);
    record->body->setAngularVelocity(velocity);
}

Math::Vec3 PhysicsServer3D::bodyGetAngularVelocity(PhysicsBodyId body) const
{
    const PhysicsBodyTag* record = mImpl->body(body);
    return record ? record->body->angularVelocity() : Math::Vec3(0.0f, 0.0f, 0.0f);
}

Math::Vec3 PhysicsServer3D::bodyGetVelocityAtPosition(PhysicsBodyId body,
        const Math::Vec3& position) const
{
    const PhysicsBodyTag* record = mImpl->body(body);
    return record ? record->body->velocityAtPoint(position) : Math::Vec3(0.0f, 0.0f, 0.0f);
}

void PhysicsServer3D::bodyApplyCentralImpulse(PhysicsBodyId body, const Math::Vec3& impulse)
{
    PhysicsBodyTag* record = mImpl->body(body);
    if (record) record->body->applyLinearImpulse(impulse);
}

void PhysicsServer3D::bodyApplyImpulse(PhysicsBodyId body, const Math::Vec3& impulse,
        const Math::Vec3& offset)
{
    PhysicsBodyTag* record = mImpl->body(body);
    if (!record) return;
    const Math::Vec3 origin = vec3Of(
            mImpl->frameOf(*record, mImpl->bodyPosition(*record), mImpl->bodyRotation(*record))
                    .col3);
    record->body->applyImpulseAtPoint(impulse, origin + offset);
}

void PhysicsServer3D::bodyApplyTorqueImpulse(PhysicsBodyId body, const Math::Vec3& impulse)
{
    PhysicsBodyTag* record = mImpl->body(body);
    if (record) record->body->applyAngularImpulse(impulse);
}

void PhysicsServer3D::bodyApplyCentralForce(PhysicsBodyId body, const Math::Vec3& force)
{
    PhysicsBodyTag* record = mImpl->body(body);
    if (record) record->body->addForce(force);
}

void PhysicsServer3D::bodyApplyForce(PhysicsBodyId body, const Math::Vec3& force,
        const Math::Vec3& offset)
{
    PhysicsBodyTag* record = mImpl->body(body);
    if (!record) return;
    const Math::Vec3 origin = vec3Of(
            mImpl->frameOf(*record, mImpl->bodyPosition(*record), mImpl->bodyRotation(*record))
                    .col3);
    record->body->addForceAtPoint(force, origin + offset);
}

void PhysicsServer3D::bodyApplyTorque(PhysicsBodyId body, const Math::Vec3& torque)
{
    PhysicsBodyTag* record = mImpl->body(body);
    if (record) record->body->addTorque(torque);
}

void PhysicsServer3D::bodySetConstantForce(PhysicsBodyId body, const Math::Vec3& force)
{
    PhysicsBodyTag* record = mImpl->body(body);
    if (record) record->constantForce = force;
}

Math::Vec3 PhysicsServer3D::bodyGetConstantForce(PhysicsBodyId body) const
{
    const PhysicsBodyTag* record = mImpl->body(body);
    return record ? record->constantForce : Math::Vec3(0.0f, 0.0f, 0.0f);
}

void PhysicsServer3D::bodySetConstantTorque(PhysicsBodyId body, const Math::Vec3& torque)
{
    PhysicsBodyTag* record = mImpl->body(body);
    if (record) record->constantTorque = torque;
}

Math::Vec3 PhysicsServer3D::bodyGetConstantTorque(PhysicsBodyId body) const
{
    const PhysicsBodyTag* record = mImpl->body(body);
    return record ? record->constantTorque : Math::Vec3(0.0f, 0.0f, 0.0f);
}

namespace
{

bool exceptionMatches(const PhysicsException& pair, std::uint32_t a, std::uint32_t b)
{
    return (pair.a == a && pair.b == b) || (pair.a == b && pair.b == a);
}

} // namespace

void PhysicsServer3D::bodyAddCollisionException(PhysicsBodyId body, PhysicsBodyId other)
{
    Impl& s = *mImpl;
    PhysicsBodyTag* first = s.body(body);
    PhysicsBodyTag* second = s.body(other);
    if (!first || !second || body == other) return;
    for (std::size_t i = 0; i < s.exceptions.size(); ++i)
        if (exceptionMatches(s.exceptions[i], body.bits(), other.bits())) return;
    PhysicsException pair;
    pair.a = body.bits();
    pair.b = other.bits();
    s.exceptions.push_back(pair);
    if (first->libId != kNoLibId && second->libId != kNoLibId)
        s.world.setPairIgnored(first->libId, second->libId, true);
}

void PhysicsServer3D::bodyRemoveCollisionException(PhysicsBodyId body, PhysicsBodyId other)
{
    Impl& s = *mImpl;
    for (std::size_t i = 0; i < s.exceptions.size(); ++i)
    {
        if (!exceptionMatches(s.exceptions[i], body.bits(), other.bits())) continue;
        s.exceptions.erase(s.exceptions.begin() + i);
        const PhysicsBodyTag* first = s.body(body);
        const PhysicsBodyTag* second = s.body(other);
        if (first && second && first->libId != kNoLibId && second->libId != kNoLibId)
            s.world.setPairIgnored(first->libId, second->libId, false);
        return;
    }
}

bool PhysicsServer3D::bodyHasCollisionException(PhysicsBodyId body, PhysicsBodyId other) const
{
    const Impl& s = *mImpl;
    for (std::size_t i = 0; i < s.exceptions.size(); ++i)
        if (exceptionMatches(s.exceptions[i], body.bits(), other.bits())) return true;
    return false;
}

void PhysicsServer3D::bodySetContactMonitor(PhysicsBodyId id, bool enabled, int maxContactsReported)
{
    Impl& s = *mImpl;
    PhysicsBodyTag* record = s.body(id);
    if (!record) return;
    record->monitor = enabled;
    record->maxContacts = maxContactsReported;
    if (!enabled || maxContactsReported <= 0) record->touching.clear();
}

int PhysicsServer3D::bodyGetContactCount(PhysicsBodyId body) const
{
    const PhysicsBodyTag* record = mImpl->body(body);
    return record ? static_cast<int>(record->touching.size()) : 0;
}

void PhysicsServer3D::bodyGetCollidingBodies(PhysicsBodyId body,
        ct::Vector<PhysicsBodyId>& out) const
{
    out.clear();
    const PhysicsBodyTag* record = mImpl->body(body);
    if (!record) return;
    for (std::size_t i = 0; i < record->touching.size(); ++i)
        out.push_back(PhysicsBodyId::from_bits(record->touching[i]));
}

void PhysicsServer3D::bodySetImpulseMonitor(PhysicsBodyId body, bool enabled)
{
    Impl& s = *mImpl;
    PhysicsBodyTag* record = s.body(body);
    if (!record || record->impulseMonitor == enabled) return;
    record->impulseMonitor = enabled;
    record->impulses.clear();
    if (enabled) ++s.impulseMonitors;
    else
        --s.impulseMonitors;
}

const ct::Vector<PhysicsContactImpulse>* PhysicsServer3D::bodyGetContactImpulses(
        PhysicsBodyId body) const
{
    const PhysicsBodyTag* record = mImpl->body(body);
    return record && record->impulseMonitor ? &record->impulses : nullptr;
}

PhysicsJointId PhysicsServer3D::jointCreate(PhysicsJointType type, PhysicsBodyId bodyA,
        PhysicsBodyId bodyB, const Math::Vec3& worldAnchor, const Math::Vec3& worldAxis)
{
    PhysicsJointFrame frame;
    frame.anchor = worldAnchor;
    frame.anchorB = worldAnchor;
    frame.axis = worldAxis;
    return jointCreate(type, bodyA, bodyB, frame);
}

PhysicsJointId PhysicsServer3D::jointCreate(PhysicsJointType type, PhysicsBodyId bodyA,
        PhysicsBodyId bodyB, const PhysicsJointFrame& frame)
{
    Impl& s = *mImpl;
    PhysicsBodyTag* a = s.body(bodyA);
    PhysicsBodyTag* b = s.body(bodyB);
    if (!a && !b) return PhysicsJointId();
    if (type == PhysicsJointType::Mouse && !a) return PhysicsJointId();
    kp::RigidBody& first = a ? *a->body : *s.anchor;
    kp::RigidBody& second = b ? *b->body : *s.anchor;
    if (&first == &second && type != PhysicsJointType::Mouse) return PhysicsJointId();
    PhysicsJointTag record;
    record.type = type;
    record.bodyA = a ? bodyA : PhysicsBodyId();
    record.bodyB = b && type != PhysicsJointType::Mouse ? bodyB : PhysicsBodyId();
    const Math::Vec3 axis = frame.axis.NormalizedSafe();
    const Math::Vec3 axisB = frame.axisB.NormalizedSafe();
    switch (type)
    {
        case PhysicsJointType::Pin:
            record.joint = new kp::PointJoint(first, second, frame.anchor);
            break;
        case PhysicsJointType::Hinge:
            record.joint = new kp::HingeJoint(first, second, frame.anchor, axis);
            break;
        case PhysicsJointType::Distance:
            record.joint = new kp::DistanceJoint(first, frame.anchor, second, frame.anchorB);
            break;
        case PhysicsJointType::Fixed:
            record.joint = new kp::FixedJoint(first, second, frame.anchor);
            break;
        case PhysicsJointType::Slider:
            record.joint = new kp::SliderJoint(first, second, frame.anchor, axis);
            break;
        case PhysicsJointType::Piston:
            record.joint = new kp::PistonJoint(first, second, frame.anchor, axis);
            break;
        case PhysicsJointType::Universal:
            record.joint = new kp::UniversalJoint(first, second, frame.anchor, axis, axisB);
            break;
        case PhysicsJointType::Wheel:
            record.joint = new kp::WheelJoint(first, second, frame.anchor, axis, axisB);
            break;
        case PhysicsJointType::Mouse:
            record.joint = new kp::MouseJoint(first, frame.anchor);
            break;
        case PhysicsJointType::ConeTwist:
            record.joint = new kp::ConeTwistJoint(first, second, frame.anchor, axis, axisB);
            break;
    }
    if (!record.joint) return PhysicsJointId();
    s.world.addJoint(record.joint);
    return s.joints.insert(static_cast<PhysicsJointTag&&>(record));
}

void PhysicsServer3D::jointFree(PhysicsJointId joint)
{
    Impl& s = *mImpl;
    PhysicsJointTag* record = s.joints.get(joint);
    if (!record || record->dying) return;
    if (s.locked)
    {
        record->dying = true;
        record->joint->setEnabled(false);
        s.jointFreeQueue.push_back(joint);
        return;
    }
    s.destroyJointNow(joint);
}

bool PhysicsServer3D::jointValid(PhysicsJointId joint) const
{
    const PhysicsJointTag* record = mImpl->joints.get(joint);
    return record && !record->dying;
}

PhysicsJointType PhysicsServer3D::jointGetType(PhysicsJointId joint) const
{
    const PhysicsJointTag* record = mImpl->joints.get(joint);
    return record ? record->type : PhysicsJointType::Pin;
}

void PhysicsServer3D::jointSetEnabled(PhysicsJointId joint, bool enabled)
{
    PhysicsJointTag* record = mImpl->joints.get(joint);
    if (record && !record->dying) record->joint->setEnabled(enabled);
}

namespace
{

PhysicsJointTag* liveJoint(PhysicsServer3D::Impl& s, PhysicsJointId id)
{
    PhysicsJointTag* record = s.joints.get(id);
    return record && !record->dying ? record : nullptr;
}

} // namespace

void PhysicsServer3D::jointSetLimits(PhysicsJointId joint, float lower, float upper)
{
    PhysicsJointTag* record = liveJoint(*mImpl, joint);
    if (!record) return;
    switch (record->type)
    {
        case PhysicsJointType::Hinge:
            static_cast<kp::HingeJoint*>(record->joint)->setLimits(lower, upper);
            break;
        case PhysicsJointType::Slider:
            static_cast<kp::SliderJoint*>(record->joint)->setLimits(lower, upper);
            break;
        case PhysicsJointType::ConeTwist:
            static_cast<kp::ConeTwistJoint*>(record->joint)->setTwistLimits(lower, upper);
            break;
        case PhysicsJointType::Piston:
            static_cast<kp::PistonJoint*>(record->joint)->setLinearLimits(lower, upper);
            break;
        case PhysicsJointType::Universal:
            static_cast<kp::UniversalJoint*>(record->joint)->setLimitsA(lower, upper);
            break;
        case PhysicsJointType::Wheel:
            static_cast<kp::WheelJoint*>(record->joint)->setSteeringLimits(lower, upper);
            break;
        default:
            break;
    }
}

void PhysicsServer3D::jointSetSecondaryLimits(PhysicsJointId joint, float lower, float upper)
{
    PhysicsJointTag* record = liveJoint(*mImpl, joint);
    if (!record) return;
    switch (record->type)
    {
        case PhysicsJointType::Piston:
            static_cast<kp::PistonJoint*>(record->joint)->setAngularLimits(lower, upper);
            break;
        case PhysicsJointType::Universal:
            static_cast<kp::UniversalJoint*>(record->joint)->setLimitsB(lower, upper);
            break;
        case PhysicsJointType::ConeTwist:
            static_cast<kp::ConeTwistJoint*>(record->joint)->setSwingLimits(lower, upper);
            break;
        default:
            break;
    }
}

void PhysicsServer3D::jointSetMotor(PhysicsJointId joint, float targetVelocity, float maxForce)
{
    PhysicsJointTag* record = liveJoint(*mImpl, joint);
    if (!record) return;
    switch (record->type)
    {
        case PhysicsJointType::Hinge:
            static_cast<kp::HingeJoint*>(record->joint)->setMotor(targetVelocity, maxForce);
            break;
        case PhysicsJointType::Slider:
            static_cast<kp::SliderJoint*>(record->joint)->setMotor(targetVelocity, maxForce);
            break;
        case PhysicsJointType::Piston:
            static_cast<kp::PistonJoint*>(record->joint)->setLinearMotor(targetVelocity, maxForce);
            break;
        case PhysicsJointType::Universal:
            static_cast<kp::UniversalJoint*>(record->joint)->setMotorA(targetVelocity, maxForce);
            break;
        case PhysicsJointType::Wheel:
            static_cast<kp::WheelJoint*>(record->joint)->setSpinMotor(targetVelocity, maxForce);
            break;
        default:
            break;
    }
}

void PhysicsServer3D::jointSetSecondaryMotor(PhysicsJointId joint, float targetVelocity,
        float maxForce)
{
    PhysicsJointTag* record = liveJoint(*mImpl, joint);
    if (!record) return;
    switch (record->type)
    {
        case PhysicsJointType::Piston:
            static_cast<kp::PistonJoint*>(record->joint)->setAngularMotor(targetVelocity, maxForce);
            break;
        case PhysicsJointType::Universal:
            static_cast<kp::UniversalJoint*>(record->joint)->setMotorB(targetVelocity, maxForce);
            break;
        case PhysicsJointType::Wheel:
            static_cast<kp::WheelJoint*>(record->joint)->setSteeringMotor(targetVelocity, maxForce);
            break;
        default:
            break;
    }
}

void PhysicsServer3D::jointSetPinMotor(PhysicsJointId joint, const Math::Vec3& worldAxis,
        float targetAngularVelocity, float maxTorque)
{
    Impl& s = *mImpl;
    PhysicsJointTag* record = liveJoint(s, joint);
    if (!record || record->type != PhysicsJointType::Pin) return;
    Math::Vec3 local = worldAxis.NormalizedSafe();
    if (const PhysicsBodyTag* a = s.body(record->bodyA))
        local = (s.bodyRotation(*a).Conjugate() * local).NormalizedSafe();
    static_cast<kp::PointJoint*>(record->joint)->setMotor(local, targetAngularVelocity, maxTorque);
}

void PhysicsServer3D::jointDisableMotor(PhysicsJointId joint)
{
    PhysicsJointTag* record = liveJoint(*mImpl, joint);
    if (!record) return;
    switch (record->type)
    {
        case PhysicsJointType::Pin:
            static_cast<kp::PointJoint*>(record->joint)->disableMotor();
            break;
        case PhysicsJointType::Hinge:
            static_cast<kp::HingeJoint*>(record->joint)->disableMotor();
            break;
        case PhysicsJointType::Slider:
            static_cast<kp::SliderJoint*>(record->joint)->disableMotor();
            break;
        case PhysicsJointType::Piston:
            static_cast<kp::PistonJoint*>(record->joint)->disableLinearMotor();
            break;
        case PhysicsJointType::Universal:
            static_cast<kp::UniversalJoint*>(record->joint)->disableMotorA();
            break;
        case PhysicsJointType::Wheel:
            static_cast<kp::WheelJoint*>(record->joint)->disableSpinMotor();
            break;
        default:
            break;
    }
}

void PhysicsServer3D::jointDisableSecondaryMotor(PhysicsJointId joint)
{
    PhysicsJointTag* record = liveJoint(*mImpl, joint);
    if (!record) return;
    switch (record->type)
    {
        case PhysicsJointType::Piston:
            static_cast<kp::PistonJoint*>(record->joint)->disableAngularMotor();
            break;
        case PhysicsJointType::Universal:
            static_cast<kp::UniversalJoint*>(record->joint)->disableMotorB();
            break;
        case PhysicsJointType::Wheel:
            static_cast<kp::WheelJoint*>(record->joint)->disableSteeringMotor();
            break;
        default:
            break;
    }
}

Math::Vec3 PhysicsServer3D::jointGetReactionForce(PhysicsJointId joint) const
{
    const PhysicsJointTag* record = mImpl->joints.get(joint);
    if (!record || record->dying || !record->joint) return Math::Vec3(0.0f, 0.0f, 0.0f);
    return record->joint->reactionForce();
}

float PhysicsServer3D::jointGetPosition(PhysicsJointId joint) const
{
    const PhysicsJointTag* record = mImpl->joints.get(joint);
    if (!record || record->dying) return 0.0f;
    switch (record->type)
    {
        case PhysicsJointType::Hinge:
            return static_cast<const kp::HingeJoint*>(record->joint)->currentAngle();
        case PhysicsJointType::Slider:
            return static_cast<const kp::SliderJoint*>(record->joint)->currentPosition();
        case PhysicsJointType::Piston:
            return static_cast<const kp::PistonJoint*>(record->joint)->currentPosition();
        case PhysicsJointType::Universal:
            return static_cast<const kp::UniversalJoint*>(record->joint)->currentAngleA();
        case PhysicsJointType::Wheel:
            return static_cast<const kp::WheelJoint*>(record->joint)->suspensionTravel();
        default:
            return 0.0f;
    }
}

float PhysicsServer3D::jointGetSecondaryPosition(PhysicsJointId joint) const
{
    const PhysicsJointTag* record = mImpl->joints.get(joint);
    if (!record || record->dying) return 0.0f;
    switch (record->type)
    {
        case PhysicsJointType::Piston:
            return static_cast<const kp::PistonJoint*>(record->joint)->currentAngle();
        case PhysicsJointType::Universal:
            return static_cast<const kp::UniversalJoint*>(record->joint)->currentAngleB();
        case PhysicsJointType::Wheel:
            return static_cast<const kp::WheelJoint*>(record->joint)->steeringAngle();
        default:
            return 0.0f;
    }
}

void PhysicsServer3D::jointSetDistance(PhysicsJointId joint, float minimum, float maximum)
{
    PhysicsJointTag* record = liveJoint(*mImpl, joint);
    if (!record || record->type != PhysicsJointType::Distance) return;
    static_cast<kp::DistanceJoint*>(record->joint)->setDistance(minimum, maximum);
}

void PhysicsServer3D::jointSetSuspension(PhysicsJointId joint, float restLength, float stiffness,
        float damping)
{
    PhysicsJointTag* record = liveJoint(*mImpl, joint);
    if (!record || record->type != PhysicsJointType::Wheel) return;
    static_cast<kp::WheelJoint*>(record->joint)->setSuspension(restLength, stiffness, damping);
}

void PhysicsServer3D::jointSetSteeringServo(PhysicsJointId joint, float targetAngle,
        float maxTorque, float maxAngularVelocity)
{
    PhysicsJointTag* record = liveJoint(*mImpl, joint);
    if (!record || record->type != PhysicsJointType::Wheel) return;
    static_cast<kp::WheelJoint*>(record->joint)
            ->setSteeringServo(targetAngle, maxTorque, maxAngularVelocity);
}

void PhysicsServer3D::jointDisableSteeringServo(PhysicsJointId joint)
{
    PhysicsJointTag* record = liveJoint(*mImpl, joint);
    if (!record || record->type != PhysicsJointType::Wheel) return;
    static_cast<kp::WheelJoint*>(record->joint)->disableSteeringServo();
}

void PhysicsServer3D::jointSetTarget(PhysicsJointId joint, const Math::Vec3& target)
{
    PhysicsJointTag* record = liveJoint(*mImpl, joint);
    if (!record || record->type != PhysicsJointType::Mouse) return;
    static_cast<kp::MouseJoint*>(record->joint)->setTarget(target);
}

void PhysicsServer3D::jointSetMouseSpring(PhysicsJointId joint, float maxForce, float frequencyHz,
        float dampingRatio)
{
    PhysicsJointTag* record = liveJoint(*mImpl, joint);
    if (!record || record->type != PhysicsJointType::Mouse) return;
    kp::MouseJoint* mouse = static_cast<kp::MouseJoint*>(record->joint);
    mouse->setMaxForce(maxForce);
    mouse->tuneSpring(frequencyHz, dampingRatio);
}

PhysicsVehicleId PhysicsServer3D::vehicleCreate(PhysicsBodyId chassis)
{
    Impl& s = *mImpl;
    PhysicsBodyTag* record = s.body(chassis);
    if (!record || record->libId == kNoLibId || record->mode != PhysicsBodyMode::Rigid)
        return PhysicsVehicleId();
    PhysicsVehicleTag vehicle;
    vehicle.chassis = chassis;
    vehicle.vehicle = new kp::RaycastVehicle(*record->body, &s.world, record->libId);
    return s.vehicles.insert(static_cast<PhysicsVehicleTag&&>(vehicle));
}

void PhysicsServer3D::vehicleFree(PhysicsVehicleId vehicle)
{
    Impl& s = *mImpl;
    PhysicsVehicleTag* record = s.vehicles.get(vehicle);
    if (!record || record->dying) return;
    if (s.locked)
    {
        record->dying = true;
        s.vehicleFreeQueue.push_back(vehicle);
        return;
    }
    s.destroyVehicleNow(vehicle);
}

bool PhysicsServer3D::vehicleValid(PhysicsVehicleId vehicle) const
{
    const PhysicsVehicleTag* record = mImpl->vehicles.get(vehicle);
    return record && !record->dying;
}

PhysicsGroundVehicleId PhysicsServer3D::groundVehicleCreatePassengerCar(PhysicsBodyId chassis,
        const PhysicsPassengerCarSettings& settings, PhysicsGroundVehicleLayout& layout)
{
    Impl& s = *mImpl;
    PhysicsBodyTag* record = s.body(chassis);
    if (!record || record->libId == kNoLibId || record->mode != PhysicsBodyMode::Rigid)
        return PhysicsGroundVehicleId();
    PhysicsGroundVehicleTag vehicle;
    vehicle.chassis = chassis;
    vehicle.vehicle = new kp::GroundVehicle();
    kp::PassengerCarOptions options;
    options.drive = settings.drive == PhysicsDriveLayout::FrontWheel ? kp::DriveLayout::FrontWheel
                    : settings.drive == PhysicsDriveLayout::AllWheel ? kp::DriveLayout::AllWheel
                                                                     : kp::DriveLayout::RearWheel;
    options.gearbox = settings.automatic ? kp::GearboxMode::Automatic : kp::GearboxMode::Manual;
    options.antiLock = settings.antiLock;
    const kp::GroundVehicleLayout built =
            kp::buildPassengerCar(*vehicle.vehicle, *record->body, options);
    const Math::Mat3 inertia = record->body->inertiaTensor();
    record->mass = built.mass;
    record->customInertia = Math::Vec3(inertia.col0.x, inertia.col1.y, inertia.col2.z);
    record->canSleep = false;
    record->body->setCanSleep(false);
    s.applyMass(*record);
    vehicle.vehicle->setBody(record->body, record->libId);
    vehicle.vehicle->setWorld(&s.world);
    vehicle.restHeight = built.restHeight;
    vehicle.engine = vehicle.vehicle->engine().parameters();
    vehicle.gearbox = vehicle.vehicle->gearbox().parameters();
    vehicle.clutch = vehicle.vehicle->clutch().parameters();
    for (std::uint32_t a = 0; a < vehicle.vehicle->axleCount(); ++a)
    {
        vehicle.tyreFriction.push_back(vehicle.vehicle->axle(a).wheel.tyre.friction);
        vehicle.handbrakeTorque.push_back(vehicle.vehicle->axle(a).wheel.maxHandbrakeTorque);
    }
    vehicle.wheelLocal.resize(vehicle.vehicle->wheelCount(), Math::Mat4::Identity());
    for (std::uint32_t w = 0; w < vehicle.vehicle->wheelCount(); ++w)
    {
        const kp::AxleParameters& axle = vehicle.vehicle->axle(w / 2);
        const float side = (w & 1u) ? -1.0f : 1.0f;
        vehicle.wheelLocal[w] = Math::Mat4::Translation(Math::Vec3(side * axle.halfTrack,
                axle.wheel.radius - built.restHeight, axle.position));
    }
    layout.mass = built.mass;
    layout.collisionHalfExtents = built.collisionHalfExtents;
    layout.restHeight = built.restHeight;
    layout.wheelbase = built.wheelbase;
    layout.track = built.track;
    return s.groundVehicles.insert(static_cast<PhysicsGroundVehicleTag&&>(vehicle));
}

void PhysicsServer3D::groundVehicleFree(PhysicsGroundVehicleId vehicle)
{
    Impl& s = *mImpl;
    PhysicsGroundVehicleTag* record = s.groundVehicles.get(vehicle);
    if (!record || record->dying) return;
    if (s.locked)
    {
        record->dying = true;
        s.groundVehicleFreeQueue.push_back(vehicle);
        return;
    }
    s.destroyGroundVehicleNow(vehicle);
}

bool PhysicsServer3D::groundVehicleValid(PhysicsGroundVehicleId vehicle) const
{
    const PhysicsGroundVehicleTag* record = mImpl->groundVehicles.get(vehicle);
    return record && !record->dying;
}

namespace
{

PhysicsGroundVehicleTag* liveGroundVehicle(ct::SlotMap32<PhysicsGroundVehicleTag>& vehicles,
        PhysicsGroundVehicleId id)
{
    PhysicsGroundVehicleTag* record = vehicles.get(id);
    return record && !record->dying && record->vehicle ? record : nullptr;
}

} // namespace

void PhysicsServer3D::groundVehicleSetRayMask(PhysicsGroundVehicleId vehicle, std::uint32_t mask)
{
    PhysicsGroundVehicleTag* record = liveGroundVehicle(mImpl->groundVehicles, vehicle);
    if (!record) return;
    kp::QueryFilter rays;
    rays.collision.mask = mask;
    record->vehicle->setRayFilter(rays);
}

void PhysicsServer3D::groundVehicleSetControls(PhysicsGroundVehicleId vehicle,
        const PhysicsGroundVehicleControls& controls)
{
    PhysicsGroundVehicleTag* record = liveGroundVehicle(mImpl->groundVehicles, vehicle);
    if (!record) return;
    kp::VehicleControls input;
    input.throttle = controls.throttle;
    input.brake = controls.brake;
    input.handbrake = controls.handbrake;
    input.steering = controls.steering;
    record->vehicle->setControls(input);
}

bool PhysicsServer3D::groundVehicleGetState(PhysicsGroundVehicleId vehicle,
        PhysicsGroundVehicleState& state) const
{
    const PhysicsGroundVehicleTag* record = liveGroundVehicle(mImpl->groundVehicles, vehicle);
    if (!record) return false;
    const kp::VehicleTelemetry& telemetry = record->vehicle->telemetry();
    state.forwardSpeed = telemetry.forwardSpeed;
    state.lateralSpeed = telemetry.lateralSpeed;
    state.groundSpeed = telemetry.groundSpeed;
    state.engineRpm = telemetry.engineRpm;
    state.gear = telemetry.gear;
    state.steeringAngle = telemetry.steeringAngle;
    return true;
}

int PhysicsServer3D::groundVehicleGetWheelCount(PhysicsGroundVehicleId vehicle) const
{
    const PhysicsGroundVehicleTag* record = liveGroundVehicle(mImpl->groundVehicles, vehicle);
    return record ? static_cast<int>(record->vehicle->wheelCount()) : 0;
}

bool PhysicsServer3D::groundVehicleGetWheelState(PhysicsGroundVehicleId vehicle, int wheel,
        PhysicsGroundWheelState& state) const
{
    const PhysicsGroundVehicleTag* record = liveGroundVehicle(mImpl->groundVehicles, vehicle);
    if (!record || wheel < 0 || static_cast<std::uint32_t>(wheel) >= record->vehicle->wheelCount())
        return false;
    const std::uint32_t index = static_cast<std::uint32_t>(wheel);
    const kp::WheelTelemetry& telemetry = record->vehicle->wheel(index);
    const kp::AxleParameters& axle = record->vehicle->axle(index / 2);
    const float side = (index & 1u) ? -1.0f : 1.0f;
    state.local =
            index < record->wheelLocal.size() ? record->wheelLocal[index] : Math::Mat4::Identity();
    state.inContact = telemetry.inContact;
    state.groundBody = mImpl->handleOfLib(telemetry.groundBody);
    state.contactPoint = telemetry.contactPoint;
    state.contactNormal = telemetry.contactNormal;
    state.radius = axle.wheel.radius;
    state.radiusScale = record->vehicle->wheelRadiusScale(index);
    state.halfTrack = axle.halfTrack;
    state.axlePosition = axle.position;
    state.slipRatio = telemetry.slipRatio;
    state.slipAngle = telemetry.slipAngle;
    state.angularVelocity = telemetry.angularVelocity;
    state.load = telemetry.load;
    const kp::TyreParameters& tyre = axle.wheel.tyre;
    const float alongPeak =
            tyre.peakSlipRatio > 0.0f ? telemetry.slipRatio / tyre.peakSlipRatio : 0.0f;
    const float acrossPeak =
            tyre.peakSlipAngle > 0.0f ? telemetry.slipAngle / tyre.peakSlipAngle : 0.0f;
    const float beyond = sqrtf(alongPeak * alongPeak + acrossPeak * acrossPeak) - 1.0f;
    state.slide = beyond < 0.0f ? 0.0f : (beyond > 1.0f ? 1.0f : beyond);
    state.detached = record->vehicle->wheelDetached(index);
    state.rest = Math::Mat4::Translation(Math::Vec3(side * axle.halfTrack,
            axle.wheel.radius - record->restHeight, axle.position));
    return true;
}

void PhysicsServer3D::groundVehicleSetWheelDetached(PhysicsGroundVehicleId vehicle, int wheel,
        bool detached)
{
    PhysicsGroundVehicleTag* record = liveGroundVehicle(mImpl->groundVehicles, vehicle);
    if (record && wheel >= 0 && static_cast<std::uint32_t>(wheel) < record->vehicle->wheelCount())
        record->vehicle->setWheelDetached(static_cast<std::uint32_t>(wheel), detached);
}

void PhysicsServer3D::groundVehicleSetWheelRadiusScale(PhysicsGroundVehicleId vehicle, int wheel,
        float scale)
{
    PhysicsGroundVehicleTag* record = liveGroundVehicle(mImpl->groundVehicles, vehicle);
    if (record && wheel >= 0 && static_cast<std::uint32_t>(wheel) < record->vehicle->wheelCount())
        record->vehicle->setWheelRadiusScale(static_cast<std::uint32_t>(wheel), scale);
}

void PhysicsServer3D::groundVehicleStopWheels(PhysicsGroundVehicleId vehicle)
{
    PhysicsGroundVehicleTag* record = liveGroundVehicle(mImpl->groundVehicles, vehicle);
    if (record) record->vehicle->stopWheels();
}

void PhysicsServer3D::groundVehicleSetAxle(PhysicsGroundVehicleId vehicle, int axle, float position,
        float halfTrack, float radius)
{
    PhysicsGroundVehicleTag* record = liveGroundVehicle(mImpl->groundVehicles, vehicle);
    if (!record || axle < 0 || static_cast<std::uint32_t>(axle) >= record->vehicle->axleCount())
        return;
    kp::AxleParameters& a = record->vehicle->axle(static_cast<std::uint32_t>(axle));
    a.position = position;
    if (halfTrack > 0.01f) a.halfTrack = halfTrack;
    if (radius > 0.01f)
    {
        record->restHeight += radius - a.wheel.radius;
        a.wheel.radius = radius;
    }
    for (std::uint32_t w = 0; w < record->vehicle->wheelCount(); ++w)
    {
        const kp::AxleParameters& each = record->vehicle->axle(w / 2);
        const float side = (w & 1u) ? -1.0f : 1.0f;
        if (w < record->wheelLocal.size())
            record->wheelLocal[w] = Math::Mat4::Translation(Math::Vec3(side * each.halfTrack,
                    each.wheel.radius - record->restHeight, each.position));
    }
}

float PhysicsServer3D::groundVehicleGetRestHeight(PhysicsGroundVehicleId vehicle) const
{
    PhysicsGroundVehicleTag* record = liveGroundVehicle(mImpl->groundVehicles, vehicle);
    return record ? record->restHeight : 0.0f;
}

void PhysicsServer3D::groundVehicleSetPowerScale(PhysicsGroundVehicleId vehicle, float scale)
{
    PhysicsGroundVehicleTag* record = liveGroundVehicle(mImpl->groundVehicles, vehicle);
    if (!record) return;
    kp::EngineParameters parameters = record->engine;
    const float factor = scale > 0.01f ? scale : 0.01f;
    for (std::uint32_t i = 0; i < kp::EngineParameters::MaxCurvePoints; ++i)
        parameters.curveTorque[i] *= factor;
    parameters.limiterBand = record->engine.limiterBand * factor;
    record->vehicle->engine().setParameters(parameters);
    kp::ClutchParameters clutch = record->clutch;
    clutch.maxTorque *= factor;
    record->vehicle->clutch().setParameters(clutch);
}

void PhysicsServer3D::groundVehicleSetTopSpeed(PhysicsGroundVehicleId vehicle,
        float metresPerSecond)
{
    PhysicsGroundVehicleTag* record = liveGroundVehicle(mImpl->groundVehicles, vehicle);
    if (!record || record->vehicle->axleCount() == 0) return;
    kp::GearboxParameters parameters = record->gearbox;
    if (metresPerSecond > 0.1f && parameters.gearCount > 0)
    {
        const float top = parameters.ratios[parameters.gearCount - 1];
        const std::uint32_t lastWheel = record->vehicle->wheelCount() - 1;
        const float radius = record->vehicle->axle(record->vehicle->axleCount() - 1).wheel.radius *
                             record->vehicle->wheelRadiusScale(lastWheel);
        const float wheelRadiansPerSecond = metresPerSecond / radius;
        const float engineRadiansPerSecond = record->engine.limiterRpm * 2.0f * 3.14159265f / 60.0f;
        parameters.finalDrive = engineRadiansPerSecond / (wheelRadiansPerSecond * top);
    }
    record->vehicle->gearbox().setParameters(parameters);
}

void PhysicsServer3D::groundVehicleSetGrip(PhysicsGroundVehicleId vehicle, float scale)
{
    PhysicsGroundVehicleTag* record = liveGroundVehicle(mImpl->groundVehicles, vehicle);
    if (!record) return;
    kp::SurfaceInfo surface;
    surface.frictionScale = scale > 0.01f ? scale : 0.01f;
    record->vehicle->setDefaultSurface(surface);
}

void PhysicsServer3D::groundVehicleSetRollInfluence(PhysicsGroundVehicleId vehicle, float influence)
{
    PhysicsGroundVehicleTag* record = liveGroundVehicle(mImpl->groundVehicles, vehicle);
    if (!record) return;
    const float clamped = influence < 0.0f ? 0.0f : (influence > 1.0f ? 1.0f : influence);
    for (std::uint32_t a = 0; a < record->vehicle->axleCount(); ++a)
        record->vehicle->axle(a).rollInfluence = clamped;
}

void PhysicsServer3D::groundVehicleSetAxleGrip(PhysicsGroundVehicleId vehicle, int axle,
        float scale)
{
    PhysicsGroundVehicleTag* record = liveGroundVehicle(mImpl->groundVehicles, vehicle);
    if (!record || axle < 0 || static_cast<std::size_t>(axle) >= record->tyreFriction.size())
        return;
    const std::uint32_t index = static_cast<std::uint32_t>(axle);
    record->vehicle->axle(index).wheel.tyre.friction =
            record->tyreFriction[index] * (scale > 0.01f ? scale : 0.01f);
}

void PhysicsServer3D::groundVehicleSetHandbrakeTorque(PhysicsGroundVehicleId vehicle, float torque)
{
    PhysicsGroundVehicleTag* record = liveGroundVehicle(mImpl->groundVehicles, vehicle);
    if (!record) return;
    for (std::size_t a = 0; a < record->handbrakeTorque.size(); ++a)
    {
        if (record->handbrakeTorque[a] <= 0.0f) continue;
        record->vehicle->axle(static_cast<std::uint32_t>(a)).wheel.maxHandbrakeTorque =
                torque > 0.0f ? torque : record->handbrakeTorque[a];
    }
}

void PhysicsServer3D::groundVehicleSelectGear(PhysicsGroundVehicleId vehicle, int gear)
{
    PhysicsGroundVehicleTag* record = liveGroundVehicle(mImpl->groundVehicles, vehicle);
    if (record) record->vehicle->selectGear(gear);
}

namespace
{

void fillWheel(kp::RaycastVehicle::Wheel& wheel, const PhysicsWheelParameters& p)
{
    wheel.chassisConnectionLocal = p.connection;
    wheel.directionLocal = p.direction;
    wheel.axleLocal = p.axle;
    wheel.restLength = p.restLength;
    wheel.radius = p.radius;
    wheel.stiffness = p.stiffness;
    wheel.dampingCompression = p.dampingCompression;
    wheel.dampingRelaxation = p.dampingRelaxation;
    wheel.frictionSlip = p.frictionSlip;
    wheel.rollInfluence = p.rollInfluence;
    wheel.maxSuspensionTravelCm = p.travel * 100.0f;
    wheel.maxSuspensionForce = p.maxForce;
    wheel.isFrontWheel = p.front;
}

} // namespace

int PhysicsServer3D::vehicleAddWheel(PhysicsVehicleId vehicle,
        const PhysicsWheelParameters& parameters)
{
    PhysicsVehicleTag* record = mImpl->vehicles.get(vehicle);
    if (!record || record->dying) return -1;
    kp::RaycastVehicle::Tuning tuning;
    const std::uint32_t index =
            record->vehicle->addWheel(parameters.connection, parameters.direction, parameters.axle,
                    parameters.restLength, parameters.radius, tuning, parameters.front);
    fillWheel(record->vehicle->wheel(index), parameters);
    return static_cast<int>(index);
}

int PhysicsServer3D::vehicleGetWheelCount(PhysicsVehicleId vehicle) const
{
    const PhysicsVehicleTag* record = mImpl->vehicles.get(vehicle);
    return record && !record->dying ? static_cast<int>(record->vehicle->wheelCount()) : 0;
}

void PhysicsServer3D::vehicleSetWheelParameters(PhysicsVehicleId vehicle, int wheel,
        const PhysicsWheelParameters& parameters)
{
    PhysicsVehicleTag* record = mImpl->vehicles.get(vehicle);
    if (!record || record->dying || wheel < 0 ||
            static_cast<std::uint32_t>(wheel) >= record->vehicle->wheelCount())
        return;
    fillWheel(record->vehicle->wheel(static_cast<std::uint32_t>(wheel)), parameters);
}

void PhysicsServer3D::vehicleSetEngineForce(PhysicsVehicleId vehicle, int wheel, float force)
{
    PhysicsVehicleTag* record = mImpl->vehicles.get(vehicle);
    if (record && !record->dying && wheel >= 0 &&
            static_cast<std::uint32_t>(wheel) < record->vehicle->wheelCount())
        record->vehicle->setEngineForce(force, static_cast<std::uint32_t>(wheel));
}

void PhysicsServer3D::vehicleSetBrake(PhysicsVehicleId vehicle, int wheel, float brake)
{
    PhysicsVehicleTag* record = mImpl->vehicles.get(vehicle);
    if (record && !record->dying && wheel >= 0 &&
            static_cast<std::uint32_t>(wheel) < record->vehicle->wheelCount())
        record->vehicle->setBrake(brake, static_cast<std::uint32_t>(wheel));
}

void PhysicsServer3D::vehicleSetSteering(PhysicsVehicleId vehicle, int wheel, float steering)
{
    PhysicsVehicleTag* record = mImpl->vehicles.get(vehicle);
    if (record && !record->dying && wheel >= 0 &&
            static_cast<std::uint32_t>(wheel) < record->vehicle->wheelCount())
        record->vehicle->setSteering(steering, static_cast<std::uint32_t>(wheel));
}

bool PhysicsServer3D::vehicleGetWheelState(PhysicsVehicleId vehicle, int wheel,
        PhysicsWheelState& state) const
{
    const Impl& s = *mImpl;
    const PhysicsVehicleTag* record = s.vehicles.get(vehicle);
    if (!record || record->dying || wheel < 0 ||
            static_cast<std::uint32_t>(wheel) >= record->vehicle->wheelCount())
        return false;
    const kp::RaycastVehicle::Wheel& source =
            record->vehicle->wheel(static_cast<std::uint32_t>(wheel));
    state.worldTransform = source.worldTransform;
    state.inContact = source.inContact;
    state.groundBody = source.inContact ? s.handleOfLib(source.groundBody) : PhysicsBodyId();
    state.contactPoint = source.contactPoint;
    state.contactNormal = source.contactNormal;
    state.suspensionLength = source.suspensionLength;
    state.rotation = source.rotation;
    state.skidInfo = source.skidInfo;
    return true;
}

float PhysicsServer3D::vehicleGetSpeedKmh(PhysicsVehicleId vehicle) const
{
    const PhysicsVehicleTag* record = mImpl->vehicles.get(vehicle);
    return record && !record->dying ? record->vehicle->currentSpeedKmHour() : 0.0f;
}

void PhysicsServer3D::vehicleResetSuspension(PhysicsVehicleId vehicle)
{
    PhysicsVehicleTag* record = mImpl->vehicles.get(vehicle);
    if (record && !record->dying) record->vehicle->resetSuspension();
}

bool PhysicsServer3D::vehicleEnableMotorcycle(PhysicsVehicleId vehicle, bool enabled)
{
    Impl& s = *mImpl;
    PhysicsVehicleTag* record = s.vehicles.get(vehicle);
    if (!record || record->dying) return false;
    if (!enabled)
    {
        delete record->motorcycle;
        record->motorcycle = nullptr;
        return true;
    }
    if (record->motorcycle) return true;
    PhysicsBodyTag* chassis = s.body(record->chassis);
    if (!chassis || !chassis->body) return false;
    record->motorcycle = new kp::MotorcycleController(*record->vehicle, *chassis->body);
    return true;
}

bool PhysicsServer3D::vehicleIsMotorcycle(PhysicsVehicleId vehicle) const
{
    const PhysicsVehicleTag* record = mImpl->vehicles.get(vehicle);
    return record && !record->dying && record->motorcycle != nullptr;
}

namespace
{

kp::MotorcycleController* motorcycleOf(PhysicsVehicleTag* record)
{
    return record && !record->dying ? record->motorcycle : nullptr;
}

} // namespace

void PhysicsServer3D::motorcycleSetSteerInput(PhysicsVehicleId vehicle, float input)
{
    if (kp::MotorcycleController* m = motorcycleOf(mImpl->vehicles.get(vehicle)))
        m->setSteerInput(input);
}

void PhysicsServer3D::motorcycleSetMaxSteerAngle(PhysicsVehicleId vehicle, float angle)
{
    if (kp::MotorcycleController* m = motorcycleOf(mImpl->vehicles.get(vehicle)))
        m->setMaxSteerAngle(angle);
}

void PhysicsServer3D::motorcycleSetCasterAngle(PhysicsVehicleId vehicle, float angle)
{
    if (kp::MotorcycleController* m = motorcycleOf(mImpl->vehicles.get(vehicle)))
        m->setCasterAngle(angle);
}

void PhysicsServer3D::motorcycleSetMaxLeanAngle(PhysicsVehicleId vehicle, float angle)
{
    if (kp::MotorcycleController* m = motorcycleOf(mImpl->vehicles.get(vehicle)))
        m->setMaxLeanAngle(angle);
}

void PhysicsServer3D::motorcycleSetLeanSpring(PhysicsVehicleId vehicle, float springConstant,
        float damping)
{
    if (kp::MotorcycleController* m = motorcycleOf(mImpl->vehicles.get(vehicle)))
        m->setLeanSpring(springConstant, damping);
}

void PhysicsServer3D::motorcycleSetLeanSpringIntegration(PhysicsVehicleId vehicle,
        float coefficient, float decay)
{
    if (kp::MotorcycleController* m = motorcycleOf(mImpl->vehicles.get(vehicle)))
        m->setLeanSpringIntegration(coefficient, decay);
}

void PhysicsServer3D::motorcycleSetLeanSmoothing(PhysicsVehicleId vehicle, float factor)
{
    if (kp::MotorcycleController* m = motorcycleOf(mImpl->vehicles.get(vehicle)))
        m->setLeanSmoothingFactor(factor);
}

void PhysicsServer3D::motorcycleSetLeanControllerEnabled(PhysicsVehicleId vehicle, bool enabled)
{
    if (kp::MotorcycleController* m = motorcycleOf(mImpl->vehicles.get(vehicle)))
        m->setLeanControllerEnabled(enabled);
}

float PhysicsServer3D::motorcycleGetLeanAngle(PhysicsVehicleId vehicle) const
{
    const PhysicsVehicleTag* record = mImpl->vehicles.get(vehicle);
    return record && !record->dying && record->motorcycle ? record->motorcycle->currentLeanAngle()
                                                          : 0.0f;
}

namespace
{

using HullEdge = kinetix::Geometry::ConvexHullComputer::Edge;

void triangulateFaces(const ct::Vector<Math::Vec3>& vertices, const ct::Vector<HullEdge>& edges,
        const ct::Vector<int>& faces, const Math::Vec3& inside, ct::Vector<std::uint32_t>& out)
{
    for (std::size_t f = 0; f < faces.size(); ++f)
    {
        const HullEdge* start = &edges[static_cast<std::size_t>(faces[f])];
        ct::Vector<std::uint32_t> loop;
        const HullEdge* edge = start;
        do
        {
            loop.push_back(static_cast<std::uint32_t>(edge->getSourceVertex()));
            edge = edge->getNextEdgeOfFace();
        } while (edge && edge != start && loop.size() < 4096);
        if (loop.size() < 3) continue;
        const Math::Vec3 a = vertices[loop[0]];
        const Math::Vec3 b = vertices[loop[1]];
        const Math::Vec3 c = vertices[loop[2]];
        const Math::Vec3 normal = Math::Vec3::Cross(b - a, c - a);
        const bool flip = Math::Vec3::Dot(normal, a - inside) < 0.0f;
        for (std::size_t i = 1; i + 1 < loop.size(); ++i)
        {
            out.push_back(loop[0]);
            out.push_back(loop[flip ? i + 1 : i]);
            out.push_back(loop[flip ? i : i + 1]);
        }
    }
}

class ParentTableView : public kp::SkeletonView
{
public:
    ParentTableView(const int* parents, std::size_t count)
            : mParents(parents),
              mCount(count)
    {
    }

    std::uint32_t boneCount() const override
    {
        return static_cast<std::uint32_t>(mCount);
    }

    std::int32_t boneParent(std::uint32_t index) const override
    {
        return index < mCount ? mParents[index] : -1;
    }

private:
    const int* mParents;
    std::size_t mCount;
};

void orientOutward(const ct::Vector<Math::Vec3>& vertices, ct::Vector<std::uint32_t>& indices,
        const Math::Vec3& inside)
{
    for (std::size_t i = 0; i + 2 < indices.size(); i += 3)
    {
        const Math::Vec3& a = vertices[indices[i]];
        const Math::Vec3 normal =
                Math::Vec3::Cross(vertices[indices[i + 1]] - a, vertices[indices[i + 2]] - a);
        if (Math::Vec3::Dot(normal, a - inside) < 0.0f)
        {
            const std::uint32_t swap = indices[i + 1];
            indices[i + 1] = indices[i + 2];
            indices[i + 2] = swap;
        }
    }
}

void proxyBox(const Math::Vec3& half, ct::Vector<Math::Vec3>& vertices,
        ct::Vector<std::uint32_t>& indices)
{
    const float extent[3] = { half.x, half.y, half.z };
    for (int axis = 0; axis < 3; ++axis)
    {
        const int u = (axis + 1) % 3;
        const int w = (axis + 2) % 3;
        for (int sign = -1; sign <= 1; sign += 2)
        {
            float corner[4][3] = {};
            const float uSigns[4] = { -1.0f, 1.0f, 1.0f, -1.0f };
            const float wSigns[4] = { -1.0f, -1.0f, 1.0f, 1.0f };
            const std::uint32_t base = static_cast<std::uint32_t>(vertices.size());
            for (int k = 0; k < 4; ++k)
            {
                corner[k][axis] = extent[axis] * static_cast<float>(sign);
                corner[k][u] = extent[u] * uSigns[k];
                corner[k][w] = extent[w] * wSigns[k];
                vertices.push_back(Math::Vec3(corner[k][0], corner[k][1], corner[k][2]));
            }
            indices.push_back(base);
            indices.push_back(base + 1);
            indices.push_back(base + 2);
            indices.push_back(base);
            indices.push_back(base + 2);
            indices.push_back(base + 3);
        }
    }
    orientOutward(vertices, indices, Math::Vec3(0.0f, 0.0f, 0.0f));
}

void proxyRings(const ct::Vector<Math::Vec3>& ringCenters, const ct::Vector<float>& ringPolar,
        float radius, int slices, ct::Vector<Math::Vec3>& vertices,
        ct::Vector<std::uint32_t>& indices)
{
    const std::size_t rings = ringCenters.size();
    for (std::size_t r = 0; r < rings; ++r)
        for (int s = 0; s < slices; ++s)
        {
            const float theta = 6.28318531f * static_cast<float>(s) / static_cast<float>(slices);
            const float ring = radius * sinf(ringPolar[r]);
            vertices.push_back(
                    ringCenters[r] + Math::Vec3(ring * cosf(theta), radius * cosf(ringPolar[r]),
                                             ring * sinf(theta)));
        }
    for (std::size_t r = 0; r + 1 < rings; ++r)
        for (int s = 0; s < slices; ++s)
        {
            const std::uint32_t next = static_cast<std::uint32_t>((s + 1) % slices);
            const std::uint32_t a = static_cast<std::uint32_t>(r * slices + s);
            const std::uint32_t b = static_cast<std::uint32_t>(r * slices) + next;
            const std::uint32_t c = static_cast<std::uint32_t>((r + 1) * slices + s);
            const std::uint32_t d = static_cast<std::uint32_t>((r + 1) * slices) + next;
            indices.push_back(a);
            indices.push_back(c);
            indices.push_back(b);
            indices.push_back(b);
            indices.push_back(c);
            indices.push_back(d);
        }
}

kp::TrimeshShape* buildSweepProxy(const kp::CollisionShape& shape)
{
    constexpr int kRings = 16;
    constexpr int kSlices = 32;
    ct::Vector<Math::Vec3> vertices;
    ct::Vector<std::uint32_t> indices;
    Math::Vec3 inside(0.0f, 0.0f, 0.0f);
    switch (shape.type())
    {
        case kp::ShapeType::Box:
            proxyBox(static_cast<const kp::BoxShape&>(shape).halfExtents(), vertices, indices);
            break;
        case kp::ShapeType::Sphere:
        {
            const float radius = static_cast<const kp::SphereShape&>(shape).radius();
            ct::Vector<Math::Vec3> centers;
            ct::Vector<float> polar;
            for (int r = 0; r <= kRings; ++r)
            {
                centers.push_back(Math::Vec3(0.0f, 0.0f, 0.0f));
                polar.push_back(3.14159265f * static_cast<float>(r) / static_cast<float>(kRings));
            }
            proxyRings(centers, polar, radius, kSlices, vertices, indices);
            orientOutward(vertices, indices, inside);
            break;
        }
        case kp::ShapeType::Capsule:
        {
            const kp::CapsuleShape& capsule = static_cast<const kp::CapsuleShape&>(shape);
            const int half = kRings / 2;
            ct::Vector<Math::Vec3> centers;
            ct::Vector<float> polar;
            for (int r = 0; r <= half; ++r)
            {
                centers.push_back(Math::Vec3(0.0f, capsule.halfHeight(), 0.0f));
                polar.push_back(1.57079633f * static_cast<float>(r) / static_cast<float>(half));
            }
            for (int r = 0; r <= half; ++r)
            {
                centers.push_back(Math::Vec3(0.0f, -capsule.halfHeight(), 0.0f));
                polar.push_back(1.57079633f +
                                1.57079633f * static_cast<float>(r) / static_cast<float>(half));
            }
            proxyRings(centers, polar, capsule.radius(), kSlices, vertices, indices);
            orientOutward(vertices, indices, inside);
            break;
        }
        case kp::ShapeType::Cylinder:
        {
            const kp::CylinderShape& cylinder = static_cast<const kp::CylinderShape&>(shape);
            const std::uint32_t sides = static_cast<std::uint32_t>(kSlices);
            const std::uint32_t top = static_cast<std::uint32_t>(2 * kSlices);
            for (std::uint32_t i = 0; i < sides; ++i)
            {
                const float angle = 6.28318531f * static_cast<float>(i) / static_cast<float>(sides);
                const float x = cosf(angle) * cylinder.radius();
                const float z = sinf(angle) * cylinder.radius();
                vertices.push_back(Math::Vec3(x, cylinder.halfHeight(), z));
                vertices.push_back(Math::Vec3(x, -cylinder.halfHeight(), z));
            }
            vertices.push_back(Math::Vec3(0.0f, cylinder.halfHeight(), 0.0f));
            vertices.push_back(Math::Vec3(0.0f, -cylinder.halfHeight(), 0.0f));
            for (std::uint32_t i = 0; i < sides; ++i)
            {
                const std::uint32_t next = (i + 1) % sides;
                const std::uint32_t a = i * 2;
                const std::uint32_t b = next * 2;
                indices.push_back(a);
                indices.push_back(a + 1);
                indices.push_back(b);
                indices.push_back(b);
                indices.push_back(a + 1);
                indices.push_back(b + 1);
                indices.push_back(top);
                indices.push_back(b);
                indices.push_back(a);
                indices.push_back(top + 1);
                indices.push_back(a + 1);
                indices.push_back(b + 1);
            }
            orientOutward(vertices, indices, inside);
            break;
        }
        case kp::ShapeType::Plane:
        {
            const kp::PlaneShape& plane = static_cast<const kp::PlaneShape&>(shape);
            const Math::Vec3 n = plane.normal();
            const Math::Vec3 helper =
                    fabsf(n.y) < 0.9f ? Math::Vec3(0.0f, 1.0f, 0.0f) : Math::Vec3(1.0f, 0.0f, 0.0f);
            const Math::Vec3 u = Math::Vec3::Cross(helper, n).Normalized();
            const Math::Vec3 w = Math::Vec3::Cross(n, u);
            const Math::Vec3 c = n * plane.constant();
            const float big = 4000.0f;
            vertices.push_back(c - u * big - w * big);
            vertices.push_back(c + u * big - w * big);
            vertices.push_back(c + u * big + w * big);
            vertices.push_back(c - u * big + w * big);
            const std::uint32_t quad[6] = { 0, 1, 2, 0, 2, 3 };
            for (std::uint32_t index: quad) indices.push_back(index);
            const Math::Vec3 normal =
                    Math::Vec3::Cross(vertices[1] - vertices[0], vertices[2] - vertices[0]);
            if (Math::Vec3::Dot(normal, n) < 0.0f)
            {
                indices[1] = 2;
                indices[2] = 1;
                indices[4] = 3;
                indices[5] = 2;
            }
            break;
        }
        case kp::ShapeType::ConvexHull:
        {
            const kp::ConvexHullShape& hull = static_cast<const kp::ConvexHullShape&>(shape);
            if (hull.vertices().size() < 4) return nullptr;
            for (std::size_t i = 0; i < hull.vertices().size(); ++i)
            {
                vertices.push_back(hull.vertices()[i]);
                inside = inside + hull.vertices()[i];
            }
            inside = inside * (1.0f / static_cast<float>(vertices.size()));
            triangulateFaces(vertices, hull.edges(), hull.faces(), inside, indices);
            break;
        }
        default:
            return nullptr;
    }
    if (indices.size() < 3) return nullptr;
    return new kp::TrimeshShape(vertices.data(), static_cast<std::uint32_t>(vertices.size()),
            indices.data(), static_cast<std::uint32_t>(indices.size()));
}

} // namespace

class PhysicsServer3D::Impl::WorldSweep final : public kp::CharacterSweepSource
{
public:
    WorldSweep(const Impl& impl, const kp::QueryFilter& filter, const Math::Vec3& up,
            ct::Vector<PhysicsCharacterHit>* hits)
            : mImpl(impl),
              mFilter(filter),
              mUp(up),
              mHits(hits)
    {
    }

    bool sweepEllipsoid(const Math::Vec3& centre, const Math::Mat3& axes, const Math::Vec3& radii,
            const Math::Vec3& velocity, kp::CharacterSweepHit& hit) const override;

private:
    const Impl& mImpl;
    kp::QueryFilter mFilter;
    Math::Vec3 mUp;
    ct::Vector<PhysicsCharacterHit>* mHits;
};

bool PhysicsServer3D::Impl::WorldSweep::sweepEllipsoid(const Math::Vec3& centre,
        const Math::Mat3& axes, const Math::Vec3& radii, const Math::Vec3& velocity,
        kp::CharacterSweepHit& hit) const
{
    const float largest = fmaxf(radii.x, fmaxf(radii.y, radii.z)) + 0.05f;
    const Math::Vec3 pad(largest, largest, largest);
    kinetix::AABB box;
    box.expand(centre - pad);
    box.expand(centre + pad);
    box.expand(centre + velocity - pad);
    box.expand(centre + velocity + pad);
    static thread_local ct::Vector<std::uint32_t> ids;
    ids.clear();
    mImpl.world.queryAABB(box, mFilter, ids);

    bool found = false;
    PhysicsBodyId nearest;
    for (std::size_t i = 0; i < ids.size(); ++i)
    {
        const PhysicsBodyId handle = mImpl.handleOfLib(ids[i]);
        const PhysicsBodyTag* record = mImpl.body(handle);
        if (!record || !record->instance || !record->body || !record->enabled) continue;
        const kp::CollisionShape* shape = record->instance->shape;
        const kp::TrimeshShape* mesh = nullptr;
        if (shape->type() == kp::ShapeType::Trimesh)
            mesh = static_cast<const kp::TrimeshShape*>(shape);
        else
        {
            if (!record->instance->proxy) record->instance->proxy = buildSweepProxy(*shape);
            mesh = record->instance->proxy;
        }
        if (!mesh) continue;
        const kp::TrimeshSweepSource source(*mesh, record->body->transform());
        kp::CharacterSweepHit local;
        if (!source.sweepEllipsoid(centre, axes, radii, velocity, local)) continue;
        if (found && local.t >= hit.t) continue;
        hit = local;
        found = true;
        nearest = handle;
    }

    if (found && mHits)
    {
        bool known = false;
        for (std::size_t i = 0; i < mHits->size(); ++i)
        {
            PhysicsCharacterHit& entry = (*mHits)[i];
            if (entry.body != nearest) continue;
            known = true;
            if (Math::Vec3::Dot(hit.normal, mUp) < Math::Vec3::Dot(entry.normal, mUp))
                entry.normal = hit.normal;
            break;
        }
        if (!known)
        {
            PhysicsCharacterHit entry;
            entry.body = nearest;
            entry.normal = hit.normal;
            mHits->push_back(entry);
        }
    }
    return found;
}

namespace
{

void applyCharacterParameters(kp::CharacterBody& body, const PhysicsCharacterParameters& p)
{
    body.setShape(p.radius, p.height);
    body.setSlopeLimit(p.slopeLimit);
    body.setSkinWidth(p.skinWidth);
    body.setStepOffset(p.stepOffset);
    body.setGroundSnapDistance(p.groundSnap);
    body.setWallMinSlideAngle(p.wallMinSlideAngle);
    body.setMaxIterations(static_cast<std::uint32_t>(p.maxIterations < 1 ? 1 : p.maxIterations));
    body.setFloorStopOnSlope(p.floorStopOnSlope);
    body.setSlideOnCeiling(p.slideOnCeiling);
    body.setUpDirection(p.up);
}

PhysicsCharacterState characterState(const kp::CharacterBody& body,
        const kp::CharacterBody::MoveResult& result)
{
    PhysicsCharacterState state;
    state.collided = result.collided;
    state.blocked = result.blocked;
    state.onFloor = body.isOnFloor();
    state.onWall = body.isOnWall();
    state.onCeiling = body.isOnCeiling();
    state.floorNormal = body.floorNormal();
    state.wallNormal = body.wallNormal();
    state.ceilingNormal = body.ceilingNormal();
    state.position = body.position();
    state.velocity = body.velocity();
    state.slopeAngle = body.slopeAngle();
    return state;
}

kp::QueryFilter characterFilter(const PhysicsCharacterParameters& parameters,
        const PhysicsBodyTag* ignored)
{
    kp::QueryFilter filter;
    filter.collision.group = parameters.collisionLayer;
    filter.collision.mask = parameters.collisionMask;
    if (ignored && ignored->libId != kNoLibId) filter.ignoredBody = ignored->libId;
    return filter;
}

} // namespace

PhysicsCharacterId PhysicsServer3D::characterCreate(const PhysicsCharacterParameters& parameters,
        const Math::Vec3& position)
{
    PhysicsCharacterTag record;
    record.body = new kp::CharacterBody();
    record.parameters = parameters;
    applyCharacterParameters(*record.body, parameters);
    record.body->setPosition(position);
    return mImpl->characters.insert(static_cast<PhysicsCharacterTag&&>(record));
}

void PhysicsServer3D::characterFree(PhysicsCharacterId character)
{
    Impl& s = *mImpl;
    PhysicsCharacterTag* record = s.characters.get(character);
    if (!record || record->dying) return;
    if (s.locked)
    {
        record->dying = true;
        s.characterFreeQueue.push_back(character);
        return;
    }
    s.destroyCharacterNow(character);
}

bool PhysicsServer3D::characterValid(PhysicsCharacterId character) const
{
    const PhysicsCharacterTag* record = mImpl->characters.get(character);
    return record && !record->dying;
}

void PhysicsServer3D::characterSetParameters(PhysicsCharacterId character,
        const PhysicsCharacterParameters& parameters)
{
    PhysicsCharacterTag* record = mImpl->characters.get(character);
    if (!record || record->dying) return;
    record->parameters = parameters;
    applyCharacterParameters(*record->body, parameters);
}

void PhysicsServer3D::characterSetPosition(PhysicsCharacterId character, const Math::Vec3& position)
{
    PhysicsCharacterTag* record = mImpl->characters.get(character);
    if (!record || record->dying) return;
    record->body->setPosition(position);
}

Math::Vec3 PhysicsServer3D::characterGetPosition(PhysicsCharacterId character) const
{
    const PhysicsCharacterTag* record = mImpl->characters.get(character);
    return record && !record->dying ? record->body->position() : Math::Vec3(0.0f, 0.0f, 0.0f);
}

void PhysicsServer3D::characterSetVelocity(PhysicsCharacterId character, const Math::Vec3& velocity)
{
    PhysicsCharacterTag* record = mImpl->characters.get(character);
    if (!record || record->dying) return;
    record->body->setVelocity(velocity);
}

Math::Vec3 PhysicsServer3D::characterGetVelocity(PhysicsCharacterId character) const
{
    const PhysicsCharacterTag* record = mImpl->characters.get(character);
    return record && !record->dying ? record->body->velocity() : Math::Vec3(0.0f, 0.0f, 0.0f);
}

void PhysicsServer3D::characterSetIgnoredBody(PhysicsCharacterId character, PhysicsBodyId body)
{
    PhysicsCharacterTag* record = mImpl->characters.get(character);
    if (!record || record->dying) return;
    record->ignored = body;
}

PhysicsCharacterState PhysicsServer3D::characterMoveAndSlide(PhysicsCharacterId character,
        float deltaTime)
{
    Impl& s = *mImpl;
    PhysicsCharacterTag* record = s.characters.get(character);
    if (!record || record->dying) return PhysicsCharacterState();
    record->hits.clear();
    const Impl::WorldSweep sweep(s, characterFilter(record->parameters, s.body(record->ignored)),
            record->parameters.up, &record->hits);
    const kp::CharacterBody::MoveResult result = record->body->moveAndSlide(deltaTime, sweep);
    return characterState(*record->body, result);
}

PhysicsCharacterState PhysicsServer3D::characterMove(PhysicsCharacterId character,
        const Math::Vec3& displacement)
{
    Impl& s = *mImpl;
    PhysicsCharacterTag* record = s.characters.get(character);
    if (!record || record->dying) return PhysicsCharacterState();
    record->hits.clear();
    const Impl::WorldSweep sweep(s, characterFilter(record->parameters, s.body(record->ignored)),
            record->parameters.up, &record->hits);
    const kp::CharacterBody::MoveResult result = record->body->move(displacement, sweep);
    return characterState(*record->body, result);
}

bool PhysicsServer3D::characterIsNearGround(PhysicsCharacterId character, float distance) const
{
    const Impl& s = *mImpl;
    const PhysicsCharacterTag* record = s.characters.get(character);
    if (!record || record->dying) return false;
    const Impl::WorldSweep sweep(s, characterFilter(record->parameters, s.body(record->ignored)),
            record->parameters.up, nullptr);
    return record->body->isNearGround(sweep, distance);
}

std::size_t PhysicsServer3D::characterGetHitCount(PhysicsCharacterId character) const
{
    const PhysicsCharacterTag* record = mImpl->characters.get(character);
    return record && !record->dying ? record->hits.size() : 0;
}

PhysicsCharacterHit PhysicsServer3D::characterGetHit(PhysicsCharacterId character,
        std::size_t index) const
{
    const PhysicsCharacterTag* record = mImpl->characters.get(character);
    if (!record || record->dying || index >= record->hits.size()) return PhysicsCharacterHit();
    return record->hits[index];
}

namespace
{

void applyCharacterRigidParameters(kp::CharacterRigidBody& controller,
        const PhysicsCharacterRigidParameters& p)
{
    controller.setMaxSlopeAngle(p.maxSlopeAngle);
    controller.setUp(p.up);
    kinetix::Plane volume;
    volume.normal = p.up;
    volume.d = -p.supportingHeight;
    controller.setSupportingVolume(volume);
}

} // namespace

PhysicsCharacterRigidId PhysicsServer3D::characterRigidCreate(PhysicsBodyId body,
        const PhysicsCharacterRigidParameters& parameters)
{
    Impl& s = *mImpl;
    if (!s.body(body)) return PhysicsCharacterRigidId();
    PhysicsCharacterRigidTag record;
    record.controller = new kp::CharacterRigidBody();
    record.body = body;
    record.parameters = parameters;
    applyCharacterRigidParameters(*record.controller, parameters);
    return s.characterRigids.insert(static_cast<PhysicsCharacterRigidTag&&>(record));
}

void PhysicsServer3D::characterRigidFree(PhysicsCharacterRigidId character)
{
    Impl& s = *mImpl;
    PhysicsCharacterRigidTag* record = s.characterRigids.get(character);
    if (!record || record->dying) return;
    if (s.locked)
    {
        record->dying = true;
        s.characterRigidFreeQueue.push_back(character);
        return;
    }
    s.destroyCharacterRigidNow(character);
}

bool PhysicsServer3D::characterRigidValid(PhysicsCharacterRigidId character) const
{
    const PhysicsCharacterRigidTag* record = mImpl->characterRigids.get(character);
    return record && !record->dying;
}

void PhysicsServer3D::characterRigidSetParameters(PhysicsCharacterRigidId character,
        const PhysicsCharacterRigidParameters& parameters)
{
    PhysicsCharacterRigidTag* record = mImpl->characterRigids.get(character);
    if (!record || record->dying) return;
    record->parameters = parameters;
    applyCharacterRigidParameters(*record->controller, parameters);
}

PhysicsGroundInfo PhysicsServer3D::characterRigidProbe(PhysicsCharacterRigidId character)
{
    Impl& s = *mImpl;
    PhysicsGroundInfo info;
    PhysicsCharacterRigidTag* record = s.characterRigids.get(character);
    if (!record || record->dying) return info;
    const PhysicsBodyTag* body = s.body(record->body);
    if (!body || body->libId == kNoLibId || !record->controller->bindToBody(s.world, body->libId))
        return info;
    record->controller->postSimulation(record->parameters.maxSeparation);
    switch (record->controller->groundState())
    {
        case kp::CharacterRigidBody::GroundState::OnGround:
            info.state = PhysicsGroundState::OnGround;
            break;
        case kp::CharacterRigidBody::GroundState::OnSteepGround:
            info.state = PhysicsGroundState::OnSteepGround;
            break;
        case kp::CharacterRigidBody::GroundState::NotSupported:
            info.state = PhysicsGroundState::NotSupported;
            break;
        case kp::CharacterRigidBody::GroundState::InAir:
            info.state = PhysicsGroundState::InAir;
            break;
    }
    info.normal = record->controller->groundNormal();
    info.position = record->controller->groundPosition();
    info.velocity = record->controller->groundVelocity();
    const std::uint32_t ground = record->controller->groundBodyId();
    if (ground != kp::CharacterRigidBody::kInvalidBodyId) info.body = s.handleOfLib(ground);
    return info;
}

bool PhysicsServer3D::convexHullMesh(const Math::Vec3* points, std::size_t pointCount,
        ct::Vector<Math::Vec3>& vertices, ct::Vector<std::uint32_t>& indices) const
{
    vertices.clear();
    indices.clear();
    if (!points || pointCount < 4) return false;
    kinetix::Geometry::ConvexHullComputer hull;
    const float shrunk = hull.compute(&points[0].x, static_cast<int>(sizeof(Math::Vec3)),
            static_cast<int>(pointCount), 0.0f, 0.0f);
    if (shrunk < 0.0f || hull.vertices.size() < 4 || hull.faces.empty()) return false;
    Math::Vec3 center(0.0f, 0.0f, 0.0f);
    for (std::size_t i = 0; i < hull.vertices.size(); ++i) center = center + hull.vertices[i];
    center = center * (1.0f / static_cast<float>(hull.vertices.size()));
    vertices.assign(hull.vertices.data(), hull.vertices.data() + hull.vertices.size());
    triangulateFaces(hull.vertices, hull.edges, hull.faces, center, indices);
    return !indices.empty();
}

bool PhysicsServer3D::shatter(const Math::Vec3* points, std::size_t pointCount,
        const Math::Vec3* sites, std::size_t siteCount, ct::Vector<PhysicsShard>& shards) const
{
    shards.clear();
    if (!points || !sites || pointCount < 4 || siteCount == 0) return false;
    kinetix::Geometry::ConvexHullComputer hull;
    const float shrunk = hull.compute(&points[0].x, static_cast<int>(sizeof(Math::Vec3)),
            static_cast<int>(pointCount), 0.0f, 0.0f);
    if (shrunk < 0.0f || hull.vertices.size() < 4) return false;
    ct::Vector<Math::Vec3> source;
    source.assign(hull.vertices.data(), hull.vertices.data() + hull.vertices.size());
    ct::Vector<Math::Vec3> cells;
    cells.assign(sites, sites + siteCount);
    ct::Vector<kinetix::Geometry::Shard> raw;
    kinetix::Geometry::VoronoiShatter::shatter(source, cells, raw);
    for (std::size_t i = 0; i < raw.size(); ++i)
    {
        const kinetix::Geometry::Shard& shard = raw[i];
        if (shard.vertices.size() < 4 || shard.faces.empty()) continue;
        PhysicsShard out;
        out.centroid = shard.centroid;
        out.volume = shard.volume;
        out.vertices.assign(shard.vertices.data(), shard.vertices.data() + shard.vertices.size());
        triangulateFaces(shard.vertices, shard.edges, shard.faces, Math::Vec3(0.0f, 0.0f, 0.0f),
                out.indices);
        if (out.indices.empty()) continue;
        shards.push_back(static_cast<PhysicsShard&&>(out));
    }
    return !shards.empty();
}

PhysicsSoftBodyId PhysicsServer3D::softBodyCreate(const Math::Vec3* positions, std::size_t count,
        const std::uint32_t* indices, std::size_t indexCount,
        const PhysicsSoftBodyParameters& parameters)
{
    if (!positions || count < 3 || !indices || indexCount < 3) return PhysicsSoftBodyId();
    for (std::size_t i = 0; i < indexCount; ++i)
        if (indices[i] >= count) return PhysicsSoftBodyId();
    PhysicsSoftBodyTag record;
    record.body = new kp::SoftBody();
    record.substeps =
            parameters.substeps < 1 ? 1 : (parameters.substeps > 16 ? 16 : parameters.substeps);
    record.body->setParticles(positions, static_cast<std::uint32_t>(count),
            parameters.mass > 1e-4f ? parameters.mass : 1e-4f);
    kp::SoftBody::BendType bend = kp::SoftBody::BendType::Distance;
    if (parameters.bend == PhysicsSoftBend::None) bend = kp::SoftBody::BendType::None;
    else if (parameters.bend == PhysicsSoftBend::Dihedral)
        bend = kp::SoftBody::BendType::Dihedral;
    record.body->buildFromMesh(indices, static_cast<std::uint32_t>(indexCount),
            parameters.structuralCompliance,
            bend == kp::SoftBody::BendType::None ? -1.0f : parameters.bendCompliance, bend);
    record.body->setPressure(parameters.pressure);
    record.body->setDamping(parameters.damping);
    record.body->setCollisionMargin(parameters.collisionMargin);
    kp::CollisionFilter filter;
    filter.group = parameters.collisionGroup;
    filter.mask = parameters.collisionMask;
    record.body->setCollisionFilter(filter);
    record.body->setGravity(mImpl->gravity);
    return mImpl->softBodies.insert(static_cast<PhysicsSoftBodyTag&&>(record));
}

void PhysicsServer3D::softBodyFree(PhysicsSoftBodyId softBody)
{
    Impl& s = *mImpl;
    PhysicsSoftBodyTag* record = s.softBodies.get(softBody);
    if (!record || record->dying) return;
    if (s.locked)
    {
        record->dying = true;
        s.softBodyFreeQueue.push_back(softBody);
        return;
    }
    s.destroySoftBodyNow(softBody);
}

bool PhysicsServer3D::softBodyValid(PhysicsSoftBodyId softBody) const
{
    const PhysicsSoftBodyTag* record = mImpl->softBodies.get(softBody);
    return record && !record->dying;
}

std::size_t PhysicsServer3D::softBodyGetParticleCount(PhysicsSoftBodyId softBody) const
{
    const PhysicsSoftBodyTag* record = mImpl->softBodies.get(softBody);
    return record && !record->dying ? record->body->particleCount() : 0;
}

void PhysicsServer3D::softBodyGetPositions(PhysicsSoftBodyId softBody, Math::Vec3* out,
        std::size_t count) const
{
    const PhysicsSoftBodyTag* record = mImpl->softBodies.get(softBody);
    if (!record || record->dying || !out) return;
    const std::size_t available = record->body->particleCount();
    for (std::size_t i = 0; i < count && i < available; ++i)
        out[i] = record->body->particle(static_cast<std::uint32_t>(i)).position;
}

void PhysicsServer3D::softBodySetParticlePosition(PhysicsSoftBodyId softBody, std::size_t index,
        const Math::Vec3& position)
{
    PhysicsSoftBodyTag* record = mImpl->softBodies.get(softBody);
    if (!record || record->dying || index >= record->body->particleCount()) return;
    kp::SoftBody::Particle& particle = record->body->particle(static_cast<std::uint32_t>(index));
    particle.position = position;
    particle.previousPosition = position;
    particle.velocity = Math::Vec3(0.0f, 0.0f, 0.0f);
}

void PhysicsServer3D::softBodySetPinned(PhysicsSoftBodyId softBody, std::size_t index, bool pinned)
{
    PhysicsSoftBodyTag* record = mImpl->softBodies.get(softBody);
    if (!record || record->dying || index >= record->body->particleCount()) return;
    record->body->setPinned(static_cast<std::uint32_t>(index), pinned);
}

bool PhysicsServer3D::softBodyIsPinned(PhysicsSoftBodyId softBody, std::size_t index) const
{
    const PhysicsSoftBodyTag* record = mImpl->softBodies.get(softBody);
    return record && !record->dying && index < record->body->particleCount() &&
           record->body->pinned(static_cast<std::uint32_t>(index));
}

void PhysicsServer3D::softBodyBuildAttachments(PhysicsSoftBodyId softBody,
        float maxDistanceMultiplier)
{
    PhysicsSoftBodyTag* record = mImpl->softBodies.get(softBody);
    if (record && !record->dying) record->body->buildAttachments(maxDistanceMultiplier);
}

void PhysicsServer3D::softBodyAddVelocity(PhysicsSoftBodyId softBody, const Math::Vec3& delta)
{
    PhysicsSoftBodyTag* record = mImpl->softBodies.get(softBody);
    if (!record || record->dying) return;
    for (std::uint32_t i = 0; i < record->body->particleCount(); ++i)
        if (record->body->particle(i).invMass > 0.0f) record->body->particle(i).velocity += delta;
}

void PhysicsServer3D::softBodySetWind(PhysicsSoftBodyId softBody, const Math::Vec3& wind)
{
    PhysicsSoftBodyTag* record = mImpl->softBodies.get(softBody);
    if (record && !record->dying) record->body->setWind(wind);
}

void PhysicsServer3D::softBodySetEnabled(PhysicsSoftBodyId softBody, bool enabled)
{
    PhysicsSoftBodyTag* record = mImpl->softBodies.get(softBody);
    if (record && !record->dying) record->enabled = enabled;
}

float PhysicsServer3D::softBodyGetWorstStretch(PhysicsSoftBodyId softBody) const
{
    const PhysicsSoftBodyTag* record = mImpl->softBodies.get(softBody);
    return record && !record->dying ? record->body->worstStretch() : 0.0f;
}

PhysicsRagdollId PhysicsServer3D::ragdollCreate(const int* boneParents, std::size_t boneCount,
        const PhysicsRagdollBones& bones, const Math::Mat4* restPose, float restScale,
        const Math::Mat4* globalPose, const Math::Mat4& ownerWorld,
        const PhysicsRagdollSettings& settings)
{
    Impl& s = *mImpl;
    if (!boneParents || boneCount == 0 || !restPose || !globalPose) return PhysicsRagdollId();
    kp::RagdollBoneMap map;
    map.hips = bones.hips;
    map.spine = bones.spine;
    map.neck = bones.neck;
    map.head = bones.head;
    map.leftArm = bones.leftArm;
    map.leftForeArm = bones.leftForeArm;
    map.leftHand = bones.leftHand;
    map.rightArm = bones.rightArm;
    map.rightForeArm = bones.rightForeArm;
    map.rightHand = bones.rightHand;
    map.leftUpLeg = bones.leftUpLeg;
    map.leftLeg = bones.leftLeg;
    map.leftFoot = bones.leftFoot;
    map.leftToes = bones.leftToes;
    map.rightUpLeg = bones.rightUpLeg;
    map.rightLeg = bones.rightLeg;
    map.rightFoot = bones.rightFoot;
    map.rightToes = bones.rightToes;
    kp::RagdollSettings kinetix;
    kinetix.scale = settings.scale;
    kinetix.fatness = settings.fatness;
    kinetix.headSize = settings.headSize;
    kinetix.massScale = settings.massScale;
    ct::Vector<Math::Mat4> rest;
    rest.assign(restPose, restPose + boneCount);
    ct::Vector<Math::Mat4> globals;
    globals.assign(globalPose, globalPose + boneCount);
    PhysicsRagdollTag record;
    record.ragdoll = new kp::Ragdoll();
    const ParentTableView view(boneParents, boneCount);
    if (!record.ragdoll->build(view, map, rest, restScale, kinetix))
    {
        delete record.ragdoll;
        return PhysicsRagdollId();
    }
    record.ragdoll->addToWorld(s.world, ownerWorld, globals);
    return s.ragdolls.insert(static_cast<PhysicsRagdollTag&&>(record));
}

void PhysicsServer3D::ragdollFree(PhysicsRagdollId ragdoll)
{
    Impl& s = *mImpl;
    PhysicsRagdollTag* record = s.ragdolls.get(ragdoll);
    if (!record || record->dying) return;
    if (s.locked)
    {
        record->dying = true;
        s.ragdollFreeQueue.push_back(ragdoll);
        return;
    }
    s.destroyRagdollNow(ragdoll);
}

bool PhysicsServer3D::ragdollValid(PhysicsRagdollId ragdoll) const
{
    const PhysicsRagdollTag* record = mImpl->ragdolls.get(ragdoll);
    return record && !record->dying;
}

void PhysicsServer3D::ragdollFollow(PhysicsRagdollId ragdoll, const Math::Mat4* globalPose,
        std::size_t boneCount, const Math::Mat4& ownerWorld)
{
    PhysicsRagdollTag* record = mImpl->ragdolls.get(ragdoll);
    if (!record || record->dying || !globalPose || boneCount == 0 || record->ragdoll->active())
        return;
    ct::Vector<Math::Mat4> globals;
    globals.assign(globalPose, globalPose + boneCount);
    record->ragdoll->setTargetPose(ownerWorld, globals);
}

bool PhysicsServer3D::ragdollActivate(PhysicsRagdollId ragdoll)
{
    Impl& s = *mImpl;
    PhysicsRagdollTag* record = s.ragdolls.get(ragdoll);
    if (!record || record->dying) return false;
    record->ragdoll->activate();
    s.setRagdollCounted(*record, record->ragdoll->active());
    return record->ragdoll->active();
}

void PhysicsServer3D::ragdollDeactivate(PhysicsRagdollId ragdoll)
{
    Impl& s = *mImpl;
    PhysicsRagdollTag* record = s.ragdolls.get(ragdoll);
    if (!record || record->dying) return;
    record->ragdoll->deactivate();
    s.setRagdollCounted(*record, false);
}

bool PhysicsServer3D::ragdollIsActive(PhysicsRagdollId ragdoll) const
{
    const PhysicsRagdollTag* record = mImpl->ragdolls.get(ragdoll);
    return record && !record->dying && record->ragdoll->active();
}

void PhysicsServer3D::ragdollWritePose(PhysicsRagdollId ragdoll, PhysicsBonePose* localPose,
        std::size_t boneCount, const Math::Mat4& ownerWorld, float alpha) const
{
    const PhysicsRagdollTag* record = mImpl->ragdolls.get(ragdoll);
    if (!record || record->dying || !localPose || boneCount == 0 || !record->ragdoll->active())
        return;
    ct::Vector<kp::LocalPose> poses;
    poses.reserve(boneCount);
    for (std::size_t i = 0; i < boneCount; ++i)
    {
        kp::LocalPose pose;
        pose.position = localPose[i].position;
        pose.rotation = toKinetix(localPose[i].rotation);
        pose.scale = localPose[i].scale;
        poses.push_back(pose);
    }
    record->ragdoll->writePose(poses, ownerWorld, alpha);
    for (std::size_t i = 0; i < boneCount; ++i)
    {
        localPose[i].position = poses[i].position;
        localPose[i].rotation = fromKinetix(poses[i].rotation);
        localPose[i].scale = poses[i].scale;
    }
}

void PhysicsServer3D::ragdollSetCollisionMask(PhysicsRagdollId ragdoll, std::uint32_t mask)
{
    PhysicsRagdollTag* record = mImpl->ragdolls.get(ragdoll);
    if (record && !record->dying) record->ragdoll->setCollisionMask(mask);
}

int PhysicsServer3D::ragdollGetPartBone(PhysicsRagdollId ragdoll, PhysicsRagdollPart part) const
{
    const PhysicsRagdollTag* record = mImpl->ragdolls.get(ragdoll);
    if (!record || record->dying || part >= PhysicsRagdollPart::Count) return -1;
    return record->ragdoll->boneIndex(static_cast<kp::RagdollPart>(part));
}

PhysicsRagdollPart PhysicsServer3D::ragdollGetPartParent(PhysicsRagdollId ragdoll,
        PhysicsRagdollPart part) const
{
    const PhysicsRagdollTag* record = mImpl->ragdolls.get(ragdoll);
    if (!record || record->dying || part >= PhysicsRagdollPart::Count)
        return PhysicsRagdollPart::Count;
    return static_cast<PhysicsRagdollPart>(
            record->ragdoll->parentOf(static_cast<kp::RagdollPart>(part)));
}

float PhysicsServer3D::ragdollGetPartMass(PhysicsRagdollId ragdoll, PhysicsRagdollPart part) const
{
    const PhysicsRagdollTag* record = mImpl->ragdolls.get(ragdoll);
    if (!record || record->dying || part >= PhysicsRagdollPart::Count) return 0.0f;
    return record->ragdoll->mass(static_cast<kp::RagdollPart>(part));
}

float PhysicsServer3D::ragdollGetFacing(PhysicsRagdollId ragdoll) const
{
    const PhysicsRagdollTag* record = mImpl->ragdolls.get(ragdoll);
    return record && !record->dying ? record->ragdoll->facing() : 0.0f;
}

Math::Mat4 PhysicsServer3D::ragdollGetPartTransform(PhysicsRagdollId ragdoll,
        PhysicsRagdollPart part) const
{
    const PhysicsRagdollTag* record = mImpl->ragdolls.get(ragdoll);
    if (!record || record->dying || part >= PhysicsRagdollPart::Count)
        return Math::Mat4::Identity();
    return record->ragdoll->body(static_cast<kp::RagdollPart>(part))->transform();
}

Math::Vec3 PhysicsServer3D::ragdollGetPartVelocity(PhysicsRagdollId ragdoll,
        PhysicsRagdollPart part) const
{
    const PhysicsRagdollTag* record = mImpl->ragdolls.get(ragdoll);
    if (!record || record->dying || part >= PhysicsRagdollPart::Count)
        return Math::Vec3(0.0f, 0.0f, 0.0f);
    return record->ragdoll->body(static_cast<kp::RagdollPart>(part))->velocity();
}

bool PhysicsServer3D::ragdollDetach(PhysicsRagdollId ragdoll, PhysicsRagdollPart part)
{
    PhysicsRagdollTag* record = mImpl->ragdolls.get(ragdoll);
    if (!record || record->dying || part >= PhysicsRagdollPart::Count || !record->ragdoll->active())
        return false;
    return record->ragdoll->detach(static_cast<kp::RagdollPart>(part));
}

bool PhysicsServer3D::ragdollIsDetached(PhysicsRagdollId ragdoll, PhysicsRagdollPart part) const
{
    const PhysicsRagdollTag* record = mImpl->ragdolls.get(ragdoll);
    if (!record || record->dying || part >= PhysicsRagdollPart::Count) return false;
    return record->ragdoll->detached(static_cast<kp::RagdollPart>(part));
}

void PhysicsServer3D::ragdollApplyImpulse(PhysicsRagdollId ragdoll, PhysicsRagdollPart part,
        const Math::Vec3& impulse, const Math::Vec3& offset)
{
    PhysicsRagdollTag* record = mImpl->ragdolls.get(ragdoll);
    if (!record || record->dying || part >= PhysicsRagdollPart::Count || !record->ragdoll->active())
        return;
    kp::RigidBody* body = record->ragdoll->body(static_cast<kp::RagdollPart>(part));
    body->setAwake(true);
    body->applyLinearImpulse(impulse);
    body->applyAngularImpulse(Math::Vec3::Cross(offset, impulse));
}

bool PhysicsServer3D::intersectRay(const PhysicsRayParameters& parameters,
        PhysicsRayResult& result) const
{
    const Impl& s = *mImpl;
    const Math::Vec3 delta = parameters.to - parameters.from;
    const float length = delta.Length();
    if (!(length > 0.0f)) return false;
    kinetix::Ray ray;
    ray.origin = parameters.from;
    ray.direction = delta / length;
    kp::QueryFilter filter;
    filter.collision.group = 0xFFFFFFFFu;
    filter.collision.mask = parameters.collisionMask;
    const PhysicsBodyTag* excluded = s.body(parameters.exclude);
    if (excluded && excluded->libId != kNoLibId) filter.ignoredBody = excluded->libId;
    kp::WorldRayHit hit;
    if (!s.world.raycast(ray, length, filter, hit)) return false;
    result.body = s.handleOfLib(hit.body);
    result.position = hit.point;
    result.normal = hit.normal;
    result.distance = hit.distance;
    result.triangle = hit.triangle;
    return true;
}

void PhysicsServer3D::overlapSphere(const Math::Vec3& center, float radius,
        std::uint32_t collisionMask, ct::Vector<PhysicsBodyId>& out) const
{
    const Impl& s = *mImpl;
    kp::QueryFilter filter;
    filter.collision.group = 0xFFFFFFFFu;
    filter.collision.mask = collisionMask;
    ct::Vector<std::uint32_t> ids;
    s.world.overlapSphere(center, radius, filter, ids);
    out.clear();
    for (std::size_t i = 0; i < ids.size(); ++i) out.push_back(s.handleOfLib(ids[i]));
}

void PhysicsServer3D::overlapBox(const Math::Box& box, std::uint32_t collisionMask,
        ct::Vector<PhysicsBodyId>& out) const
{
    const Impl& s = *mImpl;
    kp::QueryFilter filter;
    filter.collision.group = 0xFFFFFFFFu;
    filter.collision.mask = collisionMask;
    kinetix::AABB bounds;
    bounds.min = box.min;
    bounds.max = box.max;
    ct::Vector<std::uint32_t> ids;
    s.world.queryAABB(bounds, filter, ids);
    out.clear();
    for (std::size_t i = 0; i < ids.size(); ++i) out.push_back(s.handleOfLib(ids[i]));
}

void PhysicsServer3D::debugDraw(BatchRenderer& batch, float interpolation)
{
    Impl& s = *mImpl;
    kinetix::DebugDraw3D& debug = kinetix::DebugDraw3D::getSingleton();
    debug.clear();

    struct Shown
    {
        PhysicsBodyTag* record;
        Math::Vec3 position;
        Math::Quaternion rotation;
    };
    ct::Vector<Shown> shown;
    const float t = interpolation < 0.0f ? 0.0f : (interpolation > 1.0f ? 1.0f : interpolation);
    for (std::size_t i = 0; i < s.bodies.size(); ++i)
    {
        PhysicsBodyTag& record = s.bodies.items()[i];
        if (record.dying || record.libId == kNoLibId || !record.body) continue;
        Math::Vec3 position;
        Math::Quaternion rotation;
        if (record.mode == PhysicsBodyMode::Rigid && record.interpolated)
        {
            position = Math::Vec3::Lerp(record.previousPosition, s.bodyPosition(record), t);
            rotation = Math::Quaternion::Slerp(record.previousRotation, s.bodyRotation(record), t);
        }
        else if (record.mode == PhysicsBodyMode::Kinematic && record.hasTarget)
        {
            position = record.targetPosition;
            rotation = record.targetRotation;
        }
        else
            continue;
        Shown entry;
        entry.record = &record;
        entry.position = s.bodyPosition(record);
        entry.rotation = s.bodyRotation(record);
        shown.push_back(entry);
        record.body->setPosition(position);
        record.body->setOrientation(toKinetix(rotation));
    }

    s.world.debugDraw();

    const kinetix::Color rayColor(60, 220, 60, 255);
    const kinetix::Color hitColor(255, 40, 40, 255);
    for (std::size_t i = 0; i < s.vehicles.size(); ++i)
    {
        const PhysicsVehicleTag& record = s.vehicles.items()[i];
        if (record.dying || !record.vehicle) continue;
        for (std::uint32_t w = 0; w < record.vehicle->wheelCount(); ++w)
        {
            const kp::RaycastVehicle::Wheel& wheel = record.vehicle->wheel(w);
            const Math::Vec3 from = wheel.hardPointWorld;
            const Math::Vec3 to =
                    from + wheel.directionWorld * (wheel.suspensionLength + wheel.radius);
            debug.line(from, to, rayColor, false);
            if (!wheel.inContact) continue;
            debug.cross(wheel.contactPoint, 0.15f, hitColor);
            debug.line(wheel.contactPoint, wheel.contactPoint + wheel.contactNormal * 0.3f,
                    hitColor, false);
        }
    }
    for (std::size_t i = 0; i < s.groundVehicles.size(); ++i)
    {
        const PhysicsGroundVehicleTag& record = s.groundVehicles.items()[i];
        if (record.dying || !record.vehicle) continue;
        for (std::uint32_t w = 0; w < record.vehicle->wheelCount(); ++w)
        {
            const kp::WheelTelemetry& wheel = record.vehicle->wheel(w);
            if (!wheel.inContact) continue;
            debug.cross(wheel.contactPoint, 0.15f, hitColor);
            debug.line(wheel.contactPoint, wheel.contactPoint + wheel.contactNormal * 0.3f,
                    hitColor, false);
        }
    }

    const auto joints = s.joints.items();
    for (std::size_t i = 0; i < joints.size(); ++i)
    {
        const PhysicsJointTag& joint = joints[i];
        if (joint.dying || !joint.joint) continue;
        const PhysicsBodyTag* a = s.body(joint.bodyA);
        const PhysicsBodyTag* b = s.body(joint.bodyB);
        if (!a && !b) continue;
        const Math::Vec3 from = a ? a->body->position() : b->body->position();
        const Math::Vec3 to = b ? b->body->position() : from;
        const kinetix::Color color = joint.type == PhysicsJointType::Mouse
                                             ? kinetix::Color(255, 90, 90, 255)
                                             : kinetix::Color(70, 200, 255, 255);
        debug.line(from, to, color, false);
        debug.cross(from, 0.08f, color);
        debug.cross(to, 0.08f, color);
    }

    for (std::size_t i = 0; i < shown.size(); ++i)
    {
        shown[i].record->body->setPosition(shown[i].position);
        shown[i].record->body->setOrientation(toKinetix(shown[i].rotation));
    }

    const auto vehicles = s.vehicles.items();
    for (std::size_t i = 0; i < vehicles.size(); ++i)
    {
        const PhysicsVehicleTag& record = vehicles[i];
        if (record.dying || !record.vehicle) continue;
        for (std::uint32_t w = 0; w < record.vehicle->wheelCount(); ++w)
        {
            const kp::RaycastVehicle::Wheel& wheel = record.vehicle->wheel(w);
            const Math::Vec3 hub =
                    wheel.hardPointWorld + wheel.directionWorld * wheel.suspensionLength;
            debug.line(wheel.hardPointWorld, hub, kinetix::Color(255, 220, 60, 255), false);
            Math::Vec3 up = wheel.directionWorld * -1.0f;
            Math::Vec3 forward = Math::Vec3::Cross(up, wheel.axleWorld);
            if (forward.LengthSquared() < 1e-8f) continue;
            forward = forward.Normalized();
            debug.circle(hub, up, forward, wheel.radius, 20,
                    wheel.inContact ? kinetix::Color(120, 255, 120, 255)
                                    : kinetix::Color(255, 140, 60, 255));
            if (wheel.inContact)
                debug.line(wheel.contactPoint, wheel.contactPoint + wheel.contactNormal * 0.4f,
                        kinetix::Color(255, 60, 60, 255), false);
        }
    }

    batch.setDefault3DState();
    const ct::Vector<kinetix::DebugLine3D>& lines = debug.lines();
    for (std::size_t i = 0; i < lines.size(); ++i)
    {
        const kinetix::DebugLine3D& line = lines[i];
        batch.setDepthTest(line.depthTest);
        batch.setColor(line.color.r(), line.color.g(), line.color.b(), line.color.a());
        batch.drawLine3D(line.from.x, line.from.y, line.from.z, line.to.x, line.to.y, line.to.z);
    }
    debug.clear();
}

const PhysicsStats& PhysicsServer3D::getStats() const
{
    return mImpl->stats;
}

std::uint32_t PhysicsServer3D::getContactCount() const
{
    return static_cast<std::uint32_t>(mImpl->world.contactCount());
}

void PhysicsServer3D::resetStats()
{
    Impl& s = *mImpl;
    s.stats.steps = 0;
    s.stats.stepMs = 0.0;
    s.stats.averageStepMs = 0.0;
    s.stats.maxStepMs = 0.0;
    s.stepAccumulator = 0.0;
}

} // namespace prisma3d
