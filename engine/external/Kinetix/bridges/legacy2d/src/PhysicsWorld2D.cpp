#include "k2d/PhysicsWorld2D.h"

#include "k2d/CanvasRenderer.h"
#include "k2d/GameObject.h"
#include "k2d/RenderQueue.h"
#include "k2d/RigidBody2D.h"
#include "k2d/TileMapComponent.h"
#include "k2d/Profiler.h"

#include <kx/tilemapcollider.h>

#include <cmath>
#include <limits>

namespace k2d
{

namespace
{
constexpr float kDegToRad = 0.01745329251994329577f;
constexpr float kRadToDeg = 57.29577951308232088f;
constexpr float kMinShapeExtent = 0.01f;

PhysicsWorld2D* gActiveWorld = nullptr;

class PhysicsRenderDebugDraw final : public kx::DebugDraw
{
  public:
    struct Primitive
    {
        ct::Vector<Math::Vec2> points;
        Color color;
    };

    void DrawCircleShape(const kx::Transform& xf, float radius, kx::Color color) override
    {
        if (radius <= 0.0f)
            return;
        constexpr int segments = 32;
        Math::Vec2 points[segments];
        for (int i = 0; i < segments; ++i)
        {
            const float angle = (float)i * 6.28318530717958647692f / (float)segments;
            points[i] = xf.Transform(Math::Vec2(std::cos(angle) * radius, std::sin(angle) * radius));
        }
        addLoop(points, segments, toColor(color), 1.5f);
        addSegment(xf.Transform(Math::Vec2(0.0f, 0.0f)), xf.Transform(Math::Vec2(radius, 0.0f)), toColor(color), 1.0f);
    }

    void DrawPolygonShape(const kx::Transform& xf, const Math::Vec2* verts, int count, kx::Color color) override
    {
        if (!verts || count < 2)
            return;
        ct::Vector<Math::Vec2> points;
        points.resize((size_t)count);
        for (int i = 0; i < count; ++i)
            points[(size_t)i] = xf.Transform(verts[i]);
        addLoop(points.data(), count, toColor(color), 1.5f);
    }

    void DrawSegment(const Math::Vec2& a, const Math::Vec2& b, kx::Color color) override
    {
        addSegment(a, b, toColor(color), 1.5f);
    }

    void DrawPoint(const Math::Vec2& p, float size, kx::Color color) override
    {
        const float half = size > 0.0f ? size * 0.5f : 0.5f;
        Primitive& primitive = makePrimitive(toColor(color));
        primitive.points.push_back(Math::Vec2(p.x - half, p.y - half));
        primitive.points.push_back(Math::Vec2(p.x + half, p.y - half));
        primitive.points.push_back(Math::Vec2(p.x + half, p.y + half));
        primitive.points.push_back(Math::Vec2(p.x - half, p.y - half));
        primitive.points.push_back(Math::Vec2(p.x + half, p.y + half));
        primitive.points.push_back(Math::Vec2(p.x - half, p.y + half));
    }

    void DrawAABB(const Math::Vec2& lower, const Math::Vec2& upper, kx::Color color) override
    {
        const Math::Vec2 points[4] = {Math::Vec2(lower.x, lower.y), Math::Vec2(upper.x, lower.y),
                                      Math::Vec2(upper.x, upper.y), Math::Vec2(lower.x, upper.y)};
        addLoop(points, 4, toColor(color), 1.0f);
    }

    void flush(RenderQueue& queue)
    {
        if (mPrimitives.empty())
            return;

        RenderItem& item = queue.AddItem(std::numeric_limits<int>::max());
        item.blendMode = BLEND_MIX;
        for (size_t i = 0; i < mPrimitives.size(); ++i)
        {
            Primitive& primitive = mPrimitives[i];
            if (primitive.points.empty())
                continue;
            RenderCommand command;
            command.type = RenderCommand::kPolygon;
            command.color = primitive.color;
            command.polygonPoints = &primitive.points;
            command.polygonPointCount = (unsigned int)primitive.points.size();
            item.commands.push_back(command);
        }
    }

  private:
    static Color toColor(kx::Color color)
    {
        return Color::FromBytes(color.r, color.g, color.b, color.a);
    }

    Primitive& makePrimitive(const Color& color)
    {
        mPrimitives.resize(mPrimitives.size() + 1);
        Primitive& primitive = mPrimitives.back();
        primitive.color = color;
        return primitive;
    }

    void addLoop(const Math::Vec2* points, int count, const Color& color, float width)
    {
        for (int i = 0; i < count; ++i)
            addSegment(points[i], points[(i + 1) % count], color, width);
    }

    void addSegment(const Math::Vec2& a, const Math::Vec2& b, const Color& color, float width)
    {
        Math::Vec2 direction = b - a;
        const float length = direction.Length();
        if (length < 0.0001f)
            return;
        direction /= length;
        const Math::Vec2 normal(-direction.y * width * 0.5f, direction.x * width * 0.5f);
        Primitive& primitive = makePrimitive(color);
        primitive.points.push_back(a - normal);
        primitive.points.push_back(a + normal);
        primitive.points.push_back(b + normal);
        primitive.points.push_back(a - normal);
        primitive.points.push_back(b + normal);
        primitive.points.push_back(b - normal);
    }

    ct::Vector<Primitive> mPrimitives;
};
} // namespace

PhysicsWorld2D::PhysicsWorld2D(const Math::Vec2& gravity)
    : mWorld(gravity), mBodies(), mPending(), mDirtyBodies(), mFixedStep(1.0f / 60.0f), mAccumulator(0.0f),
      mLastDelta(0.0f), mCallback(nullptr), mCallbackUser(nullptr)
{
}

PhysicsWorld2D::~PhysicsWorld2D()
{
    clear();
    if (gActiveWorld == this)
        gActiveWorld = nullptr;
}

void PhysicsWorld2D::SetActive(PhysicsWorld2D* world)
{
    gActiveWorld = world;
}

PhysicsWorld2D* PhysicsWorld2D::Active()
{
    return gActiveWorld;
}

GameObject* PhysicsWorld2D::objectFor(const kx::Body* body)
{
    return body ? static_cast<GameObject*>(body->UserData()) : nullptr;
}

void PhysicsWorld2D::setCollisionCallback(CollisionCallback callback, void* user)
{
    mCallback = callback;
    mCallbackUser = user;
}

void PhysicsWorld2D::debugDraw(CanvasRenderer& canvas, unsigned flags)
{
    if (flags == 0)
        return;

    PhysicsRenderDebugDraw draw;
    kx::Draw(mWorld, draw, flags);

    RenderQueue queue;
    draw.flush(queue);
    queue.Flush(canvas);
}

void PhysicsWorld2D::releaseBody(kx::World& world, kx::Body* body)
{
    if (!body)
        return;
    // Under a collision callback Destroy is deferred and the body may still appear in events
    // or queries; cut it loose from its GameObject now.
    body->SetContactCallback(nullptr, nullptr);
    body->SetUserData(nullptr);
    world.Destroy(body);
}

void PhysicsWorld2D::removePending(RigidBody2D& rigidBody)
{
    const size_t index = rigidBody.mPendingIndex;
    if (index < mPending.size() && mPending[index] == &rigidBody)
    {
        RigidBody2D* moved = mPending.back();
        mPending[index] = moved;
        if (moved)
            moved->mPendingIndex = index;
        mPending.pop_back();
    }
    rigidBody.mPendingIndex = RigidBody2D::InvalidWorldIndex;
}

void PhysicsWorld2D::clear()
{
    for (size_t i = 0; i < mTileMaps.size(); ++i)
    {
        const ct::Vector<kx::Body*>& bodies = mTileMaps[i]->Bodies();
        for (size_t b = 0; b < bodies.size(); ++b)
        {
            bodies[b]->SetContactCallback(nullptr, nullptr);
            bodies[b]->SetUserData(nullptr);
        }
        delete mTileMaps[i];
    }
    mTileMaps.clear();

    for (size_t i = 0; i < mBodies.size(); ++i)
    {
        RigidBody2D* rigidBody = mBodies[i];
        if (!rigidBody)
            continue;
        releaseBody(mWorld, rigidBody->mBody);
        rigidBody->mBody = nullptr;
        rigidBody->mWorld = nullptr;
        rigidBody->mBodyIndex = RigidBody2D::InvalidWorldIndex;
        rigidBody->mDirtyIndex = RigidBody2D::InvalidWorldIndex;
    }
    mBodies.clear();

    for (size_t i = 0; i < mPending.size(); ++i)
        if (mPending[i])
        {
            mPending[i]->mWorld = nullptr;
            mPending[i]->mPendingIndex = RigidBody2D::InvalidWorldIndex;
        }
    mPending.clear();
    for (size_t i = 0; i < mDirtyBodies.size(); ++i)
        if (mDirtyBodies[i])
            mDirtyBodies[i]->mDirtyIndex = RigidBody2D::InvalidWorldIndex;
    mDirtyBodies.clear();
    mAccumulator = 0.0f;
}

void PhysicsWorld2D::build(GameObject& root)
{
    clear();
    collect(root);
    SetActive(this);
}

void PhysicsWorld2D::attach(GameObject& object)
{
    collect(object);
}

void PhysicsWorld2D::detach(RigidBody2D& rigidBody)
{
    const auto removeBody = [&](RigidBody2D& body)
    {
        const size_t index = body.mBodyIndex;
        if (index < mBodies.size() && mBodies[index] == &body)
        {
            RigidBody2D* moved = mBodies.back();
            mBodies[index] = moved;
            moved->mBodyIndex = index;
            mBodies.pop_back();
        }
        body.mBodyIndex = RigidBody2D::InvalidWorldIndex;
    };
    const auto removeDirty = [&](RigidBody2D& body)
    {
        const size_t index = body.mDirtyIndex;
        if (index < mDirtyBodies.size() && mDirtyBodies[index] == &body)
        {
            RigidBody2D *moved = mDirtyBodies.back();
            mDirtyBodies[index] = moved;
            moved->mDirtyIndex = index;
            mDirtyBodies.pop_back();
        }
        body.mDirtyIndex = RigidBody2D::InvalidWorldIndex;
    };

    removePending(rigidBody);
    removeBody(rigidBody);
    removeDirty(rigidBody);

    releaseBody(mWorld, rigidBody.mBody);
    rigidBody.mBody = nullptr;
    rigidBody.mWorld = nullptr;
}

bool PhysicsWorld2D::markDirty(RigidBody2D& rigidBody)
{
    if (rigidBody.mWorld != this)
        return false;
    rigidBody.mNeedsRebuild = true;
    if (rigidBody.mBody && rigidBody.mDirtyIndex == RigidBody2D::InvalidWorldIndex)
    {
        rigidBody.mDirtyIndex = mDirtyBodies.size();
        mDirtyBodies.push_back(&rigidBody);
    }
    return rigidBody.mBody != nullptr;
}

void PhysicsWorld2D::drainPending()
{
    if (mPending.empty())
        return;

    ct::Vector<RigidBody2D*> pending;
    for (size_t i = 0; i < mPending.size(); ++i)
        pending.push_back(mPending[i]);
    mPending.clear();

    for (size_t i = 0; i < pending.size(); ++i)
    {
        RigidBody2D* rigidBody = pending[i];
        if (rigidBody)
            rigidBody->mPendingIndex = RigidBody2D::InvalidWorldIndex;
        if (!rigidBody || rigidBody->mBody || !rigidBody->owner())
            continue;
        createBody(*rigidBody->owner(), *rigidBody);
    }
}

bool PhysicsWorld2D::needsRebuild(const RigidBody2D& rigidBody) const
{
    return rigidBody.mNeedsRebuild;
}

void PhysicsWorld2D::rebuildChanged()
{
    for (size_t i = 0; i < mDirtyBodies.size(); ++i)
    {
        RigidBody2D* rigidBody = mDirtyBodies[i];
        if (!rigidBody)
            continue;
        rigidBody->mDirtyIndex = RigidBody2D::InvalidWorldIndex;
        if (rigidBody->mBody && needsRebuild(*rigidBody))
            rebuildBody(*rigidBody);
    }
    mDirtyBodies.clear();
}

void PhysicsWorld2D::rebuildBody(RigidBody2D& rigidBody)
{
    GameObject* object = rigidBody.owner();
    if (!object || !rigidBody.mBody)
        return;

    const Math::Vec2 velocity = rigidBody.mBody->Velocity();
    const float angularVelocity = rigidBody.mBody->AngularVelocity();

    releaseBody(mWorld, rigidBody.mBody);
    rigidBody.mBody = nullptr;
    const size_t index = rigidBody.mBodyIndex;
    if (index < mBodies.size() && mBodies[index] == &rigidBody)
    {
        RigidBody2D* moved = mBodies.back();
        mBodies[index] = moved;
        moved->mBodyIndex = index;
        mBodies.pop_back();
    }
    rigidBody.mBodyIndex = RigidBody2D::InvalidWorldIndex;

    rigidBody.mNeedsRebuild = false;
    createBody(*object, rigidBody);

    if (rigidBody.mBody)
    {
        rigidBody.mBody->SetVelocity(velocity);
        rigidBody.mBody->SetAngularVelocity(angularVelocity);
    }
}

void PhysicsWorld2D::collect(GameObject& object)
{
    const size_t mapCount = object.componentCount<TileMapComponent>();
    for (size_t i = 0; i < mapCount; ++i)
    {
        TileMapComponent* tileMap = object.getComponentAt<TileMapComponent>(i);
        if (tileMap)
            createTileMapCollider(object, *tileMap);
    }

    const size_t count = object.componentCount<RigidBody2D>();
    for (size_t i = 0; i < count; ++i)
    {
        RigidBody2D* rigidBody = object.getComponentAt<RigidBody2D>(i);
        if (!rigidBody || rigidBody->mBody)
            continue;
        createBody(object, *rigidBody);
    }

    for (size_t i = 0; i < object.childCount(); ++i)
        collect(*object.child(i));
}

void PhysicsWorld2D::createTileMapCollider(GameObject& object, TileMapComponent& tileMap)
{
    if (tileMap.columns() <= 0 || tileMap.rows() <= 0)
        return;

    const Math::Vec2 scale = object.scale();
    const float scaleX = std::fabs(scale.x) > 0.0001f ? std::fabs(scale.x) : 1.0f;
    const float scaleY = std::fabs(scale.y) > 0.0001f ? std::fabs(scale.y) : 1.0f;
    kx::TileMapCollider* collider = new kx::TileMapCollider(mWorld);
    collider->SetMapSize(tileMap.columns(), tileMap.rows());
    collider->SetCellSize(Math::Vec2(tileMap.cellWidth() * scaleX, tileMap.cellHeight() * scaleY));
    collider->SetOffset(object.globalPosition());
    for (int y = 0; y < tileMap.rows(); ++y)
        for (int x = 0; x < tileMap.columns(); ++x)
            collider->SetSolid(x, y, tileMap.hasCollision(x, y));
    collider->Rebuild();

    const ct::Vector<kx::Body*>& bodies = collider->Bodies();
    for (size_t i = 0; i < bodies.size(); ++i)
    {
        bodies[i]->SetUserData(&object);
        bodies[i]->SetContactCallback(&PhysicsWorld2D::onContact, this);
    }
    mTileMaps.push_back(collider);
}

void PhysicsWorld2D::createBody(GameObject& object, RigidBody2D& rigidBody)
{
    // Taken over from another world's pending list: it must leave that list (detach() only
    // searches the world that owns the body).
    if (rigidBody.mPendingIndex != RigidBody2D::InvalidWorldIndex && rigidBody.mWorld)
        rigidBody.mWorld->removePending(rigidBody);

    const Math::Vec2 position = object.globalPosition();
    kx::Body* body = mWorld.CreateBody(rigidBody.bodyType(), position, object.rotationDegrees() * kDegToRad);
    if (!body)
        return;

    body->SetUserData(&object);
    body->SetFriction(rigidBody.friction());
    body->SetRestitution(rigidBody.restitution());
    body->SetLinearDamping(rigidBody.linearDamping());
    body->SetAngularDamping(rigidBody.angularDamping());
    body->SetGravityScale(rigidBody.gravityScale());
    body->SetBullet(rigidBody.bullet());
    body->SetContactCallback(&PhysicsWorld2D::onContact, this);

    rigidBody.mBody = body;
    rigidBody.mWorld = this;
    rigidBody.mBodyIndex = mBodies.size();
    mBodies.push_back(&rigidBody);

    attachShapes(object, rigidBody);
    body->SetFixedRotation(rigidBody.fixedRotation());

    if (rigidBody.mHasPendingVelocity)
    {
        body->SetVelocity(rigidBody.mPendingVelocity);
        rigidBody.mHasPendingVelocity = false;
    }
    if (rigidBody.mHasPendingAngularVelocity)
    {
        body->SetAngularVelocity(rigidBody.mPendingAngularVelocity * kDegToRad);
        rigidBody.mHasPendingAngularVelocity = false;
    }
}

void PhysicsWorld2D::attachShapes(GameObject& object, RigidBody2D& rigidBody)
{
    kx::Body* body = rigidBody.mBody;
    if (!body)
        return;

    const Math::Vec2 scale = object.scale();
    const float scaleX = std::fabs(scale.x) > 0.0001f ? std::fabs(scale.x) : 1.0f;
    const float scaleY = std::fabs(scale.y) > 0.0001f ? std::fabs(scale.y) : 1.0f;
    constexpr float kMinExtent = 0.01f;

    const ct::Vector<ShapeDesc2D>& shapes = rigidBody.shapes();
    for (size_t i = 0; i < shapes.size(); ++i)
    {
        const ShapeDesc2D& shape = shapes[i];
        const int shapesBefore = body->ShapeCount();

        switch (shape.kind)
        {
        case ShapeKind2D::Box:
        {
            float halfWidth = shape.size.x * 0.5f * scaleX;
            float halfHeight = shape.size.y * 0.5f * scaleY;
            if (halfWidth < kMinExtent)
                halfWidth = kMinExtent;
            if (halfHeight < kMinExtent)
                halfHeight = kMinExtent;
            body->AddBox(halfWidth, halfHeight, shape.offset, shape.density);
            break;
        }
        case ShapeKind2D::Circle:
        {
            const float uniformScale = scaleX > scaleY ? scaleX : scaleY;
            const float radius = shape.radius * uniformScale;
            body->AddCircle(shape.offset, radius > kMinExtent ? radius : kMinExtent, shape.density);
            break;
        }
        case ShapeKind2D::Polygon:
        {
            if (shape.points.size() < 3)
                break;
            ct::Vector<Math::Vec2> scaled;
            for (size_t p = 0; p < shape.points.size(); ++p)
                scaled.push_back(Math::Vec2(shape.points[p].x * scaleX + shape.offset.x,
                                            shape.points[p].y * scaleY + shape.offset.y));
            body->AddPolygon(scaled.data(), (int)scaled.size(), shape.density);
            break;
        }
        case ShapeKind2D::Edge:
        {
            const Math::Vec2 a(shape.start.x * scaleX + shape.offset.x, shape.start.y * scaleY + shape.offset.y);
            const Math::Vec2 b(shape.end.x * scaleX + shape.offset.x, shape.end.y * scaleY + shape.offset.y);
            body->AddEdge(a, b);
            break;
        }
        case ShapeKind2D::Chain:
        {
            if (shape.points.size() < 2)
                break;
            ct::Vector<Math::Vec2> scaled;
            for (size_t p = 0; p < shape.points.size(); ++p)
                scaled.push_back(Math::Vec2(shape.points[p].x * scaleX + shape.offset.x,
                                            shape.points[p].y * scaleY + shape.offset.y));
            body->AddChain(scaled.data(), (int)scaled.size(), shape.loop);
            break;
        }
        }

        const int shapesAfter = body->ShapeCount();
        for (int s = shapesBefore; s < shapesAfter; ++s)
        {
            body->SetSensor(s, shape.sensor);
            body->SetShapeUserData(s, &rigidBody);
            body->SetShapeFilter(s, shape.category, shape.mask);
        }
    }

    if (body->ShapeCount() == 0)
    {
        const float half = 16.0f;
        body->AddBox(half * scaleX, half * scaleY, Math::Vec2(0.0f, 0.0f), rigidBody.density());
    }
}

void PhysicsWorld2D::pushTransforms()
{
    for (size_t i = 0; i < mBodies.size(); ++i)
    {
        RigidBody2D* rigidBody = mBodies[i];
        if (!rigidBody || !rigidBody->mBody)
            continue;
        GameObject* object = rigidBody->owner();
        if (!object)
            continue;

        if (rigidBody->bodyType() == kx::BodyType::Dynamic)
            continue;

        rigidBody->mBody->SetPosition(object->globalPosition());
        rigidBody->mBody->SetAngle(object->rotationDegrees() * kDegToRad);
    }
}

void PhysicsWorld2D::pullTransforms()
{
    for (size_t i = 0; i < mBodies.size(); ++i)
    {
        RigidBody2D* rigidBody = mBodies[i];
        if (!rigidBody || !rigidBody->mBody)
            continue;
        if (rigidBody->bodyType() == kx::BodyType::Static)
            continue;

        GameObject* object = rigidBody->owner();
        if (!object)
            continue;

        const Math::Vec2 worldPosition = rigidBody->mBody->Position();
        const float worldAngle = rigidBody->mBody->Angle() * kRadToDeg;

        GameObject* parent = object->parent();
        if (parent && parent->parent())
        {
            const Math::Vec2 parentPosition = parent->globalPosition();
            const Math::Vec2 localPosition(worldPosition.x - parentPosition.x, worldPosition.y - parentPosition.y);
            object->setPositionAndRotation(localPosition, worldAngle);
        }
        else
        {
            object->setPositionAndRotation(worldPosition, worldAngle);
            continue;
        }
    }
}

void PhysicsWorld2D::step(float deltaTime)
{
    ProfileScope profileScope("physics.step");
    if (deltaTime > 0.0f)
        mLastDelta = deltaTime;
    drainPending();
    rebuildChanged();

    if (deltaTime <= 0.0f || mBodies.empty())
        return;

    pushTransforms();

    if (mFixedStep <= 0.0f)
    {
        mWorld.Step(deltaTime);
    }
    else
    {
        mAccumulator += deltaTime;
        const float maxAccumulated = mFixedStep * 8.0f;
        if (mAccumulator > maxAccumulated)
            mAccumulator = maxAccumulated;
        while (mAccumulator >= mFixedStep)
        {
            mWorld.Step(mFixedStep);
            mAccumulator -= mFixedStep;
        }
    }

    pullTransforms();
}

void PhysicsWorld2D::onContact(const kx::ContactEvent& event, void* context)
{
    PhysicsWorld2D& world = *static_cast<PhysicsWorld2D*>(context);
    if (!world.mCallback || event.phase == kx::ContactPhase::Persist)
        return;

    CollisionInfo info;
    info.self = objectFor(event.self);
    info.other = objectFor(event.other);
    info.hit = true;
    info.sensor = event.sensor;
    info.began = event.phase == kx::ContactPhase::Begin;
    if (event.manifold && event.manifold->pointCount > 0 && event.self && event.other)
    {
        const float radiusSelf = kx::ShapeRadius(event.self->Shapes()[event.shapeIndexSelf]);
        const float radiusOther = kx::ShapeRadius(event.other->Shapes()[event.shapeIndexOther]);
        kx::WorldManifold worldManifold;
        worldManifold.Initialize(event.manifold, event.self->GetTransform(), radiusSelf, event.other->GetTransform(),
                                 radiusOther);
        info.point = worldManifold.points[0];
        info.normal = worldManifold.normal;
    }

    if (info.self && info.other)
        world.mCallback(info, world.mCallbackUser);
}

GameObject* PhysicsWorld2D::raycast(const Math::Vec2& origin, const Math::Vec2& direction, float distance,
                                    Math::Vec2* outPoint, Math::Vec2* outNormal, const GameObject* ignore)
{
    const float length = std::sqrt(direction.x * direction.x + direction.y * direction.y);
    if (length < 0.0001f || distance <= 0.0f)
        return nullptr;

    const Math::Vec2 translation(direction.x / length * distance, direction.y / length * distance);

    const kx::Body* ignoreBody = nullptr;
    if (ignore)
    {
        for (size_t i = 0; i < mBodies.size(); ++i)
            if (mBodies[i] && mBodies[i]->mBody && objectFor(mBodies[i]->mBody) == ignore)
                ignoreBody = mBodies[i]->mBody;
    }

    kx::RayCastHit hit;
    if (!mWorld.RayCastClosest(origin, translation, hit, 0xFFFF, false, ignoreBody))
        return nullptr;

    if (outPoint)
        *outPoint = hit.point;
    if (outNormal)
        *outNormal = hit.normal;
    return objectFor(hit.body);
}

GameObject* PhysicsWorld2D::objectAtPoint(const Math::Vec2& point)
{
    return objectFor(mWorld.BodyAtPoint(point, false));
}

void PhysicsWorld2D::overlapCircle(const Math::Vec2& center, float radius, ct::Vector<GameObject*>& out)
{
    out.clear();
    if (radius <= 0.0f)
        return;

    ct::Vector<kx::Body*> bodies;
    mWorld.QueryCircle(center, radius, bodies);
    for (size_t i = 0; i < bodies.size(); ++i)
        if (GameObject* object = objectFor(bodies[i]))
            out.push_back(object);
}

} // namespace k2d
