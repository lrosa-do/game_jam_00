#include <math.h>
#include "kx/world.h"
#include "kx/internal/collide.h"
#include "kx/internal/raycast.h"
#include "kx/shapecast.h"

namespace kx
{

namespace
{

AABB ComputeShapeAABB(const Shape& shape, const Transform& xf)
{
    switch (shape.type)
    {
    case ShapeType::Circle:
        return shape.circle.ComputeAABB(xf);
    case ShapeType::Polygon:
        return shape.polygon.ComputeAABB(xf);
    case ShapeType::Edge:
    default:
        return shape.edge.ComputeAABB(xf);
    }
}

AABB ComputeBodyAABB(const Body& body)
{
    Transform xf = body.GetTransform();
    AABB aabb = ComputeShapeAABB(body.Shapes()[0], xf);

    for (int i = 1; i < body.ShapeCount(); ++i)
    {
        AABB shapeAabb = ComputeShapeAABB(body.Shapes()[i], xf);
        aabb.lowerBound = Min(aabb.lowerBound, shapeAabb.lowerBound);
        aabb.upperBound = Max(aabb.upperBound, shapeAabb.upperBound);
    }

    return aabb;
}

AABB ComputeBodyAABB(const Body& body, const Transform& xf)
{
    AABB aabb = ComputeShapeAABB(body.Shapes()[0], xf);
    for (int i = 1; i < body.ShapeCount(); ++i)
    {
        const AABB shapeAabb = ComputeShapeAABB(body.Shapes()[i], xf);
        aabb.lowerBound = Min(aabb.lowerBound, shapeAabb.lowerBound);
        aabb.upperBound = Max(aabb.upperBound, shapeAabb.upperBound);
    }
    return aabb;
}

bool ContactHasStatic(const ContactInfo& c)
{
    return c.a->Type() == BodyType::Static || c.b->Type() == BodyType::Static;
}

// Only awake dynamic bodies are solved; a moving kinematic body wakes what it touches
// through the sleep islands.
bool BodyIsActive(const Body* b)
{
    return b->Type() == BodyType::Dynamic && b->IsAwake();
}

bool ContactIsActive(const ContactInfo& c)
{
    return BodyIsActive(c.a) || BodyIsActive(c.b);
}

bool JointIsActive(const Joint* joint)
{
    const Body* a = joint->BodyA();
    const Body* b = joint->BodyB();
    return (a && BodyIsActive(a)) || (b && BodyIsActive(b));
}

// Kinematic bodies never sleep; a slow platform must still carry what rests on it.
bool KinematicIsMoving(const Body* b)
{
    return b->Type() == BodyType::Kinematic &&
           (Dot(b->Velocity(), b->Velocity()) > 0.0f || b->AngularVelocity() != 0.0f);
}

struct BodyQueryVisitor
{
    const DynamicTree* tree;
    ct::Vector<Body*>* hits;

    bool QueryCallback(int32_t proxyId)
    {
        hits->push_back(static_cast<Body*>(tree->GetUserData(proxyId)));
        return true;
    }
};

bool CollideShapePair(Manifold& manifold, bool& flip, const Shape& sA, const Transform& xfA, const Shape& sB,
                      const Transform& xfB)
{
    flip = false;

    if (sA.type == ShapeType::Edge && sB.type == ShapeType::Edge)
        return false;

    if (sA.type == ShapeType::Circle && sB.type == ShapeType::Circle)
    {
        CollideCircles(&manifold, sA.circle, xfA, sB.circle, xfB);
        return true;
    }

    if (sA.type == ShapeType::Polygon && sB.type == ShapeType::Polygon)
    {
        CollidePolygons(&manifold, sA.polygon, xfA, sB.polygon, xfB);
        return true;
    }

    if (sA.type == ShapeType::Polygon && sB.type == ShapeType::Circle)
    {
        CollidePolygonAndCircle(&manifold, sA.polygon, xfA, sB.circle, xfB);
        return true;
    }
    if (sA.type == ShapeType::Circle && sB.type == ShapeType::Polygon)
    {
        flip = true;
        CollidePolygonAndCircle(&manifold, sB.polygon, xfB, sA.circle, xfA);
        return true;
    }

    if (sA.type == ShapeType::Edge && sB.type == ShapeType::Circle)
    {
        CollideEdgeAndCircle(&manifold, sA.edge, xfA, sB.circle, xfB);
        return true;
    }
    if (sA.type == ShapeType::Circle && sB.type == ShapeType::Edge)
    {
        flip = true;
        CollideEdgeAndCircle(&manifold, sB.edge, xfB, sA.circle, xfA);
        return true;
    }

    if (sA.type == ShapeType::Edge && sB.type == ShapeType::Polygon)
    {
        CollideEdgeAndPolygon(&manifold, sA.edge, xfA, sB.polygon, xfB);
        return true;
    }
    flip = true;
    CollideEdgeAndPolygon(&manifold, sB.edge, xfB, sA.polygon, xfA);
    return true;
}

// Overlap test for a pair of shapes placed at the given transforms. On
// overlap fills the contact point and the normal pointing out of the other
// shape toward self.
bool ShapeOverlap(const Shape& self, const Transform& selfTransform, const Shape& other,
                  const Transform& otherTransform, vec2& outPoint, vec2& outNormal)
{
    Manifold manifold;
    bool flipped = false;
    if (!CollideShapePair(manifold, flipped, self, selfTransform, other, otherTransform) || manifold.pointCount == 0)
        return false;

    WorldManifold worldManifold;
    if (flipped)
    {
        worldManifold.Initialize(&manifold, otherTransform, ShapeRadius(other), selfTransform, ShapeRadius(self));
        outNormal = worldManifold.normal;
    }
    else
    {
        worldManifold.Initialize(&manifold, selfTransform, ShapeRadius(self), otherTransform, ShapeRadius(other));
        outNormal = -worldManifold.normal;
    }
    outPoint = worldManifold.points[0];
    return true;
}

} // namespace

namespace
{
uint64_t PairKey(const Body* a, const Body* b);
}

World::World(const vec2& gravity)
    : mGravity(gravity), mUseTree(true), mClock(nullptr), mStepStamp(0), mNextBodyId(1), mVelocityIterations(8),
      mLockDepth(0), mStepping(false), mFlushing(false), mJointDestroyedCallback(nullptr),
      mJointDestroyedContext(nullptr)
{
    mProfile = StepProfile{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    mNarrowMs = 0.0f;
}

World::~World()
{
    for (size_t i = 0; i < mJoints.size(); ++i)
        delete mJoints[i];
}

Body* World::CreateBody(BodyType type, const vec2& pos, float angle)
{
    Body* body = mBodyPool.create();
    body->mType = type;
    body->mPosition = pos;
    body->mAngle = angle;

    if (!mFreeBodyIds.empty())
    {
        body->mId = mFreeBodyIds.back();
        mFreeBodyIds.pop_back();
    }
    else
    {
        body->mId = mNextBodyId++;
    }
    mBodies.push_back(body);
    return body;
}

Body* World::CreateBox(const vec2& pos, float halfWidth, float halfHeight, float density)
{
    Body* body = CreateBody(BodyType::Dynamic, pos);
    body->AddBox(halfWidth, halfHeight, vec2(0.0f, 0.0f), density);
    return body;
}

Body* World::CreateCircle(const vec2& pos, float radius, float density)
{
    Body* body = CreateBody(BodyType::Dynamic, pos);
    body->AddCircle(vec2(0.0f, 0.0f), radius, density);
    return body;
}

Body* World::CreateStaticBox(const vec2& pos, float halfWidth, float halfHeight)
{
    Body* body = CreateBody(BodyType::Static, pos);
    body->AddBox(halfWidth, halfHeight, vec2(0.0f, 0.0f), 1.0f);
    return body;
}

Body* World::CreateKinematicBox(const vec2& pos, float halfWidth, float halfHeight)
{
    Body* body = CreateBody(BodyType::Kinematic, pos);
    body->AddBox(halfWidth, halfHeight, vec2(0.0f, 0.0f), 1.0f);
    return body;
}

Body* World::CreateEdge(const vec2& a, const vec2& b)
{
    Body* body = CreateBody(BodyType::Static, vec2(0.0f, 0.0f));
    body->AddEdge(a, b);
    return body;
}

Body* World::CreateChain(const vec2* points, int count, bool loop)
{
    Body* body = CreateBody(BodyType::Static, vec2(0.0f, 0.0f));
    body->AddChain(points, count, loop);
    return body;
}

Body* World::CreatePolygon(const vec2& pos, const vec2* points, int count, float density)
{
    Body* body = CreateBody(BodyType::Dynamic, pos);
    body->AddPolygon(points, count, density);
    return body;
}

Body* World::CreateMesh(const vec2& pos, const vec2* outline, int count, float density)
{
    Body* body = CreateBody(BodyType::Dynamic, pos);
    body->AddMesh(outline, count, density);
    return body;
}

void World::Destroy(Body* body)
{
    if (!body || body->mDestroyPending)
        return;

    bool found = false;
    for (size_t i = 0; i < mBodies.size() && !found; ++i)
        found = mBodies[i] == body;
    if (!found)
        return;

    // Contact callbacks run while mContacts and mContactStates are being walked, so while
    // the world is locked the body is only marked and queued.
    if (IsLocked())
    {
        body->mDestroyPending = true;
        mPendingBodies.push_back(body);
        return;
    }
    DestroyImmediate(body);
}

void World::DestroyImmediate(Body* body)
{
    body->mDestroyPending = true;

    // Callbacks run from here on (End events, implicit joint destruction)
    // may destroy other bodies or joints: those are queued, then flushed.
    ++mLockDepth;
    RemoveBodyContactEvents(body);
    for (;;)
    {
        Joint* attached = nullptr;
        for (size_t i = 0; i < mJoints.size() && !attached; ++i)
        {
            Joint* joint = mJoints[i];
            if (joint->BodyA() == body || joint->BodyB() == body || joint->DependsOnBody(body))
                attached = joint;
        }
        if (!attached)
            break;
        DestroyJointInternal(attached, true);
    }
    --mLockDepth;

    RemoveBodyContacts(body);

    for (auto& entry : mPairs)
    {
        Body* other = nullptr;
        if (entry.value.a == body)
            other = entry.value.b;
        else if (entry.value.b == body)
            other = entry.value.a;
        if (other && other->Type() == BodyType::Dynamic)
            other->SetAwake(true);
    }

    if (body->mProxyId != kNullNode)
    {
        for (size_t i = 0; i < mMoveBuffer.size(); ++i)
            if (mMoveBuffer[i] == body->mProxyId)
                mMoveBuffer[i] = kNullNode;
        mTree.DestroyProxy(body->mProxyId);
        body->mProxyId = kNullNode;
    }

    mDeadPairs.clear();
    for (auto& entry : mPairs)
        if (entry.value.a == body || entry.value.b == body)
            mDeadPairs.push_back(entry.key);
    for (size_t i = 0; i < mDeadPairs.size(); ++i)
        mPairs.erase(mDeadPairs[i]);

    ct::Vector<uint64_t> staleKeys;
    for (auto& entry : mImpulseMap)
    {
        uint32_t idA = static_cast<uint32_t>((entry.key >> 37) & kBodyIdMask);
        uint32_t idB = static_cast<uint32_t>((entry.key >> 10) & kBodyIdMask);
        if (idA == body->mId || idB == body->mId)
            staleKeys.push_back(entry.key);
    }
    for (size_t i = 0; i < staleKeys.size(); ++i)
        mImpulseMap.erase(staleKeys[i]);

    for (size_t i = 0; i < mBodies.size(); ++i)
    {
        if (mBodies[i] == body)
        {
            mBodies[i] = mBodies.back();
            mBodies.pop_back();
            break;
        }
    }
    for (size_t i = 0; i < mPendingBodies.size(); ++i)
    {
        if (mPendingBodies[i] == body)
        {
            mPendingBodies[i] = mPendingBodies.back();
            mPendingBodies.pop_back();
            break;
        }
    }
    mFreeBodyIds.push_back(body->mId);
    mBodyPool.destroy(body);

    FlushPending();
}

void World::RemoveBodyContacts(Body* body)
{
    size_t kept = 0;
    for (size_t i = 0; i < mContacts.size(); ++i)
    {
        ContactInfo& contact = mContacts[i];
        if (contact.a == body || contact.b == body)
        {
            Body* other = contact.a == body ? contact.b : contact.a;
            if (other->Type() == BodyType::Dynamic)
                other->SetAwake(true);
            continue;
        }
        if (kept != i)
            mContacts[kept] = contact;
        ++kept;
    }
    while (mContacts.size() > kept)
        mContacts.pop_back();
    // They point into mContacts; rebuilt by the next step.
    mDynamicContacts.clear();
    mStaticContacts.clear();
}

void World::FlushPending()
{
    if (mFlushing || mLockDepth > 0)
        return;
    mFlushing = true;
    while (!mPendingBodies.empty() || !mPendingJoints.empty())
    {
        if (!mPendingJoints.empty())
        {
            Joint* joint = mPendingJoints.back();
            mPendingJoints.pop_back();
            DestroyJointInternal(joint, false);
            continue;
        }
        Body* body = mPendingBodies.back();
        mPendingBodies.pop_back();
        DestroyImmediate(body);
    }
    mFlushing = false;
}

void World::Unlock()
{
    --mLockDepth;
    FlushPending();
}

bool World::HasJoint(const Joint* joint) const
{
    for (size_t i = 0; i < mJoints.size(); ++i)
        if (mJoints[i] == joint)
            return true;
    return false;
}

void World::AddJoint(Joint* joint)
{
    if (!joint || HasJoint(joint))
        return;
    mJoints.push_back(joint);
}

void World::DestroyJoint(Joint* joint)
{
    if (!joint || !HasJoint(joint))
        return;
    if (IsLocked())
    {
        for (size_t i = 0; i < mPendingJoints.size(); ++i)
            if (mPendingJoints[i] == joint)
                return;
        mPendingJoints.push_back(joint);
        return;
    }
    DestroyJointInternal(joint, false);
    FlushPending();
}

void World::DestroyJointInternal(Joint* joint, bool implicit)
{
    bool found = false;
    for (size_t i = 0; i < mJoints.size(); ++i)
    {
        if (mJoints[i] == joint)
        {
            mJoints[i] = mJoints.back();
            mJoints.pop_back();
            found = true;
            break;
        }
    }
    if (!found)
        return;
    for (size_t i = 0; i < mPendingJoints.size(); ++i)
    {
        if (mPendingJoints[i] == joint)
        {
            mPendingJoints[i] = mPendingJoints.back();
            mPendingJoints.pop_back();
            break;
        }
    }

    // The user may hold this pointer (a mouse joint, say): tell them before it is deleted.
    if (implicit && mJointDestroyedCallback)
    {
        ++mLockDepth;
        mJointDestroyedCallback(joint, mJointDestroyedContext);
        --mLockDepth;
    }

    for (;;)
    {
        Joint* dependent = nullptr;
        for (size_t i = 0; i < mJoints.size() && !dependent; ++i)
            if (mJoints[i]->DependsOnJoint(joint))
                dependent = mJoints[i];
        if (!dependent)
            break;
        DestroyJointInternal(dependent, true);
    }

    joint->WakeBodies();
    delete joint;
}

static bool ShapeContainsPoint(const Shape& shape, const Transform& xf, const vec2& point)
{
    if (shape.type == ShapeType::Circle)
    {
        vec2 center = xf.Transform(shape.circle.center);
        vec2 d = point - center;
        return Dot(d, d) <= shape.circle.radius * shape.circle.radius;
    }
    if (shape.type == ShapeType::Polygon)
    {
        vec2 local = InvTransformPoint(xf, point);
        const Polygon& poly = shape.polygon;
        for (int32_t i = 0; i < poly.count; ++i)
        {
            if (Dot(poly.normals[i], local - poly.vertices[i]) > 0.0f)
                return false;
        }
        return true;
    }
    return false;
}

static vec2 ClosestPointOnSegment(const vec2& point, const vec2& a, const vec2& b)
{
    vec2 edge = b - a;
    float lengthSquared = Dot(edge, edge);
    if (lengthSquared <= kEpsilon)
        return a;
    float t = Clamp(Dot(point - a, edge) / lengthSquared, 0.0f, 1.0f);
    return a + t * edge;
}

static vec2 ClosestPointOnShape(const Shape& shape, const Transform& xf, const vec2& point)
{
    if (ShapeContainsPoint(shape, xf, point))
        return point;
    if (shape.type == ShapeType::Circle)
    {
        vec2 center = xf.Transform(shape.circle.center);
        vec2 delta = point - center;
        float length = sqrtf(Dot(delta, delta));
        return length > kEpsilon ? center + delta * (shape.circle.radius / length) : center;
    }

    vec2 closest = point;
    float bestDistance = 1.0e30f;
    int count = shape.type == ShapeType::Polygon ? shape.polygon.count : 2;
    for (int i = 0; i < count; ++i)
    {
        vec2 a;
        vec2 b;
        if (shape.type == ShapeType::Polygon)
        {
            a = xf.Transform(shape.polygon.vertices[i]);
            b = xf.Transform(shape.polygon.vertices[(i + 1) % count]);
        }
        else
        {
            a = xf.Transform(shape.edge.vertex1);
            b = xf.Transform(shape.edge.vertex2);
            if (i == 1)
                break;
        }
        vec2 candidate = ClosestPointOnSegment(point, a, b);
        float distance = DistanceSquared(point, candidate);
        if (distance < bestDistance)
        {
            bestDistance = distance;
            closest = candidate;
        }
    }
    return closest;
}

Body* World::BodyAtPoint(const vec2& point, bool dynamicOnly) const
{

    if (mUseTree)
    {

        const_cast<World*>(this)->SyncProxies();

        AABB pointAABB{point, point};
        ct::Vector<Body*> hits;
        BodyQueryVisitor visitor{&mTree, &hits};
        mTree.Query(&visitor, pointAABB);

        for (size_t i = 0; i < hits.size(); ++i)
        {
            Body* body = hits[i];
            if (dynamicOnly && body->Type() != BodyType::Dynamic)
                continue;
            Transform xf = body->GetTransform();
            for (int s = 0; s < body->ShapeCount(); ++s)
                if (ShapeContainsPoint(body->Shapes()[s], xf, point))
                    return body;
        }
        return nullptr;
    }

    for (size_t i = 0; i < mBodies.size(); ++i)
    {
        Body* body = mBodies[i];
        if (dynamicOnly && body->Type() != BodyType::Dynamic)
            continue;
        Transform xf = body->GetTransform();
        for (int s = 0; s < body->ShapeCount(); ++s)
        {
            if (ShapeContainsPoint(body->Shapes()[s], xf, point))
                return body;
        }
    }
    return nullptr;
}

void World::QueryAABB(const AABB& aabb, ct::Vector<Body*>& out) const
{
    out.clear();

    if (mUseTree)
    {
        const_cast<World*>(this)->SyncProxies();

        ct::Vector<Body*> hits;
        BodyQueryVisitor visitor{&mTree, &hits};
        mTree.Query(&visitor, aabb);

        for (size_t i = 0; i < hits.size(); ++i)
        {
            Body* body = hits[i];
            if (body->ShapeCount() > 0 && TestOverlap(aabb, ComputeBodyAABB(*body)))
                out.push_back(body);
        }
        return;
    }

    for (size_t i = 0; i < mBodies.size(); ++i)
    {
        Body* body = mBodies[i];
        if (body->ShapeCount() > 0 && TestOverlap(aabb, ComputeBodyAABB(*body)))
            out.push_back(body);
    }
}

bool World::TestMotion(const Body& body, const vec2& motion, MotionResult& out, float safeMargin,
                       bool includeSensors) const
{
    out = MotionResult();
    if (body.ShapeCount() == 0)
    {
        out.travel = motion;
        return false;
    }

    const Transform selfTransform = body.GetTransform();
    const AABB start = ComputeBodyAABB(body, selfTransform);
    Transform endTransform = selfTransform;
    endTransform.tx += motion.x;
    endTransform.ty += motion.y;
    const AABB end = ComputeBodyAABB(body, endTransform);
    const float margin = safeMargin > 0.0f ? safeMargin : 0.0f;
    const vec2 expansion(margin, margin);
    const AABB sweep{Min(start.lowerBound, end.lowerBound) - expansion,
                     Max(start.upperBound, end.upperBound) + expansion};

    mBodyScratch.clear();
    QueryAABB(sweep, mBodyScratch);

    ShapeCastOutput best;
    best.fraction = 1.0f;
    int bestSelfShape = -1;
    int bestOtherShape = -1;
    Body* bestBody = nullptr;
    const Shape* selfShapes = body.Shapes();
    for (size_t candidateIndex = 0; candidateIndex < mBodyScratch.size(); ++candidateIndex)
    {
        Body* other = mBodyScratch[candidateIndex];
        if (!other || other == &body || other->ShapeCount() == 0)
            continue;
        const Transform otherTransform = other->GetTransform();
        const Shape* otherShapes = other->Shapes();
        for (int selfIndex = 0; selfIndex < body.ShapeCount(); ++selfIndex)
        {
            const AABB selfSweep = ComputeShapeAABB(selfShapes[selfIndex], selfTransform);
            for (int otherIndex = 0; otherIndex < other->ShapeCount(); ++otherIndex)
            {
                const Shape& otherShape = otherShapes[otherIndex];
                if ((!includeSensors && (selfShapes[selfIndex].isSensor || otherShape.isSensor)) ||
                    !ShouldCollide(selfShapes[selfIndex].filter, otherShape.filter))
                    continue;

                const AABB otherAabb = ComputeShapeAABB(otherShape, otherTransform);
                AABB pairSweep{Min(selfSweep.lowerBound, selfSweep.lowerBound + motion) - expansion,
                               Max(selfSweep.upperBound, selfSweep.upperBound + motion) + expansion};
                if (!TestOverlap(pairSweep, otherAabb))
                    continue;

                ShapeCastInput cast;
                cast.shapeA = &otherShape;
                cast.transformA = otherTransform;
                cast.shapeB = &selfShapes[selfIndex];
                cast.transformB = selfTransform;
                cast.translationB = motion;
                ShapeCastOutput hit;
                const bool castHit = ShapeCast(cast, hit);
                if (!castHit && !hit.initialOverlap)
                    continue;
                if (!castHit || hit.fraction <= 0.0f)
                {
                    // The shapes start overlapped or touching (within the skin): a miss would read as a
                    // free path and a hit at 0 would pin a resting body. Blocked only when the motion
                    // leads further in.
                    const float motionLength = sqrtf(Dot(motion, motion));
                    vec2 overlapPoint, overlapNormal;
                    if (ShapeOverlap(selfShapes[selfIndex], selfTransform, otherShape, otherTransform, overlapPoint,
                                     overlapNormal))
                    {
                        hit.point = overlapPoint;
                        hit.normal = overlapNormal;
                    }
                    else if (!castHit)
                        continue;
                    if (Dot(motion, hit.normal) >= -1.0e-3f * motionLength)
                        continue;
                    hit.fraction = 0.0f;
                }
                if (hit.fraction >= best.fraction)
                    continue;

                // A->B normal; B is the moving body, so it already points out of the obstacle.
                best = hit;
                bestSelfShape = selfIndex;
                bestOtherShape = otherIndex;
                bestBody = other;
            }
        }
    }

    if (!bestBody)
    {
        out.travel = motion;
        return false;
    }

    const float length = sqrtf(Dot(motion, motion));
    const float safeFraction = length > kEpsilon ? Clamp(best.fraction - margin / length, 0.0f, 1.0f) : 0.0f;
    out.body = bestBody;
    out.shapeIndexSelf = bestSelfShape;
    out.shapeIndexOther = bestOtherShape;
    out.point = best.point;
    out.normal = best.normal;
    out.fraction = best.fraction;
    out.travel = safeFraction * motion;
    out.remainder = motion - out.travel;
    out.hit = true;
    return true;
}

bool World::TestPosition(const Body& body, const vec2& position, MotionResult& out, bool includeSensors) const
{
    out = MotionResult();
    if (body.ShapeCount() == 0)
        return false;

    const Transform selfTransform = MakeTransform(position, body.Angle());
    const AABB bounds = ComputeBodyAABB(body, selfTransform);
    mBodyScratch.clear();
    QueryAABB(bounds, mBodyScratch);
    const Shape* selfShapes = body.Shapes();
    for (size_t candidateIndex = 0; candidateIndex < mBodyScratch.size(); ++candidateIndex)
    {
        Body* other = mBodyScratch[candidateIndex];
        if (!other || other == &body || other->ShapeCount() == 0)
            continue;
        const Transform otherTransform = other->GetTransform();
        const Shape* otherShapes = other->Shapes();
        for (int selfIndex = 0; selfIndex < body.ShapeCount(); ++selfIndex)
        {
            const AABB selfAabb = ComputeShapeAABB(selfShapes[selfIndex], selfTransform);
            for (int otherIndex = 0; otherIndex < other->ShapeCount(); ++otherIndex)
            {
                const Shape& otherShape = otherShapes[otherIndex];
                if ((!includeSensors && (selfShapes[selfIndex].isSensor || otherShape.isSensor)) ||
                    !ShouldCollide(selfShapes[selfIndex].filter, otherShape.filter) ||
                    !TestOverlap(selfAabb, ComputeShapeAABB(otherShape, otherTransform)))
                    continue;

                vec2 point, normal;
                if (!ShapeOverlap(selfShapes[selfIndex], selfTransform, otherShape, otherTransform, point, normal))
                    continue;

                out.normal = normal;
                out.body = other;
                out.shapeIndexSelf = selfIndex;
                out.shapeIndexOther = otherIndex;
                out.point = point;
                out.fraction = 0.0f;
                out.hit = true;
                return true;
            }
        }
    }
    return false;
}

void World::QueryCircle(const vec2& center, float radius, ct::Vector<Body*>& out) const
{
    out.clear();
    float radiusSquared = radius * radius;
    vec2 r(radius, radius);

    if (mUseTree)
    {
        const_cast<World*>(this)->SyncProxies();

        AABB queryAABB{center - r, center + r};
        ct::Vector<Body*> hits;
        BodyQueryVisitor visitor{&mTree, &hits};
        mTree.Query(&visitor, queryAABB);

        for (size_t i = 0; i < hits.size(); ++i)
        {
            Body* body = hits[i];
            if (body->ShapeCount() == 0)
                continue;
            Transform xf = body->GetTransform();
            for (int s = 0; s < body->ShapeCount(); ++s)
            {
                vec2 closest = ClosestPointOnShape(body->Shapes()[s], xf, center);
                if (DistanceSquared(center, closest) <= radiusSquared)
                {
                    out.push_back(body);
                    break;
                }
            }
        }
        return;
    }

    for (size_t i = 0; i < mBodies.size(); ++i)
    {
        Body* body = mBodies[i];
        if (body->ShapeCount() == 0)
            continue;
        Transform xf = body->GetTransform();
        bool hit = false;
        for (int s = 0; s < body->ShapeCount(); ++s)
        {
            vec2 closest = ClosestPointOnShape(body->Shapes()[s], xf, center);
            if (DistanceSquared(center, closest) <= radiusSquared)
            {
                hit = true;
                break;
            }
        }
        if (hit)
            out.push_back(body);
    }
}

void World::RayCastGather(const vec2& origin, const vec2& translation, uint16_t categoryMask,
                          bool includeSensors, const Body* ignoreBody, bool stopAtFirst,
                          ct::Vector<RayCastHit>& outHits) const
{
    outHits.clear();

    vec2 endPoint = origin + translation;
    AABB segmentAABB{Min(origin, endPoint), Max(origin, endPoint)};

    mBodyScratch.clear();
    if (mUseTree)
    {
        const_cast<World*>(this)->SyncProxies();

        BodyQueryVisitor visitor{&mTree, &mBodyScratch};
        mTree.Query(&visitor, segmentAABB);
    }
    else
    {
        mBodyScratch = mBodies;
    }

    float bestFraction = 1.0f;
    for (size_t i = 0; i < mBodyScratch.size(); ++i)
    {
        Body* body = mBodyScratch[i];
        if (body->ShapeCount() == 0 || body == ignoreBody)
            continue;
        Transform xf = body->GetTransform();
        for (int s = 0; s < body->ShapeCount(); ++s)
        {
            const Shape& shape = body->Shapes()[s];
            if (shape.isSensor && !includeSensors)
                continue;
            if ((shape.filter.category & categoryMask) == 0)
                continue;

            ShapeRayCastOutput result;
            float maxFraction = stopAtFirst ? bestFraction : 1.0f;
            if (!RayCastShape(origin, translation, maxFraction, shape, xf, result))
                continue;

            RayCastHit hit;
            hit.body = body;
            hit.shapeIndex = s;
            hit.point = origin + result.fraction * translation;
            hit.normal = result.normal;
            hit.fraction = result.fraction;
            outHits.push_back(hit);

            if (stopAtFirst && result.fraction < bestFraction)
                bestFraction = result.fraction;
        }
    }
}

bool World::RayCastClosest(const vec2& origin, const vec2& translation, RayCastHit& outHit,
                           uint16_t categoryMask, bool includeSensors, const Body* ignoreBody) const
{
    RayCastGather(origin, translation, categoryMask, includeSensors, ignoreBody, true, mRayScratch);
    if (mRayScratch.empty())
        return false;

    size_t bestIndex = 0;
    for (size_t i = 1; i < mRayScratch.size(); ++i)
        if (mRayScratch[i].fraction < mRayScratch[bestIndex].fraction)
            bestIndex = i;
    outHit = mRayScratch[bestIndex];
    return true;
}

void World::RayCastAll(const vec2& origin, const vec2& translation, ct::Vector<RayCastHit>& outHits,
                       uint16_t categoryMask, bool includeSensors, const Body* ignoreBody) const
{
    RayCastGather(origin, translation, categoryMask, includeSensors, ignoreBody, false, outHits);
}

void Explode(World& world, const vec2& center, float radius, float force, float falloff)
{
    if (radius <= 0.0f || force == 0.0f)
        return;

    ct::Vector<Body*> bodies;
    world.QueryCircle(center, radius, bodies);
    for (size_t i = 0; i < bodies.size(); ++i)
    {
        Body* body = bodies[i];
        if (body->Type() != BodyType::Dynamic)
            continue;

        Transform xf = body->GetTransform();
        vec2 point = body->WorldCenter();
        float closestDistance = 1.0e30f;
        for (int s = 0; s < body->ShapeCount(); ++s)
        {
            vec2 candidate = ClosestPointOnShape(body->Shapes()[s], xf, center);
            float candidateDistance = DistanceSquared(center, candidate);
            if (candidateDistance < closestDistance)
            {
                closestDistance = candidateDistance;
                point = candidate;
            }
        }
        vec2 delta = point - center;
        float distance = sqrtf(Dot(delta, delta));
        if (distance >= radius)
            continue;
        vec2 direction = distance > kEpsilon ? delta / distance : vec2(0.0f, -1.0f);
        float amount = 1.0f - distance / radius;
        if (falloff > 0.0f)
            amount = powf(amount, falloff);
        body->ApplyImpulse(direction * (force * amount), point);
    }
}

namespace
{
uint64_t PairKey(const Body* a, const Body* b)
{
    uint32_t lo = a->Id() < b->Id() ? a->Id() : b->Id();
    uint32_t hi = a->Id() < b->Id() ? b->Id() : a->Id();
    return (static_cast<uint64_t>(lo) << 32) | hi;
}

struct PairQueryVisitor
{
    const DynamicTree* tree;
    int32_t queryProxyId;
    ct::Vector<int32_t>* hits;

    bool QueryCallback(int32_t proxyId)
    {
        if (proxyId == queryProxyId)
            return true;
        if (tree->WasMoved(proxyId) && proxyId > queryProxyId)
            return true;
        hits->push_back(proxyId);
        return true;
    }
};
} // namespace

void World::SyncProxies()
{
    for (size_t i = 0; i < mBodies.size(); ++i)
    {
        Body* b = mBodies[i];
        if (b->ShapeCount() == 0)
            continue;

        AABB aabb = ComputeBodyAABB(*b);
        b->mTightAABB = aabb;

        if (b->mProxyId == kNullNode)
        {
            b->mProxyId = mTree.CreateProxy(aabb, b);
            b->mProxyPosition = b->mPosition;
            mMoveBuffer.push_back(b->mProxyId);
            continue;
        }

        vec2 displacement = b->mPosition - b->mProxyPosition;
        b->mProxyPosition = b->mPosition;
        if (mTree.MoveProxy(b->mProxyId, aabb, displacement))
            mMoveBuffer.push_back(b->mProxyId);
    }
}

void World::FindNewPairs()
{
    for (size_t i = 0; i < mMoveBuffer.size(); ++i)
    {
        int32_t queryProxyId = mMoveBuffer[i];
        if (queryProxyId == kNullNode)
            continue;

        mPairScratch.clear();
        PairQueryVisitor visitor{&mTree, queryProxyId, &mPairScratch};
        mTree.Query(&visitor, mTree.GetFatAABB(queryProxyId));

        Body* self = static_cast<Body*>(mTree.GetUserData(queryProxyId));
        for (size_t h = 0; h < mPairScratch.size(); ++h)
        {
            Body* other = static_cast<Body*>(mTree.GetUserData(mPairScratch[h]));
            if (self->Type() != BodyType::Dynamic && other->Type() != BodyType::Dynamic)
                continue;
            uint64_t key = PairKey(self, other);
            if (!mPairs.find(key))
                mPairs.put(key, BodyPair{self, other});
        }
    }

    for (size_t i = 0; i < mMoveBuffer.size(); ++i)
        if (mMoveBuffer[i] != kNullNode)
            mTree.ClearMoved(mMoveBuffer[i]);
    mMoveBuffer.clear();
}

bool World::JointsAllowCollision(const Body* a, const Body* b) const
{
    for (size_t i = 0; i < mJoints.size(); ++i)
    {
        Joint* joint = mJoints[i];
        Body* ja = joint->BodyA();
        Body* jb = joint->BodyB();
        if (((ja == a && jb == b) || (ja == b && jb == a)) && !joint->CollideConnected())
            return false;
    }
    return true;
}

void World::CollidePair(Body* first, Body* second)
{
    if (first->mId > second->mId)
    {
        Body* tmp = first;
        first = second;
        second = tmp;
    }

    if (!JointsAllowCollision(first, second))
        return;

    Transform xfi = first->GetTransform();
    Transform xfj = second->GetTransform();

    for (int si = 0; si < first->ShapeCount(); ++si)
    {
        AABB aabbA = ComputeShapeAABB(first->Shapes()[si], xfi);
        for (int sj = 0; sj < second->ShapeCount(); ++sj)
        {
            if (!ShouldCollide(first->Shapes()[si].filter, second->Shapes()[sj].filter))
                continue;

            AABB aabbB = ComputeShapeAABB(second->Shapes()[sj], xfj);
            if (!TestOverlap(aabbA, aabbB))
                continue;

            Manifold manifold;
            bool flip;
            if (!CollideShapePair(manifold, flip, first->Shapes()[si], xfi, second->Shapes()[sj], xfj))
                continue;

            if (manifold.pointCount == 0)
                continue;

            ContactInfo info;
            info.a = flip ? second : first;
            info.b = flip ? first : second;
            info.shapeIndexA = flip ? sj : si;
            info.shapeIndexB = flip ? si : sj;
            info.manifold = manifold;
            info.sensor = first->Shapes()[si].isSensor || second->Shapes()[sj].isSensor;
            info.solved = false;
            mContacts.push_back(info);
        }
    }
}

void World::UpdateContacts()
{
    mContacts.clear();
    if (mUseTree)
        UpdateContactsTree();
    else
        UpdateContactsBrute();
}

void World::UpdateContactsTree()
{
    SyncProxies();
    FindNewPairs();

    double n0 = mClock ? mClock() : 0.0;

    mDeadPairs.clear();
    for (auto& entry : mPairs)
    {
        Body* a = entry.value.a;
        Body* b = entry.value.b;

        if (!TestOverlap(mTree.GetFatAABB(a->mProxyId), mTree.GetFatAABB(b->mProxyId)))
        {
            mDeadPairs.push_back(entry.key);
            continue;
        }

        if (!TestOverlap(a->TightAABB(), b->TightAABB()))
            continue;

        CollidePair(a, b);
    }
    for (size_t i = 0; i < mDeadPairs.size(); ++i)
        mPairs.erase(mDeadPairs[i]);

    if (mClock)
        mNarrowMs = (float)((mClock() - n0) * 1000.0);
}

void World::UpdateContactsBrute()
{
    for (size_t i = 0; i < mBodies.size(); ++i)
    {
        Body* bi = mBodies[i];
        for (size_t j = i + 1; j < mBodies.size(); ++j)
        {
            Body* bj = mBodies[j];

            if (bi->Type() != BodyType::Dynamic && bj->Type() != BodyType::Dynamic)
                continue;

            AABB aabbA = ComputeBodyAABB(*bi);
            AABB aabbB = ComputeBodyAABB(*bj);
            if (!TestOverlap(aabbA, aabbB))
                continue;

            CollidePair(bi, bj);
        }
    }
}

void World::Step(float dt)
{
    // A Step from inside a callback would re-enter the containers being walked.
    if (dt <= 0.0f || IsLocked())
        return;

    mStepping = true;
    ++mStepStamp;

    double t0 = mClock ? mClock() : 0.0;

    for (size_t i = 0; i < mBodies.size(); ++i)
        mBodies[i]->IntegrateVelocity(mGravity, dt);

    double t1 = mClock ? mClock() : 0.0;

    UpdateContacts();
    ++mLockDepth;
    UpdateContactEvents();
    Unlock();

    BuildIslands();
    WakeIslands();

    double t2 = mClock ? mClock() : 0.0;

    InitContactConstraints();
    WarmStartContacts();

    mProfile.solveVelocityContacts = 0.0f;
    for (size_t i = 0; i < mJoints.size(); ++i)
        if (JointIsActive(mJoints[i]))
            mJoints[i]->InitVelocity(dt);

    for (int it = 0; it < mVelocityIterations; ++it)
    {
        for (size_t i = 0; i < mJoints.size(); ++i)
            if (JointIsActive(mJoints[i]))
                mJoints[i]->SolveVelocity(dt);
        SolveContactVelocities();
    }

    StoreContactImpulses();

    double t3 = mClock ? mClock() : 0.0;

    mBulletSweeps.clear();
    for (size_t i = 0; i < mBodies.size(); ++i)
    {
        Body* b = mBodies[i];
        if (b->IsBullet() && b->Type() == BodyType::Dynamic && b->IsAwake())
            mBulletSweeps.push_back(BulletSweep{b, b->WorldCenter()});
    }

    for (size_t i = 0; i < mBodies.size(); ++i)
        mBodies[i]->IntegratePosition(dt);

    double t4 = mClock ? mClock() : 0.0;

    SolveContactPositions();
    SolveBulletSweeps();
    UpdateSleeping(dt);

    mStepping = false;
    FlushPending();

    if (mClock)
    {
        double t5 = mClock();
        mProfile.integrate = (float)((t1 - t0 + t4 - t3) * 1000.0);
        mProfile.narrowphase = mNarrowMs;
        mProfile.broadphase = (float)((t2 - t1) * 1000.0) - mNarrowMs;
        mProfile.solveVelocity = (float)((t3 - t2) * 1000.0);
        mProfile.solveVelocityJoints = mProfile.solveVelocity - mProfile.solveVelocityContacts;
        mProfile.solvePosition = (float)((t5 - t4) * 1000.0);
    }
}

bool World::BulletRayCast(const Body* bullet, const vec2& origin, const vec2& translation, RayCastHit& outHit) const
{
    // Not RayCastClosest: it ignores the bullet's filter and joint collide rules, and stops
    // at a dynamic body the sweep skips, hiding the wall behind it.
    vec2 endPoint = origin + translation;
    AABB segmentAABB{Min(origin, endPoint), Max(origin, endPoint)};

    mBodyScratch.clear();
    if (mUseTree)
    {
        const_cast<World*>(this)->SyncProxies();
        BodyQueryVisitor visitor{&mTree, &mBodyScratch};
        mTree.Query(&visitor, segmentAABB);
    }
    else
    {
        mBodyScratch = mBodies;
    }

    float bestFraction = 1.0f;
    bool found = false;
    for (size_t i = 0; i < mBodyScratch.size(); ++i)
    {
        Body* other = mBodyScratch[i];
        if (other == bullet || other->Type() == BodyType::Dynamic || other->ShapeCount() == 0)
            continue;
        if (!JointsAllowCollision(bullet, other))
            continue;

        Transform xf = other->GetTransform();
        for (int s = 0; s < other->ShapeCount(); ++s)
        {
            const Shape& shape = other->Shapes()[s];
            if (shape.isSensor)
                continue;

            bool collides = false;
            for (int b = 0; b < bullet->ShapeCount() && !collides; ++b)
            {
                const Shape& bulletShape = bullet->Shapes()[b];
                collides = !bulletShape.isSensor && ShouldCollide(bulletShape.filter, shape.filter);
            }
            if (!collides)
                continue;

            ShapeRayCastOutput result;
            if (!RayCastShape(origin, translation, bestFraction, shape, xf, result))
                continue;

            bestFraction = result.fraction;
            outHit.body = other;
            outHit.shapeIndex = s;
            outHit.point = origin + result.fraction * translation;
            outHit.normal = result.normal;
            outHit.fraction = result.fraction;
            found = true;
        }
    }
    return found;
}

void World::SolveBulletSweeps()
{
    for (size_t i = 0; i < mBulletSweeps.size(); ++i)
    {
        Body* body = mBulletSweeps[i].body;
        vec2 prevCenter = mBulletSweeps[i].prevCenter;
        vec2 newCenter = body->WorldCenter();
        vec2 delta = newCenter - prevCenter;

        float distSq = Dot(delta, delta);
        if (distSq < kLinearSlop * kLinearSlop)
            continue;

        RayCastHit hit;
        if (!BulletRayCast(body, prevCenter, delta, hit))
            continue;

        float endSide = Dot(newCenter - hit.point, hit.normal);
        if (endSide >= -kLinearSlop)
            continue;

        float dist = sqrtf(distSq);
        float safeFraction = hit.fraction - kLinearSlop / dist;
        if (safeFraction < 0.0f)
            safeFraction = 0.0f;
        vec2 safeCenter = prevCenter + safeFraction * delta;

        body->ShiftCenter(safeCenter - newCenter, 0.0f);

        // Stopped at the surface; keeping the velocity into it would re-penetrate every step.
        float normalSpeed = Dot(body->mLinearVelocity, hit.normal);
        if (normalSpeed < 0.0f)
            body->mLinearVelocity -= normalSpeed * hit.normal;
    }
}

int32_t World::IslandRoot(int32_t index)
{
    int32_t root = index;
    while (mIslandParent[(size_t)root] != root)
        root = mIslandParent[(size_t)root];
    while (mIslandParent[(size_t)index] != root)
    {
        int32_t next = mIslandParent[(size_t)index];
        mIslandParent[(size_t)index] = root;
        index = next;
    }
    return root;
}

namespace
{
constexpr unsigned char kIslandAwake = 1;
constexpr unsigned char kIslandKeptAwake = 2;
} // namespace

void World::BuildIslands()
{
    // Islands: dynamic bodies linked by touching contacts or joints sleep and wake together.
    // Static and kinematic bodies do not link islands; a moving kinematic one keeps its islands awake.
    const size_t count = mBodies.size();
    mIslandParent.resize(count);
    mIslandFlags.resize(count);
    for (size_t i = 0; i < count; ++i)
    {
        mBodies[i]->mIslandIndex = (int32_t)i;
        mIslandParent[i] = (int32_t)i;
        mIslandFlags[i] = 0;
    }

    const auto link = [this](Body* a, Body* b)
    {
        const bool dynamicA = a->Type() == BodyType::Dynamic;
        const bool dynamicB = b->Type() == BodyType::Dynamic;
        if (dynamicA && dynamicB)
        {
            int32_t rootA = IslandRoot(a->mIslandIndex);
            int32_t rootB = IslandRoot(b->mIslandIndex);
            if (rootA != rootB)
                mIslandParent[(size_t)rootA] = rootB;
        }
        else if (dynamicA && KinematicIsMoving(b))
            mIslandFlags[(size_t)a->mIslandIndex] |= kIslandKeptAwake;
        else if (dynamicB && KinematicIsMoving(a))
            mIslandFlags[(size_t)b->mIslandIndex] |= kIslandKeptAwake;
    };

    for (size_t i = 0; i < mContacts.size(); ++i)
    {
        const ContactInfo& c = mContacts[i];
        if (!c.sensor && c.manifold.pointCount > 0)
            link(c.a, c.b);
    }
    for (size_t i = 0; i < mJoints.size(); ++i)
    {
        Body* bodies[4];
        const int bodyCount = mJoints[i]->ConnectedBodies(bodies);
        for (int k = 1; k < bodyCount; ++k)
            link(bodies[0], bodies[k]);
        // A joint whose first body is static or kinematic still ties the others together.
        for (int k = 2; k < bodyCount; ++k)
            link(bodies[1], bodies[k]);
    }

    for (size_t i = 0; i < count; ++i)
    {
        Body* body = mBodies[i];
        if (body->Type() != BodyType::Dynamic)
            continue;
        const int32_t root = IslandRoot((int32_t)i);
        if (body->mAwake)
            mIslandFlags[(size_t)root] |= kIslandAwake;
        if (mIslandFlags[i] & kIslandKeptAwake)
            mIslandFlags[(size_t)root] |= kIslandKeptAwake | kIslandAwake;
    }
}

void World::WakeIslands()
{
    // One awake body wakes its whole island, so the solver never pushes a body that will not integrate it.
    for (size_t i = 0; i < mBodies.size(); ++i)
    {
        Body* body = mBodies[i];
        if (body->Type() != BodyType::Dynamic || body->mAwake)
            continue;
        if (mIslandFlags[(size_t)IslandRoot((int32_t)i)] & kIslandAwake)
            body->SetAwake(true);
    }
}

void World::UpdateSleeping(float dt)
{
    // Each body under the thresholds accumulates its own sleep time; an island sleeps once its
    // least rested body has been still long enough.
    const size_t count = mBodies.size();
    if (mIslandParent.size() != count)
        BuildIslands();
    mIslandMinSleep.resize(count);
    for (size_t i = 0; i < count; ++i)
        mIslandMinSleep[i] = 1.0e30f;

    const float linearThresholdSquared = kSleepVelocity * kSleepVelocity;
    for (size_t i = 0; i < count; ++i)
    {
        Body* body = mBodies[i];
        if (body->Type() != BodyType::Dynamic || !body->mAwake)
            continue;

        const int32_t root = IslandRoot((int32_t)i);
        float linearSpeedSquared = Dot(body->mLinearVelocity, body->mLinearVelocity);
        if ((mIslandFlags[(size_t)root] & kIslandKeptAwake) || linearSpeedSquared > linearThresholdSquared ||
            fabsf(body->mAngularVelocity) > kSleepAngularVelocity)
            body->mSleepTime = 0.0f;
        else
            body->mSleepTime += dt;

        if (body->mSleepTime < mIslandMinSleep[(size_t)root])
            mIslandMinSleep[(size_t)root] = body->mSleepTime;
    }

    for (size_t i = 0; i < count; ++i)
    {
        Body* body = mBodies[i];
        if (body->Type() != BodyType::Dynamic || !body->mAwake)
            continue;
        if (mIslandMinSleep[(size_t)IslandRoot((int32_t)i)] >= kTimeToSleep)
            body->SetAwake(false);
    }
}

void World::SolveContactPointPosition(ContactInfo& c, int pointIndex, float baumgarte)
{
    Body* a = c.a;
    Body* b = c.b;
    float radiusA = ShapeRadius(a->Shapes()[c.shapeIndexA]);
    float radiusB = ShapeRadius(b->Shapes()[c.shapeIndexB]);

    Transform xfA = a->GetTransform();
    Transform xfB = b->GetTransform();

    vec2 normal;
    vec2 point;
    float separation;

    if (c.manifold.type == Manifold::kCircles)
    {
        vec2 pA = xfA.Transform(c.manifold.localPoint);
        vec2 pB = xfB.Transform(c.manifold.points[0].localPoint);
        vec2 d = pB - pA;
        float len = sqrtf(Dot(d, d));
        normal = len > kEpsilon ? d / len : vec2(0.0f, 1.0f);
        point = 0.5f * (pA + pB);
        separation = len - radiusA - radiusB;
    }
    else if (c.manifold.type == Manifold::kFaceA)
    {
        normal = Rotate(xfA, c.manifold.localNormal);
        vec2 planePoint = xfA.Transform(c.manifold.localPoint);
        vec2 clipPoint = xfB.Transform(c.manifold.points[pointIndex].localPoint);
        separation = Dot(clipPoint - planePoint, normal) - radiusA - radiusB;
        point = clipPoint;
    }
    else
    {
        normal = Rotate(xfB, c.manifold.localNormal);
        vec2 planePoint = xfB.Transform(c.manifold.localPoint);
        vec2 clipPoint = xfA.Transform(c.manifold.points[pointIndex].localPoint);
        separation = Dot(clipPoint - planePoint, normal) - radiusA - radiusB;
        point = clipPoint;
        normal = -normal;
    }

    vec2 rA = point - a->WorldCenter();
    vec2 rB = point - b->WorldCenter();

    float C = baumgarte * (separation + kLinearSlop);
    if (C < -kMaxLinearCorrection)
        C = -kMaxLinearCorrection;
    if (C > 0.0f)
        C = 0.0f;

    float rnA = Cross(rA, normal);
    float rnB = Cross(rB, normal);
    float K = a->mInvMass + b->mInvMass + a->mInvI * rnA * rnA + b->mInvI * rnB * rnB;

    float impulse = K > 0.0f ? -C / K : 0.0f;
    vec2 P = impulse * normal;

    a->ShiftCenter(-a->mInvMass * P, -a->mInvI * Cross(rA, P));
    b->ShiftCenter(b->mInvMass * P, b->mInvI * Cross(rB, P));
}

void World::SolveContactPositions()
{
    const int kPositionIterations = 3;

    for (int it = 0; it < kPositionIterations; ++it)
    {
        for (size_t i = 0; i < mJoints.size(); ++i)
            if (JointIsActive(mJoints[i]))
                mJoints[i]->SolvePosition();

        for (size_t i = 0; i < mDynamicContacts.size(); ++i)
        {
            ContactInfo& c = *mDynamicContacts[i];
            for (int i = 0; i < c.manifold.pointCount; ++i)
                SolveContactPointPosition(c, i, kBaumgarte);
        }
        for (size_t i = 0; i < mStaticContacts.size(); ++i)
        {
            ContactInfo& c = *mStaticContacts[i];
            for (int i = 0; i < c.manifold.pointCount; ++i)
                SolveContactPointPosition(c, i, kBaumgarte);
        }
    }
}

void World::InitContactConstraints()
{
    mDynamicContacts.clear();
    mStaticContacts.clear();
    for (size_t ci = 0; ci < mContacts.size(); ++ci)
    {
        ContactInfo& c = mContacts[ci];
        c.solved = false;
        if (c.sensor)
            continue;
        if (!ContactIsActive(c))
            continue;
        c.solved = true;
        if (ContactHasStatic(c))
            mStaticContacts.push_back(&c);
        else
            mDynamicContacts.push_back(&c);
        Body* a = c.a;
        Body* b = c.b;

        c.friction = sqrtf(a->mFriction * b->mFriction);
        c.restitution = a->mRestitution > b->mRestitution ? a->mRestitution : b->mRestitution;

        float radiusA = ShapeRadius(a->Shapes()[c.shapeIndexA]);
        float radiusB = ShapeRadius(b->Shapes()[c.shapeIndexB]);

        WorldManifold wm;
        wm.Initialize(&c.manifold, a->GetTransform(), radiusA, b->GetTransform(), radiusB);

        c.normal = wm.normal;
        c.tangent = vec2(-wm.normal.y, wm.normal.x);

        vec2 centerA = a->WorldCenter();
        vec2 centerB = b->WorldCenter();

        StoredImpulses* stored = mImpulseMap.find(ContactKey(c));

        for (int i = 0; i < c.manifold.pointCount; ++i)
        {
            ManifoldPoint& mp = c.manifold.points[i];

            mp.normalImpulse = 0.0f;
            mp.tangentImpulse = 0.0f;
            if (stored)
            {
                for (int k = 0; k < stored->count; ++k)
                {
                    if (stored->idKey[k] == mp.id.key)
                    {
                        mp.normalImpulse = stored->normalImpulse[k];
                        mp.tangentImpulse = stored->tangentImpulse[k];
                        break;
                    }
                }
            }

            c.rA[i] = wm.points[i] - centerA;
            c.rB[i] = wm.points[i] - centerB;

            float rnA = Cross(c.rA[i], c.normal);
            float rnB = Cross(c.rB[i], c.normal);
            float kNormal = a->mInvMass + b->mInvMass + a->mInvI * rnA * rnA + b->mInvI * rnB * rnB;
            c.normalMass[i] = kNormal > 0.0f ? 1.0f / kNormal : 0.0f;

            float rtA = Cross(c.rA[i], c.tangent);
            float rtB = Cross(c.rB[i], c.tangent);
            float kTangent = a->mInvMass + b->mInvMass + a->mInvI * rtA * rtA + b->mInvI * rtB * rtB;
            c.tangentMass[i] = kTangent > 0.0f ? 1.0f / kTangent : 0.0f;

            vec2 dv = b->mLinearVelocity + Cross(b->mAngularVelocity, c.rB[i]) - a->mLinearVelocity -
                            Cross(a->mAngularVelocity, c.rA[i]);
            float vn = Dot(dv, c.normal);

            float bias = 0.0f;
            if (vn < -kVelocityThreshold)
                bias = -c.restitution * vn;
            c.velocityBias[i] = bias;
        }
    }
}

void World::WarmStartContacts()
{
    const auto warmStart = [](ContactInfo* contact)
    {
        ContactInfo& c = *contact;
        for (int i = 0; i < c.manifold.pointCount; ++i)
        {
            const ManifoldPoint& mp = c.manifold.points[i];
            vec2 impulse = mp.normalImpulse * c.normal + mp.tangentImpulse * c.tangent;
            c.a->mLinearVelocity -= c.a->mInvMass * impulse;
            c.a->mAngularVelocity -= c.a->mInvI * Cross(c.rA[i], impulse);
            c.b->mLinearVelocity += c.b->mInvMass * impulse;
            c.b->mAngularVelocity += c.b->mInvI * Cross(c.rB[i], impulse);
        }
    };

    for (size_t i = 0; i < mDynamicContacts.size(); ++i)
        warmStart(mDynamicContacts[i]);
    for (size_t i = 0; i < mStaticContacts.size(); ++i)
        warmStart(mStaticContacts[i]);
}

void World::SolveContactVelocitiesOne(ContactInfo& c)
{
    Body* a = c.a;
    Body* b = c.b;

    for (int i = 0; i < c.manifold.pointCount; ++i)
    {
        ManifoldPoint& mp = c.manifold.points[i];

        vec2 dv = b->mLinearVelocity + Cross(b->mAngularVelocity, c.rB[i]) - a->mLinearVelocity -
                        Cross(a->mAngularVelocity, c.rA[i]);
        float vt = Dot(dv, c.tangent);
        float lambda = c.tangentMass[i] * (-vt);

        float maxFriction = c.friction * mp.normalImpulse;
        float newImpulse = mp.tangentImpulse + lambda;
        if (newImpulse < -maxFriction)
            newImpulse = -maxFriction;
        else if (newImpulse > maxFriction)
            newImpulse = maxFriction;
        lambda = newImpulse - mp.tangentImpulse;
        mp.tangentImpulse = newImpulse;

        vec2 impulse = lambda * c.tangent;
        a->mLinearVelocity -= a->mInvMass * impulse;
        a->mAngularVelocity -= a->mInvI * Cross(c.rA[i], impulse);
        b->mLinearVelocity += b->mInvMass * impulse;
        b->mAngularVelocity += b->mInvI * Cross(c.rB[i], impulse);
    }

    for (int i = 0; i < c.manifold.pointCount; ++i)
    {
        ManifoldPoint& mp = c.manifold.points[i];

        vec2 dv = b->mLinearVelocity + Cross(b->mAngularVelocity, c.rB[i]) - a->mLinearVelocity -
                        Cross(a->mAngularVelocity, c.rA[i]);
        float vn = Dot(dv, c.normal);
        float lambda = -c.normalMass[i] * (vn - c.velocityBias[i]);

        float newImpulse = mp.normalImpulse + lambda;
        if (newImpulse < 0.0f)
            newImpulse = 0.0f;
        lambda = newImpulse - mp.normalImpulse;
        mp.normalImpulse = newImpulse;

        vec2 impulse = lambda * c.normal;
        a->mLinearVelocity -= a->mInvMass * impulse;
        a->mAngularVelocity -= a->mInvI * Cross(c.rA[i], impulse);
        b->mLinearVelocity += b->mInvMass * impulse;
        b->mAngularVelocity += b->mInvI * Cross(c.rB[i], impulse);
    }
}

void World::SolveContactVelocities()
{
    double start = mClock ? mClock() : 0.0;
    for (size_t i = 0; i < mDynamicContacts.size(); ++i)
    {
        ContactInfo& c = *mDynamicContacts[i];
        SolveContactVelocitiesOne(c);
    }
    for (size_t i = 0; i < mStaticContacts.size(); ++i)
    {
        ContactInfo& c = *mStaticContacts[i];
        SolveContactVelocitiesOne(c);
    }
    if (mClock)
        mProfile.solveVelocityContacts += (float)((mClock() - start) * 1000.0);
}

void World::StoreContactImpulses()
{
    for (size_t ci = 0; ci < mContacts.size(); ++ci)
    {
        const ContactInfo& c = mContacts[ci];
        if (c.sensor)
            continue;
        if (!c.solved)
        {
            // A sleeping contact's impulses were not computed this step; keep them for the warm start on wake.
            if (StoredImpulses* kept = mImpulseMap.find(ContactKey(c)))
                kept->stamp = mStepStamp;
            continue;
        }
        StoredImpulses stored;
        stored.count = c.manifold.pointCount;
        for (int i = 0; i < c.manifold.pointCount; ++i)
        {
            stored.idKey[i] = c.manifold.points[i].id.key;
            stored.normalImpulse[i] = c.manifold.points[i].normalImpulse;
            stored.tangentImpulse[i] = c.manifold.points[i].tangentImpulse;
        }
        stored.stamp = mStepStamp;
        mImpulseMap.put(ContactKey(c), stored);
    }

    mStaleKeys.clear();
    for (auto& entry : mImpulseMap)
    {
        if (entry.value.stamp != mStepStamp)
            mStaleKeys.push_back(entry.key);
    }
    for (size_t i = 0; i < mStaleKeys.size(); ++i)
        mImpulseMap.erase(mStaleKeys[i]);
}

void World::UpdateContactEvents()
{
    for (size_t i = 0; i < mContacts.size(); ++i)
    {
        const ContactInfo& contact = mContacts[i];
        uint64_t key = ContactKey(contact);
        ContactState* state = mContactStates.find(key);
        ContactPhase phase = state ? ContactPhase::Persist : ContactPhase::Begin;

        ContactState current;
        current.a = contact.a;
        current.b = contact.b;
        current.shapeIndexA = contact.shapeIndexA;
        current.shapeIndexB = contact.shapeIndexB;
        current.manifold = contact.manifold;
        current.sensor = contact.sensor;
        current.stamp = mStepStamp;
        mContactStates.put(key, current);
        DispatchContactEvent(phase, current);
    }


    ct::Vector<ContactState> ended;
    mStaleKeys.clear();
    for (auto& entry : mContactStates)
    {
        if (entry.value.stamp != mStepStamp)
        {
            ended.push_back(entry.value);
            mStaleKeys.push_back(entry.key);
        }
    }
    for (size_t i = 0; i < mStaleKeys.size(); ++i)
        mContactStates.erase(mStaleKeys[i]);
    for (size_t i = 0; i < ended.size(); ++i)
        DispatchContactEvent(ContactPhase::End, ended[i]);
}

void World::DispatchContactEvent(ContactPhase phase, const ContactState& state)
{
    // A pending-destroy body hears nothing more; its partners still get End so every Begin is closed.
    if (state.a->mContactCallback && !state.a->mDestroyPending)
    {
        ContactEvent event{phase,           state.a,     state.b, state.shapeIndexA, state.shapeIndexB,
                           &state.manifold, state.sensor};
        state.a->mContactCallback(event, state.a->mContactContext);
    }
    if (state.b->mContactCallback && !state.b->mDestroyPending)
    {
        ContactEvent event{phase,           state.b,     state.a, state.shapeIndexB, state.shapeIndexA,
                           &state.manifold, state.sensor};
        state.b->mContactCallback(event, state.b->mContactContext);
    }
}

void World::RemoveBodyContactEvents(Body* body)
{
    // Erased before the End events go out: the map must not change while it is walked.
    ct::Vector<ContactState> ended;
    ct::Vector<uint64_t> keys;
    for (auto& entry : mContactStates)
    {
        if (entry.value.a == body || entry.value.b == body)
        {
            ended.push_back(entry.value);
            keys.push_back(entry.key);
        }
    }
    for (size_t i = 0; i < keys.size(); ++i)
        mContactStates.erase(keys[i]);
    for (size_t i = 0; i < ended.size(); ++i)
        DispatchContactEvent(ContactPhase::End, ended[i]);
}

} // namespace kx
