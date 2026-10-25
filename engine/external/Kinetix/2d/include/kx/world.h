#pragma once

#include "math.h"
#include <ct/hashmap.hpp>
#include <ct/pool.hpp>
#include <ct/vector.hpp>

#include "body.h"
#include "dynamictree.h"
#include "joints/joint.h"
#include "manifold.h"

namespace kx
{

struct ContactInfo
{
    Body* a;
    Body* b;
    int shapeIndexA;
    int shapeIndexB;
    Manifold manifold;
    vec2 normal;
    vec2 tangent;
    vec2 rA[kMaxManifoldPoints];
    vec2 rB[kMaxManifoldPoints];
    float normalMass[kMaxManifoldPoints];
    float tangentMass[kMaxManifoldPoints];
    float velocityBias[kMaxManifoldPoints];
    float friction;
    float restitution;
    bool sensor;
    // Set when the step's solver ran this contact; only those impulses are
    // worth keeping for the next warm start.
    bool solved;
};

enum class ContactPhase : unsigned char
{
    Begin,
    Persist,
    End
};

struct ContactEvent
{
    ContactPhase phase;
    Body* self;
    Body* other;
    int shapeIndexSelf;
    int shapeIndexOther;
    const Manifold* manifold;
    bool sensor;
};

struct BodyPair
{
    Body* a;
    Body* b;
};

struct StoredImpulses
{
    int count;
    uint32_t idKey[kMaxManifoldPoints];
    float normalImpulse[kMaxManifoldPoints];
    float tangentImpulse[kMaxManifoldPoints];
    uint32_t stamp;
};

struct ContactState
{
    Body* a;
    Body* b;
    int shapeIndexA;
    int shapeIndexB;
    Manifold manifold;
    bool sensor;
    uint32_t stamp;
};

struct StepProfile
{
    float broadphase;
    float narrowphase;
    float solveVelocity;
    float solveVelocityJoints;
    float solveVelocityContacts;
    float solvePosition;
    float integrate;
};

struct RayCastHit
{
    Body* body;
    int shapeIndex;
    vec2 point;
    vec2 normal;
    float fraction;
};

// Result shared by kinematic movement and non-mutating placement tests.
// The normal always points away from the obstacle and toward the tested body.
struct MotionResult
{
    Body* body = nullptr;
    int shapeIndexSelf = -1;
    int shapeIndexOther = -1;
    vec2 travel = vec2(0.0f, 0.0f);
    vec2 remainder = vec2(0.0f, 0.0f);
    vec2 point = vec2(0.0f, 0.0f);
    vec2 normal = vec2(0.0f, 0.0f);
    float fraction = 1.0f;
    bool hit = false;
};

struct BulletSweep
{
    Body* body;
    vec2 prevCenter;
};

// Called when the world deletes a joint the user did not destroy directly:
// one attached to a destroyed (or sliced) body, or one that depended on a
// destroyed joint. The joint is deleted right after the callback returns, so
// this is where the user clears every pointer they hold to it.
typedef void (*JointDestroyedCallback)(Joint* joint, void* context);

class World
{
  public:
    explicit World(const vec2& gravity);
    ~World();

    World(const World&) = delete;
    World& operator=(const World&) = delete;

    Body* CreateBody(BodyType type, const vec2& pos, float angle = 0.0f);
    Body* CreateBox(const vec2& pos, float halfWidth, float halfHeight, float density);
    Body* CreateCircle(const vec2& pos, float radius, float density);
    Body* CreateStaticBox(const vec2& pos, float halfWidth, float halfHeight);
    Body* CreateKinematicBox(const vec2& pos, float halfWidth, float halfHeight);
    Body* CreateEdge(const vec2& a, const vec2& b);
    Body* CreateChain(const vec2* points, int count, bool loop);
    Body* CreatePolygon(const vec2& pos, const vec2* points, int count, float density);
    Body* CreateMesh(const vec2& pos, const vec2* outline, int count, float density);
    Body* CreateFromImage(const vec2& pos, const unsigned char* pixels, int width, int height, int bpp,
                          unsigned char threshold, float density, float scale = 1.0f, float simplifyDegrees = 2.0f);
    // Destroys the body and the joints attached to it. Contact partners get
    // End events; the destroyed body's own callback is never called again.
    // Called while the world is stepping or dispatching callbacks - from a
    // contact callback, say - the destruction is deferred until the dispatch
    // is over: the body stays valid until then but gets no more callbacks
    // (Body::IsDestroyPending()). Destroying it twice is harmless.
    void Destroy(Body* body);

    Body* BodyAtPoint(const vec2& point, bool dynamicOnly = true) const;
    void QueryAABB(const AABB& aabb, ct::Vector<Body*>& out) const;
    void QueryCircle(const vec2& center, float radius, ct::Vector<Body*>& out) const;

    // Exact linear convex casts (GJK shape cast). They do not advance the simulation
    // or move the body; the caller owns any transform change.
    // A shape that already overlaps an obstacle (within the collision skin)
    // and moves further into it reports a hit with fraction 0 and zero
    // travel, the normal pointing out of the obstacle. Motion out of or
    // along the overlap is not blocked by that obstacle.
    bool TestMotion(const Body& body, const vec2& motion, MotionResult& out, float safeMargin = kLinearSlop,
                    bool includeSensors = false) const;
    bool TestPosition(const Body& body, const vec2& position, MotionResult& out,
                      bool includeSensors = false) const;

    bool RayCastClosest(const vec2& origin, const vec2& translation, RayCastHit& outHit,
                        uint16_t categoryMask = 0xFFFF, bool includeSensors = false,
                        const Body* ignoreBody = nullptr) const;

    void RayCastAll(const vec2& origin, const vec2& translation, ct::Vector<RayCastHit>& outHits,
                    uint16_t categoryMask = 0xFFFF, bool includeSensors = false,
                    const Body* ignoreBody = nullptr) const;

    // The world takes ownership. Adding the same joint twice is ignored.
    void AddJoint(Joint* joint);
    // Deferred like Destroy() while the world is locked.
    void DestroyJoint(Joint* joint);

    void SetJointDestroyedCallback(JointDestroyedCallback callback, void* context = nullptr)
    {
        mJointDestroyedCallback = callback;
        mJointDestroyedContext = context;
    }

    // True while Step() runs or callbacks are being dispatched; Destroy(),
    // DestroyJoint() and Slice() are deferred then, and Step() is ignored.
    bool IsLocked() const
    {
        return mLockDepth > 0 || mStepping;
    }

    void Step(float dt);

    const vec2& Gravity() const
    {
        return mGravity;
    }
    void SetGravity(const vec2& gravity)
    {
        mGravity = gravity;
    }

    const ct::Vector<Body*>& Bodies() const
    {
        return mBodies;
    }
    const ct::Vector<ContactInfo>& Contacts() const
    {
        return mContacts;
    }

    size_t BodyCount() const
    {
        return mBodies.size();
    }
    size_t ContactCount() const
    {
        return mContacts.size();
    }

    void SetVelocityIterations(int iterations)
    {
        mVelocityIterations = iterations;
    }

    void SetTreeBroadphase(bool enabled)
    {
        mUseTree = enabled;
    }
    bool TreeBroadphase() const
    {
        return mUseTree;
    }

    void SetTimeSource(double (*clock)())
    {
        mClock = clock;
    }
    const StepProfile& Profile() const
    {
        return mProfile;
    }

    const ct::Vector<Joint*>& Joints() const
    {
        return mJoints;
    }

  private:
    void UpdateContacts();
    void UpdateContactsBrute();
    void UpdateContactsTree();
    void SyncProxies();
    void FindNewPairs();
    void CollidePair(Body* first, Body* second);
    bool JointsAllowCollision(const Body* a, const Body* b) const;
    void InitContactConstraints();
    void WarmStartContacts();
    void SolveContactVelocitiesOne(ContactInfo& c);
    void SolveContactVelocities();
    void SolveContactPointPosition(ContactInfo& c, int pointIndex, float baumgarte);
    void SolveContactPositions();
    void StoreContactImpulses();
    void UpdateContactEvents();
    void DispatchContactEvent(ContactPhase phase, const ContactState& state);
    void RemoveBodyContactEvents(Body* body);
    void RemoveBodyContacts(Body* body);
    void BuildIslands();
    int32_t IslandRoot(int32_t index);
    void WakeIslands();
    void UpdateSleeping(float dt);

    void SolveBulletSweeps();
    bool BulletRayCast(const Body* bullet, const vec2& origin, const vec2& translation, RayCastHit& outHit) const;

    void DestroyImmediate(Body* body);
    void DestroyJointInternal(Joint* joint, bool implicit);
    bool HasJoint(const Joint* joint) const;
    void Unlock();
    void FlushPending();

    void RayCastGather(const vec2& origin, const vec2& translation, uint16_t categoryMask,
                       bool includeSensors, const Body* ignoreBody, bool stopAtFirst,
                       ct::Vector<RayCastHit>& outHits) const;

    static constexpr uint64_t kBodyIdBits = 27;
    static constexpr uint64_t kBodyIdMask = (uint64_t(1) << kBodyIdBits) - 1;

    static uint64_t ContactKey(const ContactInfo& c)
    {
        return (static_cast<uint64_t>(c.a->mId) << 37) | (static_cast<uint64_t>(c.b->mId) << 10) |
               (static_cast<uint64_t>(c.shapeIndexA) << 5) | static_cast<uint64_t>(c.shapeIndexB);
    }

    vec2 mGravity;
    ct::Pool<Body> mBodyPool;
    ct::Vector<Body*> mBodies;
    ct::Vector<ContactInfo> mContacts;
    ct::Vector<Joint*> mJoints;
    DynamicTree mTree;
    ct::Vector<int32_t> mMoveBuffer;
    ct::Vector<int32_t> mPairScratch;
    ct::HashMap<uint64_t, BodyPair> mPairs;
    ct::Vector<uint64_t> mDeadPairs;
    bool mUseTree;
    double (*mClock)();
    StepProfile mProfile;
    float mNarrowMs;
    ct::HashMap<uint64_t, StoredImpulses> mImpulseMap;
    ct::HashMap<uint64_t, ContactState> mContactStates;
    ct::Vector<ContactInfo *> mDynamicContacts;
    ct::Vector<ContactInfo *> mStaticContacts;
    ct::Vector<uint64_t> mStaleKeys;
    uint32_t mStepStamp;
    uint32_t mNextBodyId;
    ct::Vector<uint32_t> mFreeBodyIds;
    mutable ct::Vector<RayCastHit> mRayScratch;
    mutable ct::Vector<Body*> mBodyScratch;
    ct::Vector<BulletSweep> mBulletSweeps;
    int mVelocityIterations;

    // Deferred destruction (see Destroy()).
    int mLockDepth;
    bool mStepping;
    bool mFlushing;
    ct::Vector<Body*> mPendingBodies;
    ct::Vector<Joint*> mPendingJoints;
    JointDestroyedCallback mJointDestroyedCallback;
    void* mJointDestroyedContext;

    // Sleep islands: union-find over dynamic bodies joined by touching
    // contacts and joints, rebuilt every step.
    ct::Vector<int32_t> mIslandParent;
    ct::Vector<float> mIslandMinSleep;
    ct::Vector<unsigned char> mIslandFlags;
};

void Explode(World& world, const vec2& center, float radius, float force, float falloff);

// Cuts the body along the line through point with the given normal, replacing
// it with two bodies (the source is destroyed, deferred if the world is
// locked). Returns false and changes nothing when the line misses the body,
// when a side would get no usable shape, when the pieces need more shapes
// than a body holds, or when the body is already pending destruction.
bool Slice(World& world, Body* body, const vec2& point, const vec2& normal, Body** outPositive,
           Body** outNegative, float separationSpeed = 0.0f);

} // namespace kx
