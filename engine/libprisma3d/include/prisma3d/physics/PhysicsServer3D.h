#pragma once

#include "mathc.h"

#include <ct/function.hpp>
#include <ct/slotmap32.hpp>
#include <ct/vector.hpp>

#include <cstddef>
#include <cstdint>

namespace prisma3d
{

class BatchRenderer;
class CollisionMesh;
class Terrain;

struct PhysicsBodyTag;
using PhysicsBodyId = ct::Handle32<PhysicsBodyTag>;
struct PhysicsShapeTag;
using PhysicsShapeId = ct::Handle32<PhysicsShapeTag>;
struct PhysicsJointTag;
using PhysicsJointId = ct::Handle32<PhysicsJointTag>;
struct PhysicsVehicleTag;
using PhysicsVehicleId = ct::Handle32<PhysicsVehicleTag>;
struct PhysicsGroundVehicleTag;
using PhysicsGroundVehicleId = ct::Handle32<PhysicsGroundVehicleTag>;
struct PhysicsRagdollTag;
using PhysicsRagdollId = ct::Handle32<PhysicsRagdollTag>;
struct PhysicsSoftBodyTag;
using PhysicsSoftBodyId = ct::Handle32<PhysicsSoftBodyTag>;
struct PhysicsCharacterTag;
using PhysicsCharacterId = ct::Handle32<PhysicsCharacterTag>;

struct PhysicsCharacterParameters
{
    float radius = 0.4f;
    float height = 1.2f;
    float slopeLimit = 45.0f;
    float skinWidth = 0.02f;
    float stepOffset = 0.35f;
    float groundSnap = 0.15f;
    float wallMinSlideAngle = 0.0f;
    int maxIterations = 16;
    bool floorStopOnSlope = true;
    bool slideOnCeiling = true;
    Math::Vec3 up = Math::Vec3(0.0f, 1.0f, 0.0f);
    std::uint32_t collisionLayer = 1;
    std::uint32_t collisionMask = 0xFFFFFFFFu;
};

struct PhysicsCharacterState
{
    bool collided = false;
    bool onFloor = false;
    bool onWall = false;
    bool onCeiling = false;
    bool blocked = false;
    Math::Vec3 floorNormal = Math::Vec3(0.0f, 1.0f, 0.0f);
    Math::Vec3 wallNormal = Math::Vec3(0.0f, 0.0f, 0.0f);
    Math::Vec3 ceilingNormal = Math::Vec3(0.0f, 0.0f, 0.0f);
    Math::Vec3 position = Math::Vec3(0.0f, 0.0f, 0.0f);
    Math::Vec3 velocity = Math::Vec3(0.0f, 0.0f, 0.0f);
    float slopeAngle = 0.0f;
};

struct PhysicsCharacterRigidTag;
using PhysicsCharacterRigidId = ct::Handle32<PhysicsCharacterRigidTag>;

enum class PhysicsGroundState : std::uint8_t
{
    OnGround,
    OnSteepGround,
    NotSupported,
    InAir
};

struct PhysicsCharacterRigidParameters
{
    float maxSlopeAngle = 50.0f;
    float supportingHeight = 1.0e10f;
    float maxSeparation = 0.05f;
    Math::Vec3 up = Math::Vec3(0.0f, 1.0f, 0.0f);
};

struct PhysicsGroundInfo
{
    PhysicsGroundState state = PhysicsGroundState::InAir;
    Math::Vec3 normal = Math::Vec3(0.0f, 0.0f, 0.0f);
    Math::Vec3 position = Math::Vec3(0.0f, 0.0f, 0.0f);
    Math::Vec3 velocity = Math::Vec3(0.0f, 0.0f, 0.0f);
    PhysicsBodyId body;
};

struct PhysicsCharacterHit
{
    PhysicsBodyId body;
    Math::Vec3 normal = Math::Vec3(0.0f, 1.0f, 0.0f);
};

enum class PhysicsSoftBend : std::uint8_t
{
    None,
    Distance,
    Dihedral
};

struct PhysicsSoftBodyParameters
{
    float mass = 1.0f;
    float structuralCompliance = 0.0f;
    float bendCompliance = 0.001f;
    PhysicsSoftBend bend = PhysicsSoftBend::Distance;
    float pressure = 0.0f;
    int substeps = 4;
    float damping = 0.9048374f;
    float collisionMargin = 0.02f;
    std::uint32_t collisionGroup = 1;
    std::uint32_t collisionMask = 0xFFFFFFFFu;
};

enum class PhysicsRagdollPart : std::uint8_t
{
    Pelvis,
    Spine,
    Head,
    LeftUpperLeg,
    LeftLowerLeg,
    RightUpperLeg,
    RightLowerLeg,
    LeftUpperArm,
    LeftLowerArm,
    RightUpperArm,
    RightLowerArm,
    LeftFoot,
    RightFoot,
    Count
};

struct PhysicsRagdollSettings
{
    float scale = 1.0f;
    float fatness = 1.0f;
    float headSize = 1.0f;
    float massScale = 6.0f;
};

struct PhysicsRagdollBones
{
    int hips = -1;
    int spine = -1;
    int neck = -1;
    int head = -1;
    int leftArm = -1;
    int leftForeArm = -1;
    int leftHand = -1;
    int rightArm = -1;
    int rightForeArm = -1;
    int rightHand = -1;
    int leftUpLeg = -1;
    int leftLeg = -1;
    int leftFoot = -1;
    int leftToes = -1;
    int rightUpLeg = -1;
    int rightLeg = -1;
    int rightFoot = -1;
    int rightToes = -1;
};

struct PhysicsShard
{
    Math::Vec3 centroid = Math::Vec3(0.0f, 0.0f, 0.0f);
    float volume = 0.0f;
    ct::Vector<Math::Vec3> vertices;
    ct::Vector<std::uint32_t> indices;
};

struct PhysicsBonePose
{
    Math::Vec3 position = Math::Vec3(0.0f, 0.0f, 0.0f);
    Math::Quaternion rotation = Math::Quaternion::Identity();
    Math::Vec3 scale = Math::Vec3(1.0f, 1.0f, 1.0f);
};

enum class PhysicsBodyMode : std::uint8_t
{
    Static,
    Kinematic,
    Rigid
};

enum class PhysicsShapeType : std::uint8_t
{
    WorldBoundary,
    Sphere,
    Box,
    Capsule,
    ConcavePolygon,
    HeightMap,
    ConvexHull,
    Cylinder
};

enum class PhysicsDampMode : std::uint8_t
{
    Combine,
    Replace
};

enum class PhysicsJointType : std::uint8_t
{
    Pin,
    Hinge,
    Distance,
    Fixed,
    Slider,
    Piston,
    Universal,
    Wheel,
    Mouse,
    ConeTwist
};

struct PhysicsJointFrame
{
    Math::Vec3 anchor = Math::Vec3(0.0f, 0.0f, 0.0f);
    Math::Vec3 anchorB = Math::Vec3(0.0f, 0.0f, 0.0f);
    Math::Vec3 axis = Math::Vec3(1.0f, 0.0f, 0.0f);
    Math::Vec3 axisB = Math::Vec3(0.0f, 0.0f, 1.0f);
};

struct PhysicsContact
{
    PhysicsBodyId other;
    Math::Vec3 point = Math::Vec3(0.0f, 0.0f, 0.0f);
    Math::Vec3 normal = Math::Vec3(0.0f, 1.0f, 0.0f);
    float depth = 0.0f;
};

struct PhysicsContactImpulse
{
    PhysicsBodyId other;
    Math::Vec3 point = Math::Vec3(0.0f, 0.0f, 0.0f);
    Math::Vec3 normal = Math::Vec3(0.0f, 1.0f, 0.0f);
    float impulse = 0.0f;
};

struct PhysicsContactEvent
{
    PhysicsBodyId bodyA;
    PhysicsBodyId bodyB;
    bool entered = false;
    Math::Vec3 point = Math::Vec3(0.0f, 0.0f, 0.0f);
    Math::Vec3 normal = Math::Vec3(0.0f, 1.0f, 0.0f);
    float depth = 0.0f;
};

class PhysicsBodyListener
{
public:
    virtual ~PhysicsBodyListener()
    {
    }
    virtual void physicsBodyMoved(const Math::Mat4& frame)
    {
        (void) frame;
    }
    virtual void physicsSleepingChanged(bool sleeping)
    {
        (void) sleeping;
    }
    virtual void physicsContactStarted(const PhysicsContact& contact)
    {
        (void) contact;
    }
    virtual void physicsContactEnded(const PhysicsContact& contact)
    {
        (void) contact;
    }
};

struct PhysicsRayParameters
{
    Math::Vec3 from = Math::Vec3(0.0f, 0.0f, 0.0f);
    Math::Vec3 to = Math::Vec3(0.0f, -1.0f, 0.0f);
    std::uint32_t collisionMask = 0xFFFFFFFFu;
    PhysicsBodyId exclude;
};

struct PhysicsRayResult
{
    PhysicsBodyId body;
    Math::Vec3 position = Math::Vec3(0.0f, 0.0f, 0.0f);
    Math::Vec3 normal = Math::Vec3(0.0f, 1.0f, 0.0f);
    float distance = 0.0f;
    std::int32_t triangle = -1;
};

struct PhysicsWheelParameters
{
    Math::Vec3 connection = Math::Vec3(0.0f, 0.0f, 0.0f);
    Math::Vec3 direction = Math::Vec3(0.0f, -1.0f, 0.0f);
    Math::Vec3 axle = Math::Vec3(1.0f, 0.0f, 0.0f);
    float restLength = 0.15f;
    float travel = 0.2f;
    float radius = 0.5f;
    float stiffness = 5.88f;
    float dampingCompression = 0.83f;
    float dampingRelaxation = 0.88f;
    float frictionSlip = 10.5f;
    float maxForce = 6000.0f;
    float rollInfluence = 0.1f;
    bool front = false;
};

struct PhysicsWheelState
{
    Math::Mat4 worldTransform = Math::Mat4::Identity();
    bool inContact = false;
    PhysicsBodyId groundBody;
    Math::Vec3 contactPoint = Math::Vec3(0.0f, 0.0f, 0.0f);
    Math::Vec3 contactNormal = Math::Vec3(0.0f, 1.0f, 0.0f);
    float suspensionLength = 0.0f;
    float rotation = 0.0f;
    float skidInfo = 1.0f;
};

enum class PhysicsDriveLayout : std::uint8_t
{
    FrontWheel,
    RearWheel,
    AllWheel
};

struct PhysicsPassengerCarSettings
{
    PhysicsDriveLayout drive = PhysicsDriveLayout::RearWheel;
    bool automatic = true;
    bool antiLock = false;
};

struct PhysicsGroundVehicleLayout
{
    float mass = 0.0f;
    Math::Vec3 collisionHalfExtents = Math::Vec3(0.5f, 0.5f, 0.5f);
    float restHeight = 0.0f;
    float wheelbase = 0.0f;
    float track = 0.0f;
};

struct PhysicsGroundVehicleControls
{
    float throttle = 0.0f;
    float brake = 0.0f;
    float handbrake = 0.0f;
    float steering = 0.0f;
};

struct PhysicsGroundVehicleState
{
    float forwardSpeed = 0.0f;
    float lateralSpeed = 0.0f;
    float groundSpeed = 0.0f;
    float engineRpm = 0.0f;
    int gear = 0;
    float steeringAngle = 0.0f;
};

struct PhysicsGroundWheelState
{
    Math::Mat4 local = Math::Mat4::Identity();
    Math::Mat4 rest = Math::Mat4::Identity();
    bool inContact = false;
    PhysicsBodyId groundBody;
    Math::Vec3 contactPoint = Math::Vec3(0.0f, 0.0f, 0.0f);
    Math::Vec3 contactNormal = Math::Vec3(0.0f, 1.0f, 0.0f);
    float radius = 0.0f;
    float radiusScale = 1.0f;
    float halfTrack = 0.0f;
    float axlePosition = 0.0f;
    float slipRatio = 0.0f;
    float slipAngle = 0.0f;
    float angularVelocity = 0.0f;
    float load = 0.0f;
    float slide = 0.0f;
    bool detached = false;
};

struct PhysicsStats
{
    std::uint32_t bodies = 0;
    std::uint32_t awakeBodies = 0;
    std::uint32_t staticBodies = 0;
    std::uint32_t kinematicBodies = 0;
    std::uint32_t shapes = 0;
    std::uint32_t joints = 0;
    std::uint32_t vehicles = 0;
    std::uint32_t contacts = 0;
    std::uint32_t contactEvents = 0;
    std::uint64_t steps = 0;
    double stepMs = 0.0;
    double averageStepMs = 0.0;
    double maxStepMs = 0.0;
};

class PhysicsServer3D
{
public:
    PhysicsServer3D();
    ~PhysicsServer3D();
    PhysicsServer3D(const PhysicsServer3D&) = delete;
    PhysicsServer3D& operator=(const PhysicsServer3D&) = delete;

    static PhysicsServer3D* singleton();

    void step(double delta);
    void setActive(bool active);
    bool isActive() const;
    bool isStepping() const;
    void flushQueries();

    void setGravity(const Math::Vec3& gravity);
    const Math::Vec3& getGravity() const;
    void setDefaultLinearDamp(float damp);
    float getDefaultLinearDamp() const;
    void setDefaultAngularDamp(float damp);
    float getDefaultAngularDamp() const;
    void setSolverIterations(std::uint32_t velocityIterations, std::uint32_t positionIterations);
    void setSubsteps(std::uint32_t substeps);
    std::uint32_t getSubsteps() const;
    std::uint32_t getVelocityIterations() const;
    std::uint32_t getPositionIterations() const;
    void setContactMargin(float margin);
    float getContactMargin() const;

    PhysicsShapeId shapeCreate(PhysicsShapeType type);
    void shapeFree(PhysicsShapeId shape);
    bool shapeValid(PhysicsShapeId shape) const;
    PhysicsShapeType shapeGetType(PhysicsShapeId shape) const;
    void shapeSetBox(PhysicsShapeId shape, const Math::Vec3& halfExtents);
    Math::Vec3 shapeGetBox(PhysicsShapeId shape) const;
    void shapeSetSphere(PhysicsShapeId shape, float radius);
    float shapeGetSphere(PhysicsShapeId shape) const;
    void shapeSetCapsule(PhysicsShapeId shape, float radius, float height);
    void shapeSetCylinder(PhysicsShapeId shape, float radius, float height);
    float shapeGetCylinderRadius(PhysicsShapeId shape) const;
    float shapeGetCylinderHeight(PhysicsShapeId shape) const;
    float shapeGetCapsuleRadius(PhysicsShapeId shape) const;
    float shapeGetCapsuleHeight(PhysicsShapeId shape) const;
    void shapeSetWorldBoundary(PhysicsShapeId shape, const Math::Vec3& normal, float distance);
    void shapeSetConcaveFaces(PhysicsShapeId shape, const Math::Vec3* vertices,
            std::size_t vertexCount, const std::uint32_t* indices, std::size_t indexCount);
    void shapeSetConcaveCollision(PhysicsShapeId shape, const CollisionMesh& mesh);
    void shapeSetConvexHull(PhysicsShapeId shape, const Math::Vec3* points, std::size_t count);
    std::size_t shapeGetConvexVertexCount(PhysicsShapeId shape) const;
    Math::Vec3 shapeGetConvexCenter(PhysicsShapeId shape) const;
    void shapeSetHeightMap(PhysicsShapeId shape, int width, int depth, const float* heights);
    void shapeSetHeightMapFromTerrain(PhysicsShapeId shape, const Terrain& terrain);
    void shapeSetHeightMapTransform(PhysicsShapeId shape, const Math::Vec3& scale,
            const Math::Vec3& offset);
    bool shapeGetHeightMap(PhysicsShapeId shape, ct::Vector<float>& heights) const;
    bool shapeGetConcaveFaces(PhysicsShapeId shape, ct::Vector<Math::Vec3>& vertices,
            ct::Vector<std::uint32_t>& indices) const;
    int shapeGetHeightMapWidth(PhysicsShapeId shape) const;
    int shapeGetHeightMapDepth(PhysicsShapeId shape) const;
    Math::Vec3 shapeGetHeightMapScale(PhysicsShapeId shape) const;
    Math::Vec3 shapeGetHeightMapOffset(PhysicsShapeId shape) const;
    std::size_t shapeGetTriangleCount(PhysicsShapeId shape) const;

    PhysicsBodyId bodyCreate();
    void bodyFree(PhysicsBodyId body);
    bool bodyValid(PhysicsBodyId body) const;
    void bodySetMode(PhysicsBodyId body, PhysicsBodyMode mode);
    PhysicsBodyMode bodyGetMode(PhysicsBodyId body) const;
    void bodySetShape(PhysicsBodyId body, PhysicsShapeId shape, const Math::Mat4& localTransform);
    PhysicsShapeId bodyGetShape(PhysicsBodyId body) const;
    bool bodyIsSimulated(PhysicsBodyId body) const;
    void bodySetListener(PhysicsBodyId body, PhysicsBodyListener* listener);
    void bodySetUserData(PhysicsBodyId body, void* data);
    void* bodyGetUserData(PhysicsBodyId body) const;
    void bodySetEnabled(PhysicsBodyId body, bool enabled);
    bool bodyIsEnabled(PhysicsBodyId body) const;
    void bodySetInWorld(PhysicsBodyId body, bool inWorld);

    void bodySetTransform(PhysicsBodyId body, const Math::Mat4& frame);
    Math::Mat4 bodyGetTransform(PhysicsBodyId body) const;
    Math::Mat4 bodyGetInterpolatedTransform(PhysicsBodyId body, float fraction) const;
    Math::Vec3 bodyGetCenterOfMass(PhysicsBodyId body) const;

    void bodySetCollisionLayer(PhysicsBodyId body, std::uint32_t layer);
    std::uint32_t bodyGetCollisionLayer(PhysicsBodyId body) const;
    void bodySetCollisionMask(PhysicsBodyId body, std::uint32_t mask);
    std::uint32_t bodyGetCollisionMask(PhysicsBodyId body) const;
    void bodySetFriction(PhysicsBodyId body, float friction);
    float bodyGetFriction(PhysicsBodyId body) const;
    void bodySetBounce(PhysicsBodyId body, float bounce);
    float bodyGetBounce(PhysicsBodyId body) const;

    void bodySetMass(PhysicsBodyId body, float mass);
    float bodyGetMass(PhysicsBodyId body) const;
    void bodySetInertia(PhysicsBodyId body, const Math::Vec3& inertia);
    Math::Vec3 bodyGetInertia(PhysicsBodyId body) const;
    void bodySetLockRotation(PhysicsBodyId body, bool lock);
    bool bodyIsLockRotation(PhysicsBodyId body) const;
    void bodySetLinearDamp(PhysicsBodyId body, float damp, PhysicsDampMode mode);
    void bodySetAngularDamp(PhysicsBodyId body, float damp, PhysicsDampMode mode);
    void bodySetCanSleep(PhysicsBodyId body, bool canSleep);
    void bodySetInterpolated(PhysicsBodyId body, bool interpolated);
    bool bodyCanSleep(PhysicsBodyId body) const;
    void bodySetSleeping(PhysicsBodyId body, bool sleeping);
    bool bodyIsSleeping(PhysicsBodyId body) const;

    void bodySetLinearVelocity(PhysicsBodyId body, const Math::Vec3& velocity);
    Math::Vec3 bodyGetLinearVelocity(PhysicsBodyId body) const;
    void bodySetAngularVelocity(PhysicsBodyId body, const Math::Vec3& velocity);
    Math::Vec3 bodyGetAngularVelocity(PhysicsBodyId body) const;
    Math::Vec3 bodyGetVelocityAtPosition(PhysicsBodyId body, const Math::Vec3& position) const;
    void bodyApplyCentralImpulse(PhysicsBodyId body, const Math::Vec3& impulse);
    void bodyApplyImpulse(PhysicsBodyId body, const Math::Vec3& impulse, const Math::Vec3& offset);
    void bodyApplyTorqueImpulse(PhysicsBodyId body, const Math::Vec3& impulse);
    void bodyApplyCentralForce(PhysicsBodyId body, const Math::Vec3& force);
    void bodyApplyForce(PhysicsBodyId body, const Math::Vec3& force, const Math::Vec3& offset);
    void bodyApplyTorque(PhysicsBodyId body, const Math::Vec3& torque);
    void bodySetConstantForce(PhysicsBodyId body, const Math::Vec3& force);
    Math::Vec3 bodyGetConstantForce(PhysicsBodyId body) const;
    void bodySetConstantTorque(PhysicsBodyId body, const Math::Vec3& torque);
    Math::Vec3 bodyGetConstantTorque(PhysicsBodyId body) const;

    void bodyAddCollisionException(PhysicsBodyId body, PhysicsBodyId other);
    void bodyRemoveCollisionException(PhysicsBodyId body, PhysicsBodyId other);
    bool bodyHasCollisionException(PhysicsBodyId body, PhysicsBodyId other) const;

    void bodySetContactMonitor(PhysicsBodyId body, bool enabled, int maxContactsReported);
    int bodyGetContactCount(PhysicsBodyId body) const;
    void bodyGetCollidingBodies(PhysicsBodyId body, ct::Vector<PhysicsBodyId>& out) const;
    void bodySetImpulseMonitor(PhysicsBodyId body, bool enabled);
    const ct::Vector<PhysicsContactImpulse>* bodyGetContactImpulses(PhysicsBodyId body) const;

    PhysicsJointId jointCreate(PhysicsJointType type, PhysicsBodyId bodyA, PhysicsBodyId bodyB,
            const Math::Vec3& worldAnchor, const Math::Vec3& worldAxis);
    PhysicsJointId jointCreate(PhysicsJointType type, PhysicsBodyId bodyA, PhysicsBodyId bodyB,
            const PhysicsJointFrame& frame);
    void jointFree(PhysicsJointId joint);
    bool jointValid(PhysicsJointId joint) const;
    PhysicsJointType jointGetType(PhysicsJointId joint) const;
    void jointSetEnabled(PhysicsJointId joint, bool enabled);
    void jointSetLimits(PhysicsJointId joint, float lower, float upper);
    void jointSetSecondaryLimits(PhysicsJointId joint, float lower, float upper);
    void jointSetMotor(PhysicsJointId joint, float targetVelocity, float maxForce);
    void jointSetSecondaryMotor(PhysicsJointId joint, float targetVelocity, float maxForce);
    void jointSetPinMotor(PhysicsJointId joint, const Math::Vec3& worldAxis,
            float targetAngularVelocity, float maxTorque);
    void jointDisableMotor(PhysicsJointId joint);
    void jointDisableSecondaryMotor(PhysicsJointId joint);
    float jointGetPosition(PhysicsJointId joint) const;
    float jointGetSecondaryPosition(PhysicsJointId joint) const;
    Math::Vec3 jointGetReactionForce(PhysicsJointId joint) const;
    void jointSetDistance(PhysicsJointId joint, float minimum, float maximum);
    void jointSetSuspension(PhysicsJointId joint, float restLength, float stiffness, float damping);
    void jointSetSteeringServo(PhysicsJointId joint, float targetAngle, float maxTorque,
            float maxAngularVelocity);
    void jointDisableSteeringServo(PhysicsJointId joint);
    void jointSetTarget(PhysicsJointId joint, const Math::Vec3& target);
    void jointSetMouseSpring(PhysicsJointId joint, float maxForce, float frequencyHz,
            float dampingRatio);

    PhysicsVehicleId vehicleCreate(PhysicsBodyId chassis);
    void vehicleFree(PhysicsVehicleId vehicle);
    bool vehicleValid(PhysicsVehicleId vehicle) const;
    int vehicleAddWheel(PhysicsVehicleId vehicle, const PhysicsWheelParameters& parameters);
    int vehicleGetWheelCount(PhysicsVehicleId vehicle) const;
    void vehicleSetWheelParameters(PhysicsVehicleId vehicle, int wheel,
            const PhysicsWheelParameters& parameters);
    void vehicleSetEngineForce(PhysicsVehicleId vehicle, int wheel, float force);
    void vehicleSetBrake(PhysicsVehicleId vehicle, int wheel, float brake);
    void vehicleSetSteering(PhysicsVehicleId vehicle, int wheel, float steering);
    bool vehicleGetWheelState(PhysicsVehicleId vehicle, int wheel, PhysicsWheelState& state) const;
    float vehicleGetSpeedKmh(PhysicsVehicleId vehicle) const;
    void vehicleResetSuspension(PhysicsVehicleId vehicle);

    PhysicsGroundVehicleId groundVehicleCreatePassengerCar(PhysicsBodyId chassis,
            const PhysicsPassengerCarSettings& settings, PhysicsGroundVehicleLayout& layout);
    void groundVehicleFree(PhysicsGroundVehicleId vehicle);
    bool groundVehicleValid(PhysicsGroundVehicleId vehicle) const;
    void groundVehicleSetRayMask(PhysicsGroundVehicleId vehicle, std::uint32_t mask);
    void groundVehicleSetControls(PhysicsGroundVehicleId vehicle,
            const PhysicsGroundVehicleControls& controls);
    bool groundVehicleGetState(PhysicsGroundVehicleId vehicle,
            PhysicsGroundVehicleState& state) const;
    int groundVehicleGetWheelCount(PhysicsGroundVehicleId vehicle) const;
    bool groundVehicleGetWheelState(PhysicsGroundVehicleId vehicle, int wheel,
            PhysicsGroundWheelState& state) const;
    void groundVehicleSetWheelDetached(PhysicsGroundVehicleId vehicle, int wheel, bool detached);
    void groundVehicleSetWheelRadiusScale(PhysicsGroundVehicleId vehicle, int wheel, float scale);
    void groundVehicleStopWheels(PhysicsGroundVehicleId vehicle);
    void groundVehicleSelectGear(PhysicsGroundVehicleId vehicle, int gear);
    void groundVehicleSetAxle(PhysicsGroundVehicleId vehicle, int axle, float position,
            float halfTrack, float radius);
    float groundVehicleGetRestHeight(PhysicsGroundVehicleId vehicle) const;
    void groundVehicleSetPowerScale(PhysicsGroundVehicleId vehicle, float scale);
    void groundVehicleSetTopSpeed(PhysicsGroundVehicleId vehicle, float metresPerSecond);
    void groundVehicleSetGrip(PhysicsGroundVehicleId vehicle, float scale);
    void groundVehicleSetRollInfluence(PhysicsGroundVehicleId vehicle, float influence);
    void groundVehicleSetAxleGrip(PhysicsGroundVehicleId vehicle, int axle, float scale);
    void groundVehicleSetHandbrakeTorque(PhysicsGroundVehicleId vehicle, float torque);

    bool vehicleEnableMotorcycle(PhysicsVehicleId vehicle, bool enabled);
    bool vehicleIsMotorcycle(PhysicsVehicleId vehicle) const;
    void motorcycleSetSteerInput(PhysicsVehicleId vehicle, float input);
    void motorcycleSetMaxSteerAngle(PhysicsVehicleId vehicle, float angle);
    void motorcycleSetCasterAngle(PhysicsVehicleId vehicle, float angle);
    void motorcycleSetMaxLeanAngle(PhysicsVehicleId vehicle, float angle);
    void motorcycleSetLeanSpring(PhysicsVehicleId vehicle, float springConstant, float damping);
    void motorcycleSetLeanSpringIntegration(PhysicsVehicleId vehicle, float coefficient,
            float decay);
    void motorcycleSetLeanSmoothing(PhysicsVehicleId vehicle, float factor);
    void motorcycleSetLeanControllerEnabled(PhysicsVehicleId vehicle, bool enabled);
    float motorcycleGetLeanAngle(PhysicsVehicleId vehicle) const;

    void setWind(const Math::Vec3& wind);
    const Math::Vec3& getWind() const;
    PhysicsSoftBodyId softBodyCreate(const Math::Vec3* positions, std::size_t count,
            const std::uint32_t* indices, std::size_t indexCount,
            const PhysicsSoftBodyParameters& parameters);
    void softBodyFree(PhysicsSoftBodyId softBody);
    bool softBodyValid(PhysicsSoftBodyId softBody) const;
    std::size_t softBodyGetParticleCount(PhysicsSoftBodyId softBody) const;
    void softBodyGetPositions(PhysicsSoftBodyId softBody, Math::Vec3* out, std::size_t count) const;
    void softBodySetParticlePosition(PhysicsSoftBodyId softBody, std::size_t index,
            const Math::Vec3& position);
    void softBodySetPinned(PhysicsSoftBodyId softBody, std::size_t index, bool pinned);
    bool softBodyIsPinned(PhysicsSoftBodyId softBody, std::size_t index) const;
    void softBodyBuildAttachments(PhysicsSoftBodyId softBody, float maxDistanceMultiplier);
    void softBodyAddVelocity(PhysicsSoftBodyId softBody, const Math::Vec3& delta);
    void softBodySetWind(PhysicsSoftBodyId softBody, const Math::Vec3& wind);
    void softBodySetEnabled(PhysicsSoftBodyId softBody, bool enabled);
    float softBodyGetWorstStretch(PhysicsSoftBodyId softBody) const;

    PhysicsCharacterId characterCreate(const PhysicsCharacterParameters& parameters,
            const Math::Vec3& position);
    void characterFree(PhysicsCharacterId character);
    bool characterValid(PhysicsCharacterId character) const;
    void characterSetParameters(PhysicsCharacterId character,
            const PhysicsCharacterParameters& parameters);
    void characterSetPosition(PhysicsCharacterId character, const Math::Vec3& position);
    Math::Vec3 characterGetPosition(PhysicsCharacterId character) const;
    void characterSetVelocity(PhysicsCharacterId character, const Math::Vec3& velocity);
    Math::Vec3 characterGetVelocity(PhysicsCharacterId character) const;
    void characterSetIgnoredBody(PhysicsCharacterId character, PhysicsBodyId body);
    PhysicsCharacterState characterMoveAndSlide(PhysicsCharacterId character, float deltaTime);
    PhysicsCharacterState characterMove(PhysicsCharacterId character,
            const Math::Vec3& displacement);
    bool characterIsNearGround(PhysicsCharacterId character, float distance) const;
    std::size_t characterGetHitCount(PhysicsCharacterId character) const;
    PhysicsCharacterHit characterGetHit(PhysicsCharacterId character, std::size_t index) const;

    PhysicsCharacterRigidId characterRigidCreate(PhysicsBodyId body,
            const PhysicsCharacterRigidParameters& parameters);
    void characterRigidFree(PhysicsCharacterRigidId character);
    bool characterRigidValid(PhysicsCharacterRigidId character) const;
    void characterRigidSetParameters(PhysicsCharacterRigidId character,
            const PhysicsCharacterRigidParameters& parameters);
    PhysicsGroundInfo characterRigidProbe(PhysicsCharacterRigidId character);

    bool convexHullMesh(const Math::Vec3* points, std::size_t pointCount,
            ct::Vector<Math::Vec3>& vertices, ct::Vector<std::uint32_t>& indices) const;
    bool shatter(const Math::Vec3* points, std::size_t pointCount, const Math::Vec3* sites,
            std::size_t siteCount, ct::Vector<PhysicsShard>& shards) const;

    PhysicsRagdollId ragdollCreate(const int* boneParents, std::size_t boneCount,
            const PhysicsRagdollBones& bones, const Math::Mat4* restPose, float restScale,
            const Math::Mat4* globalPose, const Math::Mat4& ownerWorld,
            const PhysicsRagdollSettings& settings);
    void ragdollFree(PhysicsRagdollId ragdoll);
    bool ragdollValid(PhysicsRagdollId ragdoll) const;
    void ragdollFollow(PhysicsRagdollId ragdoll, const Math::Mat4* globalPose,
            std::size_t boneCount, const Math::Mat4& ownerWorld);
    bool ragdollActivate(PhysicsRagdollId ragdoll);
    void ragdollDeactivate(PhysicsRagdollId ragdoll);
    bool ragdollIsActive(PhysicsRagdollId ragdoll) const;
    void ragdollWritePose(PhysicsRagdollId ragdoll, PhysicsBonePose* localPose,
            std::size_t boneCount, const Math::Mat4& ownerWorld, float alpha) const;
    void ragdollSetCollisionMask(PhysicsRagdollId ragdoll, std::uint32_t mask);
    int ragdollGetPartBone(PhysicsRagdollId ragdoll, PhysicsRagdollPart part) const;
    float ragdollGetPartMass(PhysicsRagdollId ragdoll, PhysicsRagdollPart part) const;
    float ragdollGetFacing(PhysicsRagdollId ragdoll) const;
    Math::Mat4 ragdollGetPartTransform(PhysicsRagdollId ragdoll, PhysicsRagdollPart part) const;
    Math::Vec3 ragdollGetPartVelocity(PhysicsRagdollId ragdoll, PhysicsRagdollPart part) const;
    bool ragdollDetach(PhysicsRagdollId ragdoll, PhysicsRagdollPart part);
    bool ragdollIsDetached(PhysicsRagdollId ragdoll, PhysicsRagdollPart part) const;
    void ragdollApplyImpulse(PhysicsRagdollId ragdoll, PhysicsRagdollPart part,
            const Math::Vec3& impulse, const Math::Vec3& offset);

    bool intersectRay(const PhysicsRayParameters& parameters, PhysicsRayResult& result) const;
    void overlapSphere(const Math::Vec3& center, float radius, std::uint32_t collisionMask,
            ct::Vector<PhysicsBodyId>& out) const;
    void overlapBox(const Math::Box& box, std::uint32_t collisionMask,
            ct::Vector<PhysicsBodyId>& out) const;

    void debugDraw(BatchRenderer& batch, float interpolation = 1.0f);

    const PhysicsStats& getStats() const;
    std::uint32_t getContactCount() const;
    void resetStats();

    ct::Function<void(const PhysicsContactEvent&)> contactReported;

    struct Impl;

private:
    static PhysicsServer3D* sSingleton;

    Impl* mImpl;
};

} // namespace prisma3d
