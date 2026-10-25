#ifndef KINETIX_PHYSICS_GROUND_TRAILERCOUPLING_H
#define KINETIX_PHYSICS_GROUND_TRAILERCOUPLING_H

#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

namespace kinetix::Physics
{

class ConeTwistJoint;
class GroundVehicle;
class Joint;
class PhysicsWorld;
class PointJoint;
class RigidBody;

enum class CouplingType : u8
{
    // Tow ball or pintle hook: a point joint, free to turn every way.
    Ball,
    // Kingpin in a fifth wheel: a cone-twist joint (twist axis the common vertical), yaw up to
    // maxArticulation, pitch up to maxPitch, nearly locked in roll (maxRoll).
    FifthWheel
};

struct CouplingParameters
{
    CouplingType type = CouplingType::Ball;
    // The coupling point on each body, body space (+Z forward, +Y up).
    Math::vec3 towPoint{0.0f};
    Math::vec3 trailerPoint{0.0f};
    f32 maxArticulation = 1.4f; // rad
    f32 maxPitch = 0.26f;       // rad
    f32 maxRoll = 0.04f;        // rad
    bool brakeLink = true;
};

// Couples a trailer (a GroundVehicle without a drivetrain) to a tow vehicle with a world
// joint, makes the two bodies ignore each other's shapes, and connects the brake line. The
// joint belongs to the coupling; detach() or the destructor removes it.
class TrailerCoupling
{
public:
    TrailerCoupling() = default;
    ~TrailerCoupling();
    TrailerCoupling(const TrailerCoupling&) = delete;
    TrailerCoupling& operator=(const TrailerCoupling&) = delete;

    // Moves the trailer body (orientation copied from the tow) so its coupling point sits on
    // the tow's. Call before attach().
    static void placeTrailer(const RigidBody& tow, RigidBody& trailer, const CouplingParameters& parameters);

    // Both vehicles need body and body id set. The joint holds the points where they are now,
    // so place the trailer first. False (error logged) if something is missing.
    bool attach(PhysicsWorld& world, GroundVehicle& tow, GroundVehicle& trailer,
                const CouplingParameters& parameters);
    void detach();
    bool attached() const
    {
        return mJoint != nullptr;
    }
    const CouplingParameters& parameters() const
    {
        return mParameters;
    }
    Joint* joint() const
    {
        return mJoint;
    }

    // World distance between the coupling points, m.
    f32 separation() const;
    // Trailer heading minus tow heading about the tow's vertical, wrapped to +-pi; positive
    // when the trailer points to the tow's right.
    f32 articulationAngle() const;
    Math::vec3 towPointWorld() const;
    Math::vec3 trailerPointWorld() const;

private:
    PhysicsWorld* mWorld = nullptr;
    GroundVehicle* mTow = nullptr;
    GroundVehicle* mTrailer = nullptr;
    Joint* mJoint = nullptr;
    PointJoint* mBall = nullptr;
    ConeTwistJoint* mFifthWheel = nullptr;
    CouplingParameters mParameters;
};

} // namespace kinetix::Physics

#endif // KINETIX_PHYSICS_GROUND_TRAILERCOUPLING_H
