#ifndef KINETIX_PHYSICS_MARINE_BOAT_H
#define KINETIX_PHYSICS_MARINE_BOAT_H

#include "dynamics/Atmosphere.h"
#include "dynamics/DragElement.h"
#include "dynamics/PistonEngine.h"
#include "marine/BuoyancyBody.h"
#include "marine/HullMesh.h"
#include "marine/Hydrofoil.h"
#include "marine/MarinePropeller.h"
#include "marine/PrismBuoyancy.h"
#include "marine/ScrewPropeller.h"
#include "marine/WaterEffects.h"

#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

#include <ct/vector.hpp>

namespace kinetix::Physics
{

class RigidBody;
class WaterSurface;

// Throttle -1..1: positive ahead, negative astern, neutral inside +-throttleDeadband.
// Rudder -1..1: +1 turns the bow to starboard (right, body -X), heading increasing as for
// Aircraft.
struct BoatControls
{
    f32 throttle = 0.0f;
    f32 rudder = 0.0f;
};

// Measured on the last applyForces() or computeForces(). Radians, m/s.
struct BoatTelemetry
{
    f32 speedThroughWater = 0.0f;
    f32 forwardSpeed = 0.0f;
    // Positive moving to starboard.
    f32 lateralSpeed = 0.0f;
    f32 groundSpeed = 0.0f;
    // As Aircraft: heading turning right from world +Z; heel positive starboard side down;
    // trim positive bow up.
    f32 heading = 0.0f;
    f32 heel = 0.0f;
    f32 trim = 0.0f;
    // Positive turning to starboard.
    f32 yawRate = 0.0f;
    f32 draft = 0.0f;
    f32 submergedVolume = 0.0f;
    f32 waterplaneArea = 0.0f;
    Math::vec3 centreOfBuoyancy{0.0f};
    Math::vec3 hydrostaticForce{0.0f};
    f32 engineRpm = 0.0f;
    f32 propellerRpm = 0.0f;
    f32 thrust = 0.0f;  // all propellers, along their axes
    f32 immersion = 0.0f; // of the first propeller's disc
    // Every load on the body together: world force, torque about the centre of mass.
    Math::vec3 force{0.0f};
    Math::vec3 torque{0.0f};
};

// Rigid body that floats: hull buoyancy and hydrodynamics, hydrofoils, engines driving
// propellers, windage. Like Aircraft it does not own the body and adds no collision shape;
// the hull mesh is only for the water.
//
// Timing as Aircraft: call applyForces(step) just before each PhysicsWorld::step(), or
// register stepCallback with setStepCallback() and call applyForces() once by hand before
// the first update(). The water's clock is the caller's (WaterSurface::advance): with
// step() driven by hand, advance the water after the world step; from a step callback,
// advance it before calling applyForces().
//
// Body axes default to +Z forward, +Y up, starboard on -X (setAxes() changes them);
// hydrofoils, propellers and drag elements use the same body space.
// Which hydrodynamics the boat runs on: the module's own (BuoyancyBody,
// Hydrofoil, MarinePropeller behind a PistonEngine, windage) or the prism
// model (PrismBuoyancy over the hull and ScrewPropeller thrust).
enum class BoatHydrodynamics : u8
{
    Kinetix,
    Prism
};

class Boat
{
public:
    Boat();

    void setBody(RigidBody* body);
    RigidBody* body() const
    {
        return mBody;
    }
    void setWater(const WaterSurface* water);
    const WaterSurface* water() const
    {
        return mWater;
    }
    // The air for the windage: a shared one, or the boat's own (null).
    void setAtmosphere(const Atmosphere* atmosphere)
    {
        mSharedAtmosphere = atmosphere;
    }
    const Atmosphere& atmosphere() const
    {
        return mSharedAtmosphere ? *mSharedAtmosphere : mAtmosphere;
    }
    Atmosphere& ownAtmosphere()
    {
        return mAtmosphere;
    }

    void setAxes(const Math::vec3& forward, const Math::vec3& up);
    const Math::vec3& forwardAxis() const
    {
        return mForward;
    }
    const Math::vec3& upAxis() const
    {
        return mUp;
    }
    Math::vec3 rightAxis() const
    {
        return Math::cross(mForward, mUp);
    }

    void setHull(const HullMesh& hull);
    BuoyancyBody& hull()
    {
        return mHull;
    }
    const BuoyancyBody& hull() const
    {
        return mHull;
    }

    // `rudderMix` scales the helm's rudder onto this foil's deflection
    // (deflection = rudder * mix * maxDeflection); 0 for a fixed keel.
    // The prism model. The hull given to setHull() also feeds prismHull();
    // the screw propellers take the helm's throttle and rudder. They act
    // only in the Prism model, where the engines, propellers and windage do
    // not; the foils act in both.
    void setHydrodynamics(BoatHydrodynamics model);
    BoatHydrodynamics hydrodynamics() const
    {
        return mHydrodynamics;
    }
    PrismBuoyancy& prismHull()
    {
        return mPrismHull;
    }
    const PrismBuoyancy& prismHull() const
    {
        return mPrismHull;
    }
    u32 addScrewPropeller(const ScrewPropeller& propeller);
    u32 screwPropellerCount() const
    {
        return static_cast<u32>(mScrewPropellers.size());
    }
    ScrewPropeller& screwPropeller(u32 index)
    {
        return mScrewPropellers[index];
    }
    const ScrewPropeller& screwPropeller(u32 index) const
    {
        return mScrewPropellers[index];
    }
    // Where the prism model sends its splash, ripple and spray, and whether
    // the next applyForces() feeds them - once a frame, not every step.
    void setEffects(WaterEffects* effects);
    void setEmitEffects(bool emit)
    {
        mEmitEffects = emit;
    }

    u32 addHydrofoil(const Hydrofoil& foil, f32 rudderMix = 0.0f);
    u32 addEngine(const PistonEngine& engine);
    // `engine` from addEngine(), or -1 for a propeller held at a set rpm; `steeringMix`
    // scales the rudder onto its steering angle.
    u32 addPropeller(const MarinePropeller& propeller, s32 engine = -1, f32 steeringMix = 0.0f);
    // Drag in the AIR for the part above water (per-axis drag areas).
    u32 addWindage(const DragElement& element);

    u32 hydrofoilCount() const
    {
        return static_cast<u32>(mFoils.size());
    }
    Hydrofoil& hydrofoil(u32 index)
    {
        return mFoils[index].foil;
    }
    const Hydrofoil& hydrofoil(u32 index) const
    {
        return mFoils[index].foil;
    }
    u32 engineCount() const
    {
        return static_cast<u32>(mEngines.size());
    }
    PistonEngine& engine(u32 index)
    {
        return mEngines[index];
    }
    u32 propellerCount() const
    {
        return static_cast<u32>(mPropellers.size());
    }
    MarinePropeller& propeller(u32 index)
    {
        return mPropellers[index].propeller;
    }
    const MarinePropeller& propeller(u32 index) const
    {
        return mPropellers[index].propeller;
    }
    u32 windageCount() const
    {
        return static_cast<u32>(mWindage.size());
    }
    DragElement& windage(u32 index)
    {
        return mWindage[index];
    }

    void setControls(const BoatControls& controls);
    const BoatControls& controls() const
    {
        return mControls;
    }
    void setThrottleDeadband(f32 deadband);
    f32 throttleDeadband() const
    {
        return mDeadband;
    }

    // Evaluates hull, foils, propellers and windage at the body's state, adds the result to
    // the body and advances the engines.
    void applyForces(f32 step);
    // The same without applying or advancing anything. Updates the telemetry.
    void computeForces(Math::vec3& force, Math::vec3& torque);

    static void stepCallback(f32 step, void* userData);

    const BoatTelemetry& telemetry() const
    {
        return mTelemetry;
    }

private:
    struct FoilEntry
    {
        Hydrofoil foil;
        f32 rudderMix = 0.0f;
    };
    struct PropellerEntry
    {
        MarinePropeller propeller;
        s32 engine = -1;
        f32 steeringMix = 0.0f;
    };

    void updateHelm();
    void accumulate(bool apply, f32 step, Math::vec3& force, Math::vec3& torque);
    void updateTelemetry(const Math::vec3& force, const Math::vec3& torque);

    RigidBody* mBody = nullptr;
    const WaterSurface* mWater = nullptr;
    Atmosphere mAtmosphere;
    const Atmosphere* mSharedAtmosphere = nullptr;
    Math::vec3 mForward{0.0f, 0.0f, 1.0f};
    Math::vec3 mUp{0.0f, 1.0f, 0.0f};

    BuoyancyBody mHull;
    ct::Vector<FoilEntry> mFoils;
    ct::Vector<PistonEngine> mEngines;
    ct::Vector<PropellerEntry> mPropellers;
    ct::Vector<DragElement> mWindage;

    BoatControls mControls;
    f32 mDeadband = 0.05f;
    f32 mThrust = 0.0f;
    f32 mFirstImmersion = 0.0f;
    void accumulatePrism(bool apply, Math::vec3& force, Math::vec3& torque);

    BoatHydrodynamics mHydrodynamics = BoatHydrodynamics::Kinetix;
    PrismBuoyancy mPrismHull;
    ct::Vector<ScrewPropeller> mScrewPropellers;
    WaterEffects* mEffects = nullptr;
    bool mEmitEffects = false;
    BoatTelemetry mTelemetry;
};

// Height of a hull's body origin above flat water at which it floats level (or at
// `orientation`) carrying `mass` in water of `density`, by bisection on the exact
// submerged volume. A mass the hull cannot carry returns the height of just fully submerged.
f32 floatingHeight(const HullMesh& hull, f32 mass, f32 density,
                   const Math::quat& orientation = Math::quat());

struct SmallMotorboatLayout
{
    BoatHullParameters hullParameters;
    // The hull mesh's bounding box and its centre. The mesh is not centred on the centre of
    // mass, so a BoxShape of these half extents sits collisionCentre off the hull; a
    // ConvexHullShape of the mesh vertices needs no offset.
    Math::vec3 collisionHalfExtents{0.5f};
    Math::vec3 collisionCentre{0.0f};
    f32 restHeight = 0.0f;
    f32 mass = 0.0f;
    u32 propeller = 0;
    u32 engine = 0;
    u32 lowerUnit = 0;
    u32 screwPropeller = 0;
};

// 6 m open motorboat, 1250 kg loaded, hard-chine planing hull, 75 kW outboard steered by
// turning the whole leg (30 degrees each way). Sets the body's mass, inertia, damping and
// sleep. The body still needs a position (restHeight above the water) and a collision
// shape (the layout's box).
SmallMotorboatLayout buildSmallMotorboat(Boat& boat, RigidBody& body);

} // namespace kinetix::Physics

#endif // KINETIX_PHYSICS_MARINE_BOAT_H
