#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

#include "marine/Boat.h"

#include "dynamics/RigidBody.h"
#include "marine/WaterSurface.h"

#include <math.h>

namespace kinetix::Physics
{

namespace
{
constexpr f32 kDegrees = 3.14159265358979f / 180.0f;

// Loaded displacement: hull and outboard about 750 kg, fuel, gear and three
// people the rest.
constexpr f32 kMass = 1250.0f;
// Centre of mass relative to the hull generator's frame (keel at midship at the origin,
// transom at z = -L/2): 0.5 m above the keel, 1 m aft of midship, with the outboard's 170 kg
// hanging off it; trims the boat level at rest (within a degree).
const Math::vec3 kCentreOfMass(0.0f, 0.5f, -1.0f);
constexpr f32 kIdleRpm = 700.0f;
// The screw propeller's thrust at full throttle, for the prism model: what
// this hull needs to plane against the prism drag.
constexpr f32 kScrewThrust = 15000.0f;
} // namespace

SmallMotorboatLayout buildSmallMotorboat(Boat& boat, RigidBody& body)
{
    SmallMotorboatLayout layout;
    layout.mass = kMass;

    BoatHullParameters& hp = layout.hullParameters;
    hp.length = 6.0f;
    hp.beam = 2.25f;
    hp.depth = 1.1f;
    hp.transomBeamFraction = 0.88f;
    hp.chineBeamFraction = 0.9f;
    hp.deadrise = 18.0f * kDegrees;
    hp.bowDeadrise = 40.0f * kDegrees;
    hp.bilgeRadius = 0.0f;
    hp.forefootRise = 0.55f;
    hp.forefootStart = 0.5f;
    hp.sheerRise = 0.15f;
    hp.stations = 20;
    hp.sectionPoints = 6;
    HullMesh hull = HullMesh::boat(hp);
    hull.translate(-kCentreOfMass);

    // Gyradii typical of a small planing boat: roll 0.38 B, pitch and yaw 0.26 L.
    const f32 roll = 0.38f * hp.beam;
    const f32 pitch = 0.26f * hp.length;
    body.setBodyType(BodyType::Dynamic);
    body.setMass(kMass);
    body.setInertiaTensor(Math::mat3(Math::vec3(kMass * pitch * pitch, 0.0f, 0.0f),
                                     Math::vec3(0.0f, kMass * pitch * pitch * 1.05f, 0.0f),
                                     Math::vec3(0.0f, 0.0f, kMass * roll * roll)));
    // The water and air provide all the damping.
    body.setDamping(1.0f, 1.0f);
    body.setCanSleep(false);

    boat.setBody(&body);
    boat.setAxes(Math::vec3(0.0f, 0.0f, 1.0f), Math::vec3(0.0f, 1.0f, 0.0f));
    boat.setHull(hull);
    boat.hull().setReferenceLength(0.9f * hp.length);

    // Outboard on the transom centreline; its leg turns as a whole (thrust line and lower unit).
    const f32 transom = -0.5f * hp.length - kCentreOfMass.z;
    const f32 keel = -kCentreOfMass.y;
    const Math::vec3 hub(0.0f, keel - 0.42f, transom - 0.35f);

    {
        // The lower unit and skeg as one vertical foil ahead of the
        // propeller, from the hull bottom down past the hub: positive lift
        // to port (+X), so a positive deflection pushes the stern to port
        // and the bow to starboard.
        Wing::Description d;
        d.root = Math::vec3(0.0f, keel - 0.02f, transom - 0.28f);
        d.tip = Math::vec3(0.0f, keel - 0.62f, transom - 0.28f);
        d.rootChord = 0.34f;
        d.tipChord = 0.26f;
        d.forward = Math::vec3(0.0f, 0.0f, 1.0f);
        d.up = Math::vec3(1.0f, 0.0f, 0.0f);
        d.elementCount = 6;
        d.oswaldEfficiency = 0.8f;
        // The anti-ventilation plate and the hull above act as an end plate.
        d.aspectRatio = 2.0f * 0.6f * 0.6f / (0.6f * 0.3f);
        Hydrofoil foil(d, Airfoil::symmetric());
        foil.setMaxDeflection(30.0f * kDegrees);
        layout.lowerUnit = boat.addHydrofoil(foil, 1.0f);
    }

    {
        PistonEngine::Description e;
        e.maxPower = 75000.0f;
        e.ratedRpm = 5500.0f;
        e.maxRpm = 6000.0f;
        // Low enough that the engine idles near 700 rpm out of gear, not at the aircraft engine's
        // fast idle.
        e.idleThrottle = 0.036f;
        layout.engine = boat.addEngine(PistonEngine(e));
    }

    {
        MarinePropeller::Description p;
        p.blades.radius = 0.178f;
        p.blades.hubRadius = 0.045f;
        p.blades.bladeCount = 3;
        p.blades.rootChord = 0.13f;
        p.blades.tipChord = 0.11f;
        p.blades.pitch = 0.43f;
        p.blades.position = hub;
        p.blades.axis = Math::vec3(0.0f, 0.0f, 1.0f);
        p.blades.rotation = 1.0f;
        p.blades.inertia = 0.02f;
        p.blades.elementCount = 10;
        p.gearRatio = 2.0f;
        p.gearEfficiency = 0.95f;
        p.reverseEfficiency = 0.6f;
        p.wakeFraction = 0.05f;
        p.thrustDeduction = 0.05f;
        p.engineInertia = 0.12f;
        p.steeringAxis = Math::vec3(0.0f, 1.0f, 0.0f);
        p.maxSteeringAngle = 30.0f * kDegrees;
        MarinePropeller propeller(p, Airfoil::propeller());
        propeller.setGear(MarinePropeller::Gear::Neutral);
        propeller.setEngineAngularSpeed(kIdleRpm * 2.0f * 3.14159265358979f / 60.0f);
        layout.propeller = boat.addPropeller(propeller, static_cast<s32>(layout.engine), 1.0f);
    }

    {
        DragElement windage;
        windage.position = Math::vec3(0.0f, 0.75f, 0.3f);
        windage.dragAreas = Math::vec3(4.5f, 6.0f, 1.4f);
        boat.addWindage(windage);
    }

    // For the prism model: the screw at the hub, its shaft point half a
    // metre ahead (the thrust runs towards it), its top point half a metre
    // above.
    {
        ScrewPropeller screw(hub, hub + Math::vec3(0.0f, 0.0f, 0.5f), hub + Math::vec3(0.0f, 0.5f, 0.0f),
                             kScrewThrust);
        screw.setMaxRudderDegrees(30.0f);
        layout.screwPropeller = boat.addScrewPropeller(screw);
    }

    const HullMesh& mesh = boat.hull().hull();
    layout.collisionHalfExtents = (mesh.boundsMax() - mesh.boundsMin()) * 0.5f;
    layout.collisionCentre = (mesh.boundsMax() + mesh.boundsMin()) * 0.5f;
    layout.restHeight = floatingHeight(mesh, kMass, WaterProperties::sea().density);
    return layout;
}

} // namespace kinetix::Physics
