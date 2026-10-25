#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

#include "dynamics/Aircraft.h"

#include "dynamics/RaycastVehicle.h"
#include "dynamics/RigidBody.h"

#include <math.h>

namespace kinetix::Physics
{

namespace
{
constexpr f32 kDegrees = 3.14159265358979f / 180.0f;

// Gear geometry. The mains sit just behind the centre of mass so the aircraft
// rests on its nose wheel and can still rotate about them at flying speed.
const Math::vec3 kNoseGear(0.0f, -0.55f, 1.6f);
const Math::vec3 kLeftMainGear(1.2f, -0.55f, -0.45f);
constexpr f32 kGearRestLength = 0.55f;
constexpr f32 kWheelRadius = 0.25f;
// Suspension stiffness per unit chassis mass, per wheel (RaycastVehicle's
// convention): three wheels at 30 compress about 0.11 m under 1 g.
constexpr f32 kGearStiffness = 30.0f;
} // namespace

LightAircraftLayout buildLightAircraft(Aircraft& aircraft, RigidBody& body)
{
    // Mass and inertia of a typical four-seat high-wing aircraft. Body axes:
    // X left (pitch axis), Y up (yaw axis), Z forward (roll axis).
    constexpr f32 kMass = 1000.0f;
    body.setBodyType(BodyType::Dynamic);
    body.setMass(kMass);
    body.setInertiaTensor(Math::mat3(Math::vec3(1800.0f, 0.0f, 0.0f), Math::vec3(0.0f, 2600.0f, 0.0f),
                                     Math::vec3(0.0f, 0.0f, 1300.0f)));
    // No artificial damping: the air provides all of it.
    body.setDamping(1.0f, 1.0f);
    body.setCanSleep(false);

    aircraft.setBody(&body);
    aircraft.setAxes(Math::vec3(0.0f, 0.0f, 1.0f), Math::vec3(0.0f, 1.0f, 0.0f));

    LightAircraftLayout layout;

    // Main wing: high, 1.5 degrees of dihedral, mild taper, 2.5 degrees of
    // washout so the root stalls first and the ailerons keep working; 11 m
    // span, 15.95 m^2, aspect ratio 7.6.
    {
        Wing::Description d;
        d.root = Math::vec3(0.0f, 0.9f, 0.15f);
        d.tip = Math::vec3(5.5f, 0.9f + 5.5f * tanf(1.5f * kDegrees), 0.15f);
        d.rootChord = 1.6f;
        d.tipChord = 1.3f;
        d.rootIncidence = 1.5f * kDegrees;
        d.tipIncidence = -1.0f * kDegrees;
        d.mirrored = true;
        d.elementCount = 8;
        d.oswaldEfficiency = 0.8f;
        Wing wing(d, Airfoil::cambered());

        ControlSurface flaps;
        flaps.spanStart = 0.0f;
        flaps.spanEnd = 0.45f;
        flaps.chordFraction = 0.25f;
        flaps.maxDeflection = 30.0f * kDegrees;
        flaps.minDeflection = 0.0f;
        flaps.mix.flaps = 1.0f;
        wing.addControlSurface(flaps);

        // Primary panel is the left wing (+X): aileron +1 (roll right) puts
        // its trailing edge down; the mirrored right one goes up.
        ControlSurface ailerons;
        ailerons.spanStart = 0.5f;
        ailerons.spanEnd = 0.95f;
        ailerons.chordFraction = 0.25f;
        ailerons.maxDeflection = 15.0f * kDegrees;
        ailerons.minDeflection = 20.0f * kDegrees;
        ailerons.mix.aileron = 1.0f;
        wing.addControlSurface(ailerons);
        layout.mainWing = aircraft.addWing(wing);
    }

    // Horizontal tail: 3.4 m span, 3.57 m^2, 4.55 m behind the wing. The
    // incidence carries the download balancing the wing's nose-down moment at cruise.
    {
        Wing::Description d;
        d.root = Math::vec3(0.0f, 0.1f, -4.4f);
        d.tip = Math::vec3(1.7f, 0.1f, -4.4f);
        d.rootChord = 1.05f;
        d.tipChord = 1.05f;
        d.rootIncidence = -2.0f * kDegrees;
        d.tipIncidence = -2.0f * kDegrees;
        d.mirrored = true;
        d.elementCount = 4;
        d.oswaldEfficiency = 0.8f;
        Wing wing(d, Airfoil::symmetric());

        // Elevator +1 (nose up) lifts its trailing edge: more travel up
        // than down, as on most light aircraft.
        ControlSurface elevator;
        elevator.chordFraction = 0.4f;
        elevator.maxDeflection = 15.0f * kDegrees;
        elevator.minDeflection = 25.0f * kDegrees;
        elevator.mix.elevator = -1.0f;
        wing.addControlSurface(elevator);
        layout.horizontalTail = aircraft.addWing(wing);
        aircraft.setDownwash(layout.horizontalTail, static_cast<s32>(layout.mainWing), 1.7f);
    }

    // Vertical tail: swept, 1.45 m tall, 1.6 m^2. Positive lift points left (+X),
    // so rudder +1 pushes the tail left and the nose right. Fuselage and
    // tailplane act as an end plate, roughly doubling its aspect ratio.
    {
        Wing::Description d;
        d.root = Math::vec3(0.0f, 0.3f, -4.5f);
        d.tip = Math::vec3(0.0f, 1.75f, -4.85f);
        d.rootChord = 1.4f;
        d.tipChord = 0.8f;
        d.up = Math::vec3(1.0f, 0.0f, 0.0f);
        d.elementCount = 4;
        d.oswaldEfficiency = 0.8f;
        const f32 height = 1.45f;
        const f32 area = 0.5f * (1.4f + 0.8f) * height;
        d.aspectRatio = 2.0f * height * height / area;
        Wing wing(d, Airfoil::symmetric());

        ControlSurface rudder;
        rudder.chordFraction = 0.4f;
        rudder.maxDeflection = 25.0f * kDegrees;
        rudder.minDeflection = 25.0f * kDegrees;
        rudder.mix.rudder = 1.0f;
        wing.addControlSurface(rudder);
        layout.verticalTail = aircraft.addWing(wing);
    }

    // 120 kW at 2700 rpm on a 1.9 m two-blade fixed-pitch propeller,
    // turning clockwise seen from the cockpit.
    {
        PistonEngine::Description e;
        e.maxPower = 120000.0f;
        e.ratedRpm = 2700.0f;
        e.maxRpm = 2800.0f;
        const u32 engine = aircraft.addEngine(PistonEngine(e));

        Propeller::Description p;
        p.radius = 0.95f;
        p.hubRadius = 0.12f;
        p.bladeCount = 2;
        p.rootChord = 0.17f;
        p.tipChord = 0.09f;
        p.pitch = 1.6f;
        p.position = Math::vec3(0.0f, 0.0f, 2.0f);
        p.axis = Math::vec3(0.0f, 0.0f, 1.0f);
        p.rotation = 1.0f;
        p.inertia = 2.2f;
        p.elementCount = 12;
        const u32 index = aircraft.addPropeller(Propeller(p, Airfoil::propeller()),
                                                static_cast<s32>(engine));
        // Idling, not stopped.
        aircraft.propeller(index).setAngularSpeed(700.0f * 2.0f * 3.14159265358979f / 60.0f);
    }

    // Fuselage, gear, struts and cooling drag: about 0.42 m^2 flat-plate area
    // head on, slab sides broadside, top seen from above.
    {
        DragElement fuselage;
        fuselage.position = Math::vec3(0.0f, 0.0f, -0.3f);
        fuselage.dragAreas = Math::vec3(2.5f, 3.5f, 0.42f);
        aircraft.addDragElement(fuselage);
    }

    layout.noseGear = kNoseGear;
    layout.leftMainGear = kLeftMainGear;
    layout.rightMainGear = Math::vec3(-kLeftMainGear.x, kLeftMainGear.y, kLeftMainGear.z);
    layout.gearRestLength = kGearRestLength;
    layout.wheelRadius = kWheelRadius;
    layout.fuselageHalfExtents = Math::vec3(0.55f, 0.6f, 3.0f);
    // Strut top below the origin, plus the strut and wheel, less the static
    // compression under the aircraft's weight.
    const f32 compression = 9.81f / (3.0f * kGearStiffness);
    layout.restHeight = -kNoseGear.y + kGearRestLength + kWheelRadius - compression;
    return layout;
}

void addLightAircraftGear(RaycastVehicle& gear, const LightAircraftLayout& layout)
{
    RaycastVehicle::Tuning tuning;
    tuning.suspensionStiffness = kGearStiffness;
    // About 0.7 of critical for three wheels at that stiffness.
    tuning.suspensionCompression = 6.0f;
    tuning.suspensionDamping = 7.0f;
    tuning.maxSuspensionTravelCm = 30.0f;
    // Rubber on asphalt sideways; the wheels roll freely forwards.
    tuning.frictionSlip = 0.8f;
    tuning.maxSuspensionForce = 40000.0f;

    const Math::vec3 down(0.0f, -1.0f, 0.0f);
    const Math::vec3 axle(-1.0f, 0.0f, 0.0f);
    gear.addWheel(layout.noseGear, down, axle, layout.gearRestLength, layout.wheelRadius, tuning,
                  true);
    gear.addWheel(layout.leftMainGear, down, axle, layout.gearRestLength, layout.wheelRadius,
                  tuning, false);
    gear.addWheel(layout.rightMainGear, down, axle, layout.gearRestLength, layout.wheelRadius,
                  tuning, false);
}

} // namespace kinetix::Physics
