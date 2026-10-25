#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

#include "rotorcraft/Helicopter.h"

#include "dynamics/RigidBody.h"

#include <math.h>

namespace kinetix::Physics
{

namespace
{
constexpr f32 kDegrees = 3.14159265358979f / 180.0f;

constexpr f32 kMass = 1300.0f;
constexpr f32 kRotorRadius = 4.6f;
constexpr f32 kTipSpeed = 200.0f;
constexpr f32 kTailRotorRadius = 0.8f;

constexpr f32 kSkidDepth = 1.05f;
constexpr f32 kSkidHalfTrack = 1.1f;
constexpr f32 kSkidFront = 1.3f;
constexpr f32 kSkidRear = -1.1f;
// Each takes a quarter of the weight with 4 cm of deflection; about 0.6 of critical damping on that quarter of the mass.
constexpr f32 kSkidStiffness = 70000.0f;
constexpr f32 kSkidDamping = 5500.0f;

// NACA 0012-like section: lift slope 5.73 per rad (2 pi less a little for thickness and Reynolds
// number), Cl_max 1.35, profile drag 0.0095 (a production blade's with erosion strip and
// roughness, not a wind-tunnel 0.008).
Airfoil rotorSection()
{
    Airfoil::Parameters p;
    p.liftSlope = 5.73f;
    p.maxLift = 1.35f;
    p.minLift = -1.35f;
    p.stallWidth = 0.14f;
    p.minDrag = 0.0095f;
    p.liftDragFactor = 0.008f;
    p.maxDrag = 2.0f;
    return Airfoil(p);
}
} // namespace

LightHelicopterLayout buildLightHelicopter(Helicopter& helicopter, RigidBody& body)
{
    // Five seats, turbine, empty about 800 kg. Pitch inertia is the largest (tail boom), roll the
    // smallest. Body axes: X left (pitch), Y up (yaw), Z forward (roll).
    body.setBodyType(BodyType::Dynamic);
    body.setMass(kMass);
    body.setInertiaTensor(Math::mat3(Math::vec3(2400.0f, 0.0f, 0.0f), Math::vec3(0.0f, 2100.0f, 0.0f),
                                     Math::vec3(0.0f, 0.0f, 800.0f)));
    // The air and the gear supply all the damping.
    body.setDamping(1.0f, 1.0f);
    body.setCanSleep(false);
    helicopter.setBody(&body);

    LightHelicopterLayout layout;

    // Main rotor: solidity 0.044 over the blade span, Lock number 5.5, flap frequency 1.03 /rev,
    // anticlockwise from above at 43.5 rad/s (415 rpm); shaft tilted 4 degrees forward so the
    // fuselage hangs closer to level in cruise.
    {
        Rotor::Description d;
        d.radius = kRotorRadius;
        d.rootCutout = 0.7f;
        d.bladeCount = 3;
        d.rootChord = 0.25f;
        d.tipChord = 0.25f;
        d.twist = -8.0f * kDegrees;
        d.hingeOffset = 0.18f;
        d.flapSpring = 0.0f;
        d.bladeMass = 22.0f;
        d.hubInertia = 15.0f;
        d.position = Math::vec3(0.0f, 1.5f, 0.05f);
        d.shaftAxis = Math::vec3(0.0f, cosf(4.0f * kDegrees), sinf(4.0f * kDegrees));
        d.forward = Math::vec3(0.0f, 0.0f, 1.0f);
        d.rotation = 1.0f;
        d.radialStations = 10;
        d.azimuthStations = 12;
        helicopter.mainRotor().setAirfoil(rotorSection());
        helicopter.mainRotor().setDescription(d);
    }

    // Tail rotor: two teetering blades on the left of the boom, pushing the tail right against the
    // main rotor's torque; the same 200 m/s tip speed (geared 5.75:1).
    {
        Rotor::Description d;
        d.radius = kTailRotorRadius;
        d.rootCutout = 0.15f;
        d.bladeCount = 2;
        d.rootChord = 0.14f;
        d.tipChord = 0.14f;
        d.twist = 0.0f;
        d.hingeOffset = 0.0f;
        d.flapSpring = 0.0f;
        d.bladeMass = 1.6f;
        d.hubInertia = 0.2f;
        d.position = Math::vec3(0.3f, 1.0f, -5.7f);
        d.shaftAxis = Math::vec3(-1.0f, 0.0f, 0.0f);
        d.forward = Math::vec3(0.0f, 0.0f, 1.0f);
        d.rotation = 1.0f;
        d.radialStations = 6;
        d.azimuthStations = 8;
        helicopter.tailRotor().setAirfoil(rotorSection());
        helicopter.tailRotor().setDescription(d);
        helicopter.setTailRotorEnabled(true);
    }

    // 300 kW turboshaft. The anticipator is centred on the hover collective so the governor's
    // integrator has little to do in hover.
    {
        TurboshaftEngine::Description e;
        e.maxPower = 300000.0f;
        e.ratedTurbineRpm = 6000.0f;
        e.anticipation = 1.2f;
        e.anticipationBias = 0.0f;
        e.governorGain = 8.0f;
        e.governorIntegral = 6.0f;
        helicopter.engine().setDescription(e);
    }

    {
        RotorTransmission::Description t;
        t.mainRotorInertia = helicopter.mainRotor().polarInertia();
        t.tailRotorInertia = helicopter.tailRotor().polarInertia();
        t.tailRotorRatio = (kTipSpeed / kTailRotorRadius) / (kTipSpeed / kRotorRadius);
        t.efficiency = 0.97f;
        t.frictionTorque = 20.0f;
        t.frictionPerSpeed = 1.0f;
        helicopter.transmission().setDescription(t);
        helicopter.setRatedRotorSpeed(kTipSpeed / kRotorRadius);
    }

    {
        HelicopterRigging r;
        r.collectiveMin = 0.0f;
        r.collectiveMax = 15.0f * kDegrees;
        r.longitudinalCyclic = 12.0f * kDegrees;
        r.lateralCyclic = 7.0f * kDegrees;
        r.tailCollective = 7.0f * kDegrees;
        r.tailCollectivePerPedal = -12.6f * kDegrees;
        helicopter.setRigging(r);
    }

    // Horizontal stabiliser: inside the rotor wake in hover, out of it in cruise.
    {
        Wing::Description d;
        d.root = Math::vec3(0.0f, 0.25f, -4.0f);
        d.tip = Math::vec3(1.0f, 0.25f, -4.0f);
        d.rootChord = 0.5f;
        d.tipChord = 0.5f;
        d.rootIncidence = -2.0f * kDegrees;
        d.tipIncidence = -2.0f * kDegrees;
        d.mirrored = true;
        d.elementCount = 3;
        d.oswaldEfficiency = 0.8f;
        layout.horizontalStabiliser = helicopter.addWing(Wing(d, Airfoil::symmetric()));
    }

    // Fin: toed so that in forward flight it pushes the tail right, taking over part of the tail
    // rotor's work. Lift is along +X (left), so negative incidence gives the force to the right.
    // The boom acts as an end plate.
    {
        Wing::Description d;
        d.root = Math::vec3(0.0f, 0.45f, -5.2f);
        d.tip = Math::vec3(0.0f, 1.45f, -5.5f);
        d.rootChord = 0.7f;
        d.tipChord = 0.45f;
        d.rootIncidence = -4.0f * kDegrees;
        d.tipIncidence = -4.0f * kDegrees;
        d.up = Math::vec3(1.0f, 0.0f, 0.0f);
        d.elementCount = 3;
        d.oswaldEfficiency = 0.8f;
        const f32 height = 1.0f;
        const f32 area = 0.5f * (0.7f + 0.45f) * height;
        d.aspectRatio = 2.0f * height * height / area;
        layout.fin = helicopter.addWing(Wing(d, Airfoil::symmetric()));
    }

    // Fuselage, skids, mast and hub: 1.05 m^2 of flat-plate area head on (0.9 to 1.2 for a light
    // turbine helicopter), the slab side broadside and the cabin roof from above, which the rotor
    // wake pushes down on in hover. The boom separately.
    {
        DragElement cabin;
        cabin.position = Math::vec3(0.0f, -0.1f, 0.0f);
        cabin.dragAreas = Math::vec3(4.0f, 3.2f, 1.0f);
        helicopter.addDragElement(cabin);
        DragElement boom;
        boom.position = Math::vec3(0.0f, 0.3f, -3.4f);
        boom.dragAreas = Math::vec3(1.2f, 1.0f, 0.05f);
        helicopter.addDragElement(boom);
    }

    const Math::vec3 skids[4] = {
        Math::vec3(kSkidHalfTrack, -kSkidDepth, kSkidFront),
        Math::vec3(kSkidHalfTrack, -kSkidDepth, kSkidRear),
        Math::vec3(-kSkidHalfTrack, -kSkidDepth, kSkidFront),
        Math::vec3(-kSkidHalfTrack, -kSkidDepth, kSkidRear),
    };
    for (u32 i = 0; i < 4; ++i)
    {
        SkidContact c;
        c.position = skids[i];
        c.stiffness = kSkidStiffness;
        c.damping = kSkidDamping;
        c.tangentialStiffness = 30000.0f;
        c.tangentialDamping = 2500.0f;
        c.friction = 0.6f;
        helicopter.addSkidContact(c);
        layout.skidPoints[i] = skids[i];
    }

    layout.hullHalfExtents = Math::vec3(0.7f, 0.8f, 1.6f);
    // The rear pair carries a little more (c.g. nearer it), but the mean compression is a quarter
    // of the weight over each contact's stiffness.
    layout.restHeight = kSkidDepth - kMass * 9.81f / (4.0f * kSkidStiffness);
    layout.hoverPedals = -0.18f;
    layout.hoverCollective = 0.59f;
    return layout;
}

} // namespace kinetix::Physics
