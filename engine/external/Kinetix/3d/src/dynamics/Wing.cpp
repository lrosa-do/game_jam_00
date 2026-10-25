#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>
#include <kinetix/core/Log.h>

#include "dynamics/Wing.h"

#include "dynamics/Atmosphere.h"
#include "dynamics/RigidBody.h"

#include <math.h>

namespace kinetix::Physics
{

namespace
{
constexpr f32 kPi = 3.14159265358979f;
constexpr u32 kMaxElements = 64;

// Beyond this the uniform-downwash picture means nothing (the wing is deep in
// stall anyway); the cap only stops a pathological evaluation from tilting
// forces by an absurd angle.
constexpr f32 kMaxInducedAngle = 0.25f;

// Below this squared speed a strip produces nothing (atan2 of the flow
// components stops being meaningful).
constexpr f32 kMinSpeedSquared = 1.0e-6f;

Math::vec3 reflect(const Math::vec3& v, const Math::vec3& normal)
{
    return v - normal * (2.0f * Math::dot(v, normal));
}

bool usable(const Math::vec3& v)
{
    return isfinite(v.x) && isfinite(v.y) && isfinite(v.z) && Math::dot(v, v) > 1.0e-12f;
}

// Any unit vector perpendicular to `axis`, for a degenerate description.
Math::vec3 anyPerpendicular(const Math::vec3& axis)
{
    const Math::vec3 trial =
        fabsf(axis.y) < 0.9f ? Math::vec3(0.0f, 1.0f, 0.0f) : Math::vec3(1.0f, 0.0f, 0.0f);
    return Math::normalize(trial - axis * Math::dot(trial, axis));
}
} // namespace

Wing::Wing()
{
    rebuild();
}

Wing::Wing(const Description& description, const Airfoil& airfoil)
    : mAirfoil(airfoil)
{
    setDescription(description);
}

void Wing::setDescription(const Description& description)
{
    mDescription = description;
    mDescription.elementCount = Clamp(description.elementCount, 1u, kMaxElements);
    mDescription.rootChord = Max(description.rootChord, 1.0e-3f);
    mDescription.tipChord = Max(description.tipChord, 1.0e-3f);
    mDescription.oswaldEfficiency = Clamp(description.oswaldEfficiency, 0.1f, 1.0f);
    mDescription.aspectRatio = Max(description.aspectRatio, 0.0f);
    if (!usable(mDescription.mirrorNormal))
        mDescription.mirrorNormal = Math::vec3(1.0f, 0.0f, 0.0f);
    mDescription.mirrorNormal = Math::normalize(mDescription.mirrorNormal);
    rebuild();
}

void Wing::rebuild()
{
    const Description& d = mDescription;
    mElements.clear();

    Math::vec3 spanVector = d.tip - d.root;
    if (!usable(spanVector))
    {
        Log::error("Wing: root and tip coincide - the wing has no span");
        spanVector = Math::vec3(1.0e-3f, 0.0f, 0.0f);
    }
    const f32 panelSpan = Math::length(spanVector);
    const Math::vec3 spanAxis = spanVector / panelSpan;

    // Chord and normal are made perpendicular to the span once: the caller's
    // values are hints ("roughly forward", "roughly up"), so dihedral and sweep
    // come out of the root and tip points.
    Math::vec3 forward = d.forward - spanAxis * Math::dot(d.forward, spanAxis);
    if (!usable(forward))
        forward = anyPerpendicular(spanAxis);
    forward = Math::normalize(forward);
    Math::vec3 up = d.up - spanAxis * Math::dot(d.up, spanAxis) - forward * Math::dot(d.up, forward);
    if (!usable(up))
    {
        up = Math::normalize(Math::cross(spanAxis, forward));
        if (Math::dot(up, d.up) < 0.0f)
            up = -up;
    }
    up = Math::normalize(up);

    const u32 count = d.elementCount;
    const u32 panels = d.mirrored ? 2u : 1u;
    mElements.reserve(count * panels);
    mArea = 0.0f;
    for (u32 panel = 0; panel < panels; ++panel)
    {
        const bool mirror = panel == 1;
        for (u32 i = 0; i < count; ++i)
        {
            const f32 eta = (static_cast<f32>(i) + 0.5f) / static_cast<f32>(count);
            const f32 incidence = Math::mix(d.rootIncidence, d.tipIncidence, eta);
            Math::vec3 chordAxis = forward * cosf(incidence) + up * sinf(incidence);
            Math::vec3 normal = up * cosf(incidence) - forward * sinf(incidence);
            Math::vec3 position = d.root + spanVector * eta;
            if (mirror)
            {
                position = reflect(position, d.mirrorNormal);
                chordAxis = reflect(chordAxis, d.mirrorNormal);
                normal = reflect(normal, d.mirrorNormal);
            }

            Element element;
            element.position = position;
            element.chordAxis = chordAxis;
            element.normal = normal;
            // Recomputed per panel, not reflected: a reflection flips handedness,
            // and the moment axis must stay chord x normal on both sides for
            // "positive Cm is nose up" to hold.
            element.pitchAxis = Math::cross(chordAxis, normal);
            // Mid-strip chord times strip width is exact for a linear taper.
            element.chord = Math::mix(d.rootChord, d.tipChord, eta);
            element.area = element.chord * panelSpan / static_cast<f32>(count);
            element.spanFraction = eta;
            element.mirroredPanel = mirror;
            mElements.push_back(element);
            mArea += element.area;
        }
    }

    // Tip to tip for a mirrored pair (projected, so dihedral shortens it), the
    // panel's own length otherwise.
    mSpan = d.mirrored ? Math::length(d.tip - reflect(d.tip, d.mirrorNormal)) : panelSpan;
    if (mSpan < 1.0e-3f)
        mSpan = 2.0f * panelSpan;
    mAspectRatio = d.aspectRatio > 0.0f ? d.aspectRatio : mSpan * mSpan / Max(mArea, 1.0e-6f);
    mInducedAngle = 0.0f;
    mState = WingState();
    assignControls();
}

u32 Wing::addControlSurface(const ControlSurface& surface)
{
    ControlSurface stored = surface;
    stored.spanStart = Clamp(surface.spanStart, 0.0f, 1.0f);
    stored.spanEnd = Clamp(surface.spanEnd, stored.spanStart, 1.0f);
    stored.chordFraction = Clamp(surface.chordFraction, 0.0f, 1.0f);
    stored.maxDeflection = Clamp(surface.maxDeflection, 0.0f, 1.2f);
    stored.minDeflection = Clamp(surface.minDeflection, 0.0f, 1.2f);
    mControls.push_back(stored);
    assignControls();
    return static_cast<u32>(mControls.size() - 1);
}

void Wing::setControlCommand(u32 index, f32 command, f32 mirroredCommand)
{
    if (index >= mControls.size())
    {
        Log::error("Wing: control surface %u does not exist (the wing has %u)", index,
                   controlSurfaceCount());
        return;
    }
    mControls[index].command = Clamp(command, -1.0f, 1.0f);
    mControls[index].mirroredCommand = Clamp(mirroredCommand, -1.0f, 1.0f);
}

void Wing::assignControls()
{
    const f32 width = 1.0f / static_cast<f32>(mDescription.elementCount);
    for (Element& element : mElements)
    {
        element.control = -1;
        element.controlCoverage = 0.0f;
        const f32 start = element.spanFraction - 0.5f * width;
        const f32 end = element.spanFraction + 0.5f * width;
        for (u32 c = 0; c < mControls.size(); ++c)
        {
            const ControlSurface& surface = mControls[c];
            const f32 overlap = Min(end, surface.spanEnd) - Max(start, surface.spanStart);
            const f32 coverage = overlap / width;
            if (coverage > element.controlCoverage + 1.0e-6f)
            {
                element.control = static_cast<s32>(c);
                element.controlCoverage = Min(coverage, 1.0f);
            }
        }
    }
}

f32 Wing::finiteLiftSlope() const
{
    const f32 a0 = mAirfoil.parameters().liftSlope;
    const f32 k = kPi * mDescription.oswaldEfficiency * mAspectRatio;
    return a0 / (1.0f + a0 / k);
}

namespace
{
Math::vec3 atmosphereWind(const Math::vec3& worldPoint, const void* userData)
{
    return static_cast<const Atmosphere*>(userData)->windAt(worldPoint);
}
} // namespace

const WingState& Wing::computeForces(const RigidBody& body, const Atmosphere& atmosphere)
{
    return computeForces(body, atmosphere.sampleAt(body.position()).density, &atmosphereWind,
                         &atmosphere, nullptr);
}

const WingState& Wing::computeForces(const RigidBody& body, f32 density, FlowField flowField,
                                     const void* userData, const f32* elementScale)
{
    const f32 inducedFactor = kPi * mDescription.oswaldEfficiency * mAspectRatio;
    const f32 liftSlope = mAirfoil.parameters().liftSlope;
    const Math::vec3 inducedFlow = body.directionToWorld(mInducedFlow);

    // Per-strip flow, computed once and reused by every pass of the downwash
    // solve (only the induced angle changes).
    struct Flow
    {
        Math::vec3 position;
        Math::vec3 flowDirection;
        Math::vec3 liftDirection;
        Math::vec3 pitchAxis;
        f32 alpha;
        f32 qArea;
        f32 deflection;
        f32 chordFraction;
        bool active;
    };
    Flow flows[kMaxElements * 2];

    const u32 count = static_cast<u32>(mElements.size());
    for (u32 i = 0; i < count; ++i)
    {
        const Element& element = mElements[i];
        Flow& flow = flows[i];
        flow.active = false;

        flow.position = body.pointToWorld(element.position);
        const Math::vec3 chordAxis = body.directionToWorld(element.chordAxis);
        const Math::vec3 normal = body.directionToWorld(element.normal);
        flow.pitchAxis = body.directionToWorld(element.pitchAxis);

        // The air as the strip sees it: wind less the strip's own motion through
        // it. The wrong sign (v + wind) is invisible in still air and doubles the
        // airspeed in a headwind.
        const f32 scale = elementScale ? elementScale[i] : 1.0f;
        if (!(scale > 0.0f) || !isfinite(scale))
            continue;
        const Math::vec3 fluid = flowField ? flowField(flow.position, userData) : Math::vec3(0.0f);
        const Math::vec3 air = fluid + inducedFlow - body.velocityAtPoint(flow.position);
        // Only the flow in the chordwise plane makes lift; pitchAxis is the span
        // direction.
        const Math::vec3 planar = air - flow.pitchAxis * Math::dot(air, flow.pitchAxis);
        const f32 speedSquared = Math::dot(planar, planar);
        flow.deflection = 0.0f;
        flow.chordFraction = 0.0f;
        if (!(speedSquared > kMinSpeedSquared) || !isfinite(speedSquared))
            continue;

        flow.active = true;
        flow.flowDirection = planar / sqrtf(speedSquared);
        // Positive when the strip meets the air on its underside.
        flow.alpha = atan2f(Math::dot(planar, normal), -Math::dot(planar, chordAxis));
        // Perpendicular to the flow in the chordwise plane; the normal at zero
        // angle of attack.
        flow.liftDirection = Math::cross(flow.flowDirection, flow.pitchAxis);
        flow.qArea = 0.5f * density * speedSquared * element.area * scale;

        if (element.control >= 0)
        {
            const ControlSurface& surface = mControls[static_cast<u32>(element.control)];
            const f32 command = element.mirroredPanel ? surface.mirroredCommand : surface.command;
            // A strip only partly under the surface gets that share of the
            // deflection (linear in the lift it adds).
            flow.deflection = surface.deflectionFor(command) * element.controlCoverage;
            flow.chordFraction = surface.chordFraction;
        }
    }

    // Uniform downwash: solve alpha_i = CL(alpha - alpha_i) / (pi e AR). Plain
    // fixed-point iteration diverges once a0 > pi e AR (any fin or low-aspect
    // tail), so Newton with the attached lift slope as derivative: exact in one
    // pass while attached, still a contraction once stall flattens the slope.
    f32 induced = mInducedAngle;
    f32 liftCoefficient = 0.0f;
    const f32 newtonScale = 1.0f / (1.0f + liftSlope / inducedFactor);
    for (u32 iteration = 0; iteration < 12; ++iteration)
    {
        f32 weightedLift = 0.0f;
        f32 weight = 0.0f;
        for (u32 i = 0; i < count; ++i)
        {
            const Flow& flow = flows[i];
            if (!flow.active)
                continue;
            const AirfoilCoefficients c =
                mAirfoil.evaluate(flow.alpha - induced, flow.deflection, flow.chordFraction);
            weightedLift += c.lift * flow.qArea;
            weight += flow.qArea;
        }
        if (weight <= 0.0f)
        {
            induced = 0.0f;
            liftCoefficient = 0.0f;
            break;
        }
        liftCoefficient = weightedLift / weight;
        const f32 step = (liftCoefficient / inducedFactor - induced) * newtonScale;
        induced = Clamp(induced + step, -kMaxInducedAngle, kMaxInducedAngle);
        if (fabsf(step) < 1.0e-6f)
            break;
    }
    if (!isfinite(induced))
        induced = 0.0f;
    mInducedAngle = induced;

    const f32 cosInduced = cosf(induced);
    const f32 sinInduced = sinf(induced);
    WingState state;
    state.liftCoefficient = liftCoefficient;
    state.inducedAngle = induced;
    f32 stalledArea = 0.0f;
    f32 qAreaSum = 0.0f;
    for (u32 i = 0; i < count; ++i)
    {
        Element& element = mElements[i];
        const Flow& flow = flows[i];
        if (!flow.active)
        {
            element.angleOfAttack = 0.0f;
            element.dynamicPressure = 0.0f;
            element.lift = 0.0f;
            element.drag = 0.0f;
            element.separation = 0.0f;
            continue;
        }
        const f32 alpha = flow.alpha - induced;
        const AirfoilCoefficients c = mAirfoil.evaluate(alpha, flow.deflection, flow.chordFraction);

        // Lift and drag are across and along the EFFECTIVE flow, turned down by
        // alpha_i, so in the free stream's frame the lift leans back; that lean is
        // the induced drag.
        const f32 lift = flow.qArea * (c.lift * cosInduced - c.drag * sinInduced);
        const f32 drag = flow.qArea * (c.drag * cosInduced + c.lift * sinInduced);
        const Math::vec3 force = flow.liftDirection * lift + flow.flowDirection * drag;
        const Math::vec3 moment = flow.pitchAxis * (flow.qArea * element.chord * c.moment);

        state.force += force;
        state.torque += Math::cross(flow.position - body.position(), force) + moment;
        state.lift += lift;
        state.drag += drag;
        qAreaSum += flow.qArea;
        if (c.separation > 0.5f)
            stalledArea += element.area;

        element.angleOfAttack = alpha;
        element.dynamicPressure = flow.qArea / element.area;
        element.lift = lift;
        element.drag = drag;
        element.separation = c.separation;
    }
    state.stalledFraction = mArea > 0.0f ? stalledArea / mArea : 0.0f;
    state.dynamicPressure = mArea > 0.0f ? qAreaSum / mArea : 0.0f;
    mState = state;
    return mState;
}

const WingState& Wing::apply(RigidBody& body, f32 density, FlowField flow, const void* userData,
                             const f32* elementScale)
{
    computeForces(body, density, flow, userData, elementScale);
    body.addForce(mState.force);
    body.addTorque(mState.torque);
    return mState;
}

const WingState& Wing::apply(RigidBody& body, const Atmosphere& atmosphere)
{
    computeForces(body, atmosphere);
    // The summed force and torque about the centre of mass equal the per-strip
    // forces at their own points, in two calls.
    body.addForce(mState.force);
    body.addTorque(mState.torque);
    return mState;
}

} // namespace kinetix::Physics
