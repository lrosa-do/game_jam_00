#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>
#include <kinetix/core/Log.h>

#include "rotorcraft/Rotor.h"

#include <math.h>

namespace kinetix::Physics
{

namespace
{
constexpr f32 kPi = 3.14159265358979f;

// The loss factor is floored as the propeller's is: the momentum balance divides by it and it reaches zero only at the very tip.
constexpr f32 kMinLossFactor = 0.05f;

// Empirical turbulent-wake mass flow term, (kWakeMassFlow v)^2. Above 1 / sqrt(8) the annulus
// balance is monotonic in v for every descent rate (unique root); 0.37 puts the ideal
// autorotation at 1.65 v_h.
constexpr f32 kWakeMassFlow = 0.37f;

// Flap stops: the quasi-steady solution is clamped here so a rotor driven far outside its envelope stays a rotor.
constexpr f32 kMaxFlap = 0.35f;

// Below this rotor speed the blades rest on their stops: no centrifugal stiffening, no flapping solved.
constexpr f32 kMinFlappingSpeed = 1.0f;

constexpr u32 kAdvanceSweeps = 2;
constexpr u32 kMaxSolveSweeps = 200;

bool finiteVec(const Math::vec3& v)
{
    return isfinite(v.x) && isfinite(v.y) && isfinite(v.z);
}

f32 smoothstep01(f32 x)
{
    const f32 t = Clamp(x, 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}
} // namespace

// Everything about one operating point that is constant across elements, recomputed every sweep
// (ground and skew factors follow the inflow).
struct Rotor::Context
{
    f32 omega = 0.0f;    // rotor speed relative to the airframe
    f32 omegaAir = 0.0f; // blade speed relative to the air, yaw rate included
    Math::vec3 velocity{0.0f};
    Math::vec3 rate{0.0f};
    f32 climb = 0.0f;
    f32 inPlane = 0.0f;
    Math::vec3 downstream{0.0f}; // in-plane unit vector the wake is blown towards
    f32 density = 1.225f;
    f32 groundFactor = 1.0f; // on the induced velocity the blades see
    f32 groundRatio = 0.0f;  // x of Cheeseman-Bennett
    f32 skew = 0.0f;         // Pitt-Peters kx
    f32 skewAngle = 0.0f;
    f32 wakeWeight = 0.0f;   // turbulent-wake mass flow blend, 0..1
    f32 hoverInduced = 0.0f; // sqrt(T / (2 rho A)) at the thrust estimate
    f32 meanInduced = 0.0f;  // area mean of v(r), free air
    f32 cyclicCos = 0.0f;    // A1 after the phase and direction mapping
    f32 cyclicSin = 0.0f;    // B1
};

struct Rotor::Sweep
{
    f32 annulusThrust[MaxRadialStations];
    f32 annulusSlope[MaxRadialStations];
    f32 flapMoment[MaxAzimuthStations];
    Math::vec3 force{0.0f};
    f32 torque = 0.0f;
    f32 thrust = 0.0f;
    f32 stalledLoad = 0.0f;
    f32 totalLoad = 0.0f;
};

Rotor::Rotor()
{
    Airfoil::Parameters p;
    p.liftSlope = 5.73f;
    p.maxLift = 1.35f;
    p.minLift = -1.35f;
    p.minDrag = 0.008f;
    p.liftDragFactor = 0.008f;
    mAirfoil = Airfoil(p);
    setDescription(Description());
}

Rotor::Rotor(const Description& description, const Airfoil& airfoil)
    : mAirfoil(airfoil)
{
    setDescription(description);
}

void Rotor::setDescription(const Description& description)
{
    mDescription = description;
    Description& d = mDescription;
    d.radius = isfinite(description.radius) ? Max(description.radius, 0.05f) : 5.0f;
    d.hingeOffset = isfinite(description.hingeOffset)
                        ? Clamp(description.hingeOffset, 0.0f, 0.3f * d.radius)
                        : 0.0f;
    d.rootCutout = isfinite(description.rootCutout)
                       ? Clamp(description.rootCutout, d.hingeOffset, 0.8f * d.radius)
                       : 0.15f * d.radius;
    d.bladeCount = Clamp(description.bladeCount, 1u, 12u);
    d.rootChord = isfinite(description.rootChord) ? Max(description.rootChord, 1.0e-3f) : 0.2f;
    d.tipChord = isfinite(description.tipChord) ? Max(description.tipChord, 1.0e-3f) : 0.2f;
    d.twist = isfinite(description.twist) ? Clamp(description.twist, -0.7f, 0.7f) : 0.0f;
    d.flapSpring = isfinite(description.flapSpring) ? Max(description.flapSpring, 0.0f) : 0.0f;
    d.bladeMass = isfinite(description.bladeMass) ? Max(description.bladeMass, 0.01f) : 1.0f;
    d.hubInertia = isfinite(description.hubInertia) ? Max(description.hubInertia, 0.0f) : 0.0f;
    d.rotation = description.rotation < 0.0f ? -1.0f : 1.0f;
    d.cyclicPhase = isfinite(description.cyclicPhase) ? description.cyclicPhase : 0.0f;
    d.radialStations = Clamp(description.radialStations, 2u, MaxRadialStations);
    d.azimuthStations = Clamp(description.azimuthStations, 4u, MaxAzimuthStations);
    if (!finiteVec(d.position))
        d.position = Math::vec3(0.0f);

    if (!finiteVec(d.shaftAxis) || !(Math::dot(d.shaftAxis, d.shaftAxis) > 1.0e-12f))
    {
        Log::error("Rotor: shaft axis must be a finite, non-zero direction");
        d.shaftAxis = Math::vec3(0.0f, 1.0f, 0.0f);
    }
    d.shaftAxis = Math::normalize(d.shaftAxis);
    Math::vec3 aft = finiteVec(d.forward) ? -d.forward : Math::vec3(0.0f);
    aft -= d.shaftAxis * Math::dot(aft, d.shaftAxis);
    if (!(Math::dot(aft, aft) > 1.0e-8f))
    {
        Log::error("Rotor: forward must not be parallel to the shaft");
        aft = fabsf(d.shaftAxis.y) < 0.9f ? Math::cross(d.shaftAxis, Math::vec3(0.0f, 1.0f, 0.0f))
                                         : Math::cross(d.shaftAxis, Math::vec3(1.0f, 0.0f, 0.0f));
    }
    mShaft = d.shaftAxis;
    mAft = Math::normalize(aft);
    d.forward = -mAft;
    // psi = 90 deg is where a blade turning `rotation` about the shaft goes from aft: the advancing side.
    mAdvancing = Math::cross(mShaft, mAft) * d.rotation;
    rebuild();
    reset();
}

void Rotor::rebuild()
{
    const Description& d = mDescription;
    const f32 radius = d.radius;
    const f32 e = d.hingeOffset;

    mRadialCount = d.radialStations;
    const f32 width = (radius - d.rootCutout) / static_cast<f32>(mRadialCount);
    mFlapDampingIntegral = 0.0f;
    for (u32 j = 0; j < mRadialCount; ++j)
    {
        RadialStation& s = mRadial[j];
        s.radius = d.rootCutout + (static_cast<f32>(j) + 0.5f) * width;
        s.width = width;
        const f32 t = (s.radius - d.rootCutout) / Max(radius - d.rootCutout, 1.0e-6f);
        s.chord = Math::mix(d.rootChord, d.tipChord, t);
        // Linear twist from root to tip, zero at 0.75 R where the collective is defined.
        const f32 span = Max(radius - d.rootCutout, 1.0e-6f);
        s.twist = d.twist * (s.radius - 0.75f * radius) / span;
        const f32 arm = s.radius - e;
        mFlapDampingIntegral += s.chord * s.radius * arm * arm * width;
    }
    {
        const f32 t = (0.75f * radius - d.rootCutout) / Max(radius - d.rootCutout, 1.0e-6f);
        mRootChordAt75 = Math::mix(d.rootChord, d.tipChord, Clamp(t, 0.0f, 1.0f));
    }

    mAzimuthCount = d.azimuthStations;
    for (u32 k = 0; k < mAzimuthCount; ++k)
    {
        AzimuthStation& a = mAzimuth[k];
        const f32 psi = 2.0f * kPi * static_cast<f32>(k) / static_cast<f32>(mAzimuthCount);
        a.cosine = cosf(psi);
        a.sine = sinf(psi);
        a.radial = mAft * a.cosine + mAdvancing * a.sine;
        a.tangential = mAdvancing * a.cosine - mAft * a.sine;
    }

    const f32 length = Max(radius - e, 1.0e-3f);
    mFlapInertia = d.bladeMass * length * length / 3.0f;
    mFlapFirstMoment = d.bladeMass * length / 2.0f;
}

void Rotor::reset()
{
    for (u32 j = 0; j < MaxRadialStations; ++j)
        mInduced[j] = 0.0f;
    mFlap = FlapState();
    mLoads = RotorLoads();
}

void Rotor::setControls(f32 collective, f32 longitudinalCyclic, f32 lateralCyclic)
{
    mCollective = isfinite(collective) ? Clamp(collective, -0.6f, 0.6f) : 0.0f;
    mLongitudinalCyclic = isfinite(longitudinalCyclic) ? Clamp(longitudinalCyclic, -0.5f, 0.5f) : 0.0f;
    mLateralCyclic = isfinite(lateralCyclic) ? Clamp(lateralCyclic, -0.5f, 0.5f) : 0.0f;
}

f32 Rotor::solidity() const
{
    const Description& d = mDescription;
    const f32 meanChord = 0.5f * (d.rootChord + d.tipChord);
    const f32 bladeArea = static_cast<f32>(d.bladeCount) * meanChord * (d.radius - d.rootCutout);
    return bladeArea / discArea();
}

f32 Rotor::lockNumber(f32 density) const
{
    const f32 r2 = mDescription.radius * mDescription.radius;
    return density * mAirfoil.parameters().liftSlope * mRootChordAt75 * r2 * r2 / mFlapInertia;
}

f32 Rotor::flapFrequencyRatio(f32 angularSpeed) const
{
    if (!(angularSpeed > 1.0e-3f))
        return 0.0f;
    const f32 e = mDescription.hingeOffset;
    const f32 stiffness = (mFlapInertia + e * mFlapFirstMoment) * angularSpeed * angularSpeed +
                          mDescription.flapSpring;
    return sqrtf(stiffness / (mFlapInertia * angularSpeed * angularSpeed));
}

f32 Rotor::polarInertia() const
{
    const Description& d = mDescription;
    const f32 e = d.hingeOffset;
    const f32 length = d.radius - e;
    const f32 blade = d.bladeMass * (e * e + e * length + length * length / 3.0f);
    return static_cast<f32>(d.bladeCount) * blade + d.hubInertia;
}

bool Rotor::prepare(const RotorOperatingPoint& point, const f32* induced, f32 thrustEstimate,
                    Context& c) const
{
    const Description& d = mDescription;
    if (!isfinite(point.angularSpeed) || !finiteVec(point.velocity) ||
        !finiteVec(point.angularVelocity) || !(point.density > 0.0f) || !isfinite(point.density))
        return false;

    c.omega = Max(point.angularSpeed, 0.0f);
    c.rate = point.angularVelocity;
    // The blades turn at omega relative to the airframe, which itself yaws about the shaft.
    c.omegaAir = c.omega + d.rotation * Math::dot(c.rate, mShaft);
    c.velocity = point.velocity;
    c.density = point.density;
    c.climb = Math::dot(c.velocity, mShaft);
    const Math::vec3 inPlane = c.velocity - mShaft * c.climb;
    c.inPlane = Math::length(inPlane);
    // The air moves past the hub opposite to the hub's motion, and the wake goes with it.
    c.downstream = c.inPlane > 1.0e-4f ? inPlane * (-1.0f / c.inPlane) : Math::vec3(0.0f);

    f32 weighted = 0.0f;
    f32 area = 0.0f;
    for (u32 j = 0; j < mRadialCount; ++j)
    {
        const f32 dA = mRadial[j].radius * mRadial[j].width;
        weighted += induced[j] * dA;
        area += dA;
    }
    c.meanInduced = area > 0.0f ? weighted / area : 0.0f;
    c.hoverInduced = sqrtf(fabsf(thrustEstimate) / (2.0f * c.density * discArea()));

    // Cheeseman-Bennett, with the forward-speed term that fades it once the wake is blown back behind the disc.
    c.groundFactor = 1.0f;
    c.groundRatio = 0.0f;
    if (d.groundEffect && point.groundDistance > 0.0f && isfinite(point.groundDistance))
    {
        const f32 height = Max(point.groundDistance, 0.4f * d.radius);
        const f32 image = d.radius / (4.0f * height);
        const f32 induction = Max(fabsf(c.meanInduced), 0.1f);
        const f32 speedRatio = c.inPlane / induction;
        c.groundRatio = image * image / (1.0f + speedRatio * speedRatio);
        c.groundFactor = powf(1.0f - c.groundRatio, 1.5f);
    }

    // Wake skew and the Pitt-Peters first-harmonic gain. A wake blown up through the disc
    // (descent) is held at 90 deg: the linear form is not meant for it and the gain diverges past there.
    const f32 through = c.climb + c.groundFactor * c.meanInduced;
    c.skewAngle = c.inPlane > 1.0e-4f ? atan2f(c.inPlane, Max(through, 0.0f)) : 0.0f;
    c.skewAngle = Clamp(c.skewAngle, 0.0f, 0.5f * kPi);
    c.skew = (15.0f * kPi / 32.0f) * tanf(0.5f * c.skewAngle);

    // Turbulent-wake mass flow: in by a descent of one v_h, out again with
    // in-plane speed between 0.5 and 1.5 v_h.
    c.wakeWeight = 0.0f;
    if (c.hoverInduced > 0.05f)
    {
        const f32 descent = -c.climb / c.hoverInduced;
        const f32 lateral = c.inPlane / c.hoverInduced;
        c.wakeWeight = smoothstep01((descent - 0.2f) / 0.8f) * (1.0f - smoothstep01(lateral - 0.5f));
    }

    // Cyclic in the rotor's own frame: B1 tilts the disc forward (towards psi = 180), A1 towards
    // the advancing side, so lateral cyclic towards the body's right maps onto A1 with the sign of
    // which side advances. The phase rotates the input pattern earlier in azimuth.
    const Math::vec3 right = Math::cross(d.forward, mShaft);
    const f32 advancingIsRight = Math::dot(mAdvancing, right) >= 0.0f ? 1.0f : -1.0f;
    const f32 a1 = mLateralCyclic * advancingIsRight;
    const f32 b1 = mLongitudinalCyclic;
    const f32 cp = cosf(d.cyclicPhase);
    const f32 sp = sinf(d.cyclicPhase);
    c.cyclicCos = a1 * cp + b1 * sp;
    c.cyclicSin = b1 * cp - a1 * sp;
    return true;
}

void Rotor::sweep(const Context& c, const f32* induced, const FlapState& flap, Sweep& out) const
{
    const Description& d = mDescription;
    const f32 e = d.hingeOffset;
    const f32 radius = d.radius;
    const f32 share = static_cast<f32>(d.bladeCount) / static_cast<f32>(mAzimuthCount);
    const f32 halfRho = 0.5f * c.density;
    const f32 slope = mAirfoil.parameters().liftSlope;

    for (u32 j = 0; j < mRadialCount; ++j)
    {
        out.annulusThrust[j] = 0.0f;
        out.annulusSlope[j] = 0.0f;
    }
    out.force = Math::vec3(0.0f);
    out.torque = 0.0f;
    out.thrust = 0.0f;
    out.stalledLoad = 0.0f;
    out.totalLoad = 0.0f;

    for (u32 k = 0; k < mAzimuthCount; ++k)
    {
        const AzimuthStation& a = mAzimuth[k];
        const f32 beta = flap.coning + flap.cosine * a.cosine + flap.sine * a.sine;
        const f32 betaRate = (flap.sine * a.cosine - flap.cosine * a.sine) * c.omegaAir;
        const f32 cb = cosf(beta);
        const f32 sb = sinf(beta);
        const Math::vec3 normal = mShaft * cb - a.radial * sb;
        const f32 normalSpeed = Math::dot(c.velocity, normal);
        const f32 tangentialSpeed = Math::dot(c.velocity, a.tangential);
        // Airframe rotation moves the blade through the disc: (w x r e_r).n per unit radius.
        const f32 rateSpeed = Math::dot(Math::cross(c.rate, a.radial), normal);
        const f32 skewAzimuth = Math::dot(a.radial, c.downstream);
        const f32 cyclic = -c.cyclicCos * a.cosine - c.cyclicSin * a.sine;

        f32 moment = 0.0f;
        for (u32 j = 0; j < mRadialCount; ++j)
        {
            const RadialStation& s = mRadial[j];
            const f32 r = s.radius;
            const f32 inducedShape = c.groundFactor * (1.0f + c.skew * (r / radius) * skewAzimuth);
            const f32 inducedHere = induced[j] * inducedShape;
            const f32 ut = c.omegaAir * r + tangentialSpeed;
            const f32 up = normalSpeed + r * rateSpeed + inducedHere * cb + (r - e) * betaRate;
            const f32 speedSquared = ut * ut + up * up;
            if (speedSquared < 1.0e-8f)
                continue;
            const f32 speed = sqrtf(speedSquared);
            const f32 inflowAngle = atan2f(up, ut);
            const f32 pitch = mCollective + s.twist + cyclic;
            const AirfoilCoefficients coefficients = mAirfoil.evaluate(pitch - inflowAngle);
            const f32 q = halfRho * speedSquared * s.chord;
            const f32 lift = q * coefficients.lift;
            const f32 drag = q * coefficients.drag;
            const f32 cosPhi = ut / speed;
            const f32 sinPhi = up / speed;
            const f32 normalForce = lift * cosPhi - drag * sinPhi;
            const f32 inPlaneForce = lift * sinPhi + drag * cosPhi;
            const f32 weight = s.width * share;

            out.annulusThrust[j] += normalForce * cb * share;
            // d(normal force)/d(U_P) in attached flow, -rho c a U_T / 2: the Newton slope for the
            // annulus balance (in stall the true slope is smaller, so the step is only conservative).
            out.annulusSlope[j] -= halfRho * s.chord * slope * Max(ut, 0.0f) * inducedShape * cb * cb * share;
            out.force += (normal * normalForce - a.tangential * inPlaneForce) * weight;
            out.torque += inPlaneForce * r * weight;
            out.thrust += normalForce * cb * weight;
            moment += normalForce * (r - e) * s.width;

            const f32 load = fabsf(normalForce) * weight;
            out.totalLoad += load;
            if (coefficients.separation > 0.5f)
                out.stalledLoad += load;
        }
        out.flapMoment[k] = moment;
    }
}

f32 Rotor::updateInflow(const Context& c, const Sweep& sweep, f32* induced) const
{
    const Description& d = mDescription;
    const f32 blades = static_cast<f32>(d.bladeCount);
    const f32 wake = c.wakeWeight * kWakeMassFlow * kWakeMassFlow;
    f32 largest = 0.0f;
    for (u32 j = 0; j < mRadialCount; ++j)
    {
        const f32 r = mRadial[j].radius;
        const f32 v = induced[j];
        const f32 axial = c.climb + v;

        f32 loss = 1.0f;
        if (d.tipLoss)
        {
            const f32 through = fabsf(c.climb + c.groundFactor * v);
            const f32 sinPhi = Max(through / sqrtf(through * through + c.omegaAir * c.omegaAir * r * r + 1.0e-6f),
                                   1.0e-3f);
            const f32 exponent = 0.5f * blades * (d.radius - r) / (r * sinPhi);
            loss = Max((2.0f / kPi) * acosf(Clamp(expf(-exponent), 0.0f, 1.0f)), kMinLossFactor);
        }

        const f32 massFlow = sqrtf(c.inPlane * c.inPlane + axial * axial + wake * v * v);
        const f32 k = 4.0f * kPi * c.density * loss * r;
        const f32 momentum = k * v * massFlow;
        f32 momentumSlope = k * massFlow;
        if (massFlow > 1.0e-6f)
            momentumSlope += k * v * (axial + wake * v) / massFlow;

        const f32 residual = momentum - sweep.annulusThrust[j];
        // Blade element side falls with v and momentum rises: positive slope; floored for the degenerate still disc.
        const f32 derivative = Max(momentumSlope - sweep.annulusSlope[j], 0.05f * k + 1.0e-6f);
        f32 step = -residual / derivative;
        const f32 limit = 0.5f * fabsf(v) + 2.0f;
        step = Clamp(step, -limit, limit);
        if (!isfinite(step))
            step = 0.0f;
        induced[j] = v + step;
        largest = Max(largest, fabsf(step));
    }
    return largest;
}

f32 Rotor::updateFlapping(const Context& c, const Sweep& sweep, f32 relaxation, FlapState& flap) const
{
    const Description& d = mDescription;
    if (c.omegaAir < kMinFlappingSpeed && d.flapSpring <= 0.0f)
    {
        const f32 change = fabsf(flap.coning) + fabsf(flap.cosine) + fabsf(flap.sine);
        flap = FlapState();
        return change;
    }
    const f32 e = d.hingeOffset;
    const f32 omega = Max(c.omegaAir, 0.0f);
    const f32 omega2 = omega * omega;
    const f32 coningStiffness = (mFlapInertia + e * mFlapFirstMoment) * omega2 + d.flapSpring;
    const f32 cyclicStiffness = e * mFlapFirstMoment * omega2 + d.flapSpring;
    const f32 damping = 0.5f * c.density * mAirfoil.parameters().liftSlope * omega2 * mFlapDampingIntegral;
    const f32 gyroscopic = -2.0f * omega * (mFlapInertia + e * mFlapFirstMoment);

    f32 m0 = 0.0f;
    f32 m1c = 0.0f;
    f32 m1s = 0.0f;
    for (u32 k = 0; k < mAzimuthCount; ++k)
    {
        const AzimuthStation& a = mAzimuth[k];
        const f32 inertial = gyroscopic * Math::dot(Math::cross(c.rate, a.tangential), mShaft);
        const f32 m = sweep.flapMoment[k] + inertial;
        m0 += m;
        m1c += m * a.cosine;
        m1s += m * a.sine;
    }
    const f32 n = static_cast<f32>(mAzimuthCount);
    m0 /= n;
    m1c *= 2.0f / n;
    m1s *= 2.0f / n;

    const f32 r0 = coningStiffness * flap.coning - m0;
    const f32 r1c = cyclicStiffness * flap.cosine - m1c;
    const f32 r1s = cyclicStiffness * flap.sine - m1s;
    const f32 det = cyclicStiffness * cyclicStiffness + damping * damping;
    if (!(coningStiffness > 1.0e-9f) || !(det > 1.0e-12f))
        return 0.0f;
    f32 d0 = -r0 / coningStiffness;
    f32 d1c = (-cyclicStiffness * r1c + damping * r1s) / det;
    f32 d1s = -(damping * r1c + cyclicStiffness * r1s) / det;
    if (!isfinite(d0) || !isfinite(d1c) || !isfinite(d1s))
        return 0.0f;
    d0 *= relaxation;
    d1c *= relaxation;
    d1s *= relaxation;
    const FlapState before = flap;
    flap.coning = Clamp(flap.coning + d0, -kMaxFlap, kMaxFlap);
    flap.cosine = Clamp(flap.cosine + d1c, -kMaxFlap, kMaxFlap);
    flap.sine = Clamp(flap.sine + d1s, -kMaxFlap, kMaxFlap);
    return Max(fabsf(flap.coning - before.coning),
               Max(fabsf(flap.cosine - before.cosine), fabsf(flap.sine - before.sine)));
}

RotorLoads Rotor::finish(const Context& c, const Sweep& s, const f32* induced, const FlapState& flap) const
{
    const Description& d = mDescription;
    RotorLoads loads;
    if (!finiteVec(s.force) || !isfinite(s.torque))
        return loads;
    const Math::vec3 right = Math::cross(d.forward, mShaft);
    const f32 e = d.hingeOffset;
    const f32 omega2 = c.omegaAir * c.omegaAir;
    const f32 cyclicStiffness = e * mFlapFirstMoment * omega2 + d.flapSpring;

    loads.force = s.force;
    loads.thrust = Math::dot(s.force, mShaft);
    loads.hForce = Math::dot(s.force, mAft);
    loads.yForce = Math::dot(s.force, right);
    // Each blade flapped up by beta pulls its side of the hub up with
    // (e S_beta Omega^2 + K_beta) beta about e_r x shaft; over the disc that
    // leaves half the blades' worth times the first harmonic.
    loads.moment = (Math::cross(mAft, mShaft) * flap.cosine + Math::cross(mAdvancing, mShaft) * flap.sine) *
                   (0.5f * static_cast<f32>(d.bladeCount) * cyclicStiffness);
    loads.torque = s.torque;
    loads.power = s.torque * c.omega;

    const f32 area = discArea();
    const f32 tipSpeed = c.omega * d.radius;
    // Over the whole disc pi R^2, as momentum theory counts it: inside the
    // root cutout no blade exchanges momentum with the air.
    f32 weighted = 0.0f;
    for (u32 j = 0; j < mRadialCount; ++j)
        weighted += induced[j] * mRadial[j].radius * mRadial[j].width;
    loads.inducedVelocity = c.groundFactor * weighted / (0.5f * d.radius * d.radius);
    if (tipSpeed > 1.0e-3f)
    {
        loads.inflowRatio = (c.climb + loads.inducedVelocity) / tipSpeed;
        loads.advanceRatio = c.inPlane / tipSpeed;
        loads.thrustCoefficient = loads.thrust / (c.density * area * tipSpeed * tipSpeed);
    }

    loads.coning = flap.coning;
    loads.longitudinalFlapping = -flap.cosine;
    loads.lateralFlapping = -flap.sine;
    loads.discTiltForward = flap.cosine;
    const f32 advancingIsRight = Math::dot(mAdvancing, right) >= 0.0f ? 1.0f : -1.0f;
    loads.discTiltRight = -flap.sine * advancingIsRight;
    loads.wakeSkew = c.skewAngle;
    loads.groundEffectFactor = 1.0f / (1.0f - c.groundRatio);

    const f32 hover = sqrtf(fabsf(loads.thrust) / (2.0f * c.density * area));
    if (hover > 1.0e-3f)
    {
        loads.descentRatio = -c.climb / hover;
        loads.vortexRing = loads.descentRatio >= 0.5f && loads.descentRatio <= 2.0f &&
                           c.inPlane <= 0.9f * hover && loads.thrust > 0.0f;
    }
    loads.stalledFraction = s.totalLoad > 0.0f ? s.stalledLoad / s.totalLoad : 0.0f;
    return loads;
}

const RotorLoads& Rotor::solve(const RotorOperatingPoint& point)
{
    Context context;
    Sweep sweepResult;
    f32 thrust = mLoads.thrust;
    if (!prepare(point, mInduced, thrust, context))
    {
        reset();
        return mLoads;
    }
    for (u32 iteration = 0; iteration < kMaxSolveSweeps; ++iteration)
    {
        prepare(point, mInduced, thrust, context);
        sweep(context, mInduced, mFlap, sweepResult);
        thrust = sweepResult.thrust;
        const f32 inflowChange = updateInflow(context, sweepResult, mInduced);
        const f32 flapChange = updateFlapping(context, sweepResult, 1.0f, mFlap);
        f32 scale = 1.0f;
        for (u32 j = 0; j < mRadialCount; ++j)
            scale = Max(scale, fabsf(mInduced[j]));
        if (inflowChange < 1.0e-5f * scale && flapChange < 1.0e-6f)
            break;
    }
    prepare(point, mInduced, thrust, context);
    sweep(context, mInduced, mFlap, sweepResult);
    mLoads = finish(context, sweepResult, mInduced, mFlap);
    return mLoads;
}

const RotorLoads& Rotor::advance(const RotorOperatingPoint& point, f32 step)
{
    bool still = true;
    for (u32 j = 0; j < mRadialCount && still; ++j)
        still = mInduced[j] == 0.0f;
    if (still || !(step > 0.0f) || !isfinite(step))
        return solve(point);

    Context context;
    Sweep sweepResult;
    f32 thrust = mLoads.thrust;
    if (!prepare(point, mInduced, thrust, context))
    {
        reset();
        return mLoads;
    }

    // The tip-path plane approaches its quasi-steady attitude with the regressing flap mode's
    // time constant 2 I_beta Omega / D = 16 / (gamma Omega). Each sweep's Newton step is the
    // remaining distance; taking 1 - exp(-dt / tau) of it, split across the sweeps, integrates
    // that first-order lag exactly.
    f32 relaxation = 1.0f;
    if (mDescription.flapLag && context.omegaAir > kMinFlappingSpeed)
    {
        const f32 damping = 0.5f * context.density * mAirfoil.parameters().liftSlope *
                            context.omegaAir * context.omegaAir * mFlapDampingIntegral;
        const f32 tau = damping > 0.0f ? 2.0f * mFlapInertia * context.omegaAir / damping : 0.0f;
        if (tau > 0.0f)
            relaxation = 1.0f - expf(-step / (tau * static_cast<f32>(kAdvanceSweeps)));
    }

    for (u32 iteration = 0; iteration < kAdvanceSweeps; ++iteration)
    {
        prepare(point, mInduced, thrust, context);
        sweep(context, mInduced, mFlap, sweepResult);
        thrust = sweepResult.thrust;
        updateInflow(context, sweepResult, mInduced);
        updateFlapping(context, sweepResult, relaxation, mFlap);
    }
    prepare(point, mInduced, thrust, context);
    sweep(context, mInduced, mFlap, sweepResult);
    mLoads = finish(context, sweepResult, mInduced, mFlap);
    return mLoads;
}

RotorLoads Rotor::evaluate(const RotorOperatingPoint& point) const
{
    Context context;
    if (!prepare(point, mInduced, mLoads.thrust, context))
        return RotorLoads();
    Sweep sweepResult;
    sweep(context, mInduced, mFlap, sweepResult);
    return finish(context, sweepResult, mInduced, mFlap);
}

} // namespace kinetix::Physics
