#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>
#include <kinetix/core/Log.h>

#include "dynamics/Propeller.h"

#include "dynamics/Atmosphere.h"
#include "dynamics/RigidBody.h"

#include <math.h>

namespace kinetix::Physics
{

namespace
{
constexpr f32 kPi = 3.14159265358979f;
constexpr f32 kRpmToRadians = 2.0f * kPi / 60.0f;

// Below this a station is not moving relative to the air and carries nothing;
// it only keeps atan2 and the loss factor away from 0 / 0.
constexpr f32 kMinSpeed = 1.0e-3f;

// The loss factor goes to zero at the tip and the momentum balance divides by
// it, so it is floored (stations sit at mid-annulus; matters only for a very
// coarse disc).
constexpr f32 kMinLossFactor = 0.05f;

constexpr u32 kSwirlPasses = 3;
constexpr u32 kMaxRootIterations = 40;

struct ElementLoads
{
    f32 thrust; // per unit radius, all blades
    f32 torque; // per unit radius, all blades
    f32 loss;
};
} // namespace

Propeller::Propeller()
{
    mAirfoil = Airfoil::propeller();
    rebuild();
}

Propeller::Propeller(const Description& description, const Airfoil& airfoil)
    : mAirfoil(airfoil)
{
    setDescription(description);
}

void Propeller::setDescription(const Description& description)
{
    mDescription = description;
    Description& d = mDescription;
    d.radius = Max(description.radius, 0.01f);
    d.hubRadius = Clamp(description.hubRadius, 0.0f, 0.9f * d.radius);
    d.bladeCount = Clamp(description.bladeCount, 1u, 12u);
    d.rootChord = Max(description.rootChord, 1.0e-3f);
    d.tipChord = Max(description.tipChord, 1.0e-3f);
    d.pitch = Max(description.pitch, 0.0f);
    d.rotation = description.rotation < 0.0f ? -1.0f : 1.0f;
    d.inertia = Max(description.inertia, 1.0e-4f);
    d.elementCount = Clamp(description.elementCount, 2u, MaxStations);
    if (!(Math::dot(d.axis, d.axis) > 1.0e-12f) || !isfinite(d.axis.x) || !isfinite(d.axis.y) ||
        !isfinite(d.axis.z))
    {
        Log::error("Propeller: axis must be a finite, non-zero direction");
        d.axis = Math::vec3(0.0f, 0.0f, 1.0f);
    }
    d.axis = Math::normalize(d.axis);
    rebuild();
}

void Propeller::rebuild()
{
    const Description& d = mDescription;
    mStationCount = d.elementCount;
    const f32 width = (d.radius - d.hubRadius) / static_cast<f32>(mStationCount);
    for (u32 i = 0; i < mStationCount; ++i)
    {
        Station& station = mStations[i];
        station.radius = d.hubRadius + (static_cast<f32>(i) + 0.5f) * width;
        station.width = width;
        const f32 t = (station.radius - d.hubRadius) / Max(d.radius - d.hubRadius, 1.0e-6f);
        station.chord = Math::mix(d.rootChord, d.tipChord, t);
        station.bladeAngle = atanf(d.pitch / (2.0f * kPi * station.radius)) + d.twistOffset;
    }
}

void Propeller::setRpm(f32 rpm)
{
    mMode = Mode::Rpm;
    mOmega = isfinite(rpm) ? Max(rpm, 0.0f) * kRpmToRadians : 0.0f;
}

void Propeller::setShaftTorque(f32 torque)
{
    mMode = Mode::Torque;
    mShaftTorque = isfinite(torque) ? torque : 0.0f;
}

void Propeller::setAngularSpeed(f32 omega)
{
    mOmega = isfinite(omega) ? Max(omega, 0.0f) : 0.0f;
}

f32 Propeller::rpm() const
{
    return mOmega / kRpmToRadians;
}

f32 Propeller::advanceRatio(f32 omega, f32 axialSpeed) const
{
    const f32 revolutions = omega / (2.0f * kPi);
    if (revolutions <= 1.0e-6f)
        return 0.0f;
    return axialSpeed / (revolutions * 2.0f * mDescription.radius);
}

PropellerLoads Propeller::evaluate(f32 omega, f32 axialSpeed, f32 density) const
{
    PropellerLoads loads;
    if (!isfinite(omega) || !isfinite(axialSpeed) || !(density > 0.0f) || !isfinite(density))
        return loads;
    omega = Max(omega, 0.0f);
    // Flow from behind the disc is outside momentum theory; treated as the static case.
    const f32 freeStream = Max(axialSpeed, 0.0f);
    const f32 blades = static_cast<f32>(mDescription.bladeCount);
    const f32 hub = mDescription.hubRadius;
    const f32 tip = mDescription.radius;

    for (u32 s = 0; s < mStationCount; ++s)
    {
        const Station& station = mStations[s];
        const f32 r = station.radius;
        const f32 rotational = omega * r;
        if (rotational < kMinSpeed && freeStream < kMinSpeed)
            continue;

        f32 swirl = 0.0f;
        ElementLoads element = {0.0f, 0.0f, 1.0f};

        // Blade element loads for a given axial induced velocity at the current
        // swirl estimate.
        auto bladeElement = [&](f32 induced)
        {
            const f32 axial = freeStream + induced;
            const f32 tangential = Max(rotational - swirl, 0.0f);
            const f32 inflowAngle = atan2f(axial, tangential);
            const AirfoilCoefficients c = mAirfoil.evaluate(station.bladeAngle - inflowAngle);
            const f32 relativeSquared = axial * axial + tangential * tangential;
            const f32 k = 0.5f * density * relativeSquared * blades * station.chord;
            const f32 cosPhi = cosf(inflowAngle);
            const f32 sinPhi = sinf(inflowAngle);

            // Prandtl's tip and hub losses: the share of the annulus' ideal
            // momentum change a finite number of blades achieves.
            const f32 sinGuard = Max(fabsf(sinPhi), 1.0e-3f);
            const f32 tipExponent = 0.5f * blades * (tip - r) / (r * sinGuard);
            f32 loss = (2.0f / kPi) * acosf(Clamp(expf(-tipExponent), 0.0f, 1.0f));
            if (hub > 0.0f)
            {
                const f32 hubExponent = 0.5f * blades * (r - hub) / (r * sinGuard);
                loss *= (2.0f / kPi) * acosf(Clamp(expf(-hubExponent), 0.0f, 1.0f));
            }

            ElementLoads out;
            out.thrust = k * (c.lift * cosPhi - c.drag * sinPhi);
            out.torque = k * (c.lift * sinPhi + c.drag * cosPhi) * r;
            out.loss = Max(loss, kMinLossFactor);
            return out;
        };
        // Momentum minus blade element thrust per unit radius: the root of
        // this is the induced velocity.
        auto residual = [&](f32 induced, ElementLoads& out)
        {
            out = bladeElement(induced);
            return 4.0f * kPi * r * density * out.loss * (freeStream + induced) * induced - out.thrust;
        };

        f32 induced = 0.0f;
        bool interior = false;
        for (u32 pass = 0; pass < kSwirlPasses; ++pass)
        {
            // Bracket: momentum theory holds only for v >= -V0 / 2 (the far wake
            // at rest) and no induced velocity nears the blade speed.
            const f32 floor = -0.5f * freeStream;
            f32 lo = floor;
            f32 hi = freeStream + rotational + 1.0f;
            ElementLoads loLoads;
            ElementLoads hiLoads;
            f32 fLo = 0.0f;
            f32 fHi = 0.0f;
            bool bracketed = false;
            if (interior)
            {
                // The root moved little since the last pass: a narrow bracket
                // around the previous one usually holds it and saves iterations.
                const f32 margin = 0.02f * (fabsf(induced) + 1.0f);
                lo = Max(induced - margin, floor);
                hi = induced + margin;
                fLo = residual(lo, loLoads);
                fHi = residual(hi, hiLoads);
                bracketed = fLo < 0.0f && fHi > 0.0f;
            }
            if (!bracketed)
            {
                lo = floor;
                hi = freeStream + rotational + 1.0f;
                fLo = residual(lo, loLoads);
                fHi = residual(hi, hiLoads);
                for (u32 grow = 0; grow < 8 && fHi < 0.0f; ++grow)
                {
                    hi *= 2.0f;
                    fHi = residual(hi, hiLoads);
                }
            }
            interior = fLo < 0.0f && fHi > 0.0f;

            if (fLo >= 0.0f)
            {
                // The element brakes harder than the slowest wake momentum theory
                // allows (turbulent windmill state): held at the limit.
                induced = lo;
                element = loLoads;
            }
            else if (fHi <= 0.0f)
            {
                induced = hi;
                element = hiLoads;
            }
            else
            {
                // Illinois false position: bracketed, superlinear.
                f32 fLoScaled = fLo;
                f32 fHiScaled = fHi;
                int lastSide = 0;
                induced = lo;
                f32 previousGuess = lo;
                for (u32 iteration = 0; iteration < kMaxRootIterations; ++iteration)
                {
                    const f32 guess = (lo * fHiScaled - hi * fLoScaled) / (fHiScaled - fLoScaled);
                    ElementLoads guessLoads;
                    const f32 fGuess = residual(guess, guessLoads);
                    induced = guess;
                    element = guessLoads;
                    // Converged once the estimate stops moving or the bracket closes.
                    const f32 tolerance = 1.0e-4f * (1.0f + fabsf(guess));
                    if (fGuess == 0.0f || fabsf(hi - lo) < tolerance ||
                        (iteration > 0 && fabsf(guess - previousGuess) < tolerance))
                        break;
                    previousGuess = guess;
                    if (fGuess < 0.0f)
                    {
                        lo = guess;
                        fLoScaled = fGuess;
                        if (lastSide == -1)
                            fHiScaled *= 0.5f;
                        lastSide = -1;
                    }
                    else
                    {
                        hi = guess;
                        fHiScaled = fGuess;
                        if (lastSide == 1)
                            fLoScaled *= 0.5f;
                        lastSide = 1;
                    }
                }
            }

            // Swirl from the angular momentum the torque puts into the wake. It
            // feeds back weakly (w is a few percent of omega r), so plain
            // substitution converges: three passes land within ~1% of the
            // converged thrust.
            const f32 throughFlow = Max(freeStream + induced, 0.05f);
            const f32 target = element.torque / (4.0f * kPi * r * r * density * element.loss * throughFlow);
            swirl = Clamp(target, -0.5f * rotational, 0.5f * rotational);
        }

        loads.thrust += element.thrust * station.width;
        loads.torque += element.torque * station.width;
    }
    if (!isfinite(loads.thrust) || !isfinite(loads.torque))
        return PropellerLoads();
    loads.power = loads.torque * omega;
    return loads;
}

f32 Propeller::axialInflow(const RigidBody& body, const Atmosphere& atmosphere) const
{
    const Math::vec3 position = body.pointToWorld(mDescription.position);
    const Math::vec3 axis = body.directionToWorld(mDescription.axis);
    const Math::vec3 air = atmosphere.windAt(position) - body.velocityAtPoint(position);
    return -Math::dot(air, axis);
}

PropellerLoads Propeller::computeForces(const RigidBody& body, const Atmosphere& atmosphere,
                                        Math::vec3& force, Math::vec3& torque) const
{
    const Math::vec3 position = body.pointToWorld(mDescription.position);
    const Math::vec3 axis = body.directionToWorld(mDescription.axis);
    const f32 density = atmosphere.sampleAt(position).density;
    const PropellerLoads loads = evaluate(mOmega, axialInflow(body, atmosphere), density);

    const Math::vec3 spin = axis * mDescription.rotation;
    // Whatever drives the shaft pushes back on the airframe: the aerodynamic
    // torque in rpm mode (speed held, so they balance), the engine's torque in
    // torque mode (the difference spins the rotor up, not the body).
    const f32 shaft = mMode == Mode::Rpm ? loads.torque : mShaftTorque;
    // The rotor's angular momentum is carried by the body; turning it takes a
    // torque w x H and the airframe feels the reaction.
    const Math::vec3 momentum = spin * (mDescription.inertia * mOmega);

    force = axis * loads.thrust;
    torque = Math::cross(position - body.position(), force) - spin * shaft +
             Math::cross(momentum, body.angularVelocity());
    return loads;
}

const PropellerLoads& Propeller::apply(RigidBody& body, const Atmosphere& atmosphere, f32 step,
                                       Math::vec3* appliedForce, Math::vec3* appliedTorque)
{
    Math::vec3 force;
    Math::vec3 torque;
    mLoads = computeForces(body, atmosphere, force, torque);
    body.addForce(force);
    body.addTorque(torque);
    if (appliedForce)
        *appliedForce = force;
    if (appliedTorque)
        *appliedTorque = torque;

    if (mMode == Mode::Torque && step > 0.0f && isfinite(step))
    {
        // Linearly implicit in the aerodynamic torque, or a light rotor on a long
        // step overshoots the balance speed and oscillates. dQ/domega is taken as
        // 2 Q / omega (exact for static Q ~ omega^2); it only sets how the speed
        // approaches the balance, never where it is.
        const f32 slope = mOmega > 1.0f ? 2.0f * mLoads.torque / mOmega : 0.0f;
        const f32 effectiveInertia = mDescription.inertia + step * Max(slope, 0.0f);
        mOmega += step * (mShaftTorque - mLoads.torque) / effectiveInertia;
        if (!isfinite(mOmega) || mOmega < 0.0f)
            mOmega = 0.0f;
    }
    return mLoads;
}

} // namespace kinetix::Physics
