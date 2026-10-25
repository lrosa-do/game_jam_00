#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>
#include <kinetix/core/Log.h>

#include "dynamics/Aircraft.h"

#include "dynamics/RigidBody.h"

#include <math.h>

namespace kinetix::Physics
{

namespace
{
constexpr f32 kRpmToRadians = 2.0f * 3.14159265358979f / 60.0f;
constexpr u32 kTrimUnknowns = 6;

bool finiteVec(const Math::vec3& v)
{
    return isfinite(v.x) && isfinite(v.y) && isfinite(v.z);
}

// Illinois false position on [lo, hi] with f(lo), f(hi) of opposite sign;
// stays bracketed and is superlinear.
template <typename Function>
f32 solveBracketed(Function f, f32 lo, f32 hi, f32 fLo, f32 fHi, f32 tolerance)
{
    f32 root = lo;
    int lastSide = 0;
    for (u32 iteration = 0; iteration < 60; ++iteration)
    {
        root = (lo * fHi - hi * fLo) / (fHi - fLo);
        if (!isfinite(root))
            root = 0.5f * (lo + hi);
        const f32 value = f(root);
        if (value == 0.0f || hi - lo < tolerance)
            break;
        if ((value < 0.0f) == (fLo < 0.0f))
        {
            lo = root;
            fLo = value;
            if (lastSide == -1)
                fHi *= 0.5f;
            lastSide = -1;
        }
        else
        {
            hi = root;
            fHi = value;
            if (lastSide == 1)
                fLo *= 0.5f;
            lastSide = 1;
        }
    }
    return root;
}

// Gaussian elimination with partial pivoting; a vanished pivot leaves that
// unknown at zero.
void solveLinear(f32 a[kTrimUnknowns][kTrimUnknowns], f32 b[kTrimUnknowns], f32 x[kTrimUnknowns])
{
    constexpr u32 n = kTrimUnknowns;
    for (u32 column = 0; column < n; ++column)
    {
        u32 pivot = column;
        for (u32 row = column + 1; row < n; ++row)
            if (fabsf(a[row][column]) > fabsf(a[pivot][column]))
                pivot = row;
        if (pivot != column)
        {
            for (u32 k = 0; k < n; ++k)
                Swap(a[column][k], a[pivot][k]);
            Swap(b[column], b[pivot]);
        }
        if (fabsf(a[column][column]) < 1.0e-12f)
            continue;
        for (u32 row = column + 1; row < n; ++row)
        {
            const f32 factor = a[row][column] / a[column][column];
            for (u32 k = column; k < n; ++k)
                a[row][k] -= factor * a[column][k];
            b[row] -= factor * b[column];
        }
    }
    for (u32 i = n; i-- > 0;)
    {
        if (fabsf(a[i][i]) < 1.0e-12f)
        {
            x[i] = 0.0f;
            continue;
        }
        f32 sum = b[i];
        for (u32 k = i + 1; k < n; ++k)
            sum -= a[i][k] * x[k];
        x[i] = sum / a[i][i];
    }
}

f32 largest(const f32 values[kTrimUnknowns])
{
    f32 result = 0.0f;
    for (u32 i = 0; i < kTrimUnknowns; ++i)
        result = Max(result, fabsf(values[i]));
    return result;
}

f32 sumOfSquares(const f32 values[kTrimUnknowns])
{
    f32 result = 0.0f;
    for (u32 i = 0; i < kTrimUnknowns; ++i)
        result += values[i] * values[i];
    return result;
}
} // namespace

Aircraft::Aircraft() = default;

void Aircraft::setAxes(const Math::vec3& forward, const Math::vec3& up)
{
    if (!finiteVec(forward) || !finiteVec(up) || Math::dot(forward, forward) < 1.0e-12f)
    {
        Log::error("Aircraft: forward axis must be finite and non-zero");
        return;
    }
    const Math::vec3 f = Math::normalize(forward);
    Math::vec3 u = up - f * Math::dot(up, f);
    if (Math::dot(u, u) < 1.0e-12f)
    {
        Log::error("Aircraft: up axis must not be parallel to forward");
        return;
    }
    mForward = f;
    mUp = Math::normalize(u);
}

u32 Aircraft::addWing(const Wing& wing)
{
    mWings.push_back(wing);
    mDownwash.push_back(DownwashLink());
    return static_cast<u32>(mWings.size() - 1);
}

void Aircraft::setDownwash(u32 wing, s32 sourceWing, f32 factor)
{
    if (wing >= mWings.size() ||
        (sourceWing >= 0 && (static_cast<u32>(sourceWing) >= mWings.size() ||
                             static_cast<u32>(sourceWing) == wing)))
    {
        Log::error("Aircraft: downwash from wing %d onto wing %u - both must exist and differ",
                   sourceWing, wing);
        return;
    }
    mDownwash[wing].source = sourceWing;
    mDownwash[wing].factor = isfinite(factor) ? Max(factor, 0.0f) : 0.0f;
    if (sourceWing < 0)
        mWings[wing].setInducedFlow(Math::vec3(0.0f));
}

u32 Aircraft::addEngine(const PistonEngine& engine)
{
    mEngines.push_back(engine);
    return static_cast<u32>(mEngines.size() - 1);
}

u32 Aircraft::addPropeller(const Propeller& propeller, s32 engine)
{
    PropellerEntry entry;
    entry.propeller = propeller;
    entry.engine = engine;
    if (engine >= 0 && static_cast<u32>(engine) >= mEngines.size())
    {
        Log::error("Aircraft: engine %d does not exist - propeller left undriven", engine);
        entry.engine = -1;
    }
    if (entry.engine >= 0)
    {
        // Driven by torque from here on; the speed it had is where it starts.
        entry.propeller.setShaftTorque(0.0f);
    }
    mPropellers.push_back(entry);
    return static_cast<u32>(mPropellers.size() - 1);
}

u32 Aircraft::addDragElement(const DragElement& element)
{
    DragElement stored = element;
    stored.dragAreas = Math::max(element.dragAreas, Math::vec3(0.0f));
    mDragElements.push_back(stored);
    return static_cast<u32>(mDragElements.size() - 1);
}

void Aircraft::setControls(const ControlInputs& controls)
{
    auto unit = [](f32 value, f32 lo)
    {
        return isfinite(value) ? Clamp(value, lo, 1.0f) : 0.0f;
    };
    mControls.aileron = unit(controls.aileron, -1.0f);
    mControls.elevator = unit(controls.elevator, -1.0f);
    mControls.rudder = unit(controls.rudder, -1.0f);
    mControls.flaps = unit(controls.flaps, 0.0f);
    mControls.throttle = unit(controls.throttle, 0.0f);
    mControls.aileronTrim = unit(controls.aileronTrim, -1.0f);
    mControls.elevatorTrim = unit(controls.elevatorTrim, -1.0f);
    mControls.rudderTrim = unit(controls.rudderTrim, -1.0f);
}

void Aircraft::updateControlSurfaces()
{
    const f32 aileron = Clamp(mControls.aileron + mControls.aileronTrim, -1.0f, 1.0f);
    const f32 elevator = Clamp(mControls.elevator + mControls.elevatorTrim, -1.0f, 1.0f);
    const f32 rudder = Clamp(mControls.rudder + mControls.rudderTrim, -1.0f, 1.0f);
    const f32 flaps = mControls.flaps;

    for (Wing& wing : mWings)
    {
        for (u32 i = 0; i < wing.controlSurfaceCount(); ++i)
        {
            const ControlSurface::Mix& mix = wing.controlSurface(i).mix;
            // Roll and yaw are antisymmetric (opposite on the mirrored panel),
            // pitch and flap symmetric.
            const f32 symmetric = elevator * mix.elevator + flaps * mix.flaps;
            const f32 antisymmetric = aileron * mix.aileron + rudder * mix.rudder;
            wing.setControlCommand(i, symmetric + antisymmetric, symmetric - antisymmetric);
        }
    }
}

void Aircraft::driveEngines(f32 density)
{
    for (PistonEngine& engine : mEngines)
        engine.setThrottle(mControls.throttle);
    for (PropellerEntry& entry : mPropellers)
    {
        if (entry.engine < 0)
            continue;
        const PistonEngine& engine = mEngines[static_cast<u32>(entry.engine)];
        entry.propeller.setShaftTorque(engine.shaftTorque(entry.propeller.angularSpeed(), density));
    }
}

void Aircraft::accumulate(bool apply, f32 step, Math::vec3& force, Math::vec3& torque)
{
    force = Math::vec3(0.0f);
    torque = Math::vec3(0.0f);
    RigidBody& body = *mBody;
    const Atmosphere& air = atmosphere();
    const f32 density = air.sampleAt(body.position()).density;

    updateControlSurfaces();
    driveEngines(density);

    for (PropellerEntry& entry : mPropellers)
    {
        Math::vec3 f;
        Math::vec3 t;
        if (apply)
            entry.propeller.apply(body, air, step, &f, &t);
        else
            entry.propeller.computeForces(body, air, f, t);
        force += f;
        torque += t;
    }

    const f32 airspeed = Math::length(body.velocity() - air.windAt(body.position()));
    for (u32 i = 0; i < mWings.size(); ++i)
    {
        Wing& wing = mWings[i];
        const DownwashLink& link = mDownwash[i];
        if (link.source >= 0)
        {
            // The source's angle is from this evaluation if it came earlier in the list.
            const f32 epsilon = link.factor * mWings[static_cast<u32>(link.source)].state().inducedAngle;
            wing.setInducedFlow(mUp * (-epsilon * airspeed));
        }
        const WingState& state = apply ? wing.apply(body, air) : wing.computeForces(body, air);
        force += state.force;
        torque += state.torque;
    }

    for (const DragElement& element : mDragElements)
    {
        const Math::vec3 point = body.pointToWorld(element.position);
        const Math::vec3 flow = body.directionToLocal(air.windAt(point) - body.velocityAtPoint(point));
        const f32 speed = Math::length(flow);
        // Per body axis: a box-like body drags differently broadside than head
        // on, and this still points exactly along the flow when it is axial.
        const Math::vec3 local = element.dragAreas * flow * (0.5f * density * speed);
        const Math::vec3 f = body.directionToWorld(local);
        const Math::vec3 t = Math::cross(point - body.position(), f);
        if (apply)
        {
            body.addForce(f);
            body.addTorque(t);
        }
        force += f;
        torque += t;
    }

    updateTelemetry(force, torque);
}

void Aircraft::applyForces(f32 step)
{
    if (!mBody)
        return;
    Math::vec3 force;
    Math::vec3 torque;
    accumulate(true, step, force, torque);
}

void Aircraft::computeForces(Math::vec3& force, Math::vec3& torque)
{
    force = Math::vec3(0.0f);
    torque = Math::vec3(0.0f);
    if (!mBody)
        return;
    accumulate(false, 0.0f, force, torque);
}

void Aircraft::stepCallback(f32 step, void* userData)
{
    if (userData)
        static_cast<Aircraft*>(userData)->applyForces(step);
}

void Aircraft::updateTelemetry(const Math::vec3& force, const Math::vec3& torque)
{
    const RigidBody& body = *mBody;
    const Atmosphere& air = atmosphere();
    const AtmosphereState state = air.sampleAt(body.position());

    AircraftTelemetry t;
    t.force = force;
    t.torque = torque;
    t.density = state.density;
    t.altitude = air.altitudeAt(body.position());
    t.verticalSpeed = body.velocity().y;

    const Math::vec3 right = rightAxis();
    const Math::vec3 motion = body.directionToLocal(body.velocity() - air.windAt(body.position()));
    const f32 speed = Math::length(motion);
    t.trueAirspeed = speed;
    t.equivalentAirspeed = speed * sqrtf(state.density / Atmosphere::SeaLevelDensity);
    t.mach = speed / state.speedOfSound;
    if (speed > 1.0e-3f)
    {
        t.angleOfAttack = atan2f(-Math::dot(motion, mUp), Math::dot(motion, mForward));
        t.sideslip = asinf(Clamp(Math::dot(motion, right) / speed, -1.0f, 1.0f));
    }

    const Math::vec3 forwardWorld = body.directionToWorld(mForward);
    const Math::vec3 upWorld = body.directionToWorld(mUp);
    const Math::vec3 rightWorld = body.directionToWorld(right);
    t.pitch = asinf(Clamp(forwardWorld.y, -1.0f, 1.0f));
    t.bank = atan2f(-rightWorld.y, upWorld.y);
    t.heading = atan2f(-forwardWorld.x, forwardWorld.z);

    const Math::vec3 rates = body.directionToLocal(body.angularVelocity());
    t.bodyRates = Math::vec3(Math::dot(rates, mForward), Math::dot(rates, right),
                             -Math::dot(rates, mUp));
    mTelemetry = t;
}

f32 Aircraft::balancedPropellerSpeed(u32 index, f32 inflow, f32 density) const
{
    const PropellerEntry& entry = mPropellers[index];
    if (entry.engine < 0)
        return entry.propeller.angularSpeed();
    const PistonEngine& engine = mEngines[static_cast<u32>(entry.engine)];
    // Where the engine's torque meets the propeller's: the steady speed a
    // fixed-pitch propeller settles at for this throttle and airspeed.
    auto excess = [&](f32 omega)
    {
        return engine.shaftTorque(omega, density) -
               entry.propeller.evaluate(omega, inflow, density).torque;
    };
    const f32 hi = engine.description().maxRpm * kRpmToRadians * 1.3f;
    const f32 fLo = excess(0.0f);
    const f32 fHi = excess(hi);
    if (fLo <= 0.0f)
        return 0.0f;
    if (fHi >= 0.0f)
        return hi;
    return solveBracketed(excess, 0.0f, hi, fLo, fHi, 1.0e-3f);
}

TrimResult Aircraft::trim(const TrimTarget& target)
{
    TrimResult result;
    if (!mBody || !mBody->hasFiniteMass() || !(target.airspeed > 0.5f) ||
        !isfinite(target.airspeed))
    {
        Log::error("Aircraft: trim needs a dynamic body and an airspeed above 0.5 m/s");
        return result;
    }
    RigidBody& body = *mBody;
    const Atmosphere& air = atmosphere();

    const Math::vec3 savedPosition = body.position();
    const Math::quat savedOrientation = body.orientation();
    const Math::vec3 savedVelocity = body.velocity();
    const Math::vec3 savedAngularVelocity = body.angularVelocity();
    const ControlInputs savedControls = mControls;
    ct::Vector<f32> savedSpeeds;
    for (const PropellerEntry& entry : mPropellers)
        savedSpeeds.push_back(entry.propeller.angularSpeed());

    Math::vec3 heading = body.directionToWorld(mForward);
    heading.y = 0.0f;
    if (Math::dot(heading, heading) < 1.0e-6f)
        heading = Math::vec3(0.0f, 0.0f, 1.0f);
    heading = Math::normalize(heading);
    const Math::vec3 worldUp(0.0f, 1.0f, 0.0f);

    const bool solveThrottle = target.throttle < 0.0f;
    const f32 weight = body.mass() * Math::length(target.gravity);
    const f32 lengthScale = mWings.empty() ? 1.0f : Max(sqrtf(mWings[0].area()), 1.0f);
    const Math::vec3 wind = air.windAt(savedPosition);

    // Unknowns: angle of attack, sideslip, elevator, aileron and rudder trim,
    // and throttle or climb angle.
    const f32 lower[kTrimUnknowns] = {-0.35f, -0.5f, -1.0f, -1.0f, -1.0f,
                                      solveThrottle ? 0.0f : -0.8f};
    const f32 upper[kTrimUnknowns] = {0.6f, 0.5f, 1.0f, 1.0f, 1.0f, solveThrottle ? 1.0f : 0.8f};

    auto clampUnknowns = [&](f32 x[kTrimUnknowns])
    {
        for (u32 i = 0; i < kTrimUnknowns; ++i)
            x[i] = Clamp(x[i], lower[i], upper[i]);
    };

    // Puts the aircraft in the state x describes and measures what is left
    // of each balance, forces in weights and moments in weight-lengths.
    auto residual = [&](const f32 x[kTrimUnknowns], f32 r[kTrimUnknowns])
    {
        const f32 alpha = x[0];
        const f32 beta = x[1];
        const f32 climb = solveThrottle ? target.climbAngle : x[5];

        ControlInputs controls;
        controls.flaps = target.flaps;
        controls.throttle = solveThrottle ? x[5] : target.throttle;
        controls.elevatorTrim = x[2];
        controls.aileronTrim = x[3];
        controls.rudderTrim = x[4];
        setControls(controls);

        // Wind axes: along the flight path, and up and right of it with the
        // wings level. The body is yawed by the sideslip and then pitched by
        // the angle of attack from there.
        const Math::vec3 path = heading * cosf(climb) + worldUp * sinf(climb);
        const Math::vec3 pathUp = worldUp * cosf(climb) - heading * sinf(climb);
        const Math::vec3 pathRight = Math::cross(path, pathUp);
        const Math::vec3 yawed = path * cosf(beta) - pathRight * sinf(beta);
        const Math::vec3 forwardWorld = yawed * cosf(alpha) + pathUp * sinf(alpha);
        const Math::vec3 upWorld = pathUp * cosf(alpha) - yawed * sinf(alpha);
        const Math::vec3 rightWorld = Math::cross(forwardWorld, upWorld);

        const Math::mat3 worldBasis(forwardWorld, upWorld, rightWorld);
        const Math::mat3 bodyBasis(mForward, mUp, rightAxis());
        const Math::mat3 rotation = worldBasis * Math::transpose(bodyBasis);
        body.setOrientation(Math::normalize(Math::quat_cast(rotation)));
        body.setPosition(savedPosition);
        body.setVelocity(path * target.airspeed + wind);
        body.setAngularVelocity(Math::vec3(0.0f));

        for (PistonEngine& engine : mEngines)
            engine.setThrottle(mControls.throttle);
        for (u32 i = 0; i < mPropellers.size(); ++i)
        {
            Propeller& propeller = mPropellers[i].propeller;
            const f32 density = air.sampleAt(body.pointToWorld(propeller.description().position)).density;
            propeller.setAngularSpeed(
                balancedPropellerSpeed(i, propeller.axialInflow(body, air), density));
        }

        Math::vec3 force;
        Math::vec3 torque;
        computeForces(force, torque);
        force += target.gravity * body.mass();
        const Math::vec3 localForce = body.directionToLocal(force) / weight;
        const Math::vec3 localTorque = body.directionToLocal(torque) / (weight * lengthScale);
        r[0] = localForce.x;
        r[1] = localForce.y;
        r[2] = localForce.z;
        r[3] = localTorque.x;
        r[4] = localTorque.y;
        r[5] = localTorque.z;
    };

    f32 x[kTrimUnknowns] = {0.05f, 0.0f, 0.0f, 0.0f, 0.0f, solveThrottle ? 0.6f : -0.05f};
    f32 r[kTrimUnknowns];
    residual(x, r);
    f32 cost = sumOfSquares(r);
    f32 damping = 1.0e-4f;
    constexpr f32 kTolerance = 1.0e-4f;
    const f32 steps[kTrimUnknowns] = {1.0e-3f, 1.0e-3f, 1.0e-3f, 1.0e-3f, 1.0e-3f, 1.0e-3f};

    u32 iteration = 0;
    for (; iteration < 40 && largest(r) > kTolerance; ++iteration)
    {
        // Jacobian by forward differences, then Levenberg-Marquardt:
        // (J^T J + lambda diag) dx = -J^T r. The damping keeps an unusable
        // unknown (no rudder, say) from making the system singular.
        f32 jacobian[kTrimUnknowns][kTrimUnknowns];
        for (u32 j = 0; j < kTrimUnknowns; ++j)
        {
            f32 probe[kTrimUnknowns];
            for (u32 k = 0; k < kTrimUnknowns; ++k)
                probe[k] = x[k];
            f32 h = steps[j];
            if (probe[j] + h > upper[j])
                h = -h;
            probe[j] += h;
            f32 shifted[kTrimUnknowns];
            residual(probe, shifted);
            for (u32 i = 0; i < kTrimUnknowns; ++i)
                jacobian[i][j] = (shifted[i] - r[i]) / h;
        }

        bool improved = false;
        for (u32 attempt = 0; attempt < 8 && !improved; ++attempt)
        {
            f32 normal[kTrimUnknowns][kTrimUnknowns];
            f32 gradient[kTrimUnknowns];
            for (u32 a = 0; a < kTrimUnknowns; ++a)
            {
                gradient[a] = 0.0f;
                for (u32 i = 0; i < kTrimUnknowns; ++i)
                    gradient[a] -= jacobian[i][a] * r[i];
                for (u32 b = 0; b < kTrimUnknowns; ++b)
                {
                    f32 sum = 0.0f;
                    for (u32 i = 0; i < kTrimUnknowns; ++i)
                        sum += jacobian[i][a] * jacobian[i][b];
                    normal[a][b] = sum;
                }
                normal[a][a] += damping * (normal[a][a] + 1.0e-6f);
            }
            f32 delta[kTrimUnknowns];
            solveLinear(normal, gradient, delta);

            f32 candidate[kTrimUnknowns];
            for (u32 k = 0; k < kTrimUnknowns; ++k)
                candidate[k] = x[k] + delta[k];
            clampUnknowns(candidate);
            f32 candidateResidual[kTrimUnknowns];
            residual(candidate, candidateResidual);
            const f32 candidateCost = sumOfSquares(candidateResidual);
            if (isfinite(candidateCost) && candidateCost < cost)
            {
                for (u32 k = 0; k < kTrimUnknowns; ++k)
                {
                    x[k] = candidate[k];
                    r[k] = candidateResidual[k];
                }
                cost = candidateCost;
                damping = Max(damping * 0.3f, 1.0e-7f);
                improved = true;
            }
            else
            {
                damping *= 10.0f;
            }
        }
        if (!improved)
            break;
    }

    // Leave the body in the state of the final x, whatever the last probe was.
    residual(x, r);

    result.iterations = iteration;
    result.residual = largest(r);
    result.converged = result.residual <= kTolerance;
    result.angleOfAttack = x[0];
    result.sideslip = x[1];
    result.elevator = x[2];
    result.aileron = x[3];
    result.rudder = x[4];
    result.throttle = solveThrottle ? x[5] : target.throttle;
    result.climbAngle = solveThrottle ? target.climbAngle : x[5];
    result.rpm = mPropellers.empty() ? 0.0f : mPropellers[0].propeller.rpm();

    if (!result.converged)
    {
        body.setPosition(savedPosition);
        body.setOrientation(savedOrientation);
        body.setVelocity(savedVelocity);
        body.setAngularVelocity(savedAngularVelocity);
        mControls = savedControls;
        for (u32 i = 0; i < mPropellers.size(); ++i)
            mPropellers[i].propeller.setAngularSpeed(savedSpeeds[i]);
        return result;
    }

    // The pilot's inputs are left at zero: the trims hold the aircraft there.
    ControlInputs controls;
    controls.flaps = target.flaps;
    controls.throttle = result.throttle;
    controls.elevatorTrim = result.elevator;
    controls.aileronTrim = result.aileron;
    controls.rudderTrim = result.rudder;
    setControls(controls);
    return result;
}

} // namespace kinetix::Physics
