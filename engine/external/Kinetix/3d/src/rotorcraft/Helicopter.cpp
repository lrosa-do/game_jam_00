#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>
#include <kinetix/core/Log.h>

#include "rotorcraft/Helicopter.h"

#include "dynamics/PhysicsWorld.h"
#include "dynamics/RigidBody.h"

#include <math.h>

namespace kinetix::Physics
{

namespace
{
constexpr f32 kPi = 3.14159265358979f;
constexpr f32 kRadiansToRpm = 60.0f / (2.0f * kPi);
constexpr u32 kTrimUnknowns = 6;
// Contacts are probed from this far above their points, so one that has sunk into the ground still finds it.
constexpr f32 kContactProbeHeight = 1.0f;
constexpr f32 kAglRange = 1000.0f;

const Math::vec3 kForward(0.0f, 0.0f, 1.0f);
const Math::vec3 kUp(0.0f, 1.0f, 0.0f);
const Math::vec3 kRight(-1.0f, 0.0f, 0.0f);

bool finiteVec(const Math::vec3& v)
{
    return isfinite(v.x) && isfinite(v.y) && isfinite(v.z);
}

f32 smoothstep01(f32 x)
{
    const f32 t = Clamp(x, 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

// Gaussian elimination with partial pivoting; a vanished pivot leaves its
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

Helicopter::Helicopter()
{
    // The transmission's inertia follows the rotors it is given; this is the default rotors'.
    RotorTransmission::Description drive;
    drive.mainRotorInertia = mMainRotor.polarInertia();
    mTransmission.setDescription(drive);
    setRatedRotorSpeed(mRatedRotorSpeed);
}

void Helicopter::setGround(const PhysicsWorld* world, u32 ignoredBody)
{
    mGroundWorld = world;
    mGroundIgnoredBody = ignoredBody;
    mGroundQuery = nullptr;
    mGroundUserData = nullptr;
}

bool Helicopter::probeGround(const Math::vec3& origin, f32 maxDistance, GroundHit& hit) const
{
    const Math::vec3 down(0.0f, -1.0f, 0.0f);
    if (mGroundQuery)
        return mGroundQuery(origin, down, maxDistance, hit, mGroundUserData);
    if (mGroundWorld)
    {
        Ray ray;
        ray.origin = origin;
        ray.direction = down;
        QueryFilter filter;
        filter.ignoredBody = mGroundIgnoredBody;
        WorldRayHit worldHit;
        if (!mGroundWorld->raycast(ray, maxDistance, filter, worldHit))
            return false;
        hit.distance = worldHit.distance;
        hit.point = worldHit.point;
        hit.normal = worldHit.normal;
        return true;
    }
    const f32 distance = origin.y - mGroundHeight;
    if (distance > maxDistance)
        return false;
    hit.distance = distance;
    hit.point = Math::vec3(origin.x, mGroundHeight, origin.z);
    hit.normal = kUp;
    return true;
}

void Helicopter::setRatedRotorSpeed(f32 omega)
{
    mRatedRotorSpeed = isfinite(omega) ? Max(omega, 1.0f) : 40.0f;
    RotorTransmission::Description drive = mTransmission.description();
    drive.engineRatio = mEngine.ratedTurbineSpeed() / mRatedRotorSpeed;
    mTransmission.setDescription(drive);
}

u32 Helicopter::addWing(const Wing& wing)
{
    mWings.push_back(wing);
    return static_cast<u32>(mWings.size() - 1);
}

u32 Helicopter::addDragElement(const DragElement& element)
{
    DragElement stored = element;
    stored.dragAreas = Math::max(element.dragAreas, Math::vec3(0.0f));
    mDragElements.push_back(stored);
    return static_cast<u32>(mDragElements.size() - 1);
}

u32 Helicopter::addSkidContact(const SkidContact& contact)
{
    ContactEntry entry;
    entry.contact = contact;
    SkidContact& c = entry.contact;
    if (!finiteVec(c.position))
        c.position = Math::vec3(0.0f);
    c.stiffness = isfinite(c.stiffness) ? Max(c.stiffness, 0.0f) : 0.0f;
    c.damping = isfinite(c.damping) ? Max(c.damping, 0.0f) : 0.0f;
    c.tangentialStiffness = isfinite(c.tangentialStiffness) ? Max(c.tangentialStiffness, 1.0f) : 1.0f;
    c.tangentialDamping = isfinite(c.tangentialDamping) ? Max(c.tangentialDamping, 0.0f) : 0.0f;
    c.friction = isfinite(c.friction) ? Max(c.friction, 0.0f) : 0.0f;
    mContacts.push_back(entry);
    return static_cast<u32>(mContacts.size() - 1);
}

void Helicopter::setControls(const HelicopterControls& controls)
{
    auto unit = [](f32 value, f32 lo)
    {
        return isfinite(value) ? Clamp(value, lo, 1.0f) : 0.0f;
    };
    mControls.collective = unit(controls.collective, 0.0f);
    mControls.cyclicLongitudinal = unit(controls.cyclicLongitudinal, -1.0f);
    mControls.cyclicLateral = unit(controls.cyclicLateral, -1.0f);
    mControls.pedals = unit(controls.pedals, -1.0f);
    mControls.throttle = unit(controls.throttle, 0.0f);
    mControls.engineOn = controls.engineOn;
}

void Helicopter::startRunning()
{
    mTransmission.setRotorSpeed(mRatedRotorSpeed);
    mEngine.setCollective(mControls.collective);
    mEngine.setGasGenerator(mEngine.description().idleGasGenerator + 0.1f);
    mEngine.setRunning(true);
    mControls.engineOn = true;
    mMainRotor.reset();
    mTailRotor.reset();
}

void Helicopter::shutDown()
{
    mEngine.setGasGenerator(0.0f);
    mEngine.setRunning(false);
    mTransmission.setRotorSpeed(0.0f);
    mControls.engineOn = false;
    mMainRotor.reset();
    mTailRotor.reset();
}

void Helicopter::mixControls()
{
    const HelicopterRigging& r = mRigging;
    f32 longitudinal = mControls.cyclicLongitudinal;
    f32 lateral = mControls.cyclicLateral;
    f32 pedals = mControls.pedals;
    if (mSas.enabled && mBody)
    {
        const Math::vec3 rates = mBody->directionToLocal(mBody->angularVelocity());
        const f32 p = Math::dot(rates, kForward);
        const f32 q = Math::dot(rates, kRight);
        const f32 yaw = -Math::dot(rates, kUp);
        const f32 limit = Max(mSas.authority, 0.0f);
        // A nose-up rate is met with forward stick, a roll to the right with left stick, a yaw to the right with left pedal.
        longitudinal += Clamp(mSas.pitch * q, -limit, limit);
        lateral += Clamp(-mSas.roll * p, -limit, limit);
        pedals += Clamp(-mSas.yaw * yaw, -limit, limit);
    }
    longitudinal = Clamp(longitudinal, -1.0f, 1.0f);
    lateral = Clamp(lateral, -1.0f, 1.0f);
    pedals = Clamp(pedals, -1.0f, 1.0f);

    const f32 collective = Math::mix(r.collectiveMin, r.collectiveMax, mControls.collective);
    mMainRotor.setControls(collective, longitudinal * r.longitudinalCyclic, lateral * r.lateralCyclic);
    mTailRotor.setControls(r.tailCollective + pedals * r.tailCollectivePerPedal, 0.0f, 0.0f);

    mEngine.setThrottle(mControls.throttle);
    mEngine.setCollective(mControls.collective);
    mEngine.setRunning(mControls.engineOn);
}

RotorOperatingPoint Helicopter::operatingPoint(const Rotor& rotor, f32 omega, f32 groundDistance) const
{
    const RigidBody& body = *mBody;
    const Atmosphere& air = atmosphere();
    const Math::vec3 hub = body.pointToWorld(rotor.description().position);
    RotorOperatingPoint point;
    point.angularSpeed = omega;
    point.velocity = body.directionToLocal(body.velocityAtPoint(hub) - air.windAt(hub));
    point.angularVelocity = body.directionToLocal(body.angularVelocity());
    point.density = air.sampleAt(hub).density;
    point.groundDistance = groundDistance;
    return point;
}

Math::vec3 Helicopter::wakeVelocity(const Math::vec3& bodyPoint) const
{
    const Rotor::Description& d = mMainRotor.description();
    const RotorLoads& loads = mMainRotor.loads();
    const f32 radius = d.radius;
    const Math::vec3 shaft = mMainRotor.shaftAxis();
    const Math::vec3 relative = bodyPoint - d.position;
    const f32 below = -Math::dot(relative, shaft);
    if (below <= 0.0f || loads.inducedVelocity == 0.0f)
        return Math::vec3(0.0f);

    const RotorOperatingPoint point = operatingPoint(mMainRotor, 0.0f, -1.0f);
    Math::vec3 inPlane = point.velocity - shaft * Math::dot(point.velocity, shaft);
    const f32 speed = Math::length(inPlane);
    const Math::vec3 downstream = speed > 1.0e-3f ? inPlane * (-1.0f / speed) : Math::vec3(0.0f);
    const f32 skew = Clamp(loads.wakeSkew, 0.0f, 1.4f);
    const Math::vec3 offset = downstream * (below * tanf(skew));
    const Math::vec3 lateral = relative + shaft * below - offset;
    const f32 distance = Math::length(lateral);
    const f32 inside = 1.0f - smoothstep01((distance - 0.7f * radius) / (0.6f * radius));
    if (inside <= 0.0f)
        return Math::vec3(0.0f);
    // v at the disc to 2v in the far wake (actuator disc axial velocity distribution).
    const f32 wake = loads.inducedVelocity * (1.0f + below / sqrtf(radius * radius + below * below));
    return shaft * (-wake * inside);
}

void Helicopter::contactForces(bool apply, Math::vec3& force, Math::vec3& torque)
{
    RigidBody& body = *mBody;
    for (ContactEntry& entry : mContacts)
    {
        const SkidContact& c = entry.contact;
        SkidContactState s = entry.state;
        const Math::vec3 point = body.pointToWorld(c.position);
        s.point = point;
        s.inContact = false;
        s.normalForce = 0.0f;
        s.penetration = 0.0f;
        s.sliding = false;

        GroundHit hit;
        const Math::vec3 origin = point + kUp * kContactProbeHeight;
        const bool found = probeGround(origin, kContactProbeHeight + 0.5f, hit);
        f32 depth = 0.0f;
        Math::vec3 normal = kUp;
        if (found)
        {
            normal = Math::dot(hit.normal, hit.normal) > 1.0e-6f ? Math::normalize(hit.normal) : kUp;
            depth = Math::dot(hit.point - point, normal);
        }
        if (!found || depth <= 0.0f)
        {
            s.anchored = false;
            if (apply)
                entry.state = s;
            continue;
        }

        const Math::vec3 velocity = body.velocityAtPoint(point);
        const f32 normalSpeed = Math::dot(velocity, normal);
        const f32 normalForce = Max(c.stiffness * depth - c.damping * normalSpeed, 0.0f);

        // Stiction: a spring to where the contact first touched, sliding (dragging the anchor) once it would need more than mu N.
        const Math::vec3 onGround = point + normal * depth;
        if (!s.anchored)
        {
            s.anchor = onGround;
            s.anchored = true;
        }
        Math::vec3 stretch = onGround - s.anchor;
        stretch -= normal * Math::dot(stretch, normal);
        const Math::vec3 slip = velocity - normal * normalSpeed;
        Math::vec3 tangential = stretch * (-c.tangentialStiffness) - slip * c.tangentialDamping;
        const f32 limit = c.friction * normalForce;
        const f32 magnitude = Math::length(tangential);
        if (magnitude > limit)
        {
            tangential = magnitude > 1.0e-9f ? tangential * (limit / magnitude) : Math::vec3(0.0f);
            // Anchor moved so the spring alone would carry the sliding force: it follows the contact instead of storing the slide.
            s.anchor = onGround + tangential * (1.0f / c.tangentialStiffness);
            s.sliding = true;
        }

        const Math::vec3 f = normal * normalForce + tangential;
        const Math::vec3 t = Math::cross(point - body.position(), f);
        if (apply)
        {
            body.addForce(f);
            body.addTorque(t);
        }
        force += f;
        torque += t;
        s.inContact = true;
        s.normalForce = normalForce;
        s.penetration = depth;
        if (apply)
            entry.state = s;
    }
}

void Helicopter::accumulate(Evaluation mode, f32 step, Math::vec3& force, Math::vec3& torque)
{
    force = Math::vec3(0.0f);
    torque = Math::vec3(0.0f);
    RigidBody& body = *mBody;
    const Atmosphere& air = atmosphere();
    const bool apply = mode == Evaluation::Advance;

    mixControls();

    {
        const Math::vec3 hub = body.pointToWorld(mMainRotor.description().position);
        GroundHit hit;
        mHubGroundDistance = probeGround(hub, 4.0f * mMainRotor.description().radius, hit)
                                 ? Max(hit.distance, 0.0f)
                                 : -1.0f;
    }

    const f32 omega = mTransmission.rotorSpeed();
    const f32 tailOmega = mTransmission.tailRotorSpeed();
    const RotorOperatingPoint mainPoint = operatingPoint(mMainRotor, omega, mHubGroundDistance);
    const RotorOperatingPoint tailPoint = operatingPoint(mTailRotor, tailOmega, -1.0f);

    RotorLoads main;
    RotorLoads tail;
    switch (mode)
    {
    case Evaluation::Advance:
        main = mMainRotor.advance(mainPoint, step);
        if (mHasTailRotor)
            tail = mTailRotor.advance(tailPoint, step);
        break;
    case Evaluation::Current:
        main = mMainRotor.evaluate(mainPoint);
        if (mHasTailRotor)
            tail = mTailRotor.evaluate(tailPoint);
        break;
    case Evaluation::Steady:
        main = mMainRotor.solve(mainPoint);
        if (mHasTailRotor)
            tail = mTailRotor.solve(tailPoint);
        break;
    }

    // Shaft torques: what the drive delivers to each rotor. Stepping the drive includes the
    // rotors' acceleration; otherwise speeds are taken as steady and the shafts carry just the aerodynamic torque.
    f32 mainShaft = main.torque;
    f32 tailShaft = tail.torque;
    if (apply)
    {
        const f32 density = air.sampleAt(body.position()).density;
        const TransmissionState& drive =
            mTransmission.step(step, &mEngine, density, main.torque, mHasTailRotor ? tail.torque : 0.0f);
        mEngine.step(step, drive.turbineSpeed, density);
        mainShaft = drive.mainShaftTorque;
        tailShaft = mHasTailRotor ? drive.tailShaftTorque : 0.0f;
    }

    const Math::vec3 centre = body.position();
    auto addRotor = [&](const Rotor& rotor, const RotorLoads& loads, f32 shaftTorque)
    {
        const Rotor::Description& d = rotor.description();
        const Math::vec3 hub = body.pointToWorld(d.position);
        const Math::vec3 f = body.directionToWorld(loads.force);
        const Math::vec3 reaction = body.directionToWorld(rotor.shaftAxis()) * (-d.rotation * shaftTorque);
        const Math::vec3 t = Math::cross(hub - centre, f) + body.directionToWorld(loads.moment) + reaction;
        if (apply)
        {
            body.addForce(f);
            body.addTorque(t);
        }
        force += f;
        torque += t;
    };
    addRotor(mMainRotor, main, mainShaft);
    if (mHasTailRotor)
        addRotor(mTailRotor, tail, tailShaft);

    for (Wing& wing : mWings)
    {
        if (mWakeOnAirframe)
        {
            const Wing::Description& d = wing.description();
            const Math::vec3 reference = d.mirrored ? d.root : (d.root + d.tip) * 0.5f;
            wing.setInducedFlow(wakeVelocity(reference));
        }
        // The wing solves its induced angle by a Newton step from the last evaluation's, exact in
        // attached flow; in trim the stabiliser can sit in the stalled wake and gets a few to converge.
        if (mode == Evaluation::Steady)
        {
            f32 previous = wing.state().inducedAngle;
            for (u32 pass = 0; pass < 20; ++pass)
            {
                const f32 angle = wing.computeForces(body, air).inducedAngle;
                if (fabsf(angle - previous) < 1.0e-7f)
                    break;
                previous = angle;
            }
        }
        const WingState& state = wing.computeForces(body, air);
        if (apply)
        {
            body.addForce(state.force);
            body.addTorque(state.torque);
        }
        force += state.force;
        torque += state.torque;
    }

    const f32 density = air.sampleAt(body.position()).density;
    for (const DragElement& element : mDragElements)
    {
        const Math::vec3 point = body.pointToWorld(element.position);
        Math::vec3 airVelocity = air.windAt(point) - body.velocityAtPoint(point);
        if (mWakeOnAirframe)
            airVelocity += body.directionToWorld(wakeVelocity(element.position));
        const Math::vec3 flow = body.directionToLocal(airVelocity);
        const f32 speed = Math::length(flow);
        const Math::vec3 f = body.directionToWorld(element.dragAreas * flow * (0.5f * density * speed));
        const Math::vec3 t = Math::cross(point - centre, f);
        if (apply)
        {
            body.addForce(f);
            body.addTorque(t);
        }
        force += f;
        torque += t;
    }

    if (mode != Evaluation::Steady)
        contactForces(apply, force, torque);

    updateTelemetry(force, torque);
    HelicopterTelemetry& t = mTelemetry;
    t.thrust = main.thrust;
    t.tailRotorThrust = tail.thrust;
    t.mainRotorTorque = main.torque;
    t.tailRotorTorque = tail.torque;
    t.powerRequired = main.power + tail.power;
    t.inducedVelocity = main.inducedVelocity;
    t.inflowRatio = main.inflowRatio;
    t.advanceRatio = main.advanceRatio;
    t.coning = main.coning;
    t.longitudinalFlapping = main.longitudinalFlapping;
    t.lateralFlapping = main.lateralFlapping;
    t.discTiltForward = main.discTiltForward;
    t.discTiltRight = main.discTiltRight;
    t.vortexRing = main.vortexRing;
    t.groundEffectFactor = main.groundEffectFactor;
}

void Helicopter::applyForces(f32 step)
{
    if (!mBody)
        return;
    Math::vec3 force;
    Math::vec3 torque;
    accumulate(Evaluation::Advance, step, force, torque);
}

void Helicopter::computeForces(Math::vec3& force, Math::vec3& torque)
{
    force = Math::vec3(0.0f);
    torque = Math::vec3(0.0f);
    if (!mBody)
        return;
    accumulate(Evaluation::Current, 0.0f, force, torque);
}

void Helicopter::stepCallback(f32 step, void* userData)
{
    if (userData)
        static_cast<Helicopter*>(userData)->applyForces(step);
}

void Helicopter::updateTelemetry(const Math::vec3& force, const Math::vec3& torque)
{
    const RigidBody& body = *mBody;
    const Atmosphere& air = atmosphere();
    const AtmosphereState state = air.sampleAt(body.position());

    HelicopterTelemetry t;
    t.force = force;
    t.torque = torque;
    t.density = state.density;
    t.altitude = air.altitudeAt(body.position());
    t.verticalSpeed = body.velocity().y;
    const Math::vec3 horizontal(body.velocity().x, 0.0f, body.velocity().z);
    t.groundSpeed = Math::length(horizontal);

    const Math::vec3 motion = body.directionToLocal(body.velocity() - air.windAt(body.position()));
    const f32 speed = Math::length(motion);
    t.trueAirspeed = speed;
    t.airVelocity = Math::vec3(Math::dot(motion, kForward), Math::dot(motion, kRight), Math::dot(motion, kUp));
    if (speed > 1.0e-3f)
    {
        t.angleOfAttack = atan2f(-Math::dot(motion, kUp), Math::dot(motion, kForward));
        t.sideslip = asinf(Clamp(Math::dot(motion, kRight) / speed, -1.0f, 1.0f));
    }

    const Math::vec3 forwardWorld = body.directionToWorld(kForward);
    const Math::vec3 upWorld = body.directionToWorld(kUp);
    const Math::vec3 rightWorld = body.directionToWorld(kRight);
    t.pitch = asinf(Clamp(forwardWorld.y, -1.0f, 1.0f));
    t.bank = atan2f(-rightWorld.y, upWorld.y);
    t.heading = atan2f(-forwardWorld.x, forwardWorld.z);
    const Math::vec3 rates = body.directionToLocal(body.angularVelocity());
    t.bodyRates = Math::vec3(Math::dot(rates, kForward), Math::dot(rates, kRight), -Math::dot(rates, kUp));

    const f32 omega = mTransmission.rotorSpeed();
    const TransmissionState& drive = mTransmission.state();
    t.rotorRpm = omega * kRadiansToRpm;
    t.rotorSpeedPercent = 100.0f * omega / mRatedRotorSpeed;
    t.tailRotorRpm = mTransmission.tailRotorSpeed() * kRadiansToRpm;
    t.turbineSpeedPercent = 100.0f * mTransmission.turbineSpeed() / mEngine.ratedTurbineSpeed();
    t.gasGeneratorPercent = 100.0f * mEngine.gasGenerator();
    t.enginePower = drive.enginePower;
    t.fuelFlow = mEngine.fuelFlow(state.density);
    t.freewheelEngaged = mTransmission.engaged();

    f32 gearDepth = 0.0f;
    for (const ContactEntry& entry : mContacts)
        gearDepth = Max(gearDepth, -Math::dot(body.directionToWorld(entry.contact.position), kUp));
    GroundHit hit;
    t.heightAboveGround =
        probeGround(body.position(), kAglRange, hit) ? Max(hit.distance - gearDepth, 0.0f) : kAglRange;
    for (const ContactEntry& entry : mContacts)
    {
        if (entry.state.inContact)
        {
            t.onGround = true;
            t.gearLoad += entry.state.normalForce;
        }
    }
    mTelemetry = t;
}

HelicopterTrimResult Helicopter::trim(const HelicopterTrimTarget& target)
{
    HelicopterTrimResult result;
    if (!mBody || !mBody->hasFiniteMass() || !isfinite(target.airspeed) || target.airspeed < 0.0f ||
        !isfinite(target.climbRate) || !finiteVec(target.gravity))
    {
        Log::error("Helicopter: trim needs a dynamic body and a finite, non-negative airspeed");
        return result;
    }
    RigidBody& body = *mBody;
    const Atmosphere& air = atmosphere();

    const Math::vec3 savedPosition = body.position();
    const Math::quat savedOrientation = body.orientation();
    const Math::vec3 savedVelocity = body.velocity();
    const Math::vec3 savedAngularVelocity = body.angularVelocity();
    const HelicopterControls savedControls = mControls;
    const StabilityAugmentation savedSas = mSas;
    const Rotor savedMain = mMainRotor;
    const Rotor savedTail = mTailRotor;
    const RotorTransmission savedTransmission = mTransmission;
    const TurboshaftEngine savedEngine = mEngine;

    // The rate damping sees no rates in trim; off so it cannot matter.
    mSas.enabled = false;
    mTransmission.setRotorSpeed(mRatedRotorSpeed);

    Math::vec3 heading = body.directionToWorld(kForward);
    heading.y = 0.0f;
    if (Math::dot(heading, heading) < 1.0e-6f)
        heading = kForward;
    heading = Math::normalize(heading);
    const Math::vec3 worldRight = Math::cross(heading, kUp);
    const Math::vec3 wind = air.windAt(savedPosition);
    const f32 weight = body.mass() * Math::length(target.gravity);
    const f32 lengthScale = Max(mMainRotor.description().radius, 1.0f);

    const f32 lower[kTrimUnknowns] = {0.0f, -1.0f, -1.0f, -1.0f, -0.6f, -0.6f};
    const f32 upper[kTrimUnknowns] = {1.0f, 1.0f, 1.0f, 1.0f, 0.6f, 0.6f};

    auto residual = [&](const f32 x[kTrimUnknowns], f32 r[kTrimUnknowns])
    {
        HelicopterControls controls = mControls;
        controls.collective = x[0];
        controls.cyclicLongitudinal = x[1];
        controls.cyclicLateral = x[2];
        controls.pedals = x[3];
        controls.engineOn = true;
        controls.throttle = 1.0f;
        setControls(controls);

        const f32 pitch = x[4];
        const f32 bank = x[5];
        const Math::vec3 forward1 = heading * cosf(pitch) + kUp * sinf(pitch);
        const Math::vec3 up1 = kUp * cosf(pitch) - heading * sinf(pitch);
        const Math::vec3 up2 = up1 * cosf(bank) + worldRight * sinf(bank);
        const Math::vec3 right2 = worldRight * cosf(bank) - up1 * sinf(bank);
        const Math::mat3 worldBasis(forward1, up2, right2);
        const Math::mat3 bodyBasis(kForward, kUp, kRight);
        body.setOrientation(Math::normalize(Math::quat_cast(worldBasis * Math::transpose(bodyBasis))));
        body.setPosition(savedPosition);
        body.setVelocity(heading * target.airspeed + kUp * target.climbRate + wind);
        body.setAngularVelocity(Math::vec3(0.0f));

        Math::vec3 force;
        Math::vec3 torque;
        accumulate(Evaluation::Steady, 0.0f, force, torque);
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

    f32 x[kTrimUnknowns] = {0.6f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    f32 r[kTrimUnknowns];
    residual(x, r);
    f32 cost = sumOfSquares(r);
    f32 damping = 1.0e-3f;
    constexpr f32 kTolerance = 2.0e-4f;
    constexpr f32 kProbe = 1.0e-3f;

    u32 iteration = 0;
    for (; iteration < 40 && largest(r) > kTolerance; ++iteration)
    {
        f32 jacobian[kTrimUnknowns][kTrimUnknowns];
        for (u32 j = 0; j < kTrimUnknowns; ++j)
        {
            f32 probe[kTrimUnknowns];
            for (u32 k = 0; k < kTrimUnknowns; ++k)
                probe[k] = x[k];
            f32 h = kProbe;
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
                candidate[k] = Clamp(x[k] + delta[k], lower[k], upper[k]);
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

    residual(x, r);
    result.iterations = iteration;
    result.residual = largest(r);
    result.converged = result.residual <= kTolerance;
    result.collective = x[0];
    result.cyclicLongitudinal = x[1];
    result.cyclicLateral = x[2];
    result.pedals = x[3];
    result.pitch = x[4];
    result.bank = x[5];
    result.mainRotorPower = mMainRotor.loads().power;
    result.tailRotorPower = mHasTailRotor ? mTailRotor.loads().power : 0.0f;
    const RotorTransmission::Description& drive = mTransmission.description();
    const f32 friction = drive.frictionTorque + drive.frictionPerSpeed * mRatedRotorSpeed;
    result.powerRequired = result.mainRotorPower + result.tailRotorPower + friction * mRatedRotorSpeed;
    result.thrust = mMainRotor.loads().thrust;
    result.longitudinalFlapping = mMainRotor.loads().longitudinalFlapping;
    result.lateralFlapping = mMainRotor.loads().lateralFlapping;

    mSas = savedSas;
    if (!result.converged)
    {
        body.setPosition(savedPosition);
        body.setOrientation(savedOrientation);
        body.setVelocity(savedVelocity);
        body.setAngularVelocity(savedAngularVelocity);
        mControls = savedControls;
        mMainRotor = savedMain;
        mTailRotor = savedTail;
        mTransmission = savedTransmission;
        mEngine = savedEngine;
        return result;
    }

    // Engine at the gas generator that delivers this power through the gearbox at the rated
    // turbine speed (free-turbine line gives the full gas power), governor settled.
    const f32 density = air.sampleAt(body.position()).density;
    const TurboshaftEngine::Description& e = mEngine.description();
    const f32 available = e.maxPower * powf(density / Atmosphere::SeaLevelDensity, 0.8f);
    // Gas power: shaft power through the gearbox plus the power turbine's windage at rated speed.
    const f32 gas = Max(result.powerRequired, 0.0f) / drive.efficiency + e.windage * e.maxPower;
    const f32 ng = e.zeroPowerGasGenerator +
                   (1.0f - e.zeroPowerGasGenerator) * sqrtf(Clamp(gas / Max(available, 1.0f), 0.0f, 2.0f));
    mEngine.setCollective(result.collective);
    mEngine.setGasGenerator(Max(ng, e.idleGasGenerator));
    mEngine.setRunning(true);
    mTransmission.setRotorSpeed(mRatedRotorSpeed);
    return result;
}

} // namespace kinetix::Physics
