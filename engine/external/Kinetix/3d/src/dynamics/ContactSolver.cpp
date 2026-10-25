#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>
#include <kinetix/core/Log.h>

#include "dynamics/ContactSolver.h"

#include "dynamics/Joint.h"
#include "dynamics/RigidBody.h"

#include <math.h>

namespace kinetix::Physics
{

namespace
{
// Velocity produced by a unit impulse along `direction` at these two points
// (linear part plus what the inertia tensors let the torque do); dividing by it
// turns a wanted velocity change into the impulse.
f32 effectiveMass(const RigidBody& a, const RigidBody& b, const Math::vec3& armA,
                  const Math::vec3& armB, const Math::vec3& direction)
{
    const Math::vec3 angularA = Math::cross(armA, direction);
    const Math::vec3 angularB = Math::cross(armB, direction);
    return a.inverseMass() + b.inverseMass() +
           Math::dot(direction, Math::cross(a.inverseInertiaTensorWorld() * angularA, armA)) +
           Math::dot(direction, Math::cross(b.inverseInertiaTensorWorld() * angularB, armB));
}

Math::vec3 relativeVelocity(const RigidBody& a, const RigidBody& b, const Math::vec3& armA,
                           const Math::vec3& armB)
{
    return (b.velocity() + Math::cross(b.angularVelocity(), armB)) -
           (a.velocity() + Math::cross(a.angularVelocity(), armA));
}

// The normal points from A to B: B takes the impulse, A its opposite.
//
// Deliberately NOT RigidBody::applyImpulseAtPoint, which wakes the body: the
// solver touches every resting contact every step, which would reset the sleep
// counter of a settled stack forever. Waking belongs to the moment a contact is
// NEW, which PhysicsWorld does from its pair cache.
void applyOne(RigidBody& body, const Math::vec3& impulse, const Math::vec3& arm)
{
    if (!body.isDynamic())
        return;
    body.setVelocity(body.velocity() + impulse * body.inverseMass());
    body.setAngularVelocity(body.angularVelocity() +
                            body.inverseInertiaTensorWorld() * Math::cross(arm, impulse));
}

void applyPair(RigidBody& a, RigidBody& b, const Math::vec3& impulse, const Math::vec3& armA,
               const Math::vec3& armB)
{
    applyOne(a, -impulse, armA);
    applyOne(b, impulse, armB);
}

// Whether a constraint has anything to move this step: at least one dynamic,
// awake body. Constraints whose dynamic bodies are all asleep are skipped,
// otherwise the position pass would shift a sleeper out of its settled overlap
// and the velocity pass leave speeds it never integrates. One awake and one
// sleeping body is still solved: the world joins them into one island.
bool active(const RigidBody* a, const RigidBody* b)
{
    return (a && a->isDynamic() && a->awake()) || (b && b->isDynamic() && b->awake());
}

bool active(const Contact& contact)
{
    return contact.a && contact.b && active(contact.a, contact.b);
}

bool active(const Joint* joint)
{
    return joint && joint->enabled() && active(joint->bodyA(), joint->bodyB());
}

bool iterative(const Joint* joint)
{
    return active(joint) && !joint->solvedByArticulation();
}
} // namespace

void ContactSolver::warmStart(Contact* contacts, u32 count)
{
    for (u32 c = 0; c < count; ++c)
    {
        Contact& contact = contacts[c];
        if (!active(contact))
            continue;
        ContactManifold& manifold = contact.manifold;
        for (u32 i = 0; i < manifold.count; ++i)
        {
            ContactPoint& point = manifold.points[i];
            // Last step's answer, applied before the first iteration; otherwise a
            // tall stack sinks by whatever the iterations could not recover.
            const Math::vec3 impulse = manifold.normal * point.normalImpulse +
                                      manifold.tangent[0] * point.tangentImpulse[0] +
                                      manifold.tangent[1] * point.tangentImpulse[1];
            applyPair(*contact.a, *contact.b, impulse, point.armA, point.armB);
        }
    }
}

void ContactSolver::solveVelocity(Contact* contacts, u32 count)
{
    for (u32 c = 0; c < count; ++c)
    {
        Contact& contact = contacts[c];
        if (!active(contact) || contact.articulated)
            continue;
        RigidBody& a = *contact.a;
        RigidBody& b = *contact.b;
        ContactManifold& manifold = contact.manifold;

        for (u32 i = 0; i < manifold.count; ++i)
        {
            ContactPoint& point = manifold.points[i];
            const Math::vec3& armA = point.armA;
            const Math::vec3& armB = point.armB;

            // Friction first, against the normal impulse the last iteration
            // settled on (a cone needs a normal force to be a fraction of; this
            // iteration's would let the two chase each other).
            const f32 maxFriction = contact.friction * point.normalImpulse;
            for (u32 t = 0; t < 2; ++t)
            {
                if (point.tangentK[t] <= 0.0f)
                    continue;
                const Math::vec3& tangent = manifold.tangent[t];
                const f32 speed = Math::dot(relativeVelocity(a, b, armA, armB), tangent);
                const f32 wanted = -speed / point.tangentK[t];
                const f32 previous = point.tangentImpulse[t];
                const f32 total = Math::clamp(previous + wanted, -maxFriction, maxFriction);
                point.tangentImpulse[t] = total;
                applyPair(a, b, tangent * (total - previous), armA, armB);
            }

            if (point.normalK <= 0.0f)
                continue;
            const f32 separation =
                Math::dot(relativeVelocity(a, b, armA, armB), manifold.normal);
            // The bias is the separation speed a bounce should end at; zero for a
            // resting contact.
            const f32 wanted = -(separation - point.velocityBias) / point.normalK;

            // Clamp the ACCUMULATED impulse, not each increment: a later contact
            // may need to take back part of an earlier iteration's, and only a
            // running total can be reduced without going negative.
            const f32 previous = point.normalImpulse;
            const f32 total = Math::max(previous + wanted, 0.0f);
            point.normalImpulse = total;
            applyPair(a, b, manifold.normal * (total - previous), armA, armB);
        }
    }
}

void ContactSolver::solvePosition(Contact* contacts, u32 count)
{
    for (u32 c = 0; c < count; ++c)
    {
        Contact& contact = contacts[c];
        if (!active(contact) || contact.articulated)
            continue;
        RigidBody& a = *contact.a;
        RigidBody& b = *contact.b;
        ContactManifold& manifold = contact.manifold;
        for (u32 i = 0; i < manifold.count; ++i)
        {
            ContactPoint& point = manifold.points[i];
            // Where the surfaces are NOW. Overlap up to the slop is left alone
            // (chasing zero makes resting bodies make and lose contact every
            // frame); deeper than maxPenetration counts as maxPenetration.
            const Math::vec3 pointA = a.pointToWorld(point.localA);
            const Math::vec3 pointB = b.pointToWorld(point.localB);
            const f32 separation =
                Math::max(Math::dot(pointB - pointA, manifold.normal) + mSettings.slop,
                          -mSettings.maxPenetration);
            if (separation >= 0.0f)
                continue;

            const Math::vec3 middle = (pointA + pointB) * 0.5f;
            const Math::vec3 armA = middle - a.position();
            const Math::vec3 armB = middle - b.position();
            const f32 mass = effectiveMass(a, b, armA, armB, manifold.normal);
            if (mass <= 0.0f)
                continue;
            const Math::vec3 impulse =
                manifold.normal * (-separation * mSettings.baumgarte / mass);

            // Moved directly rather than through a bias impulse, which adds
            // velocity the bodies keep and makes a resting stack gain energy.
            a.applyPositionImpulseAtPoint(-impulse, middle);
            b.applyPositionImpulseAtPoint(impulse, middle);
        }
    }
}

void ContactSolver::solve(Contact* contacts, u32 count, Joint* const* joints, u32 jointCount,
                          f32 duration)
{
    if (duration <= 0.0f || !isfinite(duration))
        return;

    // Every enabled joint is set up, sleeping or not: setup() is where a motor
    // wakes its bodies, and only after that is it known which joints and
    // contacts are solved, so this precedes contact preparation.
    for (u32 i = 0; i < jointCount; ++i)
        if (joints[i] && joints[i]->enabled())
            joints[i]->setup(duration);

    // Tangents are rebuilt from the normal rather than trusted from the
    // narrowphase: a carried-over manifold's normal may have turned.
    //
    // No waking here: a contact exists every step a stack rests, so nothing
    // would ever sleep. Waking belongs to whoever notices a contact is NEW.
    for (u32 c = 0; c < count; ++c)
    {
        Contact& contact = contacts[c];
        if (!active(contact))
            continue;
        ContactManifold& manifold = contact.manifold;
        manifold.buildTangents();
        for (u32 i = 0; i < manifold.count; ++i)
        {
            ContactPoint& point = manifold.points[i];
            const Math::vec3 half = manifold.normal * (0.5f * point.penetration);
            point.localA = contact.a->pointToLocal(point.position + half);
            point.localB = contact.b->pointToLocal(point.position - half);
            point.armA = point.position - contact.a->position();
            point.armB = point.position - contact.b->position();
            const f32 normalK =
                effectiveMass(*contact.a, *contact.b, point.armA, point.armB, manifold.normal);
            point.normalK = normalK;
            for (u32 t = 0; t < 2; ++t)
            {
                const f32 tangentK = effectiveMass(*contact.a, *contact.b, point.armA, point.armB,
                                                   manifold.tangent[t]);
                point.tangentK[t] = tangentK;
            }
        }

        // The target separation speed of every point is decided HERE, from the
        // approach speed before any impulse is applied.
        //
        // A point still apart (negative penetration, inside the margin) is
        // speculative: the bodies may close the gap this step but no faster, so
        // the target is -gap/dt, not zero; otherwise the body hangs at margin
        // distance.
        //
        // Restitution is decided here too: after the velocity iterations the
        // approach is already zero and there is nothing to bounce. Gravity's
        // share of this step is taken back out first so a resting body does not
        // bounce off its own weight.
        for (u32 i = 0; i < manifold.count; ++i)
        {
            ContactPoint& point = manifold.points[i];
            const f32 gap = -point.penetration;
            point.velocityBias = gap > 0.0f ? -gap / duration : 0.0f;
            if (contact.restitution <= 0.0f)
                continue;
            const f32 approach = Math::dot(
                relativeVelocity(*contact.a, *contact.b, point.armA, point.armB), manifold.normal);
            const f32 gravityShare =
                Math::dot(contact.b->lastFrameAcceleration() - contact.a->lastFrameAcceleration(),
                          manifold.normal) *
                duration;
            // Below the threshold it is resting, not an impact (bouncing those keeps
            // a settled stack alive). A body that will not reach the surface this
            // step keeps the speculative target.
            if (approach < -mSettings.restitutionThreshold && approach < point.velocityBias)
                point.velocityBias = -contact.restitution * (approach - gravityShare);
        }
    }

    // Articulated groups, once every joint has measured itself and every contact
    // has its frame and target; the articulation also takes the contacts its
    // bodies make with static ground.
    mArticulation.prepare(joints, jointCount, contacts, count, duration, mSettings.baumgarte, mSettings.slop,
                          mSettings.maxPenetration);

    warmStart(contacts, count);

    for (u32 i = 0; i < jointCount; ++i)
        if (active(joints[i]))
            joints[i]->warmStart();

    for (u32 iteration = 0; iteration < mSettings.velocityIterations; ++iteration)
    {
        solveVelocity(contacts, count);
        for (u32 i = 0; i < jointCount; ++i)
            if (iterative(joints[i]))
                joints[i]->solveVelocity();
        mArticulation.solveVelocity();
    }
    mArticulation.finishVelocity();
}

void ContactSolver::solvePositions(Contact* contacts, u32 count, Joint* const* joints,
                                   u32 jointCount)
{
    for (u32 iteration = 0; iteration < mSettings.positionIterations; ++iteration)
    {
        solvePosition(contacts, count);
        for (u32 i = 0; i < jointCount; ++i)
            if (iterative(joints[i]))
                joints[i]->solvePosition(mSettings.baumgarte);
        mArticulation.solvePosition(iteration, mSettings.positionIterations);
    }
}

} // namespace kinetix::Physics
