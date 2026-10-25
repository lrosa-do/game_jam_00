#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>
#include <kinetix/core/Log.h>

#include "character/CharacterBody.h"

#include "collision/CollisionShape.h"

#include <math.h>

namespace kinetix::Physics
{

namespace
{
constexpr f32 kEpsilon = 1.0e-6f;

// The unit-sphere sweep TrimeshShape::sweepEllipsoid runs in ellipsoid space,
// for an ellipsoid that is not axis-aligned in the mesh's own space. Same
// tests and thresholds, so both paths agree wherever both apply.

// Smallest non-negative quadratic root, or false when both roots are negative.
bool solveCollision(f32 a, f32 b, f32 c, f32& t)
{
    const f32 disc = b * b - 4.0f * a * c;
    if (disc < 0.0f)
        return false;
    const f32 sq = sqrtf(disc);
    const f32 inv = 1.0f / (2.0f * a);
    f32 t0 = (-b - sq) * inv;
    f32 t1 = (-b + sq) * inv;
    if (t1 < t0)
    {
        const f32 swap = t0;
        t0 = t1;
        t1 = swap;
    }
    if (t1 < 0.0f)
        return false;
    t = (t0 < 0.0f) ? t1 : t0;
    return true;
}

bool pointInTriangle(const Math::vec3& p, const Math::vec3& v0, const Math::vec3& v1,
                     const Math::vec3& v2, const Math::vec3& normal)
{
    if (Math::dot(v0 - p, Math::cross(v1 - v0, normal)) < 0.0f)
        return false;
    if (Math::dot(v1 - p, Math::cross(v2 - v1, normal)) < 0.0f)
        return false;
    if (Math::dot(v2 - p, Math::cross(v0 - v2, normal)) < 0.0f)
        return false;
    return true;
}

// Unit sphere at `centre` against a point. tMax is in/out; negative is depth.
bool unitSphereVsPoint(const Math::vec3& centre, const Math::vec3& velocity,
                       const Math::vec3& point, f32& tMax, Math::vec3& normal)
{
    const Math::vec3 l = centre - point;
    const f32 l2 = Math::dot(l, l);
    const f32 c = l2 - 1.0f;
    if (c < 0.0f)
    {
        const f32 length = sqrtf(l2);
        const f32 t = length - 1.0f;
        if (tMax < t)
            return false;
        normal = l / length;
        tMax = t;
        return true;
    }

    if (tMax < 0.0f)
        return false;
    f32 t = 0.0f;
    if (!solveCollision(Math::dot(velocity, velocity), 2.0f * Math::dot(velocity, l), c, t))
        return false;
    if (t > tMax)
        return false;
    normal = Math::normalize(centre + velocity * t - point);
    tMax = t;
    return true;
}

bool unitSphereVsSegment(const Math::vec3& centre, const Math::vec3& velocity, const Math::vec3& v0,
                         const Math::vec3& v1, f32& tMax, Math::vec3& normal)
{
    Math::vec3 e = v1 - v0;
    const Math::vec3 l = centre - v0;
    const f32 eLength = Math::length(e);
    if (eLength < 1e-5f)
        return false;
    e /= eLength;

    const Math::vec3 x = Math::cross(l, e);
    const Math::vec3 y = Math::cross(velocity, e);
    const f32 c = Math::dot(x, x) - 1.0f;
    if (c < 0.0f)
    {
        const f32 d = Math::dot(l, e);
        if (d < 0.0f)
            return unitSphereVsPoint(centre, velocity, v0, tMax, normal);
        if (d > eLength)
            return unitSphereVsPoint(centre, velocity, v1, tMax, normal);
        const Math::vec3 onEdge = v0 + e * d;
        const f32 t = Math::length(centre - onEdge) - 1.0f;
        if (tMax < t)
            return false;
        normal = Math::normalize(centre - onEdge);
        tMax = t;
        return true;
    }

    if (tMax < 0.0f)
        return false;
    f32 t = 0.0f;
    if (!solveCollision(Math::dot(y, y), 2.0f * Math::dot(x, y), c, t))
        return false;
    if (t > tMax)
        return false;

    const Math::vec3 contactCentre = centre + velocity * t;
    const f32 d = Math::dot(contactCentre - v0, e);
    if (d < 0.0f)
        return unitSphereVsPoint(centre, velocity, v0, tMax, normal);
    if (d > eLength)
        return unitSphereVsPoint(centre, velocity, v1, tMax, normal);
    normal = Math::normalize(contactCentre - (v0 + e * d));
    tMax = t;
    return true;
}

// One-sided against the plane of `normal` (unit); tMax as above.
bool unitSphereVsTriangle(const Math::vec3& centre, const Math::vec3& velocity, const Math::vec3& v0,
                          const Math::vec3& v1, const Math::vec3& v2, const Math::vec3& normal,
                          f32& tMax, Math::vec3& hitNormal)
{
    const f32 numer = Math::dot(centre - v0, normal) - 1.0f;
    const f32 denom = Math::dot(velocity, normal);
    f32 t = tMax;
    if (numer < 0.0f || denom > -1e-7f)
    {
        if (denom > -1e-5f)
            return false;
        if (numer < -1.0f)
            return false;
        t = numer; // penetration depth, negative
    }
    else
    {
        t = -(numer / denom);
        if (t < 0.0f || t > tMax)
            return false;
    }

    const Math::vec3 contactCentre = (t < 0.0f) ? centre - normal * t : centre + velocity * t;
    if (pointInTriangle(contactCentre, v0, v1, v2, normal))
    {
        hitNormal = normal;
        tMax = t;
        return true;
    }

    bool hit = false;
    hit |= unitSphereVsSegment(centre, velocity, v0, v1, tMax, hitNormal);
    hit |= unitSphereVsSegment(centre, velocity, v1, v2, tMax, hitNormal);
    hit |= unitSphereVsSegment(centre, velocity, v2, v0, tMax, hitNormal);
    return hit;
}
} // namespace

void CharacterBody::setShape(f32 radius, f32 height)
{
    mRadius = Math::max(radius, 0.01f);
    mHeight = Math::max(height, 0.0f);
}

void CharacterBody::setSlopeLimit(f32 degrees)
{
    mSlopeLimitDegrees = Math::clamp(degrees, 0.0f, 89.0f);
    mSlopeLimitCosine = cosf(Math::radians(mSlopeLimitDegrees));
}

Math::mat4 CharacterBody::transform() const
{
    Math::mat4 result(1.0f);
    result[3] = Math::vec4(mPosition, 1.0f);
    return result;
}

void CharacterBody::jump(f32 speed)
{
    if (!mGrounded)
        return;
    mVerticalSpeed = speed;
    mGrounded = false;
}

void CharacterBody::teleport(const Math::vec3& position)
{
    mPosition = position;
    mVerticalSpeed = 0.0f;
    mVelocity = Math::vec3(0.0f);
    mGrounded = false;
    mOnWall = false;
    mOnCeiling = false;
    mGroundNormal = mUpDirection;
    mWallNormal = Math::vec3(0.0f);
    mCeilingNormal = Math::vec3(0.0f);
}

f32 CharacterBody::slopeAngle() const
{
    if (!mGrounded)
        return 0.0f;
    return Math::degrees(acosf(Math::clamp(Math::dot(mGroundNormal, mUpDirection), -1.0f, 1.0f)));
}

Math::vec3 CharacterBody::radii() const
{
    const f32 half = mHeight * 0.5f + mRadius;
    return Math::vec3(mRadius, half, mRadius);
}

Math::mat3 CharacterBody::axes() const
{
    // Any frame with up as its second column: the radii across it are equal,
    // so the turn about up does not matter.
    const Math::vec3 up = mUpDirection;
    if (Math::dot(up, Math::vec3(0.0f, 1.0f, 0.0f)) > 1.0f - 1.0e-7f)
        return Math::mat3(1.0f);
    const Math::vec3 helper =
        fabsf(up.x) < 0.9f ? Math::vec3(1.0f, 0.0f, 0.0f) : Math::vec3(0.0f, 0.0f, 1.0f);
    const Math::vec3 side = Math::normalize(Math::cross(up, helper));
    return Math::mat3(Math::cross(up, side), up, side);
}

bool TrimeshSweepSource::sweepEllipsoid(const Math::vec3& centre, const Math::mat3& axes,
                                        const Math::vec3& radii, const Math::vec3& velocity,
                                        CharacterSweepHit& hit) const
{
    if (radii.x <= 0.0f || radii.y <= 0.0f || radii.z <= 0.0f)
        return false;

    // Any invertible affine map (rotation, uneven scale, mirror): inverted,
    // never transposed.
    const Math::mat3 linear(mTransform);
    const f32 determinant = Math::determinant(linear);
    if (!(fabsf(determinant) > 1.0e-12f))
        return false;
    const Math::mat3 inverseLinear = Math::inverse(linear);
    const Math::vec3 origin(mTransform[3]);

    // Unit sphere -> world is axes * diag(radii); unit sphere -> mesh space
    // is that with inverseLinear in front. Its shape matrix A*A^T is diagonal
    // exactly when the ellipsoid is axis-aligned in mesh space, and then the
    // mesh's own sweep takes it with radii from that diagonal.
    Math::mat3 scaledAxes = axes;
    scaledAxes[0] = axes[0] * radii.x;
    scaledAxes[1] = axes[1] * radii.y;
    scaledAxes[2] = axes[2] * radii.z;
    const Math::mat3 toMesh = inverseLinear * scaledAxes;
    const Math::mat3 shape = toMesh * Math::transpose(toMesh);
    const f32 largest = Math::max(shape[0][0], Math::max(shape[1][1], shape[2][2]));
    const f32 tolerance = largest * 1.0e-6f;
    const bool aligned = fabsf(shape[1][0]) <= tolerance && fabsf(shape[2][0]) <= tolerance &&
                         fabsf(shape[2][1]) <= tolerance;

    // Both paths measure a negative t in the ellipsoid's unit-sphere space,
    // and any two unit-sphere spaces of one ellipsoid differ by a rotation,
    // so the depth means the same in either.
    f32 t = 0.0f;
    Math::vec3 normal(0.0f);
    if (aligned)
    {
        TrimeshShape::SweepHit local;
        const Math::vec3 localRadii(sqrtf(shape[0][0]), sqrtf(shape[1][1]), sqrtf(shape[2][2]));
        if (!mMesh.sweepEllipsoid(inverseLinear * (centre - origin), localRadii,
                                  inverseLinear * velocity, local))
            return false;
        hit.triangle = local.triangle;
        t = local.t;
        normal = Math::normalize(Math::transpose(inverseLinear) * local.normal);
    }
    else
    {
        // World -> unit sphere: diag(1/radii) * axes^T around the centre;
        // mesh -> unit sphere puts the mesh transform in front of that.
        const Math::vec3 inverseRadii(1.0f / radii.x, 1.0f / radii.y, 1.0f / radii.z);
        const Math::mat3 axesT = Math::transpose(axes);
        const Math::vec3 eVelocity = (axesT * velocity) * inverseRadii;

        const f32 reach = Math::max(radii.x, Math::max(radii.y, radii.z)) + 0.01f;
        AABB swept;
        swept.expand(centre - Math::vec3(reach));
        swept.expand(centre + Math::vec3(reach));
        swept.expand(centre + velocity - Math::vec3(reach));
        swept.expand(centre + velocity + Math::vec3(reach));
        static thread_local ct::Vector<u32> candidates;
        mMesh.query(transformAABB(swept, Math::inverse(mTransform)), candidates);

        // A mirroring transform reverses the winding; the front face is the
        // same half-space either way.
        const f32 winding = determinant < 0.0f ? -1.0f : 1.0f;
        f32 best = 1.0f;
        bool found = false;
        for (u32 index : candidates)
        {
            const TriangleShape triangle = mMesh.triangle(index);
            Math::vec3 e[3];
            for (u32 v = 0; v < 3; ++v)
                e[v] = (axesT * (linear * triangle.vertex(v) + origin - centre)) * inverseRadii;
            const Math::vec3 raw = Math::cross(e[1] - e[0], e[2] - e[0]) * winding;
            if (Math::length(raw) < 1.0e-12f)
                continue;

            f32 candidate = best;
            Math::vec3 eNormal(0.0f);
            if (!unitSphereVsTriangle(Math::vec3(0.0f), eVelocity, e[0], e[1], e[2],
                                      Math::normalize(raw), candidate, eNormal))
                continue;
            best = candidate;
            hit.triangle = index;
            normal = Math::normalize(axes * (eNormal * inverseRadii));
            found = true;
        }
        if (!found)
            return false;
        t = best;
    }

    hit.normal = normal;
    if (t >= 0.0f)
    {
        hit.t = t;
        return true;
    }

    // A unit of depth in unit-sphere space is the ellipsoid's own extent
    // along the world normal: its support distance |diag(radii)*axes^T*n|.
    const Math::vec3 alongAxes = Math::transpose(axes) * normal;
    hit.t = t * Math::length(alongAxes * radii);
    return true;
}

CharacterBody::Slide CharacterBody::slide(const Math::vec3& startCentre,
                                          const Math::vec3& displacement,
                                          const CharacterSweepSource& source) const
{
    Slide out;
    out.centre = startCentre;
    out.velocity = displacement;

    const Math::vec3 extents = radii();
    const Math::mat3 frame = axes();
    const Math::vec3 up = mUpDirection;

    // World-space bookkeeping only; projecting the destination with the
    // ellipsoid-space normal leaves a residual whose vertical component gets
    // amplified on the way back and launches the character off walls.
    Math::vec3 center = startCentre;
    Math::vec3 velocity = displacement;
    Math::vec3 outputVelocity = displacement;
    Math::vec3 endpoint = startCentre + displacement;
    const Math::vec3 inputHorizontal = displacement - up * Math::dot(displacement, up);

    // Every sweep looks one skin width past the move: a frame's travel is
    // often less than the skin (1 m/s at 60 Hz is 0.017 against 0.02), so
    // stopping at the move's end misses the wall every other frame and the
    // contact flickers.
    bool resolvedOverlap = false;
    u32 iteration = 0;
    for (; iteration < mMaxIterations; ++iteration)
    {
        const f32 distance = Math::length(velocity);
        if (distance < 1e-6f)
            break;
        const Math::vec3 direction = velocity / distance;
        const Math::vec3 iterationStart = center;

        CharacterSweepHit hit;
        if (!source.sweepEllipsoid(center, frame, extents, direction * (distance + mSkinWidth), hit))
        {
            center = center + velocity;
            break;
        }

        out.collided = true;
        const Math::vec3 n = hit.normal;
        const f32 upDot = Math::dot(n, up);

        // Floor within the slope limit, ceiling when facing along -up, wall
        // otherwise; each keeps the most extreme normal seen.
        const bool floor = upDot >= mSlopeLimitCosine;
        const bool ceiling = upDot <= -mSlopeLimitCosine;
        if (floor)
        {
            if (!out.grounded || upDot > Math::dot(out.groundNormal, up))
                out.groundNormal = n;
            out.grounded = true;
        }
        else if (ceiling)
        {
            if (!out.onCeiling || upDot < Math::dot(out.ceilingNormal, up))
                out.ceilingNormal = n;
            out.onCeiling = true;
        }
        else
        {
            out.steepBlock = true;
            if (!out.onWall || upDot < Math::dot(out.wallNormal, up))
                out.wallNormal = n;
            out.onWall = true;
        }

        // One push-out per move: iterating an embedded shove is how a wedged
        // character is thrown across the level in a single frame.
        if (hit.t < 0.0f)
        {
            if (resolvedOverlap)
                break;
            resolvedOverlap = true;
            center = center - n * hit.t + n * mSkinWidth;
        }
        else
        {
            // Along the move until the gap to the surface is down to the skin
            // width, or to what it already was if the move began inside the
            // skin: pushing back out to the full width re-opens a gap wider
            // than the next frame's travel (the flicker), and travelling on
            // to the contact would carry it along the surface before the wall
            // stop sees the hit.
            const f32 contact = hit.t * (distance + mSkinWidth);
            const f32 approach = Math::clamp(-Math::dot(direction, n), 0.0f, 1.0f);
            const f32 startGap = contact * approach;
            const f32 targetGap = Math::min(mSkinWidth, startGap);
            const f32 reach = Math::min(contact, distance);
            const f32 travel =
                approach > 1.0e-4f ? Math::clamp((startGap - targetGap) / approach, 0.0f, reach) : reach;
            const f32 reachedGap = (contact - travel) * approach;
            center = center + direction * travel + n * Math::max(targetGap - reachedGap, 0.0f);
        }

        outputVelocity = outputVelocity - n * Math::dot(outputVelocity, n);

        // Destination projected onto the sliding plane, not the leftover
        // velocity - the projection of the leftover gives a shorter slide.
        endpoint = endpoint - n * Math::dot(endpoint - center, n);

        // A wall steeper than the limit cannot be climbed: drop the upward
        // component so the slide goes down it. A ceiling only does so when
        // sliding along ceilings is off.
        if (ceiling ? !mSlideOnCeiling : !floor)
        {
            const f32 upSpeed = Math::dot(outputVelocity, up);
            if (upSpeed > 0.0f)
                outputVelocity -= up * upSpeed;
            if (ceiling)
            {
                const f32 upTravel = Math::dot(endpoint - center, up);
                if (upTravel > 0.0f)
                    endpoint -= up * upTravel;
            }
        }

        if (mFloorStopOnSlope && floor && Math::length(displacement) > 1e-6f &&
            Math::length(Math::normalize(displacement) + up) < 0.01f)
        {
            if (Math::length(center - iterationStart) <= mSkinWidth + kEpsilon)
                center = iterationStart;
            outputVelocity = Math::vec3(0.0f);
            endpoint = center;
            break;
        }

        // On the floor, pressing into a wall within this angle of head-on
        // stops instead of sliding. Both the remaining move and the velocity
        // lose their horizontal part, or the body slides on while reporting a
        // stop.
        if (mWallMinSlideAngleDegrees > 0.0f && out.onWall && (out.grounded || mGrounded) &&
            Math::length(inputHorizontal) > 1e-6f)
        {
            const Math::vec3 wallHorizontal = out.wallNormal - up * Math::dot(out.wallNormal, up);
            if (Math::length(wallHorizontal) > 1e-6f)
            {
                const f32 headOn =
                    -Math::dot(Math::normalize(wallHorizontal), Math::normalize(inputHorizontal));
                if (headOn > mWallMinSlideAngleCosine)
                {
                    outputVelocity = up * Math::dot(outputVelocity, up);
                    endpoint = center + up * Math::dot(endpoint - center, up);
                }
            }
        }

        velocity = endpoint - center;
    }

    out.centre = center;
    out.velocity = outputVelocity;
    return out;
}

bool CharacterBody::stepUp(const Math::vec3& startCentre, const Math::vec3& horizontal,
                           const CharacterSweepSource& source, Slide& out) const
{
    if (mStepOffset <= 0.0f || Math::dot(horizontal, horizontal) < kEpsilon)
        return false;

    const Math::vec3 extents = radii();
    const Math::mat3 frame = axes();

    const Math::vec3 up = mUpDirection * mStepOffset;
    CharacterSweepHit upHit;
    if (source.sweepEllipsoid(startCentre, frame, extents, up, upHit))
        return false;

    // The step is probed at least part of a radius ahead: one frame's move is only a few
    // centimetres, and coming down that close the lower curve of the body lands on the step's
    // edge, which reads steep, so a staircase would stop it at the first step.
    const f32 length = Math::length(horizontal);
    const Math::vec3 ahead = horizontal * (Math::max(length, mRadius * 0.6f) / length);

    // The raised path has to get through, or a wall taller than the step lets
    // the climb through and the character ratchets up it a step per frame. A
    // wall along the side (stairs between walls) only grazes it: that still
    // gets most of the way, so it is not a block.
    const Slide across = slide(startCentre + up, ahead, source);
    const Math::vec3 moved = across.centre - (startCentre + up);
    if (across.collided && Math::dot(moved, ahead) < Math::dot(ahead, ahead) * 0.5f)
        return false;

    // Nothing underneath is a gap, not a step.
    const Math::vec3 down = mUpDirection * -(mStepOffset + mSkinWidth);
    CharacterSweepHit downHit;
    if (!source.sweepEllipsoid(across.centre, frame, extents, down, downHit))
        return false;

    const Math::vec3 normal = downHit.normal;
    if (Math::dot(normal, mUpDirection) < mSlopeLimitCosine)
        return false;

    // Climb to the height found there, but move only this frame's distance.
    const Math::vec3 landed = across.centre + down * Math::max(downHit.t, 0.0f) + normal * mSkinWidth;
    const f32 rise = Math::dot(landed - startCentre, mUpDirection);
    if (rise <= kEpsilon)
        return false;

    out = across;
    out.centre = startCentre + horizontal + mUpDirection * rise;
    out.collided = true;
    out.grounded = true;
    out.groundNormal = normal;
    return true;
}

bool CharacterBody::snapToGround(const CharacterSweepSource& source)
{
    if (mGroundSnapDistance <= 0.0f)
        return false;

    const Math::vec3 extents = radii();
    const Math::mat3 frame = axes();
    const Math::vec3 down = mUpDirection * -mGroundSnapDistance;

    CharacterSweepHit hit;
    if (!source.sweepEllipsoid(mPosition, frame, extents, down, hit))
        return false;

    const Math::vec3 normal = hit.normal;
    if (Math::dot(normal, mUpDirection) < mSlopeLimitCosine)
        return false;

    // Only downward hits ahead of us; a negative t is an overlap the slide
    // has just dealt with.
    if (hit.t < 0.0f || hit.t > 1.0f)
        return false;

    const Math::vec3 travel = down * hit.t + normal * mSkinWidth;
    mPosition += mUpDirection * Math::dot(mUpDirection, travel);
    mGrounded = true;
    mGroundNormal = normal;
    return true;
}

bool CharacterBody::isNearGround(const TrimeshShape& mesh, const Math::mat4& meshTransform,
                                 f32 maxDistance) const
{
    return isNearGround(TrimeshSweepSource(mesh, meshTransform), maxDistance);
}

bool CharacterBody::isNearGround(const CharacterSweepSource& source, f32 maxDistance) const
{
    if (maxDistance <= 0.0f)
        return false;

    const Math::vec3 extents = radii();
    const Math::mat3 frame = axes();
    const Math::vec3 down = mUpDirection * -maxDistance;

    CharacterSweepHit hit;
    if (!source.sweepEllipsoid(mPosition, frame, extents, down, hit))
        return false;
    if (hit.t < 0.0f || hit.t > 1.0f)
        return false;

    return Math::dot(hit.normal, mUpDirection) >= mSlopeLimitCosine;
}

CharacterBody::MoveResult CharacterBody::move(const Math::vec3& displacement,
                                              const TrimeshShape& mesh,
                                              const Math::mat4& meshTransform)
{
    return move(displacement, TrimeshSweepSource(mesh, meshTransform));
}

CharacterBody::MoveResult CharacterBody::move(const Math::vec3& displacement,
                                              const CharacterSweepSource& source)
{
    MoveResult result;
    const Math::vec3 start = mPosition;
    const Math::vec3 up = mUpDirection;
    const Math::vec3 horizontal = displacement - up * Math::dot(displacement, up);

    Slide primary = slide(start, displacement, source);

    // Climb only when the horizontal move was actually stopped: a platform
    // seam is an edge, an edge contact reads steep even on level floor, and
    // firing the climb there is a hop at every junction.
    const Math::vec3 achieved = primary.centre - start;
    const Math::vec3 achievedHorizontal = achieved - up * Math::dot(achieved, up);
    const f32 wanted = Math::dot(horizontal, horizontal);
    const bool reallyBlocked =
        wanted > kEpsilon && Math::dot(achievedHorizontal, horizontal) < wanted * 0.5f;
    if (primary.steepBlock && reallyBlocked)
    {
        Slide climbed;
        if (stepUp(start, horizontal, source, climbed))
            primary = climbed;
    }

    mPosition = primary.centre;
    mGrounded = primary.grounded;
    mGroundNormal = primary.grounded ? primary.groundNormal : up;
    mOnWall = primary.onWall;
    mOnCeiling = primary.onCeiling;
    mWallNormal = primary.onWall ? primary.wallNormal : Math::vec3(0.0f);
    mCeilingNormal = primary.onCeiling ? primary.ceilingNormal : Math::vec3(0.0f);

    result.collided = primary.collided;
    result.grounded = primary.grounded;
    result.blocked = primary.steepBlock;
    result.onWall = primary.onWall;
    result.onCeiling = primary.onCeiling;
    result.groundNormal = mGroundNormal;
    result.wallNormal = mWallNormal;
    result.ceilingNormal = mCeilingNormal;
    result.remaining = primary.velocity;
    return result;
}

bool CharacterBody::applyFloorSnap(const TrimeshShape& mesh, const Math::mat4& meshTransform)
{
    return snapToGround(TrimeshSweepSource(mesh, meshTransform));
}

bool CharacterBody::applyFloorSnap(const CharacterSweepSource& source)
{
    return snapToGround(source);
}

CharacterBody::MoveResult CharacterBody::moveAndSlide(f32 deltaTime, const TrimeshShape& mesh,
                                                      const Math::mat4& meshTransform)
{
    return moveAndSlide(deltaTime, TrimeshSweepSource(mesh, meshTransform));
}

CharacterBody::MoveResult CharacterBody::moveAndSlide(f32 deltaTime,
                                                      const CharacterSweepSource& source)
{
    MoveResult result;
    if (!(deltaTime > 0.0f))
        return result;

    // The caller owns velocity (gravity and jump included).
    const Math::vec3 start = mPosition;
    const Math::vec3 up = mUpDirection;
    const bool wasGrounded = mGrounded;
    const bool velFacingUp = Math::dot(mVelocity, up) > 0.0f;

    result = move(mVelocity * deltaTime, source);

    // If the slide just lost a floor the body was standing on, probing down
    // and settling keeps resting contact from flickering.
    if (!result.grounded && wasGrounded && !velFacingUp &&
        Math::dot(mPosition - start, up) <= kEpsilon && snapToGround(source))
    {
        result.grounded = true;
        result.groundNormal = mGroundNormal;
    }

    mVelocity = result.remaining / deltaTime;
    mVerticalSpeed = Math::dot(mVelocity, up);
    if (result.grounded && mVerticalSpeed < 0.0f)
        mVerticalSpeed = 0.0f;
    return result;
}

CharacterBody::MoveResult CharacterBody::update(f32 deltaTime, const TrimeshShape& mesh,
                                                const Math::mat4& meshTransform)
{
    return update(deltaTime, TrimeshSweepSource(mesh, meshTransform));
}

CharacterBody::MoveResult CharacterBody::update(f32 deltaTime, const CharacterSweepSource& source)
{
    MoveResult result;
    if (!(deltaTime > 0.0f))
        return result;

    const Math::vec3 start = mPosition;
    const Math::vec3 up = mUpDirection;

    // Planted while grounded, so standing still does not accumulate downward
    // speed frame after frame.
    if (mGrounded && mVerticalSpeed < 0.0f)
        mVerticalSpeed = 0.0f;

    mVerticalSpeed += mGravity * deltaTime;
    if (mVerticalSpeed < mMaxFallSpeed)
        mVerticalSpeed = mMaxFallSpeed;

    const bool wasGrounded = mGrounded;
    result = move(mMoveInput * deltaTime + up * (mVerticalSpeed * deltaTime), source);

    // Only going down, only if standing a moment ago, and only if this frame
    // gained no height: a jump must leave the floor, a cliff walk-off must
    // fall, and snapping mid-climb undoes the climb.
    if (!result.grounded && wasGrounded && mVerticalSpeed <= 0.0f &&
        Math::dot(mPosition - start, up) <= kEpsilon && snapToGround(source))
    {
        result.grounded = true;
        result.groundNormal = mGroundNormal;
    }

    // The slid velocity, never the position delta: the delta includes the
    // push-out, and reading that back as speed launches the character off a
    // surface he only brushed.
    mVelocity = result.remaining / deltaTime;
    mVerticalSpeed = Math::dot(mVelocity, up);
    if (result.grounded && mVerticalSpeed < 0.0f)
        mVerticalSpeed = 0.0f;
    return result;
}

} // namespace kinetix::Physics
