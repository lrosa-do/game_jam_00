#include "marine/PrismBuoyancy.h"

#include "dynamics/RigidBody.h"
#include "marine/WaterSurface.h"

#include <math.h>

namespace kinetix::Physics
{

namespace
{
bool finiteVec(const Math::vec3& v)
{
    return isfinite(v.x) && isfinite(v.y) && isfinite(v.z);
}

// Where the edge from `under` (below the surface) to `over` (above it)
// crosses the surface at height `surface`.
Math::vec3 waterlinePoint(const Math::vec3& under, const Math::vec3& over, f32 surface)
{
    return under + (over - under) * ((surface - under.y) / (over.y - under.y));
}
} // namespace

void PrismBuoyancy::setHull(const HullMesh& hull)
{
    mHull = hull;
    const Math::vec3 zero(0.0f);
    mPositions.resize(hull.vertexCount(), zero);
    mVelocities.resize(hull.vertexCount(), zero);
    mForces.resize(hull.vertexCount(), zero);
}

f32 PrismBuoyancy::surfaceHeight(const Math::vec3& at) const
{
    return mWater->height(at.x, at.z);
}

Math::vec3 PrismBuoyancy::surfaceVelocity(const Math::vec3& at) const
{
    return mWater->flowVelocity(at);
}

f32 PrismBuoyancy::tetrahedronVolume(const Math::vec3& o, const Math::vec3& a, const Math::vec3& b,
                                     const Math::vec3& c) const
{
    return Math::dot(a - o, Math::cross(b - o, c - o)) / 6.0f;
}

// A piece wholly under the surface: the pressure prism along its normal, and
// the drag.
Math::vec3 PrismBuoyancy::submergedForce(const Math::vec3& a, const Math::vec3& b,
                                         const Math::vec3& c, Math::vec3 velocity, Type type)
{
    Math::vec3 normal = Math::cross(b - a, c - a);
    f32 area = Math::length(normal);
    if (area < 0.00001f)
        return Math::vec3(0.0f);
    normal = normal * (1.0f / area);
    area = area / 2.0f;

    // The prism: each corner pushed along the normal by its depth times
    // rho * g, and the volume between the piece and the pushed copy, as the
    // eight tetrahedra around the centroid of the six points.
    f32 pressure = 0.0f;
    if (type != Type::DragOnly)
    {
        const Math::vec3 ap = a + normal * ((surfaceHeight(a) - a.y) * mPressurePerMetre);
        const Math::vec3 bp = b + normal * ((surfaceHeight(b) - b.y) * mPressurePerMetre);
        const Math::vec3 cp = c + normal * ((surfaceHeight(c) - c.y) * mPressurePerMetre);
        const Math::vec3 centroid = (a + b + c + ap + bp + cp) * (1.0f / 6.0f);
        pressure += tetrahedronVolume(centroid, a, b, c);
        pressure += tetrahedronVolume(centroid, a, ap, bp);
        pressure += tetrahedronVolume(centroid, a, bp, b);
        pressure += tetrahedronVolume(centroid, b, bp, cp);
        pressure += tetrahedronVolume(centroid, b, cp, c);
        pressure += tetrahedronVolume(centroid, c, cp, ap);
        pressure += tetrahedronVolume(centroid, c, ap, a);
        pressure += tetrahedronVolume(centroid, ap, cp, bp);
    }

    // The drag, against the water's own motion, along the normal and on the
    // side the water comes from.
    Math::vec3 drag(0.0f);
    if (type != Type::Dragless)
    {
        const Math::vec3 centre = (a + b + c) * (1.0f / 3.0f);
        velocity = velocity - surfaceVelocity(centre);
        const f32 speed = Math::length(velocity);
        if (speed > 0.01f)
        {
            const Math::vec3 direction = velocity * (1.0f / speed);
            const f32 facing = fabsf(Math::dot(normal, direction));
            drag = normal * (-mHalfDensity * area * speed * speed * facing);
            if (Math::dot(normal, direction) < 0.0f)
                drag = -drag;

            // A piece pushing hard enough throws spray off whichever of its
            // corners lies at the surface, up and out along the normal.
            if (mEmitEffects && mEffects)
            {
                const f32 push = speed * facing * area;
                if (push > 1.5f)
                {
                    Math::vec3 thrown = normal * push;
                    if (thrown.y < 0.0f)
                        thrown.y = -thrown.y;

                    if (surfaceHeight(a) - a.y < 0.1f)
                        mEffects->spray(a, thrown);
                    else if (surfaceHeight(b) - b.y < 0.1f)
                        mEffects->spray(b, thrown);
                    else if (surfaceHeight(c) - c.y < 0.1f)
                        mEffects->spray(c, thrown);
                }
            }
        }
    }
    if (mSink)
        return drag;
    return normal * pressure + drag;
}

// A piece of the hull however it lies against the surface: nothing when it
// is clear of the water, the whole of it when it is under, and otherwise
// the part under the waterline - one triangle when one corner is under, two
// when two are.
Math::vec3 PrismBuoyancy::pieceForce(const Math::vec3& a, const Math::vec3& b, const Math::vec3& c,
                                     const Math::vec3& velocity, Type type)
{
    const f32 surface = surfaceHeight((a + b + c) * (1.0f / 3.0f));
    if (a.y > surface && b.y > surface && c.y > surface)
        return Math::vec3(0.0f);
    if (a.y > surface || b.y > surface || c.y > surface)
    {
        if (a.y < surface && b.y > surface && c.y > surface)
            return submergedForce(a, waterlinePoint(a, b, surface), waterlinePoint(a, c, surface), velocity,
                                  type);
        if (b.y < surface && c.y > surface && a.y > surface)
            return submergedForce(b, waterlinePoint(b, c, surface), waterlinePoint(b, a, surface), velocity,
                                  type);
        if (c.y < surface && a.y > surface && b.y > surface)
            return submergedForce(c, waterlinePoint(c, a, surface), waterlinePoint(c, b, surface), velocity,
                                  type);
        if (a.y > surface && b.y < surface && c.y < surface)
        {
            const Math::vec3 tb = waterlinePoint(b, a, surface);
            const Math::vec3 tc = waterlinePoint(c, a, surface);
            return submergedForce(tb, b, tc, velocity, type) + submergedForce(tc, b, c, velocity, type);
        }
        if (b.y > surface && c.y < surface && a.y < surface)
        {
            const Math::vec3 tc = waterlinePoint(c, b, surface);
            const Math::vec3 ta = waterlinePoint(a, b, surface);
            return submergedForce(tc, c, ta, velocity, type) + submergedForce(ta, c, a, velocity, type);
        }
        if (c.y > surface && a.y < surface && b.y < surface)
        {
            const Math::vec3 ta = waterlinePoint(a, c, surface);
            const Math::vec3 tb = waterlinePoint(b, c, surface);
            return submergedForce(ta, a, tb, velocity, type) + submergedForce(tb, a, b, velocity, type);
        }
        return Math::vec3(0.0f);
    }
    return submergedForce(a, b, c, velocity, type);
}

// One triangle of the hull, as six pieces around its centre: the two pieces
// at each corner go to that corner's vertex.
void PrismBuoyancy::accumulateTriangle(u32 ia, u32 ib, u32 ic, Type type)
{
    const Math::vec3& a = mPositions[ia];
    const Math::vec3& b = mPositions[ib];
    const Math::vec3& c = mPositions[ic];
    if (a.y > surfaceHeight(a) && b.y > surfaceHeight(b) && c.y > surfaceHeight(c))
        return;
    ++mWetTriangles;

    const Math::vec3 m = (a + b + c) * (1.0f / 3.0f);
    const Math::vec3 mab = (a + b) * 0.5f;
    const Math::vec3 mbc = (b + c) * 0.5f;
    const Math::vec3 mca = (c + a) * 0.5f;
    const Math::vec3 velocity = (mVelocities[ia] + mVelocities[ib] + mVelocities[ic]) * (1.0f / 3.0f);

    mForces[ia] += pieceForce(a, mab, m, velocity, type) + pieceForce(a, m, mca, velocity, type);
    mForces[ib] += pieceForce(b, mbc, m, velocity, type) + pieceForce(b, m, mab, velocity, type);
    mForces[ic] += pieceForce(c, mca, m, velocity, type) + pieceForce(c, m, mbc, velocity, type);
}

void PrismBuoyancy::applyForces()
{
    mForce = Math::vec3(0.0f);
    mTorque = Math::vec3(0.0f);
    mDraft = 0.0f;
    mWetTriangles = 0;
    if (!mBody || !mWater || mHull.vertexCount() == 0)
        return;
    mPressurePerMetre = mWater->density() * mWater->gravity();
    mHalfDensity = 0.5f * mWater->density();

    const ct::Vector<Math::vec3>& vertices = mHull.vertices();
    const u32 count = mHull.vertexCount();
    for (u32 i = 0; i < count; ++i)
    {
        mPositions[i] = mBody->pointToWorld(vertices[i]);
        mVelocities[i] = mBody->velocityAtPoint(mPositions[i]);
        mForces[i] = Math::vec3(0.0f);
        const f32 depth = surfaceHeight(mPositions[i]) - mPositions[i].y;
        if (depth > mDraft)
            mDraft = depth;
    }
    const ct::Vector<u32>& indices = mHull.indices();
    for (u32 t = 0; t < mHull.triangleCount(); ++t)
        accumulateTriangle(indices[3 * t], indices[3 * t + 1], indices[3 * t + 2], mType);

    const Math::vec3 centre = mBody->position();
    for (u32 i = 0; i < count; ++i)
    {
        const Math::vec3& f = mForces[i];
        if ((f.x == 0.0f && f.y == 0.0f && f.z == 0.0f) || !finiteVec(f))
            continue;
        mBody->addForceAtPoint(f, mPositions[i]);
        mForce += f;
        mTorque += Math::cross(mPositions[i] - centre, f);
    }

    // A vertex under the surface, within 0.2 m of it and moving faster than
    // 2 m/s, splashes and ripples.
    if (mEmitEffects && mEffects)
    {
        for (u32 i = 0; i < count; ++i)
        {
            const Math::vec3& p = mPositions[i];
            const f32 depth = surfaceHeight(p) - p.y;
            if (depth > 0.0f && depth < 0.2f && Math::dot(mVelocities[i], mVelocities[i]) > 4.0f)
            {
                mEffects->splash(p, mVelocities[i]);
                mEffects->ripple(p, mVelocities[i]);
            }
        }
    }
    mEmitEffects = false;
}

} // namespace kinetix::Physics
