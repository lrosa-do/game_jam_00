#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>
#include <kinetix/core/Log.h>

#include "marine/BuoyancyBody.h"

#include "dynamics/RigidBody.h"
#include "marine/WaterSurface.h"

#include <math.h>

namespace kinetix::Physics
{

namespace
{
// Below this relative speed a face element sees no flow worth a direction.
constexpr f32 kMinSpeed = 1.0e-5f;
constexpr f32 kMinReynolds = 1.0e5f;

bool finiteVec(const Math::vec3& v)
{
    return isfinite(v.x) && isfinite(v.y) && isfinite(v.z);
}

WetVertex lerpWet(const WetVertex& a, const WetVertex& b, f32 t)
{
    WetVertex out;
    out.position = a.position + (b.position - a.position) * t;
    out.flow = a.flow + (b.flow - a.flow) * t;
    out.depth = 0.0f;
    return out;
}

f32 falloff(f32 cosine, f32 exponent)
{
    const f32 c = fabsf(cosine);
    if (exponent == 1.0f)
        return c;
    if (exponent == 2.0f)
        return c * c;
    return powf(c, exponent);
}

// integral of f^2 over a triangle, f linear with corner values a, b, c, per
// unit area: (a^2 + b^2 + c^2 + ab + bc + ca) / 6.
f32 meanSquare(f32 a, f32 b, f32 c)
{
    return (a * a + b * b + c * c + a * b + b * c + c * a) / 6.0f;
}
} // namespace

u32 clipTriangleToWater(const WetVertex in[3], WetTriangle out[2], Math::vec3* waterline, bool* crosses)
{
    if (crosses)
        *crosses = false;
    const bool wet0 = in[0].depth > 0.0f;
    const bool wet1 = in[1].depth > 0.0f;
    const bool wet2 = in[2].depth > 0.0f;
    const u32 wetCount = u32(wet0) + u32(wet1) + u32(wet2);
    if (wetCount == 0)
        return 0;
    if (wetCount == 3)
    {
        out[0].vertices[0] = in[0];
        out[0].vertices[1] = in[1];
        out[0].vertices[2] = in[2];
        return 1;
    }

    // Sutherland-Hodgman against depth > 0, keeping boundary order. A wet-dry edge is cut where
    // the linear depth is zero, t = d_i / (d_i - d_j); exactly one depth is strictly positive,
    // so the denominator is never zero.
    WetVertex polygon[4];
    u32 count = 0;
    u32 cutIndex[2] = {0, 0};
    u32 cuts = 0;
    for (u32 i = 0; i < 3; ++i)
    {
        const WetVertex& a = in[i];
        const WetVertex& b = in[(i + 1) % 3];
        const bool wetA = a.depth > 0.0f;
        const bool wetB = b.depth > 0.0f;
        if (wetA)
            polygon[count++] = a;
        if (wetA != wetB)
        {
            const f32 t = Clamp(a.depth / (a.depth - b.depth), 0.0f, 1.0f);
            if (cuts < 2)
                cutIndex[cuts++] = count;
            polygon[count++] = lerpWet(a, b, t);
        }
    }

    if (waterline && cuts == 2)
    {
        const bool adjacent = (cutIndex[0] + 1) % count == cutIndex[1];
        waterline[0] = polygon[adjacent ? cutIndex[0] : cutIndex[1]].position;
        waterline[1] = polygon[adjacent ? cutIndex[1] : cutIndex[0]].position;
        if (crosses)
            *crosses = true;
    }

    out[0].vertices[0] = polygon[0];
    out[0].vertices[1] = polygon[1];
    out[0].vertices[2] = polygon[2];
    if (count == 3)
        return 1;
    out[1].vertices[0] = polygon[0];
    out[1].vertices[1] = polygon[2];
    out[1].vertices[2] = polygon[3];
    return 2;
}

f32 ittc57FrictionCoefficient(f32 reynolds)
{
    const f32 re = isfinite(reynolds) ? Max(reynolds, kMinReynolds) : kMinReynolds;
    const f32 denominator = log10f(re) - 2.0f;
    return 0.075f / (denominator * denominator);
}

BuoyancyBody::BuoyancyBody(RigidBody* body, const HullMesh& hull, const WaterSurface* water)
    : mBody(body)
    , mWater(water)
{
    setHull(hull);
}

void BuoyancyBody::setHull(const HullMesh& hull)
{
    mHull = hull;
    if (!mHull.isClosed())
        Log::warning("BuoyancyBody: the hull mesh is not closed (%u open edges) - its buoyancy "
                     "will be wrong",
                     mHull.openEdgeCount());
    mVertices.resize(mHull.vertexCount());
    mWetted.resize(mHull.triangleCount());
    mPreviousWetted.resize(mHull.triangleCount());
    for (u32 i = 0; i < mHull.triangleCount(); ++i)
    {
        mWetted[i] = 0.0f;
        mPreviousWetted[i] = 0.0f;
    }
    mHistoryValid = false;
    setForwardAxis(mForward);
}

void BuoyancyBody::setCoefficients(const HydrodynamicCoefficients& coefficients)
{
    HydrodynamicCoefficients& c = mCoefficients;
    auto clean = [](f32 value, f32 lo, f32 fallback)
    {
        return isfinite(value) ? Max(value, lo) : fallback;
    };
    c = coefficients;
    c.pressureLinear = clean(coefficients.pressureLinear, 0.0f, 0.0f);
    c.pressureQuadratic = clean(coefficients.pressureQuadratic, 0.0f, 0.0f);
    c.pressureFalloff = Clamp(clean(coefficients.pressureFalloff, 0.0f, 1.0f), 0.0f, 8.0f);
    c.suctionLinear = clean(coefficients.suctionLinear, 0.0f, 0.0f);
    c.suctionQuadratic = clean(coefficients.suctionQuadratic, 0.0f, 0.0f);
    c.suctionFalloff = Clamp(clean(coefficients.suctionFalloff, 0.0f, 1.0f), 0.0f, 8.0f);
    c.formFactor = clean(coefficients.formFactor, 0.0f, 0.0f);
    c.slamCoefficient = clean(coefficients.slamCoefficient, 0.0f, 0.0f);
    c.slamLength = clean(coefficients.slamLength, 0.0f, 0.0f);
    c.maxSlamPressure = clean(coefficients.maxSlamPressure, 0.0f, 0.0f);
}

void BuoyancyBody::setForwardAxis(const Math::vec3& forward)
{
    if (!finiteVec(forward) || !(Math::dot(forward, forward) > 1.0e-12f))
    {
        Log::error("BuoyancyBody: the forward axis must be finite and non-zero");
        return;
    }
    mForward = Math::normalize(forward);
    mHullLength = mHull.extent(mForward);
    // Beam across the forward axis, horizontally in body space (+Y up).
    Math::vec3 across = Math::cross(Math::vec3(0.0f, 1.0f, 0.0f), mForward);
    if (Math::dot(across, across) < 1.0e-8f)
        across = Math::vec3(1.0f, 0.0f, 0.0f);
    mHullHalfBeam = 0.5f * mHull.extent(across);
}

void BuoyancyBody::setReferenceLength(f32 length)
{
    mReferenceLength = isfinite(length) ? Max(length, 0.0f) : 0.0f;
}

f32 BuoyancyBody::referenceLength() const
{
    return mReferenceLength > 0.0f ? mReferenceLength : mHullLength;
}

void BuoyancyBody::applyForces(f32 step)
{
    Math::vec3 force;
    Math::vec3 torque;
    evaluate(true, step, force, torque);
}

void BuoyancyBody::computeForces(Math::vec3& force, Math::vec3& torque)
{
    evaluate(false, 0.0f, force, torque);
}

void BuoyancyBody::evaluate(bool apply, f32 step, Math::vec3& force, Math::vec3& torque)
{
    force = Math::vec3(0.0f);
    torque = Math::vec3(0.0f);
    mWaterline.clear();
    if (!mBody || !mWater || mHull.triangleCount() == 0)
    {
        mState = BuoyancyState();
        return;
    }
    RigidBody& body = *mBody;
    const WaterSurface& water = *mWater;
    const HydrodynamicCoefficients& k = mCoefficients;
    const f32 time = water.time();
    const f32 rho = water.density();
    const f32 g = water.gravity();
    const f32 rhoG = rho * g;
    const Math::vec3 centre = body.position();
    const Math::quat orientation = body.orientation();
    const Math::vec3 velocity = body.velocity();
    const Math::vec3 spin = body.angularVelocity();

    // Positions relative to the centre of mass: moments come out directly, no precision loss far
    // from the world origin.
    const ct::Vector<Math::vec3>& local = mHull.vertices();
    f32 draft = -1.0e30f;
    for (u32 i = 0; i < local.size(); ++i)
    {
        WetVertex& v = mVertices[i];
        v.position = orientation * local[i];
        const WaterSample sample = water.sample(centre + v.position, time);
        v.depth = sample.height - (centre.y + v.position.y);
        v.flow = sample.velocity;
        draft = Max(draft, v.depth);
    }

    const Math::vec3 flowAtCentre = water.flowVelocity(centre, time);
    const f32 hullSpeed = Math::length(velocity - flowAtCentre);
    const f32 length = referenceLength();
    const f32 reynolds = hullSpeed * length / water.kinematicViscosity();
    const f32 cf = ittc57FrictionCoefficient(reynolds);
    const f32 friction = k.skinFriction ? 0.5f * rho * cf * (1.0f + k.formFactor) : 0.0f;
    const bool slamming = apply && step > 0.0f && mHistoryValid && k.slamCoefficient > 0.0f;
    const f32 slamLength = k.slamLength > 0.0f ? k.slamLength : mHullHalfBeam;

    Math::vec3 hydrostatic(0.0f);
    Math::vec3 hydrostaticMoment(0.0f);
    Math::vec3 pressureDrag(0.0f);
    Math::vec3 frictionForce(0.0f);
    Math::vec3 slamForce(0.0f);
    Math::vec3 dynamicMoment(0.0f);
    // Double for the volume integrals: their terms cancel heavily on a mostly submerged hull.
    f64 volume = 0.0;
    f64 momentX = 0.0;
    f64 momentY = 0.0;
    f64 momentZ = 0.0;
    f64 capArea = 0.0;
    f64 waterlineLength = 0.0;
    f64 waterlineHeight = 0.0;
    f32 wettedArea = 0.0f;
    Math::vec3 flowSum(0.0f);
    u32 wettedTriangles = 0;

    const ct::Vector<u32>& indices = mHull.indices();
    const u32 triangles = mHull.triangleCount();
    for (u32 t = 0; t < triangles; ++t)
    {
        WetVertex corners[3] = {mVertices[indices[3 * t]], mVertices[indices[3 * t + 1]],
                                mVertices[indices[3 * t + 2]]};
        WetTriangle pieces[2];
        Math::vec3 cut[2];
        bool crosses = false;
        const u32 count = clipTriangleToWater(corners, pieces, cut, &crosses);
        mWetted[t] = 0.0f;
        if (count == 0)
            continue;
        const Math::vec3 n = orientation * mHull.triangleNormal(t);
        if (!(Math::dot(n, n) > 0.5f))
            continue;
        ++wettedTriangles;

        f32 triangleArea = 0.0f;
        Math::vec3 triangleCentroid(0.0f);
        Math::vec3 triangleFlow(0.0f);
        for (u32 p = 0; p < count; ++p)
        {
            const WetVertex& a = pieces[p].vertices[0];
            const WetVertex& b = pieces[p].vertices[1];
            const WetVertex& c = pieces[p].vertices[2];
            const f32 area = 0.5f * Math::length(Math::cross(b.position - a.position, c.position - a.position));
            if (!(area > 0.0f))
                continue;
            const f32 depthSum = a.depth + b.depth + c.depth;
            const f32 meanDepth = depthSum / 3.0f;

            const Math::vec3 f = n * (-rhoG * area * meanDepth);
            hydrostatic += f;
            if (depthSum > 0.0f)
            {
                const Math::vec3 sumPosition = a.position + b.position + c.position;
                const Math::vec3 weighted = a.position * a.depth + b.position * b.depth + c.position * c.depth;
                const Math::vec3 pressureCentre = (sumPosition * depthSum + weighted) / (4.0f * depthSum);
                hydrostaticMoment += Math::cross(pressureCentre, f);
            }

            // Volume and first moments by the divergence theorem; the waterplane closing the volume adds
            // only to the y moment.
            volume -= static_cast<f64>(n.y) * area * meanDepth;
            momentX += 0.5 * static_cast<f64>(n.x) * area *
                       meanSquare(a.position.x, b.position.x, c.position.x);
            momentY += 0.5 * static_cast<f64>(n.y) * area *
                       meanSquare(a.position.y, b.position.y, c.position.y);
            momentZ += 0.5 * static_cast<f64>(n.z) * area *
                       meanSquare(a.position.z, b.position.z, c.position.z);

            const Math::vec3 centroid = (a.position + b.position + c.position) / 3.0f;
            const Math::vec3 flow = (a.flow + b.flow + c.flow) / 3.0f;
            const Math::vec3 relative = velocity + Math::cross(spin, centroid) - flow;
            const f32 speed = Math::length(relative);
            Math::vec3 dynamic(0.0f);
            if (speed > kMinSpeed && isfinite(speed))
            {
                const f32 normalSpeed = Math::dot(relative, n);
                const f32 cosine = normalSpeed / speed;
                if (cosine > 0.0f)
                {
                    const f32 pressure = rho * (k.pressureLinear * speed + 0.5f * k.pressureQuadratic * speed * speed) *
                                         falloff(cosine, k.pressureFalloff);
                    const Math::vec3 drag = n * (-pressure * area);
                    pressureDrag += drag;
                    dynamic += drag;
                }
                else if (cosine < 0.0f)
                {
                    f32 suction = rho * (k.suctionLinear * speed + 0.5f * k.suctionQuadratic * speed * speed) *
                                  falloff(cosine, k.suctionFalloff);
                    if (k.ventilation)
                        suction = Min(suction, rhoG * meanDepth);
                    const Math::vec3 drag = n * (suction * area);
                    pressureDrag += drag;
                    dynamic += drag;
                }
                if (friction > 0.0f)
                {
                    const Math::vec3 tangential = relative - n * normalSpeed;
                    const Math::vec3 shear = tangential * (-friction * Math::length(tangential) * area);
                    frictionForce += shear;
                    dynamic += shear;
                }
            }
            dynamicMoment += Math::cross(centroid, dynamic);

            triangleArea += area;
            triangleCentroid += centroid * area;
            triangleFlow += flow * area;
        }
        mWetted[t] = triangleArea;
        wettedArea += triangleArea;
        flowSum += triangleFlow;

        if (crosses)
        {
            const Math::vec3 segment = cut[1] - cut[0];
            const f32 segmentLength = Math::length(segment);
            // Shoelace over the waterline loops: the cut segments bound the waterplane, all traversed
            // the same way round.
            capArea += 0.5 * (static_cast<f64>(cut[0].z) * cut[1].x - static_cast<f64>(cut[0].x) * cut[1].z);
            waterlineLength += segmentLength;
            waterlineHeight += static_cast<f64>(segmentLength) * 0.5 * (cut[0].y + cut[1].y);
            mWaterline.push_back(centre + cut[0]);
            mWaterline.push_back(centre + cut[1]);
        }

        if (slamming && crosses && triangleArea > 0.0f)
        {
            const f32 growth = (triangleArea - mPreviousWetted[t]) / step;
            if (growth > 0.0f)
            {
                const Math::vec3 centroid = triangleCentroid / triangleArea;
                const Math::vec3 flow = triangleFlow / triangleArea;
                const Math::vec3 relative = velocity + Math::cross(spin, centroid) - flow;
                const f32 entry = Math::dot(relative, n);
                if (entry > 0.0f)
                {
                    const f32 magnitude = Min(k.slamCoefficient * rho * slamLength * entry * growth,
                                              k.maxSlamPressure * triangleArea);
                    const Math::vec3 slam = n * (-magnitude);
                    slamForce += slam;
                    dynamicMoment += Math::cross(centroid, slam);
                }
            }
        }
    }

    // The waterplane closing the volume sits at the mean waterline height (the surface itself
    // in flat water).
    const f64 area = fabs(capArea);
    if (waterlineLength > 0.0)
    {
        const f64 level = waterlineHeight / waterlineLength;
        momentY += 0.5 * level * level * area;
    }

    BuoyancyState s;
    s.submergedVolume = static_cast<f32>(volume);
    s.submergedFraction = mHull.volume() > 0.0f ? static_cast<f32>(volume) / mHull.volume() : 0.0f;
    s.waterplaneArea = static_cast<f32>(area);
    s.wettedArea = wettedArea;
    s.draft = local.empty() ? 0.0f : draft;
    s.wettedTriangles = wettedTriangles;
    if (volume > 1.0e-12)
        s.centreOfBuoyancy = centre + Math::vec3(static_cast<f32>(momentX / volume),
                                                 static_cast<f32>(momentY / volume),
                                                 static_cast<f32>(momentZ / volume));
    else
        s.centreOfBuoyancy = centre;
    s.waterVelocity = wettedArea > 0.0f ? flowSum / wettedArea : flowAtCentre;
    s.speedThroughWater = Math::length(velocity - s.waterVelocity);
    s.reynolds = reynolds;
    s.frictionCoefficient = cf;

    Math::vec3 dissipative = pressureDrag + frictionForce + slamForce;
    Math::vec3 dissipativeMoment = dynamicMoment;
    if (apply && step > 0.0f && k.limitDissipation && body.isDynamic())
    {
        // Linear: what is along the motion through the water may at most bring it to rest within
        // the step.
        const Math::vec3 motion = velocity - s.waterVelocity;
        const f32 speed = Math::length(motion);
        if (speed > kMinSpeed)
        {
            const Math::vec3 direction = motion / speed;
            const f32 along = Math::dot(dissipative, direction);
            const f32 limit = -body.mass() * speed / step;
            if (along < limit)
            {
                dissipative += direction * (limit - along);
                s.linearLimit = limit / along;
            }
        }
        const f32 rate = Math::length(spin);
        if (rate > kMinSpeed)
        {
            const Math::vec3 axis = spin / rate;
            const f32 inverseInertia = Math::dot(axis, body.inverseInertiaTensorWorld() * axis);
            if (inverseInertia > 0.0f)
            {
                const f32 along = Math::dot(dissipativeMoment, axis);
                const f32 limit = -rate / (inverseInertia * step);
                if (along < limit)
                {
                    dissipativeMoment += axis * (limit - along);
                    s.angularLimit = limit / along;
                }
            }
        }
    }

    s.hydrostaticForce = hydrostatic;
    s.hydrostaticTorque = hydrostaticMoment;
    s.pressureDragForce = pressureDrag;
    s.frictionForce = frictionForce;
    s.slamForce = slamForce;
    s.dynamicTorque = dissipativeMoment;
    s.force = hydrostatic + dissipative;
    s.torque = hydrostaticMoment + dissipativeMoment;
    if (!finiteVec(s.force) || !finiteVec(s.torque))
    {
        Log::error("BuoyancyBody: non-finite loads - nothing applied this step");
        s.force = Math::vec3(0.0f);
        s.torque = Math::vec3(0.0f);
    }
    mState = s;
    force = s.force;
    torque = s.torque;

    if (apply)
    {
        body.addForce(force);
        body.addTorque(torque);
        for (u32 t = 0; t < triangles; ++t)
            mPreviousWetted[t] = mWetted[t];
        mHistoryValid = true;
    }
}

} // namespace kinetix::Physics
