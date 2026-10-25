#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

#include "beamstructure/BeamStructure.h"

#include <ct/sort.hpp>
#include <math.h>

namespace kinetix::Physics
{

namespace
{
constexpr f32 kDefaultSpring = 9000000.0f;
constexpr f32 kDefaultDamping = 12000.0f;
constexpr f32 kMinBeamLength = 0.1f;
constexpr f32 kSupportLimitDefault = 4.0f;
constexpr f32 kDragCoef = 0.05f;
constexpr f32 kBoundsPad = 0.05f;
constexpr int kMaxRate = 12;

f32 sign(f32 v)
{
    return v >= 0.0f ? 1.0f : -1.0f;
}

//  What a node does against a surface it is in: the force that holds it out of the surface
//  and the friction along it. `velocity` is the speed the node has relative to the surface,
//  `mass` the mass that answers, `penetration` how deep the node is.
Math::vec3 surfaceReaction(const BeamNode& node, const Math::vec3& velocity, f32 mass,
                           const Math::vec3& normal, f32 dt, const ContactSurface& surface,
                           f32 penetration)
{
    Math::vec3 force(0.0f);
    const f32 normalVelocity = Math::dot(velocity, normal);
    const f32 normalForce = Math::dot(node.force, normal);
    if (penetration < 0.0f) return force;

    //  The steady force, and the impact when the node comes in.
    f32 reaction = -normalForce;
    if (normalVelocity < 0.0f)
        reaction -= (0.8f * normalVelocity + 0.2f * (-penetration) / dt) * mass / dt;
    if (reaction <= 0.0f) return force;

    const Math::vec3 slipForce = node.force - normal * normalForce;
    Math::vec3 slip = velocity - normal * normalVelocity;
    const f32 slipSpeed = Math::length(slip);
    if (slipSpeed > 0.0f) slip = slip * (1.0f / slipSpeed);

    //  Slipping slower than the adhesion velocity, with less force along than static friction
    //  holds: the static model. Otherwise the Stribeck curve.
    const f32 moderated = reaction * surface.strength * node.frictionCoef;
    const f32 staticHold = surface.staticFriction * moderated;
    if (slipSpeed < surface.adhesionVelocity && moderated > 0.0f &&
        Math::dot(slipForce, slipForce) <= staticHold * staticHold)
    {
        const f32 friction = -staticHold * (1.0f - expf(-slipSpeed / surface.adhesionVelocity));
        force = force + normal * reaction + slip * friction - slipForce;
    }
    else
    {
        const f32 g = surface.slidingFriction +
                      (surface.staticFriction - surface.slidingFriction) *
                              expf(-powf(slipSpeed / surface.stribeckVelocity, surface.alpha));
        const f32 friction = -(g + Math::min(surface.hydrodynamicFriction * slipSpeed, 5.0f)) * moderated;
        force = force + normal * reaction + slip * friction;
    }
    return force;
}

//  Which side of a triangle a node came in on is not known from the node alone; the nodes it is
//  joined to vote, the node itself three times.
bool backfaceHit(f32 distance, const Math::vec3& normal, const Math::vec3& surfacePoint,
                 const ct::Vector<u32>& neighbours, const BeamStructure& owner)
{
    const int weight = 3;
    int indicator = weight * static_cast<int>(sign(distance));
    if (neighbours.size() > static_cast<usize>(weight))
        for (u32 id: neighbours)
            indicator += static_cast<int>(sign(Math::dot(normal, owner.node(id).position - surfacePoint)));
    return indicator < 0;
}

//  A point against a triangle: where it falls on the triangle (alpha for a, beta for b, gamma
//  for c) and how far it is from the plane, along the normal.
struct TriangleCoord
{
    f32 alpha, beta, gamma, distance;
};

struct TriangleFrame
{
    Math::vec3 a, b, c;
    Math::vec3 u, v;
    Math::vec3 n;
    Math::vec3 vn;
    f32 det;

    TriangleFrame(const Math::vec3& a_, const Math::vec3& b_, const Math::vec3& c_)
        : a(a_), b(b_), c(c_), u(a_ - c_), v(b_ - c_)
    {
        n = Math::cross(u, v);
        const f32 length = Math::length(n);
        if (length > 0.0f) n = n * (1.0f / length);
        vn = Math::cross(v, n);
        det = Math::dot(u, vn);
    }

    //  Solves  u * alpha + v * beta + n * distance = p - c.
    TriangleCoord operator()(const Math::vec3& p) const
    {
        const Math::vec3 w = p - c;
        const f32 inverse = det != 0.0f ? 1.0f / det : 0.0f;
        const f32 alpha = Math::dot(w, vn) * inverse;
        const f32 beta = Math::dot(u, Math::cross(w, n)) * inverse;
        const f32 distance = Math::dot(u, Math::cross(v, w)) * inverse;
        return { alpha, beta, 1.0f - alpha - beta, distance };
    }
};

bool insideTriangle(const TriangleCoord& local, f32 margin)
{
    return local.alpha >= 0.0f && local.beta >= 0.0f && local.gamma >= 0.0f &&
           fabsf(local.distance) <= margin;
}

//  The force of a node in a triangle: on the node, and the opposite on the three nodes of the
//  triangle, shared by where the node is on it.
void collisionForces(f32 penetration, BeamNode& hit, BeamNode& na, BeamNode& nb, BeamNode& no,
                     const TriangleCoord& coord, const Math::vec3& normal, f32 dt,
                     const ContactSurface& surface)
{
    const Math::vec3 velocity =
            hit.velocity - (na.velocity * coord.alpha + nb.velocity * coord.beta + no.velocity * coord.gamma);
    const f32 triangleMass = na.mass * coord.alpha + nb.mass * coord.beta + no.mass * coord.gamma;
    const f32 mass = (hit.mass * triangleMass) / (hit.mass + triangleMass);
    const Math::vec3 force = surfaceReaction(hit, velocity, mass, normal, dt, surface, penetration);
    hit.force = hit.force + force;
    na.force = na.force - force * coord.alpha;
    nb.force = nb.force - force * coord.beta;
    no.force = no.force - force * coord.gamma;
}

struct Point
{
    BeamStructure* owner;
    u32 node;
    Math::vec3 position;
};

bool boxesMeet(const Math::vec3& lowA, const Math::vec3& highA, const Math::vec3& lowB,
               const Math::vec3& highB)
{
    return lowA.x <= highB.x && highA.x >= lowB.x && lowA.y <= highB.y && highA.y >= lowB.y &&
           lowA.z <= highB.z && highA.z >= lowB.z;
}
} // namespace

u32 BeamStructure::addNode(const Math::vec3& position)
{
    BeamNode node;
    node.position = position;
    mNodes.push_back(node);
    return static_cast<u32>(mNodes.size() - 1);
}

u32 BeamStructure::addBeam(u32 a, u32 b, BeamKind kind, BeamBound bound, f32 spring, f32 damping,
                           f32 strength, f32 deformThreshold, f32 plasticCoef)
{
    Beam beam;
    beam.a = a;
    beam.b = b;
    beam.kind = kind;
    beam.bound = bound;
    beam.spring = spring;
    beam.damping = damping;
    beam.strength = strength;
    beam.maxPositiveStress = deformThreshold;
    beam.maxNegativeStress = -deformThreshold;
    beam.minMaxStress = deformThreshold;
    beam.plasticCoef = plasticCoef;
    beam.length = Math::length(mNodes[a].position - mNodes[b].position);
    beam.referenceLength = beam.length;
    mBeams.push_back(beam);
    return static_cast<u32>(mBeams.size() - 1);
}

void BeamStructure::addTriangle(u32 a, u32 b, u32 c, bool collides)
{
    BeamTriangle triangle;
    triangle.a = a;
    triangle.b = b;
    triangle.c = c;
    triangle.collides = collides;
    mTriangles.push_back(triangle);
}

void BeamStructure::countNeighbours()
{
    mNeighbours.clear();
    mNodeBeams.clear();
    mNeighbours.resize(mNodes.size());
    mNodeBeams.resize(mNodes.size());
    for (u32 i = 0; i < mBeams.size(); ++i)
    {
        mNeighbours[mBeams[i].a].push_back(mBeams[i].b);
        mNodeBeams[mBeams[i].a].push_back(i);
        mNeighbours[mBeams[i].b].push_back(mBeams[i].a);
        mNodeBeams[mBeams[i].b].push_back(i);
    }
}

void BeamStructure::finish(f32 dryMass, f32 loadMass, f32 collisionRange, bool minimassSkipsLoaded)
{
    mCollisionRange = collisionRange;
    countNeighbours();

    //  The load goes to the loaded nodes that have no load of their own.
    int shared = 0;
    for (const BeamNode& node: mNodes)
        if (!node.tyre && node.loaded && node.overrideMass < 0.0f) ++shared;
    for (BeamNode& node: mNodes)
    {
        if (node.tyre) continue;
        if (!node.loaded) node.mass = 0.0f;
        else if (node.overrideMass < 0.0f) node.mass = loadMass / static_cast<f32>(shared);
        else node.mass = node.overrideMass;
    }

    //  The dry mass is spread over the beams, by their length, and each beam gives half of its
    //  share to each node of it that is not a tyre.
    f32 length = 0.0f;
    for (const Beam& beam: mBeams)
    {
        if (beam.kind == BeamKind::Virtual) continue;
        const f32 half = beam.referenceLength * 0.5f;
        if (!mNodes[beam.a].tyre) length += half;
        if (!mNodes[beam.b].tyre) length += half;
    }
    for (const Beam& beam: mBeams)
    {
        if (beam.kind == BeamKind::Virtual) continue;
        const f32 half = beam.referenceLength * dryMass / length / 2.0f;
        if (!mNodes[beam.a].tyre) mNodes[beam.a].mass += half;
        if (!mNodes[beam.b].tyre) mNodes[beam.b].mass += half;
    }
    for (BeamNode& node: mNodes)
        if (!node.tyre && !(minimassSkipsLoaded && node.loaded) && node.mass < node.minimass)
            node.mass = node.minimass;
    mTotalMass = 0.0f;
    for (const BeamNode& node: mNodes) mTotalMass += node.mass;

    //  What touches what: the nodes of the collision triangles, the contacters, and the nodes the
    //  triangles of the others are tested with.
    bool anyTriangle = false;
    for (const BeamTriangle& triangle: mTriangles)
    {
        if (!triangle.collides) continue;
        anyTriangle = true;
        mNodes[triangle.a].cabNode = true;
        mNodes[triangle.b].cabNode = true;
        mNodes[triangle.c].cabNode = true;
    }
    for (BeamNode& node: mNodes)
        if (!node.contacter && !node.noGroundContact && (node.cabNode || !anyTriangle))
            node.contactable = true;
    updateBounds();
}

f32 BeamStructure::random()
{
    mRandomState ^= mRandomState << 13;
    mRandomState ^= mRandomState >> 17;
    mRandomState ^= mRandomState << 5;
    return static_cast<f32>(mRandomState & 0xFFFFFFu) / static_cast<f32>(0x7FFFFFu) - 1.0f;
}

void BeamStructure::stepNodes(f32 dt, const BeamGround& ground, const Math::vec3& gravity)
{
    for (BeamNode& node: mNodes)
    {
        if (!node.noGroundContact)
        {
            bool contacted = false;
            const f32 height = ground.heightAt(node.position.x, node.position.z);
            if (height > node.position.y)
            {
                const Math::vec3 normal = ground.normalAt(node.position.x, height, node.position.z);
                node.force = node.force + surfaceReaction(node, node.velocity, node.mass, normal, dt,
                                                          ground.surfaceAt(node.position.x, node.position.z),
                                                          height - node.position.y);
                contacted = true;
            }
            node.hasGroundContact = contacted;
        }

        node.velocity = node.velocity + node.force * (1.0f / node.mass * dt);
        node.position = node.position + node.velocity * dt;

        //  The next step starts from gravity, with a viscous drag that has a little turbulence.
        node.force = Math::vec3(0.0f, node.mass * gravity.y, 0.0f);
        const f32 speed = Math::length(node.velocity);
        const f32 drag = kDragCoef * speed;
        Math::vec3 total = node.velocity * (-drag);
        const f32 turbulence = drag * speed * 0.005f;
        total = total + Math::vec3(random(), random(), random()) * turbulence;
        node.force = node.force + total;
    }
}

void BeamStructure::updateBounds()
{
    if (mNodes.empty()) return;
    mBoundsMin = mNodes[0].position;
    mBoundsMax = mNodes[0].position;
    for (const BeamNode& node: mNodes)
    {
        mBoundsMin = Math::vec3(Math::min(mBoundsMin.x, node.position.x), Math::min(mBoundsMin.y, node.position.y),
                                Math::min(mBoundsMin.z, node.position.z));
        mBoundsMax = Math::vec3(Math::max(mBoundsMax.x, node.position.x), Math::max(mBoundsMax.y, node.position.y),
                                Math::max(mBoundsMax.z, node.position.z));
    }
    mBoundsMin = mBoundsMin - Math::vec3(kBoundsPad);
    mBoundsMax = mBoundsMax + Math::vec3(kBoundsPad);
}

void BeamStructure::stepBeams()
{
    for (u32 i = 0; i < mBeams.size(); ++i)
    {
        Beam& beam = mBeams[i];
        if (beam.disabled) continue;
        BeamNode& p1 = mNodes[beam.a];
        BeamNode& p2 = mNodes[beam.b];

        const Math::vec3 dis = p1.position - p2.position;
        f32 dislen = Math::dot(dis, dis);
        if (dislen <= 0.0f) continue;
        const f32 inverted = 1.0f / sqrtf(dislen);
        dislen *= inverted;

        const f32 difftoBeamL = dislen - beam.length;
        f32 k = beam.spring;
        f32 d = beam.damping;
        const f32 v = Math::dot(p1.velocity - p2.velocity, dis) * inverted;

        if (beam.bound == BeamBound::Shock)
        {
            //  Outside its bounds a shock goes over to the stiff spring and damper.
            f32 ratio = 0.0f;
            if (difftoBeamL > beam.longBound * beam.length)
                ratio = difftoBeamL - beam.longBound * beam.length;
            else if (difftoBeamL < -beam.shortBound * beam.length)
                ratio = -difftoBeamL - beam.shortBound * beam.length;
            if (ratio != 0.0f)
            {
                f32 boundSpring = kDefaultSpring;
                f32 boundDamping = kDefaultDamping;
                if (beam.kind == BeamKind::Hydro)
                {
                    boundSpring = beam.boundSpring;
                    boundDamping = beam.boundDamping;
                }
                k += (boundSpring - k) * ratio;
                d += (boundDamping - d) * ratio;
            }
        }
        else if (beam.bound == BeamBound::Support)
        {
            if (difftoBeamL > 0.0f)
            {
                k = 0.0f;
                d *= 0.1f;
                const f32 limit = beam.longBound > 0.0f ? beam.longBound : kSupportLimitDefault;
                if (difftoBeamL > beam.length * limit)
                {
                    beam.broken = true;
                    beam.disabled = true;
                }
            }
        }
        else if (beam.bound == BeamBound::Rope)
        {
            if (difftoBeamL < 0.0f)
            {
                k = 0.0f;
                d *= 0.1f;
            }
        }

        f32 slen = -k * difftoBeamL - d * v;
        beam.stress = slen;

        f32 len = fabsf(slen);
        if (len > beam.minMaxStress)
        {
            if (beam.kind == BeamKind::Normal && beam.bound != BeamBound::Shock && k != 0.0f)
            {
                if (slen > beam.maxPositiveStress && difftoBeamL < 0.0f) // compression
                {
                    const f32 yieldLength = beam.maxPositiveStress / k;
                    const f32 deform = difftoBeamL + yieldLength * (1.0f - beam.plasticCoef);
                    const f32 oldLength = beam.length;
                    beam.length += deform;
                    beam.length = Math::max(kMinBeamLength, beam.length);
                    slen = slen - (slen - beam.maxPositiveStress) * 0.5f;
                    len = slen;
                    if (beam.length > 0.0f && oldLength > beam.length)
                    {
                        beam.maxPositiveStress *= oldLength / beam.length;
                        beam.minMaxStress = Math::min(beam.maxPositiveStress, -beam.maxNegativeStress);
                        beam.minMaxStress = Math::min(beam.minMaxStress, beam.strength);
                    }
                }
                else if (slen < beam.maxNegativeStress && difftoBeamL > 0.0f) // extension
                {
                    const f32 yieldLength = beam.maxNegativeStress / k;
                    const f32 deform = difftoBeamL + yieldLength * (1.0f - beam.plasticCoef);
                    const f32 oldLength = beam.length;
                    beam.length += deform;
                    slen = slen - (slen - beam.maxNegativeStress) * 0.5f;
                    len = -slen;
                    if (oldLength > 0.0f && beam.length > oldLength)
                    {
                        beam.maxNegativeStress *= beam.length / oldLength;
                        beam.minMaxStress = Math::min(beam.maxPositiveStress, -beam.maxNegativeStress);
                        beam.minMaxStress = Math::min(beam.minMaxStress, beam.strength);
                    }
                    beam.strength -= deform * k;
                }
            }

            if (len > beam.strength)
            {
                //  A beam is let go unless it is on a collision triangle whose node has no more
                //  than two beams left, which would let the triangle come apart.
                const auto live = [this](u32 node)
                {
                    u32 count = 0;
                    for (u32 j: mNodeBeams[node])
                        if (!mBeams[j].disabled) ++count;
                    return count;
                };
                if (!((p1.cabNode && live(beam.a) < 3) || (p2.cabNode && live(beam.b) < 3)))
                {
                    slen = 0.0f;
                    beam.broken = true;
                    beam.disabled = true;
                }
                else
                    beam.strength = 2.0f * beam.minMaxStress;
            }
        }

        const Math::vec3 force = dis * (slen * inverted);
        p1.force = p1.force + force;
        p2.force = p2.force - force;
    }
}

void BeamStructure::stepIntraCollisions(f32 dt, const ContactSurface& surface)
{
    for (BeamNode& node: mNodes) node.hasMeshContact = false;

    //  The nodes that may hit the structure's own triangles: the contacters.
    ct::Vector<u32> contacters;
    for (u32 i = 0; i < mNodes.size(); ++i)
        if (mNodes[i].contacter) contacters.push_back(i);
    if (contacters.empty()) return;

    for (BeamTriangle& triangle: mTriangles)
    {
        if (!triangle.collides) continue;
        if (triangle.intraRate > 0)
        {
            ++triangle.intraDistance;
            --triangle.intraRate;
            continue;
        }
        if (triangle.intraDistance > 0)
        {
            triangle.intraRate = Math::min(triangle.intraDistance, kMaxRate);
            triangle.intraDistance = 0;
        }

        BeamNode& no = mNodes[triangle.a];
        BeamNode& na = mNodes[triangle.b];
        BeamNode& nb = mNodes[triangle.c];
        const Math::vec3 low(Math::min(no.position.x, Math::min(na.position.x, nb.position.x)) - mCollisionRange,
                             Math::min(no.position.y, Math::min(na.position.y, nb.position.y)) - mCollisionRange,
                             Math::min(no.position.z, Math::min(na.position.z, nb.position.z)) - mCollisionRange);
        const Math::vec3 high(Math::max(no.position.x, Math::max(na.position.x, nb.position.x)) + mCollisionRange,
                              Math::max(no.position.y, Math::max(na.position.y, nb.position.y)) + mCollisionRange,
                              Math::max(no.position.z, Math::max(na.position.z, nb.position.z)) + mCollisionRange);

        const TriangleFrame frame(na.position, nb.position, no.position);
        bool collision = false;
        for (u32 index: contacters)
        {
            BeamNode& hit = mNodes[index];
            const Math::vec3& p = hit.position;
            if (p.x < low.x || p.x > high.x || p.y < low.y || p.y > high.y || p.z < low.z || p.z > high.z)
                continue;
            if (hit.tyre) continue;
            if (index == triangle.a || index == triangle.b || index == triangle.c) continue;
            const TriangleCoord local = frame(p);
            if (!insideTriangle(local, mCollisionRange)) continue;
            collision = true;
            Math::vec3 normal = frame.n;
            f32 distance = local.distance;
            if (distance < 0.0f)
            {
                normal = normal * -1.0f;
                distance = -distance;
            }
            collisionForces(mCollisionRange - distance, hit, na, nb, no, local, normal, dt, surface);
        }
        if (collision) triangle.intraRate = -20000;
        else ++triangle.intraRate;
    }
}

void BeamStructure::resolveInterCollisions(f32 dt, const ContactSurface& surface,
                                           const ct::Vector<BeamStructure*>& structures)
{
    //  The points of the structures that are near: their contacters and contactable nodes.
    ct::Vector<Point> points;
    for (BeamStructure* other: structures)
    {
        if (other == this) continue;
        if (!boxesMeet(mBoundsMin, mBoundsMax, other->mBoundsMin, other->mBoundsMax)) continue;
        if (!mNodes.empty() && !other->mNodes.empty() &&
            Math::dot(mNodes[0].velocity - other->mNodes[0].velocity, mNodes[0].velocity - other->mNodes[0].velocity) > 16.0f)
        {
            for (BeamTriangle& t: mTriangles) t.interRate = 0;
            for (BeamTriangle& t: other->mTriangles) t.interRate = 0;
        }
        for (u32 i = 0; i < other->mNodes.size(); ++i)
            if (other->mNodes[i].contacter || other->mNodes[i].contactable)
                points.push_back({ other, i, other->mNodes[i].position });
    }
    if (points.empty()) return;
    ct::sort(points.begin(), points.end(), [](const Point& a, const Point& b) { return a.position.x < b.position.x; });

    for (BeamTriangle& triangle: mTriangles)
    {
        if (!triangle.collides) continue;
        if (triangle.interRate > 0)
        {
            ++triangle.interDistance;
            --triangle.interRate;
            continue;
        }
        triangle.interRate = Math::min(triangle.interDistance, kMaxRate);
        triangle.interDistance = 0;

        BeamNode& no = mNodes[triangle.a];
        BeamNode& na = mNodes[triangle.b];
        BeamNode& nb = mNodes[triangle.c];
        const Math::vec3 low(Math::min(no.position.x, Math::min(na.position.x, nb.position.x)) - mCollisionRange,
                             Math::min(no.position.y, Math::min(na.position.y, nb.position.y)) - mCollisionRange,
                             Math::min(no.position.z, Math::min(na.position.z, nb.position.z)) - mCollisionRange);
        const Math::vec3 high(Math::max(no.position.x, Math::max(na.position.x, nb.position.x)) + mCollisionRange,
                              Math::max(no.position.y, Math::max(na.position.y, nb.position.y)) + mCollisionRange,
                              Math::max(no.position.z, Math::max(na.position.z, nb.position.z)) + mCollisionRange);

        //  The first point at or past the low edge, by x.
        usize lo = 0;
        usize hi = points.size();
        while (lo < hi)
        {
            const usize mid = (lo + hi) / 2;
            if (points[mid].position.x < low.x) lo = mid + 1;
            else hi = mid;
        }
        bool any = false;
        const TriangleFrame frame(na.position, nb.position, no.position);
        for (usize h = lo; h < points.size() && points[h].position.x <= high.x; ++h)
        {
            const Math::vec3& p = points[h].position;
            if (p.y < low.y || p.y > high.y || p.z < low.z || p.z > high.z) continue;
            any = true;
            BeamStructure& owner = *points[h].owner;
            BeamNode& hit = owner.mNodes[points[h].node];
            const TriangleCoord local = frame(hit.position);
            if (!insideTriangle(local, mCollisionRange)) continue;
            triangle.interRate = 0;

            Math::vec3 normal = frame.n;
            f32 distance = local.distance;
            if (backfaceHit(distance, normal, no.position, owner.mNeighbours[points[h].node], owner))
            {
                normal = normal * -1.0f;
                distance = -distance;
            }
            collisionForces(mCollisionRange - distance, hit, na, nb, no, local, normal, dt, surface);
            hit.hasMeshContact = true;
            na.hasMeshContact = true;
            nb.hasMeshContact = true;
            no.hasMeshContact = true;
        }
        if (!any) ++triangle.interRate;
    }
}

void BeamStructure::place(const Math::vec3& offset, const Math::vec3& velocity)
{
    for (BeamNode& node: mNodes)
    {
        node.position = node.position + offset;
        node.velocity = velocity;
    }
    updateBounds();
}

u32 BeamStructure::brokenBeams() const
{
    u32 count = 0;
    for (const Beam& beam: mBeams)
        if (beam.broken) ++count;
    return count;
}

f32 BeamStructure::worstYield() const
{
    f32 worst = 0.0f;
    for (const Beam& beam: mBeams)
    {
        if (beam.referenceLength <= 0.0f || beam.kind != BeamKind::Normal) continue;
        const f32 change = fabsf(beam.length - beam.referenceLength) / beam.referenceLength;
        if (change > worst) worst = change;
    }
    return worst;
}

Math::vec3 BeamStructure::centre() const
{
    Math::vec3 sum(0.0f);
    f32 mass = 0.0f;
    for (const BeamNode& node: mNodes)
    {
        sum = sum + node.position * node.mass;
        mass += node.mass;
    }
    return mass > 0.0f ? sum * (1.0f / mass) : sum;
}

Math::vec3 BeamStructure::averageVelocity() const
{
    Math::vec3 sum(0.0f);
    f32 mass = 0.0f;
    for (const BeamNode& node: mNodes)
    {
        sum = sum + node.velocity * node.mass;
        mass += node.mass;
    }
    return mass > 0.0f ? sum * (1.0f / mass) : sum;
}

bool BeamStructure::finite() const
{
    for (const BeamNode& node: mNodes)
        if (!isfinite(node.position.x) || !isfinite(node.position.y) || !isfinite(node.position.z))
            return false;
    return true;
}

void BeamWorld::step()
{
    for (BeamStructure* structure: mStructures)
    {
        structure->stepNodes(kStep, *mGround, mGravity);
        structure->updateBounds();
        structure->stepBeams();
        structure->stepIntraCollisions(kStep, mStructureSurface);
    }
    for (BeamStructure* structure: mStructures)
        structure->resolveInterCollisions(kStep, mStructureSurface, mStructures);
}

} // namespace kinetix::Physics
