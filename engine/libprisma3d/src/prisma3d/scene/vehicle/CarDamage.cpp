#include "CarDamage.h"

#include "prisma3d/main/Main.h"

#include <math.h>

namespace prisma3d
{

namespace
{

constexpr float kWeightPerImpulse = 1.0f / 6500.0f;
constexpr float kReach = 1.5f;
constexpr float kGivePerSecond = 9.0f;
constexpr float kGlassBreaks = 0.3f;

float clamp01(float v)
{
    return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v);
}

float scatter(const Math::Vec3& p, float seed)
{
    const float s = sinf(p.x * 12.9898f + p.y * 78.233f + p.z * 37.719f + seed) * 43758.5453f;
    return (s - floorf(s)) * 2.0f - 1.0f;
}

void rebuildNormals(const SurfaceData& base, SurfaceData& shown, const ct::Vector<float>& weight)
{
    const std::size_t count = shown.positions.size();
    ct::Vector<Math::Vec3> sum;
    sum.resize(count, Math::Vec3(0.0f, 0.0f, 0.0f));
    for (std::size_t i = 0; i + 2 < shown.indices.size(); i += 3)
    {
        const std::uint32_t a = shown.indices[i];
        const std::uint32_t b = shown.indices[i + 1];
        const std::uint32_t c = shown.indices[i + 2];
        const Math::Vec3 n = (shown.positions[b] - shown.positions[a])
                                     .Cross(shown.positions[c] - shown.positions[a]);
        sum[a] = sum[a] + n;
        sum[b] = sum[b] + n;
        sum[c] = sum[c] + n;
    }
    for (std::size_t i = 0; i < count; ++i)
    {
        if (weight[i] <= 0.0f)
        {
            shown.normals[i] = base.normals[i];
            continue;
        }
        const Math::Vec3 rebuilt = sum[i].NormalizedSafe();
        const float mix = clamp01(weight[i] * 3.0f);
        shown.normals[i] = (base.normals[i] * (1.0f - mix) + rebuilt * mix).NormalizedSafe();
    }
}

Math::Vec3 centroidOf(const SurfaceData& d, std::size_t triangle)
{
    return (d.positions[d.indices[triangle * 3]] + d.positions[d.indices[triangle * 3 + 1]] +
                   d.positions[d.indices[triangle * 3 + 2]]) *
           (1.0f / 3.0f);
}

} // namespace

MeshId CarDamage::addPiece(const ct::String& name, PieceKind kind,
        const ct::Vector<CarPieceSurface>& parts, const ct::Vector<MaterialId>& materials,
        const Math::Mat4& bake)
{
    MeshStore& meshes = *Main::meshes();
    mPieces.push_back(Piece());
    Piece& piece = mPieces[mPieces.size() - 1];
    piece.kind = kind;
    piece.mesh = meshes.create(name.c_str());

    for (std::size_t source = 0; source < parts.size(); ++source)
    {
        Part part;
        part.base = parts[source].surface;
        if (source < materials.size()) part.material = materials[source];
        SurfaceData& d = part.base;
        const bool hasNormals = d.normals.size() == d.positions.size();
        if (!hasNormals) d.normals.resize(d.positions.size(), Math::Vec3(0.0f, 1.0f, 0.0f));
        Math::Vec3 low(1.0e9f, 1.0e9f, 1.0e9f);
        Math::Vec3 high(-1.0e9f, -1.0e9f, -1.0e9f);
        for (std::size_t i = 0; i < d.positions.size(); ++i)
        {
            d.positions[i] = bake.TransformPoint(d.positions[i]);
            d.normals[i] = bake.TransformVector(d.normals[i]).NormalizedSafe();
            if (i < d.tangents.size())
            {
                const Math::Vec3 t = bake.TransformVector(
                        Math::Vec3(d.tangents[i].x, d.tangents[i].y, d.tangents[i].z));
                d.tangents[i] = Math::Vec4(t.x, t.y, t.z, d.tangents[i].w);
            }
            low = Math::Vec3(fminf(low.x, d.positions[i].x), fminf(low.y, d.positions[i].y),
                    fminf(low.z, d.positions[i].z));
            high = Math::Vec3(fmaxf(high.x, d.positions[i].x), fmaxf(high.y, d.positions[i].y),
                    fmaxf(high.z, d.positions[i].z));
        }
        part.shown = d;
        part.weight.resize(d.positions.size(), 0.0f);
        part.drawn.resize(d.positions.size(), 0.0f);
        part.displacement.resize(d.positions.size(), Math::Vec3(0.0f, 0.0f, 0.0f));
        part.gone.resize(d.indices.size() / 3, 0);
        part.looseOf.resize(d.indices.size() / 3, -1);
        part.dentVec.resize(d.positions.size(), Math::Vec3(0.0f, 0.0f, 0.0f));
        part.dentMark.resize(d.positions.size(), 0.0f);
        part.mix.resize(d.positions.size(), 0.0f);

        SurfaceLayout room;
        room.attributeMask = kVertexPosition | kVertexNormal;
        if (!d.tangents.empty()) room.attributeMask |= kVertexTangent;
        if (!d.colors.empty()) room.attributeMask |= kVertexColor;
        if (!d.uvs.empty()) room.attributeMask |= kVertexUv;
        room.vertexCount = static_cast<std::uint32_t>(d.positions.size());
        room.indexCount = static_cast<std::uint32_t>(d.indices.size());
        if (room.vertexCount > 65535) room.indexFormat = prisma::IndexFormat::UInt32;
        room.aabb =
                Math::Box(low - Math::Vec3(1.5f, 1.5f, 1.5f), high + Math::Vec3(1.5f, 1.5f, 1.5f));
        const int surface =
                meshes.addDynamicSurface(piece.mesh, room, parts[source].material.c_str());
        if (surface < 0) continue;
        part.surface = static_cast<std::uint32_t>(surface);
        meshes.updateSurface(piece.mesh, part.surface, part.shown);
        if (source < materials.size())
            meshes.setSurfaceMaterial(piece.mesh, part.surface, materials[source]);
        piece.parts.push_back(part);
    }
    return piece.mesh;
}

Math::Vec3 CarDamage::crumple(const Math::Vec3& p) const
{
    const float length = mHigh.z - mLow.z;
    const float height = mHigh.y - mLow.y;
    const Math::Vec3 centre = (mLow + mHigh) * 0.5f;
    const float halfWidth = 0.5f * (mHigh.x - mLow.x);
    const float u = (p.x - centre.x) / halfWidth;

    const float noseStart = centre.z + 0.12f * length;
    const float tailStart = centre.z - 0.16f * length;
    const float nose = powf(clamp01((p.z - noseStart) / (mHigh.z - noseStart)), 1.4f);
    const float tail = powf(clamp01((tailStart - p.z) / (tailStart - mLow.z)), 1.4f);
    const float flank =
            powf(clamp01((fabsf(u) - 0.55f) / 0.45f), 1.2f) * (1.0f - fmaxf(nose, tail));
    const float roof = powf(clamp01((p.y - (mHigh.y - 0.22f * height)) / (0.22f * height)), 1.2f) *
                       (1.0f - fmaxf(nose, tail));
    const float ends = fmaxf(nose, tail);
    const float rough = fmaxf(ends, flank);

    Math::Vec3 d(0.0f, 0.0f, 0.0f);
    d.z = -0.20f * length * nose + 0.18f * length * tail;
    d.x = -u * halfWidth * 0.10f * (nose + tail);
    d.y = (0.10f * nose + 0.06f * tail) * (1.0f - u * u);
    d.x += (u < 0.0f ? 1.0f : -1.0f) * 0.30f * flank;
    d.y -= 0.26f * roof;
    const float crease = sinf(9.0f * p.x + 3.0f * p.y) * cosf(7.0f * p.z + 2.0f * p.x);
    d.x += 0.05f * rough * crease;
    d.y += 0.05f * rough * crease * cosf(5.0f * p.x);
    d.z += 0.06f * ends * crease;
    d = d + Math::Vec3(scatter(p, 1.0f), scatter(p, 2.0f), scatter(p, 3.0f)) * (0.025f * rough);
    return d;
}

void CarDamage::findLoose()
{
    mLoose.clear();
    mBreakParts.clear();
    const float height = mHigh.y - mLow.y;
    const float middle = 0.5f * (mLow.z + mHigh.z);
    const float length = mHigh.z - mLow.z;
    for (std::size_t pi = 0; pi < mPieces.size(); ++pi)
    {
        if (mPieces[pi].kind != PieceKind::Body) continue;
        Piece& piece = mPieces[pi];

        float widest = 0.0f;
        for (const Part& part: piece.parts)
            for (const Math::Vec3& p: part.base.positions) widest = fmaxf(widest, fabsf(p.x));

        struct Cluster
        {
            std::size_t part = 0;
            ct::Vector<std::uint32_t> triangles;
            Math::Vec3 low = Math::Vec3(1.0e9f, 1.0e9f, 1.0e9f);
            Math::Vec3 high = Math::Vec3(-1.0e9f, -1.0e9f, -1.0e9f);
            bool used = false;
        };
        ct::Vector<Cluster> clusters;
        ct::Vector<ct::Vector<unsigned char>> taken;
        for (std::size_t pa = 0; pa < piece.parts.size(); ++pa)
        {
            const Part& part = piece.parts[pa];
            const std::size_t n = part.base.positions.size();
            const std::size_t triangles = part.base.indices.size() / 3;
            taken.push_back(ct::Vector<unsigned char>());
            taken[pa].resize(triangles, 0);
            ct::Vector<std::size_t> root;
            for (std::size_t i = 0; i < n; ++i) root.push_back(i);
            auto find = [&root](std::size_t a)
            {
                while (root[a] != a)
                {
                    root[a] = root[root[a]];
                    a = root[a];
                }
                return a;
            };
            for (std::size_t t = 0; t < triangles; ++t)
                for (std::size_t k = 1; k < 3; ++k)
                {
                    const std::size_t a = find(part.base.indices[t * 3]);
                    const std::size_t b = find(part.base.indices[t * 3 + k]);
                    if (a != b) root[a] = b;
                }
            ct::Vector<std::size_t> clusterOf;
            clusterOf.resize(n, static_cast<std::size_t>(-1));
            for (std::size_t t = 0; t < triangles; ++t)
            {
                const std::size_t id = find(part.base.indices[t * 3]);
                if (clusterOf[id] == static_cast<std::size_t>(-1))
                {
                    clusterOf[id] = clusters.size();
                    clusters.push_back(Cluster());
                    clusters[clusterOf[id]].part = pa;
                }
                Cluster& c = clusters[clusterOf[id]];
                c.triangles.push_back(static_cast<std::uint32_t>(t));
                for (std::size_t k = 0; k < 3; ++k)
                {
                    const Math::Vec3& v = part.base.positions[part.base.indices[t * 3 + k]];
                    c.low = Math::Vec3(fminf(c.low.x, v.x), fminf(c.low.y, v.y),
                            fminf(c.low.z, v.z));
                    c.high = Math::Vec3(fmaxf(c.high.x, v.x), fmaxf(c.high.y, v.y),
                            fmaxf(c.high.z, v.z));
                }
            }
        }

        auto takeCluster = [&](Cluster& c, Loose& loose)
        {
            Slice slice;
            slice.piece = pi;
            slice.part = c.part;
            slice.triangles = c.triangles;
            loose.slices.push_back(slice);
            for (std::uint32_t t: c.triangles) taken[c.part][t] = 1;
            c.used = true;
        };

        for (Cluster& c: clusters)
        {
            const Math::Vec3 centre = (c.low + c.high) * 0.5f;
            const float reach = fmaxf(fabsf(c.low.x), fabsf(c.high.x));
            if (c.triangles.size() < 40 || c.triangles.size() > 900) continue;
            if (reach < 0.97f * widest || centre.z < middle || centre.y < mLow.y + 0.35f * height)
                continue;
            Loose loose;
            takeCluster(c, loose);
            mLoose.push_back(loose);
            BreakPart bp;
            bp.kind = BreakPart::Kind::Mirror;
            bp.centre = centre;
            bp.halfExtents = (c.high - c.low) * 0.5f;
            bp.mass = 1.5f;
            bp.breakForce = 1200.0f;
            bp.side = centre.x >= 0.0f ? 1.0f : -1.0f;
            mBreakParts.push_back(bp);
        }

        const std::size_t before = clusters.size();
        for (std::size_t ci = 0; ci < before; ++ci)
        {
            Cluster& c = clusters[ci];
            if (c.used || c.triangles.size() < 250 || c.triangles.size() > 1100) continue;
            const Math::Vec3 centre = (c.low + c.high) * 0.5f;
            const Math::Vec3 size = c.high - c.low;
            const bool door = size.x <= 0.4f && size.z >= 0.7f && size.z <= 1.7f &&
                              size.y >= 0.5f && fabsf(centre.x) >= 0.55f * widest &&
                              centre.z > mLow.z + 0.25f * length &&
                              centre.z < mHigh.z - 0.28f * length;
            const bool hood = size.x >= 1.2f && size.z >= 0.8f && size.z <= 1.6f &&
                              centre.z > middle + 0.3f * length &&
                              centre.y > mLow.y + 0.35f * height &&
                              c.low.y > mLow.y + 0.25f * height;
            if (!door && !hood) continue;
            Loose loose;
            takeCluster(c, loose);
            BreakPart bp;
            bp.halfExtents = Math::Vec3(fmaxf(size.x * 0.5f, 0.05f), size.y * 0.5f, size.z * 0.5f);
            bp.centre = centre;
            if (door)
            {
                const float side = centre.x >= 0.0f ? 1.0f : -1.0f;
                bp.kind = BreakPart::Kind::Door;
                bp.side = side;
                bp.mass = 22.0f;
                bp.breakForce = 26000.0f;
                bp.latchForce = 2500.0f;
                bp.hingePoint = Math::Vec3(side * fmaxf(fabsf(c.low.x), fabsf(c.high.x)), centre.y,
                        c.high.z);
                bp.hingeAxis = Math::Vec3(0.0f, 1.0f, 0.0f);
                bp.openMin = side > 0.0f ? -1.3f : 0.0f;
                bp.openMax = side > 0.0f ? 0.0f : 1.3f;
                for (std::size_t cj = 0; cj < before; ++cj)
                {
                    Cluster& in = clusters[cj];
                    if (in.used || in.triangles.size() >= c.triangles.size()) continue;
                    const Math::Vec3 m = (in.low + in.high) * 0.5f;
                    if (m.x >= c.low.x - 0.08f && m.x <= c.high.x + 0.08f && m.y >= c.low.y &&
                            m.y <= c.high.y && m.z >= c.low.z && m.z <= c.high.z)
                        takeCluster(in, loose);
                }
                for (std::size_t pj = 0; pj < mPieces.size(); ++pj)
                {
                    if (mPieces[pj].kind == PieceKind::Body) continue;
                    for (std::size_t pb = 0; pb < mPieces[pj].parts.size(); ++pb)
                    {
                        const Part& other = mPieces[pj].parts[pb];
                        Slice slice;
                        slice.piece = pj;
                        slice.part = pb;
                        for (std::size_t t = 0; t < other.base.indices.size() / 3; ++t)
                        {
                            const Math::Vec3 m = centroidOf(other.base, t);
                            const Math::Vec3 n =
                                    (other.base.normals[other.base.indices[t * 3]] +
                                            other.base.normals[other.base.indices[t * 3 + 1]] +
                                            other.base.normals[other.base.indices[t * 3 + 2]]) *
                                    (1.0f / 3.0f);
                            const bool lateral = fabsf(n.x) > 0.75f;
                            const bool inside = side * m.x > fabsf(centre.x) - 0.3f &&
                                                m.z >= c.low.z && m.z <= c.high.z;
                            const bool window =
                                    mPieces[pj].kind == PieceKind::Glass && m.y <= mHigh.y;
                            const bool lining = mPieces[pj].kind == PieceKind::Interior &&
                                                m.y >= c.low.y && m.y <= c.high.y;
                            if (lateral && inside && (window || lining))
                                slice.triangles.push_back(static_cast<std::uint32_t>(t));
                        }
                        if (!slice.triangles.empty()) loose.slices.push_back(slice);
                    }
                }
            }
            else
            {
                bp.kind = BreakPart::Kind::Hood;
                bp.side = 0.0f;
                bp.mass = 16.0f;
                bp.breakForce = 20000.0f;
                bp.latchForce = 3000.0f;
                bp.hingePoint = Math::Vec3(0.0f, centre.y, c.low.z);
                bp.hingeAxis = Math::Vec3(1.0f, 0.0f, 0.0f);
                bp.openMin = -1.3f;
                bp.openMax = 0.0f;
            }
            mLoose.push_back(loose);
            mBreakParts.push_back(bp);
        }

        for (int end = 0; end < 2; ++end)
        {
            Loose loose;
            Math::Vec3 low(1.0e9f, 1.0e9f, 1.0e9f);
            Math::Vec3 high(-1.0e9f, -1.0e9f, -1.0e9f);
            for (std::size_t pa = 0; pa < piece.parts.size(); ++pa)
            {
                const Part& part = piece.parts[pa];
                Slice slice;
                slice.piece = pi;
                slice.part = pa;
                for (std::size_t t = 0; t < part.base.indices.size() / 3; ++t)
                {
                    if (taken[pa][t]) continue;
                    const Math::Vec3 c = centroidOf(part.base, t);
                    const bool atEnd = end == 0 ? c.z > mHigh.z - 0.34f : c.z < mLow.z + 0.34f;
                    if (!atEnd || c.y > mLow.y + 0.5f * height) continue;
                    slice.triangles.push_back(static_cast<std::uint32_t>(t));
                    for (std::size_t k = 0; k < 3; ++k)
                    {
                        const Math::Vec3& v = part.base.positions[part.base.indices[t * 3 + k]];
                        low = Math::Vec3(fminf(low.x, v.x), fminf(low.y, v.y), fminf(low.z, v.z));
                        high = Math::Vec3(fmaxf(high.x, v.x), fmaxf(high.y, v.y),
                                fmaxf(high.z, v.z));
                    }
                }
                if (!slice.triangles.empty()) loose.slices.push_back(slice);
            }
            if (loose.slices.empty()) continue;
            for (const Slice& slice: loose.slices)
                for (std::uint32_t t: slice.triangles) taken[slice.part][t] = 1;
            mLoose.push_back(loose);
            BreakPart bp;
            bp.kind = BreakPart::Kind::Bumper;
            bp.centre = (low + high) * 0.5f;
            bp.halfExtents = (high - low) * 0.5f;
            bp.mass = 14.0f;
            bp.breakForce = 9000.0f;
            bp.side = 0.0f;
            mBreakParts.push_back(bp);
        }
    }
    for (std::size_t li = 0; li < mLoose.size(); ++li)
        for (const Slice& slice: mLoose[li].slices)
            for (std::uint32_t t: slice.triangles)
                mPieces[slice.piece].parts[slice.part].looseOf[t] = static_cast<int>(li);
}

void CarDamage::finish()
{
    bool first = true;
    for (const Piece& piece: mPieces)
        for (const Part& part: piece.parts)
            for (const Math::Vec3& p: part.base.positions)
            {
                if (first)
                {
                    mLow = p;
                    mHigh = p;
                    first = false;
                }
                mLow = Math::Vec3(fminf(mLow.x, p.x), fminf(mLow.y, p.y), fminf(mLow.z, p.z));
                mHigh = Math::Vec3(fmaxf(mHigh.x, p.x), fmaxf(mHigh.y, p.y), fmaxf(mHigh.z, p.z));
            }
    for (Piece& piece: mPieces)
        for (Part& part: piece.parts)
            for (std::size_t i = 0; i < part.base.positions.size(); ++i)
                part.displacement[i] = crumple(part.base.positions[i]);
    findLoose();
}

void CarDamage::hit(const Math::Vec3& point, float impulse)
{
    const float amount = impulse * kWeightPerImpulse;
    if (amount <= 0.0f) return;
    for (Piece& piece: mPieces)
        for (Part& part: piece.parts)
            for (std::size_t i = 0; i < part.base.positions.size(); ++i)
            {
                const Math::Vec3 away = part.base.positions[i] - point;
                const float distance2 = Math::Vec3::Dot(away, away);
                if (distance2 >= kReach * kReach) continue;
                const float fall = 1.0f - sqrtf(distance2) / kReach;
                const float weight = clamp01(part.weight[i] + amount * fall * fall);
                if (weight > part.weight[i])
                {
                    part.weight[i] = weight;
                    if (weight > mPeak) mPeak = weight;
                    mDirty = true;
                }
            }
}

void CarDamage::reset()
{
    for (Piece& piece: mPieces)
        for (Part& part: piece.parts)
        {
            for (std::size_t i = 0; i < part.weight.size(); ++i)
            {
                part.weight[i] = 0.0f;
                part.drawn[i] = 0.0f;
            }
            for (std::size_t t = 0; t < part.gone.size(); ++t) part.gone[t] = 0;
            for (std::size_t i = 0; i < part.dentVec.size(); ++i)
            {
                part.dentVec[i] = Math::Vec3(0.0f, 0.0f, 0.0f);
                part.dentMark[i] = 0.0f;
            }
            part.reindex = true;
        }
    for (Loose& loose: mLoose) loose.gone = false;
    mBroken = BrokenGlass();
    mPeak = 0.0f;
    mNose = 0.0f;
    mTail = 0.0f;
    mDirty = true;
}

void CarDamage::breakGlass(Piece& piece)
{
    for (Part& part: piece.parts)
        for (std::size_t t = 0; t < part.gone.size(); ++t)
        {
            if (part.gone[t]) continue;
            const std::uint32_t a = part.base.indices[t * 3];
            const std::uint32_t b = part.base.indices[t * 3 + 1];
            const std::uint32_t c = part.base.indices[t * 3 + 2];
            if ((part.drawn[a] + part.drawn[b] + part.drawn[c]) / 3.0f < kGlassBreaks) continue;
            part.gone[t] = 1;
            part.reindex = true;
            if (mBroken.points.size() < 4000)
            {
                mBroken.points.push_back(centroidOf(part.shown, t));
                mBroken.normals.push_back(
                        (part.base.normals[a] + part.base.normals[b] + part.base.normals[c])
                                .NormalizedSafe());
            }
        }
}

bool CarDamage::update(float step)
{
    if (!mDirty) return false;
    mDirty = false;
    const float give = kGivePerSecond * step;
    MeshStore& meshes = *Main::meshes();
    float nose = 0.0f;
    float tail = 0.0f;
    for (Piece& piece: mPieces)
    {
        for (Part& part: piece.parts)
            for (std::size_t i = 0; i < part.base.positions.size(); ++i)
            {
                float& reached = part.drawn[i];
                if (reached < part.weight[i])
                {
                    reached = fminf(reached + give, part.weight[i]);
                    mDirty = true;
                }
                const Math::Vec3 moved = part.displacement[i] * reached;
                part.shown.positions[i] = part.base.positions[i] + moved + part.dentVec[i];
                part.mix[i] = fmaxf(reached, part.dentMark[i]);
                if (-moved.z > nose) nose = -moved.z;
                if (moved.z > tail) tail = moved.z;
            }
        if (piece.kind == PieceKind::Glass) breakGlass(piece);
        for (Part& part: piece.parts)
        {
            if (part.reindex)
            {
                part.shown.indices.clear();
                for (std::size_t t = 0; t < part.gone.size(); ++t)
                {
                    if (part.gone[t]) continue;
                    for (std::size_t k = 0; k < 3; ++k)
                        part.shown.indices.push_back(part.base.indices[t * 3 + k]);
                }
                part.reindex = false;
            }
            rebuildNormals(part.base, part.shown, part.mix);
            meshes.updateSurface(piece.mesh, part.surface, part.shown);
        }
    }
    mNose = nose;
    mTail = tail;
    return true;
}

bool CarDamage::detachPart(std::size_t index, LoosePiece& out)
{
    if (index >= mLoose.size() || mLoose[index].gone) return false;
    Loose& loose = mLoose[index];
    const Math::Vec3 centre = mBreakParts[index].centre;
    out = LoosePiece();
    for (const Slice& slice: loose.slices)
    {
        Part& part = mPieces[slice.piece].parts[slice.part];
        SurfaceData s;
        ct::Vector<std::uint32_t> remap;
        remap.resize(part.shown.positions.size(), 0xFFFFFFFFu);
        for (std::uint32_t t: slice.triangles)
        {
            if (part.gone[t]) continue;
            for (std::size_t k = 0; k < 3; ++k)
            {
                const std::uint32_t v = part.base.indices[static_cast<std::size_t>(t) * 3 + k];
                if (remap[v] == 0xFFFFFFFFu)
                {
                    remap[v] = static_cast<std::uint32_t>(s.positions.size());
                    s.positions.push_back(part.shown.positions[v] - centre);
                    s.normals.push_back(part.shown.normals[v]);
                    if (v < part.shown.uvs.size()) s.uvs.push_back(part.shown.uvs[v]);
                    if (v < part.shown.tangents.size())
                        s.tangents.push_back(part.shown.tangents[v]);
                }
                s.indices.push_back(remap[v]);
            }
            part.gone[t] = 1;
        }
        part.reindex = true;
        if (s.indices.empty()) continue;
        out.surfaces.push_back(s);
        out.materials.push_back(part.material);
    }
    loose.gone = true;
    mDirty = true;
    return true;
}

bool CarDamage::nearestBodyVertex(const Math::Vec3& point, VertexRef& out) const
{
    float best = 1.0e30f;
    bool found = false;
    for (std::size_t pi = 0; pi < mPieces.size(); ++pi)
    {
        if (mPieces[pi].kind != PieceKind::Body) continue;
        for (std::size_t pa = 0; pa < mPieces[pi].parts.size(); ++pa)
        {
            const Part& part = mPieces[pi].parts[pa];
            for (std::size_t i = 0; i < part.base.positions.size(); ++i)
            {
                const Math::Vec3 away = part.base.positions[i] - point;
                const float distance2 = Math::Vec3::Dot(away, away);
                if (distance2 >= best) continue;
                best = distance2;
                out.piece = pi;
                out.part = pa;
                out.vertex = i;
                found = true;
            }
        }
    }
    return found;
}

Math::Vec3 CarDamage::vertexMoved(const VertexRef& ref) const
{
    if (ref.piece >= mPieces.size() || ref.part >= mPieces[ref.piece].parts.size())
        return Math::Vec3(0.0f, 0.0f, 0.0f);
    const Part& part = mPieces[ref.piece].parts[ref.part];
    if (ref.vertex >= part.shown.positions.size()) return Math::Vec3(0.0f, 0.0f, 0.0f);
    return part.shown.positions[ref.vertex] - part.base.positions[ref.vertex];
}

float CarDamage::damageNear(const Math::Vec3& point, float radius) const
{
    float most = 0.0f;
    for (const Piece& piece: mPieces)
    {
        if (piece.kind != PieceKind::Body) continue;
        for (const Part& part: piece.parts)
            for (std::size_t i = 0; i < part.base.positions.size(); ++i)
            {
                if (part.weight[i] <= most) continue;
                const Math::Vec3 away = part.base.positions[i] - point;
                if (Math::Vec3::Dot(away, away) <= radius * radius) most = part.weight[i];
            }
    }
    return most;
}

bool CarDamage::raycast(const Math::Vec3& origin, const Math::Vec3& direction, float maxDistance,
        BulletHit& out) const
{
    float near = 0.0f;
    float far = maxDistance;
    const Math::Vec3 low = mLow - Math::Vec3(0.3f, 0.3f, 0.3f);
    const Math::Vec3 high = mHigh + Math::Vec3(0.3f, 0.3f, 0.3f);
    const float o[3] = { origin.x, origin.y, origin.z };
    const float d[3] = { direction.x, direction.y, direction.z };
    const float l[3] = { low.x, low.y, low.z };
    const float h[3] = { high.x, high.y, high.z };
    for (int k = 0; k < 3; ++k)
    {
        if (fabsf(d[k]) < 1.0e-8f)
        {
            if (o[k] < l[k] || o[k] > h[k]) return false;
            continue;
        }
        float t0 = (l[k] - o[k]) / d[k];
        float t1 = (h[k] - o[k]) / d[k];
        if (t0 > t1)
        {
            const float swap = t0;
            t0 = t1;
            t1 = swap;
        }
        if (t0 > near) near = t0;
        if (t1 < far) far = t1;
        if (near > far) return false;
    }

    float best = maxDistance;
    bool found = false;
    for (std::size_t pi = 0; pi < mPieces.size(); ++pi)
        for (std::size_t pa = 0; pa < mPieces[pi].parts.size(); ++pa)
        {
            const Part& part = mPieces[pi].parts[pa];
            for (std::size_t t = 0; t < part.gone.size(); ++t)
            {
                if (part.gone[t]) continue;
                const Math::Vec3& a = part.shown.positions[part.base.indices[t * 3]];
                const Math::Vec3& b = part.shown.positions[part.base.indices[t * 3 + 1]];
                const Math::Vec3& c = part.shown.positions[part.base.indices[t * 3 + 2]];
                const Math::Vec3 e1 = b - a;
                const Math::Vec3 e2 = c - a;
                const Math::Vec3 p = direction.Cross(e2);
                const float det = Math::Vec3::Dot(e1, p);
                if (fabsf(det) < 1.0e-9f) continue;
                const float inverse = 1.0f / det;
                const Math::Vec3 from = origin - a;
                const float u = Math::Vec3::Dot(from, p) * inverse;
                if (u < 0.0f || u > 1.0f) continue;
                const Math::Vec3 q = from.Cross(e1);
                const float v = Math::Vec3::Dot(direction, q) * inverse;
                if (v < 0.0f || u + v > 1.0f) continue;
                const float distance = Math::Vec3::Dot(e2, q) * inverse;
                if (distance < 0.0f || distance >= best) continue;
                best = distance;
                found = true;
                out.distance = distance;
                out.point = origin + direction * distance;
                Math::Vec3 normal = e1.Cross(e2).NormalizedSafe();
                if (Math::Vec3::Dot(normal, direction) > 0.0f) normal = normal * -1.0f;
                out.normal = normal;
                out.piece = pi;
                out.part = pa;
                out.triangle = static_cast<std::uint32_t>(t);
                out.loose = part.looseOf[t];
                out.glass = mPieces[pi].kind == PieceKind::Glass;
            }
        }
    return found;
}

void CarDamage::bullet(const BulletHit& hit)
{
    Piece& piece = mPieces[hit.piece];
    Part& part = piece.parts[hit.part];
    if (hit.glass)
    {
        for (Part& glass: piece.parts)
            for (std::size_t t = 0; t < glass.gone.size(); ++t)
            {
                if (glass.gone[t]) continue;
                const Math::Vec3 away = centroidOf(glass.shown, t) - hit.point;
                if (Math::Vec3::Dot(away, away) > 0.14f * 0.14f) continue;
                glass.gone[t] = 1;
                glass.reindex = true;
                if (mBroken.points.size() < 4000)
                {
                    mBroken.points.push_back(centroidOf(glass.shown, t));
                    mBroken.normals.push_back(hit.normal);
                }
            }
    }
    else
    {
        const float reach = 0.16f;
        for (std::size_t i = 0; i < part.shown.positions.size(); ++i)
        {
            const Math::Vec3 away = part.shown.positions[i] - hit.point;
            const float distance2 = Math::Vec3::Dot(away, away);
            if (distance2 >= reach * reach) continue;
            const float fall = 1.0f - sqrtf(distance2) / reach;
            Math::Vec3 dent = part.dentVec[i] - hit.normal * (0.035f * fall * fall);
            const float size = dent.Length();
            if (size > 0.1f) dent = dent * (0.1f / size);
            part.dentVec[i] = dent;
            part.dentMark[i] = 1.0f;
        }
    }
    mDirty = true;
}

void CarDamage::takeBrokenGlass(BrokenGlass& out)
{
    out = mBroken;
    mBroken = BrokenGlass();
}

std::size_t CarDamage::touched() const
{
    std::size_t count = 0;
    for (const Piece& piece: mPieces)
        for (const Part& part: piece.parts)
            for (float w: part.weight)
                if (w > 0.0f) ++count;
    return count;
}

} // namespace prisma3d
