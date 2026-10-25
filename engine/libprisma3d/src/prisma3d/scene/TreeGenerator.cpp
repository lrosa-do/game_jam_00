#include "prisma3d/scene/TreeGenerator.h"

#include <math.h>
#include <string.h>

namespace prisma3d
{

namespace
{

constexpr float kPi = 3.14159265358979323846f;
constexpr float kHalfPi = 1.57079632679489661923f;
constexpr float kLimit = 1.0e6f;

Math::Vec3 rotateAxisAngle(const Math::Vec3& vec, const Math::Vec3& axis, float angle)
{
    const float cosine = cosf(angle);
    const float sine = sinf(angle);
    return vec * cosine + axis.Cross(vec) * sine + axis * (axis.Dot(vec) * (1.0f - cosine));
}

Math::Vec3 scaleInDirection(const Math::Vec3& vector, const Math::Vec3& direction, float scale)
{
    const float magnitude = vector.Dot(direction);
    return vector + direction * (magnitude * scale - magnitude);
}

Math::Vec3 normalizeSafe(const Math::Vec3& v)
{
    const float length = v.Length();
    return length > 1e-20f ? v / length : Math::Vec3(0.0f, 0.0f, 0.0f);
}

int wrap(int value, int count)
{
    const int remainder = value % count;
    return remainder < 0 ? remainder + count : remainder;
}

float treeRandom(double a)
{
    return static_cast<float>(fabs(cos(a + a * a)));
}

bool finite(const Math::Vec3& v)
{
    return isfinite(v.x) && isfinite(v.y) && isfinite(v.z);
}

float cleanFloat(float value, float fallback, float minimum)
{
    if (!isfinite(value)) value = fallback;
    if (value > kLimit) value = kLimit;
    if (value < -kLimit) value = -kLimit;
    if (value < minimum) value = minimum;
    return value;
}

struct Branch
{
    Math::Vec3 head = Math::Vec3(0.0f, 0.0f, 0.0f);
    int parent = -1;
    int child0 = -1;
    int child1 = -1;
    float length = 1.0f;
    float radius = 0.0f;
    bool trunk = false;
    bool ringed = false;
    int tip = -1;
    int ringCount[4] = { 0, 0, 0, 0 };
};

enum RingKind
{
    kRootRing = 0,
    kRing0 = 1,
    kRing1 = 2,
    kRing2 = 3
};

class TreeBuilder
{
public:
    explicit TreeBuilder(const TreeParams& params)
            : mParams(params),
              mSegments(static_cast<int>(params.segments))
    {
    }

    void build(TreeGeometry& out)
    {
        const int root = addBranch(Math::Vec3(0.0f, mParams.trunkLength, 0.0f), -1);
        mBranches[static_cast<std::size_t>(root)].length = mParams.initialBranchLength;

        splitBranch(root, static_cast<int>(mParams.levels), static_cast<int>(mParams.trunkSteps), 1,
                1);

        mRings.resize(mBranches.size() * 4u * static_cast<std::size_t>(mSegments), 0u);

        createForks(root, mParams.maxRadius);
        createTwigs(root);
        buildFaces(root);

        for (std::size_t i = 0; i < mPositions.size(); ++i)
            if (!finite(mPositions[i])) mPositions[i] = Math::Vec3(0.0f, 0.0f, 0.0f);
        for (std::size_t i = 0; i < mUvs.size(); ++i)
            if (!isfinite(mUvs[i].x) || !isfinite(mUvs[i].y)) mUvs[i] = Math::Vec2(0.0f, 0.0f);
        for (std::size_t i = 0; i < mLeafPositions.size(); ++i)
            if (!finite(mLeafPositions[i])) mLeafPositions[i] = Math::Vec3(0.0f, 0.0f, 0.0f);

        averageNormals();

        out.bark.topology = prisma::Topology::Triangles;
        out.bark.positions = mPositions;
        out.bark.normals = mNormals;
        out.bark.uvs = mUvs;
        out.bark.indices = mIndices;
        finishTangents(out.bark);

        out.leaves.topology = prisma::Topology::Triangles;
        out.leaves.positions = mLeafPositions;
        out.leaves.normals = mLeafNormals;
        out.leaves.uvs = mLeafUvs;
        out.leaves.indices = mLeafIndices;
        finishTangents(out.leaves);

        out.trunkIndices = mTrunkIndices;
    }

private:
    const TreeParams& mParams;
    int mSegments;

    ct::Vector<Branch> mBranches;
    ct::Vector<std::uint32_t> mRings;

    ct::Vector<Math::Vec3> mPositions;
    ct::Vector<Math::Vec3> mNormals;
    ct::Vector<Math::Vec2> mUvs;
    ct::Vector<std::uint32_t> mIndices;
    ct::Vector<std::uint32_t> mTrunkIndices;

    ct::Vector<Math::Vec3> mLeafPositions;
    ct::Vector<Math::Vec3> mLeafNormals;
    ct::Vector<Math::Vec2> mLeafUvs;
    ct::Vector<std::uint32_t> mLeafIndices;

    static void finishTangents(SurfaceData& surface)
    {
        bool ok = MeshStore::generateTangents(surface);
        if (ok)
        {
            for (std::size_t i = 0; i < surface.tangents.size(); ++i)
            {
                const Math::Vec4& t = surface.tangents[i];
                if (!isfinite(t.x) || !isfinite(t.y) || !isfinite(t.z) || !isfinite(t.w))
                {
                    ok = false;
                    break;
                }
            }
        }
        if (!ok)
        {
            surface.tangents.clear();
            surface.tangents.resize(surface.positions.size(), Math::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
        }
    }

    Branch& at(int index)
    {
        return mBranches[static_cast<std::size_t>(index)];
    }

    const Branch& at(int index) const
    {
        return mBranches[static_cast<std::size_t>(index)];
    }

    int addBranch(const Math::Vec3& head, int parent)
    {
        Branch branch;
        branch.head = head;
        branch.parent = parent;
        mBranches.push_back(branch);
        return static_cast<int>(mBranches.size()) - 1;
    }

    std::size_t ringSlot(int branch, RingKind kind, int i) const
    {
        return (static_cast<std::size_t>(branch) * 4u + static_cast<std::size_t>(kind)) *
                       static_cast<std::size_t>(mSegments) +
               static_cast<std::size_t>(i);
    }

    std::uint32_t ringAt(int branch, RingKind kind, int i) const
    {
        return mRings[ringSlot(branch, kind, i)];
    }

    void ringPush(int branch, RingKind kind, std::uint32_t vertex)
    {
        int& count = at(branch).ringCount[kind];
        if (count >= mSegments) return;
        mRings[ringSlot(branch, kind, count)] = vertex;
        ++count;
    }

    std::uint32_t positionCount() const
    {
        return static_cast<std::uint32_t>(mPositions.size());
    }

    void addFace(std::uint32_t a, std::uint32_t b, std::uint32_t c, bool trunk)
    {
        mIndices.push_back(a);
        mIndices.push_back(b);
        mIndices.push_back(c);
        if (!trunk) return;
        mTrunkIndices.push_back(a);
        mTrunkIndices.push_back(b);
        mTrunkIndices.push_back(c);
    }

    Math::Vec3 mirrorBranch(const Math::Vec3& vec, const Math::Vec3& norm) const
    {
        const Math::Vec3 projected = norm.Cross(vec.Cross(norm));
        return vec - projected * (mParams.branchFactor * projected.Dot(vec));
    }

    void splitBranch(int b, int level, int steps, int l1, int l2)
    {
        const int levels = static_cast<int>(mParams.levels);
        const int reverseLevel = levels - level;

        Math::Vec3 origin(0.0f, 0.0f, 0.0f);
        if (at(b).parent >= 0) origin = at(at(b).parent).head;
        else
            at(b).trunk = true;

        const Math::Vec3 head = at(b).head;
        const float branchLength = at(b).length;
        const Math::Vec3 direction = normalizeSafe(head - origin);

        const Math::Vec3 normal =
                direction.Cross(Math::Vec3(direction.z, direction.x, direction.y));
        const Math::Vec3 tangent = direction.Cross(normal);

        const float r =
                treeRandom(static_cast<double>(reverseLevel * 10 + l1 * 5 + l2 + mParams.seed));

        Math::Vec3 adjust = normal * r + tangent * (1.0f - r);
        if (r > 0.5f) adjust = -adjust;

        const float clump = (mParams.clumpMax - mParams.clumpMin) * r + mParams.clumpMin;
        Math::Vec3 first = normalizeSafe(adjust * (1.0f - clump) + direction * clump);
        Math::Vec3 second = mirrorBranch(first, direction);

        if (r > 0.5f)
        {
            const Math::Vec3 swap = first;
            first = second;
            second = swap;
        }

        if (steps > 0)
        {
            const float angle = static_cast<float>(steps) / static_cast<float>(mParams.trunkSteps) *
                                2.0f * kPi * mParams.twistRate;
            second = normalizeSafe(Math::Vec3(sinf(angle), r, cosf(angle)));
        }

        const float grow = static_cast<float>(level * level) / static_cast<float>(levels * levels) *
                           mParams.growAmount;
        const Math::Vec3 bend(static_cast<float>(reverseLevel) * mParams.sweepAmount,
                static_cast<float>(reverseLevel) * mParams.dropAmount + grow, 0.0f);
        first = normalizeSafe(first + bend);
        second = normalizeSafe(second + bend);

        const int child0 = addBranch(head + first * branchLength, b);
        const int child1 = addBranch(head + second * branchLength, b);
        at(b).child0 = child0;
        at(b).child1 = child1;

        float childLength =
                powf(branchLength, mParams.lengthFalloffPower) * mParams.lengthFalloffFactor;
        if (!isfinite(childLength)) childLength = 0.0f;
        at(child0).length = childLength;
        at(child1).length = childLength;

        if (level <= 0) return;

        if (steps > 0)
        {
            const float kink = (r - 0.5f) * 2.0f * mParams.trunkKink;
            at(child0).head = at(b).head + Math::Vec3(kink, mParams.climbRate, kink);
            at(child0).trunk = true;
            at(child0).length = at(b).length * mParams.taperRate;
            splitBranch(child0, level, steps - 1, l1 + 1, l2);
        }
        else
        {
            splitBranch(child0, level - 1, 0, l1 + 1, l2);
        }
        splitBranch(child1, level - 1, 0, l1, l2 + 1);
    }

    void createForks(int b, float radius)
    {
        at(b).radius = radius;
        if (radius > at(b).length) radius = at(b).length;

        const int segments = mSegments;
        const float segmentAngle = 2.0f * kPi / static_cast<float>(segments);

        if (at(b).parent < 0)
        {
            const Math::Vec3 axis(0.0f, 1.0f, 0.0f);
            for (int i = 0; i < segments; ++i)
            {
                const Math::Vec3 vec = rotateAxisAngle(Math::Vec3(-1.0f, 0.0f, 0.0f), axis,
                        -segmentAngle * static_cast<float>(i));
                ringPush(b, kRootRing, positionCount());
                mPositions.push_back(vec * (radius / mParams.radiusFalloffRate));
            }
        }

        if (at(b).child0 < 0)
        {
            at(b).tip = static_cast<int>(positionCount());
            mPositions.push_back(at(b).head);
            return;
        }

        const int child0 = at(b).child0;
        const int child1 = at(b).child1;
        const Math::Vec3 head = at(b).head;

        const Math::Vec3 axis = at(b).parent >= 0 ? normalizeSafe(head - at(at(b).parent).head)
                                                  : normalizeSafe(head);

        const Math::Vec3 axis1 = normalizeSafe(head - at(child0).head);
        const Math::Vec3 axis2 = normalizeSafe(head - at(child1).head);
        const Math::Vec3 tangent = normalizeSafe(axis1.Cross(axis2));

        const Math::Vec3 axis3 = normalizeSafe(tangent.Cross(normalizeSafe(-axis1 - axis2)));
        const Math::Vec3 centre =
                head + Math::Vec3(axis2.x, 0.0f, axis2.z) * (-mParams.maxRadius * 0.5f);

        at(b).ringed = true;

        float scale = mParams.radiusFalloffRate;
        if (at(child0).trunk || at(b).trunk) scale = 1.0f / mParams.taperRate;

        const std::uint32_t linch0 = positionCount();
        ringPush(b, kRing0, linch0);
        ringPush(b, kRing2, linch0);
        mPositions.push_back(centre + tangent * (radius * scale));

        std::uint32_t start = positionCount() - 1;
        const Math::Vec3 d1 = rotateAxisAngle(tangent, axis2, kHalfPi);
        const Math::Vec3 d2 = normalizeSafe(tangent.Cross(axis));
        const float alignment = d1.Dot(d2);
        const float stretch = fabsf(alignment) > 1e-6f ? 1.0f / alignment : 1.0f;
        for (int i = 1; i < segments / 2; ++i)
        {
            Math::Vec3 vec = rotateAxisAngle(tangent, axis2, segmentAngle * static_cast<float>(i));
            ringPush(b, kRing0, start + static_cast<std::uint32_t>(i));
            ringPush(b, kRing2, start + static_cast<std::uint32_t>(i));
            vec = scaleInDirection(vec, d2, stretch);
            mPositions.push_back(centre + vec * (radius * scale));
        }

        const std::uint32_t linch1 = positionCount();
        ringPush(b, kRing0, linch1);
        ringPush(b, kRing1, linch1);
        mPositions.push_back(centre + tangent * (-radius * scale));

        for (int i = segments / 2 + 1; i < segments; ++i)
        {
            const Math::Vec3 vec =
                    rotateAxisAngle(tangent, axis1, segmentAngle * static_cast<float>(i));
            ringPush(b, kRing0, positionCount());
            ringPush(b, kRing1, positionCount());
            mPositions.push_back(centre + vec * (radius * scale));
        }

        ringPush(b, kRing1, linch0);
        ringPush(b, kRing2, linch1);

        start = positionCount() - 1;
        for (int i = 1; i < segments / 2; ++i)
        {
            const Math::Vec3 vec =
                    rotateAxisAngle(tangent, axis3, segmentAngle * static_cast<float>(i));
            ringPush(b, kRing1, start + static_cast<std::uint32_t>(i));
            ringPush(b, kRing2, start + static_cast<std::uint32_t>(segments / 2 - i));
            mPositions.push_back(centre + vec * (radius * scale));
        }

        const float childRadius = radius * mParams.radiusFalloffRate;
        createForks(child0, at(child0).trunk ? radius * mParams.taperRate : childRadius);
        createForks(child1, childRadius);
    }

    void createTwigs(int b)
    {
        if (at(b).child0 >= 0)
        {
            createTwigs(at(b).child0);
            createTwigs(at(b).child1);
            return;
        }

        const int parent = at(b).parent;
        if (parent < 0 || at(parent).child0 < 0 || at(parent).child1 < 0) return;

        const Math::Vec3 parentHead = at(parent).head;
        const Math::Vec3 tangent =
                normalizeSafe((at(at(parent).child0).head - parentHead)
                                      .Cross(at(at(parent).child1).head - parentHead));
        const Math::Vec3 binormal = normalizeSafe(at(b).head - parentHead);

        const float scale = mParams.twigScale;
        const float length = at(b).length;
        const Math::Vec3 head = at(b).head;

        const Math::Vec3 corners[4] = {
            head + tangent * scale + binormal * (scale * 2.0f - length),
            head - tangent * scale + binormal * (scale * 2.0f - length),
            head - tangent * scale - binormal * length,
            head + tangent * scale - binormal * length,
        };

        const std::uint32_t base = static_cast<std::uint32_t>(mLeafPositions.size());
        for (int pass = 0; pass < 2; ++pass)
            for (int i = 0; i < 4; ++i) mLeafPositions.push_back(corners[i]);

        const std::uint32_t order[12] = { 0, 1, 2, 3, 0, 2, 6, 5, 4, 6, 4, 7 };
        for (int i = 0; i < 12; ++i) mLeafIndices.push_back(base + order[i]);

        Math::Vec3 front = normalizeSafe((corners[0] - corners[2]).Cross(corners[1] - corners[2]));
        if (front.LengthSquared() < 0.5f) front = Math::Vec3(0.0f, 1.0f, 0.0f);
        for (int i = 0; i < 4; ++i) mLeafNormals.push_back(front);
        for (int i = 0; i < 4; ++i) mLeafNormals.push_back(-front);

        for (int pass = 0; pass < 2; ++pass)
        {
            mLeafUvs.push_back(Math::Vec2(0.0f, 0.0f));
            mLeafUvs.push_back(Math::Vec2(1.0f, 0.0f));
            mLeafUvs.push_back(Math::Vec2(1.0f, 1.0f));
            mLeafUvs.push_back(Math::Vec2(0.0f, 1.0f));
        }
    }

    void buildFaces(int b)
    {
        const int segments = mSegments;
        const int child0 = at(b).child0;
        const int child1 = at(b).child1;

        if (at(b).parent < 0)
        {
            mUvs.assign(mPositions.size(), Math::Vec2(0.0f, 0.0f));

            const Math::Vec3 head = at(b).head;
            const Math::Vec3 tangent =
                    normalizeSafe((at(child0).head - head).Cross(at(child1).head - head));
            const Math::Vec3 reference(-1.0f, 0.0f, 0.0f);
            float dot = tangent.Dot(reference);
            if (dot > 1.0f) dot = 1.0f;
            if (dot < -1.0f) dot = -1.0f;
            float angle = acosf(dot);
            if (reference.Cross(tangent).Dot(normalizeSafe(head)) > 0.0f)
                angle = 2.0f * kPi - angle;

            const int offset = static_cast<int>(
                    lround(angle / (2.0 * 3.14159265358979323846) * static_cast<double>(segments)));

            for (int i = 0; i < segments; ++i)
            {
                const std::uint32_t a = ringAt(b, kRing0, i);
                const std::uint32_t rb = ringAt(b, kRootRing, wrap(i + offset + 1, segments));
                const std::uint32_t c = ringAt(b, kRootRing, wrap(i + offset, segments));
                const std::uint32_t d = ringAt(b, kRing0, wrap(i + 1, segments));

                addFace(a, d, c, true);
                addFace(d, rb, c, true);

                const float u =
                        fabsf(static_cast<float>(i) / static_cast<float>(segments) - 0.5f) * 2.0f;
                mUvs[c] = Math::Vec2(u, 0.0f);

                const float v = (mPositions[a] - mPositions[c]).Length() * mParams.vMultiplier;
                mUvs[a] = Math::Vec2(u, v);
                mUvs[ringAt(b, kRing2, i)] = Math::Vec2(u, v);
            }
        }

        if (child0 >= 0 && at(child0).ringed)
        {
            const int offset0 = matchSegment(b, kRing1, child0);
            const int offset1 = matchSegment(b, kRing2, child1);

            const float uvScale = mParams.maxRadius / at(b).radius;

            for (int i = 0; i < segments; ++i)
                stitchChild(child0, b, kRing1, offset0, i, uvScale, false);
            for (int i = 0; i < segments; ++i)
                stitchChild(child1, b, kRing2, offset1, i, uvScale, true);

            buildFaces(child0);
            buildFaces(child1);
            return;
        }

        if (child0 < 0 || child1 < 0) return;

        closeTip(child0, b, kRing1, 1.5f);
        closeTip(child1, b, kRing2, 0.5f);
    }

    int matchSegment(int branch, RingKind kind, int child) const
    {
        const int segments = mSegments;
        const Math::Vec3 branchHead = at(branch).head;

        Math::Vec3 side = normalizeSafe(mPositions[ringAt(branch, kind, 0)] - branchHead);
        side = scaleInDirection(side, normalizeSafe(at(child).head - branchHead), 0.0f);

        const Math::Vec3 childHead = at(child).head;
        int best = segments;
        float bestMatch = normalizeSafe(mPositions[ringAt(child, kRing0, 0)] - childHead).Dot(side);
        for (int i = 1; i < segments; ++i)
        {
            const Math::Vec3 d = normalizeSafe(mPositions[ringAt(child, kRing0, i)] - childHead);
            const float match = d.Dot(side);
            if (match > bestMatch)
            {
                bestMatch = match;
                best = segments - i;
            }
        }
        return best;
    }

    void stitchChild(int child, int parent, RingKind kind, int offset, int i, float uvScale,
            bool flipWinding)
    {
        const int segments = mSegments;

        const std::uint32_t a = ringAt(child, kRing0, i);
        const std::uint32_t b = ringAt(parent, kind, wrap(i + offset + 1, segments));
        const std::uint32_t c = ringAt(parent, kind, wrap(i + offset, segments));
        const std::uint32_t d = ringAt(child, kRing0, wrap(i + 1, segments));

        const bool trunk = at(child).trunk;
        if (flipWinding)
        {
            addFace(a, b, c, trunk);
            addFace(a, d, b, trunk);
        }
        else
        {
            addFace(a, d, c, trunk);
            addFace(d, b, c, trunk);
        }

        const float length = (mPositions[a] - mPositions[c]).Length() * uvScale;
        const Math::Vec2 previous = mUvs[ringAt(parent, kind, wrap(i + offset - 1, segments))];
        const Math::Vec2 uv(previous.x, previous.y + length * mParams.vMultiplier);
        mUvs[a] = uv;
        mUvs[ringAt(child, kRing2, i)] = uv;
    }

    void closeTip(int child, int parent, RingKind kind, float uvCentre)
    {
        if (at(child).tip < 0) return;

        const int segments = mSegments;
        const std::uint32_t tip = static_cast<std::uint32_t>(at(child).tip);

        for (int i = 0; i < segments; ++i)
        {
            const std::uint32_t a = ringAt(parent, kind, wrap(i + 1, segments));
            const std::uint32_t b = ringAt(parent, kind, i);
            addFace(tip, a, b, false);

            const float length = (mPositions[tip] - mPositions[b]).Length();
            mUvs[tip] = Math::Vec2(
                    fabsf(static_cast<float>(i) / static_cast<float>(segments) - uvCentre) * 2.0f,
                    length * mParams.vMultiplier);
        }
    }

    void averageNormals()
    {
        mNormals.assign(mPositions.size(), Math::Vec3(0.0f, 0.0f, 0.0f));

        for (std::size_t i = 0; i + 2 < mIndices.size(); i += 3)
        {
            const std::uint32_t a = mIndices[i];
            const std::uint32_t b = mIndices[i + 1];
            const std::uint32_t c = mIndices[i + 2];
            const Math::Vec3 normal = normalizeSafe(
                    (mPositions[b] - mPositions[c]).Cross(mPositions[b] - mPositions[a]));
            mNormals[a] += normal;
            mNormals[b] += normal;
            mNormals[c] += normal;
        }

        for (std::size_t i = 0; i < mNormals.size(); ++i)
        {
            const float length = mNormals[i].Length();
            mNormals[i] = length > 1e-8f && isfinite(length) ? mNormals[i] / length
                                                             : Math::Vec3(0.0f, 1.0f, 0.0f);
        }
    }
};

TreeParams addonDefaults()
{
    TreeParams params;
    params.segments = 6;
    params.levels = 5;
    params.lengthFalloffFactor = 0.85f;
    params.branchFactor = 2.45f;
    params.clumpMax = 0.45f;
    params.clumpMin = 0.40f;
    params.dropAmount = -0.10f;
    params.growAmount = 0.235f;
    params.sweepAmount = 0.01f;
    params.radiusFalloffRate = 0.73f;
    params.climbRate = 0.371f;
    params.trunkKink = 0.093f;
    params.twistRate = 3.02f;
    params.vMultiplier = 0.36f;
    params.seed = 10;
    return params;
}

const ct::Vector<TreePreset>& presetTable()
{
    static const ct::Vector<TreePreset> table = []
    {
        ct::Vector<TreePreset> list;

        TreeParams oak = addonDefaults();
        oak.seed = 696;
        oak.trunkSteps = 9;
        oak.initialBranchLength = 0.79f;
        oak.maxRadius = 0.22f;
        oak.trunkLength = 3.21f;
        oak.twigScale = 0.55f;
        list.push_back({ "Oak", oak, 0 });

        TreeParams willow = addonDefaults();
        willow.seed = 264;
        willow.levels = 4;
        willow.trunkSteps = 9;
        willow.initialBranchLength = 0.59f;
        willow.lengthFalloffFactor = 0.57f;
        willow.dropAmount = 0.4f;
        willow.maxRadius = 0.05f;
        willow.radiusFalloffRate = 0.7f;
        willow.twistRate = 2.62f;
        willow.trunkLength = 1.79f;
        willow.twigScale = 0.4f;
        list.push_back({ "Willow", willow, 1 });

        TreeParams shrub = addonDefaults();
        shrub.seed = 264;
        shrub.levels = 3;
        shrub.trunkSteps = 0;
        shrub.initialBranchLength = 0.19f;
        shrub.dropAmount = 0.2f;
        shrub.maxRadius = 0.04f;
        shrub.radiusFalloffRate = 0.58f;
        shrub.twistRate = 5.0f;
        shrub.trunkLength = 0.38f;
        shrub.twigScale = 0.65f;
        list.push_back({ "Shrub", shrub, 0 });

        TreeParams ash = addonDefaults();
        ash.levels = 7;
        ash.initialBranchLength = 0.59f;
        ash.lengthFalloffFactor = 0.95f;
        ash.dropAmount = -0.3f;
        ash.growAmount = 0.03f;
        ash.maxRadius = 0.18f;
        ash.radiusFalloffRate = 0.61f;
        ash.climbRate = 0.2f;
        ash.trunkKink = -0.37f;
        ash.twistRate = 1.41f;
        ash.trunkLength = 7.05f;
        ash.twigScale = 0.65f;
        list.push_back({ "Ash", ash, 1 });

        TreeParams poplar = addonDefaults();
        poplar.trunkSteps = 30;
        poplar.dropAmount = 0.0f;
        poplar.maxRadius = 0.2f;
        poplar.climbRate = 0.3f;
        poplar.trunkKink = 0.01f;
        poplar.twistRate = -2.47f;
        poplar.trunkLength = 3.61f;
        poplar.twigScale = 1.0f;
        list.push_back({ "Poplar", poplar, 2 });

        TreeParams sequoia = addonDefaults();
        sequoia.seed = 264;
        sequoia.levels = 4;
        sequoia.trunkSteps = 12;
        sequoia.initialBranchLength = 2.19f;
        sequoia.dropAmount = 0.0f;
        sequoia.growAmount = -0.38f;
        sequoia.maxRadius = 0.6f;
        sequoia.radiusFalloffRate = 0.57f;
        sequoia.climbRate = 2.02f;
        sequoia.trunkKink = 0.0f;
        sequoia.trunkLength = 11.52f;
        sequoia.twigScale = 2.0f;
        list.push_back({ "Sequoia", sequoia, 0 });

        TreeParams beech = addonDefaults();
        beech.seed = 325;
        beech.levels = 7;
        beech.trunkSteps = 4;
        beech.initialBranchLength = 0.89f;
        beech.dropAmount = 0.151f;
        beech.growAmount = 0.033f;
        beech.maxRadius = 0.21f;
        beech.climbRate = 0.37f;
        beech.twistRate = 2.21f;
        beech.trunkLength = 4.83f;
        beech.twigScale = 0.7f;
        list.push_back({ "Beech", beech, 0 });

        return list;
    }();

    return table;
}

} // namespace

static_assert(sizeof(TreeParams) == 22 * 4, "TreeParams must have no padding");

bool TreeParams::operator==(const TreeParams& other) const
{
    return memcmp(this, &other, sizeof(TreeParams)) == 0;
}

bool TreeParams::operator!=(const TreeParams& other) const
{
    return !(*this == other);
}

void TreeGeometry::clear()
{
    bark = SurfaceData();
    leaves = SurfaceData();
    trunkIndices.clear();
}

TreeParams TreeGenerator::sanitize(const TreeParams& params)
{
    const TreeParams defaults;
    TreeParams out = params;

    out.clumpMax = cleanFloat(params.clumpMax, defaults.clumpMax, -kLimit);
    out.clumpMin = cleanFloat(params.clumpMin, defaults.clumpMin, -kLimit);
    out.lengthFalloffFactor =
            cleanFloat(params.lengthFalloffFactor, defaults.lengthFalloffFactor, -kLimit);
    out.lengthFalloffPower =
            cleanFloat(params.lengthFalloffPower, defaults.lengthFalloffPower, -kLimit);
    out.branchFactor = cleanFloat(params.branchFactor, defaults.branchFactor, -kLimit);
    out.radiusFalloffRate = cleanFloat(params.radiusFalloffRate, defaults.radiusFalloffRate, 0.01f);
    out.taperRate = cleanFloat(params.taperRate, defaults.taperRate, 0.01f);
    out.maxRadius = cleanFloat(params.maxRadius, defaults.maxRadius, 0.0f);
    out.climbRate = cleanFloat(params.climbRate, defaults.climbRate, -kLimit);
    out.trunkKink = cleanFloat(params.trunkKink, defaults.trunkKink, -kLimit);
    out.twistRate = cleanFloat(params.twistRate, defaults.twistRate, -kLimit);
    out.trunkLength = cleanFloat(params.trunkLength, defaults.trunkLength, 0.0f);
    out.initialBranchLength =
            cleanFloat(params.initialBranchLength, defaults.initialBranchLength, 0.0f);
    out.dropAmount = cleanFloat(params.dropAmount, defaults.dropAmount, -kLimit);
    out.growAmount = cleanFloat(params.growAmount, defaults.growAmount, -kLimit);
    out.sweepAmount = cleanFloat(params.sweepAmount, defaults.sweepAmount, -kLimit);
    out.vMultiplier = cleanFloat(params.vMultiplier, defaults.vMultiplier, -kLimit);
    out.twigScale = cleanFloat(params.twigScale, defaults.twigScale, 0.0f);

    out.segments = params.segments < 4 ? 4u : (params.segments & ~1u);
    if (out.segments > kMaxSegments) out.segments = kMaxSegments;
    out.levels = params.levels < 1 ? 1u : params.levels;
    if (out.levels > kMaxLevels) out.levels = kMaxLevels;
    if (out.trunkSteps > kMaxTrunkSteps) out.trunkSteps = kMaxTrunkSteps;

    auto budget = [&out]() {
        return static_cast<std::uint64_t>(out.trunkSteps + 1u) * (1ull << out.levels) *
               out.segments;
    };
    while (budget() > kMaxBudget && out.trunkSteps > 0) out.trunkSteps = out.trunkSteps / 2;
    while (budget() > kMaxBudget && out.levels > 1) --out.levels;
    return out;
}

void TreeGenerator::buildShadowGeometry(const TreeGeometry& full, int leafStep, TreeGeometry& out)
{
    out.clear();
    out.bark.topology = full.bark.topology;
    out.bark.positions = full.bark.positions;
    out.bark.normals = full.bark.normals;
    out.bark.indices = full.trunkIndices;
    out.trunkIndices = full.trunkIndices;

    const std::size_t step = leafStep > 1 ? static_cast<std::size_t>(leafStep) : 1;
    const float grow = sqrtf(static_cast<float>(step));
    const std::size_t leafCount = full.leaves.positions.size() / 8;
    out.leaves.topology = full.leaves.topology;
    const std::uint32_t order[12] = { 0, 1, 2, 3, 0, 2, 6, 5, 4, 6, 4, 7 };
    for (std::size_t leaf = 0; leaf < leafCount; leaf += step)
    {
        const std::size_t first = leaf * 8;
        Math::Vec3 centre(0.0f, 0.0f, 0.0f);
        for (std::size_t i = 0; i < 4; ++i) centre += full.leaves.positions[first + i];
        centre = centre * 0.25f;

        const std::uint32_t base = static_cast<std::uint32_t>(out.leaves.positions.size());
        for (std::size_t i = 0; i < 8; ++i)
        {
            const Math::Vec3 offset = full.leaves.positions[first + i] - centre;
            out.leaves.positions.push_back(centre + offset * grow);
            if (first + i < full.leaves.normals.size())
                out.leaves.normals.push_back(full.leaves.normals[first + i]);
        }
        for (int i = 0; i < 12; ++i) out.leaves.indices.push_back(base + order[i]);
    }
}

void TreeGenerator::generate(const TreeParams& params, TreeGeometry& out)
{
    out.clear();
    const TreeParams sane = sanitize(params);
    TreeBuilder builder(sane);
    builder.build(out);
}

std::uint32_t TreeGenerator::presetCount()
{
    return static_cast<std::uint32_t>(presetTable().size());
}

const TreePreset& TreeGenerator::preset(std::uint32_t index)
{
    const ct::Vector<TreePreset>& table = presetTable();
    return table[index < table.size() ? index : table.size() - 1];
}

TreeParams TreeGenerator::randomParams(Random& random)
{
    TreeParams params;
    params.seed = random.randiRange(1, 99999);

    params.levels = static_cast<std::uint32_t>(random.randiRange(3, 5));
    params.trunkSteps = static_cast<std::uint32_t>(random.randiRange(1, 5));
    params.segments = static_cast<std::uint32_t>(random.randiRange(3, 5)) * 2u;

    params.trunkLength = random.randfRange(1.0f, 3.4f);
    params.initialBranchLength = random.randfRange(0.55f, 1.0f);

    params.lengthFalloffFactor = random.randfRange(0.78f, 0.96f);
    params.lengthFalloffPower = random.randfRange(0.8f, 1.2f);

    params.branchFactor = random.randfRange(2.0f, 3.5f);

    params.clumpMax = random.randfRange(0.55f, 0.95f);
    params.clumpMin = random.randfRange(0.2f, params.clumpMax - 0.1f);

    params.dropAmount = random.randfRange(-0.25f, 0.40f);
    params.growAmount = random.randfRange(-0.20f, 0.35f);
    params.sweepAmount = random.randfRange(-0.08f, 0.08f);

    params.maxRadius = random.randfRange(0.07f, 0.24f);
    params.radiusFalloffRate = random.randfRange(0.5f, 0.75f);
    params.taperRate = random.randfRange(0.86f, 0.99f);

    params.climbRate = random.randfRange(0.35f, 1.6f);
    params.trunkKink = random.randfRange(0.0f, 0.18f);
    params.twistRate = random.randfRange(0.0f, 20.0f);

    params.vMultiplier = random.randfRange(0.15f, 1.0f);
    params.twigScale = random.randfRange(0.22f, 0.6f);
    return params;
}

} // namespace prisma3d
