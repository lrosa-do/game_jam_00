#include "prisma3d/scene/HairStrands.h"

#include <math.h>
#include <string.h>

namespace prisma3d
{

namespace
{

const float kPi = 3.14159265358979f;

struct Rng
{
    std::uint32_t x = 0;
    std::uint32_t y = 0;

    static std::uint32_t rotl(std::uint32_t v, std::uint32_t k)
    {
        return (v << k) | (v >> (32u - k));
    }

    std::uint32_t next()
    {
        const std::uint32_t result = x * 0x9e3779bbu;
        y ^= x;
        x = rotl(x, 26) ^ y ^ (y << 9);
        y = rotl(y, 13);
        return result;
    }

    static std::uint32_t hash(std::uint32_t seed)
    {
        seed = (seed ^ 61u) ^ (seed >> 16);
        seed *= 9u;
        seed = seed ^ (seed >> 4);
        seed *= 0x27d4eb2du;
        seed = seed ^ (seed >> 15);
        return seed;
    }

    void init(std::uint32_t idX, std::uint32_t idY, std::uint32_t frameIndex)
    {
        const std::uint32_t s0 = (idX << 16) | idY;
        x = hash(s0);
        y = hash(frameIndex);
        next();
    }

    float nextFloat()
    {
        const std::uint32_t u = 0x3f800000u | (next() >> 9);
        float value = 0.0f;
        memcpy(&value, &u, sizeof(value));
        return value - 1.0f;
    }

    std::uint32_t nextUint(std::uint32_t nmax)
    {
        return static_cast<std::uint32_t>(floorf(nextFloat() * static_cast<float>(nmax)));
    }
};

struct Mat3Rows
{
    Math::Vec3 row[3];
};

Math::Vec3 mulRow(const Math::Vec3& v, const Mat3Rows& m)
{
    return m.row[0] * v.x + m.row[1] * v.y + m.row[2] * v.z;
}

Mat3Rows mul(const Mat3Rows& a, const Mat3Rows& b)
{
    Mat3Rows out;
    for (int i = 0; i < 3; ++i) out.row[i] = mulRow(a.row[i], b);
    return out;
}

float saturate(float v)
{
    return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v);
}

float fract(float v)
{
    return v - floorf(v);
}

Math::Vec3 attributeAtBary(const Math::Vec3& a0, const Math::Vec3& a1, const Math::Vec3& a2,
        float f, float g)
{
    return a0 * (1.0f - f - g) + a1 * f + a2 * g;
}

float attributeAtBary(float a0, float a1, float a2, float f, float g)
{
    return a0 * (1.0f - f - g) + a1 * f + a2 * g;
}

Mat3Rows tangentSpace(const Math::Vec3& normal)
{
    const Math::Vec3 helper =
            fabsf(normal.x) > 0.99f ? Math::Vec3(0.0f, 0.0f, 1.0f) : Math::Vec3(1.0f, 0.0f, 0.0f);
    const Math::Vec3 tangent = normal.Cross(helper).Normalized();
    const Math::Vec3 binormal = normal.Cross(tangent).Normalized();
    Mat3Rows out;
    out.row[0] = tangent;
    out.row[1] = binormal;
    out.row[2] = normal;
    return out;
}

Math::Vec3 hemispherePointCos(float u, float v)
{
    const float phi = v * 2.0f * kPi;
    const float cosTheta = sqrtf(1.0f - u);
    const float sinTheta = sqrtf(1.0f - cosTheta * cosTheta);
    return Math::Vec3(cosf(phi) * sinTheta, sinf(phi) * sinTheta, cosTheta);
}

float hashUnit(std::uint32_t h)
{
    return static_cast<float>(h >> 8) * (1.0f / 16777216.0f) * 2.0f - 1.0f;
}

Math::Vec3 hashGradient3D(const Math::Vec3& p)
{
    const std::uint32_t x = static_cast<std::uint32_t>(static_cast<std::int32_t>(p.x));
    const std::uint32_t y = static_cast<std::uint32_t>(static_cast<std::int32_t>(p.y));
    const std::uint32_t z = static_cast<std::uint32_t>(static_cast<std::int32_t>(p.z));
    const std::uint32_t seed = Rng::hash(x * 73856093u ^ y * 19349663u ^ z * 83492791u);
    const std::uint32_t h0 = Rng::hash(seed);
    const std::uint32_t h1 = Rng::hash(h0);
    const std::uint32_t h2 = Rng::hash(h1);
    return Math::Vec3(hashUnit(h0), hashUnit(h1), hashUnit(h2));
}

float lerp(float a, float b, float t)
{
    return a + (b - a) * t;
}

Math::Vec3 closestPointOnSegment(const Math::Vec3& a, const Math::Vec3& b, const Math::Vec3& c)
{
    const Math::Vec3 ab = b - a;
    const float t = (c - a).Dot(ab) / ab.Dot(ab);
    return a + ab * saturate(t);
}

float planePointDistance(const Math::Vec3& origin, const Math::Vec3& normal, const Math::Vec3& p)
{
    return normal.Dot(p - origin);
}

} // namespace

HairCollider HairCollider::sphere(const Math::Vec3& center, float radius)
{
    HairCollider collider;
    collider.type = HairColliderType::Sphere;
    collider.position = center;
    collider.radius = radius;
    return collider;
}

HairCollider HairCollider::capsule(const Math::Vec3& base, const Math::Vec3& tip, float radius)
{
    HairCollider collider;
    collider.type = HairColliderType::Capsule;
    collider.position = base;
    collider.tip = tip;
    collider.radius = radius;
    return collider;
}

HairCollider HairCollider::plane(const Math::Mat4& world, const Math::Vec3& offset, float radius)
{
    HairCollider collider;
    collider.type = HairColliderType::Plane;
    collider.position = world.TransformPoint(offset);
    collider.normal = world.TransformVector(Math::Vec3(0.0f, 1.0f, 0.0f)).Normalized();
    collider.radius = radius;
    const Math::Mat4 plane = world * Math::Mat4::Translation(offset) *
                             Math::Mat4::Scale(Math::Vec3(radius, 1.0f, radius));
    collider.planeProjection = plane.Inverse();
    return collider;
}

void HairStrands::setParams(const HairParams& params)
{
    mParams = params;
    if (mParams.segmentCount < 1) mParams.segmentCount = 1;
    if (mParams.billboardCount < 1) mParams.billboardCount = 1;
    mAspect = mParams.width * mParams.textureAspect;
}

const HairParams& HairStrands::params() const
{
    return mParams;
}

void HairStrands::setScalp(const Math::Vec3* positions, const Math::Vec3* normals,
        const float* lengths, std::uint32_t vertexCount, const std::uint32_t* indices,
        std::uint32_t indexCount)
{
    mScalpPositions.clear();
    mScalpNormals.clear();
    mScalpLengths.clear();
    mScalpIndices.clear();
    for (std::uint32_t i = 0; i < vertexCount; ++i)
    {
        mScalpPositions.push_back(positions[i]);
        mScalpNormals.push_back(normals[i]);
        mScalpLengths.push_back(lengths ? lengths[i] : 1.0f);
    }
    for (std::uint32_t i = 0; i + 2 < indexCount; i += 3)
    {
        const std::uint32_t i0 = indices[i];
        const std::uint32_t i1 = indices[i + 1];
        const std::uint32_t i2 = indices[i + 2];
        if (i0 >= vertexCount || i1 >= vertexCount || i2 >= vertexCount) continue;
        if (mScalpLengths[i0] > 0.0f || mScalpLengths[i1] > 0.0f || mScalpLengths[i2] > 0.0f)
        {
            mScalpIndices.push_back(i0);
            mScalpIndices.push_back(i1);
            mScalpIndices.push_back(i2);
        }
    }
    mRegenerate = true;
}

void HairStrands::updateScalp(const Math::Vec3* positions, const Math::Vec3* normals)
{
    for (std::size_t i = 0; i < mScalpPositions.size(); ++i)
    {
        mScalpPositions[i] = positions[i];
        mScalpNormals[i] = normals[i];
    }
}

std::uint32_t HairStrands::scalpTriangleCount() const
{
    return static_cast<std::uint32_t>(mScalpIndices.size() / 3);
}

const std::uint32_t* HairStrands::scalpIndices() const
{
    return mScalpIndices.data();
}

const float* HairStrands::scalpLengths() const
{
    return mScalpLengths.data();
}

void HairStrands::reset()
{
    mRegenerate = true;
}

std::uint32_t HairStrands::verticesPerStrand() const
{
    return (mParams.segmentCount * 2 + 2) * mParams.billboardCount;
}

std::uint32_t HairStrands::vertexCount() const
{
    return mParams.strandCount * verticesPerStrand();
}

std::uint32_t HairStrands::indexCount() const
{
    return 6 * mParams.strandCount * mParams.segmentCount * mParams.billboardCount;
}

const Math::Vec3* HairStrands::positions() const
{
    return mPositions.data();
}

const Math::Vec3* HairStrands::normals() const
{
    return mNormals.data();
}

const Math::Vec2* HairStrands::uvs() const
{
    return mUvs.data();
}

const std::uint32_t* HairStrands::indices() const
{
    return mIndices.data();
}

Math::Vec3 HairStrands::tail(std::uint32_t strand, std::uint32_t segment) const
{
    const std::size_t particle = static_cast<std::size_t>(strand) * mParams.segmentCount + segment;
    return particle < mTails.size() ? mTails[particle].current : Math::Vec3(0.0f, 0.0f, 0.0f);
}

void HairStrands::ensureBuffers()
{
    if (mBuiltStrands == mParams.strandCount && mBuiltSegments == mParams.segmentCount &&
            mBuiltBillboards == mParams.billboardCount && mPositions.size() == vertexCount())
        return;
    mBuiltStrands = mParams.strandCount;
    mBuiltSegments = mParams.segmentCount;
    mBuiltBillboards = mParams.billboardCount;
    const std::uint32_t vertices = vertexCount();
    mPositions.resize(vertices);
    mNormals.resize(vertices);
    mUvs.resize(vertices);
    mTails.resize(static_cast<std::size_t>(mParams.strandCount) * mParams.segmentCount);
    mIndicesDirty = true;
    mRegenerate = true;
}

Math::Box HairStrands::expandBounds(const Math::Box& scalpBounds, const HairParams& params,
        const Math::Mat4& world)
{
    float maxLength = params.length;
    for (std::size_t i = 0; i < params.atlasRects.size(); ++i)
        if (params.atlasRects[i].size > maxLength) maxLength = params.atlasRects[i].size;
    const Math::Vec3 extent(maxLength, maxLength, maxLength);
    const Math::Vec3 lo = scalpBounds.min - extent;
    const Math::Vec3 hi = scalpBounds.max + extent;
    Math::Box out;
    bool first = true;
    for (int corner = 0; corner < 8; ++corner)
    {
        const Math::Vec3 local((corner & 1) ? hi.x : lo.x, (corner & 2) ? hi.y : lo.y,
                (corner & 4) ? hi.z : lo.z);
        const Math::Vec3 p = world.TransformPoint(local);
        if (first)
        {
            out.min = p;
            out.max = p;
            first = false;
        }
        else
        {
            out.min = Math::Vec3::Min(out.min, p);
            out.max = Math::Vec3::Max(out.max, p);
        }
    }
    return out;
}

float HairStrands::noiseGradient3D(const Math::Vec3& p)
{
    const Math::Vec3 i(floorf(p.x), floorf(p.y), floorf(p.z));
    const Math::Vec3 f = p - i;
    const Math::Vec3 u(f.x * f.x * (3.0f - 2.0f * f.x), f.y * f.y * (3.0f - 2.0f * f.y),
            f.z * f.z * (3.0f - 2.0f * f.z));
    const float c000 = hashGradient3D(i).Dot(f);
    const float c100 =
            hashGradient3D(i + Math::Vec3(1.0f, 0.0f, 0.0f)).Dot(f - Math::Vec3(1.0f, 0.0f, 0.0f));
    const float c010 =
            hashGradient3D(i + Math::Vec3(0.0f, 1.0f, 0.0f)).Dot(f - Math::Vec3(0.0f, 1.0f, 0.0f));
    const float c110 =
            hashGradient3D(i + Math::Vec3(1.0f, 1.0f, 0.0f)).Dot(f - Math::Vec3(1.0f, 1.0f, 0.0f));
    const float c001 =
            hashGradient3D(i + Math::Vec3(0.0f, 0.0f, 1.0f)).Dot(f - Math::Vec3(0.0f, 0.0f, 1.0f));
    const float c101 =
            hashGradient3D(i + Math::Vec3(1.0f, 0.0f, 1.0f)).Dot(f - Math::Vec3(1.0f, 0.0f, 1.0f));
    const float c011 =
            hashGradient3D(i + Math::Vec3(0.0f, 1.0f, 1.0f)).Dot(f - Math::Vec3(0.0f, 1.0f, 1.0f));
    const float c111 =
            hashGradient3D(i + Math::Vec3(1.0f, 1.0f, 1.0f)).Dot(f - Math::Vec3(1.0f, 1.0f, 1.0f));
    return lerp(lerp(lerp(c000, c100, u.x), lerp(c010, c110, u.x), u.y),
            lerp(lerp(c001, c101, u.x), lerp(c011, c111, u.x), u.y), u.z);
}

float HairStrands::computeWind(const Math::Vec3& texel, float time, float speed, float randomness,
        float waveSize)
{
    Math::Vec3 position = texel + Math::Vec3(time, time, time);
    position = position * waveSize;
    float waveOffset = 0.0f;
    waveOffset += noiseGradient3D(position);
    waveOffset += noiseGradient3D(position * (1.0f / 2.0f));
    waveOffset += noiseGradient3D(position * (1.0f / 4.0f));
    waveOffset += noiseGradient3D(position * (1.0f / 8.0f));
    waveOffset += noiseGradient3D(position * (1.0f / 16.0f));
    waveOffset += noiseGradient3D(position * (1.0f / 32.0f));
    waveOffset *= randomness;
    return sinf(time * speed + waveOffset);
}

Math::Vec3 HairStrands::sampleWind(const Math::Vec3& position, float weight) const
{
    if (!(weight > 0.0f) || !mWindActive) return Math::Vec3(0.0f, 0.0f, 0.0f);
    const int size = static_cast<int>(kWindTextureSize);
    float coords[3] = { position.x, position.y, position.z };
    int i0[3];
    int i1[3];
    float w[3];
    for (int axis = 0; axis < 3; ++axis)
    {
        float f = fract(coords[axis] * 0.5f) * 2.0f;
        if (f > 1.0f) f = 2.0f - f;
        const float t = f * static_cast<float>(size) - 0.5f;
        const float base = floorf(t);
        w[axis] = t - base;
        int a = static_cast<int>(base);
        int b = a + 1;
        if (a < 0) a = 0;
        if (b < 0) b = 0;
        if (a >= size) a = size - 1;
        if (b >= size) b = size - 1;
        i0[axis] = a;
        i1[axis] = b;
    }
    const float* texels = mWind.data();
    float value = 0.0f;
    for (int corner = 0; corner < 8; ++corner)
    {
        const int x = (corner & 1) ? i1[0] : i0[0];
        const int y = (corner & 2) ? i1[1] : i0[1];
        const int z = (corner & 4) ? i1[2] : i0[2];
        const float weightX = (corner & 1) ? w[0] : 1.0f - w[0];
        const float weightY = (corner & 2) ? w[1] : 1.0f - w[1];
        const float weightZ = (corner & 4) ? w[2] : 1.0f - w[2];
        value += texels[(z * size + y) * size + x] * weightX * weightY * weightZ;
    }
    return mFrame.windDirection * (value * weight);
}

void HairStrands::beginFrame(const HairFrame& frame)
{
    mFrame = frame;
    ensureBuffers();
    const Math::Mat4 inverse = frame.transform.Inverse();
    mNormalTransform = inverse;
    const bool windWanted = frame.windDirection.x != 0.0f || frame.windDirection.y != 0.0f ||
                            frame.windDirection.z != 0.0f;
    const std::size_t texels =
            static_cast<std::size_t>(kWindTextureSize) * kWindTextureSize * kWindTextureSize;
    mWindBegin = 0;
    mWindEnd = 0;
    if (windWanted)
    {
        const bool fresh = !mWindActive || mWind.size() != texels;
        if (fresh)
        {
            mWind.resize(texels);
            mWindBegin = 0;
            mWindEnd = kWindTextureSize;
            mWindCursor = 0;
        }
        else
        {
            std::uint32_t slices = mWindSlicesPerFrame;
            if (slices == 0 || slices > kWindTextureSize) slices = kWindTextureSize;
            mWindBegin = mWindCursor;
            mWindEnd = mWindCursor + slices;
            if (mWindEnd > kWindTextureSize) mWindEnd = kWindTextureSize;
            mWindCursor = mWindEnd >= kWindTextureSize ? 0 : mWindEnd;
        }
        if (mWindEnd > mWindBegin) prepareWindLattices();
    }
    mWindActive = windWanted;
}

void HairStrands::windSlicesToCompute(std::uint32_t& begin, std::uint32_t& end) const
{
    begin = mWindBegin;
    end = mWindEnd;
}

void HairStrands::prepareWindLattices()
{
    const float time = mFrame.time;
    for (std::uint32_t octave = 0; octave < kWindOctaves; ++octave)
    {
        WindLattice& lattice = mWindLattices[octave];
        const float scale = mFrame.windWaveSize / static_cast<float>(1u << octave);
        const int lo = static_cast<int>(floorf(time * scale));
        const int hi = static_cast<int>(floorf(
                               (static_cast<float>(kWindTextureSize) - 1.0f + time) * scale)) +
                       1;
        const int n = hi - lo + 1;
        lattice.valid = n > 0 && n <= kWindLatticeLimit;
        if (!lattice.valid) continue;
        lattice.base = lo;
        lattice.size = n;
        lattice.gradients.resize(static_cast<std::size_t>(n) * n * n);
        for (int z = 0; z < n; ++z)
            for (int y = 0; y < n; ++y)
                for (int x = 0; x < n; ++x)
                    lattice.gradients[(static_cast<std::size_t>(z) * n + y) * n + x] =
                            hashGradient3D(Math::Vec3(static_cast<float>(lo + x),
                                    static_cast<float>(lo + y), static_cast<float>(lo + z)));
    }
}

float HairStrands::latticeNoise(const Math::Vec3& p, const WindLattice& lattice) const
{
    const Math::Vec3 i(floorf(p.x), floorf(p.y), floorf(p.z));
    const Math::Vec3 f = p - i;
    const Math::Vec3 u(f.x * f.x * (3.0f - 2.0f * f.x), f.y * f.y * (3.0f - 2.0f * f.y),
            f.z * f.z * (3.0f - 2.0f * f.z));
    const int n = lattice.size;
    const int x0 = static_cast<int>(i.x) - lattice.base;
    const int y0 = static_cast<int>(i.y) - lattice.base;
    const int z0 = static_cast<int>(i.z) - lattice.base;
    if (x0 < 0 || y0 < 0 || z0 < 0 || x0 + 1 >= n || y0 + 1 >= n || z0 + 1 >= n)
        return noiseGradient3D(p);
    const Math::Vec3* g = lattice.gradients.data();
    const std::size_t stride = static_cast<std::size_t>(n);
    const std::size_t o = (static_cast<std::size_t>(z0) * n + y0) * n + x0;
    const float c000 = g[o].Dot(f);
    const float c100 = g[o + 1].Dot(f - Math::Vec3(1.0f, 0.0f, 0.0f));
    const float c010 = g[o + stride].Dot(f - Math::Vec3(0.0f, 1.0f, 0.0f));
    const float c110 = g[o + stride + 1].Dot(f - Math::Vec3(1.0f, 1.0f, 0.0f));
    const float c001 = g[o + stride * stride].Dot(f - Math::Vec3(0.0f, 0.0f, 1.0f));
    const float c101 = g[o + stride * stride + 1].Dot(f - Math::Vec3(1.0f, 0.0f, 1.0f));
    const float c011 = g[o + stride * stride + stride].Dot(f - Math::Vec3(0.0f, 1.0f, 1.0f));
    const float c111 = g[o + stride * stride + stride + 1].Dot(f - Math::Vec3(1.0f, 1.0f, 1.0f));
    return lerp(lerp(lerp(c000, c100, u.x), lerp(c010, c110, u.x), u.y),
            lerp(lerp(c001, c101, u.x), lerp(c011, c111, u.x), u.y), u.z);
}

void HairStrands::computeWindSlices(std::uint32_t begin, std::uint32_t end)
{
    const int size = static_cast<int>(kWindTextureSize);
    if (end > kWindTextureSize) end = kWindTextureSize;
    const float time = mFrame.time;
    for (std::uint32_t z = begin; z < end; ++z)
        for (int y = 0; y < size; ++y)
            for (int x = 0; x < size; ++x)
            {
                Math::Vec3 position(static_cast<float>(x) + time, static_cast<float>(y) + time,
                        static_cast<float>(z) + time);
                position = position * mFrame.windWaveSize;
                float waveOffset = 0.0f;
                for (std::uint32_t octave = 0; octave < kWindOctaves; ++octave)
                {
                    const Math::Vec3 q = position * (1.0f / static_cast<float>(1u << octave));
                    const WindLattice& lattice = mWindLattices[octave];
                    waveOffset += lattice.valid ? latticeNoise(q, lattice) : noiseGradient3D(q);
                }
                waveOffset *= mFrame.windRandomness;
                mWind[(static_cast<std::size_t>(z) * size + y) * size + x] =
                        sinf(time * mFrame.windSpeed + waveOffset);
            }
}

void HairStrands::setWindSlicesPerFrame(std::uint32_t slices)
{
    mWindSlicesPerFrame = slices;
}

std::uint32_t HairStrands::windSlicesPerFrame() const
{
    return mWindSlicesPerFrame;
}

void HairStrands::simulateRange(std::uint32_t begin, std::uint32_t end)
{
    if (mScalpIndices.empty()) return;
    if (end > mParams.strandCount) end = mParams.strandCount;
    for (std::uint32_t strand = begin; strand < end; ++strand) simulateStrand(strand);
}

void HairStrands::endFrame()
{
    if (mIndicesDirty || mRegenerate) buildIndices();
    mRegenerate = false;
}

void HairStrands::simulate(const HairFrame& frame)
{
    beginFrame(frame);
    std::uint32_t begin = 0;
    std::uint32_t end = 0;
    windSlicesToCompute(begin, end);
    computeWindSlices(begin, end);
    simulateRange(0, mParams.strandCount);
    endFrame();
}

void HairStrands::buildIndices()
{
    mIndices.resize(indexCount());
    const std::uint32_t perStrand = verticesPerStrand();
    std::uint32_t i = 0;
    for (std::uint32_t strand = 0; strand < mParams.strandCount; ++strand)
    {
        const std::uint32_t vertexID0 = strand * perStrand;
        std::uint32_t rootOffset = vertexID0;
        std::uint32_t capOffset = rootOffset + 2 * mParams.billboardCount;
        for (std::uint32_t billboard = 0; billboard < mParams.billboardCount; ++billboard)
        {
            for (std::uint32_t segment = 0; segment < mParams.segmentCount; ++segment)
            {
                mIndices[i++] = rootOffset + 0;
                mIndices[i++] = rootOffset + 1;
                mIndices[i++] = capOffset + 0;
                mIndices[i++] = capOffset + 0;
                mIndices[i++] = rootOffset + 1;
                mIndices[i++] = capOffset + 1;
                rootOffset += 2;
                capOffset += 2;
            }
        }
    }
    mIndicesDirty = false;
}

void HairStrands::simulateStrand(std::uint32_t strand)
{
    const HairParams& params = mParams;
    const std::uint32_t segments = params.segmentCount;
    const std::uint32_t billboards = params.billboardCount;
    std::uint32_t v0 = strand * verticesPerStrand();

    Rng rng;
    rng.init(params.randomSeed, strand, 0);

    const std::uint32_t triangleCount = scalpTriangleCount();
    const std::uint32_t tri = rng.nextUint(triangleCount);
    const std::uint32_t i0 = mScalpIndices[tri * 3 + 0];
    const std::uint32_t i1 = mScalpIndices[tri * 3 + 1];
    const std::uint32_t i2 = mScalpIndices[tri * 3 + 2];

    float f = rng.nextFloat();
    float g = rng.nextFloat();
    if (f + g > 1.0f)
    {
        f = 1.0f - f;
        g = 1.0f - g;
    }

    Math::Vec3 base =
            attributeAtBary(mScalpPositions[i0], mScalpPositions[i1], mScalpPositions[i2], f, g);
    Math::Vec3 target =
            attributeAtBary(mScalpNormals[i0], mScalpNormals[i1], mScalpNormals[i2], f, g)
                    .Normalized();
    target = Math::Vec3(mNormalTransform.col0.x * target.x + mNormalTransform.col0.y * target.y +
                                mNormalTransform.col0.z * target.z,
            mNormalTransform.col1.x * target.x + mNormalTransform.col1.y * target.y +
                    mNormalTransform.col1.z * target.z,
            mNormalTransform.col2.x * target.x + mNormalTransform.col2.y * target.y +
                    mNormalTransform.col2.z * target.z)
                     .Normalized();
    const Math::Vec3 hemisphere = hemispherePointCos(rng.nextFloat(), rng.nextFloat());
    Math::Vec3 tangent =
            mulRow(Math::Vec3(hemisphere.x, hemisphere.y, 0.0f), tangentSpace(target)).Normalized();
    Math::Vec3 binormal = target.Cross(tangent);
    const float strandLength =
            attributeAtBary(mScalpLengths[i0], mScalpLengths[i1], mScalpLengths[i2], f, g);

    HairAtlasRect atlas;
    float atlasAspect = 1.0f;
    if (params.atlasRects.size() > 1)
    {
        const float noise = noiseGradient3D(base * params.uniformity) * 1000.0f;
        const std::uint32_t frame = noise > 0.0f ? static_cast<std::uint32_t>(noise) : 0u;
        atlas = params.atlasRects[frame % static_cast<std::uint32_t>(params.atlasRects.size())];
        atlasAspect = atlas.texMulAdd.x / atlas.texMulAdd.y;
    }
    else if (params.atlasRects.size() == 1)
    {
        atlas = params.atlasRects[0];
        atlasAspect = atlas.texMulAdd.x / atlas.texMulAdd.y;
    }

    base = mFrame.transform.TransformPoint(base);
    const Math::Vec3 collapsedBase = base;

    const Math::Vec3 diff = mFrame.cameraPosition - base;
    const float distanceSquared = diff.Dot(diff);
    const bool distanceCulled = distanceSquared > params.viewDistance * params.viewDistance;

    float len = lerp(1.0f, rng.nextFloat(), saturate(params.randomness)) * strandLength;
    len *= params.length;
    len *= atlas.size;
    len /= static_cast<float>(segments);
    const float frameX = atlasAspect * mAspect * static_cast<float>(segments) * len * 0.5f;
    const float frameY = len * 0.5f;
    const float segmentRadius = frameX > frameY ? frameX : frameY;

    Math::Vec3 bend(0.0f, 0.0f, 0.0f);
    if (params.cameraBend)
        bend = mFrame.cameraUp * ((1.0f - saturate(target.Dot(mFrame.cameraUp))) * 0.8f);

    const Math::Vec3 billboardNormal = (target + bend).Normalized();

    Mat3Rows tbn;
    tbn.row[0] = tangent;
    tbn.row[1] = billboardNormal;
    tbn.row[2] = binormal;

    const Math::Vec3 patchRoot[2] = { Math::Vec3(-1.0f, -1.0f, 0.0f),
        Math::Vec3(1.0f, -1.0f, 0.0f) };
    const Math::Vec3 patchCap[2] = { Math::Vec3(-1.0f, 1.0f, 0.0f), Math::Vec3(1.0f, 1.0f, 0.0f) };

    if (distanceCulled)
    {
        for (std::uint32_t i = 0; i < 2 * billboards; ++i)
        {
            mPositions[v0] = collapsedBase;
            mNormals[v0] = billboardNormal;
            mUvs[v0] = Math::Vec2(0.0f, 0.0f);
            ++v0;
        }
    }
    else
    {
        rng.init(params.randomSeed, strand, 1);
        for (std::uint32_t billboard = 0; billboard < billboards; ++billboard)
        {
            const float siz = billboard == 0 ? 1.0f : lerp(0.2f, 1.0f, rng.nextFloat());
            const float rot = billboard == 0 ? 0.0f : rng.nextFloat() * kPi;
            Mat3Rows variation;
            if (billboard == 0)
            {
                variation = tbn;
            }
            else
            {
                const float s = sinf(rot);
                const float c = cosf(rot);
                Mat3Rows r;
                r.row[0] = Math::Vec3(c * siz, 0.0f, -s);
                r.row[1] = Math::Vec3(0.0f, siz, 0.0f);
                r.row[2] = Math::Vec3(s, 0.0f, c * siz);
                variation = mul(r, tbn);
            }
            for (int vertex = 0; vertex < 2; ++vertex)
            {
                Math::Vec3 patchPos = patchRoot[vertex];
                Math::Vec2 uv(patchPos.x * 0.5f + 0.5f, patchPos.y * 0.5f + 0.5f);
                uv.y = 1.0f - uv.y;
                patchPos.y += 1.0f;
                uv = Math::Vec2(uv.x * atlas.texMulAdd.x + atlas.texMulAdd.z,
                        uv.y * atlas.texMulAdd.y + atlas.texMulAdd.w);
                patchPos =
                        Math::Vec3(patchPos.x * frameX, patchPos.y * frameY, patchPos.z * frameX);
                patchPos = mulRow(patchPos, variation);
                mPositions[v0] = base + patchPos;
                mNormals[v0] = billboardNormal;
                mUvs[v0] = uv;
                ++v0;
            }
        }
    }

    float dt = mFrame.deltaTime;
    if (dt < 0.0f) dt = 0.0f;
    if (dt > 1.0f / 30.0f) dt = 1.0f / 30.0f;

    const float gravityPower = params.gravityPower;
    const float stiffnessForce = params.stiffness;
    const float dragForce = params.drag;
    const Math::Vec3 boneAxis = target;
    const float boneLength = len;

    for (std::uint32_t segment = 0; segment < segments; ++segment)
    {
        const std::size_t particle = static_cast<std::size_t>(strand) * segments + segment;
        Tail& tail = mTails[particle];

        Math::Vec3 tailCurrent;
        Math::Vec3 tailNext;
        Math::Vec3 toTail;
        if (distanceCulled)
        {
            tailCurrent = base + boneAxis * boneLength;
            tailNext = tailCurrent;
            toTail = boneAxis;
        }
        else
        {
            if (mRegenerate)
            {
                const Math::Vec3 rest = base + boneAxis * boneLength;
                tail.previous = rest;
                tail.current = rest;
            }
            tailCurrent = tail.current;
            const Math::Vec3 tailPrev = tail.previous;
            const Math::Vec3 inertia = (tailCurrent - tailPrev) * (1.0f - dragForce);
            const Math::Vec3 stiffness = boneAxis * stiffnessForce;
            Math::Vec3 external = Math::Vec3(0.0f, -1.0f, 0.0f) * gravityPower;
            external = external + sampleWind(tailCurrent, static_cast<float>(segment + 1) /
                                                                  static_cast<float>(segments));
            tailNext = tailCurrent + inertia + (stiffness + external) * dt;
            toTail = (tailNext - base).Normalized();
            tailNext = base + toTail * boneLength;
        }

        if (!distanceCulled)
        {
            for (std::uint32_t c = 0; c < mFrame.colliderCount; ++c)
            {
                const HairCollider& collider = mFrame.colliders[c];
                if (collider.type == HairColliderType::Capsule)
                {
                    const Math::Vec3 closest =
                            closestPointOnSegment(collider.position, collider.tip, tailNext);
                    Math::Vec3 dir = closest - tailNext;
                    float dist = dir.Length();
                    dir = dir * (1.0f / dist);
                    dist = dist - collider.radius - segmentRadius;
                    if (dist < 0.0f)
                    {
                        tailNext = tailNext + dir * dist;
                        toTail = (tailNext - base).Normalized();
                        tailNext = base + toTail * boneLength;
                    }
                }
                else
                {
                    const Math::Vec3 closest =
                            closestPointOnSegment(base, tailNext, collider.position);
                    Math::Vec3 dir = collider.position - closest;
                    float dist = dir.Length();
                    dir = dir * (1.0f / dist);
                    if (collider.type == HairColliderType::Sphere)
                    {
                        dist = dist - collider.radius - segmentRadius;
                        if (dist < 0.0f)
                        {
                            tailNext = tailNext + dir * dist;
                            toTail = (tailNext - base).Normalized();
                            tailNext = base + toTail * boneLength;
                        }
                    }
                    else
                    {
                        dir = collider.normal.Normalized();
                        dist = planePointDistance(collider.position, dir, closest);
                        if (dist < 0.0f)
                        {
                            dir = dir * -1.0f;
                            dist = fabsf(dist);
                        }
                        dist = dist - segmentRadius;
                        if (dist < 0.0f)
                        {
                            const Math::Vec4 clip =
                                    collider.planeProjection * Math::Vec4(closest, 1.0f);
                            const Math::Vec3 uvw(clip.x * 0.5f + 0.5f, clip.y * -0.5f + 0.5f,
                                    clip.z * 0.5f + 0.5f);
                            if (uvw.x >= 0.0f && uvw.x <= 1.0f && uvw.y >= 0.0f && uvw.y <= 1.0f &&
                                    uvw.z >= 0.0f && uvw.z <= 1.0f)
                            {
                                tailNext = tailNext - dir * dist;
                                toTail = (tailNext - base).Normalized();
                                tailNext = base + toTail * boneLength;
                            }
                        }
                    }
                }
            }
            for (std::uint32_t fi = 0; fi < mFrame.forceCount; ++fi)
            {
                const HairForceField& force = mFrame.forces[fi];
                const Math::Vec3 closest = closestPointOnSegment(base, tailNext, force.position);
                Math::Vec3 dir = force.position - closest;
                const float dist = dir.Length();
                dir = dir * (1.0f / dist);
                const float falloff = 1.0f - saturate(dist / force.range);
                if (force.type == HairForceType::Point)
                    tailNext = tailNext + dir * (dt * force.gravity * falloff);
                else
                    tailNext = tailNext + force.direction * (dt * force.gravity * falloff);
                toTail = (tailNext - base).Normalized();
                tailNext = base + toTail * boneLength;
            }
        }

        const float belowPlane = planePointDistance(base, boneAxis, tailNext);
        if (belowPlane < 0.0f) tailNext = tailNext - boneAxis * belowPlane;

        tail.previous = tailCurrent;
        tail.current = tailNext;

        const Math::Vec3 normal = toTail;
        const Math::Vec3 normalBend = (normal + bend).Normalized();
        binormal = normalBend.Cross(tangent);
        tangent = binormal.Cross(normalBend);
        tbn.row[0] = tangent;
        tbn.row[1] = normalBend;
        tbn.row[2] = binormal;

        if (distanceCulled)
        {
            for (std::uint32_t i = 0; i < 2 * billboards; ++i)
            {
                mPositions[v0] = collapsedBase;
                mNormals[v0] = normal;
                mUvs[v0] = Math::Vec2(0.0f, 0.0f);
                ++v0;
            }
        }
        else
        {
            rng.init(params.randomSeed, strand, 1);
            for (std::uint32_t billboard = 0; billboard < billboards; ++billboard)
            {
                const float siz = billboard == 0 ? 1.0f : lerp(0.2f, 1.0f, rng.nextFloat());
                const float rot = billboard == 0 ? 0.0f : rng.nextFloat() * kPi;
                Mat3Rows variation;
                if (billboard == 0)
                {
                    variation = tbn;
                }
                else
                {
                    const float s = sinf(rot);
                    const float c = cosf(rot);
                    Mat3Rows r;
                    r.row[0] = Math::Vec3(c * siz, 0.0f, -s);
                    r.row[1] = Math::Vec3(0.0f, siz, 0.0f);
                    r.row[2] = Math::Vec3(s, 0.0f, c * siz);
                    variation = mul(r, tbn);
                }
                for (int vertex = 0; vertex < 2; ++vertex)
                {
                    Math::Vec3 patchPos = patchCap[vertex];
                    Math::Vec2 uv(patchPos.x * 0.5f + 0.5f, patchPos.y * 0.5f + 0.5f);
                    uv.y = lerp(static_cast<float>(segment) / static_cast<float>(segments),
                            static_cast<float>(segment + 1) / static_cast<float>(segments), uv.y);
                    uv.y = 1.0f - uv.y;
                    patchPos.y += 1.0f;
                    uv = Math::Vec2(uv.x * atlas.texMulAdd.x + atlas.texMulAdd.z,
                            uv.y * atlas.texMulAdd.y + atlas.texMulAdd.w);
                    patchPos = Math::Vec3(patchPos.x * frameX, patchPos.y * frameY,
                            patchPos.z * frameX);
                    patchPos = mulRow(patchPos, variation);
                    mPositions[v0] = base + patchPos;
                    mNormals[v0] = normalBend;
                    mUvs[v0] = uv;
                    ++v0;
                }
            }
        }

        base = tailNext;
    }
}

} // namespace prisma3d
