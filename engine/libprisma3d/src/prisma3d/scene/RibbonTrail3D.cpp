#include "prisma3d/scene/RibbonTrail3D.h"

#include "prisma3d/scene/Camera3D.h"
#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/ResourceProperties.h"
#include "prisma3d/scene/SceneJson.h"
#include "prisma3d/scene/SceneResources.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/Viewport.h"

#include <ct/json.hpp>

#include <math.h>

namespace prisma3d
{

namespace
{

const char* const kMeshName = "ribbon trail";

std::uint16_t nextRing(std::uint16_t index, int maxElements)
{
    return static_cast<std::uint16_t>((index + 1) % static_cast<std::uint16_t>(maxElements));
}

Math::Vec3 catmullRom(const Math::Vec3& p0, const Math::Vec3& p1, const Math::Vec3& p2,
        const Math::Vec3& p3, float t)
{
    float t2 = t * t;
    float t3 = t2 * t;
    return Math::Vec3(0.5f * (2.f * p1.x + (p2.x - p0.x) * t +
                                     (2.f * p0.x - 5.f * p1.x + 4.f * p2.x - p3.x) * t2 +
                                     (3.f * p1.x - 3.f * p2.x + p3.x - p0.x) * t3),
            0.5f * (2.f * p1.y + (p2.y - p0.y) * t +
                           (2.f * p0.y - 5.f * p1.y + 4.f * p2.y - p3.y) * t2 +
                           (3.f * p1.y - 3.f * p2.y + p3.y - p0.y) * t3),
            0.5f * (2.f * p1.z + (p2.z - p0.z) * t +
                           (2.f * p0.z - 5.f * p1.z + 4.f * p2.z - p3.z) * t2 +
                           (3.f * p1.z - 3.f * p2.z + p3.z - p0.z) * t3));
}

void pushVertex(SurfaceData& geometry, const Math::Vec3& position, const Math::Vec3& normal,
        const Math::Vec4& color, float u, float v)
{
    geometry.positions.push_back(position);
    geometry.normals.push_back(normal);
    geometry.colors.push_back(color);
    geometry.uvs.push_back(Math::Vec2(u, v));
}

} // namespace

RibbonTrail3D::RibbonTrail3D()
        : VisualInstance3D(),
          mChainCount(0),
          mEmitting(true),
          mElements(),
          mMaxElements(48),
          mElemLength(0.0f),
          mTrailLength(1.2f),
          mSmoothing(4),
          mGeometry(),
          mSmoothHilt(),
          mSmoothTip(),
          mSmoothCol(),
          mSmoothW(),
          mEdgePos(),
          mEdgeCol(),
          mEdgeW(),
          mMesh(),
          mSurfaceReady(false),
          mUploadedEmpty(true),
          mDefaultMaterial(),
          mMaterial()
{

    mElements.resize(static_cast<std::size_t>(kMaxChains) * static_cast<std::size_t>(mMaxElements));
    mElemLength = mTrailLength / static_cast<float>(mMaxElements);

    setCastShadows(false);

    if (MaterialStore* materials = MaterialStore::singleton())
    {
        mDefaultMaterial = materials->create(kMeshName);
        materials->setShading(mDefaultMaterial, MaterialShading::Unshaded);
        materials->setBlendMode(mDefaultMaterial, MaterialBlend::Add);
        materials->setCull(mDefaultMaterial, MaterialCull::Disabled);
        mMaterial = mDefaultMaterial;
        applyMaterial();
    }
}

RibbonTrail3D::~RibbonTrail3D()
{
    disconnectPreDraw();

    if (RenderingServer* server = RenderingServer::singleton())
    {
        server->instanceSetMaterialOverride(getInstance(), MaterialId());
        server->instanceSetBase(getInstance(), MeshId());
    }
    if (MeshStore* store = MeshStore::singleton())
        if (mMesh.valid()) store->destroy(mMesh);
    if (MaterialStore* materials = MaterialStore::singleton())
        if (mDefaultMaterial.valid()) materials->destroy(mDefaultMaterial);
}

int RibbonTrail3D::addChain(const ct::String& emitterPath, const Math::Vec4& startColor,
        const Math::Vec4& endColor, float startWidth, float endWidth, float fadeTime)
{
    if (mChainCount >= kMaxChains || emitterPath.empty()) return -1;
    Chain& chain = mChains[mChainCount];
    chain = Chain();
    chain.emitterPath = emitterPath;
    chain.startColor = startColor;
    chain.endColor = endColor;
    chain.startWidth = startWidth;
    chain.endWidth = endWidth;
    chain.fadeTime = fadeTime;
    chain.blade = false;
    chain.active = true;
    seedChain(mChainCount);
    return mChainCount++;
}

int RibbonTrail3D::addBladeChain(const ct::String& basePath, const ct::String& tipPath,
        const Math::Vec4& startColor, const Math::Vec4& endColor, float fadeTime, float startSpan,
        float endSpan)
{
    if (mChainCount >= kMaxChains || basePath.empty() || tipPath.empty()) return -1;
    Chain& chain = mChains[mChainCount];
    chain = Chain();
    chain.emitterPath = basePath;
    chain.tipPath = tipPath;
    chain.blade = true;
    chain.startColor = startColor;
    chain.endColor = endColor;
    chain.startWidth = startSpan;
    chain.endWidth = endSpan;
    chain.fadeTime = fadeTime;
    chain.active = true;
    seedChain(mChainCount);
    return mChainCount++;
}

void RibbonTrail3D::clearChains()
{
    for (int i = 0; i < mChainCount; ++i) mChains[i].active = false;
    mChainCount = 0;
    mGeometry.positions.clear();
    mGeometry.normals.clear();
    mGeometry.colors.clear();
    mGeometry.uvs.clear();
    mGeometry.indices.clear();
}

int RibbonTrail3D::getChainCount() const
{
    return mChainCount;
}

int RibbonTrail3D::getElementCount(int chain) const
{
    if (chain < 0 || chain >= mChainCount) return 0;
    return mChains[chain].count;
}

void RibbonTrail3D::setEmitting(bool emitting)
{
    if (emitting == mEmitting) return;
    mEmitting = emitting;
    if (emitting)
    {
        for (int i = 0; i < mChainCount; ++i)
            if (mChains[i].active) seedChain(i);
    }
}

bool RibbonTrail3D::isEmitting() const
{
    return mEmitting;
}

void RibbonTrail3D::setTrailLength(float length)
{
    mTrailLength = length > 0.01f ? length : 0.01f;
    mElemLength = mTrailLength / static_cast<float>(mMaxElements);
}

float RibbonTrail3D::getTrailLength() const
{
    return mTrailLength;
}

void RibbonTrail3D::setMaxElements(int count)
{
    if (count < 2) count = 2;
    if (count > 64) count = 64;
    if (count == mMaxElements) return;
    mMaxElements = count;
    mElements.resize(static_cast<std::size_t>(kMaxChains) * static_cast<std::size_t>(mMaxElements));
    mElemLength = mTrailLength / static_cast<float>(mMaxElements);
    for (int i = 0; i < mChainCount; ++i)
        if (mChains[i].active) seedChain(i);
    invalidateSurface();
}

int RibbonTrail3D::getMaxElements() const
{
    return mMaxElements;
}

void RibbonTrail3D::setSmoothing(int subdivisions)
{
    if (subdivisions < 1) subdivisions = 1;
    if (subdivisions > 8) subdivisions = 8;
    if (subdivisions == mSmoothing) return;
    mSmoothing = subdivisions;
    invalidateSurface();
}

int RibbonTrail3D::getSmoothing() const
{
    return mSmoothing;
}

void RibbonTrail3D::setMaterial(MaterialId material)
{
    mMaterial = material.valid() ? material : mDefaultMaterial;
    applyMaterial();
}

MaterialId RibbonTrail3D::getMaterial() const
{
    return mMaterial;
}

MaterialId RibbonTrail3D::getAssignedMaterial() const
{
    return mMaterial == mDefaultMaterial ? MaterialId() : mMaterial;
}

void RibbonTrail3D::update(double delta)
{
    bake();
    fade(static_cast<float>(delta));
    buildGeometry();
    uploadGeometry();
}

const SurfaceData& RibbonTrail3D::getGeometry() const
{
    return mGeometry;
}

void RibbonTrail3D::notificationHandler(int what)
{
    VisualInstance3D::notificationHandler(what);

    switch (what)
    {
        case kNotificationEnterTree:
            connectPreDraw();
            break;
        case kNotificationExitTree:
            disconnectPreDraw();
            break;
        default:
            break;
    }
}

Node3D* RibbonTrail3D::resolveEmitter(const ct::String& path) const
{
    Node* node = getNode(path);
    return node ? node->castTo<Node3D>() : nullptr;
}

bool RibbonTrail3D::resolveChain(const Chain& chain, Node3D*& emitter, Node3D*& tip) const
{
    emitter = resolveEmitter(chain.emitterPath);
    tip = chain.blade ? resolveEmitter(chain.tipPath) : nullptr;
    return emitter && (!chain.blade || tip);
}

void RibbonTrail3D::resetTrail(Chain& chain, int chainIndex, Node3D* emitter, Node3D* tip)
{
    chain.head = kChainEmpty;
    chain.tail = kChainEmpty;
    chain.count = 0;
    if (!emitter || (chain.blade && !tip)) return;

    Math::Vec3 pos = chain.blade ? tip->getGlobalPosition() : emitter->getGlobalPosition();
    int base = chainIndex * mMaxElements;

    mElements[base].position = chain.blade ? emitter->getGlobalPosition() : pos;
    mElements[base].tip = pos;
    mElements[base].color = chain.startColor;
    mElements[base].width = chain.startWidth;

    chain.head = 0;
    chain.tail = 0;
    chain.count = 1;
    chain.lastBakedPos = pos;
}

void RibbonTrail3D::seedChain(int chainIndex)
{
    Chain& chain = mChains[chainIndex];
    Node3D* emitter = nullptr;
    Node3D* tip = nullptr;
    resolveChain(chain, emitter, tip);
    resetTrail(chain, chainIndex, emitter, tip);
}

void RibbonTrail3D::bake()
{
    for (int ci = 0; mEmitting && ci < mChainCount; ++ci)
    {
        Chain& ch = mChains[ci];
        if (!ch.active) continue;

        Node3D* emitter = nullptr;
        Node3D* tipEmitter = nullptr;
        if (!resolveChain(ch, emitter, tipEmitter)) continue;

        if (ch.count == 0)
        {
            resetTrail(ch, ci, emitter, tipEmitter);
            continue;
        }

        const bool blade = ch.blade;
        Math::Vec3 worldPos =
                blade ? tipEmitter->getGlobalPosition() : emitter->getGlobalPosition();
        Math::Vec3 basePos = blade ? emitter->getGlobalPosition() : worldPos;
        int base = ci * mMaxElements;
        int maxEl = mMaxElements;
        float sqElem = mElemLength * mElemLength;

        auto trackOf = [blade](Element& e) -> Math::Vec3& { return blade ? e.tip : e.position; };

        if (ch.count == 1)
        {
            float dx = worldPos.x - ch.lastBakedPos.x;
            float dy = worldPos.y - ch.lastBakedPos.y;
            float dz = worldPos.z - ch.lastBakedPos.z;
            if (dx * dx + dy * dy + dz * dz < sqElem * 0.25f) continue;

            Math::Vec3 dir(dx, dy, dz);
            float dlen = sqrtf(dir.x * dir.x + dir.y * dir.y + dir.z * dir.z);
            if (dlen <= 0.f) continue;
            dir = Math::Vec3(dir.x / dlen, dir.y / dlen, dir.z / dlen);
            Math::Vec3 baked(worldPos.x - dir.x * mElemLength, worldPos.y - dir.y * mElemLength,
                    worldPos.z - dir.z * mElemLength);

            trackOf(mElements[base + ch.head]) = baked;
            if (blade) mElements[base + ch.head].position = basePos;

            std::uint16_t newHead = nextRing(ch.head, maxEl);
            trackOf(mElements[base + newHead]) = worldPos;
            if (blade) mElements[base + newHead].position = basePos;
            mElements[base + newHead].color = ch.startColor;
            mElements[base + newHead].width = ch.startWidth;
            ch.head = newHead;
            ch.count = 2;
            ch.lastBakedPos = worldPos;
            continue;
        }

        Element& headElem = mElements[base + ch.head];
        Math::Vec3& headTrack = trackOf(headElem);
        float dx = worldPos.x - headTrack.x;
        float dy = worldPos.y - headTrack.y;
        float dz = worldPos.z - headTrack.z;
        float sqDist = dx * dx + dy * dy + dz * dz;

        if (sqDist < sqElem)
        {
            headTrack = worldPos;
            if (blade) headElem.position = basePos;
            continue;
        }

        float inv = 1.f / sqrtf(sqDist);
        Math::Vec3 dir(dx * inv, dy * inv, dz * inv);
        headTrack = Math::Vec3(worldPos.x - dir.x * mElemLength, worldPos.y - dir.y * mElemLength,
                worldPos.z - dir.z * mElemLength);
        if (blade) headElem.position = basePos;

        std::uint16_t newHead = nextRing(ch.head, maxEl);
        if (ch.count >= static_cast<std::uint16_t>(maxEl)) ch.tail = nextRing(ch.tail, maxEl);
        else
            ch.count++;

        trackOf(mElements[base + newHead]) = worldPos;
        if (blade) mElements[base + newHead].position = basePos;
        mElements[base + newHead].color = ch.startColor;
        mElements[base + newHead].width = ch.startWidth;
        ch.head = newHead;
        ch.lastBakedPos = worldPos;
    }
}

void RibbonTrail3D::fade(float dt)
{
    for (int ci = 0; ci < mChainCount; ++ci)
    {
        Chain& ch = mChains[ci];
        if (!ch.active || ch.count < 2) continue;

        int base = ci * mMaxElements;
        int maxEl = mMaxElements;
        float fadeTime = ch.fadeTime > 1e-4f ? ch.fadeTime : 1e-4f;
        float deltaWidth = (ch.startWidth - ch.endWidth) / fadeTime;
        Math::Vec4 deltaColor((ch.startColor.x - ch.endColor.x) / fadeTime,
                (ch.startColor.y - ch.endColor.y) / fadeTime,
                (ch.startColor.z - ch.endColor.z) / fadeTime,
                (ch.startColor.w - ch.endColor.w) / fadeTime);

        int nFade = mEmitting ? ch.count - 1 : ch.count;
        std::uint16_t e = ch.tail;
        for (int k = 0; k < nFade; ++k, e = nextRing(e, maxEl))
        {
            Element& el = mElements[base + e];
            el.width -= deltaWidth * dt;
            if (el.width < 0.f) el.width = 0.f;
            el.color.x -= deltaColor.x * dt;
            el.color.y -= deltaColor.y * dt;
            el.color.z -= deltaColor.z * dt;
            el.color.w -= deltaColor.w * dt;
            if (el.color.x < 0.f) el.color.x = 0.f;
            if (el.color.y < 0.f) el.color.y = 0.f;
            if (el.color.z < 0.f) el.color.z = 0.f;
            if (el.color.w < 0.f) el.color.w = 0.f;
        }

        while (ch.count > 1)
        {
            const Element& t = mElements[base + ch.tail];
            bool invisible = t.width <= 0.f || (t.color.x <= 0.f && t.color.y <= 0.f &&
                                                       t.color.z <= 0.f && t.color.w <= 0.f);
            if (!invisible) break;
            ch.tail = nextRing(ch.tail, maxEl);
            ch.count--;
        }
    }
}

void RibbonTrail3D::buildGeometry()
{
    mGeometry.positions.clear();
    mGeometry.normals.clear();
    mGeometry.colors.clear();
    mGeometry.uvs.clear();
    mGeometry.indices.clear();

    const int maxEdges = (mMaxElements - 1) * mSmoothing + 1;
    const int maxVerts = kMaxChains * maxEdges * 2;
    const int maxIndices = kMaxChains * (maxEdges - 1) * 6;

    Math::Vec3 camPos(0.0f, 0.0f, 0.0f);
    Math::Vec3 camUp(0.0f, 1.0f, 0.0f);
    {
        Viewport* viewport = Viewport::findViewport(this);
        Camera3D* camera = viewport ? viewport->getCamera3D() : nullptr;
        if (camera)
        {
            const Math::Mat4 cameraTransform = camera->getGlobalTransform();
            camPos = cameraTransform.GetTranslation();
            camUp = Math::Vec3(cameraTransform.col1).Normalized();
        }
    }

    int vi = 0;
    int ii = 0;

    for (int ci = 0; ci < mChainCount; ++ci)
    {
        Chain& ch = mChains[ci];
        int n = ch.count;
        if (!ch.active || n < 2) continue;

        int base = ci * mMaxElements;
        int maxEl = mMaxElements;
        const bool bladeChain = ch.blade;
        const int chainStartVi = vi;

        if (bladeChain)
        {
            mSmoothHilt.resize(static_cast<std::size_t>(n));
            mSmoothTip.resize(static_cast<std::size_t>(n));
            mSmoothCol.resize(static_cast<std::size_t>(n));
            mSmoothW.resize(static_cast<std::size_t>(n));
            for (int i = 0; i < n; ++i)
            {
                const Element& e =
                        mElements[base + static_cast<std::uint16_t>((ch.tail + i) % maxEl)];
                mSmoothHilt[i] = e.position;
                mSmoothTip[i] = e.tip;
                mSmoothCol[i] = e.color;
                mSmoothW[i] = e.width;
            }

            const int edges = (n - 1) * mSmoothing + 1;
            for (int eIdx = 0; eIdx < edges && vi + 2 <= maxVerts; ++eIdx)
            {
                int seg = eIdx / mSmoothing;
                float f = static_cast<float>(eIdx % mSmoothing) / static_cast<float>(mSmoothing);
                if (seg >= n - 1)
                {
                    seg = n - 2;
                    f = 1.f;
                }

                const int i0 = seg > 0 ? seg - 1 : 0;
                const int i3 = seg + 2 < n ? seg + 2 : n - 1;
                Math::Vec3 hilt = catmullRom(mSmoothHilt[i0], mSmoothHilt[seg],
                        mSmoothHilt[seg + 1], mSmoothHilt[i3], f);
                Math::Vec3 tip = catmullRom(mSmoothTip[i0], mSmoothTip[seg], mSmoothTip[seg + 1],
                        mSmoothTip[i3], f);

                const Math::Vec4& c0 = mSmoothCol[seg];
                const Math::Vec4& c1 = mSmoothCol[seg + 1];
                Math::Vec4 col(c0.x + (c1.x - c0.x) * f, c0.y + (c1.y - c0.y) * f,
                        c0.z + (c1.z - c0.z) * f, c0.w + (c1.w - c0.w) * f);
                float w = mSmoothW[seg] + (mSmoothW[seg + 1] - mSmoothW[seg]) * f;
                float span = w < 0.f ? 0.f : (w > 1.f ? 1.f : w);

                Math::Vec3 bladeVec(tip.x - hilt.x, tip.y - hilt.y, tip.z - hilt.z);
                Math::Vec3 tipPos(hilt.x + bladeVec.x * span, hilt.y + bladeVec.y * span,
                        hilt.z + bladeVec.z * span);

                Math::Vec3 nrm(0.f, 1.f, 0.f);
                {
                    Math::Vec3 sweep(mSmoothHilt[seg + 1].x - mSmoothHilt[seg].x,
                            mSmoothHilt[seg + 1].y - mSmoothHilt[seg].y,
                            mSmoothHilt[seg + 1].z - mSmoothHilt[seg].z);
                    Math::Vec3 c = Math::Vec3::Cross(bladeVec, sweep);
                    float cl = c.x * c.x + c.y * c.y + c.z * c.z;
                    if (cl > 1e-12f)
                    {
                        cl = 1.f / sqrtf(cl);
                        nrm = Math::Vec3(c.x * cl, c.y * cl, c.z * cl);
                    }
                }

                float t = static_cast<float>(eIdx) / static_cast<float>(edges - 1);

                pushVertex(mGeometry, hilt, nrm, col, 0.f, t);
                ++vi;
                pushVertex(mGeometry, tipPos, nrm, col, 1.f, t);
                ++vi;
            }
        }
        else
        {
            mSmoothHilt.resize(static_cast<std::size_t>(n));
            mSmoothCol.resize(static_cast<std::size_t>(n));
            mSmoothW.resize(static_cast<std::size_t>(n));
            for (int i = 0; i < n; ++i)
            {
                const Element& e =
                        mElements[base + static_cast<std::uint16_t>((ch.tail + i) % maxEl)];
                mSmoothHilt[i] = e.position;
                mSmoothCol[i] = e.color;
                mSmoothW[i] = e.width;
            }

            const int edges = (n - 1) * mSmoothing + 1;
            mEdgePos.resize(static_cast<std::size_t>(edges));
            mEdgeCol.resize(static_cast<std::size_t>(edges));
            mEdgeW.resize(static_cast<std::size_t>(edges));
            for (int eIdx = 0; eIdx < edges; ++eIdx)
            {
                int seg = eIdx / mSmoothing;
                float f = static_cast<float>(eIdx % mSmoothing) / static_cast<float>(mSmoothing);
                if (seg >= n - 1)
                {
                    seg = n - 2;
                    f = 1.f;
                }

                const int i0 = seg > 0 ? seg - 1 : 0;
                const int i3 = seg + 2 < n ? seg + 2 : n - 1;
                mEdgePos[eIdx] = catmullRom(mSmoothHilt[i0], mSmoothHilt[seg], mSmoothHilt[seg + 1],
                        mSmoothHilt[i3], f);

                const Math::Vec4& c0 = mSmoothCol[seg];
                const Math::Vec4& c1 = mSmoothCol[seg + 1];
                mEdgeCol[eIdx] = Math::Vec4(c0.x + (c1.x - c0.x) * f, c0.y + (c1.y - c0.y) * f,
                        c0.z + (c1.z - c0.z) * f, c0.w + (c1.w - c0.w) * f);
                mEdgeW[eIdx] = mSmoothW[seg] + (mSmoothW[seg + 1] - mSmoothW[seg]) * f;
            }

            for (int i = 0; i < edges && vi + 2 <= maxVerts; ++i)
            {
                const Math::Vec3& pos = mEdgePos[i];
                float t = static_cast<float>(i) / static_cast<float>(edges - 1);

                Math::Vec4 col = mEdgeCol[i];
                float w = mEdgeW[i];

                Math::Vec3 toCam(camPos.x - pos.x, camPos.y - pos.y, camPos.z - pos.z);
                float dlen = toCam.x * toCam.x + toCam.y * toCam.y + toCam.z * toCam.z;
                Math::Vec3 viewDir;
                if (dlen < 1e-8f) viewDir = Math::Vec3(0, 0, 1);
                else
                {
                    float inv = 1.f / sqrtf(dlen);
                    viewDir = Math::Vec3(toCam.x * inv, toCam.y * inv, toCam.z * inv);
                }

                Math::Vec3 right;
                Math::Vec3 up;
                if (i < edges - 1)
                {
                    const Math::Vec3& next = mEdgePos[i + 1];
                    float sx = next.x - pos.x;
                    float sy = next.y - pos.y;
                    float sz = next.z - pos.z;
                    float sl = sx * sx + sy * sy + sz * sz;
                    if (sl > 1e-12f)
                    {
                        sl = 1.f / sqrtf(sl);
                        Math::Vec3 seg(sx * sl, sy * sl, sz * sl);
                        right = Math::Vec3::Cross(viewDir, seg).Normalized();
                        up = Math::Vec3::Cross(right, viewDir).Normalized();
                    }
                    else
                    {
                        right = Math::Vec3::Cross(viewDir, camUp).Normalized();
                        up = camUp;
                    }
                }
                else if (i > 0)
                {
                    const Math::Vec3& prev = mEdgePos[i - 1];
                    float sx = pos.x - prev.x;
                    float sy = pos.y - prev.y;
                    float sz = pos.z - prev.z;
                    float sl = sx * sx + sy * sy + sz * sz;
                    if (sl > 1e-12f)
                    {
                        sl = 1.f / sqrtf(sl);
                        Math::Vec3 seg(sx * sl, sy * sl, sz * sl);
                        right = Math::Vec3::Cross(viewDir, seg).Normalized();
                        up = Math::Vec3::Cross(right, viewDir).Normalized();
                    }
                    else
                    {
                        right = Math::Vec3::Cross(viewDir, camUp).Normalized();
                        up = camUp;
                    }
                }
                else
                {
                    right = Math::Vec3::Cross(viewDir, camUp).Normalized();
                    up = camUp;
                }

                pushVertex(mGeometry,
                        Math::Vec3(pos.x - right.x * w, pos.y - right.y * w, pos.z - right.z * w),
                        up, col, 0.f, t);
                ++vi;
                pushVertex(mGeometry,
                        Math::Vec3(pos.x + right.x * w, pos.y + right.y * w, pos.z + right.z * w),
                        up, col, 1.f, t);
                ++vi;
            }
        }

        int chainVerts = vi - chainStartVi;
        for (int s = 0; s + 3 < chainVerts && ii + 6 <= maxIndices; s += 2)
        {
            std::uint32_t p0 = static_cast<std::uint32_t>(chainStartVi + s);
            std::uint32_t p1 = p0 + 1;
            std::uint32_t c0 = p0 + 2;
            std::uint32_t c1 = p0 + 3;
            mGeometry.indices.push_back(p0);
            mGeometry.indices.push_back(p1);
            mGeometry.indices.push_back(c0);
            mGeometry.indices.push_back(c1);
            mGeometry.indices.push_back(c0);
            mGeometry.indices.push_back(p1);
            ii += 6;
        }

        if (vi + 2 > maxVerts) break;
    }

    if (!mGeometry.positions.empty())
    {
        const Math::Mat4 inverse = getGlobalTransform().InverseAffine();
        for (std::size_t i = 0; i < mGeometry.positions.size(); ++i)
        {
            mGeometry.positions[i] = inverse.TransformPoint(mGeometry.positions[i]);
            mGeometry.normals[i] = inverse.TransformVector(mGeometry.normals[i]).Normalized();
        }
    }
}

void RibbonTrail3D::uploadGeometry()
{
    MeshStore* store = MeshStore::singleton();
    if (!store) return;

    if (!store->valid(mMesh))
    {
        mMesh = store->create(kMeshName);
        mSurfaceReady = false;
        setBase(mMesh);
    }

    if (!mSurfaceReady)
    {
        const int maxEdges = (mMaxElements - 1) * mSmoothing + 1;
        SurfaceLayout layout;
        layout.attributeMask = kVertexPosition | kVertexNormal | kVertexColor | kVertexUv;
        layout.vertexCount = static_cast<std::uint32_t>(kMaxChains * maxEdges * 2);
        layout.indexCount = static_cast<std::uint32_t>(kMaxChains * (maxEdges - 1) * 6);
        mSurfaceReady = store->addDynamicSurface(mMesh, layout, kMeshName) >= 0;
        mUploadedEmpty = true;
    }

    const bool empty = mGeometry.positions.empty();
    if (mSurfaceReady && !(empty && mUploadedEmpty))
    {
        store->updateSurface(mMesh, 0, mGeometry);
        mUploadedEmpty = empty;
    }
}

void RibbonTrail3D::invalidateSurface()
{
    MeshStore* store = MeshStore::singleton();
    if (store && store->valid(mMesh)) store->clearSurfaces(mMesh);
    mSurfaceReady = false;
}

void RibbonTrail3D::applyMaterial()
{
    RenderingServer* server = RenderingServer::singleton();
    if (server) server->instanceSetMaterialOverride(getInstance(), mMaterial);
}

void RibbonTrail3D::connectPreDraw()
{
    RenderingServer* server = RenderingServer::singleton();
    if (!server) return;
    server->framePreDrawConnect(this,
            [this]()
            {
                SceneTree* tree = getTree();
                update(tree ? tree->getProcessDeltaTime() : 0.0);
            });
}

void RibbonTrail3D::disconnectPreDraw()
{
    RenderingServer* server = RenderingServer::singleton();
    if (server) server->framePreDrawDisconnect(this);
}

void RibbonTrail3D::bindProperties()
{
    using DB = NodeClassDB;
    using P = RibbonTrail3D;
    resourceprop::add<P, MaterialId>(PropertyInfo::resource("material", "Material"),
            &P::setMaterial, &P::getAssignedMaterial);
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "emitting"), &P::setEmitting,
            &P::isEmitting);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "trail_length", 0.01, 1000.0, 0.01, true),
            &P::setTrailLength, &P::getTrailLength);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Int, "max_elements", 2.0, 64.0, 1.0),
            &P::setMaxElements, &P::getMaxElements);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Int, "smoothing", 1.0, 8.0, 1.0),
            &P::setSmoothing, &P::getSmoothing);
}

namespace
{

constexpr std::int64_t kRibbonTrailDataVersion = 1;

}

bool RibbonTrail3D::saveData(ct::Json& data, SceneResult& result, const ct::String& path) const
{
    (void) result;
    (void) path;
    if (mChainCount == 0) return false;

    data["version"] = ct::Json(static_cast<long long>(kRibbonTrailDataVersion));
    ct::Json chains = ct::Json::array();
    for (int i = 0; i < mChainCount; ++i)
    {
        const Chain& chain = mChains[i];
        ct::Json entry = ct::Json::object();
        entry["blade"] = ct::Json(chain.blade);
        entry["emitter"] = ct::Json(chain.emitterPath);
        if (chain.blade) entry["tip"] = ct::Json(chain.tipPath);
        entry["start_color"] = scenejson::fromVec4(chain.startColor);
        entry["end_color"] = scenejson::fromVec4(chain.endColor);
        entry["start_width"] = scenejson::fromFloat(chain.startWidth);
        entry["end_width"] = scenejson::fromFloat(chain.endWidth);
        entry["fade_time"] = scenejson::fromFloat(chain.fadeTime);
        chains.push_back(entry);
    }
    data["chains"] = chains;
    return true;
}

bool RibbonTrail3D::loadData(const ct::Json& data, SceneResult& result, const ct::String& path)
{
    if (!scenejson::checkVersion(data, kRibbonTrailDataVersion, result, path)) return false;

    clearChains();
    const ct::Json* chains = scenejson::arrayField(data, "chains", result, path);
    if (!chains) return true;
    for (std::size_t i = 0; i < chains->size(); ++i)
    {
        const ct::Json& entry = chains->at(i);
        const ct::String entryPath = scenejson::element(path, "chains", i);
        if (!entry.is_object())
        {
            result.addError(entryPath, "expected an object");
            continue;
        }
        const ct::String emitter = scenejson::getString(entry, "emitter", ct::String());
        const ct::String tip = scenejson::getString(entry, "tip", ct::String());
        Math::Vec4 startColor(1.0f, 1.0f, 1.0f, 1.0f);
        Math::Vec4 endColor(1.0f, 1.0f, 1.0f, 0.0f);
        scenejson::readVec4Field(entry, "start_color", startColor, result, entryPath);
        scenejson::readVec4Field(entry, "end_color", endColor, result, entryPath);
        const bool blade = scenejson::getBool(entry, "blade", false);
        const float startWidth = scenejson::getFloat(entry, "start_width", blade ? 1.0f : 0.7f);
        const float endWidth = scenejson::getFloat(entry, "end_width", blade ? 0.0f : 0.05f);
        const float fadeTime = scenejson::getFloat(entry, "fade_time", 0.3f);
        const int index =
                blade ? addBladeChain(emitter, tip, startColor, endColor, fadeTime, startWidth,
                                endWidth)
                      : addChain(emitter, startColor, endColor, startWidth, endWidth, fadeTime);
        if (index < 0)
            result.addError(entryPath, "the chain needs an emitter path (and a tip path for a "
                                       "blade) and there is room for four chains");
    }
    return true;
}

} // namespace prisma3d
