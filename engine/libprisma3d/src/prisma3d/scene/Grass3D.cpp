#include "prisma3d/scene/Grass3D.h"

#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/SceneJson.h"
#include "prisma3d/scene/SceneResources.h"

#include "NodeFactory.h"
#include "prisma3d/main/Main.h"
#include "prisma3d/scene/Camera3D.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/Terrain3D.h"
#include "prisma3d/scene/Viewport.h"

#include "platform.h"

#include <ct/json.hpp>

#include <float.h>
#include <math.h>
#include <string.h>

namespace prisma3d
{

namespace
{

const float kPi = 3.14159265358979f;
const std::size_t kMaxPooledCells = 48;

float clampUnit(float value)
{
    return value < 0.0f ? 0.0f : (value > 1.0f ? 1.0f : value);
}

float smoothStep(float edge0, float edge1, float x)
{
    const float t = clampUnit((x - edge0) / (edge1 - edge0));
    return t * t * (3.0f - 2.0f * t);
}

float fract(float x)
{
    return x - floorf(x);
}

std::uint32_t mixBits(std::uint32_t h)
{
    h ^= h >> 16;
    h *= 0x7feb352du;
    h ^= h >> 15;
    h *= 0x846ca68bu;
    h ^= h >> 16;
    return h;
}

void buildTuftSurface(SurfaceData& out)
{
    for (int quad = 0; quad < 3; ++quad)
    {
        const float angle = static_cast<float>(quad) * (kPi / 3.0f);
        const float s = sinf(angle);
        const float c = cosf(angle);
        const Math::Vec3 normal(-c, 0.0f, s);
        const std::uint32_t first = static_cast<std::uint32_t>(out.positions.size());
        for (int corner = 0; corner < 4; ++corner)
        {
            const float cx = (corner & 1) ? 1.0f : -1.0f;
            const float cy = static_cast<float>(corner >> 1);
            out.positions.push_back(Math::Vec3(s * cx, cy, c * cx));
            out.normals.push_back(normal);
            out.colors.push_back(Math::Vec4(1.0f, 1.0f, 1.0f, 1.0f));
            out.uvs.push_back(Math::Vec2(cx * 0.5f + 0.5f, 1.0f - cy));
        }
        out.indices.push_back(first + 0);
        out.indices.push_back(first + 1);
        out.indices.push_back(first + 2);
        out.indices.push_back(first + 2);
        out.indices.push_back(first + 1);
        out.indices.push_back(first + 3);
    }
}

} // namespace

Grass3D::Grass3D()
        : Node3D(),
          mTypes(),
          mInfluencers(),
          mHeightFunction(),
          mDensityFunction(),
          mMask(nullptr),
          mMaskWidth(256),
          mMaskDepth(256),
          mMesh(),
          mCellSize(8.0f),
          mDensity(0.35f),
          mFalloffStart(0.35f),
          mDrawDistance(60.0f),
          mCastShadows(false),
          mAlphaAntialiasing(MaterialAlphaAntialiasing::Off),
          mAlphaAntialiasingEdge(0.3f),
          mReleaseMargin(8.0f),
          mSeed(12345u),
          mHasExtent(false),
          mExtent{ 0.0f, 0.0f, 0.0f, 0.0f },
          mCameraBend(0.55f),
          mWind(1.0f),
          mWindSpeed(1.0f),
          mWindDirection(0.82f, 0.0f, 0.57f),
          mStiffness(12.0f),
          mDrag(0.12f),
          mFrustumCulling(true),
          mDirty(true),
          mTime(0.0f),
          mWritten(0),
          mMoving(0),
          mFilled(0),
          mUpdateMs(0.0f),
          mConnected(false),
          mTerrain(nullptr),
          mTerrainPath(),
          mPlacement(),
          mReseated(0),
          mReseatMs(0.0f)
{
    mWindDirection = mWindDirection.Normalized();
}

Grass3D::~Grass3D()
{
    disconnectPreDraw();
    if (mTerrain) mTerrain->removeObserver(this);
    RenderingServer* server = RenderingServer::singleton();
    for (std::size_t i = 0; i < mTypes.size(); ++i)
    {
        Type* type = mTypes[i];
        if (server && type->material.valid()) server->materials().destroy(type->material);
        for (std::size_t c = 0; c < type->cells.size(); ++c)
        {
            type->cells[c]->node = nullptr;
            delete type->cells[c];
        }
        type->cells.clear();
        type->active.clear();
        type->pool.clear();
        delete type;
    }
    mTypes.clear();
    releaseResources();
    delete mMask;
    mMask = nullptr;
}

std::uint64_t Grass3D::cellKey(int cellX, int cellZ)
{
    return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(cellX)) << 32) |
           static_cast<std::uint64_t>(static_cast<std::uint32_t>(cellZ));
}

void Grass3D::cellOf(float x, float z, float cellSize, int& cellX, int& cellZ)
{
    cellX = static_cast<int>(floorf(x / cellSize));
    cellZ = static_cast<int>(floorf(z / cellSize));
}

float Grass3D::cellDistance(int cellX, int cellZ, float cellSize, float x, float z)
{
    const float minX = static_cast<float>(cellX) * cellSize;
    const float minZ = static_cast<float>(cellZ) * cellSize;
    float dx = minX - x;
    if (x - (minX + cellSize) > dx) dx = x - (minX + cellSize);
    if (dx < 0.0f) dx = 0.0f;
    float dz = minZ - z;
    if (z - (minZ + cellSize) > dz) dz = z - (minZ + cellSize);
    if (dz < 0.0f) dz = 0.0f;
    return sqrtf(dx * dx + dz * dz);
}

std::uint32_t Grass3D::cellSeed(std::uint32_t seed, int cellX, int cellZ)
{
    std::uint32_t h = mixBits(seed ^ (static_cast<std::uint32_t>(cellX) * 0x9e3779b1u));
    h = mixBits(h ^ (static_cast<std::uint32_t>(cellZ) * 0x85ebca6bu));
    return h;
}

std::uint32_t Grass3D::typeSeed(std::uint32_t seed, int type)
{
    if (type <= 0) return seed;
    return mixBits(seed ^ (static_cast<std::uint32_t>(type) * 0x632be5abu));
}

float Grass3D::nextRandom(std::uint32_t& state)
{
    state = state * 1664525u + 1013904223u;
    return static_cast<float>(state >> 8) / 16777216.0f;
}

int Grass3D::candidateCount(float density, float cellSize)
{
    const float count = floorf(density * cellSize * cellSize + 0.5f);
    return count > 0.0f ? static_cast<int>(count) : 0;
}

float Grass3D::distanceKeep(float distance, float drawDistance, float startFraction)
{
    const float start = drawDistance * startFraction;
    if (distance <= start) return 1.0f;
    if (distance >= drawDistance) return 0.0f;
    return 1.0f - (distance - start) / (drawDistance - start);
}

float Grass3D::drawFade(float distance, float drawDistance)
{
    return 1.0f - smoothStep(drawDistance * 0.7f, drawDistance, distance);
}

float Grass3D::brushFalloff(float distance, float radius)
{
    return 1.0f - smoothStep(0.0f, radius, distance);
}

float Grass3D::windWave(const Math::Vec3& base, float time, float windSpeed)
{
    const float t = time * windSpeed;
    return sinf(t * 1.6f + base.x * 0.16f + base.z * 0.11f) +
           0.5f * sinf(t * 2.7f + base.x * 0.29f);
}

Math::Vec3 Grass3D::tintOf(const Math::Vec3& root, const Math::Vec3& colourA,
        const Math::Vec3& colourB)
{
    const float sx = root.x + root.y * 3.7f + 55.3f;
    const float sz = root.z + root.y * 3.7f + 55.3f;
    const float hash = fract(sinf(sx * 127.1f + sz * 311.7f) * 43758.5453f);
    return Math::Vec3::Lerp(colourA, colourB, hash);
}

void Grass3D::simulate(const Math::Vec3& base, const Math::Vec3& up, float length, GrassTip& tip,
        const GrassSimParams& params, const GrassInfluencer* influencers, int influencerCount)
{
    const Math::Vec3 rest = base + up * length;
    Math::Vec3 current = tip.current;
    Math::Vec3 previous = tip.previous;
    if (!tip.initialised)
    {
        current = rest;
        previous = rest;
    }

    const Math::Vec3 inertia = (current - previous) * (1.0f - params.drag);
    Math::Vec3 force = (rest - current) * params.stiffness;
    const Math::Vec3 direction = params.windDirection.NormalizedSafe();
    const float wave = windWave(base, params.time, params.windSpeed);
    force += direction * (wave * params.wind);

    Math::Vec3 next = current + inertia + force * params.deltaTime;

    for (int i = 0; i < influencerCount && i < kMaxInfluencers; ++i)
    {
        const Math::Vec3 d = next - influencers[i].centre;
        const float distance = d.Length();
        const float radius = influencers[i].radius;
        if (distance < radius && distance > 1e-4f)
        {
            const Math::Vec3 push = d / distance;
            next += push * (influencers[i].force * params.deltaTime * (1.0f - distance / radius));
        }
    }

    const Math::Vec3 toTip = next - base;
    const float len = toTip.Length();
    if (len > 1e-5f) next = base + toTip * (length / len);
    else
        next = rest;

    const float below = (next - base).Dot(up);
    if (below < 0.0f)
    {
        next -= up * below;
        const Math::Vec3 p2 = next - base;
        const float l2 = p2.Length();
        if (l2 > 1e-5f) next = base + p2 * (length / l2);
    }

    tip.previous = current;
    tip.current = next;
    tip.initialised = true;
}

Grass3D::Type* Grass3D::typeAt(int type) const
{
    if (type < 0 || static_cast<std::size_t>(type) >= mTypes.size()) return nullptr;
    return mTypes[static_cast<std::size_t>(type)];
}

Grass3D::Type* Grass3D::defaultType()
{
    if (mTypes.empty()) addType(GrassTypeDesc());
    return typeAt(0);
}

int Grass3D::addType(const GrassTypeDesc& desc)
{
    if (static_cast<int>(mTypes.size()) >= kMaxTypes) return -1;
    Type* type = new Type();
    type->desc = desc;
    if (type->desc.height < 0.0f) type->desc.height = 0.0f;
    if (type->desc.width < 0.0f) type->desc.width = 0.0f;
    type->desc.alphaCut = clampUnit(type->desc.alphaCut);
    if (type->desc.densityScale < 0.0f) type->desc.densityScale = 0.0f;
    if (type->desc.scaleMax < type->desc.scaleMin) type->desc.scaleMax = type->desc.scaleMin;
    mTypes.push_back(type);
    markDirty();
    return static_cast<int>(mTypes.size()) - 1;
}

void Grass3D::destroyType(Type* type)
{
    RenderingServer* server = RenderingServer::singleton();
    for (std::size_t c = 0; c < type->cells.size(); ++c)
    {
        NodeFactory::destroy(type->cells[c]->node);
        delete type->cells[c];
    }
    type->cells.clear();
    type->active.clear();
    type->pool.clear();
    if (server && type->material.valid()) server->materials().destroy(type->material);
    delete type;
}

void Grass3D::removeType(int type)
{
    Type* removed = typeAt(type);
    if (!removed) return;
    destroyType(removed);
    mTypes.erase(mTypes.begin() + type);
    if (mMask && mMask->is_valid())
    {
        const int count = mMaskWidth * mMaskDepth;
        for (int channel = type; channel < kMaxTypes; ++channel)
        {
            for (int i = 0; i < count; ++i)
            {
                unsigned char value = 0;
                if (channel + 1 < kMaxTypes) value = mMask->pixels[i * 4 + channel + 1];
                mMask->pixels[i * 4 + channel] = value;
            }
        }
    }
    markDirty();
}

int Grass3D::getTypeCount() const
{
    return static_cast<int>(mTypes.size());
}

bool Grass3D::getType(int type, GrassTypeDesc& out) const
{
    const Type* found = typeAt(type);
    if (!found) return false;
    out = found->desc;
    return true;
}

bool Grass3D::setType(int type, const GrassTypeDesc& desc)
{
    Type* found = typeAt(type);
    if (!found) return false;
    found->desc = desc;
    if (found->desc.height < 0.0f) found->desc.height = 0.0f;
    if (found->desc.width < 0.0f) found->desc.width = 0.0f;
    found->desc.alphaCut = clampUnit(found->desc.alphaCut);
    if (found->desc.densityScale < 0.0f) found->desc.densityScale = 0.0f;
    if (found->desc.scaleMax < found->desc.scaleMin) found->desc.scaleMax = found->desc.scaleMin;
    applyMaterial(*found);
    markDirty();
    return true;
}

bool Grass3D::addTypeRegion(int type, std::uint32_t x, std::uint32_t y, std::uint32_t width,
        std::uint32_t height, float size, float weight)
{
    Type* found = typeAt(type);
    if (!found) return false;
    const TextureStore* textures = TextureStore::singleton();
    const TextureInfo* info = textures ? textures->get(found->desc.texture) : nullptr;
    if (!info || info->width == 0 || info->height == 0 || width == 0 || height == 0) return false;
    if (x + width > info->width || y + height > info->height) return false;
    GrassRegion region;
    const float atlasWidth = static_cast<float>(info->width);
    const float atlasHeight = static_cast<float>(info->height);
    region.texMulAdd = Math::Vec4(static_cast<float>(width) / atlasWidth,
            static_cast<float>(height) / atlasHeight, static_cast<float>(x) / atlasWidth,
            static_cast<float>(y) / atlasHeight);
    region.size = size;
    region.aspect = static_cast<float>(width) / static_cast<float>(height);
    region.weight = weight > 0.0f ? weight : 0.0f;
    found->regions.push_back(region);
    markDirty();
    return true;
}

void Grass3D::clearTypeRegions(int type)
{
    Type* found = typeAt(type);
    if (!found) return;
    found->regions.clear();
    markDirty();
}

int Grass3D::getTypeRegionCount(int type) const
{
    const Type* found = typeAt(type);
    return found ? static_cast<int>(found->regions.size()) : 0;
}

bool Grass3D::getTypeRegion(int type, int index, GrassRegion& out) const
{
    const Type* found = typeAt(type);
    if (!found || index < 0 || static_cast<std::size_t>(index) >= found->regions.size())
        return false;
    out = found->regions[static_cast<std::size_t>(index)];
    return true;
}

MaterialId Grass3D::getMaterial(int type) const
{
    const Type* found = typeAt(type);
    return found ? found->material : MaterialId();
}

void Grass3D::setTexture(TextureId texture)
{
    Type* type = defaultType();
    type->desc.texture = texture;
    applyMaterial(*type);
    markDirty();
}

TextureId Grass3D::getTexture() const
{
    const Type* type = typeAt(0);
    return type ? type->desc.texture : TextureId();
}

void Grass3D::setTexturePath(const ct::String& path)
{
    if (path.empty())
    {
        if (getTexture().valid()) setTexture(TextureId());
        return;
    }
    if (sceneres::pathOf(getTexture()) == sceneres::normalizePath(path)) return;
    sceneres::clearError();
    const TextureId texture = sceneres::loadTexture(path, true);
    if (!texture.valid())
    {
        log_error("grass: %s", sceneres::takeError().c_str());
        return;
    }
    setTexture(texture);
}

ct::String Grass3D::getTexturePath() const
{
    return sceneres::pathOf(getTexture());
}

void Grass3D::setTypeCount(int count)
{
    if (count < 0) count = 0;
    if (count > kMaxTypes) count = kMaxTypes;
    bool changed = false;
    while (getTypeCount() > count)
    {
        removeType(getTypeCount() - 1);
        changed = true;
    }
    while (getTypeCount() < count)
    {
        if (addType(GrassTypeDesc()) < 0) break;
        changed = true;
    }
    if (changed) notifyPropertyListChanged();
}

GrassTypeDesc Grass3D::typeOrDefault(int type) const
{
    GrassTypeDesc desc;
    getType(type, desc);
    return desc;
}

void Grass3D::setTypeDesc(int type, const GrassTypeDesc& desc)
{
    if (type < 0 || type >= kMaxTypes) return;
    bool added = false;
    while (getTypeCount() <= type)
    {
        if (addType(GrassTypeDesc()) < 0) return;
        added = true;
    }
    setType(type, desc);
    if (added) notifyPropertyListChanged();
}

void Grass3D::setTypeTexturePath(int type, const ct::String& path)
{
    if (type < 0 || type >= kMaxTypes) return;
    GrassTypeDesc desc = typeOrDefault(type);
    if (path.empty())
    {
        if (!desc.texture.valid() && type >= getTypeCount()) return;
        desc.texture = TextureId();
        setTypeDesc(type, desc);
        return;
    }
    if (sceneres::pathOf(desc.texture) == sceneres::normalizePath(path)) return;
    sceneres::clearError();
    const TextureId texture = sceneres::loadTexture(path, true);
    if (!texture.valid())
    {
        log_error("grass: %s", sceneres::takeError().c_str());
        return;
    }
    desc.texture = texture;
    setTypeDesc(type, desc);
}

ct::String Grass3D::getTypeTexturePath(int type) const
{
    return sceneres::pathOf(typeOrDefault(type).texture);
}

void Grass3D::setColourA(const Math::Vec3& colour)
{
    defaultType()->desc.colourA = colour;
    markDirty();
}

Math::Vec3 Grass3D::getColourA() const
{
    const Type* type = typeAt(0);
    return type ? type->desc.colourA : GrassTypeDesc().colourA;
}

void Grass3D::setColourB(const Math::Vec3& colour)
{
    defaultType()->desc.colourB = colour;
    markDirty();
}

Math::Vec3 Grass3D::getColourB() const
{
    const Type* type = typeAt(0);
    return type ? type->desc.colourB : GrassTypeDesc().colourB;
}

void Grass3D::setScaleMin(float scale)
{
    Type* type = defaultType();
    type->desc.scaleMin = scale > 0.0f ? scale : 0.0f;
    if (type->desc.scaleMax < type->desc.scaleMin) type->desc.scaleMax = type->desc.scaleMin;
    markDirty();
}

float Grass3D::getScaleMin() const
{
    const Type* type = typeAt(0);
    return type ? type->desc.scaleMin : GrassTypeDesc().scaleMin;
}

void Grass3D::setScaleMax(float scale)
{
    Type* type = defaultType();
    type->desc.scaleMax = scale > type->desc.scaleMin ? scale : type->desc.scaleMin;
    markDirty();
}

float Grass3D::getScaleMax() const
{
    const Type* type = typeAt(0);
    return type ? type->desc.scaleMax : GrassTypeDesc().scaleMax;
}

void Grass3D::setDensityScale(float scale)
{
    defaultType()->desc.densityScale = scale > 0.0f ? scale : 0.0f;
    markDirty();
}

float Grass3D::getDensityScale() const
{
    const Type* type = typeAt(0);
    return type ? type->desc.densityScale : GrassTypeDesc().densityScale;
}

bool Grass3D::addRegion(std::uint32_t x, std::uint32_t y, std::uint32_t width, std::uint32_t height,
        float size, float weight)
{
    defaultType();
    return addTypeRegion(0, x, y, width, height, size, weight);
}

void Grass3D::clearRegions()
{
    clearTypeRegions(0);
}

int Grass3D::getRegionCount() const
{
    return getTypeRegionCount(0);
}

bool Grass3D::getRegion(int index, GrassRegion& out) const
{
    return getTypeRegion(0, index, out);
}

void Grass3D::setHeight(float metres)
{
    defaultType()->desc.height = metres > 0.0f ? metres : 0.0f;
    markDirty();
}

float Grass3D::getHeight() const
{
    const Type* type = typeAt(0);
    return type ? type->desc.height : GrassTypeDesc().height;
}

void Grass3D::setWidth(float multiplier)
{
    defaultType()->desc.width = multiplier > 0.0f ? multiplier : 0.0f;
    markDirty();
}

float Grass3D::getWidth() const
{
    const Type* type = typeAt(0);
    return type ? type->desc.width : GrassTypeDesc().width;
}

void Grass3D::setAlphaCut(float cut)
{
    Type* type = defaultType();
    type->desc.alphaCut = clampUnit(cut);
    applyMaterial(*type);
}

float Grass3D::getAlphaCut() const
{
    const Type* type = typeAt(0);
    return type ? type->desc.alphaCut : GrassTypeDesc().alphaCut;
}

MaterialId Grass3D::getMaterial() const
{
    return getMaterial(0);
}

GrassRegion Grass3D::regionAt(const Type& type, std::uint32_t index) const
{
    if (!type.regions.empty()) return type.regions[index < type.regions.size() ? index : 0];
    GrassRegion region;
    const TextureStore* textures = TextureStore::singleton();
    const TextureInfo* info = textures ? textures->get(type.desc.texture) : nullptr;
    if (info && info->width > 0 && info->height > 0)
        region.aspect = static_cast<float>(info->width) / static_cast<float>(info->height);
    return region;
}

float Grass3D::totalWeight(const Type& type) const
{
    if (type.regions.empty()) return 1.0f;
    float total = 0.0f;
    for (std::size_t i = 0; i < type.regions.size(); ++i) total += type.regions[i].weight;
    return total;
}

void Grass3D::setCellSize(float size)
{
    mCellSize = size > 0.5f ? size : 0.5f;
    markDirty();
}

float Grass3D::getCellSize() const
{
    return mCellSize;
}

void Grass3D::setDensity(float tuftsPerSquareMetre)
{
    mDensity = tuftsPerSquareMetre > 0.0f ? tuftsPerSquareMetre : 0.0f;
    markDirty();
}

float Grass3D::getDensity() const
{
    return mDensity;
}

void Grass3D::setDensityFalloffStart(float fraction)
{
    mFalloffStart = clampUnit(fraction);
}

float Grass3D::getDensityFalloffStart() const
{
    return mFalloffStart;
}

void Grass3D::setDrawDistance(float metres)
{
    mDrawDistance = metres > 0.0f ? metres : 0.0f;
    for (std::size_t i = 0; i < mTypes.size(); ++i) applyMaterial(*mTypes[i]);
}

void Grass3D::setCastShadows(bool cast)
{
    mCastShadows = cast;
    for (std::size_t i = 0; i < mTypes.size(); ++i)
        for (std::size_t c = 0; c < mTypes[i]->cells.size(); ++c)
            if (mTypes[i]->cells[c]->node) mTypes[i]->cells[c]->node->setCastShadows(cast);
}

bool Grass3D::getCastShadows() const
{
    return mCastShadows;
}

void Grass3D::setAlphaAntialiasing(MaterialAlphaAntialiasing mode, float edge)
{
    mAlphaAntialiasing = mode;
    mAlphaAntialiasingEdge = edge;
    for (std::size_t i = 0; i < mTypes.size(); ++i) applyMaterial(*mTypes[i]);
}

MaterialAlphaAntialiasing Grass3D::getAlphaAntialiasing() const
{
    return mAlphaAntialiasing;
}

float Grass3D::getDrawDistance() const
{
    return mDrawDistance;
}

void Grass3D::setReleaseMargin(float metres)
{
    mReleaseMargin = metres > 0.0f ? metres : 0.0f;
}

float Grass3D::getReleaseMargin() const
{
    return mReleaseMargin;
}

void Grass3D::setSeed(std::uint32_t seed)
{
    mSeed = seed;
    markDirty();
}

std::uint32_t Grass3D::getSeed() const
{
    return mSeed;
}

void Grass3D::setExtent(float minX, float minZ, float maxX, float maxZ)
{
    mHasExtent = true;
    mExtent[0] = minX < maxX ? minX : maxX;
    mExtent[1] = minZ < maxZ ? minZ : maxZ;
    mExtent[2] = minX < maxX ? maxX : minX;
    mExtent[3] = minZ < maxZ ? maxZ : minZ;
    markDirty();
}

void Grass3D::clearExtent()
{
    mHasExtent = false;
    clearMask();
    markDirty();
}

bool Grass3D::hasExtent() const
{
    return mHasExtent;
}

void Grass3D::setExtentEnabled(bool enabled)
{
    if (enabled)
    {
        mHasExtent = true;
        markDirty();
    }
    else if (mHasExtent)
        clearExtent();
}

void Grass3D::setExtentMin(const Math::Vec2& minimum)
{
    mExtent[0] = minimum.x;
    mExtent[1] = minimum.y;
    markDirty();
}

Math::Vec2 Grass3D::getExtentMin() const
{
    return Math::Vec2(mExtent[0], mExtent[1]);
}

void Grass3D::setExtentMax(const Math::Vec2& maximum)
{
    mExtent[2] = maximum.x;
    mExtent[3] = maximum.y;
    markDirty();
}

Math::Vec2 Grass3D::getExtentMax() const
{
    return Math::Vec2(mExtent[2], mExtent[3]);
}

void Grass3D::setHeightFunction(const FieldFunction& function)
{
    mHeightFunction = function;
    markDirty();
}

void Grass3D::setDensityFunction(const FieldFunction& function)
{
    mDensityFunction = function;
    markDirty();
}

void Grass3D::setTerrain(Terrain3D* terrain)
{
    mTerrainPath = ct::String();
    if (terrain == mTerrain) return;
    if (mTerrain) mTerrain->removeObserver(this);
    mTerrain = terrain;
    if (mTerrain) mTerrain->addObserver(this);
    markDirty();
}

Terrain3D* Grass3D::getTerrain() const
{
    return mTerrain;
}

void Grass3D::setTerrainPath(const ct::String& path)
{
    mTerrainPath = path;
    resolveTerrain();
}

ct::String Grass3D::getTerrainPath() const
{
    if (mTerrain) return getPathTo(mTerrain);
    return mTerrainPath;
}

void Grass3D::resolveTerrain()
{
    Terrain3D* terrain = nullptr;
    if (!mTerrainPath.empty())
    {
        Node* node = getNode(mTerrainPath);
        terrain = node ? node->castTo<Terrain3D>() : nullptr;
    }
    if (terrain)
    {
        setTerrain(terrain);
        return;
    }
    const ct::String pending = mTerrainPath;
    setTerrain(nullptr);
    mTerrainPath = pending;
}

void Grass3D::setMaxSlope(float degrees)
{
    mPlacement.setMaxSlope(degrees);
    markDirty();
}

float Grass3D::getMaxSlope() const
{
    return mPlacement.getMaxSlope();
}

void Grass3D::addExclusion(const TerrainPath& path, float margin)
{
    mPlacement.addExclusion(path, margin);
    markDirty();
}

void Grass3D::clearExclusions()
{
    mPlacement.clearExclusions();
    markDirty();
}

int Grass3D::getReseatedCount() const
{
    return mReseated;
}

float Grass3D::getReseatMilliseconds() const
{
    return mReseatMs;
}

float Grass3D::groundHeight(float x, float z) const
{
    if (mTerrain && mTerrain->isLoaded())
    {
        const Math::Mat4 world = getGlobalTransform();
        const Math::Vec3 flat = world.TransformPoint(Math::Vec3(x, 0.0f, z));
        const Math::Vec3 ground(flat.x, mTerrain->getWorldHeightAt(flat), flat.z);
        return world.InverseAffine().TransformPoint(ground).y;
    }
    return mHeightFunction ? mHeightFunction(x, z) : 0.0f;
}

float Grass3D::clumpDensity(int type, float x, float z) const
{
    float density = getMaskDensity(type, x, z);
    if (mDensityFunction) density *= clampUnit(mDensityFunction(x, z));
    if (mTerrain && mTerrain->isLoaded())
    {
        const Math::Vec3 flat = getGlobalTransform().TransformPoint(Math::Vec3(x, 0.0f, z));
        const Math::Vec3 ground(flat.x, mTerrain->getWorldHeightAt(flat), flat.z);
        if (!mPlacement.accepts(*mTerrain, ground)) density = 0.0f;
    }
    return clampUnit(density);
}

void Grass3D::onTerrainDestroyed()
{
    mTerrain = nullptr;
    markDirty();
}

void Grass3D::onTerrainChanged(const Math::Box& worldBox)
{
    if (!mTerrain || !mTerrain->isLoaded()) return;
    const std::uint64_t started = Main::device() ? Main::device()->getTicksUsec() : 0;
    const Math::Mat4 inverse = getGlobalTransform().InverseAffine();
    float minX = FLT_MAX;
    float minZ = FLT_MAX;
    float maxX = -FLT_MAX;
    float maxZ = -FLT_MAX;
    for (int corner = 0; corner < 4; ++corner)
    {
        const Math::Vec3 local =
                inverse.TransformPoint(Math::Vec3((corner & 1) ? worldBox.max.x : worldBox.min.x,
                        worldBox.min.y, (corner & 2) ? worldBox.max.z : worldBox.min.z));
        minX = local.x < minX ? local.x : minX;
        maxX = local.x > maxX ? local.x : maxX;
        minZ = local.z < minZ ? local.z : minZ;
        maxZ = local.z > maxZ ? local.z : maxZ;
    }

    int firstX = 0;
    int firstZ = 0;
    int lastX = 0;
    int lastZ = 0;
    cellOf(minX, minZ, mCellSize, firstX, firstZ);
    cellOf(maxX, maxZ, mCellSize, lastX, lastZ);
    int reseated = 0;
    const float bend = mCameraBend > 0.0f ? mCameraBend : 0.0f;
    for (std::size_t t = 0; t < mTypes.size(); ++t)
    {
        Type& type = *mTypes[t];
        for (int cellZ = firstZ; cellZ <= lastZ; ++cellZ)
        {
            for (int cellX = firstX; cellX <= lastX; ++cellX)
            {
                Cell** found = type.active.find(cellKey(cellX, cellZ));
                if (!found) continue;
                Cell& cell = **found;
                bool moved = false;
                for (std::size_t k = 0; k < cell.clumps.size(); ++k)
                {
                    Clump& clump = cell.clumps[k];
                    const Math::Vec3 root = clump.info.root;
                    if (root.x < minX || root.x > maxX || root.z < minZ || root.z > maxZ) continue;
                    ++reseated;
                    clump.info.density = clumpDensity(static_cast<int>(t), root.x, root.z);
                    const float y = groundHeight(root.x, root.z);
                    if (y == root.y) continue;
                    clump.info.root.y = y;
                    clump.transform.col3.y = y;
                    moved = true;
                    if (clump.slot >= 0)
                    {
                        cell.node->setInstanceTransform(clump.slot, clump.transform);
                        ++mWritten;
                    }
                }
                if (!moved) continue;
                Math::Box bounds;
                bool any = false;
                for (std::size_t k = 0; k < cell.clumps.size(); ++k)
                {
                    const Clump& clump = cell.clumps[k];
                    if (!clump.info.inside) continue;
                    const float width = Math::Vec3(clump.transform.col0.x, clump.transform.col0.y,
                            clump.transform.col0.z)
                                                .Length();
                    const float height = Math::Vec3(clump.transform.col1.x, clump.transform.col1.y,
                            clump.transform.col1.z)
                                                 .Length();
                    const float reach = height * (1.0f + bend) + clump.length + width;
                    const Math::Vec3& r = clump.info.root;
                    bounds = bounds.Expanded(Math::Vec3(r.x - reach, r.y - reach, r.z - reach));
                    bounds = bounds.Expanded(Math::Vec3(r.x + reach, r.y + reach, r.z + reach));
                    any = true;
                }
                if (any)
                {
                    cell.boundsCentre = (bounds.min + bounds.max) * 0.5f;
                    cell.boundsRadius = (bounds.max - bounds.min).Length() * 0.5f;
                    cell.node->getMultiMesh()->setCustomAabb(bounds);
                }
            }
        }
    }
    const std::uint64_t finished = Main::device() ? Main::device()->getTicksUsec() : 0;
    mReseated = reseated;
    mReseatMs = static_cast<float>(finished - started) / 1000.0f;
}

void Grass3D::setCameraBend(float amount)
{
    mCameraBend = amount;
    for (std::size_t i = 0; i < mTypes.size(); ++i) applyMaterial(*mTypes[i]);
    markDirty();
}

float Grass3D::getCameraBend() const
{
    return mCameraBend;
}

MeshId Grass3D::getMesh() const
{
    return mMesh;
}

void Grass3D::setWind(float strength)
{
    mWind = strength;
    applyAllMaterials();
}

float Grass3D::getWind() const
{
    return mWind;
}

void Grass3D::setWindSpeed(float speed)
{
    mWindSpeed = speed;
    applyAllMaterials();
}

float Grass3D::getWindSpeed() const
{
    return mWindSpeed;
}

void Grass3D::setWindDirection(const Math::Vec3& direction)
{
    const Math::Vec3 normalised = direction.NormalizedSafe();
    if (normalised.LengthSquared() > 0.0f) mWindDirection = normalised;
    applyAllMaterials();
}

Math::Vec3 Grass3D::getWindDirection() const
{
    return mWindDirection;
}

void Grass3D::setStiffness(float stiffness)
{
    mStiffness = stiffness > 0.0f ? stiffness : 0.0f;
    applyAllMaterials();
}

float Grass3D::getStiffness() const
{
    return mStiffness;
}

void Grass3D::setDrag(float drag)
{
    mDrag = clampUnit(drag);
}

float Grass3D::getDrag() const
{
    return mDrag;
}

void Grass3D::clearInfluencers()
{
    mInfluencers.clear();
}

bool Grass3D::addInfluencer(const Math::Vec3& centre, float radius, float force)
{
    if (static_cast<int>(mInfluencers.size()) >= kMaxInfluencers || radius <= 0.0f) return false;
    GrassInfluencer influencer;
    influencer.centre = centre;
    influencer.radius = radius;
    influencer.force = force;
    mInfluencers.push_back(influencer);
    return true;
}

int Grass3D::getInfluencerCount() const
{
    return static_cast<int>(mInfluencers.size());
}

void Grass3D::applyAllMaterials()
{
    for (std::size_t i = 0; i < mTypes.size(); ++i) applyMaterial(*mTypes[i]);
}

void Grass3D::markDirty()
{
    mDirty = true;
}

void Grass3D::invalidate()
{
    mDirty = true;
}

void Grass3D::setFrustumCullingEnabled(bool enabled)
{
    mFrustumCulling = enabled;
}

bool Grass3D::isFrustumCullingEnabled() const
{
    return mFrustumCulling;
}

void Grass3D::setMaskResolution(int width, int depth)
{
    mMaskWidth = width < 2 ? 2 : width;
    mMaskDepth = depth < 2 ? 2 : depth;
}

void Grass3D::setMaskResolutionWidth(int width)
{
    mMaskWidth = width < 2 ? 2 : width;
}

int Grass3D::getMaskResolutionWidth() const
{
    return mMaskWidth;
}

void Grass3D::setMaskResolutionDepth(int depth)
{
    mMaskDepth = depth < 2 ? 2 : depth;
}

int Grass3D::getMaskResolutionDepth() const
{
    return mMaskDepth;
}

bool Grass3D::createMask(int width, int depth)
{
    if (!mHasExtent) return false;
    if (width <= 0) width = mMaskWidth;
    if (depth <= 0) depth = mMaskDepth;
    if (width < 2 || depth < 2) return false;
    Pixmap* mask = new Pixmap(width, depth, 4);
    if (!mask->is_valid())
    {
        delete mask;
        return false;
    }
    mask->fill(0, 0, 0, 0);
    delete mMask;
    mMask = mask;
    mMaskWidth = width;
    mMaskDepth = depth;
    markAllStale(-1);
    return true;
}

bool Grass3D::hasMask() const
{
    return mMask && mMask->is_valid() && mMask->components == 4;
}

int Grass3D::getMaskWidth() const
{
    return hasMask() ? mMask->width : 0;
}

int Grass3D::getMaskDepth() const
{
    return hasMask() ? mMask->height : 0;
}

void Grass3D::clearMask()
{
    if (!mMask) return;
    delete mMask;
    mMask = nullptr;
    markAllStale(-1);
}

void Grass3D::markAllStale(int type)
{
    for (std::size_t t = 0; t < mTypes.size(); ++t)
    {
        if (type >= 0 && static_cast<int>(t) != type) continue;
        Type& found = *mTypes[t];
        for (std::size_t i = 0; i < found.cells.size(); ++i)
            if (found.cells[i]->active) found.cells[i]->stale = true;
    }
}

void Grass3D::markStale(int type, float minX, float minZ, float maxX, float maxZ)
{
    int firstX = 0;
    int firstZ = 0;
    int lastX = 0;
    int lastZ = 0;
    cellOf(minX, minZ, mCellSize, firstX, firstZ);
    cellOf(maxX, maxZ, mCellSize, lastX, lastZ);
    for (std::size_t t = 0; t < mTypes.size(); ++t)
    {
        if (type >= 0 && static_cast<int>(t) != type) continue;
        Type& found = *mTypes[t];
        for (int cellZ = firstZ; cellZ <= lastZ; ++cellZ)
        {
            for (int cellX = firstX; cellX <= lastX; ++cellX)
            {
                Cell** cell = found.active.find(cellKey(cellX, cellZ));
                if (cell) (*cell)->stale = true;
            }
        }
    }
}

bool Grass3D::paintMask(const Math::Vec3& worldPosition, float radius, float amount, int type,
        bool subtract, bool allTypes)
{
    if (radius <= 0.0f || amount <= 0.0f || !mHasExtent) return false;
    if (!allTypes && !typeAt(type)) return false;
    if (!hasMask() && !createMask()) return false;

    const Math::Vec3 centre = getGlobalTransform().InverseAffine().TransformPoint(worldPosition);
    const float sizeX = mExtent[2] - mExtent[0];
    const float sizeZ = mExtent[3] - mExtent[1];
    if (sizeX <= 0.0f || sizeZ <= 0.0f) return false;
    const float scaleX = static_cast<float>(mMask->width - 1) / sizeX;
    const float scaleZ = static_cast<float>(mMask->height - 1) / sizeZ;
    int minX = static_cast<int>(floorf((centre.x - radius - mExtent[0]) * scaleX));
    int maxX = static_cast<int>(ceilf((centre.x + radius - mExtent[0]) * scaleX));
    int minZ = static_cast<int>(floorf((centre.z - radius - mExtent[1]) * scaleZ));
    int maxZ = static_cast<int>(ceilf((centre.z + radius - mExtent[1]) * scaleZ));
    const int lastX = mMask->width - 1;
    const int lastZ = mMask->height - 1;
    minX = minX < 0 ? 0 : (minX > lastX ? lastX : minX);
    maxX = maxX < 0 ? 0 : (maxX > lastX ? lastX : maxX);
    minZ = minZ < 0 ? 0 : (minZ > lastZ ? lastZ : minZ);
    maxZ = maxZ < 0 ? 0 : (maxZ > lastZ ? lastZ : maxZ);
    if (minX > maxX || minZ > maxZ) return false;

    int firstChannel = type;
    int lastChannel = type;
    if (allTypes)
    {
        firstChannel = 0;
        lastChannel = static_cast<int>(mTypes.size()) - 1;
    }

    bool changed = false;
    int changedMinX = lastX + 1;
    int changedMaxX = -1;
    int changedMinZ = lastZ + 1;
    int changedMaxZ = -1;
    for (int z = minZ; z <= maxZ; ++z)
    {
        for (int x = minX; x <= maxX; ++x)
        {
            const float localX = mExtent[0] + static_cast<float>(x) / scaleX;
            const float localZ = mExtent[1] + static_cast<float>(z) / scaleZ;
            const float dx = localX - centre.x;
            const float dz = localZ - centre.z;
            const float distance = sqrtf(dx * dx + dz * dz);
            if (distance > radius) continue;
            const float falloff = brushFalloff(distance, radius);
            const float delta = clampUnit(amount * falloff);
            bool pixelChanged = false;
            for (int channel = firstChannel; channel <= lastChannel; ++channel)
            {
                unsigned char& sample = mMask->pixels[(z * mMask->width + x) * 4 + channel];
                const float oldValue = static_cast<float>(sample) / 255.0f;
                float newValue = subtract ? oldValue - delta : oldValue + delta;
                newValue = clampUnit(newValue);
                const unsigned char encoded = static_cast<unsigned char>(newValue * 255.0f + 0.5f);
                if (encoded != sample) pixelChanged = true;
                sample = encoded;
            }
            if (pixelChanged)
            {
                changed = true;
                if (x < changedMinX) changedMinX = x;
                if (x > changedMaxX) changedMaxX = x;
                if (z < changedMinZ) changedMinZ = z;
                if (z > changedMaxZ) changedMaxZ = z;
            }
        }
    }
    if (!changed) return false;

    markStale(allTypes ? -1 : type, mExtent[0] + static_cast<float>(changedMinX - 1) / scaleX,
            mExtent[1] + static_cast<float>(changedMinZ - 1) / scaleZ,
            mExtent[0] + static_cast<float>(changedMaxX + 1) / scaleX,
            mExtent[1] + static_cast<float>(changedMaxZ + 1) / scaleZ);
    return true;
}

bool Grass3D::paint(const Math::Vec3& worldPosition, float radius, float amount, int type)
{
    return paintMask(worldPosition, radius, amount, type, false, false);
}

bool Grass3D::erase(const Math::Vec3& worldPosition, float radius, float amount, int type)
{
    return paintMask(worldPosition, radius, amount, type, true, false);
}

bool Grass3D::erase(const Math::Vec3& worldPosition, float radius, float amount)
{
    return paintMask(worldPosition, radius, amount, 0, true, true);
}

bool Grass3D::getMaskPixels(ct::Vector<std::uint8_t>& out) const
{
    if (!hasMask()) return false;
    const std::size_t count = static_cast<std::size_t>(mMask->width) * mMask->height * 4;
    out.resize(count);
    for (std::size_t i = 0; i < count; ++i) out[i] = mMask->pixels[i];
    return true;
}

bool Grass3D::setMaskPixels(const ct::Vector<std::uint8_t>& pixels)
{
    if (!hasMask()) return false;
    const std::size_t count = static_cast<std::size_t>(mMask->width) * mMask->height * 4;
    if (pixels.size() != count) return false;
    for (std::size_t i = 0; i < count; ++i) mMask->pixels[i] = pixels[i];
    markAllStale(-1);
    return true;
}

void Grass3D::fill(int type, float density)
{
    if (!typeAt(type)) return;
    if (!hasMask() && !createMask()) return;
    const unsigned char value = static_cast<unsigned char>(clampUnit(density) * 255.0f + 0.5f);
    const int count = mMask->width * mMask->height;
    for (int i = 0; i < count; ++i) mMask->pixels[i * 4 + type] = value;
    markAllStale(type);
}

float Grass3D::getMaskDensity(int type, float x, float z) const
{
    if (!hasMask() || !mHasExtent || type < 0 || type >= kMaxTypes) return 1.0f;
    const float sizeX = mExtent[2] - mExtent[0];
    const float sizeZ = mExtent[3] - mExtent[1];
    if (sizeX <= 0.0f || sizeZ <= 0.0f) return 0.0f;
    const float u = (x - mExtent[0]) / sizeX;
    const float v = (z - mExtent[1]) / sizeZ;
    if (u < 0.0f || u > 1.0f || v < 0.0f || v > 1.0f) return 0.0f;
    const float px = u * static_cast<float>(mMask->width - 1);
    const float pz = v * static_cast<float>(mMask->height - 1);
    const int x0 = static_cast<int>(floorf(px));
    const int z0 = static_cast<int>(floorf(pz));
    const int x1 = x0 + 1 < mMask->width - 1 ? x0 + 1 : mMask->width - 1;
    const int z1 = z0 + 1 < mMask->height - 1 ? z0 + 1 : mMask->height - 1;
    const float fx = px - static_cast<float>(x0);
    const float fz = pz - static_cast<float>(z0);
    const unsigned char* data = mMask->pixels;
    const int width = mMask->width;
    const float s00 = static_cast<float>(data[(z0 * width + x0) * 4 + type]) / 255.0f;
    const float s10 = static_cast<float>(data[(z0 * width + x1) * 4 + type]) / 255.0f;
    const float s01 = static_cast<float>(data[(z1 * width + x0) * 4 + type]) / 255.0f;
    const float s11 = static_cast<float>(data[(z1 * width + x1) * 4 + type]) / 255.0f;
    const float top = s00 + (s10 - s00) * fx;
    const float bottom = s01 + (s11 - s01) * fx;
    return top + (bottom - top) * fz;
}

bool Grass3D::saveMask(const char* path)
{
    if (!path || !*path || !hasMask()) return false;
    return mMask->save(path);
}

bool Grass3D::loadMask(const char* path)
{
    if (!path || !*path || !mHasExtent) return false;
    Pixmap source;
    if (!source.load(path) || source.width < 2 || source.height < 2) return false;
    Pixmap* rgba = new Pixmap(source.width, source.height, 4);
    if (!rgba->is_valid())
    {
        delete rgba;
        return false;
    }
    const bool sourceHasAlpha = source.components == 4;
    for (int z = 0; z < source.height; ++z)
    {
        for (int x = 0; x < source.width; ++x)
        {
            const Color color = source.get_pixel_color(static_cast<std::uint32_t>(x),
                    static_cast<std::uint32_t>(z));
            rgba->set_pixel(static_cast<std::uint32_t>(x), static_cast<std::uint32_t>(z), color.r(),
                    color.g(), color.b(), sourceHasAlpha ? color.a() : 0);
        }
    }
    delete mMask;
    mMask = rgba;
    mMaskWidth = rgba->width;
    mMaskDepth = rgba->height;
    markAllStale(-1);
    return true;
}

void Grass3D::ensureResources()
{
    RenderingServer* server = RenderingServer::singleton();
    if (!server) return;
    if (!server->meshes().valid(mMesh))
    {
        SurfaceData surface;
        buildTuftSurface(surface);
        mMesh = server->meshes().create("grass tuft");
        if (!server->meshes().addSurface(mMesh, surface, "tuft"))
        {
            server->meshes().destroy(mMesh);
            mMesh = MeshId();
        }
    }
    for (std::size_t i = 0; i < mTypes.size(); ++i)
    {
        Type& type = *mTypes[i];
        if (server->materials().valid(type.material)) continue;
        MaterialStore& materials = server->materials();
        type.material = materials.create("grass");
        materials.setFloat(type.material, MaterialParam::Roughness, 1.0f);
        materials.setFloat(type.material, MaterialParam::Specular, 0.0f);
        materials.setTransparency(type.material, MaterialTransparency::AlphaScissor);
        materials.setCull(type.material, MaterialCull::Disabled);
        applyMaterial(type);
        for (std::size_t c = 0; c < type.cells.size(); ++c)
            type.cells[c]->node->setSurfaceMaterial(0, type.material);
    }
}

void Grass3D::applyMaterial(Type& type)
{
    RenderingServer* server = RenderingServer::singleton();
    if (!server || !server->materials().valid(type.material)) return;
    MaterialStore& materials = server->materials();
    materials.setTexture(type.material, MaterialTexture::Albedo, type.desc.texture);
    materials.setFloat(type.material, MaterialParam::AlphaScissorThreshold, type.desc.alphaCut);
    materials.setAlphaAntialiasing(type.material, mAlphaAntialiasing, mAlphaAntialiasingEdge);
    materials.setGrass(type.material, true, mDrawDistance, mCameraBend, type.desc.colourA,
            type.desc.colourB);
    materials.setGrassWind(type.material, mWindDirection,
            mWind / (mStiffness > 0.001f ? mStiffness : 0.001f), mWindSpeed);
}

void Grass3D::releaseResources()
{
    RenderingServer* server = RenderingServer::singleton();
    if (!server) return;
    if (mMesh.valid()) server->meshes().destroy(mMesh);
    mMesh = MeshId();
}

Grass3D::Cell* Grass3D::acquireCell(Type& type, int typeIndex, int cellX, int cellZ)
{
    Cell* cell = nullptr;
    if (!type.pool.empty())
    {
        cell = type.pool.back();
        type.pool.pop_back();
    }
    else
    {
        cell = new Cell();
        cell->node = createChild<MultiMeshInstance3D>();
        cell->node->setInternal(true);
        cell->node->setUseColors(true);
        cell->node->setUseCustomData(true);
        cell->node->setMesh(mMesh);
        cell->node->setSurfaceMaterial(0, type.material);
        cell->node->setCastShadows(mCastShadows);
        type.cells.push_back(cell);
    }
    cell->cellX = cellX;
    cell->cellZ = cellZ;
    cell->active = true;
    cell->stale = false;
    fillCell(type, typeIndex, *cell);
    type.active.put(cellKey(cellX, cellZ), cell);
    return cell;
}

void Grass3D::releaseCell(Type& type, Cell* cell)
{
    type.active.erase(cellKey(cell->cellX, cell->cellZ));
    cell->active = false;
    cell->stale = false;
    cell->items.clear();
    cell->awake.clear();
    cell->clumps.clear();
    cell->shown = 0;
    cell->node->setVisibleInstanceCount(0);
    type.pool.push_back(cell);
}

void Grass3D::trimPool(Type& type)
{
    while (type.pool.size() > kMaxPooledCells)
    {
        Cell* cell = type.pool.back();
        type.pool.pop_back();
        for (std::size_t i = 0; i < type.cells.size(); ++i)
        {
            if (type.cells[i] == cell)
            {
                type.cells.erase(type.cells.begin() + static_cast<std::ptrdiff_t>(i));
                break;
            }
        }
        NodeFactory::destroy(cell->node);
        delete cell;
    }
}

void Grass3D::fillCell(Type& type, int typeIndex, Cell& cell)
{
    ++mFilled;
    const GrassTypeDesc& desc = type.desc;
    const float cellSize = mCellSize;
    const int count = candidateCount(mDensity * desc.densityScale, cellSize);
    std::uint32_t state = cellSeed(typeSeed(mSeed, typeIndex), cell.cellX, cell.cellZ);
    cell.clumps.clear();
    cell.clumps.resize(static_cast<std::size_t>(count));
    cell.items.clear();
    cell.awake.clear();
    const std::uint32_t regionCount =
            type.regions.empty() ? 1u : static_cast<std::uint32_t>(type.regions.size());
    const float total = totalWeight(type);
    const float originX = static_cast<float>(cell.cellX) * cellSize;
    const float originZ = static_cast<float>(cell.cellZ) * cellSize;
    Math::Box bounds;
    bool any = false;

    for (int k = 0; k < count; ++k)
    {
        Clump& clump = cell.clumps[static_cast<std::size_t>(k)];
        const float rx = nextRandom(state);
        const float rz = nextRandom(state);
        const float rRegion = nextRandom(state);
        const float rRotation = nextRandom(state);
        const float rScale = nextRandom(state);
        const float rKeep = nextRandom(state);

        const float x = originX + rx * cellSize;
        const float z = originZ + rz * cellSize;
        const float y = groundHeight(x, z);

        std::uint32_t region = 0;
        if (total > 0.0f)
        {
            float pick = rRegion * total;
            for (std::uint32_t i = 0; i < regionCount; ++i)
            {
                pick -= regionAt(type, i).weight;
                if (pick <= 0.0f)
                {
                    region = i;
                    break;
                }
            }
        }

        GrassClumpInfo& info = clump.info;
        info.root = Math::Vec3(x, y, z);
        info.rotation = rRotation * 2.0f * kPi;
        info.scale = desc.scaleMin + rScale * (desc.scaleMax - desc.scaleMin);
        info.region = region;
        info.keep = rKeep;
        info.inside = !mHasExtent ||
                      (x >= mExtent[0] && x <= mExtent[2] && z >= mExtent[1] && z <= mExtent[3]);
        info.density = clumpDensity(typeIndex, x, z);

        const GrassRegion shape = regionAt(type, region);
        const float height = desc.height * info.scale * shape.size;
        const float width = height * shape.aspect * 0.5f * desc.width;
        clump.length = desc.height * info.scale;
        if (clump.length < 0.001f) clump.length = 0.001f;
        clump.transform = Math::Mat4::Translation(info.root) *
                          Math::Mat4::RotationY(info.rotation) *
                          Math::Mat4::Scale(Math::Vec3(width, height, width));
        clump.regionData = shape.texMulAdd;
        clump.tip = GrassTip();
        clump.slot = -1;

        if (info.inside)
        {
            const float bend = mCameraBend > 0.0f ? mCameraBend : 0.0f;
            const float reach = height * (1.0f + bend) + clump.length + width;
            bounds = bounds.Expanded(Math::Vec3(x - reach, y - reach, z - reach));
            bounds = bounds.Expanded(Math::Vec3(x + reach, y + reach, z + reach));
            any = true;
        }
    }

    if (any)
    {
        cell.boundsCentre = (bounds.min + bounds.max) * 0.5f;
        cell.boundsRadius = (bounds.max - bounds.min).Length() * 0.5f;
    }
    else
    {
        cell.boundsCentre = Math::Vec3(originX + cellSize * 0.5f, 0.0f, originZ + cellSize * 0.5f);
        cell.boundsRadius = 0.0f;
        bounds = Math::Box();
    }

    cell.shown = 0;
    cell.node->setVisibleInstanceCount(0);
    if (cell.node->getInstanceCount() != count) cell.node->setInstanceCount(count);
    cell.node->getMultiMesh()->setCustomAabb(bounds);
}

bool Grass3D::update(const Math::Vec3& cameraPosition)
{
    return update(cameraPosition, nullptr, 0.0f);
}

bool Grass3D::update(const Math::Vec3& cameraPosition, const Math::Frustum* frustum,
        float deltaTime)
{
    const std::uint64_t started = Main::device() ? Main::device()->getTicksUsec() : 0;
    mWritten = 0;
    mMoving = 0;
    mFilled = 0;
    bool changed = false;

    if (mTypes.empty()) addType(GrassTypeDesc());
    ensureResources();
    if (!mMesh.valid()) return false;

    if (mDirty)
    {
        for (std::size_t t = 0; t < mTypes.size(); ++t)
        {
            Type& type = *mTypes[t];
            for (std::size_t i = 0; i < type.cells.size(); ++i)
                if (type.cells[i]->active) releaseCell(type, type.cells[i]);
            type.active.clear();
            trimPool(type);
        }
        mDirty = false;
        changed = true;
    }

    mTime += deltaTime;

    const Math::Mat4 world = getGlobalTransform();
    const Math::Mat4 inverse = world.InverseAffine();
    const Math::Vec3 camera = inverse.TransformPoint(cameraPosition);
    const float worldScale = Math::Vec3(world.col1.x, world.col1.y, world.col1.z).Length();

    GrassInfluencer local[kMaxInfluencers];
    const int influencerCount = static_cast<int>(mInfluencers.size());
    for (int i = 0; i < influencerCount; ++i)
    {
        local[i] = mInfluencers[static_cast<std::size_t>(i)];
        local[i].centre = inverse.TransformPoint(local[i].centre);
    }

    GrassSimParams params;
    params.stiffness = mStiffness;
    params.drag = mDrag;
    params.deltaTime = deltaTime;
    params.time = mTime;
    params.wind = mWind;
    params.windSpeed = mWindSpeed;
    params.windDirection = mWindDirection;

    for (std::size_t t = 0; t < mTypes.size(); ++t)
        updateType(*mTypes[t], static_cast<int>(t), camera, frustum, world, worldScale, local,
                influencerCount, params, changed);

    mInfluencers.clear();
    if (mWritten > 0) changed = true;
    const std::uint64_t finished = Main::device() ? Main::device()->getTicksUsec() : 0;
    mUpdateMs = static_cast<float>(finished - started) / 1000.0f;
    return changed;
}

void Grass3D::updateType(Type& type, int typeIndex, const Math::Vec3& camera,
        const Math::Frustum* frustum, const Math::Mat4& world, float worldScale,
        const GrassInfluencer* influencers, int influencerCount, const GrassSimParams& params,
        bool& changed)
{
    const float range = mDrawDistance;
    int minX = 0;
    int minZ = 0;
    int maxX = 0;
    int maxZ = 0;
    cellOf(camera.x - range, camera.z - range, mCellSize, minX, minZ);
    cellOf(camera.x + range, camera.z + range, mCellSize, maxX, maxZ);
    minX -= 1;
    minZ -= 1;
    if (mHasExtent)
    {
        int extentMinX = 0;
        int extentMinZ = 0;
        int extentMaxX = 0;
        int extentMaxZ = 0;
        cellOf(mExtent[0], mExtent[1], mCellSize, extentMinX, extentMinZ);
        extentMaxX = static_cast<int>(ceilf(mExtent[2] / mCellSize)) - 1;
        extentMaxZ = static_cast<int>(ceilf(mExtent[3] / mCellSize)) - 1;
        if (extentMinX > minX) minX = extentMinX;
        if (extentMinZ > minZ) minZ = extentMinZ;
        if (extentMaxX < maxX) maxX = extentMaxX;
        if (extentMaxZ < maxZ) maxZ = extentMaxZ;
    }

    for (std::size_t i = 0; i < type.cells.size(); ++i)
    {
        Cell* cell = type.cells[i];
        if (cell->active && cellDistance(cell->cellX, cell->cellZ, mCellSize, camera.x, camera.z) >
                                    range + mReleaseMargin)
        {
            releaseCell(type, cell);
            changed = true;
        }
    }
    trimPool(type);

    for (int cellZ = minZ; cellZ <= maxZ; ++cellZ)
    {
        for (int cellX = minX; cellX <= maxX; ++cellX)
        {
            if (cellDistance(cellX, cellZ, mCellSize, camera.x, camera.z) > range) continue;
            if (type.active.find(cellKey(cellX, cellZ))) continue;
            acquireCell(type, typeIndex, cellX, cellZ);
            changed = true;
        }
    }

    const Math::Vec3 up(0.0f, 1.0f, 0.0f);
    GrassSimParams calm = params;
    calm.wind = 0.0f;
    for (std::size_t c = 0; c < type.cells.size(); ++c)
    {
        Cell& cell = *type.cells[c];
        if (!cell.active) continue;
        if (cell.stale)
        {
            cell.stale = false;
            fillCell(type, typeIndex, cell);
            changed = true;
        }

        const float distance = cellDistance(cell.cellX, cell.cellZ, mCellSize, camera.x, camera.z);
        bool visible = distance <= range && cell.boundsRadius > 0.0f;
        if (visible && frustum && mFrustumCulling)
            visible = frustum->IntersectsSphere(world.TransformPoint(cell.boundsCentre),
                    cell.boundsRadius * worldScale);

        MultiMeshInstance3D* node = cell.node;
        for (std::size_t k = 0; k < cell.clumps.size(); ++k)
        {
            Clump& clump = cell.clumps[k];
            const Math::Vec3 offset = clump.info.root - camera;
            const float clumpDistance = offset.Length();
            const bool want = clump.info.inside &&
                              clump.info.keep < clump.info.density * distanceKeep(clumpDistance,
                                                                             range, mFalloffStart);
            if (want && clump.slot < 0)
            {
                clump.slot = static_cast<std::int32_t>(cell.items.size());
                cell.items.push_back(static_cast<std::uint32_t>(k));
                clump.tip = GrassTip();
                clump.written = Math::Vec3(0.0f, 0.0f, 0.0f);
                node->setInstanceTransform(clump.slot, clump.transform);
                node->setInstanceColor(clump.slot, clump.regionData);
                node->setInstanceCustomData(clump.slot, Math::Vec4(clump.written, clump.length));
                ++mWritten;
            }
            else if (!want && clump.slot >= 0)
            {
                if (clump.awake)
                {
                    for (std::size_t a = 0; a < cell.awake.size(); ++a)
                    {
                        if (cell.awake[a] != k) continue;
                        cell.awake[a] = cell.awake.back();
                        cell.awake.pop_back();
                        break;
                    }
                    clump.awake = false;
                }
                const std::size_t last = cell.items.size() - 1;
                if (static_cast<std::size_t>(clump.slot) != last)
                {
                    const std::uint32_t moved = cell.items[last];
                    Clump& other = cell.clumps[moved];
                    cell.items[static_cast<std::size_t>(clump.slot)] = moved;
                    other.slot = clump.slot;
                    node->setInstanceTransform(other.slot, other.transform);
                    node->setInstanceColor(other.slot, other.regionData);
                    node->setInstanceCustomData(other.slot,
                            Math::Vec4(other.written, other.length));
                    ++mWritten;
                }
                cell.items.pop_back();
                clump.slot = -1;
            }
        }

        for (int i = 0; i < influencerCount; ++i)
        {
            const GrassInfluencer& influencer = influencers[i];
            if (cell.boundsRadius <= 0.0f || (cell.boundsCentre - influencer.centre).Length() >
                                                     cell.boundsRadius + influencer.radius)
                continue;
            for (std::size_t s = 0; s < cell.items.size(); ++s)
            {
                Clump& clump = cell.clumps[cell.items[s]];
                if (clump.awake) continue;
                if ((clump.info.root - influencer.centre).Length() <
                        influencer.radius + clump.length)
                {
                    clump.awake = true;
                    clump.tip = GrassTip();
                    cell.awake.push_back(cell.items[s]);
                }
            }
        }

        for (std::size_t a = 0; a < cell.awake.size();)
        {
            Clump& clump = cell.clumps[cell.awake[a]];
            simulate(clump.info.root, up, clump.length, clump.tip, calm, influencers,
                    influencerCount);
            const Math::Vec3 rest = up * clump.length;
            const Math::Vec3 deviation = clump.tip.current - clump.info.root - rest;
            ++mMoving;
            const bool settled = deviation.LengthSquared() < 1e-8f &&
                                 (clump.tip.current - clump.tip.previous).LengthSquared() < 1e-10f;
            const Math::Vec3 target = settled ? Math::Vec3(0.0f, 0.0f, 0.0f) : deviation;
            if ((target - clump.written).LengthSquared() > 1e-10f)
            {
                node->setInstanceCustomData(clump.slot, Math::Vec4(target, clump.length));
                clump.written = target;
                ++mWritten;
            }
            if (settled)
            {
                clump.awake = false;
                clump.tip = GrassTip();
                cell.awake[a] = cell.awake.back();
                cell.awake.pop_back();
            }
            else
            {
                ++a;
            }
        }

        const int show = visible ? static_cast<int>(cell.items.size()) : 0;
        if (show != cell.shown)
        {
            node->setVisibleInstanceCount(show);
            cell.shown = show;
            changed = true;
        }
    }
}

int Grass3D::getActiveCellCount(int type) const
{
    int count = 0;
    for (std::size_t t = 0; t < mTypes.size(); ++t)
        if (type < 0 || static_cast<int>(t) == type)
            count += static_cast<int>(mTypes[t]->active.size());
    return count;
}

int Grass3D::getVisibleCellCount(int type) const
{
    int count = 0;
    for (std::size_t t = 0; t < mTypes.size(); ++t)
    {
        if (type >= 0 && static_cast<int>(t) != type) continue;
        for (std::size_t i = 0; i < mTypes[t]->cells.size(); ++i)
            if (mTypes[t]->cells[i]->active && mTypes[t]->cells[i]->shown > 0) ++count;
    }
    return count;
}

int Grass3D::getPooledCellCount(int type) const
{
    int count = 0;
    for (std::size_t t = 0; t < mTypes.size(); ++t)
        if (type < 0 || static_cast<int>(t) == type)
            count += static_cast<int>(mTypes[t]->pool.size());
    return count;
}

int Grass3D::getInstanceCount(int type) const
{
    int count = 0;
    for (std::size_t t = 0; t < mTypes.size(); ++t)
    {
        if (type >= 0 && static_cast<int>(t) != type) continue;
        for (std::size_t i = 0; i < mTypes[t]->cells.size(); ++i)
            if (mTypes[t]->cells[i]->active)
                count += static_cast<int>(mTypes[t]->cells[i]->items.size());
    }
    return count;
}

int Grass3D::getDrawnInstanceCount(int type) const
{
    int count = 0;
    for (std::size_t t = 0; t < mTypes.size(); ++t)
    {
        if (type >= 0 && static_cast<int>(t) != type) continue;
        for (std::size_t i = 0; i < mTypes[t]->cells.size(); ++i)
            if (mTypes[t]->cells[i]->active) count += mTypes[t]->cells[i]->shown;
    }
    return count;
}

int Grass3D::getWrittenInstanceCount() const
{
    return mWritten;
}

int Grass3D::getSimulatedInstanceCount() const
{
    return mMoving;
}

int Grass3D::getAwakeInstanceCount() const
{
    int count = 0;
    for (std::size_t t = 0; t < mTypes.size(); ++t)
        for (std::size_t i = 0; i < mTypes[t]->cells.size(); ++i)
            if (mTypes[t]->cells[i]->active)
                count += static_cast<int>(mTypes[t]->cells[i]->awake.size());
    return count;
}

int Grass3D::getFilledCellCount() const
{
    return mFilled;
}

float Grass3D::getUpdateMilliseconds() const
{
    return mUpdateMs;
}

bool Grass3D::hasCell(int type, int cellX, int cellZ) const
{
    const Type* found = typeAt(type);
    return found && found->active.find(cellKey(cellX, cellZ)) != nullptr;
}

int Grass3D::getCellClumpCount(int type, int cellX, int cellZ) const
{
    const Type* found = typeAt(type);
    Cell* const* cell = found ? found->active.find(cellKey(cellX, cellZ)) : nullptr;
    return cell ? static_cast<int>((*cell)->items.size()) : 0;
}

bool Grass3D::getClump(int type, int cellX, int cellZ, int index, GrassClumpInfo& out) const
{
    const Type* found = typeAt(type);
    Cell* const* cell = found ? found->active.find(cellKey(cellX, cellZ)) : nullptr;
    if (!cell || index < 0 || static_cast<std::size_t>(index) >= (*cell)->clumps.size())
        return false;
    const Clump& clump = (*cell)->clumps[static_cast<std::size_t>(index)];
    out = clump.info;
    out.live = clump.slot >= 0;
    out.awake = clump.awake;
    out.tip = clump.awake ? clump.tip.current
                          : clump.info.root + Math::Vec3(0.0f, clump.length, 0.0f);
    return true;
}

MultiMeshInstance3D* Grass3D::getCellNode(int type, int cellX, int cellZ) const
{
    const Type* found = typeAt(type);
    Cell* const* cell = found ? found->active.find(cellKey(cellX, cellZ)) : nullptr;
    return cell ? (*cell)->node : nullptr;
}

void Grass3D::notificationHandler(int what)
{
    Node3D::notificationHandler(what);

    switch (what)
    {
        case kNotificationEnterTree:
            if (!mTerrainPath.empty()) resolveTerrain();
            connectPreDraw();
            break;
        case kNotificationExitTree:
            disconnectPreDraw();
            break;
        default:
            break;
    }
}

void Grass3D::updateFromCamera()
{
    if (!isVisibleInTree()) return;
    Viewport* viewport = Viewport::findViewport(this);
    if (!viewport) return;
    Camera3D* camera = viewport->getCamera3D();
    if (!camera) return;
    SceneTree* tree = getTree();
    const float delta = tree ? static_cast<float>(tree->getProcessDeltaTime()) : 0.0f;
    const Math::Frustum frustum = Math::Frustum::FromViewProjection(
            camera->getCameraProjection() * camera->getGlobalTransform().InverseAffine());
    update(camera->getGlobalPosition(), &frustum, delta);
}

void Grass3D::connectPreDraw()
{
    RenderingServer* server = RenderingServer::singleton();
    if (!server || mConnected) return;
    server->framePreDrawConnect(this, [this]() { updateFromCamera(); });
    mConnected = true;
}

void Grass3D::disconnectPreDraw()
{
    RenderingServer* server = RenderingServer::singleton();
    if (server && mConnected) server->framePreDrawDisconnect(this);
    mConnected = false;
}

void Grass3D::validateProperty(PropertyInfo& info) const
{
    Node3D::validateProperty(info);
    if (info.name.size() < 7 || !info.name.starts_with("type_") || info.name[6] != '/') return;
    const char digit = info.name[5];
    if (digit < '0' || digit > '9') return;
    if (digit - '0' >= getTypeCount()) info.usage = PropertyUsage::NoEditor;
}

void Grass3D::bindProperties()
{
    using DB = NodeClassDB;
    using P = Grass3D;
    DB::addProperty<P>(PropertyInfo::nodePath("terrain", "Terrain3D"), &P::setTerrainPath,
            &P::getTerrainPath);
    DB::addPropertyGroup<P>("Types", "type_");
    const auto editorOnly = [](PropertyInfo info)
    {
        info.usage = PropertyUsage::Editor;
        return info;
    };
    DB::addProperty(P::classNameStatic(),
            editorOnly(PropertyInfo::range(PropertyType::Int, "type_count", 0.0,
                    static_cast<double>(kMaxTypes), 1.0)),
            PropertySetter([](Node* node, const PropertyValue& value)
                    { static_cast<P*>(node)->setTypeCount(static_cast<int>(value.asInt())); }),
            PropertyGetter(
                    [](const Node* node) {
                        return PropertyValue::fromInt(static_cast<const P*>(node)->getTypeCount());
                    }));
    static const char* const kTypeLabels[kMaxTypes] = { "Type 1", "Type 2", "Type 3", "Type 4" };
    static const char* const kTypePrefixes[kMaxTypes] = { "type_0/", "type_1/", "type_2/",
        "type_3/" };
    static const char* const kTypeNames[kMaxTypes][9] = {
        { "type_0/texture", "type_0/height", "type_0/width", "type_0/alpha_cut", "type_0/colour_a",
            "type_0/colour_b", "type_0/scale_min", "type_0/scale_max", "type_0/density_scale" },
        { "type_1/texture", "type_1/height", "type_1/width", "type_1/alpha_cut", "type_1/colour_a",
            "type_1/colour_b", "type_1/scale_min", "type_1/scale_max", "type_1/density_scale" },
        { "type_2/texture", "type_2/height", "type_2/width", "type_2/alpha_cut", "type_2/colour_a",
            "type_2/colour_b", "type_2/scale_min", "type_2/scale_max", "type_2/density_scale" },
        { "type_3/texture", "type_3/height", "type_3/width", "type_3/alpha_cut", "type_3/colour_a",
            "type_3/colour_b", "type_3/scale_min", "type_3/scale_max", "type_3/density_scale" },
    };
    for (int i = 0; i < kMaxTypes; ++i)
    {
        DB::addPropertySubgroup<P>(kTypeLabels[i], kTypePrefixes[i]);
        const char* const* names = kTypeNames[i];
        DB::addProperty(P::classNameStatic(),
                editorOnly(PropertyInfo::resource(names[0], "Texture2D")),
                PropertySetter([i](Node* node, const PropertyValue& value)
                        { static_cast<P*>(node)->setTypeTexturePath(i, value.asString()); }),
                PropertyGetter(
                        [i](const Node* node) {
                            return PropertyValue::fromResourcePath(
                                    static_cast<const P*>(node)->getTypeTexturePath(i));
                        }));
        const auto floatProperty = [&](const char* name, double minimum, double maximum,
                                           double step, float GrassTypeDesc::*field)
        {
            DB::addProperty(P::classNameStatic(),
                    editorOnly(PropertyInfo::range(PropertyType::Float, name, minimum, maximum,
                            step, maximum >= 10.0)),
                    PropertySetter(
                            [i, field](Node* node, const PropertyValue& value)
                            {
                                P* grass = static_cast<P*>(node);
                                GrassTypeDesc desc = grass->typeOrDefault(i);
                                desc.*field = static_cast<float>(value.asFloat());
                                grass->setTypeDesc(i, desc);
                            }),
                    PropertyGetter(
                            [i, field](const Node* node) {
                                return PropertyValue::fromFloat(
                                        static_cast<const P*>(node)->typeOrDefault(i).*field);
                            }));
        };
        floatProperty(names[1], 0.0, 100.0, 0.01, &GrassTypeDesc::height);
        floatProperty(names[2], 0.0, 10.0, 0.01, &GrassTypeDesc::width);
        floatProperty(names[3], 0.0, 1.0, 0.01, &GrassTypeDesc::alphaCut);
        const auto colourProperty = [&](const char* name, Math::Vec3 GrassTypeDesc::*field)
        {
            DB::addProperty(P::classNameStatic(), editorOnly(PropertyInfo::color(name, true)),
                    PropertySetter(
                            [i, field](Node* node, const PropertyValue& value)
                            {
                                P* grass = static_cast<P*>(node);
                                GrassTypeDesc desc = grass->typeOrDefault(i);
                                const Math::Vec4 colour = value.asColor();
                                desc.*field = Math::Vec3(colour.x, colour.y, colour.z);
                                grass->setTypeDesc(i, desc);
                            }),
                    PropertyGetter(
                            [i, field](const Node* node)
                            {
                                const Math::Vec3 colour =
                                        static_cast<const P*>(node)->typeOrDefault(i).*field;
                                return PropertyValue::fromColor(
                                        Math::Vec4(colour.x, colour.y, colour.z, 1.0f));
                            }));
        };
        colourProperty(names[4], &GrassTypeDesc::colourA);
        colourProperty(names[5], &GrassTypeDesc::colourB);
        floatProperty(names[6], 0.0, 10.0, 0.01, &GrassTypeDesc::scaleMin);
        floatProperty(names[7], 0.0, 10.0, 0.01, &GrassTypeDesc::scaleMax);
        floatProperty(names[8], 0.0, 10.0, 0.01, &GrassTypeDesc::densityScale);
    }
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "max_slope", 0.0, 90.0, 0.1),
            &P::setMaxSlope, &P::getMaxSlope);

    DB::addPropertyGroup<P>("Placement", "");
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "cell_size", 0.5, 1024.0, 0.01, true),
            &P::setCellSize, &P::getCellSize);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "density", 0.0, 1024.0, 0.001, true),
            &P::setDensity, &P::getDensity);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "density_falloff_start", 0.0, 1.0, 0.01),
            &P::setDensityFalloffStart, &P::getDensityFalloffStart);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "draw_distance", 0.0, 100000.0, 0.1, true),
            &P::setDrawDistance, &P::getDrawDistance);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "release_margin", 0.0, 100000.0, 0.1, true),
            &P::setReleaseMargin, &P::getReleaseMargin);
    DB::addProperty<P>(PropertyInfo(PropertyType::Int, "seed"), &P::setSeed, &P::getSeed);

    DB::addPropertyGroup<P>("Rendering", "");
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "cast_shadows"), &P::setCastShadows,
            &P::getCastShadows);
    DB::addProperty<P>(
            PropertyInfo::enumeration("alpha_antialiasing", "Off,Alpha To Coverage"),
            [](P* grass, const PropertyValue& value)
            { grass->setAlphaAntialiasing(static_cast<MaterialAlphaAntialiasing>(value.asInt())); },
            &P::getAlphaAntialiasing);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "camera_bend", 0.0, 1.0, 0.01, true, true),
            &P::setCameraBend, &P::getCameraBend);
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "frustum_culling_enabled"),
            &P::setFrustumCullingEnabled, &P::isFrustumCullingEnabled);

    DB::addPropertyGroup<P>("Wind", "wind_");
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "wind", 0.0, 16.0, 0.01, true, true),
            &P::setWind, &P::getWind);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "wind_speed", 0.0, 16.0, 0.01, true, true),
            &P::setWindSpeed, &P::getWindSpeed);
    DB::addProperty<P>(PropertyInfo(PropertyType::Vec3, "wind_direction"), &P::setWindDirection,
            &P::getWindDirection);

    DB::addPropertyGroup<P>("Mask", "");
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "extent_enabled"), &P::setExtentEnabled,
            &P::hasExtent);
    DB::addProperty<P>(PropertyInfo(PropertyType::Vec2, "extent_min"), &P::setExtentMin,
            &P::getExtentMin);
    DB::addProperty<P>(PropertyInfo(PropertyType::Vec2, "extent_max"), &P::setExtentMax,
            &P::getExtentMax);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Int, "mask_resolution_width", 2.0, 16384.0,
                               1.0, true),
            &P::setMaskResolutionWidth, &P::getMaskResolutionWidth);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Int, "mask_resolution_depth", 2.0, 16384.0,
                               1.0, true),
            &P::setMaskResolutionDepth, &P::getMaskResolutionDepth);

    DB::addPropertyGroup<P>("Simulation", "");
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "stiffness", 0.0, 1000.0, 0.01, true),
            &P::setStiffness, &P::getStiffness);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "drag", 0.0, 1.0, 0.01),
            &P::setDrag, &P::getDrag);
}

namespace
{

constexpr std::int64_t kGrassDataVersion = 1;

}

bool Grass3D::saveData(ct::Json& data, SceneResult& result, const ct::String& path) const
{
    const bool hasExclusions = mPlacement.getExclusionCount() > 0;
    const bool maskPresent = hasMask();
    if (mTypes.empty() && !hasExclusions && !maskPresent) return false;

    data["version"] = ct::Json(static_cast<long long>(kGrassDataVersion));

    ct::Json types = ct::Json::array();
    for (std::size_t i = 0; i < mTypes.size(); ++i)
    {
        const Type& type = *mTypes[i];
        const GrassTypeDesc& desc = type.desc;
        ct::Json entry = ct::Json::object();
        if (desc.texture.valid())
        {
            const ct::String texture = sceneres::pathOf(desc.texture);
            if (texture.empty())
                result.addWarning(scenejson::element(path, "types", i),
                        "the texture has no path; it was not saved");
            else
                entry["texture"] = ct::Json(texture);
        }
        entry["height"] = scenejson::fromFloat(desc.height);
        entry["width"] = scenejson::fromFloat(desc.width);
        entry["scale_min"] = scenejson::fromFloat(desc.scaleMin);
        entry["scale_max"] = scenejson::fromFloat(desc.scaleMax);
        entry["colour_a"] = scenejson::fromVec3(desc.colourA);
        entry["colour_b"] = scenejson::fromVec3(desc.colourB);
        entry["alpha_cut"] = scenejson::fromFloat(desc.alphaCut);
        entry["density_scale"] = scenejson::fromFloat(desc.densityScale);
        if (!type.regions.empty())
        {
            ct::Json regions = ct::Json::array();
            for (std::size_t r = 0; r < type.regions.size(); ++r)
            {
                const GrassRegion& region = type.regions[r];
                ct::Json item = ct::Json::object();
                item["tex_mul_add"] = scenejson::fromVec4(region.texMulAdd);
                item["size"] = scenejson::fromFloat(region.size);
                item["aspect"] = scenejson::fromFloat(region.aspect);
                item["weight"] = scenejson::fromFloat(region.weight);
                regions.push_back(item);
            }
            entry["regions"] = regions;
        }
        types.push_back(entry);
    }
    if (types.size() > 0) data["types"] = types;

    if (maskPresent)
    {
        ct::Json mask = ct::Json::object();
        mask["width"] = ct::Json(static_cast<long long>(mMask->width));
        mask["depth"] = ct::Json(static_cast<long long>(mMask->height));
        mask["pixels"] =
                scenejson::fromBlock(mMask->pixels, static_cast<std::size_t>(mMask->get_size()));
        data["mask"] = mask;
    }
    if (hasExclusions) data["exclusions"] = scenejson::fromExclusions(mPlacement);
    return true;
}

bool Grass3D::loadData(const ct::Json& data, SceneResult& result, const ct::String& path)
{
    if (!scenejson::checkVersion(data, kGrassDataVersion, result, path)) return false;

    if (const ct::Json* types = scenejson::arrayField(data, "types", result, path))
    {
        for (std::size_t i = 0; i < types->size(); ++i)
        {
            const ct::Json& entry = types->at(i);
            const ct::String entryPath = scenejson::element(path, "types", i);
            if (!entry.is_object())
            {
                result.addError(entryPath, "expected an object");
                continue;
            }
            GrassTypeDesc desc;
            const ct::String texture = scenejson::getString(entry, "texture", ct::String());
            if (!texture.empty())
            {
                sceneres::clearError();
                desc.texture = sceneres::loadTexture(texture, true);
                if (!desc.texture.valid())
                    result.addWarning(scenejson::field(entryPath, "texture"),
                            sceneres::takeError());
            }
            desc.height = scenejson::getFloat(entry, "height", desc.height);
            desc.width = scenejson::getFloat(entry, "width", desc.width);
            desc.scaleMin = scenejson::getFloat(entry, "scale_min", desc.scaleMin);
            desc.scaleMax = scenejson::getFloat(entry, "scale_max", desc.scaleMax);
            scenejson::readVec3Field(entry, "colour_a", desc.colourA, result, entryPath);
            scenejson::readVec3Field(entry, "colour_b", desc.colourB, result, entryPath);
            desc.alphaCut = scenejson::getFloat(entry, "alpha_cut", desc.alphaCut);
            desc.densityScale = scenejson::getFloat(entry, "density_scale", desc.densityScale);
            int index = static_cast<int>(i);
            if (index < getTypeCount())
            {
                setType(index, desc);
            }
            else
            {
                index = addType(desc);
                if (index < 0)
                {
                    result.addError(entryPath, "there is room for four types");
                    continue;
                }
            }
            clearTypeRegions(index);
            if (const ct::Json* regions =
                            scenejson::arrayField(entry, "regions", result, entryPath))
            {
                Type* target = typeAt(index);
                for (std::size_t r = 0; target && r < regions->size(); ++r)
                {
                    const ct::Json& item = regions->at(r);
                    GrassRegion region;
                    const ct::String regionPath = scenejson::element(entryPath, "regions", r);
                    if (!item.is_object() || !scenejson::readVec4Field(item, "tex_mul_add",
                                                     region.texMulAdd, result, regionPath))
                    {
                        result.addError(regionPath, "needs tex_mul_add");
                        continue;
                    }
                    region.size = scenejson::getFloat(item, "size", 1.0f);
                    region.aspect = scenejson::getFloat(item, "aspect", 1.0f);
                    region.weight = scenejson::getFloat(item, "weight", 1.0f);
                    target->regions.push_back(region);
                }
            }
            markDirty();
        }
    }

    if (const ct::Json* mask = scenejson::objectField(data, "mask", result, path))
    {
        const ct::String maskPath = scenejson::field(path, "mask");
        const std::int64_t width = scenejson::getInt(*mask, "width", 0);
        const std::int64_t depth = scenejson::getInt(*mask, "depth", 0);
        ct::Vector<std::uint8_t> bytes;
        const ct::Json* block = mask->find("pixels");
        if (width < 2 || depth < 2 || !block || !scenejson::toBlock(*block, bytes) ||
                bytes.size() != static_cast<std::size_t>(width * depth * 4))
        {
            result.addError(maskPath, "needs width, depth and width * depth * 4 bytes");
        }
        else if (!createMask(static_cast<int>(width), static_cast<int>(depth)))
        {
            result.addError(maskPath, "the mask needs an extent");
        }
        else
        {
            memcpy(mMask->pixels, bytes.data(), bytes.size());
            markAllStale(-1);
        }
    }

    if (const ct::Json* exclusions = data.find("exclusions"))
        scenejson::toExclusions(*exclusions, mPlacement, result,
                scenejson::field(path, "exclusions"));
    return true;
}

} // namespace prisma3d
