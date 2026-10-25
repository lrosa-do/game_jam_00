#include "prisma3d/scene/Terrain3D.h"

#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/ResourceProperties.h"
#include "prisma3d/scene/SceneJson.h"

#include "NodeFactory.h"
#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/scene/Camera3D.h"
#include "prisma3d/scene/Viewport.h"

#include "prisma3d/core/Pixmap.h"
#include "prisma3d/terrain/TerrainMapTarget.h"

#include "platform.h"

#include <ct/json.hpp>

#include <float.h>
#include <math.h>
#include <string.h>
#include <time.h>

namespace prisma3d
{

namespace
{

const char* const kDefaultMaterialName = "terrain";
const char* const kVertexMeshName = "terrain vertices";
const char* const kPoolMeshName = "terrain indices";
const char* const kNodeMeshName = "terrain node";
const char* const kCompositeMaterialName = "terrain composite";

unsigned long monotonicMilliseconds()
{
    return static_cast<unsigned long>(time_nanos() / 1000000ull);
}

std::uint32_t bytesPerTexel(prisma::TextureFormat format)
{
    switch (format)
    {
        case prisma::TextureFormat::R8:
            return 1;
        default:
            return 4;
    }
}

std::uint64_t nodeKey(const TerrainQuadTreeNode* node)
{
    return static_cast<std::uint64_t>(reinterpret_cast<std::uintptr_t>(node));
}

float packMorph(const Math::Vec2& morph)
{
    float transition = morph.x;
    transition = transition < 0.0f ? 0.0f : (transition > 1.0f ? 1.0f : transition);
    const float target = floorf(morph.y + 0.5f);
    return (target * 2.0f + transition + 2.0f) / 64.0f;
}

} // namespace

class Terrain3D::MapTarget : public TerrainMapTarget
{
public:
    explicit MapTarget(Terrain3D& owner)
            : mOwner(owner)
    {
    }

    bool createNormalMap(std::uint16_t size) override
    {
        ct::Vector<std::uint8_t> init;
        init.resize(static_cast<std::size_t>(size) * size * 4);
        for (std::size_t i = 0; i < init.size(); i += 4)
        {
            init[i] = 128;
            init[i + 1] = 255;
            init[i + 2] = 128;
            init[i + 3] = 255;
        }
        return mOwner.createMapTexture(mOwner.mNormalMap, prisma::TextureFormat::RGBA8, size, 1,
                false, prisma::kTextureSampled, init.data(), "terrain normal map");
    }
    void destroyNormalMap() override
    {
        mOwner.destroyMapTexture(mOwner.mNormalMap);
    }
    void writeNormalMap(const TerrainRect& box, const TerrainPixelBox& rgb) override
    {
        if (!mOwner.mNormalMap.valid()) return;
        ct::Vector<std::uint8_t> rgba;
        rgba.resize(static_cast<std::size_t>(rgb.width) * rgb.height * 4);
        for (std::size_t i = 0; i < static_cast<std::size_t>(rgb.width) * rgb.height; ++i)
        {
            rgba[i * 4] = rgb.data[i * 3];
            rgba[i * 4 + 1] = rgb.data[i * 3 + 1];
            rgba[i * 4 + 2] = rgb.data[i * 3 + 2];
            rgba[i * 4 + 3] = 255;
        }
        mOwner.uploadRegion(mOwner.mNormalMap, box, rgba.data(), 4, false);
    }

    bool createLightmap(std::uint16_t size) override
    {
        ct::Vector<std::uint8_t> init;
        init.resize(static_cast<std::size_t>(size) * size);
        memset(init.data(), 255, init.size());
        return mOwner.createMapTexture(mOwner.mLightmap, prisma::TextureFormat::R8, size, 1, false,
                prisma::kTextureSampled, init.data(), "terrain lightmap");
    }
    void destroyLightmap() override
    {
        mOwner.destroyMapTexture(mOwner.mLightmap);
    }
    void writeLightmap(const TerrainRect& box, const TerrainPixelBox& l8) override
    {
        if (!mOwner.mLightmap.valid()) return;
        mOwner.uploadRegion(mOwner.mLightmap, box, l8.data.data(), 1, false);
    }

    bool createColourMap(std::uint16_t size) override
    {
        ct::Vector<std::uint8_t> init;
        init.resize(static_cast<std::size_t>(size) * size * 4);
        memset(init.data(), 255, init.size());
        return mOwner.createMapTexture(mOwner.mColourMap, prisma::TextureFormat::RGBA8Srgb, size, 0,
                true, prisma::kTextureSampled, init.data(), "terrain colour map");
    }
    void destroyColourMap() override
    {
        mOwner.destroyMapTexture(mOwner.mColourMap);
    }
    void writeColourMap(const TerrainRect& box, const TerrainPixelBox& rgb) override
    {
        if (!mOwner.mColourMap.valid()) return;
        ct::Vector<std::uint8_t> rgba;
        rgba.resize(static_cast<std::size_t>(rgb.width) * rgb.height * 4);
        for (std::size_t i = 0; i < static_cast<std::size_t>(rgb.width) * rgb.height; ++i)
        {
            rgba[i * 4] = rgb.data[i * 3];
            rgba[i * 4 + 1] = rgb.data[i * 3 + 1];
            rgba[i * 4 + 2] = rgb.data[i * 3 + 2];
            rgba[i * 4 + 3] = 255;
        }
        mOwner.uploadRegion(mOwner.mColourMap, box, rgba.data(), 4, true);
    }

    bool createCompositeMap(std::uint16_t size) override
    {
        ct::Vector<std::uint8_t> init;
        init.resize(static_cast<std::size_t>(size) * size * 4);
        memset(init.data(), 0, init.size());
        return mOwner.createMapTexture(mOwner.mCompositeMap, prisma::TextureFormat::RGBA8Srgb, size,
                0, true, prisma::kTextureSampled | prisma::kTextureRenderTarget, init.data(),
                "terrain composite map");
    }
    void destroyCompositeMap() override
    {
        mOwner.destroyMapTexture(mOwner.mCompositeMap);
    }
    void renderCompositeMap(const TerrainRect& imageRect) override
    {
        mOwner.mPendingComposite.merge(imageRect);
    }

    bool createBlendTexture(std::uint8_t index, std::uint16_t size) override
    {
        while (mOwner.mBlendTextures.size() <= index) mOwner.mBlendTextures.push_back(TextureId());
        ct::Vector<std::uint8_t> init;
        init.resize(static_cast<std::size_t>(size) * size * 4);
        memset(init.data(), 0, init.size());
        return mOwner.createMapTexture(mOwner.mBlendTextures[index], prisma::TextureFormat::RGBA8,
                size, 1, false, prisma::kTextureSampled, init.data(), "terrain blend map");
    }
    void destroyBlendTexture(std::uint8_t index) override
    {
        if (index >= mOwner.mBlendTextures.size()) return;
        mOwner.destroyMapTexture(mOwner.mBlendTextures[index]);
        if (index + 1u == mOwner.mBlendTextures.size()) mOwner.mBlendTextures.pop_back();
    }
    void writeBlendTexture(std::uint8_t index, const TerrainRect& box,
            const TerrainPixelBox& rgba) override
    {
        if (index >= mOwner.mBlendTextures.size() || !mOwner.mBlendTextures[index].valid()) return;
        mOwner.uploadRegion(mOwner.mBlendTextures[index], box, rgba.data.data(), 4, false);
    }

private:
    Terrain3D& mOwner;
};

class Terrain3D::GeometryTarget : public TerrainGeometryTarget
{
public:
    explicit GeometryTarget(Terrain3D& owner)
            : mOwner(owner)
    {
    }

    void writeVertexRange(const TerrainQuadTreeNode* owner, std::uint32_t first,
            std::uint32_t count, const TerrainVertexData& source) override
    {
        mOwner.writeVertices(owner, first, count, source);
    }

    void geometryUpdated(const TerrainRect& rect) override
    {
        mOwner.refreshNodeBoxes(rect);
    }

private:
    Terrain3D& mOwner;
};

Terrain3D::Terrain3D()
        : Node3D(),
          mTerrain(),
          mMapTarget(nullptr),
          mGeometryTarget(nullptr),
          mUploadScratch(),
          mVertexBytes(0),
          mVertexRanges(0),
          mOwnersUpdated(0),
          mOwnersTouched(),
          mNormalMap(),
          mLightmap(),
          mColourMap(),
          mCompositeMap(),
          mBlendTextures(),
          mCompositeMaterial(),
          mComposite(),
          mPendingComposite(),
          mLightmapEnabled(true),
          mCompositeEnabled(true),
          mLayerNormalMapping(true),
          mParallaxMode(1),
          mParallaxScale(0.04f),
          mLightmapNear(true),
          mDetailModulation(false),
          mDetailStrength(1.0f),
          mUploadedBytes(0),
          mUploadCount(0),
          mCompositeBakes(0),
          mCompositeBakeTexels(0),
          mLastMillis(0),
          mNodesComposite(0),
          mLoaded(false),
          mCastShadows(true),
          mConnected(false),
          mDefaultMaterial(),
          mMaterial(),
          mObservers(),
          mNodes(),
          mNodeIndex(),
          mIndexPools(),
          mPoolMeshes(),
          mNodeMeshes(),
          mVertexMeshes(),
          mNodesDrawn(0),
          mNodesAtLod(),
          mTrianglesDrawn(0)
{
    setNotifyTransform(true);
    mTerrain._setMorphRequired(true);
    for (std::uint32_t i = 0; i < kLayerSlots; ++i)
        for (std::uint32_t j = 0; j < 2; ++j) mLayerOwned[i][j] = false;
    for (std::uint32_t i = 0; i < kLayerSlots; ++i) mLayerWorldSizes[i] = 0.0f;

    if (MaterialStore* materials = MaterialStore::singleton())
    {
        mDefaultMaterial = materials->create(kDefaultMaterialName);
        materials->setTerrainMorph(mDefaultMaterial, true);
        mMaterial = mDefaultMaterial;
    }
}

Terrain3D::~Terrain3D()
{
    disconnectPreDraw();
    unload();
    ct::Vector<TerrainObserver*> copy = mObservers;
    mObservers.clear();
    for (std::size_t i = 0; i < copy.size(); ++i) copy[i]->onTerrainDestroyed();
    if (MaterialStore* materials = MaterialStore::singleton())
        if (mDefaultMaterial.valid()) materials->destroy(mDefaultMaterial);
}

bool Terrain3D::load(const TerrainImportData& importData)
{
    unload();

    MeshStore* meshes = MeshStore::singleton();
    TextureStore* textures = TextureStore::singleton();
    if (!meshes || !textures)
    {
        log_error("terrain: there is no mesh store");
        return false;
    }
    if (importData.terrainAlign != TerrainAlignment::AlignXZ)
    {
        log_error("terrain: the node draws only the X-Z alignment");
        return false;
    }
    const std::uint32_t maxSize = textures->driver()->caps().maxTextureSize;
    if (importData.terrainSize > maxSize || importData.layerBlendMapSize > maxSize ||
            importData.lightMapSize > maxSize || importData.compositeMapSize > maxSize ||
            importData.defaultGlobalColourMapSize > maxSize)
    {
        log_error("terrain: a derived map is larger than the texture size limit");
        return false;
    }
    if (importData.layerList.size() > mTerrain.getMaxLayers())
    {
        log_error("terrain: too many layers for the texture slots");
        return false;
    }
    if (!mTerrain.prepare(importData)) return false;
    if (mTerrain.getNumLodLevels() > kMaxLodLevels)
    {
        log_error("terrain: too many detail levels for the morph packing");
        mTerrain.unprepare();
        return false;
    }

    mTerrain.setPosition(getGlobalPosition());
    if (!createGpuData())
    {
        log_error("terrain: could not create the geometry for every node");
        releaseGpuData();
        mTerrain.unprepare();
        return false;
    }
    mTerrain.freeCpuVertexData();
    mTerrain.load();
    mNodesAtLod.resize(mTerrain.getNumLodLevels());
    for (std::size_t i = 0; i < mNodesAtLod.size(); ++i) mNodesAtLod[i] = 0;

    mMapTarget = new MapTarget(*this);
    mTerrain.setMapTarget(mMapTarget);
    mGeometryTarget = new GeometryTarget(*this);
    mTerrain.setGeometryTarget(mGeometryTarget);
    mLoaded = true;
    if (!mObservers.empty()) notifyObservers(getAabb());
    resetUploadStats();
    mTerrain.loadMaps(true, mLightmapEnabled);
    loadLayerTextures();
    if (mCompositeEnabled && createComposite()) mTerrain._setCompositeMapRequired(true);
    refreshMaterials();
    mLastMillis = monotonicMilliseconds();
    return true;
}

void Terrain3D::unload()
{
    const bool wasLoaded = mLoaded;
    releaseGpuData();
    mTerrain.unprepare();
    mTerrain.setMapTarget(nullptr);
    mTerrain.setGeometryTarget(nullptr);
    releaseMaps();
    delete mMapTarget;
    mMapTarget = nullptr;
    delete mGeometryTarget;
    mGeometryTarget = nullptr;
    releaseLayerTextures();
    mComposite.destroy();
    if (MaterialStore* materials = MaterialStore::singleton())
    {
        if (mCompositeMaterial.valid()) materials->destroy(mCompositeMaterial);
        if (mMaterial.valid()) materials->setTerrainMaps(mMaterial, TerrainMaterialData());
    }
    mCompositeMaterial = MaterialId();
    mPendingComposite = TerrainRect();
    mLoaded = false;
    mNodesDrawn = 0;
    mNodesComposite = 0;
    mTrianglesDrawn = 0;
    mNodesAtLod.clear();
    if (wasLoaded && !mObservers.empty())
        notifyObservers(Math::Box(Math::Vec3(-FLT_MAX, -FLT_MAX, -FLT_MAX),
                Math::Vec3(FLT_MAX, FLT_MAX, FLT_MAX)));
}

bool Terrain3D::isLoaded() const
{
    return mLoaded;
}

Terrain& Terrain3D::getTerrain()
{
    return mTerrain;
}

const Terrain& Terrain3D::getTerrain() const
{
    return mTerrain;
}

void Terrain3D::collectNodes(TerrainQuadTreeNode* node)
{
    NodeGpu gpu;
    gpu.node = node;
    mNodeIndex[nodeKey(node)] = static_cast<std::uint32_t>(mNodes.size());
    mNodes.push_back(gpu);
    if (!node->isLeaf())
        for (unsigned short i = 0; i < 4; ++i) collectNodes(node->getChild(i));
}

bool Terrain3D::createVertexMesh(NodeGpu& gpu)
{
    MeshStore* meshes = MeshStore::singleton();
    const TerrainVertexData* data = gpu.node->getVertexDataRecord();
    if (!data || !data->cpuDataPresent) return false;

    SurfaceData surface;
    surface.topology = prisma::Topology::TriangleStrip;
    surface.positions = data->positions;
    surface.normals.resize(data->positions.size());
    for (std::size_t i = 0; i < surface.normals.size(); ++i)
        surface.normals[i] = Math::Vec3(0.0f, 1.0f, 0.0f);
    surface.uvs = data->uvs;
    surface.colors.resize(data->deltas.size());
    for (std::size_t i = 0; i < data->deltas.size(); ++i)
        surface.colors[i] = Math::Vec4(data->deltas[i].x, data->deltas[i].y, 0.0f, 1.0f);

    gpu.vertexMesh = meshes->create(kVertexMeshName);
    mVertexMeshes.push_back(gpu.vertexMesh);
    return meshes->addSurface(gpu.vertexMesh, surface, kVertexMeshName, true);
}

bool Terrain3D::createLodMeshes(NodeGpu& gpu)
{
    MeshStore* meshes = MeshStore::singleton();
    TerrainQuadTreeNode* node = gpu.node;
    TerrainQuadTreeNode* owner = node->getNodeWithVertexData();
    const std::uint32_t* ownerIndex = mNodeIndex.find(nodeKey(owner));
    if (!owner || !ownerIndex) return false;
    const MeshId vertexMesh = mNodes[*ownerIndex].vertexMesh;
    if (!meshes->valid(vertexMesh)) return false;

    const Math::Vec3 offset = node->getLocalCentre() - owner->getLocalCentre();
    Math::Box box = node->getBoundingBox();
    box.min = box.min + offset;
    box.max = box.max + offset;

    const std::uint16_t lodCount = node->getLodCount();
    gpu.lodMeshes.resize(lodCount);
    gpu.lodTriangles.resize(lodCount);
    for (std::uint16_t lod = 0; lod < lodCount; ++lod)
    {
        TerrainIndexKey key;
        if (!node->getIndexKey(lod, key)) return false;
        const std::uint64_t packed = key.pack();

        MeshId pool;
        const MeshId* found = mIndexPools.find(packed);
        std::uint32_t indexCount =
                static_cast<std::uint32_t>(Terrain::getNumIndexesForBatchSize(key.batchSize));
        if (found)
        {
            pool = *found;
        }
        else
        {
            ct::Vector<std::uint16_t> indices;
            indices.resize(indexCount);
            Terrain::populateIndexBuffer(indices.data(), key.batchSize, key.vdatasize,
                    key.vertexIncrement, key.xoffset, key.yoffset, key.numSkirtRowsCols,
                    key.skirtRowColSkip);
            pool = meshes->create(kPoolMeshName);
            mPoolMeshes.push_back(pool);
            mIndexPools[packed] = pool;
            if (meshes->addIndexedSurface(pool, vertexMesh, 0, indices.data(), indexCount, box,
                        kPoolMeshName) < 0)
                return false;
        }

        const MeshId mesh = meshes->create(kNodeMeshName);
        mNodeMeshes.push_back(mesh);
        if (meshes->addSharedSurface(mesh, vertexMesh, 0, pool, 0, box) < 0) return false;
        gpu.lodMeshes[lod] = mesh;
        gpu.lodTriangles[lod] = indexCount > 2 ? indexCount - 2 : 0;
    }
    return true;
}

bool Terrain3D::createGpuData()
{
    collectNodes(mTerrain.getQuadTree());

    for (std::size_t i = 0; i < mNodes.size(); ++i)
        if (mNodes[i].node->ownsVertexData() && !createVertexMesh(mNodes[i])) return false;

    for (std::size_t i = 0; i < mNodes.size(); ++i)
    {
        NodeGpu& gpu = mNodes[i];
        if (!createLodMeshes(gpu)) return false;

        MeshInstance3D* instance = createChild<MeshInstance3D>();
        instance->setInternal(true);
        gpu.instance = instance;
        instance->setPosition(gpu.node->getNodeWithVertexData()->getLocalCentre());
        instance->setMaterialOverride(mMaterial);
        instance->setCastShadows(mCastShadows);
        instance->setVisible(false);
    }
    return true;
}

void Terrain3D::releaseGpuData()
{
    for (std::size_t i = 0; i < mNodes.size(); ++i)
    {
        if (!mNodes[i].instance) continue;
        mNodes[i].instance->setMesh(MeshId());
        NodeFactory::destroy(mNodes[i].instance);
        mNodes[i].instance = nullptr;
    }
    mNodes.clear();
    mNodeIndex.clear();
    mIndexPools.clear();

    if (MeshStore* meshes = MeshStore::singleton())
    {
        for (std::size_t i = mNodeMeshes.size(); i > 0; --i) meshes->destroy(mNodeMeshes[i - 1]);
        for (std::size_t i = mPoolMeshes.size(); i > 0; --i) meshes->destroy(mPoolMeshes[i - 1]);
        for (std::size_t i = mVertexMeshes.size(); i > 0; --i)
            meshes->destroy(mVertexMeshes[i - 1]);
    }
    mNodeMeshes.clear();
    mPoolMeshes.clear();
    mVertexMeshes.clear();
}

void Terrain3D::setMaterial(MaterialId material)
{
    MaterialStore* materials = MaterialStore::singleton();
    if (materials && mMaterial.valid() && mMaterial != mDefaultMaterial && mMaterial != material)
        materials->setTerrainMaps(mMaterial, TerrainMaterialData());
    mMaterial = material.valid() ? material : mDefaultMaterial;
    if (materials && mMaterial.valid()) materials->setTerrainMorph(mMaterial, true);
    refreshMaterials();
    applyMaterial();
}

MaterialId Terrain3D::getMaterial() const
{
    return mMaterial;
}

MaterialId Terrain3D::getAssignedMaterial() const
{
    return mMaterial == mDefaultMaterial ? MaterialId() : mMaterial;
}

void Terrain3D::applyMaterial()
{
    for (std::size_t i = 0; i < mNodes.size(); ++i) applyNodeMaterial(mNodes[i]);
}

void Terrain3D::applyNodeMaterial(NodeGpu& gpu)
{
    if (!gpu.instance) return;
    gpu.instance->setMaterialOverride(
            gpu.composite && mCompositeMaterial.valid() ? mCompositeMaterial : mMaterial);
}

void Terrain3D::setMaxPixelError(float maxPixelError)
{
    mTerrain.setMaxPixelError(maxPixelError);
}

float Terrain3D::getMaxPixelError() const
{
    return mTerrain.getMaxPixelError();
}

void Terrain3D::setCastShadows(bool cast)
{
    mCastShadows = cast;
    for (std::size_t i = 0; i < mNodes.size(); ++i)
        if (mNodes[i].instance) mNodes[i].instance->setCastShadows(cast);
}

bool Terrain3D::getCastShadows() const
{
    return mCastShadows;
}

void Terrain3D::syncPosition() const
{
    const_cast<Terrain&>(mTerrain).setPosition(getGlobalPosition());
}

float Terrain3D::getWorldHeightAt(const Math::Vec3& position) const
{
    syncPosition();
    return mTerrain.getHeightAtWorldPosition(position) + mTerrain.getPosition().y;
}

float Terrain3D::getWorldSlopeAt(const Math::Vec3& position) const
{
    syncPosition();
    return mTerrain.getSlopeAtWorldPosition(position);
}

void Terrain3D::addObserver(TerrainObserver* observer)
{
    if (!observer) return;
    for (std::size_t i = 0; i < mObservers.size(); ++i)
        if (mObservers[i] == observer) return;
    mObservers.push_back(observer);
}

void Terrain3D::removeObserver(TerrainObserver* observer)
{
    for (std::size_t i = 0; i < mObservers.size(); ++i)
    {
        if (mObservers[i] != observer) continue;
        mObservers.erase(mObservers.begin() + static_cast<std::ptrdiff_t>(i));
        return;
    }
}

int Terrain3D::getObserverCount() const
{
    return static_cast<int>(mObservers.size());
}

Math::Box Terrain3D::worldBoxOf(const TerrainRect& rect) const
{
    syncPosition();
    const std::uint32_t last = static_cast<std::uint32_t>(mTerrain.getSize() - 1);
    const std::uint32_t left = static_cast<std::uint32_t>(rect.left);
    const std::uint32_t top = static_cast<std::uint32_t>(rect.top);
    const std::uint32_t right = static_cast<std::uint32_t>(rect.right - 1) < last
                                        ? static_cast<std::uint32_t>(rect.right - 1)
                                        : last;
    const std::uint32_t bottom = static_cast<std::uint32_t>(rect.bottom - 1) < last
                                         ? static_cast<std::uint32_t>(rect.bottom - 1)
                                         : last;
    Math::Vec3 a;
    Math::Vec3 b;
    mTerrain.getPoint(left, top, 0.0f, a);
    mTerrain.getPoint(right, bottom, 0.0f, b);
    const Math::Vec3& origin = mTerrain.getPosition();
    const float cell = mTerrain.getScale();
    const float minX = (a.x < b.x ? a.x : b.x) + origin.x - cell;
    const float maxX = (a.x > b.x ? a.x : b.x) + origin.x + cell;
    const float minZ = (a.z < b.z ? a.z : b.z) + origin.z - cell;
    const float maxZ = (a.z > b.z ? a.z : b.z) + origin.z + cell;
    return Math::Box(Math::Vec3(minX, origin.y + mTerrain.getMinHeight(), minZ),
            Math::Vec3(maxX, origin.y + mTerrain.getMaxHeight(), maxZ));
}

void Terrain3D::notifyObservers(const Math::Box& worldBox)
{
    ct::Vector<TerrainObserver*> copy = mObservers;
    for (std::size_t i = 0; i < copy.size(); ++i) copy[i]->onTerrainChanged(worldBox);
}

bool Terrain3D::rayIntersects(const Math::Ray& ray, Math::Vec3& position) const
{
    syncPosition();
    const TerrainRayResult result = mTerrain.rayIntersects(ray);
    if (result.hit) position = result.position;
    return result.hit;
}

Math::Box Terrain3D::getAabb() const
{
    syncPosition();
    return mTerrain.getWorldAABB();
}

void Terrain3D::updateLod(const TerrainLodView& view)
{
    if (!mLoaded) return;

    mTerrain.setPosition(getGlobalPosition());
    mTerrain.calculateCurrentLod(view);

    mNodesDrawn = 0;
    mNodesComposite = 0;
    mTrianglesDrawn = 0;
    for (std::size_t i = 0; i < mNodesAtLod.size(); ++i) mNodesAtLod[i] = 0;

    for (std::size_t i = 0; i < mNodes.size(); ++i)
    {
        NodeGpu& gpu = mNodes[i];
        const int lod = gpu.node->getCurrentLod();
        if (lod < 0)
        {
            if (gpu.shown)
            {
                gpu.instance->setVisible(false);
                gpu.shown = false;
            }
            continue;
        }

        if (gpu.shownLod != lod)
        {
            gpu.instance->setMesh(gpu.lodMeshes[static_cast<std::size_t>(lod)]);
            gpu.shownLod = lod;
        }
        const bool composite = mCompositeMaterial.valid() && gpu.node->getMaterialLodIndex() == 1;
        if (composite != gpu.composite)
        {
            gpu.composite = composite;
            applyNodeMaterial(gpu);
        }
        if (composite) ++mNodesComposite;
        const float packed = packMorph(gpu.node->getLodMorph());
        if (packed != gpu.shownMorph)
        {
            gpu.instance->setColor(Math::Vec4(1.0f, 1.0f, 1.0f, packed));
            gpu.shownMorph = packed;
        }
        if (!gpu.shown)
        {
            gpu.instance->setVisible(true);
            gpu.shown = true;
        }

        ++mNodesDrawn;
        mTrianglesDrawn += gpu.lodTriangles[static_cast<std::size_t>(lod)];
        const std::size_t global = static_cast<std::size_t>(lod + gpu.node->getBaseLod());
        if (global < mNodesAtLod.size()) ++mNodesAtLod[global];
    }
}

bool Terrain3D::updateFromCamera()
{
    if (!mLoaded) return false;
    Viewport* viewport = Viewport::findViewport(this);
    if (!viewport) return false;
    Camera3D* camera = viewport->getCamera3D();
    if (!camera) return false;

    const Math::Frustum frustum = Math::Frustum::FromViewProjection(
            camera->getCameraProjection() * camera->getGlobalTransform().InverseAffine());
    TerrainLodView view;
    view.position = camera->getGlobalPosition();
    view.frustum = &frustum;
    view.fovY = camera->getFov() * Math::DEG2RAD;
    view.viewportHeight = static_cast<float>(viewport->getHeight());
    updateLod(view);
    return true;
}

int Terrain3D::getNodeCount() const
{
    return static_cast<int>(mNodes.size());
}

int Terrain3D::getNodesDrawn() const
{
    return mNodesDrawn;
}

int Terrain3D::getNodesDrawnAtLod(int lod) const
{
    if (lod < 0 || static_cast<std::size_t>(lod) >= mNodesAtLod.size()) return 0;
    return mNodesAtLod[static_cast<std::size_t>(lod)];
}

int Terrain3D::getLodLevelCount() const
{
    return static_cast<int>(mNodesAtLod.size());
}

std::uint32_t Terrain3D::getTrianglesDrawn() const
{
    return mTrianglesDrawn;
}

int Terrain3D::getVertexMeshCount() const
{
    return static_cast<int>(mVertexMeshes.size());
}

int Terrain3D::getIndexPoolCount() const
{
    return static_cast<int>(mPoolMeshes.size());
}

int Terrain3D::getNodeMeshCount() const
{
    return static_cast<int>(mNodeMeshes.size());
}

MeshInstance3D* Terrain3D::getNodeInstance(const TerrainQuadTreeNode* node) const
{
    const std::uint32_t* index = mNodeIndex.find(nodeKey(node));
    return index ? mNodes[*index].instance : nullptr;
}

MeshId Terrain3D::getNodeMesh(const TerrainQuadTreeNode* node, int lod) const
{
    const std::uint32_t* index = mNodeIndex.find(nodeKey(node));
    if (!index || lod < 0 || static_cast<std::size_t>(lod) >= mNodes[*index].lodMeshes.size())
        return MeshId();
    return mNodes[*index].lodMeshes[static_cast<std::size_t>(lod)];
}

void Terrain3D::notificationHandler(int what)
{
    Node3D::notificationHandler(what);

    switch (what)
    {
        case kNotificationEnterTree:
            connectPreDraw();
            break;
        case kNotificationExitTree:
            disconnectPreDraw();
            break;
        case kNotificationTransformChanged:
            if (mLoaded) mTerrain.setPosition(getGlobalPosition());
            break;
        default:
            break;
    }
}

void Terrain3D::connectPreDraw()
{
    RenderingServer* server = RenderingServer::singleton();
    if (!server || mConnected) return;
    server->framePreDrawConnect(this,
            [this]()
            {
                processDeferred();
                updateFromCamera();
                flushCompositeMap();
            });
    mConnected = true;
}

void Terrain3D::disconnectPreDraw()
{
    RenderingServer* server = RenderingServer::singleton();
    if (server && mConnected) server->framePreDrawDisconnect(this);
    mConnected = false;
}

bool Terrain3D::createMapTexture(TextureId& out, prisma::TextureFormat format, std::uint32_t width,
        std::uint32_t mips, bool srgb, std::uint32_t usage, const void* data, const char* name)
{
    TextureStore* textures = TextureStore::singleton();
    if (!textures) return false;
    destroyMapTexture(out);
    prisma::TextureDesc desc;
    desc.format = format;
    desc.width = width;
    desc.height = width;
    desc.mipLevels = mips;
    desc.usage = usage;
    desc.data = data;
    desc.generateMipmaps = mips != 1;
    desc.debugName = name;
    out = textures->create(desc, srgb);
    return out.valid();
}

void Terrain3D::destroyMapTexture(TextureId& texture)
{
    if (!texture.valid()) return;
    if (TextureStore* textures = TextureStore::singleton()) textures->destroy(texture);
    texture = TextureId();
}

void Terrain3D::uploadRegion(TextureId texture, const TerrainRect& box, const void* data,
        std::uint32_t texelBytes, bool regenerateMips)
{
    TextureStore* textures = TextureStore::singleton();
    if (!textures || box.isNull()) return;
    const TextureInfo* info = textures->get(texture);
    if (!info) return;
    prisma::TextureRegion region;
    region.x = static_cast<std::uint32_t>(box.left);
    region.y = static_cast<std::uint32_t>(box.top);
    region.width = static_cast<std::uint32_t>(box.width());
    region.height = static_cast<std::uint32_t>(box.height());
    textures->driver()->updateTextureRegion(info->gpu, region, data);
    if (regenerateMips && info->mipLevels > 1) textures->driver()->generateMipmaps(info->gpu);
    mUploadedBytes += static_cast<std::uint64_t>(region.width) * region.height * texelBytes;
    ++mUploadCount;
}

void Terrain3D::releaseMaps()
{
    destroyMapTexture(mNormalMap);
    destroyMapTexture(mLightmap);
    destroyMapTexture(mColourMap);
    destroyMapTexture(mCompositeMap);
    for (std::size_t i = 0; i < mBlendTextures.size(); ++i) destroyMapTexture(mBlendTextures[i]);
    mBlendTextures.clear();
}

void Terrain3D::releaseLayerTextures()
{
    TextureStore* textures = TextureStore::singleton();
    for (std::uint32_t i = 0; i < kLayerSlots; ++i)
    {
        for (std::uint32_t j = 0; j < 2; ++j)
        {
            if (mLayerOwned[i][j] && textures) textures->destroy(mLayerTextures[i][j]);
            mLayerTextures[i][j] = TextureId();
            mLayerOwned[i][j] = false;
        }
    }
}

bool Terrain3D::createComposite()
{
    TextureStore* textures = TextureStore::singleton();
    if (!textures) return false;
    if (!mComposite.create(textures->driver(), textures)) return false;
    MaterialStore* materials = MaterialStore::singleton();
    if (!materials) return false;
    mCompositeMaterial = materials->create(kCompositeMaterialName);
    materials->setShading(mCompositeMaterial, MaterialShading::Unshaded);
    materials->setTerrainMorph(mCompositeMaterial, true);
    return true;
}

void Terrain3D::loadLayerTextures()
{
    TextureStore* textures = TextureStore::singleton();
    if (!textures) return;
    const std::uint32_t count =
            mTerrain.getLayerCount() < kLayerSlots ? mTerrain.getLayerCount() : kLayerSlots;
    for (std::uint32_t layer = 0; layer < count; ++layer)
    {
        for (std::uint32_t sampler = 0; sampler < 2; ++sampler)
        {
            if (mLayerTextures[layer][sampler].valid()) continue;
            const ct::String& name = mTerrain.getLayerTextureName(static_cast<std::uint8_t>(layer),
                    static_cast<std::uint8_t>(sampler));
            if (name.empty()) continue;
            const TextureId loaded = textures->find(name);
            if (loaded.valid())
            {
                mLayerTextures[layer][sampler] = loaded;
                mLayerOwned[layer][sampler] = false;
                continue;
            }
            const TextureId id = textures->createFromFile(name, sampler == 0, true);
            if (!id.valid())
            {
                log_error("terrain: could not load the layer texture %s", name.c_str());
                continue;
            }
            mLayerTextures[layer][sampler] = id;
            mLayerOwned[layer][sampler] = true;
        }
    }
}

void Terrain3D::buildMaterialData(TerrainMaterialData& data) const
{
    data = TerrainMaterialData();
    data.enabled = true;
    data.normalMap = mNormalMap;
    data.lightmap = mLightmap;
    data.colourMap = mColourMap;
    data.blendMap = mBlendTextures.empty() ? TextureId() : mBlendTextures[0];
    const std::uint32_t count =
            mTerrain.getLayerCount() < kLayerSlots ? mTerrain.getLayerCount() : kLayerSlots;
    data.layerCount = count;
    for (std::uint32_t i = 0; i < count; ++i)
    {
        data.layerDiffuse[i] = mLayerTextures[i][0];
        data.layerNormal[i] = mLayerTextures[i][1];
        data.uvMultiplier[i] = mTerrain.getLayerUVMultiplier(static_cast<std::uint8_t>(i));
    }
    data.normalMapping = mLayerNormalMapping && count > 0 && mLayerTextures[0][1].valid();
    data.parallax = data.normalMapping ? mParallaxMode : 0;
    data.parallaxScale = mParallaxScale;
    data.useLightmap = mLightmapNear && mLightmap.valid();
    data.useColourMap = mColourMap.valid() && mTerrain.getGlobalColourMapEnabled();
    data.detailModulation = mDetailModulation;
    data.detailStrength = mDetailStrength;
}

void Terrain3D::writeVertices(const TerrainQuadTreeNode* owner, std::uint32_t first,
        std::uint32_t count, const TerrainVertexData& source)
{
    MeshStore* meshes = MeshStore::singleton();
    const std::uint32_t* index = mNodeIndex.find(nodeKey(owner));
    if (!meshes || !index || count == 0) return;
    const MeshId mesh = mNodes[*index].vertexMesh;
    const MeshInfo* info = meshes->get(mesh);
    if (!info || info->surfaces.empty()) return;
    const SurfaceInfo& surface = info->surfaces[0];
    const std::uint32_t stride = surface.vertexStride;
    const std::uint32_t positionOffset =
            MeshStore::attributeOffset(surface.attributeMask, kVertexPosition);
    const std::uint32_t normalOffset =
            MeshStore::attributeOffset(surface.attributeMask, kVertexNormal);
    const std::uint32_t colorOffset =
            MeshStore::attributeOffset(surface.attributeMask, kVertexColor);
    const std::uint32_t uvOffset = MeshStore::attributeOffset(surface.attributeMask, kVertexUv);

    mUploadScratch.resize(static_cast<std::size_t>(count) * stride);
    for (std::uint32_t i = 0; i < count; ++i)
    {
        std::uint8_t* vertex = &mUploadScratch[static_cast<std::size_t>(i) * stride];
        const std::size_t at = static_cast<std::size_t>(first) + i;
        const float position[3] = { source.positions[at].x, source.positions[at].y,
            source.positions[at].z };
        const float normal[3] = { 0.0f, 1.0f, 0.0f };
        const float color[4] = { source.deltas[at].x, source.deltas[at].y, 0.0f, 1.0f };
        const float uv[2] = { source.uvs[at].x, source.uvs[at].y };
        memcpy(vertex + positionOffset, position, sizeof(position));
        memcpy(vertex + normalOffset, normal, sizeof(normal));
        memcpy(vertex + colorOffset, color, sizeof(color));
        memcpy(vertex + uvOffset, uv, sizeof(uv));
    }
    if (meshes->updateVertices(mesh, 0, first * stride, mUploadScratch.data(), count * stride))
    {
        bool known = false;
        for (std::size_t i = 0; i < mOwnersTouched.size(); ++i)
            if (mOwnersTouched[i] == *index) known = true;
        if (!known) mOwnersTouched.push_back(*index);
        mVertexBytes += static_cast<std::uint64_t>(count) * stride;
        ++mVertexRanges;
    }
}

void Terrain3D::refreshNodeBoxes(const TerrainRect& rect)
{
    MeshStore* meshes = MeshStore::singleton();
    if (!meshes) return;
    for (std::size_t i = 0; i < mNodes.size(); ++i)
    {
        NodeGpu& gpu = mNodes[i];
        TerrainQuadTreeNode* node = gpu.node;
        if (!node->rectIntersectsNode(rect) || gpu.lodMeshes.empty()) continue;
        TerrainQuadTreeNode* owner = node->getNodeWithVertexData();
        Math::Box box = node->getBoundingBox();
        if (box.min.x > box.max.x) continue;
        const Math::Vec3 offset = node->getLocalCentre() - owner->getLocalCentre();
        box.min = box.min + offset;
        box.max = box.max + offset;
        for (std::size_t lod = 0; lod < gpu.lodMeshes.size(); ++lod)
        {
            const MeshInfo* info = meshes->get(gpu.lodMeshes[lod]);
            if (!info || info->surfaces.empty()) continue;
            const Math::Box& have = info->surfaces[0].aabb;
            if (have.min.x == box.min.x && have.min.y == box.min.y && have.min.z == box.min.z &&
                    have.max.x == box.max.x && have.max.y == box.max.y && have.max.z == box.max.z)
                continue;
            meshes->setSurfaceAabb(gpu.lodMeshes[lod], 0, box);
        }
    }
}

void Terrain3D::update()
{
    if (!mLoaded) return;
    mTerrain.setPosition(getGlobalPosition());
    mOwnersTouched.clear();
    const TerrainRect changed = mTerrain.getDirtyGeometryRect();
    mTerrain.update();
    mOwnersUpdated = static_cast<std::uint32_t>(mOwnersTouched.size());
    if (!changed.isNull() && !mObservers.empty()) notifyObservers(worldBoxOf(changed));
}

std::uint64_t Terrain3D::getVertexUploadBytes() const
{
    return mVertexBytes;
}

std::uint32_t Terrain3D::getVertexUploadRanges() const
{
    return mVertexRanges;
}

std::uint32_t Terrain3D::getLastUpdateOwners() const
{
    return mOwnersUpdated;
}

void Terrain3D::resetEditStats()
{
    mVertexBytes = 0;
    mVertexRanges = 0;
    mOwnersUpdated = 0;
    mOwnersTouched.clear();
}

void Terrain3D::refreshMaterials()
{
    MaterialStore* materials = MaterialStore::singleton();
    if (!materials) return;
    TerrainMaterialData data;
    buildMaterialData(data);
    if (mMaterial.valid()) materials->setTerrainMaps(mMaterial, data);
    if (mCompositeMaterial.valid())
    {
        materials->setTexture(mCompositeMaterial, MaterialTexture::Albedo, mCompositeMap);
    }
}

bool Terrain3D::flushCompositeMap()
{
    if (!mLoaded || mPendingComposite.isNull() || !mCompositeMap.valid() || !mComposite.isCreated())
        return false;
    RenderingServer* server = RenderingServer::singleton();
    if (!server) return false;
    server->openFrame();

    TerrainCompositeInputs inputs;
    buildMaterialData(inputs.maps);
    inputs.maps.useLightmap = mLightmap.valid();
    inputs.lightDirection = mTerrain.getLightMapDirection();
    inputs.ambient = mTerrain.getCompositeMapAmbient();
    inputs.diffuse = mTerrain.getCompositeMapDiffuse();

    const TerrainRect rect = mPendingComposite;
    mPendingComposite = TerrainRect();
    if (!mComposite.render(inputs, mCompositeMap, rect)) return false;
    ++mCompositeBakes;
    mCompositeBakeTexels += static_cast<std::uint64_t>(rect.width()) * rect.height();
    return true;
}

void Terrain3D::rebakeComposite()
{
    mTerrain._dirtyCompositeMapRect(TerrainRect(0, 0, mTerrain.getSize(), mTerrain.getSize()));
    mTerrain.updateCompositeMap();
}

void Terrain3D::processDeferred()
{
    if (!mLoaded) return;
    const unsigned long now = monotonicMilliseconds();
    mTerrain.advanceDeferredUpdates(now - mLastMillis);
    mLastMillis = now;
    if (mTerrain.isMaterialDirty() || mTerrain.isMaterialParamsDirty())
    {
        loadLayerTextures();
        refreshMaterials();
        mTerrain.clearMaterialDirty();
    }
}

void Terrain3D::setLightmapEnabled(bool enabled)
{
    mLightmapEnabled = enabled;
}

bool Terrain3D::getLightmapEnabled() const
{
    return mLightmapEnabled;
}

void Terrain3D::setCompositeMapEnabled(bool enabled)
{
    mCompositeEnabled = enabled;
}

bool Terrain3D::getCompositeMapEnabled() const
{
    return mCompositeEnabled;
}

void Terrain3D::setLayerNormalMappingEnabled(bool enabled)
{
    mLayerNormalMapping = enabled;
    if (mLoaded)
    {
        refreshMaterials();
        rebakeComposite();
    }
}

bool Terrain3D::getLayerNormalMappingEnabled() const
{
    return mLayerNormalMapping;
}

void Terrain3D::setLayerParallax(std::uint32_t mode, float scale)
{
    mParallaxMode = mode > 2 ? 2 : mode;
    mParallaxScale = scale;
    if (mLoaded) refreshMaterials();
}

std::uint32_t Terrain3D::getLayerParallax() const
{
    return mParallaxMode;
}

void Terrain3D::setLayerParallaxMode(std::uint32_t mode)
{
    setLayerParallax(mode, mParallaxScale);
}

void Terrain3D::setLayerParallaxScale(float scale)
{
    setLayerParallax(mParallaxMode, scale);
}

float Terrain3D::getLayerParallaxScale() const
{
    return mParallaxScale;
}

void Terrain3D::setDetailModulation(bool enabled, float strength)
{
    mDetailModulation = enabled;
    mDetailStrength = strength < 0.0f ? 0.0f : (strength > 1.0f ? 1.0f : strength);
    if (mLoaded)
    {
        refreshMaterials();
        rebakeComposite();
    }
}

bool Terrain3D::getDetailModulation() const
{
    return mDetailModulation;
}

float Terrain3D::getDetailStrength() const
{
    return mDetailStrength;
}

void Terrain3D::setLightmapInNearShading(bool enabled)
{
    mLightmapNear = enabled;
    if (mLoaded) refreshMaterials();
}

bool Terrain3D::getLightmapInNearShading() const
{
    return mLightmapNear;
}

bool Terrain3D::addLayer(float worldSize, const ct::String& diffuseSpecular,
        const ct::String& normalHeight)
{
    if (mLoaded && mTerrain.getLayerCount() >= mTerrain.getMaxLayers())
    {
        log_error("terrain: the layer limit was reached");
        return false;
    }
    ct::Vector<ct::String> names;
    names.push_back(diffuseSpecular);
    names.push_back(normalHeight);
    mTerrain.addLayer(worldSize, &names);
    if (mLoaded)
    {
        loadLayerTextures();
        refreshMaterials();
        rebakeComposite();
    }
    return true;
}

bool Terrain3D::addLayer(float worldSize, TextureId diffuseSpecular, TextureId normalHeight)
{
    if (mLoaded && mTerrain.getLayerCount() >= mTerrain.getMaxLayers())
    {
        log_error("terrain: the layer limit was reached");
        return false;
    }
    ct::Vector<ct::String> names;
    if (TextureStore* textures = TextureStore::singleton())
    {
        const TextureInfo* diffuseInfo = textures->get(diffuseSpecular);
        const TextureInfo* normalInfo = textures->get(normalHeight);
        names.push_back(diffuseInfo ? diffuseInfo->path : ct::String());
        names.push_back(normalInfo ? normalInfo->path : ct::String());
    }
    mTerrain.addLayer(worldSize, names.empty() ? nullptr : &names);
    const std::uint8_t index = static_cast<std::uint8_t>(mTerrain.getLayerCount() - 1);
    if (index < kLayerSlots)
    {
        mLayerTextures[index][0] = diffuseSpecular;
        mLayerTextures[index][1] = normalHeight;
        mLayerOwned[index][0] = false;
        mLayerOwned[index][1] = false;
    }
    if (mLoaded)
    {
        refreshMaterials();
        rebakeComposite();
    }
    return true;
}

void Terrain3D::removeLayer(std::uint8_t index)
{
    if (index >= mTerrain.getLayerCount()) return;
    TextureStore* textures = TextureStore::singleton();
    const std::uint32_t count = mTerrain.getLayerCount();
    for (std::uint32_t j = 0; j < 2; ++j)
    {
        if (index < kLayerSlots && mLayerOwned[index][j] && textures)
            textures->destroy(mLayerTextures[index][j]);
    }
    for (std::uint32_t i = index; i + 1 < count && i + 1 < kLayerSlots; ++i)
    {
        for (std::uint32_t j = 0; j < 2; ++j)
        {
            mLayerTextures[i][j] = mLayerTextures[i + 1][j];
            mLayerOwned[i][j] = mLayerOwned[i + 1][j];
        }
    }
    if (count - 1 < kLayerSlots)
    {
        for (std::uint32_t j = 0; j < 2; ++j)
        {
            mLayerTextures[count - 1][j] = TextureId();
            mLayerOwned[count - 1][j] = false;
        }
    }
    mTerrain.removeLayer(index);
    if (mLoaded)
    {
        refreshMaterials();
        rebakeComposite();
    }
}

void Terrain3D::setLayerWorldSize(std::uint8_t index, float size)
{
    mTerrain.setLayerWorldSize(index, size);
    if (mLoaded)
    {
        refreshMaterials();
        rebakeComposite();
    }
}

void Terrain3D::setLayerTexture(std::uint8_t layer, std::uint8_t sampler, TextureId texture)
{
    if (layer >= kLayerSlots || sampler >= 2) return;
    TextureStore* textures = TextureStore::singleton();
    if (mLayerOwned[layer][sampler] && textures) textures->destroy(mLayerTextures[layer][sampler]);
    mLayerTextures[layer][sampler] = texture;
    mLayerOwned[layer][sampler] = false;
    if (mLoaded)
    {
        refreshMaterials();
        rebakeComposite();
    }
}

TextureId Terrain3D::getLayerTexture(std::uint8_t layer, std::uint8_t sampler) const
{
    if (layer >= kLayerSlots || sampler >= 2) return TextureId();
    return mLayerTextures[layer][sampler];
}

TerrainLayerBlendMap* Terrain3D::getLayerBlendMap(std::uint8_t layer)
{
    return mTerrain.getLayerBlendMap(layer);
}

void Terrain3D::setGlobalColourMapEnabled(bool enabled, std::uint16_t size)
{
    mTerrain.setGlobalColourMapEnabled(enabled, size);
    if (mLoaded) refreshMaterials();
}

void Terrain3D::setColourMapEnabled(bool enabled)
{
    setGlobalColourMapEnabled(enabled, mTerrain.getGlobalColourMapSize());
}

bool Terrain3D::getColourMapEnabled() const
{
    return mTerrain.getGlobalColourMapEnabled();
}

void Terrain3D::setColourMapSize(int size)
{
    if (size < 64) size = 64;
    if (size > 4096) size = 4096;
    setGlobalColourMapEnabled(mTerrain.getGlobalColourMapEnabled(),
            static_cast<std::uint16_t>(size));
}

int Terrain3D::getColourMapSize() const
{
    return mTerrain.getGlobalColourMapSize();
}

bool Terrain3D::setGlobalColourMap(const Pixmap& image)
{
    if (!mTerrain.loadGlobalColourMap(image)) return false;
    rebakeComposite();
    return true;
}

void Terrain3D::setCompositeMapDistance(float distance)
{
    mTerrain.setCompositeMapDistance(distance);
}

float Terrain3D::getCompositeMapDistance() const
{
    return mTerrain.getCompositeMapDistance();
}

void Terrain3D::setLightMapDirection(const Math::Vec3& direction)
{
    mTerrain.setLightMapDirection(direction);
}

void Terrain3D::setCompositeMapLighting(const Math::Vec3& ambient, const Math::Vec3& diffuse)
{
    mTerrain.setCompositeMapAmbient(ambient);
    mTerrain.setCompositeMapDiffuse(diffuse);
    if (mLoaded)
    {
        rebakeComposite();
    }
}

TextureId Terrain3D::getNormalMapTexture() const
{
    return mNormalMap;
}

TextureId Terrain3D::getLightmapTexture() const
{
    return mLightmap;
}

TextureId Terrain3D::getColourMapTexture() const
{
    return mColourMap;
}

TextureId Terrain3D::getCompositeMapTexture() const
{
    return mCompositeMap;
}

TextureId Terrain3D::getBlendTexture(std::uint8_t index) const
{
    return index < mBlendTextures.size() ? mBlendTextures[index] : TextureId();
}

MaterialId Terrain3D::getCompositeMaterial() const
{
    return mCompositeMaterial;
}

std::uint64_t Terrain3D::getUploadedBytes() const
{
    return mUploadedBytes;
}

std::uint32_t Terrain3D::getUploadCount() const
{
    return mUploadCount;
}

void Terrain3D::resetUploadStats()
{
    mUploadedBytes = 0;
    mUploadCount = 0;
}

std::uint64_t Terrain3D::getTextureMemoryBytes() const
{
    TextureStore* textures = TextureStore::singleton();
    if (!textures) return 0;
    auto bytesOf = [&](TextureId id) -> std::uint64_t
    {
        const TextureInfo* info = textures->get(id);
        if (!info) return 0;
        std::uint64_t total = 0;
        std::uint32_t w = info->width;
        std::uint32_t h = info->height;
        for (std::uint32_t mip = 0; mip < info->mipLevels; ++mip)
        {
            total += static_cast<std::uint64_t>(w) * h * bytesPerTexel(info->format);
            w = w > 1 ? w / 2 : 1;
            h = h > 1 ? h / 2 : 1;
        }
        return total;
    };
    std::uint64_t total =
            bytesOf(mNormalMap) + bytesOf(mLightmap) + bytesOf(mColourMap) + bytesOf(mCompositeMap);
    for (std::size_t i = 0; i < mBlendTextures.size(); ++i) total += bytesOf(mBlendTextures[i]);
    return total;
}

std::uint32_t Terrain3D::getCompositeBakes() const
{
    return mCompositeBakes;
}

std::uint64_t Terrain3D::getCompositeBakeTexels() const
{
    return mCompositeBakeTexels;
}

int Terrain3D::getNodesDrawnComposite() const
{
    return mNodesComposite;
}

bool Terrain3D::isNodeComposite(const TerrainQuadTreeNode* node) const
{
    const std::uint32_t* index = mNodeIndex.find(nodeKey(node));
    return index && mNodes[*index].composite;
}

void Terrain3D::bindProperties()
{
    using DB = NodeClassDB;
    using P = Terrain3D;
    resourceprop::add<P, MaterialId>(PropertyInfo::resource("material", "Material"),
            &P::setMaterial, &P::getAssignedMaterial);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "max_pixel_error", 0.0, 64.0, 0.01, true),
            &P::setMaxPixelError, &P::getMaxPixelError);
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "cast_shadows"), &P::setCastShadows,
            &P::getCastShadows);

    DB::addPropertyGroup<P>("Maps", "");
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "lightmap_enabled"), &P::setLightmapEnabled,
            &P::getLightmapEnabled);
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "lightmap_in_near_shading"),
            &P::setLightmapInNearShading, &P::getLightmapInNearShading);
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "composite_map_enabled"),
            &P::setCompositeMapEnabled, &P::getCompositeMapEnabled);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "composite_map_distance", 0.0,
                               100000.0, 0.1, true),
            &P::setCompositeMapDistance, &P::getCompositeMapDistance);
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "colour_map_enabled"),
            &P::setColourMapEnabled, &P::getColourMapEnabled);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Int, "colour_map_size", 64.0, 4096.0, 64.0),
            &P::setColourMapSize, &P::getColourMapSize);

    DB::addPropertyGroup<P>("Layers", "layer_");
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "layer_normal_mapping_enabled"),
            &P::setLayerNormalMappingEnabled, &P::getLayerNormalMappingEnabled);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Int, "layer_parallax_mode", 0.0, 2.0, 1.0),
            &P::setLayerParallaxMode, &P::getLayerParallax);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "layer_parallax_scale", 0.0, 1.0, 0.001, true),
            &P::setLayerParallaxScale, &P::getLayerParallaxScale);
    DB::addProperty<P>(
            PropertyInfo(PropertyType::Bool, "detail_modulation"),
            [](P* terrain, const PropertyValue& value)
            { terrain->setDetailModulation(value.asBool(), terrain->getDetailStrength()); },
            &P::getDetailModulation);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "detail_strength", 0.0, 1.0, 0.01),
            [](P* terrain, const PropertyValue& value)
            {
                terrain->setDetailModulation(terrain->getDetailModulation(),
                        static_cast<float>(value.asFloat()));
            },
            &P::getDetailStrength);

    const auto editorOnly = [](PropertyInfo info)
    {
        info.usage = PropertyUsage::Editor;
        return info;
    };
    DB::addProperty(P::classNameStatic(),
            editorOnly(PropertyInfo::range(PropertyType::Int, "layer_count", 0.0,
                    static_cast<double>(kLayerSlots), 1.0)),
            PropertySetter([](Node* node, const PropertyValue& value)
                    { static_cast<P*>(node)->setLayerCount(static_cast<int>(value.asInt())); }),
            PropertyGetter(
                    [](const Node* node) {
                        return PropertyValue::fromInt(static_cast<const P*>(node)->getLayerCount());
                    }));
    static const char* const kLayerLabels[kLayerSlots] = { "Layer 1", "Layer 2", "Layer 3",
        "Layer 4", "Layer 5" };
    static const char* const kLayerPrefixes[kLayerSlots] = { "layer_0/", "layer_1/", "layer_2/",
        "layer_3/", "layer_4/" };
    static const char* const kWorldSizeNames[kLayerSlots] = { "layer_0/world_size",
        "layer_1/world_size", "layer_2/world_size", "layer_3/world_size", "layer_4/world_size" };
    static const char* const kDiffuseNames[kLayerSlots] = { "layer_0/diffuse_specular",
        "layer_1/diffuse_specular", "layer_2/diffuse_specular", "layer_3/diffuse_specular",
        "layer_4/diffuse_specular" };
    static const char* const kNormalNames[kLayerSlots] = { "layer_0/normal_height",
        "layer_1/normal_height", "layer_2/normal_height", "layer_3/normal_height",
        "layer_4/normal_height" };
    for (std::uint8_t i = 0; i < kLayerSlots; ++i)
    {
        DB::addPropertySubgroup<P>(kLayerLabels[i], kLayerPrefixes[i]);
        DB::addProperty(P::classNameStatic(),
                editorOnly(PropertyInfo::range(PropertyType::Float, kWorldSizeNames[i], 0.0,
                        100000.0, 0.1, true)),
                PropertySetter(
                        [i](Node* node, const PropertyValue& value) {
                            static_cast<P*>(node)->setLayerSlotWorldSize(i,
                                    static_cast<float>(value.asFloat()));
                        }),
                PropertyGetter(
                        [i](const Node* node) {
                            return PropertyValue::fromFloat(
                                    static_cast<const P*>(node)->getLayerSlotWorldSize(i));
                        }));
        DB::addProperty(P::classNameStatic(),
                editorOnly(PropertyInfo::resource(kDiffuseNames[i], "Texture2D")),
                PropertySetter([i](Node* node, const PropertyValue& value)
                        { static_cast<P*>(node)->setLayerTexturePath(i, 0, value.asString()); }),
                PropertyGetter(
                        [i](const Node* node)
                        {
                            return PropertyValue::fromResourcePath(
                                    static_cast<const P*>(node)->getLayerTexturePath(i, 0));
                        }));
        DB::addProperty(P::classNameStatic(),
                editorOnly(PropertyInfo::resource(kNormalNames[i], "Texture2D")),
                PropertySetter([i](Node* node, const PropertyValue& value)
                        { static_cast<P*>(node)->setLayerTexturePath(i, 1, value.asString()); }),
                PropertyGetter(
                        [i](const Node* node)
                        {
                            return PropertyValue::fromResourcePath(
                                    static_cast<const P*>(node)->getLayerTexturePath(i, 1));
                        }));
    }
}

void Terrain3D::validateProperty(PropertyInfo& info) const
{
    Node3D::validateProperty(info);
    if (info.name.size() < 8 || !info.name.starts_with("layer_") || info.name[7] != '/') return;
    const char digit = info.name[6];
    if (digit < '0' || digit > '9') return;
    if (digit - '0' >= getLayerCount()) info.usage = PropertyUsage::NoEditor;
}

int Terrain3D::getLayerCount() const
{
    return static_cast<int>(mTerrain.getLayerCount());
}

void Terrain3D::setLayerCount(int count)
{
    const int limit = static_cast<int>(kLayerSlots);
    if (count < 0) count = 0;
    if (count > limit) count = limit;
    bool changed = false;
    while (getLayerCount() > count)
    {
        removeLayer(static_cast<std::uint8_t>(getLayerCount() - 1));
        changed = true;
    }
    while (getLayerCount() < count)
    {
        const float size = mLayerWorldSizes[static_cast<std::size_t>(getLayerCount())];
        if (!addLayer(size, ct::String(), ct::String())) break;
        changed = true;
    }
    if (changed) notifyPropertyListChanged();
}

void Terrain3D::setLayerSlotWorldSize(std::uint8_t layer, float size)
{
    if (layer >= kLayerSlots) return;
    mLayerWorldSizes[layer] = size;
    if (layer < mTerrain.getLayerCount()) setLayerWorldSize(layer, size);
}

float Terrain3D::getLayerSlotWorldSize(std::uint8_t layer) const
{
    if (layer >= kLayerSlots) return 0.0f;
    if (layer < mTerrain.getLayerCount()) return mTerrain.getLayerWorldSize(layer);
    return mLayerWorldSizes[layer];
}

void Terrain3D::setLayerTexturePath(std::uint8_t layer, std::uint8_t sampler,
        const ct::String& path)
{
    if (layer >= mTerrain.getLayerCount() || layer >= kLayerSlots || sampler >= 2) return;
    if (mTerrain.getLayerTextureName(layer, sampler) == path) return;
    TextureStore* textures = TextureStore::singleton();
    if (mLayerOwned[layer][sampler] && textures) textures->destroy(mLayerTextures[layer][sampler]);
    mLayerTextures[layer][sampler] = TextureId();
    mLayerOwned[layer][sampler] = false;
    mTerrain.setLayerTextureName(layer, sampler, path);
    if (mLoaded)
    {
        loadLayerTextures();
        refreshMaterials();
        rebakeComposite();
    }
}

ct::String Terrain3D::getLayerTexturePath(std::uint8_t layer, std::uint8_t sampler) const
{
    if (layer >= mTerrain.getLayerCount() || sampler >= 2) return ct::String();
    return mTerrain.getLayerTextureName(layer, sampler);
}

namespace
{

constexpr std::int64_t kTerrainDataVersion = 1;

}

bool Terrain3D::saveData(ct::Json& data, SceneResult& result, const ct::String& path) const
{
    (void) result;
    (void) path;
    if (!mLoaded || !mTerrain.isPrepared()) return false;

    data["version"] = ct::Json(static_cast<long long>(kTerrainDataVersion));
    const Terrain& terrain = mTerrain;
    const std::size_t texels = static_cast<std::size_t>(terrain.getSize()) * terrain.getSize();

    ct::Json setup = ct::Json::object();
    setup["size"] = ct::Json(static_cast<long long>(terrain.getSize()));
    setup["world_size"] = scenejson::fromFloat(terrain.getWorldSize());
    setup["max_batch_size"] = ct::Json(static_cast<long long>(terrain.getMaxBatchSize()));
    setup["min_batch_size"] = ct::Json(static_cast<long long>(terrain.getMinBatchSize()));
    setup["skirt_size"] = scenejson::fromFloat(terrain.getSkirtSize());
    setup["use_ray_box_distance"] = ct::Json(terrain.getUseRayBoxDistanceCalculation());
    setup["layer_blend_map_size"] =
            ct::Json(static_cast<long long>(terrain.getLayerBlendMapSize()));
    setup["light_map_size"] = ct::Json(static_cast<long long>(terrain.getLightmapSize()));
    setup["composite_map_size"] = ct::Json(static_cast<long long>(terrain.getCompositeMapSize()));
    setup["default_layer_world_size"] =
            scenejson::fromFloat(terrain.getDefaultLayerTextureWorldSize());
    setup["light_map_direction"] = scenejson::fromVec3(terrain.getLightMapDirection());
    setup["composite_map_ambient"] = scenejson::fromVec3(terrain.getCompositeMapAmbient());
    setup["composite_map_diffuse"] = scenejson::fromVec3(terrain.getCompositeMapDiffuse());
    data["setup"] = setup;
    data["heights"] = scenejson::fromFloatBlock(terrain.getHeightData(), texels);

    ct::Json layers = ct::Json::array();
    for (std::uint8_t i = 0; i < terrain.getLayerCount(); ++i)
    {
        ct::Json layer = ct::Json::object();
        layer["world_size"] = scenejson::fromFloat(terrain.getLayerWorldSize(i));
        ct::Json textures = ct::Json::array();
        for (std::uint8_t s = 0; s < Terrain::kLayerSamplerCount; ++s)
            textures.push_back(ct::Json(terrain.getLayerTextureName(i, s)));
        layer["textures"] = textures;
        layers.push_back(layer);
    }
    if (layers.size() > 0) data["layers"] = layers;

    ct::Json blends = ct::Json::array();
    for (std::uint8_t i = 0; i < terrain.getBlendTextureCount(); ++i)
    {
        const ct::Vector<std::uint8_t>& bytes = terrain.getBlendTextureData(i);
        bool any = false;
        for (std::size_t b = 0; b < bytes.size() && !any; ++b) any = bytes[b] != 0;
        if (!any) continue;
        ct::Json blend = ct::Json::object();
        blend["texture"] = ct::Json(static_cast<long long>(i));
        blend["pixels"] = scenejson::fromBlock(bytes.data(), bytes.size());
        blends.push_back(blend);
    }
    if (blends.size() > 0) data["blend_maps"] = blends;

    if (terrain.getGlobalColourMapEnabled())
    {
        ct::Json colour = ct::Json::object();
        colour["size"] = ct::Json(static_cast<long long>(terrain.getGlobalColourMapSize()));
        const ct::Vector<std::uint8_t>& pixels = terrain.getGlobalColourMapData();
        if (!pixels.empty()) colour["pixels"] = scenejson::fromBlock(pixels.data(), pixels.size());
        data["colour_map"] = colour;
    }
    return true;
}

bool Terrain3D::loadData(const ct::Json& data, SceneResult& result, const ct::String& path)
{
    if (!scenejson::checkVersion(data, kTerrainDataVersion, result, path)) return false;

    const ct::Json* setup = data.find("setup");
    const ct::Json* heightsField = data.find("heights");
    ct::Vector<float> heights;
    if (!setup || !setup->is_object() || !heightsField ||
            !scenejson::toFloatBlock(*heightsField, heights))
    {
        result.addError(path, "needs setup and heights");
        return false;
    }
    const ct::String setupPath = scenejson::field(path, "setup");

    TerrainImportData importData;
    importData.terrainAlign = TerrainAlignment::AlignXZ;
    importData.terrainSize = static_cast<std::uint16_t>(scenejson::getInt(*setup, "size", 0));
    importData.worldSize = scenejson::getFloat(*setup, "world_size", importData.worldSize);
    importData.maxBatchSize = static_cast<std::uint16_t>(
            scenejson::getInt(*setup, "max_batch_size", importData.maxBatchSize));
    importData.minBatchSize = static_cast<std::uint16_t>(
            scenejson::getInt(*setup, "min_batch_size", importData.minBatchSize));
    importData.skirtSize = scenejson::getFloat(*setup, "skirt_size", importData.skirtSize);
    importData.useRayBoxDistanceCalculation =
            scenejson::getBool(*setup, "use_ray_box_distance", false);
    importData.layerBlendMapSize = static_cast<std::uint16_t>(
            scenejson::getInt(*setup, "layer_blend_map_size", importData.layerBlendMapSize));
    importData.lightMapSize = static_cast<std::uint16_t>(
            scenejson::getInt(*setup, "light_map_size", importData.lightMapSize));
    importData.compositeMapSize = static_cast<std::uint16_t>(
            scenejson::getInt(*setup, "composite_map_size", importData.compositeMapSize));
    importData.defaultLayerTextureWorldSize = scenejson::getFloat(*setup,
            "default_layer_world_size", importData.defaultLayerTextureWorldSize);
    scenejson::readVec3Field(*setup, "light_map_direction", importData.lightMapDirection, result,
            setupPath);
    scenejson::readVec3Field(*setup, "composite_map_ambient", importData.compositeMapAmbient,
            result, setupPath);
    scenejson::readVec3Field(*setup, "composite_map_diffuse", importData.compositeMapDiffuse,
            result, setupPath);
    importData.maxPixelError = mTerrain.getMaxPixelError();
    importData.compositeMapDistance = mTerrain.getCompositeMapDistance();

    if (heights.size() != static_cast<std::size_t>(importData.terrainSize) * importData.terrainSize)
    {
        result.addError(scenejson::field(path, "heights"), "expected size * size heights");
        return false;
    }
    importData.inputFloat = heights.data();

    if (const ct::Json* layers = scenejson::arrayField(data, "layers", result, path))
    {
        for (std::size_t i = 0; i < layers->size(); ++i)
        {
            const ct::Json& entry = layers->at(i);
            const ct::String layerPath = scenejson::element(path, "layers", i);
            TerrainLayerInstance layer;
            layer.worldSize = scenejson::getFloat(entry, "world_size", layer.worldSize);
            if (const ct::Json* textures =
                            scenejson::arrayField(entry, "textures", result, layerPath))
            {
                for (std::size_t s = 0; s < textures->size(); ++s)
                    layer.textureNames.push_back(
                            textures->at(s).is_string() ? textures->at(s).str() : ct::String());
            }
            importData.layerList.push_back(layer);
        }
    }

    if (!load(importData))
    {
        result.addError(path, "the terrain could not be loaded from this data");
        return false;
    }

    if (const ct::Json* blends = scenejson::arrayField(data, "blend_maps", result, path))
    {
        for (std::size_t i = 0; i < blends->size(); ++i)
        {
            const ct::Json& entry = blends->at(i);
            const ct::String blendPath = scenejson::element(path, "blend_maps", i);
            const std::int64_t index =
                    entry.is_object() ? scenejson::getInt(entry, "texture", -1) : -1;
            ct::Vector<std::uint8_t> bytes;
            const ct::Json* pixels = entry.is_object() ? entry.find("pixels") : nullptr;
            if (index < 0 || index >= mTerrain.getBlendTextureCount() || !pixels ||
                    !scenejson::toBlock(*pixels, bytes) ||
                    bytes.size() !=
                            mTerrain.getBlendTextureData(static_cast<std::uint8_t>(index)).size())
            {
                result.addError(blendPath,
                        "needs a texture index and pixels of the blend map size");
                continue;
            }
            mTerrain.getBlendTextureData(static_cast<std::uint8_t>(index)) = bytes;
            const int size = mTerrain.getLayerBlendMapSize();
            mTerrain.uploadBlendTextureRect(static_cast<std::uint8_t>(index),
                    TerrainRect(0, 0, size, size));
        }
    }

    if (const ct::Json* colour = scenejson::objectField(data, "colour_map", result, path))
    {
        const ct::String colourPath = scenejson::field(path, "colour_map");
        const std::int64_t size = scenejson::getInt(*colour, "size", 0);
        if (size < 2)
        {
            result.addError(colourPath, "needs a size");
        }
        else
        {
            setGlobalColourMapEnabled(true, static_cast<std::uint16_t>(size));
            ct::Vector<std::uint8_t> bytes;
            const ct::Json* pixels = colour->find("pixels");
            if (pixels)
            {
                if (!scenejson::toBlock(*pixels, bytes) ||
                        bytes.size() != static_cast<std::size_t>(size * size * 3))
                {
                    result.addError(scenejson::field(colourPath, "pixels"),
                            "expected size * size * 3 bytes");
                }
                else
                {
                    Pixmap image(static_cast<int>(size), static_cast<int>(size), 3);
                    memcpy(image.pixels, bytes.data(), bytes.size());
                    if (!setGlobalColourMap(image))
                        result.addError(colourPath, "the colour map could not be loaded");
                }
            }
        }
    }
    return true;
}

} // namespace prisma3d
