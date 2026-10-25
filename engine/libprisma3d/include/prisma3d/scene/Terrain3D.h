#pragma once

#include "prisma3d/render/MaterialStore.h"
#include "prisma3d/render/MeshStore.h"
#include "prisma3d/render/TerrainComposite.h"
#include "prisma3d/render/TerrainMaterialData.h"
#include "prisma3d/scene/Node3D.h"
#include "prisma3d/scene/TerrainObserver.h"
#include "prisma3d/scene/VisualInstance3D.h"
#include "prisma3d/terrain/Terrain.h"
#include "prisma3d/terrain/TerrainGeometryTarget.h"
#include "prisma3d/terrain/TerrainLayerBlendMap.h"
#include "prisma3d/terrain/TerrainQuadTreeNode.h"

#include <ct/hashmap.hpp>
#include <ct/vector.hpp>

#include <cstdint>

namespace prisma3d
{

class Terrain3D : public Node3D
{
public:
    PRISMA3D_NODE_CLASS(Terrain3D, Node3D);
    static void bindProperties();

    bool saveData(ct::Json& data, SceneResult& result, const ct::String& path) const override;
    bool loadData(const ct::Json& data, SceneResult& result, const ct::String& path) override;
    static constexpr int kMaxLodLevels = 15;

    Terrain3D();
    ~Terrain3D() override;

    bool load(const TerrainImportData& importData);
    void unload();
    bool isLoaded() const;
    Terrain& getTerrain();
    const Terrain& getTerrain() const;

    void setMaterial(MaterialId material);
    MaterialId getMaterial() const;
    MaterialId getAssignedMaterial() const;
    void setMaxPixelError(float maxPixelError);
    float getMaxPixelError() const;
    void setCastShadows(bool cast);
    bool getCastShadows() const;

    void setLightmapEnabled(bool enabled);
    bool getLightmapEnabled() const;
    void setCompositeMapEnabled(bool enabled);
    bool getCompositeMapEnabled() const;
    void setLayerNormalMappingEnabled(bool enabled);
    bool getLayerNormalMappingEnabled() const;
    void setLayerParallax(std::uint32_t mode, float scale = 0.04f);
    std::uint32_t getLayerParallax() const;
    void setLayerParallaxMode(std::uint32_t mode);
    void setLayerParallaxScale(float scale);
    float getLayerParallaxScale() const;
    void setDetailModulation(bool enabled, float strength = 1.0f);
    bool getDetailModulation() const;
    float getDetailStrength() const;
    void setLightmapInNearShading(bool enabled);
    bool getLightmapInNearShading() const;

    bool addLayer(float worldSize, const ct::String& diffuseSpecular,
            const ct::String& normalHeight);
    bool addLayer(float worldSize, TextureId diffuseSpecular, TextureId normalHeight);
    void removeLayer(std::uint8_t index);
    void setLayerWorldSize(std::uint8_t index, float size);
    void setLayerTexture(std::uint8_t layer, std::uint8_t sampler, TextureId texture);
    TextureId getLayerTexture(std::uint8_t layer, std::uint8_t sampler) const;
    int getLayerCount() const;
    void setLayerCount(int count);
    void setLayerSlotWorldSize(std::uint8_t layer, float size);
    float getLayerSlotWorldSize(std::uint8_t layer) const;
    void setLayerTexturePath(std::uint8_t layer, std::uint8_t sampler, const ct::String& path);
    ct::String getLayerTexturePath(std::uint8_t layer, std::uint8_t sampler) const;
    TerrainLayerBlendMap* getLayerBlendMap(std::uint8_t layer);
    void setGlobalColourMapEnabled(bool enabled, std::uint16_t size = 0);
    void setColourMapEnabled(bool enabled);
    bool getColourMapEnabled() const;
    void setColourMapSize(int size);
    int getColourMapSize() const;
    bool setGlobalColourMap(const Pixmap& image);
    void setCompositeMapDistance(float distance);
    float getCompositeMapDistance() const;
    void setLightMapDirection(const Math::Vec3& direction);
    void setCompositeMapLighting(const Math::Vec3& ambient, const Math::Vec3& diffuse);
    void update();
    std::uint64_t getVertexUploadBytes() const;
    std::uint32_t getVertexUploadRanges() const;
    std::uint32_t getLastUpdateOwners() const;
    void resetEditStats();
    void refreshMaterials();
    bool flushCompositeMap();

    TextureId getNormalMapTexture() const;
    TextureId getLightmapTexture() const;
    TextureId getColourMapTexture() const;
    TextureId getCompositeMapTexture() const;
    TextureId getBlendTexture(std::uint8_t index) const;
    MaterialId getCompositeMaterial() const;
    std::uint64_t getUploadedBytes() const;
    std::uint32_t getUploadCount() const;
    void resetUploadStats();
    std::uint64_t getTextureMemoryBytes() const;
    std::uint32_t getCompositeBakes() const;
    std::uint64_t getCompositeBakeTexels() const;
    int getNodesDrawnComposite() const;
    bool isNodeComposite(const TerrainQuadTreeNode* node) const;

    float getWorldHeightAt(const Math::Vec3& position) const;
    float getWorldSlopeAt(const Math::Vec3& position) const;
    void addObserver(TerrainObserver* observer);
    void removeObserver(TerrainObserver* observer);
    int getObserverCount() const;
    bool rayIntersects(const Math::Ray& ray, Math::Vec3& position) const;
    Math::Box getAabb() const;

    void updateLod(const TerrainLodView& view);
    bool updateFromCamera();

    int getNodeCount() const;
    int getNodesDrawn() const;
    int getNodesDrawnAtLod(int lod) const;
    int getLodLevelCount() const;
    std::uint32_t getTrianglesDrawn() const;
    int getVertexMeshCount() const;
    int getIndexPoolCount() const;
    int getNodeMeshCount() const;
    MeshInstance3D* getNodeInstance(const TerrainQuadTreeNode* node) const;
    MeshId getNodeMesh(const TerrainQuadTreeNode* node, int lod) const;

protected:
    void notificationHandler(int what) override;
    void validateProperty(PropertyInfo& info) const override;

private:
    class MapTarget;
    friend class MapTarget;
    class GeometryTarget;
    friend class GeometryTarget;

    struct NodeGpu
    {
        TerrainQuadTreeNode* node = nullptr;
        MeshInstance3D* instance = nullptr;
        MeshId vertexMesh;
        ct::Vector<MeshId> lodMeshes;
        ct::Vector<std::uint32_t> lodTriangles;
        int shownLod = -1;
        float shownMorph = -1.0f;
        bool shown = false;
        bool composite = false;
    };

    void collectNodes(TerrainQuadTreeNode* node);
    bool createGpuData();
    bool createVertexMesh(NodeGpu& gpu);
    bool createLodMeshes(NodeGpu& gpu);
    void releaseGpuData();
    void writeVertices(const TerrainQuadTreeNode* owner, std::uint32_t first, std::uint32_t count,
            const TerrainVertexData& source);
    void refreshNodeBoxes(const TerrainRect& rect);
    Math::Box worldBoxOf(const TerrainRect& rect) const;
    void notifyObservers(const Math::Box& worldBox);
    void connectPreDraw();
    void disconnectPreDraw();
    void applyMaterial();
    void syncPosition() const;
    bool createMapTexture(TextureId& out, prisma::TextureFormat format, std::uint32_t width,
            std::uint32_t mips, bool srgb, std::uint32_t usage, const void* data, const char* name);
    void destroyMapTexture(TextureId& texture);
    void uploadRegion(TextureId texture, const TerrainRect& box, const void* data,
            std::uint32_t bytesPerTexel, bool regenerateMips);
    void buildMaterialData(TerrainMaterialData& data) const;
    void loadLayerTextures();
    void releaseLayerTextures();
    void releaseMaps();
    void processDeferred();
    void rebakeComposite();
    void applyNodeMaterial(NodeGpu& gpu);
    bool createComposite();

    static constexpr std::uint32_t kLayerSlots = TerrainMaterialData::kMaxLayers;

    Terrain mTerrain;
    MapTarget* mMapTarget;
    GeometryTarget* mGeometryTarget;
    ct::Vector<std::uint8_t> mUploadScratch;
    std::uint64_t mVertexBytes;
    std::uint32_t mVertexRanges;
    std::uint32_t mOwnersUpdated;
    ct::Vector<std::uint32_t> mOwnersTouched;
    TextureId mNormalMap;
    TextureId mLightmap;
    TextureId mColourMap;
    TextureId mCompositeMap;
    ct::Vector<TextureId> mBlendTextures;
    TextureId mLayerTextures[kLayerSlots][2];
    bool mLayerOwned[kLayerSlots][2];
    float mLayerWorldSizes[kLayerSlots];
    MaterialId mCompositeMaterial;
    TerrainComposite mComposite;
    TerrainRect mPendingComposite;
    bool mLightmapEnabled;
    bool mCompositeEnabled;
    bool mLayerNormalMapping;
    std::uint32_t mParallaxMode;
    float mParallaxScale;
    bool mLightmapNear;
    bool mDetailModulation;
    float mDetailStrength;
    std::uint64_t mUploadedBytes;
    std::uint32_t mUploadCount;
    std::uint32_t mCompositeBakes;
    std::uint64_t mCompositeBakeTexels;
    unsigned long mLastMillis;
    int mNodesComposite;
    bool mLoaded;
    bool mCastShadows;
    bool mConnected;
    MaterialId mDefaultMaterial;
    MaterialId mMaterial;
    ct::Vector<TerrainObserver*> mObservers;
    ct::Vector<NodeGpu> mNodes;
    ct::HashMap<std::uint64_t, std::uint32_t> mNodeIndex;
    ct::HashMap<std::uint64_t, MeshId> mIndexPools;
    ct::Vector<MeshId> mPoolMeshes;
    ct::Vector<MeshId> mNodeMeshes;
    ct::Vector<MeshId> mVertexMeshes;
    int mNodesDrawn;
    ct::Vector<int> mNodesAtLod;
    std::uint32_t mTrianglesDrawn;
};

} // namespace prisma3d
