#pragma once

#include "prisma3d/render/Impostor.h"
#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/scene/MultiMeshInstance3D.h"
#include "prisma3d/scene/Node3D.h"
#include "prisma3d/scene/TerrainObserver.h"
#include "prisma3d/scene/TerrainPlacement.h"

#include <ct/string.hpp>
#include <ct/vector.hpp>

#include <cstdint>

namespace prisma3d
{

struct ForestSpeciesDesc
{
    MeshId mesh;
    MeshId shadowMesh;
    ct::Vector<MaterialId> materials;
};

class Terrain3D;

class Forest3D : public Node3D, private TerrainObserver
{
public:
    PRISMA3D_NODE_CLASS(Forest3D, Node3D);
    static void bindProperties();

    bool saveData(ct::Json& data, SceneResult& result, const ct::String& path) const override;
    bool loadData(const ct::Json& data, SceneResult& result, const ct::String& path) override;

    Forest3D();
    ~Forest3D() override;

    static constexpr int kSpeciesSlots = 8;

    int addSpecies(const ForestSpeciesDesc& desc);
    void removeSpecies(int species);
    int getSpeciesCount() const;
    bool setSpeciesMeshPath(int species, const ct::String& path);
    ct::String getSpeciesMeshPath(int species) const;
    bool setSpeciesShadowMeshPath(int species, const ct::String& path);
    ct::String getSpeciesShadowMeshPath(int species) const;
    ImpostorId getImpostor(int species) const;
    ImpostorSetId getImpostorSet() const;
    MultiMeshInstance3D* getMeshNode(int species) const;

    int addInstance(int species, const Math::Mat4& transform);
    int addInstance(int species, const Math::Mat4& transform, bool onTerrain);
    int plant(int species, const Math::Vec3& position, float yaw, float scale);
    void clearInstances();
    int removeInstancesInRadius(const Math::Vec3& worldCentre, float radius);

    void setTerrain(Terrain3D* terrain);
    Terrain3D* getTerrain() const;
    void setTerrainPath(const ct::String& path);
    ct::String getTerrainPath() const;
    void setTerrainOffset(float offset);
    float getTerrainOffset() const;
    void setMaxSlope(float degrees);
    float getMaxSlope() const;
    void addExclusion(const TerrainPath& path, float margin);
    void clearExclusions();
    int plantOnTerrain(int species, float x, float z, float yaw, float scale);
    int getReseatedCount() const;
    float getReseatMilliseconds() const;
    int getInstanceCount() const;
    int getInstanceSpecies(int instance) const;
    bool isInstanceOnTerrain(int instance) const;
    Math::Mat4 getInstanceTransform(int instance) const;
    Math::Vec3 getInstanceCentre(int instance) const;
    float getInstanceRadius(int instance) const;

    void setSwapDistance(float distance);
    float getSwapDistance() const;
    void setDrawDistance(float distance);
    float getDrawDistance() const;
    void setImpostorsEnabled(bool enabled);
    bool isImpostorsEnabled() const;
    void setCastShadows(bool cast);
    bool getCastShadows() const;

    bool update(const Math::Vec3& cameraPosition);
    bool update(const Math::Vec3& cameraPosition, const Math::Frustum* frustum);
    void setFrustumCullingEnabled(bool enabled);
    bool isFrustumCullingEnabled() const;
    void setShadowCullMargin(float margin);
    float getShadowCullMargin() const;
    int getMeshInstanceCount() const;
    int getImpostorInstanceCount() const;
    int getMeshInstanceCount(int species) const;
    int getImpostorInstanceCount(int species) const;
    int getWrittenInstanceCount() const;

    static bool meshDissolve(float distance, float fade, float radius, float& dither);

protected:
    void notificationHandler(int what) override;
    void validateProperty(PropertyInfo& info) const override;

private:
    void resolveTerrain();
    struct SlotList
    {
        ct::Vector<std::uint32_t> items;
        ct::Vector<std::int32_t> slotOf;
    };

    struct Species
    {
        ForestSpeciesDesc desc;
        ImpostorId impostor;
        MultiMeshInstance3D* meshNode = nullptr;
        ct::Vector<std::uint32_t> instances;
        SlotList meshList;
        int cards = 0;
        bool allocated = false;
    };

    struct Instance
    {
        Math::Mat4 transform;
        Math::Vec3 position;
        Math::Vec3 centre = Math::Vec3(0.0f, 0.0f, 0.0f);
        float radius = 0.0f;
        float written = 0.0f;
        std::uint32_t species = 0;
        std::uint32_t local = 0;
        bool onTerrain = false;
    };

    void connectPreDraw();
    void disconnectPreDraw();
    void updateFromCamera();
    void releaseSpecies(Species& species);
    void allocate(Species& species);
    void applyFade();
    float fadeDistance() const;
    void refreshBounds(Instance& instance) const;
    void writeRecord(std::size_t index);
    void writeRecords();
    bool syncWorld();
    void enterWorld();
    void exitWorld();
    void onTerrainChanged(const Math::Box& worldBox) override;
    void onTerrainDestroyed() override;

    ct::Vector<Species*> mSpecies;
    ct::Vector<Instance> mInstances;
    float mSwapDistance;
    float mDrawDistance;
    bool mImpostorsEnabled;
    bool mFrustumCulling;
    float mShadowCullMargin;
    bool mCastShadows;
    int mWritten;
    bool mConnected;
    ImpostorSetId mSet;
    Math::Mat4 mWorld;
    Terrain3D* mTerrain;
    ct::String mTerrainPath;
    TerrainPlacement mPlacement;
    float mTerrainOffset;
    int mReseated;
    float mReseatMs;
};

} // namespace prisma3d
