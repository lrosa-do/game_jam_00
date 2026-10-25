#pragma once

#include "prisma3d/core/Pixmap.h"
#include "prisma3d/scene/MultiMeshInstance3D.h"
#include "prisma3d/scene/Node3D.h"
#include "prisma3d/scene/TerrainObserver.h"
#include "prisma3d/scene/TerrainPlacement.h"

#include <ct/function.hpp>
#include <ct/hashmap.hpp>
#include <ct/string.hpp>
#include <ct/vector.hpp>

#include <cstdint>

namespace prisma3d
{

struct GrassRegion
{
    Math::Vec4 texMulAdd = Math::Vec4(1.0f, 1.0f, 0.0f, 0.0f);
    float size = 1.0f;
    float aspect = 1.0f;
    float weight = 1.0f;
};

struct GrassTypeDesc
{
    TextureId texture;
    float height = 1.2f;
    float width = 0.7f;
    float scaleMin = 0.75f;
    float scaleMax = 1.25f;
    Math::Vec3 colourA = Math::Vec3(0.62f, 0.78f, 0.42f);
    Math::Vec3 colourB = Math::Vec3(0.95f, 0.92f, 0.60f);
    float alphaCut = 0.35f;
    float densityScale = 1.0f;
};

struct GrassInfluencer
{
    Math::Vec3 centre = Math::Vec3(0.0f, 0.0f, 0.0f);
    float radius = 3.0f;
    float force = 30.0f;
};

struct GrassSimParams
{
    float stiffness = 12.0f;
    float drag = 0.12f;
    float deltaTime = 1.0f / 60.0f;
    float time = 0.0f;
    float wind = 1.0f;
    float windSpeed = 1.0f;
    Math::Vec3 windDirection = Math::Vec3(0.82f, 0.0f, 0.57f);
};

struct GrassTip
{
    Math::Vec3 current = Math::Vec3(0.0f, 0.0f, 0.0f);
    Math::Vec3 previous = Math::Vec3(0.0f, 0.0f, 0.0f);
    bool initialised = false;
};

struct GrassClumpInfo
{
    Math::Vec3 root = Math::Vec3(0.0f, 0.0f, 0.0f);
    float scale = 1.0f;
    float rotation = 0.0f;
    std::uint32_t region = 0;
    float keep = 0.0f;
    float density = 1.0f;
    bool inside = true;
    bool live = false;
    bool awake = false;
    Math::Vec3 tip = Math::Vec3(0.0f, 0.0f, 0.0f);
};

class Terrain3D;

class Grass3D : public Node3D, private TerrainObserver
{
public:
    PRISMA3D_NODE_CLASS(Grass3D, Node3D);
    static void bindProperties();

    bool saveData(ct::Json& data, SceneResult& result, const ct::String& path) const override;
    bool loadData(const ct::Json& data, SceneResult& result, const ct::String& path) override;
    static constexpr int kMaxInfluencers = 8;
    static constexpr int kMaxTypes = 4;

    typedef ct::Function<float(float, float)> FieldFunction;

    Grass3D();
    ~Grass3D() override;

    int addType(const GrassTypeDesc& desc);
    void removeType(int type);
    int getTypeCount() const;
    bool getType(int type, GrassTypeDesc& out) const;
    bool setType(int type, const GrassTypeDesc& desc);
    bool addTypeRegion(int type, std::uint32_t x, std::uint32_t y, std::uint32_t width,
            std::uint32_t height, float size = 1.0f, float weight = 1.0f);
    void clearTypeRegions(int type);
    int getTypeRegionCount(int type) const;
    bool getTypeRegion(int type, int index, GrassRegion& out) const;
    MaterialId getMaterial(int type) const;

    void setTexture(TextureId texture);
    TextureId getTexture() const;
    void setTexturePath(const ct::String& path);
    ct::String getTexturePath() const;
    void setTypeCount(int count);
    GrassTypeDesc typeOrDefault(int type) const;
    void setTypeDesc(int type, const GrassTypeDesc& desc);
    void setTypeTexturePath(int type, const ct::String& path);
    ct::String getTypeTexturePath(int type) const;
    void setColourA(const Math::Vec3& colour);
    Math::Vec3 getColourA() const;
    void setColourB(const Math::Vec3& colour);
    Math::Vec3 getColourB() const;
    void setScaleMin(float scale);
    float getScaleMin() const;
    void setScaleMax(float scale);
    float getScaleMax() const;
    void setDensityScale(float scale);
    float getDensityScale() const;
    bool addRegion(std::uint32_t x, std::uint32_t y, std::uint32_t width, std::uint32_t height,
            float size = 1.0f, float weight = 1.0f);
    void clearRegions();
    int getRegionCount() const;
    bool getRegion(int index, GrassRegion& out) const;
    void setHeight(float metres);
    float getHeight() const;
    void setWidth(float multiplier);
    float getWidth() const;
    void setAlphaCut(float cut);
    float getAlphaCut() const;
    MaterialId getMaterial() const;

    void setCellSize(float size);
    float getCellSize() const;
    void setDensity(float tuftsPerSquareMetre);
    float getDensity() const;
    void setDensityFalloffStart(float fraction);
    float getDensityFalloffStart() const;
    void setDrawDistance(float metres);
    void setCastShadows(bool cast);
    bool getCastShadows() const;
    void setAlphaAntialiasing(MaterialAlphaAntialiasing mode, float edge = 0.3f);
    MaterialAlphaAntialiasing getAlphaAntialiasing() const;
    float getDrawDistance() const;
    void setReleaseMargin(float metres);
    float getReleaseMargin() const;
    void setSeed(std::uint32_t seed);
    std::uint32_t getSeed() const;
    void setExtent(float minX, float minZ, float maxX, float maxZ);
    void clearExtent();
    bool hasExtent() const;
    void setExtentEnabled(bool enabled);
    void setExtentMin(const Math::Vec2& minimum);
    Math::Vec2 getExtentMin() const;
    void setExtentMax(const Math::Vec2& maximum);
    Math::Vec2 getExtentMax() const;
    void setHeightFunction(const FieldFunction& function);
    void setDensityFunction(const FieldFunction& function);

    void setTerrain(Terrain3D* terrain);
    Terrain3D* getTerrain() const;
    void setTerrainPath(const ct::String& path);
    ct::String getTerrainPath() const;
    void setMaxSlope(float degrees);
    float getMaxSlope() const;
    void addExclusion(const TerrainPath& path, float margin);
    void clearExclusions();
    int getReseatedCount() const;
    float getReseatMilliseconds() const;

    void setCameraBend(float amount);
    float getCameraBend() const;

    void setWind(float strength);
    float getWind() const;
    void setWindSpeed(float speed);
    float getWindSpeed() const;
    void setWindDirection(const Math::Vec3& direction);
    Math::Vec3 getWindDirection() const;
    void setStiffness(float stiffness);
    float getStiffness() const;
    void setDrag(float drag);
    float getDrag() const;

    void clearInfluencers();
    bool addInfluencer(const Math::Vec3& centre, float radius, float force);
    int getInfluencerCount() const;

    void setMaskResolution(int width, int depth);
    void setMaskResolutionWidth(int width);
    int getMaskResolutionWidth() const;
    void setMaskResolutionDepth(int depth);
    int getMaskResolutionDepth() const;
    bool createMask(int width = 0, int depth = 0);
    bool hasMask() const;
    int getMaskWidth() const;
    int getMaskDepth() const;
    void clearMask();
    bool paint(const Math::Vec3& worldPosition, float radius, float amount, int type);
    bool erase(const Math::Vec3& worldPosition, float radius, float amount, int type);
    bool erase(const Math::Vec3& worldPosition, float radius, float amount);
    void fill(int type, float density);
    float getMaskDensity(int type, float x, float z) const;
    bool saveMask(const char* path);
    bool loadMask(const char* path);
    bool getMaskPixels(ct::Vector<std::uint8_t>& out) const;
    bool setMaskPixels(const ct::Vector<std::uint8_t>& pixels);

    void invalidate();
    bool update(const Math::Vec3& cameraPosition, const Math::Frustum* frustum, float deltaTime);
    bool update(const Math::Vec3& cameraPosition);
    void setFrustumCullingEnabled(bool enabled);
    bool isFrustumCullingEnabled() const;

    int getActiveCellCount(int type = -1) const;
    int getVisibleCellCount(int type = -1) const;
    int getPooledCellCount(int type = -1) const;
    int getInstanceCount(int type = -1) const;
    int getDrawnInstanceCount(int type = -1) const;
    int getWrittenInstanceCount() const;
    int getAwakeInstanceCount() const;
    int getSimulatedInstanceCount() const;
    int getFilledCellCount() const;
    float getUpdateMilliseconds() const;
    bool hasCell(int type, int cellX, int cellZ) const;
    int getCellClumpCount(int type, int cellX, int cellZ) const;
    bool getClump(int type, int cellX, int cellZ, int index, GrassClumpInfo& out) const;
    MultiMeshInstance3D* getCellNode(int type, int cellX, int cellZ) const;
    MeshId getMesh() const;

    static void cellOf(float x, float z, float cellSize, int& cellX, int& cellZ);
    static float cellDistance(int cellX, int cellZ, float cellSize, float x, float z);
    static std::uint32_t cellSeed(std::uint32_t seed, int cellX, int cellZ);
    static std::uint32_t typeSeed(std::uint32_t seed, int type);
    static float nextRandom(std::uint32_t& state);
    static int candidateCount(float density, float cellSize);
    static float distanceKeep(float distance, float drawDistance, float startFraction);
    static float drawFade(float distance, float drawDistance);
    static float windWave(const Math::Vec3& base, float time, float windSpeed);
    static void simulate(const Math::Vec3& base, const Math::Vec3& up, float length, GrassTip& tip,
            const GrassSimParams& params, const GrassInfluencer* influencers, int influencerCount);
    static Math::Vec3 tintOf(const Math::Vec3& root, const Math::Vec3& colourA,
            const Math::Vec3& colourB);
    static float brushFalloff(float distance, float radius);

protected:
    void notificationHandler(int what) override;
    void validateProperty(PropertyInfo& info) const override;

private:
    void resolveTerrain();
    struct Clump
    {
        GrassClumpInfo info;
        Math::Mat4 transform;
        Math::Vec4 regionData;
        float length = 0.0f;
        GrassTip tip;
        Math::Vec3 written = Math::Vec3(0.0f, 0.0f, 0.0f);
        std::int32_t slot = -1;
        bool awake = false;
    };

    struct Cell
    {
        int cellX = 0;
        int cellZ = 0;
        MultiMeshInstance3D* node = nullptr;
        ct::Vector<Clump> clumps;
        ct::Vector<std::uint32_t> items;
        ct::Vector<std::uint32_t> awake;
        Math::Vec3 boundsCentre = Math::Vec3(0.0f, 0.0f, 0.0f);
        float boundsRadius = 0.0f;
        int shown = 0;
        bool active = false;
        bool stale = false;
    };

    struct Type
    {
        GrassTypeDesc desc;
        ct::Vector<GrassRegion> regions;
        MaterialId material;
        ct::HashMap<std::uint64_t, Cell*> active;
        ct::Vector<Cell*> cells;
        ct::Vector<Cell*> pool;
    };

    static std::uint64_t cellKey(int cellX, int cellZ);

    void connectPreDraw();
    void disconnectPreDraw();
    void updateFromCamera();
    void ensureResources();
    void applyMaterial(Type& type);
    void releaseResources();
    void destroyType(Type* type);
    Type* typeAt(int type) const;
    Type* defaultType();
    GrassRegion regionAt(const Type& type, std::uint32_t index) const;
    float totalWeight(const Type& type) const;
    Cell* acquireCell(Type& type, int typeIndex, int cellX, int cellZ);
    void releaseCell(Type& type, Cell* cell);
    void trimPool(Type& type);
    void fillCell(Type& type, int typeIndex, Cell& cell);
    void updateType(Type& type, int typeIndex, const Math::Vec3& camera,
            const Math::Frustum* frustum, const Math::Mat4& world, float worldScale,
            const GrassInfluencer* influencers, int influencerCount, const GrassSimParams& params,
            bool& changed);
    float groundHeight(float x, float z) const;
    float clumpDensity(int type, float x, float z) const;
    void onTerrainChanged(const Math::Box& worldBox) override;
    void onTerrainDestroyed() override;
    void markDirty();
    void applyAllMaterials();
    void markStale(int type, float minX, float minZ, float maxX, float maxZ);
    void markAllStale(int type);
    bool paintMask(const Math::Vec3& worldPosition, float radius, float amount, int type,
            bool subtract, bool allTypes);

    ct::Vector<Type*> mTypes;
    ct::Vector<GrassInfluencer> mInfluencers;
    FieldFunction mHeightFunction;
    FieldFunction mDensityFunction;
    Pixmap* mMask;
    int mMaskWidth;
    int mMaskDepth;
    MeshId mMesh;
    float mCellSize;
    float mDensity;
    float mFalloffStart;
    float mDrawDistance;
    bool mCastShadows;
    MaterialAlphaAntialiasing mAlphaAntialiasing;
    float mAlphaAntialiasingEdge;
    float mReleaseMargin;
    std::uint32_t mSeed;
    bool mHasExtent;
    float mExtent[4];
    float mCameraBend;
    float mWind;
    float mWindSpeed;
    Math::Vec3 mWindDirection;
    float mStiffness;
    float mDrag;
    bool mFrustumCulling;
    bool mDirty;
    float mTime;
    int mWritten;
    int mMoving;
    int mFilled;
    float mUpdateMs;
    bool mConnected;
    Terrain3D* mTerrain;
    ct::String mTerrainPath;
    TerrainPlacement mPlacement;
    int mReseated;
    float mReseatMs;
};

} // namespace prisma3d
