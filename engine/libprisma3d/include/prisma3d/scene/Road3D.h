#pragma once

#include "prisma3d/render/MeshStore.h"
#include "prisma3d/scene/Collision.h"
#include "prisma3d/scene/TerrainObserver.h"
#include "prisma3d/scene/VisualInstance3D.h"

#include <ct/function.hpp>
#include <ct/string.hpp>
#include <ct/vector.hpp>

#include <cstdint>

namespace prisma3d
{

class Path3D;
class Curve3D;
class Terrain3D;
struct TerrainPath;

class Road3D : public VisualInstance3D, private TerrainObserver
{
public:
    typedef ct::Function<float(const Math::Vec3&)> GroundFunction;

    PRISMA3D_NODE_CLASS(Road3D, VisualInstance3D);

    static void bindProperties();

    bool saveData(ct::Json& data, SceneResult& result, const ct::String& path) const override;
    bool loadData(const ct::Json& data, SceneResult& result, const ct::String& path) override;

    enum class IntervalType
    {
        Distance,
        Subdivide
    };

    enum class RotationMode
    {
        Fixed,
        Path,
        PathFollow
    };

    Road3D();
    ~Road3D() override;

    void setWidth(float width);
    float getWidth() const;

    void setProfile(const ct::Vector<Math::Vec2>& profile);
    void clearProfile();
    const ct::Vector<Math::Vec2>& getProfile() const;

    void setPointWidth(int index, float width);
    float getPointWidth(int index) const;
    void clearPointWidths();

    void setIntervalType(IntervalType type);
    IntervalType getIntervalType() const;
    void setInterval(float interval);
    float getInterval() const;
    void setSimplifyAngle(float degrees);
    float getSimplifyAngle() const;
    void setRotationMode(RotationMode mode);
    RotationMode getRotationMode() const;
    void setRotationAccurate(bool accurate);
    bool isRotationAccurate() const;
    void setTextureRepeat(float metres);
    float getTextureRepeat() const;

    void setMaterial(MaterialId material);
    MaterialId getMaterial() const;
    MaterialId getAssignedMaterial() const;

    void setGround(const GroundFunction& ground);
    void clearGround();
    bool hasGround() const;
    void setTerrain(Terrain3D* terrain);
    Terrain3D* getTerrain() const;
    void setTerrainPath(const ct::String& path);
    ct::String getTerrainPath() const;
    void setGroundOffset(float offset);
    float getGroundOffset() const;
    void setGroundTilt(bool tilt);
    bool isGroundTilt() const;
    void onGroundChanged(const Math::Box& worldBox);
    bool buildFootprint(TerrainPath& out, float spacing) const;

    void onCurveChanged();
    bool flush();
    void rebuild();
    bool isDirty() const;
    int getBuildCount() const;

    const SurfaceData& getGeometry() const;
    bool buildCollision(CollisionMesh& out) const;
    Math::Box getAabb() const override;

protected:
    void notificationHandler(int what) override;

private:
    void resolveTerrain();
    struct Ring
    {
        Math::Mat3 basis = Math::Mat3::Identity();
        Math::Vec3 origin = Math::Vec3(0.0f, 0.0f, 0.0f);
        float distance = 0.0f;
        float scale = 1.0f;
    };

    int countSegments(const Curve3D& curve, bool closed) const;
    Ring makeRing(const Curve3D& curve, float offset, float step, float length,
            const ct::Vector<float>& pointOffsets) const;
    float widthScaleAt(const Curve3D& curve, float offset,
            const ct::Vector<float>& pointOffsets) const;
    void computePointOffsets(const Curve3D& curve, ct::Vector<float>& out) const;
    void buildRings(const Curve3D& curve, ct::Vector<Ring>& rings) const;
    void buildGeometry(const ct::Vector<Ring>& rings);
    void applyGround(const ct::Vector<Math::Vec2>& profile, const ct::Vector<Ring>& rings,
            std::size_t sides);
    void onTerrainChanged(const Math::Box& worldBox) override;
    void onTerrainDestroyed() override;
    void uploadGeometry();
    void invalidateSurface();
    void applyMaterial();
    void markDirty();
    void connectPreDraw();
    void disconnectPreDraw();

    Path3D* mPath;
    float mWidth;
    ct::Vector<Math::Vec2> mProfile;
    ct::Vector<float> mPointWidths;
    IntervalType mIntervalType;
    float mInterval;
    float mSimplifyAngle;
    RotationMode mRotationMode;
    bool mRotationAccurate;
    float mTextureRepeat;
    bool mDirty;
    int mBuildCount;
    SurfaceData mGeometry;
    MeshId mMesh;
    bool mSurfaceReady;
    bool mUploadedEmpty;
    std::uint32_t mVertexCapacity;
    std::uint32_t mIndexCapacity;
    MaterialId mDefaultMaterial;
    MaterialId mMaterial;
    GroundFunction mGround;
    Terrain3D* mTerrain;
    ct::String mTerrainPath;
    float mGroundOffset;
    bool mGroundTilt;
    float mGroundBounds[4];
    bool mGroundBoundsValid;
};

} // namespace prisma3d
