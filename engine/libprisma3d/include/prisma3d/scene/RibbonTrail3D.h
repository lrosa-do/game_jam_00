#pragma once

#include "prisma3d/render/MeshStore.h"
#include "prisma3d/scene/VisualInstance3D.h"

#include <ct/string.hpp>
#include <ct/vector.hpp>

#include <cstdint>

namespace prisma3d
{

class RibbonTrail3D : public VisualInstance3D
{
public:
    PRISMA3D_NODE_CLASS(RibbonTrail3D, VisualInstance3D);
    static void bindProperties();

    bool saveData(ct::Json& data, SceneResult& result, const ct::String& path) const override;
    bool loadData(const ct::Json& data, SceneResult& result, const ct::String& path) override;

    RibbonTrail3D();
    ~RibbonTrail3D() override;

    int addChain(const ct::String& emitterPath,
            const Math::Vec4& startColor = Math::Vec4(1.0f, 1.0f, 1.0f, 1.0f),
            const Math::Vec4& endColor = Math::Vec4(1.0f, 1.0f, 1.0f, 0.0f),
            float startWidth = 0.7f, float endWidth = 0.05f, float fadeTime = 0.3f);
    int addBladeChain(const ct::String& basePath, const ct::String& tipPath,
            const Math::Vec4& startColor = Math::Vec4(1.0f, 1.0f, 1.0f, 1.0f),
            const Math::Vec4& endColor = Math::Vec4(1.0f, 1.0f, 1.0f, 0.0f), float fadeTime = 0.3f,
            float startSpan = 1.0f, float endSpan = 0.0f);
    void clearChains();
    int getChainCount() const;
    int getElementCount(int chain) const;

    void setEmitting(bool emitting);
    bool isEmitting() const;
    void setTrailLength(float length);
    float getTrailLength() const;
    void setMaxElements(int count);
    int getMaxElements() const;
    void setSmoothing(int subdivisions);
    int getSmoothing() const;

    void setMaterial(MaterialId material);
    MaterialId getMaterial() const;
    MaterialId getAssignedMaterial() const;

    void update(double delta);
    const SurfaceData& getGeometry() const;

protected:
    void notificationHandler(int what) override;

private:
    static constexpr int kMaxChains = 4;
    static constexpr std::uint16_t kChainEmpty = 0xFFFF;

    struct Element
    {
        Math::Vec3 position = Math::Vec3(0.0f, 0.0f, 0.0f);
        Math::Vec3 tip = Math::Vec3(0.0f, 0.0f, 0.0f);
        Math::Vec4 color = Math::Vec4(1.0f, 1.0f, 1.0f, 1.0f);
        float width = 0.5f;
    };

    struct Chain
    {
        ct::String emitterPath;
        ct::String tipPath;
        bool blade = false;
        Math::Vec4 startColor = Math::Vec4(1.0f, 1.0f, 1.0f, 1.0f);
        Math::Vec4 endColor = Math::Vec4(1.0f, 1.0f, 1.0f, 0.0f);
        float startWidth = 0.7f;
        float endWidth = 0.05f;
        float fadeTime = 0.3f;
        bool active = true;
        std::uint16_t head = kChainEmpty;
        std::uint16_t tail = kChainEmpty;
        std::uint16_t count = 0;
        Math::Vec3 lastBakedPos = Math::Vec3(0.0f, 0.0f, 0.0f);
    };

    Node3D* resolveEmitter(const ct::String& path) const;
    bool resolveChain(const Chain& chain, Node3D*& emitter, Node3D*& tip) const;
    void resetTrail(Chain& chain, int chainIndex, Node3D* emitter, Node3D* tip);
    void seedChain(int chainIndex);
    void bake();
    void fade(float dt);
    void buildGeometry();
    void uploadGeometry();
    void invalidateSurface();
    void applyMaterial();
    void connectPreDraw();
    void disconnectPreDraw();

    Chain mChains[kMaxChains];
    int mChainCount;
    bool mEmitting;
    ct::Vector<Element> mElements;
    int mMaxElements;
    float mElemLength;
    float mTrailLength;
    int mSmoothing;
    SurfaceData mGeometry;
    ct::Vector<Math::Vec3> mSmoothHilt;
    ct::Vector<Math::Vec3> mSmoothTip;
    ct::Vector<Math::Vec4> mSmoothCol;
    ct::Vector<float> mSmoothW;
    ct::Vector<Math::Vec3> mEdgePos;
    ct::Vector<Math::Vec4> mEdgeCol;
    ct::Vector<float> mEdgeW;
    MeshId mMesh;
    bool mSurfaceReady;
    bool mUploadedEmpty;
    MaterialId mDefaultMaterial;
    MaterialId mMaterial;
};

} // namespace prisma3d
