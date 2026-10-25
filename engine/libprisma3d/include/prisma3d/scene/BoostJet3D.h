#pragma once

#include "prisma3d/render/MaterialStore.h"
#include "prisma3d/render/MeshStore.h"
#include "prisma3d/render/TextureStore.h"
#include "prisma3d/scene/Node3D.h"

#include <ct/string.hpp>

namespace prisma3d
{

class MeshInstance3D;

class BoostJet3D : public Node3D
{
public:
    PRISMA3D_NODE_CLASS(BoostJet3D, Node3D);
    static void bindProperties();

    static constexpr int kGlows = 4;

    BoostJet3D();
    ~BoostJet3D() override;

    void setBurning(bool burning);
    bool isBurning() const;
    void setGlowTexture(const ct::String& path);
    const ct::String& getGlowTexture() const;
    void setPhase(float phase);
    float getPhase() const;
    void setJets(bool jets);
    bool hasJets() const;
    void setGlowColor(const Math::Vec4& color);
    const Math::Vec4& getGlowColor() const;

protected:
    void notificationHandler(int what) override;

private:
    void build();
    void release();
    void flicker(float step);
    void tintGlows();

    TextureId mFlameTexture;
    MaterialId mFlameMaterial;
    MeshId mFlameMesh;
    MaterialId mGlowMaterials[kGlows];
    MeshId mGlowMeshes[kGlows];
    MeshInstance3D* mJets[2];
    MeshInstance3D* mGlows[kGlows];
    ct::String mGlowTexture;
    float mPhase;
    Math::Vec4 mGlowColor;
    bool mJetsShown;
    float mFlicker;
    bool mBurning;
    bool mBuilt;
};

} // namespace prisma3d
