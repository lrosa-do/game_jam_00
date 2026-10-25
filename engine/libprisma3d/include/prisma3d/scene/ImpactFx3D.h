#pragma once

#include "prisma3d/audio/AudioServer.h"
#include "prisma3d/render/TextureStore.h"
#include "prisma3d/scene/Node3D.h"

namespace prisma3d
{

class ParticleSystem3D;

class ImpactFx3D : public Node3D
{
public:
    PRISMA3D_NODE_CLASS(ImpactFx3D, Node3D);
    static void bindProperties();

    ImpactFx3D();
    ~ImpactFx3D() override;

    void impact(const Math::Vec3& point, float impulse);
    void smoke(const Math::Vec3& point, int count);
    void flame(const Math::Vec3& point, int count);
    void blast(const Math::Vec3& point);
    void crash(const Math::Vec3& point, float volume);
    void setCrashSounds(const ct::String& paths);
    const ct::String& getCrashSounds() const;

protected:
    void notificationHandler(int what) override;

private:
    void build();
    void release();
    Math::Vec3 scatter(float lift);
    void loadSounds();

    TextureId mDisc;
    ParticleSystem3D* mSparks;
    ParticleSystem3D* mSmoke;
    ParticleSystem3D* mFlame;
    ParticleSystem3D* mBlack;
    int mFlameBudget;
    int mSparkBudget;
    int mSmokeBudget;
    ct::String mCrashSounds;
    ct::Vector<SoundId> mSounds;
    std::size_t mNextSound;
    bool mSoundsLoaded;
};

} // namespace prisma3d
