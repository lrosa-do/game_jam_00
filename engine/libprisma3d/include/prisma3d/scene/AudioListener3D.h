#pragma once

#include "prisma3d/scene/Node3D.h"

namespace prisma3d
{

class AudioListener3D : public Node3D
{
public:
    PRISMA3D_NODE_CLASS(AudioListener3D, Node3D);
    static void bindProperties();

    AudioListener3D();
    ~AudioListener3D() override;

    void makeCurrent();
    void clearCurrent(bool enableNext = true);
    void setCurrent(bool current);
    bool isCurrent() const;
    static AudioListener3D* getCurrent();

    void setVelocity(const Math::Vec3& velocity);
    Math::Vec3 getVelocity() const;

protected:
    void notificationHandler(int what) override;

private:
    void unregister();
    void push();

    Math::Vec3 mVelocity;
    bool mCurrent;
};

} // namespace prisma3d
