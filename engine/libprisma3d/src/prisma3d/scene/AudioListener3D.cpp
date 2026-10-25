#include "prisma3d/scene/AudioListener3D.h"

#include "prisma3d/audio/AudioServer.h"
#include "prisma3d/scene/NodeClassDB.h"

#include <ct/vector.hpp>

namespace prisma3d
{

namespace
{

ct::Vector<AudioListener3D*> sListeners;
AudioListener3D* sCurrent = nullptr;

} // namespace

AudioListener3D::AudioListener3D()
        : Node3D(),
          mVelocity(0.0f, 0.0f, 0.0f),
          mCurrent(false)
{
    setNotifyTransform(true);
}

AudioListener3D::~AudioListener3D()
{
    unregister();
}

void AudioListener3D::unregister()
{
    for (std::size_t i = 0; i < sListeners.size(); ++i)
    {
        if (sListeners[i] == this)
        {
            sListeners.erase(sListeners.begin() + i);
            break;
        }
    }
    if (sCurrent == this) sCurrent = nullptr;
    mCurrent = false;
}

void AudioListener3D::makeCurrent()
{
    mCurrent = true;
    if (!isInsideTree()) return;

    if (sCurrent && sCurrent != this) sCurrent->mCurrent = false;
    sCurrent = this;
    push();
}

void AudioListener3D::clearCurrent(bool enableNext)
{
    mCurrent = false;
    if (sCurrent != this) return;

    sCurrent = nullptr;
    if (!enableNext) return;
    for (std::size_t i = sListeners.size(); i > 0; --i)
    {
        if (sListeners[i - 1] != this)
        {
            sListeners[i - 1]->makeCurrent();
            return;
        }
    }
}

void AudioListener3D::setCurrent(bool current)
{
    if (current) makeCurrent();
    else
        clearCurrent();
}

bool AudioListener3D::isCurrent() const
{
    return sCurrent == this || (mCurrent && !isInsideTree());
}

AudioListener3D* AudioListener3D::getCurrent()
{
    return sCurrent;
}

void AudioListener3D::setVelocity(const Math::Vec3& velocity)
{
    mVelocity = velocity;
    if (sCurrent == this) push();
}

Math::Vec3 AudioListener3D::getVelocity() const
{
    return mVelocity;
}

void AudioListener3D::push()
{
    AudioServer* audio = AudioServer::singleton();
    if (!audio) return;
    audio->setListenerTransform(getGlobalTransform());
    audio->setListenerVelocity(mVelocity);
}

void AudioListener3D::notificationHandler(int what)
{
    Node3D::notificationHandler(what);

    switch (what)
    {
        case kNotificationEnterTree:
            sListeners.push_back(this);
            if (mCurrent || !sCurrent) makeCurrent();
            break;
        case kNotificationExitTree:
        {
            const bool wasCurrent = sCurrent == this;
            unregister();
            if (wasCurrent && !sListeners.empty()) sListeners.back()->makeCurrent();
            break;
        }
        case kNotificationTransformChanged:
            if (sCurrent == this) push();
            break;
        default:
            break;
    }
}

void AudioListener3D::bindProperties()
{
    using DB = NodeClassDB;
    DB::addProperty<AudioListener3D>(PropertyInfo(PropertyType::Bool, "current"),
            &AudioListener3D::setCurrent, &AudioListener3D::isCurrent);
    DB::addProperty<AudioListener3D>(PropertyInfo(PropertyType::Vec3, "velocity"),
            &AudioListener3D::setVelocity, &AudioListener3D::getVelocity);
}

} // namespace prisma3d
