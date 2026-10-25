#include "prisma3d/scene/AudioStreamPlayer3D.h"

#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/ResourceProperties.h"

namespace prisma3d
{

AudioStreamPlayer3D::AudioStreamPlayer3D()
        : Node3D(),
          mPlayback(),
          mAutoplay(false)
{
    setNotifyTransform(true);
    mPlayback.setSpatial(true);
}

AudioStreamPlayer3D::~AudioStreamPlayer3D()
{
    mPlayback.stop();
}

void AudioStreamPlayer3D::setSound(SoundId sound)
{
    mPlayback.setSound(sound);
}

SoundId AudioStreamPlayer3D::getSound() const
{
    return mPlayback.getSound();
}

VoiceId AudioStreamPlayer3D::getVoice() const
{
    return mPlayback.getVoice();
}

void AudioStreamPlayer3D::setAutoplay(bool autoplay)
{
    mAutoplay = autoplay;
}

bool AudioStreamPlayer3D::isAutoplay() const
{
    return mAutoplay;
}

void AudioStreamPlayer3D::setLooping(bool looping)
{
    mPlayback.setLooping(looping);
}

bool AudioStreamPlayer3D::isLooping() const
{
    return mPlayback.isLooping();
}

void AudioStreamPlayer3D::setVolume(float volume)
{
    mPlayback.setVolume(volume);
}

float AudioStreamPlayer3D::getVolume() const
{
    return mPlayback.getVolume();
}

void AudioStreamPlayer3D::setPitch(float pitch)
{
    mPlayback.setPitch(pitch);
}

float AudioStreamPlayer3D::getPitch() const
{
    return mPlayback.getPitch();
}

void AudioStreamPlayer3D::setBus(AudioBus bus)
{
    mPlayback.setBus(bus);
}

AudioBus AudioStreamPlayer3D::getBus() const
{
    return mPlayback.getBus();
}

void AudioStreamPlayer3D::setUnitSize(float unitSize)
{
    mPlayback.setMinDistance(unitSize);
}

float AudioStreamPlayer3D::getUnitSize() const
{
    return mPlayback.getMinDistance();
}

void AudioStreamPlayer3D::setMaxDistance(float distance)
{
    mPlayback.setMaxDistance(distance);
}

float AudioStreamPlayer3D::getMaxDistance() const
{
    return mPlayback.getMaxDistance();
}

void AudioStreamPlayer3D::setRolloff(float rolloff)
{
    mPlayback.setRolloff(rolloff);
}

float AudioStreamPlayer3D::getRolloff() const
{
    return mPlayback.getRolloff();
}

void AudioStreamPlayer3D::setAttenuation(AudioAttenuation attenuation)
{
    mPlayback.setAttenuation(attenuation);
}

AudioAttenuation AudioStreamPlayer3D::getAttenuation() const
{
    return mPlayback.getAttenuation();
}

void AudioStreamPlayer3D::setVelocity(const Math::Vec3& velocity)
{
    mPlayback.setVelocity(velocity);
}

Math::Vec3 AudioStreamPlayer3D::getVelocity() const
{
    return mPlayback.getVelocity();
}

bool AudioStreamPlayer3D::play()
{
    mPlayback.setPosition(getGlobalPosition());
    return mPlayback.play();
}

void AudioStreamPlayer3D::stop()
{
    mPlayback.stop();
}

bool AudioStreamPlayer3D::pause()
{
    return mPlayback.pause();
}

bool AudioStreamPlayer3D::resume()
{
    return mPlayback.resume();
}

bool AudioStreamPlayer3D::isPlaying() const
{
    return mPlayback.isPlaying();
}

float AudioStreamPlayer3D::getPlaybackPosition() const
{
    return mPlayback.getPlaybackPosition();
}

bool AudioStreamPlayer3D::seek(float seconds)
{
    return mPlayback.seek(seconds);
}

void AudioStreamPlayer3D::notificationHandler(int what)
{
    Node3D::notificationHandler(what);

    switch (what)
    {
        case kNotificationReady:
            if (mAutoplay) play();
            break;
        case kNotificationTransformChanged:
            mPlayback.setPosition(getGlobalPosition());
            break;
        case kNotificationExitTree:
            mPlayback.stop();
            break;
        default:
            break;
    }
}

void AudioStreamPlayer3D::bindProperties()
{
    using DB = NodeClassDB;
    using P = AudioStreamPlayer3D;
    resourceprop::add<P, SoundId>(PropertyInfo::resource("stream", "AudioStream"), &P::setSound,
            &P::getSound);
    DB::addProperty<P>(
            PropertyInfo::enumeration("attenuation_model", "Inverse,Linear,Exponential,Disabled"),
            &P::setAttenuation, &P::getAttenuation);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "volume_linear", 0.0, 4.0, 0.01),
            &P::setVolume, &P::getVolume);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "unit_size", 0.0, 100.0, 0.01, true),
            &P::setUnitSize, &P::getUnitSize);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "pitch_scale", 0.01, 4.0, 0.01),
            &P::setPitch, &P::getPitch);
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "autoplay"), &P::setAutoplay,
            &P::isAutoplay);
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "loop"), &P::setLooping, &P::isLooping);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "max_distance", 0.0, 4096.0, 0.01, true),
            &P::setMaxDistance, &P::getMaxDistance);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "rolloff", 0.0, 100.0, 0.01),
            &P::setRolloff, &P::getRolloff);
    DB::addProperty<P>(PropertyInfo::enumeration("bus", "Sfx,Music"), &P::setBus, &P::getBus);
    DB::addProperty<P>(PropertyInfo(PropertyType::Vec3, "velocity"), &P::setVelocity,
            &P::getVelocity);
}

} // namespace prisma3d
