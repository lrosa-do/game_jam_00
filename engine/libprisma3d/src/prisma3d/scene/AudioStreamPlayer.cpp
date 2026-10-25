#include "prisma3d/scene/AudioStreamPlayer.h"

#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/ResourceProperties.h"

namespace prisma3d
{

AudioStreamPlayer::AudioStreamPlayer()
        : Node(),
          mPlayback(),
          mAutoplay(false)
{
}

AudioStreamPlayer::~AudioStreamPlayer()
{
    mPlayback.stop();
}

void AudioStreamPlayer::setSound(SoundId sound)
{
    mPlayback.setSound(sound);
}

SoundId AudioStreamPlayer::getSound() const
{
    return mPlayback.getSound();
}

VoiceId AudioStreamPlayer::getVoice() const
{
    return mPlayback.getVoice();
}

void AudioStreamPlayer::setAutoplay(bool autoplay)
{
    mAutoplay = autoplay;
}

bool AudioStreamPlayer::isAutoplay() const
{
    return mAutoplay;
}

void AudioStreamPlayer::setLooping(bool looping)
{
    mPlayback.setLooping(looping);
}

bool AudioStreamPlayer::isLooping() const
{
    return mPlayback.isLooping();
}

void AudioStreamPlayer::setVolume(float volume)
{
    mPlayback.setVolume(volume);
}

float AudioStreamPlayer::getVolume() const
{
    return mPlayback.getVolume();
}

void AudioStreamPlayer::setPitch(float pitch)
{
    mPlayback.setPitch(pitch);
}

float AudioStreamPlayer::getPitch() const
{
    return mPlayback.getPitch();
}

void AudioStreamPlayer::setPan(float pan)
{
    mPlayback.setPan(pan);
}

float AudioStreamPlayer::getPan() const
{
    return mPlayback.getPan();
}

void AudioStreamPlayer::setBus(AudioBus bus)
{
    mPlayback.setBus(bus);
}

AudioBus AudioStreamPlayer::getBus() const
{
    return mPlayback.getBus();
}

bool AudioStreamPlayer::play()
{
    return mPlayback.play();
}

void AudioStreamPlayer::stop()
{
    mPlayback.stop();
}

bool AudioStreamPlayer::pause()
{
    return mPlayback.pause();
}

bool AudioStreamPlayer::resume()
{
    return mPlayback.resume();
}

bool AudioStreamPlayer::isPlaying() const
{
    return mPlayback.isPlaying();
}

float AudioStreamPlayer::getPlaybackPosition() const
{
    return mPlayback.getPlaybackPosition();
}

bool AudioStreamPlayer::seek(float seconds)
{
    return mPlayback.seek(seconds);
}

void AudioStreamPlayer::notificationHandler(int what)
{
    Node::notificationHandler(what);

    switch (what)
    {
        case kNotificationReady:
            if (mAutoplay) play();
            break;
        case kNotificationExitTree:
            mPlayback.stop();
            break;
        default:
            break;
    }
}

void AudioStreamPlayer::bindProperties()
{
    using DB = NodeClassDB;
    resourceprop::add<AudioStreamPlayer, SoundId>(PropertyInfo::resource("stream", "AudioStream"),
            &AudioStreamPlayer::setSound, &AudioStreamPlayer::getSound);
    DB::addProperty<AudioStreamPlayer>(PropertyInfo(PropertyType::Bool, "autoplay"),
            &AudioStreamPlayer::setAutoplay, &AudioStreamPlayer::isAutoplay);
    DB::addProperty<AudioStreamPlayer>(PropertyInfo(PropertyType::Bool, "loop"),
            &AudioStreamPlayer::setLooping, &AudioStreamPlayer::isLooping);
    DB::addProperty<AudioStreamPlayer>(
            PropertyInfo::range(PropertyType::Float, "volume_linear", 0.0, 4.0, 0.01),
            &AudioStreamPlayer::setVolume, &AudioStreamPlayer::getVolume);
    DB::addProperty<AudioStreamPlayer>(
            PropertyInfo::range(PropertyType::Float, "pitch_scale", 0.01, 4.0, 0.01),
            &AudioStreamPlayer::setPitch, &AudioStreamPlayer::getPitch);
    DB::addProperty<AudioStreamPlayer>(
            PropertyInfo::range(PropertyType::Float, "pan", -1.0, 1.0, 0.01),
            &AudioStreamPlayer::setPan, &AudioStreamPlayer::getPan);
    DB::addProperty<AudioStreamPlayer>(PropertyInfo::enumeration("bus", "Sfx,Music"),
            &AudioStreamPlayer::setBus, &AudioStreamPlayer::getBus);
}

} // namespace prisma3d
