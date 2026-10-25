#include "prisma3d/audio/AudioPlayback.h"

namespace prisma3d
{

namespace
{

float clampValue(float value, float minimum, float maximum)
{
    return value < minimum ? minimum : (value > maximum ? maximum : value);
}

} // namespace

AudioPlayback::AudioPlayback()
        : mSound(),
          mVoice(),
          mLooping(false),
          mSpatial(false),
          mBus(AudioBus::Sfx),
          mAttenuation(AudioAttenuation::Inverse),
          mVolume(1.0f),
          mPitch(1.0f),
          mPan(0.0f),
          mMinDistance(1.0f),
          mMaxDistance(100.0f),
          mRolloff(1.0f),
          mPosition(0.0f, 0.0f, 0.0f),
          mVelocity(0.0f, 0.0f, 0.0f)
{
}

AudioPlayback::~AudioPlayback()
{
    stop();
}

AudioServer* AudioPlayback::server() const
{
    return AudioServer::singleton();
}

void AudioPlayback::push()
{
    AudioServer* audio = server();
    if (!audio || !mVoice.valid()) return;
    audio->setLooping(mVoice, mLooping);
    audio->setVolume(mVoice, mVolume);
    audio->setPitch(mVoice, mPitch);
    audio->setPan(mVoice, mPan);
    audio->setDistances(mVoice, mMinDistance, mMaxDistance);
    audio->setRolloff(mVoice, mRolloff);
    audio->setAttenuation(mVoice, mAttenuation);
    audio->setPosition(mVoice, mPosition);
    audio->setVelocity(mVoice, mVelocity);
    audio->setSpatial(mVoice, mSpatial);
}

void AudioPlayback::pushDistances()
{
    AudioServer* audio = server();
    if (audio && mVoice.valid()) audio->setDistances(mVoice, mMinDistance, mMaxDistance);
}

void AudioPlayback::setSound(SoundId sound)
{
    if (mSound == sound) return;
    stop();
    mSound = sound;
}

SoundId AudioPlayback::getSound() const
{
    return mSound;
}

VoiceId AudioPlayback::getVoice() const
{
    return mVoice;
}

void AudioPlayback::setLooping(bool looping)
{
    mLooping = looping;
    AudioServer* audio = server();
    if (audio && mVoice.valid()) audio->setLooping(mVoice, mLooping);
}

bool AudioPlayback::isLooping() const
{
    return mLooping;
}

void AudioPlayback::setVolume(float volume)
{
    mVolume = clampValue(volume, 0.0f, kAudioMaxVolume);
    AudioServer* audio = server();
    if (audio && mVoice.valid()) audio->setVolume(mVoice, mVolume);
}

float AudioPlayback::getVolume() const
{
    return mVolume;
}

void AudioPlayback::setPitch(float pitch)
{
    mPitch = clampValue(pitch, kAudioMinPitch, kAudioMaxPitch);
    AudioServer* audio = server();
    if (audio && mVoice.valid()) audio->setPitch(mVoice, mPitch);
}

float AudioPlayback::getPitch() const
{
    return mPitch;
}

void AudioPlayback::setPan(float pan)
{
    mPan = clampValue(pan, -1.0f, 1.0f);
    AudioServer* audio = server();
    if (audio && mVoice.valid()) audio->setPan(mVoice, mPan);
}

float AudioPlayback::getPan() const
{
    return mPan;
}

void AudioPlayback::setBus(AudioBus bus)
{
    mBus = bus;
    AudioServer* audio = server();
    if (audio && mVoice.valid()) audio->setBus(mVoice, mBus);
}

AudioBus AudioPlayback::getBus() const
{
    return mBus;
}

void AudioPlayback::setSpatial(bool spatial)
{
    mSpatial = spatial;
    AudioServer* audio = server();
    if (audio && mVoice.valid()) audio->setSpatial(mVoice, mSpatial);
}

bool AudioPlayback::isSpatial() const
{
    return mSpatial;
}

void AudioPlayback::setPosition(const Math::Vec3& position)
{
    mPosition = position;
    AudioServer* audio = server();
    if (audio && mVoice.valid()) audio->setPosition(mVoice, mPosition);
}

Math::Vec3 AudioPlayback::getPosition() const
{
    return mPosition;
}

void AudioPlayback::setVelocity(const Math::Vec3& velocity)
{
    mVelocity = velocity;
    AudioServer* audio = server();
    if (audio && mVoice.valid()) audio->setVelocity(mVoice, mVelocity);
}

Math::Vec3 AudioPlayback::getVelocity() const
{
    return mVelocity;
}

void AudioPlayback::setMinDistance(float distance)
{
    mMinDistance = clampValue(distance, 0.0f, kAudioMaxDistance);
    if (mMaxDistance < mMinDistance) mMaxDistance = mMinDistance;
    pushDistances();
}

float AudioPlayback::getMinDistance() const
{
    return mMinDistance;
}

void AudioPlayback::setMaxDistance(float distance)
{
    mMaxDistance = clampValue(distance, mMinDistance, kAudioMaxDistance);
    pushDistances();
}

float AudioPlayback::getMaxDistance() const
{
    return mMaxDistance;
}

void AudioPlayback::setRolloff(float rolloff)
{
    mRolloff = clampValue(rolloff, 0.0f, kAudioMaxRolloff);
    AudioServer* audio = server();
    if (audio && mVoice.valid()) audio->setRolloff(mVoice, mRolloff);
}

float AudioPlayback::getRolloff() const
{
    return mRolloff;
}

void AudioPlayback::setAttenuation(AudioAttenuation attenuation)
{
    mAttenuation = attenuation;
    AudioServer* audio = server();
    if (audio && mVoice.valid()) audio->setAttenuation(mVoice, mAttenuation);
}

AudioAttenuation AudioPlayback::getAttenuation() const
{
    return mAttenuation;
}

bool AudioPlayback::play()
{
    AudioServer* audio = server();
    if (!audio || !audio->validSound(mSound)) return false;
    stop();
    mVoice = audio->createVoice(mSound, mBus);
    if (!mVoice.valid()) return false;
    push();
    if (!audio->play(mVoice))
    {
        stop();
        return false;
    }
    return true;
}

void AudioPlayback::stop()
{
    AudioServer* audio = server();
    if (audio && mVoice.valid()) audio->destroyVoice(mVoice);
    mVoice = VoiceId();
}

bool AudioPlayback::pause()
{
    AudioServer* audio = server();
    return audio && mVoice.valid() && audio->pause(mVoice);
}

bool AudioPlayback::resume()
{
    AudioServer* audio = server();
    return audio && mVoice.valid() && audio->resume(mVoice);
}

bool AudioPlayback::isPlaying() const
{
    AudioServer* audio = server();
    return audio && mVoice.valid() && audio->isPlaying(mVoice);
}

float AudioPlayback::getPlaybackPosition() const
{
    AudioServer* audio = server();
    return audio && mVoice.valid() ? audio->getPlaybackPosition(mVoice) : 0.0f;
}

bool AudioPlayback::seek(float seconds)
{
    AudioServer* audio = server();
    return audio && mVoice.valid() && audio->seek(mVoice, seconds);
}

VoiceState AudioPlayback::getState() const
{
    AudioServer* audio = server();
    if (!audio || !mVoice.valid()) return VoiceState::Stopped;
    return audio->getState(mVoice);
}

} // namespace prisma3d
