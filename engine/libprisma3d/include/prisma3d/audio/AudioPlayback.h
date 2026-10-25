#pragma once

#include "prisma3d/audio/AudioServer.h"

namespace prisma3d
{

class AudioPlayback
{
public:
    AudioPlayback();
    ~AudioPlayback();
    AudioPlayback(const AudioPlayback&) = delete;
    AudioPlayback& operator=(const AudioPlayback&) = delete;

    void setSound(SoundId sound);
    SoundId getSound() const;
    VoiceId getVoice() const;

    void setLooping(bool looping);
    bool isLooping() const;
    void setVolume(float volume);
    float getVolume() const;
    void setPitch(float pitch);
    float getPitch() const;
    void setPan(float pan);
    float getPan() const;
    void setBus(AudioBus bus);
    AudioBus getBus() const;

    void setSpatial(bool spatial);
    bool isSpatial() const;
    void setPosition(const Math::Vec3& position);
    Math::Vec3 getPosition() const;
    void setVelocity(const Math::Vec3& velocity);
    Math::Vec3 getVelocity() const;
    void setMinDistance(float distance);
    float getMinDistance() const;
    void setMaxDistance(float distance);
    float getMaxDistance() const;
    void setRolloff(float rolloff);
    float getRolloff() const;
    void setAttenuation(AudioAttenuation attenuation);
    AudioAttenuation getAttenuation() const;

    bool play();
    void stop();
    bool pause();
    bool resume();
    bool isPlaying() const;
    float getPlaybackPosition() const;
    bool seek(float seconds);
    VoiceState getState() const;

private:
    AudioServer* server() const;
    void push();
    void pushDistances();

    SoundId mSound;
    VoiceId mVoice;
    bool mLooping;
    bool mSpatial;
    AudioBus mBus;
    AudioAttenuation mAttenuation;
    float mVolume;
    float mPitch;
    float mPan;
    float mMinDistance;
    float mMaxDistance;
    float mRolloff;
    Math::Vec3 mPosition;
    Math::Vec3 mVelocity;
};

} // namespace prisma3d
