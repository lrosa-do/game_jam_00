#pragma once

#include "prisma3d/audio/AudioPlayback.h"
#include "prisma3d/scene/Node3D.h"

namespace prisma3d
{

class AudioStreamPlayer3D : public Node3D
{
public:
    PRISMA3D_NODE_CLASS(AudioStreamPlayer3D, Node3D);
    static void bindProperties();

    AudioStreamPlayer3D();
    ~AudioStreamPlayer3D() override;

    void setSound(SoundId sound);
    SoundId getSound() const;
    VoiceId getVoice() const;

    void setAutoplay(bool autoplay);
    bool isAutoplay() const;
    void setLooping(bool looping);
    bool isLooping() const;
    void setVolume(float volume);
    float getVolume() const;
    void setPitch(float pitch);
    float getPitch() const;
    void setBus(AudioBus bus);
    AudioBus getBus() const;

    void setUnitSize(float unitSize);
    float getUnitSize() const;
    void setMaxDistance(float distance);
    float getMaxDistance() const;
    void setRolloff(float rolloff);
    float getRolloff() const;
    void setAttenuation(AudioAttenuation attenuation);
    AudioAttenuation getAttenuation() const;
    void setVelocity(const Math::Vec3& velocity);
    Math::Vec3 getVelocity() const;

    bool play();
    void stop();
    bool pause();
    bool resume();
    bool isPlaying() const;
    float getPlaybackPosition() const;
    bool seek(float seconds);

protected:
    void notificationHandler(int what) override;

private:
    AudioPlayback mPlayback;
    bool mAutoplay;
};

} // namespace prisma3d
