#pragma once

#include "prisma3d/audio/AudioPlayback.h"
#include "prisma3d/scene/Node.h"

namespace prisma3d
{

class AudioStreamPlayer : public Node
{
public:
    PRISMA3D_NODE_CLASS(AudioStreamPlayer, Node);
    static void bindProperties();

    AudioStreamPlayer();
    ~AudioStreamPlayer() override;

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
    void setPan(float pan);
    float getPan() const;
    void setBus(AudioBus bus);
    AudioBus getBus() const;

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
