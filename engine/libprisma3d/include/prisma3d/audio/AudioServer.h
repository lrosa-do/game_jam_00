#pragma once

#include "mathc.h"

#include <ct/slotmap32.hpp>
#include <ct/string.hpp>

#include <cstddef>
#include <cstdint>

namespace prisma3d
{

struct SoundTag;
using SoundId = ct::Handle32<SoundTag>;
struct VoiceTag;
using VoiceId = ct::Handle32<VoiceTag>;

struct AudioServerImpl;

constexpr float kAudioMaxVolume = 4.0f;
constexpr float kAudioMinPitch = 0.01f;
constexpr float kAudioMaxPitch = 4.0f;
constexpr float kAudioMaxDistance = 1000000.0f;
constexpr float kAudioMaxRolloff = 100.0f;

enum class AudioBus : std::uint8_t
{
    Sfx,
    Music,
    Count
};

enum class AudioAttenuation : std::uint8_t
{
    Inverse,
    Linear,
    Exponential,
    None
};

enum class VoiceState : std::uint8_t
{
    Stopped,
    Playing,
    Paused
};

struct AudioServerDesc
{
    bool nullBackend = false;
    bool manual = false;
    std::uint32_t sampleRate = 0;
    std::uint32_t channels = 0;
};

struct SoundInfo
{
    std::uint64_t frames = 0;
    std::uint32_t sampleRate = 0;
    std::uint32_t channels = 0;
    double duration = 0.0;
    std::size_t byteSize = 0;
    ct::String path;
    bool loading = false;
};

class AudioServer
{
public:
    AudioServer();
    ~AudioServer();
    AudioServer(const AudioServer&) = delete;
    AudioServer& operator=(const AudioServer&) = delete;

    static AudioServer* singleton();

    bool init(const AudioServerDesc& desc);
    void shutdown();
    bool isReady() const;
    bool isManual() const;
    std::uint32_t getSampleRate() const;
    std::uint32_t getChannels() const;

    void update();
    std::uint32_t mix(float* output, std::uint32_t frames);

    SoundId createSoundFromFile(const ct::String& path);
    SoundId requestSoundFromFile(const ct::String& path);
    SoundId createSoundFromMemory(const void* data, std::size_t size);
    SoundId findSoundByPath(const ct::String& path) const;
    void destroySound(SoundId sound);
    bool validSound(SoundId sound) const;
    const SoundInfo* getSoundInfo(SoundId sound) const;
    std::size_t getSoundCount() const;

    VoiceId createVoice(SoundId sound, AudioBus bus = AudioBus::Sfx);
    void destroyVoice(VoiceId voice);
    bool validVoice(VoiceId voice) const;
    std::size_t getVoiceCount() const;

    VoiceId playOneShot(SoundId sound, float volume = 1.0f, float pitch = 1.0f, float pan = 0.0f,
            AudioBus bus = AudioBus::Sfx);
    VoiceId playOneShotAt(SoundId sound, const Math::Vec3& position, float volume = 1.0f,
            float pitch = 1.0f, float minDistance = 1.0f, float maxDistance = 100.0f,
            float rolloff = 1.0f, AudioBus bus = AudioBus::Sfx);

    bool play(VoiceId voice);
    bool stop(VoiceId voice);
    bool pause(VoiceId voice);
    bool resume(VoiceId voice);
    VoiceState getState(VoiceId voice) const;
    bool isPlaying(VoiceId voice) const;
    // Seconds into the sound being played; 0 before it starts or without an audio device.
    float getPlaybackPosition(VoiceId voice) const;
    bool seek(VoiceId voice, float seconds);
    bool fadeIn(VoiceId voice, float seconds);
    bool fadeOut(VoiceId voice, float seconds, bool releaseWhenDone = false);

    SoundId getVoiceSound(VoiceId voice) const;
    bool setLooping(VoiceId voice, bool looping);
    bool isLooping(VoiceId voice) const;
    bool setVolume(VoiceId voice, float volume);
    float getVolume(VoiceId voice) const;
    bool setPitch(VoiceId voice, float pitch);
    float getPitch(VoiceId voice) const;
    bool setPan(VoiceId voice, float pan);
    float getPan(VoiceId voice) const;
    bool setBus(VoiceId voice, AudioBus bus);
    AudioBus getBus(VoiceId voice) const;

    bool setSpatial(VoiceId voice, bool spatial);
    bool isSpatial(VoiceId voice) const;
    bool setPosition(VoiceId voice, const Math::Vec3& position);
    Math::Vec3 getPosition(VoiceId voice) const;
    bool setVelocity(VoiceId voice, const Math::Vec3& velocity);
    Math::Vec3 getVelocity(VoiceId voice) const;
    bool setDistances(VoiceId voice, float minDistance, float maxDistance);
    float getMinDistance(VoiceId voice) const;
    float getMaxDistance(VoiceId voice) const;
    bool setRolloff(VoiceId voice, float rolloff);
    float getRolloff(VoiceId voice) const;
    bool setAttenuation(VoiceId voice, AudioAttenuation attenuation);
    AudioAttenuation getAttenuation(VoiceId voice) const;

    void setListenerTransform(const Math::Mat4& transform);
    Math::Mat4 getListenerTransform() const;
    void setListenerVelocity(const Math::Vec3& velocity);
    Math::Vec3 getListenerVelocity() const;

    void setMasterVolume(float volume);
    float getMasterVolume() const;
    void setMasterMuted(bool muted);
    bool isMasterMuted() const;
    void setBusVolume(AudioBus bus, float volume);
    float getBusVolume(AudioBus bus) const;
    void setBusMuted(AudioBus bus, bool muted);
    void setBusPitch(AudioBus bus, float pitch);
    float getBusPitch(AudioBus bus) const;
    bool isBusMuted(AudioBus bus) const;
    float getEffectiveVolume(VoiceId voice) const;

private:
    static AudioServer* sSingleton;

    AudioServerImpl* mImpl;
};

} // namespace prisma3d
