#include "prisma3d/audio/AudioServer.h"

#include "prisma3d/core/FileSystem.h"

#include "miniaudio.h"
#include "platform.h"

#include <ct/thread.hpp>
#include <ct/threadpool.hpp>
#include <ct/vector.hpp>

#include <math.h>
#include <string.h>

namespace prisma3d
{

namespace
{

constexpr std::size_t kBusCount = static_cast<std::size_t>(AudioBus::Count);
constexpr std::uint32_t kManualSampleRate = 48000;
constexpr std::uint32_t kManualChannels = 2;
constexpr ma_uint64 kNoStopTime = ~static_cast<ma_uint64>(0);

float clampValue(float value, float minimum, float maximum)
{
    return value < minimum ? minimum : (value > maximum ? maximum : value);
}

ma_attenuation_model toModel(AudioAttenuation attenuation)
{
    switch (attenuation)
    {
        case AudioAttenuation::Linear:
            return ma_attenuation_model_linear;
        case AudioAttenuation::Exponential:
            return ma_attenuation_model_exponential;
        case AudioAttenuation::None:
            return ma_attenuation_model_none;
        default:
            return ma_attenuation_model_inverse;
    }
}

ma_uint64 milliseconds(float seconds)
{
    return seconds > 0.0f ? static_cast<ma_uint64>(seconds * 1000.0f + 0.5f) : 0;
}

std::size_t busIndex(AudioBus bus)
{
    const std::size_t index = static_cast<std::size_t>(bus);
    return index < kBusCount ? index : 0;
}

} // namespace

struct AudioSound
{
    ct::Vector<std::uint8_t> bytes;
    SoundInfo info;
};

struct AudioVoice
{
    AudioVoice()
            : sound(),
              state(VoiceState::Stopped),
              autoRelease(false),
              hasDecoder(false),
              hasPlayer(false),
              pendingPlay(false),
              looping(false),
              spatial(false),
              bus(AudioBus::Sfx),
              volume(1.0f),
              pitch(1.0f),
              pan(0.0f),
              position(0.0f, 0.0f, 0.0f),
              velocity(0.0f, 0.0f, 0.0f),
              minDistance(1.0f),
              maxDistance(100.0f),
              rolloff(1.0f),
              attenuation(AudioAttenuation::Inverse)
    {
        memset(&decoder, 0, sizeof(decoder));
        memset(&player, 0, sizeof(player));
    }

    SoundId sound;
    VoiceState state;
    bool autoRelease;
    bool hasDecoder;
    bool hasPlayer;
    bool pendingPlay;
    bool looping;
    bool spatial;
    AudioBus bus;
    float volume;
    float pitch;
    float pan;
    Math::Vec3 position;
    Math::Vec3 velocity;
    float minDistance;
    float maxDistance;
    float rolloff;
    AudioAttenuation attenuation;
    ma_decoder decoder;
    ma_sound player;
};

typedef ct::SlotMap32<AudioSound*> SoundMap;
typedef SoundMap::handle_type SoundKey;
typedef ct::SlotMap32<AudioVoice*> VoiceMap;
typedef VoiceMap::handle_type VoiceKey;

struct LoadedSound
{
    SoundKey key;
    ct::Vector<std::uint8_t> bytes;
    SoundInfo info;
    bool ok = false;
};

struct AudioServerImpl
{
    AudioServerImpl()
            : contextReady(false),
              engineReady(false),
              manual(false),
              masterVolume(1.0f),
              masterMuted(false),
              listener(Math::Mat4::Identity()),
              listenerVelocity(0.0f, 0.0f, 0.0f),
              loader(nullptr)
    {
        memset(&context, 0, sizeof(context));
        memset(&engine, 0, sizeof(engine));
        memset(groups, 0, sizeof(groups));
        for (std::size_t i = 0; i < kBusCount; ++i)
        {
            groupReady[i] = false;
            busVolume[i] = 1.0f;
            busPitch[i] = 1.0f;
            busMuted[i] = false;
        }
    }

    ma_context context;
    ma_engine engine;
    ma_sound_group groups[kBusCount];
    bool groupReady[kBusCount];
    bool contextReady;
    bool engineReady;
    bool manual;
    float masterVolume;
    bool masterMuted;
    float busVolume[kBusCount];
    float busPitch[kBusCount];
    bool busMuted[kBusCount];
    Math::Mat4 listener;
    Math::Vec3 listenerVelocity;
    SoundMap sounds;
    VoiceMap voices;
    ct::ThreadPool* loader;
    ct::Mutex loadedMutex;
    ct::Vector<LoadedSound*> loaded;
};

namespace
{

SoundKey soundKey(SoundId id)
{
    return SoundKey::from_bits(id.bits());
}

VoiceKey voiceKey(VoiceId id)
{
    return VoiceKey::from_bits(id.bits());
}

SoundId soundId(SoundKey key)
{
    return SoundId::from_bits(key.bits());
}

VoiceId voiceId(VoiceKey key)
{
    return VoiceId::from_bits(key.bits());
}

AudioVoice* findVoice(const AudioServerImpl* impl, VoiceId id)
{
    if (!impl || !impl->engineReady) return nullptr;
    AudioVoice* const* entry = impl->voices.get(voiceKey(id));
    return entry ? *entry : nullptr;
}

AudioSound* findSound(const AudioServerImpl* impl, SoundId id)
{
    if (!impl || !impl->engineReady) return nullptr;
    AudioSound* const* entry = impl->sounds.get(soundKey(id));
    return entry ? *entry : nullptr;
}

void applyMixer(AudioServerImpl* impl)
{
    if (!impl->engineReady) return;
    ma_engine_set_volume(&impl->engine, impl->masterMuted ? 0.0f : impl->masterVolume);
    for (std::size_t i = 0; i < kBusCount; ++i)
    {
        if (!impl->groupReady[i]) continue;
        ma_sound_group_set_volume(&impl->groups[i], impl->busMuted[i] ? 0.0f : impl->busVolume[i]);
        ma_sound_group_set_pitch(&impl->groups[i], impl->busPitch[i]);
    }
}

void applyListener(AudioServerImpl* impl)
{
    if (!impl->engineReady) return;
    const Math::Mat4& m = impl->listener;
    Math::Vec3 forward(-m.col2.x, -m.col2.y, -m.col2.z);
    Math::Vec3 up(m.col1.x, m.col1.y, m.col1.z);
    ma_engine_listener_set_position(&impl->engine, 0, m.col3.x, m.col3.y, m.col3.z);
    if (forward.Length() > 1e-6f)
    {
        forward = forward.Normalized();
        ma_engine_listener_set_direction(&impl->engine, 0, forward.x, forward.y, forward.z);
    }
    if (up.Length() > 1e-6f)
    {
        up = up.Normalized();
        ma_engine_listener_set_world_up(&impl->engine, 0, up.x, up.y, up.z);
    }
    ma_engine_listener_set_velocity(&impl->engine, 0, impl->listenerVelocity.x,
            impl->listenerVelocity.y, impl->listenerVelocity.z);
}

void refreshState(AudioVoice* voice)
{
    if (voice->state != VoiceState::Playing || !voice->hasPlayer) return;
    if (!voice->looping && ma_sound_at_end(&voice->player) == MA_TRUE)
    {
        ma_sound_stop(&voice->player);
        voice->state = VoiceState::Stopped;
        return;
    }
    if (ma_sound_is_playing(&voice->player) != MA_TRUE) voice->state = VoiceState::Stopped;
}

void releaseVoice(AudioVoice* voice)
{
    if (voice->hasPlayer) ma_sound_uninit(&voice->player);
    if (voice->hasDecoder) ma_decoder_uninit(&voice->decoder);
    delete voice;
}

void destroyVoiceByKey(AudioServerImpl* impl, VoiceKey key)
{
    AudioVoice* const* entry = impl->voices.get(key);
    if (!entry) return;
    AudioVoice* voice = *entry;
    impl->voices.erase(key);
    releaseVoice(voice);
}

void destroyAllVoices(AudioServerImpl* impl)
{
    for (AudioVoice*& voice: impl->voices) releaseVoice(voice);
    impl->voices.clear();
}

void destroyAllSounds(AudioServerImpl* impl)
{
    for (AudioSound*& sound: impl->sounds) delete sound;
    impl->sounds.clear();
}

void applySpatial(AudioVoice* voice)
{
    ma_sound* player = &voice->player;
    ma_sound_set_spatialization_enabled(player, voice->spatial ? MA_TRUE : MA_FALSE);
    ma_sound_set_positioning(player, ma_positioning_absolute);
    ma_sound_set_attenuation_model(player, toModel(voice->attenuation));
    ma_sound_set_min_distance(player, voice->minDistance);
    ma_sound_set_max_distance(player, voice->maxDistance);
    ma_sound_set_rolloff(player, voice->rolloff);
    ma_sound_set_position(player, voice->position.x, voice->position.y, voice->position.z);
    ma_sound_set_velocity(player, voice->velocity.x, voice->velocity.y, voice->velocity.z);
}

bool probeSound(const void* data, std::size_t size, SoundInfo& info)
{
    ma_decoder decoder;
    const ma_decoder_config config = ma_decoder_config_init_default();
    if (ma_decoder_init_memory(data, size, &config, &decoder) != MA_SUCCESS) return false;

    ma_format format = ma_format_unknown;
    ma_uint32 channels = 0;
    ma_uint32 sampleRate = 0;
    ma_uint64 frames = 0;
    ma_decoder_get_data_format(&decoder, &format, &channels, &sampleRate, nullptr, 0);
    ma_decoder_get_length_in_pcm_frames(&decoder, &frames);
    ma_decoder_uninit(&decoder);
    if (channels == 0 || sampleRate == 0) return false;

    info.frames = frames;
    info.sampleRate = sampleRate;
    info.channels = channels;
    info.duration = static_cast<double>(frames) / static_cast<double>(sampleRate);
    info.byteSize = size;
    return true;
}

bool initPlayer(AudioServerImpl* impl, AudioVoice* voice, const AudioSound* source)
{
    const ma_decoder_config config = ma_decoder_config_init_default();
    if (ma_decoder_init_memory(source->bytes.data(), source->bytes.size(), &config,
                &voice->decoder) != MA_SUCCESS)
        return false;
    voice->hasDecoder = true;

    if (ma_sound_init_from_data_source(&impl->engine, &voice->decoder, 0,
                &impl->groups[busIndex(voice->bus)], &voice->player) != MA_SUCCESS)
        return false;
    voice->hasPlayer = true;

    ma_sound_set_looping(&voice->player, voice->looping ? MA_TRUE : MA_FALSE);
    ma_sound_set_volume(&voice->player, voice->volume);
    ma_sound_set_pitch(&voice->player, voice->pitch);
    ma_sound_set_pan(&voice->player, voice->pan);
    applySpatial(voice);
    return true;
}

void finishLoads(AudioServerImpl* impl)
{
    ct::Vector<LoadedSound*> done;
    {
        ct::LockGuard guard(impl->loadedMutex);
        done.swap(impl->loaded);
    }
    for (std::size_t i = 0; i < done.size(); ++i)
    {
        LoadedSound* result = done[i];
        AudioSound* const* entry = impl->sounds.get(result->key);
        AudioSound* sound = entry ? *entry : nullptr;
        if (sound && sound->info.loading)
        {
            const ct::String path = sound->info.path;
            if (result->ok)
            {
                sound->bytes.swap(result->bytes);
                sound->info = result->info;
            }
            else
            {
                log_error("audio: %s is not a supported sound", path.c_str());
            }
            sound->info.path = path;
            sound->info.loading = false;

            const SoundId id = soundId(result->key);
            for (std::size_t v = 0; v < impl->voices.size(); ++v)
            {
                AudioVoice* voice = impl->voices.items()[v];
                if (voice->sound != id || voice->hasPlayer) continue;
                if (!result->ok || !initPlayer(impl, voice, sound))
                {
                    voice->pendingPlay = false;
                    voice->state = VoiceState::Stopped;
                    continue;
                }
                if (voice->pendingPlay)
                {
                    voice->pendingPlay = false;
                    if (ma_sound_start(&voice->player) != MA_SUCCESS)
                        voice->state = VoiceState::Stopped;
                }
            }
        }
        delete result;
    }
}

void stopLoader(AudioServerImpl* impl)
{
    if (impl->loader)
    {
        impl->loader->wait_all();
        delete impl->loader;
        impl->loader = nullptr;
    }
    ct::LockGuard guard(impl->loadedMutex);
    for (std::size_t i = 0; i < impl->loaded.size(); ++i) delete impl->loaded[i];
    impl->loaded.clear();
}

} // namespace

AudioServer* AudioServer::sSingleton = nullptr;

AudioServer::AudioServer()
        : mImpl(new AudioServerImpl())
{
    sSingleton = this;
}

AudioServer::~AudioServer()
{
    shutdown();
    if (sSingleton == this) sSingleton = nullptr;
    delete mImpl;
    mImpl = nullptr;
}

AudioServer* AudioServer::singleton()
{
    return sSingleton;
}

bool AudioServer::init(const AudioServerDesc& desc)
{
    if (mImpl->engineReady) return true;

    ma_engine_config config = ma_engine_config_init();
    if (desc.manual)
    {
        config.noDevice = MA_TRUE;
        config.channels = desc.channels != 0 ? desc.channels : kManualChannels;
        config.sampleRate = desc.sampleRate != 0 ? desc.sampleRate : kManualSampleRate;
    }
    else
    {
        config.channels = desc.channels;
        config.sampleRate = desc.sampleRate;
        if (desc.nullBackend)
        {
            const ma_backend backends[1] = { ma_backend_null };
            const ma_context_config contextConfig = ma_context_config_init();
            if (ma_context_init(backends, 1, &contextConfig, &mImpl->context) != MA_SUCCESS)
            {
                log_error("audio: the null backend could not start");
                return false;
            }
            mImpl->contextReady = true;
            config.pContext = &mImpl->context;
        }
    }

    if (ma_engine_init(&config, &mImpl->engine) != MA_SUCCESS)
    {
        log_error("audio: no output device, playback is silent this run");
        if (mImpl->contextReady)
        {
            ma_context_uninit(&mImpl->context);
            mImpl->contextReady = false;
        }
        return false;
    }

    mImpl->engineReady = true;
    mImpl->manual = desc.manual;
    for (std::size_t i = 0; i < kBusCount; ++i)
    {
        if (ma_sound_group_init(&mImpl->engine, 0, nullptr, &mImpl->groups[i]) != MA_SUCCESS)
        {
            log_error("audio: a bus could not be created");
            shutdown();
            return false;
        }
        mImpl->groupReady[i] = true;
    }

    applyMixer(mImpl);
    applyListener(mImpl);
    return true;
}

void AudioServer::shutdown()
{
    stopLoader(mImpl);
    destroyAllVoices(mImpl);
    destroyAllSounds(mImpl);
    for (std::size_t i = kBusCount; i > 0; --i)
    {
        if (!mImpl->groupReady[i - 1]) continue;
        ma_sound_group_uninit(&mImpl->groups[i - 1]);
        mImpl->groupReady[i - 1] = false;
    }
    if (mImpl->engineReady)
    {
        ma_engine_uninit(&mImpl->engine);
        mImpl->engineReady = false;
    }
    if (mImpl->contextReady)
    {
        ma_context_uninit(&mImpl->context);
        mImpl->contextReady = false;
    }
    mImpl->manual = false;
}

bool AudioServer::isReady() const
{
    return mImpl->engineReady;
}

bool AudioServer::isManual() const
{
    return mImpl->engineReady && mImpl->manual;
}

std::uint32_t AudioServer::getSampleRate() const
{
    return mImpl->engineReady ? ma_engine_get_sample_rate(&mImpl->engine) : 0;
}

std::uint32_t AudioServer::getChannels() const
{
    return mImpl->engineReady ? ma_engine_get_channels(&mImpl->engine) : 0;
}

void AudioServer::update()
{
    if (!mImpl->engineReady) return;

    finishLoads(mImpl);
    ct::Vector<VoiceKey> released;
    for (std::size_t i = 0; i < mImpl->voices.size(); ++i)
    {
        AudioVoice* voice = mImpl->voices.items()[i];
        refreshState(voice);
        if (voice->autoRelease && voice->state == VoiceState::Stopped)
            released.push_back(mImpl->voices.handle_at(i));
    }
    for (std::size_t i = 0; i < released.size(); ++i) destroyVoiceByKey(mImpl, released[i]);
}

std::uint32_t AudioServer::mix(float* output, std::uint32_t frames)
{
    if (!output || frames == 0 || !isManual()) return 0;
    ma_uint64 read = 0;
    if (ma_engine_read_pcm_frames(&mImpl->engine, output, frames, &read) != MA_SUCCESS && read == 0)
        return 0;
    return static_cast<std::uint32_t>(read);
}

SoundId AudioServer::createSoundFromFile(const ct::String& path)
{
    if (!mImpl->engineReady) return SoundId();
    const ct::Vector<std::uint8_t> bytes = FileSystem::getSingleton().readBinary(path);
    if (bytes.empty())
    {
        log_error("audio: could not read %s", path.c_str());
        return SoundId();
    }
    const SoundId id = createSoundFromMemory(bytes.data(), bytes.size());
    if (!id.valid())
    {
        log_error("audio: %s is not a supported sound", path.c_str());
        return id;
    }
    findSound(mImpl, id)->info.path = path;
    return id;
}

SoundId AudioServer::createSoundFromMemory(const void* data, std::size_t size)
{
    if (!mImpl->engineReady || !data || size == 0) return SoundId();

    SoundInfo info;
    if (!probeSound(data, size, info)) return SoundId();

    AudioSound* sound = new AudioSound();
    const std::uint8_t* bytes = static_cast<const std::uint8_t*>(data);
    sound->bytes.assign(bytes, bytes + size);
    sound->info = info;
    return soundId(mImpl->sounds.insert(sound));
}

SoundId AudioServer::requestSoundFromFile(const ct::String& path)
{
    if (!mImpl->engineReady || path.empty()) return SoundId();

    AudioSound* sound = new AudioSound();
    sound->info.path = path;
    sound->info.loading = true;
    const SoundKey key = mImpl->sounds.insert(sound);

    if (!mImpl->loader) mImpl->loader = new ct::ThreadPool(1);
    AudioServerImpl* impl = mImpl;
    mImpl->loader->submit(
            [impl, key, path]()
            {
                LoadedSound* result = new LoadedSound();
                result->key = key;
                result->bytes = FileSystem::getSingleton().readBinary(path);
                result->ok = !result->bytes.empty() &&
                             probeSound(result->bytes.data(), result->bytes.size(), result->info);
                ct::LockGuard guard(impl->loadedMutex);
                impl->loaded.push_back(result);
            });
    return soundId(key);
}

void AudioServer::destroySound(SoundId sound)
{
    if (!findSound(mImpl, sound)) return;

    ct::Vector<VoiceKey> doomed;
    for (std::size_t i = 0; i < mImpl->voices.size(); ++i)
    {
        if (mImpl->voices.items()[i]->sound == sound) doomed.push_back(mImpl->voices.handle_at(i));
    }
    for (std::size_t i = 0; i < doomed.size(); ++i) destroyVoiceByKey(mImpl, doomed[i]);

    AudioSound* entry = *mImpl->sounds.get(soundKey(sound));
    mImpl->sounds.erase(soundKey(sound));
    delete entry;
}

bool AudioServer::validSound(SoundId sound) const
{
    return findSound(mImpl, sound) != nullptr;
}

SoundId AudioServer::findSoundByPath(const ct::String& path) const
{
    if (path.empty()) return SoundId();
    for (std::size_t i = 0; i < mImpl->sounds.size(); ++i)
        if (mImpl->sounds.items()[i]->info.path == path) return soundId(mImpl->sounds.handle_at(i));
    return SoundId();
}

const SoundInfo* AudioServer::getSoundInfo(SoundId sound) const
{
    const AudioSound* entry = findSound(mImpl, sound);
    return entry ? &entry->info : nullptr;
}

std::size_t AudioServer::getSoundCount() const
{
    return mImpl->sounds.size();
}

VoiceId AudioServer::createVoice(SoundId sound, AudioBus bus)
{
    AudioSound* source = findSound(mImpl, sound);
    if (!source) return VoiceId();

    if (!source->info.loading && source->bytes.empty()) return VoiceId();

    AudioVoice* voice = new AudioVoice();
    voice->sound = sound;
    voice->bus = static_cast<AudioBus>(busIndex(bus));

    if (!source->info.loading && !initPlayer(mImpl, voice, source))
    {
        releaseVoice(voice);
        return VoiceId();
    }
    return voiceId(mImpl->voices.insert(voice));
}

void AudioServer::destroyVoice(VoiceId voice)
{
    if (!findVoice(mImpl, voice)) return;
    destroyVoiceByKey(mImpl, voiceKey(voice));
}

bool AudioServer::validVoice(VoiceId voice) const
{
    return findVoice(mImpl, voice) != nullptr;
}

std::size_t AudioServer::getVoiceCount() const
{
    return mImpl->voices.size();
}

VoiceId AudioServer::playOneShot(SoundId sound, float volume, float pitch, float pan, AudioBus bus)
{
    const VoiceId voice = createVoice(sound, bus);
    if (!voice.valid()) return voice;
    findVoice(mImpl, voice)->autoRelease = true;
    setVolume(voice, volume);
    setPitch(voice, pitch);
    setPan(voice, pan);
    if (!play(voice))
    {
        destroyVoice(voice);
        return VoiceId();
    }
    return voice;
}

VoiceId AudioServer::playOneShotAt(SoundId sound, const Math::Vec3& position, float volume,
        float pitch, float minDistance, float maxDistance, float rolloff, AudioBus bus)
{
    const VoiceId voice = createVoice(sound, bus);
    if (!voice.valid()) return voice;
    findVoice(mImpl, voice)->autoRelease = true;
    setVolume(voice, volume);
    setPitch(voice, pitch);
    setPosition(voice, position);
    setDistances(voice, minDistance, maxDistance);
    setRolloff(voice, rolloff);
    setSpatial(voice, true);
    if (!play(voice))
    {
        destroyVoice(voice);
        return VoiceId();
    }
    return voice;
}

bool AudioServer::play(VoiceId voice)
{
    AudioVoice* found = findVoice(mImpl, voice);
    if (!found) return false;
    refreshState(found);
    if (found->state == VoiceState::Playing) return true;
    if (!found->hasPlayer)
    {
        found->pendingPlay = true;
        found->state = VoiceState::Playing;
        return true;
    }
    ma_sound_set_stop_time_in_pcm_frames(&found->player, kNoStopTime);
    if (ma_sound_start(&found->player) != MA_SUCCESS) return false;
    found->state = VoiceState::Playing;
    return true;
}

bool AudioServer::stop(VoiceId voice)
{
    AudioVoice* found = findVoice(mImpl, voice);
    if (!found) return false;
    found->pendingPlay = false;
    if (found->hasPlayer)
    {
        ma_sound_stop(&found->player);
        ma_sound_seek_to_pcm_frame(&found->player, 0);
    }
    found->state = VoiceState::Stopped;
    return true;
}

bool AudioServer::pause(VoiceId voice)
{
    AudioVoice* found = findVoice(mImpl, voice);
    if (!found) return false;
    refreshState(found);
    if (found->state != VoiceState::Playing) return false;
    found->pendingPlay = false;
    if (found->hasPlayer && ma_sound_stop(&found->player) != MA_SUCCESS) return false;
    found->state = VoiceState::Paused;
    return true;
}

bool AudioServer::resume(VoiceId voice)
{
    AudioVoice* found = findVoice(mImpl, voice);
    if (!found || found->state != VoiceState::Paused) return false;
    return play(voice);
}

VoiceState AudioServer::getState(VoiceId voice) const
{
    AudioVoice* found = findVoice(mImpl, voice);
    if (!found) return VoiceState::Stopped;
    refreshState(found);
    return found->state;
}

bool AudioServer::isPlaying(VoiceId voice) const
{
    return getState(voice) == VoiceState::Playing;
}

float AudioServer::getPlaybackPosition(VoiceId voice) const
{
    AudioVoice* found = findVoice(mImpl, voice);
    if (!found || !found->hasPlayer) return 0.0f;
    float seconds = 0.0f;
    if (ma_sound_get_cursor_in_seconds(&found->player, &seconds) != MA_SUCCESS) return 0.0f;
    return seconds;
}

bool AudioServer::seek(VoiceId voice, float seconds)
{
    AudioVoice* found = findVoice(mImpl, voice);
    if (!found || !found->hasPlayer) return false;
    ma_uint32 rate = 0;
    if (ma_sound_get_data_format(&found->player, nullptr, nullptr, &rate, nullptr, 0) != MA_SUCCESS ||
            rate == 0)
        return false;
    const double frame = static_cast<double>(seconds > 0.0f ? seconds : 0.0f) * rate;
    return ma_sound_seek_to_pcm_frame(&found->player, static_cast<ma_uint64>(frame)) == MA_SUCCESS;
}

bool AudioServer::fadeIn(VoiceId voice, float seconds)
{
    AudioVoice* found = findVoice(mImpl, voice);
    if (!found) return false;
    if (found->hasPlayer)
        ma_sound_set_fade_in_milliseconds(&found->player, 0.0f, 1.0f, milliseconds(seconds));
    return true;
}

bool AudioServer::fadeOut(VoiceId voice, float seconds, bool releaseWhenDone)
{
    AudioVoice* found = findVoice(mImpl, voice);
    if (!found) return false;
    const ma_uint64 duration = milliseconds(seconds);
    if (!found->hasPlayer)
    {
        if (releaseWhenDone)
        {
            found->pendingPlay = false;
            found->state = VoiceState::Stopped;
            found->autoRelease = true;
        }
        return true;
    }
    if (releaseWhenDone)
    {
        if (ma_sound_stop_with_fade_in_milliseconds(&found->player, duration) != MA_SUCCESS)
            return false;
        found->autoRelease = true;
    }
    else
    {
        ma_sound_set_fade_in_milliseconds(&found->player, -1.0f, 0.0f, duration);
    }
    return true;
}

SoundId AudioServer::getVoiceSound(VoiceId voice) const
{
    const AudioVoice* found = findVoice(mImpl, voice);
    return found ? found->sound : SoundId();
}

bool AudioServer::setLooping(VoiceId voice, bool looping)
{
    AudioVoice* found = findVoice(mImpl, voice);
    if (!found) return false;
    found->looping = looping;
    if (found->hasPlayer) ma_sound_set_looping(&found->player, looping ? MA_TRUE : MA_FALSE);
    return true;
}

bool AudioServer::isLooping(VoiceId voice) const
{
    const AudioVoice* found = findVoice(mImpl, voice);
    return found && found->looping;
}

bool AudioServer::setVolume(VoiceId voice, float volume)
{
    AudioVoice* found = findVoice(mImpl, voice);
    if (!found) return false;
    found->volume = clampValue(volume, 0.0f, kAudioMaxVolume);
    if (found->hasPlayer)
    {
        ma_sound_set_volume(&found->player, found->volume);
        ma_sound_set_fade_in_milliseconds(&found->player, 1.0f, 1.0f, 0);
    }
    return true;
}

float AudioServer::getVolume(VoiceId voice) const
{
    const AudioVoice* found = findVoice(mImpl, voice);
    return found ? found->volume : 0.0f;
}

bool AudioServer::setPitch(VoiceId voice, float pitch)
{
    AudioVoice* found = findVoice(mImpl, voice);
    if (!found) return false;
    found->pitch = clampValue(pitch, kAudioMinPitch, kAudioMaxPitch);
    if (found->hasPlayer) ma_sound_set_pitch(&found->player, found->pitch);
    return true;
}

float AudioServer::getPitch(VoiceId voice) const
{
    const AudioVoice* found = findVoice(mImpl, voice);
    return found ? found->pitch : 0.0f;
}

bool AudioServer::setPan(VoiceId voice, float pan)
{
    AudioVoice* found = findVoice(mImpl, voice);
    if (!found) return false;
    found->pan = clampValue(pan, -1.0f, 1.0f);
    if (found->hasPlayer) ma_sound_set_pan(&found->player, found->pan);
    return true;
}

float AudioServer::getPan(VoiceId voice) const
{
    const AudioVoice* found = findVoice(mImpl, voice);
    return found ? found->pan : 0.0f;
}

bool AudioServer::setBus(VoiceId voice, AudioBus bus)
{
    AudioVoice* found = findVoice(mImpl, voice);
    if (!found) return false;
    found->bus = static_cast<AudioBus>(busIndex(bus));
    if (found->hasPlayer)
        ma_node_attach_output_bus(&found->player, 0, &mImpl->groups[busIndex(bus)], 0);
    return true;
}

AudioBus AudioServer::getBus(VoiceId voice) const
{
    const AudioVoice* found = findVoice(mImpl, voice);
    return found ? found->bus : AudioBus::Sfx;
}

bool AudioServer::setSpatial(VoiceId voice, bool spatial)
{
    AudioVoice* found = findVoice(mImpl, voice);
    if (!found) return false;
    found->spatial = spatial;
    if (found->hasPlayer)
        ma_sound_set_spatialization_enabled(&found->player, spatial ? MA_TRUE : MA_FALSE);
    return true;
}

bool AudioServer::isSpatial(VoiceId voice) const
{
    const AudioVoice* found = findVoice(mImpl, voice);
    return found && found->spatial;
}

bool AudioServer::setPosition(VoiceId voice, const Math::Vec3& position)
{
    AudioVoice* found = findVoice(mImpl, voice);
    if (!found) return false;
    found->position = position;
    if (found->hasPlayer) ma_sound_set_position(&found->player, position.x, position.y, position.z);
    return true;
}

Math::Vec3 AudioServer::getPosition(VoiceId voice) const
{
    const AudioVoice* found = findVoice(mImpl, voice);
    return found ? found->position : Math::Vec3(0.0f, 0.0f, 0.0f);
}

bool AudioServer::setVelocity(VoiceId voice, const Math::Vec3& velocity)
{
    AudioVoice* found = findVoice(mImpl, voice);
    if (!found) return false;
    found->velocity = velocity;
    if (found->hasPlayer) ma_sound_set_velocity(&found->player, velocity.x, velocity.y, velocity.z);
    return true;
}

Math::Vec3 AudioServer::getVelocity(VoiceId voice) const
{
    const AudioVoice* found = findVoice(mImpl, voice);
    return found ? found->velocity : Math::Vec3(0.0f, 0.0f, 0.0f);
}

bool AudioServer::setDistances(VoiceId voice, float minDistance, float maxDistance)
{
    AudioVoice* found = findVoice(mImpl, voice);
    if (!found) return false;
    found->minDistance = clampValue(minDistance, 0.0f, kAudioMaxDistance);
    found->maxDistance = clampValue(maxDistance, found->minDistance, kAudioMaxDistance);
    if (found->hasPlayer)
    {
        ma_sound_set_min_distance(&found->player, found->minDistance);
        ma_sound_set_max_distance(&found->player, found->maxDistance);
    }
    return true;
}

float AudioServer::getMinDistance(VoiceId voice) const
{
    const AudioVoice* found = findVoice(mImpl, voice);
    return found ? found->minDistance : 0.0f;
}

float AudioServer::getMaxDistance(VoiceId voice) const
{
    const AudioVoice* found = findVoice(mImpl, voice);
    return found ? found->maxDistance : 0.0f;
}

bool AudioServer::setRolloff(VoiceId voice, float rolloff)
{
    AudioVoice* found = findVoice(mImpl, voice);
    if (!found) return false;
    found->rolloff = clampValue(rolloff, 0.0f, kAudioMaxRolloff);
    if (found->hasPlayer) ma_sound_set_rolloff(&found->player, found->rolloff);
    return true;
}

float AudioServer::getRolloff(VoiceId voice) const
{
    const AudioVoice* found = findVoice(mImpl, voice);
    return found ? found->rolloff : 0.0f;
}

bool AudioServer::setAttenuation(VoiceId voice, AudioAttenuation attenuation)
{
    AudioVoice* found = findVoice(mImpl, voice);
    if (!found) return false;
    found->attenuation = attenuation;
    if (found->hasPlayer) ma_sound_set_attenuation_model(&found->player, toModel(attenuation));
    return true;
}

AudioAttenuation AudioServer::getAttenuation(VoiceId voice) const
{
    const AudioVoice* found = findVoice(mImpl, voice);
    return found ? found->attenuation : AudioAttenuation::Inverse;
}

void AudioServer::setListenerTransform(const Math::Mat4& transform)
{
    mImpl->listener = transform;
    applyListener(mImpl);
}

Math::Mat4 AudioServer::getListenerTransform() const
{
    return mImpl->listener;
}

void AudioServer::setListenerVelocity(const Math::Vec3& velocity)
{
    mImpl->listenerVelocity = velocity;
    applyListener(mImpl);
}

Math::Vec3 AudioServer::getListenerVelocity() const
{
    return mImpl->listenerVelocity;
}

void AudioServer::setMasterVolume(float volume)
{
    mImpl->masterVolume = clampValue(volume, 0.0f, kAudioMaxVolume);
    applyMixer(mImpl);
}

float AudioServer::getMasterVolume() const
{
    return mImpl->masterVolume;
}

void AudioServer::setMasterMuted(bool muted)
{
    mImpl->masterMuted = muted;
    applyMixer(mImpl);
}

bool AudioServer::isMasterMuted() const
{
    return mImpl->masterMuted;
}

void AudioServer::setBusVolume(AudioBus bus, float volume)
{
    mImpl->busVolume[busIndex(bus)] = clampValue(volume, 0.0f, kAudioMaxVolume);
    applyMixer(mImpl);
}

float AudioServer::getBusVolume(AudioBus bus) const
{
    return mImpl->busVolume[busIndex(bus)];
}

void AudioServer::setBusMuted(AudioBus bus, bool muted)
{
    mImpl->busMuted[busIndex(bus)] = muted;
    applyMixer(mImpl);
}

bool AudioServer::isBusMuted(AudioBus bus) const
{
    return mImpl->busMuted[busIndex(bus)];
}

void AudioServer::setBusPitch(AudioBus bus, float pitch)
{
    mImpl->busPitch[busIndex(bus)] = clampValue(pitch, kAudioMinPitch, kAudioMaxPitch);
    applyMixer(mImpl);
}

float AudioServer::getBusPitch(AudioBus bus) const
{
    return mImpl->busPitch[busIndex(bus)];
}

float AudioServer::getEffectiveVolume(VoiceId voice) const
{
    const AudioVoice* found = findVoice(mImpl, voice);
    if (!found) return 0.0f;
    if (mImpl->masterMuted) return 0.0f;
    const std::size_t bus = busIndex(found->bus);
    if (mImpl->busMuted[bus]) return 0.0f;
    return found->volume * mImpl->busVolume[bus] * mImpl->masterVolume;
}

} // namespace prisma3d
