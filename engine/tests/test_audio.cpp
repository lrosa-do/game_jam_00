#include "Check.h"

#include "prisma3d/audio/AudioServer.h"
#include "prisma3d/core/FileSystem.h"
#include "prisma3d/scene/AudioListener3D.h"
#include "prisma3d/scene/AudioStreamPlayer.h"
#include "prisma3d/scene/AudioStreamPlayer3D.h"
#include "prisma3d/scene/SceneTree.h"

#include "platform.h"

#include <ct/thread.hpp>
#include <ct/vector.hpp>

#include <math.h>
#include <string.h>

using namespace prisma3d;

namespace
{

const float kPi = 3.14159265358979323846f;
const double kStep = 0.1;
const std::uint32_t kWavRate = 22050;
const std::uint32_t kOutputRate = 48000;
const std::uint32_t kOutputChannels = 2;

bool near(float a, float b, float tolerance = 1e-4f)
{
    return fabsf(a - b) <= tolerance;
}

bool nearVec3(const Math::Vec3& a, float x, float y, float z)
{
    return near(a.x, x) && near(a.y, y) && near(a.z, z);
}

void put16(ct::Vector<std::uint8_t>& out, std::uint32_t value)
{
    out.push_back(static_cast<std::uint8_t>(value & 0xffu));
    out.push_back(static_cast<std::uint8_t>((value >> 8) & 0xffu));
}

void put32(ct::Vector<std::uint8_t>& out, std::uint32_t value)
{
    put16(out, value & 0xffffu);
    put16(out, value >> 16);
}

void putTag(ct::Vector<std::uint8_t>& out, const char* tag)
{
    for (int i = 0; i < 4; ++i) out.push_back(static_cast<std::uint8_t>(tag[i]));
}

ct::Vector<std::uint8_t> makeWav(float frequency, float seconds, float amplitude = 0.5f)
{
    const std::uint32_t frames = static_cast<std::uint32_t>(seconds * static_cast<float>(kWavRate));
    ct::Vector<std::uint8_t> bytes;
    putTag(bytes, "RIFF");
    put32(bytes, 36 + frames * 2);
    putTag(bytes, "WAVE");
    putTag(bytes, "fmt ");
    put32(bytes, 16);
    put16(bytes, 1);
    put16(bytes, 1);
    put32(bytes, kWavRate);
    put32(bytes, kWavRate * 2);
    put16(bytes, 2);
    put16(bytes, 16);
    putTag(bytes, "data");
    put32(bytes, frames * 2);
    for (std::uint32_t i = 0; i < frames; ++i)
    {
        const float t = static_cast<float>(i) / static_cast<float>(kWavRate);
        const float sample = amplitude * sinf(2.0f * kPi * frequency * t);
        put16(bytes,
                static_cast<std::uint32_t>(static_cast<std::int32_t>(sample * 32767.0f)) & 0xffffu);
    }
    return bytes;
}

SoundId makeSound(AudioServer& server, float seconds = 0.25f, float frequency = 440.0f)
{
    const ct::Vector<std::uint8_t> wav = makeWav(frequency, seconds);
    return server.createSoundFromMemory(wav.data(), wav.size());
}

AudioServerDesc manualDesc()
{
    AudioServerDesc desc;
    desc.manual = true;
    return desc;
}

AudioServerDesc nullDesc()
{
    AudioServerDesc desc;
    desc.nullBackend = true;
    return desc;
}

ct::Vector<float> mixFrames(AudioServer& server, std::uint32_t frames)
{
    ct::Vector<float> out;
    out.resize(static_cast<std::size_t>(frames) * kOutputChannels);
    const std::uint32_t read = server.mix(out.data(), frames);
    CHECK(read == frames);
    return out;
}

float rms(const ct::Vector<float>& buffer, std::uint32_t channel)
{
    double sum = 0.0;
    std::size_t count = 0;
    for (std::size_t i = channel; i < buffer.size(); i += kOutputChannels)
    {
        sum += static_cast<double>(buffer[i]) * static_cast<double>(buffer[i]);
        ++count;
    }
    return count ? static_cast<float>(sqrt(sum / static_cast<double>(count))) : 0.0f;
}

float peak(const ct::Vector<float>& buffer)
{
    float result = 0.0f;
    for (std::size_t i = 0; i < buffer.size(); ++i)
    {
        const float value = fabsf(buffer[i]);
        if (value > result) result = value;
    }
    return result;
}

float level(AudioServer& server, VoiceId voice)
{
    server.play(voice);
    mixFrames(server, 1500);
    const ct::Vector<float> block = mixFrames(server, 2400);
    server.stop(voice);
    mixFrames(server, 600);
    return (rms(block, 0) + rms(block, 1)) * 0.5f;
}

void testLifecycle()
{
    {
        AudioServer server;
        CHECK(AudioServer::singleton() == &server);
        CHECK(!server.isReady());
        CHECK(!server.createSoundFromMemory("abc", 3).valid());
        CHECK(!makeSound(server).valid());
        CHECK(!server.createVoice(SoundId()).valid());
        CHECK(server.getSoundCount() == 0);
        CHECK(server.getVoiceCount() == 0);
        CHECK(server.mix(nullptr, 0) == 0);

        for (int round = 0; round < 2; ++round)
        {
            CHECK(server.init(nullDesc()));
            CHECK(server.isReady());
            CHECK(!server.isManual());
            CHECK(server.getSampleRate() > 0);
            CHECK(server.getChannels() > 0);
            CHECK(server.init(nullDesc()));
            const SoundId sound = makeSound(server);
            CHECK(sound.valid());
            const VoiceId voice = server.createVoice(sound);
            CHECK(voice.valid());
            CHECK(server.play(voice));
            server.shutdown();
            CHECK(!server.isReady());
            CHECK(!server.validSound(sound));
            CHECK(!server.validVoice(voice));
            CHECK(server.getSoundCount() == 0);
            CHECK(server.getVoiceCount() == 0);
            CHECK(!server.play(voice));
        }
        server.shutdown();

        CHECK(server.init(manualDesc()));
        CHECK(server.isManual());
        CHECK(server.getSampleRate() == kOutputRate);
        CHECK(server.getChannels() == kOutputChannels);
        server.shutdown();

        AudioServerDesc custom = manualDesc();
        custom.sampleRate = 44100;
        custom.channels = 1;
        CHECK(server.init(custom));
        CHECK(server.getSampleRate() == 44100);
        CHECK(server.getChannels() == 1);
        float mono[64];
        CHECK(server.mix(mono, 64) == 64);
    }
    CHECK(AudioServer::singleton() == nullptr);
}

void testSounds()
{
    AudioServer server;
    CHECK(server.init(manualDesc()));

    const ct::Vector<std::uint8_t> wav = makeWav(440.0f, 0.25f);
    const SoundId sound = server.createSoundFromMemory(wav.data(), wav.size());
    CHECK(sound.valid());
    CHECK(server.validSound(sound));
    CHECK(server.getSoundCount() == 1);
    const SoundInfo* info = server.getSoundInfo(sound);
    CHECK(info != nullptr);
    if (info)
    {
        CHECK(info->sampleRate == kWavRate);
        CHECK(info->channels == 1);
        CHECK(info->frames == static_cast<std::uint64_t>(0.25f * static_cast<float>(kWavRate)));
        CHECK(near(static_cast<float>(info->duration), 0.25f, 0.001f));
        CHECK(info->byteSize == wav.size());
    }

    CHECK(!server.createSoundFromMemory(nullptr, 100).valid());
    CHECK(!server.createSoundFromMemory(wav.data(), 0).valid());
    const char* garbage = "this is not an audio file, only text";
    CHECK(!server.createSoundFromMemory(garbage, strlen(garbage)).valid());
    CHECK(!server.createSoundFromMemory(wav.data(), 20).valid());
    ct::Vector<std::uint8_t> zeros;
    zeros.resize(256);
    CHECK(!server.createSoundFromMemory(zeros.data(), zeros.size()).valid());
    CHECK(server.getSoundCount() == 1);

    CHECK(!server.createSoundFromFile("audio_missing_file.wav").valid());
    const ct::String directory = ct::String(PRISMA3D_TEST_DIR) + "/audio";
    CHECK(fs_create_directory(directory.c_str()));
    const ct::String path = directory + "/tone.wav";
    CHECK(file_write(path.c_str(), wav.data(), wav.size()));
    const SoundId fromFile = server.createSoundFromFile(path);
    CHECK(fromFile.valid());
    const SoundInfo* fileInfo = server.getSoundInfo(fromFile);
    CHECK(fileInfo != nullptr && fileInfo->path == path && fileInfo->frames == 5512);
    CHECK(server.getSoundCount() == 2);

    const VoiceId a = server.createVoice(sound);
    const VoiceId b = server.createVoice(sound);
    const VoiceId c = server.createVoice(fromFile);
    CHECK(a.valid() && b.valid() && c.valid() && a != b);
    CHECK(server.getVoiceCount() == 3);
    CHECK(server.getVoiceSound(a) == sound);
    CHECK(server.getVoiceSound(c) == fromFile);

    server.destroySound(sound);
    CHECK(!server.validSound(sound));
    CHECK(server.getSoundInfo(sound) == nullptr);
    CHECK(!server.validVoice(a));
    CHECK(!server.validVoice(b));
    CHECK(server.validVoice(c));
    CHECK(server.getVoiceCount() == 1);
    CHECK(!server.createVoice(sound).valid());
    server.destroySound(sound);

    const SoundId reused = makeSound(server);
    CHECK(reused.valid());
    CHECK(reused != sound);
    CHECK(!server.validSound(sound));
    CHECK(server.validSound(reused));

    server.destroyVoice(c);
    server.destroyVoice(c);
    CHECK(!server.validVoice(c));
    CHECK(!server.validVoice(VoiceId()));
    CHECK(!server.validSound(SoundId()));
    CHECK(!server.play(c));
    CHECK(!server.setVolume(c, 1.0f));
    CHECK(server.getState(c) == VoiceState::Stopped);
    CHECK(server.getVolume(c) == 0.0f);
}

void testVoiceStates()
{
    AudioServer server;
    CHECK(server.init(manualDesc()));
    const SoundId sound = makeSound(server, 1.0f);
    const VoiceId voice = server.createVoice(sound);
    CHECK(voice.valid());

    CHECK(server.getState(voice) == VoiceState::Stopped);
    CHECK(!server.isPlaying(voice));
    CHECK(!server.pause(voice));
    CHECK(!server.resume(voice));
    CHECK(server.stop(voice));

    CHECK(server.play(voice));
    CHECK(server.getState(voice) == VoiceState::Playing);
    CHECK(server.isPlaying(voice));
    CHECK(server.play(voice));
    CHECK(server.getState(voice) == VoiceState::Playing);

    CHECK(server.pause(voice));
    CHECK(server.getState(voice) == VoiceState::Paused);
    CHECK(!server.isPlaying(voice));
    CHECK(!server.pause(voice));

    CHECK(server.resume(voice));
    CHECK(server.getState(voice) == VoiceState::Playing);
    CHECK(!server.resume(voice));

    CHECK(server.pause(voice));
    CHECK(server.play(voice));
    CHECK(server.getState(voice) == VoiceState::Playing);

    CHECK(server.stop(voice));
    CHECK(server.getState(voice) == VoiceState::Stopped);
    CHECK(!server.pause(voice));
    CHECK(server.play(voice));
    CHECK(server.isPlaying(voice));

    CHECK(!server.isLooping(voice));
    CHECK(server.setLooping(voice, true));
    CHECK(server.isLooping(voice));
    CHECK(server.setLooping(voice, false));
    CHECK(!server.isLooping(voice));

    CHECK(server.getVolume(voice) == 1.0f);
    CHECK(server.setVolume(voice, 0.25f));
    CHECK(server.getVolume(voice) == 0.25f);
    server.setVolume(voice, 100.0f);
    CHECK(server.getVolume(voice) == kAudioMaxVolume);
    server.setVolume(voice, -1.0f);
    CHECK(server.getVolume(voice) == 0.0f);
    server.setPitch(voice, 1.5f);
    CHECK(server.getPitch(voice) == 1.5f);
    server.setPitch(voice, 0.0f);
    CHECK(server.getPitch(voice) == kAudioMinPitch);
    server.setPitch(voice, 99.0f);
    CHECK(server.getPitch(voice) == kAudioMaxPitch);
    server.setPan(voice, 0.5f);
    CHECK(server.getPan(voice) == 0.5f);
    server.setPan(voice, -4.0f);
    CHECK(server.getPan(voice) == -1.0f);

    server.destroyVoice(voice);
    CHECK(!server.isPlaying(voice));
    CHECK(!server.stop(voice));
}

void testSpatialParameters()
{
    AudioServer server;
    CHECK(server.init(manualDesc()));
    const SoundId sound = makeSound(server);
    const VoiceId voice = server.createVoice(sound);

    CHECK(!server.isSpatial(voice));
    CHECK(server.getBus(voice) == AudioBus::Sfx);
    CHECK(near(server.getMinDistance(voice), 1.0f));
    CHECK(near(server.getMaxDistance(voice), 100.0f));
    CHECK(near(server.getRolloff(voice), 1.0f));
    CHECK(server.getAttenuation(voice) == AudioAttenuation::Inverse);

    CHECK(server.setSpatial(voice, true));
    CHECK(server.isSpatial(voice));
    CHECK(server.setPosition(voice, Math::Vec3(1.0f, 2.0f, 3.0f)));
    CHECK(nearVec3(server.getPosition(voice), 1.0f, 2.0f, 3.0f));
    CHECK(server.setVelocity(voice, Math::Vec3(-1.0f, 0.5f, 4.0f)));
    CHECK(nearVec3(server.getVelocity(voice), -1.0f, 0.5f, 4.0f));
    CHECK(server.setDistances(voice, 2.0f, 30.0f));
    CHECK(near(server.getMinDistance(voice), 2.0f));
    CHECK(near(server.getMaxDistance(voice), 30.0f));
    CHECK(server.setDistances(voice, 8.0f, 3.0f));
    CHECK(near(server.getMinDistance(voice), 8.0f));
    CHECK(near(server.getMaxDistance(voice), 8.0f));
    server.setDistances(voice, -5.0f, 2000000.0f);
    CHECK(near(server.getMinDistance(voice), 0.0f));
    CHECK(near(server.getMaxDistance(voice), kAudioMaxDistance));
    server.setRolloff(voice, 2.5f);
    CHECK(near(server.getRolloff(voice), 2.5f));
    server.setRolloff(voice, 1000.0f);
    CHECK(near(server.getRolloff(voice), kAudioMaxRolloff));
    server.setAttenuation(voice, AudioAttenuation::Linear);
    CHECK(server.getAttenuation(voice) == AudioAttenuation::Linear);
    server.setAttenuation(voice, AudioAttenuation::Exponential);
    CHECK(server.getAttenuation(voice) == AudioAttenuation::Exponential);
    server.setAttenuation(voice, AudioAttenuation::None);
    CHECK(server.getAttenuation(voice) == AudioAttenuation::None);
    CHECK(server.setSpatial(voice, false));
    CHECK(!server.isSpatial(voice));

    CHECK(server.setBus(voice, AudioBus::Music));
    CHECK(server.getBus(voice) == AudioBus::Music);
    const VoiceId music = server.createVoice(sound, AudioBus::Music);
    CHECK(server.getBus(music) == AudioBus::Music);

    CHECK(server.getListenerTransform() == Math::Mat4::Identity());
    const Math::Mat4 moved = Math::Mat4::Translation(Math::Vec3(5.0f, 6.0f, 7.0f));
    server.setListenerTransform(moved);
    CHECK(server.getListenerTransform() == moved);
    server.setListenerVelocity(Math::Vec3(1.0f, 2.0f, 3.0f));
    CHECK(nearVec3(server.getListenerVelocity(), 1.0f, 2.0f, 3.0f));

    const VoiceId shot = server.playOneShotAt(sound, Math::Vec3(4.0f, 0.0f, 0.0f), 0.5f, 2.0f, 3.0f,
            40.0f, 1.5f, AudioBus::Music);
    CHECK(shot.valid());
    CHECK(server.isSpatial(shot));
    CHECK(server.isPlaying(shot));
    CHECK(near(server.getVolume(shot), 0.5f));
    CHECK(near(server.getPitch(shot), 2.0f));
    CHECK(nearVec3(server.getPosition(shot), 4.0f, 0.0f, 0.0f));
    CHECK(near(server.getMinDistance(shot), 3.0f));
    CHECK(near(server.getMaxDistance(shot), 40.0f));
    CHECK(near(server.getRolloff(shot), 1.5f));
    CHECK(server.getBus(shot) == AudioBus::Music);
}

void testVolumes()
{
    AudioServer server;
    CHECK(server.init(manualDesc()));
    const SoundId sound = makeSound(server);
    const VoiceId voice = server.createVoice(sound);

    CHECK(near(server.getMasterVolume(), 1.0f));
    CHECK(near(server.getBusVolume(AudioBus::Sfx), 1.0f));
    CHECK(near(server.getBusVolume(AudioBus::Music), 1.0f));
    CHECK(!server.isMasterMuted());
    CHECK(!server.isBusMuted(AudioBus::Music));
    CHECK(near(server.getEffectiveVolume(voice), 1.0f));

    server.setVolume(voice, 0.5f);
    server.setBusVolume(AudioBus::Sfx, 0.5f);
    server.setMasterVolume(0.5f);
    CHECK(near(server.getMasterVolume(), 0.5f));
    CHECK(near(server.getBusVolume(AudioBus::Sfx), 0.5f));
    CHECK(near(server.getBusVolume(AudioBus::Music), 1.0f));
    CHECK(near(server.getEffectiveVolume(voice), 0.125f));

    server.setBus(voice, AudioBus::Music);
    CHECK(near(server.getEffectiveVolume(voice), 0.25f));
    server.setBusMuted(AudioBus::Music, true);
    CHECK(server.isBusMuted(AudioBus::Music));
    CHECK(server.getEffectiveVolume(voice) == 0.0f);
    server.setBusMuted(AudioBus::Music, false);
    server.setMasterMuted(true);
    CHECK(server.isMasterMuted());
    CHECK(server.getEffectiveVolume(voice) == 0.0f);
    server.setMasterMuted(false);
    CHECK(near(server.getEffectiveVolume(voice), 0.25f));

    server.setMasterVolume(9.0f);
    CHECK(server.getMasterVolume() == kAudioMaxVolume);
    server.setBusVolume(AudioBus::Sfx, -1.0f);
    CHECK(server.getBusVolume(AudioBus::Sfx) == 0.0f);
    CHECK(server.getEffectiveVolume(VoiceId()) == 0.0f);
}

void testFinishing()
{
    AudioServer server;
    CHECK(server.init(manualDesc()));
    const SoundId sound = makeSound(server, 0.25f);

    const VoiceId held = server.createVoice(sound);
    CHECK(server.play(held));
    mixFrames(server, 4800);
    server.update();
    CHECK(server.getState(held) == VoiceState::Playing);
    CHECK(server.isPlaying(held));

    mixFrames(server, 14400);
    server.update();
    CHECK(server.getState(held) == VoiceState::Stopped);
    CHECK(!server.isPlaying(held));
    CHECK(server.validVoice(held));
    CHECK(peak(mixFrames(server, 4800)) == 0.0f);

    CHECK(server.play(held));
    CHECK(server.isPlaying(held));
    const ct::Vector<float> replay = mixFrames(server, 4800);
    CHECK(peak(replay) > 0.1f);
    server.stop(held);

    const VoiceId looped = server.createVoice(sound);
    server.setLooping(looped, true);
    CHECK(server.play(looped));
    mixFrames(server, 48000);
    server.update();
    CHECK(server.isPlaying(looped));
    server.destroyVoice(looped);

    CHECK(server.getVoiceCount() == 1);
    const VoiceId shot = server.playOneShot(sound);
    CHECK(shot.valid());
    CHECK(server.getVoiceCount() == 2);
    mixFrames(server, 4800);
    server.update();
    CHECK(server.validVoice(shot));
    mixFrames(server, 14400);
    server.update();
    CHECK(!server.validVoice(shot));
    CHECK(server.getVoiceCount() == 1);

    const SoundId longSound = makeSound(server, 2.0f);
    const VoiceId fading = server.createVoice(longSound);
    CHECK(server.play(fading));
    mixFrames(server, 2400);
    CHECK(server.fadeOut(fading, 0.05f, true));
    mixFrames(server, 4800);
    server.update();
    CHECK(!server.validVoice(fading));
    CHECK(peak(mixFrames(server, 2400)) == 0.0f);

    const VoiceId softened = server.createVoice(longSound);
    server.setLooping(softened, true);
    CHECK(server.fadeIn(softened, 0.5f));
    CHECK(server.play(softened));
    const ct::Vector<float> start = mixFrames(server, 480);
    mixFrames(server, 36000);
    const ct::Vector<float> later = mixFrames(server, 480);
    CHECK(rms(start, 0) < rms(later, 0) * 0.5f);
    CHECK(rms(later, 0) > 0.05f);
}

void testRealTimeFinish()
{
    AudioServer server;
    CHECK(server.init(nullDesc()));
    const SoundId sound = makeSound(server, 0.1f);
    const VoiceId voice = server.createVoice(sound);
    CHECK(server.play(voice));
    CHECK(server.isPlaying(voice));

    bool finished = false;
    for (int waited = 0; waited < 3000 && !finished; waited += 10)
    {
        ct::Thread::sleep_ms(10);
        server.update();
        finished = !server.isPlaying(voice);
    }
    CHECK(finished);

    const VoiceId shot = server.playOneShot(sound);
    CHECK(shot.valid());
    bool released = false;
    for (int waited = 0; waited < 3000 && !released; waited += 10)
    {
        ct::Thread::sleep_ms(10);
        server.update();
        released = !server.validVoice(shot);
    }
    CHECK(released);
}

void testMixedOutput()
{
    AudioServer server;
    CHECK(server.init(manualDesc()));

    CHECK(peak(mixFrames(server, 2400)) == 0.0f);

    const SoundId sound = makeSound(server, 1.0f);
    const VoiceId voice = server.createVoice(sound);
    server.setLooping(voice, true);
    CHECK(peak(mixFrames(server, 2400)) == 0.0f);

    CHECK(server.play(voice));
    mixFrames(server, 1500);
    const ct::Vector<float> playing = mixFrames(server, 2400);
    CHECK(peak(playing) > 0.3f);
    CHECK(peak(playing) <= 0.6f);
    CHECK(rms(playing, 0) > 0.2f);
    CHECK(near(rms(playing, 0), rms(playing, 1), 0.001f));

    CHECK(server.pause(voice));
    mixFrames(server, 480);
    CHECK(peak(mixFrames(server, 2400)) == 0.0f);
    CHECK(server.resume(voice));
    mixFrames(server, 480);
    CHECK(peak(mixFrames(server, 2400)) > 0.3f);
    CHECK(server.stop(voice));
    mixFrames(server, 480);
    CHECK(peak(mixFrames(server, 2400)) == 0.0f);

    const float full = level(server, voice);
    CHECK(full > 0.2f);
    server.setVolume(voice, 0.5f);
    CHECK(near(level(server, voice), full * 0.5f, full * 0.02f));
    server.setVolume(voice, 1.0f);

    server.setMasterVolume(0.5f);
    CHECK(near(level(server, voice), full * 0.5f, full * 0.02f));
    server.setBusVolume(AudioBus::Sfx, 0.5f);
    CHECK(near(level(server, voice), full * 0.25f, full * 0.02f));
    server.setBusVolume(AudioBus::Sfx, 1.0f);
    server.setMasterVolume(1.0f);
    CHECK(near(level(server, voice), full, full * 0.02f));

    server.setBus(voice, AudioBus::Music);
    server.setBusVolume(AudioBus::Sfx, 0.0f);
    CHECK(near(level(server, voice), full, full * 0.02f));
    server.setBusVolume(AudioBus::Music, 0.0f);
    CHECK(level(server, voice) == 0.0f);
    server.setBusVolume(AudioBus::Music, 1.0f);
    server.setBusVolume(AudioBus::Sfx, 1.0f);
    server.setBus(voice, AudioBus::Sfx);

    server.setMasterMuted(true);
    CHECK(level(server, voice) == 0.0f);
    server.setMasterMuted(false);
    CHECK(near(level(server, voice), full, full * 0.02f));

    server.setPan(voice, -1.0f);
    CHECK(server.play(voice));
    mixFrames(server, 1500);
    const ct::Vector<float> panned = mixFrames(server, 2400);
    server.stop(voice);
    mixFrames(server, 600);
    CHECK(rms(panned, 0) > 0.2f);
    CHECK(rms(panned, 1) < 0.001f);
    server.setPan(voice, 0.0f);

    server.setPitch(voice, 2.0f);
    CHECK(near(level(server, voice), full, full * 0.05f));
    server.setPitch(voice, 1.0f);
}

// The cursor of a playing voice in seconds (for text or effects timed to a track), and a seek.
void testPlaybackPosition()
{
    AudioServer server;
    CHECK(server.init(manualDesc()));
    const SoundId sound = makeSound(server, 1.0f);
    const VoiceId voice = server.createVoice(sound);
    CHECK(server.getPlaybackPosition(voice) == 0.0f);
    CHECK(server.play(voice));
    mixFrames(server, 4800);
    const float at = server.getPlaybackPosition(voice);
    CHECK(at > 0.05f && at < 0.2f);
    mixFrames(server, 4800);
    CHECK(server.getPlaybackPosition(voice) > at);
    CHECK(server.seek(voice, 0.6f));
    CHECK(near(server.getPlaybackPosition(voice), 0.6f, 0.02f));
    mixFrames(server, 2400);
    CHECK(server.getPlaybackPosition(voice) > 0.6f);
    CHECK(!server.seek(VoiceId(), 0.1f));
}

void testDistanceAttenuation()
{
    AudioServer server;
    CHECK(server.init(manualDesc()));
    const SoundId sound = makeSound(server, 1.0f);
    const VoiceId voice = server.createVoice(sound);
    server.setLooping(voice, true);
    server.setSpatial(voice, true);
    server.setDistances(voice, 1.0f, 100.0f);
    server.setRolloff(voice, 1.0f);

    server.setPosition(voice, Math::Vec3(0.0f, 0.0f, -1.0f));
    const float near1 = level(server, voice);
    server.setPosition(voice, Math::Vec3(0.0f, 0.0f, -4.0f));
    const float near4 = level(server, voice);
    server.setPosition(voice, Math::Vec3(0.0f, 0.0f, -50.0f));
    const float far50 = level(server, voice);

    CHECK(near1 > 0.1f);
    CHECK(near4 < near1);
    CHECK(far50 < near4);
    CHECK(far50 > 0.0f);
    CHECK(near(near4, near1 * 0.25f, near1 * 0.03f));
    CHECK(near(far50, near1 / 50.0f, near1 * 0.01f));

    server.setPosition(voice, Math::Vec3(0.0f, 0.0f, -100.0f));
    const float edge = level(server, voice);
    server.setPosition(voice, Math::Vec3(0.0f, 0.0f, -500.0f));
    const float beyond = level(server, voice);
    CHECK(near(edge, beyond, edge * 0.03f));
    CHECK(near(edge, near1 / 100.0f, near1 * 0.01f));

    server.setAttenuation(voice, AudioAttenuation::None);
    server.setPosition(voice, Math::Vec3(0.0f, 0.0f, -50.0f));
    const float flat50 = level(server, voice);
    server.setPosition(voice, Math::Vec3(0.0f, 0.0f, -4.0f));
    const float flat4 = level(server, voice);
    CHECK(flat50 > far50 * 10.0f);
    CHECK(near(flat50, flat4, flat4 * 0.03f));
    server.setAttenuation(voice, AudioAttenuation::Inverse);

    server.setPosition(voice, Math::Vec3(0.0f, 0.0f, -4.0f));
    server.setSpatial(voice, false);
    CHECK(level(server, voice) > near1 * 0.9f);
    server.setSpatial(voice, true);

    server.setListenerTransform(Math::Mat4::Translation(Math::Vec3(0.0f, 0.0f, -4.0f)));
    server.setPosition(voice, Math::Vec3(0.0f, 0.0f, -5.0f));
    CHECK(near(level(server, voice), near1, near1 * 0.03f));
    server.setListenerTransform(Math::Mat4::Identity());

    server.setPosition(voice, Math::Vec3(10.0f, 0.0f, 0.0f));
    server.play(voice);
    mixFrames(server, 1500);
    const ct::Vector<float> rightSide = mixFrames(server, 2400);
    server.stop(voice);
    mixFrames(server, 600);
    CHECK(rms(rightSide, 1) > rms(rightSide, 0) * 2.0f);

    server.setListenerTransform(Math::Mat4::RotationY(kPi));
    server.play(voice);
    mixFrames(server, 1500);
    const ct::Vector<float> leftSide = mixFrames(server, 2400);
    server.stop(voice);
    mixFrames(server, 600);
    CHECK(rms(leftSide, 0) > rms(leftSide, 1) * 2.0f);
}

void testTeardown()
{
    AudioServer server;
    for (int round = 0; round < 2; ++round)
    {
        CHECK(server.init(manualDesc()));
        const SoundId sound = makeSound(server, 1.0f);
        for (int i = 0; i < 12; ++i)
        {
            const VoiceId voice =
                    server.createVoice(sound, i % 2 ? AudioBus::Music : AudioBus::Sfx);
            server.setLooping(voice, true);
            server.setSpatial(voice, i % 3 == 0);
            server.play(voice);
        }
        server.playOneShot(sound);
        mixFrames(server, 4800);
        CHECK(server.getVoiceCount() == 13);
        server.shutdown();
        CHECK(server.getVoiceCount() == 0);
    }
}

void testListenerNode()
{
    AudioServer server;
    CHECK(server.init(manualDesc()));
    SceneTree tree;
    tree.initialize();
    Node* root = tree.getRoot();

    CHECK(AudioListener3D::getCurrent() == nullptr);
    AudioListener3D* first = root->createChild<AudioListener3D>();
    CHECK(first->isClass("AudioListener3D"));
    CHECK(first->isClass(NodeClass::Node3D));
    CHECK(first->castTo<AudioListener3D>() == first);
    CHECK(first->isCurrent());
    CHECK(AudioListener3D::getCurrent() == first);

    first->setPosition(Math::Vec3(1.0f, 2.0f, 3.0f));
    tree.process(kStep);
    Math::Mat4 pushed = server.getListenerTransform();
    CHECK(near(pushed.col3.x, 1.0f) && near(pushed.col3.y, 2.0f) && near(pushed.col3.z, 3.0f));

    AudioListener3D* second = root->createChild<AudioListener3D>();
    CHECK(first->isCurrent());
    CHECK(!second->isCurrent());
    second->setPosition(Math::Vec3(-5.0f, 0.0f, 0.0f));
    tree.process(kStep);
    CHECK(near(server.getListenerTransform().col3.x, 1.0f));

    second->makeCurrent();
    CHECK(second->isCurrent());
    CHECK(!first->isCurrent());
    CHECK(AudioListener3D::getCurrent() == second);
    CHECK(near(server.getListenerTransform().col3.x, -5.0f));

    first->setPosition(Math::Vec3(9.0f, 0.0f, 0.0f));
    tree.process(kStep);
    CHECK(near(server.getListenerTransform().col3.x, -5.0f));
    second->setPosition(Math::Vec3(-6.0f, 0.0f, 0.0f));
    tree.process(kStep);
    CHECK(near(server.getListenerTransform().col3.x, -6.0f));

    second->setVelocity(Math::Vec3(0.0f, 0.0f, 2.0f));
    CHECK(nearVec3(server.getListenerVelocity(), 0.0f, 0.0f, 2.0f));
    CHECK(nearVec3(second->getVelocity(), 0.0f, 0.0f, 2.0f));

    second->queueFree();
    tree.process(kStep);
    CHECK(AudioListener3D::getCurrent() == first);
    CHECK(first->isCurrent());
    CHECK(near(server.getListenerTransform().col3.x, 9.0f));

    first->clearCurrent(false);
    CHECK(!first->isCurrent());
    CHECK(AudioListener3D::getCurrent() == nullptr);
    first->makeCurrent();
    CHECK(first->isCurrent());

    Node holder;
    AudioListener3D* pending = holder.createChild<AudioListener3D>();
    pending->makeCurrent();
    CHECK(pending->isCurrent());
    CHECK(AudioListener3D::getCurrent() == first);
    CHECK(holder.reparent(root));
    CHECK(AudioListener3D::getCurrent() == pending);
    CHECK(!first->isCurrent());

    first->queueFree();
    pending->queueFree();
    tree.process(kStep);
    CHECK(AudioListener3D::getCurrent() == nullptr);

    AudioListener3D* turned = root->createChild<AudioListener3D>();
    turned->setRotation(Math::Vec3(0.0f, kPi, 0.0f));
    tree.process(kStep);
    const SoundId sound = makeSound(server, 1.0f);
    AudioStreamPlayer3D* emitter = root->createChild<AudioStreamPlayer3D>();
    emitter->setSound(sound);
    emitter->setLooping(true);
    emitter->setPosition(Math::Vec3(10.0f, 0.0f, 0.0f));
    CHECK(emitter->play());
    mixFrames(server, 1500);
    const ct::Vector<float> block = mixFrames(server, 2400);
    CHECK(rms(block, 0) > rms(block, 1) * 2.0f);
    turned->setRotation(Math::Vec3(0.0f, 0.0f, 0.0f));
    tree.process(kStep);
    mixFrames(server, 1500);
    const ct::Vector<float> turnedBack = mixFrames(server, 2400);
    CHECK(rms(turnedBack, 1) > rms(turnedBack, 0) * 2.0f);
}

void testEmitterNode()
{
    AudioServer server;
    CHECK(server.init(manualDesc()));
    SceneTree tree;
    tree.initialize();
    Node* root = tree.getRoot();
    const SoundId sound = makeSound(server, 1.0f);

    Node3D* parent = root->createChild<Node3D>();
    parent->setPosition(Math::Vec3(10.0f, 0.0f, 0.0f));
    AudioStreamPlayer3D* player = parent->createChild<AudioStreamPlayer3D>();
    CHECK(player->isClass("AudioStreamPlayer3D"));
    CHECK(player->isClass(NodeClass::Node3D));
    CHECK(player->castTo<AudioStreamPlayer3D>() == player);
    CHECK(player->castTo<AudioListener3D>() == nullptr);

    CHECK(!player->play());
    CHECK(!player->getVoice().valid());
    CHECK(!player->isPlaying());

    player->setSound(sound);
    CHECK(player->getSound() == sound);
    player->setPosition(Math::Vec3(1.0f, 0.0f, 0.0f));
    player->setLooping(true);
    player->setVolume(0.5f);
    player->setPitch(1.25f);
    player->setUnitSize(3.0f);
    player->setMaxDistance(40.0f);
    player->setRolloff(2.0f);
    player->setAttenuation(AudioAttenuation::Linear);
    player->setBus(AudioBus::Music);
    player->setVelocity(Math::Vec3(0.0f, 1.0f, 0.0f));
    CHECK(player->isLooping());
    CHECK(near(player->getVolume(), 0.5f));
    CHECK(near(player->getPitch(), 1.25f));
    CHECK(near(player->getUnitSize(), 3.0f));
    CHECK(near(player->getMaxDistance(), 40.0f));
    CHECK(near(player->getRolloff(), 2.0f));
    CHECK(player->getAttenuation() == AudioAttenuation::Linear);
    CHECK(player->getBus() == AudioBus::Music);
    CHECK(server.getVoiceCount() == 0);

    CHECK(player->play());
    const VoiceId voice = player->getVoice();
    CHECK(voice.valid());
    CHECK(server.validVoice(voice));
    CHECK(server.getVoiceCount() == 1);
    CHECK(player->isPlaying());
    CHECK(server.isPlaying(voice));
    CHECK(server.getVoiceSound(voice) == sound);
    CHECK(server.isSpatial(voice));
    CHECK(server.isLooping(voice));
    CHECK(near(server.getVolume(voice), 0.5f));
    CHECK(near(server.getPitch(voice), 1.25f));
    CHECK(near(server.getMinDistance(voice), 3.0f));
    CHECK(near(server.getMaxDistance(voice), 40.0f));
    CHECK(near(server.getRolloff(voice), 2.0f));
    CHECK(server.getAttenuation(voice) == AudioAttenuation::Linear);
    CHECK(server.getBus(voice) == AudioBus::Music);
    CHECK(nearVec3(server.getVelocity(voice), 0.0f, 1.0f, 0.0f));
    CHECK(nearVec3(server.getPosition(voice), 11.0f, 0.0f, 0.0f));

    parent->setPosition(Math::Vec3(20.0f, 0.0f, 0.0f));
    tree.process(kStep);
    CHECK(nearVec3(server.getPosition(voice), 21.0f, 0.0f, 0.0f));
    player->setPosition(Math::Vec3(0.0f, 5.0f, 0.0f));
    tree.process(kStep);
    CHECK(nearVec3(server.getPosition(voice), 20.0f, 5.0f, 0.0f));

    player->setVolume(2.0f);
    player->setLooping(false);
    player->setUnitSize(60.0f);
    CHECK(near(server.getVolume(voice), 2.0f));
    CHECK(!server.isLooping(voice));
    CHECK(near(server.getMinDistance(voice), 60.0f));
    CHECK(near(server.getMaxDistance(voice), 60.0f));
    player->setLooping(true);

    CHECK(player->pause());
    CHECK(!player->isPlaying());
    CHECK(server.getState(voice) == VoiceState::Paused);
    CHECK(player->resume());
    CHECK(player->isPlaying());

    CHECK(player->play());
    CHECK(server.getVoiceCount() == 1);
    CHECK(player->getVoice().valid());
    CHECK(!server.validVoice(voice));
    const VoiceId second = player->getVoice();

    player->stop();
    CHECK(!player->getVoice().valid());
    CHECK(!player->isPlaying());
    CHECK(!server.validVoice(second));
    CHECK(server.getVoiceCount() == 0);

    CHECK(player->play());
    const VoiceId third = player->getVoice();
    player->setSound(SoundId());
    CHECK(!server.validVoice(third));
    CHECK(server.getVoiceCount() == 0);
    player->setSound(sound);

    CHECK(player->play());
    const VoiceId fourth = player->getVoice();
    CHECK(server.validVoice(fourth));
    player->queueFree();
    tree.process(kStep);
    CHECK(!server.validVoice(fourth));
    CHECK(server.getVoiceCount() == 0);

    AudioStreamPlayer3D* moved = root->createChild<AudioStreamPlayer3D>();
    moved->setSound(sound);
    CHECK(moved->play());
    const VoiceId movedVoice = moved->getVoice();
    CHECK(moved->reparent(parent));
    CHECK(!server.validVoice(movedVoice));
    CHECK(!moved->getVoice().valid());
    CHECK(server.getVoiceCount() == 0);
}

void testAutoplay()
{
    AudioServer server;
    CHECK(server.init(manualDesc()));
    SceneTree tree;
    tree.initialize();
    Node* root = tree.getRoot();
    const SoundId sound = makeSound(server, 1.0f);

    {
        Node holder;
        AudioStreamPlayer3D* positional = holder.createChild<AudioStreamPlayer3D>();
        positional->setSound(sound);
        positional->setAutoplay(true);
        positional->setPosition(Math::Vec3(2.0f, 0.0f, 0.0f));
        AudioStreamPlayer* flat = holder.createChild<AudioStreamPlayer>();
        flat->setSound(sound);
        flat->setAutoplay(true);
        flat->setBus(AudioBus::Music);
        AudioStreamPlayer3D* quiet = holder.createChild<AudioStreamPlayer3D>();
        quiet->setSound(sound);
        CHECK(positional->isAutoplay());
        CHECK(!quiet->isAutoplay());
        CHECK(!positional->getVoice().valid());
        CHECK(!flat->getVoice().valid());
        CHECK(server.getVoiceCount() == 0);

        CHECK(holder.reparent(root));
        CHECK(positional->getVoice().valid());
        CHECK(positional->isPlaying());
        CHECK(nearVec3(server.getPosition(positional->getVoice()), 2.0f, 0.0f, 0.0f));
        CHECK(flat->getVoice().valid());
        CHECK(flat->isPlaying());
        CHECK(server.getBus(flat->getVoice()) == AudioBus::Music);
        CHECK(!server.isSpatial(flat->getVoice()));
        CHECK(!quiet->getVoice().valid());
        CHECK(server.getVoiceCount() == 2);
        CHECK(peak(mixFrames(server, 2400)) > 0.1f);
    }
    CHECK(server.getVoiceCount() == 0);
}

void testPlayerNode()
{
    AudioServer server;
    CHECK(server.init(manualDesc()));
    SceneTree tree;
    tree.initialize();
    Node* root = tree.getRoot();
    const SoundId sound = makeSound(server, 1.0f);

    AudioStreamPlayer* player = root->createChild<AudioStreamPlayer>();
    CHECK(player->isClass("AudioStreamPlayer"));
    CHECK(player->castTo<AudioStreamPlayer>() == player);
    CHECK(!player->play());
    player->setSound(sound);
    player->setLooping(true);
    player->setVolume(0.5f);
    player->setPitch(0.75f);
    player->setPan(-0.5f);
    player->setBus(AudioBus::Music);
    CHECK(player->getSound() == sound);
    CHECK(player->isLooping());
    CHECK(near(player->getVolume(), 0.5f));
    CHECK(near(player->getPitch(), 0.75f));
    CHECK(near(player->getPan(), -0.5f));
    CHECK(player->getBus() == AudioBus::Music);

    CHECK(player->play());
    const VoiceId voice = player->getVoice();
    CHECK(server.validVoice(voice));
    CHECK(player->isPlaying());
    CHECK(!server.isSpatial(voice));
    CHECK(server.isLooping(voice));
    CHECK(near(server.getVolume(voice), 0.5f));
    CHECK(near(server.getPitch(voice), 0.75f));
    CHECK(near(server.getPan(voice), -0.5f));
    CHECK(server.getBus(voice) == AudioBus::Music);

    player->setVolume(0.25f);
    player->setPan(1.0f);
    player->setBus(AudioBus::Sfx);
    CHECK(near(server.getVolume(voice), 0.25f));
    CHECK(near(server.getPan(voice), 1.0f));
    CHECK(server.getBus(voice) == AudioBus::Sfx);

    player->setVolume(9.0f);
    CHECK(player->getVolume() == kAudioMaxVolume);

    CHECK(player->pause());
    CHECK(!player->isPlaying());
    CHECK(player->resume());
    CHECK(player->isPlaying());

    player->queueFree();
    tree.process(kStep);
    CHECK(!server.validVoice(voice));
    CHECK(server.getVoiceCount() == 0);
}

void testNodeTeardown()
{
    AudioServer server;
    CHECK(server.init(manualDesc()));
    const SoundId sound = makeSound(server, 1.0f);
    {
        SceneTree tree;
        tree.initialize();
        Node* root = tree.getRoot();
        for (int i = 0; i < 4; ++i)
        {
            AudioStreamPlayer3D* player = root->createChild<AudioStreamPlayer3D>();
            player->setSound(sound);
            player->setLooping(true);
            player->play();
            AudioStreamPlayer* flat = root->createChild<AudioStreamPlayer>();
            flat->setSound(sound);
            flat->play();
        }
        root->createChild<AudioListener3D>();
        mixFrames(server, 2400);
        CHECK(server.getVoiceCount() == 8);
    }
    CHECK(server.getVoiceCount() == 0);
    CHECK(AudioListener3D::getCurrent() == nullptr);

    SceneTree tree;
    tree.initialize();
    AudioStreamPlayer3D* player = tree.getRoot()->createChild<AudioStreamPlayer3D>();
    player->setSound(sound);
    CHECK(player->play());
    server.shutdown();
    CHECK(!player->isPlaying());
    player->stop();
    CHECK(!player->play());
}

void testNodeWithoutServer()
{
    SceneTree tree;
    tree.initialize();
    AudioStreamPlayer3D* player = tree.getRoot()->createChild<AudioStreamPlayer3D>();
    player->setSound(SoundId::from_bits(0x00100001u));
    player->setAutoplay(true);
    CHECK(!player->play());
    CHECK(!player->isPlaying());
    player->stop();
    AudioListener3D* listener = tree.getRoot()->createChild<AudioListener3D>();
    listener->setPosition(Math::Vec3(1.0f, 0.0f, 0.0f));
    listener->makeCurrent();
    tree.process(kStep);
    AudioStreamPlayer* flat = tree.getRoot()->createChild<AudioStreamPlayer>();
    CHECK(!flat->play());
}

} // namespace

int main()
{
    testLifecycle();
    testSounds();
    testVoiceStates();
    testSpatialParameters();
    testVolumes();
    testFinishing();
    testRealTimeFinish();
    testMixedOutput();
    testPlaybackPosition();
    testDistanceAttenuation();
    testTeardown();
    testListenerNode();
    testEmitterNode();
    testAutoplay();
    testPlayerNode();
    testNodeTeardown();
    testNodeWithoutServer();

    if (failures != 0)
    {
        printf("%d check(s) failed\n", failures);
        return 1;
    }
    printf("audio: ok\n");
    return 0;
}
