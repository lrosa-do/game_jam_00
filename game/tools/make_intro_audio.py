# Mixes the whole intro into one track: the narration lines in order, spaced, over the night
# ambience (rain and distant thunder outside, the precinct hum under it) that ducks under the
# voices; static before Unit-7's broken signal and a cut after it. Prints the cue of each line
# (seconds from the start) for INTRO in Assets/Scripts/game.py, which types each line at its cue.
# Usage: python make_intro_audio.py AMBIENCE_DIR out.ogg folder/id ...
# (run with the python of the TTS venv: numpy and soundfile)
import os, subprocess, sys
import numpy as np
import soundfile as sf

SR = 44100
LEAD = 2.0       # ambience alone before the first line
GAP = 0.75       # silence between lines
TAIL = 4.5       # after the last line: the title, then the fade
VOICE_DIR = 'Assets/Sounds/voice'


def load(path, stereo):
    a, sr = sf.read(path, always_2d=True)
    if sr != SR:
        n = int(len(a) * SR / sr)
        a = np.stack([np.interp(np.linspace(0, len(a) - 1, n), np.arange(len(a)), a[:, c]) for c in range(a.shape[1])], 1)
    if stereo and a.shape[1] == 1:
        a = np.repeat(a, 2, 1)
    if not stereo:
        a = a.mean(1, keepdims=True)
    return a


def static(seconds, rng):
    n = int(seconds * SR)
    noise = rng.normal(0.0, 1.0, n)
    spec = np.fft.rfft(noise)
    freq = np.fft.rfftfreq(n, 1.0 / SR)
    spec[(freq < 600.0) | (freq > 5000.0)] = 0.0
    noise = np.fft.irfft(spec, n)
    noise = noise / np.abs(noise).max()
    crackle = (rng.random(n) < 0.002) * rng.normal(0.0, 1.0, n) * 3.0
    env = np.minimum(1.0, np.minimum(np.arange(n), np.arange(n)[::-1]) / (0.05 * SR))
    return (noise * 0.6 + crackle) * env


def main():
    ambience_dir, out = sys.argv[1], sys.argv[2]
    ids = sys.argv[3:]
    rng = np.random.default_rng(12)
    voices = [load(os.path.join(VOICE_DIR, i + '.ogg'), False)[:, 0] for i in ids]
    cues = []
    t = LEAD
    for i, v in zip(ids, voices):
        if i.startswith('unit7'):
            t += 0.5  # room for the static to come in
        cues.append(t)
        t += len(v) / SR + GAP
    total = t - GAP + TAIL
    n = int(total * SR)
    mix = np.zeros((n, 2))
    talk = np.zeros(n)  # 1 where someone speaks, for the ducking
    for i, v, cue in zip(ids, voices, cues):
        s = int(cue * SR)
        v = v / (np.abs(v).max() + 1e-9) * 0.85
        mix[s:s + len(v)] += v[:, None]
        talk[s:s + len(v)] = 1.0
        if i.startswith('unit7'):
            burst = static(0.7, rng) * 0.35
            b = s - int(0.5 * SR)
            mix[b:b + len(burst)] += burst[:, None]
            hiss = static(len(v) / SR, rng) * 0.08
            mix[s:s + len(hiss)] += hiss[:, None]
            cut = static(0.18, rng) * 0.6
            e = s + len(v)
            mix[e:e + len(cut)] += cut[:, None]
    # The ambience: the city in the rain, the precinct under it; it ducks while a line plays.
    rain = load(os.path.join(ambience_dir, 'City_Rainy_distant_thunder.ogg'), True)
    hum = load(os.path.join(ambience_dir, 'police_station.ogg'), True)
    bed = np.zeros((n, 2))
    bed += np.resize(rain, (n, 2)) * 0.5
    bed += np.resize(hum, (n, 2)) * 0.35
    bed = bed / (np.abs(bed).max() + 1e-9)
    smooth = np.convolve(talk, np.ones(int(0.4 * SR)) / (0.4 * SR), 'same')
    duck = 0.32 - 0.2 * smooth
    fade_in = np.minimum(1.0, np.arange(n) / (LEAD * SR))
    fade_out = np.minimum(1.0, (n - np.arange(n)) / (2.0 * SR))
    mix += bed * (duck * fade_in * fade_out)[:, None]
    mix = mix / np.abs(mix).max() * 0.92
    # libsndfile crashes writing a long Vorbis file: a WAV, then ffmpeg makes the ogg.
    wav = out + '.wav'
    sf.write(wav, mix.astype(np.float32), SR, subtype='FLOAT')
    subprocess.run(['ffmpeg', '-y', '-loglevel', 'error', '-i', wav, '-c:a', 'libvorbis', '-q:a', '5', out], check=True)
    os.remove(wav)
    print('wrote %s, %.1f s' % (out, total))
    for i, cue in zip(ids, cues):
        print('%-20s %.2f' % (i, cue))


if __name__ == '__main__':
    main()
