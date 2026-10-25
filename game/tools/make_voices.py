# Speaks the lines of Assets/Sounds/voice/lines.csv (id;speaker;event;text) with Kokoro TTS and a
# filter per speaker, into Assets/Sounds/voice/<speaker>/<id>.ogg. The CSV is also the subtitles.
#   python make_voices.py lines.csv out_dir            all lines
#   python make_voices.py lines.csv out_dir --test     a few lines per speaker in every candidate
#                                                      voice, to pick one by ear
#   python make_voices.py lines.csv out_dir --only=in_ only the lines whose id starts so
#   python make_voices.py lines.csv out_dir --speaker=NULLOS   only that speaker
# Run with the python of a venv that has kokoro-onnx, misaki[en] and soundfile; MODEL_DIR holds
# kokoro-v1.0.onnx and voices-v1.0.bin.
import csv, os, sys
import numpy as np
import soundfile as sf
from kokoro_onnx import Kokoro
from misaki import en

MODEL_DIR = os.environ.get('MODEL_DIR', '/home/djoker/Transferências/burnout/usrdir/pveh/jam_tools/tts')

# Voice, speed and filter per speaker; CANDIDATES are tried with --test.
VOICES = {
    'ANDROID': ('af_nova', 1.0, 'light'),
    'NULLOS': ('am_eric', 1.0, 'deep'),
    'UNIT7': ('am_puck', 1.08, 'chip'),
    'BRAVURA': ('am_michael', 0.95, 'none'),
    'UNIT7_RADIO': ('am_puck', 1.0, 'radio'),
}
CANDIDATES = {
    'ANDROID': ['af_heart', 'af_nova', 'af_bella', 'bf_emma'],
    'NULLOS': ['am_onyx', 'am_fenrir', 'bm_george', 'bm_lewis', 'bm_daniel', 'am_eric'],
    'UNIT7': ['af_sky', 'af_nicole', 'am_puck', 'af_river'],
    'BRAVURA': ['am_adam', 'am_fenrir', 'am_michael', 'bm_fable'],
}


def ring(a, sr, hz, depth):
    t = np.arange(len(a)) / sr
    return a * (1.0 - depth + depth * np.sin(2.0 * np.pi * hz * t))


def echo(a, sr, seconds, gain):
    d = int(seconds * sr)
    out = a.copy()
    out[d:] += gain * a[:-d]
    return out


def crush(a, levels):
    return np.round(a * levels) / levels


# Lower pitch (and pace) by resampling: factor 0.88 is about two semitones down.
def lower(a, factor):
    return np.interp(np.arange(0, len(a), factor), np.arange(len(a)), a)


def treat(a, sr, kind):
    if kind == 'light':  # a machine, still easy to understand
        a = ring(a, sr, 45.0, 0.18)
        a = echo(a, sr, 0.012, 0.25)
    elif kind == 'heavy':  # the system itself
        a = ring(a, sr, 60.0, 0.45)
        a = crush(a, 40)
        a = echo(a, sr, 0.018, 0.35)
        a = echo(a, sr, 0.09, 0.2)
    elif kind == 'deep':  # the system itself, strong and clean: lower, a little saturated, a hall
        a = lower(a, 0.88)
        a = np.tanh(a / np.abs(a).max() * 1.6)
        a = ring(a, sr, 30.0, 0.08)
        a = echo(a, sr, 0.022, 0.18)
        a = echo(a, sr, 0.12, 0.12)
    elif kind == 'radio':  # a broken transmission: a narrow band, crackle, drop-outs
        spec = np.fft.rfft(a)
        freq = np.fft.rfftfreq(len(a), 1.0 / sr)
        spec[(freq < 350.0) | (freq > 2800.0)] = 0.0
        a = np.fft.irfft(spec, len(a))
        a = crush(a / (np.abs(a).max() + 1e-9), 24)
        rng = np.random.default_rng(7)
        a = a + rng.normal(0.0, 0.04, len(a)) * (rng.random(len(a)) < 0.3)
        gate = np.ones(len(a))
        for start in rng.integers(0, len(a), 6):
            gate[start:start + int(0.05 * sr)] = 0.15
        a = a * np.convolve(gate, np.ones(200) / 200.0, 'same')
    elif kind == 'chip':  # a small robot
        a = ring(a, sr, 110.0, 0.2)
        a = crush(a, 64)
    peak = np.abs(a).max()
    return a / peak * 0.9 if peak > 0 else a


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    test = '--test' in sys.argv
    csv_path, out = args[0], args[1]
    k = Kokoro(os.path.join(MODEL_DIR, 'kokoro-v1.0.onnx'), os.path.join(MODEL_DIR, 'voices-v1.0.bin'))
    g2p = {False: en.G2P(trf=False, british=False, fallback=None), True: en.G2P(trf=False, british=True, fallback=None)}
    rows = list(csv.DictReader(open(csv_path, encoding='utf-8'), delimiter=';'))
    speaker = next((a[len('--speaker='):] for a in sys.argv[1:] if a.startswith('--speaker=')), '')
    only = next((a[len('--only='):] for a in sys.argv[1:] if a.startswith('--only=')), '')
    if only:
        rows = [r for r in rows if r['id'].startswith(only)]
    if speaker:
        rows = [r for r in rows if r['speaker'] == speaker]
    if test:
        picked = {}
        for r in rows:
            picked.setdefault(r['speaker'], [])
            if len(picked[r['speaker']]) < 3:
                picked[r['speaker']].append(r)
        jobs = [(r, v) for s, rs in picked.items() for r in rs for v in CANDIDATES[s]]
    else:
        jobs = [(r, VOICES[r['speaker']][0]) for r in rows]
    for r, voice in jobs:
        _, speed, kind = VOICES[r['speaker']]
        phonemes, _ = g2p[voice[0] == 'b'](r['text'])
        audio, sr = k.create(phonemes, voice=voice, speed=speed, is_phonemes=True)
        audio = treat(audio, sr, kind)
        folder = os.path.join(out, r['speaker'].lower())
        os.makedirs(folder, exist_ok=True)
        name = (r['id'] + '_' + voice) if test else r['id']
        sf.write(os.path.join(folder, name + '.ogg'), audio, sr, format='OGG', subtype='VORBIS')
        print('%-16s %-10s %.1fs  %s' % (name, voice, len(audio) / sr, r['text']))


if __name__ == '__main__':
    main()
