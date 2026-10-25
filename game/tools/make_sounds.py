# Synthesizes the placeholder weapon sounds (Python standard library only).
# Usage: python3 tools/make_sounds.py
import math, os, random, struct, wave

rate = 22050
out = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'Assets', 'Sounds')

def write(name, samples):
    path = os.path.join(out, name)
    with wave.open(path, 'wb') as w:
        w.setnchannels(1); w.setsampwidth(2); w.setframerate(rate)
        w.writeframes(b''.join(struct.pack('<h', int(max(-1.0, min(1.0, s)) * 32767)) for s in samples))
    print(path, len(samples) / rate, 's')

def pistol():
    random.seed(7)
    samples = []
    length = int(rate * 0.45)
    low = 0.0
    for i in range(length):
        t = i / rate
        crack = random.uniform(-1.0, 1.0) * math.exp(-t * 60.0)
        body = random.uniform(-1.0, 1.0) * math.exp(-t * 14.0) * 0.6
        low = low * 0.92 + random.uniform(-1.0, 1.0) * 0.08
        thump = low * math.exp(-t * 9.0) * 2.5 + math.sin(2 * math.pi * 110.0 * t) * math.exp(-t * 18.0) * 0.5
        samples.append(crack * 0.9 + body + thump)
    return samples

write('pistol.wav', pistol())
