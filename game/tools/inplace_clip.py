# Removes the horizontal travel of the root bone from every animation of a GLB clip, so it plays in
# place; the vertical motion (jumps, crouches) and the sway around the straight line are kept.
# Usage: python3 tools/inplace_clip.py input.glb output.glb [--bone=Hips]
import json, struct, sys

args = [a for a in sys.argv[1:] if not a.startswith('--')]
bone = next((a[len('--bone='):] for a in sys.argv[1:] if a.startswith('--bone=')), 'Hips')
source, target = args[0], args[1]

data = bytearray(open(source, 'rb').read())
length = struct.unpack_from('<I', data, 12)[0]
document = json.loads(data[20:20 + length])
binary = 20 + length + 8
moved = 0
for animation in document.get('animations', []):
    for channel in animation['channels']:
        node = document['nodes'][channel['target']['node']]
        if channel['target']['path'] != 'translation' or not node.get('name', '').endswith(bone):
            continue
        accessor = document['accessors'][animation['samplers'][channel['sampler']]['output']]
        view = document['bufferViews'][accessor['bufferView']]
        offset = binary + view.get('byteOffset', 0) + accessor.get('byteOffset', 0)
        stride = view.get('byteStride', 12)
        count = accessor['count']
        keys = [list(struct.unpack_from('<3f', data, offset + i * stride)) for i in range(count)]
        dx = keys[-1][0] - keys[0][0]
        dz = keys[-1][2] - keys[0][2]
        for i, key in enumerate(keys):
            t = i / (count - 1) if count > 1 else 0.0
            key[0] -= dx * t
            key[2] -= dz * t
            struct.pack_into('<3f', data, offset + i * stride, *key)
        accessor.pop('min', None)
        accessor.pop('max', None)
        moved += 1
        print('%s: removed %.2f m of travel' % (animation.get('name', ''), (dx * dx + dz * dz) ** 0.5))
if not moved:
    raise SystemExit('no translation track on a bone ending in ' + bone)
text = json.dumps(document, separators=(',', ':')).encode('utf-8')
text += b' ' * ((4 - len(text) % 4) % 4)
rest = data[20 + length:]
out = struct.pack('<III', 0x46546C67, 2, 12 + 8 + len(text) + len(rest))
out += struct.pack('<II', len(text), 0x4E4F534A) + text + rest
open(target, 'wb').write(out)
