# Lifts the overlays laid on the floors (the NYPD logo in the reception, rugs, blood and puke
# stains, drain grates) a few millimetres: they sit in the very plane of the tiles under them, so
# the two flicker over each other (z-fighting). They are primitives inside the rooms' meshes, so
# their vertices are moved, not their nodes.
# Usage: python3 tools/lift_decals.py level.glb out.glb [--lift=0.008]
import json, re, struct, sys
import numpy as np

DECALS = r'^(NYPD_floor-logo|rug_|blood_|puke|Grate12)'
args = [a for a in sys.argv[1:] if not a.startswith('--')]
lift = next((float(a[len('--lift='):]) for a in sys.argv[1:] if a.startswith('--lift=')), 0.008)
source, target = args[0], args[1]
data = open(source, 'rb').read()
length = struct.unpack_from('<I', data, 12)[0]
doc = json.loads(data[20:20 + length])
offset = 20 + length
bin_length = struct.unpack_from('<I', data, offset)[0]
binary = bytearray(data[offset + 8:offset + 8 + bin_length])


def local_matrix(node):
    if 'matrix' in node:
        return np.array(node['matrix']).reshape(4, 4).T
    t = node.get('translation', [0, 0, 0])
    x, y, z, w = node.get('rotation', [0, 0, 0, 1])
    s = node.get('scale', [1, 1, 1])
    r = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    m = np.eye(4)
    m[:3, :3] = r * np.array(s)
    m[:3, 3] = t
    return m


done = set()
lifted = {}


def lift_accessor(index, world):
    acc = doc['accessors'][index]
    view = doc['bufferViews'][acc['bufferView']]
    stride = view.get('byteStride', 12)
    start = view.get('byteOffset', 0) + acc.get('byteOffset', 0)
    # The world lift (0, lift, 0) in the mesh's own space.
    up = np.linalg.inv(world[:3, :3]) @ np.array([0.0, lift, 0.0])
    for k in range(acc['count']):
        at = start + k * stride
        x, y, z = struct.unpack_from('<3f', binary, at)
        struct.pack_into('<3f', binary, at, x + up[0], y + up[1], z + up[2])
    acc['min'] = [acc['min'][i] + up[i] for i in range(3)]
    acc['max'] = [acc['max'][i] + up[i] for i in range(3)]


def walk(index, parent):
    node = doc['nodes'][index]
    world = parent @ local_matrix(node)
    if 'mesh' in node:
        for prim in doc['meshes'][node['mesh']]['primitives']:
            if 'material' not in prim:
                continue
            name = doc['materials'][prim['material']].get('name', '')
            position = prim['attributes']['POSITION']
            if re.search(DECALS, name) and position not in done:
                done.add(position)
                lift_accessor(position, world)
                lifted[name] = lifted.get(name, 0) + 1
    for child in node.get('children', []):
        walk(child, world)


for root in doc['scenes'][0]['nodes']:
    walk(root, np.eye(4))
print('lifted', lifted)

text = json.dumps(doc, separators=(',', ':')).encode('utf-8')
text += b' ' * ((4 - len(text) % 4) % 4)
with open(target, 'wb') as handle:
    handle.write(struct.pack('<III', 0x46546C67, 2, 12 + 8 + len(text) + 8 + len(binary)))
    handle.write(struct.pack('<II', len(text), 0x4E4F534A) + text)
    handle.write(struct.pack('<II', len(binary), 0x004E4942) + bytes(binary))
print('wrote', target)
