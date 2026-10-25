# Closes both ends of the street outside the garage with a wall in the texture of the wall across
# it (ExternalWall24, 10 m by 20 m a tile), so the night starts in a dead-end alley. The walls are
# meshes of the level, so physics=concave gives them collision.
# Usage: python3 tools/close_street.py level.glb out.glb
import json, struct, sys

source, target = sys.argv[1], sys.argv[2]
BOTTOM, TOP = -4.7, 15.5
NEAR, FAR = 77.5, 93.5
# x of the wall and the direction it faces
ENDS = [('garage__street_end_w', -55.1, 1.0), ('garage__street_end_e', 14.9, -1.0)]

data = open(source, 'rb').read()
length = struct.unpack_from('<I', data, 12)[0]
doc = json.loads(data[20:20 + length])
offset = 20 + length
bin_length = struct.unpack_from('<I', data, offset)[0]
binary = bytearray(data[offset + 8:offset + 8 + bin_length])

if any(node.get('name', '').startswith('garage__street_end') for node in doc['nodes']):
    raise SystemExit('the street is already closed')
wall = next(m for m in doc['materials'] if m['name'].startswith('ExternalWall24'))
doc['materials'].append({'name': 'street_end', 'pbrMetallicRoughness': dict(wall['pbrMetallicRoughness']), 'doubleSided': False})
material = len(doc['materials']) - 1


def add(blob, kind, count, lo=None, hi=None, target_kind=None):
    while len(binary) % 4:
        binary.append(0)
    view = {'buffer': 0, 'byteOffset': len(binary), 'byteLength': len(blob)}
    if target_kind:
        view['target'] = target_kind
    doc['bufferViews'].append(view)
    binary.extend(blob)
    accessor = {'bufferView': len(doc['bufferViews']) - 1, 'componentType': 5126 if kind != 'SCALAR' else 5123, 'count': count, 'type': kind}
    if lo is not None:
        accessor['min'], accessor['max'] = lo, hi
    doc['accessors'].append(accessor)
    return len(doc['accessors']) - 1


root = doc['nodes'][doc['scenes'][0]['nodes'][0]]
width = (FAR - NEAR) / 10.0
for name, x, facing in ENDS:
    # Seen from the street, left to right along z, with v = 0 at the top as glTF has it.
    corners = [(x, BOTTOM, NEAR), (x, BOTTOM, FAR), (x, TOP, FAR), (x, TOP, NEAR)]
    uvs = [(0.0, 1.0), (width, 1.0), (width, 0.0), (0.0, 0.0)]
    if facing > 0:
        corners = [(x, BOTTOM, FAR), (x, BOTTOM, NEAR), (x, TOP, NEAR), (x, TOP, FAR)]
    positions = struct.pack('<12f', *[c for p in corners for c in p])
    normals = struct.pack('<12f', *[facing, 0.0, 0.0] * 4)
    texcoords = struct.pack('<8f', *[c for t in uvs for c in t])
    indices = struct.pack('<6H', 0, 1, 2, 0, 2, 3)
    lo = [min(p[i] for p in corners) for i in range(3)]
    hi = [max(p[i] for p in corners) for i in range(3)]
    primitive = {'attributes': {'POSITION': add(positions, 'VEC3', 4, lo, hi, 34962), 'NORMAL': add(normals, 'VEC3', 4, None, None, 34962),
                                'TEXCOORD_0': add(texcoords, 'VEC2', 4, None, None, 34962)},
                 'indices': add(indices, 'SCALAR', 6, None, None, 34963), 'material': material}
    doc['meshes'].append({'name': name, 'primitives': [primitive]})
    doc['nodes'].append({'name': name, 'mesh': len(doc['meshes']) - 1})
    root.setdefault('children', []).append(len(doc['nodes']) - 1)
    print('added', name, 'at x', x)

while len(binary) % 4:
    binary.append(0)
doc['buffers'][0]['byteLength'] = len(binary)
text = json.dumps(doc, separators=(',', ':')).encode('utf-8')
text += b' ' * ((4 - len(text) % 4) % 4)
with open(target, 'wb') as handle:
    handle.write(struct.pack('<III', 0x46546C67, 2, 12 + 8 + len(text) + 8 + len(binary)))
    handle.write(struct.pack('<II', len(text), 0x4E4F534A) + text)
    handle.write(struct.pack('<II', len(binary), 0x004E4942) + bytes(binary))
print('wrote', target)
