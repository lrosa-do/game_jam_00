# Drops the duplicated triangles of double-sided materials: some props (the standing fan's grille)
# were exported with each face twice, front and back on the same three corners. A double-sided
# material already draws both sides, so the copy only fights the original for the same pixels and
# flickers (z-fighting). A triangle is a copy when another one already has the same three corners
# (to the millimetre).
# Usage: python3 tools/dedupe_faces.py level.glb out.glb
import json, struct, sys

source, target = sys.argv[1], sys.argv[2]
data = open(source, 'rb').read()
length = struct.unpack_from('<I', data, 12)[0]
doc = json.loads(data[20:20 + length])
offset = 20 + length
bin_length = struct.unpack_from('<I', data, offset)[0]
binary = bytearray(data[offset + 8:offset + 8 + bin_length])
FORMATS = {5126: ('f', 4), 5125: ('I', 4), 5123: ('H', 2), 5121: ('B', 1)}


def read(index, width):
    acc = doc['accessors'][index]
    view = doc['bufferViews'][acc['bufferView']]
    code, size = FORMATS[acc['componentType']]
    stride = view.get('byteStride', size * width)
    start = view.get('byteOffset', 0) + acc.get('byteOffset', 0)
    return [struct.unpack_from('<%d%s' % (width, code), binary, start + k * stride) for k in range(acc['count'])]


removed = 0
for mesh in doc['meshes']:
    for prim in mesh['primitives']:
        material = doc['materials'][prim['material']] if 'material' in prim else {}
        if not material.get('doubleSided') or 'indices' not in prim or prim.get('mode', 4) != 4:
            continue
        positions = read(prim['attributes']['POSITION'], 3)
        flat = [row[0] for row in read(prim['indices'], 1)]
        seen = set()
        kept = []
        for i in range(0, len(flat) - 2, 3):
            tri = flat[i:i + 3]
            key = tuple(sorted(tuple(round(c, 3) for c in positions[v]) for v in tri))
            if key in seen:
                continue
            seen.add(key)
            kept.extend(tri)
        if len(kept) == len(flat):
            continue
        removed += (len(flat) - len(kept)) // 3
        # New index data (32-bit) at the end of the binary chunk.
        while len(binary) % 4:
            binary.append(0)
        blob = struct.pack('<%dI' % len(kept), *kept)
        doc['bufferViews'].append({'buffer': 0, 'byteOffset': len(binary), 'byteLength': len(blob), 'target': 34963})
        binary.extend(blob)
        doc['accessors'].append({'bufferView': len(doc['bufferViews']) - 1, 'componentType': 5125, 'count': len(kept), 'type': 'SCALAR'})
        prim['indices'] = len(doc['accessors']) - 1
        print('%-30s %-34s -%d triangles' % (mesh.get('name', '')[:30], material.get('name', '')[:34], (len(flat) - len(kept)) // 3))
print('removed', removed, 'duplicated triangles')

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
