# Turns the light halos (the projector's beam in the briefing room) into see-through glows: in the
# original game they are added on top of the picture, but the export made them opaque, so the
# beam showed as a dark solid pyramid. The halo texture (dim grey on black) becomes a pale light
# colour with its brightness as alpha, the material is blended, double-sided and emissive (it
# glows by itself, the lightmap does not darken it).
# Usage: python3 tools/halo_blend.py level.glb out.glb
import io, json, struct, sys
from PIL import Image

source, target = sys.argv[1], sys.argv[2]
data = open(source, 'rb').read()
length = struct.unpack_from('<I', data, 12)[0]
doc = json.loads(data[20:20 + length])
offset = 20 + length
bin_length = struct.unpack_from('<I', data, offset)[0]
binary = bytearray(data[offset + 8:offset + 8 + bin_length])

done = {}
for material in doc.get('materials', []):
    if not material.get('name', '').lower().startswith('halo'):
        continue
    pbr = material.setdefault('pbrMetallicRoughness', {})
    texture = pbr.get('baseColorTexture')
    if not texture:
        continue
    source_index = doc['textures'][texture['index']]['source']
    if source_index not in done:
        image = doc['images'][source_index]
        view = doc['bufferViews'][image['bufferView']]
        start = view.get('byteOffset', 0)
        picture = Image.open(io.BytesIO(binary[start:start + view['byteLength']])).convert('RGB')
        glow = Image.new('RGBA', picture.size)
        top = max(max(p) for p in picture.getdata()) or 1
        glow.putdata([(235, 240, 255, int(255 * 0.55 * max(p) / top)) for p in picture.getdata()])
        out = io.BytesIO()
        glow.save(out, format='PNG')
        while len(binary) % 4:
            binary.append(0)
        doc['bufferViews'].append({'buffer': 0, 'byteOffset': len(binary), 'byteLength': len(out.getvalue())})
        binary.extend(out.getvalue())
        image['bufferView'] = len(doc['bufferViews']) - 1
        image['mimeType'] = 'image/png'
        done[source_index] = True
    material['alphaMode'] = 'BLEND'
    material['doubleSided'] = True
    pbr['baseColorFactor'] = [0.05, 0.05, 0.05, 1.0]
    pbr['metallicFactor'] = 0.0
    pbr['roughnessFactor'] = 1.0
    material['emissiveTexture'] = {'index': texture['index']}
    material['emissiveFactor'] = [0.9, 0.9, 1.0]
    print('halo', material['name'])

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
