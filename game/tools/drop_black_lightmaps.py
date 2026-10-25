# Drops the lightmap (occlusionTexture on the second UV set) of the materials whose lightmap is a
# tiny black image: objects the bake left out (the car in the street, a van, a mop) came out pitch
# black; without it they are lit like the dynamic objects, by the light probes.
# Usage: python3 tools/drop_black_lightmaps.py level.glb out.glb
import io, json, struct, sys
from PIL import Image

source, target = sys.argv[1], sys.argv[2]
data = open(source, 'rb').read()
length = struct.unpack_from('<I', data, 12)[0]
doc = json.loads(data[20:20 + length])
binary = data[20 + length:]
chunk = binary[8:8 + struct.unpack_from('<I', binary, 0)[0]]

dropped = {}
for material in doc['materials']:
    occlusion = material.get('occlusionTexture')
    if not occlusion:
        continue
    image = doc['images'][doc['textures'][occlusion['index']]['source']]
    view = doc['bufferViews'][image['bufferView']]
    start = view.get('byteOffset', 0)
    picture = Image.open(io.BytesIO(chunk[start:start + view['byteLength']])).convert('RGB')
    if max(picture.size) <= 8 and max(picture.getextrema()[c][1] for c in range(3)) < 10:
        del material['occlusionTexture']
        dropped[material['name']] = dropped.get(material['name'], 0) + 1
print('dropped', dropped)

text = json.dumps(doc, separators=(',', ':')).encode('utf-8')
text += b' ' * ((4 - len(text) % 4) % 4)
with open(target, 'wb') as handle:
    handle.write(struct.pack('<III', 0x46546C67, 2, 12 + 8 + len(text) + len(binary)))
    handle.write(struct.pack('<II', len(text), 0x4E4F534A) + text)
    handle.write(binary)
print('wrote', target)
