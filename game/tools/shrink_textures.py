# Shrinks the textures embedded in a GLB to at most SIZE pixels a side (JPEG for images without
# alpha, PNG otherwise) and repacks the binary chunk, so the file really gets smaller; meshes,
# skins and animations are copied as they are.
# Usage: python3 tools/shrink_textures.py input.glb output.glb [--size=1024] [--quality=90]
import io, json, struct, sys
from PIL import Image

args = [a for a in sys.argv[1:] if not a.startswith('--')]
size = next((int(a[len('--size='):]) for a in sys.argv[1:] if a.startswith('--size=')), 1024)
quality = next((int(a[len('--quality='):]) for a in sys.argv[1:] if a.startswith('--quality=')), 90)
source, target = args[0], args[1]

data = open(source, 'rb').read()
length = struct.unpack_from('<I', data, 12)[0]
doc = json.loads(data[20:20 + length])
offset = 20 + length
bin_length = struct.unpack_from('<I', data, offset)[0]
binary = data[offset + 8:offset + 8 + bin_length]

replaced = {}
for image in doc.get('images', []):
    if 'bufferView' not in image:
        continue
    view = doc['bufferViews'][image['bufferView']]
    start = view.get('byteOffset', 0)
    picture = Image.open(io.BytesIO(binary[start:start + view['byteLength']]))
    if max(picture.size) > size:
        scale = size / max(picture.size)
        picture = picture.resize((max(1, int(picture.width * scale)), max(1, int(picture.height * scale))), Image.LANCZOS)
    out = io.BytesIO()
    if picture.mode in ('RGBA', 'LA') or 'transparency' in picture.info:
        picture.save(out, format='PNG', optimize=True)
        image['mimeType'] = 'image/png'
    else:
        picture.convert('RGB').save(out, format='JPEG', quality=quality)
        image['mimeType'] = 'image/jpeg'
    replaced[image['bufferView']] = out.getvalue()
    print('image %s: %dx%d, %d KB' % (image.get('name', '?'), picture.width, picture.height, len(out.getvalue()) // 1024))

packed = bytearray()
for index, view in enumerate(doc['bufferViews']):
    chunk = replaced.get(index)
    if chunk is None:
        start = view.get('byteOffset', 0)
        chunk = binary[start:start + view['byteLength']]
    while len(packed) % 4:
        packed.append(0)
    view['byteOffset'] = len(packed)
    view['byteLength'] = len(chunk)
    packed.extend(chunk)
while len(packed) % 4:
    packed.append(0)
doc['buffers'][0]['byteLength'] = len(packed)

text = json.dumps(doc, separators=(',', ':')).encode('utf-8')
text += b' ' * ((4 - len(text) % 4) % 4)
with open(target, 'wb') as handle:
    handle.write(struct.pack('<III', 0x46546C67, 2, 12 + 8 + len(text) + 8 + len(packed)))
    handle.write(struct.pack('<II', len(text), 0x4E4F534A) + text)
    handle.write(struct.pack('<II', len(packed), 0x004E4942) + bytes(packed))
print('wrote', target, (12 + 8 + len(text) + 8 + len(packed)) // 1024, 'KB')
