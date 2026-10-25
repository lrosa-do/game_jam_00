# Puts a logo on a screen painted in a level texture: the base colour image of the materials whose
# name starts with PREFIX is scaled up, the rectangle x0 y0 x1 y1 (pixels of the original image) is
# filled with a lit screen and the logo, and the image is written back into the GLB. With --mirror
# the logo is flipped left to right first, for a screen whose UVs show the texture mirrored.
# Usage: python3 tools/screen_logo.py level.glb out.glb PREFIX logo.png x0 y0 x1 y1 [--scale=4] [--mirror]
import io, json, struct, sys
from PIL import Image, ImageDraw

args = [a for a in sys.argv[1:] if not a.startswith('--')]
scale = next((int(a[len('--scale='):]) for a in sys.argv[1:] if a.startswith('--scale=')), 4)
source, target, prefix, logo_path = args[0], args[1], args[2], args[3]
x0, y0, x1, y1 = (int(v) for v in args[4:8])

data = open(source, 'rb').read()
length = struct.unpack_from('<I', data, 12)[0]
doc = json.loads(data[20:20 + length])
offset = 20 + length
bin_length = struct.unpack_from('<I', data, offset)[0]
binary = bytearray(data[offset + 8:offset + 8 + bin_length])

images = set()
for material in doc.get('materials', []):
    if material.get('name', '').startswith(prefix):
        texture = material.get('pbrMetallicRoughness', {}).get('baseColorTexture')
        if texture:
            images.add(doc['textures'][texture['index']]['source'])
if not images:
    raise SystemExit('no material starts with ' + prefix)

logo = Image.open(logo_path).convert('RGBA')
if '--mirror' in sys.argv[1:]:
    logo = logo.transpose(Image.FLIP_LEFT_RIGHT)
for index in sorted(images):
    image = doc['images'][index]
    view = doc['bufferViews'][image['bufferView']]
    start = view.get('byteOffset', 0)
    picture = Image.open(io.BytesIO(binary[start:start + view['byteLength']])).convert('RGBA')
    picture = picture.resize((picture.width * scale, picture.height * scale), Image.LANCZOS)
    box = (x0 * scale, y0 * scale, x1 * scale, y1 * scale)
    width, height = box[2] - box[0], box[3] - box[1]
    screen = Image.new('RGBA', (width, height))
    draw = ImageDraw.Draw(screen)
    for y in range(height):
        t = y / max(height - 1, 1)
        draw.line([(0, y), (width, y)], fill=(int(236 - 18 * t), int(242 - 14 * t), int(250 - 8 * t), 255))
    fit = min(width * 0.8 / logo.width, height * 0.82 / logo.height)
    mark = logo.resize((max(1, int(logo.width * fit)), max(1, int(logo.height * fit))), Image.LANCZOS)
    screen.alpha_composite(mark, ((width - mark.width) // 2, (height - mark.height) // 2))
    picture.paste(screen, box[:2])
    out = io.BytesIO()
    picture.convert('RGB').save(out, format='PNG')
    while len(binary) % 4:
        binary.append(0)
    doc['bufferViews'].append({'buffer': 0, 'byteOffset': len(binary), 'byteLength': len(out.getvalue())})
    binary.extend(out.getvalue())
    image['bufferView'] = len(doc['bufferViews']) - 1
    image['mimeType'] = 'image/png'
    print('image %d: %dx%d with the logo' % (index, picture.width, picture.height))

doc['buffers'][0]['byteLength'] = len(binary)
text = json.dumps(doc, separators=(',', ':')).encode('utf-8')
text += b' ' * ((4 - len(text) % 4) % 4)
while len(binary) % 4:
    binary.append(0)
with open(target, 'wb') as handle:
    handle.write(struct.pack('<III', 0x46546C67, 2, 12 + 8 + len(text) + 8 + len(binary)))
    handle.write(struct.pack('<II', len(text), 0x4E4F534A) + text)
    handle.write(struct.pack('<II', len(binary), 0x004E4942) + bytes(binary))
print('wrote', target)
