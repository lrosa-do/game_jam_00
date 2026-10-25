# Draws the minimap of the level, one picture per storey: the floor (upward-facing triangles of
# the rooms within a band round the storey's height) filled dim, and the walls (the vertical
# triangles standing on that storey) as bright lines. World x runs right and world z runs down
# the picture, SCALE pixels per metre, from the fixed corner (MIN_X, MIN_Z): the game maps her
# position with the same constants (Game.draw_minimap), and the fixed frame leaves over 20 m of
# margin round the level so a window round her never leaves the picture.
# Usage: python3 tools/minimap.py level.glb Assets/Textures/Minimap
import json, os, struct, sys
import numpy as np
from PIL import Image, ImageDraw, ImageFilter

SCALE = 6
MIN_X, MIN_Z = -80.0, -48.0
SIZE_X, SIZE_Z = 120.0, 166.0  # metres covered by a picture
STOREYS = [0.0, -5.0, -10.0]
BAND = (-0.7, 1.1)  # floor height accepted round a storey: steps up to a metre above count
WALL_BAND = (0.4, 2.2)  # a wall counts on a storey when it spans this height above its floor

source, out_dir = sys.argv[1], sys.argv[2]
data = open(source, 'rb').read()
length = struct.unpack_from('<I', data, 12)[0]
doc = json.loads(data[20:20 + length])
binary = data[20 + length + 8:]
FORMATS = {5126: ('f', 4), 5125: ('I', 4), 5123: ('H', 2), 5121: ('B', 1)}


def read(index, width):
    acc = doc['accessors'][index]
    view = doc['bufferViews'][acc['bufferView']]
    code, size = FORMATS[acc['componentType']]
    stride = view.get('byteStride', size * width)
    start = view.get('byteOffset', 0) + acc.get('byteOffset', 0)
    return np.array([struct.unpack_from('<%d%s' % (width, code), binary, start + k * stride) for k in range(acc['count'])])


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


# The floor and wall triangles of the rooms (the loose props and breakables are neither).
triangles = []
walls = []


def walk(index, parent, rooms):
    node = doc['nodes'][index]
    world = parent @ local_matrix(node)
    name = node.get('name', '')
    rooms = rooms or name == 'salas'
    if 'mesh' in node and rooms and not name.startswith('break'):
        for prim in doc['meshes'][node['mesh']]['primitives']:
            positions = read(prim['attributes']['POSITION'], 3)
            indices = read(prim['indices'], 1)[:, 0].reshape(-1, 3)
            points = (world @ np.c_[positions, np.ones(len(positions))].T).T[:, :3]
            tris = points[indices]
            normals = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
            lengths = np.linalg.norm(normals, axis=1)
            keep = lengths > 1e-6
            up = np.zeros(len(tris), dtype=bool)
            up[keep] = normals[keep][:, 1] / lengths[keep] > 0.7
            triangles.extend(tris[up].tolist())
            side = np.zeros(len(tris), dtype=bool)
            side[keep] = np.abs(normals[keep][:, 1] / lengths[keep]) < 0.3
            walls.extend(tris[side].tolist())
    for child in node.get('children', []):
        walk(child, world, rooms)


for root in doc['scenes'][0]['nodes']:
    walk(root, np.eye(4), False)
triangles = np.array(triangles)
walls = np.array(walls)
low = triangles.reshape(-1, 3).min(0)
high = triangles.reshape(-1, 3).max(0)
min_x, min_z = MIN_X, MIN_Z
width = int(SIZE_X * SCALE)
height = int(SIZE_Z * SCALE)
print('level x %.1f..%.1f z %.1f..%.1f in the frame x %.0f..%.0f z %.0f..%.0f, pictures %dx%d' % (
    low[0], high[0], low[2], high[2], MIN_X, MIN_X + SIZE_X, MIN_Z, MIN_Z + SIZE_Z, width, height))
assert low[0] > MIN_X + 20 and high[0] < MIN_X + SIZE_X - 20 and low[2] > MIN_Z + 20 and high[2] < MIN_Z + SIZE_Z - 20, 'level too close to the frame'



def walls_of(storey):
    ys = walls[:, :, 1]
    top = ys.max(1)
    bottom = ys.min(1)
    keep = (bottom < storey + WALL_BAND[1]) & (top > storey + WALL_BAND[0]) & (bottom > storey - 1.5)
    return walls[keep]


os.makedirs(out_dir, exist_ok=True)
for storey in STOREYS:
    mask = Image.new('L', (width, height), 0)
    draw = ImageDraw.Draw(mask)
    count = 0
    for tri in triangles:
        y = (tri[0][1] + tri[1][1] + tri[2][1]) / 3.0
        if y < storey + BAND[0] or y > storey + BAND[1]:
            continue
        draw.polygon([((p[0] - min_x) * SCALE, (p[2] - min_z) * SCALE) for p in tri], fill=255)
        count += 1
    # The boundary of the floor is the walls: the mask less the mask shrunk by a pixel or two.
    inner = mask.filter(ImageFilter.MinFilter(5))
    edge = Image.eval(Image.merge('L', [inner]), lambda v: 255 - v)
    boundary = Image.new('L', (width, height), 0)
    boundary.paste(mask, (0, 0), edge)
    picture = Image.new('RGBA', (width, height), (0, 0, 0, 0))
    picture.paste((70, 120, 140, 150), (0, 0), mask)
    picture.paste((120, 190, 215, 200), (0, 0), boundary)
    # The walls standing on this storey, as the lines their triangles make seen from above.
    lines = ImageDraw.Draw(picture)
    wall_count = 0
    for tri in walls_of(storey):
        pts = [((p[0] - min_x) * SCALE, (p[2] - min_z) * SCALE) for p in tri]
        lines.line([pts[0], pts[1], pts[2], pts[0]], fill=(185, 240, 255, 255), width=2)
        wall_count += 1
    name = 'floor_%d.png' % int(-storey)
    picture.save(os.path.join(out_dir, name))
    print('%-12s %5d floor triangles, %5d wall triangles' % (name, count, wall_count))

print('game constants: MINIMAP_MIN_X %.1f MINIMAP_MIN_Z %.1f MINIMAP_SCALE %d MINIMAP_SIZE_X %.1f MINIMAP_SIZE_Z %.1f' % (MIN_X, MIN_Z, SCALE, SIZE_X, SIZE_Z))
