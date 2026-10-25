# Sets the door leaves of the level on their floor: the exporter left most prefab doors about half
# their height off (some hanging 1.4 m in the air, some sunk into the floor). Each leaf (a node
# whose last name part starts with door, d_cell_door) is moved up or down so its bottom is on the
# nearest storey (FLOORS); only the node's translation changes, so its collision moves with it.
# The garage exit (d_door_exit, on the street level) is right and left alone, as are the doors of
# cabinets and vans.
# Usage: python3 tools/floor_doors.py level.glb out.glb
import json, struct, sys
import numpy as np

FLOORS = [0.0, -5.0, -10.0]
source, target = sys.argv[1], sys.argv[2]
data = open(source, 'rb').read()
length = struct.unpack_from('<I', data, 12)[0]
doc = json.loads(data[20:20 + length])
binary = data[20 + length:]


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


# A door leaf of a room, as the game takes it (game.door_name): not the doors of a cabinet or a van.
def leaf(name):
    if 'cabinet' in name or 'closet' in name or 'van_parent' in name:
        return False
    last = name.replace('::', '__').split('__')[-1]
    return last.startswith('door') or last.startswith('d_cell_door')


def bottom(node, world):
    low = 1e9
    for prim in doc['meshes'][node['mesh']]['primitives']:
        acc = doc['accessors'][prim['attributes']['POSITION']]
        for x in (acc['min'][0], acc['max'][0]):
            for y in (acc['min'][1], acc['max'][1]):
                for z in (acc['min'][2], acc['max'][2]):
                    low = min(low, (world @ np.array([x, y, z, 1.0]))[1])
    return low


moved = 0


def walk(index, parent):
    global moved
    node = doc['nodes'][index]
    world = parent @ local_matrix(node)
    name = node.get('name', '')
    if 'mesh' in node and leaf(name):
        low = bottom(node, world)
        floor = min(FLOORS, key=lambda f: abs(low - f))
        dy = floor - low
        if abs(dy) > 0.02:
            # The world offset (0, dy, 0) in the parent's space, added to the translation.
            offset = np.linalg.inv(parent[:3, :3]) @ np.array([0.0, dy, 0.0])
            if 'matrix' in node:
                m = node['matrix']
                m[12] += offset[0]
                m[13] += offset[1]
                m[14] += offset[2]
            else:
                t = node.get('translation', [0.0, 0.0, 0.0])
                node['translation'] = [t[0] + offset[0], t[1] + offset[1], t[2] + offset[2]]
            world = parent @ local_matrix(node)
            moved += 1
            print('%-60s %+.2f m' % (name[:60], dy))
    for child in node.get('children', []):
        walk(child, world)


for root in doc['scenes'][0]['nodes']:
    walk(root, np.eye(4))
print('moved', moved, 'door leaves')

text = json.dumps(doc, separators=(',', ':')).encode('utf-8')
text += b' ' * ((4 - len(text) % 4) % 4)
with open(target, 'wb') as handle:
    handle.write(struct.pack('<III', 0x46546C67, 2, 12 + 8 + len(text) + len(binary)))
    handle.write(struct.pack('<II', len(text), 0x4E4F534A) + text)
    handle.write(binary)
print('wrote', target)
