# Puts the two garage vans where they were set by hand in Blender (a reference GLB): the export had
# the bodies sunk a metre into the floor and their wheels and doors off to the side. Each van node
# of the level (name with van_parent) is moved so its world box starts where the same node's does in
# the reference; van nodes the reference no longer has are dropped. Only translations change.
# Usage: python3 tools/fix_vans.py level.glb reference.glb out.glb
import json, struct, sys
import numpy as np

source, reference, target = sys.argv[1], sys.argv[2], sys.argv[3]


def read(path):
    data = open(path, 'rb').read()
    length = struct.unpack_from('<I', data, 12)[0]
    return json.loads(data[20:20 + length]), data[20 + length:]


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


def boxes(doc):
    out = {}

    def walk(index, parent):
        node = doc['nodes'][index]
        world = parent @ local_matrix(node)
        name = node.get('name', '')
        if 'van_parent' in name and 'mesh' in node:
            low = np.full(3, 1e9)
            for prim in doc['meshes'][node['mesh']]['primitives']:
                acc = doc['accessors'][prim['attributes']['POSITION']]
                for x in (acc['min'][0], acc['max'][0]):
                    for y in (acc['min'][1], acc['max'][1]):
                        for z in (acc['min'][2], acc['max'][2]):
                            low = np.minimum(low, (world @ np.array([x, y, z, 1.0]))[:3])
            out[name] = (index, low, parent)
        for child in node.get('children', []):
            walk(child, world)

    for root in doc['scenes'][0]['nodes']:
        walk(root, np.eye(4))
    return out


doc, binary = read(source)
ref, _ = read(reference)
mine = boxes(doc)
theirs = boxes(ref)
dropped = []
for name, (index, low, parent) in mine.items():
    if name not in theirs:
        dropped.append(index)
        continue
    delta = theirs[name][1] - low
    if np.abs(delta).max() < 0.005:
        continue
    offset = np.linalg.inv(parent[:3, :3]) @ delta
    node = doc['nodes'][index]
    if 'matrix' in node:
        node['matrix'][12] += offset[0]
        node['matrix'][13] += offset[1]
        node['matrix'][14] += offset[2]
    else:
        t = node.get('translation', [0.0, 0.0, 0.0])
        node['translation'] = [t[0] + offset[0], t[1] + offset[1], t[2] + offset[2]]
    print('%-62s %+.2f %+.2f %+.2f' % (name[-62:], delta[0], delta[1], delta[2]))
# A dropped node stays in the node list (indices are shared) but leaves its parent's children.
for node in doc['nodes']:
    if 'children' in node:
        node['children'] = [c for c in node['children'] if c not in dropped]
print('dropped', [doc['nodes'][i]['name'] for i in dropped])

text = json.dumps(doc, separators=(',', ':')).encode('utf-8')
text += b' ' * ((4 - len(text) % 4) % 4)
with open(target, 'wb') as handle:
    handle.write(struct.pack('<III', 0x46546C67, 2, 12 + 8 + len(text) + len(binary)))
    handle.write(struct.pack('<II', len(text), 0x4E4F534A) + text)
    handle.write(binary)
print('wrote', target)
