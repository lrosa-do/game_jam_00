# Moves the parts of a level GLB that the game can break (glass panes, screens, extinguishers...)
# out of the room meshes into nodes of their own, named break_<kind>_<n>, one per connected piece.
# A part keeps its material, so its UVs and baked lightmap stay the same; with physics=concave each
# break_ node gets its own static body, which the game removes when the part breaks.
# Usage: python3 tools/split_breakables.py level.glb out.glb [--dry-run]
#        rules are in RULES below: kind, material name pattern, smallest and largest area (m2).
#        GROUPS are whole objects made of several pieces (a car and its wheels): the pieces of the
#        materials that match, closer than GROUP_MARGIN to each other, become one node. A third
#        field limits a group to the nodes whose name starts with it (one room).
import json, math, re, struct, sys

RULES = [
    ('glass', r'raised-glass|^Glass01_', 0.05, 12.0),
    ('vase', r'^Vase_glass', 0.005, 1.0),
    ('monitor', r'ComputerScreen|SecurityMonitor', 0.01, 2.0),
]

GROUPS = [
    ('car', r'^(police_car|mercedes_suv|golf|saab)$|^vlads_car'), # vlads_car: the test car in the street
    ('extinguisher', r'^FireExt_'), # bottle, hose and handle are separate pieces
    # Furniture of the briefing room that flies about: the tables with the projector on them,
    # and the plastic chairs.
    ('table', r'^(battered_table01|Projector01)', 'briefing'),
    ('chair', r'^plastic-seat', 'briefing'),
    # Loose things around the precinct that fly, topple or roll when shot, kicked or blasted.
    ('prop', r'^(Maxhouse_Furniture_TrashBin|Trashbin0[12]|roomplant01|fan-standing-rusty|heater01|coffee-maker-crappy|paper-shredder|toolcart-colorful|Phone01|telephone_(old|new)|Keyboard01|DeskLamp_01|TableLampMetal_02|brass_tablelamp|television-old01|television-set-80s|Maxhouse_Furniture_Electronics|papercup|coffee_cup|burger01|fries)'),
    # The soda machines: they blow up and topple.
    ('vending', r'^(SodaMachine01|vending-machine-temp)'),
]
GROUP_MARGIN = 0.6

args = [a for a in sys.argv[1:] if not a.startswith('--')]
dry = '--dry-run' in sys.argv
source, target = args[0], args[1]

data = open(source, 'rb').read()
length = struct.unpack_from('<I', data, 12)[0]
doc = json.loads(data[20:20 + length])
offset = 20 + length
bin_length = struct.unpack_from('<I', data, offset)[0]
binary = bytearray(data[offset + 8:offset + 8 + bin_length])

FORMATS = {5126: ('f', 4), 5125: ('I', 4), 5123: ('H', 2), 5121: ('B', 1)}
SIZES = {'SCALAR': 1, 'VEC2': 2, 'VEC3': 3, 'VEC4': 4}

def read(index):
    acc = doc['accessors'][index]
    view = doc['bufferViews'][acc['bufferView']]
    code, size = FORMATS[acc['componentType']]
    width = SIZES[acc['type']]
    stride = view.get('byteStride', size * width)
    base = view.get('byteOffset', 0) + acc.get('byteOffset', 0)
    return [list(struct.unpack_from('<%d%s' % (width, code), binary, base + i * stride)) for i in range(acc['count'])]

def append(values, component, kind, target_kind=None):
    code, size = FORMATS[component]
    while len(binary) % 4:
        binary.append(0)
    start = len(binary)
    flat = [v for row in values for v in row]
    binary.extend(struct.pack('<%d%s' % (len(flat), code), *flat))
    view = {'buffer': 0, 'byteOffset': start, 'byteLength': len(binary) - start}
    if target_kind:
        view['target'] = target_kind
    doc['bufferViews'].append(view)
    acc = {'bufferView': len(doc['bufferViews']) - 1, 'componentType': component, 'count': len(values), 'type': kind}
    if kind == 'VEC3' and component == 5126:
        acc['min'] = [min(v[i] for v in values) for i in range(3)]
        acc['max'] = [max(v[i] for v in values) for i in range(3)]
    doc['accessors'].append(acc)
    return len(doc['accessors']) - 1

def area(tri, positions):
    a, b, c = (positions[i] for i in tri)
    u = [b[k] - a[k] for k in range(3)]
    v = [c[k] - a[k] for k in range(3)]
    cx = u[1] * v[2] - u[2] * v[1]
    cy = u[2] * v[0] - u[0] * v[2]
    cz = u[0] * v[1] - u[1] * v[0]
    return 0.5 * math.sqrt(cx * cx + cy * cy + cz * cz)

def islands(triangles, positions):
    parent = list(range(len(positions)))
    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i
    weld = {}
    for i, p in enumerate(positions):
        key = (round(p[0], 3), round(p[1], 3), round(p[2], 3))
        if key in weld:
            parent[find(i)] = find(weld[key])
        else:
            weld[key] = i
    for a, b, c in triangles:
        parent[find(b)] = find(a)
        parent[find(c)] = find(a)
    groups = {}
    for tri in triangles:
        groups.setdefault(find(tri[0]), []).append(tri)
    return list(groups.values())

materials = doc.get('materials', [])
counts = {}
made = 0

def bounds(group, positions):
    points = [positions[i] for t in group for i in t]
    return [min(p[k] for p in points) for k in range(3)], [max(p[k] for p in points) for k in range(3)]

def touches(a, b):
    return all(a[0][k] - GROUP_MARGIN <= b[1][k] and b[0][k] - GROUP_MARGIN <= a[1][k] for k in range(3))

def emit_primitive(prim, attributes, group):
    used = sorted({i for t in group for i in t})
    remap = {old: new for new, old in enumerate(used)}
    out = {'attributes': {}, 'material': prim.get('material', 0)}
    for key, values in attributes.items():
        acc = doc['accessors'][prim['attributes'][key]]
        out['attributes'][key] = append([values[i] for i in used], acc['componentType'], acc['type'], 34962)
    out['indices'] = append([[remap[i]] for t in group for i in t], 5125, 'SCALAR', 34963)
    return out

for node_index in range(len(doc['nodes'])):
    node = doc['nodes'][node_index]
    if 'mesh' not in node:
        continue
    mesh = doc['meshes'][node['mesh']]
    pieces = []
    taken = set()
    for p, prim in enumerate(mesh['primitives']):
        name = materials[prim.get('material', 0)].get('name', '') if materials else ''
        node_name = node.get('name', '')
        kind = next((g[0] for g in GROUPS if re.search(g[1], name) and (len(g) < 3 or node_name.startswith(g[2]))), None)
        if kind is None or 'indices' not in prim:
            continue
        attributes = {key: read(index) for key, index in prim['attributes'].items()}
        flat = [row[0] for row in read(prim['indices'])]
        triangles = [tuple(flat[i:i + 3]) for i in range(0, len(flat) - 2, 3)]
        for group in islands(triangles, attributes['POSITION']):
            pieces.append((kind, p, attributes, group, bounds(group, attributes['POSITION'])))
        taken.add(p)
    if not pieces:
        continue
    parent = list(range(len(pieces)))
    def root(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i
    for a in range(len(pieces)):
        for b in range(a + 1, len(pieces)):
            if pieces[a][0] == pieces[b][0] and touches(pieces[a][4], pieces[b][4]):
                parent[root(b)] = root(a)
    clusters = {}
    for i, piece in enumerate(pieces):
        clusters.setdefault(root(i), []).append(piece)
    for members in clusters.values():
        kind = members[0][0]
        counts[kind] = counts.get(kind, 0) + 1
        if dry:
            continue
        by_primitive = {}
        for piece in members:
            by_primitive.setdefault(piece[1], (piece[2], []))[1].extend(piece[3])
        primitives = [emit_primitive(mesh['primitives'][p], attrs, tris) for p, (attrs, tris) in sorted(by_primitive.items())]
        name = 'break_%s_%d' % (kind, counts[kind])
        doc['meshes'].append({'name': name, 'primitives': primitives})
        doc['nodes'].append({'name': name, 'mesh': len(doc['meshes']) - 1})
        node.setdefault('children', []).append(len(doc['nodes']) - 1)
        made += 1
    if not dry:
        mesh['primitives'] = [prim for p, prim in enumerate(mesh['primitives']) if p not in taken]
for node_index in range(len(doc['nodes'])):
    node = doc['nodes'][node_index]
    if 'mesh' not in node:
        continue
    mesh = doc['meshes'][node['mesh']]
    keep = []
    for prim in mesh['primitives']:
        name = materials[prim.get('material', 0)].get('name', '') if materials else ''
        rule = next((r for r in RULES if re.search(r[1], name)), None)
        if rule is None or 'indices' not in prim:
            keep.append(prim)
            continue
        kind, pattern, smallest, largest = rule
        attributes = {key: read(index) for key, index in prim['attributes'].items()}
        flat = [row[0] for row in read(prim['indices'])]
        triangles = [tuple(flat[i:i + 3]) for i in range(0, len(flat) - 2, 3)]
        positions = attributes['POSITION']
        leftover = []
        for group in islands(triangles, positions):
            size = sum(area(t, positions) for t in group)
            if size < smallest or size > largest:
                leftover.extend(group)
                continue
            counts[kind] = counts.get(kind, 0) + 1
            if dry:
                continue
            used = sorted({i for t in group for i in t})
            remap = {old: new for new, old in enumerate(used)}
            prim_out = {'attributes': {}, 'material': prim.get('material', 0)}
            for key, values in attributes.items():
                acc = doc['accessors'][prim['attributes'][key]]
                prim_out['attributes'][key] = append([values[i] for i in used], acc['componentType'], acc['type'], 34962)
            indices = [[remap[i]] for t in group for i in t]
            prim_out['indices'] = append(indices, 5125, 'SCALAR', 34963)
            doc['meshes'].append({'name': 'break_%s_%d' % (kind, counts[kind]), 'primitives': [prim_out]})
            doc['nodes'].append({'name': 'break_%s_%d' % (kind, counts[kind]), 'mesh': len(doc['meshes']) - 1})
            node.setdefault('children', []).append(len(doc['nodes']) - 1)
            made += 1
        if dry or len(leftover) == len(triangles):
            keep.append(prim)
        elif leftover:
            rest = dict(prim)
            rest['indices'] = append([[i] for t in leftover for i in t], 5125, 'SCALAR', 34963)
            keep.append(rest)
    if keep:
        mesh['primitives'] = keep
    elif not dry:
        del node['mesh']

print('breakables', counts)
if dry:
    sys.exit(0)
doc['buffers'][0]['byteLength'] = len(binary)
text = json.dumps(doc, separators=(',', ':')).encode('utf-8')
text += b' ' * ((4 - len(text) % 4) % 4)
while len(binary) % 4:
    binary.append(0)
total = 12 + 8 + len(text) + 8 + len(binary)
with open(target, 'wb') as out:
    out.write(struct.pack('<III', 0x46546C67, 2, total))
    out.write(struct.pack('<II', len(text), 0x4E4F534A) + text)
    out.write(struct.pack('<II', len(binary), 0x004E4942) + bytes(binary))
print('wrote', target, made, 'parts')
