# Finds the lamp fixtures of the level (primitives whose material is a lamp texture, clustered by
# proximity) and writes OmniLight3D nodes under the "Lights" node of a scene.
# Usage: python3 tools/make_lights.py Assets/Levels/esquadra_policia.glb Scenes/game.scene.json [--one-per-room]
# --one-per-room keeps one shadowed light per room (the ceiling lamp nearest the middle of the room's
# lamps, with a range that reaches them all), for character shadows over the baked lightmaps.
# --spot writes those as spot lights pointing down (one shadow map each instead of six), with the
# shadow off: the game turns it on for the light nearest the player.
import json, math, re, struct, sys

args = [a for a in sys.argv[1:] if not a.startswith('--')]
level, scene_path = args[0], args[1]
one_per_room = '--one-per-room' in sys.argv
spot = '--spot' in sys.argv
f = open(level, 'rb'); f.read(12); ln, ty = struct.unpack('<II', f.read(8)); doc = json.loads(f.read(ln)); f.tell()
bl, bt = struct.unpack('<II', f.read(8)); data = f.read(bl)
nodes = doc['nodes']; mats = doc['materials']
parent = {c: i for i, n in enumerate(nodes) for c in n.get('children', [])}

def world(i):
    t = [0.0, 0.0, 0.0]; k = i
    while k is not None:
        tr = nodes[k].get('translation', [0, 0, 0]); t = [t[a] + tr[a] for a in range(3)]; k = parent.get(k)
    return t

def positions(acc):
    a = doc['accessors'][acc]; bv = doc['bufferViews'][a['bufferView']]
    off = bv.get('byteOffset', 0) + a.get('byteOffset', 0); stride = bv.get('byteStride', 12)
    return [struct.unpack_from('<3f', data, off + i * stride) for i in range(a['count'])]

# kind: (pattern, color, energy, range, shadow, cluster radius, min vertices)
KINDS = [
    (r'lamp16', (1.0, 0.86, 0.66), 1.2, 7.0, True, 1.2, 60),
    (r'Lamp05|Lamp02|Lamp04|fluorescent', (1.0, 0.95, 0.85), 1.0, 7.0, True, 2.0, 80),
    (r'DeskLamp|TableLamp|tablelamp|lampshader', (1.0, 0.8, 0.5), 0.6, 3.5, False, 0.8, 120),
]
ROOMS = {'startroom', 'rec_room', 'interrogation01', 'reception', 'briefing', 'cells', 'cells02', 'cells03', 'cells04', 'entrance_to_cells', 'staircase', 'stairs_to_lockup', 'stairs_to_garage', 'gents', 'ladies', 'garage'}

def cluster(points, radius):
    clusters = []
    for p in points:
        for c in clusters:
            cx, cy, cz, n = c
            if math.dist((cx / n, cy / n, cz / n), p) <= radius:
                c[0] += p[0]; c[1] += p[1]; c[2] += p[2]; c[3] += 1
                break
        else:
            clusters.append([p[0], p[1], p[2], 1])
    return [((c[0] / c[3], c[1] / c[3], c[2] / c[3]), c[3]) for c in clusters]

lights = []
for ni, n in enumerate(nodes):
    if 'mesh' not in n: continue
    room = n.get('name', '').split('::')[0]
    if room not in ROOMS: continue
    w = world(ni)
    for p in doc['meshes'][n['mesh']]['primitives']:
        name = mats[p.get('material', 0)].get('name', '')
        for pattern, color, energy, reach, shadow, radius, minimum in KINDS:
            if not re.search(pattern, name, re.I): continue
            pts = [(q[0] + w[0], q[1] + w[1], q[2] + w[2]) for q in positions(p['attributes']['POSITION'])]
            for (center, count) in cluster(pts, radius):
                if count < minimum: continue
                lights.append({'room': room, 'kind': pattern.split('|')[0], 'pos': [round(v, 2) for v in center], 'color': color, 'energy': energy, 'range': reach, 'shadow': shadow})
            break

# drop near duplicates
kept = []
for l in lights:
    if any(math.dist(l['pos'], k['pos']) < 0.6 for k in kept): continue
    kept.append(l)
lights = kept
if one_per_room:
    single = []
    for room in sorted({l['room'] for l in lights}):
        group = [l for l in lights if l['room'] == room and l['shadow']] or [l for l in lights if l['room'] == room]
        mx = sum(l['pos'][0] for l in group) / len(group)
        mz = sum(l['pos'][2] for l in group) / len(group)
        best = min(group, key=lambda l: (l['pos'][0] - mx) ** 2 + (l['pos'][2] - mz) ** 2)
        reach = max(math.dist(best['pos'], l['pos']) for l in group) + 4.0
        best = dict(best, shadow=True, range=round(min(max(reach, 7.0), 14.0), 1), energy=max(best['energy'], 1.2))
        single.append(best)
    lights = single
per_room = {}
limited = []
for l in sorted(lights, key=lambda l: not l['shadow']):
    key = (l['room'], l['shadow'])
    cap = 4 if l['shadow'] else 3
    if per_room.get(key, 0) >= cap: continue
    per_room[key] = per_room.get(key, 0) + 1
    limited.append(l)
lights = limited
shadowed = sum(1 for l in lights if l['shadow'])
print('lights', len(lights), 'with shadow', shadowed)
from collections import Counter
print(Counter((l['room'], l['kind']) for l in lights).most_common(30))

children = []
for i, l in enumerate(lights):
    props = {'position': l['pos'], 'light_color': list(l['color']) + [1.0], 'light_energy': l['energy']}
    if spot:
        props.update({'rotation': [-1.5708, 0.0, 0.0], 'spot_range': min(l['range'], 8.0), 'spot_angle': 65.0})
    else:
        props['omni_range'] = l['range']
    if l['shadow']:
        props.update({'shadow_enabled': not spot, 'shadow_blur': 1.0, 'shadow_opacity': 0.85})
    children.append({'class': 'SpotLight3D' if spot else 'OmniLight3D', 'name': '%s_%s_%d' % (l['room'], l['kind'].lower(), i), 'properties': props})
scene = json.load(open(scene_path))
scene['root']['children'] = [c for c in scene['root']['children'] if c.get('name') != 'Lights']
scene['root']['children'].insert(2, {'class': 'Node3D', 'name': 'Lights', 'children': children})
json.dump(scene, open(scene_path, 'w'), indent=2); open(scene_path, 'a').write('\n')
print('wrote', scene_path)
