# Writes one GLB per animation of a GLB, with only the skeleton nodes, a skin listing them (the
# engine reads animations only from files with a skin) and that clip: no meshes, materials or
# textures. By default the per-bone translation and scale channels are dropped (only the hips keep
# their translation), so the clip plays on a skeleton with other bone lengths.
# Usage: python3 tools/extract_clips.py input.glb output_dir [--keep-channels] [--only=a,b,c] [--root=Hips]
#        python3 tools/extract_clips.py --add-skin clip.glb ...   (adds the skin to clips written earlier)
# --root names the bone that keeps its translation (default: any bone ending in Hips).
# --subtree=Spine keeps only the channels of that bone and the bones below it (an upper-body layer).
import json, os, re, struct, sys

def read_glb(path):
    data = open(path, 'rb').read()
    magic, version, length = struct.unpack_from('<III', data, 0)
    if magic != 0x46546C67:
        raise SystemExit(path + ': not a GLB')
    offset = 12
    document = None
    binary = b''
    while offset < length:
        chunk_length, chunk_type = struct.unpack_from('<II', data, offset)
        chunk = data[offset + 8:offset + 8 + chunk_length]
        if chunk_type == 0x4E4F534A:
            document = json.loads(chunk)
        elif chunk_type == 0x004E4942:
            binary = chunk
        offset += 8 + chunk_length
    return document, binary

def write_glb(path, document, binary):
    text = json.dumps(document, separators=(',', ':')).encode('utf-8')
    text += b' ' * ((4 - len(text) % 4) % 4)
    binary += b'\0' * ((4 - len(binary) % 4) % 4)
    total = 12 + 8 + len(text) + 8 + len(binary)
    with open(path, 'wb') as out:
        out.write(struct.pack('<III', 0x46546C67, 2, total))
        out.write(struct.pack('<II', len(text), 0x4E4F534A) + text)
        out.write(struct.pack('<II', len(binary), 0x004E4942) + binary)

def file_name(name, index):
    clean = re.sub(r'[^A-Za-z0-9_.-]+', '_', name).strip('_')
    return clean if clean else 'clip_%d' % index

def make_skin(nodes, channels):
    parent = {child: index for index, node in enumerate(nodes) for child in node.get('children', [])}
    targeted = {channel['target']['node'] for channel in channels}
    tops = []
    for node in targeted:
        up = parent.get(node)
        while up is not None and up not in targeted:
            up = parent.get(up)
        if up is None:
            tops.append(node)
    joints = []
    pending = sorted(tops)
    while pending:
        node = pending.pop(0)
        joints.append(node)
        pending.extend(nodes[node].get('children', []))
    return {'joints': joints}

def subtree_nodes(nodes, name):
    start = next((i for i, node in enumerate(nodes) if node.get('name') == name), None)
    if start is None:
        raise SystemExit('no bone named ' + name)
    found = set()
    pending = [start]
    while pending:
        node = pending.pop()
        found.add(node)
        pending.extend(nodes[node].get('children', []))
    return found

def extract(document, binary, animation, keep_channels, root_bone=None, subtree=None):
    nodes = document['nodes']
    allowed = subtree_nodes(nodes, subtree) if subtree else None
    channels = []
    for channel in animation.get('channels', []):
        path = channel['target']['path']
        if allowed is not None and channel['target']['node'] not in allowed:
            continue
        bone = nodes[channel['target']['node']].get('name', '')
        if not keep_channels:
            if path == 'scale':
                continue
            keep_translation = bone == root_bone if root_bone else bone.endswith('Hips')
            if path == 'translation' and not keep_translation:
                continue
        channels.append(channel)
    used = sorted({channel['sampler'] for channel in channels})
    sampler_map = {old: new for new, old in enumerate(used)}
    samplers = [dict(animation['samplers'][index]) for index in used]

    accessors = []
    views = []
    out_binary = bytearray()
    accessor_map = {}

    def copy_accessor(index):
        if index in accessor_map:
            return accessor_map[index]
        source = dict(document['accessors'][index])
        view = dict(document['bufferViews'][source['bufferView']])
        start = view.get('byteOffset', 0)
        chunk = binary[start:start + view['byteLength']]
        while len(out_binary) % 4:
            out_binary.append(0)
        new_view = {'buffer': 0, 'byteOffset': len(out_binary), 'byteLength': len(chunk)}
        if 'byteStride' in view:
            new_view['byteStride'] = view['byteStride']
        out_binary.extend(chunk)
        views.append(new_view)
        source['bufferView'] = len(views) - 1
        accessors.append(source)
        accessor_map[index] = len(accessors) - 1
        return accessor_map[index]

    for sampler in samplers:
        sampler['input'] = copy_accessor(sampler['input'])
        sampler['output'] = copy_accessor(sampler['output'])
    new_channels = []
    for channel in channels:
        entry = {'sampler': sampler_map[channel['sampler']], 'target': dict(channel['target'])}
        new_channels.append(entry)

    new_nodes = []
    for node in nodes:
        copy = {key: value for key, value in node.items() if key not in ('mesh', 'skin', 'camera', 'extensions')}
        if 'name' in copy:
            for prefix in ('mixamorig:', 'mixamorig'):
                if copy['name'].startswith(prefix) and len(copy['name']) > len(prefix):
                    copy['name'] = copy['name'][len(prefix):]
                    break
        new_nodes.append(copy)
    scenes = [{'nodes': list(scene.get('nodes', []))} for scene in document.get('scenes', [{'nodes': []}])]
    out = {
        'asset': {'version': '2.0', 'generator': 'extract_clips.py'},
        'scene': document.get('scene', 0),
        'scenes': scenes,
        'nodes': new_nodes,
        'skins': [make_skin(new_nodes, new_channels)],
        'animations': [{'name': animation.get('name', ''), 'channels': new_channels, 'samplers': samplers}],
        'accessors': accessors,
        'bufferViews': views,
        'buffers': [{'byteLength': len(out_binary)}],
    }
    return out, bytes(out_binary)

def add_skins(paths):
    for path in paths:
        document, binary = read_glb(path)
        channels = [c for a in document.get('animations', []) for c in a['channels']]
        document['skins'] = [make_skin(document['nodes'], channels)]
        write_glb(path, document, binary)
        print('%s: skin with %d joints' % (path, len(document['skins'][0]['joints'])))

def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    if '--add-skin' in sys.argv:
        add_skins(args)
        return
    if len(args) < 2:
        raise SystemExit(__doc__ or 'usage: extract_clips.py input.glb output_dir')
    keep_channels = '--keep-channels' in sys.argv
    only = None
    root_bone = None
    subtree = None
    for arg in sys.argv[1:]:
        if arg.startswith('--subtree='):
            subtree = arg[len('--subtree='):]
        if arg.startswith('--only='):
            only = set(arg[len('--only='):].split(','))
        if arg.startswith('--root='):
            root_bone = arg[len('--root='):]
    source, target = args[0], args[1]
    document, binary = read_glb(source)
    os.makedirs(target, exist_ok=True)
    animations = document.get('animations', [])
    if not animations:
        raise SystemExit(source + ': no animations')
    for index, animation in enumerate(animations):
        name = animation.get('name', '') or 'clip_%d' % index
        if only and name not in only:
            continue
        clip, clip_binary = extract(document, binary, animation, keep_channels, root_bone, subtree)
        path = os.path.join(target, file_name(name, index) + '.glb')
        write_glb(path, clip, clip_binary)
        print('%s: %d channels, %d bytes' % (path, len(clip['animations'][0]['channels']), os.path.getsize(path)))

main()
