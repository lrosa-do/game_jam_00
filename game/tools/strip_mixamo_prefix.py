# Renames the bones of GLB files from "mixamorig:Hips" or "mixamorigHips" to "Hips", in place.
# Usage: python3 tools/strip_mixamo_prefix.py file.glb [more.glb ...]
import json, struct, sys

def strip(name):
    for prefix in ('mixamorig:', 'mixamorig'):
        if name.startswith(prefix) and len(name) > len(prefix):
            return name[len(prefix):]
    return name

def rename(path):
    data = open(path, 'rb').read()
    magic, version, length = struct.unpack_from('<III', data, 0)
    json_length, json_type = struct.unpack_from('<II', data, 12)
    document = json.loads(data[20:20 + json_length])
    rest = data[20 + json_length:]
    changed = 0
    for node in document.get('nodes', []):
        name = node.get('name')
        if name and strip(name) != name:
            node['name'] = strip(name)
            changed += 1
    for animation in document.get('animations', []):
        name = animation.get('name')
        if name and strip(name) != name:
            animation['name'] = strip(name)
    text = json.dumps(document, separators=(',', ':')).encode('utf-8')
    text += b' ' * ((4 - len(text) % 4) % 4)
    header = struct.pack('<III', magic, version, 20 + len(text) + len(rest))
    open(path, 'wb').write(header + struct.pack('<II', len(text), json_type) + text + rest)
    print('%s: %d bones renamed' % (path, changed))

for path in sys.argv[1:]:
    rename(path)
