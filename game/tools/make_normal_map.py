# Makes a normal map (glTF/OpenGL convention, green up) and a roughness map from a colour
# texture: the luminance, slightly blurred, is taken as a height field. Bright paint and metal
# come out smoother than dark grime. Good enough to give painted plates, seams and rivets relief.
# Usage: python3 tools/make_normal_map.py albedo.png normal.png roughness.png [--strength=3.0]
#        [--blur=1.2] [--rough-min=0.35] [--rough-max=0.9]
import sys
import numpy as np
from PIL import Image, ImageFilter

args = [a for a in sys.argv[1:] if not a.startswith('--')]
def option(name, default):
    for a in sys.argv[1:]:
        if a.startswith('--' + name + '='):
            return float(a.split('=', 1)[1])
    return default
strength = option('strength', 3.0)
blur = option('blur', 1.2)
rough_min = option('rough-min', 0.35)
rough_max = option('rough-max', 0.9)
source, normal_path, rough_path = args[0], args[1], args[2]

albedo = Image.open(source).convert('RGB')
grey = albedo.convert('L').filter(ImageFilter.GaussianBlur(blur))
height = np.asarray(grey, dtype=np.float32) / 255.0

# Central differences, wrapping at the edges like the texture does.
dx = (np.roll(height, -1, axis=1) - np.roll(height, 1, axis=1)) * 0.5
dy = (np.roll(height, -1, axis=0) - np.roll(height, 1, axis=0)) * 0.5
nx = -dx * strength
ny = dy * strength  # image rows go down, the green channel points up
nz = np.ones_like(height)
length = np.sqrt(nx * nx + ny * ny + nz * nz)
normal = np.stack([nx / length, ny / length, nz / length], axis=-1)
Image.fromarray(((normal * 0.5 + 0.5) * 255.0 + 0.5).clip(0, 255).astype(np.uint8), 'RGB').save(normal_path)

lum = np.asarray(albedo.convert('L'), dtype=np.float32) / 255.0
rough = rough_max - (rough_max - rough_min) * lum
Image.fromarray((rough * 255.0 + 0.5).clip(0, 255).astype(np.uint8), 'L').convert('RGB').save(rough_path)
print('wrote', normal_path, rough_path)
