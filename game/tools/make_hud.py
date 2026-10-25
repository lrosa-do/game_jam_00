# Draws the HUD sprites (needs Pillow). Usage: python3 tools/make_hud.py
import math, os
from PIL import Image, ImageDraw, ImageFilter

out = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'Assets', 'Textures', 'Hud')
os.makedirs(out, exist_ok=True)
S = 4

def save(image, name):
    image = image.resize((image.width // S, image.height // S), Image.LANCZOS)
    image.save(os.path.join(out, name))
    print(name, image.size)

def reticle():
    size = 64 * S
    img = Image.new('RGBA', (size, size), (0, 0, 0, 0))
    d = ImageDraw.Draw(img)
    c = size / 2
    gap, length, width = 9 * S, 11 * S, 2 * S
    for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        d.line([(c + dx * gap, c + dy * gap), (c + dx * (gap + length), c + dy * (gap + length))], fill=(255, 255, 255, 255), width=width)
    d.ellipse([c - 1.6 * S, c - 1.6 * S, c + 1.6 * S, c + 1.6 * S], fill=(255, 255, 255, 255))
    glow = img.filter(ImageFilter.GaussianBlur(3 * S))
    base = Image.new('RGBA', (size, size), (0, 0, 0, 0))
    base.alpha_composite(Image.eval(glow, lambda v: v).point(lambda v: int(v * 0.6)) if False else glow)
    base.alpha_composite(img)
    save(base, 'reticle.png')

def ring():
    size = 96 * S
    img = Image.new('RGBA', (size, size), (0, 0, 0, 0))
    d = ImageDraw.Draw(img)
    c = size / 2
    r = 40 * S
    for a0, a1 in ((20, 70), (110, 160), (200, 250), (290, 340)):
        d.arc([c - r, c - r, c + r, c + r], a0, a1, fill=(255, 255, 255, 230), width=2 * S)
    save(img, 'ring.png')

def panel(name, width, height, accent):
    w, h = width * S, height * S
    img = Image.new('RGBA', (w, h), (0, 0, 0, 0))
    d = ImageDraw.Draw(img)
    slant = 10 * S
    body = [(slant, 0), (w, 0), (w - slant, h), (0, h)]
    d.polygon(body, fill=(8, 10, 14, 185))
    d.line([(slant, 1 * S), (w - 1 * S, 1 * S)], fill=accent + (255,), width=2 * S)
    d.line([(0, h - 1 * S), (w - slant, h - 1 * S)], fill=(255, 255, 255, 60), width=1 * S)
    save(img, name)

def fill(name, width, height, color):
    w, h = width * S, height * S
    img = Image.new('RGBA', (w, h), (0, 0, 0, 0))
    d = ImageDraw.Draw(img)
    for x in range(w):
        t = x / max(w - 1, 1)
        shade = tuple(int(color[i] * (0.7 + 0.3 * t)) for i in range(3))
        d.line([(x, 0), (x, h)], fill=shade + (235,))
    for y in range(0, h, 2 * S):
        d.line([(0, y), (w, y)], fill=(0, 0, 0, 40))
    save(img, name)

def segment():
    w, h = 22 * S, 16 * S
    img = Image.new('RGBA', (w, h), (0, 0, 0, 0))
    d = ImageDraw.Draw(img)
    d.polygon([(4 * S, 0), (w, 0), (w - 4 * S, h), (0, h)], fill=(255, 255, 255, 255))
    save(img, 'segment.png')

def vignette():
    w, h = 320 * S, 180 * S
    img = Image.new('RGBA', (w, h), (0, 0, 0, 0))
    px = img.load()
    for y in range(h):
        for x in range(w):
            nx = (x / w - 0.5) * 2.0
            ny = (y / h - 0.5) * 2.0
            r = math.sqrt(nx * nx + ny * ny)
            a = max(0.0, min(1.0, (r - 0.45) / 0.75))
            px[x, y] = (255, 255, 255, int(a * a * 255))
    img = img.resize((320, 180), Image.LANCZOS)
    img.save(os.path.join(out, 'vignette.png'))
    print('vignette.png', img.size)

def hitmark():
    size = 48 * S
    img = Image.new('RGBA', (size, size), (0, 0, 0, 0))
    d = ImageDraw.Draw(img)
    c = size / 2
    for dx, dy in ((1, 1), (-1, 1), (1, -1), (-1, -1)):
        d.line([(c + dx * 6 * S, c + dy * 6 * S), (c + dx * 16 * S, c + dy * 16 * S)], fill=(255, 255, 255, 255), width=3 * S)
    save(img, 'hitmark.png')

reticle(); ring(); panel('panel_red.png', 260, 34, (235, 60, 50)); panel('panel_cyan.png', 260, 34, (60, 190, 255))
fill('fill_red.png', 240, 22, (225, 55, 45)); fill('fill_cyan.png', 240, 22, (70, 190, 255)); segment(); vignette(); hitmark()
