# The trailer's cards, 1280x720 with a clear background for ffmpeg to lay over the frames:
# title.png (the name and the line) and end.png (the name, the credit and the CTW logo).
# Usage: python3 tools/trailer_cards.py work_dir
import os, sys
from PIL import Image, ImageDraw, ImageFont

W, H = 1280, 720
FONTS = '/usr/share/fonts/truetype/dejavu/'
BOLD = FONTS + 'DejaVuSans-Bold.ttf'
REGULAR = FONTS + 'DejaVuSans.ttf'
work = sys.argv[1]


def card(name, lines, logo=None, dim=0):
    # dim: the black laid over the whole frame under the text (the end card over the HUD).
    picture = Image.new('RGBA', (W, H), (0, 0, 0, dim))
    draw = ImageDraw.Draw(picture)
    y = H * 0.36
    for text, size, color, font_path in lines:
        font = ImageFont.truetype(font_path, size)
        width = draw.textlength(text, font=font)
        draw.text(((W - width) / 2 + 3, y + 3), text, font=font, fill=(0, 0, 0, 200))
        draw.text(((W - width) / 2, y), text, font=font, fill=color)
        y += size * 1.35
    if logo:
        mark = Image.open(logo).convert('RGBA')
        scale = 140.0 / mark.height
        mark = mark.resize((int(mark.width * scale), 140))
        picture.alpha_composite(mark, ((W - mark.width) // 2, int(H * 0.62)))
    picture.save(os.path.join(work, name))


card('title.png', [('NULL PROTOCOL', 96, (235, 245, 255, 255), BOLD),
                   ('AI GONE WRONG. NOW GONE RIGHT.', 30, (120, 210, 255, 255), REGULAR)])
card('end.png', [('NULL PROTOCOL', 84, (235, 245, 255, 255), BOLD),
                 ('A CTW JAM GAME  //  PRISMA3D ENGINE', 24, (120, 210, 255, 255), REGULAR)],
     'Assets/Textures/Logos/ctw.png', 165)
print('cards in', work)
