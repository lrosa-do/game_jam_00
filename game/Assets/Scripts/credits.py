import math

# The credits, rolled at the end of the night (ending.py) and from the main menu (boot.py): a role
# and the names under it. Edit the list to change them.
CREDITS = [
    ["GAME & ENGINE", ["LUIS SANTOS"]],
    ["GAME", ["GONCALO RIBEIRO"]],
    ["MENUS & CREDITS", ["ALEXANDRE FIGUEIREDO"]],
    ["GAME TESTERS", ["GONCALO FERNANDES", "RONALD ABRANTES"]],
    ["ENGINE", ["PRISMA3D"]],
    ["MUSIC", ["MAX PAYNE 2: THE FALL OF MAX PAYNE"]],
    ["VOICES", ["KOKORO TTS"]],
    ["CHARACTERS", ["MESHY"]],
    ["ANIMATION", ["MIXAMO", "MAX PAYNE 2"]],
    ["TOOLS", ["BLENDER"]],
]
# Where the logo is in Assets/Textures/Menu/logo.png (1672x941, the rest is clear), as a fraction
# of the picture, and its width over its height.
LOGO_UV = [0.2344, 0.0, 0.7632, 0.9330]
LOGO_ASPECT = 1.0068
SPEED = 50.0  # pixels a second at 720 lines

class Credits(Node):
    _logo = -1
    _ctw = -1
    _t = 0.0
    _height = 720.0
    _s = 1.0

    def _ready(self):
        self._logo = hud_load_image("Assets/Textures/Menu/logo.png")
        self._ctw = hud_load_image("Assets/Textures/Logos/ctw.png")

    def restart(self):
        self._t = 0.0

    # The NULL PROTOCOL logo, its top left corner at (x, y), height pixels tall.
    def logo(self, x, y, height, alpha):
        hud_image(self._logo, x, y, height * LOGO_ASPECT, height, 0.0, LOGO_UV[0], LOGO_UV[1], LOGO_UV[2], LOGO_UV[3], Color(1.0, 1.0, 1.0, alpha))

    # Rolls the credits up the screen round cx by the real seconds given, faded at the top and
    # bottom edges; the last card (the CTW mark) stops in the middle. True once it has.
    def roll(self, cx, real, alpha):
        self._t = self._t + real
        height = hud_height()
        s = height / 720.0
        self._height = height
        self._s = s
        top = 250.0 * s + 100.0 * s
        for entry in CREDITS:
            top = top + 24.0 * s + 34.0 * s * len(entry[1]) + 36.0 * s
        # The CTW mark comes in alone: the names have left the screen when it stops.
        last = top + 300.0 * s
        card = 156.0 * s
        stop = height * 0.5 - last - card * 0.5
        y = math.max(height - SPEED * s * self._t, stop)
        size = 250.0 * s
        self.logo(cx - size * LOGO_ASPECT * 0.5, y, size, alpha * self.edge(y + size * 0.5))
        self.line(cx, y + size + 16.0 * s, "A CTW JAM GAME  //  AI GONE WRONG", 15.0 * s, Color(0.45, 0.9, 1.0, 0.85 * alpha))
        ly = y + size + 100.0 * s
        for entry in CREDITS:
            self.line(cx, ly, entry[0], 14.0 * s, Color(1.0, 0.3, 0.3, 0.9 * alpha))
            ly = ly + 24.0 * s
            for name in entry[1]:
                self.line(cx, ly, name, 24.0 * s, Color(1.0, 1.0, 1.0, alpha))
                ly = ly + 34.0 * s
            ly = ly + 36.0 * s
        ly = y + last
        mark = 110.0 * s
        hud_image(self._ctw, cx - mark * 0.5, ly, mark, mark, 0.0, 0.0, 0.0, 1.0, 1.0, Color(1.0, 1.0, 1.0, alpha * self.edge(ly + mark * 0.5)))
        self.line(cx, ly + mark + 14.0 * s, "THANK YOU FOR PLAYING", 24.0 * s, Color(1.0, 1.0, 1.0, alpha))
        return y <= stop

    # Near the top or the bottom of the screen the roll fades out.
    def edge(self, y):
        band = 90.0 * self._s
        return math.clamp(y / band, 0.0, 1.0) * math.clamp((self._height - y) / band, 0.0, 1.0)

    def line(self, cx, y, text, size, color):
        a = color.a * self.edge(y + size * 0.5)
        if a <= 0.0:
            return
        hud_text(cx - hud_text_size(text, size).x * 0.5, y, text, Color(color.r, color.g, color.b, a), size)
