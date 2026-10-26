import math

# The credits, rolled at the end of the night (ending.py) and from the main menu (boot.py): a role
# and the names under it. Edit the list to change them.
CREDITS = [
    ["GAME & ENGINE", ["LUIS SANTOS"]],
    ["GAME", ["GONCALO RIBEIRO", "ALEXANDRE FIGUEIREDO"]],
    ["MENUS & CREDITS", ["ALEXANDRE FIGUEIREDO"]],
    ["GAME TESTERS", ["GONCALO FERNANDES", "RONALD ABRANTES"]],
    ["MUSIC", ["MAX PAYNE 2: THE FALL OF MAX PAYNE"]],
    ["VOICES", ["KOKORO TTS"]],
    ["CHARACTERS", ["MESHY"]],
    ["ANIMATION", ["MIXAMO", "MAX PAYNE 2"]],
    ["TOOLS", ["DEFORM", "COCOSHAPE"]],
]
# Where the logo is in Assets/Textures/Menu/logo.png (1672x941, the rest is clear), as a fraction
# of the picture, and its width over its height.
LOGO_UV = [0.2344, 0.0, 0.7632, 0.9330]
LOGO_ASPECT = 1.0068
SPEED = 50.0  # pixels a second at 720 lines
# What the game is built with, rolled after the names: the logos in Assets/Textures/Logos with
# their width over their height.
BUILT_WITH = [["Assets/Textures/Logos/prisma3d.png", 869.0 / 231.0], ["Assets/Textures/Logos/prisma.png", 724.0 / 214.0], ["Assets/Textures/Logos/kinetix.png", 774.0 / 187.0]]
BUILT_HEIGHT = 52.0  # each logo, pixels at 720 lines

class Credits(Node):
    _logo = -1
    _ctw = -1
    _built = None
    _t = 0.0
    _height = 720.0
    _s = 1.0

    def _ready(self):
        self._logo = hud_load_image("Assets/Textures/Menu/logo.png")
        self._ctw = hud_load_image("Assets/Textures/Logos/ctw.png")
        self._built = []
        for entry in BUILT_WITH:
            self._built.append(hud_load_image(entry[0]))

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
        top = top + self.built_height(s)
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
        self.built(cx, ly, s, alpha)
        ly = y + last
        mark = 110.0 * s
        hud_image(self._ctw, cx - mark * 0.5, ly, mark, mark, 0.0, 0.0, 0.0, 1.0, 1.0, Color(1.0, 1.0, 1.0, alpha * self.edge(ly + mark * 0.5)))
        self.line(cx, ly + mark + 14.0 * s, "THANK YOU FOR PLAYING", 24.0 * s, Color(1.0, 1.0, 1.0, alpha))
        return y <= stop

    def built_height(self, s):
        return 24.0 * s + len(BUILT_WITH) * (BUILT_HEIGHT + 26.0) * s + 36.0 * s

    # BUILT WITH and the logos of the engine, the renderer and the physics, one under the other.
    def built(self, cx, y, s, alpha):
        self.line(cx, y, "BUILT WITH", 14.0 * s, Color(1.0, 0.3, 0.3, 0.9 * alpha))
        y = y + 30.0 * s
        h = BUILT_HEIGHT * s
        for i in range(len(BUILT_WITH)):
            w = h * BUILT_WITH[i][1]
            hud_image(self._built[i], cx - w * 0.5, y, w, h, 0.0, 0.0, 0.0, 1.0, 1.0, Color(1.0, 1.0, 1.0, alpha * self.edge(y + h * 0.5)))
            y = y + h + 26.0 * s

    # The card shown when leaving the game (outro.py): MADE WITH and the logos, centred on (cx, cy).
    def made_with(self, cx, cy, s, alpha):
        h = 56.0 * s
        gap = 22.0 * s
        y = cy - (34.0 * s + len(BUILT_WITH) * (h + gap) - gap) * 0.5
        size = 16.0 * s
        hud_text(cx - hud_text_size("MADE WITH", size).x * 0.5, y, "MADE WITH", Color(1.0, 0.3, 0.3, 0.9 * alpha), size)
        y = y + 34.0 * s
        for i in range(len(BUILT_WITH)):
            w = h * BUILT_WITH[i][1]
            hud_image(self._built[i], cx - w * 0.5, y, w, h, 0.0, 0.0, 0.0, 1.0, 1.0, Color(1.0, 1.0, 1.0, alpha))
            y = y + h + gap

    # Near the top or the bottom of the screen the roll fades out.
    def edge(self, y):
        band = 90.0 * self._s
        return math.clamp(y / band, 0.0, 1.0) * math.clamp((self._height - y) / band, 0.0, 1.0)

    def line(self, cx, y, text, size, color):
        a = color.a * self.edge(y + size * 0.5)
        if a <= 0.0:
            return
        hud_text(cx - hud_text_size(text, size).x * 0.5, y, text, Color(color.r, color.g, color.b, a), size)
