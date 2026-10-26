import math

# The first scene of the game: the main menu over the street at night, with the logo and the CTW
# mark. START loads the night (next_scene: the level, its lightmaps, the people): the change is
# asked once LOADING is on screen, so the picture stays there through the load and the intro fades
# in over it. CONTROLS and CREDITS open over the lab. Arrows, W/S or the mouse pick, ENTER or a
# click confirms, ESC goes back (and quits from the menu).
OPTIONS = ["START", "CONTROLS", "CREDITS", "QUIT"]
CONTROLS = [
    ["W A S D", "MOVE"],
    ["MOUSE", "AIM"],
    ["LEFT MOUSE", "FIRE  //  PISTOL WHIP UP CLOSE"],
    ["RIGHT MOUSE (HOLD)", "OVERCLOCK: SLOW TIME"],
    ["C  /  MIDDLE MOUSE", "SHOOTDODGE THE WAY YOU MOVE"],
    ["SPACE", "JUMP"],
    ["Z (HOLD)", "ZOOM OVER THE SHOULDER"],
    ["F", "FLYING KICK"],
    ["Q", "SEND THE SPIDER AT YOUR TARGET"],
    ["V", "THREAT SCAN"],
    ["E (HOLD)", "HACK  //  UPLOAD AT A TERMINAL"],
    ["R", "RELOAD"],
    ["M", "MUSIC ON / OFF"],
    ["ENTER", "ACKNOWLEDGE ORDERS  //  SKIP"],
    ["ESC", "PAUSE"],
]


def glitch_text(x, y, text, size, color, split):
    hud_text(x - split, y, text, Color(1.0, 0.15, 0.2, 0.75 * color.a), size)
    hud_text(x + split, y, text, Color(0.2, 0.85, 1.0, 0.75 * color.a), size)
    hud_text(x, y, text, color, size)


class Boot(Node):
    next_scene = "Scenes/game.scene.json"
    music_volume = 0.35
    _street = -1
    _lab = -1
    _ctw = -1
    _credits = None
    _outro = None
    _music = None
    _snd_move = None
    _snd_pick = None
    _page = 0  # 0 the menu, 1 controls, 2 credits
    _choice = 0
    _clock = 0.0
    _lab_mix = 0.0  # the lab over the street, while a page is open
    _start = -1.0  # seconds since START was picked
    _frames = 0  # frames LOADING has been on screen

    def _ready(self):
        # Back from the night: there closing the window asked first (the pause panel).
        self.get_tree().set_auto_accept_quit(True)
        mouse_set_mode(MOUSE_MODE_VISIBLE)
        self._street = hud_load_image("Assets/Textures/Menu/menu_street.png")
        self._lab = hud_load_image("Assets/Textures/Menu/menu_lab.png")
        self._ctw = hud_load_image("Assets/Textures/Logos/ctw.png")
        self._credits = self.create_child("Node", "Assets/Scripts/credits.py")
        self._credits.name = "Credits"
        self._music = self.sound("Assets/Sounds/music/theme.ogg", 0.0)
        self._music.loop = True
        self._music.play()
        self._snd_move = self.sound("Assets/Sounds/fx/beep.ogg", 0.5)
        self._snd_pick = self.sound("Assets/Sounds/bt/bt_swoosh_02.ogg", 0.8)

    def sound(self, path, volume):
        player = self.create_child("AudioStreamPlayer")
        player.stream = path
        player.bus = AUDIO_BUS_MUSIC
        player.volume_linear = volume
        return player

    def _process(self, delta):
        self._clock = self._clock + delta
        width = hud_width()
        height = hud_height()
        s = height / 720.0
        leaving = math.clamp(self._start / 0.45, 0.0, 1.0) if self._start >= 0.0 else 0.0
        self._music.volume_linear = self.music_volume * math.clamp(self._clock / 2.0, 0.0, 1.0) * (1.0 - leaving)
        self._lab_mix = move_toward(self._lab_mix, 1.0 if self._page > 0 else 0.0, delta * 3.0)
        hud_rect(0.0, 0.0, width, height, Color(0.0, 0.0, 0.0, 1.0))
        self.picture(self._street, 1.0)
        if self._lab_mix > 0.0:
            self.picture(self._lab, self._lab_mix)
        band = (self._clock * 140.0 * s) % (height + 80.0 * s) - 40.0 * s
        hud_rect(0.0, band, width, 40.0 * s, Color(0.4, 0.85, 1.0, 0.02))
        if self._page == 0:
            self.draw_menu(width, height, s, (1.0 - self._lab_mix) * (1.0 - leaving))
        elif self._page == 1:
            self.draw_controls(width, height, s, self._lab_mix)
        else:
            self.draw_credits(width, height, s, delta)
        if self._start >= 0.0:
            self.loading(delta, width, height, s)
        hud_rect(0.0, 0.0, width, height, Color(0.0, 0.0, 0.0, 1.0 - math.clamp(self._clock / 0.8, 0.0, 1.0)))

    # The picture covers the screen whatever its shape (cropped at the sides or at the top) and
    # drifts slowly inside a slight zoom.
    def picture(self, image, alpha):
        width = hud_width()
        height = hud_height()
        aspect = width / height
        hw = 0.5
        hh = 0.5
        if aspect < 16.0 / 9.0:
            hw = aspect / (16.0 / 9.0) * 0.5
        else:
            hh = (16.0 / 9.0) / aspect * 0.5
        hw = hw / 1.06
        hh = hh / 1.06
        cx = 0.5 + (0.5 - hw) * math.sin(self._clock * 0.05)
        cy = 0.5 + (0.5 - hh) * math.sin(self._clock * 0.037 + 1.3)
        hud_image(image, 0.0, 0.0, width, height, 0.0, cx - hw, cy - hh, cx + hw, cy + hh, Color(1.0, 1.0, 1.0, alpha))

    # The street's left side is dark: the logo and the options go there, over a shade.
    def draw_menu(self, width, height, s, alpha):
        if alpha <= 0.0:
            return
        for i in range(24):
            f = i / 24.0
            hud_rect(width * 0.6 * f, 0.0, width * 0.6 / 24.0 + 1.0, height, Color(0.0, 0.0, 0.0, 0.6 * (1.0 - f) * (1.0 - f) * alpha))
        x = width * 0.08
        size = height * 0.5
        self._credits.logo(x - size * 0.09, height * 0.04, size, alpha)
        # Now and then the picture glitches, as NULL//OS's screens do.
        if self._clock % 7.0 < 0.12:
            for i in range(5):
                gy = math.random(0.0, height)
                gh = math.random(2.0, 9.0) * s
                shift = math.random(-24.0, 24.0) * s
                hud_rect(shift, gy, width, gh, Color(0.3, 0.9, 1.0, 0.16 * alpha))
                hud_rect(-shift, gy + gh, width, gh * 0.5, Color(1.0, 0.15, 0.15, 0.16 * alpha))
        y = height * 0.58
        row = 46.0 * s
        if self._start < 0.0 and alpha >= 1.0:
            self.menu_input(x, y, row, s)
        for i in range(len(OPTIONS)):
            top = y + row * i
            if i == self._choice:
                pulse = 0.75 + 0.25 * math.sin(self._clock * 6.0)
                hud_rect(x - 16.0 * s, top + 4.0 * s, 330.0 * s, row - 8.0 * s, Color(0.35, 0.8, 1.0, 0.16 * alpha))
                hud_rect(x - 16.0 * s, top + 4.0 * s, 4.0 * s, row - 8.0 * s, Color(1.0, 0.2, 0.25, 0.95 * alpha))
                color = Color(1.0, 0.35, 0.35, pulse * alpha) if OPTIONS[i] == "QUIT" else Color(1.0, 1.0, 1.0, pulse * alpha)
                glitch_text(x, top + 9.0 * s, "> " + OPTIONS[i], 26.0 * s, color, 1.2 * s)
            else:
                hud_text(x, top + 9.0 * s, "  " + OPTIONS[i], Color(0.45, 0.9, 1.0, 0.6 * alpha), 26.0 * s)
        dim = Color(1.0, 1.0, 1.0, 0.45 * alpha)
        hud_text(x, height - 44.0 * s, "[ENTER] SELECT    [ESC] QUIT", dim, 13.0 * s)
        mark = 84.0 * s
        mx = width - mark - 24.0 * s
        my = height - mark - 20.0 * s
        hud_image(self._ctw, mx, my, mark, mark, 0.0, 0.0, 0.0, 1.0, 1.0, Color(1.0, 1.0, 1.0, 0.9 * alpha))
        line = "A CTW JAM GAME  //  PRISMA3D ENGINE"
        hud_text(mx - 14.0 * s - hud_text_size(line, 13.0 * s).x, my + mark * 0.5 - 8.0 * s, line, dim, 13.0 * s)

    # The mouse picks the option it moves over (a mouse at rest leaves the arrows alone).
    def menu_input(self, x, y, row, s):
        if self._outro != None:
            return
        mouse = mouse_position()
        moved = mouse_delta()
        for i in range(len(OPTIONS)):
            top = y + row * i
            if mouse.x >= x - 16.0 * s and mouse.x <= x + 314.0 * s and mouse.y >= top and mouse.y < top + row:
                if (moved.x != 0.0 or moved.y != 0.0) and i != self._choice:
                    self.select(i)
                if mouse_check_button_pressed(MOUSE_LEFT):
                    self._choice = i
                    self.pick(i)
                    return
        if keyboard_check_pressed(KEY_UP) or keyboard_check_pressed(KEY_W):
            self.select((self._choice + len(OPTIONS) - 1) % len(OPTIONS))
        elif keyboard_check_pressed(KEY_DOWN) or keyboard_check_pressed(KEY_S):
            self.select((self._choice + 1) % len(OPTIONS))
        elif keyboard_check_pressed(KEY_ENTER) or keyboard_check_pressed(KEY_KP_ENTER) or keyboard_check_pressed(KEY_SPACE):
            self.pick(self._choice)
        elif keyboard_check_pressed(KEY_ESCAPE):
            self.leave()

    def select(self, index):
        self._choice = index
        self._snd_move.play()

    def pick(self, index):
        self._snd_pick.play()
        if index == 0:
            self._start = 0.0
            mouse_set_mode(MOUSE_MODE_HIDDEN)
        elif index == 3:
            self.leave()
        else:
            self._page = index
            if index == 2:
                self._credits.restart()

    # QUIT: the MADE WITH card, then the game closes (Assets/Scripts/outro.py).
    def leave(self):
        if self._outro == None:
            self._outro = self.create_child("Node", "Assets/Scripts/outro.py")
            self._outro.name = "Outro"

    # A page closes with ESC, ENTER or a click.
    def back_input(self):
        if keyboard_check_pressed(KEY_ESCAPE) or keyboard_check_pressed(KEY_ENTER) or keyboard_check_pressed(KEY_KP_ENTER) or keyboard_check_pressed(KEY_SPACE) or mouse_check_button_pressed(MOUSE_LEFT):
            self._page = 0
            self._snd_move.play()

    def draw_controls(self, width, height, s, alpha):
        self.back_input()
        x = math.min(width * 0.5, width - 600.0 * s)
        y = height * 0.1
        self.page_title(x, y, "NULL//OS  //  ENFORCEMENT UNIT MANUAL", "CONTROLS", s, alpha)
        y = y + 100.0 * s
        for pair in CONTROLS:
            hud_text(x, y, pair[0], Color(1.0, 1.0, 1.0, 0.95 * alpha), 16.0 * s)
            hud_text(x + 230.0 * s, y, pair[1], Color(0.45, 0.9, 1.0, 0.9 * alpha), 16.0 * s)
            y = y + 27.0 * s
        self.hint(width, height, s, "[ESC] BACK", alpha)

    def draw_credits(self, width, height, s, delta):
        self.back_input()
        self._credits.roll(width * 0.7, delta, self._lab_mix)
        self.hint(width, height, s, "[ESC] BACK", self._lab_mix)

    def page_title(self, x, y, tag, title, s, alpha):
        hud_text(x, y, tag, Color(0.45, 0.9, 1.0, 0.8 * alpha), 14.0 * s)
        glitch_text(x, y + 22.0 * s, title, 42.0 * s, Color(1.0, 1.0, 1.0, alpha), 2.0 * s)
        hud_rect(x, y + 78.0 * s, 420.0 * s, 3.0 * s, Color(1.0, 0.2, 0.25, 0.95 * alpha))

    def hint(self, width, height, s, text, alpha):
        size = 14.0 * s
        hud_text(width - width * 0.06 - hud_text_size(text, size).x, height - 44.0 * s, text, Color(1.0, 1.0, 1.0, 0.6 * alpha), size)

    # START: the menu fades out over the street, then LOADING shows; the night is asked for once
    # that frame is on screen.
    def loading(self, delta, width, height, s):
        self._start = self._start + delta
        if self._start < 0.45:
            return
        text = "NULL//OS  //  LOADING"
        size = 18.0 * s
        glitch_text(width - width * 0.08 - hud_text_size(text, size).x, height * 0.9, text, size, Color(1.0, 1.0, 1.0, 0.9), 1.5 * s)
        self._frames = self._frames + 1
        if self._frames == 3:
            self.get_tree().change_scene(self.next_scene, "fade", 0.6)
