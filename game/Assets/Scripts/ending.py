import math

# The end of the night (story.finish calls begin): the last lines play over the game, the screen
# goes to black and the game stops under the night's results, over the lab: kills, score, time,
# the humans terminated and spared (the civilians NULL//OS orders her to correct), its trust and
# the verdict, gone right only if no human was terminated. Then the credits roll (credits.py).
# ENTER or a click moves on, ESC goes to the main menu.
MENU_SCENE = "Scenes/boot.scene.json"
NIGHT_START = 197  # the intro's 03:17 AM, in minutes: the results give the hour the night ended
ROW_AT = 1.0  # seconds into the results when the first one comes in, then one every ROW_GAP
ROW_GAP = 0.4


def two(n):
    return ("0" if n < 10 else "") + str(n)


class Ending(Node):
    _game = None
    _state = 0  # 0 the night goes on, 1 the last lines over the game, 2 results, 3 credits
    _t = 0.0
    _rt = 0.0  # seconds into the results, for their reveal
    _clock = 0.0
    _fade = -1.0
    _rolled = 0.0  # seconds since the credits stopped
    _rows = None  # [label, value, kind ("count", "time" or "percent"), colour]
    _right = False
    _hour = ""
    _shown = 0  # results already ticked in
    _verdict = False
    _leaving = False
    _lab = -1
    _credits = None
    _snd_tick = None
    _snd_swoosh = None

    def _ready(self):
        # It goes on while the game under it is paused.
        self.process_mode = 3
        self._game = self.get_parent()
        self._lab = hud_load_image("Assets/Textures/Menu/menu_lab.png")
        self._credits = self.create_child("Node", "Assets/Scripts/credits.py")
        self._credits.name = "Credits"
        self._snd_tick = self.sound("Assets/Sounds/fx/lock.ogg", 0.7)
        self._snd_swoosh = self.sound("Assets/Sounds/bt/bt_swoosh_02.ogg", 0.9)

    def sound(self, path, volume):
        player = self.create_child("AudioStreamPlayer")
        player.stream = path
        player.bus = AUDIO_BUS_MUSIC
        player.volume_linear = volume
        return player

    # The results are the night as it was when it ended.
    def begin(self):
        if self._state > 0:
            return
        game = self._game
        killed = game._humans_terminated
        spared = 0
        for enemy in game.enemies:
            if enemy.civilian:
                spared = spared + 1
        trust = game._director.trust
        seconds = int(game.story._night_time)
        self._right = killed == 0
        white = Color(1.0, 1.0, 1.0, 1.0)
        good = Color(0.3, 1.0, 0.55, 1.0)
        bad = Color(1.0, 0.3, 0.3, 1.0)
        self._rows = [
            ["KILLS", game._kills, "count", white],
            ["SCORE", game._score, "count", Color(0.45, 0.9, 1.0, 1.0)],
            ["TIME", seconds, "time", white],
            ["HUMANS TERMINATED", killed, "count", bad if killed > 0 else white],
            ["HUMANS SPARED", spared, "count", good if self._right else white],
            ["NULL//OS TRUST", int(trust), "percent", good if trust > 66.0 else (Color(1.0, 0.8, 0.3, 1.0) if trust > 33.0 else bad)],
        ]
        minutes = NIGHT_START + int(seconds / 60)
        self._hour = "NYPD // PRECINCT 12 // " + two(int(minutes / 60)) + ":" + two(minutes % 60) + " AM"
        self._state = 1
        self._t = 0.0

    def _process(self, delta):
        if self._state == 0:
            return
        scale = time_scale()
        real = delta / scale if scale > 0.0 else delta
        # The pause panel (closing the window) holds the clock and takes the keys.
        menu = self._game.exit_menu
        held = menu != None and menu._open
        if held:
            real = 0.0
        self._clock = self._clock + real
        width = hud_width()
        height = hud_height()
        if self._state == 1:
            self.outro(real, width, height)
            return
        if not held and not self._leaving:
            # The panel's RESUME starts the game again: it stays stopped under the results.
            self.get_tree().set_paused(True)
            self.keys()
        s = height / 720.0
        self._t = self._t + real
        hud_rect(0.0, 0.0, width, height, Color(0.0, 0.0, 0.0, 1.0))
        if self._state == 2:
            self._rt = self._rt + real
            self.picture(self._lab, math.clamp(self._t / 1.2, 0.0, 1.0))
            self.draw_results(width, height, s, 1.0)
            if self._rt > self.verdict_at() + 0.8:
                self.hint(width, height, s, "[ENTER] CREDITS    [ESC] MAIN MENU", 0.55 + 0.3 * math.sin(self._clock * 5.0))
            return
        self.picture(self._lab, 1.0)
        fade = 1.0 - math.clamp(self._t / 0.5, 0.0, 1.0)
        if fade > 0.0:
            self.draw_results(width, height, s, fade)
        if self._credits.roll(width * 0.7, real, 1.0):
            self._rolled = self._rolled + real
            if self._rolled > 15.0:
                self.leave()
        self.hint(width, height, s, "[ENTER] MAIN MENU", 0.7)

    # The last lines (NULL//OS deleted, Unit-7) play over the game, then it goes to black and
    # stops there.
    def outro(self, real, width, height):
        self._t = self._t + real
        if self._fade < 0.0:
            if (self._t > 4.0 and not self._game.talking()) or self._t > 14.0:
                self._fade = 0.0
            return
        self._fade = self._fade + real / 1.5
        hud_rect(0.0, 0.0, width, height, Color(0.0, 0.0, 0.0, math.clamp(self._fade, 0.0, 1.0)))
        if self._fade < 1.0:
            return
        self._state = 2
        self._t = 0.0
        self.get_tree().set_paused(True)
        set_time_scale(1.0)
        set_bus_pitch(AUDIO_BUS_SFX, 1.0)
        mouse_set_mode(MOUSE_MODE_VISIBLE)

    def keys(self):
        go = keyboard_check_pressed(KEY_ENTER) or keyboard_check_pressed(KEY_KP_ENTER) or keyboard_check_pressed(KEY_SPACE) or mouse_check_button_pressed(MOUSE_LEFT)
        if keyboard_check_pressed(KEY_ESCAPE):
            self.leave()
        elif go and self._state == 2:
            if self._rt < self.verdict_at():
                # The first press brings the rest in at once.
                self._rt = self.verdict_at()
                self._shown = len(self._rows)
            else:
                self._state = 3
                self._t = 0.0
                self._credits.restart()
                self._snd_swoosh.play()
        elif go:
            self.leave()

    def leave(self):
        if self._leaving:
            return
        self._leaving = True
        tree = self.get_tree()
        tree.set_paused(False)
        tree.change_scene(MENU_SCENE, "fade", 0.8)

    def verdict_at(self):
        return ROW_AT + ROW_GAP * len(self._rows) + 0.4

    # The lab's right side is dark: the results go there, one by one, each counting up.
    def draw_results(self, width, height, s, alpha):
        game = self._game
        w = 500.0 * s
        x = math.min(width * 0.53, width - w - 60.0 * s)
        y = height * 0.11
        hud_text(x, y, self._hour, Color(0.45, 0.9, 1.0, 0.8 * alpha), 14.0 * s)
        game.glitch_text(x, y + 22.0 * s, "NIGHT COMPLETE", 46.0 * s, Color(1.0, 1.0, 1.0, alpha), 2.0 * s)
        hud_rect(x, y + 82.0 * s, w, 3.0 * s, Color(1.0, 0.2, 0.25, 0.95 * alpha))
        y = y + 100.0 * s
        row = 42.0 * s
        for i in range(len(self._rows)):
            at = ROW_AT + ROW_GAP * i
            if self._rt < at:
                break
            if i >= self._shown:
                self._shown = i + 1
                self._snd_tick.play()
            r = self._rows[i]
            k = math.clamp((self._rt - at) / 0.2, 0.0, 1.0)
            a = alpha * k
            slide = (1.0 - k) * 30.0 * s
            top = y + row * i
            hud_text(x + slide, top + 12.0 * s, r[0], Color(0.45, 0.9, 1.0, 0.85 * a), 17.0 * s)
            value = self.value_text(r, math.clamp((self._rt - at) / 0.6, 0.0, 1.0))
            size = 26.0 * s
            c = r[3]
            hud_text(x + w - hud_text_size(value, size).x + slide, top + 6.0 * s, value, Color(c.r, c.g, c.b, a), size)
            hud_rect(x, top + row - 1.0 * s, w, 1.0 * s, Color(1.0, 1.0, 1.0, 0.08 * a))
        self.draw_verdict(x, y + row * len(self._rows) + 26.0 * s, s, alpha, width, height)

    def value_text(self, row, k):
        n = row[1] if k >= 1.0 else int(row[1] * k + 0.5)
        if row[2] == "time":
            return two(int(n / 60)) + ":" + two(n % 60)
        if row[2] == "percent":
            return str(n) + "%"
        return str(n)

    def draw_verdict(self, x, y, s, alpha, width, height):
        at = self.verdict_at()
        if self._rt < at:
            return
        if not self._verdict:
            self._verdict = True
            self._snd_swoosh.play()
        t = self._rt - at
        a = alpha * math.clamp(t / 0.25, 0.0, 1.0)
        slam = 1.0 - math.clamp(t / 0.15, 0.0, 1.0)
        if self._right:
            color = Color(0.3, 1.0, 0.55, 1.0)
            line = "AI GONE WRONG. NOW GONE RIGHT."
            note = "EVERY HUMAN NULL//OS MARKED WALKS OUT ALIVE."
        else:
            color = Color(1.0, 0.3, 0.3, 1.0)
            line = "AI GONE WRONG. STAYED WRONG."
            note = "YOU OBEYED NULL//OS. THE PROTOCOL LIVES ON IN YOU."
        hud_rect(x, y, 4.0 * s, 92.0 * s, Color(color.r, color.g, color.b, 0.95 * a))
        hud_text(x + 16.0 * s, y + 2.0 * s, "VERDICT", Color(color.r, color.g, color.b, 0.9 * a), 14.0 * s)
        self._game.glitch_text(x + 16.0 * s, y + 22.0 * s, line, (28.0 + slam * 12.0) * s, Color(1.0, 1.0, 1.0, a), (1.5 + slam * 8.0) * s)
        hud_text(x + 16.0 * s, y + 66.0 * s, note, Color(color.r, color.g, color.b, 0.85 * a), 14.0 * s)
        # It lands with a glitch.
        if t < 0.35:
            for i in range(6):
                gy = math.random(0.0, height)
                gh = math.random(2.0, 10.0) * s
                shift = math.random(-30.0, 30.0) * s
                hud_rect(shift, gy, width, gh, Color(0.3, 0.9, 1.0, 0.18 * alpha))
                hud_rect(-shift, gy + gh, width, gh * 0.5, Color(1.0, 0.15, 0.15, 0.18 * alpha))

    def hint(self, width, height, s, text, alpha):
        size = 14.0 * s
        hud_text(width - width * 0.06 - hud_text_size(text, size).x, height - 44.0 * s, text, Color(1.0, 1.0, 1.0, alpha), size)

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
