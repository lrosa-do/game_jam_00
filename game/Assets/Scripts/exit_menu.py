import math

# Closing the window or ESC opens it: the game pauses under a panel with RESUME, BACK TO MENU (the
# main menu, MENU_SCENE) and EXIT GAME. Arrows or the mouse pick, ENTER or a click confirms, ESC
# resumes; closing the window again while it is open quits.
OPTIONS = ["RESUME", "BACK TO MENU", "EXIT GAME"]
MENU_SCENE = "Scenes/boot.scene.json"

class ExitMenu(Node):
    _open = False
    _choice = 0
    _mouse_mode = 0
    _clock = 0.0
    _fresh = False
    _outro = None

    def _ready(self):
        # Runs while the tree is paused, the rest of the game does not.
        self.process_mode = 3
        self.get_tree().set_auto_accept_quit(False)

    def _close_requested(self):
        if self._open:
            self.get_tree().quit()
            return
        self.open()

    def open(self):
        if self._open:
            return
        self._open = True
        self._fresh = True
        self._choice = 0
        self._mouse_mode = mouse_get_mode()
        mouse_set_mode(MOUSE_MODE_VISIBLE)
        self.get_tree().set_paused(True)

    def close(self):
        self._open = False
        self.get_tree().set_paused(False)
        mouse_set_mode(self._mouse_mode)

    def pick(self, index):
        if index == 0:
            self.close()
        elif index == 1:
            self.close()
            self.get_tree().change_scene(MENU_SCENE, "fade", 0.6)
        elif self._outro == None:
            # EXIT GAME: the MADE WITH card, then the game closes (Assets/Scripts/outro.py).
            self._outro = self.create_child("Node", "Assets/Scripts/outro.py")
            self._outro.name = "Outro"

    def _process(self, delta):
        if not self._open or self._outro != None:
            return
        self._clock = self._clock + delta
        # The ESC that opened it is still down this frame.
        fresh = self._fresh
        self._fresh = False
        if not fresh and keyboard_check_pressed(KEY_ESCAPE):
            self.close()
            return
        if keyboard_check_pressed(KEY_UP) or keyboard_check_pressed(KEY_W):
            self._choice = (self._choice + len(OPTIONS) - 1) % len(OPTIONS)
        if keyboard_check_pressed(KEY_DOWN) or keyboard_check_pressed(KEY_S):
            self._choice = (self._choice + 1) % len(OPTIONS)
        width = hud_width()
        height = hud_height()
        s = height / 720.0
        w = 420.0 * s
        row = 46.0 * s
        h = 92.0 * s + row * len(OPTIONS)
        x = (width - w) * 0.5
        y = (height - h) * 0.5
        mouse = mouse_position()
        clicked = mouse_check_button_pressed(MOUSE_LEFT)
        hud_rect(0.0, 0.0, width, height, Color(0.0, 0.02, 0.05, 0.6))
        hud_rect(x, y, w, h, Color(0.02, 0.05, 0.1, 0.92))
        hud_rect_outline(x, y, w, h, Color(0.35, 0.8, 1.0, 0.9), 2.0)
        hud_rect(x, y, w, 4.0 * s, Color(1.0, 0.2, 0.25, 0.95))
        hud_text(x + 18.0 * s, y + 14.0 * s, "NULL//OS", Color(0.4, 0.85, 1.0, 0.6), 13.0 * s)
        title = "TERMINATE SESSION?"
        hud_text(x + 18.0 * s, y + 34.0 * s, title, Color(1.0, 1.0, 1.0, 1.0), 24.0 * s)
        oy = y + 80.0 * s
        for i in range(len(OPTIONS)):
            over = mouse.x >= x and mouse.x <= x + w and mouse.y >= oy and mouse.y < oy + row
            if over:
                self._choice = i
                if clicked:
                    self.pick(i)
                    return
            text = OPTIONS[i]
            if i == self._choice:
                pulse = 0.75 + 0.25 * math.sin(self._clock * 6.0)
                hud_rect(x + 10.0 * s, oy + 4.0 * s, w - 20.0 * s, row - 8.0 * s, Color(0.35, 0.8, 1.0, 0.18))
                color = Color(1.0, 0.35, 0.35, pulse) if i == 2 else Color(1.0, 1.0, 1.0, pulse)
                text = "> " + text
            else:
                color = Color(0.45, 0.9, 1.0, 0.6)
                text = "  " + text
            hud_text(x + 24.0 * s, oy + 12.0 * s, text, color, 20.0 * s)
            oy = oy + row
        if not fresh and keyboard_check_pressed(KEY_ENTER):
            self.pick(self._choice)
