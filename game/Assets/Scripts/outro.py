import math

# Leaving the game: MADE WITH and the logos of what it is built with, over black for a few seconds
# (a key or a click skips them), then the game closes. Made by boot.py and exit_menu.py.
SECONDS = 3.0

class Outro(Node):
    _t = 0.0
    _credits = None

    def _ready(self):
        # Runs over a paused game too.
        self.process_mode = 3
        self._credits = self.create_child("Node", "Assets/Scripts/credits.py")
        self._credits.name = "Credits"

    def _process(self, delta):
        self._t = self._t + delta
        skip = self._t > 0.4 and (keyboard_check_pressed(KEY_ANY) or mouse_check_button_pressed(MOUSE_LEFT))
        if self._t >= SECONDS or skip:
            self.get_tree().quit()
            return
        width = hud_width()
        height = hud_height()
        alpha = math.clamp(self._t / 0.4, 0.0, 1.0) * math.clamp((SECONDS - self._t) / 0.4, 0.0, 1.0)
        hud_rect(0.0, 0.0, width, height, Color(0.0, 0.0, 0.0, math.clamp(self._t / 0.25, 0.0, 1.0)))
        self._credits.made_with(width * 0.5, height * 0.5, height / 720.0, alpha)
