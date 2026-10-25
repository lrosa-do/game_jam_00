# The quality presets for a scene without the Game node (the map view, the arena): F11 cycles
# them, F10 the fit of the 3D picture to the screen; the choice is shown for two seconds.
import math

QUALITY = [["LOW", 720.0, 0, 1], ["MEDIUM", 1080.0, 1, 1], ["HIGH", 1440.0, 2, 2], ["ULTRA", 2160.0, 3, 2]]
FITS = ["STRETCH", "KEEP", "CROP", "EXPAND"]

class RenderPresets(Node):
    quality = 1
    fit = 3
    _applied = ""
    _message = ""
    _timer = 0.0

    def _process(self, delta):
        if keyboard_check_pressed(KEY_F11):
            self.quality = (self.quality + 1) % len(QUALITY)
            q = QUALITY[self.quality]
            self._message = "RENDER // " + q[0] + "  " + str(int(math.min(q[1], hud_height()))) + "p  MSAA " + str(q[2]) + "  AA " + str(q[3])
            self._timer = 2.0
        if keyboard_check_pressed(KEY_F10):
            self.fit = (self.fit + 1) % len(FITS)
            self._message = "RENDER // FIT " + FITS[self.fit]
            self._timer = 2.0
        self.apply()
        if self._timer > 0.0:
            self._timer = self._timer - delta
            hud_text(12.0, hud_height() - 58.0, self._message, Color(0.4, 0.8, 1.0, 1.0), 20.0)

    def apply(self):
        h = hud_height()
        if h <= 0.0:
            return
        q = QUALITY[int(math.clamp(self.quality, 0, len(QUALITY) - 1))]
        lines = math.min(q[1], h)
        key = str(lines) + "/" + str(self.fit) + "/" + q[0]
        if key == self._applied:
            return
        self._applied = key
        root = self.get_tree().get_root()
        if root != None:
            root.scaling_3d_size = Vec2(math.floor(lines * 16.0 / 9.0 + 0.5), lines)
            root.scaling_3d_aspect = self.fit
            root.msaa_3d = q[2]
            root.screen_space_aa = q[3]
