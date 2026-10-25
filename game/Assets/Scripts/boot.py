import math

# The first scene of the game: the station picture with LOADING on it while the night
# (Scenes/game.scene.json: the level, its lightmaps, the people) loads. The change is asked once
# this picture is on screen, so it stays there through the load, and the intro (on the same
# picture) fades in over it.
class Boot(Node):
    next_scene = "Scenes/game.scene.json"
    _img = -1
    _frames = 0

    def _ready(self):
        self._img = hud_load_image("Assets/Textures/Menu/back_0.png")
        mouse_set_mode(MOUSE_MODE_HIDDEN)

    def _process(self, delta):
        width = hud_width()
        height = hud_height()
        s = height / 720.0
        # Cover the screen whatever its shape, as the intro does.
        u = 0.0
        v = 0.0
        aspect = width / height
        if aspect < 16.0 / 9.0:
            u = (1.0 - aspect / (16.0 / 9.0)) * 0.5
        else:
            v = (1.0 - (16.0 / 9.0) / aspect) * 0.5
        hud_rect(0.0, 0.0, width, height, Color(0.0, 0.0, 0.0, 1.0))
        hud_image(self._img, 0.0, 0.0, width, height, 0.0, u, v, 1.0 - u, 1.0 - v, Color(1.0, 1.0, 1.0, 1.0))
        text = "NULL//OS  //  LOADING"
        size = 18.0 * s
        x = width - width * 0.08 - hud_text_size(text, size).x
        y = height * 0.9
        hud_text(x - 1.5 * s, y, text, Color(1.0, 0.2, 0.25, 0.6), size)
        hud_text(x + 1.5 * s, y, text, Color(0.3, 0.85, 1.0, 0.6), size)
        hud_text(x, y, text, Color(1.0, 1.0, 1.0, 0.9), size)
        self._frames = self._frames + 1
        if self._frames == 3:
            self.get_tree().change_scene(self.next_scene, "fade", 0.6)
