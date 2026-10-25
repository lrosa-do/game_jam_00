# A bare arena to feel the bullet time: the player, props to throw around, pillars for cover.
# No Game node here, so no cops, no story, no HUD of the game: just her and the clock.

class BulletArena(Node3D):
    def _ready(self):
        self.add_to_group("arena")

    # The player's own HUD lists the controls; here only the clock, bottom right.
    def _process(self, delta):
        dim = Color(0.75, 0.8, 0.9, 0.8)
        label = "time scale " + str(math.floor(time_scale() * 100.0 + 0.5) / 100.0)
        hud_text(hud_width() - hud_text_size(label, 16.0).x - 12.0, hud_height() - 30.0, label, dim, 16.0)
