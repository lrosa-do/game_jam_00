import math

class AnimTest(Node3D):
    _anim = None
    _names = None
    _log = None
    _index = 0

    def _ready(self):
        self._names = []
        self._log = []
        anim = self.get_node("Player/Armature/Skeleton3D/AnimationPlayer")
        if anim == None:
            self._log.append("no AnimationPlayer at Player/Armature/Skeleton3D")
            return
        self._anim = anim
        self._names = anim.get_animation_list()
        for name in self._names:
            anim.set_animation_loop(name, True)
        self.play_current()

    def play_current(self):
        if self._anim == None or len(self._names) == 0:
            return
        self._anim.play(self._names[self._index], 0.2)

    def _process(self, delta):
        count = len(self._names)
        if count > 0:
            if keyboard_check_pressed(KEY_RIGHT):
                self._index = (self._index + 1) % count
                self.play_current()
            if keyboard_check_pressed(KEY_LEFT):
                self._index = (self._index + count - 1) % count
                self.play_current()
            if keyboard_check_pressed(KEY_SPACE):
                self.play_current()
        self.draw_hud()

    def draw_hud(self):
        white = Color(1.0, 1.0, 1.0, 0.9)
        dim = Color(1.0, 1.0, 1.0, 0.6)
        y = 12.0
        if self._anim != None and len(self._names) > 0:
            hud_text(12.0, y, str(self._index + 1) + " / " + str(len(self._names)) + "   " + self._names[self._index], white, 24.0)
            y = y + 30.0
            position = int(self._anim.get_current_animation_position() * 10.0) / 10.0
            length = int(self._anim.get_current_animation_length() * 10.0) / 10.0
            hud_text(12.0, y, str(position) + " / " + str(length) + " s", dim, 18.0)
            y = y + 26.0
        hud_text(12.0, y, "LEFT / RIGHT: clip   SPACE: replay   mouse: orbit   ESC: free mouse", dim, 16.0)
        y = hud_height() - 20.0 - 18.0 * len(self._log)
        for line in self._log:
            hud_text(12.0, y, line, dim, 16.0)
            y = y + 18.0
