import math

class FadingDecal(Decal3D):
    lifetime = 30.0
    fade = 5.0
    _age = 0.0

    def _process(self, delta):
        self._age = self._age + delta
        if self._age > self.lifetime - self.fade:
            alpha = math.clamp((self.lifetime - self._age) / self.fade, 0.0, 1.0)
            m = self.modulate
            self.modulate = Color(m.r, m.g, m.b, alpha)
        if self._age > self.lifetime:
            self.queue_free()
