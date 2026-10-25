import math

class BloodPool(Decal3D):
    final_size = 1.3
    grow = 3.0
    lifetime = 40.0
    _age = 0.0

    def _process(self, delta):
        self._age = self._age + delta
        t = math.clamp(self._age / self.grow, 0.0, 1.0)
        s = self.final_size * (0.25 + 0.75 * (1.0 - (1.0 - t) * (1.0 - t)))
        self.size = Vec3(s, 0.4, s)
        if self._age > self.lifetime - 5.0:
            m = self.modulate
            self.modulate = Color(m.r, m.g, m.b, math.clamp((self.lifetime - self._age) / 5.0, 0.0, 1.0))
        if self._age > self.lifetime:
            self.queue_free()
