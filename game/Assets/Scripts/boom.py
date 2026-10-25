import math

class Boom(Node3D):
    peak = 12.0
    life = 3.0
    _age = 0.0
    _light = None

    def _ready(self):
        self._light = self.get_node("Flash")
        self.get_node("Sound").play()

    def _process(self, delta):
        self._age = self._age + delta
        if self._light != None:
            self._light.light_energy = self.peak * math.max(1.0 - self._age / 0.35, 0.0)
        if self._age > self.life:
            self.queue_free()
