import math

class MuzzleFlash(Node3D):
    peak = 4.0
    _light = 0.0
    _side_a = None
    _side_b = None
    _star = None
    _glow = None
    _smoke = None
    _lamp = None

    def _ready(self):
        self.cache_nodes()

    def cache_nodes(self):
        self._side_a = self.get_node("SideA")
        self._side_b = self.get_node("SideB")
        self._star = self.get_node("Star")
        self._glow = self.get_node("Glow")
        self._smoke = self.get_node("Smoke")
        self._lamp = self.get_node("Light")

    def fire(self):
        if self._lamp == None:
            self.cache_nodes()
        if math.random(0.0, 1.0) < 0.5:
            self._side_a.emit_burst(1)
        else:
            self._side_b.emit_burst(1)
        self._star.emit_burst(1)
        self._glow.emit_burst(1)
        self._smoke.emit_burst(2)
        self._light = 1.0

    def _process(self, delta):
        if self._light <= 0.0 or self._lamp == None:
            return
        self._light = math.max(self._light - delta * 18.0, 0.0)
        self._lamp.light_energy = self._light * self.peak
