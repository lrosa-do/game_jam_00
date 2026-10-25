class BloodTrail(Node3D):
    duration = 1.4
    _age = 0.0
    _drip = 0.0
    _spray = None
    _drops = None

    def _ready(self):
        self._spray = self.get_node("Spray")
        self._drops = self.get_node("Drops")

    def _process(self, delta):
        if self._drops == None:
            self._ready()
        if self._age == 0.0:
            self._spray.emit_burst(10)
            self._drops.emit_burst(24)
        self._age = self._age + delta
        if self._age > self.duration:
            return
        self._drip = self._drip + delta * 40.0 * (1.0 - self._age / self.duration)
        if self._drip >= 1.0:
            count = int(self._drip)
            self._drip = self._drip - count
            self._drops.emit_burst(count)
