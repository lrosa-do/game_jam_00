class FxLife(Node3D):
    life = 3.0
    stop_at = 0.0
    _age = 0.0
    _stopped = False

    def _process(self, delta):
        self._age = self._age + delta
        if not self._stopped and self.stop_at > 0.0 and self._age > self.stop_at:
            self._stopped = True
            for child in self.get_children():
                if child.is_class("ParticleSystem3D"):
                    child.emission_rate = 0.0
                elif child.is_class("AudioStreamPlayer3D"):
                    child.stop()
                elif child.is_class("Light3D"):
                    child.light_energy = 0.0
        if self._age > self.life:
            self.queue_free()
