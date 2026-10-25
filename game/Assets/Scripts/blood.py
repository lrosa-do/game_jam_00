class Blood(Node3D):
    _age = 0.0

    def burst(self, direction, strength):
        self.get_node("Spray").emission_direction = direction
        self.get_node("Drops").emission_direction = direction
        self.get_node("Spray").emit_burst(int(6.0 * strength))
        self.get_node("Drops").emit_burst(int(14.0 * strength))

    def _process(self, delta):
        self._age = self._age + delta
        if self._age > 1.5:
            self.queue_free()
