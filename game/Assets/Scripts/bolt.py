import math

# The floater's shot: a red bolt with its own light and a ribbon trail, flying straight at speed.
# Each frame it sweeps a ray over the step it moves: her, and it hurts; anything else stops it.
# Then it goes out (the core hides, the light dies, the trail fades) and frees itself.
class Bolt(Node3D):
    speed = 22.0
    damage = 6.0
    life = 2.0
    direction = None
    source = None
    _age = 0.0
    _out = -1.0
    _player = None
    _core = None
    _light = None
    _trail = None

    def _ready(self):
        self._player = self.get_tree().get_first_node_in_group("player")
        self._core = self.get_node("Core")
        self._light = self.get_node("Light")
        self._trail = self.get_node("Trail")
        self._trail.add_chain("../Core", Color(1.0, 0.35, 0.25, 1.0), Color(1.0, 0.1, 0.05, 0.0), 0.12, 0.02, 0.2)

    def _process(self, delta):
        if self._out >= 0.0:
            self._out = self._out + delta
            self._light.light_energy = 2.5 * math.max(1.0 - self._out / 0.12, 0.0)
            if self._out > 0.35:
                self.queue_free()
            return
        self._age = self._age + delta
        d = self.direction
        if d == None or self._age > self.life:
            self.stop()
            return
        p = self.global_position
        step = self.speed * delta
        to = Vec3(p.x + d.x * step, p.y + d.y * step, p.z + d.z * step)
        # The floater is left out only while the bolt leaves it: it may be freed long before.
        hit = raycast(p, to, None, self.source if self._age < 0.2 else None)
        if hit != None:
            self.global_position = hit.position
            if hit.node == self._player and self._player.alive():
                self._player.take_damage(self.damage, hit.position, d, 0.3, self.source if self._age < 0.2 else None)
            self.stop()
            return
        self.global_position = to

    def stop(self):
        self._out = 0.0
        self._core.visible = False
        self._trail.emitting = False
