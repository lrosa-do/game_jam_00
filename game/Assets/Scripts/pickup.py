import math

class Pickup(Node3D):
    kind = "ammo"
    amount = 18.0
    radius = 1.1
    _age = 0.0
    _player = None
    _mesh = None

    def _ready(self):
        self.add_to_group("pickup")
        self.add_to_group("pickup_" + self.kind)
        self._mesh = self.get_node("Mesh")
        self._player = self.get_tree().get_first_node_in_group("player")
        self._age = math.random(0.0, 6.0)

    def _process(self, delta):
        self._age = self._age + delta
        if self._mesh != None:
            self._mesh.position = Vec3(0.0, 0.25 + math.sin(self._age * 3.0) * 0.05, 0.0)
            self._mesh.rotation_degrees = Vec3(0.0, self._age * 120.0, 0.0)
        if self._player == None or not self._player.alive():
            return
        p = self._player.global_position
        g = self.global_position
        dx = p.x - g.x
        dz = p.z - g.z
        if dx * dx + dz * dz > self.radius * self.radius or math.abs(p.y - g.y) > 2.0:
            return
        if self._player.pickup(self.kind, self.amount):
            self.queue_free()
