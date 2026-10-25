import math

# An explosive barrel: shot it catches fire, hisses for a moment and goes up in a big explosion;
# caught in another blast it goes almost at once, so a row of them (with the bombs and the cars
# around) chains. A rigid body: pushed, knocked over and thrown by the blasts that miss it.
class Barrel(RigidBody3D):
    health = 40.0
    fuse_time = 1.1
    blast_radius = 6.5
    blast_force = 11.0
    blast_damage = 150.0
    _health = 40.0
    _fuse = -1.0
    _done = False
    _fire = None

    def _ready(self):
        self._health = self.health

    def take_damage(self, amount, origin, direction, force, source=None):
        if self._done:
            return
        if origin != None and direction != None:
            p = self.global_position
            self.apply_impulse(Vec3(direction.x * amount * 0.4, 0.0, direction.z * amount * 0.4), Vec3(origin.x - p.x, origin.y - p.y, origin.z - p.z))
        self._health = self._health - amount
        if self._health <= 0.0:
            self.light(self.fuse_time)

    # Another explosion reached it.
    def blast(self, center):
        if self._done:
            return
        self.light(math.random(0.15, 0.35))

    def light(self, seconds):
        if self._fuse >= 0.0:
            self._fuse = math.min(self._fuse, seconds)
            return
        self._fuse = seconds
        self._fire = self.instantiate("Assets/Prefabs/burning.scene.json")
        if self._fire != None:
            self._fire.position = Vec3(0.0, 0.45, 0.0)
            sound = self._fire.get_node("Sound")
            if sound != None:
                sound.play()

    def _process(self, delta):
        if self._fuse < 0.0 or self._done:
            return
        self._fuse = self._fuse - delta
        if self._fuse <= 0.0:
            self.explode()

    def explode(self):
        if self._done:
            return
        self._done = True
        game = self.get_tree().get_first_node_in_group("game")
        p = self.global_position
        if game != None:
            game.explode(Vec3(p.x, p.y + 0.2, p.z), self.blast_radius, self.blast_force, self.blast_damage, "big", self)
        self.queue_free()
