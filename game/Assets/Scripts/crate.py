import math

# A wooden crate: shot enough, kicked or caught in a blast it breaks, the whole model swapped for
# its broken one (planks and splinters), and now and then it leaves a pickup in the wreck. A rigid
# body, so it is pushed about and thrown by the blasts that do not break it.
class Crate(RigidBody3D):
    health = 50.0
    broken_model = "Assets/Models/Props/crate_small_broken.glb"
    drop_chance = 0.45
    _health = 50.0
    _broken = False

    def _ready(self):
        self._health = self.health

    def take_damage(self, amount, origin, direction, force, source=None):
        if self._broken:
            return
        if origin != None and direction != None:
            p = self.global_position
            self.apply_impulse(Vec3(direction.x * amount * 0.3, 0.0, direction.z * amount * 0.3), Vec3(origin.x - p.x, origin.y - p.y, origin.z - p.z))
        self._health = self._health - amount
        if self._health <= 0.0:
            self.smash(direction)

    # Another explosion reached it.
    def blast(self, center):
        p = self.global_position
        self.smash(Vec3(p.x - center.x, 0.3, p.z - center.z))

    def smash(self, direction):
        if self._broken:
            return
        self._broken = True
        game = self.get_tree().get_first_node_in_group("game")
        model = self.get_node("Model")
        if game == None or model == None:
            self.queue_free()
            return
        wreck = game.instantiate(self.broken_model)
        if wreck != None:
            wreck.global_position = model.global_position
            wreck.global_rotation = model.global_rotation
            wreck.scale = model.scale
        p = self.global_position
        fx = game.instantiate("Assets/Prefabs/impact.scene.json")
        if fx != None:
            fx.global_position = p
            fx.burst(Vec3(direction.x if direction != None else 0.0, 0.5, direction.z if direction != None else 0.0), 2.0)
        boom = game.instantiate("Assets/Prefabs/boom.scene.json")
        if boom != None:
            boom.global_position = p
            boom.peak = 0.0
            boom.get_node("Sound").stream = "Assets/Sounds/break/explosiondebris_wood_medium_A.ogg"
        if math.random(0.0, 1.0) < self.drop_chance:
            kind = "ammo" if math.random(0.0, 1.0) < 0.6 else "energy"
            item = game.instantiate("Assets/Prefabs/pickup_" + kind + ".scene.json")
            if item != None:
                item.global_position = Vec3(p.x, model.global_position.y, p.z)
        self.queue_free()
