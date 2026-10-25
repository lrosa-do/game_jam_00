import math

SURFACES = {
    "metal": [22, 2, 0, "", [0.6, 0.6, 0.62], 3],
    "wood": [0, 5, 8, "Assets/Textures/Fx/splinter.png", [0.45, 0.33, 0.2], 3],
    "tile": [3, 6, 6, "Assets/Textures/Fx/chip.png", [0.78, 0.76, 0.72], 3],
    "glass": [6, 0, 6, "Assets/Textures/Fx/glass_shard.png", [0.8, 0.9, 1.0], 3],
    "cardboard": [0, 4, 3, "Assets/Textures/Fx/splinter.png", [0.7, 0.6, 0.45], 3],
    "carpet": [0, 4, 0, "", [0.35, 0.3, 0.28], 2],
    "default": [2, 6, 5, "Assets/Textures/Fx/chip.png", [0.78, 0.75, 0.7], 3],
}

def surface_kind(material):
    name = material.lower()
    for word in ["glass", "window", "mirror"]:
        if name.find(word) >= 0:
            return "glass"
    for word in ["metal", "steel", "locker", "filecabinet", "radiator", "pipe", "grate", "fence", "garage", "car_", "vent", "rail", "iron", "alu", "chrome"]:
        if name.find(word) >= 0:
            return "metal"
    for word in ["wood", "door", "divider", "table", "desk", "chair", "panel", "shelf", "book", "bench", "frame", "parquet", "cabinet"]:
        if name.find(word) >= 0:
            return "wood"
    for word in ["tile", "floor", "marble", "ceramic", "stone", "concrete", "brick", "stair", "asphalt"]:
        if name.find(word) >= 0:
            return "tile"
    for word in ["paper", "card", "box", "newspaper", "carton"]:
        if name.find(word) >= 0:
            return "cardboard"
    for word in ["carpet", "rug", "fabric", "cloth", "curtain", "sofa", "couch"]:
        if name.find(word) >= 0:
            return "carpet"
    return "default"

class Impact(Node3D):
    _age = 0.0

    def burst(self, normal, strength):
        self.get_node("Sparks").emission_direction = normal
        self.get_node("Dust").emission_direction = normal
        self.get_node("Sparks").emit_burst(int(12.0 * strength))
        self.get_node("Dust").emit_burst(int(5.0 * strength))
        self.get_node("Sound").play()

    def surface(self, normal, material):
        kind = surface_kind(material)
        look = SURFACES[kind]
        sparks = self.get_node("Sparks")
        dust = self.get_node("Dust")
        chips = self.get_node("Chips")
        c = look[4]
        for p in [sparks, dust, chips]:
            p.emission_direction = normal
        if look[0] > 0:
            sparks.emit_burst(look[0])
        if look[1] > 0:
            dust.particle_color_start = Color(c[0], c[1], c[2], 0.55)
            dust.particle_color_end = Color(c[0] * 0.8, c[1] * 0.8, c[2] * 0.8, 0.0)
            dust.emit_burst(look[1])
        if look[2] > 0 and look[3] != "":
            chips.texture = look[3]
            chips.particle_color_start = Color(c[0], c[1], c[2], 1.0)
            chips.particle_color_end = Color(c[0], c[1], c[2], 0.7)
            chips.emit_burst(look[2])
        sound = self.get_node("Sound")
        sound.stream = "Assets/Sounds/hits/" + kind + "_" + "abc"[int(math.random(0.0, look[5] - 0.01))] + ".ogg"
        sound.play()
        return kind

    def _process(self, delta):
        self._age = self._age + delta
        if self._age > 1.5:
            self.queue_free()
