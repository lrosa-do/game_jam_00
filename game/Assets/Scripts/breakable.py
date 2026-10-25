import math

def lightmap_part(name):
    i = name.find("_lm")
    return i >= 0 and i + 3 < len(name) and "0123456789".find(name[i + 3]) >= 0

# A door leaf of the level (not the doors of a cabinet or a van): "<room>__...__door_20".
def door_name(name):
    if name.find("cabinet") >= 0 or name.find("closet") >= 0 or name.find("van_parent") >= 0:
        return False
    start = 0
    for i in range(len(name) - 1):
        if name[i] == "_" and name[i + 1] == "_":
            start = i + 2
    leaf = name[start:len(name)]
    return leaf.find("door") == 0 or leaf.find("d_cell_door") == 0 or leaf.find("d_door_exit") == 0

def vehicle_name(name):
    if name.find("van_parent") < 0 and name.find("vlads_car") < 0:
        return False
    return name.find("__prefab") < 0 and name.find("::prefab") < 0 and name.find("tires") < 0

def breakable_kind(name):
    if lightmap_part(name):
        return ""
    if name.find("break_glass") == 0:
        return "glass"
    if name.find("break_monitor") == 0:
        return "monitor"
    if name.find("break_extinguisher") == 0:
        return "extinguisher"
    if name.find("break_car") == 0:
        return "car"
    if name.find("break_table") == 0 or name.find("break_chair") == 0 or name.find("break_prop") == 0:
        return "prop"
    if name.find("break_vending") == 0:
        return "vending"
    if name.find("break_vase") == 0:
        return "small"
    if vehicle_name(name):
        return "car"
    if door_name(name):
        return "door"
    if name.find("WaterMac") >= 0 or name.find("d_water") >= 0:
        return "cooler"
    if name.find("WaterBottle") >= 0 or name.find("SodaCan") >= 0:
        return "small"
    return ""

# The physics layer the hinged doors live on (layer 2), apart from the level geometry.
DOOR_LAYER = 2
# A leaf torn off its hinge: on its own layer, meeting the level only, so nobody gets wedged on it in
# the doorway.
BLOWN_LAYER = 4

class Breakable(Node3D):
    kind = "glass"
    hits = 2
    car_health = 220.0
    door_health = 120.0
    locked = False
    _denied = 0.0
    _body = None
    _mesh = None
    _broken = False
    _sfx = None
    _box = None
    _jets = 0
    _smoking = False
    _health = 0.0
    _fuse = -1.0
    _hinge = None
    _hinge_pending = False
    _zone = None

    # The piece is gone (its pieces fly on as debris): the game's zone culling lets go of it too.
    def free_body(self):
        if self._zone != None:
            self._zone.node = None
            self._zone = None
        self._body.queue_free()

    def setup(self):
        self._body = self.get_parent()
        self.kind = breakable_kind(self._body.name)
        self._mesh = self._body.get_node("Mesh")
        self._body.add_to_group("breakable")
        if self.kind == "door":
            # Doors can be blown away, so the walkable floor runs under them.
            self._body.add_to_group("navigation_ignore")
            # Room doors swing on a hinge (made next frame, out of the level walk); the cells'
            # door NULL//OS keeps shut and the garage's rolling door stay as they are.
            name = self._body.name
            self._hinge_pending = name.find("d_door_exit") < 0 and name.find("cell_door") < 0
        self.hits = 2 if self.kind == "glass" else 1
        self._health = self.door_health if self.kind == "door" else self.car_health
        if self.kind == "vending":
            self._health = 90.0
        self._sfx = self.get_node("Sfx")
        if self._sfx == None:
            self._sfx = self.create_child("AudioStreamPlayer3D")
            self._sfx.name = "Sfx"
            self._sfx.attenuation_model = 1
            self._sfx.max_distance = 30.0
            self._sfx.unit_size = 3.0
        if self._mesh != None:
            self._box = self._mesh.get_aabb()
            # A car's loose wheel or trim exported as its own car piece is a prop: it is thrown by
            # the body's blast instead of blowing up as a car of its own (the street car came as
            # five cars, the body and four tyres, and went off five times).
            if self.kind == "car":
                size = self.world_size()
                if math.max(math.max(size.x, size.y), size.z) < 1.6:
                    self.kind = "prop"
        # Idle, nothing to run each frame: switched on only for a hinge to make and keep, or a fuse.
        self.set_process(self._hinge_pending)

    def effects_parent(self):
        scene = self.get_tree().get_first_node_in_group("game")
        if scene == None:
            scene = self.get_tree().get_current_scene()
        return scene

    def play(self, path, volume):
        self._sfx.stream = path
        self._sfx.volume_linear = volume
        self._sfx.global_position = self._body.global_position
        self._sfx.play()

    def center(self):
        if self._box == None:
            return self._body.global_position
        b = self._box
        return self._mesh.to_global(Vec3((b.min.x + b.max.x) * 0.5, (b.min.y + b.max.y) * 0.5, (b.min.z + b.max.z) * 0.5))

    def world_size(self):
        return self.size_of(self._mesh, self._box)

    def length_of(self, a, b):
        return math.sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z))

    def size_of(self, mesh, box):
        a = mesh.to_global(box.min)
        b = mesh.to_global(box.max)
        return Vec3(math.max(math.abs(b.x - a.x), 0.05), math.max(math.abs(b.y - a.y), 0.05), math.max(math.abs(b.z - a.z), 0.05))

    def decal(self, texture, point, normal, size):
        decal = self.create_child("Decal3D")
        decal.texture_albedo = texture
        decal.size = Vec3(size, 0.1, size)
        decal.global_position = Vec3(point.x + normal.x * 0.01, point.y + normal.y * 0.01, point.z + normal.z * 0.01)
        ax = 0.0 - normal.z
        az = normal.x
        length = math.sqrt(ax * ax + az * az)
        if length > 0.0001:
            angle = math.acos(math.clamp(normal.y, -1.0, 1.0))
            s = math.sin(angle * 0.5)
            decal.quaternion = Quat(ax / length * s, 0.0, az / length * s, math.cos(angle * 0.5))

    def spawn(self, path, point):
        fx = self.effects_parent().instantiate(path)
        if fx != None:
            fx.global_position = point
        return fx

    # source: who hit it (the player passes herself, a cop itself, a blast nothing). The hack terminal
    # has to be protected: her own hit or a blast on it is a game over; a cop's stray shot is not.
    def hit(self, point, normal, direction, damage, source=None):
        if self._body == None:
            self.setup()
        if self.kind == "":
            return
        if self.kind == "glass":
            self.hit_glass(point, normal, direction, damage)
        elif self.kind == "monitor":
            self.hit_monitor(point, normal, source)
        elif self.kind == "extinguisher":
            self.hit_extinguisher(point, normal)
        elif self.kind == "cooler":
            self.hit_jet(point, normal)
        elif self.kind == "car":
            self.hit_car(point, normal, damage)
        elif self.kind == "prop":
            self.hit_prop(point, normal, direction, damage)
        elif self.kind == "vending":
            self.hit_vending(point, normal, direction, damage)
        elif self.kind == "door":
            if self.locked:
                self.sealed(point, normal)
            else:
                self.hit_door(point, normal, direction, damage)
        else:
            self.hit_small(point, normal)

    def _process(self, delta):
        if self._hinge_pending:
            self._hinge_pending = False
            self.make_hinge()
            if self._hinge == None:
                self.set_process(False)
            return
        if self._hinge != None and not self._broken:
            # Self-closing: the motor pulls it back towards shut, gently enough to push open.
            self._hinge.set_motor(-self._hinge.get_angle() * 3.0, 25.0)
            return
        if self._fuse < 0.0 or self._broken:
            return
        self._fuse = self._fuse - delta
        if self._fuse > 0.0:
            return
        self._fuse = -1.0
        self.set_process(self._hinge != None)
        if self.kind == "car":
            self.explode_car()
        elif self.kind == "extinguisher":
            self.explode_extinguisher()

    def hit_glass(self, point, normal, direction, damage):
        self.hits = self.hits - 1
        if damage >= 100.0:
            self.hits = 0
        if self.hits > 0:
            self.decal("Assets/Textures/Decals/hole_glass.png", point, normal, 0.5)
            self.play("Assets/Sounds/break/hit_glass_" + ["a", "b"][int(math.random(0.0, 1.99))] + ".ogg", 1.0)
            return
        self.shatter(direction)

    def shatter(self, direction):
        if self._broken:
            return
        self._broken = True
        c = self.center()
        fx = self.spawn("Assets/Prefabs/glass_shatter.scene.json", c)
        if fx != None and self._box != None:
            size = self.world_size()
            area = math.max(size.x * size.y, math.max(size.x * size.z, size.y * size.z))
            for name in ["Shards", "Glint"]:
                p = fx.get_node(name)
                p.shape_box_size = size
                p.emission_direction = Vec3(direction.x, direction.y - 0.3, direction.z)
            fx.get_node("Shards").emit_burst(int(math.clamp(area * 70.0, 25.0, 150.0)))
            fx.get_node("Glint").emit_burst(int(math.clamp(area * 20.0, 8.0, 40.0)))
            sound = fx.get_node("Sound")
            if area < 0.6:
                sound.stream = "Assets/Sounds/break/glass_break_small_a.ogg"
            else:
                sound.stream = "Assets/Sounds/break/glass_break_medium_" + ["a", "b"][int(math.random(0.0, 1.99))] + ".ogg"
            sound.play()
        self.free_body()

    # A screen pops: the glass bursts out with sparks and a small bang, and the hole smokes.
    def hit_monitor(self, point, normal, source=None):
        if self._broken:
            return
        self._broken = True
        c = self.center()
        scene = self.effects_parent()
        if (source == None or source.is_in_group("player")) and scene != None and scene.is_in_group("game") and scene.story != None:
            scene.story.terminal_broken(c)
        fx = self.spawn("Assets/Prefabs/impact.scene.json", point)
        if fx != None:
            fx.burst(normal, 1.6)
        glass = self.spawn("Assets/Prefabs/glass_shatter.scene.json", c)
        if glass != None and self._box != None:
            size = self.world_size()
            for name in ["Shards", "Glint"]:
                p = glass.get_node(name)
                p.shape_box_size = size
                p.emission_direction = Vec3(normal.x, normal.y + 0.3, normal.z)
            glass.get_node("Shards").emit_burst(30)
            glass.get_node("Glint").emit_burst(12)
            sound = glass.get_node("Sound")
            sound.stream = "Assets/Sounds/break/explosiondebris_glass_small.ogg"
            sound.play()
        self.play("Assets/Sounds/break/electric_box_destroy_01.wav", 1.0)
        self.effects_parent().explode(c, 1.4, 2.5, 15.0, "small", self._body)
        smoke = self.spawn("Assets/Prefabs/smoke_trail.scene.json", c)
        if smoke != None:
            smoke.get_node("Sound").play()
        self.free_body()

    def jet(self, point, normal, gas):
        jet = self.spawn("Assets/Prefabs/jet.scene.json", point)
        if jet == None:
            return None
        p = jet.get_node("Jet")
        p.emission_direction = normal
        if gas:
            p.texture = "Assets/Textures/Fx/smoke.png"
            p.gravity = Vec3(0.0, 0.4, 0.0)
            p.particle_size_end = Vec2(1.4, 1.4)
            p.particle_color_start = Color(0.95, 0.95, 0.95, 0.75)
            p.particle_color_end = Color(0.9, 0.9, 0.9, 0.0)
            p.particle_lifetime_max = 1.6
            p.emission_rate = 70.0
            jet.get_node("Sound").stream = "Assets/Sounds/break/steam_jet_loop.ogg"
        jet.get_node("Sound").play()
        return jet

    def hit_jet(self, point, normal):
        if self._broken or self._jets >= 3:
            return
        self._jets = self._jets + 1
        self.jet(point, normal, False)

    def hit_extinguisher(self, point, normal):
        if self._broken or self._fuse >= 0.0:
            return
        jet = self.jet(point, normal, True)
        if jet != None:
            jet.stop_at = 0.6
        self._fuse = 0.6
        self.set_process(True)

    def explode_extinguisher(self):
        self._broken = True
        c = self.center()
        cloud = self.spawn("Assets/Prefabs/white_cloud.scene.json", c)
        if cloud != None:
            cloud.get_node("Cloud").emit_burst(60)
            cloud.get_node("Sound").play()
        self.effects_parent().explode(c, 5.5, 9.0, 140.0, "big", self._body)
        # The empty bottle flies off spinning.
        if self._mesh != None and self._box != None:
            bottle = self.debris(self._mesh, self._box, c, Vec3(math.random(-4.0, 4.0), math.random(7.0, 10.0), math.random(-4.0, 4.0)), Vec3(math.random(-15.0, 15.0), math.random(-15.0, 15.0), math.random(-15.0, 15.0)))
            bottle.mass = 6.0
            self.attach_lightmap_parts(self._body, bottle, c)
        self.free_body()

    # Furniture stays part of the room until something hits it, then it is thrown as a rigid body
    # (a chair hops off a bullet, a blast or a kick sends tables flying); later hits push it.
    def hit_prop(self, point, normal, direction, damage):
        if self._broken or self._mesh == None or self._box == None:
            return
        self._broken = True
        c = self.center()
        size = self.world_size()
        heavy = size.x * size.z > 1.0
        speed = math.clamp(damage / (40.0 if heavy else 12.0), 1.5, 11.0)
        piece = self.debris(self._mesh, self._box, c, Vec3(direction.x * speed, 1.0 + speed * 0.35, direction.z * speed), Vec3(math.random(-4.0, 4.0), math.random(-6.0, 6.0), math.random(-4.0, 4.0)))
        piece.mass = 45.0 if heavy else 6.0
        self.attach_lightmap_parts(self._body, piece, c)
        fx = self.spawn("Assets/Prefabs/impact.scene.json", point)
        if fx != None:
            fx.surface(normal, "wood" if heavy else "plastic")
        self.free_body()

    # A soda machine takes a few shots, then blows: a medium blast and the machine thrown over.
    def hit_vending(self, point, normal, direction, damage):
        if self._broken:
            return
        fx = self.spawn("Assets/Prefabs/impact.scene.json", point)
        if fx != None:
            fx.surface(normal, "metal")
        self._health = self._health - damage
        if self._health > 0.0 and damage < 100.0:
            self.decal("Assets/Textures/Decals/hole_metal.png", point, normal, 0.08)
            return
        c = self.center()
        # Broken while its own blast goes off, or the blast hits it again and again.
        self._broken = True
        self.effects_parent().explode(c, 3.5, 7.0, 60.0, "medium", self._body)
        self._broken = False
        self.hit_prop(c, normal, direction, 400.0)

    # Shots splinter a door; a kick or a blast (100 damage or more) tears it off its hinges.
    def hit_door(self, point, normal, direction, damage):
        if self._broken:
            return
        metal = self.metal_door()
        fx = self.spawn("Assets/Prefabs/impact.scene.json", point)
        if fx != None:
            fx.surface(normal, "metal" if metal else "wood")
        self._health = self._health - damage
        if self._hinge != None:
            # A shot pushes the leaf on its hinge.
            o = self._body.global_position
            self._body.apply_impulse(Vec3(direction.x * damage * 0.15, 0.0, direction.z * damage * 0.15), Vec3(point.x - o.x, point.y - o.y, point.z - o.z))
        if self._health > 0.0 and damage < 100.0:
            self.decal("Assets/Textures/Decals/hole_metal.png" if metal else "Assets/Textures/Decals/hole_wood.png", point, normal, 0.08)
            if metal:
                self.play("Assets/Sounds/break/door_metal_bang_" + ["A", "B"][int(math.random(0.0, 1.99))] + ".ogg", 0.7)
            return
        self.blow_door(direction, math.max(damage, 100.0))

    # The leaf becomes a light rigid body on a vertical hinge at one edge, held to the world: she
    # (and anyone with a push) swings it open, it swings back. The controller moves onto the new
    # body, which keeps the old one's name, so shots, kicks and the story still find the door.
    def make_hinge(self):
        if self._mesh == None or self._box == None or self.locked:
            return
        old = self._body
        a = self._mesh.to_global(self._box.min)
        b = self._mesh.to_global(self._box.max)
        lo = Vec3(math.min(a.x, b.x), math.min(a.y, b.y), math.min(a.z, b.z))
        hi = Vec3(math.max(a.x, b.x), math.max(a.y, b.y), math.max(a.z, b.z))
        c = self.center()
        rigid = self.debris(self._mesh, self._box, c, Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0))
        rigid.mass = 20.0
        # On its own layer, not touching the level: modelled inside its frame, it would start wedged
        # in it. People still meet it (their mask takes layer 2) and the hinge limits keep it out of
        # the walls.
        rigid.collision_layer = DOOR_LAYER
        rigid.collision_mask = DOOR_LAYER
        self.attach_lightmap_parts(old, rigid, c)
        rigid.add_to_group("breakable")
        rigid.add_to_group("navigation_ignore")
        self.reparent(rigid)
        self._body = rigid
        if self._zone != None:
            self._zone.node = rigid
        old.queue_free()
        rigid.name = old.name
        # The hinge edge: one end of the leaf's width; on a double door the end away from the other
        # leaf, so the pair opens from the middle.
        along_z = (hi.z - lo.z) > (hi.x - lo.x)
        first = Vec3(c.x, c.y, lo.z) if along_z else Vec3(lo.x, c.y, c.z)
        second = Vec3(c.x, c.y, hi.z) if along_z else Vec3(hi.x, c.y, c.z)
        edge = first
        other = self.twin_leaf(old.name)
        if other != None and self.length_of(second, other) > self.length_of(first, other):
            edge = second
        joint = self.effects_parent().create_child("HingeJoint3D")
        joint.global_position = edge
        joint.rotation = Vec3(-1.5708, 0.0, 0.0)
        joint.angular_limit_enable = True
        joint.angular_limit_lower = -1.75
        joint.angular_limit_upper = 1.75
        joint.set_bodies(rigid, None)
        self._hinge = joint

    # The centre of the other leaf of a double door (same prefab), or None for a single door.
    def twin_leaf(self, name):
        cut = name.find("prefab")
        if cut < 0 or name.find("dual") < 0:
            return None
        prefix = name[0:cut]
        for body in self.get_tree().get_nodes_in_group("breakable"):
            if body.name != name and body.name.find(prefix) == 0:
                item = body.get_node("Breakable")
                if item != None and item._box != None:
                    return item.center()
        return None

    # Locked by the story (Unit-7's cell): a hinged leaf no longer swings.
    def lock(self):
        self.locked = True
        if self._hinge != None:
            self._hinge.angular_limit_lower = 0.0
            self._hinge.angular_limit_upper = 0.0
        else:
            self._hinge_pending = False

    # A door NULL//OS keeps shut: sparks off it, nothing gives.
    def sealed(self, point, normal):
        fx = self.spawn("Assets/Prefabs/impact.scene.json", point)
        if fx != None:
            fx.surface(normal, "metal")
        self.play("Assets/Sounds/break/door_metal_bang_" + ["A", "B"][int(math.random(0.0, 1.99))] + ".ogg", 0.8)
        clock = self.effects_parent()._clock
        if clock - self._denied > 2.0:
            self._denied = clock
            self.effects_parent().show("CELL SEALED", 1.2)

    def metal_door(self):
        name = self._body.name
        return name.find("metal") >= 0 or name.find("cell") >= 0 or name.find("exit") >= 0 or name.find("lockup") >= 0

    # Knocked out of its frame: the leaf tips over towards direction and falls flat, heavy, with a
    # metal crash (the garage's rolling door). Returns the fallen leaf's body.
    def topple(self, direction):
        if self._broken or self._mesh == None or self._box == None:
            return
        self._broken = True
        c = self.center()
        # Spin about the axis across the fall (up x direction) so the top goes first.
        door = self.debris(self._mesh, self._box, c, Vec3(direction.x * 1.5, 0.3, direction.z * 1.5), Vec3(direction.z * 2.0, 0.0, -direction.x * 2.0))
        door.mass = 900.0
        # Out of the wall's depth first, or it wedges against the frame as it tips.
        door.global_position = Vec3(c.x + direction.x * 0.8, c.y + 0.05, c.z + direction.z * 0.8)
        self.attach_lightmap_parts(self._body, door, c)
        self.play("Assets/Sounds/break/door_metal_break_" + ["A", "B"][int(math.random(0.0, 1.99))] + ".ogg", 1.0)
        self.free_body()
        return door

    def blow_door(self, direction, damage):
        if self._broken or self._mesh == None or self._box == None:
            return
        self._broken = True
        c = self.center()
        speed = math.clamp(damage / 25.0, 4.0, 12.0)
        if self._hinge != None:
            # Torn off its hinge: the same body flies.
            self._hinge.queue_free()
            self._hinge = None
            self._body.mass = 35.0
            self._body.collision_layer = BLOWN_LAYER
            self._body.collision_mask = 1
            self._body.linear_velocity = Vec3(direction.x * speed, 1.5 + direction.y * speed * 0.5, direction.z * speed)
            self._body.angular_velocity = Vec3(math.random(-3.0, 3.0), math.random(-6.0, 6.0), math.random(-3.0, 3.0))
            kind = "metal" if self.metal_door() else "wood"
            self.play("Assets/Sounds/break/door_" + kind + "_break_" + ["A", "B"][int(math.random(0.0, 1.99))] + ".ogg", 1.0)
            fx = self.spawn("Assets/Prefabs/impact.scene.json", c)
            if fx != None:
                fx.burst(Vec3(direction.x, 0.2, direction.z), 2.0)
            return
        door = self.debris(self._mesh, self._box, c, Vec3(direction.x * speed, 1.5 + direction.y * speed * 0.5, direction.z * speed), Vec3(math.random(-3.0, 3.0), math.random(-6.0, 6.0), math.random(-3.0, 3.0)))
        door.mass = 35.0
        door.collision_layer = BLOWN_LAYER
        door.collision_mask = 1
        self.attach_lightmap_parts(self._body, door, c)
        kind = "metal" if self.metal_door() else "wood"
        self.play("Assets/Sounds/break/door_" + kind + "_break_" + ["A", "B"][int(math.random(0.0, 1.99))] + ".ogg", 1.0)
        fx = self.spawn("Assets/Prefabs/impact.scene.json", c)
        if fx != None:
            fx.burst(Vec3(direction.x, 0.2, direction.z), 2.0)
        self.free_body()

    def hit_small(self, point, normal):
        if self._broken:
            return
        self._broken = True
        jet = self.spawn("Assets/Prefabs/jet.scene.json", point)
        if jet != None:
            jet.stop_at = 0.5
            p = jet.get_node("Jet")
            p.emission_direction = Vec3(0.0, 1.0, 0.0)
            p.emission_spread_angle = 70.0
            p.emit_burst(40)
            jet.get_node("Sound").play()
        self.free_body()

    def hit_car(self, point, normal, damage):
        if self._broken:
            return
        fx = self.spawn("Assets/Prefabs/impact.scene.json", point)
        if fx != None:
            fx.surface(normal, "metal")
        if damage < 100.0:
            self.decal("Assets/Textures/Decals/hole_metal.png", point, normal, 0.1)
        self._health = self._health - damage
        if not self._smoking and self._health < self.car_health * 0.5:
            self._smoking = True
            smoke = self.spawn("Assets/Prefabs/smoke_trail.scene.json", self.center())
            if smoke != None:
                smoke.life = 30.0
                smoke.stop_at = 28.0
        if self._health <= 0.0 and self._fuse < 0.0:
            self._fuse = math.random(0.25, 0.6) if damage >= 100.0 else 0.05
            self.set_process(True)

    def parts(self, out):
        name = self._body.name
        cut = -1
        for i in range(len(name)):
            if name[i] == "_":
                cut = i
        if cut <= 0:
            return
        base = name[0:cut] + "__"
        parent = self._body.get_parent()
        if parent == None:
            return
        for sibling in parent.get_children():
            if sibling != self._body and sibling.name.find(base) == 0 and not lightmap_part(sibling.name) and sibling.get_node("Mesh") != None:
                out.append(sibling)

    def attach_lightmap_parts(self, node, rigid, center):
        parent = node.get_parent()
        if parent == None:
            return
        prefix = node.name + "_lm"
        for sibling in parent.get_children():
            if sibling.name.find(prefix) != 0:
                continue
            mesh = sibling.get_node("Mesh")
            if mesh != None:
                origin = mesh.global_position
                mesh.reparent(rigid)
                mesh.position = rigid.to_local(origin)
                mesh.rotation = Vec3(0.0, 0.0, 0.0)
            sibling.queue_free()

    # The piece's body takes the piece's own rotation and its box the piece's own size: a door at
    # an angle gets a thin slab, not the fat world-aligned box around it, which it would rest on
    # in the air.
    def debris(self, mesh, box, center, velocity, spin):
        rigid = self.effects_parent().create_child("RigidBody3D")
        rigid.mass = 300.0
        o = mesh.to_global(Vec3(0.0, 0.0, 0.0))
        sx = self.length_of(mesh.to_global(Vec3(1.0, 0.0, 0.0)), o)
        sy = self.length_of(mesh.to_global(Vec3(0.0, 1.0, 0.0)), o)
        sz = self.length_of(mesh.to_global(Vec3(0.0, 0.0, 1.0)), o)
        shape = rigid.create_child("CollisionShape3D")
        shape.shape_type = 1
        shape.box_size = Vec3(math.max((box.max.x - box.min.x) * sx * 0.95, 0.04), math.max((box.max.y - box.min.y) * sy * 0.95, 0.04), math.max((box.max.z - box.min.z) * sz * 0.95, 0.04))
        rigid.global_position = center
        rigid.global_rotation = mesh.global_rotation
        origin = mesh.global_position
        mesh.reparent(rigid)
        mesh.position = rigid.to_local(origin)
        mesh.rotation = Vec3(0.0, 0.0, 0.0)
        rigid.linear_velocity = velocity
        rigid.angular_velocity = spin
        return rigid

    def explode_car(self):
        if self._broken or self._mesh == None or self._box == None:
            return
        self._broken = True
        c = self.center()
        self.effects_parent().explode(c, 7.5, 9.0, 160.0, "car", self._body)
        pieces = []
        self.parts(pieces)
        for part in pieces:
            mesh = part.get_node("Mesh")
            box = mesh.get_aabb()
            b = mesh.to_global(Vec3((box.min.x + box.max.x) * 0.5, (box.min.y + box.max.y) * 0.5, (box.min.z + box.max.z) * 0.5))
            dx = b.x - c.x
            dz = b.z - c.z
            l = math.max(math.sqrt(dx * dx + dz * dz), 0.1)
            piece = self.debris(mesh, box, b, Vec3(dx / l * 7.0, math.random(5.0, 9.0), dz / l * 7.0), Vec3(math.random(-8.0, 8.0), math.random(-8.0, 8.0), math.random(-8.0, 8.0)))
            piece.mass = 40.0
            self.attach_lightmap_parts(part, piece, b)
            part.queue_free()
        # A car weighs over a tonne: the blast only lifts it a little and nothing shoves it after.
        wreck = self.debris(self._mesh, self._box, c, Vec3(math.random(-1.0, 1.0), math.random(4.5, 6.0), math.random(-1.0, 1.0)), Vec3(math.random(-1.5, 1.5), math.random(-0.6, 0.6), math.random(-1.5, 1.5)))
        wreck.mass = 1400.0
        self.attach_lightmap_parts(self._body, wreck, c)
        fire = wreck.instantiate("Assets/Prefabs/burning.scene.json")
        if fire != None:
            fire.get_node("Sound").play()
        self.free_body()
