import math

# A man the police wired to a bomb and an AI: he sits at a computer (or lies on the floor) until
# the android comes near, then gets up and runs at her screaming, and blows up within reach.
# He blows up however he dies; a shot in the bomb sets it off at once.
POSES = {"sit": ["Knockdown_Getup_Back", 0.05], "back": ["Knockdown_Back", 0.98], "front": ["Knockdown_Front", 0.98]}
GETUPS = {"sit": ["Knockdown_Getup_Back", 0.05], "back": ["Knockdown_Getup_Back", 0.0], "front": ["Knockdown_Getup_Front", 0.0]}
FUSE_TIP = Vec3(-1.327, 1.473, 0.051)

class Kamikaze(CharacterBody3D):
    pose = "sit"
    skin = ""
    wake_range = 13.0
    wake_height = 6.0
    run_speed = 5.2
    health = 60.0
    trigger_range = 2.0
    fuse_time = 0.35
    blast_radius = 5.0
    blast_force = 10.0
    blast_damage = 110.0
    gravity = 18.0
    turn_speed = 420.0
    model_yaw = 180.0
    bomb_height = 0.55
    bomb_scale = 0.5
    touch_radius = 0.6
    _game = None
    _player = None
    _anim = None
    _model = None
    _scream = None
    _voice = None
    _fuse_sound = None
    _sparks = None
    _state = "dormant"
    _health = 60.0
    _yaw = 0.0
    _vertical = 0.0
    _think = 0.0
    _timer = 0.0
    _stagger = 0.0
    _clip = ""
    _path = None
    _path_index = 0
    _repath = 0.0
    _clear = False
    _sight = 0.0
    _dead = False
    _screaming = False
    _phys_prev = None
    _phys_cur = None
    _model_base = None

    def _ready(self):
        self._health = self.health
        self._yaw = self.rotation_degrees.y
        self._think = math.random(0.0, 0.3)
        self._phys_prev = self.global_position
        self._phys_cur = self.global_position
        self.cache_nodes()
        self.apply_skin()
        self.make_fuse()
        self.hold_pose()

    # Also after a script reload in the editor, which resets the fields without _ready.
    def cache_nodes(self):
        self._game = self.get_tree().get_first_node_in_group("game")
        self._player = self.get_tree().get_first_node_in_group("player")
        self._model = self.get_node("Model")
        self._scream = self.get_node("Scream")
        self._voice = self.get_node("Voice")
        self._fuse_sound = self.get_node("Fuse")
        if self._model != None:
            self._model.rotation_degrees = Vec3(0.0, self.model_yaw, 0.0)
            self._model_base = self._model.position
            skeleton = self._model.find_child("Skeleton3D", True)
            if skeleton != None:
                self._anim = skeleton.find_child("AnimationPlayer", False)
        if self._anim != None:
            for name in ["Run_Panic", "Idle"]:
                if self._anim.has_animation(name):
                    self._anim.set_animation_loop(name, True)
            self._anim.playback_default_blend_time = 0.15

    # Police (SWAT robot) or army skin over the man, picked at random unless set.
    def apply_skin(self):
        if self._model == None:
            return
        skin = self.skin
        if skin == "":
            skin = ["police", "robot"][int(math.random(0.0, 1.99))]
        material = "Assets/Materials/kamikaze.mat::" + skin
        body = self._model.find_child("Kamikazi_malha", True)
        if body != None:
            body.material_override = material
        attach = self._model.find_child("Bomb", True)
        bomb = attach.get_node("Bomb") if attach != None else None
        if bomb != None:
            bomb.material_override = material
            # Smaller than the game's: it shrinks towards the spring's base on the neck, the fuse
            # with it.
            bomb.scale = Vec3(self.bomb_scale, self.bomb_scale, self.bomb_scale)

    # Sparks and a flicker of light at the tip of the fuse, carried by the bomb.
    def make_fuse(self):
        if self._model == None:
            return
        attach = self._model.find_child("Bomb", True)
        bomb = attach.get_node("Bomb") if attach != None else None
        if bomb == None:
            return
        p = bomb.create_child("ParticleSystem3D")
        p.name = "FuseSparks"
        p.position = FUSE_TIP
        p.texture = "Assets/Textures/Fx/soft.png"
        p.blend_mode = 1
        p.max_particles = 60
        p.emission_mode = 0
        p.emission_rate = 18.0
        p.emission_direction = Vec3(0.0, 1.0, 0.0)
        p.emission_spread_angle = 80.0
        p.particle_lifetime_min = 0.15
        p.particle_lifetime_max = 0.4
        p.particle_speed_min = 0.6
        p.particle_speed_max = 2.2
        p.particle_size_start = Vec2(0.05, 0.05)
        p.particle_size_end = Vec2(0.01, 0.01)
        p.particle_color_start = Color(1.0, 0.9, 0.55, 1.0)
        p.particle_color_end = Color(1.0, 0.35, 0.05, 0.0)
        p.gravity = Vec3(0.0, -4.0, 0.0)
        p.drag = 0.4
        self._sparks = p
        glow = bomb.create_child("OmniLight3D")
        glow.position = FUSE_TIP
        glow.light_color = Color(1.0, 0.55, 0.2, 1.0)
        glow.light_energy = 0.8
        glow.omni_range = 1.6
        if self._fuse_sound != None:
            self._fuse_sound.play()

    def play(self, clip, speed):
        if self._anim == None:
            return
        self._anim.speed_scale = speed
        if clip == self._clip or not self._anim.has_animation(clip):
            return
        self._clip = clip
        self._anim.play(clip)

    def hold_pose(self):
        if self._anim == None:
            return
        held = POSES[self.pose] if self.pose in POSES else None
        if held == None:
            self.play("Idle", 1.0)
            return
        self.play(held[0], 1.0)
        self._anim.seek(self._anim.get_current_animation_length() * held[1], True)
        self._anim.pause()

    def say(self, sound, volume):
        if self._voice == None:
            return
        self._voice.stream = sound
        self._voice.volume_linear = volume
        self._voice.pitch_scale = math.random(0.9, 1.1)
        self._voice.play()

    def alive(self):
        return not self._dead

    def wake(self):
        if self._state != "dormant" or self._dead:
            return
        self._state = "waking"
        getup = GETUPS[self.pose] if self.pose in GETUPS else None
        if getup == None or self._anim == None:
            self._timer = 0.0
            return
        self._clip = ""
        self.play(getup[0], 1.6)
        length = self._anim.get_current_animation_length()
        self._anim.seek(length * getup[1], True)
        self._timer = length * (1.0 - getup[1]) / 1.6
        self.say("Assets/Sounds/kamikaze/wound_0" + str(int(math.random(1.0, 3.99))) + ".ogg", 1.0)

    # Gunshots and other noise reach him even without a line of sight, so a sleeping bomb carrier
    # wakes when the player makes a racket nearby. The game calls this for every bot in earshot.
    def hear(self, position):
        if self._state != "dormant" or self._dead or self.briefing():
            return
        self.wake()

    def _process(self, delta):
        if self._model == None or self._dead or self._phys_prev == None:
            return
        f = physics_interpolation_fraction()
        p = self._phys_prev
        c = self._phys_cur
        gp = self.global_position
        ox = p.x + (c.x - p.x) * f - gp.x
        oy = p.y + (c.y - p.y) * f - gp.y
        oz = p.z + (c.z - p.z) * f - gp.z
        yaw = math.radians(self._yaw)
        cs = math.cos(yaw)
        sn = math.sin(yaw)
        m = self._model_base
        self._model.position = Vec3(m.x + ox * cs - oz * sn, m.y + oy, m.z + ox * sn + oz * cs)

    def _physics_process(self, delta):
        if self._dead:
            return
        if self._player == None:
            self.cache_nodes()
            if self._player == None:
                return
        self._phys_prev = self.global_position
        self.step(delta)
        self._phys_cur = self.global_position

    def step(self, delta):
        move = Vec3(0.0, 0.0, 0.0)
        player = self._player
        p = self.global_position
        q = player.global_position
        dx = q.x - p.x
        dz = q.z - p.z
        dist = math.sqrt(dx * dx + dz * dz)
        if self._state == "dormant":
            self._think = self._think - delta
            if self._think <= 0.0:
                self._think = 0.2
                if player.alive() and dist < self.wake_range and math.abs(q.y - p.y) < self.wake_height and self.sees(player) and not self.briefing():
                    self.wake()
        elif self._state == "waking":
            self._timer = self._timer - delta
            if self._timer <= 0.0:
                self.charge()
        elif self._state == "charge":
            self._stagger = math.max(self._stagger - delta, 0.0)
            self._think = self._think - delta
            if dist <= self.trigger_range and player.alive():
                self.arm()
            elif self._think <= 0.0 and self.touching():
                self.arm()
                self._timer = 0.12
            if not self._screaming and self._game != None and self._game.screamer == None:
                self.scream()
            elif self._stagger <= 0.0:
                way = self.chase_dir(q, delta)
                move = Vec3(way.x * self.run_speed, 0.0, way.z * self.run_speed)
                self.turn_to(way.x, way.z, delta)
                self.play("Run_Panic", self.run_speed / 6.416 * 1.15)
        elif self._state == "armed":
            self.turn_to(dx, dz, delta)
            self._timer = self._timer - delta
            if self._timer <= 0.0:
                self.explode()
                return
        if self.is_on_floor():
            self._vertical = -0.5
        else:
            self._vertical = self._vertical - self.gravity * delta
        self.velocity = Vec3(move.x, self._vertical, move.z)
        self.move_and_slide(delta)
        self.rotation_degrees = Vec3(0.0, self._yaw, 0.0)

    def briefing(self):
        return self._game != None and self._game.briefing()

    def sees(self, target):
        p = self.global_position
        q = target.global_position
        hit = raycast(Vec3(p.x, p.y + 0.5, p.z), Vec3(q.x, q.y + 0.5, q.z), None, self)
        return hit == None or hit.node == target

    # Running into a door, glass, a screen, a car, a barrel or a cop sets the bomb off.
    def touching(self):
        self._think = 0.1
        p = self.global_position
        for body in overlap_sphere(Vec3(p.x, p.y + 0.2, p.z), self.touch_radius):
            if body != self and (body.is_in_group("breakable") or body.is_in_group("enemy") or body.is_in_group("barrel")):
                return True
        return False

    # One scream at a time: the others run silent until the screamer is gone.
    def scream(self):
        if self._scream == None or self._screaming or self._game == None:
            return
        if self._game.screamer != None and self._game.screamer != self:
            return
        self._game.screamer = self
        self._screaming = True
        self._scream.play()

    def charge(self):
        self._state = "charge"
        if self._game != None:
            self._game.bot_seen()
        self._path = None
        self.scream()
        if self._sparks != None:
            self._sparks.emission_rate = 70.0

    def arm(self):
        self._state = "armed"
        self._timer = self.fuse_time
        self.play("Idle", 1.0)
        if self._sparks != None:
            self._sparks.emission_rate = 160.0

    # Along the walkable floor when there is one, straight otherwise.
    def chase_dir(self, goal, delta):
        p = self.global_position
        target = goal
        nav = self._game.nav if self._game != None else None
        # With a clear view of her on the same floor he runs straight in: the bends of the path
        # only slow him down. The view is a raycast, refreshed five times a second; the path is
        # dropped so it is built fresh when the view is lost.
        self._sight = self._sight - delta
        if self._sight <= 0.0:
            self._sight = 0.2
            self._clear = self.sees(self._player)
        if self._clear and math.abs(goal.y - p.y) < 1.5:
            self._path = None
        elif nav != None and nav.is_baked():
            self._repath = self._repath - delta
            if (self._path == None or self._repath <= 0.0) and self._game.path_ticket():
                self._repath = 0.3 if (goal.x - p.x) * (goal.x - p.x) + (goal.z - p.z) * (goal.z - p.z) < 225.0 else 1.0
                self._path = nav.find_path(p, goal)
                self._path_index = 1
            elif self._path == None:
                self._path = []
            while self._path_index < len(self._path):
                c = self._path[self._path_index]
                if (c.x - p.x) * (c.x - p.x) + (c.z - p.z) * (c.z - p.z) > 0.35 * 0.35:
                    break
                self._path_index = self._path_index + 1
            if self._path_index < len(self._path):
                target = self._path[self._path_index]
        dx = target.x - p.x
        dz = target.z - p.z
        length = math.sqrt(dx * dx + dz * dz)
        if length < 0.001:
            return Vec3(0.0, 0.0, 0.0)
        return Vec3(dx / length, 0.0, dz / length)

    # Yaw so that -Z points along (dx, dz), as the android's forward.
    def turn_to(self, dx, dz, delta):
        if dx * dx + dz * dz < 0.0001:
            return
        want = math.degrees(math.atan2(-dx, -dz))
        diff = want - self._yaw
        while diff > 180.0:
            diff = diff - 360.0
        while diff < -180.0:
            diff = diff + 360.0
        limit = self.turn_speed * delta
        self._yaw = self._yaw + math.clamp(diff, -limit, limit)

    # Shots: the bomb goes off when hit; the body staggers and wakes him.
    def take_damage(self, amount, origin, direction, force, source=None):
        if self._dead:
            return
        if origin != None and origin.y > self.global_position.y + self.bomb_height:
            self.explode()
            return
        self._health = self._health - amount
        if self._health <= 0.0:
            self.explode()
            return
        self.say("Assets/Sounds/kamikaze/wound_0" + str(int(math.random(1.0, 3.99))) + ".ogg", 1.0)
        if self._state == "dormant":
            self.wake()
        elif self._state == "charge":
            self._stagger = 0.3
            self._clip = ""
            self.play("Wound" + str(int(math.random(1.0, 2.99))), 1.4)

    def headshot(self, point, direction):
        self.explode()

    def is_headshot(self, point):
        return point.y > self.global_position.y + self.bomb_height

    def dismember(self, direction, force):
        self.explode()

    def die(self, direction, force):
        self.explode()

    # A blast nearby sets his bomb off a moment later.
    def blast(self, center):
        if self._dead or self._state == "armed":
            return
        self._state = "armed"
        self._timer = math.random(0.12, 0.3)

    # Like the cops, he leaves an ammo pickup where he stood. It goes under the game node, since he
    # is freed right after, and on the floor below his body (the body origin is about 0.9 m above it).
    def drop_ammo(self):
        if self._game == None:
            return
        p = self.global_position
        floor = Vec3(p.x, p.y - 0.9, p.z)
        hit = raycast(Vec3(p.x, p.y, p.z), Vec3(p.x, p.y - 2.5, p.z), None, self)
        if hit != None and hit.node != None:
            floor = hit.position
        item = self._game.instantiate("Assets/Prefabs/pickup_ammo.scene.json")
        if item != None:
            item.global_position = floor

    def explode(self):
        if self._dead:
            return
        self._dead = True
        self.remove_from_group("damageable")
        if self._scream != None:
            self._scream.stop()
        if self._game != None and self._game.screamer == self:
            self._game.screamer = None
        if self._fuse_sound != None:
            self._fuse_sound.stop()
        p = self.global_position
        center = Vec3(p.x, p.y + 0.6, p.z)
        if self._game != None:
            self._game.bot_destroyed(self)
            self._game.explode(center, self.blast_radius, self.blast_force, self.blast_damage, "big", self)
        # Still a "bot" through its own blast, so the hack terminal knows it was not her.
        self.remove_from_group("bot")
        self.drop_ammo()
        self.queue_free()
