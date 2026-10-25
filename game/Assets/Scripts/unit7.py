import math

# Unit-7, the AI that tried to stop NULL//OS, locked in a cell: it sits with its hands on its
# head, waves for help when the android comes near and, once the door is down, follows her along
# the walkable floor to the way out.
class Unit7(CharacterBody3D):
    walk_speed = 1.8
    run_speed = 4.6
    follow_distance = 1.8
    gravity = 18.0
    turn_speed = 420.0
    model_yaw = 180.0
    health = 400.0
    attack_range = 2.2
    attack_cooldown = 1.3
    _health = 400.0
    _attack = 0.0
    _cool = 0.0
    _victim = None
    _struck = False
    _think = 0.0
    _game = None
    _player = None
    _anim = None
    _model = None
    _light = None
    _state = "caged"
    _clip = ""
    _yaw = 0.0
    _vertical = 0.0
    _age = 0.0
    _path = None
    _path_index = 0
    _repath = 0.0

    def _ready(self):
        self.add_to_group("unit7")
        self._health = self.health
        self._yaw = self.rotation_degrees.y
        self.cache_nodes()
        self.play("Sit_Hands_on_Head_Lean_Back", 1.0)

    # Also after a script reload in the editor, which resets the fields without _ready.
    def cache_nodes(self):
        self._game = self.get_tree().get_first_node_in_group("game")
        self._player = self.get_tree().get_first_node_in_group("player")
        self._model = self.get_node("Model")
        self._light = self.get_node("Glow")
        if self._model != None:
            self._model.rotation_degrees = Vec3(0.0, self.model_yaw, 0.0)
            skeleton = self._model.find_child("Skeleton3D", True)
            if skeleton != None:
                self._anim = skeleton.find_child("AnimationPlayer", False)
        if self._anim != None:
            for name in ["Sit_Hands_on_Head_Lean_Back", "Sitting_Clap", "Wave_for_Help", "Walking", "Running", "Alert"]:
                if self._anim.has_animation(name):
                    self._anim.set_animation_loop(name, True)
            self._anim.playback_default_blend_time = 0.25

    def play(self, clip, speed):
        if self._anim == None:
            return
        self._anim.speed_scale = speed
        if clip == self._clip or not self._anim.has_animation(clip):
            return
        self._clip = clip
        self._anim.play(clip)

    def alive(self):
        return self._state != "dead"

    # Bullets and blasts hurt it only once it is out of the cell.
    def take_damage(self, amount, origin, direction, force, source=None):
        if self._state != "free":
            return
        self._health = self._health - amount
        if self._health <= 0.0:
            self._state = "dead"
            self._clip = ""
            self.play("dying_backwards", 1.0)
            story = self._game.story if self._game != None else None
            if story != None:
                story.unit_lost()

    # A cop within reach: a spinning kick or the axe swing, and the cop comes apart.
    def fight(self, delta):
        if self._attack > 0.0:
            self._attack = self._attack - delta
            victim = self._victim
            if not self._struck and self._attack < 0.55 and victim != None and not victim._dead:
                self._struck = True
                p = self.global_position
                v = victim.global_position
                dx = v.x - p.x
                dz = v.z - p.z
                length = math.max(math.sqrt(dx * dx + dz * dz), 0.01)
                if length < self.attack_range + 0.6:
                    victim.dismember(Vec3(dx / length, 0.3, dz / length), 6.0)
                    if self._game != None and math.random(0.0, 1.0) < 0.3:
                        self._game.speak("u7_fight_01")
            return True
        self._cool = math.max(self._cool - delta, 0.0)
        self._think = self._think - delta
        if self._cool > 0.0 or self._think > 0.0 or self._game == None or self._game.enemies == None:
            return False
        self._think = 0.2
        p = self.global_position
        best = None
        best_d = self.attack_range
        # Cops and bomb carriers alike (the kamikazes are in the group "bot").
        for enemy in self._game.enemies + self.get_tree().get_nodes_in_group("bot"):
            if enemy._dead:
                continue
            e = enemy.global_position
            d = math.sqrt((e.x - p.x) * (e.x - p.x) + (e.z - p.z) * (e.z - p.z))
            if d < best_d and math.abs(e.y - p.y) < 1.5:
                best = enemy
                best_d = d
        if best == None:
            return False
        self._victim = best
        self._struck = False
        self._attack = 1.1
        self._cool = self.attack_cooldown
        self._clip = ""
        self.play(["Roundhouse_Kick", "Axe_Spin_Attack"][int(math.random(0.0, 1.99))], 1.4)
        e = best.global_position
        self.turn_to(e.x - p.x, e.z - p.z, 1.0)
        return True

    # The cell door is down: it stands up and goes with her.
    def wake(self):
        self._state = "free"
        sound = self.get_node("Boot")
        if sound != None:
            sound.play()

    def _physics_process(self, delta):
        if self._player == None:
            self.cache_nodes()
            if self._player == None:
                return
        self._age = self._age + delta
        p = self.global_position
        q = self._player.global_position
        dx = q.x - p.x
        dz = q.z - p.z
        dist = math.sqrt(dx * dx + dz * dz)
        move = Vec3(0.0, 0.0, 0.0)
        if self._state == "dead":
            self.velocity = Vec3(0.0, self._vertical, 0.0)
            self.move_and_slide(delta)
            return
        if self._state == "free" and self.fight(delta):
            if self._light != None:
                self._light.light_energy = 3.0
            self.velocity = Vec3(0.0, self._vertical, 0.0)
            self.move_and_slide(delta)
            self.rotation_degrees = Vec3(0.0, self._yaw, 0.0)
            return
        if self._state == "caged":
            if self._light != None:
                self._light.light_energy = 0.15
            # Seated on the top bunk until the inhibitor goes off; only the voice calls her over.
            # Held where she was placed: the seated pose sits lower than the capsule, which the
            # mattress would push up into the air.
            self.play("Sit_Hands_on_Head_Lean_Back", 1.0)
            # She faces whoever comes to the bars.
            if dist < 8.0:
                self.turn_to(dx, dz, delta)
                self.rotation_degrees = Vec3(0.0, self._yaw, 0.0)
            return
        else:
            if self._light != None:
                self._light.light_energy = 1.5 + 0.8 * math.sin(self._age * 4.0)
            if dist > self.follow_distance:
                speed = self.run_speed if dist > 5.0 else self.walk_speed
                way = self.walk_dir(q, delta)
                move = Vec3(way.x * speed, 0.0, way.z * speed)
                self.turn_to(way.x, way.z, delta)
                self.play("Running" if speed > self.walk_speed else "Walking", 1.0)
            else:
                self.turn_to(dx, dz, delta)
                self.play("Alert", 1.0)
        if self.is_on_floor():
            self._vertical = -0.5
        else:
            self._vertical = self._vertical - self.gravity * delta
        self.velocity = Vec3(move.x, self._vertical, move.z)
        self.move_and_slide(delta)
        self.rotation_degrees = Vec3(0.0, self._yaw, 0.0)

    # Along the walkable floor when there is one, straight otherwise.
    def walk_dir(self, goal, delta):
        p = self.global_position
        target = goal
        nav = self._game.nav if self._game != None else None
        if nav != None and nav.is_baked():
            self._repath = self._repath - delta
            if (self._path == None or self._repath <= 0.0) and self._game.path_ticket():
                self._repath = 0.4
                self._path = nav.find_path(p, goal)
                self._path_index = 1
            elif self._path == None:
                self._path = []
            while self._path_index < len(self._path):
                c = self._path[self._path_index]
                if (c.x - p.x) * (c.x - p.x) + (c.z - p.z) * (c.z - p.z) > 0.3 * 0.3:
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
