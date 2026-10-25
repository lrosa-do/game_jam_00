import math

# The android's mechanical spider: it keeps behind her along the walkable floor and, on Q, runs
# to the cop under the crosshair (or the nearest one ahead), leaps on it and tears it apart.
# Shot by her it protests; shot rogue_hits times in a row it turns on her for a few seconds (an AI
# gone wrong too), bites once and recalibrates; shot to pieces it is rebuilt like after a blast.
class Spider(CharacterBody3D):
    follow_distance = 1.2
    walk_speed = 3.0
    run_speed = 7.5
    hunt_range = 18.0
    leap_range = 1.7
    leap_time = 0.55
    bite_at = 0.25
    cooldown = 2.5
    gravity = 18.0
    turn_speed = 540.0
    model_yaw = 180.0
    walk_clip_speed = 1.4
    health = 120.0
    rogue_hits = 3.0
    friendly_fire = 0.35  # her own bullets: it turns on her long before it breaks
    rogue_time = 6.0
    bite_damage = 25.0
    _health = 120.0
    _fx = None
    leash = 2.0
    _anchor = None
    _settled = False
    _still = 0.0
    _anger = 0.0
    _rogue = 0.0
    _game = None
    _player = None
    _anim = None
    _model = None
    _model_base = None
    _voice = None
    _steps = None
    _buzz = None
    _state = "follow"
    _target = None
    _yaw = 0.0
    _vertical = 0.0
    _cool = 0.0
    _leap = 0.0
    _bitten = False
    _step = 0.0
    _clip = ""
    _path = None
    _path_index = 0
    _repath = 0.0
    _path_goal = None
    _phys_prev = None
    _phys_cur = None
    _wrecked = False
    _build = -1.0
    _grow = -1.0
    _model_scale = None

    def _ready(self):
        self._health = self.health
        self.add_to_group("damageable")
        self.cache_nodes()
        self._yaw = self.rotation_degrees.y
        self._phys_prev = self.global_position
        self._phys_cur = self.global_position
        self.play("Idle", 1.0)

    # Also after a script reload in the editor, which resets the fields without _ready.
    def cache_nodes(self):
        self._game = self.get_tree().get_first_node_in_group("game")
        self._player = self.get_tree().get_first_node_in_group("player")
        self._model = self.get_node("Model")
        self._voice = self.get_node("Voice")
        self._steps = self.get_node("Steps")
        self._buzz = self.get_node("Buzz")
        if self._model != None:
            self._model.rotation_degrees = Vec3(0.0, self.model_yaw, 0.0)
            self._model_base = self._model.position
            self._model_scale = self._model.scale
            skeleton = self._model.find_child("Skeleton3D", True)
            if skeleton != None:
                self._anim = skeleton.find_child("AnimationPlayer", False)
        if self._anim != None:
            for name in ["Idle", "Walk", "Threat"]:
                if self._anim.has_animation(name):
                    self._anim.set_animation_loop(name, True)
            self._anim.playback_default_blend_time = 0.12

    def play(self, clip, speed):
        if self._anim == None:
            return
        self._anim.speed_scale = speed
        if clip == self._clip or not self._anim.has_animation(clip):
            return
        self._clip = clip
        self._anim.play(clip)

    def say(self, sound, volume):
        if self._voice == None:
            return
        self._voice.stream = sound
        self._voice.volume_linear = volume
        self._voice.pitch_scale = math.random(0.9, 1.15)
        self._voice.play()

    # Built in place by the fx (Assets/Prefabs/materialize.scene.json): hidden and still until its
    # burst calls appear() (delay is only a safety net), then it grows out of the flash.
    def materialize(self, delay):
        if self._model == None:
            self.cache_nodes()
        self._build = delay
        if self._model != None:
            self._model.visible = False
        if self._buzz != None:
            self._buzz.stop()

    def appear(self):
        self._fx = None
        self._build = -1.0
        self._grow = 0.0
        if self._model != None:
            self._model.scale = Vec3(0.01, 0.01, 0.01)
            self._model.visible = True
        self.say("Assets/Sounds/spider/servoup.ogg", 0.8)

    def _process(self, delta):
        if self._model == None or self._phys_prev == None or self._phys_cur == None:
            return
        if self._grow >= 0.0:
            self._grow = self._grow + delta
            t = math.clamp(self._grow / 0.35, 0.0, 1.0)
            k = 1.0 + 2.2 * (t - 1.0) * (t - 1.0) * (t - 1.0) + 1.2 * (t - 1.0) * (t - 1.0)
            b = self._model_scale
            self._model.scale = Vec3(b.x * k, b.y * k, b.z * k)
            if t >= 1.0:
                self._grow = -1.0
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
        if self._player == None:
            self.cache_nodes()
            if self._player == None:
                return
        self._phys_prev = self.global_position
        if self._build >= 0.0:
            self._build = self._build - delta
            if self._build < 0.0:
                self.appear()
            self._phys_cur = self.global_position
            return
        self.step(delta)
        self._phys_cur = self.global_position

    def step(self, delta):
        self._cool = math.max(self._cool - delta, 0.0)
        self._anger = math.max(self._anger - delta * 0.6, 0.0)
        if keyboard_check_pressed(KEY_Q) and self._state == "follow":
            self.command()
        move = Vec3(0.0, 0.0, 0.0)
        if self._state == "rogue":
            move = self.rogue(delta)
        elif self._state == "hunt":
            move = self.hunt(delta)
        elif self._state == "leap":
            move = self.leap(delta)
        else:
            move = self.follow(delta)
        if self.is_on_floor():
            self._vertical = -0.5
        else:
            self._vertical = self._vertical - self.gravity * delta
        self.velocity = Vec3(move.x, self._vertical, move.z)
        self.move_and_slide(delta)
        self.rotation_degrees = Vec3(0.0, self._yaw, 0.0)
        self.update_buzz(delta, move.x * move.x + move.z * move.z > 0.01)

    # The servo hum only while it moves, cut half a second after it stops.
    def update_buzz(self, delta, moving):
        if self._buzz == None:
            return
        if moving:
            self._still = 0.0
            if not self._buzz.is_playing():
                self._buzz.play()
        else:
            self._still = self._still + delta
            if self._still > 0.5 and self._buzz.is_playing():
                self._buzz.stop()

    # Q: the cop under the crosshair, else the nearest one in front of her within reach.
    def command(self):
        if self._cool > 0.0:
            return
        target = self._player.aimed_enemy()
        if target == None:
            target = self.nearest_enemy()
        if target == None:
            self.say("Assets/Sounds/spider/scream01.ogg", 0.6)
            return
        self._target = target
        self._state = "hunt"
        self._path = None
        self.say("Assets/Sounds/spider/sight.ogg", 1.0)
        if self._game != None:
            self._game.say("spider", 0.7)

    def nearest_enemy(self):
        if self._game == None or self._game.enemies == None:
            return None
        p = self._player.global_position
        f = self._player.forward_flat()
        best = None
        best_dist = self.hunt_range
        for enemy in self._game.enemies:
            e = enemy.global_position
            dx = e.x - p.x
            dz = e.z - p.z
            dist = math.sqrt(dx * dx + dz * dz)
            if dist < best_dist and dist > 0.01 and (f.x * dx + f.z * dz) / dist > 0.2:
                best = enemy
                best_dist = dist
        return best

    # Settled, it stays put while she turns or stands: it only walks again once she has gone
    # leash metres from where it settled.
    def follow(self, delta):
        p = self.global_position
        player = self._player
        q = player.global_position
        if self._settled and self._anchor != None:
            ax = q.x - self._anchor.x
            az = q.z - self._anchor.z
            if ax * ax + az * az < self.leash * self.leash and math.abs(q.y - self._anchor.y) < 1.5:
                self.play("Idle", 1.0)
                return Vec3(0.0, 0.0, 0.0)
            self._settled = False
        # Close to her, from whichever side it is on: it only closes the gap.
        gx = p.x - q.x
        gz = p.z - q.z
        gap = math.max(math.sqrt(gx * gx + gz * gz), 0.01)
        goal = Vec3(q.x + gx / gap * self.follow_distance, q.y, q.z + gz / gap * self.follow_distance)
        dx = goal.x - p.x
        dz = goal.z - p.z
        dist = math.sqrt(dx * dx + dz * dz)
        if dist < 0.5:
            self._settled = True
            self._anchor = Vec3(q.x, q.y, q.z)
            self.play("Idle", 1.0)
            return Vec3(0.0, 0.0, 0.0)
        speed = self.run_speed if dist > 6.0 else self.walk_speed * math.clamp(dist / 1.5, 0.4, 1.0)
        return self.walk_towards(goal, speed, delta)

    def hunt(self, delta):
        target = self._target
        if target == None or target._dead:
            self.done()
            return Vec3(0.0, 0.0, 0.0)
        p = self.global_position
        t = target.global_position
        dx = t.x - p.x
        dz = t.z - p.z
        dist = math.sqrt(dx * dx + dz * dz)
        if dist <= self.leap_range:
            self._state = "leap"
            self._leap = 0.0
            self._bitten = False
            self.play("JumpAttack", 1.3)
            self.say("Assets/Sounds/spider/jumpattack0" + str(int(math.random(1.0, 2.99))) + ".ogg", 1.0)
            return Vec3(0.0, 0.0, 0.0)
        return self.walk_towards(t, self.run_speed, delta)

    def leap(self, delta):
        self._leap = self._leap + delta
        target = self._target
        p = self.global_position
        move = Vec3(0.0, 0.0, 0.0)
        if target != None and not target._dead:
            t = target.global_position
            dx = t.x - p.x
            dz = t.z - p.z
            dist = math.max(math.sqrt(dx * dx + dz * dz), 0.01)
            self.turn_to(dx, dz, delta)
            if dist > 0.7:
                move = Vec3(dx / dist * 6.0, 0.0, dz / dist * 6.0)
            if not self._bitten and self._leap >= self.bite_at:
                self._bitten = True
                self.say("Assets/Sounds/spider/attackscream0" + str(int(math.random(1.0, 2.99))) + ".ogg", 1.0)
                if target == self._player:
                    if dist < 1.6:
                        target.take_damage(self.bite_damage, p, Vec3(dx / dist, 0.2, dz / dist), 0.5, self)
                else:
                    target.spider_kill = True
                    target.dismember(Vec3(dx / dist, 0.3, dz / dist), 4.5)
        if self._leap >= self.leap_time:
            if target == self._player:
                self.calm()
            else:
                self.done()
        return move

    # ---- shots: source is who fired; only hers make it angry

    def take_damage(self, amount, origin, direction, force, source=None):
        if self._wrecked or self._build >= 0.0:
            return
        if source == self._player:
            amount = amount * self.friendly_fire
        self._health = self._health - amount
        if self._health <= 0.0:
            self.wreck()
            return
        if source != self._player:
            self.say("Assets/Sounds/spider/scream01.ogg", 0.7)
            return
        self._anger = self._anger + 1.0
        if self._game != None:
            self._game._glitch = math.max(self._game._glitch, 0.2)
        if self._state != "rogue" and self._state != "leap" and self._anger >= self.rogue_hits:
            self.go_rogue()
            return
        self.say("Assets/Sounds/spider/scream01.ogg", 0.8)

    def alive(self):
        return not self._wrecked and self._build < 0.0

    def go_rogue(self):
        self._state = "rogue"
        self._rogue = self.rogue_time
        self._target = self._player
        self._path = None
        self.say("Assets/Sounds/spider/sight.ogg", 1.0)
        if self._game != None:
            self._game.show("SPIDER // HOSTILE", 2.0)
            self._game._glitch = 0.7

    # Runs at her and leaps; leap() bites her once and calm() ends it.
    def rogue(self, delta):
        self._rogue = self._rogue - delta
        player = self._player
        if self._rogue <= 0.0 or player == None or not player.alive():
            self.calm()
            return Vec3(0.0, 0.0, 0.0)
        p = self.global_position
        t = player.global_position
        dx = t.x - p.x
        dz = t.z - p.z
        if math.sqrt(dx * dx + dz * dz) <= self.leap_range:
            self._state = "leap"
            self._leap = 0.0
            self._bitten = False
            self.play("JumpAttack", 1.3)
            self.say("Assets/Sounds/spider/jumpattack0" + str(int(math.random(1.0, 2.99))) + ".ogg", 1.0)
            return Vec3(0.0, 0.0, 0.0)
        return self.walk_towards(t, self.run_speed * 1.15, delta)

    def calm(self):
        self._rogue = 0.0
        self._anger = 0.0
        self.done()
        if self._game != None:
            self._game.show("SPIDER // RECALIBRATED", 2.0)

    # Caught in a blast: the body is lost, the core goes back to the android (game.spider_lost).
    def wreck(self):
        if self._wrecked:
            return
        self._wrecked = True
        # Blown up while still being built: the build effect must not call a freed body.
        if self._fx != None:
            self._fx.body = None
            self._fx = None
        if self._game != None:
            fx = self._game.instantiate("Assets/Prefabs/impact.scene.json")
            if fx != None:
                fx.global_position = self.global_position
                fx.burst(Vec3(0.0, 1.0, 0.0), 3.0)
            self._game.spider_lost()
        self.queue_free()

    def done(self):
        self._target = None
        self._settled = False
        self._state = "follow"
        self._cool = self.cooldown
        self._path = None

    # Along the walkable floor when there is one, straight otherwise.
    def walk_towards(self, goal, speed, delta):
        p = self.global_position
        target = goal
        nav = self._game.nav if self._game != None else None
        if nav != None and nav.is_baked():
            self._repath = self._repath - delta
            moved = self._path_goal == None or (goal.x - self._path_goal.x) * (goal.x - self._path_goal.x) + (goal.z - self._path_goal.z) * (goal.z - self._path_goal.z) > 1.0
            if (self._path == None or self._repath <= 0.0 or moved) and self._game.path_ticket():
                self._repath = 0.4
                self._path_goal = goal
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
        self.turn_to(dx, dz, delta)
        self.play("Walk", self.walk_clip_speed * speed / self.walk_speed)
        self._step = self._step - delta * speed / self.walk_speed
        if self._step <= 0.0 and self._steps != None:
            self._step = 0.32
            self._steps.pitch_scale = math.random(0.85, 1.2)
            self._steps.play()
        return Vec3(dx / length * speed, 0.0, dz / length * speed)

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
