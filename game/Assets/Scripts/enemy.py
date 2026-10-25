import math

class Enemy(CharacterBody3D):
    health = 100.0
    walk_speed = 1.6
    run_speed = 3.6
    turn_speed = 360.0
    gravity = 18.0
    sight_range = 22.0
    sight_angle = 75.0
    hear_range = 7.0  # this close she is noticed even behind his back (steps, a breath)
    straight_range = 15.0  # in sight this close he runs straight at her, no path asked
    shoot_range = 9.0
    damage = 9.0
    spread = 0.06
    head_line = 0.55
    model_yaw = 180.0
    energy_drop = 0.35
    corpse_time = 10.0
    anim_dir = "Assets/Animations/cop/"
    grip_yaw = 180.0
    grip_roll = 90.0
    grip_x = 0.08
    grip_y = -0.015
    _state = "guard"
    _health = 100.0
    _yaw = 0.0
    _vertical = 0.0
    _anim = None
    _clip = ""
    _player = None
    _think = 0.0
    _shot = 0.0
    _burst = 0
    _pause = 0.0
    _react = 0.0
    _alert = 0.0
    _last_seen = None
    _dead = False
    _voice = 1
    _push_x = 0.0
    _push_z = 0.0
    _muzzle = None
    _skeleton = None
    _pelvis = None
    _pool = 0.0
    _kill_kind = ""
    _phys_prev = None
    _phys_cur = None
    _model_base = None
    _model = None
    _corpse = 0.0
    _game = None
    spider_kill = False
    idle_clip = "Idle_Bored"
    seated = False
    civilian = False
    scared_clip = "Custom_Vinnie_Scared"
    carries_card = False
    _path = None
    _path_index = 0
    _repath = 0.0
    _clear = False
    _stuck_t = 0.0
    _stuck_from = None
    _side_t = 0.0
    _side_dir = None

    def _ready(self):
        self._health = self.health
        self._yaw = self.rotation_degrees.y
        self._last_seen = self.global_position
        self._voice = int(math.random(1.0, 6.99))
        self._think = math.random(0.0, 0.5)
        self._game = self.get_tree().get_first_node_in_group("game")
        skeleton = self.find_child("Skeleton3D", True)
        model = self.get_node("Model")
        self._phys_prev = self.global_position
        self._phys_cur = self.global_position
        if model != None:
            model.rotation_degrees = Vec3(0.0, self.model_yaw, 0.0)
            self._model = model
            self._model_base = model.position
        if skeleton == None:
            return
        anim = skeleton.find_child("AnimationPlayer", False)
        if anim == None:
            anim = skeleton.create_child("AnimationPlayer")
            anim.name = "AnimationPlayer"
        self._anim = anim
        self._skeleton = skeleton
        for name in ["AI_Shoot_Beretta_Forward", "Custom_Aim_Pistol", "Reload_Beretta", "Idle_Guard_Alarmed", "Run_Forward", "Normal_Walk_Forward", "GetHit_Medium", "Death_Fire_A", "React_Scared"]:
            if not anim.has_animation(name):
                anim.import_animations(self.anim_dir + name + ".glb", "")
        for name in ["Idle_Guard_Alarmed", "Run_Forward", "Normal_Walk_Forward", "Custom_Aim_Pistol", "Idle_Bored"]:
            if anim.has_animation(name):
                anim.set_animation_loop(name, True)
        anim.playback_default_blend_time = 0.15
        attach = skeleton.create_child("BoneAttachment3D")
        attach.name = "WeaponBone"
        attach.bone_name = "Bip01 R Hand"
        grip = attach.create_child("Node3D")
        grip.name = "Grip"
        grip.rotation_degrees = Vec3(self.grip_roll, self.grip_yaw, 0.0)
        gun = grip.instantiate("Assets/Models/Weapons/pistol.glb")
        if gun != None:
            gun.position = Vec3(self.grip_x, self.grip_y, 0.0)
        self._muzzle = grip.instantiate("Assets/Prefabs/muzzle_flash.scene.json")
        if self._muzzle != None:
            self._muzzle.position = Vec3(self.grip_x + 0.215, self.grip_y + 0.055, 0.0)
        self.link_ragdoll(skeleton, anim)
        if anim.has_animation(self.idle_clip):
            anim.set_animation_loop(self.idle_clip, True)
        self.play(self.idle_clip)

    def link_ragdoll(self, skeleton, anim):
        ragdoll = self.get_node("Ragdoll")
        if ragdoll == None:
            return
        ragdoll.skeleton = ragdoll.get_path_to(skeleton)
        ragdoll.animation_player = ragdoll.get_path_to(anim)
        ragdoll.set_bone_name(RAGDOLL_BONE_HIPS, "Bip01 Pelvis")
        ragdoll.set_bone_name(RAGDOLL_BONE_SPINE, "Bip01 Spine1")
        ragdoll.set_bone_name(RAGDOLL_BONE_NECK, "Bip01 Neck")
        ragdoll.set_bone_name(RAGDOLL_BONE_HEAD, "Bip01 Head")
        ragdoll.set_bone_name(RAGDOLL_BONE_LEFT_ARM, "Bip01 L UpperArm")
        ragdoll.set_bone_name(RAGDOLL_BONE_LEFT_FORE_ARM, "Bip01 L Forearm")
        ragdoll.set_bone_name(RAGDOLL_BONE_LEFT_HAND, "Bip01 L Hand")
        ragdoll.set_bone_name(RAGDOLL_BONE_RIGHT_ARM, "Bip01 R UpperArm")
        ragdoll.set_bone_name(RAGDOLL_BONE_RIGHT_FORE_ARM, "Bip01 R Forearm")
        ragdoll.set_bone_name(RAGDOLL_BONE_RIGHT_HAND, "Bip01 R Hand")
        ragdoll.set_bone_name(RAGDOLL_BONE_LEFT_UP_LEG, "Bip01 L Thigh")
        ragdoll.set_bone_name(RAGDOLL_BONE_LEFT_LEG, "Bip01 L Calf")
        ragdoll.set_bone_name(RAGDOLL_BONE_LEFT_FOOT, "Bip01 L Foot")
        ragdoll.set_bone_name(RAGDOLL_BONE_RIGHT_UP_LEG, "Bip01 R Thigh")
        ragdoll.set_bone_name(RAGDOLL_BONE_RIGHT_LEG, "Bip01 R Calf")
        ragdoll.set_bone_name(RAGDOLL_BONE_RIGHT_FOOT, "Bip01 R Foot")

    def play(self, clip):
        if clip == self._clip or self._anim == None:
            return
        if not self._anim.has_animation(clip):
            return
        self._clip = clip
        self._anim.play(clip)

    def play_once(self, clip):
        if self._anim == None or not self._anim.has_animation(clip):
            return 0.0
        self._clip = clip
        self._anim.play(clip)
        return self._anim.get_current_animation_length()

    def alive(self):
        return not self._dead

    def player(self):
        if self._player == None:
            self._player = self.get_tree().get_first_node_in_group("player")
        return self._player

    # The android, or Unit-7 once it is out and nearer to this cop.
    # The nearest of her, Unit-7 once free and her spider.
    def pick_target(self):
        player = self.player()
        if player == None:
            return None
        story = self._game.story if self._game != None else None
        p = self.global_position
        best = player
        a = player.global_position
        best_d = (a.x - p.x) * (a.x - p.x) + (a.z - p.z) * (a.z - p.z)
        unit = story._unit if story != None and story.unit_free else None
        spider = self._game.spider if self._game != None else None
        for other in [unit, spider]:
            if other == None or not other.alive():
                continue
            b = other.global_position
            d = (b.x - p.x) * (b.x - p.x) + (b.z - p.z) * (b.z - p.z)
            if d < best_d:
                best = other
                best_d = d
        return best

    def eye(self):
        p = self.global_position
        return Vec3(p.x, p.y + 0.6, p.z)

    def sees(self, target):
        if self.civilian or (self._game != None and self._game.briefing()):
            return False
        p = self.global_position
        t = target.global_position
        dx = t.x - p.x
        dz = t.z - p.z
        dist = math.sqrt(dx * dx + dz * dz)
        if dist > self.sight_range:
            return False
        if dist > self.hear_range and self._state == "guard":
            f = self.forward()
            cosine = (f.x * dx + f.z * dz) / dist
            if cosine < math.cos(math.radians(self.sight_angle)):
                return False
        hit = raycast(self.eye(), Vec3(t.x, t.y + 0.4, t.z), None, self)
        return hit != None and hit.node == target

    def face(self, target, delta):
        p = self.global_position
        wanted = math.degrees(math.atan2(-(target.x - p.x), -(target.z - p.z)))
        diff = wanted - self._yaw
        while diff > 180.0:
            diff = diff - 360.0
        while diff < -180.0:
            diff = diff + 360.0
        step = self.turn_speed * delta
        if math.abs(diff) <= step:
            self._yaw = wanted
        else:
            self._yaw = self._yaw + (step if diff > 0.0 else -step)
        self.rotation_degrees = Vec3(0.0, self._yaw, 0.0)
        return math.abs(diff) < 10.0

    def _process(self, delta):
        if self._model == None or self._dead:
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
        self._phys_prev = self.global_position
        self.physics_step(delta)
        self._phys_cur = self.global_position

    # Gunfire or a body dropping nearby: the cop turns on that spot; a civilian cowers.
    def alert_to(self, position):
        if self._dead:
            return
        if self.civilian:
            clip = self.scared_clip
            if self._clip != clip and self._anim != None and self._anim.has_animation(clip):
                self._anim.set_animation_loop(clip, True)
                self.play(clip)
                self.speak("pain")
            return
        self._last_seen = Vec3(position.x, position.y, position.z)
        self._alert = math.max(self._alert, 6.0)
        if self._state == "guard":
            self._state = "alert"
            self.seated = False
            self._react = self.play_once("React_Scared") * 0.6
            self.speak("pain")

    def physics_step(self, delta):
        if self._dead:
            self.update_pool(delta)
            self._corpse = self._corpse + delta
            if self._corpse > self.corpse_time:
                self.queue_free()
            return
        # Scenery until something happens: a civilian never moves, a seated cop stays on his
        # chair (no slide, so the chair does not push him out) until alerted.
        if self.civilian:
            return
        if self.seated and self._state == "guard":
            self._think = self._think - delta
            if self._think <= 0.0:
                self._think = 0.2
                target = self.pick_target()
                if target != None and target.alive() and self.sees(target):
                    self.alert_to(target.global_position)
            return
        target = self.pick_target()
        move = Vec3(0.0, 0.0, 0.0)
        self._think = self._think - delta
        if self._think <= 0.0:
            self._think = 0.2
            self._clear = target != None and target.alive() and self.sees(target)
            if self._clear:
                self._last_seen = target.global_position
                self._alert = 6.0
                if self._state == "guard":
                    self._state = "alert"
                    self._react = self.play_once("React_Scared") * 0.6
                    self.speak("pain")
        self._alert = math.max(self._alert - delta, 0.0)
        self._react = math.max(self._react - delta, 0.0)
        self._shot = math.max(self._shot - delta, 0.0)
        chasing = False
        if self._state == "alert" and target != None and self._react <= 0.0:
            p = self.global_position
            seen = self._last_seen
            dx = seen.x - p.x
            dz = seen.z - p.z
            dist = math.sqrt(dx * dx + dz * dz)
            shooting = self._alert > 0.0 and dist <= self.shoot_range and target.alive()
            # While sidestepping round an obstacle the cop looks where it goes, so the running
            # animation does not make it crab sideways; shooting still turns it on the target.
            aim = seen
            if self._side_t > 0.0 and not shooting:
                aim = Vec3(p.x + self._side_dir.x, p.y, p.z + self._side_dir.z)
            facing = self.face(aim, delta)
            if shooting:
                if facing:
                    self.shoot(target, delta)
            elif dist > 1.2:
                chasing = True
                move = self.steer(seen, delta)
                self.play("Run_Forward")
            else:
                self.play("Idle_Guard_Alarmed")
                if self._alert <= 0.0:
                    self._state = "guard"
        if not chasing:
            # Not trying to run any more (shooting, idle, reacting): the stuck check starts over.
            self._stuck_from = None
            self._side_t = 0.0
        if self.is_on_floor():
            self._vertical = -0.5
        else:
            self._vertical = self._vertical - self.gravity * delta
        self._push_x = self._push_x * math.max(1.0 - delta * 6.0, 0.0)
        self._push_z = self._push_z * math.max(1.0 - delta * 6.0, 0.0)
        # Kept apart from the others only while running (it looks at every cop of the night).
        apart = self.spacing() if chasing else Vec3(0.0, 0.0, 0.0)
        vx = move.x + self._push_x + apart.x
        vz = move.z + self._push_z + apart.z
        # Standing still on the floor: no slide at all. Each one sweeps the capsule through the
        # level's triangles, and most of the cops of the night just stand there.
        if not chasing and vx * vx + vz * vz < 0.01 and self.is_on_floor():
            return
        self.velocity = Vec3(vx, self._vertical, vz)
        self.move_and_slide(delta)

    # Running velocity towards a goal, with local unsticking. Barrels and crates are rigid bodies
    # outside the navigation, so a path can run straight through one and the cop would push
    # against it for ever. Twice a second the distance covered is measured; a cop that wanted to
    # run but moved less than a quarter of a metre sidesteps for a moment, then gets a new path.
    def steer(self, goal, delta):
        if self._side_t > 0.0:
            self._side_t = self._side_t - delta
            if self._side_t <= 0.0:
                # The old path ran into the obstacle, so it is dropped and asked for again.
                self._path = None
                self._stuck_from = None
            s = self.walk_speed
            return Vec3(self._side_dir.x * s, 0.0, self._side_dir.z * s)
        way = self.chase_dir(goal, delta)
        speed = self.run_speed
        move = Vec3(way.x * speed, 0.0, way.z * speed)
        p = self.global_position
        # A cop that is not really trying to move cannot be stuck.
        if speed * math.sqrt(way.x * way.x + way.z * way.z) < 0.5:
            self._stuck_from = None
            return move
        if self._stuck_from == None:
            self._stuck_from = Vec3(p.x, p.y, p.z)
            self._stuck_t = 0.0
            return move
        self._stuck_t = self._stuck_t + delta
        if self._stuck_t < 0.5:
            return move
        mx = p.x - self._stuck_from.x
        mz = p.z - self._stuck_from.z
        self._stuck_from = Vec3(p.x, p.y, p.z)
        self._stuck_t = 0.0
        if mx * mx + mz * mz >= 0.25 * 0.25:
            return move
        return self.start_sidestep(way)

    # Steps aside for 0.6 s at walking speed, perpendicular to where the cop wanted to go, on
    # the side with no solid body. Each side is probed once, here, and not every frame.
    def start_sidestep(self, way):
        p = self.global_position
        lx = -way.z
        lz = way.x
        left_blocked = self.blocked(Vec3(p.x + lx * 0.8, p.y - 0.3, p.z + lz * 0.8), 0.28)
        right_blocked = self.blocked(Vec3(p.x - lx * 0.8, p.y - 0.3, p.z - lz * 0.8), 0.28)
        # Both sides clear (or both blocked) is a coin toss, so two cops do not always agree.
        side = 1.0 if math.random(0.0, 1.0) < 0.5 else -1.0
        if left_blocked and not right_blocked:
            side = -1.0
        elif right_blocked and not left_blocked:
            side = 1.0
        self._side_dir = Vec3(lx * side, 0.0, lz * side)
        self._side_t = 0.6
        s = self.walk_speed
        return Vec3(self._side_dir.x * s, 0.0, self._side_dir.z * s)

    # Direction towards a goal along the walkable floor: the corners of a path from the
    # navigation, asked again twice a second (a far goal every 1.2 s), when the game hands out
    # this frame's path ticket; straight at the goal when it is in sight and close, or without
    # navigation.
    def chase_dir(self, goal, delta):
        p = self.global_position
        target = goal
        nav = self._game.nav if self._game != None else None
        near = (goal.x - p.x) * (goal.x - p.x) + (goal.z - p.z) * (goal.z - p.z) < self.straight_range * self.straight_range
        if self._clear and near and math.abs(goal.y - p.y) < 1.5:
            # She is in plain view and close: straight at her (the stuck check still sidesteps
            # a table in the way), and the old path is dropped.
            self._path = None
        elif nav != None and nav.is_baked():
            self._repath = self._repath - delta
            if (self._path == None or self._repath <= 0.0) and self._game.path_ticket():
                self._repath = 0.5 if (goal.x - p.x) * (goal.x - p.x) + (goal.z - p.z) * (goal.z - p.z) < 225.0 else 1.2
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

    # Push away from the other cops closer than a metre and from the player closer than 0.8 m,
    # so bodies do not walk into one another.
    def spacing(self):
        if self._game == None or self._game.enemies == None:
            return Vec3(0.0, 0.0, 0.0)
        p = self.global_position
        sx = 0.0
        sz = 0.0
        for other in self._game.enemies:
            if other == self:
                continue
            push = self.away_from(p, other.global_position, 1.0)
            sx = sx + push.x
            sz = sz + push.z
        player = self.player()
        if player != None and player.alive():
            push = self.away_from(p, player.global_position, 0.8)
            sx = sx + push.x * 2.0
            sz = sz + push.z * 2.0
        return Vec3(sx * 3.0, 0.0, sz * 3.0)

    def away_from(self, p, o, radius):
        dx = p.x - o.x
        dz = p.z - o.z
        d2 = dx * dx + dz * dz
        if d2 >= radius * radius or math.abs(o.y - p.y) > 1.5:
            return Vec3(0.0, 0.0, 0.0)
        d = math.sqrt(d2)
        if d < 0.001:
            angle = math.random(0.0, 6.283)
            return Vec3(math.cos(angle), 0.0, math.sin(angle))
        f = (radius - d) / radius
        return Vec3(dx / d * f, 0.0, dz / d * f)

    def shoot(self, target, delta):
        if self._pause > 0.0:
            self._pause = self._pause - delta
            self.play("Custom_Aim_Pistol")
            return
        if self._shot > 0.0:
            return
        if self._burst <= 0:
            self._burst = int(math.random(2.0, 4.99))
        self._burst = self._burst - 1
        self._shot = 0.28
        if self._burst <= 0:
            self._pause = math.random(0.8, 1.6)
        self._clip = ""
        self.play("AI_Shoot_Beretta_Forward")
        self.get_node("Shot").play()
        if self._muzzle != None:
            self._muzzle.fire()
        start = self.eye()
        aim = target.global_position
        aim = Vec3(aim.x + math.random(-self.spread, self.spread) * 4.0, aim.y + 0.2 + math.random(-self.spread, self.spread) * 3.0, aim.z + math.random(-self.spread, self.spread) * 4.0)
        direction = Vec3(aim.x - start.x, aim.y - start.y, aim.z - start.z)
        length = math.sqrt(direction.x * direction.x + direction.y * direction.y + direction.z * direction.z)
        if length < 0.01:
            return
        direction = Vec3(direction.x / length, direction.y / length, direction.z / length)
        end = Vec3(start.x + direction.x * 60.0, start.y + direction.y * 60.0, start.z + direction.z * 60.0)
        hit = raycast(start, end, None, self)
        if hit != None and hit.node == target:
            target.take_damage(self.damage, hit.position, direction, 0.3, self)
        elif hit != None and hit.node != None and (hit.node.is_in_group("unit7") or hit.node.is_in_group("spider")):
            hit.node.take_damage(self.damage, hit.position, direction, 0.3, self)
        elif hit != None and hit.node != None and hit.node.is_in_group("breakable"):
            hit.node.get_node("Breakable").hit(hit.position, hit.normal, direction, self.damage)

    def speak(self, kind):
        voice = self.get_node("Voice")
        if voice == None or voice.is_playing():
            return
        n = int(math.random(1.0, 6.99))
        voice.stream = "Assets/Sounds/cop/male_" + kind + "_0" + str(n) + ".wav"
        voice.play()

    def is_headshot(self, point):
        return point.y > self.global_position.y + self.head_line

    def headshot(self, point, direction):
        if self._dead:
            return
        self.bleed(point, direction, 2.0)
        self._kill_kind = "HEADSHOT"
        self.die(direction, 3.0)

    def take_damage(self, amount, origin, direction, force, source=None):
        if self._dead:
            return
        self._health = self._health - amount
        self.bleed(origin, direction, 1.0)
        self._state = "alert"
        self._alert = 6.0
        player = self.player()
        if player != None:
            self._last_seen = player.global_position
        if self._health <= 0.0:
            if force >= 1.0:
                self._kill_kind = "MELEE"
            self.die(direction, force)
            return
        if force >= 1.0:
            self._push_x = direction.x * force * 3.0
            self._push_z = direction.z * force * 3.0
            self._react = self.play_once("GetHit_Large") * 0.9
        else:
            self._react = self.play_once("GetHit_Medium") * 0.8
        self.speak("pain")

    def die(self, direction, force):
        if self.civilian:
            self._kill_kind = "INNOCENT"
        self._dead = True
        self.remove_from_group("enemy")
        self.remove_from_group("damageable")
        self.add_to_group("corpse")
        self.collision_layer = 0
        self.collision_mask = 0
        self.speak("death")
        game = self.get_tree().get_first_node_in_group("game")
        if game != None:
            game.enemy_killed(self)
        ragdoll = self.get_node("Ragdoll")
        if ragdoll != None and ragdoll.start():
            ragdoll.apply_impulse(RAGDOLL_PART_SPINE, Vec3(direction.x * force * 2.5, 1.0 + force * 0.6, direction.z * force * 2.5), Vec3(0.0, 0.0, 0.0))
        else:
            self.play_once("Death_Fire_A")
        self._pelvis = self.track("Bip01 Pelvis")
        self._pool = 1.2
        self.drop("Assets/Prefabs/pickup_ammo.scene.json", 0.0)
        if self.carries_card:
            self.drop("Assets/Prefabs/pickup_keycard.scene.json", 0.3)
        if math.random(0.0, 1.0) < self.energy_drop:
            self.drop("Assets/Prefabs/pickup_energy.scene.json", 0.45)

    def drop(self, path, side):
        spot = self.free_spot(0.5 + side)
        if spot == None:
            return
        item = self.effects_parent().instantiate(path)
        if item != None:
            item.global_position = spot

    # A floor spot near the body with room for an item: on the walkable surface, seen from the
    # body (no wall between) and with nothing solid in a sphere above it.
    def free_spot(self, distance):
        game = self.effects_parent()
        nav = game.nav if game != None and game.nav != None and game.nav.is_baked() else None
        p = self.global_position
        for i in range(8):
            angle = math.random(0.0, 6.283)
            r = distance + i * 0.1
            spot = Vec3(p.x + math.cos(angle) * r, p.y, p.z + math.sin(angle) * r)
            spot = self.floor_at(spot, nav)
            if spot == None:
                continue
            if math.abs(spot.y - (p.y - 0.85)) > 0.6:
                continue
            seen = raycast(Vec3(p.x, p.y, p.z), Vec3(spot.x, spot.y + 0.3, spot.z), None, self)
            if seen != None and seen.node != None and not self.ignorable(seen.node):
                continue
            if self.blocked(Vec3(spot.x, spot.y + 0.35, spot.z), 0.22):
                continue
            return spot
        return self.floor_at(p, nav)

    # The walkable floor nearest a point, or the floor straight below it where the navigation
    # does not reach (a room cut off from the start).
    def floor_at(self, point, nav):
        if nav != None:
            spot = nav.nearest_point(point)
            if spot != None:
                return spot
        hit = raycast(Vec3(point.x, point.y, point.z), Vec3(point.x, point.y - 2.5, point.z), None, self)
        if hit == None or hit.node == None:
            return None
        return hit.position

    def blocked(self, center, radius):
        for body in overlap_sphere(center, radius):
            if not self.ignorable(body):
                return True
        return False

    def ignorable(self, node):
        return node == self or node.is_in_group("enemy") or node.is_in_group("corpse") or node.is_in_group("player") or node.is_in_group("pickup")

    def effects_parent(self):
        scene = self.get_tree().get_first_node_in_group("game")
        if scene == None:
            scene = self.get_tree().get_current_scene()
        if scene == None:
            scene = self.get_parent()
        return scene

    def track(self, bone):
        if self._skeleton == None:
            return None
        attach = self._skeleton.create_child("BoneAttachment3D")
        attach.bone_name = bone
        return attach

    def bleed(self, point, direction, strength):
        fx = self.effects_parent().instantiate("Assets/Prefabs/blood_burst.scene.json")
        if fx != None:
            fx.global_position = point
            fx.burst(direction, strength)

    def update_pool(self, delta):
        if self._pool <= 0.0 or self._pelvis == None:
            return
        self._pool = self._pool - delta
        if self._pool > 0.0:
            return
        p = self._pelvis.global_position
        y = p.y - 0.12
        top = p.y + 0.2
        tries = 0
        while tries < 6:
            tries = tries + 1
            hit = raycast(Vec3(p.x, top, p.z), Vec3(p.x, p.y - 1.5, p.z), None, self)
            if hit == None:
                break
            if hit.node != None:
                if hit.normal.y > 0.7:
                    y = hit.position.y
                break
            top = hit.position.y - 0.02
        decal = self.effects_parent().create_child("Decal3D", "Assets/Scripts/blood_pool.py")
        decal.name = "BloodPool"
        decal.texture_albedo = "Assets/Textures/Fx/blood_pool.png"
        decal.size = Vec3(0.3, 0.4, 0.3)
        decal.modulate = Color(1.0, 0.3, 0.25, 1.0)
        decal.global_position = Vec3(p.x, y, p.z)
        decal.rotation_degrees = Vec3(0.0, math.random(0.0, 360.0), 0.0)

    def dismember(self, direction, force):
        if self._dead:
            return
        self._kill_kind = "SPIDER" if self.spider_kill else "DISMEMBERED"
        self.die(direction, force)
        cutter = self.get_node("Dismember")
        ragdoll = self.get_node("Ragdoll")
        if cutter == None or ragdoll == None:
            return
        parts = [RAGDOLL_PART_HEAD, RAGDOLL_PART_LEFT_UPPER_ARM, RAGDOLL_PART_RIGHT_UPPER_ARM, RAGDOLL_PART_LEFT_LOWER_ARM, RAGDOLL_PART_RIGHT_LOWER_ARM, RAGDOLL_PART_LEFT_UPPER_LEG, RAGDOLL_PART_RIGHT_UPPER_LEG]
        bones = ["Bip01 Head", "Bip01 L UpperArm", "Bip01 R UpperArm", "Bip01 L Forearm", "Bip01 R Forearm", "Bip01 L Thigh", "Bip01 R Thigh"]
        stumps = ["Bip01 Neck", "Bip01 L Clavicle", "Bip01 R Clavicle", "", "", "", ""]
        first = 0
        if math.random(0.0, 1.0) > 0.45:
            first = int(math.random(1.0, 6.99))
        picks = [first]
        if math.random(0.0, 1.0) < 0.4:
            second = int(math.random(1.0, 6.99))
            if second != first:
                picks.append(second)
        for i in picks:
            if not cutter.detach(parts[i]):
                continue
            piece = self.track(bones[i])
            if piece != None:
                piece.instantiate("Assets/Prefabs/blood_trail.scene.json")
            if stumps[i] != "":
                stump = self.track(stumps[i])
                if stump != None:
                    stump.instantiate("Assets/Prefabs/blood_trail.scene.json")
            ragdoll.apply_impulse(parts[i], Vec3(direction.x * force * 1.2, 1.2 + force * 0.4, direction.z * force * 1.2), Vec3(0.0, 0.0, 0.0))
