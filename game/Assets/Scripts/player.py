import math

HOLES = {"metal": "hole_metal.png", "wood": "hole_wood.png", "tile": "hole_tile.png", "glass": "hole_glass.png", "cardboard": "hole_cardboard.png", "carpet": "hole_default.png", "default": "hole_default.png"}
RUN_DIRECTIONS = ["Forward", "ForwardLeft", "ForwardRight", "Backward", "BackwardLeft", "BackwardRight", "Left", "Right"]
# Lying in a dive, a getup or dead the hips stay this high over her feet' level: some clips
# (the backward and side dives) take them under the floor.
HIPS_FLOOR = 0.18
RUN_SECTORS = ["Backward", "BackwardLeft", "Left", "ForwardLeft", "Forward", "ForwardRight", "Right", "BackwardRight"]

# The HUD images, loaded once (hud_load_image ids) and kept together in one field of the player.
record HudImages { reticle, ring, panel_red, panel_cyan, fill_red, fill_cyan, segment, vignette, hitmark, weapon }

class Player(CharacterBody3D):
    _game = None
    _build = -1.0
    _grow = -1.0
    _model_scale = None
    run_speed = 5.5
    jump_speed = 6.5
    slow_jump = 1.35  # the jump gets this much more push in bullet time
    gravity = 18.0
    sensitivity = 0.15
    pitch_limit = 70.0
    camera_side = 0.55
    camera_height = 0.35
    camera_distance = 2.2
    zoom_distance = 1.0
    zoom_side = 0.42
    zoom_fov = 38.0
    _zoom = 0.0
    force_zoom = False
    _fov = 0.0
    fire_rate = 0.16
    damage = 34.0
    max_health = 300.0
    mag_size = 12
    max_reserve = 120
    overclock_max = 4.0
    overclock_scale = 0.3
    slow_fov = 14.0  # degrees the view widens in bullet time, so the camera seems to step back from her
    shake_frequency = 24.0   # Hz of the trauma shake
    shake_amplitude_x = 0.09  # metres at full trauma
    shake_amplitude_y = 0.06
    shake_roll = 2.5          # degrees at full trauma
    shot_trauma = 0.45        # a shot: shake is the trauma squared, so this is a short 2 cm kick
    _slow_blend = 0.0
    slow_pitch = 0.5
    dodge_speed = 8.5
    dive_twist = 75.0     # degrees the spine turns to the aim during a dive before the body follows
    _dodge_yaw = 0.0
    _body_yaw = 0.0
    _twist = 0.0
    dive_cam_back = 1.1   # metres the camera pulls back during the dive
    _dive_cam = 0.0
    dive_slow_delay = 0.35
    model_yaw = 180.0
    fade_distance = 0.9
    melee_range = 1.9
    kick_range = 3.2
    kick_jump = 4.5
    kick_reach = 0.85
    foot_radius = 0.5
    melee_damage = 45.0
    _yaw = 0.0
    _pitch = 10.0
    _vertical = 0.0
    _health = 100.0
    _cooldown = 0.0
    _kick = 0.0
    _air = 0.0
    _anim = None
    _clip = ""
    _moving = 0.0
    _overclock = 4.0
    _slow = False
    _shots = 0
    _hits = 0
    _hit_timer = 0.0
    _spin = 0.0
    _melee = 0.0
    _melee_hit = 0.0
    _melee_kind = ""
    _melee_target = None
    _lunge = 0.0
    _lunge_dir = None
    _jump_pending = 0.0
    _foot_l = None
    _foot_r = None
    _kick_landed = False
    _img = None
    _hips = None
    _ground_lift = 0.0
    _damage_marks = None
    _real_dt = 0.0
    _target = False
    _near = None
    _dead = False
    _meshes = None
    _fade = -1.0
    _fwd = 0.0
    _side = 0.0
    _upper = None
    _upper_clip = ""
    _bend = None
    _pivot = None
    _camera = None
    _model = None
    _muzzle = None
    _snd_shot = None
    _snd_empty = None
    _snd_melee = None
    _snd_whoosh = None
    _snd_slow_start = None
    _snd_slow_end = None
    _snd_heart = None
    _snd_dive = None
    _snd_reload = None
    _snd_reload_bt = None
    _snd_pickup = None
    _enemies = None
    _enemies_timer = 0.0
    _dodge = 0.0
    _dodge_time = 0.0
    _dodge_dir = None
    _dodge_side = "Forward"
    _getup = 0.0
    _kill_slow = 0.0
    _draining = False
    _exhausted = False
    _mag = 12
    _reserve = 48
    _reload = 0.0
    _reload_clip = ""
    _ammo_flash = 0.0
    _energy_flash = 0.0
    _phys_prev = None
    _phys_cur = None
    _model_base = None
    _pivot_base = None

    def _ready(self):
        self._health = self.max_health
        self._overclock = self.overclock_max
        self._yaw = self.rotation_degrees.y
        self._phys_prev = self.global_position
        self._phys_cur = self.global_position
        self.cache_nodes()
        self._camera.make_current()
        mouse_set_mode(MOUSE_MODE_CAPTURED)

    def cache_nodes(self):
        self._game = self.get_tree().get_first_node_in_group("game")
        self._pivot = self.get_node("Pivot")
        self._camera = self.get_node("Pivot/Camera")
        self._model = self.get_node("Model")
        self._anim = self.get_node("Model/Armature/Skeleton3D/AnimationPlayer")
        self._upper = self.get_node("Model/Armature/Skeleton3D/Upper")
        self._bend = self.get_node("Model/Armature/Skeleton3D/AimBend")
        self._muzzle = self.get_node("Model/Armature/Skeleton3D/RightHand/Grip/Muzzle")
        self._snd_shot = self.get_node("Shot")
        self._snd_empty = self.get_node("Empty")
        self._snd_melee = self.get_node("Melee")
        self._snd_whoosh = self.get_node("Whoosh")
        self._snd_slow_start = self.get_node("SlowStart")
        self._snd_slow_end = self.get_node("SlowEnd")
        self._snd_heart = self.get_node("Heartbeat")
        self._snd_dive = self.get_node("Dive")
        self._snd_reload = self.get_node("Reload")
        self._snd_reload_bt = self.get_node("ReloadBt")
        self._snd_pickup = self.get_node("Pickup")
        self._foot_l = self.foot("FootL", "LeftFoot")
        self._hips = self.foot("HipsBone", "Hips")
        self._foot_r = self.foot("FootR", "RightFoot")
        if self._phys_prev == None:
            self._phys_prev = self.global_position
            self._phys_cur = self.global_position
        if self._anim != None:
            for name in self._anim.get_animation_list():
                self._anim.set_animation_loop(name, False)
            for name in ["Stand", "Fall"]:
                self._anim.set_animation_loop(name, True)
            for name in RUN_DIRECTIONS:
                self._anim.set_animation_loop("Mona_Run_" + name, True)
            self._anim.playback_default_blend_time = 0.15
        if self._upper != None:
            for name in self._upper.get_animation_list():
                self._upper.set_animation_loop(name, name == "Custom_Aim_Pistol_upper")
            self._upper.playback_default_blend_time = 0.1
            self._upper_clip = ""
        self._pivot_base = self._pivot.position
        self._meshes = []
        if self._model != None:
            self._model_base = self._model.position
            self._model.rotation_degrees = Vec3(0.0, self.model_yaw, 0.0)
            self.collect_meshes(self._model)
        hud = "Assets/Textures/Hud/"
        self._img = HudImages(hud_load_image(hud + "reticle.png"), hud_load_image(hud + "ring.png"),
                hud_load_image(hud + "panel_red.png"), hud_load_image(hud + "panel_cyan.png"),
                hud_load_image(hud + "fill_red.png"), hud_load_image(hud + "fill_cyan.png"),
                hud_load_image(hud + "segment.png"), hud_load_image(hud + "vignette.png"),
                hud_load_image(hud + "hitmark.png"), hud_load_image(hud + "weapons/deserteagle.png"))

    def _exit_tree(self):
        mouse_set_mode(MOUSE_MODE_VISIBLE)
        set_time_scale(1.0)
        set_bus_pitch(AUDIO_BUS_SFX, 1.0)

    def camera(self):
        if self._camera == None:
            self.cache_nodes()
        return self._camera

    def alive(self):
        return not self._dead

    def reward(self, amount):
        self._overclock = math.min(self._overclock + amount, self.overclock_max)

    def kill_slow(self, seconds):
        self._kill_slow = math.max(self._kill_slow, seconds)

    # Killed by a blast (a bomb carrier): thrown away from it and down on her front, the end of
    # the forward dive. Only the model turns to the throw, the camera stays where it was.
    def blast_death(self, direction):
        dl = math.max(math.sqrt(direction.x * direction.x + direction.z * direction.z), 0.0001)
        dx = direction.x / dl
        dz = direction.z / dl
        throw = math.degrees(math.atan2(-dx, -dz))
        if self._model != None:
            self._model.rotation_degrees = Vec3(0.0, self.model_yaw + throw - self._yaw, 0.0)
        self._lunge_dir = Vec3(dx, 0.0, dz)
        self._lunge = 7.0
        self._melee_target = None
        self._jump_pending = 4.0
        self._clip = "ShootDodge_Forward"
        self._anim.play(self._clip, 0.1)

    def respawn(self, position, yaw):
        # Back after a death: she gets up off her front where she is rebuilt.
        was_dead = self._dead
        self._dead = False
        self._health = self.max_health
        self._overclock = self.overclock_max
        self._melee = 0.0
        self._dodge = 0.0
        self._getup = 0.0
        self._reload = 0.0
        self._mag = self.mag_size
        self._reserve = int(math.max(self._reserve, 48))
        self._clip = ""
        self._upper_clip = ""
        self._yaw = yaw
        self.global_position = position
        self._phys_prev = position
        self._phys_cur = position
        self.rotation_degrees = Vec3(0.0, yaw, 0.0)
        self._body_yaw = yaw
        self._dodge_yaw = yaw
        self._twist = 0.0
        if self._model != None:
            self._model.rotation_degrees = Vec3(0.0, self.model_yaw, 0.0)
        if was_dead and self._anim != None:
            self._clip = "ShootDodge_Forward_Getup"
            self._anim.play(self._clip, 0.0)
            self._getup = self._anim.get_current_animation_length() * 0.85

    def collect_meshes(self, node):
        if node.is_class("MeshInstance3D"):
            self._meshes.append(node)
        for child in node.get_children():
            self.collect_meshes(child)

    def set_fade(self, fade):
        if fade == self._fade or self._meshes == None:
            return
        self._fade = fade
        for mesh in self._meshes:
            mesh.transparency = fade

    def captured(self):
        return mouse_get_mode() == MOUSE_MODE_CAPTURED

    def play(self, clip):
        if clip == self._clip or self._anim == None:
            return
        if not self._anim.has_animation(clip):
            return
        self._clip = clip
        self._anim.play(clip)

    def forward_flat(self):
        yaw = math.radians(self._yaw)
        return Vec3(-math.sin(yaw), 0.0, -math.cos(yaw))

    # Built by the fx when the night starts (game.start_night): hidden until its burst calls
    # appear() (delay is only a safety net), then she grows out of the flash.
    def materialize(self, delay):
        if self._model == None:
            self.cache_nodes()
        if self._model == None:
            return
        self._build = delay
        self._model_scale = self._model.scale
        self._model.visible = False

    # The build effect's burst: she grows out of it, and the orders follow shortly.
    def appear(self):
        self._build = -1.0
        self._grow = 0.0
        if self._model != None:
            self._model.scale = Vec3(0.01, 0.01, 0.01)
            self._model.visible = True
        if self._game != None and self._game._wake > 0.6:
            self._game._wake = 0.6

    def update_build(self, delta):
        if self._build >= 0.0:
            self._build = self._build - delta
            if self._build < 0.0:
                self.appear()
            return
        self._grow = self._grow + delta
        t = math.clamp(self._grow / 0.4, 0.0, 1.0)
        k = 1.0 + 2.2 * (t - 1.0) * (t - 1.0) * (t - 1.0) + 1.2 * (t - 1.0) * (t - 1.0)
        b = self._model_scale
        self._model.scale = Vec3(b.x * k, b.y * k, b.z * k)
        if t >= 1.0:
            self._grow = -1.0

    def _process(self, delta):
        # The intro covers the screen; she wakes when it ends, built by the fx, with the camera
        # already behind her.
        if self._game != None and self._game.in_intro():
            if self._build >= 0.0 or self._grow >= 0.0:
                self.update_build(delta)
                if self._pivot != None:
                    self.smooth_visuals(self._pivot)
                    self._pivot.rotation_degrees = Vec3(-self._pitch, 0.0, 0.0)
                    self.place_camera(self._pivot, self._camera)
                    self.animate()
            return
        if self._grow >= 0.0:
            self.update_build(delta)
        # ESC: the pause panel (resume, back to menu, exit game).
        if keyboard_check_pressed(KEY_ESCAPE) and self._game != None and self._game.exit_menu != None:
            self._game.exit_menu.open()
            return
        elif mouse_check_button_pressed(MOUSE_LEFT) and not self.captured() and not (self._game != None and self._game._icon_clicked):
            mouse_set_mode(MOUSE_MODE_CAPTURED)
        if self._pivot == None:
            self.cache_nodes()
        camera = self._camera
        pivot = self._pivot
        self._near = None
        self._enemies_timer = self._enemies_timer - delta
        if self._enemies_timer <= 0.0 or self._enemies == None:
            self._enemies_timer = 0.25
            self._enemies = self.get_tree().get_nodes_in_group("enemy")
        if not self._dead:
            self._near = self.melee_candidate(self.melee_range)
        if self.captured() and not self._dead:
            move = mouse_delta()
            # Slower aim while zoomed, for the long shots.
            sensitivity = self.sensitivity * (1.0 - 0.5 * self._zoom)
            self._yaw = self._yaw - move.x * sensitivity
            self._pitch = math.clamp(self._pitch + move.y * sensitivity, -self.pitch_limit, self.pitch_limit)
            if (keyboard_check_pressed(KEY_C) or mouse_check_button_pressed(MOUSE_MIDDLE)) and self._melee <= 0.0 and self._dodge <= 0.0 and self._getup <= 0.0 and self.is_on_floor():
                self.start_dodge()
            if keyboard_check_pressed(KEY_R):
                self.start_reload()
            if self._dodge > 0.0 and mouse_check_button(MOUSE_LEFT):
                self.fire(camera)
            elif self._melee <= 0.0 and self._dodge <= 0.0 and self._getup <= 0.0:
                if mouse_check_button_pressed(MOUSE_LEFT) and self._near != None:
                    self.start_melee("MeleeHit_Forward", self._near, 0.3)
                elif keyboard_check_pressed(KEY_F):
                    self.start_melee("flying_kick", self.melee_candidate(self.kick_range), 0.45)
                elif mouse_check_button(MOUSE_LEFT):
                    self.fire(camera)
        self.turn_body()
        self.update_overclock(delta)
        self.update_melee(delta)
        self.update_dodge(delta)
        real = delta
        scale = time_scale()
        if scale > 0.0:
            real = delta / scale
        self.update_reload(real)
        self._real_dt = real
        self._ammo_flash = math.max(self._ammo_flash - real, 0.0)
        self._energy_flash = math.max(self._energy_flash - real, 0.0)
        self._cooldown = math.max(self._cooldown - real, 0.0)
        self._kick = math.max(self._kick - real * 9.0, 0.0)
        self._hit_timer = math.max(self._hit_timer - real, 0.0)
        self._spin = self._spin + real * 40.0
        self.smooth_visuals(pivot)
        pivot.rotation_degrees = Vec3(-self._pitch + self._kick * 1.5, self._twist, 0.0)
        self.update_zoom(delta, camera)
        self.place_camera(pivot, camera)
        self._target = self.aim_target(camera)
        self.animate()
        if self._game == None or self._game.hud:
            self.draw_hud()

    def aim_target(self, camera):
        start = camera.global_position
        direction = camera.forward()
        end = Vec3(start.x + direction.x * 60.0, start.y + direction.y * 60.0, start.z + direction.z * 60.0)
        hit = raycast(start, end, None, self)
        if hit == None or hit.node == None:
            return False
        return hit.node.is_in_group("enemy")

    # The cop under the crosshair, if any (the spider's order).
    def aimed_enemy(self):
        if self._camera == None:
            return None
        start = self._camera.global_position
        direction = self._camera.forward()
        end = Vec3(start.x + direction.x * 60.0, start.y + direction.y * 60.0, start.z + direction.z * 60.0)
        hit = raycast(start, end, None, self)
        if hit == None or hit.node == None or not hit.node.is_in_group("enemy"):
            return None
        return hit.node

    def melee_candidate(self, reach):
        if self._enemies == None:
            self._enemies = self.get_tree().get_nodes_in_group("enemy")
        best = None
        best_dist = reach
        p = self.global_position
        f = self.forward_flat()
        for enemy in self._enemies:
            e = enemy.global_position
            dx = e.x - p.x
            dz = e.z - p.z
            dist = math.sqrt(dx * dx + dz * dz)
            if dist <= best_dist and dist > 0.01 and math.abs(e.y - p.y) < 1.2:
                if (f.x * dx + f.z * dz) / dist > 0.4:
                    best = enemy
                    best_dist = dist
        return best

    def start_melee(self, kind, target, hit_delay):
        if self._anim == None or not self._anim.has_animation(kind):
            return
        self._clip = kind
        self._anim.play(kind)
        self._melee = hit_delay + 0.45
        self._melee_hit = hit_delay
        self._melee_kind = kind
        self._melee_target = target
        self._lunge = 0.0
        self._kick_landed = False
        if kind == "flying_kick":
            self._melee = hit_delay + 0.6
            self._jump_pending = self.kick_jump
            self._lunge_dir = self.forward_flat()
            if target != None:
                p = self.global_position
                t = target.global_position
                dx = t.x - p.x
                dz = t.z - p.z
                dist = math.max(math.sqrt(dx * dx + dz * dz), 0.01)
                self._lunge_dir = Vec3(dx / dist, 0.0, dz / dist)
                self._lunge = math.max(dist - self.kick_reach, 0.0) / (hit_delay + 0.05)
            else:
                self._lunge = 2.0
        if target != None:
            p = self.global_position
            t = target.global_position
            self._yaw = math.degrees(math.atan2(-(t.x - p.x), -(t.z - p.z)))
        self._snd_whoosh.play()

    def foot(self, name, bone):
        skeleton = self.get_node("Model/Armature/Skeleton3D")
        if skeleton == None:
            return None
        attach = skeleton.get_node(name)
        if attach == None:
            attach = skeleton.create_child("BoneAttachment3D")
            attach.name = name
            attach.bone_name = bone
        return attach

    def foot_touches(self, target):
        t = target.global_position
        for foot in [self._foot_l, self._foot_r]:
            if foot == None:
                continue
            f = foot.global_position
            dx = f.x - t.x
            dz = f.z - t.z
            if dx * dx + dz * dz < self.foot_radius * self.foot_radius and f.y > t.y - 1.0 and f.y < t.y + 1.0:
                return True
        return False

    def update_melee(self, delta):
        if self._melee <= 0.0:
            return
        self._melee = self._melee - delta
        if self._melee_kind == "flying_kick":
            target = self._melee_target
            if not self._kick_landed and self._melee_hit < 0.25 and target != None and target.is_in_group("enemy") and self.foot_touches(target):
                self._kick_landed = True
                self._lunge = 0.0
                self.melee_strike()
            if self._melee_hit > 0.0:
                self._melee_hit = self._melee_hit - delta
                if self._melee_hit <= 0.0 and not self._kick_landed and (target == None or not target.is_in_group("enemy")):
                    self._kick_landed = True
                    self.melee_breakable()
            return
        if self._melee_hit > 0.0:
            self._melee_hit = self._melee_hit - delta
            if self._melee_hit <= 0.0:
                self.melee_strike()

    def melee_strike(self):
        reach = self.melee_range + 0.6
        if self._melee_kind == "flying_kick":
            reach = self.kick_range
        target = self._melee_target
        if target == None or not target.is_in_group("enemy"):
            target = self.melee_candidate(reach)
        if target == None:
            self.melee_breakable()
            return
        p = self.global_position
        t = target.global_position
        dx = t.x - p.x
        dz = t.z - p.z
        dist = math.max(math.sqrt(dx * dx + dz * dz), 0.01)
        if dist > reach:
            return
        direction = Vec3(dx / dist, 0.15, dz / dist)
        self._snd_melee.play()
        self._hits = self._hits + 1
        self._hit_timer = 0.25
        self._kick = 1.5
        if self._melee_kind == "flying_kick":
            target.dismember(direction, 3.5)
        else:
            target.take_damage(self.melee_damage, Vec3(t.x, t.y + 0.5, t.z), direction, 1.2, self)

    def melee_breakable(self):
        p = self.global_position
        f = self.forward_flat()
        start = Vec3(p.x, p.y + 0.4, p.z)
        hit = raycast(start, Vec3(start.x + f.x * 1.6, start.y, start.z + f.z * 1.6), None, self)
        if hit == None or hit.node == None or not hit.node.is_in_group("breakable"):
            return
        damage = 140.0 if self._melee_kind == "flying_kick" else 60.0
        self._snd_melee.play()
        self._kick = 1.5
        hit.node.get_node("Breakable").hit(hit.position, hit.normal, f, damage, self)

    def update_overclock(self, delta):
        real = delta
        scale = time_scale()
        if scale > 0.0:
            real = delta / scale
        self._kill_slow = math.max(self._kill_slow - real, 0.0)
        if self._overclock <= 0.0:
            self._exhausted = True
        elif self._exhausted and self._overclock >= self.overclock_max * 0.25:
            self._exhausted = False
        diving = self._dodge > 0.0 and self._dodge_time - self._dodge >= self.dive_slow_delay
        drain = not self._exhausted and (diving or (self.captured() and mouse_check_button(MOUSE_RIGHT)))
        want = not self._dead and (drain or self._kill_slow > 0.0)
        self._draining = want and drain
        if want:
            if drain:
                self._overclock = math.max(self._overclock - real, 0.0)
            if not self._slow:
                self._slow = True
                set_time_scale(self.overclock_scale)
                set_bus_pitch(AUDIO_BUS_SFX, self.slow_pitch)
                self._snd_slow_start.play()
            heart = self._snd_heart
            if not heart.is_playing():
                heart.play()
        else:
            self._overclock = math.min(self._overclock + real * 0.35, self.overclock_max)
            if self._slow:
                self._slow = False
                set_time_scale(1.0)
                set_bus_pitch(AUDIO_BUS_SFX, 1.0)
                self._snd_heart.stop()
                self._snd_slow_end.play()

    def start_dodge(self):
        forward = self._fwd
        side = self._side
        if math.hypot(forward, side) < 0.1:
            forward = 1.0
            side = 0.0
        self._fwd = forward
        self._side = side
        direction = self.run_direction()
        yaw = math.radians(self._yaw)
        x = -math.sin(yaw) * forward + math.cos(yaw) * side
        z = -math.cos(yaw) * forward - math.sin(yaw) * side
        length = math.hypot(x, z)
        self._dodge_dir = Vec3(x / length, 0.0, z / length)
        self._dodge_side = "Forward"
        if direction == "Backward" or direction == "BackwardLeft" or direction == "BackwardRight":
            self._dodge_side = "Backward"
        elif direction == "Left" or direction == "Right":
            self._dodge_side = direction
        self._clip = "ShootDodge_" + direction
        self._anim.play(self._clip, 0.1)
        self._dodge_time = self._anim.get_current_animation_length()
        self._dodge = self._dodge_time
        self._snd_dive.play()

    # In bullet time (the slow motion or a dive) the body keeps its heading and the spine turns to
    # the aim, up to dive_twist degrees; past that the body follows. Out of it the body catches up
    # with the aim. The camera pivot takes the twist back, so the view still turns with the mouse.
    def turn_body(self):
        diff = (self._yaw - self._dodge_yaw + 540.0) % 360.0 - 180.0
        if not (self._slow or self._dodge > 0.0 or self._getup > 0.0):
            diff = diff * (1.0 - math.min(self._real_dt * 10.0, 1.0))
        twist = math.clamp(diff, -self.dive_twist, self.dive_twist)
        self._dodge_yaw = self._yaw - twist
        self._twist = twist
        self._body_yaw = self._dodge_yaw
        self.rotation_degrees = Vec3(0.0, self._body_yaw, 0.0)

    def update_dodge(self, delta):
        if self._dodge > 0.0:
            self._dodge = self._dodge - delta
            if self._dodge <= 0.0:
                self._clip = "ShootDodge_" + self._dodge_side + "_Getup"
                self._anim.play(self._clip, 0.1)
                self._getup = self._anim.get_current_animation_length() * 0.85
        elif self._getup > 0.0:
            self._getup = self._getup - delta

    # Hold Z to zoom: the camera closes in over the shoulder and narrows its view. In bullet time
    # the view widens by slow_fov instead, eased over a quarter of a second of real time.
    def update_zoom(self, delta, camera):
        wanted = 1.0 if (keyboard_check(KEY_Z) or self.force_zoom) and self.alive() and self._dodge <= 0.0 else 0.0
        real = delta / time_scale() if time_scale() > 0.0 else delta
        step = real * 8.0
        self._zoom = self._zoom + math.clamp(wanted - self._zoom, -step, step)
        slow = 1.0 if self._slow else 0.0
        self._slow_blend = self._slow_blend + math.clamp(slow - self._slow_blend, -real * 4.0, real * 4.0)
        if self._fov <= 0.0:
            self._fov = camera.fov
        wide = self._fov + self.slow_fov * self._slow_blend
        camera.fov = wide + (self.zoom_fov - wide) * self._zoom

    def place_camera(self, pivot, camera):
        z = self._zoom
        # Shootdodge, as in Max Payne: the camera follows her further back, so the whole body is
        # seen in the air; it eases out and back in.
        real = self._real_dt if self._real_dt > 0.0 else 0.016
        diving = self._dodge > 0.0 or self._getup > 0.0
        self._dive_cam = math.clamp(self._dive_cam + (real * 2.0 if diving else -real * 1.5), 0.0, 1.0)
        d = self._dive_cam * self._dive_cam * (3.0 - 2.0 * self._dive_cam)
        local = Vec3(self.camera_side + (self.zoom_side - self.camera_side) * z, self.camera_height + d * 0.35, self.camera_distance + (self.zoom_distance - self.camera_distance) * z + d * self.dive_cam_back)
        start = pivot.global_position
        wanted = pivot.to_global(local)
        full = math.sqrt(local.x * local.x + local.y * local.y + local.z * local.z)
        f = 1.0
        hit = raycast(start, wanted, None, self)
        if hit != None and full > 0.0:
            f = math.clamp((hit.distance - 0.2) / full, 0.15, 1.0)
        camera.position = Vec3(local.x * f, local.y * f, local.z * f)
        # Trauma shake (the Kinetix2D camera's formula): the strength is the trauma squared, a
        # sine at shake_frequency on X and a cosine out of phase on Y, plus a little roll.
        shake = self._game.shake_amount() if self._game != None else 0.0
        if shake > 0.0:
            phase = self._game._clock * self.shake_frequency * 6.28318530718
            sx = math.sin(phase) * self.shake_amplitude_x * shake
            sy = math.cos(phase + 2.32477856366) * self.shake_amplitude_y * shake
            camera.position = camera.position + Vec3(sx, sy, 0.0)
            camera.rotation_degrees = Vec3(0.0, 0.0, math.sin(phase * 0.5 + 1.1) * self.shake_roll * shake)
        else:
            camera.rotation_degrees = Vec3(0.0, 0.0, 0.0)
        # Close to a wall, or zoomed in: she fades so the aim stays clear.
        fade = math.clamp((self.fade_distance - full * f) / (self.fade_distance * 0.5), 0.0, 0.85)
        self.set_fade(math.max(fade, self._zoom * 0.7))

    def animate(self):
        if self._dead or self._melee > 0.0 or self._dodge > 0.0 or self._getup > 0.0:
            self.aim(False)
            return
        self.aim(True)
        if self._air > 0.15:
            self.play("Fall")
            return
        if self._moving > 0.1:
            self.play("Mona_Run_" + self.run_direction())
            return
        self.play("Stand")

    def run_direction(self):
        angle = math.degrees(math.atan2(self._side, self._fwd))
        return RUN_SECTORS[int((angle + 202.5) / 45.0) % 8]

    def aim(self, on):
        if self._upper == None:
            return
        clip = ""
        if on:
            clip = "Custom_Aim_Pistol_upper"
            if self._reload > 0.0:
                clip = self._reload_clip
        if clip != self._upper_clip:
            self._upper_clip = clip
            if clip == "":
                self._upper.stop()
            else:
                self._upper.play(clip)
        if self._bend != None:
            self._bend.pitch = self._pitch * 0.8 - self._kick * 7.0 if on else 0.0
            self._bend.yaw = self._twist

    def smooth_visuals(self, pivot):
        f = physics_interpolation_fraction()
        p = self._phys_prev
        c = self._phys_cur
        gp = self.global_position
        ox = p.x + (c.x - p.x) * f - gp.x
        oy = p.y + (c.y - p.y) * f - gp.y
        oz = p.z + (c.z - p.z) * f - gp.z
        yaw = math.radians(self._body_yaw)
        cs = math.cos(yaw)
        sn = math.sin(yaw)
        local = Vec3(ox * cs - oz * sn, oy, ox * sn + oz * cs)
        b = self._pivot_base
        pivot.position = Vec3(b.x + local.x, b.y + local.y, b.z + local.z)
        if self._model != None:
            m = self._model_base
            # Hips over the model's origin (her feet's level; the lift moves both, so the gap is
            # the clip's own): raised just enough to keep the body out of the floor, eased back.
            lift = 0.0
            if self._hips != None and (self._dodge > 0.0 or self._getup > 0.0 or self._dead):
                lift = math.max(HIPS_FLOOR - (self._hips.global_position.y - self._model.global_position.y), 0.0)
            self._ground_lift = self._ground_lift + (lift - self._ground_lift) * math.min(self._real_dt * 14.0, 1.0)
            self._model.position = Vec3(m.x + local.x, m.y + local.y + self._ground_lift, m.z + local.z)

    def _physics_process(self, delta):
        self._phys_prev = self.global_position
        self.physics_step(delta)
        self._phys_cur = self.global_position

    def physics_step(self, delta):
        if self.is_on_floor():
            self._vertical = -0.5
            self._air = 0.0
        else:
            self._vertical = self._vertical - self.gravity * delta
            self._air = self._air + delta
        # Still through the intro and while she is being built.
        if self._game != None and self._game.in_intro():
            self._moving = 0.0
            self.velocity = Vec3(0.0, self._vertical, 0.0)
            self.move_and_slide(delta)
            return
        if self._dead or self._melee > 0.0:
            self._moving = 0.0
            if self._jump_pending > 0.0:
                self._vertical = self._jump_pending
                self._jump_pending = 0.0
            speed = self._lunge
            target = self._melee_target
            if target != None and target.is_in_group("enemy"):
                p = self.global_position
                t = target.global_position
                gap = math.sqrt((t.x - p.x) * (t.x - p.x) + (t.z - p.z) * (t.z - p.z)) - self.kick_reach
                speed = math.clamp(gap / delta, 0.0, self._lunge)
            elif self._melee_hit <= 0.0:
                self._lunge = math.max(self._lunge - delta * 20.0, 0.0)
                speed = self._lunge
            d = self._lunge_dir
            if d == None:
                d = self.forward_flat()
            self.velocity = Vec3(d.x * speed, self._vertical, d.z * speed)
            self.move_and_slide(delta)
            return
        if self._dodge > 0.0 or self._getup > 0.0:
            self._moving = 0.0
            speed = 0.0
            # The clip has the leap in it: full speed away for the first half, then it brakes into
            # the landing.
            if self._dodge > 0.0 and self._dodge_time > 0.0:
                t = 1.0 - self._dodge / self._dodge_time
                speed = self.dodge_speed * math.clamp(1.0 - math.max(t - 0.45, 0.0) * 2.5, 0.0, 1.0)
            d = self._dodge_dir if self._dodge_dir != None else Vec3(0.0, 0.0, 0.0)
            self.velocity = Vec3(d.x * speed, self._vertical, d.z * speed)
            self.move_and_slide(delta)
            return
        forward = 0.0
        strafe = 0.0
        if keyboard_check(KEY_W):
            forward = forward + 1.0
        if keyboard_check(KEY_S):
            forward = forward - 1.0
        if keyboard_check(KEY_D):
            strafe = strafe + 1.0
        if keyboard_check(KEY_A):
            strafe = strafe - 1.0
        yaw = math.radians(self._yaw)
        fx = -math.sin(yaw)
        fz = -math.cos(yaw)
        rx = math.cos(yaw)
        rz = -math.sin(yaw)
        length = math.hypot(forward, strafe)
        self._moving = length
        self._fwd = forward
        self._side = strafe
        vx = 0.0
        vz = 0.0
        if length > 0.0:
            vx = (fx * forward + rx * strafe) / length * self.run_speed
            vz = (fz * forward + rz * strafe) / length * self.run_speed
        if self.is_on_floor() and keyboard_check(KEY_SPACE):
            self._vertical = self.jump_speed * (self.slow_jump if self._slow else 1.0)
            self._air = 0.2
        apart = self.spacing()
        self.velocity = Vec3(vx + apart.x, self._vertical, vz + apart.z)
        self.move_and_slide(delta)

    # Steps out of any cop closer than 0.7 m, so she never stands inside one.
    def spacing(self):
        if self._game == None or self._game.enemies == None:
            return Vec3(0.0, 0.0, 0.0)
        p = self.global_position
        sx = 0.0
        sz = 0.0
        for other in self._game.enemies:
            o = other.global_position
            dx = p.x - o.x
            dz = p.z - o.z
            d2 = dx * dx + dz * dz
            if d2 >= 0.49 or math.abs(o.y - p.y) > 1.5 or d2 < 0.000001:
                continue
            d = math.sqrt(d2)
            f = (0.7 - d) / 0.7
            sx = sx + dx / d * f
            sz = sz + dz / d * f
        return Vec3(sx * 4.0, 0.0, sz * 4.0)

    def start_reload(self):
        if self._reload > 0.0 or self._mag >= self.mag_size or self._reserve <= 0 or self._upper == None:
            return
        if self._slow:
            self._reload_clip = "BT2_Reload_Handgun_upper"
            self._snd_reload_bt.play()
        else:
            self._reload_clip = "Reload_Beretta_upper"
            self._snd_reload.play()
        self._upper.play(self._reload_clip, 0.1)
        self._upper_clip = self._reload_clip
        self._reload = math.min(self._upper.get_current_animation_length() * 0.8, 1.6)
        if self._slow:
            self._reload = 0.6

    def update_reload(self, real):
        if self._reload <= 0.0:
            return
        self._reload = self._reload - real
        if self._reload <= 0.0:
            take = int(math.min(self.mag_size - self._mag, self._reserve))
            self._mag = self._mag + take
            self._reserve = self._reserve - take

    def pickup(self, kind, amount):
        if kind == "keycard":
            if self._game != None and self._game.story != None:
                self._game.story.got_card()
            self._snd_pickup.stream = "Assets/Sounds/fx/pickup_energy.ogg"
            self._snd_pickup.play()
            return True
        if kind == "ammo":
            if self._reserve >= self.max_reserve:
                return False
            self._reserve = int(math.min(self._reserve + amount, self.max_reserve))
            self._ammo_flash = 0.6
            sound = "Assets/Sounds/fx/pickup_ammo.ogg"
        else:
            if self._health >= self.max_health:
                return False
            self._health = math.min(self._health + amount, self.max_health)
            self._energy_flash = 0.6
            sound = "Assets/Sounds/fx/pickup_energy.ogg"
        self._snd_pickup.stream = sound
        self._snd_pickup.play()
        return True

    def fire(self, camera):
        if self._cooldown > 0.0 or self._reload > 0.0:
            return
        if self._mag <= 0:
            self._cooldown = 0.3
            self._snd_empty.play()
            self.start_reload()
            return
        self._cooldown = self.fire_rate
        self._mag = self._mag - 1
        if self._game != None:
            self._game.noise(self.global_position, 14.0)
        self._kick = 1.0
        if self._game != None:
            self._game.add_trauma(self.shot_trauma)
        if self._muzzle != None:
            self._muzzle.fire()
        self._shots = self._shots + 1
        self._snd_shot.play()
        start = camera.global_position
        direction = camera.forward()
        end = Vec3(start.x + direction.x * 120.0, start.y + direction.y * 120.0, start.z + direction.z * 120.0)
        hit = raycast(start, end, None, self)
        if hit == None:
            return
        self.impact(hit, direction)

    def effects_parent(self):
        scene = self.get_tree().get_first_node_in_group("game")
        if scene == None:
            scene = self.get_tree().get_current_scene()
        if scene == None:
            scene = self.get_parent()
        return scene

    def impact(self, hit, direction):
        scene = self.effects_parent()
        body = hit.node
        kind = "default"
        if body != None and body.is_in_group("breakable"):
            body.get_node("Breakable").hit(hit.position, hit.normal, direction, self.damage, self)
            return
        fx = None
        if body == None or not body.is_in_group("enemy"):
            fx = scene.instantiate("Assets/Prefabs/impact.scene.json")
        if fx != None:
            fx.global_position = Vec3(hit.position.x + hit.normal.x * 0.02, hit.position.y + hit.normal.y * 0.02, hit.position.z + hit.normal.z * 0.02)
            kind = fx.surface(hit.normal, hit.material)
        if body != None:
            if body.is_in_group("enemy"):
                self._hits = self._hits + 1
                self._hit_timer = 0.25
                if body.is_headshot(hit.position):
                    body.headshot(hit.position, direction)
                else:
                    body.take_damage(self.damage, hit.position, direction, 0.5, self)
                return
            if body.is_in_group("damageable") and not body.is_in_group("player"):
                self._hits = self._hits + 1
                self._hit_timer = 0.25
                body.take_damage(self.damage, hit.position, direction, 0.5, self)
                return
            if body.is_class("RigidBody3D"):
                body.apply_impulse(Vec3(direction.x * 1.5, direction.y * 1.5, direction.z * 1.5), Vec3(hit.position.x - body.global_position.x, hit.position.y - body.global_position.y, hit.position.z - body.global_position.z))
                return
        decal = scene.create_child("Decal3D", "Assets/Scripts/decal.py")
        decal.name = "Hole"
        size = 0.25 if kind == "glass" else 0.13
        decal.texture_albedo = "Assets/Textures/Decals/" + HOLES[kind]
        decal.size = Vec3(size, 0.2, size)
        decal.modulate = Color(1.0, 1.0, 1.0, 1.0)
        decal.global_position = Vec3(hit.position.x + hit.normal.x * 0.01, hit.position.y + hit.normal.y * 0.01, hit.position.z + hit.normal.z * 0.01)
        self.orient_up(decal, hit.normal)

    def orient_up(self, node, up):
        ax = 0.0 - up.z
        az = up.x
        length = math.sqrt(ax * ax + az * az)
        d = math.clamp(up.y, -1.0, 1.0)
        if length < 0.0001:
            if d < 0.0:
                node.quaternion = Quat(1.0, 0.0, 0.0, 0.0)
            else:
                node.quaternion = Quat(0.0, 0.0, 0.0, 1.0)
            return
        angle = math.acos(d)
        s = math.sin(angle * 0.5)
        node.quaternion = Quat(ax / length * s, 0.0, az / length * s, math.cos(angle * 0.5))

    def take_damage(self, amount, origin, direction, force, source=None):
        if self._dead:
            return
        self._health = math.max(self._health - amount, 0.0)
        self._kick = math.min(self._kick + 1.0, 4.0)
        if self._damage_marks == None:
            self._damage_marks = []
        merged = False
        dl = math.max(math.sqrt(direction.x * direction.x + direction.z * direction.z), 0.0001)
        for m in self._damage_marks:
            ml = math.max(math.sqrt(m[0] * m[0] + m[1] * m[1]), 0.0001)
            if (-direction.x * m[0] - direction.z * m[1]) / (dl * ml) > 0.9:
                m[2] = 1.2
                m[3] = m[3] + amount
                merged = True
                break
        if not merged:
            self._damage_marks.append([-direction.x, -direction.z, 1.2, amount])
        while len(self._damage_marks) > 4:
            self._damage_marks.pop(0)
        if self._health > 0.0:
            return
        self._dead = True
        self._melee = 0.0
        if force >= 2.0:
            self.blast_death(direction)
        else:
            self.play("Death_A")
        game = self.get_tree().get_first_node_in_group("game")
        if game != None:
            game.player_died()

    def draw_damage(self, cx, cy, width, height):
        if self._damage_marks == None or len(self._damage_marks) == 0:
            return
        yaw = math.radians(self._yaw)
        fx = -math.sin(yaw)
        fz = -math.cos(yaw)
        rx = math.cos(yaw)
        rz = -math.sin(yaw)
        flicker = 1.0 if int(self._spin * 0.75) % 2 == 0 else 0.55
        keep = []
        for m in self._damage_marks:
            m[2] = m[2] - self._real_dt
            if m[2] <= 0.0:
                continue
            keep.append(m)
            t = m[2] / 1.2
            theta = math.atan2(m[0] * rx + m[1] * rz, m[0] * fx + m[1] * fz)
            alpha = math.clamp(t * 1.6, 0.0, 1.0)
            if t > 0.75:
                alpha = alpha * flicker
            red = Color(1.0, 0.12, 0.1, alpha)
            radius = 118.0 + (1.0 - t) * 26.0
            for k in range(5):
                a0 = theta + math.radians(-23.0 + k * 9.5)
                a1 = a0 + math.radians(7.0)
                hud_line(cx + math.sin(a0) * radius, cy - math.cos(a0) * radius, cx + math.sin(a1) * radius, cy - math.cos(a1) * radius, red, 6.0)
            tip = radius + 22.0
            side = radius + 8.0
            tx = cx + math.sin(theta) * tip
            ty = cy - math.cos(theta) * tip
            for d in [-1.0, 1.0]:
                a = theta + math.radians(5.0) * d
                hud_line(cx + math.sin(a) * side, cy - math.cos(a) * side, tx, ty, red, 3.0)
            degrees = int(math.degrees(theta) + 360.0) % 360
            label = "IMPACT -" + str(int(m[3])) + "  //  " + str(degrees) + " DEG"
            lx = cx + math.sin(theta) * (tip + 26.0)
            ly = cy - math.cos(theta) * (tip + 26.0)
            size = hud_text_size(label, 12.0)
            hud_rect(lx - size.x * 0.5 - 6.0, ly - 9.0, size.x + 12.0, 18.0, Color(0.0, 0.0, 0.0, 0.45 * alpha))
            hud_text(lx - size.x * 0.5, ly - 7.0, label, Color(1.0, 0.35, 0.3, alpha), 12.0)
            edge = Color(1.0, 0.05, 0.05, 0.28 * alpha)
            sx = math.sin(theta)
            sy = -math.cos(theta)
            if math.abs(sx) > math.abs(sy):
                if sx > 0.0:
                    hud_rect(width - 26.0, 0.0, 26.0, height, edge)
                else:
                    hud_rect(0.0, 0.0, 26.0, height, edge)
            elif sy < 0.0:
                hud_rect(0.0, 0.0, width, 26.0, edge)
            else:
                hud_rect(0.0, height - 26.0, width, 26.0, edge)
        self._damage_marks = keep

    def draw_weapon_panel(self, width, height, cyan):
        x = width - 300.0
        y = height - 104.0
        glow = self._ammo_flash
        hud_rect(x, y, 284.0, 62.0, Color(0.02, 0.1, 0.18, 0.55 + glow * 0.4))
        hud_rect_outline(x, y, 284.0, 62.0, Color(0.4, 0.8, 1.0, 0.7 + glow * 0.3), 2.0)
        hud_image(self._img.weapon, x + 10.0, y + 6.0, 112.0, 50.0, 0.0, 0.0, 0.0, 1.0, 1.0, Color(0.75, 0.95, 1.0, 1.0))
        mag_color = Color(1.0, 1.0, 1.0, 0.95)
        if self._mag <= 3:
            mag_color = Color(1.0, 0.3, 0.25, 1.0)
        mag = str(self._mag)
        hud_text(x + 214.0 - hud_text_size(mag, 34.0).x, y + 12.0, mag, mag_color, 34.0)
        hud_text(x + 222.0, y + 24.0, "/ " + str(self._reserve), cyan, 18.0)
        if self._reload > 0.0 and int(self._spin / 12.0) % 2 == 0:
            hud_text(x + 132.0, y + 44.0, "RELOADING", Color(1.0, 0.8, 0.3, 1.0), 13.0)
        elif self._mag == 0 and self._reserve == 0:
            hud_text(x + 132.0, y + 44.0, "NO AMMO", Color(1.0, 0.3, 0.25, 1.0), 13.0)

    def draw_hud(self):
        width = hud_width()
        height = hud_height()
        cx = width * 0.5
        cy = height * 0.5
        white = Color(1.0, 1.0, 1.0, 0.92)
        dim = Color(1.0, 1.0, 1.0, 0.5)
        red = Color(1.0, 0.3, 0.25, 1.0)
        cyan = Color(0.4, 0.8, 1.0, 1.0)
        low = 1.0 - self._health / self.max_health
        if low > 0.35:
            hud_image(self._img.vignette, 0.0, 0.0, width, height, 0.0, 0.0, 0.0, 1.0, 1.0, Color(0.9, 0.05, 0.05, (low - 0.35) * 1.2))
        if self._slow:
            hud_image(self._img.vignette, 0.0, 0.0, width, height, 0.0, 0.0, 0.0, 1.0, 1.0, Color(0.3, 0.7, 1.0, 0.35))
        size = 56.0 + self._kick * 26.0
        reticle_color = Color(1.0, 1.0, 1.0, 0.95)
        if self._target or self._near != None:
            reticle_color = Color(1.0, 0.22, 0.18, 1.0)
        hud_image(self._img.reticle, cx - size * 0.5, cy - size * 0.5, size, size, 0.0, 0.0, 0.0, 1.0, 1.0, reticle_color)
        ring = 84.0 + self._kick * 20.0
        ring_color = Color(1.0, 1.0, 1.0, 0.25)
        if self._slow:
            ring_color = Color(0.4, 0.8, 1.0, 0.8)
        if self._near != None:
            ring = 120.0 + self._kick * 20.0
            ring_color = Color(1.0, 0.22, 0.18, 0.7)
        hud_image(self._img.ring, cx - ring * 0.5, cy - ring * 0.5, ring, ring, self._spin, 0.0, 0.0, 1.0, 1.0, ring_color)
        if self._near != None:
            label = "LMB WHIP   F KICK"
            hud_text(cx - hud_text_size(label, 14.0).x * 0.5, cy + ring * 0.5 + 6.0, label, red, 14.0)
        if self._hit_timer > 0.0:
            mark = 40.0 + (0.25 - self._hit_timer) * 60.0
            hud_image(self._img.hitmark, cx - mark * 0.5, cy - mark * 0.5, mark, mark, 0.0, 0.0, 0.0, 1.0, 1.0, Color(1.0, 0.3, 0.25, self._hit_timer * 4.0))
        y = height - 56.0
        hud_image(self._img.panel_red, 16.0, y, 260.0, 34.0)
        ratio = self._health / self.max_health
        if ratio > 0.0:
            hud_image(self._img.fill_red, 34.0, y + 6.0, 224.0 * ratio, 22.0, 0.0, 0.0, 0.0, ratio, 1.0, white)
        hud_text(40.0, y + 9.0, "INTEGRITY", white, 15.0)
        text = str(int(self._health / self.max_health * 100.0 + 0.5)) + "%"
        hud_text(246.0 - hud_text_size(text, 15.0).x, y + 9.0, text, white, 15.0)
        hud_image(self._img.panel_cyan, 292.0, y, 260.0, 34.0)
        filled = self._overclock / self.overclock_max
        for i in range(8):
            level = math.clamp(filled * 8.0 - i, 0.0, 1.0)
            alpha = 0.18 + 0.82 * level
            hud_image(self._img.segment, 312.0 + i * 26.0, y + 9.0, 22.0, 16.0, 0.0, 0.0, 0.0, 1.0, 1.0, Color(0.4, 0.8, 1.0, alpha))
        hud_text(312.0 + 8 * 26.0 + 6.0, y + 9.0, "OC", cyan, 15.0)
        if self._energy_flash > 0.0:
            hud_rect(16.0, y, 260.0, 34.0, Color(0.4, 1.0, 0.6, self._energy_flash * 0.6))
        self.draw_weapon_panel(width, height, cyan)
        self.draw_damage(cx, cy, width, height)
        controls = "LMB fire/whip   Z zoom   R reload   F kick   Q spider   E hack   V scan   RMB overclock   C/MMB dive   SPACE jump   ESC mouse"
        hud_text(width - hud_text_size(controls, 13.0).x - 20.0, height - 30.0, controls, dim, 13.0)
        if self._game == None or self._game.debug_keys:
            hud_text(12.0, 10.0, "fps " + str(int(fps())), dim, 12.0)
