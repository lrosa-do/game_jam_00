# The Floater: a flying unit NULL//OS builds out of a dead cop (game.spawn_floater, every
# floater_every cops she kills): the materialize effect winds up under it, it appears on the
# burst, then flies at her and shoots. The player's shots hit anything in the group "enemy" that
# has take_damage(amount, origin, direction, force, source).
#
# The model (Assets/Models/Floater/floater.glb) has a skeleton but NO animation clips, so it is
# animated by hand with bone poses on the Skeleton3D at "Model/Floater/Skeleton3D":
#   bones: Main, Gun, R_UpperArm, R_LowerArm, R_Hand, L_UpperArm, L_LowerArm, L_Hand, R_Wing,
#          L_Wing, Antena_Root, Antena01..Antena04, AntenaTip
#   skeleton.find_bone("R_Wing") -> index; skeleton.set_bone_pose_rotation(index, Quat(x, y, z, w))
#   Quat(x, y, z, w) is the raw quaternion (Quat() is identity): for an angle a about an axis
#   (ax, ay, az), Quat(ax * sin(a/2), ay * sin(a/2), az * sin(a/2), cos(a/2)).
#   (also set_bone_pose_position / set_bone_pose_scale). Flap the wings with a sine, wobble the
#   antenna, point the arms at the player; the whole body bobs and tilts from this node.
# Attachment points (BoneAttachment3D children of the skeleton): Main/ponto_Gun (muzzle),
# Main/ponto_Engine, ponto_EngineLeft, ponto_EngineRight (thrusters), R_Hand/ponto_R_Hand.
# Sounds on the prefab: Engine (loop, autoplay), Shot, Sight (when it sees her), Death, Hit.
import math

class Floater(CharacterBody3D):
    speed = 4.0          # m/s towards the player
    hover_height = 2.2   # metres above the player's feet it tries to keep
    fire_range = 12.0    # metres
    fire_rate = 0.8      # seconds between shots
    damage = 6.0
    health = 60.0
    civilian = False     # read by game.enemy_killed, as for the cops
    _kill_kind = ""
    _build = -1.0        # hidden until the materialize burst (appear)
    _grow = -1.0
    _model_scale = None
    _sighted = False
    _path = None
    _path_index = 0
    _repath = 0.0
    _look = 0.0
    _sees = False        # the last shot reached her: no need to come closer
    _game = None
    _player = None
    _skeleton = None
    _muzzle = None
    _flash = None
    _model = None
    _flash_meshes = None
    _hit_flash = 0.0
    _health = 60.0
    _cooldown = 0.0
    _clock = 0.0
    _dead = False
    _death_age = 0.0
    _death_duration = 0.8
    _death_origin = None
    _death_rotation = None
    _death_scale = None
    _death_direction = None
    _death_finished = False
    _wing_r = -1
    _wing_l = -1
    _antenna_root = -1
    _antenna_01 = -1
    _antenna_02 = -1
    _antenna_03 = -1
    _antenna_04 = -1
    _antenna_tip = -1
    _gun = -1
    _arm_r_upper = -1
    _arm_r_lower = -1
    _arm_r_hand = -1
    _arm_l_upper = -1
    _arm_l_lower = -1
    _arm_l_hand = -1

    def _ready(self):
        self._game = self.get_tree().get_first_node_in_group("game")
        self._player = self.get_tree().get_first_node_in_group("player")
        self._health = self.health
        self._model = self.get_node("Model")
        self._flash_meshes = []
        self._cache_flash_meshes(self._model)
        self._skeleton = self.get_node("Model/Floater/Skeleton3D")
        self._muzzle = self.get_node("Model/Floater/Skeleton3D/Main/ponto_Gun")
        if self._muzzle != None:
            self._flash = self._muzzle.instantiate("Assets/Prefabs/muzzle_flash.scene.json")
        self._wing_r = self._skeleton.find_bone("R_Wing")
        self._wing_l = self._skeleton.find_bone("L_Wing")
        self._antenna_root = self._skeleton.find_bone("Antena_Root")
        self._antenna_01 = self._skeleton.find_bone("Antena01")
        self._antenna_02 = self._skeleton.find_bone("Antena02")
        self._antenna_03 = self._skeleton.find_bone("Antena03")
        self._antenna_04 = self._skeleton.find_bone("Antena04")
        self._antenna_tip = self._skeleton.find_bone("AntenaTip")
        self._gun = self._skeleton.find_bone("Gun")
        self._arm_r_upper = self._skeleton.find_bone("R_UpperArm")
        self._arm_r_lower = self._skeleton.find_bone("R_LowerArm")
        self._arm_r_hand = self._skeleton.find_bone("R_Hand")
        self._arm_l_upper = self._skeleton.find_bone("L_UpperArm")
        self._arm_l_lower = self._skeleton.find_bone("L_LowerArm")
        self._arm_l_hand = self._skeleton.find_bone("L_Hand")
        self._rest_wing_r = self._skeleton.get_bone_pose_rotation(self._wing_r)
        self._rest_wing_l = self._skeleton.get_bone_pose_rotation(self._wing_l)
        self._rest_antenna_root = self._skeleton.get_bone_pose_rotation(self._antenna_root)
        self._rest_antenna_01 = self._skeleton.get_bone_pose_rotation(self._antenna_01)
        self._rest_antenna_02 = self._skeleton.get_bone_pose_rotation(self._antenna_02)
        self._rest_antenna_03 = self._skeleton.get_bone_pose_rotation(self._antenna_03)
        self._rest_antenna_04 = self._skeleton.get_bone_pose_rotation(self._antenna_04)
        self._rest_antenna_tip = self._skeleton.get_bone_pose_rotation(self._antenna_tip)
        self._rest_gun = self._skeleton.get_bone_pose_rotation(self._gun)
        self._rest_arm_r_upper = self._skeleton.get_bone_pose_rotation(self._arm_r_upper)
        self._rest_arm_r_lower = self._skeleton.get_bone_pose_rotation(self._arm_r_lower)
        self._rest_arm_r_hand = self._skeleton.get_bone_pose_rotation(self._arm_r_hand)
        self._rest_arm_l_upper = self._skeleton.get_bone_pose_rotation(self._arm_l_upper)
        self._rest_arm_l_lower = self._skeleton.get_bone_pose_rotation(self._arm_l_lower)
        self._rest_arm_l_hand = self._skeleton.get_bone_pose_rotation(self._arm_l_hand)
        self.add_to_group("enemy") 

    def _animate_bone(self, bone, rest, axis_x, axis_y, axis_z, angle):
        if bone < 0:
            return
        half_angle = angle * 0.5
        sin_half = math.sin(half_angle)
        dx = axis_x * sin_half
        dy = axis_y * sin_half
        dz = axis_z * sin_half
        dw = math.cos(half_angle)
        self._skeleton.set_bone_pose_rotation(
            bone,
            Quat(
                rest.w * dx + rest.x * dw + rest.y * dz - rest.z * dy,
                rest.w * dy - rest.x * dz + rest.y * dw + rest.z * dx,
                rest.w * dz + rest.x * dy - rest.y * dx + rest.z * dw,
                rest.w * dw - rest.x * dx - rest.y * dy - rest.z * dz
            )
        )

    # Built in place by the fx (Assets/Prefabs/materialize.scene.json): hidden and still until its
    # burst calls appear() (delay is only a safety net), then it grows out of the flash.
    def materialize(self, delay):
        self._build = delay
        if self._model != None:
            self._model.visible = False

    def appear(self):
        self._build = -1.0
        self._grow = 0.0
        if self._model != None:
            self._model_scale = self._model.scale
            self._model.scale = Vec3(0.01, 0.01, 0.01)
            self._model.visible = True

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
        if self._dead:
            self._animate_death(delta)
            return
        if self._build >= 0.0 or self._grow >= 0.0:
            self.update_build(delta)
            if self._build >= 0.0:
                return
        if self._player == None:
            return
        self._clock = self._clock + delta
        if self._hit_flash > 0.0:
            self._hit_flash = self._hit_flash - delta
            if self._hit_flash <= 0.0:
                for mesh in self._flash_meshes:
                    mesh[0].color = mesh[1]

        target = self._player.global_position
        self._look = self._look - delta
        if self._look <= 0.0:
            self._look = 0.25
            p = self._muzzle.global_position if self._muzzle != None else self.global_position
            hit = raycast(p, Vec3(target.x, target.y + 0.5, target.z), None, self)
            self._sees = hit != None and hit.node == self._player
        self._moving(delta)
        self._animate(self.global_position, target)
        # Like the cops, it holds fire while NULL//OS reads the orders.
        if self._player.alive() and (self._game == None or not self._game.briefing()):
            self._shoot(target, delta)


    def _cache_flash_meshes(self, node):
        if node == None:
            return
        for child in node.get_children():
            if child.is_class("MeshInstance3D"):
                self._flash_meshes.append([child, child.color])
            self._cache_flash_meshes(child)

    def take_damage(self, amount, origin, direction, force, source=None):
        if self._dead or self._build >= 0.0:
            return
        self._health = self._health - amount
        self.get_node("Hit").play()
        self._hit_flash = 0.12
        for mesh in self._flash_meshes:
            mesh[0].color = Color(1.8, 0.45, 0.35, 1.0)
        if self._health <= 0.0:
            self.dismember(direction, force)

    def alive(self):
        return not self._dead and self._build < 0.0

    def is_headshot(self, point):
        return False

    def die(self, direction, force):
        self.dismember(direction, force)

   
    def dismember(self, direction, force):
        if self._dead or self._build >= 0.0:
            return
        self._dead = True
        self.remove_from_group("enemy")
        self.remove_from_group("floater")
        self.collision_layer = 0
        self.collision_mask = 0
        self._death_age = 0.0
        self._death_origin = self.global_position
        self._death_rotation = self.rotation_degrees
        self._death_scale = self._model.scale
        if direction == None:
            self._death_direction = Vec3(0.0, 0.0, 0.0)
        else:
            self._death_direction = direction
        self.get_node("Death").play()
        # A kill like a cop's (score, chain, quip); cop=False so it does not build another.
        if self._game != None:
            self._game.enemy_killed(self, False)

    def _animate_death(self, delta):
        self._death_age = self._death_age + delta
        t = math.clamp(self._death_age / self._death_duration, 0.0, 1.0)
        p = self._death_origin
        d = self._death_direction
        self.global_position = Vec3(
            p.x + d.x * 0.6 * t,
            p.y - 0.8 * t,
            p.z + d.z * 0.6 * t
        )
        r = self._death_rotation
        self.rotation_degrees = Vec3(r.x + 420.0 * t, r.y + 720.0 * t, r.z + 240.0 * t)
        scale = 1.0 - 0.75 * t
        self._model.scale = Vec3(self._death_scale.x * scale, self._death_scale.y * scale, self._death_scale.z * scale)
        if t < 1.0 or self._death_finished:
            return
        death = self.get_node("Death")
        if death != None and death.is_playing() and self._death_age < 2.5:
            return
        self._death_finished = True
        if self._game != None:
            self._game.explode(self.global_position, 2.0, 6.0, 20.0, "small", self)
        self.queue_free()
        
    def _shoot(self,target,delta):
        self._cooldown = math.max(self._cooldown - delta, 0.0)
        dx = target.x - self.global_position.x
        dy = target.y - self.global_position.y
        dz = target.z - self.global_position.z
        if self._cooldown > 0.0 or not self._sees or dx * dx + dy * dy + dz * dz > self.fire_range * self.fire_range:
            return
        if not self._sighted:
            self._sighted = True
            self.get_node("Sight").play()

        start = self._muzzle.global_position
        aim = Vec3(target.x, target.y + 0.5, target.z)
        direction = Vec3(aim.x - start.x, aim.y - start.y, aim.z - start.z)
        length = math.sqrt(direction.x * direction.x + direction.y * direction.y + direction.z * direction.z)
        if length < 0.001:
            return

        self._cooldown = self.fire_rate
        direction = Vec3(direction.x / length, direction.y / length, direction.z / length)
        self.get_node("Shot").play()
        if self._flash != None:
            self._flash.fire()

        # A bolt she can see coming (and dodge): Assets/Scripts/bolt.py.
        bolt = self._game.instantiate("Assets/Prefabs/floater_bolt.scene.json") if self._game != None else None
        if bolt != None:
            bolt.global_position = start
            bolt.direction = direction
            bolt.damage = self.damage
            bolt.source = self



    def _path_corner(self, current, target, delta):
        nav = self._game.nav if self._game != None else None
        if nav == None or not nav.is_baked():
            return None
        self._repath = self._repath - delta
        if (self._path == None or self._repath <= 0.0) and self._game.path_ticket():
            self._repath = 0.5
            # From the floor under it, not hers: on the ramp down to the garage they differ.
            self._path = nav.find_path(Vec3(current.x, current.y - self.hover_height, current.z), target)
            self._path_index = 1
        if self._path == None:
            return None
        while self._path_index < len(self._path):
            c = self._path[self._path_index]
            if (c.x - current.x) * (c.x - current.x) + (c.z - current.z) * (c.z - current.z) > 0.5 * 0.5:
                return c
            self._path_index = self._path_index + 1
        return None

    def _animate(self,current,target):
        dx = target.x - current.x
        dz = target.z - current.z
        horizontal_distance = math.sqrt(dx * dx + dz * dz)
        aim_pitch = math.atan2(target.y - current.y, horizontal_distance)
        wing_flap = math.sin(self._clock * 8.0) * 0.02
        antenna_wobble = math.sin(self._clock * 3.0) * 0.12

        self._animate_bone(self._wing_r, self._rest_wing_r, 0.0, 0.0, 1.0, wing_flap)
        self._animate_bone(self._wing_l, self._rest_wing_l, 0.0, 0.0, 1.0, -wing_flap)
        self._animate_bone(self._antenna_root, self._rest_antenna_root, 0.0, 0.0, 1.0, antenna_wobble)
        self._animate_bone(self._antenna_01, self._rest_antenna_01, 0.0, 0.0, 1.0, antenna_wobble * 0.8)
        self._animate_bone(self._antenna_02, self._rest_antenna_02, 0.0, 0.0, 1.0, antenna_wobble * 0.6)
        self._animate_bone(self._antenna_03, self._rest_antenna_03, 0.0, 0.0, 1.0, antenna_wobble * 0.4)
        self._animate_bone(self._antenna_04, self._rest_antenna_04, 0.0, 0.0, 1.0, antenna_wobble * 0.25)
        self._animate_bone(self._antenna_tip, self._rest_antenna_tip, 0.0, 0.0, 1.0, antenna_wobble * 0.15)
        
        self._animate_bone(self._arm_r_upper, self._rest_arm_r_upper, 1.0, 0.0, 0.0, aim_pitch * 0.45)
        self._animate_bone(self._arm_r_lower, self._rest_arm_r_lower, 1.0, 0.0, 0.0, aim_pitch * 0.35)
        self._animate_bone(self._arm_r_hand, self._rest_arm_r_hand, 1.0, 0.0, 0.0, aim_pitch * 0.2)
        self._animate_bone(self._arm_l_upper, self._rest_arm_l_upper, 1.0, 0.0, 0.0, aim_pitch * 0.45)
        self._animate_bone(self._arm_l_lower, self._rest_arm_l_lower, 1.0, 0.0, 0.0, aim_pitch * 0.35)
        self._animate_bone(self._arm_l_hand, self._rest_arm_l_hand, 1.0, 0.0, 0.0, aim_pitch * 0.2)
        self._animate_bone(self._gun, self._rest_gun, 1.0, 0.0, 0.0, aim_pitch)


    # Flies at speed towards a spot beside her and keeps hover_height over her feet; move_and_slide
    # keeps it out of the walls and the time step keeps it the same at any frame rate.
    def _moving (self, delta):
        #start moving mechanics

        current = self.global_position
        target = self._player.global_position
        
        dx = target.x - current.x
        dz = target.z - current.z
        distance = math.sqrt(dx * dx + dz * dz)

        stop_distance = self.fire_range * 0.7
        

        if (dx<0):
            angle=-math.asin(dx/distance)
        else:
            angle=math.asin(dx/distance)

        
        futureX = target.x + 2*math.cos(angle) * 1.1
        futureZ=target.z + 2*math.sin(angle) * 1.1
        futureY=target.y + self.hover_height

        # Out of sight it goes round the walls: the corners of a path on the navigation, like
        # the cops (game.path_ticket spreads the requests over the frames), and at the height of
        # the path, so it comes down the ramps and the stairs instead of pressing on the floor.
        if not self._sees:
            corner = self._path_corner(current, target, delta)
            if corner != None:
                futureX = corner.x
                futureZ = corner.z
                futureY = corner.y + self.hover_height

        
        vx = 0.0
        vz = 0.0
        if (distance>stop_distance or not self._sees):
            mx = futureX - current.x
            mz = futureZ - current.z
            ml = math.max(math.sqrt(mx * mx + mz * mz), 0.001)
            vx = mx / ml * self.speed
            vz = mz / ml * self.speed

        #start floating mechanics
        vy = (futureY - current.y) * 1.8 + math.sin(self._clock) * 0.6

        #end floating mechanics

        #end moving mechanics

        self.velocity = Vec3(vx, vy, vz)
        self.move_and_slide(delta)
        p = self.global_position
        if distance > 0.01:
            self.look_at(Vec3(target.x, p.y, target.z))
    