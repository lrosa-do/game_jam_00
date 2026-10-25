# The Floater: a flying unit that appears halfway through the map, flies at the player and shoots.
# Skeleton for the one who takes it: fill in _process. Enemies on the ground are in enemy.py (the
# chase, the shots with raycast, take_damage); the player's shots hit anything in the group
# "enemy" that has take_damage(amount, origin, direction, force, source). Spawn one with
# game.instantiate("Assets/Prefabs/floater.scene.json") and set its global_position.
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

class Floater(Node3D):
    speed = 4.0          # m/s towards the player
    hover_height = 2.2   # metres above the player's feet it tries to keep
    fire_range = 12.0    # metres
    fire_rate = 0.5      # seconds between shots
    damage = 12.0
    health = 60.0
    _game = None
    _player = None
    _skeleton = None
    _muzzle = None
    _cooldown = 0.0
    _clock = 0.0
    _dead = False

    def _ready(self):
        self._game = self.get_tree().get_first_node_in_group("game")
        self._player = self.get_tree().get_first_node_in_group("player")
        self._skeleton = self.get_node("Model/Floater/Skeleton3D")
        self._muzzle = self.get_node("Model/Floater/Skeleton3D/Main/ponto_Gun")
        # TODO: self.add_to_group("enemy") once take_damage is implemented, so she can shoot it.

    def _process(self, delta):
        if self._dead or self._player == None:
            return
        self._clock = self._clock + delta
        # TODO 1: fly towards the player (keep hover_height above them, stop at fire_range * 0.7);
        #         bob with math.sin(self._clock * 2.0) * 0.15 and tilt into the movement.
        # TODO 2: face the player (look_at) and animate the bones: wings flap with a sine, the
        #         antenna wobbles, the arms point the gun at her (see the header for the names).
        # TODO 3: every fire_rate seconds inside fire_range: raycast from ponto_Gun to the player,
        #         play Shot, self._player.take_damage(self.damage, ...) on a hit; a muzzle flash
        #         prefab exists (Assets/Prefabs/muzzle_flash.scene.json).
        # TODO 4: take_damage(): subtract health, play Hit, flash the model; at 0 play Death and
        #         self._game.explode(self.global_position, 2.0, 6.0, 20.0, "small", self), queue_free().
        pass
