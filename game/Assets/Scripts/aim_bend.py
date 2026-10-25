import math

# Bends the spine after the animation: pitch leans it to the aim, yaw twists it (the dive turns the
# upper body to the aim while the legs keep the heading), both spread over the spine bones.
class AimBend(Node3D):
    pitch = 0.0
    yaw = 0.0
    bones = "Spine,Spine1,Spine2"
    _skeleton = None
    _ids = None

    def _ready(self):
        self._skeleton = self.get_parent()
        self._ids = []
        for name in self.bones.split(","):
            bone = self._skeleton.find_bone(name)
            if bone >= 0:
                self._ids.append(bone)

    def _process(self, delta):
        if self._skeleton == None or len(self._ids) == 0 or (math.abs(self.pitch) < 0.01 and math.abs(self.yaw) < 0.01):
            return
        half = math.radians(self.pitch / len(self._ids)) * 0.5
        s = math.sin(half)
        c = math.cos(half)
        turn = math.radians(self.yaw / len(self._ids)) * 0.5
        ts = math.sin(turn)
        tc = math.cos(turn)
        for bone in self._ids:
            q = self._skeleton.get_bone_pose_rotation(bone)
            # The twist about the bone's own axis along the spine (Y), then the lean (X).
            q = Quat(q.x * tc - q.z * ts, q.y * tc + q.w * ts, q.z * tc + q.x * ts, q.w * tc - q.y * ts)
            self._skeleton.set_bone_pose_rotation(bone, Quat(q.x * c + q.w * s, q.y * c + q.z * s, q.z * c - q.y * s, q.w * c - q.x * s))
