import math

class ThirdPersonCamera(Camera3D):
    target_path = ""
    height = 1.6
    distance = 5.0
    margin = 0.3
    sensitivity = 0.2
    min_pitch = -20.0
    max_pitch = 70.0
    spring = 10.0
    yaw = 0.0
    pitch = 20.0
    _arm_length = 5.0

    def _ready(self):
        self.make_current()
        self._arm_length = self.distance

    def _exit_tree(self):
        mouse_set_mode(MOUSE_MODE_VISIBLE)

    def _process(self, delta):
        if mouse_check_button_pressed(MOUSE_ANY):
            mouse_set_mode(MOUSE_MODE_CAPTURED)
        if keyboard_check_pressed(KEY_ESCAPE):
            mouse_set_mode(MOUSE_MODE_VISIBLE)
        if mouse_get_mode() == MOUSE_MODE_CAPTURED:
            move = mouse_delta()
            self.yaw = self.yaw - move.x * self.sensitivity
            self.pitch = math.clamp(self.pitch + move.y * self.sensitivity, self.min_pitch, self.max_pitch)
        if self.target_path == "":
            return
        target = self.get_node(self.target_path)
        if target == None:
            return
        pivot = target.global_position + Vec3(0.0, self.height, 0.0)
        yaw = math.radians(self.yaw)
        pitch = math.radians(self.pitch)
        direction = Vec3(math.sin(yaw) * math.cos(pitch), math.sin(pitch), math.cos(yaw) * math.cos(pitch))
        reach = pivot + direction * (self.distance + self.margin)
        if target.is_class("PhysicsBody3D"):
            hit = raycast(pivot, reach, None, target)
        else:
            hit = raycast(pivot, reach)
        wanted = self.distance
        if hit != None:
            wanted = math.clamp(hit.distance - self.margin, 0.2, self.distance)
        if wanted < self._arm_length:
            self._arm_length = wanted
        else:
            self._arm_length = math.lerp(self._arm_length, wanted, 1.0 - math.exp(-self.spring * delta))
        self.global_position = pivot + direction * self._arm_length
        self.look_at(pivot)
