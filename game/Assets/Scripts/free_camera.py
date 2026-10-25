# A free camera to fly the map: click captures the mouse, WASD moves, Q/E down and up, Shift is
# fast, ESC frees the mouse. Shift+F1 shows the engine statistics, Shift+F9 the profiler.
import math

class FreeCamera(Camera3D):
    speed = 8.0
    fast = 4.0
    sensitivity = 0.15
    yaw = 0.0
    pitch = 0.0

    def _ready(self):
        self.make_current()
        self.yaw = self.rotation_degrees.y
        self.pitch = self.rotation_degrees.x

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
            self.pitch = math.clamp(self.pitch - move.y * self.sensitivity, -89.0, 89.0)
        self.rotation_degrees = Vec3(self.pitch, self.yaw, 0.0)
        speed = self.speed * (self.fast if keyboard_check(KEY_LEFT_SHIFT) else 1.0) * delta
        yaw = math.radians(self.yaw)
        pitch = math.radians(self.pitch)
        forward = Vec3(-math.sin(yaw) * math.cos(pitch), math.sin(pitch), -math.cos(yaw) * math.cos(pitch))
        right = Vec3(math.cos(yaw), 0.0, -math.sin(yaw))
        p = self.position
        if keyboard_check(KEY_W):
            p = p + forward * speed
        if keyboard_check(KEY_S):
            p = p - forward * speed
        if keyboard_check(KEY_D):
            p = p + right * speed
        if keyboard_check(KEY_A):
            p = p - right * speed
        if keyboard_check(KEY_E):
            p = p + Vec3(0.0, speed, 0.0)
        if keyboard_check(KEY_Q):
            p = p - Vec3(0.0, speed, 0.0)
        self.position = p
        dim = Color(0.75, 0.8, 0.9, 0.8)
        hud_text(12.0, hud_height() - 30.0, "click captures   WASD move   Q/E down/up   SHIFT fast   F10 fit   F11 quality   SHIFT+F1 stats   SHIFT+F9 profiler   ESC mouse   CTRL+X quit", dim, 16.0)
