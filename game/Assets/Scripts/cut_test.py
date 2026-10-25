# The dismemberment test: the people of the game in a row (cops, security, the prisoner...) and keys
# that cut them, to see how each model comes apart. Fly with the free camera (click captures the mouse).
#   1 head   2 left arm   3 right arm   4 left forearm   5 right forearm   6 left leg   7 right leg
#   X  the game's own cut (a random piece or two)     Z  every piece off, blast     R  new row
#   T  rigid pieces on or off (the body split in static pieces cut at the joints, or the skinned cut)
# The first cut kills the person (a corpse with a ragdoll), the next ones take more pieces off it.
import math

NAMES = ["cop_a", "cop_b", "cop_female", "security", "bravura", "prisoner", "towel"]
BONES = ["Bip01 Head", "Bip01 L UpperArm", "Bip01 R UpperArm", "Bip01 L Forearm", "Bip01 R Forearm", "Bip01 L Thigh", "Bip01 R Thigh"]
STUMPS = ["Bip01 Neck", "Bip01 L Clavicle", "Bip01 R Clavicle", "", "", "", ""]
LABELS = ["HEAD", "L ARM", "R ARM", "L FOREARM", "R FOREARM", "L LEG", "R LEG"]

class CutTest(Node3D):
    # The enemies look the game up for these: no navigation here, nobody to hunt.
    nav = None
    enemies = None
    people = None
    _parts = None
    _message = ""
    _timer = 0.0
    _rigid = True

    def _ready(self):
        self._parts = [RAGDOLL_PART_HEAD, RAGDOLL_PART_LEFT_UPPER_ARM, RAGDOLL_PART_RIGHT_UPPER_ARM, RAGDOLL_PART_LEFT_LOWER_ARM, RAGDOLL_PART_RIGHT_LOWER_ARM, RAGDOLL_PART_LEFT_UPPER_LEG, RAGDOLL_PART_RIGHT_UPPER_LEG]
        self.spawn_row()

    def spawn_row(self):
        if self.people != None:
            for person in self.people:
                person.queue_free()
        self.people = []
        x = -2.4 * (len(NAMES) - 1) * 0.5
        for name in NAMES:
            person = self.instantiate("Assets/Prefabs/" + name + ".scene.json")
            if person != None:
                person.global_position = Vec3(x, 1.0, 0.0)
                person.rotation_degrees = Vec3(0.0, 180.0, 0.0)
                # The game clears a corpse after 10 s; here it stays until R.
                person.corpse_time = 1000000.0
                cutter = person.get_node("Dismember")
                if cutter != None:
                    cutter.set_rigid(self._rigid)
                self.people.append(person)
            x = x + 2.4
        self.say("NEW ROW")

    def say(self, text):
        self._message = text
        self._timer = 2.0

    # One piece off every person: the first time kills them, as the game's cut does.
    def cut(self, index):
        away = Vec3(0.0, 0.0, -1.0)
        for person in self.people:
            if not person._dead:
                person.die(away, 3.5)
            cutter = person.get_node("Dismember")
            if cutter == None or not cutter.detach(self._parts[index]):
                continue
            piece = person.track(BONES[index])
            if piece != None:
                piece.instantiate("Assets/Prefabs/blood_trail.scene.json")
            if STUMPS[index] != "":
                stump = person.track(STUMPS[index])
                if stump != None:
                    stump.instantiate("Assets/Prefabs/blood_trail.scene.json")
            ragdoll = person.get_node("Ragdoll")
            if ragdoll != None:
                ragdoll.apply_impulse(self._parts[index], Vec3(0.0, 2.0, -4.0), Vec3(0.0, 0.0, 0.0))
        self.say("CUT " + LABELS[index])

    def _process(self, delta):
        for i in range(7):
            if keyboard_check_pressed(KEY_1 + i):
                self.cut(i)
        if keyboard_check_pressed(KEY_X):
            for person in self.people:
                if not person._dead:
                    person.dismember(Vec3(0.0, 0.0, -1.0), 3.5)
            self.say("GAME CUT")
        if keyboard_check_pressed(KEY_Z):
            for person in self.people:
                if not person._dead:
                    person.die(Vec3(0.0, 0.0, -1.0), 3.5)
                cutter = person.get_node("Dismember")
                if cutter != None:
                    p = person.global_position
                    cutter.explode(Vec3(p.x, p.y - 0.3, p.z + 0.6))
            self.say("EVERY PIECE OFF")
        if keyboard_check_pressed(KEY_R):
            self.spawn_row()
        if keyboard_check_pressed(KEY_T):
            self._rigid = not self._rigid
            self.spawn_row()
            self.say("RIGID " + ("ON" if self._rigid else "OFF"))
        self.draw()

    def draw(self):
        dim = Color(0.75, 0.8, 0.9, 0.85)
        cyan = Color(0.4, 0.8, 1.0, 1.0)
        height = hud_height()
        hud_text(12.0, height - 56.0, "1 head  2 L arm  3 R arm  4 L forearm  5 R forearm  6 L leg  7 R leg   X game cut   Z all off   R new row   T rigid " + ("on" if self._rigid else "off"), dim, 16.0)
        camera = self.get_tree().get_root().get_camera_3d()
        if camera != None and self.people != None:
            for i in range(len(self.people)):
                person = self.people[i]
                p = person.global_position
                top = Vec3(p.x, p.y + 1.1, p.z)
                if camera.is_position_behind(top):
                    continue
                s = camera.unproject_position(top)
                name = NAMES[i]
                if person._dead:
                    cutter = person.get_node("Dismember")
                    count = cutter.get_detached_count() if cutter != None else 0
                    name = name + "  (" + str(count) + " off)"
                hud_text(s.x - hud_text_size(name, 16.0).x * 0.5, s.y - 24.0, name, cyan, 16.0)
        if self._timer > 0.0:
            self._timer = self._timer - 1.0 / 60.0
            hud_text(12.0, 12.0 + 40.0, self._message, cyan, 22.0)
