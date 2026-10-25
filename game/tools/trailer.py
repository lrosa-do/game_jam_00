# The trailer: copy to Assets/Scripts/zz_trailer.py, add it as a Node "Trailer" and a Camera3D
# "ZCam" (near 0.05) to a copy of Scenes/game.scene.json with intro False, and run
#   runner Scenes/zz_trailer.scene.json --headless --fps 30 --frames 1500 --size 1280x720 --record DIR
# then tools/trailer.sh cuts the video with the narration (the engine's sounds are not recorded).
# Each shot: start and end seconds, the stage to jump to (-1 keeps the game as it is), the
# camera ("free": a dolly from one point to another looking at a point or at her; "follow": a
# dolly of offsets round the spider; "cell": outside Unit-7's cell door looking in; "game": her
# own camera), where she stands, and actions at their seconds. A fill light rides with the free
# camera: the precinct is lit for play, not for close-ups.
import math

FPS = 30.0


class Trailer(Node):
    _f = 0
    _shot = -1
    _cam = None
    _game = None
    _done = None
    _shots = None
    _actions = None
    _light = None

    def shot_table(self):
        # [t0, t1, stage, mode, from xyz, to xyz, look, player xyz yaw or None]
        return [
            [0.0, 7.0, -1, "free", [3.0, -2.6, 88.5], [5.8, -3.0, 87.0], "player", None],
            [7.0, 10.0, -1, "free", [5.0, -2.55, 85.6], [6.9, -2.75, 85.15], "face", None],
            [10.0, 15.5, 2, "free", [-0.9, 1.7, -6.5], [-0.9, 1.5, -10.5], [-0.9, 0.9, -16.0], None],
            [15.5, 20.4, 0, "free", [-29.0, -2.4, 90.0], [-32.5, -3.2, 88.0], [-38.0, -3.6, 83.0], None],
            [20.4, 23.2, 1, "free", [-18.4, -5.0, 69.0], [-20.0, -5.3, 71.0], [-20.0, -2.4, 77.3], None],
            [23.2, 26.0, -1, "free", [-21.0, -8.0, 47.5], [-22.5, -8.3, 45.5], [-24.0, -8.6, 43.3], [-26.0, -10.0, 38.0, 20.0]],
            [26.0, 30.5, 2, "game", None, None, None, [-0.9, 0.0, -11.0, 0.0]],
            [30.5, 35.0, 3, "game", None, None, None, "target"],
            [35.0, 39.9, 2, "follow", [2.2, 0.9, 2.2], [-2.2, 0.6, 2.4], "spider", [-0.9, 0.0, -9.0, 0.0]],
            [39.9, 44.3, 4, "free", [-5.9, -7.55, 2.45], [-6.45, -7.72, 2.0], [-7.9, -8.0, 0.55], None],
            [44.3, 50.0, 7, "game", None, None, None, None],
        ]

    def action_table(self):
        # [second, action]
        return [
            [12.0, "scan"], [17.0, "car"], [17.0, "slow"], [19.6, "normal"],
            [23.3, "sleepers"], [26.2, "alert"], [27.0, "dive"], [27.0, "slow"],
            [27.4, "fire"], [27.9, "fire"], [28.4, "fire"], [29.0, "fire"], [30.2, "normal"],
            [30.6, "hack"], [31.0, "alert"],
            [35.3, "alert"], [36.0, "spider"],
            [44.4, "alert"], [46.4, "turn_unit"], [48.0, "turn_unit"],
        ]

    def _ready(self):
        self._shots = self.shot_table()
        self._actions = self.action_table()
        self._done = []
        for i in range(len(self._actions)):
            self._done.append(False)

    def _process(self, delta):
        g = self._game
        if g == None:
            g = self.get_tree().get_first_node_in_group("game")
            if g == None or g.story == None or g._player == None:
                return
            self._game = g
            self._cam = g.get_node("ZCam")
            self._light = g.get_node("ZLight")
            g.debug_keys = False
        self._f = self._f + 1
        t = self._f / FPS
        g._wake = 0.0
        g._brief_state = 3
        g._player._health = g._player.max_health
        shot = -1
        for i in range(len(self._shots)):
            if t >= self._shots[i][0] and t < self._shots[i][1]:
                shot = i
        if shot < 0:
            return
        if shot != self._shot:
            self._shot = shot
            print("TRAILER shot", shot, "at", t, "delta", delta)
            self.begin_shot(self._shots[shot])
        self.update_camera(self._shots[shot], t)
        for i in range(len(self._actions)):
            if not self._done[i] and t >= self._actions[i][0]:
                self._done[i] = True
                self.act(self._actions[i][1])

    def begin_shot(self, s):
        g = self._game
        st = g.story
        set_time_scale(1.0)
        if s[2] >= 0:
            st.jump(s[2])
            g.show("", 0.0)
        pl = g._player
        where = s[7]
        if where == "target":
            if st._target != None:
                spot = g.nav.nearest_point(st._target)
                if spot != None:
                    pl.respawn(Vec3(spot.x, spot.y + 1.1, spot.z), self.yaw_to(spot, st._target))
                    pl.global_position = Vec3(spot.x - self.dir_to(spot, st._target).x * 1.6, spot.y + 1.1, spot.z - self.dir_to(spot, st._target).z * 1.6)
        elif where != None:
            pl.respawn(Vec3(where[0], where[1] + 1.1, where[2]), where[3])
        g._floors = 0.0
        g.hud = s[3] == "game"
        if self._light != None:
            self._light.light_energy = 0.0 if s[3] == "game" else 1.4
        if s[3] == "game":
            cam = pl.camera()
            if cam != None:
                cam.make_current()
        else:
            self._cam.make_current()

    def dir_to(self, a, b):
        dx = b.x - a.x
        dz = b.z - a.z
        l = math.max(math.sqrt(dx * dx + dz * dz), 0.001)
        return Vec3(dx / l, 0.0, dz / l)

    # Her yaw that faces b from a (yaw 0 faces -z, 90 faces -x).
    def yaw_to(self, a, b):
        d = self.dir_to(a, b)
        return math.degrees(math.atan2(-d.x, -d.z))

    def update_camera(self, s, t):
        if s[3] == "game":
            return
        g = self._game
        u = math.clamp((t - s[0]) / math.max(s[1] - s[0], 0.001), 0.0, 1.0)
        u = u * u * (3.0 - 2.0 * u)
        a = s[4]
        b = s[5]
        offset = Vec3(a[0] + (b[0] - a[0]) * u, a[1] + (b[1] - a[1]) * u, a[2] + (b[2] - a[2]) * u)
        anchor = Vec3(0.0, 0.0, 0.0)
        target = None
        look = s[6]
        if s[3] == "follow":
            # Round the spider (or her, until it is built), the offsets turning with nothing.
            sp = g.spider
            anchor = sp.global_position if sp != None else g._player.global_position
            target = Vec3(anchor.x, anchor.y + 0.1, anchor.z)
        elif s[3] == "cell":
            # Outside the cell door, the offsets along the line from Unit-7 out through the door.
            st = g.story
            if st._cell != None and st._unit != None:
                c = st._cell.center()
                uu = st._unit.global_position
                d = self.dir_to(uu, c)
                anchor = Vec3(c.x, c.y - 0.6, c.z)
                offset = Vec3(d.x * offset.x, offset.y, d.z * offset.x)
                target = Vec3(uu.x, uu.y + 0.3, uu.z)
            else:
                anchor = g._player.global_position
                target = anchor
        elif look == "player":
            p = g._player.global_position
            target = Vec3(p.x, p.y + 0.4, p.z)
        elif look == "face":
            p = g._player.global_position
            target = Vec3(p.x, p.y + 0.55, p.z)
        else:
            target = Vec3(look[0], look[1], look[2])
        self._cam.global_position = Vec3(anchor.x + offset.x, anchor.y + offset.y, anchor.z + offset.z)
        self._cam.look_at(target, Vec3(0.0, 1.0, 0.0))
        if self._light != None:
            self._light.global_position = Vec3(anchor.x + offset.x, anchor.y + offset.y + 0.5, anchor.z + offset.z)

    def act(self, name):
        g = self._game
        st = g.story
        pl = g._player
        if name == "scan":
            g.trigger_scan()
        elif name == "car":
            car = st.test_car()
            if car != None:
                car.explode_car()
        elif name == "slow":
            set_time_scale(0.3)
        elif name == "normal":
            set_time_scale(1.0)
        elif name == "sleepers":
            for name in ["Sleeper_1", "Sleeper_2"]:
                bot = g.get_node(name)
                if bot != None and bot.alive():
                    bot.wake()
        elif name == "alert":
            g.noise(pl.global_position, 22.0)
        elif name == "dive":
            pl._fwd = 1.0
            pl._side = 0.0
            pl.start_dodge()
        elif name == "fire":
            cam = pl.camera()
            if cam != None:
                pl._cooldown = 0.0
                pl.fire(cam)
        elif name == "hack":
            st.autoplay = True
        elif name == "spider":
            if g.spider != None:
                g.spider._cool = 0.0
                g.spider.command()
        elif name == "turn_unit":
            if st._unit != None:
                pl._yaw = self.yaw_to(pl.global_position, st._unit.global_position)
