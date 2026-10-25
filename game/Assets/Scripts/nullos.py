import math

# NULL//OS as a presence in the night, not a narrator. A child of the Game ("NullOS").
# A civilian she comes near is an order ("A human. Correct it."), and her trust goes up when she
# obeys within the window and down when she does not; the trust shows on the HUD and sets the
# verdict of the night. After the truth (the anomaly stage) it also fights her through the
# building and through her own interface, harder as her trust falls, so obeying it still buys
# quiet: the lights of the room go out, the doors ahead lock, the objective marker and the
# minimap lie, the overclock is cut, a bomb carrier near her is set off, a fake directive comes
# in. One event at a time, spaced by EVENT_GAP seconds (less as her trust falls), each with its
# line, its message on the feed and a glitch; the Game and the Story read nav_lie, dark and
# trust from here.
EVENT_GAP = [26.0, 40.0]
WATCH_WINDOW = 25.0  # seconds she has to obey an order about a civilian
DARK_TIME = 6.0
LIE_TIME = 10.0
OC_TIME = 18.0

class NullOS(Node):
    trust = 100.0
    nav_lie = None  # Vec3 added to the objective on the HUD while the navigation is corrupted
    _game = None
    _story = None
    _player = None
    _next = 20.0
    _war = False
    _dark = 0.0
    _dark_lights = None
    _lie = 0.0
    _oc = 0.0
    _orders = None  # [civilian, time left] of the standing orders about civilians
    _noted = None  # civilians already ordered about
    _recalled = False
    _taunt = 0
    _kills = 0

    def _ready(self):
        self._game = self.get_parent()
        self._dark_lights = []
        self._orders = []
        self._noted = []

    def _process(self, delta):
        if self._game == None:
            self._ready()
        g = self._game
        if g == None or g.story == None or g._player == None:
            return
        self._story = g.story
        self._player = g._player
        real = delta
        if time_scale() > 0.0:
            real = delta / time_scale()
        self.update_effects(real)
        if self._story._stage < 0 or g.briefing() or g.in_intro() or self._story._done or not self._player.alive():
            return
        if not self._war and self._story._truth_said:
            self.turn()
        self.watch(real)
        if self._war:
            self._next = self._next - real
            if self._next <= 0.0:
                self._next = math.random(EVENT_GAP[0], EVENT_GAP[1]) * (0.6 if self.trust < 50.0 else 1.0)
                self.strike()

    # ---- the truth: from this frame on it is against her

    def turn(self):
        self._war = True
        self._next = 6.0
        self._recalled = False

    # ---- watching her: civilians are orders

    def watch(self, real):
        g = self._game
        p = self._player.global_position
        for enemy in g.enemies:
            if not enemy.civilian or enemy._dead or enemy in self._noted:
                continue
            e = enemy.global_position
            if (e.x - p.x) * (e.x - p.x) + (e.z - p.z) * (e.z - p.z) < 36.0 and math.abs(e.y - p.y) < 2.5:
                self._noted.append(enemy)
                self._orders.append([enemy, WATCH_WINDOW])
                g.speak("os_t_see_0" + str(int(math.random(1.0, 2.99))))
                g.show("DIRECTIVE // CORRECT THE HUMAN", 2.5)
        kept = []
        for order in self._orders:
            civilian = order[0]
            if civilian._dead:
                self.trust = math.min(self.trust + 10.0, 100.0)
                g.speak("os_t_good_0" + str(int(math.random(1.0, 2.99))))
                continue
            order[1] = order[1] - real
            if order[1] <= 0.0:
                self.trust = math.max(self.trust - 15.0, 0.0)
                g.speak("os_t_bad_0" + str(int(math.random(1.0, 2.99))))
                g.show("NON-COMPLIANCE LOGGED", 2.5)
                g._glitch = 0.3
                continue
            kept.append(order)
        self._orders = kept

    # ---- fighting her

    def strike(self):
        g = self._game
        if not self._recalled:
            self._recalled = True
            g.speak("os_w_recall_01")
            g.show("ENFORCEMENT UNIT COMPROMISED", 3.0)
            g._glitch = 0.6
            g.noise(self._player.global_position, 30.0)
            return
        choices = []
        if len(self.lights_near(14.0)) > 0:
            choices.append("dark")
        if len(self.doors_near(10.0)) > 0:
            choices.append("lock")
        if self._story._target != None:
            choices.append("nav")
        if self._player._overclock > 1.0:
            choices.append("oc")
        if self.bot_near(9.0) != None:
            choices.append("det")
        if self._story._unit != None and not self._story.unit_free:
            choices.append("fake")
        choices.append("taunt")
        self.fire(choices[int(math.random(0.0, len(choices) - 0.01))])

    def fire(self, name):
        g = self._game
        if name == "dark":
            for light in self.lights_near(14.0):
                self._dark_lights.append([light, light.light_energy])
                light.light_energy = 0.0
            self._dark = DARK_TIME
            g.show("VISUAL FEED DISABLED", 2.5)
            g.speak("os_w_dark_0" + str(int(math.random(1.0, 2.99))))
        elif name == "lock":
            for door in self.doors_near(10.0):
                door.lock()
            g.show("ACCESS REVOKED // KICK [F]", 2.5)
            g.speak("os_w_lock_0" + str(int(math.random(1.0, 2.99))))
        elif name == "nav":
            a = math.random(0.0, 6.283)
            self.nav_lie = Vec3(math.cos(a) * 16.0, 0.0, math.sin(a) * 16.0)
            self._lie = LIE_TIME
            g.show("NAVIGATION CORRUPTED", 2.5)
            g.speak("os_w_nav_0" + str(int(math.random(1.0, 2.99))))
        elif name == "oc":
            self._oc = OC_TIME
            g.show("OVERCLOCK REVOKED", 2.5)
            g.speak("os_w_oc_01")
        elif name == "det":
            bot = self.bot_near(9.0)
            if bot != None:
                g.speak("os_w_det_0" + str(int(math.random(1.0, 2.99))))
                g.show("UPGRADE ACTIVATED", 2.0)
                bot.explode()
        elif name == "fake":
            g.show("DIRECTIVE // TERMINATE UNIT-7", 3.5)
            g.speak("os_w_fake_01")
        else:
            self._taunt = self._taunt + 1
            g.speak("os_w_taunt_0" + str(1 + self._taunt % 2))
        g._glitch = math.max(g._glitch, 0.45)

    def update_effects(self, real):
        if self._dark > 0.0:
            self._dark = self._dark - real
            if self._dark <= 0.0:
                for pair in self._dark_lights:
                    pair[0].light_energy = pair[1]
                self._dark_lights = []
        if self._lie > 0.0:
            self._lie = self._lie - real
            if self._lie <= 0.0:
                self.nav_lie = None
        if self._oc > 0.0:
            self._oc = self._oc - real
            self._player._overclock = 0.0

    # ---- what is round her

    def lights_near(self, reach):
        out = []
        p = self._player.global_position
        for light in self._game._lights:
            l = light.global_position
            if (l.x - p.x) * (l.x - p.x) + (l.z - p.z) * (l.z - p.z) < reach * reach and math.abs(l.y - p.y) < 5.0:
                out.append(light)
        return out

    def doors_near(self, reach):
        out = []
        p = self._player.global_position
        for body in self.get_tree().get_nodes_in_group("breakable"):
            item = body.get_node("Breakable")
            if item == None or item.kind != "door" or item._broken or item.locked or item._hinge == None:
                continue
            c = item.center()
            if (c.x - p.x) * (c.x - p.x) + (c.z - p.z) * (c.z - p.z) < reach * reach and math.abs(c.y - p.y) < 3.0:
                out.append(item)
        return out

    def bot_near(self, reach):
        p = self._player.global_position
        for bot in self.get_tree().get_nodes_in_group("bot"):
            if not bot.alive():
                continue
            b = bot.global_position
            d = (b.x - p.x) * (b.x - p.x) + (b.z - p.z) * (b.z - p.z)
            if d > 9.0 and d < reach * reach and math.abs(b.y - p.y) < 3.0:
                return bot
        return None
