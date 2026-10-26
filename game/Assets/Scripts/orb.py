import math

# A floater being built (game.spawn_floater): small red stars fall in from all round onto a point
# in the air, a glowing orb swells and throbs there, then it bursts in sparks and the floater
# grows out of it (body.appear()). Shorter and smaller than the materialize of the android and
# her spider: it is the round body of the floater taking shape.
class Orb(Node3D):
    charge = 1.1     # seconds from the first stars to the burst
    radius = 1.3     # where the stars start
    size = 0.45      # the orb's radius at the burst
    rate = 90.0      # stars a second
    star_life = 0.45
    peak = 6.0
    body = None
    _age = 0.0
    _budget = 0.0
    _done = False
    _burst = 0.0
    _stars = None
    _sparks = None
    _orb = None
    _light = None

    def _ready(self):
        self._stars = self.get_node("Stars")
        self._sparks = self.get_node("Sparks")
        self._orb = self.get_node("Orb")
        self._light = self.get_node("Light")
        self._orb.scale = Vec3(0.01, 0.01, 0.01)
        self.get_node("Sound").play()

    def _process(self, delta):
        self._age = self._age + delta
        t = math.clamp(self._age / self.charge, 0.0, 1.0)
        if not self._done:
            self.gather(delta, t)
            s = self.size * (0.1 + 0.9 * t * t) * (1.0 + 0.12 * math.sin(self._age * 30.0))
            self._orb.scale = Vec3(s, s, s)
            self._orb.transparency = 0.5 - 0.3 * t
            self._light.light_energy = self.peak * 0.4 * t * t
            if t >= 1.0:
                self._done = True
                self._sparks.emit_burst(36)
                if self.body != None:
                    self.body.appear()
            return
        # The burst: the orb blows out to nothing, the light flashes and dies.
        self._burst = self._burst + delta
        b = math.clamp(self._burst / 0.25, 0.0, 1.0)
        s = self.size * (1.0 + 0.8 * b)
        self._orb.scale = Vec3(s, s, s)
        self._orb.transparency = 0.2 + 0.8 * b
        self._light.light_energy = self.peak * math.max(1.0 - self._burst / 0.35, 0.0)
        if b >= 1.0:
            self._orb.visible = False
        if self._burst > 1.0:
            self.queue_free()

    # Each star starts on a sphere round the centre and flies straight in, reaching it as it dies;
    # the sphere tightens as the orb charges.
    def gather(self, delta, t):
        p = self.global_position
        r = self.radius * (1.0 - 0.45 * t)
        self._budget = self._budget + delta * self.rate * (0.6 + 0.8 * t)
        count = int(self._budget)
        self._budget = self._budget - count
        for k in range(count):
            dx = math.random(-1.0, 1.0)
            dy = math.random(-1.0, 1.0)
            dz = math.random(-1.0, 1.0)
            l = math.max(math.sqrt(dx * dx + dy * dy + dz * dz), 0.001)
            dx = dx / l
            dy = dy / l
            dz = dz / l
            v = r / self.star_life
            self._stars.emit_particle(Vec3(p.x + dx * r, p.y + dy * r, p.z + dz * r), Vec3(-dx * v, -dy * v, -dz * v))
