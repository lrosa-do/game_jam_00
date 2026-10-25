import math

# A body being built (the spider, the android when the night starts): three arms of light wind up
# from the floor around it in a spiral, faster and tighter while the light charges, then a flash
# and a burst of sparks as the body appears. height is the centre of the body above the floor
# (the spiral climbs to twice that), radius the width of the spiral.
# The sound drives it: it starts once the spiral shows on screen, and the body appears on its burst,
# sound_lead seconds in (body.appear()), so a stall in the first frames cannot put them apart.
class Materialize(Node3D):
    height = 0.35
    radius = 1.4
    sound = "Assets/Sounds/spider/build.ogg"
    sound_lead = 1.35
    body = None
    life = 1.8  # after the body appears: the sparks and the end of the sound
    peak = 9.0
    _sound = None
    _playing = False
    _age = 0.0
    _clock = 0.0
    _started = False
    _done = False
    _cloud = None
    arms = 3
    rate = 120.0  # particles a second on each arm
    _spin = 0.0
    _rise = 0.0
    _budget = 0.0
    _sparks = None
    _light = None

    def _ready(self):
        self._cloud = self.get_node("Cloud")
        self._sparks = self.get_node("Sparks")
        self._light = self.get_node("Light")
        self._sound = self.get_node("Sound")

    def _process(self, delta):
        # Placed by then: the sparks and the light go to the middle of the body.
        if not self._started:
            self._started = True
            self._sparks.emission_offset = Vec3(0.0, self.height, 0.0)
            self._light.position = Vec3(0.0, self.height + 0.3, 0.0)
        self._age = self._age + delta
        if not self._done:
            self.spiral(delta, self._clock / self.sound_lead if self._playing else 0.0)
        if not self._playing:
            if self._age >= 0.3 and self._cloud.get_active_count() >= 12:
                self._playing = True
                self._sound.stream = self.sound
                self._sound.play()
            return
        self._clock = self._clock + delta
        t = self._clock / self.sound_lead
        if not self._done and t >= 1.0:
            self._done = True
            self._sparks.emit_burst(40)
            if self.body != None:
                self.body.appear()
        if t < 1.0:
            self._light.light_energy = self.peak * 0.35 * t * t * (0.7 + 0.3 * math.sin(self._age * 40.0))
        else:
            self._light.light_energy = self.peak * math.max(1.0 - (self._clock - self.sound_lead) / 0.4, 0.0)
        if self._clock > self.sound_lead + self.life:
            self.queue_free()

    # The arms climb from the floor to the top of the body and start again, each particle left
    # where it was laid with a little drift along the turn: a rising helix. t (0..1) is the charge.
    def spiral(self, delta, t):
        p = self.global_position
        top = self.height * 2.0
        self._spin = self._spin + delta * (5.0 + 9.0 * t)
        self._rise = (self._rise + delta * (1.1 + 1.8 * t)) % 1.0
        r = self.radius * (0.55 - 0.25 * t)
        self._budget = self._budget + delta * self.rate
        count = int(self._budget)
        self._budget = self._budget - count
        for k in range(count):
            # Spread along this frame's turn, so the arms stay continuous at any frame rate.
            back = (count - 1 - k) / math.max(count, 1)
            a0 = self._spin - back * delta * (5.0 + 9.0 * t)
            y = p.y + 0.06 + ((self._rise - back * delta * (1.1 + 1.8 * t)) % 1.0) * top
            for arm in range(self.arms):
                a = a0 + arm * 6.2832 / self.arms
                ca = math.cos(a)
                sa = math.sin(a)
                self._cloud.emit_particle(Vec3(p.x + ca * r, y, p.z + sa * r), Vec3(-sa * 0.5, 0.25, ca * 0.5))
