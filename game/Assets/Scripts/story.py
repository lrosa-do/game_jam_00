import math

# The night of the android, stage by stage down the police station. NULL//OS, the station's AI,
# has gone wrong: it took the humans for the threat and wires them into bombs. She is one of its
# units and obeys, until the interrogation terminal shows her what the protocol is and that
# Unit-7, the AI that tried to stop it, is locked in the cells. Freeing it ends the night.
# Each stage: a title, two order lines, what ends it and the line NULL//OS reads the orders with
# (Assets/Sounds/voice/nullos/<id>.ogg). The night starts in the alley outside
# the garage and ends through the same door.
STAGES = [
    ["NULL//OS // CALIBRATION", ["TEST YOUR MOTOR FUNCTIONS", "THEN DESTROY THE TEST VEHICLE"], "training", "os_brief_00"],
    ["NULL//OS // DEPLOYMENT", ["ENTER THE PRECINCT THROUGH THE GARAGE", "BOMB CARRIERS BLOW ON CONTACT. SHOOT FIRST"], "enter", "os_brief_0b"],
    ["DIRECTIVE 01 // ACTIVATION", ["RETRIEVE THE ACCESS CARD", "THE DESK SERGEANT CARRIES IT"], "card", "os_brief_01"],
    ["DIRECTIVE 02 // INTERROGATION", ["HACK THE INTERROGATION TERMINAL", "HOLD [E] AT THE SCREEN"], "hack", "os_brief_02"],
    ["NULL//OS // ANOMALY", ["FIND UNIT-7 IN THE CELL BLOCK", "THE HUMANS ARE WIRED TO BOMBS"], "reach", "os_brief_03"],
    ["NULL//OS // TRANSPORTS", ["DESTROY THE BOMB TRANSPORTS", "THEY LEAVE FOR THE CITY"], "transports", "os_brief_04"],
    ["NULL//OS // KILL SWITCH", ["UPLOAD THE KILL SWITCH AT SECURITY", "IT SHUTS DOWN UNIT-7'S INHIBITOR. HOLD [E]"], "upload", "os_brief_05"],
    ["UNIT-7 // RESCUE", ["GET TO UNIT-7'S CELL", "STAND CLEAR OF THE BARS"], "free", "os_brief_06"],
    ["UNIT-7 // WAY OUT", ["TAKE UNIT-7 TO THE GARAGE EXIT", "AI GONE WRONG. NOW GONE RIGHT."], "escort", "os_brief_07"],
]

# The calibration in the alley, one move at a time: what to do and the keys for it. The last step
# is the car parked in the street (a breakable "car" past the garage door).
TRAINING = [
    ["MOVE", "W A S D"],
    ["JUMP", "SPACE"],
    ["DIVE", "C OR MIDDLE MOUSE + DIRECTION"],
    ["ZOOM", "HOLD Z"],
    ["OVERCLOCK // SLOW TIME", "HOLD RIGHT MOUSE"],
    ["SCAN FOR TARGETS", "V"],
    ["DESTROY THE TEST VEHICLE", "LEFT MOUSE"],
]

# Two bomb carriers lying at the foot of the garage ramp, one on his back and one on his front:
# they get up and run at her when she sees them, hears a shot or is shot. They are the scene's
# Sleeper_1 and Sleeper_2; this (x, z, pose, yaw) makes them when the scene has none.
GARAGE_SLEEPERS = [[-24.0, 50.0, "back", 30.0], [-15.0, 47.0, "front", 200.0]]

# Debug: where she starts when a stage is jumped to (x, y, z, yaw), snapped to the walkable floor.
STAGE_STARTS = [[8.0, -4.7, 85.0, 90.0], [-14.0, -4.7, 82.0, 50.0], [-0.9, 0.0, -8.1, 0.0], [-0.9, 0.0, -6.0, 0.0], [-3.0, -10.0, -9.0, 0.0], [-12.0, -10.0, 40.0, 0.0], [0.0, -10.0, -11.0, 180.0], [-3.0, -10.0, -5.4, -90.0], [-3.0, -10.0, -5.4, -90.0]]

UNIT_PREFAB = "Assets/Prefabs/unit7.scene.json"

# The meeting in the briefing room (floor -5): Captain Bravura talks by the projector, cops sit
# on the plastic chairs (seat centres taken from the level) facing him, one smokes by the wall.
MEETING_FLOOR = -5.0
MEETING_CHAIRS = [[4.44, -16.12], [6.03, -15.74], [6.95, -15.99], [4.49, -13.32], [6.87, -13.39]]
MEETING_CLIPS = ["Custom_Sit_Listen", "Custom_Sit_Listen_Action_A", "Custom_Sit_Listen_Action_B", "Custom_Sit_Listen_Action_C", "Custom_Sit_Listen_Action_D"]
MEETING_COPS = ["Assets/Prefabs/cop_a.scene.json", "Assets/Prefabs/cop_b.scene.json", "Assets/Prefabs/security.scene.json"]

# Boxes of the level, x0 x1 y0 y1 z0 z1.
REGIONS = {
    "floor0": [-10.0, 8.2, -1.0, 4.5, -20.2, 10.0],
    "interrogation": [-10.0, -1.1, -0.5, 4.5, -5.0, 9.9],
    "level5": [-10.4, 10.1, -6.0, -0.6, -20.2, 9.0],
    "cells": [-17.9, 6.9, -11.5, -5.6, -23.4, 14.1],
    "entrance": [-0.5, 6.9, -11.5, -5.6, -17.9, -10.0],
    "garage": [-55.1, 14.9, -16.0, -9.5, -22.5, 93.5],
}

COPS = ["Assets/Prefabs/cop_a.scene.json", "Assets/Prefabs/cop_b.scene.json", "Assets/Prefabs/cop_female.scene.json", "Assets/Prefabs/security.scene.json"]

class Story(Node):
    start_stage = 0
    autoplay = False
    hack_time = 8.0
    upload_time = 30.0
    max_cops = 6
    max_bots = 2
    checkpoint = None
    checkpoint_yaw = 0.0
    unit_free = False
    _unit = None
    _cell = None
    _lost = -1.0
    _game = None
    _player = None
    _stage = -1
    _ready_to_start = False
    _target = None
    _target_label = ""
    _progress = 0.0
    _count = 0
    _needed = 0
    _items = None
    _wave = 0.0
    _waves_left = 0
    _spawned = None
    _done = False
    _end = 0.0
    _hacking = False
    _step = 0
    _ambush = False
    _gate = None  # the garage gate once knocked down, watched until it lies flat
    _gate_check = 0.0
    _breakout = -1.0
    _truth_said = False
    _waved = False
    _cell_talk = False
    _meeting_said = False
    _step_time = 0.0

    def _ready(self):
        self._game = self.get_parent()
        self._player = self.get_tree().get_first_node_in_group("player")
        self._spawned = []

    def _process(self, delta):
        if self._game == None or self._player == None:
            self._ready()
            return
        if self._stage >= 0 and self._game.debug_keys:
            self.debug_keys()
        nav = self._game.nav
        if self._stage < 0:
            # The first stage waits for the floor, which the waves and markers need.
            # The cell, the meeting and the civilians are set up behind the intro.
            if nav != None and nav.is_baked():
                if not self._ready_to_start:
                    self._ready_to_start = True
                    self.setup_cell()
                    self.setup_meeting()
                    self.setup_civilians()
                    self.setup_garage()
                if not self._game.in_intro():
                    self.begin(self.start_stage)
            return
        if self._done:
            self._end = self._end + delta
            return
        # Captain Bravura's briefing plays when she walks into the meeting room.
        if not self._meeting_said and self._player != None:
            q = self._player.global_position
            if math.abs(q.y - MEETING_FLOOR) < 2.0 and (q.x - 5.4) * (q.x - 5.4) + (q.z + 13.0) * (q.z + 13.0) < 49.0:
                self._meeting_said = True
                for line in ["br_meet_01", "br_meet_02", "br_meet_03"]:
                    self._game.speak(line)
        if self._lost >= 0.0:
            self._lost = self._lost - delta
            if self._lost < 0.0:
                self.restore_unit()
            return
        self.watch_gate(delta)
        kind = STAGES[self._stage][2]
        if kind == "training":
            self.update_training(delta)
        elif kind == "enter":
            if not self._ambush and not self._game.briefing():
                self.ambush()
            p = self._player.global_position
            if p.z < 75.0 or (self._target != None and self.distance(self._target) < 2.5):
                self.next()
        elif kind == "card":
            self.update_card()
        elif kind == "hack":
            self.update_hold(delta, self.hack_time)
        elif kind == "reach":
            # After the anomaly orders, she knows: the line that turns the night.
            if not self._truth_said and not self._game.briefing():
                self._truth_said = True
                self._game.say("truth", 1.0)
            # Unit-7 calls out when she comes near the cells.
            if not self._waved and self._target != None and self.distance(self._target) < 10.0:
                self._waved = True
                self._game.speak("u7_wave_0" + str(int(math.random(1.0, 2.99))))
            # At the bars: she offers to break it open, Unit-7 has no power left and sends her
            # on (the kill switch); the stage moves on when they are done talking.
            if not self._cell_talk and self._target != None and self.distance(self._target) < 4.0:
                self._cell_talk = True
                self._game.show("CELL SEALED BY NULL//OS", 3.0)
                self._game._glitch = 0.6
                self._game._voice.queue = []
                for line in ["an_cell_01", "u7_cell_01", "u7_cell_02", "an_cell_02"]:
                    self._game.speak(line)
            if self._cell_talk and not self._game.talking():
                self.next()
        elif kind == "free":
            self.update_free(delta)
        elif kind == "transports":
            self.update_transports()
        elif kind == "upload":
            self.update_hold(delta, self.upload_time)
        elif kind == "escort":
            self.update_escort()
        self.update_waves(delta)

    # ---- stages

    # ---- debug: F1..F9 jump to a stage, Page Down skips the current one (game.debug_keys)

    def debug_keys(self):
        if keyboard_check_pressed(KEY_PAGE_DOWN):
            self._game.show("DEBUG // STAGE SKIPPED", 1.5)
            self.next()
            return
        for i in range(len(STAGES)):
            if keyboard_check_pressed(KEY_F1 + i):
                self.jump(i)
                return

    def stage_of(self, kind):
        for i in range(len(STAGES)):
            if STAGES[i][2] == kind:
                return i
        return -1

    # The stages before index count as played: the test car and the transports blown, the garage
    # gate down, the story lines of the cells said. The keycard needs nothing: holding it only
    # ever finished its own stage.
    def catch_up(self, index):
        if index > self.stage_of("training"):
            car = self.test_car()
            if car != None and not car._broken:
                car.explode_car()
        if index > self.stage_of("enter"):
            self._ambush = True
            door = self.exit_item()
            if door != None and not door._broken:
                door.topple(Vec3(0.0, 0.0, 1.0))
        past = index > self.stage_of("reach")
        self._truth_said = past
        self._waved = past
        self._cell_talk = past
        if index > self.stage_of("transports"):
            self.setup_transports()
            for car in self._items:
                if not car._broken:
                    car.explode_car()
            self._items = []

    def jump(self, index):
        free_at = 0
        for i in range(len(STAGES)):
            if STAGES[i][2] == "free":
                free_at = i
        # Past the rescue Unit-7 is out and at her side.
        if index > free_at and self._cell != None and not self._cell._broken:
            self._cell.blow_door(Vec3(-1.0, 0.2, 0.0), 260.0)
        start = STAGE_STARTS[index]
        spot = self._game.nav.nearest_point(Vec3(start[0], start[1], start[2]))
        yaw = start[3]
        # The cell's stages start in front of Unit-7's cell door, wherever it was placed.
        kind = STAGES[index][2]
        if (kind == "reach" or kind == "free" or kind == "escort") and self._cell != None and self._unit != None:
            c = self._cell.center()
            u = self._unit.global_position
            dl = math.max(math.sqrt((c.x - u.x) * (c.x - u.x) + (c.z - u.z) * (c.z - u.z)), 0.01)
            spot = self._game.nav.nearest_point(Vec3(c.x + (c.x - u.x) / dl * 2.5, c.y - 1.2, c.z + (c.z - u.z) / dl * 2.5))
            yaw = math.degrees(math.atan2((c.x - u.x) / dl, (c.z - u.z) / dl))
        if spot == None:
            spot = Vec3(start[0], start[1], start[2])
        self._player.respawn(Vec3(spot.x, spot.y + 1.1, spot.z), yaw)
        self._game._floors = 0.0
        # Whoever was shooting at her where she came from forgets it.
        if self._game.enemies != None:
            for enemy in self._game.enemies:
                if not enemy._dead:
                    enemy.calm()
        # After the move, so the blasts of the stages played go off far from her.
        self.catch_up(index)
        if index > free_at and self._unit != None:
            self.unit_free = True
            self._unit.global_position = Vec3(spot.x + 1.0, spot.y + 0.85, spot.z)
            self._unit.wake()
        self._done = False
        self._lost = -1.0
        self.begin(index)
        self._game.show("DEBUG // STAGE " + str(index + 1), 1.5)

    def begin(self, index):
        self._stage = index
        self._progress = 0.0
        self._count = 0
        self._target = None
        self._target_label = ""
        self._waves_left = 0
        self._hacking = False
        stage = STAGES[index]
        game = self._game
        game.mission = stage[0]
        game._orders = stage[1]
        game.stage_kills = 0
        game.brief_voice = "Assets/Sounds/voice/nullos/" + stage[3] + ".ogg"
        game._brief_state = 0
        game._brief = 0.0
        game._brief_close = 0.0
        if index > 0:
            game.brief_delay = 0.4
            game._glitch = 0.5
        p = self._player.global_position
        self.checkpoint = Vec3(p.x, p.y, p.z)
        self.checkpoint_yaw = self._player._yaw
        kind = stage[2]
        if kind == "training":
            self._step = 0
            self._step_time = 0.0
            car = self.test_car()
            self._items = [car] if car != None else []
        elif kind == "enter":
            self._ambush = False
            self._target = self.exit_door()
            self._target_label = "GARAGE"
        elif kind == "card":
            self.setup_card()
        elif kind == "hack":
            self._target = self.screen_in(REGIONS["interrogation"])
            self._target_label = "TERMINAL"
        elif kind == "reach":
            game.place_bots(REGIONS["level5"], 3)
            self.spawn_cops(REGIONS["level5"], 2)
            self._target = self._cell.center() if self._cell != None else Vec3(5.0, -8.0, -14.0)
            self._target_label = "UNIT-7"
        elif kind == "free":
            if self._cell != None:
                self._target = self._cell.center()
            self._target_label = "UNIT-7"
            self._breakout = -1.0
            game.show("INHIBITOR OFFLINE", 2.5)
            self.spawn_cops(REGIONS["cells"], 2)
        elif kind == "transports":
            self.setup_transports()
            self._waves_left = 4
            self._wave = 6.0
        elif kind == "upload":
            self._target = self.screen_in(REGIONS["entrance"])
            self._target_label = "SECURITY"
        elif kind == "escort":
            self.setup_exit()
            game.speak("u7_follow_01")
            self._waves_left = 3
            self._wave = 8.0

    def next(self):
        if self._stage + 1 < len(STAGES):
            self.begin(self._stage + 1)
        else:
            self.finish()

    def finish(self):
        self._game.speak("os_end_01")
        self._game.speak("u7_end_01")
        self._done = True
        self._target = None
        self._game.music_final = True
        self._game.show("AI GONE WRONG // NOW GONE RIGHT", 8.0)
        self._game._glitch = 1.2
        self._game._flash = 0.3

    # Card: the sergeant is the cop nearest the start; the card drops when he dies.
    def setup_card(self):
        best = None
        best_d = 1000000.0
        for enemy in self._game.enemies:
            d = self.distance(enemy.global_position)
            if d < best_d:
                best_d = d
                best = enemy
        if best != None:
            best.carries_card = True
            self._items = [best]
            self._target_label = "SERGEANT"

    def update_card(self):
        if self._items != None and len(self._items) > 0:
            sergeant = self._items[0]
            if sergeant._dead:
                self._items = []
                self._target_label = "ACCESS CARD"
            else:
                p = sergeant.global_position
                self._target = Vec3(p.x, p.y + 1.1, p.z)
        else:
            card = self.get_tree().get_first_node_in_group("pickup_keycard")
            self._target = card.global_position if card != None else None

    def got_card(self):
        if self._stage >= 0 and STAGES[self._stage][2] == "card":
            self._game.show("ACCESS GRANTED", 2.0)
            self.next()

    # Hold E by the target for a number of seconds; cops answer while it runs.
    # The terminal she has to hack is shot to pieces: NULL//OS ends the session (the death and the
    # respawn at the start of the stage are the game over the night has).
    def terminal_broken(self, center):
        kind = STAGES[self._stage][2] if self._stage >= 0 else ""
        if (kind != "hack" and kind != "upload") or self._target == None or self._done:
            return
        t = self._target
        if (t.x - center.x) * (t.x - center.x) + (t.z - center.z) * (t.z - center.z) > 1.0:
            return
        player = self._player
        if player == None or not player.alive():
            return
        player.take_damage(100000.0, player.global_position, Vec3(0.0, 0.0, 1.0), 2.0, None)
        self._game.show("TERMINAL LOST // GAME OVER", 3.5)
        self._game._glitch = 1.5
        self._game._flash = 0.3

    def update_hold(self, delta, seconds):
        self._hacking = False
        if self._target == None:
            return
        near = self.distance(self._target) < 2.2
        if near and (keyboard_check(KEY_E) or self.autoplay) and self._player.alive():
            if self._progress <= 0.0:
                self._waves_left = 3 if seconds < 20.0 else 6
                self._wave = 1.5
            self._hacking = True
            self._progress = self._progress + delta / seconds
            if self._progress >= 1.0:
                if STAGES[self._stage][2] == "hack":
                    self._game.show("UNIT-7 LOCATED // CELL BLOCK", 2.5)
                else:
                    self._game.show("NULL//OS DELETED", 3.0)
                    self._game._glitch = 1.5
                self.next()

    # Unit-7's cell, chosen at the start: a door of the cell block, sealed by NULL//OS until the
    # kill switch is in; Unit-7 waits behind it.
    # Unit-7's cell: the door of the cell block nearest to where Unit-7 was placed in the scene
    # (without one, the barred cell door), locked until it breaks out.
    def setup_cell(self):
        self._unit = self.placed("Unit7")
        door = None
        best = 1000000.0
        for body in self.get_tree().get_nodes_in_group("breakable"):
            item = body.get_node("Breakable")
            if item == None or item.kind != "door" or item._broken:
                continue
            c = item.center()
            if not self.inside(c, REGIONS["cells"]):
                continue
            if self._unit != None:
                u = self._unit.global_position
                d = (c.x - u.x) * (c.x - u.x) + (c.y - u.y) * (c.y - u.y) + (c.z - u.z) * (c.z - u.z)
            else:
                d = self.distance(c) - (100.0 if body.name.find("cell_door") >= 0 else 0.0)
            if d < best:
                best = d
                door = item
        self._cell = door
        if door != None:
            door.lock()
            if self._unit == None:
                self.place_unit(door.center())

    # Unit-7 stands in the cell behind the door: the floor beyond it, away from her.
    def place_unit(self, door):
        p = self._player.global_position
        dx = door.x - p.x
        dz = door.z - p.z
        length = math.max(math.sqrt(dx * dx + dz * dz), 0.01)
        spot = self._game.nav.nearest_point(Vec3(door.x + dx / length * 1.6, door.y - 0.9, door.z + dz / length * 1.6))
        if spot == None:
            spot = Vec3(door.x + dx / length * 1.6, door.y - 0.9, door.z + dz / length * 1.6)
        unit = self._game.instantiate(UNIT_PREFAB)
        if unit != None:
            unit.global_position = Vec3(spot.x, spot.y + 0.85, spot.z)
            yaw = math.degrees(math.atan2(dx / length, dz / length))
            unit.rotation_degrees = Vec3(0.0, yaw, 0.0)
            unit._yaw = yaw
        self._unit = unit

    # With the inhibitor down Unit-7 is at full strength: when she gets close it charges up and
    # blows the bars out itself.
    def update_free(self, delta):
        door = self._cell
        if door != None and not door._broken:
            if self._breakout < 0.0:
                if self._target != None and self.distance(self._target) < 8.0:
                    self._breakout = 1.6
                    self._game.show("UNIT-7 // POWER RESTORED", 2.0)
                    self._game._glitch = 0.6
                return
            self._breakout = self._breakout - delta
            if self._breakout > 0.0:
                return
            c = door.center()
            out = Vec3(-1.0, 0.0, 0.0)
            if self._unit != None:
                u = self._unit.global_position
                dl = math.max(math.sqrt((c.x - u.x) * (c.x - u.x) + (c.z - u.z) * (c.z - u.z)), 0.01)
                out = Vec3((c.x - u.x) / dl, 0.3, (c.z - u.z) / dl)
            door.blow_door(out, 260.0)
            self._game.speak("u7_free_0" + str(int(math.random(1.0, 2.99))))
            self._game.boom(c, "Assets/Sounds/explosion/explosion_medium_A.ogg", 8.0)
            self._game._flash = 0.25
            return
        self.unit_free = True
        if self._unit != None:
            self._unit.wake()
        self._game.show("UNIT-7 ONLINE", 2.5)
        self._game._glitch = 1.0
        self.next()

    # Transports: the vans of the garage (and police cars to make three).
    # ---- calibration

    def test_car(self):
        for body in self.get_tree().get_nodes_in_group("breakable"):
            item = body.get_node("Breakable")
            if item != None and item.kind == "car" and not item._broken and item.center().z > 77.0:
                return item
        return None

    def update_training(self, delta):
        player = self._player
        # ENTER skips the calibration (not the ENTER that closes the orders: they are still
        # closing on that frame).
        if self._step < len(TRAINING) and not self._game.briefing() and keyboard_check_pressed(KEY_ENTER):
            self._target = None
            self._game.show("CALIBRATION SKIPPED", 2.0)
            self.next()
            return
        if self._step >= len(TRAINING):
            # A moment to watch the car burn.
            self._step_time = self._step_time + delta
            if self._step_time > 2.5:
                self.next()
            return
        done = False
        if self._step == 0:
            if player._moving > 0.1:
                self._step_time = self._step_time + delta
            done = self._step_time > 1.0
        elif self._step == 1:
            done = player._air > 0.3 and player._dodge <= 0.0
        elif self._step == 2:
            done = player._dodge > 0.0
        elif self._step == 3:
            if player._zoom > 0.9:
                self._step_time = self._step_time + delta
            done = self._step_time > 0.5
        elif self._step == 4:
            if player._draining:
                self._step_time = self._step_time + delta
            done = self._step_time > 0.3
        elif self._step == 5:
            done = self._game._scan >= 0.0
        else:
            car = self._items[0] if len(self._items) > 0 else None
            if car == None or car._broken:
                done = True
            else:
                self._target = car.center()
                self._target_label = "TEST VEHICLE"
        if done:
            self._step = self._step + 1
            self._step_time = 0.0
            self._game._snd_beep.play()
            if self._step >= len(TRAINING):
                self._target = None
                self._game.show("CALIBRATION COMPLETE", 2.5)

    def draw_training(self):
        y = 34.0
        for i in range(len(TRAINING)):
            step = TRAINING[i]
            if i < self._step:
                text = "> " + step[0] + "  OK"
                color = Color(0.4, 1.0, 0.5, 0.75)
            elif i == self._step:
                text = "> " + step[0] + "   [" + step[1] + "]"
                color = Color(1.0, 1.0, 1.0, 0.75 + 0.25 * math.sin(self._game._clock * 6.0))
            else:
                text = "> " + step[0]
                color = Color(0.45, 0.9, 1.0, 0.35)
            hud_rect(12.0, y - 2.0, 3.0, 18.0, color)
            hud_text(22.0, y, text, color, 15.0 if i == self._step else 13.0)
            y = y + 22.0
        if self._step < len(TRAINING):
            hud_text(22.0, y + 6.0, "[ENTER] SKIP CALIBRATION", Color(1.0, 1.0, 1.0, 0.45), 12.0)

    def setup_transports(self):
        cars = []
        for body in self.get_tree().get_nodes_in_group("breakable"):
            item = body.get_node("Breakable")
            # The cars of the garage; the one in the street was the calibration.
            if item == None or item.kind != "car" or item._broken or item.center().z > 77.0:
                continue
            cars.append(item)
        chosen = []
        for car in cars:
            if car.get_parent().name.find("van") >= 0:
                chosen.append(car)
        for car in cars:
            if len(chosen) >= 3:
                break
            if not car in chosen:
                chosen.append(car)
        self._items = chosen
        self._needed = len(chosen)

    def update_transports(self):
        count = 0
        nearest = None
        best = 1000000.0
        for car in self._items:
            if car._broken:
                count = count + 1
                continue
            c = car.center()
            d = self.distance(c)
            if d < best:
                best = d
                nearest = c
        self._count = count
        self._target = nearest
        self._target_label = "TRANSPORT"
        if count >= self._needed:
            self._game.show("TRANSPORTS DOWN", 2.5)
            self.next()

    # The garage exit; the night ends when she and Unit-7 both stand by it.
    def setup_exit(self):
        self._target_label = "EXIT"
        self._target = self.exit_door()

    # The people of the night that are placed in the scene (seen and moved in the editor): Unit7,
    # Prisoner, Towel, Sleeper_1, Sleeper_2. Only when one is missing is it made here, where it was
    # before (a cell's floor is off the walkable floor, so those are not snapped to it).
    def placed(self, name):
        return self._game.get_node(name)

    def setup_garage(self):
        nav = self._game.nav
        for i in range(len(GARAGE_SLEEPERS)):
            s = GARAGE_SLEEPERS[i]
            if self.placed("Sleeper_" + str(i + 1)) != None:
                continue
            spot = nav.nearest_point(Vec3(s[0], -10.0, s[1]))
            yaw = s[3]
            if spot == None:
                continue
            bot = self._game.instantiate("Assets/Prefabs/kamikaze.scene.json")
            if bot == None:
                return
            bot.global_position = Vec3(spot.x, spot.y + 0.9, spot.z)
            bot.rotation_degrees = Vec3(0.0, yaw, 0.0)
            bot._yaw = yaw
            bot.pose = s[2]
            bot.hold_pose()

    def exit_door(self):
        door = self.exit_item()
        return door.center() if door != None else Vec3(-20.0, -3.0, 77.0)

    def exit_item(self):
        for body in self.get_tree().get_nodes_in_group("breakable"):
            if body.name.find("d_door_exit") >= 0:
                return body.get_node("Breakable")
        return None

    # Once the orders are read the garage door is knocked out of its frame and falls into the
    # street, and the first bomb carrier comes running up the ramp from the dark below: heard
    # screaming before he is seen.
    # Three seconds after the gate is knocked down it must lie flat; a leaf still standing (it
    # caught on the frame or on someone) gets a shove at the top.
    def watch_gate(self, delta):
        if self._gate == None:
            return
        self._gate_check = self._gate_check - delta
        if self._gate_check > 0.0:
            return
        gate = self._gate
        self._gate = None
        tilt = math.abs(gate.rotation_degrees.x)
        if tilt > 180.0:
            tilt = 360.0 - tilt
        if tilt < 45.0:
            gate.apply_impulse(Vec3(0.0, 0.0, 1800.0), Vec3(0.0, 2.0, 0.0))
            gate.angular_velocity = Vec3(2.5, 0.0, 0.0)

    def ambush(self):
        self._ambush = True
        door = self.exit_item()
        if door == None:
            return
        c = door.center()
        self._gate = door.topple(Vec3(0.0, 0.0, 1.0))
        self._gate_check = 3.0
        self._game._glitch = 0.5
        self._game.show("BOMB CARRIER INCOMING", 2.5)
        nav = self._game.nav
        spot = nav.nearest_point(Vec3(c.x, c.y - 5.5, c.z - 22.0)) if nav != None else None
        if spot == None:
            return
        bot = self._game.instantiate("Assets/Prefabs/kamikaze.scene.json")
        if bot == None:
            return
        bot.global_position = Vec3(spot.x, spot.y + 0.9, spot.z)
        bot._yaw = 0.0
        bot.pose = "idle"
        bot.hold_pose()
        bot.wake()

    def update_escort(self):
        if self._target == None or self.distance(self._target) > 4.0:
            return
        if self._unit != None:
            u = self._unit.global_position
            t = self._target
            if (u.x - t.x) * (u.x - t.x) + (u.z - t.z) * (u.z - t.z) > 25.0:
                return
        self.next()

    # Unit-7 fell: the escort starts again from the checkpoint with a new body beside her.
    def unit_lost(self):
        self._game.show("UNIT-7 LOST", 3.0)
        self._game._glitch = 1.0
        self._lost = 4.0

    def restore_unit(self):
        if self._unit != None:
            self._unit.queue_free()
        self._game.place_player()
        p = self.checkpoint if self.checkpoint != None else self._player.global_position
        unit = self._game.instantiate(UNIT_PREFAB)
        if unit != None:
            unit.global_position = Vec3(p.x + 1.0, p.y, p.z)
            unit.wake()
        self._unit = unit
        self.begin(self._stage)

    # ---- scenery

    def scenery(self, prefab, position, yaw, clip):
        npc = self._game.instantiate(prefab)
        if npc == None:
            return None
        npc.global_position = position
        npc._yaw = yaw
        npc.rotation_degrees = Vec3(0.0, yaw, 0.0)
        if clip != "":
            npc.idle_clip = clip
            npc._clip = ""
            if npc._anim != None and npc._anim.has_animation(clip):
                npc._anim.set_animation_loop(clip, True)
            npc.play(clip)
        self._game.register_enemy(npc)
        return npc

    def setup_meeting(self):
        y = MEETING_FLOOR + 1.0
        front = self._game.nav.nearest_point(Vec3(5.4, MEETING_FLOOR, -11.4))
        if front == None:
            front = Vec3(5.4, MEETING_FLOOR, -11.4)
        self.scenery("Assets/Prefabs/bravura.scene.json", Vec3(front.x, front.y + 1.0, front.z), 0.0, "Custom_Vinnie_Talk")
        for i in range(len(MEETING_CHAIRS)):
            chair = MEETING_CHAIRS[i]
            cop = self.scenery(MEETING_COPS[i % len(MEETING_COPS)], Vec3(chair[0], y, chair[1]), 180.0, MEETING_CLIPS[i % len(MEETING_CLIPS)])
            if cop != None:
                cop.seated = True
        self.scenery("Assets/Prefabs/cop_b.scene.json", Vec3(2.6, y, -11.6), -90.0, "Idle_Smoke")

    # An innocent drunk in a cell of block 3, and a woman in a towel in the ladies' room.
    def setup_civilians(self):
        self.civilian("Assets/Prefabs/prisoner.scene.json", "Prisoner", Vec3(-4.8, -10.0, -16.9), 0.0)
        self.civilian("Assets/Prefabs/towel.scene.json", "Towel", Vec3(-6.4, -5.0, -19.0), 180.0)

    def civilian(self, prefab, name, spot, yaw):
        if self.placed(name) == None:
            self.scenery(prefab, Vec3(spot.x, spot.y + 1.0, spot.z), yaw, "")

    # ---- waves

    def update_waves(self, delta):
        if self._waves_left <= 0:
            return
        self._wave = self._wave - delta
        if self._wave > 0.0:
            return
        self._waves_left = self._waves_left - 1
        kind = STAGES[self._stage][2]
        if kind == "hack":
            self._wave = 6.0
            self.spawn_cops(REGIONS["floor0"], 2)
        elif kind == "transports" or kind == "escort":
            self._wave = 9.0
            self.spawn_bots(REGIONS["garage"], 1)
            self.spawn_cops(REGIONS["garage"], 2)
        else:
            self._wave = 7.0
            self.spawn_cops(REGIONS["cells"], 2)
            self.spawn_bots(REGIONS["cells"], 1)

    def alive_spawned(self, group):
        count = 0
        for node in self.get_tree().get_nodes_in_group(group):
            count = count + 1
        return count

    # A floor spot in the region, away from her and out of her sight.
    def spawn_spot(self, region):
        nav = self._game.nav
        p = self._player.global_position
        for i in range(60):
            q = nav.random_point()
            if q == None or not self.inside(Vec3(q.x, q.y + 0.5, q.z), region):
                continue
            d = self.distance(q)
            if d < 7.0:
                continue
            hit = raycast(Vec3(p.x, p.y + 0.6, p.z), Vec3(q.x, q.y + 1.2, q.z), None, self._player)
            if hit == None and d < 14.0:
                continue
            return q
        return None

    def spawn_cops(self, region, count):
        for i in range(count):
            if self.alive_spawned("enemy") >= self.max_cops:
                return
            spot = self.spawn_spot(region)
            if spot == None:
                return
            cop = self._game.instantiate(COPS[int(math.random(0.0, len(COPS) - 0.01))])
            if cop == None:
                return
            cop.global_position = Vec3(spot.x, spot.y + 1.0, spot.z)
            self._game.register_enemy(cop)
            p = self._player.global_position
            cop._state = "alert"
            cop._alert = 30.0
            cop._last_seen = Vec3(p.x, p.y, p.z)

    def spawn_bots(self, region, count):
        for i in range(count):
            if self.alive_spawned("bot") >= self.max_bots:
                return
            spot = self.spawn_spot(region)
            if spot == None:
                return
            bot = self._game.instantiate("Assets/Prefabs/kamikaze.scene.json")
            if bot == None:
                return
            bot.global_position = Vec3(spot.x, spot.y + 0.9, spot.z)
            bot.pose = "idle"
            bot.hold_pose()
            bot.wake()

    # ---- helpers

    def screen_in(self, region):
        best = None
        mid = Vec3((region[0] + region[1]) * 0.5, (region[2] + region[3]) * 0.5, (region[4] + region[5]) * 0.5)
        best_d = 1000000.0
        for body in self.get_tree().get_nodes_in_group("breakable"):
            item = body.get_node("Breakable")
            if item == None or item.kind != "monitor":
                continue
            c = item.center()
            if not self.inside(c, region):
                continue
            d = (c.x - mid.x) * (c.x - mid.x) + (c.z - mid.z) * (c.z - mid.z)
            if d < best_d:
                best_d = d
                best = c
        if best == None:
            best = mid
        return best

    def inside(self, p, r):
        return p.x >= r[0] and p.x <= r[1] and p.y >= r[2] and p.y <= r[3] and p.z >= r[4] and p.z <= r[5]

    def distance(self, q):
        p = self._player.global_position
        return math.sqrt((q.x - p.x) * (q.x - p.x) + (q.y - p.y) * (q.y - p.y) + (q.z - p.z) * (q.z - p.z))

    # ---- HUD: the orders with progress, and a marker on the objective.

    def draw_objectives(self):
        if self._stage < 0:
            return False
        stage = STAGES[self._stage]
        kind = stage[2]
        if kind == "training":
            self.draw_training()
            self.draw_marker()
            return True
        y = 34.0
        i = 0
        for order in stage[1]:
            text = "> " + order
            if i == 0 and kind == "transports":
                text = text + "  " + str(self._count) + "/" + str(self._needed)
            color = Color(0.45, 0.9, 1.0, 0.9) if i == 0 else Color(0.45, 0.9, 1.0, 0.55)
            hud_rect(12.0, y - 2.0, 3.0, 18.0, color)
            hud_text(22.0, y, text, color, 15.0 if i == 0 else 13.0)
            y = y + 22.0
            i = i + 1
        if (kind == "hack" or kind == "upload") and self._progress > 0.0:
            self.draw_bar(y + 6.0)
        if self._game._spider_rebuild > 0.0:
            hud_text(22.0, y + 40.0, "SPIDER OFFLINE // REBUILD " + str(int(self._game._spider_rebuild + 0.99)) + "s", Color(1.0, 0.45, 0.3, 0.9), 13.0)
        self.draw_marker()
        return True

    def draw_bar(self, y):
        w = 240.0
        label = "UPLOADING" if STAGES[self._stage][2] == "upload" else "HACKING"
        if not self._hacking:
            label = label + "  PAUSED"
        hud_rect(22.0, y, w, 10.0, Color(0.0, 0.0, 0.0, 0.5))
        hud_rect(22.0, y, w * math.clamp(self._progress, 0.0, 1.0), 10.0, Color(0.3, 1.0, 0.5, 0.9))
        hud_text(22.0, y + 14.0, label + " " + str(int(self._progress * 100.0)) + "%", Color(0.3, 1.0, 0.5, 0.9), 12.0)

    # Behind her or off the screen: an arrow on an oval round the middle of the screen, pointing
    # the way to turn (straight up is straight ahead), with the name and distance.
    def draw_edge_arrow(self, camera, target, width, height, color):
        local = camera.to_local(target)
        a = math.atan2(local.x, -local.z)
        dx = math.sin(a)
        dy = -math.cos(a)
        cx = width * 0.5 + dx * width * 0.36
        cy = height * 0.5 + dy * height * 0.36
        size = 24.0 * height / 720.0
        # A chevron: two strokes from the tip back to either side.
        tx = cx + dx * size
        ty = cy + dy * size
        px = -dy
        py = dx
        bx = cx - dx * size * 0.4
        by = cy - dy * size * 0.4
        hud_line(tx, ty, bx + px * size, by + py * size, color, 6.0)
        hud_line(tx, ty, bx - px * size, by - py * size, color, 6.0)
        hud_line(tx, ty, cx - dx * size * 1.2, cy - dy * size * 1.2, color, 4.0)
        label = self._target_label + "  " + str(int(self.distance(target))) + "M"
        # The label on the inner side of the arrow, towards the middle.
        lx = cx - dx * size * 2.6
        ly = cy - dy * size * 2.6 - 6.0
        text = 15.0 * height / 720.0
        hud_text(lx - hud_text_size(label, text).x * 0.5, ly, label, color, text)

    def draw_marker(self):
        target = self._target
        if target == None or self._game.briefing():
            return
        camera = self._player.camera()
        if camera == None:
            return
        # The HUD shows where NULL//OS says it is (a lie while it corrupts the navigation).
        target = self._game.lied(target)
        pulse = 0.6 + 0.4 * math.sin(self._game._clock * 6.0)
        color = Color(0.3, 1.0, 0.55, 0.9 * pulse)
        width = hud_width()
        height = hud_height()
        s = None
        if not camera.is_position_behind(target):
            s = camera.unproject_position(target)
        margin = 40.0
        if s == None or s.x < margin or s.x > width - margin or s.y < margin or s.y > height - margin:
            self.draw_edge_arrow(camera, target, width, height, color)
            return
        hud_rect_outline(s.x - 9.0, s.y - 9.0, 18.0, 18.0, color, 2.0)
        hud_rect(s.x - 2.0, s.y - 2.0, 4.0, 4.0, color)
        label = self._target_label + "  " + str(int(self.distance(target))) + "M"
        if (STAGES[self._stage][2] == "hack" or STAGES[self._stage][2] == "upload") and self.distance(target) < 2.2:
            label = "[E] " + self._target_label
        hud_text(s.x - hud_text_size(label, 12.0).x * 0.5, s.y + 14.0, label, color, 12.0)
