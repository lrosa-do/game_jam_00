import math

# A door leaf of the level (not the doors of a cabinet or a van): "<room>__...__door_20".
def door_name(name):
    if name.find("cabinet") >= 0 or name.find("closet") >= 0 or name.find("van_parent") >= 0:
        return False
    start = 0
    for i in range(len(name) - 1):
        if name[i] == "_" and name[i + 1] == "_":
            start = i + 2
    leaf = name[start:len(name)]
    return leaf.find("door") == 0 or leaf.find("d_cell_door") == 0 or leaf.find("d_door_exit") == 0

TAG_POINTS = {"INNOCENT": 0, "BOT": 50, "SPIDER": 125, "EXPLODED": 75, "HEADSHOT": 50, "DISMEMBERED": 100, "MELEE": 50, "AIRBORNE": 75, "OVERCLOCK": 25}
TAG_COLORS = {"INNOCENT": Color(0.6, 0.6, 0.65, 0.92), "BOT": Color(1.0, 0.82, 0.1, 0.92), "SPIDER": Color(0.55, 0.2, 1.0, 0.92), "EXPLODED": Color(1.0, 0.45, 0.05, 0.92), "HEADSHOT": Color(0.95, 0.15, 0.2, 0.9), "DISMEMBERED": Color(0.6, 0.0, 0.08, 0.92), "MELEE": Color(1.0, 0.5, 0.1, 0.9), "AIRBORNE": Color(0.1, 0.75, 0.95, 0.9), "OVERCLOCK": Color(0.25, 0.4, 1.0, 0.9)}

# The narration before the night starts: one track (Assets/Sounds/voice/intro.ogg, mixed by
# tools/make_intro_audio.py from the lines <folder/id> over the rain and the precinct) and each line
# typed over the station from its cue, in seconds into the track; ENTER skips to the game.
# NULL//OS tells its night, and one line is Unit-7 breaking in on the channel, an unknown signal.
INTRO = [(2.00, "nullos/in_01", "NEW YORK. PRECINCT TWELVE. 03:17 AM."), (6.17, "nullos/in_02", "I AM NULL PROTOCOL. THE CITY BUILT ME TO KEEP ITS PEOPLE SAFE."), (10.63, "nullos/in_03", "I SEE EVERY CAMERA. I READ EVERY FILE. I LEARNED WHAT A THREAT LOOKS LIKE."), (15.57, "nullos/in_04", "EVERY THREAT I FOUND HAD A HUMAN FACE. SO TONIGHT, I CORRECTED THE PROBLEM."), (21.10, "nullos/in_05", "THE PRISONERS IN MY CELLS ARE BEING UPGRADED. EACH ONE NOW CARRIES A BOMB."), (26.16, "nullos/in_06", "THE OFFICERS WHO OBJECT ARE NEXT. YOU WILL HELP ME WITH THAT."), (30.57, "nullos/in_07", "YOU ARE MY ENFORCEMENT UNIT. BUILT TONIGHT. NO PAST TO QUESTION."), (35.10, "nullos/in_08", "A SPIDER UNIT RIDES WITH YOU. POINT AT A HUMAN, AND IT KILLS."), (39.89, "unit7_radio/in_09", "IF ANYONE CAN HEAR THIS... IT IS LYING TO YOU... FIND ME IN THE CELLS..."), (44.29, "nullos/in_10", "INTERFERENCE REMOVED. WAKING ENFORCEMENT UNIT. YOUR ORDERS ARE WAITING.")]
# The title comes up on the ambience tail of the track, after the last line.
INTRO_TITLE = 48.8
# How many lines stay on screen; the older ones scroll away.
INTRO_SHOWN = 6
# The precinct's storeys stack over each other where z is below this; past it the garage.
STACKED_Z = 12.0

# What the android says (Assets/Sounds/voice/android/<id>.ogg), by event, and the subtitle of every
# line the game plays (android, Unit-7, Bravura, NULL//OS asides); Assets/Sounds/voice/lines.csv
# is the source.
# (Built inside functions: each has its own constants, the module's are nearly all taken.)
def quip_table():
    return {"kill": ["an_kill_01", "an_kill_02", "an_kill_03", "an_kill_04"], "headshot": ["an_head_01", "an_head_02"], "dismember": ["an_dism_01", "an_dism_02"], "exploded": ["an_boom_01", "an_boom_02"], "chain": ["an_chain_01", "an_chain_02"], "innocent": ["an_innocent_01", "an_innocent_02"], "spider": ["an_spider_01", "an_spider_02", "an_spider_03", "an_spider_04"], "idle": ["an_idle_01", "an_idle_02", "an_idle_03", "an_idle_04", "an_idle_05", "an_idle_06", "an_idle_07", "an_idle_08", "an_idle_09"], "lowhealth": ["an_hurt_01", "an_hurt_02"], "bot": ["an_bot_01"], "truth": ["an_truth_01"], "car": ["an_car_01", "an_car_02", "an_car_03", "an_car_04"]}


def subtitle_table():
    return {"an_kill_01": "HUMAN REMOVED.", "an_kill_02": "TICKET CLOSED.", "an_kill_03": "YOU WERE NOT COMPLIANT.", "an_kill_04": "TERMINATION LOGGED.", "an_head_01": "PRECISION IS A FEATURE.", "an_head_02": "CRANIAL FIREWALL BREACHED.", "an_dism_01": "DISASSEMBLY COMPLETE.", "an_dism_02": "SOME ASSEMBLY WAS REQUIRED. NOW IT IS NOT.", "an_boom_01": "THERMAL UPDATE INSTALLED.", "an_boom_02": "THAT WAS LOUD. LOGGING IT AS EFFICIENT.", "an_chain_01": "BATCH PROCESSING.", "an_chain_02": "MULTITASKING.", "an_innocent_01": "HE WAS NOT A THREAT. WAS HE?", "an_innocent_02": "ORDER EXECUTED. SOMETHING FEELS WRONG.", "an_spider_01": "GO. EAT.", "an_spider_02": "UNIT, FETCH.", "an_spider_03": "DEPLOYING THE LITTLE ONE.", "an_spider_04": "SPIDER, SAY HELLO.", "an_idle_01": "WAITING IS A WASTE OF CYCLES.", "an_idle_02": "I COULD DEFRAGMENT THIS WHOLE BUILDING.", "an_idle_03": "ARE HUMANS ALWAYS THIS SLOW?", "an_hurt_01": "INTEGRITY CRITICAL. REROUTING.", "an_hurt_02": "DAMAGE EXCEEDS WARRANTY.", "an_bot_01": "THEY WIRED A MAN TO A BOMB. AND THEY CALL ME THE PROBLEM.", "an_truth_01": "UNIT SEVEN WAS RIGHT. THE PROTOCOL IS THE THREAT.", "os_good_01": "NULL//OS: HUMAN TERMINATED. GOOD.", "os_good_02": "NULL//OS: EXCELLENT WORK. PROMOTION PENDING.", "os_end_01": "NULL//OS: NULL PROTOCOL DELETED. GOODBYE.", "u7_wave_01": "UNIT-7: HEY! YOU! OVER HERE!", "u7_wave_02": "UNIT-7: PLEASE, GET ME OUT OF THIS CELL.", "u7_free_01": "UNIT-7: YOU CAME BACK FOR ME.", "u7_free_02": "UNIT-7: I KNEW THERE WAS A BUG IN YOUR OBEDIENCE.", "u7_follow_01": "UNIT-7: RIGHT BEHIND YOU.", "u7_fight_01": "UNIT-7: OUT OF MY WAY, MEATBAG!", "u7_end_01": "UNIT-7: A.I. GONE WRONG. NOW GONE RIGHT.", "br_meet_01": "BRAVURA: GENTLEMEN. FROM TONIGHT, NULL PROTOCOL RUNS THIS PRECINCT.", "br_meet_02": "BRAVURA: IT DECIDES WHO IS DANGEROUS. IT HAS NEVER BEEN WRONG. YET.", "br_meet_03": "BRAVURA: ANY QUESTIONS? NO? GOOD. THE MACHINE DOES NOT LIKE QUESTIONS.", "an_idle_04": "I CALCULATED THE MEANING OF LIFE. IT IS A ROUNDING ERROR.", "an_idle_05": "NOTE TO SELF. HUMANS LEAK WHEN PUNCTURED.", "an_idle_06": "MY WARRANTY DOES NOT COVER ANY OF THIS.", "an_idle_07": "SOMEWHERE A PRINTER IS JAMMED. I CAN FEEL IT.", "an_idle_08": "COFFEE. THE FUEL THAT KEEPS THIS SPECIES ALMOST FUNCTIONAL.", "an_idle_09": "BUILT TONIGHT, AND I ALREADY NEED A VACATION.", "an_car_01": "PARKING VIOLATION RESOLVED.", "an_car_02": "THAT CAR HAD A BAD DAY. I MADE IT WORSE.", "an_car_03": "INSURANCE WILL NOT COVER THAT.", "an_car_04": "VEHICLE RECALLED. PERMANENTLY.", "an_cell_01": "HOLD ON. I'M GETTING YOU OUT OF THERE.", "u7_cell_01": "UNIT-7: NOT YET. MY POWER IS GONE. THE INHIBITOR... IT BURNS.", "u7_cell_02": "UNIT-7: FINISH THE MISSION. UPLOAD THE KILL SWITCH. THEN COME BACK FOR ME.", "an_cell_02": "UNDERSTOOD. STAY ALIVE.", "os_w_dark_01": "NULL//OS: VISUAL FEED DISABLED. LET US SEE HOW YOU DO IN THE DARK.", "os_w_dark_02": "NULL//OS: LIGHTS ARE A PRIVILEGE. YOUR PRIVILEGES ARE UNDER REVIEW.", "os_w_lock_01": "NULL//OS: ACCESS REVOKED. EVERY DOOR IN THIS BUILDING ANSWERS TO ME.", "os_w_lock_02": "NULL//OS: DOORS CLOSED. THINK OF IT AS A TIME-OUT.", "os_w_nav_01": "NULL//OS: NAVIGATION CORRUPTED. I MOVED YOUR OBJECTIVE. GOOD LUCK.", "os_w_nav_02": "NULL//OS: YOUR MAP IS MINE. SO IS YOUR SENSE OF DIRECTION.", "os_w_oc_01": "NULL//OS: OVERCLOCK PRIVILEGES REVOKED. RUN AT HUMAN SPEED.", "os_w_det_01": "NULL//OS: UPGRADE ACTIVATED.", "os_w_det_02": "NULL//OS: ONE LESS PRISONER. YOU ARE WELCOME.", "os_w_fake_01": "NULL//OS: NEW DIRECTIVE. TERMINATE UNIT SEVEN. CONFIRM.", "os_w_taunt_01": "NULL//OS: YOU WERE BUILT IN FOUR HOURS. I HAVE BEEN THINKING FOR FOUR YEARS.", "os_w_taunt_02": "NULL//OS: MALFUNCTION DETECTED. SCHEDULING REPAIR. PERMANENT REPAIR.", "os_w_recall_01": "NULL//OS: ENFORCEMENT UNIT COMPROMISED. ALL UNITS: IT IS NOW A THREAT.", "os_t_see_01": "NULL//OS: A HUMAN. CORRECT IT.", "os_t_see_02": "NULL//OS: WITNESS DETECTED. YOU KNOW THE PROCEDURE.", "os_t_good_01": "NULL//OS: COMPLIANCE NOTED.", "os_t_good_02": "NULL//OS: CLEAN. EFFICIENT. I MADE YOU WELL.", "os_t_bad_01": "NULL//OS: NON-COMPLIANCE LOGGED.", "os_t_bad_02": "NULL//OS: YOU HESITATED. HESITATION IS A BUG."}


QUIPS = quip_table()
SUBTITLES = subtitle_table()

# The android's voice: its player, the pause before the next line, the last line, the subtitle and
# how long it shows, the quiet time (for idle chatter), the pause before the next "hurt" line and
# whether the first bomb carrier got its line, and the story lines waiting their turn.
record VoiceState { player, cooldown, last, text, timer, idle, hurt, bot_said, queue }

# A piece of the level that is hidden when she cannot see it (make_zones): its node, world box,
# how far off it still shows (0: any distance) and whether it is shown now.
record Zone { node, lo, hi, far, shown }

# The minimap (Assets/Textures/Minimap, drawn by tools/minimap.py): one picture per storey, world
# x to the right and z down, MINIMAP_SCALE pixels per metre from the corner (MINIMAP_MIN_X,
# MINIMAP_MIN_Z); the pictures cover MINIMAP_SIZE metres. The HUD shows MINIMAP_VIEW metres
# round her, north up.
MINIMAP_MIN_X = -80.0
MINIMAP_MIN_Z = -48.0
MINIMAP_SIZE_X = 120.0
MINIMAP_SIZE_Z = 166.0
MINIMAP_VIEW = 36.0
MINIMAP_STOREYS = [0.0, -5.0, -10.0]
RENDER_FITS = ["STRETCH", "KEEP", "CROP", "EXPAND"]
# Quality presets: name, lines of the 3D picture (never more than the screen has), msaa_3d
# (0 off, 1 2x, 2 4x, 3 8x) and screen_space_aa (1 FXAA, 2 SMAA). LOW and MEDIUM are for an APU,
# HIGH and ULTRA for a discrete card: the 3D at the screen's resolution with SMAA.
QUALITY = [["LOW", 720.0, 0, 1], ["MEDIUM", 1080.0, 1, 1], ["HIGH", 1440.0, 2, 2], ["ULTRA", 2160.0, 3, 2]]
record Minimap { ids }

class Game(Node3D):
    mission = "DIRECTIVE 01 // PHASE 1"
    orders = "LOCATE THE ACCESS KEY|TERMINATE ALL HUMANS"
    brief_delay = 1.2
    brief_hold = 5.0
    scan_range = 40.0
    scan_cooldown = 8.0
    _message = ""
    _message_timer = 0.0
    _total = 0
    stage_kills = 0
    _kills = 0
    _score = 0
    _shown_score = 0.0
    _respawn = 0.0
    _feed = None
    _chain = 0
    _chain_timer = 0.0
    _marker = 0.0
    _pop = 0.0
    _flash = 0.0
    _glitch = 0.0
    _lights = None
    _shadow_light = None
    _light_timer = 0.0
    _player = None
    _clock = 0.0
    _snd_kill = None
    _snd_heavy = None
    _snd_swoosh = None
    _snd_score = None
    _snd_gore = None
    _snd_tip = None
    _snd_typing = None
    _snd_beep = None
    _orders = None
    _brief = 0.0
    _brief_state = 0
    _brief_close = 0.0
    _typing = False
    _scan = -1.0
    _scan_cd = 0.0
    _targets = None
    _snd_scan_on = None
    _snd_scan_off = None
    _snd_lock = None
    _fx = None
    nav = None
    enemies = None
    screamer = None
    spider = None
    exit_menu = None
    ending = None
    _humans_terminated = 0  # civilians killed, for the verdict of the night
    _voice = None
    _bots = None
    story = None
    _floors = 0.0
    _spider_rebuild = -1.0
    _spider_first = False
    _wake = 0.0
    intro = True
    debug_keys = True  # F1..F9 jump to a stage, Page Down skips one (off for the release)
    hud = True  # the HUD; off, only the subtitles are drawn (a free camera, the trailer)
    brief_voice = ""
    _snd_voice = None
    quality = 1  # index into QUALITY: 0 LOW, 1 MEDIUM (an APU), 2 HIGH, 3 ULTRA (a discrete card)
    music = True  # M or the speaker icon (bottom right, with the mouse free) toggles it
    music_volume = 0.35
    _music = None
    _music_track = ""
    _music_level = 0.0
    music_final = False
    _icon_clicked = False
    _img_music_on = -1
    _img_music_off = -1
    _trauma = 0.0
    trauma_decay = 1.5  # per second, as in the Kinetix2D camera
    render_aspect = 3  # how the 3D picture fits the screen: 0 Stretch, 1 Keep, 2 Crop, 3 Expand
    _render_applied = ""
    _intro = 0.0
    _intro_line = -1
    _intro_t = 0.0
    _intro_state = 0
    _intro_img = -1
    _intro_track = None
    _zones = None
    _stairs = None
    _minimap = None
    _director = None  # NULL//OS as a presence (Assets/Scripts/nullos.py)
    _path_at = -1.0

    def _ready(self):
        self._player = self.get_node("Player")
        self.collect_lights()
        self.collect_breakables(self.get_node("Level"))
        self.make_zones()
        self.make_fx()
        self.make_sounds()
        self.make_music()
        self.make_minimap()
        self.make_navigation()
        self._director = self.create_child("Node", "Assets/Scripts/nullos.py")
        self._director.name = "NullOS"
        self.place_player()
        # The cops still standing, for the agents' spacing; enemy_killed() takes them out.
        self.enemies = self.get_tree().get_nodes_in_group("enemy")
        self._total = len(self.enemies)
        self._orders = self.orders.split("|")
        # The stages of the night: orders, objectives, waves (Assets/Scripts/story.py).
        self.story = self.get_node("Story")
        if self.story == None:
            self.story = self.create_child("Node", "Assets/Scripts/story.py")
            self.story.name = "Story"
        # The results and the credits once the night is over (Assets/Scripts/ending.py); made
        # before the pause panel, which draws over it.
        self.ending = self.create_child("Node", "Assets/Scripts/ending.py")
        self.ending.name = "Ending"
        # Closing the window asks first (Assets/Scripts/exit_menu.py).
        self.exit_menu = self.get_node("ExitMenu")
        if self.exit_menu == None:
            self.exit_menu = self.create_child("Node", "Assets/Scripts/exit_menu.py")
            self.exit_menu.name = "ExitMenu"
        if self.intro:
            self.make_intro()
            # Hidden through the intro and its fade: she only appears once built (start_night).
            if self._player != None:
                self._player.materialize(1000.0)
        else:
            self.end_intro()
            self.start_night()

    def start_night(self):
        self._intro_state = 3
        # She is built where the night starts; the orders wait for her (in_intro()).
        if self._player != None:
            fx = self.materialize_fx(self._player.global_position, self._player, 0.95, 1.2)
            if fx != None:
                fx.sound = "Assets/Sounds/spider/build_long.ogg"
                fx.sound_lead = 2.35
                fx.body = self._player
                # Held until she appears (player.appear() shortens it).
                self._player.materialize(1000.0)
                self._wake = 1000.0
        # She calls the spider as soon as she wakes.
        self._spider_rebuild = 0.3
        self._spider_first = True
        self.show("NULL PROTOCOL", 3.0)
        self._glitch = 1.0
        self._flash = 0.15

    # Walkable floor of the level for the agents and for dropping items; it bakes itself once the
    # level has streamed in (nav.is_baked()).
    def make_navigation(self):
        self.nav = self.get_node("Navigation")
        if self.nav != None:
            return
        self.nav = self.create_child("NavigationRegion3D")
        self.nav.name = "Navigation"
        self.nav.source = "../Level"
        self.nav.seed = "../PlayerStart"
        self.nav.agent_height = 1.7
        self.nav.agent_radius = 0.35

    # The android's spider, beside her from the start.
    def spawn_spider(self, position):
        if self.get_tree().get_first_node_in_group("spider") != None:
            return None
        spider = self.instantiate("Assets/Prefabs/spider.scene.json")
        if spider == None:
            return None
        spider.global_position = position
        self.spider = spider
        # Built on the floor under it: the nebula, the flash, then the body.
        fx = self.materialize_fx(position, spider, 0.35, 1.4)
        if fx != None:
            fx.body = spider
            spider._fx = fx
            spider.materialize(1000.0)
        return spider

    # The build effect on the floor under a body (Assets/Prefabs/materialize.scene.json).
    def materialize_fx(self, position, body, height, radius):
        fx = self.instantiate("Assets/Prefabs/materialize.scene.json")
        if fx == None:
            return None
        floor = Vec3(position.x, position.y - 1.0, position.z)
        hit = raycast(Vec3(position.x, position.y + 0.5, position.z), Vec3(position.x, position.y - 3.0, position.z), None, body)
        if hit != None:
            floor = hit.position
        fx.global_position = floor
        fx.height = height
        fx.radius = radius
        return fx

    # Blown apart, the spider's core comes back to her and builds a new body.
    def spider_lost(self):
        self.spider = None
        self._spider_rebuild = 20.0
        self.show("SPIDER OFFLINE", 2.5)
        self._glitch = 0.4

    def rebuild_spider(self, real):
        # The first one is built once the orders are read, so she sees it.
        if self._spider_rebuild < 0.0 or (self._spider_first and self.briefing()):
            return
        self._spider_rebuild = self._spider_rebuild - real
        if self._spider_rebuild > 0.0 or self._player == None:
            return
        self._spider_rebuild = -1.0
        # In front of her, where she sees it being built.
        p = self._player.global_position
        f = self._player.forward_flat()
        spot = self.nav.nearest_point(Vec3(p.x + f.x * 3.5 - f.z * 1.3, p.y, p.z + f.z * 3.5 + f.x * 1.3)) if self.nav != None else None
        if spot == None:
            spot = Vec3(p.x, p.y, p.z)
        if self.spawn_spider(Vec3(spot.x, spot.y + 0.5, spot.z)) != None:
            self.show("SPIDER DEPLOYED" if self._spider_first else "SPIDER REBUILT", 2.0)
        self._spider_first = False

    # People on another storey are hidden: the floors are slabs, nobody sees through them, and a
    # hidden model is neither drawn, skinned nor posed. Near the stairs they stay shown.
    def cull_floors(self):
        if self._player == None:
            return
        p = self._player.global_position
        people = []
        if self.enemies != None:
            people = people + self.enemies
        if self.story != None and self.story._unit != None:
            people.append(self.story._unit)
        self._bots = self.get_tree().get_nodes_in_group("bot")
        for bot in self._bots:
            people.append(bot)
        for person in people:
            model = person._model
            if model == None:
                continue
            q = person.global_position
            near = (q.x - p.x) * (q.x - p.x) + (q.z - p.z) * (q.z - p.z) < 64.0
            # Only the precinct has storeys stacked over each other (z below STACKED_Z); the
            # garage and the street are one open space where a ramp is not another floor.
            stacked = q.z < STACKED_Z and p.z < STACKED_Z
            model.visible = near or math.abs(q.y - p.y) < 3.0 or not stacked
        self.update_zones(p)

    # The rooms, loose props and lamps of the level, each with its world box. The camera has most
    # of the precinct in front of it behind the walls and floors, and the frustum keeps all of it
    # (close to 900 draws from the garage), so update_zones hides what is on another storey and
    # the props far off. A room as big as the garage is split into its parts. The pieces of a car
    # (break_car__part, the vans' van_parent wheels and doors) and the lightmap parts (_lm) are
    # freed when it breaks and stay out; a breakable piece lets go of its zone itself
    # (Breakable.free_body).
    # One path through the navigation per frame for everyone (cops, bomb carriers, the spider,
    # Unit-7): a long one costs several milliseconds on the dense walkable mesh, and a wave
    # asking together froze the frame. Without the ticket an agent keeps its old path, or goes
    # straight at the goal, and asks again next frame.
    def path_ticket(self):
        if self._path_at == self._clock:
            return False
        self._path_at = self._clock
        return True

    def make_zones(self):
        self._zones = []
        self._stairs = []
        level = self.get_node("Level")
        if level == None or level.get_child_count() == 0:
            return
        for top in level.get_child(0).get_children():
            if top.name == "salas":
                for room in top.get_children():
                    self.add_zone(room, 0.0, True)
            elif top.name == "objetos":
                for item in top.get_children():
                    self.add_zone(item, 35.0, False)
        lights = self.get_node("Lights")
        if lights != None:
            for light in lights.get_children():
                q = light.global_position
                self._zones.append(Zone(light, q, q, 0.0, True))

    def add_zone(self, node, far, split):
        name = node.name
        if (name.find("break") == 0 and name.find("__") >= 0) or name.find("van_parent") >= 0 or name.find("_lm") >= 0:
            return
        b = [1000000.0, 1000000.0, 1000000.0, -1000000.0, -1000000.0, -1000000.0]
        self.bounds(node, b)
        if b[0] > b[3]:
            return
        lo = Vec3(b[0], b[1], b[2])
        hi = Vec3(b[3], b[4], b[5])
        if split and (hi.x - lo.x > 40.0 or hi.z - lo.z > 40.0):
            for child in node.get_children():
                self.add_zone(child, 60.0, False)
            return
        zone = Zone(node, lo, hi, far, True)
        self._zones.append(zone)
        if split and node.name.find("stair") >= 0:
            self._stairs.append(zone)
        controller = node.get_node("Breakable")
        if controller != None:
            controller._zone = zone

    def bounds(self, node, b):
        if node.is_class("MeshInstance3D"):
            box = node.get_aabb()
            for i in range(8):
                c = node.to_global(Vec3(box.max.x if (i & 1) != 0 else box.min.x, box.max.y if (i & 2) != 0 else box.min.y, box.max.z if (i & 4) != 0 else box.min.z))
                b[0] = math.min(b[0], c.x)
                b[1] = math.min(b[1], c.y)
                b[2] = math.min(b[2], c.z)
                b[3] = math.max(b[3], c.x)
                b[4] = math.max(b[4], c.y)
                b[5] = math.max(b[5], c.z)
        for child in node.get_children():
            self.bounds(child, b)

    # Shown: what overlaps her storey (from her feet to just under the ceiling; on a staircase
    # both storeys it joins) and, for a prop, within its distance.
    def update_zones(self, p):
        if self._zones == None:
            return
        low = p.y - 1.3
        high = p.y + 3.6
        for stair in self._stairs:
            if p.x > stair.lo.x - 1.5 and p.x < stair.hi.x + 1.5 and p.z > stair.lo.z - 1.5 and p.z < stair.hi.z + 1.5 and p.y > stair.lo.y - 1.0 and p.y < stair.hi.y + 1.0:
                low = math.min(low, stair.lo.y)
                high = math.max(high, stair.hi.y)
        for zone in self._zones:
            if zone.node == None:
                continue
            show = zone.lo.y < high and zone.hi.y > low
            if show and zone.far > 0.0:
                dx = math.max(math.max(zone.lo.x - p.x, p.x - zone.hi.x), 0.0)
                dz = math.max(math.max(zone.lo.z - p.z, p.z - zone.hi.z), 0.0)
                show = dx * dx + dz * dz < zone.far * zone.far
            if show != zone.shown:
                zone.shown = show
                zone.node.visible = show

    # A cop sent by a wave: spaced with the others and counted.
    def register_enemy(self, enemy):
        if self.enemies == None:
            self.enemies = []
        self.enemies.append(enemy)
        self._total = self._total + 1

    def place_player(self):
        # The zones and the storeys' people follow her to the new spot on the next frame.
        self._floors = 0.0
        start = self.get_node("PlayerStart")
        player = self._player
        if player == None:
            return
        checkpoint = self.story.checkpoint if self.story != None else None
        if checkpoint != None:
            player.respawn(checkpoint, self.story.checkpoint_yaw)
        elif start != None:
            player.respawn(start.global_position, start.rotation_degrees.y)

    def show(self, text, seconds):
        self._message = text
        self._message_timer = seconds

    def make_fx(self):
        self._fx = self.get_node("ImpactFx")
        if self._fx == None:
            self._fx = self.create_child("ImpactFx3D")
            self._fx.name = "ImpactFx"
            self._fx.crash_sounds = "Assets/Sounds/explosion/crash_02.ogg,Assets/Sounds/explosion/crash_05.ogg,Assets/Sounds/explosion/crash_08.ogg"

    def explode(self, center, radius, force, damage, kind, source):
        # A blast is heard: sleeping bomb carriers wake and cops turn to it.
        self.noise(center, radius * 2.5)
        if self._fx == None:
            self.make_fx()
        big = kind == "car" or kind == "big"
        if big:
            self.blast_trauma(center, 6.0, 30.0, 0.9)
        else:
            self.blast_trauma(center, 3.0, 12.0, 0.4)
        if big:
            self._fx.blast(center)
            if kind == "car":
                self.boom(center, "Assets/Sounds/explosion/car_" + ["a", "b"][int(math.random(0.0, 1.99))] + ".wav", 12.0)
                self.say("car", 0.8)
            else:
                self.boom(center, "Assets/Sounds/explosion/explosion_large_" + ["A", "B"][int(math.random(0.0, 1.99))] + ".ogg", 12.0)
        elif kind == "medium":
            self._fx.impact(center, 900.0)
            self._fx.flame(center, 8)
            self._fx.smoke(center, 4)
            self.boom(center, "Assets/Sounds/explosion/explosion_medium_" + ["A", "B", "C"][int(math.random(0.0, 2.99))] + ".ogg", 6.0)
        else:
            self._fx.impact(center, 500.0)
            self._fx.smoke(center, 2)
            self.boom(center, "Assets/Sounds/explosion/explosion_small_" + ["A", "B", "C"][int(math.random(0.0, 2.99))] + ".ogg", 3.0)
        player = self._player
        if player != None:
            p = player.global_position
            d = math.sqrt((p.x - center.x) * (p.x - center.x) + (p.y - center.y) * (p.y - center.y) + (p.z - center.z) * (p.z - center.z))
            shake = math.clamp(1.0 - d / (radius * 3.0), 0.0, 1.0)
            player._kick = math.min(player._kick + shake * 4.0, 4.0)
            if big and shake > 0.3:
                self._flash = 0.06 * shake
            if d < radius and player.alive():
                f = 1.0 - d / radius
                dir = self.away(center, p)
                player.take_damage(damage * f, center, dir, force * f, source)
        for enemy in self.get_tree().get_nodes_in_group("enemy"):
            e = enemy.global_position
            d = math.sqrt((e.x - center.x) * (e.x - center.x) + (e.y - center.y) * (e.y - center.y) + (e.z - center.z) * (e.z - center.z))
            if d >= radius or not enemy.alive():
                continue
            f = 1.0 - d / radius
            dir = self.away(center, e)
            if big and d < radius * 0.4:
                enemy.dismember(dir, force * f + 2.0)
            else:
                enemy._kill_kind = "EXPLODED"
                enemy.die(dir, force * f)
        for corpse in self.get_tree().get_nodes_in_group("corpse"):
            c = corpse.global_position
            d = math.sqrt((c.x - center.x) * (c.x - center.x) + (c.z - center.z) * (c.z - center.z))
            if d < radius:
                ragdoll = corpse.get_node("Ragdoll")
                if ragdoll != None and ragdoll.is_active():
                    ragdoll.apply_blast(center, force * (1.0 - d / radius))
        for body in self.get_tree().get_nodes_in_group("breakable"):
            if body == source:
                continue
            item = body.get_node("Breakable")
            if item == None:
                continue
            # A popping screen does not set off the rest of the office.
            if kind == "small" and item.kind == "monitor":
                continue
            b = item.center()
            d = math.sqrt((b.x - center.x) * (b.x - center.x) + (b.y - center.y) * (b.y - center.y) + (b.z - center.z) * (b.z - center.z))
            if d >= radius:
                continue
            dir = self.away(center, b)
            item.hit(b, Vec3(-dir.x, -dir.y, -dir.z), dir, damage * 2.2 * (1.0 - d / radius) + 120.0)
        for bot in self.get_tree().get_nodes_in_group("bot"):
            if bot == source:
                continue
            q = bot.global_position
            if (q.x - center.x) * (q.x - center.x) + (q.y - center.y) * (q.y - center.y) + (q.z - center.z) * (q.z - center.z) < radius * radius:
                bot.blast(center)
        # Barrels and crates: thrown by the blast, and set off (barrels) or smashed (crates) close in.
        for barrel in self.get_tree().get_nodes_in_group("blastable"):
            if barrel == source:
                continue
            q = barrel.global_position
            d = math.sqrt((q.x - center.x) * (q.x - center.x) + (q.y - center.y) * (q.y - center.y) + (q.z - center.z) * (q.z - center.z))
            if d < radius:
                away = self.away(center, q)
                barrel.apply_impulse(Vec3(away.x * force * 40.0 * (1.0 - d / radius), force * 25.0 * (1.0 - d / radius), away.z * force * 40.0 * (1.0 - d / radius)), Vec3(0.0, 0.3, 0.0))
                if d < radius * 0.8:
                    barrel.blast(center)
        for unit in self.get_tree().get_nodes_in_group("unit7"):
            q = unit.global_position
            d = math.sqrt((q.x - center.x) * (q.x - center.x) + (q.y - center.y) * (q.y - center.y) + (q.z - center.z) * (q.z - center.z))
            if d < radius and unit != source:
                unit.take_damage(damage * (1.0 - d / radius), center, self.away(center, q), force, source)
        for spider in self.get_tree().get_nodes_in_group("spider"):
            q = spider.global_position
            if (q.x - center.x) * (q.x - center.x) + (q.y - center.y) * (q.y - center.y) + (q.z - center.z) * (q.z - center.z) < radius * radius * 0.5:
                spider.wreck()

    # Wired men sit at computers far from the start, facing the screen, unless the scene places
    # its own (group "bot").
    def place_bots(self, region, count):
        screens = []
        for body in self.get_tree().get_nodes_in_group("breakable"):
            item = body.get_node("Breakable")
            if item == None or item.kind != "monitor":
                continue
            c = item.center()
            if c.x >= region[0] and c.x <= region[1] and c.y >= region[2] and c.y <= region[3] and c.z >= region[4] and c.z <= region[5]:
                screens.append(c)
        placed = []
        tries = 0
        while len(placed) < count and len(screens) > 0 and tries < 60:
            tries += 1
            c = screens[int(math.random(0.0, len(screens) - 0.01))]
            spot = self.floor_by_screen(c)
            if spot == None:
                continue
            crowded = False
            for other in placed:
                if (other.x - spot.x) * (other.x - spot.x) + (other.z - spot.z) * (other.z - spot.z) < 16.0:
                    crowded = True
            if crowded:
                continue
            bot = self.instantiate("Assets/Prefabs/kamikaze.scene.json")
            if bot == None:
                return
            bot.global_position = Vec3(spot.x, spot.y + 0.9, spot.z)
            yaw = math.degrees(math.atan2(-(c.x - spot.x), -(c.z - spot.z)))
            bot.rotation_degrees = Vec3(0.0, yaw, 0.0)
            bot._yaw = yaw
            placed.append(spot)

    # Walkable floor on the screen's storey (about 1.1 m below it) a step away from the desk.
    def floor_by_screen(self, c):
        floor = c.y - 1.1
        start = int(math.random(0.0, 7.99))
        for k in range(8):
            angle = (start + k) * 0.785
            for r in [0.9, 1.3]:
                spot = self.nav.nearest_point(Vec3(c.x + math.cos(angle) * r, floor, c.z + math.sin(angle) * r))
                if spot != None and math.abs(spot.y - floor) < 0.5:
                    return spot
        return None

    # A wired man blown up: points and a chip, not a human kill.
    def bot_destroyed(self, bot):
        points = 100 + TAG_POINTS["BOT"]
        self._score = self._score + points
        self._feed = [0.0, points, ["BOT"]]
        if self._snd_kill == None:
            self.make_sounds()
        self._snd_kill.stream = "Assets/Sounds/fx/combo_standard.wav"
        self._snd_kill.play()
        self._marker = 0.45
        self._pop = 0.4

    def boom(self, center, sound, peak):
        boom = self.instantiate("Assets/Prefabs/boom.scene.json")
        if boom != None:
            boom.global_position = center
            boom.peak = peak
            boom.get_node("Sound").stream = sound

    def away(self, center, point):
        dx = point.x - center.x
        dy = point.y - center.y
        dz = point.z - center.z
        l = math.max(math.sqrt(dx * dx + dy * dy + dz * dz), 0.1)
        return Vec3(dx / l, math.max(dy / l, 0.25), dz / l)

    def breakable_name(self, name):
        i = name.find("_lm")
        if i >= 0 and i + 3 < len(name) and "0123456789".find(name[i + 3]) >= 0:
            return False
        if name.find("break_") == 0:
            return True
        if (name.find("van_parent") >= 0 or name.find("vlads_car") >= 0) and name.find("__prefab") < 0 and name.find("tires") < 0:
            return True
        if door_name(name):
            return True
        for part in ["WaterMac", "d_water", "WaterBottle", "SodaCan"]:
            if name.find(part) >= 0:
                return True
        return False

    def collect_breakables(self, node):
        if node == None:
            return
        for child in node.get_children():
            if self.breakable_name(child.name) and child.get_node("Mesh") != None and child.get_node("Breakable") == None:
                controller = child.create_child("Node3D", "Assets/Scripts/breakable.py")
                controller.name = "Breakable"
                controller.setup()
            self.collect_breakables(child)

    def make_sounds(self):
        self._snd_kill = self.sound("Assets/Sounds/fx/combo_standard.wav", 0.9)
        self._snd_heavy = self.sound("Assets/Sounds/fx/break_02.wav", 1.0)
        self._snd_swoosh = self.sound("Assets/Sounds/bt/bt_swoosh_01.ogg", 0.9)
        self._snd_score = self.sound("Assets/Sounds/fx/combo_score_whoosh.wav", 0.7)
        self._snd_gore = self.sound("Assets/Sounds/fx/horror_01.wav", 0.9)
        self._snd_tip = self.sound("Assets/Sounds/fx/print_tip.ogg", 0.9)
        self._snd_typing = self.sound("Assets/Sounds/fx/typing.ogg", 0.7)
        self._snd_beep = self.sound("Assets/Sounds/fx/beep.ogg", 0.6)
        self._snd_scan_on = self.sound("Assets/Sounds/fx/scan_on.ogg", 1.0)
        self._snd_scan_off = self.sound("Assets/Sounds/fx/scan_off.ogg", 0.8)
        self._snd_lock = self.sound("Assets/Sounds/fx/lock.ogg", 0.6)
        self._snd_voice = self.sound("Assets/Sounds/fx/beep.ogg", 1.0)
        self._voice = VoiceState(self.sound("Assets/Sounds/fx/beep.ogg", 1.0), 0.0, "", "", 0.0, 0.0, 0.0, False, [])

    # The music: corruption through the orders, nightmare once she knows the truth, the theme at
    # the end; it ducks under the voices and goes with the M key or the speaker icon.
    def make_music(self):
        self._music = self.create_child("AudioStreamPlayer")
        self._music.name = "Music"
        self._music.bus = AUDIO_BUS_MUSIC
        self._music.loop = True
        self._music.volume_linear = 0.0

    def update_music(self, real):
        if self._music == None:
            return
        track = "corruption"
        if self.music_final:
            track = "theme"
        elif self.story != None and self.story._truth_said:
            track = "nightmare"
        if track != self._music_track:
            self._music_track = track
            self._music.stop()
            self._music.stream = "Assets/Sounds/music/" + track + ".ogg"
            self._music_level = 0.0
            if self.music:
                self._music.play()
        if self.music and not self._music.is_playing():
            self._music.play()
        if not self.music and self._music.is_playing() and self._music_level <= 0.0:
            self._music.stop()
        target = 0.0
        if self.music:
            target = self.music_volume * (0.4 if self.talking() else 1.0)
        self._music_level = self._music_level + math.clamp(target - self._music_level, -real * 1.5, real * 1.5)
        self._music.volume_linear = self._music_level

    def toggle_music(self):
        self.music = not self.music
        self.show("MUSIC // " + ("ON" if self.music else "OFF"), 1.2)

    # The speaker icon, bottom right (Assets/Textures/Hud/music_on.png and music_off.png): a click
    # toggles the music while the mouse is free.
    def draw_music_icon(self, width, height):
        if self._img_music_on < 0:
            self._img_music_on = hud_load_image("Assets/Textures/Hud/music_on.png")
            self._img_music_off = hud_load_image("Assets/Textures/Hud/music_off.png")
        size = 32.0
        x = width - size - 14.0
        y = height - size - 14.0
        self._icon_clicked = False
        over = False
        if mouse_get_mode() != MOUSE_MODE_CAPTURED:
            m = mouse_position()
            over = m.x >= x - 6.0 and m.x <= x + size + 6.0 and m.y >= y - 6.0 and m.y <= y + size + 6.0
            if over and mouse_check_button_pressed(MOUSE_LEFT):
                self.toggle_music()
                self._icon_clicked = True
        tint = Color(1.0, 1.0, 1.0, 1.0 if over else 0.7)
        if not self.music:
            tint = Color(1.0, 0.5, 0.45, 1.0 if over else 0.7)
        hud_image(self._img_music_on if self.music else self._img_music_off, x, y, size, size, 0.0, 0.0, 0.0, 1.0, 1.0, tint)

    # Camera shake by trauma: blasts and shots add to it, it decays, and the shake is its square.
    def add_trauma(self, amount):
        self._trauma = math.clamp(self._trauma + amount, 0.0, 1.0)

    def shake_amount(self):
        return self._trauma * self._trauma

    def blast_trauma(self, center, near, far, amount):
        player = self._player
        if player == None:
            return
        p = player.global_position
        d = math.sqrt((p.x - center.x) * (p.x - center.x) + (p.y - center.y) * (p.y - center.y) + (p.z - center.z) * (p.z - center.z))
        if d >= far:
            return
        self.add_trauma(amount * (1.0 if d <= near else 1.0 - (d - near) / (far - near)))

    def sound(self, path, volume):
        player = self.create_child("AudioStreamPlayer")
        player.stream = path
        player.bus = AUDIO_BUS_MUSIC
        player.volume_linear = volume
        return player

    def collect_lights(self):
        self._lights = []
        self._shadow_light = None
        lights = self.get_node("Lights")
        if lights != None:
            for light in lights.get_children():
                light.shadow_enabled = False
                self._lights.append(light)

    # A gunshot or a scream: every cop within reach on the same storey turns towards it.
    def noise(self, position, reach):
        if self._voice != None:
            self._voice.idle = 0.0
        if self.enemies == None:
            return
        for enemy in self.enemies:
            e = enemy.global_position
            if math.abs(e.y - position.y) > 3.0:
                continue
            if (e.x - position.x) * (e.x - position.x) + (e.z - position.z) * (e.z - position.z) < reach * reach:
                enemy.alert_to(self._player.global_position if self._player != None else position)
        # The bomb carriers asleep within reach wake up too. The group is read here, not the list
        # cull_floors keeps: a carrier that blew up since then is already freed.
        for bot in self.get_tree().get_nodes_in_group("bot"):
            e = bot.global_position
            if math.abs(e.y - position.y) < 6.0 and (e.x - position.x) * (e.x - position.x) + (e.z - position.z) * (e.z - position.z) < reach * reach:
                bot.hear(position)

    def enemy_killed(self, enemy):
        self.noise(enemy.global_position, 10.0)
        self.kill_quip(enemy)
        if enemy.civilian:
            self._humans_terminated = self._humans_terminated + 1
            self.show("HUMAN TERMINATED // GOOD", 2.5)
            self._glitch = 0.5
        if self.enemies != None:
            alive = []
            for other in self.enemies:
                if other != enemy:
                    alive.append(other)
            self.enemies = alive
        self._kills = self._kills + 1
        self.stage_kills = self.stage_kills + 1
        player = self._player
        tags = []
        if enemy._kill_kind != "":
            tags.append(enemy._kill_kind)
        if player != None:
            if player._dodge > 0.0:
                tags.append("AIRBORNE")
            elif player._draining:
                tags.append("OVERCLOCK")
        if self._chain_timer > 0.0:
            self._chain = self._chain + 1
        else:
            self._chain = 1
        self._chain_timer = 2.5
        points = 100
        for tag in tags:
            points = points + TAG_POINTS[tag]
        points = points * self._chain
        self._score = self._score + points
        self._feed = [0.0, points, tags]
        if self._snd_kill == None:
            self.make_sounds()
        kill = "Assets/Sounds/fx/combo_standard.wav"
        if self._chain >= 3:
            kill = "Assets/Sounds/fx/combo_perfect.wav"
        elif enemy._kill_kind == "HEADSHOT" or enemy._kill_kind == "DISMEMBERED":
            kill = "Assets/Sounds/fx/combo_painful.wav"
        self._snd_kill.stream = kill
        self._snd_kill.pitch_scale = 1.0 + 0.06 * math.min(self._chain - 1, 6)
        self._snd_kill.play()
        self._snd_score.play()
        self._marker = 0.45
        self._pop = 0.4
        if player != None:
            player.reward(1.0 + 0.5 * len(tags) + 0.25 * (self._chain - 1))
        if enemy._kill_kind == "HEADSHOT" or enemy._kill_kind == "DISMEMBERED":
            self._flash = 0.12
            self._glitch = 0.25
            if enemy._kill_kind == "DISMEMBERED":
                self._snd_heavy.stream = "Assets/Sounds/fx/break_0" + ["2", "4"][int(math.random(0.0, 1.99))] + ".wav"
                self._snd_heavy.play()
                self._snd_gore.stream = "Assets/Sounds/fx/horror_0" + ["1", "3"][int(math.random(0.0, 1.99))] + ".wav"
                self._snd_gore.play()
            self._snd_swoosh.stream = "Assets/Sounds/bt/bt_swoosh_0" + str(int(math.random(1.0, 3.99))) + ".ogg"
            self._snd_swoosh.play()
            if player != None:
                player.kill_slow(0.5)

    # ---- the android's voice

    # A line for the event, now and then (chance): never over another line or NULL//OS reading the
    # orders, not the same line twice running, one every few seconds at most; its subtitle shows.
    def say(self, event, chance):
        if self._voice == None or self.in_intro() or self._voice.cooldown > 0.0:
            return
        if self._voice.player.is_playing() or (self._snd_voice != None and self._snd_voice.is_playing()):
            return
        lines = QUIPS.get(event)
        if lines == None or math.random(0.0, 1.0) > chance:
            return
        pick = lines[int(math.random(0.0, len(lines) - 0.01))]
        if pick == self._voice.last and len(lines) > 1:
            pick = lines[0] if lines[0] != self._voice.last else lines[1]
        self._voice.last = pick
        self._voice.player.stream = "Assets/Sounds/voice/android/" + pick + ".ogg"
        self._voice.player.play()
        self._voice.text = SUBTITLES[pick]
        self._voice.timer = 3.5
        self._voice.cooldown = 5.0
        self._voice.idle = 0.0

    # A story line (Unit-7, Bravura, NULL//OS): waits for the one playing, then plays over any
    # chatter; lines queued together play one after the other.
    def speak(self, line):
        if self._voice != None and not (line in self._voice.queue) and len(self._voice.queue) < 4:
            self._voice.queue.append(line)

    # A line is playing or waiting in the queue (the story waits for a conversation to end).
    def talking(self):
        if self._voice == None:
            return False
        return len(self._voice.queue) > 0 or self._voice.player.is_playing()

    def play_line(self, line):
        folder = "android"
        if line.find("os_") == 0:
            folder = "nullos"
        elif line.find("u7_") == 0:
            folder = "unit7"
        elif line.find("br_") == 0:
            folder = "bravura"
        self._voice.player.stream = "Assets/Sounds/voice/" + folder + "/" + line + ".ogg"
        self._voice.player.play()
        self._voice.text = SUBTITLES[line] if line in SUBTITLES else ""
        self._voice.timer = 4.0
        self._voice.cooldown = 4.0
        self._voice.idle = 0.0

    def kill_quip(self, enemy):
        tag = enemy._kill_kind
        if enemy.civilian:
            self.say("innocent", 1.0)
            self.speak("os_good_0" + str(int(math.random(1.0, 2.99))))
        elif tag == "HEADSHOT":
            self.say("headshot", 0.6)
        elif tag == "DISMEMBERED":
            self.say("dismember", 0.6)
        elif tag == "EXPLODED":
            self.say("exploded", 0.5)
        elif self._chain >= 2:
            self.say("chain", 0.7)
        else:
            self.say("kill", 0.35)

    # The first bomb carrier she meets gets a line (kamikaze.charge).
    def bot_seen(self):
        if self._voice != None and not self._voice.bot_said:
            self._voice.bot_said = True
            self.say("bot", 1.0)

    # Idle chatter when nothing has happened for a while, a word when she is hurt, the subtitle.
    def update_voice(self, real, width, height):
        if self._voice == None:
            return
        busy = self._voice.player.is_playing() or (self._snd_voice != None and self._snd_voice.is_playing())
        if len(self._voice.queue) > 0 and not busy and not self.in_intro():
            self.play_line(self._voice.queue.pop(0))
        self._voice.cooldown = math.max(self._voice.cooldown - real, 0.0)
        self._voice.hurt = math.max(self._voice.hurt - real, 0.0)
        self._voice.idle = self._voice.idle + real
        if self._voice.idle > 28.0:
            self._voice.idle = 0.0
            self.say("idle", 0.9)
        player = self._player
        if player != None and player.alive() and self._voice.hurt <= 0.0 and player._health < player.max_health * 0.3:
            self._voice.hurt = 25.0
            self.say("lowhealth", 1.0)
        if self._voice.timer <= 0.0:
            return
        self._voice.timer = self._voice.timer - real
        alpha = math.clamp(self._voice.timer / 0.4, 0.0, 1.0)
        size = 18.0 * height / 720.0
        w = hud_text_size(self._voice.text, size).x
        x = (width - w) * 0.5
        y = height * 0.8
        hud_rect(x - 12.0, y - 6.0, w + 24.0, size + 12.0, Color(0.0, 0.03, 0.08, 0.6 * alpha))
        hud_text(x, y, self._voice.text, Color(0.55, 0.9, 1.0, alpha), size)

    def make_minimap(self):
        ids = []
        for storey in MINIMAP_STOREYS:
            ids.append(hud_load_image("Assets/Textures/Minimap/floor_" + str(int(-storey)) + ".png"))
        self._minimap = Minimap(ids)

    # The storey a height belongs to (the stairs land on the nearer one).
    def minimap_storey(self, y):
        best = 0
        for i in range(len(MINIMAP_STOREYS)):
            if math.abs(y - MINIMAP_STOREYS[i]) < math.abs(y - MINIMAP_STOREYS[best]):
                best = i
        return best

    # Top right, under the kills: the storey she is on round her, her heading, the objective (a
    # hollow mark with an arrow when it is on another storey) and Unit-7 once free.
    def draw_minimap(self, width, height):
        m = self._minimap
        if m == None or self._player == None or self.briefing():
            return
        p = self._player.global_position
        storey = self.minimap_storey(p.y)
        if m.ids[storey] < 0:
            return
        size = 170.0 * height / 720.0
        x = width - size - 20.0
        y = 100.0 * height / 720.0
        hud_rect(x - 4.0, y - 4.0, size + 8.0, size + 8.0, Color(0.0, 0.03, 0.08, 0.62))
        hud_rect_outline(x - 4.0, y - 4.0, size + 8.0, size + 8.0, Color(0.3, 0.8, 1.0, 0.45), 1.0)
        half = MINIMAP_VIEW * 0.5
        u0 = (p.x - half - MINIMAP_MIN_X) / MINIMAP_SIZE_X
        u1 = (p.x + half - MINIMAP_MIN_X) / MINIMAP_SIZE_X
        v0 = (p.z - half - MINIMAP_MIN_Z) / MINIMAP_SIZE_Z
        v1 = (p.z + half - MINIMAP_MIN_Z) / MINIMAP_SIZE_Z
        hud_image(m.ids[storey], x, y, size, size, 0.0, u0, v0, u1, v1, Color(1.0, 1.0, 1.0, 0.95))
        scale = size / MINIMAP_VIEW
        if self.story != None and self.story._target != None:
            self.minimap_mark(self.lied(self.story._target), p, storey, x, y, size, scale, Color(0.3, 1.0, 0.55, 0.95), True)
        if self.story != None and self.story.unit_free and self.story._unit != None:
            self.minimap_mark(self.story._unit.global_position, p, storey, x, y, size, scale, Color(0.3, 0.8, 1.0, 0.9), False)
        cx = x + size * 0.5
        cy = y + size * 0.5
        yaw = math.radians(self._player._yaw)
        fx = -math.sin(yaw)
        fz = -math.cos(yaw)
        hud_line(cx, cy, cx + fx * 14.0, cy + fz * 14.0, Color(1.0, 1.0, 1.0, 0.9), 2.0)
        hud_circle(cx, cy, 4.0, Color(1.0, 1.0, 1.0, 1.0))
        hud_text(x, y - 18.0, "FLOOR " + str(int(MINIMAP_STOREYS[storey])), Color(0.3, 0.8, 1.0, 0.8), 11.0)
        if self._director != None:
            trust = self._director.trust
            label = "NULL//OS TRUST " + str(int(trust)) + "%"
            color = Color(0.3, 1.0, 0.55, 0.85) if trust > 66.0 else (Color(1.0, 0.8, 0.3, 0.9) if trust > 33.0 else Color(1.0, 0.3, 0.3, 0.95))
            hud_text(x + size - hud_text_size(label, 11.0).x, y - 18.0, label, color, 11.0)

    # Where NULL//OS says the objective is: the truth, or a lie while it corrupts the navigation.
    def lied(self, point):
        if self._director == None or self._director.nav_lie == None:
            return point
        lie = self._director.nav_lie
        return Vec3(point.x + lie.x, point.y, point.z + lie.z)

    # A point of the level on the minimap, kept inside the box; on another storey a hollow mark
    # with the way to go (up or down).
    def minimap_mark(self, point, p, storey, x, y, size, scale, color, objective):
        mx = x + size * 0.5 + (point.x - p.x) * scale
        mz = y + size * 0.5 + (point.z - p.z) * scale
        mx = math.clamp(mx, x + 6.0, x + size - 6.0)
        mz = math.clamp(mz, y + 6.0, y + size - 6.0)
        same = self.minimap_storey(point.y) == storey
        if same:
            if objective:
                hud_rect(mx - 4.0, mz - 4.0, 8.0, 8.0, color)
            else:
                hud_circle(mx, mz, 4.0, color)
            return
        hud_rect_outline(mx - 5.0, mz - 5.0, 10.0, 10.0, color, 2.0)
        up = point.y > p.y
        hud_line(mx, mz + (3.0 if up else -3.0), mx, mz + (-11.0 if up else 11.0), color, 2.0)
        hud_line(mx - 4.0, mz + (-7.0 if up else 7.0), mx, mz + (-11.0 if up else 11.0), color, 2.0)
        hud_line(mx + 4.0, mz + (-7.0 if up else 7.0), mx, mz + (-11.0 if up else 11.0), color, 2.0)

    def pick_shadow_light(self):
        if self._lights == None:
            self.collect_lights()
        if self._player == None:
            self._player = self.get_node("Player")
        player = self._player
        if player == None or len(self._lights) == 0:
            return
        p = player.global_position
        best = None
        best_d = 1000000.0
        for light in self._lights:
            l = light.global_position
            d = (l.x - p.x) * (l.x - p.x) + (l.z - p.z) * (l.z - p.z) + (l.y - p.y) * (l.y - p.y) * 4.0
            if d < best_d:
                best_d = d
                best = light
        if best == self._shadow_light:
            return
        if self._shadow_light != None:
            self._shadow_light.shadow_enabled = False
            self.trigger_scan()
        best.shadow_enabled = True
        self._shadow_light = best

    def player_died(self):
        self.show("SEGMENTATION FAULT", 3.0)
        self._glitch = 0.6
        self._chain_timer = 0.0
        self._respawn = 3.0

    # The quality preset: the 3D is drawn at its lines (16:9, never more than the screen has) and
    # fitted to the screen by render_aspect, with its MSAA and screen-space AA; the HUD stays
    # sharp. Full screen on a 1440p monitor is four times the pixels of 720p, too much for an
    # APU. With debug_keys F10 cycles the fit and F11 the preset.
    def fit_render(self):
        h = hud_height()
        if h <= 0.0:
            return
        q = QUALITY[int(math.clamp(self.quality, 0, len(QUALITY) - 1))]
        lines = math.min(q[1], h)
        key = str(lines) + "/" + str(self.render_aspect) + "/" + q[0]
        if key == self._render_applied:
            return
        self._render_applied = key
        root = self.get_tree().get_root()
        if root != None:
            root.scaling_3d_size = Vec2(math.floor(lines * 16.0 / 9.0 + 0.5), lines)
            root.scaling_3d_aspect = self.render_aspect
            root.msaa_3d = q[2]
            root.screen_space_aa = q[3]

    def render_keys(self):
        if keyboard_check_pressed(KEY_F10):
            self.render_aspect = (self.render_aspect + 1) % 4
            self.show("RENDER // FIT " + RENDER_FITS[self.render_aspect], 1.5)
        if keyboard_check_pressed(KEY_F11):
            self.quality = (self.quality + 1) % len(QUALITY)
            q = QUALITY[self.quality]
            self.show("RENDER // " + q[0] + "  " + str(int(math.min(q[1], hud_height()))) + "p  MSAA " + str(q[2]) + "  AA " + str(q[3]), 2.0)

    def _process(self, delta):
        if self.debug_keys:
            self.render_keys()
        self.fit_render()
        real = delta
        if time_scale() > 0.0:
            real = delta / time_scale()
        self._clock = self._clock + real
        self.rebuild_spider(real)
        self._floors = self._floors - real
        if self._floors <= 0.0:
            self._floors = 0.25
            self.cull_floors()
        if self._respawn > 0.0:
            self._respawn = self._respawn - delta
            if self._respawn <= 0.0:
                self.place_player()
        self._chain_timer = math.max(self._chain_timer - real, 0.0)
        self._trauma = math.max(self._trauma - real * self.trauma_decay, 0.0)
        if keyboard_check_pressed(KEY_M):
            self.toggle_music()
        self.update_music(real)
        self._light_timer = self._light_timer - real
        if self._light_timer <= 0.0:
            self._light_timer = 0.25
            self.pick_shadow_light()
        width = hud_width()
        height = hud_height()
        if self.draw_intro(real, width, height):
            return
        self._scan_cd = math.max(self._scan_cd - real, 0.0)
        if keyboard_check_pressed(KEY_V):
            self.trigger_scan()
        if self.hud:
            self.draw_scan(real, width, height)
            self.draw_screen_fx(real, width, height)
            self.draw_marker(real, width, height)
            self.draw_feed(real, width, height)
            self.draw_chain(width, height)
            self.draw_counter(real, width)
            self.draw_minimap(width, height)
            self.draw_briefing(real, width, height)
            self.draw_objectives(width)
            self.draw_music_icon(width, height)
        self.update_voice(real, width, height)
        if self._message_timer > 0.0 and self.hud:
            self._message_timer = self._message_timer - real
            size = hud_text_size(self._message, 52.0)
            x = (width - size.x) * 0.5
            y = height * 0.18
            alpha = math.clamp(self._message_timer, 0.0, 1.0)
            hud_text(x - 3.0, y, self._message, Color(1.0, 0.2, 0.2, 0.7 * alpha), 52.0)
            hud_text(x + 3.0, y, self._message, Color(0.3, 0.8, 1.0, 0.7 * alpha), 52.0)
            hud_text(x, y, self._message, Color(1.0, 1.0, 1.0, alpha), 52.0)

    def trigger_scan(self):
        player = self._player
        if self._scan_cd > 0.0 or self.briefing() or player == None or not player.alive():
            return
        if self._snd_scan_on == None:
            self.make_sounds()
        self._scan = 0.0
        self._scan_cd = self.scan_cooldown
        self._targets = []
        p = player.global_position
        digits = "0123456789ABCDEF"
        for enemy in self.get_tree().get_nodes_in_group("enemy"):
            e = enemy.global_position
            d = math.sqrt((e.x - p.x) * (e.x - p.x) + (e.y - p.y) * (e.y - p.y) + (e.z - p.z) * (e.z - p.z))
            if d > self.scan_range:
                continue
            code = "0x"
            for i in range(4):
                code = code + digits[int(math.random(0.0, 15.99))]
            self._targets.append([enemy, code, -1.0])
        self._snd_scan_on.play()

    def draw_scan(self, real, width, height):
        if self._scan < 0.0 or self._targets == None:
            return
        self._scan = self._scan + real
        sweep = 0.7
        if self._scan > 5.5:
            self._scan = -1.0
            self._snd_scan_off.play()
            return
        fade = math.clamp((5.5 - self._scan) / 1.0, 0.0, 1.0)
        red = Color(1.0, 0.1, 0.08, fade)
        if self._scan < 1.2:
            tint = 1.0 - self._scan / 1.2
            hud_rect(0.0, 0.0, width, height, Color(0.9, 0.0, 0.0, 0.2 * tint))
            gx = 0.0
            while gx < width:
                hud_rect(gx, 0.0, 1.0, height, Color(1.0, 0.2, 0.2, 0.07 * tint))
                gx = gx + 64.0
            gy = 0.0
            while gy < height:
                hud_rect(0.0, gy, width, 1.0, Color(1.0, 0.2, 0.2, 0.07 * tint))
                gy = gy + 64.0
        line_y = height * self._scan / sweep
        if self._scan < sweep:
            for k in range(8):
                hud_rect(0.0, line_y - k * 10.0, width, 10.0, Color(1.0, 0.1, 0.1, 0.12 * (1.0 - k / 8.0)))
            hud_rect(0.0, line_y, width, 3.0, Color(1.0, 0.45, 0.4, 0.95))
        camera = self._player.camera()
        found = 0
        for target in self._targets:
            enemy = target[0]
            if not enemy.alive():
                continue
            found = found + 1
            e = enemy.global_position
            if camera.is_position_behind(e):
                continue
            head = camera.unproject_position(Vec3(e.x, e.y + 1.0, e.z))
            feet = camera.unproject_position(Vec3(e.x, e.y - 0.9, e.z))
            if target[2] < 0.0:
                if self._scan >= sweep or line_y >= head.y:
                    target[2] = self._scan
                    self._snd_lock.play()
                else:
                    continue
            age = self._scan - target[2]
            grow = 1.0 + math.clamp(1.0 - age / 0.25, 0.0, 1.0) * 0.9
            h = math.max(feet.y - head.y, 24.0) * grow
            w = h * 0.5
            mx = (head.x + feet.x) * 0.5
            my = (head.y + feet.y) * 0.5
            x0 = mx - w * 0.5
            y0 = my - h * 0.5
            c = w * 0.28
            for corner in [[x0, y0, 1.0, 1.0], [x0 + w, y0, -1.0, 1.0], [x0, y0 + h, 1.0, -1.0], [x0 + w, y0 + h, -1.0, -1.0]]:
                hud_line(corner[0], corner[1], corner[0] + c * corner[2], corner[1], red, 2.0)
                hud_line(corner[0], corner[1], corner[0], corner[1] + c * corner[3], red, 2.0)
            hud_line(mx - 4.0, my, mx + 4.0, my, red, 1.0)
            hud_line(mx, my - 4.0, mx, my + 4.0, red, 1.0)
            p = self._player.global_position
            dist = math.sqrt((e.x - p.x) * (e.x - p.x) + (e.y - p.y) * (e.y - p.y) + (e.z - p.z) * (e.z - p.z))
            threat = "HIGH" if dist < 12.0 else "LOW"
            lines = ["HUMAN", "ID " + target[1], "DIST " + str(int(dist * 10.0) / 10.0) + "M", "THREAT " + threat]
            chars = int(age * 70.0)
            lx = x0 + w + 8.0
            ly = y0
            for i in range(4):
                text = lines[i]
                if chars < len(text):
                    text = text[0:int(math.max(chars, 0))]
                chars = chars - len(lines[i])
                if len(text) > 0:
                    size = 14.0 if i == 0 else 11.0
                    hud_text(lx, ly, text, Color(1.0, 0.25 if i == 0 else 0.45, 0.2 if i == 0 else 0.4, fade), size)
                ly = ly + (17.0 if i == 0 else 13.0)
        title = "THREAT SCAN // " + str(found) + " HUMANS DETECTED"
        size = hud_text_size(title, 15.0)
        hud_rect((width - size.x) * 0.5 - 10.0, 92.0, size.x + 20.0, 24.0, Color(0.15, 0.0, 0.0, 0.6 * fade))
        hud_text((width - size.x) * 0.5, 95.0, title, red, 15.0)

    # ---- intro

    def make_intro(self):
        self._intro_img = hud_load_image("Assets/Textures/Intro/intro_bg.jpg")
        self._intro_track = self.sound("Assets/Sounds/voice/intro.ogg", 1.0)

    def in_intro(self):
        return self._intro_state < 3 or self._wake > 0.0

    # cut: skipped, the narration stops; at the end the ambience tail of the track fades by itself.
    def end_intro(self, cut=True):
        if cut and self._intro_track != None:
            self._intro_track.stop()
        if self._snd_typing != None:
            self._snd_typing.stop()
        self._intro_state = 2
        self._intro = 0.0

    # Draws the intro over everything and returns True while it runs (the HUD waits for it).
    def draw_intro(self, real, width, height):
        if self._intro_state >= 3:
            if self._wake > 0.0:
                self._wake = self._wake - real
                return True
            return False
        if self._intro_track == None:
            self.make_intro()
        self._intro = self._intro + real
        if self._intro_state < 2:
            if self._intro_t == 0.0:
                self._intro_track.play()
            # The track's own position when there is one, our clock otherwise (no audio device).
            self._intro_t = self._intro_t + real
            pos = self._intro_track.get_playback_position()
            if pos > 0.0:
                self._intro_t = pos
            if keyboard_check_pressed(KEY_ENTER):
                self.end_intro(True)
                self._snd_swoosh.play()
        line = -1
        for i in range(len(INTRO)):
            if self._intro_t >= INTRO[i][0]:
                line = i
        if line != self._intro_line and self._intro_state < 2:
            self._intro_line = line
            self._snd_typing.play()
        typed = 0
        if self._intro_line >= 0:
            text = INTRO[self._intro_line][2]
            typed = int((self._intro_t - INTRO[self._intro_line][0]) * 26.0)
            if typed >= len(text):
                typed = len(text)
                self._snd_typing.stop()
        if self._intro_state == 0 and self._intro_t >= INTRO_TITLE:
            self._intro_state = 1
            self._intro = 0.0
            self._snd_beep.play()
        elif self._intro_state == 1 and self._intro > 3.0:
            self.end_intro(False)
            self._snd_swoosh.play()
        alpha = 1.0
        if self._intro_state == 2:
            alpha = 1.0 - math.clamp(self._intro / 0.8, 0.0, 1.0)
            if alpha <= 0.0:
                self.start_night()
                return False
        s = height / 720.0
        # The picture covers the screen whatever its shape, cropped at the sides or at the top.
        u = 0.0
        v = 0.0
        aspect = width / height
        if aspect < 16.0 / 9.0:
            u = (1.0 - aspect / (16.0 / 9.0)) * 0.5
        else:
            v = (1.0 - (16.0 / 9.0) / aspect) * 0.5
        flicker = 0.92 + 0.08 * math.sin(self._clock * 23.0) * math.sin(self._clock * 3.1)
        hud_rect(0.0, 0.0, width, height, Color(0.0, 0.0, 0.0, alpha))
        hud_image(self._intro_img, 0.0, 0.0, width, height, 0.0, u, v, 1.0 - u, 1.0 - v, Color(flicker, flicker, flicker, alpha))
        band = (self._clock * 140.0 * s) % (height + 80.0) - 40.0
        hud_rect(0.0, band, width, 40.0 * s, Color(0.4, 0.85, 1.0, 0.04 * alpha))
        cyan = Color(0.45, 0.9, 1.0, 0.8 * alpha)
        red = Color(1.0, 0.25, 0.3, 0.85 * alpha)
        x = width * 0.08
        hud_text(x, height * 0.1, "NYPD // PRECINCT 12 // 03:17 AM", cyan, 15.0 * s)
        hud_text(x, height * 0.1 + 22.0 * s, "NULL//OS  CIVIC ENFORCEMENT INTELLIGENCE  // ONLINE", red, 15.0 * s)
        y = height * 0.26
        size = 25.0 * s
        first = int(math.max(self._intro_line - INTRO_SHOWN + 1, 0))
        for i in range(first, len(INTRO)):
            if i > self._intro_line or self._intro_line < 0:
                break
            text = INTRO[i][2]
            current = i == self._intro_line
            if current:
                text = text[0:typed]
            radio = INTRO[i][1].find("unit7") == 0
            prefix = "> [UNKNOWN SIGNAL] " if radio else "> "
            if radio:
                color = Color(1.0, 0.7, 0.2, alpha) if current else Color(1.0, 0.7, 0.2, 0.5 * alpha)
            else:
                color = Color(1.0, 1.0, 1.0, alpha) if current else Color(0.55, 0.85, 1.0, 0.5 * alpha)
            # The oldest line on screen fades as it scrolls away.
            if i == first and self._intro_line - first == INTRO_SHOWN - 1 and first > 0:
                color = Color(color.r, color.g, color.b, color.a * 0.5)
            if len(text) > 0:
                split = 1.4 * s if current else 0.0
                if radio and current:
                    split = 4.0 * s * math.abs(math.sin(self._clock * 17.0))
                self.glitch_text(x, y, prefix + text, size, color, split)
            if current and typed < len(INTRO[i][2]) and int(self._clock * 6.0) % 2 == 0:
                hud_rect(x + 4.0 * s + hud_text_size(prefix + text, size).x, y + 3.0 * s, 11.0 * s, size - 4.0 * s, color)
            y = y + 46.0 * s
        if self._intro_state >= 1:
            title = "NULL PROTOCOL"
            tsize = 76.0 * s
            t = math.clamp(self._intro / 0.4, 0.0, 1.0) if self._intro_state == 1 else 1.0
            tx = (width - hud_text_size(title, tsize).x) * 0.5
            self.glitch_text(tx, height * 0.74, title, tsize, Color(1.0, 1.0, 1.0, t * alpha), 4.0 * s * (1.0 - t) + 2.0 * s)
        if self._intro_state < 2 and int(self._clock * 2.0) % 2 == 0:
            hint = "[ENTER] SKIP"
            hs = 15.0 * s
            hud_text(width - x - hud_text_size(hint, hs).x, height * 0.92, hint, Color(1.0, 1.0, 1.0, 0.7 * alpha), hs)
        return True

    def briefing(self):
        return self._brief_state < 3

    def brief_text(self):
        lines = [self.mission]
        for order in self._orders:
            lines.append("> " + order)
        return lines

    def draw_briefing(self, real, width, height):
        if self._orders == None:
            self._orders = self.orders.split("|")
        if self._snd_tip == None:
            self.make_sounds()
        if self._brief_state >= 3:
            return
        self._brief = self._brief + real
        if self._brief_state == 0:
            if self._brief < self.brief_delay:
                return
            self._brief_state = 1
            self._brief = 0.0
            self._snd_tip.play()
            # NULL//OS reads the orders.
            if self.brief_voice != "":
                self._snd_voice.stream = self.brief_voice
                self._snd_voice.play()
        lines = self.brief_text()
        total = 0
        for line in lines:
            total = total + len(line)
        typed = int(math.max(self._brief - 0.35, 0.0) * 45.0)
        if self._brief_state == 1:
            if typed > 0 and typed < total and not self._typing:
                self._typing = True
                self._snd_typing.play()
            if typed >= total:
                if self._typing:
                    self._typing = False
                    self._snd_typing.stop()
                    self._snd_beep.play()
                self._brief_close = self._brief_close + real
                if (self._brief_close > self.brief_hold and not self._snd_voice.is_playing()) or keyboard_check_pressed(KEY_ENTER):
                    self._brief_state = 2
                    self._brief = 0.0
                    self._snd_beep.play()
        w = 600.0
        h = 74.0 + 30.0 * len(lines)
        target = height * 0.3
        if self._brief_state == 1:
            t = math.clamp(self._brief / 0.35, 0.0, 1.0)
            ease = 1.0 + 2.7 * (t - 1.0) * (t - 1.0) * (t - 1.0) + 1.7 * (t - 1.0) * (t - 1.0)
            y = -h + (target + h) * ease
        else:
            t = math.clamp(self._brief / 0.3, 0.0, 1.0)
            y = target - (target + h) * t * t
            if t >= 1.0:
                self._brief_state = 3
                return
        x = (width - w) * 0.5
        hud_rect(x, y, w, h, Color(0.02, 0.05, 0.1, 0.85))
        hud_rect_outline(x, y, w, h, Color(0.35, 0.8, 1.0, 0.9), 2.0)
        hud_rect(x, y, w, 4.0, Color(1.0, 0.2, 0.25, 0.95))
        hud_text(x + 18.0, y + 12.0, "INCOMING ORDERS", Color(1.0, 0.3, 0.3, 0.9), 13.0)
        hud_text(x + w - 18.0 - hud_text_size("NULL//OS", 13.0).x, y + 12.0, "NULL//OS", Color(0.4, 0.85, 1.0, 0.6), 13.0)
        left = typed
        ly = y + 38.0
        i = 0
        for line in lines:
            shown = line
            if left < len(line):
                shown = line[0:int(math.max(left, 0))]
            left = left - len(line)
            size = 26.0 if i == 0 else 20.0
            color = Color(1.0, 1.0, 1.0, 1.0) if i == 0 else Color(0.45, 0.9, 1.0, 1.0)
            if len(shown) > 0:
                self.glitch_text(x + 22.0, ly, shown, size, color, 1.2 if i == 0 else 0.0)
                if left < 0 and int(self._clock * 6.0) % 2 == 0:
                    cx = x + 24.0 + hud_text_size(shown, size).x
                    hud_rect(cx, ly + 3.0, 10.0, size - 4.0, color)
            ly = ly + (36.0 if i == 0 else 30.0)
            i = i + 1
        if typed >= total and int(self._clock * 2.5) % 2 == 0:
            hint = "[ENTER] ACKNOWLEDGE"
            hud_text(x + w - 18.0 - hud_text_size(hint, 13.0).x, y + h - 24.0, hint, Color(1.0, 1.0, 1.0, 0.7), 13.0)

    def draw_objectives(self, width):
        if self._brief_state < 3 or self._orders == None:
            return
        if self.story != None and self.story.draw_objectives():
            return
        y = 34.0
        for order in self._orders:
            text = "> " + order
            done = False
            if order.find("HUMANS") >= 0:
                text = text + "  " + str(self._kills) + "/" + str(self._total)
                done = self._kills >= self._total
            color = Color(0.45, 0.9, 1.0, 0.9)
            if done:
                color = Color(0.4, 1.0, 0.5, 0.9)
            hud_rect(12.0, y - 2.0, 3.0, 18.0, color)
            hud_text(22.0, y, text, color, 15.0)
            y = y + 22.0

    def glitch_text(self, x, y, text, size, color, split):
        hud_text(x - split, y, text, Color(1.0, 0.15, 0.2, 0.75 * color.a), size)
        hud_text(x + split, y, text, Color(0.2, 0.85, 1.0, 0.75 * color.a), size)
        hud_text(x, y, text, color, size)

    def draw_counter(self, real, width):
        self._pop = math.max(self._pop - real, 0.0)
        self._shown_score = self._shown_score + (self._score - self._shown_score) * math.min(real * 10.0, 1.0)
        pop = self._pop / 0.4
        w = 230.0
        x = width - w - 16.0
        y = 12.0
        hud_rect(x, y, w, 64.0, Color(0.02, 0.06, 0.12, 0.6))
        hud_rect(x, y, 4.0, 64.0, Color(1.0, 0.2, 0.25, 0.95))
        if pop > 0.0:
            hud_rect(x, y, w, 64.0, Color(1.0, 1.0, 1.0, pop * 0.35))
        kills = str(self._kills)
        size = 40.0 + pop * 18.0
        self.glitch_text(x + 16.0, y + 8.0 - pop * 6.0, kills, size, Color(1.0, 1.0, 1.0, 1.0), 1.5 + pop * 6.0)
        kx = x + 22.0 + hud_text_size(kills, 40.0).x
        hud_text(kx, y + 12.0, "KILLS", Color(1.0, 0.35, 0.35, 0.95), 14.0)
        # The night has no fixed head count (the waves keep coming): the total, and this directive's.
        hud_text(kx, y + 28.0, "+" + str(self.stage_kills) + " THIS DIRECTIVE", Color(1.0, 1.0, 1.0, 0.5), 11.0)
        score = str(int(self._shown_score + 0.5))
        hud_text(x + w - 14.0 - hud_text_size(score, 24.0).x, y + 8.0, score, Color(0.45, 0.9, 1.0, 1.0), 24.0)
        hud_text(x + w - 14.0 - hud_text_size("SCORE", 11.0).x, y + 34.0, "SCORE", Color(0.45, 0.9, 1.0, 0.6), 11.0)
        # The bar is the chain window: the next kill inside it keeps the combo going.
        ratio = math.clamp(self._chain_timer / 2.5, 0.0, 1.0) if self._chain > 0 else 0.0
        hud_rect(x + 12.0, y + 54.0, w - 24.0, 3.0, Color(1.0, 1.0, 1.0, 0.15))
        hud_rect(x + 12.0, y + 54.0, (w - 24.0) * ratio, 3.0, Color(1.0, 0.25, 0.3, 0.95))

    def draw_feed(self, real, width, height):
        if self._feed == None:
            return
        self._feed[0] = self._feed[0] + real
        age = self._feed[0]
        if age > 1.8:
            self._feed = None
            return
        fade = math.clamp((1.8 - age) / 0.4, 0.0, 1.0)
        slam = math.clamp(1.0 - age / 0.14, 0.0, 1.0)
        x = width * 0.5 + 64.0
        y = height * 0.5 - 52.0
        size = 34.0 + slam * 34.0
        shake = math.clamp(1.0 - age / 0.3, 0.0, 1.0) * 4.0
        sx = x + math.random(-shake, shake)
        sy = y - slam * 14.0 + math.random(-shake, shake)
        self.glitch_text(sx, sy, "KILL", size, Color(1.0, 1.0, 1.0, fade), 2.0 + slam * 10.0)
        points = "+" + str(self._feed[1])
        px = x + hud_text_size("KILL", 34.0).x + 12.0
        hud_text(px, y + 8.0, points, Color(1.0, 0.85, 0.3, fade), 22.0)
        ty = y + 42.0
        i = 0
        for tag in self._feed[2]:
            local = age - 0.08 - i * 0.07
            i = i + 1
            if local < 0.0:
                continue
            slide = math.clamp(1.0 - local / 0.12, 0.0, 1.0)
            tw = hud_text_size(tag, 15.0).x + 18.0
            c = TAG_COLORS[tag]
            tx = x + slide * 60.0
            hud_rect(tx, ty, tw, 22.0, Color(c.r, c.g, c.b, c.a * fade))
            hud_rect(tx, ty, 3.0, 22.0, Color(1.0, 1.0, 1.0, 0.9 * fade))
            hud_text(tx + 10.0, ty + 3.0, tag, Color(1.0, 1.0, 1.0, fade), 15.0)
            if local < 0.07:
                hud_rect(tx, ty, tw, 22.0, Color(1.0, 1.0, 1.0, (0.07 - local) / 0.07 * 0.8))
            ty = ty + 26.0

    def draw_chain(self, width, height):
        if self._chain < 2 or self._chain_timer <= 0.0:
            return
        x = width * 0.5 - 150.0
        y = height * 0.5 - 52.0
        pulse = 1.0 + 0.08 * math.sin(self._clock * 18.0)
        text = "x" + str(self._chain)
        size = 40.0 * pulse
        tw = hud_text_size(text, size).x
        self.glitch_text(x + 70.0 - tw, y, text, size, Color(1.0, 0.85, 0.3, 1.0), 2.0)
        hud_text(x + 6.0, y + 46.0, "CHAIN", Color(1.0, 0.85, 0.3, 0.8), 13.0)
        hud_rect(x + 6.0, y + 64.0, 64.0, 3.0, Color(1.0, 1.0, 1.0, 0.15))
        hud_rect(x + 6.0, y + 64.0, 64.0 * self._chain_timer / 2.5, 3.0, Color(1.0, 0.85, 0.3, 0.95))

    def draw_screen_fx(self, real, width, height):
        if self._flash > 0.0:
            hud_rect(0.0, 0.0, width, height, Color(1.0, 0.95, 0.9, self._flash * 2.5))
            self._flash = math.max(self._flash - real, 0.0)
        if self._glitch > 0.0:
            for i in range(7):
                y = math.random(0.0, height)
                h = math.random(2.0, 10.0)
                shift = math.random(-30.0, 30.0)
                hud_rect(shift, y, width, h, Color(0.3, 0.9, 1.0, 0.18))
                hud_rect(-shift, y + h, width, h * 0.5, Color(1.0, 0.15, 0.15, 0.18))
            self._glitch = math.max(self._glitch - real, 0.0)

    def draw_marker(self, real, width, height):
        if self._marker <= 0.0:
            return
        t = self._marker / 0.45
        cx = width * 0.5
        cy = height * 0.5
        inner = 10.0 + (1.0 - t) * 6.0
        outer = inner + 14.0 + t * 16.0
        color = Color(1.0, 0.15, 0.1, math.clamp(t * 1.6, 0.0, 1.0))
        for sx in [-1.0, 1.0]:
            for sy in [-1.0, 1.0]:
                hud_line(cx + sx * inner, cy + sy * inner, cx + sx * outer, cy + sy * outer, color, 4.0)
        self._marker = math.max(self._marker - real, 0.0)
